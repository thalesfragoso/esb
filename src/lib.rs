//! Rust implementation of Nordic's Enhanced ShockBurst communication protocol
//!
//! This crate implements the Enhanced ShockBurst protocol with dynamic payload size up to 252 bytes
//! and acknowledgement support.
//!
//! The communication is performed by two devices in different roles, one acting as the
//! Primary Transmitter (PTX) and a second one acting as the Primary Receiver (PRX). The
//! transaction is always started by the PTX, and bi-directional communication is achieved via
//! acknowledgement packets, where the PRX can embed a payload.
//!
//! This crate makes use of [`bbqueue`](https://docs.rs/bbqueue) to handle buffering of packets and
//! to be able to achieve a zero-copy implementation. For timing requirements, the payload that must
//! be sent together with an acknowledgement must be pre-buffered. When a packet that demands an
//! acknowledgement is received in PRX mode the driver will try to dequeue a payload from the
//! transmit queue to be sent, an acknowledgement with a zero sized payload will be sent if the
//! transmit queue is empty.
//!
//! # Timing Requirements
//!
//! For better communication stability, both the radio and timer interrupts must be top priority,
//! and the driver's methods should be called at the beginning of the interrupt handler. In
//! the current implementation, the data rate is fixed at 2Mbps.
//!
//! There are three configurable options that directly affect the timing of the communication:
//!
//! - Wait for acknowledgement timeout (us) - Default: 120 microseconds.
//!     - It is used in PTX mode while sending a packet that requested for an acknowledgement. It
//!       must be bigger than the [Ramp-up](#ramp-up) time.
//!
//! - Retransmit delay offset (us) - Default: 500 microseconds.
//!     - The delay between the end of a transmission and the start of a retransmission when an
//!       acknowledgement was expected but not received. It must be bigger than the
//!       `acknowledgement timeout` plus a constant offset of 62 microseconds.
//!
//! - Number of retransmit attempts - Default: 3 attempts.
//!     - The number of times the driver will retransmit a packet that requires an acknowledgement.
//!       After all the attempts are carried out, the driver will drop the packet and proceed to
//!       transmit the next one in the queue.
//!
//! # Supported devices and crate features
//!
//! | Device   | Feature |
//! | :---     | :---    |
//! | nRF51822 | 51      |
//! | nRF52810 | 52810   |
//! | nRF52832 | 52832   |
//! | nRF52840 | 52840   |
//!
//! Other devices might be compatible with this implementation, however, at this point, the only
//! tested devices are the ones in the table above.
//!
//! # Ramp-up
//!
//! The radio's hardware requires a time before the start or reception of a transmission. This time
//! is 140 microseconds in the nRF5 devices. However, nRF52 devices have a Fast Ramp-up feature,
//! where this time is reduced to 40 microseconds. This feature can be enabled by using the
//! `fast-ru` feature of this crate. Care must be taken when using the Fast Ramp-up while
//! communicating with devices that do not support it, the timing configuration must take this case
//! into account.
//!
//! # In-queue packet representation
//!
//! This crate uses some bytes of queue space to pass information between the user and the driver.
//! The user must take this into account when choosing the size of the
//! [EsbBuffer](buffer/struct.EsbBuffer.html). Moreover, the characteristics of the underlying
//! BipBuffer must be considered, for more information refer to [bbqueue docs](https://docs.rs/bbqueue).
//!
//! | Used by bbqueue framed    | SW USE                         |               ACTUAL DMA PART                                      |
//! | :---                      | :---                           | :---                                                               |
//! | frame_size - 1 to 2 bytes | rssi - 1 byte \| pipe - 1 byte | length - 1 byte \| pid_no_ack - 1 byte \| payload - 1 to 252 bytes |
//!
//! The maximum in-queue packet size is 258 bytes (with a 252 bytes payload).
//!
//! # Compatibility with nRF24L01+
//!
//! This implementation is only compatible with nRF24L01+ devices when using a
//! [configuration](struct.Config.html) with a maximum packet size no bigger than 32 bytes
//! (inclusive). That is required because the nRF24L01+ only supports payloads up to that size and
//! uses a 6-bits effective payload length that must be configured in the nRF5 radio.
//!
//! # Examples
//!
//! Usage examples can be found at the [demos repository](https://github.com/thalesfragoso/esb-demos).
//!

#![no_std]

pub mod app;
pub mod buffer;
pub mod irq;
pub mod payload;
pub mod peripherals;

// Export crate relevant items
pub use crate::{
    app::{Addresses, EsbApp},
    buffer::EsbBuffer,
    irq::{EsbIrq, IrqTimer},
    payload::{EsbHeader, EsbHeaderBuilder},
};

use core::default::Default;
// Export dependency items necessary to create a backing structure
pub use bbqueue::BBBuffer;

// TODO: Figure it out good values
const RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS: u16 = 120;
const RETRANSMIT_DELAY_US_OFFSET: u16 = 62;
const RETRANSMIT_DELAY: u16 = 500;
const MAXIMUM_TRANSMIT_ATTEMPTS: u8 = 3;
const ENABLED_PIPES: u8 = 0xFF;

#[cfg(not(feature = "fast-ru"))]
pub(crate) const RAMP_UP_TIME: u16 = 140;

// This is only true if we enable the fast ramp-up time, which we do
#[cfg(feature = "fast-ru")]
pub(crate) const RAMP_UP_TIME: u16 = 40;

/// Crate-wide error type
#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    /// Unable to add item to the incoming queue, queue is full. After issuing this error,
    /// [EsbIrq](irq/struct.EsbIrq.html) will be put in the Idle state
    IncomingQueueFull,

    /// Unable to add item to the outgoing queue, queue is full
    OutgoingQueueFull,

    /// Grant already in progress
    GrantInProgress,

    /// Unable to pop item from the queue, queue is empty
    QueueEmpty,

    /// Unable to split to producer/consumer halves, the
    /// buffer has already been split
    AlreadySplit,

    /// Values out of range
    InvalidParameters,

    // The requested packet was larger than the configured max payload size
    MaximumPacketExceeded,

    /// Internal Error, if you encounter this error, please report it, it is a bug
    InternalError,

    /// [EsbIrq](irq/struct.EsbIrq.html) reached the maximum number of attempts to send a packet
    /// that requested for an acknowledgement, the packet will be removed from the queue and
    /// [EsbIrq](irq/struct.EsbIrq.html) will try to send the next one
    MaximumAttempts,
}

/// Tx Power
pub type TxPower = peripherals::TXPOWER_A;

/// Protocol configuration
#[derive(Copy, Clone)]
pub struct Config {
    /// Number of microseconds to wait for an acknowledgement before timing out
    wait_for_ack_timeout: u16,
    /// Delay, in microseconds, between retransmissions when the radio does not receive an
    /// acknowledgement
    retransmit_delay: u16,
    /// Maximum number of transmit attempts when an acknowledgement is not received
    maximum_transmit_attempts: u8,
    /// A bit mask representing the pipes that the radio must listen while receiving, the LSb is
    /// pipe zero
    enabled_pipes: u8,
    /// Tx Power
    tx_power: TxPower,
    /// Maximum payload size in bytes that the driver will send or receive.
    ///
    /// This allows for a more efficient usage of the receiver queue and makes this driver
    /// compatible with nRF24L01+ modules when this size is 32 bytes or less
    maximum_payload_size: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            wait_for_ack_timeout: RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS,
            retransmit_delay: RETRANSMIT_DELAY,
            maximum_transmit_attempts: MAXIMUM_TRANSMIT_ATTEMPTS,
            enabled_pipes: ENABLED_PIPES,
            tx_power: TxPower::_0DBM,
            maximum_payload_size: 252,
        }
    }
}

/// A builder for an `Config` structure
///
/// The builder is converted into an `Config` by calling the
/// `check()` method.
///
/// ## Example
///
/// ```rust
/// use esb::ConfigBuilder;
///
/// let config_result = ConfigBuilder::default()
///     .wait_for_ack_timeout(50)
///     .retransmit_delay(240)
///     .maximum_transmit_attempts(4)
///     .enabled_pipes(0x01)
///     .check();
///
/// assert!(config_result.is_ok());
/// ```
///
/// ## Default Config Contents
///
/// By default, the following settings will be used:
///
/// | Field                               | Default Value |
/// | :---                                | :---          |
/// | Ack Timeout                         | 120 us        |
/// | Retransmit Delay                    | 500 us        |
/// | Maximum number of transmit attempts | 3             |
/// | Enabled Pipes                       | 0xFF          |
/// | Tx Power                            | 0dBm          |
/// | Maximum payload size                | 252 bytes     |
///
pub struct ConfigBuilder(Config);

impl Default for ConfigBuilder {
    fn default() -> Self {
        Self(Config::default())
    }
}

impl ConfigBuilder {
    /// Sets number of microseconds to wait for an acknowledgement before timing out
    pub fn wait_for_ack_timeout(mut self, micros: u16) -> Self {
        self.0.wait_for_ack_timeout = micros;
        self
    }

    // TODO: document 62
    /// Sets retransmit delay, must be bigger than `wait_for_ack_timeout` field plus 62 and bigger
    /// than the ramp-up time (140us without fast-ru and 40us with fast-ru)
    pub fn retransmit_delay(mut self, micros: u16) -> Self {
        self.0.retransmit_delay = micros;
        self
    }

    /// Sets maximum number of transmit attempts
    pub fn maximum_transmit_attempts(mut self, n: u8) -> Self {
        self.0.maximum_transmit_attempts = n;
        self
    }

    /// Sets enabled pipes for receiving
    pub fn enabled_pipes(mut self, enabled_pipes: u8) -> Self {
        self.0.enabled_pipes = enabled_pipes;
        self
    }

    /// Sets the tx power
    pub fn tx_power(mut self, tx_power: TxPower) -> Self {
        self.0.tx_power = tx_power;
        self
    }

    /// Sets the maximum payload size
    pub fn max_payload_size(mut self, payload_size: u8) -> Self {
        self.0.maximum_payload_size = payload_size;
        self
    }

    pub fn check(self) -> Result<Config, Error> {
        let bad_ack_timeout = self.0.wait_for_ack_timeout < 44;
        let bad_retransmit_delay = self.0.retransmit_delay
            <= self.0.wait_for_ack_timeout + RETRANSMIT_DELAY_US_OFFSET
            || self.0.retransmit_delay <= RAMP_UP_TIME;
        let bad_size = self.0.maximum_payload_size > 252;

        if bad_ack_timeout || bad_retransmit_delay || bad_size {
            Err(Error::InvalidParameters)
        } else {
            Ok(self.0)
        }
    }
}
