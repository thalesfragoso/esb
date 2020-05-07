//! Rust implementation of Nordic's Enhanced ShockBurst

#![no_std]
// james sorry
#![allow(dead_code)]

pub mod peripherals;

pub(crate) mod app;
pub(crate) mod buffer;
pub(crate) mod irq;
pub(crate) mod payload;

// Export crate relevant items
pub use crate::{
    app::EsbApp,
    buffer::EsbBuffer,
    irq::{EsbIrq, State},
    payload::{EsbHeader, EsbHeaderBuilder},
};

use core::default::Default;
// Export dependency items necessary to create a backing structure
pub use bbqueue::{consts, ArrayLength, BBBuffer, ConstBBBuffer};

const RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS: u16 = 48;
const RETRANSMIT_DELAY_US_OFFSET: u16 = 62;
const RETRANSMIT_DELAY: u16 = 250;
const MAXIMUM_TRANSMIT_ATTEMPTS: u8 = 3;
const ENABLED_PIPES: u8 = 0xFF;

// TODO: Document Ramp-up time
#[cfg(feature = "51")]
pub(crate) const RAMP_UP_TIME: u16 = 140;

// This is only true if we enable the fast ramp-up time, which we do
#[cfg(not(feature = "51"))]
pub(crate) const RAMP_UP_TIME: u16 = 40;

/// Crate-wide error type
#[derive(Debug, PartialEq, Eq)]
pub enum Error {
    // TODO(AJM): Do we still need these?
    // EOF,
    // InProgress,
    /// Unable to add item to the incoming queue, queue is full. After issuing this error,
    /// [EsbIrq](struct.EsbIrq.html) will be put in the Idle state
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

    /// Internal Error, if you encounter this error, please report it, it is a bug
    InternalError,

    /// [EsbIrq](struct.EsbIrq.html) reached the maximum number of attempts to send a packet that
    /// requested for an acknowledgement, the packet will be removed from the queue and
    /// [EsbIrq](struct.EsbIrq.html) will try to send the next one
    MaximumAttempts,
}

/// Protocol configuration
#[derive(Copy, Clone)]
pub struct Config {
    /// Number of microseconds to wait for an acknowledgement before timing out
    wait_for_ack_timeout: u16,
    /// Delay, in microseconds, between retransmissions when the radio does not receive an acknowledgement
    retransmit_delay: u16,
    /// Maximum number of transmit attempts when an acknowledgement is not received
    maximum_transmit_attempts: u8,
    /// A bit mask representing the pipes that the radio must listen while receiving, the LSb is
    /// pipe zero
    enabled_pipes: u8,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            wait_for_ack_timeout: RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS,
            retransmit_delay: RETRANSMIT_DELAY,
            maximum_transmit_attempts: MAXIMUM_TRANSMIT_ATTEMPTS,
            enabled_pipes: ENABLED_PIPES,
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
/// | Ack Timeout                         | 48 us         |
/// | Retransmit Delay                    | 250 us        |
/// | Maximum number of transmit attempts | 3             |
/// | Enabled Pipes                       | 0xFF          |
///
pub struct ConfigBuilder(Config);

impl Default for ConfigBuilder {
    fn default() -> Self {
        Self(Config::default())
    }
}

impl ConfigBuilder {
    /// Sets `wait_for_ack_timeout` field, must be bigger than 43 us
    pub fn wait_for_ack_timeout(mut self, micros: u16) -> Self {
        self.0.wait_for_ack_timeout = micros;
        self
    }

    // TODO: document 62
    /// Sets `retransmit_delay` field, must be bigger than `wait_for_ack_timeout` field plus 62 and
    /// bigger than the ramp-up time (130us for nRF51 and 40us for nRF52)
    pub fn retransmit_delay(mut self, micros: u16) -> Self {
        self.0.retransmit_delay = micros;
        self
    }

    /// Sets `maximum_transmit_attempts` field
    pub fn maximum_transmit_attempts(mut self, n: u8) -> Self {
        self.0.maximum_transmit_attempts = n;
        self
    }

    /// Sets `enabled_pipes` field
    pub fn enabled_pipes(mut self, enabled_pipes: u8) -> Self {
        self.0.enabled_pipes = enabled_pipes;
        self
    }

    pub fn check(self) -> Result<Config, Error> {
        let bad_ack_timeout = self.0.wait_for_ack_timeout < 44;
        let bad_retransmit_delay = self.0.retransmit_delay
            <= self.0.wait_for_ack_timeout + RETRANSMIT_DELAY_US_OFFSET
            || self.0.retransmit_delay <= RAMP_UP_TIME;

        if bad_ack_timeout || bad_retransmit_delay {
            Err(Error::InvalidParameters)
        } else {
            Ok(self.0)
        }
    }
}
