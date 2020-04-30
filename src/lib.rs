//! Rust implementation of Nordic's Enhanced ShockBurst

#![no_std]
// james sorry
#![allow(dead_code)]

pub mod peripherals;

pub(crate) mod app;
pub(crate) mod buffer;
pub(crate) mod irq;
pub(crate) mod payload;

// This should be made configurable later
const RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS: u16 = 48;
const RETRANSMIT_DELAY_US_OFFSET: u16 = 62;
const RETRANSMIT_DELAY: u16 = 250;
const MAXIMUM_TRANSMIT_ATTEMPTS: u8 = 3;

#[cfg(feature = "51")]
const RAMP_UP_TIME: u8 = 140;

// This is only true if we enable the fast ramp-up time, which we do
#[cfg(not(feature = "51"))]
const RAMP_UP_TIME: u8 = 40;

pub enum Error {
    EOF,
    InProgress,
    QueueFull,
    QueueEmpty,
    AlreadySplit,
    InvalidParameters,
}

#[derive(PartialEq)]
pub enum State {
    Idle,
    /// The radio is preparing for reception (in PTX)
    RampUpRx,
    /// The radio is preparing for transmission.
    RampUpTx,
    RampUpTxNoAck,
    /// ESB on PTX state transmitting.
    TransmitterTx,
    TransmitterTxNoAck,
    /// ESB on PTX state waiting for ack.
    TransmitterWaitAck,
    /// ESB in the PRX state listening for packets
    Receiver,
}
