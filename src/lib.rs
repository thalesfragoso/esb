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
pub use crate::{app::EsbApp, buffer::EsbBuffer, irq::EsbIrq, payload::EsbHeader};

// Export dependency items necessary to create a backing structure
pub use bbqueue::{consts, ArrayLength, BBBuffer, ConstBBBuffer};

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

/// Crate-wide error type
pub enum Error {
    // TODO(AJM): Do we still need these?
    // EOF,
    // InProgress,
    /// Unable to add item to the queue, queue is full
    QueueFull,

    /// Unable to pop item from the queue, queue is empty
    QueueEmpty,

    /// Unable to split to producer/consumer halves, the
    /// buffer has already been split
    AlreadySplit,

    /// Values out of range
    InvalidParameters,
}
