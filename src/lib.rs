//! Rust implementation of Nordic's Enhanced ShockBurst

#![no_std]

#[cfg(feature = "51")]
use nrf51::RADIO;

#[cfg(feature = "52810")]
use nrf52810_pac::RADIO;

#[cfg(feature = "52832")]
use nrf52832_pac::RADIO;

#[cfg(feature = "52840")]
use nrf52840_pac::RADIO;

use core::{mem::MaybeUninit, ops::Add};
use generic_array::{
    typenum::{consts::U2, Unsigned},
    ArrayLength,
};
use heapless::spsc::{Consumer, Producer, SingleCore};

pub mod packet;
pub mod peripherals;
use packet::{Addresses, Payload};
use peripherals::{ESBRadio, ESBTimer};
pub(crate) mod james;

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
    QueueEmpty,
    AlreadySplit,
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
