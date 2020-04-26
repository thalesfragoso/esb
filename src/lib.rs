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
use packet::{Addresses, Buffer, Payload};
use peripherals::{ESBRadio, ESBTimer};

const SRAM_LOWER: usize = 0x2000_0000;
const SRAM_UPPER: usize = 0x3000_0000;

// This should me made configurable later
const RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS: u8 = 48;
const RETRANSMIT_DELAY_US_OFFSET: u8 = 62;
const RETRANSMIT_DELAY: u8 = 250;
const MAXIMUM_TRANSMIT_ATTEMPTS: u8 = 3;

fn slice_in_ram(slice: &[u8]) -> bool {
    let ptr = slice.as_ptr() as usize;
    ptr >= SRAM_LOWER && (ptr + slice.len()) < SRAM_UPPER
}

pub enum Error {
    EOF,
    InProgress,
    QueueEmpty,
}

#[derive(PartialEq)]
enum State {
    Idle,
    TransmitterTxNoAck,
    TransmitterTx,
    TransmitterWaitAck,
    ReceiverNoAck,
    Receiver,
}

pub struct ESB<'a, M, S, T>
where
    M: ArrayLength<MaybeUninit<u8>> + Unsigned + Add<U2>,
    S: ArrayLength<Payload<M>> + Unsigned,
    <M as Add<U2>>::Output: ArrayLength<u8> + Unsigned + 'static,
    T: ESBTimer,
{
    tx_consumer: Consumer<'a, Payload<M>, S, u8, SingleCore>,
    rx_producer: Producer<'a, Payload<M>, S, u8, SingleCore>,
    radio: ESBRadio<<M as Add<U2>>::Output>,
    timer: T,
    current_state: State,
    addresses: Addresses,
    attempts: u8,
}

impl<'a, M, S, T> ESB<'a, M, S, T>
where
    M: ArrayLength<MaybeUninit<u8>> + Unsigned + Add<U2>,
    S: ArrayLength<Payload<M>> + Unsigned,
    <M as Add<U2>>::Output: ArrayLength<u8> + Unsigned + 'static,
    T: ESBTimer,
{
    pub fn new(
        tx_consumer: Consumer<'a, Payload<M>, S, u8, SingleCore>,
        rx_producer: Producer<'a, Payload<M>, S, u8, SingleCore>,
        tx_buf: &'static mut Buffer<<M as Add<U2>>::Output>,
        rx_buf: &'static mut Buffer<<M as Add<U2>>::Output>,
        radio: RADIO,
        timer: T,
        addresses: Addresses,
    ) -> Self {
        assert!(slice_in_ram(tx_buf.inner.as_slice()) && slice_in_ram(rx_buf.inner.as_slice()));
        let radio = ESBRadio::new(radio, tx_buf, rx_buf);
        let mut esb = Self {
            tx_consumer,
            rx_producer,
            radio,
            timer,
            current_state: State::Idle,
            addresses,
            attempts: 0,
        };
        esb.radio.init(M::U8, &esb.addresses);
        esb.timer.init();
        esb
    }

    pub fn start_tx(&mut self) -> Result<(), Error> {
        if self.current_state != State::Idle {
            return Err(Error::InProgress);
        }
        if !self.tx_consumer.ready() {
            return Err(Error::QueueEmpty);
        }
        let payload = self.tx_consumer.dequeue().unwrap();
        // "active-low"
        let ack = payload.pid_and_no_ack & 0x01 == 0x01;
        self.radio.transmit(payload, ack);
        self.attempts += 1;
        if ack {
            self.current_state = State::TransmitterWaitAck;
        } else {
            self.current_state = State::TransmitterTxNoAck;
        }
        Ok(())
    }
}
