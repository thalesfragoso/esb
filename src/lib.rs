//! Rust implementation of Nordic's Enhanced ShockBurst

#![no_std]

use core::{mem::MaybeUninit, ops::Add};
use generic_array::{
    typenum::{consts::U2, Unsigned},
    ArrayLength,
};
use heapless::spsc::{Consumer, Producer, SingleCore};

pub mod packet;
pub mod peripherals;
use packet::{Addresses, Buffer, Payload, Pcf};
use peripherals::{ESBRadio, ESBTimer};

pub const SRAM_LOWER: usize = 0x2000_0000;
pub const SRAM_UPPER: usize = 0x3000_0000;

fn slice_in_ram(slice: &[u8]) -> bool {
    let ptr = slice.as_ptr() as usize;
    ptr >= SRAM_LOWER && (ptr + slice.len()) < SRAM_UPPER
}

const RX_WAIT_FOR_ACK_TIMEOUT_US_2MBPS: u8 = 48;
const RETRANSMIT_DELAY_US_OFFSET: u8 = 62;

pub enum Error {
    EOF,
}

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
    tx_buf: &'static Buffer<<M as Add<U2>>::Output>,
    rx_buf: &'static Buffer<<M as Add<U2>>::Output>,
    radio: ESBRadio,
    timer: T,
    current_state: State,
    addresses: Addresses,
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
        tx_buf: &'static Buffer<<M as Add<U2>>::Output>,
        rx_buf: &'static Buffer<<M as Add<U2>>::Output>,
        radio: ESBRadio,
        timer: T,
        addresses: Addresses,
    ) -> Self {
        assert!(slice_in_ram(tx_buf.inner.as_slice()) && slice_in_ram(rx_buf.inner.as_slice()));
        let mut esb = Self {
            tx_consumer,
            rx_producer,
            tx_buf,
            rx_buf,
            radio,
            timer,
            current_state: State::Idle,
            addresses,
        };
        esb.radio.init(M::U8, &esb.addresses);
        esb.timer.init();
        esb
    }
}
