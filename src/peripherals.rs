#[cfg(feature = "51")]
use nrf51 as pac;

#[cfg(feature = "52810")]
use nrf52810_pac as pac;

#[cfg(feature = "52832")]
use nrf52832_pac as pac;

#[cfg(feature = "52840")]
use nrf52840_pac as pac;

use core::{
    mem::MaybeUninit,
    ops::{Deref, DerefMut},
    ptr,
    sync::atomic::{compiler_fence, Ordering},
};
use generic_array::ArrayLength;

use crate::packet::{Addresses, Buffer, Payload};
use pac::RADIO;

pub struct ESBRadio<N: ArrayLength<u8> + 'static> {
    radio: RADIO,
    tx_buf: &'static mut Buffer<N>,
    rx_buf: &'static mut Buffer<N>,
}

const CRC_INIT: u32 = 0x0000_FFFF;
const CRC_POLY: u32 = 0x0000_11021;

#[inline]
fn bytewise_bit_swap(value: u32) -> u32 {
    value.reverse_bits().swap_bytes()
}

#[inline]
fn address_conversion(value: u32) -> u32 {
    value.reverse_bits()
}

impl<N: ArrayLength<u8> + 'static> ESBRadio<N> {
    pub(crate) fn new(
        radio: RADIO,
        tx_buf: &'static mut Buffer<N>,
        rx_buf: &'static mut Buffer<N>,
    ) -> Self {
        ESBRadio {
            radio,
            tx_buf,
            rx_buf,
        }
    }

    pub(crate) fn init(&mut self, max_payload: u8, addresses: &Addresses) {
        self.radio.mode.write(|w| w.mode().nrf_2mbit());
        let len_bits = if max_payload <= 32 { 6 } else { 8 };
        // Convert addresses to remain compatible with nRF24L devices
        let base0 = address_conversion(u32::from_le_bytes(addresses.base0));
        let base1 = address_conversion(u32::from_le_bytes(addresses.base1));
        let prefix0 = bytewise_bit_swap(u32::from_le_bytes(addresses.prefixes0));
        let prefix1 = bytewise_bit_swap(u32::from_le_bytes(addresses.prefixes1));

        self.radio.shorts.write(|w| {
            w.ready_start()
                .enabled()
                .end_disable()
                .enabled()
                .address_rssistart()
                .enabled()
                .disabled_rssistop()
                .enabled()
        });

        // TODO: configurable tx_power
        unsafe {
            self.radio
                .pcnf0
                .write(|w| w.lflen().bits(len_bits).s1len().bits(3));

            self.radio.pcnf1.write(|w| {
                w.maxlen()
                    .bits(max_payload)
                    // 4-Byte Base Address + 1-Byte Address Prefix
                    .balen()
                    .bits(4)
                    // Nordic's code doesn't use whitening, maybe enable in the future ?
                    //.whiteen()
                    //.set_bit()
                    .statlen()
                    .bits(0)
                    .endian()
                    .big()
            });

            self.radio
                .crcinit
                .write(|w| w.crcinit().bits(CRC_INIT & 0x00FF_FFFF));

            self.radio
                .crcpoly
                .write(|w| w.crcpoly().bits(CRC_POLY & 0x00FF_FFFF));

            self.radio.base0.write(|w| w.bits(base0));
            self.radio.base1.write(|w| w.bits(base1));

            self.radio.prefix0.write(|w| w.bits(prefix0));
            self.radio.prefix1.write(|w| w.bits(prefix1));

            // NOTE(unsafe) `rf_channel` was checked to be between 0 and 100 during the creation of
            // the `Adresses` object
            self.radio
                .frequency
                .write(|w| w.frequency().bits(addresses.rf_channel));
        }
    }

    // Transmit a packet and setup interrupts
    pub(crate) fn transmit<M: ArrayLength<MaybeUninit<u8>>>(
        &mut self,
        payload: Payload<M>,
        ack: bool,
    ) {
        let tx_slice = self.tx_buf.deref_mut();
        tx_slice[0] = payload.len;
        tx_slice[1] = payload.pid_and_no_ack;
        let tx_payload_slice = &mut tx_slice[2..];

        // NOTE(unsafe) Safe since tx_slice length is guaranteed to be (`payload.buf` length + 2)
        unsafe {
            ptr::copy_nonoverlapping(
                payload.deref().as_ptr(),
                tx_payload_slice.as_mut_ptr(),
                payload.len as usize,
            )
        }

        if ack {
            // Go to RX mode after the transmission
            self.radio.shorts.modify(|_, w| w.disabled_rxen().enabled());
            // Enable `disabled` interrupt to know when the radio finished transmission and `ready`
            // to know when it started listening for the ack
            self.radio
                .intenset
                .write(|w| w.disabled().set_bit().ready().set_bit());
        } else {
            self.radio
                .intenset
                .write(|w| w.disabled().set_bit().ready().clear_bit());
        }
        unsafe {
            // NOTE(unsafe) Pipe fits in 3 bits
            self.radio
                .txaddress
                .write(|w| w.txaddress().bits(payload.pipe));
            // NOTE(unsafe) Pipe only goes from 0 through 7
            self.radio.rxaddresses.write(|w| w.bits(1 << payload.pipe));

            self.radio
                .packetptr
                .write(|w| w.bits(tx_slice.as_ptr() as u32));
            self.radio.events_address.write(|w| w.bits(0));
            self.radio.events_payload.write(|w| w.bits(0));
            self.radio.events_disabled.write(|w| w.bits(0));

            // "Preceding reads and writes cannot be moved past subsequent writes."
            compiler_fence(Ordering::Release);

            self.radio.tasks_txen.write(|w| w.bits(1));
        }
    }
}

mod sealed {
    pub trait Sealed {}
}

/// Trait implemented for the nRF timer peripherals.
pub trait ESBTimer: sealed::Sealed {
    /// Initialize the timer with a 1MHz rate.
    fn init(&mut self);

    /// Configures the timer's interrupt to fire after a given time in micro seconds.
    fn set_interrupt(&mut self, micros: u8);

    /// Acknowledges this timer's interrupt.
    fn clear_interrupt(&mut self);

    /// Returns whether a timer interrupt is currently pending. This must be called by the interrupt
    /// handler to avoid spurious timer events.
    fn is_pending(&self) -> bool;
}

macro_rules! impl_timer {
    ( $($ty:ty),+ ) => {
        $(
            impl ESBTimer for $ty {
                fn init(&mut self) {
                    self.bitmode.write(|w| w.bitmode()._32bit());
                    // 2^4 = 16
                    // 16 MHz / 16 = 1 MHz = µs resolution
                    self.prescaler.write(|w| unsafe { w.prescaler().bits(4) });
                    self.tasks_clear.write(|w| unsafe { w.bits(1) });
                    self.tasks_start.write(|w| unsafe { w.bits(1) });
                }

                fn set_interrupt(&mut self, micros: u8) {
                    self.cc[0].write(|w| unsafe { w.bits(micros as u32) });
                    self.events_compare[0].reset();
                    self.intenset.write(|w| w.compare0().set());
                }

                fn clear_interrupt(&mut self) {
                    self.intenclr.write(|w| w.compare0().clear());
                    self.events_compare[0].reset();
                }

                fn is_pending(&self) -> bool {
                    self.events_compare[0].read().bits() == 1u32
                }
            }

            impl sealed::Sealed for $ty {}
        )+
    };
}

#[cfg(not(feature = "51"))]
impl_timer!(pac::TIMER0, pac::TIMER1, pac::TIMER2);

#[cfg(feature = "51")]
impl_timer!(pac::TIMER0);
