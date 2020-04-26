#[cfg(feature = "51")]
use nrf51 as pac;

#[cfg(feature = "52810")]
use nrf52810_pac as pac;

#[cfg(feature = "52832")]
use nrf52832_pac as pac;

#[cfg(feature = "52840")]
use nrf52840_pac as pac;

use crate::packet::Addresses;
use pac::RADIO;

pub struct ESBRadio {
    inner: RADIO,
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

impl ESBRadio {
    pub fn new(radio: RADIO) -> Self {
        ESBRadio { inner: radio }
    }

    pub(crate) fn init(&mut self, max_payload: u8, addresses: &Addresses) {
        self.inner.mode.write(|w| w.mode().nrf_2mbit());
        let len_bits = if max_payload <= 32 { 6 } else { 8 };
        // Convert addresses to remain compatible with nRF24L devices
        let base0 = address_conversion(u32::from_le_bytes(addresses.base0));
        let base1 = address_conversion(u32::from_le_bytes(addresses.base1));
        let prefix0 = bytewise_bit_swap(u32::from_le_bytes(addresses.prefixes0));
        let prefix1 = bytewise_bit_swap(u32::from_le_bytes(addresses.prefixes1));

        unsafe {
            self.inner
                .pcnf0
                .write(|w| w.lflen().bits(len_bits).s1len().bits(3));

            self.inner.pcnf1.write(|w| {
                w.maxlen()
                    .bits(max_payload)
                    // 4-Byte Base Address + 1-Byte Address Prefix
                    .balen()
                    .bits(4)
                    // Nordic's code doesn't use whitening, maybe enable in the future
                    //.whiteen()
                    //.set_bit()
                    .statlen()
                    .bits(0)
                    .endian()
                    .big()
            });

            self.inner
                .crcinit
                .write(|w| w.crcinit().bits(CRC_INIT & 0x00FF_FFFF));

            self.inner
                .crcpoly
                .write(|w| w.crcpoly().bits(CRC_POLY & 0x00FF_FFFF));

            self.inner.base0.write(|w| w.bits(base0));
            self.inner.base1.write(|w| w.bits(base1));

            self.inner.prefix0.write(|w| w.bits(prefix0));
            self.inner.prefix1.write(|w| w.bits(prefix1));

            // NOTE(unsafe) `rf_channel` was checked to be between 0 and 100 during the creation of
            // the `Adresses` object
            self.inner
                .frequency
                .write(|w| w.frequency().bits(addresses.rf_channel));
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
