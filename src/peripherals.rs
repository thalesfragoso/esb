#[cfg(feature = "51")]
use nrf51 as pac;

#[cfg(feature = "52810")]
use nrf52810_pac as pac;

#[cfg(feature = "52832")]
use nrf52832_pac as pac;

#[cfg(feature = "52840")]
use nrf52840_pac as pac;

use bbqueue::ArrayLength;
use core::sync::atomic::{compiler_fence, Ordering};

use crate::{
    app::Addresses,
    payload::{PayloadR, PayloadW},
};
pub(crate) use pac::{Interrupt, NVIC, RADIO};

const CRC_INIT: u32 = 0x0000_FFFF;
const CRC_POLY: u32 = 0x0001_1021;

#[inline]
fn bytewise_bit_swap(value: u32) -> u32 {
    value.reverse_bits().swap_bytes()
}

#[inline]
fn address_conversion(value: u32) -> u32 {
    value.reverse_bits()
}

pub struct ESBRadio<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    radio: RADIO,
    tx_grant: Option<PayloadR<OutgoingLen>>,
    rx_grant: Option<PayloadW<IncomingLen>>,
}

impl<OutgoingLen, IncomingLen> ESBRadio<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    pub(crate) fn new(radio: RADIO) -> Self {
        ESBRadio {
            radio,
            tx_grant: None,
            rx_grant: None,
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

        // Enable fast ramp-up if the hardware supports it
        #[cfg(not(feature = "51"))]
        self.radio.modecnf0.modify(|_, w| w.ru().fast());

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

    // Clears the End event to not retrigger the interrupt
    #[inline]
    pub(crate) fn clear_disabled_event(&mut self) {
        self.radio.events_disabled.write(|w| unsafe { w.bits(0) });
    }

    // Clears the End event to not retrigger the interrupt
    #[inline]
    pub(crate) fn clear_end_event(&mut self) {
        self.radio.events_end.write(|w| unsafe { w.bits(0) });
    }

    // Clears the Ready event to not retrigger the interrupt
    #[inline]
    pub(crate) fn clear_ready_event(&mut self) {
        self.radio.events_ready.write(|w| unsafe { w.bits(0) });
    }

    // Disables Ready interrupt
    #[inline]
    pub(crate) fn disable_ready_interrupt(&mut self) {
        self.radio.intenclr.write(|w| w.ready().set_bit());
    }

    // TODO: Change to the bbqueue's Grants
    // Transmit a packet and setup interrupts
    pub(crate) fn transmit(&mut self, payload: PayloadR<OutgoingLen>, ack: bool) {
        if ack {
            // Go to RX mode after the transmission
            self.radio.shorts.modify(|_, w| w.disabled_rxen().enabled());
            // Enable `disabled` interrupt to know when the radio finished transmission and `ready`
            // to know when it started listening for the ack
            self.radio
                .intenset
                .write(|w| w.disabled().set_bit().ready().set_bit());
        } else {
            self.radio.intenset.write(|w| w.disabled().set_bit());
        }
        unsafe {
            // NOTE(unsafe) Pipe fits in 3 bits
            self.radio
                .txaddress
                .write(|w| w.txaddress().bits(payload.pipe()));
            // NOTE(unsafe) Pipe only goes from 0 through 7
            self.radio
                .rxaddresses
                .write(|w| w.bits(1 << payload.pipe()));

            self.radio
                .packetptr
                .write(|w| w.bits(payload.dma_pointer() as u32));
            self.radio.events_address.write(|w| w.bits(0));
            self.clear_disabled_event();
            self.clear_ready_event();
            self.clear_end_event();
            //self.radio.events_payload.write(|w| w.bits(0)); do we need this ? Probably not

            // "Preceding reads and writes cannot be moved past subsequent writes."
            compiler_fence(Ordering::Release);

            self.radio.tasks_txen.write(|w| w.bits(1));
        }
        self.tx_grant = Some(payload);
    }

    // TODO: Change to bbqueue's Grant
    // Must be called after the end of TX if the user requested for an ack
    pub(crate) fn prepare_for_ack(&mut self, mut rx_buf: PayloadW<IncomingLen>) {
        // We need a compiler fence here because the DMA will automatically start listening for
        // packets after the ramp-up is completed
        // "Preceding reads and writes cannot be moved past subsequent writes."
        compiler_fence(Ordering::Release);
        self.radio
            .packetptr
            .write(|w| unsafe { w.bits(rx_buf.dma_pointer() as u32) });
        self.rx_grant = Some(rx_buf);
        // this already fired
        self.radio
            .shorts
            .modify(|_, w| w.disabled_rxen().disabled());
    }

    // TODO: Change to the bbqueue's Grants
    // Returns true if the ack was received successfully
    pub(crate) fn check_ack(&mut self) -> bool {
        let ret = self.radio.crcstatus.read().crcstatus().is_crcok();
        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::Acquire);
        // TODO: commit the grant if everything is ok
        ret
    }

    // Disables the radio and the `radio disabled` interrupt
    pub(crate) fn stop(&mut self) {
        self.radio.intenclr.write(|w| w.disabled().set_bit());
        self.radio.tasks_disable.write(|w| unsafe { w.bits(1) });

        // Wait for the disable event to kick in, to make sure that the `task_disable` write won't
        // trigger an interrupt
        while self.radio.events_disabled.read().bits() == 0 {}
        self.clear_disabled_event();

        // "Subsequent reads and writes cannot be moved ahead of preceding reads."
        compiler_fence(Ordering::Acquire);
    }
}

mod sealed {
    pub trait Sealed {}
}

/// Trait implemented for the nRF timer peripherals.
pub trait ESBTimer: sealed::Sealed {
    /// Initialize the timer with a 1MHz rate.
    fn init(&mut self);

    /// Configures the timer's interrupt used for the retransmit, to fire after a given time in
    /// micro seconds.
    fn set_interrupt_retransmit(&mut self, micros: u16);

    /// Acknowledges the retransmit interrupt.
    fn clear_interrupt_retransmit(&mut self);

    /// Returns whether the retransmit interrupt is currently pending.
    fn is_retransmit_pending(&self) -> bool;

    /// Configures the timer's interrupt used for the acknowledge timeout, to fire after a given
    /// time in micro seconds.
    fn set_interrupt_ack(&mut self, micros: u16);

    /// Acknowledges the ack timeout interrupt.
    fn clear_interrupt_ack(&mut self);

    /// Returns whether the ack timeout interrupt is currently pending.
    fn is_ack_pending(&self) -> bool;

    /// Stops the timer.
    fn stop(&mut self);
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
                }

                // CC[0] will be used for the retransmit timeout and CC[1] will be used for the ack
                // timeout

                fn set_interrupt_retransmit(&mut self, micros: u16) {
                    self.cc[0].write(|w| unsafe { w.bits(micros as u32) });
                    self.events_compare[0].reset();
                    self.intenset.write(|w| w.compare0().set());

                    // Clears and starts the counter
                    self.tasks_clear.write(|w| unsafe { w.bits(1) });
                    self.tasks_start.write(|w| unsafe { w.bits(1) });
                }

                fn clear_interrupt_retransmit(&mut self) {
                    self.intenclr.write(|w| w.compare0().clear());
                    self.events_compare[0].reset();

                    self.stop();
                }

                #[inline]
                fn is_retransmit_pending(&self) -> bool {
                    self.events_compare[0].read().bits() == 1u32
                }

                fn set_interrupt_ack(&mut self, micros: u16) {
                    // get current counter
                    self.tasks_capture[1].write(|w| unsafe { w.bits(1) });
                    let current_counter = self.cc[1].read().bits();

                    self.cc[1].write(|w| unsafe { w.bits(current_counter + micros as u32) });
                    self.events_compare[1].reset();
                    self.intenset.write(|w| w.compare1().set());
                }

                fn clear_interrupt_ack(&mut self) {
                    self.intenclr.write(|w| w.compare1().clear());
                    self.events_compare[1].reset();
                }

                #[inline]
                fn is_ack_pending(&self) -> bool {
                    self.events_compare[1].read().bits() == 1u32
                }

                #[inline]
                fn stop(&mut self) {
                    self.tasks_stop.write(|w| unsafe { w.bits(1) });
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
