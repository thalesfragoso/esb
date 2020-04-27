use crate::packet::Addresses;
use crate::peripherals::{ESBRadio, ESBTimer, RADIO};
use crate::Error;
use crate::State;
use bbqueue::{
    framed::{FrameConsumer, FrameProducer, FrameGrantW},
    ArrayLength, BBBuffer,
};
use core::result::Result;

// I'm gunna be const!
pub struct EsbBuffer<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    pub app_to_radio_buf: BBBuffer<OutgoingLen>,
    pub radio_to_app_buf: BBBuffer<IncomingLen>,
    // Probably some more stuff?
}

impl<OutgoingLen, IncomingLen> EsbBuffer<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    fn try_split<Timer: ESBTimer>(
        &'static self, // TODO(AJM): This seems odd
        timer: Timer,
        radio: RADIO,
        addresses: Addresses,
        max_payload_bytes: u8,
    ) -> Result<
        (
            EsbApp<OutgoingLen, IncomingLen>,
            EsbIrq<OutgoingLen, IncomingLen, Timer>,
        ),
        Error,
    > {
        let (atr_prod, atr_cons) = self
            .app_to_radio_buf
            .try_split_framed()
            .map_err(|_| Error::AlreadySplit)?;
        let (rta_prod, rta_cons) = self
            .radio_to_app_buf
            .try_split_framed()
            .map_err(|_| Error::AlreadySplit)?;

        let app = EsbApp {
            prod_to_radio: atr_prod,
            cons_from_radio: rta_cons,
        };

        let mut irq = EsbIrq {
            prod_to_app: rta_prod,
            cons_from_app: atr_cons,
            timer,
            radio: ESBRadio::new(radio),
            state: State::Idle,
            addresses,
            attempts: 0,
        };

        // TODO: Set up radio?
        irq.radio.init(max_payload_bytes, &irq.addresses);
        irq.timer.init();

        Ok((app, irq))
    }
}

/// Stuff for the application side of things
/// (like ...?)
pub struct EsbApp<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    prod_to_radio: FrameProducer<'static, OutgoingLen>,
    cons_from_radio: FrameConsumer<'static, IncomingLen>,
    // Probably some more stuff
    // How do we pend the interrupt?
    // Blindly pend the interrupt?
}

/// Stuff for the interrupt side of things
/// (like BleRadio)
pub struct EsbIrq<OutgoingLen, IncomingLen, Timer>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
    Timer: ESBTimer,
{
    prod_to_app: FrameProducer<'static, IncomingLen>,
    cons_from_app: FrameConsumer<'static, OutgoingLen>,
    timer: Timer,
    radio: ESBRadio,
    state: State,
    addresses: Addresses,
    attempts: u8,
}

impl<OutgoingLen, IncomingLen, Timer> EsbIrq<OutgoingLen, IncomingLen, Timer>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
    Timer: ESBTimer,
{

}

// |               ACTUAL DMA PART                                    | SW USE                        |
// | length - 1 byte | pid_no_ack - 1 byte | payload - 1 to 252 bytes | rssi - 1 byte | pipe - 1 byte |

pub struct PayloadW<N: ArrayLength<u8>> {
    pub grant: FrameGrantW<'static, N>,
}

impl<N> PayloadW<N>
where
    N: ArrayLength<u8>
{
    pub fn new(
        raw_grant: FrameGrantW<'static, N>,
        pid_no_ack: u8,
        pipe: u8,
        rssi: u8, // doesn't mean anything when sending, only when receiving
    ) -> Self {
        // length gets written on commit

        // TODO(AJM), review multiple raw access, bounds checking
        raw_grant[1] = pid_no_ack;
        raw
    }
}

use core::ops::{Deref, DerefMut};

impl<N> Deref for PayloadW<N>
where
    N: ArrayLength<u8>
{
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        let len = self.grant.len();
        let payload_len = len - 1 - 1 - 1 - 1;
        &self.grant[2..payload_len + 2]
    }
}

impl<N> DerefMut for PayloadW<N>
where
    N: ArrayLength<u8>,
{
    fn deref_mut(&mut self) -> &mut [u8] {
        let len = self.grant.len();
        let payload_len = len - 1 - 1 - 1 - 1;
        &mut self.grant[2..payload_len + 2]
    }
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
