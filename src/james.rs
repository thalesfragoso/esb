use crate::packet::Addresses;
use crate::peripherals::{ESBRadio, ESBTimer, RADIO};
use crate::Error;
use crate::State;
use bbqueue::{
    framed::{FrameConsumer, FrameProducer, FrameGrantW, FrameGrantR},
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
    radio: ESBRadio<IncomingLen, OutgoingLen>,
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

// | SW USE                        |               ACTUAL DMA PART                                    |
// | rssi - 1 byte | pipe - 1 byte | length - 1 byte | pid_no_ack - 1 byte | payload - 1 to 252 bytes |

pub struct PayloadHeader {
    rssi: u8,
    pipe: u8,
    length: u8,
    pid_no_ack: u8,
}

pub type PhBytes = [u8; 4];
impl PayloadHeader {

    fn to_bytes(self) -> PhBytes {

        [
            self.rssi,
            self.pipe,

            // DO NOT REORDER!
            self.length,
            self.pid_no_ack,
        ]
    }

    fn from_bytes(bytes: &PhBytes) -> Self {
        Self {
            rssi: bytes[Self::rssi_idx()],
            pipe: bytes[Self::pipe_idx()],
            length: bytes[Self::length_idx()],
            pid_no_ack: bytes[Self::pid_no_ack_idx()],
        }
    }

    const fn rssi_idx() -> usize {
        0
    }

    const fn pipe_idx() -> usize {
        1
    }

    const fn length_idx() -> usize {
        // DO NOT CHANGE! HW DEPENDANT
        2
    }

    const fn pid_no_ack_idx() -> usize {
        // DO NOT CHANGE! HW DEPENDANT
        3
    }

    const fn header_size() -> usize {
        core::mem::size_of::<PhBytes>()
    }

    const fn dma_payload_offset() -> isize {
        2
    }
}

pub struct PayloadR<N: ArrayLength<u8>> {
    pub grant: FrameGrantR<'static, N>,
}

impl<N> PayloadR<N>
where
    N: ArrayLength<u8>
{
    pub fn new(
        raw_grant: FrameGrantR<'static, N>,
    ) -> Self {
        Self {
            grant: raw_grant
        }
    }

    pub fn get_header(&self) -> PayloadHeader {
        const LEN: usize = PayloadHeader::header_size();
        let mut bytes = [0u8; LEN];
        bytes.copy_from_slice(&self.grant[..LEN]);
        PayloadHeader::from_bytes(&bytes)
    }

    pub unsafe fn dma_pointer(&mut self) -> *const u8 {
        self.grant
            .deref()
            .as_ptr()
            .offset(PayloadHeader::dma_payload_offset())
    }

    pub fn pipe(&self) -> u8 {
        self.grant[PayloadHeader::pipe_idx()]
    }

    pub fn release(self) {
        self.grant.release()
    }
}

use core::ops::{Deref, DerefMut};

impl<N> Deref for PayloadR<N>
where
    N: ArrayLength<u8>
{
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        let len = self.grant.len();
        &self.grant[PayloadHeader::header_size()..]
    }
}

pub struct PayloadW<N: ArrayLength<u8>> {
    pub grant: FrameGrantW<'static, N>,
}

impl<N> PayloadW<N>
where
    N: ArrayLength<u8>
{
    pub(crate) fn new_from_app(
        mut raw_grant: FrameGrantW<'static, N>,
        header: PayloadHeader, // HMMMMMM
    ) -> Self {
        raw_grant[..PayloadHeader::header_size()].copy_from_slice(&header.to_bytes());
        Self {
            grant: raw_grant
        }
    }

    pub(crate) fn new_from_radio(
        mut raw_grant: FrameGrantW<'static, N>,
    ) -> Self {
        Self {
            grant: raw_grant,
        }
    }

    pub(crate) fn update_header(
        &mut self,
        header: PayloadHeader
    ) {
        self.grant[..PayloadHeader::header_size()].copy_from_slice(&header.to_bytes());
    }

    pub unsafe fn dma_pointer(&mut self) -> *mut u8 {
        self.grant
            .deref_mut()
            .as_mut_ptr()
            .offset(PayloadHeader::dma_payload_offset())
    }

    pub fn pipe(&self) -> u8 {
        self.grant[PayloadHeader::pipe_idx()]
    }

    pub fn payload_len(&self) -> usize {
        self.grant[PayloadHeader::length_idx()] as usize
    }

    pub fn commit_all(self) {
        let payload_len = self.payload_len();
        self.grant.commit(payload_len + PayloadHeader::header_size())
    }

    pub fn commit(mut self, used: usize) {
        let max_payload_len = self.payload_len();
        let payload_len = core::cmp::min(used, max_payload_len);
        self.grant[PayloadHeader::length_idx()] = payload_len as u8;

        self.grant.commit(payload_len + PayloadHeader::header_size())
    }
}

impl<N> Deref for PayloadW<N>
where
    N: ArrayLength<u8>
{
    type Target = [u8];

    fn deref(&self) -> &Self::Target {
        let len = self.grant.len();
        &self.grant[PayloadHeader::header_size()..]
    }
}

impl<N> DerefMut for PayloadW<N>
where
    N: ArrayLength<u8>,
{
    fn deref_mut(&mut self) -> &mut [u8] {
        &mut self.grant[PayloadHeader::header_size()..]
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
