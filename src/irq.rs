use crate::{
    app::Addresses,
    peripherals::{ESBRadio, ESBTimer},
    State,
};
use bbqueue::{
    framed::{FrameConsumer, FrameProducer},
    ArrayLength,
};

/// Stuff for the interrupt side of things
/// (like BleRadio)
pub struct EsbIrq<OutgoingLen, IncomingLen, Timer>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
    Timer: ESBTimer,
{
    pub(crate) prod_to_app: FrameProducer<'static, IncomingLen>,
    pub(crate) cons_from_app: FrameConsumer<'static, OutgoingLen>,
    pub(crate) timer: Timer,
    pub(crate) radio: ESBRadio<OutgoingLen, IncomingLen>,
    pub(crate) state: State,
    pub(crate) addresses: Addresses,
    pub(crate) attempts: u8,
}

impl<OutgoingLen, IncomingLen, Timer> EsbIrq<OutgoingLen, IncomingLen, Timer>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
    Timer: ESBTimer,
{
}
