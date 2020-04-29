use crate::packet::Addresses;
use crate::peripherals::{ESBRadio, ESBTimer, RADIO};
use crate::Error;
use crate::State;
use bbqueue::{
    framed::{FrameConsumer, FrameProducer, FrameGrantW, FrameGrantR},
    ArrayLength, BBBuffer,
};
use core::result::Result;

/// Stuff for the application side of things
/// (like ...?)
pub struct EsbApp<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    pub(crate) prod_to_radio: FrameProducer<'static, OutgoingLen>,
    pub(crate) cons_from_radio: FrameConsumer<'static, IncomingLen>,
    // Probably some more stuff
    // How do we pend the interrupt?
    // Blindly pend the interrupt?
}
