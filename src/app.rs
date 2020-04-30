use crate::packet::Addresses;
use crate::payload::{PayloadHeader, PayloadR, PayloadW};
use crate::peripherals::{ESBRadio, ESBTimer, Interrupt, NVIC, RADIO};
use crate::Error;
use crate::State;
use bbqueue::{
    framed::{FrameConsumer, FrameGrantR, FrameGrantW, FrameProducer},
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

impl<OutgoingLen, IncomingLen> EsbApp<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    pub fn grant_packet(&mut self, header: PayloadHeader) -> Result<PayloadW<OutgoingLen>, Error> {
        let grant = self
            .prod_to_radio
            .grant(header.payload_len() + PayloadHeader::header_size())
            .map_err(|_| Error::QueueFull)?;
        Ok(PayloadW::new_from_app(grant, header))
    }

    /// Starts the radio sending all packets in the queue.
    /// The radio will send until the queue has been drained.
    pub fn start_tx(&mut self) {
        // Do we need to do anything other than pend the interrupt?
        NVIC::pend(Interrupt::RADIO)
    }

    pub fn msg_ready(&mut self) -> bool {
        // Dropping the grant does not release it.
        self.cons_from_radio.read().is_some()
    }

    pub fn read_packet(&mut self) -> Option<PayloadR<IncomingLen>> {
        self.cons_from_radio.read().map(PayloadR::new)
    }
}
