use crate::payload::{PayloadHeader, PayloadR, PayloadW};
use crate::peripherals::{Interrupt, NVIC};
use crate::Error;
use bbqueue::{
    framed::{FrameConsumer, FrameProducer},
    ArrayLength,
};

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
    pub fn grant_packet(
        &mut self,
        payload_length: u8,
        pid: u8,
        pipe: u8,
        no_ack: bool,
    ) -> Result<PayloadW<OutgoingLen>, Error> {
        if payload_length > 252 || pid > 3 || pipe > 7 {
            return Err(Error::InvalidParameters);
        }
        // This is weird, Nordic uses the `no-ack` bit as "active-low"
        let pid_no_ack = pid << 1 | if no_ack { 0x00 } else { 0x01 };
        let header = PayloadHeader::from_bytes([0, pipe, payload_length, pid_no_ack]);
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

/// Addresses used for communication.
/// ESB uses up to eight pipes to address communication, each pipe has an unique address which is
/// composed by the base address and the prefix. Pipe 0 has an unique base and prefix, while the
/// other pipes share a base address but have different prefixes.
pub struct Addresses {
    pub(crate) base0: [u8; 4],
    pub(crate) base1: [u8; 4],
    pub(crate) prefixes0: [u8; 4],
    pub(crate) prefixes1: [u8; 4],
    pub(crate) rf_channel: u8,
}

impl Addresses {
    /// Creates a new instance of `Addresses`
    ///
    /// * `base0` - Base address for pipe 0.
    /// * `base1` - Base address for pipe 1-7.
    /// * `prefixes0` - Prefixes for pipes 0-3, in order.
    /// * `prefixes1` - Prefixes for pipes 4-7, in order.
    /// * `rf_channel` - Channel to be used by the radio hardware (must be between 0 and 100).
    ///
    /// # Panics
    ///
    /// This function will panic if `rf_channel` is bigger than 100.
    pub fn new(
        base0: [u8; 4],
        base1: [u8; 4],
        prefixes0: [u8; 4],
        prefixes1: [u8; 4],
        rf_channel: u8,
    ) -> Self {
        // TODO(AJM): Move to a builder pattern here?
        assert!(rf_channel <= 100);
        Self {
            base0,
            base1,
            prefixes0,
            prefixes1,
            rf_channel,
        }
    }
}
