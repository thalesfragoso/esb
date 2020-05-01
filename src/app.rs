use crate::payload::{EsbHeader, HeaderBytes, PayloadR, PayloadW};
use crate::peripherals::{Interrupt, NVIC};
use crate::Error;
use bbqueue::{
    framed::{FrameConsumer, FrameProducer},
    ArrayLength,
};

/// This is the primary Application-side interface.
///
/// It is intended to be used outside of the `RADIO` interrupt,
/// and allows for sending or receiving frames from the ESB Radio
/// hardware.
pub struct EsbApp<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    // TODO(AJM): Make a constructor for this so we don't
    // need to make these fields pub(crate)
    pub(crate) prod_to_radio: FrameProducer<'static, OutgoingLen>,
    pub(crate) cons_from_radio: FrameConsumer<'static, IncomingLen>,
}

impl<OutgoingLen, IncomingLen> EsbApp<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    /// Obtain a grant for an outgoing packet to be sent over the Radio
    ///
    /// When space is available, this function will return a [`PayloadW`],
    /// which can be written into for data to be sent over the radio. If
    /// the given parameters are incorrect, or if no space is available,
    /// an error will be returned.
    ///
    /// Notes on valid values:
    ///
    /// * `payload_length` must be between 0 and 252 bytes, inclusive.
    /// * `pid` must be between 0 and 3, inclusive.
    /// * `pipe` must be between 0 and 7, inclusive.
    pub fn grant_packet(
        &mut self,

        // TODO(AJM): @thalesfragoso, why don't we have the user just
        // provide a header here? It would make the interface less complex.
        // We could also provide a builder for the header.
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
        let header = EsbHeader::from_bytes(HeaderBytes([0, pipe, payload_length, pid_no_ack]));
        let grant = self
            .prod_to_radio
            .grant(header.payload_len() + EsbHeader::header_size())
            .map_err(|_| Error::QueueFull)?;
        Ok(PayloadW::new_from_app(grant, header))
    }

    /// Starts the radio sending all packets in the queue.
    ///
    /// The radio will send until the queue has been drained.
    pub fn start_tx(&mut self) {
        // TODO(AJM): Is this appropriate for PRX? Or is this a PTX-only
        // sort of interface?

        // Do we need to do anything other than pend the interrupt?
        NVIC::pend(Interrupt::RADIO)
    }

    /// Is there a received message that is ready to be read?
    ///
    /// Returns `true` if a call to `read_packet` would return `Some`.
    pub fn msg_ready(&mut self) -> bool {
        // Dropping the grant does not release it.
        self.cons_from_radio.read().is_some()
    }

    /// Attempt to read a packet that has been received via the radio
    ///
    /// Returns `Some(PayloadR)` if a packet is ready to be read,
    /// otherwise `None`.
    pub fn read_packet(&mut self) -> Option<PayloadR<IncomingLen>> {
        self.cons_from_radio.read().map(PayloadR::new)
    }
}

/// Addresses used for communication.
///
/// ESB uses up to eight pipes to address communication, each pipe has an unique address which is
/// composed by the base address and the prefix. Pipe 0 has an unique base and prefix, while the
/// other pipes share a base address but have different prefixes.
pub struct Addresses {
    /// Base address for pipe 0
    pub(crate) base0: [u8; 4],
    /// Base address for pipe 1-7
    pub(crate) base1: [u8; 4],
    /// Prefixes for pipes 0-3, in order
    pub(crate) prefixes0: [u8; 4],
    /// `prefixes1` - Prefixes for pipes 4-7, in order
    pub(crate) prefixes1: [u8; 4],
    /// Channel to be used by the radio hardware (must be between 0 and 100)
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
    ) -> Result<Self, Error> {
        // TODO(AJM): Move to a builder pattern here?
        if rf_channel > 100 {
            return Err(Error::InvalidParameters);
        }
        Ok(Self {
            base0,
            base1,
            prefixes0,
            prefixes1,
            rf_channel,
        })
    }
}
