use crate::{
    payload::{EsbHeader, PayloadR, PayloadW},
    peripherals::{Interrupt, NVIC},
    Error,
};
use bbqueue::{
    framed::{FrameConsumer, FrameProducer},
    ArrayLength, Error as BbqError,
};
use core::default::Default;

/// Driver's maximum payload size
///
/// Can be acquired through [EsbApp](struct.EsbApp.html)
#[derive(Copy, Clone)]
pub struct DriverMaximumPayload {
    pub(crate) inner_checked: u8,
}

impl DriverMaximumPayload {
    /// Creates a `DriverMaximumPayload` with an unchecked value. It is recommended to use the
    /// `maximum_payload_size` method from [EsbApp](struct.EsbApp.html) instead of this one
    ///
    /// # Safety
    ///
    /// The value passed to this method must be the same as the one in the `Config` struct used
    /// to initialize the driver
    pub const unsafe fn new_unchecked(maximum: u8) -> Self {
        DriverMaximumPayload {
            inner_checked: maximum,
        }
    }
}

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
    pub(crate) maximum_payload: DriverMaximumPayload,
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
    /// or if a grant is already in progress, an error will be returned.
    ///
    /// ## Notes
    ///
    /// Once a grant has been created, the maximum size of the grant can not
    /// be increased, only shrunk. If a larger grant is needed, you must
    /// `drop` the old grant, and create a new one.
    ///
    /// Only one grant may be active at a time.
    pub fn grant_packet(&mut self, header: EsbHeader) -> Result<PayloadW<OutgoingLen>, Error> {
        let grant_result = self
            .prod_to_radio
            .grant(header.payload_len() + EsbHeader::header_size());

        let grant = grant_result.map_err(|err| match err {
            BbqError::GrantInProgress => Error::GrantInProgress,
            BbqError::InsufficientSize => Error::OutgoingQueueFull,
            _ => Error::InternalError,
        })?;
        Ok(PayloadW::new_from_app(grant, header))
    }

    /// Starts the radio sending all packets in the queue.
    ///
    /// The radio will send until the queue has been drained.
    #[inline]
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

    /// Gets the maximum payload size that the driver was configured to use
    #[inline]
    pub fn maximum_payload_size(&self) -> DriverMaximumPayload {
        self.maximum_payload
    }
}

/// Addresses used for communication.
///
/// ESB uses up to eight pipes to address communication, each pipe has an unique address which is
/// composed by the base address and the prefix. Pipe 0 has an unique base and prefix, while the
/// other pipes share a base address but have different prefixes.
///
/// Default values:
///
/// | Field      | Default Value            |
/// | :---       | :---                     |
/// | base0      | [0xE7, 0xE7, 0xE7, 0xE7] |
/// | base1      | [0xC2, 0xC2, 0xC2, 0xC2] |
/// | prefixes0  | [0xE7, 0xC2, 0xC3, 0xC4] |
/// | prefixes1  | [0xC5, 0xC6, 0xC7, 0xC8] |
/// | rf_channel | 2                        |
///
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

// TODO: make a builder
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

impl Default for Addresses {
    fn default() -> Self {
        Self {
            base0: [0xE7, 0xE7, 0xE7, 0xE7],
            base1: [0xC2, 0xC2, 0xC2, 0xC2],
            prefixes0: [0xE7, 0xC2, 0xC3, 0xC4],
            prefixes1: [0xC5, 0xC6, 0xC7, 0xC8],
            rf_channel: 2,
        }
    }
}
