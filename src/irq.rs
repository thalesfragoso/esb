use crate::{
    app::Addresses,
    peripherals::{EsbRadio, EsbTimer},
};
use bbqueue::{
    framed::{FrameConsumer, FrameProducer},
    ArrayLength,
};

/// The current state of the radio
#[derive(PartialEq)]
pub enum State {
    /// The radio is idle
    Idle,
    /// The radio is preparing for reception (in PTX)
    RampUpRx,
    /// The radio is preparing for transmission.
    RampUpTx,
    /// ESB on PTX state transmitting.
    TransmitterTx,
    TransmitterTxNoAck,
    /// ESB on PTX state waiting for ack.
    TransmitterWaitAck,
    /// ESB on PTX state waiting for the retransmit timeout.
    TransmitterWaitRetransmit,
    /// ESB in the PRX state listening for packets
    Receiver,
}

/// This is the primary RADIO-interrupt-side interface.
///
/// It is intended to be used inside of the `RADIO` interrupt,
/// and allows for sending or receiving frames from the Application
/// hardware.
pub struct EsbIrq<OutgoingLen, IncomingLen, Timer>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
    Timer: EsbTimer,
{
    /// Producer to send incoming frames FROM the radio, TO the application
    pub(crate) prod_to_app: FrameProducer<'static, IncomingLen>,

    /// Consumer to receive outgoing frames TO the radio, FROM the application
    pub(crate) cons_from_app: FrameConsumer<'static, OutgoingLen>,

    /// Peripheral timer, use for ACK and other timeouts
    pub(crate) timer: Timer,

    /// Wrapping structure of the nRF RADIO peripheral
    pub(crate) radio: EsbRadio<OutgoingLen, IncomingLen>,

    /// Current state of the Radio/IRQ task
    pub(crate) state: State,

    /// The assigned addresses of the ESB radio an pipes
    pub(crate) addresses: Addresses,

    /// The number of attempts to send the current packet
    pub(crate) attempts: u8,
}

impl<OutgoingLen, IncomingLen, Timer> EsbIrq<OutgoingLen, IncomingLen, Timer>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
    Timer: EsbTimer,
{
    // Hmmm, this needs some methods
}
