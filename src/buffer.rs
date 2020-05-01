use crate::{
    app::{Addresses, EsbApp},
    irq::{EsbIrq, State},
    peripherals::{EsbRadio, EsbTimer, RADIO},
    Error,
};
use bbqueue::{ArrayLength, BBBuffer};

/// This is the backing structure for the ESB interface
///
/// It is intended to live at `'static` scope, and provides
/// storage for the `EsbApp` and `EsbIrq` interfaces
///
/// ## Creating at static scope
///
/// Currently due to lacking const generics, the UX for this
/// isn't great. You'll probably want something like this:
///
/// ## NOTE
///
/// Although the members of this struct are public, due to const
/// generic limitations, they are not intended to be used directly,
/// outside of `static` creation.
///
/// This could cause unintended, though not undefined, behavior.
///
/// TL;DR: It's not unsafe, but it's also not going to work correctly.
///
/// ```rust
/// // This creates an ESB storage structure with room for
/// // 512 bytes of outgoing packets (including headers),
/// // and 256 bytes of incoming packets (including
/// // headers).
/// # use esb::{BBBuffer, consts::*, ConstBBBuffer, EsbBuffer};
/// static BUFFER: EsbBuffer<U512, U256> = EsbBuffer {
///     app_to_radio_buf: BBBuffer( ConstBBBuffer::new() ),
///     radio_to_app_buf: BBBuffer( ConstBBBuffer::new() ),
/// };
/// ```
pub struct EsbBuffer<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    pub app_to_radio_buf: BBBuffer<OutgoingLen>,
    pub radio_to_app_buf: BBBuffer<IncomingLen>,
}

impl<OutgoingLen, IncomingLen> EsbBuffer<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    /// Attempt to split the `static` buffer into handles for Interrupt and App context
    ///
    /// This function will only succeed once. If the underlying buffers have also
    /// been split directly, this function will also fail.
    ///
    /// Upon splitting, the Radio will be initialized and set to idle.
    #[allow(clippy::type_complexity)]
    pub fn try_split<Timer: EsbTimer>(
        &'static self,
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
            radio: EsbRadio::new(radio),
            state: State::Idle,
            addresses,
            attempts: 0,
        };

        irq.radio.init(max_payload_bytes, &irq.addresses);
        irq.timer.init();

        Ok((app, irq))
    }
}
