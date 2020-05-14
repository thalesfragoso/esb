use crate::{
    app::{Addresses, EsbApp},
    irq::{EsbIrq, IrqTimer, State},
    peripherals::{EsbRadio, EsbTimer, RADIO},
    Config, Error,
};
use bbqueue::{ArrayLength, BBBuffer};
use core::{
    marker::PhantomData,
    sync::atomic::{AtomicBool, Ordering},
};

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
/// # use core::sync::atomic::AtomicBool;
/// static BUFFER: EsbBuffer<U512, U256> = EsbBuffer {
///     app_to_radio_buf: BBBuffer( ConstBBBuffer::new() ),
///     radio_to_app_buf: BBBuffer( ConstBBBuffer::new() ),
///     timer_flag: AtomicBool::new(false),
/// };
/// ```
pub struct EsbBuffer<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    pub app_to_radio_buf: BBBuffer<OutgoingLen>,
    pub radio_to_app_buf: BBBuffer<IncomingLen>,
    pub timer_flag: AtomicBool,
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
    /// Upon splitting, the Radio will be initialized and set to
    /// [IdleTx](enum.State.html#variant.IdleTx).
    #[allow(clippy::type_complexity)]
    pub fn try_split<T: EsbTimer>(
        &'static self,
        timer: T,
        radio: RADIO,
        addresses: Addresses,
        config: Config,
    ) -> Result<
        (
            EsbApp<OutgoingLen, IncomingLen>,
            EsbIrq<OutgoingLen, IncomingLen, T>,
            IrqTimer<T>,
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

        // Clear the timer flag
        self.timer_flag.store(false, Ordering::Release);

        let app = EsbApp {
            prod_to_radio: atr_prod,
            cons_from_radio: rta_cons,
            maximum_payload: config.maximum_payload_size,
        };

        let mut irq = EsbIrq {
            prod_to_app: rta_prod,
            cons_from_app: atr_cons,
            timer,
            radio: EsbRadio::new(radio),
            state: State::IdleTx,
            addresses,
            attempts: 0,
            timer_flag: &self.timer_flag,
            config,
        };

        let irq_timer = IrqTimer {
            timer_flag: &self.timer_flag,
            _timer: PhantomData,
        };

        irq.radio.init(
            irq.config.maximum_payload_size,
            irq.config.tx_power,
            &irq.addresses,
        );
        irq.timer.init();

        Ok((app, irq, irq_timer))
    }
}
