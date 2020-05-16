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
    pub fn try_release<T: EsbTimer>(
        &'static self,
        mut app: EsbApp<OutgoingLen, IncomingLen>,
        mut irq: EsbIrq<OutgoingLen, IncomingLen, T>,
        timer: IrqTimer<T>,
    ) -> Result<
        (T, RADIO),
        (
            EsbApp<OutgoingLen, IncomingLen>,
            EsbIrq<OutgoingLen, IncomingLen, T>,
            IrqTimer<T>,
        ),
    > {
        // TODO: We probably need to disable/ensure the radio is idle?
        // TODO: @thalesfragoso help

        let prod1 = app.prod_to_radio;
        let cons1 = irq.cons_from_app;
        let prod2 = irq.prod_to_app;
        let cons2 = app.cons_from_radio;

        let atr_res = self.app_to_radio_buf.try_release_framed(prod1, cons1);
        let rta_res = self.radio_to_app_buf.try_release_framed(prod2, cons2);

        match (atr_res, rta_res) {
            (Ok(()), Ok(())) => {
                // TODO: We probably want a `release` method on the `EsbIrq` structure
                // that makes sure that we are idle, and gives us these items back, instead
                // of just partially destructuring and making these fields pub(crate).
                //
                // That should probably be done up around the other TODO comment above,
                // where we can early return a little easier
                Ok((irq.timer, irq.radio.radio))
            }
            (Ok(()), Err((prod2, cons2))) => {
                irq.prod_to_app = prod2;
                app.cons_from_radio = cons2;

                let (prod1, cons1) = self.app_to_radio_buf.try_split_framed().unwrap();
                app.prod_to_radio = prod1;
                irq.cons_from_app = cons1;

                Err((app, irq, timer))
            }
            (Err((prod1, cons1)), Ok(())) => {
                app.prod_to_radio = prod1;
                irq.cons_from_app = cons1;

                let (prod2, cons2) = self.radio_to_app_buf.try_split_framed().unwrap();
                irq.prod_to_app = prod2;
                app.cons_from_radio = cons2;

                Err((app, irq, timer))
            }
            (Err((prod1, cons1)), Err((prod2, cons2))) => {
                app.prod_to_radio = prod1;
                irq.cons_from_app = cons1;
                irq.prod_to_app = prod2;
                app.cons_from_radio = cons2;

                Err((app, irq, timer))
            }
        }
    }

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
        max_payload_bytes: u8,
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

        irq.radio.init(max_payload_bytes, &irq.addresses);
        irq.timer.init();

        Ok((app, irq, irq_timer))
    }
}
