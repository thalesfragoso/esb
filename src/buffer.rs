use crate::{
    app::{Addresses, EsbApp},
    irq::{Disabled, EsbIrq, IrqTimer},
    peripherals::{EsbRadio, EsbTimer, RADIO},
    Config, Error,
};
use bbqueue::BBBuffer;
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
/// ```rust
/// // This creates an ESB storage structure with room for
/// // 512 bytes of outgoing packets (including headers),
/// // and 256 bytes of incoming packets (including
/// // headers).
/// # use esb::EsbBuffer;
/// static BUFFER: EsbBuffer<512, 256> = EsbBuffer::new();
/// ```
pub struct EsbBuffer<const OUTGOING_LEN: usize, const INCOMING_LEN: usize> {
    pub(crate) app_to_radio_buf: BBBuffer<OUTGOING_LEN>,
    pub(crate) radio_to_app_buf: BBBuffer<INCOMING_LEN>,
    pub(crate) timer_flag: AtomicBool,
}

impl<const OUTGOING_LEN: usize, const INCOMING_LEN: usize> EsbBuffer<OUTGOING_LEN, INCOMING_LEN> {
    /// Create a new EsbBuffer. This is typically to be done at static scope/lifetime.
    ///
    /// ```rust
    /// // This creates an ESB storage structure with room for
    /// // 512 bytes of outgoing packets (including headers),
    /// // and 256 bytes of incoming packets (including
    /// // headers).
    /// # use esb::EsbBuffer;
    /// static BUFFER: EsbBuffer<512, 256> = EsbBuffer::new();
    /// ```
    pub const fn new() -> Self {
        EsbBuffer {
            app_to_radio_buf: BBBuffer::new(),
            radio_to_app_buf: BBBuffer::new(),
            timer_flag: AtomicBool::new(false),
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
        config: Config,
    ) -> Result<
        (
            EsbApp<OUTGOING_LEN, INCOMING_LEN>,
            EsbIrq<T, Disabled, OUTGOING_LEN, INCOMING_LEN>,
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
            state: Disabled,
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
