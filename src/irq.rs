use crate::{
    app::Addresses,
    payload::{PayloadR, PayloadW},
    peripherals::{EsbRadio, EsbTimer, Interrupt, RxPayloadState, NVIC},
    Config, Error, RAMP_UP_TIME,
};
use bbqueue::{
    framed::{FrameConsumer, FrameProducer},
    ArrayLength,
};
use core::{
    marker::PhantomData,
    sync::atomic::{AtomicBool, Ordering},
};

const MAX_PACKET_SIZE: usize = 256;

/// The current state of the radio
#[derive(PartialEq, Copy, Clone)]
pub enum State {
    /// The radio is idle in PTX mode
    IdleTx,
    /// The radio is idle in PRX mode
    IdleRx,
    /// The radio is preparing for reception in PTX mode
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
    /// Transmitting the acknowledgement
    TransmittingAck,
    /// Transmitting the acknowledgement for a repeated packet
    TransmittingRepeatedAck,
}

pub struct IrqTimer<T: EsbTimer> {
    /// Flag to determine if the timer caused the interrupt
    pub(crate) timer_flag: &'static AtomicBool,
    pub(crate) _timer: PhantomData<T>,
}

impl<T: EsbTimer> IrqTimer<T> {
    /// Must be called inside the timer interrupt handler.
    pub fn timer_interrupt(&mut self) {
        // Check which event triggered the timer
        let retransmit_event = T::is_retransmit_pending();
        let ack_timeout_event = T::is_ack_pending();

        if retransmit_event {
            T::clear_interrupt_retransmit();
        }
        // Retransmit might have been triggered just after ack, check and clear ack too
        if ack_timeout_event {
            T::clear_interrupt_ack();
        }
        self.timer_flag.store(true, Ordering::Release);
        NVIC::pend(Interrupt::RADIO);
    }
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

    /// Flag to determine if the timer caused the interrupt
    pub(crate) timer_flag: &'static AtomicBool,

    /// Protocol configuration
    pub(crate) config: Config,
}

impl<OutgoingLen, IncomingLen, Timer> EsbIrq<OutgoingLen, IncomingLen, Timer>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
    Timer: EsbTimer,
{
    /// Must be called inside the radio interrupt handler.
    #[allow(clippy::cognitive_complexity)]
    pub fn radio_interrupt(&mut self) -> Result<State, Error> {
        let disabled_event = self.radio.check_disabled_event();
        let ready_event = self.radio.check_ready_event();
        let timer_event = self.timer_flag.load(Ordering::Acquire);

        // We only trigger the interrupt in these three events, if we didn't trigger it then the
        // user did.
        let user_event = !disabled_event && !ready_event && !timer_event;

        self.clear_flags();

        if user_event && (self.state != State::IdleTx || self.state != State::IdleRx) {
            return Ok(self.state);
        }

        match self.state {
            State::IdleTx => {
                // Interrupt received means that the user pushed a packet to the queue.
                debug_assert!(user_event);
                self.send_packet()?;
            }
            State::TransmitterTxNoAck => {
                // Transmission ended
                self.radio.finish_tx_no_ack();
                self.send_packet()?;
            }
            State::RampUpTx => {
                debug_assert!(ready_event);

                // The radio will be disabled if we retransmit, because of that we need to take into
                // account the ramp-up time for TX
                self.timer
                    .set_interrupt_retransmit(self.config.retransmit_delay - RAMP_UP_TIME);
                self.state = State::TransmitterTx;
            }
            State::TransmitterTx => {
                debug_assert!(disabled_event);

                let packet = self
                    .prod_to_app
                    .grant(MAX_PACKET_SIZE)
                    .map(PayloadW::new_from_radio);
                if let Ok(packet) = packet {
                    self.radio.prepare_for_ack(packet);
                    self.state = State::RampUpRx;
                } else {
                    self.radio.stop();
                    Timer::clear_interrupt_retransmit();
                    self.state = State::IdleTx;
                    return Err(Error::IncomingQueueFull);
                }
            }
            State::RampUpRx => {
                debug_assert!(ready_event);

                self.radio.disable_ready_interrupt();
                self.timer
                    .set_interrupt_ack(self.config.wait_for_ack_timeout);
                self.state = State::TransmitterWaitAck;
            }
            State::TransmitterWaitAck => {
                let mut retransmit = false;
                if disabled_event {
                    // We got an ack, check it
                    Timer::clear_interrupt_ack();
                    if self.radio.check_ack()? {
                        // Everything went fine, `clear_interrupt_retransmit` also resets and stops
                        // the timer
                        Timer::clear_interrupt_retransmit();
                        self.attempts = 0;
                        self.send_packet()?;
                    } else {
                        // CRC mismatch, wait for retransmission
                        retransmit = true;
                    }
                } else {
                    debug_assert!(timer_event);
                    // Ack timeout
                    retransmit = true;
                }
                if retransmit {
                    self.radio.stop();
                    self.attempts += 1;
                    self.state = State::TransmitterWaitRetransmit;
                }
                if self.attempts > self.config.maximum_transmit_attempts {
                    Timer::clear_interrupt_retransmit();

                    // We reached the maximum number of attempts, `radio.stop()` dropped the radio
                    // grants and we will release the last packet and try the next one
                    if let Some(old_packet) = self.cons_from_app.read() {
                        old_packet.release();
                    }
                    self.attempts = 0;
                    self.send_packet()?;
                    return Err(Error::MaximumAttempts);
                }
            }
            State::TransmitterWaitRetransmit => {
                debug_assert!(timer_event);
                // The timer interrupt cleared and stopped the timer by now
                self.send_packet()?;
            }
            State::Receiver => {
                debug_assert!(disabled_event);
                // We got a packet, check it
                match self.radio.check_packet(&mut self.cons_from_app)? {
                    // Do nothing, the radio will return to rx
                    RxPayloadState::BadCRC => {}
                    RxPayloadState::NoAck => {
                        self.prepare_receiver(|this, grant| {
                            this.radio.complete_rx_no_ack(Some(grant));
                            Ok(())
                        })?;
                    }
                    RxPayloadState::RepeatedNoAck => {
                        // this goes back to rx
                        self.radio.complete_rx_no_ack(None);
                    }
                    RxPayloadState::Ack => {
                        self.state = State::TransmittingAck;
                    }
                    RxPayloadState::RepeatedAck => {
                        self.state = State::TransmittingRepeatedAck;
                    }
                }
            }
            State::TransmittingAck => {
                debug_assert!(disabled_event);
                // We finished transmitting the acknowledgement, get ready for the next packet
                self.prepare_receiver(|this, grant| {
                    // This goes back to rx
                    this.radio.complete_rx_ack(Some(grant))?;
                    this.state = State::Receiver;
                    Ok(())
                })?;
            }
            State::TransmittingRepeatedAck => {
                debug_assert!(disabled_event);
                // This goes back to rx
                self.radio.complete_rx_ack(None)?;
                self.state = State::Receiver;
            }
            State::IdleRx => {
                debug_assert!(user_event);
                self.start_receiving()?;
            }
        }
        Ok(self.state)
    }

    /// Changes the stack to the receiving state
    pub fn start_receiving(&mut self) -> Result<(), Error> {
        if self.state != State::IdleTx || self.state != State::IdleRx {
            // Put the radio in a known state
            self.radio.stop();
            Timer::clear_interrupt_retransmit();
            Timer::clear_interrupt_ack();
            self.clear_flags();
        }
        self.prepare_receiver(|this, grant| {
            this.radio.start_receiving(grant, this.config.enabled_pipes);
            this.state = State::Receiver;
            Ok(())
        })
    }

    #[inline]
    fn clear_flags(&mut self) {
        self.radio.clear_disabled_event();
        self.radio.clear_ready_event();
        self.timer_flag.store(false, Ordering::Release);
    }

    fn send_packet(&mut self) -> Result<(), Error> {
        if let Some(packet) = self.cons_from_app.read().map(PayloadR::new) {
            let ack = !packet.no_ack();
            self.radio.transmit(packet, ack);
            if ack {
                self.state = State::RampUpTx;
                Ok(())
            } else {
                self.state = State::TransmitterTxNoAck;
                Ok(())
            }
        } else {
            self.radio.disable_disabled_interrupt();
            self.state = State::IdleTx;
            Err(Error::OutgoingQueueEmpty)
        }
    }

    fn prepare_receiver<F>(&mut self, f: F) -> Result<(), Error>
    where
        F: FnOnce(&mut Self, PayloadW<IncomingLen>) -> Result<(), Error>,
    {
        if let Ok(grant) = self
            .prod_to_app
            .grant(MAX_PACKET_SIZE)
            .map(PayloadW::new_from_radio)
        {
            f(self, grant)?;
            Ok(())
        } else {
            self.radio.stop();
            self.state = State::IdleRx;
            Err(Error::IncomingQueueFull)
        }
    }
}
