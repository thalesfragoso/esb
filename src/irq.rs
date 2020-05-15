use crate::{
    app::Addresses,
    payload::{EsbHeader, PayloadR, PayloadW},
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

/// The current state of the radio
#[derive(PartialEq, Copy, Clone, Debug)]
pub enum State {
    /// The radio is idle in PTX mode
    IdleTx,
    /// The radio is idle in PRX mode
    IdleRx,
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

struct Events {
    disabled: bool,
    timer: bool,
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
        let Events { disabled, timer } = self.check_and_clear_flags();

        // We only trigger the interrupt in these three events, if we didn't trigger it then the
        // user did.
        let user_event = !disabled && !timer;

        if user_event && !(self.state == State::IdleTx || self.state == State::IdleRx) {
            return Ok(self.state);
        }

        match self.state {
            State::IdleTx => {
                // Interrupt received means that the user pushed a packet to the queue.

                // TODO: Sometimes we get a wrong timer interrupt here, found out what is the
                // problem
                //debug_assert!(user_event);
                if user_event {
                    self.send_packet();
                }
            }
            State::TransmitterTxNoAck => {
                // Transmission ended
                self.radio.finish_tx_no_ack();
                self.send_packet();
            }
            State::TransmitterTx => {
                debug_assert!(disabled, "TransmitterTx de: {}, te: {}", disabled, timer);

                let packet = self
                    .prod_to_app
                    .grant(self.config.maximum_payload_size as usize + EsbHeader::header_size())
                    .map(PayloadW::new_from_radio);
                if let Ok(packet) = packet {
                    self.radio.prepare_for_ack(packet);
                    self.state = State::TransmitterWaitAck;
                } else {
                    self.radio.stop(true);
                    self.state = State::IdleTx;
                    return Err(Error::IncomingQueueFull);
                }

                // The radio will be disabled if we retransmit, because of that we need to take into
                // account the ramp-up time for TX
                self.timer
                    .set_interrupt_retransmit(self.config.retransmit_delay - RAMP_UP_TIME);

                // Takes into account the RX ramp-up time
                self.timer
                    .set_interrupt_ack(self.config.wait_for_ack_timeout + RAMP_UP_TIME);
            }
            State::TransmitterWaitAck => {
                let mut retransmit = false;
                if disabled {
                    // We got an ack, check it
                    Timer::clear_interrupt_ack();
                    if self.radio.check_ack()? {
                        // Everything went fine, `clear_interrupt_retransmit` also resets and stops
                        // the timer
                        Timer::clear_interrupt_retransmit();
                        self.attempts = 0;
                        self.send_packet();
                    } else {
                        // CRC mismatch, wait for retransmission
                        retransmit = true;
                    }
                } else {
                    debug_assert!(timer, "TransmitterWaitAck de: {}, te: {}", disabled, timer);
                    // Ack timeout
                    retransmit = true;
                }
                if retransmit {
                    self.radio.stop(true);
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
                    self.send_packet();
                    return Err(Error::MaximumAttempts);
                }
            }
            State::TransmitterWaitRetransmit => {
                debug_assert!(
                    timer,
                    "TransmitterWaitRetransmit de: {}, te: {}",
                    disabled, timer
                );
                // The timer interrupt cleared and stopped the timer by now
                self.send_packet();
            }
            State::Receiver => {
                debug_assert!(disabled, "Receiver de: {}, te: {}", disabled, timer);
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
                debug_assert!(disabled, "TransmittingAck de: {}, te: {}", disabled, timer);
                // We finished transmitting the acknowledgement, get ready for the next packet
                self.prepare_receiver(|this, grant| {
                    // This goes back to rx
                    this.radio.complete_rx_ack(Some(grant))?;
                    this.state = State::Receiver;
                    Ok(())
                })?;
            }
            State::TransmittingRepeatedAck => {
                debug_assert!(
                    disabled,
                    "TransmittingRepeatedAck de: {}, te: {}",
                    disabled, timer
                );
                // This goes back to rx
                self.radio.complete_rx_ack(None)?;
                self.state = State::Receiver;
            }
            State::IdleRx => {
                debug_assert!(
                    user_event,
                    "TransmittingRepeatedAck de: {}, te: {}",
                    disabled, timer
                );
                self.start_receiving()?;
            }
        }
        Ok(self.state)
    }

    /// Changes esb to the receiving state
    pub fn start_receiving(&mut self) -> Result<(), Error> {
        if self.state != State::IdleTx || self.state != State::IdleRx {
            // Put the radio in a known state
            self.radio.stop(true);
            Timer::clear_interrupt_retransmit();
            Timer::clear_interrupt_ack();
            let _ = self.check_and_clear_flags();
        }
        self.prepare_receiver(|this, grant| {
            this.radio.start_receiving(grant, this.config.enabled_pipes);
            this.state = State::Receiver;
            Ok(())
        })
    }

    /// Stops the receiving
    pub fn stop_receiving(&mut self) {
        // Put the radio in a known state
        self.radio.stop(true);
        Timer::clear_interrupt_retransmit();
        Timer::clear_interrupt_ack();
        let _ = self.check_and_clear_flags();

        self.state = State::IdleRx;
    }

    fn check_and_clear_flags(&mut self) -> Events {
        let evts = Events {
            disabled: self.radio.check_disabled_event(),
            timer: self.timer_flag.load(Ordering::Acquire),
        };

        if evts.disabled {
            self.radio.clear_disabled_event();
        }
        if evts.timer {
            self.timer_flag.store(false, Ordering::Release);
        }

        // TODO: Try to remove this, probably not necessary
        NVIC::unpend(Interrupt::RADIO);

        evts
    }

    fn send_packet(&mut self) {
        if let Some(packet) = self.cons_from_app.read().map(PayloadR::new) {
            let ack = !packet.no_ack();
            self.radio.transmit(packet, ack);
            if ack {
                self.state = State::TransmitterTx;
            } else {
                self.state = State::TransmitterTxNoAck;
            }
        } else {
            self.radio.disable_disabled_interrupt();
            self.state = State::IdleTx;
        }
    }

    fn prepare_receiver<F>(&mut self, f: F) -> Result<(), Error>
    where
        F: FnOnce(&mut Self, PayloadW<IncomingLen>) -> Result<(), Error>,
    {
        if let Ok(grant) = self
            .prod_to_app
            .grant(self.config.maximum_payload_size as usize + EsbHeader::header_size())
            .map(PayloadW::new_from_radio)
        {
            f(self, grant)?;
            Ok(())
        } else {
            self.radio.stop(true);
            self.state = State::IdleRx;
            Err(Error::IncomingQueueFull)
        }
    }
}
