use crate::{
    app::{Addresses, EsbApp},
    irq::EsbIrq,
    peripherals::{ESBRadio, ESBTimer, RADIO},
    Error, State,
};
use bbqueue::{ArrayLength, BBBuffer};

// I'm gunna be const!
pub struct EsbBuffer<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    app_to_radio_buf: BBBuffer<OutgoingLen>,
    radio_to_app_buf: BBBuffer<IncomingLen>,
    // Probably some more stuff?
}

impl<OutgoingLen, IncomingLen> EsbBuffer<OutgoingLen, IncomingLen>
where
    OutgoingLen: ArrayLength<u8>,
    IncomingLen: ArrayLength<u8>,
{
    #[allow(clippy::type_complexity)]
    pub fn try_split<Timer: ESBTimer>(
        &'static self, // TODO(AJM): This seems odd
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
            radio: ESBRadio::new(radio),
            state: State::Idle,
            addresses,
            attempts: 0,
        };

        irq.radio.init(max_payload_bytes, &irq.addresses);
        irq.timer.init();

        Ok((app, irq))
    }
}
