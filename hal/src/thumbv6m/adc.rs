//! Analogue-to-Digital Conversion
use crate::clock::GenericClockController;
#[allow(deprecated)]
use crate::gpio::v1;
use crate::gpio::v2::*;
use crate::hal::adc::{Channel, OneShot};
use crate::pac::{adc, ADC, PM};

/// Samples per reading
pub use adc::avgctrl::SAMPLENUM_A as SampleRate;
/// Clock frequency relative to the system clock
pub use adc::ctrlb::PRESCALER_A as Prescaler;
/// Reading resolution in bits
///
/// For the resolution of Arduino boards,
/// see the [analogueRead](https://www.arduino.cc/reference/en/language/functions/analog-io/analogread/) docs.
pub use adc::ctrlb::RESSEL_A as Resolution;
/// The gain level
pub use adc::inputctrl::GAIN_A as Gain;
/// Reference voltage (or its source)
pub use adc::refctrl::REFSEL_A as Reference;

/// An ADC where results are accessible via interrupt servicing.
pub struct InterruptAdc<ADC, C>
where
    C: ConversionMode<ADC>,
{
    adc: Adc<ADC>,
    m: core::marker::PhantomData<C>,
}

/// `Adc` encapsulates the device ADC
pub struct Adc<ADC> {
    adc: ADC,
}

/// Describes how an interrupt-driven ADC should finalize the peripheral
/// upon the completion of a conversion.
pub trait ConversionMode<ADC> {
    fn on_start(adc: &mut Adc<ADC>);
    fn on_complete(adc: &mut Adc<ADC>);
    fn on_stop(adc: &mut Adc<ADC>);
}

pub struct SingleConversion;
pub struct FreeRunning;

impl Adc<ADC> {
    /// Create a new `Adc` instance. The default configuration is:
    /// * 1/32 prescaler
    /// * 12 bit resolution
    /// * 1 sample
    /// * 1/2 gain
    /// * 1/2 VDDANA reference voltage
    pub fn adc(adc: ADC, pm: &mut PM, clocks: &mut GenericClockController) -> Self {
        pm.apbcmask.modify(|_, w| w.adc_().set_bit());

        // set to 1 / (1 / (48000000 / 32) * 6) = 250000 SPS
        let gclk0 = clocks.gclk0();
        clocks.adc(&gclk0).expect("adc clock setup failed");
        while adc.status.read().syncbusy().bit_is_set() {}

        adc.ctrla.modify(|_, w| w.swrst().set_bit());
        while adc.status.read().syncbusy().bit_is_set() {}

        adc.ctrlb.modify(|_, w| {
            w.prescaler().div32();
            w.ressel()._12bit()
        });
        while adc.status.read().syncbusy().bit_is_set() {}

        adc.inputctrl.modify(|_, w| w.muxneg().gnd()); // No negative input (internal gnd)
        while adc.status.read().syncbusy().bit_is_set() {}

        let mut newadc = Self { adc };
        newadc.sample_length(5);
        newadc.samples(adc::avgctrl::SAMPLENUM_A::_1);
        newadc.gain(adc::inputctrl::GAIN_A::DIV2);
        newadc.reference(adc::refctrl::REFSEL_A::INTVCC1);

        newadc
    }

    /// Set how many adc peripheral clock cycles (post-prescaler)
    /// are needed to take a  single sample. 
    pub fn sample_length(&mut self, length: u8) {
        self.adc.sampctrl.modify(|_, w| unsafe { w.samplen().bits(length) }); //sample length
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    /// Set the sample rate
    pub fn samples(&mut self, samples: SampleRate) {
        use adc::avgctrl::SAMPLENUM_A;
        self.adc.avgctrl.modify(|_, w| {
            w.samplenum().variant(samples);
            unsafe {
                // Table 32-3 (32.6.7) specifies the adjres
                // values necessary for each SAMPLENUM value.
                w.adjres().bits(match samples {
                    SAMPLENUM_A::_1 => 0,
                    SAMPLENUM_A::_2 => 1,
                    SAMPLENUM_A::_4 => 2,
                    SAMPLENUM_A::_8 => 3,
                    _ => 4,
                })
            }
        });
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    /// Set the gain factor
    pub fn gain(&mut self, gain: Gain) {
        self.adc.inputctrl.modify(|_, w| w.gain().variant(gain));
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    /// Set the voltage reference
    pub fn reference(&mut self, reference: Reference) {
        self.adc
            .refctrl
            .modify(|_, w| w.refsel().variant(reference));
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    /// Set the prescaler for adjusting the clock relative to the system clock
    pub fn prescaler(&mut self, prescaler: Prescaler) {
        self.adc
            .ctrlb
            .modify(|_, w| w.prescaler().variant(prescaler));
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    /// Set the input resolution.
    pub fn resolution(&mut self, resolution: Resolution) {
        self.adc.ctrlb.modify(|_, w| w.ressel().variant(resolution));
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    fn power_up(&mut self) {
        while self.adc.status.read().syncbusy().bit_is_set() {}
        self.adc.ctrla.modify(|_, w| w.enable().set_bit());
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    fn power_down(&mut self) {
        while self.adc.status.read().syncbusy().bit_is_set() {}
        self.adc.ctrla.modify(|_, w| w.enable().clear_bit());
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    #[inline(always)]
    fn start_conversion(&mut self) {
        // start conversion
        self.adc.swtrig.modify(|_, w| w.start().set_bit());
        // do it again because the datasheet tells us to
        self.adc.swtrig.modify(|_, w| w.start().set_bit());
    }

    fn enable_freerunning(&mut self) {
        self.adc.ctrlb.modify(|_, w| w.freerun().set_bit());
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    fn disable_freerunning(&mut self) {
        self.adc.ctrlb.modify(|_, w| w.freerun().set_bit());
        while self.adc.status.read().syncbusy().bit_is_set() {}
    }

    fn synchronous_convert(&mut self) -> u16 {
        self.start_conversion();
        while self.adc.intflag.read().resrdy().bit_is_clear() {}

        self.adc.result.read().result().bits()
    }

    /// Enables an interrupt when conversion is ready.
    fn enable_interrupts(&mut self) {
        self.adc.intflag.write(|w| w.resrdy().set_bit());
        self.adc.intenset.write(|w| w.resrdy().set_bit());
    }

    /// Disables the interrupt for when conversion is ready.
    fn disable_interrupts(&mut self) {
        self.adc.intenclr.write(|w| w.resrdy().set_bit());
    }

    fn service_interrupt_ready(&mut self) -> Option<u16> {
        if self.adc.intflag.read().resrdy().bit_is_set() {
            self.adc.intflag.write(|w| w.resrdy().set_bit());

            Some(self.adc.result.read().result().bits())
        } else {
            None
        }
    }

    /// Sets the mux to a particular pin. The pin mux is enabled-protected,
    /// so must be called while the peripheral is disabled.
    fn mux<PIN: Channel<ADC, ID=u8>>(&mut self, _pin: &mut PIN) {
        let chan = PIN::channel();
        while self.adc.status.read().syncbusy().bit_is_set() {}
        self.adc.inputctrl.modify(|_, w| unsafe { w.muxpos().bits(chan) });
    }
}

impl ConversionMode<ADC> for SingleConversion  {
    fn on_start(_adc: &mut Adc<ADC>) {
    }
    fn on_complete(adc: &mut Adc<ADC>) {
        adc.disable_interrupts();
        adc.power_down();
    }
    fn on_stop(_adc: &mut Adc<ADC>) {
    }
}

impl ConversionMode<ADC> for FreeRunning {
    fn on_start(adc: &mut Adc<ADC>) {
        adc.enable_freerunning();
    }
    fn on_complete(_adc: &mut Adc<ADC>) {
    }
    fn on_stop(adc: &mut Adc<ADC>) {
        adc.disable_interrupts();
        adc.power_down();
        adc.disable_freerunning();
    }
}

impl<C> InterruptAdc<ADC, C>
    where C: ConversionMode<ADC>
{
    pub fn service_interrupt_ready(&mut self) -> Option<u16> {
        if let Some(res) = self.adc.service_interrupt_ready() {
            C::on_complete(&mut self.adc);
            Some(res)
        } else {
            None
        }
    }

    /// Starts a conversion sampling the specified pin.
    pub fn start_conversion<PIN: Channel<ADC, ID=u8>>(&mut self, pin: &mut PIN) {
        self.adc.mux(pin);
        self.adc.power_up();
        C::on_start(&mut self.adc);
        self.adc.enable_interrupts();
        self.adc.start_conversion();
    }

    pub fn stop_conversion(&mut self) {
        C::on_stop(&mut self.adc);
    }
}

impl<C> From<Adc<ADC>> for InterruptAdc<ADC, C>
    where C: ConversionMode<ADC>
{
    fn from(adc: Adc<ADC>) -> Self {
        Self {
            adc,
            m: core::marker::PhantomData{},
        }
    }
}

impl<WORD, PIN> OneShot<ADC, WORD, PIN> for Adc<ADC>
where
    WORD: From<u16>,
    PIN: Channel<ADC, ID = u8>,
{
    type Error = ();

    fn read(&mut self, pin: &mut PIN) -> nb::Result<WORD, Self::Error> {
        self.mux(pin);
        self.power_up();
        let result = self.synchronous_convert();
        self.power_down();

        Ok(result.into())
    }
}

macro_rules! adc_pins {
    (
        $(
            $PinId:ident: $CHAN:literal
        ),+
    ) => {
        $(
            impl Channel<ADC> for Pin<$PinId, AlternateB> {
               type ID = u8;
               fn channel() -> u8 { $CHAN }
            }
        )+
    }
}

/// Implement [`Channel`] for [`v1::Pin`]s based on the implementations for
/// `v2` [`Pin`]s
#[allow(deprecated)]
impl<I> Channel<ADC> for v1::Pin<I, v1::PfB>
where
    I: PinId,
    Pin<I, AlternateB>: Channel<ADC, ID = u8>,
{
    type ID = u8;
    fn channel() -> u8 {
        Pin::<I, AlternateB>::channel()
    }
}

#[cfg(feature = "samd11")]
adc_pins! {
    PA02: 0,
    PA04: 2,
    PA05: 3,
    PA14: 6,
    PA15: 7
}

#[cfg(feature = "samd21")]
adc_pins! {
    PA02: 0,
    PA03: 1,
    PA04: 4,
    PA05: 5,
    PA06: 6,
    PA07: 7,
    PA08: 16,
    PA09: 17,
    PA10: 18,
    PA11: 19
}

#[cfg(feature = "min-samd21g")]
adc_pins! {
    PB02: 10,
    PB03: 11,
    PB08: 2,
    PB09: 3
}

#[cfg(feature = "min-samd21j")]
adc_pins! {
    PB00: 8,
    PB01: 9,
    PB04: 12,
    PB05: 13,
    PB06: 14,
    PB07: 15
}
