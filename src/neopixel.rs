//! Four WS2812 RGB LEDs on P1 (P0.03; the Plus V2 used P15), driven by the nRF PWM sequence engine.
//!
//! PWM0 runs at 16 MHz with a period of 20 ticks (1.25 µs). Each colour bit is
//! one PWM period; the duty words come from [`francor_maqueen_plus::ws2812::encode`].

use embassy_nrf::Peri;
use embassy_nrf::gpio::Pin;
use embassy_nrf::pwm::{
    self, Config, Prescaler, SequenceConfig, SequenceLoad, SequencePwm, SingleSequenceMode,
    SingleSequencer,
};
use embassy_time::Timer;
use francor_maqueen_plus::ws2812::{Rgb, buffer_len, encode, rainbow};

/// Number of LEDs on the Maqueen Plus V3.
pub const LED_COUNT: usize = 4;
const WORDS: usize = buffer_len(LED_COUNT);

/// The RGB LED strip.
pub struct Neopixel {
    pwm: SequencePwm<'static>,
    words: [u16; WORDS],
    leds: [Rgb; LED_COUNT],
    brightness: u8,
}

impl Neopixel {
    /// Configure a PWM instance and its output pin.
    pub fn new(
        pwm: Peri<'static, impl pwm::Instance>,
        pin: Peri<'static, impl Pin>,
    ) -> Result<Self, pwm::Error> {
        let mut config = Config::default();
        config.sequence_load = SequenceLoad::Common;
        config.prescaler = Prescaler::Div1;
        config.max_duty = 20;
        let pwm = SequencePwm::new_1ch(pwm, pin, config)?;
        Ok(Neopixel {
            pwm,
            words: [0; WORDS],
            leds: [Rgb::BLACK; LED_COUNT],
            brightness: 255,
        })
    }

    /// Global brightness 0..255 applied on the next update (default 255).
    pub fn set_brightness(&mut self, brightness: u8) {
        self.brightness = brightness;
    }

    /// Current colours (before brightness scaling).
    pub fn leds(&self) -> [Rgb; LED_COUNT] {
        self.leds
    }

    /// Set one LED (0..=3) and update the strip. Out-of-range indices are ignored.
    pub async fn set(&mut self, index: usize, colour: Rgb) {
        if let Some(slot) = self.leds.get_mut(index) {
            *slot = colour;
        }
        self.flush().await;
    }

    /// Set all LEDs to one colour and update the strip.
    pub async fn fill(&mut self, colour: Rgb) {
        self.write([colour; LED_COUNT]).await;
    }

    /// Switch all LEDs off.
    pub async fn clear(&mut self) {
        self.fill(Rgb::BLACK).await;
    }

    /// Show a hue gradient from `start_hue` to `end_hue` (degrees) across the strip.
    pub async fn rainbow(&mut self, start_hue: u16, end_hue: u16) {
        self.write(rainbow(start_hue, end_hue)).await;
    }

    /// Set all four colours and update the strip.
    pub async fn write(&mut self, leds: [Rgb; LED_COUNT]) {
        self.leds = leds;
        self.flush().await;
    }

    async fn flush(&mut self) {
        // The buffer is sized exactly, so encode cannot fail.
        let _ = encode(&self.leds, self.brightness, &mut self.words);
        let mut seq_config = SequenceConfig::default();
        seq_config.end_delay = 799; // keep the line low after the last word (latch)
        let sequencer = SingleSequencer::new(&mut self.pwm, &self.words, seq_config);
        if sequencer.start(SingleSequenceMode::Times(1)).is_err() {
            return;
        }
        // 97 words × 1.25 µs ≈ 121 µs plus the end delay. Dropping the sequencer
        // stops the PWM, so wait until the sequence has certainly finished.
        Timer::after_millis(2).await;
    }
}
