//! micro:bit buttons A and B (active low, pull-up).

use embassy_nrf::Peri;
use embassy_nrf::gpio::{Input, Pin, Pull};
use embassy_time::Timer;

/// One push button.
pub struct Button {
    pin: Input<'static>,
}

impl Button {
    /// Configure the pin as an input with pull-up.
    pub fn new(pin: Peri<'static, impl Pin>) -> Self {
        Button {
            pin: Input::new(pin, Pull::Up),
        }
    }

    /// Whether the button is currently held down.
    pub fn is_pressed(&self) -> bool {
        self.pin.is_low()
    }

    /// Wait for a press (falling edge), then 30 ms to swallow contact bounce.
    pub async fn wait_for_press(&mut self) {
        self.pin.wait_for_falling_edge().await;
        Timer::after_millis(30).await;
    }

    /// Wait for a release (rising edge), then 30 ms to swallow contact bounce.
    pub async fn wait_for_release(&mut self) {
        self.pin.wait_for_rising_edge().await;
        Timer::after_millis(30).await;
    }
}
