//! NEC infrared receiver on P16.
//!
//! The receiver output idles high and goes low while the 38 kHz carrier is
//! present. This module measures the time between edges with `embassy_time`
//! (32 768 Hz tick, ~30 µs resolution — fine for NEC's 562 µs unit) and feeds
//! the pure decoder in the driver crate.

use embassy_nrf::Peri;
use embassy_nrf::gpio::{Input, Pin, Pull};
use embassy_time::{Duration, Instant, with_timeout};
use francor_maqueen_plus::nec::{Decoder, NecFrame, PinLevel};

/// Infrared receiver.
pub struct IrReceiver {
    pin: Input<'static>,
    decoder: Decoder,
}

impl IrReceiver {
    /// Configure the receiver pin with a pull-up.
    pub fn new(pin: Peri<'static, impl Pin>) -> Self {
        IrReceiver {
            pin: Input::new(pin, Pull::Up),
            decoder: Decoder::new(),
        }
    }

    /// Wait for the next complete NEC frame. Noise and partial frames are
    /// discarded silently; after 20 ms without an edge the decoder resets.
    pub async fn next_frame(&mut self) -> NecFrame {
        const SILENCE: Duration = Duration::from_millis(20);
        let mut level_start = Instant::now();
        let mut high = self.pin.is_high();
        loop {
            match with_timeout(SILENCE, self.pin.wait_for_any_edge()).await {
                Ok(()) => {
                    let now = Instant::now();
                    let ended = if high { PinLevel::High } else { PinLevel::Low };
                    let elapsed = (now - level_start).as_micros() as u32;
                    level_start = now;
                    high = !high;
                    if let Some(frame) = self.decoder.feed(ended, elapsed) {
                        return frame;
                    }
                }
                Err(_) => {
                    self.decoder.reset();
                    level_start = Instant::now();
                    high = self.pin.is_high();
                }
            }
        }
    }
}
