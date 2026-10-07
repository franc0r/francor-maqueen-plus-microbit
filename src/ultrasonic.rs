//! Legacy HC-SR04 / URM10 ultrasonic sensor on P13 (TRIG) and P14 (ECHO).
//!
//! A stock Maqueen Plus V3 ships with the matrix lidar instead, but the MakeCode
//! extension keeps the block, so the sensor is supported for retrofitted robots.

use embassy_nrf::Peri;
use embassy_nrf::gpio::{Input, Level, Output, OutputDrive, Pin, Pull};
use embassy_time::{Duration, Instant, with_timeout};

/// Ultrasonic distance sensor.
pub struct Ultrasonic {
    trig: Output<'static>,
    echo: Input<'static>,
}

impl Ultrasonic {
    /// Configure the trigger output and echo input.
    pub fn new(trig: Peri<'static, impl Pin>, echo: Peri<'static, impl Pin>) -> Self {
        Ultrasonic {
            trig: Output::new(trig, Level::Low, OutputDrive::Standard),
            echo: Input::new(echo, Pull::None),
        }
    }

    /// Distance in cm, clamped to 300, or `None` when no echo arrives.
    /// Retries once, like the MakeCode block.
    pub async fn distance_cm(&mut self) -> Option<u16> {
        for _ in 0..2 {
            if let Some(cm) = self.measure().await {
                return Some(cm);
            }
        }
        None
    }

    async fn measure(&mut self) -> Option<u16> {
        // 300 cm round trip at the extension's 59.259 µs/cm ≈ 18 ms; the block waits 58 ms.
        const ECHO_TIMEOUT: Duration = Duration::from_millis(58);
        self.trig.set_high();
        cortex_m::asm::delay(64 * 10); // 10 µs at 64 MHz
        self.trig.set_low();
        with_timeout(ECHO_TIMEOUT, self.echo.wait_for_high())
            .await
            .ok()?;
        let start = Instant::now();
        with_timeout(ECHO_TIMEOUT, self.echo.wait_for_low())
            .await
            .ok()?;
        let us = start.elapsed().as_micros() as u32;
        let cm = us * 1000 / 59_259;
        if cm == 0 {
            None
        } else {
            Some(cm.min(300) as u16)
        }
    }
}
