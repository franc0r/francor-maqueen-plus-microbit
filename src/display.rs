//! The micro:bit V2's 5×5 LED matrix.
//!
//! The matrix is wired as five row drivers and five column sinks: an LED lights when
//! its row is high and its column is low. There is no hardware to do that for us, so
//! [`Display::run`] multiplexes the rows in software, one row every 2 ms (100 Hz for
//! the whole frame). Run it as its own task; everything else talks to the shared
//! frame through [`show`], [`set`] and [`clear`], which are lock-free and can be
//! called from anywhere.

use core::sync::atomic::{AtomicU32, Ordering};

use embassy_nrf::Peri;
use embassy_nrf::gpio::{Level, Output, OutputDrive, Pin};
use embassy_time::Timer;

/// Pixel coordinates: `frame[y * 5 + x]`, `0` = off, anything else = on, (0, 0) top left.
pub type Frame = [u8; 25];

/// The frame currently shown, one bit per pixel (bit `y * 5 + x`).
static FRAME: AtomicU32 = AtomicU32::new(0);

/// How long each row stays lit before the next one. 5 rows × 2 ms = 100 Hz.
const ROW_TIME_US: u64 = 2000;

/// Replace the whole frame.
pub fn show(frame: &Frame) {
    let mut bits = 0u32;
    for (i, px) in frame.iter().enumerate() {
        if *px != 0 {
            bits |= 1 << i;
        }
    }
    FRAME.store(bits, Ordering::Relaxed);
}

/// Switch every LED off.
pub fn clear() {
    FRAME.store(0, Ordering::Relaxed);
}

/// Switch one LED on or off. Coordinates outside 0..5 are ignored.
pub fn set(x: usize, y: usize, on: bool) {
    if x >= 5 || y >= 5 {
        return;
    }
    let bit = 1 << (y * 5 + x);
    if on {
        FRAME.fetch_or(bit, Ordering::Relaxed);
    } else {
        FRAME.fetch_and(!bit, Ordering::Relaxed);
    }
}

/// The frame currently shown.
pub fn frame() -> Frame {
    let bits = FRAME.load(Ordering::Relaxed);
    let mut f = [0u8; 25];
    for (i, px) in f.iter_mut().enumerate() {
        *px = ((bits >> i) & 1) as u8;
    }
    f
}

/// The matrix pins. Hand it to a task and call [`Display::run`].
pub struct Display {
    rows: [Output<'static>; 5],
    cols: [Output<'static>; 5],
}

impl Display {
    /// Take the ten matrix pins (rows 1–5, columns 1–5, in that order).
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        row1: Peri<'static, impl Pin>,
        row2: Peri<'static, impl Pin>,
        row3: Peri<'static, impl Pin>,
        row4: Peri<'static, impl Pin>,
        row5: Peri<'static, impl Pin>,
        col1: Peri<'static, impl Pin>,
        col2: Peri<'static, impl Pin>,
        col3: Peri<'static, impl Pin>,
        col4: Peri<'static, impl Pin>,
        col5: Peri<'static, impl Pin>,
    ) -> Self {
        fn out(pin: Peri<'static, impl Pin>, level: Level) -> Output<'static> {
            Output::new(pin, level, OutputDrive::Standard)
        }
        Display {
            rows: [
                out(row1, Level::Low),
                out(row2, Level::Low),
                out(row3, Level::Low),
                out(row4, Level::Low),
                out(row5, Level::Low),
            ],
            // Columns are sinks: high = LED off.
            cols: [
                out(col1, Level::High),
                out(col2, Level::High),
                out(col3, Level::High),
                out(col4, Level::High),
                out(col5, Level::High),
            ],
        }
    }

    /// Multiplex the rows forever. Spawn this as a task; it never returns.
    pub async fn run(mut self) -> ! {
        loop {
            for y in 0..5 {
                let bits = FRAME.load(Ordering::Relaxed);
                for (x, col) in self.cols.iter_mut().enumerate() {
                    if bits & (1 << (y * 5 + x)) != 0 {
                        col.set_low();
                    } else {
                        col.set_high();
                    }
                }
                self.rows[y].set_high();
                Timer::after_micros(ROW_TIME_US).await;
                self.rows[y].set_low();
            }
        }
    }
}
