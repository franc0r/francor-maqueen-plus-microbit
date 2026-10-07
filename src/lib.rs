//! micro:bit V2 support for the DFRobot Maqueen Plus V3.
//!
//! [`Board::new`] takes the embassy-nrf peripherals and returns ready-to-use
//! drivers for everything on the robot:
//!
//! | Field | Hardware | micro:bit pin | nRF52833 |
//! |---|---|---|---|
//! | `robot` | Maqueen coprocessor, I2C 0x10 | P19 SCL / P20 SDA | P0.26 / P1.00 |
//! | `lidar` | 8×8 matrix lidar, I2C 0x33 (DIP switches; see [`Board::with_lidar_address`]) | same bus | same |
//! | `neopixel` | 4× WS2812 | P1 | P0.03 (PWM0) |
//! | `ir` | NEC infrared receiver | P16 | P1.02 |
//! | `ultrasonic` | HC-SR04 TRIG / ECHO (legacy) | P13 / P14 | P0.17 / P0.01 |
//! | `button_a`, `button_b` | micro:bit buttons | — | P0.14 / P0.23 |
//! | `display` | 5×5 LED matrix | — | rows P0.21/22/15/24/19, cols P0.28/11/31, P1.05, P0.30 |
#![no_std]
#![warn(missing_docs)]

pub mod buttons;
pub mod display;
pub mod ir;
pub mod neopixel;
pub mod ultrasonic;

/// The hardware-agnostic driver crate, re-exported for convenience.
pub use francor_maqueen_plus as driver;

use embassy_embedded_hal::shared_bus::asynch::i2c::I2cDevice;
use embassy_nrf::twim::{self, Twim};
use embassy_nrf::{Peripherals, bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Delay;
use francor_maqueen_plus::lidar::LidarAddress;
use francor_maqueen_plus::{MaqueenPlusV3, MatrixLidar};
use static_cell::StaticCell;

pub use buttons::Button;
pub use display::Display;
pub use ir::IrReceiver;
pub use neopixel::Neopixel;
pub use ultrasonic::Ultrasonic;

bind_interrupts!(struct Irqs {
    TWISPI0 => twim::InterruptHandler<peripherals::TWISPI0>;
});

/// The external I2C bus on the edge connector, shared by robot and lidar.
pub type I2cBus = Mutex<NoopRawMutex, Twim<'static>>;
/// One handle onto the shared bus.
pub type SharedI2c = I2cDevice<'static, NoopRawMutex, Twim<'static>>;
/// Robot driver as configured by [`Board`].
pub type Robot = MaqueenPlusV3<SharedI2c, Delay>;
/// Lidar driver as configured by [`Board`].
pub type Lidar = MatrixLidar<SharedI2c, Delay>;

/// All Maqueen-related drivers on the micro:bit.
///
/// `robot` and `lidar` share one physical I2C bus (`TWISPI0`) through an
/// `embassy-embedded-hal` [`I2cDevice`], which locks the underlying `Twim` for the
/// duration of each individual `write`/`read` call. That per-transaction locking keeps
/// single I2C operations from interleaving, but each driver method here is `&mut self`
/// and runs several transactions in a row (for example `init`'s reset-then-poll, or a
/// lidar `send`/`recv` pair) — Rust's borrow checker already guarantees that no other
/// task can call a method on that *same* driver while one such multi-transaction
/// sequence is in flight, so the sequence cannot be interrupted by another task driving
/// the same sensor. Do not wrap `robot` or `lidar` in a `Mutex` and poll them from two
/// tasks: that would let two multi-transaction sequences on the same device interleave
/// on the bus and corrupt both.
pub struct Board {
    /// Robot coprocessor (motors, headlights, line sensors, patrol, PID).
    pub robot: Robot,
    /// Matrix lidar. Call `init` on it before use; it needs ~3 s after power-on.
    pub lidar: Lidar,
    /// The four RGB LEDs under the chassis.
    pub neopixel: Neopixel,
    /// Infrared remote receiver.
    pub ir: IrReceiver,
    /// Legacy ultrasonic sensor on P13/P14 (not fitted on a stock V3).
    pub ultrasonic: Ultrasonic,
    /// micro:bit button A.
    pub button_a: Button,
    /// micro:bit button B.
    pub button_b: Button,
    /// 5×5 LED matrix. Spawn [`Display::run`] in a task, then draw with [`display::show`].
    pub display: Display,
}

impl Board {
    /// Configure every peripheral the robot uses.
    ///
    /// The lidar is expected at [`LidarAddress::Addr33`], the DIP-switch setting the
    /// Maqueen Plus V3 ships with (the sensor's own factory default would be 0x30).
    /// Use [`Board::with_lidar_address`] if the switches on your sensor are set differently.
    ///
    /// Panics if called a second time (the I2C bus lives in a static cell).
    pub fn new(p: Peripherals) -> Board {
        Self::with_lidar_address(p, LidarAddress::Addr33)
    }

    /// Like [`Board::new`], with the lidar at `lidar_addr` (DIP switches on the sensor).
    ///
    /// Panics if called a second time (the I2C bus lives in a static cell).
    pub fn with_lidar_address(p: Peripherals, lidar_addr: LidarAddress) -> Board {
        static BUS: StaticCell<I2cBus> = StaticCell::new();
        static TX_BUF: StaticCell<[u8; 16]> = StaticCell::new();

        let mut config = twim::Config::default();
        config.frequency = twim::Frequency::K100;
        let twim = Twim::new(
            p.TWISPI0,
            Irqs,
            p.P1_00,
            p.P0_26,
            config,
            TX_BUF.init([0; 16]),
        );
        let bus: &'static I2cBus = BUS.init(Mutex::new(twim));

        Board {
            robot: MaqueenPlusV3::new(I2cDevice::new(bus), Delay),
            lidar: MatrixLidar::new(I2cDevice::new(bus), Delay, lidar_addr),
            neopixel: defmt::unwrap!(Neopixel::new(p.PWM0, p.P0_03)),
            ir: IrReceiver::new(p.P1_02),
            ultrasonic: Ultrasonic::new(p.P0_17, p.P0_01),
            button_a: Button::new(p.P0_14),
            button_b: Button::new(p.P0_23),
            display: Display::new(
                p.P0_21, p.P0_22, p.P0_15, p.P0_24, p.P0_19, // rows 1–5
                p.P0_28, p.P0_11, p.P0_31, p.P1_05, p.P0_30, // columns 1–5
            ),
        }
    }
}
