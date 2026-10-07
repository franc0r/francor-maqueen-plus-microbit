//! Cycle the eight headlight colours: left, then right, then both.
#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::Timer;
use francor_maqueen_plus::{CarLightColor, Side};
use francor_maqueen_plus_microbit::Board;
use {defmt_rtt as _, panic_probe as _};

const COLOURS: [CarLightColor; 8] = [
    CarLightColor::Red,
    CarLightColor::Green,
    CarLightColor::Yellow,
    CarLightColor::Blue,
    CarLightColor::Purple,
    CarLightColor::Cyan,
    CarLightColor::White,
    CarLightColor::Off,
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut board = Board::new(p);
    if let Err(e) = board.robot.init().await {
        error!("robot init failed: {:?}", e);
        loop {
            Timer::after_secs(1).await;
        }
    }
    loop {
        for side in [Side::Left, Side::Right, Side::Both] {
            for colour in COLOURS {
                info!("{:?} -> {:?}", side, colour);
                if let Err(e) = board.robot.set_headlight(side, colour).await {
                    error!("headlight: {:?}", e);
                }
                Timer::after_millis(400).await;
            }
        }
    }
}
