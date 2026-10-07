//! Colour cycle and rainbow on the four RGB LEDs. No robot init needed.
#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use francor_maqueen_plus::ws2812::Rgb;
use francor_maqueen_plus_microbit::Board;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut board = Board::new(p);
    board.neopixel.set_brightness(64);
    info!("neopixel demo");
    loop {
        for colour in [Rgb::RED, Rgb::GREEN, Rgb::BLUE, Rgb::WHITE] {
            board.neopixel.fill(colour).await;
            Timer::after_millis(400).await;
        }
        for i in 0..4 {
            board.neopixel.clear().await;
            board.neopixel.set(i, Rgb::YELLOW).await;
            Timer::after_millis(250).await;
        }
        for offset in (0..360).step_by(10) {
            board.neopixel.rainbow(offset, offset + 270).await;
            Timer::after_millis(50).await;
        }
    }
}
