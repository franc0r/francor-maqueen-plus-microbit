//! LED matrix: a heart, then a running dot. Button A toggles between the two.
#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use francor_maqueen_plus_microbit::{Board, Display, display};
use {defmt_rtt as _, panic_probe as _};

#[rustfmt::skip]
const HEART: display::Frame = [
    0, 1, 0, 1, 0,
    1, 1, 1, 1, 1,
    1, 1, 1, 1, 1,
    0, 1, 1, 1, 0,
    0, 0, 1, 0, 0,
];

#[embassy_executor::task]
async fn matrix(d: Display) -> ! {
    d.run().await
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut board = Board::new(p);
    spawner.spawn(matrix(board.display).unwrap());
    info!("display: heart; press A for the running dot");
    loop {
        display::show(&HEART);
        board.button_a.wait_for_press().await;
        display::clear();
        let mut i = 0;
        loop {
            display::set((i + 24) % 25 % 5, (i + 24) % 25 / 5, false);
            display::set(i % 5, i / 5, true);
            i = (i + 1) % 25;
            Timer::after_millis(80).await;
            if board.button_a.is_pressed() {
                board.button_a.wait_for_release().await;
                break;
            }
        }
    }
}
