//! Distance readout for a retrofitted HC-SR04 on P13/P14.
#![no_std]
#![no_main]

use defmt::info;
use embassy_executor::Spawner;
use embassy_time::Timer;
use francor_maqueen_plus_microbit::Board;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut board = Board::new(p);
    info!("ultrasonic demo (no robot init needed)");
    loop {
        match board.ultrasonic.distance_cm().await {
            Some(cm) => info!("distance: {} cm", cm),
            None => info!("distance: no echo"),
        }
        Timer::after_millis(200).await;
    }
}
