//! Print wheel speed, light sensors and intersection state twice a second while driving slowly.
#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::Timer;
use francor_maqueen_plus::{Direction, Motor};
use francor_maqueen_plus_microbit::Board;
use {defmt_rtt as _, panic_probe as _};

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
    info!("telemetry: hold A to drive forward slowly");
    loop {
        if board.button_a.is_pressed() {
            let _ = board
                .robot
                .set_motor(Motor::Both, Direction::Forward, 60)
                .await;
        } else if let Err(e) = board.robot.stop(Motor::Both).await {
            error!("stop: {:?}", e);
        }
        match (
            board.robot.wheel_speed().await,
            board.robot.light_sensors().await,
            board.robot.intersection().await,
        ) {
            (Ok(w), Ok(l), Ok(i)) => info!(
                "wheels {} {} cm/s×5 | light L {} R {} | intersection {}",
                w.left, w.right, l.left, l.right, i
            ),
            _ => error!("telemetry read failed"),
        }
        Timer::after_millis(500).await;
    }
}
