//! Drive a small square: forward, turn, repeat. Button A stops.
#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::Timer;
use francor_maqueen_plus::{Direction, Motor};
use francor_maqueen_plus_microbit::Board;
use {defmt_rtt as _, panic_probe as _};

async fn init_robot(board: &mut Board) {
    if let Err(e) = board.robot.init().await {
        error!("robot init failed: {:?}", e);
        loop {
            Timer::after_secs(1).await;
        }
    }
}

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut board = Board::new(p);
    init_robot(&mut board).await;
    info!("motors: square, press A to stop");
    'drive: loop {
        for _ in 0..4 {
            let _ = board
                .robot
                .set_motor(Motor::Both, Direction::Forward, 120)
                .await;
            if let Either::First(()) =
                select(board.button_a.wait_for_press(), Timer::after_millis(1000)).await
            {
                break 'drive;
            }
            let _ = board.robot.stop(Motor::Both).await;
            Timer::after_millis(200).await;
            // spin right: left forward, right backward
            let _ = board
                .robot
                .set_motor(Motor::Left, Direction::Forward, 100)
                .await;
            let _ = board
                .robot
                .set_motor(Motor::Right, Direction::Backward, 100)
                .await;
            // 450 ms: tune for 90° on your floor.
            if let Either::First(()) =
                select(board.button_a.wait_for_press(), Timer::after_millis(450)).await
            {
                break 'drive;
            }
            let _ = board.robot.stop(Motor::Both).await;
            Timer::after_millis(200).await;
        }
    }
    if let Err(e) = board.robot.stop(Motor::Both).await {
        error!("stop: {:?}", e);
    }
    info!("stopped");
    loop {
        Timer::after_secs(1).await;
    }
}
