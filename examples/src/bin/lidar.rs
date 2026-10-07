//! Matrix lidar: print an 8×8 distance grid on A, then drive with obstacle avoidance on B.
//! A failed point read in the 8×8 scan is logged once per row and stored as `u16::MAX`
//! (65535) in that row's printed grid.
#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::Timer;
use francor_maqueen_plus::lidar::{LidarMode, Suggestion};
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
    info!("lidar: waiting 3 s for the sensor to boot");
    Timer::after_secs(3).await;
    info!("lidar: A = 8x8 scan, B = obstacle avoidance until A");
    loop {
        if board.button_a.is_pressed() {
            if let Err(e) = board.lidar.init(LidarMode::Matrix8x8).await {
                error!("lidar init: {:?}", e);
                Timer::after_secs(1).await;
                continue;
            }
            for y in 0..8u8 {
                let mut row = [0u16; 8];
                let mut row_failed = false;
                for (x, cell) in row.iter_mut().enumerate() {
                    *cell = match board.lidar.point(x as u8, y).await {
                        Ok(mm) => mm,
                        Err(_) => {
                            row_failed = true;
                            u16::MAX
                        }
                    };
                }
                if row_failed {
                    error!(
                        "row {}: one or more points failed, shown as {}",
                        y,
                        u16::MAX
                    );
                }
                info!(
                    "row {}: {} {} {} {} {} {} {} {}",
                    y, row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7]
                );
            }
            Timer::after_millis(500).await;
        } else if board.button_b.is_pressed() {
            if let Err(e) = board.lidar.init(LidarMode::ObstacleAvoidance4x4).await {
                error!("lidar init: {:?}", e);
                Timer::after_secs(1).await;
                continue;
            }
            let _ = board.lidar.set_obstacle_distance(200).await;
            board.lidar.set_timeout_ms(500);
            info!("obstacle avoidance running, press A to stop");
            while !board.button_a.is_pressed() {
                match board.lidar.obstacle_data().await {
                    Ok(d) => {
                        info!(
                            "L {} F {} R {} mm -> {:?} emergency={}",
                            d.left_mm, d.front_mm, d.right_mm, d.suggestion, d.emergency
                        );
                        let _ = match (d.emergency, d.suggestion) {
                            (true, _) => board.robot.stop(Motor::Both).await,
                            (_, Suggestion::Front) => {
                                board
                                    .robot
                                    .set_motor(Motor::Both, Direction::Forward, 80)
                                    .await
                            }
                            (_, Suggestion::Left) => {
                                let _ = board
                                    .robot
                                    .set_motor(Motor::Left, Direction::Backward, 70)
                                    .await;
                                board
                                    .robot
                                    .set_motor(Motor::Right, Direction::Forward, 70)
                                    .await
                            }
                            (_, Suggestion::Right) => {
                                let _ = board
                                    .robot
                                    .set_motor(Motor::Left, Direction::Forward, 70)
                                    .await;
                                board
                                    .robot
                                    .set_motor(Motor::Right, Direction::Backward, 70)
                                    .await
                            }
                            (_, Suggestion::Unknown(_)) => board.robot.stop(Motor::Both).await,
                        };
                    }
                    Err(e) => {
                        error!("obstacle data: {:?}", e);
                        // Do not keep driving blind on a read failure.
                        if let Err(e) = board.robot.stop(Motor::Both).await {
                            error!("stop: {:?}", e);
                        }
                    }
                }
                Timer::after_millis(100).await;
            }
            if let Err(e) = board.robot.stop(Motor::Both).await {
                error!("stop: {:?}", e);
            }
            info!("stopped");
        }
        Timer::after_millis(50).await;
    }
}
