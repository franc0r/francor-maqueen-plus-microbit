//! Built-in line following (runs inside the coprocessor). A starts, B stops.
//! Calibrate first: hold the Calc key on the robot ~2 s while it sits on the black line.
#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_futures::select::{Either, select};
use embassy_time::Timer;
use francor_maqueen_plus::{CrossroadMode, Intersection, PatrolSpeed, TJunctionMode};
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
    let _ = board.robot.set_patrol_speed(PatrolSpeed::S3).await;
    let _ = board
        .robot
        .set_crossroad_mode(CrossroadMode::Straight)
        .await;
    let _ = board.robot.set_t_junction_mode(TJunctionMode::Left).await;
    info!("line_patrol: A = start, B = stop");
    loop {
        board.button_a.wait_for_press().await;
        if let Err(e) = board.robot.set_patrolling(true).await {
            error!("start: {:?}", e);
            continue;
        }
        info!("patrolling");
        // Report intersections until B is pressed.
        loop {
            match select(board.button_b.wait_for_press(), Timer::after_millis(250)).await {
                Either::First(()) => break,
                Either::Second(()) => {
                    if let Ok(i) = board.robot.intersection().await
                        && i != Intersection::None
                    {
                        info!("intersection {}", i);
                    }
                }
            }
        }
        if let Err(e) = board.robot.set_patrolling(false).await {
            error!("stop: {:?}", e);
        }
        info!("stopped");
    }
}
