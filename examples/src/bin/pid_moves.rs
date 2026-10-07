//! Encoder-based PID moves: 20 cm forward, 90° right, 20 cm forward, 90° left, 20 cm backward.
//! The pattern does not return to the start: it ends 20 cm to the side of the start
//! point, facing the original heading.
#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::Timer;
use francor_maqueen_plus::{Direction, Wait};
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
    info!("pid_moves: press A to run the pattern");
    loop {
        board.button_a.wait_for_press().await;
        info!("forward 20 cm");
        report(
            board
                .robot
                .pid_distance(Direction::Forward, 20, Wait::UntilDone)
                .await,
        );
        info!("turn +90");
        report(board.robot.pid_angle(90, Wait::UntilDone).await);
        info!("forward 20 cm");
        report(
            board
                .robot
                .pid_distance(Direction::Forward, 20, Wait::UntilDone)
                .await,
        );
        info!("turn -90");
        report(board.robot.pid_angle(-90, Wait::UntilDone).await);
        info!("backward 20 cm");
        report(
            board
                .robot
                .pid_distance(Direction::Backward, 20, Wait::Return)
                .await,
        );
        // Poll while the move runs so the wheel-speed readout is meaningful (it would
        // read 0 once the move has already finished).
        let mut polls = 0u32;
        loop {
            Timer::after_millis(100).await;
            match board.robot.wheel_speed().await {
                Ok(w) => info!("wheel speed: {} / {} (raw, cm/s = raw/5)", w.left, w.right),
                Err(e) => error!("wheel speed: {:?}", e),
            }
            polls += 1;
            match board.robot.pid_busy().await {
                Ok(false) => break,
                Ok(true) => {
                    if polls >= 50 {
                        error!("pid_moves: backward move still busy after 50 polls, giving up");
                        break;
                    }
                }
                Err(e) => {
                    error!("pid busy: {:?}", e);
                    break;
                }
            }
        }
        info!("done");
    }
}

fn report<E: defmt::Format>(r: core::result::Result<(), francor_maqueen_plus::Error<E>>) {
    if let Err(e) = r {
        error!("pid move failed: {:?}", e);
    }
}
