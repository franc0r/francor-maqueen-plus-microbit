//! Print every NEC frame from the IR remote and map a few keys to motor commands.
//! Key codes are remote-specific; run once, read the codes off the log, then edit the match.
#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::Timer;
use francor_maqueen_plus::{Direction, Motor};
use francor_maqueen_plus_microbit::Board;
use {defmt_rtt as _, panic_probe as _};

// Codes of the DFRobot mini remote (command byte); adjust to your remote.
const KEY_UP: u8 = 0x16;
const KEY_DOWN: u8 = 0x1A;
const KEY_LEFT: u8 = 0x14;
const KEY_RIGHT: u8 = 0x18;
const KEY_OK: u8 = 0x15;

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
    info!("ir_remote: point the remote at the front of the robot");
    loop {
        let frame = board.ir.next_frame().await;
        info!(
            "IR address 0x{:x} command 0x{:x} repeat={}",
            frame.address, frame.command, frame.repeat
        );
        if frame.repeat {
            continue;
        }
        let r = match frame.command {
            KEY_UP => {
                board
                    .robot
                    .set_motor(Motor::Both, Direction::Forward, 120)
                    .await
            }
            KEY_DOWN => {
                board
                    .robot
                    .set_motor(Motor::Both, Direction::Backward, 120)
                    .await
            }
            KEY_LEFT => {
                let _ = board
                    .robot
                    .set_motor(Motor::Left, Direction::Backward, 100)
                    .await;
                board
                    .robot
                    .set_motor(Motor::Right, Direction::Forward, 100)
                    .await
            }
            KEY_RIGHT => {
                let _ = board
                    .robot
                    .set_motor(Motor::Left, Direction::Forward, 100)
                    .await;
                board
                    .robot
                    .set_motor(Motor::Right, Direction::Backward, 100)
                    .await
            }
            KEY_OK => board.robot.stop(Motor::Both).await,
            _ => Ok(()),
        };
        if let Err(e) = r {
            error!("motor: {:?}", e);
        }
    }
}
