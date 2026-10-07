//! Print the digital line state and the five ADC values five times a second.
//! Use it to find your black/white threshold (white ≈ 3800, black ≈ 2700 on the workshop track).
#![no_std]
#![no_main]

use defmt::{error, info};
use embassy_executor::Spawner;
use embassy_time::Timer;
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
    loop {
        match (
            board.robot.line_state().await,
            board.robot.line_adc_all().await,
        ) {
            (Ok(s), Ok(a)) => info!(
                "L2 {} {} | L1 {} {} | M {} {} | R1 {} {} | R2 {} {}",
                s.l2() as u8,
                a.l2,
                s.l1() as u8,
                a.l1,
                s.m() as u8,
                a.m,
                s.r1() as u8,
                a.r1,
                s.r2() as u8,
                a.r2
            ),
            (Err(e), _) | (_, Err(e)) => error!("read failed: {:?}", e),
        }
        Timer::after_millis(200).await;
    }
}
