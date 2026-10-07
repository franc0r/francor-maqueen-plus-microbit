//! Pre-workshop hardware check: exercises every actuator briefly and prints every sensor once.
//!
//! Expected log (values vary):
//! ```text
//! hardware_check: waiting for the Maqueen coprocessor
//! firmware version: V3.x
//! headlights: red / green / blue / off
//! motors: forward 300 ms, backward 300 ms
//! line state: L2=0 L1=0 M=1 R1=0 R2=0  adc: 3801 3790 2710 3788 3805
//! light: left=512 right=498  wheel speed: 0 0 cm/s×5  intersection: 0
//! neopixel: rainbow
//! lidar: init ok, 8x8 grid in mm (4000 = no return), y=0 is the top row
//! y=0: 4000 4000 4000 1979 1965 1933 1992 4000
//! ...
//! y=7:  194  209  212  215  216  211  209 4000
//! lidar: nearest 194 mm at x=0 y=7
//! ultrasonic: none (not fitted)
//! hardware_check done
//! ```
#![no_std]
#![no_main]

use defmt::{error, info, warn};
use embassy_executor::Spawner;
use embassy_time::Timer;
use francor_maqueen_plus::lidar::LidarMode;
use francor_maqueen_plus::{CarLightColor, Direction, Motor, Side};
use francor_maqueen_plus_microbit::Board;
use {defmt_rtt as _, panic_probe as _};

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());
    let mut board = Board::new(p);

    info!("hardware_check: waiting for the Maqueen coprocessor");
    if let Err(e) = board.robot.init().await {
        error!(
            "robot init failed: {:?} — is the Maqueen switched on and the micro:bit seated?",
            e
        );
        loop {
            Timer::after_secs(1).await;
        }
    }
    let mut buf = [0u8; 32];
    match board.robot.version(&mut buf).await {
        Ok(v) => info!("firmware version: {}", v),
        Err(e) => error!("version read failed: {:?}", e),
    }

    // Headlights
    for (name, colour) in [
        ("red", CarLightColor::Red),
        ("green", CarLightColor::Green),
        ("blue", CarLightColor::Blue),
        ("off", CarLightColor::Off),
    ] {
        if let Err(e) = board.robot.set_headlight(Side::Both, colour).await {
            error!("headlight {}: {:?}", name, e);
        }
        Timer::after_millis(300).await;
    }
    info!("headlights: red / green / blue / off");

    // Motors
    if let Err(e) = board
        .robot
        .set_motor(Motor::Both, Direction::Forward, 80)
        .await
    {
        error!("motor forward: {:?}", e);
    }
    Timer::after_millis(300).await;
    if let Err(e) = board
        .robot
        .set_motor(Motor::Both, Direction::Backward, 80)
        .await
    {
        error!("motor backward: {:?}", e);
    }
    Timer::after_millis(300).await;
    if let Err(e) = board.robot.stop(Motor::Both).await {
        error!("motor stop: {:?}", e);
    }
    info!("motors: forward 300 ms, backward 300 ms");

    // Line sensors
    match (
        board.robot.line_state().await,
        board.robot.line_adc_all().await,
    ) {
        (Ok(s), Ok(a)) => info!(
            "line state: L2={} L1={} M={} R1={} R2={}  adc: {} {} {} {} {}",
            s.l2() as u8,
            s.l1() as u8,
            s.m() as u8,
            s.r1() as u8,
            s.r2() as u8,
            a.l2,
            a.l1,
            a.m,
            a.r1,
            a.r2
        ),
        (Err(e), _) | (_, Err(e)) => error!("line sensors: {:?}", e),
    }

    // Telemetry
    match (
        board.robot.light_sensors().await,
        board.robot.wheel_speed().await,
        board.robot.intersection().await,
    ) {
        (Ok(l), Ok(w), Ok(i)) => info!(
            "light: left={} right={}  wheel speed: {} {} cm/s×5  intersection: {}",
            l.left, l.right, w.left, w.right, i
        ),
        _ => error!("telemetry read failed"),
    }

    // Neopixel
    board.neopixel.rainbow(0, 270).await;
    info!("neopixel: rainbow");

    // Lidar: full 8x8 grid so a hand or box in front of the robot is visible.
    match board.lidar.init(LidarMode::Matrix8x8).await {
        Ok(()) => {
            // The first frame after a mode switch can be garbage; read and discard it.
            let _ = board.lidar.point(3, 3).await;
            info!("lidar: init ok, 8x8 grid in mm (4000 = no return), y=0 is the top row");
            let mut nearest = (u16::MAX, 0u8, 0u8);
            for y in 0..8u8 {
                let mut row = [u16::MAX; 8];
                for (x, cell) in row.iter_mut().enumerate() {
                    match board.lidar.point(x as u8, y).await {
                        Ok(mm) => {
                            *cell = mm;
                            if mm < nearest.0 {
                                nearest = (mm, x as u8, y);
                            }
                        }
                        Err(e) => error!("lidar point {},{}: {:?}", x, y, e),
                    }
                }
                info!(
                    "y={}: {} {} {} {} {} {} {} {}",
                    y, row[0], row[1], row[2], row[3], row[4], row[5], row[6], row[7]
                );
            }
            if nearest.0 != u16::MAX {
                info!(
                    "lidar: nearest {} mm at x={} y={}",
                    nearest.0, nearest.1, nearest.2
                );
            }
        }
        Err(e) => warn!(
            "lidar: init failed ({:?}) — wait 3 s after power-on, check the cable",
            e
        ),
    }

    // Ultrasonic (optional hardware)
    match board.ultrasonic.distance_cm().await {
        Some(cm) => info!("ultrasonic: {} cm", cm),
        None => info!("ultrasonic: none (not fitted)"),
    }

    board.neopixel.clear().await;
    info!("hardware_check done");
    loop {
        Timer::after_secs(1).await;
    }
}
