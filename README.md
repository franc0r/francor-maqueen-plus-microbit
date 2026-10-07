# francor-maqueen-plus-microbit

BBC micro:bit V2 (nRF52833) support for the DFRobot Maqueen Plus V3, built on
[embassy-nrf](https://github.com/embassy-rs/embassy) and the hardware-agnostic
[francor-maqueen-plus](https://github.com/franc0r/francor-maqueen-plus) driver
(re-exported as `francor_maqueen_plus_microbit::driver`).

`Board::new` takes the embassy-nrf peripherals and returns ready-to-use drivers:

| Field | Hardware | micro:bit pin | nRF52833 |
|---|---|---|---|
| `robot` | Maqueen coprocessor, I2C 0x10 | P19 SCL / P20 SDA | P0.26 / P1.00 |
| `lidar` | 8×8 matrix lidar, I2C 0x33 (DIP switches; see [`Board::with_lidar_address`]) | same bus | same |
| `neopixel` | 4× WS2812 | P1 | P0.03 (PWM0) |
| `ir` | NEC infrared receiver | P16 | P1.02 |
| `ultrasonic` | HC-SR04 TRIG / ECHO (legacy) | P13 / P14 | P0.17 / P0.01 |
| `button_a`, `button_b` | micro:bit buttons | — | P0.14 / P0.23 |
| `display` | 5×5 LED matrix | — | rows P0.21/22/15/24/19, cols P0.28/11/31, P1.05, P0.30 |

## Examples

The `examples/` package holds one binary per feature (`motors`, `lidar`, `ir_remote`,
`line_patrol`, `hardware_check`, …). Flash one with [probe-rs](https://probe.rs) and stream
the defmt logs:

```bash
cargo run --release -p francor-maqueen-plus-microbit-examples --bin hardware_check
```
