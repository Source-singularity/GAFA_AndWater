# PlanB Breathing Controller

Standalone fallback firmware for the STM32F407 board.

This PlanB build does not read the LD2450 radar or wait for interaction input. After power-on it runs a calm breathing effect across the full installation:

- 38 front servos, matching the front position drawing.
- 14 side servos, matching the side position drawing.
- LED positions are tracked separately in `position_mapping.md`; the current code still assumes grouped LED signal pins until the LED position drawing is added.

## Hardware

- STM32 board: `black_f407ze`
- I2C bus: `PB6` = SCL, `PB7` = SDA
- Servo drivers: four PCA9685 boards, 50 Hz
- Servo type: 360 degree continuous rotation, `1500us` stop
- PlanB servo motion: one-direction speed modulation only. The linkage converts rotation into back-and-forth motion.
- LED brightness range: `2` to `20`

## PCA9685 Mapping

| Board | Address | Area | Channels |
|---|---:|---|---|
| 1 | `0x40` | Front boxes 1-16 | ch0-ch15 -> box 1-16 |
| 2 | `0x60` | Front boxes 17-32 | ch0-ch15 -> box 17-32 |
| 3 | `0x50` | Front boxes 33-38 | ch0-ch5 -> box 33-38 |
| 4 | `0x68` | Side boxes 1-14 | ch2 -> box 14, ch3-ch15 -> box 1-13 |

## LED Mapping

| Area | Current signal pins |
|---|---|
| Front | `PE0 PE1 PE2 PE3 PE4 PE5 PE6 PE7 PE8 PE9` |
| Side | `PD0 PD1 PD2 PD3 PD4` |

## Tuning

The main tuning values are near the top of `src/main.cpp`:

- `kServoMinRunUs` and `kServoMaxRunUs`: servo angular speed range for the breathing cycle.
- `kBreathPeriodMs`: full breathing cycle time.
- `kLedMinBrightness` and `kLedMaxBrightness`: LED brightness limits, currently `2` to `20`.

If the LED boards are addressable LED strips instead of direct PWM inputs, replace `writeLedChannel()` in `src/main.cpp`.
