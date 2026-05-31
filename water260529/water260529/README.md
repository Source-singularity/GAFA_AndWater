# PlanB Breathing Controller

Standalone fallback firmware for the STM32F407 board.

This PlanB build does not read the LD2450 radar or wait for interaction input. After power-on it runs a calm breathing effect across the full installation:

- 38 front servos, matching the front position drawing.
- 14 side servos, matching the side position drawing.
- LED positions are tracked separately in `position_mapping.md`; the firmware carries the same front/side logic layout as the web preview.
- Yellow/purple 1 x 1 light boxes are wired by column and driven as addressable LED strips.
- Blue 1 x 1 panels are driven by separate column-group LED strip signals.
- Blue merged 2 x 2 strip blocks are driven by separate LED strip signals.

## Hardware

- STM32 board: `black_f407ze`
- I2C bus: `PB6` = SCL, `PB7` = SDA
- Servo drivers: four PCA9685 boards, 50 Hz
- Servo type: 360 degree continuous rotation, `1500us` stop
- PlanB servo motion: one-direction speed modulation only. The linkage converts rotation into back-and-forth motion.
- LED brightness range: `2` to `20`
- Firmware effect logic matches `front_breath_preview.html`; default no-interaction mode is `Breath`. `Ripple` and `Wander` are available in `src/main.cpp` through `kActivePattern`.
- LED color palette: yellow `#F1E88E`, purple `#A25DDB`, blue `#4696D8`; effects scale brightness only.

## PCA9685 Mapping

| Board | Address | Area | Channels |
|---|---:|---|---|
| 1 | `0x40` | Front boxes 1-16 | ch0-ch15 -> box 1-16 |
| 2 | `0x60` | Front boxes 17-32 | ch0-ch15 -> box 17-32 |
| 3 | `0x50` | Front boxes 33-38 | ch0-ch5 -> box 33-38 |
| 4 | `0x68` | Side boxes 1-14 | ch2 -> box 14, ch3-ch15 -> box 1-13 |

## LED Mapping

Front physical LED units:

- 6 strip brightness units, one for each merged 2 x 2 area.
- 42 panel brightness units, one for each remaining 1 x 1 area.
- 48 independently addressed front brightness units total.

Side preview LED units:

- 1 strip brightness unit for the merged 2 x 2 side area.
- 20 panel brightness units for the remaining 1 x 1 side areas.
- 21 independently addressed side brightness units total.

| Area | Current signal pins |
|---|---|
| Front yellow/purple light-box columns | `PE0 PE1 PE2 PE3 PE4 PE5 PE6 PE7 PE8 PE9 PE10` |
| Side yellow/purple light-box columns | `PD0 PD1 PD2 PD3` |
| Blue 1 x 1 panel column groups | `PD4 PD5 PD6 PD7 PD8 PD9` |
| Blue 2 x 2 strip blocks | `PD10 PD11 PD12 PD13 PD14 PD15 PC0` |

Detailed pin ownership lives in `position_mapping.md`.

LED segment sizes:

- Yellow/purple 1 x 1 light boxes: 256 LEDs per box.
- Blue 1 x 1 panels: 60 LEDs per panel.
- Blue 2 x 2 strip blocks: 120 LEDs per block.

## Tuning

The main tuning values are near the top of `src/main.cpp`:

- `kServoMinRunUs` and `kServoMaxRunUs`: servo angular speed range for the breathing cycle.
- `kBreathPeriodMs`: full breathing cycle time.
- `kLedMinBrightness` and `kLedMaxBrightness`: LED brightness limits, currently `2` to `20`.

The standalone WS2812B side LED test lives in `test/ws2812b_side_test.cpp`.
