# Position Mapping

This file records two separate mappings. The servo position maps below are the source of truth for physical positions.

- Servo maps: sparse grids. `-` means this physical grid cell has no servo box.
- LED panel maps: full grids. Every physical grid cell has an LED panel.

## Servo Position Maps

### Front Servo Map

The front servo view is represented as a 6 x 11 sparse grid.

| Row | C1 | C2 | C3 | C4 | C5 | C6 | C7 | C8 | C9 | C10 | C11 |
|----:|---:|---:|---:|---:|---:|---:|---:|---:|---:|----:|----:|
|  R1 |  1 |  2 |  3 |  4 |  5 |  6 |  7 |  8 |  9 |  10 |   - |
|  R2 | 11 | 12 | 13 | 14 |  - |  - |  - |  - | 15 |  16 |   - |
|  R3 | 17 | 18 | 19 | 20 |  - |  - |  - |  - | 21 |  22 |   - |
|  R4 |  - |  - |  - |  - |  - | 23 | 24 |  - |  - |  25 |   - |
|  R5 |  - |  - |  - |  - |  - | 26 | 27 |  - |  - |   - |   - |
|  R6 | 28 | 29 | 30 | 31 | 32 | 33 | 34 | 35 | 36 |  37 |  38 |

### Front Servo Columns

| Column | Box numbers from top to bottom, including sparse rows |
|-------:|---|
|     C1 | 1, 11, 17, -, -, 28 |
|     C2 | 2, 12, 18, -, -, 29 |
|     C3 | 3, 13, 19, -, -, 30 |
|     C4 | 4, 14, 20, -, -, 31 |
|     C5 | 5, -, -, -, -, 32 |
|     C6 | 6, -, -, 23, 26, 33 |
|     C7 | 7, -, -, 24, 27, 34 |
|     C8 | 8, -, -, -, -, 35 |
|     C9 | 9, 15, 21, -, -, 36 |
|    C10 | 10, 16, 22, 25, -, 37 |
|    C11 | -, -, -, -, -, 38 |

### Side Servo Map

The side servo view is represented as a 5 x 4 sparse grid.

| Row | C1 | C2 | C3 | C4 |
|----:|---:|---:|---:|---:|
|  R1 |  1 |  2 |  3 |  - |
|  R2 |  4 |  5 |  - |  - |
|  R3 |  - |  - |  6 |  7 |
|  R4 |  - |  8 |  9 | 10 |
|  R5 | 11 | 12 | 13 | 14 |

### Side Servo Columns

| Column | Box numbers from top to bottom, including sparse rows |
|-------:|---|
|     C1 | 1, 4, -, -, 11 |
|     C2 | 2, 5, -, 8, 12 |
|     C3 | 3, -, 6, 9, 13 |
|     C4 | -, -, 7, 10, 14 |

## LED Panel Maps

These maps use the edited servo-position grid as the physical reference, but every cell exists because every grid position has an LED panel.

### Front LED Panel Map

| Row | C1 | C2 | C3 | C4 | C5 | C6 | C7 | C8 | C9 | C10 | C11 |
|----:|---|---|---|---|---|---|---|---|---|---|---|
|  R1 | F-R1C1 | F-R1C2 | F-R1C3 | F-R1C4 | F-R1C5 | F-R1C6 | F-R1C7 | F-R1C8 | F-R1C9 | F-R1C10 | F-R1C11 |
|  R2 | F-R2C1 | F-R2C2 | F-R2C3 | F-R2C4 | F-R2C5 | F-R2C6 | F-R2C7 | F-R2C8 | F-R2C9 | F-R2C10 | F-R2C11 |
|  R3 | F-R3C1 | F-R3C2 | F-R3C3 | F-R3C4 | F-R3C5 | F-R3C6 | F-R3C7 | F-R3C8 | F-R3C9 | F-R3C10 | F-R3C11 |
|  R4 | F-R4C1 | F-R4C2 | F-R4C3 | F-R4C4 | F-R4C5 | F-R4C6 | F-R4C7 | F-R4C8 | F-R4C9 | F-R4C10 | F-R4C11 |
|  R5 | F-R5C1 | F-R5C2 | F-R5C3 | F-R5C4 | F-R5C5 | F-R5C6 | F-R5C7 | F-R5C8 | F-R5C9 | F-R5C10 | F-R5C11 |
|  R6 | F-R6C1 | F-R6C2 | F-R6C3 | F-R6C4 | F-R6C5 | F-R6C6 | F-R6C7 | F-R6C8 | F-R6C9 | F-R6C10 | F-R6C11 |

### Side LED Panel Map

| Row | C1 | C2 | C3 | C4 |
|----:|---|---|---|---|
|  R1 | S-R1C1 | S-R1C2 | S-R1C3 | S-R1C4 |
|  R2 | S-R2C1 | S-R2C2 | S-R2C3 | S-R2C4 |
|  R3 | S-R3C1 | S-R3C2 | S-R3C3 | S-R3C4 |
|  R4 | S-R4C1 | S-R4C2 | S-R4C3 | S-R4C4 |
|  R5 | S-R5C1 | S-R5C2 | S-R5C3 | S-R5C4 |

## Servo Driver Mapping

| Area | PCA9685 address | Channel mapping |
|---|---:|---|
| Front boxes 1-16 | `0x40` | ch0-ch15 -> front box 1-16 |
| Front boxes 17-32 | `0x60` | ch0-ch15 -> front box 17-32 |
| Front boxes 33-38 | `0x50` | ch0-ch5 -> front box 33-38 |
| Side boxes 1-14 | `0x68` | ch2 -> side box 14, ch3-ch15 -> side box 1-13 |

## LED Signal Mapping

The LED signal mapping is still pending the dedicated LED position drawing. The pins below are the current wiring assumption only; they are not a spatial-position source.

| Area | Columns | Pins |
|---|---:|---|
| Front | 1-10 | `PE0`, `PE1`, `PE2`, `PE3`, `PE4`, `PE5`, `PE6`, `PE7`, `PE8`, `PE9` |
| Side | 1-5 | `PD0`, `PD1`, `PD2`, `PD3`, `PD4` |
