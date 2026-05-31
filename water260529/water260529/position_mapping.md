# Position Mapping

This file records the box number to physical position mapping from the two reference drawings.

## Front View

The front view is represented as a 6 x 11 sparse grid. `-` means the grid position is physically empty or has no servo box. Uppercase letters mark blue merged 2 x 2 LED strip blocks; lowercase letters mark blue 1 x 1 LED panels.

| Row | C1 | C2 | C3 | C4 | C5 | C6 | C7 | C8 | C9 | C10 | C11 |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| R1 | 1 | 2 | 3 | 4 | 5 | 6 | 7 | 8 | a | 9 | 10 |
| R2 | 11 | 12 | 13 | 14 | C | C | B | B | b | 15 | 16 |
| R3 | 17 | 18 | 19 | 20 | C | C | B | B | c | 21 | 22 |
| R4 | F | F | E | E | D | D | 23 | 24 | A | A | d |
| R5 | F | F | E | E | D | D | 26 | 27 | A | A | 25 |
| R6 | 28 | 29 | 30 | 31 | 32 | 33 | 34 | 35 | 36 | 37 | 38 |

### Front Columns

| Column | Box numbers from top to bottom, including sparse rows |
|---:|---|
| C1 | 1, 11, 17, F, F, 28 |
| C2 | 2, 12, 18, F, F, 29 |
| C3 | 3, 13, 19, E, E, 30 |
| C4 | 4, 14, 20, E, E, 31 |
| C5 | 5, C, C, D, D, 32 |
| C6 | 6, C, C, D, D, 33 |
| C7 | 7, B, B, 23, 26, 34 |
| C8 | 8, B, B, 24, 27, 35 |
| C9 | a, b, c, A, A, 36 |
| C10 | 9, 15, 21, A, A, 37 |
| C11 | 10, 16, 22, d, 25, 38 |

### Front Merged LED Blocks

The front preview has these merged 2 x 2 LED blocks. Coordinates are 1-based `(x, y)`.

| Block | Covered coordinates |
|---:|---|
| A | (9,4), (10,4), (9,5), (10,5) |
| B | (7,2), (8,2), (7,3), (8,3) |
| C | (5,2), (6,2), (5,3), (6,3) |
| D | (5,4), (6,4), (5,5), (6,5) |
| E | (3,4), (4,4), (3,5), (4,5) |
| F | (1,4), (2,4), (1,5), (2,5) |

### Front Named Blue LED Panels

These are blue 1 x 1 LED panels. Coordinates are 1-based `(x, y)`.

| Panel | Coordinate |
|---:|---|
| a | (9,1) |
| b | (9,2) |
| c | (9,3) |
| d | (11,4) |

Front LED unit count:

- 6 strip units, one per merged 2 x 2 block.
- 42 panel units, one per remaining 1 x 1 grid position.
- 48 independently addressed front brightness units total.

## Side View

The side view is represented as a 6 x 4 sparse grid. `-` means the grid position is physically empty or has no servo box. Uppercase letters mark blue merged 2 x 2 LED strip blocks; lowercase letters mark blue 1 x 1 LED panels.

| Row | C1 | C2 | C3 | C4 |
|---:|---:|---:|---:|---:|
| R1 | 1 | 2 | 3 | e |
| R2 | G | G | f | g |
| R3 | G | G | 4 | 5 |
| R4 | h | i | 6 | 7 |
| R5 | j | 8 | 9 | 10 |
| R6 | 11 | 12 | 13 | 14 |

### Side Columns

| Column | Box numbers from top to bottom, including sparse rows |
|---:|---|
| C1 | 1, G, G, h, j, 11 |
| C2 | 2, G, G, i, 8, 12 |
| C3 | 3, f, 4, 6, 9, 13 |
| C4 | e, g, 5, 7, 10, 14 |

### Side Merged LED Blocks

The side preview has these merged 2 x 2 LED blocks. Coordinates are 1-based `(x, y)`.

| Block | Covered coordinates |
|---:|---|
| G | (1,2), (2,2), (1,3), (2,3) |

### Side Named Blue LED Panels

These are blue 1 x 1 LED panels. Coordinates are 1-based `(x, y)`.

| Panel | Coordinate |
|---:|---|
| e | (4,1) |
| f | (3,2) |
| g | (4,2) |
| h | (1,4) |
| i | (2,4) |
| j | (1,5) |

Side LED unit count:

- 1 strip unit, one per merged 2 x 2 block.
- 20 panel units, one per remaining 1 x 1 grid position.
- 21 independently addressed side brightness units total.

## Servo Driver Mapping

| Area | PCA9685 address | Channel mapping |
|---|---:|---|
| Front boxes 1-16 | `0x40` | ch0-ch15 -> front box 1-16 |
| Front boxes 17-32 | `0x60` | ch0-ch15 -> front box 17-32 |
| Front boxes 33-38 | `0x50` | ch0-ch5 -> front box 33-38 |
| Side boxes 1-14 | `0x68` | ch2 -> side box 14, ch3-ch15 -> side box 1-13 |

## LED Signal Pin Mapping

The LED hardware is split into three signal groups:

- Yellow/purple 1 x 1 light-box panels: one data signal per column.
- Blue 1 x 1 LED panels: one data signal per blue-panel column group.
- Blue 2 x 2 LED strip blocks: one data signal per named block.

### Yellow/Purple Light-Box Column Signals

These signals drive the non-blue 1 x 1 light boxes in each column.

| Area | Column | Signal pin |
|---|---:|---|
| Front | C1 | `PE0` |
| Front | C2 | `PE1` |
| Front | C3 | `PE2` |
| Front | C4 | `PE3` |
| Front | C5 | `PE4` |
| Front | C6 | `PE5` |
| Front | C7 | `PE6` |
| Front | C8 | `PE7` |
| Front | C9 | `PE8` |
| Front | C10 | `PE9` |
| Front | C11 | `PE10` |
| Side | C1 | `PD0` |
| Side | C2 | `PD1` |
| Side | C3 | `PD2` |
| Side | C4 | `PD3` |

### Blue 1 x 1 Panel Column Signals

These signals drive named blue 1 x 1 panels by column group.

| Area | Blue panels | Column | Signal pin |
|---|---|---:|---|
| Front | `a`, `b`, `c` | C9 | `PD4` |
| Front | `d` | C11 | `PD5` |
| Side | `h`, `j` | C1 | `PD6` |
| Side | `i` | C2 | `PD7` |
| Side | `f` | C3 | `PD8` |
| Side | `e`, `g` | C4 | `PD9` |

### Blue 2 x 2 Strip Signals

These signals drive the named blue 2 x 2 strip blocks. Each block has its own data signal.

| Strip block | Signal pin |
|---:|---|
| A | `PD10` |
| B | `PD11` |
| C | `PD12` |
| D | `PD13` |
| E | `PD14` |
| F | `PD15` |
| G | `PC0` |

### Reserved Control Pins

| Use | Pins |
|---|---|
| PCA9685 I2C | `PB6` SCL, `PB7` SDA |
| Suggested to avoid | `PA13`, `PA14` for SWD debug; `PA11`, `PA12` if USB CDC remains enabled |

## Preview Color Groups

These groups mirror the current web preview color logic.

| Area | Purple servo boxes | Yellow servo boxes | Empty positions |
|---|---|---|---|
| Front | 6, 7, 9, 13, 15, 19, 23, 26, 28, 29, 31, 33, 36, 37 | All other front servo boxes | Blue |
| Side | 4, 5, 8, 9 | All other side servo boxes | Blue |
