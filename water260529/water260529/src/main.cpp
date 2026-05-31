#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <FastLED.h>
#include <math.h>

// ---------------- Hardware configuration ----------------

#define I2C_SDA_PIN PB7
#define I2C_SCL_PIN PB6

#define LED_FRONT_PANEL_C1_PIN PE0
#define LED_FRONT_PANEL_C2_PIN PE1
#define LED_FRONT_PANEL_C3_PIN PE2
#define LED_FRONT_PANEL_C4_PIN PE3
#define LED_FRONT_PANEL_C5_PIN PE4
#define LED_FRONT_PANEL_C6_PIN PE5
#define LED_FRONT_PANEL_C7_PIN PE6
#define LED_FRONT_PANEL_C8_PIN PE7
#define LED_FRONT_PANEL_C9_PIN PE8
#define LED_FRONT_PANEL_C10_PIN PE9
#define LED_FRONT_PANEL_C11_PIN PE10

#define LED_SIDE_PANEL_C1_PIN PD0
#define LED_SIDE_PANEL_C2_PIN PD1
#define LED_SIDE_PANEL_C3_PIN PD2
#define LED_SIDE_PANEL_C4_PIN PD3

#define LED_BLUE_PANEL_FRONT_C9_PIN PD4
#define LED_BLUE_PANEL_FRONT_C11_PIN PD5
#define LED_BLUE_PANEL_SIDE_C1_PIN PD6
#define LED_BLUE_PANEL_SIDE_C2_PIN PD7
#define LED_BLUE_PANEL_SIDE_C3_PIN PD8
#define LED_BLUE_PANEL_SIDE_C4_PIN PD9

#define LED_STRIP_A_PIN PD10
#define LED_STRIP_B_PIN PD11
#define LED_STRIP_C_PIN PD12
#define LED_STRIP_D_PIN PD13
#define LED_STRIP_E_PIN PD14
#define LED_STRIP_F_PIN PD15
#define LED_STRIP_G_PIN PC0

static constexpr uint8_t kFrontRows = 6;
static constexpr uint8_t kFrontCols = 11;
static constexpr uint8_t kSideRows = 6;
static constexpr uint8_t kSideCols = 4;
static constexpr uint8_t kMaxRows = 6;
static constexpr uint8_t kMaxCols = 11;

static constexpr uint16_t kPanelLedCount = 256;
static constexpr uint16_t kBlueUnitLedCount = 60;
static constexpr uint16_t kStripLedCount = 120;
static constexpr uint8_t kLedSignalCount = 28;

static constexpr uint8_t kFrontPanelColumnCount = 11;
static constexpr uint8_t kSidePanelColumnCount = 4;
static constexpr uint8_t kBluePanelGroupCount = 6;
static constexpr uint8_t kStripBlockCount = 7;

static constexpr uint8_t kFrontPanelUnitsPerColumn[kFrontPanelColumnCount] = {
  4, 4, 4, 4, 2, 2, 4, 4, 1, 4, 5
};

static constexpr uint8_t kSidePanelUnitsPerColumn[kSidePanelColumnCount] = {
  2, 3, 5, 4
};

static constexpr uint8_t kBluePanelUnitsPerGroup[kBluePanelGroupCount] = {
  3, 1, 2, 1, 1, 2
};

TwoWire planbI2C(I2C_SDA_PIN, I2C_SCL_PIN);

Adafruit_PWMServoDriver frontServoA(0x40, planbI2C); // front boxes 1-16
Adafruit_PWMServoDriver frontServoB(0x60, planbI2C); // front boxes 17-32
Adafruit_PWMServoDriver frontServoC(0x50, planbI2C); // front boxes 33-38
Adafruit_PWMServoDriver sideServo(0x68, planbI2C);   // side boxes 1-14

Adafruit_PWMServoDriver* const kServoDrivers[] = {
  &frontServoA,
  &frontServoB,
  &frontServoC,
  &sideServo
};

// ---------------- Effect selection ----------------

enum class Pattern : uint8_t {
  Breath,
  Ripple,
  Wander
};

// PlanB is no-interaction by default. Change this constant to preview the other
// two hardware patterns after the wiring is confirmed.
static constexpr Pattern kActivePattern = Pattern::Breath;
static constexpr float kEffectSpeed = 1.0f;

// ---------------- Effect tuning copied from the web preview ----------------

static constexpr uint32_t kFrameMs = 20;
static constexpr uint32_t kBreathPeriodMs = 5200;
static constexpr uint32_t kRipplePeriodMs = 4200;
static constexpr uint32_t kWanderStepMs = 160;
static constexpr uint32_t kWanderBlueBreathPeriodMs = 6800;
static constexpr uint8_t kWanderTrailLength = 4;
static constexpr uint8_t kWanderDotCount = 5;

static constexpr float kXPhase = 0.32f;
static constexpr float kYPhase = 0.46f;
static constexpr float kSidePhaseOffset = 1.15f;
static constexpr float kTwoPi = 6.28318530718f;

static constexpr uint8_t kLedMinBrightness = 2;
static constexpr uint8_t kLedMaxBrightness = 20;

static const CRGB kBlueBaseColor(70, 150, 216);    // #4696D8
static const CRGB kYellowBaseColor(241, 232, 142); // #F1E88E
static const CRGB kPurpleBaseColor(162, 93, 219);  // #A25DDB

// Continuous-rotation servo speed range. 1500us is stop; these values run one
// direction only, matching the linkage that converts rotation to reciprocation.
static constexpr uint16_t kServoStopUs = 1500;
static constexpr uint16_t kServoMinRunUs = 1540;
static constexpr uint16_t kServoMaxRunUs = 1740;

// ---------------- Layout copied from front_breath_preview.html ----------------

enum class Surface : uint8_t {
  Front,
  Side
};

enum class CellColor : uint8_t {
  Blue,
  Yellow,
  Purple
};

enum class LedKind : uint8_t {
  Panel,
  Strip
};

struct MergedBlock {
  uint8_t row;
  uint8_t col;
  uint8_t rowSpan;
  uint8_t colSpan;
};

struct LedUnit {
  Surface surface;
  int8_t box;
  uint8_t row;
  uint8_t col;
  uint8_t rowSpan;
  uint8_t colSpan;
  CellColor color;
  char label;
  LedKind kind;
};

struct ServoUnit {
  Surface surface;
  uint8_t box;
  uint8_t row;
  uint8_t col;
  uint8_t driverIndex;
  uint8_t channel;
};

struct RippleState {
  int32_t cycle = -1;
  uint8_t centerRow = 0;
  uint8_t centerCol = 0;
};

struct Walker {
  uint8_t row = 0;
  uint8_t col = 0;
  uint8_t previousRow = 0;
  uint8_t previousCol = 0;
  int16_t visitedStep[kMaxRows][kMaxCols];
};

struct WanderState {
  int32_t lastStep = -1;
  bool initialized = false;
  Walker walkers[kWanderDotCount];
};

static const int8_t kFrontGrid[kFrontRows][kFrontCols] = {
  { 1,  2,  3,  4,  5,  6,  7,  8, -1,  9, 10 },
  {11, 12, 13, 14, -1, -1, -1, -1, -1, 15, 16 },
  {17, 18, 19, 20, -1, -1, -1, -1, -1, 21, 22 },
  {-1, -1, -1, -1, -1, -1, 23, 24, -1, -1, -1 },
  {-1, -1, -1, -1, -1, -1, 26, 27, -1, -1, 25 },
  {28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38 }
};

static const int8_t kSideGrid[kSideRows][kSideCols] = {
  { 1,  2,  3, -1 },
  {-1, -1, -1, -1 },
  {-1, -1,  4,  5 },
  {-1, -1,  6,  7 },
  {-1,  8,  9, 10 },
  {11, 12, 13, 14 }
};

static const MergedBlock kFrontMergedBlocks[] = {
  {3, 8, 2, 2}, // A
  {1, 6, 2, 2}, // B
  {1, 4, 2, 2}, // C
  {3, 4, 2, 2}, // D
  {3, 2, 2, 2}, // E
  {3, 0, 2, 2}  // F
};

static const MergedBlock kSideMergedBlocks[] = {
  {1, 0, 2, 2}  // G
};

static constexpr uint8_t kMaxLedUnits = 69;
static constexpr uint8_t kMaxServoUnits = 52;

static LedUnit ledUnits[kMaxLedUnits];
static uint8_t ledUnitCount = 0;

static ServoUnit servoUnits[kMaxServoUnits];
static uint8_t servoUnitCount = 0;

static RippleState frontRipple;
static RippleState sideRipple;
static WanderState frontWander;
static WanderState sideWander;

static CRGB frontPanelC1[kPanelLedCount * 4];
static CRGB frontPanelC2[kPanelLedCount * 4];
static CRGB frontPanelC3[kPanelLedCount * 4];
static CRGB frontPanelC4[kPanelLedCount * 4];
static CRGB frontPanelC5[kPanelLedCount * 2];
static CRGB frontPanelC6[kPanelLedCount * 2];
static CRGB frontPanelC7[kPanelLedCount * 4];
static CRGB frontPanelC8[kPanelLedCount * 4];
static CRGB frontPanelC9[kPanelLedCount * 1];
static CRGB frontPanelC10[kPanelLedCount * 4];
static CRGB frontPanelC11[kPanelLedCount * 5];

static CRGB sidePanelC1[kPanelLedCount * 2];
static CRGB sidePanelC2[kPanelLedCount * 3];
static CRGB sidePanelC3[kPanelLedCount * 5];
static CRGB sidePanelC4[kPanelLedCount * 4];

static CRGB bluePanelFrontC9[kBlueUnitLedCount * 3];  // c, b, a
static CRGB bluePanelFrontC11[kBlueUnitLedCount * 1]; // d
static CRGB bluePanelSideC1[kBlueUnitLedCount * 2];   // j, h
static CRGB bluePanelSideC2[kBlueUnitLedCount * 1];   // i
static CRGB bluePanelSideC3[kBlueUnitLedCount * 1];   // f
static CRGB bluePanelSideC4[kBlueUnitLedCount * 2];   // g, e

static CRGB stripA[kStripLedCount];
static CRGB stripB[kStripLedCount];
static CRGB stripC[kStripLedCount];
static CRGB stripD[kStripLedCount];
static CRGB stripE[kStripLedCount];
static CRGB stripF[kStripLedCount];
static CRGB stripG[kStripLedCount];

static uint32_t lastFrameMs = 0;

// ---------------- Math helpers ----------------

static float clamp01(float value) {
  if (value < 0.0f) {
    return 0.0f;
  }
  if (value > 1.0f) {
    return 1.0f;
  }
  return value;
}

static float wave01(float phase) {
  return 0.5f - 0.5f * cosf(phase);
}

static float gamma01(float value) {
  value = clamp01(value);
  return value * value;
}

static uint8_t brightnessForValue(float value) {
  const float corrected = gamma01(value);
  return (uint8_t)roundf(kLedMinBrightness + corrected * (kLedMaxBrightness - kLedMinBrightness));
}

static CRGB colorForCellColor(CellColor color) {
  switch (color) {
    case CellColor::Purple:
      return kPurpleBaseColor;
    case CellColor::Yellow:
      return kYellowBaseColor;
    case CellColor::Blue:
    default:
      return kBlueBaseColor;
  }
}

static CRGB scaledColorForBrightness(CellColor color, uint8_t brightness) {
  const uint8_t amount = (uint8_t)constrain(
    map(brightness, kLedMinBrightness, kLedMaxBrightness, 0, 255),
    0,
    255
  );
  CRGB output = colorForCellColor(color);
  output.nscale8_video(amount);
  return output;
}

static uint16_t servoPulseForValue(float value) {
  const float corrected = gamma01(value);
  const float pulse = kServoMinRunUs + corrected * (kServoMaxRunUs - kServoMinRunUs);
  return (uint16_t)roundf(pulse);
}

static uint32_t scaledPeriod(uint32_t periodMs) {
  return (uint32_t)max(1.0f, periodMs / kEffectSpeed);
}

static uint32_t scaledStep(uint32_t stepMs) {
  return (uint32_t)max(1.0f, stepMs / kEffectSpeed);
}

static uint8_t rowsForSurface(Surface surface) {
  return surface == Surface::Front ? kFrontRows : kSideRows;
}

static uint8_t colsForSurface(Surface surface) {
  return surface == Surface::Front ? kFrontCols : kSideCols;
}

static float phaseOffsetForSurface(Surface surface) {
  return surface == Surface::Front ? 0.0f : kSidePhaseOffset;
}

static bool isPurpleBox(Surface surface, uint8_t box) {
  if (surface == Surface::Front) {
    switch (box) {
      case 6:
      case 7:
      case 9:
      case 13:
      case 15:
      case 19:
      case 23:
      case 26:
      case 28:
      case 29:
      case 31:
      case 33:
      case 36:
      case 37:
        return true;
      default:
        return false;
    }
  }

  switch (box) {
    case 4:
    case 5:
    case 8:
    case 9:
      return true;
    default:
      return false;
  }
}

static CellColor colorForBox(Surface surface, int8_t box) {
  if (box < 0) {
    return CellColor::Blue;
  }
  return isPurpleBox(surface, (uint8_t)box) ? CellColor::Purple : CellColor::Yellow;
}

static char ledLabelFor(Surface surface, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan) {
  if (surface == Surface::Front) {
    if (rowSpan > 1 || colSpan > 1) {
      if (row == 3 && col == 8) return 'A';
      if (row == 1 && col == 6) return 'B';
      if (row == 1 && col == 4) return 'C';
      if (row == 3 && col == 4) return 'D';
      if (row == 3 && col == 2) return 'E';
      if (row == 3 && col == 0) return 'F';
    } else {
      if (row == 0 && col == 8) return 'a';
      if (row == 1 && col == 8) return 'b';
      if (row == 2 && col == 8) return 'c';
      if (row == 3 && col == 10) return 'd';
    }
    return '\0';
  }

  if (rowSpan > 1 || colSpan > 1) {
    if (row == 1 && col == 0) return 'G';
  } else {
    if (row == 0 && col == 3) return 'e';
    if (row == 1 && col == 2) return 'f';
    if (row == 1 && col == 3) return 'g';
    if (row == 3 && col == 0) return 'h';
    if (row == 3 && col == 1) return 'i';
    if (row == 4 && col == 0) return 'j';
  }
  return '\0';
}

// ---------------- Layout builders ----------------

static bool blockCovers(const MergedBlock& block, uint8_t row, uint8_t col) {
  return row >= block.row &&
         row < block.row + block.rowSpan &&
         col >= block.col &&
         col < block.col + block.colSpan;
}

static const MergedBlock* mergedBlockAt(
  const MergedBlock* blocks,
  uint8_t blockCount,
  uint8_t row,
  uint8_t col
) {
  for (uint8_t i = 0; i < blockCount; i++) {
    if (blocks[i].row == row && blocks[i].col == col) {
      return &blocks[i];
    }
  }
  return nullptr;
}

static bool isCoveredByMergedBlock(
  const MergedBlock* blocks,
  uint8_t blockCount,
  uint8_t row,
  uint8_t col
) {
  for (uint8_t i = 0; i < blockCount; i++) {
    if (blockCovers(blocks[i], row, col) && (blocks[i].row != row || blocks[i].col != col)) {
      return true;
    }
  }
  return false;
}

static int8_t gridBoxAt(Surface surface, uint8_t row, uint8_t col) {
  return surface == Surface::Front ? kFrontGrid[row][col] : kSideGrid[row][col];
}

static bool servoRouteFor(Surface surface, uint8_t box, uint8_t& driverIndex, uint8_t& channel) {
  if (surface == Surface::Front) {
    if (box >= 1 && box <= 16) {
      driverIndex = 0;
      channel = box - 1;
      return true;
    }
    if (box >= 17 && box <= 32) {
      driverIndex = 1;
      channel = box - 17;
      return true;
    }
    if (box >= 33 && box <= 38) {
      driverIndex = 2;
      channel = box - 33;
      return true;
    }
    return false;
  }

  if (box == 14) {
    driverIndex = 3;
    channel = 2;
    return true;
  }
  if (box >= 1 && box <= 13) {
    driverIndex = 3;
    channel = box + 2;
    return true;
  }
  return false;
}

static void addLedUnit(Surface surface, int8_t box, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan) {
  if (ledUnitCount >= kMaxLedUnits) {
    return;
  }
  ledUnits[ledUnitCount++] = {
    surface,
    box,
    row,
    col,
    rowSpan,
    colSpan,
    colorForBox(surface, box),
    ledLabelFor(surface, row, col, rowSpan, colSpan),
    (rowSpan > 1 || colSpan > 1) ? LedKind::Strip : LedKind::Panel
  };
}

static void addServoUnit(Surface surface, uint8_t box, uint8_t row, uint8_t col) {
  if (servoUnitCount >= kMaxServoUnits) {
    return;
  }

  uint8_t driverIndex = 0;
  uint8_t channel = 0;
  if (!servoRouteFor(surface, box, driverIndex, channel)) {
    return;
  }

  servoUnits[servoUnitCount++] = {
    surface,
    box,
    row,
    col,
    driverIndex,
    channel
  };
}

static void buildSurfaceLayout(
  Surface surface,
  const MergedBlock* blocks,
  uint8_t blockCount
) {
  const uint8_t rows = rowsForSurface(surface);
  const uint8_t cols = colsForSurface(surface);

  for (uint8_t row = 0; row < rows; row++) {
    for (uint8_t col = 0; col < cols; col++) {
      const int8_t box = gridBoxAt(surface, row, col);
      if (box > 0) {
        addServoUnit(surface, (uint8_t)box, row, col);
      }

      if (isCoveredByMergedBlock(blocks, blockCount, row, col)) {
        continue;
      }

      const MergedBlock* block = mergedBlockAt(blocks, blockCount, row, col);
      if (block != nullptr) {
        addLedUnit(surface, box, row, col, block->rowSpan, block->colSpan);
      } else {
        addLedUnit(surface, box, row, col, 1, 1);
      }
    }
  }
}

static void buildLayout() {
  ledUnitCount = 0;
  servoUnitCount = 0;
  buildSurfaceLayout(Surface::Front, kFrontMergedBlocks, sizeof(kFrontMergedBlocks) / sizeof(kFrontMergedBlocks[0]));
  buildSurfaceLayout(Surface::Side, kSideMergedBlocks, sizeof(kSideMergedBlocks) / sizeof(kSideMergedBlocks[0]));
}

// ---------------- Ripple pattern ----------------

static RippleState& rippleForSurface(Surface surface) {
  return surface == Surface::Front ? frontRipple : sideRipple;
}

static void chooseRippleCenter(Surface surface, RippleState& ripple) {
  ripple.centerRow = (uint8_t)random(rowsForSurface(surface));
  ripple.centerCol = (uint8_t)random(colsForSurface(surface));
}

static float maxDistanceFromCenter(Surface surface, const RippleState& ripple) {
  float maxDistance = 0.0f;
  const uint8_t rows = rowsForSurface(surface);
  const uint8_t cols = colsForSurface(surface);
  for (uint8_t row = 0; row < rows; row++) {
    for (uint8_t col = 0; col < cols; col++) {
      const float distance = hypotf((float)col - ripple.centerCol, (float)row - ripple.centerRow);
      if (distance > maxDistance) {
        maxDistance = distance;
      }
    }
  }
  return maxDistance;
}

static float rippleValue(Surface surface, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan, uint32_t now) {
  const uint32_t periodMs = scaledPeriod(kRipplePeriodMs);
  RippleState& ripple = rippleForSurface(surface);
  const int32_t cycleIndex = now / periodMs;

  if (cycleIndex != ripple.cycle) {
    ripple.cycle = cycleIndex;
    chooseRippleCenter(surface, ripple);
  }

  const float progress = (now % periodMs) / (float)periodMs;
  const float maxDistance = maxDistanceFromCenter(surface, ripple) + 1.2f;
  const float radius = progress * maxDistance;
  const float centerCol = col + (colSpan - 1) * 0.5f;
  const float centerRow = row + (rowSpan - 1) * 0.5f;
  const float distance = hypotf(centerCol - ripple.centerCol, centerRow - ripple.centerRow);
  static constexpr float kRingWidth = 0.95f;
  return clamp01(1.0f - fabsf(distance - radius) / kRingWidth);
}

// ---------------- Wander pattern ----------------

static WanderState& wanderForSurface(Surface surface) {
  return surface == Surface::Front ? frontWander : sideWander;
}

static void clearVisited(Walker& walker) {
  for (uint8_t row = 0; row < kMaxRows; row++) {
    for (uint8_t col = 0; col < kMaxCols; col++) {
      walker.visitedStep[row][col] = -20000;
    }
  }
}

static void initWalker(Surface surface, Walker& walker, int32_t step) {
  clearVisited(walker);
  walker.row = (uint8_t)random(rowsForSurface(surface));
  walker.col = (uint8_t)random(colsForSurface(surface));
  walker.previousRow = walker.row;
  walker.previousCol = walker.col;
  walker.visitedStep[walker.row][walker.col] = (int16_t)step;
}

static void initWalkers(Surface surface, int32_t step) {
  WanderState& state = wanderForSurface(surface);
  for (uint8_t i = 0; i < kWanderDotCount; i++) {
    initWalker(surface, state.walkers[i], step);
  }
  state.lastStep = step;
  state.initialized = true;
}

static void advanceWalker(Surface surface, Walker& walker, int32_t step) {
  struct Position {
    uint8_t row;
    uint8_t col;
  };

  Position options[4];
  uint8_t optionCount = 0;
  const uint8_t rows = rowsForSurface(surface);
  const uint8_t cols = colsForSurface(surface);

  if (walker.row > 0) {
    options[optionCount++] = { (uint8_t)(walker.row - 1), walker.col };
  }
  if (walker.row + 1 < rows) {
    options[optionCount++] = { (uint8_t)(walker.row + 1), walker.col };
  }
  if (walker.col > 0) {
    options[optionCount++] = { walker.row, (uint8_t)(walker.col - 1) };
  }
  if (walker.col + 1 < cols) {
    options[optionCount++] = { walker.row, (uint8_t)(walker.col + 1) };
  }

  if (optionCount == 0) {
    return;
  }

  if (optionCount > 1) {
    for (uint8_t i = 0; i < optionCount; i++) {
      if (options[i].row == walker.previousRow && options[i].col == walker.previousCol) {
        options[i] = options[optionCount - 1];
        optionCount--;
        break;
      }
    }
  }

  const uint8_t nextIndex = (uint8_t)random(optionCount);
  walker.previousRow = walker.row;
  walker.previousCol = walker.col;
  walker.row = options[nextIndex].row;
  walker.col = options[nextIndex].col;
  walker.visitedStep[walker.row][walker.col] = (int16_t)step;
}

static void updateWanderSurface(Surface surface, int32_t currentStep) {
  WanderState& state = wanderForSurface(surface);
  if (!state.initialized || state.lastStep < 0 || currentStep - state.lastStep > 24) {
    initWalkers(surface, currentStep);
    return;
  }

  const int32_t targetStep = min(currentStep, state.lastStep + 24);
  for (int32_t step = state.lastStep + 1; step <= targetStep; step++) {
    for (uint8_t i = 0; i < kWanderDotCount; i++) {
      advanceWalker(surface, state.walkers[i], step);
    }
  }
  state.lastStep = targetStep;
}

static void updateWanderStates(uint32_t now) {
  const int32_t currentStep = now / scaledStep(kWanderStepMs);
  updateWanderSurface(Surface::Front, currentStep);
  updateWanderSurface(Surface::Side, currentStep);
}

static bool positionCoveredByUnit(const LedUnit& unit, uint8_t row, uint8_t col) {
  return row >= unit.row &&
         row < unit.row + unit.rowSpan &&
         col >= unit.col &&
         col < unit.col + unit.colSpan;
}

static float wanderTrailValue(const LedUnit& unit, uint32_t now) {
  const int32_t currentStep = now / scaledStep(kWanderStepMs);
  WanderState& state = wanderForSurface(unit.surface);
  float value = 0.0f;

  for (uint8_t i = 0; i < kWanderDotCount; i++) {
    const Walker& walker = state.walkers[i];
    for (uint8_t row = unit.row; row < unit.row + unit.rowSpan; row++) {
      for (uint8_t col = unit.col; col < unit.col + unit.colSpan; col++) {
        const int16_t visited = walker.visitedStep[row][col];
        const int32_t age = currentStep - visited;
        if (age >= 0 && age <= kWanderTrailLength) {
          const float candidate = (kWanderTrailLength + 1 - age) / (float)(kWanderTrailLength + 1);
          if (candidate > value) {
            value = candidate;
          }
        }
      }
    }
  }
  return value;
}

static float wanderTrailValueForServo(const ServoUnit& servo, uint32_t now) {
  LedUnit unit = {
    servo.surface,
    (int8_t)servo.box,
    servo.row,
    servo.col,
    1,
    1,
    colorForBox(servo.surface, (int8_t)servo.box),
    '\0',
    LedKind::Panel
  };
  return wanderTrailValue(unit, now);
}

static float wanderBlueBreathValue(const LedUnit& unit, uint32_t now) {
  const uint32_t periodMs = scaledPeriod(kWanderBlueBreathPeriodMs);
  const float cycleMs = now % periodMs;
  const float centerCol = unit.col + (unit.colSpan - 1) * 0.5f;
  const float phase = (cycleMs / (float)periodMs) * kTwoPi - centerCol * 0.42f + phaseOffsetForSurface(unit.surface);
  return wave01(phase);
}

// ---------------- Pattern dispatch ----------------

static float breathLedValue(Surface surface, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan, uint32_t now) {
  (void)row;
  (void)rowSpan;
  const uint32_t periodMs = scaledPeriod(kBreathPeriodMs);
  const float basePhase = ((now % periodMs) / (float)periodMs) * kTwoPi + phaseOffsetForSurface(surface);
  const float centerCol = col + (colSpan - 1) * 0.5f;
  return wave01(basePhase + centerCol * kXPhase);
}

static float breathServoValue(const ServoUnit& servo, uint32_t now) {
  const uint32_t periodMs = scaledPeriod(kBreathPeriodMs);
  const float basePhase = ((now % periodMs) / (float)periodMs) * kTwoPi + phaseOffsetForSurface(servo.surface);
  return wave01(basePhase + servo.col * kXPhase + servo.row * kYPhase);
}

static float ledValueForUnit(const LedUnit& unit, uint32_t now) {
  if (kActivePattern == Pattern::Ripple) {
    return rippleValue(unit.surface, unit.row, unit.col, unit.rowSpan, unit.colSpan, now);
  }
  if (kActivePattern == Pattern::Wander) {
    if (unit.color == CellColor::Blue) {
      return wanderBlueBreathValue(unit, now);
    }
    return wanderTrailValue(unit, now);
  }
  return breathLedValue(unit.surface, unit.row, unit.col, unit.rowSpan, unit.colSpan, now);
}

static float servoValueForUnit(const ServoUnit& servo, uint32_t now) {
  if (kActivePattern == Pattern::Ripple) {
    return rippleValue(servo.surface, servo.row, servo.col, 1, 1, now);
  }
  if (kActivePattern == Pattern::Wander) {
    return wanderTrailValueForServo(servo, now);
  }
  return breathServoValue(servo, now);
}

// ---------------- Hardware output ----------------

static void resetLedBuffers() {
  FastLED.clear(false);
}

static void fillLedSegment(CRGB* leds, uint16_t ledCount, uint8_t segmentIndex, uint16_t segmentSize, const CRGB& color) {
  const uint32_t start = (uint32_t)segmentIndex * segmentSize;
  if (start >= ledCount) {
    return;
  }
  const uint16_t count = (uint16_t)min((uint32_t)segmentSize, (uint32_t)ledCount - start);
  fill_solid(leds + start, count, color);
}

static uint8_t panelSegmentIndexFor(const LedUnit& unit) {
  uint8_t index = 0;
  for (uint8_t i = 0; i < ledUnitCount; i++) {
    const LedUnit& other = ledUnits[i];
    if (other.surface != unit.surface ||
        other.kind != LedKind::Panel ||
        other.color == CellColor::Blue ||
        other.col != unit.col) {
      continue;
    }
    if (other.row > unit.row) {
      index++;
    }
  }
  return index;
}

static void writeFrontPanelColumn(const LedUnit& unit, const CRGB& color) {
  const uint8_t segment = panelSegmentIndexFor(unit);
  switch (unit.col) {
    case 0: fillLedSegment(frontPanelC1, sizeof(frontPanelC1) / sizeof(frontPanelC1[0]), segment, kPanelLedCount, color); break;
    case 1: fillLedSegment(frontPanelC2, sizeof(frontPanelC2) / sizeof(frontPanelC2[0]), segment, kPanelLedCount, color); break;
    case 2: fillLedSegment(frontPanelC3, sizeof(frontPanelC3) / sizeof(frontPanelC3[0]), segment, kPanelLedCount, color); break;
    case 3: fillLedSegment(frontPanelC4, sizeof(frontPanelC4) / sizeof(frontPanelC4[0]), segment, kPanelLedCount, color); break;
    case 4: fillLedSegment(frontPanelC5, sizeof(frontPanelC5) / sizeof(frontPanelC5[0]), segment, kPanelLedCount, color); break;
    case 5: fillLedSegment(frontPanelC6, sizeof(frontPanelC6) / sizeof(frontPanelC6[0]), segment, kPanelLedCount, color); break;
    case 6: fillLedSegment(frontPanelC7, sizeof(frontPanelC7) / sizeof(frontPanelC7[0]), segment, kPanelLedCount, color); break;
    case 7: fillLedSegment(frontPanelC8, sizeof(frontPanelC8) / sizeof(frontPanelC8[0]), segment, kPanelLedCount, color); break;
    case 8: fillLedSegment(frontPanelC9, sizeof(frontPanelC9) / sizeof(frontPanelC9[0]), segment, kPanelLedCount, color); break;
    case 9: fillLedSegment(frontPanelC10, sizeof(frontPanelC10) / sizeof(frontPanelC10[0]), segment, kPanelLedCount, color); break;
    case 10: fillLedSegment(frontPanelC11, sizeof(frontPanelC11) / sizeof(frontPanelC11[0]), segment, kPanelLedCount, color); break;
    default: break;
  }
}

static void writeSidePanelColumn(const LedUnit& unit, const CRGB& color) {
  const uint8_t segment = panelSegmentIndexFor(unit);
  switch (unit.col) {
    case 0: fillLedSegment(sidePanelC1, sizeof(sidePanelC1) / sizeof(sidePanelC1[0]), segment, kPanelLedCount, color); break;
    case 1: fillLedSegment(sidePanelC2, sizeof(sidePanelC2) / sizeof(sidePanelC2[0]), segment, kPanelLedCount, color); break;
    case 2: fillLedSegment(sidePanelC3, sizeof(sidePanelC3) / sizeof(sidePanelC3[0]), segment, kPanelLedCount, color); break;
    case 3: fillLedSegment(sidePanelC4, sizeof(sidePanelC4) / sizeof(sidePanelC4[0]), segment, kPanelLedCount, color); break;
    default: break;
  }
}

static bool bluePanelRoute(char label, CRGB*& leds, uint16_t& ledCount, uint8_t& segment) {
  switch (label) {
    case 'c': leds = bluePanelFrontC9; ledCount = sizeof(bluePanelFrontC9) / sizeof(bluePanelFrontC9[0]); segment = 0; return true;
    case 'b': leds = bluePanelFrontC9; ledCount = sizeof(bluePanelFrontC9) / sizeof(bluePanelFrontC9[0]); segment = 1; return true;
    case 'a': leds = bluePanelFrontC9; ledCount = sizeof(bluePanelFrontC9) / sizeof(bluePanelFrontC9[0]); segment = 2; return true;
    case 'd': leds = bluePanelFrontC11; ledCount = sizeof(bluePanelFrontC11) / sizeof(bluePanelFrontC11[0]); segment = 0; return true;
    case 'j': leds = bluePanelSideC1; ledCount = sizeof(bluePanelSideC1) / sizeof(bluePanelSideC1[0]); segment = 0; return true;
    case 'h': leds = bluePanelSideC1; ledCount = sizeof(bluePanelSideC1) / sizeof(bluePanelSideC1[0]); segment = 1; return true;
    case 'i': leds = bluePanelSideC2; ledCount = sizeof(bluePanelSideC2) / sizeof(bluePanelSideC2[0]); segment = 0; return true;
    case 'f': leds = bluePanelSideC3; ledCount = sizeof(bluePanelSideC3) / sizeof(bluePanelSideC3[0]); segment = 0; return true;
    case 'g': leds = bluePanelSideC4; ledCount = sizeof(bluePanelSideC4) / sizeof(bluePanelSideC4[0]); segment = 0; return true;
    case 'e': leds = bluePanelSideC4; ledCount = sizeof(bluePanelSideC4) / sizeof(bluePanelSideC4[0]); segment = 1; return true;
    default: return false;
  }
}

static bool stripRoute(char label, CRGB*& leds, uint16_t& ledCount) {
  ledCount = kStripLedCount;
  switch (label) {
    case 'A': leds = stripA; return true;
    case 'B': leds = stripB; return true;
    case 'C': leds = stripC; return true;
    case 'D': leds = stripD; return true;
    case 'E': leds = stripE; return true;
    case 'F': leds = stripF; return true;
    case 'G': leds = stripG; return true;
    default: return false;
  }
}

static void writeLedUnit(const LedUnit& unit, uint8_t brightness) {
  const CRGB color = scaledColorForBrightness(unit.color, brightness);

  if (unit.kind == LedKind::Strip) {
    CRGB* leds = nullptr;
    uint16_t ledCount = 0;
    if (stripRoute(unit.label, leds, ledCount)) {
      fill_solid(leds, ledCount, color);
    }
    return;
  }

  if (unit.color == CellColor::Blue) {
    CRGB* leds = nullptr;
    uint16_t ledCount = 0;
    uint8_t segment = 0;
    if (bluePanelRoute(unit.label, leds, ledCount, segment)) {
      fillLedSegment(leds, ledCount, segment, kBlueUnitLedCount, color);
    }
    return;
  }

  if (unit.surface == Surface::Front) {
    writeFrontPanelColumn(unit, color);
  } else {
    writeSidePanelColumn(unit, color);
  }
}

static void flushLedBuffers() {
  FastLED.show();
}

static void writeServo(const ServoUnit& servo, uint16_t pulseUs) {
  kServoDrivers[servo.driverIndex]->writeMicroseconds(servo.channel, pulseUs);
}

static void allServosStop() {
  for (uint8_t i = 0; i < servoUnitCount; i++) {
    writeServo(servoUnits[i], kServoStopUs);
  }
}

static void allLedsOff() {
  FastLED.clear(true);
}

static void setupLedPins() {
  FastLED.addLeds<WS2812B, LED_FRONT_PANEL_C1_PIN, GRB>(frontPanelC1, sizeof(frontPanelC1) / sizeof(frontPanelC1[0]));
  FastLED.addLeds<WS2812B, LED_FRONT_PANEL_C2_PIN, GRB>(frontPanelC2, sizeof(frontPanelC2) / sizeof(frontPanelC2[0]));
  FastLED.addLeds<WS2812B, LED_FRONT_PANEL_C3_PIN, GRB>(frontPanelC3, sizeof(frontPanelC3) / sizeof(frontPanelC3[0]));
  FastLED.addLeds<WS2812B, LED_FRONT_PANEL_C4_PIN, GRB>(frontPanelC4, sizeof(frontPanelC4) / sizeof(frontPanelC4[0]));
  FastLED.addLeds<WS2812B, LED_FRONT_PANEL_C5_PIN, GRB>(frontPanelC5, sizeof(frontPanelC5) / sizeof(frontPanelC5[0]));
  FastLED.addLeds<WS2812B, LED_FRONT_PANEL_C6_PIN, GRB>(frontPanelC6, sizeof(frontPanelC6) / sizeof(frontPanelC6[0]));
  FastLED.addLeds<WS2812B, LED_FRONT_PANEL_C7_PIN, GRB>(frontPanelC7, sizeof(frontPanelC7) / sizeof(frontPanelC7[0]));
  FastLED.addLeds<WS2812B, LED_FRONT_PANEL_C8_PIN, GRB>(frontPanelC8, sizeof(frontPanelC8) / sizeof(frontPanelC8[0]));
  FastLED.addLeds<WS2812B, LED_FRONT_PANEL_C9_PIN, GRB>(frontPanelC9, sizeof(frontPanelC9) / sizeof(frontPanelC9[0]));
  FastLED.addLeds<WS2812B, LED_FRONT_PANEL_C10_PIN, GRB>(frontPanelC10, sizeof(frontPanelC10) / sizeof(frontPanelC10[0]));
  FastLED.addLeds<WS2812B, LED_FRONT_PANEL_C11_PIN, GRB>(frontPanelC11, sizeof(frontPanelC11) / sizeof(frontPanelC11[0]));

  FastLED.addLeds<WS2812B, LED_SIDE_PANEL_C1_PIN, GRB>(sidePanelC1, sizeof(sidePanelC1) / sizeof(sidePanelC1[0]));
  FastLED.addLeds<WS2812B, LED_SIDE_PANEL_C2_PIN, GRB>(sidePanelC2, sizeof(sidePanelC2) / sizeof(sidePanelC2[0]));
  FastLED.addLeds<WS2812B, LED_SIDE_PANEL_C3_PIN, GRB>(sidePanelC3, sizeof(sidePanelC3) / sizeof(sidePanelC3[0]));
  FastLED.addLeds<WS2812B, LED_SIDE_PANEL_C4_PIN, GRB>(sidePanelC4, sizeof(sidePanelC4) / sizeof(sidePanelC4[0]));

  FastLED.addLeds<WS2812B, LED_BLUE_PANEL_FRONT_C9_PIN, GRB>(bluePanelFrontC9, sizeof(bluePanelFrontC9) / sizeof(bluePanelFrontC9[0]));
  FastLED.addLeds<WS2812B, LED_BLUE_PANEL_FRONT_C11_PIN, GRB>(bluePanelFrontC11, sizeof(bluePanelFrontC11) / sizeof(bluePanelFrontC11[0]));
  FastLED.addLeds<WS2812B, LED_BLUE_PANEL_SIDE_C1_PIN, GRB>(bluePanelSideC1, sizeof(bluePanelSideC1) / sizeof(bluePanelSideC1[0]));
  FastLED.addLeds<WS2812B, LED_BLUE_PANEL_SIDE_C2_PIN, GRB>(bluePanelSideC2, sizeof(bluePanelSideC2) / sizeof(bluePanelSideC2[0]));
  FastLED.addLeds<WS2812B, LED_BLUE_PANEL_SIDE_C3_PIN, GRB>(bluePanelSideC3, sizeof(bluePanelSideC3) / sizeof(bluePanelSideC3[0]));
  FastLED.addLeds<WS2812B, LED_BLUE_PANEL_SIDE_C4_PIN, GRB>(bluePanelSideC4, sizeof(bluePanelSideC4) / sizeof(bluePanelSideC4[0]));

  FastLED.addLeds<WS2812B, LED_STRIP_A_PIN, GRB>(stripA, kStripLedCount);
  FastLED.addLeds<WS2812B, LED_STRIP_B_PIN, GRB>(stripB, kStripLedCount);
  FastLED.addLeds<WS2812B, LED_STRIP_C_PIN, GRB>(stripC, kStripLedCount);
  FastLED.addLeds<WS2812B, LED_STRIP_D_PIN, GRB>(stripD, kStripLedCount);
  FastLED.addLeds<WS2812B, LED_STRIP_E_PIN, GRB>(stripE, kStripLedCount);
  FastLED.addLeds<WS2812B, LED_STRIP_F_PIN, GRB>(stripF, kStripLedCount);
  FastLED.addLeds<WS2812B, LED_STRIP_G_PIN, GRB>(stripG, kStripLedCount);

  FastLED.setBrightness(255);
}

static void setupServoDriver(Adafruit_PWMServoDriver& driver) {
  driver.begin();
  driver.setOscillatorFrequency(27000000);
  driver.setPWMFreq(50);
}

static void printStartupInfo() {
  Serial.println();
  Serial.println("PlanB front/side preview logic started");
  Serial.print("Servo units: ");
  Serial.println(servoUnitCount);
  Serial.print("LED logic units: ");
  Serial.println(ledUnitCount);
  Serial.print("LED signal outputs: ");
  Serial.println(kLedSignalCount);
  Serial.println("Panel columns: front PE0-PE10, side PD0-PD3");
  Serial.println("Blue 1x1 strips: PD4-PD9");
  Serial.println("Blue 2x2 strips: PD10-PD15, PC0");
  Serial.println("PCA9685: 0x40, 0x60, 0x50, 0x68");
  Serial.print("Active pattern: ");
  if (kActivePattern == Pattern::Ripple) {
    Serial.println("Ripple");
  } else if (kActivePattern == Pattern::Wander) {
    Serial.println("Wander");
  } else {
    Serial.println("Breath");
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);

  randomSeed(micros());
  buildLayout();

  planbI2C.begin();
  setupServoDriver(frontServoA);
  setupServoDriver(frontServoB);
  setupServoDriver(frontServoC);
  setupServoDriver(sideServo);
  delay(10);

  setupLedPins();
  allLedsOff();
  allServosStop();
  initWalkers(Surface::Front, 0);
  initWalkers(Surface::Side, 0);

  printStartupInfo();
}

void loop() {
  const uint32_t now = millis();
  if (now - lastFrameMs < kFrameMs) {
    return;
  }
  lastFrameMs = now;

  if (kActivePattern == Pattern::Wander) {
    updateWanderStates(now);
  }

  resetLedBuffers();

  for (uint8_t i = 0; i < ledUnitCount; i++) {
    const float value = ledValueForUnit(ledUnits[i], now);
    writeLedUnit(ledUnits[i], brightnessForValue(value));
  }
  flushLedBuffers();

  for (uint8_t i = 0; i < servoUnitCount; i++) {
    const float value = servoValueForUnit(servoUnits[i], now);
    writeServo(servoUnits[i], servoPulseForValue(value));
  }
}
