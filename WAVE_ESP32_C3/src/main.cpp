#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <FastLED.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ╔══════════════════════════════════════════════════════════════╗
// ║  双面双雷达机械彩灯交互系统 v5.6 (终极校准物理整合编译版)        ║
// ║  特色：双雷达独立并发 / 15路灯独立设色 / 52路舵机级联限制    ║
// ║        校准：大G和小f自动硬路由、PC2大B等参数终极统一锁定     ║
// ╚══════════════════════════════════════════════════════════════╝

// ---------------- Button and State Machine configuration ----------------

#define BUTTON_PIN PC5 // PC5 常闭物理开关输入引脚 [1]

enum SystemMode {
  MODE_RADAR = 0,         // 模式 0：雷达自动识别交互模式 (原 Plan A)
  MODE_PRESET_BREATH,     // 模式 1：预设-呼吸脉动动画 (原 Plan B Breath)
  MODE_PRESET_RIPPLE,     // 模式 2：预设-波纹涟漪动画 (原 Plan B Ripple)
  MODE_PRESET_WANDER,     // 模式 3：预设-随机游走动画 (原 Plan B Wander)
  NUM_MODES
};

SystemMode currentMode = MODE_RADAR;

// ---------------- Serial and radar configuration ----------------

#define DEBUG_SERIAL Serial

#define RADAR1_TX_PIN PB10
#define RADAR1_RX_PIN PB11
#define RADAR1_BAUD 256000
HardwareSerial radar1Serial(USART3);

#define RADAR2_TX_PIN PA2
#define RADAR2_RX_PIN PA3
#define RADAR2_BAUD 256000
HardwareSerial radar2Serial(USART2);

// ---------------- LED and color configuration ----------------

static constexpr uint16_t PANEL_LEDS = 256;
static constexpr uint16_t SMALL_PANEL_LEDS = 60;
static constexpr uint16_t BIG_PANEL_LEDS = 120;

// 🟢 统一定义色彩基准（包含您最新的标准紫色） [2]
static const CRGB COLOR_YELLOW(237, 223, 2); 
static const CRGB COLOR_BLUE(70, 150, 216);   
static const CRGB COLOR_PURPLE(237, 62, 216); // 👈 采用您最新校准的紫色 #ED3ED8

static constexpr uint8_t FRONT_LED_COUNT = 11;
Adafruit_NeoPixel stripFront[FRONT_LED_COUNT] = {
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE0, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(4 * PANEL_LEDS, PB2, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE2, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE3, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(2 * PANEL_LEDS, PE4, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(2 * PANEL_LEDS, PE5, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE6, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE7, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(1 * PANEL_LEDS, PE8, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE9, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(5 * PANEL_LEDS, PE10, NEO_GRB + NEO_KHZ800)
};

// 🟢 终极锁定的蓝色灯带实例定义与引脚线序
Adafruit_NeoPixel stripBigA(BIG_PANEL_LEDS, PE11, NEO_GRB + NEO_KHZ800); 
Adafruit_NeoPixel stripBigB(BIG_PANEL_LEDS, PC2,  NEO_RBG + NEO_KHZ800);  // 👈 PC2 + NEO_RBG 锁定
Adafruit_NeoPixel stripBigC(BIG_PANEL_LEDS, PE13, NEO_BRG + NEO_KHZ800);  
Adafruit_NeoPixel stripBigD(BIG_PANEL_LEDS, PE14, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripBigE(BIG_PANEL_LEDS, PE15, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel stripBigF(BIG_PANEL_LEDS, PC0,  NEO_GRB + NEO_KHZ800);

Adafruit_NeoPixel stripSmall_ac(3 * SMALL_PANEL_LEDS, PC1, NEO_GRB + NEO_KHZ800); 
Adafruit_NeoPixel stripSmall_d(1 * SMALL_PANEL_LEDS, PB1, NEO_RBG + NEO_KHZ800);   

Adafruit_NeoPixel stripBigG(BIG_PANEL_LEDS, PD4, NEO_GBR + NEO_KHZ800);   // 👈 GBR 锁定

Adafruit_NeoPixel stripSmall_hj(2 * SMALL_PANEL_LEDS, PD5, NEO_GRB + NEO_KHZ800); 
Adafruit_NeoPixel stripSmall_i(1 * SMALL_PANEL_LEDS, PD6, NEO_GRB + NEO_KHZ800);  
Adafruit_NeoPixel stripSmall_f(1 * SMALL_PANEL_LEDS, PD7, NEO_GBR + NEO_KHZ800);   // 👈 GBR 锁定
Adafruit_NeoPixel stripSmall_eg(2 * SMALL_PANEL_LEDS, PD8, NEO_GRB + NEO_KHZ800);  

static constexpr uint8_t SIDE_LED_COUNT = 4;
Adafruit_NeoPixel stripSide[SIDE_LED_COUNT] = {
  Adafruit_NeoPixel(3 * PANEL_LEDS, PD0, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(4 * PANEL_LEDS, PD1, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(4 * PANEL_LEDS, PD2, NEO_GRB + NEO_KHZ800),
  Adafruit_NeoPixel(3 * PANEL_LEDS, PD3, NEO_GRB + NEO_KHZ800)
};

// ---------------- Servo PCA9685 configuration ----------------

#define SERVO_SDA PB7
#define SERVO_SCL PB6
TwoWire myI2C(SERVO_SDA, SERVO_SCL);

Adafruit_PWMServoDriver pwm1(0x40, myI2C);
Adafruit_PWMServoDriver pwm2(0x60, myI2C);
Adafruit_PWMServoDriver pwm3(0x50, myI2C);
Adafruit_PWMServoDriver pwm4(0x68, myI2C);

static constexpr uint16_t SERVO_STOP_PULSE = 310;

const int BOX_TO_SERVO_CHAN_SIDE[14] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
const int BOX_TO_COL_SIDE[14] = {
  0, 1, 2, 2, 3, 2, 3, 1, 2, 3, 0, 1, 2, 3
};

Adafruit_PWMServoDriver* const kServoDrivers[] = {
  &pwm1,
  &pwm2,
  &pwm3,
  &pwm4
};

// ---------------- Radar data and interaction constants ----------------

static constexpr uint8_t FRAME_LEN = 30;
static constexpr uint8_t BUFFER_SIZE = 128;
static constexpr double DIST_E_MAX = 0.25;
static constexpr double DIST_D_MAX = 0.50;
static constexpr double DIST_C_MAX = 0.90;
static constexpr double DIST_B_MAX = 1.40;
static constexpr double DIST_A_MAX = 2.00;
static constexpr uint8_t LED_BRIGHTNESS_MIN = 2;
static constexpr uint8_t LED_BRIGHTNESS_MAX = 20;
static constexpr uint8_t FRONT_ZONES = 11;
static constexpr uint8_t SIDE_ZONES = 4;

struct KalmanFilter {
  double x_est;
  double p_est;
  bool init;
};

struct KFPair {
  KalmanFilter kfX;
  KalmanFilter kfY;
};

struct Target {
  double x, y;
  double distance;
  double angle;
  double speed;
  double resolution;
  bool valid;
  double fx, fy;
  double fDist;
  double fAngle;
};

struct RadarStream {
  uint8_t ringBuf[BUFFER_SIZE];
  int ringHead;
  int ringTail;
  int ringCount;
  uint32_t byteCount;
  uint32_t validFrameCount;
  uint32_t fakeHeadCount;
  uint32_t noHeadCount;
  uint32_t lastValidFrameTime;
  Target targets[3];
  KFPair kf[3];
};

RadarStream r1;
RadarStream r2;

bool colActive1[FRONT_ZONES] = {false};
double colDistance1[FRONT_ZONES] = {0};
bool colActive2[SIDE_ZONES] = {false};
double colDistance2[SIDE_ZONES] = {0};
bool zoneActive1[FRONT_ZONES][5] = {};
bool zoneActive2[SIDE_ZONES][5] = {};

uint32_t lastOutputTime = 0;
uint32_t lastStatusTime = 0;
uint32_t lastNoFrameNoticeMs = 0;

const int FRONT_STRIP_BOX_MAP[11][5] = {
  {28, 17, 11, 1, -1},
  {29, 18, 12, 2, -1},
  {30, 19, 13, 3, -1},
  {31, 20, 14, 4, -1},
  {32, 5, -1, -1, -1},
  {33, 6, -1, -1, -1},
  {34, 26, 23, 7, -1},
  {35, 27, 24, 8, -1},
  {36, -1, -1, -1, -1},
  {37, 21, 15, 9, -1},
  {38, 25, 22, 16, 10}
};

const int SIDE_STRIP_BOX_MAP[4][4] = {
  {11, 4, 1, -1},
  {12, 8, 5, 2},
  {13, 9, 6, 3},
  {14, 10, 7, -1}
};

const int FRONT_BOX_TO_COL[38] = {
  0, 1, 2, 3, 4, 5, 6, 7, 9, 10,
  0, 1, 2, 3, 9, 10,
  0, 1, 2, 3, 9, 10,
  6, 7, 10,
  6, 7,
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
};

// ---------------- Preset logic configuration (from Plan B) ----------------

static constexpr uint8_t kFrontRows = 6;
static constexpr uint8_t kFrontCols = 11;
static constexpr uint8_t kSideRows = 6;
static constexpr uint8_t kSideCols = 4;
static constexpr uint8_t kMaxRows = 6;
static constexpr uint8_t kMaxCols = 11;

static constexpr float kEffectSpeed = 1.0f;
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

static constexpr uint16_t kServoStopUs = 1500;
static constexpr uint16_t kServoMinRunUs = 1540;
static constexpr uint16_t kServoMaxRunUs = 1740;

enum class Surface : uint8_t { Front, Side };
enum class CellColor : uint8_t { Blue, Yellow, Purple };
enum class LedKind : uint8_t { Panel, Strip };

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
  {3, 8, 2, 2}, 
  {1, 6, 2, 2}, 
  {1, 4, 2, 2}, 
  {3, 4, 2, 2}, 
  {3, 2, 2, 2}, 
  {3, 0, 2, 2}  
};

static const MergedBlock kSideMergedBlocks[] = {
  {1, 0, 2, 2}  
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

// ── 转发声明区 ──
double   calcCoordinate(int low, int high);
double   calcUint16(int low, int high);
double   kalmanUpdate(KalmanFilter& kf, double measurement);
int      distToLayer(double dist);
int      distToBrightness(double dist);
uint8_t  activeBrightness(bool active, double dist);
int8_t   distToServoSpeed(double dist);
uint32_t scaledColor(const CRGB& base, uint8_t brightness);
uint32_t getBoxColor(int boxId, bool isFront, uint8_t brightness);
void     set60ServoSpeed(Adafruit_PWMServoDriver& driver, uint8_t channel, int8_t speed);
void     setFrontServoSpeed(uint8_t boxId, int8_t speed);
void     fillStrip(Adafruit_NeoPixel& strip, uint32_t color);
void     scanI2C();
void     initOneStrip(Adafruit_NeoPixel& strip);
void     initLeds();
void     bootLedSelfTest();
void     setupPwmDriver(Adafruit_PWMServoDriver& driver, const char* name);
void     stopAllServos();
void     clearZones();
void     markFrontZone(double angle, double dist);
void     markSideZone(double angle, double dist);
void     summarizeAllZones();
void     driveOutputs();
void     printStatus();
void     parseFrame(uint8_t* frame, bool isRadar1);
void     processFrameAll();
void     parseFromRingBuffer(bool isRadar1);
void     enableRadarEngineeringMode(HardwareSerial& serialPort, const char* name);
void     readRadarSerial(HardwareSerial& serialPort, RadarStream& r);

float clamp01(float value);
float wave01(float phase);
float gamma01(float value);
uint8_t brightnessForValue(float value);
CRGB colorForCellColor(CellColor color);
uint16_t servoPulseForValue(float value);
uint32_t scaledPeriod(uint32_t periodMs);
uint32_t scaledStep(uint32_t stepMs);
uint8_t rowsForSurface(Surface surface);
uint8_t colsForSurface(Surface surface);
float phaseOffsetForSurface(Surface surface);
bool isPurpleBox(Surface surface, uint8_t box);
CellColor colorForBox(Surface surface, int8_t box);
char ledLabelFor(Surface surface, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan);
bool blockCovers(const MergedBlock& block, uint8_t row, uint8_t col);
const MergedBlock* mergedBlockAt(const MergedBlock* blocks, uint8_t blockCount, uint8_t row, uint8_t col);
bool isCoveredByMergedBlock(const MergedBlock* blocks, uint8_t blockCount, uint8_t row, uint8_t col);
int8_t gridBoxAt(Surface surface, uint8_t row, uint8_t col);
bool servoRouteFor(Surface surface, uint8_t box, uint8_t& driverIndex, uint8_t& channel);
void addLedUnit(Surface surface, int8_t box, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan);
void addServoUnit(Surface surface, uint8_t box, uint8_t row, uint8_t col);
void buildSurfaceLayout(Surface surface, const MergedBlock* blocks, uint8_t blockCount);
void buildLayout();
RippleState& rippleForSurface(Surface surface);
void chooseRippleCenter(Surface surface, RippleState& ripple);
float maxDistanceFromCenter(Surface surface, const RippleState& ripple);
float rippleValue(Surface surface, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan, uint32_t now);
WanderState& wanderForSurface(Surface surface);
void clearVisited(Walker& walker);
void initWalker(Surface surface, Walker& walker, int32_t step);
void initWalkers(Surface surface, int32_t step);
void advanceWalker(Surface surface, Walker& walker, int32_t step);
void updateWanderSurface(Surface surface, int32_t currentStep);
void updateWanderStates(uint32_t now);
bool positionCoveredByUnit(const LedUnit& unit, uint8_t row, uint8_t col);
float wanderTrailValue(const LedUnit& unit, uint32_t now);
float wanderTrailValueForServo(const ServoUnit& servo, uint32_t now);
float wanderBlueBreathValue(const LedUnit& unit, uint32_t now);
float breathLedValue(Surface surface, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan, uint32_t now);
float breathServoValue(const ServoUnit& servo, uint32_t now);
uint8_t panelSegmentIndexFor(const LedUnit& unit);

// 新设：独立物理校准核心函数
uint32_t getCalibratedColor(int boxId, bool isFront, uint8_t brightness);
uint32_t getNeoColorForUnit(const LedUnit& unit, uint8_t brightness);
void writeLedUnitToNeoPixelDirect(const LedUnit& unit, uint32_t neoColor);
void writeServo(const ServoUnit& servo, uint16_t pulseUs);
void allServosStop();

// ---------------- Utility helpers ----------------

static void debugBegin() {
  DEBUG_SERIAL.begin(115200);
  const uint32_t startMs = millis();
  while (!DEBUG_SERIAL && millis() - startMs < 3000) {
    delay(10);
  }
}

static void logLine(const char* msg) {
  DEBUG_SERIAL.println(msg);
}

double calcCoordinate(int low, int high) {
  int raw = low + high * 256;
  if (raw & 0x8000) {
    return (double)raw - 32768.0;
  }
  return (double)(0 - raw);
}

double calcUint16(int low, int high) {
  return (double)(low + high * 256);
}

double kalmanUpdate(KalmanFilter& kf, double measurement) {
  if (!kf.init) {
    kf.x_est = measurement;
    kf.p_est = 1.0;
    kf.init = true;
    return measurement;
  }
  const double p_pred = kf.p_est + 0.05;
  const double K = p_pred / (p_pred + 2.0);
  kf.x_est = kf.x_est + K * (measurement - kf.x_est);
  kf.p_est = (1.0 - K) * p_pred;
  return kf.x_est;
}

int distToLayer(double dist) {
  if (dist > DIST_B_MAX) return 0;
  if (dist > DIST_C_MAX) return 1;
  if (dist > DIST_D_MAX) return 2;
  if (dist > DIST_E_MAX) return 3;
  return 4;
}

int distToBrightness(double dist) {
  double ratio = 1.0 - (dist / DIST_A_MAX);
  ratio = constrain(ratio, 0.0, 1.0);
  return (int)(LED_BRIGHTNESS_MIN + ratio * (LED_BRIGHTNESS_MAX - LED_BRIGHTNESS_MIN));
}

uint8_t activeBrightness(bool active, double dist) {
  return active ? (uint8_t)distToBrightness(dist) : 0;
}

int8_t distToServoSpeed(double dist) {
  double ratio = 1.0 - (dist / DIST_A_MAX);
  ratio = constrain(ratio, 0.0, 1.0);
  return (int8_t)(10 + ratio * (80 - 20));
}

uint32_t scaledColor(const CRGB& base, uint8_t brightness) {
  const double scale = (double)brightness / (double)LED_BRIGHTNESS_MAX;
  return Adafruit_NeoPixel::Color(
    (uint8_t)(base.r * scale),
    (uint8_t)(base.g * scale),
    (uint8_t)(base.b * scale)
  );
}

// ---------------- 🟢 终极色彩核心引擎（大G和小f自动硬路由校准） [2] ----------------

uint32_t getCalibratedColor(int boxId, bool isFront, uint8_t brightness) {
  bool isBlue = false;
  bool isPurple = false;

  if (isFront) {
    if (boxId >= 101 || boxId >= 201) {
      isBlue = true;
    } else if (boxId == 6 || boxId == 7 || boxId == 9 || boxId == 13 ||
               boxId == 15 || boxId == 19 || boxId == 23 || boxId == 26 ||
               boxId == 28 || boxId == 29 || boxId == 31 || boxId == 33 ||
               boxId == 36 || boxId == 37) {
      isPurple = true;
    }
  } else {
    if (boxId == 301 || boxId >= 401) {
      isBlue = true;
    } else if (boxId == 4 || boxId == 5 || boxId == 8 || boxId == 9) {
      isPurple = true;
    }
  }

  if (isBlue) {
    // 🟢 特殊通路：针对 G（301）和 f（402），当系统命令其亮蓝色时，自动注入校准色 (0, 150, 20) [2]
    if (!isFront) {
      if (boxId == 301 || boxId == 402) {
        return scaledColor(CRGB(0, 150, 20), brightness);
      }
    }
    return scaledColor(COLOR_BLUE, brightness);
  }
  if (isPurple) {
    return scaledColor(COLOR_PURPLE, brightness);
  }
  return scaledColor(COLOR_YELLOW, brightness);
}

uint32_t getBoxColor(int boxId, bool isFront, uint8_t brightness) {
  return getCalibratedColor(boxId, isFront, brightness);
}

void set60ServoSpeed(Adafruit_PWMServoDriver& driver, uint8_t channel, int8_t speed) {
  uint16_t pulse;
  if (speed == 0) {
    pulse = SERVO_STOP_PULSE;
  } else if (speed > 0) {
    pulse = map(speed, 0, 100, SERVO_STOP_PULSE, 385);
  } else {
    pulse = map(speed, -100, 0, 230, SERVO_STOP_PULSE);
  }
  driver.setPWM(channel, 0, pulse);
}

void setFrontServoSpeed(uint8_t boxId, int8_t speed) {
  if (boxId < 1 || boxId > 38) return;
  if (boxId <= 16) {
    set60ServoSpeed(pwm1, boxId - 1, speed);
  } else if (boxId <= 32) {
    set60ServoSpeed(pwm2, boxId - 17, speed);
  } else {
    set60ServoSpeed(pwm3, boxId - 33, speed);
  }
}

void fillStrip(Adafruit_NeoPixel& strip, uint32_t color) {
  strip.fill(color, 0, strip.numPixels()); 
  strip.show();
}

void scanI2C() {
  DEBUG_SERIAL.println("I2C scan start");
  uint8_t found = 0;
  for (uint8_t address = 1; address < 127; address++) {
    myI2C.beginTransmission(address);
    const uint8_t error = myI2C.endTransmission();
    if (error == 0) {
      DEBUG_SERIAL.print("I2C found 0x");
      if (address < 16) DEBUG_SERIAL.print('0');
      DEBUG_SERIAL.println(address, HEX);
      found++;
    }
  }
  DEBUG_SERIAL.print("I2C scan done, found ");
  DEBUG_SERIAL.println(found);
}

void initOneStrip(Adafruit_NeoPixel& strip) {
  strip.begin();
  strip.setBrightness(30); // 👈 在此行强行写死！即便有代码产生了 255 的颜色，也会被强制等比例缩减至 30
  strip.clear();
  strip.show();
}

void initLeds() {
  logLine("LED init start");
  for (int c = 0; c < FRONT_LED_COUNT; c++) initOneStrip(stripFront[c]);
  initOneStrip(stripBigA);
  initOneStrip(stripBigB);
  initOneStrip(stripBigC);
  initOneStrip(stripBigD);
  initOneStrip(stripBigE);
  initOneStrip(stripBigF);
  initOneStrip(stripSmall_ac);
  initOneStrip(stripSmall_d);
  for (int c = 0; c < SIDE_LED_COUNT; c++) initOneStrip(stripSide[c]);
  initOneStrip(stripBigG);
  initOneStrip(stripSmall_hj);
  initOneStrip(stripSmall_i);
  initOneStrip(stripSmall_f);
  initOneStrip(stripSmall_eg);
  logLine("LED init done");
}

void bootLedSelfTest() {
  logLine("LED self test start");
  fillStrip(stripFront[0], scaledColor(COLOR_YELLOW, 8));
  fillStrip(stripSide[0], scaledColor(COLOR_PURPLE, 8));
  fillStrip(stripBigG, scaledColor(COLOR_BLUE, 8));
  delay(450);
  fillStrip(stripFront[0], 0);
  fillStrip(stripSide[0], 0);
  fillStrip(stripBigG, 0);
  logLine("LED self test done");
}

void setupPwmDriver(Adafruit_PWMServoDriver& driver, const char* name) {
  DEBUG_SERIAL.print("PCA9685 ");
  DEBUG_SERIAL.print(name);
  DEBUG_SERIAL.println(" begin");
  driver.begin();
  driver.setOscillatorFrequency(27000000);
  driver.setPWMFreq(50);
  delay(10);
  DEBUG_SERIAL.print("PCA9685 ");
  DEBUG_SERIAL.print(name);
  DEBUG_SERIAL.println(" ready");
}

void stopAllServos() {
  for (int b = 0; b < 38; b++) setFrontServoSpeed(b + 1, 0);
  for (int b = 0; b < 14; b++) set60ServoSpeed(pwm4, BOX_TO_SERVO_CHAN_SIDE[b], 0);
}

// ---------------- Zone and output logic (Plan A) ----------------

void clearZones() {
  for (int c = 0; c < FRONT_ZONES; c++) {
    for (int l = 0; l < 5; l++) zoneActive1[c][l] = false;
  }
  for (int c = 0; c < SIDE_ZONES; c++) {
    for (int l = 0; l < 5; l++) zoneActive2[c][l] = false;
  }
}

void markFrontZone(double angle, double dist) {
  if (dist <= 0 || dist > DIST_A_MAX) return;
  const int layer = distToLayer(dist);
  if (angle >= -40.0 && angle <= -28.0) zoneActive1[0][layer] = true;
  if (angle >= -32.0 && angle <= -20.0) zoneActive1[1][layer] = true;
  if (angle >= -24.0 && angle <= -12.0) zoneActive1[2][layer] = true;
  if (angle >= -16.0 && angle <= -4.0)  zoneActive1[3][layer] = true;
  if (angle >= -8.0 && angle <= 4.0) zoneActive1[4][layer] = true;
  if (angle >= -4.0 && angle <= 12.0) zoneActive1[5][layer] = true;
  if (angle >= 4.0 && angle <= 20.0) zoneActive1[6][layer] = true;
  if (angle >= 12.0 && angle <= 28.0) zoneActive1[7][layer] = true;
  if (angle >= 20.0 && angle <= 32.0) zoneActive1[8][layer] = true;
  if (angle >= 28.0 && angle <= 36.0) zoneActive1[9][layer] = true;
  if (angle >= 32.0 && angle <= 40.0) zoneActive1[10][layer] = true;
}

void markSideZone(double angle, double dist) {
  if (dist <= 0 || dist > DIST_A_MAX) return;
  const int layer = distToLayer(dist);
  if (angle >= -40.0 && angle <= -15.0) zoneActive2[0][layer] = true;
  if (angle >= -22.0 && angle <= 5.0) zoneActive2[1][layer] = true;
  if (angle >= -5.0 && angle <= 22.0) zoneActive2[2][layer] = true;
  if (angle >= 15.0 && angle <= 40.0) zoneActive2[3][layer] = true;
}

void summarizeAllZones() {
  const double layerDist[5] = {
    (DIST_B_MAX + DIST_A_MAX) / 2.0,
    (DIST_C_MAX + DIST_B_MAX) / 2.0,
    (DIST_D_MAX + DIST_C_MAX) / 2.0,
    (DIST_E_MAX + DIST_D_MAX) / 2.0,
    DIST_E_MAX / 2.0
  };

  for (int c = 0; c < FRONT_ZONES; c++) {
    colActive1[c] = false;
    colDistance1[c] = DIST_A_MAX;
    double closest = DIST_A_MAX + 1.0;
    for (int l = 0; l < 5; l++) {
      if (zoneActive1[c][l]) {
        colActive1[c] = true;
        if (layerDist[l] < closest) closest = layerDist[l];
      }
    }
    if (colActive1[c]) colDistance1[c] = closest;
  }

  for (int c = 0; c < SIDE_ZONES; c++) {
    colActive2[c] = false;
    colDistance2[c] = DIST_A_MAX;
    double closest = DIST_A_MAX + 1.0;
    for (int l = 0; l < 5; l++) {
      if (zoneActive2[c][l]) {
        colActive2[c] = true;
        if (layerDist[l] < closest) closest = layerDist[l];
      }
    }
    if (colActive2[c]) colDistance2[c] = closest;
  }
}

void driveOutputs() {
  static bool lastInit = false;
  static uint8_t lastFrontBri[11];
  static uint8_t lastBigSmallBri[8];
  static uint8_t lastSideBri[4];
  static uint8_t lastSideBigSmallBri[5];
  static int8_t lastFrontSpeed[38];
  static int8_t lastSideSpeed[14];

  if (!lastInit) {
    memset(lastFrontBri, 255, sizeof(lastFrontBri));
    memset(lastBigSmallBri, 255, sizeof(lastBigSmallBri));
    memset(lastSideBri, 255, sizeof(lastSideBri));
    memset(lastSideBigSmallBri, 255, sizeof(lastSideBigSmallBri));
    memset(lastFrontSpeed, 127, sizeof(lastFrontSpeed));
    memset(lastSideSpeed, 127, sizeof(lastSideSpeed));
    lastInit = true;
  }

  bool frontLedChanged = false;
  for (int c = 0; c < FRONT_ZONES; c++) {
    const uint8_t bri = activeBrightness(colActive1[c], colDistance1[c]);
    if (bri == lastFrontBri[c]) continue;
    lastFrontBri[c] = bri;
    frontLedChanged = true;
    for (int seg = 0; seg < 5; seg++) {
      const int boxId = FRONT_STRIP_BOX_MAP[c][seg];
      if (boxId == -1) break;
      const uint32_t color = getBoxColor(boxId, true, bri);
      for (int i = 0; i < PANEL_LEDS; i++) {
        stripFront[c].setPixelColor(seg * PANEL_LEDS + i, color);
      }
    }
  }

  const uint8_t frontAuxBri[8] = {
    activeBrightness(colActive1[8], colDistance1[8]),
    activeBrightness(colActive1[6], colDistance1[6]),
    activeBrightness(colActive1[4], colDistance1[4]),
    activeBrightness(colActive1[4], colDistance1[4]),
    activeBrightness(colActive1[2], colDistance1[2]),
    activeBrightness(colActive1[0], colDistance1[0]),
    activeBrightness(colActive1[8], colDistance1[8]),
    activeBrightness(colActive1[10], colDistance1[10])
  };
  Adafruit_NeoPixel* frontAuxStrips[8] = {
    &stripBigA, &stripBigB, &stripBigC, &stripBigD,
    &stripBigE, &stripBigF, &stripSmall_ac, &stripSmall_d
  };
  const int frontAuxIds[8] = {101, 102, 103, 104, 105, 106, 201, 204};
  for (int i = 0; i < 8; i++) {
    if (frontAuxBri[i] != lastBigSmallBri[i]) {
      lastBigSmallBri[i] = frontAuxBri[i];
      fillStrip(*frontAuxStrips[i], getBoxColor(frontAuxIds[i], true, frontAuxBri[i]));
    }
  }
  if (frontLedChanged) {
    for (int c = 0; c < FRONT_ZONES; c++) stripFront[c].show();
  }

  bool sideLedChanged = false;
  for (int c = 0; c < SIDE_ZONES; c++) {
    const uint8_t bri = activeBrightness(colActive2[c], colDistance2[c]);
    if (bri == lastSideBri[c]) continue;
    lastSideBri[c] = bri;
    sideLedChanged = true;
    for (int seg = 0; seg < 4; seg++) {
      const int boxId = SIDE_STRIP_BOX_MAP[c][seg];
      if (boxId == -1) break;
      const uint32_t color = getBoxColor(boxId, false, bri);
      for (int i = 0; i < PANEL_LEDS; i++) {
        stripSide[c].setPixelColor(seg * PANEL_LEDS + i, color);
      }
    }
  }

  const uint8_t sideAuxBri[5] = {
    activeBrightness(colActive2[0], colDistance2[0]),
    activeBrightness(colActive2[0], colDistance2[0]),
    activeBrightness(colActive2[1], colDistance2[1]),
    activeBrightness(colActive2[2], colDistance2[2]),
    activeBrightness(colActive2[3], colDistance2[3])
  };
  Adafruit_NeoPixel* sideAuxStrips[5] = {
    &stripBigG, &stripSmall_hj, &stripSmall_i, &stripSmall_f, &stripSmall_eg
  };
  const int sideAuxIds[5] = {301, 404, 405, 402, 401};
  for (int i = 0; i < 5; i++) {
    if (sideAuxBri[i] != lastSideBigSmallBri[i]) {
      lastSideBigSmallBri[i] = sideAuxBri[i];
      fillStrip(*sideAuxStrips[i], getBoxColor(sideAuxIds[5], false, sideAuxBri[i]));
    }
  }
  if (sideLedChanged) {
    for (int c = 0; c < SIDE_ZONES; c++) stripSide[c].show();
  }

  for (int b = 0; b < 38; b++) {
    const int col = FRONT_BOX_TO_COL[b];
    const int8_t speed = colActive1[col] ? distToServoSpeed(colDistance1[col]) : 0;
    if (speed != lastFrontSpeed[b]) {
      lastFrontSpeed[b] = speed;
      setFrontServoSpeed(b + 1, speed);
    }
  }

  for (int b = 0; b < 14; b++) {
    const int col = BOX_TO_COL_SIDE[b];
    const int channel = BOX_TO_SERVO_CHAN_SIDE[b];
    const int8_t speed = colActive2[col] ? distToServoSpeed(colDistance2[col]) : 0;
    if (speed != lastSideSpeed[b]) {
      lastSideSpeed[b] = speed;
      set60ServoSpeed(pwm4, channel, speed);
    }
  }
}

void printStatus() {
  DEBUG_SERIAL.print("\033[H"); 

  DEBUG_SERIAL.println("════════════════════════════════════════════════════════");
  DEBUG_SERIAL.println("  双面双雷达机械彩灯交互系统已上线 (MKS-F407ZET)          ");
  DEBUG_SERIAL.println("════════════════════════════════════════════════════════");

  DEBUG_SERIAL.println("[正面 11 路列监视]");
  for (int c = 0; c < 11; c++) {
    DEBUG_SERIAL.print(" 列"); DEBUG_SERIAL.print(c + 1);
    if (colActive1[c]) {
      DEBUG_SERIAL.print("【动】距:"); DEBUG_SERIAL.print(colDistance1[c], 1); 
      DEBUG_SERIAL.print("m 亮:"); DEBUG_SERIAL.print(distToBrightness(colDistance1[c])); DEBUG_SERIAL.print(" ");
    } else {
      DEBUG_SERIAL.print("【静】距:--m 亮: 0 ");
    }
    if ((c + 1) % 4 == 0) DEBUG_SERIAL.println();
  }
  DEBUG_SERIAL.println("\n--------------------------------------------------------");

  DEBUG_SERIAL.println("[侧面 4 路列监视]");
  for (int c = 0; c < 4; c++) {
    DEBUG_SERIAL.print(" 列"); DEBUG_SERIAL.print(c + 1);
    if (colActive2[c]) {
      DEBUG_SERIAL.print("【动】距:"); DEBUG_SERIAL.print(colDistance2[c], 1); 
      DEBUG_SERIAL.print("m 亮:"); DEBUG_SERIAL.print(distToBrightness(colDistance2[c])); DEBUG_SERIAL.print(" |");
    } else {
      DEBUG_SERIAL.print("【静】距:--m 亮: 0 |");
    }
  }
  DEBUG_SERIAL.println("\n--------------------------------------------------------");

  DEBUG_SERIAL.println("[正面雷达 1 目标轨迹监测]");
  bool anyFront = false;
  for (int i = 0; i < 3; i++) {
    if (r1.targets[i].valid) {
      anyFront = true;
      DEBUG_SERIAL.print("  ↳ 目标"); DEBUG_SERIAL.print(i + 1);
      DEBUG_SERIAL.print(" 原始("); DEBUG_SERIAL.print(r1.targets[i].x, 0);
      DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(r1.targets[i].y, 0); DEBUG_SERIAL.print(")");
      DEBUG_SERIAL.print(" 滤波("); DEBUG_SERIAL.print(r1.targets[i].fx, 0);
      DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(r1.targets[i].fy, 0); DEBUG_SERIAL.print(")");
      DEBUG_SERIAL.print(" 角="); DEBUG_SERIAL.print(r1.targets[i].fAngle, 1);
      DEBUG_SERIAL.print("° 距="); DEBUG_SERIAL.print(r1.targets[i].fDist, 2); DEBUG_SERIAL.println("m      ");
    } else {
      DEBUG_SERIAL.println("                                                                    "); 
    }
  }
  if (!anyFront) DEBUG_SERIAL.println("  (当前暂无有效正面目标)                                           ");

  DEBUG_SERIAL.println("--------------------------------------------------------");

  DEBUG_SERIAL.println("[侧面雷达 2 目标轨迹监测]");
  bool anySide = false;
  for (int i = 0; i < 3; i++) {
    if (r2.targets[i].valid) {
      anySide = true;
      DEBUG_SERIAL.print("  ↳ 目标"); DEBUG_SERIAL.print(i + 1);
      DEBUG_SERIAL.print(" 原始("); DEBUG_SERIAL.print(r2.targets[i].x, 0);
      DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(r2.targets[i].y, 0); DEBUG_SERIAL.print(")");
      DEBUG_SERIAL.print(" 滤波("); DEBUG_SERIAL.print(r2.targets[i].fx, 0);
      DEBUG_SERIAL.print(","); DEBUG_SERIAL.print(r2.targets[i].fy, 0); DEBUG_SERIAL.print(")");
      DEBUG_SERIAL.print(" 角="); DEBUG_SERIAL.print(r2.targets[i].fAngle, 1);
      DEBUG_SERIAL.print("° 距="); DEBUG_SERIAL.print(r2.targets[i].fDist, 2); DEBUG_SERIAL.println("m      ");
    } else {
      DEBUG_SERIAL.println("                                                                    "); 
    }
  }
  if (!anySide) DEBUG_SERIAL.println("  (当前暂无有效侧面目标)                                           ");

  DEBUG_SERIAL.println("════════════════════════════════════════════════════════");
  DEBUG_SERIAL.print("\033[J"); 
}

void parseFrame(uint8_t* frame, bool isRadar1) {
  int off = 4;
  RadarStream& r = isRadar1 ? r1 : r2;

  for (int i = 0; i < 3; i++) {
    const int xL = frame[off + 0], xH = frame[off + 1];
    const int yL = frame[off + 2], yH = frame[off + 3];
    const int sL = frame[off + 4], sH = frame[off + 5];
    const int rL = frame[off + 6], rH = frame[off + 7];
    off += 8;

    const bool allZero = !xL && !xH && !yL && !yH && !sL && !sH && !rL && !rH;
    if (allZero) {
      r.targets[i].valid = false;
      r.kf[i].kfX.init = false;
      r.kf[i].kfY.init = false;
      continue;
    }

    r.targets[i].x = calcCoordinate(xL, xH);
    r.targets[i].y = calcCoordinate(yL, yH);
    r.targets[i].speed = calcCoordinate(sL, sH);
    r.targets[i].resolution = calcUint16(rL, rH);
    r.targets[i].distance = sqrt(r.targets[i].x * r.targets[i].x + r.targets[i].y * r.targets[i].y) / 1000.0;
    r.targets[i].angle = atan2(r.targets[i].x, r.targets[i].y) * 180.0 / PI;
    r.targets[i].valid = true;

    r.targets[i].fx = kalmanUpdate(r.kf[i].kfX, r.targets[i].x);
    r.targets[i].fy = kalmanUpdate(r.kf[i].kfY, r.targets[i].y);
    r.targets[i].fDist = sqrt(r.targets[i].fx * r.targets[i].fx + r.targets[i].fy * r.targets[i].fy) / 1000.0;
    r.targets[i].fAngle = atan2(r.targets[i].fx, r.targets[i].fy) * 180.0 / PI;
  }
}

void processFrameAll() {
  clearZones();
  for (int i = 0; i < 3; i++) {
    if (r1.targets[i].valid) markFrontZone(r1.targets[i].fAngle, r1.targets[i].fDist);
    if (r2.targets[i].valid) markSideZone(r2.targets[i].fAngle, r2.targets[i].fDist);
  }
  summarizeAllZones();
  driveOutputs();
}

void parseFromRingBuffer(bool isRadar1) {
  RadarStream& r = isRadar1 ? r1 : r2;

  while (r.ringCount >= FRAME_LEN) {
    int frameStartOffset = -1;
    for (int i = 0; i <= r.ringCount - FRAME_LEN; i++) {
      const int p0 = (r.ringTail + i + 0) % BUFFER_SIZE;
      const int p1 = (r.ringTail + i + 1) % BUFFER_SIZE;
      const int p2 = (r.ringTail + i + 2) % BUFFER_SIZE;
      const int p3 = (r.ringTail + i + 3) % BUFFER_SIZE;
      if (r.ringBuf[p0] == 0xAA && r.ringBuf[p1] == 0xFF &&
          r.ringBuf[p2] == 0x03 && r.ringBuf[p3] == 0x00) {
        frameStartOffset = i;
        break;
      }
    }

    if (frameStartOffset == -1) {
      const int drop = r.ringCount - FRAME_LEN + 1;
      r.ringTail = (r.ringTail + drop) % BUFFER_SIZE;
      r.ringCount -= drop;
      r.noHeadCount++;
      return;
    }

    if (frameStartOffset > 0) {
      r.ringTail = (r.ringTail + frameStartOffset) % BUFFER_SIZE;
      r.ringCount -= frameStartOffset;
    }
    if (r.ringCount < FRAME_LEN) return;

    const int tail1 = (r.ringTail + 28) % BUFFER_SIZE;
    const int tail2 = (r.ringTail + 29) % BUFFER_SIZE;
    if (r.ringBuf[tail1] == 0x55 && r.ringBuf[tail2] == 0xCC) {
      uint8_t frame[FRAME_LEN];
      for (int i = 0; i < FRAME_LEN; i++) frame[i] = r.ringBuf[(r.ringTail + i) % BUFFER_SIZE];
      r.validFrameCount++;
      r.lastValidFrameTime = millis();

      parseFrame(frame, isRadar1);
      processFrameAll();

      r.ringTail = (r.ringTail + FRAME_LEN) % BUFFER_SIZE;
      r.ringCount -= FRAME_LEN;
    } else {
      r.ringTail = (r.ringTail + 1) % BUFFER_SIZE;
      r.ringCount--;
      r.fakeHeadCount++;
    }
  }
}

void enableRadarEngineeringMode(HardwareSerial& serialPort, const char* name) {
  static const uint8_t enterConfig[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x04, 0x00, 0xFF, 0x00, 0x01, 0x00, 0x04, 0x03, 0x02, 0x01};
  static const uint8_t enableEng[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0x62, 0x00, 0x04, 0x03, 0x02, 0x01};
  static const uint8_t exitConfig[] = {0xFD, 0xFC, 0xFB, 0xFA, 0x02, 0x00, 0xFE, 0x00, 0x04, 0x03, 0x02, 0x01};
  DEBUG_SERIAL.print(name);
  DEBUG_SERIAL.println(" engineering mode command");
  serialPort.write(enterConfig, sizeof(enterConfig));
  delay(150);
  serialPort.write(enableEng, sizeof(enableEng));
  delay(150);
  serialPort.write(exitConfig, sizeof(exitConfig));
  delay(150);
}

void readRadarSerial(HardwareSerial& serialPort, RadarStream& r) {
  while (serialPort.available()) {
    const uint8_t b = serialPort.read();
    r.byteCount++;
    r.ringBuf[r.ringHead] = b;
    r.ringHead = (r.ringHead + 1) % BUFFER_SIZE;
    if (r.ringCount < BUFFER_SIZE) {
      r.ringCount++;
    } else {
      r.ringTail = (r.ringTail + 1) % BUFFER_SIZE;
    }
  }
}

// ---------------- Preset mode implementations (from Plan B) ----------------

float clamp01(float value) {
  if (value < 0.0f) return 0.0f;
  if (value > 1.0f) return 1.0f;
  return value;
}

float wave01(float phase) {
  return 0.5f - 0.5f * cosf(phase);
}

float gamma01(float value) {
  value = clamp01(value);
  return value * value;
}

uint8_t brightnessForValue(float value) {
  const float corrected = gamma01(value);
  return (uint8_t)roundf(kLedMinBrightness + corrected * (kLedMaxBrightness - kLedMinBrightness));
}

CRGB colorForCellColor(CellColor color) {
  switch (color) {
    case CellColor::Purple: return COLOR_PURPLE;
    case CellColor::Yellow: return COLOR_YELLOW;
    case CellColor::Blue:
    default:                return COLOR_BLUE;
  }
}

uint16_t servoPulseForValue(float value) {
  const float corrected = gamma01(value);
  const float pulse = kServoMinRunUs + corrected * (kServoMaxRunUs - kServoMinRunUs);
  return (uint16_t)roundf(pulse);
}

uint32_t scaledPeriod(uint32_t periodMs) {
  return (uint32_t)max(1.0f, periodMs / kEffectSpeed);
}

uint32_t scaledStep(uint32_t stepMs) {
  return (uint32_t)max(1.0f, stepMs / kEffectSpeed);
}

uint8_t rowsForSurface(Surface surface) {
  return surface == Surface::Front ? kFrontRows : kSideRows;
}

uint8_t colsForSurface(Surface surface) {
  return surface == Surface::Front ? kFrontCols : kSideCols;
}

float phaseOffsetForSurface(Surface surface) {
  return surface == Surface::Front ? 0.0f : kSidePhaseOffset;
}

bool isPurpleBox(Surface surface, uint8_t box) {
  if (surface == Surface::Front) {
    switch (box) {
      case 6: case 7: case 9: case 13: case 15: case 19: case 23:
      case 26: case 28: case 29: case 31: case 33: case 36: case 37:
        return true;
      default:
        return false;
    }
  }
  switch (box) {
    case 4: case 5: case 8: case 9:
      return true;
    default:
      return false;
  }
}

CellColor colorForBox(Surface surface, int8_t box) {
  if (box < 0) return CellColor::Blue;
  return isPurpleBox(surface, (uint8_t)box) ? CellColor::Purple : CellColor::Yellow;
}

char ledLabelFor(Surface surface, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan) {
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

bool blockCovers(const MergedBlock& block, uint8_t row, uint8_t col) {
  return row >= block.row &&
         row < block.row + block.rowSpan &&
         col >= block.col &&
         col < block.col + block.colSpan;
}

const MergedBlock* mergedBlockAt(const MergedBlock* blocks, uint8_t blockCount, uint8_t row, uint8_t col) {
  for (uint8_t i = 0; i < blockCount; i++) {
    if (blocks[i].row == row && blocks[i].col == col) return &blocks[i];
  }
  return nullptr;
}

bool isCoveredByMergedBlock(const MergedBlock* blocks, uint8_t blockCount, uint8_t row, uint8_t col) {
  for (uint8_t i = 0; i < blockCount; i++) {
    if (blockCovers(blocks[i], row, col) && (blocks[i].row != row || blocks[i].col != col)) return true;
  }
  return false;
}

int8_t gridBoxAt(Surface surface, uint8_t row, uint8_t col) {
  return surface == Surface::Front ? kFrontGrid[row][col] : kSideGrid[row][col];
}

bool servoRouteFor(Surface surface, uint8_t box, uint8_t& driverIndex, uint8_t& channel) {
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

void addLedUnit(Surface surface, int8_t box, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan) {
  if (ledUnitCount >= kMaxLedUnits) return;
  ledUnits[ledUnitCount++] = {
    surface, box, row, col, rowSpan, colSpan,
    colorForBox(surface, box),
    ledLabelFor(surface, row, col, rowSpan, colSpan),
    (rowSpan > 1 || colSpan > 1) ? LedKind::Strip : LedKind::Panel
  };
}

void addServoUnit(Surface surface, uint8_t box, uint8_t row, uint8_t col) {
  if (servoUnitCount >= kMaxServoUnits) return;
  uint8_t driverIndex = 0;
  uint8_t channel = 0;
  if (!servoRouteFor(surface, box, driverIndex, channel)) return;
  servoUnits[servoUnitCount++] = { surface, box, row, col, driverIndex, channel };
}

void buildSurfaceLayout(Surface surface, const MergedBlock* blocks, uint8_t blockCount) {
  const uint8_t rows = rowsForSurface(surface);
  const uint8_t cols = colsForSurface(surface);

  for (uint8_t row = 0; row < rows; row++) {
    for (uint8_t col = 0; col < cols; col++) {
      const int8_t box = gridBoxAt(surface, row, col);
      if (box > 0) addServoUnit(surface, (uint8_t)box, row, col);
      if (isCoveredByMergedBlock(blocks, blockCount, row, col)) continue;

      const MergedBlock* block = mergedBlockAt(blocks, blockCount, row, col);
      if (block != nullptr) {
        addLedUnit(surface, box, row, col, block->rowSpan, block->colSpan);
      } else {
        addLedUnit(surface, box, row, col, 1, 1);
      }
    }
  }
}

void buildLayout() {
  ledUnitCount = 0;
  servoUnitCount = 0;
  buildSurfaceLayout(Surface::Front, kFrontMergedBlocks, sizeof(kFrontMergedBlocks) / sizeof(kFrontMergedBlocks[0]));
  buildSurfaceLayout(Surface::Side, kSideMergedBlocks, sizeof(kSideMergedBlocks) / sizeof(kSideMergedBlocks[0]));
}

// ── Ripple pattern math ──
RippleState& rippleForSurface(Surface surface) {
  return surface == Surface::Front ? frontRipple : sideRipple;
}

void chooseRippleCenter(Surface surface, RippleState& ripple) {
  ripple.centerRow = (uint8_t)random(rowsForSurface(surface));
  ripple.centerCol = (uint8_t)random(colsForSurface(surface));
}

float maxDistanceFromCenter(Surface surface, const RippleState& ripple) {
  float maxDistance = 0.0f;
  const uint8_t rows = rowsForSurface(surface);
  const uint8_t cols = colsForSurface(surface);
  for (uint8_t row = 0; row < rows; row++) {
    for (uint8_t col = 0; col < cols; col++) {
      const float distance = hypotf((float)col - ripple.centerCol, (float)row - ripple.centerRow);
      if (distance > maxDistance) maxDistance = distance;
    }
  }
  return maxDistance;
}

float rippleValue(Surface surface, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan, uint32_t now) {
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

// ── Wander pattern math ──
WanderState& wanderForSurface(Surface surface) {
  return surface == Surface::Front ? frontWander : sideWander;
}

void clearVisited(Walker& walker) {
  for (uint8_t row = 0; row < kMaxRows; row++) {
    for (uint8_t col = 0; col < kMaxCols; col++) {
      walker.visitedStep[row][col] = -20000;
    }
  }
}

void initWalker(Surface surface, Walker& walker, int32_t step) {
  clearVisited(walker);
  walker.row = (uint8_t)random(rowsForSurface(surface));
  walker.col = (uint8_t)random(colsForSurface(surface));
  walker.previousRow = walker.row;
  walker.previousCol = walker.col;
  walker.visitedStep[walker.row][walker.col] = (int16_t)step;
}

void initWalkers(Surface surface, int32_t step) {
  WanderState& state = wanderForSurface(surface);
  for (uint8_t i = 0; i < kWanderDotCount; i++) {
    initWalker(surface, state.walkers[i], step);
  }
  state.lastStep = step;
  state.initialized = true;
}

void advanceWalker(Surface surface, Walker& walker, int32_t step) {
  struct Position { uint8_t row; uint8_t col; };
  Position options[4];
  uint8_t optionCount = 0;
  const uint8_t rows = rowsForSurface(surface);
  const uint8_t cols = colsForSurface(surface);

  if (walker.row > 0) options[optionCount++] = { (uint8_t)(walker.row - 1), walker.col };
  if (walker.row + 1 < rows) options[optionCount++] = { (uint8_t)(walker.row + 1), walker.col };
  if (walker.col > 0) options[optionCount++] = { walker.row, (uint8_t)(walker.col - 1) };
  if (walker.col + 1 < cols) options[optionCount++] = { walker.row, (uint8_t)(walker.col + 1) };

  if (optionCount == 0) return;
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

void updateWanderSurface(Surface surface, int32_t currentStep) {
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

void updateWanderStates(uint32_t now) {
  const int32_t currentStep = now / scaledStep(kWanderStepMs);
  updateWanderSurface(Surface::Front, currentStep);
  updateWanderSurface(Surface::Side, currentStep);
}

bool positionCoveredByUnit(const LedUnit& unit, uint8_t row, uint8_t col) {
  return row >= unit.row &&
         row < unit.row + unit.rowSpan &&
         col >= unit.col &&
         col < unit.col + unit.colSpan;
}

float wanderTrailValue(const LedUnit& unit, uint32_t now) {
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
          if (candidate > value) value = candidate;
        }
      }
    }
  }
  return value;
}

float wanderTrailValueForServo(const ServoUnit& servo, uint32_t now) {
  LedUnit unit = {
    servo.surface, (int8_t)servo.box, servo.row, servo.col, 1, 1,
    colorForBox(servo.surface, (int8_t)servo.box), '\0', LedKind::Panel
  };
  return wanderTrailValue(unit, now);
}

float wanderBlueBreathValue(const LedUnit& unit, uint32_t now) {
  const uint32_t periodMs = scaledPeriod(kWanderBlueBreathPeriodMs);
  const float cycleMs = now % periodMs;
  const float centerCol = unit.col + (unit.colSpan - 1) * 0.5f;
  const float phase = (cycleMs / (float)periodMs) * kTwoPi - centerCol * 0.42f + phaseOffsetForSurface(unit.surface);
  return wave01(phase);
}

// ── Breath pattern math ──
float breathLedValue(Surface surface, uint8_t row, uint8_t col, uint8_t rowSpan, uint8_t colSpan, uint32_t now) {
  (void)row; (void)rowSpan;
  const uint32_t periodMs = scaledPeriod(kBreathPeriodMs);
  const float basePhase = ((now % periodMs) / (float)periodMs) * kTwoPi + phaseOffsetForSurface(surface);
  const float centerCol = col + (colSpan - 1) * 0.5f;
  return wave01(basePhase + centerCol * kXPhase);
}

float breathServoValue(const ServoUnit& servo, uint32_t now) {
  const uint32_t periodMs = scaledPeriod(kBreathPeriodMs);
  const float basePhase = ((now % periodMs) / (float)periodMs) * kTwoPi + phaseOffsetForSurface(servo.surface);
  return wave01(basePhase + servo.col * kXPhase + servo.row * kYPhase);
}

uint8_t panelSegmentIndexFor(const LedUnit& unit) {
  uint8_t index = 0;
  for (uint8_t i = 0; i < ledUnitCount; i++) {
    const LedUnit& other = ledUnits[i];
    if (other.surface != unit.surface ||
        other.kind != LedKind::Panel ||
        other.color == CellColor::Blue ||
        other.col != unit.col) {
      continue;
    }
    if (other.row > unit.row) index++;
  }
  return index;
}

// ---------------- 🟢 预设渲染桥接核心：自动分配物理校准色 ----------------

uint32_t getNeoColorForUnit(const LedUnit& unit, uint8_t brightness) {
  int boxId = -1;
  bool isFront = (unit.surface == Surface::Front);

  if (unit.kind == LedKind::Strip) {
    if (unit.label == 'A') boxId = 101;
    else if (unit.label == 'B') boxId = 102;
    else if (unit.label == 'C') boxId = 103;
    else if (unit.label == 'D') boxId = 104;
    else if (unit.label == 'E') boxId = 105;
    else if (unit.label == 'F') boxId = 106;
    else if (unit.label == 'G') boxId = 301;
  } 
  else if (unit.color == CellColor::Blue) {
    if (unit.label == 'a' || unit.label == 'b' || unit.label == 'c') boxId = 201;
    else if (unit.label == 'd') boxId = 204;
    else if (unit.label == 'j' || unit.label == 'h') boxId = 404;
    else if (unit.label == 'i') boxId = 405;
    else if (unit.label == 'f') boxId = 402;
    else if (unit.label == 'g' || unit.label == 'e') boxId = 401;
  } 
  else {
    uint8_t segment = panelSegmentIndexFor(unit);
    if (isFront) {
      if (unit.col < 11 && segment < 5) {
        boxId = FRONT_STRIP_BOX_MAP[unit.col][segment];
      }
    } else {
      if (unit.col < 4 && segment < 4) {
        boxId = SIDE_STRIP_BOX_MAP[unit.col][segment];
      }
    }
  }

  if (boxId != -1) {
    // 强制路由至物理校准核心，使 G 闪、f 闪在预设模式下同样具备正确色温 [2]
    return getCalibratedColor(boxId, isFront, brightness);
  }
  return 0; 
}

void writeLedUnitToNeoPixelDirect(const LedUnit& unit, uint32_t neoColor) {
  if (unit.kind == LedKind::Strip) {
    switch (unit.label) {
      case 'A': stripBigA.fill(neoColor, 0, stripBigA.numPixels()); break;
      case 'B': stripBigB.fill(neoColor, 0, stripBigB.numPixels()); break;
      case 'C': stripBigC.fill(neoColor, 0, stripBigC.numPixels()); break;
      case 'D': stripBigD.fill(neoColor, 0, stripBigD.numPixels()); break;
      case 'E': stripBigE.fill(neoColor, 0, stripBigE.numPixels()); break;
      case 'F': stripBigF.fill(neoColor, 0, stripBigF.numPixels()); break;
      case 'G': stripBigG.fill(neoColor, 0, stripBigG.numPixels()); break;
    }
    return;
  }

  if (unit.color == CellColor::Blue) {
    switch (unit.label) {
      case 'c': stripSmall_ac.fill(neoColor, 0, 60); break;
      case 'b': stripSmall_ac.fill(neoColor, 60, 60); break;
      case 'a': stripSmall_ac.fill(neoColor, 120, 60); break;
      case 'd': stripSmall_d.fill(neoColor, 0, stripSmall_d.numPixels()); break;
      case 'j': stripSmall_hj.fill(neoColor, 0, 60); break;
      case 'h': stripSmall_hj.fill(neoColor, 60, 60); break;
      case 'i': stripSmall_i.fill(neoColor, 0, stripSmall_i.numPixels()); break;
      case 'f': stripSmall_f.fill(neoColor, 0, stripSmall_f.numPixels()); break;
      case 'g': stripSmall_eg.fill(neoColor, 0, 60); break;
      case 'e': stripSmall_eg.fill(neoColor, 60, 60); break;
    }
    return;
  }

  const uint8_t segment = panelSegmentIndexFor(unit);
  if (unit.surface == Surface::Front) {
    if (unit.col < FRONT_LED_COUNT) {
      for (int i = 0; i < PANEL_LEDS; i++) {
        stripFront[unit.col].setPixelColor(segment * PANEL_LEDS + i, neoColor);
      }
    }
  } else {
    if (unit.col < SIDE_LED_COUNT) {
      for (int i = 0; i < PANEL_LEDS; i++) {
        stripSide[unit.col].setPixelColor(segment * PANEL_LEDS + i, neoColor);
      }
    }
  }
}

void writeServo(const ServoUnit& servo, uint16_t pulseUs) {
  kServoDrivers[servo.driverIndex]->writeMicroseconds(servo.channel, pulseUs);
}

void allServosStop() {
  for (uint8_t i = 0; i < servoUnitCount; i++) {
    writeServo(servoUnits[i], kServoStopUs);
  }
}

// ── Physical NC Button polling ──
void checkButton() {
  static uint32_t lastDebounceTime = 0;
  static bool lastButtonState = false; 

  bool rawState = (digitalRead(BUTTON_PIN) == HIGH); 
  
  if (rawState != lastButtonState) {
    if (millis() - lastDebounceTime > 50) { 
      if (rawState) { 
        currentMode = (SystemMode)((int)currentMode + 1);
        if (currentMode >= NUM_MODES) {
          currentMode = MODE_RADAR;
        }

        DEBUG_SERIAL.print("\n>>> System mode switched to: ");
        DEBUG_SERIAL.println((int)currentMode);

        stopAllServos();
        allServosStop();

        for (int c = 0; c < FRONT_LED_COUNT; c++) { stripFront[c].clear(); stripFront[c].show(); }
        for (int c = 0; c < SIDE_LED_COUNT; c++) { stripSide[c].clear(); stripSide[c].show(); }
        fillStrip(stripBigA, 0); fillStrip(stripBigB, 0); fillStrip(stripBigC, 0);
        fillStrip(stripBigD, 0); fillStrip(stripBigE, 0); fillStrip(stripBigF, 0);
        fillStrip(stripSmall_ac, 0); fillStrip(stripSmall_d, 0); fillStrip(stripBigG, 0);
        fillStrip(stripSmall_hj, 0); fillStrip(stripSmall_i, 0); fillStrip(stripSmall_f, 0);
        fillStrip(stripSmall_eg, 0);

        if (currentMode == MODE_RADAR) {
          clearZones();
          summarizeAllZones();
        } else if (currentMode == MODE_PRESET_WANDER) {
          initWalkers(Surface::Front, millis() / scaledStep(kWanderStepMs));
          initWalkers(Surface::Side, millis() / scaledStep(kWanderStepMs));
        }
      }
      lastButtonState = rawState;
      lastDebounceTime = millis();
    }
  }
}

// ---------------- Arduino setup and loop ----------------

void setup() {
  debugBegin();
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println("PlanA ZET Optimized Integrated booting");

  pinMode(BUTTON_PIN, INPUT_PULLUP); // 物理 PC5 NC 上拉 [1]

  // 🟢 极速信号边缘优化，彻底解决高频线路上首尾数据畸变错位问题 [1]
  // 🟢 极速信号边缘优化，彻底解决高频线路上首尾数据畸变错位、闪烁问题 [1]
  #ifdef ARDUINO_ARCH_STM32
  // 使用标准 STM32 HAL 库直接配置引脚速率为 VERY_HIGH，绕开平台底层宏参数不一致的编译问题
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH; // 👈 强制设定为极速 slew-rate 模式

  GPIO_InitStruct.Pin = GPIO_PIN_7;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); // PD7 (f)

  GPIO_InitStruct.Pin = GPIO_PIN_1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); // PB1 (d)

  GPIO_InitStruct.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct); // PC2 (B)

  GPIO_InitStruct.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct); // PD4 (G)
  #endif

  for (int c = 0; c < FRONT_ZONES; c++) colDistance1[c] = DIST_A_MAX;
  for (int c = 0; c < SIDE_ZONES; c++) colDistance2[c] = DIST_A_MAX;

  buildLayout(); 
  initLeds();
  bootLedSelfTest();

  DEBUG_SERIAL.println("I2C begin PB7/PB6");
  myI2C.begin();
  myI2C.setClock(400000); 
  scanI2C();

  setupPwmDriver(pwm1, "0x40 front 1-16");
  setupPwmDriver(pwm2, "0x60 front 17-32");
  setupPwmDriver(pwm3, "0x50 front 33-38");
  setupPwmDriver(pwm4, "0x68 side 1-14");
  stopAllServos();
  allServosStop();
  DEBUG_SERIAL.println("Servos stopped");

  DEBUG_SERIAL.println("Radar serial begin");
  radar1Serial.setRx(RADAR1_RX_PIN);
  radar1Serial.setTx(RADAR1_TX_PIN);
  radar1Serial.begin(RADAR1_BAUD);
  radar2Serial.setRx(RADAR2_RX_PIN);
  radar2Serial.setTx(RADAR2_TX_PIN);
  radar2Serial.begin(RADAR2_BAUD);

  for (int i = 0; i < 3; i++) {
    r1.kf[i].kfX = {0, 1.0, false};
    r1.kf[i].kfY = {0, 1.0, false};
    r2.kf[i].kfX = {0, 1.0, false};
    r2.kf[i].kfY = {0, 1.0, false};
  }

  enableRadarEngineeringMode(radar1Serial, "radar1");
  enableRadarEngineeringMode(radar2Serial, "radar2");

  while (radar1Serial.available()) radar1Serial.read();
  while (radar2Serial.available()) radar2Serial.read();

  DEBUG_SERIAL.print("\033[2J\033[H"); 
  DEBUG_SERIAL.println("PlanA ZET (System Ready)");
}

void loop() {
  checkButton();

  if (currentMode == MODE_RADAR) {
    // ---------------- MODE 0: 雷达自动识别交互模式 (Plan A) ----------------
    readRadarSerial(radar1Serial, r1);
    parseFromRingBuffer(true);

    readRadarSerial(radar2Serial, r2);
    parseFromRingBuffer(false);

    const uint32_t now = millis();
    bool expired = false;
    if (now - r1.lastValidFrameTime > 1500) {
      for (int i = 0; i < 3; i++) r1.targets[i].valid = false;
      expired = true;
    }
    if (now - r2.lastValidFrameTime > 1500) {
      for (int i = 0; i < 3; i++) r2.targets[i].valid = false;
      expired = true;
    }
    if (expired) processFrameAll();

    if (now - lastStatusTime >= 300) {
      printStatus();
      lastStatusTime = now;
    }

    if (r1.validFrameCount == 0 && r2.validFrameCount == 0 && now - lastNoFrameNoticeMs >= 3000) {
      DEBUG_SERIAL.println("No valid radar frame yet. Check radar power, TX/RX crossing, baud 256000.");
      lastNoFrameNoticeMs = now;
    }

  } else {
    // ---------------- MODES 1-3: 艺术美学预设动画模式 (Plan B) ----------------
    const uint32_t now = millis();
    static uint32_t lastPresetFrameMs = 0;

    if (now - lastPresetFrameMs >= kFrameMs) {
      lastPresetFrameMs = now;

      if (currentMode == MODE_PRESET_WANDER) {
        updateWanderStates(now);
      }

      // a. 计算并更新色彩
      for (uint8_t i = 0; i < ledUnitCount; i++) {
        float val = 0.0f;
        if (currentMode == MODE_PRESET_BREATH) {
          val = breathLedValue(ledUnits[i].surface, ledUnits[i].row, ledUnits[i].col, ledUnits[i].rowSpan, ledUnits[i].colSpan, now);
        } else if (currentMode == MODE_PRESET_RIPPLE) {
          val = rippleValue(ledUnits[i].surface, ledUnits[i].row, ledUnits[i].col, ledUnits[i].rowSpan, ledUnits[i].colSpan, now);
        } else if (currentMode == MODE_PRESET_WANDER) {
          if (ledUnits[i].color == CellColor::Blue) {
            val = wanderBlueBreathValue(ledUnits[i], now);
          } else {
            val = wanderTrailValue(ledUnits[i], now);
          }
        }
        uint8_t brightness = brightnessForValue(val);
        uint32_t neoColor = getNeoColorForUnit(ledUnits[i], brightness);
        writeLedUnitToNeoPixelDirect(ledUnits[i], neoColor);
      }

      // 帧尾统一输出，杜绝中间闪烁
      for (int c = 0; c < FRONT_LED_COUNT; c++) stripFront[c].show();
      for (int c = 0; c < SIDE_LED_COUNT; c++) stripSide[c].show();
      stripBigA.show(); stripBigB.show(); stripBigC.show(); stripBigD.show(); stripBigE.show(); stripBigF.show();
      stripSmall_ac.show(); stripSmall_d.show();
      stripBigG.show(); stripSmall_hj.show(); stripSmall_i.show(); stripSmall_f.show(); stripSmall_eg.show();

      // b. 机械舵机摆动控制
      for (uint8_t i = 0; i < servoUnitCount; i++) {
        float val = 0.0f;
        if (currentMode == MODE_PRESET_BREATH) {
          val = breathServoValue(servoUnits[i], now);
        } else if (currentMode == MODE_PRESET_RIPPLE) {
          val = rippleValue(servoUnits[i].surface, servoUnits[i].row, servoUnits[i].col, 1, 1, now);
        } else if (currentMode == MODE_PRESET_WANDER) {
          val = wanderTrailValueForServo(servoUnits[i], now);
        }
        writeServo(servoUnits[i], servoPulseForValue(val));
      }
    }

    if (now - lastStatusTime >= 300) {
      DEBUG_SERIAL.print("\033[H"); 
      DEBUG_SERIAL.println("════════════════════════════════════════════════════════");
      DEBUG_SERIAL.print("  双面双雷达机械彩灯交互系统已上线 - [预设模式: ");
      if (currentMode == MODE_PRESET_BREATH) DEBUG_SERIAL.print("呼吸脉动");
      else if (currentMode == MODE_PRESET_RIPPLE) DEBUG_SERIAL.print("波纹涟漪");
      else if (currentMode == MODE_PRESET_WANDER) DEBUG_SERIAL.print("随机游走");
      DEBUG_SERIAL.println("] ");
      DEBUG_SERIAL.println("════════════════════════════════════════════════════════");
      DEBUG_SERIAL.println("  💡 按压 PC5 开关可切换雷达感应模式或循环切换预设。");
      DEBUG_SERIAL.println("════════════════════════════════════════════════════════");
      DEBUG_SERIAL.print("\033[J"); 
      lastStatusTime = now;
    }
  }

  delay(1); 
}
