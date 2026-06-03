#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>                // 解决 Adafruit 编译依赖
#include <FastLED.h>            // 用于极为方便的 CRGB 颜色管理
#include <Adafruit_NeoPixel.h>  // 用于 STM32 硬件灯光物理输出
#include <Adafruit_PWMServoDriver.h> // 用于级联舵机控制
#include <math.h>

// ╔══════════════════════════════════════════════════════════════╗
// ║  双面双雷达机械彩灯交互大一统系统 v4.5 (STM32F407 终极无错版)   ║
// ║  特色：双雷达独立并发 / 15路灯手拉手并独立设色 / 52路舵机级联║
// ║        310绝对零位 / 舵机0.5倍速限制 / 全向前声明防报错       ║
// ╚══════════════════════════════════════════════════════════════╝

// ################################################################
// §1  硬件引脚 & 设备配置（三组串口与引脚全部在此写死，绝无冲突）
// ################################################################

// §1-A  【第一组：电脑 TTL 调试监听口】(固定占用主板硬件串口 1 - USART1) [1]
#define MONITOR_TX_PIN   PA9    // 主板 A9 接口 -> 接调试小板的 RX
#define MONITOR_RX_PIN   PA10   // 主板 A10 接口 -> 接调试小板的 TX
#define MONITOR_SERIAL   Serial1

// §1-B  【第二组：正面雷达 (Radar 1) 通信口】(固定占用主板硬件串口 3 - USART3)
#define RADAR1_TX_PIN    PB10   // 主板 B10 接口 -> 接正面雷达的 RX
#define RADAR1_RX_PIN    PB11   // 主板 B11 接口 -> 接正面雷达的 TX
#define RADAR1_BAUD      256000
HardwareSerial RADAR1_SERIAL(USART3);

// §1-C  【第三组：侧面雷达 (Radar 2) 通信口】(固定占用主板硬件串口 2 - USART2) [1]
#define RADAR2_TX_PIN    PA2    // 主板 A2 接口 -> 接侧面雷达的 RX
#define RADAR2_RX_PIN    PA3    // 主板 A3 接口 -> 接侧面雷达的 TX [1]
#define RADAR2_BAUD      256000
HardwareSerial RADAR2_SERIAL(USART2);

// §1-D  【正面 11 路灯带引脚配置】(对应主板左侧排针 PE0 ~ PE10)
#define FRONT_LED_COUNT 11
const int FRONT_LED_PINS[FRONT_LED_COUNT] = {PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9, PE10};

// §1-E  【侧面 4 路灯带引脚配置】(对应主板左侧排针 PD0 ~ PD3) [2]
#define SIDE_LED_COUNT  4
const int SIDE_LED_PINS[SIDE_LED_COUNT] = {PD0, PD1, PD2, PD3}; // [2]

#define PANEL_LEDS     256 // 每个灯板有 256 颗灯珠

// 实例化正面 11 路、侧面 4 路，总共 15 路硬件灯带控制器
Adafruit_NeoPixel stripFront[FRONT_LED_COUNT] = {
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE0, NEO_GRB + NEO_KHZ800),  // C1 (1,11,17,28) - 4板级联
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE1, NEO_GRB + NEO_KHZ800),  // C2 (2,12,18,29) - 4板
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE2, NEO_GRB + NEO_KHZ800),  // C3 (3,13,19,30) - 4板
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE3, NEO_GRB + NEO_KHZ800),  // C4 (4,14,20,31) - 4板
  Adafruit_NeoPixel(2 * PANEL_LEDS, PE4, NEO_GRB + NEO_KHZ800),  // C5 (5,32) - 2板
  Adafruit_NeoPixel(2 * PANEL_LEDS, PE5, NEO_GRB + NEO_KHZ800),  // C6 (6,33) - 2板
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE6, NEO_GRB + NEO_KHZ800),  // C7 (7,23,26,34) - 4板
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE7, NEO_GRB + NEO_KHZ800),  // C8 (8,24,27,35) - 4板
  Adafruit_NeoPixel(1 * PANEL_LEDS, PE8, NEO_GRB + NEO_KHZ800),  // C9 (36) - 1板
  Adafruit_NeoPixel(4 * PANEL_LEDS, PE9, NEO_GRB + NEO_KHZ800),  // C10 (9,15,21,37) - 4板
  Adafruit_NeoPixel(5 * PANEL_LEDS, PE10, NEO_GRB + NEO_KHZ800)  // C11 (10,16,22,25,38) - 5板
};

Adafruit_NeoPixel stripSide[SIDE_LED_COUNT] = {
  Adafruit_NeoPixel(3 * PANEL_LEDS, PD0, NEO_GRB + NEO_KHZ800),  // C1 (1,4,11) - 3板 [2]
  Adafruit_NeoPixel(4 * PANEL_LEDS, PD1, NEO_GRB + NEO_KHZ800),  // C2 (2,5,8,12) - 4板 [2]
  Adafruit_NeoPixel(4 * PANEL_LEDS, PD2, NEO_GRB + NEO_KHZ800),  // C3 (3,6,9,13) - 4板 [2]
  Adafruit_NeoPixel(3 * PANEL_LEDS, PD3, NEO_GRB + NEO_KHZ800)   // C4 (7,10,14) - 3板 [2]
};

// 创建 FastLED 颜色缓冲区
CRGB ledsFront[FRONT_LED_COUNT * 5 * PANEL_LEDS];
CRGB ledsSide[SIDE_LED_COUNT * 4 * PANEL_LEDS];

// 【舵机 I2C 总线配置】(SDA=B7, SCL=B6) [1]
#define SERVO_SDA      PB7
#define SERVO_SCL      PB6
TwoWire myI2C(SERVO_SDA, SERVO_SCL);

// 声明 4 块级联的舵机扩展板（地址：0x40, 0x60, 0x50, 0x68） [2]
Adafruit_PWMServoDriver pwm1 = Adafruit_PWMServoDriver(0x40, myI2C); // 正面板 1
Adafruit_PWMServoDriver pwm2 = Adafruit_PWMServoDriver(0x60, myI2C); // 正面板 2
Adafruit_PWMServoDriver pwm3 = Adafruit_PWMServoDriver(0x50, myI2C); // 正面板 3
Adafruit_PWMServoDriver pwm4 = Adafruit_PWMServoDriver(0x68, myI2C); // 侧面板 4

// 侧面 14 个物理箱子按顺序依次、不跳跃地插在第 4 块舵机板 (0x68) 的 2~15 号通道上
const int BOX_TO_SERVO_CHAN_SIDE[14] = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};

// 侧面 14 个物理箱子所属的控制列 (0=C1, 1=C2, 2=C3, 3=C4)
const int BOX_TO_COL_SIDE[14] = {0, 1, 2, 0, 1, 2, 3, 1, 2, 3, 0, 1, 2, 3};

// 状态自检 LED
#define BOARD_LED      PB4

// ################################################################
// §2  C++ 声明顺序重整区（必须最先声明所有数据结构，防止未定义报错）
// ################################################################
struct BoxDef {
  int    id;
  double angleMin;
  double angleMax;
};

struct KalmanFilter {
  double x_est;   
  double p_est;   
  bool   init;
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
  bool   valid;
  double fx, fy;
  double fDist;
  double fAngle;
};

// 距离分层与控制常量
#define FRAME_LEN    30
#define BUFFER_SIZE  128
#define DIST_E_MAX   0.25   
#define DIST_D_MAX   0.50   
#define DIST_C_MAX   0.90   
#define DIST_B_MAX   1.40   
#define DIST_A_MAX   2.00   
#define LED_BRIGHTNESS_MIN   2    
#define LED_BRIGHTNESS_MAX  20    
#define SERVO_STOP_PULSE    310   // 绝对静止脉宽 [2.4.6]

#define FRONT_ZONES  11
#define SIDE_ZONES   4
#define NUM_ZONES    5 // 占位保持兼容

// 补齐：将 noHeadCount 和 fakeHeadCount 写入结构体中
struct RadarStream {
  uint8_t  ringBuf[BUFFER_SIZE];
  int      ringHead;
  int      ringTail;
  int      ringCount;
  uint32_t byteCount;
  uint32_t validFrameCount;
  uint32_t fakeHeadCount;  
  uint32_t noHeadCount;    
  uint32_t lastValidFrameTime;
  Target   targets[3];
  KFPair   kf[3];
};

// 声明全局实例
RadarStream r1; // 正面雷达
RadarStream r2; // 侧面雷达

bool   colActive1[FRONT_ZONES] = {false};
double colDistance1[FRONT_ZONES] = {2.0};
bool   colActive2[SIDE_ZONES] = {false};
double colDistance2[SIDE_ZONES] = {2.0};

bool zoneActive1[FRONT_ZONES][5];  
bool zoneActive2[SIDE_ZONES][5];   

unsigned long lastOutputTime = 0;
unsigned long lastStatusTime = 0;

// ################################################################
// §3  FORWARD DECLARATIONS (全向前声明白名单，彻底根治未定义报错 ❗)
// ################################################################
double   calcCoordinate(int low, int high);
double   calcUint16(int low, int high);
double   kalmanUpdate(KalmanFilter &kf, double measurement);
int      distToLayer(double dist);
int      distToBrightness(double dist);
int8_t   distToServoSpeed(double dist);
void     set60ServoSpeed(Adafruit_PWMServoDriver &driver, uint8_t channel, int8_t speed);
void     setFrontServoSpeed(uint8_t boxId, int8_t speed);
uint32_t getBoxColor(int boxId, bool isFront, uint8_t brightness);
int      getFrontBoxCol(int boxId);
void     clearZones();
void     markFrontZone(double angle, double dist);
void     markSideZone(double angle, double dist);
void     summarizeAllZones();
void     showLeds();
void     driveOutputs();
void     printStatus();
void     parseFrame(uint8_t *frame, bool isRadar1);
void     processFrameAll();
void     parseFromRingBuffer(bool isRadar1);
void     enableRadarEngineeringMode(HardwareSerial &serial);

// ################################################################
// §4  物理箱体颜色与引脚映射查找表
// ################################################################
const CRGB COLOR_PURPLE = CRGB(150, 50, 200);
const CRGB COLOR_YELLOW = CRGB(230, 180, 40);

const int FRONT_STRIP_BOX_MAP[11][5] = {
  {1, 11, 17, 28, -1},  
  {2, 12, 18, 29, -1},  
  {3, 13, 19, 30, -1},  
  {4, 14, 20, 31, -1},  
  {5, 32, -1, -1, -1},  
  {6, 33, -1, -1, -1},  
  {7, 23, 26, 34, -1},  
  {8, 24, 27, 35, -1},  
  {36, -1, -1, -1, -1}, 
  {9, 15, 21, 37, -1},  
  {10, 16, 22, 25, 38}  
};

const int SIDE_STRIP_BOX_MAP[4][4] = {
  {1, 4, 11, -1},       
  {2, 5, 8, 12},        
  {3, 6, 9, 13},        
  {7, 10, 14, -1}       
};

const int FRONT_BOX_TO_COL[38] = {
  0, 1, 2, 3, 4, 5, 6, 7, 9, 10,  
  0, 1, 2, 3, 9, 10,              
  0, 1, 2, 3, 9, 10,              
  6, 7, 10,                       
  6, 7,                           
  0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10 
};

// ################################################################
// §5  底层纯净数学与映射辅助函数实现
// ################################################################
double calcCoordinate(int low, int high) {
  int raw = low + high * 256;
  if (raw & 0x8000) return (double)raw - 32768.0;
  return (double)(0 - raw);
}

double calcUint16(int low, int high) {
  return (double)(low + high * 256);
}

double kalmanUpdate(KalmanFilter &kf, double measurement) {
  if (!kf.init) {
    kf.x_est = measurement;
    kf.p_est = 1.0;
    kf.init  = true;
    return measurement;
  }
  double p_pred = kf.p_est + 0.05; 
  double K      = p_pred / (p_pred + 2.0); 
  kf.x_est      = kf.x_est + K * (measurement - kf.x_est);
  kf.p_est      = (1.0 - K) * p_pred;
  return kf.x_est;
}

int distToLayer(double dist) {
  if (dist > DIST_B_MAX)  return 0;  // 行A
  if (dist > DIST_C_MAX)  return 1;  // 行B
  if (dist > DIST_D_MAX)  return 2;  // 行C
  if (dist > DIST_E_MAX)  return 3;  // 行D
  return 4;                          // 行E
}

int distToBrightness(double dist) {
  double ratio = 1.0 - (dist / DIST_A_MAX);  
  ratio = constrain(ratio, 0.0, 1.0);
  return (int)(LED_BRIGHTNESS_MIN + ratio * (LED_BRIGHTNESS_MAX - LED_BRIGHTNESS_MIN));
}

int8_t distToServoSpeed(double dist) {
  double ratio = 1.0 - (dist / DIST_A_MAX); 
  ratio = constrain(ratio, 0.0, 1.0);
  return (int8_t)(20 + ratio * (80 - 20)); 
}

// 统一控制 360° 舵机转速并进行 0.5 倍物理减慢计算
void set60ServoSpeed(Adafruit_PWMServoDriver &driver, uint8_t channel, int8_t speed) {
  uint16_t pulse;
  if (speed == 0) {
    pulse = SERVO_STOP_PULSE; 
  } 
  else if (speed > 0) {
    pulse = map(speed, 0, 100, SERVO_STOP_PULSE, 385); // 顺时针限速
  } 
  else {
    pulse = map(speed, -100, 0, 230, SERVO_STOP_PULSE); // 逆时针限速
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

uint32_t getBoxColor(int boxId, bool isFront, uint8_t brightness) {
  double scale = (double)brightness / 20.0;
  uint8_t r = 0, g = 0, b = 0;
  bool isPurple = false;

  if (isFront) {
    if (boxId == 6 || boxId == 7 || boxId == 9 || boxId == 13 || boxId == 15 || 
        boxId == 19 || boxId == 23 || boxId == 26 || boxId == 28 || boxId == 29 || 
        boxId == 31 || boxId == 33 || boxId == 36 || boxId == 37) {
      isPurple = true;
    }
  } else {
    if (boxId == 5 || boxId == 8 || boxId == 9) {
      isPurple = true;
    }
  }

  if (isPurple) {
    r = (uint8_t)(COLOR_PURPLE.r * scale);
    g = (uint8_t)(COLOR_PURPLE.g * scale);
    b = (uint8_t)(COLOR_PURPLE.b * scale);
  } else {
    r = (uint8_t)(COLOR_YELLOW.r * scale);
    g = (uint8_t)(COLOR_YELLOW.g * scale);
    b = (uint8_t)(COLOR_YELLOW.b * scale);
  }
  return Adafruit_NeoPixel::Color(r, g, b);
}

int getFrontBoxCol(int boxId) {
  if (boxId < 1 || boxId > 38) return 0;
  return FRONT_BOX_TO_COL[boxId - 1];
}

// ################################################################
// §6  交互业务逻辑与网格计算函数实现
// ################################################################
void clearZones() {
  for (int c = 0; c < FRONT_ZONES; c++) for (int l = 0; l < 5; l++) zoneActive1[c][l] = false;
  for (int c = 0; c < SIDE_ZONES; c++)  for (int l = 0; l < 5; l++) zoneActive2[c][l] = false;
}

void markFrontZone(double angle, double dist) {
  if (dist <= 0 || dist > DIST_A_MAX) return;
  int layer = distToLayer(dist);
  if (angle >= -40.0 && angle <= -28.0) zoneActive1[0][layer] = true;
  if (angle >= -32.0 && angle <= -20.0) zoneActive1[1][layer] = true;
  if (angle >= -24.0 && angle <= -12.0) zoneActive1[2][layer] = true;
  if (angle >= -16.0 && angle <= -4.0)  zoneActive1[3][layer] = true;
  if (angle >= -8.0  && angle <= 4.0)   zoneActive1[4][layer] = true;
  if (angle >= -4.0  && angle <= 12.0)  zoneActive1[5][layer] = true;
  if (angle >= 4.0   && angle <= 20.0)  zoneActive1[6][layer] = true;
  if (angle >= 12.0  && angle <= 28.0)  zoneActive1[7][layer] = true;
  if (angle >= 20.0  && angle <= 32.0)  zoneActive1[8][layer] = true;
  if (angle >= 28.0  && angle <= 36.0)  zoneActive1[9][layer] = true;
  if (angle >= 32.0  && angle <= 40.0)  zoneActive1[10][layer] = true;
}

void markSideZone(double angle, double dist) {
  if (dist <= 0 || dist > DIST_A_MAX) return;
  int layer = distToLayer(dist);
  if (angle >= -40.0 && angle <= -15.0) zoneActive2[0][layer] = true; 
  if (angle >= -22.0 && angle <= 5.0)   zoneActive2[1][layer] = true; 
  if (angle >= -5.0  && angle <= 22.0)  zoneActive2[2][layer] = true; 
  if (angle >= 15.0  && angle <= 40.0)  zoneActive2[3][layer] = true; 
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

// 占位空函数，保持硬件大驱动兼容
void showLeds() {
}

void driveOutputs() {
  // 1. 正面 11 路灯带刷新 (仅在变化时刷新)
  static uint8_t lastFrontBri[11] = {99,99,99,99,99,99,99,99,99,99,99};
  bool frontLedChanged = false;

  for (int c = 0; c < 11; c++) {
    uint8_t currentBri = colActive1[c] ? distToBrightness(colDistance1[c]) : 0;
    if (currentBri != lastFrontBri[c]) {
      lastFrontBri[c] = currentBri;
      frontLedChanged = true;
      
      for (int seg = 0; seg < 5; seg++) {
        int boxId = FRONT_STRIP_BOX_MAP[c][seg];
        if (boxId == -1) break;
        uint32_t col = getBoxColor(boxId, true, currentBri);
        for (int i = 0; i < PANEL_LEDS; i++) {
          stripFront[c].setPixelColor(seg * PANEL_LEDS + i, col);
        }
      }
    }
  }
  if (frontLedChanged) {
    for (int c = 0; c < 11; c++) stripFront[c].show();
  }

  // 2. 侧面 4 路灯带刷新 (仅在变化时刷新)
  static uint8_t lastSideBri[4] = {99,99,99,99};
  bool sideLedChanged = false;

  for (int c = 0; c < 4; c++) {
    uint8_t currentBri = colActive2[c] ? distToBrightness(colDistance2[c]) : 0;
    if (currentBri != lastSideBri[c]) {
      lastSideBri[c] = currentBri;
      sideLedChanged = true;
      
      for (int seg = 0; seg < 4; seg++) {
        int boxId = SIDE_STRIP_BOX_MAP[c][seg];
        if (boxId == -1) break;
        uint32_t col = getBoxColor(boxId, false, currentBri);
        for (int i = 0; i < PANEL_LEDS; i++) {
          stripSide[c].setPixelColor(seg * PANEL_LEDS + i, col);
        }
      }
    }
  }
  if (sideLedChanged) {
    for (int c = 0; c < 4; c++) stripSide[c].show();
  }

  // 3. 正面 38 路舵机下发 (仅在变化时下发)
  static int8_t lastFrontSpeed[38] = {99};
  for (int b = 0; b < 38; b++) {
    int col = FRONT_BOX_TO_COL[b];
    int8_t currentSpeed = colActive1[col] ? distToServoSpeed(colDistance1[col]) : 0;
    if (currentSpeed != lastFrontSpeed[b]) {
      lastFrontSpeed[b] = currentSpeed;
      setFrontServoSpeed(b + 1, currentSpeed); 
    }
  }

  // 4. 侧面 14 路舵机下发 (仅在变化时下发)
  static int8_t lastSideSpeed[14] = {99};
  for (int b = 0; b < 14; b++) {
    int col = BOX_TO_COL_SIDE[b];
    int chan = BOX_TO_SERVO_CHAN_SIDE[b];
    int8_t currentSpeed = colActive2[col] ? distToServoSpeed(colDistance2[col]) : 0;
    if (currentSpeed != lastSideSpeed[b]) {
      lastSideSpeed[b] = currentSpeed;
      set60ServoSpeed(pwm4, chan, currentSpeed); 
    }
  }
}

void printStatus() {
  MONITOR_SERIAL.print("\033[H"); 

  MONITOR_SERIAL.println("════════════════════════════════════════════════");
  MONITOR_SERIAL.println("  双面双雷达机械彩灯交互大系统已上线 (STM32F407)  ");
  MONITOR_SERIAL.println("════════════════════════════════════════════════");

  MONITOR_SERIAL.println("[正面 11 路列监视]");
  for (int c = 0; c < 11; c++) {
    MONITOR_SERIAL.print(" 列"); MONITOR_SERIAL.print(c + 1);
    if (colActive1[c]) {
      MONITOR_SERIAL.print("【动】距:"); MONITOR_SERIAL.print(colDistance1[c], 1); 
      MONITOR_SERIAL.print("m 亮:"); MONITOR_SERIAL.print(distToBrightness(colDistance1[c])); MONITOR_SERIAL.print(" ");
    } else {
      MONITOR_SERIAL.print("【静】距:--m 亮: 0 ");
    }
    if ((c + 1) % 4 == 0) MONITOR_SERIAL.println();
  }
  MONITOR_SERIAL.println("\n------------------------------------------------");

  MONITOR_SERIAL.println("[侧面 4 路列监视]");
  for (int c = 0; c < 4; c++) {
    MONITOR_SERIAL.print(" 列"); MONITOR_SERIAL.print(c + 1);
    if (colActive2[c]) {
      MONITOR_SERIAL.print("【动】距:"); MONITOR_SERIAL.print(colDistance2[c], 1); 
      MONITOR_SERIAL.print("m 亮:"); MONITOR_SERIAL.print(distToBrightness(colDistance2[c])); MONITOR_SERIAL.print(" |");
    } else {
      MONITOR_SERIAL.print("【静】距:--m 亮: 0 |");
    }
  }
  MONITOR_SERIAL.println("\n════════════════════════════════════════════════");
  MONITOR_SERIAL.print("\033[J"); 
}

void parseFrame(uint8_t *frame, bool isRadar1) {
  int off = 4;
  RadarStream &r = isRadar1 ? r1 : r2;

  for (int i = 0; i < 3; i++) {
    int xL=frame[off+0], xH=frame[off+1];
    int yL=frame[off+2], yH=frame[off+3];
    int sL=frame[off+4], sH=frame[off+5];
    int rL=frame[off+6], rH=frame[off+7];
    off += 8;

    bool allZero = !xL&&!xH&&!yL&&!yH&&!sL&&!sH&&!rL&&!rH;
    if (allZero) {
      r.targets[i].valid = false;
      r.kf[i].kfX.init = false;
      r.kf[i].kfY.init = false;
      continue;
    }

    r.targets[i].x          = calcCoordinate(xL, xH);
    r.targets[i].y          = calcCoordinate(yL, yH);
    r.targets[i].speed      = calcCoordinate(sL, sH);
    r.targets[i].resolution = calcUint16(rL, rH);
    r.targets[i].distance   = sqrt(r.targets[i].x*r.targets[i].x +
                                 r.targets[i].y*r.targets[i].y) / 1000.0;
    r.targets[i].angle      = atan2(r.targets[i].x, r.targets[i].y) * 180.0 / PI;
    r.targets[i].valid      = true;

    r.targets[i].fx     = kalmanUpdate(r.kf[i].kfX, r.targets[i].x);
    r.targets[i].fy     = kalmanUpdate(r.kf[i].kfY, r.targets[i].y);
    r.targets[i].fDist  = sqrt(r.targets[i].fx*r.targets[i].fx +
                             r.targets[i].fy*r.targets[i].fy) / 1000.0;
    r.targets[i].fAngle = atan2(r.targets[i].fx, r.targets[i].fy) * 180.0 / PI;
  }
}

void processFrameAll() {
  clearZones();
  for (int i = 0; i < 3; i++) {
    if (r1.targets[i].valid) markFrontZone(r1.targets[i].fAngle, r1.targets[i].fDist);
  }
  for (int i = 0; i < 3; i++) {
    if (r2.targets[i].valid) markSideZone(r2.targets[i].fAngle, r2.targets[i].fDist);
  }
  summarizeAllZones();
  driveOutputs();
}

void parseFromRingBuffer(bool isRadar1) {
  RadarStream &r = isRadar1 ? r1 : r2;

  while (r.ringCount >= FRAME_LEN) {
    int frameStartOffset = -1;
    for (int i = 0; i <= r.ringCount - FRAME_LEN; i++) {
      int p0 = (r.ringTail+i+0) % BUFFER_SIZE;
      int p1 = (r.ringTail+i+1) % BUFFER_SIZE;
      int p2 = (r.ringTail+i+2) % BUFFER_SIZE;
      int p3 = (r.ringTail+i+3) % BUFFER_SIZE;
      if (r.ringBuf[p0]==0xAA && r.ringBuf[p1]==0xFF &&
          r.ringBuf[p2]==0x03 && r.ringBuf[p3]==0x00) {
        frameStartOffset = i;
        break;
      }
    }
    if (frameStartOffset == -1) {
      int drop = r.ringCount - FRAME_LEN + 1;
      r.ringTail   = (r.ringTail + drop) % BUFFER_SIZE;
      r.ringCount -= drop;
      r.noHeadCount++;
      return;
    }
    if (frameStartOffset > 0) {
      r.ringTail   = (r.ringTail + frameStartOffset) % BUFFER_SIZE;
      r.ringCount -= frameStartOffset;
    }
    if (r.ringCount < FRAME_LEN) return;

    int tail1 = (r.ringTail + 28) % BUFFER_SIZE;
    int tail2 = (r.ringTail + 29) % BUFFER_SIZE;
    if (r.ringBuf[tail1]==0x55 && r.ringBuf[tail2]==0xCC) {
      uint8_t frame[FRAME_LEN];
      for (int i = 0; i < FRAME_LEN; i++)
        frame[i] = r.ringBuf[(r.ringTail+i) % BUFFER_SIZE];
      r.validFrameCount++;
      r.lastValidFrameTime = millis();

      parseFrame(frame, isRadar1);
      processFrameAll();

      if (millis() - lastOutputTime >= 300) {
        printStatus();
        lastOutputTime = millis();
      }
      r.ringTail   = (r.ringTail + FRAME_LEN) % BUFFER_SIZE;
      r.ringCount -= FRAME_LEN;
    } else {
      r.ringTail  = (r.ringTail + 1) % BUFFER_SIZE;
      r.ringCount--;
      r.fakeHeadCount++;
    }
  }
}

void enableRadarEngineeringMode(HardwareSerial &serial) {
  static const uint8_t enterConfig[] = {0xFD,0xFC,0xFB,0xFA,0x04,0x00,0xFF,0x00,0x01,0x00,0x04,0x03,0x02,0x01};
  static const uint8_t enableEng[]   = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0x62,0x00,0x04,0x03,0x02,0x01};
  static const uint8_t exitConfig[]  = {0xFD,0xFC,0xFB,0xFA,0x02,0x00,0xFE,0x00,0x04,0x03,0x02,0x01};
  serial.write(enterConfig, sizeof(enterConfig)); delay(150);
  serial.write(enableEng,   sizeof(enableEng));   delay(150);
  serial.write(exitConfig,  sizeof(exitConfig));  delay(150);
}

// ################################################################
// §SETUP
// ################################################################
void setup() {
  MONITOR_SERIAL.setRx(MONITOR_RX_PIN);
  MONITOR_SERIAL.setTx(MONITOR_TX_PIN);
  MONITOR_SERIAL.begin(115200);
  delay(1000);

  // 初始化正面 11 路灯带
  for (int c = 0; c < 11; c++) {
    stripFront[c].begin();
    stripFront[c].clear();
    stripFront[c].show();
  }
  // 初始化侧面 4 路灯带
  for (int c = 0; c < 4; c++) {
    stripSide[c].begin();
    stripSide[c].clear();
    stripSide[c].show();
  }

  // 极速模式 I2C 启动 (400kHz) [1]
  myI2C.begin();
  myI2C.setClock(400000);

  // 启动并初始化 4 块级联的舵机板，设置绝对停止零位为 310 [2.4.6]
  pwm1.begin(); pwm1.setPWMFreq(50);
  pwm2.begin(); pwm2.setPWMFreq(50);
  pwm3.begin(); pwm3.setPWMFreq(50);
  pwm4.begin(); pwm4.setPWMFreq(50);
  for (int b = 0; b < 38; b++) setFrontServoSpeed(b + 1, 0);
  for (int b = 0; b < 14; b++) set60ServoSpeed(pwm4, BOX_TO_SERVO_CHAN_SIDE[b], 0);

  // 初始化正面雷达串口 (USART3 -> B10, B11)
  RADAR1_SERIAL.setRx(RADAR1_RX_PIN);
  RADAR1_SERIAL.setTx(RADAR1_TX_PIN);
  RADAR1_SERIAL.begin(RADAR1_BAUD);

  // 初始化侧面雷达串口 (USART2 -> A2, A3) [1]
  RADAR2_SERIAL.setRx(RADAR2_RX_PIN);
  RADAR2_SERIAL.setTx(RADAR2_TX_PIN);
  RADAR2_SERIAL.begin(RADAR2_BAUD);

  // 初始化所有卡尔曼滤波器
  for (int i = 0; i < 3; i++) {
    r1.kf[i].kfX = {0, 1.0, false}; r1.kf[i].kfY = {0, 1.0, false};
    r2.kf[i].kfX = {0, 1.0, false}; r2.kf[i].kfY = {0, 1.0, false};
  }

  // 开启双雷达工程模式
  enableRadarEngineeringMode(RADAR1_SERIAL);
  enableRadarEngineeringMode(RADAR2_SERIAL);

  // 清空雷达接收缓冲区
  while (RADAR1_SERIAL.available()) RADAR1_SERIAL.read();
  while (RADAR2_SERIAL.available()) RADAR2_SERIAL.read();

  MONITOR_SERIAL.print("\033[2J\033[H"); // 强制全屏清空并重置光标
  MONITOR_SERIAL.println();
  MONITOR_SERIAL.println("════════════════════════════════════════════════");
  MONITOR_SERIAL.println("  双面双雷达交互大一统主控系统 v4.5 上线成功");
  MONITOR_SERIAL.println("  双路雷达独立解析、15路多段独立设色彩灯、310绝对静止舵机");
  MONITOR_SERIAL.println("════════════════════════════════════════════════");
}

// ################################################################
// §LOOP
// ################################################################
void loop() {
  // 1. 读取正面雷达数据 (Serial3)
  while (RADAR1_SERIAL.available()) {
    uint8_t b = RADAR1_SERIAL.read();
    r1.byteCount++;
    r1.ringBuf[r1.ringHead] = b;
    r1.ringHead = (r1.ringHead + 1) % BUFFER_SIZE;
    if (r1.ringCount < BUFFER_SIZE) r1.ringCount++;
    else r1.ringTail = (r1.ringTail + 1) % BUFFER_SIZE;
  }
  parseFromRingBuffer(true); // 解析正面雷达

  // 2. 读取侧面雷达数据 (Serial2) [1]
  while (RADAR2_SERIAL.available()) {
    uint8_t b = RADAR2_SERIAL.read();
    r2.byteCount++;
    r2.ringBuf[r2.ringHead] = b;
    r2.ringHead = (r2.ringHead + 1) % BUFFER_SIZE;
    if (r2.ringCount < BUFFER_SIZE) r2.ringCount++;
    else r2.ringTail = (r2.ringTail + 1) % BUFFER_SIZE;
  }
  parseFromRingBuffer(false); // 解析侧面雷达

  // 3. 超时保护
  if (millis() - r1.lastValidFrameTime > 1500) {
    for (int i = 0; i < 3; i++) r1.targets[i].valid = false;
    processFrameAll();
  }
  if (millis() - r2.lastValidFrameTime > 1500) {
    for (int i = 0; i < 3; i++) r2.targets[i].valid = false;
    processFrameAll();
  }

  delay(20); // 维持 20ms 的高速空转
}
