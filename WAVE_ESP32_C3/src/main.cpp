#include <Arduino.h>
#include <math.h>

// ╔══════════════════════════════════════════════════════════════╗
// ║  LD2450 图1正面箱体系统 v3                                   ║
// ║  新增：竖列联动 / 灯板颜色亮度 / 舵机转速 / 虚拟Zone网格     ║
// ║        卡尔曼滤波 / 对区不对人逻辑 / 分段注释                 ║
// ╚══════════════════════════════════════════════════════════════╝

// ================================================================
// ── §0  结构体定义区 (必须最先声明，供全局变量及函数原型使用) ────
// ================================================================
struct BoxDef {
  int    id;
  double angleMin;
  double angleMax;
};

struct KalmanFilter {
  double x_est;   // 估计值
  double p_est;   // 估计误差协方差
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
  // 滤波后坐标
  double fx, fy;
  double fDist;
  double fAngle;
};

// ================================================================
// ── §1  函数原型声明区 (PlatformIO 编译必需的前向声明) ──────────
// ================================================================
double calcCoordinate(int low, int high);
double calcUint16(int low, int high);
double kalmanUpdate(KalmanFilter &kf, double measurement);
int findBox(const BoxDef* defs, int count, double angle);
int angleToCol(double angle);
int distToLayer(double dist);
void getColBoxes(int col, int out[5]);
int distToBrightness(double dist);
int distToServoUs(double dist);
void applyLED(int col, int brightness, bool on);
void applyServo(int col, int servoUs, bool run);
void clearZones();
void markZone(double angle, double dist);
void summarizeZones();
void driveOutputs();
void printStatus();
void parseFrame(uint8_t *frame);
void processFrame();
void parseFromRingBuffer();

// ================================================================
// ── §2  常量与全局变量配置区 ──────────────────────────────────
// ================================================================

// ---------- 行B：boxes 11~16，玻璃区 0°~+24° 不触发 ----------
const BoxDef rowB[] = {
  {11, -40.0, -24.0},
  {12, -24.0, -16.0},
  {13, -16.0,  -8.0},
  {14,  -8.0,   0.0},
  {15,  24.0,  32.0},
  {16,  32.0,  40.0},
};
const int rowB_count = 6;

// ---------- 行C：boxes 17~22，布局同行B ----------
const BoxDef rowC[] = {
  {17, -40.0, -24.0},
  {18, -24.0, -16.0},
  {19, -16.0,  -8.0},
  {20,  -8.0,   0.0},
  {21,  24.0,  32.0},
  {22,  32.0,  40.0},
};
const int rowC_count = 6;

// ---------- 行D：boxes 23~27，仅右侧 0°~+40° ----------
const BoxDef rowD_top[] = {
  {23,   0.0,  16.0},
  {24,  16.0,  32.0},
  {25,  32.0,  40.0},
};
const BoxDef rowD_bot[] = {
  {26,   0.0,  16.0},
  {27,  16.0,  32.0},
};
const int rowD_top_count = 3;
const int rowD_bot_count = 2;

// 硬件引脚 & 串口配置
#define RADAR_RX_PIN   4       // 雷达 TX → ESP32 RX
#define RADAR_TX_PIN   5       // 雷达 RX → ESP32 TX
#define RADAR_BAUD     256000

// 舵机 PWM 引脚（每列一个，共10列）
const int SERVO_PIN[10] = {12, 13, 14, 15, 16, 17, 18, 19, 21, 22};

// 灯板 PWM 引脚（每列一个，共10列）
const int LED_PIN[10]   = {25, 26, 27, 32, 33, 34, 35, 36, 39, 23};

// 帧解析常量
#define FRAME_LEN    30
#define BUFFER_SIZE  128
#define PRINT_EACH_BYTE false

// 距离分层阈值（米）
#define DIST_E_MAX   0.25   // 行E  0     ~ 0.25m  boxes 28-38
#define DIST_D_MAX   0.50   // 行D  0.25  ~ 0.50m  boxes 23-27
#define DIST_C_MAX   0.90   // 行C  0.50  ~ 0.90m  boxes 17-22
#define DIST_B_MAX   1.40   // 行B  0.90  ~ 1.40m  boxes 11-16
#define DIST_A_MAX   2.00   // 行A  1.40  ~ 2.00m  boxes  1-10

// 灯板参数
#define LED_BRIGHTNESS_MIN   8    // 默认/最远时亮度（PWM 0-255 映射后）
#define LED_BRIGHTNESS_MAX  20    // 最近时最大亮度
#define LED_COLOR_R  200    // 白蓝光 R 分量
#define LED_COLOR_G  220    // 白蓝光 G 分量
#define LED_COLOR_B  255    // 白蓝光 B 分量（蓝色偏强）

// 舵机参数
#define SERVO_STOP_US    1500   // 停止脉宽(μs)
#define SERVO_MIN_US     1520   // 最慢正转（距离最远时）
#define SERVO_MAX_US     1600   // 最快正转（距离最近时）

// 卡尔曼滤波参数
#define KF_Q   0.05    // 过程噪声
#define KF_R   2.0     // 测量噪声

// 虚拟 Zone 网格定义
#define NUM_ZONES   10          // 角度方向分10列，与行A对齐
#define ZONE_ANGLE_MIN  -40.0
#define ZONE_ANGLE_MAX   40.0
bool zoneActive[NUM_ZONES][5];  // 列0~9，层0=行A … 层4=行E

// 串口实例
KFPair kf[3];  // 对应3个雷达目标槽
Target targets[3];

// 环形缓冲区
uint8_t ringBuf[BUFFER_SIZE];
int ringHead  = 0;
int ringTail  = 0;
int ringCount = 0;

// 系统统计 & 时序
unsigned long lastOutputTime  = 0;
unsigned long lastStatusTime  = 0;
uint32_t byteCount       = 0;
uint32_t validFrameCount = 0;
uint32_t fakeHeadCount   = 0;
uint32_t noHeadCount     = 0;

// 当前帧输出结果
int activeCol    = -1;
double activeDistance = 2.0;   // 当前最近目标距离

// 竖列映射表
const int colToRowB[10] = {11, 12, 13, 14, -1, -1, -1, 15, 16, -1};
const int colToRowC[10] = {17, 18, 19, 20, -1, -1, -1, 21, 22, -1};
const int colToRowD[10] = {-1, -1, -1, -1, -1, 23, 23, 24, 25, 25};
const int colToRowE[10] = {28, 29, 30, 31, 32, 33, 34, 35, 36, 37};

// ================================================================
// ── §3  函数具体实现区 ──────────────────────────────────────────
// ================================================================

double calcCoordinate(int low, int high) {
  int raw = low + high * 256;
  if (raw & 0x8000) return (double)raw - 32768.0;
  return (double)(0 - raw);
}

double calcUint16(int low, int high) {
  return (double)(low + high * 256);
}

double kalmanUpdate(KalmanFilter &kfCurrent, double measurement) {
  if (!kfCurrent.init) {
    kfCurrent.x_est = measurement;
    kfCurrent.p_est = 1.0;
    kfCurrent.init  = true;
    return measurement;
  }
  double p_pred = kfCurrent.p_est + KF_Q;
  double K      = p_pred / (p_pred + KF_R);
  kfCurrent.x_est      = kfCurrent.x_est + K * (measurement - kfCurrent.x_est);
  kfCurrent.p_est      = (1.0 - K) * p_pred;
  return kfCurrent.x_est;
}

int findBox(const BoxDef* defs, int count, double angle) {
  for (int i = 0; i < count; i++) {
    double lo = defs[i].angleMin;
    double hi = defs[i].angleMax;
    bool inRange = (i == count - 1)
      ? (angle >= lo && angle <= hi)
      : (angle >= lo && angle <  hi);
    if (inRange) return defs[i].id;
  }
  return -1;
}

int angleToCol(double angle) {
  if (angle < ZONE_ANGLE_MIN) return 0;
  if (angle > ZONE_ANGLE_MAX) return NUM_ZONES - 1;
  int col = (int)floor((angle - ZONE_ANGLE_MIN) /
            ((ZONE_ANGLE_MAX - ZONE_ANGLE_MIN) / NUM_ZONES));
  if (col < 0)           col = 0;
  if (col >= NUM_ZONES)  col = NUM_ZONES - 1;
  return col;
}

int distToLayer(double dist) {
  if (dist > DIST_B_MAX)  return 0;  // 行A
  if (dist > DIST_C_MAX)  return 1;  // 行B
  if (dist > DIST_D_MAX)  return 2;  // 行C
  if (dist > DIST_E_MAX)  return 3;  // 行D
  return 4;                          // 行E
}

void getColBoxes(int col, int out[5]) {
  int c = col - 1;  // 转0-based
  out[0] = col;                          // 行A
  out[1] = (c >= 0 && c < 10) ? colToRowB[c] : -1;  // 行B
  out[2] = (c >= 0 && c < 10) ? colToRowC[c] : -1;  // 行C
  out[3] = (c >= 0 && c < 10) ? colToRowD[c] : -1;  // 行D
  out[4] = (c >= 0 && c < 10) ? colToRowE[c] : -1;  // 行E
}

int distToBrightness(double dist) {
  double ratio = 1.0 - (dist / DIST_A_MAX);  // 近=1 远=0
  ratio = constrain(ratio, 0.0, 1.0);
  int brightness = (int)(LED_BRIGHTNESS_MIN +
    ratio * (LED_BRIGHTNESS_MAX - LED_BRIGHTNESS_MIN));
  return brightness;
}

int distToServoUs(double dist) {
  double ratio = 1.0 - (dist / DIST_A_MAX);
  ratio = constrain(ratio, 0.0, 1.0);
  int us = (int)(SERVO_MIN_US + ratio * (SERVO_MAX_US - SERVO_MIN_US));
  return us;
}

void applyLED(int col, int brightness, bool on) {
  if (col < 0 || col >= 10) return;
  int pwm = on ? map(brightness, LED_BRIGHTNESS_MIN, LED_BRIGHTNESS_MAX, 30, 255) : 0;
  analogWrite(LED_PIN[col], pwm);
}

void applyServo(int col, int servoUs, bool run) {
  if (col < 0 || col >= 10) return;
  int duty = run ? (int)((long)servoUs * 255 / 20000) : 0;
  analogWrite(SERVO_PIN[col], duty);
}

void clearZones() {
  for (int c = 0; c < NUM_ZONES; c++)
    for (int l = 0; l < 5; l++)
      zoneActive[c][l] = false;
}

void markZone(double angle, double dist) {
  if (angle < ZONE_ANGLE_MIN || angle > ZONE_ANGLE_MAX) return;
  if (dist <= 0 || dist > DIST_A_MAX) return;
  int col   = angleToCol(angle);
  int layer = distToLayer(dist);
  zoneActive[col][layer] = true;
}

void summarizeZones() {
  activeCol      = -1;
  activeDistance = DIST_A_MAX;

  double closestDist = DIST_A_MAX + 1.0;
  int    closestCol  = -1;

  const double layerDist[5] = {
    (DIST_B_MAX + DIST_A_MAX) / 2.0,  // 行A
    (DIST_C_MAX + DIST_B_MAX) / 2.0,  // 行B
    (DIST_D_MAX + DIST_C_MAX) / 2.0,  // 行C
    (DIST_E_MAX + DIST_D_MAX) / 2.0,  // 行D
    DIST_E_MAX / 2.0                   // 行E
  };

  for (int c = 0; c < NUM_ZONES; c++) {
    for (int l = 0; l < 5; l++) {
      if (zoneActive[c][l]) {
        if (layerDist[l] < closestDist) {
          closestDist = layerDist[l];
          closestCol  = c;
        }
      }
    }
  }

  activeCol      = closestCol;
  activeDistance = (closestCol >= 0) ? closestDist : DIST_A_MAX;
}

void driveOutputs() {
  for (int c = 0; c < 10; c++) {
    applyLED(c, LED_BRIGHTNESS_MIN, false);
    applyServo(c, SERVO_STOP_US, false);
  }

  if (activeCol < 0) return;

  int brightness = distToBrightness(activeDistance);
  int servoUs    = distToServoUs(activeDistance);

  int colBoxes[5];
  getColBoxes(activeCol + 1, colBoxes);

  applyLED(activeCol, brightness, true);
  applyServo(activeCol, servoUs, true);
}

void printStatus() {
  Serial.println();
  Serial.println("══════════════════════════════════════");
  Serial.println("  LD2450 图1正面箱体系统 v3");
  Serial.println("══════════════════════════════════════");

  if (activeCol >= 0) {
    int colBoxes[5];
    getColBoxes(activeCol + 1, colBoxes);
    int brightness = distToBrightness(activeDistance);
    int servoUs    = distToServoUs(activeDistance);

    Serial.print("[Zone] 激活列: "); Serial.print(activeCol + 1);
    Serial.print("  距离: "); Serial.print(activeDistance, 2); Serial.println("m");
    Serial.print("[LED]  亮度="); Serial.print(brightness);
    Serial.print("  色调=白蓝(R"); Serial.print(LED_COLOR_R);
    Serial.print(" G"); Serial.print(LED_COLOR_G);
    Serial.print(" B"); Serial.print(LED_COLOR_B); Serial.println(")");
    Serial.print("[舵机] 脉宽="); Serial.print(servoUs); Serial.println("μs");
    Serial.print("[竖列] 同时激活箱体: ");
    const char* rowName[5] = {"行A","行B","行C","行D","行E"};
    for (int r = 0; r < 5; r++) {
      if (colBoxes[r] > 0) {
        Serial.print(rowName[r]);
        Serial.print("#"); Serial.print(colBoxes[r]);
        Serial.print(" ");
      }
    }
    Serial.println();
  } else {
    Serial.println("[Zone] 无有效目标");
    Serial.println("[LED]  全灭");
    Serial.println("[舵机] 停止");
  }

  Serial.println("--------------------------------------");
  bool anyTarget = false;
  for (int i = 0; i < 3; i++) {
    if (!targets[i].valid) continue;
    anyTarget = true;
    Serial.print("[目标"); Serial.print(i+1); Serial.print("]");
    Serial.print(" 原始("); Serial.print(targets[i].x,0);
    Serial.print(","); Serial.print(targets[i].y,0); Serial.print(")");
    Serial.print(" 滤波("); Serial.print(targets[i].fx,0);
    Serial.print(","); Serial.print(targets[i].fy,0); Serial.print(")");
    Serial.print(" 角="); Serial.print(targets[i].fAngle,1);
    Serial.print("° 距="); Serial.print(targets[i].fDist,2); Serial.println("m");
  }
  if (!anyTarget) Serial.println("（无原始目标数据）");
  Serial.println("══════════════════════════════════════");
}

void parseFrame(uint8_t *frame) {
  int off = 4;
  for (int i = 0; i < 3; i++) {
    int xL=frame[off+0], xH=frame[off+1];
    int yL=frame[off+2], yH=frame[off+3];
    int sL=frame[off+4], sH=frame[off+5];
    int rL=frame[off+6], rH=frame[off+7];
    off += 8;

    bool allZero = !xL&&!xH&&!yL&&!yH&&!sL&&!sH&&!rL&&!rH;
    if (allZero) {
      targets[i].valid = false;
      kf[i].kfX.init = false;
      kf[i].kfY.init = false;
      continue;
    }

    targets[i].x          = calcCoordinate(xL, xH);
    targets[i].y          = calcCoordinate(yL, yH);
    targets[i].speed      = calcCoordinate(sL, sH);
    targets[i].resolution = calcUint16(rL, rH);
    targets[i].distance   = sqrt(targets[i].x*targets[i].x +
                                 targets[i].y*targets[i].y) / 1000.0;
    targets[i].angle      = atan2(targets[i].x, targets[i].y) * 180.0 / PI;
    targets[i].valid      = true;

    targets[i].fx     = kalmanUpdate(kf[i].kfX, targets[i].x);
    targets[i].fy     = kalmanUpdate(kf[i].kfY, targets[i].y);
    targets[i].fDist  = sqrt(targets[i].fx*targets[i].fx +
                             targets[i].fy*targets[i].fy) / 1000.0;
    targets[i].fAngle = atan2(targets[i].fx, targets[i].fy) * 180.0 / PI;
  }
}

void processFrame() {
  clearZones();
  for (int i = 0; i < 3; i++) {
    if (!targets[i].valid) continue;
    markZone(targets[i].fAngle, targets[i].fDist);
  }
  summarizeZones();
  driveOutputs();
}

void parseFromRingBuffer() {
  while (ringCount >= FRAME_LEN) {
    int frameStartOffset = -1;
    for (int i = 0; i <= ringCount - FRAME_LEN; i++) {
      int p0 = (ringTail+i+0) % BUFFER_SIZE;
      int p1 = (ringTail+i+1) % BUFFER_SIZE;
      int p2 = (ringTail+i+2) % BUFFER_SIZE;
      int p3 = (ringTail+i+3) % BUFFER_SIZE;
      if (ringBuf[p0]==0xAA && ringBuf[p1]==0xFF &&
          ringBuf[p2]==0x03 && ringBuf[p3]==0x00) {
        frameStartOffset = i;
        break;
      }
    }
    if (frameStartOffset == -1) {
      int drop = ringCount - FRAME_LEN + 1;
      ringTail   = (ringTail + drop) % BUFFER_SIZE;
      ringCount -= drop;
      noHeadCount++;
      return;
    }
    if (frameStartOffset > 0) {
      ringTail   = (ringTail + frameStartOffset) % BUFFER_SIZE;
      ringCount -= frameStartOffset;
    }
    if (ringCount < FRAME_LEN) return;

    int tail1 = (ringTail + 28) % BUFFER_SIZE;
    int tail2 = (ringTail + 29) % BUFFER_SIZE;
    if (ringBuf[tail1]==0x55 && ringBuf[tail2]==0xCC) {
      uint8_t frame[FRAME_LEN];
      for (int i = 0; i < FRAME_LEN; i++)
        frame[i] = ringBuf[(ringTail+i) % BUFFER_SIZE];
      validFrameCount++;
      parseFrame(frame);
      processFrame();

      if (millis() - lastOutputTime >= 100) {
        printStatus();
        lastOutputTime = millis();
      }
      ringTail   = (ringTail + FRAME_LEN) % BUFFER_SIZE;
      ringCount -= FRAME_LEN;
    } else {
      ringTail  = (ringTail + 1) % BUFFER_SIZE;
      ringCount--;
      fakeHeadCount++;
    }
  }
}

// ================================================================
// ── §4  系统初始化与主循环 ──────────────────────────────────────
// ================================================================

void setup() {
  Serial.begin(115200);
  delay(500);

  // 初始化雷达串口
  Serial1.begin(RADAR_BAUD, SERIAL_8N1, RADAR_RX_PIN, RADAR_TX_PIN);

  // 初始化 LED & 舵机引脚
  for (int c = 0; c < 10; c++) {
    pinMode(LED_PIN[c],   OUTPUT);
    pinMode(SERVO_PIN[c], OUTPUT);
    analogWrite(LED_PIN[c],   0);
    analogWrite(SERVO_PIN[c], 0);
  }

  // 初始化卡尔曼滤波器
  for (int i = 0; i < 3; i++) {
    kf[i].kfX = {0, 1.0, false};
    kf[i].kfY = {0, 1.0, false};
  }

  // 清空 Zone
  clearZones();

  // 清空雷达缓冲
  while (Serial1.available()) Serial1.read();

  Serial.println();
  Serial.println("══════════════════════════════════════");
  Serial.println("  LD2450 图1正面箱体系统 v3 启动");
  Serial.println("  §3 距离阈值 §4 灯光 §5 舵机 §6 卡尔曼");
  Serial.println("══════════════════════════════════════");
}

void loop() {
  // 读雷达字节入环形缓冲
  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    byteCount++;
    ringBuf[ringHead] = b;
    ringHead = (ringHead + 1) % BUFFER_SIZE;
    if (ringCount < BUFFER_SIZE) {
      ringCount++;
    } else {
      ringTail = (ringTail + 1) % BUFFER_SIZE;
    }
  }

  // 尝试解析完整帧
  parseFromRingBuffer();

  // 定时打印系统统计
  if (millis() - lastStatusTime >= 5000) {
    Serial.print("[STAT] bytes=");    Serial.print(byteCount);
    Serial.print(" frames=");        Serial.print(validFrameCount);
    Serial.print(" fakeHead=");      Serial.print(fakeHeadCount);
    Serial.print(" noHead=");        Serial.println(noHeadCount);
    lastStatusTime = millis();
  }
}
