#include <Arduino.h>
#include <math.h>

// ╔══════════════════════════════════════════════════════════════╗
// ║  LD2450 图1正面箱体系统 v3                                   ║
// ║  新增：竖列联动 / 灯板颜色亮度 / 舵机转速 / 虚拟Zone网格     ║
// ║        卡尔曼滤波 / 对区不对人逻辑 / 分段注释                 ║
// ╚══════════════════════════════════════════════════════════════╝

// ################################################################
// §0  BoxDef 结构体（必须最先声明，Arduino IDE 不自动前向声明结构体）
// ################################################################
struct BoxDef {
  int    id;
  double angleMin;
  double angleMax;
};

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

// ################################################################
// §1  硬件引脚 & 串口配置
//     ★ 需要更换引脚时只改这里 ★
// ################################################################
#define RADAR_RX_PIN   4       // 雷达 TX → ESP32 RX
#define RADAR_TX_PIN   5       // 雷达 RX → ESP32 TX（可不接）
#define RADAR_BAUD     256000

// §1-A  舵机 PWM 引脚（每列一个，共10列）
//        列号:      1    2    3    4    5    6    7    8    9   10
const int SERVO_PIN[10] = {12, 13, 14, 15, 16, 17, 18, 19, 21, 22};

// §1-B  灯板 PWM 引脚（每列一个，共10列）
//        列号:      1    2    3    4    5    6    7    8    9   10
const int LED_PIN[10]   = {25, 26, 27, 32, 33, 34, 35, 36, 39, 23};
//  注：GPIO 34/35/36/39 仅输入，实际接线请用带输出能力的引脚，此处为示意

// ################################################################
// §2  帧解析常量
// ################################################################
#define FRAME_LEN    30
#define BUFFER_SIZE  128
#define PRINT_EACH_BYTE false

// ################################################################
// §3  距离分层阈值（米）
//     ★ 调整距离触发边界只改这里 ★
// ################################################################
#define DIST_E_MAX   0.25   // 行E  0     ~ 0.25m  boxes 28-38
#define DIST_D_MAX   0.50   // 行D  0.25  ~ 0.50m  boxes 23-27
#define DIST_C_MAX   0.90   // 行C  0.50  ~ 0.90m  boxes 17-22
#define DIST_B_MAX   1.40   // 行B  0.90  ~ 1.40m  boxes 11-16
#define DIST_A_MAX   2.00   // 行A  1.40  ~ 2.00m  boxes  1-10

// ################################################################
// §4  灯板参数
//     ★ 调整颜色/亮度只改这里 ★
// ################################################################
#define LED_BRIGHTNESS_MIN   8    // 默认/最远时亮度（PWM 0-255 映射后）
#define LED_BRIGHTNESS_MAX  20    // 最近时最大亮度
// 白蓝混光比例（0=纯蓝 255=纯白），用双通道PWM实现时分别控制
// 单通道PWM板：亮度直接输出到 LED_PIN，颜色由硬件灯珠固定为白蓝
// 以下值仅用于串口调试打印，实际控灯见 applyLED()
#define LED_COLOR_R  200    // 白蓝光 R 分量
#define LED_COLOR_G  220    // 白蓝光 G 分量
#define LED_COLOR_B  255    // 白蓝光 B 分量（蓝色偏强）

// ################################################################
// §5  舵机参数
//     ★ 调整转速范围只改这里 ★
//     360°舵机：1500μs=停止，<1500=正转，>1500=反转
//     这里用 PWM duty（0-255）模拟，实际应用请换成 Servo 库
// ################################################################
#define SERVO_STOP_US    1500   // 停止脉宽(μs)
#define SERVO_MIN_US     1520   // 最慢正转（距离最远时）
#define SERVO_MAX_US     1600   // 最快正转（距离最近时）
// 无目标时自动停止

// ################################################################
// §6  卡尔曼滤波参数
//     ★ 调整滤波平滑度只改这里 ★
//     Q 越大 → 越信任新数据（响应快但抖动）
//     R 越大 → 越平滑（响应慢但稳定）
// ################################################################
#define KF_Q   0.05    // 过程噪声
#define KF_R   2.0     // 测量噪声

// ################################################################
// §7  虚拟 Zone 网格定义
//     ★ 调整 Zone 分区边界只改这里 ★
//     Zone 编号 0~9 对应角度列（同行A列），行方向由距离判断
//     "对区不对人"：只要有任意坐标点落入该 Zone 就激活
// ################################################################
#define NUM_ZONES   10          // 角度方向分10列，与行A对齐
#define ZONE_ANGLE_MIN  -40.0
#define ZONE_ANGLE_MAX   40.0
// Zone 在距离方向复用 DIST_X_MAX 分层，共5层×10列 = 50个 Zone 单元
// 激活状态数组：zoneActive[列][层] true=有目标
bool zoneActive[NUM_ZONES][5];  // 列0~9，层0=行A … 层4=行E

// ################################################################
// §8  卡尔曼滤波器结构（每个雷达目标独立一组）
// ################################################################
struct KalmanFilter {
  double x_est;   // 估计值
  double p_est;   // 估计误差协方差
  bool   init;
};

struct KFPair {
  KalmanFilter kfX;
  KalmanFilter kfY;
};
KFPair kf[3];  // 对应3个雷达目标槽

// ################################################################
// §9  目标数据结构
// ################################################################
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
Target targets[3];

// ################################################################
// §10  环形缓冲区
// ################################################################
uint8_t ringBuf[BUFFER_SIZE];
int ringHead  = 0;
int ringTail  = 0;
int ringCount = 0;

// ################################################################
// §11  系统统计 & 时序
// ################################################################
unsigned long lastOutputTime  = 0;
unsigned long lastStatusTime  = 0;
uint32_t byteCount       = 0;
uint32_t validFrameCount = 0;
uint32_t fakeHeadCount   = 0;
uint32_t noHeadCount     = 0;

// ################################################################
// §12  当前帧输出结果（由 Zone 逻辑汇总，非单目标）
// ################################################################
// 激活的列号（0-based），-1=无
int activeCol    = -1;
double activeDistance = 2.0;   // 当前最近目标距离（用于控制亮度/转速）

// ================================================================
// ── 函数区 ──────────────────────────────────────────────────────
// ================================================================

// ################################################################
// §F1  LD2450 字节解码
// ################################################################
double calcCoordinate(int low, int high) {
  int raw = low + high * 256;
  if (raw & 0x8000) return (double)raw - 32768.0;
  return (double)(0 - raw);
}
double calcUint16(int low, int high) {
  return (double)(low + high * 256);
}

// ################################################################
// §F2  卡尔曼滤波更新
//      输入测量值，返回滤波后估计值
// ################################################################
double kalmanUpdate(KalmanFilter &kf, double measurement) {
  if (!kf.init) {
    kf.x_est = measurement;
    kf.p_est = 1.0;
    kf.init  = true;
    return measurement;
  }
  // 预测
  double p_pred = kf.p_est + KF_Q;
  // 更新（卡尔曼增益）
  double K      = p_pred / (p_pred + KF_R);
  kf.x_est      = kf.x_est + K * (measurement - kf.x_est);
  kf.p_est      = (1.0 - K) * p_pred;
  return kf.x_est;
}

// ################################################################
// §F3  BoxDef 数组查找
// ################################################################
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

// ################################################################
// §F4  角度 → Zone 列号（0-based，0=最左 9=最右）
// ################################################################
int angleToCol(double angle) {
  if (angle < ZONE_ANGLE_MIN) return 0;
  if (angle > ZONE_ANGLE_MAX) return NUM_ZONES - 1;
  int col = (int)floor((angle - ZONE_ANGLE_MIN) /
            ((ZONE_ANGLE_MAX - ZONE_ANGLE_MIN) / NUM_ZONES));
  if (col < 0)           col = 0;
  if (col >= NUM_ZONES)  col = NUM_ZONES - 1;
  return col;
}

// ################################################################
// §F5  距离 → Zone 层号（0=行A远 … 4=行E极近）
// ################################################################
int distToLayer(double dist) {
  if (dist > DIST_B_MAX)  return 0;  // 行A
  if (dist > DIST_C_MAX)  return 1;  // 行B
  if (dist > DIST_D_MAX)  return 2;  // 行C
  if (dist > DIST_E_MAX)  return 3;  // 行D
  return 4;                          // 行E
}

// ################################################################
// §F6  竖列映射表
//      给定行A的箱体号（1~10），返回同列其他行的箱体号列表
//      列号：col 1~10（与行A box编号一致）
//
//      行A:  box = col          (1~10)
//      行B:  col1→11, col2→12, col3→13, col4→14,
//            col5/6/7→玻璃(-1), col8→15, col9→16, col10→玻璃(-1)
//      行C:  同行B偏移 +6       (17~22)
//      行D:  col6~8→23, col9→24, col10→25 (仅右半)
//      行E:  box = 27 + col    (28~38，col1→28 … col10→37)
//            注：行E共11格，列1~10各对应一格，最右多1格(38)归入col10
//
//  返回：colBoxes[5]，索引0=行A，1=行B，2=行C，3=行D，4=行E
//        值=-1 表示该层该列无箱体（玻璃或无结构）
// ################################################################

// 行B 列映射（col 1-based → box id，-1=玻璃）
const int colToRowB[10] = {11, 12, 13, 14, -1, -1, -1, 15, 16, -1};
// 行C 列映射
const int colToRowC[10] = {17, 18, 19, 20, -1, -1, -1, 21, 22, -1};
// 行D 列映射（仅右半）
const int colToRowD[10] = {-1, -1, -1, -1, -1, 23, 23, 24, 25, 25};
// 行E 列映射（col1→28 … col10→37，38归col10不单独列出）
const int colToRowE[10] = {28, 29, 30, 31, 32, 33, 34, 35, 36, 37};

void getColBoxes(int col, int out[5]) {
  // col: 1-based
  int c = col - 1;  // 转0-based
  out[0] = col;                          // 行A
  out[1] = (c >= 0 && c < 10) ? colToRowB[c] : -1;  // 行B
  out[2] = (c >= 0 && c < 10) ? colToRowC[c] : -1;  // 行C
  out[3] = (c >= 0 && c < 10) ? colToRowD[c] : -1;  // 行D
  out[4] = (c >= 0 && c < 10) ? colToRowE[c] : -1;  // 行E
}

// ################################################################
// §F7  距离 → 灯板亮度（线性映射，距离越近越亮）
//      返回值范围: LED_BRIGHTNESS_MIN ~ LED_BRIGHTNESS_MAX
// ################################################################
int distToBrightness(double dist) {
  // dist 范围 0 ~ DIST_A_MAX
  double ratio = 1.0 - (dist / DIST_A_MAX);  // 近=1 远=0
  ratio = constrain(ratio, 0.0, 1.0);
  int brightness = (int)(LED_BRIGHTNESS_MIN +
    ratio * (LED_BRIGHTNESS_MAX - LED_BRIGHTNESS_MIN));
  return brightness;
}

// ################################################################
// §F8  距离 → 舵机脉宽（线性映射，距离越近转越快）
//      返回值单位: μs，用于 analogWrite 模拟或 Servo.writeMicroseconds()
// ################################################################
int distToServoUs(double dist) {
  double ratio = 1.0 - (dist / DIST_A_MAX);
  ratio = constrain(ratio, 0.0, 1.0);
  int us = (int)(SERVO_MIN_US + ratio * (SERVO_MAX_US - SERVO_MIN_US));
  return us;
}

// ################################################################
// §F9  实际驱动灯板（单通道PWM示例）
//      col: 0-based列号，brightness: LED_BRIGHTNESS_MIN~MAX
//      ★ 如使用 WS2812 等 RGB 灯带，在此替换为 FastLED/NeoPixel 调用 ★
// ################################################################
void applyLED(int col, int brightness, bool on) {
  if (col < 0 || col >= 10) return;
  // 将亮度 8~20 映射到 PWM 0~255
  int pwm = on ? map(brightness, LED_BRIGHTNESS_MIN, LED_BRIGHTNESS_MAX, 30, 255) : 0;
  analogWrite(LED_PIN[col], pwm);
}

// ################################################################
// §F10  实际驱动舵机（PWM 模拟，建议用 Servo 库替换）
//       col: 0-based列号，servoUs: 脉宽μs，run: 是否运转
//       ★ 替换为 Servo 库时：servo[col].writeMicroseconds(servoUs) ★
// ################################################################
void applyServo(int col, int servoUs, bool run) {
  if (col < 0 || col >= 10) return;
  // 简单 PWM 模拟，50Hz → period 20ms = 20000μs
  // duty = servoUs / 20000 * 255
  int duty = run ? (int)((long)servoUs * 255 / 20000) : 0;
  analogWrite(SERVO_PIN[col], duty);
}

// ################################################################
// §F11  Zone 网格清空（每帧开始时调用）
// ################################################################
void clearZones() {
  for (int c = 0; c < NUM_ZONES; c++)
    for (int l = 0; l < 5; l++)
      zoneActive[c][l] = false;
}

// ################################################################
// §F12  将一个滤波后坐标点标记进 Zone 网格
// ################################################################
void markZone(double angle, double dist) {
  if (angle < ZONE_ANGLE_MIN || angle > ZONE_ANGLE_MAX) return;
  if (dist <= 0 || dist > DIST_A_MAX) return;
  int col   = angleToCol(angle);
  int layer = distToLayer(dist);
  zoneActive[col][layer] = true;
}

// ################################################################
// §F13  从 Zone 网格汇总：找最近目标所在列与距离
//        "对区不对人"：不关心目标ID，只关心哪个Zone有点
// ################################################################
void summarizeZones() {
  activeCol      = -1;
  activeDistance = DIST_A_MAX;

  double closestDist = DIST_A_MAX + 1.0;
  int    closestCol  = -1;

  // 层距离代表值（中间值）
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

// ################################################################
// §F14  输出驱动：根据 activeCol / activeDistance
//        点亮对应竖列所有灯，并控制对应列舵机
// ################################################################
void driveOutputs() {
  // 先全部关闭
  for (int c = 0; c < 10; c++) {
    applyLED(c, LED_BRIGHTNESS_MIN, false);
    applyServo(c, SERVO_STOP_US, false);
  }

  if (activeCol < 0) return;  // 无目标，全灭

  int brightness = distToBrightness(activeDistance);
  int servoUs    = distToServoUs(activeDistance);

  // 点亮整列（col 0-based → 1-based传入getColBoxes）
  int colBoxes[5];
  getColBoxes(activeCol + 1, colBoxes);  // 转1-based

  // 点亮该列的 LED 和舵机
  applyLED(activeCol, brightness, true);
  applyServo(activeCol, servoUs, true);
}

// ################################################################
// §F15  串口调试输出
// ################################################################
void printStatus() {
  Serial.println();
  Serial.println("══════════════════════════════════════");
  Serial.println("  LD2450 图1正面箱体系统 v3");
  Serial.println("══════════════════════════════════════");

  // Zone 汇总
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

  // 各目标滤波后数据
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

// ################################################################
// §F16  解析一帧雷达数据（填充 targets[]）
// ################################################################
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
      // 目标消失时重置卡尔曼
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

    // §F16-KF  卡尔曼滤波 X / Y
    targets[i].fx     = kalmanUpdate(kf[i].kfX, targets[i].x);
    targets[i].fy     = kalmanUpdate(kf[i].kfY, targets[i].y);
    targets[i].fDist  = sqrt(targets[i].fx*targets[i].fx +
                             targets[i].fy*targets[i].fy) / 1000.0;
    targets[i].fAngle = atan2(targets[i].fx, targets[i].fy) * 180.0 / PI;
  }
}

// ################################################################
// §F17  主帧处理：清 Zone → 标记 → 汇总 → 驱动输出
// ################################################################
void processFrame() {
  // 1. 清空 Zone 网格
  clearZones();

  // 2. 所有有效目标的滤波坐标标记进 Zone（对区不对人）
  for (int i = 0; i < 3; i++) {
    if (!targets[i].valid) continue;
    markZone(targets[i].fAngle, targets[i].fDist);
  }

  // 3. 汇总：找最近激活 Zone
  summarizeZones();

  // 4. 驱动灯板 & 舵机
  driveOutputs();
}

// ################################################################
// §F18  解析环形缓冲区（帧同步 + 校验）
// ################################################################
void parseFromRingBuffer() {
  while (ringCount >= FRAME_LEN) {
    // 找帧头 AA FF 03 00
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

    // 校验帧尾 55 CC
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

// ################################################################
// §SETUP
// ################################################################
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

// ################################################################
// §LOOP
// ################################################################
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
