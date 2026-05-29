#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

// ---------------- Hardware configuration ----------------

#define I2C_SDA_PIN PB7
#define I2C_SCL_PIN PB6

static constexpr uint8_t kServoDriverCount = 4;
static constexpr uint8_t kFrontServoCount = 38;
static constexpr uint8_t kSideServoCount = 14;
static constexpr uint8_t kServoCount = kFrontServoCount + kSideServoCount;
static constexpr uint8_t kFrontLedCount = 10;
static constexpr uint8_t kSideLedCount = 5;
static constexpr uint8_t kLedCount = kFrontLedCount + kSideLedCount;

static const uint8_t kServoDriverAddr[kServoDriverCount] = {
  0x40, // front boxes 1-16
  0x60, // front boxes 17-32
  0x50, // front boxes 33-38
  0x68  // side boxes 1-14, channels 2-15
};

struct ServoBox {
  uint8_t driver;
  uint8_t channel;
  uint8_t box;
  uint8_t x;
  uint8_t y;
  bool side;
};

// Spatial positions come from position_mapping.md, not from the old README.
// x/y are zero-based column/row coordinates used only to phase the fallback wave.
static const ServoBox kServoBox[kServoCount] = {
  {0, 0, 1, 0, 0, false},   {0, 1, 2, 1, 0, false},
  {0, 2, 3, 2, 0, false},   {0, 3, 4, 3, 0, false},
  {0, 4, 5, 4, 0, false},   {0, 5, 6, 5, 0, false},
  {0, 6, 7, 6, 0, false},   {0, 7, 8, 7, 0, false},
  {0, 8, 9, 8, 0, false},   {0, 9, 10, 9, 0, false},
  {0, 10, 11, 0, 1, false}, {0, 11, 12, 1, 1, false},
  {0, 12, 13, 2, 1, false}, {0, 13, 14, 3, 1, false},
  {0, 14, 15, 8, 1, false}, {0, 15, 16, 9, 1, false},

  {1, 0, 17, 0, 2, false},  {1, 1, 18, 1, 2, false},
  {1, 2, 19, 2, 2, false},  {1, 3, 20, 3, 2, false},
  {1, 4, 21, 8, 2, false},  {1, 5, 22, 9, 2, false},
  {1, 6, 23, 5, 3, false},  {1, 7, 24, 6, 3, false},
  {1, 8, 25, 9, 3, false},  {1, 9, 26, 5, 4, false},
  {1, 10, 27, 6, 4, false}, {1, 11, 28, 0, 5, false},
  {1, 12, 29, 1, 5, false}, {1, 13, 30, 2, 5, false},
  {1, 14, 31, 3, 5, false}, {1, 15, 32, 4, 5, false},

  {2, 0, 33, 5, 5, false},  {2, 1, 34, 6, 5, false},
  {2, 2, 35, 7, 5, false},  {2, 3, 36, 8, 5, false},
  {2, 4, 37, 9, 5, false},  {2, 5, 38, 10, 5, false},

  {3, 2, 14, 3, 4, true},   {3, 3, 1, 0, 0, true},
  {3, 4, 2, 1, 0, true},    {3, 5, 3, 2, 0, true},
  {3, 6, 4, 0, 1, true},    {3, 7, 5, 1, 1, true},
  {3, 8, 6, 2, 2, true},    {3, 9, 7, 3, 2, true},
  {3, 10, 8, 1, 3, true},   {3, 11, 9, 2, 3, true},
  {3, 12, 10, 3, 3, true},  {3, 13, 11, 0, 4, true},
  {3, 14, 12, 1, 4, true},  {3, 15, 13, 2, 4, true}
};

static const uint32_t kLedPin[kLedCount] = {
  PE0, PE1, PE2, PE3, PE4, PE5, PE6, PE7, PE8, PE9,
  PD0, PD1, PD2, PD3, PD4
};

// ---------------- Effect tuning ----------------

static constexpr uint16_t kServoStopUs = 1500;
static constexpr uint16_t kServoMinRunUs = 1520;
static constexpr uint16_t kServoMaxRunUs = 1600;

static constexpr uint8_t kLedMinBrightness = 2;
static constexpr uint8_t kLedMaxBrightness = 20;

static constexpr uint32_t kFrameMs = 20;
static constexpr uint32_t kBreathPeriodMs = 5200;
static constexpr float kXPhase = 0.32f;
static constexpr float kYPhase = 0.46f;
static constexpr float kSidePhaseOffset = 1.15f;

TwoWire planbI2C(I2C_SDA_PIN, I2C_SCL_PIN);
Adafruit_PWMServoDriver servoDriver0(kServoDriverAddr[0], planbI2C);
Adafruit_PWMServoDriver servoDriver1(kServoDriverAddr[1], planbI2C);
Adafruit_PWMServoDriver servoDriver2(kServoDriverAddr[2], planbI2C);
Adafruit_PWMServoDriver servoDriver3(kServoDriverAddr[3], planbI2C);
Adafruit_PWMServoDriver* servoDriver[kServoDriverCount] = {
  &servoDriver0, &servoDriver1, &servoDriver2, &servoDriver3
};

static uint32_t lastFrameMs = 0;

float wave01(float phase) {
  return 0.5f - 0.5f * cosf(phase);
}

uint8_t brightnessForBreath(float value) {
  value = constrain(value, 0.0f, 1.0f);
  float gammaCorrected = value * value;
  return (uint8_t)(kLedMinBrightness +
                   gammaCorrected * (kLedMaxBrightness - kLedMinBrightness));
}

uint16_t servoPulseForBreath(float value) {
  value = constrain(value, 0.0f, 1.0f);
  float gammaCorrected = value * value;
  int pulse = (int)(kServoMinRunUs +
                    gammaCorrected * (kServoMaxRunUs - kServoMinRunUs));
  return (uint16_t)constrain(pulse, kServoMinRunUs, kServoMaxRunUs);
}

float phaseForPosition(uint8_t x, uint8_t y, bool side) {
  return x * kXPhase + y * kYPhase + (side ? kSidePhaseOffset : 0.0f);
}

void writeServoUs(const ServoBox& servo, uint16_t pulseUs) {
  if (servo.driver >= kServoDriverCount) {
    return;
  }
  servoDriver[servo.driver]->writeMicroseconds(servo.channel, pulseUs);
}

void writeLedChannel(uint8_t index, uint8_t pwm) {
  if (index >= kLedCount) {
    return;
  }
  analogWrite(kLedPin[index], pwm);
}

void allOutputsIdle() {
  for (uint8_t i = 0; i < kServoCount; i++) {
    writeServoUs(kServoBox[i], kServoStopUs);
  }
  for (uint8_t i = 0; i < kLedCount; i++) {
    writeLedChannel(i, 0);
  }
}

void setup() {
  Serial.begin(115200);
  delay(800);

  planbI2C.begin();
  for (uint8_t i = 0; i < kServoDriverCount; i++) {
    servoDriver[i]->begin();
    servoDriver[i]->setOscillatorFrequency(27000000);
    servoDriver[i]->setPWMFreq(50);
    delay(10);
  }

  for (uint8_t i = 0; i < kLedCount; i++) {
    pinMode(kLedPin[i], OUTPUT);
    analogWrite(kLedPin[i], 0);
  }

  allOutputsIdle();

  Serial.println();
  Serial.println("PlanB breathing controller started");
  Serial.println("Servos: 4 PCA9685 boards, speed follows LED brightness");
  Serial.println("LEDs: brightness range 2-20, front PE0-PE9, side PD0-PD4");
}

void loop() {
  uint32_t now = millis();
  if (now - lastFrameMs < kFrameMs) {
    return;
  }
  lastFrameMs = now;

  const float twoPi = 6.28318530718f;
  float basePhase = ((now % kBreathPeriodMs) / (float)kBreathPeriodMs) * twoPi;

  for (uint8_t i = 0; i < kServoCount; i++) {
    const ServoBox& servo = kServoBox[i];
    float phase = basePhase + phaseForPosition(servo.x, servo.y, servo.side);
    float breath = wave01(phase);
    writeServoUs(servo, servoPulseForBreath(breath));
  }

  for (uint8_t i = 0; i < kLedCount; i++) {
    bool side = i >= kFrontLedCount;
    uint8_t col = side ? (i - kFrontLedCount) : i;
    float phase = basePhase + col * kXPhase + (side ? kSidePhaseOffset : 0.0f);
    writeLedChannel(i, brightnessForBreath(wave01(phase)));
  }
}
