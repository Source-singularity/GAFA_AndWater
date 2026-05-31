#include <Arduino.h>
#include <FastLED.h>

// Use the board silkscreen pin D0, not STM32 port PD0.
static constexpr uint8_t kLedDataPin = 0;
static constexpr uint16_t kLedCount = 120;
static constexpr uint8_t kBrightness = 30;

CRGB leds[kLedCount];

void setup() {
  Serial.begin(115200);
  delay(800);

  FastLED.addLeds<WS2812B, kLedDataPin, GRB>(leds, kLedCount)
    .setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(kBrightness);

  fill_solid(leds, kLedCount, CRGB::Blue);
  FastLED.show();

  Serial.println("WS2812B test: solid blue on board D0");
}

void loop() {
  fill_solid(leds, kLedCount, CRGB::Blue);
  FastLED.show();
  delay(1000);
}
