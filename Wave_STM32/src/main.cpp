#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>  // 👈 虽然我们不用 SPI，但加上这一行可以强制引导编译器加载 SPI 库

// 定向硬件引脚 (对应右侧排针 B7 和 B6)
#define CUSTOM_I2C_SDA   PB7   
#define CUSTOM_I2C_SCL   PB6   

TwoWire myI2C(CUSTOM_I2C_SDA, CUSTOM_I2C_SCL);

void setup() {
  Serial.begin(115200);
  delay(2000);
  Serial.println("\n--- I2C 设备扫描器启动 ---");
  myI2C.begin();
}

void loop() {
  byte error, address;
  int nDevices = 0;

  Serial.println("正在扫描 I2C 总线上的所有设备...");

  // 扫描 1 到 127 的所有 I2C 地址
  for (address = 1; address < 127; address++ ) {
    myI2C.beginTransmission(address);
    error = myI2C.endTransmission();

    if (error == 0) {
      Serial.print("发现 I2C 设备！硬件地址: 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("");
      nDevices++;
    }
  }

  if (nDevices == 0) {
    Serial.println("[警告] 未发现任何 I2C 设备！请检查 SDA/SCL 是否接错。\n");
  } else {
    Serial.print("扫描完毕，共发现 ");
    Serial.print(nDevices);
    Serial.println(" 个设备。\n");
  }

  delay(4000); // 每 4 秒扫一次
}