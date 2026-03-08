#include <Arduino.h>
#include "config.h"
#include "globals.h"
#include "utils.h"
#include "display.h"
#include "logic.h"
#include "radio.h"

void setup() {
  Serial.begin(115200);
  delay(200);

  // 初始化按键
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  // 读取初始按钮状态，消抖并提示
  delay(50);
  int rawButton1 = digitalRead(BUTTON1_PIN);
  bool initialButton1State = (rawButton1 == HIGH);  // 反转逻辑！
  Serial.printf("按钮1初始状态: digitalRead=%d, 判定为%s\n",
                rawButton1, initialButton1State ? "按下" : "未按下");
  button1Pressing = false;
  button1PressStartTime = 0;
  if (initialButton1State) {
    Serial.println("警告: 启动时检测到按钮1被按下，请释放按钮");
  } else {
    Serial.println("按钮状态正常");
  }

  // LED矩阵
  matrix.begin();
  matrix.setBrightness(40);
  matrix.fillScreen(0);

  // 启动画面
  for (int i = 0; i < 8; i++) {
    matrix.fillScreen(0);
    matrix.drawPixel(i, i, matrix.Color(32, 32, 32));
    if (i > 0) matrix.drawPixel(i - 1, i - 1, matrix.Color(16, 16, 16));
    matrix.show();
    delay(100);
  }
  matrix.fillScreen(0);
  matrix.show();

  // 无线
  WiFi.macAddress(selfMac);
  initWiFiEspNow();

  Serial.println("ESP32-S3 抓捕游戏已启动");
  Serial.printf("MAC地址: %s\n", macToString(selfMac).c_str());
  Serial.println("=== 队伍选择模式 ===");
  Serial.println("按键2: 切换队伍颜色 (红->绿->蓝->黄->红...)");
  Serial.println("按键1: 确认选择的队伍");
  Serial.printf("当前选择: %d\n", (int)selectedTeam);
}

void loop() {
  unsigned long now = millis();

  // 调试：每2秒打印一次按钮状态
  static unsigned long lastDebugPrint = 0;
  if (now - lastDebugPrint > 2000) {
    int rawBtn = digitalRead(BUTTON1_PIN);
    Serial.printf("[MONITOR] 按钮1: digitalRead=%d, button1Pressing=%s\n",
                  rawBtn, button1Pressing ? "true" : "false");
    lastDebugPrint = now;
  }

  // 逻辑
  handleGameLogic();

  // 心跳
  if (now - lastPing >= PING_INTERVAL) {
    sendPing();
    lastPing = now;
  }

  // 显示
  updateMatrix();

  delay(50);
}
