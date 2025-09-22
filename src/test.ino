#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>

// 定义引脚
#define BUTTON1_PIN 3     // 第一个按钮连接到GPIO3
#define BUTTON2_PIN 7     // 第二个按钮连接到GPIO7
#define MATRIX_PIN 14     // LED矩阵连接到GPIO14 (与main.ino保持一致)

// 矩阵配置
#define MW 8
#define MH 8
Adafruit_NeoMatrix matrix(
  MW, MH, MATRIX_PIN,
  NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
  NEO_RGB + NEO_KHZ800
);

// 按钮状态变量
bool button1_state = false;
bool button2_state = false;
bool last_button1_state = false;
bool last_button2_state = false;

// LED矩阵状态和模式
int display_mode = 0;  // 0: 关闭, 1: 模式1, 2: 模式2, 3: 模式3
unsigned long last_update = 0;
int animation_frame = 0;

void setup() {
  Serial.begin(115200);
  
  // 设置按钮引脚为输入，启用内部上拉电阻
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  
  // 初始化LED矩阵
  matrix.begin();
  matrix.setBrightness(40);  // 设置适中的亮度
  matrix.fillScreen(0);      // 清空显示
  matrix.show();
  
  Serial.println("ESP32 按钮控制LED矩阵启动");
  Serial.println("按下GPIO3或GPIO7的按钮来切换显示模式");
  Serial.println("模式: 0=关闭, 1=彩虹, 2=呼吸灯, 3=随机点阵");
}

// 显示模式函数
void displayMode0() {
  // 模式0: 关闭所有LED
  matrix.fillScreen(0);
}

void displayMode1() {
  // 模式1: 彩虹渐变效果
  matrix.fillScreen(0);
  for (int x = 0; x < MW; x++) {
    for (int y = 0; y < MH; y++) {
      int hue = (x * 32 + y * 32 + animation_frame * 4) % 256;
      uint32_t color = matrix.ColorHSV(hue * 256);
      matrix.drawPixel(x, y, color);
    }
  }
}

void displayMode2() {
  // 模式2: 呼吸灯效果
  int brightness = (sin(animation_frame * 0.1) + 1) * 127;
  uint16_t color = matrix.Color(brightness, 0, brightness);
  matrix.fillScreen(color);
}

void displayMode3() {
  // 模式3: 随机点阵闪烁
  static unsigned long last_random_update = 0;
  if (millis() - last_random_update > 200) {
    matrix.fillScreen(0);
    for (int i = 0; i < 10; i++) {
      int x = random(MW);
      int y = random(MH);
      uint16_t color = matrix.Color(random(255), random(255), random(255));
      matrix.drawPixel(x, y, color);
    }
    last_random_update = millis();
  }
}

void updateDisplay() {
  // 根据当前模式更新显示
  switch (display_mode) {
    case 0: displayMode0(); break;
    case 1: displayMode1(); break;
    case 2: displayMode2(); break;
    case 3: displayMode3(); break;
    default: display_mode = 0; break;
  }
  matrix.show();
}

void loop() {
  // 读取按钮状态（按下时为LOW，因为使用了上拉电阻）
  button1_state = !digitalRead(BUTTON1_PIN);
  button2_state = !digitalRead(BUTTON2_PIN);
  
  // 检测按钮1的按下事件（下一个模式）
  if (button1_state && !last_button1_state) {
    display_mode = (display_mode + 1) % 4;  // 循环切换模式 0-3
    animation_frame = 0;  // 重置动画帧
    Serial.println("按钮1被按下，切换到模式: " + String(display_mode));
    delay(50);  // 简单的去抖动
  }
  
  // 检测按钮2的按下事件（上一个模式）
  if (button2_state && !last_button2_state) {
    display_mode = (display_mode - 1 + 4) % 4;  // 循环切换模式 3-0
    animation_frame = 0;  // 重置动画帧
    Serial.println("按钮2被按下，切换到模式: " + String(display_mode));
    delay(50);  // 简单的去抖动
  }
  
  // 保存当前按钮状态
  last_button1_state = button1_state;
  last_button2_state = button2_state;
  
  // 更新动画帧和显示
  if (millis() - last_update > 50) {  // 每50ms更新一次动画
    animation_frame++;
    last_update = millis();
  }
  
  updateDisplay();
  delay(10);  // 小延迟避免过度占用CPU
}