# CatchGameMod (ESP32-S3 Matrix, ESP-NOW)
将单文件工程拆分为模块化结构：显示、逻辑、无线、电气配置与全局状态分离。

## 目录
- `CatchGameMod.ino` 入口（setup/loop）
- `config.h` 常量、枚举、数据包
- `globals.h / globals.cpp` 全局对象与状态
- `utils.h / utils.cpp` 工具方法（颜色、按钮、距离换算、边框绘制等）
- `display.h / display.cpp` 矩阵显示渲染
- `logic.h / logic.cpp` 游戏逻辑与按键处理
- `radio.h / radio.cpp` ESP‑NOW 初始化、收发回调、心跳与抓捕

## 依赖
- `esp32` 内核
- `Adafruit_NeoPixel` / `Adafruit_NeoMatrix`
- WiFi/ESP‑NOW 由 ESP32 SDK 提供

## 使用
1. 保证文件夹名与 `.ino` 同名（已为 `CatchGameMod`）。
2. 使用 Arduino IDE 打开 `.ino` 并选择你的 ESP32‑S3 板卡与端口。
3. 编译上传即可。

> 如需更换按键或引脚、队伍颜色、冷却时间等，修改 `config.h` 即可。
