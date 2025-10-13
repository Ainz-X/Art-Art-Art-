/*
 * ESP32-S3 Matrix - Nearby Radar (ESP-NOW + 8x8 NeoPixel)
 * - 每台板子定时广播心跳包
 * - 接收同伴并读取 RSSI，映射到 8 列柱状条（最多显示 8 台）
 * - 颜色随强度从红(-100)到绿(-30)渐变
 */

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_NeoPixel.h>
#include <Adafruit_NeoMatrix.h>
#include <map>
#include <vector>
#include <algorithm>

// ====== 引脚配置 ======
#define BUTTON1_PIN 3     // 第一个按钮连接到GPIO3 (开始/结束游戏)
#define BUTTON2_PIN 7     // 第二个按钮连接到GPIO7 (抓捕)
#define MATRIX_PIN 14     // LED矩阵连接到GPIO14 (与main.ino保持一致)

// ====== 矩阵配置（按你的实测已调好） ======
#define MW 8
#define MH 8
// 颜色顺序：你拍照呈红色 -> 说明应使用 RGB（不是默认 GRB）
#define COLOR_ORDER NEO_RGB
Adafruit_NeoMatrix matrix(
  MW, MH, MATRIX_PIN,
  // 物理首像素在板子左下角 → BOTTOM + LEFT
  NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS,
  COLOR_ORDER + NEO_KHZ800
);

// ====== Radar/ESP-NOW 参数 ======
static const char MAGIC[4] = {'R','D','R','1'};
static const uint8_t BCAST[6] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

static const uint8_t  WIFI_CHANNEL   = 1;      // 所有设备一致
static const uint32_t PING_INTERVAL  = 1000;   // 心跳间隔 ms
static const uint32_t PEER_EXPIRE_MS = 10000;  // 下线时间 ms
static const float    RSSI_ALPHA     = 0.7f;   // RSSI 平滑

// ====== 游戏参数 ======
static const int      CAPTURE_DISTANCE = -60;  // RSSI阈值，信号强于此值才能抓捕
static const uint32_t COOLDOWN_TIME_MS = 5000; // 冷却时间5秒
static const uint32_t SEARCH_TIME_MS   = 8000; // 搜索时间8秒

// ====== 游戏状态 ======
enum GameState {
  TEAM_SELECT,  // 队伍选择
  WAITING,      // 等待开始
  SEARCHING,    // 搜索设备中
  PLAYING,      // 游戏进行中
  VICTORY       // 胜利状态
};

enum PlayerTeam {
  TEAM_NEUTRAL = 0,
  TEAM_RED     = 1,
  TEAM_GREEN   = 2,
  TEAM_BLUE    = 3,
  TEAM_YELLOW  = 4,
  TEAM_PURPLE  = 5,
  TEAM_CYAN    = 6,
  TEAM_WHITE   = 7
};

// ====== 手动队伍选择 ======
PlayerTeam selectedTeam = TEAM_RED;  // 默认红队，可通过按键选择

GameState gameState = TEAM_SELECT;
PlayerTeam myTeam = TEAM_NEUTRAL;
unsigned long gameStartTime = 0;
unsigned long lastCaptureTime = 0;
bool canCapture = true;

// ====== 胜利状态 ======
unsigned long victoryTime = 0;
PlayerTeam winningTeam = TEAM_NEUTRAL;

// ====== 按键状态 ======
bool lastButton1 = false;
bool lastButton2 = false;
unsigned long lastButton1Press = 0;
unsigned long lastButton2Press = 0;
const unsigned long DEBOUNCE_TIME = 50;

// 长按重置功能
unsigned long button1PressStartTime = 0;
bool button1Pressing = false;
const unsigned long LONG_PRESS_TIME = 3000; // 3秒长按

struct Packet { 
  char magic[4]; 
  uint8_t mac[6]; 
  uint32_t seq; 
  PlayerTeam team;      // 玩家队伍
  GameState state;      // 游戏状态
  uint8_t captureCmd;   // 抓捕命令 (0=无, 1=抓捕, 2=被抓)
} __attribute__((packed));

struct PeerInfo { 
  int rssi; 
  unsigned long lastSeen; 
  PlayerTeam team;
  GameState state;
  bool justCaptured;
};

std::map<String, PeerInfo> peers;
uint8_t selfMac[6];
uint32_t seqNo = 0;
unsigned long lastPing = 0;

// ====== 函数声明 ======
float rssiToDistanceMeters(int rssi, float txPower = -59.0f, float n = 2.0f);
int distanceToEdgeLen(float d);
uint16_t colorForTeam(PlayerTeam team);
void drawEdgeBar(int side, int len, uint16_t color);
bool checkVictory();
// ====== 小工具 ======
String macToString(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  return String(buf);
}

inline int clampRSSI(int r){ return constrain(r, -100, -30); }

// 根据队伍返回颜色
uint16_t colorForTeam(PlayerTeam team) {
  switch(team) {
    case TEAM_RED:    return matrix.Color(255, 0, 0);
    case TEAM_GREEN:  return matrix.Color(0, 255, 0);
    case TEAM_BLUE:   return matrix.Color(0, 0, 255);
    case TEAM_YELLOW: return matrix.Color(255, 255, 0);
    case TEAM_PURPLE: return matrix.Color(255, 0, 255);
    case TEAM_CYAN:   return matrix.Color(0, 255, 255);
    case TEAM_WHITE:  return matrix.Color(255, 255, 255);
    default:          return matrix.Color(64, 64, 64); // 灰色表示中性
  }
}

uint16_t colorForRSSI(int rssi) {
  int v = clampRSSI(rssi);
  uint8_t r = map(v, -100, -30, 255, 0);
  uint8_t g = map(v, -100, -30, 0, 255);
  return matrix.Color(r, g, 0);
}

// 分配队伍颜色 (手动选择)
PlayerTeam assignTeamByMac(const uint8_t mac[6]) {
  return selectedTeam;
}

// 按键检测
bool readButton(int pin, bool &lastState, unsigned long &lastPress) {
  bool current = digitalRead(pin) == LOW; // 假设按下为LOW
  unsigned long now = millis();
  
  if (current != lastState && (now - lastPress) > DEBOUNCE_TIME) {
    lastPress = now;
    lastState = current;
    return current; // 返回按下状态
  }
  return false;
}
void prunePeers() {
  unsigned long now = millis();
  std::vector<String> dead;
  for (auto &kv : peers) if (now - kv.second.lastSeen > PEER_EXPIRE_MS) dead.push_back(kv.first);
  for (auto &k : dead) peers.erase(k);
}

// 检查是否所有玩家都是同一队伍（胜利条件）
bool checkVictory() {
  // 至少需要2个玩家（自己+至少1个其他玩家）
  if (peers.size() < 1) {
    return false;
  }
  
  // 检查所有在线玩家是否都是同一队伍
  for (auto &kv : peers) {
    if (kv.second.team != myTeam) {
      return false; // 发现不同队伍的玩家
    }
  }
  
  // 所有玩家都是同一队伍
  return true;
}

// ====== 游戏逻辑 ======
void handleGameLogic() {
  unsigned long now = millis();
  
  // 检查冷却时间
  if (!canCapture && (now - lastCaptureTime) > COOLDOWN_TIME_MS) {
    canCapture = true;
  }
  
  // 搜索阶段显示进度 (每2秒更新一次)
  static unsigned long lastSearchUpdate = 0;
  if (gameState == SEARCHING && (now - lastSearchUpdate) > 2000) {
    unsigned long elapsed = now - gameStartTime;
    int remainingSeconds = (SEARCH_TIME_MS - elapsed) / 1000;
    Serial.printf("搜索中... 发现 %d 个设备，剩余 %d 秒\n", peers.size(), max(0, remainingSeconds));
    lastSearchUpdate = now;
  }
  
  // 按键1处理 - 支持长按重置
  // 注意：根据实际硬件，可能需要反转逻辑
  // 如果不按时进度条增加，说明逻辑反了，需要改为 == HIGH
  bool button1Current = digitalRead(BUTTON1_PIN) == HIGH;  // 反转逻辑！
  
  // 调试：检测按钮状态变化
  static bool lastDebugState = false;
  if (button1Current != lastDebugState) {
    Serial.printf("[DEBUG] 按钮1状态变化: %s (digitalRead=%d)\n", 
                  button1Current ? "按下" : "释放", 
                  digitalRead(BUTTON1_PIN));
    lastDebugState = button1Current;
  }
  
  // 检测按键1按下（从false变为true）
  if (button1Current && !button1Pressing) {
    button1Pressing = true;
    button1PressStartTime = now;
    Serial.println("[DEBUG] 开始计时长按");
  }
  // 检测按键1释放（从true变为false）
  else if (!button1Current && button1Pressing) {
    button1Pressing = false;
    unsigned long pressDuration = now - button1PressStartTime;
    Serial.printf("[DEBUG] 按钮释放，持续时长: %.2f秒\n", pressDuration / 1000.0f);
    
    // 根据按压时长和游戏状态处理
    if (gameState == SEARCHING || gameState == PLAYING || gameState == VICTORY) {
      // 在游戏中或胜利状态需要长按3秒才能重置
      if (pressDuration >= LONG_PRESS_TIME) {
        gameState = TEAM_SELECT;
        myTeam = TEAM_NEUTRAL;
        peers.clear();
        Serial.println("长按重置 - 游戏结束，重新选择队伍");
      } else {
        Serial.printf("需要长按3秒才能重置 (当前: %.1f秒)\n", pressDuration / 1000.0f);
      }
    } else {
      // 在其他状态下，短按即可
      if (pressDuration < LONG_PRESS_TIME) {
        switch(gameState) {
          case TEAM_SELECT:
            // 确认选择的队伍，进入等待状态
            gameState = WAITING;
            Serial.printf("已选择队伍: %d (", selectedTeam);
            switch(selectedTeam) {
              case TEAM_RED: Serial.print("红色"); break;
              case TEAM_GREEN: Serial.print("绿色"); break;
              case TEAM_BLUE: Serial.print("蓝色"); break;
              case TEAM_YELLOW: Serial.print("黄色"); break;
              default: Serial.print("未知"); break;
            }
            Serial.println(") - 按按键1开始游戏");
            break;
            
          case WAITING:
            gameState = SEARCHING;
            gameStartTime = now;
            myTeam = assignTeamByMac(selfMac);
            Serial.printf("开始搜索设备... 我的队伍: %d (颜色: ", myTeam);
            switch(myTeam) {
              case TEAM_RED: Serial.print("红色"); break;
              case TEAM_GREEN: Serial.print("绿色"); break;
              case TEAM_BLUE: Serial.print("蓝色"); break;
              case TEAM_YELLOW: Serial.print("黄色"); break;
              default: Serial.print("未知"); break;
            }
            Serial.println(")");
            break;
        }
      }
    }
  }
  
  // 长按过程中显示进度提示
  if (button1Pressing && (gameState == SEARCHING || gameState == PLAYING || gameState == VICTORY)) {
    unsigned long pressDuration = now - button1PressStartTime;
    static unsigned long lastProgressPrint = 0;
    if (pressDuration >= LONG_PRESS_TIME) {
      // 已达到3秒，等待释放
      if (now - lastProgressPrint > 200) {
        Serial.println(">>> 释放按键以重置游戏 <<<");
        lastProgressPrint = now;
      }
    } else {
      // 显示进度
      if (now - lastProgressPrint > 500) {
        Serial.printf("长按重置中... %.1f/3.0秒\n", pressDuration / 1000.0f);
        lastProgressPrint = now;
      }
    }
  }
  
  // 按键2处理
  if (readButton(BUTTON2_PIN, lastButton2, lastButton2Press)) {
    if (gameState == TEAM_SELECT) {
      // 切换队伍选择
      switch(selectedTeam) {
        case TEAM_RED: selectedTeam = TEAM_GREEN; break;
        case TEAM_GREEN: selectedTeam = TEAM_BLUE; break;
        case TEAM_BLUE: selectedTeam = TEAM_YELLOW; break;
        case TEAM_YELLOW: selectedTeam = TEAM_RED; break;
        default: selectedTeam = TEAM_RED; break;
      }
      Serial.printf("切换到队伍: %d (", selectedTeam);
      switch(selectedTeam) {
        case TEAM_RED: Serial.print("红色"); break;
        case TEAM_GREEN: Serial.print("绿色"); break;
        case TEAM_BLUE: Serial.print("蓝色"); break;
        case TEAM_YELLOW: Serial.print("黄色"); break;
        default: Serial.print("未知"); break;
      }
      Serial.println(")");
    } else if (gameState == PLAYING && canCapture) {
      // 抓捕功能 - 只抓捕敌对队伍
      bool foundTarget = false;
      for (auto &kv : peers) {
        if (kv.second.rssi > CAPTURE_DISTANCE && kv.second.team != myTeam) {
          Serial.printf("尝试抓捕敌人 %s (队伍: %d, RSSI: %d)\n", 
                        kv.first.c_str(), kv.second.team, kv.second.rssi);
          sendCaptureCommand(kv.first);
          canCapture = false;
          lastCaptureTime = now;
          foundTarget = true;
          break;
        } else if (kv.second.rssi > CAPTURE_DISTANCE && kv.second.team == myTeam) {
          Serial.printf("跳过队友 %s (队伍: %d, RSSI: %d) - 不抓捕同队\n", 
                        kv.first.c_str(), kv.second.team, kv.second.rssi);
        }
      }
      if (!foundTarget) {
        Serial.println("附近没有可抓捕的敌人");
      }
    }
  }
  
  // 搜索阶段转换为游戏阶段
  if (gameState == SEARCHING && (now - gameStartTime) > SEARCH_TIME_MS) {
    gameState = PLAYING;
    Serial.printf("游戏开始！我的队伍: %d，发现 %d 个设备\n", myTeam, peers.size());
  }
  
  // 游戏阶段检查胜利条件
  if (gameState == PLAYING && checkVictory()) {
    gameState = VICTORY;
    victoryTime = now;
    winningTeam = myTeam;
    Serial.printf("🏆 胜利！队伍 %d 统一了所有玩家！\n", myTeam);
  }
}

// ====== 显示：根据游戏状态显示 ======
void updateMatrix() {
  prunePeers();
  matrix.fillScreen(0);
  
  switch(gameState) {
    case TEAM_SELECT:
      // 队伍选择状态：显示当前选择的队伍颜色
      {
        // 显示选择的队伍颜色 (中心区域)
        matrix.fillRect(3, 3, 2, 2, colorForTeam(selectedTeam));
        
        // 显示选择提示 (四角闪烁)
        static bool blink = false;
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 300) {
          blink = !blink;
          lastBlink = millis();
        }
        if (blink) {
          matrix.drawPixel(1, 1, matrix.Color(255, 255, 255)); // 左下
          matrix.drawPixel(6, 1, matrix.Color(255, 255, 255)); // 右下
          matrix.drawPixel(1, 6, matrix.Color(255, 255, 255)); // 左上
          matrix.drawPixel(6, 6, matrix.Color(255, 255, 255)); // 右上
        }
      }
      break;
      
    case WAITING:
      // 等待状态：闪烁蓝色中心
      {
        static bool blink = false;
        static unsigned long lastBlink = 0;
        if (millis() - lastBlink > 500) {
          blink = !blink;
          lastBlink = millis();
        }
        if (blink) {
          matrix.fillRect(3, 3, 2, 2, matrix.Color(0, 0, 100));
        }
      }
      break;
      
    case SEARCHING:
      // 搜索状态：显示搜索动画和发现的设备
      {
        unsigned long elapsed = millis() - gameStartTime;
        int progress = map(elapsed, 0, SEARCH_TIME_MS, 0, 8);
        
        // 进度条 (顶部)
        for (int i = 0; i < progress; i++) {
          matrix.drawPixel(i, 7, matrix.Color(100, 100, 0));
        }
        
        // 显示我的颜色 (中心)
        matrix.fillRect(3, 3, 2, 2, colorForTeam(myTeam));
        
        // 显示发现的设备数量 (底部用点表示)
        int deviceCount = peers.size();
        for (int i = 0; i < min(8, deviceCount); i++) {
          matrix.drawPixel(i, 0, matrix.Color(0, 255, 0)); // 绿色点表示发现的设备
        }
        
        // 显示发现的设备 (左右两侧，只显示队伍颜色)
        int displayCount = 0;
        for (auto &kv : peers) {
          if (displayCount < 6) { // 最多显示6个其他设备
            int x, y;
            if (displayCount < 3) {
              // 左侧显示前3个
              x = 0;
              y = displayCount + 2;
            } else {
              // 右侧显示后3个
              x = 7;
              y = (displayCount - 3) + 2;
            }
            matrix.drawPixel(x, y, colorForTeam(kv.second.team));
            displayCount++;
          }
        }
        
        // === 长按重置进度条（覆盖在顶部进度条上） ===
        if (button1Pressing && button1PressStartTime > 0) {
          unsigned long pressDuration = millis() - button1PressStartTime;
          int longPressProgress = map(min(pressDuration, LONG_PRESS_TIME), 0, LONG_PRESS_TIME, 0, 8);
          for (int i = 0; i < longPressProgress; i++) {
            matrix.drawPixel(i, 7, matrix.Color(255, 165, 0)); // 橙色进度条覆盖黄色
          }
        }
      }
      break;
      
    case PLAYING:
    {
      // === 中心显示：保持你原有逻辑 ===
      uint16_t myColor = colorForTeam(myTeam);
      if (!canCapture) {
        // 冷却时稍微暗一点
        myColor = matrix.Color(
          ((myColor >> 11) & 0x1F) >> 2,
          ((myColor >> 5)  & 0x3F) >> 2,
          ( myColor        & 0x1F) >> 2
        );
      }
      matrix.fillRect(3, 3, 2, 2, myColor);

      // === 选出最多 4 个“非本队”邻居，按 RSSI 近→远 ===
      std::vector<std::pair<String, PeerInfo>> enemies;
      enemies.reserve(peers.size());
      for (auto &kv : peers) {
        if (kv.second.team != myTeam) enemies.push_back(kv);
      }
      std::sort(enemies.begin(), enemies.end(),
                [](const auto &a, const auto &b){ return a.second.rssi > b.second.rssi; });

      // === 将最近的4个分配到 TOP/RIGHT/BOTTOM/LEFT 四个边 ===
      const uint8_t SIDES[4] = {0,1,2,3};
      int shown = 0;
      for (size_t i = 0; i < enemies.size() && shown < 4; ++i) {
        const auto &mac  = enemies[i].first;
        const auto &peer = enemies[i].second;
        // RSSI→距离估算
        float d = rssiToDistanceMeters(peer.rssi);
        int   L = distanceToEdgeLen(d);
        uint16_t c = colorForTeam(peer.team);
        // 把第 shown 个敌人放到第 shown 条边
        drawEdgeBar(SIDES[shown], L, c);
        // 调试输出
        Serial.printf("[PLAYING] enemy %s team=%d RSSI=%d -> %.2fm, len=%d, side=%d\n",
                      mac.c_str(), (int)peer.team, peer.rssi, d, L, (int)SIDES[shown]);
        shown++;
      }

      // === 可抓捕目标闪烁边框（保持你的提示逻辑） ===
      bool hasTarget = false;
      for (auto &kv : peers) {
        if (kv.second.rssi > CAPTURE_DISTANCE && kv.second.team != myTeam) {
          hasTarget = true; break;
        }
      }
      if (hasTarget && canCapture) {
        static bool captureFlash = false;
        static unsigned long lastFlash = 0;
        if (millis() - lastFlash > 200) { captureFlash = !captureFlash; lastFlash = millis(); }
        if (captureFlash) {
          for (int i = 2; i <= 5; i++) {
            matrix.drawPixel(i, 2, matrix.Color(255, 255, 255));
            matrix.drawPixel(i, 5, matrix.Color(255, 255, 255));
            matrix.drawPixel(2, i, matrix.Color(255, 255, 255));
            matrix.drawPixel(5, i, matrix.Color(255, 255, 255));
          }
        }
      }

      // === 冷却时间条（保留） ===
      if (!canCapture) {
        unsigned long elapsed = millis() - lastCaptureTime;
        int progress = map(elapsed, 0, COOLDOWN_TIME_MS, 0, 5);
        for (int i = 0; i < progress; i++) matrix.drawPixel(i, 0, matrix.Color(255, 0, 0));
      }
      
      // === 长按重置进度条（显示在顶部） ===
      if (button1Pressing && button1PressStartTime > 0) {
        unsigned long pressDuration = millis() - button1PressStartTime;
        int longPressProgress = map(min(pressDuration, LONG_PRESS_TIME), 0, LONG_PRESS_TIME, 0, 8);
        for (int i = 0; i < longPressProgress; i++) {
          matrix.drawPixel(i, 7, matrix.Color(255, 165, 0)); // 橙色进度条
        }
      }
    }
    break;
    
    case VICTORY:
      // 胜利状态：显示胜利动画
      {
        unsigned long elapsed = millis() - victoryTime;
        uint16_t victoryColor = colorForTeam(winningTeam);
        
        // 动画阶段1：全屏闪烁 (0-3秒)
        if (elapsed < 3000) {
          static bool flash = false;
          static unsigned long lastFlash = 0;
          if (millis() - lastFlash > 200) {
            flash = !flash;
            lastFlash = millis();
          }
          if (flash) {
            matrix.fillScreen(victoryColor);
          }
        }
        // 动画阶段2：旋转边框 (3-6秒)
        else if (elapsed < 6000) {
          int phase = ((millis() / 100) % 28); // 28步完成一圈
          // 画边框
          for (int i = 0; i < 8; i++) {
            matrix.drawPixel(i, 0, matrix.Color(50, 50, 50)); // 底部
            matrix.drawPixel(i, 7, matrix.Color(50, 50, 50)); // 顶部
            matrix.drawPixel(0, i, matrix.Color(50, 50, 50)); // 左边
            matrix.drawPixel(7, i, matrix.Color(50, 50, 50)); // 右边
          }
          // 旋转的亮点
          if (phase < 7) { // 顶部从左到右
            matrix.drawPixel(phase, 7, victoryColor);
          } else if (phase < 14) { // 右边从上到下
            matrix.drawPixel(7, 7 - (phase - 7), victoryColor);
          } else if (phase < 21) { // 底部从右到左
            matrix.drawPixel(7 - (phase - 14), 0, victoryColor);
          } else { // 左边从下到上
            matrix.drawPixel(0, (phase - 21), victoryColor);
          }
          // 中心显示队伍颜色
          matrix.fillRect(3, 3, 2, 2, victoryColor);
        }
        // 动画阶段3：烟花效果 (6秒后)
        else {
          static unsigned long lastFirework = 0;
          if (millis() - lastFirework > 500) {
            lastFirework = millis();
            // 随机烟花位置
            int x = random(1, 7);
            int y = random(1, 7);
            matrix.drawPixel(x, y, victoryColor);
            if (x > 0) matrix.drawPixel(x-1, y, victoryColor);
            if (x < 7) matrix.drawPixel(x+1, y, victoryColor);
            if (y > 0) matrix.drawPixel(x, y-1, victoryColor);
            if (y < 7) matrix.drawPixel(x, y+1, victoryColor);
          }
          // 中心大字 "V"
          matrix.fillRect(3, 3, 2, 2, victoryColor);
        }
        
        // === 长按重置进度条（在胜利状态也显示） ===
        if (button1Pressing && button1PressStartTime > 0) {
          unsigned long pressDuration = millis() - button1PressStartTime;
          int longPressProgress = map(min(pressDuration, LONG_PRESS_TIME), 0, LONG_PRESS_TIME, 0, 8);
          for (int i = 0; i < longPressProgress; i++) {
            matrix.drawPixel(i, 0, matrix.Color(255, 165, 0)); // 橙色进度条在底部
          }
        }
      }
      break;
  }

  matrix.show();
}

// ====== 接收回调（支持 2.x/3.x 内核） ======
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data || len < (int)sizeof(Packet)) return;
  const Packet *p = reinterpret_cast<const Packet*>(data);
  if (memcmp(p->magic, MAGIC, 4) != 0) return;

  String macStr = macToString(info->src_addr);
  int rssiNow = 0; if (info->rx_ctrl) rssiNow = info->rx_ctrl->rssi;

  // 处理抓捕命令
  if (p->captureCmd == 1 && gameState == PLAYING) {
    // 收到抓捕命令，检查是否在范围内 AND 不是同队
    if (rssiNow > CAPTURE_DISTANCE && p->team != myTeam) {
      // 被敌对队伍抓捕，改变队伍
      PlayerTeam oldTeam = myTeam;
      myTeam = p->team;
      Serial.printf("被敌队 %s 抓捕！队伍从 %d 变为 %d\n", macStr.c_str(), oldTeam, myTeam);
      
      // 强制冷却
      canCapture = false;
      lastCaptureTime = millis();
    } else if (p->team == myTeam) {
      // 同队队友的抓捕命令，忽略
      Serial.printf("收到队友 %s 的抓捕命令，忽略（同队不会被抓）\n", macStr.c_str());
    } else {
      // 在范围外
      Serial.printf("收到 %s 的抓捕命令，但不在范围内 (RSSI: %d)\n", macStr.c_str(), rssiNow);
    }
  }

  auto it = peers.find(macStr);
  if (it == peers.end()) {
    peers[macStr] = { rssiNow, millis(), p->team, p->state, false };
    // 调试：新发现设备
    if (gameState == SEARCHING) {
      Serial.printf("发现新设备: %s, 队伍: %d, RSSI: %d\n", 
                    macStr.c_str(), p->team, rssiNow);
    }
  } else {
    int old = it->second.rssi;
    it->second.rssi = (int)(RSSI_ALPHA * old + (1.0f - RSSI_ALPHA) * rssiNow);
    it->second.lastSeen = millis();
    it->second.team = p->team;
    it->second.state = p->state;
  }
}

// ====== 发心跳 ======
void sendPing() {
  Packet pkt; 
  memcpy(pkt.magic, MAGIC, 4); 
  memcpy(pkt.mac, selfMac, 6); 
  pkt.seq = ++seqNo;
  pkt.team = myTeam;
  pkt.state = gameState;
  pkt.captureCmd = 0; // 普通心跳包
  esp_now_send(BCAST, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ====== 发送抓捕命令 ======
void sendCaptureCommand(const String& targetMac) {
  Packet pkt; 
  memcpy(pkt.magic, MAGIC, 4); 
  memcpy(pkt.mac, selfMac, 6); 
  pkt.seq = ++seqNo;
  pkt.team = myTeam;
  pkt.state = gameState;
  pkt.captureCmd = 1; // 抓捕命令
  esp_now_send(BCAST, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}
// === 距离估算与边绘制（新增） ===

// 经验模型：1米参考发射功率 txPower≈-59dBm，室内路径损耗指数 n≈2.0
float rssiToDistanceMeters(int rssi, float txPower, float n) {
  rssi = constrain(rssi, -100, -30);
  float ratio = (txPower - (float)rssi) / (10.0f * n);
  // 避免数值过大：限定 0.1m ~ 20m
  float d = powf(10.0f, ratio);
  return constrain(d, 0.1f, 20.0f);
}

// 距离(米)映射到边条长度(1~7，越近越长)
int distanceToEdgeLen(float meters) {
  // 0.3m → 7格； 8m → 1格（可按实际场地调）
  float m = constrain(meters, 0.3f, 8.0f);
  int len = map((int)(m * 100), (int)(0.3f * 100), (int)(8.0f * 100), 7, 1);
  return constrain(len, 1, 7);
}

// 在四条边绘制长度为 len 的条（方向：0=TOP,1=RIGHT,2=BOTTOM,3=LEFT）
void drawEdgeBar(uint8_t side, int len, uint16_t color) {
  len = constrain(len, 1, 7);
  switch (side) {
    case 0: { // TOP (y=7，从x=0向右)
      for (int x = 0; x < len; ++x) matrix.drawPixel(x, 7, color);
    } break;
    case 1: { // RIGHT (x=7，从y=7向下)
      for (int y = 7; y > 7 - len; --y) matrix.drawPixel(7, y, color);
    } break;
    case 2: { // BOTTOM (y=0，从x=7向左)
      for (int x = 7; x > 7 - len; --x) matrix.drawPixel(x, 0, color);
    } break;
    case 3: { // LEFT (x=0，从y=0向上)
      for (int y = 0; y < len; ++y) matrix.drawPixel(0, y, color);
    } break;
  }
}

// ====== 初始化 ======
void initWiFiEspNow() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true, true);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) while (true) delay(1000);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, BCAST, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.ifidx = WIFI_IF_STA;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);
}

// ====== 入口 ======
void setup() {
  Serial.begin(115200);
  delay(200);

  // 初始化按键
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  
  // 读取初始按钮状态，确保 button1Pressing 正确初始化
  delay(50); // 等待引脚稳定
  int rawButton1 = digitalRead(BUTTON1_PIN);
  bool initialButton1State = (rawButton1 == HIGH);  // 反转逻辑！
  
  Serial.printf("按钮1初始状态: digitalRead=%d, 判定为%s\n", 
                rawButton1, 
                initialButton1State ? "按下" : "未按下");
  
  button1Pressing = false; // 启动时强制设为未按下状态
  button1PressStartTime = 0; // 清零计时器
  
  if (initialButton1State) {
    Serial.println("警告: 启动时检测到按钮1被按下，请释放按钮");
  } else {
    Serial.println("按钮状态正常");
  }

  matrix.begin();
  matrix.setBrightness(40);           // 亮度适中；可调 20~60
  matrix.fillScreen(0);
  
  // 开机自检：显示启动动画
  for (int i = 0; i < 8; i++) {
    matrix.fillScreen(0);
    matrix.drawPixel(i, i, matrix.Color(32, 32, 32));
    if (i > 0) matrix.drawPixel(i-1, i-1, matrix.Color(16, 16, 16));
    matrix.show();
    delay(100);
  }
  matrix.fillScreen(0);
  matrix.show();

  WiFi.macAddress(selfMac);
  initWiFiEspNow();
  
  Serial.println("ESP32-S3 抓捕游戏已启动");
  Serial.printf("MAC地址: %s\n", macToString(selfMac).c_str());
  Serial.println("=== 队伍选择模式 ===");
  Serial.println("按键2: 切换队伍颜色 (红->绿->蓝->黄->红...)");
  Serial.println("按键1: 确认选择的队伍");
  Serial.printf("当前选择: %d (", selectedTeam);
  switch(selectedTeam) {
    case TEAM_RED: Serial.print("红色"); break;
    case TEAM_GREEN: Serial.print("绿色"); break;
    case TEAM_BLUE: Serial.print("蓝色"); break;
    case TEAM_YELLOW: Serial.print("黄色"); break;
    default: Serial.print("未知"); break;
  }
  Serial.println(")");
}


void loop() {
  unsigned long now = millis();
  
  // 调试：每2秒打印一次按钮状态
  static unsigned long lastDebugPrint = 0;
  if (now - lastDebugPrint > 2000) {
    int rawBtn = digitalRead(BUTTON1_PIN);
    Serial.printf("[MONITOR] 按钮1: digitalRead=%d, button1Pressing=%s\n", 
                  rawBtn, 
                  button1Pressing ? "true" : "false");
    lastDebugPrint = now;
  }
  
  // 处理游戏逻辑和按键
  handleGameLogic();
  
  // 发送心跳
  if (now - lastPing >= PING_INTERVAL) { 
    sendPing(); 
    lastPing = now; 
  }
  
  // 更新显示
  updateMatrix();
  
  delay(50); // 更快的响应时间
}