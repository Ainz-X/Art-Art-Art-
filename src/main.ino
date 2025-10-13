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
  NEO_MATRIX_BOTTOM + NEO_MATRIX_LEFT + NEO_MATRIX_ROWS + NEO_MATRIX_ZIGZAG,
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
  PLAYING       // 游戏进行中
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

// ====== 按键状态 ======
bool lastButton1 = false;
bool lastButton2 = false;
unsigned long lastButton1Press = 0;
unsigned long lastButton2Press = 0;
const unsigned long DEBOUNCE_TIME = 50;

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
  
  // 按键处理
  if (readButton(BUTTON1_PIN, lastButton1, lastButton1Press)) {
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
        
      case SEARCHING:
      case PLAYING:
        gameState = TEAM_SELECT;
        myTeam = TEAM_NEUTRAL;
        peers.clear();
        Serial.println("游戏结束 - 重新选择队伍");
        break;
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
      // 抓捕功能
      for (auto &kv : peers) {
        if (kv.second.rssi > CAPTURE_DISTANCE && kv.second.team != myTeam) {
          Serial.printf("尝试抓捕 %s (RSSI: %d)\n", kv.first.c_str(), kv.second.rssi);
          sendCaptureCommand(kv.first);
          canCapture = false;
          lastCaptureTime = now;
          break;
        }
      }
    }
  }
  
  // 搜索阶段转换为游戏阶段
  if (gameState == SEARCHING && (now - gameStartTime) > SEARCH_TIME_MS) {
    gameState = PLAYING;
    Serial.printf("游戏开始！我的队伍: %d，发现 %d 个设备\n", myTeam, peers.size());
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
        int progress = map(elapsed, 0, COOLDOWN_TIME_MS, 0, 8);
        for (int i = 0; i < progress; i++) matrix.drawPixel(i, 0, matrix.Color(255, 0, 0));
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
    // 收到抓捕命令，检查是否在范围内
    if (rssiNow > CAPTURE_DISTANCE) {
      // 被抓捕，改变队伍
      PlayerTeam oldTeam = myTeam;
      myTeam = p->team;
      Serial.printf("被 %s 抓捕！队伍从 %d 变为 %d\n", macStr.c_str(), oldTeam, myTeam);
      
      // 强制冷却
      canCapture = false;
      lastCaptureTime = millis();
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
float rssiToDistanceMeters(int rssi, float txPower = -59.0f, float n = 2.0f) {
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