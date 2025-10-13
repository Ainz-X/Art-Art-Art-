/*
 * ESP32-S3 Matrix - Capture Game + Distance Bars (Merged)
 * This file merges the "neighbor distance bars" visualization from sprint 1
 * into the existing test.cpp capture game code. A new helper
 * drawDistanceBars() is added and invoked in SEARCHING and PLAYING states.
 *
 * How it shows distance:
 * - Up to 8 strongest peers are shown as columns (x=0..7).
 * - Column height 0..8 maps from RSSI -100..-30 (sprint1 logic).
 * - Column color uses red->green gradient by RSSI (sprint1 logic).
 * - Bars are drawn first; game overlays (center square, progress, etc.) are drawn on top.
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
#define BUTTON1_PIN 3     // GPIO3 (开始/结束游戏)
#define BUTTON2_PIN 7     // GPIO7 (抓捕)
#define MATRIX_PIN 14     // LED矩阵 GPIO14

// ====== 矩阵配置（与你现有板卡保持一致） ======
#define MW 8
#define MH 8
#define COLOR_ORDER NEO_RGB
Adafruit_NeoMatrix matrix(
  MW, MH, MATRIX_PIN,
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
static const int      CAPTURE_DISTANCE = -60;  // RSSI阈值
static const uint32_t COOLDOWN_TIME_MS = 5000; // 冷却5秒
static const uint32_t SEARCH_TIME_MS   = 8000; // 搜索8秒

// ====== 游戏状态与队伍 ======
enum GameState { TEAM_SELECT, WAITING, SEARCHING, PLAYING };
enum PlayerTeam {
  TEAM_NEUTRAL = 0, TEAM_RED=1, TEAM_GREEN=2, TEAM_BLUE=3,
  TEAM_YELLOW=4, TEAM_PURPLE=5, TEAM_CYAN=6, TEAM_WHITE=7
};

// ====== 封包/同伴状态 ======
struct Packet {
  char magic[4];
  uint8_t mac[6];
  uint32_t seq;
  PlayerTeam team;
  GameState state;
  uint8_t captureCmd; // 0=心跳,1=抓捕
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

// ====== 本机状态 ======
GameState gameState = TEAM_SELECT;
PlayerTeam selectedTeam = TEAM_RED;
PlayerTeam myTeam = TEAM_NEUTRAL;
unsigned long gameStartTime = 0;
unsigned long lastCaptureTime = 0;
bool canCapture = true;

// ====== 按键去抖 ======
bool lastButton1 = false, lastButton2 = false;
unsigned long lastButton1Press = 0, lastButton2Press = 0;
const unsigned long DEBOUNCE_TIME = 50;
// ====== 双键长按复位（PLAYING 状态） ======
unsigned long bothPressStart = 0;
const unsigned long DUAL_LONGPRESS_MS = 5000; // 5 秒


// ====== 工具函数 ======
String macToString(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  return String(buf);
}
inline int clampRSSI(int r){ return constrain(r, -100, -30); }

uint16_t colorForRSSI(int rssi) {
  int v = clampRSSI(rssi);
  uint8_t r = map(v, -100, -30, 255, 0);
  uint8_t g = map(v, -100, -30, 0, 255);
  return matrix.Color(r, g, 0);
}

uint16_t colorForTeam(PlayerTeam team) {
  switch(team) {
    case TEAM_RED:    return matrix.Color(255, 0, 0);
    case TEAM_GREEN:  return matrix.Color(0, 255, 0);
    case TEAM_BLUE:   return matrix.Color(0, 0, 255);
    case TEAM_YELLOW: return matrix.Color(255, 255, 0);
    case TEAM_PURPLE: return matrix.Color(255, 0, 255);
    case TEAM_CYAN:   return matrix.Color(0, 255, 255);
    case TEAM_WHITE:  return matrix.Color(255, 255, 255);
    default:          return matrix.Color(64, 64, 64);
  }
}

struct RGB { uint8_t r,g,b; };

RGB rgbForTeam(PlayerTeam team) {
  switch(team) {
    case TEAM_RED:    return {255, 0, 0};
    case TEAM_GREEN:  return {0, 255, 0};
    case TEAM_BLUE:   return {0, 0, 255};
    case TEAM_YELLOW: return {255, 255, 0};
    case TEAM_PURPLE: return {255, 0, 255};
    case TEAM_CYAN:   return {0, 255, 255};
    case TEAM_WHITE:  return {255, 255, 255};
    default:          return {64, 64, 64};
  }
}

uint16_t scaledColor(RGB base, int scale /*0..255*/) {
  if (scale < 0) scale = 0;
  if (scale > 255) scale = 255;
  uint8_t r = (uint16_t)base.r * scale / 255;
  uint8_t g = (uint16_t)base.g * scale / 255;
  uint8_t b = (uint16_t)base.b * scale / 255;
  return matrix.Color(r,g,b);
}

PlayerTeam assignTeamByMac(const uint8_t mac[6]) { return selectedTeam; }

bool readButton(int pin, bool &lastState, unsigned long &lastPress) {
  bool current = digitalRead(pin) == LOW; // 按下为LOW
  unsigned long now = millis();
  if (current != lastState && (now - lastPress) > DEBOUNCE_TIME) {
    lastPress = now;
    lastState = current;
    return current;
  }
  return false;
}

void prunePeers() {
  unsigned long now = millis();
  std::vector<String> dead;
  for (auto &kv : peers) if (now - kv.second.lastSeen > PEER_EXPIRE_MS) dead.push_back(kv.first);
  for (auto &k : dead) peers.erase(k);
}

// 在 PLAYING 状态时检测两个按键是否同时长按 5 秒以重置游戏
void checkDualLongPressReset() {
  if (gameState != PLAYING) { bothPressStart = 0; return; }

  bool b1 = (digitalRead(BUTTON1_PIN) == LOW);
  bool b2 = (digitalRead(BUTTON2_PIN) == LOW);
  unsigned long now = millis();

  if (b1 && b2) {
    if (bothPressStart == 0) bothPressStart = now;
    else if (now - bothPressStart >= DUAL_LONGPRESS_MS) {
      // 执行 reset：回到 TEAM_SELECT，清空队伍与邻居
      gameState = TEAM_SELECT;
      myTeam = TEAM_NEUTRAL;
      selectedTeam = TEAM_RED;   // 可按需设默认队伍
      peers.clear();
      canCapture = true;
      lastCaptureTime = 0;
      gameStartTime = 0;
      bothPressStart = 0;

      // 可选：给个小动画提示
      matrix.fillScreen(0);
      for (int i=0;i<4;i++) {
        matrix.drawRect(i, i, 8-2*i, 8-2*i, matrix.Color(64,64,64));
        matrix.show(); delay(60);
      }
      matrix.fillScreen(0);
      matrix.show();
    }
  } else {
    // 任一松开，计时清零
    bothPressStart = 0;
  }
}


// ====== 来自 sprint1 的“相邻芯片距离条形图” ======

// ====== 四周边缘显示“其他颜色队伍”的距离（仅在 PLAYING 时调用） ======
enum Side { TOP=0, RIGHT=1, BOTTOM=2, LEFT=3 };

void drawEdgeBar(Side side, int slot /*0..7*/, int length /*1..4*/, uint16_t color) {
  length = constrain(length, 1, 4);
  slot = constrain(slot, 0, 7);
  switch (side) {
    case TOP:    // y=7 向下延伸
      for (int d=0; d<length; ++d) matrix.drawPixel(slot, 7 - d, color);
      break;
    case BOTTOM: // y=0 向上延伸
      for (int d=0; d<length; ++d) matrix.drawPixel(slot, 0 + d, color);
      break;
    case LEFT:   // x=0 向右延伸
      for (int d=0; d<length; ++d) matrix.drawPixel(0 + d, slot, color);
      break;
    case RIGHT:  // x=7 向左延伸
      for (int d=0; d<length; ++d) matrix.drawPixel(7 - d, slot, color);
      break;
  }
}

/*
 * 根据队伍把“非我方”设备分到四个边：TOP/RIGHT/BOTTOM/LEFT。
 * 映射规则（固定顺序，遇到我方则跳过，最多四队）：
 *   顺序：RED, GREEN, BLUE, YELLOW, PURPLE, CYAN, WHITE
 *   边：  TOP, RIGHT, BOTTOM, LEFT （依次分配）
 * 每个边按 RSSI 由强到弱取前 8 台；条长度按 RSSI ∈ [-100,-30] → [1,4]；
 * 颜色用该队伍的基色，并按 RSSI 调整亮度。
 */
void drawOtherTeamsOnEdges(PlayerTeam myTeam) {
  prunePeers();

  // 收集“非我方”按队伍分组
  std::map<PlayerTeam, std::vector<PeerInfo>> byTeam;
  for (auto &kv : peers) {
    const PeerInfo &p = kv.second;
    if (p.team == myTeam) continue;
    byTeam[p.team].push_back(p);
  }

  // 队伍选择顺序与边映射
  std::vector<PlayerTeam> order = {
    TEAM_RED, TEAM_GREEN, TEAM_BLUE, TEAM_YELLOW, TEAM_PURPLE, TEAM_CYAN, TEAM_WHITE
  };
  std::vector<Side> sides = { TOP, RIGHT, BOTTOM, LEFT };

  int sideIdx = 0;
  for (auto t : order) {
    if (sideIdx >= (int)sides.size()) break;
    auto it = byTeam.find(t);
    if (it == byTeam.end()) continue;

    Side side = sides[sideIdx++];
    auto &vec = it->second;
    std::sort(vec.begin(), vec.end(), [](const PeerInfo& a, const PeerInfo& b){
      return a.rssi > b.rssi;
    });

    // 渲染该队伍到对应边
    int count = std::min(8, (int)vec.size());
    RGB base = rgbForTeam(t);
    for (int i = 0; i < count; ++i) {
      int v = clampRSSI(vec[i].rssi);
      int len = map(v, -100, -30, 1, 4);      // 条向内延伸长度
      int bright = map(v, -100, -30, 100, 255); // 亮度
      uint16_t col = scaledColor(base, bright);
      drawEdgeBar(side, i, len, col);
    }
  }
}
void drawDistanceBars() {
  prunePeers();

  std::vector<std::pair<String, PeerInfo>> list;
  list.reserve(peers.size());
  for (auto &kv : peers) list.push_back(kv);
  std::sort(list.begin(), list.end(),
            [](const auto &a, const auto &b){ return a.second.rssi > b.second.rssi; });

  int cols = std::min(8, (int)list.size());
  for (int i = 0; i < cols; ++i) {
    int rssi = list[i].second.rssi;
    int h = map(clampRSSI(rssi), -100, -30, 0, 8);   // 0~8格
    uint16_t c = colorForRSSI(rssi);
    for (int y = 0; y < h; ++y) matrix.drawPixel(i, y, c);  // 底部为 y=0
  }
}

// ====== 游戏逻辑 ======
void sendCaptureCommand(const String& targetMac); // 前置声明

void handleGameLogic() {
  unsigned long now = millis();
  if (!canCapture && (now - lastCaptureTime) > COOLDOWN_TIME_MS) canCapture = true;

  // 按键1：状态流转
  if (readButton(BUTTON1_PIN, lastButton1, lastButton1Press)) {
    switch(gameState) {
      case TEAM_SELECT:
        gameState = WAITING;
        break;
      case WAITING:
        gameState = SEARCHING;
        gameStartTime = now;
        myTeam = assignTeamByMac(selfMac);
        break;
      case SEARCHING:
      case PLAYING:
        gameState = TEAM_SELECT;
        myTeam = TEAM_NEUTRAL;
        peers.clear();
        break;
    
  // 检测双键 5 秒长按以执行 reset（仅 PLAYING 有效）
  checkDualLongPressReset();
}
  }

  // 按键2：切队或抓捕
  if (readButton(BUTTON2_PIN, lastButton2, lastButton2Press)) {
    if (gameState == TEAM_SELECT) {
      switch(selectedTeam) {
        case TEAM_RED: selectedTeam = TEAM_GREEN; break;
        case TEAM_GREEN: selectedTeam = TEAM_BLUE; break;
        case TEAM_BLUE: selectedTeam = TEAM_YELLOW; break;
        default: selectedTeam = TEAM_RED; break;
      }
    } else if (gameState == PLAYING && canCapture) {
      for (auto &kv : peers) {
        if (kv.second.rssi > CAPTURE_DISTANCE && kv.second.team != myTeam) {
          sendCaptureCommand(kv.first);
          canCapture = false;
          lastCaptureTime = now;
          break;
        }
      }
    }
  }

  // 搜索阶段计时结束 -> PLAYING
  if (gameState == SEARCHING && (now - gameStartTime) > SEARCH_TIME_MS) {
    gameState = PLAYING;
  }
}

// ====== 显示 ======
void updateMatrix() {
  matrix.fillScreen(0);

  // 先画“距离条形图”，再叠加各状态UI
  if (gameState == SEARCHING || gameState == PLAYING) {
    drawDistanceBars();
  }

  switch(gameState) {
    case TEAM_SELECT: {
      matrix.fillRect(3, 3, 2, 2, colorForTeam(selectedTeam));
      static bool blink = false; static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 300) { blink = !blink; lastBlink = millis(); }
      if (blink) {
        matrix.drawPixel(1, 1, matrix.Color(255, 255, 255));
        matrix.drawPixel(6, 1, matrix.Color(255, 255, 255));
        matrix.drawPixel(1, 6, matrix.Color(255, 255, 255));
        matrix.drawPixel(6, 6, matrix.Color(255, 255, 255));
      }
    } break;

    case WAITING: {
      static bool blink = false; static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 500) { blink = !blink; lastBlink = millis(); }
      if (blink) matrix.fillRect(3, 3, 2, 2, matrix.Color(0, 0, 100));
    } break;

    case SEARCHING: {
      unsigned long elapsed = millis() - gameStartTime;
      int progress = map(elapsed, 0, SEARCH_TIME_MS, 0, 8);
      for (int i = 0; i < progress; i++) matrix.drawPixel(i, 7, matrix.Color(100, 100, 0));
      matrix.fillRect(3, 3, 2, 2, colorForTeam(myTeam));
      int deviceCount = peers.size();
      for (int i = 0; i < min(8, deviceCount); i++) matrix.drawPixel(i, 0, matrix.Color(0, 255, 0));
    } break;

    case PLAYING: {
      uint16_t myColor = colorForTeam(myTeam);
      if (!canCapture) {
        // 冷却中降亮度
        myColor = matrix.Color( (uint8_t)( ( (myColor>>11) & 0x1F) << 3 ) >> 2,
                                (uint8_t)( ( (myColor>>5)  & 0x3F) << 2 ) >> 2,
                                (uint8_t)( (  myColor      & 0x1F) << 3 ) >> 2 );
      }
      matrix.fillRect(3, 3, 2, 2, myColor);

      // 在四周显示其他队伍的距离条
      drawOtherTeamsOnEdges(myTeam);

      bool hasTarget = false;
      for (auto &kv : peers) if (kv.second.rssi > CAPTURE_DISTANCE && kv.second.team != myTeam) { hasTarget = true; break; }
      if (hasTarget && canCapture) {
        static bool captureFlash = false; static unsigned long lastFlash = 0;
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
      if (!canCapture) {
        unsigned long elapsed = millis() - lastCaptureTime;
        int progress = map(elapsed, 0, COOLDOWN_TIME_MS, 0, 8);
        for (int i = 0; i < progress; i++) matrix.drawPixel(i, 0, matrix.Color(255, 0, 0));
      }
    } break;
  }
  matrix.show();
}

// ====== ESP-NOW 回调/发送 ======
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data || len < (int)sizeof(Packet)) return;
  const Packet *p = reinterpret_cast<const Packet*>(data);
  if (memcmp(p->magic, MAGIC, 4) != 0) return;

  String macStr = macToString(info->src_addr);
  int rssiNow = 0; if (info->rx_ctrl) rssiNow = info->rx_ctrl->rssi;

  if (p->captureCmd == 1 && gameState == PLAYING) {
    if (rssiNow > CAPTURE_DISTANCE) {
      myTeam = p->team; // 被抓，换队
      canCapture = false;
      lastCaptureTime = millis();
    }
  }

  auto it = peers.find(macStr);
  if (it == peers.end()) {
    peers[macStr] = { rssiNow, millis(), p->team, p->state, false };
  } else {
    int old = it->second.rssi;
    it->second.rssi = (int)(RSSI_ALPHA * old + (1.0f - RSSI_ALPHA) * rssiNow);
    it->second.lastSeen = millis();
    it->second.team = p->team;
    it->second.state = p->state;
  }
}

void sendPing() {
  Packet pkt;
  memcpy(pkt.magic, MAGIC, 4);
  memcpy(pkt.mac, selfMac, 6);
  pkt.seq = ++seqNo;
  pkt.team = myTeam;
  pkt.state = gameState;
  pkt.captureCmd = 0;
  esp_now_send(BCAST, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void sendCaptureCommand(const String& targetMac) {
  Packet pkt;
  memcpy(pkt.magic, MAGIC, 4);
  memcpy(pkt.mac, selfMac, 6);
  pkt.seq = ++seqNo;
  pkt.team = myTeam;
  pkt.state = gameState;
  pkt.captureCmd = 1;
  esp_now_send(BCAST, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

// ====== 初始化/主循环 ======
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

void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);

  matrix.begin();
  matrix.setBrightness(40);
  matrix.fillScreen(0);
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

  Serial.println("ESP32-S3 抓捕游戏 + 距离条形图 已启动");
}

void loop() {
  unsigned long now = millis();

  handleGameLogic();

  if (now - lastPing >= PING_INTERVAL) { sendPing(); lastPing = now; }

  updateMatrix();

  delay(50);
}
