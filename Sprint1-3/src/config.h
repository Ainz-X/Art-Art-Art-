#pragma once
#include <Adafruit_NeoMatrix.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

// ====== 引脚配置 ======
#define BUTTON1_PIN 3
#define BUTTON2_PIN 7
#define MATRIX_PIN  14

// ====== 矩阵配置 ======
#define MW 8
#define MH 8
#define COLOR_ORDER NEO_RGB

// ====== WiFi/ESP-NOW 参数 ======
static const char     MAGIC[4]       = {'R','D','R','1'};
static const uint8_t  BCAST[6]       = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};
static const uint8_t  WIFI_CHANNEL   = 1;
static const uint32_t PING_INTERVAL  = 1000;
static const uint32_t PEER_EXPIRE_MS = 10000;
static const float    RSSI_ALPHA     = 0.7f;

// ====== 游戏参数 ======
static const int      CAPTURE_DISTANCE = -60;
static const uint32_t COOLDOWN_TIME_MS = 5000;
static const uint32_t SEARCH_TIME_MS   = 8000;

// ====== 游戏状态 / 队伍 ======
enum GameState {
  TEAM_SELECT, WAITING, SEARCHING, PLAYING, VICTORY
};

enum PlayerTeam {
  TEAM_NEUTRAL = 0,
  TEAM_RED = 1, TEAM_GREEN = 2, TEAM_BLUE = 3,
  TEAM_YELLOW = 4, TEAM_PURPLE = 5, TEAM_CYAN = 6, TEAM_WHITE = 7
};

// ====== 数据包 ======
struct Packet {
  char magic[4];
  uint8_t mac[6];
  uint32_t seq;
  PlayerTeam team;
  GameState state;
  uint8_t captureCmd; // 0=心跳 1=抓捕
} __attribute__((packed));
