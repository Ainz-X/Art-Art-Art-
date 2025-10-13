#pragma once
#include <Adafruit_NeoMatrix.h>
#include <map>
#include <Arduino.h>
#include "config.h"

extern Adafruit_NeoMatrix matrix;

struct PeerInfo {
  int rssi;
  unsigned long lastSeen;
  PlayerTeam team;
  GameState state;
  bool justCaptured;
};

extern std::map<String, PeerInfo> peers;

// 自身信息/序列号/心跳
extern uint8_t selfMac[6];
extern uint32_t seqNo;
extern unsigned long lastPing;

// 选择与当前状态
extern PlayerTeam selectedTeam;
extern GameState gameState;
extern PlayerTeam myTeam;
extern unsigned long gameStartTime;
extern unsigned long lastCaptureTime;
extern bool canCapture;

// 防守机制
extern bool isBeingCaptured;
extern unsigned long captureStartTime;
extern const unsigned long CAPTURE_WINDOW;
extern String capturingPlayer;
extern PlayerTeam capturingTeam;
extern bool canDefend;
extern unsigned long lastDefendTime;
extern const unsigned long DEFEND_COOLDOWN;

// 胜利状态
extern unsigned long victoryTime;
extern PlayerTeam winningTeam;

// 按键状态
extern bool lastButton1;
extern bool lastButton2;
extern unsigned long lastButton1Press;
extern unsigned long lastButton2Press;
extern const unsigned long DEBOUNCE_TIME;

// 长按
extern unsigned long button1PressStartTime;
extern bool button1Pressing;
extern const unsigned long LONG_PRESS_TIME;
