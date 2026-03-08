#pragma once
#include <Arduino.h>
#include "config.h"
#include "globals.h"

String macToString(const uint8_t mac[6]);
inline int clampRSSI(int r){ return constrain(r, -100, -30); }

uint16_t colorForTeam(PlayerTeam team);
uint16_t colorForRSSI(int rssi);
PlayerTeam assignTeamByMac(const uint8_t mac[6]);

bool readButton(int pin, bool &lastState, unsigned long &lastPress);
void prunePeers();
bool checkVictory();

// 距离估算 & 边绘制
float rssiToDistanceMeters(int rssi, float txPower = -59.0f, float n = 2.0f);
int distanceToEdgeLen(float meters);
void drawEdgeBar(uint8_t side, int len, uint16_t color);
