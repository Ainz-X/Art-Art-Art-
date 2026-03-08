#include "utils.h"
#include <algorithm>
#include <math.h>

String macToString(const uint8_t mac[6]) {
  char buf[18];
  snprintf(buf, sizeof(buf), "%02X:%02X:%02X:%02X:%02X:%02X",
           mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
  return String(buf);
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

uint16_t colorForRSSI(int rssi) {
  int v = clampRSSI(rssi);
  uint8_t r = map(v, -100, -30, 255, 0);
  uint8_t g = map(v, -100, -30, 0, 255);
  return matrix.Color(r, g, 0);
}

PlayerTeam assignTeamByMac(const uint8_t mac[6]) {
  // 当前为手动选择
  return selectedTeam;
}

bool readButton(int pin, bool &lastState, unsigned long &lastPress) {
  bool current = digitalRead(pin) == LOW; // 假设LOW为按下
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

bool checkVictory() {
  if (peers.size() < 1) return false;
  for (auto &kv : peers) if (kv.second.team != myTeam) return false;
  return true;
}

float rssiToDistanceMeters(int rssi, float txPower, float n) {
  rssi = constrain(rssi, -100, -30);
  float ratio = (txPower - (float)rssi) / (10.0f * n);
  float d = powf(10.0f, ratio);
  return constrain(d, 0.1f, 20.0f);
}

int distanceToEdgeLen(float meters) {
  float m = constrain(meters, 0.3f, 8.0f);
  int len = map((int)(m * 100), (int)(0.3f * 100), (int)(8.0f * 100), 7, 1);
  return constrain(len, 1, 7);
}

void drawEdgeBar(uint8_t side, int len, uint16_t color) {
  len = constrain(len, 1, 7);
  switch (side) {
    case 0: for (int x = 0; x < len; ++x) matrix.drawPixel(x, 7, color); break;      // TOP
    case 1: for (int y = 7; y > 7 - len; --y) matrix.drawPixel(7, y, color); break;  // RIGHT
    case 2: for (int x = 7; x > 7 - len; --x) matrix.drawPixel(x, 0, color); break;  // BOTTOM
    case 3: for (int y = 0; y < len; ++y) matrix.drawPixel(0, y, color); break;      // LEFT
  }
}
