#include "display.h"
#include "globals.h"
#include "utils.h"
#include <algorithm>

void updateMatrix() {
  prunePeers();
  matrix.fillScreen(0);

  switch (gameState) {
    case TEAM_SELECT: {
      matrix.fillRect(3, 3, 2, 2, colorForTeam(selectedTeam));
      static bool blink = false; static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 300) { blink = !blink; lastBlink = millis(); }
      if (blink) {
        matrix.drawPixel(1, 1, matrix.Color(255,255,255));
        matrix.drawPixel(6, 1, matrix.Color(255,255,255));
        matrix.drawPixel(1, 6, matrix.Color(255,255,255));
        matrix.drawPixel(6, 6, matrix.Color(255,255,255));
      }
    } break;

    case WAITING: {
      static bool blink = false; static unsigned long lastBlink = 0;
      if (millis() - lastBlink > 500) { blink = !blink; lastBlink = millis(); }
      if (blink) matrix.fillRect(3, 3, 2, 2, matrix.Color(0,0,100));
    } break;

    case SEARCHING: {
      unsigned long elapsed = millis() - gameStartTime;
      int progress = map(elapsed, 0, SEARCH_TIME_MS, 0, 8);
      for (int i = 0; i < progress; i++) matrix.drawPixel(i, 7, matrix.Color(100,100,0));

      matrix.fillRect(3, 3, 2, 2, colorForTeam(myTeam));

      int deviceCount = peers.size();
      for (int i = 0; i < min(8, deviceCount); i++)
        matrix.drawPixel(i, 0, matrix.Color(0,255,0));

      int displayCount = 0;
      for (auto &kv : peers) {
        if (displayCount < 6) {
          int x,y;
          if (displayCount < 3) { x = 0; y = displayCount + 2; }
          else { x = 7; y = (displayCount - 3) + 2; }
          matrix.drawPixel(x, y, colorForTeam(kv.second.team));
          displayCount++;
        }
      }

      if (button1Pressing && button1PressStartTime > 0) {
        unsigned long pressDuration = millis() - button1PressStartTime;
        int longPressProgress = map(min(pressDuration, LONG_PRESS_TIME), 0, LONG_PRESS_TIME, 0, 8);
        for (int i = 0; i < longPressProgress; i++)
          matrix.drawPixel(i, 7, matrix.Color(255,165,0));
      }
    } break;

    case PLAYING: {
      uint16_t myColor = colorForTeam(myTeam);
      if (!canCapture) {
        myColor = matrix.Color( (uint8_t)( (myColor>>16) & 0xFF )>>2,
                                (uint8_t)( (myColor>> 8) & 0xFF )>>2,
                                (uint8_t)(  myColor      & 0xFF )>>2 );
      }
      matrix.fillRect(3, 3, 2, 2, myColor);

      std::vector<std::pair<String, PeerInfo>> enemies;
      for (auto &kv : peers) if (kv.second.team != myTeam) enemies.push_back(kv);
      std::sort(enemies.begin(), enemies.end(),
        [](const auto &a, const auto &b){ return a.second.rssi > b.second.rssi; });

      const uint8_t SIDES[4] = {0,1,2,3};
      int shown = 0;
      for (size_t i = 0; i < enemies.size() && shown < 4; ++i) {
        const auto &mac  = enemies[i].first;
        const auto &peer = enemies[i].second;
        float d = rssiToDistanceMeters(peer.rssi);
        int   L = distanceToEdgeLen(d);
        uint16_t c = colorForTeam(peer.team);
        drawEdgeBar(SIDES[shown], L, c);
        Serial.printf("[PLAYING] enemy %s team=%d RSSI=%d -> %.2fm, len=%d, side=%d\n",
                      mac.c_str(), (int)peer.team, peer.rssi, d, L, (int)SIDES[shown]);
        shown++;
      }

      bool hasTarget = false;
      for (auto &kv : peers)
        if (kv.second.rssi > CAPTURE_DISTANCE && kv.second.team != myTeam) { hasTarget = true; break; }

      if (hasTarget && canCapture) {
        static bool captureFlash = false; static unsigned long lastFlash = 0;
        if (millis() - lastFlash > 200) { captureFlash = !captureFlash; lastFlash = millis(); }
        if (captureFlash) {
          for (int i = 2; i <= 5; i++) {
            matrix.drawPixel(i, 2, matrix.Color(255,255,255));
            matrix.drawPixel(i, 5, matrix.Color(255,255,255));
            matrix.drawPixel(2, i, matrix.Color(255,255,255));
            matrix.drawPixel(5, i, matrix.Color(255,255,255));
          }
        }
      }

      if (isBeingCaptured) {
        if (canDefend) {
          static bool dangerFlash = false; static unsigned long lastDangerFlash = 0;
          if (millis() - lastDangerFlash > 100) { dangerFlash = !dangerFlash; lastDangerFlash = millis(); }
          if (dangerFlash) {
            for (int i = 0; i < 8; i++) {
              matrix.drawPixel(i, 0, matrix.Color(255,0,0));
              matrix.drawPixel(i, 7, matrix.Color(255,0,0));
              matrix.drawPixel(0, i, matrix.Color(255,0,0));
              matrix.drawPixel(7, i, matrix.Color(255,0,0));
            }
          }
        } else {
          for (int i = 0; i < 8; i++) {
            matrix.drawPixel(i, 0, matrix.Color(100,0,0));
            matrix.drawPixel(i, 7, matrix.Color(100,0,0));
            matrix.drawPixel(0, i, matrix.Color(100,0,0));
            matrix.drawPixel(7, i, matrix.Color(100,0,0));
          }
        }
        unsigned long elapsed = millis() - captureStartTime;
        unsigned long remaining = CAPTURE_WINDOW - elapsed;
        int timeProgress = map(remaining, 0, CAPTURE_WINDOW, 0, 8);
        for (int i = 0; i < timeProgress; i++)
          matrix.drawPixel(i, 7, matrix.Color(255,100,0));
      }

      if (!canCapture && !isBeingCaptured) {
        unsigned long elapsed = millis() - lastCaptureTime;
        int progress = map(elapsed, 0, COOLDOWN_TIME_MS, 0, 5);
        for (int i = 0; i < progress; i++)
          matrix.drawPixel(i, 0, matrix.Color(255, 0, 0));
      }

      if (!canDefend && !isBeingCaptured) {
        unsigned long elapsed = millis() - lastDefendTime;
        int progress = map(elapsed, 0, DEFEND_COOLDOWN, 0, 8);
        for (int i = 0; i < progress; i++)
          matrix.drawPixel(i, 0, matrix.Color(0, 0, 255));
      }

      if (button1Pressing && button1PressStartTime > 0) {
        unsigned long pressDuration = millis() - button1PressStartTime;
        int longPressProgress = map(min(pressDuration, LONG_PRESS_TIME), 0, LONG_PRESS_TIME, 0, 8);
        for (int i = 0; i < longPressProgress; i++)
          matrix.drawPixel(i, 7, matrix.Color(255,165,0));
      }
    } break;

    case VICTORY: {
      unsigned long elapsed = millis() - victoryTime;
      uint16_t victoryColor = colorForTeam(winningTeam);

      if (elapsed < 3000) {
        static bool flash = false; static unsigned long lastFlash = 0;
        if (millis() - lastFlash > 200) { flash = !flash; lastFlash = millis(); }
        if (flash) matrix.fillScreen(victoryColor);
      } else if (elapsed < 6000) {
        int phase = ((millis() / 100) % 28);
        for (int i = 0; i < 8; i++) {
          matrix.drawPixel(i, 0, matrix.Color(50,50,50));
          matrix.drawPixel(i, 7, matrix.Color(50,50,50));
          matrix.drawPixel(0, i, matrix.Color(50,50,50));
          matrix.drawPixel(7, i, matrix.Color(50,50,50));
        }
        if (phase < 7) matrix.drawPixel(phase, 7, victoryColor);
        else if (phase < 14) matrix.drawPixel(7, 7-(phase-7), victoryColor);
        else if (phase < 21) matrix.drawPixel(7-(phase-14), 0, victoryColor);
        else matrix.drawPixel(0, (phase-21), victoryColor);

        matrix.fillRect(3, 3, 2, 2, victoryColor);
      } else {
        static unsigned long lastFirework = 0;
        if (millis() - lastFirework > 500) {
          lastFirework = millis();
          int x = random(1, 7), y = random(1, 7);
          matrix.drawPixel(x, y, victoryColor);
          if (x > 0) matrix.drawPixel(x-1, y, victoryColor);
          if (x < 7) matrix.drawPixel(x+1, y, victoryColor);
          if (y > 0) matrix.drawPixel(x, y-1, victoryColor);
          if (y < 7) matrix.drawPixel(x, y+1, victoryColor);
        }
        matrix.fillRect(3, 3, 2, 2, victoryColor);
      }

      if (button1Pressing && button1PressStartTime > 0) {
        unsigned long pressDuration = millis() - button1PressStartTime;
        int longPressProgress = map(min(pressDuration, LONG_PRESS_TIME), 0, LONG_PRESS_TIME, 0, 8);
        for (int i = 0; i < longPressProgress; i++)
          matrix.drawPixel(i, 0, matrix.Color(255,165,0));
      }
    } break;
  }

  matrix.show();
}
