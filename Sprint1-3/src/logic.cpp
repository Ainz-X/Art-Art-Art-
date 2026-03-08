#include "logic.h"
#include "globals.h"
#include "utils.h"
#include "radio.h"

void handleGameLogic() {
  unsigned long now = millis();

  if (!canCapture && (now - lastCaptureTime) > COOLDOWN_TIME_MS) {
    canCapture = true;
  }

  if (!canDefend && (now - lastDefendTime) > DEFEND_COOLDOWN) {
    canDefend = true;
    Serial.println("防守冷却结束，可以再次防守");
  }

  if (isBeingCaptured && (now - captureStartTime) > CAPTURE_WINDOW) {
    PlayerTeam oldTeam = myTeam;
    myTeam = capturingTeam;
    isBeingCaptured = false;
    Serial.printf("❌ 未能防守！队伍从 %d 变为 %d\n", oldTeam, myTeam);
    canCapture = false;
    lastCaptureTime = now;
  }

  static unsigned long lastSearchUpdate = 0;
  if (gameState == SEARCHING && (now - lastSearchUpdate) > 2000) {
    unsigned long elapsed = now - gameStartTime;
    int remainingSeconds = (SEARCH_TIME_MS - elapsed) / 1000;
    Serial.printf("搜索中... 发现 %d 个设备，剩余 %d 秒\n", peers.size(), max(0, remainingSeconds));
    lastSearchUpdate = now;
  }

  // 按钮1（反转逻辑：HIGH=按下）
  bool button1Current = digitalRead(BUTTON1_PIN) == HIGH;

  static bool lastDebugState = false;
  if (button1Current != lastDebugState) {
    Serial.printf("[DEBUG] 按钮1状态变化: %s (digitalRead=%d)\n",
                  button1Current ? "按下" : "释放",
                  digitalRead(BUTTON1_PIN));
    lastDebugState = button1Current;
  }

  if (button1Current && !button1Pressing) {
    button1Pressing = true;
    button1PressStartTime = now;
    Serial.println("[DEBUG] 开始计时长按");
  } else if (!button1Current && button1Pressing) {
    button1Pressing = false;
    unsigned long pressDuration = now - button1PressStartTime;
    Serial.printf("[DEBUG] 按钮释放，持续时长: %.2f秒\n", pressDuration / 1000.0f);

    if (gameState == SEARCHING || gameState == PLAYING || gameState == VICTORY) {
      if (pressDuration >= LONG_PRESS_TIME) {
        gameState = TEAM_SELECT;
        myTeam = TEAM_NEUTRAL;
        peers.clear();
        Serial.println("长按重置 - 游戏结束，重新选择队伍");
      } else {
        Serial.printf("需要长按3秒才能重置 (当前: %.1f秒)\n", pressDuration / 1000.0f);
      }
    } else {
      if (pressDuration < LONG_PRESS_TIME) {
        switch (gameState) {
          case TEAM_SELECT:
            gameState = WAITING;
            Serial.printf("已选择队伍: %d - 按按键1开始游戏\n", (int)selectedTeam);
            break;
          case WAITING:
            gameState = SEARCHING;
            gameStartTime = now;
            myTeam = assignTeamByMac(selfMac);
            Serial.printf("开始搜索设备... 我的队伍: %d\n", (int)myTeam);
            break;
          default: break;
        }
      }
    }
  }

  if (button1Pressing && (gameState == SEARCHING || gameState == PLAYING || gameState == VICTORY)) {
    unsigned long pressDuration = now - button1PressStartTime;
    static unsigned long lastProgressPrint = 0;
    if (pressDuration >= LONG_PRESS_TIME) {
      if (now - lastProgressPrint > 200) {
        Serial.println(">>> 释放按键以重置游戏 <<<");
        lastProgressPrint = now;
      }
    } else {
      if (now - lastProgressPrint > 500) {
        Serial.printf("长按重置中... %.1f/3.0秒\n", pressDuration / 1000.0f);
        lastProgressPrint = now;
      }
    }
  }

  // 按钮2（短按：切换队伍 / 防守 / 抓捕）
  if (readButton(BUTTON2_PIN, lastButton2, lastButton2Press)) {
    if (gameState == TEAM_SELECT) {
      switch(selectedTeam) {
        case TEAM_RED:    selectedTeam = TEAM_GREEN; break;
        case TEAM_GREEN:  selectedTeam = TEAM_BLUE;  break;
        case TEAM_BLUE:   selectedTeam = TEAM_YELLOW;break;
        case TEAM_YELLOW: selectedTeam = TEAM_RED;   break;
        default:          selectedTeam = TEAM_RED;   break;
      }
      Serial.printf("切换到队伍: %d\n", (int)selectedTeam);
    } else if (isBeingCaptured && canDefend) {
      isBeingCaptured = false;
      canDefend = false;
      lastDefendTime = now;
      Serial.printf("🛡️ 防守成功！抵挡了来自 %s 的抓捕\n", capturingPlayer.c_str());
      Serial.println("防守进入30秒冷却");
    } else if (isBeingCaptured && !canDefend) {
      unsigned long remainingCooldown = (DEFEND_COOLDOWN - (now - lastDefendTime)) / 1000;
      Serial.printf("防守冷却中，还剩 %lu 秒\n", remainingCooldown);
    } else if (gameState == PLAYING && canCapture && !isBeingCaptured) {
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
          Serial.printf("跳过队友 %s (队伍: %d, RSSI: %d)\n",
                        kv.first.c_str(), kv.second.team, kv.second.rssi);
        }
      }
      if (!foundTarget) {
        Serial.println("附近没有可抓捕的敌人");
      }
    }
  }

  if (gameState == SEARCHING && (now - gameStartTime) > SEARCH_TIME_MS) {
    gameState = PLAYING;
    Serial.printf("游戏开始！我的队伍: %d，发现 %d 个设备\n", (int)myTeam, peers.size());
  }

  if (gameState == PLAYING && checkVictory()) {
    gameState = VICTORY;
    victoryTime = now;
    winningTeam = myTeam;
    Serial.printf("🏆 胜利！队伍 %d 统一了所有玩家！\n", (int)myTeam);
  }
}
