#include "radio.h"
#include "config.h"
#include "globals.h"
#include "utils.h"

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);

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

void sendCaptureCommand(const String& /*targetMac*/) {
  Packet pkt;
  memcpy(pkt.magic, MAGIC, 4);
  memcpy(pkt.mac, selfMac, 6);
  pkt.seq = ++seqNo;
  pkt.team = myTeam;
  pkt.state = gameState;
  pkt.captureCmd = 1;
  esp_now_send(BCAST, reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
}

void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (!info || !data || len < (int)sizeof(Packet)) return;
  const Packet *p = reinterpret_cast<const Packet*>(data);
  if (memcmp(p->magic, MAGIC, 4) != 0) return;

  String macStr = macToString(info->src_addr);
  int rssiNow = 0; if (info->rx_ctrl) rssiNow = info->rx_ctrl->rssi;

  if (p->captureCmd == 1 && gameState == PLAYING) {
    if (rssiNow > CAPTURE_DISTANCE && p->team != myTeam && !isBeingCaptured) {
      isBeingCaptured = true;
      captureStartTime = millis();
      capturingPlayer = macStr;
      capturingTeam = p->team;
      Serial.printf("⚠️ 正在被 %s 抓捕！(队伍: %d)\n", macStr.c_str(), p->team);
    } else if (isBeingCaptured) {
      Serial.printf("收到 %s 的抓捕命令，但已经在被抓捕状态中\n", macStr.c_str());
    } else if (p->team == myTeam) {
      Serial.printf("收到队友 %s 的抓捕命令，忽略\n", macStr.c_str());
    } else {
      Serial.printf("收到 %s 的抓捕命令，但不在范围内 (RSSI: %d)\n", macStr.c_str(), rssiNow);
    }
  }

  auto it = peers.find(macStr);
  if (it == peers.end()) {
    peers[macStr] = { rssiNow, millis(), p->team, p->state, false };
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
