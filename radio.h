#pragma once
#include <Arduino.h>
#include <esp_now.h>

void initWiFiEspNow();
void onDataRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len);
void sendPing();
void sendCaptureCommand(const String& targetMac);
