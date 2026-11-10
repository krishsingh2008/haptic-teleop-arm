// Glove <-> hand link. Default: a 3-wire cable (TX, RX, GND) between the two boards.
// Going wireless later: uncomment USE_ESPNOW, upload to both boards. Nothing else changes.
#pragma once
#include <Arduino.h>
// #define USE_ESPNOW

#ifndef USE_ESPNOW
inline void linkBegin(int rx, int tx) { Serial1.begin(921600, SERIAL_8N1, rx, tx); }
template <typename T> inline void linkSend(const T& p) { Serial1.write((const uint8_t*)&p, sizeof(T)); }
template <typename T> inline bool linkRead(T& p) {
  bool got = false;
  while (Serial1.available() >= (int)sizeof(T)) {
    if (Serial1.peek() != MAGIC) { Serial1.read(); continue; }  // resync after a dropped byte
    T tmp;
    Serial1.readBytes((uint8_t*)&tmp, sizeof(T));
    p = tmp;
    got = true;
  }
  return got;
}
#else
#include <WiFi.h>
#include <esp_now.h>
// shortcut: broadcast, fine for one glove + one hand; pair by MAC address if another set is nearby
static uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t rx_buf[32];
static volatile bool rx_new = false;
static void onRecv(const esp_now_recv_info_t*, const uint8_t* data, int len) {  // esp32 core 3.x signature
  if (len <= (int)sizeof(rx_buf) && data[0] == MAGIC) { memcpy(rx_buf, data, len); rx_new = true; }
}
inline void linkBegin(int, int) {
  WiFi.mode(WIFI_STA);
  esp_now_init();
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  esp_now_add_peer(&peer);
}
template <typename T> inline void linkSend(const T& p) { esp_now_send(BROADCAST, (const uint8_t*)&p, sizeof(T)); }
template <typename T> inline bool linkRead(T& p) {
  if (!rx_new) return false;
  memcpy(&p, rx_buf, sizeof(T));
  rx_new = false;
  return true;
}
#endif
