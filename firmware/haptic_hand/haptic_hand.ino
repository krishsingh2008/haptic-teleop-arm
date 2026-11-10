// Haptic robot hand: ONE sketch for both ESP32-S3 boards.
// Set ROLE_HAND below, upload to the hand board; flip it to 0, upload to the glove board.
// Arduino IDE: board "ESP32S3 Dev Module", Flash Size 16MB, PSRAM "OPI PSRAM",
// esp32 core 3.x, library ESP32Servo. Upload through the USB-C port labelled UART/COM.
#define ROLE_HAND 1   // 1 = hand board, 0 = glove board

#include "hand_logic.h"
#include "link.h"

// Pin rules: analog inputs on ADC1 only (GPIO1-10; ADC2 stops working while the radio is on).
// Never use 0, 3, 45, 46 (boot straps), 19/20 (USB), 35-37 (PSRAM on N16R8), 43/44 (USB serial).
// Every sensor divider runs from 3V3, never 5 V: these pins are not 5 V tolerant.
const int SENSOR_PIN[N_FINGERS] = {1, 2, 4, 5, 6};        // thumb..pinky: FSRs on the hand, bend sensors on the glove
const int LINK_TX = 17, LINK_RX = 18;                      // cable mode: TX of one board -> RX of the other, plus GND

#if ROLE_HAND
#include <ESP32Servo.h>
const int SERVO_PIN[N_SERVOS] = {11, 12, 13, 14, 15, 16};  // signal wires only; servo power comes straight from the 5 V 5 A supply
const float THUMB_ROT_DEG = 70;                            // thumb rotation at full thumb bend; tune on the hand
// Bend calibration (open hand, fist) per finger: set ROLE_HAND 0 + CAL true on the glove and copy the numbers here.
BendCal cal[N_FINGERS] = {{1200, 2600}, {1200, 2600}, {1200, 2600}, {1200, 2600}, {1200, 2600}};
Servo servo[N_SERVOS];
FingerCtl ctl[N_FINGERS];
GlovePacket glove = {};
#else
const int BUZZ_PIN[N_FINGERS] = {11, 12, 13, 14, 15};      // each drives a transistor + flyback diode, never a motor directly
const bool CAL = false;                                    // true: print raw bend readings for calibration
HandPacket hand = {};
uint16_t smooth[N_FINGERS];
uint8_t seq = 0;
#endif
uint32_t last_rx_ms = 0;

void setup() {
  Serial.begin(115200);
  analogReadResolution(12);
#if ROLE_HAND
  // one servo at a time: six starting together can trip the supply's overload cutoff
  for (int i = 0; i < N_SERVOS; i++) { servo[i].setPeriodHertz(50); servo[i].attach(SERVO_PIN[i], 500, 2400); servo[i].write(0); delay(150); }
#else
  for (int i = 0; i < N_FINGERS; i++) { pinMode(BUZZ_PIN[i], OUTPUT); smooth[i] = analogRead(SENSOR_PIN[i]); }
#endif
  linkBegin(LINK_RX, LINK_TX);
}

void loop() {
  static uint32_t last = millis();
  uint32_t now = millis();
  if (now - last < 10) return;  // 100 Hz
  [[maybe_unused]] float dt = (now - last) / 1000.0f;   // only the hand uses it
  last = now;

#if ROLE_HAND
  if (linkRead(glove)) last_rx_ms = now;
  bool link_ok = now - last_rx_ms < LINK_TIMEOUT_MS;   // glove silent -> fingers relax
  HandPacket out = {MAGIC, glove.seq, {}};
  for (int i = 0; i < N_FINGERS; i++) {
    float f = fsrForceN(analogRead(SENSOR_PIN[i]));
    float closure = bendToClosure(glove.bend[i], cal[i]);
    servo[i].write((int)ctl[i].update(closure, f, link_ok, dt));
    out.force_cN[i] = (uint16_t)(f * 100);
  }
  servo[5].write(link_ok ? (int)(bendToClosure(glove.bend[0], cal[0]) * THUMB_ROT_DEG) : 0);
  linkSend(out);
#else
  GlovePacket out = {MAGIC, seq++, {}};
  for (int i = 0; i < N_FINGERS; i++) out.bend[i] = smooth[i] = (smooth[i] * 3 + analogRead(SENSOR_PIN[i])) / 4;  // ~30 ms smoothing
  linkSend(out);
  if (linkRead(hand)) last_rx_ms = now;
  bool fresh = now - last_rx_ms < LINK_TIMEOUT_MS;
  for (int i = 0; i < N_FINGERS; i++) analogWrite(BUZZ_PIN[i], fresh ? hapticDuty(hand.force_cN[i] / 100.0f) : 0);
  if (CAL) { for (int i = 0; i < N_FINGERS; i++) { Serial.print(out.bend[i]); Serial.print(' '); } Serial.println(); }
#endif
}
