// Shared logic for the glove and hand boards (ESP32-S3). No Arduino calls here,
// so the same code runs on the boards and in the PC simulator (sim/sim.cpp).
#pragma once
#include <stdint.h>
#include <math.h>

const int N_FINGERS = 5;   // thumb, index, middle, ring, pinky
const int N_SERVOS  = 6;   // 5 curl servos + thumb rotation (driven by the thumb sensor)

// ---- tuning (change these after testing) ----
const float FIST_DEG    = 106.0f;  // servo angle that makes a full fist (finger sim)
const float F_TOUCH     = 0.3f;    // N at the fingertip that counts as "touching"
const float F_LIMIT     = 3.0f;    // N: stop squeezing harder than this
const float F_HAPTIC_MAX = 3.0f;   // N that gives full-strength buzz on the glove
const uint32_t LINK_TIMEOUT_MS = 250;  // no glove data for this long -> open the hand

#pragma pack(push, 1)
struct GlovePacket { uint8_t magic; uint8_t seq; uint16_t bend[N_FINGERS]; };   // glove -> hand, raw ADC 0-4095
struct HandPacket  { uint8_t magic; uint8_t seq; uint16_t force_cN[N_FINGERS]; }; // hand -> glove, centinewtons
#pragma pack(pop)
const uint8_t MAGIC = 0xA5;

// Per-finger bend-sensor calibration: raw ADC reading with the hand open and in a fist.
struct BendCal { uint16_t open, fist; };

inline float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }

// Raw bend reading -> how closed the finger should be (0 = open, 1 = fist).
inline float bendToClosure(uint16_t raw, BendCal c) {
  if (c.fist == c.open) return 0;
  return clamp01((float)((int)raw - (int)c.open) / (float)((int)c.fist - (int)c.open));
}

// FSR 400 on a 10k pull-down divider from 3.3 V: ADC reading -> force in newtons.
// shortcut: generic FSR curve R = 10k / F^0.9; calibrate with a kitchen scale for real numbers.
inline float fsrForceN(uint16_t raw) {
  if (raw < 20) return 0;
  float v = raw / 4095.0f;                      // fraction of 3.3 V
  float r_fsr = 10000.0f * (1 - v) / v;         // ohms
  return powf(10000.0f / r_fsr, 1 / 0.9f);     // invert R = 10k / F^0.9
}

// One finger on the hand board: follows the glove, but never squeezes past F_LIMIT.
struct FingerCtl {
  float limit_deg = 180;  // ceiling on the command, lowered while the fingertip is over F_LIMIT
  float cmd_deg = 0;

  // dt_s: loop period; target_closure: from the glove (0-1); force_n: this fingertip; link_ok: glove data fresh
  float update(float target_closure, float force_n, bool link_ok, float dt_s) {
    float target = link_ok ? target_closure * FIST_DEG : 0;   // failsafe: relax when the link drops
    if (force_n > F_LIMIT) limit_deg = cmd_deg - 40 * dt_s;   // back off 40 deg/s while over the limit
    else if (force_n < 0.8f * F_LIMIT) limit_deg += 120 * dt_s;
    if (limit_deg > 180) limit_deg = 180;
    if (limit_deg < 0) limit_deg = 0;
    float goal = target < limit_deg ? target : limit_deg;
    // No position feedback from an MG90S, so never let the command run ahead of what the servo
    // can follow under load (~250 deg/s closing). Otherwise the force limit acts too late.
    float up = 250 * dt_s, down = 600 * dt_s;
    cmd_deg += goal > cmd_deg ? (goal - cmd_deg < up ? goal - cmd_deg : up) : (cmd_deg - goal < down ? goal - cmd_deg : -down);
    return cmd_deg;
  }
};

// Glove side: fingertip force -> vibration strength 0-255 (0 below F_TOUCH).
inline uint8_t hapticDuty(float force_n) {
  if (force_n < F_TOUCH) return 0;
  return (uint8_t)(60 + 195 * clamp01((force_n - F_TOUCH) / (F_HAPTIC_MAX - F_TOUCH)));  // 60 = weakest buzz you can feel
}
