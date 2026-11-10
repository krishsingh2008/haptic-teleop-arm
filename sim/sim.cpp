// PC simulation of the full hand running the real control code (firmware/haptic_hand/hand_logic.h).
// Build + run:  g++ -O2 -std=c++17 sim.cpp -o sim && ./sim
// Models: glove bend sensors, the glove<->hand link (cable or ESP-NOW with loss), MG90S servos
// (50 Hz frames, load-dependent speed, stall at 11.2 N line pull), tendon + elastic return,
// a 65 mm tennis ball, FSR 400 dividers into a noisy 12-bit ADC, and the 5 V servo supply.
#include <cstdio>
#include <cmath>
#include <random>
#include <deque>
#include "../firmware/haptic_hand/hand_logic.h"

const double DT = 0.001;                      // 1 ms physics step
const double R_DRUM = 15.8, T_STALL = 11.2;   // mm, N (1.8 kg*cm MG90S)
const double L_FIST = FIST_DEG * M_PI / 180 * R_DRUM;           // line wound for a full fist
const double CONTACT[5] = {0.35, 0.50, 0.48, 0.52, 0.60};       // closure where each finger meets the ball
const double K_CONTACT = 0.8;                 // N of extra line tension per mm wound past contact
const char* NAME[5] = {"thumb", "index", "middle", "ring", "pinky"};

double Tfree(double c) { return 0.3 + 2.5 * c; }  // elastic return, from the rev C finger sim (0.2 N/mm cord)

struct Result { double lat_ms, haptic_ms, touch_t, peak_force, peak_amp, hold_amp, open_after_drop_ms, max_stall_s; int drops; };

Result run(bool espnow, bool force_limit, FILE* csv) {
  std::mt19937 rng(7);
  std::normal_distribution<double> adc_noise(0, 12);
  std::uniform_real_distribution<double> u(0, 1);
  const double link_ms = espnow ? 3.0 : 1.0, loss = espnow ? 0.02 : 0.0;
  BendCal cal = {1200, 2600};

  FingerCtl ctl[5];
  double servo[6] = {0}, cmd[6] = {0}, force[5] = {0}, stall_t[5] = {0};
  std::deque<std::pair<double, GlovePacket>> g2h;   // packets in flight
  std::deque<std::pair<double, HandPacket>> h2g;
  GlovePacket glove_rx = {};
  HandPacket hand_rx = {};
  double smooth[5] = {-1, -1, -1, -1, -1};
  double last_glove_rx = -1, last_hand_rx = -1, duty_index = 0;
  Result r = {NAN, NAN, NAN, 0, 0, 0, NAN, 0, 0};
  double hold_sum = 0; int hold_n = 0;

  for (int k = 0; k <= 8000; k++) {
    double t = k * DT;
    // the person: open 0-1 s, squeeze 1.0-1.5 s, hold, glove link dies 3.0-4.0 s, open 6.0-6.5 s
    double want = t < 1 ? 0 : t < 1.5 ? (t - 1) / 0.5 : t < 6 ? 1 : t < 6.5 ? 1 - (t - 6) / 0.5 : 0;
    bool glove_alive = !(t >= 3.0 && t < 4.0);

    // glove board, 100 Hz
    if (k % 10 == 0 && glove_alive) {
      GlovePacket p = {MAGIC, (uint8_t)k, {}};
      for (int i = 0; i < 5; i++) {   // same 4-tap smoothing as the glove firmware
        double raw = cal.open + want * (cal.fist - cal.open) + adc_noise(rng);
        smooth[i] = smooth[i] < 0 ? raw : (smooth[i] * 3 + raw) / 4;
        p.bend[i] = (uint16_t)smooth[i];
      }
      if (u(rng) >= loss) g2h.push_back({t + link_ms / 1000, p}); else r.drops++;
      while (!h2g.empty() && h2g.front().first <= t) { hand_rx = h2g.front().second; h2g.pop_front(); last_hand_rx = t; }
      bool fresh = last_hand_rx >= 0 && (t - last_hand_rx) * 1000 < LINK_TIMEOUT_MS;
      duty_index = fresh ? hapticDuty(hand_rx.force_cN[1] / 100.0) : 0;
    }
    while (!g2h.empty() && g2h.front().first <= t) { glove_rx = g2h.front().second; g2h.pop_front(); last_glove_rx = t; }

    // hand board, 100 Hz
    if (k % 10 == 0) {
      bool link_ok = last_glove_rx >= 0 && (t - last_glove_rx) * 1000 < LINK_TIMEOUT_MS;
      HandPacket out = {MAGIC, glove_rx.seq, {}};
      for (int i = 0; i < 5; i++) {
        double v = force[i] > 0.01 ? 1.0 / (1.0 + 1.0 / pow(force[i], 0.9)) : 0;   // true FSR: R = 10k / F^0.9
        int raw = (int)std::round(std::min(4095.0, std::max(0.0, v * 4095 + adc_noise(rng))));
        float f = fsrForceN((uint16_t)raw);
        float closure = bendToClosure(glove_rx.bend[i], cal);
        cmd[i] = ctl[i].update(closure, force_limit ? f : 0.0f, link_ok, 0.01f);
        out.force_cN[i] = (uint16_t)(f * 100);
        if (i == 0) cmd[5] = link_ok ? closure * 70 : 0;
      }
      if (u(rng) >= loss) h2g.push_back({t + link_ms / 1000, out}); else r.drops++;
    }

    // servos: a new pulse only lands every 20 ms frame
    static double latched[6];
    if (k % 20 == 0) for (int s = 0; s < 6; s++) latched[s] = std::round(cmd[s]);
    double amps = 0;
    for (int s = 0; s < 6; s++) {
      double c_free = servo[s] * M_PI / 180 * R_DRUM / L_FIST;
      double T = 0.5;                              // thumb rotation: light load
      if (s < 5) {
        double cc = CONTACT[s];
        T = c_free <= cc ? Tfree(c_free) : Tfree(cc) + K_CONTACT * (c_free - cc) * L_FIST;
        force[s] = c_free <= cc ? 0 : 0.5 * (T - Tfree(cc));   // fingertip force
      }
      double load = std::min(1.0, T / T_STALL);
      double err = latched[s] - servo[s];
      double speed = err > 0 ? 600 * (1 - load) : 600;   // deg/s, MG90S 0.1 s/60 deg; unwinding is helped by the elastics
      bool stalled = err > 0.5 && load >= 0.999;
      if (fabs(err) > 0.5 && !stalled) servo[s] += std::max(-speed * DT, std::min(speed * DT, err));
      bool moving = fabs(err) > 0.5;
      amps += stalled ? 0.7 : moving ? 0.12 + 0.6 * load : 0.01 + 0.68 * load;
      if (s < 5 && (stalled || (load > 0.9 && !moving))) stall_t[s] += DT;
    }
    r.peak_amp = std::max(r.peak_amp, amps);
    if (t > 2.0 && t < 3.0) { hold_sum += amps; hold_n++; }
    for (int i = 0; i < 5; i++) r.peak_force = std::max(r.peak_force, force[i]);
    if (std::isnan(r.lat_ms) && servo[1] >= 0.25 * FIST_DEG) r.lat_ms = (t - 1.125) * 1000;    // glove hit 25% at 1.125 s
    if (std::isnan(r.touch_t) && force[1] > F_TOUCH) r.touch_t = t;
    if (std::isnan(r.haptic_ms) && duty_index > 0) r.haptic_ms = (t - r.touch_t) * 1000;   // touch -> glove buzz
    if (t >= 3.0 && std::isnan(r.open_after_drop_ms) && servo[1] < 5) r.open_after_drop_ms = (t - 3.0) * 1000;
    if (csv && k % 5 == 0)
      fprintf(csv, "%d,%d,%.3f,%.3f,%.2f,%.2f,%.3f,%.3f,%.0f\n", espnow, force_limit, t, want, servo[1], cmd[1], force[1], amps, duty_index);
  }
  r.hold_amp = hold_sum / hold_n;
  for (int i = 0; i < 5; i++) r.max_stall_s = std::max(r.max_stall_s, stall_t[i]);
  return r;
}

int main() {
  FILE* csv = fopen("sim_out.csv", "w");
  fprintf(csv, "espnow,limit,t,glove,index_deg,index_cmd,index_force,servo_amps,glove_buzz\n");
  printf("%-26s %8s %9s %9s %9s %9s %10s %8s\n", "case", "lag ms", "peak N", "peak A", "hold A", "stall s", "drop->open", "lost pk");
  struct { const char* n; bool e, l; } cases[] = {{"cable, force limit ON", 0, 1}, {"ESP-NOW, force limit ON", 1, 1}, {"ESP-NOW, force limit OFF", 1, 0}};
  Result res[3];
  for (int c = 0; c < 3; c++) {
    res[c] = run(cases[c].e, cases[c].l, csv);
    Result& r = res[c];
    printf("%-26s %8.0f %9.2f %9.2f %9.2f %9.2f %8.0f ms %8d\n", cases[c].n, r.lat_ms, r.peak_force, r.peak_amp, r.hold_amp, r.max_stall_s, r.open_after_drop_ms, r.drops);
  }
  printf("index finger touches the ball at %.3f s; glove buzzes %.0f ms later over cable, %.0f ms over ESP-NOW\n", res[1].touch_t, res[0].haptic_ms, res[1].haptic_ms);
  fclose(csv);

  // self-check: the things the firmware promises
  int fails = 0;
  auto check = [&](bool ok, const char* what) { printf("%s  %s\n", ok ? "PASS" : "FAIL", what); fails += !ok; };
  check(res[1].peak_force <= F_LIMIT * 1.35, "force limit keeps fingertip force near F_LIMIT over ESP-NOW");
  check(res[1].max_stall_s < 0.1 && res[2].max_stall_s > 1.0, "limit keeps servos out of stall; without it they sit stalled on the ball");
  check(res[1].open_after_drop_ms < 600, "hand opens within 0.6 s when the glove link drops");
  check(res[0].peak_amp < 5.0 && res[1].peak_amp < 5.0, "servo current stays under the 5 A supply with the limit on");
  check(res[1].lat_ms < 150, "glove-to-finger lag under 150 ms");
  return fails;
}
