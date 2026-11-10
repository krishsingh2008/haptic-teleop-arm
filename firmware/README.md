# Firmware

One Arduino sketch, `haptic_hand/haptic_hand.ino`, runs both ESP32-S3 boards.

**Setup:** esp32 board package 3.x and the ESP32Servo library. Board "ESP32S3 Dev Module", Flash 16MB, PSRAM "OPI PSRAM". Upload through the USB-C port labelled UART/COM.

1. Set `#define ROLE_HAND 1` and upload to the hand board. Set it to `0` and upload to the glove board.
2. **Link:** connect glove pin 17 to hand pin 18, hand 17 to glove 18, and GND to GND. For wireless, uncomment `#define USE_ESPNOW` in `link.h`.

| What | Pins | Notes |
|---|---|---|
| FSRs (hand) / bend sensors (glove) | 1, 2, 4, 5, 6 | 10k divider powered from 3V3, never 5 V |
| Finger servos (hand) | 11-16 | Signal only. Servo power comes straight from a 5 V 6 A supply, with its GND joined to the board |
| Vibration motors (glove) | 11-15 | Each through a transistor and flyback diode |
| Never use | 0, 3, 45, 46, 19, 20, 35-37, 43, 44 | Boot, USB and PSRAM pins |

**Calibration:**
- Set `CAL = true` on the glove and copy the open-hand and fist readings into `cal[]`.
- Tune `F_LIMIT` (in `hand_logic.h`) with a kitchen scale.

**Simulate:** `g++ -O2 -std=c++17 ../sim/sim.cpp -o handsim && ./handsim`
