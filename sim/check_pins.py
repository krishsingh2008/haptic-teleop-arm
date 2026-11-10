"""Checks the pin map in haptic_hand.ino against the ESP32-S3 N16R8 rules.  Run: python3 check_pins.py"""
import re, pathlib

src = (pathlib.Path(__file__).parent.parent / "firmware" / "haptic_hand" / "haptic_hand.ino").read_text()
arr = lambda name: [int(x) for x in re.search(name + r"\[[^\]]*\]\s*=\s*\{([^}]*)\}", src).group(1).split(",")]
one = lambda name: int(re.search(name + r"\s*=\s*(\d+)", src).group(1))

sensors, servos, buzz = arr("SENSOR_PIN"), arr("SERVO_PIN"), arr("BUZZ_PIN")
link = [one("LINK_TX"), one("LINK_RX")]
FORBIDDEN = {0: "boot strap", 3: "boot strap", 45: "boot strap", 46: "boot strap", 19: "USB", 20: "USB",
             35: "PSRAM", 36: "PSRAM", 37: "PSRAM", 43: "USB serial", 44: "USB serial"}
ok = True
def check(cond, msg):
    global ok
    print(("PASS  " if cond else "FAIL  ") + msg)
    ok &= cond

for board, pins in (("hand", sensors + servos + link), ("glove", sensors + buzz + link)):
    bad = [f"{p} ({FORBIDDEN[p]})" for p in pins if p in FORBIDDEN]
    check(not bad, f"{board}: no reserved pins used {bad or ''}")
    check(len(set(pins)) == len(pins), f"{board}: no pin used twice")
    check(all(p <= 48 and not 22 <= p <= 34 for p in pins), f"{board}: every pin exists on the DevKitC-1 header")
check(all(1 <= p <= 10 for p in sensors), "all 5 analog sensors on ADC1 (GPIO1-10), so they keep working with ESP-NOW on")
check(len(servos) <= 8, f"hand: {len(servos)} servo PWM outputs fit the S3's 8 LEDC channels")
check(len(buzz) <= 8, f"glove: {len(buzz)} haptic PWM outputs fit the S3's 8 LEDC channels")
print("ALL PASS" if ok else "SOME CHECKS FAILED")
raise SystemExit(0 if ok else 1)
