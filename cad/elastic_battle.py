"""Battle of 10 knuckle-elastic tensioner designs for the rev B finger (same 1 mm cord on every knuckle,
tension set per knuckle, ideally with a screwdriver).  Run: python elastic_battle.py

Each design is scored on numbers worked out from the finger's real geometry (17 x 15 mm segments,
1.9 mm gap between neighbouring fingers, so only the BACK of each finger is reachable in the hand).
Hard requirements fail a design outright; everything else is a weighted 0-10 score.
"""
import math

CORD = 1.0              # mm, polyurethane / latex round cord
TENSION = 2.0           # N, about the most any knuckle cord carries at full fist
PREST_NEEDED = 15.0     # mm of adjustment range wanted (0 to ~55% stretch of a 27 mm cord)
GOOD_STEP = (0.5, 2.0)  # mm of cord per smallest repeatable adjustment that feels "fine-tunable"

def wind_per_turn(shaft_d):            # cord taken up per full turn on a shaft of diameter d
    return math.pi * (shaft_d + CORD)

D = [
    dict(name="1. Knot + filament anchor pin (rev B as printed)",
         how="Tie a knot, pin it through a cross-hole. To change tension, untie and re-tie.",
         step=None, range=None, hold=10, lock="knot", access_back=True, tool="none",
         install_steps=6, threading="through a closed 3 mm channel", extra_parts=0, wear=8, repeat=2, fits=True),
    dict(name="2. Twist the anchor pin (friction peg)",
         how="Wind the cord onto the 1.75 mm filament pin and twist it with pliers; friction holds it.",
         step=wind_per_turn(1.75) / 4, range=2 * wind_per_turn(1.75), hold=3, lock="friction in snug hole",
         access_back=False, tool="pliers", install_steps=6, threading="closed channel", extra_parts=0, wear=6, repeat=5, fits=True),
    dict(name="3. Hex click-peg, side mounted",
         how="Printed peg with a hex head in a hex pocket on the side face; pull out, turn a flat, push in.",
         step=wind_per_turn(2.4) / 6, range=3 * wind_per_turn(2.4), hold=7, lock="hex detent",
         access_back=False, tool="fingers", install_steps=7, threading="closed channel", extra_parts=3, wear=6, repeat=9, fits=True,
         note="Lifting the peg to turn it lets the cord spin it back: you lose the setting while adjusting."),
    dict(name="4. Hex click-peg, back mounted",
         how="Same peg standing upright in a pocket on the back; slot head for a screwdriver.",
         step=wind_per_turn(2.4) / 6, range=3 * wind_per_turn(2.4), hold=7, lock="hex detent",
         access_back=True, tool="screwdriver", install_steps=6, threading="open back groove", extra_parts=3, wear=6, repeat=9, fits=True,
         note="Same spin-back problem while lifted; a 2.4 mm printed hex at 0.2 mm layers is soft and rounds off."),
    dict(name="5. Screw capstan, back mounted",
         how="An M2 self-tapping screw stands in an open pocket on the back. The cord's end loop goes over the "
             "screw; turning the screw winds the cord onto it. Thread friction stops it unwinding.",
         step=wind_per_turn(2.0) / 8, range=3 * wind_per_turn(2.0), hold=9, lock="screw thread in PLA (self-locking)",
         access_back=True, tool="screwdriver", install_steps=4, threading="open back groove", extra_parts=3, wear=8, repeat=8, fits=True),
    dict(name="6. Grub-screw clamp",
         how="Pull the cord to a pen mark with pliers, then clamp it with an M2 grub screw from the back.",
         step=None, range=12.0, hold=5, lock="clamp (can crush or slip on elastic)",
         access_back=True, tool="hex key", install_steps=5, threading="open back groove", extra_parts=3, wear=3, repeat=4, fits=True,
         note="The screw only locks; it can't tighten. Clamping a stretchy cord also thins and weakens it."),
    dict(name="7. Lead-screw sled",
         how="A captive M2 nut slides in a slot along the finger; turning a lengthwise screw pulls the cord anchor.",
         step=0.4 / 4, range=8.0, hold=10, lock="screw thread (self-locking)",
         access_back=False, tool="screwdriver", install_steps=7, threading="open back groove", extra_parts=3, wear=8, repeat=10, fits=False,
         note="The screw head would face a knuckle, so only the base cord could be reached; 8 mm of travel is too short."),
    dict(name="8. Eccentric cam plug",
         how="A round plug on the back with an off-centre cord post; turning it with a screwdriver moves the anchor.",
         step=0.6, range=4.0, hold=5, lock="friction",
         access_back=True, tool="screwdriver", install_steps=5, threading="open back groove", extra_parts=1, wear=7, repeat=7, fits=True,
         note="Only about 4 mm of range: you'd still have to get the knot position nearly right first."),
    dict(name="9. Deflection screw",
         how="A screw from the back pushes the middle of the cord down into a pocket, lengthening its path.",
         step=0.05, range=1.5, hold=9, lock="screw thread",
         access_back=True, tool="screwdriver", install_steps=4, threading="open back groove", extra_parts=1, wear=4, repeat=8, fits=True,
         note="Pushing a 27 mm cord sideways 4 mm only adds about 1.5 mm of stretch, and it rubs at the push point."),
    dict(name="10. Ratchet peg with printed pawl",
         how="Peg with sawtooth teeth and a flexing printed pawl, turned one way with a screwdriver; press to release.",
         step=wind_per_turn(2.4) / 10, range=3 * wind_per_turn(2.4), hold=6, lock="printed pawl",
         access_back=True, tool="screwdriver", install_steps=6, threading="open back groove", extra_parts=2, wear=7, repeat=9, fits=False,
         note="Teeth and a pawl this small (under 8 mm) don't print cleanly at 0.4 mm nozzle; the pawl creeps."),
]

W = dict(hold=20, range=15, step=10, repeat=10, wear=5, install=20, access=10, parts=5, fit=5)


def score(d):
    s = {}
    s["hold"] = d["hold"]
    s["range"] = 0 if d["range"] is None else min(10, 10 * d["range"] / PREST_NEEDED)
    if d["step"] is None:
        s["step"] = 0
    else:
        lo, hi = GOOD_STEP
        s["step"] = 10 if lo <= d["step"] <= hi else max(0, 10 - 4 * abs(math.log2(d["step"] / (lo if d["step"] < lo else hi))))
    s["repeat"] = d["repeat"]
    s["wear"] = d["wear"]
    s["install"] = max(0, 10 - 1.5 * (d["install_steps"] - 3)) * (1.0 if d["threading"] == "open back groove" else 0.6)
    s["access"] = 10 if d["access_back"] else 0
    s["parts"] = 10 - 2.5 * d["extra_parts"] / 3
    s["fit"] = 10 if d["fits"] else 0
    total = sum(W[k] * s[k] for k in W) / sum(W.values()) * 10
    hard = d["access_back"] and d["fits"]   # must be reachable in the assembled hand and fit the 17 x 15 mm finger
    return total, s, hard


if __name__ == "__main__":
    rows = []
    for d in D:
        t, s, hard = score(d)
        rows.append((t if hard else t * 0.5, t, hard, d, s))
    rows.sort(key=lambda r: -r[0])
    for final, t, hard, d, s in rows:
        st = "-" if d["step"] is None else f"{d['step']:.2f}"
        rg = "-" if d["range"] is None else f"{d['range']:.0f}"
        print(f"{final:5.1f}  {'' if hard else '(fails a hard rule) '}{d['name']}")
        print(f"       step {st} mm, range {rg} mm, lock: {d['lock']}, tool: {d['tool']}, install steps {d['install_steps']}")
        if d.get("note"):
            print("       ! " + d["note"])
    print("WINNER:", rows[0][3]["name"])
