"""Checks for the rev C finger: tendon path open end to end, screw pilots clear of the tendon and the
outside, keyholes/pockets inside the part, and no collisions between segments through the curl.
Run: python verify_revC.py"""
import json, math
import cadquery as cq
from cadquery import Solid, Vector
import finger_revC as C, hand_cad as HC

Lf = HC.hand_geometry(json.load(open("hand_params.json")))["fingers"][0]["L"]
C.M3 = 1.6   # M2 pilots for the base screws
P = {k: v.val() for k, v in C.build(Lf).items()}
ok = lambda c, m: print(("PASS " if c else "FAIL ") + m)
def solid_at(s, p, r=0.12): return s.intersect(Solid.makeSphere(r, Vector(*p))).Volume() > 1e-6

ok(all(s.isValid() for s in P.values()), "all four parts are valid single solids")
ok(solid_at(P["proximal"], (20, 0, 0)) and not solid_at(P["proximal"], (20, 9, 0)), "probe sanity")
# fishing line: sample its centreline inside each part (straight finger)
def line_pts(part):
    A = C.ARM
    if part == "base": return [(x, -A[0]) for x in [-21.5 + i for i in range(20)]]
    if part == "proximal": return [(x, -A[0] + (A[0] - A[1]) * x / Lf[0]) for x in [-6 + 2 * i for i in range(int((Lf[0] + 6) / 2))]]
    if part == "middle": return [(x, -A[1] + (A[1] - A[2]) * x / Lf[1]) for x in [-6 + 2 * i for i in range(int((Lf[1] + 6) / 2))]]
    return [(x, -A[2]) for x in [-6 + 1.5 * i for i in range(int((Lf[2] + 5.5) / 1.5))]]
for n in P:
    blocked = [p for p in line_pts(n) if solid_at(P[n], (p[0], p[1], 0), 0.6)]
    ok(not blocked, f"{n}: fishing-line channel open (1.2 mm probe){' blocked at ' + str(blocked[:3]) if blocked else ''}")
ok(not solid_at(P["distal"], (Lf[2] - 0.3, -C.ARM[2], 0), 0.6), "fishing line exits the fingertip front into the knot pocket")
# screw pilots: material all round, clear of the fishing line by >= 1 mm
caps = [("proximal", C.GAP + 5.5, C.CAP_Z), ("proximal", Lf[0] - C.R - 7.0, -C.CAP_Z), ("distal", C.GAP + 6.5, C.CAP_Z)]
for n, x, z in caps:
    ring = [(x + (C.PILOT_R + 0.9) * math.cos(t), y, z + (C.PILOT_R + 0.9) * math.sin(t)) for t in [k * math.pi / 4 for k in range(8)] for y in (-3.8, -2.0, -0.2)]
    ok(all(solid_at(P[n], q) for q in ring), f"{n} screw at x {x:.1f}: pilot has 0.9 mm+ of plastic all round")
    flex_y = {"proximal": -C.ARM[0] + (C.ARM[0] - C.ARM[1]) * x / Lf[0], "distal": -C.ARM[2]}[n]
    gap = math.hypot(abs(z), max(0.0, C.PILOT_BOTTOM - flex_y)) - C.PILOT_R - C.FLEX_D / 2
    ok(gap >= 1.0, f"{n} screw at x {x:.1f}: {gap:.1f} mm of plastic between pilot and fishing line")
    ok(not solid_at(P[n], (x, 4.0, z), 0.4) and not solid_at(P[n], (x + (C.CAP_R - 0.5) * (1 if z > 0 else 1), 3.0, z), 0.3), f"{n} screw pocket open for the cord wraps")
    side = C.W / 2 - (abs(z) + C.CAP_R)
    ok(side >= 1.5, f"{n} screw pocket side wall {side:.1f} mm")
# keyholes: knot hole open, narrower groove beside it (knot can't pull through)
keys = [("base", -C.BASE_L + 4.0), ("middle", C.GAP + 2.5), ("middle", Lf[1] - C.R - 3.5)]
for n, x in keys:
    ok(not solid_at(P[n], (x, 1.5, 0), 0.5) and solid_at(P[n], (x, 2.0, C.KEY_R + 0.4)), f"{n} keyhole at x {x:.1f}: open, with a wall beside it")
# no segment collisions through the curl
def posed(curl):
    out = {"base": P["base"]}; loc = cq.Location(); X = {"proximal": 0.0, "middle": Lf[0], "distal": Lf[1]}
    for n, a in zip(("proximal", "middle", "distal"), curl):
        loc = loc * cq.Location(Vector(X[n], 0, 0)) * cq.Location(Vector(), Vector(0, 0, 1), -a)
        out[n] = P[n].moved(loc)
    return out
for curl in [(0, 0, 0), (30, 30, 20), (60, 60, 50), (90, 95, 80)]:
    q = posed(curl); names = list(q)
    worst = max(q[a].intersect(q[b]).Volume() for i, a in enumerate(names) for b in names[i + 1:])
    ok(worst < 0.5, f"curl {curl}: segments don't collide (max overlap {worst:.2f} mm3)")
