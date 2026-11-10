"""Targeted checks for the review findings."""
import numpy as np, cadquery as cq
from cadquery import Vector
import hand_cad as HC
p, G, parts = HC.build("hand_params.json")
palm = parts["palm"].val(); car = parts["thumb_carrier"].val(); C = G["thumb"]["C"]
def inside(s, pt, r=0.35):
    """Robust: does a small sphere at pt overlap the solid?"""
    ball = cq.Solid.makeSphere(r, Vector(*map(float, pt)))
    try:
        return s.intersect(ball).Volume() > 1e-4
    except Exception:
        return s.isInside(Vector(*map(float, pt)), 0.01)
ok = lambda c, msg: print(("PASS " if c else "FAIL ") + msg)
# sanity: the boolean probe must see solid where the palm is obviously solid, or every "open channel" pass is meaningless
ok(inside(palm, (50.0, 0.0, -18.0)) and inside(car, tuple(G["thumb"]["M"] + G["thumb"]["m"] * -15.0)) and not inside(palm, (50.0, 0.0, 30.0)),
   "probe sanity: solid reads as solid, air reads as air")
# 1. lanes vs S6 pocket: every lane centreline point is in air and >= 2.2 mm from the servo envelope
env = HC.s6_box(C, 0.2).val()
for fi, f in enumerate(G["fingers"]):
    ex = f["B"] + f["U"] * (-HC.F.BASE_L) + (-f["N"]) * HC.F.FLEX_Y
    pts = [ex - f["U"] * 6, np.array([ex[0] - 18, 0.75 * f["B"][1] + 0.25 * HC.LANES[fi], HC.LANE_Z])] + ([np.array([50.0, HC.LANES[fi], HC.LANE_Z])] if fi == 0 else []) + [np.array([0.5, HC.LANES[fi], HC.LANE_Z])]
    samp = [a + (b - a) * t for a, b in zip(pts[:-1], pts[1:]) for t in np.linspace(0, 1, 10)]
    bad = [q.round(1).tolist() for q in samp if inside(palm, q)]
    air = not bad
    if bad: print("   blocked at", bad[:4])
    dmin = min(env.distance(cq.Vertex.makeVertex(*map(float, q))) for q in samp[::3])
    roof = min(abs(-22.0 - q[2]) for q in samp)  # distance to back face
    ok(air and dmin > 2.2, f"finger {fi} tendon channel open end to end, {dmin:.1f} mm from the S6 pocket")
# 2. thumb tendon: along the axis through the wall and the hub, then to the arm channel
axis = [np.array([x, C[1], C[2]]) for x in np.linspace(-0.5, 7.5, 12)]
ok(all(not inside(palm, q) for q in axis[:9]), "thumb tendon passes through the pivot wall on the axis")
ok(all(not inside(car, q) for q in axis), "thumb tendon passes along the hub axis")
th = G["thumb"]; a_start = th["M"] + th["m"] * (-(th["Lm"] - 2.0)) + (-th["n"]) * HC.F.FLEX_Y
conn = [np.array([8.0, C[1], C[2]]) + (a_start - np.array([8.0, C[1], C[2]])) * t for t in np.linspace(0, 1, 12)]
bad = [q.round(1).tolist() for q in conn if inside(car, q)]
ok(not bad, "thumb tendon connector channel is open inside the carrier" + (f" (blocked at {bad[:3]})" if bad else ""))
arm_end = th["M"] + th["m"] * (-HC.F.BASE_L + 1) + (-th["n"]) * HC.F.FLEX_Y
armc = [a_start + (arm_end - a_start) * t for t in np.linspace(0, 1, 10)]
ok(all(not inside(car, q) for q in armc), "thumb flexor channel open up the arm")
# 3. carrier vs servo through the swing
vols = []
for beta in np.arange(0, 41, 5):
    rot = cq.Location(Vector(*C)) * cq.Location(Vector(), Vector(1, 0, 0), float(beta)) * cq.Location(Vector(*(-C)))
    vols.append(car.moved(rot).intersect(HC.s6_box(C, 0.0).val()).Volume())
ok(max(vols) < 0.5, f"carrier never enters the S6 envelope (max overlap {max(vols):.2f} mm3)")
# 4. pivot wall: solid all round the bore at x = 2
ring = [np.array([2.0, C[1] + 3.5 * np.cos(a), C[2] + 3.5 * np.sin(a)]) for a in np.linspace(0, 2 * np.pi, 16, endpoint=False)]
ok(all(inside(palm, q) for q in ring), "pivot wall solid around the bore (x 0-4)")
# 5. S6 tabs enclosed by palm material
yc = C[1] - 5.9
tabs = [np.array([29.4, yc + s * (HC.S6_TAB_SPAN / 2 + 1.2), C[2]]) for s in (-1, 1)]
ok(all(inside(palm, q) for q in tabs), "palm material beyond both S6 tab ends")
# 7. single solids
for k, v in parts.items():
    n = len(v.solids().vals())
    if n != 1:
        print("FAIL", k, "has", n, "solids")
print("PASS every part is a single solid" if all(len(v.solids().vals()) == 1 for v in parts.values()) else "")
# 8. outer socket walls
sp = HC.W + p["gap"]
for s in (1, -1):
    q = np.array([85.0, s * (1.5 * sp + HC.W / 2 + 0.2 + 1.2), -7.5])
    ok(inside(palm, q), f"outer socket wall >= 1.2 mm on the {'thumb' if s > 0 else 'little'} side")
# 9. S6 is covered on the palm side (its top sits ~2 mm above the palm face): sample 0.6 mm above the
#    servo top over the body and both tabs; every sample must be inside palm material
yc6 = C[1] - 5.9; zt = C[2] + HC.S6_T / 2 + 0.2 + 0.6
pts = [(x, y) for x in np.linspace(HC.S6_FACE + 0.5, HC.S6_FACE + HC.S6_LEN - 0.5, 6) for y in np.linspace(yc6 - HC.S6_W / 2 + 0.5, yc6 + HC.S6_W / 2 - 0.5, 6)]
pts += [(x, yc6 + s * (HC.S6_TAB_SPAN / 2 - 0.5)) for x in np.linspace(HC.S6_TAB_X[0] + 0.3, HC.S6_TAB_X[1] - 0.3, 3) for s in (1, -1)]
# the strip next to the horn notch is open on purpose (x < S6_FACE + 0.5, toward the thumb)
miss = [q for q in pts if not inside(palm, (q[0], q[1], zt), r=0.25)]
ok(not miss, f"S6 covered on the palm side ({len(pts) - len(miss)}/{len(pts)} samples under palm material){' missing ' + str(miss[:4]) if miss else ''}")
