"""Interference check: moving parts vs the palm and each other in several poses."""
import sys, json, itertools
import cadquery as cq
import hand_cad as HC

p, G, parts = HC.build(sys.argv[1] if len(sys.argv) > 1 else "hand_params.json")
poses = {"open": ([[0, 0, 0]] * 4 + [[0, 0]], 0.0), "half": ([[45, 45, 30]] * 4 + [[45, 40]], 20.0),
         "fist": ([[90, 95, 80]] * 4 + [[0, 0]], 0.0), "thumb_curl": ([[0, 0, 0]] * 4 + [[60, 50]], 40.0),
         "thumb_swing": ([[0, 0, 0]] * 4 + [[0, 0]], 40.0)}
def vol(a, b):
    try:
        return a.intersect(b).Volume()
    except Exception:
        return -1
for name, (curls, beta) in poses.items():
    A = HC.assembly(G, parts, curls, beta)
    A["servo_S6"] = HC.s6_box(G["thumb"]["C"]).val()
    keys = list(A)
    bad = []
    for a, b in itertools.combinations(keys, 2):
        if a == b: continue
        ba, bb = A[a].BoundingBox(), A[b].BoundingBox()
        if ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin:
            continue
        v = vol(A[a], A[b])
        if v > 1.0:
            bad.append((a, b, round(v, 1)))
    print(name, "overlaps >1 mm3:", bad if bad else "none")
