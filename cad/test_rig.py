"""One-finger test rig: holds one finger base (2 x M2 x 16 flat-head screws from below), one MG90S and one spool_32, so a single finger can be
driven by its servo before printing the whole hand.  Run:  python test_rig.py
Rig axes: x along the finger (knuckle pin at x = 0), z up. The finger lies back-face down and curls up.
The finger tendon leaves the base and runs straight back, tangent to the spool drum."""
import math, os
import cadquery as cq
from cadquery import Solid, Vector
import finger_revB as F
import hand_cad as HC

R, W = F.R, F.W
PLATE_Z0, PLATE_T = -29.0, 5.0
TOP = PLATE_Z0 + PLATE_T
DRUM_Z = 10.5                        # estimated drum centre height for an MG90S + round horn + spool_32
BLOCK_TOP = DRUM_Z - (R - F.FLEX_Y)  # so the tendon leaves the base at drum height (back face 13 mm below it)
DRUM_R, FLANGE_R = 15.8, 18.3
SHAFT = (-48.0, -DRUM_R)             # drum tangent to the tendon line (y = 0)
SHAFT_OFF = 5.9                      # MG90S shaft is 5.9 mm from the body centre
SC = (SHAFT[0] - SHAFT_OFF, SHAFT[1])  # servo body centre
BLOCK_X = (-F.BASE_L - 0.4, -9.0)     # stops 9 mm behind the pin, like the palm front


def wp(s): return cq.Workplane("XY").add(s)


def rig():
    x0, x1, y0, y1 = SC[0] - 21.0, BLOCK_X[1], SC[1] - 10.5, W / 2 + 2.0
    plate = cq.Workplane("XY").box(x1 - x0, y1 - y0, PLATE_T, centered=False).translate((x0, y0, PLATE_Z0)).edges("|Z").fillet(5)
    # MG90S frame: the servo hangs by its tabs on the frame top, so its exact body height doesn't matter.
    # The opening runs through the plate, and slots in both end walls (open at the bottom) take the cable.
    sx, sy = SC
    frame = cq.Workplane("XY").box(36, 16, 15.5, centered=False).translate((sx - 18, sy - 8, TOP))
    for xx in (-13.8, 13.8):
        frame = frame.cut(wp(Solid.makeCylinder(0.9, 8, Vector(sx + xx, sy, TOP + 10))))
    block = cq.Workplane("XY").box(BLOCK_X[1] - BLOCK_X[0], W + 2 * 1.45, BLOCK_TOP + 3.0 - TOP, centered=False).translate((BLOCK_X[0], -W / 2 - 1.45, TOP))
    block = block.edges("|Z").fillet(1.2)
    body = plate.union(frame).union(block)
    body = body.cut(cq.Workplane("XY").box(23.4, 12.8, 30, centered=False).translate((sx - 11.7, sy - 6.4, PLATE_Z0 - 1)))
    for s in (1, -1):
        body = body.cut(cq.Workplane("XY").box(8, 9.0, TOP + 7.0 - PLATE_Z0 + 1, centered=False).translate((sx + s * 14.0 - 4, sy - 4.5, PLATE_Z0 - 1)))
    # finger seat: 3 mm deep pocket so the base can't twist (open at the knuckle end), 2 x M3 x 16 from below
    body = body.cut(cq.Workplane("XY").box(BLOCK_X[1] - BLOCK_X[0] + 2, W + 0.5, 5, centered=False).translate((BLOCK_X[0] - 0.01, -W / 2 - 0.25, BLOCK_TOP)))
    # gap in the seat walls at the base's knuckle-elastic cross pin, so the pin ends can stick out
    body = body.cut(cq.Workplane("XY").box(3.0, W + 6, 5, centered=False).translate((-F.BASE_L + 5.0 - 1.5, -W / 2 - 3, BLOCK_TOP)))
    hx = -F.BASE_L / 2 - 2
    for yy in (-5.0, 5.0):
        # M2 x 16 flat-head screw from below: screwdriver pocket, countersunk seat, clearance hole
        top = BLOCK_TOP - 8.0
        body = body.cut(wp(Solid.makeCylinder(1.2, 40, Vector(hx, yy, PLATE_Z0 - 1))))
        body = body.cut(wp(Solid.makeCylinder(2.5, top - PLATE_Z0 + 1, Vector(hx, yy, PLATE_Z0 - 1))))
        body = body.cut(wp(Solid.makeCone(2.2, 1.2, 1.0, Vector(hx, yy, top - 0.01), Vector(0, 0, 1))))
    return body


def to_rig(shape):
    """finger frame (X along, Y back(+)/palm(-), Z across) -> rig frame: (X, Y, Z) -> (X, Z, BLOCK_TOP + R - Y)"""
    loc = cq.Location(Vector(0, 0, BLOCK_TOP + R)) * cq.Location(Vector(), Vector(1, 0, 0), -90)
    return shape.moved(loc)


def servo_env():
    sx, sy = SC
    body = Solid.makeBox(22.8, 12.2, 22.5, Vector(sx - 11.4, sy - 6.1, TOP + 15.5 - 16.0))   # tab underside ~16 mm up
    tabs = Solid.makeBox(32.5, 12.2, 2.5, Vector(sx - 16.25, sy - 6.1, TOP + 15.5))
    spool = Solid.makeCylinder(FLANGE_R, 11.4, Vector(SHAFT[0], SHAFT[1], DRUM_Z - 6.7 - 0.3))
    return body.fuse(tabs), spool


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    r = rig()
    print("rig valid", r.val().isValid(), "solids", len(r.solids().vals()), "vol %.1f cm3" % (r.val().Volume() / 1000))
    # fit checks
    m3 = F.M3; F.M3 = 1.6; base = F.base(); F.M3 = m3
    import json
    Lf = HC.hand_geometry(json.load(open(os.path.join(here, "hand_params.json"))))["fingers"][0]["L"]
    std = HC.finger_parts(Lf)
    serv, spl = servo_env()
    ok = lambda c, m: print(("PASS " if c else "FAIL ") + m)
    probe = lambda pt: r.val().intersect(Solid.makeSphere(0.3, Vector(*pt))).Volume() > 1e-4
    ok(probe((-15, 0, TOP - 2)) and not probe((-15, 0, BLOCK_TOP + 5)), "probe sanity")
    ok(r.val().intersect(serv).Volume() < 0.5, "MG90S body + tabs fit the frame")
    base_r = to_rig(base.val())
    ok(r.val().intersect(base_r).Volume() < 0.5, "finger base sits on the block without overlap")
    ok(spl.intersect(base_r).Volume() < 0.01 and spl.intersect(r.val()).Volume() < 0.01, "spool clears the block and the base")
    for curl in ((0, 0, 0), (45, 45, 30), (90, 95, 80)):
        loc = cq.Location(); worst = 0.0
        x = {"proximal": 0.0, "middle": Lf[0], "distal": Lf[1]}   # each pin from the previous one
        for name, a in zip(("proximal", "middle", "distal"), curl):
            loc = loc * cq.Location(Vector(x[name], 0, 0)) * cq.Location(Vector(), Vector(0, 0, 1), -a)
            seg = to_rig(std[name].val().moved(loc))
            worst = max(worst, seg.intersect(r.val()).Volume(), seg.intersect(spl).Volume())
        ok(worst < 0.5, f"finger at curl {curl} clears the rig and spool (max overlap {worst:.2f} mm3)")
    # tendon: from the base exit straight back to the drum tangent point, open air all the way
    zt = BLOCK_TOP + R - F.FLEX_Y
    seg = Solid.makeCylinder(0.4, -F.BASE_L - 0.6 - SHAFT[0], Vector(SHAFT[0], 0, zt), Vector(1, 0, 0))
    ok(seg.intersect(r.val()).Volume() < 1e-3, f"tendon line clear from base to drum (height {zt:.1f} mm, drum centre est. {DRUM_Z})")
    os.makedirs(os.path.join(here, "parts"), exist_ok=True)
    cq.exporters.export(r, os.path.join(here, "parts", "finger_test_rig.stl"), tolerance=0.02, angularTolerance=0.1)
    cq.exporters.export(r, os.path.join(here, "parts", "finger_test_rig.step"))
    # washers for a stand printed before the M2 switch (its holes were sized for M3): one under each screw head
    w = wp(Solid.makeCylinder(3.0, 1.2, Vector(0, 0, 0))).cut(wp(Solid.makeCylinder(1.15, 3, Vector(0, 0, -1))))
    cq.exporters.export(w, os.path.join(here, "parts", "rig_washer.stl"), tolerance=0.02, angularTolerance=0.1)
    print("exported finger_test_rig + rig_washer")
