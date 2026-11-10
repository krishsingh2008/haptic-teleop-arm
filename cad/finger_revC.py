"""
Haptic Teleop Hand - tendon-driven robot finger, rev C
Run:  python finger_revC.py      (writes revC_*.stl / .step next to this file)

What changed from rev B (and why):
  * Screw-tuned knuckle elastics. Every knuckle uses the SAME 1 mm round elastic cord. One end of
    each cord is a knot dropped into a keyhole on the back; the other end is a loop over an M2 x 10
    self-tapping screw standing in an open pocket on the back. Turning the screw clockwise winds
    cord onto it (about 1.2 mm per 1/8 turn) and the thread friction holds it. Won a 10-design
    battle (elastic_battle.py): self-locking, screwdriver from the back of the hand, no threading.
  * Open back groove instead of a closed elastic channel: lay the cord in, nothing to thread.
  * No anchor pins at all for the elastics (5 fewer pins than rev B).
  * Tendon moment arm shrinks from base to tip: 5.5 mm at the base knuckle, 4.5 mm at the middle,
    3.5 mm at the tip. Same line pull gives the base the most torque, so it leads the closing
    (Leddy & Dollar 2019 recommend a proximal arm about 2x the distal one).
  * The fishing line exits straight out of the fingertip into a knot pocket, so it threads from
    the tip in one straight push and locks with a knot. No tip cross-pin.
  * Fits tuned for the Bambu A1 + PLA coupon results (snug 1.90, free 2.15, slot +0.15 per side).

Axes (every part): X along the finger, Y back(+)/palm(-), Z across. Pins run along Z.
Print every part back face DOWN. No supports.
"""
import os, math
import cadquery as cq
from cadquery import Solid, Vector

PIN_TIGHT, PIN_FREE, SLOT_CLR = 1.90, 2.15, 0.15

W, D = 17.0, 15.0
R = D / 2
TONGUE = 7.0
GAP = 9.5
STOP_CLR = 0.3
FLEX_D = 2.0
ARM = (5.5, 4.5, 3.5)            # tendon moment arm at base, middle, tip knuckle (mm from the pin)
GROOVE_W = 2.8                   # palm-side groove under each tongue (fishing line)
BACK_GROOVE_W = 2.6              # back-side groove over each knuckle (elastic cord)
DORSAL_FLOOR = 3.5               # cord wraps each knuckle on this radius (+ half the cord)
EL_W, EL_FLOOR = 1.8, 3.6        # open back groove for the 1 mm cord along each segment
KEY_R, KEY_FLOOR = 1.4, 0.8      # knot keyhole: 2.8 mm hole down to Y = 0.8; the 1.8 mm groove stops the knot
CAP_R, CAP_FLOOR = 3.1, 0.6      # capstan pocket (6.2 mm) down to Y = 0.6
CAP_Z = 3.2                      # pocket sits off-centre so its screw clears the fishing line
PILOT_R, PILOT_BOTTOM = 0.8, -4.3  # 1.6 mm pilot for an M2 self-tapping screw
TIP_KNOT_R, TIP_KNOT_DEPTH = 1.6, 2.5
PROXIMAL_L, MIDDLE_L, DISTAL_L = 40.0, 25.0, 20.0
BASE_L = 22.0
M3 = 1.6                         # base screw pilots: M2 x 16 flat-head self-tappers from the palm or stand
FSR_D, FSR_X = 8.2, 15.0
SLOT_W = TONGUE + 2 * SLOT_CLR
CHAMFER = 1.0


def wp(s): return cq.Workplane("XY").add(s)
def box(x0, x1, y0, y1, z0=-W / 2, z1=W / 2): return wp(Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0)))
def cyl_z(r, x, y, length): return wp(Solid.makeCylinder(r, length, Vector(x, y, -length / 2), Vector(0, 0, 1)))
def cyl_y(r, x, z, y0, y1): return wp(Solid.makeCylinder(r, y1 - y0, Vector(x, y0, z), Vector(0, 1, 0)))


def cyl_line(d, p0, p1, z=0.0):
    """Cylinder of diameter d from (x0, y0) to (x1, y1) in the z plane."""
    v = Vector(p1[0] - p0[0], p1[1] - p0[1], 0)
    return wp(Solid.makeCylinder(d / 2, v.Length, Vector(p0[0], p0[1], z), v.normalized()))


def line_y(p0, p1, x):
    t = (x - p0[0]) / (p1[0] - p0[0]) if p1[0] != p0[0] else 0.0
    return p0[1] + t * (p1[1] - p0[1])


def mouth(part, x_face, y, opens_toward):
    r_in, r_out = FLEX_D / 2, FLEX_D / 2 + CHAMFER
    if opens_toward > 0:
        cone = Solid.makeCone(r_in, r_out + 0.2, CHAMFER + 0.2, Vector(x_face - CHAMFER, y, 0), Vector(1, 0, 0))
    else:
        cone = Solid.makeCone(r_in, r_out + 0.2, CHAMFER + 0.2, Vector(x_face + CHAMFER, y, 0), Vector(-1, 0, 0))
    return part.cut(wp(cone))


def tongue(arm):
    """Rear hinge tongue of a segment; arm = tendon moment arm of the knuckle it forms."""
    t = cyl_z(R, 0, 0, TONGUE).union(box(0, GAP + 0.5, -R, R, -TONGUE / 2, TONGUE / 2))
    t = t.cut(cyl_z(PIN_FREE / 2, 0, 0, TONGUE + 2))
    g = GROOVE_W / 2
    t = t.cut(box(-R - 1, GAP + 1, -R - 1, -arm + FLEX_D / 2 + 0.3, -g, g))
    gb = BACK_GROOVE_W / 2
    back = box(-R - 1, 0, 0, R + 1, -gb, gb).cut(cyl_z(DORSAL_FLOOR, 0, 0, W + 2)).union(box(0, GAP + 1, DORSAL_FLOOR, R + 1, -gb, gb))
    return t.cut(back)


def clevis(xp):
    solid = box(xp - R - 1, xp, -R, R).union(cyl_z(R, xp, 0, W)).union(box(xp, xp + GAP - STOP_CLR, 0, R))
    slot = box(xp - R - 0.6, xp + GAP + 1, -R - 1, R + 1, -SLOT_W / 2, SLOT_W / 2)
    return solid, slot


def back_groove(part, x0, x1):
    return part.cut(box(x0, x1, EL_FLOOR, R + 1, -EL_W / 2, EL_W / 2))


def keyhole(part, x):
    return part.cut(cyl_y(KEY_R, x, 0, KEY_FLOOR, R + 1))


def capstan(part, x, side=+1):
    z = side * CAP_Z
    return part.cut(cyl_y(CAP_R, x, z, CAP_FLOOR, R + 1)).cut(cyl_y(PILOT_R, x, z, PILOT_BOTTOM, CAP_FLOOR + 0.01))


def base():
    solid, slot = clevis(0)
    part = box(-BASE_L, 0, -R, R).union(solid).cut(slot).cut(cyl_z(PIN_TIGHT / 2, 0, 0, W + 2))
    p0, p1 = (-BASE_L - 1, -ARM[0]), (GAP + 2, -ARM[0])
    part = part.cut(cyl_line(FLEX_D, p0, p1))
    part = mouth(mouth(part, -R - 0.6, -ARM[0], +1), -BASE_L, -ARM[0], -1)
    part = back_groove(part, -BASE_L + 2.5, GAP + 2)
    part = keyhole(part, -BASE_L + 4.0)                    # base-knuckle cord: knot end
    for z in (-5.0, 5.0):
        part = part.cut(cyl_y(M3 / 2, -BASE_L / 2 - 2, z, -R - 1, R + 1))
    return part


def phalanx(L, arm_rear, arm_front, features):
    """features: list of ('cap', x, side) or ('key', x)."""
    part = tongue(arm_rear).union(box(GAP, L - R - 1, -R, R))
    solid, slot = clevis(L)
    part = part.union(solid).cut(slot).cut(cyl_z(PIN_TIGHT / 2, L, 0, W + 2))
    p0, p1 = (0.0, -arm_rear), (L, -arm_front)
    a, b = (-R - 2, line_y(p0, p1, -R - 2)), (L + GAP + 2, line_y(p0, p1, L + GAP + 2))
    part = part.cut(cyl_line(FLEX_D, a, b))
    part = mouth(part, GAP, line_y(p0, p1, GAP), -1)
    part = mouth(part, L - R - 0.6, line_y(p0, p1, L - R - 0.6), +1)
    part = back_groove(part, GAP - 1, L + GAP + 2)
    for f in features:
        part = capstan(part, f[1], f[2]) if f[0] == "cap" else keyhole(part, f[1])
    return part


def distal(L, arm, cap_x):
    body = box(GAP, L, -R, R)
    body = body.edges(cq.selectors.BoxSelector((L - 0.1, -R - 1, -W), (L + 0.1, R + 1, W))).fillet(4.0)
    part = tongue(arm).union(body)
    part = part.cut(cyl_line(FLEX_D, (-R - 2, -arm), (L + 1, -arm)))          # straight out of the tip
    part = mouth(part, GAP, -arm, -1)
    part = part.cut(wp(Solid.makeCylinder(TIP_KNOT_R, TIP_KNOT_DEPTH + 1, Vector(L - TIP_KNOT_DEPTH, -arm, 0), Vector(1, 0, 0))))
    part = back_groove(part, GAP - 1, cap_x + 1)
    part = capstan(part, cap_x, +1)                        # tip-knuckle cord: screw end
    return part.cut(cyl_y(FSR_D / 2, FSR_X, 0, -R - 1, -R + 0.6))


def build(L=(PROXIMAL_L, MIDDLE_L, DISTAL_L)):
    L0, L1, L2 = L
    return {
        "base": base(),
        # base-knuckle cord screw at the back of the long segment; middle-knuckle cord screw at its front
        "proximal": phalanx(L0, ARM[0], ARM[1], [("cap", GAP + 5.5, +1), ("cap", L0 - R - 7.0, -1)]),
        # middle segment: knot ends of the middle-knuckle cord (rear) and of the tip-knuckle cord (front)
        "middle": phalanx(L1, ARM[1], ARM[2], [("key", GAP + 2.5), ("key", L1 - R - 3.5)]),
        "distal": distal(L2, ARM[2], GAP + 6.5),
    }


# elastic cord path, for the simulator and the 3D guide: (part, x, z) of each end, per knuckle
def cord_ends(L=(PROXIMAL_L, MIDDLE_L, DISTAL_L)):
    L0, L1, L2 = L
    return [(("base", -BASE_L + 4.0, 0.0), ("proximal", GAP + 5.5, CAP_Z)),
            (("proximal", L0 - R - 7.0, -CAP_Z), ("middle", GAP + 2.5, 0.0)),
            (("middle", L1 - R - 3.5, 0.0), ("distal", GAP + 6.5, CAP_Z))]


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    for name, p in build().items():
        v = p.val()
        print(name, "valid", v.isValid(), "solids", len(p.solids().vals()), "vol %.2f cm3" % (v.Volume() / 1000))
        cq.exporters.export(p, os.path.join(here, f"revC_{name}.stl"), tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(p, os.path.join(here, f"revC_{name}.step"))
