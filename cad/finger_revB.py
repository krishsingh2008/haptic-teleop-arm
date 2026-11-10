"""
Haptic Teleop Hand - tendon-driven robot finger, rev B
Parametric CadQuery model. Run:  python finger_revB.py
Writes STEP (Fusion 360) and STL (print) files next to this script.

What changed from rev A (and why):
  * Rev A ran the tendon through the hinge tongue and out BEHIND the pin, so past ~8 deg of bend the
    tendon straightened the joint instead of curling it. Rev B keeps the flexor tendon 5.5 mm on the
    palm side of every pin, in a straight channel that crosses each gap through a groove under the
    tongue. Moment arm: 5.5 mm straight, growing to ~10 mm at 90 deg (it bowstrings across the gap).
  * Separate elastic per knuckle (tied to filament-pin anchors) instead of one long cord, so each
    joint's spring can be tuned. The same back channel also takes an extensor tendon if you build
    the push-pull version.
  * Tendon ends on a cross-pin in the fingertip, clear of the FSR pocket.
  * Every channel mouth has a 1 mm chamfer, so the line bends over a rounded edge, not a sharp one.

Axes (every part): X along the finger, Y dorsal(+)/ventral(-), Z across. Pins run along Z.
Print every part dorsal face DOWN. No supports. Pins: 1.75 mm filament.

STEP 1: print coupon.stl and set PIN_TIGHT, PIN_FREE and SLOT_CLR from it before printing these parts.
"""
import os
import cadquery as cq
from cadquery import Solid, Vector

# ---------- set these from the tolerance coupon ----------
PIN_TIGHT = 1.90   # your coupon: snug hole 4 (Bambu A1, PLA)
PIN_FREE = 2.15    # your coupon: free hole 2
SLOT_CLR = 0.15    # your coupon: slot 1 (7.2 mm) fits; 0.05 mm margin over (7.2 - 7.0) / 2

# ---------- geometry (mm) ----------
W, D = 17.0, 15.0
R = D / 2
TONGUE = 7.0
GAP = 9.5
STOP_CLR = 0.3
FLEX_Y, FLEX_D = -5.5, 2.0      # flexor tendon channel (palm side)
EXT_Y, EXT_D = 5.0, 3.0         # elastic / extensor channel (back side): fits 2-2.5 mm round elastic
GROOVE_W = 2.8                  # width of the palm-side groove under each tongue
BACK_GROOVE_W = 3.6             # width of the back-side groove (elastic cord)
DORSAL_FLOOR = 3.5              # groove floor radius; the cord centreline wraps at ~4.75 mm
PROXIMAL_L, MIDDLE_L, DISTAL_L = 40.0, 25.0, 20.0
BASE_L = 22.0
M3 = 3.3
FSR_D, FSR_X = 8.2, 15.0        # FSR 400 (7.6 mm active area) pocket on the fingertip
SLOT_W = TONGUE + 2 * SLOT_CLR


def wp(s): return cq.Workplane("XY").add(s)
def box(x0, x1, y0, y1, z0=-W / 2, z1=W / 2): return wp(Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, Vector(x0, y0, z0)))
def cyl_z(r, x, y, length): return wp(Solid.makeCylinder(r, length, Vector(x, y, -length / 2), Vector(0, 0, 1)))
def cyl_x(d, y, x0, x1): return wp(Solid.makeCylinder(d / 2, x1 - x0, Vector(x0, y, 0), Vector(1, 0, 0)))
def cyl_y(r, x, z, y0, y1): return wp(Solid.makeCylinder(r, y1 - y0, Vector(x, y0, z), Vector(0, 1, 0)))


def tongue():
    t = cyl_z(R, 0, 0, TONGUE).union(box(0, GAP + 0.5, -R, R, -TONGUE / 2, TONGUE / 2))
    t = t.cut(cyl_z(PIN_FREE / 2, 0, 0, TONGUE + 2))
    g = GROOVE_W / 2
    # palm-side groove: the flexor crosses under the pin without touching the tongue
    t = t.cut(box(-R - 1, GAP + 1, -R - 1, FLEX_Y + FLEX_D / 2 + 0.3, -g, g))
    # back-side groove: the cord wraps a 4.5 mm radius floor over the knuckle
    gb = BACK_GROOVE_W / 2
    back = box(-R - 1, 0, 0, R + 1, -gb, gb).cut(cyl_z(DORSAL_FLOOR, 0, 0, W + 2)).union(box(0, GAP + 1, DORSAL_FLOOR, R + 1, -gb, gb))
    return t.cut(back)


def clevis(xp):
    solid = box(xp - R - 1, xp, -R, R).union(cyl_z(R, xp, 0, W)).union(box(xp, xp + GAP - STOP_CLR, 0, R))
    slot = box(xp - R - 0.6, xp + GAP + 1, -R - 1, R + 1, -SLOT_W / 2, SLOT_W / 2)
    return solid, slot


def channels(part, x0, x1):
    return part.cut(cyl_x(FLEX_D, FLEX_Y, x0, x1)).cut(cyl_x(EXT_D, EXT_Y, x0, x1))


CHAMFER = 1.0   # rounds every channel mouth so the line never bends over a sharp edge


def mouth(part, x_face, y, d, opens_toward):
    """Chamfer a channel mouth on a face at x = x_face. opens_toward = -1 (face looks -x) or +1."""
    r_in, r_out = d / 2, d / 2 + CHAMFER
    if opens_toward > 0:   # face looks toward +x: cone widens as x increases
        cone = Solid.makeCone(r_in, r_out + 0.2, CHAMFER + 0.2, Vector(x_face - CHAMFER, y, 0), Vector(1, 0, 0))
    else:
        cone = Solid.makeCone(r_in, r_out + 0.2, CHAMFER + 0.2, Vector(x_face + CHAMFER, y, 0), Vector(-1, 0, 0))
    return part.cut(wp(cone))


def mouths(part, x_face, opens_toward):
    return mouth(mouth(part, x_face, FLEX_Y, FLEX_D, opens_toward), x_face, EXT_Y, EXT_D, opens_toward)


def cross_pin(part, x, y):
    return part.cut(cyl_z(PIN_TIGHT / 2, x, y, W + 2))


def phalanx(L, anchors):
    part = tongue().union(box(GAP, L - R - 1, -R, R))
    solid, slot = clevis(L)
    part = part.union(solid).cut(slot).cut(cyl_z(PIN_TIGHT / 2, L, 0, W + 2))
    part = channels(part, -R - 2, L + GAP + 2)
    part = mouths(mouths(part, GAP, -1), L - R - 0.6, +1)
    for x in anchors:
        part = cross_pin(part, x, EXT_Y)
    return part


def distal(L):
    body = box(GAP, L, -R, R)
    body = body.edges(cq.selectors.BoxSelector((L - 0.1, -R - 1, -W), (L + 0.1, R + 1, W))).fillet(4.0)
    part = tongue().union(body)
    part = part.cut(cyl_x(FLEX_D, FLEX_Y, -R - 2, 13.5)).cut(cyl_x(EXT_D, EXT_Y, -R - 2, 13.5))
    part = mouths(part, GAP, -1)
    part = cross_pin(part, 12.0, FLEX_Y)          # flexor tendon ties off here
    part = cross_pin(part, 12.0, EXT_Y)           # DIP elastic (or extensor) ties off here
    return part.cut(cyl_y(FSR_D / 2, FSR_X, 0, -R - 1, -R + 0.6))


def base():
    solid, slot = clevis(0)
    part = box(-BASE_L, 0, -R, R).union(solid).cut(slot).cut(cyl_z(PIN_TIGHT / 2, 0, 0, W + 2))
    part = channels(part, -BASE_L - 1, GAP + 2)
    part = mouths(mouths(part, -R - 0.6, +1), -BASE_L, -1)
    part = cross_pin(part, -BASE_L + 5.0, EXT_Y)  # MCP elastic anchor
    for z in (-5.0, 5.0):
        part = part.cut(cyl_y(M3 / 2, -BASE_L / 2 - 2, z, -R - 1, R + 1))
    return part


PARTS = {
    "base": base(),
    "proximal": phalanx(PROXIMAL_L, anchors=[GAP + 5.0, PROXIMAL_L - R - 6.0]),
    "middle": phalanx(MIDDLE_L, anchors=[13.0]),
    "distal": distal(DISTAL_L),
}
PIN_X = {"proximal": 0.0, "middle": PROXIMAL_L, "distal": MIDDLE_L}
# tendon geometry for the simulator: exit at the start of each clevis slot, entry at the next body
FLEX_P = (-(R + 0.6), FLEX_Y)
FLEX_Q = (GAP, FLEX_Y)


def posed(curl=(0, 0, 0)):
    out = {"base": PARTS["base"].val()}
    loc = cq.Location()
    for name, a in zip(("proximal", "middle", "distal"), curl):
        loc = loc * cq.Location(Vector(PIN_X[name], 0, 0)) * cq.Location(Vector(), Vector(0, 0, 1), -a)
        out[name] = PARTS[name].val().moved(loc)
    return out


if __name__ == "__main__":
    here = os.path.dirname(os.path.abspath(__file__))
    for name, p in PARTS.items():
        cq.exporters.export(p, os.path.join(here, f"revB_{name}.stl"), tolerance=0.02, angularTolerance=0.1)
        cq.exporters.export(p, os.path.join(here, f"revB_{name}.step"))
    asm = cq.Assembly(name="robot_finger_revB")
    for name, s in posed().items():
        asm.add(cq.Workplane().add(s), name=name, color=cq.Color(0.96, 0.77, 0.33))
    asm.save(os.path.join(here, "revB_assembly.step"))
    print("exported to", here)
