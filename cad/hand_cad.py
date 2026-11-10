"""
Haptic Teleop Hand - full hand CAD (rev B fingers, 3+1 layout, 2-servo thumb)
Parametric CadQuery model. Run:  python hand_cad.py hand_params.json
Writes one STL + STEP per printed part, plus hand_assembly.step, into ./parts/.

Frames
  Hand:   x toward the fingers, y toward the thumb, z out of the palm (palm face is z = 0).
  Finger: rev B convention - X along the finger, Y dorsal(+)/ventral(-), Z across; pins along Z.
  A finger whose knuckle pin is at B, straight direction U and curl direction N sits at
  X = U, Y = -N, Z = -(U x N), origin B.

Printed parts
  palm              palm with the U-shaped cup ridges, four finger sockets, internal tendon channels, thumb-servo pocket
  finger_base x5    rev B base (4 fingers + spare), screws into the palm with 2 x M2 x 16 flat-head self-tappers
  proximal/middle/distal  3 identical fingers (42.7/28.1/23.7 mm) + little (37.0/24.4/20.6 mm)
  thumb_carrier     hub on the S6 servo + 38 mm metacarpal + thumb knuckle clevis
  thumb_proximal / thumb_distal  (45.3 / 28.8 mm)
  forearm_tray      holds S1-S5 in three staggered rows, with the tendon guide comb
  spool_32 x4, spool_20 x1   finger and thumb tendon spools for MG90S round horns
"""
import os, sys, json, math
import numpy as np
import cadquery as cq
from cadquery import Solid, Vector
from OCP.gp import gp_Trsf

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE); sys.path.insert(0, os.path.join(HERE, "..", "finger-revB"))
import finger_revB as F          # rev B part generators (set PIN_TIGHT / PIN_FREE / SLOT_CLR there)

R, W, D = F.R, F.W, F.D
M3, M2 = 3.3, 2.2
# every screw in the hand is an M2 flat-head self-tapping screw (one 800-piece kit)
M2_PILOT, M2_CLEAR, M2_CSK = 1.6, 2.4, 4.4   # pilot to thread into, clearance hole, countersink diameter


def wp(s): return cq.Workplane("XY").add(s)


def frame_loc(B, U, N):
    """Location that maps the finger frame onto the hand frame."""
    U = np.asarray(U, float); N = np.asarray(N, float)
    X = U / np.linalg.norm(U); Y = -N / np.linalg.norm(N); Z = np.cross(X, Y)
    t = gp_Trsf()
    t.SetValues(X[0], Y[0], Z[0], B[0], X[1], Y[1], Z[1], B[1], X[2], Y[2], Z[2], B[2])
    return cq.Location(t)


def placed(shape, loc):
    return shape.val().moved(loc) if isinstance(shape, cq.Workplane) else shape.moved(loc)


def rod(a, b, r):
    """Solid cylinder from a to b (plus ball ends) - used to cut smooth tendon channels."""
    a = np.asarray(a, float); b = np.asarray(b, float); d = b - a; L = float(np.linalg.norm(d))
    s = wp(Solid.makeCylinder(r, L, Vector(*a), Vector(*(d / L))))
    return s.union(wp(Solid.makeSphere(r, Vector(*a)))).union(wp(Solid.makeSphere(r, Vector(*b))))


def path_cut(part, pts, r):
    for a, b in zip(pts[:-1], pts[1:]):
        part = part.cut(rod(a, b, r))
    return part


# ----------------------------------------------------------------------------- geometry from the simulator
def hand_geometry(p):
    W_, sp = W, W + p["gap"]
    z = np.array([0.0, 0, 1])
    fingers = []
    for i in range(4):
        Lf, m, dd = p["Lf"][i], p["m"][i], p["dd"][i]
        L1 = Lf / (1 + m + m * dd); L = [L1, m * L1, m * dd * L1]
        y = (1.5 - i) * sp
        B = np.array([p["Lp"] + p["dx"][i], y, -D / 2 + p.get("zarch", 0.0) * abs(i - 1.5) / 1.5])
        a = math.radians(p["fan"] * (1.5 - i)); U = np.array([math.cos(a), math.sin(a), 0.0])
        c = math.radians(p["conv"] * i - p.get("cconv", 0.0) * (1.5 - i) / 1.5)
        N = math.cos(c) * z - math.sin(c) * np.cross(U, z); N /= np.linalg.norm(N)
        fingers.append(dict(B=B, U=U, N=N, L=L))
    y_hi0 = 1.5 * sp + W / 2                       # finger-row edge (the simulator's palm edge)
    C = np.array([p["xt"], y_hi0 + p["yt"], p["zt"]])
    y_hi, y_lo = y_hi0 + 2.5, -y_hi0 - 2.5          # 2.5 mm side walls outside the outer sockets
    yaw, pitch, roll = map(math.radians, (p["yaw"], p["pitch"], p["roll"]))
    mvec = np.array([math.cos(pitch) * math.cos(yaw), math.cos(pitch) * math.sin(yaw), math.sin(pitch)])
    n0 = z - mvec * mvec.dot(z); n0 /= np.linalg.norm(n0)
    nvec = math.cos(roll) * n0 + math.sin(roll) * np.cross(mvec, n0)
    thumb = dict(C=C, m=mvec, n=nvec, Lm=p["Lm"], L=[p["Lt1"], p["Lt2"]], M=C + p["Lm"] * mvec)
    return dict(fingers=fingers, thumb=thumb, y_hi=y_hi, y_lo=y_lo, y_hi0=y_hi0, pads=p.get("pads") or {})


# ----------------------------------------------------------------------------- S6 / thumb pivot layout (hand frame)
S6_FACE = 21.5          # x of the S6 case face (shaft side); the hub + horn live between the pivot wall and here
S6_LEN, S6_W, S6_T = 22.7, 23.0, 12.2      # MG90S body along the shaft, across, thickness
S6_TAB_X = (27.8, 31.0)                      # tab slot
S6_TAB_SPAN = 33.0
LANES = {0: 9.5, 1: 3.5, 2: -3.5, 3: -10.0}  # where each finger tendon leaves the wrist end (y), all at z = -15
LANE_Z = -15.0


def s6_box(C, grow=0.0):
    yc = C[1] - 5.9
    body = cq.Workplane("XY").box(S6_LEN + 2 * grow, S6_W + 2 * grow, S6_T + 2 * grow, centered=False).translate(
        (S6_FACE - grow, yc - S6_W / 2 - grow, C[2] - S6_T / 2 - grow))
    tabs = cq.Workplane("XY").box(S6_TAB_X[1] - S6_TAB_X[0] + 2 * grow, S6_TAB_SPAN + 2 * grow, S6_T + 2 * grow, centered=False).translate(
        (S6_TAB_X[0] - grow, yc - S6_TAB_SPAN / 2 - grow, C[2] - S6_T / 2 - grow))
    return body.union(tabs)


# ----------------------------------------------------------------------------- printed parts
def finger_parts(L):
    return dict(proximal=F.phalanx(L[0], anchors=[F.GAP + 5.0, L[0] - R - 6.0]),
                middle=F.phalanx(L[1], anchors=[min(13.0, L[1] - R - 3.5)]),
                distal=F.distal(L[2]))


DEBUG = False
def _dbg(b, tag):
    if DEBUG:
        sols = b.solids().vals()
        probe = b.val().intersect(Solid.makeSphere(0.35, Vector(50, 0, -18))).Volume()
        print("  ", tag, b.val().isValid(), len(sols), "probe", round(probe, 4))


def palm_part(G):
    fs = G["fingers"]; th = G["thumb"]
    x_front = min(f["B"][0] for f in fs) - R - 1.5          # clear of the proximal tongues as they swing
    yl, yh = G["y_lo"], G["y_hi"]
    T = 22.0
    C = th["C"]; yc = C[1] - 5.9
    body = palm_shell(G, x_front, T)
    body = body.union(thenar_pillow(G, x_front))
    _dbg(body, "mound only")
    pads = pads_part(G, x_front)       # the cup ridges: same crest lines and height as the simulated palm
    if pads is not None:
        body = body.union(pads)
    _dbg(body, "mound")
    # finger sockets: the rev B base sits in the palm, pin centre on the knuckle point
    for fi, f in enumerate(fs):
        loc = frame_loc(f["B"], f["U"], f["N"])
        env = F.box(-F.BASE_L - 0.4, R + 0.6, -R - 0.3, R + 0.3, -W / 2 - 0.2, W / 2 + 0.2)
        body = body.cut(wp(placed(env, loc)))
        # M2 x 16 flat-head screws from the back of the palm into 1.6 mm pilots in the base, heads countersunk flush
        Yd = -np.asarray(f["N"], float); Yd /= np.linalg.norm(Yd); Zd = np.cross(np.asarray(f["U"], float) / np.linalg.norm(f["U"]), Yd)
        for zz in (-5.0, 5.0):
            hx = -F.BASE_L / 2 - 2
            p0 = np.asarray(f["B"], float) + np.asarray(f["U"], float) / np.linalg.norm(f["U"]) * hx + Zd * zz
            y_exit = (-T - p0[2]) / Yd[2]                       # where the screw axis meets the back of the palm
            hole = F.cyl_y(M2_CLEAR / 2, hx, zz, -R - 1, y_exit + 3)
            csk = wp(Solid.makeCone(M2_CLEAR / 2, M2_CSK / 2, (M2_CSK - M2_CLEAR) / 2, Vector(hx, y_exit - (M2_CSK - M2_CLEAR) / 2, zz), Vector(0, 1, 0)))
            csk = csk.union(F.cyl_y(M2_CSK / 2, hx, zz, y_exit - 0.01, y_exit + 3))
            body = body.cut(wp(placed(hole.union(csk), loc)))
        # tendon: base exit (palm side) -> down to z = -15 -> its lane at the wrist end
        ex = f["B"] + f["U"] * (-F.BASE_L) + (-f["N"]) * F.FLEX_Y
        lane = LANES[fi]
        pts = [ex + f["U"] * 1.0, ex - f["U"] * 6, np.array([ex[0] - 18, 0.75 * f["B"][1] + 0.25 * lane, LANE_Z])]
        if fi == 0:      # the index lane swings inboard early to stay clear of S6
            pts.append(np.array([50.0, lane, LANE_Z]))
        pts.append(np.array([-1.0, lane, LANE_Z]))
        body = path_cut(body, pts, 1.25)
        _dbg(body, f"finger {fi}")
    _dbg(body, "sockets+lanes")
    # notch for the hub, horn and carrier arm, between the wall and the S6 face
    body = body.cut(cq.Workplane("XY").box(S6_FACE - 4.0, yh + 20 - (C[1] - 12.0), 40, centered=False).translate((4.0, C[1] - 12.0, C[2] - 10.5)))
    _dbg(body, "notch")
    # thumb pivot: 4 mm wall at the wrist end with the pivot bore; the thumb tendon runs along the axis
    body = body.cut(wp(Solid.makeCylinder(2.15, 6.0, Vector(-1.0, C[1], C[2]), Vector(1, 0, 0))))
    _dbg(body, "pivot")
    # S6 pocket, open to the back of the palm (servo drops in from behind, a cover holds it)
    yc = C[1] - 5.9
    hz = (C[2] + S6_T / 2 + 0.2) - (-T - 5)          # open from the back up to the servo's top face only
    for bx in (cq.Workplane("XY").box(S6_LEN + 0.4, S6_W + 0.4, hz, centered=False).translate((S6_FACE - 0.2, yc - S6_W / 2 - 0.2, -T - 5)),
               cq.Workplane("XY").box(S6_TAB_X[1] - S6_TAB_X[0] + 0.4, S6_TAB_SPAN + 0.4, hz, centered=False).translate((S6_TAB_X[0] - 0.2, yc - S6_TAB_SPAN / 2 - 0.2, -T - 5))):
        body = body.cut(bx)
        _dbg(body, "pocket box")
    _dbg(body, "pocket")
    # cover recess on the back face + two M2 pilot holes (M2 x 8 flat-head screws)
    body = body.cut(cq.Workplane("XY").box(32.0, S6_TAB_SPAN + 3.0, 3.0, centered=False).translate((17.0, yc - S6_TAB_SPAN / 2 - 1.5, -T - 0.01)))
    for xx in (18.0, 47.0):
        body = body.cut(wp(Solid.makeCylinder(M2_PILOT / 2, 10.0, Vector(xx, yc, -T - 1), Vector(0, 0, 1))))
    # wrist flange: 4 M2 pilot holes, 14 mm deep (the 2-axis wrist screws in here)
    for yy in (-22.0, 22.0):
        for zz in (-6.0, -16.0):
            body = body.cut(wp(Solid.makeCylinder(M2_PILOT / 2, 14.0, Vector(-1.0, yy, zz), Vector(1, 0, 0))))
    return body, x_front


def palm_shell(G, x_front, T, z_top=0.0, raw=False):
    """Organic palm body: narrows toward the wrist, a rounded thenar bulge on the thumb side that
    houses S6, straight sides only where the finger sockets need walls, soft edges all round."""
    yh, yl = G["y_hi"], G["y_lo"]
    rf = 4.0
    pts_little = [(6.0, yl + 2.0), (24.0, yl - 0.2), (55.0, yl)]
    pts_thumb = [(70.0, yh + 0.3), (56.0, 46.0), (46.0, 51.0), (31.0, 52.2), (17.0, 50.8), (8.0, 46.5), (3.0, 43.0)]
    w = (cq.Workplane("XY").moveTo(0.0, yl + 9.0)
         .spline(pts_little + [(x_front - rf - 6, yl)], includeCurrent=True)
         .lineTo(x_front - rf, yl).threePointArc((x_front - rf * 0.293, yl + rf * 0.293), (x_front, yl + rf))
         .lineTo(x_front, yh - rf).threePointArc((x_front - rf * 0.293, yh - rf * 0.293), (x_front - rf, yh))
         .spline(pts_thumb + [(0.0, 40.5)], includeCurrent=True)
         .close())
    body = w.extrude(T).translate((0, 0, -T + z_top))
    if not raw:
        try:
            body = body.faces(">Z").edges().fillet(3.5)
        except Exception:
            pass
        try:
            body = body.faces("<Z").edges().fillet(2.5)
        except Exception:
            pass
    return body


def thenar_pillow(G, x_front):
    """Soft raised pad at the thumb base (where a real hand's thenar muscle is). It covers the top of S6,
    which sits 2 mm proud of the palm face, and runs flush into the thumb-side wall of the palm."""
    top = 3.4
    rr = cq.Workplane("XY").box(46.0 - 19.5, 70.0 - 11.0, 8.0, centered=False).translate((19.5, 11.0, top - 8.0))
    rr = rr.edges("|Z").fillet(6.0)
    pil = palm_shell(G, x_front, 8.0, z_top=top, raw=True).intersect(rr)
    try:
        pil = pil.faces(">Z").edges().fillet(2.4)
    except Exception:
        pass
    return pil


def pads_part(G, x_front):
    """The cup: rounded 8 mm ridges (half-buried r = 8 mm rods) on the same crest lines as the simulated
    ridges. Little-finger side and heel join through a rounded corner; the thumb-side ridge starts in
    front of the thumb carrier."""
    yh = G["y_hi0"]; yl = -yh; r = 8.0
    names = G["pads"]
    if not names:
        return None
    zc = max(names.values()) - r
    x1 = x_front - 0.5
    P = lambda x, y: np.array([x, y, zc])
    chains = []
    if names.get("hypothenar") or names.get("heel"):
        cx, cy = 18.0, yl + 16.0               # corner arc centre (radius 8)
        arc = [P(cx + 8 * math.cos(a), cy + 8 * math.sin(a)) for a in np.radians([270, 247.5, 225, 202.5, 180])]
        chains.append([P(x1 - 12.0, yl + r)] + arc + [P(10.0, 11.0)])   # ends clear of the thumb carrier at full swing
    if names.get("thenar"):
        chains.append([P(x1 - 22.0, yh - r), P(30.0, yh - r)])
    out = None
    for ch in chains:
        for a, b in zip(ch[:-1], ch[1:]):
            seg = rod(a, b, r)
            out = seg if out is None else out.union(seg)
    return out.intersect(cq.Workplane("XY").box(400, 400, 40, centered=False).translate((-50, -200, -0.5)))


def thumb_carrier(G):
    """Hub on the swing axis (x through C) + arm + rev B knuckle clevis. A boss pivots in the palm's
    wrist-end wall; S6's horn drives the hub through a radial slot. The thumb tendon runs down the
    swing axis (so swinging the thumb doesn't change its length), then into the arm."""
    th = G["thumb"]; C, m, n, Lm = th["C"], th["m"], th["n"], th["Lm"]
    loc = frame_loc(th["M"], m, n)
    arm = F.box(-(Lm - 6.0), -F.BASE_L + 1, -R, R, -W / 2 + 1, W / 2 - 1)
    part = wp(placed(F.base(), loc)).union(wp(placed(arm, loc)))
    hub = wp(Solid.makeCylinder(9.4, 18.0 - 4.5, Vector(4.5, C[1], C[2]), Vector(1, 0, 0)))
    boss = wp(Solid.makeCylinder(2.0, 4.2, Vector(0.6, C[1], C[2]), Vector(1, 0, 0)))
    part = part.union(hub).union(boss)
    # keep the arm and hub inside x = 4.5 .. 20 so they clear the wall and the servo
    part = part.intersect(cq.Workplane("XY").box(400, 400, 400, centered=False).translate((0.6, -200, -200)))
    part = part.cut(cq.Workplane("XY").box(10, 400, 400, centered=False).translate((S6_FACE - 1.5, -200, -200)).cut(
        cq.Workplane("XY").box(400, 400, 400, centered=False).translate((-50, -200, C[2] + 9.0))))
    # tendon: along the axis (x 0 -> 8), then a straight channel to the arm's flexor channel, then up the arm
    a_start = th["M"] + m * (-(Lm - 2.0)) + (-n) * F.FLEX_Y
    part = part.cut(wp(Solid.makeCylinder(1.1, 9.0, Vector(-1.0, C[1], C[2]), Vector(1, 0, 0))))
    part = part.cut(rod(np.array([8.0, C[1], C[2]]), a_start, 1.1))
    part = part.cut(wp(placed(F.cyl_x(F.FLEX_D, F.FLEX_Y, -(Lm - 2.0), -F.BASE_L + 2), loc)))
    # horn slot on the servo face of the hub: open toward the back of the palm (-z) so the horn slides in
    part = part.cut(cq.Workplane("XY").box(2.7, 4.8, 20.0, centered=False).translate((18.0 - 2.7, C[1] - 2.4, C[2] - 20.0 + 3.5)))
    part = part.cut(wp(Solid.makeCylinder(3.8, 2.7, Vector(18.0 - 2.7, C[1], C[2]), Vector(1, 0, 0))))   # horn centre boss
    return part


def s6_cover(G):
    """Holds S6 in its pocket from the back of the palm (2 x M2 x 8 flat-head screws into the palm)."""
    C = G["thumb"]["C"]; yc = C[1] - 5.9; T = 22.0
    plate = cq.Workplane("XY").box(31.6, S6_TAB_SPAN + 2.6, 2.8, centered=False).translate((17.2, yc - S6_TAB_SPAN / 2 - 1.3, -T))
    try:
        plate = plate.edges("|Z").fillet(3.0)
    except Exception:
        pass
    zb = C[2] - S6_T / 2 - 0.2                               # servo underside
    block = cq.Workplane("XY").box(S6_LEN - 2, S6_W - 4, zb - (-T + 2.8), centered=False).translate((S6_FACE + 1, yc - S6_W / 2 + 2, -T + 2.8))
    part = plate.union(block)
    for xx in (18.0, 47.0):
        part = part.cut(wp(Solid.makeCylinder(M2_CLEAR / 2, 6, Vector(xx, yc, -T - 1), Vector(0, 0, 1))))
        part = part.cut(wp(Solid.makeCone(M2_CSK / 2, M2_CLEAR / 2, (M2_CSK - M2_CLEAR) / 2, Vector(xx, yc, -T), Vector(0, 0, 1))))
    return part


def spool(r):
    """Tendon spool for an MG90S round horn: horn recess under a 1 mm web, counterbored centre screw."""
    fl_b = wp(Solid.makeCylinder(r + 2.5, 3.2, Vector(0, 0, 0)))
    drum = wp(Solid.makeCylinder(r, 7.0, Vector(0, 0, 3.2)))
    fl_t = wp(Solid.makeCylinder(r + 2.5, 1.2, Vector(0, 0, 10.2)))
    s = fl_b.union(drum).union(fl_t)
    s = s.cut(wp(Solid.makeCylinder(10.2, 2.2, Vector(0, 0, -0.1))))                 # round-horn recess
    s = s.cut(wp(Solid.makeCylinder(1.2, 20, Vector(0, 0, -1))))                     # horn screw
    s = s.cut(wp(Solid.makeCylinder(2.3, 20, Vector(0, 0, 3.2))))                    # counterbore for the screw head
    for a in (0.0, math.pi / 2, math.pi, 1.5 * math.pi):                               # horn screw holes (pick the ones that line up)
        s = s.cut(wp(Solid.makeCylinder(0.8, 4, Vector(7.0 * math.cos(a), 7.0 * math.sin(a), -0.5))))
    s = s.cut(wp(Solid.makeCylinder(0.9, 2 * r + 6, Vector(-(r + 3), 0, 6.7), Vector(1, 0, 0))))   # tendon tie hole
    return s


SLOTS = {"S1": (-55, 19), "S2": (-55, -19), "S3": (-100, -19), "S4": (-145, -19), "S5": (-100, 19)}


def forearm_tray():
    plate = cq.Workplane("XY").box(156, 86, 5, centered=False).translate((-178, -43, -29)).edges("|Z").fillet(6)
    walls = cq.Workplane("XY").box(156, 3, 16, centered=False).translate((-178, 40, -27)).union(
        cq.Workplane("XY").box(156, 3, 16, centered=False).translate((-178, -43, -27)))
    tray = plate.union(walls)
    for sid, (sx, sy) in SLOTS.items():
        frame = cq.Workplane("XY").box(36, 16, 15.5, centered=False).translate((sx - 18, sy - 8, -24))
        frame = frame.cut(cq.Workplane("XY").box(23.4, 12.8, 20, centered=False).translate((sx - 11.7, sy - 6.4, -25)))
        for xx in (-13.8, 13.8):
            frame = frame.cut(wp(Solid.makeCylinder(0.9, 8, Vector(sx + xx, sy, -14))))
        tray = tray.union(frame)
    # posts for the comb (the comb is its own part, printed flat, glued into the post notches)
    for yy in (37.0, -42.0):
        post = cq.Workplane("XY").box(6, 5, 63, centered=False).translate((-35, yy, -27))
        post = post.cut(cq.Workplane("XY").box(7, 6, 6.2, centered=False).translate((-35.5, yy - 0.5, 30)))
        tray = tray.union(post)
    return tray


def wrist_guide(C):
    """Wall at the wrist end of the tray: one hole per tendon, lined up with the palm lanes."""
    wall = cq.Workplane("XY").box(4, 84, 29, centered=False).translate((-26, -42, -29))
    wall = wall.union(cq.Workplane("XY").box(4, 14, 10, centered=False).translate((-26, C[1] - 7, -4.5 - 5)))
    for y in LANES.values():
        wall = wall.cut(wp(Solid.makeCylinder(1.3, 8, Vector(-27, y, LANE_Z), Vector(1, 0, 0))))
    wall = wall.cut(wp(Solid.makeCylinder(1.3, 8, Vector(-27, C[1], C[2]), Vector(1, 0, 0))))
    return wall


def tendon_comb():
    """Bar over the front servo row that lifts the back-row tendons clear of the front spools.
    Print lying flat (holes horizontal)."""
    bar = cq.Workplane("XY").box(6, 84, 6, centered=False).translate((-35, -42, 30))
    for y in (16.0, 5.6, -5.6, -16.0, 27.2):
        bar = bar.cut(wp(Solid.makeCylinder(1.3, 10, Vector(-36, y, 33), Vector(1, 0, 0))))
    return bar


def build(params_path):
    p = json.load(open(params_path))
    G = hand_geometry(p)
    parts = {}
    palm, x_front = palm_part(G)
    # the notch already clears the carrier through its whole 0-40 deg swing (check_fit.py verifies it)
    carrier = thumb_carrier(G)
    parts["palm"] = palm
    m3 = F.M3; F.M3 = M2_PILOT            # finger bases get 1.6 mm pilots so the M2 screws thread in
    parts["finger_base"] = F.base(); F.M3 = m3
    std = finger_parts(G["fingers"][0]["L"]); lit = finger_parts(G["fingers"][3]["L"])
    for k, v in std.items():
        parts[f"finger_{k}"] = v
    for k, v in lit.items():
        parts[f"little_{k}"] = v
    parts["thumb_carrier"] = carrier
    parts["thumb_proximal"] = F.phalanx(G["thumb"]["L"][0], anchors=[F.GAP + 5.0, G["thumb"]["L"][0] - R - 6.0])
    parts["thumb_distal"] = F.distal(G["thumb"]["L"][1])
    parts["s6_cover"] = s6_cover(G)
    parts["forearm_tray"] = forearm_tray().union(wrist_guide(G["thumb"]["C"]))
    parts["tendon_comb"] = tendon_comb()
    parts["spool_32"] = spool(15.8)
    parts["spool_20"] = spool(10.1)
    return p, G, parts


QTY = {"palm": 1, "palm_pads": 1, "finger_base": 5, "finger_proximal": 3, "finger_middle": 3, "finger_distal": 3,
       "little_proximal": 1, "little_middle": 1, "little_distal": 1, "thumb_carrier": 1, "thumb_proximal": 1, "thumb_distal": 1,
       "forearm_tray": 1, "spool_32": 4, "spool_20": 1, "s6_cover": 1, "tendon_comb": 1}


def assembly(G, parts, curls=None, beta=0.0):
    """Posed assembly in the hand frame. curls: 5 lists of joint angles (deg)."""
    curls = curls or [[0, 0, 0]] * 4 + [[0, 0]]
    out = {"palm": parts["palm"].val()}
    if "palm_pads" in parts:
        out["palm_pads"] = parts["palm_pads"].val()
    for i, f in enumerate(G["fingers"]):
        pre = "finger" if i < 3 else "little"
        base_loc = frame_loc(f["B"], f["U"], f["N"])
        out[f"base_{i}"] = parts["finger_base"].val().moved(base_loc)
        loc = base_loc
        xs = [0.0, f["L"][0], f["L"][1]]
        for name, a, x in zip(("proximal", "middle", "distal"), curls[i], xs):
            loc = loc * cq.Location(Vector(x, 0, 0)) * cq.Location(Vector(), Vector(0, 0, 1), -a)
            out[f"{pre}_{name}_{i}"] = parts[f"{pre}_{name}"].val().moved(loc)
    th = G["thumb"]
    rot = cq.Location(Vector(*th["C"])) * cq.Location(Vector(), Vector(1, 0, 0), beta) * cq.Location(Vector(*(-th["C"])))
    out["thumb_carrier"] = parts["thumb_carrier"].val().moved(rot)
    loc = rot * frame_loc(th["M"], th["m"], th["n"])
    for name, a, x in zip(("thumb_proximal", "thumb_distal"), curls[4], [0.0, th["L"][0]]):
        loc = loc * cq.Location(Vector(x, 0, 0)) * cq.Location(Vector(), Vector(0, 0, 1), -a)
        out[name] = parts[name].val().moved(loc)
    out["forearm_tray"] = parts["forearm_tray"].val()
    out["s6_cover"] = parts["s6_cover"].val(); out["tendon_comb"] = parts["tendon_comb"].val()
    return out


if __name__ == "__main__":
    params = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "hand_params.json")
    p, G, parts = build(params)
    od = os.path.join(HERE, "parts"); os.makedirs(od, exist_ok=True)
    for name, s in parts.items():
        cq.exporters.export(s, os.path.join(od, f"{name}.stl"), tolerance=0.03, angularTolerance=0.15)
        cq.exporters.export(s, os.path.join(od, f"{name}.step"))
        v = s.val().Volume() / 1000
        print(f"{name:18s} x{QTY[name]}  {v:6.1f} cm3  valid={s.val().isValid()}")
    asm = cq.Assembly(name="haptic_teleop_hand")
    for name, s in assembly(G, parts).items():
        col = cq.Color(0.96, 0.77, 0.33) if any(k in name for k in ("proximal", "middle", "distal")) else cq.Color(0.45, 0.52, 0.6)
        asm.add(cq.Workplane().add(s), name=name, color=col)
    asm.save(os.path.join(HERE, "hand_assembly.step"))
    print("exported to", od)
