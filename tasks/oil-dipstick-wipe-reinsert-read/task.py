"""A lawn tractor engine's oil check on the low-cost rig: the genuine yellow-handled dipstick of a Kohler KT725 V-twin
seated in its genuine oil fill tube over a quart can of oil, at the corner of a black plywood box that stands in for
the engine's blower housing, all at the engine's real sizes and places from Kohler's dimension drawing, with a folded
shop rag and an oil check card with ADD, OK and OVER pockets and a marker puck. Checking the oil means pulling the dipstick, wiping the blade clean on the rag (the first pull reads a false, smeared level because oil
splashes up the tube while the engine runs), pushing it back in until the cap seats, pulling it again and holding it
still to read the level, then seating it again, putting the rag down and reporting the level by leaving the puck in
the matching pocket. The level changes from trial to trial (below ADD, between the marks, or above FULL), so the report
can only be right if the blade is read.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the rig and its sliding dipstick; nothing on disk changes)
import guardkit as gk  # noqa: E402  (the gripper's own shape, for the let-go checks)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = ("Check the oil level on the engine with the dipstick, mark it on the check card with the puck, then put "
               "everything back.")

# The rig (the low-cost build of the README) is fixed to the table on an 8 mm plywood base that stands for the engine's
# mounting surface. The oil fill tube rises straight up at the housing box's front corner, where Kohler's drawing for
# the KT725 (service manual 32 690 03 Rev. H, p.5 top view: "OIL FILL & DIPSTICK (YELLOW)") puts it on the engine; the
# dipstick slides along the tube's axis. The tube frame has its origin at the
# middle of the tube's mouth (the collar's top rim) and its z axis along the tube, out of the mouth (here world z).
SX, SY = 0.36, -0.12  # the tube's mouth in world x, y (the robot's near right corner of the housing box)
PLATE_T = 0.008  # the plywood base, standing for the engine's mounting surface
MOUTH = PLATE_T + 0.275  # the collar's top 27.5 cm above the base (the engine's height, read off the p.5 drawing)
U = np.array([0.0, 0.0, 1.0])  # the tube's axis, out of the mouth: straight up (the drawing's tube is near vertical)
# The tube frame is the world frame turned a quarter turn about z: the blade's width (tube x) runs along world y and its
# thickness (tube y) along world x, so its flat faces look toward and away from the robot.
TILT = (0.0, 0.0, float(math.sqrt(0.5)), float(math.sqrt(0.5)))  # x, y, z, w (plain floats: Warp rejects float64)
R_TUBE = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
MOUTH_W = np.array([SX, SY, TOP + MOUTH])  # the mouth's middle in world coordinates
BLADE = 0.245  # the dipstick blade, from the cap's underside to the tip: to 3 cm above the base, in the can's oil
# (with the cap and T-handle below, the stick is 0.29 m overall, sized to the drawing's tube; handle top ~31 cm up)
BLADE_W, BLADE_T = 0.010, 0.003  # blade width (along the tube frame's x) and thickness (along y)
CAP_R, CAP_H = 0.016, 0.010  # the cap's disc, which seats on the tube's rim
HANDLE = (0.035, 0.018, 0.035)  # the T-handle above the cap: width (tube x), thickness (y), length along the tube (z)
HANDLE_C = CAP_H + HANDLE[2] / 2  # the handle's middle along the tube from the link origin (the cap's underside)
TUBE_R = 0.0105  # the fill tube's outside radius (about 2.1 cm across in the drawing)
COLLAR_R, COLLAR_H = 0.018, 0.05  # the wider collar at the tube's top, which the cap seats on
TUBE_IN = 0.20  # the tube runs this far down, through the can's lid into the can
RAG = (0.04, 0.04, 0.025)  # the folded shop rag (pad): x, y, z
RAG_START = (0.20, 0.0, TOP + RAG[2] / 2)

# The oil level, set per trial (a real trial sets it with a syringe before each run). ADD is 0.8 cm and FULL 2.0 cm
# above the tip; each case sits 4 mm or more from the nearest mark, so it reads the same by eye from any camera.
ADD_MARK, FULL_MARK = 0.008, 0.020
LEVELS = {"low": 0.004, "ok": 0.014, "over": 0.027}  # m above the tip: below ADD, between the marks, above FULL
CASES = ("low", "ok", "over")
# The oil check card (fixed to the bench, beside the rag) and its marker puck. The pockets run ADD, OK, OVER from the
# robot's left to its right (left to right in the top camera).
CARD_C = (0.20, 0.13)  # the card's middle, x, y (in front of the housing, on the robot's left)
CARD = (0.06, 0.16, 0.010)  # x, y, z: 6 x 16 cm, 1 cm tall
FLOOR = 0.005  # each pocket is 5 mm deep: its floor 5 mm above the table, the card's top 1 cm
POCKET = 0.044  # pockets 4.4 cm square
POCKET_DY = 0.052  # pocket middles 5.2 cm apart along y (0.8 cm walls between them)
POCKETS = {"low": (CARD_C[0], CARD_C[1] + POCKET_DY), "ok": CARD_C, "over": (CARD_C[0], CARD_C[1] - POCKET_DY)}
POCKET_NAME = {"low": "ADD", "ok": "OK", "over": "OVER"}
PUCK_R, PUCK_H = 0.015, 0.02  # the marker puck: 3 cm across, 2 cm tall
PUCK_START = (0.14, 0.22, TOP + PUCK_H / 2)

# Scoring (the dipstick's joint position q is how far the cap is above its seat, m)
SEAT = 0.003  # m: q at or under this is seated (cap down on the rim; a 3 mm gap is a visible lift of the cap)
OUT = BLADE + 0.005  # m: the blade's tip at least 5 mm above the mouth: the dipstick is out of the tube
TIP_IN = BLADE - 0.005  # m: under this the tip is back inside the tube (it picks up the splash on the tube wall)
WIPE_ZONE = 0.06  # m: the oiled length of the blade (estimated): from the tip up past the highest level set (2.7 cm, the
# "over" case) and the FULL/ADD crosshatch, plus the splash film the running engine throws up the tube; drawn as the smear
WIPE_SPAN = WIPE_ZONE  # m: the stretch from the tip that the rag's middle must slide over in contact: the whole oiled
# length, so the pad (about 4 cm tall on edge, 2 cm either side of its middle) rubs from below the tip to 2 cm above it
WIPE_FRAC = 0.9  # the middle must slide over 90% of that span (5.4 cm) in contact: a stroke may stop up to 6 mm short
# (the arm lags its target), but a dab (no slide), a spot wipe or a stroke over part of the oiled length does not count
# that are both in contact, so nothing is credited across a gap (a dab, a lift and a dab elsewhere add up to nothing)
SLIDE_MAX = 0.005  # m per control step (10 cm/s): a faster jump between two touching steps is not credited (the demo's
# stroke slides about 0.75 mm per step; a pad yanked along faster skips over the oil rather than wiping it)
BIN = 0.001  # m: the blade is tracked in 1 mm bins
RAG_TOUCH = 0.027  # m: rag middle within this of the blade's axis: half the pad (2 cm) plus the blade's half-thickness and
# the cloth pressing round it (about 0.5 cm); a rag hovering clear of the blade sits farther out than this
RAG_GAP = 0.002  # m: and the rag's outline within 2 mm of the blade's faces (the cloth's nap and the pad's give): a rigid
# pad held 5 mm clear of the blade can still have its middle within RAG_TOUCH, so the outline is checked as well
READ_STEPS = 20  # control steps (1 s at 20 Hz) the dipstick must be held out and still to read it
STILL = 0.01  # m/s: the dipstick moving slower than this (its travel per control step) counts as still
CONTROL_HZ = 20
# Held: a gripper with both fingers touching the yellow T-handle (gk.gripping, contact as below) and their opening
# stopped by it: across the handle's 1.8 cm thickness or 3.5 cm width (from 1.8 cm less 15 % of full opening to 3.5 cm
# plus 3 %). The dipstick is drawn and pushed by its handle only: the blade carries the oil, so a gripper touching the
# blade (its own shape within 2 mm) while the dipstick is off its seat ends the trial at 0 (a pinched blade smears the
# band and the reading). An empty fist or an open hand around the handle does not hold it.
HANDLE_GRIP = (gk.band(HANDLE[1])[0], gk.band(HANDLE[0])[1])
# Let go at the end: no gripper touching the dipstick's cap and handle, the rag or the puck, judged by the gripper's own
# shape (palm and both fingers at their present opening, gk.touching: within 2 mm, the solver's resting gap plus the box
# fit to the meshes). A hand hovering beside them, however close, has let go.
HANDLE_SHAPE = gk.Block((0.0, 0.0, HANDLE_C), HANDLE)  # in the dipstick's frame
BLADE_SHAPE = gk.Block((0.0, 0.0, -BLADE / 2), (BLADE_W, BLADE_T, BLADE))
STICK_SHAPE = (HANDLE_SHAPE, gk.Ring(0.0, CAP_R, 0.0, CAP_H), BLADE_SHAPE)
RAG_SHAPE = gk.Block((0.0, 0.0, 0.0), RAG)
PUCK_SHAPE = gk.Ring(0.0, PUCK_R, -PUCK_H / 2, PUCK_H / 2)
RAG_DOWN = 0.01  # m: the rag at rest with its bottom 1 cm or less above the table
IN_POCKET = 0.012  # m: the puck's middle within this of a pocket's middle along x and y (a puck lying flat in the
# 4.4 cm pocket can be at most 7 mm off; one hanging on a wall with its middle outside the pocket does not count)
SUNK = 0.002  # m: and its bottom at least 2 mm below the card's top (down in the 5 mm pocket, not lying on the rim)
DEBOUNCE = 2  # control steps: the puck in a pocket for 2 steps is a report made (one step can be a physics pop)

SHROUD = kit.Look((0.05, 0.05, 0.055), 0.6)  # the housing box: plywood painted plain black
PLY = kit.Look((0.74, 0.62, 0.45), 0.85)  # the unpainted plywood base the box is screwed to
CAN = kit.Look((0.72, 0.73, 0.75), 0.35, metallic=0.8)  # the unlined steel quart can (tinplate)
DARK = kit.Look((0.14, 0.14, 0.15), 0.6)
YELLOW = kit.Look((0.96, 0.76, 0.08), 0.45)  # Kohler yellow: dipstick cap and handle
STEEL = kit.Look((0.82, 0.83, 0.85), 0.25, metallic=0.9)
BLACK = kit.Look((0.06, 0.06, 0.06), 0.7)
RAGLOOK = kit.Look((0.20, 0.35, 0.70), 0.9)  # blue shop towel
CARDLOOK = kit.Look((0.20, 0.21, 0.23), 0.6)  # dark gray card
POCKET_LOOK = {"low": kit.Look((0.95, 0.55, 0.05), 0.5), "ok": kit.Look((0.10, 0.65, 0.20), 0.5),
               "over": kit.Look((0.80, 0.08, 0.06), 0.5)}  # amber ADD, green OK, red OVER
PUCKLOOK = kit.Look((0.95, 0.95, 0.93), 0.4)  # white puck
LABEL = kit.Look((0.97, 0.97, 0.95), 0.5)  # the pockets' white label symbols
TICK = kit.Look((0.02, 0.02, 0.02), 0.6)  # the ADD and FULL marks


def _box(lo, hi, look, solid=True):
    c = tuple((a + b) / 2 for a, b in zip(lo, hi))
    s = tuple(b - a for a, b in zip(lo, hi))
    return kit.box(c, s, look), (kit.Collider(c, size=s) if solid else None)


def _parts(boxes):
    meshes, colliders = [], []
    for lo, hi, look, solid in boxes:
        m, c = _box(lo, hi, look, solid)
        meshes.append(m)
        if c is not None:
            colliders.append(c)
    return tuple(meshes), tuple(colliders)


# The rig (the low-cost build, README "Real-world build"): Kohler's genuine fill tube 32 123 14-S and dipstick
# 32 038 14-S, the tube standing in a 1 quart steel can of oil, at the corner of a 41 x 41 x 30 cm box of 12 mm plywood
# painted black that stands where the KT725's blower housing is, its footprint, top and corners from Kohler's dimension
# drawing (p.5). The box is screwed to an 8 mm plywood base, 47 x 50 cm. Its near right corner (where the tube stands)
# is cut off as two flat faces from 20 cm along the front and the right side to a point 6 cm in from both; they run from
# the top down to 20 cm above the base and are open below, where the can stands under the tube. The other three corners
# are cut at 45 degrees, 13 cm along each side (12 cm at the far right corner).
BASE_C, BASE = (0.512, 0.062), (0.47, 0.50)  # the plywood base's middle (world x, y) and its size
BX0, BX1, BY0, BY1 = 0.307, 0.717, -0.145, 0.265  # the box's outside, 41 x 41 cm (world x, y)
BOX_H, PLY_T = 0.30, 0.012  # the box's height above the base, the plywood's thickness
CUT, CUT_IN, CUT_Z = 0.20, 0.06, 0.20  # the tube corner's faces: 20 cm along each side, 6 cm in, down to 20 cm up
CHAMFER = {"far_right": 0.12, "far_left": 0.13, "near_left": 0.13}  # the other corners' 45 degree cuts
CAN_R, CAN_H = 0.054, 0.124  # the 1 quart can: 10.8 cm across, 12.4 cm tall, on the base under the tube
MS = PLATE_T  # the base's top: the box, the can and the tube's mouth height are measured from here
TAB_Z = MS + 0.225  # the tube's tab, screwed to the corner face just above its lower edge


def _outline():
    """The box's outline (world x, y), counterclockwise from above, from the tube corner's right-hand face."""
    c = CHAMFER
    return [(BX0 + CUT, BY0), (BX1 - c["far_right"], BY0), (BX1, BY0 + c["far_right"]), (BX1, BY1 - c["far_left"]),
            (BX1 - c["far_left"], BY1), (BX0 + c["near_left"], BY1), (BX0, BY1 - c["near_left"]), (BX0, BY0 + CUT),
            (BX0 + CUT_IN, BY0 + CUT_IN)]


CORNER_FACES = (7, 8)  # the outline's edges from these vertices are the two faces at the tube


def _prism(poly, z0, z1, look):
    """A closed slab over a convex outline (x, y points, counterclockwise), from z0 to z1."""
    n = len(poly)
    pts = [(x, y, z) for z in (z0, z1) for x, y in poly]
    counts, idx = [], []
    for k in range(n):
        counts.append(4)
        idx += [k, (k + 1) % n, (k + 1) % n + n, k + n]
    counts += [n, n]
    idx += list(range(n))[::-1] + [n + k for k in range(n)]
    return kit._mesh(pts, counts, idx, look)


def _span(poly, x):
    """The outline's y extent at x (a convex outline)."""
    ys = []
    n = len(poly)
    for k in range(n):
        (xa, ya), (xb, yb) = poly[k], poly[(k + 1) % n]
        if min(xa, xb) - 1e-9 <= x <= max(xa, xb) + 1e-9:
            ys += [ya, yb] if abs(xb - xa) < 1e-9 else [ya + (yb - ya) * (x - xa) / (xb - xa)]
    return min(ys), max(ys)


def _strips(poly, breaks, z0, z1, y_floor=None):
    """Box colliders inside the outline: one per strip between x breaks, as wide in y as the outline is at both ends.
    y_floor (x_until, y) raises the strips' low side to y where they end at or before x_until (the open corner)."""
    out = []
    for xa, xb in zip(breaks, breaks[1:]):
        (la, ha), (lb, hb) = _span(poly, xa), _span(poly, xb)
        lo, hi = max(la, lb), min(ha, hb)
        if y_floor is not None and xb <= y_floor[0] + 1e-9:
            lo = max(lo, y_floor[1])
        if hi - lo > 1e-4:
            out.append(((xa, lo, z0), (xb, hi, z1)))
    return out


def _housing():
    """The plywood base, the black box with its cut corners, the quart can under the tube and the tube's tab, in the
    base's frame (origin on the table at BASE_C)."""
    ox, oy = BASE_C
    poly = [(x - ox, y - oy) for x, y in _outline()]
    top = MS + BOX_H
    m = [kit.box((0.0, 0.0, PLATE_T / 2), (BASE[0], BASE[1], PLATE_T), PLY),
         _prism(poly, top - PLY_T, top, SHROUD)]  # the base and the box's top
    for k in range(len(poly)):  # the sides, 12 mm plywood; the two faces at the tube stop 20 cm above the base
        a, b = np.array(poly[k]), np.array(poly[(k + 1) % len(poly)])
        t = (b - a) / np.linalg.norm(b - a)
        inward = np.array([-t[1], t[0]]) * PLY_T / 2
        z0 = MS + CUT_Z if k in CORNER_FACES else MS
        zc = (z0 + top - PLY_T) / 2
        m.append(kit.band([(*(a + inward), zc), (*(b + inward), zc)], top - PLY_T - z0, PLY_T, SHROUD,
                          normal=(0, 0, 1)))
    # The quart can on the base under the tube, its rolled rims, and the lid the tube goes down through
    cx, cy = SX - ox, SY - oy
    m.append(kit.lathe([(0.0, MS), (CAN_R - 0.002, MS), (CAN_R, MS + 0.003), (CAN_R, MS + CAN_H - 0.006),
                        (CAN_R + 0.0015, MS + CAN_H - 0.004), (CAN_R + 0.0015, MS + CAN_H), (CAN_R - 0.005, MS + CAN_H),
                        (CAN_R - 0.005, MS + CAN_H - 0.003), (0.0, MS + CAN_H - 0.003)], CAN, center=(cx, cy, 0.0),
                       n=40))
    # The tube's tab, from the tube to the nearer corner face, screwed to it
    tube = np.array([cx, cy])
    face = []
    for i in CORNER_FACES:
        a, b = np.array(poly[i]), np.array(poly[(i + 1) % len(poly)])
        u = float(np.clip((tube - a) @ (b - a) / ((b - a) @ (b - a)), 0.0, 1.0))
        face.append(a + u * (b - a))
    f = min(face, key=lambda p: float(np.linalg.norm(p - tube)))
    d = (f - tube) / np.linalg.norm(f - tube)
    m.append(kit.band([(*(tube + d * (TUBE_R - 0.001)), TAB_Z), (*f, TAB_Z)], 0.012, 0.0015, STEEL,
                      normal=(0, 0, 1)))
    # Colliders: the base, the box as strips inside its outline (below 20 cm without the open corner), and the can as
    # four blocks round a 2.4 cm channel for the blade
    x0, x1 = BX0 - ox, BX1 - ox
    breaks = sorted(set([round(x0 + 0.02 * k, 6) for k in range(11)] + [round(x1 - 0.13 + 0.026 * k, 6) for k in range(6)]
                        + [x1]))
    cols = [kit.Collider((0.0, 0.0, PLATE_T / 2), size=(BASE[0], BASE[1], PLATE_T))]
    boxes = (_strips(poly, breaks, MS + CUT_Z, top) +
             _strips(poly, breaks, MS, MS + CUT_Z, y_floor=(x0 + CUT, BY0 + CUT - oy)))
    s2 = CAN_R / math.sqrt(2)
    boxes += [((cx - s2, cy - s2, MS), (cx - 0.012, cy + s2, MS + CAN_H)),
              ((cx + 0.012, cy - s2, MS), (cx + s2, cy + s2, MS + CAN_H)),
              ((cx - 0.012, cy - s2, MS), (cx + 0.012, cy - 0.012, MS + CAN_H)),
              ((cx - 0.012, cy + 0.012, MS), (cx + 0.012, cy + s2, MS + CAN_H))]
    for lo, hi in boxes:
        cols.append(kit.Collider(tuple((p + q) / 2 for p, q in zip(lo, hi)), size=tuple(q - p for p, q in zip(lo, hi))))
    return kit.Prop("block", pos=(*BASE_C, TOP), size=(BASE[0], BASE[1], 0.31), meshes=tuple(m), colliders=tuple(cols))


def _tab(center, w, t, h, r, look):
    """A flat tab with rounded corners: w across (x), t thick (y), h tall (z)."""
    cx, cy, cz = center
    parts = [kit.box(center, (w, t, h - 2 * r), look), kit.box(center, (w - 2 * r, t, h), look)]
    for sx in (-1, 1):
        for sz in (-1, 1):
            parts.append(kit.cylinder((cx + sx * (w / 2 - r), cy, cz + sz * (h / 2 - r)), r, t, look, axis="y", n=16))
    return parts


def _engine():
    """The fill tube and its dipstick, in the tube frame (origin at the mouth's middle, z straight up)."""
    meshes = (kit.cylinder((0.0, 0.0, -TUBE_IN / 2), TUBE_R, TUBE_IN, BLACK),  # the tube
              kit.lathe([(TUBE_R, -COLLAR_H - 0.01), (COLLAR_R, -COLLAR_H), (COLLAR_R, -0.003), (COLLAR_R - 0.002, 0.0),
                         (0.012, 0.0), (0.012, -0.005)], BLACK, n=32))  # the collar the cap seats on
    colliders = (kit.Collider((0.0, 0.0, -0.27), size=(0.02, 0.02, 0.006)),)  # the tube's base, below the blade's tip
    # Dipstick (link frame at the cap's underside, on the tube's axis): blade below, cap and T-handle above.
    marks = FULL_MARK - ADD_MARK  # the crosshatch between ADD and FULL near the tip
    cap = [(0.0, 0.0), (CAP_R - 0.001, 0.0), (CAP_R, 0.002)]
    for k in range(4):  # ribbed grip round the cap
        cap += [(CAP_R, 0.002 + 0.002 * k), (CAP_R - 0.001, 0.003 + 0.002 * k)]
    cap += [(CAP_R - 0.002, CAP_H), (0.0, CAP_H)]
    s_meshes = (
        kit.box((0.0, 0.0, -BLADE / 2), (BLADE_W, BLADE_T, BLADE), STEEL),
        kit.box((0.0, 0.0, -BLADE + ADD_MARK + marks / 2), (BLADE_W * 1.02, BLADE_T * 1.2, marks),
                kit.Look((0.45, 0.46, 0.48), 0.5, metallic=0.6)),
        # the ADD and FULL marks: black ticks 1.5 mm long standing 2 mm proud of the blade's edges, so they show
        # beside the oil band from every side
        kit.box((0.0, 0.0, -BLADE + ADD_MARK), (BLADE_W + 0.004, BLADE_T * 1.5, 0.0015), TICK),
        kit.box((0.0, 0.0, -BLADE + FULL_MARK), (BLADE_W + 0.004, BLADE_T * 1.5, 0.0015), TICK),
        kit.lathe(cap, YELLOW, n=32),
        *_tab((0.0, 0.0, HANDLE_C), HANDLE[0], HANDLE[1], HANDLE[2], 0.008, YELLOW),
    )
    s_colliders = (
        kit.Collider((0.0, 0.0, -BLADE / 2), size=(BLADE_W, BLADE_T, BLADE)),
        kit.Collider((0.0, 0.0, CAP_H / 2), size=(2 * CAP_R, 2 * CAP_R, CAP_H)),
        kit.Collider((0.0, 0.0, HANDLE_C), size=HANDLE),
    )
    # The stick has its weight (50 g): let go part way out, it slides back down the vertical tube onto its seat.
    stick = kit.Link("stick", "prismatic", "Z", (0.0, 0.0, 0.0), lower=0.0, upper=0.30, start=0.0, mass=0.05,
                     meshes=s_meshes, colliders=s_colliders, damping=0.5)
    return kit.Mechanism("engine", pos=tuple(float(v) for v in MOUTH_W), size=(0.04, 0.04, 0.60), tilt_xyzw=TILT, meshes=meshes,
                         colliders=colliders, links=(stick,))


def _rag():
    folds = [kit.box((0.0, 0.0, 0.0), RAG, RAGLOOK),
             kit.box((0.0, 0.0, RAG[2] / 2 - 0.004), (RAG[0] * 1.01, RAG[1] * 1.01, 0.001),
                     kit.Look((0.16, 0.29, 0.60), 0.9))]
    return kit.Prop("rag", pos=RAG_START, size=RAG, mass=0.04, meshes=tuple(folds),
                    colliders=(kit.Collider((0.0, 0.0, 0.0), size=RAG),))


SYM_Z, SYM_T, SYM_W, SYM_L = FLOOR + 0.0013, 0.0006, 0.006, 0.028  # label strokes: height, thickness, width, length


def _symbol(case, y):
    """The pocket's label, drawn in white on its floor so every camera can tell the pockets apart without color: a
    minus sign for ADD (too little oil), a check mark for OK and a plus sign for OVER (too much). Strokes are 6 mm wide
    and 2.8 cm long; the minus runs across the robot's view (along y)."""
    if case == "low":
        return (kit.box((0.0, y, SYM_Z), (SYM_W, SYM_L, SYM_T), LABEL),)
    if case == "over":
        return (kit.box((0.0, y, SYM_Z), (SYM_W, SYM_L, SYM_T), LABEL),
                kit.box((0.0, y, SYM_Z), (SYM_L, SYM_W, SYM_T), LABEL))
    # check mark, seen from the robot (x away, y to its left): a short stroke down to the right, a long one up
    path = [(0.001, y + 0.011, SYM_Z), (-0.007, y + 0.004, SYM_Z), (0.010, y - 0.012, SYM_Z)]
    return (kit.band(path, SYM_T, SYM_W, LABEL, normal=(0, 0, 1)),)


def _card():
    """The oil check card: a 5 mm base and 5 mm walls round three 4.4 cm pockets, each floor colored and labelled
    with a white symbol (minus ADD, check mark OK, plus OVER)."""
    cx, cy, h = CARD[0] / 2, CARD[1] / 2, CARD[2]
    p = POCKET / 2
    parts = [((-cx, -cy, 0.0), (cx, cy, FLOOR), CARDLOOK, True),  # base
             ((-cx, -cy, FLOOR), (-p, cy, h), CARDLOOK, True),  # near rail
             ((p, -cy, FLOOR), (cx, cy, h), CARDLOOK, True)]  # far rail
    ys = sorted(POCKETS[k][1] - CARD_C[1] for k in CASES)
    edges = [-cy] + [y for m in ys for y in (m - p, m + p)] + [cy]
    for a, b in zip(edges[0::2], edges[1::2]):  # the end walls and the walls between pockets
        parts.append(((-p, a, FLOOR), (p, b, h), CARDLOOK, True))
    meshes, colliders = _parts(parts)
    for k in CASES:  # colored pocket floors (visual, 1 mm), each with its white label symbol on top
        y = POCKETS[k][1] - CARD_C[1]
        meshes += (kit.box((0.0, y, FLOOR + 0.0005), (POCKET * 0.98, POCKET * 0.98, 0.001), POCKET_LOOK[k]),)
        meshes += _symbol(k, y)
    return kit.Prop("card", pos=(*CARD_C, TOP), size=CARD, meshes=meshes, colliders=colliders)


def _puck():
    return kit.Prop("puck", pos=PUCK_START, size=(2 * PUCK_R, 2 * PUCK_R, PUCK_H), mass=0.015,
                    meshes=(kit.cylinder((0.0, 0.0, 0.0), PUCK_R, PUCK_H, PUCKLOOK),
                            kit.cylinder((0.0, 0.0, PUCK_H / 2), PUCK_R * 0.35, 0.0006, DARK)),
                    colliders=(kit.Collider((0.0, 0.0, 0.0), radius=PUCK_R, height=PUCK_H),))


OBJECTS = [_housing(), _engine(), _rag(), _card(), _puck()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here; a scene reset (kit.episode()) starts a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.phase = "seated"  # seated (dirty) -> out (dirty, can be wiped) -> clean_out -> reseated -> second -> read
        self.swept = np.zeros(int(round(WIPE_SPAN / BIN)), bool)  # bins (from the tip) the rag's middle has slid over
        self.rubbed = np.zeros(int(round(WIPE_ZONE / BIN)), bool)  # bins the sliding pad has covered (oil drawing only)
        self.last_s, self.gap = None, 0  # the rag's middle along the blade at its last contact, steps since then
        self.read_steps = 0
        self.reseated = False  # wiped, then pushed back in by hand until the cap seated
        self.read = False  # pulled again after that seat and held out still for 1 s
        self.half_seat = False  # a wiped blade went back into the tube without seating and came out again
        self.final_seat = False  # after the reading, pushed back in by hand until the cap seated
        self.last_q, self.speed = None, 0.0
        self.case = None  # this trial's oil level: "low", "ok" or "over" (set at the first step)
        self.in_steps = 0  # control steps in a row the puck has been down in a pocket
        self.early = False  # the puck went into a pocket before the true reading: a guess, not a report
        self.blade_touched = False  # a gripper touched the blade while the dipstick was off its seat


_H = _History()


def _trial_case(rag_xy):
    """This trial's oil level. OIL_CASE=low|ok|over sets it (to run each case). Otherwise it comes from the seeded
    reset: every free object starts jittered by up to 3 cm, and the rag's offset from its place, in whole millimetres
    (x + y), modulo 3 picks the case, so it changes with the seed but the rag's position is no cue a camera could use.
    With no jitter (object_jitter=0) the rag is in place and the level is between the marks."""
    env = os.environ.get("OIL_CASE", "").strip().lower()
    if env in LEVELS:
        return env
    off = np.asarray(rag_xy, float)[:2] - np.asarray(RAG_START[:2])
    if np.linalg.norm(off) < 0.002:
        return "ok"
    return CASES[int(math.floor((off[0] + off[1]) / 0.001)) % 3]


def _pocket(objects):
    """The pocket the puck is down in (its middle within 1.2 cm of the pocket's middle, its bottom 2 mm or more below
    the card's top), or None."""
    pk = objects.get("puck")
    if pk is None or float(pk.low[2]) > TOP + CARD[2] - SUNK:
        return None
    p = np.asarray(pk.pos, float)
    for k, (x, y) in POCKETS.items():
        if abs(p[0] - x) <= IN_POCKET and abs(p[1] - y) <= IN_POCKET:
            return k
    return None


def _report(objects):
    """The level reported at the end: the pocket the puck rests in, let go (no gripper touching it)."""
    k = _pocket(objects)
    pk = objects.get("puck")
    if k is None or not pk.resting() or gk.touching(objects, "puck", PUCK_SHAPE):
        return None
    return k


def _q(objects):
    return kit.joint(objects, "engine/stick").pos


def _holding(objects):
    """A gripper holding the dipstick: both fingers touching its T-handle with their opening stopped by it, not an
    empty fist or an open hand beside it."""
    return bool(gk.gripping(objects, "engine/stick", HANDLE_SHAPE, HANDLE_GRIP))


def _wiped():
    """The rag's middle has slid over at least 90% of the blade's 6 cm oiled length, in contact, during this pull."""
    return _H.swept.sum() >= WIPE_FRAC * len(_H.swept) - 1e-9


def _mark(arr, a, b):
    a, b = max(int(round(a / BIN)), 0), min(int(round(b / BIN)), len(arr))
    if b > a:
        arr[a:b] = True


def _tube(p):
    """World point p in the tube frame: (across, along y, along the tube from the mouth)."""
    return R_TUBE.T @ (np.asarray(p, float) - MOUTH_W)


def _rag_pass(objects, q):
    """Track one pull's wipe, in the tube frame. The rag touches the blade when its middle is within 2.7 cm (RAG_TOUCH)
    of the blade's axis, its own shape (the pad as it lies, turned with the rag) is within 2 mm (RAG_GAP) of the blade's
    faces, and its extent along the tube overlaps the blade out of the mouth. Between two touching steps the stretch of
    blade its middle slid over is marked; a rag held still on the blade (a dab) marks nothing."""
    r = objects.get("rag")
    tip = q - BLADE  # the blade's tip along the tube from the mouth
    touch = False
    if r is not None:
        c = _tube(r.pos)
        rr = gk._rot(r.quat_xyzw)
        half = float(np.abs(U @ rr) @ (np.asarray(RAG) / 2))  # the pad's half extent along the tube
        st = objects.get("engine/stick")
        gap = (gk._box_gap(np.asarray(r.pos, float), rr, gk.Block((0.0, 0.0, 0.0), RAG), np.asarray(st.pos, float),
                           gk._rot(st.quat_xyzw), BLADE_SHAPE) if st is not None else 1.0)
        touch = (math.hypot(c[0], c[1]) <= RAG_TOUCH and gap <= RAG_GAP and c[2] - half < q
                 and c[2] + half > max(tip, 0.0))
    if not touch:
        _H.gap += 1
        _H.last_s = None  # contact lost: the next touch starts a new slide, and the gap is not credited
        return
    s = c[2] - tip  # the rag's middle along the blade, from the tip
    if _H.last_s is not None and abs(s - _H.last_s) <= SLIDE_MAX:
        a, b = sorted((_H.last_s, s))
        _mark(_H.swept, a, b)
        if b - a >= BIN / 2:  # the oil is rubbed off wherever the sliding pad's face passed (drawn only)
            _mark(_H.rubbed, max(a - half, -tip), b + half)
    _H.last_s, _H.gap = s, 0


def _reset_wipe():
    _H.swept[:] = False
    _H.rubbed[:] = False
    _H.last_s, _H.gap = None, 0


def _back_to_start():
    """Seated without a wipe before it (or dropped back in): the next pull is a first pull again."""
    _H.phase = "seated"
    _reset_wipe()


def _update(objects):
    global _H
    if _H.episode != kit.episode():
        _H = _History()
    if _H.case is None:
        r = objects.get("rag")
        _H.case = _trial_case(r.pos if r is not None else RAG_START)
    # A report made before the true reading is a guess: the puck down in any pocket for 2 steps before then marks it.
    _H.in_steps = _H.in_steps + 1 if _pocket(objects) is not None else 0
    if _H.in_steps >= DEBOUNCE and not _H.read:
        _H.early = True
    js = kit.joint(objects, "engine/stick")
    q, held = js.pos, _holding(objects)
    if q > SEAT and gk.touching(objects, "engine/stick", BLADE_SHAPE):  # the oily blade handled: smeared, no reading
        _H.blade_touched = True
    speed = abs(q - _H.last_q) * CONTROL_HZ if _H.last_q is not None else 0.0  # from the joint's travel per step
    _H.last_q, _H.speed = q, speed
    ph = _H.phase
    if ph == "seated":  # seated, blade smeared (the start, or after a seat that did not follow a wipe)
        if q >= OUT:
            _H.phase = "out"
    elif ph == "out":  # out with a smeared blade: the rag can wipe it
        _rag_pass(objects, q)
        if _wiped():
            _H.phase = "clean_out"
        elif q <= SEAT:
            _back_to_start()
        elif q < TIP_IN:  # the tip back in the tube picks up the splash again: a part wipe before it is lost
            _reset_wipe()
    elif ph in ("clean_out", "clean_in", "second", "second_in"):
        if ph == "second" and q >= OUT and speed < STILL and held:  # the true reading: out, held, still
            _H.read_steps += 1
            if _H.read_steps >= READ_STEPS:
                _H.phase, _H.read = "after_read", True
        elif ph == "second":
            _H.read_steps = 0
        if _H.phase != "after_read":
            if q <= SEAT:  # a clean blade seated by hand reads true on the next pull
                if held:
                    _H.phase = "reseated"
                    _H.reseated = _H.reseated or ph in ("clean_out", "clean_in")
                else:
                    _back_to_start()
            elif q < TIP_IN and ph in ("clean_out", "second"):
                _H.phase = "clean_in" if ph == "clean_out" else "second_in"
            elif q >= OUT and ph in ("clean_in", "second_in"):  # back in part way and out again: smeared again
                _H.phase, _H.half_seat = "out", True
                _reset_wipe()
    elif ph == "reseated":
        if q >= OUT:
            _H.phase, _H.read_steps = "second", 0
    elif ph == "after_read":  # the end state is checked in _job; the closing seat must be pushed in by hand
        if q <= SEAT and held:
            _H.final_seat = True
        elif q > SEAT:
            _H.final_seat = False

    _draw_oil(objects, q)


# --- The oil on the blade, drawn as markers (visual only; the scorer knows the level from the trial's case). A smeared
# blade shows a ragged amber film over its lower 6 cm, which disappears where the rag has passed; after a wipe and a full
# seat the blade shows a clean oil band from the tip up to this trial's level (below ADD, between the marks, or above FULL).
SMEAR = ((0.0, 0.060, 0.0013), (0.0, 0.045, 0.0016), (0.0, 0.030, 0.0018))  # (from, to, extra thickness) streaks
_VIS = {"markers": None, "failed": False}
_N_SEG = 24


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        oil = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in (0.85, 0.52, 0.06)),
                                          roughness=0.1, metallic=0.0)
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/World/Visuals/dipstick_oil", markers={"oil": sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0),
                                                                                        visual_material=oil)}))
    except Exception as e:  # visuals are optional
        print(f"dipstick: no oil visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _oil_segments():
    """(from, to, extra thickness) spans above the tip, in m, of oil to draw on the blade now."""
    if _H.phase in ("reseated", "second", "second_in", "after_read"):
        return [(0.0, LEVELS[_H.case or "ok"], 0.0012)]
    if _H.phase in ("clean_out", "clean_in"):
        return []
    segs = []
    for a, b, t in SMEAR:  # unwiped runs of each streak
        k0 = int(a / BIN)
        run = None
        for k in range(k0, int(b / BIN)):
            dirty = k >= len(_H.rubbed) or not _H.rubbed[k]
            if dirty and run is None:
                run = k
            if not dirty and run is not None:
                segs.append((run * BIN, k * BIN, t))
                run = None
        if run is not None:
            segs.append((run * BIN, b, t))
    return segs[:_N_SEG]


def _draw_oil(objects, q):
    if "isaaclab" not in sys.modules:
        return
    m = _markers()
    if m is None:
        return
    tip = q - BLADE  # along the tube from the mouth
    tr, sc = [], []
    for a, b, t in _oil_segments():
        tr.append(tuple(MOUTH_W + (tip + (a + b) / 2) * U))
        sc.append((BLADE_W + 0.001, BLADE_T + t, max(b - a, 1e-4)))  # the film wraps the blade's edges
    while len(tr) < _N_SEG:
        tr.append((0.0, 0.0, -10.0)), sc.append((1e-5, 1e-5, 1e-5))
    m.visualize(translations=np.array(tr, float), orientations=np.tile(TILT, (_N_SEG, 1)),
                scales=np.array(sc, float), marker_indices=np.zeros(_N_SEG, int))


def _let_go(objects):
    """No gripper touching the dipstick's cap or handle."""
    return not gk.touching(objects, "engine/stick", STICK_SHAPE)


def _rag_down(objects):
    """The rag at rest, its bottom 1 cm or less above the table, and no gripper touching it."""
    r = objects.get("rag")
    return (r is not None and float(r.low[2]) - TOP < RAG_DOWN and r.resting()
            and not gk.touching(objects, "rag", RAG_SHAPE))


def _job(objects):
    report = _report(objects)
    if _H.blade_touched:  # the blade gripped or touched by a gripper off its seat: handled by the handle only
        return 0.0
    if _H.early or (_pocket(objects) is not None and not _H.read):  # reported before the true reading: a guess
        return 0.0
    if report is not None and report != _H.case:  # a wrong report: worse than none, so guessing never pays
        return 0.0
    if not _H.reseated:  # never wiped and pushed back in: a first-pull reading, or nothing
        return 0.0
    if not _H.read:  # wiped and reinserted, but no true second reading (half-seated, or not read)
        return 0.25
    if report is None:  # read, but no level reported (puck not in a pocket, still held, or on a wall)
        return 0.5
    closed = _H.final_seat and _q(objects) <= SEAT and _let_go(objects)
    return 1.0 if closed and _rag_down(objects) else 0.75


def success(objects):
    """Job done (wiped, reseated, read, reseated, the right level reported, everything let go, rag down) and the
    dipstick still. Where the arms end up does not matter. Called first each step, so it also records the history."""
    _update(objects)
    return _job(objects) == 1.0 and _H.speed < STILL


def score(objects):
    """0 unless the blade was pulled out by the handle, wiped (the rag's middle slid, in contact, over at least 5.4 cm of
    the blade's 6 cm oiled length in that pull; a dab or a spot wipe is not a wipe) and pushed back in by the handle
    until the cap seated (within 3 mm). Also 0 if a gripper touched the blade while it was off its seat, for a wrong
    report (the puck left in another level's pocket) or a report made before the true reading (the puck in any pocket
    before then). 0.25 for the wipe and reseat without a
    true second reading (pulled again from a full seat and held out and still for 1 s). 0.5 for a true reading with
    no report (the puck not let go in a pocket). 0.75 for the right report with the dipstick not seated by hand and
    let go, or the rag not put down. 1.0 for all of it. Possible scores: 0, 0.25, 0.5, 0.75, 1."""
    return _job(objects)


# --- Scripted demo: the right arm draws the dipstick straight up by its handle and pushes it home, the left arm wipes
# the blade with the rag and reports with the puck.
R_PITCH, R_ROLL = -math.pi / 2, math.pi / 2  # the right grip level, pointing along +y (into the corner from the
# housing's open front side), its fingers closing along x across the handle's 1.8 cm thickness
A_HANDLE = 0.032  # the tool point 3.2 cm above the cap's underside (the handle's middle is at 2.75 cm): the pads (4 cm
# wide, here up and down) stay clear of the cap's disc
R_UP = 0.07  # the open hand waits this far above its grip on the handle, its fingers either side of the handle
R_OPEN = 0.45 * FINGER_OPEN  # opened 4.2 cm round the 1.8 cm handle: 1.2 cm each side, and the fingers clear of
# the housing box beside the corner
Q_PULL = 0.275  # each pull draws the cap 27.5 cm up: the blade's tip 3 cm out of the mouth (the tool point ~59 cm up,
# inside the arm's reach for this level grip, about 61-62 cm here)
DEMO_PUSH = -0.004  # push 4 mm past the seat so the cap is pressed down
L_PITCH = -1.3  # for the wipe the left wrist pitches about 75 deg: the gripper reaches in from the robot's side, level
# with the blade, and holds the pad on edge with its broad face to the blade's near face
RAG_DX = 0.0145  # the rag's middle 1.45 cm in front of (toward the robot from) the blade's axis: the pad, held on
# edge, reaches 1.25-1.7 cm out from its middle (more if it tilts in the grip), so its face rests on the blade's face
RAG_GRIP_DZ = 0.012  # the rag is picked with the tool point 1.2 cm above its middle, so the fingers hold only its upper
# half and the pad stands out past the fingertips
PITCHED_CARRY = (0.012, 0.011)  # in that grip the pad's middle rides this far along x and z of the tool point (as
# measured in the sim)
WIPE_A = (0.035, 0.11)  # the rag's middle set on the blade 5 mm above the tip (3 cm out of the mouth; a pad brought
# up from under the tip catches on its end) and drawn up to 8 cm above it, past the 6 cm oiled length with 2 cm to
# spare: the slid stretch is 7.5 cm. Drawn up the blade, the pad's leading edge rides off the blade instead of digging in
PUCK_GRIP_Z = PUCK_H + 0.003  # tool point 3 mm above the puck's top: the pads (13 mm past it) hold its upper 1 cm
PUCK_DROP_Z = FLOOR + PUCK_GRIP_Z + 0.002  # let go with the puck's bottom 2 mm above the pocket's floor
# The reading: while the right hand holds the dipstick out and still, the left arm brings its wrist camera up to the
# blade's oiled end (the oil band and the ADD and FULL marks) and holds it there; only then does it go for the puck.
READ_PITCH = -1.45  # the left wrist pitched about 83 deg: the camera, above the fingers, looks down about 32 deg
READ_AT = 0.016  # the camera aims at the blade 1.6 cm above its tip (the band and both marks, 0-2.7 cm, in view)
READ_D = 0.10  # m from the camera to that point: the 3 cm from the tip up fills about a quarter of the image's height
READ_BACK = 0.07  # the camera waits and leaves this much farther back along its line of sight
READ_S = 1.5  # s the camera is held still on the blade
def _on_tube(a, dx=0.0, dy=0.0, dz=0.0):
    """The world point a m along the tube's axis from the mouth's middle, shifted dx, dy, dz."""
    p = MOUTH_W + a * U
    return (float(p[0] + dx), float(p[1] + dy), float(p[2] + dz))


def _r(t, q, finger, label, up=0.0):
    """The right hand at the handle with the dipstick's cap q m above its seat (up: that much higher)."""
    return kit.Tilted(t, _on_tube(q + A_HANDLE, dz=up), finger, label, roll=R_ROLL, pitch=R_PITCH)


def _pull_and_push(t_hold, hold_label):
    """Down round the handle, grip it, draw the dipstick straight up, hold it out t_hold s, push it home, let go."""
    return [_r(1.0, 0.0, R_OPEN, "fingers down round the handle"),
            _r(0.6, 0.0, FINGER_CLOSED, "grip the handle"),
            _r(5.0, Q_PULL, FINGER_CLOSED, "draw the dipstick straight up"),
            _r(t_hold, Q_PULL, FINGER_CLOSED, hold_label),
            _r(5.0, DEMO_PUSH, FINGER_CLOSED, "push it back down the tube"),
            _r(0.8, DEMO_PUSH, FINGER_CLOSED, "press the cap down"),
            _r(0.5, DEMO_PUSH, R_OPEN, "let go"),
            _r(0.8, 0.0, R_OPEN, "hand up off the handle", up=R_UP)]


def _read_tcp(back=0.0):
    """The left tool point that puts the left wrist camera READ_D (+ back) m from the pulled-out blade, aimed at it."""
    from yam_arena.ik import quat_xyzw_to_matrix
    from yam_arena.station import wrist_camera
    import yam_arena.demo as ydemo

    cam = wrist_camera("left")  # its mount in the gripper link frame
    rg = kit._rot_xyz(0.0, READ_PITCH) @ ydemo.grip_rotation(0.0)
    rc = rg @ quat_xyzw_to_matrix(np.array(cam.rot_xyzw, float))
    look = rc @ np.array([0.0, 0.0, -1.0])  # the camera looks along its -z
    aim = MOUTH_W + (Q_PULL - BLADE + READ_AT) * U
    cam_w = aim - (READ_D + back) * look
    tcp = cam_w - rg @ (np.asarray(cam.pos, float) - np.array([0.0, 0.0, 0.134]))
    return tuple(float(v) for v in tcp)


def demo():
    right = [_r(3.0, 0.0, R_OPEN, "over the handle", up=R_UP)]
    t_out = 3.0 + 1.0 + 0.6 + 5.0  # the first pull is out

    ry, rz = RAG_START[1], RAG_START[2] - TOP
    # (in low and close to the robot first: a wider sweep brushed the left forearm past the housing's near corner)
    left = [Waypoint(2.0, (0.16, 0.18, TOP + 0.14), FINGER_OPEN, "in toward the robot, clear of the housing"),
            Waypoint(1.6, (RAG_START[0], ry, TOP + rz + 0.06), FINGER_OPEN, "over the rag"),
            Waypoint(1.0, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ), FINGER_OPEN, "down on the rag"),
            Waypoint(0.6, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ), FINGER_CLOSED, "pick up the rag"),
            Waypoint(1.0, (RAG_START[0], ry, TOP + 0.20), FINGER_CLOSED, "lift")]
    rag_tcp = lambda a, dx=0.0, dz=0.0: _on_tube(a, -RAG_DX - PITCHED_CARRY[0] + dx,  # noqa: E731
                                                  dz=-PITCHED_CARRY[1] + dz)
    wait = t_out - sum(w.duration for w in left)
    left += [Waypoint(max(wait - 3.0, 0.2), (RAG_START[0], ry, TOP + 0.20), FINGER_CLOSED, "wait with the rag"),
             kit.Tilted(3.0, rag_tcp(WIPE_A[0], -0.05), FINGER_CLOSED, "in front of the blade's tip", pitch=L_PITCH),
             kit.Tilted(1.0, rag_tcp(WIPE_A[0]), FINGER_CLOSED, "rag on the blade", pitch=L_PITCH),
             kit.Tilted(5.0, rag_tcp(WIPE_A[1]), FINGER_CLOSED, "wipe up the blade", pitch=L_PITCH),
             kit.Tilted(0.8, rag_tcp(WIPE_A[1], -0.05, 0.01), FINGER_CLOSED, "rag off", pitch=L_PITCH)]
    t_wiped = sum(w.duration for w in left)
    left += [Waypoint(2.5, (RAG_START[0], ry, TOP + rz + 0.05), FINGER_CLOSED, "rag back over its place"),
             Waypoint(1.0, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ + 0.002), FINGER_CLOSED, "rag down"),
             Waypoint(0.6, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ + 0.002), FINGER_OPEN, "let go of the rag"),
             Waypoint(0.8, (RAG_START[0], ry, TOP + rz + 0.07), FINGER_OPEN, "hand up")]

    right += _pull_and_push(max(t_wiped - t_out, 0.2), "hold it out for the wipe")
    t_out2 = sum(w.duration for w in right) + 1.0 + 0.6 + 5.0  # the second pull is out
    # The left wrist camera waits back along its line of sight, comes up to the blade once it is out and still, holds
    # READ_S s, and backs off; the right hand holds the dipstick out the whole time.
    t_look = 1.2  # s from the waiting pose to the reading pose
    left += [kit.Tilted(max(t_out2 - sum(w.duration for w in left), 2.0), _read_tcp(READ_BACK), FINGER_OPEN,
                        "wrist camera back from the blade", pitch=READ_PITCH),
             kit.Tilted(0.4, _read_tcp(READ_BACK), FINGER_OPEN, "wait for the blade to stop", pitch=READ_PITCH),
             kit.Tilted(t_look, _read_tcp(), FINGER_OPEN, "wrist camera up to the blade", pitch=READ_PITCH),
             kit.Tilted(READ_S, _read_tcp(), FINGER_OPEN, "read the level on the wrist camera", pitch=READ_PITCH),
             kit.Tilted(0.8, _read_tcp(READ_BACK), FINGER_OPEN, "camera back off", pitch=READ_PITCH)]
    t_read = sum(w.duration for w in left) - 0.8  # the reading is taken
    right += _pull_and_push(max(t_read + 0.3 - t_out2, 1.5), "hold still for the reading")
    right += [kit.Home(2.5), kit.Home(1.5, label="hold at home")]

    # The report. This scripted demonstration is told this trial's level (OIL_CASE, or between the marks with no
    # jitter): it shows the blade to its wrist camera but does not read the image, so it only shows the motions can be
    # done. The scored policy is told nothing and has to read the oil band off the blade (best seen on that camera).
    case = _trial_case(RAG_START[:2])
    px, py = PUCK_START[0], PUCK_START[1]
    kx, ky = POCKETS[case]
    left += [Waypoint(2.2, (px, py, TOP + PUCK_GRIP_Z + 0.06), FINGER_OPEN, "over the puck"),
             Waypoint(1.0, (px, py, TOP + PUCK_GRIP_Z), FINGER_OPEN, "down on the puck"),
             Waypoint(0.6, (px, py, TOP + PUCK_GRIP_Z), FINGER_CLOSED, "pick up the puck"),
             Waypoint(1.0, (px, py, TOP + PUCK_GRIP_Z + 0.05), FINGER_CLOSED, "lift the puck"),
             Waypoint(1.5, (kx, ky, TOP + PUCK_GRIP_Z + 0.04), FINGER_CLOSED, f"over the {POCKET_NAME[case]} pocket"),
             Waypoint(1.0, (kx, ky, TOP + PUCK_DROP_Z), FINGER_CLOSED, "puck into the pocket"),
             Waypoint(0.6, (kx, ky, TOP + PUCK_DROP_Z), FINGER_OPEN, "let go of the puck"),
             Waypoint(0.8, (kx, ky, TOP + PUCK_DROP_Z + 0.07), FINGER_OPEN, "hand up")]
    total = max(sum(w.duration for w in right), sum(w.duration for w in left) + 2.5)
    right[-1] = kit.Home(1.5 + total - sum(w.duration for w in right), label="hold at home")
    left += [kit.Home(2.5), kit.Home(max(total - sum(w.duration for w in left) - 2.5, 0.0) + 1.5, label="hold at home")]
    return {"left": left, "right": right}
