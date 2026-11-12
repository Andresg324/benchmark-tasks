"""Casting a concrete test cylinder, at 1:2.5 scale. A 4 x 8 in plastic cylinder mold (here 4.1 cm across inside,
8.1 cm deep) stands on the table next to a steel sample pan of fresh concrete, a scoop on its stand, a tamping rod
hanging in a rack and a hand float lying flat.

ASTM C31/C31M (making concrete test specimens in the field) fills a 4 x 8 in cylinder consolidated by rodding in two
layers: about half the mold, rodded 25 times throughout its depth with a 3/8 in rod, the strokes spread over the
cross-section, the mold's outside tapped 10 to 15 times; then the rest, rodded 25 times with each stroke reaching about
1 in into the layer below, tapped again; the top is then struck off flat. A cylinder filled in one go, rodded too few
times, in one spot or not through each layer keeps voids, breaks low in the compression test and can get good
concrete rejected.

The concrete is dosekit's scored bulk model (dosekit.py, on pourkit and yamkit): dips with the scoop, pours from the
tipped scoop into the mold, rod strokes that consolidate a band from just below the rod's tip up to the surface (each
logged with the layer it was made in, how deep it went and where), taps on the mold's outside (counted per layer),
and the strike-off under the float's edge. Visuals only:
concrete standing above the mold's rim is drawn orange, so heaped and struck read apart at 320x176.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (modeled props; nothing on disk changes)
import pourkit as pk  # noqa: E402,F401  (dosekit builds on it)
import dosekit as dk  # noqa: E402  (import last: its hook runs after yamkit's and pourkit's)
import guardkit as gk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Make a test cylinder from this batch of concrete."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# --- Scene, world coordinates (m): x forward from the arm-base midpoint, y to the robot's left, z up from the table.
# Scale 1:2.5 throughout: a 4 x 8 in mold (101.6 x 203.2 mm) is 40.6 x 81.3 mm here.
MOLD = (0.310, 0.050)  # the mold's axis
MOLD_R, MOLD_R_OUT = 0.0205, 0.0235  # 4.1 cm inside, 3 mm wall
MOLD_FLOOR, MOLD_H = 0.003, 0.084  # bottom 3 mm thick; 8.1 cm deep inside; rim 8.4 cm up
DEPTH = MOLD_H - MOLD_FLOOR

PAN = (0.320, -0.105)  # the sample pan's axis
PAN_R, PAN_R_OUT, PAN_H, PAN_FLOOR = 0.065, 0.068, 0.055, 0.003  # 13 cm steel pan, 5.5 cm deep
CONCRETE_G = 1300.0  # g of fresh concrete in the pan: 4.5 cm deep, top 4.8 cm up

STAND = (0.210, -0.160)  # the scoop stand; the scoop's handle center starts above it
HANDLE_Z = 0.0805  # handle axis height at the start (the handle rests on the stand's post)
HANDLE_R, HANDLE_L = 0.0125, 0.070  # round handle 2.5 cm across, along y
BOWL_Y, RIM_Z = 0.0775, -0.039  # the bowl's axis and rim, relative to the handle center
BOWL_R, BOWL_D = 0.018, 0.0260  # bowl 3.6 cm across inside, 2.6 cm deep: 26.5 mL level, about 31 mL heaped
BOWL_R_OUT = 0.0195
BOWL_BOTTOM = RIM_Z - BOWL_D - 0.0015
SCOOP_MASS = 0.050  # kg (weighted handle)

RACK = (0.205, 0.150)  # the rod rack: the rod hangs in its fork's slot at this x, y
FORK_Z = 0.098  # top of the fork's prongs; the rod's grip block rests on them
ROD_R = 0.0025  # 5 mm steel rod (3/8 in at 1:2.5 is 3.8 mm; thickened so the physics keeps it straight)
ROD_L = 0.086  # rod below the grip block: 8.6 cm (with the block, 11.1 cm overall; C31 asks for mold depth + 4 in)
BLOCK = (0.030, 0.030, 0.025)  # black grip block on the rod's top (a printed sleeve on the real rod)
ROD_MASS = 0.060
TIP = (0.0, 0.0, -(BLOCK[2] / 2 + ROD_L))  # the rod's rounded tip, from its origin (the block's center)
ROD0 = np.array([*RACK, TOP + FORK_Z + BLOCK[2] / 2])

FLOAT = (0.320, 0.220)  # the hand float lies flat here, its long side along x
PLATE = (0.100, 0.040, 0.004)  # aluminum blade 10 x 4 cm, 4 mm thick; its underside's long edge strikes off
POST = (0.012, 0.012, 0.020)
KNOB = (0.030, 0.030, 0.025)
FLOAT_MASS = 0.050

CONCRETE = dk.MATERIALS["concrete"]  # loose 2.20, dipped 2.22, packed 2.40 g/mL; repose 38 deg; press_gain 0.12
MOLD_BIN = dk.Bin("mold", sections=((MOLD_R, DEPTH),), floor=MOLD_FLOOR, scoop=False, dip_into=False, jolt_dv=None,
                  heap_color=(0.98, 0.55, 0.12))  # nothing is scooped back out of the mold
PAN_BIN = dk.Bin("pan", sections=((PAN_R, PAN_H - PAN_FLOOR),), floor=PAN_FLOOR, fill={"concrete": CONCRETE_G},
                 scoop=False, jolt_dv=None)
SCOOP_BIN = dk.Bin("scoop", obj="scoop", at=(0.0, BOWL_Y, RIM_Z), sections=((BOWL_R, BOWL_D),), floor=-BOWL_D,
                   jolt_dv=None)
ROD = dk.Presser("rod", tip=TIP)  # a rod: each stroke consolidates from 6 mm below its tip up to the surface
FLOAT_EDGE = dk.Edge("float", a=(-PLATE[0] / 2, 0.0, 0.0), b=(PLATE[0] / 2, 0.0, 0.0))
# Taps on the mold's side (C31: 10 to 15 light taps per layer, with a mallet or, on a plastic mold, an open hand):
# a tap is a gripper's outer surface meeting the mold's wall below the rim and drawing back (see _Taps). The closed
# hand's outline, from the YAM finger and gripper meshes, in mm: for each height above the tool point (along the hand,
# -12 = toward the fingertips), how far out the hand reaches across its closing axis (local x) at each distance
# 0, 4, ... 32 mm along the fingers' width (local y), and the other way round; 0 = no material there. The fingers
# form an open frame: 4 to 8 mm out near the tool point, 10 to 20 mm out on the side rails 1.6 to 4.4 cm up and
# 12 mm or more to the side, where a round mold's wall curves away. In the sim a straight-down hand pressed slowly on
# the mold stopped with its tool point 10.7 mm off the wall, where this outline just meets it.
HAND_X_BY_Y = ((-12, (4, 4, 4, 0, 0, 0, 0, 0, 0)), (-8, (5, 5, 5, 0, 0, 0, 0, 0, 0)), (-4, (5, 5, 5, 5, 0, 0, 0, 0, 0)),
               (0, (6, 6, 6, 6, 0, 0, 0, 0, 0)), (4, (6, 7, 7, 7, 5, 0, 0, 0, 0)), (8, (0, 7, 8, 8, 7, 0, 0, 0, 0)),
               (12, (0, 8, 8, 8, 8, 0, 0, 0, 0)), (16, (0, 0, 9, 9, 9, 9, 0, 0, 0)),
               (20, (0, 0, 10, 11, 11, 11, 0, 0, 0)), (24, (0, 0, 9, 12, 12, 12, 11, 0, 0)),
               (28, (0, 0, 0, 14, 14, 14, 14, 0, 0)), (32, (0, 0, 0, 14, 15, 15, 15, 0, 0)),
               (36, (0, 0, 0, 0, 17, 17, 17, 16, 0)), (40, (0, 0, 0, 0, 18, 18, 18, 18, 0)),
               (44, (0, 0, 0, 0, 20, 20, 20, 20, 0)), (48, (14, 14, 14, 21, 21, 21, 21, 21, 21)),
               (52, (14, 20, 21, 23, 23, 23, 23, 23, 23)), (56, (24, 24, 24, 24, 24, 24, 24, 24, 23)))
HAND_Y_BY_X = ((-12, (8, 8, 0, 0, 0, 0, 0, 0, 0)), (-8, (9, 9, 0, 0, 0, 0, 0, 0, 0)),
               (-4, (11, 11, 0, 0, 0, 0, 0, 0, 0)), (0, (13, 13, 10, 0, 0, 0, 0, 0, 0)),
               (4, (14, 14, 13, 0, 0, 0, 0, 0, 0)),
               (8, (16, 16, 15, 0, 0, 0, 0, 0, 0)), (12, (17, 17, 17, 0, 0, 0, 0, 0, 0)),
               (16, (19, 19, 19, 0, 0, 0, 0, 0, 0)), (20, (21, 21, 21, 20, 0, 0, 0, 0, 0)),
               (24, (22, 22, 22, 22, 0, 0, 0, 0, 0)), (28, (24, 24, 24, 24, 0, 0, 0, 0, 0)),
               (32, (25, 25, 25, 25, 25, 0, 0, 0, 0)), (36, (25, 27, 27, 27, 27, 0, 0, 0, 0)),
               (40, (0, 28, 28, 28, 28, 27, 0, 0, 0)), (44, (0, 30, 30, 30, 30, 30, 0, 0, 0)),
               (48, (0, 31, 31, 31, 31, 31, 0, 0, 0)), (52, (0, 33, 33, 33, 33, 33, 32, 0, 0)),
               (56, (0, 34, 34, 34, 34, 34, 32, 0, 0)))
_HAND_PTS = np.array(sorted({(sa * v, sb * 4 * k, -h) for h, row in HAND_X_BY_Y for k, v in enumerate(row) if v
                             for sa in (-1, 1) for sb in (-1, 1)}
                            | {(sb * 4 * k, sa * v, -h) for h, row in HAND_Y_BY_X for k, v in enumerate(row) if v
                               for sa in (-1, 1) for sb in (-1, 1)})) / 1000.0  # m, the tool point's frame
TAP_GAP = 0.0015  # m: a hand point within 1.5 mm of the wall's outer face touches it (the outline is rounded to 1 mm)
TAP_BACK = 0.010  # m: it counts again only after the hand draws back 1 cm (a bounce is one tap)
dk.setup(materials=[CONCRETE], bins=[MOLD_BIN, PAN_BIN, SCOOP_BIN], tools=[ROD, FLOAT_EDGE], floor_z=TOP)

# --- Scoring thresholds (reasons in score() and the README).
LAYER_MIN = 0.25 * DEPTH  # a lift at least a quarter of the mold deep is a layer; less is a top-up
L1_LO, L1_HI = 0.35 * DEPTH, 0.65 * DEPTH  # C31: the first layer is about half the mold
STROKES = 25  # C31: 25 strokes per layer for a 4 x 8 in cylinder
FLOOR_REACH = 0.008  # m: a first-layer stroke reaches "throughout its depth" with the tip within 8 mm of the floor
INTO_BELOW = 0.009  # m: a second-layer stroke reaches at least 9 mm into the first layer (C31: about 1 in, 1.0 cm at
# 1:2.5; 1 mm under it for the rod's give at the bottom of a stroke)
CENTER_R = 0.005  # m: strokes within 5 mm of the axis are in the center zone
SECTOR_R = 0.009  # m: strokes at least 9 mm off it (just under half the 2.05 cm bore radius) in one of four sectors;
# strokes in between count in no zone, so a cluster around the axis is not "over the cross-section"
TOPUP_MAX = 0.005  # m: all material added after the second layer is a top-up only up to 5 mm in all (C31: a small
# quantity to bring the top level before striking off); more is an unrodded third layer, however it is split
STRIKE_SHARE = 0.9  # the float's edge must pass over at least 90% of the mouth after the last stroke or addition
ZONES_MIN, ZONE_STROKES = 4, 3  # C31 "distributed uniformly over the cross-section": 4 of the 5 zones, 3+ strokes each
TAPS_MIN, TAPS_MAX = 10, 15  # C31: tap the mold's outside 10 to 15 times after rodding each layer
LEVEL_TOL, FILL_TOL = 0.002, 0.02  # struck: nothing over 2 mm above the rim, and filled to within 2% of the brim
HANDS_OFF = 0.040  # m: a tool point within 4 cm of a tool's grip is still holding it
HANDS_ON_PART = 0.025  # m: or within 2.5 cm of any other part of it (a 5 mm rod or the bowl's wall between the pads)
DT = 0.05
HOME_TOL = 0.03
HOME_STILL = 0.002
GRIPPERS = ("left_gripper", "right_gripper")

BLACK = kit.Look((0.06, 0.06, 0.07), 0.6)
STEEL = kit.Look((0.78, 0.79, 0.80), 0.35, metallic=0.6)
WHITE = kit.Look((0.93, 0.93, 0.92), 0.6)
RED = kit.Look((0.75, 0.12, 0.10), 0.5)
GRIP = kit.Look((0.10, 0.10, 0.11), 0.7)
TAPE = kit.Look((0.95, 0.85, 0.10), 0.6)
ALU = kit.Look((0.82, 0.83, 0.85), 0.3, metallic=0.7)
WOOD = kit.Look((0.62, 0.45, 0.28), 0.7)
GRAY = kit.Look((0.45, 0.46, 0.48), 0.5)


def _ring(r_out, h, s=0.004, n=48, z0=0.0):
    """A round wall as a ring of small boxes (colliders are axis-aligned)."""
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        r = r_out - s / 2
        out.append(kit.Collider((r * math.cos(a), r * math.sin(a), z0 + h / 2), size=(s, s, h)))
    return out


def _mold():
    """A fixed black plastic cylinder mold, open on top."""
    m = (kit.lathe([(0.0, 0.0), (MOLD_R_OUT, 0.0), (MOLD_R_OUT, MOLD_H), (MOLD_R, MOLD_H), (MOLD_R, MOLD_FLOOR),
                    (0.0, MOLD_FLOOR)], BLACK),)
    c = [kit.Collider((0, 0, MOLD_FLOOR / 2), radius=MOLD_R_OUT, height=MOLD_FLOOR)] + _ring(MOLD_R_OUT, MOLD_H, 0.003)
    return kit.Prop("mold", pos=(*MOLD, TOP), size=(2 * MOLD_R_OUT, 2 * MOLD_R_OUT, MOLD_H), meshes=m,
                    colliders=tuple(c))


def _pan():
    """A fixed steel sample pan holding the batch."""
    m = (kit.lathe([(0.0, 0.0), (PAN_R_OUT, 0.0), (PAN_R_OUT, PAN_H), (PAN_R, PAN_H), (PAN_R, PAN_FLOOR),
                    (0.0, PAN_FLOOR)], STEEL),)
    c = [kit.Collider((0, 0, PAN_FLOOR / 2), radius=PAN_R_OUT, height=PAN_FLOOR)] + _ring(PAN_R_OUT, PAN_H)
    return kit.Prop("pan", pos=(*PAN, TOP), size=(2 * PAN_R_OUT, 2 * PAN_R_OUT, PAN_H), meshes=m, colliders=tuple(c))


def _scoop():
    """A steel scoop: round bowl, its red handle rising from the rim; origin at the handle's center."""
    neck_a, neck_b = np.array([0.0, HANDLE_L / 2 - 0.002, -0.004]), np.array([0.0, BOWL_Y - BOWL_R_OUT + 0.001, RIM_Z])
    neck = neck_b - neck_a
    m = (kit.cylinder((0, 0, 0), HANDLE_R, HANDLE_L, RED, axis="y"),
         kit.cylinder(tuple((neck_a + neck_b) / 2), 0.005, float(np.linalg.norm(neck)), STEEL, axis=neck),
         kit.lathe([(0.0, BOWL_BOTTOM), (BOWL_R_OUT, BOWL_BOTTOM), (BOWL_R_OUT, RIM_Z), (BOWL_R, RIM_Z),
                    (BOWL_R, RIM_Z - BOWL_D), (0.0, RIM_Z - BOWL_D)], STEEL, center=(0.0, BOWL_Y, 0.0)))
    c = (kit.Collider((0, 0, 0), radius=HANDLE_R, height=HANDLE_L, axis="Y"),
         kit.Collider((0.0, BOWL_Y, (RIM_Z + BOWL_BOTTOM) / 2), radius=BOWL_R_OUT, height=RIM_Z - BOWL_BOTTOM))
    return kit.Prop("scoop", pos=(*STAND, TOP + HANDLE_Z), size=(0.04, 0.20, 0.13), mass=SCOOP_MASS, meshes=m,
                    colliders=c)


def _stand():
    """A fixed white scoop stand: a narrow post under the handle and a low pad under the bowl."""
    post_h = HANDLE_Z - HANDLE_R - 0.0005
    pad_h = HANDLE_Z + BOWL_BOTTOM - 0.0005
    m, c = [], []
    for center, size in (((0.0, -0.002, post_h / 2), (0.016, 0.066, post_h)),
                         ((0.0, BOWL_Y, pad_h / 2), (0.035, 0.035, pad_h))):
        m.append(kit.box(center, size, WHITE))
        c.append(kit.Collider(center, size=size))
    return kit.Prop("stand", pos=(*STAND, TOP), size=(0.04, 0.16, post_h), meshes=tuple(m), colliders=tuple(c))


def _rod():
    """The tamping rod: a steel rod with a rounded tip under a black grip block; origin at the block's center. A
    yellow tape band sits 8.1 cm above the tip (it reaches the rim when the tip is on the mold's floor)."""
    zb = -BLOCK[2] / 2
    m = (kit.box((0, 0, 0), BLOCK, GRIP),
         kit.cylinder((0, 0, zb - ROD_L / 2 + ROD_R / 2), ROD_R, ROD_L - ROD_R, STEEL),
         kit.lathe([(0.0, TIP[2]), (ROD_R * 0.7, TIP[2] + ROD_R * 0.3), (ROD_R, TIP[2] + ROD_R)], STEEL),
         kit.cylinder((0, 0, TIP[2] + DEPTH), ROD_R + 0.0004, 0.004, TAPE))
    c = (kit.Collider((0, 0, 0), size=BLOCK),
         kit.Collider((0, 0, zb - ROD_L / 2), radius=ROD_R, height=ROD_L))
    return kit.Prop("rod", pos=tuple(float(v) for v in ROD0), size=(BLOCK[0], BLOCK[1], BLOCK[2] + ROD_L),
                    mass=ROD_MASS, meshes=m, colliders=c)  # plain floats: a float64 pose fails Isaac Lab's reset


def _rack():
    """A fixed wooden rod rack: a post with a fork on top whose slot opens toward the robot (-x); the rod hangs in
    the slot by its grip block."""
    prong_y, prong_w, prong_t, prong_x = 0.012, 0.012, 0.005, (-0.025, 0.0375)
    parts = [((0.045, 0.0, FORK_Z / 2), (0.015, 0.050, FORK_Z)),  # post, 2.25 cm behind the block
             ((0.035, 0.0, 0.003), (0.070, 0.060, 0.006))]  # foot
    for s in (-1, 1):
        cx = (prong_x[0] + prong_x[1]) / 2
        parts.append(((cx, s * prong_y, FORK_Z - prong_t / 2), (prong_x[1] - prong_x[0], prong_w, prong_t)))
    m = tuple(kit.box(c, s, WOOD) for c, s in parts)
    c = tuple(kit.Collider(c, size=s) for c, s in parts)
    return kit.Prop("rack", pos=(*RACK, TOP), size=(0.11, 0.06, FORK_Z), meshes=m, colliders=c)


def _float():
    """A hand float: a flat aluminum blade with a post and a black knob; origin at the blade's underside center."""
    parts = [((0, 0, PLATE[2] / 2), PLATE, ALU), ((0, 0, PLATE[2] + POST[2] / 2), POST, GRAY),
             ((0, 0, PLATE[2] + POST[2] + KNOB[2] / 2), KNOB, GRIP)]
    m = tuple(kit.box(c, s, lk) for c, s, lk in parts)
    c = tuple(kit.Collider(c, size=s) for c, s, _ in parts)
    return kit.Prop("float", pos=(*FLOAT, TOP), size=(PLATE[0], PLATE[1], PLATE[2] + POST[2] + KNOB[2]),
                    mass=FLOAT_MASS, meshes=m, colliders=c)


OBJECTS = [_mold(), _pan(), _scoop(), _stand(), _rod(), _rack(), _float()]
KNOB_C = np.array([0.0, 0.0, PLATE[2] + POST[2] + KNOB[2] / 2])  # the float's knob center, from its origin
_SCOOP_PTS = np.array([(0.0, y, 0.0) for y in np.arange(-HANDLE_L / 2, HANDLE_L / 2 + 1e-9, 0.01)]
                      + [(0.0, BOWL_Y, RIM_Z - BOWL_D / 2), (0.0, BOWL_Y - BOWL_R_OUT, RIM_Z / 2)])
_ROD_PTS = np.array([(0.0, 0.0, z) for z in np.arange(0.0, TIP[2] - 1e-9, -0.01)] + [TIP])
_FLOAT_PTS = np.array([(x, 0.0, PLATE[2] / 2) for x in np.arange(-PLATE[0] / 2, PLATE[0] / 2 + 1e-9, 0.01)]
                      + [tuple(KNOB_C)])


# Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still (guardkit keeps
# the start pose per trial and starts afresh on every scene reset).
HOME = gk.ArmsHome(tol=HOME_TOL, still=HOME_STILL)


class _Adds:
    """The kit's step at which concrete last went into the mold, per trial."""

    def __init__(self):
        self.ep, self.g, self.step = None, 0.0, -1


_ADDS = _Adds()


class _Taps:
    """Taps on the mold's outside, per trial: (lift on top, step). A gripper taps when some point of its closed hand's
    outline (HAND_PROFILE, turned with the hand) below the rim comes within TAP_GAP of the wall's outer face, its tool
    point outside the wall; it can tap again once every point is TAP_BACK further out."""

    def __init__(self):
        self.ep, self.log, self.touching, self.gap = None, [], {}, {}

    def feed(self, objects):
        if self.ep != kit.episode():
            self.ep, self.log, self.touching, self.gap = kit.episode(), [], {}, {}
        for g in GRIPPERS:
            if g not in objects:
                continue
            s = objects[g]
            tcp = np.asarray(s.pos, float)
            pts = tcp + (pk._rotm(s.quat_xyzw) @ _HAND_PTS.T).T
            below = pts[(pts[:, 2] >= TOP) & (pts[:, 2] <= RIM - 0.001)]
            r_tcp = math.hypot(tcp[0] - MOLD[0], tcp[1] - MOLD[1])
            gap = (float(np.min(np.hypot(below[:, 0] - MOLD[0], below[:, 1] - MOLD[1]))) - MOLD_R_OUT
                   if len(below) and r_tcp >= MOLD_R_OUT else math.inf)
            self.gap[g] = gap
            if not self.touching.get(g) and gap <= TAP_GAP:
                self.touching[g] = True
                L = dk.lifts(objects, "mold")
                self.log.append((L[-1].index if L else -1, dk._S.step))
            elif self.touching.get(g) and gap > TAP_GAP + TAP_BACK:
                self.touching[g] = False


TAPS = _Taps()


def _update(objects):
    HOME.feed(objects)
    TAPS.feed(objects)
    if _ADDS.ep != kit.episode():
        _ADDS.ep, _ADDS.g, _ADDS.step = kit.episode(), 0.0, -1
    g = dk.amount(objects, "mold")
    if g > _ADDS.g + 0.5:  # 0.5 g in all since the last mark: a slow trickle is caught too
        _ADDS.step, _ADDS.g = dk._S.step, g
    elif g < _ADDS.g:
        _ADDS.g = g


def struck_after_last(objects):
    """Share of the mold's mouth the float's edge passed over at striking height after the last rod stroke and the
    last concrete added."""
    log = dk.stroke_log(objects, "mold")
    since = max(log[-1].step if log else -1, _ADDS.step)
    return dk.swept_since(objects, "mold", since)


def _arms_home(objects):
    return HOME.home(objects)


def _grips(objects):
    """Where each tool is held: the scoop's handle center, the rod's grip block, the float's knob (world)."""
    f = objects["float"]
    return (np.asarray(objects["scoop"].pos, float), np.asarray(objects["rod"].pos, float),
            np.asarray(f.pos, float) + pk._rotm(f.quat_xyzw) @ KNOB_C)


def _parts(objects):
    """Points 1 cm apart along each loose tool (world): the scoop's handle, neck and bowl, the rod from its block to
    its tip, the float's blade and knob. A hand closed on any of them is still holding the tool."""
    out = []
    for name, pts in (("scoop", _SCOOP_PTS), ("rod", _ROD_PTS), ("float", _FLOAT_PTS)):
        s = objects[name]
        out += list(np.asarray(s.pos, float) + (pk._rotm(s.quat_xyzw) @ pts.T).T)
    return out


def _held(objects):
    """A tool still in hand: some tool point within 4 cm of a tool's grip (scoop handle's center, rod's block, float's
    knob), or within 2.5 cm of any other part of it (the fingers closed on the rod's shaft or the scoop's bowl), or a
    tool point or pad tip within 1 cm of a tool's outline."""
    tcps = [np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects]
    return (any(np.linalg.norm(t - p) < HANDS_OFF for t in tcps for p in _grips(objects))
            or any(np.linalg.norm(t - p) < HANDS_ON_PART for t in tcps for p in _parts(objects))
            or not all(gk.released_clear(objects, t, None, 0.01) for t in ("scoop", "rod", "float")))


def _in_mold(objects, p):
    """Point p within the mold's outline and below 3 cm over its rim: left standing in or lying on the mold."""
    return (math.hypot(p[0] - MOLD[0], p[1] - MOLD[1]) < MOLD_R_OUT + 0.005 and p[2] < TOP + MOLD_H + 0.03)


def rod_tip(objects):
    s = objects["rod"]
    return np.asarray(s.pos, float) + pk._rotm(s.quat_xyzw) @ np.asarray(TIP)


def zone(s):
    """Where a stroke went in: "center" within 5 mm of the axis, the 90-degree sector around +x, +y, -x or -y at 9 mm
    or more, else None."""
    if s.r < CENTER_R:
        return "center"
    if s.r < SECTOR_R:
        return None
    return ("+x", "+y", "-x", "-y")[int(((s.ang + math.pi / 4) % (2 * math.pi)) // (math.pi / 2))]


def spread(strokes):
    """True when the strokes cover at least 4 of the 5 zones with at least 3 strokes each (strokes 5 to 9 mm off the
    axis count in none)."""
    n = {}
    for s in strokes:
        z = zone(s)
        if z is not None:
            n[z] = n.get(z, 0) + 1
    return sum(1 for v in n.values() if v >= ZONE_STROKES) >= ZONES_MIN


def layer_taps(objects, lift, after_step):
    """Taps on the mold made while `lift` was on top, after step `after_step` (its 25th stroke)."""
    return sum(1 for t_lift, t_step in TAPS.log if t_lift == lift and t_step > after_step)


def layers_ok(objects):
    """ASTM C31's two layers for a rodded 4 x 8 in cylinder, from the kit's lift, stroke and tap records. Returns
    (ok, why)."""
    L = dk.lifts(objects, "mold")
    if len(L) < 2:
        return False, "fewer than two layers"
    first, second = L[0], L[1]
    if first.top - first.bottom < LAYER_MIN or second.top - second.bottom < LAYER_MIN:
        return False, "a first or second lift under a quarter of the mold (not a layer)"
    if any(x.top - x.bottom >= LAYER_MIN for x in L[2:]):
        return False, "three or more layers"
    if sum(x.top - x.bottom for x in L[2:]) > TOPUP_MAX:
        return False, "over 5 mm topped up after the second layer (an unrodded third layer)"
    if not L1_LO <= first.top <= L1_HI:
        return False, "first layer not about half the mold"
    log = dk.stroke_log(objects, "mold")
    s1 = [s for s in log if s.lift == first.index and s.deepest <= FLOOR_REACH]
    if len(s1) < STROKES:
        return False, f"first layer rodded {len(s1)} times through its depth"
    s2 = [s for s in log if s.lift == second.index and s.lift_bottom - s.deepest >= INTO_BELOW]
    if len(s2) < STROKES:
        return False, f"second layer rodded {len(s2)} times into the first"
    if not spread(s1) or not spread(s2):
        return False, "strokes not spread over the cross-section"
    for name, lift, ss in (("first", first.index, s1), ("second", second.index, s2)):
        n = layer_taps(objects, lift, ss[STROKES - 1].step)
        if not TAPS_MIN <= n <= TAPS_MAX:
            return False, f"{name} layer tapped {n} times after rodding"
    return True, "two layers, each rodded 25 times over the cross-section and tapped 10 to 15 times"


def _job(objects):
    if _held(objects):
        return 0.0  # nothing counts while a tool is still in hand
    for t in ("scoop", "rod", "float"):
        s = objects[t]
        if not s.resting(0.01, 0.5) or s.pos[2] < TOP - 0.01:
            return 0.0  # a tool still moving, or dropped off the table
    if _in_mold(objects, rod_tip(objects)) or _in_mold(objects, np.asarray(objects["scoop"].pos)
                                                       + np.array([0.0, BOWL_Y, RIM_Z])):
        return 0.0  # the rod or the scoop left in the mold
    f = objects["float"]
    if any(_in_mold(objects, p) for p in np.asarray(f.pos, float) + (pk._rotm(f.quat_xyzw) @ _FLOAT_PTS.T).T):
        return 0.0  # the float left lying on the mold
    ok, _ = layers_ok(objects)
    if not ok:
        return 0.0  # filled or rodded wrong: the specimen is not a C31 cylinder, whatever its top looks like
    if (dk.level_struck(objects, "mold", tol=LEVEL_TOL, fill_tol=FILL_TOL)
            and struck_after_last(objects) >= STRIKE_SHARE):
        return 1.0
    return 0.25  # consolidated right, but the top is heaped or sunk (not struck off flat and full)


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """Two layers, each rodded 25 times through its depth (the top one into the bottom one) over the cross-section
    and tapped 10 to 15 times, the top struck off
    flat and full, every tool set down, both arms home and still. Called first each step, so it also records the
    history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """Scored on the final state from the kit's record of how the mold was filled and rodded. A layer is a lift
    (concrete added between rod strokes) at least a quarter of the mold deep; up to 5 mm in all added after the
    second layer is a top-up, which C31 allows. 0 unless: exactly two layers and no more than 5 mm on top of them;
    the first layer's top 35% to 65% of the way up the mold (C31: about half); at least 25 strokes made while the
    first layer was on top with the rod's tip within 8 mm of the floor (C31: 25 strokes throughout its depth); at
    least 25 strokes made while the second layer was on top reaching at least 9 mm below its bottom (C31: about 1
    in, 1.0 cm at 1:2.5); each layer's 25 strokes spread over the cross-section (at least 3 strokes in each of at
    least 4 of 5 zones: within 5 mm of the axis, and the four 90-degree sectors 9 mm or more off it; C31: uniformly
    over the cross-section); and 10 to 15 taps on the mold's outside below the rim (the hand's outline within 2 mm
    of the wall, then drawn back 1 cm) after each layer's 25th stroke and before the next layer (C31). Also 0 while
    a tool is still held (see _held), while a tool is still moving or off the table, or with the rod or the scoop
    left in the mold or the float lying on it. Then: 1.0 when the float's edge was drawn over at least 90% of the
    mouth after the last stroke and the last concrete added, and the top is struck (nothing more than 2 mm above the
    rim, filled to within 2% of the brim), and both arms are back home and still; 0.9 for the same with the arms not
    home; 0.25 for the layers right but the top heaped, sunk or not struck off. Doing nothing scores 0. Possible
    scores: 0, 0.25, 0.9, 1.0."""
    return _score(objects)


# --- The demo. Right arm: the scoop, with the formula pilot's side grip on its round handle from the -y side (yaw 90
# deg, roll 1.35 rad, pitch +0.5 rad; tipping it toward -x is a turn of the wrist about world y). Left arm: the rod
# and the float, gripped on their blocks with the hand leaning 29 deg. The arms take turns over the mold; the idle one
# waits away from it.
YAW, ROLL, P0 = math.pi / 2, 1.35, 0.5
POUR_TURN = 2.0  # rad (115 deg, toward -x: the wrist turns from +0.5 to -1.5 rad, where joint4 stops): past 38 + 3 deg
GRIP_DY = -0.015  # the tool point sits 1.5 cm behind the handle's center
HANDLE0 = np.array([*STAND, TOP + HANDLE_Z])
TCP0 = HANDLE0 + [0.0, GRIP_DY, 0.0]
MOUTH_OFF = np.array([0.0, BOWL_Y, RIM_Z])
RIM = TOP + MOLD_H
PAN_SURF = TOP + PAN_FLOOR + CONCRETE_G / CONCRETE.loose / 1e6 / (math.pi * PAN_R ** 2)
DIP_XY = (PAN[0], PAN[1] + 0.035)  # the bowl dips 3.5 cm to the +y side of the pan's axis (the hand stays outside)
DIP_DEPTH = 0.008
SCOOP_ML = 31.0  # what one heaped scoop takes out of the pan (to lower each next dip)
CARRY_Z = TOP + 0.12  # the scoop's rim height while carried (its bottom 3.6 cm over the mold's rim)
POUR_LIP_Z = RIM + 0.035
PAD_DOWN = 0.016 - 0.013  # top-down grips: the tool point 3 mm under a block's top (pads 1.6 cm down it)
ROD_TCP = np.asarray(ROD0) + [0.0, 0.0, BLOCK[2] / 2 - PAD_DOWN]  # tool point gripping the hanging rod
ROD_DZ = ROD_TCP[2] - (ROD0[2] + TIP[2])  # tool point above the rod's tip: 10.8 cm
FLOAT_TCP = np.array([*FLOAT, TOP + PLATE[2] + POST[2] + KNOB[2] - PAD_DOWN])
FLOAT_DZ = FLOAT_TCP[2] - TOP  # tool point above the blade's underside
SPOKES = [(0.013, 0.0), (0.0, 0.013), (-0.013, 0.0), (0.0, -0.013)]  # rod points 13 mm off the axis (the rod's edge
# 5 mm inside the wall; the sector zones start at 9 mm)
FIRST_PTS = SPOKES + [(0.0, 0.0)]
# Second layer: 12 mm spokes and the center, 5 strokes each, the whole pattern shifted 3 mm toward +x and -y. In the
# sim the second layer's 13 mm strokes (each coming straight down from above the rim) landed 1 to 5 mm toward -x and
# 2 to 5 mm toward +y of their aim: the +x and -y strokes 8 to 10 mm off the axis, the +y ones 17 mm (the rod's edge
# 1 mm from the wall), and the rod turned in the grip after 9 strokes.
# In the first 1.0 run the +x strokes still landed only 9.1 to 9.4 mm out (the other sectors 11 to 13 mm), so the +x
# stroke's bottom is aimed 2 mm further out (its entry above the rim unchanged, the rod's edge 4 mm inside the wall).
L2_SHIFT = (0.003, -0.003)
L2_PTS = [(0.012, 0.0, 0.002, 0.0), (0.0, 0.012), (-0.012, 0.0), (0.0, -0.012), (0.0, 0.0)] * 5
L1_TIP_LO = TOP + MOLD_FLOOR + 0.005  # first layer: aimed 5 mm above the floor (C31: don't strike the bottom; in
# the sim a tip that struck the floor turned the rod in the grip)
L1_TIP_HI = TOP + MOLD_FLOOR + 0.054  # first layer: up past its loose top (about 5 cm)
L2_INTO = 0.011  # second layer: the tip 1.1 cm below the first layer's top
L2_TIP_HI = RIM + 0.030  # second layer: up past its heap (the hand lags 1.2 cm on the way up; the tip clears the rim
# by about 1.8 cm before the move over; at 2 cm with a 0.15 s move the rod swung and turned in the grip)
L1_TOP_EST = 0.041  # the first layer's top after rodding, from the mold's floor (4.1 cm; checked offline)


def tcp_for_mouth(mouth):
    """Tool point that puts the level scoop's rim center at `mouth`."""
    return tuple(np.asarray(mouth, float) - MOUTH_OFF + [0.0, GRIP_DY, 0.0])


def _w(dur, pos, finger, label, pitch=P0):
    return kit.Tilted(dur, tuple(float(v) for v in pos), finger, label, YAW, ROLL, pitch)


def _down(dur, pos, finger, label):
    """A straight-down grip waypoint (fingers closing along y)."""
    return kit.Tilted(dur, tuple(float(v) for v in pos), finger, label, 0.0, 0.0, 0.0)


LEFT_PITCH = -0.5  # rad: the left hand leans 29 deg, its fingers toward +x; held straight down over the mold its
# tool point tops out about 17 cm up, short of the rod's 20 cm over the second layer, leaning it reaches 26 cm


def _left(dur, pos, finger, label):
    """A left-arm waypoint: the leaning grip on the rod's block or the float's knob (fingers closing along y). The
    hand takes the block or knob already leaning, so the rod hangs plumb and the float lies flat while held."""
    return kit.Tilted(dur, tuple(float(v) for v in pos), finger, label, 0.0, 0.0, LEFT_PITCH)


ROD_ROLL = -0.5  # rad: for the rod the hand turns 90 deg (fingers closing along x) and leans 29 deg toward -y instead:
# the pads then hold the block on its x faces, so the rod's swing along x (the carry, a stroke jammed on the floor)
# turns against the pads' faces, not about their pinch axis (in the sim a block pinched on its y faces slipped 8 deg)


def _rodw(dur, pos, finger, label):
    """A left-arm waypoint holding the rod: fingers closing along x, the hand leaning toward -y."""
    return kit.Tilted(dur, tuple(float(v) for v in pos), finger, label, math.pi / 2, ROD_ROLL, 0.0)


def _scoopful(k):
    """Scoop k: over the pan, dip, lift out level, carry to the mold and tip it in over the rim's center."""
    surface = PAN_SURF - (k - 1) * SCOOP_ML / 1e6 / (math.pi * PAN_R ** 2)
    out = [_w(1.2, tcp_for_mouth((*DIP_XY, CARRY_Z)), FINGER_CLOSED, f"scoop {k}: over the pan")]
    out += dk.dip(TCP0, HANDLE0 + MOUTH_OFF, (*DIP_XY, surface), DIP_DEPTH, drag=(0.0, 0.006), above=0.04,
                  roll=ROLL, pitch=P0, yaw=YAW, finger=FINGER_CLOSED, label=f"scoop {k}: dip")
    out.append(_w(1.0, tcp_for_mouth((*DIP_XY, CARRY_Z)), FINGER_CLOSED, f"scoop {k}: lift out level"))
    lip0 = HANDLE0 + MOUTH_OFF + [-BOWL_R, 0.0, 0.0]  # tipping toward -x: the -x side of the rim is the lip
    # start the tip nearly level (2.3 deg, inside dosekit's 3 deg slack): concrete's 38 deg repose sheds the heap
    # at the 26 deg start the formula pilot used
    out += pk.pour(TCP0, lip0, (*MOLD, POUR_LIP_Z), roll=ROLL, pitch=P0, yaw=YAW, start_pitch=P0 - 0.04,
                   to_pitch=P0 - POUR_TURN, above=0.04, steps=8, duration=2.5, hold=1.2, back=1.8,
                   finger=FINGER_CLOSED, label=f"scoop {k}: into the mold")
    return out


ROD_AIM = (0.009, -0.008)  # m: in the sim the held rod's tip bottoms out about 9 mm toward -x and 8 mm toward +y of
# where the hand is sent (arm lag and the rod's give in the grip), so each stroke's bottom is aimed that much the other
# way
SETTLE = 0.015  # m: each downstroke slows for its last 1.5 cm (a full-speed stop ran the arm 8 mm or more past it)


def _rod_strokes(points, tip_lo, tip_hi, n, label, clear=False, shift=(0.0, 0.0)):
    """`n` rod strokes over `points` (x, y offsets from the mold's axis): down to `tip_lo` (slowing for the last
    1.5 cm), back up to `tip_hi` (tip heights, world). With `clear`, each stroke comes straight back up and the rod
    moves over to its next point only up there (the tip out of the mold, so it never drags across the rim); the rod
    then comes down over the point itself and only the stroke's bottom is aimed ROD_AIM off (aimed off up there, the
    tip of a stroke 13 mm off the axis came down on the rim)."""
    out, prev = [], None
    for k in range(n):
        dx, dy, *extra = points[k % len(points)]
        dx, dy = dx + shift[0], dy + shift[1]
        ex, ey = extra or (0.0, 0.0)  # a point's own extra aim at the stroke's bottom only
        x, y = MOLD[0] + dx + ROD_AIM[0] + ex, MOLD[1] + dy + ROD_AIM[1] + ey
        x0, y0 = (MOLD[0] + dx, MOLD[1] + dy) if clear else (x, y)
        if clear:
            if prev is not None:
                out.append(_rodw(0.35, (*prev, tip_hi + ROD_DZ), FINGER_CLOSED, f"{label} {k + 1}: up"))
            out.append(_rodw(0.30, (x0, y0, tip_hi + ROD_DZ), FINGER_CLOSED, f"{label} {k + 1}: over"))
        else:
            out.append(_rodw(0.30, (x, y, tip_hi + ROD_DZ), FINGER_CLOSED, f"{label} {k + 1}: up"))
        out += [_rodw(0.30, (x, y, tip_lo + SETTLE + ROD_DZ), FINGER_CLOSED, f"{label} {k + 1}: down"),
                _rodw(0.15, (x, y, tip_lo + ROD_DZ), FINGER_CLOSED, f"{label} {k + 1}: bottom")]
        prev = (x0, y0)
    if clear:
        out.append(_rodw(0.30, (*prev, tip_hi + ROD_DZ), FINGER_CLOSED, f"{label} {n}: up"))
    out.append(_rodw(0.4, (MOLD[0], MOLD[1], tip_hi + ROD_DZ), FINGER_CLOSED, f"{label}: out"))
    return out


def _dur(ws):
    return sum(w.duration for w in ws)


TAP_Z = TOP + 0.045  # tool point height for the taps: the fingers' side meets the mold at mid height
TAP_R = MOLD_R_OUT + 0.0107  # the tool point's distance from the mold's axis where a closed hand pressed on it stopped
# in the sim (10.7 mm off the wall)
TAP_OUT = np.array([MOLD[0], MOLD[1] - TAP_R - 0.020, TAP_Z])  # 2 cm off the mold's -y side
TAP_IN = np.array([MOLD[0], MOLD[1] - TAP_R + 0.005, TAP_Z])  # aimed 5 mm into the wall: the fixed mold stops it
N_TAPS = 12


def _grip_scoop(r_out):
    """Right arm: from behind the scoop's handle, around it, grip, lift it off the stand, over the pan."""
    return [_w(1.5, r_out + [0, 0, 0.03], FINGER_OPEN, "behind the handle"),
            _w(1.0, r_out, FINGER_OPEN, "level with the handle"),
            _w(1.2, TCP0, FINGER_OPEN, "fingers around the handle"),
            _w(0.8, TCP0, FINGER_CLOSED, "grip"),
            _w(1.0, TCP0 + [0, 0, 0.04], FINGER_CLOSED, "lift off the stand"),
            _w(1.2, tcp_for_mouth((*DIP_XY, CARRY_Z)), FINGER_CLOSED, "over the pan")]


def _scoop_down(r_out):
    """Right arm: lay the scoop back on its stand, let go, back out along the handle and up."""
    return [_w(2.0, TCP0 + [0, 0, 0.05], FINGER_CLOSED, "over the stand"),
            _w(1.2, TCP0 + [0, 0, 0.004], FINGER_CLOSED, "lay it on the stand"),
            _w(0.6, TCP0 + [0, 0, 0.004], FINGER_OPEN, "let go"),
            _w(1.0, r_out + [0, 0, 0.004], FINGER_OPEN, "back out along the handle"),
            _w(0.8, r_out + [0, 0, 0.03], FINGER_OPEN, "up")]


def _mold_taps(k):
    """Right arm, closed hand straight down: N_TAPS light taps of the fingers' side on the mold's -y side at mid
    height (C31 allows an open hand on a plastic mold), then up and away."""
    out = [_down(1.5, TAP_OUT + [0, 0, 0.06], FINGER_CLOSED, f"taps {k}: over the mold's side"),
           _down(0.8, TAP_OUT, FINGER_CLOSED, f"taps {k}: down beside it")]
    for j in range(N_TAPS):
        out += [_down(0.40, TAP_IN, FINGER_CLOSED, f"taps {k}: tap {j + 1}"),  # 0.25 s stopped 2 to 4 mm short
                _down(0.10, TAP_IN, FINGER_CLOSED, f"taps {k}: on the wall {j + 1}"),
                _down(0.25, TAP_OUT, FINGER_CLOSED, f"taps {k}: back {j + 1}")]
    out.append(_down(0.8, TAP_OUT + [0, 0, 0.06], FINGER_CLOSED, f"taps {k}: up"))
    return out


def demo():
    """Right arm: grip the scoop, two scoops of concrete into the mold, lay the scoop back on its stand. Left arm: lift
    the rod out of its rack, rod the first layer 25 times to just above the floor, spread over the center and four
    points 13 mm off it, and hang it in the rack. Right arm: tap the mold's side 12 times with the closed hand, grip
    the scoop again, two more scoops (the second layer, overfilling slightly), lay it down. Left arm: take the rod
    again, rod the second layer 25 times over the center and four points 12 mm off it, aimed 1.1 cm into the first
    (the strokes reach about 1.7 cm in the sim). Right arm: 12 more taps, home. Left arm: hang the rod
    back in its rack, pick up the float, draw its edge across the rim toward -y (the excess falls off on that side),
    lay it down, home."""
    r_out = TCP0 + [0.0, -0.05, 0.0]  # the fingertips 1 cm clear of the handle's end
    rack_up = ROD_TCP + [0.0, 0.0, 0.015]  # the block 1.5 cm off the fork
    rack_out = rack_up + [-0.05, 0.0, 0.0]  # the rod slid out of the slot toward the robot
    rod_wait = rack_out + [0.0, 0.0, 0.06]  # holding the rod up beside the rack
    over_mold = (MOLD[0], MOLD[1], L2_TIP_HI + ROD_DZ + 0.003)

    def wait_r(ws, until, label):
        last = ws[-1]
        ws.append(kit.Tilted(max(until - _dur(ws), 0.3), last.pos, last.finger, label, last.yaw, last.roll,
                             last.pitch))

    right = _grip_scoop(r_out) + _scoopful(1) + _scoopful(2)
    layer1_poured = _dur(right)
    right += _scoop_down(r_out)

    def take_rod():
        return [_rodw(1.5, rack_up + [0, 0, 0.04], FINGER_OPEN, "over the rod's block"),
                _rodw(1.0, ROD_TCP, FINGER_OPEN, "around the block"),
                _rodw(0.6, ROD_TCP, FINGER_CLOSED, "grip rod"),
                _rodw(0.8, rack_up, FINGER_CLOSED, "lift the block off the fork"),
                _rodw(1.2, rack_out, FINGER_CLOSED, "slide the rod out of the slot"),
                _rodw(1.0, rod_wait, FINGER_CLOSED, "raise the rod")]

    def hang_rod():
        return [_rodw(1.0, rack_out, FINGER_CLOSED, "lower beside the slot"),
                _rodw(1.2, rack_up, FINGER_CLOSED, "slide the rod into the slot"),
                _rodw(0.8, ROD_TCP + [0, 0, -0.002], FINGER_CLOSED, "set the block on the fork"),
                _rodw(0.6, ROD_TCP + [0, 0, -0.002], FINGER_OPEN, "let go of rod"),
                _rodw(1.0, ROD_TCP + [0, 0, 0.05], FINGER_OPEN, "up off the rod")]

    left = take_rod()
    wait_r(left, layer1_poured, "wait for the first layer")
    left += [_rodw(2.0, over_mold, FINGER_CLOSED, "rod over the mold")]
    left += _rod_strokes(FIRST_PTS, L1_TIP_LO, L1_TIP_HI, 25, "layer 1 stroke")
    left += [_rodw(0.6, over_mold, FINGER_CLOSED, "rod up out of the mold"),
             _rodw(2.0, rod_wait, FINGER_CLOSED, "back beside the rack")]
    layer1_rodded = _dur(left)
    left += hang_rod() + take_rod()  # set it in the rack while the second layer goes in, then take it again

    wait_r(right, layer1_rodded - 1.5, "wait for the rodding")
    right += _mold_taps(1)
    right += _grip_scoop(r_out)[0:1] + _grip_scoop(r_out)[1:]
    right += _scoopful(3) + _scoopful(4)
    layer2_poured = _dur(right)
    right += _scoop_down(r_out)

    wait_r(left, layer2_poured, "wait for the second layer")
    left += [_rodw(2.0, over_mold, FINGER_CLOSED, "rod over the mold")]
    l2_lo = TOP + MOLD_FLOOR + L1_TOP_EST - L2_INTO
    left += _rod_strokes(L2_PTS, l2_lo, L2_TIP_HI, 25, "layer 2 stroke", clear=True, shift=L2_SHIFT)
    left += [_rodw(0.6, over_mold, FINGER_CLOSED, "rod up out of the mold"),
             _rodw(2.0, rod_wait, FINGER_CLOSED, "back to the rack")]
    layer2_rodded = _dur(left)
    left += hang_rod()

    wait_r(right, layer2_rodded - 1.5, "wait for the rodding")
    right += _mold_taps(2) + [kit.Home(2.0)]
    right_home = _dur(right)

    strike_z = RIM + 0.001 + FLOAT_DZ  # the blade's underside 1 mm over the rim where it starts
    press = 0.0025  # and sinking 2.5 mm as it goes, so it rides on the rim (held level it rode up to 2.5 mm high;
    # started lower, its edge caught the rim's outside)
    y0, y1 = MOLD[1] + 0.050, MOLD[1] - 0.065  # the blade starts clear of the rim and of the rack's post
    left += [_left(1.8, FLOAT_TCP + [0, 0, 0.05], FINGER_OPEN, "over the float's knob"),
             _left(1.0, FLOAT_TCP, FINGER_OPEN, "around the knob"),
             _left(0.6, FLOAT_TCP, FINGER_CLOSED, "grip float"),
             _left(1.0, FLOAT_TCP + [0, 0, 0.12], FINGER_CLOSED, "lift the float")]
    wait_r(left, right_home, "wait for the right arm to clear")
    left += [_left(1.8, (MOLD[0], y0, strike_z + 0.03), FINGER_CLOSED, "float over the rim's far side"),
             _left(0.8, (MOLD[0], y0, strike_z), FINGER_CLOSED, "down to the rim"),
             _left(3.0, (MOLD[0], y1, strike_z - press), FINGER_CLOSED, "strike off toward -y"),
             _left(0.8, (MOLD[0], y1, strike_z + 0.04), FINGER_CLOSED, "lift the float"),
             _left(2.0, FLOAT_TCP + [0, 0, 0.11], FINGER_CLOSED, "back over its spot"),  # blade over the rack
             _left(1.5, FLOAT_TCP + [0, 0, 0.002], FINGER_CLOSED, "lay it down"),
             _left(0.6, FLOAT_TCP + [0, 0, 0.002], FINGER_OPEN, "let go of float"),
             _left(0.8, FLOAT_TCP + [0, 0, 0.05], FINGER_OPEN, "up off the float"),
             kit.Home(2.0), kit.Home(2.0, label="hold at home")]
    right.append(kit.Home(max(_dur(left) - _dur(right), 0.5), label="stay home"))
    return {"left": left, "right": right}
