"""Measuring powdered infant formula into a bottle. A wide-neck bottle stands in a holder with 120 mL (4 fl oz) of water
in it, next to an opened tin of formula powder with a leveling bar across its opening and the tin's scoop lying on a
scoop stand.

US standard powdered formulas are mixed at one unpacked, level scoop of powder per 2 fl oz of water, so this bottle
takes exactly two level scoops (17.8 g). The scoop is dipped into the powder, which leaves it heaped, and then drawn
up under the tin's leveling bar so the heap is cut back into the tin. A heaped scoop holds about 29% more powder
than a level one, and a pressed-down one more still; too much powder (or too little) makes the feed wrong for the
baby, and powder in water cannot be taken back out.

The powder is dosekit's scored bulk model (dosekit.py, on pourkit and yamkit): heaps at the angle of repose, dips,
the strike-off under the fixed bar, pours from the tipped scoop into the bottle's neck, spills. Visuals only: the
powder standing above the scoop's rim is drawn orange, so heaped and level read apart at 320x176.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (modeled props; nothing on disk changes)
import pourkit as pk  # noqa: E402,F401  (dosekit builds on it)
import dosekit as dk  # noqa: E402  (import last: its hook runs after yamkit's and pourkit's)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402
import yam_arena.demo as _yd  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Measure the formula into this bottle."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# --- Scene, world coordinates (m): x forward from the arm-base midpoint, y to the robot's left, z up from the table.
TIN = (0.310, 0.110)  # the formula tin's axis
TIN_R_IN, TIN_R_OUT, TIN_H = 0.070, 0.073, 0.100  # a 14.6 cm steel tin, 10 cm tall, lid off
TIN_FLOOR = 0.002
POWDER_G = 700.0  # g of powder in the tin: its top about 6.5 mm below the rim
BAR_Y = 0.015  # the leveling bar spans the opening along x, 1.5 cm to the +y side of the tin's axis
BAR_W, BAR_T = 0.010, 0.004  # its width (y) and thickness; its underside, the cutting edge, is at the rim

STAND = (0.230, -0.125)  # the scoop stand; the scoop's handle center starts above it (closer in y and the right arm
# cannot hold the side grip this low)
HANDLE_Z = 0.0805  # handle axis height at the start (the handle rests on the stand's post)
HANDLE_R, HANDLE_L = 0.0125, 0.070  # round handle 2.5 cm across (a grip sleeve on the real scoop), along y
BOWL_Y, RIM_Z = 0.0775, -0.039  # the bowl's axis and rim, relative to the handle center (handle rises from the bowl)
BOWL_R, BOWL_D = 0.018, 0.0175  # bowl inside radius and depth: 17.8 mL, one level scoop = 8.9 g of powder
BOWL_R_OUT, BOWL_BOTTOM = 0.0195, -0.058
SCOOP_MASS = 0.045  # kg (weighted handle; a real scoop is ~5 g, too light to grip reliably)

BOTTLE = (0.400, -0.030)  # the bottle's axis (the hand pours from 9 cm to its -y: nearer the base the wrist runs out;
# at x 32 the hand, which hangs 5 cm under the tool point in this grip, landed on the bottle's neck during the dip)
BOTTLE_R, BOTTLE_H = 0.030, 0.115  # 6 cm across, 11.5 cm to the top of its neck
NECK_R_IN = 0.025  # wide neck: 5 cm opening
WATER_ML = 120.0  # 4 fl oz

FORMULA = dk.Material("formula", loose=0.497, dipped=0.50, tapped=0.62, packed=0.64, repose=42.0,
                      color=(0.97, 0.94, 0.80), flow=2.5)  # dk.MATERIALS["formula"]
SCOOP_BIN = dk.Bin("scoop", obj="scoop", at=(0.0, BOWL_Y, RIM_Z), sections=((BOWL_R, BOWL_D),), floor=-BOWL_D,
                   jolt_dv=None, heap_color=(1.0, 0.45, 0.05))
TIN_BIN = dk.Bin("tin", sections=((TIN_R_IN, TIN_H - TIN_FLOOR),), floor=TIN_FLOOR, fill={"formula": POWDER_G},
                 scoop=False, jolt_dv=None)
BOTTLE_BIN = dk.Bin("bottle", sections=((BOTTLE_R - 0.0015, 0.093), (NECK_R_IN, 0.020)), floor=0.002,
                    scoop=False, dip_into=False, jolt_dv=None)  # nothing comes back out of a bottle of water
LEVELER = dk.Edge("tin", a=(-0.068, BAR_Y, TIN_H), b=(0.068, BAR_Y, TIN_H), below=0.002)  # the bar has a
# collider: a scoop is drawn up against its underside (2 mm contact slop), and one carried over its top cuts nothing
dk.setup(materials=[FORMULA], bins=[TIN_BIN, SCOOP_BIN, BOTTLE_BIN],
         tools=[LEVELER, dk.Presser("left_gripper", depth=0.02), dk.Presser("right_gripper", depth=0.02)],
         floor_z=TOP)  # a fingertip pressing the powder packs the top 2 cm

# --- Scoring thresholds (reasons in score() and the README).
LEVEL_G = SCOOP_BIN.brim * FORMULA.rho_dip  # one level scoop: 17.8 mL x 0.50 g/mL = 8.9 g (labels: 8.6 to 8.8 g)
DOSE_G = 2 * LEVEL_G  # two level scoops for 4 fl oz of water: 17.8 g
TOL = 0.07  # +-7%: a strike-off 1 mm high or low changes a scoop by 0.5 g (5.7%)
SPILL_MAX = 1.0  # g of powder on the counter
SWEPT_MIN = 0.99  # share of the scoop's mouth the bar must have passed over (at its underside) since the last dip
HANDS_OFF = 0.040  # m: a tool point within 4 cm of the scoop's body (handle, neck or bowl) is still on it
# the scoop's body as a line, scoop frame: the handle's two ends, the neck's foot, the bowl's center
SCOOP_LINE = np.array([(0.0, -HANDLE_L / 2, 0.0), (0.0, HANDLE_L / 2, 0.0), (0.0, BOWL_Y - BOWL_R_OUT, RIM_Z),
                       (0.0, BOWL_Y, RIM_Z - BOWL_D / 2)])
DT = 0.05
HOME_TOL = 0.03
HOME_STILL = 0.002
GRIPPERS = ("left_gripper", "right_gripper")

STEEL = kit.Look((0.90, 0.90, 0.88), 0.35, metallic=0.5)
BAND = kit.Look((0.55, 0.72, 0.88), 0.6)
GRAY = kit.Look((0.50, 0.52, 0.55), 0.4, metallic=0.4)
BLUE = kit.Look((0.08, 0.18, 0.62), 0.4)
WHITE = kit.Look((0.93, 0.93, 0.92), 0.6)
CLEAR = kit.Look((0.92, 0.95, 0.98), 0.1, opacity=0.30)
WATER = kit.Look((0.62, 0.80, 0.96), 0.1)
RING = kit.Look((0.35, 0.35, 0.38), 0.5)
MARK = kit.Look((0.10, 0.10, 0.12), 0.5)


def _tin():
    """A fixed steel tin, lid off, with a gray leveling bar across its opening (with a collider: a scoop is drawn up
    against its underside, where the kit's Edge does the cutting)."""
    m = [kit.lathe([(0.0, 0.0), (TIN_R_OUT, 0.0), (TIN_R_OUT, TIN_H), (TIN_R_IN, TIN_H), (TIN_R_IN, TIN_FLOOR),
                    (0.0, TIN_FLOOR)], STEEL),
         kit.cylinder((0, 0, 0.045), TIN_R_OUT + 0.0005, 0.05, BAND),  # a plain paper label band
         kit.box((0.0, BAR_Y, TIN_H + BAR_T / 2), (2 * math.sqrt(TIN_R_IN ** 2 - BAR_Y ** 2), BAR_W, BAR_T), GRAY)]
    bar = ((0.0, BAR_Y, TIN_H + BAR_T / 2), (2 * math.sqrt(TIN_R_IN ** 2 - BAR_Y ** 2), BAR_W, BAR_T))
    c = [kit.Collider((0, 0, TIN_FLOOR / 2), radius=TIN_R_OUT, height=TIN_FLOOR), kit.Collider(bar[0], size=bar[1])]
    n, s = 48, 0.008  # the wall as a ring of small boxes (colliders are axis-aligned)
    for k in range(n):
        a = 2 * math.pi * k / n
        r = TIN_R_OUT - s / 2
        c.append(kit.Collider((r * math.cos(a), r * math.sin(a), TIN_H / 2), size=(s, s, TIN_H)))
    return kit.Prop("tin", pos=(*TIN, TOP), size=(2 * TIN_R_OUT, 2 * TIN_R_OUT, TIN_H + BAR_T), meshes=tuple(m),
                    colliders=tuple(c))


def _scoop():
    """The scoop: a round dark-blue bowl with its handle rising from the rim, origin at the handle's center."""
    neck_a, neck_b = np.array([0.0, HANDLE_L / 2 - 0.002, -0.004]), np.array([0.0, BOWL_Y - BOWL_R_OUT + 0.001, RIM_Z])
    neck = neck_b - neck_a
    m = (kit.cylinder((0, 0, 0), HANDLE_R, HANDLE_L, BLUE, axis="y"),
         kit.cylinder(tuple((neck_a + neck_b) / 2), 0.005, float(np.linalg.norm(neck)), BLUE, axis=neck),
         kit.lathe([(0.0, BOWL_BOTTOM), (BOWL_R_OUT, BOWL_BOTTOM), (BOWL_R_OUT, RIM_Z), (BOWL_R, RIM_Z),
                    (BOWL_R, RIM_Z - BOWL_D), (0.0, RIM_Z - BOWL_D)], BLUE, center=(0.0, BOWL_Y, 0.0)))
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


def _bottle():
    """A fixed wide-neck bottle in a weighted holder ring, 120 mL of water in it, the 120 mL line marked."""
    water_h = WATER_ML / 1e6 / (math.pi * (BOTTLE_R - 0.0015) ** 2)
    m = (kit.cylinder((0, 0, 0.003), 0.045, 0.006, RING),
         kit.lathe([(0.0, 0.0), (BOTTLE_R, 0.0), (BOTTLE_R, 0.095), (NECK_R_IN + 0.002, 0.100),
                    (NECK_R_IN + 0.002, BOTTLE_H), (NECK_R_IN, BOTTLE_H), (NECK_R_IN, 0.100)], CLEAR),
         kit.cylinder((0, 0, 0.002 + water_h / 2), BOTTLE_R - 0.0025, water_h, WATER),
         kit.cylinder((0, 0, 0.002 + water_h), BOTTLE_R + 0.0003, 0.0012, MARK))
    c = (kit.Collider((0, 0, BOTTLE_H / 2), radius=BOTTLE_R, height=BOTTLE_H),
         kit.Collider((0, 0, 0.003), radius=0.045, height=0.006))
    return kit.Prop("bottle", pos=(*BOTTLE, TOP), size=(0.09, 0.09, BOTTLE_H), meshes=m, colliders=c)


OBJECTS = [_tin(), _scoop(), _stand(), _bottle()]


# --- What happened during the run. success() and score() are called after every control step (after the kit has
# moved the powder for that step); the history is started afresh on every scene reset (kit.episode()).
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.home = None  # each gripper's tool point at the trial's first scored step
        self.last = None
        self.still = False
        self.most_in_bottle = 0.0  # g: the most powder the bottle has held this trial (it can only grow)
        self.unleveled = False  # a scoopful went into the bottle without being drawn across the leveling bar


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    now = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    if _H.home is None and len(now) == 2:
        _H.home = now
    _H.still = _H.last is not None and all(g in _H.last and np.linalg.norm(now[g] - _H.last[g]) < HOME_STILL
                                           for g in now)
    _H.last = now
    g = dk.amount(objects, "bottle")
    if g > _H.most_in_bottle + 1e-6:  # powder arrived in the bottle this step: was this load leveled at the bar?
        loads = [d for d in dk.dips() if d.scoop == "scoop"]
        if not loads or dk.swept_since(objects, "scoop", loads[-1].end) < SWEPT_MIN:
            _H.unleveled = True
    _H.most_in_bottle = max(_H.most_in_bottle, g)


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still."""
    if _H.home is None or not _H.still:
        return False
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - _H.home[g]) < HOME_TOL for g in GRIPPERS)


def _dist_to_scoop(objects, point):
    """Distance from `point` to the scoop's body line (handle, neck, bowl), as the scoop now lies."""
    s = objects["scoop"]
    line = np.asarray(s.pos, float) + SCOOP_LINE @ pk._rotm(s.quat_xyzw).T
    p, best = np.asarray(point, float), np.inf
    for a, b in zip(line[:-1], line[1:]):
        t = float(np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0))
        best = min(best, float(np.linalg.norm(p - (a + t * (b - a)))))
    return best


def _held(objects):
    """Some tool point within 4 cm of the scoop's handle, neck or bowl: a hand still on the scoop, wherever it holds
    it (the tool point sits between the fingertips, within about 2 cm of what they hold)."""
    return any(g in objects and _dist_to_scoop(objects, objects[g].pos) < HANDS_OFF for g in GRIPPERS)


def _job(objects):
    if _H.unleveled:
        return 0.0  # a scoopful went in without being drawn across the leveling bar since it was dipped
    if _H.most_in_bottle > DOSE_G * (1 + TOL):
        return 0.0  # too much powder went in (heaped, packed or a third scoop): it cannot come back out
    if _held(objects):
        return 0.0  # nothing counts while the scoop is still in hand (by its handle, neck or bowl)
    s = objects["scoop"]
    if not s.resting(0.01, 0.5) or s.pos[2] < TOP:
        return 0.0  # the scoop still moving, or dropped off the counter
    if dk.spilled(objects) >= SPILL_MAX:
        return 0.0  # powder on the counter: missed the bottle or knocked off the scoop
    g = dk.amount(objects, "bottle")
    if DOSE_G * (1 - TOL) <= g <= DOSE_G * (1 + TOL):
        return 1.0  # two level scoops
    if LEVEL_G * (1 - TOL) <= g <= LEVEL_G * (1 + TOL):
        return 0.25  # one level scoop in: half the job, nothing over
    return 0.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """Two level scoops in the bottle, each drawn across the leveling bar, nothing spilled, the scoop set down, both arms home and still. Called first
    each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """Scored on the final state; the powder is weighed by the kit. A correct feed is two level scoops, 17.8 g (one
    level scoop holds 17.8 mL of powder at 0.50 g/mL, 8.9 g). 0 for the whole trial if any scoopful reached the bottle
    without the bar having passed over all of the scoop's mouth (at its underside) since that scoopful was dipped:
    the label's level scoop is leveled with the leveler, and the model's exact shedding of a heap by tilting is not
    something a real cohesive powder does. 0 for the whole trial once the bottle has held more than
    17.8 g + 7% = 19.1 g (one heaped scoop with one level is up to 20.3 g, though part of a heap can fall off on the carry, so the
    strike-off rule is what catches it; packed or third scoops more), since powder in
    water cannot be taken back out. Otherwise 0 while a tool point is within 4 cm of the scoop's handle, neck or bowl, while
    the scoop is still moving or off the counter, or with 1 g or more of powder spilled. Then: 1.0 for 16.6 to 19.1 g
    in the bottle (+-7%: a strike-off 1 mm high or low changes a scoop by 0.5 g) with both arms back home and still,
    0.9 for the same with the arms not home; 0.25 for 8.3 to 9.5 g (exactly one level scoop, nothing over); 0 for
    anything else, including doing nothing. Possible scores: 0, 0.25, 0.9, 1.0."""
    return _score(objects)


# --- The demo: the right arm only. Side grip on the round handle, the hand pointing +y along it (yaw 90 deg, roll
# 1.35 rad: pourkit's verified pouring grip), fingers closing across it; tipping the scoop is then a turn of the
# wrist (pitch about world y). The grip is taken at pitch +0.5 rad and the scoop is tipped toward -x, down to -1.75
# rad: the right arm's wrist reaches that range over the bottle (it runs out near +1.1 rad, so not the other way).
YAW, ROLL, P0 = math.pi / 2, 1.35, 0.5
POUR_TURN = 2.15  # rad (123 deg, tipping toward -x: the wrist turns from +0.5 to -1.65 rad; joint4 limits it): the body empties past 42 + 3 deg
GRIP_DY = -0.015  # the tool point sits 1.5 cm behind the handle's center (more of the handle in the fingers)
HANDLE0 = np.array([*STAND, TOP + HANDLE_Z])
TCP0 = HANDLE0 + [0.0, GRIP_DY, 0.0]
MOUTH_OFF = np.array([0.0, BOWL_Y, RIM_Z])  # the bowl's rim center from the handle center, scoop level
RIM = TOP + TIN_H
PRESS = 0.009  # aim the rim 9 mm into the bar: the loaded arm tracks about 3.5 mm low and the scoop pitches about
# 2 deg nose-down in the grip (its bowl 3 mm low), so this presses the rim up to the bar's underside (its collider
# stops it there)
DROP = 0.004  # backing out, aim the rim 4 mm over the rim height: it runs a few mm under the bar, clear of it and
# above the powder (6.5 mm down; the scoop's mouth 4 mm under it would fill it again and count as a new dip)


_UP = -(kit._rot_xyz(ROLL, P0) @ _yd.grip_rotation(YAW))[:, 1]  # back of the
# hand: the direction across the handle that the open fingers straddle it along


def tcp_for_mouth(mouth):
    """Tool point that puts the level scoop's rim center at `mouth`."""
    return tuple(np.asarray(mouth, float) - MOUTH_OFF + [0.0, GRIP_DY, 0.0])


def _w(dur, pos, finger, label, pitch=P0):
    return kit.Tilted(dur, tuple(float(v) for v in pos), finger, label, YAW, ROLL, pitch)


def _cycle(k, surface):
    """One scoop: over the tin, dip, draw up under the leveling bar, carry to the bottle, tip it in."""
    tx, ty = TIN
    dip_xy = (tx, ty - 0.025)  # dip on the near side of the bar
    out = [_w(1.2, tcp_for_mouth((*dip_xy, TOP + 0.15)), FINGER_CLOSED, f"scoop {k}: over the tin")]
    out += dk.dip(TCP0, HANDLE0 + MOUTH_OFF, (*dip_xy, surface), 0.010, drag=(0.0, 0.008), above=0.05, roll=ROLL,
                  pitch=P0, yaw=YAW, finger=FINGER_CLOSED, label=f"scoop {k}: dip")
    bar = ty + BAR_Y
    y_in = bar - 0.016  # the bowl's leading wall under the bar, the heap still short of the bar's line
    y_end = bar + 0.014  # every heap cell past the bar's line; the trailing wall still under the bar's -y edge
    y_out = bar - 0.028  # the whole bowl back clear of the bar on the dip side
    out += [_w(1.0, tcp_for_mouth((tx, ty - 0.017, RIM - 0.001)), FINGER_CLOSED, f"scoop {k}: rim just under the bar"),
            _w(1.0, tcp_for_mouth((tx, y_in, RIM - 0.001)), FINGER_CLOSED, f"scoop {k}: leading wall under the bar"),
            _w(0.6, tcp_for_mouth((tx, y_in, RIM + PRESS)), FINGER_CLOSED, f"scoop {k}: press up against it"),
            _w(2.5, tcp_for_mouth((tx, y_end, RIM + PRESS)), FINGER_CLOSED, f"scoop {k}: draw it along the bar"),
            _w(0.6, tcp_for_mouth((tx, y_end, RIM + DROP)), FINGER_CLOSED, f"scoop {k}: drop off the bar"),
            _w(2.0, tcp_for_mouth((tx, y_out, RIM + DROP)), FINGER_CLOSED, f"scoop {k}: back out under it"),
            _w(1.0, tcp_for_mouth((tx, y_out, TOP + 0.15)), FINGER_CLOSED, f"scoop {k}: lift out level")]
    lip0 = HANDLE0 + MOUTH_OFF + [-BOWL_R, 0.0, 0.0]  # tipping toward -x (the robot): the -x side of the rim is the lip
    out += pk.pour(TCP0, lip0, (*BOTTLE, TOP + BOTTLE_H + 0.035), roll=ROLL, pitch=P0, yaw=YAW,
                   start_pitch=P0 - 0.45, to_pitch=P0 - POUR_TURN, above=0.04, steps=8, duration=3.0, hold=1.5,
                   back=2.0, finger=FINGER_CLOSED, label=f"scoop {k}: into the bottle")
    return out


def demo():
    """Right arm: slide the fingers around the scoop's handle from behind, grip, lift it off the stand; twice: dip it
    into the powder, slide the heaped scoop's rim up under the tin's leveling bar, press it up against the bar and
    draw it along so the heap is cut back into the tin, drop clear and back out, carry it level to the bottle and tip
    it in over the neck; lay the scoop back on its stand, back out, go home. Left arm stays home."""
    surface = TOP + TIN_FLOOR + POWDER_G / FORMULA.loose / 1e6 / (math.pi * TIN_R_IN ** 2)
    back = TCP0 + 0.06 * _UP  # the open hand straddles the handle and comes down onto it across its axis
    right = [_w(1.5, back + [0, 0, 0.03], FINGER_OPEN, "above the handle"),
             _w(1.0, back, FINGER_OPEN, "fingers either side of the handle"),
             _w(1.2, TCP0, FINGER_OPEN, "down around the handle"),
             _w(0.8, TCP0, FINGER_CLOSED, "grip"),
             _w(1.0, TCP0 + [0, 0, 0.04], FINGER_CLOSED, "lift off the stand"),
             _w(1.2, TCP0 + [0, 0, TOP + 0.15 - (TCP0[2] + RIM_Z)], FINGER_CLOSED, "raise clear of the tin")]
    right += _cycle(1, surface)
    right += _cycle(2, surface - 2 * SCOOP_BIN.brim / 1e6 / (math.pi * TIN_R_IN ** 2) * 0.5)
    right += [_w(2.0, TCP0 + [0, 0, 0.05], FINGER_CLOSED, "over the stand"),
              _w(1.2, TCP0 + [0, 0, 0.004], FINGER_CLOSED, "lay it on the stand"),
              _w(0.6, TCP0 + [0, 0, 0.004], FINGER_OPEN, "let go"),
              _w(1.0, back + [0, 0, 0.004], FINGER_OPEN, "lift off the handle"),
              _w(0.8, back + [0, 0, 0.03], FINGER_OPEN, "up"),
              kit.Home(2.0), kit.Home(2.0, label="hold at home")]
    left = [kit.Home(sum(w.duration for w in right), label="stay home")]
    return {"left": left, "right": right}
