"""A prep counter with a dial stem thermometer and a glass of ice water. The thermometer (a 2.7 cm dial on a steel stem,
10.8 cm from the dial's middle to the pointed tip, with the usual dimple 5 cm up from the tip marking the top of its
sensing area) stands upright in its holder: a clear sleeve on a weighted black base. Farther out on the robot's right
is a clear glass tumbler packed with crushed ice and topped up with water to 7.8 cm.

The ice-point method, as food safety courses teach it: put the stem in ice water with the whole sensing area (tip to
dimple) under the water, keep it off the glass's bottom and sides, and hold it there steady for 30 seconds, until the
needle stops moving, before reading it and turning the calibration nut to 32 F. Then it goes back in its holder. The
instruction only says to calibrate.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props below; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Calibrate the probe thermometer."

# The thermometer. Its origin is the middle of the dial; the stem hangs below it.
DIAL_R, DIAL_H = 0.0135, 0.022
NUT_H, NUT_R = 0.008, 0.0065
STEM_R = 0.0025
TIP_BELOW = DIAL_H / 2 + 0.097  # from the origin down to the tip
DIMPLE_ABOVE_TIP = 0.050  # top of the sensing area (2 in)

HOLDER_XY = (0.25, -0.09)
HOLDER_BASE_H, SLEEVE_TOP, BORE = 0.010, 0.050, 0.016
THERMO_START = (HOLDER_XY[0], HOLDER_XY[1], TOP + HOLDER_BASE_H + TIP_BELOW)

CUP_XY = (0.28, -0.22)
CUP_R, CUP_T, CUP_H, CUP_FLOOR = 0.040, 0.003, 0.088, 0.006
CUP_IN = CUP_R - CUP_T
SURFACE = 0.078  # water level above the table

BOTTOM_GAP = 0.005  # the tip at least 5 mm above the glass's floor
WALL_GAP = 0.006  # the stem at least 6 mm from the glass's side
STILL = 0.03  # m/s: held steady, measured from how far it moves between control steps (20 Hz)
STEP_S = 0.05
MAX_TILT = math.radians(10)
FULL_STEPS = 600  # 30 s at 20 Hz
BACK_XY, BACK_Z, BACK_TILT = 0.008, 0.006, math.radians(12)  # back in the sleeve: tip in the bore, resting on its floor
BACK_STILL = 0.0005  # m per control step
SHORT_STEPS = 200  # 10 s

# -- looks ------------------------------------------------------------------------------------

STEEL = kit.Look((0.82, 0.83, 0.85), 0.22, metallic=1.0)
FACE = kit.Look((0.97, 0.97, 0.95), 0.6)
INK = kit.Look((0.10, 0.10, 0.12), 0.6)
RED = kit.Look((0.85, 0.08, 0.06), 0.4)
LENS = kit.Look((0.90, 0.95, 1.00), 0.02, opacity=0.18)
DIMPLE = kit.Look((0.35, 0.36, 0.38), 0.5, metallic=1.0)
GLASS = kit.Look((0.86, 0.92, 0.95), 0.05, opacity=0.22)
WATER = kit.Look((0.78, 0.89, 1.00), 0.05, opacity=0.30)
ICE = kit.Look((0.93, 0.97, 1.00), 0.15, opacity=0.70)
SLEEVE = kit.Look((0.88, 0.92, 0.96), 0.1, opacity=0.40)
BASE = kit.Look((0.08, 0.08, 0.09), 0.6)


def square_hole(center_z, hole, outer, thick):
    """Four box colliders round a square opening `hole` wide, in a plate `outer` wide."""
    s = (outer - hole) / 2
    o = hole / 2 + s / 2
    return [kit.Collider((o, 0, center_z), size=(s, outer, thick)), kit.Collider((-o, 0, center_z), size=(s, outer, thick)),
            kit.Collider((0, o, center_z), size=(hole, s, thick)), kit.Collider((0, -o, center_z), size=(hole, s, thick))]


def glass_vessel(r, h, floor, surface, glass, liquid, wall=0.0025, n=48):
    """An open glass vessel holding liquid, drawn so the renderer shows both: the glass is drawn only above the liquid
    line (an open band, so no cap closes the mouth), the liquid's own surface forms the vessel's sides below that line,
    and a thin glass floor sits under it. See-through layers drawn over each other hide one another."""
    ring = [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), (surface + h) / 2) for k in range(n + 1)]
    return [kit.band(ring, h - surface, wall, glass, normal=(0, 0, 1)),
            kit.cylinder((0, 0, floor / 2), r, floor, glass, n=n),
            kit.cylinder((0, 0, (floor + surface) / 2), r, surface - floor, liquid, n=n)]


def round_walls(r_in, height, z0, wall):
    """Four walls on the main directions and four square posts on the diagonals, each touching a circle of radius r_in
    from outside (colliders can't be turned)."""
    zc, side, a = z0 + height / 2, 2 * r_in * math.tan(math.pi / 8) + 0.004, wall / 2
    out = [kit.Collider((sx * (r_in + a), sy * (r_in + a), zc), size=(wall, side, height) if sx else (side, wall, height))
           for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1))]
    d = (r_in + a * math.sqrt(2)) / math.sqrt(2)
    out += [kit.Collider((sx * d, sy * d, zc), size=(wall, wall, height)) for sx in (1, -1) for sy in (1, -1)]
    return out


def _thermometer():
    """Dial stem thermometer: steel case with a white face, black ticks, a red pointer and a clear lens; a hex
    calibration nut under the dial; a 5 mm steel stem with a pointed tip and a dimple 5 cm up from it."""
    top = DIAL_H / 2
    tip = -TIP_BELOW
    meshes = [kit.cylinder((0, 0, 0), DIAL_R, DIAL_H, STEEL, n=48),
              kit.cylinder((0, 0, top + 0.0004), DIAL_R - 0.002, 0.0008, FACE, n=48),
              kit.cylinder((0, 0, top + 0.0030), DIAL_R - 0.0005, 0.0040, LENS, n=48),
              kit.cylinder((0, 0, top + 0.0012), 0.0010, 0.0010, INK)]
    for k in range(36):  # ticks round the scale, longer every fifth
        t = 2 * math.pi * k / 36 + math.pi / 2
        ln = 0.0026 if k % 5 == 0 else 0.0014
        rc = DIAL_R - 0.0025 - ln / 2
        size = (abs(ln * math.cos(t)) + 0.0006, abs(ln * math.sin(t)) + 0.0006, 0.0006)
        meshes.append(kit.box((rc * math.cos(t), rc * math.sin(t), top + 0.0010), size, INK))
    meshes.append(kit.box((0.0035, 0.0004, top + 0.0016), (0.008, 0.0009, 0.0006), RED))  # pointer near 32 F
    meshes.append(kit.cylinder((0, 0, -top - NUT_H / 2), NUT_R, NUT_H, STEEL, n=6))  # hex calibration nut
    stem_top = -top - NUT_H
    stem_len = stem_top - (tip + 0.006)
    meshes.append(kit.cylinder((0, 0, stem_top - stem_len / 2), STEM_R, stem_len, STEEL, n=20))
    meshes.append(kit.lathe([(0.0, tip), (STEM_R, tip + 0.006)], STEEL, n=20))
    meshes.append(kit.cylinder((0, 0, tip + DIMPLE_ABOVE_TIP), STEM_R + 0.0003, 0.0018, DIMPLE, n=20))
    colliders = (kit.Collider((0, 0, 0), radius=DIAL_R, height=DIAL_H),
                 kit.Collider((0, 0, (-top + tip) / 2), size=(2 * STEM_R, 2 * STEM_R, TIP_BELOW - top)))
    return kit.Prop("thermometer", pos=THERMO_START, size=(2 * DIAL_R, 2 * DIAL_R, TIP_BELOW + top), mass=0.04,
                    meshes=tuple(meshes), colliders=colliders)


def _holder():
    meshes = (kit.cylinder((0, 0, HOLDER_BASE_H / 2), 0.025, HOLDER_BASE_H, BASE, n=40),
              kit.lathe([(0.0, HOLDER_BASE_H), (0.0105, HOLDER_BASE_H), (0.0105, SLEEVE_TOP), (0.0085, SLEEVE_TOP),
                         (0.0085, HOLDER_BASE_H + 0.001), (0.0, HOLDER_BASE_H + 0.001)], SLEEVE, n=32))
    h = SLEEVE_TOP - HOLDER_BASE_H
    colliders = [kit.Collider((0, 0, HOLDER_BASE_H / 2), size=(0.05, 0.05, HOLDER_BASE_H))] + square_hole(
        HOLDER_BASE_H + h / 2, BORE, 0.021, h)
    return kit.Prop("holder", pos=(HOLDER_XY[0], HOLDER_XY[1], TOP), size=(0.05, 0.05, SLEEVE_TOP),
                    meshes=meshes, colliders=tuple(colliders))


def _cup():
    """A clear glass tumbler packed with crushed ice and water. The water and ice are only seen, not felt."""
    meshes = glass_vessel(CUP_R - CUP_T / 2, CUP_H, CUP_FLOOR, SURFACE, GLASS, WATER, wall=CUP_T, n=56)
    rng = np.random.default_rng(7)
    for layer, z in enumerate(np.arange(CUP_FLOOR + 0.008, SURFACE + 0.006, 0.011)):
        n = 9
        for k in range(n):  # a ring of chunks, leaving the middle open where the stem goes
            t = 2 * math.pi * (k + 0.5 * (layer % 2)) / n + rng.uniform(-0.15, 0.15)
            r = rng.uniform(0.022, CUP_IN - 0.008)
            s = rng.uniform(0.009, 0.014)
            meshes.append(kit.box((r * math.cos(t), r * math.sin(t), z + rng.uniform(-0.002, 0.002)),
                                  (s, s * rng.uniform(0.7, 1.0), s * rng.uniform(0.6, 0.9)), ICE))
    colliders = [kit.Collider((0, 0, CUP_FLOOR / 2), size=(2 * CUP_IN, 2 * CUP_IN, CUP_FLOOR))] + round_walls(
        CUP_IN, CUP_H - CUP_FLOOR, CUP_FLOOR, 0.004)
    return kit.Prop("ice_cup", pos=(CUP_XY[0], CUP_XY[1], TOP), size=(2 * CUP_R + 0.004, 2 * CUP_R + 0.004, CUP_H),
                    meshes=tuple(meshes), colliders=tuple(colliders))


OBJECTS = [_holder(), _cup(), _thermometer()]
_START = {o.name: np.array(o.pos) for o in OBJECTS if o.mass is not None}


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
GRIPPERS = ("left_gripper", "right_gripper")
class _History:
    def __init__(self):
        self.episode = kit.episode()  # which trial this history belongs to
        self.last = None
        self.last_tip = None
        self.clean = 0  # steps in a row with the sensing area under the water, clear of the glass, held steady
        self.under = 0  # steps in a row with the sensing area under the water, however it is held
        self.best_clean = 0
        self.best_under = 0
        self.back = False  # this step: back in the holder


_H = _History()


def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _tilt(s):
    x, y = s.quat_xyzw[0], s.quat_xyzw[1]
    return math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0)))


def _points(s):
    """World positions of the tip and the dimple."""
    p, r = np.asarray(s.pos, float), _rot(s.quat_xyzw)
    return p + r @ np.array([0, 0, -TIP_BELOW]), p + r @ np.array([0, 0, -TIP_BELOW + DIMPLE_ABOVE_TIP])


def _off_axis(pt):
    return float(np.hypot(pt[0] - CUP_XY[0], pt[1] - CUP_XY[1]))


def _under(s):
    """The whole sensing area under the water: the tip inside the glass and the dimple below the surface."""
    tip, dimple = _points(s)
    return bool(_off_axis(tip) < CUP_IN and dimple[2] < TOP + SURFACE and tip[2] < TOP + SURFACE)


def _clean(s, moved):
    """Under the water as above, and also: the tip at least 5 mm above the glass's floor, the stem (tip to dimple) at
    least 6 mm from its side, within 10 degrees of upright, and moving slower than 3 cm/s. `moved` is how far the tip
    went since the last control step. (The speed is measured from positions because the simulator's reported velocity
    of a held object chatters even when it is still.)"""
    tip, dimple = _points(s)
    clear_side = max(_off_axis(tip), _off_axis(dimple)) < CUP_IN - WALL_GAP - STEM_R
    return bool(_under(s) and tip[2] > TOP + CUP_FLOOR + BOTTOM_GAP and clear_side and _tilt(s) < MAX_TILT
                and moved < STILL * STEP_S)


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    tracked = list(_START) + [g for g in GRIPPERS if g in objects]
    now = {n: np.asarray(objects[n].pos, dtype=float) for n in tracked}
    _H.last = now
    t = objects["thermometer"]
    tip = _points(t)[0]
    moved = np.inf if _H.last_tip is None else float(np.linalg.norm(tip - _H.last_tip))
    _H.last_tip = tip
    _H.under = _H.under + 1 if _under(t) else 0
    _H.clean = _H.clean + 1 if _clean(t, moved) else 0
    _H.best_under = max(_H.best_under, _H.under)
    _H.best_clean = max(_H.best_clean, _H.clean)
    _H.back = _back(t, moved)


def _back(s, moved):
    """Put back: standing in the holder's sleeve (within 12 degrees of upright) with its tip inside the bore (within 8 mm
    of its axis) and down on the bore's floor (within 6 mm), and not moving (under 0.5 mm per control step)."""
    tip = _points(s)[0]
    floor = TOP + HOLDER_BASE_H
    return bool(np.hypot(tip[0] - HOLDER_XY[0], tip[1] - HOLDER_XY[1]) < BACK_XY and abs(tip[2] - floor) < BACK_Z
                and _tilt(s) < BACK_TILT and moved < BACK_STILL)


def _score():
    if _H.best_clean >= FULL_STEPS:
        return 1.0 if _H.back else 0.75
    if _H.best_clean >= SHORT_STEPS:
        return 0.5
    if _H.best_under >= SHORT_STEPS:
        return 0.25
    return 0.0


def success(objects):
    """1.0: the sensing area held in the ice water, clear of the glass, for 30 s in a row, then the thermometer back in
    its holder. Called first each step, so it also records the step's history."""
    _update(objects)
    return _score() == 1.0


def score(objects):
    """From the longest unbroken stretch of the run (at 20 Hz). 30 s or more with the sensing area under the water, the
    tip at least 5 mm off the bottom, the stem at least 6 mm from the side, within 10 degrees of upright and held steady
    (under 3 cm/s) scores 1.0 if the thermometer is back upright and at rest in its holder at the end, and 0.75 if it
    isn't. 0.5 for 10 to 30 s like that (read too soon). 0.25 if the sensing area was under
    the water for 10 s or more but never 10 s clear of the glass and steady (resting on the bottom or leaning on the
    side, as when it is dropped in). The 0.5 and 0.25 levels don't depend on where it ends. 0 otherwise, including a
    dip of only the tip. Possible scores: 0, 0.25, 0.5, 0.75, 1."""
    return _score()


def demo():
    """Right arm: lift the thermometer out of its holder by the dial, carry it over the glass, lower the stem into the
    middle of the ice water until the tip is 1.3 cm off the bottom, hold it still for 32 s, then lift it straight out
    and lower it back into its holder's sleeve."""
    travel = TOP + 0.214
    grip_dz = 0.004  # TCP above the dial's middle when gripping, so the finger pads close on the dial's case
    lag = 0.0  # measured: at rest the arm reaches these targets within 2 mm, so no offset
    swing = 0.003  # aim 3 mm past the sleeve's axis in x: the stem hangs a few degrees back in the grip
    sag = 0.003  # and 3 mm to the left: the arm falls short toward its own base
    hx, hy = HOLDER_XY
    cx, cy = CUP_XY
    pick_z = THERMO_START[2] + grip_dz
    hold_z = TOP + CUP_FLOOR + 0.013 + TIP_BELOW + grip_dz
    above_z = TOP + SLEEVE_TOP + 0.015 + TIP_BELOW + grip_dz  # the tip 1.5 cm over the sleeve
    w = [Waypoint(1.5, (hx, hy + lag, travel), FINGER_OPEN, "over the holder"),
         Waypoint(1.5, (hx, hy + lag, pick_z), FINGER_OPEN, "down to the dial"),
         Waypoint(0.8, (hx, hy + lag, pick_z), FINGER_CLOSED, "grip"),
         Waypoint(1.5, (hx, hy + lag, travel), FINGER_CLOSED, "lift out"),
         Waypoint(1.5, (cx, cy + lag, travel), FINGER_CLOSED, "over the glass"),
         Waypoint(1.0, (cx, cy + lag, travel), FINGER_CLOSED, "settle"),
         Waypoint(2.0, (cx, cy + lag, hold_z), FINGER_CLOSED, "into the ice water"),
         Waypoint(32.0, (cx, cy + lag, hold_z), FINGER_CLOSED, "hold 30 s"),
         Waypoint(2.0, (cx, cy + lag, travel), FINGER_CLOSED, "lift straight out"),
         Waypoint(1.5, (hx + swing, hy + lag, travel), FINGER_CLOSED, "over the holder"),
         Waypoint(1.0, (hx + swing, hy + sag, travel), FINGER_CLOSED, "settle"),
         Waypoint(1.5, (hx + swing, hy + sag, above_z), FINGER_CLOSED, "tip just over the sleeve"),
         Waypoint(1.5, (hx + swing, hy + sag, above_z), FINGER_CLOSED, "settle"),
         Waypoint(2.5, (hx + swing, hy + sag, pick_z + 0.003), FINGER_CLOSED, "into the sleeve"),
         Waypoint(0.6, (hx + swing, hy + sag, pick_z + 0.003), FINGER_OPEN, "let go"),
         Waypoint(1.0, (hx + swing, hy + sag, pick_z + 0.03), FINGER_OPEN, "back off"),
         Waypoint(2.0, (hx + swing, hy + sag, pick_z + 0.03), FINGER_OPEN, "hold")]
    return {"right": w}
