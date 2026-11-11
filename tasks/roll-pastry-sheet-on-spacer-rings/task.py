"""A block of puff pastry on a floured pastry board, and a wooden rolling pin with 3 mm spacer rings on both ends of its
barrel, lying in its rest behind the pastry.

The pastry is rolled into a sheet 3 mm thick and at least 20 cm long (to line the floor of a 20 cm square pan), with
both hands on the pin's handles so that both rings ride on the board: then the barrel runs 3 mm above the board all the
way across. A pin with one end lifted leaves a wedge (thick on that side), and the dough springs back, so it takes
several passes to come down to the rings. Bakers roll from the middle out to each end and stop short of the edge
(Food Republic, "The Part Of Puff Pastry Dough You Should Never Roll Out": put the pin over the center and roll
outward; America's Test Kitchen: "roll dough outward from center to edge"): a pin rolled off the end flattens the cut
edge and seals puff pastry's layers there, so that edge does not rise.

Model (doughkit): the pastry has no collider; the pin rides on its rings. Each slice of pastry that the barrel rolls
across with its lowest line below the slice's top is pressed to gap + half its excess over the gap (the gap is the
barrel's height above the board where it crosses), the slice lengthens and the pastry beyond it moves out, volume
kept. A pin rolled more than 5 mm past an end while pressing seals that end's last 1 cm, pressed to 0.6 x the gap.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props; nothing on disk changes)
import guardkit as gk  # noqa: E402  (grip, release and arms-home checks)
import doughkit as dk  # noqa: E402  (the rolling model, updated every control step)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Roll this puff pastry out into a 3 mm sheet at least 20 cm long."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
BOARD = (0.295, 0.0)  # the pastry board's center (x, y)
BOARD_SIZE = (0.31, 0.28, 0.015)  # 31 cm deep, 28 cm across, 1.5 cm thick (maple)
BOARD_FRICTION = (0.15, 0.10)  # static, dynamic: a floured board under a wooden pin
BZ = TOP + BOARD_SIZE[2]  # the board's top

PASTRY = (0.295, 0.0)  # the block's center
PASTRY_SIZE = (0.065, 0.20, 0.010)  # 6.5 cm long (x), 20 cm wide (y), 10 mm thick: 130 mL

R_BARREL = 0.025  # the pin's barrel: 5 cm across ...
BARREL_HALF = 0.13  # ... 26 cm long
RING_GAP = 0.003  # the spacer rings stand 3 mm proud of the barrel ...
R_RING = R_BARREL + RING_GAP
RING_IN, RING_OUT = 0.115, 0.13  # ... and sit on the barrel's last 1.5 cm at each end
R_HANDLE = 0.0125  # handles 2.5 cm across ...
HANDLE_IN, HANDLE_OUT = 0.13, 0.215  # ... 8.5 cm long; 43 cm overall
PIN_MASS = 0.45  # kg
REST_X = 0.415  # the pin rest: two wooden cradles on the table, one under each handle's outer end
REST_Y = (0.195, 0.215)  # |y| span of each cradle
REST_AXIS = TOP + 0.050  # the pin's axis in its rest: the rings 7 mm above the board
LIP = (0.005, 0.010)  # each cradle's front and back lips: 5 mm thick, 1 cm above the floor that carries the handle
REST_PLAY = 0.005  # m: room between the handle and each lip (a set-down pin drops in)
PIN_START = (REST_X, 0.0, REST_AXIS)  # in its rest, behind the pastry (a round pin left on the board rolls away)

# --- The rule's numbers (reasons in score() and the README).
TARGET_T = 0.003  # m: the sheet's thickness (the rings' gap)
TOL_T = 0.0005  # m: a cell counts as on spec within 0.5 mm of 3 mm
SPEC_SHARE = 0.90  # share of the sheet's cells that must be on spec (the 1 cm trimmed off each end is ~10 %)
MIN_LEN = 0.20  # m: every strip of the sheet at least this long (the pan's floor)
PART_LEN = 0.18  # m: 90 % of that (about one more pass to go) earns the partial score
PIN_DOWN = 0.005  # m: rings within 5 mm of the board, or axis within 5 mm of its rest seat, level within LEVEL_DEG: set down
LEVEL_DEG = 3.0
CLEAR = 0.01  # m: both hands this far clear of the pin is let go

GRIPPERS = ("left_gripper", "right_gripper")
GRIP_Y = 0.165  # where the hands take the handles (|y|), inboard of the rest

WOOD = kit.Look((0.80, 0.62, 0.40), 0.6)
DARKWOOD = kit.Look((0.55, 0.36, 0.20), 0.5)
BOARDWOOD = kit.Look((0.66, 0.48, 0.30), 0.8)  # oiled maple, darker than the table so the pastry reads on it
RINGLOOK = kit.Look((0.20, 0.45, 0.80), 0.4)  # silicone spacer rings


def _board():
    top = ((0.0, 0.0, BOARD_SIZE[2] / 2), BOARD_SIZE)
    return kit.Prop("board", pos=(*BOARD, TOP), size=BOARD_SIZE, meshes=(kit.box(*top, BOARDWOOD),),
                    colliders=(kit.Collider(*top),), friction=BOARD_FRICTION)


def _pin():
    m = [kit.cylinder((0, 0, 0), R_BARREL, 2 * BARREL_HALF, WOOD, axis="y")]
    c = [kit.Collider((0, 0, 0), radius=R_BARREL, height=2 * BARREL_HALF, axis="Y")]
    for s in (-1, 1):
        ring = (0.0, s * (RING_IN + RING_OUT) / 2, 0.0)
        handle = (0.0, s * (HANDLE_IN + HANDLE_OUT) / 2, 0.0)
        m += [kit.cylinder(ring, R_RING, RING_OUT - RING_IN, RINGLOOK, axis="y"),
              kit.cylinder(handle, R_HANDLE, HANDLE_OUT - HANDLE_IN, DARKWOOD, axis="y")]
        c += [kit.Collider(ring, radius=R_RING, height=RING_OUT - RING_IN, axis="Y"),
              kit.Collider(handle, radius=R_HANDLE, height=HANDLE_OUT - HANDLE_IN, axis="Y")]
    return kit.Prop("pin", pos=PIN_START, size=(2 * R_RING, 2 * HANDLE_OUT, 2 * R_RING), mass=PIN_MASS,
                    meshes=tuple(m), colliders=tuple(c))


def _rest():
    floor = REST_AXIS - R_HANDLE - TOP
    m, c = [], []
    for s in (-1, 1):
        yc, wy = s * sum(REST_Y) / 2, REST_Y[1] - REST_Y[0]
        parts = [((0.0, yc, floor / 2), (0.045, wy, floor))]
        for k in (-1, 1):
            parts.append(((k * (R_HANDLE + REST_PLAY + LIP[0] / 2), yc, floor + LIP[1] / 2), (LIP[0], wy, LIP[1])))
        for center, size in parts:
            m.append(kit.box(center, size, DARKWOOD))
            c.append(kit.Collider(center, size=size))
    return kit.Prop("rest", pos=(REST_X, 0.0, TOP), size=(0.045, 2 * REST_Y[1], floor + LIP[1]), meshes=tuple(m),
                    colliders=tuple(c))


OBJECTS = [_board(), _rest(), _pin()]

# the pin's parts in its own frame, for the release check
PIN_PARTS = (gk.Seg((0, -BARREL_HALF, 0), (0, BARREL_HALF, 0), R_RING),
             gk.Seg((0, -HANDLE_OUT, 0), (0, HANDLE_OUT, 0), R_HANDLE))

# the handles (left at +y, right at -y) and the barrel with its rings, in the pin's own frame
HANDLES = (gk.Seg((0, HANDLE_IN, 0), (0, HANDLE_OUT, 0), R_HANDLE),
           gk.Seg((0, -HANDLE_IN, 0), (0, -HANDLE_OUT, 0), R_HANDLE))
BARREL = gk.Block((0.0, 0.0, 0.0), (2 * R_RING, 2 * BARREL_HALF, 2 * R_RING))
HANDLE_BAND = gk.band(2 * R_HANDLE)  # fingers closed on a 2.5 cm handle
TOUCH = 0.01  # m: a tool point or pad tip this close to the barrel or rings is a hand on them


def _gate(objects):
    """(both handles held, a hand on the barrel or rings). The barrel of a handled pin turns on its axle between the
    handles: rolled by both handles it turns over the pastry; a hand on the barrel stops it, and it skids and drags."""
    left = gk.grip_on(objects, "pin", HANDLES[0], HANDLE_BAND)
    right = gk.grip_on(objects, "pin", HANDLES[1], HANDLE_BAND)
    both = any(a != b for a in left for b in right)
    on_barrel = any(gk.dist(objects, "pin", p, BARREL) < TOUCH
                    for g in GRIPPERS if g in objects for p in gk.tool_points(objects, g))
    return both, on_barrel


dk.setup(dk.Sheet(center=PASTRY, size=PASTRY_SIZE, board_z=BZ, x_range=(BOARD[0] - BOARD_SIZE[0] / 2,
                                                                         BOARD[0] + BOARD_SIZE[0] / 2)),
         dk.Pin("pin", radius=R_BARREL, length=2 * BARREL_HALF), gate=_gate)

HOME = gk.ArmsHome()


def _rotm(q):
    return dk._rotm(q)


def pin_down(objects):
    """The pin set down, axis within 3 degrees of level: back in its rest (axis within 5 mm of the rest's seat height
    and within 1 cm of its x), or lying on its rings on the board (their bottoms within 5 mm of the board's top)."""
    o = objects["pin"]
    a = _rotm(o.quat_xyzw) @ np.array([0.0, 1.0, 0.0])
    if math.degrees(math.asin(min(1.0, abs(a[2])))) > LEVEL_DEG:
        return False
    z = float(o.pos[2])
    in_rest = abs(z - REST_AXIS) <= PIN_DOWN and abs(float(o.pos[0]) - REST_X) <= 0.01
    return in_rest or abs(z - R_RING - BZ) <= PIN_DOWN


def released(objects):
    return gk.released_clear(objects, "pin", PIN_PARTS, CLEAR)


def sheet(objects=None):
    """(on-spec share, shortest strip length m, thinnest cell m, sealed cells) of the pastry now."""
    t, sealed = dk.grid()
    cov = t > 0
    on = (np.abs(t[cov] - TARGET_T) <= TOL_T + 1e-9).mean()
    return float(on), float(dk.lengths().min()), float(t[cov].min()), int(sealed.sum())


def _job(objects):
    if not released(objects) or not pin_down(objects):
        return 0.0  # the pin still in a hand, or not set down (in its rest or on the board)
    if dk.torn():
        return 0.0  # pressed with a hand on the barrel: it skidded and dragged the pastry, for good
    on, length, thin, sealed = sheet(objects)
    if sealed:
        return 0.0  # rolled off an end: the cut edge is pressed thin and its layers sealed, for good
    if length >= MIN_LEN and on >= SPEC_SHARE:
        return 1.0
    if length >= PART_LEN:
        return 0.5  # most of the way: at least 18 cm long but short of the pan or uneven (about one pass to go)
    return 0.0  # barely started, or a strip left thick and short (a wedge from a tilted pin)


def _score(objects):
    HOME.feed(objects)
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not HOME.home(objects):
        return 0.9
    return job


def success(objects):
    return _score(objects) == 1.0


def score(objects):
    """The pastry is read on a 1 cm grid. Only rolling with both handles held counts (the barrel turns on its axle
    between them). 1.0 if every 1 cm strip of the sheet is at least 20 cm long (the floor of a 20 cm square pan), at
    least 90 % of its cells are 3 mm thick within 0.5 mm (the rings' gap; the remaining 10 % is about the 1 cm a baker
    trims off each end of a 21 cm sheet), no end has been rolled over, the pastry is not torn, the pin is set down
    level (within 3 degrees) in its rest or on its rings on the board (within 5 mm) with both hands 1 cm clear of it,
    and both arms are back home and still (0.9 if not home). 0.5 if, with the same pin, end and tear conditions, every
    strip is at least 18 cm long (90 % of the length, about one pass to go) but the sheet is short of 20 cm or fewer
    than 90 % of its cells are on spec. 0 if an end was rolled over (the pin more than 5 mm past it while pressing, two
    steps in a row: sealed, latched), the pastry was pressed with a hand on the barrel or rings (a tool point or pad
    tip within 1 cm, two steps in a row: the barrel skids and drags it; torn, latched), any strip is shorter than 18 cm
    (barely started, or a wedge left by a tilted pin), or the pin is still held or not set down. Possible scores: 0,
    0.5, 0.9, 1.0."""
    return _score(objects)


# --- Demo: both arms take the pin by its handles, lift it over the middle of the block, set it down on its rings
# and roll from the middle out to each end and back, stopping short of the end, five passes; then lift it back into
# its rest, let go and go home.
CENTER_X = PASTRY[0]
AXIS_Z = BZ + R_RING  # tool point on the handle's axis with the pin on its rings
PRESS = 0.004  # m: the hands are commanded this far below the axis, so the rings stay on the board
LIFT = 0.05
SPEED = 0.04  # m/s along a stroke
YAW = math.pi / 2  # fingers close along x, across the handle
# Stroke ends (x of the barrel's line), five passes, worked out with doughkit: 4 mm short of the end at that pass (the
# pin trails its hands by about 1 cm at the turn, so it stops about 1.4 cm short)
STROKES = (0.332, 0.295, 0.249, 0.295, 0.361, 0.295, 0.219, 0.295, 0.385, 0.295, 0.201, 0.295, 0.394, 0.295, 0.194, 0.295,
           0.398, 0.295, 0.192, 0.295)


def _arm(y):
    half = FINGER_OPEN
    out = [Waypoint(2.5, (REST_X, y, REST_AXIS + LIFT), half, "over the handle", yaw=YAW),
           Waypoint(1.2, (REST_X, y, REST_AXIS), half, "around the handle", yaw=YAW),
           Waypoint(0.6, (REST_X, y, REST_AXIS), FINGER_CLOSED, "grip", yaw=YAW),
           Waypoint(1.0, (REST_X, y, REST_AXIS + LIFT), FINGER_CLOSED, "lift the pin out of its rest", yaw=YAW),
           Waypoint(1.5, (CENTER_X, y, AXIS_Z + LIFT), FINGER_CLOSED, "over the middle of the block", yaw=YAW),
           Waypoint(1.2, (CENTER_X, y, AXIS_Z - PRESS), FINGER_CLOSED, "rings down on the board", yaw=YAW)]
    x = CENTER_X
    for k, end in enumerate(STROKES):
        out.append(Waypoint(abs(end - x) / SPEED + 0.3, (end, y, AXIS_Z - PRESS), FINGER_CLOSED, f"stroke {k + 1}",
                            yaw=YAW))
        x = end
    out += [Waypoint(1.0, (x, y, AXIS_Z + LIFT), FINGER_CLOSED, "lift the pin off the pastry", yaw=YAW),
            Waypoint(1.5, (REST_X, y, REST_AXIS + LIFT), FINGER_CLOSED, "over its rest", yaw=YAW),
            Waypoint(1.2, (REST_X, y, REST_AXIS), FINGER_CLOSED, "into its rest", yaw=YAW),
            Waypoint(0.6, (REST_X, y, REST_AXIS), FINGER_OPEN, "let go", yaw=YAW),
            Waypoint(1.0, (REST_X, y, REST_AXIS + LIFT), FINGER_OPEN, "clear of the pin", yaw=YAW)]
    return out


def demo():
    """Both arms on the pin's handles; strokes from the middle out to each end and back; let go; home."""
    left, right = _arm(GRIP_Y), _arm(-GRIP_Y)
    tail = [kit.Home(2.0), kit.Home(SUCCESS_HOLD_S + 1.0, label="hold home")]
    return {"left": left + tail, "right": right + list(tail)}
