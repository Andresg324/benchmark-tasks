"""Half a sheet of store-bought puff pastry (Pepperidge Farm Puff Pastry Sheets: one 24 x 23 cm sheet, 3/16 inch thick,
thawed cold and unfolded, cut in half) on a floured maple board, and a handled maple rolling pin (12 inch barrel, 2.4
inch across) with 1/8 inch silicone spacer rings on both ends of its barrel, lying on the board behind the pastry.

The pastry is rolled to an even 1/8 inch (3.2 mm), the thickness recipes roll this pastry to (the maker's own recipes
roll a sheet out to 15 x 10 inches, which is 1/8 inch from its 3/16). The rings set that thickness only while both ride
on the board: then the barrel runs 3.2 mm above it all the way across. A pin with one end lifted leaves a wedge (thick
on that side), and the dough springs back, so it takes several passes to come down to the rings.

Model (doughkit): the pastry has no collider; the pin rides on its rings. Each slice of pastry that the barrel rolls
across with its lowest line below the slice's top is pressed to gap + half its excess over the gap (the gap is the
barrel's height above the board where it crosses), the slice lengthens and the pastry beyond it moves out, volume
kept. Rolling past an end presses the end like any other slice (the rings hold the barrel 3.2 mm up there too).
The model gives the pin no rolling resistance (the pastry pushes back on nothing), so in the sim the rings keep the
pin level on the flat board whatever the hands do; a real trial checks the sheet's evenness directly (a depth gauge
at the same grid of points the sim reads).
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props; nothing on disk changes)
import guardkit as gk  # noqa: E402  (grip and contact checks)
import doughkit as dk  # noqa: E402  (the rolling model, updated every control step)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Roll this puff pastry out to an even 1/8 inch (3.2 mm) thick."

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
BOARD = (0.25, 0.0)  # the pastry board's center (x, y)
BOARD_SIZE = (0.30, 0.40, 0.019)  # 30 cm deep, 40 cm across, 1.9 cm thick: a 12 x 16 x 3/4 inch maple board
BOARD_FRICTION = (0.15, 0.10)  # static, dynamic: stands in for a barrel that turns on its bushings (see README)
BZ = TOP + BOARD_SIZE[2]  # the board's top
X_RANGE = (BOARD[0] - BOARD_SIZE[0] / 2, BOARD[0] + BOARD_SIZE[0] / 2)  # the board's front and back edges
CELL = 0.02  # m: the pastry model's cell (a 2 cm grid lined up with the board's front edge and center line)
ROWS = (0, 2, 4, 7, 9, 11)  # the 2 cm bands read: centered 11, 7 and 3 cm either side of the center line
COLS = slice(0, None, 2)  # every other 2 cm cell along x: x 11, 15, 19, ... cm (a 4 cm pitch from the front edge)

PASTRY = (0.24, 0.0)  # the half sheet's center
PASTRY_SIZE = (0.115, 0.24, 0.0048)  # half of a 23 x 24 cm sheet: 11.5 cm (x) by 24 cm (y), 4.8 mm (3/16 inch) thick

R_BARREL = 0.0305  # the pin's barrel: 6.1 cm (2.4 inch) across ...
BARREL_HALF = 0.1525  # ... 30.5 cm (12 inch) long
RING_GAP = 0.0032  # the 1/8 inch spacer rings stand 3.2 mm proud of the barrel ...
R_RING = R_BARREL + RING_GAP
RING_IN, RING_OUT = 0.1375, 0.1525  # ... and sit on the barrel's last 1.5 cm at each end (outside the 24 cm sheet)
R_HANDLE = 0.0125  # handles 2.5 cm across ...
HANDLE_IN, HANDLE_OUT = 0.1525, 0.2665  # ... 11.4 cm (4.5 inch) long; 53.3 cm (21 inch) overall
PIN_MASS = 0.73  # kg (1.6 lb)
PIN_X = 0.35  # the pin lies on its rings on the board, axis along y, behind the pastry
PIN_START = (PIN_X, 0.0, BZ + R_RING)

# --- The rule's numbers (reasons in score() and the README).
TARGET_T = 0.0032  # m: 1/8 inch, the sheet's thickness (the rings' gap)
TOL_T = 0.0003  # m: a point is on spec within 0.3 mm of 3.2 mm (2.9 to 3.5 mm, about 10 %)
SPEC_SHARE = 0.90  # share of the measured points that must be on spec (the outer row of points, at the cut and rolled
#                    edges, is half of the 24 points; 10 % lets two of 24 be off)
HALF_T = 0.0037  # m: at least 90 % of points at most 3.7 mm (within 0.5 mm, about one pass to go) earns half
QUARTER_T = 0.0042  # m: at least 90 % of points at most 4.2 mm (0.6 mm off the 4.8 mm start) and even earns a quarter
WEDGE_MAX = 0.0003  # m: the 2 cm bands' average thicknesses may differ by at most this across the width (the same
#                     0.3 mm a point is allowed off 3.2 mm); a pin rolled with one end lifted leaves the sheet thicker
#                     on that side, and puff pastry rises higher and bakes slower where it is thicker
PIN_DOWN = 0.005  # m: rings within 5 mm of the board or table, level within LEVEL_DEG: set down
LEVEL_DEG = 3.0

GRIP_Y = 0.195  # where the hands take the handles (|y|), 4.25 cm out from the barrel's end

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


OBJECTS = [_board(), _pin()]

# The pin's shape in its own frame (axis along y), as its colliders: the barrel, a spacer ring on each end of it, and
# the handles (left at +y, right at -y), as flat-ended cylinders.
PIN_SHAPE = (gk.Cyl((0, -BARREL_HALF, 0), (0, BARREL_HALF, 0), R_BARREL),
             gk.Cyl((0, RING_IN, 0), (0, RING_OUT, 0), R_RING),
             gk.Cyl((0, -RING_IN, 0), (0, -RING_OUT, 0), R_RING),
             gk.Cyl((0, HANDLE_IN, 0), (0, HANDLE_OUT, 0), R_HANDLE),
             gk.Cyl((0, -HANDLE_IN, 0), (0, -HANDLE_OUT, 0), R_HANDLE))


def _gate(objects):
    """True while both hands are on the pin: each gripper's own shape (palm and fingers at their present opening)
    touching it within 2 mm (gk.touching), anywhere on its handles, barrel or rings, gripping it or pressing on it with
    a closed hand. The sim's pin slides on its rings instead of turning, so the model cannot tell rolling from pushing
    the pin along; it counts presses only while two hands are on the pin, so that both ends can be kept down. A hand
    hovering off the pin does not count, and strokes made with one hand (or none) just do not press."""
    return len(set(gk.touching(objects, "pin", PIN_SHAPE))) >= 2


dk.setup(dk.Sheet(center=PASTRY, size=PASTRY_SIZE, board_z=BZ, x_range=X_RANGE, cell=CELL),
         dk.Pin("pin", radius=R_BARREL, length=2 * BARREL_HALF), gate=_gate)


def _rotm(q):
    return dk._rotm(q)


def pin_down(objects):
    """The pin set down, axis within 3 degrees of level, lying on its rings (their bottoms within 5 mm of the board's
    top, or of the table's beside it)."""
    o = objects["pin"]
    a = _rotm(o.quat_xyzw) @ np.array([0.0, 1.0, 0.0])
    if math.degrees(math.asin(min(1.0, abs(a[2])))) > LEVEL_DEG:
        return False
    z = float(o.pos[2]) - R_RING
    return abs(z - BZ) <= PIN_DOWN or abs(z - TOP) <= PIN_DOWN


def released(objects):
    """Let go: no gripper touching the pin's barrel, rings or handles (gk.touching, the gripper's own shape within
    2 mm). An open hand still around a handle without touching it has let go; the pin rests on its own."""
    return not gk.touching(objects, "pin", PIN_SHAPE)


def points(objects=None):
    """The thickness (m) at each measuring point, on a 4 cm grid: every other 2 cm cell along x from the board's front
    edge (x 11, 15, 19, ... cm), in the bands 3, 7 and 11 cm either side of its center line (rows), 0 where there is no
    pastry. 24 points on the demo's sheet."""
    return dk.grid()[list(ROWS)][:, COLS]


def sheet(objects=None):
    """(on-spec share, share at most HALF_T, share at most QUARTER_T, thinnest point m) over the points covered."""
    t = points(objects)
    p = t[t > 0]
    if p.size == 0:
        return 0.0, 0.0, 0.0, 0.0
    on = float((np.abs(p - TARGET_T) <= TOL_T + 1e-9).mean())
    return on, float((p <= HALF_T + 1e-9).mean()), float((p <= QUARTER_T + 1e-9).mean()), float(p.min())


def wedge(objects=None):
    """How much thicker the thickest band read is than the thinnest, each averaged over its points (m): 0 for a pin
    kept level, which presses every band alike; a pin with one end lifted leaves that side thicker."""
    t = points(objects)
    means = [row[row > 0].mean() for row in t if (row > 0).any()]
    return float(max(means) - min(means)) if means else 0.0


def on_board(objects=None):
    """All the pastry still on the board (within its front and back edges), where it can be measured."""
    e = dk.ends()
    return bool(e[:, 0].min() >= X_RANGE[0] - 1e-9 and e[:, 1].max() <= X_RANGE[1] + 1e-9)


def _job(objects):
    on, half, quarter, _ = sheet(objects)
    even = wedge(objects) <= WEDGE_MAX + 1e-9
    if not on_board(objects):
        return 0.0  # pastry pushed off the board
    if on >= SPEC_SHARE and even:
        s = 1.0
    elif half >= SPEC_SHARE:
        s = 0.5  # within 0.5 mm nearly everywhere, but not finished or not even (about one pass to go)
    elif quarter >= SPEC_SHARE and even:
        s = 0.25  # thinned evenly by at least 0.6 mm: rolled level, about two passes to go
    else:
        return 0.0  # nothing done, under one pass, or a wedge from a tilted pin
    if not released(objects) or not pin_down(objects):
        return min(s, 0.5)  # a hand still touching the pin, or the pin not set down
    return s


def _score(objects):
    return _job(objects)


def success(objects):
    return _score(objects) == 1.0


def score(objects):
    """The pastry's thickness is read on a 4 cm grid: the centers of every other 2 cm square along x from the board's
    front edge, in the bands 3, 7 and 11 cm either side of its center line, at every point the pastry covers (24 on
    the demo's sheet; the same points a real trial reads with a depth gauge). Doing
    nothing scores 0 (the half sheet starts 4.8 mm, 3/16 inch, thick). In the sim only, rolling counts while both
    hands are on the pin (each gripper touching it within 2 mm; the sim's pin slides rather than turns, so this stands
    in for a pin rolled under two hands, which can keep both ends down). 1.0 if at least 90 % of the points (22 of
    24) are 3.2 mm (1/8 inch, the rings' gap) within 0.3 mm, the bands' average thicknesses differ by at most 0.3 mm across the
    width (a pin with one end lifted leaves a wedge), all the pastry is still on the board, and the pin is set down
    level (within 3 degrees) on its rings on the board or table (within 5 mm) with no hand touching it (2 mm contact
    tolerance). 0.5 if at least 90 % of points are at most 3.7 mm (about one pass to go) but the sheet is not finished,
    or if the sheet is done but a hand still touches the pin or it is not set down. 0.25 if at least 90 % of points
    are at most 4.2 mm and the bands are within 0.3 mm of each other (about two passes to go). 0 otherwise (nothing
    done, one pass, a wedge left by a tilted pin, or pastry pushed off the board). Where the arms end is not scored.
    Possible scores: 0, 0.25, 0.5, 1.0."""
    return _score(objects)


# --- Demo: both arms take the pin by its handles where it lies, lift it over the middle of the pastry, set it down on
# its rings and roll from the middle out to each end and back, stopping short of the end; then lift it back to where
# it lay, let go and go home.
CENTER_X = PASTRY[0]
AXIS_Z = BZ + R_RING  # tool point on the handle's axis with the pin on its rings
PRESS = 0.004  # m: the hands are commanded this far below the axis, so the rings stay on the board
LIFT = 0.05
SPEED = 0.04  # m/s along a stroke
YAW = math.pi / 2  # fingers close along x, across the handle
# Stroke ends (x of the barrel's line), four passes, each from the middle out to the far end and back, then out to
# the near end and back, aimed about 6 mm past the end as it then lies (worked out with doughkit): the pin trails its
# hands by about 1 cm at each turn, and rolling past an end presses the end like any other slice. The farthest, x 33
# cm with the hands at |y| 19.5 cm, is 0.35 m from the arm's base; the nearest is x 15 cm.
STROKES = (0.305, 0.24, 0.175, 0.24, 0.32, 0.24, 0.16, 0.24, 0.33, 0.24, 0.15, 0.24, 0.33, 0.24, 0.15, 0.24)


def _arm(y):
    half = FINGER_OPEN
    out = [Waypoint(2.5, (PIN_X, y, AXIS_Z + LIFT), half, "over the handle", yaw=YAW),
           Waypoint(1.2, (PIN_X, y, AXIS_Z), half, "around the handle", yaw=YAW),
           Waypoint(0.6, (PIN_X, y, AXIS_Z), FINGER_CLOSED, "grip", yaw=YAW),
           Waypoint(1.0, (PIN_X, y, AXIS_Z + LIFT), FINGER_CLOSED, "lift the pin", yaw=YAW),
           Waypoint(1.5, (CENTER_X, y, AXIS_Z + LIFT), FINGER_CLOSED, "over the middle of the pastry", yaw=YAW),
           Waypoint(1.2, (CENTER_X, y, AXIS_Z - PRESS), FINGER_CLOSED, "rings down on the board", yaw=YAW)]
    x = CENTER_X
    for k, end in enumerate(STROKES):
        out.append(Waypoint(abs(end - x) / SPEED + 0.3, (end, y, AXIS_Z - PRESS), FINGER_CLOSED, f"stroke {k + 1}",
                            yaw=YAW))
        x = end
    out += [Waypoint(1.0, (x, y, AXIS_Z + LIFT), FINGER_CLOSED, "lift the pin off the pastry", yaw=YAW),
            Waypoint(1.5, (PIN_X, y, AXIS_Z + LIFT), FINGER_CLOSED, "back over where it lay", yaw=YAW),
            Waypoint(1.2, (PIN_X, y, AXIS_Z), FINGER_CLOSED, "down on its rings", yaw=YAW),
            Waypoint(0.6, (PIN_X, y, AXIS_Z), FINGER_OPEN, "let go", yaw=YAW),
            Waypoint(1.0, (PIN_X, y, AXIS_Z + LIFT), FINGER_OPEN, "clear of the pin", yaw=YAW)]
    return out


def demo():
    """Both arms on the pin's handles; strokes from the middle out to each end and back; let go; home."""
    left, right = _arm(GRIP_Y), _arm(-GRIP_Y)
    tail = [kit.Home(2.0), kit.Home(SUCCESS_HOLD_S + 1.0, label="hold home")]
    return {"left": left + tail, "right": right + list(tail)}
