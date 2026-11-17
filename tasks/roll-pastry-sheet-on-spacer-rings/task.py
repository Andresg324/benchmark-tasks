"""A sheet of store-bought crescent roll dough (Pillsbury Original Crescent, from the refrigerated 8 oz can, at room
temperature: balled, rolled out between two 3/16 inch guide strips and trimmed to 11.5 x 24 cm) on a floured maple
board, and a handled maple rolling pin (12 inch barrel, 2.4 inch across) with 1/8 inch silicone spacer rings on both
ends of its barrel, lying on the board behind the dough.

The dough is rolled to an even 1/8 inch (3.2 mm). The rings set that thickness only while both ride on the board: then
the barrel runs 3.2 mm above it all the way across. A pin with one end lifted leaves a wedge (thick on that side), and
the dough springs back, so it takes several passes. Then the pin is put away: set down off the dough and let go.

Model (doughkit): the dough has no collider; the pin rides on its rings. The dough is elastic up to a yield strain (5 %)
and then flows at its flow stress (DOUGH_STRESS). Each slice the barrel rolls across, or presses down on, with its
lowest line held below the slice's top for two control steps is squeezed by h; if h is past the yield squeeze
(5 % of the slice's thickness) the slice thins by half the excess beyond it (the rest springs back), lengthens, and the
dough beyond it moves out, volume kept; under it the dough springs fully back. The dough pushes back on the pin
(elastically, then at the flow stress, over the barrel's contact) and against its motion, so the arms have to hold the
pin down and drive it through: a pin that is not pressed rides up and presses less. One or two hands may roll; the pin's
own weight stays under the yield load. A real trial checks the sheet's evenness directly (a caliper at the same points
the sim reads).
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
INSTRUCTION = "Roll this crescent dough out to an even 1/8 inch (3.2 mm) thick, then set the rolling pin down off the dough."

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
BOARD = (0.25, 0.0)  # the board's center (x, y)
BOARD_SIZE = (0.30, 0.40, 0.019)  # 30 cm deep, 40 cm across, 1.9 cm thick: a 12 x 16 x 3/4 inch maple board
BOARD_FRICTION = (0.25, 0.20)  # static, dynamic: wood on wood (Serway and Jewett, Physics for Scientists and
#                                Engineers, Table 5.1: 0.25 to 0.5 static, 0.2 kinetic; the low static end, the board
#                                floured); the sim's pin slides on its rings where the real barrel turns (README)
BZ = TOP + BOARD_SIZE[2]  # the board's top
X_RANGE = (BOARD[0] - BOARD_SIZE[0] / 2, BOARD[0] + BOARD_SIZE[0] / 2)  # the board's front and back edges
CELL = 0.02  # m: the dough model's cell (a 2 cm grid lined up with the board's front edge and center line)
ROWS = (0, 2, 4, 7, 9, 11)  # the 2 cm bands read: centered 11, 7 and 3 cm either side of the center line
COLS = slice(0, None, 2)  # every other 2 cm cell along x: x 11, 15, 19, ... cm (a 4 cm pitch from the front edge)
END_IN = 0.01  # m: each band is also read 1 cm inside each of its rolled ends, so an unrolled end shows up
END_CLEAR = 0.02  # m: a row point within 2 cm of an end point is left to the end point (no near-duplicate readings)

DOUGH = (0.24, 0.0)  # the sheet's center
DOUGH_SIZE = (0.115, 0.24, 0.0048)  # rolled between 3/16 inch guide strips and trimmed: 11.5 cm (x) by 24 cm (y),
#                                     4.8 mm thick (132.5 mL, from one 8 oz can)

R_BARREL = 0.0305  # the pin's barrel: 6.1 cm (2.4 inch) across ...
BARREL_HALF = 0.1525  # ... 30.5 cm (12 inch) long
RING_GAP = 0.0032  # the 1/8 inch spacer rings stand 3.2 mm proud of the barrel ...
R_RING = R_BARREL + RING_GAP
RING_IN, RING_OUT = 0.1375, 0.1525  # ... and sit on the barrel's last 1.5 cm at each end (outside the 24 cm sheet)
R_HANDLE = 0.0125  # handles 2.5 cm across ...
HANDLE_IN, HANDLE_OUT = 0.1525, 0.2665  # ... 11.4 cm (4.5 inch) long; 53.3 cm (21 inch) overall
PIN_MASS = 0.73  # kg (1.6 lb)
DOUGH_STRESS = 13e3  # Pa: the dough's flow stress, from a hand check on this dough as trials use it (README): a first
#                      pass (4.8 mm sheet, 24 cm wide, 1.6 mm off) read 29 to 34 N at its peak on a kitchen scale under
#                      the board; F = stress x 0.24 m x sqrt(2 R h - h^2) = stress x 2.34e-3 m^2 gives 12.4 to 14.5 kPa.
#                      At 13 kPa the first pass pushes the pin up 30.4 N and back 4.2 N
PIN_X = 0.35  # the pin lies on its rings on the board, axis along y, behind the dough
PIN_START = (PIN_X, 0.0, BZ + R_RING)

# --- The rule's numbers (reasons in score() and the README).
TARGET_T = 0.0032  # m: 1/8 inch, the sheet's thickness (the rings' gap)
TOL_T = 0.0003  # m: a point is on spec within 0.3 mm of 3.2 mm (2.9 to 3.5 mm, about 10 %)
SPEC_SHARE = 0.90  # share of the measured points that must be on spec (the outer bands and the end points, at the cut
#                    and rolled edges, are most of the points; 10 % lets about three of 30 be off)
HALF_T = 0.0037  # m: at least 90 % of points at most 3.7 mm (within 0.5 mm, about one pass to go) earns half
QUARTER_T = 0.0042  # m: at least 90 % of points at most 4.2 mm (0.6 mm off the 4.8 mm start) and even earns a quarter
WEDGE_MAX = 0.0003  # m: the 2 cm bands' average thicknesses may differ by at most this across the width (the same
#                     0.3 mm a point is allowed off 3.2 mm); a pin rolled with one end lifted leaves the sheet thicker
#                     on that side, and crescent dough rises higher and bakes slower where it is thicker
PIN_DOWN = 0.005  # m: rings within 5 mm of the board or table, level within LEVEL_DEG: set down
LEVEL_DEG = 3.0

GRIP_Y = 0.195  # where the hands take the handles (|y|), 4.25 cm out from the barrel's end

WOOD = kit.Look((0.80, 0.62, 0.40), 0.6)
DARKWOOD = kit.Look((0.55, 0.36, 0.20), 0.5)
BOARDWOOD = kit.Look((0.66, 0.48, 0.30), 0.8)  # oiled maple, darker than the table so the dough reads on it
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


# --- Let go (contact between the arms and the pin): the arms' real collision meshes (yam_links.npz: yam_arena's arm description,
# link3 to the fingertips, as surface points every 1.5 mm in each link's own frame), posed each control step from the
# simulator's link poses, against the pin's colliders. A link touches the pin when one of its points lies on
# the pin's surface or inside it (overlap, as PhysX's contact leaves the pressed shapes). guardkit's box model of the
# gripper is off by several mm against the real meshes, so it is used only offline (no link poses), also as overlap.
TOUCH = 0.0  # m: overlap (a gap of 0 or less), not a proximity window
ARM_LINKS = ("link3", "link4", "link5", "gripper", "tip_left", "tip_right")  # the upper arm out to the fingertips
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}  # "left_robot/gripper" -> (pos, R) in the env frame, this control step (empty offline)
_MESH: dict = {}
_GAP_MEMO: dict = {}


def mesh(name):
    """(points (n x 3, m, in the link's frame), their center, radius about it) of an arm link's collision mesh."""
    if not _MESH:
        d = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "yam_links.npz"))
        unit = float(d["unit"][0])  # stored as int16 in 0.05 mm steps
        for k in d.files:
            if k != "unit":
                pts = d[k].astype(float) * unit
                c = pts.mean(axis=0)
                _MESH[k] = (pts, c, float(np.linalg.norm(pts - c, axis=1).max()))
    return _MESH[name]


def _cyl_gap(q, part) -> np.ndarray:
    """Distance (m, 0 inside) from pin-frame points q to a flat-ended cylinder `part`."""
    a, b = np.asarray(part.a, float), np.asarray(part.b, float)
    L = float(np.linalg.norm(b - a))
    u = (b - a) / L
    d = q - a
    s = d @ u
    radial = np.linalg.norm(d - s[:, None] * u, axis=1)
    return np.hypot(np.maximum(radial - part.r, 0.0), np.maximum(np.maximum(-s, s - L), 0.0))


def link_gap(objects, key, poses=None) -> float:
    """The smallest gap (m) between arm link `key` ("left_robot/tip_left") as posed and the pin's colliders."""
    poses = LINK_POSES if poses is None else poses
    o, R = poses[key]
    pin = objects["pin"]
    co, Ro = np.asarray(pin.pos, float), _rotm(pin.quat_xyzw)
    memo = (key, np.asarray(o, float).tobytes(), np.asarray(R, float).tobytes(), co.tobytes(), Ro.tobytes())
    if memo in _GAP_MEMO:
        return _GAP_MEMO[memo]
    pts, c, r = mesh(key.split("/")[1])
    centre = (np.asarray(o, float) + np.asarray(R, float) @ c - co) @ Ro
    if min(float(_cyl_gap(centre[None, :], p)[0]) for p in PIN_SHAPE) - r > 0.01:
        gap = 0.01 + 1e-6  # the whole link more than 1 cm off the pin
    else:
        q = (np.asarray(o, float) + pts @ np.asarray(R, float).T - co) @ Ro
        gap = min(float(_cyl_gap(q, p).min()) for p in PIN_SHAPE)
    if len(_GAP_MEMO) > 512:
        _GAP_MEMO.clear()
    _GAP_MEMO[memo] = gap
    return gap


def _rotm(q):
    return dk._rotm(q)


def _read_links(base) -> None:
    """Every arm link's pose (env frame) this control step, into LINK_POSES (the simulator's quaternions x, y, z, w)."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError:
            continue
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        for link in ARM_LINKS:
            if link in names:
                i = names.index(link)
                LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, dk._rotm(Q[i]))


dk.setup(dk.Sheet(center=DOUGH, size=DOUGH_SIZE, board_z=BZ, x_range=X_RANGE, cell=CELL),
         dk.Pin("pin", radius=R_BARREL, length=2 * BARREL_HALF, stress=DOUGH_STRESS),
         before=_read_links)


PUT_CLEAR = 0.01  # m: put away means the barrel's lowest line at least 1 cm clear of the dough on the board


def clear_of_dough(objects) -> bool:
    """The pin is not lying on the dough: along the whole barrel its lowest line is at least PUT_CLEAR (1 cm) clear of
    the rolled sheet's outline (each 2 cm strip's x extent), whichever way the pin lies."""
    o = objects["pin"]
    R = _rotm(o.quat_xyzw)
    a = R @ np.array([0.0, 1.0, 0.0])
    ends = dk.ends()
    ys = dk._state().ys
    for s in np.linspace(-BARREL_HALF, BARREL_HALF, 61):
        x, y = (np.asarray(o.pos, float) + s * a)[:2]
        near = np.abs(ys - y) <= CELL / 2 + PUT_CLEAR
        if near.any() and ((ends[near, 0] - PUT_CLEAR <= x) & (x <= ends[near, 1] + PUT_CLEAR)).any():
            return False
    return True


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
    """Let go: no part of either arm touching the pin's barrel, rings or handles (in the sim the real link meshes, from
    the upper arm to the fingertips, within 0.5 mm; offline the gripper boxes). An open hand still around a handle
    without touching it has let go; the pin rests on its own."""
    if not LINK_POSES:
        return not gk.touching(objects, "pin", PIN_SHAPE, TOUCH)
    return all(link_gap(objects, k) > TOUCH for k in LINK_POSES)


def points(objects=None):
    """The thickness (m) at each measuring point, one row per band read (the 2 cm bands 3, 7 and 11 cm either side of
    the board's center line): the points every 4 cm along x from 1 cm behind the board's front edge (x 11, 15, 19, ...
    cm) that the dough covers, except any within 2 cm of an end point, then the two end points 1 cm inside the band's
    rolled ends (last two columns); 0 where nothing is read. 30 points on the demo's sheet."""
    xc = dk.cell_x()[COLS]
    grid = dk.grid()[list(ROWS)][:, COLS]
    strips = dk.strips()
    out = np.zeros((len(ROWS), grid.shape[1] + 2))
    for r, j in enumerate(ROWS):
        e, t = strips[j]
        ends = (e[0] + END_IN, e[-1] - END_IN)
        row = grid[r].copy()
        for k, x in enumerate(ends):
            row[np.abs(xc - x) < END_CLEAR] = 0.0
            i = int(np.clip(np.searchsorted(e, x, side="right") - 1, 0, len(t) - 1))
            out[r, grid.shape[1] + k] = t[i]
        out[r, :grid.shape[1]] = row
    return out


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
    """All the dough still on the board (within its front and back edges), where it can be measured."""
    e = dk.ends()
    return bool(e[:, 0].min() >= X_RANGE[0] - 1e-9 and e[:, 1].max() <= X_RANGE[1] + 1e-9)


def _job(objects):
    on, half, quarter, _ = sheet(objects)
    even = wedge(objects) <= WEDGE_MAX + 1e-9
    if not on_board(objects):
        return 0.0  # dough pushed off the board
    if on >= SPEC_SHARE and even:
        s = 1.0
    elif half >= SPEC_SHARE:
        s = 0.5  # within 0.5 mm nearly everywhere, but not finished or not even (about one pass to go)
    elif quarter >= SPEC_SHARE and even:
        s = 0.25  # thinned evenly by at least 0.6 mm: rolled level, about two passes to go
    else:
        return 0.0  # nothing done, under one pass, or a wedge from a tilted pin
    if not released(objects) or not pin_down(objects) or not clear_of_dough(objects):
        return min(s, 0.5)  # a hand still touching the pin, or the pin not put away (set down off the dough)
    return s


_DEBUG = int(os.environ.get("ROLL_DEBUG", "0") or 0)  # print the state every this many scoring calls (0: never)
_DBG = {"n": 0}


_MEMO = {"key": None, "out": None}


def _key(objects):
    """The step's state that the score depends on (success() and score() are both asked each control step)."""
    S = dk._state()
    parts = [kit.episode(), S.step, S.presses]
    for name in sorted(objects):
        o = objects[name]
        parts.append((name, np.asarray(o.pos, float).tobytes(), np.asarray(o.quat_xyzw, float).tobytes()))
    parts.append(tuple(sorted(kit._OPENING.items())) if hasattr(kit, "_OPENING") else None)
    parts.append(tuple((k, v[0].tobytes()) for k, v in sorted(LINK_POSES.items())))
    return tuple(parts)


def _score(objects):
    key = _key(objects)
    if key != _MEMO["key"]:
        _MEMO["key"], _MEMO["out"] = key, _job(objects)
    out = _MEMO["out"]
    if _DEBUG:
        _DBG["n"] += 1
        if _DBG["n"] % _DEBUG == 0:
            p = points(objects)
            gaps = {k: round(link_gap(objects, k) * 1e3, 2) for k in LINK_POSES if "tip" in k or "gripper" in k}
            print(f"roll dbg {_DBG['n']} score {out} sheet {np.round(sheet(objects), 3).tolist()} "
                  f"wedge {wedge(objects) * 1e3:.2f}mm max {p.max() * 1e3:.2f} min {p[p > 0].min() * 1e3:.2f} "
                  f"pin {np.round(np.asarray(objects['pin'].pos) * 1e3, 1).tolist()} rel {released(objects)} "
                  f"down {pin_down(objects)} clear {clear_of_dough(objects)} peak {np.round(dk.peak(), 1).tolist()} "
                  f"gaps_mm {gaps}", flush=True)
            a = _rotm(objects["pin"].quat_xyzw) @ np.array([0.0, 1.0, 0.0])
            lines = [dk.pin_line(objects, y) for y in (-0.11, 0.0, 0.11)]
            print(f"roll dbg2 tilt {math.degrees(math.asin(np.clip(a[2], -1, 1))):.2f}deg yaw "
                  f"{math.degrees(math.atan2(-a[0], a[1])):.2f}deg gaps {[round(l[1] * 1e3, 2) if l else None for l in lines]} "
                  f"bands {[round(float(r[r > 0].mean()) * 1e3, 2) for r in p if (r > 0).any()]} "
                  f"wrench {np.round(dk.wrench()[0], 1).tolist()}", flush=True)
    return out


def success(objects):
    return _score(objects) == 1.0


def score(objects):
    """The dough's thickness is read in the 2 cm bands 3, 7 and 11 cm either side of the board's center line: in each
    band at the points every 4 cm along x from 1 cm behind the board's front edge that the dough covers, and 1 cm
    inside each of the band's rolled ends (a point within 2 cm of an end point is left to it); 30 points on the demo's
    sheet, the same points a real trial reads with a caliper. Doing nothing scores 0 (the sheet starts 4.8 mm, 3/16
    inch, thick). One or two hands may roll; the dough thins only under a load past its yield. 1.0 if at least 90 % of
    the points are 3.2 mm (1/8 inch, the rings' gap) within 0.3 mm, the bands' average thicknesses differ by at most
    0.3 mm across the width (a pin with one end lifted leaves a wedge), all the dough is still on the board, and the
    pin is put away: set down level (within 3 degrees) on its rings on the board or table (within 5 mm), its barrel at
    least 1 cm clear of the dough, with no part of the arms touching it (overlap of their real shapes). 0.5 if at least
    90 % of points are at most 3.7 mm (about one pass to go) but the sheet is not finished, or if the sheet is done but
    the pin is not put away (held, left on the dough, or not set down). 0.25 if at least 90 % of points are at most
    4.2 mm and the bands are within 0.3 mm of each other (about two passes to go). 0 otherwise (nothing done, one
    pass, a wedge left by a tilted pin, or dough pushed off the board). Where the arms end is not scored. Possible
    scores: 0, 0.25, 0.5, 1.0."""
    return _score(objects)


# --- Demo: both arms take the pin by its handles where it lies, lift it over the middle of the dough, set it down on
# its rings and roll from the middle out to each end and back, stopping short of the end; then lift it back to where
# it lay, let go and go home.
CENTER_X = DOUGH[0]
AXIS_Z = BZ + R_RING  # tool point on the handle's axis with the pin on its rings
PRESS = 0.010  # m: the hands are commanded this far below the axis, so they hold the rings down on the board against
#                the dough's push (about 30 N up on the first pass)
LIFT = 0.05
SPEED = 0.04  # m/s along a stroke
TURN_HOLD = 0.5  # s: held at each turn, so the pin, which trails the hands against the dough's push, reaches the end
YAW = math.pi / 2  # fingers close along x, across the handle
# Stroke ends (x of the barrel's line), five passes, each from the middle out to the far end and back, then out to
# the near end and back, aimed about 6 to 8 mm past the end as it then lies (worked out with doughkit): the pin trails its
# hands by about 1 cm at each turn, and rolling past an end presses the end like any other slice. Offline the five
# passes finish the sheet with the pin stopping anywhere from at to 2 cm short of each end. The farthest, x 33 cm with
# the hands at |y| 19.5 cm, is 0.35 m from the arm's base; the nearest is x 15 cm.
STROKES = (0.305, 0.24, 0.175, 0.24, 0.32, 0.24, 0.16, 0.24, 0.33, 0.24, 0.15, 0.24, 0.33, 0.24, 0.15, 0.24,
           0.33, 0.24, 0.15, 0.24)


def _arm(y):
    half = FINGER_OPEN
    out = [Waypoint(2.5, (PIN_X, y, AXIS_Z + LIFT), half, "over the handle", yaw=YAW),
           Waypoint(1.2, (PIN_X, y, AXIS_Z), half, "around the handle", yaw=YAW),
           Waypoint(0.6, (PIN_X, y, AXIS_Z), FINGER_CLOSED, "grip", yaw=YAW),
           Waypoint(1.0, (PIN_X, y, AXIS_Z + LIFT), FINGER_CLOSED, "lift the pin", yaw=YAW),
           Waypoint(1.5, (CENTER_X, y, AXIS_Z + LIFT), FINGER_CLOSED, "over the middle of the dough", yaw=YAW),
           Waypoint(1.2, (CENTER_X, y, AXIS_Z - PRESS), FINGER_CLOSED, "rings down on the board", yaw=YAW)]
    x = CENTER_X
    for k, end in enumerate(STROKES):
        out.append(Waypoint(abs(end - x) / SPEED + 0.3, (end, y, AXIS_Z - PRESS), FINGER_CLOSED, f"stroke {k + 1}",
                            yaw=YAW))
        out.append(Waypoint(TURN_HOLD, (end, y, AXIS_Z - PRESS), FINGER_CLOSED, "hold at the turn", yaw=YAW))
        x = end
    out += [Waypoint(1.0, (x, y, AXIS_Z + LIFT), FINGER_CLOSED, "lift the pin off the dough", yaw=YAW),
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
