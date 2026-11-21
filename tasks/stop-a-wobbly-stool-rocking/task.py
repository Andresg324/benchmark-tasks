"""A one-step hardwood stool (an eHemco solid hardwood step stool, 31 cm tall) stands on the table in front of the arms.
One of its four legs is 4 mm shorter than the others, so the stool rocks: it rests on three feet, and a press on the
corner over the fourth foot tips it onto that foot and lifts the foot diagonally opposite. Two rigid plastic shims
(Wobble Wedges, the standard size) lie on the table, one beside each side of the stool. The instruction is only to stop
the stool rocking.

A wobble is the stool's four feet not lying in one plane. It rocks about the diagonal through the two feet that always
bear (the short leg's neighbors); the two feet that take turns lifting are the short leg and its diagonal partner. A
shim of the right thickness under either of those two feet brings all four feet down together; a shim under one of the
other two feet makes the rock worse. Which leg is short changes with the seed (STOOL_SEED, environment variable,
default 0: leg = seed mod 4), so the robot must find the rocking corners (by pressing the corners, or by looking for
the foot that hangs) before it shims.

The stool and the shims are free rigid bodies; a shim lifts a foot only by contact (its sloped top under the foot's
flat bottom) and stays by friction. The scorer tests the rock as a person would: once both arms have been at least
2 cm from the stool for 2 s and the stool is still, it presses each seat corner down with 20 N in turn (a force the
scorer applies to the stool in the simulator) and reads how far each foot moves between the presses.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import gripkit  # noqa: E402
import yamgeom as yg  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Stop the stool from rocking."
SEED = int(os.environ.get("STOOL_SEED", "0"))
SHORT = SEED % 4  # which leg is short (see CORNERS)

# Friction (static, dynamic, combine). Wood on wood, clean and dry: 0.4 / 0.3 (Engineering ToolBox gives 0.25-0.5).
# Polypropylene on wood: 0.4 / 0.3 as well (the shim's ribbed faces are the maker's "non-slip"). "min": the lower of the
# two touching surfaces, so the YAM's rubber pads (1.0 / 0.8) grip the shim at the shim's own value.
WOOD_MU = (0.4, 0.3, "min")
SHIM_MU = (0.4, 0.3, "min")
WOOD = kit.Look((0.78, 0.60, 0.40), 0.7)
WOOD_DARK = kit.Look((0.62, 0.45, 0.28), 0.7)
SHIM_LOOK = kit.Look((0.06, 0.06, 0.07), 0.6)  # the rigid wedges come clear or black; black shows on the oak table

# -- the stool (eHemco solid hardwood step stool, 1 step: 12.25 in tall, 12.75 x 12.75 in overall, 6.6 lb; square top,
# four splayed legs, four stretchers). Its own frame: origin on the table under the seat's middle (the long legs' feet
# at z = 0), x forward (away from the robot), y to the robot's left. Part sizes are estimated from maker photos.
STOOL_H = 0.311  # 12.25 in
FOOT_SPAN = 0.324  # 12.75 in: outside to outside of the feet
SEAT = 0.280  # square top
SEAT_T = 0.022
LEG_W = 0.032  # square legs
FOOT_C = FOOT_SPAN / 2 - LEG_W / 2  # a foot's middle from the stool's middle, along x and along y (0.146)
LEG_TOP_C = 0.118  # a leg's middle at its top (under the seat), along x and y: the legs splay out 28 mm
SEAT_Z0 = STOOL_H - SEAT_T
RAIL_Z, RAIL_W, RAIL_H = 0.095, 0.018, 0.020  # the stretchers: middle height, thickness, height
STOOL_KG = 3.0  # 6.6 lb
STOOL_COM = (0.0, 0.0, 0.22)  # m: its middle, 22 cm up (the seat is most of the wood)
SHORT_BY = 0.004  # m: the short leg
CORNERS = ((-1, 1), (1, 1), (1, -1), (-1, -1))  # legs 0..3: front-left, back-left, back-right, front-right
LEG_NAMES = ("front left", "back left", "back right", "front right")
STOOL_XY = (0.340, 0.0)  # world: the stool's middle, 34 cm in front of the arm bases
PRESS_IN = 0.115  # m: the scorer's press points, on the seat top over each leg, this far from the middle along x and y


def partner(leg: int) -> int:
    return (leg + 2) % 4


def _prism(bottom, top, look, material=None):
    """A convex piece (a hull collider and its visible mesh) between a bottom and a top ring of four points (counter-
    clockwise seen from above)."""
    pts = np.vstack([np.asarray(bottom, float), np.asarray(top, float)])
    c = pts.mean(axis=0)
    n = 4
    f = []
    for k in range(n):
        a, b = k, (k + 1) % n
        f += [(a, b, b + n), (a, b + n, a + n)]
    f += [(0, k + 1, k) for k in range(1, n - 1)]
    f += [(n, n + k, n + k + 1) for k in range(1, n - 1)]
    col = kit.Collider(tuple(float(v) for v in c), hull=tuple(tuple(float(v) for v in p - c) for p in pts),
                       faces=tuple(f), material=material)
    return kit._mesh(pts, [3] * len(f), [i for t in f for i in t], look), col


def _square(cx, cy, h, z):
    return [(cx - h, cy - h, z), (cx + h, cy - h, z), (cx + h, cy + h, z), (cx - h, cy + h, z)]


def leg_c(z: float) -> float:
    """A leg's middle (from the stool's middle, along x and along y) at height z."""
    return FOOT_C + (LEG_TOP_C - FOOT_C) * z / SEAT_Z0


def foot_z(leg: int, short: int = SHORT) -> float:
    return SHORT_BY if leg == short else 0.0


def _stool_parts(short: int):
    out = []
    h = SEAT / 2
    out.append((kit.box((0, 0, SEAT_Z0 + SEAT_T / 2), (SEAT, SEAT, SEAT_T), WOOD),
                kit.Collider((0.0, 0.0, SEAT_Z0 + SEAT_T / 2), size=(SEAT, SEAT, SEAT_T))))
    del h
    for i, (sx, sy) in enumerate(CORNERS):
        z0 = foot_z(i, short)
        bottom = _square(sx * leg_c(z0), sy * leg_c(z0), LEG_W / 2, z0)
        top = _square(sx * LEG_TOP_C, sy * LEG_TOP_C, LEG_W / 2, SEAT_Z0)
        out.append(_prism(bottom, top, WOOD_DARK))
    c = leg_c(RAIL_Z)
    inner = c - LEG_W / 2
    for sy in (1, -1):  # side stretchers along x
        size = (2 * inner, RAIL_W, RAIL_H)
        ctr = (0.0, sy * c, RAIL_Z)
        out.append((kit.box(ctr, size, WOOD_DARK), kit.Collider(ctr, size=size)))
    for sx in (1, -1):  # front and back stretchers along y
        size = (RAIL_W, 2 * inner, RAIL_H)
        ctr = (sx * c, 0.0, RAIL_Z)
        out.append((kit.box(ctr, size, WOOD_DARK), kit.Collider(ctr, size=size)))
    return out


def _axis_rot(u, a):
    u = np.asarray(u, float) / np.linalg.norm(u)
    k = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * k @ k


def foot_points(short: int = SHORT) -> np.ndarray:
    """(4, 3): the middle of each foot's flat bottom, in the stool's frame."""
    return np.array([(sx * leg_c(foot_z(i, short)), sy * leg_c(foot_z(i, short)), foot_z(i, short))
                     for i, (sx, sy) in enumerate(CORNERS)])


def _quat(R):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    R = np.asarray(R, float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2 * math.sqrt(1 + tr)
        q = ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4)
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        q = (s / 4, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    elif R[1, 1] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        q = ((R[0, 1] + R[1, 0]) / s, s / 4, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    else:
        s = 2 * math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
        q = ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4, (R[1, 0] - R[0, 1]) / s)
    return tuple(float(v) for v in q)


STOOL_R0 = np.eye(3)  # it stands level on its three long legs, the short foot 4 mm up
_FEET_LOW = min(float((np.array(b) @ STOOL_R0.T)[2]) for i in range(4) for b in _square(
    CORNERS[i][0] * leg_c(foot_z(i)), CORNERS[i][1] * leg_c(foot_z(i)), LEG_W / 2, foot_z(i)))
STOOL_POS = (STOOL_XY[0], STOOL_XY[1], TOP - _FEET_LOW + 0.0002)  # the lowest foot corner 0.2 mm over the table
_SP = _stool_parts(SHORT)
STOOL = kit.Prop(
    "stool", pos=STOOL_POS, mass=STOOL_KG, size=(FOOT_SPAN, FOOT_SPAN, STOOL_H),
    meshes=tuple(m for m, _ in _SP), colliders=tuple(c for _, c in _SP), tilt_xyzw=_quat(STOOL_R0),
    material=WOOD_MU, com=STOOL_COM, solver_iterations=(32, 4))

# -- the shims (Wobble Wedges, standard rigid: 1.92 x 1.11 in, tapering from 0.03 in to 0.25 in, polypropylene). Own
# frame: origin at the middle of the flat bottom, x from the thin end (-x) to the thick end (+x), z up.
SHIM_L, SHIM_W = 0.0488, 0.0282
SHIM_T0, SHIM_T1 = 0.00076, 0.00635
SHIM_SLOPE = (SHIM_T1 - SHIM_T0) / SHIM_L
SHIM_KG = 0.0044  # 4.9 cm3 of polypropylene (0.9 g/cm3)
SHIM_Y = 0.235  # world: the shims' middles beside the stool, at y = +-SHIM_Y, x = the stool's middle


def shim_thickness(u: float) -> float:
    """The shim's thickness (m) u m from its thin end."""
    return SHIM_T0 + SHIM_SLOPE * min(max(u, 0.0), SHIM_L)


def _shim_part():
    x0, x1, w = -SHIM_L / 2, SHIM_L / 2, SHIM_W / 2
    bottom = [(x0, -w, 0.0), (x1, -w, 0.0), (x1, w, 0.0), (x0, w, 0.0)]
    top = [(x0, -w, SHIM_T0), (x1, -w, SHIM_T1), (x1, w, SHIM_T1), (x0, w, SHIM_T0)]
    return _prism(bottom, top, SHIM_LOOK, SHIM_MU)


_SH = _shim_part()


def _shim(name, side):
    """side +1: on the robot's left of the stool, thin end toward the stool (-y); -1: on the right."""
    yaw = math.pi / 2 if side > 0 else -math.pi / 2
    return kit.Prop(name, pos=(STOOL_XY[0], side * SHIM_Y, TOP + 0.0002), yaw=yaw, mass=SHIM_KG,
                    size=(SHIM_L, SHIM_W, SHIM_T1), meshes=(_SH[0],), colliders=(_SH[1],), material=SHIM_MU,
                    solver_iterations=(32, 4))


SHIMS = ("shim_left", "shim_right")
OBJECTS = [STOOL, _shim("shim_left", 1), _shim("shim_right", -1)]
gripkit.use_rubber_pads()  # the YAM's finger pads are rubber-faced

# -- scoring ---------------------------------------------------------------------------------------------------------
PRESS_N = 20.0  # N: the scorer's press on each seat corner (about the weight of a 2 kg bag)
CLEAR = 0.02  # m: arms clear = no point of any arm link within 2 cm of the stool's box (feet to seat, splay included)
CLEAR_S = 2.0
STILL_V, STILL_W = 0.01, 0.05  # m/s, rad/s: the stool still before the presses start
RAMP, HOLD = 4, 10  # control steps: each press goes on over 0.2 s, holds 0.5 s (read at its end), comes off over 0.2 s
PRESS_STEPS = 2 * RAMP + HOLD
ROCK_FULL, ROCK_HALF = 0.001, 0.002  # m: rock for 1.0 (under 1 mm) and 0.5 (under 2 mm: less than half the start's 4)
SPAWN_TOL = 0.005  # m: the stool must spawn this close to STOOL_POS (object jitter off)
MOVED = 0.02  # m: the stool's middle moved more than this on the table -> 0
TIPPED_DEG = 10.0
ROBOTS = ("left_robot", "right_robot")
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
LINK_POSES: dict = {}  # "right_robot/gripper" -> (pos, R) in the env frame at this control step (filled in the sim)
_SIM: dict = {}  # "base": the simulator's env at this step; "force": the press applied now (corner, N)


def rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def feet_world(objects) -> np.ndarray:
    s = objects["stool"]
    return np.asarray(s.pos, float) + foot_points() @ rotm(s.quat_xyzw).T


def press_point(objects, corner: int) -> np.ndarray:
    sx, sy = CORNERS[corner]
    s = objects["stool"]
    return np.asarray(s.pos, float) + rotm(s.quat_xyzw) @ np.array([sx * PRESS_IN, sy * PRESS_IN, STOOL_H])


def stool_moved(objects) -> float:
    """How far (m) the stool's middle is from where the scene puts it (a fixed layout: -E object_jitter=0)."""
    p = np.asarray(objects["stool"].pos, float)
    return float(math.hypot(p[0] - STOOL_POS[0], p[1] - STOOL_POS[1]))


def stool_tilt_deg(objects) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, rotm(objects["stool"].quat_xyzw)[2, 2]))))


UNDER_MIN = 13  # of the 49 points: at least a quarter of the foot's bottom over the shim
_FOOT_GRID = np.array([(a, b) for a in np.linspace(-1, 1, 7) for b in np.linspace(-1, 1, 7)]) * (LEG_W / 2)


def shim_under(objects, leg: int) -> bool:
    """A shim lying on the table (tipped under 15 degrees, its bottom within 2 mm of it) with at least a quarter of the
    foot's flat bottom over it (13 of 49 points across the foot's bottom inside the shim's outline, seen from above)."""
    fw = feet_world(objects)[leg]
    pts = np.column_stack([fw[0] + _FOOT_GRID[:, 0], fw[1] + _FOOT_GRID[:, 1]])
    for name in SHIMS:
        s = objects[name]
        R = rotm(s.quat_xyzw)
        p = np.asarray(s.pos, float)
        if R[2, 2] < math.cos(math.radians(15)) or p[2] - TOP > 0.002:
            continue
        loc = (pts - p[:2]) @ R[:2, :2]  # into the shim's frame (it lies flat)
        if np.count_nonzero((np.abs(loc[:, 0]) <= SHIM_L / 2) & (np.abs(loc[:, 1]) <= SHIM_W / 2)) >= UNDER_MIN:
            return True
    return False


def _box_gap(objects) -> float:
    """Smallest distance (m, capped at CLEAR) from any arm link's surface points to the stool's box (in its frame)."""
    s = objects["stool"]
    co, Rc = np.asarray(s.pos, float), rotm(s.quat_xyzw)
    lo = np.array([-FOOT_SPAN / 2, -FOOT_SPAN / 2, -0.01])
    hi = np.array([FOOT_SPAN / 2, FOOT_SPAN / 2, STOOL_H])
    mid, half = (lo + hi) / 2, (hi - lo) / 2
    best = CLEAR
    for key, (o, R) in LINK_POSES.items():
        link = key.split("/")[1]
        _, cen, rad = yg.cloud(link)
        c = (np.asarray(o, float) + np.asarray(R, float) @ cen - co) @ Rc
        if np.linalg.norm(np.maximum(np.abs(c - mid) - half, 0.0)) - rad >= CLEAR:
            continue
        p = (np.asarray(o, float) + yg.points(link) @ np.asarray(R, float).T - co) @ Rc
        d = np.linalg.norm(np.maximum(np.abs(p - mid) - half, 0.0), axis=1)
        best = min(best, float(d.min()))
    return best


_SHIM_CORNERS = np.array([(x, y, z) for x in (-SHIM_L / 2, SHIM_L / 2) for y in (-SHIM_W / 2, SHIM_W / 2)
                          for z in (0.0, SHIM_T1)])
HELD = 0.005  # m: a fingertip this close to a shim holds it (the shim then counts as part of the arm)


def shim_flat(objects, name) -> bool:
    """The shim lying on the table: tipped under 15 degrees, its bottom within 2 mm of the table."""
    s = objects[name]
    return bool(rotm(s.quat_xyzw)[2, 2] >= math.cos(math.radians(15)) and float(s.pos[2]) - TOP <= 0.002)


def shim_near_stool(objects, name) -> bool:
    """Any corner of the shim within CLEAR of the stool's box (the same box as the arms' check)."""
    s, st = objects[name], objects["stool"]
    p = np.asarray(s.pos, float) + _SHIM_CORNERS @ rotm(s.quat_xyzw).T
    q = (p - np.asarray(st.pos, float)) @ rotm(st.quat_xyzw)
    lo = np.array([-FOOT_SPAN / 2, -FOOT_SPAN / 2, -0.01])
    hi = np.array([FOOT_SPAN / 2, FOOT_SPAN / 2, STOOL_H])
    return bool(np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0.0), axis=1).min() < CLEAR)


def shim_held(objects, name) -> bool:
    """Any point of either fingertip of either arm within HELD of the shim's box."""
    s = objects[name]
    co, Rc = np.asarray(s.pos, float), rotm(s.quat_xyzw)
    lo, hi = np.array([-SHIM_L / 2, -SHIM_W / 2, 0.0]), np.array([SHIM_L / 2, SHIM_W / 2, SHIM_T1])
    for key, (o, R) in LINK_POSES.items():
        link = key.split("/")[1]
        if link not in ("tip_left", "tip_right"):
            continue
        _, cen, rad = yg.cloud(link)
        c = (np.asarray(o, float) + np.asarray(R, float) @ cen - co) @ Rc
        if np.linalg.norm(np.maximum(np.maximum(lo - c, c - hi), 0.0)) - rad >= HELD:
            continue
        p = (np.asarray(o, float) + yg.points(link) @ np.asarray(R, float).T - co) @ Rc
        if np.linalg.norm(np.maximum(np.maximum(lo - p, p - hi), 0.0), axis=1).min() < HELD:
            return True
    return False


def arms_clear(objects) -> bool:
    """Every arm link at least CLEAR from the stool's box, and no shim near the stool that is held by a fingertip or
    not lying flat on the table (a held or propped shim is a prop, not a fix)."""
    if _box_gap(objects) < CLEAR - 1e-9:
        return False
    for name in SHIMS:
        if shim_near_stool(objects, name) and (shim_held(objects, name) or not shim_flat(objects, name)):
            return False
    return True


class Ledger:
    """One trial: the hands-off clock, the scorer's corner presses and the result of the last full set of presses."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.clear = 0  # control steps with both arms clear
        self.engaged = False  # the robot has worked on the stool (an arm within CLEAR of it, or a shim near it)
        self.checked = False  # the stool's spawn position has been checked
        self.i = None  # step within the presses, or None
        self.feet = []  # each press's foot heights (4,) read at the end of its hold
        self.result = None  # (tier, rock m) of the last full set of presses, kept while the arms stay clear
        self.force = None  # (corner, N) to apply now

    def step(self, objects, clear: bool) -> None:
        self.force = None
        if not clear or any(shim_near_stool(objects, name) for name in SHIMS):
            self.engaged = True
        if not clear:
            self.clear, self.i, self.result, self.feet = 0, None, None, []
            return
        self.clear += 1
        s = objects["stool"]
        still = (np.linalg.norm(np.asarray(s.lin_vel, float)) < STILL_V
                 and np.linalg.norm(np.asarray(s.ang_vel, float)) < STILL_W)
        if (self.i is None and self.result is None and self.engaged and self.clear >= round(CLEAR_S / DT)
                and still):
            self.i, self.feet = 0, []
        if self.i is None:
            return
        k, j = divmod(self.i, PRESS_STEPS)
        if j == RAMP + HOLD - 1:
            self.feet.append(feet_world(objects)[:, 2].copy())
        if j < RAMP:
            f = PRESS_N * (j + 1) / RAMP
        elif j < RAMP + HOLD:
            f = PRESS_N
        else:
            f = PRESS_N * (PRESS_STEPS - 1 - j) / RAMP
        self.force = (k, f) if f > 0 else None
        self.i += 1
        if self.i >= 4 * PRESS_STEPS:
            self.i = None
            z = np.array(self.feet)
            rock = float((z.max(axis=0) - z.min(axis=0)).max())
            self.result = (tier(objects, rock), rock)

    def score(self) -> float:
        return self.result[0] if self.result is not None else 0.0


def tier(objects, rock: float) -> float:
    if stool_moved(objects) > MOVED or stool_tilt_deg(objects) > TIPPED_DEG:
        return 0.0
    if rock < ROCK_FULL:
        return 1.0
    if rock < ROCK_HALF:
        return 0.5
    rocking = (SHORT, partner(SHORT))
    others = ((SHORT + 1) % 4, (SHORT + 3) % 4)
    if any(shim_under(objects, i) for i in rocking) and not any(shim_under(objects, i) for i in others):
        return 0.25
    return 0.0


_L = None


def _read_links(base) -> None:
    """Every arm link's pose (env frame) at this control step, into LINK_POSES. A missing arm or link stops the trial:
    the hands-off check would otherwise pass silently."""
    _SIM["base"] = base
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"stool: no {robot} in the scene for the hands-off check") from e
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"stool: {robot} has no link(s) {missing} for the hands-off check")
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
    if gripkit._STATE.get("bound", 0) < 0:
        raise RuntimeError("stool: the finger pads' rubber material could not be bound")
    if not _READ_ONCE:
        _READ_ONCE.append(1)
        print(f"stool: short leg {SHORT} ({LEG_NAMES[SHORT]}); hands-off check on {len(LINK_POSES)} arm links; "
              f"pad shapes with rubber: {gripkit._STATE.get('bound')}", file=sys.stderr, flush=True)


_READ_ONCE: list = []
kit.STATE_HOOKS.append(_read_links)
if "isaaclab" in sys.modules:  # in the simulator: build the arm links' points as the task loads
    yg.warm(ARM_LINKS)


def _apply(objects, force) -> None:
    """Put the scorer's press on the stool in the simulator (a force straight down at a seat corner), or take it
    off. Offline (no simulator) it does nothing."""
    base = _SIM.get("base")
    if base is None or _SIM.get("force") == force:
        return
    import torch

    asset = base.scene["stool"]
    dev = asset.device
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    if force is None:
        f, p = [0.0, 0.0, 0.0], press_point(objects, 0)
    else:
        f, p = [0.0, 0.0, -float(force[1])], press_point(objects, force[0])
    F = torch.tensor([[f]], dtype=torch.float32, device=dev)
    P = torch.tensor([[[float(v) for v in p + origin]]], dtype=torch.float32, device=dev)
    asset.set_external_force_and_torque(F, torch.zeros_like(F), positions=P, body_ids=[0],
                                        env_ids=torch.tensor([0], device=dev), is_global=True)
    _SIM["force"] = force


def _trace(objects, n) -> None:
    """STOOL_PROBE=<file>: every 5th control step, the feet's heights (mm over the table), both shims' middles and
    tilt, and both tool points, for debugging probe runs."""
    path = os.environ.get("STOOL_PROBE")
    if not path or n is None or n % 5:
        return
    fz = (feet_world(objects)[:, 2] - TOP) * 1000
    parts = [f"{n}", "feet " + " ".join(f"{v:.2f}" for v in fz),
             f"stool moved {stool_moved(objects) * 1000:.1f}mm tilt {stool_tilt_deg(objects):.2f}"]
    for name in SHIMS:
        s = objects[name]
        p = np.asarray(s.pos, float)
        parts.append(f"{name} {p[0]:.4f} {p[1]:.4f} {(p[2] - TOP) * 1000:.2f}mm tilt "
                     f"{math.degrees(math.acos(min(1.0, rotm(s.quat_xyzw)[2, 2]))):.1f}")
    for g in ("left_gripper", "right_gripper"):
        if g in objects:
            p = np.asarray(objects[g].pos, float)
            parts.append(f"{g} {p[0]:.4f} {p[1]:.4f} {(p[2] - TOP) * 1000:.1f}mm open {kit.opening(objects, g):.2f}")
    parts.append(f"clear {_L.clear}")
    with open(path, "a") as f:
        f.write(" | ".join(parts) + "\n")


def _update(objects):
    global _L
    if _L is None or _L.episode != kit.episode():
        _L = Ledger()
        _SIM["force"] = "reset"  # forces any leftover press off on the first step of a trial
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else ("obj", id(objects))
    if key == _L.key:
        return _L
    _L.key = key
    if not _L.checked and n is not None and n >= 5:
        _L.checked = True
        if stool_moved(objects) > SPAWN_TOL:  # nothing touches the stool in the first 0.25 s
            raise RuntimeError(f"stool: spawned {stool_moved(objects) * 1000:.0f} mm from its fixed start; the moved "
                               "check needs a fixed layout: run with -E object_jitter=0")
    _L.step(objects, arms_clear(objects))
    _apply(objects, _L.force)
    _trace(objects, n)
    if _L.result is not None and _L.i is None and not getattr(_L, "_said", False):
        _L._said = True
        print(f"stool: presses done, rock {_L.result[1] * 1000:.2f} mm, moved {stool_moved(objects) * 1000:.1f} mm, "
              f"tier {_L.result[0]}", file=sys.stderr,
              flush=True)
    if _L.result is None:
        _L._said = False
    return _L


def success(objects):
    return _update(objects).score() >= 1.0


def score(objects):
    """1.0: after both arms have been 2 cm clear of the stool for 2 s, the scorer's 20 N press on each seat corner in
    turn moves no foot 1 mm or more, and the stool is where it started (within 2 cm, upright). 0.5: no foot moves 2 mm.
    0.25: a shim lies under a foot of the rocking pair (none under the other two). 0 otherwise, and 0 while an arm is
    within 2 cm of the stool or the presses have not finished."""
    return _update(objects).score()


# -- demo -------------------------------------------------------------------------------------------------------------
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open
TIP_BELOW_TCP = 0.013  # m: the pad tips past the tool point
YAW_X = math.pi / 2  # fingers close along x (across a shim lying along y)
PICK_U = 0.043  # m from the thin end: where the fingers hold the shim (5.7 mm thick there, 5.8 mm from its thick end)
SNUG_T = 0.0045  # m: aimed shim thickness under the foot's outer edge (gap 4 mm; the hand stops a few mm short)
PRESS_DEPTH = 0.008  # m: a press goes this far past the seat corner's level height
PRESS_PITCH = -0.9  # rad: the pressing hand points down and 52 degrees forward


def finger_for(half: float) -> float:
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * min(FINGER_HALF_OPEN, max(0.0, half)) / FINGER_HALF_OPEN


def _tw(duration, pos, finger, label, yaw=0.0, pitch=0.0, roll=0.0):
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label, yaw=float(yaw),
                      pitch=float(pitch), roll=float(roll))


def _t(seq):
    return sum(w.duration for w in seq)


def _wait(seq, until, label="wait"):
    d = until - _t(seq)
    if d > 1e-6:
        w = seq[-1]
        if isinstance(w, kit.Home):
            seq.append(kit.Home(float(d), label=label))
        else:
            seq.append(kit.Tilted(float(d), w.pos, w.finger, label, yaw=w.yaw, roll=w.roll, pitch=w.pitch))


DEMO_PRESS_IN = 0.095  # m: the demo presses 9.5 cm from the middle along x and y (well inside the seat's edges)


def _seat_corner_level(corner: int) -> np.ndarray:
    sx, sy = CORNERS[corner]
    return np.array([STOOL_XY[0] + sx * DEMO_PRESS_IN, STOOL_XY[1] + sy * DEMO_PRESS_IN, TOP + STOOL_H])


def _press(corner: int, down: float) -> list:
    """Closed fingers onto the seat corner (pointing down and forward: the seat is too high for a straight-down hand
    near the robot), push `down` past its level height, and back up."""
    p = _seat_corner_level(corner)
    shut = finger_for(0.0)
    tip = TIP_BELOW_TCP * np.array([math.sin(-PRESS_PITCH), 0.0, -math.cos(PRESS_PITCH)])  # tool point -> pad tips
    over = p - tip + (0, 0, 0.04)
    at = p - tip - (0, 0, down)
    return [_tw(3.0, over, shut, f"over the {LEG_NAMES[corner]} corner", pitch=PRESS_PITCH),
            _tw(1.5, at, shut, f"press the {LEG_NAMES[corner]} corner down", pitch=PRESS_PITCH),
            _tw(1.0, at, shut, "hold: watch which foot lifts", pitch=PRESS_PITCH),
            _tw(1.0, over, shut, "let go", pitch=PRESS_PITCH)]


def shim_target(leg: int, t: float = SNUG_T, side=None):
    """The shim's middle (world xy) pushed under `leg` from the stool's side (along y) until its thickness under the
    foot's outer edge is t."""
    sx, sy = CORNERS[leg]
    fx, fy = STOOL_XY[0] + sx * FOOT_C, STOOL_XY[1] + sy * FOOT_C
    edge = fy + sy * LEG_W / 2  # the foot's outer face
    u = (t - SHIM_T0) / SHIM_SLOPE  # how far the thin end goes past it
    thin = edge - sy * u
    return np.array([fx, thin + sy * SHIM_L / 2])


SHIM_LEAN = 0.5  # rad: the hand leans out from the stool (fingers down and in), so the gripper and wrist clear the
# seat's edge while the fingertips work under it


def _shim_moves(leg: int, t: float, deep: float = 0.0) -> list:
    """Pick up the shim on the leg's side by its thick end, carry it to the foot, set it on the table 1 cm outside
    the foot and push it in along y until its thickness under the foot's edge is t. deep > 0: then let go, close the
    fingers behind its thick end and push it `deep` m further. Positions below are of the pad tips' middle."""
    sx, sy = CORNERS[leg]
    roll = -sy * SHIM_LEAN
    tip = TIP_BELOW_TCP * np.array([0.0, -sy * math.sin(SHIM_LEAN), -math.cos(SHIM_LEAN)])  # tool point -> tips
    s0 = np.array([STOOL_XY[0], sy * SHIM_Y])
    grip = sy * (PICK_U - SHIM_L / 2)  # the grip point from the shim's middle, along y
    z = TOP + 0.004  # the pad tips' middle 4 mm over the table (leaning, their low edge about 0.3 mm over it)
    up = z + 0.03
    open_ = finger_for(0.025)
    hold = finger_for(SHIM_W / 2 - 0.004)
    goal = shim_target(leg, t)
    outside = np.array([goal[0], STOOL_XY[1] + sy * (FOOT_C + LEG_W / 2 + 0.010 + SHIM_L / 2)])

    def w(d, c, dy, zz, f, label):
        return _tw(d, (c[0], c[1] + dy - tip[1], zz - tip[2]), f, label, YAW_X, roll=roll)

    out = [w(3.0, s0, grip, up, open_, "over the shim"),
           w(1.5, s0, grip, z, open_, "fingers either side of its thick end"),
           w(1.0, s0, grip, z, hold, "pinch it"),
           w(1.0, s0, grip, z + 0.012, hold, "lift it"),
           w(3.0, outside, grip, z + 0.012, hold, "carry it beside the hanging foot, thin end in"),
           w(1.0, outside, grip, z + 0.0005, hold, "set it on the table"),
           w(4.0, goal, grip, z + 0.0005, hold, "push it under the foot until snug"),
           w(1.0, goal, grip, z + 0.0005, open_, "let go"),
           w(1.5, goal, grip, up + 0.03, open_, "lift the fingers")]
    if deep > 0:
        shut = finger_for(0.0)
        back = sy * (SHIM_L / 2 + 0.010)  # closed fingers' middle 10 mm behind the thick end
        out += [w(1.0, goal, back, up, shut, "close the fingers behind the shim"),
                w(1.5, goal, back, z, shut, "down behind its thick end"),
                w(3.0, goal, back - sy * (deep + 0.002), z, shut, "push it further in"),
                w(1.5, goal, back - sy * (deep + 0.002), up + 0.03, shut, "lift")]
    return out


def demo():
    """Press the corner over the short leg's diagonal partner (nothing moves: that foot is down), then the corner over
    the short leg (the stool tips onto the short foot and the partner's foot lifts; let go, it rocks back), then the
    same arm pushes its shim under the short foot, which hangs, until snug; both arms go home and wait for the
    scorer's presses. STOOL_VARIANT=nothing: both arms stay put; pressonly: the presses, no shim; wrong: the shim goes
    under the other foot on that side, which always bears; deep: the shim is pushed 18 mm past snug (about 6.6 mm under
    the foot: over-shimmed); hold: the shim as in the demo, then the arm keeps a hand on the seat to the end."""
    variant = os.environ.get("STOOL_VARIANT", "")
    if variant == "nothing":
        return {"left": [kit.Home(20.0, label="stay at the ready pose")],
                "right": [kit.Home(20.0, label="stay at the ready pose")]}
    s, q = SHORT, partner(SHORT)
    side = lambda leg: "left" if CORNERS[leg][1] > 0 else "right"  # noqa: E731
    arms = {"left": [kit.Home(0.5, label="start")], "right": [kit.Home(0.5, label="start")]}
    arms[side(q)] += _press(q, PRESS_DEPTH)
    arms[side(q)].append(kit.Home(4.5, label="back to the ready pose"))
    _wait(arms[side(s)], _t(arms[side(q)]) - 4.5)
    arms[side(s)] += _press(s, PRESS_DEPTH + SHORT_BY * DEMO_PRESS_IN / FOOT_C)
    leg, deep = s, 0.0
    if variant == "pressonly":
        leg = None
    elif variant == "wrong":
        leg = (s + 1) % 4 if side((s + 1) % 4) == side(s) else (s + 3) % 4
    elif variant == "deep":
        deep = 0.018  # 18 mm further: about 6.6 mm under the foot's edge, 2.6 mm more than the gap
    if leg is not None:
        arms[side(s)] += _shim_moves(leg, SNUG_T, deep)
    if variant == "hold":
        arms[side(s)] += _press(s, 0.0)[:2]
        arms[side(s)].append(_tw(40.0, arms[side(s)][-1].pos, finger_for(0.0), "keep a hand on the seat",
                                 pitch=PRESS_PITCH))
    else:
        arms[side(s)].append(kit.Home(4.5, label="back to the ready pose"))
    end = max(_t(arms["left"]), _t(arms["right"])) + 9.0
    for k in arms:
        _wait(arms[k], end, "hands off: the scorer presses each corner")
    return arms
