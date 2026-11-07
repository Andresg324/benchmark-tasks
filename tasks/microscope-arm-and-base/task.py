"""A student compound microscope stands on a grey rubber bench mat to the robot's left; an empty mat of the same kind
lies to the robot's right. The microscope faces the robot's right: its curved arm rises from the back of the base on
the robot's left side, its stage, nosepiece and objectives sit in the middle, and its eyepiece tube leans toward the
front, over the long front of the base. Cream enamel body, black stage, chrome nosepiece and objectives.
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
INSTRUCTION = "Move the microscope to the other bench mat."

# Bench mats.
MAT_X, MAT_LX, MAT_LY, MAT_H = 0.30, 0.15, 0.185, 0.003
START_Y, GOAL_Y = 0.16, -0.04

# The microscope, in its own frame: origin at the middle of the base, +y toward the arm (the back), -y toward the
# front (the eyepiece side), so it faces the robot's right.
BASE_LX, BASE_Y0, BASE_Y1, BASE_H = 0.07, -0.10, 0.075, 0.026
ARM_W, ARM_Y0, ARM_Y1, ARM_TOP = 0.030, 0.045, 0.075, 0.140  # the arm's column, from the base up to ARM_TOP
HEAD_Y0, HEAD_Y1, HEAD_Z0, HEAD_Z1 = -0.018, 0.038, 0.115, 0.150
STAGE_Y, STAGE_Z, STAGE = -0.005, 0.075, (0.070, 0.065, 0.006)
TUBE_LEAN = math.radians(20)
MASS = 0.30
SCOPE_START = (MAT_X, START_Y - (BASE_Y0 + BASE_Y1) / 2, TOP + MAT_H + BASE_H / 2)  # base centered on the mat

# The finger pads close along the gripper's own x axis, each starting FINGER_REACH from the tool point. The scorer
# checks what lies on that closing line, at the height of the pads.
FINGER_REACH = 0.047
PAD_DOWN = 0.006  # the line's height: this far below the tool point, toward the pad tips
HOLDING = 0.97  # a gripper this far open or less, near the microscope, is holding it
LIFTED = 0.01  # the base this far above its resting height
VIOLATION_STEPS = 3
DRAG_LIMIT = 0.04  # cm of travel along the bench without lifting
TIP_LIMIT = math.radians(25)
ON_MAT = 0.03
MAX_TURN = math.radians(10)  # the base's yaw at the end, against how it stood on the first mat
CLEAR = 0.06  # how far both tool points must be from the microscope for the run to count as done

# -- looks and models -------------------------------------------------------------------------

ENAMEL = kit.Look((0.90, 0.88, 0.82), 0.35)
ENAMEL_SHADE = kit.Look((0.82, 0.80, 0.74), 0.4)
BLACK = kit.Look((0.06, 0.06, 0.07), 0.45)
CHROME = kit.Look((0.86, 0.87, 0.90), 0.15, metallic=1.0)
GLASS = kit.Look((0.75, 0.85, 0.95), 0.05, opacity=0.6)
RUBBER = kit.Look((0.10, 0.10, 0.10), 0.9)
MAT = kit.Look((0.30, 0.32, 0.34), 0.95)
OBJ_COLORS = (kit.Look((0.85, 0.10, 0.10), 0.3), kit.Look((0.95, 0.80, 0.10), 0.3), kit.Look((0.15, 0.35, 0.85), 0.3))


def _microscope():
    zb = BASE_H / 2  # the base's top
    ly = BASE_Y1 - BASE_Y0
    yc = (BASE_Y1 + BASE_Y0) / 2
    m, c = [], []

    def col(part, center, size=None, radius=0.0, height=0.0):
        c.append(kit.Collider(tuple(center), size=size, radius=radius, height=height))
        half = np.array(size) / 2 if size is not None else np.array([radius, radius, height / 2])
        PARTS.setdefault(part, []).append((np.asarray(center) - half, np.asarray(center) + half))

    # Base, with a bevelled top, rubber feet and the illuminator.
    m += [kit.box((0, yc, -0.002), (BASE_LX, ly, BASE_H - 0.004), ENAMEL),
          kit.box((0, yc, zb - 0.002), (BASE_LX - 0.006, ly - 0.006, 0.004), ENAMEL_SHADE)]
    for fx in (-0.025, 0.025):
        for fy in (BASE_Y0 + 0.012, BASE_Y1 - 0.012):
            m.append(kit.cylinder((fx, fy, -zb + 0.001), 0.006, 0.002, RUBBER))
    m += [kit.lathe([(0.016, zb), (0.016, zb + 0.004), (0.013, zb + 0.008), (0.0, zb + 0.009)], ENAMEL_SHADE,
                    center=(0, STAGE_Y, 0)),
          kit.cylinder((0, STAGE_Y, zb + 0.0092), 0.009, 0.001, GLASS)]
    col("base", (0, yc, 0), size=(BASE_LX, ly, BASE_H))
    col("illuminator", (0, STAGE_Y, zb + 0.0045), radius=0.016, height=0.009)
    # The arm: a curved column rising from the back of the base, its back bowed out to make the handle.
    ay = (ARM_Y0 + ARM_Y1) / 2
    path = [(0, ay, zb - 0.002), (0, ay + 0.002, 0.04), (0, ay + 0.006, 0.08), (0, ay + 0.005, 0.11),
            (0, ay, 0.13), (0, ay - 0.004, ARM_TOP)]
    m.append(kit.band(path, ARM_W, ARM_Y1 - ARM_Y0, ENAMEL, normal=(1, 0, 0)))
    col("arm", (0, ay, (zb + ARM_TOP) / 2), size=(ARM_W, ARM_Y1 - ARM_Y0, ARM_TOP - zb))
    # Coarse and fine focus knobs on both sides of the arm.
    for sx in (-1, 1):
        m.append(kit.cylinder((sx * (ARM_W / 2 + 0.006), ay, 0.082), 0.013, 0.010, BLACK, axis="x"))
        m.append(kit.cylinder((sx * (ARM_W / 2 + 0.014), ay, 0.082), 0.007, 0.006, BLACK, axis="x"))
    col("focus knobs", (0, ay, 0.082), size=(ARM_W + 0.034, 0.026, 0.026))
    # Stage on its bracket, with two spring clips.
    m += [kit.box((0, STAGE_Y, STAGE_Z), STAGE, BLACK),
          kit.box((0, (STAGE_Y + STAGE[1] / 2 + ARM_Y0) / 2, STAGE_Z - 0.004),
                  (0.024, ARM_Y0 - STAGE_Y - STAGE[1] / 2 + 0.002, 0.012), ENAMEL_SHADE)]
    for sx in (-1, 1):
        m.append(kit.band([(sx * 0.022, STAGE_Y + 0.02, STAGE_Z + 0.004), (sx * 0.020, STAGE_Y, STAGE_Z + 0.0045),
                           (sx * 0.018, STAGE_Y - 0.016, STAGE_Z + 0.0035)], 0.004, 0.001, CHROME,
                          normal=(1, 0, 0)))
    m.append(kit.box((0, STAGE_Y, STAGE_Z + 0.0032), (0.025, 0.075, 0.001), GLASS))  # a slide under the clips
    m.append(kit.box((0, STAGE_Y, STAGE_Z + 0.0038), (0.008, 0.008, 0.0004), OBJ_COLORS[0]))
    col("stage", (0, STAGE_Y, STAGE_Z), size=STAGE)
    col("stage", (0, (STAGE_Y + STAGE[1] / 2 + ARM_Y0) / 2, STAGE_Z - 0.004),
        size=(0.024, ARM_Y0 - STAGE_Y - STAGE[1] / 2, 0.012))
    # Head, nosepiece and three objectives with colored bands.
    hy, hz = (HEAD_Y0 + HEAD_Y1) / 2, (HEAD_Z0 + HEAD_Z1) / 2
    m.append(kit.box((0, hy, hz), (0.042, HEAD_Y1 - HEAD_Y0, HEAD_Z1 - HEAD_Z0), ENAMEL))
    col("head", (0, hy, hz), size=(0.042, HEAD_Y1 - HEAD_Y0, HEAD_Z1 - HEAD_Z0))
    nz = HEAD_Z0 - 0.004
    m.append(kit.lathe([(0.0, nz - 0.004), (0.019, nz - 0.003), (0.021, nz + 0.003), (0.0, nz + 0.004)], CHROME,
                       center=(0, STAGE_Y + 0.004, 0)))
    col("nosepiece", (0, STAGE_Y + 0.004, nz), radius=0.021, height=0.008)
    for k, (ox, oy) in enumerate(((0.0, -0.010), (0.010, 0.006), (-0.010, 0.006))):
        length = (0.024, 0.020, 0.016)[k]
        cz = nz - 0.004 - length / 2
        m.append(kit.cylinder((ox, STAGE_Y + 0.004 + oy, cz), 0.0055, length, CHROME))
        m.append(kit.cylinder((ox, STAGE_Y + 0.004 + oy, cz + 0.002), 0.0058, 0.002, OBJ_COLORS[k]))
    col("objectives", (0, STAGE_Y + 0.004, nz - 0.016), size=(0.03, 0.03, 0.022))
    # Body tube leaning toward the front, and the eyepiece.
    lean = np.array([0.0, -math.sin(TUBE_LEAN), math.cos(TUBE_LEAN)])
    t0 = np.array([0.0, 0.006, HEAD_Z1 - 0.004])
    m += [kit.cylinder(t0 + 0.028 * lean, 0.011, 0.056, ENAMEL, axis=lean),
          kit.cylinder(t0 + 0.062 * lean, 0.0095, 0.014, BLACK, axis=lean),
          kit.cylinder(t0 + 0.070 * lean, 0.0085, 0.003, GLASS, axis=lean)]
    for f in (0.2, 0.55, 0.85):  # boxes along the tube for its collider
        p = t0 + f * 0.07 * lean
        col("tube", p, size=(0.022, 0.022, 0.024))
    return kit.Prop("microscope", pos=SCOPE_START, size=(BASE_LX, 0.18, 0.24), mass=MASS, meshes=tuple(m),
                    colliders=tuple(c))


def _mat(name, y):
    return kit.Prop(name, pos=(MAT_X, y, TOP + MAT_H / 2), size=(MAT_LX, MAT_LY, MAT_H),
                    meshes=(kit.box((0, 0, 0), (MAT_LX, MAT_LY, MAT_H), MAT),),
                    colliders=(kit.Collider((0, 0, 0), size=(MAT_LX, MAT_LY, MAT_H)),))


PARTS = {}  # part name: boxes (low, high corners) in the microscope's frame, filled in by _microscope()
OBJECTS = [_mat("start_mat", START_Y), _mat("goal_mat", GOAL_Y), _microscope()]
_START = {"microscope": np.array(SCOPE_START)}


def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _local(scope, p):
    """A world point in the microscope's own frame."""
    return _rot(scope.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(scope.pos, float))


def _inside(p, zone):
    return all(lo <= v <= hi for v, (lo, hi) in zip(p, zone))


def _tilt(s):
    qx, qy = s.quat_xyzw[0], s.quat_xyzw[1]
    return math.acos(max(-1.0, min(1.0, 1 - 2 * (qx * qx + qy * qy))))


def _held_part(objects, side):
    """What a gripper holds of the microscope: None if its fingers are fully open or nothing of the microscope lies
    between them; otherwise 'arm' or 'base' when only that part lies on the line the fingers close along, at the
    height of the pads, and 'other' when any other part (eyepiece, tube, head, nosepiece, objectives, stage, focus
    knobs, illuminator) does."""
    if side not in objects or kit.opening(objects, side) > HOLDING:
        return None
    g, scope = objects[side], objects["microscope"]
    r = _rot(g.quat_xyzw)
    mid = np.asarray(g.pos, float) + r @ np.array([0.0, 0.0, PAD_DOWN])
    line = [_local(scope, mid + r @ np.array([t, 0.0, 0.0])) for t in np.linspace(-FINGER_REACH, FINGER_REACH, 25)]
    hit = {part for part, boxes in PARTS.items() for p in line
           if any(np.all(p >= lo - 0.003) and np.all(p <= hi + 0.003) for lo, hi in boxes)}
    if not hit:
        return None
    if hit - {"arm", "base"}:
        return "other"
    return "arm" if hit == {"arm"} else "base" if hit == {"base"} else "other"


def _holds(objects):
    """The parts held, one entry per gripper that holds the microscope."""
    return [h for h in (_held_part(objects, s) for s in ("left_gripper", "right_gripper")) if h]


def _carried_right(holds):
    return sorted(holds) == ["arm", "base"]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()  # which trial this history belongs to
        self.last = None
        self.bad_run = 0
        self.dragged = 0.0
        self.carried = False  # lifted at least once, the right way
        self.violation = ""  # never forgiven once set
        self.grips = None  # both tool points last step
        self.grips_still = False


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    s = objects["microscope"]
    now = np.asarray(s.pos, float)
    lifted = now[2] > SCOPE_START[2] + LIFTED
    if _H.last is not None and not lifted:
        _H.dragged += float(np.hypot(*(now[:2] - _H.last[:2])))
    _H.last = now
    grips = [np.asarray(objects[k].pos, float) for k in ("left_gripper", "right_gripper") if k in objects]
    _H.grips_still = _H.grips is not None and len(grips) == len(_H.grips) and all(
        np.linalg.norm(a - b) < 0.001 for a, b in zip(grips, _H.grips))
    _H.grips = grips
    if lifted:
        ok = _carried_right(_holds(objects))
        _H.bad_run = 0 if ok else _H.bad_run + 1
        _H.carried |= ok
        if _H.bad_run >= VIOLATION_STEPS:
            _H.violation = _H.violation or "lifted without one hand on the arm and one on the base"
    else:
        _H.bad_run = 0
    if _H.dragged > DRAG_LIMIT:
        _H.violation = _H.violation or "slid along the bench"
    if _tilt(s) > TIP_LIMIT:
        _H.violation = _H.violation or "tipped"


def _score(objects):
    s = objects["microscope"]
    if _H.violation or not _H.carried or _tilt(s) > math.radians(10) or not s.resting() or _holds(objects):
        return 0.0
    down = s.pos[2] < SCOPE_START[2] + LIFTED
    base = np.asarray(s.pos) + _rot(s.quat_xyzw) @ np.array([0.0, (BASE_Y0 + BASE_Y1) / 2, 0.0])  # the base's center
    on_mat = abs(base[0] - MAT_X) < ON_MAT and abs(base[1] - GOAL_Y) < ON_MAT
    if down and on_mat and abs(_yaw(s)) <= MAX_TURN:
        return 1.0
    if down and np.linalg.norm(np.asarray(s.pos) - _START["microscope"]) > 0.05:
        return 0.25  # carried properly, but set down turned, or somewhere other than the other mat
    return 0.0


def _yaw(s):
    """The microscope's turn about the vertical from how it stood at the start (radians, -pi..pi)."""
    x, y, z, w = s.quat_xyzw
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def _clear(objects):
    """Both tool points at least CLEAR from the microscope's bounding box."""
    s = objects["microscope"]
    for side in ("left_gripper", "right_gripper"):
        if side in objects:
            p = np.asarray(objects[side].pos, float)
            gap = np.maximum(np.maximum(np.asarray(s.low) - p, p - np.asarray(s.high)), 0.0)
            if np.linalg.norm(gap) < CLEAR:
                return False
    return True


def success(objects):
    """The microscope stands upright on the other mat, let go, carried there with one gripper on the arm and the
    other on the base, never slid, never tipped, and both grippers have moved at least 6 cm clear of it and stopped.
    Called first each step, so it also records the step's history."""
    _update(objects)
    return _score(objects) == 1.0 and _clear(objects) and _H.grips_still


def score(objects):
    """0 if the microscope was ever off the bench without exactly one holding gripper on its arm and one on its
    base (for 3 steps in a row), slid more than 4 cm along the bench, or tipped past 25 degrees; never forgiven.
    Otherwise, once both grippers have let go, 1.0 standing upright (within 10 degrees) and still with its base
    center within 3 cm of the other mat's center and turned no more than 10 degrees from how it stood on the first
    mat; 0.25 if it was carried properly but set down turned more than that, or set down upright and still more
    than 5 cm from where it started but not on the other mat; else 0. Success also needs both grippers 6 cm clear.
    Possible scores: 0, 0.25, 1."""
    return _score(objects)


def demo():
    """Both arms at once: the left gripper closes across the arm's handle and the right across the front of the
    base, both lift 4 cm together, carry the microscope across to the other mat, set it down and let go."""
    left_y, right_y = ARM_GRIP_Y, BASE_GRIP_Y
    lx, ly = LAG_LEFT
    rx, ry = LAG_RIGHT
    arm_z = SCOPE_START[2] + ARM_TOP - 0.016
    base_z = SCOPE_START[2] - BASE_H / 2 + 0.004 + 0.013
    lift = 0.04
    dy = GOAL_Y - START_Y

    def plan(x, y, z, hover, lag, yaw, fix, park):
        """One arm's moves. `fix` (dx, dyaw) is added at the far mat: carrying the microscope across loads the left
        arm, which then settles a few cm off and turns its wrist, so it is aimed back to keep the base square."""
        ax, ay = x + lag[0], y + lag[1]
        gx, gyaw = ax + fix[0], yaw + fix[1]
        return [Waypoint(2.0, (ax, ay, hover), FINGER_OPEN, "above", yaw),
                Waypoint(1.5, (ax, ay, z), FINGER_OPEN, "descend", yaw),
                Waypoint(1.0, (ax, ay, z), FINGER_CLOSED, "close", yaw),
                Waypoint(0.8, (ax, ay, z), FINGER_CLOSED, "settle", yaw),
                Waypoint(1.5, (ax, ay, z + lift), FINGER_CLOSED, "lift", yaw),
                Waypoint(3.0, (gx, ay + dy, z + lift), FINGER_CLOSED, "carry", gyaw),
                Waypoint(1.0, (gx, ay + dy, z + lift), FINGER_CLOSED, "steady", gyaw),
                Waypoint(1.5, (gx, ay + dy, z + 0.002), FINGER_CLOSED, "set down", gyaw),
                Waypoint(0.8, (gx, ay + dy, z + 0.002), FINGER_OPEN, "let go", gyaw),
                Waypoint(1.2, (gx, ay + dy, hover), FINGER_OPEN, "up", gyaw),
                Waypoint(2.0, park, FINGER_OPEN, "back off", yaw),
                Waypoint(2.5, park, FINGER_OPEN, "hold", yaw)]

    # Both grips close along x. The wrists turn opposite ways so each wrist camera sits on the outer side.
    left = plan(MAT_X, SCOPE_START[1] + left_y, arm_z, TOP + 0.24, (lx, ly), np.pi / 2, FIX_LEFT, (0.18, 0.20, TOP + 0.22))
    right = plan(MAT_X, SCOPE_START[1] + right_y, base_z, TOP + 0.20, (rx, ry), -np.pi / 2, FIX_RIGHT,
                 (0.16, -0.28, TOP + 0.20))
    return {"left": left, "right": right}


ARM_GRIP_Y = (ARM_Y0 + ARM_Y1) / 2  # the handle, in the microscope's frame
BASE_GRIP_Y = BASE_Y0 + 0.020  # the front of the base, ahead of the stage and the eyepiece
LAG_LEFT = (0.0, 0.0)
LAG_RIGHT = (0.0, 0.0)
FIX_LEFT = (-0.0125, 0.21)  # measured: see plan()
FIX_RIGHT = (0.0166, 0.21)
