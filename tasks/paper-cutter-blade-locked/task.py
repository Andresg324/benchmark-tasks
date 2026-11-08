"""A tabletop guillotine paper cutter on the robot's right. Its blade arm is hinged at the back corner and held raised by
the counterbalance spring that real cutters have. A stack of trimmed cards still lies on the cutting bed under the
blade line, and an empty out-tray stands on the robot's left. To put the cutter away, the cards come off the bed and the
blade arm goes all the way down until its safety latch catches; an arm let go before that springs back up.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props and the hinged cutter; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Put the paper cutter away."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# The cutter (fixed): a cutting bed with a hinged blade arm along its left edge (seen from the robot).
BASE_X, BASE_Y = 0.32, -0.12
BED = (0.22, 0.26, 0.02)  # x by y by z
BED_TOP = TOP + BED[2]
ARM_Y = -0.03  # the blade line
HINGE = (BASE_X + 0.10, ARM_Y, BED_TOP + 0.006)  # the pivot pin at the back corner
ARM_L, ARM_W, ARM_T = 0.20, 0.022, 0.012
HANDLE = (-0.19, 0.0, 0.016)  # the handle's center in the arm's frame (arm lowered: along -x from the pin)
HANDLE_R, HANDLE_W = 0.013, 0.07
RAISED = 0.96  # rad: where the counterbalance spring holds the arm (55 degrees)
LATCH_AT, LATCH_ZONE = 0.0, 0.03  # the catch: within 0.03 rad (1.7 degrees) of fully down

# The cards (free) on the bed under the blade line, and the out-tray (fixed) on the left.
CARDS = (0.07, 0.07, 0.025)
CARDS_START = (0.255, -0.04, BED_TOP + CARDS[2] / 2)  # front of the bed, under the blade line
TRAY_X, TRAY_Y, TRAY_IN, TRAY_H, TRAY_WALL = 0.28, 0.16, (0.12, 0.14), 0.02, 0.005

# Scoring
STILL = 0.02  # rad/s: the arm at rest
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving

STEEL = kit.Look((0.78, 0.79, 0.82), 0.25, metallic=1.0)
EDGE = kit.Look((0.92, 0.93, 0.95), 0.15, metallic=1.0)
BED_LOOK = kit.Look((0.86, 0.85, 0.80), 0.7)
GRID = kit.Look((0.20, 0.35, 0.65), 0.6)
RULE = kit.Look((0.55, 0.56, 0.58), 0.3, metallic=0.8)
HANDLE_LOOK = kit.Look((0.08, 0.08, 0.09), 0.5)
PIVOT = kit.Look((0.25, 0.26, 0.28), 0.4, metallic=0.6)
LATCH_LOOK = kit.Look((0.85, 0.15, 0.10), 0.5)
PAPER = kit.Look((0.97, 0.97, 0.95), 0.9)
PAPER_EDGE = kit.Look((0.83, 0.83, 0.80), 0.9)
TRAY_LOOK = kit.Look((0.18, 0.19, 0.22), 0.6)


def _cutter():
    """Bed with a printed grid and a metal rule at the back; the arm is a steel bar with a bevelled blade on its edge,
    a black handle at the front and a red safety catch at the front of the bed, where the arm comes to rest."""
    bx, by, bz = BED
    ay = ARM_Y - BASE_Y  # the blade line in the bed's frame
    meshes = [kit.box((0, 0, 0), BED, BED_LOOK)]
    for k in range(1, 6):  # grid lines across and along the bed
        meshes.append(kit.box((-bx / 2 + k * bx / 6, 0, bz / 2 + 0.0004), (0.0012, by - 0.01, 0.0008), GRID))
        meshes.append(kit.box((0, -by / 2 + k * by / 6, bz / 2 + 0.0004), (bx - 0.01, 0.0012, 0.0008), GRID))
    meshes.append(kit.box((bx / 2 - 0.008, 0, bz / 2 + 0.004), (0.014, by, 0.008), RULE))  # back rule
    meshes.append(kit.box((HINGE[0] - BASE_X, ay, bz / 2 + 0.010), (0.03, 0.03, 0.02), PIVOT))  # pivot block
    meshes.append(kit.box((-bx / 2 + 0.012, ay + 0.022, bz / 2 + 0.008), (0.016, 0.012, 0.016), LATCH_LOOK))  # catch
    colliders = [kit.Collider((0, 0, 0), size=BED),
                 kit.Collider((bx / 2 - 0.008, 0, bz / 2 + 0.004), size=(0.014, by, 0.008)),
                 kit.Collider((-bx / 2 + 0.012, ay + 0.022, bz / 2 + 0.008), size=(0.016, 0.012, 0.016))]
    hx, hy, hz = HANDLE
    arm_meshes = (
        kit.box((-ARM_L / 2, 0, 0), (ARM_L, ARM_W, ARM_T), STEEL),
        kit.box((-ARM_L / 2 + 0.01, -ARM_W / 2 - 0.0015, -0.002), (ARM_L - 0.02, 0.003, ARM_T - 0.004), EDGE),
        kit.cylinder((hx, hy, hz), HANDLE_R, HANDLE_W, HANDLE_LOOK, axis="y"),
        kit.cylinder((0, 0, 0), 0.006, 0.034, PIVOT, axis="y"),
    )
    arm_colliders = (kit.Collider((-ARM_L / 2, 0, 0), size=(ARM_L, ARM_W, ARM_T)),
                     kit.Collider((hx, hy, hz), size=(2 * HANDLE_R, HANDLE_W, 2 * HANDLE_R)))
    arm = kit.Link("arm", "revolute", "Y", (HINGE[0] - BASE_X, ay, HINGE[2] - (TOP + bz / 2)), lower=0.0, upper=RAISED,
                   start=RAISED, mass=0.15, meshes=arm_meshes, colliders=arm_colliders, spring=1.2, rest=1.3, floating=True,
                   damping=0.08, latch=kit.Latch(LATCH_AT, LATCH_ZONE, hold=60.0))
    return kit.Mechanism("cutter", pos=(BASE_X, BASE_Y, TOP + bz / 2), size=(bx, by, 0.05), meshes=tuple(meshes),
                         colliders=tuple(colliders), links=(arm,))


def _cards():
    """A stack of trimmed cards: white faces with a slightly darker band of edges showing the layers."""
    x, y, z = CARDS
    meshes = [kit.box((0, 0, 0), CARDS, PAPER)]
    for k in range(1, 5):
        meshes.append(kit.box((0, 0, -z / 2 + k * z / 5), (x + 0.0006, y + 0.0006, 0.0006), PAPER_EDGE))
    return kit.Prop("cards", pos=CARDS_START, size=CARDS, mass=0.06, meshes=tuple(meshes),
                    colliders=(kit.Collider((0, 0, 0), size=CARDS),))


def _tray():
    ix, iy = TRAY_IN
    w, h = TRAY_WALL, TRAY_H
    parts = [((0, 0, -h / 2 + 0.002), (ix + 2 * w, iy + 2 * w, 0.004)),
             ((ix / 2 + w / 2, 0, 0), (w, iy + 2 * w, h)), ((-ix / 2 - w / 2, 0, 0), (w, iy + 2 * w, h)),
             ((0, iy / 2 + w / 2, 0), (ix, w, h)), ((0, -iy / 2 - w / 2, 0), (ix, w, h))]
    return kit.Prop("tray", pos=(TRAY_X, TRAY_Y, TOP + h / 2), size=(ix + 2 * w, iy + 2 * w, h),
                    meshes=tuple(kit.box(c, s, TRAY_LOOK) for c, s in parts),
                    colliders=tuple(kit.Collider(c, size=s) for c, s in parts))


OBJECTS = [_cutter(), _tray(), _cards()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
GRIPPERS = ("left_gripper", "right_gripper")


class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.home = None  # each gripper's tool point at the trial's first scored step
        self.last = None
        self.still = False


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


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still."""
    if _H.home is None or not _H.still:
        return False
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - _H.home[g]) < HOME_TOL for g in GRIPPERS)


# --- Scoring. The job is scored at the end; the latch itself remembers that it caught.
def _on_bed(s):
    """The cards' center over the cutting bed (they still block the blade or are in the way)."""
    return bool(abs(s.pos[0] - BASE_X) < BED[0] / 2 and abs(s.pos[1] - BASE_Y) < BED[1] / 2
                and s.pos[2] < BED_TOP + 0.05)


REST_TOL = 0.012  # the cards' underside within this of the table top or the tray floor counts as lying there
TRAY_FLOOR = 0.004  # the tray floor's top, above the table


def _held(objects):
    """A gripper closed (opening below 0.6) with its tool point within 5 cm of the cards' center."""
    c = np.asarray(objects["cards"].pos)
    return any(g in objects and kit.opening(objects, g) < 0.6 and np.linalg.norm(np.asarray(objects[g].pos) - c) < 0.05
               for g in ("left_gripper", "right_gripper"))


def _set_aside(objects):
    """The cards off the bed, lying flat on the table or on the tray floor (underside within 1.2 cm of that surface),
    at rest and let go."""
    s = objects["cards"]
    x, y = s.pos[0], s.pos[1]
    flat = 1 - 2 * (s.quat_xyzw[0] ** 2 + s.quat_xyzw[1] ** 2) > math.cos(math.radians(20))
    bottom = s.pos[2] - CARDS[2] / 2
    lying = abs(bottom - TOP) < REST_TOL or abs(bottom - (TOP + TRAY_FLOOR)) < REST_TOL
    return bool(not _on_bed(s) and lying and flat and s.resting() and not _held(objects)
                and abs(x) < 0.6 and abs(y) < 0.6)


def _rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _job(objects):
    j = kit.joint(objects, "cutter/arm")
    if not _set_aside(objects):
        return 0.0
    if j.latched and abs(j.vel) < STILL:
        return 1.0
    return 0.25


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """The job done (cards set aside, arm latched) and, with REQUIRE_ARMS_HOME, both arms home and still. Called first
    each step, so it also records the history."""
    _update(objects)
    return _job(objects) == 1.0 and (_arms_home(objects) or not REQUIRE_ARMS_HOME)


def score(objects):
    """1.0: the cards off the cutting bed, lying flat on the table or the tray floor and let go, and the blade arm down
    with its safety latch caught. 0.25: the cards set aside but the arm not latched (raised, or let go before it caught and sprung back up).
    0: the cards still on the bed, whatever the arm does (they block the blade's way down). With REQUIRE_ARMS_HOME, a
    finished job with the arms not back home and still scores 0.9 instead of 1.0. Possible: 0, 0.25, 0.9, 1."""
    return _score(objects)


def demo():
    """Left arm: lift the cards off the front of the bed, draw them toward the robot (away from the raised handle) and
    set them in the tray. Right arm, after that: grip the blade arm's handle and swing it down along its arc until the
    latch catches, let go and back away. Both arms end back in their starting pose."""
    travel = TOP + 0.22
    lag = 0.012
    # Left arm: cards from the bed straight to the tray (the bed's far left part is in the left arm's reach).
    grip_z = CARDS_START[2] + CARDS[2] / 2 - 0.012 + 0.013
    cx, cy = CARDS_START[0], CARDS_START[1] - lag  # the left arm falls short toward its base (+y)
    place_z = TOP + 0.004 + CARDS[2] / 2 + 0.003
    left = [Waypoint(1.8, (cx - 0.03, cy, travel - 0.06), FINGER_OPEN, "over the cards, in front of the handle"),
            Waypoint(1.2, (cx, cy, grip_z + 0.05), FINGER_OPEN, "above the cards"),
            Waypoint(1.0, (cx, cy, grip_z), FINGER_OPEN, "descend"),
            Waypoint(0.8, (cx, cy, grip_z), FINGER_CLOSED, "grip"),
            Waypoint(0.8, (cx, cy, grip_z + 0.03), FINGER_CLOSED, "lift clear of the bed"),
            Waypoint(1.0, (cx - 0.05, cy + 0.04, grip_z + 0.05), FINGER_CLOSED, "draw toward the robot, away from the handle"),
            Waypoint(1.4, (TRAY_X, TRAY_Y, travel - 0.04), FINGER_CLOSED, "carry"),
            Waypoint(1.0, (TRAY_X, TRAY_Y, place_z), FINGER_CLOSED, "lower"),
            Waypoint(0.6, (TRAY_X, TRAY_Y, place_z), FINGER_OPEN, "release"),
            Waypoint(0.8, (TRAY_X, TRAY_Y, travel - 0.04), FINGER_OPEN, "retreat")]
    left += [Waypoint(1.5, (0.20, 0.26, TOP + 0.22), FINGER_OPEN, "out of the way")]
    # Right arm: wait for the cards, then press the arm down along its arc, holding the handle.
    def handle_at(t):
        return np.array(HINGE) + _rot_y(t) @ np.array(HANDLE)

    hold = [Waypoint(sum(w.duration for w in left[:-1]) + 0.5, (0.18, -0.26, TOP + 0.20), FINGER_OPEN, "wait")]
    yaw = np.pi / 2  # fingers close along x, across the handle
    start = handle_at(RAISED)
    path = [Waypoint(1.5, (start[0], start[1], start[2] + 0.06), FINGER_OPEN, "over the handle", yaw),
            Waypoint(1.0, tuple(start), FINGER_OPEN, "to the handle", yaw),
            Waypoint(0.8, tuple(start), FINGER_CLOSED, "grip", yaw)]
    for k in range(1, 13):
        t = RAISED * (1 - k / 12) - (0.02 if k == 12 else 0.0)
        p = handle_at(t)
        path.append(Waypoint(0.35, tuple(p), FINGER_CLOSED, "swing down", yaw))
    end = handle_at(-0.02)
    path += [Waypoint(0.8, tuple(end), FINGER_CLOSED, "press home", yaw),
             Waypoint(0.6, tuple(end), FINGER_OPEN, "let go", yaw),
             Waypoint(1.0, (end[0], end[1], end[2] + 0.10), FINGER_OPEN, "back off", yaw),
             kit.Home(2.0), kit.Home(2.0, label="hold at home")]
    pad = sum(w.duration for w in hold + path) - sum(w.duration for w in left) - 4.0
    left += [Waypoint(max(pad, 0.5), (0.20, 0.26, TOP + 0.22), FINGER_OPEN, "wait"), kit.Home(2.0),
             kit.Home(2.0, label="hold at home")]
    return {"left": left, "right": hold + path}
