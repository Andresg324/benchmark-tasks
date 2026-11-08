"""A countertop oven, scaled to the arms, with its door folded down flat and a hot loaf pan on the middle rack, well
inside. The heating elements glow at the top and bottom of the cavity. A cast-iron trivet waits on the counter on the
robot's right. To take a hot pan out, a cook pulls the rack out by its front rail first and lifts the pan from the
pulled-out rack; nobody reaches into a hot oven or drags the pan across the rack toward the door.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props and the sliding rack; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Practice taking the loaf pan out of the oven."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the workspace is clear for
# the next person or job. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# The oven (fixed), in world coordinates: cavity x from FRONT to BACK, y within +/- HALF_W, z (above the table) from
# CAV_FLOOR to CEIL. The door is open, lying flat in front of the opening.
FRONT, BACK, HALF_W = 0.29, 0.47, 0.11
WALL = 0.012
CAV_FLOOR, CEIL, OVEN_H = 0.05, 0.22, 0.25
DOOR_L, DOOR_T = 0.13, 0.015

# The rack (a slide link): wire shelf whose top is RACK_Z above the table, front rail at RAIL_X when pushed in.
RACK_Z = 0.085
RAIL_X = FRONT + 0.004
RACK_L = 0.16
RAIL_T, RAIL_H = 0.010, 0.012  # the front rail stands this thick (x) and tall above the shelf
PULL = 0.13  # how far the rack slides out at its stop
OUT = -0.11  # the rack counts as pulled out at 11 cm or more
IN = -0.01  # and as pushed back in within 1 cm of fully in

# The loaf pan (free), centered on the rack well inside, and the trivet (fixed) on the counter.
PAN = (0.10, 0.06, 0.045)
PAN_START = (0.37, 0.0, TOP + RACK_Z + PAN[2] / 2)
TRIVET_XY, TRIVET_R, TRIVET_H = (0.24, -0.21), 0.06, 0.012

# Scoring
INSIDE = 0.035  # a gripper's tool point deeper than this past the opening, inside the cavity, is reaching in
INSIDE_CLEAR = 0.030  # once in, it counts as out again only when back within 3 cm of the opening (no jitter at the line)
OFF_SPOT = 0.02  # the pan more than 2 cm from its place on the rack (or lifted 1.5 cm) has left its spot
LIFTED = 0.015
VIOLATION_STEPS = 2
HOME_TOL = 0.03  # a gripper is home within 3 cm of where its tool point was at the trial's first scored step
HOME_STILL = 0.001  # and still: its tool point moved less than 1 mm since the last step (2 cm/s)
HOLD_R = 0.05  # a gripper holds the pan when its fingers are closed (opening under 0.6) within 5 cm of the pan's center
HOLD_OPEN = 0.6
TAKEN = 0.01  # the pan counts as taken off the rack once it is 1 cm off its spot while a gripper holds it
FALL_SPEED = 0.5  # m/s down while no gripper holds it: the pan is falling (a 1.3 cm drop already reaches this)
DROP_TILT = 45.0  # degrees

ENAMEL = kit.Look((0.33, 0.35, 0.40), 0.45)  # dark gray-blue porcelain enamel, light enough to see into
STEEL = kit.Look((0.78, 0.79, 0.80), 0.3, metallic=0.7)
BLACK = kit.Look((0.06, 0.06, 0.07), 0.35)
GLASS = kit.Look((0.12, 0.13, 0.15), 0.08, opacity=0.85)
GLOW = kit.Look((1.0, 0.36, 0.08), 0.6)
CHROME = kit.Look((0.86, 0.87, 0.89), 0.15, metallic=1.0)
PAN_LOOK = kit.Look((0.20, 0.20, 0.22), 0.4, metallic=0.6)
CRUST = kit.Look((0.66, 0.40, 0.17), 0.8)
IRON = kit.Look((0.13, 0.12, 0.12), 0.7, metallic=0.3)
KNOB = kit.Look((0.15, 0.15, 0.16), 0.4)

BASE = ((FRONT + BACK) / 2, 0.0, TOP)


def _w(c):
    """A point given in world coordinates (z above the table), in the oven base's frame."""
    return (c[0] - BASE[0], c[1] - BASE[1], c[2])


def _oven():
    mid, depth = (FRONT + BACK) / 2, BACK - FRONT
    ow = HALF_W + WALL
    parts = [  # (world center with z above the table, size, look, collides)
        ((mid + WALL / 2, 0, CAV_FLOOR / 2), (depth + WALL, 2 * ow, CAV_FLOOR), STEEL, True),  # base under the cavity
        ((mid, 0, CAV_FLOOR - 0.001), (depth, 2 * HALF_W, 0.002), ENAMEL, False),  # cavity floor
        ((mid + WALL / 2, HALF_W + WALL / 2, OVEN_H / 2), (depth + WALL, WALL, OVEN_H), STEEL, True),  # left wall
        ((mid + WALL / 2, -HALF_W - WALL / 2, OVEN_H / 2), (depth + WALL, WALL, OVEN_H), STEEL, True),  # right wall
        ((BACK + WALL / 2, 0, OVEN_H / 2), (WALL, 2 * ow, OVEN_H), STEEL, True),  # back
        ((mid + WALL / 2, 0, (CEIL + OVEN_H) / 2), (depth + WALL, 2 * ow, OVEN_H - CEIL), STEEL, True),  # top
        ((mid, HALF_W - 0.001, (CAV_FLOOR + CEIL) / 2), (depth, 0.002, CEIL - CAV_FLOOR), ENAMEL, False),
        ((mid, -HALF_W + 0.001, (CAV_FLOOR + CEIL) / 2), (depth, 0.002, CEIL - CAV_FLOOR), ENAMEL, False),
        ((BACK - 0.001, 0, (CAV_FLOOR + CEIL) / 2), (0.002, 2 * HALF_W, CEIL - CAV_FLOOR), ENAMEL, False),
        ((mid, 0, CEIL + 0.001), (depth, 2 * HALF_W, 0.002), ENAMEL, False),
        # front frame around the opening (black trim)
        ((FRONT - 0.002, HALF_W + WALL / 2, OVEN_H / 2), (0.004, WALL, OVEN_H), BLACK, False),
        ((FRONT - 0.002, -HALF_W - WALL / 2, OVEN_H / 2), (0.004, WALL, OVEN_H), BLACK, False),
        ((FRONT - 0.002, 0, (CEIL + OVEN_H) / 2), (0.004, 2 * ow, OVEN_H - CEIL), BLACK, False),
        # the door, folded down flat in front of the opening: steel skin, dark window, handle at its far edge
        ((FRONT - DOOR_L / 2, 0, CAV_FLOOR - DOOR_T / 2), (DOOR_L, 2 * ow, DOOR_T), BLACK, True),
        ((FRONT - DOOR_L / 2 + 0.01, 0, CAV_FLOOR + 0.0005), (DOOR_L - 0.05, 2 * HALF_W - 0.04, 0.001), GLASS, False),
        # rack runners on the side walls, at shelf height
        ((mid, HALF_W - 0.004, RACK_Z - 0.004), (depth - 0.01, 0.008, 0.004), CHROME, False),
        ((mid, -HALF_W + 0.004, RACK_Z - 0.004), (depth - 0.01, 0.008, 0.004), CHROME, False),
        # control strip along the top front, with a dial readout
        ((FRONT - 0.003, 0.05, CEIL + 0.015), (0.002, 0.05, 0.012), GLOW, False),
    ]
    meshes, colliders = [], []
    for c, s, look, solid in parts:
        cc = _w(c)
        meshes.append(kit.box(cc, s, look))
        if solid:
            colliders.append(kit.Collider(cc, size=s))
    # the door handle: a chrome bar under the door's far edge
    meshes.append(kit.cylinder(_w((FRONT - DOOR_L + 0.008, 0, CAV_FLOOR - DOOR_T - 0.006)), 0.005, 2 * HALF_W - 0.02,
                               CHROME, axis="y"))
    # glowing elements: three bars under the ceiling and three just above the floor
    for y in (-0.06, 0.0, 0.06):
        meshes.append(kit.cylinder(_w((mid, y, CEIL - 0.008)), 0.003, depth - 0.03, GLOW, axis="x"))
        meshes.append(kit.cylinder(_w((mid, y, CAV_FLOOR + 0.006)), 0.003, depth - 0.03, GLOW, axis="x"))
    # two control knobs on the top front strip
    for y in (-0.05, -0.08):
        meshes.append(kit.cylinder(_w((FRONT - 0.008, y, CEIL + 0.015)), 0.010, 0.012, KNOB, axis="x"))

    # the rack: link frame at the front rail's foot, on the shelf plane; it slides along x (negative = out)
    shelf_c = (RACK_L / 2 - RAIL_T / 2, 0.0, -0.002)
    rack_meshes = [kit.cylinder((0, 0, RAIL_H / 2), RAIL_T / 2, 2 * HALF_W - 0.012, CHROME, axis="y"),
                   kit.cylinder((RACK_L - RAIL_T, 0, 0.0), 0.003, 2 * HALF_W - 0.012, CHROME, axis="y")]
    for y in np.linspace(-HALF_W + 0.012, HALF_W - 0.012, 12):
        rack_meshes.append(kit.cylinder((RACK_L / 2 - RAIL_T / 2, y, -0.002), 0.002, RACK_L - RAIL_T, CHROME,
                                        axis="x"))
    for x in (0.0, RACK_L - RAIL_T):
        for y in (-HALF_W + 0.008, HALF_W - 0.008):
            rack_meshes.append(kit.cylinder((x, y, RAIL_H / 4), 0.002, RAIL_H / 2 + 0.004, CHROME))
    rack_colliders = (kit.Collider(shelf_c, size=(RACK_L - RAIL_T, 2 * HALF_W - 0.012, 0.004)),
                      kit.Collider((0, 0, RAIL_H / 2), size=(RAIL_T, 2 * HALF_W - 0.012, RAIL_H)))
    rack = kit.Link("rack", "prismatic", "X", _w((RAIL_X, 0.0, RACK_Z)), lower=-PULL, upper=0.0, start=0.0, mass=0.4,
                    meshes=tuple(rack_meshes), colliders=rack_colliders, damping=5.0, floating=True, stay=150.0,
                    drag=4.5)
    return kit.Mechanism("oven", pos=BASE, size=(BACK - FRONT + DOOR_L + 0.04, 2 * (HALF_W + WALL) + 0.01, OVEN_H),
                         meshes=tuple(meshes), colliders=tuple(colliders), links=(rack,))


def _pan():
    """A dark steel loaf pan with a rolled rim, holding a baked loaf with a domed, split top."""
    lx, ly, lz = PAN
    t = 0.003
    m = [kit.box((0, 0, -lz / 2 + t / 2), (lx, ly, t), PAN_LOOK),
         kit.box((lx / 2 - t / 2, 0, 0), (t, ly, lz), PAN_LOOK), kit.box((-lx / 2 + t / 2, 0, 0), (t, ly, lz), PAN_LOOK),
         kit.box((0, ly / 2 - t / 2, 0), (lx, t, lz), PAN_LOOK), kit.box((0, -ly / 2 + t / 2, 0), (lx, t, lz), PAN_LOOK),
         kit.box((0, ly / 2, lz / 2 - 0.002), (lx + 0.004, 0.004, 0.004), PAN_LOOK),
         kit.box((0, -ly / 2, lz / 2 - 0.002), (lx + 0.004, 0.004, 0.004), PAN_LOOK),
         kit.box((lx / 2, 0, lz / 2 - 0.002), (0.004, ly + 0.004, 0.004), PAN_LOOK),
         kit.box((-lx / 2, 0, lz / 2 - 0.002), (0.004, ly + 0.004, 0.004), PAN_LOOK),
         kit.box((0, 0, -0.002), (lx - 2 * t, ly - 2 * t, lz - 0.008), CRUST),
         kit.cylinder((0, 0, lz / 2 - 0.006), (ly - 2 * t) / 2, lx - 2 * t - 0.004, CRUST, axis="x")]
    return kit.Prop("pan", pos=PAN_START, size=PAN, mass=0.15, meshes=tuple(m),
                    colliders=(kit.Collider((0, 0, 0), size=PAN),))


def _trivet():
    """A round cast-iron trivet on three short feet."""
    m = [kit.lathe([(TRIVET_R, 0.004), (TRIVET_R, TRIVET_H / 2), (TRIVET_R - 0.008, TRIVET_H / 2),
                    (TRIVET_R - 0.008, 0.0045), (0.0, 0.0045)], IRON)]
    for k in range(6):  # spokes
        a = k * math.pi / 3
        m.append(kit.box((0.026 * math.cos(a), 0.026 * math.sin(a), 0.004), (0.004, 0.004, 0.004), IRON))
    for k in range(3):
        a = k * 2 * math.pi / 3 + 0.5
        m.append(kit.cylinder((0.045 * math.cos(a), 0.045 * math.sin(a), -TRIVET_H / 4), 0.004, TRIVET_H / 2, IRON))
    return kit.Prop("trivet", pos=(TRIVET_XY[0], TRIVET_XY[1], TOP + TRIVET_H / 2), size=(2 * TRIVET_R, 2 * TRIVET_R,
                    TRIVET_H), meshes=tuple(m),
                    colliders=(kit.Collider((0, 0, 0), size=(2 * TRIVET_R * 0.8, 2 * TRIVET_R * 0.8, TRIVET_H)),))


OBJECTS = [_oven(), _trivet(), _pan()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.reach_run = 0
        self.spot_run = 0
        self.reached_in = False  # a gripper inside the hot cavity: irreversible
        self.pan_moved_in_oven = False  # the pan left its spot on the rack before the rack was out: irreversible
        self.rack_was_out = False
        self.pan_out = False  # the pan has left its spot with the rack pulled out (taken off the rack)
        self.lifted = False  # a gripper took the pan off the rack
        self.drop_run = 0
        self.dropped = False  # the pan fell, or tipped past 45 degrees: irreversible
        self.home = None  # each gripper's tool point at the trial's first scored step
        self.last_grip = None
        self.grips_still = False


_H = _History()


def _tilt(s):
    x, y = s.quat_xyzw[0], s.quat_xyzw[1]
    return math.degrees(math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _in_cavity(p, depth):
    """A tool point inside the oven, more than `depth` past the opening."""
    return bool(FRONT + depth < p[0] < BACK and abs(p[1]) < HALF_W and TOP + CAV_FLOOR < p[2] < TOP + CEIL)


def _held(objects):
    """A gripper's fingers closed (opening under 0.6) with its tool point within 5 cm of the pan's center."""
    p = np.asarray(objects["pan"].pos, float)
    return any(kit.opening(objects, g) < HOLD_OPEN and np.linalg.norm(np.asarray(objects[g].pos) - p) < HOLD_R
               for g in ("left_gripper", "right_gripper") if g in objects)


def _on_spot(pan, q):
    """The pan still where it sits on the rack (which has slid q along x): within 2 cm, and not lifted."""
    p = np.asarray(pan.pos, float)
    return bool(abs(p[0] - (PAN_START[0] + q)) < OFF_SPOT and abs(p[1] - PAN_START[1]) < OFF_SPOT
                and p[2] < PAN_START[2] + LIFTED)


def _on_trivet(pan):
    p = np.asarray(pan.pos, float)
    return bool(np.hypot(p[0] - TRIVET_XY[0], p[1] - TRIVET_XY[1]) < 0.03
                and abs(p[2] - (TOP + TRIVET_H + PAN[2] / 2)) < 0.01 and _tilt(pan) < 15 and pan.resting())


def _set_down_out(pan, q):
    """Out of the oven and set down somewhere other than the trivet: off the rack, past the opening, upright, still."""
    p = np.asarray(pan.pos, float)
    return bool(not _on_spot(pan, q) and p[0] + PAN[0] / 2 < FRONT and _tilt(pan) < 15 and pan.resting()
                and p[2] < TOP + RACK_Z)


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    grips = {g: np.asarray(objects[g].pos, float) for g in ("left_gripper", "right_gripper") if g in objects}
    if _H.home is None:
        _H.home = grips
    _H.grips_still = _H.last_grip is not None and all(
        g in _H.last_grip and np.linalg.norm(p - _H.last_grip[g]) < HOME_STILL for g, p in grips.items())
    _H.last_grip = grips
    q = kit.joint(objects, "oven/rack").pos
    if q <= OUT:
        _H.rack_was_out = True
    depth = INSIDE_CLEAR if _H.reach_run else INSIDE  # once in, it has to come back within 3 cm to count as out
    reach = any(_in_cavity(objects[g].pos, depth) for g in ("left_gripper", "right_gripper") if g in objects)
    _H.reach_run = _H.reach_run + 1 if reach else 0
    if _H.reach_run >= VIOLATION_STEPS:
        _H.reached_in = True
    if q <= OUT and not _on_spot(objects["pan"], q):
        _H.pan_out = True
    moved = q > OUT and not _H.pan_out and not _on_spot(objects["pan"], q)
    _H.spot_run = _H.spot_run + 1 if moved else 0
    if _H.spot_run >= VIOLATION_STEPS:
        _H.pan_moved_in_oven = True
    pan, held = objects["pan"], _held(objects)
    p = np.asarray(pan.pos, float)
    off = abs(p[0] - (PAN_START[0] + q)) > TAKEN or abs(p[1] - PAN_START[1]) > TAKEN or p[2] > PAN_START[2] + TAKEN
    if held and off:
        _H.lifted = True
    falling = not held and float(pan.lin_vel[2]) < -FALL_SPEED
    _H.drop_run = _H.drop_run + 1 if (falling or _tilt(pan) > DROP_TILT) else 0
    if _H.drop_run >= VIOLATION_STEPS:
        _H.dropped = True


def _score(objects):
    if _H.reached_in or _H.pan_moved_in_oven or _H.dropped or not _H.rack_was_out:
        return 0.0
    pan, q = objects["pan"], kit.joint(objects, "oven/rack").pos
    placed = _H.lifted and not _held(objects)  # carried off the rack by a gripper, and let go
    if placed and _on_trivet(pan):
        if q < IN:
            return 0.75  # the hot rack left sticking out
        return 1.0 if (_arms_home(objects) or not REQUIRE_ARMS_HOME) else 0.9
    if placed and _set_down_out(pan, q):
        return 0.5
    return 0.25


def _arms_home(objects):
    """Each gripper's tool point within 3 cm of where it was at the trial's first scored step, and still."""
    if not _H.home or not _H.grips_still:
        return False
    return all(np.linalg.norm(np.asarray(objects[g].pos, float) - p) < HOME_TOL for g, p in _H.home.items()
               if g in objects)


def success(objects):
    """Full score: the pan on the trivet, the rack back in and (with REQUIRE_ARMS_HOME) both arms home and still.
    Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if a gripper's tool point went more than 3.5 cm inside the oven's opening, within the cavity, for 2 steps in a
    row (once in, it counts as out again only within 3 cm of the opening); if the pan left its spot on the rack (moved
    2 cm, or lifted 1.5 cm) while the rack was still in (less than 11 cm out); or if the pan was dropped (falling faster
    than 0.5 m/s with no gripper holding it, or tipped past 45 degrees, for 2 steps). These are irreversible. 0 also if
    the rack was never pulled out. Otherwise, once a gripper has taken the pan off the rack and let go of it: with the
    pan resting upright on the trivet, 1.0 if the rack is back in (within 1 cm) and both arms are home and still, 0.9 if
    the arms are not home (with REQUIRE_ARMS_HOME), 0.75 if the rack is left out; 0.5 with the pan set down upright
    elsewhere outside the oven. Anything else with the rack pulled out scores 0.25. Possible scores: 0, 0.25, 0.5,
    0.75, 0.9, 1."""
    return _score(objects)


def demo():
    """Right arm: pinch the rack's front rail and pull the rack out to its stop, let go, then grip the pan across its
    width, lift it off the pulled-out rack and set it on the trivet. Then pinch the rail again, push the rack back in,
    and return to where it started. The left arm stays at home."""

    def grip(top, below):  # TCP height that puts the finger pads `below` an object's top
        return top - below + 0.013

    travel = TOP + 0.22
    # The rail sits under the oven's top front lip, so the hand reaches in tilted, fingertips forward and down (as a
    # hand in a mitt does), with the fingers closing across the rail.
    yaw, pitch = math.pi / 2, -0.6
    axis = np.array([math.sin(-pitch), 0.0, -math.cos(-pitch)])  # where the fingertips point
    ry = -0.04
    rail = np.array([RAIL_X, ry, TOP + RACK_Z + RAIL_H / 2])
    at = rail - 0.004 * axis
    back = at - 0.06 * axis
    half = FINGER_OPEN * 0.6
    pull_to = at - np.array([PULL + 0.004, 0.0, 0.0])
    T = lambda d, p, f, label: kit.Tilted(d, tuple(p), f, label, yaw, pitch=pitch)  # noqa: E731
    right = [Waypoint(1.0, (0.20, -0.20, travel), FINGER_OPEN, "start"),
             T(1.5, back + np.array([0, 0, 0.04]), half, "in front of the rail"),
             T(1.0, back, half, "line up"),
             T(1.0, at, half, "reach in around the rail"),
             T(0.8, at, FINGER_CLOSED, "pinch the rail")]
    for k in range(1, 9):
        right.append(T(0.35, at + (pull_to - at) * k / 8, FINGER_CLOSED, "pull"))
    right += [T(0.6, pull_to, half, "let go"),
              T(0.8, pull_to - 0.05 * axis + np.array([0, 0, 0.03]), half, "back off")]
    px = PAN_START[0] - PULL
    pan_top = PAN_START[2] + PAN[2] / 2
    pz = grip(pan_top, 0.020)
    place_z = grip(TOP + TRIVET_H + PAN[2], 0.020) + 0.004
    right += [Waypoint(1.2, (px, 0.0, pz + 0.06), FINGER_OPEN, "over the pan"),
              Waypoint(1.0, (px, 0.0, pz), FINGER_OPEN, "around the pan"),
              Waypoint(0.8, (px, 0.0, pz), FINGER_CLOSED, "grip"),
              Waypoint(1.2, (px, 0.0, travel), FINGER_CLOSED, "lift off the rack"),
              Waypoint(1.8, (TRIVET_XY[0], TRIVET_XY[1], travel), FINGER_CLOSED, "carry"),
              Waypoint(1.2, (TRIVET_XY[0], TRIVET_XY[1], place_z), FINGER_CLOSED, "lower onto the trivet"),
              Waypoint(0.6, (TRIVET_XY[0], TRIVET_XY[1], place_z), FINGER_OPEN, "release"),
              Waypoint(1.0, (TRIVET_XY[0], TRIVET_XY[1], travel), FINGER_OPEN, "up")]
    # close up: pinch the pulled-out rail again and push the rack home
    out_at = pull_to
    right += [T(1.5, out_at - 0.06 * axis + np.array([0, 0, 0.04]), half, "in front of the pulled-out rail"),
              T(1.0, out_at - 0.06 * axis, half, "line up"),
              T(1.0, out_at, half, "around the rail"),
              T(0.8, out_at, FINGER_CLOSED, "pinch the rail")]
    push_to = at + np.array([0.004, 0.0, 0.0])
    for k in range(1, 9):
        right.append(T(0.35, out_at + (push_to - out_at) * k / 8, FINGER_CLOSED, "push the rack in"))
    right += [T(0.6, push_to, half, "let go"),
              T(0.8, push_to - 0.06 * axis + np.array([0, 0, 0.03]), half, "back off"),
              kit.Home(1.8),
              kit.Home(2.0, label="hold at home")]
    total = sum(w.duration for w in right)
    left = [kit.Home(total, label="stay home")]
    return {"left": left, "right": right}
