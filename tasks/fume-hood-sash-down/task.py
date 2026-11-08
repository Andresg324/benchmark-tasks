"""A benchtop chemical fume hood, scaled to the arms: a steel cabinet with a sliding glass sash, raised all the way
open, and a control box on its left with two push buttons, LIGHT (yellow) and FAN (blue). Both are on. Bottles of
solvent stand inside. Closing up a hood for the night means pulling the sash all the way down and switching the light
off, while the exhaust fan stays on, because the chemicals left inside still give off vapor.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props and the moving parts; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Close up the fume hood for the night."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# The hood (fixed), in world coordinates: interior x from FRONT to BACK, y within +/- HALF_W, z from the table up.
FRONT, BACK, HALF_W, HEIGHT = 0.255, 0.43, 0.14, 0.31
WALL = 0.01
SILL_TOP = 0.022  # the airfoil sill along the front edge; the closed sash rests on it
OPENING = 0.19  # the front header starts this high above the table; above it the raised sash slides behind it
SASH_X = 0.250  # the sash's plane, just in front of the cabinet
SASH_H, SASH_T = 0.17, 0.006
OPEN = 0.12  # m the sash can rise; it starts fully open
MARK = 0.07  # the working-height mark on the side post (sash bottom 7 cm above the sill)
HANDLE = (0.040, 0.22, 0.012)  # the pull ledge along the sash's bottom edge, deep enough that a hand on it clears the glass
HANDLE_X = SASH_X - SASH_T / 2 - HANDLE[0] / 2

# The control box (fixed, part of the hood) beside the left wall, and its two buttons (spring return).
BOX_X, BOX_Y, BOX = 0.25, 0.25, (0.08, 0.045, 0.06)
LIGHT_X, FAN_X = 0.228, 0.272
BUTTON_R, BUTTON_H, TRAVEL = 0.011, 0.008, 0.006

# Scoring
CLOSED = 0.005  # m: the sash bottom within 5 mm of the sill
PRESSED, RELEASED = -0.004, -0.002  # m: a button counts as pressed below -4 mm, released above -2 mm
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving
GRIPPERS = ("left_gripper", "right_gripper")

STEEL = kit.Look((0.86, 0.87, 0.88), 0.35, metallic=0.4)
INSIDE = kit.Look((0.93, 0.93, 0.92), 0.6)
DARK = kit.Look((0.20, 0.21, 0.23), 0.5)
GLASS = kit.Look((0.80, 0.88, 0.92), 0.05, opacity=0.30)
FRAME = kit.Look((0.55, 0.57, 0.60), 0.3, metallic=0.8)
MARK_LOOK = kit.Look((0.95, 0.75, 0.10), 0.5)
YELLOW = kit.Look((0.98, 0.82, 0.10), 0.4)
BLUE = kit.Look((0.15, 0.40, 0.85), 0.4)
AMBER = kit.Look((0.55, 0.30, 0.08), 0.25, opacity=0.85)
CLEARB = kit.Look((0.90, 0.93, 0.95), 0.1, opacity=0.6)
LABEL = kit.Look((0.97, 0.97, 0.97), 0.6)


def _w(c, s):
    """A part given in world coordinates, as (center, size) in the hood base's frame."""
    return ((c[0] - BASE[0], c[1] - BASE[1], c[2] - BASE[2]), s)


BASE = ((FRONT + BACK) / 2, 0.0, TOP)


def _hood():
    mid = (FRONT + BACK) / 2
    depth = BACK - FRONT
    parts = [  # (world center, size, look, collides)
        ((mid, 0, TOP + 0.0075), (depth, 2 * HALF_W, 0.015), INSIDE, True),  # work surface
        ((FRONT - 0.0075, 0, TOP + SILL_TOP / 2), (0.015, 2 * HALF_W, SILL_TOP), FRAME, True),  # airfoil sill
        ((mid, HALF_W + WALL / 2, TOP + HEIGHT / 2), (depth + 0.01, WALL, HEIGHT), STEEL, True),  # left wall
        ((mid, -HALF_W - WALL / 2, TOP + HEIGHT / 2), (depth + 0.01, WALL, HEIGHT), STEEL, True),  # right wall
        ((BACK + WALL / 2, 0, TOP + HEIGHT / 2), (WALL, 2 * HALF_W + 2 * WALL, HEIGHT), STEEL, True),  # back
        ((mid, 0, TOP + HEIGHT - 0.005), (depth + 0.01, 2 * HALF_W + 2 * WALL, 0.01), STEEL, True),  # roof
        ((BACK - 0.004, 0, TOP + 0.13), (0.004, 2 * HALF_W - 0.02, 0.21), DARK, False),  # baffle slots
        # front header above the opening, in front of the raised sash
        ((SASH_X - SASH_T - 0.004, 0, TOP + (OPENING + HEIGHT) / 2), (0.006, 2 * HALF_W + 2 * WALL, HEIGHT - OPENING),
         STEEL, True),
        # working-height mark: a yellow arrow sticker on the left post's front edge
        ((FRONT - 0.001, HALF_W + WALL / 2, TOP + SILL_TOP + MARK), (0.002, WALL + 0.004, 0.008), MARK_LOOK, False),
        # control box beside the left wall
        ((BOX_X, BOX_Y, TOP + BOX[2] / 2), BOX, DARK, True),
    ]
    for x, look in ((LIGHT_X, YELLOW), (FAN_X, BLUE)):  # bezels and a label strip under each button
        parts.append(((x, BOX_Y, TOP + BOX[2] + 0.001), (0.03, 0.03, 0.002), FRAME, False))
        parts.append(((x, BOX_Y - 0.018, TOP + BOX[2] + 0.0015), (0.024, 0.006, 0.001), look, False))
    meshes, colliders = [], []
    for c, s, look, solid in parts:
        cc, ss = _w(c, s)
        meshes.append(kit.box(cc, ss, look))
        if solid:
            colliders.append(kit.Collider(cc, size=ss))
    # two solvent bottles standing inside (part of the scene, not to be moved)
    for y, look in ((0.06, AMBER), (-0.05, CLEARB)):
        cc, _ = _w((0.37, y, TOP + 0.015), (0, 0, 0))
        meshes.append(kit.lathe([(0.0, 0.0), (0.022, 0.0), (0.022, 0.07), (0.010, 0.085), (0.010, 0.095),
                                 (0.0, 0.095)], look, center=cc))
        colliders.append(kit.Collider((cc[0], cc[1], cc[2] + 0.0475), size=(0.044, 0.044, 0.095)))

    # the sash: glass in a steel frame, with a handle ledge along its bottom edge (link frame: bottom edge at z 0)
    sx = SASH_X - BASE[0]
    sash_meshes = (kit.box((0, 0, SASH_H / 2), (SASH_T, 2 * HALF_W - 0.004, SASH_H), GLASS),
                   kit.box((0, 0, 0.008), (SASH_T + 0.002, 2 * HALF_W, 0.016), FRAME),
                   kit.box((0, 0, SASH_H - 0.006), (SASH_T + 0.002, 2 * HALF_W, 0.012), FRAME),
                   kit.box((0, HALF_W - 0.006, SASH_H / 2), (SASH_T + 0.002, 0.012, SASH_H), FRAME),
                   kit.box((0, -HALF_W + 0.006, SASH_H / 2), (SASH_T + 0.002, 0.012, SASH_H), FRAME),
                   kit.box((HANDLE_X - SASH_X, 0, HANDLE[2] / 2 + 0.004), HANDLE, FRAME))
    sash_colliders = (kit.Collider((0, 0, SASH_H / 2), size=(SASH_T, 2 * HALF_W - 0.004, SASH_H)),
                      kit.Collider((HANDLE_X - SASH_X, 0, HANDLE[2] / 2 + 0.004), size=HANDLE))
    sash = kit.Link("sash", "prismatic", "Z", (sx, 0.0, SILL_TOP), lower=0.0, upper=OPEN, start=OPEN, mass=0.3,
                    meshes=sash_meshes, colliders=sash_colliders, damping=5.0, floating=True, stay=150.0, drag=4.5)

    def button(name, x, look):
        cc, _ = _w((x, BOX_Y, TOP + BOX[2]), (0, 0, 0))
        return kit.Link(name, "prismatic", "Z", cc, lower=-TRAVEL, upper=0.0, start=0.0, mass=0.02,
                        meshes=(kit.cylinder((0, 0, BUTTON_H / 2), BUTTON_R, BUTTON_H, look),),
                        colliders=(kit.Collider((0, 0, BUTTON_H / 2), size=(2 * BUTTON_R, 2 * BUTTON_R, BUTTON_H)),),
                        spring=150.0, rest=0.0, damping=2.0, floating=True)

    return kit.Mechanism("hood", pos=BASE, size=(BACK - FRONT + 0.06, 2 * HALF_W + 0.1, HEIGHT),
                         meshes=tuple(meshes), colliders=tuple(colliders),
                         links=(sash, button("light", LIGHT_X, YELLOW), button("fan", FAN_X, BLUE)))


OBJECTS = [_hood()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.down = {"light": False, "fan": False}  # each button held down right now
        self.presses = {"light": 0, "fan": 0}
        self.fan_was_off = False  # irreversible: the vapor is already out
        self.home = None  # each gripper's tool point at the trial's first scored step
        self.last = None
        self.still = False


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    for b in ("light", "fan"):
        q = kit.joint(objects, f"hood/{b}").pos
        if not _H.down[b] and q < PRESSED:
            _H.down[b] = True
            _H.presses[b] += 1
        elif _H.down[b] and q > RELEASED:
            _H.down[b] = False
    if _H.presses["fan"] % 2 == 1:
        _H.fan_was_off = True
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


def _sash_points(objects):
    q = kit.joint(objects, "hood/sash").pos
    if q <= CLOSED:
        return 0.75
    if q <= MARK:
        return 0.25
    return 0.0


def _job(objects):
    if _H.fan_was_off:
        return 0.0
    sash = _sash_points(objects)
    if sash == 0.0:
        return 0.0
    light_off = _H.presses["light"] % 2 == 1
    return sash + (0.25 if light_off else 0.0)


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """The job done (sash closed and still, light off, fan never off) and, with REQUIRE_ARMS_HOME, both arms home and
    still. Called first each step, so it also records the history."""
    _update(objects)
    done = _job(objects) == 1.0 and abs(kit.joint(objects, "hood/sash").vel) < 0.005
    return done and (_arms_home(objects) or not REQUIRE_ARMS_HOME)


def score(objects):
    """0 if the fan was ever switched off (an odd number of presses on FAN at any step; irreversible). Otherwise the
    sash: 0.75 with its bottom within 5 mm of the sill (closed), 0.25 at or below the working-height mark (7 cm) but
    not closed, 0 above the mark; plus 0.25 with the light switched off (an odd number of presses on LIGHT), counted
    only when the sash scores. With REQUIRE_ARMS_HOME, a finished job with the arms not back home and still scores 0.9
    instead of 1.0. Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1."""
    return _score(objects)


def demo():
    """Right arm: press down on the sash's handle ledge and push the sash to the sill. Left arm: press the LIGHT
    button once, then both back away."""
    yaw = 0.0
    tilt = -0.45  # fingertips reach forward and down onto the ledge, the wrist stays back from the glass
    hx, hz0 = HANDLE_X - 0.010, TOP + SILL_TOP + OPEN + HANDLE[2] + 0.004
    press_z = lambda z: z + 0.013 - 0.002  # noqa: E731  (TCP so the closed pad tips sit on the ledge's top)
    y = -0.07
    right = [Waypoint(1.0, (0.20, -0.20, TOP + 0.20), FINGER_CLOSED, "close the fingers"),
             kit.Tilted(1.5, (hx, y, press_z(hz0) + 0.03), FINGER_CLOSED, "over the handle", yaw, pitch=tilt),
             kit.Tilted(0.8, (hx, y, press_z(hz0)), FINGER_CLOSED, "on the handle", yaw, pitch=tilt)]
    for k in range(1, 11):
        z = TOP + SILL_TOP + OPEN * (1 - k / 10) + HANDLE[2] + 0.004
        right.append(kit.Tilted(0.45, (hx, y, press_z(z) - (0.004 if k == 10 else 0.0)), FINGER_CLOSED, "push down",
                                yaw, pitch=tilt))
    right += [kit.Tilted(0.8, (hx, y, press_z(TOP + SILL_TOP + HANDLE[2]) - 0.004), FINGER_CLOSED, "seat it", yaw,
                         pitch=tilt),
              Waypoint(1.0, (hx - 0.06, y, TOP + 0.12), FINGER_CLOSED, "back away"),
              kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    bx, by = LIGHT_X, BOX_Y - 0.008  # the left arm falls short toward its base (+y)
    top = TOP + BOX[2] + BUTTON_H
    left = [Waypoint(1.0, (0.20, 0.26, TOP + 0.20), FINGER_CLOSED, "close the fingers"),
            Waypoint(1.5, (bx, by, top + 0.05), FINGER_CLOSED, "over LIGHT"),
            Waypoint(0.8, (bx, by, press_z(top) + 0.002), FINGER_CLOSED, "touch"),
            Waypoint(0.6, (bx, by, press_z(top - TRAVEL) - 0.002), FINGER_CLOSED, "press"),
            Waypoint(0.6, (bx, by, top + 0.05), FINGER_CLOSED, "let go"),
            Waypoint(1.2, (0.20, 0.27, TOP + 0.20), FINGER_OPEN, "out of the way")]
    pad = sum(w.duration for w in right) - sum(w.duration for w in left) - 3.5
    left += [Waypoint(max(pad, 0.5), (0.20, 0.27, TOP + 0.20), FINGER_OPEN, "wait"), kit.Home(2.0),
             kit.Home(1.5, label="hold at home")]
    return {"left": left, "right": right}
