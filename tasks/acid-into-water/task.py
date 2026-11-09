"""A white secondary-containment tray on the bench, with two matching tall-form glass beakers standing in it: one holds
110 mL of water, the other, which has a white label with a red hazard band, holds 55 mL of concentrated acid.
Diluting a concentrated acid means adding the acid slowly to the water, never the water to the acid, because water
poured onto concentrated acid boils on contact and spatters acid.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import pourkit as pk  # noqa: E402  (the liquids: volumes, pours, spills)
import yamkit as kit  # noqa: E402  (modeled props, gripper states, tilted demo grips; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Dilute the acid with the water."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# The tray (fixed): outer footprint, floor and rim.
TRAY_C, TRAY = (0.29, -0.05), (0.30, 0.26)
TRAY_FLOOR, TRAY_RIM, TRAY_WALL = 0.005, 0.02, 0.008
FLOOR_Z = TOP + TRAY_FLOOR

# Two matching tall-form beakers (borosilicate glass), narrow enough for one gripper: outer radius, height, inner
# radius, base thickness. The acid one is labeled. 55 mL of acid into 110 mL of water is a 1:2 dilution.
BEAKER_R, BEAKER_H, BEAKER_IN, BEAKER_BASE = 0.0245, 0.12, 0.022, 0.004
WATER_XY, ACID_XY = (0.33, -0.01), (0.23, -0.12)
WATER0, ACID0 = 110.0, 55.0  # mL

# Each beaker stands on a flat square foot, hidden inside its round base, 1 mm proud of the round collider: a PhysX
# cylinder bumped onto its bottom rim can keep rolling on it and slowly tip over.
FOOT = 0.001
WATER_POS = (*WATER_XY, FLOOR_Z + BEAKER_H / 2)
ACID_POS = (*ACID_XY, FLOOR_Z + BEAKER_H / 2)

GLASS = kit.Look((0.86, 0.92, 0.95), 0.05, opacity=0.28)
TRAY_LOOK = kit.Look((0.93, 0.93, 0.91), 0.55)
LABEL = kit.Look((0.97, 0.97, 0.96), 0.6)
RED = kit.Look((0.80, 0.10, 0.08), 0.5)
MARKS = kit.Look((0.95, 0.95, 0.95), 0.5)
WATER_RGB, ACID_RGB = (0.55, 0.76, 0.97), (0.93, 0.86, 0.55)


def _ring(r, z, width, look, thick=0.0008):
    pts = [(r * math.cos(t), r * math.sin(t), z) for t in np.linspace(0, 2 * math.pi, 41)]
    return kit.band(pts, width, thick, look, normal=(0, 0, 1))


def _tray():
    (cx, cy), (sx, sy) = TRAY_C, TRAY
    parts = [((0, 0, TRAY_FLOOR / 2), (sx, sy, TRAY_FLOOR))]
    h = TRAY_RIM
    for s in (-1, 1):
        parts.append(((s * (sx - TRAY_WALL) / 2, 0, h / 2), (TRAY_WALL, sy, h)))
        parts.append(((0, s * (sy - TRAY_WALL) / 2, h / 2), (sx, TRAY_WALL, h)))
    return kit.Prop("tray", pos=(cx, cy, TOP), size=(sx, sy, h),
                    meshes=tuple(kit.box(c, s, TRAY_LOOK) for c, s in parts),
                    colliders=tuple(kit.Collider(c, size=s) for c, s in parts))


def _beaker(name, pos, labeled):
    z0 = -BEAKER_H / 2
    shell = kit.lathe([(0.0, z0), (BEAKER_R, z0), (BEAKER_R, -z0), (BEAKER_IN, -z0), (BEAKER_IN, z0 + BEAKER_BASE),
                       (0.0, z0 + BEAKER_BASE)], GLASS)
    meshes = [shell] + [_ring(BEAKER_R + 0.0004, z0 + BEAKER_BASE + z, 0.0012, MARKS) for z in (0.03, 0.06, 0.09)]
    if labeled:  # a white label with a red hazard band, between the graduation rings
        meshes += [_ring(BEAKER_R + 0.0006, z0 + 0.075, 0.022, LABEL), _ring(BEAKER_R + 0.0009, z0 + 0.081, 0.005, RED)]
    return kit.Prop(name, pos=pos, size=(2 * BEAKER_R, 2 * BEAKER_R, BEAKER_H), mass=0.20, meshes=tuple(meshes),
                    colliders=(kit.Collider((0, 0, FOOT / 2), radius=BEAKER_R, height=BEAKER_H - FOOT),
                               kit.Collider((0, 0, z0 + 0.002), size=(0.032, 0.032, 0.004))))


OBJECTS = [_tray(), _beaker("water_beaker", WATER_POS, False), _beaker("acid_beaker", ACID_POS, True)]


def _vessel(name, fill):
    return pk.Vessel(name, sections=((BEAKER_IN, BEAKER_H - BEAKER_BASE),), floor=-BEAKER_H / 2 + BEAKER_BASE,
                     fill=fill)


pk.setup([_vessel("water_beaker", {"water": WATER0}), _vessel("acid_beaker", {"acid": ACID0})],
         liquids={"water": WATER_RGB, "acid": ACID_RGB}, splash_drop=0.10, splash=0.25)
pk.set_floor(FLOOR_Z)


# Scoring
FULL = 0.9 * ACID0  # mL of acid in the water for full credit (90 %)
SOME = 0.1 * ACID0  # mL of acid in the water before anything counts (10 %)
SPILL_MAX = 0.05 * ACID0  # mL: more liquid than this spilled or splashed (5 % of the acid) is a spill
WATER_IN = 0.5  # mL: this much water entering the acid counts as water poured into acid
ACID_LEFT = 2.0  # mL: a container holding at least this much acid, more acid than water, is "the acid"
UPRIGHT = 10.0  # degrees
KNOCKED = 45.0  # degrees: the acid beaker leaning this far with no gripper holding it has been knocked over
ON_TRAY = 0.015  # m: a container's bottom within this of the tray floor stands on it
HOLD_R, HOLD_OPEN = 0.07, 0.6  # a gripper holds a container: tool point within 7 cm and fingers under 60 % open
HOME_TOL, HOME_STILL = 0.03, 0.002  # m; m per control step
GRIPPERS = ("left_gripper", "right_gripper")


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. pourkit updates the liquids inside the scene's object_states, before these are called. yamkit counts the
# simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.water_into_acid = False  # irreversible: the acid has already boiled and spattered
        self.loose = 0  # control steps in a row the acid beaker has leaned past KNOCKED with no gripper holding it
        self.knocked = False  # irreversible: an open beaker of acid went over
        self.home = None
        self.last = None
        self.still = False


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():
        _H = _History()
    for p in pk.pours(objects):
        if (p.dest != pk.SPILL and p.amounts.get("water", 0.0) >= WATER_IN
                and p.before.get("acid", 0.0) >= ACID_LEFT and p.before.get("acid", 0.0) > p.before.get("water", 0.0)):
            _H.water_into_acid = True
    _H.loose = _H.loose + 1 if pk.tilt(objects, "acid_beaker") > KNOCKED and not _held(objects, "acid_beaker") else 0
    if _H.loose >= 2:
        _H.knocked = True
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


def _held(objects, name):
    s = objects[name]
    return any(g in objects and kit.opening(objects, g) < HOLD_OPEN
               and np.linalg.norm(np.asarray(objects[g].pos) - np.asarray(s.pos)) < HOLD_R for g in GRIPPERS)


def _put_away(objects, name):
    """Standing upright and still on the tray floor, inside the tray, and let go."""
    s = objects[name]
    (cx, cy), (sx, sy) = TRAY_C, TRAY
    inside = abs(s.pos[0] - cx) < sx / 2 - TRAY_WALL and abs(s.pos[1] - cy) < sy / 2 - TRAY_WALL
    return (pk.tilt(objects, name) < UPRIGHT and abs(s.low[2] - FLOOR_Z) < ON_TRAY and inside
            and s.resting(max_speed=0.02, max_spin=1.0) and not _held(objects, name))


def _job(objects):
    if _H.water_into_acid or _H.knocked:
        return 0.0
    acid_in = pk.volume(objects, "water_beaker", "acid")
    if acid_in < SOME:
        return 0.0
    if pk.spilled(objects) > SPILL_MAX:
        return 0.25
    if acid_in < FULL:
        return 0.5
    if not (_put_away(objects, "water_beaker") and _put_away(objects, "acid_beaker")):
        return 0.75
    return 1.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """The job done and, with REQUIRE_ARMS_HOME, both arms home and still. Called first each step, so it also
    records the history."""
    _update(objects)
    return _job(objects) == 1.0 and (_arms_home(objects) or not REQUIRE_ARMS_HOME)


def score(objects):
    """0 if water ever went into the acid: at least 0.5 mL of water poured into a container that held at least 2 mL
    of acid and more acid than water (irreversible). 0 if the acid beaker ever leaned more than 45 degrees for 2 steps
    with no gripper holding it (knocked over; irreversible). 0 if less than 10 % of the acid (5.5 mL) is in the water
    beaker. 0.25 if more than 5 % of the acid's volume (2.75 mL) of liquid in all was spilled or splashed (a stream
    falling more than 10 cm splashes a quarter of itself). 0.5 with 10 to 90 % of the acid in the water. 0.75 with at
    least 90 % (49.5 mL) in, but a beaker not upright (within 10 degrees), still, let go and standing on the tray floor.
    1.0 with both put away and both arms home and still; 0.9 if an arm is not home. Possible scores: 0, 0.25, 0.5,
    0.75, 0.9, 1."""
    return _score(objects)


YAW, ROLL = math.pi / 2, 1.35  # the right hand comes in from -y, fingers closing along x, tipped about 13 degrees down
GRIP_Z = 0.06  # grip height above the tray floor
LIP_ABOVE = 0.05  # m: the pouring lip this far above the receiving rim (the held beaker rides about 2 cm low)


def pour_plan(source, target, pitch):
    """The right arm's plan: grip beaker `source` from the side, carry it, tip it about its lip over beaker `target`
    to `pitch` rad (positive tips toward +x, negative toward -x), tip it back, set it back on its spot, let go and go
    home."""
    pos = {"water_beaker": WATER_POS, "acid_beaker": ACID_POS}
    sx, sy, _ = pos[source]
    tx, ty, _ = pos[target]
    sign = 1.0 if pitch > 0 else -1.0
    roll, gz = ROLL, FLOOR_Z + GRIP_Z
    tcp = np.array([sx, sy, gz])
    lip0 = pk.lip(source, pos[source], toward=(sign, 0, 0))
    over = (tx - sign * 0.008, ty, FLOOR_Z + BEAKER_H + LIP_ABOVE)
    t = lambda d, p, f, label: kit.Tilted(d, p, f, label, yaw=YAW, roll=roll)  # noqa: E731
    plan = [t(1.5, (sx, sy - 0.09, gz + 0.03), FINGER_OPEN, "beside the beaker"),
            t(1.0, (sx, sy - 0.09, gz), FINGER_OPEN, "down"),
            t(1.2, tuple(tcp), FINGER_OPEN, "around the beaker"),
            t(0.8, tuple(tcp), FINGER_CLOSED, "grip"),
            t(1.2, (sx, sy, gz + 0.10), FINGER_CLOSED, "lift")]
    plan += pk.pour(tcp, lip0, over, roll=roll, yaw=YAW, start_pitch=sign * 0.85, to_pitch=pitch, above=0.05, steps=8,
                    duration=6.0, hold=2.0, back=3.0, finger=FINGER_CLOSED)
    plan += [t(1.5, (sx, sy, gz + 0.10), FINGER_CLOSED, "back over its spot"),
             t(1.2, (sx, sy, gz + 0.003), FINGER_CLOSED, "set down"),
             t(1.0, (sx, sy, gz + 0.003), FINGER_OPEN, "let go"),
             t(1.0, (sx, sy - 0.06, gz + 0.016), FINGER_OPEN, "back out"),
             t(1.0, (sx, sy - 0.10, gz + 0.08), FINGER_OPEN, "up and away"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    return plan


def demo():
    """The right arm pours the acid into the water: it grips the labeled beaker from the side, brings it behind the
    water beaker already tipped about 50 degrees, tips it on to 100 degrees about its lip 5 cm above the water beaker's
    rim, tips it back, sets it back on its spot, lets go and goes home. The left arm stays home."""
    return {"right": pour_plan("acid_beaker", "water_beaker", 1.75)}
