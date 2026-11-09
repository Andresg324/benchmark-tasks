"""A plumbed emergency eyewash, bench-mounted on the table: a steel bowl on a supply housing, two spray heads each under
a yellow flip-top dust cover, and a push flag on a quarter-turn stay-open ball valve at the bowl's front right. Its
weekly test (ANSI/ISEA Z358.1, 5.5.2) is to open the valve and let it run until the water comes out clear, which proves
the supply and flushes sediment and bacteria out of the line, then to shut it and put the dust covers back on.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the eyewash as an articulation; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Do the weekly test on the eyewash station."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the workspace is clear for
# the next person or job. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# The unit (fixed). Its base frame sits on the table at BASE; the coordinates below are in that frame, z above the table.
BASE = (0.31, 0.0, TOP)
HOUSING = (0.09, 0.15, 0.04)  # the supply housing the bowl sits on (x by y by z)
BOWL_Z, BOWL_R, BOWL_H = 0.04, 0.10, 0.05  # the bowl's underside height, rim radius and depth
FLOOR_Z = BOWL_Z + 0.006  # the bowl's inner floor
HEAD_X, HEAD_Y = -0.005, 0.045  # the spray heads at y = +/- HEAD_Y
HEAD_R, HEAD_TOP = 0.012, 0.074
DRAIN_X = 0.04

# The dust covers: a cap on each head, hinged at the head's back edge (about y). 0 = on the head; the water flips it
# up and back to COVER_OPEN, past upright, where it stays until pushed forward; past upright it falls shut.
CAP_R, CAP_T = 0.014, 0.008
COVER_HINGE_X = HEAD_X + HEAD_R
COVER_OPEN = 1.92  # rad (110 degrees)

# The push flag: a plate on the valve's vertical stem at the bowl's front right. 0 = closed, pointing to the robot's
# right (-y); a quarter turn pushes it back (+x) to open, alongside the bowl. It stays where it is left.
STEM = (-0.07, -0.15)  # in the base frame (world x 0.24, y -0.15)
FLAG_Z0, FLAG_H = 0.06, 0.045  # the plate spans z FLAG_Z0 to FLAG_Z0 + FLAG_H above the table
FLAG_IN, FLAG_OUT, FLAG_T = 0.01, 0.06, 0.004  # the plate runs from 1 to 6 cm out from the stem, 4 mm thick
FLAG_OPEN = math.pi / 2

# Water: two jets per tint on a slide that hides them inside the housing; the kit shows them only while the valve is
# open: tinted rust for the first CLEAR_S seconds of an unbroken flow, clear after.
JET_L, JET_HIDE = 0.045, 0.08
CLEAR_S = 10.0

# Scoring
OPEN_Q = math.radians(80)  # the valve is open (full flow) with the flag turned 80 degrees or more
SHUT_Q = math.radians(10)  # and shut with the flag back within 10 degrees of closed
COVER_ON = math.radians(10)  # a dust cover is on with its cap within 10 degrees of its head
FLUSH_STEPS = 240  # the flush: the valve open without a break for 240 control steps (12 s at 20 Hz)
HANDS_OFF = 0.045  # m: a gripper's tool point closer than this to the flag's or a cap's center is on it
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.001  # m per control step: the tool point not moving
GRIPPERS = ("left_gripper", "right_gripper")

STEEL = kit.Look((0.80, 0.81, 0.83), 0.25, metallic=0.9)
BRUSHED = kit.Look((0.70, 0.71, 0.73), 0.45, metallic=0.8)
CHROME = kit.Look((0.88, 0.89, 0.91), 0.12, metallic=1.0)
HOUSING_LOOK = kit.Look((0.86, 0.86, 0.84), 0.6)
BRASS = kit.Look((0.78, 0.62, 0.30), 0.35, metallic=0.8)
DARK = kit.Look((0.10, 0.10, 0.11), 0.6)
NOZZLE = kit.Look((0.95, 0.95, 0.93), 0.5)
CAP_LOOK = kit.Look((0.98, 0.78, 0.05), 0.45)  # safety-yellow plastic
FLAG_LOOK = kit.Look((0.05, 0.55, 0.25), 0.45)  # emergency-green face on a steel flag
RUST_WATER = kit.Look((0.50, 0.20, 0.03), 0.3, opacity=1.0)  # strongly rust-brown, opaque
CLEAR_WATER = kit.Look((0.86, 0.94, 1.0), 0.05, opacity=0.35)  # pale blue-white, see-through


def _jets(look):
    """The two water streams in the jet link's frame (at the head tops): each rises JET_L from its head and arcs
    4.5 cm toward the user, falling back into the bowl, as an eyewash's soft streams do; and the water pooling in the
    bowl around the drain."""
    m = []
    for y in (-HEAD_Y, HEAD_Y):
        path = [(-0.045 * t, y, 4 * JET_L * t * (1 - t)) for t in np.linspace(0.0, 1.0, 14)]
        m.append(kit.band(path, 0.013, 0.010, look, normal=(0, 1, 0)))
    m.append(kit.cylinder((-HEAD_X, 0.0, FLOOR_Z + 0.002 - HEAD_TOP), 0.05, 0.002, look))  # the pool
    return tuple(m)


def _eyewash():
    meshes = [
        kit.box((0, 0, HOUSING[2] / 2), HOUSING, HOUSING_LOOK),  # supply housing
        kit.box((0, 0, 0.002), (HOUSING[0] + 0.03, HOUSING[1] + 0.03, 0.004), BRUSHED),  # mounting plate
        # the bowl: a steel shell (outer wall, rolled rim, inner wall, floor)
        kit.lathe([(0.035, 0.0), (0.075, 0.02), (BOWL_R, BOWL_H - 0.005), (BOWL_R, BOWL_H), (BOWL_R - 0.006, BOWL_H),
                   (BOWL_R - 0.008, BOWL_H - 0.005), (0.07, 0.022), (0.03, 0.006), (0.0, 0.006)], STEEL,
                  center=(0, 0, BOWL_Z)),
        kit.cylinder((DRAIN_X, 0, FLOOR_Z + 0.0005), 0.012, 0.001, DARK),  # drain
    ]
    for y in (-HEAD_Y, HEAD_Y):  # the spray heads: a chrome riser, a head and a white nozzle face
        meshes.append(kit.cylinder((HEAD_X, y, (FLOOR_Z + HEAD_TOP - 0.012) / 2), 0.006, HEAD_TOP - 0.012 - FLOOR_Z,
                                   CHROME))
        meshes.append(kit.cylinder((HEAD_X, y, HEAD_TOP - 0.006), HEAD_R, 0.012, CHROME))
        meshes.append(kit.cylinder((HEAD_X, y, HEAD_TOP - 0.0005), HEAD_R - 0.002, 0.0012, NOZZLE))
        meshes.append(kit.box((COVER_HINGE_X + 0.002, y, HEAD_TOP - 0.003), (0.006, 0.012, 0.006), CHROME))  # hinge lug
    # supply pipe from the housing's right side to the valve, the valve body and its stem collar
    sx, sy = STEM
    meshes.append(kit.cylinder((sx, (sy - HOUSING[1] / 2) / 2, 0.03), 0.007, abs(sy) - HOUSING[1] / 2, BRASS,
                               axis="y"))
    meshes.append(kit.cylinder((sx, sy, 0.035), 0.012, 0.04, BRASS))
    meshes.append(kit.cylinder((sx, sy, FLAG_Z0 - 0.003), 0.008, 0.006, CHROME))
    colliders = [kit.Collider((0, 0, HOUSING[2] / 2), size=HOUSING),
                 kit.Collider((0, 0, FLOOR_Z - 0.003), size=(0.14, 0.14, 0.006))]
    wall_z, wall_h = BOWL_Z + BOWL_H / 2 + 0.01, BOWL_H - 0.02  # the bowl's upper wall, 6 to 9 cm above the table
    for k in range(16):  # the wall as a ring of axis-aligned slabs (the kit's colliders do not turn)
        a = 2 * math.pi * k / 16
        c, s = math.cos(a), math.sin(a)
        if abs(abs(c) - abs(s)) < 0.1:  # the diagonals: square blocks
            colliders.append(kit.Collider((0.088 * c, 0.088 * s, wall_z), size=(0.022, 0.022, wall_h)))
        else:
            size = (0.012, 0.04, wall_h) if abs(c) > abs(s) else (0.04, 0.012, wall_h)
            colliders.append(kit.Collider((0.093 * c, 0.093 * s, wall_z), size=size))
    for y in (-HEAD_Y, HEAD_Y):
        colliders.append(kit.Collider((HEAD_X, y, (FLOOR_Z + HEAD_TOP) / 2), size=(2 * HEAD_R, 2 * HEAD_R,
                                                                                  HEAD_TOP - FLOOR_Z)))
    colliders.append(kit.Collider((sx, sy, 0.035), size=(0.024, 0.024, 0.04)))  # valve body

    links = []
    # the flag: link frame on the stem at the plate's bottom; at 0 the plate points along -y
    fm = (kit.cylinder((0, 0, FLAG_H / 2), 0.005, FLAG_H, CHROME),
          kit.box((0, -(FLAG_IN + FLAG_OUT) / 2, FLAG_H / 2), (FLAG_T, FLAG_OUT - FLAG_IN, FLAG_H), STEEL),
          kit.box((-FLAG_T / 2 - 0.0005, -(FLAG_IN + FLAG_OUT) / 2, FLAG_H / 2),
                  (0.001, FLAG_OUT - FLAG_IN - 0.008, FLAG_H - 0.008), FLAG_LOOK),
          kit.box((FLAG_T / 2 + 0.0005, -(FLAG_IN + FLAG_OUT) / 2, FLAG_H / 2),
                  (0.001, FLAG_OUT - FLAG_IN - 0.008, FLAG_H - 0.008), FLAG_LOOK),
          kit.box((0, -FLAG_IN / 2, 0.006), (0.006, FLAG_IN, 0.008), STEEL))
    links.append(kit.Link("flag", "revolute", "Z", (sx, sy, FLAG_Z0), lower=0.0, upper=FLAG_OPEN, start=0.0, mass=0.12,
                          meshes=fm, colliders=(kit.Collider((0, -(FLAG_IN + FLAG_OUT) / 2, FLAG_H / 2),
                                                             size=(FLAG_T + 0.002, FLAG_OUT - FLAG_IN, FLAG_H)),),
                          damping=0.02, floating=True, stay=0.05, drag=0.01))
    # the covers: link frame on the hinge at the head's back edge; at 0 the cap lies on the head
    for side, y in (("right", -HEAD_Y), ("left", HEAD_Y)):
        cm = (kit.cylinder((-HEAD_R - 0.002, 0, CAP_T / 2), CAP_R, CAP_T, CAP_LOOK),
              kit.cylinder((-HEAD_R - 0.002, 0, CAP_T + 0.0005), CAP_R - 0.004, 0.001,
                           kit.Look((0.90, 0.68, 0.03), 0.5)),
              kit.box((-0.002, 0, CAP_T / 2), (0.006, 0.010, CAP_T), CAP_LOOK),
              kit.box((-2 * CAP_R - 0.004, 0, CAP_T / 2), (0.006, 0.012, 0.004), CAP_LOOK))  # finger tab at the front
        links.append(kit.Link(f"cover_{side}", "revolute", "Y", (COVER_HINGE_X, y, HEAD_TOP), lower=0.0,
                              upper=COVER_OPEN, start=0.0, mass=0.05, meshes=cm,
                              colliders=(kit.Collider((-HEAD_R - 0.002, 0, CAP_T / 2),
                                                      size=(2 * CAP_R, 2 * CAP_R, CAP_T)),),
                              damping=0.001, driven_by="flag", driven_at=OPEN_Q, push=0.03, drag=0.0))
    # the water: rusty while the line flushes, then clear
    for name, look, show in (("water_rust", RUST_WATER, (0.0, CLEAR_S)), ("water_clear", CLEAR_WATER, (CLEAR_S, 1e9))):
        links.append(kit.Link(name, "prismatic", "Z", (HEAD_X, 0.0, HEAD_TOP), lower=-JET_HIDE, upper=0.0,
                              start=-JET_HIDE, mass=0.02, meshes=_jets(look),
                              # a 3 mm guide block between the heads (shown) or inside the housing (hidden): PhysX
                              # needs a shape on a moving part
                              colliders=(kit.Collider((0.0, 0.0, 0.01), size=(0.003, 0.003, 0.003)),), damping=0.5, floating=True,
                              driven_by="flag", driven_at=OPEN_Q, show=show, show_hold=5.0))
    return kit.Mechanism("eyewash", pos=BASE, size=(0.22, 0.42, BOWL_Z + BOWL_H), meshes=tuple(meshes),
                         colliders=tuple(colliders), links=tuple(links))


OBJECTS = [_eyewash()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.open_run = 0  # control steps in a row with the valve open
        self.flushed = False  # the valve stayed open for FLUSH_STEPS in a row at some point: kept for the trial
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
    flag = kit.joint(objects, "eyewash/flag").pos
    _H.open_run = _H.open_run + 1 if flag >= OPEN_Q else 0
    if _H.open_run >= FLUSH_STEPS:
        _H.flushed = True


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still."""
    if _H.home is None or not _H.still:
        return False
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - _H.home[g]) < HOME_TOL for g in GRIPPERS)


def _flag_center(q):
    """The flag plate's center in world coordinates, with the flag turned q."""
    r, mid = (FLAG_IN + FLAG_OUT) / 2, FLAG_Z0 + FLAG_H / 2
    return np.array([BASE[0] + STEM[0] + r * math.sin(q), BASE[1] + STEM[1] - r * math.cos(q), TOP + mid])


def _cap_center(y, q):
    """A dust cap's center in world coordinates, with the cover turned q up from its head."""
    d = HEAD_R + 0.002
    return np.array([BASE[0] + COVER_HINGE_X - d * math.cos(q), BASE[1] + y, TOP + HEAD_TOP + d * math.sin(q)])


def _hands_off(objects):
    """No gripper's tool point within 4.5 cm of the flag plate's or a cap's center (nothing still held or pressed)."""
    parts = [_flag_center(kit.joint(objects, "eyewash/flag").pos),
             _cap_center(-HEAD_Y, kit.joint(objects, "eyewash/cover_right").pos),
             _cap_center(HEAD_Y, kit.joint(objects, "eyewash/cover_left").pos)]
    return all(np.linalg.norm(np.asarray(objects[g].pos, float) - p) >= HANDS_OFF for g in GRIPPERS if g in objects
               for p in parts)


def _job(objects):
    if not _H.flushed:
        return 0.0  # never flushed: nothing done, or only a short press
    if kit.joint(objects, "eyewash/flag").pos > SHUT_Q:
        return 0.25  # flushed, but the water left running (or the valve left part open)
    covers_on = all(kit.joint(objects, f"eyewash/cover_{s}").pos <= COVER_ON for s in ("left", "right"))
    if not covers_on or not _hands_off(objects):
        return 0.5  # flushed and shut, but a dust cover left off, or a gripper still on the flag or a cover
    return 1.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """The test done (flushed, shut, covers on, hands off) and, with REQUIRE_ARMS_HOME, both arms home and still.
    Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 unless the valve stayed open (the flag turned 80 degrees or more of its 90) for 240 control steps in a row
    (12 s at 20 Hz) at some point in the run: doing nothing, or a short press, scores 0. After such a flush, at the
    end: 0.25 if the flag is not back within 10 degrees of closed (water still running); 0.5 if it is but a dust
    cover is more than 10 degrees off its head, or a gripper's tool point is still within 4.5 cm of the flag's or a
    cap's center; 1.0 with both covers on and the hands off, if both arms are also home and still (0.9 if not, with
    REQUIRE_ARMS_HOME). Possible scores: 0, 0.25, 0.5, 0.9, 1."""
    return _score(objects)


# --- The scripted demo
def _w(p):
    """A base-frame point (z above the table) in world coordinates."""
    return np.array([BASE[0] + p[0], BASE[1] + p[1], TOP + p[2]])


def demo():
    """Right arm: push the flag back a quarter turn (the valve opens and stays open), wait while the water runs rusty
    and then clear, push the flag back to closed from the other side, then push the right dust cover forward over its
    head so it falls shut, and go home. Left arm, after that: push the left cover shut and go home."""
    travel = TOP + 0.18
    r = (FLAG_IN + FLAG_OUT) / 2 + 0.005  # push near the middle of the plate
    z = TOP + FLAG_Z0 + FLAG_H / 2 + 0.013 + 0.008  # pad tips a little above the plate's middle
    side = 0.016  # the tool point this far off the plate's face
    stem = _w((STEM[0], STEM[1], 0))[:2]

    def at(q, s):
        """Tool point beside the flag at angle q, on the opening (s = -1) or closing (s = +1) side."""
        u = np.array([math.sin(q), -math.cos(q)])
        n = np.array([math.cos(q), math.sin(q)])
        p = stem + r * u + s * side * n
        return (float(p[0]), float(p[1]), z)

    right = [Waypoint(1.2, (0.20, -0.22, travel), FINGER_CLOSED, "start")]
    a = at(-0.12, -1)
    right += [Waypoint(1.5, (a[0], a[1], z + 0.06), FINGER_CLOSED, "in front of the flag"),
              Waypoint(1.0, a, FINGER_CLOSED, "down beside it")]
    for k in range(1, 11):
        right.append(Waypoint(0.3, at(-0.12 + (FLAG_OPEN + 0.25) * k / 10, -1), FINGER_CLOSED, "push the flag back"))
    b = at(FLAG_OPEN + 0.13, -1)
    right += [Waypoint(0.8, (b[0], b[1] - 0.02, z), FINGER_CLOSED, "off the flag"),
              Waypoint(1.0, (b[0], b[1] - 0.02, z + 0.08), FINGER_CLOSED, "up"),
              Waypoint(9.0, (b[0] - 0.01, b[1] - 0.03, z + 0.10), FINGER_CLOSED, "let the water run until clear")]
    c = at(FLAG_OPEN + 0.10, +1)
    right += [Waypoint(1.5, (c[0], c[1], z + 0.06), FINGER_CLOSED, "behind the open flag"),
              Waypoint(1.0, c, FINGER_CLOSED, "down beside it")]
    for k in range(1, 11):
        right.append(Waypoint(0.3, at(FLAG_OPEN + 0.10 - (FLAG_OPEN + 0.25) * k / 10, +1), FINGER_CLOSED,
                              "pull the flag closed"))
    d = at(-0.15, +1)
    right += [Waypoint(0.8, (d[0] + 0.02, d[1], z), FINGER_CLOSED, "off the flag"),
              Waypoint(1.0, (d[0] + 0.02, d[1], z + 0.08), FINGER_CLOSED, "up")]

    def cover(y):
        """Waypoints that push one open cover forward from behind (it falls shut past upright)."""
        hx = BASE[0] + COVER_HINGE_X
        cz = TOP + HEAD_TOP + 0.016 + 0.013  # pad tips 1.6 cm above the hinge
        return [Waypoint(1.5, (hx + 0.035, y, cz + 0.06), FINGER_CLOSED, "behind the open cover"),
                Waypoint(1.0, (hx + 0.035, y, cz), FINGER_CLOSED, "down behind it"),
                Waypoint(1.5, (hx - 0.010, y, cz), FINGER_CLOSED, "push it forward"),
                Waypoint(0.8, (hx - 0.035, y, cz), FINGER_CLOSED, "on over the head"),
                Waypoint(0.8, (hx - 0.035, y, cz + 0.06), FINGER_CLOSED, "up")]

    right += cover(-HEAD_Y + 0.01) + [kit.Home(2.0), kit.Home(1.0, label="at home")]
    t_right = sum(w.duration for w in right)
    left = [kit.Home(t_right - 1.5, label="wait at home")] + cover(HEAD_Y - 0.01)
    left += [kit.Home(2.0), kit.Home(2.5, label="hold at home")]
    t_left = sum(w.duration for w in left)
    right.append(kit.Home(t_left - t_right + 0.5, label="hold at home"))
    return {"left": left, "right": right}
