"""An M36 x 4 hold-down stud stands up out of a 25 mm steel base plate, with a hardened flat washer on it. A heavy
hex nut has been set on the stud's end but not started. A mechanic starting a nut by hand sets it square on the
stud and first turns it backward (counterclockwise seen from above) until its thread start passes the stud's and it
drops onto the thread with a small click; only then does it turn the nut forward, and the nut runs down freely. A nut
turned forward without that drop can go on crooked and cross-thread: it binds after part of a turn, and its thread and
the stud's are ruined. The model is deterministic: the nut's thread start is set half a turn past the stud's, so it
drops after 180 degrees backward, and it cross-threads if forced 60 degrees forward before that.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import twistkit as tk  # noqa: E402  (threaded caps on top of yamkit; this copy adds the thread-start model)
import yamkit as kit  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Start the nut on the stud and run it down two full turns by hand, then let go."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the workspace is clear
# for the next person or job. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# The base plate (fixed): 200 x 160 x 25 mm steel, its center at PLATE_C; the stud stands at STUD_XY.
PLATE_C = (0.30, 0.04)
PLATE = (0.200, 0.160, 0.025)
STUD_XY = (0.30, 0.06)
# The stud: M36 x 4 (36 mm across the thread, 4 mm pitch); the washer: ISO 7089, 66 mm across, 5 mm thick.
STUD_R, STUD_ROOT = 0.018, 0.0155
WASHER_R, WASHER_T = 0.033, 0.005
SEAT = PLATE[2] + WASHER_T  # the nut's underside when run down onto the washer (above the plate's foot)
# The nut: ISO 4032 M36 heavy hex, 55 mm across flats, 31 mm tall, 0.37 kg.
NUT_AF, NUT_H, NUT_MASS = 0.055, 0.031, 0.37
PERCH_TURNS = 4.5  # perched on the stud's end, its underside 4.5 turns (18 mm) above the washer
STUD_TOP = SEAT + 0.004 * PERCH_TURNS + 0.006  # the stud's chamfered end reaches 6 mm into the perched nut
THREAD = tk.Thread(pitch=0.004, turns=PERCH_TURNS + 3.0, start=-360.0 * PERCH_TURNS, max_tight=10.0,
                   drop=180.0, cross=60.0, bind=180.0, lift_stiffness=10000.0)  # stiff: the 0.37 kg nut sags 0.4 mm
NUT = "stud/nut"

# Scoring
RUN_TURNS = 2.0  # run down two full turns after the drop (8 mm): two threads fully engaged, so it is started true
PART_TURNS = 1.0  # one full turn: started, but not yet shown to run freely
GRIP_SLACK = 60.0  # at most 60 degrees of the nut's turning, in each direction, done without a gripper turning it:
# room for the nut coasting or being nudged as the fingers open and close between strokes (a few degrees each, a dozen
# regrips), a sixth of a turn (0.7 mm of travel), far short of the 180 back and 720 forward the job needs
HOME_TOL = 0.03  # a gripper is home within 3 cm of where its tool point was at the trial's first scored step
HOME_STILL = 0.001  # and still: its tool point moved less than 1 mm since the last step (2 cm/s)
HOLD_OPEN = 0.75  # a gripper holds the nut when its fingers are under 75 % open (55 mm is 58 %) with its tool point
HOLD_MARGIN = 0.02  # within 2 cm of the nut's outside

STEEL = kit.Look((0.55, 0.57, 0.60), 0.45, metallic=0.6)
PLATE_LOOK = kit.Look((0.30, 0.32, 0.34), 0.6, metallic=0.4)
ZINC = kit.Look((0.80, 0.80, 0.76), 0.35, metallic=0.7)
NUT_LOOK = kit.Look((0.20, 0.21, 0.23), 0.5, metallic=0.6)  # black-oxide heavy hex nut
YELLOW = kit.Look((0.95, 0.78, 0.10), 0.5)  # paint dots at the thread starts
WHITE = kit.Look((0.93, 0.93, 0.90), 0.6)


def _stud():
    """The fixed body: base plate, washer and the threaded stud (thread drawn as 4 mm rings)."""
    sx, sy = STUD_XY[0] - PLATE_C[0], STUD_XY[1] - PLATE_C[1]
    meshes = [kit.box((-sx, -sy, PLATE[2] / 2), PLATE, PLATE_LOOK)]
    for cx, cy in ((-0.075, -0.055), (0.075, -0.055)):  # two of the plate's own countersunk anchor bolts
        meshes.append(kit.cylinder((cx - sx, cy - sy, PLATE[2] + 0.0005), 0.009, 0.001, STEEL, n=6))
    meshes.append(kit.lathe([(0.0090, PLATE[2]), (WASHER_R, PLATE[2]), (WASHER_R, SEAT), (0.0190, SEAT),
                             (0.0190, PLATE[2] + 0.0001)], ZINC, n=40))
    prof = [(0.0, PLATE[2])]
    z = PLATE[2]
    while z + 0.004 < STUD_TOP - 0.003:
        prof += [(STUD_ROOT, z), (STUD_R, z + 0.002)]
        z += 0.004
    prof += [(STUD_ROOT, z), (STUD_ROOT, STUD_TOP - 0.0025), (STUD_ROOT - 0.0025, STUD_TOP), (0.0, STUD_TOP)]
    meshes.append(kit.lathe(prof, STEEL, n=28))
    # yellow paint dot on the stud's end at its thread start (toward the robot, -x), as a fitter would mark it
    meshes.append(kit.cylinder((-0.008, 0.0, STUD_TOP + 0.0002), 0.0025, 0.0004, YELLOW, n=12))
    cols = (kit.Collider((-sx, -sy, PLATE[2] / 2), size=PLATE),
            kit.Collider((0, 0, PLATE[2] + WASHER_T / 2), radius=WASHER_R, height=WASHER_T))
    return meshes, cols


def _nut():
    """The nut, in its own frame (origin on the axis at its underside). Hex with chamfered corners and a yellow paint
    dot at its thread start on the top face (half a turn from the stud's), so the turning shows."""
    rc = NUT_AF / math.sqrt(3)  # corner radius of the hex
    prof = [(0.0185, 0.0), (rc - 0.002, 0.0), (rc, 0.002), (rc, NUT_H - 0.002), (rc - 0.002, NUT_H),
            (0.0185, NUT_H), (0.0185, 0.0)]
    meshes = [kit.lathe(prof, NUT_LOOK, n=6)]
    meshes.append(kit.cylinder((0.024, 0.0, NUT_H + 0.0002), 0.003, 0.0004, YELLOW, n=12))
    meshes.append(kit.box((0.0, NUT_AF / 2 + 0.0002, NUT_H / 2), (0.003, 0.0008, NUT_H - 0.006), WHITE))  # on a flat
    cols = (kit.Collider((0, 0, NUT_H / 2), radius=NUT_AF / 2, height=NUT_H),)
    return tk.Cap("nut", anchor=(0.0, 0.0, SEAT), mass=NUT_MASS, meshes=tuple(meshes), colliders=cols,
                  grip=(0.0, 0.0, NUT_H / 2), width=NUT_AF)


def _joint():
    meshes, cols = _stud()
    return tk.Threaded("stud", pos=(STUD_XY[0], STUD_XY[1], TOP), size=(PLATE[0], PLATE[1], STUD_TOP),
                       meshes=tuple(meshes), colliders=cols, cap=_nut(), thread=THREAD)


OBJECTS = [_joint()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here (twistkit keeps the thread's: drop, clicks, cross-threading). yamkit counts the simulator's scene resets
# (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.home = None
        self.last_grip = None
        self.grips_still = False


_H = _History()


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


def _held(objects):
    """A gripper with its fingers under 75 % open and its tool point within 2 cm of the nut's outside (from its
    underside to its top). Wider than twistkit's `held`, so a loosely closed hand still on the nut counts as holding."""
    base = np.asarray(objects[NUT].pos, float)
    for g in ("left_gripper", "right_gripper"):
        if g not in objects or kit.opening(objects, g) >= HOLD_OPEN:
            continue
        d = np.asarray(objects[g].pos, float) - base
        if -HOLD_MARGIN <= d[2] <= NUT_H + HOLD_MARGIN and math.hypot(d[0], d[1]) < NUT_AF / 2 + HOLD_MARGIN:
            return True
    return False


def _arms_home(objects):
    """Each gripper's tool point within 3 cm of where it was at the trial's first scored step, and still."""
    if not _H.home or not _H.grips_still:
        return False
    return all(np.linalg.norm(np.asarray(objects[g].pos, float) - p) < HOME_TOL for g, p in _H.home.items()
               if g in objects)


def _score(objects):
    # forced forward 60 degrees or more before it dropped onto the thread: cross-threaded, ruined (irreversible)
    if tk.cross_threaded(objects, NUT):
        return 0.0
    # never dropped onto the thread: not started
    if not tk.engaged(objects, NUT):
        return 0.0
    # turned by something other than a gripper turning with it (knocked, swiped, pushed round by the other arm)
    if (tk.unheld_deg(objects, NUT, tightening=True) > GRIP_SLACK
            or tk.unheld_deg(objects, NUT, tightening=False) > GRIP_SLACK):
        return 0.0
    # still holding the nut: the job isn't finished
    if _held(objects):
        return 0.0
    run = tk.run_turns(objects, NUT)
    if run < PART_TURNS:
        return 0.25  # dropped onto the thread, but run down less than a full turn
    if run < RUN_TURNS:
        return 0.5  # started true and run down a turn, but short of two
    return 1.0 if (_arms_home(objects) or not REQUIRE_ARMS_HOME) else 0.9


def success(objects):
    """Full score: the nut dropped onto the thread, run down at least two turns by hand, let go, and (with
    REQUIRE_ARMS_HOME) both arms home and still. Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if at any point the nut was turned forward (clockwise seen from above) 60 degrees or more from where it sat
    before it had dropped onto the thread (cross-threaded; irreversible). 0 if it never dropped onto the thread (it
    drops after 180 degrees backward). 0 if more than 60 degrees of its turning in either direction was done without a
    gripper holding it and turning with it (twistkit's roll budget). 0 if a gripper still holds it. Otherwise, counting
    the turns it has run down from where it dropped: 0.25 under 1 turn, 0.5 from 1 to under 2, and 1.0 at 2 or more
    with both arms home and still (0.9 if not, with REQUIRE_ARMS_HOME).
    Possible scores: 0, 0.25, 0.5, 0.9, 1."""
    return _score(objects)


# --- Demo ----------------------------------------------------------------------------------------
GRIP_Z = TOP + SEAT + THREAD.pitch * PERCH_TURNS + NUT_H / 2  # tool point at the perched nut's middle
BACK = 240.0  # backward past the drop at 180 degrees (the slip between strokes takes some)
FORWARD = 900.0  # forward: 2.5 turns, for the two needed plus the slip
# Strokes, totals and start yaws are multiples of 60 degrees, so each regrip lands the pads on a pair of flats as on a
# real hex nut. A waypoint yaw of 0 closes the fingers along world -y, and the nut's flats face 30 + 60k degrees, so
# yaws that are multiples of 60 put the pads square on two flats. The left arm rolls -155 to 80 degrees here.
STROKE = 120.0
BACK_YAW, FORWARD_YAW = -120.0, 60.0


def demo():
    """Left arm: grip the perched nut, turn it backward (counterclockwise seen from above) 240 degrees in two
    120-degree strokes so it drops onto the thread, then turn it forward with a regrasp loop (grip, roll the wrist 120
    degrees, let go, roll back, repeat; 900 degrees of roll in all), riding down the stud, let go, lift clear and go
    home. The right arm stays home."""
    left = tk.twist_waypoints(STUD_XY, GRIP_Z, BACK, side="left", start_yaw=BACK_YAW, stroke=STROKE,
                              pitch=THREAD.pitch, label="turn back to the drop")
    z_on = GRIP_Z + THREAD.pitch * (BACK - 180.0) / 360.0
    left += tk.twist_waypoints(STUD_XY, z_on, -FORWARD, side="left", start_yaw=FORWARD_YAW, stroke=STROKE,
                               pitch=THREAD.pitch, label="run it down")
    left += [kit.Home(2.0), kit.Home(2.0, label="hold at home")]
    total = sum(w.duration for w in left)
    right = [kit.Home(total, label="stay home")]
    return {"left": left, "right": right}

