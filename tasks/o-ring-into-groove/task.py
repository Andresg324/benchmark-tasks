"""Fit an O-ring into its groove on a hydraulic O-ring boss (ORB) plug, stretched over the thread by hand.

A zinc-plated steel SAE -24 straight-thread O-ring boss plug (1-7/8-12 UN thread, 2-1/4 inch hex) stands hex-down in
a bench holder. Its O-ring groove (the undercut between the thread and the hex) takes an AS568-924 blue 90-durometer
fluorocarbon (FKM) O-ring, which lies flat on the plug's end face.

The job is a hydraulic fitter's: put two fingertips into the ring, stretch it just enough to pass over the thread,
carry it down square to the groove against the hex, draw the fingers out so it closes into the groove, and leave it
seated all the way round. Stretching it past half again its size (50%, the installation limit: half the 100%
minimum elongation of the 90-durometer FKM) thins its section for good, so it no longer seals.
Pushing it down the thread instead of stretching it rolls it: its section twists, and a ring left twisted in the
groove spirals and leaks. Letting it go tilted leaves one side on the thread, where it is cut when the plug is
screwed into its port.

The ring is modeled by ringkit (see ringkit.py, task copy): its position along the plug, turn and stretch are joints
of the holder, driven each step from the gripper's fingertips (rk.FingerTips); it is drawn with markers.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import ringkit as rk  # noqa: E402  (the ring on the plug; imports yamkit)
import yamkit as kit  # noqa: E402  (spawns the modeled props and the moving parts; nothing on disk changes)
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Fit the O-ring into the groove at the base of the plug's thread."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# --- The plug and its O-ring (m). SAE J1926 -24 port plug; AS568-924 O-ring (the -24 boss size).
SX, SY = 0.34, -0.10  # the plug's axis on the table
HOLDER_H = 0.040  # the bench holder's top above the table
HEX_AF, HEX_H = 0.05715, 0.014  # 2-1/4 inch across flats, 14 mm tall
HEX_TOP = HOLDER_H + HEX_H  # the hex face: the groove's lower wall
GROOVE_W = 0.0036  # the undercut's width along the plug
THREAD_LEN = 0.019
END_H = HEX_TOP + GROOVE_W + THREAD_LEN  # the plug's end face above the table (7.66 cm)
THREAD_D = 0.04763  # 1-7/8-12 UN major diameter
ROOT_D = 0.0453  # the thread's root (12 threads per inch: 2.1 mm pitch)
GROOVE_D = 0.0442  # the undercut's diameter: the ring sits on it stretched 1.2%
FREE_ID = 0.04369  # AS568-924: 43.69 mm inner diameter
RING_CS = 0.0030  # its section (3.00 mm)
STOP = END_H - HEX_TOP  # the hex face, m below the end face
GROOVE_AT = STOP - GROOVE_W / 2  # the groove's middle, below the end face

# --- Scoring thresholds (each with its reason).
LIMIT_ID = 1.5 * FREE_ID  # 65.5 mm: 50% stretch, the installation limit for O-rings (half the 100% minimum ultimate
#                          elongation of the 90-durometer FKM, ASTM D2000 M2HK910, used for O-ring boss seals);
#                          stretched further, the section thins for good and no longer fills the groove
CLEAR = 0.0002  # m (diametral): the stretched ring must clear the 47.6 mm thread by 0.2 mm to slide over it
TILT_LIMIT = 4.0  # deg, atan(fingertips' height difference / their spacing) when let go: tilted more than this,
#                   the 50 mm ring's far side sits 3.5 mm off the groove plane, more than its 3.0 mm section, so that
#                   side lands on the thread instead of in the groove
TWIST_LIMIT = 90.0  # deg of roll left in the ring's section: a quarter turn, 2.4 mm of rolling on the thread for its
#                    3.0 mm section; the mold parting line, which a fitter checks runs round the ring's outside, is
#                    then on its face for all to see (it spirals)
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving
GRIPPERS = ("left_gripper", "right_gripper")

ZINC = kit.Look((0.78, 0.79, 0.80), 0.35, metallic=0.8)
HOLDER_LOOK = kit.Look((0.20, 0.32, 0.55), 0.5, metallic=0.3)  # blue anodized aluminum
FKM = kit.Look((0.05, 0.30, 0.95), 0.8)  # matte bright blue fluorocarbon: shows against the zinc and the table

RING = rk.Ring("oring", end=(0.0, 0.0, END_H), axis="-Z", shaft_d=THREAD_D, free_id=FREE_ID, thickness=RING_CS,
               width=RING_CS, length=STOP, grooves=(rk.Groove(at=GROOVE_AT, width=GROOVE_W, diameter=GROOVE_D),),
               stop=STOP, limit_id=LIMIT_ID, clear=CLEAR, tilt_limit=TILT_LIMIT, roll=True, look=FKM,
               hide=END_H + 0.01)  # placeholder put away inside the table


def _holder():
    meshes = [kit.box((0, 0, HOLDER_H / 2), (0.09, 0.09, HOLDER_H), HOLDER_LOOK),
              kit.cylinder((0, 0, HOLDER_H + HEX_H / 2), HEX_AF / 2 / math.cos(math.pi / 6), HEX_H, ZINC, n=6)]
    # the plug above the hex: the undercut, then the thread (12 per inch) up to a 1.5 mm chamfer at the end
    prof = [(GROOVE_D / 2, HEX_TOP), (GROOVE_D / 2, HEX_TOP + GROOVE_W)]
    pitch = 0.0254 / 12
    z = HEX_TOP + GROOVE_W
    while z + pitch < END_H - 0.0015:
        prof += [(ROOT_D / 2, z + 0.0002), (THREAD_D / 2, z + pitch / 2), (ROOT_D / 2, z + pitch - 0.0002)]
        z += pitch
    prof += [(THREAD_D / 2, END_H - 0.0015), (THREAD_D / 2 - 0.0015, END_H), (0.0, END_H)]
    meshes.append(kit.lathe(prof, ZINC, n=64))
    colliders = [kit.Collider((0, 0, HOLDER_H / 2), size=(0.09, 0.09, HOLDER_H)),
                 kit.Collider((0, 0, HOLDER_H + HEX_H / 2), radius=HEX_AF / 2, height=HEX_H),
                 kit.Collider((0, 0, (HEX_TOP + END_H) / 2), radius=THREAD_D / 2, height=END_H - HEX_TOP)]
    return kit.Mechanism("holder", pos=(SX, SY, TOP), size=(0.09, 0.09, END_H), meshes=tuple(meshes),
                         colliders=tuple(colliders), links=rk.links(RING))


FINGERS = rk.FingerTips(grippers=("left_gripper", "right_gripper"), full_gap=2 * abs(FINGER_OPEN))
KEY = rk.attach("holder", RING, FINGERS)

OBJECTS = [_holder()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
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


def _job(objects):
    if rk.overspread(objects, KEY):
        return 0.0  # stretched past 50% at some point: its section has thinned for good (irreversible)
    if not rk.seated(objects, KEY):
        return 0.0  # not closed into the groove (on the thread or the end face, landed tilted) or fingers still in it
    if rk.twist_deg(objects, KEY) > TWIST_LIMIT:
        return 0.0  # seated, but rolled on: its section is left twisted in the groove
    return 1.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """The ring seated square and untwisted, nothing touching it, and (with REQUIRE_ARMS_HOME) both arms home and
    still. Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if the ring was ever stretched past 65.5 mm bore (50%; irreversible). 0 unless it is seated: in the groove,
    closed onto its 44.2 mm bottom, not landed tilted more than 4 degrees, and no fingertip in it or pressing on it.
    0 if it is left with more than 90 degrees of twist (rolled on instead of stretched over). Otherwise 1.0 if both
    arms are also back home and still (REQUIRE_ARMS_HOME), else 0.9. Possible scores: 0, 0.9, 1."""
    return _score(objects)


# --- Demo (right arm). Finger targets are half the gap between the pads (FINGER_OPEN = 94 mm open).
def _finger(gap):
    return -gap / 2


F_IN = _finger(0.008)  # fingers nearly closed: they fit in the 43.7 mm ring with room to spare
F_STRETCH = _finger(0.057)  # pads 57 mm apart: 4.7 mm clear of the thread each side; the ring stretched 28.7%
TIP = 0.0128  # the pad tips below the tool point in a straight-down grip


def demo():
    """Right arm: fingertips into the ring on the plug's end, open them to stretch it over the thread, carry it down
    to the hex face, draw the fingers straight up out of it so it closes into the groove, then home. Left arm stays
    home."""
    end = TOP + END_H
    hexz = TOP + HEX_TOP

    def at(tip_z):
        return (SX, SY, tip_z + TIP)

    right = [Waypoint(2.0, at(end + 0.06), F_IN, "over the plug, fingers closed"),
             Waypoint(1.5, at(end + 0.012), F_IN, "down to the ring"),
             Waypoint(1.0, at(end + 0.001), F_IN, "fingertips into the ring"),
             Waypoint(2.5, at(end + 0.001), F_STRETCH, "stretch it over the thread"),
             Waypoint(3.0, at(hexz + 0.0003), F_STRETCH, "carry it down to the hex"),
             Waypoint(1.0, at(hexz + 0.0003), F_STRETCH, "square on the hex face"),
             Waypoint(2.0, at(end + 0.02), F_STRETCH, "fingers out: it closes into the groove"),
             Waypoint(1.0, at(end + 0.08), F_STRETCH, "clear of the plug"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    total = sum(w.duration for w in right)
    left = [kit.Home(total - 1.5), kit.Home(1.5, label="hold at home")]
    return {"left": left, "right": right}
