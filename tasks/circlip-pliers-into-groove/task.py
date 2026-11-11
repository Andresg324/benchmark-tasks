"""Fit an external retaining ring (circlip, DIN 471 for a 40 mm shaft) into its groove with circlip pliers.

A steel shaft stands upright in a bench fixture with a spur gear on it; the groove just above the gear takes the
retaining ring that keeps the gear on. The yellow zinc-plated spring-steel ring lies on the shaft's end face and the
circlip pliers (90-degree bent tips, external type: squeezing the handles spreads the tips) lie on a rubber rest block.

The job is a mechanic's: put the pliers' tips in the ring's two lug holes, spread the ring just enough to clear the
shaft, slide it down to the groove, let it close into the groove, turn it in the groove to check it is seated, and
put the pliers down. A spring-steel ring spread past its elastic limit (the ring maker's maximum installation
diameter) takes a permanent set: it sits loose in its groove and can walk out under load, so it is scrap even if it
looks seated. A ring let go short of the groove grips the shaft in the wrong place, and one let go tilted lands half
in the groove (cocked).

The ring is modeled by ringkit (see ringkit.py): its axial position, turn and opening are joints of the fixture,
driven each step from the pliers' tips; the pliers are a loose articulated tool whose handles the gripper squeezes.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import ringkit as rk  # noqa: E402  (the ring on the shaft; imports yamkit)
import tracekit as tk  # noqa: E402  (per-trial, once-per-step memory for the debounced twist latch)
import yamkit as kit  # noqa: E402  (spawns the modeled props and the moving parts; nothing on disk changes)
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.station import TABLE_CENTER, TABLE_SIZE  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Fit the retaining ring into the groove on the shaft with the circlip pliers, then put the pliers down."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# --- The shaft and its ring (m). DIN 471 external retaining ring for a 40 mm shaft.
SX, SY = 0.36, -0.10  # the shaft's axis on the table
END_H = 0.105  # the shaft's end face above the table
SHAFT_D = 0.040
FREE_ID = 0.0365  # the ring's inner diameter at rest (DIN 471, d3 for a 40 mm shaft)
GROOVE_D = 0.0375  # groove bottom diameter (DIN 471, d2)
GROOVE_W = 0.00185  # groove width (DIN 471, m)
RING_T = 0.00175  # ring thickness (DIN 471, s)
RING_W = 0.0045  # the ring's radial section
GEAR_FACE = 0.03903  # the gear's top face, m below the shaft's end face: the groove's lower wall sits on it
GROOVE_AT = GEAR_FACE - GROOVE_W / 2  # the groove's middle, below the end face
LUG_GAP = 0.012  # the ring's lug holes, center to center, at rest
LUG_OUT = 0.0058  # the lug holes' centers beyond the ring's inner edge

# --- Scoring thresholds (each with its reason).
LIMIT_ID = 0.0410  # m: the largest bore the ring may be spread to while fitting it, 1 mm (2.5%) over the shaft, the
#                   usual maximum installation spread for a 40 mm DIN 471 ring; past it the spring steel takes a set
CLEAR = 0.0002  # m (diametral): the spread ring must clear the 40 mm shaft by 0.2 mm to slide along it
TILT_LIMIT = 4.0  # deg: let go tilted more than this, a 49 mm-wide ring's far side sits 3.4 mm off the groove plane,
#                   nearly twice the 1.85 mm groove width, so that side closes onto the shaft beside the groove
TWIST_STEPS = 2  # control steps in a row (0.1 s) past TWIST_LIMIT for the twist to count: a one-step PhysX pop of the
#                 pliers doesn't scrap the ring
TWIST_LIMIT = 10.0  # deg: ends twisted apart along the shaft by more than this (2.1 mm across the 12 mm lug spacing)
#                    bend the ring out of its plane for good (ring makers: never twist a retaining ring)
CHECK_DEG = 15.0  # deg: turning the ring this far in its groove (5 mm at its rim) is the check that it is free all
#                   round in the groove, i.e. seated, not caught on the shaft or cocked
HOLD_R = 0.03  # m: a closed gripper's tool point within 3 cm of the pliers' center line (handle butt to tips) holds
#               them: the 1.2 cm handles plus the ~1.8 cm a held object rides off the reported tool point
DOWN_Z = 0.05  # m: the pliers' grip point within 5 cm of the table (on the 3 cm rest block or the bench) = put down
DOWN_LOW = 0.01  # m: and no more than 1 cm below the table top (the handles are 1.2 cm tall): lower is on the floor
TABLE_X = (TABLE_CENTER[0] - TABLE_SIZE[0] / 2, TABLE_CENTER[0] + TABLE_SIZE[0] / 2)  # the bench's footprint
TABLE_Y = (TABLE_CENTER[1] - TABLE_SIZE[1] / 2, TABLE_CENTER[1] + TABLE_SIZE[1] / 2)
STILL = 0.001  # m per control step: the pliers not moving
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving
GRIPPERS = ("left_gripper", "right_gripper")

STEEL = kit.Look((0.70, 0.71, 0.72), 0.35, metallic=0.8)
BRIGHT = kit.Look((0.80, 0.80, 0.81), 0.25, metallic=0.9)
CAST = kit.Look((0.30, 0.33, 0.37), 0.6, metallic=0.3)
GEAR = kit.Look((0.42, 0.44, 0.47), 0.5, metallic=0.5)  # hobbed steel, darker than the ground shaft
RING_LOOK = kit.Look((0.95, 0.74, 0.08), 0.45, metallic=0.2)  # yellow zinc-chromate plating, a common ring finish;
#                                                             bright so the ring shows against the steel at 320 x 176
GRIP = kit.Look((0.80, 0.10, 0.08), 0.6)
JAW = kit.Look((0.25, 0.26, 0.28), 0.4, metallic=0.7)
RUBBER = kit.Look((0.12, 0.12, 0.12), 0.9)

RING = rk.Ring("clip", end=(0.0, 0.0, END_H), axis="-Z", shaft_d=SHAFT_D, free_id=FREE_ID, thickness=RING_T,
               width=RING_W, length=END_H - 0.05,
               grooves=(rk.Groove(at=GROOVE_AT, width=GROOVE_W, diameter=GROOVE_D),), stop=GEAR_FACE,
               limit_id=LIMIT_ID, clear=CLEAR, lug_gap=LUG_GAP, lug_out=LUG_OUT, lug_angle=math.pi,
               tilt_limit=TILT_LIMIT, look=RING_LOOK, hide=END_H + 0.02)  # placeholder put away inside the table


def _fixture():
    gz = END_H - GEAR_FACE  # the gear's top face above the table
    meshes = [kit.box((0, 0, 0.006), (0.12, 0.12, 0.012), CAST),  # base plate
              kit.cylinder((0, 0, 0.031), 0.032, 0.038, CAST)]  # bearing housing
    # the shaft, turned: the groove just above the gear and a small chamfer at the end
    g0, g1 = gz, gz + GROOVE_W
    meshes.append(kit.lathe([(SHAFT_D / 2, 0.050), (SHAFT_D / 2, g0), (GROOVE_D / 2, g0), (GROOVE_D / 2, g1),
                             (SHAFT_D / 2, g1), (SHAFT_D / 2, END_H - 0.0015), (SHAFT_D / 2 - 0.0015, END_H),
                             (0.0, END_H)], BRIGHT, n=48))
    meshes.append(kit.cylinder((0, 0, (0.050 + gz) / 2), 0.026, gz - 0.050, GEAR, n=48))  # the gear's body
    for k in range(30):  # its teeth
        a = 2 * math.pi * k / 30
        meshes.append(kit.cylinder((0.0262 * math.cos(a), 0.0262 * math.sin(a), (0.050 + gz) / 2), 0.0019,
                                   gz - 0.0505, GEAR, n=10))
    colliders = [kit.Collider((0, 0, 0.006), size=(0.12, 0.12, 0.012)),
                 kit.Collider((0, 0, 0.031), radius=0.032, height=0.038),
                 kit.Collider((0, 0, (0.050 + gz) / 2), radius=0.028, height=gz - 0.050),
                 kit.Collider((0, 0, (gz + END_H) / 2), radius=SHAFT_D / 2, height=END_H - gz)]
    return kit.Mechanism("fixture", pos=(SX, SY, TOP), size=(0.12, 0.12, END_H), meshes=tuple(meshes),
                         colliders=tuple(colliders), links=rk.links(RING))


# --- The circlip pliers (m), in their own frame: origin at the grip point between the handles at mid-height, x toward
# the tips. The base half carries handle A (+y) and the jaw that crosses to tip A; the other half (link "jaw") slides
# along y: squeezing the handles together by q spreads the tips apart by q (handle gap 40 mm - q, tip gap 12 mm + q).
TIP_X = 0.075  # the tips' distance from the grip point
TIP_LEN = 0.017  # the bent tips reach this far below the jaws' underside
TIP_END = -0.004 - TIP_LEN  # z of the tip ends in the pliers' frame
HANDLE_GAP = 0.040  # outer faces of the handles at rest
SQUEEZE_MAX = 0.018  # the handles' stop
PLIERS_MASS, JAW_MASS = 0.08, 0.05
SPRING_K, SPRING_PRE = 100.0, 1.0  # N/m and N: the handles' return spring (1 N at rest, 2.8 N fully squeezed)


def _jaw_path(sign):
    """A jaw's center line from its handle (y = sign * 15 mm) across to its tip (y = -sign * 6 mm)."""
    return [(0.030, sign * 0.015, 0.0), (0.045, sign * 0.012, 0.0), (0.060, -sign * 0.002, 0.0),
            (TIP_X, -sign * 0.006, 0.0)]


def _pliers():
    hz = 0.012  # the handles' height
    tip_a = (TIP_X, -0.006, -0.004 - TIP_LEN / 2)
    base_meshes = (kit.box((-0.005, 0.015, 0.0), (0.080, 0.010, hz), GRIP),
                   kit.band([(x, y, z - 0.0022) for x, y, z in _jaw_path(1)], 0.0044, 0.005, JAW, normal=(0, 0, 1)),
                   kit.cylinder(tip_a, 0.0013, TIP_LEN, JAW),
                   kit.cylinder((0.0525, 0.0045, 0.0), 0.0035, 0.010, BRIGHT))  # the pivot rivet
    jaw_meshes = (kit.box((-0.005, -0.015, 0.0), (0.080, 0.010, hz), GRIP),
                  kit.band([(x, y, z + 0.0022) for x, y, z in _jaw_path(-1)], 0.0044, 0.005, JAW, normal=(0, 0, 1)),
                  kit.cylinder((TIP_X, 0.006, -0.004 - TIP_LEN / 2), 0.0013, TIP_LEN, JAW))
    jaw = kit.Link("jaw", "prismatic", "Y", (0.0, 0.0, 0.0), lower=0.0, upper=SQUEEZE_MAX, start=0.0, mass=JAW_MASS,
                   meshes=jaw_meshes, colliders=(kit.Collider((-0.005, -0.015, 0.0), size=(0.080, 0.010, hz)),),
                   damping=0.0, law=rk.HandleSpring(rest=-SPRING_PRE / SPRING_K, k=SPRING_K, damp=1.0))
    return rk.free_tool(kit.Mechanism(
        "pliers", pos=PLIERS_POS, size=(0.125, 0.04, 0.03), mass=PLIERS_MASS, meshes=base_meshes,
        colliders=(kit.Collider((-0.005, 0.015, 0.0), size=(0.080, 0.010, hz)),),
        links=(jaw,)), com=(0.0, 0.0, 0.0))  # weight at the grip point, so the pliers don't pitch in the gripper


REST = (0.256, -0.22, 0.015)  # the rubber rest block's center (8 x 3 x 3 cm)
PLIERS_POS = (REST[0] + 0.005, REST[1], TOP + 0.030 + 0.006)  # lying on the block, tips hanging past its end
TIPS = rk.TipPair(("pliers", (TIP_X, -0.006, TIP_END)), ("pliers/jaw", (TIP_X, 0.006, TIP_END)), catch=0.004)
KEY = rk.attach("fixture", RING, TIPS)

OBJECTS = [_fixture(), _pliers(),
           kit.Prop("rest", pos=(REST[0], REST[1], TOP + REST[2]), size=(0.08, 0.03, 0.03),
                    meshes=(kit.box((0, 0, 0), (0.08, 0.03, 0.03), RUBBER),),
                    colliders=(kit.Collider((0, 0, 0), size=(0.08, 0.03, 0.03)),))]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.home = None  # each gripper's tool point at the trial's first scored step
        self.last = None
        self.still = False
        self.pliers = None  # the pliers' grip point at the last step
        self.pliers_still = False


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
    p = np.asarray(objects["pliers"].pos, float)
    _H.pliers_still = _H.pliers is not None and np.linalg.norm(p - _H.pliers) < STILL
    _H.pliers = p


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still."""
    if _H.home is None or not _H.still:
        return False
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - _H.home[g]) < HOME_TOL for g in GRIPPERS)


HANDLE_BUTT = -0.045  # x of the handles' far end in the pliers' frame (the tips are at TIP_X)


def _seg_dist(p, a, b):
    ab = b - a
    f = min(max(float(np.dot(p - a, ab) / np.dot(ab, ab)), 0.0), 1.0)
    return float(np.linalg.norm(p - (a + f * ab)))


def _pliers_held(objects):
    """A gripper with its fingers closed to the handles' width or less and its tool point within 3 cm of the pliers'
    center line anywhere from the handle butt to the tips (an open hand hovering there does not hold them)."""
    o = objects["pliers"]
    p, R = np.asarray(o.pos, float), rk._rotm(o.quat_xyzw)
    a, b = p + R @ np.array([HANDLE_BUTT, 0.0, 0.0]), p + R @ np.array([TIP_X, 0.0, 0.0])
    shut = (HANDLE_GAP + 0.004) / (2 * abs(FINGER_OPEN))
    return any(g in objects and _seg_dist(np.asarray(objects[g].pos, float), a, b) < HOLD_R
               and kit.opening(objects, g) < shut for g in GRIPPERS)


def _pliers_down(objects):
    """Put down: out of the ring, nobody holding them, lying still over the bench with the grip point from 1 cm below
    to 5 cm above its top (not dropped on the floor)."""
    x, y, z = (float(v) for v in objects["pliers"].pos)
    return (not rk.engaged(objects, KEY) and not _pliers_held(objects) and _H.pliers_still
            and -DOWN_LOW <= z - TOP < DOWN_Z and TABLE_X[0] < x < TABLE_X[1] and TABLE_Y[0] < y < TABLE_Y[1])


TRACE = tk.Trace()


def _watch(objects):
    """Once per control step (success() and score() both call it): the ring's ends twisted past the limit."""
    if TRACE.new_step(objects):
        TRACE.track("twisted", abs(rk.state(KEY).twist) > TWIST_LIMIT, steps=TWIST_STEPS)


def _job(objects):
    if rk.overspread(objects, KEY) or TRACE.ever("twisted"):
        return 0.0  # the ring has taken a set: scrap, whatever happened next
    if not rk.seated(objects, KEY) or not _pliers_down(objects):
        return 0.0  # not in the groove (short of it, cocked, still spread or held), or the pliers still in hand
    if rk.turned_deg(objects, KEY) < CHECK_DEG - 1e-6:
        return 0.5  # seated, but never turned in its groove to check it
    return 1.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """The ring seated and checked, the pliers put down and (with REQUIRE_ARMS_HOME) both arms home and still. Called
    first each step, so it also records the history."""
    _update(objects)
    _watch(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if the ring was ever spread past 41.0 mm bore or its ends twisted apart more than 10 degrees for 2 control
    steps in a row (it has taken a set; irreversible). 0 unless the ring is seated: in the groove, closed onto its 37.5 mm bottom, not cocked
    (let go tilted more than 4 degrees) and out of the pliers; and 0 unless the pliers are put down (out of the
    ring, no gripper holding them, still, over the bench with the grip point from 1 cm below to 5 cm above its top).
    Then 1.0 if the ring's net turn in its groove (the span of angles it reached there, so wiggles count only their
    width) is at least 15 degrees, else 0.5. With REQUIRE_ARMS_HOME, a full job with the arms not back
    home and still scores 0.9. Possible scores: 0, 0.5, 0.9, 1."""
    _watch(objects)
    return _score(objects)


# --- Demo (right arm). Finger targets are half the gap between the pads (FINGER_OPEN = 94 mm open).
def _finger(gap):
    return -gap / 2


F_HOLD = _finger(0.037)  # a light grip on the handles (3 mm squeeze: tips 15 mm apart)
F_SPREAD = _finger(0.0262)  # squeezed to spread the ring to about 40.6 mm bore (handles' spring takes ~1 mm)
LUG_R = FREE_ID / 2 + LUG_OUT  # the lug holes' distance from the shaft's axis


def demo():
    """Right arm: pick the pliers off the rest, put the tips in the ring's lug holes, squeeze to spread it to about
    40.6 mm, slide it down onto the gear face, let it close into the groove, turn it 15 degrees each way in the
    groove, lift the pliers off, lay them back on the rest, home. Left arm stays home."""
    px, py, pz = PLIERS_POS[0], PLIERS_POS[1], PLIERS_POS[2]
    ex, ey = SX - LUG_R - TIP_X, SY  # grip point with the tips over the lug holes
    ez = TOP + END_H + 0.0003 - TIP_END  # tips 1.45 mm into the holes of the ring lying on the end face
    sz = TOP + END_H - (GEAR_FACE + 0.0005) - TIP_END  # tips 0.5 mm past the gear face: the ring's far face on it

    def orbit(deg, z, finger, label, t):  # the grip point turned about the shaft's axis, the wrist with it
        a = math.radians(deg)
        dx, dy = ex - SX, ey - SY
        return Waypoint(t, (SX + dx * math.cos(a) - dy * math.sin(a), SY + dx * math.sin(a) + dy * math.cos(a), z),
                        finger, label, a)

    right = [Waypoint(1.5, (px, py, pz + 0.08), FINGER_OPEN, "over the pliers"),
             Waypoint(1.5, (px, py, pz), FINGER_OPEN, "around the handles"),
             Waypoint(1.0, (px, py, pz), F_HOLD, "grip the handles"),
             Waypoint(1.5, (px, py, pz + 0.11), F_HOLD, "lift the pliers"),
             Waypoint(2.0, (ex, ey, ez + 0.02), F_HOLD, "over the ring's lug holes"),
             Waypoint(1.5, (ex, ey, ez), F_HOLD, "tips into the lug holes"),
             Waypoint(1.5, (ex, ey, ez), F_SPREAD, "spread the ring"),
             Waypoint(4.0, (ex, ey, sz), F_SPREAD, "slide it down onto the gear"),
             Waypoint(1.0, (ex, ey, sz), F_HOLD, "let it close into the groove"),
             orbit(8, sz, F_HOLD, "turn it in the groove", 1.0), orbit(15, sz, F_HOLD, "turn it in the groove", 1.0),
             orbit(0, sz, F_HOLD, "turn it back", 1.5), orbit(-15, sz, F_HOLD, "turn it the other way", 1.5),
             orbit(0, sz, F_HOLD, "turn it back", 1.5),
             Waypoint(1.0, (ex, ey, sz + 0.03), F_HOLD, "lift the tips out"),
             Waypoint(2.0, (px, py, pz + 0.08), F_HOLD, "over the rest"),
             Waypoint(1.5, (px, py, pz + 0.002), F_HOLD, "lay the pliers on the rest"),
             Waypoint(0.8, (px, py, pz + 0.002), FINGER_OPEN, "let go"),
             Waypoint(1.0, (px, py, pz + 0.08), FINGER_OPEN, "back away"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    total = sum(w.duration for w in right)
    left = [kit.Home(total - 1.5), kit.Home(1.5, label="hold at home")]
    return {"left": left, "right": right}
