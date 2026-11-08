"""A lab's hazardous-waste station at the end of the day. A white HDPE waste jug with a yellow hazardous-waste label
stands in a black secondary-containment tray just right of the middle of the bench. A gray waste funnel still stands
in the jug's neck, where it was left after the last waste was added, and the jug's red screw cap lies on the bench on
the robot's right. Beyond the cap, also on the right, is a blue drip tray with a funnel holder in it. Tidying the
station means taking the funnel out, standing it in its holder on the drip tray, and putting the cap back on the jug.
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
INSTRUCTION = "You're done adding waste for today. Tidy up the waste station."
# Arms home: when True, full credit also needs both arms back where they started, so the workspace is clear for the
# next person or job. The job done with the arms elsewhere then scores 0.9.
REQUIRE_ARMS_HOME = True

# The jug (fixed, standing in its containment tray): a square body and a round neck, open at the top.
JUG_X, JUG_Y = 0.32, -0.06
BODY, BODY_H = 0.11, 0.115  # square body, its height
NECK_OUT, NECK_IN, NECK_H = 0.032, 0.024, 0.020  # the neck's outer and inner width (its collider is a square ring)
NECK_TOP = TOP + BODY_H + NECK_H
CONT_IN, CONT_H, CONT_WALL = 0.16, 0.03, 0.005  # the containment tray around the jug

# The funnel (free). Its origin is the center of the cone's flat base; the stem hangs below it, the cone and a molded
# grip tab rise above it. Standing in the neck, the cone's base rests on the neck's rim.
STEM_W, STEM_L = 0.014, 0.018
BASE_W, BASE_T = 0.042, 0.006
CONE_TOP_R, CONE_H = 0.040, 0.045
TAB = (0.030, 0.025, 0.025)  # x by y by z: the grip tab sticking out of the cone toward the robot (-x)
TAB_C = (-(CONE_TOP_R + TAB[0] / 2 + 0.008), 0.0, 0.030)  # clear of the cone so the fingers fit around it
FUNNEL_START = (JUG_X, JUG_Y, NECK_TOP + BASE_T / 2)

# The cap (free): a flat top and a skirt that goes down over the neck.
CAP_W, CAP_H, SKIRT_IN, SKIRT_T = 0.052, 0.025, 0.046, 0.003
TOP_T = 0.006
CAP_SEATED_Z = NECK_TOP - (CAP_H / 2 - TOP_T)  # origin height when the top's underside rests on the neck's rim
CAP_START = (0.215, -0.20, TOP + CAP_H / 2)

# The drip tray (fixed) with a funnel holder: a ring the stem drops into, on a post.
TRAY_X, TRAY_Y, TRAY_IN, TRAY_H, TRAY_WALL = 0.345, -0.225, (0.15, 0.14), 0.015, 0.005
HOLDER_TOP = TOP + 0.05
HOLDER_OUT, HOLDER_IN = 0.034, 0.024

# Tolerances.
FUNNEL_IN_R = 0.025  # funnel origin within this of the neck's axis ...
FUNNEL_IN_Z = 0.03  # ... and below the neck's top plus this: still in the neck
CAP_R, CAP_DZ, CAP_TILT = 0.009, 0.004, math.radians(10)
HOLDER_R, HOLDER_DZ, HOLDER_TILT = 0.008, 0.006, math.radians(20)  # funnel seated in its holder
LYING_DZ = 0.012  # a funnel lying down: its lowest point within this of the floor it lies on
WALL_MARGIN = 0.003  # ... and its whole outline inside the drip tray's walls
HELD_OPEN = 0.6  # a gripper closed past this ...
HELD_R = 0.04  # ... with its tool point this close to the cap or the funnel's tab is holding it
HOME_R = 0.03  # each gripper's tool point within this of where it was at the trial's first scored step
HOME_STILL = 0.002  # ... and moving less than this per control step (4 cm/s)
GRIPPERS = ("left_gripper", "right_gripper")

# -- looks and models -------------------------------------------------------------------------

HDPE = kit.Look((0.93, 0.93, 0.89), 0.55)
LABEL_Y = kit.Look((1.0, 0.83, 0.10), 0.6)
LABEL_R = kit.Look((0.80, 0.08, 0.06), 0.6)
LABEL_W = kit.Look((0.97, 0.97, 0.95), 0.7)
BLACK = kit.Look((0.08, 0.08, 0.09), 0.7)
FUNNEL_LOOK = kit.Look((0.55, 0.57, 0.60), 0.45)
CAP_LOOK = kit.Look((0.78, 0.10, 0.08), 0.45)
TRAY_LOOK = kit.Look((0.20, 0.42, 0.78), 0.5)
STEEL = kit.Look((0.75, 0.76, 0.79), 0.3, metallic=1.0)


def _ring(cx, cy, z0, out_w, in_w, h):
    """A square ring of four boxes (outer width out_w, inner in_w, height h, bottom at z0), as (center, size) pairs."""
    t = (out_w - in_w) / 2
    zc = z0 + h / 2
    return [((cx + (in_w + t) / 2, cy, zc), (t, out_w, h)), ((cx - (in_w + t) / 2, cy, zc), (t, out_w, h)),
            ((cx, cy + (in_w + t) / 2, zc), (in_w, t, h)), ((cx, cy - (in_w + t) / 2, zc), (in_w, t, h))]


def _jug():
    """Fixed. Origin on the table under the jug's center."""
    neck = _ring(0.0, 0.0, BODY_H, NECK_OUT, NECK_IN, NECK_H)
    meshes = [kit.box((0, 0, BODY_H / 2 - 0.004), (BODY, BODY, BODY_H - 0.008), HDPE),
              kit.box((0, 0, BODY_H - 0.004), (BODY - 0.016, BODY - 0.016, 0.008), HDPE),  # rounded shoulder
              kit.lathe([(NECK_IN / 2, BODY_H), (NECK_OUT / 2, BODY_H), (NECK_OUT / 2, BODY_H + NECK_H),
                         (NECK_IN / 2, BODY_H + NECK_H), (NECK_IN / 2, BODY_H + 0.002)], HDPE),
              # carry handle on the back corner
              kit.band([(0.035, 0.035, BODY_H - 0.002), (0.05, 0.05, BODY_H + 0.012), (0.056, 0.056, BODY_H - 0.03)],
                       0.012, 0.006, HDPE, normal=(1, -1, 0)),
              # hazardous-waste label on the face toward the robot
              kit.box((-BODY / 2 - 0.0006, 0, 0.06), (0.001, 0.08, 0.06), LABEL_Y),
              kit.box((-BODY / 2 - 0.0010, 0, 0.082), (0.001, 0.08, 0.012), LABEL_R),
              kit.box((-BODY / 2 - 0.0010, 0, 0.050), (0.001, 0.06, 0.025), LABEL_W)]
    cols = [kit.Collider((0, 0, BODY_H / 2), size=(BODY, BODY, BODY_H))] + [
        kit.Collider(c, size=s) for c, s in neck]
    return kit.Prop("jug", pos=(JUG_X, JUG_Y, TOP), size=(BODY, BODY, BODY_H + NECK_H), meshes=tuple(meshes),
                    colliders=tuple(cols))


def _containment():
    walls = _ring(0.0, 0.0, 0.0, CONT_IN + 2 * CONT_WALL, CONT_IN, CONT_H)
    w = CONT_IN + 2 * CONT_WALL
    meshes = [kit.box(c, s, BLACK) for c, s in walls] + [kit.box((0, 0, 0.0005), (w, w, 0.001), BLACK)]
    return kit.Prop("containment", pos=(JUG_X, JUG_Y, TOP), size=(w, w, CONT_H), meshes=tuple(meshes),
                    colliders=tuple(kit.Collider(c, size=s) for c, s in walls))


def _funnel():
    cone = [(STEM_W / 2, -STEM_L), (STEM_W / 2, 0.0), (BASE_W / 2, BASE_T / 2), (CONE_TOP_R, CONE_H),
            (CONE_TOP_R - 0.003, CONE_H), (BASE_W / 2 - 0.003, BASE_T / 2 + 0.002), (0.0, BASE_T / 2 + 0.002)]
    web_x0, web_x1 = -0.026, TAB_C[0] + TAB[0] / 2
    web = kit.box(((web_x0 + web_x1) / 2, 0.0, TAB_C[2] + TAB[2] / 2 - 0.003), (web_x0 - web_x1, 0.010, 0.006), FUNNEL_LOOK)
    meshes = (kit.lathe(cone, FUNNEL_LOOK, n=48), kit.box(TAB_C, TAB, FUNNEL_LOOK), web)  # the web joins tab and cone
    cols = (kit.Collider((0, 0, -STEM_L / 2), size=(STEM_W, STEM_W, STEM_L)),
            kit.Collider((0, 0, 0), size=(BASE_W, BASE_W, BASE_T)),
            kit.Collider((0, 0, 0.018), size=(0.056, 0.056, 0.024)),  # the cone, in two steps
            kit.Collider((0, 0, 0.038), size=(0.074, 0.074, 0.014)),
            kit.Collider(TAB_C, size=TAB))
    return kit.Prop("funnel", pos=FUNNEL_START, size=(2 * CONE_TOP_R, 2 * CONE_TOP_R + TAB[1], CONE_H + STEM_L),
                    mass=0.06, meshes=meshes, colliders=cols)


def _cap():
    z_top = CAP_H / 2 - TOP_T / 2
    skirt_h = CAP_H - TOP_T
    t = SKIRT_T
    skirt = [((SKIRT_IN / 2 + t / 2, 0, -TOP_T / 2), (t, SKIRT_IN + 2 * t, skirt_h)),
             ((-SKIRT_IN / 2 - t / 2, 0, -TOP_T / 2), (t, SKIRT_IN + 2 * t, skirt_h)),
             ((0, SKIRT_IN / 2 + t / 2, -TOP_T / 2), (SKIRT_IN, t, skirt_h)),
             ((0, -SKIRT_IN / 2 - t / 2, -TOP_T / 2), (SKIRT_IN, t, skirt_h))]
    meshes = (kit.lathe([(CAP_W / 2 - 0.001, -CAP_H / 2), (CAP_W / 2, -CAP_H / 2 + 0.002), (CAP_W / 2, CAP_H / 2 - 0.002),
                         (CAP_W / 2 - 0.002, CAP_H / 2), (0.0, CAP_H / 2)], CAP_LOOK, n=40),)
    cols = (kit.Collider((0, 0, z_top), size=(CAP_W, CAP_W, TOP_T)),) + tuple(kit.Collider(c, size=s) for c, s in skirt)
    return kit.Prop("cap", pos=CAP_START, size=(CAP_W, CAP_W, CAP_H), mass=0.04, meshes=meshes, colliders=cols)


def _drip_tray():
    ix, iy = TRAY_IN
    w = TRAY_WALL
    walls = [((ix / 2 + w / 2, 0, TRAY_H / 2), (w, iy + 2 * w, TRAY_H)), ((-ix / 2 - w / 2, 0, TRAY_H / 2), (w, iy + 2 * w, TRAY_H)),
             ((0, iy / 2 + w / 2, TRAY_H / 2), (ix, w, TRAY_H)), ((0, -iy / 2 - w / 2, TRAY_H / 2), (ix, w, TRAY_H))]
    floor = ((0, 0, 0.001), (ix, iy, 0.002))
    ring_h = 0.012
    ring = _ring(0.0, 0.0, HOLDER_TOP - TOP - ring_h, HOLDER_OUT, HOLDER_IN, ring_h)
    post = ((HOLDER_OUT / 2 + 0.004, 0.0, (HOLDER_TOP - TOP) / 2), (0.008, 0.008, HOLDER_TOP - TOP))  # behind the ring
    meshes = ([kit.box(c, s, TRAY_LOOK) for c, s in walls + [floor]] + [kit.box(c, s, STEEL) for c, s in ring + [post]])
    cols = [kit.Collider(c, size=s) for c, s in walls + [floor] + ring + [post]]
    return kit.Prop("drip_tray", pos=(TRAY_X, TRAY_Y, TOP), size=(ix + 2 * w, iy + 2 * w, HOLDER_TOP - TOP),
                    meshes=tuple(meshes), colliders=tuple(cols))


OBJECTS = [_containment(), _jug(), _drip_tray(), _funnel(), _cap()]


# -- scoring ----------------------------------------------------------------------------------


def _tilt(s):
    x, y = s.quat_xyzw[0], s.quat_xyzw[1]
    return math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0)))


def _funnel_in_neck(f):
    """The funnel still in (or resting on) the jug's neck: its origin near the neck's axis and low."""
    return bool(np.hypot(f.pos[0] - JUG_X, f.pos[1] - JUG_Y) < FUNNEL_IN_R and f.pos[2] < NECK_TOP + FUNNEL_IN_Z)


def _cap_seated(c):
    """The cap down on the neck: centered on it within 9 mm, its top resting on the rim (height within 4 mm),
    upright within 10 degrees, and at rest."""
    return bool(np.hypot(c.pos[0] - JUG_X, c.pos[1] - JUG_Y) < CAP_R and abs(c.pos[2] - CAP_SEATED_Z) < CAP_DZ
                and _tilt(c) < CAP_TILT and c.resting())


def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _held(objects):
    """Names of the parts a gripper is holding: fingers closed past 60 percent and the tool point within 4 cm of the
    cap's center or of the funnel's grip tab."""
    f, c = objects["funnel"], objects["cap"]
    tab = np.asarray(f.pos) + _rot(f.quat_xyzw) @ np.asarray(TAB_C)
    held = set()
    for g in ("left_gripper", "right_gripper"):
        if g not in objects or not kit.opening(objects, g) <= HELD_OPEN:  # open, or no reading: not holding
            continue
        p = np.asarray(objects[g].pos)
        if np.linalg.norm(p - np.asarray(c.pos)) < HELD_R:
            held.add("cap")
        if np.linalg.norm(p - tab) < HELD_R or np.linalg.norm(p - np.asarray(f.pos)) < HELD_R:
            held.add("funnel")
    return held


def _funnel_stored(f):
    """The funnel set down on the drip tray and at rest: either seated in its holder (stem in the ring, its origin
    within 8 mm of the ring's axis and 6 mm of its seated height, upright within 20 degrees), or lying on the tray's
    floor (lowest point within 1.2 cm of it) with its whole outline inside the tray's walls."""
    if not f.resting():
        return False
    in_holder = (np.hypot(f.pos[0] - TRAY_X, f.pos[1] - TRAY_Y) < HOLDER_R
                 and abs(f.pos[2] - (HOLDER_TOP + BASE_T / 2)) < HOLDER_DZ and _tilt(f) < HOLDER_TILT)
    ix, iy = TRAY_IN
    lo, hi = np.asarray(f.low), np.asarray(f.high)
    inside = (lo[0] > TRAY_X - ix / 2 + WALL_MARGIN and hi[0] < TRAY_X + ix / 2 - WALL_MARGIN
              and lo[1] > TRAY_Y - iy / 2 + WALL_MARGIN and hi[1] < TRAY_Y + iy / 2 - WALL_MARGIN)
    lying = inside and abs(lo[2] - (TOP + 0.002)) < LYING_DZ
    return bool(in_holder or lying)


def _funnel_on_bench(f):
    """The funnel set down on the bare bench top (upright or on its side) and at rest: its lowest point within 1.2 cm
    of the table top, and its center not over the drip tray or the containment tray (a funnel draped over a tray's
    wall is neither stored nor on the bench). A funnel knocked off the table is not on the bench."""
    ix, iy = TRAY_IN
    over_tray = (abs(f.pos[0] - TRAY_X) < ix / 2 + TRAY_WALL and abs(f.pos[1] - TRAY_Y) < iy / 2 + TRAY_WALL)
    w = CONT_IN / 2 + CONT_WALL
    over_cont = abs(f.pos[0] - JUG_X) < w and abs(f.pos[1] - JUG_Y) < w
    return bool(f.resting() and abs(float(np.asarray(f.low)[2]) - TOP) < LYING_DZ and not over_tray and not over_cont)


def _score(objects):
    f, c = objects["funnel"], objects["cap"]
    if _funnel_in_neck(f):
        return 0.0
    held = _held(objects)  # a part still in a gripper's hand earns nothing
    seated = _cap_seated(c) and "cap" not in held
    stored = _funnel_stored(f) and "funnel" not in held
    on_bench = _funnel_on_bench(f) and "funnel" not in held
    if seated and stored:
        return 1.0
    if seated and on_bench:
        return 0.5
    if stored:
        return 0.25
    return 0.0


# --- What happened during the run. success() and score() are called after every control step (success first), so
# the run's history is kept here: where each gripper started, for the arms-home check. yamkit counts the simulator's
# scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()  # which trial this history belongs to
        self.home = None  # each gripper's tool point at the trial's first scored step
        self.last = None
        self.still = False


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    now = {g: np.asarray(objects[g].pos, dtype=float) for g in GRIPPERS if g in objects}
    if _H.home is None:
        _H.home = now
    _H.still = _H.last is not None and all(
        g in _H.last and np.linalg.norm(now[g] - _H.last[g]) < HOME_STILL for g in now)
    _H.last = now


def _arms_home(objects):
    """Both arms back home: each gripper's tool point within 3 cm of where it was at the trial's first scored step,
    and still."""
    if _H.home is None or not _H.still:
        return False
    return all(np.linalg.norm(np.asarray(objects[g].pos) - _H.home[g]) < HOME_R for g in _H.home if g in objects)


def _total(objects):
    job = _score(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """The job done (1.0) and, with REQUIRE_ARMS_HOME, both arms back home and still. Called first each step, so it
    also records the step's history."""
    _update(objects)
    return _total(objects) == 1.0


def score(objects):
    """Scored at the end. 0 while the funnel is still in the jug's neck, whatever else (a cap on top of it included).
    A part still held by a closed gripper earns nothing. Otherwise 1.0 with the cap seated on the neck and the funnel
    on the drip tray (in its holder, or lying inside the tray's walls); 0.5 with the cap seated and the funnel set
    down on the bare bench; 0.25 with the funnel on the drip tray but the cap not seated; 0 otherwise (a funnel
    knocked off the table included). With REQUIRE_ARMS_HOME, the job fully done with the arms not back home and still
    scores 0.9. Possible scores: 0, 0.25, 0.5, 0.9, 1."""
    return _total(objects)


# -- demo -------------------------------------------------------------------------------------

def demo():
    """Right arm only (everything is on its side of the bench, clear of the middle where its reach is least exact):
    grip the funnel's tab, lift the funnel straight up out of the neck, carry it to the drip tray, lower its stem into
    the holder and let go; then pick the cap off the bench, carry it over the jug, lower its skirt over the neck,
    press it onto the rim, let go and go back to its start pose."""
    travel = TOP + 0.205
    # Offsets measured in runs: the cap, held with the fingers closing along y, rides about 2.1 cm toward -y of the
    # tool point, so it is aimed that much toward +y.
    ride_y = 0.021
    yaw = 0.0  # fingers close along y: across the tab, and across the cap
    tab = np.array(FUNNEL_START) + np.array(TAB_C)
    tab_grip_z = tab[2] + TAB[2] / 2 - 0.012 + 0.013
    lift_dz = tab_grip_z - FUNNEL_START[2]  # tool point above the funnel's origin while it is held
    pick_f = (tab[0], tab[1], tab_grip_z)
    over_holder = (TRAY_X + TAB_C[0], TRAY_Y + TAB_C[1] - 0.006)  # held by its tab, the funnel rides 0.6 cm toward +y
    holder_z = HOLDER_TOP + BASE_T / 2 + lift_dz + 0.002
    cap_grip_z = CAP_START[2] + CAP_H / 2 - 0.012 + 0.013
    cap_dz = cap_grip_z - CAP_START[2]
    pick_c = (CAP_START[0], CAP_START[1], cap_grip_z)
    over_neck = (JUG_X, JUG_Y + ride_y)
    seat_z = CAP_SEATED_Z + cap_dz - 0.006  # a little below seated: the arm sags under load, so press on
    right = [
        Waypoint(1.5, (pick_f[0], pick_f[1], travel), FINGER_OPEN, "over the tab", yaw),
        Waypoint(1.5, pick_f, FINGER_OPEN, "down to the tab", yaw),
        Waypoint(0.8, pick_f, FINGER_CLOSED, "grip", yaw),
        Waypoint(1.6, (pick_f[0], pick_f[1], travel), FINGER_CLOSED, "lift straight out", yaw),
        Waypoint(1.8, (over_holder[0], over_holder[1], travel), FINGER_CLOSED, "carry", yaw),
        Waypoint(1.0, (over_holder[0], over_holder[1], holder_z + 0.015), FINGER_CLOSED, "over the holder", yaw),
        Waypoint(0.8, (over_holder[0], over_holder[1], holder_z + 0.015), FINGER_CLOSED, "settle", yaw),
        Waypoint(1.2, (over_holder[0], over_holder[1], holder_z), FINGER_CLOSED, "into the holder", yaw),
        Waypoint(0.6, (over_holder[0], over_holder[1], holder_z), FINGER_OPEN, "release", yaw),
        Waypoint(1.0, (over_holder[0], over_holder[1], travel), FINGER_OPEN, "up", yaw),
        Waypoint(1.5, (pick_c[0], pick_c[1], travel), FINGER_OPEN, "over the cap", yaw),
        Waypoint(1.5, pick_c, FINGER_OPEN, "down to the cap", yaw),
        Waypoint(0.8, pick_c, FINGER_CLOSED, "grip", yaw),
        Waypoint(1.2, (pick_c[0], pick_c[1], travel), FINGER_CLOSED, "lift", yaw),
        Waypoint(1.8, (over_neck[0], over_neck[1], travel), FINGER_CLOSED, "carry", yaw),
        Waypoint(1.0, (over_neck[0], over_neck[1], seat_z + 0.02), FINGER_CLOSED, "over the neck", yaw),
        Waypoint(1.0, (over_neck[0], over_neck[1], seat_z + 0.02), FINGER_CLOSED, "settle", yaw),
        Waypoint(1.2, (over_neck[0], over_neck[1], seat_z), FINGER_CLOSED, "onto the neck", yaw),
        Waypoint(0.6, (over_neck[0], over_neck[1], seat_z), FINGER_OPEN, "release", yaw),
        Waypoint(1.0, (over_neck[0], over_neck[1], travel), FINGER_OPEN, "up", yaw),
        kit.Home(2.0),  # back to the exact pose it started in (position and wrist angle)
        kit.Home(1.5, label="hold"),
    ]
    return {"right": right}
