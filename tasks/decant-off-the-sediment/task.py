"""A clear glass wine bottle of old red wine stands on the table beside an empty clear glass decanter. The bottle has
stood upright for days, so its sediment (a dark crust of tannins and pigment, with the wine it holds) lies on the
bottle's bottom under 108 mL of clear wine. Decanting means pouring the clear wine off slowly, watching the sediment
creep along the bottle toward the neck, and stopping when it reaches the shoulder, so the last of the wine stays in
the bottle with the sediment.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import pourkit as pk  # noqa: E402  (the liquids: volumes, pours, spills, the settled sediment)
import yamkit as kit  # noqa: E402  (modeled props, gripper states, tilted demo grips; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Decant the bottle."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# The bottle: a small (scaled) clear glass wine bottle. Outer body radius and height, shoulder, neck; inner sections
# (radius, height) from the inside floor up: body, shoulder, neck. Glass 2.5 mm, base 4 mm.
B_R, B_BODY, B_SH, B_NECK_R, B_H = 0.024, 0.089, 0.012, 0.0105, 0.131
B_BASE = 0.004
B_SECTIONS = ((0.0215, 0.085), (0.014, 0.012), (0.0085, 0.030))
WINE0, SEDIMENT0 = 108.0, 12.0  # mL: clear wine over the settled sediment (the sediment layer is 8 mm deep)
SEAL = math.pi * (0.014 ** 2 * 0.012 + 0.0085 ** 2 * 0.030) * 1e6  # 14.2 mL: the shoulder and neck's wine

# The decanter: a wide-bottomed clear glass carafe with a narrower neck. Outer: base radius and height, neck radius,
# total height; inner sections from the inside floor.
D_R, D_LOW, D_NECK_R, D_H, D_BASE = 0.058, 0.045, 0.033, 0.085, 0.004
D_SECTIONS = ((0.054, D_LOW - D_BASE), (0.030, D_H - D_LOW))

BOTTLE_XY, DECANTER_XY = (0.23, -0.12), (0.33, -0.01)
FOOT = 0.001  # a flat square foot 1 mm proud under each round base: a PhysX cylinder can roll on its bottom rim
BOTTLE_POS = (*BOTTLE_XY, TOP + B_H / 2)
DECANTER_POS = (*DECANTER_XY, TOP + D_H / 2)

DGLASS = kit.Look((0.88, 0.93, 0.96), 0.05, opacity=0.07)
RIM_LOOK = kit.Look((0.72, 0.84, 0.76), 0.1)  # the glass's edges, seen as pale green lines
FOIL = kit.Look((0.55, 0.08, 0.10), 0.35, metallic=0.6)  # the foil capsule's cut edge below the lip
WINE_RGB, SEDIMENT_RGB = (0.80, 0.14, 0.20), (0.10, 0.06, 0.04)


def _ring(r, z, width, look, thick=0.0008):
    pts = [(r * math.cos(t), r * math.sin(t), z) for t in np.linspace(0, 2 * math.pi, 41)]
    return kit.band(pts, width, thick, look, normal=(0, 0, 1))


def _bottle():
    z0 = -B_H / 2
    zb, zs = z0 + B_BODY, z0 + B_BODY + B_SH
    # The renderer hides the liquid markers behind any see-through mesh, so the bottle's glass is drawn as thin
    # glass-colored rings (base, heel, shoulder, neck, lip) around an otherwise invisible wall: the liquid inside, with
    # its dark sediment layer, stays in view, as it would through clear glass. The colliders are the solid bottle.
    rings = [(B_R, z0 + 0.002, 0.004), (B_R, z0 + 0.03, 0.0015), (B_R, z0 + 0.06, 0.0015), (B_R, zb - 0.001, 0.002),
             (0.0175, zb + 0.006, 0.002), (B_NECK_R, zs + 0.002, 0.003), (B_NECK_R, -z0 - 0.002, 0.004)]
    meshes = tuple(_ring(r + 0.0004, z, w, RIM_LOOK) for r, z, w in rings) + (_ring(B_NECK_R + 0.0008, -z0 - 0.012,
                                                                                    0.010, FOIL),)
    neck_h = B_H - B_BODY
    return kit.Prop("bottle", pos=BOTTLE_POS, size=(2 * B_R, 2 * B_R, B_H), mass=0.22, meshes=meshes,
                    colliders=(kit.Collider((0, 0, z0 + FOOT + (B_BODY - FOOT) / 2), radius=B_R,
                                            height=B_BODY - FOOT),
                               kit.Collider((0, 0, zb + neck_h / 2), radius=B_NECK_R, height=neck_h),
                               kit.Collider((0, 0, z0 + 0.002), size=(0.032, 0.032, 0.004))))


def _decanter():
    z0 = -D_H / 2
    zl = z0 + D_LOW
    shell = kit.lathe([(0.0, z0), (D_R, z0), (D_R, zl - 0.012), (D_NECK_R, zl + 0.004), (D_NECK_R, -z0),
                       (0.030, -z0), (0.030, zl + 0.003), (0.054, zl - 0.013), (0.054, z0 + D_BASE),
                       (0.0, z0 + D_BASE)], DGLASS)
    up = D_H - D_LOW
    return kit.Prop("decanter", pos=DECANTER_POS, size=(2 * D_R, 2 * D_R, D_H), mass=0.60, meshes=(shell,),
                    colliders=(kit.Collider((0, 0, z0 + FOOT + (D_LOW - FOOT) / 2), radius=D_R, height=D_LOW - FOOT),
                               kit.Collider((0, 0, zl + up / 2), radius=D_NECK_R, height=up),
                               kit.Collider((0, 0, z0 + 0.002), size=(0.07, 0.07, 0.004))))


# The light card: a white card standing on its own foot just behind the decanter on the +y side, so the bottle's
# shoulder and neck are seen against it from the right wrist camera (which looks from -y) and the dark sediment band
# shows as a silhouette, as a candle behind the neck does in a cellar. 16 x 16 cm, 3 mm thick, foot 16 x 5 cm.
CARD_W, CARD_H, CARD_T, FOOT_D = 0.16, 0.16, 0.003, 0.05
CARD_POS = (0.29, DECANTER_XY[1] + D_R + 0.02, TOP + CARD_H / 2)
CARD_LOOK = kit.Look((0.97, 0.96, 0.92), 0.9)


def _card():
    z0 = -CARD_H / 2
    face = kit.box((0, FOOT_D / 2 - CARD_T / 2, 0.0), (CARD_W, CARD_T, CARD_H), CARD_LOOK)
    foot = kit.box((0, 0, z0 + 0.003), (CARD_W, FOOT_D, 0.006), CARD_LOOK)
    return kit.Prop("light_card", pos=CARD_POS, size=(CARD_W, FOOT_D, CARD_H), mass=0.6, meshes=(face, foot),
                    colliders=(kit.Collider((0, FOOT_D / 2 - CARD_T / 2, 0.0), size=(CARD_W, CARD_T, CARD_H)),
                               kit.Collider((0, 0, z0 + 0.003), size=(CARD_W, FOOT_D, 0.006))))


OBJECTS = [_bottle(), _decanter(), _card()]

# The sediment's creep (pourkit's Vessel.slide_*): with the bottle leaning past 75 degrees and less than 50 mL of
# clear wine left over it (under half the wine: the wine no longer covers the crust along the bottle's low side), the
# crust slides toward the neck, from the floor to the shoulder in 4 s with no stream and up to 5 times faster under a
# full stream (the stream drags it). Tipped back under 75 degrees it stops where it is. Once at the shoulder it pours
# with the wine, and past 85 degrees it runs out of the neck on its own at 2 mL/s. (Times scaled from about a minute
# of real decanting.)
SLIDE_BELOW, SLIDE_TILT, SLIDE_S, SLIDE_GAIN = 50.0, 75.0, 4.0, 4.0
SLURRY_TILT, SLURRY_FLOW = 85.0, 2.0
# Glugging: tipped past 93 degrees the neck points down, air gulps back up through the wine and the churn lifts the
# sediment, which reaches the shoulder in 0.5 s. A decanter keeps the neck above horizontal for this reason, so a steep
# flip-and-count pour fails however it is timed. (The demo peaks at about 87 degrees.)
GLUG_TILT, GLUG_S = 93.0, 0.5

pk.setup([pk.Vessel("bottle", sections=B_SECTIONS, floor=-B_H / 2 + B_BASE,
                    fill={"wine": WINE0, "sediment": SEDIMENT0}, seal=SEAL,
                    slide_below=SLIDE_BELOW, slide_tilt=SLIDE_TILT, slide_s=SLIDE_S, slide_gain=SLIDE_GAIN,
                    slurry_tilt=SLURRY_TILT, slurry_flow=SLURRY_FLOW, glug_tilt=GLUG_TILT, glug_s=GLUG_S),
          pk.Vessel("decanter", sections=D_SECTIONS, floor=-D_H / 2 + D_BASE)],
         liquids={"wine": WINE_RGB, "sediment": SEDIMENT_RGB}, splash_drop=0.10, splash=0.25,
         settled=("sediment",))
pk.set_floor(TOP)


# Scoring
SOME = 0.10 * WINE0  # mL of wine in the decanter before anything counts (10 %, 10.8 mL)
FULL = 0.60 * WINE0  # mL of wine in the decanter for full credit (60 %, 64.8 mL): the sediment starts to slide once
#                     under 50 mL (46 %) is left, and a careful pourer stops a few seconds later with roughly a third of
#                     the wine left with the sediment, which is what decanting an old bottle loses
SEDIMENT_MAX = 1.0  # mL of sediment in the decanter: more clouds the wine (1/12 of the sediment; about 2.5 steps'
#                     worth of the stream once the sediment reaches the neck)
SPILL_MAX = 3.0  # mL of liquid spilled or splashed in all (a few drops on the cloth are normal; 3 mL is a stain)
UPRIGHT = 10.0  # degrees
KNOCKED = 45.0  # degrees: a container leaning this far with no gripper holding it has been knocked over
ON_TABLE = 0.01  # m: the bottle's bottom within 1 cm of the table top stands on it
HOLD_R, HOLD_OPEN = 0.07, 0.6  # a gripper holds a container: tool point within 7 cm and fingers under 60 % open
HOME_TOL, HOME_STILL = 0.03, 0.002  # m; m per control step
GRIPPERS = ("left_gripper", "right_gripper")


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. pourkit updates the liquids inside the scene's object_states, before these are called. yamkit counts the
# simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.loose = {"bottle": 0, "decanter": 0}  # control steps in a row leaning past KNOCKED with no gripper on it
        self.knocked = False  # irreversible: an open bottle or the decanter went over
        self.home = None
        self.last = None
        self.still = False


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():
        _H = _History()
    for n in _H.loose:
        _H.loose[n] = _H.loose[n] + 1 if pk.tilt(objects, n) > KNOCKED and not _held(objects, n) else 0
        if _H.loose[n] >= 2:
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


def _set_down(objects):
    """The bottle standing upright (within 10 degrees) and still on the table, and let go."""
    s = objects["bottle"]
    return (pk.tilt(objects, "bottle") < UPRIGHT and abs(s.low[2] - TOP) < ON_TABLE
            and s.resting(max_speed=0.02, max_spin=1.0) and not _held(objects, "bottle"))


def _job(objects):
    if _H.knocked:
        return 0.0
    wine_in = pk.volume(objects, "decanter", "wine")
    if wine_in < SOME:
        return 0.0
    if pk.volume(objects, "decanter", "sediment") > SEDIMENT_MAX:
        return 0.0
    if pk.spilled(objects) > SPILL_MAX:
        return 0.25
    if wine_in < FULL:
        return 0.5
    if not _set_down(objects):
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
    """0 if the bottle or the decanter ever leaned more than 45 degrees for 2 steps with no gripper holding it
    (knocked over; irreversible). 0 if less than 10 % of the clear wine (10.8 mL) is in the decanter. 0 if more than
    1 mL of sediment is in the decanter (the bottle was poured past the shoulder). 0.25 if more than 3 mL of liquid
    in all was spilled or splashed (a stream falling more than 10 cm splashes a quarter of itself). 0.5 with 10 to
    60 % of the wine in the decanter (stopped far too early). 0.75 with at least 60 % (64.8 mL) in but the bottle not
    standing upright (within 10 degrees) and still on the table, let go. 1.0 with the bottle set down and both arms
    home and still; 0.9 if an arm is not home. Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1."""
    return _score(objects)


YAW, ROLL = math.pi / 2, 1.35  # the right hand comes in from -y, fingers closing along x, tipped about 13 degrees down
GRIP_Z = 0.055  # grip height above the table, on the bottle's body below the shoulder (lower grips swing the lip off the decanter)
LIP_ABOVE = 0.05  # m: the bottle's lip this far above the decanter's rim (the held bottle rides about 2 cm low)
PEAK = 1.53  # rad: how far the demo tips the bottle (about 87 degrees in the sim, reached over 8 s, held 0.5 s and tipped
#              back over 1.2 s: 72.4 mL of wine goes in and the sediment stops 0.93 of the way to the shoulder)


def pour_plan(peak=PEAK):
    """The right arm's plan: grip the bottle from the side, carry it, tip it about its lip over the decanter to
    `peak` rad, tip it back, set it back on its spot, let go and go home."""
    sx, sy, _ = BOTTLE_POS
    tx, ty, _ = DECANTER_POS
    gz = TOP + GRIP_Z
    tcp = np.array([sx, sy, gz])
    lip0 = pk.lip("bottle", BOTTLE_POS, toward=(1.0, 0, 0))
    over = (tx - 0.008, ty, TOP + D_H + LIP_ABOVE)
    t = lambda d, p, f, label: kit.Tilted(d, p, f, label, yaw=YAW, roll=ROLL)  # noqa: E731
    plan = [t(1.5, (sx, sy - 0.09, gz + 0.03), FINGER_OPEN, "beside the bottle"),
            t(1.0, (sx, sy - 0.09, gz), FINGER_OPEN, "down"),
            t(1.2, tuple(tcp), FINGER_OPEN, "around the bottle"),
            t(0.8, tuple(tcp), FINGER_CLOSED, "grip"),
            t(1.2, (sx, sy, gz + 0.10), FINGER_CLOSED, "lift")]
    plan += pk.pour(tcp, lip0, over, roll=ROLL, yaw=YAW, start_pitch=0.85, to_pitch=peak, above=0.05, steps=10,
                    duration=8.0, hold=0.5, back=1.2, finger=FINGER_CLOSED)
    plan += [t(1.5, (sx, sy, gz + 0.10), FINGER_CLOSED, "back over its spot"),
             t(1.2, (sx, sy, gz + 0.003), FINGER_CLOSED, "set down"),
             t(1.0, (sx, sy, gz + 0.003), FINGER_OPEN, "let go"),
             t(1.0, (sx, sy - 0.06, gz + 0.016), FINGER_OPEN, "back out"),
             t(1.0, (sx, sy - 0.10, gz + 0.08), FINGER_OPEN, "up and away"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    return plan


def demo():
    """The right arm decants: it grips the bottle from the side, brings it over the decanter already tipped about 50
    degrees, tips it slowly on to about 87 degrees about its lip 5 cm above the decanter's rim, while the sediment
    creeps most of the way to the shoulder, tips it back before it gets there, sets it back on its spot,
    lets go and goes home. The left arm stays home."""
    return {"right": pour_plan()}
