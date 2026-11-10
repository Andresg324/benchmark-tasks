"""A glass electric kettle on its power base, nearly empty, with its lid standing open and a white MIN and MAX scale on
the front of the glass; a cylindrical glass jug of water stands on a cork coaster beside it. Boiling water means
filling the kettle to between its MIN and MAX lines, closing the lid and only then pressing the switch down: below MIN
the element runs dry, above MAX the water boils over through the lid and spout onto the base's electrical contacts, and
with the lid open the steam never reaches the automatic switch-off, so the kettle keeps boiling.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import mechkit as mk  # noqa: E402  (click positions for the lid and the switch)
import pourkit as pk  # noqa: E402  (the water: volumes, pours, spills)
import yamkit as kit  # noqa: E402  (modeled props, the hinged kettle, gripper states, tilted grips)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Put the kettle on for tea."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)
D = math.radians

# Scale: 0.39 of a real 1.7 L glass kettle, so volumes are 1:17 (0.39 cubed). Real -> sim: MIN 0.5 L -> 29 mL,
# MAX 1.7 L -> 100 mL, brim 2.2 L -> 128 mL, 0.15 L left in the kettle -> 9 mL, 1.95 L in the jug -> 115 mL.
SCALE_ML = 17.0  # real mL per sim mL

# The kettle and its power base, one fixed mechanism. Heights are above the table top, in the kettle's frame (origin
# on the table under the kettle's axis).
KX, KY = 0.33, -0.01
BASE_R, BASE_H = 0.040, 0.010  # the round power base
BODY_R, GLASS = 0.0295, 0.0025  # outer radius, glass wall
IN_R = BODY_R - GLASS  # 2.7 cm inside: 22.9 cm2 of water surface, so 1 mL raises the level 0.44 mm
FLOOR = 0.022  # the inside bottom (the steel heating plate, above a 1.2 cm steel collar)
DEPTH = 0.056  # inside depth to the rim
RIM = FLOOR + DEPTH  # 7.8 cm
AREA = math.pi * IN_R ** 2
MIN_ML, MAX_ML = 29.0, 100.0  # the lines: 0.5 L and 1.7 L real
MIN_Z, MAX_Z = FLOOR + MIN_ML * 1e-6 / AREA, FLOOR + MAX_ML * 1e-6 / AREA  # 1.27 cm and 4.37 cm above the floor
KETTLE0 = 9.0  # mL left in the kettle (0.15 L real), 3.9 mm deep: under MIN

# The lid: hinged at the back of the rim on the handle's side (+y), standing open past vertical at the start.
LID_T = 0.005
LID_HINGE = (0.0, BODY_R, RIM + LID_T / 2)
LID_OPEN, LID_CLOSED = D(-109), 0.0  # rad about x; negative opens it up toward +y
LID_SHUT_TOL = D(7)  # closed: within 7 degrees, the lid's front edge within 7 mm of the rim (the catch pulls it in)
LID_POUR = D(45)  # water goes in only while the lid is open more than 45 degrees (lower, the lid is in the way)

# The handle (+y side) and the on/off switch: a lever at the foot of the handle sticking out toward the robot. Down is
# on, up is off (20 degrees apart; a real kettle's switch travels about that far and stays down until it boils).
HANDLE_Y = BODY_R + 0.025  # the handle's grip bar
SW_PIVOT = (-0.006, HANDLE_Y, 0.040)
SW_LEN = 0.035
SW_OFF, SW_ON = D(10), D(-10)
SW_ON_TOL = D(4)  # on: the lever more than 4 degrees below level (past the free band between the two clicks)

# The jug (free): a cylindrical glass jug, 4.9 cm across and 12 cm tall, on its cork coaster (fixed).
JUG_XY = (0.23, -0.12)
COASTER_R, COASTER_H = 0.035, 0.004
JUG_R, JUG_H, JUG_IN, JUG_BASE = 0.0245, 0.12, 0.022, 0.004
JUG0 = 115.0  # mL (1.95 L real): emptying it into the kettle gives 124 mL, 24 mL (0.4 L real) over MAX
JUG_FLOOR_Z = TOP + COASTER_H
JUG_POS = (*JUG_XY, JUG_FLOOR_Z + JUG_H / 2)
FOOT = 0.001  # a flat square foot 1 mm proud under the jug's round base: a bumped cylinder can roll on its rim

GLASS_LOOK = kit.Look((0.86, 0.92, 0.95), 0.05, opacity=0.28)
STEEL = kit.Look((0.80, 0.81, 0.83), 0.25, metallic=1.0)
BLACK = kit.Look((0.07, 0.07, 0.08), 0.45)
MARK = kit.Look((0.97, 0.97, 0.97), 0.5)
SWITCH_TIP = kit.Look((0.10, 0.30, 0.75), 0.4)
CORK = kit.Look((0.66, 0.50, 0.32), 0.85)
WATER_RGB = (0.25, 0.52, 0.95)  # a strong blue, so the level reads against the lines in a 320 x 176 camera
LINE_W = 0.004  # m: MIN and MAX printed as bold red rings 4 mm tall all the way round (1 cm at full size, bolder than
# a real 1-2 mm print) so the top and wrist cameras at 320 x 176 can pick them out from any side
LINE = kit.Look((0.85, 0.08, 0.06), 0.5)


def _arc(r, z, width, look, half=D(40), center=math.pi, n=17, thick=0.0008):
    """A printed line on the glass: an arc of the band on the side facing the robot (-x)."""
    pts = [(r * math.cos(t), r * math.sin(t), z) for t in np.linspace(center - half, center + half, n)]
    return kit.band(pts, width, thick, look, normal=(0, 0, 1))


def _ring(r, z, width, look, thick=0.0008):
    pts = [(r * math.cos(t), r * math.sin(t), z) for t in np.linspace(0, 2 * math.pi, 41)]
    return kit.band(pts, width, thick, look, normal=(0, 0, 1))


def _kettle():
    """Power base, steel collar, glass body with the MIN/MAX scale, steel rim, black handle, spout, hinged lid and
    the switch lever."""
    hz0, hz1 = 0.030, 0.072  # the handle's lower and upper arms
    meshes = [kit.cylinder((0, 0, BASE_H / 2), BASE_R, BASE_H, BLACK),
              kit.cylinder((0, 0, (BASE_H + FLOOR) / 2), BODY_R, FLOOR - BASE_H, STEEL),
              kit.lathe([(0.0, FLOOR - 0.001), (BODY_R, FLOOR - 0.001), (BODY_R, RIM), (IN_R, RIM), (IN_R, FLOOR),
                         (0.0, FLOOR)], GLASS_LOOK),
              _ring(BODY_R + 0.0003, RIM - 0.002, 0.004, STEEL),
              # the scale: MIN and MAX lines, a short tick every 0.25 L between them, and a vertical bar
              _ring(BODY_R + 0.0006, MIN_Z, LINE_W, LINE),
              _ring(BODY_R + 0.0006, MAX_Z, LINE_W, LINE),
              kit.box((-BODY_R - 0.0004, 0.0, (MIN_Z + MAX_Z) / 2), (0.0008, 0.0015, MAX_Z - MIN_Z), MARK)]
    for ml in np.arange(750.0, 1700.0, 250.0):
        z = FLOOR + ml / SCALE_ML * 1e-6 / AREA
        meshes.append(_arc(BODY_R + 0.0006, z, 0.0008, MARK, half=D(8), center=math.pi + D(9), n=5))
    meshes += [kit.box((0, HANDLE_Y, (hz0 + hz1) / 2), (0.012, 0.010, hz1 - hz0 + 0.008), BLACK),  # grip bar
               kit.box((0, (BODY_R + HANDLE_Y) / 2, hz1), (0.012, HANDLE_Y - BODY_R + 0.004, 0.008), BLACK),
               kit.box((0, (BODY_R + HANDLE_Y) / 2, hz0), (0.012, HANDLE_Y - BODY_R + 0.004, 0.010), BLACK),
               kit.box((0, -BODY_R - 0.004, RIM - 0.006), (0.014, 0.010, 0.008), STEEL)]  # spout
    colliders = [kit.Collider((0, 0, BASE_H / 2), radius=BASE_R, height=BASE_H),
                 kit.Collider((0, 0, (BASE_H + RIM) / 2), radius=BODY_R, height=RIM - BASE_H),
                 kit.Collider((0, HANDLE_Y, (hz0 + hz1) / 2), size=(0.012, 0.010, hz1 - hz0 + 0.008)),
                 kit.Collider((0, (BODY_R + HANDLE_Y) / 2, hz1), size=(0.012, HANDLE_Y - BODY_R + 0.004, 0.008)),
                 kit.Collider((0, (BODY_R + HANDLE_Y) / 2, hz0), size=(0.012, HANDLE_Y - BODY_R + 0.004, 0.010))]
    lid_c = (0.0, -BODY_R, 0.0)  # the lid's center in its own frame (hinge at the origin)
    lid = kit.Link("lid", "revolute", "X", LID_HINGE, lower=D(-111), upper=0.0, start=LID_OPEN, mass=0.06,
                   law=mk.Detents(positions=(LID_CLOSED, LID_OPEN), labels=("CLOSED", "OPEN"), zone=D(7), pull=0.6,
                                  damp=0.03, drag=0.02),
                   meshes=(kit.cylinder(lid_c, BODY_R, LID_T, BLACK),
                           kit.cylinder((0.0, -BODY_R, LID_T / 2 + 0.003), 0.008, 0.006, STEEL),
                           kit.cylinder((0, 0, 0), 0.003, 0.016, STEEL, axis="x")),
                   colliders=(kit.Collider(lid_c, radius=BODY_R, height=LID_T),
                              kit.Collider((0.0, -BODY_R, LID_T / 2 + 0.003), radius=0.008, height=0.006)))
    sw = kit.Link("switch", "revolute", "Y", SW_PIVOT, lower=D(-12), upper=D(12), start=SW_OFF, mass=0.06,
                  floating=True,
                  law=mk.Detents(positions=(SW_OFF, SW_ON), labels=("OFF", "ON"), zone=D(6), pull=0.6, damp=0.03),
                  meshes=(kit.box((-SW_LEN / 2, 0, 0), (SW_LEN, 0.012, 0.006), BLACK),
                          kit.box((-SW_LEN + 0.006, 0, 0.0035), (0.012, 0.0125, 0.002), SWITCH_TIP)),
                  colliders=(kit.Collider((-SW_LEN / 2, 0, 0), size=(SW_LEN, 0.012, 0.006)),))
    return kit.Mechanism("kettle", pos=(KX, KY, TOP), size=(2 * BASE_R, 2 * BASE_R + 0.03, RIM + 0.07),
                         meshes=tuple(meshes), colliders=tuple(colliders), links=(lid, sw))


def _coaster():
    return kit.Prop("coaster", pos=(*JUG_XY, TOP + COASTER_H / 2), size=(2 * COASTER_R, 2 * COASTER_R, COASTER_H),
                    meshes=(kit.cylinder((0, 0, 0), COASTER_R, COASTER_H, CORK),),
                    colliders=(kit.Collider((0, 0, 0), radius=COASTER_R, height=COASTER_H),))


def _jug():
    z0 = -JUG_H / 2
    shell = kit.lathe([(0.0, z0), (JUG_R, z0), (JUG_R, -z0), (JUG_IN, -z0), (JUG_IN, z0 + JUG_BASE),
                       (0.0, z0 + JUG_BASE)], GLASS_LOOK)
    return kit.Prop("jug", pos=JUG_POS, size=(2 * JUG_R, 2 * JUG_R, JUG_H), mass=0.20,
                    meshes=(shell, _ring(JUG_R + 0.0004, -z0 - 0.004, 0.003, STEEL)),
                    colliders=(kit.Collider((0, 0, FOOT / 2), radius=JUG_R, height=JUG_H - FOOT),
                               kit.Collider((0, 0, z0 + 0.002), size=(0.032, 0.032, 0.004))))


OBJECTS = [_kettle(), _coaster(), _jug()]


def lid_angle(objects):
    """How far the lid is open, degrees (0 = shut)."""
    return -math.degrees(kit.joint(objects, "kettle/lid").pos)


def _lid_lets_water_in(objects):
    return kit.joint(objects, "kettle/lid").pos < -LID_POUR


pk.setup([pk.Vessel("kettle", sections=((IN_R, DEPTH),), floor=FLOOR, fill={"water": KETTLE0},
                    accepts=_lid_lets_water_in),
          pk.Vessel("jug", sections=((JUG_IN, JUG_H - JUG_BASE),), floor=-JUG_H / 2 + JUG_BASE, fill={"water": JUG0})],
         liquids={"water": WATER_RGB}, splash_drop=0.10, splash=0.25)
pk.set_floor(TOP)


# Scoring
SPILL_MAX = 3.0  # mL (about 50 mL real, a puddle that reaches the power base): more spilled or splashed is a spill
UPRIGHT = 10.0  # degrees
KNOCKED = 45.0  # degrees: the jug leaning this far with no gripper holding it has been knocked over
ON_COASTER_Z, ON_COASTER_R = 0.012, 0.03  # m: the jug's bottom within 1.2 cm of the coaster top, center within 3 cm
HOLD_R, HOLD_OPEN = 0.07, 0.6  # a gripper holds the jug: tool point within 7 cm and fingers under 60 % open
HOME_TOL, HOME_STILL = 0.03, 0.002  # m; m per control step
GRIPPERS = ("left_gripper", "right_gripper")


def lid_shut(objects):
    """The lid clicked shut: its detent reads CLOSED and the hinge is within 7 degrees of shut."""
    return mk.setting(objects, "kettle/lid") == "CLOSED" and lid_angle(objects) < math.degrees(LID_SHUT_TOL)


def switched_on(objects):
    """The switch clicked down: its detent reads ON and the lever is more than 4 degrees below level."""
    return mk.setting(objects, "kettle/switch") == "ON" and kit.joint(objects, "kettle/switch").pos < -SW_ON_TOL


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. pourkit updates the water inside the scene's object_states, before these are called. yamkit counts the
# simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.on_steps = 0  # control steps in a row the switch has been on
        self.boiled = False  # the kettle has been switched on (on for 2 steps in a row)
        self.water_on = None  # mL in the kettle when it was first switched on
        self.dry = False  # irreversible: switched on with the water under MIN (the element runs dry)
        self.over = False  # irreversible: water over MAX (no sink to pour it off; it boils over onto the base)
        self.lid_open_on = False  # irreversible: on while the lid was not shut (it boils on and on)
        self.loose = 0
        self.knocked = False  # irreversible: the jug went over
        self.home = None
        self.last = None
        self.still = False


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():
        _H = _History()
    water = pk.volume(objects, "kettle")
    if water > MAX_ML:
        _H.over = True
    on = switched_on(objects)
    _H.on_steps = _H.on_steps + 1 if on else 0
    if _H.on_steps >= 2:  # clicked on (2 steps rules out a one-step physics glitch)
        if not _H.boiled:
            _H.boiled, _H.water_on = True, water
        if water < MIN_ML:
            _H.dry = True
        if not lid_shut(objects):
            _H.lid_open_on = True
    _H.loose = _H.loose + 1 if pk.tilt(objects, "jug") > KNOCKED and not _held(objects) else 0
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


def _held(objects):
    s = objects["jug"]
    return any(g in objects and kit.opening(objects, g) < HOLD_OPEN
               and np.linalg.norm(np.asarray(objects[g].pos) - np.asarray(s.pos)) < HOLD_R for g in GRIPPERS)


def _jug_back(objects):
    """The jug upright (within 10 degrees), still, let go and standing on its coaster."""
    s = objects["jug"]
    on = (abs(s.low[2] - (TOP + COASTER_H)) < ON_COASTER_Z
          and math.hypot(s.pos[0] - JUG_XY[0], s.pos[1] - JUG_XY[1]) < ON_COASTER_R)
    return pk.tilt(objects, "jug") < UPRIGHT and on and s.resting(max_speed=0.02, max_spin=1.0) and not _held(objects)


def _job(objects):
    if _H.dry or _H.over or _H.lid_open_on or _H.knocked:
        return 0.0
    water = pk.volume(objects, "kettle")
    if water < MIN_ML:
        return 0.0
    if switched_on(objects) and not lid_shut(objects):
        return 0.0  # on with the lid open, even if only at the very last step (the 2-step latch can't see that yet)
    if not switched_on(objects) or pk.spilled(objects) > SPILL_MAX:
        return 0.25
    if not _jug_back(objects):
        return 0.75
    return 1.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def _debug(objects):
    if os.environ.get("KETTLE_DEBUG"):
        g = objects.get("left_gripper")
        print("KETTLE", pk.step(), "left", np.round(g.pos - [0, 0, TOP], 4) if g is not None else None,
              "lid", round(lid_angle(objects), 1), mk.setting(objects, "kettle/lid"),
              "sw", round(math.degrees(kit.joint(objects, "kettle/switch").pos), 1),
              mk.setting(objects, "kettle/switch"), "water", round(pk.volume(objects, "kettle"), 1), flush=True)


def success(objects):
    """The job done and, with REQUIRE_ARMS_HOME, both arms home and still. Called first each step, so it also
    records the history."""
    _update(objects)
    _debug(objects)
    return _job(objects) == 1.0 and (_arms_home(objects) or not REQUIRE_ARMS_HOME)


def score(objects):
    """0, irreversibly, if the kettle was switched on (switch clicked down for 2 steps) with under 29 mL in it (under
    MIN, 0.5 L real: the element runs dry), if the water in it ever went over 100 mL (over MAX, 1.7 L real: it boils
    over onto the base, and there is no sink to pour it off), if it was ever on while the lid was not clicked shut
    (within 7 degrees), or if the jug leaned more than 45 degrees for 2 steps with no gripper holding it (knocked
    over). 0 with less than 29 mL in the kettle at the end (includes doing nothing). 0.25 with the kettle filled
    between the lines but not switched on at the end, or with more than 3 mL (about 50 mL real) spilled or splashed.
    0.75 switched on and boiling, filled between the lines with the lid shut, but the jug not put back (upright within
    10 degrees, still, let go, on its coaster). 1.0 with the jug back and both arms home and still; 0.9 if an arm is
    not home. Possible scores: 0, 0.25, 0.75, 0.9, 1."""
    return _score(objects)


# --- Demo
YAW, ROLL = math.pi / 2, 1.35  # the right hand comes in from -y, fingers closing along x, tipped about 13 degrees down
GRIP_Z = 0.06  # grip height above the coaster
LIP_ABOVE = 0.05  # m: the jug's lip this far above the kettle's rim (the held jug rides about 2 cm low)
POUR_PITCH = 1.30  # rad (74.5 degrees): tipped this far the jug keeps about 58 mL, so about 57 mL go in (66 mL in the sim run, 1.1 L real)


def pour_plan(pitch=POUR_PITCH):
    """The right arm: grip the jug from the side, bring it behind the kettle already tipped 49 degrees, tip it about
    its lip to `pitch` rad over the open kettle, tip it back, set it back on its coaster, let go and go home."""
    sx, sy, _ = JUG_POS
    gz = JUG_FLOOR_Z + GRIP_Z
    tcp = np.array([sx, sy, gz])
    lip0 = pk.lip("jug", JUG_POS, toward=(1, 0, 0))
    over = (KX - 0.008, KY, TOP + RIM + LIP_ABOVE)
    t = lambda d, p, f, label: kit.Tilted(d, p, f, label, yaw=YAW, roll=ROLL)  # noqa: E731
    plan = [t(1.5, (sx, sy - 0.09, gz + 0.03), FINGER_OPEN, "beside the jug"),
            t(1.0, (sx, sy - 0.09, gz), FINGER_OPEN, "down"),
            t(1.2, tuple(tcp), FINGER_OPEN, "around the jug"),
            t(0.8, tuple(tcp), FINGER_CLOSED, "grip"),
            t(1.2, (sx, sy, gz + 0.10), FINGER_CLOSED, "lift")]
    plan += pk.pour(tcp, lip0, over, roll=ROLL, yaw=YAW, start_pitch=0.85, to_pitch=pitch, above=0.05, steps=8,
                    duration=6.0, hold=2.0, back=3.0, finger=FINGER_CLOSED)
    plan += [t(1.5, (sx, sy, gz + 0.10), FINGER_CLOSED, "back over the coaster"),
             t(1.2, (sx, sy, gz + 0.003), FINGER_CLOSED, "set down"),
             t(1.0, (sx, sy, gz + 0.003), FINGER_OPEN, "let go"),
             t(1.0, (sx, sy - 0.06, gz + 0.016), FINGER_OPEN, "back out"),
             t(1.0, (sx, sy - 0.10, gz + 0.08), FINGER_OPEN, "up and away"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    return plan


LAG = 0.0  # m: no offset needed here (the tool point tracked the lever within 2 mm in the sim)
PRESS = 0.006  # m: the tool point this far above the lever's top presses the closed pad tips 7 mm into it


def _lever_top(q, r=SW_LEN - 0.006):
    """World point on top of the switch lever's blue tip, at lever angle q."""
    px, py, pz = SW_PIVOT
    return np.array([KX + px - r * math.cos(q), KY + py, TOP + pz + r * math.sin(q) + 0.0045])


def left_plan(wait):
    """The left arm, after `wait` s: closed fingers push the open lid over from behind until it falls shut, then press
    the switch lever down past its click, lift off and go home."""
    hy = KY + LID_HINGE[1]
    z = TOP + RIM + 0.04  # tool point 4 cm over the rim: the pad tips push the lid about 2.5 cm above its hinge
    plan = [Waypoint(wait, (0.20, 0.27, TOP + 0.20), FINGER_OPEN, "wait"),
            Waypoint(1.8, (KX, hy + 0.07, z + 0.05), FINGER_CLOSED, "behind the open lid"),
            Waypoint(1.0, (KX, hy + 0.06, z), FINGER_CLOSED, "down beside it")]
    for k, dy in enumerate((0.04, 0.025, 0.012, 0.0, -0.015, -0.03)):
        plan.append(Waypoint(0.5, (KX, hy + dy, z), FINGER_CLOSED, "push the lid over"))
    plan.append(Waypoint(1.0, (KX, hy - 0.03, z + 0.04), FINGER_CLOSED, "up off the shut lid"))
    up = np.array([0.0, 0.0, 1.0])
    lag = np.array([0.0, -LAG, 0.0])
    p0 = _lever_top(SW_OFF) + up * PRESS + lag
    plan += [Waypoint(1.5, tuple(p0 + up * 0.05), FINGER_CLOSED, "over the switch"),
             Waypoint(0.8, tuple(p0 + up * 0.004), FINGER_CLOSED, "onto the switch")]
    for k in range(1, 6):
        q = SW_OFF + (D(-12) - SW_OFF) * k / 5
        plan.append(Waypoint(0.4, tuple(_lever_top(q) + up * PRESS + lag), FINGER_CLOSED, "press it down"))
    plan += [Waypoint(0.6, tuple(_lever_top(D(-12)) + up * 0.05 + lag), FINGER_CLOSED, "lift off"),
             Waypoint(1.2, (0.22, 0.20, TOP + 0.18), FINGER_OPEN, "away"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    return plan


def demo():
    """The right arm pours about 57 mL from the jug into the open kettle, tipping it to 74.5 degrees about its lip 5 cm
    over the rim, which brings the water to about 66 mL (1.1 L real, between the lines), and sets the jug back on its
    coaster. Once the right arm has left the kettle, the left arm pushes the lid shut, presses the switch down and
    goes home."""
    right = pour_plan()
    wait = sum(w.duration for w in right[:-6]) - 1.0  # the jug is back over the coaster, clear of the kettle
    return {"right": right, "left": left_plan(wait)}
