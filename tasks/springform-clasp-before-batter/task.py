"""A 10 cm (4 inch) mini springform pan stands on the counter, its clasp lever swung open and its loose base resting
on the top lip of the ring's groove instead of in it; a narrow glass jug of cake batter stands beside it. A springform
holds batter only when it is closed up: the base pressed down into the groove at the bottom of the ring, then the side
clasp pushed in until it snaps over center, which draws the ring tight around the base's edge. With the clasp open the
ring's seam stands open and the batter runs out through it; with the base above the groove it runs out under the ring.
Either way it ends up on the counter or the oven floor. So the baker seats the base, snaps the clasp shut, and only
then pours the batter in and sets the jug down.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import mechkit as mk  # noqa: E402  (the over-center clasp and the ring that follows it)
import pourkit as pk  # noqa: E402  (the batter: volumes, pours, spills, the pan's leak)
import yamkit as kit  # noqa: E402  (modeled props, the pan mechanism, gripper states, tilted grips)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Pour the cake batter into the springform pan."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# --- The pan: a real 10 cm (4 inch) mini springform at full size, fixed to the counter (a real trial tapes it down).
# Heights are above the table top in the pan's frame (origin on the table under the pan's axis).
PX, PY = 0.33, 0.0
R_IN, WALL, RING_H = 0.050, 0.0015, 0.045  # the ring: inner radius, sheet steel wall, height
RM, R_OUT = R_IN + WALL / 2, R_IN + WALL
GROOVE_Z = 0.004  # the base plate's underside when seated in the groove (the ring's rolled bottom bead is below it)
BASE_T = 0.003  # the base plate's thickness
FLOOR_Z = GROOVE_Z + BASE_T  # 7 mm: the inside floor (the seated base's top)
BASE_UP = 0.006  # m: at the start the base rests on the groove's top lip, 6 mm above its seat
SEATED = 0.001  # m: the base within 1 mm of its seat counts as seated (the groove is 3 mm tall; 1 mm up, its edge is
#                 still inside it)
SNAP = 0.003  # m: pressed more than halfway down (3 mm) the base's edge slips past the lip and snaps into the groove
BASE_K = 300.0  # N/m: the lip holds the base up (0.9 N at the halfway point, a light finger press)
CLAMP_K = 3000.0  # N/m: a locked ring clamps the base where it is (it can't be pressed down or pulled up)
# The clasp: a lever on a vertical pin on the ring's left side (+y), lying flat along the ring when closed.
LEVER_L, LEVER_Z = 0.040, 0.024  # lever length; pin height
PIN = (0.012, R_OUT + 0.0045, LEVER_Z)  # the pin, in the pan's frame; closed, the lever points to -x from it
OPEN = -0.95  # rad (54 degrees): fully swung out
CENTER = -0.30  # rad (17 degrees out): the over-center point; inside it the clasp snaps shut, outside it springs open
LOCK_AT = -0.06  # rad (3.4 degrees, 2.4 mm at the lever's end): within this of flat the clasp is locked over center
SNAP_TORQUE = 0.12  # N*m: the ring's spring pushing the clasp to one side of center (3 N at the tab)
# The ring's seam: the ring is two halves here, the left (-x) half turning on a pin at the ring's right (-y) side so
# the seam at +y opens as the clasp swings out, by GAP_OPEN fully open (a real ring springs open 4-6 mm at the seam).
GAP_OPEN = 0.005
SPREAD = GAP_OPEN / (2 * RM)  # rad the ring's left half turns at OPEN

# --- The batter and its leaks. A real springform holds batter only with the base seated and the clasp locked.
# Leak rates from slot flow, Q = w h^3 rho g H / (12 mu L), for cake batter (mu about 2 Pa*s, rho 1,000 kg/m^3) 1.5 cm
# deep (H = 7.5 mm average head): the open seam is a slot 5 mm wide, 1.5 cm tall and 2 mm deep (the seam's overlap),
# about 3 mL/s; a base on the lip leaves a 1 mm slot all round under the ring (w 31 cm, 6 mm across the lip), about
# 0.16 mL/s by the formula; 0.4 mL/s (a 1.4 mm gap) is used, since a base resting on the lip sits slightly crooked.
LEAK_OPEN, LEAK_UNSEATED = 3.0, 0.4  # mL/s
JUG0 = 150.0  # mL of batter in the jug
BATTER_RGB = (0.96, 0.80, 0.42)

# The jug (free): a narrow glass pouring jug, 4.9 cm across and 12 cm tall, standing on the counter.
JUG_XY = (0.23, -0.13)
JUG_R, JUG_H, JUG_IN, JUG_BASE = 0.0245, 0.12, 0.022, 0.004
JUG_POS = (*JUG_XY, TOP + JUG_H / 2)
JUG_FLOW = 25.0  # mL/s: the most the jug pours (a thick batter stream, about a 1 cm rope)
FOOT = 0.001  # a flat square foot 1 mm proud under the jug's round base: a bumped cylinder can roll on its rim

STEEL = kit.Look((0.80, 0.81, 0.83), 0.3, metallic=0.9)
NONSTICK = kit.Look((0.13, 0.13, 0.14), 0.5, metallic=0.3)
CLASP = kit.Look((0.70, 0.71, 0.73), 0.35, metallic=0.9)
GLASS_LOOK = kit.Look((0.86, 0.92, 0.95), 0.05, opacity=0.28)


def _arc(r, z, t0, t1, n=24, about=(0.0, 0.0)):
    """Points on a circle of radius r at height z from angle t0 to t1 (rad), relative to `about`."""
    return [(r * math.cos(t) - about[0], r * math.sin(t) - about[1], z) for t in np.linspace(t0, t1, n)]


def _half(t0, t1, about=(0.0, 0.0)):
    """One half of the ring from angle t0 to t1: the wall, its rolled top bead and the bottom bead (around the groove),
    and its colliders: collider boxes can't turn, so the wall is a row of 4 mm square posts every 7.5 degrees, centered
    2.2 mm outside R_IN (their corners stay 0.8 mm outside the base's edge; 2.6 mm gaps, too narrow for a finger)."""
    meshes = (kit.band(_arc(RM, RING_H / 2, t0, t1, about=about), RING_H, WALL, STEEL, normal=(0, 0, 1)),
              kit.band(_arc(RM + 0.0008, RING_H - 0.0015, t0, t1, about=about), 0.003, 0.0035, STEEL,
                       normal=(0, 0, 1)),
              kit.band(_arc(RM + 0.0008, 0.0025, t0, t1, about=about), 0.005, 0.003, STEEL, normal=(0, 0, 1)))
    colliders = []
    n = 24
    for k in range(n):
        t = t0 + (t1 - t0) * (k + 0.5) / n
        c = (R_IN + 0.0022) * math.cos(t) - about[0], (R_IN + 0.0022) * math.sin(t) - about[1]
        colliders.append(kit.Collider((c[0], c[1], RING_H / 2), size=(0.004, 0.004, RING_H)))
    return meshes, tuple(colliders)


class SeatBase:
    """The base's joint (a mechkit-style law, see mechkit.py): its height above its seat in the groove. Resting on the
    groove's lip (above SNAP) it is held up there; pressed below SNAP its edge slips into the groove and it is pulled
    down onto its seat. While the clasp is locked the ring clamps it wherever it is. The base is floating (its weight
    rests on the lip or the groove), so only these forces move it."""

    def drive(self, key, q, qd, J, mem):
        mem["seated"] = q <= SEATED
        if J["clasp"] >= LOCK_AT:
            at = mem.setdefault("clamped_at", q)
            return at, CLAMP_K, 0.0, 5.0
        mem.pop("clamped_at", None)
        return (BASE_UP if q > SNAP else 0.0), BASE_K, 0.0, 2.0


def _pan():
    fixed_m, fixed_c = _half(-math.pi / 2, math.pi / 2)  # the right (+x) half, fixed, with the clasp's pin
    pin_m = (kit.cylinder((PIN[0], PIN[1], LEVER_Z), 0.0028, 0.024, CLASP),
             kit.box((PIN[0] + 0.004, R_OUT + 0.0015, LEVER_Z), (0.012, 0.003, 0.024), CLASP))  # the pin's bracket
    pivot = (0.0, -RM)  # the left half turns about the ring's -y side
    move_m, move_c = _half(math.pi / 2, 3 * math.pi / 2, about=pivot)
    keeper = kit.box((-0.006 - pivot[0], R_OUT + 0.0015 - pivot[1], LEVER_Z), (0.010, 0.003, 0.016), CLASP)
    band = kit.Link("ring", "revolute", "Z", (pivot[0], pivot[1], 0.0), lower=-0.005, upper=SPREAD + 0.005,
                    start=SPREAD, mass=0.08, meshes=move_m + (keeper,), colliders=move_c,
                    law=mk.Follow(source="clasp", ratio=SPREAD / OPEN, lo=0.0, hi=SPREAD, rate=0.02))
    # the clasp lever (link frame at the pin): a flat bar to -x with a thumb tab curled out at its end
    lever = kit.Link("clasp", "revolute", "Z", PIN, lower=OPEN, upper=0.0, start=OPEN, mass=0.06,
                     meshes=(kit.box((-LEVER_L / 2, 0.0, 0.0), (LEVER_L, 0.003, 0.016), CLASP),
                             kit.box((-LEVER_L + 0.005, 0.004, 0.0), (0.010, 0.008, 0.020), CLASP)),
                     colliders=(kit.Collider((-LEVER_L / 2, 0.0, 0.0), size=(LEVER_L, 0.003, 0.016)),
                                kit.Collider((-LEVER_L + 0.005, 0.004, 0.0), size=(0.010, 0.008, 0.020))),
                     law=mk.OverCenter(center=CENTER, lock_at=LOCK_AT, push=SNAP_TORQUE, dead=0.03, damp=0.02))
    base = kit.Link("bottom", "prismatic", "Z", (0.0, 0.0, GROOVE_Z), lower=0.0, upper=0.008, start=BASE_UP,
                    mass=0.10, floating=True, law=SeatBase(),
                    meshes=(kit.cylinder((0, 0, BASE_T / 2), R_IN - 0.0006, BASE_T, NONSTICK),),
                    colliders=(kit.Collider((0, 0, BASE_T / 2), radius=R_IN - 0.003, height=BASE_T),))
    return kit.Mechanism("pan", pos=(PX, PY, TOP), size=(2 * R_OUT + 0.01, 2 * R_OUT + 0.05, RING_H),
                         meshes=fixed_m + pin_m, colliders=fixed_c, links=(lever, band, base))


def _jug():
    z0 = -JUG_H / 2
    shell = kit.lathe([(0.0, z0), (JUG_R, z0), (JUG_R, -z0), (JUG_IN, -z0), (JUG_IN, z0 + JUG_BASE),
                       (0.0, z0 + JUG_BASE)], GLASS_LOOK)
    rim = kit.band(_arc(JUG_R + 0.0004, -z0 - 0.004, 0, 2 * math.pi, n=41), 0.003, 0.0008, STEEL, normal=(0, 0, 1))
    return kit.Prop("jug", pos=JUG_POS, size=(2 * JUG_R, 2 * JUG_R, JUG_H), mass=0.20, meshes=(shell, rim),
                    colliders=(kit.Collider((0, 0, FOOT / 2), radius=JUG_R, height=JUG_H - FOOT),
                               kit.Collider((0, 0, z0 + 0.002), size=(0.032, 0.032, 0.004))))


OBJECTS = [_pan(), _jug()]


def clasp_locked(objects):
    """The clasp over center: the lever within 3.4 degrees of flat (LOCK_AT), read from the joint itself and from the
    over-center law's own flag."""
    return kit.joint(objects, "pan/clasp").pos >= LOCK_AT and mk.locked(objects, "pan/clasp")


def base_seated(objects):
    """The base within 1 mm of its seat in the groove."""
    return kit.joint(objects, "pan/bottom").pos <= SEATED


def sealed(objects):
    """The clasp locked with the base seated, and the base was already seated when the clasp locked: a ring clamped
    onto a base still up on its lip leaves a gap under the ring however the base is pressed down afterwards."""
    return clasp_locked(objects) and base_seated(objects) and _H.clamped_seated is not False


def _leak(objects):
    """mL/s the pan loses: through the open seam while the clasp is not locked, else under the ring while the base is
    above its seat (the joint positions themselves, so it holds even before the law has run once)."""
    if kit.joint(objects, "pan/clasp").pos < LOCK_AT:
        return LEAK_OPEN
    if kit.joint(objects, "pan/bottom").pos > SEATED:
        return LEAK_UNSEATED
    return 0.0


pk.setup([pk.Vessel("pan", sections=((R_IN, RING_H - FLOOR_Z),), floor=FLOOR_Z, leak=_leak,
                    leak_at=(-0.002, R_OUT + 0.002, 0.003)),
          pk.Vessel("jug", sections=((JUG_IN, JUG_H - JUG_BASE),), floor=-JUG_H / 2 + JUG_BASE, fill={"batter": JUG0},
                    max_flow=JUG_FLOW)],
         liquids={"batter": BATTER_RGB}, splash_drop=0.10, splash=0.25)
pk.set_floor(TOP)


# --- Scoring
SOME = 10.0  # mL of batter in the pan before anything counts (a 1.3 mm skim over the base)
FULL = 110.0  # mL in the pan for full credit (73 % of the jug, 1.4 cm deep): what a pour leaves in the jug is about
#               15-25 mL of batter clinging to its walls and the last that won't run out without scraping
LEAK_TOL = 0.5  # mL: batter in the pan while it is not sealed (a few drops: any real pour puts in more in a step)
SPILL_MAX = 3.0  # mL spilled on the counter in all (a coin-sized drip; more is a mess to wipe from under the pan)
UPRIGHT = 10.0  # degrees
KNOCKED = 45.0  # degrees: the jug leaning this far with no gripper holding it has been knocked over
ON_TABLE = 0.01  # m: the jug's bottom within 1 cm of the table top stands on it
HOLD_R, HOLD_OPEN = 0.07, 0.6  # a gripper holds the jug: tool point within 7 cm and fingers under 60 % open
ON_CLASP = 0.04  # m: a tool point within 4 cm of the clasp's tab is still on the clasp (holding it shut)
HOME_TOL, HOME_STILL = 0.03, 0.002  # m; m per control step
GRIPPERS = ("left_gripper", "right_gripper")


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. pourkit updates the batter inside the scene's object_states, before these are called. yamkit counts the
# simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.open_steps = 0  # control steps in a row with batter in the pan while it was not sealed
        self.leaked = False  # irreversible: batter was in the pan while it was not sealed (it ran out under the ring)
        self.clamped_seated = None  # at the step the clasp locked: was the base seated? (None while it is open)
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
    if clasp_locked(objects):
        if _H.clamped_seated is None:
            _H.clamped_seated = base_seated(objects)
    else:
        _H.clamped_seated = None
    _H.open_steps = _H.open_steps + 1 if pk.volume(objects, "pan") > LEAK_TOL and not sealed(objects) else 0
    if _H.open_steps >= 2:  # 2 steps rules out a one-step physics glitch
        _H.leaked = True
    if pk.leaked(objects, "pan") > LEAK_TOL:  # a slow trickle that runs straight out never fills the pan, but it is
        _H.leaked = True  # batter on the counter all the same
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


def tab_xyz(q):
    """World position of the clasp's thumb tab with the lever at angle q."""
    c, s = math.cos(q), math.sin(q)
    lx, ly = -LEVER_L + 0.005, 0.004
    return np.array([PX + PIN[0] + c * lx - s * ly, PY + PIN[1] + s * lx + c * ly, TOP + LEVER_Z])


def _on_clasp(objects):
    tab = tab_xyz(kit.joint(objects, "pan/clasp").pos)
    return any(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - tab) < ON_CLASP for g in GRIPPERS)


def _jug_down(objects):
    """The jug upright (within 10 degrees), still, on the table and let go."""
    s = objects["jug"]
    return (pk.tilt(objects, "jug") < UPRIGHT and abs(s.low[2] - TOP) < ON_TABLE
            and s.resting(max_speed=0.02, max_spin=1.0) and not _held(objects))


def _job(objects):
    if _H.leaked or _H.knocked:
        return 0.0
    batter = pk.volume(objects, "pan")
    if batter < SOME or not sealed(objects):
        return 0.0
    if _held(objects) or _on_clasp(objects):
        return 0.0  # nothing counts while the jug is still held or a hand still holds the clasp
    if pk.spilled(objects) > SPILL_MAX:
        return 0.25
    if batter < FULL:
        return 0.5
    if not _jug_down(objects):
        return 0.75
    return 1.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def _debug(objects):
    path = os.environ.get("SPRINGFORM_DEBUG")  # a file to append one line per step to
    if path:
        g = {k: np.round(np.asarray(objects[k].pos) - [0, 0, TOP], 3).tolist() for k in GRIPPERS if k in objects}
        print("SPRING", pk.step(), g, "clasp", round(math.degrees(kit.joint(objects, "pan/clasp").pos), 1),
              "ring", round(math.degrees(kit.joint(objects, "pan/ring").pos), 2),
              "base_mm", round(kit.joint(objects, "pan/bottom").pos * 1000, 2), "sealed", sealed(objects),
              "pan", round(pk.volume(objects, "pan"), 1), "jug", round(pk.volume(objects, "jug"), 1),
              "jug_tilt", round(pk.tilt(objects, "jug"), 1), "spill", round(pk.spilled(objects), 2),
              "leaked", _H.leaked, "score", _score(objects), file=open(path, "a"))


def success(objects):
    """The job done and, with REQUIRE_ARMS_HOME, both arms home and still. Called first each step, so it also
    records the history."""
    _update(objects)
    _debug(objects)
    return _job(objects) == 1.0 and (_arms_home(objects) or not REQUIRE_ARMS_HOME)


def score(objects):
    """0, irreversibly, if there was ever more than 0.5 mL of batter in the pan for 2 steps while it was not sealed
    (sealed: the clasp locked, within 3.4 degrees of flat, and the base within 1 mm of its seat in the groove), since
    that batter runs out under the ring, or if the jug leaned more than 45 degrees for 2 steps with no gripper holding
    it (knocked over). 0 with less than 10 mL in the pan at the end (includes doing nothing, and closing the pan
    without pouring). 0 while the jug is still held at the end or a tool point is within 4 cm of the clasp's tab. 0.25
    with more than 3 mL spilled on the counter. 0.5 with 10 to 110 mL in the pan. 0.75 with at least 110 mL in but the
    jug not standing upright (within 10 degrees) and still on the table. 1.0 with the jug set down and both arms home
    and still; 0.9 if an arm is not home. Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1."""
    return _score(objects)


# --- Demo
YAW, ROLL = math.pi / 2, 1.35  # the right hand comes in from -y, fingers closing along x, tipped about 13 degrees down
GRIP_Z = 0.06  # grip height above the table
LIP_ABOVE = 0.05  # m: the jug's lip this far above the pan's rim (the held jug rides about 2 cm low)
POUR_PITCH = 1.60  # rad: how far the demo tips the jug


def pour_plan(wait, pitch=POUR_PITCH):
    """The right arm: grip the jug from the side and lift it, wait `wait` s, bring it over the pan already tipped 49
    degrees, tip it about its lip to `pitch` rad, tip it back, set it back on its spot, let go and go home."""
    sx, sy, _ = JUG_POS
    gz = TOP + GRIP_Z
    tcp = np.array([sx, sy, gz])
    lip0 = pk.lip("jug", JUG_POS, toward=(1, 0, 0))
    over = (PX - 0.012, PY - 0.005, TOP + RING_H + LIP_ABOVE)
    t = lambda d, p, f, label: kit.Tilted(d, p, f, label, yaw=YAW, roll=ROLL)  # noqa: E731
    plan = [t(1.5, (sx, sy - 0.09, gz + 0.03), FINGER_OPEN, "beside the jug"),
            t(1.0, (sx, sy - 0.09, gz), FINGER_OPEN, "down"),
            t(1.2, tuple(tcp), FINGER_OPEN, "around the jug"),
            t(0.8, tuple(tcp), FINGER_CLOSED, "grip"),
            t(1.2, (sx, sy, gz + 0.06), FINGER_CLOSED, "lift"),
            t(wait, (sx, sy, gz + 0.06), FINGER_CLOSED, "wait for the pan to be closed")]
    plan += pk.pour(tcp, lip0, over, roll=ROLL, yaw=YAW, start_pitch=0.85, to_pitch=pitch, above=0.05, steps=10,
                    duration=7.0, hold=2.0, back=2.0, finger=FINGER_CLOSED)
    plan += [t(1.5, (sx, sy, gz + 0.06), FINGER_CLOSED, "back over its spot"),
             t(1.2, (sx, sy, gz + 0.003), FINGER_CLOSED, "set down"),
             t(1.0, (sx, sy, gz + 0.003), FINGER_OPEN, "let go"),
             t(1.0, (sx, sy - 0.06, gz + 0.016), FINGER_OPEN, "back out"),
             t(1.0, (sx, sy - 0.10, gz + 0.08), FINGER_OPEN, "up and away"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    return plan


PRESS_Z = 0.010  # m: the closed fingers' tool point this high presses the pad tips (1.3 cm below it) onto the seat
PUSH_OFF = 0.016  # m: the tool point this far outside the tab's middle (pad half-width plus the arm's lag)


def _tab_push(q, off=PUSH_OFF):
    """Tool point beside the tab's outer face (the side away from the ring) with the lever at q."""
    n = np.array([-math.sin(q), math.cos(q), 0.0])  # the lever's outward normal
    return tab_xyz(q) + n * off + np.array([0.0, 0.0, 0.010])


def left_plan():
    """The left arm: press the base down into its groove with closed fingers, then push the clasp's tab in along its
    arc until it snaps over center, back off and go home."""
    c = (PX, PY)
    plan = [Waypoint(2.0, (c[0], c[1], TOP + 0.12), FINGER_CLOSED, "over the pan"),
            Waypoint(1.5, (c[0], c[1], TOP + 0.05), FINGER_CLOSED, "down into the ring"),
            Waypoint(1.2, (c[0], c[1], TOP + PRESS_Z), FINGER_CLOSED, "press the base into the groove"),
            Waypoint(0.8, (c[0], c[1], TOP + PRESS_Z), FINGER_CLOSED, "hold"),
            Waypoint(1.2, (c[0], c[1], TOP + 0.10), FINGER_CLOSED, "up out of the ring")]
    p0 = _tab_push(OPEN, PUSH_OFF + 0.03)
    plan += [Waypoint(1.5, tuple(p0 + [0, 0, 0.05]), FINGER_CLOSED, "over the clasp"),
             Waypoint(1.0, tuple(p0), FINGER_CLOSED, "beside the clasp's tab")]
    for k, q in enumerate(np.linspace(OPEN, 0.12, 8)):
        plan.append(Waypoint(0.5, tuple(_tab_push(q)), FINGER_CLOSED, "push the clasp shut"))
    plan += [Waypoint(0.8, tuple(_tab_push(0.0, PUSH_OFF + 0.04)), FINGER_CLOSED, "back off"),
             Waypoint(1.0, tuple(_tab_push(0.0, PUSH_OFF + 0.06) + [0, 0, 0.08]), FINGER_CLOSED, "up and away"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    return plan


def demo():
    """The left arm presses the base down into the ring's groove, pushes the clasp shut until it snaps over center and
    goes home. Meanwhile the right arm picks up the jug and holds it beside the pan; once the left arm is clear, it
    pours the batter in, tipping the jug about its lip 5 cm over the rim, sets the jug back on its spot and goes
    home."""
    left = left_plan()
    clear = sum(w.duration for w in left[:-2]) + 0.5  # the left arm on its way home, clear of the pan
    wait = max(0.5, clear - 5.7 - 1.5)  # the right arm's grip and lift take 5.7 s; then 1.5 s to come over the pan
    return {"left": left, "right": pour_plan(wait)}
