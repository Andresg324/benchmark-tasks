"""A red cast-iron pitcher pump bolted to a wooden well platform, its spout over a steel pail, and a white enamel cup
of water on the table. The pump has sat unused, so its leather plunger cup has dried and no longer seals: stroking
the handle only moves air. Starting it means pouring the cup of priming water into the open top of the pump head
first, onto the leather, and then stroking the handle steadily until water runs from the spout into the pail.
Pumping dry first scuffs the dry leather against the bore and lifts nothing; too little priming water leaves the cup
unsealed; stopping after a few strokes stops before the water arrives.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import mechkit as mk  # noqa: E402  (the handle's stroke counting: HandPump)
import pourkit as pk  # noqa: E402  (the priming water, the pumped water and the pail)
import yamkit as kit  # noqa: E402  (modeled props, the hinged handle, gripper states, tilted grips)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Get the pump going and pump water into the pail."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)
D = math.radians

# Scale: 0.55 of a real No. 2 pitcher pump (a 3 in. bore), so lengths are 0.55x and volumes 1:6 (0.55 cubed).
# Real -> sim: a cup of priming water (240 mL) -> 40 mL; one stroke's lift (about 450 mL) -> 75 mL.
SCALE_ML = 6.0  # real mL per sim mL

# The pump: one fixed mechanism. Heights above the table top, in the pump's frame (origin on the table under the pump
# head's axis).
PX, PY = 0.32, 0.0
PLAT = (0.10, 0.10, 0.025)  # the wooden well platform (the cover over the well), its center 0.5 cm toward the robot
PLAT_X = -0.005
FLANGE_R, FLANGE_H = 0.042, 0.008  # the pump's base flange, bolted to the platform
BODY_R = 0.032  # the head's outside radius
IN_R = 0.028  # the bore of the open top (5.6 cm; 3 in. bore plus the bowl's flare, scaled)
RIM = 0.09  # the head's top rim, 9 cm above the table
FLOOR = 0.05  # the leather plunger cup at the top of its stroke, 4 cm under the rim
AREA = math.pi * IN_R ** 2  # 24.6 cm2: 1 mL stands 0.41 mm deep over the leather
PRIME_ML = 20.0  # mL standing on the leather for it to seal: 8 mm of water, enough to cover the cup's 6 mm flared lip
#                  so the whole leather soaks and swells against the bore (120 mL real, half the cup)
PER_STROKE = 75.0  # mL lifted by each sealed stroke once the water has arrived (450 mL real: 3 in. bore, 4 in. stroke)
LIFT = 3  # sealed strokes that only draw the air out of the drop pipe before water reaches the spout (a short pipe;
#           a real shallow well takes 3 to 10 strokes)

# The handle: hinged on a fulcrum bracket at the head's +y side, 2.5 cm above the rim, reaching 14 cm out toward +y.
# Up is positive (about the x axis). It rests up against its stop, held there by the plunger rod's weight (a spring
# here); pushing it down lifts the plunger and the water.
PIVOT = (0.0, BODY_R + 0.004, RIM + 0.025)  # 11.5 cm above the table
H_UP, H_DOWN = D(20), D(-25)  # the handle's stops: the top of its grip travels from 16.6 cm down to 7.3 cm above
#                                the table (a 9.3 cm stroke at the hand; 17 cm real)
STROKE_HI, STROKE_LO = D(12), D(-15)  # a stroke: from at least 12 degrees up to at least 15 degrees down, 60 % of the
#                                        full 45-degree swing (short jabs at the handle's middle move no water)
GRIP_Y, GRIP_TOP = 0.12, 0.010  # the wooden grip's middle (from the pivot) and its top above the handle's axis
H_MASS = 0.15  # kg

IRON = kit.Look((0.55, 0.07, 0.05), 0.55, metallic=0.3)  # red-painted cast iron
DARK = kit.Look((0.10, 0.10, 0.11), 0.5, metallic=0.4)
WOOD = kit.Look((0.62, 0.45, 0.28), 0.8)
GRIP = kit.Look((0.78, 0.60, 0.36), 0.7)
LEATHER = kit.Look((0.36, 0.22, 0.12), 0.9)
STEEL = kit.Look((0.74, 0.76, 0.78), 0.3, metallic=1.0)
ENAMEL = kit.Look((0.94, 0.94, 0.92), 0.3)
RIM_BLUE = kit.Look((0.12, 0.22, 0.55), 0.4)
WATER_RGB = (0.25, 0.52, 0.95)  # a strong blue, readable in a 320 x 176 camera

# The spout: an iron pipe out of the head's +x side, 7.2 cm up, turned down over the pail.
SPOUT_Z, SPOUT_R, SPOUT_END = 0.072, 0.008, 0.092
SPOUT_OUT = (PX + SPOUT_END - 0.004, PY, TOP + SPOUT_Z - 0.013)  # where the water leaves it (world)

# The pail (fixed in place): a steel pail 9 cm across and 5.2 cm tall, under the spout.
PAIL_XY = (PX + 0.097, PY)
PAIL_R, PAIL_IN, PAIL_H, PAIL_BASE = 0.045, 0.042, 0.052, 0.003
PAIL_POS = (*PAIL_XY, TOP + PAIL_H / 2)

# The cup (free): white enamel, 4 cm across and 7.5 cm tall, 40 g, holding 40 mL (a cup of water, 240 mL real).
CUP_XY = (0.25, -0.08)
CUP_R, CUP_H, CUP_IN, CUP_BASE = 0.020, 0.075, 0.0175, 0.004
CUP0 = 40.0
CUP_POS = (*CUP_XY, TOP + CUP_H / 2)
FOOT = 0.001  # a flat square foot 1 mm proud under the cup's round base: a bumped cylinder can roll on its rim


def _ring(r, z, width, look, thick=0.0012):
    pts = [(r * math.cos(t), r * math.sin(t), z) for t in np.linspace(0, 2 * math.pi, 41)]
    return kit.band(pts, width, thick, look, normal=(0, 0, 1))


def _sealed():
    return pk.volume(None, "pump") >= PRIME_ML


def _deliver(ml):
    pk.add("pail", "well", ml, source="spout", at=SPOUT_OUT)


HANDLE_LAW = mk.HandPump(handle="handle", stroke_lo=STROKE_LO, stroke_hi=STROKE_HI, per_stroke=PER_STROKE, top=1e9,
                         rest=H_UP + 0.15, k=0.5, damp=0.05, lift=LIFT, sealed=_sealed, deliver=_deliver)


def _pump():
    """Platform, flange, head with its open top and the leather plunger cup inside, spout, fulcrum bracket and the
    hinged handle with its wooden grip."""
    prof = [(0.0, PLAT[2]), (BODY_R, PLAT[2]), (BODY_R, RIM - 0.008), (BODY_R + 0.004, RIM), (IN_R, RIM),
            (IN_R, FLOOR), (0.0, FLOOR)]
    py0 = PIVOT[1]
    cheek = lambda s: (s * 0.0105, py0, (RIM - 0.012 + PIVOT[2] + 0.008) / 2)  # noqa: E731
    cheek_size = (0.005, 0.012, PIVOT[2] + 0.008 - (RIM - 0.012))
    meshes = (kit.box((PLAT_X, 0, PLAT[2] / 2), PLAT, WOOD),
              kit.cylinder((0, 0, PLAT[2] + FLANGE_H / 2), FLANGE_R, FLANGE_H, IRON),
              kit.lathe(prof, IRON),
              _ring(BODY_R + 0.0006, RIM - 0.026, 0.004, IRON),
              kit.cylinder((0, 0, FLOOR + 0.0015), IN_R - 0.0005, 0.003, LEATHER),  # the leather cup's top
              kit.cylinder((0, 0, FLOOR + 0.004), 0.004, 0.006, DARK),  # the plunger nut
              kit.cylinder(((BODY_R + SPOUT_END) / 2 - 0.004, 0, SPOUT_Z), SPOUT_R, SPOUT_END - BODY_R, IRON,
                           axis="x"),
              kit.cylinder((SPOUT_END - 0.004, 0, SPOUT_Z - 0.008), SPOUT_R * 0.9, 0.012, IRON),
              kit.box(cheek(1), cheek_size, IRON), kit.box(cheek(-1), cheek_size, IRON),
              kit.cylinder((0, py0, PIVOT[2]), 0.003, 0.028, DARK, axis="x"))  # the fulcrum pin
    colliders = (kit.Collider((PLAT_X, 0, PLAT[2] / 2), size=PLAT),
                 kit.Collider((0, 0, (PLAT[2] + RIM) / 2), radius=BODY_R, height=RIM - PLAT[2]),
                 kit.Collider(((BODY_R + SPOUT_END) / 2, 0, SPOUT_Z), size=(SPOUT_END - BODY_R, 0.016, 0.016)),
                 kit.Collider(cheek(1), size=cheek_size), kit.Collider(cheek(-1), size=cheek_size))
    bar0, bar1 = -0.006, GRIP_Y - 0.017
    handle = kit.Link("handle", "revolute", "X", PIVOT, lower=H_DOWN, upper=H_UP, start=H_UP, mass=H_MASS,
                      floating=True, law=HANDLE_LAW,
                      meshes=(kit.box((0, (bar0 + bar1) / 2, 0), (0.012, bar1 - bar0, 0.010), IRON),
                              kit.box((0, GRIP_Y, 0.0), (0.022, 0.036, 0.020), GRIP)),
                      colliders=(kit.Collider((0, (bar0 + bar1) / 2, 0), size=(0.012, bar1 - bar0, 0.010)),
                                 kit.Collider((0, GRIP_Y, 0.0), size=(0.022, 0.036, 0.020))))
    return kit.Mechanism("pump", pos=(PX, PY, TOP), size=(PLAT[0], PLAT[1], PIVOT[2] + 0.06), meshes=meshes,
                         colliders=colliders, links=(handle,))


def _pail():
    z0 = -PAIL_H / 2
    shell = kit.lathe([(0.0, z0), (PAIL_R - 0.004, z0), (PAIL_R, -z0), (PAIL_IN, -z0), (PAIL_IN - 0.004, z0 + PAIL_BASE),
                       (0.0, z0 + PAIL_BASE)], STEEL)
    return kit.Prop("pail", pos=PAIL_POS, size=(2 * PAIL_R, 2 * PAIL_R, PAIL_H),
                    meshes=(shell, _ring(PAIL_R + 0.0004, -z0 - 0.003, 0.004, STEEL)),
                    colliders=(kit.Collider((0, 0, 0), radius=PAIL_R, height=PAIL_H),))


def _cup():
    z0 = -CUP_H / 2
    shell = kit.lathe([(0.0, z0), (CUP_R, z0), (CUP_R, -z0), (CUP_IN, -z0), (CUP_IN, z0 + CUP_BASE),
                       (0.0, z0 + CUP_BASE)], ENAMEL)
    return kit.Prop("cup", pos=CUP_POS, size=(2 * CUP_R, 2 * CUP_R, CUP_H), mass=0.04,
                    meshes=(shell, _ring(CUP_R + 0.0004, -z0 - 0.002, 0.003, RIM_BLUE)),
                    colliders=(kit.Collider((0, 0, FOOT / 2), radius=CUP_R, height=CUP_H - FOOT),
                               kit.Collider((0, 0, z0 + 0.002), size=(0.028, 0.028, 0.004))))


OBJECTS = [_pump(), _pail(), _cup()]

pk.setup([pk.Vessel("pump", sections=((IN_R, RIM - FLOOR),), floor=FLOOR),
          pk.Vessel("pail", sections=((PAIL_IN, PAIL_H - PAIL_BASE),), floor=-PAIL_H / 2 + PAIL_BASE),
          pk.Vessel("cup", sections=((CUP_IN, CUP_H - CUP_BASE),), floor=-CUP_H / 2 + CUP_BASE, fill={"water": CUP0})],
         liquids={"water": WATER_RGB, "well": WATER_RGB}, splash_drop=0.10, splash=0.25)
pk.set_floor(TOP)


# Scoring
FLOW_ML = 150.0  # mL of pumped water in the pail for "flowing": two delivering strokes (0.9 L real), so the pump has
#                  held its prime from one stroke to the next rather than giving a single gush
UPRIGHT = 10.0  # degrees
ON_TABLE = 0.01  # m: the cup's bottom within 1 cm of the table top
HOLD_R, HOLD_OPEN = 0.045, 0.6  # a gripper holds the cup: tool point within 4.5 cm of its center (on its wall), fingers
# under 60 % open
DRY_MAX = 2  # dry strokes allowed: a stroke or two while the priming water goes in is common practice and barely
# touches the leather; more than that is pumping a dry pump
HANDLE_NEAR = 0.04  # m: a tool point within 4 cm of the handle's grip, open or closed, is a hand still on the handle
HANDLE_BACK = STROKE_HI  # let go, the handle springs back up past 12 degrees
HOME_TOL, HOME_STILL = 0.03, 0.002  # m; m per control step
GRIPPERS = ("left_gripper", "right_gripper")


def handle_deg(objects):
    return math.degrees(kit.joint(objects, "pump/handle").pos)


def handle_grip(objects, q=None):
    """World point at the middle of the top of the handle's wooden grip."""
    q = kit.joint(objects, "pump/handle").pos if q is None else q
    p = np.asarray(objects["pump"].pos, float) + np.array(PIVOT)
    return p + np.array([0.0, GRIP_Y * math.cos(q) - GRIP_TOP * math.sin(q), GRIP_Y * math.sin(q) + GRIP_TOP * math.cos(q)])


def strokes(objects):
    """The handle's stroke counts this trial: total, sealed ("wet") and dry."""
    s = mk.state("pump/handle")
    return {k: int(s.get(k, 0)) for k in ("strokes", "wet", "dry")}


class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.home = None
        self.last = None
        self.still = False


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():
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


def _cup_held(objects):
    s = objects["cup"]
    return any(g in objects and kit.opening(objects, g) < HOLD_OPEN
               and np.linalg.norm(np.asarray(objects[g].pos) - np.asarray(s.pos)) < HOLD_R for g in GRIPPERS)


def _hand_on_handle(objects):
    g0 = handle_grip(objects)
    return any(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - g0) < HANDLE_NEAR for g in GRIPPERS)


def let_go(objects):
    """Nothing held at the end: no gripper holding the cup, no tool point on the handle, and the handle back up."""
    return not _cup_held(objects) and not _hand_on_handle(objects) and handle_deg(objects) >= math.degrees(HANDLE_BACK)


def _cup_down(objects):
    """The cup standing upright (within 10 degrees) and still on the table top."""
    s = objects["cup"]
    in_pail = math.hypot(s.pos[0] - PAIL_XY[0], s.pos[1] - PAIL_XY[1]) < PAIL_R + 0.01  # set down in the pail: not put away
    return (pk.tilt(objects, "cup") < UPRIGHT and abs(s.low[2] - TOP) < ON_TABLE and not in_pail
            and s.resting(max_speed=0.02, max_spin=1.0))


def pumped(objects):
    """mL of well water the pump has put in the pail (the priming water poured into the pail does not count)."""
    return pk.volume(objects, "pail", "well")


def _job(objects):
    if pumped(objects) <= 0.0:
        return 0.0
    if not let_go(objects):
        return 0.0
    if strokes(objects)["dry"] > DRY_MAX:
        return 0.25
    if pumped(objects) < FLOW_ML:
        return 0.5
    if not _cup_down(objects):
        return 0.75
    return 1.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def _debug(objects):
    out = os.environ.get("PUMP_DEBUG")  # a file to append one line per step to
    if out:
        g = objects.get("left_gripper")
        print("PUMP", pk.step(), "left", np.round(np.asarray(g.pos) - [0, 0, TOP], 4) if g is not None else None,
              "handle", round(handle_deg(objects), 1), strokes(objects), "head", round(pk.volume(objects, "pump"), 1),
              "cup", round(pk.volume(objects, "cup"), 1), "pail", round(pumped(objects), 1),
              "spill", round(pk.spilled(objects), 1), "cup tilt", round(pk.tilt(objects, "cup"), 1),
              file=open(f"{out}.{os.getpid()}", "a"))


def success(objects):
    """The job done and, with REQUIRE_ARMS_HOME, both arms home and still. Called first each step, so it also
    records the history."""
    _update(objects)
    _debug(objects)
    return _job(objects) == 1.0 and (_arms_home(objects) or not REQUIRE_ARMS_HOME)


def score(objects):
    """0 with no pumped water in the pail (includes doing nothing, pumping with too little priming water, which is
    under 20 mL on the leather, and stopping in the first 3 sealed strokes, before the water arrives); priming water
    poured into the pail does not count. 0 if at the end a gripper still holds the cup, a tool point is within 4 cm of
    the handle's grip, or the handle is not back up past 12 degrees. 0.25 if the handle made more than 2 full strokes
    (from 12 degrees up to 15 degrees down) before the head held 20 mL (pumped dry: the dry leather is scuffed), whatever came
    after. 0.5 with pumped water in the pail but under 150 mL (two delivering strokes). 0.75 with 150 mL or more but
    the cup not standing upright (within 10 degrees) and still on the table. 1.0 with the cup set down and both arms
    home and still; 0.9 if an arm is not home. Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1."""
    return _score(objects)


# --- Demo
YAW, ROLL = math.pi / 2, 1.35  # the right hand comes in from -y, fingers closing along x, tipped about 13 degrees down
GRIP_Z = 0.045  # grip height above the table on the cup
LIP_ABOVE = 0.05  # m: the cup's lip this far above the head's rim (the held cup rides about 2 cm low)
START_PITCH = 0.85  # rad (49 degrees): the cup holds all 40 mL to about 59 degrees, and its body clears the rim
POUR_PITCH = 1.55  # rad (89 degrees): the cup nearly empties


def pour_plan():
    """The right arm: grip the cup from the side, bring it over the pump head, tip it about its lip to empty it onto
    the leather, tip it back, set it back on its spot, let go and go home."""
    sx, sy, _ = CUP_POS
    gz = TOP + GRIP_Z
    tcp = np.array([sx, sy, gz])
    lip0 = pk.lip("cup", CUP_POS, toward=(1, 0, 0))
    over = (PX - 0.008, PY, TOP + RIM + LIP_ABOVE)
    t = lambda d, p, f, label: kit.Tilted(d, p, f, label, yaw=YAW, roll=ROLL)  # noqa: E731
    plan = [t(2.5, (sx, sy - 0.09, gz + 0.03), FINGER_OPEN, "beside the cup"),
            t(1.0, (sx, sy - 0.09, gz), FINGER_OPEN, "down"),
            t(1.8, tuple(tcp), FINGER_OPEN, "around the cup"),
            t(0.8, tuple(tcp), FINGER_CLOSED, "grip"),
            t(1.2, (sx, sy, gz + 0.10), FINGER_CLOSED, "lift")]
    plan += pk.pour(tcp, lip0, over, roll=ROLL, yaw=YAW, start_pitch=START_PITCH, to_pitch=POUR_PITCH, above=0.05,
                    steps=8, duration=4.0, hold=1.5, back=2.0, finger=FINGER_CLOSED)
    plan += [t(1.5, (sx, sy, gz + 0.10), FINGER_CLOSED, "back over its spot"),
             t(1.2, (sx, sy, gz + 0.003), FINGER_CLOSED, "set down"),
             t(1.0, (sx, sy, gz + 0.003), FINGER_OPEN, "let go"),
             t(1.5, (sx, sy - 0.06, gz + 0.016), FINGER_OPEN, "back out"),
             t(1.0, (sx, sy - 0.08, gz + 0.06), FINGER_OPEN, "up and away"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    return plan


PRESS = 0.006  # m: the tool point this far above the grip's top presses the closed pad tips 7 mm into it
PUSH_TO = D(-29)  # aim past the bottom stop (-25 degrees): the arm lags its target, so the handle still reaches it
N_STROKES = 6  # 3 to bring the water up, then 3 delivering strokes (225 mL; 2 would do, one spare)


def _grip_world(q):
    return np.array([PX, PY + PIVOT[1] + GRIP_Y * math.cos(q) - GRIP_TOP * math.sin(q),
                     TOP + PIVOT[2] + GRIP_Y * math.sin(q) + GRIP_TOP * math.cos(q)])


def pump_plan(wait):
    """The left arm, after `wait` s: closed fingers on top of the handle's grip push it down to its bottom stop and
    lift off so it springs back up, `N_STROKES` times, then go home."""
    up = np.array([0.0, 0.0, 1.0])
    above = _grip_world(H_UP) + up * (PRESS + 0.012)
    plan = [Waypoint(wait, (0.20, 0.27, TOP + 0.20), FINGER_OPEN, "wait"),
            Waypoint(2.0, tuple(above + np.array([0.0, 0.03, 0.0])), FINGER_CLOSED, "over the handle")]
    for s in range(N_STROKES):
        plan.append(Waypoint(0.6, tuple(_grip_world(H_UP) + up * PRESS), FINGER_CLOSED, f"stroke {s + 1}: onto it"))
        for k in range(1, 7):
            q = H_UP + (PUSH_TO - H_UP) * k / 6
            plan.append(Waypoint(0.3, tuple(_grip_world(q) + up * PRESS), FINGER_CLOSED, f"stroke {s + 1}: down"))
        plan.append(Waypoint(0.4, tuple(_grip_world(PUSH_TO) + up * PRESS), FINGER_CLOSED, f"stroke {s + 1}: bottom"))
        plan.append(Waypoint(0.7, tuple(above), FINGER_CLOSED, f"stroke {s + 1}: lift off"))
    plan += [Waypoint(1.0, tuple(above + np.array([-0.04, 0.05, 0.0])), FINGER_OPEN, "away"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    return plan


def demo():
    """The right arm pours the cup of priming water into the pump head's open top, onto the leather, and sets the cup
    back on its spot. Once the cup is clear of the pump, the left arm pushes the handle down to its stop and lets it
    spring back up six times: three strokes draw the air out of the drop pipe, then three deliver 225 mL into the
    pail. Both arms go home."""
    right = pour_plan()
    wait = sum(w.duration for w in right[:-7])  # the cup is back over its spot, clear of the pump
    return {"right": right, "left": pump_plan(wait)}
