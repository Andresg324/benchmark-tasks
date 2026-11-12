"""Draining a small round stock-water tank by siphon, at 1:5 scale. The tank stands on a concrete pad with 6 cm of
water in it; a wide black rubber tub stands on the table to its right, lower down. The hose's two ends stand on the
table: each is a short clear end tube taped to a weighted wooden stick, the intake (green tape) on the left and the
outlet (red tape) on the right, with the outlet's hose taped on up the stick's side to its top.

The start without a pump and without sucking on the hose:
sink the hose so it fills with water, close one end (a thumb over it, or the hose pinched shut), keep the other end
under water while the closed end is carried out over the rim and down below the tank's water, then open it; the water
runs until the tank is down to the intake. Lifted out open, the hose drains back and air gets in. The intake end is
light and the vinyl hose stiff, so once the hose is bent out over the rim the intake springs up out of the water unless
it is held down.
With no clip or weight on the rim, one hand has to hold the intake down while the other carries the outlet.

Model (siphonkit, on pourkit): no fluid in the sim. The hose fills in 2 s with both ends and the intake's hose under
the water, the outlet open and no more than 10 mL of air left where the outlet's taped run stands above the water; the outlet is closed (pinched) while a gripper's pads squeeze the stick across the taped
hose; while primed, open and with the intake under, water runs Q = k sqrt(head) into the vessel under the outlet, else
onto the floor; the prime is lost (latched) when the intake comes out, the outlet is above the tank's water while open,
or the tank is drawn down to the intake.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (modeled props; nothing on disk changes)
import pourkit as pk  # noqa: E402  (the water in the tank and the tub)
import siphonkit as sk  # noqa: E402  (import last: its hook runs after yamkit's and pourkit's)
import guardkit as gk  # noqa: E402  (grip, release and arms-home checks)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Drain the stock tank into the tub with the hose."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still

# --- Scene (m): x forward from the arm bases, y to the robot's left, z up from the table top.
TANK = (0.33, 0.0)  # the tank's center
PAD = (0.19, 0.23, 0.037)  # concrete pad under it
TANK_R, TANK_WALL, TANK_FLOOR, TANK_H = 0.100, 0.005, 0.005, 0.063  # inside radius; wall; floor; rim above the pad
WATER = 0.050  # m of water at the start (1,571 mL)
TUB = (0.27, -0.215)
TUB_R, TUB_WALL, TUB_FLOOR, TUB_H = 0.100, 0.005, 0.003, 0.065

# a hose end: a 2.5 cm square stick on a 4 cm foot; a 1.4 cm clear end tube taped up its +y face
FOOT = (0.04, 0.04, 0.006)
STICK = (0.025, 0.025, 0.079)  # from the foot's top
END_H = FOOT[2] + STICK[2]  # 8.5 cm
COL_R = 0.007  # the end tube's (and hose's) outside radius
COL_Y = STICK[1] / 2 + COL_R  # its axis, in the end's frame
MOUTH = (0.0, COL_Y, 0.008)  # the open end, 2 mm above the foot's top
TUBE_TOP = (0.0, COL_Y, 0.035)  # where the soft hose joins the end tube
RUN_TOP = (0.0, COL_Y, END_H)  # the outlet's hose, taped up the stick, leaves over its top
INTAKE0, OUTLET0 = (0.22, 0.20), (0.21, -0.09)  # where the ends stand at the start
OUTLET_MASS = 0.12  # kg: stick, steel foot, tube and taped hose
INTAKE_MASS = 0.04  # kg: stick, plastic foot and tube (light: the hose lifts it unless it is held)

HOSE_R, HOSE_L = 0.00635, 0.6  # 12.7 mm (1/2 in) inside, 60 cm
FILL_S = 2.0  # s for a sunk hose to fill (bubbles stop)

PINE = kit.Look((0.80, 0.66, 0.46), 0.7)
STEEL = kit.Look((0.40, 0.41, 0.43), 0.4, 0.6)
PLASTIC = kit.Look((0.20, 0.45, 0.25), 0.6)
CLEAR = kit.Look((0.90, 0.95, 1.00), 0.05, 0.0, 0.3)
HOSE = kit.Look((0.86, 0.88, 0.86), 0.35)
GREEN = kit.Look((0.15, 0.60, 0.20), 0.5)
RED = kit.Look((0.85, 0.12, 0.10), 0.5)
POLY = kit.Look((0.52, 0.56, 0.60), 0.6)
CONCRETE = kit.Look((0.62, 0.61, 0.58), 0.9)
RUBBER = kit.Look((0.10, 0.10, 0.10), 0.8)


def _ring(r_out, h, s, z0, n=48):
    """A round wall as a ring of small boxes (colliders are axis-aligned)."""
    r = r_out - s / 2
    return [kit.Collider((r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), z0 + h / 2),
                         size=(s, s, h)) for k in range(n)]


def _tank():
    """The pad and the tank on it, fixed; origin on the table under the tank's center."""
    ro = TANK_R + TANK_WALL
    z0 = PAD[2]
    m = (kit.box((0, 0, z0 / 2), PAD, CONCRETE),
         kit.lathe([(0.0, z0), (ro, z0), (ro, z0 + TANK_H), (TANK_R, z0 + TANK_H), (TANK_R, z0 + TANK_FLOOR),
                    (0.0, z0 + TANK_FLOOR)], POLY))
    c = [kit.Collider((0, 0, z0 / 2), size=PAD), kit.Collider((0, 0, z0 + TANK_FLOOR / 2), size=(0.14, 0.14, TANK_FLOOR))]
    c += _ring(ro, TANK_H, 0.006, z0)
    return kit.Prop("tank", pos=(*TANK, TOP), size=(PAD[0], PAD[1], z0 + TANK_H), meshes=m, colliders=tuple(c))


def _tub():
    ro = TUB_R + TUB_WALL
    m = (kit.lathe([(0.0, 0.0), (ro, 0.0), (ro, TUB_H), (TUB_R, TUB_H), (TUB_R, TUB_FLOOR), (0.0, TUB_FLOOR)], RUBBER),)
    c = [kit.Collider((0, 0, TUB_FLOOR / 2), size=(0.14, 0.14, TUB_FLOOR))] + _ring(ro, TUB_H, 0.006, 0.0)
    return kit.Prop("tub", pos=(*TUB, TOP), size=(2 * ro, 2 * ro, TUB_H), meshes=m, colliders=tuple(c))


def _end(name, xy, tape, run, yaw=0.0):
    """A hose end, origin under its foot. `run`: the hose taped on up the stick to its top and a steel foot (the
    outlet); else a plastic foot (the light intake)."""
    sz = STICK[2]
    m = [kit.box((0, 0, FOOT[2] / 2), FOOT, STEEL if run else PLASTIC), kit.box((0, 0, FOOT[2] + sz / 2), STICK, PINE),
         kit.cylinder((0, COL_Y, (MOUTH[2] + TUBE_TOP[2]) / 2), COL_R, TUBE_TOP[2] - MOUTH[2], CLEAR)]
    if run:
        m.append(kit.cylinder((0, COL_Y, (TUBE_TOP[2] + END_H) / 2), COL_R, END_H - TUBE_TOP[2], HOSE))
    for z in (0.015, 0.029):  # tape wraps
        m.append(kit.box((0, COL_R, z), (STICK[0] + 0.002, STICK[1] + 2 * COL_R + 0.002, 0.006), tape))
    c = [kit.Collider((0, 0, FOOT[2] / 2), size=FOOT), kit.Collider((0, 0, FOOT[2] + sz / 2), size=STICK),
         kit.Collider((0, COL_Y, (MOUTH[2] + TUBE_TOP[2]) / 2), size=(2 * COL_R, 2 * COL_R, TUBE_TOP[2] - MOUTH[2]))]
    if run:  # the hose run under its tape: a strip the stick's width (the fingers bear on all of it)
        c.append(kit.Collider((0, COL_Y, (TUBE_TOP[2] + END_H) / 2), size=(STICK[0], 2 * COL_R, END_H - TUBE_TOP[2])))
        m.append(kit.box((0, COL_Y, (TUBE_TOP[2] + END_H) / 2), (STICK[0], 2 * COL_R - 0.002, END_H - TUBE_TOP[2] - 0.004), tape))
    return kit.Prop(name, pos=(*xy, TOP), size=(FOOT[0], STICK[1] + 2 * COL_R + 0.008, END_H), mass=OUTLET_MASS if run else INTAKE_MASS, yaw=yaw,
                    meshes=tuple(m), colliders=tuple(c))


OUTLET_YAW = math.pi / 2 + 0.4  # the outlet stands with its taped hose toward the robot (-x), turned 23 degrees
# so the right hand pinching it keeps its wrist camera clear of the left hand
OBJECTS = [_tank(), _tub(), _end("intake", INTAKE0, GREEN, False), _end("outlet", OUTLET0, RED, True, OUTLET_YAW)]

FLOOR_Z = TOP + PAD[2] + TANK_FLOOR  # the tank's inside floor
SURFACE0 = FLOOR_Z + WATER
pk.setup([pk.Vessel("tank", sections=((TANK_R, TANK_H - TANK_FLOOR),), floor=PAD[2] + TANK_FLOOR,
                    fill={"water": math.pi * TANK_R ** 2 * WATER * 1e6}),
          pk.Vessel("tub", sections=((TUB_R, TUB_H - TUB_FLOOR),), floor=TUB_FLOOR)],
         liquids={"water": (0.45, 0.62, 0.55)})
pk.set_floor(TOP)


# --- Grips (guardkit parts in an end's own frame).
STICK_PART = gk.Block((0, 0, FOOT[2] + STICK[2] / 2), STICK)
COLUMN = gk.Block((0, COL_Y, END_H / 2), (STICK[0], 2 * COL_R, END_H))
END_PARTS = (gk.Block((0, 0, FOOT[2] / 2), FOOT), STICK_PART, COLUMN)
ACROSS = gk.band(STICK[0])  # fingers closed on the bare stick (2.5 cm, pads on its plain faces)
OVER_HOSE = gk.band(STICK[1] + 2 * COL_R)  # fingers closed across the stick and the taped hose (3.9 cm)
PINCH_Z = 0.050  # the tool point at least 5 cm up the outlet: pads on the taped hose above the end tube
PINCH_DEG = 30.0  # the fingers' closing line within 30 degrees of square across the hose


def _rot(q):
    return gk._rot(q)


def gripping(objects, name):
    """Grippers with a real grip on end `name`: tool point within 1 cm of the stick and the fingers stopped by it
    (on its plain faces, or across it and the tube)."""
    out = []
    for g in gk.GRIPPERS:
        if g not in objects:
            continue
        op = kit.opening(objects, g)
        if (ACROSS[0] <= op <= ACROSS[1] or OVER_HOSE[0] <= op <= OVER_HOSE[1]) and \
                gk.dist(objects, name, objects[g].pos, (STICK_PART, COLUMN)) <= gk.GRIP_PAD:
            out.append(g)
    return tuple(out)


def capped(objects):
    """The outlet's hose pinched shut: a gripper closed across the stick and the taped hose (3.9 cm), its tool point
    at least 5 cm up the stick (on the hose run, above the end tube) and its fingers closing within 30 degrees of the
    stick's y axis (pads on the hose and the opposite face, not on the two plain faces)."""
    for g in gripping(objects, "outlet"):
        if not OVER_HOSE[0] <= kit.opening(objects, g) <= OVER_HOSE[1]:
            continue
        if gk.local(objects, "outlet", objects[g].pos)[2] < PINCH_Z:
            continue
        fx = _rot(objects[g].quat_xyzw)[:, 0]  # the finger-closing line
        ey = _rot(objects["outlet"].quat_xyzw)[:, 1]
        if abs(float(fx @ ey)) >= math.cos(math.radians(PINCH_DEG)):
            return True
    return False


def _world(objects, name, p):
    o = objects[name]
    return np.asarray(o.pos, float) + _rot(o.quat_xyzw) @ np.asarray(p, float)


class _Float(gk._PerStep):
    """The intake end is light (40 g: a pine stick on a plastic foot) and the hose is stiff vinyl: once the hose is bent
    out over the rim toward the tub (the outlet's mouth outside the tank), it springs the intake end up off the floor
    and out of the water unless a hand holds it down. True while that holds: the intake is taken as up at the rim."""

    def _fresh(self):
        self.up = False

    def feed(self, objects):
        if "left_gripper" not in objects or "intake" not in objects:
            return False  # the reset draw (no grippers yet)
        if self._tick(objects):
            self.up = (not gripping(objects, "intake")
                       and sk._inside(objects, "tank", _world(objects, "intake", MOUTH))
                       and not sk._inside(objects, "tank", _world(objects, "outlet", MOUTH)))
        return self.up


def trapped_air(objects):
    """mL of air left in the outlet's taped hose run where it stands above the tank's surface while sunk (up one side of
    the stick to its top and back down to the water), with the outlet standing in the tank; else 0."""
    if not sk._inside(objects, "tank", _world(objects, "outlet", MOUTH)):
        return 0.0
    over = max(0.0, float(_world(objects, "outlet", RUN_TOP)[2]) - sk._surface_z(objects, "tank"))
    return 2.0 * over * math.pi * HOSE_R ** 2 * 1e6


RIM_DIR = (np.array(TUB) - np.array(TANK)) / np.linalg.norm(np.array(TUB) - np.array(TANK))  # the hose's way over
AIR_MAX = 10.0  # mL (an eighth of the 76 mL hose): a slug the starting flow (0.6 m/s in a 12.7 mm bore) sweeps
# out; an outlet standing on the tank floor leaves 9 mL in its run above the water
FLOAT = _Float()

sk.setup(sk.Siphon(source="tank", receiver="tub", intake=sk.Point("intake", MOUTH), outlet=sk.Point("outlet", MOUTH),
                   via=(sk.Point("intake", TUBE_TOP), sk.Point("outlet", RUN_TOP)),
                   sink=(sk.Point("intake", MOUTH), sk.Point("intake", TUBE_TOP), sk.Point("outlet", MOUTH),
                         sk.Point("outlet", TUBE_TOP)),
                   rim=sk.Point("tank", (RIM_DIR[0] * (TANK_R + TANK_WALL), RIM_DIR[1] * (TANK_R + TANK_WALL),
                                         PAD[2] + TANK_H + 0.006)),
                   radius=HOSE_R, length=HOSE_L, prime="submerge", submerge_s=FILL_S, capped=capped,
                   pulled=FLOAT.feed, trapped=trapped_air, trapped_max=AIR_MAX), color=(0.45, 0.62, 0.55))

# --- The rule's numbers (reasons in score() and the README).
DRAWN_Z = FLOOR_Z + MOUTH[2] + sk._CFG["siphon"].margin  # the tank's surface when an intake standing on its floor gulps air
DRAIN_ML = math.pi * TANK_R ** 2 * (SURFACE0 - DRAWN_Z) * 1e6  # 1,162 mL above that
FULL = 0.90  # share of DRAIN_ML in the tub: an intake held 1 cm off the floor leaves 27 % of it behind
SOME = 0.25  # under a quarter in the tub: nothing worth calling a drain
SPILL_MAX = 20.0  # mL: a quarter second of the starting stream (about 81 mL/s) missing the tub
HOME = gk.ArmsHome()


def into_tub(objects):
    return pk.poured(objects, "tank", "tub")


def placed(objects):
    """The outlet's mouth inside the tub; the intake's mouth inside the tank or the tub."""
    return (sk._inside(objects, "tub", _world(objects, "outlet", MOUTH))
            and any(sk._inside(objects, v, _world(objects, "intake", MOUTH)) for v in ("tank", "tub")))


def _job(objects):
    if not all(gk.released_clear(objects, n, END_PARTS, 0.01) for n in ("intake", "outlet")):
        return 0.0  # a hose end still in a hand
    if pk.spilled(objects) > SPILL_MAX:
        return 0.0
    got = into_tub(objects)
    if got < SOME * DRAIN_ML:
        return 0.0  # never started (or barely)
    if sk.lost_prime(objects, "outlet_high") or sk.lost_prime(objects, "intake_out"):
        return 0.25  # the prime was lost on the way: the hose drained back
    if not sk.lost_prime(objects, "drawn_down") or got < FULL * DRAIN_ML:
        return 0.5  # still running, or stopped by an intake held off the bottom
    if not placed(objects):
        return 0.75
    return 1.0


def _score(objects):
    HOME.feed(objects)
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not HOME.home(objects):
        return 0.9
    return job


def success(objects):
    return _score(objects) == 1.0


def score(objects):
    """1.0 when the tank has been drawn down to the intake (its surface at the intake's mouth: the siphon gulps air and
    stops by itself) with at least 90 % of the 1,162 mL above that level run into the tub, the prime never lost on the
    way (no intake out of the water, including an intake left unheld while the outlet is out of the tank, and no open
    outlet above the tank's surface), no more than 20 mL spilled, both hose ends let go (every tool point and pad tip
    1 cm clear) with the outlet's mouth in the tub and the intake's in the tank or the tub, and both arms back home and
    still (0.9 if not home). 0.75: all of that but an end left elsewhere. 0.5: primed and never lost, at least a
    quarter in the tub, but still running at the end or stopped short (under 90 %: the intake held off the bottom).
    0.25: at least a quarter in the tub but the prime lost on the way, even if it was started again. 0: under a quarter
    in the tub (nothing done, never primed, one hand only), more than 20 mL spilled, or an end still held.
    Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1.0."""
    return _score(objects)


# --- Demo: the left arm sets the intake on the tank's floor and holds it there; the right arm sets the outlet in the
# tank beside it and lets go while the hose fills, takes it again across the taped hose (pinching it shut), carries it
# out over the rim and down into the tub, and lets go (opening it); both arms go home while the tank drains.
GRIP_Z = 0.066  # tool point above the intake's bottom when gripping (pad tips 5.3 cm up, 3 mm over the water)
PINCH_GRIP_Z = 0.085  # on the outlet: level with its top, pads on the taped hose (the right hand reaches no lower
# in the tank while the left holds the intake)
REGRIP_TRIM = -0.032  # m: the right hand stops about 1.8 cm above its target here with the left hand in the tank
# (sim), so it is sent 3.2 cm lower (its pads then cover the top 2.5 cm of the taped hose)
REGRIP_SLIP = -0.016  # m: the re-grip lands 1.6 cm lower on the outlet than planned (tool point 6.9 cm up it), so the
# carry after it is sent that much lower to keep the outlet's bottom where planned
OVER_RIM_R = PAD[2] + TANK_H + 0.010  # the outlet's bottom carried 1 cm over the rim
OVER_RIM = PAD[2] + TANK_H + 0.013  # an end's bottom carried 1.3 cm over the tank's rim
IN_INTAKE = (TANK[0] + 0.040, TANK[1] + 0.045)  # where the intake stands in the tank (far left)
INTO_TANK = (TANK[0] - 0.020, TANK[1] + 0.060)  # where it crosses the rim on its way in
IN_OUTLET = (TANK[0] - 0.025, TANK[1] - 0.040)  # (near right: 6 cm nearer than the intake, so the hands clear)
LIFT_OUTLET = (IN_OUTLET[0] - 0.005, IN_OUTLET[1] - 0.008)  # up and toward the tub side (in reach at the top)
CROSS = (TANK[0] + RIM_DIR[0] * TANK_R, TANK[1] + RIM_DIR[1] * TANK_R)  # over the rim, toward the tub
IN_TUB = (TUB[0], TUB[1])
TANK_BED = PAD[2] + TANK_FLOOR  # an end standing on the tank's floor
HALF = -0.03  # fingers part-open (6 cm) inside the tank
EASED = 0.5 * FINGER_OPEN  # fingers eased off the outlet (4.7 cm apart, 4 mm clear of each side): open, but it can't turn
YAW_X = math.pi / 2  # fingers closing along x: on the intake's plain faces; across the outlet's taped hose


HOLD_S = 39.0  # s the left hand holds the intake down: until the tank is drawn down
DRAIN_WAIT_S = 6.0  # s at home at the end


def _at(xy, bottom, d=(0.0, 0.0), gz=GRIP_Z):
    return (xy[0] + d[0], xy[1] + d[1], TOP + bottom + gz)


def _ar(xy, bottom, d=(0.0, 0.0)):
    """The right hand on the outlet: pads at the top of its taped hose."""
    return _at(xy, bottom, d, PINCH_GRIP_Z)


def demo():
    W = Waypoint
    p = (-COL_R * math.sin(OUTLET_YAW), COL_R * math.cos(OUTLET_YAW))  # the grip's middle across the outlet's stick
    # and its taped hose (2.5 + 1.4 cm), toward the hose
    YR = OUTLET_YAW  # fingers closing square across the outlet's taped hose
    YL = YAW_X
    left = [W(2.0, _at(INTAKE0, 0.04), FINGER_OPEN, "over the intake", YL),
            W(1.2, _at(INTAKE0, 0.0), FINGER_OPEN, "around its stick", YL),
            W(0.6, _at(INTAKE0, 0.0), FINGER_CLOSED, "grip", YL),
            W(1.5, _at(INTAKE0, OVER_RIM), FINGER_CLOSED, "lift", YL),
            W(2.0, _at(INTO_TANK, OVER_RIM), FINGER_CLOSED, "over the tank", YL),
            W(1.0, _at(IN_INTAKE, OVER_RIM - 0.045), FINGER_CLOSED, "over its spot", YL),
            W(1.5, _at(IN_INTAKE, TANK_BED), FINGER_CLOSED, "onto the tank's floor", YL),
            W(HOLD_S, _at(IN_INTAKE, TANK_BED), FINGER_CLOSED, "hold it down", YL),
            W(0.6, _at(IN_INTAKE, TANK_BED), HALF, "let go", YL),
            W(0.8, _at(IN_INTAKE, OVER_RIM - 0.045), HALF, "up off it", YL),
            W(1.0, _at(INTO_TANK, OVER_RIM), HALF, "up out of the tank", YL)]
    right = [W(3.0, _ar(OUTLET0, 0.06, p), FINGER_OPEN, "wait", YR),
             W(2.0, _ar(OUTLET0, 0.04, p), HALF, "over the outlet", YR),
             W(1.5, _ar(OUTLET0, 0.0, p), HALF, "around it (part-open: the tank's rim is 4 cm off)", YR),
             W(1.0, _ar(OUTLET0, 0.0, p), FINGER_CLOSED, "grip", YR),
             W(2.0, _ar(OUTLET0, OVER_RIM_R, p), FINGER_CLOSED, "lift", YR),
             W(2.0, _ar(LIFT_OUTLET, OVER_RIM_R, p), FINGER_CLOSED, "over the tank", YR),
             W(1.5, _ar(IN_OUTLET, TANK_BED, p), FINGER_CLOSED, "into the water", YR),
             W(0.6, _ar(IN_OUTLET, TANK_BED, p), EASED, "ease the pinch: the hose fills", YR),
             W(3.1, _ar(IN_OUTLET, TANK_BED + REGRIP_TRIM, p), EASED, "wait for the bubbles to stop", YR),
             W(1.2, _ar(IN_OUTLET, TANK_BED + REGRIP_TRIM, p), FINGER_CLOSED, "pinch the hose shut", YR),
             W(1.2, _ar(IN_OUTLET, TANK_BED + 0.02, p), FINGER_CLOSED, "lift it, pinched", YR),
             W(1.5, _ar(LIFT_OUTLET, OVER_RIM_R + REGRIP_SLIP, p), FINGER_CLOSED, "up to the rim", YR),
             W(1.0, _ar(CROSS, OVER_RIM_R + REGRIP_SLIP + 0.005, p), FINGER_CLOSED, "out over the rim", YR),
             W(2.0, _ar(IN_TUB, OVER_RIM_R + REGRIP_SLIP, p), FINGER_CLOSED, "over the tub", YR),
             W(1.5, _ar(IN_TUB, TUB_FLOOR + REGRIP_SLIP, p), FINGER_CLOSED, "down into the tub", YR),
             W(0.6, _ar(IN_TUB, TUB_FLOOR, p), FINGER_OPEN, "let go: it runs", YR),
             W(1.0, _ar(IN_TUB, 0.09, p), FINGER_OPEN, "up and clear", YR)]
    t_left = sum(w.duration for w in left)
    t_right = sum(w.duration for w in right)
    return {"left": left + [kit.Home(2.5), kit.Home(DRAIN_WAIT_S, label="hold home")],
            "right": right + [kit.Home(2.5), kit.Home(t_left - t_right + DRAIN_WAIT_S, label="home while it drains")]}
