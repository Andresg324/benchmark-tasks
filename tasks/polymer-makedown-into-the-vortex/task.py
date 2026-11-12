"""A 250 mL beaker of water held in a collar on the bench, a dredger (a shaker with a side handle) holding 0.5 g of dry
polymer powder, and a glass stirring rod standing in its wooden rest.

Dry polymer (polyacrylamide flocculant, or a gum such as xanthan) is made up into a stock by sifting it slowly into
water that is already being stirred, so each grain is wetted on its own. Powder that lands on still water, or piles up
faster than the swirl spreads and wets it, gels on its outside within seconds into fish-eyes: lumps with a dry
core that never dissolve. So one hand keeps the rod going while the other sprinkles.

Model (dispersekit, read from the poses every control step): powder leaves the shaker once its cap points past 95
degrees from straight up, at a rate growing with the square of the extra tilt to 5 g/s cap straight down (about 0.07
g/s at 105 degrees), and falls plumb from the cap. The rod tip moving through the water drives a swirl that wets
floating powder one grain-layer at a time; powder floating 1 s on still water, or heaped thicker than one layer, gels
into fish-eyes for good.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props; nothing on disk changes)
import guardkit as gk  # noqa: E402  (release and arms-home checks)
import dispersekit as dk  # noqa: E402  (the powder model, updated every control step)
import yam_arena.demo as _yd  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = ("Make up the polymer stock: get all of the powder in the shaker dissolved into the water in the "
               "beaker, then put the rod and the shaker back.")
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
BEAKER = (0.28, 0.0)  # the beaker's axis
R_OUT, R_IN = 0.035, 0.033  # a 250 mL low-form Griffin beaker: 7 cm across ...
BEAKER_H = 0.095  # ... 9.5 cm tall
GLASS_FLOOR = 0.003  # its inside floor
WATER_ML = 200.0
DEPTH = WATER_ML * 1e-6 / (math.pi * R_IN ** 2)  # 5.85 cm of water
SURFACE = TOP + GLASS_FLOOR + DEPTH
COLLAR = (0.035, 0.041, 0.012)  # a fixed rubber collar round the foot: inner and outer radius, height

ROD_BLOCK = (0.28, 0.13)  # the rod's rest: a wooden block with a hole the rod stands in
BLOCK = (0.04, 0.04, 0.03)
HOLE = 0.018  # 1.8 cm square hole ...
HOLE_FLOOR = 0.005  # ... 2.5 cm deep
ROD_LEN, ROD_R = 0.15, 0.004  # glass stirring rod, 15 cm long, 8 mm across
SLEEVE_R, SLEEVE_L = 0.007, 0.04  # a silicone grip sleeve 1.4 cm across and 4 cm long round its middle ...
COLLAR_R, COLLAR_H = 0.011, 0.006  # ... with a collar 2.2 cm across at its lower end (it rests on the fingers)
COLLAR_Z = -(SLEEVE_L + COLLAR_H) / 2  # the collar's center below the rod's middle
ROD_MASS = 0.030  # kg: 19 g of glass and 11 g of silicone
ROD_START = (*ROD_BLOCK, TOP + HOLE_FLOOR + ROD_LEN / 2)  # the rod's middle (its origin), standing in the hole

SHAKER_AT = (0.31, -0.11)  # the shaker (a dredger) stands on the bench
BODY_R, BODY_H = 0.02, 0.09  # a round white HDPE body 4 cm across, 9 cm tall ...
CAP_R, CAP_H = 0.021, 0.012  # ... a perforated screw cap ...
HANDLE_R, HANDLE_L = 0.011, 0.05  # ... and a round handle 2.2 cm across, 5 cm long, out of its side toward -y ...
GRIP_H = 0.05  # ... at the height of its center of mass, so tipping it about the handle puts no twist on the grip
HANDLE = (0.0, -(BODY_R + HANDLE_L / 2 - 0.002), GRIP_H)  # the handle's center in the shaker's frame
SHAKER_MASS = 0.040
DOSE = 0.5  # g of dry polymer in the shaker: 0.5 g in 200 mL is a 0.25 % stock
CAP = (0.0, 0.0, BODY_H + CAP_H)  # the perforated face, in the shaker's frame (origin at the middle of its foot)

# --- The powder (dispersekit). The cap's seven 3.6 mm holes let nothing through until it points 95 degrees from
# straight up; past that the flow grows with the square of the extra tilt, to 5 g/s cap straight down (180 degrees):
# about 0.07 g/s at 105 degrees, a trickle, and the whole dose in a tenth of a second turned over.
SHAKE_FROM, SHAKE_RAMP, SHAKE_RATE = 95.0, 85.0, 5.0

# --- The rule's numbers (reasons in score() and the README).
ADDED_MIN = 0.95  # at least 95 % of the dose on the water: the stock's strength within 5 % of its 0.25 % label
LUMP_MAX = 0.03  # fish-eyes (and powder still floating, which gels once the stirring stops) at most 3 % of the dose
SPILL_MAX = 0.01  # g of powder beside the beaker at most: wetted polyacrylamide on a bench or floor is slick as ice
UPRIGHT_DEG = 10.0  # the shaker stood back up within 10 degrees ...
DOWN = 0.005  # ... with its foot within 5 mm of the bench
REST_SLACK = 0.003  # the rod's tip within 3 mm of the rest's hole, below the rest's top
CLEAR = 0.01  # m: every tool point and pad tip this far clear of a tool is let go

GLASS = kit.Look((0.85, 0.92, 0.95), 0.05, 0.0, 0.25)
WATER = kit.Look((0.55, 0.75, 0.92), 0.1, 0.0, 0.55)
RUBBER = kit.Look((0.15, 0.15, 0.16), 0.9)
WOOD = kit.Look((0.62, 0.45, 0.28), 0.7)
HDPE = kit.Look((0.95, 0.95, 0.93), 0.6)
CAPLOOK = kit.Look((0.85, 0.30, 0.20), 0.5)
HOLES = kit.Look((0.10, 0.10, 0.10), 0.9)
SILICONE = kit.Look((0.20, 0.45, 0.80), 0.6)
ROD_GLASS = kit.Look((0.80, 0.88, 0.90), 0.1, 0.0, 0.6)


def _beaker():
    m = [kit.lathe([(R_OUT - 0.002, 0.0), (R_OUT, 0.002), (R_OUT, BEAKER_H), (R_IN, BEAKER_H), (R_IN, GLASS_FLOOR),
                    (0.0, GLASS_FLOOR)], GLASS, n=40),
         kit.cylinder((0, 0, GLASS_FLOOR + DEPTH / 2), R_IN - 0.0005, DEPTH, WATER, n=40),
         kit.lathe([(COLLAR[0], 0.0), (COLLAR[1], 0.0), (COLLAR[1], COLLAR[2]), (COLLAR[0], COLLAR[2])], RUBBER)]
    floor = kit.Collider((0, 0, GLASS_FLOOR / 2), size=(2 * R_IN, 2 * R_IN, GLASS_FLOOR))
    return kit.Prop("beaker", pos=(*BEAKER, TOP), size=(2 * COLLAR[1], 2 * COLLAR[1], BEAKER_H), meshes=tuple(m),
                    colliders=(floor,) + _wall())


def _wall(n=40):
    """The glass wall as `n` square posts 5.8 mm on a side, edge to edge round the axis (yamkit's box colliders don't
    turn, so small posts stand in for a curved wall)."""
    r = (R_IN + R_OUT) / 2
    s = 2 * math.pi * r / n + 0.0005
    return tuple(kit.Collider((r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), BEAKER_H / 2),
                              size=(s, s, BEAKER_H)) for k in range(n))


def _rod():
    m = (kit.cylinder((0, 0, 0), ROD_R, ROD_LEN, ROD_GLASS, n=16),
         kit.cylinder((0, 0, 0), SLEEVE_R, SLEEVE_L, SILICONE, n=20),
         kit.cylinder((0, 0, COLLAR_Z), COLLAR_R, COLLAR_H, SILICONE, n=20))
    c = (kit.Collider((0, 0, 0), radius=ROD_R, height=ROD_LEN),
         kit.Collider((0, 0, 0), radius=SLEEVE_R, height=SLEEVE_L),
         kit.Collider((0, 0, COLLAR_Z), radius=COLLAR_R, height=COLLAR_H))
    return kit.Prop("rod", pos=ROD_START, size=(2 * COLLAR_R, 2 * COLLAR_R, ROD_LEN), mass=ROD_MASS, meshes=m,
                    colliders=c)


def _rest():
    bx, by, bz = BLOCK
    wall = (bx - HOLE) / 2
    parts = [((0, 0, HOLE_FLOOR / 2), (bx, by, HOLE_FLOOR))]
    for s in (-1, 1):
        parts.append(((s * (HOLE + wall) / 2, 0, bz / 2), (wall, by, bz)))
        parts.append(((0, s * (HOLE + wall) / 2, bz / 2), (HOLE, wall, bz)))
    return kit.Prop("rod_rest", pos=(*ROD_BLOCK, TOP), size=BLOCK,
                    meshes=tuple(kit.box(cn, sz, WOOD) for cn, sz in parts),
                    colliders=tuple(kit.Collider(cn, size=sz) for cn, sz in parts))


def _shaker():
    body = (0, 0, BODY_H / 2)
    cap = (0, 0, BODY_H + CAP_H / 2)
    m = [kit.cylinder(body, BODY_R, BODY_H, HDPE, n=28), kit.cylinder(cap, CAP_R, CAP_H, CAPLOOK, n=28),
         kit.cylinder(HANDLE, HANDLE_R, HANDLE_L, CAPLOOK, axis="y", n=20)]
    for i in range(7):  # the holes in the cap's face
        a = 2 * math.pi * i / 6
        r = 0.0 if i == 6 else 0.009
        m.append(kit.cylinder((r * math.cos(a), r * math.sin(a), BODY_H + CAP_H + 0.0003), 0.0018, 0.001, HOLES,
                              n=10))
    c = (kit.Collider(body, radius=BODY_R, height=BODY_H), kit.Collider(cap, radius=CAP_R, height=CAP_H),
         kit.Collider(HANDLE, radius=HANDLE_R, height=HANDLE_L, axis="Y"))
    return kit.Prop("shaker", pos=(*SHAKER_AT, TOP), size=(2 * CAP_R, 2 * CAP_R + 2 * HANDLE_L, BODY_H + CAP_H),
                    mass=SHAKER_MASS,
                    meshes=tuple(m), colliders=c)


OBJECTS = [_beaker(), _rest(), _rod(), _shaker()]

ROD_PARTS = (gk.Seg((0, 0, -ROD_LEN / 2), (0, 0, ROD_LEN / 2), ROD_R),
             gk.Seg((0, 0, -SLEEVE_L / 2), (0, 0, SLEEVE_L / 2), SLEEVE_R),
             gk.Seg((0, 0, COLLAR_Z - COLLAR_H / 2), (0, 0, COLLAR_Z + COLLAR_H / 2), COLLAR_R))
SHAKER_PARTS = (gk.Seg((0, 0, 0), (0, 0, BODY_H), BODY_R), gk.Seg((0, 0, BODY_H), (0, 0, BODY_H + CAP_H), CAP_R),
                gk.Seg(np.add(HANDLE, (0, -HANDLE_L / 2, 0)), np.add(HANDLE, (0, HANDLE_L / 2, 0)), HANDLE_R))
HANDLE_PART = SHAKER_PARTS[2]
ROD_TIP = (0.0, 0.0, -ROD_LEN / 2)

dk.setup(dk.Beaker("beaker", radius=R_IN, floor=GLASS_FLOOR, fill=WATER_ML),
         shakers=[dk.Shaker("shaker", spout=CAP, rate=SHAKE_RATE, load=DOSE, tilt=SHAKE_FROM, ramp=SHAKE_RAMP,
                            power=2.0)],
         stirrers=[dk.Stirrer("rod", tip=ROD_TIP)])

HOME = gk.ArmsHome()
UNMANNED = gk.Latch(steps=3)  # powder coming out for 3 steps in a row while the dredger is not in one hand and the rod
# in the other (a one- or two-step grip flicker in the sim does not count)
HANDLE_BAND = (0.06, gk.band(2 * HANDLE_R)[1])  # finger opening of a real grip on the handle: up to the 2.2 cm handle's
# width plus 3 %; down to 0.06 because the fingers pinch the round handle off its center (a chord: the sim's demo reads
# 0.09-0.17); an empty fist reads 0, a half-open hand 0.5
SLEEVE_BAND = gk.band(2 * SLEEVE_R)  # a grip on the rod's 1.4 cm sleeve: 0.075-0.18 (the demo reads 0.13-0.14)


def pouring(objects=None):
    """Powder fell from the cap this step."""
    return bool(dk._state().streams)


def two_handed(objects):
    """A hand grips the dredger's handle (tool point within 1 cm of it, fingers stopped by it) and a different hand
    grips the rod's sleeve the same way."""
    on_shaker = gk.grip_on(objects, "shaker", HANDLE_PART, HANDLE_BAND, pad=0.01)
    on_rod = gk.grip_on(objects, "rod", ROD_PARTS[1], SLEEVE_BAND, pad=0.01)
    return bool(on_shaker) and any(g not in on_shaker for g in on_rod)


def _up(objects, name):
    q = objects[name].quat_xyzw
    return 1 - 2 * (float(q[0]) ** 2 + float(q[1]) ** 2)  # cos of the tilt from upright


def shaker_down(objects):
    """The shaker standing up on the bench: within 10 degrees of upright, its foot within 5 mm of the bench."""
    o = objects["shaker"]
    return _up(objects, "shaker") >= math.cos(math.radians(UPRIGHT_DEG)) and float(o.pos[2]) - TOP <= DOWN


def rod_tip(objects):
    o = objects["rod"]
    return np.asarray(o.pos, float) + dk._rotm(o.quat_xyzw) @ np.asarray(ROD_TIP)


def rod_in_rest(objects):
    """The rod stood back in its rest: its tip in the hole (within 3 mm of it), below the rest's top."""
    tip = rod_tip(objects)
    near = HOLE / 2 + REST_SLACK
    return (abs(tip[0] - ROD_BLOCK[0]) <= near and abs(tip[1] - ROD_BLOCK[1]) <= near
            and tip[2] <= TOP + BLOCK[2])


def released(objects):
    return (gk.released_clear(objects, "rod", ROD_PARTS, CLEAR)
            and gk.released_clear(objects, "shaker", SHAKER_PARTS, CLEAR))


def stock(objects=None):
    """(share of the dose on the water, share lumped or still floating, grams spilled)."""
    return dk.added() / DOSE, (dk.lumped() + dk.surface()) / DOSE, dk.spilled()


def _job(objects):
    if UNMANNED.feed(objects, pouring(objects) and not two_handed(objects)):
        return 0.0  # powder let out of a dredger nobody holds (parked tipped), or with no other hand on the rod
    if not released(objects) or not (shaker_down(objects) and rod_in_rest(objects)):
        return 0.0  # a tool still in a hand, or not put back
    added, lumped, spilled = stock(objects)
    if added >= ADDED_MIN and lumped <= LUMP_MAX and spilled <= SPILL_MAX:
        return 1.0
    return 0.0  # fish-eyes (on still water, shaken in too fast, one hand at a time), not all added, or spilled


def _score(objects):
    HOME.feed(objects)
    UNMANNED.feed(objects, pouring(objects) and not two_handed(objects))
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not HOME.home(objects):
        return 0.9
    return job


def success(objects):
    return _score(objects) == 1.0


def score(objects):
    """The powder model tracks grams in the shaker, floating on the water, dispersed, lumped and spilled. 1.0 if at
    least 95 % of the 0.5 g dose has landed on the water (the stock within 5 % of its strength), at most 3 % of the
    dose (0.015 g) has gelled into fish-eyes or still floats (it gels once the stirring stops), at most 0.01 g fell
    beside the beaker, the shaker stands on the bench within 10 degrees of upright (foot within 5 mm), the rod
    stands back in its rest (tip in the hole, below the rest's top), every tool point and pad tip is 1 cm clear of
    both tools, and both arms are back home and still (0.9 if not home). 0 otherwise: fish-eyes from powder on still
    water (shaken in before the stirring starts, during a pause of over a second, or by one hand switching between
    shaker and rod), from powder shaken in faster than the swirl spreads it (the shaker turned over), powder left in
    the shaker or spilled, or a tool still held or not put back. Also 0, for good, once powder has come out of the
    cap for 3 control steps in a row (0.15 s) while no hand grips the dredger's handle or no other hand grips the
    rod's sleeve (tool point within 1 cm, fingers stopped by it): a dredger parked tipped over the beaker doses
    itself, and one hand cannot both sprinkle and stir. Possible scores: 0, 0.9, 1.0."""
    return _score(objects)


# --- Demo. Left arm: take the rod from its rest in a side grip, carry it into the beaker and stir in circles. Right
# arm: take the shaker in a side grip, bring it over the beaker and tip it just past level so the powder trickles in
# while the left arm stirs, then stand it back on the bench. The left arm stirs 3 s more, lifts the rod out, stands
# it back in its rest and lets go. Both go home.
L_YAW, L_ROLL = -math.pi / 2, -1.35  # left: hand pointing toward -y, fingers closing along x
R_YAW, R_ROLL, R_P0 = math.pi / 2, 1.35, 0.5  # right: hand pointing toward +y (the grip of the pour pilots)
_UP = -(kit._rot_xyz(R_ROLL, R_P0) @ _yd.grip_rotation(R_YAW))[:, 1]  # the back of the right hand: the open fingers
# straddle the handle and come down onto it along this direction
STIR = (0.268, 0.008)  # circle center of the tool point (the rod's tip runs about 4 mm toward -x of it) ...
STIR_R = 0.015  # ... 1.5 cm radius (the arm's lag makes the tip's circle about 1 cm; the rod clears the cap)
STIR_Z = 0.035  # tip 3.5 cm above the table (2.6 cm under the surface)
LIFT_Z = 0.185  # tool point high enough to carry the rod's tip over the rim
TIP_LAG = (0.006, 0.017)  # the rod's tip sits this far -x and -y of the tool point (the side grip holds it 13
# degrees off vertical; seen in the sim)
REV_S = 0.6  # s per turn of the rod
STIR_FROM, STIR_TO = 10.3, 31.0  # s: the left arm stirs over this span
CAP_AT = (0.267, -0.022, TOP + 0.15)  # where the cap is aimed: in the sim the dredger turns about 19 degrees in the
# hand on the way over and its cap hangs 3.1 cm further +x and 1 cm +y, at x 29.8, y -1.2 (2.1 cm off the axis)
GRIP_DY = -0.012  # the tool point 1.2 cm behind the handle's center (more of the handle in the fingers)
GRIP = np.add(HANDLE, (0.0, GRIP_DY, 0.0))  # the tool point in the shaker's frame
TURN = math.radians(107.0)  # commanded; the dredger holds about 2 degrees less in the hand
HELD_DEG = 105.0  # the tilt it holds over the water in the sim: the cap 15 degrees below level, a trickle
SPRINKLE_S = 10.0  # held there until it is empty


def _lw(dur, pos, finger, label):
    return kit.Tilted(dur, tuple(float(v) for v in pos), finger, label, L_YAW, L_ROLL, 0.0)


def _rw(dur, pos, finger, label, turn=0.0):
    return kit.Tilted(dur, tuple(float(v) for v in pos), finger, label, R_YAW, R_ROLL, R_P0 - turn)


def shaker_tcp(cap, turn):
    """Tool point that puts the cap at `cap` with the shaker turned `turn` rad (top toward -x) from upright."""
    q = kit._rot_xyz(R_ROLL, R_P0 - turn) @ kit._rot_xyz(R_ROLL, R_P0).T
    return np.asarray(cap, float) - q @ (np.asarray(CAP) - GRIP)


def _left():
    x, y, z0 = ROD_START
    tcp0 = np.array([x, y, z0])
    out = [_lw(3.5, tcp0 + [0, 0.045, 0.01], FINGER_OPEN, "beside the rod"),
           _lw(1.2, tcp0, FINGER_OPEN, "around the sleeve"),
           _lw(0.6, tcp0, FINGER_CLOSED, "grip"),
           _lw(1.5, (x, y, TOP + LIFT_Z), FINGER_CLOSED, "lift out of the rest"),
           _lw(2.0, (*STIR, TOP + LIFT_Z), FINGER_CLOSED, "over the beaker"),
           _lw(1.5, (STIR[0] + STIR_R, STIR[1], TOP + STIR_Z + ROD_LEN / 2), FINGER_CLOSED, "tip into the water")]
    n = 8  # waypoints per turn
    for k in range(1, n * int(round((STIR_TO - STIR_FROM) / REV_S)) + 1):
        a = 2 * math.pi * k / n
        out.append(_lw(REV_S / n, (STIR[0] + STIR_R * math.cos(a), STIR[1] + STIR_R * math.sin(a),
                                   TOP + STIR_Z + ROD_LEN / 2), FINGER_CLOSED, "stir"))
    over = (x + TIP_LAG[0], y + TIP_LAG[1], TOP + LIFT_Z)
    down = (x + TIP_LAG[0], y + TIP_LAG[1], TOP + HOLE_FLOOR + 0.004 + ROD_LEN / 2)
    out += [_lw(1.5, (*STIR, TOP + LIFT_Z), FINGER_CLOSED, "lift out of the water"),
            _lw(2.5, over, FINGER_CLOSED, "over the rest"),
            _lw(1.5, down, FINGER_CLOSED, "into the hole"),
            _lw(0.6, down, FINGER_OPEN, "let go"),
            _lw(1.2, np.array(down) + [0, 0.03, 0.02], FINGER_OPEN, "back off"),
            kit.Home(4.0), kit.Home(SUCCESS_HOLD_S + 1.5, label="hold home")]
    return out


def _right():
    x, y = SHAKER_AT
    tcp0 = np.array([x, y, TOP]) + GRIP
    over = shaker_tcp(CAP_AT, TURN)
    return [kit.Home(5.0, label="wait for the stirring"),
            _rw(4.5, tcp0 + 0.06 * _UP + [0, 0, 0.02], FINGER_OPEN, "above the handle"),
            _rw(1.0, tcp0 + 0.06 * _UP, FINGER_OPEN, "fingers either side of the handle"),
            _rw(1.2, tcp0, FINGER_OPEN, "down round the handle"),
            _rw(0.6, tcp0, FINGER_CLOSED, "grip"),
            _rw(1.0, tcp0 + [0, 0, 0.08], FINGER_CLOSED, "lift"),
            _rw(3.0, shaker_tcp(np.array(CAP_AT) + [0.05, -0.04, 0.02], TURN / 2), FINGER_CLOSED, "toward the beaker",
                TURN / 2),
            _rw(1.5, shaker_tcp(np.array(CAP_AT) + [0.02, -0.02, 0.0], math.radians(90.0)), FINGER_CLOSED,
                "beside the rod", math.radians(90.0)),
            _rw(1.0, shaker_tcp(CAP_AT, math.radians(90.0)), FINGER_CLOSED, "cap over the water", math.radians(90.0)),
            _rw(1.0, over, FINGER_CLOSED, "tip past level", TURN),
            _rw(SPRINKLE_S, over, FINGER_CLOSED, "sprinkle", TURN),
            _rw(1.5, shaker_tcp(np.array(CAP_AT) + [0.02, 0, 0.03], 0.3), FINGER_CLOSED, "tip back up", 0.3),
            _rw(3.0, tcp0 + [0, 0, 0.08], FINGER_CLOSED, "over its spot"),
            _rw(1.2, tcp0 + [0, 0, 0.003], FINGER_CLOSED, "stand it down"),
            _rw(0.6, tcp0 + [0, 0, 0.003], FINGER_OPEN, "let go"),
            _rw(1.2, tcp0 + 0.06 * _UP, FINGER_OPEN, "up off the handle"),
            kit.Home(4.5), kit.Home(SUCCESS_HOLD_S + 1.5, label="hold home")]


def demo():
    """Left arm stirs with the rod; right arm trickles the powder in while it does; tools put back; home."""
    return {"left": _left(), "right": _right()}

