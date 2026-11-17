"""A glucose meter's control-solution check, as a nurse runs it before using a meter or a new bottle of strips.

On a clean counter: an Ascensia Contour Next One blood glucose meter lying display up, its test strip port toward the
robot; an open bottle of Contour Next test strips with one strip pulled up out of the pack, standing 15 mm proud of
the bottle's rim; a bottle of Contour Next control solution (normal), shaken and capped, lying tip toward the robot
in a rest made of two stacks of glass slides; and a clean glass microscope slide (the maker's "clean, nonabsorbent
surface").

Run the control test: uncap the control bottle (one hand holds it, the other pulls the cap off and stands it on
the counter) and lay it back in its rest; take the strip, push its grey end fully into the meter's port (the meter turns on), squeeze one
small drop of control solution onto the slide, touch the strip's sample tip to the drop and hold it there until the
strip has drawn the sample, then put the meter down, cap the control bottle and lay it back in its rest. The meter's
reading is compared with the range printed on the strip bottle.

Physical model (rigid bodies; disclosed in the README): the meter is a free 36 g body whose port is a slot with
rounded lips; a sprung contact roller inside the port presses on an inserted strip (the meter's contacts), which is
what holds a strip in the port. The strip is a thin rigid plate. The control bottle is a squeezable dropper (dropkit:
sprung wall pads; a drop forms only above the opening squeeze force with the tip pointing down, and falls straight
down). The drop on the slide is a lumped bead (its volume's spherical cap); the strip draws the sample when its tip
stays in the bead for the sip time with no other part of the strip in the liquid. The cap is a rigid cup that slides
over the bottle's conical nozzle. Every threshold below appears once, with the same value in task.json and README.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import gripkit  # noqa: E402
import yamkit as kit  # noqa: E402
import dropkit as dk  # noqa: E402  (after yamkit)
import guardkit as gk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, GRIPPER  # noqa: E402
from yam_arena.demo import grip_rotation  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = ("Run a control-solution test on the glucose meter with a new strip, then put the meter down, cap the "
               "control solution and lay it back in its rest.")
_DEBUG = int(os.environ.get("GLUCOSE_DEBUG", "0") or 0)  # print telemetry every N control steps (0: off)

# -- looks --------------------------------------------------------------------------------------------------------------
METER_LOOK = kit.Look((0.10, 0.10, 0.11), 0.45)  # black meter body
SCREEN = kit.Look((0.05, 0.07, 0.09), 0.15)  # its display
PORT_LIGHT = kit.Look((0.92, 0.93, 0.95), 0.3)  # the white port-light ring
BUTTON = kit.Look((0.75, 0.75, 0.76), 0.4)
STRIP_LOOK = kit.Look((0.93, 0.93, 0.90), 0.5)  # the strip's white face
CONTACTS = kit.Look((0.55, 0.56, 0.58), 0.35, metallic=0.3)  # its grey contact end
VIAL_LOOK = kit.Look((0.20, 0.32, 0.62), 0.45)  # the blue strip bottle
PACK_LOOK = kit.Look((0.88, 0.88, 0.86), 0.5)  # the other strips' ends, seen in the bottle
LDPE = kit.Look((0.95, 0.95, 0.93), 0.35, opacity=0.9)  # the control bottle
CAP_LOOK = kit.Look((0.97, 0.97, 0.97), 0.4)
SLIDE_STACK = kit.Look((0.80, 0.88, 0.89), 0.1)  # the rest's stacked glass slides
GLASS = kit.Look((0.86, 0.92, 0.93), 0.05)

# -- the meter ----------------------------------------------------------------------------------------------------------
# Ascensia Contour Next One: 97 x 28 x 14.9 mm, 36 g (user guide, Specifications). Its frame: origin in the middle of
# its underside, x along its length with the test strip port in the -x end, y across, z up (display up).
M_L, M_W, M_T = 0.097, 0.028, 0.0149
M_KG = 0.036
PORT_D = 0.009  # m: how far the strip's grey end goes in, from the end face to the contacts' back stop
SLOT_W, SLOT_H = 0.0076, 0.0010  # the port's slot: the strip's width + 0.6 mm, its thickness + 0.6 mm
LIP_R = 0.0020  # the slot's rounded lips (top and bottom), which guide a strip in that is up to ~1 mm off
SIDE_R = 0.0020  # ... and its rounded sides
X_FACE = -M_L / 2
X_STOP = X_FACE + PORT_D  # the contacts' back stop
Z_SLOT = M_T / 2  # the slot's middle height
FLOOR = Z_SLOT - SLOT_H / 2
ROLL_R = 0.0020  # the sprung contact (a roller across the slot) presses an inserted strip onto the slot floor
ROLL_X = X_STOP - ROLL_R - 0.0005
ROLL_GAP = 0.0002  # at rest the roller stands this far over the slot floor (a 0.4 mm strip lifts it 0.2 mm)
ROLL_TRAVEL = 0.0015
ROLL_K = 150.0  # N/m ...
ROLL_PRELOAD = 0.0020  # ... pulled toward a rest this far below its stop: 0.3 N on the strip (+0.03 N lifted); a
# strip connector's contacts press 0.1-0.3 N each (Roche, US 8,398,443); the roller lumps them at the upper end
METER_FRICTION = (0.30, 0.25)  # ABS plastic

# -- the strip ----------------------------------------------------------------------------------------------------------
# Contour Next test strip: a thin plastic plate, grey contact end (goes into the port) and the sample tip at the other
# end, which "sips" the sample by capillary action (user guide). Not published: 33 x 7 x 0.4 mm (the setup checks it
# with a caliper). Its frame: origin in its middle, x from the sample tip (-x) to the grey end (+x), y across, z through it.
S_L, S_W, S_T = 0.033, 0.0070, 0.0004
S_KG = 0.0002  # 0.2 g
GREY_L = 0.009  # the grey contact end's length (drawn)
SIP_L = 0.003  # m: only the last 3 mm of the strip's tip may touch the liquid

# -- the strip bottle ---------------------------------------------------------------------------------------------------
# Contour Next 50-count strip bottle: 32 mm across and 52 mm tall, flip-top lid open. The strips
# stand on end inside; the one to be used has been pulled up so it stands PROTRUDE above the rim (the others are
# drawn as the pack's tops just under the rim).
V_R, V_H, V_WALL = 0.016, 0.052, 0.0015
PROTRUDE = 0.015
V_POS = (0.200, -0.200)
STRIP_START = (V_POS[0], V_POS[1], TOP + V_H + PROTRUDE - S_L / 2)
SLOT_CLEAR = 0.00025  # each side of the standing strip, in the pack

# -- the control solution, its rest and cap -----------------------------------------------------------------------------
# Contour Next control solution (normal): a small LDPE dropper bottle, 19 mm across and 53 mm long with its
# nozzle, 9 g full. Its frame: origin mid-body, +x toward the tip; the squeeze pads face +-y and +-z.
B_R = 0.0095
B_BODY = (-0.026, 0.012)
B_SHOULDER = 0.016  # the shoulder's top: where the cap's rim seats
B_TIP = 0.027
TIP_R = 0.0015
NOZ_R = 0.0036  # the nozzle at the shoulder
B_KG = 0.009
PAD_KG = 0.0015  # each wall pad (the bottle core gets the rest)
GRIP_X = -0.017  # the demo holds the body here, near its base (room for the cap hand)
WALL_K = 3000.0  # N/m per wall (3 N per mm; 12 N at the pads' stop)
PAD_T, PAD_TRAVEL = 0.003, 0.004
PAD_X, PAD_W = (-0.024, 0.010), 0.012
OPEN_FORCE = 8.3  # N on the walls before liquid leaves the tip: the mean first-drop squeeze of 78 dropper bottles
# (8.3 +- 3.0 N, range 4.2-15.7 N; Wide Variation of Squeezing Force ... among Eyedropper Bottles, PMC6501172)
DROP_UL = 30.0  # uL: one drop (0.03 g)
BOTTLE_FRICTION = (0.25, 0.20)  # LDPE
REST_C = (0.260, -0.070)  # the rest; the bottle lies 2 cm toward the robot of its middle (nozzle out over its front)
# The rest: two stacks of four plain 75 x 25 x 1 mm glass slides taped to the counter, long sides along x, 20 mm
# apart: a groove 20 mm wide and 4 mm deep whose floor is the counter.
SLIDE_L, SLIDE_W, SLIDE_T, STACK_N = 0.075, 0.025, 0.001, 4
GROOVE_W, GROOVE_D = 0.020, STACK_N * SLIDE_T
REST_SIZE = (SLIDE_L, GROOVE_W + 2 * SLIDE_W, GROOVE_D)
REST_FLOOR = TOP
BOTTLE_START = (REST_C[0] - 0.020, REST_C[1], REST_FLOOR + B_R)
B_YAW = math.pi  # the bottle lies tip toward the robot (-x)
CAP_R, CAP_IN, CAP_H, CAP_TOP_T = 0.0065, 0.0045, 0.014, 0.002  # cap: outer and bore radius, height, top thickness
CAP_KG = 0.002
CAP_START = (0.200, 0.110)  # where the robot stands the cap, open side down, once it is off (tape mark)
CAP_ON_GAP = 0.0002  # m: at the start the cap's rim stands this far off the shoulder (seated; no overlap at spawn)

# -- the slide ----------------------------------------------------------------------------------------------------------
SLIDE_C = (0.240, 0.040)  # a plain 75 x 25 x 1 mm glass microscope slide, long side along x
SLIDE_SIZE = (0.075, 0.025, 0.001)
SLIDE_TOP = TOP + SLIDE_SIZE[2]
# A drop of control solution on clean glass: a spherical cap with the drop's volume and a 60 degree contact angle
# (DROP_UL 30 uL -> 3.1 mm footprint radius, 1.8 mm high).
BEAD_A = (DROP_UL * 1e-9 / (math.pi * math.tan(math.radians(30)) / 6 * (3 + math.tan(math.radians(30)) ** 2))) ** (1 / 3)
BEAD_H = BEAD_A * math.tan(math.radians(30))
_BEAD_RS = (BEAD_A ** 2 + BEAD_H ** 2) / (2 * BEAD_H)  # its sphere's radius
dk.BEAD_R = BEAD_A  # drawn as a flattened bead of this footprint

# -- the meter's start and the demo's key poses ---------------------------------------------------------------------------
METER_START = (0.340, 0.160)


def _box(lo, hi, look) -> kit.Mesh:
    return kit.box(tuple((a + b) / 2 for a, b in zip(lo, hi)), tuple(b - a for a, b in zip(lo, hi)), look)


def _cbox(lo, hi) -> kit.Collider:
    return kit.Collider(tuple((a + b) / 2 for a, b in zip(lo, hi)), size=tuple(b - a for a, b in zip(lo, hi)))


# The meter's body: the rear (behind the back stop) solid; in front of it, the slot with boxes above, below and beside
# it, rounded lips at the mouth, and a cavity over the slot for the contact roller.
_W2 = M_W / 2
_CAV = (ROLL_X - ROLL_R - 0.0003, X_STOP)  # the roller's cavity along x
_M_PARTS = (
    ((X_STOP, -_W2, 0.0), (M_L / 2, _W2, M_T)),  # behind the back stop
    ((X_FACE + LIP_R, -_W2, 0.0), (X_STOP, _W2, FLOOR)),  # under the slot (behind the lower lip)
    ((X_FACE, -_W2, 0.0), (X_FACE + LIP_R, _W2, FLOOR - LIP_R)),  # under the lower lip
    ((X_FACE + LIP_R, -_W2, FLOOR + SLOT_H), (_CAV[0], _W2, M_T)),  # over the slot, in front of the roller
    ((_CAV[0], -_W2, FLOOR + ROLL_GAP + 2 * ROLL_R + ROLL_TRAVEL + 0.0003), (X_STOP, _W2, M_T)),  # over the roller's cavity
    ((X_FACE, -_W2, FLOOR + SLOT_H + LIP_R), (X_FACE + LIP_R, _W2, M_T)),  # over the upper lip
    ((X_FACE + SIDE_R, SLOT_W / 2, FLOOR), (X_STOP, _W2, FLOOR + SLOT_H)),  # beside the slot (+y)
    ((X_FACE + SIDE_R, -_W2, FLOOR), (X_STOP, -SLOT_W / 2, FLOOR + SLOT_H)),  # beside the slot (-y)
)
_M_LIPS = (
    kit.Collider((X_FACE + LIP_R, 0.0, FLOOR - LIP_R), radius=LIP_R, height=M_W, axis="Y"),  # lower lip
    kit.Collider((X_FACE + LIP_R, 0.0, FLOOR + SLOT_H + LIP_R), radius=LIP_R, height=M_W, axis="Y"),  # upper lip
    kit.Collider((X_FACE + SIDE_R, SLOT_W / 2 + SIDE_R, Z_SLOT), radius=SIDE_R, height=SLOT_H + 2 * LIP_R),
    kit.Collider((X_FACE + SIDE_R, -SLOT_W / 2 - SIDE_R, Z_SLOT), radius=SIDE_R, height=SLOT_H + 2 * LIP_R),
)
_M_MESHES = (
    _box((X_FACE + 0.0015, -_W2, 0.0), (M_L / 2, _W2, M_T), METER_LOOK),  # body (the port face drawn separately)
    _box((X_FACE, -_W2, 0.0), (X_FACE + 0.0015, _W2, FLOOR - 0.0010), METER_LOOK),
    _box((X_FACE, -_W2, FLOOR + SLOT_H + 0.0010), (X_FACE + 0.0015, _W2, M_T), METER_LOOK),
    _box((X_FACE - 0.0002, -SLOT_W / 2 - 0.0012, FLOOR - 0.0010), (X_FACE + 0.0002, SLOT_W / 2 + 0.0012,
                                                                        FLOOR + SLOT_H + 0.0010), PORT_LIGHT),
    _box((X_FACE + 0.020, -0.010, M_T), (X_FACE + 0.055, 0.010, M_T + 0.0004), SCREEN),
    kit.cylinder((X_FACE + 0.072, 0.0, M_T + 0.0005), 0.006, 0.001, BUTTON),
)
ROLLER = kit.Link(
    "contact", "prismatic", "Z", anchor=(ROLL_X, 0.0, FLOOR + ROLL_GAP + ROLL_R), lower=0.0, upper=ROLL_TRAVEL,
    start=0.0, mass=0.002, spring=ROLL_K, rest=-ROLL_PRELOAD, damping=0.2,
    meshes=(kit.cylinder((0.0, 0.0, 0.0), ROLL_R, SLOT_W - 0.0006, CONTACTS, axis="y"),),
    colliders=(kit.Collider((0.0, 0.0, 0.0), radius=ROLL_R, height=SLOT_W - 0.0006, axis="Y"),))
METER = kit.Mechanism("meter", pos=(METER_START[0], METER_START[1], TOP), mass=M_KG, size=(M_L, M_W, M_T),
                      meshes=_M_MESHES, colliders=tuple(_cbox(lo, hi) for lo, hi in _M_PARTS) + _M_LIPS,
                      links=(ROLLER,), friction=METER_FRICTION)

# The strip stands in the bottle grey end down, its faces toward +-y: strip x -> world -z, strip z -> world +y.
_S_ROT0 = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, 1.0], [-1.0, 0.0, 0.0]])


def _quat(R) -> tuple:
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1 + R[i, i] - R[j, j] - R[k, k])) * 2
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i], q[j], q[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    return tuple(q)


STRIP = kit.Prop(
    "strip", pos=STRIP_START, mass=S_KG, size=(S_W, S_T, S_L), tilt_xyzw=tuple(float(v) for v in _quat(_S_ROT0)),
    meshes=(_box((-S_L / 2, -S_W / 2, -S_T / 2), (S_L / 2 - GREY_L, S_W / 2, S_T / 2), STRIP_LOOK),
            _box((S_L / 2 - GREY_L, -S_W / 2, -S_T / 2), (S_L / 2, S_W / 2, S_T / 2), CONTACTS)),
    colliders=(_cbox((-S_L / 2, -S_W / 2, -S_T / 2), (S_L / 2, S_W / 2, S_T / 2)),))

# The strip bottle: its wall as 12 boxes round its axis, a floor, the pack of strips as two blocks either side of the
# standing strip (to 3 mm under the rim), and the strip's seat in the pack.
_VN = 12
_STRIP_FOOT = V_H + PROTRUDE - S_L  # the standing strip's foot over the counter
_V_WALL = tuple(kit.Collider((V_R * 0.98 * math.cos(2 * math.pi * k / _VN), V_R * 0.98 * math.sin(2 * math.pi * k / _VN),
                              V_H / 2), size=(V_WALL, 2 * V_R * math.tan(math.pi / _VN) + 0.0005, V_H))
                for k in range(_VN))
_PACK_IN = V_R - V_WALL - 0.0015  # the pack blocks' half-extent, inside the wall
_V_PACK = (
    _cbox((-_PACK_IN * 0.7, S_T / 2 + SLOT_CLEAR, 0.002), (_PACK_IN * 0.7, _PACK_IN * 0.7, V_H - 0.003)),
    _cbox((-_PACK_IN * 0.7, -_PACK_IN * 0.7, 0.002), (_PACK_IN * 0.7, -S_T / 2 - SLOT_CLEAR, V_H - 0.003)),
    _cbox((-_PACK_IN * 0.7, -S_T / 2 - SLOT_CLEAR, 0.002), (_PACK_IN * 0.7, S_T / 2 + SLOT_CLEAR, _STRIP_FOOT)),
    _cbox((-V_R * 0.9, -V_R * 0.9, 0.0), (V_R * 0.9, V_R * 0.9, 0.002)))
VIAL = kit.Prop(
    "strip_bottle", pos=(V_POS[0], V_POS[1], TOP), mass=0.030, size=(2 * V_R, 2 * V_R, V_H),
    meshes=(kit.lathe([(0.0, 0.0), (V_R, 0.0), (V_R, V_H), (V_R - V_WALL, V_H), (V_R - V_WALL, 0.002),
                       (0.0, 0.002)], VIAL_LOOK, n=36),
            _box((-_PACK_IN * 0.7, -_PACK_IN * 0.7, V_H - 0.004), (_PACK_IN * 0.7, _PACK_IN * 0.7, V_H - 0.003),
                 PACK_LOOK),
            _box((-V_R - 0.024, -V_R * 0.8, V_H - 0.001), (-V_R - 0.002, V_R * 0.8, V_H + 0.002), VIAL_LOOK)),  # lid
    colliders=_V_WALL + _V_PACK)

# The control bottle (dropkit): a rigid core and four sprung wall pads, a nozzle cone (stacked cylinders) and shoulder.
_pad_c = ((PAD_X[0] + PAD_X[1]) / 2, 0.0, 0.0)
_PAD_SIZE = {"Y": (PAD_X[1] - PAD_X[0], PAD_T, PAD_W), "Z": (PAD_X[1] - PAD_X[0], PAD_W, PAD_T)}
_CORE = B_R - PAD_T - PAD_TRAVEL
PADS = (("l", "Y", 1), ("r", "Y", -1), ("t", "Z", 1), ("b", "Z", -1))
BOTTLE_PROFILE = [(0.0, B_BODY[0]), (B_R, B_BODY[0]), (B_R, B_BODY[1]), (0.0060, B_SHOULDER - 0.001),
                  (0.0060, B_SHOULDER), (NOZ_R, B_SHOULDER), (TIP_R + 0.0003, B_TIP - 0.0003), (0.0, B_TIP)]
_NOZ_STEPS = 4  # the nozzle's cone as stacked cylinders


class _PadSpring:
    """The wall's spring; the pad's travel opens only once the spring acts (the first control step), so a lying
    bottle's lower pad does not sag to its stop while the scene settles (eye-drop task lesson)."""

    sets_limits = True

    def __init__(self, k, lo, hi):
        self.k, self.lo, self.hi = k, lo, hi

    def drive(self, key, q, qd, J, mem):
        mem["limits"] = (self.lo, self.hi)
        return 0.0, self.k, 0.0, 2.0


def _pad(side, axis, sg):
    anchor = (0.0, sg * (B_R - PAD_T / 2), 0.0) if axis == "Y" else (0.0, 0.0, sg * (B_R - PAD_T / 2))
    lo, hi = (-PAD_TRAVEL, 0.0) if sg > 0 else (0.0, PAD_TRAVEL)
    return kit.Link(f"pad_{side}", "prismatic", axis, anchor=anchor, lower=-1e-4, upper=1e-4, start=0.0, mass=PAD_KG,
                    meshes=(kit.box(_pad_c, _PAD_SIZE[axis], LDPE),),
                    colliders=(kit.Collider(_pad_c, size=_PAD_SIZE[axis]),),
                    law=_PadSpring(WALL_K, lo, hi))


def _nozzle_colliders() -> tuple:
    out = [kit.Collider(((B_BODY[1] + B_SHOULDER) / 2, 0.0, 0.0), radius=0.0060, height=B_SHOULDER - B_BODY[1],
                        axis="X")]
    seg = (B_TIP - B_SHOULDER) / _NOZ_STEPS
    for k in range(_NOZ_STEPS):
        x0 = B_SHOULDER + k * seg
        r = NOZ_R + (TIP_R - NOZ_R) * (k + 0.5) / _NOZ_STEPS  # each step at the cone's mean radius over it
        out.append(kit.Collider((x0 + seg / 2, 0.0, 0.0), radius=r, height=seg, axis="X"))
    return tuple(out)


BOTTLE = kit.Mechanism(
    "bottle", pos=BOTTLE_START, yaw=B_YAW, size=(B_TIP - B_BODY[0], 2 * B_R, 2 * B_R), mass=B_KG - 4 * PAD_KG,
    meshes=(kit.lathe(BOTTLE_PROFILE, LDPE, axis="x", n=28),),
    colliders=(kit.Collider((sum(B_BODY) / 2, 0.0, 0.0), size=(B_BODY[1] - B_BODY[0], 2 * _CORE, 2 * _CORE)),
               *_nozzle_colliders()),
    links=tuple(_pad(*p) for p in PADS), friction=BOTTLE_FRICTION)
BOTTLE_SPEC = dk.Bottle("bottle", pads=((("pad_l", -1), ("pad_r", 1)), (("pad_t", -1), ("pad_b", 1))),
                        tip=(B_TIP - TIP_R, 0.0, 0.0), axis=(1, 0, 0), wall_k=WALL_K, open_force=OPEN_FORCE,
                        flow=50.0, drop_ul=DROP_UL, tip_down_deg=30.0, capacity_ul=2500.0)

_STACKS = tuple(((-SLIDE_L / 2, sg * GROOVE_W / 2, 0.0), (SLIDE_L / 2, sg * (GROOVE_W / 2 + SLIDE_W), GROOVE_D))
                for sg in (-1, 1))
_STACKS = tuple((tuple(min(a, b) for a, b in zip(lo, hi)), tuple(max(a, b) for a, b in zip(lo, hi))) for lo, hi in _STACKS)
REST = kit.Prop(
    "bottle_rest", pos=(REST_C[0], REST_C[1], TOP), size=REST_SIZE,
    meshes=tuple(_box(lo, hi, SLIDE_STACK) for lo, hi in _STACKS),
    colliders=tuple(_cbox(lo, hi) for lo, hi in _STACKS))

# The cap: a cup. Its frame: origin in the middle of its open rim, +z through its top. It starts on the bottle (its
# axis along the bottle's, rim at the shoulder); the robot takes it off and stands it open side down at CAP_START.
_CN = 12
_CAP_RING = tuple(kit.Collider((((CAP_R + CAP_IN) / 2) * math.cos(2 * math.pi * k / _CN),
                                ((CAP_R + CAP_IN) / 2) * math.sin(2 * math.pi * k / _CN), (CAP_H - CAP_TOP_T) / 2),
                               size=(CAP_R - CAP_IN, 2 * CAP_R * math.tan(math.pi / _CN), CAP_H - CAP_TOP_T))
                  for k in range(_CN))
CAP = kit.Prop(
    "cap", pos=(BOTTLE_START[0] - B_SHOULDER - CAP_ON_GAP, BOTTLE_START[1], BOTTLE_START[2]), mass=CAP_KG,
    size=(2 * CAP_R, 2 * CAP_R, CAP_H), tilt_xyzw=(0.0, -math.sqrt(0.5), 0.0, math.sqrt(0.5)),  # +z -> -x (on the tip)
    meshes=(kit.lathe([(CAP_IN, 0.0), (CAP_R, 0.0), (CAP_R, CAP_H), (0.0, CAP_H), (0.0, CAP_H - CAP_TOP_T),
                       (CAP_IN, CAP_H - CAP_TOP_T), (CAP_IN, 0.0)], CAP_LOOK, n=28),),
    colliders=_CAP_RING + (kit.Collider((0.0, 0.0, CAP_H - CAP_TOP_T / 2), radius=CAP_R, height=CAP_TOP_T),))

SLIDE = kit.Prop("slide", pos=(SLIDE_C[0], SLIDE_C[1], TOP), size=SLIDE_SIZE,
                 meshes=(_box((-SLIDE_SIZE[0] / 2, -SLIDE_SIZE[1] / 2, 0.0), (SLIDE_SIZE[0] / 2, SLIDE_SIZE[1] / 2,
                                                                             SLIDE_SIZE[2]), GLASS),),
                 colliders=(_cbox((-SLIDE_SIZE[0] / 2, -SLIDE_SIZE[1] / 2, 0.0), (SLIDE_SIZE[0] / 2,
                                                                                  SLIDE_SIZE[1] / 2, SLIDE_SIZE[2])),))

OBJECTS = [METER, STRIP, VIAL, BOTTLE, REST, CAP, SLIDE]
gripkit.use_rubber_pads()  # rubber pad faces, as on the real YAM (measured grip envelope)

# -- the scored thresholds -----------------------------------------------------------------------------------------------
SEAT_TOL = 0.0010  # m: the strip's grey end within 1 mm of the contacts' back stop: fully in, the meter on
SEAT_ALIGN_DEG = 6.0  # ... and the strip lined up with the port within 6 degrees
SIP_S = 1.0  # s: the strip's tip held in the drop this long (contiguous) draws the sample (the meter beeps)
COUNTDOWN_S = 5.0  # s: the meter counts down 5 s with the strip in, then shows the result (user guide)
DROP_FRESH_S = 60.0  # s: the drop is sampled within 60 s of landing ("immediately", user guide)
TOUCH_GAP = 0.0010  # m: the strip within 1 mm of the bottle's tip counts as touching it (control applied from the bottle)
TIP_IN_SLACK = 0.0005  # m: the tip point counts as in the bead within 0.5 mm of its surface
CAP_OFF_TOL = 0.0015  # m: capped: the cap's rim centre within 1.5 mm of the shoulder's centre, ...
CAP_ALIGN_DEG = 10.0  # ... its axis within 10 degrees of the bottle's
DOWN_TOL = 0.003  # m: the meter resting on the counter (underside within 3 mm of it, tilted 10 degrees or less)
DOWN_TILT_DEG = 10.0
BOTTLE_DOWN_TOL = 0.004  # m: the bottle lying in its rest (its axis within 4 mm of the groove's floor + radius, ...
BOTTLE_REST_OFF = 0.030  # ... and within 3 cm of the rest's middle)
DROP_SPEED = 0.8  # m/s: the meter or the control bottle falling faster than this was dropped
RELEASE_MARGIN = 0.010  # m: every tool point and pad tip this far from meter, strip, bottle and cap
STILL_STEP = 0.0005  # m per control step
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
SIP_STEPS = round(SIP_S / DT)
COUNT_STEPS = round(COUNTDOWN_S / DT)
FRESH_STEPS = round(DROP_FRESH_S / DT)
dk.setup(BOTTLE_SPEC, lambda objects, tip_w: land(objects, tip_w))


# -- geometry ------------------------------------------------------------------------------------------------------------
def pose(objects, name) -> tuple:
    s = objects[name]
    return np.asarray(s.pos, float), gk._rot(np.asarray(s.quat_xyzw, float))


def to_local(objects, name, p) -> np.ndarray:
    o, R = pose(objects, name)
    return R.T @ (np.asarray(p, float) - o)


def to_world(objects, name, p) -> np.ndarray:
    o, R = pose(objects, name)
    return o + R @ np.asarray(p, float)


def strip_point(objects, x, y=0.0, z=0.0) -> np.ndarray:
    return to_world(objects, "strip", (x, y, z))


def grey_end(objects) -> np.ndarray:
    return strip_point(objects, S_L / 2)


def sample_tip(objects) -> np.ndarray:
    return strip_point(objects, -S_L / 2)


def seat_depth(objects) -> float:
    """How far (m) the strip's grey end is short of the contacts' back stop (0 = fully in; negative = past it)."""
    return float(X_STOP - to_local(objects, "meter", grey_end(objects))[0])


def seated(objects) -> bool:
    """The strip fully in the port: its grey end within 1 mm of the back stop, inside the slot, lined up with it."""
    g = to_local(objects, "meter", grey_end(objects))
    _o, Rm = pose(objects, "meter")
    _o, Rs = pose(objects, "strip")
    ang = math.degrees(math.acos(float(np.clip(Rm[:, 0] @ Rs[:, 0], -1.0, 1.0))))
    return (abs(seat_depth(objects)) <= SEAT_TOL and abs(g[1]) <= SLOT_W / 2 and abs(g[2] - Z_SLOT) <= SLOT_H
            and ang <= SEAT_ALIGN_DEG)


def bead_height(r: float) -> float:
    """The drop's surface height (m) over the glass at distance r from its centre (0 outside its footprint)."""
    if r >= BEAD_A:
        return 0.0
    return math.sqrt(_BEAD_RS ** 2 - r * r) - (_BEAD_RS - BEAD_H)


def in_bead(p, at, slack: float = 0.0) -> bool:
    """Point p (world) in the drop that landed at `at` (on the glass), within `slack` of its surface."""
    r = math.hypot(p[0] - at[0], p[1] - at[1])
    return r <= BEAD_A + slack and p[2] - at[2] <= bead_height(min(r, BEAD_A)) + slack


def _strip_body_points(objects) -> list:
    """Points on the strip's four long edges, from SIP_L behind its tip to its grey end."""
    out = []
    for x in np.linspace(-S_L / 2 + SIP_L, S_L / 2, 16):
        for y in (-S_W / 2, 0.0, S_W / 2):
            for z in (-S_T / 2, S_T / 2):
                out.append(strip_point(objects, x, y, z))
    return out


def _strip_tip_points(objects) -> list:
    return [strip_point(objects, -S_L / 2, y, z) for y in (-S_W / 4, 0.0, S_W / 4) for z in (-S_T / 2, 0.0, S_T / 2)]


def slide_drops() -> list:
    """Landing points of the drops that landed on the slide."""
    return [d for d in dk.drops() if d.zone == "slide"]


def strip_box_dist(objects, p) -> float:
    q = np.abs(to_local(objects, "strip", p)) - np.array([S_L / 2, S_W / 2, S_T / 2])
    return float(np.linalg.norm(np.maximum(q, 0.0)) + min(0.0, float(q.max())))


def land(objects, tip_w):
    """Where a drop leaving the tip lands, falling straight down: on the strip or the meter (control put on the strip
    straight from the bottle), on the slide, or on the counter."""
    tip_w = np.asarray(tip_w, float)
    for z in np.arange(tip_w[2], TOP - 0.001, -0.0005):
        p = np.array([tip_w[0], tip_w[1], z])
        if "strip" in objects and strip_box_dist(objects, p) <= 0.0005:
            return "strip", p, {}
        if "meter" in objects:
            m = to_local(objects, "meter", p)
            if abs(m[0]) <= M_L / 2 and abs(m[1]) <= M_W / 2 and -0.0005 <= m[2] <= M_T:
                return "meter", p, {}
        if (abs(p[0] - SLIDE_C[0]) <= SLIDE_SIZE[0] / 2 and abs(p[1] - SLIDE_C[1]) <= SLIDE_SIZE[1] / 2
                and p[2] <= SLIDE_TOP + 0.0005):
            return "slide", np.array([p[0], p[1], SLIDE_TOP]), {}
    return "counter", np.array([tip_w[0], tip_w[1], TOP]), {}


def capped(objects) -> bool:
    """The cap on the bottle: its rim's centre within 1.5 mm of the shoulder's centre and its axis within 10 degrees of
    the bottle's (the rim seated on the shoulder; the nozzle inside the cap)."""
    c = to_local(objects, "bottle", objects["cap"].pos)
    _o, Rb = pose(objects, "bottle")
    _o, Rc = pose(objects, "cap")
    ang = math.degrees(math.acos(float(np.clip(Rb[:, 0] @ Rc[:, 2], -1.0, 1.0))))
    return float(np.linalg.norm(c - np.array([B_SHOULDER, 0.0, 0.0]))) <= CAP_OFF_TOL and ang <= CAP_ALIGN_DEG


def bottle_in_rest(objects) -> bool:
    p, R = pose(objects, "bottle")
    return (abs(float(p[2]) - (REST_FLOOR + B_R)) <= BOTTLE_DOWN_TOL
            and math.hypot(p[0] - BOTTLE_START[0], p[1] - BOTTLE_START[1]) <= BOTTLE_REST_OFF and abs(float(R[2, 0])) <= 0.17)


def meter_down(objects) -> bool:
    p, R = pose(objects, "meter")
    tilt = math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))
    return abs(float(p[2]) - TOP) <= DOWN_TOL and tilt <= DOWN_TILT_DEG


METER_PARTS = (gk.Block((0.0, 0.0, M_T / 2), (M_L, M_W, M_T)),)
STRIP_PARTS = (gk.Block((0.0, 0.0, 0.0), (S_L, S_W, S_T)),)
BOTTLE_PARTS = (gk.Seg((B_BODY[0], 0.0, 0.0), (B_TIP, 0.0, 0.0), B_R),)
CAP_PARTS = (gk.Ring(0.0, CAP_R, 0.0, CAP_H),)


def released(objects) -> bool:
    return all(gk.released_clear(objects, n, parts, RELEASE_MARGIN) for n, parts in
               (("meter", METER_PARTS), ("strip", STRIP_PARTS), ("bottle", BOTTLE_PARTS), ("cap", CAP_PARTS)))


# -- one trial's record --------------------------------------------------------------------------------------------------
class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.n = 0
        self.last = None
        self.ruined = ""  # why the trial scores 0 (meter or bottle dropped; strip touched the bottle; control on strip)
        self.smeared = False  # the strip's body (not just its tip) went into the liquid: an invalid result
        self.sip = 0
        self.filled_at = None  # the step the strip drew the sample (the meter beeped)
        self.count = 0
        self.result = False  # the meter showed a result (strip seated through the 5 s countdown after the beep)
        self.aborted = False  # the strip came out during the countdown (no result; that strip is used)
        self.good = 0

    def step(self, objects) -> None:
        self.n += 1
        pts = {k: np.asarray(objects[k].pos, float) for k in ("meter", "bottle", "strip", "cap")}
        if self.last is not None:
            for k in ("meter", "bottle"):
                if (pts[k][2] - self.last[k][2]) / DT < -DROP_SPEED:
                    self.ruined = self.ruined or f"{k} dropped"
        if strip_box_dist(objects, dk.tip(objects, BOTTLE_SPEC)) <= TOUCH_GAP:
            self.ruined = self.ruined or "strip touched the bottle's tip"
        if any(d.zone in ("strip", "meter") for d in dk.drops()):
            self.ruined = self.ruined or "control solution put on the strip or meter from the bottle"
        is_seated = seated(objects)
        fresh = [d.at for d in slide_drops() if self.n - d.step <= FRESH_STEPS]
        body = _strip_body_points(objects)
        for d in slide_drops():
            if any(in_bead(p, d.at) for p in body):
                self.smeared = True
        tip_in = any(in_bead(p, at, TIP_IN_SLACK) for at in fresh for p in _strip_tip_points(objects))
        if self.filled_at is None:
            self.sip = self.sip + 1 if (is_seated and tip_in and not self.smeared) else 0
            if self.sip >= SIP_STEPS:
                self.filled_at = self.n
        elif not self.result and not self.aborted:
            if is_seated:
                self.count += 1
                self.result = self.count >= COUNT_STEPS
            else:
                self.aborted = True
        still = self.last is not None and all(float(np.abs(pts[k] - self.last[k]).max()) <= STILL_STEP for k in pts)
        self.last = pts
        done = (self.result and not self.smeared and capped(objects) and bottle_in_rest(objects)
                and meter_down(objects) and is_seated and released(objects) and still)
        self.good = self.good + 1 if done else 0
        if _DEBUG and "left_gripper" in objects and (self.n % _DEBUG == 0 or dk.squeeze_force(objects, BOTTLE_SPEC) > 2.0):
            g = to_local(objects, "meter", grey_end(objects))
            print(f"glucose {self.n} seat {seat_depth(objects) * 1000:+.1f}mm grey_m {np.round(g * 1000, 1)} "
                  f"seated {is_seated} roller {kit.joint(objects, 'meter/contact').pos * 1000:.2f}mm "
                  f"strip {np.round(pts['strip'], 4)} meter {np.round(pts['meter'], 4)} tip {np.round(sample_tip(objects), 4)} "
                  f"drops {[(d.step, d.zone, None if d.at is None else np.round(d.at, 4).tolist()) for d in dk.drops()]} "
                  f"sip {self.sip} filled {self.filled_at} count {self.count} result {self.result} smear {self.smeared} "
                  f"capped {capped(objects)} cap {np.round(to_local(objects, 'bottle', pts['cap']) * 1000, 1)} "
                  f"rest {bottle_in_rest(objects)} down {meter_down(objects)} rel {released(objects)} "
                  f"ruined '{self.ruined}' sq {dk.squeeze_force(objects, BOTTLE_SPEC):.1f}N "
                  f"capw {np.round(pts['cap'], 4)} capax {np.round(pose(objects, 'cap')[1][:, 2], 2)} "
                  f"bottle {np.round(pts['bottle'], 4)} tipdir {np.round(dk.tip_dir(objects, BOTTLE_SPEC), 2)} "
                  f"hang {dk.hanging():.1f} L {np.round(objects['left_gripper'].pos, 4)} "
                  f"R {np.round(objects['right_gripper'].pos, 4)}", flush=True)

    def tier(self, objects) -> float:
        if self.ruined:
            return 0.0
        valid = self.result and not self.smeared
        if valid and self.good >= HOLD_STEPS:
            return 1.0
        if valid:
            return 0.75
        is_seated = seated(objects)
        if is_seated and slide_drops() and not self.smeared and not self.aborted:
            return 0.5
        if is_seated:
            return 0.25
        return 0.0


_L = Ledger()


def _update(objects):
    """Advance the trial's record once per control step; success() and score() share it."""
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    dk.update(objects)
    _L.step(objects)


def success(objects):
    """The control test done and everything put away: a valid result (the strip seated, its tip alone drew the sample
    from a drop on the slide, and it stayed seated through the 5 s countdown), the meter resting on the counter with
    the strip in, the control bottle capped and lying in its rest, nothing dropped or contaminated, every hand clear
    and everything still: this step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """0 if the meter or the control bottle was dropped, the strip touched the bottle's tip, or control solution was
    put on the strip or meter straight from the bottle. Otherwise: 1.0 for a valid result with the meter down (strip
    in), the bottle capped in its rest, and every hand clear and everything still for 1 s; 0.75 for a valid result
    otherwise; 0.5 for the strip seated (meter on) with a drop on the slide, not yet sampled (or sampled and the
    strip still in but the countdown not finished); 0.25 for the strip seated. A smeared strip (its body in the
    liquid) or a strip pulled out during the countdown never gives a result. Doing nothing scores 0."""
    _update(objects)
    return _L.tier(objects)


# -- the simulator hook: dropkit and the record step once per control step, after yamkit's object_states ------------------
def _patch(scene) -> None:
    if getattr(scene, "_glucose", False):
        return
    scene._glucose = True
    inner = scene.object_states

    def object_states(env, names):
        out = inner(env, names)
        if "strip" in out and "meter" in out:
            _update(out)
        return out

    scene.object_states = object_states


class _Hook(kit._Hook):
    def exec_module(self, module):
        self._inner.exec_module(module)
        kit._patch(module)
        dk._patch(module)
        _patch(module)


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:  # replace yamkit's (and dropkit's) import hook with one that adds the record step after theirs
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook())


# -- scripted demo: open loop from the fixed layout ------------------------------------------------------------------------
# Left hand: takes the meter by its far end from above, lifts it and holds it level with its port toward the right
# hand; after the drop is on the slide it tips the meter port-end down 45 degrees and touches the strip's tip to the
# drop, holds it there, then puts the meter back down and lets go; takes the cap, turns it open side forward and pushes
# it onto the nozzle of the bottle the right hand holds level.
# Right hand: pinches the standing strip's upper end across its faces, pulls it out of the bottle, turns it level (gray
# end forward, faces up) and pushes it into the port; lets go; takes the control bottle from its rest, tips it tip-down
# toward the robot over the slide and squeezes one drop; holds it level, tip toward the robot, for the cap; lays it back
# in its rest; goes home.
_GRIP0 = np.asarray(grip_rotation(0.0), float)  # the arena's straight-down grip: columns closing, width, approach
F_HALF_OPEN = 0.047  # m: each finger's face from the tool point at full open
K_F = float(GRIPPER["stiffness"])


def finger(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * max(0.0, half) / F_HALF_OPEN


def frame(approach, close) -> np.ndarray:
    z = np.asarray(approach, float) / np.linalg.norm(approach)
    x = np.asarray(close, float) - (np.asarray(close, float) @ z) * z
    x /= np.linalg.norm(x)
    return np.column_stack([x, np.cross(z, x), z])


def decompose(R) -> tuple:
    """(yaw, roll, pitch) with kit.Tilted's rotation ry(pitch) @ rx(roll) @ grip_rotation(yaw) equal to R."""
    A = np.asarray(R, float) @ _GRIP0.T
    roll = math.asin(max(-1.0, min(1.0, -A[1, 2])))
    return math.atan2(A[1, 0], A[1, 1]), roll, math.atan2(A[0, 2], A[2, 2])


def tw(duration, pos, fing, label, R) -> kit.Tilted:
    yaw, roll, pitch = decompose(R)
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(fing), label, yaw=yaw, roll=roll,
                      pitch=pitch)


def ry(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


DOWN_Y = frame((0, 0, -1), (0, -1, 0))  # fingers straight down, closing along y (grip_rotation(0))
# -- strip hand --
STRIP_PINCH = 0.012  # m: the pads' tips this far under the standing strip's top (3 mm over the bottle's rim)
STRIP_TCP_OUT = 0.013 - STRIP_PINCH  # the tool point this far beyond the strip's tip end (pad tips 13 mm past it)
INSERT_R = frame((1, 0, 0), (0, 0, -1))  # fingers forward, closing up-down: the strip level, grey end forward
PUSH_PAST = 0.0040  # m: the insert is commanded this far past the back stop (probe p1: the loaded hand stalls ~2.5 mm short)
# -- meter hand --
M_GRIP_X = 0.030  # m: the meter held this far from its middle toward its far end
M_SQUEEZE = gripkit.squeeze_target(M_W * 1000, 4.0)  # across its 28 mm width, 4 mm squeeze (about 10 N a finger)
M_TCP = np.array([M_GRIP_X, 0.0, M_T / 2 + 0.013])  # the tool point in the meter's frame: pad tips at mid-height
M_INS = np.array([0.360, 0.020, TOP + 0.060])  # the meter's origin while the strip goes in
M_LIFT_SAG = 0.003  # m: the loaded hand hangs about this much low (the meter is light)
DIP_TILT = math.radians(-45.0)  # the meter tipped port-end down 45 degrees to touch the drop
DIP_Z = -0.0010  # m: the strip's tip centre aimed this far under the glass top (probe p2: it rides 1.6 mm over its aim)
DWELL_S = 2.0  # s held in the drop (the sip takes SIP_S)
# -- bottle hand --
B_GRIP_R = DOWN_Y  # taking the lying bottle from above, closing across it along y
FINGER_HOLD = B_R - 0.0005 * (K_F + WALL_K) / K_F  # each wall pushed 0.5 mm: 1.5 N, under the 8.3 N opening force
FINGER_SQUEEZE = B_R - PAD_TRAVEL - 0.006  # 6 mm past the walls' stop: 12 N (probe q1: 3 mm past gave 9.8 N)
FINGER_FIRM = B_R - 0.0015 * (K_F + WALL_K) / K_F  # each wall pushed 1.5 mm: 4.5 N, still under the opening force, while
# the cap is pushed on (probe p6: at 1 N the bottle slid 6 mm along the fingers and the cap stopped 1.8 mm short)
SQUEEZE_S = 0.3  # s at full squeeze (probe p2: 0.1 s grew 26.5 of 30 uL; the 8.3 N opening is reached later)
DROP_TILT = math.radians(45.0)  # the bottle tipped tip-down 45 degrees over the slide
TIP_OVER = 0.012  # m: its tip this far over the glass when squeezed
DROP_AT = (SLIDE_C[0] - 0.012, SLIDE_C[1])  # where the drop is aimed on the slide
# -- cap hand --
CAP_SQUEEZE = gripkit.squeeze_target(2 * CAP_R * 1000, 3.0)
CAP_TRIM = np.array([0.0, -0.0016, -0.0017])  # m: probe p4: the held cap sat 1.6 mm left of and 1.7 mm over the
# nozzle's axis (the cap turned in the pinch, the bottle in its hand), so the bore caught the nozzle's tip
CAP_PUSH = 0.0050  # m: the cap is pushed this far past seated (the shoulder stops it; probe p5: 1.5 mm left it 2.5 mm short, the held bottle giving 2.7 mm)
CAP_R_HAND = ry(-math.pi / 2) @ DOWN_Y  # the cap turned open side forward, fingers forward, to push it on
CAP_HOLD = np.array([0.300, 0.000, TOP + 0.100])  # the right hand's tool point holding the bottle level for the cap
CAP_WAIT = np.array([0.220, -0.170, TOP + 0.100])  # ... and while the meter is dipped (clear of the left arm)
B_ROT0 = np.array([[-1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, 1.0]])  # the bottle's start rotation (yaw 180)
WIDE = finger(0.020)


def strip_rel(R_hand) -> tuple:
    """The strip's rotation and its middle's offset from the tool point, for a hand at R_hand (the pinch keeps the
    strip's x along the approach and its faces across the closing axis)."""
    Rs = R_hand @ (DOWN_Y.T @ _S_ROT0)
    return Rs, Rs @ np.array([S_L / 2 + STRIP_TCP_OUT, 0.0, 0.0])


def strip_grasp_tcp() -> np.ndarray:
    top = TOP + V_H + PROTRUDE
    return np.array([V_POS[0], V_POS[1], top + STRIP_TCP_OUT])


def insert_tcp(meter_origin, past: float = 0.0, back: float = 0.0) -> np.ndarray:
    """The tool point that puts the strip's grey end `past` beyond the back stop (`back` m short of it instead)."""
    stop = np.asarray(meter_origin, float) + np.array([X_STOP, 0.0, Z_SLOT])
    Rs, off = strip_rel(INSERT_R)
    return stop + np.array([past - back, 0.0, 0.0]) - (off + Rs @ np.array([S_L / 2, 0.0, 0.0]))


def meter_tcp(origin, R_m=np.eye(3)) -> np.ndarray:
    return np.asarray(origin, float) + R_m @ M_TCP


def dip_meter_origin(at) -> np.ndarray:
    """The meter's origin with the strip seated, tipped DIP_TILT, its sample tip DIP_Z over the glass at `at`."""
    R = ry(DIP_TILT)
    tip_m = np.array([X_STOP - S_L, 0.0, Z_SLOT])
    return np.array([at[0], at[1], SLIDE_TOP + DIP_Z]) - R @ tip_m


B_IN_HAND = 0.008  # m: the bottle's axis rides this far beyond the tool point along the fingers (probe p3: taken from
# the rest's groove, the pads stop above its axis; the drop landed 6.6 mm long and the held bottle sat 8 mm low)


def bottle_tcp(center, R_b, R_hand=None) -> np.ndarray:
    """The tool point for a bottle centred at `center`, turned R_b, held by a hand turned R_hand (default B_GRIP_R)
    at GRIP_X along its body."""
    R_hand = B_GRIP_R if R_hand is None else R_hand
    return np.asarray(center, float) + R_b @ np.array([GRIP_X, 0.0, 0.0]) - R_hand[:, 2] * B_IN_HAND


def drop_tcp() -> np.ndarray:
    R_b = ry(-DROP_TILT) @ B_ROT0
    tip_b = np.array([B_TIP, 0.0, 0.0])
    center = np.array([DROP_AT[0], DROP_AT[1], SLIDE_TOP + TIP_OVER]) - R_b @ tip_b
    return bottle_tcp(center, R_b, ry(-DROP_TILT) @ B_GRIP_R)


CAP_PRESS = 0.003  # m: the closed fingertips pressed this far into the seated cap's top (probe p7: pushed on in the
# pinch, the light cap tilted and stopped 1.8 mm short; a press on its top squares it, as a thumb does)


def press_tcp(past: float = 0.0, back: float = 0.0) -> np.ndarray:
    """The tool point (fingers closed, turned CAP_R_HAND) whose pad tips touch the top of a fully seated cap on the
    bottle held at CAP_HOLD, `past` m further in (`back` m short instead)."""
    center = CAP_HOLD - B_ROT0 @ np.array([GRIP_X, 0.0, 0.0]) + B_GRIP_R[:, 2] * B_IN_HAND
    axis = B_ROT0 @ np.array([1.0, 0.0, 0.0])  # toward the bottle's tip
    top = center + axis * (B_SHOULDER + CAP_H)
    tips = top + axis * (back - past)
    return tips - CAP_R_HAND[:, 2] * 0.013


def cap_tcp(past: float = 0.0, back: float = 0.0) -> np.ndarray:
    """The tool point holding the cap (turned by CAP_R_HAND) with its rim `past` beyond the shoulder of the bottle held
    level at CAP_HOLD (`back` m short of it instead)."""
    center = CAP_HOLD - B_ROT0 @ np.array([GRIP_X, 0.0, 0.0]) + B_GRIP_R[:, 2] * B_IN_HAND
    shoulder = center + B_ROT0 @ np.array([B_SHOULDER, 0.0, 0.0])
    rim = shoulder + B_ROT0 @ np.array([back - past, 0.0, 0.0])
    return rim + (CAP_R_HAND @ DOWN_Y.T) @ np.array([0.0, 0.0, CAP_H / 2 + 0.013]) + CAP_TRIM


CAP_MOVE_S = 2.5  # s: the right hand brings the bottle from its rest (or CAP_WAIT) to CAP_HOLD
UNCAP_LIFT = (3.5, 1.5, 0.6, 1.2)  # s: the right hand over the bottle, around it, holding it, lifting it out
T_UNCAP_HELD = sum(UNCAP_LIFT) + CAP_MOVE_S + 0.5  # the capped bottle held firmly at CAP_HOLD


UNCAP_R = frame((1, 0, 0), (0, 0, -1))  # fingers forward, closing up-down on the cap's sides (a pinch along y would
# let the cap pivot about the pinch line when the hand turns down; probe q1: it was set down lying on its side)
SET_R = ry(math.pi / 2) @ UNCAP_R  # the same hand turned fingers down: the cap's axis vertical, open side down
CAP_SET_TRIM = np.array([0.0011, 0.0004, 0.0])  # probe q2: set down at the plain target, the cap stood 1.1 mm short in x
# and 0.4 mm in y of CAP_START; picked there for capping it then rode 1.1 mm low in the pinch and jammed on the nozzle


def uncap_grip(back: float = 0.0) -> np.ndarray:
    """The tool point (turned CAP_R_HAND) pinching the cap seated on the bottle held at CAP_HOLD (`back` m short)."""
    return cap_tcp(back=back) - CAP_TRIM  # the trim is for a cap carried in the pinch; this one sits on the nozzle


def left_uncap() -> tuple:
    """The left hand takes the cap off the bottle the right hand holds level at CAP_HOLD and stands it open side down
    at CAP_START. Returns its waypoints and the time the cap is off the nozzle."""
    pre = uncap_grip(back=0.015)
    cg = np.array([CAP_START[0], CAP_START[1], TOP + CAP_H / 2 + 0.013]) + CAP_SET_TRIM
    out = [tw(5.0, pre - np.array([0.030, 0, 0]), WIDE, "fingers forward, beside where the cap will be", UNCAP_R)]
    out.append(tw(max(0.5, T_UNCAP_HELD + 0.3 - 5.0), pre - np.array([0.030, 0, 0]), WIDE, "wait for the bottle",
                  UNCAP_R))
    out += [tw(1.2, pre, WIDE, "in line with the cap", UNCAP_R),
            tw(1.5, uncap_grip(), WIDE, "around the cap", UNCAP_R),
            tw(0.6, uncap_grip(), CAP_SQUEEZE, "pinch it", UNCAP_R),
            tw(1.5, uncap_grip(back=0.020), CAP_SQUEEZE, "pull it off the nozzle", UNCAP_R)]
    t_off = sum(w.duration for w in out)
    out += [tw(1.0, pre - np.array([0.040, 0, 0]), CAP_SQUEEZE, "clear of the bottle", UNCAP_R),
            tw(5.0, cg + np.array([0, 0, 0.050]), CAP_SQUEEZE, "turn it open side down over its mark", SET_R),
            tw(1.2, cg + np.array([0, 0, 0.002]), CAP_SQUEEZE, "stand it on the counter", SET_R),
            tw(0.6, cg + np.array([0, 0, 0.002]), WIDE, "let go", SET_R),
            tw(1.0, cg + np.array([0, 0, 0.050]), WIDE, "up", SET_R),
            kit.Home(2.0)]
    return out, t_off


def right_uncap(t_off: float) -> list:
    """The right hand takes the capped bottle out of its rest, holds it level for the left hand to uncap, and lays it
    back in its rest."""
    bg = bottle_tcp(np.asarray(BOTTLE_START, float), B_ROT0)
    a, b, c, d = UNCAP_LIFT
    out = [tw(a, bg + np.array([0, 0, 0.040]), WIDE, "over the capped control bottle", B_GRIP_R),
           tw(b, bg, WIDE, "around its body", B_GRIP_R),
           tw(c, bg, finger(FINGER_HOLD), "hold it, not squeezing", B_GRIP_R),
           tw(d, bg + np.array([0, 0, 0.050]), finger(FINGER_HOLD), "lift it out of the rest", B_GRIP_R),
           tw(CAP_MOVE_S, CAP_HOLD, finger(FINGER_HOLD), "held level for the cap to come off", B_GRIP_R),
           tw(0.5, CAP_HOLD, finger(FINGER_FIRM), "hold it firmly", B_GRIP_R)]
    t = sum(w.duration for w in out)
    out += [tw(max(0.5, t_off + 0.5 - t), CAP_HOLD, finger(FINGER_FIRM), "hold while the cap comes off", B_GRIP_R),
            tw(0.5, CAP_HOLD, finger(FINGER_HOLD), "ease the grip", B_GRIP_R),
            tw(3.0, bg + np.array([0, 0, 0.040]), finger(FINGER_HOLD), "back over the rest", B_GRIP_R),
            tw(1.5, bg + np.array([0, 0, 0.001]), finger(FINGER_HOLD), "into the groove", B_GRIP_R),
            tw(0.6, bg + np.array([0, 0, 0.001]), WIDE, "let go", B_GRIP_R),
            tw(1.0, bg + np.array([0, 0, 0.050]), WIDE, "up", B_GRIP_R),
            kit.Home(2.0)]  # the strip is taken from home, as when the bottle started uncapped (probe q1: coming
    # straight from the rest, the strip jammed at the port's mouth and slid 4 mm back in the pinch)
    return out


def right_plan(t_off: float) -> tuple:
    """The right hand's waypoints and the times (s) of its key events."""
    first = right_uncap(t_off)
    t0 = sum(w.duration for w in first)
    g = strip_grasp_tcp()
    up = g + np.array([0.0, 0.0, 0.045])
    pre = insert_tcp(M_INS, back=0.015)
    near = insert_tcp(M_INS, back=0.004)
    seat = insert_tcp(M_INS, past=PUSH_PAST)
    out = [tw(3.0, g + np.array([0, 0, 0.030]), WIDE, "over the standing strip", DOWN_Y),
           tw(1.5, g, finger(0.006), "pads either side of the strip's top", DOWN_Y),
           tw(0.6, g, FINGER_CLOSED, "pinch the strip", DOWN_Y),
           tw(1.5, up, FINGER_CLOSED, "pull it up out of the bottle", DOWN_Y),
           tw(4.0, pre + np.array([0, 0, 0.020]), FINGER_CLOSED, "turn it level, grey end forward", INSERT_R),
           tw(1.2, pre, FINGER_CLOSED, "in line with the port", INSERT_R),
           tw(1.0, near, FINGER_CLOSED, "at the port's mouth", INSERT_R),
           tw(2.0, seat, FINGER_CLOSED, "push it fully in", INSERT_R),
           tw(0.6, seat, FINGER_CLOSED, "hold it in (the meter beeps)", INSERT_R),
           tw(0.6, seat, finger(0.004), "let go", INSERT_R),
           tw(1.2, seat - np.array([0.040, 0.0, 0.0]), finger(0.006), "back the fingers off", INSERT_R)]
    t_inserted = t0 + sum(w.duration for w in out[:9])
    out = first + out
    b0 = np.asarray(BOTTLE_START, float)
    bg = bottle_tcp(b0, B_ROT0)
    out += [tw(3.5, bg + np.array([0, 0, 0.040]), WIDE, "over the control bottle", B_GRIP_R),
            tw(1.5, bg, WIDE, "around its body", B_GRIP_R),
            tw(0.6, bg, finger(FINGER_HOLD), "hold it, not squeezing", B_GRIP_R),
            tw(1.2, bg + np.array([0, 0, 0.050]), finger(FINGER_HOLD), "lift it out of the rest", B_GRIP_R)]
    d = drop_tcp()
    R_drop = ry(-DROP_TILT) @ B_GRIP_R
    out += [tw(2.5, d + np.array([0, 0, 0.030]), finger(FINGER_HOLD), "tip down over the slide", R_drop),
            tw(1.2, d, finger(FINGER_HOLD), "tip 12 mm over the glass", R_drop),
            tw(0.1, d, finger(FINGER_SQUEEZE), "squeeze", R_drop),
            tw(SQUEEZE_S, d, finger(FINGER_SQUEEZE), "one drop", R_drop),
            tw(0.1, d, finger(FINGER_HOLD), "ease off", R_drop),
            tw(0.6, d, finger(FINGER_HOLD), "let the drop settle", R_drop)]
    t_drop = sum(w.duration for w in out)
    out += [tw(1.2, d + np.array([0, -0.060, 0.050]), finger(FINGER_HOLD), "up and away from the slide", R_drop),
            tw(3.0, CAP_WAIT, finger(FINGER_HOLD), "level, out of the way", B_GRIP_R)]
    return out, {"inserted": t_inserted, "drop": t_drop}


def right_finish(t_now: float, ev: dict) -> list:
    """Hold the bottle level for the cap once the cap is turned; then lay it back in its rest and go home."""
    bg = bottle_tcp(np.asarray(BOTTLE_START, float), B_ROT0)
    out = [tw(max(0.5, ev["cap_turned"] - t_now), CAP_WAIT, finger(FINGER_HOLD), "wait, level", B_GRIP_R),
           tw(CAP_MOVE_S, CAP_HOLD, finger(FINGER_HOLD), "held level for the cap", B_GRIP_R),
           tw(0.5, CAP_HOLD, finger(FINGER_FIRM), "hold it firmly", B_GRIP_R)]
    t_now += sum(w.duration for w in out)
    out += [tw(max(0.5, ev["capped"] + 1.2 - t_now), CAP_HOLD, finger(FINGER_FIRM), "hold for the cap", B_GRIP_R),
            tw(0.5, CAP_HOLD, finger(FINGER_HOLD), "ease the grip", B_GRIP_R),
            tw(3.0, bg + np.array([0, 0, 0.040]), finger(FINGER_HOLD), "back over the rest", B_GRIP_R),
            tw(1.5, bg + np.array([0, 0, 0.001]), finger(FINGER_HOLD), "into the groove", B_GRIP_R),
            tw(0.6, bg + np.array([0, 0, 0.001]), WIDE, "let go", B_GRIP_R),
            tw(1.0, bg + np.array([0, 0, 0.050]), WIDE, "up", B_GRIP_R),
            kit.Home(2.5)]
    return out


def left_plan(ev: dict, first: list) -> list:
    start = np.array([METER_START[0], METER_START[1], TOP])
    g = meter_tcp(start)
    sag = np.array([0.0, 0.0, M_LIFT_SAG])
    out = list(first)
    out += [tw(3.0, g + np.array([0, 0, 0.040]), WIDE, "over the meter's far end", DOWN_Y),
           tw(1.5, g, WIDE, "around its sides", DOWN_Y),
           tw(0.8, g, M_SQUEEZE, "hold it", DOWN_Y),
           tw(1.5, g + np.array([0, 0, 0.060]), M_SQUEEZE, "lift it", DOWN_Y),
           tw(2.5, meter_tcp(M_INS) + sag, M_SQUEEZE, "port toward the right hand, level", DOWN_Y)]
    t = sum(w.duration for w in out)
    out.append(tw(max(0.5, ev["inserted"] + 1.0 - t), meter_tcp(M_INS) + sag, M_SQUEEZE, "steady for the strip",
                  DOWN_Y))
    t = sum(w.duration for w in out)
    out.append(tw(max(0.5, ev["drop"] + 2.5 - t), meter_tcp(M_INS) + sag, M_SQUEEZE, "wait for the drop", DOWN_Y))
    R_dip = ry(DIP_TILT)
    at = np.array([DROP_AT[0], DROP_AT[1]])
    dip = dip_meter_origin(at)
    over = dip + np.array([0.0, 0.0, 0.015])
    out += [tw(2.5, meter_tcp(over, R_dip) + sag, M_SQUEEZE, "tip it port-end down over the drop", R_dip @ DOWN_Y),
            tw(1.5, meter_tcp(dip, R_dip) + sag, M_SQUEEZE, "strip's tip into the drop", R_dip @ DOWN_Y),
            tw(DWELL_S, meter_tcp(dip, R_dip) + sag, M_SQUEEZE, "hold until it beeps", R_dip @ DOWN_Y),
            tw(1.0, meter_tcp(over + np.array([0, 0, 0.020]), R_dip) + sag, M_SQUEEZE, "lift it off", R_dip @ DOWN_Y),
            tw(2.5, g + np.array([0, 0, 0.020]), M_SQUEEZE, "level, back over its place", DOWN_Y),
            tw(1.2, g + np.array([0, 0, 0.001]), M_SQUEEZE, "set it down", DOWN_Y),
            tw(0.6, g + np.array([0, 0, 0.001]), WIDE, "let go", DOWN_Y),
            tw(1.0, g + np.array([0, 0, 0.050]), WIDE, "up", DOWN_Y)]
    cg = np.array([CAP_START[0], CAP_START[1], TOP + CAP_H / 2 + 0.013])
    pre = cap_tcp(back=0.015)
    out += [tw(2.0, cg + np.array([0, 0, 0.040]), WIDE, "over the cap", DOWN_Y),
            tw(1.5, cg, WIDE, "around it", DOWN_Y),
            tw(0.6, cg, CAP_SQUEEZE, "hold it", DOWN_Y),
            tw(1.2, cg + np.array([0, 0, 0.050]), CAP_SQUEEZE, "lift it", DOWN_Y)]
    t = sum(w.duration for w in out)
    out.append(tw(5.0, pre - np.array([0.030, 0, 0]), CAP_SQUEEZE, "turn it open side forward", CAP_R_HAND))
    ev["cap_turned"] = sum(w.duration for w in out)  # only now does the right hand bring the bottle in (probe p2:
    # bringing it in while the left reached for the cap, the elbows met and knocked the bottle off its pose)
    out += [tw(CAP_MOVE_S + 0.5, pre - np.array([0.030, 0, 0]), CAP_SQUEEZE, "wait for the bottle", CAP_R_HAND),
            tw(1.2, pre, CAP_SQUEEZE, "in line with the nozzle", CAP_R_HAND),
            tw(1.5, cap_tcp(past=CAP_PUSH), CAP_SQUEEZE, "push it on", CAP_R_HAND),
            tw(0.5, cap_tcp(past=CAP_PUSH), CAP_SQUEEZE, "hold", CAP_R_HAND),
            tw(0.6, cap_tcp(), WIDE, "let go", CAP_R_HAND),
            tw(1.0, press_tcp(back=0.030), WIDE, "back off", CAP_R_HAND),
            tw(0.5, press_tcp(back=0.030), FINGER_CLOSED, "close the fingers", CAP_R_HAND),
            tw(1.5, press_tcp(past=CAP_PRESS), FINGER_CLOSED, "press the cap home with the fingertips", CAP_R_HAND),
            tw(0.5, press_tcp(past=CAP_PRESS), FINGER_CLOSED, "hold", CAP_R_HAND)]
    ev["capped"] = sum(w.duration for w in out)
    out += [tw(1.0, press_tcp(back=0.040), FINGER_CLOSED, "back off", CAP_R_HAND),
            kit.Home(2.5)]
    return out


def demo():
    """Both hands first uncap the control bottle (right holds it, left pulls the cap and stands it on the counter),
    and the right lays it back in its rest. Right hand: strip out of the bottle and into the meter's port; a drop of control on the slide; the bottle held
    level for the cap, then laid back in its rest. Left hand: holds the meter for the strip, touches the strip's tip to
    the drop, puts the meter down, pushes the cap onto the bottle's nozzle."""
    first, t_off = left_uncap()
    r, ev = right_plan(t_off)
    left = left_plan(ev, first)
    r += right_finish(sum(w.duration for w in r), ev)
    tl, tr = sum(w.duration for w in left), sum(w.duration for w in r)
    if tl < tr:
        left.append(kit.Home(tr - tl))
    elif tr < tl:
        r.append(kit.Home(tl - tr))
    return {"left": left, "right": r}
