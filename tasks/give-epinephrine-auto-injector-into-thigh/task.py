"""On a nurse's work table (the station table): an injection training pad (Limbs & Things 00310, 21 x 16 x 11 cm, a
skin-and-muscle block made to strap to an arm or thigh) lying in front of the arms with its long side across them,
standing for the middle of the patient's outer thigh; a white medication tray on the right with an epinephrine
auto-injector trainer (EpiPen Trainer: the EpiPen's body and mechanism without needle or drug) lying in the groove of
a foam rest, blue safety release on, orange tip toward the robot.

The job is a registered nurse's: "Administer medications to patients and monitor patients for reactions or side
effects" (O*NET 29-1141.00). The maker's taught steps: grasp the pen with the orange tip pointing down; pull the blue
safety release straight up without bending or twisting it; swing and push the orange tip firmly into the middle of
the outer thigh, at a right angle, until it clicks; hold it there for 3 seconds; remove it. The real failures are kept
possible: the pen held upside down (blue end at the thigh, the thumb-injection error), the release pulled off at an
angle (it jams on its pin), firing into the air, the table or off the target, pressing at a slant so the tip skids,
lifting before 3 s, dropping the pen.

The orange sleeve is a slide in the pen on a return spring stiff enough that it reaches its click depth (7 mm in) at
23 N, the measured activation force of the same cartridge design (EpiPen Jr: 23.01 +- 3.96 N, n = 15; Schwirtz &
Seeger, J Asthma Allergy 2010;3:159-167). For the 0.3 mg EpiPen every activation measured was below 50 N and 20 N was
used as its activation load (Schwirtz & Seeger, J Asthma Allergy 2012;5:39-49). The trainer's own force is not
published. While the blue release is on its pin the sleeve is blocked, as in the real pen. The release is a loose cap
over a 10 mm pin at the blue end; its press fit (EpiPen Jr: 9.56 +- 2.36 N, 2010 study) is not modeled, so any
straight pull takes it off. Where the tip
lands is measured in the pad's own frame. Every threshold below appears once, with the same value in task.json and
the README.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import guardkit as gk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 1200
INSTRUCTION = ("Give the trainer auto-injector into the target on the training pad: pull off the blue safety release, "
               "press the orange tip into the pad until it clicks, hold it there for 3 seconds, then lay the trainer "
               "back on the tray.")

ORANGE = kit.Look((0.95, 0.45, 0.08), 0.5)
BLUE = kit.Look((0.10, 0.35, 0.80), 0.45)
BODY = kit.Look((0.72, 0.73, 0.74), 0.45)  # the trainer's gray body (the US EpiPen Trainer is shaded gray)
LABEL = kit.Look((0.55, 0.56, 0.58), 0.5)  # its darker gray TRAINER label band
SKIN = kit.Look((0.86, 0.70, 0.60), 0.6)
PAD_BASE = kit.Look((0.30, 0.30, 0.32), 0.7)
TARGET = kit.Look((0.20, 0.55, 0.25), 0.7)  # the printed target outline on the pad's cover
TRAY_LOOK = kit.Look((0.96, 0.96, 0.95), 0.4)
FOAM = kit.Look((0.92, 0.92, 0.90), 0.9)

# -- the pen (EpiPen Trainer). Its own frame: origin at the body's middle, x along the pen toward the blue end. About
#    14 cm overall (it fits the 5-1/2 in carrier tube) and 2.1 cm across. ----------------------------------------------
BODY_L, BODY_R = 0.104, 0.0105
BODY_KG = 0.030
SLEEVE_OUT = 0.012  # m: the orange sleeve stands this far out of the body at rest
SLEEVE_R, SLEEVE_LEN = 0.0075, 0.020
SLEEVE_KG = 0.006
TRAVEL = 0.008  # m: the sleeve's travel into the body
CLICK_Q = 0.007  # m: pushed in this far, the trainer clicks (fires)
HOLD_Q = 0.004  # m: after the click, the tip counts as held on the thigh while pushed in at least this far
F_FIRE = float(os.environ.get("F_FIRE", "23.0"))  # N at the click depth (EpiPen Jr, Schwirtz & Seeger 2010); settable
K_FIRE = F_FIRE / CLICK_Q  # N/m, the sleeve's return spring
K_BLOCK = 2.0e5  # N/m: the sleeve held while the blue release is on (it blocks the trigger)
D_SLEEVE = 4.0  # N*s/m
POST_R, POST_L = 0.004, 0.010  # the release's pin seat at the blue end
CAP_L, CAP_OUT, CAP_IN = 0.024, 0.009, 0.0055  # blue release: length, outer half-width, bore half-width (1.5 mm play round the 4 mm pin: a straight pull slides it off without binding)
CAP_KG = 0.003
CAP_SEAT = np.array([BODY_L / 2 + CAP_L / 2 + 0.0005, 0.0, 0.0])  # the release's middle, pen frame, when it is on
CAP_OFF = 0.015  # m: the release more than this from its seat (past the 10 mm pin) is off
TIP_LOCAL = np.array([-BODY_L / 2 - SLEEVE_OUT, 0.0, 0.0])  # the sleeve's tip at rest, pen frame
STATE = {"cap_on": True}  # set by the scorer each control step; the sleeve's law reads it on the next


class SleeveSpring:
    """The orange sleeve's drive: a stiff PD toward fully out, K_FIRE while the release is off (23 N at the click),
    K_BLOCK while it is on (blocked). Implicit (PhysX), so the stiff spring is stable on the light sleeve."""

    def drive(self, key, q, qd, jq, mem):
        return 0.0, (K_BLOCK if STATE["cap_on"] else K_FIRE), 0.0, D_SLEEVE


SLEEVE = kit.Link(
    "sleeve", "prismatic", "X", anchor=(-BODY_L / 2, 0.0, 0.0), lower=0.0, upper=TRAVEL, mass=SLEEVE_KG,
    meshes=(kit.cylinder((-SLEEVE_OUT + SLEEVE_LEN / 2, 0.0, 0.0), SLEEVE_R, SLEEVE_LEN, ORANGE, axis="x"),),
    colliders=(kit.Collider((-SLEEVE_OUT + SLEEVE_LEN / 2, 0.0, 0.0), radius=SLEEVE_R, height=SLEEVE_LEN, axis="X"),),
    law=SleeveSpring())

PEN_START = (0.240, -0.275)  # pen middle on the rest (x, y); yaw 0: orange tip toward the robot (-x)
TRAY_C, TRAY_SIZE = (0.240, -0.275), (0.220, 0.140, 0.010)  # white ABS medication tray, fixed
TRAY_TOP = TOP + TRAY_SIZE[2]
REST_SIZE = (0.160, 0.050, 0.010)  # white foam rest with a 2.4 cm groove 4 mm deep along x, on the tray
REST_FLOOR = TRAY_TOP + REST_SIZE[2] - 0.004
PEN_Z0 = REST_FLOOR + BODY_R
PEN = kit.Mechanism(
    "pen", pos=(PEN_START[0], PEN_START[1], PEN_Z0), mass=BODY_KG,
    size=(BODY_L + 2 * SLEEVE_OUT, 2 * BODY_R, 2 * BODY_R),
    meshes=(kit.cylinder((0.0, 0.0, 0.0), BODY_R, BODY_L, BODY, axis="x"),
            kit.cylinder((-0.005, 0.0, 0.0), BODY_R + 0.0003, 0.050, LABEL, axis="x"),
            kit.cylinder((BODY_L / 2 + POST_L / 2, 0.0, 0.0), POST_R, POST_L, BODY, axis="x")),
    colliders=(kit.Collider((0.0, 0.0, 0.0), radius=BODY_R, height=BODY_L, axis="X"),
               kit.Collider((BODY_L / 2 + POST_L / 2, 0.0, 0.0), radius=POST_R, height=POST_L, axis="X")),
    links=(SLEEVE,))


def _cap_parts() -> tuple:
    """The release's walls round its bore and its end plate (its own frame: origin at its middle, x out)."""
    wall = CAP_OUT - CAP_IN
    tube = CAP_L - 0.004
    tx = -CAP_L / 2 + tube / 2
    return (((tx, CAP_IN + wall / 2, 0.0), (tube, wall, 2 * CAP_OUT)),
            ((tx, -CAP_IN - wall / 2, 0.0), (tube, wall, 2 * CAP_OUT)),
            ((tx, 0.0, CAP_IN + wall / 2), (tube, 2 * CAP_IN, wall)),
            ((tx, 0.0, -CAP_IN - wall / 2), (tube, 2 * CAP_IN, wall)),
            ((CAP_L / 2 - 0.002, 0.0, 0.0), (0.004, 2 * CAP_OUT, 2 * CAP_OUT)))


CAP = kit.Prop(
    "release", pos=(PEN_START[0] + float(CAP_SEAT[0]), PEN_START[1], PEN_Z0), mass=CAP_KG,
    size=(CAP_L, 2 * CAP_OUT, 2 * CAP_OUT),
    meshes=(kit.cylinder((0.0, 0.0, 0.0), CAP_OUT, CAP_L, BLUE, axis="x"),),
    colliders=tuple(kit.Collider(c, size=s) for c, s in _cap_parts()))

TRAY = kit.Prop("tray", pos=(TRAY_C[0], TRAY_C[1], TOP), size=TRAY_SIZE,
                meshes=(kit.box((0, 0, TRAY_SIZE[2] / 2), TRAY_SIZE, TRAY_LOOK),),
                colliders=(kit.Collider((0, 0, TRAY_SIZE[2] / 2), size=TRAY_SIZE),))
_GW, _GD = 0.024, 0.004  # the rest's groove width and depth
_SIDE = (REST_SIZE[1] - _GW) / 2
_REST_PARTS = (((0.0, 0.0, (REST_SIZE[2] - _GD) / 2), (REST_SIZE[0], REST_SIZE[1], REST_SIZE[2] - _GD)),
               ((0.0, _GW / 2 + _SIDE / 2, REST_SIZE[2] - _GD / 2), (REST_SIZE[0], _SIDE, _GD)),
               ((0.0, -_GW / 2 - _SIDE / 2, REST_SIZE[2] - _GD / 2), (REST_SIZE[0], _SIDE, _GD)))
REST = kit.Prop("pen_rest", pos=(TRAY_C[0], TRAY_C[1], TRAY_TOP), size=REST_SIZE,
                meshes=tuple(kit.box(c, s, FOAM) for c, s in _REST_PARTS),
                colliders=tuple(kit.Collider(c, size=s) for c, s in _REST_PARTS))

# -- the injection pad (Limbs & Things 00310: 21 cm long, 16 cm wide, 11 cm high, 0.6 kg), fixed (its own weight and
#    the bench mat hold it; strapped on a thigh it would be held by the strap). Modelled as a 3 cm base under a
#    cylinder crown (radius 8.5 cm, axis along y, 2.5 cm above the table): 16.2 cm wide at the table, 11 cm high. ----
PAD_C = (0.300, -0.090)
PAD_L, PAD_R, PAD_AXIS_Z = 0.210, 0.085, 0.025
CREST_Z = TOP + PAD_AXIS_Z + PAD_R  # the crown's top, 11 cm above the table
TARGET_Y = 0.030  # m: the printed target spans 6 cm along the pad (y) ...
TARGET_ARC = 0.020  # ... and 4 cm across its crown (2 cm of arc either side of the top)
ON_SKIN = 0.004  # m: the tip within 4 mm of the skin surface is on it (the trainer's tip on a soft cover)
SQUARE_DEG = 20.0  # the pen within 20 degrees of the skin's normal counts as at a right angle
PAD = kit.Prop(
    "pad", pos=(PAD_C[0], PAD_C[1], TOP), size=(0.162, PAD_L, PAD_AXIS_Z + PAD_R),
    meshes=(kit.box((0.0, 0.0, 0.015), (0.162, PAD_L, 0.030), PAD_BASE),
            kit.cylinder((0.0, 0.0, PAD_AXIS_Z), PAD_R, PAD_L, SKIN, axis="y"),
            kit.box((0.0, 0.0, PAD_AXIS_Z + PAD_R + 0.0003), (2 * TARGET_ARC, 2 * TARGET_Y, 0.0006), TARGET)),
    colliders=(kit.Collider((0.0, 0.0, 0.015), size=(0.162, PAD_L, 0.030)),
               kit.Collider((0.0, 0.0, PAD_AXIS_Z), radius=PAD_R, height=PAD_L, axis="Y")))

OBJECTS = [PAD, TRAY, REST, PEN, CAP]

# -- scoring ----------------------------------------------------------------------------------------------------------
HOLD_S = 3.0  # the maker's hold after the click
HOLD_STEPS = round(HOLD_S / DT)
OFF_TABLE_Z = TOP - 0.05  # the pen's middle below this: it fell off the table
ON_TRAY_Z = 0.030  # m: the pen's middle within 3 cm above the tray's top, inside its outline: lying on the tray
LYING_DEG = 30.0  # its axis within 30 degrees of horizontal
STILL_STEP = 0.0005  # m per control step (1 cm/s)
SUCCESS_STEPS = round(SUCCESS_HOLD_S / DT)
PEN_PARTS = (gk.Seg((-BODY_L / 2 - SLEEVE_OUT, 0.0, 0.0), (BODY_L / 2 + POST_L, 0.0, 0.0), BODY_R),)
GRIPPERS = ("left_gripper", "right_gripper")


def rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pen_axis(objects) -> np.ndarray:
    """Unit vector along the pen toward its blue end (world)."""
    return rotm(objects["pen"].quat_xyzw)[:, 0]


def sleeve_q(objects) -> float:
    return float(kit.joint(objects, "pen/sleeve").pos)


def tip_world(objects) -> np.ndarray:
    """Where the orange sleeve's tip is now (it moves in with the sleeve)."""
    p = objects["pen"]
    return np.asarray(p.pos, float) + rotm(p.quat_xyzw) @ (TIP_LOCAL + np.array([sleeve_q(objects), 0.0, 0.0]))


def cap_on(objects) -> bool:
    """The blue release still on its pin: its middle within CAP_OFF of its seat on the pen."""
    p = objects["pen"]
    seat = np.asarray(p.pos, float) + rotm(p.quat_xyzw) @ CAP_SEAT
    return float(np.linalg.norm(np.asarray(objects["release"].pos, float) - seat)) <= CAP_OFF


def on_target(objects) -> bool:
    """The tip on the pad's skin inside the printed target, the pen within SQUARE_DEG of the skin's normal there,
    pointing into it (orange end at the thigh). Measured in the pad's frame (the pad does not turn)."""
    t = tip_world(objects)
    pad = np.asarray(objects["pad"].pos, float)
    rel = t - pad
    if abs(rel[1]) > TARGET_Y:
        return False
    radial = np.array([rel[0], 0.0, rel[2] - PAD_AXIS_Z])
    r = float(np.linalg.norm(radial))
    if r < 1e-6 or abs(r - PAD_R) > ON_SKIN:
        return False
    normal = radial / r
    if abs(PAD_R * math.atan2(normal[0], normal[2])) > TARGET_ARC:
        return False
    into = -pen_axis(objects)  # from the blue end toward the orange tip
    return float(np.dot(into, -normal)) >= math.cos(math.radians(SQUARE_DEG))


def on_tray(objects) -> bool:
    p = np.asarray(objects["pen"].pos, float)
    tray = np.asarray(objects["tray"].pos, float)
    inside = abs(p[0] - tray[0]) <= TRAY_SIZE[0] / 2 and abs(p[1] - tray[1]) <= TRAY_SIZE[1] / 2
    above = 0.0 <= p[2] - TRAY_TOP <= ON_TRAY_Z
    flat = abs(float(pen_axis(objects)[2])) <= math.sin(math.radians(LYING_DEG))
    return inside and above and flat


class Ledger:
    """One trial: the release, the click (where it fired), the 3 s hold after it, the fall latch and the end check."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        STATE["cap_on"] = True
        self.cap_off = False
        self.fired = False
        self.misfire = False
        self.hold = 0
        self.early = False
        self.fell = False
        self.last = None
        self.good = 0
        self.click = 0

    def step(self, objects) -> None:
        on = cap_on(objects)
        STATE["cap_on"] = on and not self.cap_off
        if not on:
            self.cap_off = True
        if float(objects["pen"].pos[2]) < OFF_TABLE_Z:
            self.fell = True
        q = sleeve_q(objects)
        target = on_target(objects)
        if not self.fired and self.cap_off:
            self.click = self.click + 1 if q >= CLICK_Q else 0
            if self.click >= 2:  # two control steps in a row: a click, not a solver blip
                self.fired = True
                self.misfire = not target
        elif self.fired and not self.misfire and not self.early and self.hold < HOLD_STEPS:
            if q >= HOLD_Q and target:
                self.hold += 1
            else:
                self.early = True
        p = np.asarray(objects["pen"].pos, float)
        still = self.last is not None and float(np.linalg.norm(p - self.last)) <= STILL_STEP
        self.last = p
        done = (self.tier() == 0.75 and on_tray(objects) and still
                and gk.released_clear(objects, "pen", PEN_PARTS, 0.01, grippers=GRIPPERS))
        self.good = self.good + 1 if done else 0

    def tier(self) -> float:
        if self.fell or self.misfire:
            return 0.0
        if self.fired:
            return 0.75 if self.hold >= HOLD_STEPS else 0.5
        return 0.25 if self.cap_off else 0.0


_L = None


def _update(objects):
    global _L
    if _L is None or _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else ("obj", id(objects))
    if key == _L.key:
        return
    _L.key = key
    _L.step(objects)


def success(objects):
    """Fired into the target, held 3 s, and the pen lying on the tray, let go and still: this step counts toward the
    1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """1.0: the pen fired (orange sleeve in to its 7 mm click) with its tip on the printed target of the pad, square to
    the skin (within 20 degrees), held there pushed in at least 4 mm for 3 s after the click, then laid on the tray,
    let go by both hands and still for 1 s. 0.75: fired and held like that, not yet laid on the tray, let go or
    still. 0.5: fired on target but lifted (or let off, or slid off the target) before 3 s. 0.25: the blue release
    pulled off, the pen not yet fired. Otherwise 0, including doing nothing. The whole trial scores 0 if the pen fires
    anywhere but the target (or at a slant, or blue end first) or falls off the table."""
    _update(objects)
    s = _L.tier()
    if s == 0.75 and _L.good >= SUCCESS_STEPS:
        s = 1.0
    return s


# -- scripted demo: open loop from the fixed layout -------------------------------------------------------------------
# Right hand: takes the pen off its rest from above (fingers across it, at its middle), lifts it and turns its grip
# forward so the pen stands orange tip down, holds it while the left hand pulls the release straight up, then carries
# it over the target, pushes the tip down into the crown until it clicks and holds it 3.6 s, lifts it, turns it back
# level and lays it in the rest's groove. Left hand: waits behind the release at its height, slides its open fingers
# round it from behind (the left arm cannot reach far enough above it to come down onto it), pulls it straight up 2 cm,
# sets it down on the table on the left, then moves away.
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open
TIP_PAST_TCP = 0.0128  # m: the fingertips reach this far past the tool point
CARRY_Z = TOP + 0.120
UPRIGHT = -math.pi / 2  # pitch that turns the straight-down grip to point forward (+x): the held pen stands up
CAP_POSE = np.array([0.260, 0.070, TOP + 0.090])  # the pen's middle while the release comes off (tip 2.6 cm up, 4.4 cm clear of the pad's +y end)
LEVEL_POSE = np.array([0.270, -0.225, TOP + 0.180])  # where the right hand turns the pen back level after the hold (over the tray, clear of the pad)
STAND_POSE = np.array([0.270, -0.225, TOP + 0.200])  # the pen is stood up here, beside the pad (tip 13.6 cm up, over the 11 cm crown when carried)
SIDE_IN = 0.060  # m: the left hand comes in along +x from this far behind the release, at grip height
SWING = float(os.environ.get("SWING", "0.040"))  # m: the tip starts this far above the crown
PUSH_PAST = float(os.environ.get("PUSH_PAST", "0.065"))  # m: the grip is commanded this far past touching (the arm gives ~2.5 cm at 13 N, so 6.5 cm brings the 23 N click)
HOLD_DEMO_S = 3.6
PEN_GRIP_UP = 0.004  # m: the tool point this far above the pen's axis on the rest (fingertips clear of the foam)
CAP_GRIP_UP = 0.008  # m: the left tool point this far above the release's middle: the downward fingertips (12.8 mm past it) end 7.7 mm above the pen body, on the release
PULL_TOP = 0.189  # m above the table: the pull ends here, 2.6 cm up (the release clears its 10 mm pin after 1 cm)


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * max(0.0, half) / FINGER_HALF_OPEN


def _w(d, pos, f, label, yaw=0.0):
    return Waypoint(float(d), tuple(float(v) for v in pos), float(f), label, float(yaw))


def _t(d, pos, f, label, pitch):
    return kit.Tilted(float(d), tuple(float(v) for v in pos), float(f), label, 0.0, 0.0, float(pitch))


R_TIMES = (2.5, 2.0, 0.8, 1.5, 1.2, 2.4, 1.8, 1.0)  # up to holding the pen upright at CAP_POSE
CAP_START_S = sum(R_TIMES)
L_TIMES = (1.5, 0.6, 1.2)  # down to the release, pinch, pull straight up
CAP_DONE_S = CAP_START_S + sum(L_TIMES) + 2.3  # the right hand moves only once the left has carried the release clear


def right_plan() -> list:
    x0, y0 = PEN_START
    z0 = PEN_Z0 + PEN_GRIP_UP
    wide, shut = finger_for(0.018), FINGER_CLOSED
    tip_up = BODY_L / 2 + SLEEVE_OUT  # pen middle above its tip when upright
    over = (PAD_C[0], PAD_C[1], CREST_Z + tip_up + SWING)
    press = (PAD_C[0], PAD_C[1], CREST_Z + tip_up - PUSH_PAST)
    return [_w(R_TIMES[0], (x0, y0, CARRY_Z), wide, "over the pen"),
            _w(R_TIMES[1], (x0, y0, z0), wide, "down round its middle"),
            _w(R_TIMES[2], (x0, y0, z0), shut, "grip it"),
            _w(R_TIMES[3], (x0, y0, CARRY_Z), shut, "lift it"),
            _w(R_TIMES[4], STAND_POSE, shut, "up beside the pad"),
            _t(R_TIMES[5], STAND_POSE, shut, "stand it up, orange tip down", UPRIGHT),
            _t(R_TIMES[6], (CAP_POSE[0], CAP_POSE[1], STAND_POSE[2]), shut, "carry it upright, high over the pad", UPRIGHT),
            _t(R_TIMES[7], CAP_POSE, shut, "down to where the release comes off", UPRIGHT),
            _t(CAP_DONE_S - CAP_START_S, CAP_POSE, shut, "hold it while the release comes off", UPRIGHT),
            _t(1.2, (CAP_POSE[0], CAP_POSE[1], over[2]), shut, "lift it straight up, tip clear of the pad", UPRIGHT),
            _t(1.6, over, shut, "over the target", UPRIGHT),
            _t(0.6, press, shut, "push the tip in until it clicks", UPRIGHT),
            _t(HOLD_DEMO_S, press, shut, "hold it 3 s", UPRIGHT),
            _t(1.0, over, shut, "lift it off", UPRIGHT),
            _t(1.5, (LEVEL_POSE[0], LEVEL_POSE[1], over[2]), shut, "back toward the tray, high over the pad", UPRIGHT),
            _t(1.0, LEVEL_POSE, shut, "down beside the pad", UPRIGHT),
            _w(4.5, LEVEL_POSE, shut, "turn it level"),
            _w(1.5, (x0, y0, CARRY_Z), shut, "over the rest"),
            _w(1.5, (x0, y0, z0 + 0.003), shut, "down into the groove"),
            _w(0.8, (x0, y0, z0 + 0.003), wide, "let go"),
            _w(1.2, (x0, y0, CARRY_Z), wide, "lift clear"),
            _w(1.5, (0.18, -0.30, CARRY_Z), wide, "out of the way"),
            _w(3.0, (0.18, -0.30, CARRY_Z), wide, "wait")]


def left_plan() -> list:
    cap = CAP_POSE + np.array([0.0, 0.0, float(CAP_SEAT[0])])
    grip = cap + np.array([0.0, 0.0, CAP_GRIP_UP])
    behind = grip - np.array([SIDE_IN, 0.0, 0.0])
    pulled = np.array([grip[0], grip[1], TOP + PULL_TOP])
    wide, shut = finger_for(0.016), FINGER_CLOSED
    aside = (0.200, 0.200, TOP + 0.030)  # well to the left of the pen's path to the target
    return [_w(3.0, behind, wide, "behind the release, at its height"),
            _w(CAP_START_S - 3.0, behind, wide, "wait for the pen"),
            _w(L_TIMES[0], grip, wide, "slide the open fingers round it"),
            _w(L_TIMES[1], grip, shut, "pinch it"),
            _w(L_TIMES[2], pulled, shut, "pull it straight up"),
            _w(2.0, (aside[0], aside[1], pulled[2]), shut, "carry it aside"),
            _w(1.2, aside, shut, "down to the table"),
            _w(0.6, aside, wide, "let it go"),
            _w(1.2, (aside[0], aside[1], CARRY_Z), wide, "lift clear"),
            _w(1.5, (0.18, 0.30, CARRY_Z), wide, "out of the way"),
            _w(20.0, (0.18, 0.30, CARRY_Z), wide, "wait")]


def demo():
    """Right hand takes the pen and stands it up; left hand pulls the release; right hand fires it into the target,
    holds it 3 s and lays it back on the tray."""
    return {"left": left_plan(), "right": right_plan()}
