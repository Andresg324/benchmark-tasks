"""A new spin-on oil filter (Kawasaki 49065-0710, the Teryx and Teryx4 filter) has been started on the threaded nipple
of a remote oil filter mount (a filter head) bolted to the bench, filter pointing up, three quarters of a turn short of
its gasket touching the head. Beside it on the bench lies the tool Kawasaki's torque figure assumes: a 65 mm, 14-flute
cap-type filter wrench on a 24-inch, 1/2-inch-drive ratchet. Fitting it is the engine maker's service procedure: spin
it on by hand until the gasket contacts the head, then tighten it with the wrench to the specified 18 N*m, about three
quarters of a turn past contact, then take the wrench off. Left at contact or a little past it, the filter can work
loose under engine vibration and leak; cranked much past it, the gasket can be crushed, roll out of its groove and blow
out under oil pressure, and the filter may not come off at the next change.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import twistkit as tk  # noqa: E402  (threaded parts on top of yamkit; nothing on disk changes)
import yamkit as kit  # noqa: E402
import guardkit as gk  # noqa: E402  (after yamkit)
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Fit the new oil filter."

# The filter head (fixed, bolted to the bench): a die-cast aluminum remote oil filter mount, its top face the machined
# gasket land the filter seals on. Heights are above the table.
HEAD_C = (0.30, 0.30)  # the filter's axis (left of center: the left hand spins it on, both hands work the wrench)
HEAD_X, HEAD_Y, HEAD_H = 0.075, 0.090, 0.034  # the block
LAND_R, LAND_H = 0.036, 0.006  # the round gasket land on top of the block
SEAT = HEAD_H + LAND_H  # 4.0 cm: where the filter's gasket meets the head

# The filter: a small spin-on oil filter (motorcycle and small-engine size), true size. Heights are above its gasket face.
CAN_R, CAN_H = 0.033, 0.070  # 6.6 cm across, 7.0 cm tall
FILTER_MASS = 0.25
# M20 x 1.5 thread: 1.5 mm per turn. The filter starts three quarters of a turn (270 degrees) short of gasket contact,
# about four turns onto the nipple (five at contact), so it comes off only if it is unscrewed 4.25 more turns. Before
# contact it spins freely (a small hold where it is left). Past contact the squeezed gasket resists turning with a torque
# that rises linearly with the angle, to TORQUE_SPEC at three quarters of a turn past contact (the makers' equivalence:
# 3/4 turn past contact gives the specified torque), and on at the same slope (29 N*m at 1 1/4 turns): a stiff hold
# where it was left that slips at that torque, so it opposes turning either way and never turns the filter by itself
# (stick-slip friction, not a spring). A hand turning the can itself slips on it at about 2.5 N*m, about 40 degrees
# past contact; the rest needs the wrench. The hard stop at 540 degrees past contact (a turn and a half) is the gasket
# squeezed flat. The can travels down at most 0.5 mm past contact (the gasket compressing). twistkit's slide spring
# (3,000 N/m) sags about 0.8 mm under the filter, so its gasket starts about 0.3 mm above the land rather than the
# thread's 1.1 mm; the score reads the angle, not the height.
# Kawasaki's service data for this filter (Teryx4 750 owner's manual, torque table, p. 101): "Oil Filter 18 N*m
# (1.8 kgf*m, 13 ft*lb)", after a thin film of oil on the gasket.
TORQUE_SPEC = 18.0  # N*m
GASKET_RATE = TORQUE_SPEC / math.radians(270.0)  # 3.8 N*m per radian past contact
RUN_ON = 0.3  # N*m: turning it on the thread before contact (the wrench meets this before the gasket touches)
THREAD = tk.Thread(pitch=0.0015, turns=5.0, start=-270.0, max_tight=540.0, resist=0.03, tight_resist=0.05,
                   gasket=GASKET_RATE, gasket_max=GASKET_RATE * math.radians(540.0), seat_travel=0.0005,
                   max_torque=40.0)


def gasket_torque(past_deg: float) -> float:
    """The torque (N*m) the filter takes to turn further, at `past_deg` degrees past gasket contact."""
    if past_deg <= 0.0:
        return RUN_ON
    return max(GASKET_RATE * math.radians(past_deg), RUN_ON)


# Scoring: degrees past gasket contact (the seat). Filter makers print "three quarters of a turn" or "three quarters to
# one turn" after gasket contact, so full credit spans both readings.
LOOSE = 90.0  # under a quarter turn past contact (a third of the least any maker asks): not fitted: 0
SNUG = 180.0  # 90 to 180 (a quarter to a half turn): partly tightened, well short of the makers' figure: 0.25
GOOD_LO, GOOD_HI = 240.0, 360.0  # three quarters of a turn (less 30 degrees for judging it by hand) to one full turn: 1.0
CRUSH = 450.0  # past a full turn (the most any maker allows): 0.5; past 1.25 turns at any point (can crush the gasket): 0
GRIP_SLACK = 45.0  # at most 45 degrees (an eighth of a turn) of the turning past contact may be done without a hand
# Holding the filter (twistkit.held, with this task's span): fingers closed on the can (opening from 15 % of full
# closed past its 6.6 cm to 3 % open past it, 2.8 mm of clearance in all, about the pads' compliance) with the tool point
# within 1.5 cm of the can's axis, anywhere from 1 cm above the gasket face to 1.5 cm above the top: the pad tips sit
# about 1.3 cm below the tool point, so that is every grasp height with the pads on the can, from its rim to its base.
GRIP_ALONG = (0.010, CAN_H + 0.015)
CARRY = 30.0  # a holding wrist's roll not yet matched by the filter is kept at most 30 degrees (two 15-degree control
# steps): the filter catching up a step or two late as the pads take up slack; more is the pads slipping on the can
# Let go at the end: no gripper touching the filter, judged by the gripper's own shape (gk.touching: palm and both
# fingers, boxes fitted to the YAM's collision meshes, at their present opening) within gk.CONTACT (2 mm: the solver's
# resting gap plus the box fit) of the can, from its gasket face to its top. A hand hovering beside or over the can,
# open or closed, is not on it; a grip on the can (twistkit.held) is.
FILTER_SHAPE = gk.Ring(0.0, CAN_R, 0.0, CAN_H)  # in the filter's frame: origin on the axis at the gasket's face

ALU = kit.Look((0.72, 0.73, 0.74), 0.45, metallic=0.6)
LAND = kit.Look((0.85, 0.86, 0.87), 0.25, metallic=0.8)
STEEL = kit.Look((0.55, 0.56, 0.58), 0.35, metallic=0.7)
BOLT = kit.Look((0.20, 0.20, 0.22), 0.5, metallic=0.5)
PLUG = kit.Look((0.25, 0.27, 0.30), 0.6)  # gray plastic shipping plugs in the hose ports
CAN = kit.Look((0.07, 0.07, 0.08), 0.35, metallic=0.2)  # black painted steel can
WHITE = kit.Look((0.95, 0.95, 0.93), 0.6)
RUBBER = kit.Look((0.06, 0.06, 0.06), 0.8)
BLACK = kit.Look((0.05, 0.05, 0.05), 0.6)


def _filter_cap():
    """The filter in its own frame: origin on the axis at the gasket's face (the seat when it touches the head)."""
    meshes = [
        kit.lathe([(0.022, 0.0), (0.0285, 0.0), (0.0285, 0.004), (0.022, 0.004)], RUBBER),  # the gasket ring
        kit.lathe([(0.0, 0.003), (0.031, 0.003), (0.031, 0.006), (0.0, 0.006)], STEEL),  # the base plate
        kit.lathe([(CAN_R - 0.001, 0.005), (CAN_R, 0.007), (CAN_R, CAN_H - 0.003), (CAN_R - 0.003, CAN_H),
                   (0.0, CAN_H)], CAN),  # the can: a plain flat top with a small rolled edge, painted like the sides
        kit.lathe([(CAN_R + 0.0004, 0.022), (CAN_R + 0.0004, 0.048), (CAN_R - 0.002, 0.048), (CAN_R - 0.002, 0.022)],
                  WHITE),  # the white printed band
    ]
    # the printed fitting marks, on the side only (the top is plain): an index stripe down the can's side, white on the
    # black paint and black where it crosses the white band (it faces the robot at gasket contact), and three short black
    # quarter-turn ticks on the band
    for z0, z1, look in ((0.008, 0.022, WHITE), (0.048, CAN_H - 0.004, WHITE), (0.022, 0.048, BLACK)):
        meshes.append(kit.box((-CAN_R - 0.0008, 0.0, (z0 + z1) / 2), (0.0012, 0.006, z1 - z0), look))
    for k in range(1, 4):
        a = -math.pi / 2 * k  # tightening is clockwise seen from above: tick k is k quarter turns past contact
        c, s = math.cos(math.pi + a), math.sin(math.pi + a)
        meshes.append(kit.box(((CAN_R + 0.0008) * c, (CAN_R + 0.0008) * s, 0.035), (0.004, 0.004, 0.010), BLACK))
    # grip flutes pressed into the top of the can
    for k in range(20):
        a = 2 * math.pi * k / 20
        meshes.append(kit.cylinder(((CAN_R - 0.0006) * math.cos(a), (CAN_R - 0.0006) * math.sin(a), 0.058), 0.0012,
                                   0.014, CAN, n=6))
    colliders = (kit.Collider((0, 0, 0.002), radius=0.0285, height=0.004),
                 kit.Collider((0, 0, (0.004 + CAN_H) / 2), radius=CAN_R, height=CAN_H - 0.004))
    return tk.Cap("filter", anchor=(0.0, 0.0, SEAT), mass=FILTER_MASS, meshes=tuple(meshes), colliders=colliders,
                  grip=(0.0, 0.0, CAN_H - 0.015), width=2 * CAN_R, along=GRIP_ALONG)


def _head():
    meshes = [kit.box((0, 0, HEAD_H / 2), (HEAD_X, HEAD_Y, HEAD_H), ALU),
              kit.lathe([(LAND_R, HEAD_H), (LAND_R, SEAT), (0.0, SEAT)], LAND),
              kit.cylinder((0, 0, SEAT + 0.006), 0.010, 0.012, STEEL)]  # the threaded nipple (inside the filter)
    for sy in (-1, 1):  # the inlet and outlet hose ports on the sides, plugged with gray shipping plugs
        meshes.append(kit.cylinder((0, sy * (HEAD_Y / 2 + 0.006), HEAD_H / 2), 0.009, 0.012, ALU, axis="y"))
        meshes.append(kit.cylinder((0, sy * (HEAD_Y / 2 + 0.014), HEAD_H / 2), 0.007, 0.006, PLUG, axis="y"))
    for bx, by in ((-0.028, 0.035), (0.028, -0.035), (-0.028, -0.035), (0.028, 0.035)):  # the mounting bolts
        meshes.append(kit.cylinder((bx, by, HEAD_H + 0.003), 0.005, 0.006, BOLT, n=6))
    # a black witness line on the front of the land: the filter's index stripe lines up with it at gasket contact
    meshes.append(kit.box((-LAND_R - 0.0004, 0.0, HEAD_H + LAND_H / 2), (0.0008, 0.003, LAND_H), BLACK))
    meshes.append(kit.box((-HEAD_X / 2 - 0.0004, 0.0, HEAD_H - 0.008), (0.0008, 0.003, 0.016), BLACK))
    colliders = (kit.Collider((0, 0, HEAD_H / 2), size=(HEAD_X, HEAD_Y, HEAD_H)),
                 kit.Collider((0, 0, HEAD_H + LAND_H / 2), radius=LAND_R, height=LAND_H))
    return tk.Threaded("head", pos=(HEAD_C[0], HEAD_C[1], TOP), size=(HEAD_X, HEAD_Y + 0.04, SEAT + CAN_H), mass=None,
                       meshes=tuple(meshes), colliders=colliders, cap=_filter_cap(), thread=THREAD)


# --- The wrench: a 65 mm, 14-flute cap-type filter wrench on a 24-inch, 1/2-inch-drive ratchet (GearWrench 81364XP,
# 24.0 in overall). Modeled like the accepted compression-joint task's spanner: the cap is drawn only, and once it is
# down on the can, centered within 3 mm, the model holds it on the flutes and couples it to the filter. The ratchet
# drives the cap when turned clockwise (seen from above: tightening) and freewheels the other way (pawl drag 0.2 N*m),
# so the handle swings back for the next stroke without letting go. Its handle is the only collider (the hands hold
# it). An articulation on the bench: lift (z) -> turn about the filter's axis -> slide x -> slide y (the wrench), in
# the frame of a point on the filter's axis at the can's top. The wrench lies on the bench, cap down, beside the head
# at the start; lifted, it moves freely; lying on the bench its slides and turn are held.
TOOL = "tool"
CUP_R, CUP_DEPTH = 0.0355, 0.015  # the cap: 65 mm across the flutes inside, 3 mm walls; it fits over the can's top 15 mm
HEAD_R, HEAD_TOP = 0.022, 0.040  # the ratchet's head (and the square drive in the cap) above the cap's inside floor
HANDLE_Z, HANDLE_R = 0.026, 0.011  # the handle's axis above the cap's floor; it is 22 mm thick at the grip
HANDLE_X = (-0.600, -0.035)  # along the tool's -x: the handle from the head (24.0 in overall from the cap's axis)
TOOL_MASS = 1.0  # kg: the 81364XP (about 0.9 kg) plus the cap
SEAT_TOL = 0.002  # m: the cap down on the can (the lift within 2 mm of resting on its top)
CENTER_TOL = 0.003  # m: and centered on its axis within 3 mm (the flutes engage)
LIFT_FREE = 0.017  # m: lifted this far the cap's rim clears the can's top
CLEAR = 0.10  # m: the cap 10 cm or more off the filter's axis is clear of the head and can lie on the bench
LIFT_TABLE = CUP_DEPTH - (SEAT + CAN_H)  # the lift with the wrench lying on the bench, cap down (-9.5 cm)
LIFT_MAX = 0.10
K_TOOL = 400.0  # N*m/rad: the wrench, cap and can flexing between the handle and the gasket (2.6 degrees at 18 N*m)
D_TOOL = 0.5  # N*m*s/rad
PAWL_DRAG = 0.2  # N*m: the ratchet freewheeling back
# Where the wrench starts and works (see the demo): its handle swings clockwise (seen from above) through WRENCH_STROKE
# degrees from STROKE_A0 (the handle's direction from the filter's axis, degrees from +x) on each stroke. At the start it
# lies on the bench cap down, its handle along STROKE_A0, slid TOOL_SIDE sideways (along the tool's y) from the axis.
STROKE_A0 = 280.0
WRENCH_STROKE = 30.0
TOOL_SIDE = 0.13
TOOL_TURN0 = math.radians(STROKE_A0 - 180.0)  # the handle runs along the tool's -x
TOOL_X0, TOOL_Y0 = 0.0, TOOL_SIDE
TOOL_HOLD_D = 0.05  # m: a gripper holds the wrench with its tool point within 5 cm of the handle's axis
HOLD_OPEN = 0.6  # and its fingers under 60 % open (an open hand resting near the wrench is not holding it)


class ToolModel:
    """What the wrench is doing this trial: seated on the can or not, the pawl's reference (the wrench's turn at which
    the pawl bears, minus the filter's unscrew angle), and the filter angle the wrench drives it to (read by the
    thread's drive)."""

    def __init__(self):
        self.episode = kit.episode()
        self.coupled = False
        self.ref = 0.0
        self.drive_u = None  # the unscrew angle (rad) the wrench has turned the filter to this step
        self.lock_xy = None
        self.lock_turn = None
        self.turned = 0.0  # rad of filter turning (tightening) done by the wrench this trial


_TM = ToolModel()


def tool_model() -> ToolModel:
    global _TM
    if _TM.episode != kit.episode():
        _TM = ToolModel()
    return _TM


def _filter_u() -> float:
    return tk._track(KEY).u


def tool_step(J: dict) -> ToolModel:
    """Advance the wrench's model by one control step. J: the wrench's joints by link name ("t_lift", "t_turn", "t_x",
    "tool"). The filter's angle is read from the thread. Called by the lift's law in the sim (the first link), by the
    tests offline."""
    tm = tool_model()
    tm.drive_u = None  # the head's thread (earlier in the step: OBJECTS order) has applied the last step's
    lift, q, x, y = J["t_lift"], J["t_turn"], J["t_x"], J["tool"]
    off = math.hypot(x, y)
    seated = lift <= SEAT_TOL and off <= CENTER_TOL
    u = _filter_u()
    if seated and not tk._track(KEY).off:
        if not tm.coupled:
            tm.coupled, tm.ref = True, q - u
        if q > u + tm.ref:  # turned back counterclockwise: the ratchet freewheels and the pawl clicks on
            tm.ref = q - u
        pawl = u + tm.ref
        need = gasket_torque(-math.degrees(u))
        if K_TOOL * (pawl - q) > need:  # pushed past what the gasket holds: the filter turns with the wrench
            lo = math.radians(-THREAD.max_tight)
            new_u = max(q - tm.ref + need / K_TOOL, lo)
            if new_u < u:
                tm.turned += u - new_u
                tm.drive_u = new_u
    else:
        tm.coupled = False
    down = lift <= LIFT_TABLE + 0.005 or seated
    tm.lock_xy = (x, y) if down else None
    tm.lock_turn = q if lift <= LIFT_TABLE + 0.005 else None
    return tm


class ToolLiftLaw:
    """The wrench's height: its weight holds it down onto the can's top (off-center), down over the can (centered), or
    onto the bench (clear of the head). Calls the model."""

    sets_limits = True

    def drive(self, key, q, qd, J, mem):
        tool_step(J)
        off = math.hypot(J["t_x"], J["tool"])
        lo = 0.0 if off <= CENTER_TOL else (LIFT_FREE if off < CLEAR else LIFT_TABLE)
        mem["limits"] = (lo, LIFT_MAX)
        return q, 0.0, 0.0, 2.0


class ToolTurnLaw:
    """The wrench's turn about the filter's axis: on the can, the pawl's spring against the gasket (only clockwise:
    the other way it freewheels with the pawl's drag); lying on the bench, held; lifted, free."""

    sets_limits = True

    def drive(self, key, q, qd, J, mem):
        tm = tool_model()
        mem["limits"] = None
        if tm.coupled:
            pawl = _filter_u() + tm.ref
            if tm.drive_u is not None:
                pawl = tm.drive_u + tm.ref
            if q <= pawl:
                return pawl, K_TOOL, 0.0, D_TOOL
            return q, 0.0, (-PAWL_DRAG if qd > 0.0 else 0.0), D_TOOL
        if tm.lock_turn is not None:
            mem["limits"] = (tm.lock_turn - 0.004, tm.lock_turn + 0.004)
        return q, 0.0, 0.0, 0.3


class ToolSlideLaw:
    """The wrench's slides (in its turned frame): held while it is down (on the can, or lying on the bench)."""

    sets_limits = True

    def __init__(self, k, lower, upper):
        self.k, self.lower, self.upper = k, lower, upper

    def drive(self, key, q, qd, J, mem):
        tm = tool_model()
        mem["limits"] = None
        if tm.lock_xy is not None:
            at = tm.lock_xy[self.k]
            mem["limits"] = (max(self.lower, at - 0.0005), min(self.upper, at + 0.0005))
        elif J["t_lift"] < LIFT_FREE - 0.002 and abs(q) >= CLEAR:
            # carried low, clear of the head: it can't slide in over the head until lifted over the can's top
            mem["limits"] = (CLEAR, self.upper) if q > 0 else (self.lower, -CLEAR)
        return q, 0.0, 0.0, 3.0


CHROME = kit.Look((0.80, 0.81, 0.83), 0.2, metallic=0.95)
GRIP_LOOK = kit.Look((0.10, 0.10, 0.11), 0.7)  # the ratchet's black grip
CAP_LOOK = kit.Look((0.30, 0.31, 0.33), 0.4, metallic=0.8)  # the cap: dark gray steel


def _tool_parts():
    """The wrench's meshes and colliders in its own frame (origin on the cap's axis at its inside floor; the handle
    runs along -x)."""
    meshes = [kit.lathe([(CUP_R - 0.003, -CUP_DEPTH), (CUP_R, -CUP_DEPTH), (CUP_R, 0.004), (0.0, 0.004), (0.0, 0.0),
                         (CUP_R - 0.003, 0.0)], CAP_LOOK, n=28)]  # the cap's fluted cup (drawn plain)
    meshes.append(kit.cylinder((0, 0, 0.010), 0.0095, 0.012, CHROME))  # the drive adapter
    meshes.append(kit.cylinder((0, 0, (0.016 + HEAD_TOP) / 2), HEAD_R, HEAD_TOP - 0.016, CHROME))  # the ratchet head
    meshes.append(kit.cylinder((0.004, 0.0, HEAD_TOP + 0.001), 0.006, 0.003, CHROME))  # the direction lever
    hl = HANDLE_X[1] - HANDLE_X[0]
    mid = (HANDLE_X[0] + HANDLE_X[1]) / 2
    meshes.append(kit.box((mid, 0.0, HANDLE_Z), (hl, 0.016, 0.012), CHROME))
    meshes.append(kit.cylinder((HANDLE_X[0] + 0.075, 0.0, HANDLE_Z), HANDLE_R, 0.15, GRIP_LOOK, axis="x"))
    colliders = (kit.Collider((mid, 0.0, HANDLE_Z), size=(hl, 2 * HANDLE_R, 2 * HANDLE_R)),)
    return tuple(meshes), colliders


def _tool():
    pin = (0.0, 0.0, 0.0)
    reach = 0.30
    meshes, colliders = _tool_parts()
    lift = kit.Link("t_lift", "prismatic", "Z", pin, lower=LIFT_TABLE, upper=LIFT_MAX, start=LIFT_TABLE, mass=0.005,
                    damping=0.0, law=ToolLiftLaw())
    turn = kit.Link("t_turn", "revolute", "Z", pin, lower=-2 * math.pi, upper=2 * math.pi, start=TOOL_TURN0,
                    mass=0.005, damping=0.0, parent="t_lift", law=ToolTurnLaw())
    sx = kit.Link("t_x", "prismatic", "X", pin, lower=-reach, upper=reach, start=TOOL_X0, mass=0.005, damping=0.0,
                  parent="t_turn", law=ToolSlideLaw(0, -reach, reach))
    body = kit.Link("tool", "prismatic", "Y", pin, lower=-reach, upper=reach, start=TOOL_Y0, mass=TOOL_MASS,
                    meshes=meshes, colliders=colliders, damping=0.0, parent="t_x", law=ToolSlideLaw(1, -reach, reach))
    return kit.Mechanism(TOOL, pos=(HEAD_C[0], HEAD_C[1], TOP + SEAT + CAN_H), size=(1.3, 1.3, 0.25), mass=None,
                         links=(lift, turn, sx, body))


OBJECTS = [_head(), _tool()]
KEY = "head/filter"
TOOL_LINKS = ("t_lift", "t_turn", "t_x", "tool")


def tool_joints(objects=None) -> dict:
    """The wrench's joints now (its start pose if the simulator has not reported them)."""
    start = {"t_lift": LIFT_TABLE, "t_turn": TOOL_TURN0, "t_x": TOOL_X0, "tool": TOOL_Y0}
    out = {}
    for n in TOOL_LINKS:
        js = kit._JOINTS.get(f"{TOOL}/{n}")
        out[n] = js.pos if js is not None else start[n]
    return out


def tool_point(J, local):
    """A point given in the wrench's frame (x, y, z), in world coordinates, from its joints."""
    t = J["t_turn"]
    c, s_ = math.cos(t), math.sin(t)
    lx, ly = local[0] + J["t_x"], local[1] + J["tool"]
    return np.array([HEAD_C[0] + c * lx - s_ * ly, HEAD_C[1] + s_ * lx + c * ly,
                     TOP + SEAT + CAN_H + J["t_lift"] + local[2]])


def tool_seated(J) -> bool:
    return J["t_lift"] <= SEAT_TOL and math.hypot(J["t_x"], J["tool"]) <= CENTER_TOL


def tool_set_down(J) -> bool:
    """The wrench lying on the bench (off the filter, clear of the head)."""
    return J["t_lift"] <= LIFT_TABLE + 0.005


def tool_holders(objects, J) -> set:
    """Grippers holding the wrench: fingers under 60 % open with the tool point within 5 cm of the handle's axis."""
    a, b = tool_point(J, (HANDLE_X[0], 0.0, HANDLE_Z)), tool_point(J, (HANDLE_X[1], 0.0, HANDLE_Z))
    out = set()
    for g in GRIPPERS:
        if g not in objects or kit.opening(objects, g) >= HOLD_OPEN:
            continue
        p = np.asarray(objects[g].pos, float)
        u = float(np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0))
        if np.linalg.norm(p - (a + u * (b - a))) < TOOL_HOLD_D:
            out.add(g)
    return out


_orig_drive_thread = tk._drive_thread


def _drive_thread(base, spec):
    """The thread's own drive, then the wrench's: where the wrench turned the filter this step, the filter is put there
    (the cap on the flutes carries it round)."""
    _orig_drive_thread(base, spec)
    tm = tool_model()
    if tm.drive_u is None or not isinstance(spec, tk.Threaded):
        return
    art = base.scene[spec.name]
    jn = list(art.joint_names)
    jc = jn.index(f"{spec.cap.name}_joint")
    t = tk._track(tk._key(spec))
    np_ = lambda v: getattr(v, "torch", v).detach().cpu().numpy().astype(np.float64)  # noqa: E731
    qc = float(np_(art.data.joint_pos)[0][jc])
    new = qc + spec.thread.hand * (tm.drive_u - t.u)
    kit._put_joint(art, jc, new)
    kit._set_drive(art, jc, target=new, stiffness=spec.thread.gasket_hold, effort=0.0)
    tm.drive_u = None


tk._drive_thread = _drive_thread


def _hold_thread_while_settling():
    """Hold the filter on its thread from the very first physics step. The simulator settles the scene for 0.5 s after
    each reset, before the first camera frame, and twistkit sets the thread's drives only from the first control step
    on; until then the filter's slide has no drive and the filter dropped about 5 mm onto the slide's stop, then was
    lifted back to its start height in the first second of the video. This applies the same drives before every
    settling step, so the first frame shows the filter where it starts and nothing moves before a hand touches it."""
    ie = sys.modules.get("yam_arena.inspect_embodiment")
    cls = getattr(ie, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_oil_filter_hold", False):
        return
    base_reset = cls.reset

    def reset(self, *a, **k):
        self._oil_filter_fresh = True  # a new trial: the thread's record starts over at the start angle

        def step(action, _orig=None):
            env = self._env
            if getattr(self, "_oil_filter_fresh", False):
                tk._TRACK.pop(KEY, None)
                self._oil_filter_fresh = False
            spec = kit._MECHS.get(KEY.split("/")[0])
            if spec is not None:
                tk._drive_thread(env.unwrapped, spec)
            return env._oil_filter_step(action)

        orig_ensure = self._ensure_sim

        def ensure(goal):
            orig_ensure(goal)
            env = self._env
            if env is not None and not hasattr(env, "_oil_filter_step"):
                env._oil_filter_step = env.step
                env.step = step

        self._ensure_sim = ensure
        try:
            return base_reset(self, *a, **k)
        finally:
            del self._ensure_sim
            if self._env is not None and hasattr(self._env, "_oil_filter_step"):
                self._env.step = self._env._oil_filter_step  # control steps run as before
                del self._env._oil_filter_step

    cls.reset = reset
    cls._oil_filter_hold = True


_hold_thread_while_settling()


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.peak = -1e9  # the most the filter has been tightened past gasket contact this trial (degrees)
        self.high = 0.0  # the furthest past contact reached so far (degrees, from 0): only new ground counts as turning
        self.prev_holders = set()  # the grippers holding the filter at the last step
        self.last_quat = {}  # each gripper's orientation at the last step
        self.budget = {g: 0.0 for g in GRIPPERS}  # each holding gripper's wrist roll (tightening way) not yet matched
        #                                          by filter turning
        self.unheld = 0.0  # new ground past contact gained without a holding gripper's wrist turning it (degrees)
        self.tool_turned = 0.0  # the wrench's turning of the filter counted so far (degrees)
        self.tool_pool = 0.0  # turning the wrench did while a hand held it, not yet matched to new ground (degrees)


GRIPPERS = ("left_gripper", "right_gripper")
_H = _History()


def _holders(objects):
    """Every gripper whose fingers are closed on the can right now (twistkit's own held test, one gripper at a time,
    so a second hand on the can is seen too)."""
    return {g for g in GRIPPERS if g in objects and tk._held_now({KEY: objects[KEY], g: objects[g]}, KEY) == g}


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    past = tk.past_seat_deg(objects, KEY)
    _H.peak = max(_H.peak, past)
    # Who turned it: each gripper holding the filter at either end of this step lends its own wrist roll (clockwise
    # seen from above) to its own budget (at most CARRY ahead of the filter), and new ground past contact is credited
    # from the largest holding budget; whatever no holding wrist turned (a swipe, a knock, the other arm, a hand merely
    # around the can) is unheld. A second hand near the can, or on it helping, does not void the holder's turning: the
    # turning is credited only up to what a holding wrist actually rolled, so a swipe still shows as unheld.
    now = _holders(objects)
    holders = now | _H.prev_holders
    quats = {g: np.asarray(objects[g].quat_xyzw, float) for g in GRIPPERS if g in objects}
    for g in GRIPPERS:
        if g in holders and g in quats and g in _H.last_quat:
            _H.budget[g] = min(_H.budget[g] - _yaw_change(_H.last_quat[g], quats[g]), CARRY)
        elif g not in holders:
            _H.budget[g] = 0.0
    # The wrench: turning it did on the filter this step counts as a hand's when a hand holds the wrench.
    tm = tool_model()
    tool_now = math.degrees(tm.turned)
    if tool_now > _H.tool_turned and tool_holders(objects, tool_joints(objects)):
        _H.tool_pool += tool_now - _H.tool_turned
    _H.tool_turned = tool_now
    gain = max(past - _H.high, 0.0)
    if gain > 0:
        by_tool = min(gain, _H.tool_pool)
        _H.tool_pool -= by_tool
        credit = min(gain - by_tool, max([_H.budget[g] for g in holders] + [0.0]))
        for g in holders:  # every hand on the can turned with it: the turning is used up from each budget
            _H.budget[g] = max(_H.budget[g] - credit, 0.0)
        _H.unheld += gain - by_tool - credit
        _H.high = past
    _H.prev_holders = now
    _H.last_quat = quats


def _yaw_change(q0, q1):
    """The rotation from orientation q0 to q1 (x, y, z, w) about the world vertical, in degrees (counterclockwise seen
    from above positive): the twist part of q1 * q0^-1."""
    x0, y0, z0, w0 = -q0[0], -q0[1], -q0[2], q0[3]
    x1, y1, z1, w1 = q1
    z = w1 * z0 + x1 * y0 - y1 * x0 + z1 * w0
    w = w1 * w0 - x1 * x0 - y1 * y0 - z1 * z0
    a = math.degrees(2 * math.atan2(z, w)) if (z or w) else 0.0
    return (a + 180.0) % 360.0 - 180.0


def _hand_on_filter(objects):
    """A gripper still on the filter: closed on the can (twistkit.held), or its palm or fingers touching the can."""
    return bool(tk.held(objects, KEY) or gk.touching(objects, KEY, FILTER_SHAPE))


def _score(objects):
    past = tk.past_seat_deg(objects, KEY)  # degrees past gasket contact now (negative: not yet touching)
    # irreversible: unscrewed off the nipple at any point, or tightened more than 1.25 turns past contact (can crush the gasket)
    if tk.came_off(objects, KEY) or _H.peak > CRUSH:
        return 0.0
    if past < LOOSE:
        return 0.0  # short of contact, or under a quarter turn past it: can leak or work loose
    if _H.unheld > GRIP_SLACK:
        return 0.0  # spun past contact by a knock, a swipe or the other arm, not turned by a hand holding it
    if past < SNUG:
        s = 0.25  # a quarter to a half turn past contact: partly tightened
    elif past < GOOD_LO or _H.peak > GOOD_HI:
        s = 0.5  # half to two thirds of a turn, or past a full turn (now or at any point) but not crushed
    else:
        s = 1.0  # 240 to 360 degrees past contact, never beyond 360
    if _hand_on_filter(objects):
        return min(s, 0.5)  # not let go
    J = tool_joints(objects)
    if not tool_set_down(J) or tool_holders(objects, J):
        return min(s, 0.5)  # the wrench still on the filter, or still in a hand: the job is not finished
    return s


def success(objects):
    """Full score: the filter 240 to 360 degrees past gasket contact (never past 360), turned there by a hand holding
    it or by the wrench while a hand held the wrench, let go, and the wrench laid on the bench with no hand holding it
    (where the arms end is not scored). Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """Angles are degrees past gasket contact (the seat); the filter starts at -270. 0 if the filter came off its
    thread at any point, or was ever tightened more than 450 degrees past contact (enough to crush the gasket: irreversible).
    0 if it ends under 90 degrees past contact, or if more than 45 degrees of the new ground it gained past contact was
    not turned by a holding hand: each step, a gripper holding the filter at either end of the step (twistkit.held:
    fingers closed on the can, tool point on its axis, anywhere along the can) credits only as much as its own wrist
    rolled clockwise (at most 30 degrees of it carried ahead of the filter; with both hands on the can, the larger); the
    rest (a knock, a swipe, the other arm, a hand loosely around the can) is unheld, and turning
    back and forth over the same angles counts only once. Otherwise 0.25 for 90 to 180 degrees; 0.5 for 180 to 240,
    for 360 to 450, or if it was ever past 360; 1.0 for 240 to 360 and never past 360. Any score is capped at 0.5 while
    a hand is still on the filter (closed on the can, or its palm or fingers touching it: the gripper's own shape at
    its present opening within 2 mm of the can; a hand hovering near it is not on it), and while the wrench is not
    laid on the bench or a hand holds it (fingers under 60 % open, tool point within 5 cm of its handle). Turning the
    wrench did on the filter counts as a holding hand's only at steps when a hand holds the wrench. Where the arms end is
    not scored, and a second hand near or on the can does not void the holder's turning. Possible scores: 0, 0.25,
    0.5, 1."""
    return _score(objects)


# --- Demo ----------------------------------------------------------------------------------------
GRIP_Z = TOP + SEAT + CAN_H - 0.015  # the tool point 1.5 cm below the can's top (pad tips 2.8 cm down its side)
TURN = 305.0  # by hand: 270 to gasket contact, then 20 past it (the gasket then holds 1.3 N*m), plus 15 for slip
STROKE = 120.0
R_GRIP = {"right": 0.55, "left": 0.22}  # where each hand holds the wrench's handle, from the cap's axis (m)
SHARE = {"right": 0.67, "left": 0.33}  # each hand's share of the torque: alike fractions of what each arm can push
#                                        there (about 13 N right at 0.55 m, 16 N left at 0.22 m, at rated torque)
MARGIN = 1.0  # N*m pushed past what the gasket holds, so the filter keeps turning through the stroke
TARGET_PAST = 300.0  # degrees past contact the demo aims for (the middle of 240 to 360)
PAD_BELOW = 0.006  # the pad tips this far below the handle's axis: the fingers close round it
CARRY_LIFT = 0.021  # the wrench carried with its cap 6 mm over the can's top (it clears it at 1.7 cm of lift)


def _wrench_J(a_deg, lift=0.0, side=0.0):
    """The wrench's joints with its handle along `a_deg` (from the filter's axis, degrees from +x), lifted `lift` and
    slid `side` along its y."""
    return {"t_lift": lift, "t_turn": math.radians(a_deg - 180.0), "t_x": 0.0, "tool": side}


_GIVE = {}


def _give(side, tcp, yaw):
    """The arm's give at its pad tips (m/N, 3x3, world) with the tool point at `tcp` gripping straight down at `yaw`:
    J K^-1 J^T with the YAM's servo stiffness (80 N*m/rad shoulder joints, 10 wrist)."""
    key = (side, round(yaw, 3), *(round(float(v), 3) for v in tcp))
    if key not in _GIVE:
        import yam_arena.demo as yd
        from yam_arena.constants import PAD_TIP_PAST_TCP, SHOULDER, WRIST
        from yam_arena.station import BASES
        base = np.asarray(BASES[side], float)
        q, _, _ = yd.ik.solve(np.asarray(tcp, float), yd.ik.matrix_to_quat_xyzw(yd.grip_rotation(yaw)), q_init=None,
                              base_pos=base)

        def tip(q):
            p, rm = yd.ik.tcp_pose(q, base)
            return np.asarray(p) + PAD_TIP_PAST_TCP * np.asarray(rm)[:, 2]

        jac = np.zeros((3, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            jac[:, i] = (tip(q + dq) - tip(q - dq)) / 2e-6
        kinv = np.diag([1 / SHOULDER["stiffness"]] * 3 + [1 / WRIST["stiffness"]] * 3)
        _GIVE[key] = jac @ kinv @ jac.T
    return _GIVE[key]


YAW_TURN = {"left": 180.0, "right": 0.0}  # each wrist's yaw on the handle: a - 180, plus this (0 or 180: the same grip turned round)


def _yaw(a_deg, side="right"):
    """The wrist's yaw for fingers closing across a handle along `a_deg`: a - 180 (+ YAW_TURN), in -180..180."""
    return math.radians((a_deg - 180.0 + YAW_TURN[side] + 180.0) % 360.0 - 180.0)


def _hand(side, a_deg, lift=0.0, slide=0.0, force=(0.0, 0.0, 0.0), up=0.0):
    """A hand's tool point on the wrench's handle (wrench at a_deg, lift, slide), aimed past the pads by the arm's give
    so that the pads push `force` (N, world) on the handle and hold `up` N of its weight."""
    from yam_arena.constants import PAD_TIP_PAST_TCP
    p = tool_point(_wrench_J(a_deg, lift, slide), (-R_GRIP[side], 0.0, HANDLE_Z))
    tcp = p + np.array([0.0, 0.0, PAD_TIP_PAST_TCP - PAD_BELOW])
    yaw = _yaw(a_deg, side)
    f = np.asarray(force, float) + np.array([0.0, 0.0, up])
    return tuple(float(v) for v in tcp + _give(side, tuple(tcp), yaw) @ f), yaw


def _push(side, a_deg, torque):
    """The pads' push (N, world) for this hand's share of `torque` (N*m, clockwise seen from above) on a handle along
    a_deg."""
    a = math.radians(a_deg)
    f = SHARE[side] * torque / R_GRIP[side]
    return (f * math.sin(a), -f * math.cos(a), 0.0)


WEIGHT_SHARE = {"right": 0.30, "left": 0.70}  # the wrench's 9.8 N between the hands, by where they hold it


def _pair(dur, label, a_deg, lift=0.0, slide=0.0, torque=0.0, finger=None, carry=False):
    """One waypoint for each hand on the handle."""
    from yam_arena.constants import FINGER_CLOSED
    out = {}
    for side in ("left", "right"):
        up = WEIGHT_SHARE[side] * TOOL_MASS * 9.81 if carry else 0.0
        pos, yaw = _hand(side, a_deg, lift, slide, _push(side, a_deg, torque), up)
        out[side] = Waypoint(dur, pos, FINGER_CLOSED if finger is None else finger, label, yaw)
    return out


def wrench_plan():
    """The wrench's work as (label, handle angle, lift, slide, torque, carry, fingers, seconds) steps, and the filter's
    angle past contact it predicts at the end (the pawl takes up the wrench's flex at each stroke's start)."""
    from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN
    a0, side = STROKE_A0, TOOL_SIDE
    plan = [("over the handle", a0, LIFT_TABLE, side, 0.0, False, FINGER_OPEN, 2.0),
            ("onto the handle", a0, LIFT_TABLE, side, 0.0, False, FINGER_OPEN, 1.0),
            ("grip the handle", a0, LIFT_TABLE, side, 0.0, False, FINGER_CLOSED, 0.8),
            ("lift the wrench", a0, CARRY_LIFT, side, 0.0, True, FINGER_CLOSED, 1.6),
            ("over the filter", a0, CARRY_LIFT, 0.0, 0.0, True, FINGER_CLOSED, 2.0),
            ("cap onto the can", a0, 0.004, 0.0, 0.0, True, FINGER_CLOSED, 1.2),
            ("seated", a0, 0.0, 0.0, 0.0, False, FINGER_CLOSED, 0.6)]
    past = TURN - 270.0 - 15.0
    n = 0
    a_end = a0
    while past < TARGET_PAST - 5.0 and n < 12:
        n += 1
        take_up = math.degrees(gasket_torque(past) / K_TOOL)
        length = min(WRENCH_STROKE, TARGET_PAST - past + take_up)
        steps = 5
        for k in range(1, steps + 1):
            a = a0 - length * k / steps
            p = past + max(length * k / steps - take_up, 0.0)
            plan.append((f"stroke {n}", a, 0.0, 0.0, gasket_torque(p) + MARGIN, False, FINGER_CLOSED, 0.8))
        past += length - take_up
        a_end = a0 - length
        plan.append((f"stroke {n}: ease off", a_end, 0.0, 0.0, 0.0, False, FINGER_CLOSED, 0.6))
        if past < TARGET_PAST - 5.0:
            plan.append((f"stroke {n}: ratchet back", a0, 0.0, 0.0, 0.0, False, FINGER_CLOSED, 1.6))
    plan += [("lift the wrench off", a_end, CARRY_LIFT, 0.0, 0.0, True, FINGER_CLOSED, 1.2),
             ("aside", a_end, CARRY_LIFT, side, 0.0, True, FINGER_CLOSED, 2.0),
             ("lay it on the bench", a_end, LIFT_TABLE + 0.003, side, 0.0, False, FINGER_CLOSED, 1.6),
             ("let go", a_end, LIFT_TABLE + 0.003, side, 0.0, False, FINGER_OPEN, 0.8),
             ("hands up", a_end, LIFT_TABLE + 0.06, side, 0.0, False, FINGER_OPEN, 1.0)]
    return plan, past


def demo():
    """Left arm: open over the filter, descend around the top of the can, and spin it on clockwise (seen from above) in
    strokes of 120 degrees, three quarters of a turn to gasket contact and 20 degrees past it, where the gasket starts
    to hold; it lets go and lifts clear. The right arm waits at home. Then both hands take the wrench's handle (right
    near its end, 0.55 m out; left 0.22 m out), lift it, carry the cap over the filter and lower it onto the can, and
    swing the handle clockwise in 30-degree strokes, ratcheting back between them, pushing with what the gasket takes
    plus 1 N*m (up to 19 N*m: about 23 N right and 29 N left), each hand aimed past the handle by its arm's give. When the plan reaches 300 degrees past contact they ease off, lift the wrench off, set it on the bench beside
    the head, let go and go home."""
    from yam_arena.constants import FINGER_OPEN
    left = tk.twist_waypoints(HEAD_C, GRIP_Z, -TURN, side="left", stroke=STROKE, pitch=THREAD.pitch,
                              label="spin the filter on")
    t_spin = sum(w.duration for w in left)
    right = [kit.Home(t_spin, label="wait at home")]
    plan, _ = wrench_plan()
    for label, a, lift, slide, torque, carry, finger, dur in plan:
        pr = _pair(dur, label, a, lift, slide, torque, finger, carry)
        if label == "over the handle":  # come down from above the handle
            for side in ("left", "right"):
                w = pr[side]
                pr[side] = Waypoint(w.duration, (w.pos[0], w.pos[1], w.pos[2] + 0.06), FINGER_OPEN, w.label, w.yaw)
        left.append(pr["left"])
        right.append(pr["right"])
    left += [kit.Home(2.5), kit.Home(2.0, label="hold at home")]
    right += [kit.Home(2.5), kit.Home(2.0, label="hold at home")]
    return {"left": left, "right": right}
