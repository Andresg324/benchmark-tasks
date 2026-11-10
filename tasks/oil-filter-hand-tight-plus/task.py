"""A new spin-on oil filter has been started on the threaded nipple of a remote oil filter mount (a filter head) bolted to
the bench, filter pointing up, and spun down by hand until it is three quarters of a turn short of its gasket touching
the head. Fitting it is what the filter maker prints on the can and every lube technician learns: spin it on until the
gasket contacts the head, then tighten it a further three quarters of a turn by hand, no tool. Left at contact or a
little past it, the filter works loose under engine vibration and leaks; cranked much past it, the gasket is crushed,
can roll out of its groove and blow out under oil pressure, and the filter cannot be removed by hand at the next change.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import twistkit as tk  # noqa: E402  (threaded parts on top of yamkit; nothing on disk changes)
import yamkit as kit  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Fit the new oil filter."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the workspace is clear for
# the next person or job. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# The filter head (fixed, bolted to the bench): a die-cast aluminum remote oil filter mount, its top face the machined
# gasket land the filter seals on. Heights are above the table.
HEAD_C = (0.30, -0.12)  # the filter's axis
HEAD_X, HEAD_Y, HEAD_H = 0.075, 0.090, 0.034  # the block
LAND_R, LAND_H = 0.036, 0.006  # the round gasket land on top of the block
SEAT = HEAD_H + LAND_H  # 4.0 cm: where the filter's gasket meets the head

# The filter: a small spin-on oil filter (motorcycle and small-engine size), true size. Heights are above its gasket face.
CAN_R, CAN_H = 0.033, 0.070  # 6.6 cm across, 7.0 cm tall
FILTER_MASS = 0.25
# M20 x 1.5 thread: 1.5 mm per turn. The filter starts three quarters of a turn (270 degrees) short of gasket contact,
# about five turns onto the nipple, so it comes off only if it is unscrewed five more turns. Before contact it spins
# freely (a small hold where it is left). Past contact the squeezed gasket adds a turning friction that grows with the
# angle, 0.1 N*m per radian (0.47 N*m at three quarters of a turn, 0.63 at a full turn), capped at 1 N*m so the arm can
# still finish: a stiff hold where it was left that slips at that torque, so it opposes turning either way and never
# turns the filter by itself (stick-slip friction, not a spring). The hard stop at 540 degrees past
# contact (a turn and a half) is the gasket squeezed flat. The can travels down at most 0.5 mm past contact (the gasket
# compressing), so the gasket does not sink into the land.
THREAD = tk.Thread(pitch=0.0015, turns=5.0, start=-270.0, max_tight=540.0, resist=0.03, tight_resist=0.05,
                   gasket=0.1, gasket_max=1.0, seat_travel=0.0005)

# Scoring: degrees past gasket contact (the seat). Filter makers print "three quarters of a turn" or "three quarters to
# one turn" after gasket contact, so full credit spans both readings.
LOOSE = 90.0  # under a quarter turn past contact (a third of the least any maker asks): not fitted: 0
SNUG = 180.0  # 90 to 180 (a quarter to a half turn): partly tightened, well short of the makers' figure: 0.25
GOOD_LO, GOOD_HI = 240.0, 360.0  # three quarters of a turn (less 30 degrees for judging it by hand) to one full turn: 1.0
CRUSH = 450.0  # past a full turn (the most any maker allows): 0.5; past 1.25 turns at any point: gasket crushed, 0
GRIP_SLACK = 45.0  # at most 45 degrees (an eighth of a turn) of the turning past contact may be done without a hand
# Holding the filter (twistkit.held, with this task's span): fingers closed on the can (opening from 15 % of full
# closed past its 6.6 cm to 3 % open past it, 2.8 mm of clearance in all, about the pads' compliance) with the tool point
# within 1.5 cm of the can's axis, anywhere from 1 cm above the gasket face to 1.5 cm above the top: the pad tips sit
# about 1.3 cm below the tool point, so that is every grasp height with the pads on the can, from its rim to its base.
GRIP_ALONG = (0.010, CAN_H + 0.015)
CARRY = 30.0  # a holding wrist's roll not yet matched by the filter is kept at most 30 degrees (two 15-degree control
# steps): the filter catching up a step or two late as the pads take up slack; more is the pads slipping on the can
HOME_TOL = 0.03  # a gripper is home within 3 cm of where its tool point started: the arm's ~1.6 cm tracking lag plus margin
HOME_STILL = 0.001  # and still: its tool point moved under 1 mm in the last step (2 cm/s), settled rather than passing by
HOLD_OPEN = 0.85  # a hand is still on the filter with its fingers under 85 % open (pads within 7 mm of the can each side)
HOLD_MARGIN = 0.015  # and its tool point within 1.5 cm of the can's outside (from the gasket to 1.5 cm above the top)

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


OBJECTS = [_head()]
KEY = "head/filter"


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.peak = -1e9  # the most the filter has been tightened past gasket contact this trial (degrees)
        self.high = 0.0  # the furthest past contact reached so far (degrees, from 0): only new ground counts as turning
        self.prev_holder = None  # the gripper holding the filter at the last step
        self.last_quat = {}  # each gripper's orientation at the last step
        self.budget = 0.0  # wrist roll (tightening way) of the holding gripper not yet matched by filter turning
        self.unheld = 0.0  # new ground past contact gained without a holding gripper's wrist turning it (degrees)
        self.home = None
        self.last_grip = None
        self.grips_still = False


_H = _History()


OTHER_CLEAR = 0.04  # m: the other gripper's tool point must stay this far outside the can's wall (and below its top
# + 2 cm) for the holder's roll to be credited: with a second hand on the can, the holder's wrist roll no longer shows
# who turned it


def _other_hand_on(objects, holder):
    """The non-holding gripper's tool point within 4 cm of the can's wall, from the head's top to 2 cm over the can."""
    other = "left_gripper" if holder == "right_gripper" else "right_gripper"
    if other not in objects:
        return False
    p = np.asarray(objects[other].pos, float)
    across = math.hypot(p[0] - HEAD_C[0], p[1] - HEAD_C[1])
    return across < CAN_R + OTHER_CLEAR and TOP + HEAD_H - 0.01 < p[2] < TOP + HEAD_H + CAN_H + 0.10


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    past = tk.past_seat_deg(objects, KEY)
    _H.peak = max(_H.peak, past)
    # Who turned it: a gripper holding the filter at either end of this step lends its own wrist roll (clockwise seen
    # from above) to a budget (at most CARRY ahead of the filter), and new ground past contact is credited from that
    # budget; whatever the holding wrist did not turn (a swipe, the other arm, a hand merely around the can) is unheld.
    holder = tk.holder(objects, KEY) or _H.prev_holder
    quats = {g: np.asarray(objects[g].quat_xyzw, float) for g in ("left_gripper", "right_gripper") if g in objects}
    if holder is not None and holder in quats and holder in _H.last_quat:
        _H.budget = min(_H.budget - _yaw_change(_H.last_quat[holder], quats[holder]), CARRY)
    elif holder is None:
        _H.budget = 0.0
    gain = max(past - _H.high, 0.0)
    if gain > 0:
        credit = min(gain, max(_H.budget, 0.0)) if holder is not None and not _other_hand_on(objects, holder) else 0.0
        _H.budget -= credit
        _H.unheld += gain - credit
        _H.high = past
    _H.prev_holder = tk.holder(objects, KEY)
    _H.last_quat = quats
    grips = {g: np.asarray(objects[g].pos, float) for g in ("left_gripper", "right_gripper") if g in objects}
    if _H.home is None:
        _H.home = grips
    _H.grips_still = _H.last_grip is not None and all(
        g in _H.last_grip and np.linalg.norm(p - _H.last_grip[g]) < HOME_STILL for g, p in grips.items())
    _H.last_grip = grips


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
    """A gripper with its fingers under 85 % open and its tool point within 1.5 cm of the can's outside (from the
    gasket face to 1.5 cm above the top): still holding or squeezing the filter."""
    if tk.held(objects, KEY):
        return True
    axis = np.array([HEAD_C[0], HEAD_C[1], TOP + SEAT])
    for g in ("left_gripper", "right_gripper"):
        if g not in objects or kit.opening(objects, g) >= HOLD_OPEN:
            continue
        d = np.asarray(objects[g].pos, float) - axis
        if np.hypot(d[0], d[1]) < CAN_R + HOLD_MARGIN and -0.005 < d[2] < CAN_H + HOLD_MARGIN:
            return True
    return False


def _arms_home(objects):
    """Each gripper's tool point within 3 cm of where it was at the trial's first scored step, and still."""
    if not _H.home or not _H.grips_still:
        return False
    return all(np.linalg.norm(np.asarray(objects[g].pos, float) - p) < HOME_TOL for g, p in _H.home.items()
               if g in objects)


def _score(objects):
    past = tk.past_seat_deg(objects, KEY)  # degrees past gasket contact now (negative: not yet touching)
    # irreversible: unscrewed off the nipple at any point, or tightened more than 1.25 turns past contact (crushed)
    if tk.came_off(objects, KEY) or _H.peak > CRUSH:
        return 0.0
    if past < LOOSE:
        return 0.0  # short of contact, or under a quarter turn past it: leaks or works loose
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
    if s == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return s


def success(objects):
    """Full score: the filter 240 to 360 degrees past gasket contact (never past 360), turned there by a hand holding
    it, let go, and (with REQUIRE_ARMS_HOME) both arms home and still. Called first each step, so it also records the
    history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """Angles are degrees past gasket contact (the seat); the filter starts at -270. 0 if the filter came off its
    thread at any point, or was ever tightened more than 450 degrees past contact (the gasket crushed: irreversible).
    0 if it ends under 90 degrees past contact, or if more than 45 degrees of the new ground it gained past contact was
    not turned by a holding hand: each step, a gripper holding the filter at either end of the step (twistkit.held:
    fingers closed on the can, tool point on its axis, anywhere along the can) credits only as much as its own wrist
    rolled clockwise (at most 30 degrees of it carried ahead of the filter); the rest (a knock, a swipe, the other arm, a hand loosely around the can) is unheld, and turning
    back and forth over the same angles counts only once. Otherwise 0.25 for 90 to 180 degrees; 0.5 for 180 to 240,
    for 360 to 450, or if it was ever past 360; 1.0 for 240 to 360 and never past 360. Any score is capped at 0.5 while
    a hand is still on the filter (fingers under 85 % open, tool point within 1.5 cm of the can). Full credit needs
    both arms home and still (0.9 if not, with REQUIRE_ARMS_HOME). Possible scores: 0, 0.25, 0.5, 0.9, 1."""
    return _score(objects)


# --- Demo ----------------------------------------------------------------------------------------
GRIP_Z = TOP + SEAT + CAN_H - 0.015  # the tool point 1.5 cm below the can's top (pad tips 2.8 cm down its side)
TURN = 585.0  # 270 to gasket contact, then 300 past it (the middle of 240 to 360), plus 15 for the grip slipping
STROKE = 120.0


def demo():
    """Right arm: open over the filter, descend around the top of the can, and spin it on clockwise (seen from above)
    in strokes of 120 degrees: close, roll the wrist in 15-degree steps (sinking with the thread), open, roll back,
    close again. Four strokes of 120 degrees and one of 105 roll the wrist 585 degrees: three quarters of a turn to gasket
    contact, 300 degrees past it (the middle of the full-credit range), and 15 degrees for the pads slipping on the can.
    It lets go, lifts clear and goes home. The left arm stays home."""
    right = tk.twist_waypoints(HEAD_C, GRIP_Z, -TURN, side="right", stroke=STROKE, pitch=THREAD.pitch,
                               label="spin the filter on")
    right += [kit.Home(2.5), kit.Home(2.0, label="hold at home")]
    total = sum(w.duration for w in right)
    left = [kit.Home(total, label="stay home")]
    return {"left": left, "right": right}
