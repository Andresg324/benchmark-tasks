"""Offline checks of the oil filter task: the thread's model (it catches only started square and pressed on, a crooked
start cross-threads, the gasket's torque), the ratchet's pawl, and the scorer on step-by-step histories of the filter,
the wrench and the two grippers, with near misses and cheats. No simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, fk = m.kit, m.fk
TH = m.THREAD
bad = 0


def case(name, got, want):
    global bad
    ok = (abs(got - want) < 1e-9) if isinstance(want, float) else got == want
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got} (want {want})", name)


# -- the thread ------------------------------------------------------------------------------------------------------
def start(tilt_deg=0.0, lat=0.0, out=None, turn_deg=60.0, step_deg=5.0):
    """A filter pressed against the threads' crests (or `out` from the land) and turned `turn_deg` clockwise."""
    f = fk.FilterModel(0, TH)
    J = dict(f_up=lat, f_side=0.0, f_out=TH.d0 + 0.0003 if out is None else out, f_pitch=math.radians(tilt_deg),
             f_roll=0.0, filter=0.0)
    for _ in range(int(round(turn_deg / step_deg))):
        J["filter"] -= math.radians(step_deg)
        cmd = f.step(J)
        if f.state == fk.ENGAGED:
            J["f_out"] = cmd["f_out"][0]
            J["f_pitch"] = J["f_roll"] = J["f_up"] = J["f_side"] = 0.0
    return f, J


f, _ = start(0.0)
case("started square: the thread catches", f.state, fk.ENGAGED)
f, _ = start(3.5)
case("3.5 degrees off: still catches", f.state, fk.ENGAGED)
f, _ = start(5.0)
case("5 degrees off and turned: cross-threaded", f.state, fk.CROSSED)
f, _ = start(9.0)
case("9 degrees off: not on the nipple, turning does nothing", f.state, fk.FREE)
f, _ = start(0.0, lat=0.004)
case("4 mm off the axis: the nipple's chamfer guides it on, the thread catches", f.state, fk.ENGAGED)
f, _ = start(0.0, lat=0.008)
case("8 mm off the axis: not on the nipple", f.state, fk.FREE)
f, _ = start(0.0, out=TH.d0 + 0.006)
case("held 6 mm short of the nipple and turned: nothing", f.state, fk.FREE)
f, J = start(0.0, turn_deg=10.0)
case("turned only 10 degrees against the crests: not caught yet", f.state, fk.FREE)
f, J = start(0.0, turn_deg=3 * 360.0 + 60.0)
case("three turns and 60 degrees on (the first 30 start the thread): 30 past contact", round(math.degrees(f.past), 3),
     30.0)
cmd = f.step(J)
case("its axial target 30 past contact: 1/12 turn x 1.5 mm into the gasket", round(cmd["f_out"][0], 6), -0.000125)
case("gasket torque at 270 past contact (N*m)", round(TH.torque(math.radians(270.0)), 3), 18.0)
case("before contact it runs on at 0.3 N*m", TH.torque(-0.5), 0.3)
case("the spin's drive is capped at the gasket's torque", round(cmd["filter"][4], 3), round(TH.torque(f.past), 3))
f, J = start(0.0, turn_deg=90.0)
for _ in range(30):
    J["filter"] += math.radians(10.0)
    f.step(J)
case("unscrewed past where it caught: loose again, flagged", (f.state, f.came_off), (fk.FREE, True))

# -- the ratchet ------------------------------------------------------------------------------------------------------
r = fk.RatchetModel(0)
case("the pawl starts at the cap's position", round(r.step(0.0, 0.0)[4][1], 4), 0.0001)
lim1 = r.step(-math.radians(10.0), -0.5)[4][1]
case("swung back 10 degrees: the pawl drops 3 clicks", (round(math.degrees(lim1 - 1e-4), 3), r.clicks), (-9.0, 3))
case("freewheeling drags at 0.2 N*m", r.step(-0.2, -0.5)[2], 0.2)


# -- the scorer -------------------------------------------------------------------------------------------------------
def st(p, R=np.eye(3)):
    p = np.array(p, float)
    return ObjectState(pos=p, quat_xyzw=np.array(m._m_to_quat(R)), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - .01, high=p + .01)


AX = np.array((m.X_LAND, m.Y_F, m.Z_F))
HOME_L, HOME_R = (0.20, 0.27, m.TOP + 0.22), (0.18, -0.30, m.TOP + 0.22)
W = 2 * m.CAN_R / (2 * 0.04695)  # the opening with the pads on the can


class Trial:
    """One trial: the filter on the thread at `past` degrees past contact (or free, or cross-threaded), the wrench on
    the bench or seated on the filter, the grippers as (position, opening, roll about +x in degrees)."""

    def __init__(self, past=0.0, state=fk.ENGAGED):
        kit._EPISODE["count"] += 1
        self.fm = m.fmodel()
        self.fm.state = state
        self.set(past)
        self.L, self.R = (HOME_L, 1.0, 0.0), (HOME_R, 1.0, 0.0)
        self.tool_on = False
        self.step()

    def set(self, past):
        self.fm.turned = self.fm.seat + math.radians(past)
        self.fm.peak_past = max(self.fm.peak_past, math.radians(past))

    def step(self, past=None, L=None, R=None, tool_on=None):
        if past is not None:
            self.set(past)
        self.L, self.R = L or self.L, R or self.R
        if tool_on is not None:
            self.tool_on = tool_on
        objs = {m.KEY: st(AX, m.R_BASE @ m._rot_z(-self.fm.turned))}
        if self.tool_on:
            tp = AX + np.array((-m.CAN_H - 0.0005, 0.0, 0.0))
            objs[m.TOOL], objs[m.CAP] = st(tp, m.R_MOUNT), st(tp, m.R_MOUNT)
        else:
            objs[m.TOOL], objs[m.CAP] = st(m.TOOL_P0, m.R_FLAT), st(m.TOOL_P0, m.R_FLAT)
        for g, (p, op, roll) in (("left_gripper", self.L), ("right_gripper", self.R)):
            objs[g] = st(p, m._rot_x(math.radians(roll)) @ m._rot_y(-90.0) @ m._td(0.0))
            kit._OPENING[g] = op
        self.ok = m.success(objs)
        self.sc = m.score(objs)
        return self.sc

    def home(self, n=5):
        for _ in range(n):
            self.step(L=(HOME_L, 1.0, 0.0), R=(HOME_R, 1.0, 0.0))
        return self.sc


ON_CAN = tuple(AX + np.array((-m.GRASP_UP, 0.0, 0.0)))  # the left tool point closed on the can


def hand_turn(t, to, rate=15.0, slip=0.0):
    """The left hand closed on the can rolls clockwise (+ about +x) `rate` a step, the filter following, less
    `slip`; it regrips every 150 degrees."""
    roll = 0.0
    while t.fm.past < math.radians(to) - 1e-9:
        roll += rate
        t.step(past=min(to, math.degrees(t.fm.past) + rate - slip), L=(ON_CAN, W, roll))
        if roll >= 150.0:
            t.step(L=(ON_CAN, 1.0, roll))
            roll = 0.0
            t.step(L=(ON_CAN, 1.0, roll))
    return t


GRIP = tuple(AX + np.array((-m.CAN_H - 0.04, -0.55, 0.0)))  # the right hand on the wrench's handle


def wrench_turn(t, to, held=True, rate=5.0):
    """The wrench seated on the filter turns it to `to`, a hand on its handle (or not)."""
    t.step(tool_on=True, R=(GRIP, 0.2, 0.0) if held else (HOME_R, 1.0, 0.0), L=(HOME_L, 1.0, 0.0))
    while t.fm.past < math.radians(to) - 1e-9:
        t.step(past=min(to, math.degrees(t.fm.past) + rate))
    return t


def finish(t):
    t.step(tool_on=False)
    return t.home()


t = hand_turn(Trial(0.0), 20.0)
wrench_turn(t, 275.0)
case("by hand to 20, the wrench to 275, the wrench off, let go: 1.0", finish(t), 1.0)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
case("the same, the wrench left on the filter: 0.5", t.step(R=(HOME_R, 1.0, 0.0)), 0.5)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
t.step(tool_on=False, R=(HOME_R, 1.0, 0.0))
case("the same, a hand left closed on the can: 0.5", t.step(L=(ON_CAN, W, 0.0)), 0.5)
case("left hand-tight (20 past): 0", finish(hand_turn(Trial(0.0), 20.0)), 0.0)
case("the wrench to 150: 0.25", finish(wrench_turn(hand_turn(Trial(0.0), 20.0), 150.0)), 0.25)
case("the wrench to 210: 0.5", finish(wrench_turn(hand_turn(Trial(0.0), 20.0), 210.0)), 0.5)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 380.0)
for p in range(375, 300, -5):
    t.step(past=float(p))  # backed off: the history keeps the peak
case("cranked to 380, backed off to 300: 0.5", finish(t), 0.5)
case("cranked past 450: 0", finish(wrench_turn(hand_turn(Trial(0.0), 20.0), 460.0)), 0.0)
case("turned by the wrench with no hand on it (knocked): 0",
     finish(wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0, held=False)), 0.0)
t = Trial(0.0)
for p in range(5, 280, 5):
    t.step(past=float(p))
case("turned with no hand on the can or the wrench: 0", finish(t), 0.0)
case("by hand alone (the hand's own roll) to 275: 1.0", finish(hand_turn(Trial(0.0), 275.0)), 1.0)
t = Trial(0.0)
for p in range(5, 280, 5):
    t.step(past=float(p), L=(ON_CAN, W, 0.0))  # closed on it, wrist not rolling
case("closed on the can, wrist still, filter turned by something else: 0", finish(t), 0.0)
case("cross-threaded: 0", finish(Trial(0.0, state=fk.CROSSED)), 0.0)
case("never threaded (left on the bench): 0", finish(Trial(0.0, state=fk.FREE)), 0.0)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
t.fm.came_off = True
case("unscrewed off and back on at some point: 0", finish(t), 0.0)

print("all pass" if not bad else f"{bad} FAILED")
