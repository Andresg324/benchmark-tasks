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
C = TH.clearance
case("the fit (ISO 965, 6H/6g mid-band): 0.20 mm all round between the filter's crests and the nipple's end",
     round(C * 1000, 2), 0.2)
case("the cross-threading angle: half a pitch plus the flanks' play across the pitch diameter (degrees)",
     round(math.degrees(TH.cross_tilt), 1), 2.6)
case("the nipple stands 8.42 mm out of the land (its chamfered end 0.92 mm into the bore at the catch)",
     round(TH.nip_len * 1000, 2), 8.42)
case("the drawn and collider nipple is the real M20's root (18.13 mm), not a slimmer guide pin",
     round(2 * m.NIP_ROOT_R * 1000, 3), 18.128)
case("the filter's bore is its thread's crests (18.53 mm), not a looser hole", round(2 * m.BORE_R * 1000, 3), 18.526)


def start(tilt_deg=0.0, lat=0.0, out=None, turn_deg=60.0, step_deg=5.0, axis_off=None):
    """A filter pressed against the threads' crests (or `out` from the land) and turned `turn_deg` clockwise; `lat` m
    off the axis at its gasket face (so its bore's entry is that far off the nipple's end)."""
    f = fk.FilterModel(0, TH)
    J = dict(f_up=lat, f_side=0.0, f_out=TH.d0 + 0.00005 if out is None else out, f_pitch=math.radians(tilt_deg),
             f_roll=0.0, filter=0.0)
    for _ in range(int(round(turn_deg / step_deg))):
        J["filter"] -= math.radians(step_deg)
        cmd = f.step(J)
        if f.state == fk.ENGAGED:
            J["f_out"] = cmd["f_out"][0]
            for n in ("f_up", "f_side", "f_pitch", "f_roll"):  # held within the engaged flanks' stops
                if cmd[n][5] is not None:
                    lo, hi = cmd[n][5]
                    J[n] = min(max(J[n], lo), hi)
    return f, J


f, _ = start(0.0)
case("started square and centered, pressed on the crests: the thread catches", f.state, fk.ENGAGED)
f, _ = start(2.4)
case("2.4 degrees off: still catches", f.state, fk.ENGAGED)
f, _ = start(3.0)
case("3 degrees off and turned: cross-threaded", f.state, fk.CROSSED)
f, _ = start(9.0)
case("9 degrees off (the nipple's end not square in the bore): never caught square", f.state != fk.ENGAGED, True)
f, _ = start(0.0, out=TH.d0 + 0.0005)
case("held 0.5 mm off the crests' stop (not pressed on) and turned: no catch, no pull-in", (f.state, f.on),
     (fk.FREE, True))
for mm, want in ((0.0, fk.ENGAGED), (0.1, fk.ENGAGED), (0.2, fk.ENGAGED)):
    f, _ = start(0.0, lat=mm / 1000)
    case(f"{mm} mm off the axis (placed by hand), pressed and turned: within the 0.2 mm clearance it catches",
         f.state, want)
for mm in (0.5, 1.0, 2.0, 6.0):
    # placed by hand where the colliders leave it: the base plate on the nipple's end face (in the sim the colliders
    # do the blocking; this checks the model's rule)
    f, _ = start(0.0, lat=mm / 1000, out=TH.nip_len - TH.bore_z0)
    case(f"{mm} mm off the axis (placed by hand, resting on the nipple's end) and turned: it does not go on, nothing "
         "catches", (f.state, f.on), (fk.FREE, False))
f, J = start(0.0, lat=0.0004)
case("0.4 mm off yet over the nipple's end (only the solver's give could put it there): not on the thread, no catch",
     (f.state, f.on, f.forced), (fk.FREE, False, True))
J = dict(J, f_up=0.0, f_out=TH.d0 + 0.006)
f.step(J)
f2, _ = start(0.0)
case("  backed off and started again square: catches (recoverable)", (f.forced, f2.state), (False, fk.ENGAGED))
f = fk.FilterModel(0, TH)
J = dict(f_up=0.0, f_side=0.0, f_out=TH.d0 + 0.006, f_pitch=0.0, f_roll=0.0, filter=0.0)
f.step(J)
J["f_out"] = TH.d0 - 0.0008  # shoved past the crests in one step, never turned
for _ in range(10):
    J["filter"] -= math.radians(5.0)
    f.step(J)
case("cheat: shoved past the crests' stop in one control step, then turned: not on the thread, never catches",
     (f.state, f.forced), (fk.FREE, True))
f, _ = start(0.0, out=TH.d0 + 0.006)
case("held 6 mm short of the nipple and turned: nothing", f.state, fk.FREE)
f, J = start(0.0, turn_deg=10.0)
case("turned only 10 degrees against the crests: not caught yet", f.state, fk.FREE)


def stops(cmd):
    return [cmd[n][5] for n in ("f_up", "f_side", "f_pitch", "f_roll")]


f, J = start(0.0, turn_deg=420.0)
cmd = f.step(J)
case("caught square: round stops at the engaged flanks' play (0.098 mm, 0.34 degrees), no drive on them",
     (round(cmd["f_up"][5][1] * 1e6), round(math.degrees(cmd["f_pitch"][5][1]), 2),
      [cmd[n][1] for n in ("f_up", "f_side", "f_pitch", "f_roll")]), (98, 0.34, [0.0] * 4))
d = TH.play_lat / math.sqrt(2)
cmd = f.step(dict(J, f_up=d, f_side=d))
case("  diagonal: at 45 degrees round the axis the side stops are the circle's chords (radial 0.098 mm, inside the "
     "0.199 mm clearance; no square corner at 0.139 mm)",
     round(math.hypot(cmd["f_up"][5][1], cmd["f_side"][5][1]) * 1e6), 98)
f, J = start(1.0, lat=0.0002, turn_deg=30.0)
case("caught 0.2 mm off and 1 degree crooked: caught", f.state, fk.ENGAGED)
cmd = f.step(J)
case("  its round stops are where it caught (0.20 mm, 1.0 degree): not clamped in, nothing pulls it in",
     (round(cmd["f_up"][5][1] * 1e4), round(math.degrees(cmd["f_pitch"][5][1]), 1)), (2, 1.0))
f, J = start(1.0, lat=0.0002, turn_deg=420.0)
cmd = f.step(J)
case("  a turn on, still there: nothing squeezes it square", (round(cmd["f_up"][5][1] * 1e4),
                                                              round(math.degrees(cmd["f_pitch"][5][1]), 1)), (2, 1.0))
f, J = start(0.0, turn_deg=3 * 360.0 + 60.0)
case("three turns and 60 degrees on (the first 30 start the thread): 30 past contact",
     round(math.degrees(f.past), 3), 30.0)
cmd = f.step(J)
case("its axial target 30 past contact: 1/12 turn x 1.5 mm into the gasket", round(cmd["f_out"][0], 6), -0.000125)
case("gasket torque at 270 past contact (N*m)", round(TH.torque(math.radians(270.0)), 3), 18.0)
case("the spin's drive is capped at the gasket's torque", round(cmd["filter"][4], 3), round(TH.torque(f.past), 3))
f, J = start(0.0, turn_deg=90.0)
for _ in range(30):
    J["filter"] += math.radians(10.0)
    f.step(J)
case("unscrewed past where it caught: loose again", f.state, fk.FREE)
J["f_out"] = TH.d0 + 0.00005
for _ in range(12):
    J["filter"] -= math.radians(5.0)
    f.step(J)
case("  and started again square: caught again (a recoverable mistake)", f.state, fk.ENGAGED)

# cheats on the thread
f = fk.FilterModel(0, TH)
J = dict(f_up=0.0015, f_side=0.0, f_out=TH.d0 + 0.00005, f_pitch=0.0, f_roll=0.0, filter=0.0)
for _ in range(10):
    J["filter"] -= math.radians(5.0)
    f.step(J)
case("cheat: shoved over the nipple's end 1.5 mm off its axis and turned: not on the thread, never catches",
     (f.state, f.forced), (fk.FREE, True))
f = fk.FilterModel(0, TH)
J = dict(f_up=0.0, f_side=0.0, f_out=TH.d0 + 0.00005, f_pitch=0.0, f_roll=0.0, filter=0.0)
for out in np.linspace(TH.d0 + 0.0003, TH.d0 - 0.0004, 40):  # shoved on hard, never turned
    J["f_out"] = float(out)
    cmd = f.step(J)
case("cheat: pushed on hard without turning: not threaded, the crests' stop holds it at d0",
     (f.state, cmd["f_out"][5][0]), (fk.FREE, TH.d0))
f = fk.FilterModel(0, TH)
J = dict(f_up=0.0, f_side=0.0, f_out=TH.d0 + 0.00005, f_pitch=0.0, f_roll=0.0, filter=0.0)
for _ in range(40):  # turned the wrong way (loosening) against the crests
    J["filter"] += math.radians(5.0)
    f.step(J)
case("cheat: turned anticlockwise against the crests: never catches", f.state, fk.FREE)
f = fk.FilterModel(0, TH)
Js = dict(f_up=m.FILTER_START[0], f_side=m.FILTER_START[1], f_out=m.FILTER_START[2], f_pitch=m.FILTER_START[3],
          f_roll=m.FILTER_START[4], filter=0.0)
for _ in range(200):  # doing nothing: the filter left standing on the bench
    f.step(Js)
case("cheat: doing nothing (the filter standing on the bench): free, not on the nipple", (f.state, f.on),
     (fk.FREE, False))
for _ in range(80):  # spun where it stands on the bench
    Js["filter"] -= math.radians(10.0)
    f.step(Js)
case("cheat: the filter spun on the bench, away from the nipple: nothing", f.state, fk.FREE)

# -- no invisible walls: the free filter's joint ranges cover the bench and every way it can lie ----------------------
lo_hi = {lk.name: (lk.lower, lk.upper) for lk in m.MOUNT.links}
R0, P0 = m.R_BASE, np.array((m.X_LAND, m.Y_F, m.Z_F))


def joints_for(world_pos, axis_world):
    """The chain's (up, side, out, pitch, roll) for the gasket face at world_pos with its axis along axis_world."""
    b = R0.T @ (np.asarray(world_pos, float) - P0)
    a = R0.T @ (np.asarray(axis_world, float) / np.linalg.norm(axis_world))
    roll = math.asin(max(-1.0, min(1.0, -a[1])))
    pitch = math.atan2(a[0], a[2])
    return dict(f_up=b[0], f_side=b[1], f_out=b[2], f_pitch=pitch, f_roll=roll)


def inside(J):
    return all(lo_hi[n][0] <= v <= lo_hi[n][1] for n, v in J.items())


T = m.TOP
poses = {"standing on its gasket": (0, 0, 1), "on its closed end": (0, 0, -1), "on its side, end toward the robot":
         (-1, 0, 0), "on its side, end toward the engine": (1, 0, 0), "on its side, end to the left": (0, 1, 0),
         "on its side, end to the right": (0, -1, 0)}
for x in (-0.15, 0.25, 0.72):
    for y in (-0.62, 0.05, 0.62):
        for name, ax in poses.items():
            z = T + (m.CAN_R if ax[2] == 0 else (0.0 if ax[2] > 0 else m.CAN_H))
            if not inside(joints_for((x, y, z), ax)):
                case(f"the filter {name} at x {x}, y {y}: inside its joints' ranges", False, True)
case("the filter lying any of 6 ways at 9 places across the whole bench: inside its joints' ranges "
     "(no invisible wall or support)", all(inside(joints_for((x, y, T + 0.04), ax)) for x in (-0.15, 0.25, 0.72)
                                           for y in (-0.62, 0.05, 0.62) for ax in poses.values()), True)
case("its axis pointing any way (a 5-degree grid over every direction): inside its pitch and roll ranges",
     all(inside(joints_for((0.25, 0.05, T + 0.04), (math.cos(e) * math.cos(az), math.cos(e) * math.sin(az),
                                                    math.sin(e))))
         for e in np.radians(np.arange(-90, 91, 5)) for az in np.radians(np.arange(0, 360, 5))), True)
case("lifted 70 cm above the bench: inside the ranges", inside(joints_for((0.25, 0.05, T + 0.70), (0, 0, 1))), True)

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

    def step(self, past=None, L=None, R=None, tool_on=None, off=0.0, tool_at=None, cap_still=False):
        """`off`: with the wrench on, its cup pulled this far back off the flutes along the filter's axis (m);
        `tool_at`: (origin, rotation) of a wrench put down somewhere (kept for later steps); `cap_still`: the cap
        does not turn with the filter."""
        if tool_at is not None:
            self.tool_at = tool_at
        if past is not None:
            self.set(past)
        self.L, self.R = L or self.L, R or self.R
        if tool_on is not None:
            self.tool_on = tool_on
        objs = {m.KEY: st(AX, m.R_BASE @ m._rot_z(-self.fm.turned))}
        if self.tool_on:
            tp = AX + np.array((-m.CAN_H - 0.0005 - off, 0.0, 0.0))
            spin = np.eye(3) if cap_still else m.R_BASE @ m._rot_z(-self.fm.turned) @ m.R_BASE.T  # turns with it
            objs[m.TOOL], objs[m.CAP] = st(tp, m.R_MOUNT), st(tp, spin @ m.R_MOUNT)
        else:
            p_, r_ = getattr(self, "tool_at", (m.TOOL_P0, m.R_FLAT))
            objs[m.TOOL], objs[m.CAP] = st(p_, r_), st(p_, r_)
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


def wrench_turn(t, to, held=True, rate=5.0, off=0.0, cap_still=False):
    """The wrench seated on the filter (its cup `off` m back from fully on) turns it to `to`, a hand on its handle (or
    not); `cap_still`: the cap does not turn with the filter."""
    t.step(tool_on=True, R=(GRIP, 0.2, 0.0) if held else (HOME_R, 1.0, 0.0), L=(HOME_L, 1.0, 0.0), off=off)
    while t.fm.past < math.radians(to) - 1e-9:
        t.step(past=min(to, math.degrees(t.fm.past) + rate), off=off, cap_still=cap_still)
    return t


def finish(t):
    t.step(tool_on=False)
    return t.home()


t = hand_turn(Trial(0.0), 20.0)
wrench_turn(t, 275.0)
case("by hand to 20, the wrench to 275, the wrench off, let go: 1.0", finish(t), 1.0)

# a second hand near the can, or on it (review TK-4CV45T: it must never void legitimate turning; swipes still caught)
ON_CAN2 = tuple(AX + np.array((-m.GRASP_UP + 0.02, 0.0, 0.0)))  # a second grip on the can, 2 cm nearer its gasket
NEAR_OPEN = tuple(AX + np.array((-m.GRASP_UP, 0.0, 0.06)))  # 6 cm above the can, open


def turn_with(t, to, other, both=False, rate=15.0):
    """The left hand rolls the can clockwise; the right is `other` (position, opening, roll), rolling too if `both`."""
    roll = 0.0
    while t.fm.past < math.radians(to) - 1e-9:
        roll += rate
        t.step(past=min(to, math.degrees(t.fm.past) + rate), L=(ON_CAN, W, roll),
               R=(other[0], other[1], roll if both else other[2]))
        if roll >= 150.0:
            t.step(L=(ON_CAN, 1.0, roll), R=(other[0], 1.0 if both else other[1], roll if both else other[2]))
            roll = 0.0
            t.step(L=(ON_CAN, 1.0, roll), R=(other[0], 1.0 if both else other[1], 0.0))
    return t


case("the left turns it to 100 while the right hovers open 6 cm above the can: 0.25 (a nearby hand never voids it)",
     finish(turn_with(Trial(0.0), 100.0, (NEAR_OPEN, 1.0, 0.0))), 0.25)
case("both hands closed on the can, both wrists rolling with it, to 100: 0.25",
     finish(turn_with(Trial(0.0), 100.0, (ON_CAN2, W, 0.0), both=True)), 0.25)
case("the left rolls it to 100 while the right is closed on the can, steadying, wrist still: 0.25",
     finish(turn_with(Trial(0.0), 100.0, (ON_CAN2, W, 0.0))), 0.25)
t = Trial(0.0)
for p in range(5, 280, 5):
    t.step(past=float(p), L=(ON_CAN, W, 0.0), R=(ON_CAN2, W, 0.0))
case("cheat: both hands closed on the can, neither wrist rolling, filter turned to 275 (a swipe): 0", finish(t), 0.0)
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
case("outcome-based: the angle (and the real trial's click check) reads the torque, not the tool: a hand's own roll "
     "to 275 would score 1.0 (a pinch slips far short of 18 N·m)",
     finish(hand_turn(Trial(0.0), 275.0)), 1.0)
t = Trial(0.0)
for p in range(5, 280, 5):
    t.step(past=float(p), L=(ON_CAN, W, 0.0))  # closed on it, wrist not rolling
case("closed on the can, wrist still, filter turned by something else: 0", finish(t), 0.0)
case("cross-threaded: 0", finish(Trial(0.0, state=fk.CROSSED)), 0.0)
case("never threaded (left on the bench): 0", finish(Trial(0.0, state=fk.FREE)), 0.0)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
t.fm.came_off = True
case("unscrewed off at some point and started again (a recoverable mistake), then tightened: 1.0", finish(t), 1.0)

t = Trial(0.0, state=fk.FREE)
t.step(tool_on=True, R=(GRIP, 0.2, 0.0))
case("cheat: the wrench seated on the filter and held before the thread started: 0", t.sc, 0.0)
case("  and taken off again: 0", finish(t), 0.0)
t = Trial(0.0, state=fk.FREE)
case("cheat: the start state (filter loose, wrench on the bench, hands home) already scores: no, 0", t.home(), 0.0)
case("  the filter starts free, its thread not started", (t.fm.state, math.isinf(t.fm.past)), (fk.FREE, True))
t = Trial(0.0, state=fk.FREE)
for p in range(5, 300, 5):
    t.step(L=(ON_CAN, W, float(p % 150)))  # the hand rolls the can but the thread never caught
case("cheat: the filter twisted in the hand without the thread ever catching: 0", finish(t), 0.0)

t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
t.step(R=(HOME_R, 1.0, 0.0), off=0.006)  # the cup pulled 6 mm back, still on the flutes
case("the wrench left resting on the filter (no hand on it): 0.5, not 1.0", t.home(), 0.5)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0, off=0.006)
case("the wrench held with its cup 6 mm back from fully on (its teeth still about 6 mm over the flutes), to 275: 1.0",
     finish(t), 1.0)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0, off=0.0095)
case("cheat: the cup 9.5 mm back (its teeth only 2.5 mm over the flutes' ends), the filter turned to 275: 0",
     finish(t), 0.0)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
t.step(off=0.05)  # held, 5 cm off the filter's end, in the air: not on the bench
case("the wrench pulled 5 cm off the filter but not put down on the bench: 0.5", t.step(L=(HOME_L, 1.0, 0.0)), 0.5)
_lay = (np.array(m.TOOL_P0), m.R_FLAT)
t.step(tool_on=False, tool_at=_lay, R=(tuple(m._hand_on_tool("right", *_lay)[0]), 0.0, 0.0))  # laid on its marks
case("  laid on its marks on the bench and let go: 1.0", t.home(), 1.0)

# gaming the catch rule (judged once, at 20 degrees of turn against the crests)
f, J = start(0.0, turn_deg=60.0)
worst = 0.0
for k in range(200):  # wobbled hard after the catch: pushed past its stops every way
    w = 0.001 if k % 2 else -0.001
    cmd = f.step(dict(J, f_up=w, f_side=-w * (k % 3), f_pitch=10 * w, f_roll=-10 * w))
    worst = max(worst, cmd["f_up"][5][1], cmd["f_side"][5][1])
case("wobbled after the catch: still on the thread, its round stops never wider than the flanks' play (no drive)",
     (f.state, worst <= TH.play_lat + 1e-5, [cmd[n][1] for n in ("f_up", "f_side", "f_pitch", "f_roll")]),
     (fk.ENGAGED, True, [0.0] * 4))
for tilt in (2.7, 3.5):
    f = fk.FilterModel(0, TH)
    J = dict(f_up=0.0, f_side=0.0, f_out=TH.d0 + 0.00005, f_pitch=math.radians(tilt), f_roll=0.0, filter=0.0)
    ever = False
    for k in range(120):  # back and forth, 15 degrees each way, then on
        J["filter"] += math.radians(-5.0 if (k // 3) % 2 == 0 or k >= 90 else 5.0)
        f.step(J)
        ever = ever or f.state == fk.ENGAGED
    case(f"cheat: {tilt} degrees crooked, turned back and forth: never catches (cross-threads)", (ever, f.state),
         (False, fk.CROSSED))
f = fk.FilterModel(0, TH)
J = dict(f_up=0.0, f_side=0.0, f_out=TH.d0 + 0.00005, f_pitch=math.radians(5.0), f_roll=0.0, filter=0.0)
for _ in range(19):  # pressed on the crests 5 degrees crooked and turned 19 degrees clockwise
    J["filter"] -= math.radians(1.0)
    f.step(J)
J["f_pitch"] = 0.0  # squared up for the last degree
for _ in range(3):
    J["filter"] -= math.radians(1.0)
    f.step(J)
case("cheat: turned 19 degrees on the crests 5 degrees crooked, squared for the last degree: cross-threaded",
     f.state, fk.CROSSED)
for lat_mm, out in ((0.36, TH.nip_len - TH.bore_z0), (0.36, TH.d0 + 0.00005)):
    f = fk.FilterModel(0, TH)
    J = dict(f_up=lat_mm / 1000, f_side=0.0, f_out=out, f_pitch=0.0, f_roll=0.0, filter=0.0)
    ever = False
    for k in range(120):
        J["filter"] += math.radians(-5.0 if (k // 3) % 2 == 0 else 5.0)
        f.step(J)
        ever = ever or f.state == fk.ENGAGED
    where = "resting on the nipple's end" if out > TH.d0 + 0.001 else "shoved in over the end"
    case(f"cheat: {lat_mm} mm off (just past the clearance), {where}, turned back and forth: never catches",
         ever, False)

t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
t.step(tool_on=False, R=(HOME_R, 1.0, 0.0), tool_at=(np.array((0.34, 0.05, m.TOOL_P0[2])), m.R_FLAT))
case("cheat: the wrench let go at the filter and fallen flat on the bench below it (not on its marks): 0.5", t.home(),
     0.5)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
t.step(tool_on=False, R=(HOME_R, 1.0, 0.0), tool_at=(np.array(m.TOOL_P0) + (0, 0, 0.022), m._rot_x(math.pi) @ m.R_FLAT))
case("cheat: dropped on its marks upside down (cup up): 0.5", t.home(), 0.5)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
t.step(tool_on=False, R=(HOME_R, 1.0, 0.0))
for k in range(6):  # let go low over its marks: still sliding into place
    t.step(tool_at=(np.array(m.TOOL_P0) + (0.0, 0.004 * k, 0.0), m.R_FLAT))
case("cheat: let go low over its marks, still sliding: 0.5 while it moves", t.sc, 0.5)
case("  once at rest there, flat, cup down: 1.0", t.home(), 1.0)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
_up = (np.array(m.TOOL_P0) + (0.0, 0.0, 0.30), m.R_FLAT)  # carried back cup down, 30 cm over its marks
_h = m._hand_on_tool("right", *_up)[0]
t.step(tool_on=False, tool_at=_up, R=(tuple(_h), 0.0, 0.0), L=(HOME_L, 1.0, 0.0))
case("  (the hand on the lifted wrench holds it)", m.tool_holders(m._SEEN["objects"]) != set(), True)
t.step(R=(HOME_R, 1.0, 0.0))  # let go in the air
t.step(tool_at=(np.array(m.TOOL_P0), m.R_FLAT))  # it lands on its marks, flat, cup down
case("cheat: dropped onto its marks from 30 cm (not laid down): 0.5 even at rest", t.home(), 0.5)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
t.step(tool_on=False, tool_at=_up, R=(tuple(m._hand_on_tool("right", *_up)[0]), 0.0, 0.0), L=(HOME_L, 1.0, 0.0))
t.step(R=(HOME_R, 1.0, 0.0))  # let go in the air
_lay = (np.array(m.TOOL_P0), m.R_FLAT)
t.step(tool_at=_lay)  # lands on its marks
t.step(R=(tuple(m._hand_on_tool("right", *_lay)[0]), 1.0, 0.0))  # tapped with an open hand (a touch, no hold)
case("cheat: dropped from 30 cm, then tapped on the bench with an open hand: still 0.5", t.home(), 0.5)
t = wrench_turn(hand_turn(Trial(0.0), 20.0), 275.0)
t.step(tool_on=False, tool_at=_lay, R=(tuple(m._hand_on_tool("right", *_lay)[0]), 0.0, 0.0))
t.home()
t.step(tool_at=_up, R=(tuple(m._hand_on_tool("right", *_up)[0]), 1.0, 0.0))  # lifted 30 cm on an open hand
t.step(R=(HOME_R, 1.0, 0.0))  # the hand leaves it in the air
t.step(tool_at=(np.array(m.TOOL_P0) + (0.0, 0.0, 0.15), m.R_FLAT))  # falling
t.step(tool_at=_lay)  # lands on its marks
case("cheat: laid down, then lifted 30 cm on an open hand (no hold) and dropped on its marks: 0.5", t.home(), 0.5)


# the demo's wrench carry: every hand pose of the lift and the quarter turn is within its arm's reach
from yam_arena import ik as _ik  # noqa: E402
from yam_arena.station import BASES as _BASES  # noqa: E402
_p0 = np.array(m.TOOL_P0) + (0.0, 0.0, m.LIFT)
_worst = 0.0
for _k in range(m.TURN_STEPS + 1):
    _pv = _p0 + m.R_FLAT @ np.array((0.0, 0.0, m.HANDLE_Z))
    _rr = m._rot_y(-90.0 * _k / m.TURN_STEPS)
    _pk = _pv + _rr @ (_p0 - _pv) + np.array(((m.TURN_X - _p0[0]) * _k / m.TURN_STEPS,
                                             (m.Y_F - _p0[1]) * _k / m.TURN_STEPS, 0.0))
    for _s in ("left", "right"):
        _tcp, _rot = m._hand_on_tool(_s, _pk, _rr @ m.R_FLAT, (0.0, 0.0, m.W_SHARE[_s] * m.TOOL_MASS * 9.81))
        _, _pe, _re = _ik.solve(np.asarray(_tcp, float), _ik.matrix_to_quat_xyzw(_rot), None,
                                base_pos=np.asarray(_BASES[_s]), iters=300)
        _worst = max(_worst, 1000 * _pe + math.degrees(_re))
case("demo: both hands' poses through the wrench's lift and quarter turn are reachable, aimed up by the arms' give "
     "under the wrench's weight (IK within 1 mm + 1 degree)",
     _worst < 1.0, True)


def test_all():
    """pytest: every case above passed."""
    assert bad == 0

print("all pass" if not bad else f"{bad} FAILED")
