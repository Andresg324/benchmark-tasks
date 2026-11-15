"""Offline checks of the oil filter task's scorer: step-by-step histories of the filter's thread and the two grippers
(position, finger opening and wrist yaw), run through twistkit's own held check and the task's success()/score(), with
near misses and the cheats an adversarial review tried. No simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, tk = m.kit, m.tk
TOP, KEY = m.TOP, m.KEY
tk.register(*[o for o in m.OBJECTS if isinstance(o, tk.Threaded)])
L_HOME, R_HOME = (0.20, 0.27, TOP + 0.22), (0.18, -0.30, TOP + 0.22)
AX = (m.HEAD_C[0], m.HEAD_C[1])
W = 2 * m.CAN_R / (2 * abs(tk.FINGER_OPEN))  # the opening with the pads on the can (about 0.70)
ABOVE = (*AX, TOP + 0.20)  # a hand lifted clear of the filter
AWAY = ((0.30, -0.15, TOP + 0.15), 1.0, 0.0)
bad = 0


def st(p, yaw_deg=0.0):
    p = np.array(p, float)
    a = math.radians(yaw_deg) / 2
    return ObjectState(pos=p, quat_xyzw=np.array((0, 0, math.sin(a), math.cos(a))), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=p - .01, high=p + .01)


def sg(p, yaw_deg=0.0):
    """A gripper pointing straight down (pad tips below the tool point, as in the demo), its wrist turned `yaw_deg`
    about the vertical: the quaternion Rz(yaw) * Rx(180 degrees)."""
    p = np.array(p, float)
    a = math.radians(yaw_deg) / 2
    return ObjectState(pos=p, quat_xyzw=np.array((math.cos(a), math.sin(a), 0.0, 0.0)), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=p - .01, high=p + .01)


def grip_z(below_top):
    """Tool point height for a grasp `below_top` metres under the can's top."""
    return TOP + m.SEAT + m.CAN_H - below_top


class Trial:
    """One trial: each step sets the filter's angle (degrees past contact) and both grippers as (position, opening,
    wrist yaw in degrees), runs twistkit's held accounting and the task's success() and score()."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.past = m.THREAD.start
        self.L = (L_HOME, 1.0, 0.0)
        self.R = (R_HOME, 1.0, 0.0)
        self.step()

    def step(self, past=None, L=None, R=None):
        if past is not None:
            self.past = past
        self.L, self.R = L or self.L, R or self.R
        tk._track(KEY).u = math.radians(-self.past)  # what twistkit reads from the simulator's hinge
        objs = {"head": st((*AX, TOP)), KEY: st((*AX, TOP + m.SEAT))}
        for g, (p, op, yaw) in (("left_gripper", self.L), ("right_gripper", self.R)):
            objs[g] = sg(p, yaw)
            kit._OPENING[g] = op
        tk._account(objs)
        self.ok = m.success(objs)
        self.sc = m.score(objs)
        return self.sc

    def hold(self, n=1, **kw):
        for _ in range(n):
            self.step(**kw)
        return self.sc

    def home(self, n=30):
        self.R = (R_HOME, 1.0, 0.0)
        self.L = (L_HOME, 1.0, 0.0)
        return self.hold(n)


def twist(t, to, z=None, stroke=120.0, rate=15.0, lag=0.0, slip=0.0, side="R"):
    """A regrasp loop by one hand at tool height z: close on the can, roll the wrist clockwise `rate` degrees a step
    (the filter turns `rate - slip`), open, roll back, repeat until the filter is at `to`. `lag`: fraction of a step's
    roll that happens in the step the fingers land and in the step they open (sloppy timing)."""
    z = grip_z(0.015) if z is None else z
    g = (*AX, z)
    yaw = 0.0

    def hand(op):
        return {side: (g, op, yaw)}

    while t.past < to - 1e-9:
        t.step(**hand(1.0))
        if lag:  # the filter already turning in the step the fingers land
            yaw -= lag * rate
            t.step(past=min(to, t.past + lag * (rate - slip)), **hand(W))
        else:
            t.step(**hand(W))
        k = 0.0
        while k < stroke - 1e-9 and t.past < to - 1e-9:
            d = min(rate, stroke - k)
            yaw -= d
            t.step(past=min(to, t.past + d * (rate - slip) / rate), **hand(W))
            k += d
        if lag:  # still rolling in the step the fingers open
            yaw -= lag * rate
            t.step(past=min(to, t.past + lag * (rate - slip)), **hand(1.0))
        else:
            t.step(**hand(1.0))
        yaw = 0.0  # roll back with the fingers open
        t.step(**hand(1.0))
    t.step(**{side: (ABOVE, 1.0, 0.0)})
    return t


def swipe(t, to, rate=15.0, L=None, R=None):
    """The filter turned to `to` by something other than a holding hand (a knock, a swipe, the other arm), with the
    grippers as given (default: as they are)."""
    while abs(t.past - to) > 1e-9:
        s = math.copysign(min(rate, abs(to - t.past)), to - t.past)
        t.step(past=t.past + s, L=L, R=R)
    return t


def done(t):
    return t.home()


def case(name, got, want):
    global bad
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)


LIFTED = (ABOVE, 1.0, 0.0)

# --- bands ---------------------------------------------------------------------------------------
case("nothing", Trial().hold(100), 0.0)
case("spun on to gasket contact and left there (the tempting wrong answer)", done(twist(Trial(), 0)), 0.0)
for deg, want in ((80, 0.0), (90, 0.25), (170, 0.25), (200, 0.5), (235, 0.5), (241, 1.0), (270, 1.0), (300, 1.0),
                  (330, 1.0), (359, 1.0), (380, 0.5), (440, 0.5), (460, 0.0)):
    case(f"{deg} degrees past contact", done(twist(Trial(), deg)), want)
case("cranked to 460 then backed off to 300 (crushed: irreversible)",
     done(swipe(twist(Trial(), 460), 300, R=LIFTED)), 0.0)
case("taken to 400 then backed off to 300", done(swipe(twist(Trial(), 400), 300, R=LIFTED)), 0.5)
case("tightened to 300 then loosened back to 60", done(swipe(twist(Trial(), 300), 60, R=LIFTED)), 0.0)
t = Trial()
tk.set_angle(KEY, -2100.0, held_now=False)  # flags it off, as the simulator's thread update does
t.step(past=-2100.0)
case("unscrewed off the nipple, then spun back on to 300", done(twist(t, 300)), 0.0)

# --- correct jobs done differently --------------------------------------------------------------
case("grasp 1.5 cm below the top (as the demo)", done(twist(Trial(), 300)), 1.0)
case("firmer, lower grasp: tool point 3.2 cm below the top", done(twist(Trial(), 300, z=grip_z(0.032))), 1.0)
case("grasp at mid-can: tool point 3.5 cm below the top", done(twist(Trial(), 300, z=grip_z(0.035))), 1.0)
case("low grasp: tool point 5.5 cm below the top", done(twist(Trial(), 300, z=grip_z(0.055))), 1.0)
case("shallow grasp: tool point at the can's top", done(twist(Trial(), 300, z=grip_z(0.0))), 1.0)
for stroke in (120.0, 90.0, 60.0, 45.0):
    for lag in (0.25, 0.5, 1.0):
        case(f"{stroke:.0f}-degree strokes, {lag} of a step rolled in each grasp and release step",
             done(twist(Trial(), 300, stroke=stroke, lag=lag)), 1.0)
case("pads slipping 3 degrees a step on the can", done(twist(Trial(), 300, slip=3.0)), 1.0)
case("left arm does it (the filter is in reach of either)", done(twist(Trial(), 300, side="L")), 1.0)

# --- review, Nov 12: a second hand near the can, or helping, does not void the holder's turning ----------------
near = ((AX[0] - 0.05, AX[1] + 0.03, grip_z(0.03)), 1.0, 0.0)  # left tool point 2.5 cm outside the can's wall

def twist_with_left(t, to, left):
    """twist() by the right hand with the left gripper held at `left` throughout."""
    t.L = left
    return twist(t, to)


case("right hand turns it to 300 while the left hovers open 2.5 cm from the can",
     done(twist_with_left(Trial(), 300, near)), 1.0)
case("right hand turns it to 300 while the left rests closed 1 cm from the can (steadying the head)",
     done(twist_with_left(Trial(), 300, ((AX[0] - m.CAN_R - 0.01, AX[1], grip_z(0.03)), 0.0, 0.0))), 1.0)
t = Trial()
gR, gL = (*AX, grip_z(0.015)), (*AX, grip_z(0.045))
yaw = 0.0
t.step(R=(gR, W, 0.0), L=(gL, W, 0.0))
while t.past < 300 - 1e-9:  # both hands closed on the can, both wrists rolling with it, 10 degrees a step
    yaw -= 10.0
    t.step(past=t.past + 10.0, R=(gR, W, yaw), L=(gL, W, yaw))
t.step(R=(gR, 1.0, yaw), L=(gL, 1.0, yaw))
case("both hands on the can, turning it together to 300", done(t), 1.0)
t = Trial()
yaw = 0.0
t.step(R=(gR, W, 0.0), L=(gL, W, 0.0))
while t.past < 300 - 1e-9:  # both hands closed on the can, only the right wrist rolling (the left steadying, slipping)
    yaw -= 10.0
    t.step(past=t.past + 10.0, R=(gR, W, yaw), L=(gL, W, 0.0))
t.step(R=(gR, 1.0, yaw), L=(gL, 1.0, 0.0))
case("both hands on the can, the right wrist turning it to 300, the left only steadying", done(t), 1.0)
t = Trial()
t.step(R=(gR, W, 0.0), L=(gL, W, 0.0))
while t.past < 300 - 1e-9:  # both hands closed on the can, neither wrist rolling, yet it turns: swiped
    t.step(past=t.past + 10.0)
t.step(R=(gR, 1.0, 0.0), L=(gL, 1.0, 0.0))
case("both hands closed on the can, neither wrist rolling, the filter turned to 300 anyway (swiped)", done(t), 0.0)

# --- cheats and near misses ---------------------------------------------------------------------
case("knocked spin: turned to 300 with no grip on it", done(swipe(Trial(), 300)), 0.0)
case("gripped to contact, then swiped the rest of the way to 300", done(swipe(twist(Trial(), 0), 300, R=LIFTED)), 0.0)
case("gripped to 260, then knocked another 40 to 300", done(swipe(twist(Trial(), 260), 300, R=LIFTED)), 1.0)
case("gripped to 150, then knocked another 100 to 250", done(swipe(twist(Trial(), 150), 250, R=LIFTED)), 0.0)
other = ((AX[0] - 0.05, AX[1] + 0.03, grip_z(0.03)), 1.0, 0.0)
case("one hand hovering loosely around the can (84 % open) while the other arm swipes it to 300",
     done(swipe(Trial(), 300, R=((*AX, grip_z(0.015)), 0.84, 0.0), L=other)), 0.0)
case("one hand closed on the can, wrist still, while the other arm swipes it to 300",
     done(swipe(Trial(), 300, R=((*AX, grip_z(0.015)), W, 0.0), L=other)), 0.0)
t = swipe(twist(Trial(), 150), 0, R=LIFTED)
twist(t, 150)
case("held 0 to 150, swiped back to 0, held 0 to 150 again, swiped 150 to 270 (laundering)",
     done(swipe(t, 270, R=LIFTED)), 0.0)
t = twist(Trial(), 100)
g = ((*AX, grip_z(0.015)), W)
t.step(R=(*g, 0.0))
for k in range(12):  # the closed wrist rolls 180 degrees with the pads slipping, the filter not moving
    t.step(R=(*g, -15.0 * (k + 1)))
case("gripped to 100, wrist rolled 180 slipping on the can, then the other arm swipes it to 300 (budget laundering)",
     done(swipe(t, 300, L=other)), 0.0)
t = Trial()
t.step(R=(*g, 0.0))
yaw = 0.0
while t.past < 0:  # one long held roll from 270 short of contact to contact, banking no turning for later
    yaw -= 15.0
    t.step(past=t.past + 15.0, R=(*g, yaw))
case("held roll up to contact, then the hand stays closed and still while the other arm swipes it to 250",
     done(swipe(t, 250, L=other)), 0.0)
t = twist(Trial(), 0)
yaw = 0.0
t.step(R=(*g, yaw))
while t.past < 300 - 1e-9:  # a closed hand creeping its wrist 0.1 degree a step while the other arm turns it 3
    yaw -= 0.1
    t.step(past=t.past + 3.0, R=(*g, yaw), L=other)
case("closed hand creeping its wrist 0.1 degree a step, the other arm turning it 3 a step to 300", done(t), 0.0)
t = twist(Trial(), 300)
for k in range(200):
    t.step(past=300 - 0.5 * (k % 2))
case("a correct job, then the filter jittering half a degree for 200 steps", done(t), 1.0)

t = twist(Trial(), 300)
t.R = ((*AX, grip_z(0.015)), W, 0.0)
case("300 degrees, still gripping the filter at the end", t.hold(40), 0.5)
t = twist(Trial(), 300)
t.R = ((*AX, TOP + m.SEAT + m.CAN_H + 0.01), 0.0, 0.0)
case("300 degrees, a closed hand resting on the filter top at the end", t.hold(40), 0.5)
t = twist(Trial(), 300)
t.R = ((AX[0] - 0.04, AX[1], grip_z(0.015)), 0.0, 0.0)
case("300 degrees, a closed hand against the can's side at the end", t.hold(40), 0.5)
t = twist(Trial(), 400)
t.R = ((*AX, grip_z(0.015)), W, 0.0)
case("400 degrees and still gripping (no more than 0.5)", t.hold(40), 0.5)
t = twist(Trial(), 300)
t.R = ((*AX, TOP + m.SEAT + m.CAN_H + 0.01), 1.0, 0.0)
case("300 degrees, an open hand 1 cm above the filter at the end (let go)", t.hold(40), 1.0)
# --- let go: judged by contact (the gripper's own shape within 2 mm of the can), not by a zone around it ----------
TIP = 0.0128  # the pad tips' reach below the tool point, pointing down (guardkit's tip box)
_body = m.gk.hand_boxes(0.0)[1]  # a closed hand's finger body: its outer face 2.4 cm out along the closing axis
FIST = abs(_body.center[0]) + _body.size[0] / 2
CAN_TOP = TOP + m.SEAT + m.CAN_H
for name, R, want in [
        ("closed fist beside the can, its fingers 5 mm off the side (the old 1.5 cm zone capped it at 0.5)",
         ((AX[0] - m.CAN_R - FIST - 0.005, AX[1], grip_z(0.03)), 0.0, 0.0), 1.0),
        ("fingers half open, tips 5 mm over the can's top (the old zone capped it at 0.5)",
         ((*AX, CAN_TOP + TIP + 0.005), 0.5, 0.0), 1.0),
        ("fingers 79 % open around the can, tool point on its axis, pads 4 mm off its wall (the old zone capped it)",
         ((*AX, grip_z(0.03)), (m.CAN_R + 0.004) / abs(tk.FINGER_OPEN), 0.0), 1.0),
        ("fingers half open, tips 1 mm over the can's top: touching it", ((*AX, CAN_TOP + TIP + 0.001), 0.5, 0.0), 0.5),
        ("open hand, fingertips resting on the can's top: touching it", ((*AX, CAN_TOP + TIP), 0.35, 0.0), 0.5),
        ("closed fist beside the can, its fingers 1 mm off the side: touching it",
         ((AX[0] - m.CAN_R - FIST - 0.001, AX[1], grip_z(0.03)), 0.0, 0.0), 0.5),
        ("wide-open hand, a finger's inner face 1 mm off the can's wall: touching it",
         ((AX[0] + abs(tk.FINGER_OPEN) - m.CAN_R - 0.001, AX[1], grip_z(0.03)), 1.0, 0.0), 0.5)]:
    t = twist(Trial(), 300)
    t.R = R
    case(f"300 degrees at the end, {name}", t.hold(40), want)

t = twist(Trial(), 300)
t.R = AWAY
case("300 degrees, arm left away from home (not scored)", t.hold(40), 1.0)
t = twist(Trial(), 300)
for k in range(40):
    t.step(R=((R_HOME[0], R_HOME[1] + 0.002 * (k % 2), R_HOME[2]), 1.0, 0.0))
case("300 degrees, arms at home but still moving (not scored)", t.sc, 1.0)
t = twist(Trial(), 120)
t.R = AWAY
case("120 degrees and an arm away", t.hold(40), 0.25)


# --- twistkit's held check itself ----------------------------------------------------------------
def held(gripper, opening):
    kit._EPISODE["count"] += 1
    objs = {KEY: st((*AX, TOP + m.SEAT)), "right_gripper": st(gripper), "left_gripper": st(L_HOME)}
    kit._OPENING["right_gripper"], kit._OPENING["left_gripper"] = opening, 1.0
    tk._account(objs)
    return tk.held(objs, KEY)


ON = (*AX, grip_z(0.015))
for name, args, want in [
        ("fingers closed on the can, centered", (ON, W), True),
        ("fingers on the can, tool point 1 cm off the axis", (tuple(np.add(ON, (0.01, 0, 0))), W), True),
        ("fingers squeezing 10 % past the can's width (pad compliance)", (ON, W - 0.10), True),
        ("fingers 2 % wider than the can", (ON, W + 0.02), True),
        ("fingers 5 % wider than the can (5 mm of air)", (ON, W + 0.05), False),
        ("fingers hovering at 84 % open", (ON, 0.84), False),
        ("an empty closed hand resting on the filter top", ((*AX, grip_z(-0.017)), 0.0), False),
        ("an empty closed hand beside the can", (tuple(np.add(ON, (0.045, 0, 0))), 0.0), False),
        ("an open hand around the can", (ON, 1.0), False),
        ("fingers at can width but 4 cm to the side", (tuple(np.add(ON, (0.04, 0, 0))), W), False),
        ("fingers at can width, tool point 1 cm above the top (pad tips 3 mm down the rim)", ((*AX, grip_z(-0.010)), W),
         True),
        ("fingers at can width with the tool point 2.5 cm above the top (pads clear of the can)",
         ((*AX, grip_z(-0.025)), W), False),
        ("fingers at can width, tool point 1.5 cm above the gasket face (pads at the can's base)",
         ((*AX, grip_z(0.055)), W), True),
        ("tool point 6.5 cm below the top (pads below the can, on the head)", ((*AX, grip_z(0.065)), W), False)]:
    got = held(*args)
    ok = got == want
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"held={got}:", name)

t = twist(Trial(), 300)
done(t)
bad += not t.ok
print("ok  " if t.ok else "FAIL", "success once done and the arms home")
t = twist(Trial(), 300)
t.R = AWAY
t.hold(40)
bad += not t.ok
print("ok  " if t.ok else "FAIL", "success with an arm left away from home (arm position is not scored)")
done(twist(Trial(), 460))  # a reset clears an earlier trial's crushed gasket
got = done(twist(Trial(), 300))
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", got, "a reset clears an earlier trial's over-tightening")
got = Trial().hold(40)
bad += got != 0.0 or abs(tk.past_seat_deg(None, KEY) + 270.0) > 1e-9
print("ok  " if got == 0.0 else "FAIL", got, "after a reset the filter is back at 270 degrees short of contact")
bad += hasattr(m, "REQUIRE_ARMS_HOME") or hasattr(m, "_other_hand_on")
print("ok   no arm-home or other-hand-nearby rule left in the scorer")


# The settling hold: the simulator steps the scene for 0.5 s after each reset before the first frame, and the thread's
# drives must act in those steps too (without them the filter dropped onto its slide's stop and rose back in the
# first second of the video). A fake embodiment checks that every settling step is preceded by the thread's drives,
# that a second reset does the same, and that control steps after the reset are left as they were.
import sys  # noqa: E402
import types  # noqa: E402


class _Env:
    def __init__(self):
        self.unwrapped, self.log = self, []

    def reset(self):
        pass

    def step(self, action):
        self.log.append(len(_calls))


class _Emb:
    def __init__(self):
        self._env = None

    def _ensure_sim(self, goal):
        if self._env is None:
            self._env = _Env()

    def reset(self, *a, **k):
        self._ensure_sim(None)
        self._env.reset()
        for _ in range(10):
            self._env.step(None)
        return "obs"


_calls = []
_name = "yam_arena.inspect_embodiment"
_saved, _drive, _had = sys.modules.get(_name), tk._drive_thread, "head" in kit._MECHS
sys.modules[_name] = types.SimpleNamespace(YamArenaEmbodiment=_Emb)
tk._drive_thread = lambda base, spec: _calls.append(spec.name)
kit._MECHS.setdefault("head", m.OBJECTS[0])
try:
    m._hold_thread_while_settling()
    e = _Emb()
    e.reset()
    first = list(e._env.log)
    e.reset()
    second = e._env.log[10:]
    e._env.step(None)  # a control step after the reset: no extra drive call (twistkit sets them as before)
    after = e._env.log[-1]
finally:
    tk._drive_thread = _drive
    if _saved is None:
        sys.modules.pop(_name)
    else:
        sys.modules[_name] = _saved
    if not _had:
        kit._MECHS.pop("head")
ok = first == list(range(1, 11)) and second == list(range(11, 21)) and after == 20 and _calls == ["head"] * 20
bad += not ok
print("ok  " if ok else "FAIL", "the thread's drives act in every settling step after each reset, and only then")
# --- the wrench (cap filter wrench on a ratchet): its model, and the scorer with it ------------------------------
def tset(**j):
    """Put the wrench's joints in the joint cache, as the simulator reports them."""
    for n, v in j.items():
        kit._JOINTS[f"{m.TOOL}/{n}"] = kit.JointState(pos=v, vel=0.0, latched=False)


def tjoints(turn, lift=0.0, x=0.0, y=0.0):
    return {"t_lift": lift, "t_turn": turn, "t_x": x, "tool": y}


def tool_run(t, to_past, hands_on=True, step_deg=2.0, handle_r=0.55):
    """Seat the wrench on the can and swing its handle clockwise in 40-degree strokes (ratcheting back between them),
    pushing each stroke past the gasket's hold, until the filter is `to_past` degrees past contact; a hand on the
    handle near its end if `hands_on`. Each step: the model, then the filter put where the wrench drove it."""
    q = math.radians(m.STROKE_A0 - 180.0)
    stroke_left = m.WRENCH_STROKE
    while t.past < to_past - 1e-6:
        J = tjoints(q)
        tset(**J)
        tm = m.tool_step(J)
        if tm.drive_u is not None:
            tk._track(KEY).u = tm.drive_u
            tm.drive_u = None
        hp = m.tool_point(J, (-handle_r, 0.0, m.HANDLE_Z))
        R = (tuple(hp + np.array([0, 0, 0.013])), 0.05 if hands_on else 1.0, 0.0) if hands_on else (R_HOME, 1.0, 0.0)
        t.step(past=-math.degrees(tk._track(KEY).u), R=R)
        if stroke_left <= 0:  # ratchet back with the hand on the handle
            for _ in range(int(m.WRENCH_STROKE / step_deg)):
                q += math.radians(step_deg)
                J = tjoints(q)
                tset(**J)
                m.tool_step(J)
                hp = m.tool_point(J, (-handle_r, 0.0, m.HANDLE_Z))
                t.step(R=(tuple(hp + np.array([0, 0, 0.013])), 0.05, 0.0) if hands_on else None)
            stroke_left = m.WRENCH_STROKE
        # swing on: the hand leads the pawl by what the gasket takes, plus a little
        need = m.gasket_torque(t.past)
        q -= math.radians(step_deg) + need / m.K_TOOL * 0.0
        stroke_left -= step_deg
    return t


def tool_off(t):
    """Lift the wrench off, slide it aside and lay it on the bench; hands away."""
    q = kit._JOINTS[f"{m.TOOL}/t_turn"].pos
    for J in (tjoints(q, lift=0.03), tjoints(q, lift=0.03, y=m.TOOL_SIDE), tjoints(q, lift=m.LIFT_TABLE, y=m.TOOL_SIDE)):
        tset(**J)
        m.tool_step(J)
        t.step()
    return t


def tool_trial(by_hand=20.0):
    t = Trial()
    twist(t, by_hand)  # spun on by hand to contact and a little past it
    t.home(5)
    return t


# The gasket's torque: 18 N*m at three quarters of a turn past contact (Kawasaki's figure), linear in the angle.
tq_ok = (abs(m.gasket_torque(270.0) - 18.0) < 1e-9 and abs(m.gasket_torque(135.0) - 9.0) < 1e-9
         and m.gasket_torque(-30.0) == m.RUN_ON and m.gasket_torque(40.0) > 2.5)
bad += not tq_ok
print("ok  " if tq_ok else "FAIL", "gasket torque: 18 N*m at 3/4 turn past contact, 9 at 3/8, over 2.5 (a hand's slip) by 40 degrees")

# The model: seated on the can, a push under what the gasket holds turns nothing; past it, the filter follows; turned
# back, the ratchet freewheels and the filter stays; lifted, it lets go.
kit._EPISODE["count"] += 1
tk._track(KEY).u = math.radians(-100.0)  # 100 degrees past contact: the gasket holds 6.7 N*m
q0 = math.radians(m.STROKE_A0 - 180.0)
m.tool_step(tjoints(q0))
small = m.tool_step(tjoints(q0 - 0.010)).drive_u  # 0.01 rad x 400 = 4 N*m: under 6.7
big = m.tool_step(tjoints(q0 - 0.050)).drive_u  # 20 N*m: the filter turns
u_after = big
tk._track(KEY).u = big
back = m.tool_step(tjoints(q0 + 0.30)).drive_u  # turned back 0.35 rad: freewheels
ref_after = m.tool_model().ref
lifted = m.tool_step(tjoints(q0, lift=0.03))
off_center = m.tool_step(tjoints(q0, x=0.005))
model_ok = (small is None and big is not None and abs(math.degrees(-big) - (100.0 + math.degrees(0.050 - 6.7 / 400 * 0.0)
            - math.degrees(m.gasket_torque(100.0) / m.K_TOOL))) < 0.5 and back is None
            and abs(ref_after - ((q0 + 0.30) - u_after)) < 1e-9 and not lifted.coupled and not off_center.coupled)
bad += not model_ok
print("ok  " if model_ok else "FAIL", "the wrench: holds under the gasket's torque, turns it past, freewheels back, lets go lifted or off-center")

# Lying on the bench its slides and turn are held; lifted, free; seated, slides held.
lie = m.tool_step(tjoints(q0, lift=m.LIFT_TABLE, y=m.TOOL_SIDE))
lock_ok = lie.lock_xy is not None and lie.lock_turn is not None
up = m.tool_step(tjoints(q0, lift=0.03, y=0.05))
lock_ok = lock_ok and up.lock_xy is None and up.lock_turn is None
seat = m.tool_step(tjoints(q0))
lock_ok = lock_ok and seat.lock_xy is not None and seat.lock_turn is None
bad += not lock_ok
print("ok  " if lock_ok else "FAIL", "slides held lying on the bench and seated, free when lifted")

# Lift limits: centered, it can sit down on the can; off-center near it, it rests on the can's top; clear, on the bench.
law, mem = m.ToolLiftLaw(), {}
law.drive("tool/t_lift", 0.0, 0.0, tjoints(q0), mem)
l_c = mem["limits"][0]
law.drive("tool/t_lift", 0.03, 0.0, tjoints(q0, lift=0.03, y=0.05), mem)
l_n = mem["limits"][0]
law.drive("tool/t_lift", 0.03, 0.0, tjoints(q0, lift=0.03, y=m.TOOL_SIDE), mem)
l_f = mem["limits"][0]
lim_ok = l_c == 0.0 and l_n == m.LIFT_FREE and l_f == m.LIFT_TABLE and abs(m.TOOL_SIDE) >= m.CLEAR
sl, smem = m.ToolSlideLaw(1, -0.3, 0.3), {}
m.tool_step(tjoints(q0, lift=-0.04, y=m.TOOL_SIDE))
sl.drive("tool/tool", m.TOOL_SIDE, 0.0, tjoints(q0, lift=-0.04, y=m.TOOL_SIDE), smem)
lim_ok = lim_ok and smem["limits"] == (m.CLEAR, 0.3)
m.tool_step(tjoints(q0, lift=0.03, y=m.TOOL_SIDE))
sl.drive("tool/tool", m.TOOL_SIDE, 0.0, tjoints(q0, lift=0.03, y=m.TOOL_SIDE), smem)
lim_ok = lim_ok and smem["limits"] is None
# Down over the can, the cup stays on it: pushed sideways it stays over the can (the stop below it stays at the can's
# top, nothing lifts it), and it leaves the can only lifted until its rim clears the top.
kit._EPISODE["count"] += 1
law, mem = m.ToolLiftLaw(), {}
law.drive("tool/t_lift", 0.03, 0.0, tjoints(q0, lift=0.03), mem)
law.drive("tool/t_lift", 0.006, 0.0, tjoints(q0, lift=0.006, y=0.001), mem)
law.drive("tool/t_lift", 0.006, 0.0, tjoints(q0, lift=0.006, y=0.005), mem)
cup_ok = mem["limits"][0] == 0.0 and m.tool_model().over_can
law.drive("tool/t_lift", 0.016, 0.0, tjoints(q0, lift=0.016, y=0.012), mem)  # off-center, not over the can
cup_ok = cup_ok and mem["limits"][0] == 0.0  # still over it: the rim has not cleared the top
law.drive("tool/t_lift", 0.018, 0.0, tjoints(q0, lift=0.018, y=0.012), mem)
cup_ok = cup_ok and not m.tool_model().over_can and mem["limits"][0] == m.LIFT_FREE
law.drive("tool/t_lift", 0.016, 0.0, tjoints(q0, lift=0.016, y=0.012), mem)
cup_ok = cup_ok and mem["limits"][0] == 0.016  # resting on the can's top from above: the stop is under the cap
law.drive("tool/t_lift", -0.09, 0.0, tjoints(q0, lift=-0.09, x=-0.02, y=0.097), mem)  # laid beside the head
cup_ok = cup_ok and mem["limits"][0] == m.LIFT_TABLE  # on the bench, not sinking through it
lim_ok = lim_ok and cup_ok
bad += not lim_ok
print("ok  " if lim_ok else "FAIL", "the wrench sits down on the can only centered; near it, on its top; clear of it, on the bench; carried low it cannot slide in over the head")

# The scorer with the wrench.
kit._EPISODE["count"] += 1
case("wrench: spun on by hand, tightened to 300 with the wrench, laid on the bench, hands away",
     done(tool_off(tool_run(tool_trial(), 300.0))), 1.0)
case("wrench: tightened to 300 but left on the filter", done(tool_run(tool_trial(), 300.0)), 0.5)
t = tool_run(tool_trial(), 300.0)
tool_off(t)
J = tool_joints = m.tool_joints()
hp = m.tool_point(J, (-0.5, 0.0, m.HANDLE_Z))
case("wrench: laid down but still held", t.hold(30, R=(tuple(hp + np.array([0, 0, 0.013])), 0.05, 0.0)), 0.5)
case("wrench: only to 200 past contact", done(tool_off(tool_run(tool_trial(), 200.0))), 0.5)
case("wrench: cranked to 470 (past 1.25 turns)", done(tool_off(tool_run(tool_trial(), 470.0))), 0.0)
case("wrench: turned with no hand on it (not possible in the sim: a stand-in for a knock)",
     done(tool_off(tool_run(tool_trial(), 300.0, hands_on=False))), 0.0)
tset(t_lift=m.LIFT_TABLE, t_turn=m.TOOL_TURN0, t_x=m.TOOL_X0, tool=m.TOOL_Y0)

print("all pass" if not bad else f"{bad} FAILED")
