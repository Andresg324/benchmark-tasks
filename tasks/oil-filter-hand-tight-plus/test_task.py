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
            objs[g] = st(p, yaw)
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
case("300 degrees, an open hand 1 cm above the filter at the end", t.hold(40), 0.9)
t = twist(Trial(), 300)
t.R = AWAY
case("300 degrees, arm left away from home", t.hold(40), 0.9)
t = twist(Trial(), 300)
for k in range(40):
    t.step(R=((R_HOME[0], R_HOME[1] + 0.002 * (k % 2), R_HOME[2]), 1.0, 0.0))
case("300 degrees, arms at home but still moving", t.sc, 0.9)
t = twist(Trial(), 120)
t.R = AWAY
case("120 degrees and an arm away (no 0.9 for a partial)", t.hold(40), 0.25)


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
bad += t.ok
print("ok  " if not t.ok else "FAIL", "no success with an arm away from home")
done(twist(Trial(), 460))  # a reset clears an earlier trial's crushed gasket
got = done(twist(Trial(), 300))
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", got, "a reset clears an earlier trial's over-tightening")
got = Trial().hold(40)
bad += got != 0.0 or abs(tk.past_seat_deg(None, KEY) + 270.0) > 1e-9
print("ok  " if got == 0.0 else "FAIL", got, "after a reset the filter is back at 270 degrees short of contact")
m.REQUIRE_ARMS_HOME = False
t = twist(Trial(), 300)
t.R = AWAY
got = t.hold(40)
m.REQUIRE_ARMS_HOME = True
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", got, "REQUIRE_ARMS_HOME off: done, arm away, scores 1.0")
print("all pass" if not bad else f"{bad} FAILED")
