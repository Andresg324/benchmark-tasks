"""Offline checks of the riffle-split scorer: the pour pan moved pose by pose through rifflekit's sand model (the
demo's level tip, the mistakes and near misses), then set down and let go, no simulator.

    /data/IsaacLab-Arena/.venv/bin/python test_task.py      (prints "all pass")
"""

import importlib.util
import math
import os

import numpy as np
from yam_arena.task_file import ObjectState

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("riffle_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit, rk = m.kit, m.rk

TOP = m.TOP
PARK = {"left_gripper": np.array([0.20, 0.27, TOP + 0.20]), "right_gripper": np.array([0.20, -0.27, TOP + 0.20])}
PAN0 = np.array([*m.PAN0, TOP])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _quat(R):
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def st(pos, q=(0, 0, 0, 1.0), vel=(0, 0, 0), half=0.01):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.array(vel, float), ang_vel=np.zeros(3),
                       low=p - half, high=p + half)


class Trial:
    """A fresh trial: the pan full on the table, the arms parked at home."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.pos, self.R = PAN0.copy(), np.eye(3)
        self.g = {k: v.copy() for k, v in PARK.items()}
        self.open = {k: 1.0 for k in PARK}
        self.vel = np.zeros(3)
        self.out = None
        self.step()

    def objs(self):
        return {"pan": st(self.pos, _quat(self.R), self.vel), "splitter": st((m.SPLIT_X, 0.0, TOP)),
                **{k: st(v) for k, v in self.g.items()}}

    def step(self, n=1):
        for _ in range(n):
            kit._OPENING.update(self.open)
            o = self.objs()
            rk.update(o)
            self.out = (m.score(o), m.success(o))
        return self.out

    def hold_handles(self):
        """Both hands closed on the handles (tool points at the grip spots)."""
        for g, sy in (("left_gripper", 1), ("right_gripper", -1)):
            self.g[g] = self.pos + self.R @ np.array([0.0, sy * m.GRIP_Y, m.HANDLE_ZC])
            self.open[g] = 0.25

    def pose(self, R, lip=None, n=1):
        """Pan turned to R with the lip's middle at `lip` (world), over n steps (the hands follow the handles)."""
        lip = m.LIP_AT if lip is None else np.asarray(lip, float)
        start, R0 = self.pos.copy(), self.R.copy()
        target = lip - R @ m.LIP_LOCAL
        for k in range(1, n + 1):
            self.pos = start + (target - start) * k / n
            self.R = R if k == n else R0
            self.hold_handles()
            self.step()

    def pour(self, deg=60.0, roll=0.0, lip=None, ramp=120, hold=80):
        """Tip toward the lip to `deg` over `ramp` steps (rolled `roll` deg: +y end up), hold, tip back level."""
        for k in range(1, ramp + 1):
            self.pose(_rx(math.radians(roll)) @ m._ry(-math.radians(deg) * k / ramp), lip)
        self.step(hold)
        self.pose(_rx(math.radians(roll)), lip)

    def set_down(self, home=True, let_go=True):
        self.pos, self.R = PAN0.copy(), np.eye(3)
        self.hold_handles()
        self.step(3)
        if let_go:
            self.open = {k: 1.0 for k in PARK}
            for g, sy in (("left_gripper", 1), ("right_gripper", -1)):
                self.g[g] = self.pos + np.array([0.0, sy * (m.GRIP_Y + 0.06), m.HANDLE_ZC])
            self.step(3)
        if home:
            self.g = {k: v.copy() for k, v in PARK.items()}
            self.step(5)
        return self.out


def demo_like(**kw):
    t = Trial()
    t.pour(**kw)
    return t


def check(name, got, want):
    assert got == want, f"{name}: {got} != {want}"
    print(f"ok  {name}: {got}")


# The demo's pour: level lip over the hopper's middle, 60 degrees, set down, let go, home.
t = demo_like()
ch = rk.chutes()
print("    chutes g:", np.round(ch, 1).tolist(), "catch:", {k: round(v, 1) for k, v in rk.catch().items()})
check("level pour, set down, let go, home", t.set_down(), (1.0, True))
check("same, arms not home", demo_like().set_down(home=False), (0.9, False))
# a fast but correct tip (20 steps = 1 s) still pours through the whole row
t = Trial()
t.pour(ramp=20)
check("fast level tip", t.set_down(), (1.0, True))

# Do nothing / never let go / never set down
t = Trial()
t.step(200)
check("do nothing", t.set_down(home=True, let_go=True), (0.0, False))
check("poured but still holding the handles", demo_like().set_down(home=False, let_go=False), (0.0, False))
t = demo_like()
t.hold_handles()
t.open = {k: 1.0 for k in PARK}  # open hands left on the handles: not let go (pads within 1 cm)
t.step(5)
check("poured, left over the splitter, hands open on the handles", t.out, (0.0, False))
t = demo_like()
t.pos = PAN0.copy()
t.R = _rx(math.radians(20))  # set down on one handle, tipped 20 degrees
t.open = {k: 1.0 for k in PARK}
t.g = {k: v.copy() for k, v in PARK.items()}
t.step(5)
check("set down tipped 20 degrees", t.out, (0.0, False))

# Off-level lip: the low end's chutes get more
check("lip rolled 3 degrees (about 9 mm over the handles)", demo_like(roll=3.0).set_down(), (1.0, True))
check("lip rolled 6 degrees", demo_like(roll=6.0).set_down()[0], 0.5)
check("lip rolled 10 degrees", demo_like(roll=10.0).set_down()[0], 0.25)
# One arm lifting one end (the pan hangs from one handle and pours over its low corner)
check("one end lifted 35 degrees", demo_like(deg=45.0, roll=35.0).set_down()[0], 0.25)
# Poured over one end of the row only / off the hopper
check("lip 3 cm off-center (two end chutes miss)", demo_like(lip=m.LIP_AT + [0, 0.03, 0]).set_down()[0], 0.25)
check("poured short of the hopper onto the table", demo_like(lip=m.LIP_AT + [0.06, 0, 0]).set_down()[0], 0.0)
check("poured straight into the front catch pan", demo_like(lip=(m.CATCH_X[1], 0, TOP + 0.08)).set_down()[0], 0.0)
# Partial pours
t = demo_like(deg=30.0)
print("    tipped only 30 degrees:", round(rk.through() / rk.sample(), 3), "through")
check("tipped only 30 degrees (a third stays in the pan)", t.set_down()[0], 0.25)
t = Trial()
t.pour(deg=60.0, ramp=120, hold=0)
print("    tipped and straight back:", round(rk.through() / rk.sample(), 3))
# turned right over (past 81 degrees) along the full lip: all of it goes down the chutes, evenly
check("turned over 100 degrees along the lip", demo_like(deg=100.0, ramp=10).set_down(), (1.0, True))

# A jolt on the way that tips sand onto the table for 2 steps still counts after the proper pour (spills never go back)
t = Trial()
for k in range(2):
    t.pose(m._ry(-math.radians(42)), lip=(0.36, 0.0, TOP + 0.15))
t.pose(np.eye(3), lip=(0.36, 0.0, TOP + 0.15))
print("    jolt spilled g:", round(rk.spilled(), 1))
t.pour()
check("2-step 42-degree jolt onto the table, then a level pour", t.set_down()[0], 0.25)

# Two hands, one on each handle bar, for at least 90 % of the grams poured
class Hands(Trial):
    """The demo's pour with other hand placements: `place(trial)` sets self.g / self.open for each step."""

    def __init__(self, place):
        self.place = place
        super().__init__()

    def hold_handles(self):
        Trial.hold_handles(self)
        self.place(self)


def back_wall_pinch(t):  # one hand pinching the middle of the back wall, the other home
    t.g["left_gripper"] = t.pos + t.R @ np.array([m.PAN_D / 2 + 0.0015, 0.0, 0.04])
    t.open["left_gripper"] = 0.05
    t.g["right_gripper"], t.open["right_gripper"] = PARK["right_gripper"].copy(), 1.0


def one_handle(t):  # the left hand on its handle, the right home
    t.g["right_gripper"], t.open["right_gripper"] = PARK["right_gripper"].copy(), 1.0


def same_handle(t):  # both hands on the +y handle
    t.g["right_gripper"] = t.pos + t.R @ np.array([0.0, m.GRIP_Y - 0.04, m.HANDLE_ZC])


def fist(t):  # the right hand an empty closed fist at its handle
    t.open["right_gripper"] = 0.0


def open_hand(t):  # the right hand wide open around its handle
    t.open["right_gripper"] = 1.0


for name, place in (("one hand pinching the back wall's middle (QC attack)", back_wall_pinch),
                    ("one hand on its handle, the other home", one_handle),
                    ("both hands on the same handle", same_handle),
                    ("right hand an empty closed fist at its handle", fist),
                    ("right hand wide open around its handle", open_hand)):
    t = Hands(place)
    t.pour()
    check(name, t.set_down()[0], 0.0)
t = Hands(lambda t: t.open.update(right_gripper=0.32))  # the loaded pan pries the fingers open a little (sim: 0.32)
t.pour()
check("right fingers pried to 0.32 open on the bar", t.set_down(), (1.0, True))
t = Hands(lambda t: t.R[2, 0] > math.sin(math.radians(36)) and open_hand(t))
t.pour(ramp=60)  # the right hand opens once the pan is past 36 degrees, as the sand starts to run
check("right hand lets go halfway through the tip", t.set_down()[0], 0.0)

# The hopper's end flanges catch a pour a few mm off center into the end chutes
check("lip 3 mm off center", demo_like(lip=m.LIP_AT + [0, 0.003, 0]).set_down()[0], 1.0)
print("    3 mm off center spilled g:", round(rk.spilled(), 1))
check("lip 6 mm off center (the far end chute gets 40 % less)", demo_like(lip=m.LIP_AT + [0, 0.006, 0]).set_down()[0],
      0.5)

# A new trial resets the sand
t = Trial()
assert rk.through() == 0 and abs(rk.in_pan() - m.SAMPLE_G) < 1e-6, "episode reset"
print("ok  new trial: full pan, empty chutes")

# The text's numbers match the code
assert round(m.LOST_FULL * m.SAMPLE_G) == 13 and m.HALF_FULL == (0.45, 0.55) and m.EVEN == 0.35
assert round(m.HALF_FULL[0] * m.SAMPLE_G) == 300 and round(m.HALF_FULL[1] * m.SAMPLE_G) == 367
print("all pass")
