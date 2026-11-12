"""Offline checks of the dividers scorer: step-by-step joint histories, near misses included, no simulator.

    /data/IsaacLab-Arena/.venv/bin/python test_task.py        (prints "all pass")
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, pv = m.kit, m.pv
PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
START = {"lift": 0.0, "dv_turn": 0.0, "dv_x": m.CX - m.R / 2 - m.MX, "dv": m.CY - m.MY, "tip": 0.0, "swivel": 0.0,
         "knob": 0.0}


DOWNWARD = (1.0, 0.0, 0.0, 0.0)  # a gripper pointing straight down (turned 180 degrees about x), as in the demo


def st(pos, quat=(0, 0, 0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def to_world(j, local):
    """A point in the dividers' frame (turned and leaning as the joints have them), in world coordinates."""
    f = m._frame(j)
    x, y, z, w = f.quat_xyzw
    rot = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
    return f.pos + rot @ np.asarray(local, float)


def handle_center(j):
    return to_world(j, (m.R / 2, 0.0, m.KNOB_Z0 + j["knob"] + m.KNOB_H / 2))


class Trial:
    """One trial; each step() is one control step through the pivot kit's record and the scorer. By default the
    right hand is on the dividers' handle; hand=False parks it (let go)."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.j = dict(START)

    def step(self, hand=True, right=None, opening=None, **j):
        """opening: the right fingers' opening (default: closed on the 2.6 cm handle while the hand is on it, open
        otherwise)."""
        self.j.update(j)
        kit._OPENING["left_gripper"] = 1.0
        kit._OPENING["right_gripper"] = opening if opening is not None else 0.28 if hand and right is None else 1.0
        for k, v in self.j.items():
            kit._JOINTS[f"bench/{k}"] = kit.JointState(v, 0.0, False)
        pv.step(m.KEY, dict(self.j))
        g = dict(PARK)
        if right is not None:
            g["right_gripper"] = right
        elif hand:
            g["right_gripper"] = tuple(handle_center(self.j))
        objs = {"bench": st(m.BASE), **{k: st(p, DOWNWARD) for k, p in g.items()}}
        ok = m.success(objs)
        return m.score(objs), ok

    def rest(self, n=10, **kw):
        out = None
        for _ in range(n):
            out = self.step(**kw)
        return out

    def carry_to(self, dx, dy, n=10):
        """Lift the dividers 3 cm, slide them (in the turned frame) to (dx, dy), set them down."""
        self.step(lift=0.03, knob=0.0)
        x0, y0 = self.j["dv_x"], self.j["dv"]
        for k in range(1, n + 1):
            self.step(dv_x=x0 + (dx - x0) * k / n, dv=y0 + (dy - y0) * k / n)
        self.step(lift=0.0)

    def swing(self, a0, a1, n=None, knob=-0.003, **kw):
        n = n or max(1, int(abs(a1 - a0) / 5))
        for k in range(1, n + 1):
            self.step(dv_turn=math.radians(a0 + (a1 - a0) * k / n), knob=knob, **kw)

    def regrip(self):
        """Let go of the handle (the spring pushes it up), roll the wrist back with the hand off it (the demo's hand is
        off for about 3.6 s, 72 steps; 4 here), then grip and press again; nothing turns."""
        self.step(knob=0.0)
        self.rest(4, knob=0.0, hand=False)
        self.step(knob=0.0)
        self.step(knob=-0.003)

    def to_cork(self, turn_deg):
        """Lift off, carry back so the handle is over the cork's center, set down."""
        th = math.radians(turn_deg)
        c, s = math.cos(th), math.sin(th)
        wx, wy = m.CX - m.R / 2 * c - m.MX, m.CY - m.R / 2 * s - m.MY  # the pivot point's world offset from the mark
        self.carry_to(c * wx + s * wy, -s * wx + c * wy)


def full_job(t, end=375.0, home=True, regrip=False):
    """Carry the dividers to the mark, press and swing the full circle without letting go (a handle that turns in the
    hand), lift off, back to the cork. regrip=True lets go on the plate between three 125-degree strokes instead."""
    t.rest(3, hand=False)
    t.carry_to(0.0002, -0.0001)
    t.step(knob=-0.003)
    if regrip:
        for a0, a1 in ((0.0, 125.0), (125.0, 250.0), (250.0, end)):
            t.swing(a0, a1)
            t.regrip()
    else:
        t.swing(0.0, end)
    t.step(knob=0.0)
    t.to_cork(end)
    t.rest(2, hand=False)
    return t.rest(25, hand=False, right=None if home else (0.30, -0.15, m.TOP + 0.18))


def check(name, got, want):
    assert got == want, f"{name}: got {got}, want {want}"
    print(f"ok  {name}: {got}")


# do nothing: the dividers stay in the cork
t = Trial()
check("do nothing", t.rest(30, hand=False)[0], 0.0)

# the full job: one stroke round the mark, held throughout, back in the cork, hands off
t = Trial()
s, ok = full_job(t)
check("full job", (s, ok), (1.0, True))
check("arc", m.arc_deg() >= 360, True)

# the same, arms not home: the job alone earns full credit
t = Trial()
check("full job, arm not home", full_job(t, home=False)[0], 1.0)

# review, Nov 11: letting go between strokes with the dividers standing on the plate. Real dividers fall over, so only
# the first 125-degree stroke counts (the sim's joints keep them upright; the scorer applies the fall)
t = Trial()
check("full job with let-go regrips", full_job(t, regrip=True)[0], 0.0)
check("let-go regrips: fallen", m._H.fallen, True)
check("let-go regrips: arc stops at the first stroke", 124.5 <= m.arc_deg() <= 125.5, True)

# an empty closed fist resting on the handle, or an open hand around it, is not holding: they fall
for name, op in (("empty fist", 0.0), ("open hand", 1.0)):
    t = Trial()
    t.carry_to(0.0, 0.0)
    t.step(knob=-0.003)
    t.swing(0.0, 90.0)
    t.rest(3, opening=op, knob=-0.003)
    t.swing(90.0, 370.0, opening=op)
    t.step(knob=0.0)
    t.to_cork(370.0)
    check(f"{name} on the handle: fallen", m._H.fallen, True)
    check(f"{name} on the handle: 90 deg only", t.rest(10, hand=False)[0], 0.0)

# a one-step flicker of the fingers (a pressure pop) is not a let-go
t = Trial()
t.carry_to(0.0, 0.0)
t.step(knob=-0.003)
t.swing(0.0, 180.0)
t.step(opening=1.0, knob=-0.003)
t.swing(180.0, 375.0)
t.step(knob=0.0)
t.to_cork(375.0)
check("one-step flicker: not fallen", m._H.fallen, False)
check("one-step flicker", t.rest(10, hand=False)[0], 1.0)

# let go standing in the cork (points stuck in it): they stand, nothing falls
t = Trial()
t.rest(10, hand=False)
check("let go in the cork: not fallen", m._H.fallen, False)

# still holding the handle at the end: nothing counts
t = Trial()
full_job(t)
check("still held at the end", t.rest(5, hand=True)[0], 0.0)

# full circle, but the dividers left standing in the mark (not back in the cork)
t = Trial()
t.carry_to(0.0, 0.0)
t.step(knob=-0.003)
t.swing(0.0, 365.0)
t.step(knob=0.0)
check("full circle, left on the plate", t.rest(10, hand=False)[0], 0.75)

# short of a full circle: 350 degrees -> half credit only; 170 -> nothing
t = Trial()
t.carry_to(0.0, 0.0)
t.step(knob=-0.003)
t.swing(0.0, 350.0)
t.step(knob=0.0)
t.to_cork(350.0)
check("350 deg swing", t.rest(10, hand=False)[0], 0.5)
t = Trial()
t.carry_to(0.0, 0.0)
t.step(knob=-0.003)
t.swing(0.0, 170.0)
t.step(knob=0.0)
check("170 deg swing", t.rest(10, hand=False)[0], 0.0)

# back and forth over the same 90 degrees counts each degree once
t = Trial()
t.carry_to(0.0, 0.0)
t.step(knob=-0.003)
for _ in range(5):
    t.swing(0.0, 90.0)
    t.swing(90.0, 0.0)
t.step(knob=0.0)
check("back and forth 90 deg: arc", m.arc_deg() <= 92, True)
check("back and forth 90 deg", t.rest(10, hand=False)[0], 0.0)

# started off the mark: set down 0.8 mm off (not seated), pressed, swung: a stray circle, 0 even after a proper one
t = Trial()
t.carry_to(0.0008, 0.0)
t.step(knob=-0.003)
check("0.8 mm off: not seated", pv.engaged({}, m.KEY), False)
t.swing(0.0, 10.0)
check("off-mark start latched", m._H.skated, True)
t.carry_to(0.0, 0.0)
t.step(knob=-0.003)
t.swing(10.0, 380.0)
t.step(knob=0.0)
t.to_cork(380.0)
check("off-mark start, then a full circle", t.rest(25, hand=False)[0], 0.0)

# 0.4 mm off seats (within the mark's cone)
t = Trial()
t.carry_to(0.0004, 0.0)
t.step(knob=-0.003)
check("0.4 mm off: seated", pv.engaged({}, m.KEY), True)

# skated mid-swing: the pressure came off while still swinging (point down, unseated) -> 0, irreversible
t = Trial()
t.carry_to(0.0, 0.0)
t.step(knob=-0.003)
t.swing(0.0, 150.0)
t.swing(150.0, 156.0, knob=-0.0005)  # handle barely pressed: the point is out of the mark, 6 deg turned on the plate
check("skated latched", m._H.skated, True)
t.swing(156.0, 380.0)
t.step(knob=0.0)
t.to_cork(380.0)
check("skate, then completed", t.rest(25, hand=False)[0], 0.0)

# skated out sideways: the point pushed 3.6 mm off the mark while pressed (out past the rim and the play) -> 0
t = Trial()
t.carry_to(0.0, 0.0)
t.step(knob=-0.003)
t.swing(0.0, 200.0)
for k in range(1, 13):
    t.step(dv_x=0.0003 * k)
check("pushed out of the mark", m._H.skated, True)

# review, Nov 10: seated 0.45 mm off center, then the sim's play shifts it 0.6 mm more (1.05 mm): still seated, scores
t = Trial()
t.carry_to(0.00045, 0.0)
t.step(knob=-0.003)
t.step(dv_x=0.0006)
check("seated 0.45 mm off, shifted 0.6 mm by play: still seated", m._H.skated, False)
t.step(knob=0.0)
check("pushed out, 200 deg", t.rest(10, hand=False)[0], 0.0)

# a tiny settle while unseated (0.3 mm) is not a scratch
t = Trial()
t.carry_to(0.0003, 0.0)
t.step(dv_x=0.0001)
t.step(knob=-0.003)
check("small settle not a skate", m._H.skated, False)

# turning in the air (lifted 2 mm) scribes nothing and scratches nothing
t = Trial()
t.carry_to(0.0, 0.0)
t.step(lift=0.002, knob=-0.003)
t.swing(0.0, 360.0, lift=0.002)
t.step(lift=0.0, knob=0.0)
check("swing in the air: no arc", m.arc_deg(), 0.0)
check("swing in the air: no skate", m._H.skated, False)
check("swing in the air", t.rest(10, hand=False)[0], 0.0)

# the point drops into the mark under the dividers' own weight, but turning without a press rides it out
t = Trial()
t.carry_to(0.0, 0.0)
t.step(knob=0.0)
check("set in the mark, not pressed: seated", pv.engaged({}, m.KEY), True)
t.swing(0.0, 0.8, n=2, knob=0.0)
check("0.8 deg unpressed: still seated", pv.engaged({}, m.KEY), True)
t.swing(0.8, 1.6, n=2, knob=-0.001)
check("1.6 deg with the handle only 1 mm down: ridden out", pv.engaged({}, m.KEY), False)
t = Trial()
t.carry_to(0.0, 0.0)
t.swing(0.0, 30.0, knob=-0.0016)
check("handle 1.6 mm down through 30 deg: seated", pv.engaged({}, m.KEY), True)
check("handle 1.6 mm down: no skate", m._H.skated, False)

# letting go at the end of a stroke: the hand's release wiggles the dividers +-0.8 deg unpressed, the point stays in
t = Trial()
t.carry_to(0.0, 0.0)
t.swing(0.0, 120.0)
for d in (0.5, 0.8, 0.2, -0.6, -0.8, 0.0):
    t.step(dv_turn=math.radians(120.0 + d), knob=0.0)
check("release wiggle: still seated", pv.engaged({}, m.KEY), True)
check("release wiggle: no skate", m._H.skated, False)

# a pressed handle with no swing, a hand hovering, a regrip that does not turn: no credit
t = Trial()
t.carry_to(0.0, 0.0)
t.rest(20, knob=-0.003)
t.regrip()
check("pressing with no swing", t.rest(10, hand=False)[0], 0.0)

# dragging the dividers across the plate on their points (not lifted) scratches it
t = Trial()
t.carry_to(0.0, 0.03)
for k in range(1, 11):
    t.step(dv=0.03 - 0.003 * k)
check("dragged across the plate", m._H.skated, True)

# a scene reset (new trial) clears the history
t = Trial()
check("reset: first step scores 0", t.step(hand=False)[0], 0.0)
check("reset clears the skate", m._H.skated, False)
check("reset arc 0", m.arc_deg(), 0.0)

# the in-cork check reads the joints: points down and inside the cork
t = Trial()
check("starts in the cork", m.in_cork(t.j), True)
check("lifted out of the cork", m.in_cork(dict(t.j, lift=0.01)), False)

# the tip joint (Nov 12): held but leaning more than 10 degrees on the plate counts as fallen; 8 degrees in hand does not;
# a lean in the air (carried) is not a fall
t = Trial()
t.carry_to(0.0, 0.0)
t.step(knob=-0.003)
t.swing(0.0, 90.0, tip=math.radians(8.0))
check("held, leaning 8 deg: not fallen", m._H.fallen, False)
t.step(tip=math.radians(12.0))
check("leaning 12 deg on the plate: fallen", m._H.fallen, True)
t = Trial()
t.step(lift=0.03, tip=math.radians(30.0))
t.rest(3, tip=0.0)
check("leaning in the air: not fallen", m._H.fallen, False)

# the handle's center follows the lean: 30 degrees over moves it sideways by its height x sin(30)
j = dict(START, dv_turn=0.0, tip=math.radians(30.0))
hc, h0 = handle_center(j), handle_center(dict(j, tip=0.0))
hh = m.KNOB_Z0 + m.KNOB_H / 2
check("lean moves the handle sideways", abs((h0[1] - hc[1]) - hh * 0.5) < 1e-9 and abs(hc[0] - h0[0]) < 1e-9, True)

# the lean's law: on the steel nothing holds them up (only the small lean torque, no stiffness); carried in the air the
# hand holds them upright; stuck in the cork the lean is held
on = dict(START, dv_x=0.0, dv=0.0)
tq = m.Topple.drive("bench/tip", 0.05, 0.0, on, {})
check("on the plate: no upright stiffness", tq[1] == 0.0 and tq[2] > 0.0, True)
tq = m.Topple.drive("bench/tip", 0.05, 0.0, dict(on, lift=0.02), {})
check("in the air: held upright by the hand", tq[0] == 0.0 and tq[1] > 0.0, True)
mem = {}
m.Topple.drive("bench/tip", 0.01, 0.0, START, mem)
check("in the cork: lean held where stuck", mem["limits"], (0.01 - m.STUCK, 0.01 + m.STUCK))

# contact, not distance (Nov 12 audit): a hand at grip width hovering 5 mm over the handle is not holding (the
# dividers fall); at the end, finger tips resting on the handle's top are still on them (0 until let go), and 5 mm
# above it they are off (1.0)
t = Trial()
t.carry_to(0.0, 0.0)
t.step(knob=-0.003)
t.swing(0.0, 40.0)
hc = handle_center(t.j)
side = (hc[0], hc[1], hc[2] + m.KNOB_H / 2 + 0.0128 + 0.005)  # the finger tips (12.8 mm past the tool point) 5 mm
# over the handle's top, the fingers at the opening of a grip on it
t.rest(3, right=side, opening=0.28, knob=-0.003)
check("fingers at grip width 5 mm over the handle: not holding, fallen", m._H.fallen, True)
t = Trial()
full_job(t)
top = handle_center(t.j) + np.array([0.0, 0.0, m.KNOB_H / 2 + 0.0128])  # the tool point with the closed finger tips
# (12.8 mm past it) just on the handle's top
for gap, want in ((0.001, 0.0), (0.005, 1.0)):
    check(f"closed hand, finger tips {gap * 1000:.0f} mm over the handle's top at the end",
          t.step(hand=False, right=tuple(top + [0.0, 0.0, gap]), opening=0.0)[0], want)

print("all pass")
