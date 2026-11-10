"""Offline checks of the wire-nut task's scorer: step-by-step histories of the connector's turning, the right wire's
position and the grippers, with near misses. No simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, tk = m.kit, m.tk
TOP = m.TOP
tk.register(*[o for o in m.OBJECTS if isinstance(o, tk.Threaded)])
PARK = {"left_gripper": (0.20, 0.27, TOP + 0.22), "right_gripper": (0.18, -0.30, TOP + 0.22)}
BASE = np.array([m.BOX_C[0], m.BOX_C[1], TOP])
START_DEG = -m.THREAD.start  # 915 degrees back from snug
bad = 0


def st(pos):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0.0, 0, 0, 1]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


class W:
    b = -m.SHORT  # the right wire's offset from even


ON = 2 * m.NUT_R / (2 * abs(m.FINGER_OPEN))  # opening with the pads on the connector


def yawq(deg):
    a = math.radians(deg) / 2
    return np.array([0.0, 0.0, math.sin(a), math.cos(a)])


def nut_pos():
    back = tk.backed_off_deg(None, m.NUT)
    return BASE + (0, 0, m.SEAT + m.THREAD.pitch * max(back, 0.0) / 360.0)


def step(nut=None, held=False, b=None, grippers=None, grip_dz=0.014, grip_open=None, follow=True):
    """nut: degrees back from snug this step (as PhysX would report it); held: the left gripper closed on the
    connector at `grip_dz` above its skirt (opening `grip_open`, default on its width), its wrist rolled with the
    connector if `follow`; b: the right wire's offset (m); grippers: name -> position (fingers open) or
    (position, opening) or (position, opening, yaw degrees). Runs twistkit's real grip accounting."""
    t = tk._track(m.NUT)
    if nut is not None:
        t.u = math.radians(nut)
        t.rise = m.THREAD.pitch * nut / 360.0
    if b is not None:
        W.b = b
    kit._JOINTS[m.WIRE_B] = kit.JointState(pos=W.b, vel=0.0, latched=False)
    objs = {"box": st(BASE), m.NUT: st(nut_pos()), m.WIRE_B: st(BASE + (0, 0, W.b))}
    g = dict(PARK, **(grippers or {}))
    if held:
        yaw = math.degrees(t.u) if follow else 0.0
        g["left_gripper"] = (tuple(nut_pos() + (0, 0, grip_dz)), ON if grip_open is None else grip_open, yaw)
    for k, v in g.items():
        if isinstance(v[0], tuple):
            pos, op, yaw = (v + (0.0,))[:3]
        else:
            pos, op, yaw = v, 1.0, 0.0
        objs[k] = st(pos)
        objs[k] = ObjectState(pos=objs[k].pos, quat_xyzw=yawq(yaw), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                              low=objs[k].low, high=objs[k].high)
        kit._OPENING[k] = op
    tk._account(objs)
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames):
    """frames: callables or (steps, kwargs). A fresh trial each time."""
    kit._EPISODE["count"] += 1
    W.b = -m.SHORT
    out = step()  # the trial's first scored step: arms at PARK, so home is PARK
    for n, kw in frames:
        for i in range(n):
            out = step(**(kw if i == 0 else {k: v for k, v in kw.items() if k != "nut"}))
    return out


def twist(to, frm=None, held=True, by=15.0, **kw):
    """The connector turned from `frm` (default: where it is) to `to` degrees back from snug in 15-degree steps, the
    left hand on it (with a step of grip before and after) if `held`."""
    frames = [(1, dict(held=held, **kw))] if held else []
    a = START_DEG if frm is None else frm
    sgn = -1 if to < a else 1
    while abs(a - to) > 1e-9:
        a = max(a - by, to) if sgn < 0 else min(a + by, to)
        frames.append((1, dict(nut=a, held=held, **kw)))
    return frames + ([(1, dict(held=held, **kw))] if held else [])


def pull(to, frm=-m.SHORT, n=12):
    return [(1, dict(b=frm + (to - frm) * k / n)) for k in range(1, n + 1)]


DONE = [(40, {})]
EVEN = pull(0.0)
AWAY = {"right_gripper": (0.30, -0.15, TOP + 0.15)}

cases = [
    ("nothing", [(100, {})], 0.0),
    ("the right wire evened, connector not turned", EVEN + DONE, 0.0),
    ("connector twisted snug without evening the wires (the tempting wrong answer)", twist(-15.0) + DONE, 0.0),
    ("wires evened, connector twisted snug (15 past the seat), arms home", EVEN + twist(-15.0) + DONE, 1.0),
    ("the same, an arm not home", EVEN + twist(-15.0) + [(40, dict(grippers=AWAY))], 0.9),
    ("wires evened, connector 40 degrees short of snug", EVEN + twist(40.0) + DONE, 1.0),
    ("wires evened, connector 50 degrees short of snug (loose)", EVEN + twist(50.0) + DONE, 0.5),
    ("wires evened, connector 180 degrees short (copper just covered)", EVEN + twist(180.0) + DONE, 0.5),
    ("wires evened, connector 200 degrees short (copper showing)", EVEN + twist(200.0) + DONE, 0.25),
    ("wires evened, connector only started (one turn on, copper showing)", EVEN + twist(540.0) + DONE, 0.25),
    ("wires evened, connector 550 degrees short (under a full turn on)", EVEN + twist(550.0) + DONE, 0.0),
    ("wires evened, connector half a turn on, never bitten", EVEN + twist(700.0) + DONE, 0.0),
    ("right wire 2.5 mm short when twisted on", pull(-0.0025) + twist(-15.0) + DONE, 1.0),
    ("right wire 3.5 mm short when twisted on", pull(-0.0035) + twist(-15.0) + DONE, 0.0),
    ("right wire pulled 5 mm past the left one", pull(0.005) + twist(-15.0) + DONE, 0.0),
    ("twisted on short, then the wire pulled up level (into a snug nut)", twist(-15.0) + EVEN + DONE, 0.0),
    ("twisted on short, then the wire evened while only 1.5 turns on, then snugged",
     twist(400.0) + EVEN + twist(-15.0, frm=400.0) + DONE, 0.0),
    ("evened after half a turn on (coil not yet gripping), then snugged",
     twist(700.0) + EVEN + twist(-15.0, frm=700.0) + DONE, 1.0),
    ("twisted on short, backed off, wires evened, twisted on again",
     twist(-15.0) + twist(800.0, frm=-15.0) + EVEN + twist(-15.0, frm=800.0) + DONE, 1.0),
    ("snug, then the wire tugged up 3 mm", EVEN + twist(-15.0) + pull(0.003, frm=0.0) + DONE, 0.0),
    ("snug, then the wire nudged 1.5 mm", EVEN + twist(-15.0) + pull(0.0015, frm=0.0) + DONE, 1.0),
    ("connector spun on with no grip on it (swiped)", EVEN + twist(-15.0, held=False) + DONE, 0.0),
    ("the first 300 degrees by hand, the rest knocked", EVEN + twist(615.0) + twist(-15.0, frm=615.0, held=False)
     + DONE, 0.0),
    ("snug, then backed off to 300 degrees", EVEN + twist(-15.0) + twist(300.0, frm=-15.0) + DONE, 0.25),
    ("snug but the connector still held at the end", EVEN + twist(-15.0)
     + [(40, dict(grippers={"left_gripper": ((0.30, 0.0, TOP + m.SEAT + 0.014), 0.21)}))], 0.0),
    ("snug but a closed hand resting on the connector top", EVEN + twist(-15.0)
     + [(40, dict(grippers={"left_gripper": ((0.30, 0.0, TOP + m.SEAT + m.NUT_L + 0.012), 0.0)}))], 0.0),
    ("snug but the right wire still pinched", EVEN + twist(-15.0)
     + [(40, dict(grippers={"right_gripper": ((0.25, -0.030, TOP + m.JOG), 0.05)}))], 0.0),
    ("snug, an open hand 2 cm above the connector", EVEN + twist(-15.0)
     + [(40, dict(grippers={"left_gripper": ((0.30, 0.0, TOP + m.SEAT + m.NUT_L + 0.02), 1.0)}))], 0.9),
    ("done, arms at home but still moving", EVEN + twist(-15.0)
     + [(1, dict(grippers={"right_gripper": (0.18, -0.30 + 0.002 * (k % 2), TOP + 0.22)})) for k in range(40)], 0.9),
]


# twistkit's grip accounting (QC of an earlier twistkit task): heights, transitions, hovering, laundering
def regrasp(to, stroke=150.0, lag=0.0, **kw):
    """A regrasp loop like the demo's: close, roll `stroke`, open, roll back; with `lag`, that fraction of a 15-degree
    step already rolls in the step the fingers land and the step they open."""
    frames, a = [], START_DEG
    while a > to + 1e-9:
        n = min(stroke, a - to)
        frames.append((1, dict(held=True, grip_open=1.0, **kw)))  # the hand around the connector, open
        if lag:
            a -= lag * 15.0
            frames.append((1, dict(nut=a, held=True, **kw)))
        k = 0.0
        while k < n - 2 * lag * 15.0 - 1e-9 and a > to + 1e-9:
            d = min(15.0, n - k, a - to)
            a -= d
            k += d
            frames.append((1, dict(nut=a, held=True, **kw)))
        if lag and a > to + 1e-9:
            a -= lag * 15.0
            frames.append((1, dict(nut=a, held=True, grip_open=1.0, **kw)))
    return frames


def swipe(to, frm):
    sgn = -1 if to < frm else 1
    out, a = [], frm
    while abs(a - to) > 1e-9:
        a = max(a - 15.0, to) if sgn < 0 else min(a + 15.0, to)
        out.append((1, dict(nut=a)))
    return out


def open_roll(a, wider):
    """The left hand open `wider` (m) than the connector around it at grip height, its wrist rolled with it."""
    pos = BASE + (0, 0, m.SEAT + m.THREAD.pitch * max(a, 0.0) / 360.0 + 0.014)
    return {"left_gripper": (tuple(pos), ON + wider / (2 * abs(m.FINGER_OPEN)), a)}


HOVER = {"left_gripper": ((m.BOX_C[0], m.BOX_C[1], TOP + m.SEAT + 0.024), ON + 0.15, 0.0)}
cases += [
    ("regrasp loop, demo-like grip", EVEN + regrasp(-15.0) + DONE, 1.0),
    ("regrasp loop, grip low on the skirt (tool point 1 mm above the skirt)", EVEN + regrasp(-15.0, grip_dz=0.001)
     + DONE, 1.0),
    ("regrasp loop, grip near the top (tool point 1 cm above the connector top)",
     EVEN + regrasp(-15.0, grip_dz=m.NUT_L + 0.008) + DONE, 1.0),
    ("regrasp loop, short 45-degree strokes, half a step rolling in each grip and release step",
     EVEN + regrasp(-15.0, stroke=45.0, lag=0.5) + DONE, 1.0),
    ("hand 2 cm above the connector top, closed, while it turns (not on it)",
     EVEN + regrasp(-15.0, grip_dz=m.NUT_L + 0.02) + DONE, 0.0),
    ("open hand hovering around the connector (14 mm wider) while the other arm swipes it on",
     EVEN + [(1, dict(nut=a, grippers=HOVER)) for a in np.arange(START_DEG - 15, -15.0 - 1e-9, -15.0)] + DONE, 0.0),
    ("open hand 6 mm clear of the connector all round, rolling with it, while the other arm turns it on",
     EVEN + [(1, dict(nut=a, grippers=open_roll(a, 0.012))) for a in np.arange(START_DEG - 15, -15.0 - 1e-9, -15.0)]
     + DONE, 0.0),
    ("open hand 3 mm clear of the connector all round, rolling with it, while the other arm turns it on",
     EVEN + [(1, dict(nut=a, grippers=open_roll(a, 0.006))) for a in np.arange(START_DEG - 15, -15.0 - 1e-9, -15.0)]
     + DONE, 0.0),
    ("hand closed on the connector but its wrist not turning while it turns (turned by something else)",
     EVEN + twist(-15.0, follow=False) + DONE, 0.0),
    ("laundering: held 915->600, swiped back to 915, held to 600 again, swiped the rest to snug",
     EVEN + twist(600.0) + swipe(915.0, 600.0) + twist(600.0, frm=915.0) + swipe(-15.0, 600.0) + DONE, 0.0),
    ("held to snug, 60 degrees knocked loose and 60 knocked back (under the 90-degree allowance)",
     EVEN + twist(-15.0) + swipe(45.0, -15.0) + swipe(-15.0, 45.0) + DONE, 1.0),
]

for name, frames, want in cases:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

got, ok = run(EVEN + twist(-15.0) + DONE)
bad += not ok
print("ok  " if ok else "FAIL", "success once done and the arms home")
got, ok = run(EVEN + twist(-15.0) + [(40, dict(grippers=AWAY))])
bad += ok
print("ok  " if not ok else "FAIL", "no success with an arm away from home")
run(EVEN + twist(-15.0) + DONE)  # a reset clears the earlier trial: the connector back at the start, wire short
got, _ = run([(40, {})])
bad += got != 0.0 or abs(tk.backed_off_deg(None, m.NUT) - START_DEG) > 1e-9
print("ok  " if got == 0.0 else "FAIL", got, "after a reset the connector is back off the coil")
m.REQUIRE_ARMS_HOME = False
got, _ = run(EVEN + twist(-15.0) + [(40, dict(grippers=AWAY))])
m.REQUIRE_ARMS_HOME = True
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", got, "REQUIRE_ARMS_HOME off: done, arm away, scores 1.0")
print("all pass" if not bad else f"{bad} FAILED")
