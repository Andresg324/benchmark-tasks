"""Offline checks of the nut-starting task's scorer: step-by-step histories of the nut's turning (through the local
twistkit's thread-start model and its real grip accounting) and the grippers, with near misses. No simulator.

    python test_task.py        (with the yam_arena package importable)

Angles are the nut's unscrew angle in degrees from the washer (twistkit's u): it starts perched at START (4.5 turns),
drops onto the thread at START + 180, and is run down two turns at START + 180 - 720.
"""

import math
import os

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, tk = m.kit, m.tk
TOP = m.TOP
tk.register(*[o for o in m.OBJECTS if isinstance(o, tk.Threaded)])
PARK = {"left_gripper": (0.20, 0.27, TOP + 0.22), "right_gripper": (0.18, -0.30, TOP + 0.22)}
BASE = np.array([m.STUD_XY[0], m.STUD_XY[1], TOP])
START = -m.THREAD.start  # 1620
DROP = START + 180.0
TWO = DROP - 720.0  # two turns run down
ON = m.NUT_AF / (2 * abs(FINGER_OPEN))  # opening with the pads on the nut's flats
bad = 0


def st(pos, yaw=0.0):
    p = np.array(pos, float)
    a = math.radians(yaw) / 2
    return ObjectState(pos=p, quat_xyzw=np.array([0.0, 0.0, math.sin(a), math.cos(a)]), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=p - 0.01, high=p + 0.01)


def nut_pos():
    return BASE + (0, 0, m.SEAT + m.THREAD.pitch * math.degrees(tk.thread_height_u(m.NUT)) / 360.0)


def step(nut=None, held=False, grippers=None, grip_dz=None, grip_open=None, follow=True, holder="left_gripper"):
    """nut: its angle this step (as PhysX would report it); held: `holder` closed on the nut at `grip_dz` above its
    underside (default its middle), opening `grip_open` (default on its flats), its wrist rolled with the nut if
    `follow`; grippers: name -> position (fingers open) or (position, opening) or (position, opening, yaw)."""
    t = tk._track(m.NUT)
    if nut is not None:
        t.u = math.radians(nut)
    tk.start_model(m.NUT)
    objs = {"stud": st(BASE), m.NUT: st(nut_pos())}
    g = dict(PARK, **(grippers or {}))
    if held:
        yaw = math.degrees(t.u) if follow else 0.0
        dz = m.NUT_H / 2 if grip_dz is None else grip_dz
        g[holder] = (tuple(nut_pos() + (0, 0, dz)), ON if grip_open is None else grip_open, yaw)
    for k, v in g.items():
        pos, op, yaw = (v + (0.0,))[:3] if isinstance(v[0], tuple) else (v, 1.0, 0.0)
        objs[k] = st(pos, yaw)
        kit._OPENING[k] = op
    tk._account(objs)
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames):
    """frames: (steps, kwargs). A fresh trial each time."""
    kit._EPISODE["count"] += 1
    out = step()  # the trial's first scored step: arms at PARK, so home is PARK
    for n, kw in frames:
        for i in range(n):
            out = step(**(kw if i == 0 else {k: v for k, v in kw.items() if k != "nut"}))
    return out


def cur():
    return math.degrees(tk._track(m.NUT).u)


def twist(to, frm, held=True, by=15.0, **kw):
    """The nut turned from `frm` to `to` in 15-degree steps, held (a step of grip before and after) if `held`."""
    frames = [(1, dict(held=held, **kw))] if held else []
    a, sgn = frm, (-1 if to < frm else 1)
    while abs(a - to) > 1e-9:
        a = max(a - by, to) if sgn < 0 else min(a + by, to)
        frames.append((1, dict(nut=a, held=held, **kw)))
    return frames + ([(1, dict(held=held, **kw))] if held else [])


def swipe(to, frm):
    return twist(to, frm, held=False)


def regrasp(to, frm, stroke=150.0, **kw):
    """A regrasp loop like the demo's: open around the nut, close, roll `stroke`, open, roll back."""
    frames, a = [], frm
    sgn = -1 if to < frm else 1
    while abs(a - to) > 1e-9:
        n = min(stroke, abs(a - to))
        frames.append((1, dict(held=True, grip_open=1.0, **kw)))
        frames += twist(a + sgn * n, a, **kw)[1:-1]
        a += sgn * n
        frames.append((1, dict(held=True, grip_open=1.0, **kw)))
    return frames


BACK = twist(START + 215.0, START)  # turned back past the drop
RUN = twist(TWO - 30.0, START + 215.0)  # then run down a little over two turns
DONE = [(40, {})]
AWAY = {"right_gripper": (0.30, -0.15, TOP + 0.15)}
HOVER = {"left_gripper": ((m.STUD_XY[0], m.STUD_XY[1], TOP + m.SEAT + 0.018 + m.NUT_H / 2), ON + 0.15, 0.0)}

cases = [
    ("nothing", [(100, {})], 0.0),
    ("the trained way: back to the drop, two turns down, let go, arms home", BACK + RUN + DONE, 1.0),
    ("the same, an arm not home", BACK + RUN + [(40, dict(grippers=AWAY))], 0.9),
    ("the same with demo-like regrasp strokes", regrasp(START + 215.0, START, stroke=110.0)
     + regrasp(TWO - 30.0, START + 215.0) + DONE, 1.0),
    ("the same, the right arm doing it", twist(START + 215.0, START, holder="right_gripper")
     + twist(TWO - 30.0, START + 215.0, holder="right_gripper") + DONE, 1.0),
    ("turned forward straight away, no back-turn (the tempting wrong answer): jams", twist(START - 180.0, START)
     + DONE, 0.0),
    ("forward 45 degrees, then back to the drop and two turns down (under the 60-degree cross limit)",
     twist(START - 45.0, START) + twist(START + 215.0, START - 45.0) + RUN + DONE, 1.0),
    ("forward 75 degrees (cross-threaded), backed off, then started right: irreversible",
     twist(START - 75.0, START) + twist(START + 215.0, START - 75.0) + RUN + DONE, 0.0),
    ("turned back 170 degrees only (no drop), then forward", twist(START + 170.0, START)
     + twist(START - 120.0, START + 170.0) + DONE, 0.0),
    ("turned back 170 degrees only, then let go", twist(START + 170.0, START) + DONE, 0.0),
    ("dropped, then let go without running it down", BACK + DONE, 0.25),
    ("dropped, run down 0.9 turns", BACK + twist(DROP - 324.0, START + 215.0) + DONE, 0.25),
    ("dropped, run down 1.5 turns", BACK + twist(DROP - 540.0, START + 215.0) + DONE, 0.5),
    ("dropped, run down 1.95 turns", BACK + twist(DROP - 702.0, START + 215.0) + DONE, 0.5),
    ("dropped, run down to the washer (4.5 turns)", BACK + twist(DROP - 1620.0, START + 215.0) + DONE, 1.0),
    ("turned back a turn and a half (dropped, clicked over again), then two turns down from the last drop",
     twist(START + 540.0 + 30.0, START) + twist(TWO - 30.0, START + 570.0) + DONE, 1.0),
    ("turned back a turn and a half, then only two turns forward (one turn down)",
     twist(START + 570.0, START) + twist(START + 570.0 - 720.0, START + 570.0) + DONE, 0.5),
    ("done but the nut still held at the end", BACK + RUN
     + [(40, dict(grippers={"left_gripper": ((m.STUD_XY[0], m.STUD_XY[1], TOP + m.SEAT + 0.007), 0.5)}))], 0.0),
    ("done but a closed hand resting on the nut's top", BACK + RUN
     + [(40, dict(grippers={"left_gripper": ((m.STUD_XY[0], m.STUD_XY[1], TOP + m.SEAT + 0.009 + m.NUT_H + 0.01),
                                             0.0)}))], 0.0),
    ("done, an open hand 2 cm above the nut", BACK + RUN
     + [(40, dict(grippers={"left_gripper": ((m.STUD_XY[0], m.STUD_XY[1], TOP + m.SEAT + 0.009 + m.NUT_H + 0.02),
                                             1.0)}))], 0.9),
    ("done, arms at home but still moving", BACK + RUN
     + [(1, dict(grippers={"right_gripper": (0.18, -0.30 + 0.002 * (k % 2), TOP + 0.22)})) for k in range(40)], 0.9),
    # twistkit's grip accounting: swipes, hovering, a hand on the nut that does not turn it, laundering
    ("back-turn swiped (no grip), run down by hand", swipe(START + 215.0, START) + RUN + DONE, 0.0),
    ("back-turn by hand, run down swiped", BACK + swipe(TWO - 30.0, START + 215.0) + DONE, 0.0),
    ("open hand hovering around the nut (14 mm wider) while the other arm swipes it",
     [(1, dict(nut=a, grippers=HOVER)) for a in np.arange(START + 15, START + 215.0 + 1e-9, 15.0)] + RUN + DONE, 0.0),
    ("hand closed on the nut but its wrist not turning while the nut turns", twist(START + 215.0, START, follow=False)
     + twist(TWO - 30.0, START + 215.0, follow=False) + DONE, 0.0),
    ("hand closed 2 cm above the nut's top while it turns (not on it)",
     twist(START + 215.0, START, grip_dz=m.NUT_H + 0.02) + twist(TWO - 30.0, START + 215.0, grip_dz=m.NUT_H + 0.02)
     + DONE, 0.0),
    ("laundering: by hand down a turn, swiped back up, by hand down again, rest swiped",
     BACK + twist(DROP - 360.0, START + 215.0) + swipe(START + 215.0, DROP - 360.0)
     + twist(DROP - 360.0, START + 215.0) + swipe(TWO - 30.0, DROP - 360.0) + DONE, 0.0),
    ("done, then knocked 45 degrees each way (under the 60-degree allowance)", BACK + RUN
     + swipe(TWO + 15.0, TWO - 30.0) + swipe(TWO - 30.0, TWO + 15.0) + DONE, 1.0),
]

for name, frames, want in cases:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

# the model itself
run(twist(START - 180.0, START) + twist(START - 400.0, START - 180.0))
t = tk._track(m.NUT)
ok = t.crossed and not t.engaged
bad += not ok
print("ok  " if ok else "FAIL", "forced forward: cross-threaded, never engaged")
run(twist(START + 179.0, START))
h_before = tk.thread_height_u(m.NUT)
run(twist(START + 181.0, START))
h_after = tk.thread_height_u(m.NUT)
drop_mm = m.THREAD.pitch * math.degrees(h_before - h_after) / 360.0 * 1000
ok = 1.9 < drop_mm < 2.1 and tk.engaged(None, m.NUT)
bad += not ok
print("ok  " if ok else "FAIL", f"the drop at 180 degrees back is {drop_mm:.2f} mm (half the 4 mm pitch)")

got, ok = run(BACK + RUN + DONE)
bad += not ok
print("ok  " if ok else "FAIL", "success once done and the arms home")


def wiggle(to, frm, by=15.0):
    """The nut turned from `frm` to `to` by something else while the left hand, closed on its flats, rolls its wrist
    +`by` and -`by` degrees on alternate steps (net roll 0): the nut turns only on the + steps."""
    frames, a, yaw, sgn, k = [], frm, 0.0, (1 if to > frm else -1), 0
    while abs(a - to) > 1e-9:
        if k % 2 == 0:
            yaw += sgn * by
            a = min(a + by, to) if sgn > 0 else max(a - by, to)
        else:
            yaw -= sgn * by
        frames.append((1, dict(nut=a, wiggle=yaw)))
        k += 1
    return frames


_step = step


def step(nut=None, wiggle=None, **kw):  # noqa: F811 (adds the wiggling hand, placed on the nut each step)
    if wiggle is None:
        return _step(nut, **kw)
    t = tk._track(m.NUT)
    if nut is not None:
        t.u = math.radians(nut)
    tk.start_model(m.NUT)
    return _step(None, grippers={"left_gripper": (tuple(nut_pos() + (0, 0, m.NUT_H / 2)), ON, wiggle)})


# +-30 and +-45 degrees are the #1411 hole (wire-nut): a twistkit that clamps the roll budget to +-15 degrees both ways
# drops the counterclockwise roll on each wiggle back, so the other arm's swipe earned about half credit. Here the
# grip's net credit is bounded by the wrist's net roll (ROLL_NET_LAG), so the wiggle earns at most 1.25 x its size.
for by in (10.0, 15.0, 30.0, 45.0):
    got, _ = run(wiggle(START + 215.0, START, by) + wiggle(TWO - 30.0, START + 215.0, by) + DONE)
    credited = math.degrees(abs(tk._track(m.NUT).gripped))
    ok = got == 0.0 and credited <= tk.ROLL_GAIN * by + math.degrees(tk.ROLL_NET_LAG) + 1e-6
    bad += not ok
    print("ok  " if ok else "FAIL", f"{got:.2f} (want 0.0) a hand on the nut wiggling +-{by:.0f} degrees (net roll 0) "
          f"while something else turns it back and down: {credited:.1f} degrees credited, net")
# the nut spawns at its start height: the slide's joint is zeroed there (twistkit Threaded.lift_zero), so PhysX's default
# drive target 0 holds it perched while the scene settles instead of letting it sag to the stop and jump up
spec = m.OBJECTS[0]
lift = spec.links[0]
want_z = m.SEAT + m.THREAD.pitch * m.PERCH_TURNS
ok = (lift.start == 0.0 and abs(spec.lift_zero - m.THREAD.pitch * m.PERCH_TURNS) < 1e-12
      and abs(lift.anchor[2] - want_z) < 1e-12 and lift.lower < 0.0 < lift.upper)
bad += not ok
print("ok  " if ok else "FAIL", f"the slide's zero is the perch, {want_z * 1000:.1f} mm above the plate's foot")
got, ok = run(BACK + RUN + [(40, dict(grippers=AWAY))])
bad += ok
print("ok  " if not ok else "FAIL", "no success with an arm away from home")
run(BACK + RUN + DONE)  # a reset clears the earlier trial: the nut back perched, not engaged
got, _ = run([(40, {})])
t = tk._track(m.NUT)
ok = got == 0.0 and abs(cur() - START) < 1e-9 and not t.engaged and t.shift == 0.0 and not t.crossed
bad += not ok
print("ok  " if ok else "FAIL", got, "after a reset the nut is perched again and not engaged")
m.REQUIRE_ARMS_HOME = False
got, _ = run(BACK + RUN + [(40, dict(grippers=AWAY))])
m.REQUIRE_ARMS_HOME = True
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", got, "REQUIRE_ARMS_HOME off: done, arm away, scores 1.0")
print("all pass" if not bad else f"{bad} FAILED")
