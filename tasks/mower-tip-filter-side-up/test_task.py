"""Offline checks of the mower scorer: step-by-step mower poses and gripper states run through success() and score() the
way the simulator calls them, near misses included. No simulator.

    /data/IsaacLab-Arena/.venv/bin/python test_task.py      (prints "all pass")
"""

import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_spec = importlib.util.spec_from_file_location("mower_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit = m.kit
from yam_arena.task_file import ObjectState  # noqa: E402

HOME = {"left_gripper": np.array([0.323, 0.30, m.TOP + 0.26]), "right_gripper": np.array([0.323, -0.30, m.TOP + 0.26])}
AWAY = np.array([0.25, 0.20, m.TOP + 0.15])
START = np.array([m.X0, m.Y0, m.TOP])
fails = 0


def check(cond, what):
    global fails
    print(("ok  " if cond else "FAIL") + " " + what)
    fails += not cond


def quat(axis, deg):
    h = math.radians(deg) / 2
    v = {"x": (1, 0, 0), "y": (0, 1, 0)}[axis]
    return np.array([v[0] * math.sin(h), v[1] * math.sin(h), v[2] * math.sin(h), math.cos(h)])


def qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


YAWQ = np.array([0.0, 0.0, math.sin(m.YAW / 2), math.cos(m.YAW / 2)])


def pose(side=0.0, back=0.0):
    """Mower orientation as the sim reports it: the scene's yaw, then (in the mower's frame) `back` degrees front up
    and `side` degrees filter side up (negative: filter side down)."""
    return qmul(YAWQ, qmul(quat("x", back), quat("y", -side)))


def mower(q, pos=START, moving=False):
    p = np.asarray(pos, float)
    v = np.array([0.05, 0, 0]) if moving else np.zeros(3)
    return ObjectState(pos=p, quat_xyzw=q, lin_vel=v, ang_vel=np.zeros(3), low=p - [0.12, 0.12, 0.0],
                       high=p + [0.12, 0.12, 0.18])


def grip(p):
    p = np.asarray(p, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - [0, 0, 0.013], high=p)


def new_trial():
    kit._EPISODE["count"] += 1


def step(q, pos=START, left=None, closed=False, moving=False):
    kit._OPENING["left_gripper"] = 0.0 if closed else 1.0
    kit._OPENING["right_gripper"] = 1.0
    objs = {"mower": mower(q, pos, moving), "left_gripper": grip(HOME["left_gripper"] if left is None else left),
            "right_gripper": grip(HOME["right_gripper"])}
    ok = m.success(objs)
    return m.score(objs), ok


def run(phases):
    """Phases of (steps, side_from, side_to, back_from, back_to, extra kwargs); returns the last (score, success)."""
    out = None
    for n, s0, s1, b0, b1, kw in phases:
        for k in range(n):
            f = (k + 1) / n
            out = step(pose(s0 + (s1 - s0) * f, b0 + (b1 - b0) * f), **kw)
    return out


HELD = {"left": START + [0.0, 0.05, 0.07], "closed": True}
TIP = [(10, 0, 0, 0, 0, {}), (40, 0, 78, 0, 0, HELD), (10, 78, 92, 0, 0, {"left": AWAY}), (30, 92, 92, 0, 0, {})]

# --- The port's geometry: 25 degrees with the filter side or the handle down, never the right way up to 150.
check(not m.port_under(mower(pose(-24.5))) and m.port_under(mower(pose(-25.5))), "filter side down: port under at 25 deg")
check(not m.port_under(mower(pose(0, 24.5))) and m.port_under(mower(pose(0, 25.5))), "handle down: port under at 25 deg")
check(not m.port_under(mower(pose(0, -60))), "front down (nose-over) 60 deg: port stays up")
check(all(not m.port_under(mower(pose(a))) for a in range(0, 151, 2)), "filter side up 0-150 deg: port never under")
check(m.port_under(mower(pose(170))), "upside down: port under")
check(abs(m.filter_side_up_deg(mower(pose(92))) - 92) < 1e-6 and abs(m.twist_deg(mower(pose(0, 30))) - 30) < 1e-6,
      "angle readings")

# --- The job done right.
new_trial()
s, ok = run(TIP)
check(s == 1.0 and ok, f"filter side up, resting, let go, arms home: 1.0 ({s})")
new_trial()
s, _ = run(TIP[:-1] + [(30, 92, 92, 0, 0, {"left": AWAY})])
check(s == 0.9, f"same, left arm parked away from home: 0.9 ({s})")
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (40, 0, 74, 0, 0, HELD), (30, 74, 74, 0, 0, {})])
check(s == 1.0, f"resting at 74 deg (just past 70): 1.0 ({s})")

# --- Do nothing, half done, held, moving, dragged, upside down, twisted.
new_trial()
s, _ = run([(100, 0, 0, 0, 0, {})])
check(s == 0.0, f"do nothing: 0 ({s})")
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (40, 0, 60, 0, 0, HELD), (30, 60, 60, 0, 0, {})])
check(s == 0.0, f"left at 60 deg (propped, not on its side): 0 ({s})")
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (40, 0, 92, 0, 0, HELD), (30, 92, 92, 0, 0, HELD)])
check(s == 0.0, f"on its side but still gripped at the end: 0 ({s})")
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (40, 0, 92, 0, 0, HELD), (30, 92, 92, 0, 0, {"left": START + [0.0, 0.05, 0.07],
                                                                                "closed": True, "moving": False})])
check(s == 0.0, "closed fingers on the lug at the end: 0")
new_trial()
s, _ = run(TIP[:-1] + [(30, 92, 92, 0, 0, {"moving": True})])
check(s == 0.0, f"still sliding or rocking: 0 ({s})")
new_trial()
s, _ = run(TIP[:-1] + [(30, 92, 92, 0, 0, {"pos": START + [-0.16, 0.16, 0.0]})])
check(s == 0.0, f"dragged 23 cm across the bench: 0 ({s})")
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (40, 0, 175, 0, 0, HELD), (30, 175, 175, 0, 0, {})])
check(s == 0.0 and m.flooded(), f"rolled on over upside down: 0, flooded ({s})")
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (40, 0, 92, 0, 0, HELD), (10, 92, 92, 0, 22, HELD), (30, 92, 92, 22, 22, {})])
check(s == 0.0, f"on its side but twisted 22 deg about its side: 0 ({s})")

# --- The wrong ways: filter side down, corrected after, handle down.
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (40, 0, -85, 0, 0, HELD), (30, -85, -85, 0, 0, {})])
check(s == 0.0 and m.flooded(), f"tipped filter side down onto its side: 0 ({s})")
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (12, 0, -35, 0, 0, HELD), (8, -35, -35, 0, 0, HELD), (12, -35, 0, 0, 0, HELD)] + TIP[1:])
check(s == 0.0 and m.flooded(), f"filter side down to 35 deg for 0.4 s, set back, then tipped right: 0 ({s}, "
      f"{m.oil_in_filter():.1f} mL)")
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (30, 0, 0, 0, 55, HELD), (30, 0, 0, 55, 55, {})])
check(s == 0.0 and m.flooded(), f"tipped back on its handle: 0 ({s})")
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (20, 0, 0, 0, 20, HELD), (20, 0, 0, 20, 0, HELD)] + TIP[1:])
check(s == 1.0 and not m.flooded(), f"handle pressed down 20 deg (under 25) and back, then tipped right: 1.0 ({s})")

# --- Thresholds: a one-step pop does not count; any confirmed dip (2 steps in a row) soaks the filter.
for under, want in ((1, 1.0), (2, 0.0), (9, 0.0)):
    new_trial()
    ph = [(10, 0, 0, 0, 0, {}), (under, -30, -30, 0, 0, HELD), (5, 0, 0, 0, 0, HELD)] + TIP[1:]
    s, _ = run(ph)
    check(s == want, f"filter side down 30 deg for {under} steps, then tipped right: {want} ({s}, "
          f"{m.oil_in_filter():.2f} mL)")
new_trial()
for k in range(6):  # three 1-step pops: each shorter than the 2-step confirm, so nothing counts
    step(pose(-30 if k % 2 == 0 else 0), **HELD)
check(m.oil_in_filter() == 0.0 and not m.flooded(), "isolated one-step pops add no oil")

# --- The reviewer's flick: filter side down to 60 degrees and back within 9 steps, then the right tip: 0.
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (4, 0, -60, 0, 0, HELD), (5, -60, 0, 0, 0, HELD)] + TIP[1:])
check(s == 0.0 and m.flooded(), f"fast flick to 60 deg filter side down, then tipped right: 0 ({s})")
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (3, 0, -24, 0, 0, HELD), (5, -24, -24, 0, 0, HELD), (3, -24, 0, 0, 0, HELD)] + TIP[1:])
check(s == 1.0 and not m.flooded(), f"filter side down to 24 deg (under 25) and back, then tipped right: 1.0 ({s})")

# --- Travel is measured at the deck's center: a clean tip moves it about 12 cm; 20 cm is the limit.
new_trial()
s, _ = run(TIP)
c0 = np.array([m.X0, m.Y0, m.TOP + m.DECK_C[2]])
travel = np.linalg.norm(m.deck_center(mower(pose(92)))[:2] - c0[:2])
check(s == 1.0 and abs(travel - m.DECK_C[2] * math.sin(math.radians(92))) < 1e-6,
      f"travel is read at the deck's center (on its side in place, the center sits {travel * 100:.1f} cm over)")
new_trial()
s, _ = run(TIP[:-1] + [(30, 92, 92, 0, 0, {"pos": START + [0.0, 0.12, 0.0]})])
check(s == 1.0, f"origin moved 12 cm (a normal tip's deck travel): 1.0 ({s})")
new_trial()
s, _ = run(TIP[:-1] + [(30, 92, 92, 0, 0, {"pos": START + [0.0, 0.21, 0.0]})])
check(s == 0.0, f"moved 21 cm: dragged, 0 ({s})")

# --- A new trial starts clean.
new_trial()
s, _ = run([(10, 0, 0, 0, 0, {}), (20, 0, -40, 0, 0, HELD)])
new_trial()
s, _ = run(TIP)
check(s == 1.0, f"a flood in the trial before does not carry over: 1.0 ({s})")

# --- The demo's plan: the lug is gripped at the start and the arc stays inside the arms' reach and lift.
p0 = m.lug_tcp(0.0)
lug = m.to_world(m.LUG_C)
check(abs(p0[0] - lug[0]) < 1e-9 and abs(p0[1] - lug[1]) < 1e-9 and p0[2] - m.TOP < 0.075 and lug[1] < m.Y0,
      "demo grips the filter-side lug, on the robot's right")
peak = max(m.lug_tcp(math.radians(a))[2] - m.TOP for a in range(0, 79))
check(peak < 0.22, f"demo's arc peaks {peak * 100:.1f} cm above the table (lift limit 22 cm)")
check(all(0.15 <= m.lug_tcp(math.radians(a))[0] <= 0.40 and abs(m.lug_tcp(math.radians(a))[1]) < 0.30 for a in range(0, 79)),
      "demo's arc within x 0.15-0.40, |y| < 0.30")

print("all pass" if fails == 0 else f"{fails} FAILED")
