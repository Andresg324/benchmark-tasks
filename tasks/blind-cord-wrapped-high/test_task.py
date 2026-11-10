"""Offline tests for blind-cord-wrapped-high: scoring cases and the local cablekit fixes. Run: python3 test_task.py"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402
import cablekit as ck  # noqa: E402
import task as T  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

TOP = T.TOP
ck.register(T.CORD)
KEY = "cord/tassel"
FIT = T.CORD.end
N = T.CORD.segments + 1


def st(pos, low=None, high=None, q=(0.0, 0.0, 0.0, 1.0)):
    pos = np.asarray(pos, float)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos - 0.01 if low is None else np.asarray(low, float),
                       high=pos + 0.01 if high is None else np.asarray(high, float))


HOME = {"left_gripper": st((0.05, 0.30, TOP + 0.26)), "right_gripper": st((0.05, -0.30, TOP + 0.26))}


def scene(pts, tassel_low=None, grippers=None, open_=1.0):
    """objects as score() sees them: the cord along pts, the tassel hanging 2 cm below its last point."""
    pts = np.asarray(pts, float)
    end = pts[-1]
    tl = end[2] - 0.045 if tassel_low is None else tassel_low
    objs = {"cord": ck.state_from_points("cord", pts), KEY: st(end - [0, 0, 0.02], low=(end[0] - .01, end[1] - .01, tl))}
    objs.update(HOME if grippers is None else grippers)
    kit._OPENING.update({"left_gripper": open_, "right_gripper": open_})
    return objs


def resample(path, n=N):
    """n points evenly along a polyline."""
    path = np.asarray(path, float)
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1))]
    s = np.linspace(0, d[-1], n)
    return np.array([np.interp(s, d, path[:, k]) for k in range(3)]).T


def coil(c, turns, r=0.022, start=math.pi / 2, k=60):
    """Points circling a cleat's post (radius r, in the y-z plane just off the wall), `turns` times."""
    a = start - np.linspace(0, 2 * math.pi * turns, k)
    return np.c_[np.full(k, c[0] - 0.012), c[1] + 1.4 * r * np.cos(a), c[2] + r * np.sin(a)]


A = T.ANCHOR


def wrapped(c=T.CLEAT, n_turns=2.0, tail=0.0):
    """Anchor -> top of the cleat -> n turns -> a tail hanging `tail` below the cleat."""
    path = [A, (c[0] - 0.012, c[1], c[2] + 0.03)]
    path += list(coil(c, n_turns))
    end = path[-1]
    if tail:
        path.append((end[0], end[1], c[2] - tail))
    return resample(path)


def hanging():
    """The starting state: straight down from the anchor and along the floor (table) toward the robot."""
    return T.CORD.nodes()


def check(name, got, want):
    assert abs(got - want) < 1e-9, f"{name}: got {got}, want {want}"
    print(f"ok  {name}: {got}")


# -- scoring --------------------------------------------------------------------------------------------
# job(): the score before the arms-home check
check("do nothing (cord hanging to the floor)", T.job(scene(hanging())), 0.0)
w = wrapped()
check("two turns on the high cleat, tassel tucked up, hands off", T.job(scene(w, tassel_low=T.CLEAT[2] - 0.03)), 1.0)
check("one turn on the high cleat counts", T.job(scene(wrapped(n_turns=1.1), tassel_low=T.CLEAT[2] - 0.03)), 1.0)
check("half a turn (draped over the high cleat) = 0", T.job(scene(wrapped(n_turns=0.5), tassel_low=T.CLEAT[2] - .03)),
      0.0)
check("wrapped on the low cleat = 0", T.job(scene(wrapped(T.LOW), tassel_low=T.LOW[2] - 0.03)), 0.0)
both = resample([A, *coil(T.CLEAT, 1.5), (T.CLEAT[0] - 0.012, T.LOW[1] - 0.04, T.LOW[2] + 0.02), *coil(T.LOW, 1.0)])
check("high wrap but the tail tied off on the low cleat = 0", T.job(scene(both, tassel_low=T.LOW[2] - 0.03)), 0.0)
loop = resample([A, (A[0], A[1], TOP + 0.02), (A[0], T.CLEAT[1], TOP + 0.02), (A[0], T.CLEAT[1], T.CLEAT[2] - 0.03),
                 *coil(T.CLEAT, 2.0)])
check("wrapped high but a loop hangs below the line = 0.25", T.job(scene(loop, tassel_low=T.CLEAT[2] - 0.03)), 0.25)
check("wrapped high but the tassel hangs below the line = 0.25",
      T.job(scene(wrapped(tail=0.06), tassel_low=T.CLEAT[2] - 0.10)), 0.25)
g_near = dict(HOME, right_gripper=st(w[-1] + [0, 0, 0.01]))
check("wrapped and the gripper still holds the cord at the end = 0",
      T.job(scene(w, tassel_low=T.CLEAT[2] - 0.03, grippers=g_near, open_=0.2)), 0.0)
check("wrapped, gripper open but still touching the cord = 0",
      T.job(scene(w, tassel_low=T.CLEAT[2] - 0.03, grippers=g_near, open_=1.0)), 0.0)
cleat_only = resample([*coil(T.CLEAT, 2.0)])
check("winding outside the cleat zone does not count (big loop around it)",
      abs(T.turns(resample([(T.CLEAT[0] - .01, T.CLEAT[1] + .12 * math.cos(a), T.CLEAT[2] + .12 * math.sin(a))
                            for a in np.linspace(0, 4 * math.pi, 80)]), T.CLEAT)), 0.0)
check("two coils behind the horns = 2 turns", T.turns(cleat_only, T.CLEAT), 2.0)
C = T.CLEAT


def ring(out, ry, rz, n_turns=1.0, k=80, sign=1):
    """A loop around the high cleat's post, `out` from the wall, half-widths ry (along the horns) and rz."""
    a = math.pi / 2 - sign * np.linspace(0, 2 * math.pi * n_turns, k)
    return np.c_[np.full(k, C[0] - out), C[1] + ry * np.cos(a), C[2] + rz * np.sin(a)]


# a turn has to pass behind the horns (between them and the wall), not just sweep round the post's axis
check("loop in front of the horn tips (4.5 cm out, 3 cm radius) = 0 turns", T.turns(ring(0.045, 0.03, 0.03), C), 0.0)
check("loop lying on the horns' front face (3.1 cm out) = 0 turns", T.turns(ring(0.031, 0.03, 0.03), C), 0.0)
check("loop past the horn tips, behind them (1 cm out, 4.5 cm radius) = 0 turns", T.turns(ring(0.01, 0.045, 0.03), C),
      0.0)
check("round turn behind the horns (1.2 cm out) = 1", T.turns(ring(0.012, 0.03, 0.02), C), 1.0)
check("round turn the other way round = 1", T.turns(ring(0.012, 0.03, 0.02, sign=-1), C), 1.0)
check("tall loop behind the horns reaching 5.5 cm above and below the post = 1",
      T.turns(ring(0.012, 0.03, 0.055), C), 1.0)
check("two tall turns, 2 cm links (resampled) = 2", T.turns(resample(ring(0.012, 0.03, 0.055, 2.0)), C), 2.0)
t8 = np.linspace(0, 4 * math.pi, 160)  # figure-eight (the cleat hitch): behind one horn, across the post's front,
x8 = np.where(np.abs(np.sin(t8)) > 0.6, C[0] - 0.012, C[0] - 0.035)  # behind the other horn, across again; twice
eight = np.c_[x8, C[1] + 0.03 * np.sin(t8), C[2] + 0.025 * np.sin(2 * t8)]
check("two figure-eights behind the horns = 2 (the sweep cancels but each horn is passed twice)", T.turns(eight, C), 2.0)
eight_front = eight.copy()
eight_front[:, 0] = C[0] - 0.035
check("the same figure-eight in front of the horns = 0", T.turns(eight_front, C), 0.0)
hairpin = np.array([(C[0] - 0.012, C[1] + 0.025, C[2] + 0.02), (C[0] - 0.012, C[1] + 0.025, C[2] - 0.02),
                    (C[0] - 0.012, C[1] + 0.02, C[2] + 0.02)])
check("hairpin behind one horn (down and back up) = 0", T.turns(hairpin, C), 0.0)

# -- the local cablekit fixes ---------------------------------------------------------------------------
# 1) an empty closed gripper next to the tassel does not hold it; a gripper closed on the 18 mm tassel does
s = st((0.30, -0.15, TOP + 0.10), q=(0.0, 0.0, 0.0, 1.0))
objs = {KEY: s, "right_gripper": st(s.pos + ck._rotm_xyzw(s.quat_xyzw) @ (FIT.dz + [0.02, 0, 0]))}
kit._OPENING["right_gripper"] = 0.0
assert not ck._holds(objs, "right_gripper", s, FIT), "closed empty fist counted as holding"
kit._OPENING["right_gripper"] = FIT.size[1] / ck.FULL_OPEN  # fingers stopped on the tassel's 18 mm
assert ck._holds(objs, "right_gripper", s, FIT)
kit._OPENING["right_gripper"] = 1.0
assert not ck._holds(objs, "right_gripper", s, FIT)
print("ok  cablekit: an empty closed gripper never holds; one closed on the tassel does")

# 2) attach targets are boxes in the target body's frame, not spheres
t = ck.Target("slot", body="b", point=(0, 0, 0), size=(0.04, 0.01, 0.01))
yaw90 = np.array([[0.0, -1, 0], [1, 0, 0], [0, 0, 1]])
assert ck.in_target(t, (0.018, 0, 0), (0, 0, 0))  # along the long side: inside
assert not ck.in_target(t, (0, 0.018, 0), (0, 0, 0))  # same distance across: outside (a sphere would accept both)
assert ck.in_target(t, (0, 0.018, 0), (0, 0, 0), yaw90)  # body turned 90 degrees: the box turns with it
assert not ck.in_target(t, (0.018, 0, 0), (0, 0, 0), yaw90)
assert ck.in_target(ck.Target("r", body=None, point=(0, 0, 0), radius=0.01), (0.009, 0.009, 0.009), (0, 0, 0))
print("ok  cablekit: targets are boxes in the target's frame")

# 3) the scorer's drift checks are 3D: the hands-off distance and the 1.2 m line use full 3D points, so a gripper
#    beside the cord at the same height (only y apart) still counts as on it
g_side = dict(HOME, right_gripper=st(w[-1] + [0, 0.03, 0]))
check("gripper 3 cm beside the cord at the same height = still hands on (0)",
      T.job(scene(w, tassel_low=T.CLEAT[2] - 0.03, grippers=g_side)), 0.0)
line = resample([(0.30, -0.25, TOP + 0.12), (0.30, -0.05, TOP + 0.12)])
mid = line[N // 2]
assert not T.hands_on(scene(line, grippers=dict(HOME, right_gripper=st(mid + [-0.03, 0, 0.03]))))  # 4.2 cm in 3D
assert T.hands_on(scene(line, grippers=dict(HOME, right_gripper=st(mid + [-0.02, 0, 0.03]))))  # 3.6 cm in 3D
print("ok  hands-off distance is measured in 3D")

# -- arms home, settling and the per-trial history ----------------------------------------------------------
good = scene(w, tassel_low=T.CLEAT[2] - 0.03)
T._H = T._History()
T._update(good)  # first step: grippers recorded where they start (HOME)
T._update(good)
check("done, cord settled, arms home and still", T.score(good), 1.0)
assert T.success(good)
away = dict(good, right_gripper=st((0.20, -0.25, TOP + 0.15)))
T._update(away)
T._update(away)
check("done but the right arm not back home = 0.9", T.score(away), 0.9)
assert not T.success(away)
moving = scene(w + [0, 0, 0.005], tassel_low=T.CLEAT[2] - 0.03)
assert not T.success(moving) and not T._H.cord_still, "a cord still sliding (5 mm per step) counts as settled"
check("done but the cord still sliding (5 mm per step) at the end = 0", T.score(moving), 0.0)
print("ok  success() needs the cord settled (3D drift under 2 mm per step) and the arms home")
kit._EPISODE["count"] += 1
T._update(away)  # new trial: home is wherever the grippers are now
assert T._H.episode == kit.episode() and np.allclose(T._H.home["right_gripper"], away["right_gripper"].pos)
print("ok  kit.episode() resets the scorer's history")

# 4) kit.episode(): a new trial resets the cord's attach/held history
ck.set_held(KEY, True)
assert ck.held({KEY: s}, KEY)
kit._EPISODE["count"] += 1
assert not ck.held({KEY: s}, KEY), "held state survived a scene reset"
print("ok  kit.episode() starts each trial afresh")
print("all pass")
