"""Offline checks of the eyewash task's scorer: step-by-step histories of the flag (valve), the two dust covers and the
grippers, traps and near misses included. No simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
TOP = m.TOP
PARK = {"left_gripper": (0.20, 0.27, TOP + 0.22), "right_gripper": (0.18, -0.30, TOP + 0.22)}
OPEN, SHUT, CAP_OFF = m.FLAG_OPEN, 0.0, m.COVER_OPEN


def st(pos):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0.0, 0, 0, 1]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def step(flag, left=0.0, right=0.0, grippers=None):
    """One control step: the flag's and covers' joint angles, and gripper tool points (default: parked at home)."""
    kit._JOINTS["eyewash/flag"] = kit.JointState(flag, 0.0, False)
    kit._JOINTS["eyewash/cover_left"] = kit.JointState(left, 0.0, False)
    kit._JOINTS["eyewash/cover_right"] = kit.JointState(right, 0.0, False)
    g = dict(PARK, **(grippers or {}))
    objs = {"eyewash": st(m.BASE)}
    for k, v in g.items():
        objs[k] = st(v)
        kit._OPENING[k] = 1.0
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames, new_trial=True):
    """frames: (steps, flag, left cover, right cover[, grippers]). A fresh trial unless new_trial is False."""
    if new_trial:
        kit._EPISODE["count"] += 1
    out = step(SHUT)  # the trial's first scored step: the arms at their ready pose (PARK), so home is PARK
    for fr in frames:
        n, flag, left, right = fr[:4]
        g = fr[4] if len(fr) > 4 else None
        for _ in range(n):
            out = step(flag, left, right, g)
    return out


N = m.FLUSH_STEPS
on_flag = {"right_gripper": tuple(m._flag_center(SHUT))}
on_cap = {"left_gripper": tuple(m._cap_center(m.HEAD_Y, 0.0) + np.array([0, 0, 0.02]))}
near_flag = {"right_gripper": tuple(m._flag_center(SHUT) + np.array([0.0, -0.05, 0.0]))}
away = {"right_gripper": (0.25, -0.25, TOP + 0.15)}

CASES = [
    ("nothing done", [(300, SHUT, 0, 0)], 0.0),
    ("covers flipped open by hand, valve never opened", [(300, SHUT, CAP_OFF, CAP_OFF)], 0.0),
    ("momentary press: open 1 s, shut, covers back on", [(20, OPEN, CAP_OFF, CAP_OFF), (5, SHUT, CAP_OFF, CAP_OFF),
                                                        (100, SHUT, 0, 0)], 0.0),
    ("open 11.95 s (one step short), shut, covers on", [(N - 1, OPEN, CAP_OFF, CAP_OFF), (100, SHUT, 0, 0)], 0.0),
    ("two 8 s spells with a 1-step break (not continuous)", [(160, OPEN, CAP_OFF, CAP_OFF), (1, SHUT, CAP_OFF, CAP_OFF),
                                                             (160, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, 0, 0)], 0.0),
    ("flag held at 79 degrees for 20 s (never fully open), shut, covers on",
     [(400, math.radians(79), CAP_OFF, CAP_OFF), (50, SHUT, 0, 0)], 0.0),
    ("full flush, valve left running", [(400, OPEN, CAP_OFF, CAP_OFF)], 0.25),
    ("full flush, flag left part open at 30 degrees", [(N, OPEN, CAP_OFF, CAP_OFF), (50, math.radians(30), 0, 0)], 0.25),
    ("full flush, valve left running, covers pushed back on", [(N, OPEN, CAP_OFF, CAP_OFF), (50, OPEN, 0, 0)], 0.25),
    ("full flush, shut, both covers left off", [(N, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, CAP_OFF, CAP_OFF)], 0.5),
    ("full flush, shut, left cover left off", [(N, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, CAP_OFF, 0)], 0.5),
    ("full flush, shut, right cover half shut (45 degrees)", [(N, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, 0, math.pi / 4)],
     0.5),
    ("full flush, shut, covers on, a gripper still on the flag",
     [(N, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, 0, 0, on_flag)], 0.5),
    ("full flush, shut, covers on, a gripper still pressing a cap",
     [(N, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, 0, 0, on_cap)], 0.5),
    ("full flush, shut, covers on, a gripper 5 cm from the flag (hands off, not home)",
     [(N, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, 0, 0, near_flag)], 0.9),
    ("full flush, shut, covers on, an arm parked away from home", [(N, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, 0, 0, away)],
     0.9),
    ("full flush (exactly 12 s), shut, covers on, arms home", [(N, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, 0, 0)], 1.0),
    ("covers 9 degrees open count as on", [(N, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, math.radians(9), 0)], 1.0),
    ("short press, then a full flush later, all restored",
     [(20, OPEN, CAP_OFF, CAP_OFF), (40, SHUT, 0, 0), (N + 20, OPEN, CAP_OFF, CAP_OFF), (60, SHUT, 0, 0)], 1.0),
]

bad = 0
for name, frames, want in CASES:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

# success() needs the full score; a reset starts a clean history (no flush carried over into the next trial).
sc, su = run([(N, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, 0, 0)])
sc3, _ = run([(50, SHUT, 0, 0)], new_trial=False)  # no reset: the same trial, still flushed
sc2, su2 = run([(50, SHUT, 0, 0)])  # the next trial: nothing done
run([(N, OPEN, CAP_OFF, CAP_OFF)])  # half a flush counted in one trial ...
sc4, _ = run([(20, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, 0, 0)])  # ... does not carry its open run into the next
m.REQUIRE_ARMS_HOME = False
sc5, su5 = run([(N, OPEN, CAP_OFF, CAP_OFF), (50, SHUT, 0, 0, away)])
m.REQUIRE_ARMS_HOME = True
for flag, name in ((sc == 1.0 and su, "done and arms home: 1.0 and success"),
                   (sc2 == 0.0 and not su2, "a reset clears the flush: the next trial doing nothing scores 0"),
                   (sc3 == 1.0, "without a reset the history carries on"),
                   (sc4 == 0.0, "a reset clears the open-run counter"),
                   (sc5 == 1.0 and su5, "with REQUIRE_ARMS_HOME off, the job alone is 1.0")):
    bad += not flag
    print(("ok  " if flag else "FAIL"), name)
print("all pass" if not bad else f"{bad} FAILED")
raise SystemExit(1 if bad else 0)
