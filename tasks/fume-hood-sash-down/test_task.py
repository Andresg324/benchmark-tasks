"""Offline checks of the fume hood's scorer: step-by-step joint histories, no simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
PARK = {"left_gripper": (0.20, 0.27, 0.95), "right_gripper": (0.18, -0.26, 0.95)}


def st(pos):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def step(sash, light=0.0, fan=0.0, grippers=PARK):
    """One control step: set the three joints, call success() (which records the history) and return score()."""
    kit._JOINTS["hood/sash"] = kit.JointState(sash, 0.0, False)
    kit._JOINTS["hood/light"] = kit.JointState(light, 0.0, False)
    kit._JOINTS["hood/fan"] = kit.JointState(fan, 0.0, False)
    objs = {"hood": st(m.BASE), **{g: st(p) for g, p in grippers.items()}}
    m.success(objs)
    return m.score(objs), m.success(objs)


def run(frames):
    """frames: (steps, sash, light, fan). A fresh trial each time (the kit's reset counter moves on)."""
    kit._EPISODE["count"] += 1
    out = (0.0, False)
    for n, sash, light, fan in frames:
        for _ in range(n):
            out = step(sash, light, fan)
    return out


PRESS = -m.TRAVEL  # a button pushed all the way
OPEN, MARKED, SHUT = m.OPEN, m.MARK - 0.01, 0.0


def press(button, sash):
    return [(3, sash, PRESS if button == "light" else 0.0, PRESS if button == "fan" else 0.0), (3, sash, 0.0, 0.0)]


CASES = [
    ("nothing done", [(20, OPEN, 0, 0)], 0.0),
    ("sash lowered to the working mark only", [(20, MARKED, 0, 0)], 0.25),
    ("sash at the mark, light off", press("light", MARKED) + [(10, MARKED, 0, 0)], 0.5),
    ("sash closed, light left on", [(20, SHUT, 0, 0)], 0.75),
    ("sash closed, light off", press("light", SHUT) + [(10, SHUT, 0, 0)], 1.0),
    ("light off but sash left open", press("light", OPEN) + [(10, OPEN, 0, 0)], 0.0),
    ("everything off (the tempting answer)", press("light", SHUT) + press("fan", SHUT) + [(10, SHUT, 0, 0)], 0.0),
    ("fan off then back on", press("fan", SHUT) + press("fan", SHUT) + press("light", SHUT) + [(10, SHUT, 0, 0)], 0.0),
    ("light pressed twice (back on)", press("light", SHUT) + press("light", SHUT) + [(10, SHUT, 0, 0)], 0.75),
    ("button half pushed (not a press)", [(5, SHUT, -0.003, 0), (5, SHUT, 0, 0)], 0.75),
    ("sash 4 mm above the sill (counts as closed)", press("light", 0.004) + [(10, 0.004, 0, 0)], 1.0),
    ("sash 1 cm above the sill (not closed)", press("light", 0.010) + [(10, 0.010, 0, 0)], 0.5),
    ("sash exactly at the 7 cm mark", [(20, m.MARK, 0, 0)], 0.25),
    ("sash 3 mm above the mark", press("light", m.MARK + 0.003) + [(10, m.MARK + 0.003, 0, 0)], 0.0),
]

bad = 0
for name, frames, want in CASES:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

# a fan press in one trial does not carry into the next
run(press("fan", SHUT))
got, _ = run(press("light", SHUT) + [(10, SHUT, 0, 0)])
ok = got == 1.0
bad += not ok
print(("ok  " if ok else "FAIL"), "a new trial starts with a clean history")

# Arms home: full score and success need both tool points back within 3 cm of their start (the first step), and still.
handle = (m.HANDLE_X, -0.05, m.TOP + m.SILL_TOP + 0.01)
away = {"left_gripper": PARK["left_gripper"], "right_gripper": handle}
kit._EPISODE["count"] += 1
for f in press("light", SHUT):
    for _ in range(f[0]):
        step(f[1], f[2], f[3])
step(SHUT, grippers=away)
sc_away, su_away = step(SHUT, grippers=away)
step(SHUT)
sc_home, su_home = step(SHUT)
checks = [(sc_away == 0.9 and not su_away, "done, a gripper left at the handle: 0.9, no success"),
          (sc_home == 1.0 and su_home, "done and arms back home: 1.0 and success")]
# A reset clears the recorded home: the next trial records its own first position.
kit._EPISODE["count"] += 1
step(OPEN, grippers=away)  # trial starts with the right gripper at the handle: that is its home
for f in press("light", SHUT):
    for _ in range(f[0]):
        step(f[1], f[2], f[3], grippers=away)
sc_reset, _ = step(SHUT, grippers=away)
checks.append((sc_reset == 1.0, "a reset clears the recorded home"))
m.REQUIRE_ARMS_HOME = False
kit._EPISODE["count"] += 1
step(OPEN)
for f in press("light", SHUT):
    for _ in range(f[0]):
        step(f[1], f[2], f[3])
step(SHUT, grippers=away)
sc_off, su_off = step(SHUT, grippers=away)
m.REQUIRE_ARMS_HOME = True
checks.append((sc_off == 1.0 and su_off, "with REQUIRE_ARMS_HOME off, the job alone is 1.0"))
for flag, name in checks:
    bad += not flag
    print(("ok  " if flag else "FAIL"), name)
print("all pass" if not bad else f"{bad} FAILED")
raise SystemExit(1 if bad else 0)
