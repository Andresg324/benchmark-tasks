"""Offline checks of the waste station's scorer: hand-made object states, no simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit


def st(pos, tilt=0.0, vel=0.0, low=None, high=None):
    """A state turned `tilt` radians about x. `low`/`high` default to a 1 cm box around `pos`."""
    q = (math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2))
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q), lin_vel=np.array([vel, 0, 0]), ang_vel=np.zeros(3),
                       low=p - 0.01 if low is None else np.array(low, float),
                       high=p + 0.01 if high is None else np.array(high, float))


def funnel_up(x, y, z, vel=0.0):
    """An upright funnel with its origin at (x, y, z), with the outline of its visible model."""
    r = m.CONE_TOP_R
    tab_x = m.TAB_C[0] - m.TAB[0] / 2
    return st((x, y, z), vel=vel, low=(x + tab_x, y - r, z - m.STEM_L), high=(x + r, y + r, z + m.CONE_H))


def funnel_lying(x, y, floor_z):
    """A funnel on its side, its axis along y, lying on a floor at floor_z."""
    r = m.CONE_TOP_R
    tab_x = m.TAB_C[0] - m.TAB[0] / 2
    return st((x, y, floor_z + r), tilt=math.pi / 2, low=(x + tab_x, y - m.CONE_H, floor_z),
              high=(x + r, y + m.STEM_L, floor_z + 2 * r))


FAR = ((0.16, 0.26, m.TOP + 0.20), (0.16, -0.26, m.TOP + 0.20))
OPEN = (1.0, 1.0)


def scene(funnel, cap, grippers=FAR, opening=OPEN):
    """The objects, and the two grippers' openings (set into the kit when the case is scored)."""
    return ({"funnel": funnel, "cap": cap, "left_gripper": st(grippers[0]), "right_gripper": st(grippers[1]),
             "jug": st((m.JUG_X, m.JUG_Y, m.TOP)), "containment": st((m.JUG_X, m.JUG_Y, m.TOP)),
             "drip_tray": st((m.TRAY_X, m.TRAY_Y, m.TOP))}, opening)


def use(case):
    objs, opening = case
    kit._OPENING["left_gripper"], kit._OPENING["right_gripper"] = opening
    return objs


def run(case):
    """A fresh trial: the first scored step with both arms at FAR (their start), then the case held for 3 steps
    (success() first each step, as the runner does). Returns the final score."""
    m._H = m._History()
    m.success(use(scene(F_IN, C_START)))
    objs = use(case)
    for _ in range(3):
        m.success(objs)
    return m.score(objs)


SEAT_F = m.HOLDER_TOP + m.BASE_T / 2
F_IN = funnel_up(*m.FUNNEL_START)
F_HOLDER = funnel_up(m.TRAY_X + 0.003, m.TRAY_Y - 0.003, SEAT_F)
F_HOVER = funnel_up(m.TRAY_X, m.TRAY_Y, SEAT_F + 0.025)  # held still 2.5 cm over the holder
F_TRAY_LYING = funnel_lying(m.TRAY_X + 0.015, m.TRAY_Y + 0.01, m.TOP + 0.002)
F_OVER_WALL = funnel_lying(m.TRAY_X + 0.015, m.TRAY_Y + 0.06, m.TOP + 0.002)  # draped over the tray's wall
F_BENCH = funnel_up(0.22, 0.10, m.TOP + m.STEM_L)
F_BENCH_LYING = funnel_lying(0.22, 0.10, m.TOP)
F_FLOOR = funnel_lying(0.30, 0.10, m.TOP - 0.75)
F_HELD_ABOVE_NECK = funnel_up(m.JUG_X, m.JUG_Y, m.NECK_TOP + 0.06)
F_HELD_STILL_BENCH = funnel_up(0.22, 0.10, m.TOP + 0.10)
C_START = st(m.CAP_START)
C_SEATED = st((m.JUG_X + 0.006, m.JUG_Y - 0.004, m.CAP_SEATED_Z))
C_HIGH = st((m.JUG_X, m.JUG_Y, m.CAP_SEATED_Z + 0.006))  # 6 mm proud: not down on the rim
C_ON_FUNNEL = st((m.JUG_X, m.JUG_Y, m.FUNNEL_START[2] + m.CONE_H + m.CAP_H / 2))
C_LOOSE_ON_RIM = st((m.JUG_X + 0.02, m.JUG_Y, m.CAP_SEATED_Z + 0.012), tilt=0.5)
C_TILTED = st((m.JUG_X, m.JUG_Y, m.CAP_SEATED_Z), tilt=math.radians(13))
C_UPSIDE_DOWN = st((m.JUG_X, m.JUG_Y, m.CAP_SEATED_Z), tilt=math.pi)
C_MOVING = st((m.JUG_X, m.JUG_Y, m.CAP_SEATED_Z), vel=0.2)
G_ON_CAP = (np.array(C_SEATED.pos) + (0, 0, 0.013), FAR[1])
TAB_HOLDER = np.array(F_HOLDER.pos) + np.array(m.TAB_C)

CASES = [
    ("nothing done", scene(F_IN, C_START), 0.0),
    ("cap put on top of the funnel left in the neck", scene(F_IN, C_ON_FUNNEL), 0.0),
    ("funnel left in, cap seated somehow beside it", scene(F_IN, C_SEATED), 0.0),
    ("funnel in its holder, cap seated", scene(F_HOLDER, C_SEATED), 1.0),
    ("funnel lying inside the drip tray, cap seated", scene(F_TRAY_LYING, C_SEATED), 1.0),
    ("funnel draped over the drip tray's wall, cap seated", scene(F_OVER_WALL, C_SEATED), 0.0),
    ("funnel held still 2.5 cm over its holder, cap seated", scene(F_HOVER, C_SEATED), 0.0),
    ("funnel standing on the bare bench, cap seated", scene(F_BENCH, C_SEATED), 0.5),
    ("funnel lying on the bare bench, cap seated", scene(F_BENCH_LYING, C_SEATED), 0.5),
    ("funnel knocked off the table, cap seated", scene(F_FLOOR, C_SEATED), 0.0),
    ("funnel held still over the bench, cap seated", scene(F_HELD_STILL_BENCH, C_SEATED), 0.0),
    ("cap still held on the neck, funnel in its holder", scene(F_HOLDER, C_SEATED, G_ON_CAP, (0.1, 1.0)), 0.25),
    ("open gripper resting at the cap, funnel in its holder: done, arm not home", scene(F_HOLDER, C_SEATED, G_ON_CAP, (1.0, 1.0)), 0.9),
    ("done, right arm parked 10 cm from its start", scene(F_HOLDER, C_SEATED, (FAR[0], (0.16, -0.16, m.TOP + 0.20))), 0.9),
    ("done, arms home", scene(F_HOLDER, C_SEATED), 1.0),
    ("funnel still gripped by its tab in the holder, cap seated", scene(F_HOLDER, C_SEATED, (TAB_HOLDER, FAR[1]), (0.1, 1.0)), 0.0),
    ("funnel in its holder, cap still on the bench", scene(F_HOLDER, C_START), 0.25),
    ("funnel in its holder, cap 6 mm proud of the rim", scene(F_HOLDER, C_HIGH), 0.25),
    ("funnel in its holder, cap tilted 13 degrees", scene(F_HOLDER, C_TILTED), 0.25),
    ("funnel in its holder, cap perched crooked on the rim", scene(F_HOLDER, C_LOOSE_ON_RIM), 0.25),
    ("funnel in its holder, cap upside down on the neck", scene(F_HOLDER, C_UPSIDE_DOWN), 0.25),
    ("funnel in its holder, cap still moving", scene(F_HOLDER, C_MOVING), 0.25),
    ("funnel on the bench, cap on the bench", scene(F_BENCH, C_START), 0.0),
    ("funnel held above the neck at the end, cap on the bench", scene(F_HELD_ABOVE_NECK, C_START), 0.0),
]

bad = 0
for name, objs, want in CASES:
    got = run(objs)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

# A gripper with no reading counts as open, not as holding (job scored directly).
use(scene(F_HOLDER, C_SEATED, G_ON_CAP))
kit._OPENING.clear()
got = m._score(scene(F_HOLDER, C_SEATED, G_ON_CAP)[0])
bad += got != 1.0
print(("ok  " if got == 1.0 else "FAIL"), f"{got:.2f} (want 1.0) no gripper reading")

# success(): the job done, both arms home and still.
m._H = m._History()
m.success(use(scene(F_IN, C_START)))
done = use(scene(F_HOLDER, C_SEATED))
m.success(done)
s_home = m.success(done)
away = use(scene(F_HOLDER, C_SEATED, (FAR[0], (0.20, -0.12, m.TOP + 0.15))))
s_away = m.success(away) or m.success(away)
moving = use(scene(F_HOLDER, C_SEATED, (FAR[0], (FAR[1][0], FAR[1][1] + 0.01, FAR[1][2]))))
s_moving = m.success(moving)  # within 3 cm of home but moved 1 cm this step
for name, got, want in [("success when done and arms home", s_home, True), ("no success with an arm away", s_away, False),
                        ("no success while an arm is still moving", s_moving, False)]:
    bad += got != want
    print(("ok  " if got == want else "FAIL"), name)

# A reset clears the recorded home: the next trial records its own first step.
m._H = m._History()
m.success(use(scene(F_IN, C_START)))
kit._EPISODE["count"] += 1  # the simulator resets the scene
other = ((0.20, 0.20, m.TOP + 0.15), (0.20, -0.20, m.TOP + 0.15))
m.success(use(scene(F_IN, C_START, other)))
got = all(np.allclose(m._H.home[g], p) for g, p in zip(m.GRIPPERS, other))
bad += not got
print(("ok  " if got else "FAIL"), "reset clears the recorded home")

print("all pass" if not bad else f"{bad} FAILED")
