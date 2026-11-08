"""Offline checks of the paper cutter's scorer: hand-made object and joint states, no simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit


def st(pos, tilt=0.0, vel=0.0):
    q = (math.sin(tilt / 2), 0.0, 0.0, math.cos(tilt / 2))
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q), lin_vel=np.array([vel, 0, 0]), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def scene(cards, arm_q, latched=False, arm_vel=0.0, grippers=((0.20, 0.27, 0.95), (0.18, -0.26, 0.95))):
    """The objects, and the arm's joint state (set into the kit when the case is scored)."""
    objs = {"cutter": st((m.BASE_X, m.BASE_Y, m.TOP + 0.01)), "cutter/arm": st(m.HINGE), "tray": st((m.TRAY_X, m.TRAY_Y, m.TOP)),
            "cards": cards, "left_gripper": st(grippers[0]), "right_gripper": st(grippers[1])}
    return objs, kit.JointState(pos=arm_q, vel=arm_vel, latched=latched)


def use(case):
    objs, joint = case
    kit._JOINTS["cutter/arm"] = joint
    return objs


HOME = {"left_gripper": (0.20, 0.27, 0.95), "right_gripper": (0.18, -0.26, 0.95)}


def evaluate(case, start=HOME):
    """A fresh trial: the first step has the grippers at `start` (recorded as home), then two steps of the case's state
    (the second shows the grippers still). Returns (score, success)."""
    objs = use(case)
    kit._EPISODE["count"] += 1
    m.success({**objs, **{g: st(p) for g, p in start.items()}})
    m.success(objs)
    su = m.success(objs)
    return m.score(objs), su


ON_BED = st(m.CARDS_START)
IN_TRAY = st((m.TRAY_X, m.TRAY_Y, m.TOP + 0.004 + m.CARDS[2] / 2))
ON_TABLE = st((0.25, 0.05, m.TOP + m.CARDS[2] / 2))
handle_down = np.array(m.HINGE) + np.array(m.HANDLE)

CASES = [
    ("nothing done", scene(ON_BED, m.RAISED), 0.0),
    ("arm latched with the cards still on the bed", scene(ON_BED, 0.0, latched=True), 0.0),
    ("cards in the tray, arm still raised", scene(IN_TRAY, m.RAISED), 0.25),
    ("cards in the tray, arm let go before the catch", scene(IN_TRAY, 0.4), 0.25),
    ("cards in the tray, arm latched", scene(IN_TRAY, 0.0, latched=True), 1.0),
    ("cards on the table off the bed, arm latched", scene(ON_TABLE, 0.0, latched=True), 1.0),
    ("cards tipped on edge on the table, arm latched", scene(st((0.25, 0.05, m.TOP + 0.035), tilt=1.4), 0.0, latched=True), 0.0),
    ("cards still moving (held or falling), arm latched", scene(st((0.25, 0.05, m.TOP + 0.10), vel=0.2), 0.0, latched=True), 0.0),
    ("arm latched but still bouncing", scene(IN_TRAY, 0.01, latched=True, arm_vel=0.3), 0.25),
    ("cards held still 7 cm above the bed's side, arm latched", scene(st((0.25, 0.05, m.TOP + 0.07)), 0.0, latched=True), 0.0),
    ("cards held still just above the table, arm raised", scene(st((0.25, 0.05, m.TOP + 0.04)), m.RAISED), 0.0),
    ("cards on the table but a closed gripper still on them", scene(ON_TABLE, 0.0, latched=True,
                                                                    grippers=((0.20, 0.27, 0.95), (0.25, 0.05, m.TOP + 0.02))), 0.0),
]
kit._OPENING.update({"left_gripper": 1.0, "right_gripper": 0.05})  # left open, right closed (only near the cards in the case above)

bad = 0
for name, objs, want in CASES:
    got, _ = evaluate(objs)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

# Arms home: full score and success need both tool points back within 3 cm of their start, and still.
done = scene(IN_TRAY, 0.0, latched=True)
sc, su = evaluate(done)
away = scene(IN_TRAY, 0.0, latched=True, grippers=((0.20, 0.27, 0.95), tuple(handle_down)))
sc2, su2 = evaluate(away)
near = scene(IN_TRAY, 0.0, latched=True, grippers=((0.20, 0.27, 0.95), (0.18, -0.24, 0.95)))  # 2 cm off: home
sc3, _ = evaluate(near)
# A reset clears the recorded home: the next trial records its own.
kit._EPISODE["count"] += 1
m.success(use(scene(IN_TRAY, 0.0, latched=True, grippers=((0.25, 0.25, 0.90), (0.25, -0.25, 0.90)))))
kit._EPISODE["count"] += 1
objs = use(done)
m.success(objs)
m.success(objs)
reset_ok = m.score(objs) == 1.0
# REQUIRE_ARMS_HOME off: the finished job scores 1.0 wherever the arms are.
m.REQUIRE_ARMS_HOME = False
sc4, su4 = evaluate(away)
m.REQUIRE_ARMS_HOME = True
for flag, name in ((sc == 1.0 and su, "done and arms home: 1.0 and success"),
                   (sc2 == 0.9 and not su2, "done, a gripper still on the handle: 0.9, no success"),
                   (sc3 == 1.0, "done, a gripper 2 cm from its start: home"),
                   (reset_ok, "a reset clears the recorded home"),
                   (sc4 == 1.0 and su4, "with REQUIRE_ARMS_HOME off, done alone is 1.0")):
    bad += not flag
    print(("ok  " if flag else "FAIL"), name)
print("all pass" if not bad else f"{bad} FAILED")
raise SystemExit(1 if bad else 0)
