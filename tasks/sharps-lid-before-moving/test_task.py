"""Offline checks of the sharps container task's scorer: step-by-step histories of the lid joint, the container, the
syringes and the grippers, with near misses. No simulator.

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
C0 = np.array(m.CONT_START)
TOTE = np.array((m.TOTE_XY[0], m.TOTE_XY[1], C0[2]))
OPEN, AJAR, SHUT = m.LID_OPEN, 0.35, 0.0  # AJAR: lowered to 20 degrees and held there, not shut


def st(pos, tilt_deg=0.0):
    p = np.array(pos, float)
    a = math.radians(tilt_deg) / 2
    return ObjectState(pos=p, quat_xyzw=np.array([math.sin(a), 0, 0, math.cos(a)]), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=p - 0.01, high=p + 0.01)


def syringes_in(c):
    c = np.asarray(c, float)
    return {s: c + np.array([0, y, m.FILL + m.SYR[2] / 2 - m.CONT[2] / 2]) for s, y in zip(m.SYRINGES, m.SYR_Y)}


def step(q, latched, cont, grippers=None, tilt=0.0, syr=None):
    """grippers: name -> position (fingers open) or (position, opening). syr: name -> position (default: inside)."""
    kit._JOINTS["sharps/lid"] = kit.JointState(q, 0.0, latched)
    g = dict(PARK, **(grippers or {}))
    objs = {"sharps": st(cont, tilt), "tote": st((*m.TOTE_XY, TOP + m.TOTE_H / 2))}
    for s, p in dict(syringes_in(cont), **(syr or {})).items():
        objs[s] = st(p)
    for k, v in g.items():
        pos, op = (v[0], v[1]) if isinstance(v[0], tuple) else (v, 1.0)
        objs[k] = st(pos)
        kit._OPENING[k] = op
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames):
    """frames: (steps, lid q, latched, container position[, grippers[, tilt[, syringes]]]). A fresh trial each time."""
    kit._EPISODE["count"] += 1
    out = step(OPEN, False, C0)  # the trial's first scored step: the arms at their ready pose (PARK here)
    for fr in frames:
        n, q, lat, c = fr[:4]
        g = fr[4] if len(fr) > 4 else None
        tilt = fr[5] if len(fr) > 5 else 0.0
        syr = fr[6] if len(fr) > 6 else None
        for _ in range(n):
            out = step(q, lat, c, g, tilt, syr)
    return out


def hold(p):  # the right gripper closed on the container (a 6 cm container leaves the fingers 0.64 open)
    return {"right_gripper": ((p[0], p[1], p[2] + 0.03), 0.64)}


def path(a, b, n=10):
    return [tuple(a + (b - a) * k / n) for k in range(1, n + 1)]


UP = C0 + np.array([0, 0, 0.15])
OVER_TOTE = TOTE + np.array([0, 0, 0.15])
SNAP = [(5, 1.0, False, C0), (5, AJAR, False, C0), (5, SHUT, True, C0)]
CARRY = ([(1, SHUT, True, tuple(p), hold(p)) for p in path(C0, UP)]
         + [(1, SHUT, True, tuple(p), hold(p)) for p in path(UP, OVER_TOTE)]
         + [(1, SHUT, True, tuple(p), hold(p)) for p in path(OVER_TOTE, TOTE)])
DONE = [(40, SHUT, True, tuple(TOTE))]
OPEN_CARRY = ([(1, OPEN, False, tuple(p), hold(p)) for p in path(C0, UP)]
              + [(1, OPEN, False, tuple(p), hold(p)) for p in path(UP, OVER_TOTE)]
              + [(1, OPEN, False, tuple(p), hold(p)) for p in path(OVER_TOTE, TOTE)])
NEAR_HOLD = {"right_gripper": ((TOTE[0], TOTE[1], TOTE[2] + 0.03), 0.64)}
NEAR_OPEN = {"right_gripper": ((TOTE[0], TOTE[1], TOTE[2] + 0.03), 1.0)}
COUNTER = (0.25, 0.10, C0[2])
OVER12 = TOTE + np.array([0, 0, 0.12])
SQUEEZE = {"right_gripper": ((TOTE[0], TOTE[1] - 0.04, TOTE[2]), 0.9),
           "left_gripper": ((TOTE[0], TOTE[1] + 0.04, TOTE[2]), 0.9)}


def SLIP(*ops):
    """Carried 12 cm above the tote's floor; the gripper stays put while the container falls out of it."""
    tool = (OVER12[0], OVER12[1], OVER12[2] + 0.03)
    fall = [OVER12 - np.array([0, 0, d]) for d in (0.012, 0.049, 0.12)]
    return (SNAP + [(1, SHUT, True, tuple(p), hold(p)) for p in path(C0, UP)]
            + [(1, SHUT, True, tuple(p), hold(p)) for p in path(UP, OVER12)]
            + [(1, SHUT, True, tuple(p), {"right_gripper": (tool, op)}) for p, op in zip(fall, ops)]
            + [(5, SHUT, True, tuple(TOTE), {"right_gripper": (tool, 1.0)})] + DONE)

cases = [
    ("nothing", [(100, OPEN, False, tuple(C0))], 0.0),
    ("lid lowered to 20 degrees, not shut", [(5, 1.0, False, C0), (100, AJAR, False, tuple(C0))], 0.0),
    ("lid snapped shut, container left in place", SNAP + [(40, SHUT, True, tuple(C0))], 0.25),
    ("snapped, carried to the tote, set down upright, arms home", SNAP + CARRY + DONE, 1.0),
    ("the same, but the arm still parked over the tote", SNAP + CARRY + [(40, SHUT, True, tuple(TOTE), NEAR_OPEN)],
     0.9),
    ("the same, but still held in the tote", SNAP + CARRY + [(40, SHUT, True, tuple(TOTE), NEAR_HOLD)], 0.25),
    ("snapped, in the tote, leaning on a wall at 30 degrees", SNAP + CARRY + [(40, SHUT, True, tuple(TOTE), None, 30)],
     0.5),
    ("snapped, carried and set on the counter instead", SNAP
     + [(1, SHUT, True, tuple(p), hold(p)) for p in path(C0, np.array(COUNTER))] + [(40, SHUT, True, COUNTER)], 0.25),
    ("carried to the tote open, lid snapped there", OPEN_CARRY + [(5, SHUT, True, tuple(TOTE))] + DONE, 0.0),
    ("lid held ajar while carried, snapped in the tote", SNAP[:2]
     + [(1, AJAR, False, tuple(p), hold(p)) for p in path(C0, UP)] + [(5, SHUT, True, tuple(UP))]
     + [(1, SHUT, True, tuple(p), hold(p)) for p in path(UP, OVER_TOTE)]
     + [(1, SHUT, True, tuple(p), hold(p)) for p in path(OVER_TOTE, TOTE)] + DONE, 0.0),
    ("pushed 4 cm along the counter with the lid open, pushed back, then done right",
     [(5, OPEN, False, tuple(C0 + [0.04, 0, 0])), (5, OPEN, False, tuple(C0))] + SNAP + CARRY + DONE, 0.0),
    ("nudged 2 cm while sweeping the lid over (within the spot)",
     [(5, 1.0, False, tuple(C0 + [0.02, 0, 0]))] + SNAP + CARRY + DONE, 1.0),
    ("off its spot for 1 step only (a glitch)", [(1, OPEN, False, tuple(C0 + [0.05, 0, 0]))] + SNAP + CARRY + DONE,
     1.0),
    ("lifted 2 cm with the lid open, set back", [(5, OPEN, False, tuple(C0 + [0, 0, 0.02]))] + SNAP + CARRY + DONE,
     0.0),
    ("tilted 25 degrees in place with the lid open", [(5, OPEN, False, tuple(C0), None, 25)] + SNAP + CARRY + DONE,
     0.0),
    ("lid within the zone but the latch never caught", SNAP[:2] + [(40, 0.01, False, tuple(C0))], 0.0),
    ("latch flag set but lid forced 10 degrees open", SNAP + CARRY + [(40, 0.17, True, tuple(TOTE))], 0.0),
    ("latch forced open in the air while carried", SNAP + [(1, SHUT, True, tuple(p), hold(p)) for p in path(C0, UP)]
     + [(5, 0.5, True, tuple(UP), hold(UP)), (5, SHUT, True, tuple(UP), hold(UP))]
     + [(1, SHUT, True, tuple(p), hold(p)) for p in path(UP, OVER_TOTE)]
     + [(1, SHUT, True, tuple(p), hold(p)) for p in path(OVER_TOTE, TOTE)] + DONE, 0.0),
    ("snapped, tipped over into the tote", SNAP + CARRY + [(40, SHUT, True, tuple(TOTE), None, 90)], 0.0),
    ("snapped, knocked over on the counter", SNAP + [(40, SHUT, True, tuple(C0), None, 80)], 0.0),
    ("snapped, tipped 70 degrees for a moment while carried, set down upright", SNAP
     + [(1, SHUT, True, tuple(p), hold(p)) for p in path(C0, UP)] + [(3, SHUT, True, tuple(UP), hold(UP), 70)]
     + [(1, SHUT, True, tuple(p), hold(p)) for p in path(UP, OVER_TOTE)]
     + [(1, SHUT, True, tuple(p), hold(p)) for p in path(OVER_TOTE, TOTE)] + DONE, 0.0),
    ("a syringe taken out before snapping", [(5, OPEN, False, tuple(C0), None, 0, {"syringe_a": (0.2, 0.0, TOP)})]
     + SNAP + CARRY + DONE, 0.0),
    ("container just outside the tote, on the counter next to it",
     SNAP + [(1, SHUT, True, tuple(p), hold(p)) for p in path(C0, TOTE + [0.0, 0.12, 0.0])]
     + [(40, SHUT, True, tuple(TOTE + [0.0, 0.12, 0.0]))], 0.25),
    ("container resting on the tote's rim (held up 4.5 cm)",
     SNAP + CARRY + [(40, SHUT, True, tuple(TOTE + [0.06, 0, 0.045]), None, 20)], 0.25),
    ("snapped, let go 12 cm above the tote, lands upright", SNAP + CARRY[:20]
     + [(1, SHUT, True, tuple(OVER_TOTE - [0, 0, 0.03 * k])) for k in range(1, 5)] + DONE, 0.0),
    ("snapped, lowered fast while held, set down", SNAP + CARRY[:20]
     + [(1, SHUT, True, tuple(OVER_TOTE - [0, 0, 0.0375 * k]), hold(OVER_TOTE - [0, 0, 0.0375 * k])) for k in range(1, 5)]
     + DONE, 1.0),
    ("snapped, slips out 12 cm above the tote as the fingers open slowly (0.70, 0.76, 0.82)", SLIP(0.70, 0.76, 0.82),
     0.0),
    ("snapped, slips out 12 cm above the tote, fingers snap fully open", SLIP(1.0, 1.0, 1.0), 0.0),
    ("snapped, slips out 12 cm up while the fingers stay at 0.66", SLIP(0.66, 0.66, 0.66), 0.0),
    ("snapped, in the tote, squeezed between two open grippers", SNAP + CARRY + [(40, SHUT, True, tuple(TOTE), SQUEEZE)],
     0.25),
    ("container still moving in the tote", SNAP + CARRY
     + [(1, SHUT, True, tuple(TOTE + [0.002 * (k % 2), 0, 0])) for k in range(40)], 0.25),
]


def check():
    bad = 0
    for name, frames, want in cases:
        got, ok = run(frames)
        flag = "ok " if abs(got - want) < 1e-9 else "BAD"
        bad += flag == "BAD"
        print(f"{flag} {got:4.2f} (want {want:4.2f}) success={ok!s:5} {name}")
    # a new trial (scene reset) starts with a clean history: a violation in one trial does not carry over
    run([(5, OPEN, False, tuple(C0 + [0.05, 0, 0]))])
    got, _ = run(SNAP + CARRY + DONE)
    flag = "ok " if got == 1.0 else "BAD"
    bad += flag == "BAD"
    print(f"{flag} {got:4.2f} (want 1.00) after a failed trial, a reset trial done right")
    # and the same trial without a reset keeps the violation
    kit._EPISODE["count"] += 1
    step(OPEN, False, C0)
    for _ in range(5):
        step(OPEN, False, C0 + [0.05, 0, 0])
    got = 0
    for fr in SNAP + CARRY + DONE:
        for _ in range(fr[0]):
            got, _ok = step(fr[1], fr[2], fr[3], fr[4] if len(fr) > 4 else None, fr[5] if len(fr) > 5 else 0.0)
    flag = "ok " if got == 0.0 else "BAD"
    bad += flag == "BAD"
    print(f"{flag} {got:4.2f} (want 0.00) violation kept within one trial")
    print("all pass" if not bad else f"{bad} FAILED")
    return bad


if __name__ == "__main__":
    raise SystemExit(1 if check() else 0)
