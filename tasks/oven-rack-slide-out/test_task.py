"""Offline checks of the oven task's scorer: step-by-step histories of the rack joint, the pan and the grippers, with
near misses. No simulator.

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


def st(pos, tilt_deg=0.0, moving=False, vz=0.0):
    p = np.array(pos, float)
    a = math.radians(tilt_deg) / 2
    v = np.array([0.05 if moving else 0.0, 0, vz])
    return ObjectState(pos=p, quat_xyzw=np.array([math.sin(a), 0, 0, math.cos(a)]), lin_vel=v, ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def step(q, pan, grippers=None, tilt=0.0, moving=False, vz=0.0):
    """grippers: name -> position (fingers open) or (position, opening)."""
    kit._JOINTS["oven/rack"] = kit.JointState(q, 0.0, False)
    g = dict(PARK, **(grippers or {}))
    objs = {"oven": st(m.BASE), "trivet": st((*m.TRIVET_XY, TOP + m.TRIVET_H / 2)), "pan": st(pan, tilt, moving, vz)}
    for k, v in g.items():
        pos, op = (v[0], v[1]) if isinstance(v[0], tuple) else (v, 1.0)
        objs[k] = st(pos)
        kit._OPENING[k] = op
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames):
    """frames: (steps, rack q, pan position[, grippers dict[, pan tilt[, pan vertical speed]]]). A fresh trial each
    time."""
    kit._EPISODE["count"] += 1
    out = step(0.0, P0)  # the trial's first scored step: the arms at their ready pose (PARK here), so home is PARK
    for fr in frames:
        n, q, pan = fr[0], fr[1], fr[2]
        g = fr[3] if len(fr) > 3 else None
        tilt = fr[4] if len(fr) > 4 else 0.0
        vz = fr[5] if len(fr) > 5 else 0.0
        for _ in range(n):
            out = step(q, pan, g, tilt, vz=vz)
    return out


P0 = m.PAN_START
OUTQ = -m.PULL
on_rack = lambda q: (P0[0] + q, P0[1], P0[2])  # noqa: E731
up = lambda q: (P0[0] + q, P0[1], P0[2] + 0.08)  # noqa: E731
TRIVET = (m.TRIVET_XY[0], m.TRIVET_XY[1], TOP + m.TRIVET_H + m.PAN[2] / 2)
COUNTER = (0.20, 0.15, TOP + m.PAN[2] / 2)
PULL = [(1, -m.PULL * k / 10, on_rack(-m.PULL * k / 10)) for k in range(1, 11)]
def hold(p):  # the right gripper closed on the pan at p
    return {"right_gripper": ((p[0], p[1], p[2] + 0.01), 0.1)}


CARRY = (0.24, -0.10, TOP + 0.20)
# taken off the pulled-out rack by the right gripper, carried, set down on the trivet, then let go
OUTSIDE = [(5, OUTQ, on_rack(OUTQ), hold(on_rack(OUTQ))), (5, OUTQ, up(OUTQ), hold(up(OUTQ))),
           (5, OUTQ, CARRY, hold(CARRY))]
DONE = [(40, 0.0, TRIVET)]  # pan on the trivet, rack pushed back in, arms home and still
IN_OVEN = {"right_gripper": (0.40, -0.02, TOP + 0.12)}
AT_RAIL = {"right_gripper": (m.RAIL_X, -0.04, TOP + m.RACK_Z + 0.017)}

cases = [
    ("nothing", [(100, 0.0, P0)], 0.0),
    ("rack pulled out only", [(20, 0.0, P0, AT_RAIL)] + PULL + [(40, OUTQ, on_rack(OUTQ))], 0.25),
    ("rack out, pan to the trivet, rack left out", PULL + OUTSIDE + [(40, OUTQ, TRIVET)], 0.75),
    ("rack out, pan set on the counter", PULL + OUTSIDE + [(40, OUTQ, COUNTER)], 0.5),
    ("rack out, pan back on the rack", PULL + OUTSIDE + [(40, OUTQ, on_rack(OUTQ))], 0.25),
    ("rack out, pan on the trivet, rack pushed back in", PULL + OUTSIDE + [(40, 0.0, TRIVET)], 1.0),
    ("pan on the trivet but tipped 30 degrees", PULL + OUTSIDE + [(40, OUTQ, TRIVET, None, 30)], 0.25),
    ("reached into the oven, then did it right", [(3, 0.0, P0, IN_OVEN)] + PULL + OUTSIDE + [(40, OUTQ, TRIVET)], 0.0),
    ("tool point inside for 1 step only", [(1, 0.0, P0, IN_OVEN), (5, 0.0, P0)] + PULL + OUTSIDE
     + DONE, 1.0),
    ("pinching the front rail is not reaching in", [(30, 0.0, P0, AT_RAIL)] + PULL + OUTSIDE + DONE, 1.0),
    ("pan lifted straight out of the oven", [(5, 0.0, up(0.0)), (5, 0.0, (0.24, -0.10, TOP + 0.20))]
     + [(40, 0.0, TRIVET)], 0.0),
    ("pan dragged 3 cm toward the door on the rack", [(5, 0.0, (P0[0] - 0.03, 0, P0[2]))] + PULL + OUTSIDE
     + [(40, OUTQ, TRIVET)], 0.0),
    ("pan lifted with the rack only 8 cm out", [(5, -0.08, on_rack(-0.08)), (5, -0.08, up(-0.08))]
     + [(5, OUTQ, (0.24, -0.10, TOP + 0.20)), (40, OUTQ, TRIVET)], 0.0),
    ("pan on the trivet with the rack never pulled", [(40, 0.0, TRIVET)], 0.0),
    # QC: credit for setting the pan down needs a gripper to have taken it off the rack and let go of it
    ("pan knocked off the pulled rack onto the door (falls)", PULL + [(3, OUTQ, (0.20, 0.0, TOP + 0.07), None, 0, -0.9),
     (40, OUTQ, (0.20, 0.0, TOP + m.CAV_FLOOR + m.PAN[2] / 2))], 0.0),
    ("pan shoved off the rack without a grip, no fall seen", PULL + [(40, OUTQ, COUNTER)], 0.25),
    ("pan still held on the trivet at the end", PULL + OUTSIDE + [(40, OUTQ, TRIVET, hold(TRIVET))], 0.25),
    ("pan dropped from the grip onto the counter", PULL + OUTSIDE + [(3, OUTQ, (0.22, 0.10, TOP + 0.10), None, 0, -1.0),
     (40, OUTQ, COUNTER)], 0.0),
    ("pan set down gently (a few mm) and let go", PULL + OUTSIDE + [(2, OUTQ, TRIVET, hold(TRIVET), 0, -0.3)]
     + DONE, 1.0),
    ("pan tipped 50 degrees while carried", PULL + OUTSIDE + [(3, OUTQ, CARRY, hold(CARRY), 50)]
     + [(40, OUTQ, TRIVET)], 0.0),
    ("tool point jittering across the 3.5 cm line", [(1, 0.0, P0, {"right_gripper": (m.FRONT + 0.036, -0.02, TOP + 0.12)}),
     (1, 0.0, P0, {"right_gripper": (m.FRONT + 0.032, -0.02, TOP + 0.12)})] * 3 + PULL + OUTSIDE
     + [(40, OUTQ, TRIVET)], 0.0),
    ("tool point held 3.2 cm in, never past 3.5", [(30, 0.0, P0, {"right_gripper": (m.FRONT + 0.032, -0.02, TOP + 0.12)})]
     + PULL + OUTSIDE + DONE, 1.0),
    # Andres: close up (rack back in) and arms home
    ("pan on the trivet, rack in, arms home and still", PULL + OUTSIDE + DONE, 1.0),
    ("done, but the right arm not home", PULL + OUTSIDE + [(40, 0.0, TRIVET, {"right_gripper": (0.30, -0.15, TOP + 0.15)})], 0.9),
    ("done, arms at home but still moving", PULL + OUTSIDE + [(1, 0.0, TRIVET, {"right_gripper": (0.18, -0.30 + 0.002 * (k % 2), TOP + 0.22)}) for k in range(40)], 0.9),
    ("rack pushed only halfway back", PULL + OUTSIDE + [(40, -0.05, TRIVET)], 0.75),
]

bad = 0
for name, frames, want in cases:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

# success needs both grippers clear of the pan
got, ok = run(PULL + OUTSIDE + [(10, OUTQ, TRIVET, {"right_gripper": (TRIVET[0], TRIVET[1], TRIVET[2] + 0.02)})])
bad += ok
print("ok  " if not ok else "FAIL", "no success while a gripper still holds the pan")
got, ok = run(PULL + OUTSIDE + DONE)
bad += not ok
print("ok  " if ok else "FAIL", "success once done, rack in and the arms home")
# a reset clears the latches
run([(3, 0.0, P0, IN_OVEN)])
got, _ = run(PULL + OUTSIDE + DONE)
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", "a reset clears an earlier trial's violation")
# success needs the arms home
got, ok = run(PULL + OUTSIDE + [(40, 0.0, TRIVET, {"right_gripper": (0.30, -0.15, TOP + 0.15)})])
bad += ok
print("ok  " if not ok else "FAIL", "no success with the job done but an arm away from home")
# with the setting off, the job alone is full credit
m.REQUIRE_ARMS_HOME = False
got, ok = run(PULL + OUTSIDE + [(40, 0.0, TRIVET, {"right_gripper": (0.30, -0.15, TOP + 0.15)})])
m.REQUIRE_ARMS_HOME = True
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", got, "REQUIRE_ARMS_HOME off: done, arms away, scores 1.0")
# a reset clears the recorded home: trial 1 starts with the right arm elsewhere, trial 2 starts and ends at PARK
kit._EPISODE["count"] += 1
step(0.0, P0, {"right_gripper": (0.30, -0.15, TOP + 0.15)})
got, _ = run(PULL + OUTSIDE + DONE)
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", got, "a reset records home afresh")
print("all pass" if not bad else f"{bad} FAILED")
