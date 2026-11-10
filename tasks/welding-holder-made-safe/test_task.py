"""Offline checks of the welding-holder task's scorer: step-by-step histories of the holder, the stub and the grippers,
with near misses. No simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
TT = m.TT
PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.22), "right_gripper": (0.18, -0.30, m.TOP + 0.22)}
H0, S0 = m.HOLDER_START, m.STUB_START
HUNG = (m.HANGER_XY[0], m.HANGER_XY[1], TT + m.SLOT_Z)  # the holder seated in both forks
IN_CAN = (m.CAN_XY[0], m.CAN_XY[1], TT + 0.004)  # the stub lying on the can's floor
LIFT = (H0[0], H0[1], TT + 0.10)
CARRY = (0.28, 0.18, TT + 0.15)


def quat(yaw=0.0, roll=0.0):
    """Yaw about z, then roll about the body's y axis (tips the holder onto its side)."""
    cy, sy, cr, sr = math.cos(yaw / 2), math.sin(yaw / 2), math.cos(roll / 2), math.sin(roll / 2)
    return np.array([-sy * sr, cy * sr, sy * cr, cy * cr])  # q_yaw * q_roll(y), as x, y, z, w


def st(pos, q=None):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat() if q is None else q, lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def stub_with(holder):
    """The stub still clamped in the jaws of a holder at `holder` (yaw 0)."""
    return (holder[0], holder[1] + sum(m.STUB_Y) / 2, holder[2] + 0.016 + m.STUB_D / 2)


def step(f):
    """f: optional holder pos, holder_q, stub pos (default: in the jaws), stub_q, grippers (name -> pos or (pos, op))."""
    hp = f.get("holder", H0)
    kit._JOINTS["holder/lever"] = kit.JointState(0.0, 0.0, False)
    objs = {"holder": st(hp, f.get("holder_q")), "stub": st(f.get("stub", stub_with(hp)), f.get("stub_q", quat(-math.pi / 2)))}
    for k, v in dict(PARK, **f.get("grippers", {})).items():
        pos, op = (v[0], v[1]) if isinstance(v[0], tuple) else (v, 1.0)
        objs[k] = st(pos)
        kit._OPENING[k] = op
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames):
    """frames: (steps, frame dict). A fresh trial each time; its first step has the arms at PARK (home)."""
    kit._EPISODE["count"] += 1
    out = step({})
    for n, f in frames:
        for _ in range(n):
            out = step(f)
    return out


def grip(p, op=0.1, side="left_gripper"):  # a gripper on the holder's handle, the holder's origin at p
    return {side: ((p[0], p[1] + 0.038, p[2] + 0.011), op)}


STUB_OUT = [(10, {"stub": (S0[0], S0[1] - 0.06, S0[2]), "grippers": {"right_gripper": ((S0[0], S0[1] - 0.06, S0[2]), 0.1)}}),
            (10, {"stub": IN_CAN})]
HANG = [(5, {"stub": IN_CAN, "holder": LIFT, "grippers": grip(LIFT)}),
        (5, {"stub": IN_CAN, "holder": CARRY, "grippers": grip(CARRY)}),
        (5, {"stub": IN_CAN, "holder": HUNG, "grippers": grip(HUNG)})]
DONE = [(40, {"stub": IN_CAN, "holder": HUNG})]
ON_TABLE = (0.30, -0.05, TT + 0.0035)
DROP_AT = (0.30, 0.05, TT)

cases = [
    ("nothing", [(100, {})], 0.0),
    ("stub in the can, holder hung, arms home", STUB_OUT + HANG + DONE, 1.0),
    ("done, an arm not home", STUB_OUT + HANG + [(40, {"stub": IN_CAN, "holder": HUNG,
                                                     "grippers": {"left_gripper": (0.30, 0.0, m.TOP + 0.20)}})], 0.9),
    ("done, an arm at home but still moving", STUB_OUT + HANG + [(1, {"stub": IN_CAN, "holder": HUNG, "grippers": {
        "right_gripper": (0.18, -0.30 + 0.002 * (k % 2), m.TOP + 0.22)}}) for k in range(40)], 0.9),
    # the trap: hung with the stub still in the jaws
    ("hung with the stub still in the jaws", [(5, {"holder": LIFT, "grippers": grip(LIFT)}),
                                              (5, {"holder": HUNG, "grippers": grip(HUNG)}), (40, {"holder": HUNG})], 0.1),
    ("hung, stub pulled 2 cm (its end still 6 mm in the jaws)",
     [(5, {"holder": HUNG, "grippers": grip(HUNG)}), (40, {"holder": HUNG, "stub": (
         HUNG[0], HUNG[1] + sum(m.STUB_Y) / 2 - 0.020, HUNG[2] + 0.0192)})], 0.1),
    ("hung, stub pulled 3 cm (just clear of the jaws) and left there",
     [(5, {"holder": HUNG, "grippers": grip(HUNG)}), (40, {"holder": HUNG, "stub": (
         HUNG[0], HUNG[1] + sum(m.STUB_Y) / 2 - 0.030, HUNG[2] + 0.0192)})], 0.25),
    ("stub in the can, holder left on the steel", STUB_OUT + [(40, {"stub": IN_CAN})], 0.0),
    # the stub elsewhere
    ("hung, stub dropped on the steel table", [(10, {"stub": ON_TABLE})] + [(h[0], dict(h[1], stub=ON_TABLE)) for h in HANG]
     + [(40, {"stub": ON_TABLE, "holder": HUNG})], 0.25),
    ("hung, stub still held in a gripper",
     [(5, {"holder": HUNG, "grippers": grip(HUNG), "stub": (0.25, -0.15, TT + 0.10)})]
     + [(40, {"holder": HUNG, "stub": (0.25, -0.15, TT + 0.10),
              "grippers": {"right_gripper": ((0.25, -0.15, TT + 0.10), 0.1)}})], 0.25),
    ("hung, stub held just over the can", [(5, {"holder": HUNG, "grippers": grip(HUNG), "stub": IN_CAN})]
     + [(40, {"holder": HUNG, "stub": IN_CAN, "grippers": {"right_gripper": (IN_CAN, 0.1)}})], 0.25),
    ("hung, stub fell to the floor", [(5, {"stub": (0.30, -0.45, m.TOP - 0.5)})] + HANG[:0]
     + [(5, {"holder": HUNG, "grippers": grip(HUNG), "stub": (0.30, -0.45, m.TOP - 0.5)}),
        (40, {"holder": HUNG, "stub": (0.30, -0.45, m.TOP - 0.5)})], 0.1),
    ("stub fell to the floor, later put in the can", [(5, {"stub": (0.30, -0.45, m.TOP - 0.5)})] + HANG + DONE, 0.1),
    ("stub below the table for 1 step only (noise)", [(1, {"stub": (0.30, -0.45, m.TOP - 0.5)})] + STUB_OUT + HANG
     + DONE, 1.0),
    # QC: holder dropped or laid back on the grounded steel after being picked up
    ("holder picked up, dropped on the bench, then hung", STUB_OUT + [
        (5, {"stub": IN_CAN, "holder": LIFT, "grippers": grip(LIFT)}), (5, {"stub": IN_CAN, "holder": ON_TABLE})]
     + HANG + DONE, 0.0),
    ("holder dropped onto its side on the bench, then hung", STUB_OUT + [
        (5, {"stub": IN_CAN, "holder": LIFT, "grippers": grip(LIFT)}),
        (5, {"stub": IN_CAN, "holder": (0.30, 0.05, TT + 0.018), "holder_q": quat(0.0, math.pi / 2)})] + HANG + DONE, 0.0),
    ("holder set down on the bench for 1 step only (noise)", STUB_OUT + [
        (5, {"stub": IN_CAN, "holder": LIFT, "grippers": grip(LIFT)}), (1, {"stub": IN_CAN, "holder": ON_TABLE})]
     + HANG + DONE, 1.0),
    ("holder lifted 1 cm (not clear of the steel), set back, regripped and hung", STUB_OUT + [
        (5, {"stub": IN_CAN, "holder": (H0[0], H0[1], TT + 0.01), "grippers": grip((H0[0], H0[1], TT + 0.01))}),
        (5, {"stub": IN_CAN, "holder": H0, "grippers": grip(H0, op=1.0)})] + HANG + DONE, 1.0),
    ("holder lifted 2 cm, set back with the gripper still on it, then hung", STUB_OUT + [
        (5, {"stub": IN_CAN, "holder": (H0[0], H0[1], TT + 0.02), "grippers": grip((H0[0], H0[1], TT + 0.02))}),
        (5, {"stub": IN_CAN, "holder": H0, "grippers": grip(H0)})] + HANG + DONE, 0.0),
    ("handed from one arm to the other in the air, then hung", STUB_OUT + [
        (5, {"stub": IN_CAN, "holder": LIFT, "grippers": grip(LIFT)}),
        (5, {"stub": IN_CAN, "holder": LIFT, "grippers": dict(grip(LIFT), **grip(LIFT, side="right_gripper"))}),
        (5, {"stub": IN_CAN, "holder": LIFT, "grippers": grip(LIFT, side="right_gripper")})] + HANG + DONE, 1.0),
    # independent review: drops near a gripper, held down, low drops
    ("dropped on the steel with an open gripper hovering 5 cm above, then hung", STUB_OUT + [
        (5, {"stub": IN_CAN, "holder": LIFT, "grippers": grip(LIFT)}),
        (40, {"stub": IN_CAN, "holder": DROP_AT, "grippers": {"left_gripper": ((DROP_AT[0], DROP_AT[1] + 0.04, TT + 0.07),
                                                                                1.0)}})] + HANG + DONE, 0.0),
    ("laid back on the steel and held there 2 s, then hung", STUB_OUT + [
        (5, {"stub": IN_CAN, "holder": LIFT, "grippers": grip(LIFT)}),
        (40, {"stub": IN_CAN, "holder": DROP_AT, "grippers": grip(DROP_AT)})] + HANG + DONE, 0.0),
    ("lifted 2.5 cm, dropped, then hung", STUB_OUT + [
        (5, {"stub": IN_CAN, "holder": (H0[0], H0[1], TT + 0.025), "grippers": grip((H0[0], H0[1], TT + 0.025))}),
        (20, {"stub": IN_CAN, "holder": H0})] + HANG + DONE, 0.0),
    ("fumbled from 5 cm, the open hand stays put, regripped and hung", STUB_OUT + [
        (5, {"stub": IN_CAN, "holder": (H0[0], H0[1], TT + 0.05), "grippers": grip((H0[0], H0[1], TT + 0.05))}),
        (20, {"stub": IN_CAN, "holder": H0, "grippers": grip((H0[0], H0[1], TT + 0.05), op=1.0)})] + HANG + DONE, 0.0),
    ("holder slid along the bench, then hung", STUB_OUT + [(10, {"stub": IN_CAN, "holder": (0.30, 0.12, TT)})]
     + HANG + DONE, 1.0),
    # QC: hanging from the gripper, not the hanger
    ("holder held at the hanger at the end", STUB_OUT + HANG + [(40, {"stub": IN_CAN, "holder": HUNG,
                                                                  "grippers": grip(HUNG)})], 0.0),
    ("holder resting on open fingers at the hanger", STUB_OUT + HANG + [(40, {"stub": IN_CAN, "holder": HUNG,
                                                                         "grippers": grip(HUNG, op=1.0)})], 0.0),
    ("holder held 2 cm over the forks", STUB_OUT + [
        (40, {"stub": IN_CAN, "holder": (HUNG[0], HUNG[1], HUNG[2] + 0.02),
              "grippers": grip((HUNG[0], HUNG[1], HUNG[2] + 0.02))})], 0.0),
    # QC: near misses of the seat
    ("holder perched 1.5 cm high on the prongs", STUB_OUT + HANG + [(40, {"stub": IN_CAN, "holder": (
        HUNG[0], HUNG[1], HUNG[2] + 0.015)})], 0.0),
    ("holder 5 mm off in x (handle against a prong)", STUB_OUT + HANG + [(40, {"stub": IN_CAN, "holder": (
        HUNG[0] + 0.005, HUNG[1], HUNG[2])})], 1.0),
    ("holder 8 mm off in x (on a prong, not in the slot)", STUB_OUT + HANG + [(40, {"stub": IN_CAN, "holder": (
        HUNG[0] + 0.008, HUNG[1], HUNG[2])})], 0.0),
    ("holder 2.5 cm along y (the guard on a fork)", STUB_OUT + HANG + [(40, {"stub": IN_CAN, "holder": (
        HUNG[0], HUNG[1] + 0.025, HUNG[2])})], 0.0),
    ("hung reversed (jaws toward +y), handle in both forks", STUB_OUT + HANG[:2] + [
        (40, {"stub": IN_CAN, "holder": (HUNG[0], 0.330, HUNG[2]), "holder_q": quat(math.pi)})], 1.0),
    ("hung reversed but shifted onto the guard", STUB_OUT + HANG[:2] + [
        (40, {"stub": IN_CAN, "holder": (HUNG[0], 0.300, HUNG[2]), "holder_q": quat(math.pi)})], 0.0),
    ("hung reversed with the stub still in", HANG[:2] + [
        (40, {"holder": (HUNG[0], 0.330, HUNG[2]), "holder_q": quat(math.pi), "stub": (HUNG[0], 0.330 + 0.08, HUNG[2] + 0.0192)})],
     0.1),
    ("holder 2 cm off in x (outside the slots)", STUB_OUT + HANG + [(40, {"stub": IN_CAN, "holder": (
        HUNG[0] + 0.02, HUNG[1], HUNG[2])})], 0.0),
    ("holder across the hanger (90 degrees)", STUB_OUT + HANG + [(40, {"stub": IN_CAN, "holder": HUNG,
                                                                     "holder_q": quat(math.pi / 2)})], 0.0),
    ("holder hung tipped 30 degrees", STUB_OUT + HANG + [(40, {"stub": IN_CAN, "holder": HUNG,
                                                             "holder_q": quat(0.0, math.radians(30))})], 0.0),
    ("hung, then taken down to the table", STUB_OUT + HANG + DONE + [(40, {"stub": IN_CAN, "holder": (0.30, 0.0, TT)})],
     0.0),
    ("knocked off the table", [(40, {"holder": (0.3, 0.6, m.TOP - 0.5)})], 0.0),
]

# A new trial (the simulator's reset) clears the history: a bench drop in the last trial does not carry over.
fails = 0
for name, frames, want in cases:
    got, ok = run(frames)
    flag = "ok " if abs(got - want) < 1e-9 and ok == (want == 1.0) else "BAD"
    fails += flag == "BAD"
    print(f"{flag} {got:.2f} (want {want:.2f}) {name}")

run(STUB_OUT + [(5, {"stub": IN_CAN, "holder": LIFT, "grippers": grip(LIFT)}), (5, {"stub": IN_CAN, "holder": ON_TABLE})])
got, _ = run(STUB_OUT + HANG + DONE)
print(f"{'ok ' if got == 1.0 else 'BAD'} {got:.2f} (want 1.00) history cleared by a new trial")
fails += got != 1.0
print("all pass" if not fails else f"{fails} FAILED")
