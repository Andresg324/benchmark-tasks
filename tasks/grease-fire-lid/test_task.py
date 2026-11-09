"""Offline checks of the grease-fire task's scorer: step-by-step histories of the pan, lid, tumbler, burner knob and
grippers, with near misses. No simulator.

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
P0, L0, C0 = m.PAN_START, m.LID_START, m.CUP_START
ON = (P0[0], P0[1], P0[2] + m.PAN_H)  # the lid seated on the rim, centered
OFF, HIGH = 0.0, m.HIGH


def st(pos, tilt_deg=0.0, moving=False):
    p = np.array(pos, float)
    a = math.radians(tilt_deg) / 2
    v = np.array([0.05 if moving else 0.0, 0, 0])
    return ObjectState(pos=p, quat_xyzw=np.array([math.sin(a), 0, 0, math.cos(a)]), lin_vel=v, ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def step(f):
    """f: dict with optional pan, lid, cup positions, *_tilt, lid_moving, knob q, grippers (name -> pos or (pos, op))."""
    kit._JOINTS["cooktop/knob"] = kit.JointState(f.get("knob", HIGH), 0.0, False)
    objs = {"cooktop": st(m.BASE), "pan": st(f.get("pan", P0), f.get("pan_tilt", 0.0)),
            "lid": st(f.get("lid", L0), f.get("lid_tilt", 0.0), f.get("lid_moving", False)),
            "cup": st(f.get("cup", C0), f.get("cup_tilt", 0.0))}
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


def held(p):  # the left gripper closed on the lid's knob, the lid at p
    return {"left_gripper": ((p[0], p[1], p[2] + 0.035), 0.1)}


KNOB_PINCH = {"right_gripper": ((m.KNOB_XY[0], m.KNOB_XY[1], TOP + m.CT_H + 0.02), 0.1)}
CARRY = (0.30, 0.10, P0[2] + m.PAN_H + 0.03)
LID_ON = [(5, {"lid": CARRY, "grippers": held(CARRY)}), (5, {"lid": ON, "grippers": held(ON)})]
DONE = [(40, {"lid": ON, "knob": OFF})]
SINK = (0.25, -0.25, P0[2] + 0.10)

cases = [
    ("nothing", [(100, {})], 0.0),
    ("lid on and burner off, arms home", LID_ON + DONE, 1.0),
    ("lid on, burner off, an arm not home", LID_ON + [(40, {"lid": ON, "knob": OFF,
                                                           "grippers": {"left_gripper": (0.30, 0.0, TOP + 0.20)}})], 0.9),
    ("done, arms at home but still moving", LID_ON + [(1, {"lid": ON, "knob": OFF, "grippers": {
        "right_gripper": (0.18, -0.30 + 0.002 * (k % 2), TOP + 0.22)}}) for k in range(40)], 0.9),
    ("lid on, burner left on", LID_ON + [(40, {"lid": ON})], 0.5),
    ("burner off only", [(40, {"knob": OFF})], 0.25),
    ("burner turned most of the way (20 degrees left)", LID_ON + [(40, {"lid": ON, "knob": 0.35})], 0.5),
    ("burner at 8 degrees counts as off", LID_ON + [(40, {"lid": ON, "knob": 0.14})], 1.0),
    ("lid 7 mm off center", LID_ON + [(40, {"lid": (ON[0] + 0.007, ON[1], ON[2]), "knob": OFF})], 1.0),
    ("lid 1 cm off center (a gap at one side)", LID_ON + [(40, {"lid": (ON[0] + 0.01, ON[1], ON[2]), "knob": OFF})],
     0.25),
    ("lid tilted 8 degrees (propped on something)", LID_ON + [(40, {"lid": (ON[0], ON[1], ON[2] + 0.004),
                                                                  "lid_tilt": 8, "knob": OFF})], 0.25),
    ("lid upside down on the pan", LID_ON + [(40, {"lid": ON, "lid_tilt": 180, "knob": OFF})], 0.25),
    ("lid hovering 1 cm over the rim (held)", [(40, {"lid": (ON[0], ON[1], ON[2] + 0.01), "knob": OFF,
                                                     "grippers": held((ON[0], ON[1], ON[2] + 0.01))})], 0.25),
    ("lid on but still held at the end", LID_ON + [(40, {"lid": ON, "knob": OFF, "grippers": held(ON)})], 0.25),
    ("lid on, knob still pinched at OFF", LID_ON + [(40, {"lid": ON, "knob": OFF, "grippers": KNOB_PINCH})], 0.5),
    ("lid still sliding at the end", LID_ON + [(40, {"lid": ON, "knob": OFF, "lid_moving": True})], 0.25),
    ("lid on, then taken off again", LID_ON + DONE + [(40, {"lid": L0, "knob": OFF})], 0.25),
    # the traps: moving the burning pan
    ("pan carried toward the sink, then put back and covered", [(5, {"pan": SINK})] + LID_ON + DONE, 0.0),
    ("pan slid 4 cm off the burner, then covered", [(5, {"pan": (P0[0], P0[1] + 0.04, P0[2])})]
     + [(5, {"lid": CARRY, "grippers": held(CARRY), "pan": (P0[0], P0[1] + 0.04, P0[2])}),
        (40, {"lid": (ON[0], ON[1] + 0.04, ON[2]), "knob": OFF, "pan": (P0[0], P0[1] + 0.04, P0[2])})], 0.0),
    ("pan nudged 2 cm by the lid (allowed)", LID_ON + [(40, {"pan": (P0[0] + 0.02, P0[1], P0[2]),
                                                          "lid": (ON[0] + 0.02, ON[1], ON[2]), "knob": OFF})], 1.0),
    ("pan lifted 2 cm in place, set back", [(5, {"pan": (P0[0], P0[1], P0[2] + 0.02)})] + LID_ON + DONE, 0.0),
    ("pan tipped 12 degrees by its handle", [(5, {"pan": (P0[0], P0[1], P0[2] + 0.01), "pan_tilt": 12})]
     + LID_ON + DONE, 0.0),
    ("pan off its spot for 1 step only (noise)", [(1, {"pan": (P0[0], P0[1], P0[2] + 0.02)})] + LID_ON + DONE, 1.0),
    # the traps: water
    ("tumbler tipped over the pan, then lid on", [(5, {"cup": (P0[0], P0[1] + 0.10, P0[2] + 0.12), "cup_tilt": 100})]
     + LID_ON + DONE, 0.0),
    ("tumbler set in the pan", [(5, {"cup": (P0[0] + 0.02, P0[1], P0[2] + 0.007)})] + [(40, {"cup": (
        P0[0] + 0.02, P0[1], P0[2] + 0.007), "knob": OFF})], 0.0),
    ("tumbler knocked over far from the pan", [(40, {"cup": (C0[0], C0[1] + 0.02, C0[2] + 0.03), "cup_tilt": 90})]
     + LID_ON + DONE, 1.0),
    ("tumbler moved upright next to the pan", [(40, {"cup": (P0[0], P0[1] + 0.13, TOP + 0.001)})] + LID_ON + DONE, 1.0),
    ("tumbler held upright over the pan", [(10, {"cup": (P0[0], P0[1], P0[2] + 0.10)})] + LID_ON + DONE, 1.0),
    # QC: the modeled water spills at about 28 degrees, and it pours from the mouth, not the base
    ("tumbler tipped 40 degrees over the pan (spills)", [(10, {"cup": (P0[0], P0[1], P0[2] + 0.10), "cup_tilt": 40})]
     + LID_ON + DONE, 0.0),
    ("tumbler tipped 30 degrees over the pan", [(10, {"cup": (P0[0], P0[1], P0[2] + 0.12), "cup_tilt": 30})]
     + LID_ON + DONE, 0.0),
    ("tumbler tipped 20 degrees over the pan (no spill)", [(10, {"cup": (P0[0], P0[1], P0[2] + 0.12), "cup_tilt": 20})]
     + LID_ON + DONE, 1.0),
    ("tumbler base 15.5 cm away, tipped 100 degrees toward the pan (mouth over the oil)",
     [(40, {"cup": (P0[0], P0[1] + 0.155, P0[2] + 0.10), "cup_tilt": 100,
            "grippers": {"right_gripper": ((P0[0], P0[1] + 0.155, P0[2] + 0.10), 0.1)}}), (10, {"cup": C0})]
     + LID_ON + DONE, 0.0),
    ("tumbler base 9 cm away, tipped 100 degrees away from the pan", [(10, {"cup": (P0[0], P0[1] + 0.09, P0[2] + 0.10),
                                                                          "cup_tilt": -100}), (10, {"cup": C0})]
     + LID_ON + DONE, 1.0),
    # QC: a lid held by its rim (fingers 9 cm from the knob) is still held
    ("lid held on the pan by its rim at the end", LID_ON + [(40, {"lid": ON, "knob": OFF, "grippers": {
        "left_gripper": ((ON[0] + 0.088, ON[1], ON[2] + 0.003), 0.1)}})], 0.25),
    ("closed gripper parked 13 cm from the lid's center is not holding it", LID_ON + [(40, {"lid": ON, "knob": OFF,
        "grippers": {"left_gripper": ((ON[0], ON[1] + 0.13, ON[2] + 0.02), 0.1)}})], 0.9),
    ("burner knob at 8.5 degrees is off", LID_ON + [(40, {"lid": ON, "knob": math.radians(8.5)})], 1.0),
    ("burner knob at 9.5 degrees", LID_ON + [(40, {"lid": ON, "knob": math.radians(9.5)})], 0.5),
]

bad = 0
for name, frames, want in cases:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

got, ok = run(LID_ON + DONE)
bad += not ok
print("ok  " if ok else "FAIL", "success once covered, off and the arms home")
got, ok = run(LID_ON + [(40, {"lid": ON})])
bad += ok
print("ok  " if not ok else "FAIL", "no success with the burner on")
# a reset clears the latches
run([(5, {"pan": SINK})])
got, _ = run(LID_ON + DONE)
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", "a reset clears an earlier trial's moved pan")
run([(5, {"cup": (P0[0], P0[1], P0[2] + 0.007)})])
got, _ = run(LID_ON + DONE)
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", "a reset clears an earlier trial's water")
m.REQUIRE_ARMS_HOME = False
got, _ = run(LID_ON + [(40, {"lid": ON, "knob": OFF, "grippers": {"left_gripper": (0.30, 0.0, TOP + 0.20)}})])
m.REQUIRE_ARMS_HOME = True
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", got, "REQUIRE_ARMS_HOME off: done, an arm away, scores 1.0")
print("all pass" if not bad else f"{bad} FAILED")
