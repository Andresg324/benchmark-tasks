"""Offline checks of the blast-gate scorer: step-by-step gate and gripper histories, near misses included, no
simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
S = m.GATE_START
T = m.TRAVEL


def st(pos):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def step(q, vel=None, grippers=None):
    """One control step: q = {gate: position}, grippers = {name: tool point}. Returns (score, success)."""
    for n in m.BRANCHES:
        kit._JOINTS[f"gates/{n}"] = kit.JointState(q[n], (vel or {}).get(n, 0.0), False)
    g = dict(PARK, **(grippers or {}))
    objs = {"shop": st(m.SHOP), "gates": st(m.GATES), **{k: st(p) for k, p in g.items()}}
    ok = m.success(objs)
    return m.score(objs), ok


def knob_at(name, q):
    return (m.X_G, m.BRANCHES[name] + q + m.KNOB_Y, m.TOP + m.KNOB_Z)


def run(final, n=30, grippers=None, vel=None, first=None):
    """A fresh trial: the start state with the arms at PARK, then `n` steps at `final`."""
    kit._EPISODE["count"] += 1
    step(first or dict(S))
    out = None
    for _ in range(n):
        out = step(final, vel, grippers)
    return out


def gates(saw, planer, sander):
    return {"saw": saw, "planer": planer, "sander": sander}


AWAY = {"left_gripper": (0.25, 0.25, m.TOP + 0.15)}  # done, but the left arm parked away from home
cases = [
    ("do nothing (start state)", gates(S["saw"], S["planer"], S["sander"]), {}, 0.0),
    ("start state is not correct", gates(T, 0.0, 0.022), {}, 0.0),
    ("right line-up, arms home", gates(0.0, T, 0.0), {}, 1.0),
    ("right line-up at the thresholds", gates(m.SHUT_MAX, m.OPEN_MIN, m.SHUT_MAX), {}, 1.0),
    ("planer open 1 mm short of the line", gates(0.0, m.OPEN_MIN - 0.001, 0.0), {}, 0.0),
    ("saw left 1 mm past shut", gates(m.SHUT_MAX + 0.001, T, 0.0), {}, 0.25),
    ("open all three", gates(T, T, T), {}, 0.0),
    ("open only the planer", gates(T, T, 0.022), {}, 0.0),
    ("planer open, saw shut, sander half open", gates(0.0, T, 0.022), {}, 0.25),
    ("planer open, sander shut, saw open", gates(T, T, 0.0), {}, 0.25),
    ("planer open, both others part open", gates(0.02, T, 0.02), {}, 0.0),
    ("all three shut", gates(0.0, 0.0, 0.0), {}, 0.0),
    ("wrong machine: saw open, others shut", gates(T, 0.0, 0.0), {}, 0.0),
    ("wrong machine: sander open, others shut", gates(0.0, 0.0, T), {}, 0.0),
    ("right line-up, arm not home", gates(0.0, T, 0.0), AWAY, 0.9),
    ("right line-up, still gripping the planer handle", gates(0.0, T, 0.0),
     {"left_gripper": knob_at("planer", T)}, 0.0),
    ("right line-up, hand 3.9 cm from the sander handle", gates(0.0, T, 0.0),
     {"right_gripper": tuple(np.array(knob_at("sander", 0.0)) + [0, 0, 0.039])}, 0.0),
    ("right line-up, hand 4.1 cm above the handle (let go)", gates(0.0, T, 0.0),
     {"right_gripper": tuple(np.array(knob_at("sander", 0.0)) + [0, 0, 0.041])}, 0.9),
]

fails = 0
for name, q, g, want in cases:
    got, ok = run(q, grippers=g)
    status = "ok" if abs(got - want) < 1e-9 and ok == (want == 1.0) else "FAIL"
    fails += status == "FAIL"
    print(f"{status:4s} {got:.2f} (want {want:.2f}) {name}")

# a gate still sliding at the end
got, _ = run(gates(0.0, T, 0.0), vel={"planer": 0.02})
print(("ok  " if got == 0.0 else "FAIL") + f" {got:.2f} (want 0.00) planer gate still sliding")
fails += got != 0.0

# history reset with kit.episode(): a trial that ends with the arm away does not carry its home into the next trial
run(gates(0.0, T, 0.0), grippers=AWAY)
kit._EPISODE["count"] += 1
step(dict(S), grippers=AWAY)  # the new trial starts with the left arm at AWAY: that is its home now
got, ok = step(gates(0.0, T, 0.0), grippers=AWAY)
got, ok = step(gates(0.0, T, 0.0), grippers=AWAY)
print(("ok  " if got == 1.0 else "FAIL") + f" {got:.2f} (want 1.00) home re-recorded after a scene reset")
fails += got != 1.0

# the start state really is wrong for the planer and the demo's targets are inside the thresholds
for n, want in (("saw", "open"), ("planer", "shut"), ("sander", "part")):
    kit._JOINTS[f"gates/{n}"] = kit.JointState(S[n], 0.0, False)
    if m.gate_state({}, n) != want:
        print(f"FAIL start state of {n}: {m.gate_state({}, n)}")
        fails += 1

print("all pass" if not fails else f"{fails} FAILED")
