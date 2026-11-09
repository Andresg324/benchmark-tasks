"""Offline checks of the autoclave task's scorer: step-by-step histories of the three caps' threads, the bottles and the
grippers, with near misses. No simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, tk = m.kit, m.tk
TOP = m.TOP
tk.register(*[o for o in m.OBJECTS if isinstance(o, tk.Threaded)])
PARK = {"left_gripper": (0.20, 0.27, TOP + 0.22), "right_gripper": (0.18, -0.30, TOP + 0.22)}
NAMES = list(m.BOTTLES)
m.FINGER_OPEN = FINGER_OPEN
bad = 0


def st(pos, tilt_deg=0.0, moving=False):
    p = np.array(pos, float)
    a = math.radians(tilt_deg) / 2
    v = np.array([0.05 if moving else 0.0, 0, 0])
    return ObjectState(pos=p, quat_xyzw=np.array([math.sin(a), 0, 0, math.cos(a)]), lin_vel=v, ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


START = {n: (s[0], s[1], TOP + 0.001) for n, (s, _, _) in m.BOTTLES.items()}
TRAYED = {n: (slot[0], slot[1], TOP + m.TRAY_FLOOR) for n, (_, slot, _) in m.BOTTLES.items()}


def step(bottles, caps=None, grippers=None, tilt=None, held=None):
    """bottles: name -> base position; caps: name -> degrees past the seat (positive = tight), set this step;
    held: names whose cap a gripper holds this step (the turning then counts as gripped); tilt: name -> degrees;
    grippers: name -> position (fingers open) or (position, opening)."""
    held = held or ()
    for n, deg in (caps or {}).items():
        tk.set_angle(f"{n}/cap", deg, held_now=n in held)
    for n in NAMES:
        if n not in (caps or {}):
            tk._track(f"{n}/cap").held = n in held
    g = dict(PARK, **(grippers or {}))
    objs = {"tray": st((*m.TRAY_C, TOP))}
    for n in NAMES:
        objs[n] = st(bottles[n], (tilt or {}).get(n, 0.0))
        objs[f"{n}/cap"] = st(np.add(bottles[n], (0, 0, m.SEAT)))
    for k, v in g.items():
        pos, op = (v[0], v[1]) if isinstance(v[0], tuple) else (v, 1.0)
        objs[k] = st(pos)
        kit._OPENING[k] = op
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames):
    """frames: (steps, bottles, caps, grippers, tilt, held). A fresh trial each time."""
    kit._EPISODE["count"] += 1
    out = step(START)  # the trial's first scored step: arms at their ready pose (PARK), so home is PARK
    for fr in frames:
        n, bottles = fr[0], fr[1]
        caps = fr[2] if len(fr) > 2 else None
        g = fr[3] if len(fr) > 3 else None
        tilt = fr[4] if len(fr) > 4 else None
        held = fr[5] if len(fr) > 5 else ()
        for i in range(n):
            out = step(bottles, caps if i == 0 else None, g, tilt, held)
    return out


def loosen(deg, names=NAMES, where=TRAYED):
    """Each named cap turned from 20 degrees past the seat to `deg` back from it, in 15-degree steps, held."""
    frames = []
    for nm in names:
        a = -20.0
        while a < deg:
            a = min(a + 15.0, deg)
            frames.append((1, where, {nm: -a}, None, None, (nm,)))
    return frames


CARRY = [(5, TRAYED)]
DONE = [(40, TRAYED)]
AWAY = {"right_gripper": (0.30, -0.15, TOP + 0.15)}

cases = [
    ("nothing", [(100, START)], 0.0),
    ("bottles in the tray, caps still tight (the tempting wrong answer)", CARRY + DONE, 0.0),
    ("caps loosened 150 degrees, bottles in the tray, arms home", CARRY + loosen(150) + DONE, 1.0),
    ("caps loosened 150, arms not home", CARRY + loosen(150) + [(40, TRAYED, None, AWAY)], 0.9),
    ("caps loosened 150, bottles left on the bench", loosen(150, where=START) + [(40, START)], 0.5),
    ("caps loosened, one bottle left on the bench", CARRY + loosen(150)
     + [(40, dict(TRAYED, medium=START["medium"]))], 0.5),
    ("two caps loosened, one still tight", CARRY + loosen(150, NAMES[:2]) + DONE, 0.0),
    ("a cap loosened only 40 degrees (under 45)", CARRY + loosen(150, NAMES[:2]) + loosen(40, NAMES[2:]) + DONE, 0.0),
    ("a cap loosened exactly 45 degrees", CARRY + loosen(45) + DONE, 1.0),
    ("caps loosened 270 degrees (three quarters of a turn)", CARRY + loosen(270) + DONE, 1.0),
    ("a cap loosened 300 degrees (barely on)", CARRY + loosen(150, NAMES[:2]) + loosen(300, NAMES[2:]) + DONE, 0.25),
    ("a cap taken off (past 1.25 turns) and screwed back to 150", CARRY + loosen(150, NAMES[:2])
     + loosen(460, NAMES[2:]) + [(1, TRAYED, {"medium": -150.0}, None, None, ("medium",))] + DONE, 0.0),
    ("a cap at 440 degrees, never off (barely on)", CARRY + loosen(150, NAMES[:2]) + loosen(440, NAMES[2:]) + DONE,
     0.25),
    ("caps loosened then retightened", CARRY + loosen(150) + [(1, TRAYED, {n: 10.0 for n in NAMES}, None, None,
                                                                tuple(NAMES))] + DONE, 0.0),
    ("a bottle knocked over (tipped 60 degrees) then stood back up", CARRY + loosen(150)
     + [(3, TRAYED, None, None, {"broth": 60.0})] + DONE, 0.0),
    ("a bottle tipped 50 degrees for 1 step only", CARRY + loosen(150) + [(1, TRAYED, None, None, {"broth": 50.0})]
     + DONE, 1.0),
    ("a bottle leaning 20 degrees in the tray at the end", CARRY + loosen(150) + [(40, TRAYED, None, None,
                                                                                 {"saline": 20.0})], 0.5),
    ("a cap spun 150 degrees with no grip on it (knocked)", CARRY + [(1, TRAYED, {n: -150.0 for n in NAMES})] + DONE,
     0.0),
    ("one cap spun without a grip, the others loosened", CARRY + loosen(150, NAMES[:2])
     + [(1, TRAYED, {"medium": -150.0})] + DONE, 0.0),
    ("a bottle still held at the end", CARRY + loosen(150)
     + [(40, TRAYED, None, {"left_gripper": ((TRAYED["broth"][0], TRAYED["broth"][1], TOP + 0.05), 0.4)})], 0.5),
    ("a cap still gripped at the end", CARRY + loosen(150)
     + [(40, TRAYED, None, {"left_gripper": ((0.33, 0.11, TOP + m.TRAY_FLOOR + m.SEAT + 0.01), 0.39)})], 0.5),
    ("a closed hand resting on a cap top at the end", CARRY + loosen(150)
     + [(40, TRAYED, None, {"left_gripper": ((0.33, 0.11, TOP + m.TRAY_FLOOR + m.SEAT + m.CAP_H + 0.01), 0.0)})], 0.5),
    ("a closed hand at the bottle's foot at the end", CARRY + loosen(150)
     + [(40, TRAYED, None, {"right_gripper": ((0.33, -0.11 + 0.03, TOP + m.TRAY_FLOOR + 0.01), 0.0)})], 0.5),
    ("an open hand 1 cm above a cap at the end", CARRY + loosen(150)
     + [(40, TRAYED, None, {"left_gripper": ((0.33, 0.11, TOP + m.TRAY_FLOOR + m.SEAT + m.CAP_H + 0.01), 1.0)})], 0.9),
    ("a bottle on the tray rim (2 cm up)", CARRY + loosen(150)
     + [(40, dict(TRAYED, broth=(0.33, 0.17, TOP + 0.035)))], 0.5),
    ("a bottle just outside the tray's end wall", CARRY + loosen(150)
     + [(40, dict(TRAYED, saline=(0.33, -0.20, TOP + 0.001)))], 0.5),
    ("a bottle against the rim inside (base 1 mm from the wall)", CARRY + loosen(150)
     + [(40, dict(TRAYED, broth=(0.33, m.TRAY_Y / 2 - m.TRAY_WALL - m.BODY_R - 0.001, TOP + m.TRAY_FLOOR)))], 1.0),
    ("a bottle's base 3 mm over the rim", CARRY + loosen(150)
     + [(40, dict(TRAYED, broth=(0.33, m.TRAY_Y / 2 - m.TRAY_WALL - m.BODY_R + 0.003, TOP + m.TRAY_FLOOR)))], 0.5),
    ("a bottle still sliding in the tray at the end (2 mm a step)", CARRY + loosen(150)
     + [(1, dict(TRAYED, broth=(0.33, 0.11 - 0.002 * (k % 2), TOP + m.TRAY_FLOOR))) for k in range(40)], 0.5),
    ("done, arms at home but still moving", CARRY + loosen(150)
     + [(1, TRAYED, None, {"right_gripper": (0.18, -0.30 + 0.002 * (k % 2), TOP + 0.22)}) for k in range(40)], 0.9),
]

# The grip check itself (twistkit's accounting, as the simulator runs it): who counts as holding a cap.
def grip_turn(gripper, opening, degrees=150.0, steps=12):
    """Turn the broth cap from snug by `degrees` while the left gripper sits at `gripper` with `opening`; return the
    degrees credited as gripped."""
    kit._EPISODE["count"] += 1
    key = "broth/cap"
    cap_pos = np.add(TRAYED["broth"], (0, 0, m.SEAT))
    for k in range(steps + 1):
        t = tk._track(key)
        t.u = math.radians(-20.0 + (degrees + 20.0) * k / steps)
        objs = {key: st(cap_pos), "left_gripper": st(gripper), "right_gripper": st(PARK["right_gripper"])}
        kit._OPENING["left_gripper"], kit._OPENING["right_gripper"] = opening, 1.0
        tk._account(objs)
    return tk.gripped_deg(None, key)


CAP_MID = tuple(np.add(TRAYED["broth"], (0, 0, m.SEAT + m.CAP_H / 2)))
ON_CAP = 2 * m.CAP_R / (2 * abs(m.FINGER_OPEN))
for name, args, want_credit in [
        ("fingers closed on the cap, centered", (CAP_MID, ON_CAP), True),
        ("fingers on the cap, tool point 1 cm off the axis", (tuple(np.add(CAP_MID, (0.01, 0, 0))), ON_CAP), True),
        ("an empty closed hand resting on the cap top", (tuple(np.add(CAP_MID, (0, 0, 0.012))), 0.0), False),
        ("an empty closed hand beside the cap", (tuple(np.add(CAP_MID, (0.025, 0, 0))), 0.0), False),
        ("an open hand around the cap", (CAP_MID, 1.0), False),
        ("fingers at cap width but 2.5 cm to the side", (tuple(np.add(CAP_MID, (0.025, 0, 0))), ON_CAP), False)]:
    got = grip_turn(*args)
    ok = (got > 100) == want_credit
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"credited {got:.0f} degrees:", name)

for name, frames, want in cases:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

got, ok = run(CARRY + loosen(150) + DONE)
bad += not ok
print("ok  " if ok else "FAIL", "success once done and the arms home")
got, ok = run(CARRY + loosen(150) + [(40, TRAYED, None, AWAY)])
bad += ok
print("ok  " if not ok else "FAIL", "no success with an arm away from home")
# a reset clears an earlier trial's cap removal and the threads' records
run(CARRY + loosen(460))
got, _ = run(CARRY + loosen(150) + DONE)
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", got, "a reset clears an earlier trial's removed cap")
got, _ = run([(40, START)])
bad += got != 0.0 or abs(tk.past_seat_deg(None, "broth/cap") - 20.0) > 1e-9
print("ok  " if got == 0.0 else "FAIL", got, "after a reset the caps are tight again (20 degrees past the seat)")
m.REQUIRE_ARMS_HOME = False
got, _ = run(CARRY + loosen(150) + [(40, TRAYED, None, AWAY)])
m.REQUIRE_ARMS_HOME = True
bad += got != 1.0
print("ok  " if got == 1.0 else "FAIL", got, "REQUIRE_ARMS_HOME off: done, arm away, scores 1.0")
print("all pass" if not bad else f"{bad} FAILED")
