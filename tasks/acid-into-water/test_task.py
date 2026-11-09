"""Offline checks of the acid-dilution scorer and the pour model: step-by-step container poses, no simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, pk = m.kit, m.pk
HOME = {"left_gripper": (0.20, 0.27, 0.95), "right_gripper": (0.20, -0.27, 0.95)}
BEAKER, BOTTLE = np.array(m.WATER_POS), np.array(m.ACID_POS)
BEAKER_RIM = m.FLOOR_Z + m.BEAKER_H


def quat_y(deg):
    a = math.radians(deg) / 2
    return np.array([0.0, math.sin(a), 0.0, math.cos(a)])


def st(pos, deg=0.0, half=(0.02, 0.02, 0.045), speed=0.0):
    p = np.array(pos, float)
    h = np.array(half)
    return ObjectState(pos=p, quat_xyzw=quat_y(deg), lin_vel=np.array([speed, 0, 0]), ang_vel=np.zeros(3),
                       low=p - h, high=p + h)


def tipped(name, lip_at, deg):
    """Container `name` tipped `deg` toward +x about its lip (the rim point on the +x side), lip at `lip_at`."""
    v = pk._CFG["vessels"][name]
    lip_local = np.array([v.mouth_r, 0.0, v.floor + v.height])
    return np.asarray(lip_at, float) - pk._rotm(quat_y(deg)) @ lip_local


def frame(bottle=None, beaker=None, grip=None, opening=1.0, bdeg=0.0, kdeg=0.0):
    bottle = BOTTLE if bottle is None else bottle
    beaker = BEAKER if beaker is None else beaker
    grippers = dict(HOME)
    if grip is not None:
        grippers["right_gripper"] = grip
    return {"_opening": opening,
            "tray": st((*m.TRAY_C, m.TOP), half=(0.15, 0.13, 0.0)),
            "water_beaker": st(beaker, kdeg, half=(m.BEAKER_R, m.BEAKER_R, m.BEAKER_H / 2)),
            "acid_beaker": st(bottle, bdeg, half=(m.BEAKER_R, m.BEAKER_R, m.BEAKER_H / 2)),
            **{g: st(p, half=(0.01, 0.01, 0.01)) for g, p in grippers.items()}}


def run(frames):
    """frames: list of (steps, objects). A fresh trial each time (the kit's reset counter moves on)."""
    kit._EPISODE["count"] += 1
    out = (0.0, False)
    for n, objs in [(2, frame())] + frames:  # each trial starts with the arms home
        kit._OPENING.update({"left_gripper": 1.0, "right_gripper": objs["_opening"]})
        objs = {k: v for k, v in objs.items() if k != "_opening"}
        for _ in range(n):
            pk.update(objs)  # in the sim the scene hook does this before success() and score()
            ok = m.success(objs)
            out = (m.score(objs), ok)
    return out


def pour_bottle(lip_xy, lip_dz, peak=110.0, held=True, n=6):
    """Tip the bottle about its lip to `peak` degrees in `n` stages, hold, tip back and put it away."""
    lip = (*lip_xy, BEAKER_RIM + lip_dz)
    f = []
    for k in range(1, n + 1):
        d = 60 + (peak - 60) * k / n
        f.append((10, frame(bottle=tipped("acid_beaker", lip, d), bdeg=d, grip=lip, opening=0.0 if held else 1.0)))
    f.append((40, frame(bottle=tipped("acid_beaker", lip, peak), bdeg=peak, grip=lip, opening=0.0 if held else 1.0)))
    return f


def pour_beaker_into_bottle(peak=80.0):
    lip = (BOTTLE[0] - 0.005, BOTTLE[1], m.FLOOR_Z + m.BEAKER_H + 0.03)
    return [(60, frame(beaker=tipped("water_beaker", lip, peak), kdeg=peak, grip=lip, opening=0.0))]


PUT_AWAY = [(5, frame(grip=(0.23, -0.12, 0.79), opening=1.0)), (20, frame())]
LIFTED = BOTTLE + np.array([0, 0, 0.1])
CENTER = (BEAKER[0] - 0.008, BEAKER[1])

CASES = [
    ("nothing done", [(40, frame())], 0.0),
    ("acid poured slowly into the water, acid beaker put back, arms home", pour_bottle(CENTER, 0.035) + PUT_AWAY, 1.0),
    ("same, but the right arm left beside the acid beaker",
     pour_bottle(CENTER, 0.035) + [(20, frame(grip=(0.23, -0.20, 0.79), opening=1.0))], 0.9),
    ("acid in, acid beaker still held up in the gripper",
     pour_bottle(CENTER, 0.035) + [(20, frame(bottle=LIFTED, grip=LIFTED, opening=0.0))], 0.75),
    ("acid in, then the acid beaker let go lying on its side",
     pour_bottle(CENTER, 0.035) + [(20, frame(bdeg=90.0))], 0.0),
    ("acid in, acid beaker set down leaning 30 degrees", pour_bottle(CENTER, 0.035) + [(20, frame(bdeg=30.0))], 0.75),
    ("acid in, acid beaker set down outside the tray",
     pour_bottle(CENTER, 0.035) + [(20, frame(bottle=BOTTLE + np.array([0, -0.12, -0.005])))], 0.75),
    ("only tipped to 85 degrees: part of the acid in", pour_bottle(CENTER, 0.035, peak=85.0) + PUT_AWAY, 0.5),
    ("acid poured from 15 cm above the rim (splashes)", pour_bottle(CENTER, 0.15) + PUT_AWAY, 0.25),
    ("acid poured beside the water beaker, onto the tray", pour_bottle((BEAKER[0] + 0.06, BEAKER[1]), 0.035) + PUT_AWAY,
     0.0),
    ("half the stream misses: lip just past the rim",
     pour_bottle((BEAKER[0] - m.BEAKER_IN + 0.004, BEAKER[1]), 0.035, peak=88.0)
     + pour_bottle((BEAKER[0] + 0.07, BEAKER[1]), 0.035) + PUT_AWAY, 0.25),
    ("water poured into the acid (the tempting wrong order)", pour_beaker_into_bottle() + PUT_AWAY, 0.0),
    ("a little water into the acid, then the acid into the water",
     pour_beaker_into_bottle(peak=65.0) + PUT_AWAY + pour_bottle(CENTER, 0.035) + PUT_AWAY, 0.0),
    # the acid beaker is empty by then, so this is not water into acid; it only takes acid back out of the water
    ("acid into the water, then most of the mix poured back into the acid beaker",
     pour_bottle(CENTER, 0.035) + PUT_AWAY + pour_beaker_into_bottle() + PUT_AWAY, 0.5),
    ("acid beaker knocked over so it empties into the water beaker",
     pour_bottle(CENTER, 0.035, held=False) + PUT_AWAY, 0.0),
    ("acid beaker knocked over on the tray", [(40, frame(bottle=BOTTLE - np.array([0, 0, 0.02]), bdeg=95.0))], 0.0),
    ("water beaker knocked over, then acid poured in what is left",
     [(30, frame(kdeg=95.0)), (5, frame())] + pour_bottle(CENTER, 0.035) + PUT_AWAY, 0.25),
]

bad = 0
# the wrong-order cases really put water into the acid (they are 0 for that reason, not for lack of a pour)
for frames in (pour_beaker_into_bottle(), pour_beaker_into_bottle(peak=65.0)):
    run(frames)
    ok = pk.volume(None, "acid_beaker", "water") >= m.WATER_IN and m._H.water_into_acid
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"water reached the acid ({pk.volume(None, 'acid_beaker', 'water'):.1f} mL) and "
          "the violation latched")
for name, frames, want in CASES:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name,
          f"[acid in {pk.volume(None, 'water_beaker', 'acid'):.1f} mL, spilled {pk.spilled():.1f} mL]")

# the pour history is ordered and names who poured into whom
run(pour_beaker_into_bottle(peak=65.0) + PUT_AWAY + pour_bottle(CENTER, 0.035) + PUT_AWAY)
order = [(p.source, p.dest) for p in pk.pours()]
ok = order[:2] == [("water_beaker", "acid_beaker"), ("acid_beaker", "water_beaker")]
bad += not ok
print(("ok  " if ok else "FAIL"), "pour history in order:", order)

# a violation in one trial does not carry into the next
run(pour_beaker_into_bottle())
got, _ = run(pour_bottle(CENTER, 0.035) + PUT_AWAY)
ok = got == 1.0
bad += not ok
print(("ok  " if ok else "FAIL"), "a new trial starts with full containers and a clean history")

# liquid is conserved
run(pour_bottle(CENTER, 0.035) + pour_beaker_into_bottle())
total = sum(pk.volume(None, n) for n in ("water_beaker", "acid_beaker")) + pk.spilled()
ok = abs(total - (m.ACID0 + m.WATER0)) < 1e-6
bad += not ok
print(("ok  " if ok else "FAIL"), f"liquid conserved ({total:.6f} mL)")

print("all pass" if bad == 0 else f"{bad} FAILED")
raise SystemExit(bad)
