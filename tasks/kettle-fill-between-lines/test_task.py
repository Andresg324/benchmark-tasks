"""Offline checks of the kettle scorer and the pour model: step-by-step poses of the jug, the lid and the switch, no
simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, pk, mk = m.kit, m.pk, m.mk
HOME = {"left_gripper": (0.20, 0.27, 0.95), "right_gripper": (0.20, -0.27, 0.95)}
JUG = np.array(m.JUG_POS)
KETTLE = np.array((m.KX, m.KY, m.TOP))
RIM_Z = m.TOP + m.RIM
LINKS = {lk.name: lk for lk in m.OBJECTS[0].links}
OPEN, SHUT = math.degrees(-m.LID_OPEN), 0.0  # lid opening, degrees
OFF, ON = math.degrees(m.SW_OFF), math.degrees(m.SW_ON)  # switch lever, degrees


def quat_y(deg):
    a = math.radians(deg) / 2
    return np.array([0.0, math.sin(a), 0.0, math.cos(a)])


def st(pos, deg=0.0, half=(0.02, 0.02, 0.045)):
    p = np.array(pos, float)
    h = np.array(half)
    return ObjectState(pos=p, quat_xyzw=quat_y(deg), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - h, high=p + h)


def tipped(lip_at, deg):
    """The jug tipped `deg` toward +x about its lip (the rim point on the +x side), lip at `lip_at`."""
    v = pk._CFG["vessels"]["jug"]
    lip_local = np.array([v.mouth_r, 0.0, v.floor + v.height])
    return np.asarray(lip_at, float) - pk._rotm(quat_y(deg)) @ lip_local


def frame(jug=None, jdeg=0.0, lid=OPEN, switch=OFF, grip=None, opening=1.0, left=None, labels=None):
    """One step's scene. `lid` is how far the lid is open and `switch` the lever angle, degrees. `labels` overrides
    the detent labels the kit would read (a stale or wrong flag)."""
    grippers = dict(HOME)
    if grip is not None:
        grippers["right_gripper"] = grip
    if left is not None:
        grippers["left_gripper"] = left
    return {"_opening": opening, "_lid": -math.radians(lid), "_switch": math.radians(switch), "_labels": labels or {},
            "kettle": st(KETTLE, half=(m.BASE_R, m.BASE_R, 0.0)),
            "coaster": st((*m.JUG_XY, m.TOP + m.COASTER_H / 2), half=(m.COASTER_R, m.COASTER_R, m.COASTER_H / 2)),
            "jug": st(JUG if jug is None else jug, jdeg, half=(m.JUG_R, m.JUG_R, m.JUG_H / 2)),
            **{g: st(p, half=(0.01, 0.01, 0.01)) for g, p in grippers.items()}}


def set_joints(objs):
    """What yamkit does each step before the scorer: the joints, and each detent law's memory from its joint."""
    for name in ("lid", "switch"):
        key, lk, q = f"kettle/{name}", LINKS[name], objs[f"_{name}"]
        kit._JOINTS[key] = kit.JointState(pos=q, vel=0.0, latched=False)
        mem = kit._law_mem(key, lk)
        lk.law.drive(key, q, 0.0, {}, mem)
        if name in objs["_labels"]:
            mem["label"] = objs["_labels"][name]


def run(frames):
    """frames: list of (steps, objects). A fresh trial each time (the kit's reset counter moves on)."""
    kit._EPISODE["count"] += 1
    out = (0.0, False)
    for n, objs in [(2, frame())] + frames:  # each trial starts with the arms home
        kit._OPENING.update({"left_gripper": 1.0, "right_gripper": objs["_opening"]})
        set_joints(objs)
        objs = {k: v for k, v in objs.items() if not k.startswith("_")}
        for _ in range(n):
            pk.update(objs)  # in the sim the scene hook does this before success() and score()
            ok = m.success(objs)
            out = (m.score(objs), ok)
    return out


CENTER = (m.KX - 0.008, m.KY)


def pour(peak=74.5, lid=OPEN, switch=OFF, lip_xy=CENTER, lip_dz=0.05, held=True, n=6):
    """Tip the jug about its lip (`lip_dz` over the kettle's rim) from 60 degrees to `peak` in `n` stages, hold it
    there, then tip it back upright over the kettle."""
    lip = (*lip_xy, RIM_Z + lip_dz)
    o = 0.0 if held else 1.0
    f = []
    for k in range(1, n + 1):
        d = 60 + (peak - 60) * k / n
        f.append((10, frame(jug=tipped(lip, d), jdeg=d, lid=lid, switch=switch, grip=lip, opening=o)))
    f.append((40, frame(jug=tipped(lip, peak), jdeg=peak, lid=lid, switch=switch, grip=lip, opening=o)))
    f.append((10, frame(jug=tipped(lip, 40), jdeg=40, lid=lid, switch=switch, grip=lip, opening=o)))
    return f


def jug_back(lid=OPEN, switch=OFF):
    return [(5, frame(lid=lid, switch=switch, grip=(0.23, -0.12, 0.81), opening=1.0)),
            (5, frame(lid=lid, switch=switch))]


def close_lid(switch=OFF):
    return [(5, frame(lid=a, switch=switch, left=(m.KX, 0.06, 0.87))) for a in (100, 80, 40, 3)] + \
           [(5, frame(lid=SHUT, switch=switch))]


def press(lid=SHUT):
    return [(3, frame(lid=lid, switch=a, left=(0.29, 0.03, 0.80))) for a in (8, 0, -6, -12)] + \
           [(20, frame(lid=lid, switch=ON))]


GOOD = pour() + jug_back() + close_lid() + press()

CASES = [
    ("nothing done", [(40, frame())], 0.0),
    ("filled between the lines, jug back, lid shut, switched on, arms home", GOOD, 1.0),
    ("same, but the left arm left resting on the switch",
     GOOD + [(20, frame(lid=SHUT, switch=ON, left=(0.29, 0.03, 0.80)))], 0.9),
    ("switched on first (dry), then filled and the lid shut, switch still on",
     close_lid() + press() + [(5, frame(lid=OPEN, switch=ON))] + pour(switch=ON) + jug_back(switch=ON)
     + close_lid(switch=ON), 0.0),
    ("switched on while still nearly empty, lid shut", close_lid() + press(), 0.0),
    ("switch pressed with the lid still open (filled right)", pour() + jug_back() + press(lid=OPEN), 0.0),
    ("filled, lid shut, switched on, then the lid opened again", GOOD + [(10, frame(lid=OPEN, switch=ON))], 0.0),
    ("the whole jug poured in (over MAX), lid shut, switched on",
     pour(peak=100) + jug_back() + close_lid() + press(), 0.0),
    ("the whole jug poured in (over MAX), never switched on", pour(peak=100) + jug_back() + close_lid(), 0.0),
    ("too little poured (under MIN), lid shut, switched on", pour(peak=63) + jug_back() + close_lid() + press(), 0.0),
    ("filled between the lines and lid shut, but never switched on", pour() + jug_back() + close_lid(), 0.25),
    ("switched on, then switched off again", GOOD + [(20, frame(lid=SHUT, switch=OFF))], 0.25),
    ("lid shut first, then the water poured onto the lid (runs off)",
     close_lid() + pour(lid=SHUT) + jug_back(lid=SHUT) + press(), 0.0),
    ("poured from 15 cm over the rim (splashes), otherwise right",
     pour(lip_dz=0.15) + jug_back() + close_lid() + press(), 0.25),
    ("half the stream misses the opening",
     pour(peak=70, lip_xy=(m.KX - m.IN_R - 0.002, m.KY)) + pour(peak=76) + jug_back() + close_lid() + press(), 0.25),
    ("filled, lid, on, but the jug still held up in the gripper",
     pour() + close_lid() + press() + [(20, frame(jug=JUG + [0, 0, 0.1], lid=SHUT, switch=ON, grip=JUG + [0, 0, 0.1],
                                                  opening=0.0))], 0.75),
    ("filled, lid, on, jug set down on the table beside its coaster",
     pour() + jug_back() + close_lid() + press() + [(20, frame(jug=JUG + [0.0, 0.08, -0.004], lid=SHUT, switch=ON))],
     0.75),
    ("filled, lid, on, then the jug let go lying on its side",
     pour() + jug_back() + close_lid() + press() + [(20, frame(jdeg=90.0, lid=SHUT, switch=ON))], 0.0),
    ("jug let go mid-pour, falls over into the kettle", pour(held=False) + jug_back() + close_lid() + press(), 0.0),
    ("kit flag says the lid is CLOSED but it hangs 20 degrees open: not shut when switched on",
     pour() + jug_back() + close_lid() + [(5, frame(lid=20, labels={"lid": "CLOSED"}))]
     + [(20, frame(lid=20, switch=ON, labels={"lid": "CLOSED"}))], 0.0),
    ("kit flag says ON but the lever is still up: not switched on",
     pour() + jug_back() + close_lid() + [(20, frame(lid=SHUT, switch=OFF, labels={"switch": "ON"}))], 0.25),
    ("one-step glitch of the switch while empty, then done right",
     [(1, frame(switch=ON)), (5, frame())] + GOOD, 1.0),
]

bad = 0
for name, frames, want in CASES:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name,
          f"[kettle {pk.volume(None, 'kettle'):.1f} mL, jug {pk.volume(None, 'jug'):.1f} mL, spill {pk.spilled():.1f}]")

# the good fill lands between the lines with margin, and the demo's pour pitch keeps that margin
run(pour())
w = pk.volume(None, "kettle")
ok = m.MIN_ML + 20 < w < m.MAX_ML - 20
bad += not ok
print(("ok  " if ok else "FAIL"), f"a 74.5 degree pour leaves {w:.1f} mL in the kettle (lines {m.MIN_ML}-{m.MAX_ML})")
for d in (69.5, 79.5):  # 5 degrees either way still lands between the lines
    run(pour(peak=d))
    w = pk.volume(None, "kettle")
    ok = m.MIN_ML < w < m.MAX_ML
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"a {d} degree pour leaves {w:.1f} mL (still between the lines)")

# a closed lid really blocks the stream: everything poured on it is spilled, the kettle gains nothing
run(close_lid() + pour(lid=SHUT))
ok = abs(pk.volume(None, "kettle") - m.KETTLE0) < 1e-9 and pk.spilled() > 10
bad += not ok
print(("ok  " if ok else "FAIL"), f"closed lid: kettle stays at {pk.volume(None, 'kettle'):.1f} mL, "
      f"{pk.spilled():.1f} mL ran off")

# switching on before filling is an order violation that a later fill does not undo
run(close_lid() + press() + [(5, frame(lid=OPEN, switch=ON))] + pour(switch=ON))
ok = m._H.dry and m._H.boiled and m._H.water_on < m.MIN_ML
bad += not ok
print(("ok  " if ok else "FAIL"), f"switch-on recorded with {m._H.water_on:.1f} mL; the dry boil latched")

# a violation in one trial does not carry into the next
run(close_lid() + press())
got, _ = run(GOOD)
ok = got == 1.0
bad += not ok
print(("ok  " if ok else "FAIL"), "a new trial starts with the kettle nearly empty and a clean history")

# water is conserved
run(pour(peak=100) + jug_back())
total = pk.volume(None, "kettle") + pk.volume(None, "jug") + pk.spilled()
ok = abs(total - (m.KETTLE0 + m.JUG0)) < 1e-6
bad += not ok
print(("ok  " if ok else "FAIL"), f"water conserved ({total:.6f} mL)")

print("all pass" if bad == 0 else f"{bad} FAILED")
raise SystemExit(bad)
