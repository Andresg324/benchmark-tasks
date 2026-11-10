"""Offline checks of the decanting scorer and the sediment pour model: step-by-step container poses, no simulator.

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
BOTTLE, DECANTER = np.array(m.BOTTLE_POS), np.array(m.DECANTER_POS)
RIM = m.TOP + m.D_H


def quat_y(deg):
    a = math.radians(deg) / 2
    return np.array([0.0, math.sin(a), 0.0, math.cos(a)])


def st(pos, deg=0.0, half=(0.02, 0.02, 0.045), speed=0.0):
    p = np.array(pos, float)
    h = np.array(half)
    return ObjectState(pos=p, quat_xyzw=quat_y(deg), lin_vel=np.array([speed, 0, 0]), ang_vel=np.zeros(3),
                       low=p - h, high=p + h)


def tipped(lip_at, deg):
    """The bottle tipped `deg` toward +x about its lip (the rim point on the +x side), lip at `lip_at`."""
    v = pk._CFG["vessels"]["bottle"]
    lip_local = np.array([v.mouth_r, 0.0, v.floor + v.height])
    return np.asarray(lip_at, float) - pk._rotm(quat_y(deg)) @ lip_local


def frame(bottle=None, grip=None, opening=1.0, bdeg=0.0, ddeg=0.0):
    bottle = BOTTLE if bottle is None else bottle
    grippers = dict(HOME)
    if grip is not None:
        grippers["right_gripper"] = grip
    return {"_opening": opening,
            "bottle": st(bottle, bdeg, half=(m.B_R, m.B_R, m.B_H / 2)),
            "decanter": st(DECANTER, ddeg, half=(m.D_R, m.D_R, m.D_H / 2)),
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


def pour(lip_xy, lip_dz, peak=90.0, held=True, n=8):
    """Tip the bottle about its lip to `peak` degrees in `n` stages, hold 3 s."""
    lip = (*lip_xy, RIM + lip_dz)
    o = 0.0 if held else 1.0
    f = [(10, frame(bottle=tipped(lip, 60 + (peak - 60) * k / n), bdeg=60 + (peak - 60) * k / n, grip=lip, opening=o))
         for k in range(1, n + 1)]
    return f + [(60, frame(bottle=tipped(lip, peak), bdeg=peak, grip=lip, opening=o))]


def tilt_frames(degs, lip_xy=None, lip_dz=0.05, held=True):
    """One control step per tilt in `degs`, the bottle tipped about its lip held over `lip_xy`."""
    lip = (*(CENTER if lip_xy is None else lip_xy), RIM + lip_dz)
    o = 0.0 if held else 1.0
    return [(1, frame(bottle=tipped(lip, d), bdeg=d, grip=lip, opening=o)) for d in degs]


def watched(rate=0.2, stop=0.6, back=3.0, lip_xy=None, lip_dz=0.05):
    """A watch-and-stop pour: tip from 60 degrees by `rate` degrees per step while watching the sediment, and as soon
    as its front is `stop` of the way to the shoulder tip back by `back` degrees per step to 60. Closed loop, so it
    runs the liquids itself; returns the tilts it used, to replay as frames."""
    kit._EPISODE["count"] += 1
    lip = (*(CENTER if lip_xy is None else lip_xy), RIM + lip_dz)
    degs, d, stopping = [], 60.0, False
    while d >= 60.0 and len(degs) < 2000:
        objs = {k: v for k, v in frame(bottle=tipped(lip, d), bdeg=d, grip=lip, opening=0.0).items() if k != "_opening"}
        pk.update(objs)
        degs.append(d)
        if not stopping:
            d = min(d + rate, 100.0)
            stopping = pk.front(None, "bottle") >= stop
        else:
            d -= back
    return degs


def blind(peak, hold, rate=0.5):
    """Tip from 60 to `peak` degrees at `rate` degrees per step, hold `hold` steps, tip back at 3 degrees per step:
    no looking at the sediment."""
    up = [60 + rate * i for i in range(int((peak - 60) / rate))]
    down = [peak - 3.0 * i for i in range(1, int((peak - 60) / 3.0) + 1)]
    return up + [peak] * hold + down


SET_DOWN = [(5, frame(grip=(0.23, -0.12, m.TOP + 0.055), opening=1.0)), (20, frame())]
LIFTED = BOTTLE + np.array([0, 0, 0.1])
CENTER = (DECANTER[0] - 0.008, DECANTER[1])
BESIDE = (DECANTER[0] + 0.08, DECANTER[1])
DECANT_DEGS = watched()
DECANT = tilt_frames(DECANT_DEGS)  # the reference careful pour: watch the sediment, stop at 0.6 of the way

CASES = [
    ("nothing done", [(40, frame())], 0.0),
    ("decanted watching the sediment, bottle set down, arms home", DECANT + SET_DOWN, 1.0),
    ("same, but the right arm left beside the bottle",
     DECANT + [(20, frame(grip=(0.23, -0.20, 0.79), opening=1.0))], 0.9),
    ("decanted, bottle still held up in the gripper at the end",
     DECANT + [(20, frame(bottle=LIFTED, grip=LIFTED, opening=0.0))], 0.75),
    ("decanted, bottle still gripped standing on its spot", DECANT
     + [(20, frame(grip=(0.23, -0.12, m.TOP + 0.055), opening=0.0))], 0.75),
    ("decanted, bottle set down leaning 30 degrees", DECANT + [(20, frame(bdeg=30.0))], 0.75),
    ("decanted, bottle let go lying on its side", DECANT + [(20, frame(bdeg=90.0))], 0.0),
    ("poured to the last drop (110 degrees): sediment in the decanter", pour(CENTER, 0.05, peak=110.0) + SET_DOWN, 0.0),
    ("poured just past the shoulder (96 degrees)", pour(CENTER, 0.05, peak=96.0) + SET_DOWN, 0.0),
    ("decanted well, then tipped again to the end", DECANT + SET_DOWN
     + pour(CENTER, 0.05, peak=110.0) + SET_DOWN, 0.0),
    ("stopped far too early (82 degrees): part of the wine in", pour(CENTER, 0.05, peak=82.0) + SET_DOWN, 0.5),
    ("poured from 15 cm above the rim (splashes)", tilt_frames(watched(lip_dz=0.15), lip_dz=0.15) + SET_DOWN, 0.25),
    ("poured beside the decanter, onto the table", tilt_frames(watched(lip_xy=BESIDE), lip_xy=BESIDE)
     + SET_DOWN, 0.0),
    ("bottle knocked over on the table", [(40, frame(bottle=BOTTLE - np.array([0, 0, 0.02]), bdeg=95.0))], 0.0),
    ("bottle knocked over so it empties into the decanter", tilt_frames(DECANT_DEGS, held=False) + SET_DOWN, 0.0),
    ("decanter knocked over and stood back up, then decanted", [(30, frame(ddeg=95.0)), (5, frame())]
     + DECANT + SET_DOWN, 0.0),
    ("arm hovers over the decanter, nothing poured", [(40, frame(grip=(DECANTER[0], DECANTER[1], RIM + 0.03)))], 0.0),
    ("watch-and-stop, tipping slowly (0.1 degree per step)", tilt_frames(watched(rate=0.1)) + SET_DOWN, 1.0),
    # tipping fast runs past 93 degrees (neck below horizontal) before enough wine is out: it glugs, the sediment is
    # churned up early, and the watcher has to stop short
    ("watch-and-stop, tipping fast (0.8 degree per step): glugs, stops short", tilt_frames(watched(rate=0.8)) + SET_DOWN,
     0.5),
    # a blind flip neck-down for a counted time: glugging churns the sediment up whatever the timing
    ("blind flip to 110 degrees, held 64 steps", tilt_frames([60.0, 110.0] + [110.0] * 64 + [60.0]) + SET_DOWN, 0.0),
    ("blind flip to 150 degrees, held 60 steps", tilt_frames([60.0, 150.0] + [150.0] * 60 + [60.0]) + SET_DOWN, 0.0),
    ("blind flip to 95 degrees, held 64 steps", tilt_frames([60.0, 95.0] + [95.0] * 64 + [60.0]) + SET_DOWN, 0.0),
    ("watch-and-stop, slow, tipping back once the sediment is 0.9 of the way to the shoulder",
     tilt_frames(watched(rate=0.1, stop=0.9)) + SET_DOWN, 1.0),
    ("watch-and-stop on the drawn cue (the sediment's edge entering the shoulder), tipping at 0.4 degree per step",
     tilt_frames(watched(rate=0.4, stop=pk.LEAD_FROM)) + SET_DOWN, 1.0),
    ("stopped as soon as the sediment began to move (0.2 of the way): too little wine in",
     tilt_frames(watched(rate=0.1, stop=0.2)) + SET_DOWN, 0.5),
    ("over-pour: watched, but kept tipping until the sediment was at the shoulder",
     tilt_frames(watched(stop=1.2, back=1.5)) + SET_DOWN, 0.0),
    ("over-pour: blind tip to 89 degrees and hold 3 s", tilt_frames(blind(89, 60)) + SET_DOWN, 0.0),
]

bad = 0
for name, frames, want in CASES:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name,
          f"[wine in {pk.volume(None, 'decanter', 'wine'):.1f} mL, sediment in "
          f"{pk.volume(None, 'decanter', 'sediment'):.2f} mL, spilled {pk.spilled():.1f} mL]")

# a blind fixed-tilt hold (no looking at the sediment) does not reliably score 1.0: over peaks 86-92 degrees, holds of
# 0-5 s and three tipping speeds, it scores 1.0 in under a quarter of the tries, and no (peak, hold) does so at every
# tipping speed
grid = {(p, h, r): run(tilt_frames(blind(p, h, r)) + SET_DOWN)[0]
        for p in (86, 87, 88, 89, 90, 92) for h in (0, 20, 40, 60, 100) for r in (0.15, 0.5, 1.5)}
full = sum(v == 1.0 for v in grid.values()) / len(grid)
robust = [(p, h) for p in (86, 87, 88, 89, 90, 92) for h in (0, 20, 40, 60, 100)
          if all(grid[(p, h, r)] == 1.0 for r in (0.15, 0.5, 1.5))]
ok = full < 0.25 and not robust
bad += not ok
print(("ok  " if ok else "FAIL"), f"blind fixed-tilt holds: {full:.0%} score 1.0; robust to tipping speed: {robust}")

# the sediment stays on the bottle's floor while 50 mL or more of clear wine covers it, however long the hold
run(pour(CENTER, 0.05, peak=84.0) + [(200, frame(bottle=tipped((*CENTER, RIM + 0.05), 84.0), bdeg=84.0,
                                                  grip=(*CENTER, RIM + 0.05), opening=0.0))])
ok = (pk.volume(None, "decanter", "sediment") == 0.0 and pk.front(None, "bottle") == 0.0
      and pk.volume(None, "bottle", "wine") >= m.SLIDE_BELOW)
bad += not ok
print(("ok  " if ok else "FAIL"), f"sediment stays put while {m.SLIDE_BELOW:.0f} mL or more of wine covers it "
      f"({pk.volume(None, 'bottle', 'wine'):.1f} mL left after a 13 s hold at 84 degrees)")
# tipped back under 75 degrees, the sediment stops where it is
degs = watched()
f0 = pk.front(None, "bottle")
run(tilt_frames(degs) + [(200, frame(bottle=tipped((*CENTER, RIM + 0.05), 70.0), bdeg=70.0,
                                     grip=(*CENTER, RIM + 0.05), opening=0.0))])
ok = abs(pk.front(None, "bottle") - f0) < 1e-9 and pk.volume(None, "decanter", "sediment") == 0.0
bad += not ok
print(("ok  " if ok else "FAIL"), f"held at 70 degrees after the pour, the sediment stays at {f0:.2f} of the way")
run(pour(CENTER, 0.05, peak=110.0))
ok = pk.volume(None, "decanter", "sediment") > 0.8 * m.SEDIMENT0
bad += not ok
print(("ok  " if ok else "FAIL"), f"poured out to the end, the sediment follows "
      f"({pk.volume(None, 'decanter', 'sediment'):.1f} of {m.SEDIMENT0} mL)")

# a violation in one trial does not carry into the next (kit.episode() resets the liquids and the history)
run(pour(CENTER, 0.05, peak=110.0) + [(20, frame(bdeg=95.0))])
got, _ = run(DECANT + SET_DOWN)
ok = got == 1.0 and not m._H.knocked
bad += not ok
print(("ok  " if ok else "FAIL"), "a new trial starts with a full bottle and a clean history")

# liquid is conserved
run(pour(CENTER, 0.05, peak=110.0) + pour((DECANTER[0] + 0.08, DECANTER[1]), 0.05))
total = sum(pk.volume(None, n) for n in ("bottle", "decanter")) + pk.spilled()
ok = abs(total - (m.WINE0 + m.SEDIMENT0)) < 1e-6
bad += not ok
print(("ok  " if ok else "FAIL"), f"liquid conserved ({total:.6f} mL)")

print("all pass" if bad == 0 else f"{bad} FAILED")
raise SystemExit(bad)
