"""Offline checks of the pitcher-pump scorer, the handle's stroke law and the water model: step-by-step poses of the
cup, the handle and the grippers, no simulator.

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
CUP = np.array(m.CUP_POS)
PUMP = np.array((m.PX, m.PY, m.TOP))
HANDLE = m.OBJECTS[0].links[0]
UP, DOWN = math.degrees(m.H_UP), math.degrees(m.H_DOWN)
OVER = (m.PX - 0.008, m.PY, m.TOP + m.RIM + 0.05)  # where the demo holds the cup's lip


def quat_y(deg):
    a = math.radians(deg) / 2
    return np.array([0.0, math.sin(a), 0.0, math.cos(a)])


def st(pos, deg=0.0, half=(0.02, 0.02, 0.02)):
    p = np.array(pos, float)
    h = np.array(half)
    return ObjectState(pos=p, quat_xyzw=quat_y(deg), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - h, high=p + h)


def tipped(lip_at, deg):
    """The cup's center when tipped `deg` toward +x about its lip (the rim point on the +x side) at `lip_at`."""
    v = pk._CFG["vessels"]["cup"]
    return np.asarray(lip_at, float) - pk._rotm(quat_y(deg)) @ np.array([v.mouth_r, 0.0, v.floor + v.height])


def frame(cup=None, cdeg=0.0, handle=UP, right=None, left=None, ropen=1.0, lopen=1.0):
    """One step's scene: the cup's center and tilt, the handle angle (degrees, up positive), the grippers' tool
    points and how open they are (1 open, 0 closed)."""
    g = dict(HOME)
    if right is not None:
        g["right_gripper"] = right
    if left is not None:
        g["left_gripper"] = left
    half = np.array((m.CUP_R, m.CUP_R, m.CUP_H / 2))
    c = CUP if cup is None else np.asarray(cup, float)
    ext = half if abs(cdeg) < 45 else half[[2, 1, 0]]  # lying on its side, the box is long in x
    return {"_open": {"right_gripper": ropen, "left_gripper": lopen}, "_handle": math.radians(handle),
            "pump": st(PUMP, half=(0.05, 0.05, 0.0)),
            "pail": st(m.PAIL_POS, half=(m.PAIL_R, m.PAIL_R, m.PAIL_H / 2)),
            "cup": ObjectState(pos=c, quat_xyzw=quat_y(cdeg), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                               low=c - ext, high=c + ext),
            **{k: st(p, half=(0.01, 0.01, 0.01)) for k, p in g.items()}}


def run(frames):
    """frames: list of (steps, objects). A fresh trial each time (the kit's reset counter moves on). Each step does
    what the sim does before the scorer: the handle's joint and its law, then the water."""
    kit._EPISODE["count"] += 1
    out = (0.0, False)
    for n, objs in [(2, frame())] + frames:  # each trial starts with the arms home
        kit._OPENING.update(objs["_open"])
        q = objs["_handle"]
        o = {k: v for k, v in objs.items() if not k.startswith("_")}
        for _ in range(n):
            kit._JOINTS["pump/handle"] = kit.JointState(pos=q, vel=0.0, latched=False)
            HANDLE.law.drive("pump/handle", q, 0.0, {"handle": q}, kit._law_mem("pump/handle", HANDLE))
            pk.update(o)
            ok = m.success(o)
            out = (m.score(o), ok)
    return out


def pour(peak=88.0, over=OVER, n=6, dest_ok=True):
    """The right hand tips the cup about its lip at `over` from 50 degrees to `peak`, holds, and tips it back."""
    f = []
    for k in range(1, n + 1):
        d = 50 + (peak - 50) * k / n
        f.append((6, frame(cup=tipped(over, d), cdeg=d, right=tipped(over, d), ropen=0.0)))
    f.append((30, frame(cup=tipped(over, peak), cdeg=peak, right=tipped(over, peak), ropen=0.0)))
    f.append((6, frame(cup=tipped(over, 40), cdeg=40, right=tipped(over, 40), ropen=0.0)))
    return f


def cup_back():
    """Set the cup down on its spot, let go and go home."""
    return [(5, frame(right=CUP, ropen=0.0)), (5, frame(right=CUP, ropen=1.0)), (5, frame())]


def press(to=-24.0, frm=UP, closed=True):
    """One push of the handle by the left hand (closed fingers on the grip) from `frm` to `to` degrees, then the hand
    lifts off and the handle springs back up."""
    f = []
    for k in range(7):
        a = frm + (to - frm) * k / 6
        f.append((2, frame(handle=a, left=m.handle_grip(None, math.radians(a)) + [0, 0, 0.019],
                           lopen=0.0 if closed else 1.0)))
    f.append((4, frame(handle=UP, left=m.handle_grip(None, m.H_UP) + [0, 0, 0.05], lopen=0.0)))
    return f


def strokes(n, **kw):
    return [s for _ in range(n) for s in press(**kw)]


def home():
    return [(10, frame())]


def setup_pump_pos():
    """handle_grip() reads the pump's position from the scene; tests pass None, so give it a stand-in."""
    orig = m.handle_grip

    def grip(objects, q=None):
        if objects is None:
            objects = {"pump": st(PUMP)}
        return orig(objects, q)
    m.handle_grip = grip


setup_pump_pos()
GOOD = pour() + cup_back() + strokes(6) + home()

CASES = [
    ("nothing done", [(40, frame())], 0.0),
    ("primed with the cup, six strokes, cup set down, arms home", GOOD, 1.0),
    ("same, but the left arm left hovering over the handle at the end (open fingers)",
     pour() + cup_back() + strokes(6) + [(20, frame(left=m.handle_grip(None, m.H_UP) + [0, 0, 0.02]))], 0.0),
    ("same, but the left hand still pushing the handle down at the end",
     pour() + cup_back() + strokes(6) + [(20, frame(handle=DOWN, left=m.handle_grip(None, m.H_DOWN) + [0, 0, 0.019],
                                                    lopen=0.0))], 0.0),
    ("same, handle left down (stuck) with the hand away", pour() + cup_back() + strokes(6) + [(20, frame(handle=DOWN))],
     0.0),
    ("same, but the cup still held up in the right hand at the end",
     pour() + strokes(6) + [(20, frame(cup=CUP + [0, 0, 0.08], right=CUP + [0, 0, 0.08], ropen=0.0))], 0.0),
    ("same, arm not home (right arm parked over the table)",
     pour() + cup_back() + strokes(6) + [(20, frame(right=(0.25, -0.20, m.TOP + 0.15)))], 0.9),
    ("pumped dry first (three strokes), then primed and pumped six", strokes(3) + GOOD, 0.25),
    # review, Nov 10: a stroke or two while the priming water goes in is common practice (DRY_MAX = 2)
    ("two dry strokes (pumping as the priming water goes in), then primed and pumped six", strokes(2) + GOOD, 1.0),
    ("one dry stroke, then primed and pumped six", strokes(1) + GOOD, 1.0),
    ("pumped dry only (twelve strokes, never primed)", strokes(12) + home(), 0.0),
    ("too little priming water (cup tipped to 64 degrees: about 9 mL in), then six strokes",
     pour(peak=64) + cup_back() + strokes(6) + home(), 0.0),
    ("too little, then topped up with the rest and pumped again: the first strokes were dry",
     pour(peak=64) + cup_back() + strokes(3) + pour() + cup_back() + strokes(6) + home(), 0.25),
    ("primed, but stopped after three strokes (before the water arrives)", pour() + cup_back() + strokes(3) + home(),
     0.0),
    ("primed, four strokes (one delivering stroke, 75 mL)", pour() + cup_back() + strokes(4) + home(), 0.5),
    ("primed, five strokes (two delivering strokes, 150 mL)", pour() + cup_back() + strokes(5) + home(), 1.0),
    ("primed, then short jabs at the handle's middle (10 to -10 degrees), twenty of them",
     pour() + cup_back() + strokes(20, frm=10, to=-10) + home(), 0.0),
    ("primed, strokes that go down but never back up past 12 degrees (handle held low)",
     pour() + cup_back() + [s for _ in range(6) for s in [(3, frame(handle=0.0)), (3, frame(handle=-24.0))]]
     + home(), 0.0),
    ("priming water poured into the pail instead of the pump, then six strokes",
     pour(over=(m.PAIL_XY[0] - 0.01, m.PAIL_XY[1], m.TOP + m.PAIL_H + 0.05)) + cup_back() + strokes(6) + home(), 0.0),
    ("primed and pumped, cup let go lying on its side on the table",
     pour() + strokes(6) + [(20, frame(cup=(m.CUP_XY[0], m.CUP_XY[1], m.TOP + m.CUP_R), cdeg=90.0))], 0.75),
    ("primed and pumped, cup left standing on the pump platform (not on the table)",
     pour() + strokes(6) + [(20, frame(cup=(m.PX - 0.04, -0.03, m.TOP + m.PLAT[2] + m.CUP_H / 2)))], 0.75),
    ("primed and pumped with the cup still in hand, then set down: order of release does not matter",
     pour() + strokes(6) + cup_back() + home(), 1.0),
]

bad = 0
for name, frames, want in CASES:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    s = m.strokes(None)
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name,
          f"[head {pk.volume(None, 'pump'):.1f} mL, pumped {pk.volume(None, 'pail', 'well'):.0f} mL, "
          f"strokes {s['strokes']} dry {s['dry']}, spill {pk.spilled():.1f}]")


def check(ok, text):
    global bad
    bad += not ok
    print(("ok  " if ok else "FAIL"), text)


run(pour())
w = pk.volume(None, "pump")
check(w >= m.PRIME_ML + 10, f"the demo's 88 degree pour puts {w:.1f} mL on the leather (seal needs {m.PRIME_ML})")
run(pour(peak=80))
w = pk.volume(None, "pump")
check(w >= m.PRIME_ML, f"an 80 degree pour still primes it ({w:.1f} mL)")

run(strokes(2) + GOOD)
check(m.strokes(None)["dry"] == 2, "dry strokes stay counted after the pump is primed (irreversible)")

got, _ = run(GOOD)
check(got == 1.0 and m.strokes(None)["dry"] == 0, "a new trial starts with a clean stroke count and a full cup")

run(pour() + cup_back() + strokes(6))
total = pk.volume(None, "pump") + pk.volume(None, "cup") + pk.volume(None, "pail", "water") + pk.spilled()
check(abs(total - m.CUP0) < 1e-6, f"priming water conserved ({total:.6f} mL)")
check(abs(pk.volume(None, "pail", "well") - 3 * m.PER_STROKE) < 1e-9, "six sealed strokes deliver three strokes' water")

run(strokes(1))
mem = kit._law_mem("pump/handle", HANDLE)
q0 = HANDLE.law.drive("pump/handle", m.H_DOWN, 0.0, {}, mem)[0]
check(q0 > m.H_UP, "the handle's law pulls it back up past its top stop (the plunger rod's weight)")

print("all pass" if bad == 0 else f"{bad} FAILED")
raise SystemExit(bad)
