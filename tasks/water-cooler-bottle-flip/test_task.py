"""Offline checks of the water-cooler scorer and the bottle's water model: step-by-step poses, no simulator.

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
START = np.array(m.JUG_POS)
HELD = 0.27  # gripper opening closed on a 2.5 cm handle (pads 9.4 cm apart fully open)
OVER = np.array([m.COOLER_XY[0], m.COOLER_XY[1], m.UP])
SEAT = np.array([m.COOLER_XY[0], m.COOLER_XY[1], m.CROWN_Z + m.SH2_TOP])  # handles' axis of a seated bottle


def quat(deg_y, deg_x=0.0):
    """About world y by deg_y (the neck toward +x), then about world x by deg_x (a sideways swing)."""
    ay, ax = math.radians(deg_y) / 2, math.radians(deg_x) / 2
    qy = np.array([0.0, math.sin(ay), 0.0, math.cos(ay)])
    qx = np.array([math.sin(ax), 0.0, 0.0, math.cos(ax)])
    x1, y1, z1, w1 = qx
    x2, y2, z2, w2 = qy
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def st(pos, q=None, half=(0.05, 0.05, 0.07), speed=0.0):
    p = np.array(pos, float)
    h = np.array(half)
    return ObjectState(pos=p, quat_xyzw=quat(0) if q is None else q, lin_vel=np.array([speed, 0, 0]),
                       ang_vel=np.zeros(3), low=p - h, high=p + h)


def frame(jug=None, deg=0.0, swing=0.0, hands=2, speed=0.0, home=False):
    """The bottle's handles' axis at `jug`, turned `deg` about y (and `swing` about x); `hands` grippers closed on
    its handles (the rest at home, open), or all at home and open with home=True."""
    jug = START if jug is None else np.asarray(jug, float)
    q = quat(deg, swing)
    lugs = [jug + pk._rotm(q) @ np.asarray(c) for c in m._lugs_local()]  # [-y, +y]
    g, opening = dict(HOME), {"left_gripper": 1.0, "right_gripper": 1.0}
    if not home:
        for name, lug in (("left_gripper", lugs[1]), ("right_gripper", lugs[0]))[:hands]:
            g[name], opening[name] = tuple(lug), HELD
    return {"_opening": opening, "cooler": st((*m.COOLER_XY, m.TOP), half=(0.06, 0.06, 0.0)),
            "jug": st(jug, q, speed=speed), **{n: st(p, half=(0.01, 0.01, 0.01)) for n, p in g.items()}}


def run(frames):
    """frames: list of (steps, objects). A fresh trial each time (the kit's reset counter moves on)."""
    kit._EPISODE["count"] += 1
    out = (0.0, False)
    for n, objs in [(2, frame(home=True))] + frames:  # each trial starts with the arms home
        kit._OPENING.update(objs["_opening"])
        objs = {k: v for k, v in objs.items() if k != "_opening"}
        for _ in range(n):
            objs = dict(objs)  # a new dict each step: offline, tracekit tells steps apart by the objects dict
            pk.update(objs)  # in the sim the scene hook does this before success() and score()
            ok = m.success(objs)
            out = (m.score(objs), ok)
    return out


def lerp(a, b, s):
    return np.asarray(a, float) + s * (np.asarray(b, float) - np.asarray(a, float))


def carry(a, b, steps, deg=0.0, hands=2, swing=0.0):
    return [(1, frame(lerp(a, b, (k + 1) / steps), deg, swing, hands)) for k in range(steps)]


def turn(at, steps, d0=0.0, d1=180.0, hands=2, swing=0.0):
    """Turn about the handles' axis at `at` from d0 to d1 degrees in `steps` control steps (smoothstep)."""
    out = []
    for k in range(steps):
        s = (k + 1) / steps
        out.append((1, frame(at, d0 + (d1 - d0) * s * s * (3 - 2 * s), swing, hands)))
    return out


LIFT = [(10, frame())] + carry(START, START + [0, 0, m.UP - START[2]], 40) + carry(
    START + [0, 0, m.UP - START[2]], OVER, 40)


def seat_from(over, seat, steps=16, hands=2):
    return carry(over, seat, steps, 180.0, hands)


SEATED = seat_from(OVER, SEAT) + [(16, frame(SEAT, 180.0))]
RELEASE = [(10, frame(SEAT, 180.0, hands=0)), (20, frame(SEAT, 180.0, home=True))]
FLIP = LIFT + turn(OVER, 20) + SEATED + RELEASE  # the reference: turned in 1 s right over the crown
OFF = np.array([0.015, 0.0, 0.0])


def tip_about_neck(P, d0, d1, steps):
    """Turn about the neck's tip held at P (not about the handles)."""
    out = []
    for k in range(steps):
        d = d0 + (d1 - d0) * (k + 1) / steps
        out.append((1, frame(np.asarray(P) - pk._rotm(quat(d)) @ np.array([0, 0, m.NECK_TOP]), d)))
    return out


def hover_fist(frames):
    """The right gripper fully closed (empty fist) 2.5 cm off the bottle's -y handle, the left really holding."""
    out = []
    for n, f in frames:
        f = dict(f)
        j = f["jug"]
        lug = np.asarray(j.pos) + pk._rotm(j.quat_xyzw) @ np.array([0, -m.LUG_Y, 0])
        f["right_gripper"] = st(lug + [0, -0.025, 0], half=(0.01,) * 3)
        f["_opening"] = {"left_gripper": HELD, "right_gripper": 0.0}
        out.append((n, f))
    return out


def shoulder_hold(n):
    f = frame(SEAT, 180.0, home=True)
    f["left_gripper"] = st(np.asarray(SEAT) + [0, 0, -0.045], half=(0.01,) * 3)
    f["_opening"] = {"left_gripper": 0.4, "right_gripper": 1.0}
    return (n, f)


def swap_hands(at, steps):
    """A two-hand turn where past 90 degrees only the left hand holds for 2 steps out of every 4."""
    out = []
    for k in range(steps):
        s = (k + 1) / steps
        f = frame(at, 180 * s)
        if 180 * s > 90 and (k // 2) % 2 == 0:
            f["_opening"] = {"left_gripper": HELD, "right_gripper": 1.0}
        out.append((1, f))
    return out


NECK_OVER = np.array([m.COOLER_XY[0], 0.0, m.CROWN_Z + 0.05])  # the neck's tip 5 cm above the opening
NECK_START = NECK_OVER - np.array([0, 0, m.NECK_TOP])
TO_SEAT = carry(NECK_OVER + [0, 0, m.NECK_TOP], SEAT, 16, 180.0) + [(16, frame(SEAT, 180.0))] + RELEASE

CASES = [
    ("nothing done", [(60, frame(home=True))], 0.0),
    ("lifted, turned neck-down in 1 s over the crown, seated, let go, arms home", FLIP, 1.0),
    ("same, but an arm left beside the bottle", LIFT + turn(OVER, 20) + SEATED
     + [(10, frame(SEAT, 180.0, hands=0)), (20, {**frame(SEAT, 180.0, home=True),
                                                  "left_gripper": st((0.21, 0.12, 0.95), half=(0.01,) * 3)})], 0.9),
    ("seated but still held at the end", LIFT + turn(OVER, 20) + SEATED, 0.5),
    ("turned over in 1.6 s, still in one motion: within the limit", LIFT + turn(OVER, 32) + SEATED + RELEASE, 1.0),
    ("turned over slowly (3 s)", LIFT + turn(OVER, 60) + SEATED + RELEASE, 0.25),
    ("paused 1 s on its side mid-turn", LIFT + turn(OVER, 10, 0, 90) + [(20, frame(OVER, 90.0))]
     + turn(OVER, 10, 90, 180) + SEATED + RELEASE, 0.25),
    ("paused 1 s at 135 degrees, neck out over the crown's edge", LIFT + turn(OVER, 15, 0, 135)
     + [(20, frame(OVER, 135.0))] + turn(OVER, 5, 135, 180) + SEATED + RELEASE, 0.25),
    ("turned over above the table, then carried neck-down to the cooler", [(10, frame())]
     + carry(START, START + [0, 0, m.UP - START[2]], 40) + turn(START + [0, 0, m.UP - START[2]], 20)
     + carry(START + [0, 0, m.UP - START[2]], OVER, 40, 180.0) + SEATED + RELEASE, 0.25),
    ("seated 1.5 cm off center: the neck lands on the crown and the bottle leans on it",
     LIFT + turn(OVER, 20) + seat_from(OVER, SEAT + OFF + [0, 0, 0.02]) + [(10, frame(SEAT + OFF + [0, 0, 0.02], 180))]
     + [(10, frame(SEAT + OFF + [0.01, 0, 0.0], 160.0, hands=0)), (20, frame(SEAT + OFF + [0.01, 0, 0.0], 160.0,
                                                                              home=True))], 0.0),
    ("neck in the receptacle but leaning 8 degrees", LIFT + turn(OVER, 20) + seat_from(OVER, SEAT)
     + [(10, frame(SEAT, 172.0)), (10, frame(SEAT, 172.0, hands=0)), (20, frame(SEAT, 172.0, home=True))], 0.0),
    ("neck only 1 cm down: propped on the crown, not sitting on its shoulder", LIFT + turn(OVER, 20)
     + seat_from(OVER, SEAT + [0, 0, 0.01]) + [(10, frame(SEAT + [0, 0, 0.01], 180.0, hands=0)),
                                                (20, frame(SEAT + [0, 0, 0.01], 180.0, home=True))], 0.0),
    ("one arm: the bottle swings 70 degrees sideways on the single handle as it lifts, then is set down",
     [(10, frame(hands=1))] + carry(START, START + [0, 0, 0.06], 20, hands=1, swing=70.0)
     + [(20, frame(START + [0, 0, 0.06], swing=70.0, hands=1)), (20, frame(START, home=True))], 0.0),
    ("one arm: lifted level by one handle, turned and seated (if the physics let it)",
     LIFT[:1] + carry(START, OVER, 60, hands=1) + turn(OVER, 20, hands=1) + seat_from(OVER, SEAT, hands=1)
     + RELEASE, 0.0),
    ("one arm lets go just before the turn, the other turns it alone", LIFT + turn(OVER, 20, hands=1) + SEATED
     + RELEASE, 0.0),
    ("dropped on its side, then picked up and seated", [(10, frame()), (10, frame(START + [0, 0, -0.01], 90.0, hands=0))]
     + LIFT + turn(OVER, 20) + SEATED + RELEASE, 0.0),
    ("let go on the crown before the neck was in: it topples", LIFT + turn(OVER, 20)
     + [(10, frame(OVER - [0, 0, 0.02], 180.0, hands=0)), (10, frame(SEAT + [0.06, 0, -0.03], 100.0, hands=0)),
        (20, frame(SEAT + [0.06, 0, -0.03], 100.0, home=True))], 0.0),
    ("knocked the bottle over and left it", [(10, frame()), (40, frame(START + [0.05, 0, -0.01], 90.0, home=True))],
     0.0),
    ("spilled past the limit, then a perfect second flip from the same trial", LIFT + turn(OVER, 10, 0, 120)
     + [(20, frame(OVER, 120.0))] + turn(OVER, 10, 120, 0) + turn(OVER, 20) + SEATED + RELEASE, 0.25),
    ("QC: one arm turns and seats it, the other an empty closed fist 2.5 cm off the far handle",
     hover_fist(LIFT + turn(OVER, 20, hands=1) + seat_from(OVER, SEAT, hands=1)) + RELEASE, 0.0),
    ("QC: seated, then still held at the end by a gripper closed on the shoulder (not a handle)",
     LIFT + turn(OVER, 20) + SEATED + [(10, frame(SEAT, 180.0, hands=0)), shoulder_hold(40)], 0.5),
    ("QC: slow 4 s turn about the neck's tip 5 cm over the opening (the reservoir overflows past its air lock)",
     [(10, frame())] + carry(START, NECK_START, 60) + tip_about_neck(NECK_OVER, 0, 180, 80) + TO_SEAT, 0.25),
    ("QC: 3 s on its side with the glug falling into the opening (reservoir overflows)",
     [(10, frame())] + carry(START, NECK_START, 60) + tip_about_neck(NECK_OVER, 0, 90, 10)
     + [(60, tip_about_neck(NECK_OVER, 90, 90, 1)[0][1])] + tip_about_neck(NECK_OVER, 90, 180, 10) + TO_SEAT, 0.25),
    ("QC: 3 s turn about the neck's tip right over the opening: most of the glug is caught (accepted)",
     [(10, frame())] + carry(START, NECK_START, 60) + tip_about_neck(NECK_OVER, 0, 180, 60) + TO_SEAT, 1.0),
    ("QC: seated, then knocked off the crown", FLIP + [(30, frame(SEAT + [0.06, 0, -0.03], 100.0, home=True))], 0.0),
    ("QC: hands swapped every 2 steps past 90 degrees (one hand 3+ steps in all)", LIFT + swap_hands(OVER, 20)
     + SEATED + RELEASE, 0.0),
]

bad = 0
for name, frames, want in CASES:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name,
          f"[spilled {pk.spilled():.1f} mL, reservoir {pk.volume(None, 'cooler'):.1f} mL, bottle "
          f"{pk.volume(None, 'jug'):.1f} mL]")

# the reference flip spills well under the limit and the seated bottle tops the reservoir up to its neck, then stops
run(FLIP + [(100, frame(SEAT, 180.0, home=True))])
ok = pk.spilled() < 0.75 * m.SPILL_MAX and abs(pk.volume(None, "cooler") - m.SEAL_ML) < 1e-6
bad += not ok
print(("ok  " if ok else "FAIL"), f"reference flip: {pk.spilled():.1f} mL spilled (limit {m.SPILL_MAX}); reservoir "
      f"stops at {pk.volume(None, 'cooler'):.1f} mL (air lock {m.SEAL_ML:.1f})")

# water is conserved
total = sum(pk.volume(None, n) for n in ("jug", "cooler")) + pk.spilled()
ok = abs(total - (m.WATER0 + m.RES0)) < 1e-6
bad += not ok
print(("ok  " if ok else "FAIL"), f"water conserved ({total:.6f} mL)")

# a spill in one trial does not carry into the next
run(LIFT + turn(OVER, 60) + SEATED + RELEASE)
got, _ = run(FLIP)
ok = got == 1.0
bad += not ok
print(("ok  " if ok else "FAIL"), "a new trial starts with a full bottle and a clean history")

# an empty closed fist near the bottle is not a hold
f = frame(SEAT, 180.0, home=True)
f["left_gripper"] = st(np.asarray(f["jug"].pos) + [0, 0.0, 0.08], half=(0.01,) * 3)
f["_opening"] = {"left_gripper": 0.0, "right_gripper": 1.0}
kit._OPENING.update(f["_opening"])
ok = m._hands_on({k: v for k, v in f.items() if k != "_opening"}) == 0
bad += not ok
print(("ok  " if ok else "FAIL"), "a closed gripper 8 cm from the handles holds nothing")

# an empty closed fist right on a handle is not a hold either; fingers stopped by it (27 % open) are
f = frame(SEAT, 180.0)
for op, want in ((0.0, 0), (HELD, 2), (0.9, 0)):
    f["_opening"] = {"left_gripper": op, "right_gripper": op}
    kit._OPENING.update(f["_opening"])
    got = m._hands_on({k: v for k, v in f.items() if k != "_opening"})
    ok = got == want
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"fingers {op:.2f} open on both handles: {got} hands (want {want})")

print("all pass" if bad == 0 else f"{bad} FAILED")
raise SystemExit(bad)
