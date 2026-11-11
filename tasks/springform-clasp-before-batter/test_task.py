"""Offline checks of the springform scorer, the pan's leak and the clasp and base laws: step-by-step poses of the jug
and the pan's joints, no simulator.

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
PAN = np.array((m.PX, m.PY, m.TOP))
RIM_Z = m.TOP + m.RING_H
LINKS = {lk.name: lk for lk in m.OBJECTS[0].links}
OPEN, SHUT = m.OPEN, 0.0  # clasp angles, rad
UP, DOWN = m.BASE_UP, 0.0  # base heights above its seat, m


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


def frame(jug=None, jdeg=0.0, clasp=OPEN, base=UP, grip=None, opening=1.0, left=None, locked_flag=None):
    """One step's scene. `clasp` is the lever angle (rad, 0 = flat and locked) and `base` the base's height above its
    seat (m). `locked_flag` overrides the over-center law's own flag (a stale or wrong flag)."""
    grippers = dict(HOME)
    if grip is not None:
        grippers["right_gripper"] = grip
    if left is not None:
        grippers["left_gripper"] = left
    return {"_opening": opening, "_clasp": clasp, "_base": base, "_flag": locked_flag,
            "pan": st(PAN, half=(m.R_OUT, m.R_OUT, 0.0)),
            "jug": st(JUG if jug is None else jug, jdeg, half=(m.JUG_R, m.JUG_R, m.JUG_H / 2)),
            **{g: st(p, half=(0.01, 0.01, 0.01)) for g, p in grippers.items()}}


def set_joints(objs):
    """What yamkit does each step before the scorer: the joints, and each law's memory from its joint."""
    q = {"clasp": objs["_clasp"], "bottom": objs["_base"], "ring": m.SPREAD / m.OPEN * objs["_clasp"]}
    for name, lk in LINKS.items():
        key = f"pan/{name}"
        kit._JOINTS[key] = kit.JointState(pos=q[name], vel=0.0, latched=False)
        lk.law.drive(key, q[name], 0.0, q, kit._law_mem(key, lk))
    if objs["_flag"] is not None:
        kit._law_mem("pan/clasp", LINKS["clasp"])["locked"] = objs["_flag"]


def run(frames, trace=False):
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
    if trace:
        print("   pan", round(pk.volume(None, "pan"), 1), "spill", round(pk.spilled(), 2), "leaked",
              round(pk.leaked(None, "pan"), 2))
    return out


CENTER = (m.PX - 0.012, m.PY - 0.005)


def pour(peak=88.0, clasp=SHUT, base=DOWN, lip_xy=CENTER, lip_dz=0.05, held=True, n=6):
    """Tip the jug about its lip (`lip_dz` over the pan's rim) from 60 degrees to `peak` in `n` stages, hold it there,
    then tip it back upright."""
    lip = (*lip_xy, RIM_Z + lip_dz)
    o = 0.0 if held else 1.0
    f = []
    for k in range(1, n + 1):
        d = 60 + (peak - 60) * k / n
        f.append((10, frame(jug=tipped(lip, d), jdeg=d, clasp=clasp, base=base, grip=lip, opening=o)))
    f.append((40, frame(jug=tipped(lip, peak), jdeg=peak, clasp=clasp, base=base, grip=lip, opening=o)))
    f.append((10, frame(jug=tipped(lip, 40), jdeg=40, clasp=clasp, base=base, grip=lip, opening=o)))
    return f


def jug_back(clasp=SHUT, base=DOWN):
    return [(5, frame(clasp=clasp, base=base, grip=(0.23, -0.13, 0.81), opening=1.0)),
            (5, frame(clasp=clasp, base=base))]


PRESS_AT = (m.PX, m.PY, m.TOP + 0.01)


def seat(clasp=OPEN):
    return [(3, frame(clasp=clasp, base=b, left=PRESS_AT)) for b in (0.006, 0.004, 0.002, 0.0)] + \
           [(3, frame(clasp=clasp, base=DOWN))]


def close(base=DOWN):
    return [(3, frame(clasp=a, base=base, left=m.tab_xyz(a) + [0, 0.02, 0.01])) for a in (-0.9, -0.6, -0.3, -0.1, 0)] \
        + [(5, frame(clasp=SHUT, base=base))]


GOOD = seat() + close() + pour() + jug_back()
FULL_END = frame(clasp=SHUT, base=DOWN)

def slow_tip(clasp, base, d0, d1, dd):
    """Tip the jug from d0 to d1 degrees in dd-degree steps over the pan (a slow trickle)."""
    lip, f, d = (*CENTER, RIM_Z + 0.05), [], d0
    while d <= d1:
        f.append((1, frame(jug=tipped(lip, d), jdeg=d, clasp=clasp, base=base, grip=lip, opening=0.0)))
        d += dd
    return f


CASES = [
    # review, Nov 10: a trickle slow enough to run straight out of the open seam never fills the pan, but still leaks
    ("slow trickle into the open pan (runs straight out), then clasp shut and poured",
     seat() + slow_tip(OPEN, DOWN, 60, 64.5, 0.05) + close() + pour() + jug_back(), 0.0),
    # review: clasp locked with the base still up on its lip, then a hand holds the base down through the pour
    ("clasp locked on a raised base, base held down by a hand through the pour",
     close(base=UP) + [(5, frame(clasp=SHUT, base=DOWN, left=PRESS_AT))]
     + [(n, dict(fr, left_gripper=st(PRESS_AT, half=(.01, .01, .01)))) for n, fr in pour() + jug_back()], 0.0),
    # review 2, Nov 10: locking on a raised base is not irreversible; reopening, seating and relocking before any
    # batter goes in is the correct recovery
    ("clasp locked on the raised base, reopened, base seated, relocked, then poured",
     close(base=UP) + [(5, frame(clasp=OPEN, base=UP))] + seat() + close() + pour() + jug_back(), 1.0),
    ("nothing done", [(40, frame())], 0.0),
    ("seated, clasp snapped shut, batter poured, jug set down, arms home", GOOD, 1.0),
    ("same, but the right arm left hovering away from home", GOOD + [(20, frame(clasp=SHUT, base=DOWN,
                                                                                 grip=(0.25, -0.05, 0.95)))], 0.9),
    ("pan closed up right but nothing poured", seat() + close() + [(40, FULL_END)], 0.0),
    ("batter poured with the clasp open (base seated), clasp closed afterward",
     seat() + pour(clasp=OPEN) + jug_back(clasp=OPEN) + close(), 0.0),
    ("clasp closed with the base still on the lip (clamped high), batter poured",
     close(base=UP) + pour(base=UP) + jug_back(base=UP), 0.0),
    ("base on the lip, clasp shut, poured, then reopened, seated and closed again",
     close(base=UP) + pour(base=UP) + [(5, frame(clasp=OPEN, base=UP))] + seat() + close() + jug_back(), 0.0),
    ("poured right, then the clasp opened at the end (the ring lets go)", GOOD + [(20, frame(clasp=OPEN, base=DOWN))],
     0.0),
    ("nothing sealed, batter poured straight in", pour(clasp=OPEN, base=UP) + jug_back(clasp=OPEN, base=UP), 0.0),
    ("poured right, but the jug still held over the pan at the end",
     seat() + close() + pour() + [(20, frame(jug=JUG + [0, 0, 0.1], clasp=SHUT, grip=JUG + [0, 0, 0.1],
                                            opening=0.0, base=DOWN))], 0.0),
    ("poured right, jug down, but the left hand still on the clasp's tab at the end",
     GOOD + [(20, frame(clasp=SHUT, base=DOWN, left=m.tab_xyz(0.0) + [0, 0.02, 0.0]))], 0.0),
    ("too little poured (tipped to 68 degrees)", seat() + close() + pour(peak=68) + jug_back(), 0.5),
    ("poured from 15 cm over the rim (splashes)", seat() + close() + pour(lip_dz=0.15) + jug_back(), 0.25),
    ("half the stream misses the pan",
     seat() + close() + pour(peak=72, lip_xy=(m.PX - m.R_IN - 0.004, m.PY)) + pour() + jug_back(), 0.25),
    ("poured right, jug let go leaning 20 degrees (propped on the pan)",
     seat() + close() + pour() + [(20, frame(jdeg=20.0, clasp=SHUT, base=DOWN))], 0.75),
    ("poured right, then the jug let go lying on its side", seat() + close() + pour() +
     [(20, frame(jdeg=90.0, clasp=SHUT, base=DOWN))], 0.0),
    ("jug let go mid-pour (falls over)", seat() + close() + pour(held=False) + jug_back(), 0.0),
    ("kit flag says locked but the lever hangs 11 degrees out (not over center): poured",
     seat() + [(5, frame(clasp=-0.2, base=DOWN, locked_flag=True))] + pour(clasp=-0.2) + jug_back(clasp=-0.2), 0.0),
    ("lever pushed past center but held 6 degrees out (not in the lock zone): poured",
     seat() + pour(clasp=-0.1) + jug_back(clasp=-0.1), 0.0),
    ("base pressed only 0.8 mm short of its seat (within 1 mm counts as seated)",
     seat()[:3] + [(3, frame(base=0.0008))] + close(base=0.0008) + pour(base=0.0008) + jug_back(base=0.0008), 1.0),
    ("base 2 mm up (still out of the groove): poured", close(base=0.002) + pour(base=0.002) + jug_back(base=0.002),
     0.0),
    ("a one-step glitch of the clasp reading open with batter in the pan is not a leak",
     seat() + close() + pour() + [(1, frame(clasp=-0.2, base=DOWN))] + jug_back(), 1.0),
]


def check_reset():
    """A finished leak in one trial does not carry into the next (kit.episode() reset)."""
    run(pour(clasp=OPEN, base=UP))
    s, _ = run(GOOD)
    assert s == 1.0, f"history not reset between trials: {s}"
    s, _ = run([(40, frame())])
    assert s == 0.0, f"do-nothing after a good trial scored {s}"
    print("ok   history resets with each trial")


def check_leak_rates():
    """The pan holds batter only sealed: open seam 3 mL/s, base on the lip 0.4 mL/s, sealed none."""
    for clasp, base, rate in ((OPEN, DOWN, m.LEAK_OPEN), (SHUT, UP, m.LEAK_UNSEATED), (SHUT, DOWN, 0.0)):
        kit._EPISODE["count"] += 1
        f = frame(clasp=clasp, base=base)
        set_joints(f)
        objs = {k: v for k, v in f.items() if not k.startswith("_")}
        pk.update(objs)
        pk._S.contents["pan"]["batter"] = 100.0
        for _ in range(200):  # 10 s
            pk.update(objs)
        lost = 100.0 - pk.volume(None, "pan")
        assert abs(lost - rate * 10) < 0.05, (clasp, base, lost)
        assert abs(pk.leaked(None, "pan") - lost) < 1e-9 and abs(pk.spilled() - lost) < 1e-9
    print("ok   leak rates: open seam 30 mL in 10 s, base on the lip 4 mL, sealed none")


# --- The joint laws, integrated like PhysX's implicit drive (as in ~/yam-tools/test_mechkit.py)
DT = 0.05


def integrate(lk, ext, steps, J=None, q0=None, inertia=None, sub=12):
    key = "pan/" + lk.name
    q = lk.start if q0 is None else q0
    qd = 0.0
    J = dict(J or {})
    inertia = inertia or (2e-5 if lk.kind == "revolute" else lk.mass)
    for n in range(steps):
        J[lk.name] = q
        t, s, e, d = lk.law.drive(key, q, qd, J, kit._law_mem(key, lk))
        for _ in range(sub):
            h = DT / sub
            f = s * (t - q) + e + (ext(n) if callable(ext) else ext) - 0.01 * qd
            qd = (qd + h * (f - s * h * qd) / inertia) / (1 + h * (d + s * h) / inertia)
            q = min(max(q + qd * h, lk.lower), lk.upper)
            if q in (lk.lower, lk.upper):
                qd = 0.0
    return q


def check_laws():
    clasp, base = LINKS["clasp"], LINKS["bottom"]
    kit._EPISODE["count"] += 1
    q = integrate(clasp, 0.0, 40)
    assert q < m.CENTER and not mk.locked(None, "pan/clasp"), q
    kit._EPISODE["count"] += 1
    q = integrate(clasp, lambda n: 0.3 if n < 15 else 0.0, 60)  # pushed in past center, then let go
    assert q >= m.LOCK_AT and mk.locked(None, "pan/clasp"), q
    kit._EPISODE["count"] += 1
    q = integrate(clasp, lambda n: 0.1 if n < 30 else 0.0, 60)  # a push weaker than the ring's spring (0.12 N*m)
    assert q < m.CENTER and not mk.locked(None, "pan/clasp"), q
    open_ = {"clasp": OPEN}
    kit._EPISODE["count"] += 1
    q = integrate(base, 0.0, 40, J=open_)
    assert abs(q - m.BASE_UP) < 2e-4, q  # left alone it stays on the lip
    q = integrate(base, -0.5, 40, J=open_)
    assert q > m.SNAP, q  # a light touch (0.5 N) doesn't seat it
    q = integrate(base, lambda n: -3.0 if n < 10 else 0.0, 60, J=open_)
    assert q <= m.SEATED, q  # a firm press (3 N) snaps it into the groove, and it stays there
    kit._EPISODE["count"] += 1
    q = integrate(base, lambda n: -3.0 if n < 10 else 0.0, 60, J={"clasp": 0.0})
    assert abs(q - m.BASE_UP) < 1e-3, q  # the locked ring clamps it on the lip: it can't be pressed down
    print("ok   clasp snaps shut past center and a push weaker than the ring's spring leaves it open; base seats on a firm press, not a touch,"
          " and is clamped by a locked ring")


def check_masses():
    """Links under 50 g make the drives ring or blow up in PhysX (springs and latches stop holding)."""
    for lk in LINKS.values():
        assert lk.mass >= 0.05, (lk.name, lk.mass)
    print("ok   every moving part is at least 50 g")


def main():
    fails = 0
    for label, frames, want in CASES:
        got, _ = run(frames)
        good = abs(got - want) < 1e-9
        fails += not good
        print(("ok  " if good else "FAIL"), f"{got:.2f} (want {want:.2f})", label)
    check_reset()
    check_leak_rates()
    check_laws()
    check_masses()
    if fails:
        raise SystemExit(f"{fails} failed")
    print("all pass")


if __name__ == "__main__":
    main()
