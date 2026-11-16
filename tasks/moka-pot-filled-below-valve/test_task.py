"""Offline checks of the moka pot scorer: hand-made step-by-step states of the pot, the carafe and the grippers, no
simulator. The water in the base is set directly (pourkit's pour model is tested by its pilot, acid-into-water).

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, pk = m.kit, m.pk
HOME = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
BASE = np.array((m.BX, m.BY, m.FLOOR_Z))
RIM = BASE + (0, 0, m.RIM)
SEAT = RIM.copy()  # the basket in: its flange's bottom on the rim
TOUCH = RIM + (0, 0, m.K_FLANGE_T + m.GASKET.protrude)  # the gasket just touching the basket's flange
CREST = TOUCH + (0, 0, m.CREST)  # set on straight, resting on the thread's crests
KSTART = np.array(m.BASKET_POS)
TOP0 = np.array((*m.TOP_XY, m.TOP + m.COLLAR_H + 0.0005))
GRIP_OPEN = 0.58  # fingers closed on the top's body (as read in the sim)


def q_yaw(deg, tilt=0.0):
    """Yaw about z (degrees), then optionally tilted about x."""
    a, t = math.radians(deg) / 2, math.radians(tilt) / 2
    qz = np.array([0.0, 0.0, math.sin(a), math.cos(a)])
    qx = np.array([math.sin(t), 0.0, 0.0, math.cos(t)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


DOWN = np.array((0.70711, 0.70711, 0.0, 0.0))  # a gripper pointing straight down, fingers closing along y at yaw 0


def hand(pos, yaw=0.0):
    """A gripper's state: straight down, turned `yaw` degrees about the vertical."""
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=qmul(q_yaw(yaw), DOWN), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.02, high=p + 0.02)


def st(pos, yaw=0.0, tilt=0.0, half=0.02):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=q_yaw(yaw, tilt), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - half,
                       high=p + half)


def frame(top=None, top_yaw=0.0, top_tilt=0.0, jug=None, left=None, left_yaw=0.0, left_open=1.0, right=None,
          right_yaw=0.0, right_open=1.0, basket=None, basket_tilt=0.0, jug_tilt=0.0, base_yaw=0.0, base_tilt=0.0):
    """One step's states. The basket is in the base unless `basket` says where it is (BASKET is the default)."""
    g = {"left_gripper": (left if left is not None else HOME["left_gripper"], left_yaw, left_open),
         "right_gripper": (right if right is not None else HOME["right_gripper"], right_yaw, right_open)}
    objs = {"base": st(BASE, base_yaw, base_tilt),
            "top": st(TOP0 if top is None else top, top_yaw, top_tilt),
            "jug": st(m.JUG_POS if jug is None else jug, 0.0, jug_tilt),
            "basket": st(BASKET if basket is None else basket, 0.0, basket_tilt),
            "cup": st((*m.CUP_XY, m.TOP))}
    objs.update({k: hand(p, y) for k, (p, y, _) in g.items()})
    return objs, {k: o for k, (_, _, o) in g.items()}


BASKET = SEAT


def run(frames, water=0.0, spill=0.0):
    """frames: list of (steps, (objects, openings)). A fresh trial each time (the kit's reset counter moves on); the
    base is given `water` mL and the table `spill` mL after the first step."""
    kit._EPISODE["count"] += 1
    out = (0.0, False)
    first = True
    for n, (objs, opens) in [(2, frame(basket=KSTART))] + frames:
        kit._OPENING.update(opens)
        for _ in range(n):
            pk.update(objs)
            if first:
                pk._S.contents["base"] = {"water": water} if water else {}
                pk._S.contents["jug"] = {"water": max(m.JUG0 - water - spill, 0.0)}
                pk._S.spilled = {"water": spill} if spill else {}
                first = False
            ok = m.success(objs)
            out = (m.score(objs), ok)
    return out


GRIP_Z = RIM[2] + m.GRIP_DZ
SNUG_C = 0.0002  # a gasket compression past the snug line


def seated(c):
    return TOUCH - (0, 0, c)


def place(holder="left"):
    """Carry the top over and set it on the thread's crests, held."""
    g = (RIM[0], RIM[1], GRIP_Z + m.CREST)
    return [(3, frame(top=CREST + (0, 0, 0.03), **{holder: g + np.array((0, 0, 0.03)), f"{holder}_open": GRIP_OPEN})),
            (3, frame(top=CREST, **{holder: g, f"{holder}_open": GRIP_OPEN}))]


def screw(c=SNUG_C, tilt=0.0, off=(0.0, 0.0), steps=10, holder="left"):
    """The top turned down the thread from the crests to the gasket pressed `c` (m), held and steadied; returns the
    frames and the final pose."""
    f = []
    end = seated(c) + (*off, 0.0)
    for k in range(1, steps + 1):
        p = CREST + (end - CREST) * k / steps
        f.append((1, frame(top=p, top_yaw=-60.0 * k, top_tilt=tilt, **{holder: (RIM[0], RIM[1], p[2] + m.GRIP_DZ),
                                                                       f"{holder}_open": GRIP_OPEN},
                           right=(m.STEADY_X, m.STEADY_Y, m.FLOOR_Z + m.BASE_GZ), right_open=0.85)))
    return f, end


def let_go(end, tilt=0.0, home=True):
    return [(3, frame(top=end, top_yaw=-600.0, top_tilt=tilt, left=(RIM[0], RIM[1], GRIP_Z + 0.08)))] + \
           ([(5, frame(top=end, top_yaw=-600.0, top_tilt=tilt))] if home else [])


GOOD = 140.0


def good(c=SNUG_C, water=GOOD, spill=0.0, home=True, tilt=0.0, off=(0.0, 0.0)):
    f, end = screw(c, tilt, off)
    return run(place() + f + let_go(end, tilt, home), water=water, spill=spill)


def check(name, got, want):
    assert abs(got[0] - want) < 1e-9, f"{name}: score {got[0]}, wanted {want}"
    print(f"ok  {name}: {got[0]}")


def pour_frames(steps=60, tilt=100.0):
    """The carafe tipped `tilt` degrees about x, its lip 5 cm over the middle of the base's mouth, for `steps` steps."""
    probe = frame(jug=np.array(m.JUG_POS), jug_tilt=tilt)[0]
    _, _, _, _, lip = pk._pose(probe, pk._CFG["vessels"]["jug"])
    at = np.array(m.JUG_POS) + (RIM + (0, 0, 0.05)) - lip
    return [(steps, frame(jug=at, jug_tilt=tilt, basket=BASKET))]


def basket_tests():
    global BASKET
    f, end = screw()
    # the basket never put in (left in its cup) under a screwed top: no screwing credit, so filled only
    BASKET = KSTART
    check("basket left out, top screwed on", run(place() + f + let_go(end), water=GOOD), 0.25)
    # ... or propped crooked in the mouth (10 degrees, 5 mm up) under the top
    crooked = st(SEAT + (0.002, 0, 0.005), 0.0, 10.0)
    tilted = [(n, ({**o, "basket": crooked}, op)) for n, (o, op) in place() + f + let_go(end)]
    check("basket propped crooked in the mouth", run(tilted, water=GOOD), 0.25)
    # flat on the rim against the mouth's wall (the modeled cup can sit 2.9 mm off center): still in
    BASKET = SEAT + (0.0, 0.0029, 0.0)
    check("basket flat on the rim, 2.9 mm off center", run(place() + f + let_go(end), water=GOOD), 1.0)
    BASKET = SEAT
    # tipped 60 degrees for three steps on the way over (the grounds fall out), then put in: filled only
    tip = [(3, frame(top=TOP0, basket=SEAT + (0, -0.1, 0.06), basket_tilt=60.0))]
    check("basket tipped 60 degrees on the way, then put in", run(tip + place() + f + let_go(end), water=GOOD), 0.25)
    # a one-step physics pop past 45 degrees is not a spill
    pop = [(1, frame(top=TOP0, basket=SEAT + (0, -0.1, 0.06), basket_tilt=60.0)),
           (2, frame(top=TOP0, basket=SEAT + (0, -0.1, 0.06)))]
    check("one-step pop past 45 degrees", run(pop + place() + f + let_go(end), water=GOOD), 1.0)
    # the coffee's patch is not drawn offline, and the latch keeps where it fell
    assert m._H.coffee_lost is None
    run(tip, water=GOOD)
    assert m._H.coffee_lost is not None and abs(m._H.coffee_lost[1] - (SEAT[1] - 0.1)) < 1e-9
    # basket in, top on but not turned: 0.5
    check("basket in, top set on the crests, not turned", run(place() + let_go(CREST), water=GOOD), 0.5)
    # top screwed with no basket, then the basket dropped on top of it (lying on the lid)
    BASKET = KSTART
    f2 = place() + f + let_go(end)
    f2 += [(5, frame(top=end, basket=end + (0, 0, m.T_H + 0.012), basket_tilt=0.0))]
    check("top screwed, basket dropped on its lid", run(f2, water=GOOD), 0.25)
    # top screwed with no basket under it, then the basket appears in the base (put in after the top: no credit)
    f3 = place() + f + let_go(end) + [(5, frame(top=end, basket=SEAT))]
    check("basket in only after the top was screwed", run(f3, water=GOOD), 0.5)
    BASKET = SEAT
    # weak fill with everything else done, and a weak fill with only the basket in
    check("short pour, basket in, top snug", good(water=m.LOW_ML - 5), 0.25)
    check("short pour, basket in, top left off", run([(5, frame())], water=m.LOW_ML - 5), 0.0)
    # the order of water and basket: poured with the basket already in, the stream lands on the grounds (spilled)
    BASKET = KSTART
    run(pour_frames())
    dry_first = pk.volume(None, "base")
    assert dry_first > 100.0, f"pouring into the open base should fill it, got {dry_first:.1f} mL"
    BASKET = SEAT
    run(pour_frames())
    assert pk.volume(None, "base") == 0.0 and pk.spilled() > 100.0, "a stream onto the seated basket must spill"
    BASKET = SEAT
    seq_bad = pour_frames(20, 80.0) + place() + f + let_go(end)
    check("basket in first, then poured (onto the grounds)", run(seq_bad), 0.0)
    BASKET = KSTART
    seq = pour_frames(20, 80.0) + [(2, frame(basket=KSTART))]  # 129.9 mL, then the carafe stood back up
    BASKET = SEAT
    check("poured 130 mL into the open base, then the basket and the top", run(seq + place() + f + let_go(end)), 1.0)
    assert m._lets_water_in(frame(basket=KSTART, top=TOP0)[0])
    assert not m._lets_water_in(frame(basket=SEAT, top=TOP0)[0])


def main():
    check("full job", good(), 1.0)
    assert good()[1], "success() should be true"
    check("do nothing", run([]), 0.0)
    check("top screwed on an empty base", good(water=0.0), 0.0)
    check("water above the valve", good(water=m.VALVE_ML + 2), 0.0)
    check("carafe emptied in (175 mL)", good(water=m.JUG0), 0.0)
    check("weak: 1.5 cm under the valve", good(water=m.ml_below(m.TOP_LINE_Z - 0.015)), 0.25)
    check("just under the valve", good(water=m.VALVE_ML - 1), 1.0)
    check("just above the low line", good(water=m.LOW_ML + 1), 1.0)
    check("filled, basket and top left out", run([], water=GOOD), 0.25)
    check("filled, basket in, top left off", run([(5, frame())], water=GOOD), 0.5)
    check("a dribble poured in, basket in, top left off", run([(5, frame())], water=60.0), 0.0)
    # snug is the gasket pressed: down on it but not pressed far enough (under the torque), or only touching
    check("gasket pressed 0.12 mm (past the line)", good(c=0.00012), 1.0)
    check("gasket pressed 0.08 mm (short of snug)", good(c=0.00008), 0.5)
    check("gasket only touching", good(c=0.0), 0.5)
    check("left on the crests, not screwed", good(c=-m.CREST), 0.5)
    # cross-threaded: crooked on the wrong groove (2.6 degrees is one groove over the thread), even if pressed down
    check("cross-threaded 2.6 degrees, pressed", good(tilt=2.6), 0.5)
    check("cross-threaded 5 degrees", good(tilt=5.0, c=-0.002), 0.5)
    check("1 degree of wobble in the right groove", good(tilt=1.0), 1.0)
    check("3 mm off the base's axis", good(off=(0.003, 0.0)), 0.5)
    check("spilled 10 mL", good(spill=10.0), 0.75)
    check("arms not home (not scored)", good(home=False), 1.0)
    assert not hasattr(m, "REQUIRE_ARMS_HOME") and not hasattr(m, "_arms_home"), "arm-home rule should be gone"
    assert not hasattr(m, "_hold_at_stop") and not hasattr(m, "STOP"), "no scripted thread stop: the thread is physical"
    # the steadying hand: still on the base at the end, or the base knocked over
    f, end = screw()
    held = let_go(end)[:1] + [(5, frame(top=end, right=(m.STEADY_X, m.STEADY_Y, m.FLOOR_Z + m.BASE_GZ), right_open=0.85))]
    check("right hand still holding the base at the end", run(place() + f + held, water=GOOD), 0.75)
    side = (m.BX, m.BY - m.B_FLATS / 2 - 0.024 - 0.010, m.FLOOR_Z + m.RIM + 0.03)
    check("closed hand parked 1 cm beside the base, over its shoulder", run(place() + f + let_go(end)[:1] + [(5, frame(
        top=end, right=side, right_open=0.0))], water=GOOD), 1.0)
    tipped = let_go(end)[:1] + [(5, frame(top=end, base_tilt=15.0))]
    check("base knocked over 15 degrees at the end (top and basket left upright)", run(place() + f + tipped, water=GOOD), 0.25)
    # still holding the top at the end, or the carafe
    held = [(5, frame(top=end, left=(RIM[0], RIM[1], end[2] + m.GRIP_DZ), left_open=GRIP_OPEN))]
    check("top still gripped at the end", run(place() + f + held, water=GOOD), 0.75)
    end_j = let_go(end)[:1] + [(5, frame(top=end, jug=(0.23, -0.20, m.TOP + 0.10), right=(0.23, -0.20, m.TOP + 0.10),
                                          right_open=0.3))]
    check("carafe still held at the end", run(place() + f + end_j, water=GOOD), 0.75)
    # the top knocked off after being screwed, or lifted 8 mm
    check("screwed, then knocked off", run(place() + f + let_go(end)[:1] + [(5, frame(
        top=end + (0.09, 0.0, -0.05), top_tilt=80.0))], water=GOOD), 0.5)
    check("screwed, then left raised 8 mm", run(place() + f + let_go(end)[:1] + [(5, frame(
        top=end + (0, 0, 0.008)))], water=GOOD), 0.5)
    basket_tests()
    assert good()[0] == 1.0
    check("reset after a full job, then nothing", run([]), 0.0)
    print("all pass")


if __name__ == "__main__":
    main()
