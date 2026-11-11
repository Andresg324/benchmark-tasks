"""Offline checks of beltkit's tracking law and of the conveyor task's scorer: step-by-step histories of the two
take-up knobs and the grippers, with near misses. No simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.constants import FINGER_OPEN
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, tk, bk = m.kit, m.tk, m.bk
TOP = m.TOP
tk.register(*[o for o in m.OBJECTS if isinstance(o, tk.Threaded)])
PARK = {"left_gripper": (0.20, 0.27, TOP + 0.22), "right_gripper": (0.18, -0.30, TOP + 0.22)}
KNOB_POS = {s: (m.SCREW_X[s], m.KNOB_IN, TOP + m.Z_BELT - m.PULLEY_R) for s in m.SCREW_X}
HELD_OPEN = 2 * (m.KNOB_HUB + m.KNOB_LOBE) / (2 * abs(FINGER_OPEN))  # the opening with the pads on the lobes
DT = bk.DT
bad = 0


def st(pos):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


class Trial:
    """A fresh trial. Knob turns are in turns tightened from the start (+ = clockwise from the knob's end)."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.turn = {"near": 0.0, "far": 0.0}
        self.t = 0.0
        self.out = None
        self.step(PARK)

    def step(self, grippers=None, held=()):
        for s, k in m.KNOBS.items():
            tk.set_angle(k, m.START_DEG + 360.0 * self.turn[s], held_now=s in held)
        g = dict(PARK, **(grippers or {}))
        objs = {"conveyor": st((0, 0, TOP))}
        for s, k in m.KNOBS.items():
            objs[f"takeup_{s}"] = st((m.SCREW_X[s], 0, TOP))
            objs[k] = st(KNOB_POS[s])
        for k, v in g.items():
            pos, op = (v[0], v[1]) if isinstance(v[0], tuple) else (v, 1.0)
            objs[k] = st(pos)
            kit._OPENING[k] = op
        bk.update(objs)
        ok = m.success(objs)
        self.t += DT
        self.out = (m.score(objs), ok)
        return self.out

    def wait(self, seconds, grippers=None, held=()):
        for _ in range(int(round(seconds / DT))):
            self.step(grippers, held)
        return self.out

    def turn_knob(self, side, turns, seconds, held_by_hand=True):
        """Turn a knob `turns` (tightening +) evenly over `seconds`, a gripper on it, then let go."""
        n = max(1, int(round(seconds / DT)))
        hand = {"right_gripper": (KNOB_POS[side][0], m.KNOB_MID, KNOB_POS[side][2])}, HELD_OPEN
        for _ in range(n):
            self.turn[side] += turns / n
            g = {"right_gripper": (hand[0]["right_gripper"], HELD_OPEN)} if held_by_hand else None
            self.step(g, (side,) if held_by_hand else ())
        return self.out


def quarters(tr, side, total, wait=8.0):
    """Adjust `side` by `total` turns in quarter-turn strokes of 2 s, each followed by `wait` s of watching."""
    n = int(round(abs(total) / 0.25))
    for k in range(n):
        tr.turn_knob(side, math.copysign(0.25, total), 2.0)
        tr.wait(wait)
    rest = total - math.copysign(0.25 * n, total)
    if abs(rest) > 1e-9:
        tr.turn_knob(side, rest, 2.0)
        tr.wait(wait)
    return tr.out


def check(name, got, want):
    global bad
    ok = got == want if not isinstance(want, list) else want[0] <= got <= want[1]
    print(("ok  " if ok else "FAIL"), name, got, want)
    bad += not ok


# ---- 1. the kit's law, alone --------------------------------------------------------------------------------
b = m.BELT
q_mm = 0.25 * b.pitch / b.span * b.center_len * 1000
check("a quarter turn moves the settle point 6.7 mm", round(q_mm, 1), 6.7)
check("start settle point 13.3 mm toward the far frame", round(b.settle_at(0.0) * 1000, 1), 13.3)

tr = Trial()
e = [bk.offset()]
tr.wait(2.0)
check("left alone the belt drifts toward +x (the far frame)", bk.offset() > e[0], True)
tr.wait(30.0)
check("left alone it rubs the far frame", bk.rubbed() and bk.rub_side() == 1, True)
check("  between 15 and 20 s in", 15.0 < bk._state().rub_step * DT < 20.0, True)

tr = Trial()
tr.wait(10.5)
idle = bk.rate()  # drift rate 10.5 s in, left alone
tr = Trial()
tr.wait(1.0)
tr.turn_knob("far", 0.5, 1.0)
tr.wait(0.5)
check("lag: just after tightening the far side the belt still drifts out", bk.rate() > 0, True)
tr.wait(8.0)
check("about 2 revolutions later it runs back toward the near side", bk.rate() < 0, True)

tr = Trial()
tr.wait(1.0)
tr.turn_knob("near", -0.25, 1.0)
tr.wait(8.5)
check("loosening the near side a quarter turn slows the drift out", bk.rate() < 0.5 * idle, True)

tr = Trial()
tr.turn_knob("near", 0.25, 1.0)
tr.wait(8.0)
check("tightening the near side (wrong side) speeds the drift out", bk.rate() > 0 and bk.settle() > 0.015, True)

tr = Trial()
tr.turn_knob("far", 0.5, 2.0)
tr.wait(90.0)
check("half a turn on the far side settles within 0.5 mm of center", abs(bk.offset()) < 0.0005, True)
check("  and steady (under 0.01 mm/s)", bk.max_rate(10.0) < 1e-5, True)
check("  without a rub", bk.rubbed(), False)

tr = Trial()
tr.turn_knob("far", 0.5, 2.0)
tr.turn_knob("near", 1.0, 3.0)
tr.turn_knob("far", 1.0, 3.0)
check("stretch after 1.5 + 1 turns tightening: over the 0.8 percent window", bk.stretch(None) > m.STRETCH_HI, True)
check("stretch at the start is 0.5 percent", round(Trial() and 0.5, 2), 0.5)

tr = Trial()
tr.turn_knob("far", 2.5, 4.0)
check("the slot stops the take-up at 3 mm (2 turns)", round(b.advance(m.START_DEG + 900.0) * 1000, 2), 3.0)

# interpolation: a quarter turn spun within one control step still steers like a slow one
tr = Trial()
tr.turn_knob("far", 0.25, DT)
tr.wait(20.0)
fast = bk.offset()
tr = Trial()
tr.turn_knob("far", 0.25, 1.0)
tr.wait(20.0 - 1.0 + DT)
check("a fast quarter turn ends where a slow one does (within 0.3 mm)", abs(fast - bk.offset()) < 0.0003, True)

# ---- 2. the scorer --------------------------------------------------------------------------------------------
HOME = PARK


def demo_like(extra=None):
    tr = Trial()
    tr.wait(4.0)
    tr.turn_knob("far", 0.25, 2.0)
    tr.wait(12.0)
    tr.turn_knob("far", 0.25, 2.0)
    if extra:
        extra(tr)
    return tr


tr = demo_like()
check("demo-like: still drifting right after the second turn = 0", tr.wait(3.0)[0], 0.0)
check("demo-like: settled, home = 1.0", tr.wait(60.0), (1.0, True))
check("  peak offset stayed under the 10 mm rub", bk.rubbed(), False)

tr = Trial()
check("do nothing = 0 (rubs)", tr.wait(150.0)[0], 0.0)

tr = Trial()
tr.wait(4.0)
tr.turn_knob("near", 0.25, 2.0)
check("wrong side: tighten the near take-up = 0", tr.wait(120.0)[0], 0.0)

tr = Trial()
tr.wait(4.0)
tr.turn_knob("far", 1.5, 6.0)
check("one big crank (1.5 turns on the far side) = 0 (overshoots into the near frame)", tr.wait(120.0)[0], 0.0)
check("  rubbed the near frame", bk.rub_side(), -1)

tr = Trial()
tr.wait(4.0)
for i in range(4):  # impatient: a quarter turn every 4 s without waiting it out
    tr.turn_knob("far", 0.25, 2.0)
    tr.wait(2.0)
check("impatient (four quarter turns, 4 s apart) = 0", tr.wait(120.0)[0], 0.0)

tr = Trial()
tr.wait(14.0)
check("late start: no rub yet at 14 s", bk.rubbed(), False)
tr.wait(8.0)
tr.turn_knob("far", 0.5, 2.0)
check("rubbed the far frame, then tracked to center = 0 (irreversible)", tr.wait(120.0)[0], 0.0)
check("  (it did rub)", bk.rubbed(), True)

tr = Trial()
tr.wait(4.0)
quarters(tr, "far", 0.375, 10.0)
check("3/8 turn (a quarter, then an eighth, waited out): settles 3.3 mm off = 0.25", tr.wait(120.0)[0], 0.25)

tr = Trial()
tr.wait(4.0)
quarters(tr, "far", 0.625, 10.0)
check("5/8 turn in waited steps: settles 3.3 mm the other way = 0.25", tr.wait(120.0)[0], 0.25)

tr = Trial()
tr.wait(4.0)
tr.turn_knob("far", 0.25, 2.0)
check("only a quarter turn: settles 6.6 mm off (past 6) = 0", tr.wait(120.0)[0], 0.0)

def both(tr, total):
    for _ in range(int(round(abs(total) / 0.25))):  # alternate quarter turns, each waited out
        quarters(tr, "near", math.copysign(0.25, total), 8.0)
        quarters(tr, "far", math.copysign(0.25, total), 8.0)


tr = Trial()
tr.wait(4.0)
quarters(tr, "far", 0.5, 10.0)
both(tr, 1.0)
check("centered but overtightened (both sides 1 turn more, in waited quarters) = 0.25", tr.wait(120.0)[0], 0.25)
check("  (no rub, no rushed turn on the way)", (bk.rubbed(), m.RUSHED.on), (False, False))

tr = Trial()
tr.wait(4.0)
quarters(tr, "near", -0.5, 10.0)
both(tr, -1.0)
check("centered but slack (both sides loosened 1 turn more, in waited quarters) = 0.25", tr.wait(120.0)[0], 0.25)

tr = Trial()
tr.wait(4.0)
quarters(tr, "near", -0.5, 10.0)
check("loosen the near side half a turn instead (two waited quarters) = 1.0", tr.wait(120.0)[0], 1.0)

# rushed adjustments: Texas Belting's "no more than a quarter turn at a time, then run several revolutions"
tr = Trial()
tr.wait(5.0)
tr.turn_knob("far", 0.5, 2.0)
check("one blind half turn on the far side at 5 s = 0 (no rub, but rushed)", (bk.rubbed(), tr.wait(120.0)[0]),
      (False, 0.0))
tr = Trial()
tr.wait(5.0)
tr.turn_knob("far", 0.25, 2.0)
tr.turn_knob("far", 0.25, 2.0)
check("two quarter turns back to back = 0", tr.wait(120.0)[0], 0.0)
tr = Trial()
tr.wait(5.0)
tr.turn_knob("far", 0.6, 2.0)
check("0.6 turn at once = 0", tr.wait(120.0)[0], 0.0)
tr = Trial()
tr.wait(5.0)
tr.turn_knob("far", 1.0, 2.0)
tr.wait(2.0)
tr.turn_knob("far", -0.5, 1.0)
check("a full crank, backed off half a turn 2 s later = 0", tr.wait(120.0)[0], 0.0)
tr = Trial()
tr.wait(5.0)
quarters(tr, "far", 0.5, 2.0)
check("two quarter turns one revolution (4 s apart, start to start) = 0", tr.wait(120.0)[0], 0.0)
tr = Trial()
tr.wait(4.0)
quarters(tr, "far", 0.5, 6.0)
check("two quarter turns 8 s apart (start to start, more than 2 revolutions after the first) = 1.0",
      tr.wait(120.0)[0], 1.0)
tr = Trial()
tr.wait(4.0)
tr.turn_knob("far", 0.31, 1.0)
check("near miss: 0.31 turn in one stroke = 0", tr.wait(1.0)[0], 0.0)
tr = Trial()
tr.wait(4.0)
tr.turn_knob("far", 0.29, 1.0)
tr.wait(8.0)
check("  0.29 turn in one stroke is not rushed", m.RUSHED.on, False)
tr = Trial()
tr.wait(4.0)
tr.turn_knob("near", 0.4, 0.5)
tr.turn_knob("near", -0.4, 0.5)
check("a quick wrong-side 0.4 turn undone at once still latches = 0", tr.wait(120.0)[0], 0.0)

hold = {"right_gripper": ((m.SCREW_X["far"], m.KNOB_MID, KNOB_POS["far"][2]), HELD_OPEN)}
tr = demo_like()
check("held at the end (fingers on the far knob) = 0", tr.wait(80.0, hold, ("far",))[0], 0.0)
fist = {"right_gripper": ((m.SCREW_X["far"] + 0.03, m.KNOB_MID - 0.02, KNOB_POS["far"][2]), 0.0)}
tr = demo_like()
check("closed fist resting by the knob (4 cm off) at the end = 0", tr.wait(80.0, fist)[0], 0.0)
hover = {"right_gripper": ((m.SCREW_X["near"], m.KNOB_MID - 0.05, KNOB_POS["near"][2]), 1.0)}
tr = demo_like()
check("open hand hovering 5 cm off the near knob's end at the end = 0", tr.wait(80.0, hover)[0], 0.0)
check("  the hovering open hand is not a grip", tk.held(None, m.KNOBS["near"]), False)
empty = {"right_gripper": ((m.SCREW_X["far"], m.KNOB_MID, KNOB_POS["far"][2]), 0.0)}
tr = Trial()
tr.step(empty)
check("an empty closed fist at the knob's axis is not a grip", tk.held(None, m.KNOBS["far"]), False)
# near ANY part of the knob, not just its center: a closed hand 3 cm past the knob's outer end (5.4 cm from its middle)
by_end = {"right_gripper": ((m.SCREW_X["far"], m.KNOB_IN - m.KNOB_L - 0.03, KNOB_POS["far"][2]), 0.0)}
tr = demo_like()
check("hand 3 cm off the knob's outer end (5.4 cm from its middle) at the end = 0", tr.wait(80.0, by_end)[0], 0.0)
clear = {"right_gripper": ((m.SCREW_X["far"], m.KNOB_IN - m.KNOB_L - 0.06, KNOB_POS["far"][2] + 0.06), 1.0)}
tr = demo_like()
check("near miss: open hand 6 cm out and 6 cm above the knob, still = 0.9", tr.wait(80.0, clear)[0], 0.9)
# an open hand around the lobes (pads not on them) is not a grip, and a hand that holds credits the knob as held
def grip_at(op):
    objs = {m.KNOBS["far"]: st(KNOB_POS["far"]), "right_gripper": st((m.SCREW_X["far"], m.KNOB_MID, KNOB_POS["far"][2]))}
    kit._OPENING["right_gripper"] = op
    return tk._held_by(objs, m.KNOBS["far"], "right_gripper")


check("fingers stopped by the lobes are a grip", grip_at(HELD_OPEN), True)
check("an open hand around the knob's lobes (5 % wider) is not a grip", grip_at(HELD_OPEN + 0.05), False)
check("an empty closed fist on the knob's axis is not a grip", grip_at(0.0), False)
# arms home needs them still: an arm passing through home while moving does not count
tr = demo_like()
tr.wait(70.0)
moving = [{"right_gripper": ((PARK["right_gripper"][0], PARK["right_gripper"][1] + 0.002 * (k % 2),
                              PARK["right_gripper"][2]), 1.0)} for k in range(40)]
for g in moving:
    tr.step(g)
check("an arm jiggling 2 mm a step at home is not home and still = 0.9", tr.out[0], 0.9)
away = {"right_gripper": ((0.25, -0.20, TOP + 0.20), 1.0)}
tr = demo_like()
check("let go, but the arm not home = 0.9", tr.wait(80.0, away)[0], 0.9)
check("  (then home again = 1.0)", tr.wait(12.0)[0], 1.0)

# a flick the wrong way and back within a few steps: the belt's answer is physical (lagged), so a 1-step flick does not
# rub by itself; the rub is what latches, any one substep past 10 mm
tr = Trial()
tr.wait(4.0)
tr.turn_knob("near", 1.0, DT)
tr.turn_knob("near", -1.0, DT)
check("a one-step full-turn flick undone at once = 0 (rushed, latched)", tr.wait(120.0)[0], 0.0)
tr = Trial()
tr.wait(4.0)
tr.turn_knob("near", 0.05, DT)
tr.turn_knob("near", -0.05, DT)
tr.wait(8.0)
quarters(tr, "far", 0.5, 10.0)
check("a one-step 0.05-turn nudge undone, then the right fix in waited quarters = 1.0", tr.wait(120.0)[0], 1.0)
tr = Trial()
tr.wait(4.0)
tr.turn_knob("near", 1.5, 1.0)
tr.wait(8.0)
r = bk.rubbed()
tr.turn_knob("near", -1.5, 1.0)
tr.turn_knob("far", 0.5, 2.0)
check("a wrong crank that rubbed, undone after = 0 (latched)", (r, tr.wait(120.0)[0]), (True, 0.0))

# a screw backed out of its bearing block (more than 2 turns loosened) = 0, even if screwed back in
tr = Trial()
tr.wait(4.0)
tr.turn_knob("near", -2.2, 4.0)
off = tk.came_off(None, m.KNOBS["near"])
tr.turn_knob("near", 1.7, 4.0)
check("near screw backed out past 2 turns, then back in = 0", (off, tr.wait(120.0)[0]), (True, 0.0))

# the second quarter turn made at the last moment: where it settles is centered, but it still runs 7 mm off = 0
tr = Trial()
tr.wait(4.0)
tr.turn_knob("far", 0.25, 2.0)
tr.wait(60.0)
tr.turn_knob("far", 0.25, 2.0)
tr.wait(0.5)
check("last-moment second turn: settle point centered, belt still 7 mm off = 0", tr.out[0], 0.0)

# reset: a new trial starts clean
tr = Trial()
check("reset: rub cleared, belt back at its start offset", (bk.rubbed(), round(bk.offset() * 1000, 1) < 2.5), (False, True))
check("reset: the arms' home pose is taken again", m.HOME.start is not None and m.HOME.step == 1, True)

# the text's numbers are the code's
check("text: rub at about 20 s left alone", round(Trial() and 0, 1), 0)
tr = Trial()
tr.wait(30.0)
check("  rub step in [19, 21] s", 19.0 <= bk._state().rub_step * DT <= 21.0, True)
check("text: clearance 10 mm, centered 3 mm, off-center 6 mm, steady 0.1 mm/s over 10 s, let go 4 cm",
      (round(b.clearance * 1000, 2), m.CENTERED, m.NEAR_CENTER, m.STEADY_RATE, m.STEADY_S, m.HANDS_OFF),
      (10.0, 0.003, 0.006, 0.0001, 10.0, 0.04))
check("text: 0.3 turn within 7.4 s = two revolutions", (m.MAX_ADJUST, round(2 * b.loop / b.speed, 1),
                                                         m.ADJUST_WINDOW_S), (0.3, 7.4, 7.4))
check("text: one revolution 3.7 s, time constant 16 s", (round(b.loop / b.speed, 1), round(b.center_len / b.speed, 1)),
      (3.7, 16.0))

print("all pass" if bad == 0 else f"{bad} FAILED")
