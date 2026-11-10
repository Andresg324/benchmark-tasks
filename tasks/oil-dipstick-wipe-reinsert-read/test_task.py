"""Offline checks of the dipstick scorer: step-by-step dipstick, rag and gripper histories, no simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
T = m.TOP
HOME = {"left_gripper": (0.10, 0.25, T + 0.20), "right_gripper": (0.10, -0.25, T + 0.20)}
RAG_HOME = m.RAG_START
OUTQ = m.BLADE + 0.012


def st(pos, half=0.01):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - half, high=p + half)


def rag_st(pos):
    p = np.array(pos, float)
    h = np.array(m.RAG) / 2
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - h, high=p + h)


def handle_at(q):
    return (m.SX, m.SY, T + m.MOUTH + q + m.HANDLE_C)


class Trial:
    """One trial: q (dipstick), who holds it, where the rag and the grippers are."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.q, self.qd = 0.0, 0.0
        self.right, self.ropen = HOME["right_gripper"], 1.0
        self.left, self.lopen = HOME["left_gripper"], 1.0
        self.rag = RAG_HOME
        self.out = (0.0, False)
        self.step()  # the trial's first step, with the arms home

    def step(self, n=1):
        for _ in range(n):
            kit._JOINTS["engine/stick"] = kit.JointState(self.q, self.qd, False)
            kit._OPENING["right_gripper"], kit._OPENING["left_gripper"] = self.ropen, self.lopen
            objs = {"engine": st(m.BASE), "engine/stick": st((m.SX, m.SY, T + m.MOUTH + self.q)),
                    "rag": rag_st(self.rag), "right_gripper": st(self.right), "left_gripper": st(self.left)}
            ok = m.success(objs)
            self.out = (m.score(objs), ok)
        return self.out

    # moves
    def grab(self):
        self.right, self.ropen = handle_at(self.q), 0.3
        return self.step(3)

    def slide(self, q1, n=20):
        q0 = self.q
        for k in range(1, n + 1):
            self.q = q0 + (q1 - q0) * k / n
            self.qd = (q1 - q0) / n * 20
            if self.ropen < 0.5:
                self.right = handle_at(self.q)
            self.step()
        self.qd = 0.0
        return self.step()

    def hold(self, n):
        return self.step(n)

    def take_rag(self):
        self.left, self.lopen = (RAG_HOME[0], RAG_HOME[1], RAG_HOME[2]), 0.3
        self.step(2)
        self.rag = (RAG_HOME[0], RAG_HOME[1], T + 0.15)
        self.left = self.rag
        return self.step(2)

    def wipe(self, z0, z1, n=30, dy=0.0225, off=1):
        for k in range(n + 1):
            self.rag = (m.SX, m.SY + dy, z0 + (z1 - z0) * k / n)
            self.left = self.rag
            self.step()
        self.rag = (m.SX, m.SY + 0.08, z1)  # rag lifted off the blade for `off` steps
        self.left = self.rag
        return self.step(off)

    def rag_down(self):
        self.rag = RAG_HOME
        self.left = RAG_HOME
        self.step(2)
        self.lopen = 1.0
        self.left = HOME["left_gripper"]
        return self.step(2)

    def let_go(self):
        self.ropen = 1.0
        self.step(2)
        self.right = HOME["right_gripper"]
        return self.step(3)


WT, WB = T + m.MOUTH + 0.055, T + m.MOUTH + 0.016  # a full wipe stroke (rag middle)


def full(t, reseat_q=0.0, read=30, final_q=0.0, put_rag=True, release=True, wipe=(WT, WB), dy=0.0225):
    t.grab()
    t.slide(OUTQ)
    t.take_rag()
    if wipe:
        t.wipe(*wipe, dy=dy)
    t.slide(reseat_q)
    t.hold(5)
    t.slide(OUTQ)
    t.hold(read)
    t.slide(final_q)
    if release:
        t.let_go()
    if put_rag:
        t.rag_down()
    return t.hold(25)


def check(name, got, want):
    ok = abs(got - want) < 1e-9
    print(f"{'ok ' if ok else 'BAD'} {name}: {got} (want {want})")
    return ok


results = []
t = Trial()
results.append(check("do nothing", t.hold(40)[0], 0.0))

t = Trial()
results.append(check("full procedure", full(t)[0], 1.0))
results.append(check("full procedure succeeds", float(t.out[1]), 1.0))

t = Trial()
full(t)
t.right = (0.20, -0.22, T + 0.18)  # done but the right arm not home
results.append(check("done, arm not home", t.step(3)[0], 0.9))

t = Trial()  # single pull: pull, read, push back in (no wipe)
t.grab(); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.take_rag(); t.rag_down()
results.append(check("single pull, read, reinsert", t.hold(25)[0], 0.0))

t = Trial()
results.append(check("wipe skipped (pull, reseat, pull, read, reseat)", full(t, wipe=None)[0], 0.0))

t = Trial()
results.append(check("rag far from the blade", full(t, dy=0.06)[0], 0.0))

t = Trial()  # a rag passed 3 cm from the axis hovers about 8 mm clear of the blade's face: not a wipe
results.append(check("rag hovering 3 cm from the blade", full(t, dy=0.03)[0], 0.0))

t = Trial()
results.append(check("short wipe (3 cm of the 4 cm span)", full(t, wipe=(WB + 0.03, WB))[0], 0.0))

S0 = T + m.MOUTH + OUTQ - m.BLADE  # the tip's height when the test pulls the blade out (the rag's middle here: s = 0)


def strokes(t, *legs, off=10):
    """Pull, wipe with the given strokes (rag middle heights above the tip), each followed by `off` steps off the blade,
    then reseat, read, reseat, let go and put the rag down."""
    t.grab(); t.slide(OUTQ); t.take_rag()
    for z0, z1 in legs:
        t.wipe(S0 + z0, S0 + z1, off=off)
    t.slide(0.0); t.hold(5); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down()
    return t.hold(25)[0]


t = Trial()  # the rag pressed on the blade and held there 1.5 s: touching is not wiping
results.append(check("dab (rag held still on the blade)", strokes(t, (0.03, 0.03)), 0.0))

t = Trial()  # several dabs up and down the blade, lifted off between them: no sliding, nothing wiped
results.append(check("dabs at 4, 2.5 and 0.5 cm, lifted between", strokes(t, (0.04, 0.04), (0.025, 0.025), (0.005, 0.005)),
                     0.0))

t = Trial()  # the middle slid from 4 cm to 2 cm: the tip and the crosshatch stay smeared
results.append(check("half the length (4 to 2 cm)", strokes(t, (0.04, 0.02)), 0.0))

t = Trial()  # scrubbed up and down the upper half ten times: the union is still half the span
results.append(check("half the length scrubbed back and forth", strokes(t, *[(0.04, 0.02), (0.02, 0.04)] * 5, off=0), 0.0))

t = Trial()  # review, Nov 10: a full stroke with the rigid pad held 4.5 mm clear of the blade (middle still in RAG_TOUCH)
results.append(check("full procedure, rag 4.5 mm clear of the blade", full(t, dy=0.026)[0], 0.0))

t = Trial()  # review: two dabs 4.2 cm apart with 3 steps off between: nothing credited across the gap
results.append(check("dab at 4.2 cm, 3 steps off, dab at the tip", strokes(t, (0.042, 0.042), (0.0, 0.0), off=3), 0.0))

t = Trial()  # review: one touching step at 4.2 cm, the next at the tip: a jump, not a slide
t.grab(); t.slide(OUTQ); t.take_rag()
t.wipe(S0 + 0.042, S0 + 0.0, n=1, off=10)  # two touching steps, 4.2 cm apart
t.slide(0.0); t.hold(5); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down()
results.append(check("one-step jump from 4.2 cm to the tip", t.hold(25)[0], 0.0))

t = Trial()  # one stroke, middle from 4.2 cm to the tip
results.append(check("full stroke (4.2 cm to the tip)", strokes(t, (0.042, 0.0)), 1.0))

t = Trial()  # the same stroke drawn upward
results.append(check("full stroke upward", strokes(t, (0.0, 0.042)), 1.0))

t = Trial()  # 3.4 cm of the 4 cm: 85%
results.append(check("stroke stopping 6 mm short of the tip", strokes(t, (0.04, 0.006)), 1.0))

t = Trial()  # two strokes, rag lifted off 0.5 s between, together covering the span (each alone is half): counted,
# because together they rub every part of the blade a single stroke would
results.append(check("two half strokes covering the span", strokes(t, (0.042, 0.02), (0.021, 0.0)), 1.0))

t = Trial()  # half a stroke, the tip dipped back into the tube (smeared again), out, the other half: not a wipe
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(S0 + 0.042, S0 + 0.02, off=10); t.slide(0.06); t.slide(OUTQ)
t.wipe(S0 + 0.021, S0, off=10); t.slide(0.0); t.hold(5); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down()
results.append(check("half stroke, tip back in the tube, other half", t.hold(25)[0], 0.0))

t = Trial()  # a full stroke with the contact lost 2 steps in the middle (the pad chattering): still one stroke
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(S0 + 0.042, S0 + 0.022, n=15, off=2); t.wipe(S0 + 0.02, S0, n=15, off=10)
t.slide(0.0); t.hold(5); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down()
results.append(check("full stroke with a 2-step contact gap", t.hold(25)[0], 1.0))

t = Trial()  # rag touched at the top, lifted 0.5 s, touched at the bottom: the gap is not counted as slid over
results.append(check("touch at the top, lift, touch at the bottom", strokes(t, (0.042, 0.042), (0.0, 0.0)), 0.0))

t = Trial()
results.append(check("half-seated reinsert before the second pull (never seated)", full(t, reseat_q=0.045)[0], 0.0))

t = Trial()
results.append(check("second pull not held still 1 s", full(t, read=10)[0], 0.25))

t = Trial()
results.append(check("left out at the end", full(t, final_q=OUTQ)[0], 0.5))

t = Trial()
results.append(check("left half in at the end", full(t, final_q=0.04)[0], 0.5))

t = Trial()
results.append(check("still held at the end", full(t, release=False)[0], 0.5))

t = Trial()
results.append(check("rag still in hand at the end", full(t, put_rag=False)[0], 0.75))

t = Trial()  # the rag swiped by the other arm while the stick is held still: a real wipe, counts
results.append(check("rag drawn down the still blade by the other arm", full(t)[0], 1.0))

t = Trial()  # the wiped blade pushed half in and out again, then wiped again and done right: recovered
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB); t.slide(0.04); t.slide(OUTQ)
t.wipe(WT, WB); t.slide(0.0); t.hold(5); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down()
results.append(check("half-dip then re-wiped and done right", t.hold(25)[0], 1.0))

t = Trial()  # dropped back in after the wipe (not pushed by hand), then the rest
t.grab(); t.slide(OUTQ); t.take_rag(); t.wipe(WT, WB); t.ropen = 1.0; t.right = HOME["right_gripper"]; t.slide(0.0)
t.grab(); t.slide(OUTQ); t.hold(30); t.slide(0.0); t.let_go(); t.rag_down()
results.append(check("dropped back in instead of pushed (never seated by hand)", t.hold(25)[0], 0.0))

t = Trial()  # wiping while the blade is still in the tube does nothing
t.take_rag(); t.wipe(T + m.MOUTH + 0.03, T + m.MOUTH + 0.005)
results.append(check("rag rubbed on the cap with the stick seated", t.hold(5)[0], 0.0))

full(Trial())  # reset: a new trial after a perfect one starts from 0
t = Trial()
results.append(check("history reset after a full trial", t.hold(10)[0], 0.0))

print("all pass" if all(results) else "FAILURES")
