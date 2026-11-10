"""Offline checks of the corner-turn scorer: step-by-step joint histories, near misses included, no simulator.

    python test_task.py        (with the yam_arena package importable; prints "all pass")
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, pv = m.kit, m.pv
PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
DOWN_A = math.radians(-58)  # the handwheel with the needle at the bottom of its stroke
UP_A = m.WHEEL_START


def st(pos, tip=0.0):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - np.array([0.01, 0.01, tip]), high=p + 0.01)


class Trial:
    """One trial's joints; each call to step() is one control step through the kit's record and the scorer."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.j = {"wheel": UP_A, "lifter": m.FOOT_DOWN_Q, "panel_turn": 0.0, "panel_x": 0.0, "panel": 0.0}

    def step(self, left=None, right=None, **j):
        self.j.update(j)
        for k, v in self.j.items():
            kit._JOINTS[f"machine/{k}"] = kit.JointState(v, 0.0, False)
        pv.step(m.KEY, dict(self.j))
        g = dict(PARK)
        if left is not None:
            g["left_gripper"] = left
        if right is not None:
            g["right_gripper"] = right
        objs = {"machine": st(m.BASE), **{k: st(p, m.PAD_TIP_PAST_TCP) for k, p in g.items()}}
        ok = m.success(objs)
        return m.score(objs), ok

    def rest(self, n=10, **kw):
        out = None
        for _ in range(n):
            out = self.step(**kw)
        return out

    def wheel(self, a0, a1, n=8):
        for k in range(1, n + 1):
            self.step(wheel=a0 + (a1 - a0) * k / n, right=m._knob_tcp(a0 + (a1 - a0) * k / n))

    def foot(self, q0, q1, n=6):
        for k in range(1, n + 1):
            q = q0 + (q1 - q0) * k / n
            self.step(lifter=q, right=m._grip_tcp(q))

    def turn(self, d0, d1, n=30, press=True):
        """The panel turned from d0 to d1 degrees, by pads pressing on it 10 cm from the needle."""
        for k in range(1, n + 1):
            d = d0 + (d1 - d0) * k / n
            b = math.radians(60 + d)
            pad = (m.NX + 0.10 * math.cos(b), m.NY + 0.10 * math.sin(b), m.TOP + m.OVER + m.TIP - 0.004)
            self.step(panel_turn=math.radians(d), left=pad if press else None)


def check(got, want, what):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"{what}: got {got}, want {want}")
    print(f"ok   {what}: {got}")


def correct(t, home=True, lower=True):
    t.rest(2)
    t.wheel(UP_A, DOWN_A)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.turn(0, 90)
    if lower:
        t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    return t.rest(25, right=None if home else (0.2, -0.1, m.TOP + 0.18))


def test_nothing():
    check(Trial().rest(40)[0], 0.0, "doing nothing")


def test_correct():
    s, ok = correct(Trial())
    check(s, 1.0, "needle down, foot up, 90 deg about the needle, foot down, hands off, home")
    assert ok


def test_not_home():
    check(correct(Trial(), home=False)[0], 0.9, "the same, right arm left over the machine")


def test_forgot_foot():
    check(correct(Trial(), lower=False)[0], 0.75, "turned right but the foot left up")


def test_needle_up_turn():
    t = Trial()
    t.rest(2)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.turn(0, 90)  # needle still up: the panel spun off the needle
    t.wheel(UP_A, DOWN_A)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 0.0, "turned with the needle up, then needle down and foot down")


def test_needle_up_turn_undone():
    t = Trial()
    t.rest(2)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.turn(0, 30)
    t.turn(30, 0)  # turned back: the hole is still where it was, but the rule is irreversible
    t.wheel(UP_A, DOWN_A)
    t.turn(0, 90)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 0.0, "a turn off the needle undone before the real turn")


def test_small_nudge_needle_up():
    t = Trial()
    t.rest(2)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.turn(0, 3.5)  # a 3.5 deg bump with the needle up (inside the 8 deg tolerance)
    t.turn(3.5, 0)
    t.wheel(UP_A, DOWN_A)
    t.turn(0, 90)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 1.0, "8 deg in all off the needle is tolerated (3.5 there, 3.5 back)")


def test_foot_down_turn():
    t = Trial()
    t.rest(2)
    t.wheel(UP_A, DOWN_A)
    t.turn(0, 90)  # foot never raised: dragged round under the foot
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 0.0, "turned with the foot down")


def test_foot_down_late():
    t = Trial()
    t.rest(2)
    t.wheel(UP_A, DOWN_A)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.turn(0, 75)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    t.turn(75, 90)  # the last 15 deg under the foot
    check(t.rest(25)[0], 0.0, "the last 15 deg dragged under the lowered foot")


def test_half_turn():
    t = Trial()
    t.rest(2)
    t.wheel(UP_A, DOWN_A)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.turn(0, 50)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 0.5, "turned 50 deg about the needle")
    t2 = Trial()
    t2.rest(2)
    t2.wheel(UP_A, DOWN_A)
    t2.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t2.turn(0, 40)
    check(t2.rest(25)[0], 0.25, "40 deg is not yet halfway: set up only")


def test_over_and_under():
    for d, want in ((81.0, 0.5), (82.5, 1.0), (97.5, 1.0), (99.0, 0.5), (130.0, 0.5)):
        t = Trial()
        t.rest(2)
        t.wheel(UP_A, DOWN_A)
        t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
        t.turn(0, d)
        t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
        check(t.rest(25)[0], want, f"turned {d} deg")


def test_setup_only():
    t = Trial()
    t.rest(2)
    t.wheel(UP_A, DOWN_A)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    check(t.rest(25)[0], 0.25, "needle in at the corner and foot up, no turn")
    check(t.rest(5, right=m._grip_tcp(m.FOOT_UP_Q))[0], 0.0, "the same while still holding the lifter grip")


def test_cheap_partials():
    t = Trial()
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    check(t.rest(25)[0], 0.0, "foot raised, needle never lowered")
    t = Trial()
    t.wheel(UP_A, DOWN_A)
    check(t.rest(25)[0], 0.0, "needle lowered, foot never raised")
    t = Trial()
    t.wheel(UP_A, math.radians(25))  # needle 1.4 mm above the leather's top: not in
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    check(t.rest(25)[0], 0.0, "handwheel turned short of the needle going in")
    t = Trial()
    t.rest(5, left=(m.NX + 0.05, m.NY + 0.05, m.TOP + m.OVER + m.TIP))
    check(t.rest(25)[0], 0.0, "pads pressed on the panel, nothing else")


def test_off_corner_hole():
    t = Trial()
    t.rest(2)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.rest(5, panel_x=0.005)  # the panel slid 5 mm before the needle came down
    t.wheel(UP_A, DOWN_A)
    t.turn(0, 90)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 0.0, "the needle went in 5 mm off the corner")
    t = Trial()
    t.rest(2)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.rest(5, panel_x=0.0015)  # 1.5 mm: within the 2 mm tolerance
    t.wheel(UP_A, DOWN_A)
    t.turn(0, 90)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 1.0, "1.5 mm off the corner is tolerated")


def test_forced_sideways():
    t = Trial()
    t.rest(2)
    t.wheel(UP_A, DOWN_A)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.turn(0, 90)
    t.rest(3, panel=0.003)  # pushed 3 mm through the needle's stops
    t.rest(3, panel=0.0)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 0.0, "the panel forced 3 mm sideways on the needle (bent needle)")


def test_moved_after_turn():
    t = Trial()
    t.rest(2)
    t.wheel(UP_A, DOWN_A)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.turn(0, 90)
    t.wheel(DOWN_A, UP_A)  # needle raised again
    t.rest(5, panel_x=0.01)  # and the panel slid 1 cm off the corner
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 0.0, "turned, then the needle raised and the panel slid off the corner (set-up undone)")


def test_still_pressing():
    t = Trial()
    correct(t)
    b = math.radians(150)
    pad = (m.NX + 0.10 * math.cos(b), m.NY + 0.10 * math.sin(b), m.TOP + m.OVER + m.TIP)
    check(t.rest(5, left=pad)[0], 0.0, "all done but the pads still resting on the panel")
    check(t.rest(5, right=m._knob_tcp(DOWN_A))[0], 0.0, "all done but a hand on the handwheel knob")


def test_reset():
    t = Trial()
    t.rest(2)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.turn(0, 90)
    check(t.rest(5)[0], 0.0, "trial 1: turned with the needle up")
    check(correct(Trial())[0], 1.0, "trial 2 after the scene reset: a clean history, full score")


def test_needle_geometry():
    check(round(m.needle_tip(m.WHEEL_START) - m.OVER, 4), 0.0065, "start: the tip 6.5 mm above the leather")
    assert not m.needle_in({"wheel": m.WHEEL_START, "panel_x": 0.0, "panel": 0.0})
    assert m.needle_in({"wheel": DOWN_A, "panel_x": 0.0, "panel": 0.0})
    assert not m.needle_in({"wheel": DOWN_A, "panel_x": 0.02, "panel": 0.0}), "slid off: the needle misses it"
    print("ok   needle in/out geometry")


def test_turn_jitter():
    """QC fix 1: sim jitter while the foot is down (or the needle out) must not add up; a slow real turn must."""
    rng = np.random.default_rng(0)
    t = Trial()
    correct(t)
    for _ in range(1700):  # the rest of a 2,400-step run, jittering 8e-5 rad (0.005 deg) a step under the foot
        out = t.step(panel_turn=math.radians(90) + rng.normal(0, 8e-5))
    check(out[0], 1.0, "a correct run then 1,700 steps of resting jitter under the foot")
    t = Trial()
    t.rest(2)
    t.wheel(UP_A, DOWN_A)
    for k in range(1, 401):  # 0.03 deg a step, under the 0.05 deg dead band each step: 12 deg in all, foot down
        t.step(panel_turn=math.radians(0.03 * k))
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.turn(12, 90)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 0.0, "a slow 12 deg drag under the foot (0.03 deg a step) still counts in full")


def test_foot_between_detents():
    """QC fix 2: the foot is down only with the lever in its down detent's zone (q <= 0 + 0.002)."""
    assert m.foot_down({"lifter": m.FOOT_DOWN_Q}) and m.foot_down({"lifter": 0.0})
    assert not m.foot_down({"lifter": 0.01}) and not m.foot_down({"lifter": 0.04})
    t = Trial()
    t.rest(2)
    t.wheel(UP_A, DOWN_A)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.turn(0, 90)
    t.foot(m.FOOT_UP_Q, 0.04)  # left between the detents: the foot 3.4 mm above the leather
    check(t.rest(25)[0], 0.75, "lever left between the detents (foot 3.4 mm up) is not foot down")
    t = Trial()
    t.rest(2)
    t.wheel(UP_A, DOWN_A)
    t.foot(m.FOOT_DOWN_Q, -0.05)  # lever only nudged, still in the down zone: the scorer counts the foot down
    t.turn(0, 90)
    t.foot(-0.05, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 0.0, "turned with the lever nudged but still in its down zone")


def test_partial_needle_hole():
    """QC fix 3: a tip below the leather's top off the corner is a stray hole, though not through."""
    t = Trial()
    t.rest(2)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.rest(3, panel_x=0.006)
    a = math.radians(22)  # tip 0.9 mm above the table: in the leather, not 1 mm past its underside
    assert not m.needle_in({"wheel": a, "panel_x": 0.006, "panel": 0.0}) and m.needle_tip(a) < m.OVER
    t.wheel(UP_A, a)
    t.wheel(a, UP_A)
    t.rest(3, panel_x=0.0)
    t.wheel(UP_A, DOWN_A)
    t.turn(0, 90)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 0.0, "the needle punched part way in 6 mm off the corner, then the real job")
    t = Trial()
    t.rest(2)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.wheel(UP_A, math.radians(25))  # 1.4 mm above the leather's top: no hole
    t.wheel(math.radians(25), UP_A)
    t.wheel(UP_A, DOWN_A)
    t.turn(0, 90)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 1.0, "the needle stopped 1.4 mm above the leather (no hole), then the real job")


def test_swept_onto_needle():
    """QC fix 4: a panel pushed onto an already lowered needle is torn, even if it ends at the corner."""
    t = Trial()
    t.rest(2)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.rest(3, panel_x=0.02)  # slid off the needle's line
    t.wheel(UP_A, DOWN_A)  # needle down beside the panel
    t.step(panel_x=0.0)  # swept 2 cm back under it in one step, ending at the corner
    t.turn(0, 90)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(25)[0], 0.0, "panel swept onto the lowered needle in one step")


def test_hand_on_lever_arm():
    """QC fix 5: a hand on the lifter lever's arm (not its grip) is still a hand on the machine."""
    t = Trial()
    correct(t)
    q = m.FOOT_DOWN_Q
    p = (m.LIFTER[0] - 0.025 * math.cos(q), m.LIFTER[1], m.TOP + m.LIFTER[2] + 0.025 * math.sin(q) + 0.01)
    check(t.rest(5, right=p)[0], 0.0, "a hand on the lever's arm 2.5 cm from its pivot")
    check(t.rest(25)[0], 1.0, "and full score once it is back home and still")


def test_undone_setup():
    """QC fix 6: the 0.25 set-up needs the needle still in at the corner; a run undone back to its start scores 0."""
    t = Trial()
    t.rest(2)
    t.wheel(UP_A, DOWN_A)
    t.foot(m.FOOT_DOWN_Q, m.FOOT_UP_Q)
    t.foot(m.FOOT_UP_Q, m.FOOT_DOWN_Q)
    check(t.rest(5)[0], 0.25, "needle in at the corner, foot raised then lowered: set up")
    t.wheel(DOWN_A, UP_A)
    check(t.rest(25)[0], 0.0, "and then the needle raised again: back at the start state")


def test_home_at_time_limit():
    """QC fix 6: arms must be home and still for 1 s; passing through home on the last steps is 0.9."""
    t = Trial()
    correct(t, home=False)
    t.step(right=(0.2, -0.26, m.TOP + 0.2))
    check(t.step(right=(0.2, -0.2605, m.TOP + 0.2))[0], 0.9, "arms reach home only on the last 2 steps")
    check(t.rest(18, right=(0.2, -0.2605, m.TOP + 0.2))[0], 0.9, "home and still 19 steps (0.95 s): not yet")
    check(t.step(right=(0.2, -0.2605, m.TOP + 0.2))[0], 1.0, "home and still 20 steps: 1 s")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
    print("all pass")
