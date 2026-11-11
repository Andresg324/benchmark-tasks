"""Offline tests of the circlip task's scorer (no simulator). Run: /data/IsaacLab-Arena/.venv/bin/python test_task.py

The ring model runs as in the sim (ringkit.update each step); the pliers, grippers and fixture are hand-made states.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402
import ringkit as rk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import ObjectState, load  # noqa: E402

T = load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "task.py")).module
KEY = T.KEY
R_HOME = np.array([0.20, -0.30, T.TOP + 0.25])
L_HOME = np.array([0.20, 0.30, T.TOP + 0.25])
I = (0.0, 0.0, 0.0, 1.0)


def st(pos, quat=I):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p,
                       high=p)


class Sim:
    """A trial: the right gripper at `tcp` with finger gap `gap`; the pliers held in it (squeezed to the gap) or
    lying where they were let go."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.tcp = R_HOME.copy()
        self.gap = 2 * abs(FINGER_OPEN)
        self.pliers = np.array(T.PLIERS_POS, float)
        self.q = 0.0
        self.yaw = 0.0
        self.held = False
        self.roll = 0.0  # deg: the pliers rolled about their length (tips at different heights; the ring tilts)
        self.scores = []

    def objects(self):
        rot = np.array([[math.cos(self.yaw), -math.sin(self.yaw), 0], [math.sin(self.yaw), math.cos(self.yaw), 0],
                        [0, 0, 1]])
        h = math.radians(self.roll) / 2
        quat = kit._quat_mul((0.0, 0.0, math.sin(self.yaw / 2), math.cos(self.yaw / 2)), (math.sin(h), 0.0, 0.0,
                                                                                        math.cos(h)))
        if self.held:  # the gripper squeezes the handles: they sit symmetric about the tool point
            self.q = min(max(T.HANDLE_GAP - self.gap, 0.0), T.SQUEEZE_MAX)
            self.pliers = self.tcp - rot @ np.array([0.0, self.q / 2, 0.0])
        else:
            self.q = 0.0
        kit._OPENING["right_gripper"] = self.gap / (2 * abs(FINGER_OPEN))
        kit._OPENING["left_gripper"] = 1.0
        return {"fixture": st((T.SX, T.SY, T.TOP)), "pliers": st(self.pliers, quat),
                "pliers/jaw": st(self.pliers + rot @ np.array([0.0, self.q, 0.0]), quat),
                "rest": st((T.REST[0], T.REST[1], T.TOP + T.REST[2])),
                "right_gripper": st(self.tcp), "left_gripper": st(L_HOME)}

    def step(self, n=1):
        for _ in range(n):
            o = self.objects()
            rk.update(o)
            T.success(o)
            self.scores.append(T.score(o))
        return self

    def move(self, to, steps=20, gap=None, yaw=None):
        a, b = self.tcp.copy(), np.asarray(to, float)
        g0, g1 = self.gap, (self.gap if gap is None else gap)
        y0, y1 = self.yaw, (self.yaw if yaw is None else yaw)
        for k in range(1, steps + 1):
            f = k / steps
            self.tcp, self.gap, self.yaw = a + f * (b - a), g0 + f * (g1 - g0), y0 + f * (y1 - y0)
            self.step()
        return self

    @property
    def score(self):
        return self.scores[-1]


EX, EY = T.SX - T.LUG_R - T.TIP_X, T.SY
EZ = T.TOP + T.END_H + 0.0003 - T.TIP_END
SZ = T.TOP + T.END_H - (T.GEAR_FACE + 0.0005) - T.TIP_END
HOLD, SPREAD = 0.037, 0.0274  # handle gaps: light grip; squeezed to a 40.6 mm bore


def pick(s):
    s.step(2)
    s.move(T.PLIERS_POS, gap=2 * abs(FINGER_OPEN))
    s.held = True
    s.move(T.PLIERS_POS, steps=5, gap=HOLD)
    s.move((T.PLIERS_POS[0], T.PLIERS_POS[1], T.PLIERS_POS[2] + 0.1))
    return s


def to_ring(s, spread=SPREAD, down_to=SZ):
    s.move((EX, EY, EZ + 0.02))
    s.move((EX, EY, EZ))
    assert rk.engaged(None, KEY), "tips in the lug holes"
    s.move((EX, EY, EZ), steps=10, gap=spread)
    s.move((EX, EY, down_to), steps=40)
    return s


def orbit(s, deg, steps=15):
    a = math.radians(deg)
    dx, dy = EX - T.SX, EY - T.SY
    s.move((T.SX + dx * math.cos(a) - dy * math.sin(a), T.SY + dx * math.sin(a) + dy * math.cos(a), s.tcp[2]),
           steps=steps, yaw=a)


def put_down(s, home=True):
    s.move((s.tcp[0], s.tcp[1], s.tcp[2] + 0.03), yaw=0.0)
    s.move((T.PLIERS_POS[0], T.PLIERS_POS[1], T.PLIERS_POS[2] + 0.08))
    s.move(T.PLIERS_POS)
    s.move(T.PLIERS_POS, steps=5, gap=2 * abs(FINGER_OPEN))
    s.held = False
    s.move((T.PLIERS_POS[0], T.PLIERS_POS[1], T.PLIERS_POS[2] + 0.08))
    if home:
        s.move(R_HOME)
    s.step(10)
    return s


def full(spread=SPREAD, turn=15, home=True, down_to=SZ):
    s = to_ring(pick(Sim()), spread, down_to)
    s.move((EX, EY, down_to), steps=10, gap=HOLD)
    if turn:  # like the demo: out one way, back through the start to the other side, and back
        orbit(s, turn)
        orbit(s, -turn)
        orbit(s, 0)
    return put_down(s, home)


def test_do_nothing():
    s = Sim().step(50)
    assert max(s.scores) == 0.0


def test_full_job():
    s = full()
    assert rk.seated(None, KEY) and not rk.overspread(None, KEY)
    assert 40.4 < rk.max_d(None, KEY) * 1000 < 40.8, rk.max_d(None, KEY)
    assert rk.turned_deg(None, KEY) >= 29
    assert s.score == 1.0, s.score
    assert T.success(s.objects())


def test_not_home():
    s = full(home=False)
    assert s.score == 0.9


def test_unchecked():
    s = full(turn=0)
    assert s.score == 0.5


def test_small_turn_not_a_check():
    s = full(turn=6)  # 6 degrees each way: a 12 degree span < 15
    assert s.score == 0.5


def test_overspread_is_scrap():
    s = full(spread=0.0255)  # a 41.3 mm bore: past the 41.0 mm limit, though it still closes into the groove
    assert rk.overspread(None, KEY)
    assert s.score == 0.0 and max(s.scores) == 0.0


def test_not_spread_enough_stays_on_end():
    s = to_ring(pick(Sim()), spread=0.0300)  # a 39.6 mm bore does not clear the 40 mm shaft
    put_down(s)
    assert rk.position(None, KEY) < 0 and s.score == 0.0


def test_short_of_groove():
    s = full(down_to=SZ + 0.012)  # let go 1.2 cm short: it grips the shaft there
    assert rk.groove(None, KEY) is None and not rk.seated(None, KEY)
    assert s.score == 0.0


def test_cocked():
    s = pick(Sim())
    s.move((EX, EY, EZ + 0.02))
    s.move((EX, EY, EZ))
    s.move((EX, EY, EZ), steps=10, gap=SPREAD)
    s.roll = 6.0  # the pliers rolled 6 degrees: the ring is held tilted
    s.move((EX, EY, SZ), steps=40)
    assert rk.max_tilt_deg(None, KEY) > 5.5
    s.move((EX, EY, SZ), steps=10, gap=HOLD)  # let it close tilted: it lands cocked
    assert rk.cocked(None, KEY)
    s.roll = 0.0
    orbit(s, 15)
    orbit(s, 0)
    put_down(s)
    assert rk.cocked(None, KEY) and not rk.seated(None, KEY) and rk.turned_deg(None, KEY) == 0
    assert s.score == 0.0 and max(s.scores) == 0.0


def test_still_held_scores_nothing():
    s = to_ring(pick(Sim()))
    s.move((EX, EY, SZ), steps=10, gap=HOLD)
    orbit(s, 15)
    orbit(s, 0)
    s.move((EX, EY, SZ + 0.03))  # out of the ring, still in hand
    assert rk.seated(None, KEY)
    s.step(20)
    assert max(s.scores) == 0.0


def test_pliers_left_in_ring():
    s = to_ring(pick(Sim()))
    s.move((EX, EY, SZ), steps=10, gap=HOLD)
    orbit(s, 15)
    orbit(s, 0)
    s.move((EX, EY, SZ), steps=5, gap=2 * abs(FINGER_OPEN))  # let go of the handles with the tips in the lugs
    s.held = False
    s.pliers = np.array([EX, EY, SZ])
    s.move(R_HOME)
    s.step(10)
    assert rk.engaged(None, KEY) and s.score == 0.0


def test_pliers_dropped_from_height_not_down():
    s = full()
    s.held = False
    s.pliers = np.array([0.25, -0.20, T.TOP + 0.12])  # left hanging in the air (e.g. resting on the fixture top)
    s.step(10)
    assert s.score == 0.0


def test_twisted_ring_is_scrap():
    s = pick(Sim())
    s.move((EX, EY, EZ + 0.02))
    s.move((EX, EY, EZ - 0.0025))  # tips 4 mm into the lug holes (2.2 mm past the ring)
    for _ in range(T.TWIST_STEPS):  # held twisted for 2 control steps in a row
        o = s.objects()
        o["pliers/jaw"] = st(o["pliers/jaw"].pos + np.array([0, 0, 0.0034]))  # one tip 3.4 mm higher: ~12 degrees
        rk.update(o)
        T.success(o), T.score(o)
    assert rk.engaged(None, KEY) and rk.max_twist_deg(None, KEY) > T.TWIST_LIMIT, rk.max_twist_deg(None, KEY)
    assert T.TRACE.ever("twisted")
    s.move((EX, EY, EZ), steps=10, gap=SPREAD)
    s.move((EX, EY, SZ), steps=40)
    s.move((EX, EY, SZ), steps=10, gap=HOLD)
    orbit(s, 15)
    orbit(s, 0)
    put_down(s)
    assert rk.seated(None, KEY) and s.score == 0.0


def test_hover_touch_push_scores_nothing():
    s = pick(Sim())
    s.move((EX, EY, EZ))
    s.move((EX, EY, SZ), steps=30)  # pushing down without spreading: tips slip out, ring stays on the end
    s.move((T.SX, T.SY, T.TOP + 0.13))
    put_down(s)
    assert max(s.scores) == 0.0 and rk.position(None, KEY) < 0


def _seated_in_hand():
    s = to_ring(pick(Sim()))
    s.move((EX, EY, SZ), steps=10, gap=HOLD)
    return s


def test_wiggle_is_not_a_turn():
    s = _seated_in_hand()
    for _ in range(10):  # +-1 degree ten times: 40 degrees of motion, a 2 degree net turn
        orbit(s, 1.0, steps=1)
        orbit(s, -1.0, steps=1)
    orbit(s, 0, steps=1)
    assert rk.turned_deg(None, KEY) < 2.5, rk.turned_deg(None, KEY)
    put_down(s)
    assert s.score == 0.5


def test_jitter_is_not_a_turn():
    s = _seated_in_hand()
    rng, base = np.random.default_rng(0), s.tcp.copy()
    for _ in range(200):  # 0.1 mm of tool-point noise for 10 s
        s.tcp = base + rng.normal(0, 0.0001, 3) * np.array([1, 1, 0])
        s.step()
    s.tcp = base
    assert rk.turned_deg(None, KEY) < 5, rk.turned_deg(None, KEY)
    put_down(s)
    assert s.score == 0.5


def test_one_way_turn_counts():
    s = _seated_in_hand()
    orbit(s, 16)  # turned 16 degrees one way and left there
    s.move((s.tcp[0], s.tcp[1], s.tcp[2] + 0.03))  # tips lifted straight out
    assert not rk.engaged(None, KEY)
    put_down(s)
    assert 15.5 < rk.turned_deg(None, KEY) < 16.5 and s.score == 1.0


def test_pliers_on_floor_not_down():
    s = full()
    assert s.score == 1.0
    s.pliers = np.array([0.30, -0.45, T.TOP - 0.7])  # knocked off onto the floor
    s.step(30)
    assert s.score == 0.0


def test_pliers_beside_table_not_down():
    s = full()
    s.pliers = np.array([T.TABLE_X[1] + 0.05, -0.20, T.TOP + 0.006])  # level with the top but past the bench's edge
    s.step(30)
    assert s.score == 0.0


def test_held_by_handle_butt():
    s = _seated_in_hand()
    orbit(s, 15)
    orbit(s, 0)
    put_down(s, home=False)
    assert s.score == 0.9
    s.tcp = s.pliers + np.array([-0.04, 0.0, 0.0])  # fingers closed on the handles' end, 4 cm from the grip point
    s.gap = HOLD
    s.step(30)
    assert T._pliers_held(s.objects()) and s.score == 0.0
    s.tcp = s.pliers + np.array([0.07, 0.0, 0.0])  # or on the jaws, by the tips
    s.step(5)
    assert T._pliers_held(s.objects()) and s.score == 0.0
    s.tcp = s.pliers + np.array([0.0, 0.05, 0.0])  # a closed hand 5 cm to the side holds nothing
    s.step(5)
    assert not T._pliers_held(s.objects())


def _cocked_slow_release(gap):
    s = pick(Sim())
    s.move((EX, EY, EZ + 0.02))
    s.move((EX, EY, EZ))
    s.move((EX, EY, EZ), steps=10, gap=SPREAD)
    s.roll = 6.0
    s.move((EX, EY, SZ), steps=40)
    s.move((EX, EY, SZ), steps=20, gap=gap)  # relax to just above the groove bottom, still tilted
    s.move((EX, EY, SZ), steps=1, gap=HOLD)  # then let it close
    cocked = rk.cocked(None, KEY)
    s.roll = 0.0
    orbit(s, 15)
    orbit(s, 0)
    put_down(s)
    return cocked, s.score


def test_tilted_slow_release_is_cocked():
    # the reviewer's bypass: relax the ring to within 1 um of the groove bottom tilted 6 degrees, then let go
    for gap in (0.0367890625, 0.0368, 0.0369, 0.0365):
        cocked, score = _cocked_slow_release(gap)
        assert cocked and score == 0.0, (gap, cocked, score)


def test_history_resets():
    full()
    s = Sim().step(5)  # a new trial
    assert s.score == 0.0 and rk.turned_deg(None, KEY) == 0 and not rk.overspread(None, KEY)
    assert T._H.home is not None and np.allclose(T._H.home["right_gripper"], R_HOME)


def test_thresholds_match_ring():
    assert T.RING.limit_id == T.LIMIT_ID and T.LIMIT_ID > T.SHAFT_D + T.CLEAR
    assert T.RING.free_id < T.GROOVE_D < T.SHAFT_D
    # the demo's squeeze opens the ring between "clears the shaft" and the elastic limit
    d = T.FREE_ID + (T.HANDLE_GAP - (2 * abs(T.F_SPREAD))) / math.pi
    assert T.SHAFT_D + T.CLEAR < d + 0.0011 and d < T.LIMIT_ID


def test_one_step_twist_pop_is_not_scrap():
    """A single control step with the tips 12 degrees apart (a PhysX pop of the pliers) does not latch the twist."""
    s = pick(Sim())
    s.move((EX, EY, EZ + 0.02))
    s.move((EX, EY, EZ - 0.0025))
    o = s.objects()
    o["pliers/jaw"] = st(o["pliers/jaw"].pos + np.array([0, 0, 0.0034]))
    rk.update(o)
    T.success(o), T.score(o)
    assert rk.max_twist_deg(None, KEY) > T.TWIST_LIMIT and not T.TRACE.ever("twisted")
    s.move((EX, EY, EZ), steps=10, gap=SPREAD)
    s.move((EX, EY, SZ), steps=40)
    s.move((EX, EY, SZ), steps=10, gap=HOLD)
    orbit(s, 15)
    orbit(s, -15)
    orbit(s, 0)
    put_down(s)
    assert s.score == 1.0, s.score


def test_twist_latch_resets_between_trials():
    test_twisted_ring_is_scrap()
    s = Sim().step(3)
    assert not T.TRACE.ever("twisted") and s.score == 0.0
    assert full().score == 1.0


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print("all pass")
