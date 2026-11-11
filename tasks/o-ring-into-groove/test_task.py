"""Offline tests of the O-ring task's scorer (no simulator). Run: /data/IsaacLab-Arena/.venv/bin/python test_task.py

The ring model runs as in the sim (ringkit.update each step); the grippers and the holder are hand-made states.
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
FULL = 2 * abs(FINGER_OPEN)
END = T.TOP + T.END_H
HEXZ = T.TOP + T.HEX_TOP


def down_quat(yaw=0.0, tilt=0.0, about="x"):
    """A straight-down grip (fingers closing along world y at yaw 0), tilted `tilt` deg about world x (the line
    between the fingertips tilts: one tip higher than the other) or world y (about that line: both tips level)."""
    q = (0.70711, 0.70711, 0.0, 0.0)
    q = kit._quat_mul((0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)), q)
    h = math.radians(tilt) / 2
    ax = (math.sin(h), 0.0, 0.0) if about == "x" else (0.0, math.sin(h), 0.0)
    return kit._quat_mul((*ax, math.cos(h)), q)


def st(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p,
                       high=p)


class Sim:
    """A trial: the right gripper's tool point at `tcp`, its pads `gap` apart, pointing straight down (or tilted)."""

    def __init__(self, about="x"):
        kit._EPISODE["count"] += 1
        self.tcp = R_HOME.copy()
        self.gap = FULL
        self.tilt = 0.0
        self.about = about
        self.scores = []

    def objects(self):
        kit._OPENING["right_gripper"] = self.gap / FULL
        kit._OPENING["left_gripper"] = 1.0
        return {"holder": st((T.SX, T.SY, T.TOP)), "right_gripper": st(self.tcp, down_quat(0.0, self.tilt, self.about)),
                "left_gripper": st(L_HOME, down_quat())}

    def step(self, n=1):
        for _ in range(n):
            o = self.objects()
            rk.update(o)
            T.success(o)
            self.scores.append(T.score(o))
        return self

    def move(self, to, steps=20, gap=None, tilt=None):
        a, b = self.tcp.copy(), np.asarray(to, float)
        g0, g1 = self.gap, (self.gap if gap is None else gap)
        t0, t1 = self.tilt, (self.tilt if tilt is None else tilt)
        for k in range(1, steps + 1):
            f = k / steps
            self.tcp, self.gap, self.tilt = a + f * (b - a), g0 + f * (g1 - g0), t0 + f * (t1 - t0)
            self.step()
        return self

    def tips_at(self, z, xy=(T.SX, T.SY)):
        return np.array([xy[0], xy[1], z + T.TIP])

    @property
    def o(self):
        return self.objects()

    @property
    def last(self):
        return self.scores[-1]


def into_ring(s):
    s.move(s.tips_at(END + 0.06), gap=0.008)
    s.move(s.tips_at(END + 0.001))
    assert rk.engaged(s.o, KEY), "fingertips in the ring take hold"
    return s


def demo_like(gap=0.057, home=True, tilt_out=0.0, about="x", tilt_steps=10):
    s = into_ring(Sim(about))
    s.move(s.tips_at(END + 0.001), gap=gap, steps=30)
    s.move(s.tips_at(HEXZ + 0.0015), steps=40)
    if tilt_out:
        s.move(s.tcp, tilt=tilt_out, steps=tilt_steps)
    s.move(s.tips_at(END + 0.02), steps=30)
    s.move(s.tips_at(END + 0.08), steps=10)
    if home:
        s.move(R_HOME, steps=30)
        s.step(10)
    return s


def test_do_nothing():
    s = Sim().step(60)
    assert max(s.scores) == 0.0
    assert not rk.seated(s.o, KEY) and rk.groove(s.o, KEY) is None


def test_demo_full():
    s = demo_like()
    o = s.o
    assert rk.seated(o, KEY) and not rk.engaged(o, KEY) and not rk.cocked(o, KEY)
    assert abs(rk.position(o, KEY) - T.GROOVE_AT) < 1e-9  # cross-check: the ring's position is the groove's middle
    assert abs(rk.inner_d(o, KEY) - T.GROOVE_D) < 1e-9
    assert abs(rk.max_d(o, KEY) / T.FREE_ID - 1.287) < 0.002, rk.max_d(o, KEY)  # stretched 28.7%, under 50%
    assert rk.twist_deg(o, KEY) == 0.0
    assert s.last == 1.0, s.last


def test_not_home():
    s = demo_like(home=False)
    s.step(5)
    assert s.last == 0.9, s.last


def test_still_held_at_groove():
    s = into_ring(Sim())
    s.move(s.tips_at(END + 0.001), gap=0.057, steps=30)
    s.move(s.tips_at(HEXZ + 0.0015), steps=40)
    s.step(20)
    assert rk.engaged(s.o, KEY) and not rk.seated(s.o, KEY)
    assert max(s.scores) == 0.0, "no credit while the fingers still hold it open"


def test_overstretch_latched():
    s = into_ring(Sim())
    s.move(s.tips_at(END + 0.001), gap=FULL, steps=30)  # opened all the way: 78% stretch
    assert rk.overspread(s.o, KEY)
    s.move(s.tips_at(END + 0.001), gap=0.057, steps=20)  # back to a sane stretch, then a perfect fit
    s.move(s.tips_at(HEXZ + 0.0015), steps=40)
    s.move(s.tips_at(END + 0.08), steps=40)
    s.move(R_HOME, steps=30)
    s.step(10)
    assert not rk.seated(s.o, KEY) and max(s.scores) == 0.0, "an over-stretch is never forgiven (it sits loose)"


def test_just_under_limit():
    s = demo_like(gap=0.073)  # 49%
    assert not rk.overspread(s.o, KEY) and s.last == 1.0, (rk.max_d(s.o, KEY), s.last)


def test_too_little_stretch_cannot_pass():
    s = into_ring(Sim())
    s.move(s.tips_at(END + 0.001), gap=0.030, steps=20)  # fingers still over the end face: the ring stays loose
    assert rk.inner_d(s.o, KEY) == T.FREE_ID
    s.move(s.tips_at(END + 0.06), steps=20)
    assert not rk.engaged(s.o, KEY) and abs(rk.position(s.o, KEY) + T.RING_CS / 2) < 1e-9


def test_rolled_on_with_a_fingertip():
    for steps in (60, 5):  # slowly, or fast (4 mm a step)
        s = Sim()
        xy = (T.SX, T.SY - 0.0245)  # fingers closed, their tips on the ring's top beside the thread
        s.move(s.tips_at(END + 0.03, xy), gap=0.0, steps=20)
        s.move(s.tips_at(END + 0.0025, xy), steps=10)
        s.move(s.tips_at(HEXZ + 0.0005, xy), steps=steps)  # pushed down the thread: it rolls
        assert rk.twist_deg(s.o, KEY) > 300, rk.twist_deg(s.o, KEY)
        s.move(s.tips_at(END + 0.06, xy), steps=20)
        s.move(R_HOME, steps=30)
        s.step(10)
        assert rk.groove(s.o, KEY) == 0 and rk.seated(s.o, KEY), "it does end up in the groove"
        assert max(s.scores) == 0.0, "but twisted: 0"


def test_rolled_part_way():
    s = Sim()
    xy = (T.SX, T.SY - 0.0245)
    s.move(s.tips_at(END + 0.0025, xy), gap=0.0, steps=20)
    s.move(s.tips_at(END - 0.004, xy), steps=10)  # rolled 4 mm onto the thread
    s.move(s.tips_at(END + 0.03, xy), steps=10)
    s.move(R_HOME, steps=30)
    s.step(10)
    assert rk.twist_deg(s.o, KEY) > 100 and not rk.seated(s.o, KEY)
    assert max(s.scores) == 0.0


def test_tilted_release_cocked():
    for tilt, steps in ((5.0, 10), (8.0, 10), (12.0, 10), (20.0, 10), (8.0, 1), (8.0, 40)):
        s = demo_like(tilt_out=tilt, tilt_steps=steps)  # one fingertip lifted: it leaves the ring tilted
        assert rk.cocked(s.o, KEY) and not rk.seated(s.o, KEY), (tilt, steps, rk.max_tilt_deg(s.o, KEY))
        assert max(s.scores) == 0.0, (tilt, steps)


def test_small_tilt_ok():
    s = demo_like(tilt_out=2.0)
    assert rk.seated(s.o, KEY) and s.last == 1.0
    assert abs(rk.max_tilt_deg(s.o, KEY) - 2.0) < 0.05, "tilt is the fingertips' height difference over their gap"


def test_gripper_angle_alone_does_not_tilt_it():
    s = demo_like(tilt_out=8.0, about="y")  # tilted about the line between the tips: both still at one height
    assert rk.max_tilt_deg(s.o, KEY) < 0.5 and rk.seated(s.o, KEY) and s.last == 1.0


def test_poking_a_seated_ring_does_not_roll_it():
    s = demo_like(home=False)
    assert rk.seated(s.o, KEY) and rk.twist_deg(s.o, KEY) == 0.0
    xy = (T.SX, T.SY - 0.0245)  # closed fingertips down beside the thread, onto and into the seated ring
    s.move(s.tips_at(END + 0.01, xy), gap=0.0, steps=20)
    for z in (HEXZ + 0.003, HEXZ + 0.0015, HEXZ + 0.0003):
        s.move(s.tips_at(z, xy), steps=10)
        s.step(20)
        assert rk.max_roll_deg(s.o, KEY) == 0.0, (z, rk.max_roll_deg(s.o, KEY))
    s.move(s.tips_at(END + 0.06, xy), steps=20)
    s.move(R_HOME, steps=30)
    s.step(10)
    assert rk.seated(s.o, KEY) and rk.max_roll_deg(s.o, KEY) == 0.0 and s.last == 1.0


def test_tip_beside_the_section_does_not_push():
    s = Sim()
    xy = (T.SX, T.SY - 0.0245)
    s.move(s.tips_at(END + 0.0025, (T.SX, T.SY - 0.06)), gap=0.0, steps=20)  # level with the ring, off to one side
    s.move(s.tips_at(END + 0.0025, xy), steps=10)  # slid in sideways into its section: not on its face
    s.move(s.tips_at(END + 0.0005, xy), steps=10)
    assert abs(rk.position(s.o, KEY) + T.RING_CS / 2) < 1e-9 and rk.max_roll_deg(s.o, KEY) == 0.0


def test_let_go_on_thread():
    s = into_ring(Sim())
    s.move(s.tips_at(END + 0.001), gap=0.057, steps=30)
    s.move(s.tips_at(END - 0.010), steps=20)  # halfway down the thread
    s.move(s.tips_at(END + 0.06), steps=20)
    s.move(R_HOME, steps=30)
    s.step(10)
    o = s.o
    assert not rk.engaged(o, KEY) and rk.groove(o, KEY) is None and not rk.seated(o, KEY)
    assert abs(rk.inner_d(o, KEY) - T.THREAD_D) < 1e-9, "it grips the thread"
    assert max(s.scores) == 0.0


def test_hover_and_touch():
    s = Sim()
    s.move(s.tips_at(END + 0.005), gap=0.008, steps=20)  # hovering just above the ring
    s.move(s.tips_at(END + 0.005, (T.SX + 0.04, T.SY)), steps=20)  # beside the plug
    s.move(s.tips_at(HEXZ + 0.002, (T.SX + 0.04, T.SY)), gap=FULL, steps=20)  # down beside it, open
    s.move(R_HOME, steps=20)
    s.step(10)
    assert max(s.scores) == 0.0 and not rk.overspread(s.o, KEY)


def test_episode_resets_history():
    s = demo_like()
    assert s.last == 1.0
    s2 = Sim().step(5)  # a scene reset: the ring is back on the end face, history fresh
    assert not rk.seated(s2.o, KEY) and max(s2.scores) == 0.0
    assert T._H.episode == kit.episode()


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
            print("ok", name)
    print("all pass")
