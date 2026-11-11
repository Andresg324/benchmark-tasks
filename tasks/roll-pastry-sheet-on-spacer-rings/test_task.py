"""Offline checks of the pastry-rolling model and scorer: step-by-step pin and gripper histories at 20 Hz, no simulator.

    python test_task.py        (with the yam_arena package, or the offline stub, importable)
"""

import importlib.util
import math
import os

import numpy as np
from yam_arena.task_file import ObjectState

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("pastry_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit, dk = m.kit, m.dk

PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
GRIPPED = 0.27  # finger opening on the 2.5 cm handle


def st(pos, quat=(0, 0, 0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


class Trial:
    """A fresh trial: the pin on the board behind the pastry, arms at PARK. step() runs one control step as the hook
    does: the kit's update, then success() and score()."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.pin = np.array(m.PIN_START, float)
        self.quat = (0, 0, 0, 1.0)
        self.g = dict(PARK)
        self.open = {g: 1.0 for g in PARK}
        self.last = None
        self.step()

    def objects(self):
        objs = {"board": st((*m.BOARD, m.TOP)), "pin": st(self.pin, self.quat)}
        objs.update({g: st(p, (1.0, 0.0, 0.0, 0.0)) for g, p in self.g.items()})  # pointing down
        return objs

    def step(self, n=1):
        for _ in range(n):
            kit._OPENING.update(self.open)
            objs = self.objects()
            dk.update(objs)
            ok = m.success(objs)
            self.last = (m.score(objs), ok)
        return self.last

    def put(self, x, lift=0.0, tilt=0.0, hands=("left_gripper", "right_gripper")):
        """The pin's barrel line at x, its middle `lift` above resting on its rings, tilted `tilt` degrees about x
        (positive lifts the -y end ... about its middle), held by `hands`."""
        self.pin = np.array([x, 0.0, m.AXIS_Z + lift])
        self.quat = qx(tilt)
        a = dk._rotm(self.quat) @ np.array([0.0, 1.0, 0.0])
        for g, s in (("left_gripper", 1), ("right_gripper", -1)):
            if g in hands:
                self.g[g] = tuple(self.pin + s * m.GRIP_Y * a)
                self.open[g] = GRIPPED

    def roll(self, x1, speed=m.SPEED, **kw):
        x0 = float(self.pin[0])
        n = max(1, int(round(abs(x1 - x0) / (speed * 0.05))))
        for k in range(1, n + 1):
            self.put(x0 + (x1 - x0) * k / n, **kw)
            self.step()

    def lift_to(self, x):
        self.put(float(self.pin[0]), lift=m.LIFT)
        self.step(5)
        self.put(x, lift=m.LIFT)
        self.step(10)
        self.put(x)
        self.step(5)

    def let_go(self, home=True):
        for g in self.g:
            self.open[g] = 1.0
        self.step(3)
        for g in self.g:
            p = self.g[g]
            self.g[g] = (p[0], p[1], p[2] + 0.08)
        self.step(3)
        if home:
            self.g = dict(PARK)
        return self.step(25)

    def strokes(self, ends, **kw):
        for e in ends:
            self.roll(e, **kw)


def demo_rolling(t, **kw):
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES, **kw)


# -- the model --------------------------------------------------------------------------------------------------------

def test_volume_conserved():
    t = Trial()
    v0 = dk.volume()
    assert abs(v0 - float(np.prod(m.PASTRY_SIZE))) < 1e-9, v0
    demo_rolling(t)
    t.roll(0.45)  # and off the end, sealing it
    assert abs(dk.volume() - v0) < 1e-12, (dk.volume(), v0)


def test_level_pin_even_sheet():
    t = Trial()
    demo_rolling(t)
    on, length, thin, sealed = m.sheet()
    assert on >= m.SPEC_SHARE and length >= m.MIN_LEN and sealed == 0 and thin >= 0.0029, (on, length, thin, sealed)
    tt, _ = dk.grid()
    rows = [r[r > 0] for r in tt]
    assert max(abs(r.mean() - rows[0].mean()) for r in rows) < 1e-6  # every strip alike


def test_tilted_pin_wedge():
    t = Trial()
    t.lift_to(m.CENTER_X)
    # one end held 4 mm high over the 26 cm barrel: about 0.9 degrees
    tilt = math.degrees(math.atan2(0.004, 2 * m.RING_OUT))
    t.strokes(m.STROKES, lift=0.002, tilt=tilt)
    tt, _ = dk.grid()
    lo_side = tt[0][tt[0] > 0].mean()  # -y strip
    hi_side = tt[-1][tt[-1] > 0].mean()
    assert hi_side - lo_side > 0.002 or lo_side - hi_side > 0.002, (lo_side, hi_side)


def test_edge_over_roll_thins_and_seals():
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.roll(0.44)  # straight off the far end
    tt, sealed = dk.grid()
    assert sealed.any() and tt[tt > 0].min() < 0.002, tt[tt > 0].min()


def test_lifted_pin_does_nothing():
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.put(m.CENTER_X, lift=0.02)
    t.step()
    for e in m.STROKES:
        t.roll(e, lift=0.02)
    assert abs(dk.lengths().min() - m.PASTRY_SIZE[0]) < 1e-9


# -- the scorer -------------------------------------------------------------------------------------------------------

def test_do_nothing_zero():
    t = Trial()
    assert t.step(40)[0] == 0.0


def test_demo_full_score():
    t = Trial()
    demo_rolling(t)
    s, ok = t.let_go()
    assert s == 1.0 and ok, s


def test_not_home_09():
    t = Trial()
    demo_rolling(t)
    assert t.let_go(home=False)[0] == 0.9


def test_still_held_zero():
    t = Trial()
    demo_rolling(t)
    t.g = {g: p for g, p in t.g.items()}
    assert t.step(25)[0] == 0.0  # fingers closed on the handles
    for g in t.g:
        t.open[g] = 1.0
    assert t.step(25)[0] == 0.0  # open hands still around the handles
    t.g["left_gripper"] = PARK["left_gripper"]
    t.g["right_gripper"] = (m.CENTER_X, -0.10, m.AXIS_Z + 0.02)  # a hand resting on the barrel, not the handle
    t.open["right_gripper"] = 0.0
    assert t.step(25)[0] == 0.0


def test_pin_held_up_zero():
    t = Trial()
    demo_rolling(t)
    t.put(m.CENTER_X, lift=0.03)
    t.step()
    t.pin = t.pin.copy()
    t.g = dict(PARK)
    for g in t.g:
        t.open[g] = 1.0
    assert t.step(25)[0] == 0.0  # "let go" with the pin hanging in the air (not resting on the board)


def test_one_pass_zero():
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:4])
    assert dk.lengths().min() < m.MIN_LEN
    assert t.let_go()[0] == 0.0


def test_one_arm_tilt_zero():
    """One hand on one handle, the other end of the pin 4 mm up: the high side stays thick and short."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    tilt = math.degrees(math.atan2(0.004, 2 * m.RING_OUT))
    for e in m.STROKES + (0.39, 0.30, 0.205, 0.30):  # extra passes
        t.roll(e, lift=0.002, tilt=tilt, hands=("left_gripper",))
    t.put(m.CENTER_X)
    t.step()
    assert dk.lengths().min() < m.MIN_LEN
    assert t.let_go()[0] == 0.0


def test_three_passes_half():
    """Three of the four passes: 19.8 cm and 85 % on spec, 0.5."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:12])
    on, length, _, sealed = m.sheet()
    assert m.PART_LEN <= length < m.MIN_LEN and sealed == 0, (on, length, sealed)
    assert t.let_go()[0] == 0.5


def test_roll_over_edges_zero():
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes((0.43, 0.17, 0.43, 0.17, 0.30))
    assert t.let_go()[0] == 0.0


def test_flick_past_end_latches():
    t = Trial()
    demo_rolling(t)
    end = dk.ends()[:, 1].max()
    t.roll(end + 0.012, speed=0.1)  # a quick 1.2 cm overshoot (0.12 s) ...
    t.roll(m.CENTER_X, speed=0.1)  # ... straight back
    assert m.sheet()[3] > 0
    assert t.let_go()[0] == 0.0


def test_one_step_pop_past_end_not_sealed():
    """A single-step physics pop of the pin 7 mm past the end and straight back: debounced, not sealed."""
    t = Trial()
    demo_rolling(t)
    end = dk.ends()[:, 1].max()
    t.roll(end + 0.007, speed=0.5)
    t.roll(end - 0.03, speed=0.5)
    assert m.sheet()[3] == 0
    t.roll(m.CENTER_X)
    assert t.let_go()[0] == 1.0


def test_seal_confirmed_when_grip_opens_next_step():
    """Roll past the end, then open one hand the very next step (still past the end): the seal still latches."""
    t = Trial()
    demo_rolling(t)
    end = dk.ends()[:, 1].max()
    t.roll(end + 0.012, speed=0.1)
    t.put(end + 0.014)
    t.open["left_gripper"] = 1.0
    t.step()
    t.roll(m.CENTER_X)
    assert m.sheet()[3] > 0
    assert t.let_go()[0] == 0.0


def test_one_arm_center_grip_zero():
    """QC attack: one gripper closed on the barrel's middle, the pin level by symmetry: nothing counts, torn."""
    t = Trial()

    def put1(x, lift=0.0, tilt=0.0, hands=None):
        t.pin = np.array([x, 0.0, m.AXIS_Z + lift])
        t.quat = qx(tilt)
        t.g["left_gripper"] = tuple(t.pin)
        t.open["left_gripper"] = 0.53

    t.put = put1
    demo_rolling(t)
    assert dk.torn() and abs(dk.lengths().min() - m.PASTRY_SIZE[0]) < 1e-9
    t.pin = np.array(m.PIN_START, float)
    assert t.let_go()[0] == 0.0


def test_one_handle_one_barrel_zero():
    """Right hand on its handle, left hand closed on the barrel near its ring: no presses count, and it tears."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    base = t.put

    def put(x, lift=0.0, tilt=0.0, hands=None):
        base(x, lift, tilt, hands=("right_gripper",))
        t.g["left_gripper"] = (x, 0.10, m.AXIS_Z + lift)  # closed on the barrel, inboard of the left ring
        t.open["left_gripper"] = 0.53

    t.put = put
    for e in m.STROKES:
        t.roll(e)
    assert dk.torn() and dk.lengths().min() < m.PART_LEN
    assert t.let_go()[0] == 0.0


def test_open_hands_pushing_handles_no_press():
    """Both hands open around the handles, pushing the pin along: not held, nothing presses."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    for e in m.STROKES:
        x0 = float(t.pin[0])
        n = max(1, int(round(abs(e - x0) / (m.SPEED * 0.05))))
        for k in range(1, n + 1):
            t.put(x0 + (e - x0) * k / n)
            for g in t.open:
                t.open[g] = 1.0
            t.step()
    assert abs(dk.lengths().min() - m.PASTRY_SIZE[0]) < 1e-9 and not dk.torn()


def test_fast_correct_rolling_full():
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES, speed=0.4)  # 2 cm per control step
    assert t.let_go()[0] == 1.0


def test_pin_set_down_off_center_ok():
    t = Trial()
    demo_rolling(t)
    t.lift_to(m.PIN_START[0])  # back where it started (it lifts clear of the pastry)
    assert m.sheet()[3] == 0
    assert t.let_go()[0] == 1.0


def test_pin_back_in_rest_full():
    t = Trial()
    demo_rolling(t)
    t.put(m.CENTER_X, lift=m.LIFT)
    t.step(5)
    t.put(m.REST_X, lift=m.LIFT)
    t.step(10)
    t.put(m.REST_X, lift=m.REST_AXIS - m.AXIS_Z)
    t.step(5)
    assert m.sheet()[3] == 0
    assert t.let_go()[0] == 1.0


def test_numbers_in_text():
    doc = m.score.__doc__
    for s in ("20 cm", "18 cm", "90 %", "3 mm", "0.5 mm", "3 degrees", "5 mm", "1 cm"):
        assert s in doc, s
    assert m.MIN_LEN == 0.20 and m.SPEC_SHARE == 0.90 and m.TOL_T == 0.0005 and m.LEVEL_DEG == 3.0
    assert m.PIN_DOWN == 0.005 and m.CLEAR == 0.01 and dk.OVER == 0.005 and dk.EDGE == 0.01 and m.TOUCH == 0.01
    assert "two steps in a row" in doc and "both handles held" in doc


if __name__ == "__main__":
    for name, f in list(globals().items()):
        if name.startswith("test_"):
            f()
            print("ok", name)
    print("all pass")
