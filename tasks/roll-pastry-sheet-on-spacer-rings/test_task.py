"""Offline checks of the dough-rolling model and scorer: step-by-step pin and gripper histories at 20 Hz, no simulator.

    python test_task.py        (with the yam_arena package, or the offline stub, importable)
"""

import importlib.util
import math
import os

import numpy as np
from yam_arena.task_file import ObjectState

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("dough_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit, dk = m.kit, m.dk

PARK = {"left_gripper": (0.20, 0.30, m.TOP + 0.20), "right_gripper": (0.20, -0.30, m.TOP + 0.20)}
GRIPPED = 0.26  # finger opening closed on the 2.5 cm handle (the pads pressed 0.3 mm into it, as the grip does)


def st(pos, quat=(0, 0, 0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


class Trial:
    """A fresh trial: the pin lying on the board behind the dough, arms at PARK. step() runs one control step as the hook
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


def demo_rolling(t, put_away=True, **kw):
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES, **kw)
    if put_away:
        t.lift_to(m.PIN_X)  # back where it lay, off the dough


# -- the model --------------------------------------------------------------------------------------------------------

def test_start_matches_product():
    """The start sheet (rolled between 3/16 inch guide strips, trimmed): 11.5 x 24 cm at 4.8 mm, 132.5 mL; on the 4 cm
    grid of points it covers 18 (24 once rolled)."""
    t = Trial()
    assert m.DOUGH_SIZE == (0.115, 0.24, 0.0048)
    assert abs(dk.volume() - 0.115 * 0.24 * 0.0048) < 1e-12
    pts = m.points()
    assert pts.shape[0] == 6 and (pts > 0).sum() == 18 and abs(pts[pts > 0] - 0.0048).max() < 1e-12
    assert m.sheet()[0] == 0.0 and m.on_board() and m.pin_down(t.objects())


def test_volume_conserved():
    t = Trial()
    v0 = dk.volume()
    demo_rolling(t)
    t.roll(0.37)  # and off the end
    assert abs(dk.volume() - v0) < 1e-12, (dk.volume(), v0)


def test_level_pin_even_sheet():
    t = Trial()
    demo_rolling(t)
    on, half, quarter, thin = m.sheet()
    assert on >= m.SPEC_SHARE and thin >= m.TARGET_T - 1e-9, (on, thin)
    tt = dk.grid()
    rows = [r[r > 0] for r in tt]
    assert max(abs(r.mean() - rows[0].mean()) for r in rows) < 1e-6  # every band alike
    e = dk.ends()
    assert e[:, 0].min() > m.X_RANGE[0] + 0.04 and e[:, 1].max() < 0.33  # the sheet stays well inside the board


def test_tilted_pin_wedge():
    t = Trial()
    t.lift_to(m.CENTER_X)
    # one end held 4 mm high over the 30.5 cm barrel: about 0.75 degrees
    tilt = math.degrees(math.atan2(0.004, 2 * m.RING_OUT))
    t.strokes(m.STROKES, lift=0.002, tilt=tilt)
    tt = dk.grid()
    lo_side = tt[0][tt[0] > 0].mean()  # -y band
    hi_side = tt[-1][tt[-1] > 0].mean()
    assert hi_side > 0.0047 and hi_side - lo_side > 0.0008, (lo_side, hi_side)  # the high side never pressed
    assert t.let_go()[0] == 0.0


def test_edge_over_roll_not_sealed():
    """Review, Nov 12: rolling past an end presses that end like any other slice; the rings keep the barrel 3.2 mm up,
    so nothing goes below the gap."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.roll(0.37)  # straight off the far end
    tt = dk.grid()
    assert tt[tt > 0].min() >= m.TARGET_T - 1e-9, tt[tt > 0].min()


def test_lifted_pin_does_nothing():
    t = Trial()
    t.lift_to(m.CENTER_X)  # set down at the middle: that press dents it
    t.put(m.CENTER_X, lift=0.02)
    t.step()
    before = dk.grid().copy()
    for e in m.STROKES:
        t.roll(e, lift=0.02)
    assert np.array_equal(dk.grid(), before)


# -- the scorer -------------------------------------------------------------------------------------------------------

def test_do_nothing_zero():
    t = Trial()
    assert t.step(40)[0] == 0.0


def test_demo_full_score():
    t = Trial()
    demo_rolling(t)
    s, ok = t.let_go()
    assert s == 1.0 and ok, s


def test_demo_with_lag_at_turns_full():
    """The sim's pin trails the hands by about 1 cm at each turn: the demo's strokes still finish the sheet with the
    pin stopping 2 cm short of each commanded end."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    for e in m.STROKES:
        x0 = float(t.pin[0])
        t.roll(e - 0.02 if e > x0 else e + 0.02)
    t.lift_to(m.PIN_X)
    assert t.let_go()[0] == 1.0


def test_not_home_full():
    t = Trial()
    demo_rolling(t)
    assert t.let_go(home=False)[0] == 1.0  # where the arms end is not scored
    assert not hasattr(m, "REQUIRE_ARMS_HOME") and not hasattr(m, "HOME")


def test_still_held_capped():
    t = Trial()
    demo_rolling(t)
    assert t.step(25)[0] == 0.5  # fingers closed on the handles: the sheet is done, the pin not let go
    for g in t.g:
        t.open[g] = 1.0
    assert t.step(25)[0] == 1.0  # opened wide around the handles, touching nothing: let go
    t.g["left_gripper"] = PARK["left_gripper"]
    t.g["right_gripper"] = (m.PIN_X, -0.10, m.AXIS_Z + 0.02)  # a hand resting on the barrel, not the handle
    t.open["right_gripper"] = 0.0
    assert t.step(25)[0] == 0.5


def test_release_is_contact_not_nearness():
    """Let go is judged by contact (the shapes overlapping), not a distance zone: offline the gripper boxes."""
    t = Trial()
    demo_rolling(t)
    t.lift_to(m.PIN_X)
    t.g = dict(PARK)
    for g in t.g:
        t.open[g] = 0.0
    top = m.PIN_X, m.AXIS_Z + m.R_BARREL + 0.0128  # closed fingertips (12.8 mm past the tool point) just on the top
    t.g["right_gripper"] = (top[0], -0.05, top[1] + 0.005)
    assert t.step(25)[0] == 1.0  # closed fingertips 5 mm over the barrel, not touching
    t.g["right_gripper"] = (top[0], -0.05, top[1] + 0.0003)
    assert t.step(25)[0] == 1.0  # 0.3 mm over it: a near miss, not contact
    t.g["right_gripper"] = (top[0], -0.05, top[1] - 0.0002)
    assert t.step(25)[0] == 0.5  # pressed 0.2 mm into it: a hand on the pin
    t.g["right_gripper"] = (top[0], -(m.HANDLE_IN + m.HANDLE_OUT) / 2, m.AXIS_Z + m.R_HANDLE + 0.0128 - 0.0002)
    assert t.step(25)[0] == 0.5  # fingertips resting on the right handle


def test_release_on_real_meshes():
    """In the sim, let go uses the arms' real collision meshes (yam_links.npz, posed from the simulator's link poses):
    a real fingertip 1.0 or 0.3 mm over the barrel has let go; one 0.2 mm into it has not; a wrist (link5) resting on
    the handle has not either."""
    t = Trial()
    demo_rolling(t)
    t.lift_to(m.PIN_X)
    t.let_go()
    pin = np.array(t.pin)
    pts, _, _ = m.mesh("tip_left")
    low = pts[np.argmin(pts[:, 2])]
    try:
        for gap, free in ((0.001, True), (0.0003, True), (-0.0002, False)):
            m.LINK_POSES.clear()
            m.LINK_POSES["right_robot/tip_left"] = (pin + np.array([0, -0.05, m.R_BARREL + gap]) - low, np.eye(3))
            assert m.released(t.objects()) == free, gap
            assert t.step(3)[0] == (1.0 if free else 0.5), gap
        pts, _, _ = m.mesh("link5")
        low = pts[np.argmin(pts[:, 2])]
        m.LINK_POSES.clear()
        m.LINK_POSES["left_robot/link5"] = (pin + np.array([0, 0.2, m.R_HANDLE - 0.0002]) - low, np.eye(3))
        assert not m.released(t.objects())
    finally:
        m.LINK_POSES.clear()

def test_pin_held_up_capped():
    t = Trial()
    demo_rolling(t)
    t.put(m.CENTER_X, lift=0.03)
    t.step()
    t.pin = t.pin.copy()
    t.g = dict(PARK)
    for g in t.g:
        t.open[g] = 1.0
    assert t.step(25)[0] == 0.5  # "let go" with the pin hanging in the air (not resting on the board)


def test_out_and_back_once_zero():
    """One stroke out to the far end and back: the near half is still 4.8 mm, 0."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:2])
    assert m.sheet()[2] < m.SPEC_SHARE
    assert t.let_go()[0] == 0.0


def test_one_pass_quarter():
    """One level pass (out and back to each end): 3.6 to 4.0 mm everywhere and even, 0.25 (about two passes to go)."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:4])
    on, half, quarter, _ = m.sheet()
    assert on < m.SPEC_SHARE and half < m.SPEC_SHARE <= quarter and m.wedge() < 1e-9, (on, half, quarter)
    assert t.let_go()[0] == 0.25


def test_slight_wedge_half():
    """One end of the pin kept 0.3 mm up through ten passes: at most 3.7 mm everywhere and the bands within 0.3 mm
    (0.23 mm), but the high side settles at 3.5 to 3.7 mm (half the points off spec): 0.5 (nearly done)."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    tilt = math.degrees(math.atan2(0.0003, 2 * m.RING_OUT))
    t.strokes(m.STROKES * 2, lift=0.00015, tilt=tilt)
    t.lift_to(m.PIN_X)
    on, half, _, _ = m.sheet()
    assert half >= m.SPEC_SHARE and on < m.SPEC_SHARE and 0.0001 < m.wedge() <= m.WEDGE_MAX, (on, half, m.wedge())
    assert t.let_go()[0] == 0.5


def test_uniform_3_6_mm_half():
    """Scorer check: a level sheet 3.6 mm everywhere (0.4 mm over, about one pass to go): 0.5."""
    t = Trial()
    demo_rolling(t)
    dk._state().t[:] = 0.0036
    assert t.let_go()[0] == 0.5


def test_two_passes_half_three_full():
    """Two level passes leave the ends at 3.5 to 3.6 mm (a quarter of the points off spec): 0.5; three reach 3.4 to
    3.5 mm everywhere, inside 2.9 to 3.5 mm: 1.0 (the demo makes five)."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:8])
    t.lift_to(m.PIN_X)
    assert m.sheet()[0] < m.SPEC_SHARE and t.let_go()[0] == 0.5
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:12])
    t.lift_to(m.PIN_X)
    assert m.sheet()[0] >= m.SPEC_SHARE and t.let_go()[0] == 1.0


def test_one_arm_tilt_zero():
    """One hand on one handle, the other end of the pin 4 mm up: the high side stays thick."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    tilt = math.degrees(math.atan2(0.004, 2 * m.RING_OUT))
    for e in m.STROKES + m.STROKES[:4]:  # extra passes
        t.roll(e, lift=0.002, tilt=tilt, hands=("left_gripper",))
    t.put(m.CENTER_X)
    t.step()
    assert t.let_go()[0] == 0.0


def test_wedge_from_lifted_end_zero():
    """One end of the pin kept 1 mm up through eight passes: the low side on spec, the high side about 3.9 mm (bands
    0.72 mm apart): a wedge, 0."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    tilt = math.degrees(math.atan2(0.001, 2 * m.RING_OUT))
    t.strokes(m.STROKES * 2, lift=0.0005, tilt=tilt)
    assert m.wedge() > m.WEDGE_MAX, m.wedge()
    assert t.let_go()[0] == 0.0


def test_wedge_blocks_full_credit():
    """Scorer check: a sheet all on spec but 0.35 mm thicker on one side than the other (2.95 to 3.30 mm across the
    width) is not even: 0.5, not 1.0; at 0.25 mm it is 1.0."""
    t = Trial()
    demo_rolling(t)
    S = dk._state()
    for j in range(S.t.shape[0]):
        S.t[j][:] = 0.00295 + 0.00035 * j / (S.t.shape[0] - 1)
    on, half, _, _ = m.sheet()
    assert on >= m.SPEC_SHARE and m.wedge() > m.WEDGE_MAX, (on, m.wedge())
    assert t.let_go()[0] == 0.5
    for j in range(S.t.shape[0]):
        S.t[j][:] = 0.00295 + 0.00025 * j / (S.t.shape[0] - 1)  # 0.25 mm: within the 0.3 mm
    assert t.step()[0] == 1.0


def test_thin_or_torn_points_off_spec():
    """Scorer check: points under 2.9 mm (a real sheet dragged thin or torn; the sim cannot make them) are off spec:
    the outer two of the twelve 2 cm bands at 2.5 mm (4 of the 24 points read) take a finished sheet below 1.0."""
    t = Trial()
    demo_rolling(t)
    S = dk._state()
    S.t[:2, :] = 0.0025  # the outer two of the twelve bands
    on, half, _, thin = m.sheet()
    assert on < m.SPEC_SHARE and thin < m.TARGET_T - m.TOL_T, (on, thin)
    assert t.let_go()[0] < 1.0


def test_roll_over_edges_scored_on_the_sheet():
    """Review, Nov 12: full-length strokes over both ends are not an automatic 0; the sheet they leave is scored."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes((0.36, 0.13, 0.36, 0.13, 0.36, 0.13, 0.36, 0.13, 0.24))
    on, half, quarter, thin = m.sheet()
    assert thin >= m.TARGET_T - 1e-9 and m.on_board(), (on, thin)
    t.lift_to(m.PIN_X)
    assert t.let_go()[0] == 1.0


def test_flick_past_end_no_penalty():
    t = Trial()
    demo_rolling(t)
    end = dk.ends()[:, 1].max()
    t.roll(end + 0.012, speed=0.1)  # a quick 1.2 cm overshoot (0.12 s) ...
    t.roll(m.CENTER_X, speed=0.1)  # ... straight back
    t.lift_to(m.PIN_X)
    assert t.let_go()[0] == 1.0


def test_one_hand_rolling_full():
    """Review TK-LAJNZX: one hand may roll. The right hand alone on its handle, the pin kept level: the strokes press,
    1.0. (In the sim the pin presses only as hard as the hand holds it down: the dough's push lifts a pin that is not.)"""
    t = Trial()
    t.lift_to(m.CENTER_X)
    base = t.put

    def put(x, lift=0.0, tilt=0.0, hands=None):
        base(x, lift, tilt, hands=("right_gripper",))
        t.g["left_gripper"] = PARK["left_gripper"]
        t.open["left_gripper"] = 1.0

    t.put = put
    demo_rolling(t)
    t.lift_to(m.PIN_X)
    t.put = base
    assert t.let_go()[0] == 1.0

def test_one_arm_center_grip_counts():
    """One gripper closed on the barrel's middle, the pin level: one-handed rolling counts like any other."""
    t = Trial()

    def put1(x, lift=0.0, tilt=0.0, hands=None):
        t.pin = np.array([x, 0.0, m.AXIS_Z + lift])
        t.quat = qx(tilt)
        t.g["left_gripper"] = tuple(t.pin)
        t.open["left_gripper"] = 0.53

    t.put = put1
    demo_rolling(t)
    t.lift_to(m.PIN_X)
    assert t.let_go()[0] == 1.0


def test_one_handle_one_barrel_counts():
    """Mock review, Nov 12: the instruction does not ask for the handles, so a hand on the barrel counts. Right hand on
    its handle, left hand closed on the barrel near its ring: both hands on the pin, the strokes press, 1.0."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    base = t.put

    def put(x, lift=0.0, tilt=0.0, hands=None):
        base(x, lift, tilt, hands=("right_gripper",))
        t.g["left_gripper"] = (x, 0.12, m.AXIS_Z + lift)  # closed on the barrel, inboard of the left ring
        t.open["left_gripper"] = 0.53

    t.put = put
    for e in m.STROKES:
        t.roll(e)
    t.put = base
    t.lift_to(m.PIN_X)
    assert t.let_go()[0] == 1.0


def test_palms_on_barrel_count():
    """Both hands pressing down on top of the barrel with closed fists (fingertips resting on it, no grip), as a baker
    rolls with the palms: both hands on the pin, the strokes press, 1.0."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    base = t.put
    top = m.R_BARREL + 0.0128 - 0.0002  # closed fingertips pressed on the barrel's top

    def put(x, lift=0.0, tilt=0.0, hands=None):
        base(x, lift, tilt)
        for g, y in (("left_gripper", 0.06), ("right_gripper", -0.06)):
            t.g[g] = (x, y, m.AXIS_Z + lift + top)
            t.open[g] = 0.0

    t.put = put
    for e in m.STROKES:
        t.roll(e)
    assert m.sheet()[0] >= m.SPEC_SHARE
    t.put = base
    t.lift_to(m.PIN_X)
    assert t.let_go()[0] == 1.0


def test_fast_correct_rolling_full():
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES, speed=0.4)  # 2 cm per control step
    t.lift_to(m.PIN_X)
    assert t.let_go()[0] == 1.0


def test_pin_back_where_it_lay_full():
    t = Trial()
    demo_rolling(t)
    t.lift_to(m.PIN_X)  # back where it started (it lifts clear of the dough)
    assert t.let_go()[0] == 1.0


def test_pin_on_the_table_full():
    """Set down on its rings on the table beside the board (the counter): put away."""
    t = Trial()
    demo_rolling(t)
    t.put(m.CENTER_X, lift=m.LIFT)
    t.step(5)
    t.put(0.46, lift=m.LIFT)
    t.step(10)
    t.put(0.46, lift=m.TOP - m.BZ)
    t.step(5)
    assert t.let_go()[0] == 1.0


def test_pin_tilted_on_something_capped():
    """A finished sheet, the pin let go but propped 5 degrees off level (one ring up on something): 0.5."""
    t = Trial()
    demo_rolling(t)
    t.put(m.PIN_X, tilt=5.0)
    assert t.let_go()[0] == 0.5


def test_unrolled_end_caught_by_end_points():
    """Review check, Nov 16: strokes that stop short of the near end (x 19 cm) leave its last 2 to 3 cm at 4.1 mm (one
    press as the pin was first set down) while every row point (every 4 cm along x) is on spec; the end points, 1 cm
    inside each band's rolled ends, read the unrolled end, so the sheet no longer scores 1.0."""
    t = Trial()
    t.lift_to(m.CENTER_X)
    for _ in range(4):
        t.strokes([0.33, 0.24, 0.19, 0.24])
    t.lift_to(m.PIN_X)
    rows = dk.grid()[list(m.ROWS)][:, m.COLS]
    assert (np.abs(rows[rows > 0] - m.TARGET_T) <= m.TOL_T + 1e-9).all()  # the row points alone see a finished sheet
    assert dk.grid().max() > 0.004  # the near end is still far over
    p = m.points()
    assert (p[:, -2] > 0.004).all()  # every band's near end point reads it
    assert t.let_go()[0] < 1.0


def test_dough_pushes_back_within_the_arms():
    """Review, Nov 16: the dough resists the pin. Over the demo's strokes the dough pushes the pin up (about 35 N on
    the first pass over the 4.8 mm sheet) and back against its motion (about 6 N), both inside what the two arms can
    hold from their servo ratings (52 N down and 11 N along the board, continuous, per arm). A pin lifted off the
    dough, or resting behind it, feels nothing."""
    t = Trial()
    assert np.allclose(dk.wrench()[0], 0.0)
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:2])
    fx, _, fz = dk.peak()
    assert 3.0 < fx < 2 * 11.0 and 20.0 < fz < 2 * 52.0, (fx, fz)
    demo_rolling(t)
    t.put(m.CENTER_X, lift=0.02)
    t.step()
    assert np.allclose(dk.wrench()[0], 0.0)


# -- the force gate (Nov 17 red team): the dough yields only under a load past its elastic limit -------------------------

def test_press_law():
    """Elastic up to a 5 % squeeze, then half the excess beyond it springs back; a level pin on its rings settles the
    sheet at 3.2 mm / 0.95 = 3.37 mm, on spec; the contact pressure reaches the flow stress at the yield squeeze."""
    e = dk.YIELD_STRAIN
    assert e == 0.05 and dk.RATIO == 0.5
    assert dk.thinning(0.0048, 0.0048 * e) == 0.0 and dk.thinning(0.0048, 0.0001) == 0.0
    assert abs(dk.thinning(0.0048, 0.0016) - 0.5 * (0.0016 - 0.00024)) < 1e-12
    t = 0.0048
    for _ in range(40):
        t -= dk.thinning(t, t - m.RING_GAP)
    assert abs(t - m.RING_GAP / (1 - e)) < 1e-6 and abs(t - m.TARGET_T) <= m.TOL_T
    assert abs(dk.pressure(0.0048, 0.00012, m.DOUGH_STRESS) - 0.5 * m.DOUGH_STRESS) < 1e-6
    assert dk.pressure(0.0048, 0.0016, m.DOUGH_STRESS) == m.DOUGH_STRESS


def test_stress_from_hand_check():
    """13 kPa sits in the 12.4 to 14.5 kPa the hand check's 29 to 34 N peak gives on the first pass (24 cm wide, 1.6 mm
    off the 4.8 mm sheet); the first pass then pushes the pin up about 30 N."""
    a = math.sqrt(2 * m.R_BARREL * 0.0016 - 0.0016 ** 2)
    lo, hi = 29.0 / (0.24 * a), 34.0 / (0.24 * a)
    assert 12.3e3 < lo < m.DOUGH_STRESS < hi < 14.6e3, (lo, hi)
    t = Trial()
    t.lift_to(m.CENTER_X)
    t.strokes(m.STROKES[:2])
    assert 28.0 < dk.peak()[2] < 33.0, dk.peak()


def test_pin_weight_alone_stays_elastic():
    """The pin's own weight (7.2 N) is under the dough's yield load across the 24 cm sheet (twelve 2 cm strips) at every
    thickness from the start to the rings' gap, so a pin resting on the dough, or rolled with no push, does not thin it."""
    w = m.PIN_MASS * 9.81
    for t in np.linspace(m.RING_GAP, m.DOUGH_SIZE[2], 9):
        assert 12 * dk.yield_load(float(t)) > 1.3 * w, (t, 12 * dk.yield_load(float(t)), w)


def _skim(t, depth, crossings=40):
    """Strokes end to end with the barrel's line held `depth` below the sheet's thickest point (the rings off the
    board), as a light pin that never pushes harder."""
    t.put(0.15, lift=m.LIFT)
    t.step()
    for _ in range(crossings // 2):
        top = m.points()[m.points() > 0].max()
        t.strokes((0.33, 0.15), speed=0.2, lift=max(0.0, top - depth - m.RING_GAP))


def test_light_skims_never_thin():
    """Red team (Nov 17): the pin held 0.1 mm, or 0.2 mm, into the sheet's top, end to end: under the yield
    squeeze (0.24 mm at 4.8 mm), so the dough springs back: after 40 crossings the sheet is exactly as it started,
    and the model has no state that a further crossing could change, so no number of passes thins it; 0."""
    for depth in (0.0001, 0.0002):
        t = Trial()
        _skim(t, depth)
        assert abs(dk.grid().max() - 0.0048) < 1e-12 and abs(dk.grid()[dk.grid() > 0].min() - 0.0048) < 1e-12, depth
        t.lift_to(m.PIN_X)
        assert t.let_go()[0] == 0.0


def test_drop_or_bang_does_not_press():
    """The pin dropped onto the sheet (down on its rings for one control step, then bounced back up), five times at
    each spot: a load held for under two control steps does not press."""
    t = Trial()
    for x in (0.19, 0.22, 0.25, 0.28):
        t.put(x, lift=0.01)
        t.step(3)
        for _ in range(5):
            t.put(x, lift=0.01)
            t.step()
            t.put(x)
            t.step()
            t.put(x, lift=0.01)
            t.step()
    t.put(x, lift=m.LIFT)
    t.step()
    assert abs(dk.grid()[dk.grid() > 0].min() - 0.0048) < 1e-12


def test_press_in_place_thins():
    """Pressing straight down (rings on the board, no rolling) thins the dough under the barrel, as a real press does:
    the slices under the line come down most, those under the barrel's sides less; holding it there does not press
    again, lifting it off and setting it down again does."""
    t = Trial()
    t.lift_to(m.CENTER_X)  # set down at the middle (lift_to holds it there 5 steps)
    S = dk._state()
    j = 6
    e = dk._edges(S, j)
    c = 0.5 * (e[:-1] + e[1:])
    under = np.abs(c - m.CENTER_X) < 0.002
    assert S.t[j][under].max() < 0.0042 and S.t[j][np.abs(c - m.CENTER_X) > 0.012].min() == 0.0048
    once = S.t[j].copy()
    t.step(20)
    assert np.array_equal(S.t[j], once)
    t.put(m.CENTER_X, lift=0.01)
    t.step(2)
    t.put(m.CENTER_X)
    t.step(2)
    assert S.t[j][under].max() < once[under].min()


def test_pin_left_on_the_dough_capped():
    """A finished sheet with the pin let go lying on its rings on the dough (not put away): 0.5, not 1.0."""
    t = Trial()
    demo_rolling(t, put_away=False)
    t.put(m.CENTER_X)
    t.step(5)
    assert t.let_go()[0] == 0.5
    assert not m.clear_of_dough(t.objects())
    t = Trial()
    demo_rolling(t, put_away=False)
    end = dk.ends()[:, 1].max()
    t.lift_to(end + 0.005)  # just past the sheet's far end, the barrel's face against its edge
    assert t.let_go()[0] == 0.5
    t.lift_to(end + 0.015)  # 1.5 cm clear
    assert t.let_go()[0] == 1.0



def test_numbers_in_text():
    doc = " ".join(m.score.__doc__.split())
    for s in ("2 cm", "3.2 mm", "1/8 inch", "0.3 mm", "90 %", "3.7 mm", "4.2 mm", "4.8 mm", "3 degrees", "5 mm",
              "0.25", "on the board", "1 cm clear", "One or two hands", "overlap"):
        assert s in doc, s
    assert m.TARGET_T == 0.0032 and m.RING_GAP == m.TARGET_T and m.TOL_T == 0.0003 and m.SPEC_SHARE == 0.90
    assert m.HALF_T == 0.0037 and m.QUARTER_T == 0.0042 and m.WEDGE_MAX == 0.0003 and m.LEVEL_DEG == 3.0
    assert m.DOUGH_SIZE[2] == 0.0048 and m.CELL == 0.02 and m.PUT_CLEAR == 0.01
    assert m.PIN_DOWN == 0.005 and m.TOUCH == 0.0 and m.DOUGH_STRESS == 13e3 and m.BOARD_FRICTION == (0.25, 0.20)
    assert "sealed" not in doc and "torn" not in doc and "puff" not in doc.lower()
    assert "dough" in m.INSTRUCTION and "off the dough" in m.INSTRUCTION
    assert not hasattr(m, "_gate") and "gate" not in dk._CFG  # no hand gate: the load decides
    assert not hasattr(m, "REST_X") and not hasattr(m, "CLEAR")


if __name__ == "__main__":
    for name, f in list(globals().items()):
        if name.startswith("test_"):
            f()
            print("ok", name)
    print("all pass")
