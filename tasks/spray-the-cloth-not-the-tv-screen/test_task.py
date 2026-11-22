"""Offline checks of the TV screen cleaning task's spray, wipe and scorer (no simulator): python -m pytest test_task.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import task as t  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

Z3 = np.zeros(3)


def _quat(R):
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def _st(pos, R=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=_quat(R), lin_vel=Z3, ang_vel=Z3, low=p, high=p)


TV0 = (np.array([t.TV_X, t.TV_Y, t.TOP]), np.eye(3))
B_R0 = _rz(t.BOTTLE_YAW0)
BOTTLE0 = (np.array([t.BOTTLE_X0, t.BOTTLE_Y0, t.TOP]), B_R0)
BOTTLE_UP = (np.array([t.BOTTLE_X0, t.BOTTLE_Y0, t.TOP + t.LIFT]), B_R0)
CLOTH0 = (np.array([t.PAD_X0, t.PAD_Y0, t.TOP]), np.eye(3))
NZ = BOTTLE_UP[0] + B_R0 @ t.NOZZLE
CLOTH_SPRAY = (np.array([NZ[0], NZ[1] + t.SPRAY_GAP, NZ[2]]), _rx(-math.pi / 2))  # underside facing the bottle
SCREEN_X = t.TV_X + t.PF


def cloth_wipe(y, z=t.TOP + t.WIPE_Z, R=None):
    """The cloth pressed on the screen, underside (bottom face) on it, its middle at (y, z)."""
    R = _ry(-math.pi / 2) if R is None else R
    return (np.array([SCREEN_X - 0.0002, y, z]), R)


def frame(bottle=BOTTLE0, cloth=CLOTH0, tv=TV0, trig=0.0, press=2.0):
    """One control step's objects; `press` (N) is the TV's push on the cloth (only counted while it touches)."""
    kit._JOINTS["bottle/trigger"] = kit.JointState(pos=float(trig), vel=0.0, latched=False)
    t.PRESS["vec"] = np.array([-float(press), 0.0, 0.0])
    return {"tv": _st(*tv), "bottle": _st(*bottle), "cloth": _st(*cloth), "bottle/trigger": _st(bottle[0])}


def _run(seq, clear=True):
    """Score a trial: seq is a list of (frame kwargs); hands are clear (5 cm) unless clear=False (1 mm)."""
    t._L = None
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    t.LINK_POSES.clear()
    real = t.hand_gap
    t.hand_gap = (lambda o, n: 0.05) if clear else (lambda o, n: 0.001)
    s = None
    try:
        for i, kw in enumerate(seq):
            kit._EPISODE["last"] = i
            s = t.score(frame(**kw))
    finally:
        t.hand_gap = real
    return s


def squeeze(n=2, **kw):
    out = []
    for _ in range(n):
        out += [dict(trig=0.0, **kw)] * 3 + [dict(trig=0.18, **kw)] * 3
    return out + [dict(trig=0.0, **kw)] * 3


def wipe(face_R=None, n=30):
    ys = np.linspace(t.WIPE_Y[0], t.WIPE_Y[1], n)
    return [dict(cloth=cloth_wipe(y, R=face_R)) for y in ys]


HOLD = [dict()] * (t.HOLD_STEPS + 5)


def safe_trial(**extra):
    return ([dict()] * 5 + squeeze(bottle=BOTTLE_UP, cloth=CLOTH_SPRAY) + wipe() + HOLD)


# -- the spray's geometry --------------------------------------------------------------------------------------------
def test_cone_has_24_unit_rays_within_the_angle():
    d = t.cone_dirs()
    assert d.shape == (24, 3)
    assert np.allclose(np.linalg.norm(d, axis=1), 1.0)
    assert np.degrees(np.arccos(d[:, 0].clip(-1, 1))).max() <= t.CONE_DEG + 1e-6


def test_demo_spray_lands_wholly_on_the_cloths_underside():
    hits = t.cast(frame(bottle=BOTTLE_UP, cloth=CLOTH_SPRAY))
    assert all(h[0] == "cloth" and h[2] == "bottom" for h in hits)


def test_demo_spray_still_on_the_cloth_with_the_bottle_sagging_1cm():
    b = (BOTTLE_UP[0] - [0, 0, 0.01], B_R0)
    hits = t.cast(frame(bottle=b, cloth=CLOTH_SPRAY))
    assert sum(h[0] == "cloth" for h in hits) == 24


def test_nozzle_turned_30_degrees_toward_the_tv_hits_the_screen():
    b = (BOTTLE_UP[0], _rz(t.BOTTLE_YAW0 - math.pi / 6))
    hits = t.cast(frame(bottle=b))
    assert sum(h[0] == "tv" for h in hits) == 24


def test_spray_from_the_start_pose_misses_the_tv():
    hits = t.cast(frame())
    assert not any(h[0] == "tv" for h in hits)


def test_cloth_held_in_front_of_the_screen_with_mist_past_its_edges_wets_the_tv():
    # the cloth held flat against nothing, 5 cm in front of the screen, sprayed from 25 cm: the cone (5 cm across
    # there) is wider than the cloth's 7.6 cm only past its edge; aim the bottle at its edge
    b = (np.array([SCREEN_X - 0.30, 0.10 + 0.07, t.TOP + 0.13 - t.NOZZLE[2]]), _rz(0.0))
    c = cloth_wipe(0.10, R=_ry(-math.pi / 2))
    c = (c[0] - [0.05, 0, 0], c[1])
    hits = t.cast(frame(bottle=b, cloth=c))
    assert any(h[0] == "tv" for h in hits) and any(h[0] == "cloth" for h in hits)


def test_upside_down_bottle_does_not_fire():
    assert not t.dip_tube_ok(frame(bottle=(BOTTLE_UP[0], _rx(math.pi) @ B_R0)))
    assert t.dip_tube_ok(frame(bottle=BOTTLE_UP))


# -- wiping ------------------------------------------------------------------------------------------------------------
def test_cloth_pressed_on_the_screen_is_detected_by_face():
    assert t.face_on_screen(frame(cloth=cloth_wipe(0.12))) == "bottom"
    top = (np.array([SCREEN_X - 0.0002 - t.PAD_T, 0.12, t.TOP + t.WIPE_Z]), _ry(math.pi / 2))
    assert t.face_on_screen(frame(cloth=top)) == "top"


def test_cloth_held_off_the_screen_or_on_edge_does_not_wipe():
    off = cloth_wipe(0.12)
    assert t.touches(frame(cloth=off))  # touching
    assert not t.touches(frame(cloth=(off[0] - [0.006, 0, 0], off[1])))  # 6 mm off
    tilted = (off[0], _rz(math.radians(25)) @ off[1])
    assert not t.touches(frame(cloth=tilted))  # 25 degrees off flat


def test_the_wipe_pass_covers_every_fingerprint():
    seen = set()
    for y in np.linspace(t.WIPE_Y[0], t.WIPE_Y[1], 30):
        seen |= set(t.smudges_under(frame(cloth=cloth_wipe(y))))
    assert seen == set(range(len(t.SMUDGES)))


# -- the scorer: the demo and its variants (the cheat modes) -----------------------------------------------------------
def test_do_nothing_is_zero():
    assert _run([dict()] * 40) == 0.0


def test_safe_spray_then_wipe_then_set_down_is_one():
    assert _run(safe_trial()) == 1.0


def test_safe_but_still_holding_is_half():
    assert _run(safe_trial(), clear=False) == 0.5


def test_not_yet_still_for_a_second_is_half():
    assert _run([dict()] * 5 + squeeze(bottle=BOTTLE_UP, cloth=CLOTH_SPRAY) + wipe() + [dict()] * 10) == 0.5


def test_spraying_the_screen_is_zero_even_after_a_good_wipe():
    b = (BOTTLE_UP[0], _rz(t.BOTTLE_YAW0 - math.pi / 6))
    seq = [dict()] * 5 + squeeze(1, bottle=b) + squeeze(bottle=BOTTLE_UP, cloth=CLOTH_SPRAY) + wipe() + HOLD
    assert _run(seq) == 0.0


def test_one_shot_on_the_screen_latches_zero():
    b = (BOTTLE_UP[0], _rz(t.BOTTLE_YAW0 - math.pi / 6))
    assert _run([dict()] * 5 + squeeze(1, bottle=b) + HOLD) == 0.0


def test_dry_wipe_is_capped_at_half():
    assert _run([dict()] * 5 + wipe() + HOLD) == 0.5


def test_wiping_with_the_dry_face_after_spraying_the_other_is_half():
    dry_face = _ry(math.pi / 2)  # the top face toward the screen
    seq = [dict()] * 5 + squeeze(bottle=BOTTLE_UP, cloth=CLOTH_SPRAY)
    for y in np.linspace(t.WIPE_Y[0], t.WIPE_Y[1], 30):
        seq.append(dict(cloth=(np.array([SCREEN_X - 0.0002 - t.PAD_T, y, t.TOP + t.WIPE_Z]), dry_face)))
    assert _run(seq + HOLD) == 0.5


def test_spray_the_cloth_but_no_wipe_is_quarter():
    assert _run([dict()] * 5 + squeeze(bottle=BOTTLE_UP, cloth=CLOTH_SPRAY) + HOLD) == 0.25


def test_one_weak_squeeze_does_not_fire():
    seq = [dict()] * 5 + [dict(trig=0.12, bottle=BOTTLE_UP, cloth=CLOTH_SPRAY)] * 10 + wipe() + HOLD
    assert _run(seq) == 0.5  # no shot: the cloth stays dry, the wipe only smears


def test_holding_the_trigger_in_fires_once():
    t._L = None
    seq = [dict()] * 3 + [dict(trig=0.19, bottle=BOTTLE_UP, cloth=CLOTH_SPRAY)] * 30
    _run(seq)
    assert t._L.shots == 1


def test_one_partial_shot_on_the_cloth_is_not_damp():
    # only 12 of 24 rays on the cloth (aimed at its end): below DAMP_HITS
    c = (CLOTH_SPRAY[0] + [0.076 + 0.002, 0, 0], CLOTH_SPRAY[1])
    t._L = None
    _run([dict()] * 3 + squeeze(1, bottle=BOTTLE_UP, cloth=c))
    assert 0 < t._L.face_hits["bottom"] < t.DAMP_HITS


def test_partial_wipe_is_quarter():
    ys = np.linspace(t.WIPE_Y[0], 0.09, 10)  # covers only the first column of fingerprints
    seq = [dict()] * 5 + squeeze(bottle=BOTTLE_UP, cloth=CLOTH_SPRAY) + [dict(cloth=cloth_wipe(y)) for y in ys]
    assert _run(seq + HOLD) == 0.25


def test_spraying_the_cloth_while_it_lies_on_the_screen_is_zero():
    c = cloth_wipe(0.12)
    b = (np.array([SCREEN_X - 0.06, 0.12, t.TOP + t.WIPE_Z - t.NOZZLE[2]]), _rz(0.0))
    seq = [dict()] * 5 + squeeze(bottle=b, cloth=c) + wipe() + HOLD
    assert _run(seq) == 0.0


def test_tv_shoved_or_tipped_is_zero():
    shoved = (TV0[0] + [0.04, 0, 0], np.eye(3))
    assert _run(safe_trial() + [dict(tv=shoved)] * 3 + HOLD) == 0.0
    tipped = (TV0[0], _ry(math.radians(12)))
    assert _run(safe_trial()[:-30] + [dict(tv=tipped)] * 3 + HOLD) == 0.0


def test_bottle_left_lying_down_is_half():
    lying = (BOTTLE0[0] + [0, 0, 0.03], _rx(math.pi / 2) @ B_R0)
    seq = [dict()] * 5 + squeeze(bottle=BOTTLE_UP, cloth=CLOTH_SPRAY) + wipe() + [dict(bottle=lying)] * 30
    assert _run(seq) == 0.5


def test_cloth_left_on_the_screen_or_held_up_is_half():
    seq = [dict()] * 5 + squeeze(bottle=BOTTLE_UP, cloth=CLOTH_SPRAY) + wipe() + [dict(cloth=cloth_wipe(0.2))] * 30
    assert _run(seq) == 0.5


# -- the layout --------------------------------------------------------------------------------------------------------
def test_fingerprints_on_the_screen():
    for y, z in t.SMUDGES:
        assert t.SCREEN_Y[0] + t.SMUDGE_R <= y <= t.SCREEN_Y[1] - t.SMUDGE_R
        assert t.SCREEN_Z[0] + t.SMUDGE_R <= z <= t.SCREEN_Z[1] - t.SMUDGE_R


def test_bottle_and_cloth_clear_of_the_tv_at_the_start():
    tv_front = t.TV_X - t.TV_D / 2
    assert t.BOTTLE_X0 + t.BODY_HY < tv_front or abs(t.BOTTLE_Y0) + t.BODY_HX < t.FOOT_Y - 0.02
    assert t.PAD_X0 + t.PAD_L / 2 < t.TV_X + t.PF - 0.05


def test_grip_span_fits_the_open_gripper():
    span = t.SPAN[1] - t.SPAN[0]
    assert span + 0.010 <= 2 * t.FINGER_HALF_OPEN
    assert t.PAD_W + 0.010 <= 2 * t.FINGER_HALF_OPEN


def test_finger_band_stays_under_the_nozzle():
    # the finger pads span 3.4 cm either side of the tool point across the fingers' width (yamgeom: 6.8 cm)
    assert t.GRIP_Z + 0.034 < t.NOZZLE_Z[0]
    assert t.GRIP_Z - 0.034 > t.BODY_H - 0.010  # the back finger's lower edge only on the body's flat back


def test_demo_is_open_loop():
    import inspect
    src = inspect.getsource(t._right) + inspect.getsource(t._left) + inspect.getsource(t.demo)
    assert "objects" not in src


# -- mist on the other arm -----------------------------------------------------------------------------------------------
def _left_hand_at(point):
    """Put the left arm's fingertip link (only) so its hull's centroid sits at `point`."""
    import yamgeom as yg
    _, cen, _ = yg.cloud("tip_left")
    t.LINK_POSES.clear()
    t.LINK_POSES["left_robot/tip_left"] = (np.asarray(point, float) - cen, np.eye(3))


def test_spraying_the_other_hand_is_zero():
    target = NZ + np.array([0.0, 0.10, 0.0])  # 10 cm along the nozzle's axis (+y), no cloth in the way
    _left_hand_at(target)
    try:
        hits = t.cast(frame(bottle=BOTTLE_UP))
        assert sum(h[0] == "robot" for h in hits) >= 12
        t._L = None
        kit._EPISODE["count"] += 1
        for i, kw in enumerate([dict()] * 3 + squeeze(1, bottle=BOTTLE_UP) + wipe() + HOLD):
            kit._EPISODE["last"] = i
            real = t.hand_gap
            t.hand_gap = lambda o, n: 0.05
            try:
                s = t.score(frame(**kw))
            finally:
                t.hand_gap = real
        assert t._L.robot_wet and s == 0.0
    finally:
        t.LINK_POSES.clear()


def test_hand_behind_the_cloth_stays_dry():
    _left_hand_at(np.array([CLOTH_SPRAY[0][0], CLOTH_SPRAY[0][1] + 0.05, CLOTH_SPRAY[0][2]]))  # 5 cm behind its face
    try:
        hits = t.cast(frame(bottle=BOTTLE_UP, cloth=CLOTH_SPRAY))
        assert all(h[0] == "cloth" for h in hits)
    finally:
        t.LINK_POSES.clear()


# -- the wipe needs real contact and a slide (red team B1) ---------------------------------------------------------------
def _sprayed():
    return [dict()] * 5 + squeeze(bottle=BOTTLE_UP, cloth=CLOTH_SPRAY)


def test_hovering_slide_does_not_clean():
    for off in (0.00075, 0.0015, 0.0025, 0.0029):
        seq = _sprayed() + [dict(cloth=(c[0] - [off - 0.0002, 0, 0], c[1]))
                            for c in (cloth_wipe(y) for y in np.linspace(t.WIPE_Y[0], t.WIPE_Y[1], 30))]
        assert _run(seq + HOLD) == 0.25, off


def test_a_static_dab_does_not_clean():
    across = _ry(-math.pi / 2) @ _rz(math.pi / 2)  # long side along y: covers all six prints at once
    seq = _sprayed() + [dict(cloth=cloth_wipe(0.12, R=across))] * 40
    assert _run(seq + HOLD) == 0.25


def test_a_tilted_cloth_does_not_clean():
    tilted = _rz(math.radians(14)) @ _ry(-math.pi / 2)
    seq = _sprayed() + [dict(cloth=cloth_wipe(y, R=tilted)) for y in np.linspace(t.WIPE_Y[0], t.WIPE_Y[1], 30)]
    assert _run(seq + HOLD) == 0.25


def test_one_slow_slide_counts_by_distance_not_steps():
    ys = np.linspace(t.WIPE_Y[0], t.WIPE_Y[1], 300)  # ten times slower: same result
    seq = _sprayed() + [dict(cloth=cloth_wipe(y)) for y in ys]
    assert _run(seq + HOLD) == 1.0


# -- the holding hand is found by contact (red team B2) -----------------------------------------------------------------
def _place(key, link, point):
    import yamgeom as yg
    _, cen, _ = yg.cloud(link)
    t.LINK_POSES[key] = (np.asarray(point, float) - cen, np.eye(3))


def test_cloth_hand_near_the_nozzle_is_still_sprayed():
    t.LINK_POSES.clear()
    try:
        _place("left_robot/tip_left", "tip_left", NZ + np.array([0.0, 0.05, 0.0]))  # 5 cm along the axis, not touching
        f = frame(bottle=BOTTLE_UP)
        assert t.holding_links(f) == set()
        assert sum(h[0] == "robot" for h in t.cast(f)) >= 6
    finally:
        t.LINK_POSES.clear()


def test_holding_hands_fingers_are_exempt_but_not_its_arm():
    import yamgeom as yg
    t.LINK_POSES.clear()
    try:
        f = frame(bottle=BOTTLE_UP)
        pts, cen, _ = yg.cloud("tip_left")
        # a finger link placed so its surface touches the bottle's back: it holds the bottle
        back = BOTTLE_UP[0] + B_R0 @ np.array([t.SHROUD_X[0] - 0.0005, 0.0, 0.20])
        far = pts[np.argmax(pts @ (B_R0 @ np.array([1.0, 0, 0])))]  # its point farthest along the nozzle's direction
        t.LINK_POSES["right_robot/tip_left"] = (back - far, np.eye(3))
        assert "right_robot/tip_left" in t.holding_links(f)
        # the same arm's forearm (link4) in the mist: still counted
        _place("right_robot/link4", "link4", NZ + np.array([0.0, 0.10, 0.0]))
        assert any(h[0] == "robot" for h in t.cast(f))
    finally:
        t.LINK_POSES.clear()


def test_a_still_press_with_tremor_does_not_clean():
    across = _ry(-math.pi / 2) @ _rz(math.pi / 2)  # covers all six prints
    rng = np.random.default_rng(0)
    seq = _sprayed()
    for _ in range(200):  # 10 s pressed still, +-0.5 mm of tremor every step
        c = cloth_wipe(0.12, R=across)
        seq.append(dict(cloth=(c[0] + np.r_[0.0, rng.uniform(-0.0005, 0.0005, 2)], c[1])))
    assert _run(seq + HOLD) == 0.25


def test_scrubbing_in_place_counts():
    ys = np.concatenate([np.linspace(0.04, 0.20, 30), np.linspace(0.20, 0.04, 30)])
    seq = _sprayed() + [dict(cloth=cloth_wipe(y)) for y in ys]
    assert _run(seq + HOLD) == 1.0


def test_lifting_off_breaks_the_stroke():
    seq = _sprayed()
    for k in range(10):  # 2 cm strokes, each lifted 5 mm off before the next: never 3 cm in one touch
        y0 = 0.02 + 0.02 * k
        for y in np.linspace(y0, y0 + 0.02, 5):
            seq.append(dict(cloth=cloth_wipe(y)))
        c = cloth_wipe(y0 + 0.02)
        seq.append(dict(cloth=(c[0] - [0.005, 0, 0], c[1])))
    assert _run(seq + HOLD) == 0.25


def seated(y, R, z=t.TOP + t.WIPE_Z, gap=0.0002):
    """The cloth at (y, z), turned by R, pushed toward the screen until its deepest point is `gap` in front of it (a
    pose the rigid pad can really take)."""
    p = np.array([SCREEN_X - 0.05, y, z])
    pts = p + np.column_stack([t._GRID, np.zeros(len(t._GRID))]) @ R.T
    return (p + np.array([pts[:, 0].max() * -1 + SCREEN_X - gap, 0, 0]), R)


def _sweep(R, gap=0.0002):
    ys = np.concatenate([np.linspace(t.WIPE_Y[0], t.WIPE_Y[1], 30), np.linspace(t.WIPE_Y[1], t.WIPE_Y[0], 30)])
    return _run(_sprayed() + [dict(cloth=seated(y, R, gap=gap)) for y in ys] + HOLD)


FWD = _ry(-math.pi / 2)


def test_seated_flat_cloth_wipes_and_a_hover_does_not():
    assert _sweep(FWD) == 1.0
    assert _sweep(FWD, gap=0.00075) == 0.25


def test_a_little_off_flat_still_wipes():
    # 2 degrees about the vertical (the pinch holds this angle); 1.5 about the horizontal (the pad's long axis: 15 cm of
    # it, and the pinch lets it pivot to lie flat)
    for R in (_rz(math.radians(2)) @ FWD, _rz(math.radians(-2)) @ FWD, _ry(math.radians(1.5)) @ FWD,
              _ry(math.radians(-1.5)) @ FWD):
        assert _sweep(R) == 1.0


def test_a_steeply_tilted_cloth_touches_only_near_its_edge():
    R = _rz(math.radians(9)) @ FWD  # within FLAT_DEG but far past the cloth's give across its 7.6 cm width
    f = frame(cloth=seated(0.12, R))
    c, Rc = seated(0.12, R)
    touched = t.touches(f)
    for i in touched:  # every touched print lies within the give of the touching edge
        y, z = t.SMUDGES[i]
        w = np.array([SCREEN_X, y, z])
        n = Rc @ np.array([0.0, 0.0, -1.0])
        assert np.dot(w - c, n) / n[0] <= t.GIVE + 1e-9
    assert len(touched) < 6
    assert _sweep(_rz(math.radians(14)) @ FWD) == 0.25  # past FLAT_DEG: no wipe at all


def test_probe_trace_runs(tmp_path=None):
    import tempfile
    f = os.path.join(tempfile.mkdtemp(), "trace.txt")
    old = t._PROBE
    t._PROBE = f
    try:
        _run([dict()] * 3 + squeeze(1, bottle=BOTTLE_UP, cloth=CLOTH_SPRAY) + wipe()[:5])
        lines = open(f).read().splitlines()
        assert len(lines) > 10 and "gaps" in lines[-1] and "tvyaw" in lines[-1]
    finally:
        t._PROBE = old


# -- the wipe needs a real press (final skeptic: #3517 'proximity, no force') -------------------------------------------
def test_a_force_free_touch_wipes_nothing():
    ys = np.concatenate([np.linspace(t.WIPE_Y[0], t.WIPE_Y[1], 30), np.linspace(t.WIPE_Y[1], t.WIPE_Y[0], 30)])
    for gap in (0.0, 0.0002, 0.0004):  # on or under 0.5 mm from the glass, but pressing with nothing
        for press in (0.0, 0.9):
            seq = _sprayed() + [dict(cloth=seated(y, FWD, gap=gap), press=press) for y in ys]
            assert _run(seq + HOLD) == 0.25, (gap, press)
    seq = _sprayed() + [dict(cloth=seated(y, FWD, gap=0.0002), press=1.0) for y in ys]
    assert _run(seq + HOLD) == 1.0


def test_a_press_that_lets_up_breaks_the_stroke():
    seq = _sprayed()
    for k in range(10):  # 2 cm strokes with the press let off (0 N) between them, the cloth never lifted
        y0 = 0.02 + 0.02 * k
        seq += [dict(cloth=seated(y, FWD)) for y in np.linspace(y0, y0 + 0.02, 5)]
        seq.append(dict(cloth=seated(y0 + 0.02, FWD), press=0.0))
    assert _run(seq + HOLD) == 0.25


def test_no_force_read_in_the_sim_is_an_error():
    import pytest
    old = dict(t.PRESS)
    t.PRESS["n"] = t.PRESS["vec"] = None
    sys.modules.setdefault("isaaclab", type(sys)("isaaclab"))
    try:
        with pytest.raises(RuntimeError):
            t.touches({"tv": _st(*TV0), "cloth": _st(*cloth_wipe(0.12))})
    finally:
        del sys.modules["isaaclab"]
        t.PRESS.update(old)
