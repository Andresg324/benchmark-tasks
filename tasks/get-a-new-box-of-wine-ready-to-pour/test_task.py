"""Offline checks of the box-of-wine task's scorer and layout (no simulator): python -m pytest test_task.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import task as t  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

Z3 = np.zeros(3)
C0 = np.array([t.X_C, t.Y_C, t.TOP])


def _st(pos, R=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(t._quat(np.asarray(R, float))), lin_vel=Z3, ang_vel=Z3, low=p,
                       high=p)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _objs(neck, R=np.eye(3), carton=C0, cR=np.eye(3)):
    """The carton at `carton` turned by cR, the tap's neck at `neck` (carton frame) turned by R (carton frame)."""
    cp = np.asarray(carton, float)
    return {"carton": _st(cp, cR), "carton/spigot": _st(cp + cR @ np.asarray(neck, float), cR @ R)}


def _torn(on=True):
    kit._LAW_MEM["carton/tab_x"] = (kit._EPISODE["count"], {"torn": on, "peak": 0.0})


START = np.array(t.SPIGOT_START)
SEAT = np.array(t.SEAT)
OUT_HANG = np.array([t.SEAT[0] - 0.018, 0.0, t.SEAT[2] - 0.03])


def _run(frames, torn=True):
    t._L = None
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    t.LINK_POSES.clear()
    _torn(torn)
    keep, s = [], None
    for f in frames:
        f = f() if callable(f) else f
        keep.append(f)
        s = t.score(f)
    return s


def _hold(o, n=t.HOLD_STEPS + 3):
    return [o] * 1 + [dict(o) for _ in range(n)]


def test_untouched_is_zero():
    assert _run(_hold(_objs(START)), torn=False) == 0.0


def test_tab_torn_only():
    assert _run(_hold(_objs(START)), torn=True) == t.TAB_T


def test_seated_is_one():
    assert _run(_hold(_objs(SEAT))) == 1.0


def test_seated_but_not_held_long_enough():
    assert _run([_objs(SEAT)] * 5) == t.OUT_T


def test_dangling_outside_is_half():
    assert t.tap_out(_objs(OUT_HANG))
    assert not t.seated(_objs(OUT_HANG))
    assert _run(_hold(_objs(OUT_HANG))) == t.OUT_T


def test_upside_down_in_slot_is_half():
    o = _objs(SEAT, _rx(math.pi))
    assert not t.seated(o)
    assert _run(_hold(o)) == t.OUT_T
    # turned part way over and pitched, as a sim run left it: still out, not seated
    o = _objs(SEAT, _ry(math.radians(-8)) @ _rx(math.radians(132)))
    assert t.tap_out(o) and not t.seated(o)
    assert _run(_hold(o)) == t.OUT_T


def test_tap_inside_is_not_out():
    assert not t.tap_out(_objs(START))
    assert not t.tap_out(_objs(np.array([0.010, 0.0, t.SPIGOT_START[2]])))


def test_tilted_too_far_is_not_seated():
    assert t.seated(_objs(SEAT, _ry(math.radians(12))))
    assert not t.seated(_objs(SEAT, _ry(math.radians(18))))


def test_pointing_into_the_box_is_not_seated():
    assert not t.seated(_objs(SEAT, np.diag([-1.0, -1.0, 1.0])))


def test_groove_off_the_panel_is_not_seated():
    assert not t.seated(_objs(SEAT + [0.004, 0, 0]))
    assert not t.seated(_objs(SEAT + [-0.004, 0, 0]))
    assert t.seated(_objs(SEAT + [0.002, 0, 0]))


def test_neck_on_the_opening_edge_not_in_the_slot():
    assert not t.seated(_objs(np.array([SEAT[0], 0.03, t.HOLE_Z[0] + t.NECK_R])))
    assert not t.seated(_objs(np.array([SEAT[0], 0.0, t.HOLE_Z[0] + 0.012])))


def _link_on_tap(link, tip_offset):
    """Place arm link `link` so its nearest hull point sits `tip_offset` m in front of the tap body's front face."""
    tp = C0 + SEAT
    pts = t.yg.cloud(link)[0]
    p0 = pts[np.argmax(pts[:, 0])]  # the link's foremost hull point goes on the body's front face, at its middle
    front = tp[0] + t.BODY[0][0]
    return np.array([front - tip_offset, tp[1], tp[2] + t.BODY[2][0] / 2]) - p0


def _run_with_link(link, offset):
    t._L = None
    kit._EPISODE["count"] += 1
    _torn(True)
    t.LINK_POSES.clear()
    t.LINK_POSES[f"right_robot/{link}"] = (_link_on_tap(link, offset), np.eye(3))
    keep, s = [], None
    for _ in range(t.HOLD_STEPS + 3):
        f = _objs(SEAT)
        keep.append(f)
        s = t.score(f)
    t.LINK_POSES.clear()
    return s


def test_held_in_the_slot_is_half():
    """A fingertip touching the tap at the end: not let go."""
    assert _run_with_link("tip_left", -0.001) == t.OUT_T


def test_forearm_resting_on_the_tap_is_half():
    assert _run_with_link("link3", -0.001) == t.OUT_T


def test_hand_between_touch_and_clear_is_half():
    """3.5 mm away: not holding it, but not clear either."""
    assert _run_with_link("tip_left", 0.0035) == t.OUT_T


def test_hand_clear_is_one():
    assert _run_with_link("tip_left", 0.010) == 1.0


def test_roll_limits():
    assert t.seated(_objs(SEAT, _rx(math.radians(25))))
    assert not t.seated(_objs(SEAT, _rx(math.radians(35))))


def test_neck_jammed_high_in_the_slot_is_not_seated():
    assert not t.seated(_objs(SEAT + [0, 0, 0.005]))


def test_hand_gap_detects_touch():
    o = _objs(SEAT)
    tp = C0 + SEAT
    t.LINK_POSES.clear()
    pts, cen, _ = t.yg.cloud("tip_left")
    # put the fingertip's nearest point 1 mm in front of the tap body's front face
    front = tp[0] + t.BODY[0][0]
    shift = np.array([front - 0.001 - pts[:, 0].max(), tp[1] - cen[1], tp[2] - cen[2]])
    t.LINK_POSES["right_robot/tip_left"] = (shift, np.eye(3))
    assert t.hand_gap(o) <= t.TOUCH
    t.LINK_POSES["right_robot/tip_left"] = (shift - [0.01, 0, 0], np.eye(3))
    assert t.hand_gap(o) > t.CLEAR
    t.LINK_POSES.clear()


def test_tap_still_moving_does_not_count():
    frames = [_objs(SEAT + [0.0, 0.0, 0.002 * (k % 2)]) for k in range(t.HOLD_STEPS + 5)]
    assert _run(frames) == t.OUT_T


def test_perforation_law():
    law = t.Perforation(t.TAB_K, t.F_TEAR)
    mem = law.fresh(None)
    assert law.drive("k", 0.001, 0.0, {}, mem)[1] == t.TAB_K and not mem["torn"]
    law.drive("k", t.TAB_GIVE * 1.01, 0.0, {}, mem)
    assert mem["torn"]
    assert law.drive("k", 0.0, 0.0, {}, mem)[1] == 0.0  # torn for good


def test_layout_fits():
    # the flanges (30 mm squares) do not pass through the 24 mm slot; the tap's body (32 x 34) passes the opening
    assert 2 * t.FLANGE_HALF > 2 * t.NOTCH_Y
    assert 2 * t.NECK_R < 2 * t.NOTCH_Y
    assert 2 * t.NECK_HALF > t.PT
    assert t.HOLE_Z[1] - t.HOLE_Z[0] > t.BODY[2][1] - t.BODY[2][0] + 0.03
    # the tap spawns 2 mm over the bag (outlet over the low step, flanges over the high one) and settles onto it
    assert abs(t.SPIGOT_START[2] + t.SPOUT_Z[0] - t.BAG_LOW_Z - 0.002) < 1e-6
    assert abs(t.SPIGOT_START[2] - t.FLANGE_HALF - t.BAG_STEP_Z - 0.002) < 1e-6
    # nothing of the articulation starts within 1 mm of the carton (contacts at spawn kick a free articulation)
    assert t.TAB_GAP >= 0.0015
    assert t.SPIGOT_START[0] + t.NECK_HALF >= t.BAG_STEP_X
    # seated, the tap's rear (gland) clears the bag's front
    assert t.SEAT[0] + t.GLAND_X[1] < t.BAG_LOW_X
    # the tap's chain lets it reach the seat and a little further out
    assert t.REACH["x"][0] < t.SEAT[0] - t.SPIGOT_START[0]
    assert t.REACH["z"][0] < t.SEAT[2] - t.SPIGOT_START[2]
    # every object has a visible mesh
    assert t.CARTON.meshes and all(lk.meshes for lk in t.CARTON.links if lk.colliders)


def test_demo_variants_build():
    for v in ("", "nothing", "dangling", "upside", "held", "nosteady"):
        t.VARIANT = v
        plan = t.demo()
        assert abs(sum(w.duration for w in plan["left"]) - sum(w.duration for w in plan["right"])) < 1e-6
    t.VARIANT = ""


def test_carton_knocked_over_is_zero():
    # seated in the carton's frame, but the carton lies on its back (and was knocked over at any point)
    lying = _ry(-math.pi / 2)
    assert _run(_hold(_objs(SEAT, cR=lying))) == 0.0
    frames = [_objs(SEAT, cR=_ry(-math.radians(50)))] + _hold(_objs(SEAT))
    assert _run(frames) == 0.0


def _tipped_back(deg):
    """The carton turned `deg` back about its back bottom edge (its front lifts): its origin and rotation."""
    R = _ry(math.radians(deg))
    edge = C0 + [t.CD, 0.0, 0.0]
    return edge - R @ np.array([t.CD, 0.0, 0.0]), R


def test_carton_leaning_a_little_still_counts():
    org, R = _tipped_back(8)
    assert _run([_objs(SEAT)] + _hold(_objs(SEAT, carton=org, cR=R))) == 1.0
    org, R = _tipped_back(20)
    assert _run([_objs(SEAT)] + _hold(_objs(SEAT, carton=org, cR=R))) == t.OUT_T


def test_carton_shoved_off_its_place_is_half():
    start = [_objs(SEAT)]
    moved = C0 + [t.SLID + 0.005, 0.0, 0.0]
    assert _run(start + _hold(_objs(SEAT, carton=moved))) == t.OUT_T
    near = C0 + [t.SLID - 0.005, 0.0, 0.0]
    assert _run(start + _hold(_objs(SEAT, carton=near))) == 1.0


def test_carton_sliding_does_not_count_as_still():
    frames = [_objs(SEAT, carton=C0 + [0.0015 * k, 0.0, 0.0]) for k in range(t.HOLD_STEPS + 5)]
    assert _run(frames) == t.OUT_T


def _rz(a):
    c, s_ = math.cos(a), math.sin(a)
    return np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])


def test_carton_turned_in_place_is_not_shoved():
    # turned 60 degrees about its footprint's middle: the footprint has not moved
    R = _rz(math.radians(60))
    mid = C0 + [t.CD / 2, 0.0, 0.0]
    origin = mid - R @ np.array([t.CD / 2, 0.0, 0.0])
    assert _run([_objs(SEAT)] + _hold(_objs(SEAT, carton=origin, cR=R))) == 1.0


def test_carton_held_tilted_or_lifted_is_half():
    assert _run([_objs(SEAT)] + _hold(_objs(SEAT, cR=_ry(-math.radians(20))))) == t.OUT_T
    assert _run([_objs(SEAT)] + _hold(_objs(SEAT, carton=C0 + [0.0, 0.0, 0.02]))) == t.OUT_T
