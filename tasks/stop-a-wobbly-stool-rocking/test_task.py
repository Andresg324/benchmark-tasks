"""Offline checks of the wobbly-stool task's geometry, scorer and demo plan (no simulator): python -m pytest test_task.py"""
import math
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import task as t  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

Z3 = np.zeros(3)


def _st(pos, R=np.eye(3), v=Z3):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(t._quat(np.asarray(R, float))), lin_vel=np.asarray(v, float),
                       ang_vel=Z3, low=p, high=p)


def _shim_pose(xy, side):
    yaw = math.pi / 2 if side > 0 else -math.pi / 2
    c, s = math.cos(yaw), math.sin(yaw)
    return _st((xy[0], xy[1], t.TOP + 0.0002), np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]]))


SHIM_START = {"shim_left": ((t.STOOL_XY[0], t.SHIM_Y), 1), "shim_right": ((t.STOOL_XY[0], -t.SHIM_Y), -1)}


def _objs(stool_R=None, stool_dx=(0.0, 0.0), shims=None, v=Z3):
    R = t.STOOL_R0 if stool_R is None else stool_R
    p = np.array(t.STOOL_POS) + np.array([stool_dx[0], stool_dx[1], 0.0])
    out = {"stool": _st(p, R, v)}
    for name, (xy, side) in SHIM_START.items():
        out[name] = _shim_pose(*(shims or {}).get(name, (xy, side)))
    return out


def _run(press_pose, n_clear=40, near_at=None, shims=None, stool_dx=(0.0, 0.0), engage=True):
    """One trial offline: an arm within 2 cm at step 0 (engage; the presses wait for it), then arms clear (or `near_at`
    steps within 2 cm), the stool resting (moved by stool_dx from step 10); during each press, the stool takes
    press_pose(corner). Returns the score after the presses."""
    if engage:
        near_at = {0} | set(near_at or ())
    t._L = None
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    t.LINK_POSES.clear()
    scores = []
    for n in range(n_clear + 4 * t.PRESS_STEPS + 6):
        kit._EPISODE["last"] = n
        L = t._L
        corner = L.force[0] if (L is not None and L.force is not None) else None
        R = press_pose(corner) if corner is not None else None
        objs = _objs(R, stool_dx if n >= 10 else (0.0, 0.0), shims)
        if near_at is not None and n in near_at:
            t._update.__globals__["arms_clear"] = lambda o: False
        else:
            t._update.__globals__["arms_clear"] = _REAL_CLEAR
        scores.append(t.score(objs))
    t._update.__globals__["arms_clear"] = _REAL_CLEAR
    return scores


_REAL_CLEAR = t.arms_clear


def _rock_model(gap):
    """The partner's corner pressed: nothing moves (the stool stands level on its three long legs). The short leg's
    corner pressed: the stool turned about the neighbors' diagonal until the short foot is `gap` m lower (the gap a
    shim leaves under it)."""
    f = t.foot_points()
    u = f[(t.SHORT + 3) % 4] - f[(t.SHORT + 1) % 4]
    best = None
    for sign in (1, -1):
        for ang in np.linspace(0, 0.04, 8001):
            R = t._axis_rot(u, sign * ang)
            if abs(f[t.SHORT, 2] - (f @ R.T)[t.SHORT, 2] - gap) < 5e-6:
                best = R
                break
        if best is not None:
            break

    def pose(corner):
        return best if corner == t.SHORT else t.STOOL_R0
    return pose


def test_rest_pose_every_seed():
    """Level on the three long legs, the short foot 4 mm up (flat feet: the stool's middle lies over the far edges of
    the neighbors' feet only when tipped further than 4 mm allows, so it rests level until pressed)."""
    for short in range(4):
        z = t.foot_points(short)
        assert z[short, 2] == pytest.approx(0.004)
        assert all(z[k, 2] == 0.0 for k in range(4) if k != short)


def test_quaternion_order():
    R = t._axis_rot((1.0, 2.0, 0.5), 0.7)
    assert np.allclose(t.rotm(t._quat(R)), R, atol=1e-9)
    q = t._quat(R)
    assert np.allclose(t.rotm((q[0], q[1], q[2], q[3])), R)  # (x, y, z, w), as Isaac Lab and yam_arena give it


def test_start_scores_zero():
    s = _run(_rock_model(0.004))
    assert max(s) == 0.0
    assert t._L.result is not None and t._L.result[1] == pytest.approx(0.004, abs=2e-4)


def test_no_rock_full_score():
    s = _run(lambda c: t.STOOL_R0)
    assert s[-1] == 1.0


def test_half_rock_half_score():
    assert _run(_rock_model(0.0015))[-1] == 0.5


def test_presses_wait_for_hands_off():
    s = _run(lambda c: t.STOOL_R0, n_clear=30)  # hands off for only 1.5 s before the presses would be due
    assert max(s[:40 + 4 * t.PRESS_STEPS - 1]) == 0.0
    s = _run(lambda c: t.STOOL_R0, n_clear=40, near_at=set(range(0, 20)))
    assert max(s) == 0.0  # the clock restarts at step 20: the presses are not done in time


def test_hand_back_on_the_stool_voids_result():
    t._L = None
    s = _run(lambda c: t.STOOL_R0, n_clear=40 + 4 * t.PRESS_STEPS + 5)
    assert s[-1] == 1.0
    s = _run(lambda c: t.STOOL_R0, near_at={40 + 4 * t.PRESS_STEPS + 2})
    assert s[-1] == 0.0


def test_moved_stool_scores_zero():
    assert _run(lambda c: t.STOOL_R0, stool_dx=(0.025, 0.0))[-1] == 0.0
    assert _run(lambda c: t.STOOL_R0, stool_dx=(0.015, 0.0))[-1] == 1.0


def test_shim_under_rocking_foot_quarter():
    p = t.SHORT
    side = 1 if t.CORNERS[p][1] > 0 else -1
    name = "shim_left" if side > 0 else "shim_right"
    shims = {name: (tuple(t.shim_target(p, 0.003)), side)}  # 3 mm under the edge: 20 mm in, 0.6 of the foot
    assert _run(_rock_model(0.0025), shims=shims)[-1] == 0.25
    shims = {name: (tuple(t.shim_target(p, 0.0015)), side)}  # only its thin tip under the edge: no credit
    assert _run(_rock_model(0.0035), shims=shims)[-1] == 0.0
    # under a foot that always bears: 0
    wrong = (p + 1) % 4 if (1 if t.CORNERS[(p + 1) % 4][1] > 0 else -1) == side else (p + 3) % 4
    shims = {name: (tuple(t.shim_target(wrong, 0.004)), side)}
    assert _run(_rock_model(0.004), shims=shims)[-1] == 0.0


def test_shim_under_geometry():
    o = _objs()
    assert not any(t.shim_under(o, i) for i in range(4))  # the shims start beside the stool
    for leg in range(4):
        side = 1 if t.CORNERS[leg][1] > 0 else -1
        name = "shim_left" if side > 0 else "shim_right"
        o = _objs(stool_R=np.eye(3), shims={name: (tuple(t.shim_target(leg)), side)})
        assert t.shim_under(o, leg)
        assert sum(t.shim_under(o, i) for i in range(4)) == 1


def test_snug_target_thickness():
    for leg in range(4):
        sx, sy = t.CORNERS[leg]
        c = t.shim_target(leg)
        thin = c[1] - sy * t.SHIM_L / 2
        edge = t.STOOL_XY[1] + sy * (t.FOOT_C + t.LEG_W / 2)
        assert t.shim_thickness(abs(edge - thin)) == pytest.approx(t.SNUG_T, abs=1e-6)


def test_press_force_inside_footprint():
    """The scorer's press points lie inside the feet's outline, so a press cannot tip the stool over."""
    assert t.PRESS_IN < t.FOOT_C - t.LEG_W / 2


def test_demo_fits_run_limit_every_variant():
    for v in ("", "nothing", "pressonly", "wrong", "deep", "hold"):
        os.environ["STOOL_VARIANT"] = v
        try:
            d = t.demo()
        finally:
            os.environ.pop("STOOL_VARIANT", None)
        assert set(d) == {"left", "right"}
        assert max(t._t(d["left"]), t._t(d["right"])) * 20 < t.RUN_LIMIT_STEPS


def test_no_pxr_at_import_or_demo():
    code = ("import sys; sys.path.insert(0, %r); import task; task.demo(); "
            "assert 'pxr' not in sys.modules and 'isaaclab' not in sys.modules; print('ok')") % HERE
    out = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, timeout=120)
    assert out.returncode == 0 and "ok" in out.stdout, out.stderr[-2000:]


def test_mass_and_sizes():
    assert t.STOOL_KG == pytest.approx(6.6 * 0.4536, abs=0.01)
    assert t.STOOL_H == pytest.approx(12.25 * 0.0254, abs=5e-4)
    assert t.FOOT_SPAN == pytest.approx(12.75 * 0.0254, abs=5e-4)
    assert t.SHIM_L == pytest.approx(1.92 * 0.0254, abs=2e-4)
    assert t.SHIM_W == pytest.approx(1.11 * 0.0254, abs=2e-4)
    assert t.SHIM_T0 == pytest.approx(0.03 * 0.0254, abs=2e-5)
    assert t.SHIM_T1 == pytest.approx(0.25 * 0.0254, abs=2e-5)
    assert math.degrees(math.atan(t.SHIM_SLOPE)) == pytest.approx(6.45, abs=0.15)


def _tip_at(point):
    """LINK_POSES with one fingertip (tip_left, unrotated) whose lowest hull point sits at `point`."""
    pts = t.yg.points("tip_left")
    low = pts[np.argmin(pts[:, 2])]
    return {"left_robot/tip_left": (np.asarray(point, float) - low, np.eye(3))}


def test_box_check_with_real_fingertip_hull():
    o = _objs(stool_R=np.eye(3))
    corner = np.array([t.STOOL_POS[0] + t.PRESS_IN, t.PRESS_IN, t.STOOL_POS[2] + t.STOOL_H + 0.0005])
    t.LINK_POSES.clear()
    t.LINK_POSES.update(_tip_at(corner))
    assert not t.arms_clear(o)  # a fingertip resting on the seat
    t.LINK_POSES.clear()
    t.LINK_POSES.update(_tip_at(corner + np.array([0, 0, 0.025])))
    assert t.arms_clear(o)  # 2.5 cm above it
    t.LINK_POSES.clear()


def test_propped_or_held_shim_is_not_hands_off():
    """A shim stood on end on the seat corner (pressing it down) keeps the presses from starting, held or not; a held
    shim braced flat against a foot does too; a shim left lying flat under a foot does not."""
    up = np.array([[0, 0, 1.0], [0, 1.0, 0], [-1.0, 0, 0]])  # its length vertical
    o = _objs()
    corner = t.press_point(o, t.SHORT)
    o["shim_left"] = _st(corner + np.array([0, 0, t.SHIM_L / 2]), up)
    t.LINK_POSES.clear()
    assert not t.arms_clear(o)
    p = t.SHORT
    side = 1 if t.CORNERS[p][1] > 0 else -1
    name = "shim_left" if side > 0 else "shim_right"
    o = _objs(shims={name: (tuple(t.shim_target(p)), side)})
    assert t.arms_clear(o)
    sh = o[name]
    grip = np.asarray(sh.pos, float) + t.rotm(sh.quat_xyzw) @ np.array([t.SHIM_L / 2 - 0.002, 0, t.SHIM_T1 + 0.001])
    t.LINK_POSES.update(_tip_at(grip))
    assert not t.arms_clear(o)
    t.LINK_POSES.clear()
    assert _run(lambda c: t.STOOL_R0, shims=None)[-1] == 1.0


def test_no_presses_before_the_robot_engages():
    """Arms clear all along and no shim near the stool: the scorer never presses (no free diagnosis), score 0."""
    s = _run(lambda c: t.STOOL_R0, engage=False)
    assert s[-1] == 0.0 and t._L.result is None and t._L.i is None


def test_spawn_off_its_fixed_start_stops_the_trial():
    """The moved check needs a fixed layout: a stool spawned 1 cm off (object jitter on) raises at step 5."""
    t._L = None
    kit._EPISODE["count"] += 1
    t.LINK_POSES.clear()
    with pytest.raises(RuntimeError, match="object_jitter=0"):
        for n in range(8):
            kit._EPISODE["last"] = n
            t.score(_objs(None, (0.01, 0.0), None))


if __name__ == "__main__":
    import sys

    rc = pytest.main([__file__, "-q", "-p", "no:cacheprovider"])
    print("all pass" if rc == 0 else "FAIL")
    sys.exit(rc)
