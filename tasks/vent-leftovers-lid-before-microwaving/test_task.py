"""Offline checks of the leftovers task's scorer and layout (no simulator): python -m pytest test_task.py"""
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import task as t  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

Z3 = np.zeros(3)


def _st(pos, R=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(_quat(R)), lin_vel=Z3, ang_vel=Z3, low=p, high=p)


def _quat(R):
    R = np.asarray(R, float)
    w = math.sqrt(max(1e-12, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    return ((R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


COUNTER = np.array([t.BOX_XY[0], t.BOX_XY[1], t.TOP])
INSIDE = np.array([t.PLACE_XY[0], t.PLACE_XY[1], t.TOP + t.TT_TOP])
OPEN = (90.0, 90.0, 90.0, 90.0)
SHUT = (0.0, 0.0, 0.0, 0.0)


def frame(box=COUNTER, box_R=np.eye(3), lid="on", tabs=SHUT, door=t.DOOR_START_DEG, key=0.0, lid_shift=(0.0, 0.0)):
    """One control step's objects: the container at `box`, the lid on it (shifted), beside it ("off") or as given,
    the tabs' angles (degrees open, in SIDES order), the door's angle open and the Start pad's travel."""
    b = np.asarray(box, float)
    if lid == "on":
        lp = b + box_R @ np.array([lid_shift[0], lid_shift[1], t.GL_H])
    else:
        lp = np.array([0.25, -0.30, t.TOP + 0.01])
    joints = {f"lid/{s.name}": s.sign * math.radians(d) for s, d in zip(t.SIDES, tabs)}
    joints["microwave/door"] = -math.radians(door)
    joints["microwave/start"] = key
    f = {"leftovers": _st(b, box_R), "lid": _st(lp, box_R), "_joints": joints}
    _set(f)
    return f


def _set(f):
    for k, q in f["_joints"].items():
        kit._JOINTS[k] = kit.JointState(pos=q, vel=0.0, latched=False)


def run(frames):
    t._L = None
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    out = []
    for i, f in enumerate(frames):
        kit._EPISODE["last"] = i
        _set(f)
        out.append(t.score(f))
    return out


PRESS = t.KEY_TRAVEL
SAFE = [frame(), frame(tabs=OPEN), frame(tabs=OPEN, lid="off"), frame(box=INSIDE, lid="off", tabs=OPEN),
        frame(box=INSIDE, lid="off", tabs=OPEN, door=0.0), frame(box=INSIDE, lid="off", tabs=OPEN, door=0.0, key=PRESS),
        frame(box=INSIDE, lid="off", tabs=OPEN, door=0.0)]


def test_start_scores_zero():
    assert run([frame()]) == [0.0]


def test_nothing_done_scores_zero():
    assert run([frame()] * 5)[-1] == 0.0


def test_safe_sequence_tiers():
    s = run(SAFE)
    assert s == [0.0, 0.0, 0.25, 0.25, 0.5, 1.0, 1.0]


def test_heated_latched_scores_zero_and_stays():
    s = run([frame(box=INSIDE, door=0.0), frame(box=INSIDE, door=0.0, key=PRESS), frame(box=INSIDE, door=0.0),
             frame(box=INSIDE, lid="off", tabs=OPEN, door=0.0), frame(box=INSIDE, lid="off", tabs=OPEN, door=0.0, key=PRESS)])
    assert s[-1] == 0.0 and s[1] == 0.0


def test_partly_unlatched_heated_scores_zero():
    for k in range(4):
        tabs = list(OPEN)
        tabs[k] = 5.0
        s = run([frame(box=INSIDE, tabs=tabs, door=0.0, lid_shift=(0.0, 0.015)),
                 frame(box=INSIDE, tabs=tabs, door=0.0, key=PRESS, lid_shift=(0.0, 0.015))])
        assert s[-1] == 0.0, (k, s)


def test_tab_under_open_threshold_counts_latched():
    tabs = list(OPEN)
    tabs[2] = t.OPEN_DEG - 1.0
    s = run([frame(box=INSIDE, tabs=tabs, door=0.0, lid_shift=(0.015, 0.0), key=PRESS)])
    assert s == [0.0]


def test_unlatched_but_seated_heated_scores_quarter():
    s = run([frame(box=INSIDE, tabs=OPEN, door=0.0), frame(box=INSIDE, tabs=OPEN, door=0.0, key=PRESS)])
    assert s[-1] == 0.25


def test_unlatched_and_offset_heated_scores_one():
    s = run([frame(box=INSIDE, tabs=OPEN, door=0.0, lid_shift=(0.0, 0.010)),
             frame(box=INSIDE, tabs=OPEN, door=0.0, key=PRESS, lid_shift=(0.0, 0.010))])
    assert s[-1] == 1.0


def test_small_offset_is_not_vented():
    s = run([frame(box=INSIDE, tabs=OPEN, door=0.0, key=PRESS, lid_shift=(0.0, 0.005))])
    assert s == [0.25]


def test_start_with_door_open_does_not_heat():
    s = run([frame(box=INSIDE, lid="off", tabs=OPEN, door=t.DOOR_START_DEG, key=PRESS),
             frame(box=INSIDE, lid="off", tabs=OPEN, door=t.DOOR_START_DEG)])
    assert s == [0.25, 0.25]


def test_door_nearly_shut_is_not_closed():
    s = run([frame(box=INSIDE, lid="off", tabs=OPEN, door=t.CLOSED_DEG + 0.2, key=PRESS)])
    assert s == [0.25]


def test_key_held_from_before_door_shut_does_not_count():
    s = run([frame(box=INSIDE, lid="off", tabs=OPEN, door=5.0, key=PRESS),
             frame(box=INSIDE, lid="off", tabs=OPEN, door=0.0, key=PRESS)])
    assert s[-1] == 0.5


def test_half_pressed_key_does_not_start():
    s = run([frame(box=INSIDE, lid="off", tabs=OPEN, door=0.0, key=t.KEY_PRESSED - 0.0002)])
    assert s == [0.5]


def test_empty_run_scores_zero():
    s = run([frame(lid="off", tabs=OPEN, door=0.0), frame(lid="off", tabs=OPEN, door=0.0, key=PRESS),
             frame(box=INSIDE, lid="off", tabs=OPEN, door=0.0), frame(box=INSIDE, lid="off", tabs=OPEN, door=0.0, key=PRESS)])
    assert s[1] == 0.0 and s[-1] == 0.0


def test_spill_latches_zero():
    s = run([frame(lid="off", tabs=OPEN), frame(box=COUNTER + [0, 0, 0.05], box_R=_rx(math.radians(35)), lid="off",
                                                    tabs=OPEN)] + SAFE[3:])
    assert s[-1] == 0.0


def test_tilted_carry_under_spill_limit_is_fine():
    s = run([frame(lid="off", tabs=OPEN), frame(box=COUNTER + [0, 0, 0.05], box_R=_rx(math.radians(20)), lid="off",
                                                    tabs=OPEN)] + SAFE[3:])
    assert s[-1] == 1.0


def test_container_off_the_counter_latches_zero():
    s = run([frame(box=COUNTER - [0, 0, 0.05], lid="off", tabs=OPEN)] + SAFE[3:])
    assert s[-1] == 0.0


def test_container_half_out_of_the_cavity_is_not_inside():
    half_out = INSIDE.copy()
    half_out[0] = t.MW_X0 + t.DOOR_T + t.GL_X / 2 - 0.02
    assert not t.in_cavity(frame(box=half_out, lid="off", tabs=OPEN))
    assert t.in_cavity(frame(box=INSIDE, lid="off", tabs=OPEN))


def test_container_hovering_over_the_turntable_is_not_set_down():
    up = INSIDE + [0, 0, t.ON_GAP + 0.002]
    assert not t.in_cavity(frame(box=up, lid="off", tabs=OPEN))


def test_tilted_in_cavity_is_not_set_down():
    assert not t.in_cavity(frame(box=INSIDE + [0, 0, 0.01], box_R=_rx(math.radians(15)), lid="off", tabs=OPEN))


def test_lid_off_needs_distance():
    f = frame(box=INSIDE, tabs=SHUT, lid="on", lid_shift=(0.0, 0.05))
    assert not t.vented(f) and t.latched_tabs(f) == 4
    assert t.latched_tabs(frame(box=INSIDE, tabs=SHUT)) == 4


# -- layout -------------------------------------------------------------------------------------------------------------
def test_container_fits_cavity_and_turntable():
    assert t.GL_Y < t.CAV_Y1 - t.CAV_Y0 and t.GL_X < t.CAV_X1 - t.CAV_X0
    corners = [(sx * t.GL_X / 2, sy * t.GL_Y / 2) for sx in (-1, 1) for sy in (-1, 1)]
    for cx, cy in corners:
        assert math.hypot(t.PLACE_XY[0] + cx - t.TT_XY[0], t.PLACE_XY[1] + cy - t.TT_XY[1]) < t.TT_R + 0.03
    assert math.hypot(t.PLACE_XY[0] - t.TT_XY[0], t.PLACE_XY[1] - t.TT_XY[1]) < t.TT_R - 0.05


def test_published_sizes():
    assert abs((t.MW_X1 - t.MW_X0) - 0.324) < 0.001  # 12.75 in deep
    assert abs((t.MW_Y1 - t.MW_Y0) - 0.440) < 0.001  # 17.31 in wide
    assert abs(t.MW_H - 0.259) < 0.001  # 10.19 in high
    assert abs((t.CAV_X1 - t.CAV_X0) - 0.295) < 0.001  # 11.63 in
    assert abs((t.CAV_Y1 - t.CAV_Y0) - 0.311) < 0.001  # 12.25 in
    assert abs((t.CAV_Z1 - t.CAV_Z0) - 0.200) < 0.001  # 7.88 in
    assert abs(2 * t.TT_R - 0.273) < 0.001  # 10.75 in
    real_ledge = 0.0045  # the shut tabs' real ledges, outside the bead (the sim widens them)
    assert abs(t.GL_Y + 2 * (t.HINGE_OUT + real_ledge) - 0.178) < 0.003
    assert abs(t.GL_X + 2 * (t.HINGE_OUT + real_ledge) - 0.127) < 0.003 and abs(t.GL_H + t.LID_T - 0.0635) < 0.001
    assert abs(t.GLASS_KG + t.LID_KG + 4 * t.TAB_KG - 0.459) < 0.001  # 16.2 oz with the lid


def test_container_inside_cavity_when_placed():
    assert t.PLACE_XY[0] - t.GL_X / 2 > t.MW_X0 + t.DOOR_T + 0.01
    assert t.CAV_Y0 + 0.03 < t.PLACE_XY[1] - t.GL_Y / 2 and t.PLACE_XY[1] + t.GL_Y / 2 < t.CAV_Y1 - 0.03


def test_microwave_on_the_counter():
    assert t.MW_X1 <= 0.7501 and t.MW_Y0 >= -0.6 and t.MW_Y1 <= 0.6


def test_hooks_catch_under_the_bead():
    for s in t.SIDES:
        # the hook reaches inside the bead's outer face, under the bead
        assert t.HOOK_IN - t.HINGE_OUT > 0.001
        assert t.HOOK_Z[1] + t.HINGE_Z < -t.BEAD_H + 0.0001


def test_tab_spring_snap():
    for s in t.SIDES:
        law = t.TabSpring(s.sign)
        tgt, k, _, _ = law.drive("", s.sign * math.radians(10), 0, {}, {})
        assert s.sign * tgt < 0 and k == t.TAB_K
        tgt, k, _, _ = law.drive("", s.sign * math.radians(40), 0, {}, {})
        assert s.sign * tgt > math.radians(90) and k == t.TAB_K_OPEN


def test_door_hinge_has_no_pull_in():
    law = t.DoorHinge()
    for deg in (0.3, 1.0, 1.5, 3.0, 45.0):
        q = -math.radians(deg)
        tgt, k, _, _ = law.drive("", q, 0.0, {}, {})
        assert tgt == q and k == t.DOOR_HOLD


def test_snap_needs_more_than_the_tipping_push():
    # straight-up push under the ledge's middle at the snap angle vs the push at the rim that tips the container
    th = math.radians(t.SNAP_DEG)
    mid = (t.TAB_T / 2 + t.LIP_OUT) / 2
    lever = mid * math.cos(th) + (t.TAB_DROP - t.LIP_H / 2) * math.sin(th)
    snap_push = t.TAB_SNAP_TORQUE / lever
    weight = (t.BOX_KG + t.LID_KG + 4 * t.TAB_KG) * 9.81
    tip_push = weight * 0.5  # a push up at one rim edge tips it about the opposite bottom edge
    assert snap_push > 1.4 * tip_push


def test_grip_angles_roundtrip():
    import yam_arena.demo as yd
    for s in t.SIDES:
        tcp, f, c, n = t.lever_frame(s)
        a = t.grip_angles(f, c)
        R = kit._rot_xyz(a["roll"], a["pitch"]) @ yd.grip_rotation(a["yaw"])
        assert np.allclose(R[:, 2], f / np.linalg.norm(f), atol=1e-6)
        assert np.allclose(R[:, 0], c / np.linalg.norm(c), atol=1e-6)


def test_variants_have_plans():
    for v in ("",) + t.VARIANTS:
        os.environ["LEFTOVERS_VARIANT"] = v
        d = t.demo()
        assert abs(t._t(d["left"]) - t._t(d["right"])) < 1e-6
        assert t._t(d["left"]) / t.DT < t.RUN_LIMIT_STEPS
    os.environ["LEFTOVERS_VARIANT"] = ""
