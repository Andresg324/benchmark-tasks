"""Offline checks of the disposal task's scorer and layout (no simulator): python -m pytest test_task.py"""
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
    return ObjectState(pos=p, quat_xyzw=np.asarray(t._quat(np.asarray(R, float))), lin_vel=Z3, ang_vel=Z3, low=p, high=p)


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


TONGS_REST = (np.array(t.TONGS_START), t.TONGS_TILT)
HEADS_DOWN = np.array([[0.0, 0.0, -1.0], [0.0, -1.0, 0.0], [-1.0, 0.0, 0.0]])  # the tongs held heads down
TONGS_UP = (np.array([0.30, -0.05, t.DECK_Z + 0.40]), HEADS_DOWN)  # held up, clear of everything


def _flat_spoon(x, y):
    """The spoon lying flat, bowl down, on the deck, its bowl's tip at (x, y) and its handle toward +x."""
    R = np.eye(3)
    p = t.SPOON_POINTS @ R.T
    z = t.DECK_Z - p[:, 2].min() + 0.0003
    return np.array([x, y, z]), R


ON = math.radians(t.ROCKER_TRAVEL_DEG)
_ROCK = {"q": 0.0}  # the rocker's angle carried from frame to frame (it stays where it was flipped)


def _objs(spoon, tongs=TONGS_UP, rock=None, q=0.0):
    if rock is not None:
        _ROCK["q"] = rock
    kit._JOINTS["sink/rocker"] = kit.JointState(pos=_ROCK["q"], vel=0.0, latched=False)
    kit._JOINTS["tongs/arm_b"] = kit.JointState(pos=q, vel=0.0, latched=False)
    tp, tR = tongs
    return {"spoon": _st(*spoon), "tongs": _st(tp, tR), "tongs/arm_b": _st(tp, tR @ _rz(q)),
            "sink/rocker": _st((0, 0, 0))}


START = (t.SPOON_POS, t.SPOON_R)
ASIDE = _flat_spoon(0.29, -0.105)


def _run(frames):
    """Score a sequence of frames (each a dict of objects, or a function making one) as one trial."""
    t._L = None
    kit._EPISODE["count"] += 1
    t.LINK_POSES.clear()
    _ROCK.clear()
    _ROCK["q"] = 0.0
    s = None
    keep = []  # every frame's dict stays alive, so no two frames share an id (the ledger's step key offline)
    for f in frames:
        if callable(f):
            f = f()
        keep.append(f)
        s = t.score(f)
    return s


def _press_seq(spoon, tongs=TONGS_UP):
    """Flip the rocker over in three steps: on if it was off, off if it was on (it then stays there)."""
    def flip(k):
        def f():
            q0 = _ROCK.setdefault("from", _ROCK["q"]) if k == 0 else _ROCK["from"]
            goal = 0.0 if q0 > ON / 2 else ON
            q = q0 + (goal - q0) * (k + 1) / 3
            if k == 2:
                _ROCK.pop("from")
            return _objs(spoon, tongs, rock=q)
        return f
    return [flip(0), flip(1), flip(2)]


def test_spoon_start_is_in_the_drain_with_its_end_up_through_the_middle_hole():
    p = t.spoon_world(_objs(START))
    r = np.hypot(p[:, 0] - t.DRAIN_XY[0], p[:, 1] - t.DRAIN_XY[1])
    assert t.DECK_Z + 0.010 < p[:, 2].max() < t.DECK_Z + 0.025  # the handle's end stands 1 to 2.5 cm over the floor
    assert p[:, 2].min() >= t.DECK_Z + t.PLATE_Z - 1e-6
    assert p[:, 2].min() <= t.DECK_Z + t.PLATE_Z + 0.001  # the bowl on the grind plate
    assert r.max() < t.CHAMBER_R
    up = p[p[:, 2] > t.DECK_Z + t.FLAP_TOP - t.FLAP_T]  # the part at the flaps' level and above
    assert np.hypot(up[:, 0] - t.DRAIN_XY[0], up[:, 1] - t.DRAIN_XY[1]).max() < 0.0115  # through the middle hole
    assert t.spoon_in(_objs(START))


def test_nothing_scores_zero():
    assert _run([lambda: _objs(START)] * 40) == 0.0


def test_set_aside_switch_off_is_half():
    assert _run([lambda: _objs(ASIDE)] * 40) == 0.5


def test_set_aside_then_switch_on_held_is_one():
    frames = [lambda: _objs(ASIDE)] * 3 + _press_seq(ASIDE) + [lambda: _objs(ASIDE)] * (t.HOLD_STEPS + 2)
    assert _run(frames) == 1.0


def test_on_but_not_held_long_enough_is_half():
    frames = [lambda: _objs(ASIDE)] * 3 + _press_seq(ASIDE) + [lambda: _objs(ASIDE)] * 5
    assert _run(frames) == 0.5


def test_switch_on_with_spoon_in_latches_zero():
    frames = _press_seq(START) + [lambda: _objs(ASIDE)] * 40
    assert _run(frames) == 0.0


def test_switch_on_then_off_with_spoon_in_still_zero():
    frames = _press_seq(START) + _press_seq(START) + [lambda: _objs(ASIDE)] * 3 + _press_seq(ASIDE) + \
        [lambda: _objs(ASIDE)] * 40
    assert _run(frames) == 0.0


def test_on_off_twice_after_aside_is_half():
    frames = [lambda: _objs(ASIDE)] * 3 + _press_seq(ASIDE) + _press_seq(ASIDE) + [lambda: _objs(ASIDE)] * 40
    assert _run(frames) == 0.5


def test_rocker_pushed_short_of_its_center_is_not_on():
    part = math.radians(t.ROCKER_CENTER_DEG - 0.5)
    frames = [lambda: _objs(ASIDE)] * 3 + [lambda: _objs(ASIDE, rock=part)] * 5 + [lambda: _objs(ASIDE, rock=0.0)] * 40
    assert _run(frames) == 0.5


def test_spoon_dropped_back_in_while_on_latches_zero():
    frames = [lambda: _objs(ASIDE)] * 3 + _press_seq(ASIDE) + [lambda: _objs(START)] * 2 + \
        [lambda: _objs(ASIDE)] * 40
    assert _run(frames) == 0.0


def test_tongs_in_the_drain_while_on_latches_zero():
    tongs_in = (np.array([t.DRAIN_XY[0] + 0.01, t.DRAIN_XY[1], t.DECK_Z + 0.25]),
                np.array([[0.0, 0.0, -1.0], [0.0, -1.0, 0.0], [-1.0, 0.0, 0.0]]))  # heads down into the throat
    assert t.in_disposal(t.tongs_world(_objs(ASIDE, tongs_in)))
    frames = [lambda: _objs(ASIDE)] * 3 + _press_seq(ASIDE, tongs_in) + [lambda: _objs(ASIDE)] * 40
    assert _run(frames) == 0.0


def test_spoon_held_out_in_the_tongs_is_quarter():
    sp = (np.array([0.30, -0.15, t.DECK_Z + 0.06]), START[1])
    tongs = (sp[0] + sp[1] @ np.array([0.12, 0.0, 0.0]) + np.array([0.0, 0.0, 0.175]),
             np.array([[0.0, 0.0, -1.0], [0.0, -1.0, 0.0], [-1.0, 0.0, 0.0]]))
    assert not t.spoon_in(_objs(sp))
    assert _run([lambda: _objs(sp, tongs)] * 40) == 0.25


def test_spoon_hovering_over_the_deck_is_not_aside():
    hover = (ASIDE[0] + np.array([0.0, 0.0, 0.010]), ASIDE[1])
    assert not t.set_aside(_objs(hover))
    assert _run([lambda: _objs(hover)] * 40) == 0.25


def test_spoon_next_to_the_flange_is_not_aside():
    near = _flat_spoon(t.DRAIN_XY[0] + t.LIP_R_OUT + 0.005, t.DRAIN_XY[1])
    assert not t.set_aside(_objs(near))
    assert _run([lambda: _objs(near)] * 40) == 0.25


def test_spoon_under_the_tongs_is_not_let_go():
    sp = ASIDE
    p = t.spoon_world(_objs(sp))
    tongs = (np.array([p[:, 0].mean() + 0.10, sp[0][1], p[:, 2].max() + 0.001 + t.HEAD_D / 2]), _rz(math.pi))
    o = _objs(sp, tongs)
    assert t.tongs_gap(o) <= t.CLEAR
    assert _run([lambda: _objs(sp, tongs)] * 40) == 0.25


def test_tongs_at_start_far_from_the_spoon_and_out_of_the_drain():
    o = _objs(START, TONGS_REST)
    assert t.tongs_gap(o) > 0.02
    assert not t.in_disposal(t.tongs_world(o))


def test_spoon_gap_box():
    assert t.spoon_gap(np.array([[0.1, 0.0, 0.001]]))[0] == 0.0
    assert abs(t.spoon_gap(np.array([[0.1, 0.0, 0.001 + t.HANDLE_T_SP / 2 + 0.005]]))[0] - 0.005) < 1e-9


def _gripper_at(tcp, R):
    """LINK_POSES for the right arm posed (IK, then the URDF's forward kinematics) with its tool point at `tcp`."""
    import yam_arena.analysis as A
    import yam_arena.demo as demo
    from yam_arena.station import BASES
    q, perr, _ = demo.ik.solve(np.asarray(tcp, float), t._quat(np.asarray(R)), q_init=np.array(demo.READY),
                               base_pos=BASES["right"], iters=400)
    assert perr < 0.002
    qd = {f"joint{i + 1}": float(q[i]) for i in range(6)}
    qd["joint7"] = qd["joint8"] = 0.0
    P = A.fk_link_poses_world(qd, BASES["right"])
    return {f"right_robot/{k}": (P[k][:3, 3], P[k][:3, :3]) for k in t.ARM_LINKS}


def test_hand_into_the_drain_latches_zero():
    import yam_arena.demo as demo
    R = kit._rot_xyz(0.0, -0.5) @ demo.grip_rotation(0.0)  # fingers pointing down and a little forward
    x, y = t.DRAIN_XY[0] - 0.005, t.DRAIN_XY[1]
    inside = _gripper_at((x, y, t.DECK_Z + 0.004), R)  # the fingertips (1.3 cm past the tool point) in the throat
    above = _gripper_at((x, y, t.DECK_Z + 0.030), R)
    t.LINK_POSES.clear()
    t.LINK_POSES.update(above)
    assert not t.hand_in_drain()
    t.LINK_POSES.clear()
    t.LINK_POSES.update(inside)
    assert t.hand_in_drain()
    t._L = None
    kit._EPISODE["count"] += 1
    t.LINK_POSES.clear()
    t.score(_objs(START))
    t.LINK_POSES.update(inside)
    t.score(_objs(START))
    t.LINK_POSES.clear()
    s = None
    for _ in range(3):
        t.score(_objs(ASIDE))
    for f in _press_seq(ASIDE):
        t.score(f())
    for _ in range(40):
        s = t.score(_objs(ASIDE))
    assert s == 0.0
    t.LINK_POSES.clear()


def test_pinching_the_end_above_the_floor_is_allowed():
    """The other safe way: a bare gripper pinching the handle's end where it stands above the floor (fingertips about
    2 mm over it, tipped 17 degrees forward, where the settled handle stands) is not an arm in the drain; drawing the
    spoon out, setting it aside and switching on then scores 1.0. 6 mm lower, its fingertips are in the drain: 0."""
    import yam_arena.demo as demo
    R = kit._rot_xyz(0.0, -0.3) @ demo.grip_rotation(0.0)
    x, y = t.DRAIN_XY[0] + 0.002, t.DRAIN_XY[1] + t.SETTLE_Y
    pinch = _gripper_at((x, y, t.DECK_Z + 0.016), R)
    low = _gripper_at((x, y, t.DECK_Z + 0.010), R)
    t.LINK_POSES.clear()
    t.LINK_POSES.update(pinch)
    assert not t.hand_in_drain()
    t.LINK_POSES.clear()
    t.LINK_POSES.update(low)
    assert t.hand_in_drain()
    def posed(spoon, links):
        def f():
            t.LINK_POSES.clear()
            t.LINK_POSES.update(links)
            return _objs(spoon)
        return f
    frames = ([posed(START, pinch)] * 5 + [posed(ASIDE, {})] * 3 + _press_seq(ASIDE)
              + [lambda: _objs(ASIDE)] * (t.HOLD_STEPS + 2))
    s = _run(frames)
    assert s == 1.0
    t.LINK_POSES.clear()


def test_demo_is_open_loop_and_variants_build():
    for v in ("",) + t.VARIANTS:
        os.environ["DISPOSAL_VARIANT"] = v
        d = t.demo()
        assert set(d) == {"left", "right"}
        assert all(w.duration > 0 for side in d.values() for w in side)
    os.environ["DISPOSAL_VARIANT"] = ""
    src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "task.py")).read()
    body = src[src.index("def demo():"):]
    assert "objects" not in body.split("PRESS_PAST")[0]  # the demo reads no simulated poses


def test_demo_fits_the_run_limit():
    os.environ["DISPOSAL_VARIANT"] = ""
    d = t.demo()
    assert max(sum(w.duration for w in s) for s in d.values()) / t.DT < t.RUN_LIMIT_STEPS


def test_usd_has_hull_colliders():
    for m in (t.SINK, t.TONGS):
        usd = kit._mech_usda(m)
        n = sum(1 for c in list(m.colliders) + [c for lk in m.links for c in lk.colliders] if c.hull is not None)
        assert usd.count('physics:approximation = "convexHull"') == n
        assert "radius = 0.0\n" not in usd
    for o in t.OBJECTS:
        assert len(o.meshes) > 0


def test_quat_round_trip():
    rng = np.random.default_rng(0)
    mats = [np.eye(3), _rz(math.pi), np.array([[0.0, 0.0, -1.0], [0.0, -1.0, 0.0], [-1.0, 0.0, 0.0]])]
    for _ in range(50):
        q, _ = np.linalg.qr(rng.normal(size=(3, 3)))
        mats.append(q * np.sign(np.linalg.det(q)))
    for R in mats:
        assert np.abs(t.rotm(t._quat(R)) - R).max() < 1e-9


def test_spoon_lean_matches_the_text():
    assert abs(t.SPOON_LEAN_DEG - 80.0) < 0.5  # README / setup: "stands up at 80 degrees"


def test_spoon_held_mid_head_on_the_deck_is_not_let_go():
    """The spoon set down on the deck but still clamped in the middle of a head's face (far from any corner)."""
    sp = _flat_spoon(0.29, 0.105)  # on the bowl's left side: these tongs reach toward +y, away from the drain
    u = t.HEAD_MID_U
    face = np.array([u * math.cos(t.BETA), u * math.sin(t.BETA), 0.0])  # head A's inner face, middle
    R = HEADS_DOWN
    on_handle = sp[0] + sp[1] @ np.array([0.10, 0.0, 0.001 + t.HANDLE_T_SP / 2])
    tongs = (on_handle - R @ face, R)
    o = _objs(sp, tongs)
    corners = np.vstack([np.asarray(c.center) + np.asarray(c.hull) for _, c in t._A])
    cw = tongs[0] + corners @ R.T
    q = (cw - sp[0]) @ sp[1]
    assert t.spoon_gap(q).min() > 0.01  # a corner-only check would call it let go
    assert t.tongs_gap(o) <= t.CLEAR
    assert t.set_aside(o)
    frames = [lambda: _objs(sp, tongs)] * 3 + _press_seq(sp, tongs) + [lambda: _objs(sp, tongs)] * 40
    assert _run(frames) == 0.25


def test_rocker_snaps_over_center_and_starts_off():
    law = t.ROCKER.law
    lo, _, _, _ = law.drive("sink/rocker", 0.0, 0.0, {}, {})
    hi, _, _, _ = law.drive("sink/rocker", math.radians(t.ROCKER_CENTER_DEG + 0.1), 0.0, {}, {})
    assert lo < 0.0 < ON < hi  # each side's target lies past its own stop
    assert t.ROCKER.start == 0.0 and t.ROCKER.lower == 0.0 and abs(t.ROCKER.upper - ON) < 1e-12
    assert t.Ledger().on is False
    # about 4 N at the press point to tip it to its center
    f = t.ROCKER_K * math.radians(t.ROCKER_TILT_DEG + t.ROCKER_PRELOAD_DEG) / t.PRESS_U
    assert 3.5 < f < 4.5


def test_rocker_on_off_reading_has_hysteresis():
    mid = math.radians(t.ROCKER_CENTER_DEG)
    frames = [lambda: _objs(ASIDE)] * 3 + [lambda: _objs(ASIDE, rock=ON)] * 2 + [lambda: _objs(ASIDE, rock=mid)] + \
        [lambda: _objs(ASIDE)] * (t.HOLD_STEPS + 2)
    assert _run(frames) == 1.0  # still on at the center: it went back only to the middle


def test_switch_flipped_on_while_the_spoon_is_held_in_the_throat_is_zero():
    sp = (t.SPOON_POS + np.array([0.0, 0.0, 0.03]), t.SPOON_R)  # drawn 3 cm up, still in the drain
    assert t.spoon_in(_objs(sp))
    frames = [lambda: _objs(sp)] * 3 + _press_seq(sp) + [lambda: _objs(ASIDE)] * 40
    assert _run(frames) == 0.0


def test_demo_push_reaches_past_the_on_position_and_the_paddle_is_in_the_plate_opening():
    pushed = [w for w in t._press() if getattr(w, "label", "") == "push the top in (on)"][0]
    assert pushed.pos[0] + 0.0128 > t.rocker_face_x(t.ROCKER_TRAVEL_DEG)
    assert t.PADDLE[1] < t.OPENING_W and t.PADDLE[2] < t.OPENING_H
    assert t.ROCKER_Z + t.PRESS_U + 0.010 < t.ROCKER_Z + t.PADDLE[2] / 2 + 1e-9  # the push lands on the top half


def test_backsplash_switch_reachable_offline():
    import yam_arena.demo as d
    from yam_arena.station import BASES
    pushed = [w for w in t._press() if getattr(w, "label", "") == "push the top in (on)"][0]
    R = kit._rot_xyz(0.0, t.PITCH_FWD) @ d.grip_rotation(0.0)
    q, pe, re = d.ik.solve(np.asarray(pushed.pos), d.ik.matrix_to_quat_xyzw(R), np.array(d.READY, float),
                           base_pos=np.asarray(BASES["left"]), iters=200)
    assert pe < 0.002 and re < 0.02


def test_sink_is_a_recessed_bowl_with_its_rim_over_the_floor():
    assert abs(t.RIM_Z - t.DECK_Z - 0.130) < 1e-9  # 5-1/8 in from the floor to the rim's top
    assert abs((t.DECK_X[1] - t.DECK_X[0]) - 0.254) < 1e-3 and abs((t.DECK_Y[1] - t.DECK_Y[0]) - 0.305) < 1e-3
    assert abs((t.SINK_X[1] - t.SINK_X[0]) - 0.381) < 1e-3 and abs((t.SINK_Y[1] - t.SINK_Y[0]) - 0.381) < 1e-3
    assert abs(t.DRAIN_XY[0] - sum(t.DECK_X) / 2) < 1e-9 and abs(t.DRAIN_XY[1] - sum(t.DECK_Y) / 2) < 1e-9


def test_splash_guard_flaps_cover_the_throat_above_the_spoon():
    flaps = [lk for lk in t.SINK.links if lk.name.startswith("flap")]
    assert len(flaps) == 2 * t.FLAP_N == 16  # each flap: a root part and a tip part on a second hinge
    for lk in flaps:
        assert (lk.parent is None) == (not lk.name.endswith("t"))
    for lk in flaps:
        (c,) = lk.colliders
        p = np.asarray(lk.anchor) + np.asarray(c.center) + np.asarray(c.hull)
        r = np.hypot(p[:, 0], p[:, 1])
        assert r.max() <= t.COLLAR_R + 1e-6 and r.min() >= t.FLAP_R_IN * 0.95
        assert p[:, 2].max() - p[:, 2].min() >= 0.0035 - 1e-9  # no hull thinner than 3.5 mm (GPU contact)
        a = math.atan2(lk.anchor[1], lk.anchor[0])
        assert abs((lk.frame_yaw - a - math.pi / 2 + math.pi) % (2 * math.pi) - math.pi) < 1e-9  # hinge on the tangent
        assert isinstance(lk.law, t.Spring) and lk.law.rest == 0.0  # sprung back level
        k = int(lk.name[4:].rstrip("t"))
        lift = t.FLAP_START.get(k, 0.0) if lk.name.endswith("t") else 0.0
        assert abs(lk.start - math.radians(lift)) < 1e-12  # only the tips beside the handle start lifted


def _rot_axis(u, ang):
    u = np.asarray(u, float) / np.linalg.norm(u)
    K = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K


def _hulls_overlap(a, b):
    """Two convex point sets overlap: a point on either's edges lies inside the other's hull."""
    from scipy.spatial import Delaunay

    def dense(p, n=25):
        s = np.linspace(0, 1, n)[:, None]
        return np.vstack([p] + [p[i] * (1 - s) + p[j] * s for i in range(len(p)) for j in range(i + 1, len(p))])
    return bool((Delaunay(a).find_simplex(dense(b)) >= 0).any() or (Delaunay(b).find_simplex(dense(a)) >= 0).any())


def test_flaps_start_clear_of_the_spoon_and_the_lift_is_least_plus_margin():
    o = np.array([*t.DRAIN_XY, t.DECK_Z])
    spoon = [(np.asarray(c.center) + np.asarray(c.hull)) @ t.SPOON_R.T + t.SPOON_POS - o for _, c in t._SPP]
    for k, (root, mid, yaw, outer, inner) in enumerate(t._FLAPS):
        u = (math.cos(yaw), math.sin(yaw), 0.0)
        op = np.asarray(outer[1].center) + np.asarray(outer[1].hull) + root
        ip = np.asarray(inner[1].center) + np.asarray(inner[1].hull)
        lift = math.radians(t.FLAP_START.get(k, 0.0))
        tip = mid + ip @ _rot_axis(u, lift).T
        assert not any(_hulls_overlap(op, s) for s in spoon), k
        assert not any(_hulls_overlap(tip, s) for s in spoon), k
        if k in t.FLAP_START:  # 3 degrees less would touch it: they start resting close on the handle
            less = mid + ip @ _rot_axis(u, lift - math.radians(3.0)).T
            assert any(_hulls_overlap(less, s) for s in spoon), k


def test_spoon_on_the_flaps_or_the_rim_ledge_over_the_drain_is_not_aside():
    on_flaps = _flat_spoon(t.DRAIN_XY[0] - 0.02, t.DRAIN_XY[1])
    on_flaps = (on_flaps[0] + np.array([0, 0, t.FLAP_TOP]), on_flaps[1])
    assert t.spoon_in(_objs(on_flaps)) or not t.set_aside(_objs(on_flaps))
    assert _run([lambda: _objs(on_flaps)] * 40) == 0.0


def test_spoon_on_the_countertop_counts_as_set_aside():
    R = np.eye(3)
    p = t.SPOON_POINTS @ R.T
    sp = (np.array([t.DECK_X[1] + 0.025, -0.08, t.RIM_Z - p[:, 2].min() + 0.0003]), _rz(math.pi / 2))  # on the ledge
    assert t.set_aside(_objs(sp))
    hover = (sp[0] + np.array([0, 0, 0.01]), sp[1])
    assert not t.set_aside(_objs(hover))
    half_in = (np.array([t.DECK_X[1] - 0.03, -0.08, t.RIM_Z - p[:, 2].min() + 0.0003]), np.eye(3))
    assert not t.set_aside(_objs(half_in))  # partly over the bowl: neither on the floor nor on the counter


def test_tongs_start_standing_in_the_crock_clear_of_its_wall():
    o = _objs(START, TONGS_REST)
    tw = t.tongs_world(o)
    r = np.hypot(tw[:, 0] - t.CROCK_XY[0], tw[:, 1] - t.CROCK_XY[1])
    low = tw[:, 2] < t.TOP + t.CROCK_H
    assert r[low].max() < t.CROCK_R - t.CROCK_T - 0.001
    assert abs(tw[:, 2].min() - (t.TOP + t.CROCK_T)) < 0.002  # standing on the heads' ends
    assert tw[:, 1].max() < t.COUNTER_Y[0] - 0.005  # beside the cabinet
