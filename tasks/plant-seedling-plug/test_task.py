"""Offline checks of the planting scorer and demo, no simulator. Crumbs are given as the simulator gives them (each
crumb's center in objects["soil"]); the plug's pose and the grippers are built from the task's own geometry. Covers the
starting layout (crumbs apart, inside the planter box, enough mix to fill the hole), the 16 probe points (set out from
the plug: in the gap and inside the hole's wall for every plug placement in the permitted range, off-center, turned
and tilted; a filled hole round an off-center, turned plug scores full; outside the range scores 0; an empty hole reads 0, heaped mix counts as full and never more, a hole filled on one side only, a pocket between
points, the 40/60/85 % boundaries), every tier, the spill cap, a hand left on the plug, a tilted or proud plug, the
firmness push (requested on the plug, passed and failed), a new trial, the thresholds in the text, and the demo plan
(finger signs, timing, run limit).

    python3 test_task.py
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.demo import grip_rotation
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import pushkit as pk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402

AWAY = {"left_gripper": (0.20, 0.20, M.TOP + 0.17), "right_gripper": (0.20, -0.18, M.TOP + 0.17)}


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def quat(r):
    """A rotation matrix as (x, y, z, w), by its largest component (exact for turns near 180 degrees too)."""
    r = np.asarray(r, float)
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        w = math.sqrt(1 + tr) / 2
        return np.array([(r[2, 1] - r[1, 2]) / (4 * w), (r[0, 2] - r[2, 0]) / (4 * w), (r[1, 0] - r[0, 1]) / (4 * w), w])
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    v = math.sqrt(max(1e-12, 1 + r[i, i] - r[j, j] - r[k, k])) / 2
    out = np.zeros(4)
    out[i] = v
    out[j] = (r[j, i] + r[i, j]) / (4 * v)
    out[k] = (r[k, i] + r[i, k]) / (4 * v)
    out[3] = (r[k, j] - r[j, k]) / (4 * v)
    return out


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def tilted(deg, toward):
    """A tilt of `deg` degrees with the plug's top leaning toward the plan direction `toward` (degrees)."""
    a = math.radians(toward)
    return rz(a) @ np.array([[math.cos(math.radians(deg)), 0.0, math.sin(math.radians(deg))], [0.0, 1.0, 0.0],
                             [-math.sin(math.radians(deg)), 0.0, math.cos(math.radians(deg))]]) @ rz(-a)


def placed(off=(0.0, 0.0), yaw=0.0, rot=None, depth=None):
    """(position, rotation) of the plug with its top's center `off` (m) from the hole's axis in plan, turned `yaw`
    degrees about its axis (then `rot` applied, e.g. a tilt), its top `depth` under the surface."""
    r = (np.eye(3) if rot is None else rot) @ rz(math.radians(yaw))
    depth = M.SET_DEPTH if depth is None else depth
    top = np.array([M.POT[0] + off[0], M.POT[1] + off[1], M.Z_S - depth])
    return tuple(top - r @ np.array([0.0, 0.0, M.PLUG_H / 2])), r


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat(rot), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - 0.01,
                       high=p + 0.01)


def fresh():
    kit._EPISODE["count"] += 1
    M._L = M.Ledger()
    pk.request("plug", None)


START_SOIL = np.array([g.pos for g in M.GRAINS])
PLANTED = (M.POT[0], M.POT[1], M.Z_S - M.SET_DEPTH - M.PLUG_H / 2)


def filled_soil(sides=(True, True, True, True), share=1.0, spill=0, off=(0.0, 0.0), yaw=0.0):
    """Crumbs as a filled hole: clods stacked square, in the plug's own frame, in the gap round a level plug whose top's
    center is `off` from the hole's axis and which is turned `yaw` degrees (only the given sides, in the plug's +x, +y,
    -x, -y order), the rest left on the surface ring, and `spill` crumbs on the bench. With `yaw`, pass
    clod_quats(yaw) as the clods' turn."""
    pts, pitch = [], 2 * M.GRAIN_R + 0.0003
    z0 = M.Z_S - M.SET_DEPTH - M.PLUG_H  # the plug's bottom
    z = M.HOLE_FLOOR + M.GRAIN_R
    r = rz(math.radians(yaw))[:2, :2]
    corners = np.array([(sx * M.GRAIN_R, sy * M.GRAIN_R) for sx in (-1, 1) for sy in (-1, 1)]) @ r.T
    layer = 0
    while z < M.Z_S + M.GRAIN_R * 0.5:
        frac = min(1.0, max(0.0, (z - z0) / M.PLUG_H))
        hx, hy = (w / 2 for w in M.plug_width(frac))
        o = (pitch / 2) * (layer % 2)
        for j in range(-9, 10):
            for i in range(-9, 10):
                x, y = i * pitch + o, j * pitch + o  # in the plug's frame, from its axis
                w = np.array(off) + r @ np.array([x, y])  # from the hole's axis
                if np.hypot(*(w + corners).T).max() > M.R_HOLE + 0.002:
                    continue
                if z > z0 - M.GRAIN_R and abs(x) < hx + M.GRAIN_R and abs(y) < hy + M.GRAIN_R:
                    continue
                a = math.degrees(math.atan2(y, x))
                k = min(range(4), key=lambda q: abs((a - (0, 90, 180, -90)[q] + 180) % 360 - 180))
                if sides[k]:
                    pts.append((M.POT[0] + w[0], M.POT[1] + w[1], z))
        z += pitch
        layer += 1
    n_fill = int(round(len(pts) * share))
    pts = pts[:n_fill]
    rest = [tuple(p) for p in START_SOIL[len(pts):]]
    soil = np.array(pts + rest, float)[:len(M.GRAINS)]
    for k in range(spill):
        soil[-1 - k] = (0.08, -0.05 - 0.013 * k, M.TOP + M.GRAIN_R)
    return soil


def clod_quats(yaw):
    a = math.radians(yaw)
    return np.tile(np.array([0.0, 0.0, math.sin(a / 2), math.cos(a / 2)]), (len(M.GRAINS), 1))


def objs(plug_pos=PLANTED, plug_rot=np.eye(3), soil=None, left=None, quats=None):
    o = {"planter": st((M.POT[0], M.POT[1], M.TOP))}
    o["plug"] = st(plug_pos, plug_rot)
    o["soil"] = START_SOIL.copy() if soil is None else soil
    if quats is not None:
        o["soil_quat"] = quats
    opening = {}
    for g, p in AWAY.items():
        o[g] = st(p, grip_rotation(0.0))
        opening[g] = 1.0
    if left is not None:
        o["left_gripper"] = st(left, grip_rotation(M.YAW_PLUG))
        opening["left_gripper"] = (M.PLUG_TOP[0] / 2) / abs(FINGER_OPEN)
    return o, opening


STEP = [0]


def feed(states):
    """One control step per state, keyed as the simulator keys them (episode, step), so success() and score() on the
    same step update the history once."""
    s = 0.0
    for o, opening in states:
        STEP[0] += 1
        kit._EPISODE["last"] = STEP[0]
        kit._OPENING.update(opening)
        M.success(dict(o))
        s = M.score(dict(o))
    return s


def run(n, **kw):
    return feed([objs(**kw)] * n)


def test_layout():
    p = START_SOIL
    d = np.linalg.norm(p[:, None, :] - p[None, :, :], axis=2) + np.eye(len(p))
    check(len(p) == M.N_GRAINS, f"{M.N_GRAINS} crumbs")
    check(d.min() >= 2 * M.GRAIN_R * 1.12, f"clods start apart, turned (closest {d.min() * 1000:.1f} mm)")
    r = np.hypot(p[:, 0] - M.POT[0], p[:, 1] - M.POT[1])
    check(r.min() >= M.R_HOLE + 0.006 and r.max() <= M.BED_X / 2 - M.GRAIN_R, "clods lie round the hole, in the planter box")
    check(p[:, 2].max() + M.GRAIN_R <= M.Z_S + 0.025, "the heap is low (2.5 cm)")
    check(abs(M.PLUG_START[1]) - M.PLUG_TOP[1] / 2 > M.BED_Y / 2 + M.BED_WALL, "the plug stands on the bench, beside the planter box")
    ratio = M.N_GRAINS * M.GRAIN_BULK / M.VOID
    check(1.1 <= ratio <= 1.4, f"loose mix is {ratio:.2f} x the gap's volume ({M.VOID * 1e6:.0f} mL)")
    check(abs(M.N_GRAINS * M.GRAIN_MASS * 1000 - 200) < 1, "200 g of loose mix")
    check(abs(M.N_GRAINS * M.GRAIN_BULK * 1e6 - 444) < 5, "about 440 mL of loose mix")
    w = M.PLUG_TOP
    check(math.hypot(w[0], w[1]) / 2 < M.R_HOLE, "the plug's top fits the hole corner to corner")
    v = M.PLUG_H * (M.PLUG_BOT[0] * M.PLUG_BOT[1] + w[0] * w[1] + math.sqrt(np.prod(M.PLUG_BOT) * np.prod(w))) / 3
    check(abs(v * 1e6 - 118) < 8, f"plug volume {v * 1e6:.0f} mL (the 3601 cell: 118 mL)")


def test_nothing_done():
    fresh()
    s = run(60, plug_pos=(M.PLUG_START[0], M.PLUG_START[1], M.TOP + M.PLUG_H / 2))
    check(s == 0.0, "doing nothing scores 0")


def test_full_and_push():
    fresh()
    soil = filled_soil()
    s = run(M.HOLD_STEPS - 1, soil=soil)
    check(s == 0.75, f"planted and filled, before the push: 0.75 (got {s})")
    st0 = M._L.state
    check(st0["sides"] == 4 and st0["fill"] >= M.FILL_FULL, f"all four sides level, fill {st0['fill']:.2f}")
    s = run(3, soil=soil)
    req = pk.pending().get("plug")
    check(req is not None and abs(req[0][1] - M.PUSH_N) < 1e-9, "the side push is requested on the plug")
    check(abs(req[1][2] - (M.Z_S - M.SET_DEPTH + M.PUSH_ABOVE)) < 1e-6, "pushed 5 cm above the plug's top")
    s = run(2 * M.PUSH_STEPS + M.SETTLE_STEPS, soil=soil)
    check(s == 1.0, f"firm through the push: 1.0 (got {s})")
    check(pk.pending().get("plug") is None, "the push is released after the test")
    s = run(20, soil=soil)
    check(s == 1.0, "stays 1.0 while it stands")


def test_push_fails():
    fresh()
    soil = filled_soil()
    run(M.HOLD_STEPS + 2, soil=soil)
    states = [objs(soil=soil, plug_rot=rx(math.radians(min(8.0, 0.5 * k)))) for k in range(M.PUSH_STEPS)]
    feed(states)
    check(M._L.failed and pk.pending().get("plug") is None, "the push stops once the plug has turned 5 degrees")
    s = run(M.PUSH_STEPS + M.SETTLE_STEPS, soil=soil, plug_rot=rx(math.radians(4.0)))
    check(s == 0.75 and M._L.push == 0, f"failed the push: 0.75, and it is not pushed again (got {s})")


def test_sides_and_fill():
    fresh()
    s = run(30, soil=filled_soil(sides=(True, True, True, False)))
    check(s == 0.5, f"three sides filled: 0.5 (got {s})")
    fresh()
    s = run(30, soil=filled_soil(sides=(True, False, True, False)))
    check(s == 0.25, f"two sides filled: 0.25 (got {s})")
    fresh()
    s = run(30, soil=filled_soil(sides=(True, False, False, False)))
    f = M._L.state["fill"]
    check(s == 0.0 and abs(f - 0.25) < 1e-9, f"a hole filled on one side only: fill {f:.2f}, 0 (got {s})")


def probes_at(fills, plug_pos=PLANTED):
    """Soil as one clod under each template point, its top at that point's given fill (share of the hole's depth
    above the floor; above 1 is heaped over the surface); a fill of 0 leaves the point bare."""
    pts = M.probe_points(objs(plug_pos=plug_pos)[0])
    soil = START_SOIL.copy()
    for k, f in enumerate(fills):
        if f > 0:
            soil[k] = (pts[k, 0], pts[k, 1], M.HOLE_FLOOR + f * M.HOLE_D - M.GRAIN_R)
    return soil


PLACEMENTS = [(0.0, 0.0), (0.003, 0.0)] + [(M.PLACE_R * math.cos(math.radians(a)), M.PLACE_R * math.sin(math.radians(a)))
                                           for a in range(0, 360, 15)]
TILTS = [None] + [tilted(M.TILT_FULL, a) for a in range(0, 360, 45)]


def test_probe_points():
    """For every placement in the permitted range (the top's center up to 0.6 cm off the hole's axis in 24
    directions, turned every 10 degrees, upright or tilted 10 degrees eight ways), an empty hole reads its floor at
    all 16 points: each probe's whole end lands inside the hole's wall and clear of the plug."""
    o, _ = objs()
    pts = M.probe_points(o)
    check(len(pts) == 16 and all(M.PROBE_SIDE.count(k) == 4 for k in range(4)), "16 points, four on each side")
    far = START_SOIL[:1]  # no mix in the hole
    n, worst_wall, worst = 0, 1.0, 0.0
    for off in PLACEMENTS:
        for yaw in range(0, 360, 10):
            for rot in TILTS:
                pos, r = placed(off, yaw, rot)
                o, _ = objs(plug_pos=pos, plug_rot=r)
                h = M.probe_heights(o, far)
                worst = max(worst, float(np.abs(h - M.HOLE_FLOOR).max()))
                p = M.probe_points(o)
                worst_wall = min(worst_wall, float(M.R_HOLE - M.PROBE_D / 2 - np.hypot(p[:, 0] - M.POT[0],
                                                                                         p[:, 1] - M.POT[1]).max()))
                n += 1
    check(worst < 1e-9, f"{n} placements in range: every probe reads the empty hole's floor (off by at most "
                        f"{worst * 1000:.2f} mm): in the gap, never on the plug or the hole's edge")
    check(worst_wall >= 0.0008, f"the probes' ends stay {worst_wall * 1000:.1f} mm or more inside the hole's wall")
    upright = []
    for yaw in range(0, 360, 10):  # the clearance to the plug's faces
        _, r = placed((0.0, 0.0), yaw)
        rel = (M.probe_points(objs(plug_pos=placed((0.0, 0.0), yaw)[0], plug_rot=r)[0]) - np.array(M.POT)) @ r[:2, :2]
        upright.append(np.maximum(np.abs(rel[:, 0]) - M.PLUG_TOP[0] / 2, np.abs(rel[:, 1]) - M.PLUG_TOP[1] / 2).min())
    check(abs(min(upright) - M.PROBE_OUT) < 1e-9, f"every point {M.PROBE_OUT * 100:.1f} cm out from its face, any turn")
    fresh()
    s = run(30)
    check(s == 0.0, "the plug in an empty hole scores 0")


def test_off_center_turned():
    """A hole filled round a plug that stands off-center (up to the 0.6 cm limit) and turned: full credit, as for a
    centered one; one side left bare is still caught; past the limit the trial scores 0."""
    cases = ((M.PLACE_R, 0.0, 0.0), (0.0, -M.PLACE_R, 0.0), (0.0042, 0.0042, 30.0), (-M.PLACE_R, 0.0, 45.0),
             (0.003, -0.005, 72.0), (0.0, 0.0, 90.0), (-0.004, 0.004, 137.0), (0.0, M.PLACE_R, -20.0))
    for x, y, yaw in cases:
        fresh()
        pos, r = placed((x, y), yaw)
        soil = filled_soil(off=(x, y), yaw=yaw)
        kw = dict(soil=soil, plug_pos=pos, plug_rot=r, quats=clod_quats(yaw))
        s = run(M.HOLD_STEPS - 1, **kw)
        st0 = M._L.state
        check(s == 0.75 and st0["sides"] == 4 and st0["fill"] >= M.FILL_FULL,
              f"off-center ({x * 100:.2f}, {y * 100:.2f}) cm, turned {yaw:.0f} deg, filled: fill {st0['fill']:.2f}, "
              f"4 sides, 0.75 before the push (got {s})")
        s = run(3 + 2 * M.PUSH_STEPS + M.SETTLE_STEPS, **kw)
        check(s == 1.0, f"  and 1.0 after the push (got {s})")
        fresh()
        bare = filled_soil(sides=(True, True, True, False), off=(x, y), yaw=yaw)
        s = run(30, soil=bare, plug_pos=pos, plug_rot=r, quats=clod_quats(yaw))
        check(s == 0.5 and M._L.state["sides"] == 3, f"  the same with its -y side bare: 3 sides, 0.5 (got {s})")
    for x, y, yaw in ((0.008, 0.0, 0.0), (0.0, -0.0065, 30.0), (-0.012, 0.012, 45.0)):
        fresh()
        pos, r = placed((x, y), yaw)
        s = run(30, soil=filled_soil(off=(x, y), yaw=yaw), plug_pos=pos, plug_rot=r, quats=clod_quats(yaw))
        check(s == 0.0 and not M._L.state["hole"],
              f"top's center {math.hypot(x, y) * 100:.2f} cm off the axis (past 0.6 cm): outside the range, 0 (got {s})")


def test_heaped_capped():
    fresh()
    half = [1.0 if k < 8 else 0.0 for k in range(16)]
    heaped = [3.0 if k < 8 else 0.0 for k in range(16)]  # the same sides with mix heaped 12 cm above the surface
    o, _ = objs()
    f1 = M.fill_share(M.probe_heights(o, probes_at(half)))
    f2 = M.fill_share(M.probe_heights(o, probes_at(heaped)))
    check(abs(f1 - 0.5) < 1e-9 and abs(f2 - 0.5) < 1e-9, f"heaped mix counts as full, never more ({f1:.2f}, {f2:.2f})")
    lv = M.side_levels(M.probe_heights(o, probes_at(heaped)))
    check(max(lv) <= M.Z_S + 1e-9, "a side's level is capped at the surface")
    soil = filled_soil(sides=(True, True, False, False))
    pts = M.probe_points(o)
    for k in range(8):  # pile clods high on the two filled sides
        soil[-1 - k] = (pts[k, 0], pts[k, 1], M.Z_S + 0.03)
    s = run(30, soil=soil)
    check(s == 0.25 and abs(M._L.state["fill"] - 0.5) < 1e-9 and M._L.state["sides"] == 2,
          f"two sides filled and heaped high: still two sides, fill 0.50, 0.25 (got {s})")


def test_pocket_between_points():
    """An empty pocket 2.4 cm long along a face, through the whole gap, anywhere along it: a probe falls in and that
    side no longer counts as filled. (Points on a side are 0.7-1.0 cm apart, 1.2-1.4 cm from the corners; a clod beside
    a pocket reaches 5.5 mm into it and the probe's end is 9.5 mm across, so with 1.1 cm clods a pocket of 2 cm can be
    bridged.)"""
    base = filled_soil()
    for side, (ax, half_face) in enumerate(((0, M.PLUG_TOP[1] / 2), (1, M.PLUG_TOP[0] / 2), (0, M.PLUG_TOP[1] / 2),
                                            (1, M.PLUG_TOP[0] / 2))):
        sign = 1.0 if side < 2 else -1.0
        length = 0.024
        worst = -1.0
        for c in np.arange(-half_face + length / 2, half_face - length / 2 + 1e-9, 0.002):
            soil = base.copy()
            d = soil[:, :2] - np.array(M.POT)
            along = d[:, 1 - ax]
            out = sign * d[:, ax]
            face = M.PLUG_TOP[ax] / 2
            pocket = (np.abs(along - c) <= length / 2) & (out > face - 0.001) & (soil[:, 2] < M.Z_S + 0.02)
            soil[pocket] = START_SOIL[pocket]  # that mix is back on the surface ring
            o, _ = objs(soil=soil)
            lv = M.side_levels(M.probe_heights(o, soil))
            worst = max(worst, lv[side] - (M.Z_S - M.SET_DEPTH))
        check(worst < -M.LEVEL_TOL, f"side {side}: a 2.4 cm pocket anywhere along it fails the side "
                                    f"(highest level {worst * 100:.1f} cm)")


def test_fill_boundaries():
    eps = 0.002
    for f, tier_at, tier_below in ((M.FILL_QUARTER, 0.25, 0.0), (M.FILL_FULL, 0.75, 0.5)):
        for ff, want in ((f + eps, tier_at), (f - eps, tier_below)):
            fresh()
            s = run(5, soil=probes_at([ff] * 16))
            check(abs(M._L.state["fill"] - ff) < 1e-9 and s == want,
                  f"every point at {ff:.3f}: fill {M._L.state['fill']:.3f}, {want} (got {s})")
    # 60 %: three sides filled and one bare. With the plug's top 9.9 mm under the surface (the deepest still level), a
    # side whose points are at 0.80 of the depth is within 1 cm of the plug's top, so 12 points at 0.80 is a 60 % fill
    # (with the plug's top at the surface, three filled sides alone already give over 60 %)
    sunk = (PLANTED[0], PLANTED[1], M.Z_S - 0.0099 - M.PLUG_H / 2)
    for ff, want in ((0.80 + eps, 0.5), (0.80 - eps, 0.25)):
        fresh()
        s = run(5, soil=probes_at([ff] * 12 + [0.0] * 4, plug_pos=sunk), plug_pos=sunk)
        st0 = M._L.state
        check(st0["sides"] == 3 and abs(st0["fill"] - 0.75 * ff) < 1e-9 and s == want,
              f"three sides at {ff:.3f}, one bare: fill {st0['fill']:.4f}, {want} (got {s})")


def test_tilt_depth_hand_spill():
    fresh()
    soil = filled_soil()
    pos, r = placed(rot=rx(math.radians(9)))
    s = run(M.HOLD_STEPS - 1, soil=soil, plug_pos=pos, plug_rot=r)
    check(s == 0.75, f"tilted 9 degrees: still planted, 0.75 before the push (got {s})")
    for deg in (11, 15, 25):
        fresh()
        pos, r = placed(rot=rx(math.radians(deg)))
        s = run(60, soil=soil, plug_pos=pos, plug_rot=r)
        check(s == 0.0, f"tilted {deg} degrees: outside the range (upright within 10), 0 (got {s})")
    fresh()
    proud = (PLANTED[0], PLANTED[1], PLANTED[2] + 0.02)
    s = run(60, soil=soil, plug_pos=proud)
    check(s < 1.0 and s != 0.75, f"2 cm proud is not planted level (got {s})")
    fresh()
    s = run(60, soil=soil, left=(PLANTED[0], PLANTED[1], M.Z_S - M.SET_DEPTH - M.GRIP_BELOW_TOP))
    check(s == 0.25, f"still held: 0.25 (got {s})")
    check(pk.pending().get("plug") is None, "no push while a hand holds it")
    fresh()
    s = run(60, soil=filled_soil(spill=int(0.12 * M.N_GRAINS)))
    check(s <= 0.5, f"12 % spilled caps at 0.5 (got {s})")


def plant(o, bend=(0.0, 0.0)):
    """The stem segments and leaves on a level planted plug, each stem joint bent `bend` degrees about x."""
    c = np.asarray(o["plug"].pos, float)
    r = np.eye(3)
    z = c + np.array([0.0, 0.0, M.PLUG_H / 2])
    for k, b in enumerate(bend):
        r = r @ rx(math.radians(b))
        o[f"plug/stem{k}"] = st(z, r)
        z = z + r @ np.array([0.0, 0.0, M.STEM_SEG])
    for name, seg, h, deg, length in M.LEAVES + (M.TOP_LEAF,):
        o[f"plug/{name}"] = st(c + np.array([0.0, 0.0, M.PLUG_H / 2 + h]))
    return o


def test_damage():
    fresh()
    soil = filled_soil()
    o, op = objs(soil=soil)
    s = feed([(plant(dict(o), bend=(25.0, 30.0)), op)] * (M.HOLD_STEPS + 2 * M.PUSH_STEPS + M.SETTLE_STEPS + 5))
    check(s == 1.0 and M._L.damage is None, f"stem bent 25-30 degrees a joint (springy, not kinked): 1.0 (got {s})")
    check(M.stem_bends(plant(dict(o), bend=(25.0, 30.0)))[1] > 29.9, "the bend is read joint by joint")
    s = feed([(plant(dict(o), bend=(0.0, M.KINK_DEG + 5)), op)] * 2)
    check(s == 0.0 and M._L.damage == "kinked", f"a joint bent past {M.KINK_DEG:.0f} degrees: kinked, 0 (got {s})")
    s = feed([(plant(dict(o)), op)] * 40)
    check(s == 0.0, "a kink stays: 0 even when the stem springs back")
    fresh()
    s = feed([(plant(dict(o), bend=(0.0, M.KINK_DEG + 5)), op)] + [(plant(dict(o)), op)] * 40)
    check(M._L.damage is None, "one step past the limit (a glitch) is not a kink")
    # a hand pinching the stem: the left gripper's pads across the middle segment
    fresh()
    o2 = plant(dict(o))
    mid = np.asarray(o2["plug/stem1"].pos) + np.array([0.0, 0.0, M.STEM_SEG / 3])
    o2["left_gripper"] = st(mid, grip_rotation(M.YAW_PLUG))
    op2 = dict(op, left_gripper=(M.STEM_D / 2 + 0.0005) / abs(FINGER_OPEN))
    s = feed([(o2, op2)] * 3)
    check(M._L.damage == "pinched" and s == 0.0, f"the stem squeezed between the pads: pinched, 0 (got {s})")
    fresh()
    op3 = dict(op, left_gripper=0.6)  # the same hand open round it: no squeeze
    feed([(o2, op3)] * 3)
    check(M._L.damage is None, "an open hand round the stem is not a pinch")
    fresh()
    o4 = dict(o2)
    o4["left_gripper"] = st(mid + np.array([M.STEM_D / 2 + 0.0055, 0.0, 0.0]), grip_rotation(M.YAW_PLUG))
    feed([(o4, dict(op, left_gripper=0.0))] * 3)
    check(M._L.damage is None, "one closed finger resting against the stem is not a pinch")


def test_plant_model():
    m = M._plug()
    names = [lk.name for lk in m.links]
    check(all(f"stem{k}" in names and f"bend{k}" in names for k in range(2)), "two stem segments, each on two bend axes")
    check(abs(M.STEM_H - 0.09) < 1e-9 and abs(M.STEM_D - 0.0036) < 1e-9, "stem 9 cm tall, 3.6 mm across")
    ei = [k * M.STEM_SEG for k in M.STEM_K]
    check(ei[0] > ei[1], "stiff woody base, soft top")
    # a 0.5 N push 5 cm above the plug's top, held on the stem: the static bend stays under the kink limit
    th0 = math.degrees(M.PUSH_N * 0.05 / M.STEM_K[0])
    th1 = math.degrees(M.PUSH_N * (0.05 - M.STEM_SEG) / M.STEM_K[1])
    for name, seg, h, deg, length in M.LEAVES:
        lk = next(x for x in m.links if x.name == name)
        sw = next(x for x in m.links if x.name == f"{name}_swing")
        check(sw.parent == f"stem{seg}" and sw.axis == "Z" and lk.parent == sw.name and lk.axis in "XY" and lk.colliders,
              f"{name}: a body on a springy two-axis petiole joint on stem segment {seg}")
    name, seg, h, deg, length = M.TOP_LEAF
    lk = next(x for x in m.links if x.name == name)
    check(lk.parent == f"stem{seg}" and lk.axis in "XY" and lk.colliders and h > M.STEM_SEG,
          f"{name}: the youngest leaf, on one springy joint on the upper stem")
    depth = {}
    for x in m.links:
        depth[x.name] = depth.get(x.parent, 0) + 1
    check(max(depth.values()) <= 5, f"the articulation is at most five links deep (got {max(depth.values())})")
    check(max(th0, th1) < M.KINK_DEG, f"the spring-gauge push bends the stem {th0:.0f} and {th1:.0f} degrees: no kink")
    import yamkit as kit2
    for lk in m.links:  # the springs hold the plant from the start (also while the scene settles)
        check(kit2._settle_stiffness(lk) == lk.law.k > 0, f"{lk.name}: its spring is on the actuator from the start")
        check(lk.colliders, f"{lk.name} has a collider (every articulation body needs a shape)")
    mass = M.PLUG_MASS + sum(lk.mass for lk in m.links)
    check(0.074 < mass < 0.0775, f"plug and plant {mass * 1000:.1f} g")

    # a leaf squeezed between the pads is a pinch too
    fresh()
    soil = filled_soil()
    o, op = objs(soil=soil)
    o2 = plant(dict(o))
    name, seg, h, deg, length = M.LEAVES[0]
    u = np.array([math.cos(math.radians(deg)), math.sin(math.radians(deg)), 0.0])
    p = np.asarray(o2[f"plug/{name}"].pos) + u * 0.04
    zc = np.array([0.0, 0.0, 1.0])  # the fingers close vertically, across the leaf's thickness, the hand coming from the side
    ap = np.array([-u[1], u[0], 0.0])
    o2["right_gripper"] = st(p - ap * 0.0, np.column_stack([zc, np.cross(ap, zc), ap]))
    feed([(o2, dict(op, right_gripper=(M.LEAF_T / 2 + 0.0005) / abs(FINGER_OPEN)))] * 3)
    check(M._L.damage == "pinched", "a leaf squeezed flat between the pads: pinched")


def test_new_trial():
    fresh()
    soil = filled_soil()
    run(M.HOLD_STEPS + 2 * M.PUSH_STEPS + M.SETTLE_STEPS + 5, soil=soil)
    check(M._L.firm, "firm in trial 1")
    kit._EPISODE["count"] += 1
    s = run(1)
    check(s == 0.0 and not M._L.firm, "a new trial starts afresh")


def test_demo_plan():
    plans = M.demo()
    for side, wps in plans.items():
        t = sum(w.duration for w in wps)
        check(t / M.DT + 40 < M.RUN_LIMIT_STEPS, f"{side} plan {t:.1f} s fits the run limit")
        for w in wps:
            check_f = FINGER_OPEN <= w.finger <= FINGER_CLOSED
            if not check_f:
                check(False, f"{side} {w.label}: finger {w.finger} in range")
    tl = sum(w.duration for w in plans["left"])
    tr = sum(w.duration for w in plans["right"])
    check(abs(tl - tr) < 0.01, "both plans end together")
    check(plans["right"][-1].duration / M.DT >= M.HOLD_STEPS + 2 * M.PUSH_STEPS + M.SETTLE_STEPS,
          "the plan waits long enough at the end for the hold and the push test")
    pinch = [w for w in plans["left"] if w.label == "pinch"][0]
    check(pinch.finger == FINGER_CLOSED and plans["left"][0].finger == FINGER_OPEN, "signed finger targets")


def test_text():
    folder = HERE
    tj = json.loads((folder / "task.json").read_text()) if (folder / "task.json").exists() else None
    if tj is None:
        print("skip text checks: no task.json yet")
        return
    check(tj["instruction"] == M.INSTRUCTION, "task.json instruction matches task.py")
    check(len(tj["context"]) <= 1000, f"context {len(tj['context'])} chars")
    sc = tj["scoring"]
    for txt in ("1 cm", "10 degrees", "0.6 cm", "1.0 cm", "85%", "60%", "40%", "10%", "the weight of 50 g", "5 cm", "5 degrees", "16 points",
                "9.5 mm", "Hirschi"):
        check(txt in sc, f"scoring text says {txt}")
    check(":" not in tj["title"], "no colon in the title")
    check(re.search(r"TBD", json.dumps(tj)) is None, "no TBD")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("all passed")
    sys.exit(0)
