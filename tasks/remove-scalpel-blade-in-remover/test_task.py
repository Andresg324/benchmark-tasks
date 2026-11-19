"""Offline checks of the scalpel-blade task's scorer, remover mechanism and demo geometry, no simulator. Poses are set by
hand, as the simulator reports them each control step. Covers: doing nothing (0); the start layout (blade on, resting in
the dish); the demo's insert pose putting the blade at the stop; the remover's catch (a press with the blade at the stop
frees it; a press before the blade got there, or a pull before the press, does not); the tiers 0.4 -> 0.7 -> 1.0 with
the 2 s hold; the hazards (a gripper on the blade, the bladed scalpel in the tray, a drop); the thresholds in the text.

    python3 test_task.py
"""
import json
import math
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import yamkit as kit  # noqa: E402

AWAY = {"left_gripper": (0.15, 0.30, M.TOP + 0.30), "right_gripper": (0.15, -0.30, M.TOP + 0.30)}


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(M._quat(rot), float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.05, high=p + 0.05)


def fresh():
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    M.WELDS.offline_reset()


def world(handle=None, blade=None, remover=None, btn=0.0, hands=None, opening=0.3):
    """One control step's objects: the handle (origin, R), the blade (origin, R; default on the handle), the remover's
    base origin, the button's depth (m), grippers (pos, R) by name."""
    h_o, h_R = handle if handle is not None else (M.H_START, M.R_START)
    b_o, b_R = blade if blade is not None else (h_o, h_R)
    r = np.array([M.REMOVER_POS[0], M.REMOVER_POS[1], M.TOP]) if remover is None else np.asarray(remover, float)
    o = {"handle": st(h_o, h_R), "blade": st(b_o, b_R), "remover": st(r),
         "dish": st((M.DISH_POS[0], M.DISH_POS[1], M.TOP)), "tray": st((M.TRAY_POS[0], M.TRAY_POS[1], M.TOP))}
    hands = hands or {}
    for g, p in AWAY.items():
        pos, R = hands.get(g, (p, M.DOWN_Y))
        o[g] = st(pos, R)
    o["remover/button"] = st(r + np.array([0, M.BTN_Y, M.R_H]))
    WORLD_STATE[id(o)] = (btn, opening)
    put(o)
    return o


WORLD_STATE = {}


def put(o):
    """The joint and gripper readings that go with this step's objects (the simulator sets them each step)."""
    btn, opening = WORLD_STATE[id(o)]
    kit._JOINTS["remover/button"] = kit.JointState(pos=-btn, vel=0.0, latched=False)
    for g in AWAY:
        kit._OPENING[g] = opening


def run(steps):
    """Feed a list of object dicts as successive control steps; return the scores."""
    out = []
    for o in steps:
        kit._EPISODE["last"] = (kit._EPISODE["last"] or 0) + 1
        put(o)
        out.append(M.score(o))
    return out


SEAT = (M.insert_origin(), M.R_FLAT)
OUT = (M.insert_origin(back=0.075), M.R_FLAT)
LAID = (M.tray_origin(), M.R_FLAT)


def test_start():
    fresh()
    o = world()
    check(M.blade_on(o), "start: the blade is on the handle")
    check(not M.at_stop(o) and not M.captured(o), "start: the blade is not in the remover")
    check(run([world() for _ in range(30)])[-1] == 0.0, "doing nothing scores 0")
    check(not M._L.ruined, "doing nothing fires no hazard")
    tip = M.H_START + M.R_START @ np.array([M.B_X1, 0.0, M.B_Z0])
    check(abs(tip[2] - M.DISH_FLOOR) < 0.0006, f"the blade's tip rests on the dish floor ({(tip[2] - M.DISH_FLOOR) * 1000:.2f} mm)")
    tail = M.H_START + M.R_START @ np.array([-M.H_LEN, 0.0, 0.0])
    check(tail[1] < M.DISH_POS[1] - M.D_B and tail[2] > M.TOP + M.D_H, "the handle's tail overhangs the rim, raised")
    check(15 < math.degrees(M.TILT) < 25, f"the scalpel lies tilted {math.degrees(M.TILT):.1f} deg on the rim")


def test_insert_pose():
    fresh()
    o = world(handle=SEAT)
    check(M.at_stop(o), "the demo's insert pose puts the blade flat in the slot at the stop")
    short = (M.insert_origin(back=0.003), M.R_FLAT)
    check(not M.at_stop(world(handle=short)), "3 mm short of the stop is not at the stop")
    tilted = (M.insert_origin(), rx_about_tip(math.radians(8)))
    check(not M.at_stop(world(handle=tilted)), "a blade tipped 8 degrees out of the slot's plane is not flat in the slot")
    # the handle's fitting fits the 17 x 5.2 mm slot: every corner of the handle part that goes in is inside the channel
    pts = [M.to_local(o, "remover", M.to_world(o, "handle", c)) for c in M.corners(M.TANG_LO, M.TANG_HI)]
    check(all(M._CH[0] < p[0] < M._CH[1] and M._CH[2] < p[2] < M._CH[3] for p in pts), "the fitting clears the slot's walls")
    blade = [M.to_local(o, "remover", M.to_world(o, "blade", c)) for c in M.BLADE_PTS]
    check(all(M._CH[2] < p[2] < M._CH[3] for p in blade), "the blade clears the slot's floor and roof")


def rx_about_tip(a):
    return M.rx(a) @ M.R_FLAT


def test_mechanism_and_tiers():
    fresh()
    steps = [world(handle=SEAT, btn=0.0) for _ in range(3)]  # at the stop, button up: the catch is set
    steps += [world(handle=SEAT, btn=0.003)]  # pressed
    s = run(steps)
    check(not M.WELDS.state("lug") and M.WELDS.state("catch"), "a press with the blade at the stop frees it into the remover")
    check(s[-1] == 0.0, "freed but the handle still in the slot: not yet scored (0)")
    # the blade stays where it was freed (held by the catch); the handle comes back out bare
    b = SEAT
    s = run([world(handle=OUT, blade=b, btn=0.003)])
    check(M.captured(world(handle=OUT, blade=b)), "the blade in the remover, the bare handle out: captured")
    check(s[-1] == 0.4, "captured, the handle held: 0.4")
    s = run([world(handle=LAID, blade=b, hands={"right_gripper": (M.hand_for(*LAID)[0], M.hand_for(*LAID)[1])})])
    check(s[-1] == 0.7, "the bare handle in the tray, still held: 0.7")
    s = run([world(handle=LAID, blade=b) for _ in range(M.HOLD_STEPS + 2)])
    check(s[M.HOLD_STEPS - 2] == 0.7 and s[-1] == 1.0, "hands clear and still for 2 s: 1.0")
    check(M.success(world(handle=LAID, blade=b)), "success() at the end")
    tipped = st(np.array([M.REMOVER_POS[0], M.REMOVER_POS[1], M.TOP + 0.0125]), M.ry(math.pi / 2))
    o = world(handle=LAID, blade=b)
    o["remover"] = tipped
    check(not M.remover_upright(o), "a remover lying on its side is not upright")


def test_no_catch_paths():
    fresh()
    run([world(handle=SEAT, btn=0.003) for _ in range(3)])  # button already down when the blade arrives
    check(M.WELDS.state("lug"), "a button held down before the blade reached the stop does not free it")
    fresh()
    run([world(handle=SEAT, btn=0.0), world(handle=OUT, btn=0.0)])
    check(M.WELDS.state("lug") and M.blade_on(world(handle=OUT)), "pulled out before the press: the blade stays on")
    s = run([world(handle=LAID) for _ in range(30)])
    check(s[-1] == 0.0 and M._L.ruined, "the bladed scalpel laid in the tray scores 0 (latched)")
    fresh()
    run([world(handle=(M.insert_origin(back=0.004), M.R_FLAT), btn=0.0), world(handle=(M.insert_origin(back=0.004), M.R_FLAT), btn=0.003)])
    check(M.WELDS.state("lug"), "a press with the blade 4 mm short of the stop does not free it")


def test_hazards():
    fresh()
    # a closed fingertip resting on the blade's flat
    tip = M.H_START + M.R_START @ np.array([0.010, 0.0, M.B_Z0 + M.B_TC])
    tcp = tip + np.array([0.0, 0.0, 0.0128 + 0.0005])
    s = run([world(hands={"left_gripper": (tcp, M.DOWN_Y)}, opening=0.0)])
    check(s[-1] == 0.0 and "touched the blade" in M._L.ruined, "a fingertip on the blade: 0 (latched)")
    fresh()
    g0, Rg0 = M.hand_for(M.H_START, M.R_START)
    s = run([world(hands={"right_gripper": (g0, Rg0)}, opening=M.H_SQUEEZE / M.FINGER_OPEN)])
    check(not M._L.ruined, f"the demo's grip on the handle does not touch the blade "
                           f"(gap {M.gk.hand_gap(world(hands={'right_gripper': (g0, Rg0)}), 'right_gripper', 'blade', M.BLADE_PARTS, 0.2) * 1000:.0f} mm)")
    fresh()
    h1 = (M.H_START + np.array([0, 0, 0.10]), M.R_START)
    h2 = (M.H_START + np.array([0, 0, 0.06]), M.R_START)
    s = run([world(handle=h1), world(handle=h2)])
    check(s[-1] == 0.0 and "dropped" in M._L.ruined, "the bladed scalpel falling 0.8 m/s: dropped, 0")
    fresh()
    press = M.remover_top(M.BTN_Y, M.BTN_TRAVEL - M.BTN_PROUD)  # where the hand stops: the button bottomed (the
    # command goes deeper, into the solid remover, for force)
    o = world(handle=SEAT, hands={"left_gripper": (press, M.DOWN_Y)}, opening=0.0)
    check(not M.gk.touching(o, "blade", M.BLADE_PARTS, M.TOUCH_TOL), "the press on the button does not touch the blade")
    seat_hand = M.hand_for(*SEAT)
    o = world(handle=SEAT, hands={"right_gripper": seat_hand}, opening=M.H_SQUEEZE / M.FINGER_OPEN)
    check(not M.gk.touching(o, "remover", M.REMOVER_PARTS, 0.0), "the scalpel hand at the stop stays off the remover")
    check(np.linalg.norm(seat_hand[0][:2] - press[:2]) > 0.09, "the two hands stay 9 cm apart at the press")


def test_text():
    j = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    check(j["instruction"] == M.INSTRUCTION, "task.json instruction matches task.py")
    for v in ("1.5 mm", "17 x 5.2 mm", "2 mm", "0.5 m/s", "10 degrees", "1 mm", "4 cm", "0.6 mm", "1 cm", "0.4", "0.7", "1.0"):
        check(v in j["scoring"] and v in readme, f"'{v}' in task.json scoring and README")
    check(len(j["title"]) <= 120 and ":" not in j["title"], "title within 120 characters, no colon")


def test_gaming():
    """Every way found to earn credit without taking the blade off in the remover; each scores 0 (or caps below 1.0),
    and each is visible on the real trial's video."""
    # 1. doing nothing, and the start state: no end condition already holds
    fresh()
    o = world()
    check(not M.captured(o) and not M.in_tray(o, "handle", M.HANDLE_PTS), "G1 start: not captured, handle not in the tray")
    check(run([world() for _ in range(25)])[-1] == 0.0, "G1 doing nothing: 0")
    # 2. the blade taken off by a gripper instead of the remover: a touch is a latched 0 ...
    fresh()
    pinch = M.H_START + M.R_START @ np.array([0.010, 0.0, M.B_Z0])
    s = run([world(hands={"left_gripper": (pinch + np.array([0, 0, 0.003]), M.DOWN_Y)}, opening=0.05)])
    check(s[-1] == 0.0 and "touched the blade" in M._L.ruined, "G2 a gripper pinching the blade: 0 (latched)")
    # ... and a blade off the handle anywhere but inside the remover earns nothing
    fresh()
    loose = (np.array([0.20, 0.10, M.TOP + 0.002]), M.R_FLAT)
    s = run([world(handle=LAID, blade=loose) for _ in range(M.HOLD_STEPS + 2)])
    check(s[-1] == 0.0, "G2 blade loose on the table, bare handle in the tray: 0")
    # 3. the bladed scalpel put (or dropped) in the tray, even if taken out again later
    fresh()
    s = run([world(handle=LAID)] + [world(handle=SEAT) for _ in range(3)])
    check(s[-1] == 0.0 and "instrument tray" in M._L.ruined, "G3 bladed scalpel in the tray, then out: 0 (latched)")
    # 4. the scalpel left in the slot without the press, hands off: not captured
    fresh()
    s = run([world(handle=SEAT) for _ in range(M.HOLD_STEPS + 2)])
    check(s[-1] == 0.0, "G4 scalpel parked in the slot, blade still on: 0")
    # 5. freed but the handle left in the slot: not captured until the bare handle is out
    fresh()
    s = run([world(handle=SEAT, btn=0.0), world(handle=SEAT, btn=0.003)] + [world(handle=SEAT, blade=SEAT) for _ in range(5)])
    check(s[-1] == 0.0, "G5 freed, handle still in the slot: 0")
    # 6. knocking the remover: tipped over before the blade goes in, the blade cannot be freed; tipped after, at most 0.7
    fresh()
    over = np.array([M.REMOVER_POS[0], M.REMOVER_POS[1], M.TOP + 0.0125])
    o = world(handle=SEAT, remover=over)
    o["remover"] = st(over, M.ry(math.pi / 2))
    check(not M.at_stop(o), "G6 remover knocked over: the blade is not at the slot's stop")
    fresh()
    run([world(handle=SEAT, btn=0.0), world(handle=SEAT, btn=0.003)])
    steps = []
    for _ in range(M.HOLD_STEPS + 2):
        o = world(handle=LAID, blade=SEAT)
        o["remover"] = st(np.array([M.REMOVER_POS[0], M.REMOVER_POS[1], M.TOP]), M.rx(math.radians(20)))
        o["blade"] = st(*SEAT)  # (blade pose kept where it was freed; the remover's tilt alone is what is tested)
        steps.append(o)
    s = run(steps)
    check(s[-1] == 0.7 and not M.remover_upright(steps[-1]), "G6 remover knocked over after the catch: 0.7, not 1.0")
    # 7. still holding the handle in the tray: no full credit
    fresh()
    run([world(handle=SEAT, btn=0.0), world(handle=SEAT, btn=0.003)])
    hold = M.hand_for(*LAID)
    s = run([world(handle=LAID, blade=SEAT, hands={"right_gripper": hold}) for _ in range(M.HOLD_STEPS + 2)])
    check(s[-1] == 0.7, "G7 bare handle in the tray but still held: 0.7")
    # 8. the button pressed first and held, then the blade pushed in: no catch (the press must come after)
    fresh()
    run([world(handle=(M.insert_origin(back=0.010), M.R_FLAT), btn=0.003), world(handle=SEAT, btn=0.003),
         world(handle=OUT, btn=0.003)])
    check(M.WELDS.state("lug") and M.blade_on(world(handle=OUT)), "G8 button held before the blade arrived: blade stays on")


def test_arm_contact_and_press():
    """Every arm link's collision shape (upper arm to fingertips) is checked for contact with the blade; the press,
    aimed with the fingertips' collision shapes, lands inside the button."""
    fresh()
    o = world(handle=OUT)  # the scalpel held level, out of the slot
    ob, Rb = M.pose(o, "blade")
    top = ob + Rb @ np.array([0.005, 0.0, M.B_Z0 + M.B_T])  # a point on the real blade's upper face
    pts, c, r = M.mesh("link5")
    low = pts[np.argmin(pts[:, 2])]  # link5's lowest point, turned as in its own frame
    M.LINK_POSES.clear()
    M.LINK_POSES["right_robot/link5"] = (top + np.array([0, 0, 0.0001]) - low, np.eye(3))
    s = run([world(handle=OUT)])
    check(s[-1] == 0.0 and "link5" in M._L.ruined, "the wrist (link5) resting on the blade: 0 (latched)")
    fresh()
    M.LINK_POSES["right_robot/link5"] = (top + np.array([0, 0, 0.004]) - low, np.eye(3))
    run([world(handle=OUT)])
    check(not M._L.ruined, "the wrist 4 mm over the blade: no contact")
    M.LINK_POSES.clear()
    press = M.remover_top(M.BTN_Y, M.BTN_TRAVEL - M.BTN_PROUD)
    W = M.tips_world(M.DOWN_Y, press)
    cap = np.array([M.REMOVER_POS[0], M.REMOVER_POS[1] + M.BTN_Y, M.TOP + M.R_H])
    low_z = W[:, 2].min()
    band = W[W[:, 2] <= low_z + M.BTN_PROUD] - cap
    check(abs(band[:, 0]).max() < M.BTN[0] / 2 - 1e-3 and abs(band[:, 1]).max() < M.BTN[1] / 2 - 1e-3,
          f"the fingertips' lowest 3.5 mm lie inside the 20 x 14 mm button (x {band[:, 0].min() * 1e3:.1f}..{band[:, 0].max() * 1e3:.1f}, "
          f"y {band[:, 1].min() * 1e3:.1f}..{band[:, 1].max() * 1e3:.1f} mm)")
    check(M.PRESS_PAST > M.BTN_TRAVEL, "the press is commanded past the button's stop (the arm gives under load)")
    short = (M.insert_origin(back=0.0012), M.R_FLAT)
    fresh()
    run([world(handle=short, btn=0.0), world(handle=short, btn=0.003)])
    check(M.WELDS.state("lug"), "1.2 mm short of the stop (not seated): the press does not free the blade")


def test_hand_boxes_hold_the_hulls():
    """guardkit's gripper boxes (closed) are fitted at run time to the robot's collision hulls: every hull point of the
    gripper and both fingertips lies in one of them."""
    import yamgeom as yg
    from yam_arena.constants import TCP_OFFSET

    p = np.vstack([yg.points("gripper") - np.asarray(TCP_OFFSET), yg.tcp_points("tip_left"), yg.tcp_points("tip_right")])
    inside = np.zeros(len(p), bool)
    for bx in M.gk.hand_boxes(0.0):
        inside |= np.all(np.abs(p - np.asarray(bx.center)) <= np.asarray(bx.size) / 2 + 1e-9, axis=1)
    check(inside.all(), f"gripper hull points outside guardkit's boxes: {int((~inside).sum())} of {len(p)}")


def test_hull_samples_cover_the_surface():
    """The contact threshold leans on the sampling: random points on the outer surface of the upper arm's, the wrist's and
    a fingertip's collision hulls (area-weighted, none inside another hull of the link) all lie closer to a sample point than
    TOUCH_TOL, so a real touch reads under the threshold."""
    import yamgeom as yg
    from scipy.spatial import cKDTree

    rng = np.random.default_rng(0)
    for link in ("link3", "link5", "tip_left"):
        hs = yg.hulls(link)
        found = []
        for i, (_, tri, _) in enumerate(hs):
            a = np.linalg.norm(np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0]), axis=1) / 2
            k = rng.choice(len(tri), 3000, p=a / a.sum())
            r = np.sqrt(rng.random(3000))[:, None]
            w = rng.random(3000)[:, None]
            p = tri[k, 0] * (1 - r) + tri[k, 1] * r * (1 - w) + tri[k, 2] * r * w
            for j, (_, _, planes) in enumerate(hs):
                if j != i:
                    p = p[(p @ planes[:, :3].T - planes[:, 3]).max(axis=1) > -2e-5]
            found.append(p)
        d = cKDTree(yg.points(link)).query(np.vstack(found))[0]
        check(d.max() < M.TOUCH_TOL, f"{link}: every surface point within {d.max() * 1000:.2f} mm of a sample "
                                     f"(99th percentile {np.percentile(d, 99) * 1000:.2f} mm), under TOUCH_TOL")


def _hull_lid_over_blade(o, link, gap):
    """The pose (pos, R, world) of arm link `link` with a face of its collision hulls that bridges a pocket of its
    visual mesh (yamgeom.proud_points) facing down onto the real blade's upper face, turned about the blade's normal
    and lowered until its hulls are `gap` m over the blade: the turn that keeps the visual mesh farthest from the
    blade, with the hulls' and the visual mesh's gaps (m)."""
    import trimesh
    import yamgeom as yg
    from scipy.spatial.transform import Rotation

    ob, Rb = M.pose(o, "blade")
    lo, hi = np.array(M.B_RLO), np.array(M.B_RHI)
    hull = yg.points(link)
    vis, _ = trimesh.sample.sample_surface_even(trimesh.Trimesh(yg.visual_vertices(link), yg._visual_faces(link)),
                                                200000, seed=0)
    best = None
    for q, n, _ in yg.proud_points(link):
        for yaw in np.linspace(0.0, 2 * math.pi, 24, endpoint=False):
            R = Rotation.from_euler("z", yaw).as_matrix() @ yg.turn_to(n, (0.0, 0.0, -1.0))  # in the blade's frame
            pos = np.array([(lo[0] + hi[0]) / 2, 0.0, hi[2] + 0.01]) - R @ q
            h = hull @ R.T + pos
            over = np.all((h[:, :2] >= lo[:2]) & (h[:, :2] <= hi[:2]), axis=1)
            if not over.any():
                continue
            pos = pos - np.array([0.0, 0.0, float((h[over, 2] - hi[2]).min()) - gap])
            hg = float(M._box_gap(hull @ R.T + pos, lo, hi).min())
            vg = float(M._box_gap(vis @ R.T + pos, lo, hi).min())
            if best is None or vg > best[3]:
                best = (ob + Rb @ pos, Rb @ R, hg, vg)
    return best


def test_hull_touch_visual_clear():
    """Anti-gaming: contact is judged on the robot's collision hulls (from the environment, at run time), not its
    visual meshes. A wrist or upper arm whose hulls come within 0.1 mm of the real blade, at a face that bridges a pocket
    of its visual mesh, touches it (the trial is 0), though its visual mesh stays more than TOUCH_TOL (0.6 mm) clear of
    the blade, so a check on the visual mesh would miss it."""
    for link in ("link5", "link3"):
        fresh()
        o = world(handle=OUT)
        pos, R, hg, vg = _hull_lid_over_blade(o, link, 0.0001)
        M.LINK_POSES.clear()
        M.LINK_POSES[f"right_robot/{link}"] = (pos, R)
        try:
            run([world(handle=OUT)])
        finally:
            M.LINK_POSES.clear()
        check(link in M._L.ruined and hg <= 0.0001 + 1e-9 and vg > M.TOUCH_TOL + 0.0001,
              f"{link}: hulls {hg * 1000:.2f} mm over the blade, visual mesh {vg * 1000:.1f} mm: a touch, 0")


def test_end_on_touch_at_blade_ends():
    """Anti-gaming (the per-step check's cull): each arm link's farthest collision-hull point 0.3 mm off the blade's
    tip or heel, end-on along the blade with the link beyond it, is a touch (the trial is 0); the scorer's arm_gap
    (as the trial calls it, upto 1 cm) equals a full scan of the link's points."""
    import yamgeom as yg

    lo, hi = np.array(M.B_RLO), np.array(M.B_RHI)
    for link in M.ARM_LINKS:
        pts, c, r = M.mesh(link)
        far = pts[int(np.argmax(np.linalg.norm(pts - c, axis=1)))]
        for end, u in ((hi[0], 1.0), (lo[0], -1.0)):
            fresh()
            o = world(handle=OUT)
            ob, Rb = M.pose(o, "blade")
            R = yg.turn_to(far - c, (-u, 0.0, 0.0))  # in the blade's frame: the far point faces the blade's end
            pos = np.array([end + u * 0.0003, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2]) - R @ far
            full = float(M._box_gap(pts @ R.T + pos, lo, hi).min())
            M.LINK_POSES.clear()
            M.LINK_POSES[f"right_robot/{link}"] = (ob + Rb @ pos, Rb @ R)
            try:
                gap, _ = M.arm_gap(o, upto=0.01)
                run([world(handle=OUT)])
            finally:
                M.LINK_POSES.clear()
            check(abs(gap - full) < 1e-9 and full < M.TOUCH_TOL and link in M._L.ruined,
                  f"{link} end-on {'tip' if u > 0 else 'heel'} side: full scan {full * 1000:.2f} mm, arm_gap "
                  f"{gap * 1000:.2f} mm: a touch, 0")


class _Arr:
    """A stand-in for a simulator tensor: .detach().cpu().numpy()."""

    def __init__(self, a):
        self.a = np.asarray(a, float)

    def __getitem__(self, i):
        return _Arr(self.a[i])

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.a


class _Art:
    def __init__(self, names):
        self.body_names = list(names)
        self.data = type("D", (), {"body_pos_w": _Arr(np.zeros((1, len(names), 3))),
                                   "body_quat_w": _Arr(np.tile([0.0, 0.0, 0.0, 1.0], (1, len(names), 1)))})()


class _Base:
    def __init__(self, arts):
        self.scene = type("S", (dict,), {})(arts)
        self.scene.env_origins = _Arr(np.zeros((1, 3)))


def test_sim_link_read_is_strict():
    """In the simulator the blade-contact check reads all 12 arm-link poses (6 links x 2 arms) or stops with an error:
    a missing robot or link, or no poses at all, never leaves links unchecked or falls back to the box model."""
    full = ["base", "link1", "link2"] + list(M.ARM_LINKS)
    M.LINK_POSES.clear()
    M._read_links(_Base({r: _Art(full) for r in M.ROBOTS}))
    check(set(M.LINK_POSES) == {f"{r}/{k}" for r in M.ROBOTS for k in M.ARM_LINKS},
          f"{len(M.LINK_POSES)} link poses read, all of them")
    for arts, what in (({M.ROBOTS[0]: _Art(full)}, "a robot missing"),
                       ({r: _Art([n for n in full if n != "tip_right"]) for r in M.ROBOTS}, "tip_right missing"),
                       ({M.ROBOTS[0]: _Art(full), M.ROBOTS[1]: _Art([n for n in full if n != "link4"])},
                        "one arm's link4 missing")):
        M.LINK_POSES.clear()
        try:
            M._read_links(_Base(arts))
            raised = ""
        except RuntimeError as e:
            raised = str(e)
        check(raised.startswith("[scalpel] blade contact:"), f"{what}: raises ({raised[:70]}), never skips")
    M.LINK_POSES.clear()
    fresh()
    M._IN_SIM["on"] = True
    try:
        run([world(handle=OUT)])
        raised = ""
    except RuntimeError as e:
        raised = str(e)
    finally:
        M._IN_SIM["on"] = False
        M._IN_SIM["told"] = False
    check(raised.startswith("[scalpel] blade contact: no arm link poses"),
          "in the sim, no link poses: raises, never the box model")
    fresh()
    s = run([world(handle=OUT)])
    check(len(s) == 1 and not M._L.ruined, "offline (the tests only): the box model runs with no link poses")


def test_robot_usd_matches_urdf_hulls():
    """warm() (run when the scene module is patched, with the app up) checks that the robot USD the simulator loads has
    the URDF's collision hulls for every arm link; hulls()/points() read only the URDF, so the demo plans without pxr."""
    import yamgeom as yg

    counts = yg.check()
    check(all(counts.get(k, 0) >= 1 for k in M.ARM_LINKS), f"robot USD hulls match the URDF's: {counts}")


def test_redteam():
    """The red-team cases (Nov 17): each scores what the real act deserves."""
    # R1: the remover, blade inside, carried into the instrument tray with the bare handle: not at its place: 0.7 at most
    fresh()
    run([world(handle=SEAT, btn=0.0), world(handle=SEAT, btn=0.003)])
    r0 = np.array([M.REMOVER_POS[0], M.REMOVER_POS[1], M.TOP])
    r1 = np.array([M.TRAY_POS[0] + 0.045, M.TRAY_POS[1], M.TRAY_FLOOR])
    blade = (SEAT[0] + (r1 - r0), SEAT[1])
    lay = (M.tray_origin() + np.array([-0.035, 0, 0]), M.R_FLAT)
    s = run([world(handle=lay, blade=blade, remover=r1) for _ in range(M.HOLD_STEPS + 3)])
    check(s[-1] == 0.7, "R1 remover carried into the tray with the handle: 0.7, not 1.0")
    # ... and standing on the table but 6 cm off its marks, or 2 mm up: not at its place
    o = world(remover=r0 + np.array([0.06, 0, 0]))
    check(not M.remover_upright(o), "R1 remover 6 cm off its tape marks: not at its place")
    o = world(remover=r0 + np.array([0, 0, 0.002]))
    check(not M.remover_upright(o), "R1 remover 2 mm off the table top: not standing on the table")
    # R2: contact means touching the real 0.4 mm blade: a wrist 1.3 mm clear of it is no touch, 0.1 mm is
    o = world(handle=OUT)
    ob, Rb = M.pose(o, "blade")
    real_top = ob + Rb @ np.array([0.005, 0.0, M.B_Z0 + M.B_T])
    pts, c, r = M.mesh("link5")
    low = pts[np.argmin(pts[:, 2])]
    for gap, touch in ((0.0013, False), (0.0009, False), (0.0001, True)):
        fresh()
        M.LINK_POSES.clear()
        M.LINK_POSES["right_robot/link5"] = (real_top + np.array([0, 0, gap]) - low, np.eye(3))
        run([world(handle=OUT)])
        check(bool(M._L.ruined) == touch, f"R2 wrist {gap * 1000:.1f} mm over the real blade: {'touch' if touch else 'no touch'}")
    M.LINK_POSES.clear()
    # R3: blade side down, seated and pressed: the remover takes it blade side up only
    fresh()
    seat_dn = (M.insert_origin() + np.array([0, 0, 2 * M.B_Z0 + M.B_TC]), M.R_FLAT @ M.rx(math.pi))
    run([world(handle=seat_dn, btn=0.0), world(handle=seat_dn, btn=0.003)])
    check(M.WELDS.state("lug"), "R3 blade side down at the stop: the press does not free it")
    # R4: the press frees the blade at 2.5 mm of button travel, not before
    fresh()
    run([world(handle=SEAT, btn=0.0), world(handle=SEAT, btn=0.0024)])
    check(M.WELDS.state("lug"), "R4 button 2.4 mm down: not yet")
    run([world(handle=SEAT, btn=0.0025)])
    check(not M.WELDS.state("lug"), "R4 button 2.5 mm down: freed")
    # R5: the remover held off the table (it is a hand-held tool): the press still works
    fresh()
    up = np.array([0, 0, 0.03])
    run([world(handle=(SEAT[0] + up, SEAT[1]), remover=r0 + up, btn=0.0),
         world(handle=(SEAT[0] + up, SEAT[1]), remover=r0 + up, btn=0.003)])
    check(not M.WELDS.state("lug"), "R5 remover held 30 mm up: the press frees the blade")
    # R6: the bladed scalpel slipping 12 mm onto the table (0.48 m/s) is not a drop; 0.8 m/s is (test_hazards)
    fresh()
    h = M.H_START
    run([world(handle=(h + np.array([0, 0, 0.024]), M.R_START)), world(handle=(h, M.R_START))])
    check(not M._L.ruined, "R6 the bladed scalpel set down 24 mm in one step (0.48 m/s): no drop")




if __name__ == "__main__":
    for t in (test_start, test_insert_pose, test_mechanism_and_tiers, test_no_catch_paths, test_hazards, test_gaming, test_arm_contact_and_press, test_redteam,
              test_hand_boxes_hold_the_hulls, test_hull_touch_visual_clear, test_hull_samples_cover_the_surface,
              test_end_on_touch_at_blade_ends, test_sim_link_read_is_strict,
              test_robot_usd_matches_urdf_hulls):
        t()
    if (HERE / "task.json").exists() and (HERE / "README.md").exists():
        test_text()
    print("all pass")
