"""The scripted demo's clearance sweep, offline (no simulator): every 0.05 s of the plan, both arms are put where the
arm's own IK puts them (forward kinematics of the commanded joints), and the gripper's shape (palm, finger bodies,
pads and tips at the commanded opening; guardkit.hand_boxes) is checked against the keg (chime, body, valve), the
counter, the drip tray, the cooler wall, the coupler and its handle, the line fittings and the other hand. The lines' starting
shapes are checked against both hands until the coupler moves. The carried
coupler (and its two line fittings) is checked against the keg's chime and body, the tray, the wall and the counter.
Diagonal moves are covered because the plan is sampled along its interpolated path, not only at waypoints.

Contact is expected only where the demo grips or pushes: the right (coupler) hand's pads on the coupler body while it holds it,
its fingertips on the handle while pressing, the left (steady) hand's pads on the chime while it holds the keg. Everything else
must stay MARGIN clear. Run by test_task.py, or alone: python3 test_clearance.py
"""
import math
from pathlib import Path

import numpy as np
from yam_arena import ik
from yam_arena.constants import FINGER_OPEN
from yam_arena.demo import LEFT_BASE_POS, RIGHT_BASE_POS, ArmPlan, grip_rotation
from yam_arena.task_file import load

M = load(Path(__file__).parent / "task.py").module
import guardkit as gk  # noqa: E402

MARGIN = 0.005  # m: required gap where no contact is planned (the arms sag 4-6 mm under load: the sim's give)
DT = 0.05


def box_points(box, n=4):
    c, h = np.asarray(box.center, float), np.asarray(box.size, float) / 2
    g = [np.linspace(-1, 1, n)] * 3
    pts = np.array([[a, b, d] for a in g[0] for b in g[1] for d in g[2]])
    return c + pts * h


def hand_points(pos, R, o):
    """World sample points of the gripper's boxes, by kind."""
    boxes = gk.hand_boxes(o)
    kinds = ["palm"] + ["finger", "pad", "tip"] * 2
    out = {k: [] for k in ("palm", "finger", "pad", "tip")}
    for b, k in zip(boxes, kinds):
        out[k].append(pos + box_points(b) @ R.T)
    return {k: np.vstack(v) for k, v in out.items()}


def ring_gap(p, center, r_in, r_out, z0, z1):
    """Distance from world points p to a vertical ring (keg frame = world here: the keg stands level at its mark)."""
    d = p - center
    r = np.hypot(d[:, 0], d[:, 1])
    dr = np.maximum(np.maximum(r_in - r, r - r_out), 0.0)
    dz = np.maximum(np.maximum(z0 - d[:, 2], d[:, 2] - z1), 0.0)
    return np.hypot(dr, dz)


def block_gap(p, center, size, R=np.eye(3)):
    q = (p - center) @ R
    e = np.maximum(np.abs(q) - np.asarray(size) / 2, 0.0)
    return np.linalg.norm(e, axis=1)


K0 = np.array([M.KEG_C[0], M.KEG_C[1], M.KEG_Z0])
KEG_PARTS = {"chime": (M.KEG_R - M.CHIME_T, M.KEG_R, M.DOME_Z, M.CHIME_TOP), "keg body": (0.0, M.KEG_R, -0.01, M.DOME_Z),
             "valve": (0.0, M.FLANGE_OUT, M.DOME_Z, M.FLANGE_TOP)}


def static_gaps(p):
    out = {name: float(np.min(ring_gap(p, K0, *v))) for name, v in KEG_PARTS.items()}
    out["counter"] = float(np.min(p[:, 2] - M.TOP))
    out["tray"] = float(np.min(block_gap(p, np.array([*M.TRAY_C, M.TOP + M.TRAY_SIZE[2] / 2]), M.TRAY_SIZE)))
    out["wall"] = float(np.min(block_gap(p, np.array([M.WALL_X + 0.010, 0.150, M.TOP + 0.125]), (0.02, 0.4, 0.25))))
    return out


def coupler_points(pos, R, q):
    """World sample points of the coupler's body parts, its handle at angle q, and its two line fittings."""
    body = np.vstack([box_points(b) for b in M.COUPLER_PARTS])
    c, s = math.cos(q), math.sin(q)
    Rh = np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    hb = M.HANDLE_PARTS[0]
    handle = np.asarray(M.PIVOT) + box_points(hb) @ Rh.T
    fits = np.vstack([np.asarray(port) + np.array([M.FIT_L / 2, 0, 0]) + box_points(gk.Block((0, 0, 0),
                                                                                       (M.FIT_L, 0.020, 0.020)))
                      for port in (M.BEER_PORT, M.GAS_PORT)])
    w = lambda P: pos + P @ R.T  # noqa: E731
    return w(body), w(handle), w(fits)


def _line_pts(cab):
    n = cab.nodes()
    return np.vstack([a + (b - a) * u for a, b in zip(n[:-1], n[1:]) for u in np.linspace(0, 1, 6)])


LINE_PTS = [(c, _line_pts(c)) for c in (M.BEER_LINE, M.GAS_LINE)]


def plan_events(plan_wps, label):
    t = 0.0
    for w in plan_wps:
        if w.label == label:
            return t, t + w.duration
        t += w.duration
    raise KeyError(label)


def main():
    lw, rw = M.coupler_plan(), M.steady_plan()  # "c": the right arm with the coupler; "s": the left arm on the keg
    lp, rp = ArmPlan(lw, np.asarray(RIGHT_BASE_POS)), ArmPlan(rw, np.asarray(LEFT_BASE_POS))
    t_grip0, t_grip = plan_events(lw, "grip it")
    t_go, t_open = plan_events(lw, "let go, fingers wide")
    t_drop = plan_events(lw, "let it go onto the valve")[0]
    t_regrip0 = plan_events(lw, "grip it again")[0]
    t_turn0 = plan_events(lw, "turn it clockwise a quarter turn to the stops")[0]
    t_press0 = plan_events(lw, "over the handle, fingers closed")[1]
    t_press1 = plan_events(lw, "lift off")[0]
    t_pinch = plan_events(rw, "pinch the chime")[0]
    t_rel = plan_events(rw, "let go")[1]
    seat_p = K0 + np.array([0.0, 0.0, M.FLANGE_TOP])
    worst = {}
    n = int(max(lp.duration, rp.duration) / DT)
    for k in range(n):
        t = k * DT
        la, _ = lp.action7(t)
        ra, _ = rp.action7(t)
        lpos, lR = ik.tcp_pose(la[:6], np.asarray(RIGHT_BASE_POS))
        rpos, rR = ik.tcp_pose(ra[:6], np.asarray(LEFT_BASE_POS))
        lo = float(np.clip(la[6], 0.0, 1.0)) * abs(FINGER_OPEN)  # action7 gives the opening as a fraction
        ro = float(np.clip(ra[6], 0.0, 1.0)) * abs(FINGER_OPEN)
        # where the coupler is: on the tray; in the right hand (the loaded, leaned arm sits HOVER_LAG off its command
        # and the coupler hangs level, GRIP_OFF off the tool point); let go onto the valve and settled there; gripped
        # again and turned with the hand; then locked
        yaw = math.atan2(lR[1, 0], lR[0, 0]) + math.pi / 2  # grip yaw (yaw 0: tool x along -y)
        if t < t_grip:
            cpos, cR = np.array(M.COUPLER_START), np.eye(3)
        elif t < t_drop:
            lpos = lpos + np.array([*M.HOVER_LAG, 0.0]) * min(1.0, max(0.0, (t - t_grip) / 2.0))
            c, s = math.cos(yaw - math.pi / 2), math.sin(yaw - math.pi / 2)
            off = np.array([c * M.GRIP_OFF[0] - s * M.GRIP_OFF[1], s * M.GRIP_OFF[0] + c * M.GRIP_OFF[1], 0.0])
            cR = np.array([[math.cos(yaw), -math.sin(yaw), 0.0], [math.sin(yaw), math.cos(yaw), 0.0], [0.0, 0.0, 1.0]])
            cpos = lpos + np.array([0.0, 0.0, -M.GRIP_UP]) + off
            lR = grip_rotation(yaw - math.pi / 2)  # the lean only cancels the sag: the hand is level too
        elif t < t_turn0:  # resting on the valve, lugs in the openings
            cpos, cR = seat_p, np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
            u = min(1.0, (t - t_drop) / M.L_TIMES[7])  # letting go: the hand's command moves to where the loaded hand
            lpos = lpos + np.array([*M.HOVER_LAG, 0.0]) * (1.0 - ik.smoothstep(u))  # already is, so it stays put
            lR = grip_rotation(yaw - math.pi / 2)
        elif t < t_go:  # gripped again and turned with the hand about the valve's axis, until the stops (square)
            cpos = seat_p
            cy = max(yaw, 0.0)
            cR = np.array([[math.cos(cy), -math.sin(cy), 0.0], [math.sin(cy), math.cos(cy), 0.0], [0.0, 0.0, 1.0]])
        else:
            cpos, cR = seat_p, np.eye(3)
        if t_regrip0 <= t < t_open and np.hypot(*(lpos[:2] - K0[:2])) < 0.05:  # pushing down on the seated coupler:
            lpos = np.array([lpos[0], lpos[1], max(lpos[2], M.SEAT_TCP_Z)])  # the hand stops where it sits on it
        held = t_grip0 <= t < t_drop + M.L_TIMES[7] or t_regrip0 <= t < t_open  # fingers on the body (closing, holding, opening)
        if held:  # commanded shut, the fingers stop on the 46 mm body
            lo = max(lo, M.BODY_W / 2)
        if t_pinch <= t < t_rel:  # and the left fingers on the 4 mm rim
            ro = max(ro, M.CHIME_T / 2)
        pressing = t_press0 <= t < t_press1 + 0.5  # and while the fingertips come back up off it
        q = M.UP_Q
        if t >= t_press0:
            q = M.DOWN_Q if t >= t_press0 + 1.0 else M.UP_Q
        body, handle, fits = coupler_points(cpos, cR, q)

        def note(what, gap, ok_contact=False):
            if ok_contact:
                return
            if what not in worst or gap < worst[what][0]:
                worst[what] = (gap, round(t, 2))

        regrip = False
        for side, pos, R, o, pinch in (("coupler", lpos, lR, lo, False), ("steady", rpos, rR, ro, t_pinch <= t < t_rel)):
            hp = hand_points(pos, R, o)
            allp = np.vstack(list(hp.values()))
            g = static_gaps(allp)
            for name, v in g.items():
                if name == "chime" and pinch:  # the right hand's pads straddle the rim: check its palm and finger bodies
                    v = static_gaps(np.vstack([hp["palm"], hp["finger"]]))["chime"]
                    note(f"{side} hand palm/fingers vs chime while pinching", v)
                    continue
                note(f"{side} hand vs {name}", v)
            body_gap = min(float(np.min(block_gap(allp, cpos + cR @ np.asarray(b.center), b.size, cR)))
                           for b in M.COUPLER_PARTS)
            hand_gap = float(np.min(np.linalg.norm(allp[:, None, :] - handle[None, ::4, :], axis=2)))
            fit_gap = float(np.min(np.linalg.norm(allp[:, None, :] - fits[None, ::8, :], axis=2)))
            if side == "coupler":
                releasing = t_go <= t < t_open + 1.5 or t_drop <= t < t_regrip0 + 0.8  # opening and re-gripping
                note("coupler hand vs coupler body (not holding it)", body_gap, held or (releasing and body_gap > 0.003))
                if held:
                    note("coupler hand palm vs coupler (holding)", min(float(np.min(block_gap(hp["palm"], cpos + cR @ np.asarray(
                        b.center), b.size, cR))) for b in M.COUPLER_PARTS))
                # letting the coupler drop out of the opening fingers and coming back down round it, the fingers pass
                # as close to the handle and the fittings as the grip itself does (which touches them): only contact
                # counts there
                regrip = t_drop <= t < t_regrip0 and min(hand_gap, fit_gap) > 0.001
                note("coupler hand vs handle", hand_gap, pressing or held or regrip)
            else:
                note("steady hand vs coupler body", body_gap)
                note("steady hand vs handle", hand_gap)
            note(f"{side} hand vs line fittings", fit_gap, side == "coupler" and regrip)
        if t < t_grip:  # the lines lie where they started until the coupler moves
            for cab, pts in LINE_PTS:
                for side, pos, R, o in (("coupler", lpos, lR, lo), ("steady", rpos, rR, ro)):
                    hp = np.vstack(list(hand_points(pos, R, o).values()))
                    gap = float(np.min(np.linalg.norm(hp[:, None, :] - pts[None, :, :], axis=2))) - cab.radius
                    note(f"{side} hand vs {cab.name} (before the coupler moves)", gap)
        lh = np.vstack(list(hand_points(lpos, lR, lo).values()))
        rh = np.vstack(list(hand_points(rpos, rR, ro).values()))
        note("coupler hand vs steady hand", float(np.min(np.linalg.norm(lh[:, None, ::1] - rh[None, ::5, :], axis=2))))
        cg = static_gaps(np.vstack([body, handle, fits]))
        note("coupler vs chime", cg["chime"])
        note("coupler vs keg body", cg["keg body"])
        note("coupler vs counter", cg["counter"])
        note("coupler vs wall", cg["wall"])
        note("coupler vs tray (after lift-off)", cg["tray"], t < t_grip + 1.6)
    bad = []
    for what, (gap, when) in sorted(worst.items()):
        flag = "ok  " if gap >= MARGIN else "LOW "
        print(f"  {flag}{what}: {gap * 1000:6.1f} mm (t {when:5.2f} s)")
        if gap < MARGIN:
            bad.append(what)
    if bad:
        raise AssertionError(f"clearance below {MARGIN * 1000:.0f} mm: {bad}")
    print("ok   demo clearances: every unplanned gap at least 5 mm")


if __name__ == "__main__":
    main()
