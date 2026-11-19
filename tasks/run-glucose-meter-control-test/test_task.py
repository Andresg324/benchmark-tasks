"""Offline checks of the glucose-meter control test's scorer and demo geometry, no simulator. Poses are set by hand, as
the simulator reports them each control step. Covers: doing nothing (0); the start layout (strip standing in its bottle,
not seated; meter down; bottle in its rest, capped); the demo's strip pinch matching the strip's spawn pose; the
insert pose seating the strip (and 2 mm short not); the dip pose putting only the strip's tip in the drop (and a flat
dip smearing it); drop landing zones (slide, counter, strip); the full sequence of tiers 0.25 -> 0.5 -> 0.75 -> 1.0
with the 1 s hold; a strip pulled out during the countdown; a smear; the strip touching the bottle's tip; a dropped
meter; the cap seated on the nozzle vs beside it; the bottle in its rest vs on or across a slide stack, on end,
held up or out in front (and the demo's logged laid-back pose); cheats: squeezing the capped bottle, the strip on
the nozzle or in a hanging drop, a drop put beside the strip, the strip upside down, a hand lying on the meter, a
smear after the result; each object's friction; the thresholds in the text.

    python3 test_task.py
"""
import json
import math
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import dropkit as dk  # noqa: E402
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


def joints(roller=0.0, squeeze=0.0):
    kit._JOINTS["meter/contact"] = kit.JointState(pos=roller, vel=0.0, latched=False)
    for name, sg in (("pad_l", -1), ("pad_r", 1), ("pad_t", -1), ("pad_b", 1)):
        kit._JOINTS[f"bottle/{name}"] = kit.JointState(pos=sg * squeeze, vel=0.0, latched=False)


def bottle_start():
    return np.array(M.BOTTLE_START), M.B_ROT0


def cap_start():
    """The cap as it spawns: on the bottle's nozzle."""
    return np.array(M.CAP.pos), dk._rot(np.asarray(M.CAP.tilt_xyzw, float))


def cap_set_down():
    """The cap once the robot has taken it off: open side down on its mark."""
    return np.array([M.CAP_START[0], M.CAP_START[1], M.TOP]), np.eye(3)


def strip_in(meter_o, R_m=np.eye(3), short=0.0):
    """The strip seated in a meter at (meter_o, R_m), its gray end `short` m from the back stop."""
    Rs = R_m  # strip frame aligned with the meter's (x along the port, faces up)
    gray = np.asarray(meter_o) + R_m @ np.array([M.X_STOP - short, 0.0, M.Z_SLOT])
    return gray - Rs @ np.array([M.S_L / 2, 0.0, 0.0]), Rs


def world(meter=None, strip=None, bottle=None, cap=None, grippers=AWAY, roller=0.0, squeeze=0.0):
    joints(roller, squeeze)
    m = meter or (np.array([M.METER_START[0], M.METER_START[1], M.TOP]), np.eye(3))
    s = strip or (np.array(M.STRIP_START), M._S_ROT0)
    b = bottle or bottle_start()
    c = cap or cap_start()
    out = {"meter": st(*m), "strip": st(*s), "bottle": st(*b), "cap": st(*c)}
    for g, p in grippers.items():
        out[g] = st(p)
    kit._OPEN_TARGET.clear()
    if squeeze > 0.0:  # the right hand pinching the bottle's body at GRIP_X, commanded to give that wall travel
        out["right_gripper"] = pinch_bottle(*b)
        push = squeeze * M.WALL_K * (M.K_F + M.WALL_K) / (M.K_F * M.WALL_K)
        kit._OPEN_TARGET["right_gripper"] = max(0.0, (M.B_R - push) / M.F_HALF_OPEN)
    return out


_TOUCH = {}


def pinch_bottle(center, R_b):
    """The right hand around the bottle's body at GRIP_X (turned with the bottle), fingertips on the wall: sets its
    actual opening (found once, with the robot's hulls) and returns its state."""
    import handkit as hk
    R_hand = R_b @ M.B_ROT0.T @ M.B_GRIP_R
    tcp = M.bottle_tcp(center, R_b, R_hand)
    if "op" not in _TOUCH:
        lo, hi = 0.0, 1.0
        for _ in range(30):
            mid = (lo + hi) / 2
            kit._OPENING["right_gripper"] = mid
            w = {"bottle": st(center, R_b), "right_gripper": st(tcp, R_hand)}
            g = hk.gap(w, "right_gripper", "bottle", M.BODY_SHAPE, links=("tip_left", "tip_right"))
            lo, hi = (mid, hi) if min(g.values()) < 0 else (lo, mid)
        _TOUCH["op"] = hi
    kit._OPENING["right_gripper"] = _TOUCH["op"]
    return st(tcp, R_hand)


def run(frames):
    """Feed frames (each a dict) step by step; return the score after the last."""
    s = 0.0
    for f in frames:
        s = M.score(f)
    return s


def put_drop(at, step=1):
    dk._fresh_if_new()
    dk._S.drops.append(dk.Drop(step, np.array([at[0], at[1], M.SLIDE_TOP + 0.012]), "slide",
                               np.array([at[0], at[1], M.SLIDE_TOP]), {}))


def capped_cap(b_center, R_b):
    rim = np.asarray(b_center) + R_b @ np.array([M.B_SHOULDER, 0.0, 0.0])
    Rc = R_b @ np.array([[0.0, 0.0, 1.0], [0.0, 1.0, 0.0], [-1.0, 0.0, 0.0]])  # cap z along the bottle's x
    return rim, Rc


def main():
    # -- nothing done ----------------------------------------------------------------------------------------------
    fresh()
    check(run([world() for _ in range(40)]) == 0.0, "doing nothing scores 0")
    w0 = world()
    check(not M.seated(w0), "start: the strip is not seated")
    check(M.meter_down(w0) and M.bottle_in_rest(w0), "start: the meter down, the bottle in its rest")
    check(M.capped(w0), "start: the bottle is capped (the cap spawns seated on the nozzle)")
    check(not M.capped(world(cap=cap_set_down())), "the cap on its mark: not capped")
    check(M.released(w0), "start: hands clear")

    # -- the strip pinch matches the spawn pose ---------------------------------------------------------------------
    g = M.strip_grasp_tcp()
    Rs, off = M.strip_rel(M.STRIP_PICK_R)
    check(np.allclose(Rs, M._S_ROT0) and np.allclose(g + off, M.STRIP_START, atol=1e-6),
          "the demo's pinch holds the strip where it stands")
    tips_z = g[2] - 0.013
    check(M.TOP + M.V_H + 0.0005 < tips_z < M.TOP + M.V_H + M.PROTRUDE, "the pad tips stay over the bottle's rim")
    Rs_i, off_i = M.strip_rel(M.INSERT_R)
    check(np.allclose(Rs_i[:, 0], [1, 0, 0]) and abs(abs(Rs_i[2, 2]) - 1) < 1e-9,
          "turned for the port: gray end forward, faces up and down")
    seat_tcp = M.insert_tcp(M.M_INS)
    s_c = seat_tcp + off_i
    fresh()
    w = world(meter=(M.M_INS, np.eye(3)), strip=(s_c, Rs_i))
    check(M.seated(w) and abs(M.seat_depth(w)) < 1e-6, "the insert pose puts the gray end on the back stop")
    w = world(meter=(M.M_INS, np.eye(3)), strip=(s_c - np.array([0.002, 0, 0]), Rs_i))
    check(not M.seated(w), "2 mm short of the back stop is not seated")
    pre = M.insert_tcp(M.M_INS, back=0.015) + off_i + Rs_i @ np.array([M.S_L / 2, 0, 0])
    check(pre[0] < M.M_INS[0] + M.X_FACE - 0.005, "the line-up pose keeps the strip out of the port")
    tips = seat_tcp + M.INSERT_R[:, 2] * 0.013
    check(tips[0] < M.M_INS[0] + M.X_FACE - 0.005, "seated, the pad tips stay 5 mm clear of the meter's end face")

    # the strip: a face pinch, a second push after a re-grip (probes: one push could leave it 2.5 mm short), and a
    # release that leaves it in place (the edge pinch of pf-155056..pf-170837 carried it away on a finger)
    check(M.STRIP_PICK_R is M.DOWN_Y and M.STRIP_SQUEEZE == M.FINGER_CLOSED, "the strip is pinched on its faces")
    rel_pick = M.STRIP_PICK_R.T @ M._S_ROT0
    check(np.allclose(M.INSERT_R @ rel_pick, Rs_i) and np.allclose(Rs_i, np.eye(3)),
          "the strip turns with the hand from the pick to the insert, right side up")
    plan_r = M.right_plan(5.0)[0]
    labels = [getattr(w, "label", "") for w in plan_r]
    k1 = labels.index("push it fully in")
    seq = labels[k1:k1 + 9]
    check(seq == ["push it fully in", "hold it", "open", "slide back along the strip", "pinch it again",
                  "push it home again", "hold it in (the meter beeps)", "let go", "back the fingers off"],
          "push, open, slide back, re-pinch, push home, hold, let go in place, back off")
    seat_w, back_w = plan_r[k1], plan_r[k1 + 3]
    check(np.allclose(np.subtract(seat_w.pos, back_w.pos), [M.REGRIP_BACK, 0.0, 0.0]), "the re-grip slides straight back along the strip")
    short = 0.0025 + 0.0025  # m: a strip left 2.5 mm short, plus the 2.5 mm the loaded hand stalls short (probe p1)
    check(M.REGRIP_BACK >= short, "the second push travels far enough to seat a strip left 2.5 mm short")
    check(M.STRIP_PINCH - M.REGRIP_BACK >= 0.005, "re-pinched, the pads still hold at least 5 mm of the strip")
    rel_half = (M.STRIP_RELEASE - M.FINGER_CLOSED) / (M.FINGER_OPEN - M.FINGER_CLOSED) * M.F_HALF_OPEN
    check(rel_half > M.S_T / 2 + 0.003, "open, the faces stand 3 mm or more off the strip")
    let_go, off_w = plan_r[k1 + 7], plan_r[k1 + 8]
    check(np.allclose(let_go.pos, seat_w.pos) and let_go.finger == off_w.finger == M.STRIP_RELEASE,
          "the hand opens in place before it backs off, and stays open")
    k_in = labels.index("hold it in (the meter beeps)")
    t_in = sum(w.duration for w in plan_r[:k_in + 1])
    check(abs(M.right_plan(5.0)[1]["inserted"] - t_in) < 1e-9, "the meter hand waits for the second push")

    # -- the dip ------------------------------------------------------------------------------------------------------
    at = np.array([M.DROP_AT[0], M.DROP_AT[1], M.SLIDE_TOP])
    R_d = M.ry(M.DIP_TILT)
    mo0 = M.dip_meter_origin(at)
    tip0 = M.sample_tip(world(meter=(mo0, R_d), strip=strip_in(mo0, R_d)))
    check(np.allclose(tip0, [at[0], at[1], M.SLIDE_TOP + M.DIP_Z], atol=1e-6), "the dip pose aims the tip at the drop")
    rest_z = M.S_T / 2 * math.cos(abs(M.DIP_TILT)) + 0.0001  # the strip's lower corner resting on the glass
    mo = mo0 + np.array([0.0, 0.0, rest_z - M.DIP_Z])
    so = strip_in(mo, R_d)
    w = world(meter=(mo, R_d), strip=so)
    check(any(M.in_bead(p, at, M.TIP_IN_SLACK) for p in M._strip_tip_points(w)), "tip counted in the bead")
    hi = world(meter=(mo + np.array([0, 0, 0.0016]), R_d), strip=strip_in(mo + np.array([0, 0, 0.0016]), R_d))
    check(any(M.in_bead(p, at, M.TIP_IN_SLACK) for p in M._strip_tip_points(hi)), "1.6 mm higher, still in the bead")
    check(not any(M.in_bead(p, at) for p in M._strip_body_points(w)), "tip resting on the glass at 45 degrees: no other part is in the liquid")
    flat_o = np.array([at[0] - 0.004, at[1], M.SLIDE_TOP + 0.0004]) - np.array([M.X_STOP - M.S_L, 0.0, M.Z_SLOT])
    wf = world(meter=(flat_o, np.eye(3)), strip=strip_in(flat_o))
    check(any(M.in_bead(p, at) for p in M._strip_body_points(wf)), "laid flat in the drop, the strip's face smears it")
    low_corner = min((mo + R_d @ np.array([x, y, 0.0]))[2] for x in (-M.M_L / 2, M.M_L / 2) for y in (-0.014, 0.014))
    check(low_corner > M.SLIDE_TOP + 0.008, "dipping, the meter's body stays 8 mm over the glass")

    # -- closed-loop dip: the left hand's dip waypoints move onto the drop that landed (pf-162628, pb-204321) ----------
    from yam_arena import ik as _ik
    from yam_arena import station as _station
    plan = M.demo()
    dips = [w for w in plan["left"] if getattr(w, "label", "") in
            ("tip it port-end down over the drop", "strip's tip into the drop", "hold until it beeps", "lift it off")]
    nominal = [w.pos for w in dips]
    for landed in ((0.2253, 0.0406), (0.2283, 0.0363)):
        fresh()
        M.score(world())
        dk._fresh_if_new()
        dk._S.drops.append(dk.Drop(M._L.n, np.zeros(3), "slide", np.array([landed[0], landed[1], M.SLIDE_TOP]), {}))
        M.score(world())
        tcps = M.dip_tcps(np.array(landed), np.array([0.0, 0.0, M.M_LIFT_SAG]))
        check(np.allclose(dips[1].pos, tcps["dip"]) and np.allclose(dips[2].pos, tcps["dip"])
              and np.allclose(dips[0].pos, tcps["over"]) and np.allclose(dips[3].pos, tcps["lift"]),
              f"a drop seen at {landed}: the dip waypoints move onto it")
        at_l = np.array([landed[0], landed[1], M.SLIDE_TOP])
        mo_l = M.dip_meter_origin(at_l[:2]) + np.array([0, 0, M.S_T / 2 * math.cos(abs(M.DIP_TILT)) + 0.0001 - M.DIP_Z])
        w_l = world(meter=(mo_l, R_d), strip=strip_in(mo_l, R_d))
        check(any(M.in_bead(p, at_l, M.TIP_IN_SLACK) for p in M._strip_tip_points(w_l))
              and not any(M.in_bead(p, at_l) for p in M._strip_body_points(w_l)),
              f"there, the strip's tip is in that drop and no other part of it")
    fresh()
    M.score(world())
    check([w.pos for w in dips] == nominal, "a new trial puts the dip back to its nominal aim")
    base = _station.BASES["left"]
    R_tcp = M.ry(M.DIP_TILT) @ M.DOWN_Y
    q0 = np.array(_ik.solve(np.asarray(nominal[1]), _ik.matrix_to_quat_xyzw(R_tcp), np.zeros(6), base_pos=base,
                            iters=300)[0])
    worst = 0.0
    for dx in (-0.005, 0.0, 0.005):
        for dy in (-0.005, 0.0, 0.005):
            for role in ("over", "dip", "lift"):
                p_t = M.dip_tcps(np.array(M.DROP_AT) + [dx, dy], np.array([0.0, 0.0, M.M_LIFT_SAG]))[role]
                _q, pe, re = _ik.solve(p_t, _ik.matrix_to_quat_xyzw(R_tcp), q0, base_pos=base, iters=200)
                worst = max(worst, pe)
    check(worst < 0.001, f"the dip poses for drops within 5 mm of the aim are all in reach ({worst * 1000:.2f} mm IK error)")

    # -- drop landing zones -------------------------------------------------------------------------------------------
    w = world()
    check(M.land(w, (at[0], at[1], M.SLIDE_TOP + 0.012))[0] == "slide", "a drop over the slide lands on it")
    check(M.land(w, (0.10, -0.05, M.TOP + 0.02))[0] == "counter", "elsewhere it lands on the counter")
    ws = world(meter=(M.M_INS, np.eye(3)), strip=(s_c, Rs_i))
    over_strip = M.strip_point(ws, -0.010) + np.array([0, 0, 0.010])
    check(M.land(ws, over_strip)[0] == "strip", "a drop over the inserted strip lands on the strip")
    dt = M.drop_tcp()
    tip_w = dt + (M.ry(-M.DROP_TILT) @ M.B_GRIP_R)[:, 2] * M.B_IN_HAND + M.ry(-M.DROP_TILT) @ M.B_ROT0 @ np.array([M.B_TIP - M.GRIP_X, 0.0, 0.0])
    check(np.allclose(tip_w[:2], M.DROP_AT) and abs(tip_w[2] - M.SLIDE_TOP - M.TIP_OVER) < 1e-9,
          "the squeeze pose holds the tip 12 mm over the aim point")
    check(abs(M.DROP_AT[0] - M.SLIDE_C[0]) + M.BEAD_A < M.SLIDE_SIZE[0] / 2, "the drop lands well inside the slide")

    # -- the full sequence of tiers ------------------------------------------------------------------------------------
    fresh()
    ins = (M.M_INS, np.eye(3))
    frames = [world(meter=ins, strip=(s_c, Rs_i), grippers={"left_gripper": M.meter_tcp(M.M_INS),
                                                            "right_gripper": AWAY["right_gripper"]})]
    check(run(frames) == 0.25, "strip seated (meter on): 0.25")
    put_drop(at[:2], step=M._L.n)
    check(run([world(meter=ins, strip=(s_c, Rs_i))]) == 0.5, "a drop on the slide too: 0.5")
    dip = [world(meter=(mo, R_d), strip=so) for _ in range(M.SIP_STEPS + 2)]
    check(run(dip) == 0.5 and M._L.filled_at is not None, "tip held in the drop for the sip time: sample drawn")
    count = [world(meter=(mo + np.array([0, 0, 0.03]), R_d), strip=strip_in(mo + np.array([0, 0, 0.03]), R_d))
             for _ in range(M.COUNT_STEPS)]
    check(run(count) == 0.75 and M._L.result, "seated through the 5 s countdown: a result, 0.75")
    down = np.array([M.METER_START[0], M.METER_START[1], M.TOP])
    b_c, R_b = bottle_start()
    hi = mo + np.array([0, 0, 0.03])
    run([world(meter=(hi + (down - hi) * k / 40, R_d), strip=strip_in(hi + (down - hi) * k / 40, R_d))
         for k in range(1, 41)])  # carried down slowly (not a drop)
    fin = [world(meter=(down, np.eye(3)), strip=strip_in(down), cap=capped_cap(b_c, R_b)) for _ in range(M.HOLD_STEPS + 2)]
    check(run(fin[:3]) == 0.75, "everything put away but not yet held 1 s: 0.75")
    check(run(fin[3:]) == 1.0, "held still 1 s with hands clear: 1.0")
    check(run([world(meter=(down, np.eye(3)), strip=strip_in(down), cap=capped_cap(b_c, R_b),
                     grippers={"left_gripper": M.meter_tcp(down), "right_gripper": AWAY["right_gripper"]})]) == 0.75,
          "a hand back on the meter: 0.75 again")
    on_stack = (b_c + np.array([0.0, M.GROOVE_W / 2 + M.SLIDE_W / 2, M.GROOVE_D - 0.0003]), R_b)
    check(run([world(meter=(down, np.eye(3)), strip=strip_in(down), bottle=on_stack, cap=capped_cap(*on_stack))
               for _ in range(M.HOLD_STEPS + 2)]) == 0.75, "the capped bottle laid on top of a slide stack: 0.75")
    fresh()
    check(run([world(meter=(down, np.eye(3)), strip=strip_in(down), cap=capped_cap(b_c, R_b))
               for _ in range(30)]) == 0.25, "a seated strip with no drop and no sample, put away: 0.25 only")

    # -- failures ------------------------------------------------------------------------------------------------------
    fresh()
    run([world(meter=ins, strip=(s_c, Rs_i))])
    put_drop(at[:2], step=M._L.n)
    run([world(meter=(mo, R_d), strip=so) for _ in range(M.SIP_STEPS + 1)])
    out = world(meter=(mo + np.array([0, 0, 0.03]), R_d), strip=(np.array([0.1, 0.0, M.TOP + 0.0002]), np.eye(3)))
    check(run([out] * 5) == 0.0 and M._L.aborted, "strip pulled out during the countdown: no result, 0")
    fresh()
    flat = dict(meter=(flat_o, np.eye(3)), strip=strip_in(flat_o))
    run([world(**flat)])
    put_drop(at[:2], step=M._L.n)
    check(run([world(**flat) for _ in range(M.SIP_STEPS + M.COUNT_STEPS + 2)]) == 0.25 and M._L.smeared,
          "a smeared (flat) application never gives a result: 0.25 for the seated strip")
    fresh()
    bt = M.ry(-M.DROP_TILT) @ M.B_ROT0
    tip_at = M.strip_point(ws, -M.S_L / 2 + 0.002)
    b_center = tip_at - bt @ np.array([M.B_TIP - M.TIP_R, 0.0, 0.0])
    check(run([world(meter=ins, strip=(s_c, Rs_i), bottle=(b_center, bt))]) == 0.0
          and "touched" in M._L.ruined, "the strip touching the bottle's tip: 0")
    fresh()
    run([world(meter=ins, strip=(s_c, Rs_i))])
    check(run([world(meter=(M.M_INS - np.array([0, 0, 0.06]), np.eye(3)), strip=(s_c - np.array([0, 0, 0.06]), Rs_i))])
          == 0.0 and "meter dropped" in M._L.ruined, "the meter dropped (6 cm in a step): 0")
    fresh()
    run([world(meter=ins, strip=(s_c, Rs_i))])
    dk._S.drops.append(dk.Drop(M._L.n, np.zeros(3), "strip", np.zeros(3), {}))
    check(run([world(meter=ins, strip=(s_c, Rs_i))]) == 0.0, "control dripped onto the strip from the bottle: 0")

    # -- the cap -------------------------------------------------------------------------------------------------------
    rim, Rc = capped_cap(b_c, R_b)
    check(M.capped(world(cap=(rim, Rc))), "cap rim on the shoulder, axis along the bottle: capped")
    check(not M.capped(world(cap=(rim + R_b @ np.array([0.004, 0, 0]), Rc))), "cap 4 mm off the shoulder: not capped")
    check(M.CAP_IN > M.NOZ_R and M.CAP_H - M.CAP_TOP_T > M.B_TIP - M.B_SHOULDER,
          "the nozzle fits inside the cap's bore (radius and depth)")
    rel = M.CAP_R_HAND @ M.DOWN_Y.T  # the cap turns with the hand
    c_off = rel @ np.array([0.0, 0.0, M.CAP_H / 2 + 0.013])
    held = (M.CAP_HOLD - M.B_ROT0 @ np.array([M.GRIP_X, 0.0, 0.0]) + M.B_GRIP_R[:, 2] * M.B_IN_HAND, M.B_ROT0)
    check(M.capped(world(bottle=held, cap=(M.cap_tcp() - c_off - M.CAP_TRIM, rel))), "the demo's push pose seats the cap on the shoulder")
    rim_pre = M.cap_tcp(back=0.015) - c_off - M.CAP_TRIM
    noz_tip = held[0] + M.B_ROT0 @ np.array([M.B_TIP, 0.0, 0.0])
    check(rim_pre[0] < noz_tip[0] - 0.003, "lined up, the cap's rim stands 3 mm or more off the nozzle's tip")
    check(np.allclose(M.bottle_tcp(*held), M.CAP_HOLD), "the held bottle's grip point is the right hand's tool point")

    # -- the rest: lying in the groove, not on a stack, across the stacks, on end, held or out in front ---------------
    lie = lambda c, R=R_b: M.bottle_in_rest(world(bottle=(np.asarray(c, float), R)))  # noqa: E731
    floor_z = M.REST_FLOOR + M.B_R
    check(abs(b_c[2] - floor_z) < 1e-12 and lie(b_c), "the start pose: in the groove on its floor")
    check(lie([0.2391, -0.0711, 0.7572]), "the demo's logged pose after laying it back (1.1 mm to one side, 0.2 mm up)")
    check(lie(b_c + [0, 0, 0.0019]) and not lie(b_c + [0, 0, 0.0021]), "the axis within 2 mm of its resting height")
    top = b_c + np.array([0.0, M.GROOVE_W / 2 + M.SLIDE_W / 2, M.GROOVE_D])
    check(not lie(top) and not lie(top - [0, 0, 0.0005]), "lying on top of a slide stack (4 mm up): not in the rest")
    beside = M.GROOVE_W / 2 + 2 * M.SLIDE_W + M.B_R
    check(not lie(b_c + [0, beside, 0]) and not lie(b_c + [0, -beside, 0]), "on the counter beside the rest: not in it")
    rz90 = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])
    across = np.array([M.REST_C[0], M.REST_C[1], floor_z + M.GROOVE_D])
    check(not lie(across, rz90 @ R_b), "lying across both stacks: not in the rest")
    check(not lie(b_c + [-0.029, 0, 0]), "on the counter 29 mm out in front of its start (body past the slides): not in the rest")
    check(lie(b_c + [-0.004, 0, 0]) and lie(b_c + [0.030, 0, 0]), "anywhere along the groove with its body between the stacks")
    tilt = math.asin(M.GROOVE_D / (M.B_BODY[1] - M.B_BODY[0]))
    check(not lie(b_c + [0, 0, M.GROOVE_D / 2], M.ry(tilt) @ R_b), "propped with one end 4 mm up: not in the rest")
    upright = np.array([b_c[0], b_c[1], M.REST_FLOOR - M.B_BODY[0]])
    check(not lie(upright, M.ry(math.pi / 2) @ R_b) and not lie(upright, M.ry(-math.pi / 2) @ R_b),
          "standing on end in the groove: not in the rest")
    check(not lie(b_c + [0, 0, 0.010]), "held 1 cm over the groove: not in the rest")

    # -- cheats: a capped squeeze, touching the nozzle, a drop put beside the strip, upside down, a hand on the meter --
    bt = M.ry(-M.DROP_TILT) @ M.B_ROT0
    tip_target = np.array([at[0], at[1], M.SLIDE_TOP + 0.012])
    bcen = tip_target - bt @ np.array([M.B_TIP - M.TIP_R, 0, 0])
    fresh()
    run([world(meter=ins, strip=(s_c, Rs_i))])
    w_sq = world(meter=ins, strip=(s_c, Rs_i), bottle=(bcen, bt), cap=capped_cap(bcen, bt), squeeze=0.004)
    check(M.cap_covers(w_sq) and dk.tip_down(w_sq, M.BOTTLE_SPEC), "the capped bottle held tip down")
    run([world(meter=ins, strip=(s_c, Rs_i), bottle=(bcen, bt), cap=capped_cap(bcen, bt), squeeze=0.004)
         for _ in range(10)])
    check(not dk.drops() and dk.hanging() == 0.0, "squeezed hard with the cap on: no drop (the cap closes the tip)")
    rim, Rc = capped_cap(bcen, bt)
    run([world(meter=ins, strip=(s_c, Rs_i), bottle=(bcen, bt), cap=(rim + bt @ np.array([0.004, 0, 0]), Rc),
               squeeze=0.004) for _ in range(10)])
    check(not dk.drops(), "the cap pulled 4 mm off but still over the nozzle: no drop")
    run([world(meter=ins, strip=(s_c, Rs_i), bottle=(bcen, bt), cap=cap_set_down(), squeeze=0.004) for _ in range(10)])
    check(len(dk.drops()) >= 1 and dk.drops()[0].zone == "slide", "the same squeeze uncapped: a drop on the slide")
    fresh()
    run([world(meter=ins, strip=(s_c, Rs_i))])
    for _ in range(8):
        run([world(meter=ins, strip=(s_c, Rs_i), bottle=(bcen, bt), cap=capped_cap(bcen, bt), squeeze=0.004)])
    run([world(meter=(mo, R_d), strip=so) for _ in range(M.SIP_STEPS + 2)])
    run([world(meter=(hi, R_d), strip=strip_in(hi, R_d)) for _ in range(M.COUNT_STEPS)])
    run([world(meter=(hi + (down - hi) * k / 40, R_d), strip=strip_in(hi + (down - hi) * k / 40, R_d)) for k in range(1, 41)])
    check(run([world(meter=(down, np.eye(3)), strip=strip_in(down)) for _ in range(M.HOLD_STEPS + 3)]) == 0.25,
          "never uncapped (squeezing the capped bottle over the slide): 0.25, no sample")

    tipw = bcen + bt @ np.array([M.B_TIP, 0, 0])  # the nozzle's end
    fresh()
    run([world(meter=ins, strip=(s_c, Rs_i))])
    on_end = tipw + np.array([-0.005, 0.0, -M.S_T / 2])
    check(run([world(meter=ins, strip=(on_end, np.eye(3)), bottle=(bcen, bt), cap=cap_set_down())]) == 0.0
          and "touched" in M._L.ruined, "a strip pressed on the nozzle's end face: touched the tip, 0")
    fresh()
    run([world(meter=ins, strip=(s_c, Rs_i))])
    r2 = M.TIP_R + 0.0003 + (M.NOZ_R - M.TIP_R - 0.0003) * 0.002 / (M.B_TIP - M.B_SHOULDER)  # the cone 2 mm back
    side = tipw + bt @ np.array([-0.002, 0.0, 0.0]) + np.array([0.0, r2 + M.S_T / 2 + 0.0002, 0.0])
    rx90 = np.array([[1.0, 0.0, 0.0], [0.0, 0.0, -1.0], [0.0, 1.0, 0.0]])  # the strip's faces toward +-y
    check(M.tip_touched(world(strip=(side, rx90), bottle=(bcen, bt))), "a strip 0.2 mm from the nozzle's side near its end: touched")
    check(not M.tip_touched(world(strip=(on_end - np.array([0, 0, 0.003]), np.eye(3)), bottle=(bcen, bt))),
          "the strip 3 mm under the tilted nozzle's end (1.7 mm below its lowest edge): not touched")
    w_h = world(bottle=(bcen, bt))
    dk._fresh_if_new()
    dk._S.hanging = 20.0  # uL hanging from the tip, not yet a drop
    hang = dk.tip(w_h, M.BOTTLE_SPEC) - np.array([0.0, 0.0, (3 * 20e-9 / (4 * math.pi)) ** (1 / 3)])
    check(M.tip_touched(world(strip=(hang - np.array([0.005, 0, 0]), np.eye(3)), bottle=(bcen, bt))),
          "the strip's tip in a drop still hanging from the bottle: touched (control from the bottle)")
    dk._S.hanging = 0.0

    fresh()
    run([world(meter=ins, strip=(s_c, Rs_i))])
    wd = world(meter=(mo, R_d), strip=so)
    tipS = M.sample_tip(wd)
    check(M.land(wd, tipS + np.array([-0.001, 0, 0.006]))[0] == "strip", "a drop falling 1 mm beside the strip's tip hits it")
    check(M.land(wd, tipS + np.array([-0.0025, 0, 0.006]))[0] == "strip",
          "a drop landing 2.5 mm beside the strip's tip spreads onto it: put on the strip")
    check(M.land(wd, tipS + np.array([-0.010, 0, 0.012]))[0] == "slide", "1 cm from the strip: on the slide")

    flip = np.diag([1.0, -1.0, -1.0])
    check(not M.seated(world(meter=ins, strip=(s_c, Rs_i @ flip))), "the strip in upside down: not seated (E 3)")

    side_fist = {"left_gripper": (down[0], down[1], down[2] + M.M_T + 0.034), "right_gripper": AWAY["right_gripper"]}
    check(not M.released(world(meter=(down, np.eye(3)), strip=strip_in(down), grippers=side_fist)),
          "the tool point 34 mm over the meter with the hand's body on it: not released")
    check(M.released(world(meter=(down, np.eye(3)), strip=strip_in(down))), "hands away: released")

    fresh()
    run([world(meter=ins, strip=(s_c, Rs_i))])
    put_drop(at[:2], step=M._L.n)
    run([world(meter=(mo, R_d), strip=so) for _ in range(M.SIP_STEPS + 2)])
    run([world(meter=(hi, R_d), strip=strip_in(hi, R_d)) for _ in range(M.COUNT_STEPS)])
    run([world(meter=(hi + (flat_o - hi) * k / 80, np.eye(3)), strip=strip_in(hi + (flat_o - hi) * k / 80))
         for k in range(1, 81)])
    check(run([world(meter=(flat_o, np.eye(3)), strip=strip_in(flat_o))]) == 0.75,
          "the strip laid in the drop after the result: the meter keeps its reading, 0.75")
    check(M.STRIP.friction == M.STRIP_FRICTION and M.CAP.friction == M.CAP_FRICTION
          and M.SLIDE.friction == M.GLASS_FRICTION and M.REST.friction == M.TAPE_FRICTION,
          "strip, cap, slide and rest carry their own materials' friction")

    # -- the cap: a close sliding fit on the nozzle's band; it stays on whenever the demo moves the capped bottle ----
    import gripkit
    check(abs(M.CAP_IN - M.NOZ_R - M.CAP_CLEAR) < 1e-12 and M.CAP_CLEAR <= 0.0003, "the cap's bore clears the band by 0.2 mm")
    over = M.B_SHOULDER + M.SEAL_L - (M.B_SHOULDER + M.CAP_ON_GAP)  # the band's length inside the seated cap
    tilt = math.degrees(math.atan(2 * M.CAP_CLEAR / over))
    check(tilt < M.CAP_ALIGN_DEG / 2, f"seated, the cap can tip at most {tilt:.1f} degrees on the band (capped allows 10)")
    mu_cap = (M.CAP_FRICTION[0] + M.BOTTLE_FRICTION[0]) / 2  # PhysX averages the two materials
    check(M.SEAL_L > 0 and all(a >= b for a, b in zip([r for r, _x in M.NOZZLE_PROFILE], [r for r, _x in M.NOZZLE_PROFILE][1:])),
          "the band is cylindrical (no taper to push the cap off) and the nozzle only narrows past it")
    # the right hand's moves while the cap is on: the bottle level (B_GRIP_R), so only the moves' acceleration along
    # the bottle's axis could slide the cap; a velocity change counts as made within one control step (0.05 s)
    plan = M.right_uncap(5.0)
    cut = next(i for i, w in enumerate(plan) if "cap to come off" in w.label)
    capped_moves = [w for w in plan[:cut + 1] if np.allclose(M.tilted_R(w) if hasattr(M, "tilted_R") else M.B_GRIP_R, M.B_GRIP_R)]
    pos = [np.asarray(w.pos, float) for w in capped_moves]
    vel = [((b - a) / w.duration)[0] for a, b, w in zip(pos, pos[1:], capped_moves[1:])]
    acc = max(abs(b - a) / 0.05 for a, b in zip([0.0] + vel, vel + [0.0]))
    check(acc < 0.5 * mu_cap * 9.81, f"carried capped, the bottle's axial acceleration ({acc:.2f} m/s^2) stays under half "
          f"what friction holds ({mu_cap * 9.81:.2f} m/s^2): the cap stays on")
    weight_hold = mu_cap * M.CAP_KG * 9.81
    mu_pad = lambda m: (gripkit.RUBBER_PAD[0] + m) / 2  # noqa: E731
    pinch = 2 * gripkit.squeeze_force_n(3.0) * mu_pad(M.CAP_FRICTION[0])
    check(pinch > 100 * weight_hold, "the uncap pinch pulls the cap off with a wide margin")

    # -- the nozzle and the cap's bore are smooth hulls (no ledges); the demo's uncap pull clears the nozzle ------
    noz = [c for c in M.BOTTLE.colliders if c.hull is not None and len(c.hull) == len(M.NOZZLE_PROFILE) * M.HULL_N]
    check(len(noz) == 1 and len(noz[0].hull) == len(M.NOZZLE_PROFILE) * M.HULL_N,
          "the nozzle (band and cone) is one smooth convex hull, not stacked cylinders")
    radii = [r for r, _x in M.NOZZLE_PROFILE]
    check(all(a >= b for a, b in zip(radii, radii[1:])), "from the band to the tip the nozzle only narrows: no ledge to catch")
    pts = np.array(noz[0].hull)
    flats = min(np.hypot(pts[:M.HULL_N, 1], pts[:M.HULL_N, 2])) * math.cos(math.pi / M.HULL_N)
    check(abs(flats - M.NOZ_R) < 1e-6, "the hull's flats sit at the band's radius (the pads' seat as designed)")
    check(all(c.hull is not None for c in M.CAP.colliders), "the cap's wall and top are hulls too (a round bore)")
    wall = np.array(M.CAP.colliders[0].hull)
    check(abs(min(np.hypot(wall[:, 0], wall[:, 1])) * math.cos(math.pi / M._CN) - M.CAP_IN) < 1e-9,
          "the bore's flats sit at its radius")
    give = 0.010  # m: the right arm's give under the fit's hold, seen in probe pf-143018
    inside = M.B_TIP - M.B_SHOULDER - M.CAP_ON_GAP  # how far the nozzle reaches into the seated cap
    check(M.CAP_PULL - give >= inside + 0.005, "the uncap pull, less the arm's give, clears the nozzle by 5 mm")
    held = (M.CAP_HOLD - M.B_ROT0 @ np.array([M.GRIP_X, 0.0, 0.0]) + M.B_GRIP_R[:, 2] * M.B_IN_HAND, M.B_ROT0)
    rim0, Rc0 = capped_cap(*held)
    pulled = rim0 + (M.uncap_grip(back=M.CAP_PULL) - M.uncap_grip()) * (M.CAP_PULL - give) / M.CAP_PULL
    w_off = world(bottle=held, cap=(pulled, Rc0))
    check(not M.cap_covers(w_off) and not M.capped(w_off), "replayed: at the end of the pull the cap is off the nozzle")
    w_part = world(bottle=held, cap=(rim0 + (M.uncap_grip(back=0.008) - M.uncap_grip()), Rc0))
    check(M.cap_covers(w_part), "replayed: 8 mm into the pull the cap still covers the tip (no drop yet)")

    # -- the squeeze: the walls' force from the pinching hand's commanded closing, whatever the bottle's roll -------
    check(not getattr(M.BOTTLE, "links", ()) and any(c.hull is not None and len(c.hull) == 2 * M.HULL_N
                                                     for c in M.BOTTLE.colliders),
          "the bottle is one rigid round body (no wall pads)")
    k_s = M.K_F * M.WALL_K / (M.K_F + M.WALL_K)
    held_b = (np.array(M.BOTTLE_START), M.B_ROT0)
    for half_mm, want in ((M.B_R * 1000 + 1.0, 0.0), (M.B_R * 1000, 0.0), (M.B_R * 1000 - 0.8, 0.0008 * k_s),
                          (M.B_R * 1000 - 3.0, 0.003 * k_s), (0.0, M.WALL_K * M.PAD_TRAVEL)):
        w_f = world(bottle=held_b)
        w_f["right_gripper"] = pinch_bottle(*held_b)
        kit._OPEN_TARGET["right_gripper"] = half_mm / 1000 / M.F_HALF_OPEN
        check(abs(M.wall_force(w_f) - want) < 1e-6, f"fingers commanded to {half_mm:.1f} mm from the axis: "
              f"{M.wall_force(w_f):.2f} N on the walls")
    check(abs(M.wall_force(world(bottle=held_b, squeeze=0.0008)) - 0.0008 * M.WALL_K) < 1e-6
          and M.wall_force(world(bottle=held_b)) == 0.0, "no hand on the bottle: no squeeze")
    w_off = world(bottle=held_b)
    w_off["right_gripper"] = st(M.bottle_tcp(*held_b) + np.array([0.0, 0.0, 0.03]), M.B_GRIP_R)
    kit._OPEN_TARGET["right_gripper"] = 0.0
    check(M.wall_force(w_off) == 0.0, "fingers closed in the air over the bottle: no squeeze (both tips must be on it)")
    forces = []
    for roll_deg in (0.0, 10.0, 20.0, 45.0):  # the bottle rolled about its own axis inside the same pinch
        r = math.radians(roll_deg)
        Rx = np.array([[1, 0, 0], [0, math.cos(r), -math.sin(r)], [0, math.sin(r), math.cos(r)]])
        w_r = world(bottle=held_b, squeeze=M.PAD_TRAVEL)
        w_r["bottle"] = st(held_b[0], M.B_ROT0 @ Rx)
        forces.append(M.wall_force(w_r))
    check(all(abs(f - M.WALL_K * M.PAD_TRAVEL) < 1e-6 for f in forces),
          f"rolled 0/10/20/45 deg in the hand: the same squeeze ({', '.join(f'{f:.1f}' for f in forces)} N)")
    check(M.finger(M.FINGER_SQUEEZE) == M.FINGER_CLOSED, "the drop squeeze is commanded past the walls' stop")
    full = M.WALL_K * M.PAD_TRAVEL
    t_drop = M.DROP_UL / (M.BOTTLE_SPEC.flow * (full - M.OPEN_FORCE))
    check(full >= M.OPEN_FORCE + 2.0 and t_drop <= M.SQUEEZE_S, f"at the stop the walls take {full:.0f} N and a drop "
          f"forms in {t_drop:.2f} s, inside the {M.SQUEEZE_S} s squeeze")
    n_sq = round((0.1 + M.SQUEEZE_S) / 0.05)
    for f_n, lo, hi in ((10.4, 1, 1), (M.WALL_K * M.PAD_TRAVEL, 1, 1)):
        fresh()
        run([world(meter=ins, strip=(s_c, Rs_i))])
        for _ in range(n_sq):
            run([world(meter=ins, strip=(s_c, Rs_i), bottle=(bcen, bt), cap=cap_set_down(), squeeze=f_n / M.WALL_K)])
        check(lo <= len(dk.drops()) <= hi, f"a {M.SQUEEZE_S} s squeeze at {f_n:.1f} N: {len(dk.drops())} drop(s)")
    fresh()
    run([world(meter=ins, strip=(s_c, Rs_i))])
    for _ in range(n_sq):
        run([world(meter=ins, strip=(s_c, Rs_i), bottle=(bcen, bt), cap=capped_cap(bcen, bt), squeeze=M.PAD_TRAVEL)])
    check(not dk.drops(), "the same full squeeze with the cap on: no drop")

    # -- QC cheats (glucose_qc_cheats.py): no squeeze without a real pinch on the body ------------------------------
    import handkit as _hk2

    def _tips(w_):
        return _hk2.gap(w_, "right_gripper", "bottle", M.BODY_SHAPE, links=("tip_left", "tip_right"))

    def _place(w_, R_hand, start, direction, target=0.0005):
        lo_, hi_ = 0.0, 0.08
        for _ in range(40):
            mid_ = (lo_ + hi_) / 2
            w_["right_gripper"] = st(np.asarray(start) + mid_ * np.asarray(direction), R_hand)
            lo_, hi_ = (lo_, mid_) if min(_tips(w_).values()) < target else (mid_, hi_)
        w_["right_gripper"] = st(np.asarray(start) + hi_ * np.asarray(direction), R_hand)

    b0 = bottle_start()
    fresh()
    w_q = world()
    kit._OPENING["right_gripper"] = 0.0
    kit._OPEN_TARGET["right_gripper"] = 0.0
    top = b0[0] + np.array([0, 0, 0.10]) + b0[1] @ np.array([M.GRIP_X, 0, 0])
    _place(w_q, M.DOWN_Y, top, (0, 0, -1))
    check(M.wall_force(w_q) == 0.0, "QC Q1: closed fingertips resting on top of the body: no squeeze")
    _place(w_q, M.DOWN_Y, top, (0, 0, -1), target=0.0009)
    check(M.wall_force(w_q) == 0.0, "QC Q1b: closed fingertips hovering 0.9 mm over it: no squeeze")
    R_side = M.frame((0, 1, 0), (1, 0, 0))
    side = b0[0] + b0[1] @ np.array([M.GRIP_X, 0, 0]) - np.array([0, 0.10, 0])
    fresh()
    w_q = world()
    kit._OPENING["right_gripper"] = 0.0
    kit._OPEN_TARGET["right_gripper"] = 0.0
    _place(w_q, R_side, side, (0, 1, 0))
    check(M.wall_force(w_q) == 0.0, "QC Q2: closed fingertips against the body's side: no squeeze")
    fresh()
    w_q = world()
    R_len = M.frame((0, 0, -1), (1, 0, 0))
    mid_b = b0[0] + b0[1] @ np.array([(M.B_BODY[0] + M.B_BODY[1]) / 2, 0, 0])
    w_q["right_gripper"] = st(mid_b + np.array([0, 0, 0.004]), R_len)
    lo_o, hi_o = 0.0, 1.0
    for _ in range(30):
        m_o = (lo_o + hi_o) / 2
        kit._OPENING["right_gripper"] = m_o
        lo_o, hi_o = (m_o, hi_o) if min(_tips(w_q).values()) < 0 else (lo_o, m_o)
    kit._OPENING["right_gripper"] = hi_o
    kit._OPEN_TARGET["right_gripper"] = 0.0
    check(M.wall_force(w_q) == 0.0, "QC Q3: a lengthwise pinch, base to body end: no squeeze")
    fresh()
    w_q = world()
    for dx in np.arange(0.0, 0.03, 0.001):
        w_q["right_gripper"] = st(b0[0] + b0[1] @ np.array([M.B_SHOULDER + dx, 0, 0]), M.DOWN_Y)
        kit._OPENING["right_gripper"] = M.CAP_R / M.F_HALF_OPEN
        kit._OPEN_TARGET["right_gripper"] = 0.0
        if min(_tips(w_q).values()) >= 0:
            break
    check(M.wall_force(w_q) == 0.0, "QC Q4b: a pinch on the cap past the shoulder: no squeeze")
    fresh()
    Rb5 = M.ry(-math.radians(45)) @ M.B_ROT0
    c5 = np.array([M.SLIDE_C[0], M.SLIDE_C[1], M.SLIDE_TOP]) - Rb5 @ np.array([M.B_TIP, 0, 0]) + np.array([0, 0, 0.012])
    w_q = world(bottle=(c5, Rb5), cap=cap_set_down())
    kit._OPENING["right_gripper"] = 0.0
    kit._OPEN_TARGET["right_gripper"] = 0.0
    _place(w_q, R_side, c5 + Rb5 @ np.array([M.GRIP_X, 0, 0]) - np.array([0, 0.10, 0]), (0, 1, 0))
    for _ in range(8):
        run([dict(w_q)])
    check(M.wall_force(w_q) == 0.0 and not dk.drops(),
          "QC Q5: uncapped and tip down, closed fingertips touching its side: no squeeze, no drop")
    # re-QC N6/N6b: fingers held apart by something else (the cap 6.5 mm, the meter 7.45 mm, 9 mm), their ends over
    # the body: not a pinch (the body must be between the fingers)
    for face in (M.CAP_R, 0.00745, 0.009):
        fresh()
        w_q = world(cap=cap_set_down())
        kit._OPENING["right_gripper"] = face / M.F_HALF_OPEN
        kit._OPEN_TARGET["right_gripper"] = 0.0
        top = b0[0] + b0[1] @ np.array([M.GRIP_X, 0, 0]) + np.array([0, 0, 0.10])
        _place(w_q, M.DOWN_Y, top, (0, 0, -1))
        check(not M.pinching_body(w_q, "right_gripper") and M.wall_force(w_q) == 0.0,
              f"re-QC N6: faces held {face * 1000:.2f} mm apart by something else, ends over the body: no squeeze")
    fresh()
    Rb6 = M.ry(-math.radians(45)) @ M.B_ROT0
    c6 = np.array([M.SLIDE_C[0], M.SLIDE_C[1], M.SLIDE_TOP]) - Rb6 @ np.array([M.B_TIP, 0, 0]) + np.array([0, 0, 0.012])
    w_q = world(bottle=(c6, Rb6), cap=cap_set_down())
    R6 = Rb6 @ M.B_ROT0.T @ M.B_GRIP_R
    kit._OPENING["right_gripper"] = M.CAP_R / M.F_HALF_OPEN
    kit._OPEN_TARGET["right_gripper"] = 0.0
    _place(w_q, R6, c6 + Rb6 @ np.array([M.GRIP_X, 0, 0]) - R6[:, 2] * 0.10, R6[:, 2])
    w_q["cap"] = st(np.asarray(w_q["right_gripper"].pos), R6)
    for _ in range(12):
        run([dict(w_q)])
    check(M.wall_force(w_q) == 0.0 and not dk.drops(),
          "re-QC N6b: tipped and uncapped, the cap in the hand, finger ends over the body: no squeeze, no drop")
    for tilt in (0.0, 45.0):
        Rt = M.ry(-math.radians(tilt)) @ M.B_ROT0
        ct = np.array(M.BOTTLE_START) if tilt == 0.0 else c6
        fresh()
        w_q = world(bottle=(ct, Rt), squeeze=M.PAD_TRAVEL)
        check(abs(M.wall_force(w_q) - M.WALL_K * M.PAD_TRAVEL) < 1e-6,
              f"the demo's pinch (axis {M.B_IN_HAND * 1000:.0f} mm past the tool point) squeezes 12 N, tipped {tilt:.0f} deg")

    fresh()
    w_q = world(squeeze=M.PAD_TRAVEL)
    half = kit._OPENING["right_gripper"] * M.F_HALF_OPEN
    check(abs(M.wall_force(w_q) - M.WALL_K * M.PAD_TRAVEL) < 1e-6 and half > M.B_R - M.PAD_TRAVEL - 0.001,
          f"the demo's pinch (faces {half * 1000:.2f} mm from the axis) still squeezes 12 N")
    kit._OPENING["right_gripper"] = 0.0077 / M.F_HALF_OPEN
    check(M.pinching_body(w_q, "right_gripper"), "faces sunk to 7.7 mm under the squeeze (pb-204321): still a pinch")
    kit._OPENING["right_gripper"] = (M.B_R - M.PAD_TRAVEL - 0.0015) / M.F_HALF_OPEN
    check(not M.pinching_body(w_q, "right_gripper"), "faces 5 mm from the axis, past the walls' give: not a pinch")

    # -- a squeeze with the cap off: a drop at the real opening force, none just under it -----------------------------
    for f_n, want in ((M.OPEN_FORCE + 0.3, True), (M.OPEN_FORCE - 0.3, False)):
        fresh()
        run([world(meter=ins, strip=(s_c, Rs_i))])
        run([world(meter=ins, strip=(s_c, Rs_i), bottle=(bcen, bt), cap=cap_set_down(), squeeze=f_n / M.WALL_K)
             for _ in range(40)])
        check(bool(dk.drops()) == want, f"cap off, tip down, {f_n:.1f} N on the walls: {'a drop' if want else 'no drop'}")

    # -- the text --------------------------------------------------------------------------------------------------------
    tj = json.loads((HERE / "task.json").read_text())
    text = tj["scoring"] + tj["setup"]
    for v in ("1 mm", "6 degrees", "1 s", "5 s", "60 s", "3 mm", "1.5 mm", "10 degrees", "0.8 m/s", "within 2 mm",
              "9.5 mm radius", "4 mm higher", "E 3", "1.93 mm", "either way round", "last 3 mm"):
        check(v in text, f"task.json states {v}")
    check(tj["instruction"] == M.INSTRUCTION, "task.json instruction matches")
    print("all ok")


if __name__ == "__main__":
    main()
