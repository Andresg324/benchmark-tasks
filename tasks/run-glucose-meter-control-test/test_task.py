"""Offline checks of the glucose-meter control test's scorer and demo geometry, no simulator. Poses are set by hand, as
the simulator reports them each control step. Covers: doing nothing (0); the start layout (strip standing in its bottle,
not seated; meter down; bottle in its rest, capped); the demo's strip pinch matching the strip's spawn pose; the
insert pose seating the strip (and 2 mm short not); the dip pose putting only the strip's tip in the drop (and a flat
dip smearing it); drop landing zones (slide, counter, strip); the full sequence of tiers 0.25 -> 0.5 -> 0.75 -> 1.0
with the 1 s hold; a strip pulled out during the countdown; a smear; the strip touching the bottle's tip; a dropped
meter; the cap seated on the nozzle vs beside it; the thresholds in the text.

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
    return np.array(M.CAP.pos), M.gk._rot(np.asarray(M.CAP.tilt_xyzw, float))


def cap_set_down():
    """The cap once the robot has taken it off: open side down on its mark."""
    return np.array([M.CAP_START[0], M.CAP_START[1], M.TOP]), np.eye(3)


def strip_in(meter_o, R_m=np.eye(3), short=0.0):
    """The strip seated in a meter at (meter_o, R_m), its grey end `short` m from the back stop."""
    Rs = R_m  # strip frame aligned with the meter's (x along the port, faces up)
    grey = np.asarray(meter_o) + R_m @ np.array([M.X_STOP - short, 0.0, M.Z_SLOT])
    return grey - Rs @ np.array([M.S_L / 2, 0.0, 0.0]), Rs


def world(meter=None, strip=None, bottle=None, cap=None, grippers=AWAY, roller=0.0, squeeze=0.0):
    joints(roller, squeeze)
    m = meter or (np.array([M.METER_START[0], M.METER_START[1], M.TOP]), np.eye(3))
    s = strip or (np.array(M.STRIP_START), M._S_ROT0)
    b = bottle or bottle_start()
    c = cap or cap_start()
    out = {"meter": st(*m), "strip": st(*s), "bottle": st(*b), "cap": st(*c)}
    for g, p in grippers.items():
        out[g] = st(p)
    return out


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
    Rs, off = M.strip_rel(M.DOWN_Y)
    check(np.allclose(Rs, M._S_ROT0) and np.allclose(g + off, M.STRIP_START, atol=1e-6),
          "the demo's pinch holds the strip where it stands")
    tips_z = g[2] - 0.013
    check(M.TOP + M.V_H + 0.0005 < tips_z < M.TOP + M.V_H + M.PROTRUDE, "the pad tips stay over the bottle's rim")
    Rs_i, off_i = M.strip_rel(M.INSERT_R)
    check(np.allclose(Rs_i[:, 0], [1, 0, 0]) and abs(abs(Rs_i[2, 2]) - 1) < 1e-9,
          "turned for the port: grey end forward, faces up and down")
    seat_tcp = M.insert_tcp(M.M_INS)
    s_c = seat_tcp + off_i
    fresh()
    w = world(meter=(M.M_INS, np.eye(3)), strip=(s_c, Rs_i))
    check(M.seated(w) and abs(M.seat_depth(w)) < 1e-6, "the insert pose puts the grey end on the back stop")
    w = world(meter=(M.M_INS, np.eye(3)), strip=(s_c - np.array([0.002, 0, 0]), Rs_i))
    check(not M.seated(w), "2 mm short of the back stop is not seated")
    pre = M.insert_tcp(M.M_INS, back=0.015) + off_i + Rs_i @ np.array([M.S_L / 2, 0, 0])
    check(pre[0] < M.M_INS[0] + M.X_FACE - 0.005, "the line-up pose keeps the strip out of the port")
    tips = seat_tcp + M.INSERT_R[:, 2] * 0.013
    check(tips[0] < M.M_INS[0] + M.X_FACE - 0.005, "seated, the pad tips stay 5 mm clear of the meter's end face")

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
    check(M.CAP_IN > M.NOZ_R + 0.0003 and M.CAP_H - M.CAP_TOP_T > M.B_TIP - M.B_SHOULDER,
          "the nozzle fits inside the cap's bore (radius and depth)")
    rel = M.CAP_R_HAND @ M.DOWN_Y.T  # the cap turns with the hand
    c_off = rel @ np.array([0.0, 0.0, M.CAP_H / 2 + 0.013])
    held = (M.CAP_HOLD - M.B_ROT0 @ np.array([M.GRIP_X, 0.0, 0.0]) + M.B_GRIP_R[:, 2] * M.B_IN_HAND, M.B_ROT0)
    check(M.capped(world(bottle=held, cap=(M.cap_tcp() - c_off - M.CAP_TRIM, rel))), "the demo's push pose seats the cap on the shoulder")
    rim_pre = M.cap_tcp(back=0.015) - c_off - M.CAP_TRIM
    noz_tip = held[0] + M.B_ROT0 @ np.array([M.B_TIP, 0.0, 0.0])
    check(rim_pre[0] < noz_tip[0] - 0.003, "lined up, the cap's rim stands 3 mm or more off the nozzle's tip")
    check(np.allclose(M.bottle_tcp(*held), M.CAP_HOLD), "the held bottle's grip point is the right hand's tool point")

    # -- the text --------------------------------------------------------------------------------------------------------
    tj = json.loads((HERE / "task.json").read_text())
    text = tj["scoring"] + tj["setup"]
    for v in ("1 mm", "6 degrees", "1 s", "5 s", "60 s", "3 mm", "1.5 mm", "10 degrees", "0.8 m/s"):
        check(v in text, f"task.json states {v}")
    check(tj["instruction"] == M.INSTRUCTION, "task.json instruction matches")
    print("all ok")


if __name__ == "__main__":
    main()
