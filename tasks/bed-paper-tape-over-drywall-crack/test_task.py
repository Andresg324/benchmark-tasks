"""Offline checks of the tape scorer, the bed model and the demo, no simulator. Tape and knife states are built by hand
from the task's own geometry (task.DOWN, task.ACROSS, task.R_KNIFE_PASS), with the right gripper closed on the knife's
handle when the knife is held. Covers every tier and the cheats: doing nothing, the start state, the tape dropped
loosely on the crack, the tape pressed by a hand (no knife), a knife pushed with no hand on it, the tape off the line,
a steep (scraping) pass that starves the bed, a starved (dry) bed, compound pushed with no tape, a wrinkle, a hand left
on the tape, the knife still held or left on the wall, the bond's peel, a new trial, the bond joints' USD, and the
thresholds in the text.

    python3 -m pytest -q test_task.py      (or: python3 test_task.py)
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import bedkit as bk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena import station as _station  # noqa: E402

AWAY = {"left_gripper": (0.15, 0.30, M.TOP + 0.30), "right_gripper": (0.15, -0.25, M.TOP + 0.30)}
GRIP_DOWN = np.asarray(_demo.grip_rotation(0.0), float)
GRIP_ON_HANDLE = 0.016 / 0.0475  # pads on the 30 mm handle's sides


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=bk.quat(np.asarray(rot)), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def fresh(start=M.start_bed):
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    M._L = M.Ledger(start)


FIXED = {"panel": st(M.PANEL_POS, M.R_PANEL), "stop": st(M.STOP.pos), "brace": st(M.BRACE.pos),
         "offcut": st(M.BLOCK.pos)}
R_SHEET_START = np.array([[math.cos(M.TAPE_YAW), -math.sin(M.TAPE_YAW), 0], [math.sin(M.TAPE_YAW), math.cos(M.TAPE_YAW), 0],
                          [0, 0, 1.0]])


def tape_on_bench():
    return M.TAPE.forward(np.zeros(M.TAPE.n - 1), M.TAPE.pos, R_SHEET_START)


def tape_laid(offset=0.0, turn_deg=0.0, lift=0.0, bump=None, flipped=False):
    """Strip frames of the tape lying on the face along the crack's line: center line `offset` m across, turned
    `turn_deg` in the face's plane about its top end, `lift` m off the face, `bump`: (strip, m) lifts one strip.
    `flipped`: its bench-top side on the wall (strip z into the wall), as the demo's fold lays it."""
    t = math.radians(turn_deg)
    y = math.cos(t) * M.DOWN + math.sin(t) * M.ACROSS
    z = M.NORMAL
    if flipped:
        R = np.column_stack([np.cross(y, -z), y, -z])
        top = M.CRACK_TOP + M.TOP_OVER * M.CRACK_UP + offset * M.ACROSS
        center = top + y * M.TAPE_LEN / 2 + z * (M.TAPE_COLLIDER_T / 2 + lift)
        return M.TAPE.forward(np.zeros(M.TAPE.n - 1), center, R)
    x = np.cross(y, z)
    R = np.column_stack([x, y, z])
    top = M.CRACK_TOP + M.TOP_OVER * M.CRACK_UP + offset * M.ACROSS
    center = top + y * M.TAPE_LEN / 2 + z * (M.TAPE_COLLIDER_T / 2 + lift)
    fr = M.TAPE.forward(np.zeros(M.TAPE.n - 1), center, R)
    if bump is not None:
        i, h = bump
        fr[i] = (fr[i][0] + z * h, fr[i][1])
    return fr


KNIFE_REST_R = M.R_KNIFE_REST


def knife_pass_rot(deg=M.KNIFE_DEG):
    th = math.radians(deg)
    b = math.cos(th) * M.CRACK_UP - math.sin(th) * M.NORMAL
    return np.column_stack([b, -M.ACROSS, np.cross(b, -M.ACROSS)])


def knife_at_edge(edge_mid, R):
    """Knife origin that puts the lower face of the edge `edge_mid` (world)."""
    low = np.array([M.BLADE_LEN, 0.0, M.BLADE_Z - M.BLADE_T / 2])
    return np.asarray(edge_mid, float) - R @ low


def objs(tape_frames, kpos=None, krot=None, held=False, hand_on=None):
    o = dict(FIXED)
    if kpos is None:
        kpos, krot = np.asarray(M.KNIFE_START), KNIFE_REST_R
    o["knife"] = st(kpos, krot)
    for i, (p, R) in enumerate(tape_frames):
        o[M.TAPE.key(i)] = st(p, R)
    opening = {}
    for g, p in AWAY.items():
        o[g] = st(p, GRIP_DOWN)
        opening[g] = 1.0
    if held:
        o["right_gripper"] = st(np.asarray(kpos) + krot @ M.GRIP_LOCAL,
                                krot @ KNIFE_REST_R.T @ np.asarray(_demo.grip_rotation(M.KNIFE_YAW), float))
        opening["right_gripper"] = GRIP_ON_HANDLE
    if hand_on is not None:  # closed left fingertips pressed on a point of the tape
        p = np.asarray(hand_on, float)
        o["left_gripper"] = st(p + M.NORMAL * (0.0005 + 0.013), _r_down_into_face())
        opening["left_gripper"] = 0.0
    return o, opening


def _r_down_into_face():
    a = -M.NORMAL
    c = M.ACROSS
    return np.column_stack([c, np.cross(a, c), a])


def feed(states):
    s = 0.0
    for o, opening in states:
        kit._OPENING.update(opening)
        n = kit._EPISODE.get("last")
        kit._EPISODE["last"] = 0 if n is None else n + 1
        o = dict(o)
        M.success(o)
        s = M.score(o)
    return s


def stroke(tape, deg=M.KNIFE_DEG, held=True, start=None, end=None, speed=M.SPEED, height=None):
    """States for a knife drawn from `start` to `end` (default: the demo's stroke) with its edge on the tape's top."""
    R = knife_pass_rot(deg)
    e0 = M.CRACK_TOP + M.STROKE[0] * M.CRACK_UP if start is None else start
    e1 = M.CRACK_BOT + M.STROKE[1] * M.CRACK_UP if end is None else end
    h = M.TAPE_COLLIDER_T + 0.0002 if height is None else height
    n = int(np.linalg.norm(e1 - e0) / (speed * M.DT)) + 1
    out = []
    for k in range(-4, n + 5):
        f = min(1.0, max(0.0, k / n))
        e = e0 + f * (e1 - e0) + h * M.NORMAL
        kp = knife_at_edge(e, R)
        out.append(objs(tape, kp, R, held=held))
    return out


def set_down(tape, steps=30):
    return [objs(tape) for _ in range(steps)]


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    assert cond, what


# ---------------------------------------------------------------------------------------------------------------------
def test_start_and_nothing():
    fresh()
    check(feed([objs(tape_on_bench())]) == 0.0, "start state scores 0 (tape on the bench)")
    check(feed([objs(tape_on_bench()) for _ in range(40)]) == 0.0, "doing nothing scores 0")


def test_full_job_scores_1():
    fresh()
    tape = tape_laid()
    s = feed(stroke(tape))
    st_ = M._L.view["st"]
    check(all(st_["covered"]), "every station covered")
    check(max(st_["offset"]) < 1e-6, "centered")
    check(all(st_["bonded"]), f"every station bonded ({sum(st_['bonded'])})")
    check(all(abs(d - 0.0008) < 1e-6 for d in st_["depth"]), f"0.8 mm under the tape: {st_['depth'][:3]}")
    check(all(st_["squeezed"]), "squeezed out at both edges everywhere")
    check(s == 0.75, f"bedded, knife still in hand: 0.75 ({s})")
    s = feed(set_down(tape))
    check(s == 1.0, f"knife set down, hands off, still 1 s: 1.0 ({s})")


def test_hold_needs_a_full_second():
    fresh()
    tape = tape_laid()
    feed(stroke(tape))
    check(feed(set_down(tape, 10)) == 0.75, "set down 0.5 s: still 0.75")


def test_laid_not_bedded():
    fresh()
    check(feed([objs(tape_laid())] * 30) == 0.0, "laid centered, no knife: 0 (the knife never worked it)")


def test_off_line():
    fresh()
    tape = tape_laid(offset=0.005)
    check(feed(stroke(tape) + set_down(tape)) == 0.25, "5 mm off the line, knife-bedded: 0.25")
    fresh()
    tape = tape_laid(turn_deg=3.0)
    check(feed(stroke(tape) + set_down(tape)) == 0.25, "turned 3 degrees (14 mm off at the bottom), bedded: 0.25")
    fresh()
    tape = tape_laid(offset=0.020)
    check(feed(stroke(tape) + set_down(tape)) == 0.25, "20 mm off, bedded: the crack still under the tape (0.25)")
    fresh()
    check(feed([objs(tape_laid(offset=0.005))] * 5) == 0.0, "5 mm off, no knife: 0")
    fresh()
    check(feed([objs(tape_laid(offset=0.030))] * 5) == 0.0, "30 mm off: the crack not under the tape (0)")
    fresh()
    check(feed([objs(tape_laid(turn_deg=15.0))] * 5) == 0.0, "turned 15 degrees: misses the crack's bottom (0)")
    fresh()
    tape = tape_laid(offset=0.005)
    s = feed(stroke(tape) + set_down(tape))
    check(s == 0.25, f"off the line and bedded: 0.25 ({s})")


def test_dropped_loosely():
    fresh()
    check(feed([objs(tape_laid(lift=0.008))] * 10) == 0.0, "tape held 8 mm off the face (not on the bed): 0")
    fresh()
    tape = tape_laid(lift=0.003)
    s = feed(stroke(tape, height=0.003 + M.TAPE_COLLIDER_T + 0.0002) + set_down(tape))
    check(M._L.bed.bonded.sum() == 0, "knife over a tape held 3 mm off the face bonds nothing")
    check(s == 0.0, f"... and the knife never pressed it: 0 ({s})")


def test_pressed_by_hand_not_knife():
    fresh()
    tape = tape_laid()
    states = []
    for k in range(80):
        p = M.CRACK_TOP + (M.CRACK_BOT - M.CRACK_TOP) * k / 79 + M.NORMAL * M.TAPE_COLLIDER_T
        states.append(objs(tape, hand_on=p))
    s = feed(states + set_down(tape))
    check(M._L.bed.bonded.sum() == 0 and M._L.bed.pressed.sum() == 0, "fingers run down the tape bed nothing")
    check(s == 0.0, f"pressed by hand, no knife: 0 ({s})")


def test_knife_not_held():
    fresh()
    tape = tape_laid()
    s = feed(stroke(tape, held=False) + set_down(tape))
    check(M._L.bed.bonded.sum() == 0, "a knife moved with no hand on it beds nothing")
    check(s == 0.0, f"0 ({s})")


def test_steep_pass_starves():
    fresh()
    tape = tape_laid()
    s = feed(stroke(tape, deg=65.0) + set_down(tape))
    check(M._L.bed.starved.sum() > 20 and M._L.bed.bonded.sum() == 0, "a 65 degree pass scrapes the bed thin: starved")
    check(s == 0.5, f"steep pass: 0.5 ({s})")
    fresh()
    tape = tape_laid()
    s = feed(stroke(tape, deg=50.0) + set_down(tape))
    check(s == 1.0, f"50 degrees leaves {M.LAW.film(50.0) * 1000:.2f} mm: bedded ({s})")


def test_starved_bed():
    fresh(lambda U, V: M.start_bed(U, V, depth=0.0003))
    tape = tape_laid()
    s = feed(stroke(tape) + set_down(tape))
    check(M._L.bed.bonded.sum() == 0 and not any(M._L.view["st"]["squeezed"]), "a dry 0.3 mm bed: nothing squeezes out")
    check(s == 0.5, f"starved bed: 0.5 ({s})")
    fresh(lambda U, V: M.start_bed(U, V, depth=0.0009))
    tape = tape_laid()
    s = feed(stroke(tape) + set_down(tape))
    check(s == 0.5, f"a thin 0.9 mm bed: too little squeezes out to count ({s})")


def test_compound_without_tape():
    fresh()
    s = feed(stroke(tape_on_bench(), height=0.0002) + set_down(tape_on_bench()))
    check(M._L.bed.scraped > 0 and s == 0.0, f"knife pushed along the bed with no tape: 0 ({s})")


def test_partial_bedding():
    fresh()
    tape = tape_laid()
    mid = M.CRACK_MID_W
    s = feed(stroke(tape, end=mid) + set_down(tape))
    check(s == 0.5, f"knife stopped halfway: 0.5 ({s})")


def test_wrinkle():
    fresh()
    tape = tape_laid()
    feed(stroke(tape))
    mid = int(round((M.TOP_OVER + M.CRACK_LEN / 2) / M.TAPE.widths[0]))  # the strip over the crack's middle
    bumped = tape_laid(bump=(mid, 0.002))
    s = feed(set_down(bumped))
    check(s == 0.75, f"a strip over the crack 2 mm off the face (a ridge): 0.75 ({s})")


def test_hand_left_on_tape():
    fresh()
    tape = tape_laid()
    feed(stroke(tape))
    s = feed([objs(tape, hand_on=M.CRACK_TOP + 0.04 * M.CRACK_UP + M.NORMAL * M.TAPE_COLLIDER_T)] * 30)
    check(s == 0.75, f"a hand still on the tape: 0.75 ({s})")


def test_knife_left_on_wall():
    fresh()
    tape = tape_laid()
    sts = stroke(tape)
    feed(sts)
    o, _ = sts[-1]
    kp, kq = o["knife"].pos, bk.rotm(o["knife"].quat_xyzw)
    s = feed([objs(tape, kp, kq, held=False)] * 30)
    check(s == 0.75, f"knife left on the wall, released: 0.75 ({s})")


def test_peel():
    fresh()
    tape = tape_laid()
    feed(stroke(tape) + set_down(tape))
    i = M.STATIONS.shape[0] // 2 + round(M.TOP_OVER / M.TAPE.widths[0])  # a strip over the crack's middle
    peeled = tape_laid(bump=(i, 0.003))
    feed([objs(peeled)])
    check(not M._L.bed.bonded[i] and M._L.peels == 1, "a bonded strip lifted 3 mm lets go (peeled)")
    check(M._L.bed.bonded[i - 1] and M._L.bed.bonded[i + 1], "its neighbours stay bonded")


def test_new_trial_resets():
    fresh()
    tape = tape_laid()
    feed(stroke(tape) + set_down(tape))
    fresh()
    check(M._L.bed.bonded.sum() == 0 and abs(M._L.bed.h.max() - M.BED_T) < 1e-12, "a new trial: fresh bed, no bonds")


def test_bed_volume_conserved():
    fresh()
    tape = tape_laid()
    v0 = float(M._L.bed.h.sum()) * M._L.bed.area
    feed(stroke(tape))
    v1 = float(M._L.bed.h.sum()) * M._L.bed.area + M._L.bed.scraped
    check(abs(v1 - v0) < 1e-12, f"compound conserved (squeezed out + struck onto the blade): {v0 * 1e6:.2f} mL")


def test_usd_patch_and_bonds():
    from pxr import Usd, UsdGeom, UsdPhysics

    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/World/envs/env_0")
    prim = UsdGeom.Xform.Define(stage, "/World/envs/env_0/tape").GetPrim()
    UsdGeom.Xform.Define(stage, "/World/envs/env_0/tape/base")
    for i in range(M.TAPE.n):
        if i != M.TAPE.root:
            UsdGeom.Xform.Define(stage, f"/World/envs/env_0/tape/{M.TAPE.link_name(i)}")
    M._author_bonds(prim)
    joints = [p for p in stage.Traverse() if p.GetName().startswith("bond_")]
    check(len(joints) == M.TAPE.n, "one bond joint per strip")
    j = UsdPhysics.Joint(joints[0])
    check(j.GetJointEnabledAttr().Get() is False, "authored switched off")
    check(j.GetExcludeFromArticulationAttr().Get() is True, "outside the articulation")
    dx = UsdPhysics.DriveAPI(joints[0], "transX")
    check(abs(dx.GetStiffnessAttr().Get() - M.BOND_PEEL / M.BOND_STRETCH) < 1e-6, "peel spring")
    dy = UsdPhysics.DriveAPI(joints[0], "transY")
    check(dy.GetStiffnessAttr().Get() == 0.0 and abs(dy.GetMaxForceAttr().Get() - M.BOND_SHEAR) < 1e-6,
          "shear: a capped damper, no spring (no pull toward a set point)")
    for ax in ("rotX", "rotY", "rotZ"):
        check(not joints[0].HasAPI(UsdPhysics.LimitAPI, ax), f"{ax} free")
    q = j.GetLocalRot0Attr().Get()
    R0 = bk.rotm((q.GetImaginary()[0], q.GetImaginary()[1], q.GetImaginary()[2], q.GetReal()))
    check(np.allclose(R0[:, 0], -M.NORMAL, atol=1e-6), "frame 0's x is into the wall (the peel axis)")
    p0 = np.array(j.GetLocalPos0Attr().Get(), float)
    check(abs(float((p0 - M.face_point(0.0, 0.0)) @ M.NORMAL) - M.TAPE_COLLIDER_T / 2) < 1e-6,
          "frame 0 lies where a lying strip's middle is (half its box out from the face)")
    c, _ = M.TAPE.box_local(0)
    check(np.allclose(np.array(j.GetLocalPos1Attr().Get(), float), c, atol=1e-7),
          "frame 1 is the strip's middle (the same whichever side of the tape is on the wall)")


def test_tape_springs_from_the_start():
    mech = [o for o in M.OBJECTS if o.name == "tape"][0]
    names = {lk.name for lk in mech.links}
    check(set(M._TAPE_GAINS) == names, "every tape hinge gets its actuator spring and damping at creation")
    check(all(k > 0 and c > 0 for k, c in M._TAPE_GAINS.values()), "positive gains")
    import types
    import isaaclab.actuators as act
    seen = {}

    def fake_plain(scene, spec):
        cfg = act.ImplicitActuatorCfg(joint_names_expr=["s1_joint"], stiffness=0.0, damping=0.0, friction=0.0)
        seen["cfg"] = cfg
        return types.SimpleNamespace(cfg=cfg)

    old = M._plain_mech_object
    M._plain_mech_object = fake_plain
    try:
        M._mech_object_tape(None, mech)
    finally:
        M._plain_mech_object = old
    k, c = M._TAPE_GAINS["s1"]
    check(abs(seen["cfg"].stiffness - k) < 1e-12 and abs(seen["cfg"].damping - c) < 1e-12,
          "the tape's actuator is created with the hinge's spring, not zero")
    check(act.ImplicitActuatorCfg is not None and isinstance(act.ImplicitActuatorCfg, type), "the patch is undone")


def test_tape_is_real():
    check(abs(M.TAPE_W - 2.0625 * 0.0254) < 1e-4, "2-1/16 in. wide")
    check(abs(M.TAPE.total_mass / M.TAPE_LEN - 0.138 * M.TAPE_W) < 1e-6, "7.2 g/m (138 g/m^2)")
    check(abs(M.TAPE.widths[0] - 0.0067) < 1e-4 and M.TAPE.n <= 64, "6.7 mm pitch, one articulation")
    check(M.TAPE.mass(0) < 1e-4, f"each strip weighs {M.TAPE.mass(0) * 1000:.3f} g (not a heavy link)")


def test_demo_plan():
    d = M.demo()
    check(set(d) == {"left", "right"}, "two arms")
    total = max(sum(w.duration for w in v) for v in d.values())
    check(total / M.DT < M.RUN_LIMIT_STEPS - 100, f"demo fits the run limit ({total:.1f} s)")
    R = M.R_KNIFE_PASS
    check(abs(bk.blade_angle_deg(R, M.NORMAL) - M.KNIFE_DEG) < 1e-6, "the pass is at KNIFE_DEG to the face")
    check(abs(float(R[:, 1] @ M.DOWN)) < 1e-9 and abs(float(R[:, 1] @ M.NORMAL)) < 1e-9,
          "the edge lies square across the tape, flat to the face")
    check(np.allclose(M.R_PICK @ M.R_PIN.T @ (-M.NORMAL), M.TAPE_DIR, atol=1e-9),
          "the hold is the same on the bench and at the face (the tape leaves the fingertips along them)")


def test_demo_reach():
    """Every control step of both arms' plans within 10 mm of its target (a hand tipped 30 degrees near the face would
    leave the left's IK at joint 3's stop, 12 cm off the hold, the set pass and home)."""
    from yam_arena import ik
    d = M.demo()
    for side, base in _station.BASES.items():  # the sim's bases (SCENE "station"), as yam_scripted uses
        plan = _demo.ArmPlan(d[side], base)
        worst, where = 0.0, ""
        for k in range(int(plan.duration / M.DT)):
            a, label = plan.action7(k * M.DT)
            pos = plan.target(k * M.DT)[0]
            e = float(np.linalg.norm(ik.tcp_pose(a[:6], base)[0] - pos))
            if e > worst:
                worst, where = e, label
        check(worst < 0.010, f"{side} plan reachable: worst IK miss {worst * 1000:.1f} mm ({where})")


def _tilted_pass_rot(deg=M.KNIFE_DEG, tilt_deg=0.0, turn_deg=0.0):
    """The pass rotation with the edge tilted out of the face (about the blade's own x) and/or turned in the face's
    plane (about the face normal)."""
    R = knife_pass_rot(deg)
    t = math.radians(tilt_deg)
    Rx = np.array([[1, 0, 0], [0, math.cos(t), -math.sin(t)], [0, math.sin(t), math.cos(t)]])
    u = math.radians(turn_deg)
    n = M.NORMAL
    K = np.array([[0, -n[2], n[1]], [n[2], 0, -n[0]], [-n[1], n[0], 0]])
    Rn = np.eye(3) + math.sin(u) * K + (1 - math.cos(u)) * K @ K
    return Rn @ R @ Rx


def _stroke_R(tape, R, lateral=0.0, lift_extra=0.0):
    e0 = M.CRACK_TOP + M.STROKE[0] * M.CRACK_UP + lateral * M.ACROSS
    e1 = M.CRACK_BOT + M.STROKE[1] * M.CRACK_UP + lateral * M.ACROSS
    n = int(np.linalg.norm(e1 - e0) / (M.SPEED * M.DT)) + 1
    low = np.array([[M.BLADE_LEN, y, M.BLADE_Z - M.BLADE_T / 2] for y in np.linspace(-M.BLADE_W / 2, M.BLADE_W / 2, 41)])
    out = []
    for k in range(-4, n + 5):
        f = min(1.0, max(0.0, k / n))
        e = e0 + f * (e1 - e0)
        # put the lowest edge sample on the tape's top
        kp0 = e - R @ np.array([M.BLADE_LEN, 0.0, M.BLADE_Z - M.BLADE_T / 2])
        h = float(np.min((low @ R.T + kp0 - M.face_point(0, 0)) @ M.NORMAL))
        kp = kp0 + (M.TAPE_COLLIDER_T + 0.0002 + lift_extra - h) * M.NORMAL
        out.append(objs(tape, kp, R, held=True))
    return out


def test_corner_drag_tears():
    fresh()
    tape = tape_laid()
    s = feed(_stroke_R(tape, _tilted_pass_rot(tilt_deg=12.0)) + set_down(tape))
    check(s <= 0.5, f"only the knife's corner pressing (edge tilted 12 degrees): at most 0.5 ({s}, torn {M._L.bed.torn})")


def test_edge_turned_along_tape():
    fresh()
    tape = tape_laid()
    s = feed(_stroke_R(tape, _tilted_pass_rot(turn_deg=80.0)) + set_down(tape))
    check(s == 0.5, f"edge turned 80 degrees (pressing a narrow band along the tape): 0.5 ({s})")


def test_knife_off_to_one_side():
    fresh()
    tape = tape_laid()
    s = feed(_stroke_R(tape, knife_pass_rot(), lateral=0.040) + set_down(tape))
    check(s == 0.5, f"knife 40 mm to one side (one edge band never pressed): 0.5 ({s})")


def test_bonded_tape_slid_onto_line():
    fresh()
    tape = tape_laid(offset=0.006)
    feed(stroke(tape, start=M.CRACK_TOP + M.STROKE[0] * M.CRACK_UP + 0.006 * M.ACROSS,
                end=M.CRACK_BOT + M.STROKE[1] * M.CRACK_UP + 0.006 * M.ACROSS))
    s = feed(set_down(tape_laid()))
    check(s == 0.0 and M._L.bed.bonded.sum() == 0,
          f"bedded 6 mm off, then slid onto the line: bonds let go and the knife's work is undone, 0 ({s})")


def test_finger_pressing_during_pass():
    fresh()
    tape = tape_laid()
    sts = []
    p = M.CRACK_MID_W + M.NORMAL * M.TAPE_COLLIDER_T
    for o, op in stroke(tape):
        o2, op2 = objs(tape, o["knife"].pos, bk.rotm(o["knife"].quat_xyzw), held=True, hand_on=p)
        sts.append((o2, op2))
    s = feed(sts + set_down(tape))
    check(s <= 0.5, f"a finger pressed on the tape over the crack during the pass: not bedded there ({s})")


def _hand_run(tape, a=None, b=None, n=60):
    """States of closed fingertips run down the tape's top from a to b (default: the crack's top to its bottom)."""
    a = M.CRACK_TOP if a is None else a
    b = M.CRACK_BOT if b is None else b
    return [objs(tape, hand_on=a + (b - a) * k / (n - 1) + M.NORMAL * M.TAPE_COLLIDER_T) for k in range(n)]


def test_tack_needs_a_press_on_the_bed():
    fresh()
    feed([objs(tape_laid())] * 20)
    check(M._L.tacked.sum() == 0, "tape lying on the bed with no finger on it: no tack")
    fresh()
    s = feed(_hand_run(tape_laid()) + set_down(tape_laid()))
    check(M._L.tacked.sum() >= round(M.CRACK_LEN / M.TAPE.widths[0]), f"fingers run down the tape on the bed: the pressed strips stick ({M._L.tacked.sum()})")
    check(M._L.bed.bonded.sum() == 0 and M._L.bed.pressed.sum() == 0, "tack is not a bond and presses no compound")
    check(s == 0.0, f"tacked by hand, no knife: 0 ({s})")
    fresh()
    sw = [objs(tape_laid(), hand_on=M.CRACK_TOP + (M.CRACK_BOT - M.CRACK_TOP) * k / 5 + M.NORMAL * M.TAPE_COLLIDER_T)
          for k in range(6)]
    s = feed(sw + set_down(tape_laid()))
    check(s == 0.0, f"a gripper swiped down the tape in 6 steps, no knife: 0 ({s})")
    for hover in (0.0015, 0.0025):
        fresh()
        feed(_hand_run(tape_laid(), M.CRACK_TOP + hover * M.NORMAL, M.CRACK_BOT + hover * M.NORMAL))
        check(M._L.tacked.sum() == 0, f"fingertips hovering {hover * 1000:.1f} mm above the tape tack nothing")
    fresh()
    tape = tape_laid(lift=0.003)
    feed(_hand_run(tape))
    check(M._L.tacked.sum() == 0, "fingers on a tape 3 mm off the face: no tack (no reach from a distance)")
    fresh()
    off = 0.08  # the tape's middle 8 cm across, off the 10 cm bed (bare face)
    tape = tape_laid(offset=off)
    feed(_hand_run(tape, M.CRACK_TOP + off * M.ACROSS, M.CRACK_BOT + off * M.ACROSS))
    check(M._L.tacked.sum() == 0, "pressed on bare face beside the bed: no tack")
    fresh(lambda U, V: np.zeros_like(U))
    feed(_hand_run(tape_laid()))
    check(M._L.tacked.sum() == 0, "no compound at all: a hand press sticks nothing")


def test_tack_peels_and_bond_replaces_it():
    fresh()
    tape = tape_laid()
    feed(_hand_run(tape))
    n0 = int(M._L.tacked.sum())
    feed([objs(tape_laid(lift=0.004))] * 3)
    check(M._L.tacked.sum() == 0 and n0 > 0, "lifting the tacked tape 4 mm lets the tack go")
    fresh()
    tape = tape_laid()
    s = feed(_hand_run(tape) + stroke(tape) + set_down(tape))
    check(not np.any(M._L.tacked & M._L.bed.bonded), "a bonded strip carries no tack")
    check(s == 1.0, f"set by hand, then bedded by the knife: the full job scores 1.0 ({s})")


def test_text_matches():
    tj = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    for key in ("setup", "scoring", "fidelity"):
        check(key in tj and tj[key], f"task.json has {key}")
    text = tj["scoring"] + readme
    for val in ("3 mm", "0.4", "1.6 mm", "90%", "1 cm", "1 s"):
        check(val in text, f"text states {val}")
    check(tj["instruction"] == M.INSTRUCTION, "instruction matches")
    check(not re.search(r"\bspread", tj["title"].lower()), "the title is about laying tape, not spreading")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
    print("all passed")
    sys.exit(0)


def test_flipped_tape_reads_as_lying():
    """The demo's fold lays the tape bench-top side down (strip z into the wall). Read from a fixed -z side, a flat strip
    would seem 1 mm off the face, so pressed strips would not tack (TACK_GAP 1 mm), every strip would read as a wrinkle
    (FLAT 0.7 mm) and a bond would peel at once (BOND_STRETCH 0.5 mm). Either side down reads the same."""
    fr = bk.face_frame(M.FACE, M.FIXED_POSES["panel"])
    for flipped in (False, True):
        fresh()
        tape = tape_laid(flipped=flipped)
        check(float(tape[0][1][:, 2] @ M.NORMAL) * (-1 if flipped else 1) > 0.99, f"flipped={flipped}: built that way")
        lifts = [M._L.bed.lift(fr, tape, i) for i in range(M.TAPE.n)]
        check(max(abs(v) for v in lifts) < 1e-6, f"flipped={flipped}: flat strips read 0 mm off the face")
        fresh()
        s = feed(_hand_run(tape) + set_down(tape))
        n = int(M._L.tacked.sum())
        check(n >= round(M.CRACK_LEN / M.TAPE.widths[0]) and s == 0.0, f"flipped={flipped}: finger-set tacks {n}, 0")
        fresh()
        s = feed(stroke(tape) + set_down(tape))
        check(s == 1.0, f"flipped={flipped}: knife-bedded, no wrinkle read: 1.0 ({s})")
        fresh()
        feed(_hand_run(tape) + [objs(tape_laid(lift=0.004, flipped=flipped))] * 3)
        check(M._L.tacked.sum() == 0, f"flipped={flipped}: lifted 4 mm, the tack lets go")


def test_set_passes_never_slide_on_the_tape():
    """Fingertips run down the tape would drag it with the pads' friction (off the face, out of the left's pinch), so
    the set passes dab: wherever the planned fingertips are within 3 mm of the tape's top they do not move along the
    face by more than 1 mm a second."""
    d = M.demo()
    o, n = M.face_point(0.0, 0.0), M.NORMAL
    for side in ("right",):  # the left aims its presses along the arm's give: test_left_dabs_press_straight_in
        plan = _demo.ArmPlan(d[side], _station.BASES[side])
        prev, worst, presses = None, 0.0, 0
        for k in range(int(plan.duration / M.DT)):
            pos, R = plan.target(k * M.DT)[:2]
            if not np.isclose(np.asarray(R)[:, 2] @ -n, 1.0, atol=0.15):
                prev = None
                continue
            tip = np.asarray(pos) + M.PAD_TIP_PAST_TCP * np.asarray(R)[:, 2]
            near = float((tip - o) @ n) < M.TAPE_COLLIDER_T + 0.003
            if near and prev is not None:
                dp = tip - prev
                worst = max(worst, float(np.linalg.norm(dp - (dp @ n) * n)) / M.DT)
            presses += near and prev is None
            prev = tip if near else None
        check(worst < 0.001, f"{side}: the fingertips never slide along the tape while on it ({worst * 1000:.2f} mm/s)")
        check(presses >= 4, f"{side}: it sets the tape in dabs ({presses})")


def test_left_dabs_press_straight_in():
    """A dab aimed straight into the face slides the fingertips about 1 mm sideways as the arm's joint springs give
    under the press, and the pads carry the loose tape off the crack. Modelled with
    those springs (the face stops the fingertips; they give by C n, C = J K^-1 J^T), the left's fingertips do not move
    along the face while they press, and every press lands on the tape's middle."""
    from yam_arena import ik
    d, base = M.demo(), _station.BASES["left"]
    plan = _demo.ArmPlan(d["left"], base)
    o, n = M.face_point(0.0, 0.0), M.NORMAL
    slides, centres, straight = [], [], []
    start = None
    for k in range(int(plan.duration / M.DT)):
        a, label = plan.action7(k * M.DT)
        p, R = ik.tcp_pose(a[:6], base)
        if not np.isclose(R[:, 2] @ -n, 1.0, atol=0.15):
            continue
        tip = p + M.PAD_TIP_PAST_TCP * R[:, 2]
        press = M.TAPE_COLLIDER_T - float((tip - o) @ n)  # how far the aim is past the tape's top
        if press <= 0:
            start = None
            continue
        c = M.tip_compliance(a[:6], base) @ n
        u = c / float(c @ n)
        at = tip + press * u  # where the face stops the fingertips
        if start is None:
            start = at
            centres.append(float((at - M.CRACK_MID_W) @ M.ACROSS))
            straight.append(float(np.linalg.norm(u - n)) * M.LEFT_PRESS)
        dp = at - start
        slides.append(float(np.linalg.norm(dp - (dp @ n) * n)))
    check(len(centres) >= 4, f"the left presses the top part in dabs ({len(centres)})")
    check(max(slides) < 0.0002, f"the left's fingertips stay where they land while pressing ({max(slides) * 1000:.2f} mm)")
    check(max(abs(c) for c in centres) < 0.0005,
          f"every press lands on the crack's line ({max(abs(c) for c in centres) * 1000:.2f} mm off)")
    check(max(abs(c) for c in centres) + 0.0064 < M.TAPE_W / 2, "the closed fingertips lie inside the tape's width")
    check(min(straight) > 0.0005, f"aimed straight in, each press would slide {min(straight) * 1000:.1f}+ mm (why the aim follows the arm's give)")



# -- every arm link's hulls (the simulator fills LINK_POSES each step; here a link is placed by hand) ------------------
def link_on(point, normal, link="link5", robot="left_robot", gap=0.0005):
    """{key: (pos, R)} placing `link`'s hulls (its own frame unturned) so that their lowest point along `normal` lies
    `gap` out from `point` and every other hull point is further out."""
    import yamgeom as yg
    pts = yg.cloud(link)[0]
    low = pts[int(np.argmin(pts @ np.asarray(normal, float)))]
    return {f"{robot}/{link}": (np.asarray(point, float) + gap * np.asarray(normal, float) - low, np.eye(3))}


def with_links(links, states):
    """Feed `states` with these arm link poses, then clear them."""
    M.LINK_POSES.clear()
    M.LINK_POSES.update(links)
    try:
        return feed(states)
    finally:
        M.LINK_POSES.clear()


def test_forearm_left_on_tape():
    """No part of either arm on the tape at the end: a forearm (link5) resting on bedded tape over the crack, with
    both grippers away, caps it at 0.75; its touch also latches the strip it rests on."""
    fresh()
    tape = tape_laid()
    feed(stroke(tape))
    p = M.CRACK_MID_W + M.NORMAL * M.TAPE_COLLIDER_T
    links = link_on(p, M.NORMAL)
    check(M.arm_gap(objs(tape)[0], M.STRIPS[0].key, M.STRIP_BOXES[M.STRIPS[0].key]) == M.LINK_FAR,
          "no link poses: no arm gap")
    M.LINK_POSES.update(links)
    g = min(M.arm_gap(objs(tape)[0], s.key, M.STRIP_BOXES[s.key]) for s in M.STRIPS)
    M.LINK_POSES.clear()
    check(abs(g - 0.0005) < 2e-4, f"the forearm's hulls 0.5 mm off the tape ({g * 1000:.2f} mm)")
    s = with_links(links, set_down(tape))
    check(s == 0.75, f"a forearm left on the bedded tape: 0.75 ({s})")
    check(M._L.fingered.sum() >= 1, f"the strip under it is latched ({int(M._L.fingered.sum())})")
    fresh()
    feed(stroke(tape))
    far = link_on(p + 0.01 * M.NORMAL, M.NORMAL)
    s = with_links(far, set_down(tape))
    check(s == 1.0, f"the same forearm 1 cm off the tape: 1.0 ({s})")


def test_link_on_tape_during_pass_latches():
    fresh()
    tape = tape_laid()
    p = M.CRACK_MID_W + M.NORMAL * M.TAPE_COLLIDER_T
    s = with_links(link_on(p, M.NORMAL, "link4", "right_robot"), stroke(tape) + set_down(tape))
    check(M._L.fingered.sum() >= 1, "an arm link on bedded tape over the crack during the pass latches it")
    check(s < 1.0, f"and the trial cannot score 1.0 ({s})")


def test_wrist_pinning_knife_is_not_released():
    fresh()
    tape = tape_laid()
    feed(stroke(tape))
    kp, kR = np.asarray(M.KNIFE_START, float), KNIFE_REST_R
    top = kp + kR @ (np.asarray(M.HANDLE[0]) + np.array([0.0, 0.0, M.HANDLE[1][2] / 2]))
    links = link_on(top, np.array([0.0, 0.0, 1.0]), "gripper", "right_robot", gap=0.001)
    s = with_links(links, set_down(tape))
    check(s == 0.75, f"the knife on the bench with a wrist resting on its handle is not released: 0.75 ({s})")


class _Arr:
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


def _fake_base(names):
    class Art:
        body_names = list(names)
        data = type("D", (), {"body_pos_w": _Arr(np.zeros((1, len(names), 3))),
                              "body_quat_w": _Arr(np.tile([1.0, 0, 0, 0], (1, len(names), 1)))})()

    class Scene(dict):
        env_origins = _Arr(np.zeros((1, 3)))

    sc = Scene(left_robot=Art(), right_robot=Art())
    return type("B", (), {"scene": sc})()


def test_missing_link_or_bonds_stop_the_trial():
    import pytest
    old = (list(M._BONDS["paths"]), list(M._BONDS["tack_paths"]), list(M._READ_ONCE))
    try:
        with pytest.raises(RuntimeError):
            M._read_links(_fake_base(M.ARM_LINKS[:-1]))  # no tip_right
        M._READ_ONCE.clear()
        M._BONDS["paths"], M._BONDS["tack_paths"] = [], []
        with pytest.raises(RuntimeError):
            M._read_links(_fake_base(M.ARM_LINKS))  # bonds never authored
        M._BONDS["paths"] = M._BONDS["tack_paths"] = ["x"] * M.TAPE.n
        M._read_links(_fake_base(M.ARM_LINKS))
        check(len(M.LINK_POSES) == 2 * len(M.ARM_LINKS), "every link of both arms read")
    finally:
        M._BONDS["paths"], M._BONDS["tack_paths"] = old[0], old[1]
        M._READ_ONCE[:] = old[2]
        M.LINK_POSES.clear()


def test_variants_plans(monkeypatch):
    """The sim cheat variants (DRYWALL_VARIANT, read in demo() only): their plans and where they end."""
    from yam_arena import ik
    got = {}
    for v in M.VARIANTS:
        monkeypatch.setenv("DRYWALL_VARIANT", v)
        got[v] = M.demo()
    monkeypatch.setenv("DRYWALL_VARIANT", "nonsense")
    import pytest
    with pytest.raises(ValueError):
        M.demo()
    labels = {v: {s: [w.label for w in d[s]] for s in d} for v, d in got.items()}
    check("grip" not in labels["hand_only"]["right"] and labels["hand_only"]["left"][-1] == "home",
          "hand_only: no knife; both hands end at home, off the tape")
    check(labels["bare_bed"]["left"] == ["home"] and "pinch the top end" not in labels["bare_bed"]["right"]
          and labels["bare_bed"]["right"][-3:] == ["let go", "lift clear", "wait"],
          "bare_bed: the tape is never picked up; the knife is drawn and laid back")
    check(labels["knife_left"]["right"][-1] == "keep the knife on the wall" and "let go" not in labels["knife_left"]["right"],
          "knife_left: ends holding the knife on the wall (no home or lay-down after it)")
    check(labels["steep"]["right"] == labels[""]["right"], "steep: the same plan as the demo ...")
    k = labels["steep"]["right"].index("draw the knife down the tape")
    _, R_s = M._pass_rotations(M.STEEP_DEG)
    w = got["steep"]["right"][k]
    plan = _demo.ArmPlan(got["steep"]["right"], _station.BASES["right"])
    t_end = sum(x.duration for x in got["steep"]["right"][:k + 1]) - 1e-3
    a, _ = plan.action7(0.0)
    worst = 0.0
    for i in range(int(plan.duration / M.DT)):
        a, _ = plan.action7(i * M.DT)
        worst = max(worst, float(np.linalg.norm(ik.tcp_pose(a[:6], _station.BASES["right"])[0]
                                                - plan.target(i * M.DT)[0])))
    p, R = ik.tcp_pose(plan.action7(t_end)[0][:6], _station.BASES["right"])
    check(worst < 0.010, f"steep: reachable (worst IK miss {worst * 1000:.1f} mm)")
    check(abs(bk.blade_angle_deg(R @ R_s.T @ M._pass_rotations(M.STEEP_DEG)[0], M.NORMAL) - M.STEEP_DEG) < 1.0
          and isinstance(w, kit.Tilted), f"... with the blade at {M.STEEP_DEG:.0f} degrees")
    fresh()
    tape = tape_laid()
    sc = feed(stroke(tape, deg=M.STEEP_DEG, start=M.CRACK_TOP + M.STEEP_START * M.CRACK_UP) + set_down(tape))
    check(sc == 0.5, f"steep: its stroke (60 degrees, from 4 cm below the top end) starves the bed: 0.5 ({sc})")
