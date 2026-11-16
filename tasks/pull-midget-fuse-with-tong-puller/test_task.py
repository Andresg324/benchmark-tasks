"""Offline checks of the clip law, the readouts and the scorer (no simulator). Run:
    python tasks/pull-midget-fuse-with-tong-puller/test_task.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

import task as T  # noqa: E402
import clipkit as ck  # noqa: E402

QI = (0.0, 0.0, 0.0, 1.0)


def qy(a):
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def qz(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def qmul(a, b):
    return np.array(T.kit._quat_mul(a, b))


def st(pos, quat=QI, vel=(0.0, 0.0, 0.0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=p - 0.01, high=p + 0.01)


def seat(name):
    return np.array([T.X_F, T.POLES[name], T.Z_C])


def puller(objects, origin, quat=QI, close=0.0):
    """Puller base at `origin` (its frame origin), turned by quat; each jaw closed by `close` m at the jaw face."""
    objects["puller"] = st(origin, quat)
    ang = close / (T.JAW_X - T.HINGE_X)
    R = T.gk._rot(quat)
    for side, s in (("left", 1), ("right", -1)):
        anchor = np.array([T.HINGE_X, s * 0.0035, 0.0])
        objects[f"puller/{side}"] = st(np.asarray(origin, float) + R @ anchor, qmul(quat, qz(-s * ang)))


def scene(f2=None, f2_quat=QI, f2_vel=(0, 0, 0), f1=None, f3=None, puller_at=None, puller_quat=QI, close=0.0,
          grip_at=None):
    o = {}
    o["fuse_f1"] = st(seat("fuse_f1") if f1 is None else f1)
    o["fuse_f3"] = st(seat("fuse_f3") if f3 is None else f3)
    o["fuse_f2"] = st(seat("fuse_f2") if f2 is None else f2, f2_quat, f2_vel)
    o["tray"] = st((T.TRAY[0], T.TRAY[1], T.TOP + T.TRAY_SIZE[2] / 2))
    puller(o, puller_at if puller_at is not None else (T.PULLER_START[0], T.PULLER_START[1], T.TOP), puller_quat,
           close)
    g = grip_at if grip_at is not None else (0.10, -0.30, T.TOP + 0.30)
    o["right_gripper"] = st(g, (1.0, 0.0, 0.0, 0.0))  # pointing down
    o["left_gripper"] = st((0.10, 0.30, T.TOP + 0.30), (1.0, 0.0, 0.0, 0.0))
    return o


BENCH = (T.PULLER_START[0], T.PULLER_START[1], T.TOP)
GRIP_ORIGIN = np.array([T.X_F - (T.JAW_X), 0.0, T.Z_C - T.JAW_Z])  # puller origin with its jaws round F2's middle
CLOSE = T.JAW_IN - T.BODY_HALF  # each jaw's travel to the fuse's 9.6 mm prism


def fresh():
    T.WATCH._fresh()
    T.WATCH._ep = None


def run(frames):
    fresh()
    s = None
    for f in frames:
        s = T.score(f)
    return s


def check(name, got, want):
    ok = abs(got - want) < 1e-9 if isinstance(want, float) else got == want
    print(("ok   " if ok else "FAIL ") + f"{name}: {got} (want {want})")
    return ok


def main():
    res = []
    # -- geometry and clip law
    c = T.CLIP["fuse_f2"]
    res.append(check("seated fuse: no end out", T.ends_out(scene(), "fuse_f2"), [False, False]))
    top_out = seat("fuse_f2") + np.array([-0.0015, 0, 0])
    res.append(check("1.5 mm out: still in", T.ends_out(scene(f2=top_out), "fuse_f2"), [False, False]))
    piv = np.array([T.X_F + T.FUSE_R, 0.0, T.Z_C - T.END - T.FERRULE_L / 2])

    def rocked(a):
        R = T.gk._rot(qy(-a))
        return piv + R @ (seat("fuse_f2") - piv), qy(-a)

    p, q = rocked(math.radians(10))
    res.append(check("rocked 10 deg about the bottom clip: top out only", T.ends_out(scene(f2=p, f2_quat=q),
                                                                                      "fuse_f2"), [False, True]))
    holds = ck.Holds(T.CLIPS)
    poses = {n: (seat(n), QI) for n in T.POLES}
    f = holds.update(poses)
    res.append(check("held at start: 2 pushes each", [len(v) for v in f.values()], [2, 2, 2]))
    res.append(check("push is 6 N into the clips (+x)", [float(v) for v in f["fuse_f2"][0][1]], [6.0, 0.0, 0.0]))
    poses["fuse_f2"] = (p, q)
    f = holds.update(poses)
    res.append(check("rocked: top let go, bottom held", (len(f["fuse_f2"]), holds.held["fuse_f2"]), (1, [True, False])))
    poses["fuse_f2"] = (seat("fuse_f2") + np.array([-0.001, 0, 0]), QI)
    holds.update(poses)
    res.append(check("1 mm out is not reseated", holds.held["fuse_f2"], [True, False]))
    poses["fuse_f2"] = (seat("fuse_f2"), QI)
    holds.update(poses)
    res.append(check("pushed fully back: held again", holds.held["fuse_f2"], [True, True]))
    # a constant force on a 9 g body has no stiffness to ring at 240 Hz; the only switch is the release (one way until
    # reseated within 0.6 mm, a 1.9 mm hysteresis band)
    res.append(check("hysteresis band >= 1.5 mm", c.release - c.reseat >= 0.0015, True))

    # -- jaws
    grip = scene(puller_at=GRIP_ORIGIN, close=CLOSE)
    res.append(check("jaws closed round F2", T.jaws_on(grip), True))
    res.append(check("jaws open round F2 (15 mm): not on", T.jaws_on(scene(puller_at=GRIP_ORIGIN, close=0.001)),
                     False))
    res.append(check("jaws closed on a ferrule: not on the body",
                     T.jaws_on(scene(puller_at=GRIP_ORIGIN + np.array([0, 0, T.END]), close=CLOSE)), False))
    res.append(check("jaws closed beside F2 (on F1's side): not on",
                     T.jaws_on(scene(puller_at=GRIP_ORIGIN + np.array([0, 0.010, 0]), close=CLOSE)), False))
    res.append(check("puller on the bench: jaws not touching F2", T.jaw_touching(scene()), False))
    for dx in (-0.007, 0.007):
        res.append(check(f"jaws closed, fuse {dx * 1000:+.0f} mm along the jaw faces: on",
                         T.jaws_on(scene(puller_at=GRIP_ORIGIN + np.array([dx, 0, 0]), close=CLOSE)), True))
    res.append(check("jaws closed past their tips (fuse 12 mm beyond the faces' end): not on",
                     T.jaws_on(scene(puller_at=GRIP_ORIGIN + np.array([-0.021, 0, 0]), close=CLOSE)), False))

    # -- scoring
    res.append(check("nothing done: 0", run([scene()] * 3), 0.0))
    res.append(check("top end rocked out (cocked): 0.25",
                     run([scene(puller_at=GRIP_ORIGIN, close=CLOSE), scene(f2=p, f2_quat=q, puller_at=GRIP_ORIGIN,
                                                                           close=CLOSE)]), 0.25))
    out_pos = seat("fuse_f2") + np.array([-0.05, 0, 0.02])
    held_out = scene(f2=out_pos, puller_at=GRIP_ORIGIN + np.array([-0.05, 0, 0.02]), close=CLOSE)
    res.append(check("out of both clips in the jaws: 0.5", run([grip, held_out]), 0.5))
    tray_p = np.array([T.TRAY[0], T.TRAY[1], T.TOP + 0.007])
    lying = qy(math.pi / 2)
    in_tray = scene(f2=tray_p, f2_quat=lying, puller_at=BENCH)
    res.append(check("pulled in the jaws, let go in the tray, still: 1.0", run([grip, held_out, in_tray]), 1.0))
    moving = scene(f2=tray_p, f2_quat=lying, f2_vel=(0.1, 0, 0), puller_at=BENCH)
    res.append(check("in the tray but still rolling: 0.75", run([grip, held_out, moving]), 0.75))
    jaws_in_tray = scene(f2=tray_p, puller_at=tray_p - np.array([T.JAW_X, 0, T.JAW_Z]), close=CLOSE)
    res.append(check("in the tray still in the jaws: 0.75", run([grip, held_out, jaws_in_tray]), 0.75))
    tcp = tray_p + np.array([0, 0, 0.004])
    T.kit._OPENING["right_gripper"] = 0.0  # fingers closed on it
    in_hand = scene(f2=tray_p, f2_quat=lying, puller_at=BENCH, grip_at=tcp)
    res.append(check("in the tray, a gripper still on it: 0.75", run([grip, held_out, in_hand]), 0.75))
    T.kit._OPENING.clear()
    bench = scene(f2=np.array([0.25, -0.12, T.TOP + 0.005]), f2_quat=lying, puller_at=BENCH)
    res.append(check("dropped on the bench: 0.5", run([grip, held_out, bench]), 0.5))
    floor = scene(f2=np.array([0.36, 0.0, T.TOP + 0.007]), f2_quat=lying, puller_at=BENCH)
    res.append(check("dropped inside the enclosure: 0.25", run([grip, held_out, floor]), 0.25))
    # bare removal: out of both clips with no jaws on it, then put back, gripped and done properly: capped at 0.5
    bare = scene(f2=out_pos, grip_at=out_pos + np.array([0, 0, 0.004]))
    res.append(check("pulled by the gripper alone, into the tray: capped 0.5", run([scene(), bare, in_tray]), 0.5))
    res.append(check("pulled bare, put back, then done with the puller: still 0.5",
                     run([scene(), bare, scene(), grip, held_out, in_tray]), 0.5))
    knocked = seat("fuse_f2") + np.array([-0.06, 0, 0])
    res.append(check("knocked out of its clips into the enclosure: 0.25", run([scene(), scene(f2=knocked)]), 0.25))
    res.append(check("knocked out, then carried to the tray: capped 0.5",
                     run([scene(), scene(f2=knocked), grip, held_out, in_tray]), 0.5))
    # neighbours
    f1_out = seat("fuse_f1") + np.array([-0.004, 0, 0])
    res.append(check("F1 pulled 4 mm out for 2 steps, then the job done: 0",
                     run([scene(f1=f1_out), scene(f1=f1_out), grip, held_out, in_tray]), 0.0))
    res.append(check("F1 4 mm out for one step only (a physics pop): not counted",
                     run([scene(f1=f1_out), scene(), grip, held_out, in_tray]), 1.0))
    f3_side = seat("fuse_f3") + np.array([0, -0.004, 0])
    res.append(check("F3 knocked 4 mm sideways at the end: 0",
                     run([grip, held_out, in_tray, scene(f2=tray_p, f2_quat=lying, f3=f3_side,
                                                         puller_at=BENCH)] + [
                         scene(f2=tray_p, f2_quat=lying, f3=f3_side, puller_at=BENCH)]), 0.0))
    f1_brush = seat("fuse_f1") + np.array([-0.0015, 0, 0])
    res.append(check("F1 brushed 1.5 mm (still in its clips): no penalty",
                     run([scene(f1=f1_brush)] * 3 + [grip, held_out, in_tray]), 1.0))
    up_high = scene(f2=tray_p, f2_quat=lying, puller_at=(0.10, -0.15, T.TOP + 0.13),
                    grip_at=(0.10 + T.GRASP_X, -0.15, T.TOP + 0.13 + T.GRASP_Z))
    T.kit._OPENING["right_gripper"] = (T.REST_W - 0.0015) / T.W_GRIP
    res.append(check("fuse in the tray, puller still in hand: 0.75", run([grip, held_out, up_high]), 0.75))
    on_bench_held = scene(f2=tray_p, f2_quat=lying, puller_at=BENCH,
                          grip_at=(BENCH[0] + T.GRASP_X, BENCH[1], T.TOP + T.GRASP_Z))
    res.append(check("puller laid on the bench but still pinched: 0.75", run([grip, held_out, on_bench_held]), 0.75))
    T.kit._OPENING.clear()
    in_tray_puller = scene(f2=tray_p, f2_quat=lying, puller_at=(T.TRAY[0] - 0.06, T.TRAY[1] + 0.02, T.TOP + 0.003))
    res.append(check("puller left lying in the tray: 0.75", run([grip, held_out, in_tray_puller]), 0.75))
    elsewhere = scene(f2=tray_p, f2_quat=lying, puller_at=(0.12, 0.05, T.TOP))
    res.append(check("puller laid flat elsewhere on the bench, let go: 1.0", run([grip, held_out, elsewhere]), 1.0))
    res.append(check("success only at 1.0", (T.success(in_tray), T.success(moving)), (True, False)))

    # -- demo: every waypoint above the bench and reachable by the right arm's IK
    import yam_arena.demo as yd
    from yam_arena.station import BASES
    plan = T.demo()
    worst, low = 0.0, 1.0
    for w in plan["right"]:
        if isinstance(w, T.kit.Home):
            continue
        rot = yd.grip_rotation(getattr(w, "yaw", 0.0))
        if isinstance(w, T.kit.Tilted):
            rot = T.kit._rot_xyz(w.roll, w.pitch) @ rot
        q, _, _ = yd.ik.solve(np.asarray(w.pos, float), yd.ik.matrix_to_quat_xyzw(rot), q_init=None,
                              base_pos=np.asarray(BASES["right"], float))
        p, _ = yd.ik.tcp_pose(q, np.asarray(BASES["right"], float))
        worst = max(worst, float(np.linalg.norm(np.asarray(p) - np.asarray(w.pos))))
        low = min(low, w.pos[2] - T.TOP)
    res.append(check("demo: worst IK position error under 2 mm", worst < 0.002, True))
    print(f"     (worst IK error {worst * 1000:.2f} mm, lowest tool point {low * 1000:.1f} mm above the bench)")
    res.append(check("demo: tool point never below 13 mm (pad tips above the bench)", low >= 0.013, True))
    dur = sum(w.duration for w in plan["right"])
    print(f"     demo plan {dur:.1f} s = {dur * 20:.0f} steps")
    print("all pass" if all(res) else f"{res.count(False)} FAIL")
    return all(res)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
