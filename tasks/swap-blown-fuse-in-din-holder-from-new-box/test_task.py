"""Offline checks of the carrier laws, the readouts and the scorer, including adversarial runs (no simulator). Run:
    python tasks/swap-blown-fuse-in-din-holder-from-new-box/test_task.py
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

import task as T  # noqa: E402
import carrierkit as ck  # noqa: E402

QI = (0.0, 0.0, 0.0, 1.0)


def qy(a):
    return (0.0, math.sin(a / 2), 0.0, math.cos(a / 2))


def st(pos, quat=QI, vel=(0.0, 0.0, 0.0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.asarray(vel, float),
                       ang_vel=np.zeros(3), low=p - 0.01, high=p + 0.01)


def seat_pose(a):
    """A fuse lying seated in F2's carrier open `a` rad: (pos, quat)."""
    return T.car_point((T.FX, 0.0, T.FZ), a), qy(-a)


AWAY_L, AWAY_R = (0.10, 0.30, T.TOP + 0.30), (0.10, -0.30, T.TOP + 0.30)
TRAY_P = np.array([T.TRAY[0], T.TRAY[1], T.TOP + 0.007])
BENCH_P = np.array([0.26, -0.18, T.TOP + T.FUSE_R])


def scene(f2=0.0, f1=0.0, f3=0.0, old=None, old_vel=(0, 0, 0), moved=None, grip_r=None, grip_l=None, open_r=1.0):
    """The scene: carrier openings (deg); the blown fuse seated in F2 unless `old` (pos, quat) says where; `moved`
    {fuse: (pos, quat[, vel])} for spares off their start; grippers away unless placed."""
    o = {}
    for n, deg, y in (("f1", f1, 0.045), ("f2", f2, 0.0), ("f3", f3, -0.045)):
        o[f"holders/{n}"] = st(T.HINGE_W + np.array([0.0, y, 0.0]), qy(-math.radians(deg)))
    o["fuse_old"] = st(*(old if old is not None else seat_pose(math.radians(f2))), vel=old_vel)
    for n in T.SPARES:
        o[n] = st(T.spare_start(n), T.LYING)
    for n, v in (moved or {}).items():
        o[n] = st(*v)
    for b, (x, y) in T.BOXES.items():
        o[b] = st((x, y, T.TOP + T.BOX_H / 2))
    o["tray"] = st((T.TRAY[0], T.TRAY[1], T.TOP + T.TRAY_SIZE[2] / 2))
    o["right_gripper"] = st(grip_r if grip_r is not None else AWAY_R, (1.0, 0.0, 0.0, 0.0))
    o["left_gripper"] = st(grip_l if grip_l is not None else AWAY_L, (1.0, 0.0, 0.0, 0.0))
    T.kit._OPENING["right_gripper"] = open_r
    T.kit._OPENING["left_gripper"] = 1.0
    return o


OPEN_DEG = math.degrees(T.OPEN)
OLD_TRAY = (TRAY_P, qy(math.pi / 2))
NEW_OPEN = {"spare_6a_3": seat_pose(T.OPEN)}
NEW_SHUT = {"spare_6a_3": seat_pose(0.0)}
LIFTED = {"spare_6a_3": (T.spare_start("spare_6a_3") + np.array([0, 0, 0.07]), T.LYING)}


def run(frames):
    T.WATCH._fresh()
    T.WATCH._ep = None
    s = None
    for f in frames:
        s = T.score(f)
    return s


def check(name, got, want):
    ok = abs(got - want) < 1e-9 if isinstance(want, float) else got == want
    print(("ok   " if ok else "FAIL ") + f"{name}: {got} (want {want})")
    return ok


def full_job():
    """A correct swap, frame by frame."""
    return [scene(), scene(f2=35), scene(f2=OPEN_DEG),
            scene(f2=OPEN_DEG, old=(seat_pose(T.OPEN)[0] + 0.03 * T._out_dir(T.OPEN), qy(-T.OPEN))),
            scene(f2=OPEN_DEG, old=OLD_TRAY), scene(f2=OPEN_DEG, old=OLD_TRAY, moved=LIFTED),
            scene(f2=OPEN_DEG, old=OLD_TRAY, moved=NEW_OPEN), scene(f2=30, old=OLD_TRAY, moved={
                "spare_6a_3": seat_pose(math.radians(30))}), scene(f2=0.0, old=OLD_TRAY, moved=NEW_SHUT)]


def main():
    res = []
    T.kit._OPENING.clear()
    # -- the clip and latch laws
    holds = ck.Holds(T.SEAT, T.FUSES, start=("fuse_old",))
    car = (T.HINGE_W, QI)
    poses = {n: (T.spare_start(n), T.LYING) for n in T.SPARES}
    poses["fuse_old"] = seat_pose(0.0)
    f = holds.update(poses, car)
    res.append(check("start: the blown fuse held at both ends, spares not", [len(f[n]) for n in T.FUSES],
                     [2, 0, 0, 0, 0, 0]))
    res.append(check("clip force 0.8 N toward the carrier's floor (-x shut)", [round(float(v), 6) for v in
                                                                              f["fuse_old"][0][1]], [-0.8, 0.0, 0.0]))
    p_open, q_open = seat_pose(T.OPEN)
    car_o = (T.HINGE_W, qy(-T.OPEN))
    poses["fuse_old"] = (p_open + 0.0015 * T._out_dir(T.OPEN), q_open)
    f = holds.update(poses, car_o)
    res.append(check("open carrier, fuse 1.5 mm out: still held", holds.held["fuse_old"], [True, True]))
    poses["fuse_old"] = (p_open + 0.004 * T._out_dir(T.OPEN), q_open)
    holds.update(poses, car_o)
    res.append(check("4 mm out: let go", holds.held["fuse_old"], [False, False]))
    poses["spare_6a_3"] = (p_open + 0.0003 * T._out_dir(T.OPEN), q_open)
    holds.update(poses, car_o)
    res.append(check("a spare laid in (0.3 mm off the floor): caught", holds.held["spare_6a_3"], [True, True]))
    poses["spare_6a_2"] = (p_open + np.array([0.0, 0.0014, 0.0007]), q_open)
    f = holds.update(poses, car_o)
    res.append(check("a spare laid in 1.4 mm off-centre, 0.7 mm up: caught, and pressed down and toward the middle",
                     (holds.held["spare_6a_2"], bool(f["spare_6a_2"][0][1][1] < -0.1), bool(f["spare_6a_2"][0][1][2] < 0)),
                     ([True, True], True, True)))
    res.append(check("but not counted seated until centred (1 mm)", T.seated_in_f2(scene(
        f2=OPEN_DEG, old=OLD_TRAY, moved={"spare_6a_3": (p_open + np.array([0.0, 0.0014, 0.0007]), q_open)}),
        "spare_6a_3"), False))
    poses["spare_6a_3"] = (p_open, tuple(T.kit._quat_mul(q_open, (1.0, 0.0, 0.0, 0.0))))  # turned end for end
    res.append(check("a fuse lying end for end seats the same", T.seated_in_f2(
        scene(f2=OPEN_DEG, old=OLD_TRAY, moved={"spare_6a_3": poses["spare_6a_3"]}), "spare_6a_3"), True))
    snap = T.SNAP
    res.append(check("latch: within 6 deg of shut pulls shut", snap.drive("", -math.radians(3), 0, {}, {})[:2],
                     (0.0, 0.5)))
    res.append(check("free between", snap.drive("", -math.radians(30), 0, {}, {})[1], 0.0))
    res.append(check("detent: near the stop holds open", snap.drive("", -math.radians(86), 0, {}, {})[:2],
                     (-T.OPEN, 0.5)))
    res.append(check("latch holds 0.05 N*m at its zone's edge (about 1 N at the tab)",
                     round(snap.k_latch * snap.zone, 3), 0.052))

    sizes = [c.size for o in T.OBJECTS for c in list(o.colliders) + [c for lk in getattr(o, "links", ()) for c in
                                                                      lk.colliders] if c.size is not None]
    res.append(check("every box collider has a positive size", all(min(z) > 0 for z in sizes), True))

    # -- readouts
    res.append(check("opening read from the link: 35 deg", round(math.degrees(T.opening(scene(f2=35), "f2")), 6),
                     35.0))
    res.append(check("start: blown fuse seated in F2", T.seated_in_f2(scene(), "fuse_old"), True))
    res.append(check("start: spares in their cartons", all(T.in_box(scene(), n) for n in T.SPARES), True))

    # -- scoring: the full job and partial credit
    res.append(check("nothing done: 0", run([scene(), scene()]), 0.0))
    res.append(check("full swap: 1.0", run(full_job()), 1.0))
    res.append(check("F2 opened and left: 0.25", run([scene(), scene(f2=OPEN_DEG)]), 0.25))
    res.append(check("F2 opened then shut again, nothing swapped: 0", run([scene(), scene(f2=OPEN_DEG), scene()]), 0.0))
    res.append(check("blown fuse out, new fuse left in the box: 0.25",
                     run([scene(), scene(f2=OPEN_DEG), scene(f2=OPEN_DEG, old=OLD_TRAY)]), 0.25))
    res.append(check("blown fuse out, new fuse lifted out of its box: 0.5",
                     run([scene(f2=OPEN_DEG), scene(f2=OPEN_DEG, old=OLD_TRAY, moved=LIFTED)]), 0.5))
    dropped = {"spare_6a_3": (BENCH_P, qy(math.pi / 2))}
    res.append(check("new fuse dropped on the bench: 0.5",
                     run([scene(f2=OPEN_DEG), scene(f2=OPEN_DEG, old=OLD_TRAY, moved=dropped)]), 0.5))
    res.append(check("new fuse seated, F2 still open: 0.75",
                     run([scene(f2=OPEN_DEG), scene(f2=OPEN_DEG, old=OLD_TRAY, moved=NEW_OPEN)]), 0.75))
    old_bench = (BENCH_P + np.array([0, -0.03, 0]), qy(math.pi / 2))
    res.append(check("shut on the new fuse, blown fuse on the bench (not the tray): 0.75",
                     run(full_job()[:-1] + [scene(old=old_bench, moved=NEW_SHUT)]), 0.75))
    near = T.HINGE_W + np.array([-0.014, 0.0, 0.053])  # left tool point at the tab, fingers closed on it
    T.kit._OPENING["left_gripper"] = 0.0
    fin = full_job()
    last = scene(old=OLD_TRAY, moved=NEW_SHUT, grip_l=near)
    T.kit._OPENING["left_gripper"] = 0.19
    res.append(check("shut on it but a hand still on F2's carrier: 0.75", run(fin[:-1] + [last]), 0.75))
    rolling = scene(old=(OLD_TRAY[0], OLD_TRAY[1]), old_vel=(0.1, 0, 0), moved=NEW_SHUT)
    res.append(check("blown fuse still rolling in the tray: 0.75", run(fin[:-1] + [rolling]), 0.75))
    res.append(check("success only at 1.0", (T.success(fin[-1]), T.success(rolling)), (True, False)))

    # -- adversarial: hazards give 0 whatever was reached before or after (never a frozen milestone)
    wrong_open = {"spare_10a_2": seat_pose(T.OPEN)}
    res.append(check("10 A fuse seated in F2 (open) for 2 steps, then swapped for a 6 A and finished: 0",
                     run(full_job()[:5] + [scene(f2=OPEN_DEG, old=OLD_TRAY, moved=wrong_open) for _ in range(2)]
                         + full_job()[5:]), 0.0))
    res.append(check("10 A fuse seated and F2 shut on it: 0",
                     run(full_job()[:5] + [scene(f2=OPEN_DEG, old=OLD_TRAY, moved=wrong_open),
                                           scene(old=OLD_TRAY, moved={"spare_10a_2": seat_pose(0.0)})]), 0.0))
    res.append(check("10 A fuse seated for one step only (passing through): not counted",
                     run(full_job()[:5] + [scene(f2=OPEN_DEG, old=OLD_TRAY, moved=wrong_open)] + full_job()[5:]),
                     1.0))
    out_o = (seat_pose(T.OPEN)[0] + 0.03 * T._out_dir(T.OPEN), qy(-T.OPEN))
    res.append(check("blown fuse taken out, put back and F2 shut on it: 0",
                     run([scene(f2=OPEN_DEG), scene(f2=OPEN_DEG, old=out_o), scene(f2=OPEN_DEG), scene()]), 0.0))
    res.append(check("blown fuse put back, then the job done properly: still 0",
                     run([scene(f2=OPEN_DEG), scene(f2=OPEN_DEG, old=out_o), scene(f2=OPEN_DEG),
                          scene(f2=OPEN_DEG)] + full_job()[3:]), 0.0))
    res.append(check("F1 opened 10 deg for 2 steps at the start, then the full swap: 0",
                     run([scene(f1=10), scene(f1=10), scene(f1=0)] + full_job()), 0.0))
    res.append(check("F3 opened after the new fuse is seated (0.75 earned): 0",
                     run(full_job()[:7] + [scene(f2=OPEN_DEG, f3=8, old=OLD_TRAY, moved=NEW_OPEN) for _ in range(2)]
                         + full_job()[7:]), 0.0))
    res.append(check("F3 opened at the very end, after 1.0: 0",
                     run(full_job() + [scene(f3=12, old=OLD_TRAY, moved=NEW_SHUT) for _ in range(2)]), 0.0))
    res.append(check("F1 nudged 3 deg (under 5): no penalty",
                     run([scene(f1=3) for _ in range(2)] + full_job()), 1.0))

    # -- demo: every waypoint reachable by its arm's IK and above the bench
    import yam_arena.demo as yd
    from yam_arena.station import BASES
    plan = T.demo()
    worst, low = 0.0, 1.0
    for side in ("left", "right"):
        for w in plan[side]:
            if isinstance(w, T.kit.Home):
                continue
            rot = yd.grip_rotation(getattr(w, "yaw", 0.0))
            if isinstance(w, T.kit.Tilted):
                rot = T.kit._rot_xyz(w.roll, w.pitch) @ rot
            q, _, _ = yd.ik.solve(np.asarray(w.pos, float), yd.ik.matrix_to_quat_xyzw(rot), q_init=None,
                                  base_pos=np.asarray(BASES[side], float))
            p, _ = yd.ik.tcp_pose(q, np.asarray(BASES[side], float))
            err = float(np.linalg.norm(np.asarray(p) - np.asarray(w.pos)))
            if err > 0.002:
                print(f"     IK {side} '{w.label}' error {err * 1000:.1f} mm")
            worst = max(worst, err)
            low = min(low, w.pos[2] - T.TOP)
    res.append(check("demo: worst IK position error under 2 mm", worst < 0.002, True))
    print(f"     (worst IK error {worst * 1000:.2f} mm, lowest tool point {low * 1000:.1f} mm above the bench)")
    res.append(check("demo: tool point never below 13 mm (fingertips above the bench)", low >= 0.013, True))
    dur = sum(w.duration for w in plan["right"])
    print(f"     demo plan {dur:.1f} s = {dur * 20:.0f} steps")
    T.kit._OPENING.clear()
    print("all pass" if all(res) else f"{res.count(False)} FAIL")
    return all(res)


if __name__ == "__main__":
    sys.exit(0 if main() else 1)
