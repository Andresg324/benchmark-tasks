"""Offline checks of the keg-tapping scorer and demo geometry, no simulator. Each trial step is built by hand: the keg's
and coupler's poses and the handle's joint angle (as the simulator reports them each control step), the line
attachments, the hands where given. Covers: the start layout (nothing overlapping at spawn); the bayonet's geometry
(the lugs pass only through the openings, turn under the flange clockwise to the stops at a locked angle, and the
other way stop short of it); doing nothing; seated (0.25); turned to lock (0.5); handle down (0.75); let go and still
1 s (1.0), still held (0.75); the handle pushed past level before the lock, the keg tipped, the coupler dropped, a line
pulled off (latched 0); the keg turned or slid on the counter (scores the same: everything is in the keg's frame); a
new trial; the demo's poses and its clearances along the whole plan (both hands, the carried coupler and its line
fittings against the keg, tray, wall and counter); the thresholds in the text.

    python3 test_task.py
"""
import json
import math
from pathlib import Path

import numpy as np
from yam_arena import ik
from yam_arena.demo import LEFT_BASE_POS, RIGHT_BASE_POS, ArmPlan, grip_rotation
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import cablekit as ck  # noqa: E402
import guardkit as gk  # noqa: E402
import yamkit as kit  # noqa: E402

AWAY = {"left_gripper": (0.15, 0.32, M.TOP + 0.30), "right_gripper": (0.15, -0.32, M.TOP + 0.30)}  # far from all
KEYS = [key for _, key in M.LINES]
ck.register(M.BEER_LINE, M.GAS_LINE)


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def quat(R):
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        return np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    v = np.zeros(4)
    v[i] = math.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k])) / 2
    v[j] = (R[j, i] + R[i, j]) / (4 * v[i])
    v[k] = (R[k, i] + R[i, k]) / (4 * v[i])
    v[3] = (R[k, j] - R[j, k]) / (4 * v[i])
    return v


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def st(pos, R=np.eye(3), half=0.03):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat(R), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - half, high=p + half)


KEG0 = np.array([M.KEG_C[0], M.KEG_C[1], M.KEG_Z0])


def seat_pose(turn_deg, keg=KEG0, Rk=np.eye(3), lift=0.0):
    """The coupler's pose seated on the valve at a turn (keg frame degrees), `lift` m above seated."""
    Rc = Rk @ rz(math.radians(turn_deg))
    p = np.asarray(keg, float) + Rk @ np.array([0.0, 0.0, M.FLANGE_TOP + lift])
    return p, Rc


def fresh():
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    M._L = None


class Trial:
    def __init__(self):
        self.n = 0

    def objects(self, coupler=None, keg=KEG0, Rk=np.eye(3), q=M.UP_Q, hands=(), lines=("port", "port"), beside=None):
        """One step. coupler: (pos, R) or None (on the tray, as it starts). hands: grippers resting on the coupler."""
        cp, cR = coupler if coupler is not None else (np.array(M.COUPLER_START), np.eye(3))
        o = {"keg": st(keg, Rk, half=0.12), "coupler": st(cp, cR, half=0.05),
             "drip_tray": st((M.TRAY_C[0], M.TRAY_C[1], M.TOP), half=0.06),
             "cooler_wall": st((M.WALL_X + 0.01, 0.15, M.TOP), half=0.1)}
        hp = np.asarray(cp, float) + cR @ np.array(M.PIVOT)
        o["coupler/handle"] = st(hp, cR @ np.array([[math.cos(q), 0, math.sin(q)], [0, 1, 0],
                                                    [-math.sin(q), 0, math.cos(q)]]), half=0.04)
        for g, p in AWAY.items():
            o[g] = st(p, grip_rotation(0.0))
            kit._OPENING[g] = 1.0
        for g in hands:
            o[g] = st(np.asarray(cp, float) + cR @ np.array([0, 0, M.GRIP_UP]), grip_rotation(0.0))
            kit._OPENING[g] = 0.45
        if beside is not None:  # a wide-open hand this far beside the coupler (coupler frame), at its grip height
            o[beside[0]] = st(np.asarray(cp, float) + cR @ np.array([0.0, beside[1], M.GRIP_UP]), grip_rotation(0.0))
            kit._OPENING[beside[0]] = 1.0
        kit._JOINTS["coupler/handle"] = kit.JointState(pos=float(q), vel=0.0, latched=False)
        self.n += 1
        kit._EPISODE["last"] = self.n
        for key, t in zip(KEYS, lines):
            ck.set_attached(key, t)
        return o


def run(trial, steps, **kw):
    s = None
    for _ in range(steps):
        s = M.score(trial.objects(**kw))
    return s


# -- geometry ---------------------------------------------------------------------------------------------------------
def _in_sector_ring(xy, z, r_in, r_out, spans, z0, z1):
    r = math.hypot(*xy)
    a = math.degrees(math.atan2(xy[1], xy[0])) % 360
    return r_in <= r <= r_out and z0 <= z <= z1 and any(lo <= a <= hi or lo <= a + 360 <= hi for lo, hi in spans)


FLANGE_SPANS = [(90 + M.NOTCH_HALF, 270 - M.NOTCH_HALF), (270 + M.NOTCH_HALF, 450 - M.NOTCH_HALF)]
STOP_SPANS = [(d - M.STOP_HALF, d + M.STOP_HALF) for d in M.STOP_DEG]


def lug_points(turn_deg, dz):
    """Sample points of both lugs (keg frame) with the coupler turned turn_deg and its roof dz above the flange top."""
    pts = []
    for s in (-1, 1):
        for r in np.linspace(M.LUG_IN, M.SKIRT_IN, 5):
            for t in np.linspace(-M.LUG_W / 2, M.LUG_W / 2, 5):
                for z in np.linspace(*M.LUG_Z, 3):
                    p = rz(math.radians(turn_deg)) @ np.array([s * r, t, z])
                    pts.append((p[:2], M.FLANGE_TOP + dz + z))
    return pts


def lug_hits(turn_deg, dz, spans, z0, z1, r_in, r_out):
    return any(_in_sector_ring(xy, z, r_in, r_out, spans, z0, z1) for xy, z in lug_points(turn_deg, dz))


def flange_hit(turn, dz):
    return lug_hits(turn, dz, FLANGE_SPANS, *M.FLANGE_Z, M.FLANGE_IN, M.FLANGE_OUT)


def stop_hit(turn):
    return lug_hits(turn, 0.0, STOP_SPANS, M.GROOVE[0] + 0.001, M.GROOVE[1], M.CORE_R - 0.002, M.FLANGE_OUT)


def test_bayonet():
    fresh()
    check(not any(flange_hit(90.0, dz) or flange_hit(-90.0, dz) for dz in np.linspace(0.0, 0.025, 26)),
          "lugs pass down through the openings at every height")
    for turn in (70.0, 45.0, 0.0, 135.0):
        check(flange_hit(turn, 0.012), f"turned {turn:.0f} deg off the openings, the lugs land on the flange")
    # clockwise from +90: free down to the stop; the angle where a stop is first met
    a = 90.0
    while a > -60 and not stop_hit(a - 0.5):
        a -= 0.5
    check(abs(a) <= M.LOCK_DEG and abs(a) < 3.0, f"clockwise turn ends at the stops at {a:.1f} deg (locked within 3)")
    check(not any(flange_hit(t, 0.0) for t in np.arange(90.0, a, -1.0)), "lugs under the flange all the way round")
    b = 90.0
    while b < 200 and not stop_hit(b + 0.5):
        b += 0.5
    check(min(abs(b), 180 - abs(b)) > M.LOCK_DEG + 10, f"the wrong way it stops at {b:.1f} deg, not locked")
    check(M.LUG_IN - math.sqrt(2) * 0.7 * M.CORE_R >= 0.002, "lugs clear the valve body by 2 mm or more")
    check(M.SKIRT_IN - M.FLANGE_OUT >= 0.0029, "skirt 3 mm clear of the flange")
    w = 2 * (M.FLANGE_IN + M.FLANGE_OUT) / 2 * math.sin(math.radians(M.NOTCH_HALF))
    check(w - M.LUG_W >= 0.005, f"opening {w * 1000:.1f} mm for a {M.LUG_W * 1000:.0f} mm lug")


def carried_poses():
    """The coupler's planned poses while the left hand holds it (from the demo plan's own targets)."""
    lw = M.coupler_plan()
    plan = ArmPlan(lw, np.asarray(RIGHT_BASE_POS))
    t0 = sum(w.duration for w in lw[:3])
    t1 = t0 + sum(w.duration for w in lw[3:11])
    g0 = grip_rotation(0.0)
    out = []
    for t in np.arange(t0, t1, 0.05):
        pos, rot, _, _ = plan.target(t)
        out.append((np.asarray(pos) - np.array([0.0, 0.0, M.GRIP_UP]), rot @ g0.T))
    return out


def test_layout():
    fresh()
    o = Trial().objects()
    check(M.score(o) == 0.0, "nothing done: 0")
    check(abs(2 * M.KEG_R - 9.25 * 0.0254) < 0.001, "keg 9-1/4 in across (sixtel)")
    check(M.KEG_KG == 7.5, "keg at its 16.5 lb empty mass")
    check(abs(M.COUPLER_START[2] - M.SKIRT_H - M.TRAY_TOP - 0.001) < 1e-9, "coupler 1 mm above the tray at spawn")
    check(abs(M.KEG_Z0 - M.TOP - 0.001) < 1e-9 and abs(M.KEG.pos[2] - M.KEG_Z0) < 1e-9, "keg 1 mm above the counter at spawn")
    sk_lo = np.array(M.COUPLER_START[:2]) - (M.SKIRT_IN + M.SKIRT_T)
    sk_hi = np.array(M.COUPLER_START[:2]) + (M.SKIRT_IN + M.SKIRT_T)
    tr_lo = np.array(M.TRAY_C) - np.array(M.TRAY_SIZE[:2]) / 2
    tr_hi = np.array(M.TRAY_C) + np.array(M.TRAY_SIZE[:2]) / 2
    check(np.all(sk_lo > tr_lo) and np.all(sk_hi < tr_hi), "coupler stands inside the drip tray")
    d = math.hypot(M.TRAY_C[0] + 0.06 - M.KEG_C[0], M.TRAY_C[1] - 0.06 - M.KEG_C[1])
    check(math.hypot(M.TRAY_C[0] - M.KEG_C[0], M.TRAY_C[1] - M.KEG_C[1]) - M.KEG_R - 0.085 > 0.02,
          f"tray clear of the keg ({d:.3f})")
    for cab, port in ((M.BEER_LINE, M.BEER_PORT), (M.GAS_LINE, M.GAS_PORT)):
        n = cab.nodes()
        L = cab.segments * cab.seg_len
        jaw = n[-1] + M.FIT_L * (n[-1] - n[-2]) / np.linalg.norm(n[-1] - n[-2])
        check(np.linalg.norm(jaw - np.array(M._start_port(port))) < 1e-3, f"{cab.name}: fitting on its port at spawn")
        check(np.min(n[:, 2]) >= M.TOP + cab.radius - 1e-6, f"{cab.name}: never below the counter")
        far = max(np.linalg.norm(np.asarray(cab.path[0]) - c - R @ np.asarray(port)) for c, R in carried_poses())
        check(L >= 1.4 * far, f"{cab.name}: {L:.2f} m, slack for the carry (furthest {far:.2f} m)")
        r = np.hypot(n[:, 0] - M.KEG_C[0], n[:, 1] - M.KEG_C[1])
        check(np.min(r) > M.KEG_R + cab.radius + 0.01, f"{cab.name}: clear of the keg at spawn")
        check(np.max(n[:, 0]) < M.WALL_X - cab.radius, f"{cab.name}: in front of the wall")
    for cab in (M.BEER_LINE, M.GAS_LINE):  # the explicit attachment springs inside their stability limits at 240 Hz
        f, dt = cab.end, 1.0 / 240
        roll_i = f.mass * (f.size[1] ** 2 + f.size[2] ** 2) / 12
        m = f.mass + cab.cable_mass / cab.segments
        ok = (f.turn_stiffness * dt * dt / roll_i < 1.0 and f.turn_damping * dt / roll_i < 1.5
              and math.sqrt(f.stiffness / m) * dt < 1.0)
        check(ok, f"{cab.name}: attachment springs stable at 240 Hz")
    import test_clearance as clearance

    a, b = clearance.LINE_PTS[0][1], clearance.LINE_PTS[1][1]
    gap = float(np.min(np.linalg.norm(a[:, None] - b[None], axis=2))) - M.BEER_R - M.GAS_R
    check(gap >= 0.005, f"the two lines {gap * 1000:.0f} mm apart at spawn")
    f = M.HANDLE_PUSH / (M.PIVOT[0] - M.PRESS_X)
    check(f < 52.0, f"handle spring {f:.0f} N at the demo's press point, within the YAM's 52 N push down")
    check(M.HANDLE_PUSH / 0.06 < 52.0, "handle push within the YAM's 52 N continuous push down")


def test_ladder():
    fresh()
    t = Trial()
    check(run(t, 3, coupler=seat_pose(90.0), hands=("right_gripper",)) == 0.25, "seated, lugs through: 0.25")
    check(run(t, 3, coupler=seat_pose(90.0, lift=0.018), hands=("right_gripper",)) == 0.0,
          "resting on the flange (lugs on it): 0")
    check(run(t, 3, coupler=seat_pose(45.0), hands=("right_gripper",)) == 0.25, "half turned: 0.25")
    check(run(t, 3, coupler=seat_pose(-1.0), hands=("right_gripper",)) == 0.5, "turned to the stops: 0.5")
    check(run(t, 3, coupler=seat_pose(-1.0, lift=0.002), hands=("right_gripper",)) == 0.5, "risen 2 mm against the flange: still locked")
    check(run(t, 3, coupler=seat_pose(-1.0), q=M.DOWN_Q, hands=("right_gripper",)) == 0.75, "handle down, held: 0.75")
    check(run(t, 10, coupler=seat_pose(-1.0), q=M.DOWN_Q) == 0.75, "let go, half a second: 0.75")
    check(run(t, 15, coupler=seat_pose(-1.0), q=M.DOWN_Q) == 1.0 and M.success(t.objects(
        coupler=seat_pose(-1.0), q=M.DOWN_Q)), "let go and still 1 s: 1.0, success")
    fresh()
    t = Trial()
    run(t, 3, coupler=seat_pose(-89.0))
    check(run(t, 25, coupler=seat_pose(-179.0), q=M.DOWN_Q) == 1.0, "the other locked position (handle away): 1.0")


def test_keg_moves():
    fresh()
    t = Trial()
    keg = KEG0 + np.array([0.03, -0.02, 0.0])
    Rk = rz(math.radians(25.0))
    check(run(t, 3, coupler=seat_pose(90.0, keg, Rk), keg=keg, Rk=Rk) == 0.25, "keg slid and turned: seated read in its frame")
    s = run(t, 25, coupler=seat_pose(0.0, keg, Rk), keg=keg, Rk=Rk, q=M.DOWN_Q)
    check(s == 1.0, "keg slid and turned, coupler locked on it: 1.0")
    fresh()
    t = Trial()
    Rk = rz(math.radians(-90.0))
    check(run(t, 3, coupler=seat_pose(90.0, KEG0, Rk), Rk=Rk) == 0.25,
          "keg turned along with the coupler (relative turn 0): still only seated")


def test_faults():
    fresh()
    t = Trial()
    check(run(t, 3, coupler=seat_pose(90.0), q=math.radians(-10.0), hands=("right_gripper",)) == 0.0,
          "handle past level before the turn: 0")
    check(run(t, 30, coupler=seat_pose(-1.0), q=M.DOWN_Q) == 0.0, "... latched")
    fresh()
    t = Trial()
    check(run(t, 3, q=math.radians(-10.0)) == 0.0, "handle pressed on the tray: 0")
    fresh()
    t = Trial()
    check(run(t, 1, coupler=seat_pose(90.0), q=math.radians(-10.0)) == 0.0, "one step past level (a blip): latched 0")
    fresh()
    t = Trial()
    tip = rx(math.radians(15.0))
    run(t, 3, keg=KEG0, Rk=tip)
    check(run(t, 30, coupler=seat_pose(-1.0), q=M.DOWN_Q) == 0.0, "keg tipped 15 deg: latched 0")
    fresh()
    t = Trial()
    run(t, 3, coupler=(np.array([0.20, 0.10, M.TOP + 0.03]), rx(math.radians(80.0))))
    check(run(t, 30, coupler=seat_pose(-1.0), q=M.DOWN_Q) == 0.0, "coupler knocked over: latched 0")
    fresh()
    t = Trial()
    run(t, 3, coupler=seat_pose(-1.0), lines=("port", None))
    check(run(t, 30, coupler=seat_pose(-1.0), q=M.DOWN_Q) == 0.0, "gas line pulled off: latched 0")
    fresh()
    t = Trial()
    check(run(t, 30, coupler=seat_pose(-1.0), q=M.DOWN_Q) == 1.0, "new trial: latches cleared")


def test_handle_kept_held():
    fresh()
    t = Trial()
    run(t, 3, coupler=seat_pose(-1.0), q=M.DOWN_Q)
    s = run(t, 30, coupler=seat_pose(-1.0), q=M.DOWN_Q, hands=("right_gripper",))
    check(s == 0.75, "hand still on the coupler: 0.75")
    fresh()
    t = Trial()
    run(t, 3, coupler=seat_pose(-1.0), q=M.DOWN_Q)
    o = t.objects(coupler=seat_pose(-1.0), q=M.DOWN_Q, beside=("right_gripper", 0.050))
    tips = min(gk.dist(o, "coupler", p, M.COUPLER_PARTS) for p in gk.tool_points(o, "right_gripper"))
    check(tips > M.CLEAR and gk.hand_gap(o, "right_gripper", "coupler", M.COUPLER_PARTS) <= 0.0,
          "open hand beside the coupler: tool point and pad tip clear, a finger on it")
    s = run(t, 30, coupler=seat_pose(-1.0), q=M.DOWN_Q, beside=("right_gripper", 0.050))
    check(s == 0.75, "open hand resting a finger on the locked coupler: not let go, 0.75")


def test_text():
    j = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    check(j["instruction"] == M.INSTRUCTION and M.INSTRUCTION in readme, "instruction matches")
    for txt in (j["scoring"], readme):
        for needle in ("4 mm", "10 degrees", "45 degrees", "1 cm", "1 s", "0.25", "0.5", "0.75", "1.0"):
            check(needle in txt, f"'{needle}' in the text")
    check(f"{M.RUN_LIMIT_STEPS:,}" in j["run_limit"], "run limit matches")


if __name__ == "__main__":
    test_bayonet()
    test_layout()
    test_ladder()
    test_keg_moves()
    test_faults()
    test_handle_kept_held()
    import test_clearance as clearance  # the demo's clearance sweep, in its own file

    clearance.main()
    test_text()
    print("all ok")
