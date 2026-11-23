"""Offline checks of the shampoo-dose scorer, no simulator. Bottle and dish states are built by hand from the geometry
the demo uses; the bottle's wall pushes and the lid's angle are set as its joint states. Covers: doing nothing (the start
state); the demo's dose replayed from its own hand plan (1.0 once the lid is clicked, everything stands and is still for
1 s); each cheat (cap left open, lid left on its catch, lid held down by the hand, squeezing with the cap shut, a dumped
dose, nothing); no squeeze or a light one (nothing comes out at any tilt); too little and too much; drips outside the dish; the dish knocked over; the bottle not set down; a
new trial; the lid's detent; the model's USD; the text's thresholds.

    python3 test_task.py
"""
import json
import math
import os
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import yamkit as kit  # noqa: E402


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=M.sq.quat(np.asarray(rot)), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


_CACHE = {}


def _fk(robot, q6, finger):
    import yam_arena.demo as demo
    from yam_arena.analysis import fk_link_poses_world

    base = demo.LEFT_BASE_POS if robot == "left_robot" else demo.RIGHT_BASE_POS
    q = {f"joint{i + 1}": float(q6[i]) for i in range(6)}
    q["joint7"] = q["joint8"] = float(finger)
    poses = fk_link_poses_world(q, base)
    return {f"{robot}/{k}": (P[:3, 3].copy(), P[:3, :3].copy()) for k, P in poses.items() if k in M.ARM_LINKS}


def ready_links():
    if "ready" not in _CACHE:
        import yam_arena.demo as demo

        out = {}
        for r in M.ROBOTS:
            out.update(_fk(r, demo.READY, M.FINGER_OPEN))
        _CACHE["ready"] = out
    return _CACHE["ready"]


def gripping_links():
    """The left hand holding the standing bottle as the demo does; the right at the ready pose."""
    if "grip" not in _CACHE:
        import yam_arena.demo as demo

        q, pe, _ = demo.ik.solve(M.LEFT_AT, demo.ik.matrix_to_quat_xyzw(M.R_LEFT), np.array(demo.READY, float),
                                 base_pos=demo.LEFT_BASE_POS, iters=400)
        check(pe < 0.002, f"IK reaches the left's grip ({pe * 1000:.1f} mm)")
        out = _fk("left_robot", q, M.bottle_grip(M.HOLD_PUSH))
        out.update({k: v for k, v in ready_links().items() if k.startswith("right_robot")})
        _CACHE["grip"] = out
    return _CACHE["grip"]


def lid_links():
    """The right's fingertips resting on the lid's top (left at the ready pose)."""
    if "lid" not in _CACHE:
        import yam_arena.demo as demo

        tcp = np.array([M.BX + M.PUSH_DX, M.BY - 0.010, M.TOP + M.HEIGHT + M.TIP_HALF + 0.001])
        q, pe, _ = demo.ik.solve(tcp, demo.ik.matrix_to_quat_xyzw(M.R_FWD_V), np.array(demo.READY, float),
                                 base_pos=demo.RIGHT_BASE_POS, iters=400)
        check(pe < 0.002, f"IK reaches the lid's top ({pe * 1000:.1f} mm)")
        out = _fk("right_robot", q, M.FINGER_CLOSED)
        out.update({k: v for k, v in ready_links().items() if k.startswith("left_robot")})
        _CACHE["lid"] = out
    return _CACHE["lid"]


class Trial:
    def __init__(self):
        kit._EPISODE["count"] = kit._EPISODE.get("count", 0) + 1
        kit._EPISODE["last"] = 0
        M._L = None
        self.n = 0
        self.bottle = (np.array(M.BOTTLE_START, float) - np.array([0, 0, M.SPAWN_GAP]), np.eye(3))
        self.dish = (np.array(M.DISH_START, float) - np.array([0, 0, M.SPAWN_GAP]), np.eye(3))
        self.lid = 0.0  # opening angle, rad
        self.push = 0.0
        self.links = ready_links()

    def objects(self):
        for link, sign in M.PANEL_SIGN.items():
            kit._JOINTS[f"bottle/{link}"] = kit.JointState(sign * self.push, 0.0, False)
        kit._JOINTS["bottle/lid"] = kit.JointState(-self.lid, 0.0, False)
        bp, br = self.bottle
        lp = bp + br @ np.asarray(M.HINGE)
        return {"bottle": st(bp, br), "dish": st(*self.dish), "bottle/lid": st(lp, br @ rx(-self.lid))}

    def step(self, n=1, links=None):
        M.LINK_POSES.clear()
        M.LINK_POSES.update(self.links if links is None else links)
        s = None
        for _ in range(n):
            self.n += 1
            kit._EPISODE["last"] = self.n
            o = self.objects()
            M.success(o)
            s = M.score(o)
        return s

    def hold_at(self, tcp, rot):
        """The bottle where the left hand at tool point `tcp`, rotation `rot` (world) holds it."""
        rb = np.asarray(rot) @ M.R_LEFT.T
        self.bottle = (np.asarray(tcp) - rb @ M._GRIP_IN_BOTTLE, rb)

    def pour(self, push, hold_s, settle_s=M.SETTLE_S + 0.5, tail_s=M.TAIL_S, where=None, deg=M.POUR_DEG):
        """Roll the bottle cap-down at the pour pose (or `where`), wait, squeeze `push` a side for `hold_s`, ease back
        to the holding grip, then stand it back on its mark."""
        r = rx(math.radians(deg)) @ M.R_LEFT
        tcp = M.pour_tcp() if where is None else np.asarray(where)
        self.push = WALL_HOLD
        for k in range(1, 11):  # the roll, over 0.5 s
            self.hold_at(tcp, rx(math.radians(deg * k / 10)) @ M.R_LEFT)
            self.step()
        self.hold_at(tcp, r)
        self.step(round(settle_s / M.DT))
        self.push = push
        self.step(round(hold_s / M.DT))
        self.push = WALL_HOLD
        self.step(round(tail_s / M.DT))
        for k in range(9, -1, -1):
            self.hold_at(tcp, rx(math.radians(deg * k / 10)) @ M.R_LEFT)
            self.step()
        self.bottle = (np.array(M.BOTTLE_START, float) - np.array([0, 0, M.SPAWN_GAP]), np.eye(3))
        self.push = 0.0
        return self.step()

    @property
    def dish_g(self):
        return M._L.shampoo.dish_g()

    @property
    def off_g(self):
        return M._L.shampoo.off_g()


# The walls' push in the simulator falls short of the hand's command (the gripper gives as the pressure rises). These
# are the pushes the demo's probe measured (mean of the two walls, SHAMPOO_PROBE trace: 2.0 + 3.4 and 5.2 + 6.5 mm), used for the offline
# squeezes below: the holding grip, and the dosing squeeze held at its steady value.
WALL_HOLD = 0.0027
WALL_DOSE = 0.00585


def wall(cmd):
    """The wall push (m) for a hand command `cmd` (m a side), scaled between the two measured points."""
    return WALL_HOLD + (cmd - M.HOLD_PUSH) * (WALL_DOSE - WALL_HOLD) / (M.DOSE_PUSH - M.HOLD_PUSH)


def dosed():
    """The demo's squeeze with the lid open; returns the trial with the bottle back on its mark."""
    t = Trial()
    t.lid = M.OPEN_A
    t.step(10)
    t.pour(WALL_DOSE, M.DOSE_S)
    check(M.DOSE_G[0] <= t.dish_g <= M.DOSE_G[1], f"the demo's squeeze puts a coin-sized dose in the dish "
                                                   f"({t.dish_g:.2f} g)")
    check(t.off_g <= M.OFF_G, f"nothing outside the dish ({t.off_g:.2f} g)")
    return t


def test_nothing():
    t = Trial()
    check(t.step(80) == 0.0, "doing nothing (the start state: cap shut, dish empty) scores 0")


def test_demo_replay():
    """The demo's own left-hand plan, replayed through the flow model (the wall push as commanded)."""
    import yam_arena.demo as demo

    plan = demo.ArmPlan(M.demo()["left"], demo.LEFT_BASE_POS)
    t = Trial()
    t.lid = M.OPEN_A
    held = False
    for k in range(int(plan.duration / M.DT)):
        pos, rot, finger, label = plan.target(k * M.DT)
        half = (finger - M.FINGER_CLOSED) / (M.FINGER_OPEN - M.FINGER_CLOSED) * M.FINGER_HALF_OPEN
        t.push = wall(max(0.0, M.D / 2 - half)) if M.D / 2 - half > 0 else 0.0
        held = held or t.push > 0
        if held and label not in ("let go", "back out", "home"):
            t.hold_at(pos, rot)
        else:
            t.bottle = (np.array(M.BOTTLE_START, float) - np.array([0, 0, M.SPAWN_GAP]), np.eye(3))
        if label == "back out":  # the right shuts the lid once the bottle stands on its mark
            t.lid = 0.0
        t.step()
    check(M.DOSE_G[0] <= t.dish_g <= M.DOSE_G[1], f"the demo's plan doses {t.dish_g:.2f} g into the dish")
    check(t.off_g <= M.OFF_G, f"the demo's plan drips {t.off_g:.2f} g outside the dish")
    check(abs(M._L.shampoo.balance()) < 1e-9, "every drop is accounted for")
    check(t.step(30) == 1.0, "dosed, cap clicked, all standing and still for 1 s: 1.0")


def test_finish():
    t = dosed()
    t.lid = 0.0
    check(t.step(5, links=gripping_links()) == 0.5, "cap clicked but the bottle still held: 0.5")
    check(t.step(10) == 0.5, "let go, still for 0.5 s: 0.5")
    check(t.step(15) == 1.0, "still for 1 s: 1.0")


def test_cap_left_open():
    t = dosed()
    t.lid = M.OPEN_A
    check(t.step(60) == 0.5, "cap left open: 0.5")


def test_lid_on_catch():
    t = dosed()
    t.lid = M.BEAD_A
    check(t.step(60) == 0.5, "lid pushed back but resting on its catch (not clicked): 0.5")
    t.lid = M.SHUT_A + 0.01
    check(t.step(60) == 0.5, "lid just short of the click: 0.5")


def test_lid_held_by_hand():
    t = dosed()
    t.lid = 0.04  # pressed under the catch by the hand, which stays on it
    check(t.step(60, links=lid_links()) == 0.5, "lid held down by the hand at the end: 0.5")
    check(M.arm_gap(t.objects()) <= M.CLEAR, "the hand on the lid counts as touching (per-hull gap)")


def test_squeeze_cap_shut():
    t = Trial()
    t.step(10)
    t.pour(WALL_DOSE, 3.0)
    check(M._L.shampoo.dispensed == 0.0, "squeezing with the cap shut dispenses nothing")
    check(t.step(40) == 0.0, "squeezed with the cap shut: 0")
    t.lid = M.OPEN_A
    check(t.step(40) == 0.0, "opened afterwards, upright: still nothing in the dish: 0")


def test_pressure_sealed():
    t = Trial()
    t.hold_at(M.pour_tcp(), rx(math.radians(M.POUR_DEG)) @ M.R_LEFT)
    t.push = WALL_DOSE
    t.step(40)
    check(M._L.shampoo.p > 1.0, f"a squeeze on the shut bottle raises the pressure inside ({M._L.shampoo.p:.1f} kPa)")


def test_dumped():
    t = Trial()
    t.lid = M.OPEN_A
    t.pour(M.PANEL_TRAVEL, M.DUMP_S)
    check(t.dish_g > M.HIGH_G, f"the walls pushed all the way in for {M.DUMP_S:.0f} s dump {t.dish_g:.1f} g")
    t.lid = 0.0
    check(t.step(40) == 0.0, "dumped dose, cap shut, all still: 0")


def test_too_little_too_much():
    t = Trial()
    t.lid = M.OPEN_A
    t.pour(wall(M.WEAK_PUSH), M.WEAK_S)
    t.lid = 0.0
    s = t.step(40)
    check(M._L.shampoo.dispensed == 0.0 and s == 0.0, f"a light squeeze (the 'weak' variant's {M.WEAK_PUSH * 1000:.1f} mm "
                                                      f"command, {M.WEAK_S:.0f} s) stays under the opening pressure: "
                                                      f"nothing comes out: {s}")
    t = Trial()
    t.lid = M.OPEN_A
    t.pour(WALL_DOSE, 0.6)
    g = t.dish_g
    t.lid = 0.0
    s = t.step(40)
    check(M.LOW_G <= g < M.DOSE_G[0] and s == 0.25, f"the demo's squeeze let go after 0.6 s: {g:.2f} g, too little: {s}")
    t = Trial()
    t.lid = M.OPEN_A
    t.pour(WALL_DOSE + 0.0015, 4.0)
    g = t.dish_g
    t.lid = 0.0
    s = t.step(40)
    check(M.DOSE_G[1] < g <= M.HIGH_G and s == 0.25, f"a harder, longer squeeze: {g:.2f} g, too much: {s}")


def test_drips_outside():
    t2 = Trial()
    t2.lid = M.OPEN_A
    t2.pour(WALL_DOSE, M.DOSE_S, where=M.pour_tcp() + np.array([0.0, -0.060, 0.0]))
    t2.lid = 0.0
    check(t2.dish_g == 0.0 and t2.off_g > M.OFF_G and t2.step(40) == 0.0, "squeezed beside the dish: all on the counter: 0")
    t3 = Trial()
    t3.lid = M.OPEN_A
    t3.pour(WALL_DOSE, M.DOSE_S * 0.5)
    t3.pour(WALL_DOSE, M.DOSE_S * 0.5, where=M.pour_tcp() + np.array([0.0, -0.060, 0.0]))
    t3.lid = 0.0
    s = t3.step(40)
    check(t3.off_g > M.OFF_G and s <= 0.25, f"half in the dish ({t3.dish_g:.2f} g), half beside it "
                                            f"({t3.off_g:.2f} g): {s}")


def test_dish_knocked_over():
    t = dosed()
    t.dish = (t.dish[0] + np.array([0, 0, 0.02]), rx(math.radians(80)))
    t.step(2)
    t.dish = (np.array(M.DISH_START, float), np.eye(3))
    t.lid = 0.0
    check(t.dish_g == 0.0 and t.step(40) == 0.0, "the dish knocked over after dosing: spilled: 0")


def test_bottle_not_standing():
    t = dosed()
    t.lid = 0.0
    t.bottle = (t.bottle[0] + np.array([0.0, 0.0, 0.03]), rx(math.radians(90)))
    check(t.step(40) == 0.5, "cap clicked but the bottle left lying on its side: 0.5")


def test_new_trial():
    dosed()
    t = Trial()
    check(t.step(5) == 0.0 and M._L.shampoo.dish_ml == 0.0, "a new trial starts with an empty dish")


def test_detent():
    law = M.SnapLid()
    mem = {}

    def torque(a):
        target, k, eff, _ = law.drive("bottle/lid", -a, 0.0, {}, mem)
        return k * (-target - a) - eff  # the torque opening the lid (the joint's own torque is k * (target - q))

    check(torque(0.03) < 0 and abs(torque(M.SHUT_A - 1e-6)) >= 0.049, "shut: held, 0.05 N*m to pull it past the catch")
    check(torque(M.SHUT_A + 1e-6) > 0.039, "just past the catch: pushed open (0.04 N*m to press it back)")
    check(abs(torque(M.BEAD_A)) < 1e-9, "resting on the catch at 11.5 degrees")
    check(torque(1.0) < 0 and torque(1.8) > 0, "the living hinge: below upright it falls back, above it opens")
    check(abs(torque(M.OPEN_A)) < 1e-9, "the open lid rests at its 115 degree stop")
    tab = M.HOLD_K * M.SHUT_A / M.LID_REACH
    check(1.0 <= tab <= 2.0, f"opening takes {tab:.1f} N at the thumb tab")


def test_model_files():
    text = kit._mech_usda(M.BOTTLE)
    check('def Xform "Root"\n{' in text and "PhysicsArticulationRootAPI" in text.split('def Xform "base"')[1][:400],
          "the bottle's articulation root sits on its base body (squeezekit)")
    for o in M.OBJECTS:
        check(len(o.meshes) > 0, f"{o.name} has a visible mesh")
    for c in M.DISH.colliders:
        if c.hull is not None:
            p = np.array(c.hull)
            ext = p.max(0) - p.min(0)
            check(min(ext) >= 0.003, "every hull collider is at least 3 mm thick")
            break


def test_variants():
    for v in ("", "open", "dump", "closed", "held", "nothing", "wait", "weak"):
        M.DEMO_VARIANT = v
        d = M.demo()
        check(len(d["left"]) > 0 and len(d["right"]) > 0, f"demo variant {v or 'full'} builds")
    M.DEMO_VARIANT = ""
    import yam_arena.demo as demo

    for v in ("", "wait"):
        M.DEMO_VARIANT = v
        T = max(demo.ArmPlan(M.demo()[s], demo.LEFT_BASE_POS).duration for s in ("left", "right"))
        check(T / M.DT < M.RUN_LIMIT_STEPS, f"the {v or 'demo'} plan ({T:.1f} s) fits the run limit")
    M.DEMO_VARIANT = ""


def _cap_down(t, deg, secs):
    """Hold the open bottle at `deg` from upright, orifice 6 cm over the dish's floor, walls unpushed, for `secs`."""
    r = rx(math.radians(deg))
    want = np.array([M.DISH_XY[0], M.DISH_XY[1], M.TOP + M.DISH_FLOOR + 0.060])
    t.bottle = (want - r @ np.asarray(M.ORIFICE), r)
    t.push = 0.0
    t.step(round(secs / M.DT))


def _stand(t):
    t.bottle = (np.array(M.BOTTLE_START, float) - np.array([0, 0, M.SPAWN_GAP]), np.eye(3))
    t.push = 0.0
    return t.step()


def test_zero_squeeze():
    """No squeeze at all: the shampoo's weight alone stays under the orifice's opening pressure at every tilt, so the
    open bottle held cap-down gives nothing, however long, and the "wait" variant scores 0. Also no slow leak from the
    head-space vacuum: nothing at all leaves."""
    head = max(M.Shampoo().head(math.cos(math.radians(d))) for d in range(0, 71))
    check(head < M.VALVE_KPA, f"the shampoo's weight gives at most {head:.2f} kPa, under the {M.VALVE_KPA} kPa opening")
    for deg in (110.0, 118.0, 135.0, 150.0, 180.0):
        t = Trial()
        t.lid = M.OPEN_A
        _cap_down(t, deg, 300.0)
        check(M._L.shampoo.dispensed == 0.0, f"open, {deg:.0f} degrees from upright, unsqueezed for 5 minutes: nothing")
    t = Trial()
    t.lid = M.OPEN_A
    t.step(10)
    t.pour(WALL_HOLD, 0.0, settle_s=M.SETTLE_S + 0.5 + M.WAIT_LONG_S, tail_s=0.0)
    t.lid = 0.0
    s = t.step(40)
    check(t.dish_g == 0.0 and s == 0.0, f"wait: {M.WAIT_LONG_S:.0f} s cap-down at the demo's tilt on the holding grip, "
                                        f"no squeeze, then shut: {t.dish_g:.2f} g: {s}")
    t = Trial()
    t.lid = M.OPEN_A
    _cap_down(t, 180.0, 5.0)
    t.push = WALL_DOSE  # squeezed after it has stood cap-down: the stored vacuum is not a dose either
    t.step(round(M.DOSE_S / M.DT))
    t.push = 0.0
    t.step(round(60 / M.DT))
    g = t.dish_g
    check(M.LOW_G < g and M._L.shampoo.q_now == 0.0, f"straight down, squeezed {M.DOSE_S} s then let go: {g:.2f} g, "
                                                     f"then it stops (no drip after the squeeze)")


def test_text():
    j = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    check(j["instruction"] == M.INSTRUCTION, "task.json instruction == INSTRUCTION")
    check(M.INSTRUCTION in readme, "README carries the instruction")
    for what, val in (("dose", "1.5 to 4.0 g"), ("off", "0.3 g"), ("low", "0.5 g"), ("high", "15 g")):
        check(val in j["scoring"] and val in readme, f"the {what} threshold ({val}) is in task.json and README")
    check("2,400" in j["run_limit"], "run limit 2,400 steps")


def test_lid_on_catch_squeeze():
    """The tab lifted just past the catch (the lid springs onto it at 11.5 degrees), then squeezed over the dish."""
    t = Trial()
    t.lid = M.BEAD_A
    t.step(10)
    t.pour(WALL_DOSE, M.DOSE_S)
    t.lid = 0.0
    s = t.step(40)
    check(t.dish_g == 0.0 and t.off_g > M.OFF_G and s <= 0.25,
          f"squeezed with the lid on its catch: it lies over the orifice, {t.off_g:.2f} g onto the cap, none in the "
          f"dish: {s}")


def test_lid_3deg_squeeze():
    t = Trial()
    t.lid = math.radians(3.5)
    t.step(10)
    t.pour(WALL_DOSE, M.DOSE_S)
    t.lid = 0.0
    s = t.step(40)
    check(t.dish_g == 0.0 and s <= 0.25, f"squeezed with the lid 3.5 degrees open: none in the dish: {s}")


def test_lid_clear_angle():
    check(abs(math.degrees(M.LID_CLEAR_A) - 60.85) < 0.1, "the lid clears the orifice's line at 61 degrees open")
    t = Trial()
    t.lid = math.radians(55.0)
    t.pour(WALL_DOSE, M.DOSE_S)
    check(t.dish_g == 0.0, "lid 55 degrees open: still over the orifice, nothing in the dish")
    t = Trial()
    t.lid = math.radians(70.0)
    t.pour(WALL_DOSE, M.DOSE_S)
    check(M.DOSE_G[0] <= t.dish_g <= M.DOSE_G[1], f"lid 70 degrees open: clear of the stream ({t.dish_g:.2f} g)")


def test_missing_joints_raise():
    t = Trial()
    o = t.objects()
    del kit._JOINTS["bottle/lid"]
    try:
        M.lid_open(o)
        ok = False
    except RuntimeError:
        ok = True
    check(ok, "an unreadable lid joint stops the trial (RuntimeError), it is not read as shut")
    t.objects()
    del kit._JOINTS["bottle/wall_l"]
    try:
        M.pushes(o)
        ok = False
    except RuntimeError:
        ok = True
    check(ok, "an unreadable wall joint stops the trial (RuntimeError)")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    only = sys.argv[1:]
    for f in tests:
        if only and f.__name__ not in only:
            continue
        print(f"-- {f.__name__}")
        f()
    print("all passed")
