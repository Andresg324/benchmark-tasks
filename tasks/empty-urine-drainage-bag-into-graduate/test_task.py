"""Offline checks of the drainage-bag scorer, the drain model and the demo geometry, no simulator. Link states come from
the task's own forward kinematics (fk) for chosen joint angles; joint angles are set as joint states, as the simulator
does each control step. Covers: doing nothing; the start layout (no flow from the stowed spout even with the valve
open, the graduate clear of the drain spot); a full drain (0.75, then 1.0 once the valve is closed, the tube is back in
its clip and both hands are clear for 1 s); the drain model against Torricelli; placement robustness (the graduate
turned or moved within its mouth's reach scores the same); a missed stream; a spout touch; the valve left open; the
tube left out of its clip; a dropped tube draining onto the floor; a tipped graduate; a hand still on the collar; a
new trial; the demo's draining pose, aim and timing; the thresholds in the text.

    python3 test_task.py
"""
import json
import math
from pathlib import Path

import numpy as np
from yam_arena.demo import grip_rotation
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import liquidkit as lk  # noqa: E402
import yamkit as kit  # noqa: E402

AWAY = {"left_gripper": (0.15, 0.30, M.TOP + 0.30), "right_gripper": (0.15, -0.30, M.TOP + 0.30)}
DRAIN = (M.DRAIN_A, M.DRAIN_PHI - M.DRAIN_A)
STOW = (M.STOW_A, M.STOW_B)


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(lk._quat(np.asarray(rot, float)), float), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=p - 0.05, high=p + 0.05)


def rz(a):
    return M.rot_axis("Z", a)


def fresh():
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None


class Trial:
    """A hand-built trial: the tube at joint angles (stub, tube), the valve at `valve` rad, the graduate where we put it,
    the left hand on the collar or away."""

    def __init__(self):
        fresh()
        self.a, self.b = STOW
        self.v = 0.0
        self.gpos = np.array([*M.GR_START, M.TOP])
        self.grot = np.eye(3)
        self.hand_on_collar = False
        self.s = 0.0
        self.n = 0

    def at(self, ab, valve=None, gpos=None, grot=None):
        self.a, self.b = ab
        if valve is not None:
            self.v = valve
        if gpos is not None:
            self.gpos = np.asarray(gpos, float)
        if grot is not None:
            self.grot = np.asarray(grot, float)
        return self

    def objects(self):
        f = M.fk(a=self.a, b=self.b, v=self.v)
        o = {"drainbag": st(M.STAND_POS), "graduate": st(self.gpos, self.grot)}
        for k in ("bag", "tube_x", "tube_a", "tube_b", "valve"):
            o[f"drainbag/{k}"] = st(*f[k])
        opening = {}
        for g, p in AWAY.items():
            o[g] = st(p, grip_rotation(0.0))
            opening[g] = 1.0
        if self.hand_on_collar:
            o["left_gripper"] = st(f["collar"], M.collar_turn(self.a + self.b, 0.0) @ grip_rotation(0.0))
            opening["left_gripper"] = (M.COLLAR_R - 0.002) / M.FINGER_HALF_OPEN
        kit._OPENING.update(opening)
        return o

    def step(self, n=1):
        for _ in range(n):
            for key, q in (("tube_a", self.a), ("tube_b", self.b), ("valve", self.v)):
                kit._JOINTS[f"drainbag/{key}"] = kit.JointState(q, 0.0, False)
            kit._EPISODE["last"] = self.n
            self.n += 1
            o = self.objects()
            M.success(o)
            self.s = M.score(o)
        return self.s


PLACE = np.array([*M.graduate_place(), M.TOP])
HOLD = M.HOLD_STEPS + 2


def drain_all(t, gpos=PLACE, grot=np.eye(3), seconds=None):
    t.at(DRAIN, valve=M.VALVE_OPEN_Q, gpos=gpos, grot=grot)
    return t.step(round((seconds or (M.drain_seconds() + 3.0)) / M.DT))


def test_nothing_done():
    t = Trial()
    check(t.step(30) == 0.0, "nothing done scores 0")
    check(M._L.graduate_ml() == 0.0 and M._L.spilled_ml() == 0.0, "no flow with the valve closed")
    check(abs(M._L.bag / M.ML - M.START_ML) < 1e-9, "the bag starts with 600 mL")


def test_start_layout():
    t = Trial()
    t.at(STOW, valve=M.VALVE_OPEN_Q)
    t.step(40)
    check(M._L.bag / M.ML == M.START_ML and M._L.spilled_ml() == 0.0,
          "the stowed spout points up above the bag's level: even opened it does not flow")
    f = M.fk()
    level = f["tube_a"][0][2] + M.bag_depth(M.START_ML * M.ML)
    check(f["tip"][2] - level > 0.005, f"the stowed spout is {1000 * (f['tip'][2] - level):.0f} mm above the full level")
    check(M.stowed(*STOW) and not M.stowed(M.PULL_A, math.pi - M.PULL_A), "stowed in the clip; the pulled tube is out")
    check(np.linalg.norm(np.array(M.GR_START) - PLACE[:2]) > 0.15, "the graduate starts away from the drain spot")
    check(not M.touches(t.objects()), "no touch at the start")


def test_full_drain():
    t = Trial()
    t.hand_on_collar = True
    drain_all(t)
    g, s = M._L.graduate_ml(), M._L.spilled_ml()
    check(g >= M.FULL_ML and s <= M.CLEAN_ML, f"the demo's aim drains the bag into the graduate ({g:.1f} mL, {s:.2f} "
                                             f"spilled)")
    check(abs(g + s + M._L.bag / M.ML - M.START_ML) < 1e-6, "volume is conserved")
    check(t.s == 0.5, "drained with the valve still open scores 0.5")
    t.at(DRAIN, valve=0.0)
    check(t.step(5) == 0.75, "valve closed, tube still out: 0.75")
    t.at(STOW)
    check(t.step(HOLD) == 0.75, "tube back in its clip but the hand still on the collar: 0.75")
    t.hand_on_collar = False
    check(t.step(2) == 0.75, "released: not yet held 1 s")
    check(t.step(HOLD) == 1.0, "released, clipped, closed and still for 1 s: 1.0")


def test_drain_rate():
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    head = f["tube_a"][0][2] + M.bag_depth(M.START_ML * M.ML) - f["tip"][2]
    q = M.CD * M.OUTLET_A * math.sqrt(2 * 9.81 * head) / M.ML
    check(20.0 < q < 40.0, f"the full bag starts draining at {q:.1f} mL/s (Torricelli, Cd 0.61, 6 mm bore)")
    check(15.0 < M.drain_seconds() < 60.0, f"it empties in {M.drain_seconds():.0f} s")
    t = Trial()
    t.at(DRAIN, valve=math.copysign((M.VALVE_OPEN + math.radians(M.VALVE_CLOSED_DEG)) / 2, M.VALVE_OPEN_Q), gpos=PLACE)
    t.step(20)
    half = M._L.graduate_ml()
    t2 = Trial()
    t2.at(DRAIN, valve=M.VALVE_OPEN_Q, gpos=PLACE)
    t2.step(20)
    check(abs(half / M._L.graduate_ml() - 0.5) < 0.05, "a half-open valve passes half the flow")
    check(M.valve_open_fraction(-math.radians(M.VALVE_CLOSED_DEG) + 0.01) == 0.0, "within 10 degrees: closed")


def test_placement_robust():
    results = []
    for yaw in (0.0, math.pi / 2, math.pi, -2.0):
        for dx, dy in ((0.0, 0.0), (0.01, 0.0), (-0.01, 0.0), (0.0, 0.015), (0.0, -0.015)):
            t = Trial()
            drain_all(t, gpos=PLACE + [dx, dy, 0.0], grot=rz(yaw))
            results.append((M._L.graduate_ml(), M._L.spilled_ml(), t.s))
    gs = [r[0] for r in results]
    check(max(gs) - min(gs) < 1e-6 and all(r[1] == 0.0 for r in results),
          "the graduate turned any way or moved 1-1.5 cm: the same volume caught, nothing spilled")


def test_missed_stream():
    t = Trial()
    drain_all(t, gpos=PLACE + [0.12, 0.0, 0.0])
    check(M._L.spilled_ml() > M.SPILL_MAX_ML and t.s == 0.0, "the graduate out of the stream's way: spilled, 0")


def test_touch():
    t = Trial()
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    o, _ = M.graduate_frame(t.objects())
    # move the graduate so its rim (near side) sits right under the spout's end
    rim_near = np.array([PLACE[0] - M.GR_T - M.GR_WALL / 2, PLACE[1], M.TOP + M.GR_RIM_Z])
    shift = f["tip"] - rim_near - np.array([0.0, 0.0, M.COLLAR_R * 0.5])
    t.at(DRAIN, valve=0.0, gpos=PLACE + [shift[0], shift[1], 0.0])
    t.gpos[2] = M.TOP + shift[2]
    t.step(1)
    check(M._L.touched, "the spout on the rim is a touch")
    t.at(DRAIN, valve=M.VALVE_OPEN_Q, gpos=PLACE)
    t.gpos[2] = M.TOP
    t.step(round((M.drain_seconds() + 3) / M.DT))
    t.at(STOW, valve=0.0)
    check(t.step(HOLD) == M.TOUCH_CAP, "a touch at any time caps a full drain at 0.25")
    t2 = Trial()
    t2.at(DRAIN, valve=0.0, gpos=PLACE)
    t2.step(5)
    check(not M._L.touched, "the demo's draining pose does not touch")


def test_left_open_and_out():
    t = Trial()
    drain_all(t)
    t.at(STOW)
    check(t.step(HOLD) == 0.5, "tube clipped but the valve left open: 0.5")
    t2 = Trial()
    drain_all(t2)
    t2.at(DRAIN, valve=0.0)
    check(t2.step(HOLD) == 0.75, "valve closed, tube left out of its clip: 0.75")


def test_dropped_tube():
    t = Trial()
    t.at((0.0, 0.0), valve=M.VALVE_OPEN_Q, gpos=PLACE)
    t.step(round(10 / M.DT))
    check(M._L.spilled_ml() > M.SPILL_MAX_ML and t.s == 0.0, "a dropped tube hangs down and drains onto the floor: 0")


def test_tipped_graduate():
    t = Trial()
    drain_all(t)
    t.at(DRAIN, valve=0.0, grot=M.rot_axis("X", math.radians(80)))
    t.step(40)
    check(M._L.spilled_ml() > M.SPILL_MAX_ML and t.s == 0.0, "the full graduate knocked over: spilled, 0")


def test_new_trial():
    t = Trial()
    drain_all(t)
    Trial().step(2)
    check(M._L.graduate_ml() == 0.0 and abs(M._L.bag / M.ML - M.START_ML) < 1e-9 and not M._L.touched,
          "a new trial starts with a full bag and an empty graduate")


def test_demo_geometry():
    f = M.fk(a=DRAIN[0], b=DRAIN[1])
    gap = f["tip"][2] - (M.TOP + M.GR_RIM_Z)
    check(0.01 < gap < 0.06, f"the draining spout is {1000 * gap:.0f} mm above the rim")
    c = np.array(M.graduate_place())
    check(np.linalg.norm(f["tip"][:2] - c) < M.GR_T - 0.02, "the spout is over the graduate's mouth")
    t = Trial()
    t.at(DRAIN, valve=0.0, gpos=PLACE)
    o, _ = M.graduate_frame(t.objects())
    pts = M.spout_points(t.objects())
    check(all(M.wall_distance(p - o) > r + 0.008 for p, r in pts), "the spout stays 8 mm or more from the wall")
    bag_bottom = f["tube_a"][0][2]
    check(M.TOP + M.GR_RIM_Z < bag_bottom - 0.05, "the graduate fits under the bag's bottom")
    d = M.demo()
    dur = max(sum(w.duration for w in d[s]) for s in d)
    check(dur * 20 + 100 < M.RUN_LIMIT_STEPS, f"the demo ({dur:.0f} s) fits the run limit")
    labels = [w.label for w in d["left"]]
    t_swing = sum(w.duration for w in d["left"][:labels.index("raise the stub 1")])
    check(t_swing >= M.LEFT_DONE_S - 1e-9, "the tube swings down only after the graduate is down")
    rl = [w.label for w in d["right"]]
    t_open = sum(w.duration for w in d["right"][:rl.index("twist the valve open")])
    t_aim = sum(w.duration for w in d["left"][:labels.index("hold the aim while the valve is opened")])
    check(t_open >= t_aim - 1e-9, "the valve is opened only once the spout is aimed")
    t_close = sum(w.duration for w in d["right"][:rl.index("twist the valve closed")])
    t_up = sum(w.duration for w in d["left"][:labels.index("swing the tube back up 1")])
    check(t_up >= t_close + M.TWIST_S - 1e-9, "the tube swings back up only after the valve is closed")
    check(M.T_CLOSING - M.T_FLOW >= M.drain_seconds(upto=0.97, frac=M.HOLD_OPEN_FRAC),
          "the valve stays open until 97 % of the bag has passed even at 60 % open")
    turn = M.collar_turn(M.DRAIN_PHI, -M.VALVE_OPEN) @ M.collar_turn(M.DRAIN_PHI, 0.0).T
    ang = math.degrees(math.acos((np.trace(turn) - 1) / 2))
    check(abs(ang - 90.0) < 1e-6, "the twist turns the hand 90 degrees about the collar")
    m0 = M.collar_turn(M.DRAIN_PHI, 0.0)
    d_axis = np.array([-math.sin(M.DRAIN_PHI), 0.0, -math.cos(M.DRAIN_PHI)])
    check(abs(float((m0 @ np.array([0.0, 1.0, 0.0])) @ d_axis)) < 1e-9 and abs(float((m0 @ np.array([0, 0, -1.0]))
                                                                                     @ d_axis)) < 1e-9,
          "the fingers close and point square to the collar's axis")


def test_text_thresholds():
    j = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    for s in ("570 mL", "600 mL", "5 mL", "20 mL", "300 mL", "100 mL", "10 degrees", "1 s", "0.25", "1 mm"):
        check(s in j["scoring"], f"task.json scoring states '{s}'")
        check(s in readme, f"README states '{s}'")
    check(j["instruction"] == M.INSTRUCTION, "instruction matches")
    check("2,400 steps" in j["run_limit"] and M.RUN_LIMIT_STEPS == 2400, "run limit 2,400")
    check(len(j.get("context", "")) <= 1000, f"context at most 1,000 characters ({len(j.get('context', ''))})")
    check("Real-world evaluation (reproducible protocol)" in readme, "README has the reproducible real-world protocol")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_") and callable(fn):
            fn()
    print("all pass")
