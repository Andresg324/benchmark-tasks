"""Offline checks of the toothpaste-dose scorer, no simulator. Tube and brush states are built by hand from the same
geometry the demo uses; the tube's wall pushes are set as its joint states. Covers: doing nothing; a holding grip
(no paste); the demo's pea dose (1.0 once the tube is down and all is still for 1 s); too much (a half load and an adult
strip); too little (a smear); paste off the bristles (on the counter; on the bristles' edge, running off); the dose
scored the same wherever and however the brush is held (moved, tilted, laid down after dosing); bristles tipped past
60 degrees while squeezing (nothing lands); the brush flipped with paste on it; the tube still in hand; the 1 s hold;
a new trial; the thresholds in the text; and the demo plan's reach.

    python3 test_task.py
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
import yamkit as kit  # noqa: E402

AWAY = {"left_gripper": (0.15, 0.30, M.TOP + 0.30), "right_gripper": (0.15, -0.30, M.TOP + 0.30)}
GRIP_DOWN = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
NOZZLE_DOWN = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])  # the tube's -x (nozzle) to world -z


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


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def fresh():
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    M._L = None


class Trial:
    """A hand-built trial: the brush and tube where we put them, the tube's walls pushed in `push` m a side."""

    def __init__(self):
        fresh()
        self.brush = (np.array(M.BRUSH_START, float), np.eye(3))
        self.tube = (np.array(M.TUBE_START, float), np.eye(3))
        self.hand_on_tube = False
        self.s = 0.0

    def nozzle_over(self, dx=0.0, dy=0.0, gap=M.NOZZLE_GAP, rot=NOZZLE_DOWN):
        """The nozzle's mouth `gap` over the bristle top's middle (offset dx, dy along the brush's own x, y)."""
        bp, br = self.brush
        mouth = bp + br @ np.array([dx, dy, 0.0]) + np.array([0.0, 0.0, gap])
        self.tube = (mouth - rot @ np.array(M.TIP), rot)

    def tube_down(self):
        self.tube = (np.array(M.TUBE_START, float), np.eye(3))

    def step(self, push=0.0, n=1):
        for _ in range(n):
            for link, sign in M.PANEL_SIGN.items():
                d = push if link in ("panel_l", "panel_r") else 0.0
                kit._JOINTS[f"tube/{link}"] = kit.JointState(sign * d, 0.0, False)
            o = {"brush": st(*self.brush), "tube": st(*self.tube)}
            opening = {}
            for g, p in AWAY.items():
                o[g] = st(p, GRIP_DOWN)
                opening[g] = 1.0
            if self.hand_on_tube:
                tp, tr = self.tube
                o["right_gripper"] = st(tp, tr @ rz(math.pi / 2) @ GRIP_DOWN)
                opening["right_gripper"] = M.tube_grip(push)
            kit._OPENING.update(opening)
            M.success(o)
            self.s = M.score(o)
        return self.s

    def squeeze(self, push, hold_s, tail_s=M.TAIL_S):
        n_ramp = 6
        for k in range(1, n_ramp + 1):
            self.step(push * k / n_ramp)
        self.step(push, round(hold_s / M.DT))
        self.step(M.HOLD_PUSH, round(tail_s / M.DT))

    def finish(self, still_s=1.5):
        self.tube_down()
        return self.step(0.0, round(still_s / M.DT))

    @property
    def on(self):
        return M._L.paste.on_g()

    @property
    def off(self):
        return M._L.paste.off_g()


def test_nothing():
    t = Trial()
    check(t.step(0.0, 60) == 0.0, "doing nothing scores 0")


def test_holding_grip():
    t = Trial()
    t.nozzle_over()
    t.step(M.HOLD_PUSH, 100)
    check(t.on == 0.0 and t.off == 0.0, f"a holding grip ({M.HOLD_PUSH * 1000:.1f} mm a side) dispenses nothing")


def pea():
    """The demo's squeeze on the brush lying at its start, then the tube put down: returns the trial."""
    t = Trial()
    t.nozzle_over()
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(M.PEA_G[0] <= t.on <= M.PEA_G[1], f"the demo's squeeze leaves a pea dose ({t.on:.3f} g)")
    check(t.off <= M.OFF_G, f"nothing off the bristles ({t.off:.3f} g)")
    check(t.step(0.0, 5) == 0.75, "dosed, tube still over the brush: 0.75")
    check(t.finish(0.5) == 0.75, "tube down for only 0.5 s: 0.75")
    check(t.finish(1.5) == 1.0, "tube down and all still for 1 s: 1.0")
    return t


def test_pea():
    pea()


def test_too_much():
    t = Trial()
    t.nozzle_over()
    t.squeeze(*M.VARIANTS["much"])
    check(M.PEA_G[1] < t.on <= M.OVER_G, f"a firmer squeeze: too much ({t.on:.3f} g)")
    check(t.finish() == 0.25, "too much (0.35-0.70 g) scores 0.25")
    t = Trial()
    t.nozzle_over()
    t.squeeze(0.0040, 3.0)
    check(t.on + t.off > M.OVER_G, f"an adult strip ({t.on + t.off:.3f} g dispensed)")
    check(t.finish() <= 0.25, f"an adult strip scores at most 0.25 ({t.s})")
    t = Trial()
    t.nozzle_over()
    t.squeeze(0.0060, 4.0)
    check(t.finish() == 0.0 and (t.on > M.OVER_G or t.off > M.OFF_G),
          f"a full load beyond 0.70 g on the bristles scores 0 ({t.on:.3f} g on, {t.off:.3f} g off)")


def test_too_little():
    t = Trial()
    t.nozzle_over()
    t.squeeze(*M.VARIANTS["smear"])
    check(M.SMEAR_G <= t.on < M.PEA_G[0], f"a short squeeze: a smear ({t.on:.3f} g)")
    check(t.finish() == 0.25, "a smear scores 0.25")
    t = Trial()
    t.nozzle_over()
    t.squeeze(0.0009, 0.3)
    check(t.on < M.SMEAR_G and t.finish() == 0.0, f"a trace ({t.on:.3f} g) scores 0")


def test_off_bristles():
    t = Trial()
    t.nozzle_over(dy=0.030)  # beside the head: onto the counter
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(t.on == 0.0 and t.off > M.OFF_G, f"squeezed beside the brush: all off ({t.off:.3f} g)")
    check(t.finish() == 0.0, "on the counter scores 0")
    t = Trial()
    t.nozzle_over(dx=0.0085)  # on the far end of the bristle field: part of the dab slumps off onto the head
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(t.off > M.OFF_G, f"dosed on the field's edge: some runs off ({t.on:.3f} on, {t.off:.3f} off)")
    check(t.finish() in (0.25, 0.5), f"a dab half off the bristles scores below 0.75 ({t.s})")


def test_placement_robust():
    """The dose is measured in the brush's frame: the same squeeze gives the same score whether the brush lies on the
    counter, is held up somewhere else, or held turned and tilted; and moving it afterwards changes nothing."""
    ref = pea()
    for label, pos, rot in (("lying on the counter", M.BRUSH_START, np.eye(3)),
                            ("held up elsewhere", (0.250, -0.050, M.TOP + 0.12), np.eye(3)),
                            ("held turned 40 deg and tilted 25 deg", (0.300, 0.000, M.TOP + 0.10),
                             rz(math.radians(40)) @ rx(math.radians(25)))):
        t = Trial()
        t.brush = (np.asarray(pos, float), rot)
        t.nozzle_over()
        t.squeeze(M.DOSE_PUSH, M.DOSE_S)
        check(abs(t.on - ref.on) < 0.01, f"brush {label}: {t.on:.3f} g on, as with the brush level ({ref.on:.3f})")
        t.brush = (np.array([0.33, 0.12, M.TOP + 0.014]), rz(math.radians(-30)))
        check(t.finish() == 1.0, f"brush {label}, then laid down elsewhere: 1.0")


def test_placement_sweep():
    """Every brush pose a policy can reach: the same squeeze over the bristles' middle leaves the same grams on them
    (held anywhere over the counter, any heading, tipped up to 40 degrees about either of its axes), and the trial ends
    1.0 once the tube is down; a nozzle anywhere over the field lands on it, one past the head's end lands off it."""
    ref = None
    n = 0
    for pos in ((0.32, 0.08, M.TOP + 0.014), (0.20, -0.10, M.TOP + 0.08), (0.30, 0.00, M.TOP + 0.15),
                (0.38, 0.15, M.TOP + 0.10)):
        for k in range(8):
            for tilt in ((0.0, 0.0), (40.0, 0.0), (0.0, 40.0), (-25.0, 25.0)):
                rot = rz(k * math.pi / 4) @ rx(math.radians(tilt[0])) @ ry(math.radians(tilt[1]))
                t = Trial()
                t.brush = (np.asarray(pos, float), rot)
                t.nozzle_over()
                t.squeeze(M.DOSE_PUSH, M.DOSE_S)
                ref = t.on if ref is None else ref
                s = t.finish()
                if abs(t.on - ref) > 0.01 or t.off > M.OFF_G or s != 1.0:
                    check(False, f"brush at {pos} heading {k} tilt {tilt}: {t.on:.3f} g on, {t.off:.3f} off, {s}")
                n += 1
    check(n == 128, f"same dose ({ref:.3f} g) and 1.0 at all {n} brush poses")
    for dx, dy in ((-0.007, 0.0), (0.007, 0.0), (0.0, -0.003), (0.0, 0.003)):
        t = Trial()
        t.nozzle_over(dx, dy)
        t.squeeze(M.DOSE_PUSH, M.DOSE_S)
        check(t.on >= M.PEA_G[0], f"nozzle {dx * 1000:+.0f}, {dy * 1000:+.0f} mm off the field's middle: "
              f"{t.on:.3f} g on the bristles")
    t = Trial()
    t.nozzle_over(0.025, 0.0)
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(t.on == 0.0 and t.off > M.OFF_G, "nozzle 25 mm along the brush (past the head): the dab lands off")


def test_tipped_while_squeezing():
    t = Trial()
    t.brush = (np.array([0.30, 0.02, M.TOP + 0.10]), rx(math.radians(70)))
    t.nozzle_over()
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    check(t.on == 0.0, "bristles 70 degrees from up while squeezing: nothing lands on them")


def test_flipped():
    t = Trial()
    t.nozzle_over()
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    t.brush = (t.brush[0], rx(math.radians(120)))
    t.step(0.0, 3)
    t.brush = (t.brush[0], np.eye(3))
    check(t.finish() == 0.25, "brush turned bristles-down with the dab on it: capped at 0.25")


def test_tube_held():
    t = Trial()
    t.nozzle_over()
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    t.tube_down()
    t.hand_on_tube = True
    check(M.tube_held({**{g: st(p, GRIP_DOWN) for g, p in AWAY.items()},
                       "tube": st(*t.tube), "right_gripper": st(t.tube[0], rz(math.pi / 2) @ GRIP_DOWN)}),
          "a hand closed round the tube counts as holding it")
    check(t.step(0.0, 40) == 0.75, "tube down but still in hand: 0.75")


def test_brush_left_on_side():
    t = Trial()
    t.nozzle_over()
    t.squeeze(M.DOSE_PUSH, M.DOSE_S)
    t.brush = (t.brush[0], rx(math.radians(60)))
    check(t.finish() == 0.75, "brush left with the bristles 60 degrees from up: 0.75")


def test_new_trial():
    pea()
    t = Trial()
    check(t.step(0.0, 30) == 0.0 and t.on == 0.0, "a new trial starts with no paste and scores 0")


def test_text_numbers():
    js = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    text = js["scoring"] + " " + js["setup"] + " " + js["fidelity"]
    for v in ("0.15", "0.35", "0.05 g", "0.70", "45 degrees", "60 degrees", "1.30 g/mL", "0.01 g"):
        check(v in text and v in readme, f"'{v}' in task.json and the README")
    check(js["instruction"] == M.INSTRUCTION, "instruction matches task.py")
    check(re.search(r"1,200 steps", js["run_limit"]) is not None and M.RUN_LIMIT_STEPS == 1200, "run limit 1,200")
    check(js["category"] == "Societal relatability", "category")
    check(len(js["title"]) <= 120 and len(js["fidelity"]) <= 500, "title and fidelity within the limits")


def test_demo_plan():
    plan = M.demo()
    total = {a: sum(w.duration for w in p) for a, p in plan.items()}
    check(max(total.values()) / M.DT <= M.RUN_LIMIT_STEPS, f"demo fits the run limit ({max(total.values()):.1f} s)")
    xs = [w.pos[0] for p in plan.values() for w in p]
    check(0.15 <= min(xs) and max(xs) <= 0.45, f"tool points within x 0.15-0.45 m ({min(xs):.3f}-{max(xs):.3f})")
    def until(arm, label):
        labels = [w.label for w in plan[arm]]
        return sum(w.duration for w in plan[arm][:labels.index(label) + 1])
    t_at = until("right", "nozzle 4 mm over the bristles")
    t_done = until("right", "lift off the dab")
    t_held = until("left", "hold it bristles-up")
    t_free = until("left", "hold still while it is dosed")
    check(until("left", "flick the lid 80 deg") <= until("right", "hold it up for the lid") + 3.0
          and until("left", "fingertip behind the thumb tab") >= until("right", "hold it up for the lid"),
          "the lid is flicked while the tube is held up")
    check(t_held <= t_at and t_done <= t_free, "the brush is held still for the whole dose")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for f in tests:
        f()
    print("all pass")
    sys.exit(0)
