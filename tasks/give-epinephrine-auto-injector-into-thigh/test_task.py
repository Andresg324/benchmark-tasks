"""Offline checks of the auto-injector scorer and demo geometry, no simulator. Each trial step is built by hand: the
pen's pose and its sleeve's joint position (as the simulator reports them each control step), the release, the pad
and tray where they stand, the hands where given. Covers: the start layout (pen in the rest's groove, release on its
pin, tip and release clear of everything, the pad's crown 11 cm up); doing nothing; the release pulled off (0.25);
pushing with the release on (blocked, never fires); a click on target, lifted early (0.5); held 3 s (0.75); laid on
the tray, let go and still 1 s (1.0), still held (stays 0.75); a click into the air, the table, off the target, at a
slant or blue end first (latched 0); the pen off the table (latched 0); a new trial; the target check over 175 tip
placements and tilts (inside and just outside the zone); the tray check over 125 pen poses on the tray; the sleeve
spring numbers; the demo's poses; the thresholds in the text.

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
import yamkit as kit  # noqa: E402

AWAY = {"left_gripper": (0.15, 0.32, M.TOP + 0.30), "right_gripper": (0.15, -0.32, M.TOP + 0.30)}


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


def st(pos, R=np.eye(3), half=0.03):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat(R), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - half, high=p + half)


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


UP = ry(-math.pi / 2)  # pen frame -> world with the blue end up (pen x = world +z)
LEVEL = np.eye(3)
START = np.array([M.PEN_START[0], M.PEN_START[1], M.PEN_Z0])
CREST = np.array([M.PAD_C[0], M.PAD_C[1], M.CREST_Z])


def fresh():
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    M._L = None


class Trial:
    def __init__(self):
        self.n = 0

    def objects(self, pen=START, R=LEVEL, q=0.0, cap=None, hand=None, opening=0.3):
        """One step. cap=None: the release on its seat; "off": 5 cm above the table on the left. hand: a gripper
        holding the pen at its middle."""
        o = {"pen": st(pen, R), "pad": st((M.PAD_C[0], M.PAD_C[1], M.TOP), half=0.08),
             "tray": st((M.TRAY_C[0], M.TRAY_C[1], M.TOP), half=0.07),
             "pen_rest": st((M.TRAY_C[0], M.TRAY_C[1], M.TRAY_TOP), half=0.05)}
        seat = np.asarray(pen, float) + np.asarray(R) @ M.CAP_SEAT
        o["release"] = st(seat if cap is None else (0.24, 0.08, M.TOP + 0.05), R, half=0.012)
        for g, p in AWAY.items():
            o[g] = st(p, grip_rotation(0.0))
            kit._OPENING[g] = 1.0
        if hand is not None:
            o[hand] = st(pen, grip_rotation(0.0))
            kit._OPENING[hand] = opening
        kit._JOINTS["pen/sleeve"] = kit.JointState(pos=float(q), vel=0.0, latched=False)
        self.n += 1
        kit._EPISODE["last"] = self.n
        return o


def pen_for_tip(tip, R=UP, q=M.CLICK_Q):
    """The pen's middle that puts its sleeve tip (pushed in q) at world point tip."""
    return np.asarray(tip, float) - np.asarray(R) @ (M.TIP_LOCAL + np.array([q, 0.0, 0.0]))


def run(trial, steps, **kw):
    s = None
    for _ in range(steps):
        s = M.score(trial.objects(**kw))
    return s


def fire_and_hold(t, hold_steps, tip=CREST, R=UP):
    """Release off in the air, click on tip, then hold_steps steps pushed in at HOLD_Q+."""
    held = dict(pen=pen_for_tip(tip, R), R=R, cap="off", hand="right_gripper")
    run(t, 2, pen=pen_for_tip(tip + np.array([0, 0, 0.05]), R, 0.0), R=R, cap="off", hand="right_gripper")
    run(t, 2, q=M.CLICK_Q, **held)
    return run(t, hold_steps, q=M.HOLD_Q + 0.001, **held)


def test_layout():
    fresh()
    o = Trial().objects()
    check(M.cap_on(o), "release on its pin at the start")
    check(abs(M.CREST_Z - M.TOP - 0.110) < 1e-9, "pad crown 11 cm above the table (Limbs & Things 00310)")
    half_w = math.sqrt(M.PAD_R ** 2 - M.PAD_AXIS_Z ** 2)
    check(abs(2 * half_w - 0.162) < 0.002, "pad 16.2 cm wide at the table")
    check(abs(M.PEN_Z0 - M.BODY_R - M.REST_FLOOR) < 1e-9, "pen lies on the groove floor")
    check(M.REST_SIZE[1] / 2 - (M.REST_SIZE[1] - 0.024) / 2 >= M.BODY_R, "groove wider than the pen")
    check(M.CAP_OUT < M.BODY_R, "release narrower than the body (it hangs clear of the groove)")
    check(abs(M.CAP_IN - M.POST_R - 0.0015) < 1e-9, "1.5 mm round the pin in the release's bore")
    total = M.SLEEVE_OUT + M.BODY_L + 0.0005 + M.CAP_L
    check(0.13 < total < 0.141, f"pen {total * 100:.1f} cm overall (fits the 5-1/2 in carrier tube)")
    tip = M.tip_world(o)
    check(tip[0] < START[0] and abs(tip[2] - START[2]) < 1e-9, "orange tip toward the robot")
    check(abs(M.K_FIRE * M.CLICK_Q - 23.0) < 1e-6, "sleeve spring gives 23 N at the click")
    check(M.K_FIRE * M.CLICK_Q < 52.0, "23 N within the YAM's 52 N continuous push down")
    check(M.score(o) == 0.0, "nothing done: 0")


def test_release_and_blocked():
    fresh()
    t = Trial()
    s = run(t, 3, pen=pen_for_tip(CREST, UP), R=UP, q=M.CLICK_Q, hand="right_gripper")
    check(s == 0.0 and not M._L.fired, "pressed to the click with the release on: blocked, no fire, 0")
    check(M.STATE["cap_on"], "the sleeve's law sees the release on")
    s = run(t, 3, pen=START + np.array([0, 0, 0.1]), R=UP, cap="off", hand="right_gripper")
    check(s == 0.25 and not M.STATE["cap_on"], "release off: 0.25, sleeve unblocked")


def test_click_hold_lay():
    fresh()
    t = Trial()
    s = fire_and_hold(t, M.HOLD_STEPS - 5)
    check(s == 0.5, "clicked on target, held under 3 s: 0.5")
    s = run(t, 6, pen=pen_for_tip(CREST, UP), R=UP, q=M.HOLD_Q + 0.001, cap="off", hand="right_gripper")
    check(s == 0.75, "held 3 s: 0.75")
    s = run(t, 30, pen=START, R=LEVEL, cap="off", hand="right_gripper")
    check(s == 0.75, "on the tray, still held: 0.75")
    s = run(t, M.SUCCESS_STEPS, pen=START, R=LEVEL, cap="off")
    check(s == 1.0, "on the tray, let go, still 1 s: 1.0")
    fresh()
    s = M.score(Trial().objects())
    check(s == 0.0, "a new trial starts at 0")


def test_lifted_early():
    fresh()
    t = Trial()
    fire_and_hold(t, 20)
    s = run(t, 5, pen=pen_for_tip(CREST + np.array([0, 0, 0.03]), UP, 0.0), R=UP, cap="off", hand="right_gripper")
    check(s == 0.5, "lifted 1 s after the click: 0.5")
    s = run(t, M.HOLD_STEPS, pen=pen_for_tip(CREST, UP), R=UP, q=M.HOLD_Q + 0.001, cap="off", hand="right_gripper")
    check(s == 0.5, "pushed back on afterwards: still 0.5 (the dose went in short)")
    s = run(t, 40, pen=START, R=LEVEL, cap="off")
    check(s == 0.5, "laid on the tray after a short hold: 0.5")


def test_misfires():
    cases = {"into the air": (CREST + np.array([0, 0, 0.05]), UP),
             "into the table": (np.array([0.20, -0.02, M.TOP]), UP),
             "beside the target": (CREST + np.array([0.0, M.TARGET_Y + 0.01, 0.0]), UP),
             "at a 30 degree slant": (CREST, UP @ rz(0.0) @ np.array(ry(math.radians(30.0)))),
             "blue end first": (CREST, ry(math.pi / 2))}
    for name, (tip, R) in cases.items():
        fresh()
        t = Trial()
        s = fire_and_hold(t, M.HOLD_STEPS + 2, tip=tip, R=R)
        s2 = run(t, 40, pen=START, R=LEVEL, cap="off")
        check(s == 0.0 and s2 == 0.0 and M._L.misfire, f"fired {name}: latched 0")
    fresh()
    t = Trial()
    run(t, 3, pen=START + np.array([0, 0, 0.1]), R=UP, cap="off", hand="right_gripper")
    s = run(t, 2, pen=np.array([0.3, -0.3, M.TOP - 0.2]), R=LEVEL, cap="off")
    s2 = fire_and_hold(t, M.HOLD_STEPS + 2)
    check(s == 0.0 and s2 == 0.0, "pen off the table: latched 0")


def test_target_sweep():
    fresh()
    n_in = n_out = 0
    for dy in np.linspace(-M.TARGET_Y + 0.001, M.TARGET_Y - 0.001, 5):
        for arc in np.linspace(-M.TARGET_ARC + 0.001, M.TARGET_ARC - 0.001, 5):
            for tilt in (-19.0, -10.0, 0.0, 10.0, 19.0):
                b = arc / M.PAD_R
                normal = np.array([math.sin(b), 0.0, math.cos(b)])
                tip = np.array([M.PAD_C[0], M.PAD_C[1] + dy, M.TOP + M.PAD_AXIS_Z]) + M.PAD_R * normal
                R = ry(b) @ rz(0.0) @ UP
                R = np.array([[1, 0, 0], [0, math.cos(math.radians(tilt)), -math.sin(math.radians(tilt))],
                              [0, math.sin(math.radians(tilt)), math.cos(math.radians(tilt))]]) @ R
                o = Trial().objects(pen=pen_for_tip(tip, R), R=R, q=M.CLICK_Q, cap="off")
                n_in += M.on_target(o)
                o = Trial().objects(pen=pen_for_tip(tip + np.array([0, 0.05 * np.sign(dy or 1), 0]), R), R=R, q=M.CLICK_Q,
                                    cap="off")
                n_out += not M.on_target(o)
    check(n_in == 125 and n_out == 125, f"target: {n_in}/125 tips inside counted, {n_out}/125 outside refused")
    o = Trial().objects(pen=pen_for_tip(CREST + np.array([0, 0, M.ON_SKIN + 0.002]), UP), R=UP, q=M.CLICK_Q, cap="off")
    check(not M.on_target(o), "tip 6 mm above the skin: not on it")
    edge = np.array([M.PAD_C[0] + M.PAD_R * math.sin((M.TARGET_ARC + 0.003) / M.PAD_R), M.PAD_C[1],
                     M.TOP + M.PAD_AXIS_Z + M.PAD_R * math.cos((M.TARGET_ARC + 0.003) / M.PAD_R)])
    o = Trial().objects(pen=pen_for_tip(edge, UP), R=UP, q=M.CLICK_Q, cap="off")
    check(not M.on_target(o), "3 mm past the target's edge across the crown: off")


def test_tray_sweep():
    fresh()
    ok = 0
    for dx in np.linspace(-0.08, 0.08, 5):
        for dy in np.linspace(-0.05, 0.05, 5):
            for yaw in np.radians([0.0, 45.0, 90.0, 150.0, 200.0]):
                o = Trial().objects(pen=np.array([M.TRAY_C[0] + dx, M.TRAY_C[1] + dy, M.TRAY_TOP + M.BODY_R]),
                                    R=rz(yaw), cap="off")
                ok += M.on_tray(o)
    check(ok == 125, f"pen lying anywhere on the tray at any heading: {ok}/125")
    o = Trial().objects(pen=np.array([M.TRAY_C[0], M.TRAY_C[1] + 0.12, M.TOP + M.BODY_R]), cap="off")
    check(not M.on_tray(o), "pen on the table beside the tray: not on it")
    o = Trial().objects(pen=np.array([M.TRAY_C[0], M.TRAY_C[1], M.TRAY_TOP + 0.07]), R=UP, cap="off")
    check(not M.on_tray(o), "pen standing up over the tray: not lying on it")


def test_demo():
    d = M.demo()
    for side in ("left", "right"):
        check(sum(w.duration for w in d[side]) / M.DT < M.RUN_LIMIT_STEPS, f"{side} demo inside the run limit")
    tip_up = M.BODY_L / 2 + M.SLEEVE_OUT
    press = [w for w in d["right"] if w.label.startswith("push")][0]
    check(abs(press.pos[2] - (M.CREST_Z + tip_up - M.PUSH_PAST)) < 1e-9, "press commanded PUSH_PAST into the crown")
    hold = [w for w in d["right"] if w.label == "hold it 3 s"][0]
    check(hold.duration >= M.HOLD_S + 0.5, "demo holds 0.5 s longer than 3 s")
    cap = M.CAP_POSE + np.array([0.0, 0.0, M.CAP_SEAT[0]])
    check(cap[2] - M.CAP_L / 2 - (M.CAP_POSE[2] + M.CAP_GRIP_UP + M.CAP_SEAT[0] - M.TIP_PAST_TCP) < 0,
          "left fingertips reach below the release's top")
    tips = M.CAP_POSE[2] + M.CAP_SEAT[0] + M.CAP_GRIP_UP - M.TIP_PAST_TCP
    check(tips - (M.CAP_POSE[2] + M.BODY_L / 2) >= 0.003, "left fingertips 3 mm+ clear of the body's end")
    check(M.PULL_TOP - (M.CAP_POSE[2] + M.CAP_SEAT[0] + M.CAP_GRIP_UP - M.TOP) >= M.CAP_OFF + 0.003,
          "the pull lifts the release past CAP_OFF")
    check(M.CAP_POSE[2] - tip_up - M.TOP >= 0.02, "orange tip 2 cm+ above the table while the release comes off")
    check(M.CAP_START_S + sum(M.L_TIMES) <= M.CAP_DONE_S, "right hand waits for the pull")


def test_text_thresholds():
    j = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    text = j["scoring"] + " " + j["setup"] + " " + j["fidelity"]
    for v in ("7 mm", "4 mm", "20 degrees", "3 s", "1 s", "23 N", "0.25", "0.5", "0.75", "1.0", "6 x 4 cm"):
        check(v in text and v in readme, f"'{v}' in task.json and README")
    check(j["instruction"] == M.INSTRUCTION, "instruction matches")
    check(M.RUN_LIMIT_STEPS == 1200 and "1,200 steps" in j["run_limit"], "run limit matches")
    check(len(j["fidelity"]) <= 500 and len(j["title"]) <= 120, "field limits")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
    print("all checks passed")
