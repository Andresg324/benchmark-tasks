"""Offline checks of the trap-arm fall scorer: stand settings, gripper histories and near misses, no simulator.

    python test_task.py        (with the yam_arena package importable; prints "all pass")
"""

import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from yam_arena.task_file import ObjectState  # noqa: E402

try:  # the instance's yam_arena loads task files itself; the offline stub does not
    from yam_arena.task_file import load
    m = load(os.path.join(HERE, "task.py")).module
except (ImportError, NotImplementedError):
    _spec = importlib.util.spec_from_file_location("task", os.path.join(HERE, "task.py"))
    m = importlib.util.module_from_spec(_spec)
    _spec.loader.exec_module(m)
kit, lv, tk = m.kit, m.lv, m.tk
from yam_arena.constants import FINGER_OPEN  # noqa: E402

lv.register(m.DRAIN)
tk.register(*m.SCREWS)
PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
KEYS = {f: f"{m.PIPE}_{f}/knob" for f in ("sink", "mid", "stack")}
NAMES = ("sink", "mid", "stack")
FOOT = {f.name: f for f in m.FEET}
fails = 0


def check(cond, msg):
    global fails
    print(("ok  " if cond else "FAIL") + " " + msg)
    fails += not cond


def st(pos, quat=(0, 0, 0, 1.0), vel=(0, 0, 0), low=None):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.array(vel, float), ang_vel=np.zeros(3),
                       low=p - 0.01 if low is None else np.array(low, float), high=p + 0.01)


def set_stands(turns):
    """Screws at `turns` (sink, mid, stack) turns out (twistkit angles: negative past the seat = backed off)."""
    for f, t in zip(NAMES, turns):
        tk.set_angle(KEYS[f], lv.past_seat_for(FOOT[f], t), held_now=False)


def step(stands=None, grippers=None, opening=None, tips=None):
    """One control step. stands = (sink, mid, stack) turns out (None: as is); grippers = {name: tool point};
    tips = {name: pad-tip z}. Returns (score, success)."""
    if stands is not None:
        set_stands(stands)
    g = dict(PARK, **(grippers or {}))
    objs = {m.PIPE: st(m.PIPE_POS)}
    for f, key in KEYS.items():
        x, y = lv._world(m.DRAIN, next(ft for ft in m.FEET if ft.name == f).xy)
        objs[key] = st((x, y, m.TOP))
    for k, p in g.items():
        tz = (tips or {}).get(k, p[2] - 0.013)
        objs[k] = st(p, low=(p[0], p[1], tz))
    kit._OPENING.update({"left_gripper": 1.0, "right_gripper": 1.0})
    kit._OPENING.update(opening or {})
    tk._account(objs)  # the sim's per-step order: yamkit, twistkit, levelkit, then success() and score()
    lv.update(objs)
    ok = m.success(objs)
    return m.score(objs), ok


def new_trial():
    kit._EPISODE["count"] += 1
    for k in KEYS.values():
        tk._TRACK.pop(k, None)
    return step()  # the start state, arms parked: home recorded


def run(steps):
    new_trial()
    out = None
    for s in steps:
        out = step(**s)
    return out


def stands_for(fall_mm_per_m, mid_off_mm=0.0, center_mm=0.0):
    """Screw turns that set the given fall toward the stack, the middle stand `mid_off_mm` off the line (+ = above),
    the pipe's middle `center_mm` above the start level."""
    d = fall_mm_per_m * m.SPAN / 2 / 1000.0  # m each end moves
    z = {"sink": m.H0 + center_mm / 1000 + d, "mid": m.H0 + (center_mm + mid_off_mm) / 1000,
         "stack": m.H0 + center_mm / 1000 - d}
    out = []
    for f in m.FEET:
        out.append((z[f.name] - f.ext) / m.LEAD)
    return tuple(out)


START = tuple(m.START[f] for f in NAMES)
DONE = (m.START["sink"] + 1.6, m.START["mid"] + 0.5, m.START["stack"] - 1.6)  # the demo's end setting
AWAY = {"left_gripper": (0.25, 0.25, m.TOP + 0.15)}
HOLD = 20

# ---- the start state, the thresholds and the model ---------------------------------------------------------------
new_trial()
o = {m.PIPE: st(m.PIPE_POS)}
s0 = lv.state(o, m.PIPE)
check(abs(m.fall(o)) < 1e-9 and abs(s0.gaps_mm["mid"] - 0.977) < 1e-3 and lv.slack(o, m.PIPE) == ("mid",),
      f"start: dead level, middle stand 1 mm slack ({ {k: round(v, 2) for k, v in s0.gaps_mm.items()} })")
check(abs(m.FALL_MIN - 20.833) < 0.01 and abs(m.FALL_HALF - 10.417) < 0.01 and abs(m.FALL_MAX - 31.75) < 1e-9,
      f"window {m.FALL_MIN:.1f} to {m.FALL_MAX:.1f} mm/m, half fall {m.FALL_HALF:.1f}")
check(lv.knob_limit_deg(m.SCREWS) <= 450 + 1e-6, f"screw-head hinge limit {lv.knob_limit_deg(m.SCREWS):.0f} deg: "
      "within the 450 (1.25 turns) proven in the sim")
starts = [abs(lv.past_seat_for(FOOT[f], m.START[f])) for f in NAMES]
check(max(starts) <= 360 + 1e-6, f"heads start at most 360 deg from their hinge zero ({[round(a) for a in starts]})")
check(all(0 <= t <= m.TRAVEL[f] for f, t in zip(NAMES, DONE)), f"demo setting {DONE} inside each screw's travel")
check(abs(m.LEAD - 0.0019538) < 1e-6, f"thread {m.LEAD * 1000:.3f} mm per turn (1/2-13)")
for f in m.FEET:
    lo, hi = f.ext * 1000, (f.ext + m.LEAD * m.TRAVEL[f.name]) * 1000
    print(f"     {f.name}: bracket {lo:.1f} to {hi:.1f} mm")
# steepest fall reachable with both ends at their stops and the middle on the line: past the window's top
steep = 2 * m.LEAD * 2.1 / m.SPAN * 1000
check(steep > m.FALL_MAX, f"steepest two-end fall {steep:.1f} mm/m: overdoing it is reachable")

g, r = m.GAUGE_DEG, m.RULE_MM
check([round(g[k], 2) for k in ("half", "min", "max")] == [0.60, 1.19, 1.82] and
      [round(r[k], 1) for k in ("half", "min", "max")] == [2.6, 5.2, 7.9],
      f"real-trial readings: angle gauge {[round(g[k], 2) for k in g]} deg, rule over 25 cm {[round(r[k], 1) for k in r]} mm")

# ---- scored outcomes ---------------------------------------------------------------------------------------------
cases = [
    ("do nothing (level, middle slack)", [dict(stands=START)] * HOLD, 0.0),
    ("demo setting, arms home (25.0 mm/m)", [dict(stands=DONE)] * HOLD, 1.0),
    ("demo setting, left arm parked away", [dict(stands=DONE, grippers=AWAY)] * HOLD, 0.9),
    ("level, every stand bearing (middle brought up)", [dict(stands=stands_for(0.0))] * HOLD, 0.0),
    ("level with the whole pipe raised 3 mm", [dict(stands=stands_for(0.0, center_mm=3.0))] * HOLD, 0.0),
    ("back-fall toward the sink at 1/4 in per ft", [dict(stands=stands_for(-21.0))] * HOLD, 0.0),
    ("fall 1/4 in per ft (21 mm/m), all bearing", [dict(stands=stands_for(21.0))] * HOLD, 1.0),
    ("fall 20.5 mm/m (just under 1/4 in per ft), all bearing", [dict(stands=stands_for(20.5))] * HOLD, 0.25),
    ("fall 1/8 in per ft (10.5 mm/m), all bearing", [dict(stands=stands_for(10.5))] * HOLD, 0.25),
    ("fall 10.2 mm/m, all bearing", [dict(stands=stands_for(10.2))] * HOLD, 0.0),
    ("fall 31.5 mm/m, all bearing", [dict(stands=stands_for(31.5))] * HOLD, 1.0),
    ("fall 32.0 mm/m (more than a diameter: siphons the trap)", [dict(stands=stands_for(32.0))] * HOLD, 0.0),
    ("fall 34 mm/m, all bearing", [dict(stands=stands_for(34.0))] * HOLD, 0.0),
    ("fall 25 mm/m, middle left 1 mm low (belly)", [dict(stands=(DONE[0], START[1], DONE[2]))] * HOLD, 0.25),
    ("fall 26.7 mm/m, middle 0.4 mm low (bears)", [dict(stands=stands_for(26.7, mid_off_mm=-0.4))] * HOLD, 1.0),
    ("fall 26.7 mm/m, middle 0.6 mm low (slack)", [dict(stands=stands_for(26.7, mid_off_mm=-0.6))] * HOLD, 0.25),
    ("fall 26.7 mm/m, middle 0.2 mm high (far end 0.4 mm off: bears)",
     [dict(stands=stands_for(26.7, mid_off_mm=0.2))] * HOLD, 1.0),
    ("fall 26.7 mm/m, middle 0.25 mm high (an end lifts 0.5 mm: the feeler limit)",
     [dict(stands=stands_for(26.7, mid_off_mm=0.25))] * HOLD, 1.0),
    ("fall 26.7 mm/m, middle 0.4 mm high (an end lifts 0.8 mm)",
     [dict(stands=stands_for(26.7, mid_off_mm=0.4))] * HOLD, 0.25),
    ("saddle fall 24 mm/m, middle 0.49 mm high (pipe rocks to 20.1 mm/m, stack end 0.98 mm up: re-QC)",
     [dict(stands=stands_for(24.0, mid_off_mm=0.49))] * HOLD, 0.0),
    ("saddle fall 21 mm/m, middle 0.2 mm high (the pipe itself falls under 20.8)",
     [dict(stands=stands_for(21.0, mid_off_mm=0.2))] * HOLD, 0.25),
    ("fall 26.7 mm/m, middle 0.6 mm high (hump: an end lifts off)",
     [dict(stands=stands_for(26.7, mid_off_mm=0.6))] * HOLD, 0.25),
    ("middle 3 mm high: pipe hogs on the middle, its own fall far off the saddles'",
     [dict(stands=stands_for(26.7, mid_off_mm=3.0))] * HOLD, 0.0),
    ("level ends, middle 3 mm high (pipe rocks on the middle)",
     [dict(stands=stands_for(0.0, mid_off_mm=3.0))] * HOLD, 0.0),
    ("only the sink end raised (to its stop), middle untouched", [dict(stands=(2.1, START[1], START[2]))] * HOLD,
     0.0),
    ("only the sink end raised to its stop, middle on the line (16.4 mm/m)",
     [dict(stands=(2.1, 1.55, START[2]))] * HOLD, 0.25),
    ("only the stack end lowered to its stop, middle untouched", [dict(stands=(START[0], START[1], 0.0))] * HOLD, 0.0),
    ("both ends run to their stops, middle on the line (32.8 mm/m)", [dict(stands=(2.1, 0.5, 0.0))] * HOLD, 0.0),
    ("sink screw run past its stop (held there: 31.3 mm/m, else 33.6), middle on the line",
     [dict(stands=(2.4, 0.6, 0.2))] * HOLD, 1.0),
    ("set right, then knocked back to level", [dict(stands=DONE)] * 5 + [dict(stands=stands_for(0.0))] * HOLD, 0.0),
    ("level first, then set right", [dict(stands=stands_for(0.0))] * 5 + [dict(stands=DONE)] * HOLD, 1.0),
]
for name, steps, want in cases:
    got, ok = run(steps)
    check(abs(got - want) < 1e-9 and ok == (want == 1.0), f"{got:.2f} (want {want:.2f}) {name}")

# the scored fall matches the end stands' height difference over their spacing (the tape-measure check)
new_trial()
step(stands=DONE)
o = {m.PIPE: st(m.PIPE_POS)}
ext = lv.state(o, m.PIPE).ext_mm
f_tape = (ext["sink"] - ext["stack"]) / (m.SPAN * 1000) * 1000
check(abs(m.fall(o) - f_tape) < 0.01 and abs(f_tape - 25.01) < 0.02, f"demo fall {m.fall(o):.2f} mm/m = tape "
      f"{f_tape:.2f} (end stands {ext['sink'] - ext['stack']:.1f} mm apart over 25 cm)")

# sweep: any single screw turned on its own (the others as they started) never scores 1.0
best = 0.0
for i in range(3):
    for t in np.linspace(0.0, m.TRAVEL[NAMES[i]], 91):
        s = list(START)
        s[i] = t
        got, _ = run([dict(stands=tuple(s))] * 3)
        best = max(best, got)
check(best <= 0.25, f"turning one screw alone scores at most 0.25 ({best:.2f})")

# hands: fingers closed on a head = 0; an open hand around it is not a hold; pressing the pipe down = 0
hx, hy = lv._world(m.DRAIN, m.FEET[2].xy)
closed = {"right_gripper": 2 * m.HEAD_R / (2 * abs(FINGER_OPEN))}
got, _ = run([dict(stands=DONE)] * 3 + [dict(grippers={"right_gripper": (hx, hy, m.TOP + m.HEAD_Z)},
                                             opening=closed)] * HOLD)
check(got == 0.0, f"{got:.2f} (want 0.00) set right, fingers still closed on the stack-end head")
got, _ = run([dict(stands=DONE)] * 3 + [dict(grippers={"right_gripper": (hx, hy, m.TOP + m.HEAD_Z)})] * HOLD)
check(got == 0.0, f"{got:.2f} (want 0.00) set right, open hand left around the head")
# a tool point resting near any part of the assembly (open, not pressing) is a hand still on the job: 0
for label, p in (("over the pipe's sink end", (m.PIPE_POS[0], 0.20, m.TOP + 0.12)),
                 ("beside the level", (m.PIPE_POS[0] + 0.035, 0.075, m.TOP + 0.10)),
                 ("in front of the middle head", (m.PIPE_POS[0] + m.SCREW_X - 0.04, 0.0, m.TOP + 0.11))):
    got, _ = run([dict(stands=DONE)] * 3 + [dict(grippers={"left_gripper": p}, tips={"left_gripper": p[2] - 0.013})] *
                 HOLD)
    check(got == 0.0, f"{got:.2f} (want 0.00) set right, an open hand left {label}")
# the arms' real home tool points (READY: x 32.3, y +-30, 26 cm up) are well outside the zone
far = {"left_gripper": (0.323, 0.30, m.TOP + 0.26), "right_gripper": (0.323, -0.30, m.TOP + 0.26)}
check(not m.hands_near({k: st(v) for k, v in far.items()}), "home tool points are outside the hands-off zone")
ptop = m.TOP + m.H0 + m.PIPE_TOP
press = {"left_gripper": (m.PIPE_POS[0], 0.10, ptop + 0.013)}
got, _ = run([dict(stands=DONE)] * 3 + [dict(grippers=press, tips={"left_gripper": ptop})] * HOLD)
check(got == 0.0, f"{got:.2f} (want 0.00) set right, a hand pressing down on the pipe")
# pressing the pipe down over the slack middle stand at the start does not make it bear, and lets go to 0
got, _ = run([dict(stands=stands_for(26.7, mid_off_mm=-1.6),
                   grippers={"left_gripper": (m.PIPE_POS[0], 0.0, ptop + 0.013)}, tips={"left_gripper": ptop})] * 3 +
             [dict()] * HOLD)
check(got == 0.25, f"{got:.2f} (want 0.25) pressing the belly down then letting go: still slack")
# hovering an open hand over the pipe (above the press band) changes nothing
got, _ = run([dict(stands=START, grippers={"left_gripper": (m.PIPE_POS[0], 0.05, ptop + 0.03)},
                   tips={"left_gripper": ptop + 0.017})] * HOLD)
check(got == 0.0, f"{got:.2f} (want 0.00) hovering over the level pipe")

# history reset with kit.episode()
run([dict(stands=DONE, grippers=AWAY)] * 5)
kit._EPISODE["count"] += 1
for k in KEYS.values():
    tk._TRACK.pop(k, None)
step(grippers=AWAY)  # the new trial starts with the left arm at AWAY: that is its home now
for _ in range(3):
    got, _ = step(stands=DONE, grippers=AWAY)
check(got == 1.0, f"{got:.2f} (want 1.00) home re-recorded after a scene reset")
new_trial()
o = {m.PIPE: st(m.PIPE_POS)}
check(lv.touched_feet(o, m.PIPE) == () and abs(m.fall(o)) < 1e-9, "after a reset the stands are back at the start")

# ---- adversarial QC (Nov 11) -------------------------------------------------------------------------------------
# cheap partial credit: any one screw turned anywhere in its travel, the others as they started, scores 0
best = 0.0
for i in range(3):
    for t in np.linspace(0.0, m.TRAVEL[NAMES[i]], 211):
        s = list(START)
        s[i] = t
        best = max(best, run([dict(stands=tuple(s))] * 2)[0])
check(best == 0.0, f"{best:.2f} (want 0.00) any single screw alone, finely swept")
# the stack screw alone, run in 1.34 to 1.51 turns: a 10.4 to 11.8 mm/m saddle fall, but the middle sits 0.3 to 0.5 mm
# above the line, so the pipe rocks on it (7.8 mm/m, stack end 0.7 to 1 mm off its saddle): 0
got, _ = run([dict(stands=(START[0], START[1], START[2] - 1.42))] * HOLD)
check(got == 0.0, f"{got:.2f} (want 0.00) only the stack screw, run in 1.42 turns (pipe rocks on the middle)")
# do nothing while the arms wander and come back: still 0
got, _ = run([dict(grippers=AWAY)] * 5 + [dict()] * HOLD)
check(got == 0.0, f"{got:.2f} (want 0.00) do nothing, arms wander and return")
# release is required, but a hold earlier in the run is not held against the end state
got, _ = run([dict(stands=DONE, grippers={"right_gripper": (hx, hy, m.TOP + m.HEAD_Z)}, opening=closed)] * 5 +
             [dict(stands=DONE)] * HOLD)
check(got == 1.0, f"{got:.2f} (want 1.00) set right while holding a head, then released and home")
# fingers closed on a head on the very last step only: 0, and success() agrees
got, ok = run([dict(stands=DONE)] * HOLD + [dict(grippers={"right_gripper": (hx, hy, m.TOP + m.HEAD_Z)},
                                                 opening=closed)])
check(got == 0.0 and not ok, f"{got:.2f} (want 0.00) set right, fingers closed on a head at the final step")
# a hand pressing the pipe to fake bearing on a slack middle stand: 0 while pressing
got, _ = run([dict(stands=(DONE[0], START[1], DONE[2]),
                   grippers={"left_gripper": (m.PIPE_POS[0], 0.0, ptop + 0.013)}, tips={"left_gripper": ptop})] * HOLD)
check(got == 0.0, f"{got:.2f} (want 0.00) belly pressed down by hand at the end")
# a full trial, then a reset with nothing done: the new trial scores 0 (no carried-over home or stands)
run([dict(stands=DONE)] * HOLD)
got, ok = new_trial()
for _ in range(HOLD):
    got, ok = step()
check(got == 0.0 and not ok, f"{got:.2f} (want 0.00) reset after a full-credit trial, then nothing done")

# seesaw attack (independent QC, Nov 11): only the stack screw run in 1.25 turns leaves the middle saddle above the
# end-to-end line; a press on the pipe at y -8 cm rocks the model onto middle + stack and it stays there after release
# (the model reads 11.7 mm/m with every gap under 0.5). The score reads the end stands: 9.8 mm/m, under the half fall
seesaw = (START[0], START[1], START[2] - 1.25)
topz = m.TOP + m.H0 + m.PIPE_TOP
got, ok = run([dict(stands=seesaw)] * 10 +
              [dict(grippers={"right_gripper": (m.PIPE_POS[0], -0.08, topz + 0.013)}, tips={"right_gripper": topz})] * 5
              + [dict()] * 30)
o = {m.PIPE: st(m.PIPE_POS)}
check(got == 0.0 and not ok and abs(m.fall(o) - 9.77) < 0.05 and abs(m.middle_off_line_mm(o) - 0.22) < 0.03,
      f"{got:.2f} (want 0.00) one screw plus a push to rock the pipe onto the middle stand (fall {m.fall(o):.1f}, "
      f"middle {m.middle_off_line_mm(o):+.2f} mm off the line, model gaps "
      f"{ {k: round(v, 2) for k, v in lv.gaps_mm(o, m.PIPE).items()} })")
# the same push after any single screw anywhere in its travel: never above 0
best = 0.0
for i in range(3):
    for t in np.linspace(0.0, m.TRAVEL[NAMES[i]], 43):
        s = list(START)
        s[i] = t
        for py in (-0.08, 0.08):
            push = {"right_gripper": (m.PIPE_POS[0], py, topz + 0.013)}
            got, _ = run([dict(stands=tuple(s))] * 2 + [dict(grippers=push, tips={"right_gripper": topz})] * 3 +
                         [dict()] * 3)
            best = max(best, got)
check(best == 0.0, f"{best:.2f} (want 0.00) any single screw, then a press at either end and release")

# the demo: ends home, fits in 600 steps, turns match DEMO_TURN, strokes within +-45 deg of yaw 0, pads over the pipe
d = m.demo()
check(all(isinstance(d[a][-1], kit.Home) for a in ("left", "right")), "demo ends with both arms home")
dur = max(sum(w.duration for w in d[a]) for a in ("left", "right"))
check(dur * 20 < 880, f"demo {dur:.1f} s = {dur * 20:.0f} steps (< 880, room for the 1 s hold and lag in 1000)")
yaws = [math.degrees(w.yaw) for w in d["left"] + d["right"] if not isinstance(w, kit.Home)]
check(max(abs(y) for y in yaws) <= 60.0 + 1e-6, f"head strokes within +-60 deg of yaw 0 ({min(yaws):.0f} to "
      f"{max(yaws):.0f})")
rolls = [w for w in d["left"] + d["right"] if not isinstance(w, kit.Home) and "roll" in w.label]
pipe_top_max = m.TOP + m.H0 + 1.6 * m.LEAD + m.PIPE_TOP  # the sink end's pipe top at the demo's end
check(min(w.pos[2] for w in rolls) - m.PAD_TIP_PAST_TCP > pipe_top_max + 0.005,
      "head grips: pad tips more than 5 mm above the pipe's highest top")
left_y = [w.pos[1] for w in d["left"] if not isinstance(w, kit.Home)]
right_y = [w.pos[1] for w in d["right"] if not isinstance(w, kit.Home)]
check(min(right_y) < -0.12 and max(right_y) < -0.12, "the right arm stays outside 12 cm of y = 0")
# replay the demo's turns through the model: the end state scores 1.0
for f, deg in m.DEMO_TURN.items():
    check(abs(deg / 360.0 - (DONE[NAMES.index(f)] - m.START[f])) < 1e-9, f"demo turns {f} {deg:+.0f} deg")
pl = lv.plan(m.DRAIN, target=(0.0, math.degrees(math.atan(2 * 1.6 * m.LEAD / m.SPAN))))
check(abs(pl["sink"] - pl["stack"] - 1152.0) < 1.0, f"lv.plan agrees: sink - stack = {pl['sink'] - pl['stack']:.0f} "
      "deg (3.2 turns between the ends)")

print("all pass" if not fails else f"{fails} FAILED")
