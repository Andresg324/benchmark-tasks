"""Offline checks of the brake-bleeding scorer: step-by-step pedal, wrench and gripper histories run through the livekit
pressure network, near misses included, plus a replay of the scripted demo's timing. No simulator.

    PY=R/venv/bin/python R/yam-kits/chat/offline/runtest.sh <this file>      (prints "all pass")
"""

import importlib.util
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
_spec = importlib.util.spec_from_file_location("brake_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit, lk = m.kit, m.lk
from yam_arena.task_file import ObjectState  # noqa: E402

PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
AWAY = {"left_gripper": (0.25, 0.20, m.TOP + 0.15)}  # done, but the left arm parked away from home
DOWN, UP_Q = -m.TRAVEL, 0.0  # pedal on the floor / up
SHUT, OPEN = m.SHUT, m.OPEN
fails = 0


def st(pos):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def check(cond, what):
    global fails
    print(("ok  " if cond else "FAIL") + " " + what)
    fails += not cond


def new_trial():
    kit._EPISODE["count"] += 1


def step(pedal, wrench, grippers=None, pvel=0.0, wvel=0.0):
    """One control step: pedal angle (0 up, -TRAVEL on the floor), wrench angle (0 shut, pi/2 a quarter turn open),
    gripper tool points. Runs the livekit network as the hook would, then success() and score()."""
    kit._JOINTS["pedal_stand/pedal"] = kit.JointState(pedal, pvel, False)
    kit._JOINTS["caliper/wrench"] = kit.JointState(wrench, wvel, False)
    g = dict(PARK, **(grippers or {}))
    objs = {"pedal_stand": st((*m.PIVOT[:2], m.TOP)), "caliper": st((*m.CAL, m.TOP)), "jar": st((*m.JAR, m.TOP)),
            **{k: st(p) for k, p in g.items()}}
    lk.update(objs)
    ok = m.success(objs)
    return m.score(objs), ok


def run(phases):
    """Phases of (steps, pedal, wrench, grippers); returns (score, success) at the last step."""
    out = None
    for n, p, w, grp in phases:
        for _ in range(n):
            out = step(p, w, grp)
    return out


def cycle(open_deg=90.0, release_open=False, hold_s=1.0, grp=None):
    """One pedal-method cycle: press, open the bleeder (ramped over 1.8 s), hold, then either shut it and let the
    pedal up (right) or let the pedal up first and shut it after (the mistake)."""
    ph = [(10, DOWN, SHUT, grp)]
    q = math.radians(open_deg)
    ph += [(4, DOWN, q * i / 9, grp) for i in range(1, 10)]
    ph += [(int(hold_s * 20), DOWN, q, grp)]
    if release_open:
        ph += [(25, UP_Q, q, grp), (10, UP_Q, SHUT, grp)]
    else:
        ph += [(4, DOWN, q * i / 9, grp) for i in range(8, -1, -1)] + [(25, UP_Q, SHUT, grp)]
    return ph


START = [(10, UP_Q, SHUT, None)]


def trial(phases, end=(30, UP_Q, SHUT, None)):
    new_trial()
    return run(START + phases + ([end] if end else []))


# --- the network itself
new_trial()
run(START)
check(abs(m.air(None) - m.AIR0) < 1e-6, f"start: {m.air(None):.2f} mL of air in the caliper")
airs = [m.air(None)]
for _ in range(5):
    run(cycle())
    airs.append(m.air(None))
check(all(b < a for a, b in zip(airs, airs[1:])), "every right-order cycle takes air out: "
      + ", ".join(f"{a:.2f}" for a in airs))
check(airs[4] <= m.FIRM_AIR < airs[3], f"four cycles reach a firm pedal ({airs[4]:.3f} <= {m.FIRM_AIR}), three don't")
check(lk.hazard(None) is None and lk.inflow(None, "caliper", "gas") < 1e-9, "no air came back in, no hazard")
check(lk.order_ok(None, ["open:bleeder", "close:bleeder", "open:bleeder", "close:bleeder"]),
      "the log shows the bleeder opening and shutting each cycle")

# the threshold: how far open the bleeder can be when the pedal comes up before air is drawn in (the task's rule:
# past 20 degrees, where it starts passing fluid)
safe = []
for deg in (10, 15, 19, 21, 25, 30, 35, 40, 90):
    new_trial()
    run(START + cycle(open_deg=deg, release_open=True))
    safe.append((deg, lk.hazard(None, "air_in") is None))
check(all(ok for d, ok in safe if d < 20) and not any(ok for d, ok in safe if d > 20),
      "pedal up with the bleeder open: no hazard under 20 degrees, air_in past 20 " + str(safe))
new_trial()
run(START + [(10, DOWN, SHUT, None), (5, UP_Q, SHUT, None), (3, UP_Q, math.radians(30), None),
             (20, UP_Q, SHUT, None)])
check(lk.hazard(None, "air_in") is not None, "bleeder opened past 20 degrees during the pedal's 0.8 s return: air_in")
new_trial()
run(START + [(10, DOWN, SHUT, None), (20, UP_Q, SHUT, None), (5, UP_Q, math.radians(30), None),
             (20, UP_Q, SHUT, None)])
check(lk.hazard(None, "air_in") is None, "bleeder opened after the pedal's return is over: no hazard")

# repeated wrong-order cycles plateau above a firm pedal (the physics itself denies 1.0; the latch decides 0)
new_trial()
run(START)
for _ in range(10):
    run(cycle(release_open=True))
check(m.air(None) > 2 * m.FIRM_AIR, f"ten pedal-up-first cycles leave {m.air(None):.2f} mL: never firm")

cases = [
    ("do nothing", [], 0.0),
    ("pump the pedal ten times, bleeder shut", [(20, DOWN, SHUT, None), (20, UP_Q, SHUT, None)] * 10, 0.0),
    ("open the bleeder and shut it, no pedal", [(40, UP_Q, OPEN, None), (20, UP_Q, SHUT, None)], 0.0),
    ("one right cycle (0.76 mL left): real progress", cycle(), 0.25),
    ("three right cycles (0.31 mL): still spongy", cycle() * 3, 0.25),
    ("four right cycles, arms home", cycle() * 4, 1.0),
    ("five right cycles, arms home", cycle() * 5, 1.0),
    ("five right cycles, left arm not home", cycle(grp=None) * 5 + [(30, UP_Q, SHUT, AWAY)], 0.9),
    ("CORE MISTAKE: pedal up with the bleeder open, once", cycle(release_open=True), 0.0),
    ("CORE MISTAKE on the first cycle, then five right ones (latched)",
     cycle(release_open=True) + cycle() * 5, 0.0),
    ("five right cycles, then one mistake at the end (latched)", cycle() * 5 + cycle(release_open=True), 0.0),
    ("five right cycles, bleeder left open at the end", cycle() * 5 + [(30, UP_Q, OPEN, None)], 0.0),
    ("five right cycles, bleeder left 15 degrees open at the end (outside its detent)",
     cycle() * 5 + [(30, UP_Q, math.radians(15), None)], 0.0),
    ("five right cycles, bleeder 8 degrees from shut at the end (inside its detent)",
     cycle() * 5 + [(30, UP_Q, math.radians(8), None)], 1.0),
    ("five right cycles, pedal still held down at the end", cycle() * 5 + [(30, DOWN, SHUT, None)], 0.0),
    ("five cycles with the bleeder only cracked 25 degrees, shut each time: a little air out, not firm",
     cycle(open_deg=25) * 5, 0.25),
    ("EXPLOIT: bleeder held cracked 30 degrees, pedal pumped 25 times, then shut",
     [(5, UP_Q, math.radians(30), None)] + [(10, DOWN, math.radians(30), None), (10, UP_Q, math.radians(30), None)] * 25
     + [(10, UP_Q, SHUT, None)], 0.0),
    ("EXPLOIT: bleeder held cracked 35 degrees, pedal pumped 25 times, then shut",
     [(5, UP_Q, math.radians(35), None)] + [(10, DOWN, math.radians(35), None), (10, UP_Q, math.radians(35), None)] * 25
     + [(10, UP_Q, SHUT, None)], 0.0),
]
ENDS = ("not home", "at the end")  # these cases' last phase is the end state itself
for name, phases, want in cases:
    got, ok = trial(phases, end=None if any(e in name for e in ENDS) else (30, UP_Q, SHUT, None))
    check(abs(got - want) < 1e-9 and ok == (want == 1.0), f"{got:.2f} (want {want:.2f}) {name}")

# hands: still on the pedal pad or on the wrench handle; let go nearby
new_trial()
run(START + cycle() * 5)
pad_top = tuple(m.pad_points({})[4])
got, _ = run([(30, UP_Q, SHUT, {"left_gripper": pad_top})])
check(got == 0.0, f"{got:.2f} (want 0.00) bled, a hand resting on the pedal pad")
new_trial()
run(START + cycle() * 5)
tip = tuple(m.wrench_points({})[-1])
got, _ = run([(30, UP_Q, SHUT, {"right_gripper": tip})])
check(got == 0.0, f"{got:.2f} (want 0.00) bled, a hand still on the wrench handle")
new_trial()
run(START + cycle() * 5)
got, _ = run([(30, UP_Q, SHUT, {"right_gripper": tuple(np.array(tip) + [0, -0.05, 0.03])})])
check(got == 0.9, f"{got:.2f} (want 0.90) bled, the right hand let go 5 cm from the handle (not home)")

# moving parts at the end
new_trial()
run(START + cycle() * 5)
got, _ = step(UP_Q, SHUT, pvel=0.5)
check(got == 0.0, f"{got:.2f} (want 0.00) pedal still swinging at 0.5 rad/s")
got, _ = step(UP_Q, SHUT, wvel=0.5)
check(got == 0.0, f"{got:.2f} (want 0.00) wrench still swinging at 0.5 rad/s")

# cheap tricks: hovering over the pedal, knocking the wrench, half presses
got, _ = trial([(60, UP_Q, SHUT, {"left_gripper": tuple(np.array(pad_top) + [0, 0, 0.05])})])
check(got == 0.0, f"{got:.2f} (want 0.00) hovering over the pedal")
got, _ = trial([(20, UP_Q, math.radians(30), None), (20, UP_Q, SHUT, None)] * 5)
check(got == 0.0 and lk.hazard(None) is None, f"{got:.2f} (want 0.00) wrench knocked 30 degrees and back, five times")
got, _ = trial([(20, -0.5 * m.TRAVEL, SHUT, None), (5, -0.5 * m.TRAVEL, OPEN, None), (20, -0.5 * m.TRAVEL, OPEN, None),
                (5, -0.5 * m.TRAVEL, SHUT, None), (20, UP_Q, SHUT, None)] * 5)
check(got == 0.0, f"{got:.2f} (want 0.00) half presses (under 70% of the stroke) with the bleeder worked")

# episode reset: a hazard trial does not carry over; home re-recorded
trial(cycle(release_open=True))
check(lk.hazard(None, "air_in") is not None, "the mistake trial logged air_in")
got, ok = trial(cycle() * 5)
check(got == 1.0 and ok and abs(lk.gas(None, "caliper") - 0.122) < 0.03,
      f"{got:.2f} (want 1.00) next trial starts clean, with the air back at 1.2 mL")
new_trial()
step(UP_Q, SHUT, AWAY)  # this trial starts with the left arm at AWAY: that is its home now
run([(n, p, w, AWAY) for n, p, w, _ in cycle() * 5])
got, _ = run([(30, UP_Q, SHUT, AWAY)])
check(got == 1.0, f"{got:.2f} (want 1.00) home re-recorded after a scene reset")

# --- QC review (Nov 10): strokes, partial returns, same-step releases, one-step pops
ph = []
for _ in range(5):
    ph += [(10, DOWN, SHUT, None), (40, DOWN, OPEN, None), (20, 0.71 * DOWN, OPEN, None), (5, 0.71 * DOWN, SHUT, None),
           (25, UP_Q, SHUT, None)]
got, _ = trial(ph)
check(got == 0.0 and lk.hazard(None, "air_in") is not None,
      f"{got:.2f} (want 0) pedal let 29% back up from the floor with the bleeder open (still past 70%): air_in")
ph = [(10, 0.71 * DOWN, SHUT, None)]
for _ in range(5):
    ph += [(2, 0.69 * DOWN, SHUT, None), (5, 0.71 * DOWN, SHUT, None), (40, 0.71 * DOWN, OPEN, None),
           (5, 0.71 * DOWN, SHUT, None)]
got, _ = trial(ph + [(25, UP_Q, SHUT, None)])
check(got == 0.0 and m.air(None) > m.PART_AIR,
      f"{got:.2f} (want 0) 69%-71% wiggles are not new pumps ({m.air(None):.2f} mL left)")
ph = []
for _ in range(5):
    ph += [(10, DOWN, SHUT, None), (40, DOWN, OPEN, None), (1, UP_Q, SHUT, None), (25, UP_Q, SHUT, None)]
got, _ = trial(ph)
check(got == 0.0 and lk.hazard(None, "air_in") is not None,
      f"{got:.2f} (want 0) wrench snapped shut on the same step the pedal came up: air_in")
ph = []
for _ in range(5):
    ph += [(10, DOWN, SHUT, None), (40, DOWN, OPEN, None), (1, DOWN, math.radians(21), None),
           (1, UP_Q, math.radians(19), None), (25, UP_Q, SHUT, None)]
got, _ = trial(ph)
check(got == 0.0 and lk.hazard(None, "air_in") is not None,
      f"{got:.2f} (want 0) wrench at 21 deg the step before the pedal rose: air_in")
ph = cycle() * 4 + [(10, DOWN, SHUT, None), (20, DOWN, OPEN, None), (1, 0.69 * DOWN, OPEN, None),
                    (20, DOWN, OPEN, None), (5, DOWN, SHUT, None), (25, UP_Q, SHUT, None)]
got, ok = trial(ph)
check(got == 1.0 and ok and lk.hazard(None) is None,
      f"{got:.2f} (want 1.00) a one-step pop above 70% with the bleeder open is not a return")

# --- the scripted demo, replayed: pedal angle from the left tool point's height, wrench angle from the right tool
# point's bearing about the bleeder (lagging 0.06 rad behind, as the straddling fingers' slop lets it)
d = m.demo()
check(isinstance(d["left"][-1], kit.Home) and isinstance(d["right"][-1], kit.Home), "demo: both arms end at home")
TL, TR = sum(w.duration for w in d["left"]), sum(w.duration for w in d["right"])
check(abs(TL - TR) < 1e-6 and TL * 20 < 1200, f"demo: both arms end together, {TL * 20:.0f} steps (< 1,200)")


def sample(wps, t, home):
    """Linear interpolation of tool point and finger between waypoints (Home = the parked start)."""
    prev = np.array(home, float)
    t0 = 0.0
    for w in wps:
        tgt = np.array(home, float) if isinstance(w, kit.Home) else np.array(w.pos, float)
        if t <= t0 + w.duration:
            return prev + (tgt - prev) * (t - t0) / w.duration
        prev, t0 = tgt, t0 + w.duration
    return prev


pad_rest_top = m.PIVOT[2] + m.PAD_DZ + m.PAD[2] / 2
new_trial()
wq, pq, got, ok = SHUT, 0.0, None, False
sw_max = 0.0
for k in range(int(TL * 20) + 30):
    t = k * 0.05
    L = sample(d["left"], t, PARK["left_gripper"])
    Rr = sample(d["right"], t, PARK["right_gripper"])
    tip_z = L[2] - m.TOP - m.PAD_TIP_PAST_TCP
    near_pad = abs(L[0] - m._press_xy()[0]) < 0.03 and abs(L[1] - m.PIVOT[1]) < 0.03
    pq = max(-m.TRAVEL, min(0.0, -math.asin(min(1.0, max(0.0, pad_rest_top - tip_z) / m.LEVER)))) if near_pad else 0.0
    rb = Rr[:2] - np.array(m.BLEED)
    if np.linalg.norm(rb) < 0.10 and Rr[2] - m.TOP < m.WRENCH_Z + 0.03:
        bearing = math.atan2(rb[0], -rb[1])  # 0 along -y, +pi/2 along +x
        wq = min(max(wq, bearing - 0.06), bearing + 0.06)  # pushed by whichever finger leads
        wq = min(max(wq, SHUT - 0.05), OPEN + 0.05)
    sw_max = max(sw_max, wq)
    got, ok = step(pq, wq, {"left_gripper": tuple(L), "right_gripper": tuple(Rr)})
check(sw_max > OPEN - m.SHUT_TOL, f"demo replay: the wrench reaches the open detent ({math.degrees(sw_max):.0f} deg)")
check(lk.hazard(None) is None, "demo replay: no air drawn back in (the bleeder always shut before the pedal came up)")
check(got == 1.0 and ok, f"{got:.2f} (want 1.00) demo replay ends bled and home ({m.air(None):.3f} mL left)")
check(lk.import_order_ok(), "livekit was imported last")

print("all pass" if not fails else f"{fails} FAILED")
