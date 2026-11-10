"""Offline checks of the pallet-jack task's scorer: step-by-step histories of the jack (pivot position, fork lift, handle,
pedal, the pump's memory), the pallet (on the forks or loose) and the grippers, with near misses and cheap tricks. No
simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
TOP = m.TOP
PARK = {"left_gripper": (0.32, 0.305, TOP + 0.26), "right_gripper": (0.32, -0.305, TOP + 0.26)}
X, Y0 = m.JACK_X, m.PIVOT_Y0
Y1 = Y0 - m.TRAVEL
GAP = m.STRINGER_H - m.FORK_Z1  # the forks rise this far before they touch the deck
SAG = 0.0011  # the loaded forks sit 1.1 mm below the pump's level (measured in the sim)
FULL = m.LIFT


def yaw_q(yaw):
    return np.array([0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)])


def tilted(deg, yaw):
    """Yaw, then tipped `deg` about the world x axis."""
    a = math.radians(deg) / 2
    return np.array(kit._quat_mul((math.sin(a), 0, 0, math.cos(a)), tuple(yaw_q(yaw))))


def st(pos, q, half=0.01):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - half,
                       high=p + half)


def set_joints(lift, handle, pedal, level):
    kit._JOINTS["jack/forks"] = kit.JointState(lift, 0.0, False)
    kit._JOINTS["jack/handle"] = kit.JointState(handle, 0.0, False)
    kit._JOINTS["jack/pedal"] = kit.JointState(pedal, 0.0, False)
    kit._LAW_MEM["jack/forks"] = (kit._EPISODE["count"], {"start": 0.0, "level": level, "strokes": 0})


def grip_on_handle(pivot_y, handle):
    th = -handle
    return (X, pivot_y + math.sin(th) * m.HANDLE_L, TOP + m.PIVOT_Z + math.cos(th) * m.HANDLE_L)


def pedal_point(pivot_y):
    return (X + m.PEDAL_AT[1] - m.PEDAL_ARM, pivot_y - m.PEDAL_AT[0], TOP + m.PEDAL_AT[2] + 0.002 + m.PAD_TIP_PAST_TCP)


def step(f):
    """One control step from a dict: y (pivot), lift (fork joint), level (pump memory, default = lift), handle, pedal,
    pallet ("on" = seated on the forks, or (pos, tilt degrees)), grips (name -> pos (open), (pos, opening), "hold" =
    closed on the grip bar, "pedal" = closed fist on the pedal), dx (jack sideways offset)."""
    y, lift = f.get("y", Y0), f.get("lift", 0.0)
    level = f.get("level", lift)
    handle, pedal = f.get("handle", 0.0), f.get("pedal", 0.0)
    set_joints(lift, handle, pedal, level)
    jp = np.array([X + f.get("dx", 0.0), y, TOP])
    jq = yaw_q(m.JACK_YAW)
    pal = f.get("pallet", "on")
    if isinstance(pal, str):
        pp = jp + m._rot(jq) @ np.array([m.PALLET_X, 0.0, 0.0005 + max(0.0, lift - GAP - SAG)])
        pq = jq
    else:
        pp, pq = np.asarray(pal[0], float), tilted(pal[1], m.JACK_YAW)
    objs = {"jack": st(jp, jq), "pallet": st(pp, pq), "bay": st((*m.BAY, TOP), yaw_q(0))}
    g = dict(PARK, **(f.get("grips") or {}))
    for k, v in g.items():
        if v == "hold":
            pos, op = grip_on_handle(y, handle), 0.21
        elif v == "pedal":
            pos, op = pedal_point(y), 0.0
        elif isinstance(v[0], tuple):
            pos, op = v
        else:
            pos, op = v, 1.0
        p = np.array(pos, float)
        objs[k] = ObjectState(pos=p, quat_xyzw=yaw_q(0), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                              low=p - [0, 0, 0.013], high=p)
        kit._OPENING[k] = op
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames, new_trial=True):
    """frames: list of (steps, dict). A fresh trial each time."""
    if new_trial:
        kit._EPISODE["count"] += 1
    out = step({})  # the trial's first scored step: the arms at their ready pose
    for n, f in frames:
        for _ in range(n):
            out = step(f)
    return out


def ramp(a, b, n=10):
    return [a + (b - a) * k / n for k in range(1, n + 1)]


H = {"left_gripper": "hold"}
P = {"left_gripper": "pedal"}


def pump(strokes, y=Y0, lift0=0.0, per=m.PER_STROKE):
    """Pull the handle back and return it `strokes` times; each return raises the forks one stroke."""
    fr, lift = [], lift0
    for _ in range(strokes):
        fr += [(1, dict(y=y, lift=lift, handle=h, grips=H)) for h in ramp(0.0, -0.65, 4)]
        lift = min(FULL, lift + per)
        fr += [(1, dict(y=y, lift=lift, handle=h, grips=H)) for h in ramp(-0.65, 0.0, 4)]
    return fr


def roll(y_from, y_to, lift, n=20, **kw):
    return [(1, dict(y=y, lift=lift, grips=H, **kw)) for y in ramp(y_from, y_to, n)]


def lower(y, lift_from, n=10, pedal=0.3):
    return [(1, dict(y=y, lift=l, pedal=pedal, grips=P)) for l in ramp(lift_from, 0.0, n)]


def nudged(d):
    """The pallet pushed `d` along the floor toward the bay with the forks down, then pumped up there."""
    fr = [(1, dict(y=y, grips=H, pallet=((X, y - m.PALLET_X, TOP + 0.0005), 0.0))) for y in ramp(Y0, Y0 - d, 6)]
    return fr + [(f[0], dict(f[1], y=f[1]["y"] - d)) for f in pump(4)]


LET_GO = [(40, dict(y=Y1))]
GOOD = pump(4) + roll(Y0, Y1, FULL) + [(3, dict(y=Y1, lift=FULL))] + lower(Y1, FULL) + LET_GO
END_DOWN = dict(y=Y1)


def dragged_with(lift):
    """Roll to the bay with the forks at `lift` (pallet on the floor if lift <= GAP), then 'lower' and let go."""
    off = max(0.0, lift - GAP)
    fr = [(1, dict(y=y, lift=lift, grips=H, pallet=((X, y - m.PALLET_X, TOP + 0.0005 + off), 0.0)))
          for y in ramp(Y0, Y1, 20)]
    return fr + lower(Y1, lift) + LET_GO


cases = [
    ("nothing", [(100, {})], 0.0),
    ("the whole job, arms home", GOOD, 1.0),
    ("the whole job, left arm parked above the jack, fingers open", GOOD[:-1]
     + [(40, dict(y=Y1, grips={"left_gripper": ((X, Y1, TOP + 0.20), 1.0)}))], 0.9),
    ("the whole job, left arm parked far off, fingers closed", GOOD[:-1]
     + [(40, dict(y=Y1, grips={"left_gripper": ((0.20, 0.25, TOP + 0.20), 0.0)}))], 0.9),
    ("forks left raised in the bay, handle upright, let go (the trap)", pump(4) + roll(Y0, Y1, FULL)
     + [(40, dict(y=Y1, lift=FULL))], 0.25),
    ("forks lowered only halfway", pump(4) + roll(Y0, Y1, FULL) + lower(Y1, FULL, 5)[:3]
     + [(40, dict(y=Y1, lift=FULL * 0.4))], 0.25),
    ("pump memory says down but the fork joint still 5 mm up", pump(4) + roll(Y0, Y1, FULL)
     + [(40, dict(y=Y1, lift=0.005, level=0.0))], 0.25),
    ("fork joint down but the pump memory still raised", pump(4) + roll(Y0, Y1, FULL)
     + [(40, dict(y=Y1, lift=0.0, level=FULL))], 0.25),
    ("pedal still held down by a closed fist at the end", pump(4) + roll(Y0, Y1, FULL) + lower(Y1, FULL)
     + [(40, dict(y=Y1, pedal=0.3, grips=P))], 0.5),
    ("pedal pressed by an open gripper resting on it at the end", pump(4) + roll(Y0, Y1, FULL) + lower(Y1, FULL)
     + [(40, dict(y=Y1, pedal=0.3, grips={"left_gripper": (pedal_point(Y1), 1.0)}))], 0.5),
    ("handle still gripped at the end", pump(4) + roll(Y0, Y1, FULL) + lower(Y1, FULL)
     + [(40, dict(y=Y1, grips=H))], 0.5),
    ("handle left pulled back 30 degrees", pump(4) + roll(Y0, Y1, FULL) + lower(Y1, FULL)
     + [(40, dict(y=Y1, handle=-math.radians(30)))], 0.5),
    ("handle let fall flat on the floor", pump(4) + roll(Y0, Y1, FULL) + lower(Y1, FULL)
     + [(40, dict(y=Y1, handle=-1.35))], 0.5),
    ("handle 6 degrees back (inside its upright zone)", pump(4) + roll(Y0, Y1, FULL) + lower(Y1, FULL)
     + [(40, dict(y=Y1, handle=-math.radians(6)))], 1.0),
    ("the whole job but the jack still rolling back out from under the pallet at the end", GOOD[:-1]
     + [(1, dict(y=Y1 + 0.002 * k, pallet=((X, Y1 - m.PALLET_X, TOP + 0.0005), 0.0))) for k in range(1, 20)], 0.5),
    ("forks never raised: the jack drags the pallet into the bay on the floor", dragged_with(0.0), 0.0),
    ("one pump stroke only (forks touch the deck, pallet still on the floor), rolled", pump(1)
     + dragged_with(m.PER_STROKE), 0.0),
    ("two strokes: pallet 2.4 mm up after the forks' 1.1 mm sag, rolled, then lowered", pump(2)
     + roll(Y0, Y1, 2 * m.PER_STROKE) + lower(Y1, 2 * m.PER_STROKE) + LET_GO, 1.0),
    ("pallet dragged 1.7 cm TOWARD the bay before it lifts (under NUDGE, over NUDGE_TOWARD), then the whole job",
     nudged(m.NUDGE - 0.001) + roll(Y0 - m.NUDGE + 0.001, Y1, FULL) + lower(Y1, FULL) + LET_GO, 0.0),
    ("pallet rocked 4 mm toward the bay by the pump strokes (under NUDGE_TOWARD), then the whole job",
     nudged(m.NUDGE_TOWARD - 0.001) + roll(Y0 - m.NUDGE_TOWARD + 0.001, Y1, FULL) + lower(Y1, FULL) + LET_GO, 1.0),
    ("pallet rocked 1.7 cm BACK toward the operator by the pump strokes (under NUDGE), then the whole job",
     nudged(-(m.NUDGE - 0.001)) + roll(Y0 + m.NUDGE - 0.001, Y1, FULL) + lower(Y1, FULL) + LET_GO, 1.0),
    ("pallet pushed 1.9 cm on the floor before the first lift (past NUDGE), then the whole job",
     nudged(0.019) + roll(Y0 - 0.019, Y1, FULL) + lower(Y1, FULL) + LET_GO, 0.0),
    ("forks touching the deck, pallet 0.9 mm off the floor (under CLEAR: still dragging), rolled", pump(2)
     + roll(Y0, Y1, GAP + SAG + 0.0004) + lower(Y1, GAP + SAG + 0.0004) + LET_GO, 0.0),
    ("carried to 3.5 cm short of the bay, lowered there, let go (no drag)", pump(4) + roll(Y0, Y1 + 0.035, FULL)
     + lower(Y1 + 0.035, FULL) + [(40, dict(y=Y1 + 0.035))], 0.25),
    ("pallet pushed 1.9 cm on the floor before lifting and 1.9 cm after lowering",
     [(1, dict(y=y, grips=H, pallet=((X, y - m.PALLET_X, TOP + 0.0005), 0.0))) for y in ramp(Y0, Y0 - 0.019, 6)]
     + [(f[0], dict(f[1], y=f[1]["y"] - 0.019)) for f in pump(4)]
     + roll(Y0 - 0.019, Y1 + 0.019, FULL) + lower(Y1 + 0.019, FULL)
     + [(1, dict(y=y, grips=H, pallet=((X, y - m.PALLET_X, TOP + 0.0005), 0.0))) for y in ramp(Y1 + 0.019, Y1, 6)]
     + LET_GO, 0.0),
    ("carried to 3.5 cm short, lowered, then pushed 1.9 cm into the bay with the forks down", pump(4)
     + roll(Y0, Y1 + 0.035, FULL) + lower(Y1 + 0.035, FULL)
     + [(1, dict(y=y, grips=H, pallet=((X, y - m.PALLET_X, TOP + 0.0005), 0.0))) for y in ramp(Y1 + 0.035, Y1 + 0.016, 6)]
     + [(40, dict(y=Y1 + 0.016))], 0.0),
    ("pallet settles 3 mm as it is set down in the bay", pump(4) + roll(Y0, Y1, FULL) + lower(Y1, FULL)
     + [(40, dict(y=Y1, pallet=((X + 0.003, Y1 - m.PALLET_X, TOP + 0.0005), 0.0)))], 1.0),
    ("set down twice, sliding 3 mm on the floor each time (the slides add up; a lift doesn't reset them)", pump(4)
     + roll(Y0, Y1 + 0.03, FULL) + lower(Y1 + 0.03, FULL)
     + [(3, dict(y=Y1 + 0.03, pallet=((X + 0.003, Y1 + 0.03 - m.PALLET_X, TOP + 0.0005), 0.0)))]
     + pump(4, y=Y1 + 0.03) + roll(Y1 + 0.03, Y1, FULL) + lower(Y1, FULL)
     + [(40, dict(y=Y1, pallet=((X + 0.003, Y1 - m.PALLET_X, TOP + 0.0005), 0.0)))], 0.0),
    ("carried 3 cm away from the bay and lowered", pump(4) + roll(Y0, Y0 + 0.03, FULL, 6) + lower(Y0 + 0.03, FULL)
     + [(40, dict(y=Y0 + 0.03))], 0.0),
    ("pallet dragged 3 cm on the floor first, then the whole job (irreversible)",
     [(1, dict(y=y, pallet=((X, y - m.PALLET_X, TOP + 0.0005), 0.0))) for y in ramp(Y0, Y0 - 0.03, 5)]
     + [(f[0], dict(f[1], y=f[1]["y"] - 0.03)) for f in pump(4)]
     + roll(Y0 - 0.03, Y1, FULL) + lower(Y1, FULL) + LET_GO, 0.0),
    ("carried 13 cm, lowered early, then dragged the last 4 cm into the bay", pump(4)
     + roll(Y0, Y1 + 0.04, FULL) + lower(Y1 + 0.04, FULL)
     + [(1, dict(y=y, grips=H, pallet=((X, y - m.PALLET_X, TOP + 0.0005), 0.0))) for y in ramp(Y1 + 0.04, Y1, 8)]
     + LET_GO, 0.0),
    ("pumped and lowered in place, never moved", pump(4) + lower(Y0, FULL) + [(40, {})], 0.0),
    ("carried 2.5 cm only and lowered (under the 3 cm move)", pump(4) + roll(Y0, Y0 - 0.025, FULL, 5)
     + lower(Y0 - 0.025, FULL) + [(40, dict(y=Y0 - 0.025))], 0.0),
    ("carried halfway and lowered there", pump(4) + roll(Y0, Y0 - 0.085, FULL) + lower(Y0 - 0.085, FULL)
     + [(40, dict(y=Y0 - 0.085))], 0.25),
    ("stopped 2.5 cm short: the pallet's back edge over the tape", pump(4) + roll(Y0, Y1 + 0.025, FULL)
     + lower(Y1 + 0.025, FULL) + [(40, dict(y=Y1 + 0.025))], 0.25),
    ("stopped 1 cm short: still inside the bay", pump(4) + roll(Y0, Y1 + 0.01, FULL) + lower(Y1 + 0.01, FULL)
     + [(40, dict(y=Y1 + 0.01))], 1.0),
    ("jack drifted 2.5 cm sideways: the pallet's side over the tape", pump(4) + roll(Y0, Y1, FULL, dx=0.025)
     + [(1, dict(y=Y1, lift=l, pedal=0.3, grips=P, dx=0.025)) for l in ramp(FULL, 0.0)]
     + [(40, dict(y=Y1, dx=0.025))], 0.25),
    ("pallet pushed into the bay by a gripper, jack left behind",
     [(1, dict(pallet=((X, y, TOP + 0.0005), 0.0), grips={"right_gripper": ((X, y + 0.09, TOP + 0.05), 0.0)}))
      for y in ramp(Y0 - m.PALLET_X, m.BAY[1], 20)]
     + [(40, dict(pallet=((X, m.BAY[1], TOP + 0.0005), 0.0)))], 0.0),
    ("pallet tipped off the forks", pump(4) + [(40, dict(lift=FULL, pallet=((X, Y0 - 0.15, TOP + 0.02), 40.0)))],
     0.0),
    ("pallet tipped 20 degrees for one step only, then the whole job", pump(4)[:-1]
     + [(1, dict(lift=FULL, pallet=((X, Y0 - m.PALLET_X, TOP + 0.012), 20.0)))] + roll(Y0, Y1, FULL)
     + lower(Y1, FULL) + LET_GO, 1.0),
]

failed = 0
for name, frames, want in cases:
    got, ok = run(frames)
    good = abs(got - want) < 1e-9 and ok == (want == 1.0)
    failed += not good
    print(("ok  " if good else "FAIL"), f"{name}: {got} (want {want})")

# A new trial (kit.episode() changes) starts with a clean history: a dragged run does not carry over, and a full run's
# credit doesn't either.
run(dragged_with(0.0))
got, _ = run(GOOD)
ok = got == 1.0
got2, _ = run([(60, {})])
ok = ok and got2 == 0.0
failed += not ok
print(("ok  " if ok else "FAIL"), f"history resets with each trial: {got}, then do-nothing {got2}")

print("all pass" if not failed else f"{failed} failed")
