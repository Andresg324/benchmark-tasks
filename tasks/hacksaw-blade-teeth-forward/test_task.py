"""Offline checks of the hacksaw task's scorer: step-by-step histories of the blade's pose, the wing nut's turning and
the grippers, with near misses. No simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, tk = m.kit, m.tk
TOP = m.TOP
tk.register(*[o for o in m.OBJECTS if isinstance(o, tk.Threaded)])
PARK = {"left_gripper": (0.20, 0.27, TOP + 0.22), "right_gripper": (0.18, -0.30, TOP + 0.22)}
NUT_P = np.array([m.SAW_X, m.SAW_Y + m.NUT_Y, TOP + m.H])
SEATED = (m.SAW_X, m.SAW_Y, TOP + m.H)
ON = m.NUT_WING_T / (2 * abs(m.FINGER_OPEN))  # opening with the pads on the wings
bad = 0


def quat(yaw=0.0, roll=0.0):
    """Blade orientation: `roll` about its long axis (local y) first, then `yaw` about the vertical (degrees)."""
    r, y = math.radians(roll) / 2, math.radians(yaw) / 2
    qr = np.array([0.0, math.sin(r), 0.0, math.cos(r)])
    qy = np.array([0.0, 0.0, math.sin(y), math.cos(y)])
    return np.asarray(kit._quat_mul(qy, qr), float)


def qy_axis(deg):
    """A gripper rolled `deg` about world +y (its finger axis turning in the xz-plane)."""
    a = math.radians(deg) / 2
    return np.array([0.0, math.sin(a), 0.0, math.cos(a)])


def st(pos, q=(0.0, 0.0, 0.0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


class W:
    blade = (m.BLADE_START, quat(math.degrees(m.BLADE_YAW)))
    roll = 0.0  # the right hand's roll about +y (degrees) while on the nut


START = (m.BLADE_START, quat(math.degrees(m.BLADE_YAW)))
GOOD = (SEATED, quat(180.0))


def step(nut=None, held=False, blade=None, grippers=None, follow=True, grip_open=None, roll=None):
    """nut: degrees past snug this step (as PhysX would report it); held: the right gripper closed end-on on the wing
    nut, its wrist rolled with the nut if `follow` (else kept still); blade: (pos, quat) from now on; grippers: name ->
    position (fingers open) or (position, opening) or (position, opening, roll about +y). Runs twistkit's accounting."""
    t = tk._track(m.NUT)
    if nut is not None:
        if held and follow:
            W.roll += nut - (-math.degrees(t.u))
        t.u = math.radians(-nut)
    if roll is not None:
        W.roll = roll
    if blade is not None:
        W.blade = blade
    objs = {"saw": st((m.SAW_X, m.SAW_Y, TOP)), m.NUT: st(NUT_P), "blade": st(*W.blade)}
    g = dict(PARK, **(grippers or {}))
    if held:
        g["right_gripper"] = (tuple(NUT_P), ON if grip_open is None else grip_open, W.roll)
    for k, v in g.items():
        if isinstance(v[0], tuple):
            pos, op, roll = (v + (0.0,))[:3]
        else:
            pos, op, roll = v, 1.0, 0.0
        objs[k] = st(pos, qy_axis(roll))
        kit._OPENING[k] = op
    tk._account(objs)
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames):
    """frames: (steps, kwargs). A fresh trial each time, starting from the scene's start."""
    kit._EPISODE["count"] += 1
    W.blade, W.roll = START, 0.0
    t = tk._track(m.NUT)
    out = step()  # the trial's first scored step: arms at PARK, so home is PARK
    assert abs(-math.degrees(t.u) - m.THREAD.start) < 1e-6
    for n, kw in frames:
        for i in range(n):
            out = step(**(kw if i == 0 else {k: v for k, v in kw.items() if k not in ("nut", "blade")}))
    return out


def turn(to, frm=None, held=True, by=15.0, **kw):
    """The wing nut turned to `to` degrees past snug in 15-degree steps, the right hand on it (a step of grip before
    and after) if `held`."""
    frames = [(1, dict(held=held, **kw))] if held else []
    a = m.THREAD.start if frm is None else frm
    while abs(a - to) > 1e-9:
        a = min(a + by, to) if to > a else max(a - by, to)
        frames.append((1, dict(nut=a, held=held, **kw)))
    return frames + ([(1, dict(held=held, **kw))] if held else [])


def strokes(to, frm=None, stroke=180.0, **kw):
    """Regrasp loop: turn `stroke` degrees held, let go and roll back open, repeat."""
    out, a = [], m.THREAD.start if frm is None else frm
    while a < to - 1e-9:
        b = min(a + stroke, to)
        out += turn(b, frm=a, **kw)
        out.append((2, dict(grippers={"right_gripper": (tuple(NUT_P + (0, -0.03, 0)), 1.0)})))
        out.append((2, {}))
        a = b
    return out


def place(pose=GOOD):
    """The left hand carries the blade (held: fingers closed at its middle) and lets go of it at `pose`."""
    mid = np.asarray(pose[0], float)
    return [(3, dict(blade=pose, grippers={"left_gripper": (tuple(mid), 0.1)})),
            (2, dict(grippers={"left_gripper": (tuple(mid + (0, 0, 0.06)), 1.0)}))]


DONE = [(40, {})]
TAUT = 540.0
def wiggle(amp, step=15.0, op=None, target=600.0):
    """A closed hand that never opens rolls `amp` degrees forward while something else pushes the nut along, then rolls
    back the same amount with the nut left where it is (another arm's swipe laundered through a wiggling grip)."""
    fr = place() + [(1, dict(held=True, roll=0.0, grip_open=op))]
    a, roll = m.THREAD.start, 0.0
    while a < target - 1e-9:
        for _ in range(int(amp / step)):
            a += step
            roll += step
            fr.append((1, dict(nut=a, held=True, follow=False, roll=roll, grip_open=op)))
        for _ in range(int(amp / step)):
            roll -= step
            fr.append((1, dict(held=True, follow=False, roll=roll, grip_open=op)))
    return fr + DONE


cases = [
    # review fixes, Nov 10
    ("overtightened to 1400 degrees, then backed off into the window", place() + strokes(1400.0)
     + turn(600.0, frm=1400.0) + DONE, 0.25),
    ("nut turned by an empty, fully closed hand (opening 0)", place() + turn(TAUT, grip_open=0.0) + DONE, 0.0),
    ("closed hand wiggles +-60 degrees, nut pushed along only on its forward half", wiggle(60.0), 0.0),
    ("closed hand wiggles +-180 degrees, nut pushed along only on its forward half", wiggle(180.0), 0.0),
    ("blade knocked 2.5 mm up for one step while tensioning", place() + strokes(300.0)
     + [(1, dict(blade=((m.SAW_X, m.SAW_Y, TOP + m.H + 0.0025), quat(180.0)))), (1, dict(blade=GOOD))]
     + strokes(TAUT, frm=300.0) + DONE, 1.0),
    ("blade seated 3.1 mm along its slots (still over both pins)",
     place(((m.SAW_X, m.SAW_Y + 0.0031, TOP + m.H), quat(180.0))) + strokes(TAUT) + DONE, 1.0),
    ("do nothing", [(100, {})], 0.0),
    ("blade fitted right, 1.5 turns past snug, both home", place() + strokes(TAUT) + DONE, 1.0),
    ("same, but the left arm not home", place() + strokes(TAUT) + [(40, dict(grippers={
        "left_gripper": (0.30, 0.25, TOP + 0.15)}))], 0.9),
    ("blade fitted right, wing nut never turned past snug", place() + DONE, 0.25),
    ("blade fitted right, nut only to snug", place() + strokes(0.0) + DONE, 0.25),
    ("blade fitted right, nut half a turn past snug (slack)", place() + strokes(180.0) + DONE, 0.5),
    ("blade fitted right, nut 3 turns past snug (overtight)", place() + strokes(1080.0) + DONE, 0.25),
    ("1 turn past snug (edge of the window)", place() + strokes(360.0) + DONE, 1.0),
    ("2.5 turns past snug (edge of the window)", place() + strokes(900.0) + DONE, 1.0),
    # the teeth the wrong way
    ("blade carried straight across, arrow toward the handle (teeth to the back bar)",
     place((SEATED, quat(0.0))) + strokes(TAUT) + DONE, 0.0),
    ("blade turned over: teeth toward the robot but pointing to the handle",
     place((SEATED, quat(0.0, 180.0))) + strokes(TAUT) + DONE, 0.0),
    ("blade turned over and end for end: arrow forward, teeth toward the back bar",
     place((SEATED, quat(180.0, 180.0))) + strokes(TAUT) + DONE, 0.0),
    ("blade turned only 160 degrees (20 off: its holes miss the pins)",
     place((SEATED, quat(160.0))) + strokes(TAUT) + DONE, 0.0),
    # not on both pins
    ("one pin only: blade 5 mm along, its far end resting on top of the other pin",
     place(((m.SAW_X, m.SAW_Y + 0.005, TOP + m.H + 0.0025), quat(180.0))) + strokes(TAUT) + DONE, 0.0),
    ("blade 4 mm toward the robot (holes off the pins)",
     place(((m.SAW_X - 0.004, m.SAW_Y, TOP + m.H), quat(180.0))) + strokes(TAUT) + DONE, 0.0),
    ("blade on top of both pins, not down over them (5 mm high)",
     place(((m.SAW_X, m.SAW_Y, TOP + m.H + 0.005), quat(180.0))) + strokes(TAUT) + DONE, 0.0),
    ("blade left on its blocks, nut tightened", strokes(TAUT) + DONE, 0.0),
    # order
    ("nut tightened first, blade laid on the pins after", strokes(TAUT) + place() + DONE, 0.0),
    ("nut tightened first, backed off to slack, blade fitted, tightened again",
     strokes(TAUT) + turn(-90.0, frm=TAUT) + [(2, {})] + place() + strokes(TAUT, frm=-90.0) + DONE, 1.0),
    ("blade fitted and tightened, then pulled off the pins", place() + strokes(TAUT) +
     [(5, dict(blade=START))] + DONE, 0.0),
    # turning without a real grip
    ("nut turned with no hand on it (knocked round)", place() + turn(TAUT, held=False) + DONE, 0.0),
    ("nut turned by something else while the right hand hovers open around it", place() +
     turn(TAUT, held=True, grip_open=0.6) + DONE, 0.0),
    ("hand closed on the nut but the wrist never rolls (the nut turned by another push)", place() +
     turn(TAUT, follow=False) + DONE, 0.0),
    ("hand closed on the nut wiggling its wrist back and forth while the nut turns one way", place() +
     [(1, dict(held=True))] +
     [(1, dict(nut=-180.0 + 15.0 * (k + 1), held=True, follow=False, roll=20.0 * (k % 2)))
      for k in range(48)] + DONE, 0.0),
    # not let go
    ("left hand still pinching the blade at the end", place() + strokes(TAUT) +
     [(40, dict(grippers={"left_gripper": (SEATED, 0.1)}))], 0.0),
    ("right hand still closed on the wing nut at the end", place() + strokes(TAUT) + [(40, dict(held=True))], 0.0),
]


for name, frames, want in cases:
    got, ok = run(frames)
    flag = "ok " if abs(got - want) < 1e-9 else "BAD"
    if flag == "BAD":
        bad += 1
    print(f"{flag} {got:.2f} (want {want:.2f})  {name}")

# a reset mid-way (a new trial) clears the history: a trial that did everything is followed by one that does nothing
run(place() + strokes(TAUT) + DONE)
got, _ = run([(60, {})])
print(f"{'ok ' if got == 0.0 else 'BAD'} {got:.2f} (want 0.00)  reset after a full trial, then nothing")
bad += got != 0.0

# the wing nut's rolling wrist, credited from the right gripper's own roll about the nut's (horizontal) axis
t = tk._track(m.NUT)
for a, want in ((0.0, 0.0), (90.0, 90.0), (-45.0, -45.0)):
    got = math.degrees(tk._grip_yaw(st((0, 0, 0), qy_axis(a)), (0.0, 1.0, 0.0)))
    flag = "ok " if abs(got - want) < 1e-6 else "BAD"
    bad += flag == "BAD"
    print(f"{flag} grip roll about +y {got:.1f} (want {want:.1f})")

print("all pass" if bad == 0 else f"{bad} FAILED")
