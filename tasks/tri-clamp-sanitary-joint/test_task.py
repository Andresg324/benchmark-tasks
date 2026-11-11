"""Offline checks of the tri-clamp task's scorer: step-by-step histories of the clamp's tilt, the wing nut's turning and
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
HOME = {"left_gripper": (0.323, 0.30, TOP + 0.258), "right_gripper": (0.323, -0.30, TOP + 0.258)}
NUT_P = np.array(m.NUT_XYZ)
ON = m.NUT_WING_T / (2 * abs(m.FINGER_OPEN))  # opening with the pads on the wings
bad = 0


def qy_axis(deg):
    """A gripper rolled `deg` about world +y (its finger axis turning in the xz-plane)."""
    a = math.radians(deg) / 2
    return np.array([0.0, math.sin(a), 0.0, math.cos(a)])


def st(pos, q=(0.0, 0.0, 0.0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


class W:
    tilt = m.Q_COCK  # the clamp's tilt (degrees, hinge side up)
    roll = 0.0  # the left hand's roll about +y (degrees) while on the nut


def step(nut=None, held=False, tilt=None, grippers=None, follow=True, grip_open=None):
    """nut: degrees past contact this step (as PhysX would report it); held: the left gripper closed end-on on the wing
    nut, its wrist rolled with the nut if `follow` (tightening = rolling clockwise seen from +y, i.e. negative about
    +y); tilt: the clamp's tilt from now on (degrees); grippers: name -> position (fingers open) or (position, opening)
    or (position, opening, roll about +y). Runs twistkit's accounting, then success() and score() as the sim does."""
    t = tk._track(m.NUT)
    if nut is not None:
        if held and follow:
            W.roll -= nut - (-math.degrees(t.u))
        t.u = math.radians(-nut)
    if tilt is not None:
        W.tilt = tilt
    kit._JOINTS[m.CLAMP] = kit.JointState(pos=math.radians(W.tilt), vel=0.0, latched=False)
    objs = {"stand": st((m.X0, m.Y0, TOP)), m.NUT: st(NUT_P)}
    g = dict(HOME, **(grippers or {}))
    if held:
        g["left_gripper"] = (tuple(NUT_P + (0, 0.004, 0)), ON if grip_open is None else grip_open, W.roll)
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


def new_trial():
    kit._EPISODE["count"] += 1
    W.tilt, W.roll = m.Q_COCK, 0.0
    kit._OPENING.update({"left_gripper": 1.0, "right_gripper": 1.0})
    step()  # the trial's first scored step, both arms home (where "home" is recorded)


def turn(a, b, held=True, n=None, **kw):
    """Turn the nut from a to b degrees past contact in 5-degree steps, a gripper on it if `held`."""
    n = n or max(1, int(abs(b - a) / 5))
    out = None
    for k in range(1, n + 1):
        out = step(nut=a + (b - a) * k / n, held=held, **kw)
    return out


def regrip_turn(a, b, **kw):
    """Turn from a to b in 180-degree strokes with a let-go and roll-back between them (as the demo does)."""
    x = a
    while x < b - 1e-9:
        y = min(b, x + 180.0)
        step(held=True)  # pinch
        turn(x, y, **kw)
        step(held=False)  # let go
        W.roll += (y - x)  # roll back open
        step(held=False)
        x = y


def idle(n=25, **kw):
    out = None
    for _ in range(n):
        out = step(**kw)
    return out


def check(name, got, want):
    global bad
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(f"{'ok  ' if ok else 'FAIL'} {name}: {got} (want {want})")


PRESS = (m.PRESS_XY[0], m.PRESS_XY[1], m.PRESS_TOP + 0.013)

# 1. doing nothing: cocked clamp, nut 1.5 turns loose
new_trial()
check("do nothing", idle(40)[0], 0.0)

# 2. the job: press square, tighten one turn past contact with regrips, let go, arms home
new_trial()
step(grippers={"left_gripper": (PRESS, 0.0)})
step(tilt=0.0, grippers={"left_gripper": (PRESS, 0.0)})
regrip_turn(m.THREAD.start, 360.0)
check("full job, arms home", idle(10)[0], 1.0)
check("success() true at full job", float(step()[1]), 1.0)

# 3. same, but arms parked away from home
new_trial()
step(tilt=0.0)
regrip_turn(m.THREAD.start, 360.0)
park = {"left_gripper": (0.20, 0.20, TOP + 0.20)}
check("full job, arm not home", idle(10, grippers=park)[0], 0.9)

# 4. still holding the wing nut at the end
new_trial()
step(tilt=0.0)
regrip_turn(m.THREAD.start, 360.0)
step(held=True)
check("still holding the nut", step(held=True)[0], 0.0)

# 5. still pressing the clamp at the end (hand at the hinge, fingers closed)
new_trial()
step(tilt=0.0)
regrip_turn(m.THREAD.start, 360.0)
check("still pressing the clamp", idle(5, grippers={"left_gripper": (PRESS, 0.0)})[0], 0.0)

# 6. tightened while cocked (the tempting move): 0, and reseating after backing off does not undo it
new_trial()
regrip_turn(m.THREAD.start, 360.0)
check("tightened cocked", idle(5)[0], 0.0)
regrip_turn(360.0, 360.0)
turn(360.0, -100.0, held=True)  # backed off below contact (loosening with a grip)
step(tilt=0.0)
regrip_turn(-100.0, 360.0)
check("tightened cocked, then backed off and reseated", idle(5)[0], 0.0)

# 7. cocked, nut only to contact + 5 degrees (inside the 10-degree noise band), then seated and tightened: fine
new_trial()
regrip_turn(m.THREAD.start, 5.0)
step(tilt=0.0)
regrip_turn(5.0, 360.0)
check("touched contact cocked (5 deg), then seated", idle(10)[0], 1.0)

# 8. cocked, nut 15 degrees past contact (just past the band): latched
new_trial()
regrip_turn(m.THREAD.start, 15.0)
step(tilt=0.0)
regrip_turn(15.0, 360.0)
check("15 deg past contact while cocked", idle(10)[0], 0.0)

# 9. seat tolerance: 0.79 degrees counts as square, 0.81 does not (it latches once tightened)
new_trial()
step(tilt=0.79)
regrip_turn(m.THREAD.start, 360.0)
check("tilt 0.79 deg", idle(10)[0], 1.0)
new_trial()
step(tilt=0.81)
regrip_turn(m.THREAD.start, 360.0)
check("tilt 0.81 deg", idle(10)[0], 0.0)

# 10. square but the nut never past contact: nothing holds the joint
new_trial()
step(tilt=0.0)
check("square, nut loose", idle(10)[0], 0.0)
regrip_turn(m.THREAD.start, 0.0)
check("square, nut at contact", idle(10)[0], 0.0)
regrip_turn(0.0, 10.0)
check("square, nut 10 deg past contact", idle(10)[0], 0.0)

# 11. loose: past contact but under half a turn
new_trial()
step(tilt=0.0)
regrip_turn(m.THREAD.start, 170.0)
check("170 deg past contact (loose)", idle(10)[0], 0.5)
regrip_turn(170.0, 185.0)
check("185 deg past contact (snug)", idle(10)[0], 1.0)

# 12. overtight: past 1.5 turns, backed off into the window: still 0.25
new_trial()
step(tilt=0.0)
regrip_turn(m.THREAD.start, 560.0)
check("560 deg past contact (overtight)", idle(10)[0], 0.25)
turn(560.0, 360.0, held=True)
step(held=False)
check("overtight then backed off to 360", idle(10)[0], 0.25)
new_trial()
step(tilt=0.0)
regrip_turn(m.THREAD.start, 535.0)
check("535 deg past contact (top of the window)", idle(10)[0], 1.0)

# 13. the nut spun by a swipe, no grip on it: 0
new_trial()
step(tilt=0.0)
turn(m.THREAD.start, 360.0, held=False)
check("nut spun without a grip", idle(10)[0], 0.0)

# 14. a closed hand resting on the nut, its wrist still, while something else spins the nut: not credited
new_trial()
step(tilt=0.0)
turn(m.THREAD.start, 360.0, held=True, follow=False)
check("held but the wrist did not turn it", idle(10)[0], 0.0)

# 15. an empty closed hand at the nut does not count as a grip
new_trial()
step(tilt=0.0)
turn(m.THREAD.start, 360.0, held=True, grip_open=0.0)
check("empty closed fist at the nut", idle(10)[0], 0.0)

# 16. a scene reset clears a latched cocked tightening
new_trial()
regrip_turn(m.THREAD.start, 360.0)
check("cocked tightening (before reset)", idle(3)[0], 0.0)
new_trial()
step(tilt=0.0)
regrip_turn(m.THREAD.start, 360.0)
check("after reset, done right", idle(10)[0], 1.0)

# 17. clamp squared after the nut was tightened square, then knocked back up (the law locks it, but if PhysX moved
#     it anyway the scorer reads the joint): drawn up while tilted counts as cocked
new_trial()
step(tilt=0.0)
regrip_turn(m.THREAD.start, 360.0)
step(tilt=2.0)
check("tight, then the clamp reads tilted", idle(5)[0], 0.0)
step(tilt=0.0)
check("... and square again: still latched", idle(10)[0], 0.0)

# 18. hovering an open hand 3 cm from the nut at the end: not let go
new_trial()
step(tilt=0.0)
regrip_turn(m.THREAD.start, 360.0)
check("open hand hovering at the nut", idle(5, grippers={"left_gripper": tuple(NUT_P + (0, 0.03, 0))})[0], 0.0)

# 19. the seating law (sim side): cocked holds at Q_COCK, a press past Q_SLIP seats it, the nut past contact locks it
law, mem = m.ClampSeat(), {"mode": "cocked", "lock": None}
tk._track(m.NUT).u = math.radians(540.0)
check("law: cocked target", round(math.degrees(law.drive(m.CLAMP, math.radians(5.0), 0, {}, mem)[0]), 6), m.Q_COCK)
law.drive(m.CLAMP, math.radians(4.0), 0, {}, mem)
check("law: pressed past Q_SLIP seats", float(mem["mode"] == "seated"), 1.0)
check("law: seated target", law.drive(m.CLAMP, math.radians(2.0), 0, {}, mem)[0], 0.0)
tk._track(m.NUT).u = math.radians(-30.0)
check("law: locked where it is", round(law.drive(m.CLAMP, math.radians(0.2), 0, {}, mem)[0], 9),
      round(math.radians(0.2), 9))
check("law: lock holds the first angle", round(law.drive(m.CLAMP, math.radians(1.0), 0, {}, mem)[0], 9),
      round(math.radians(0.2), 9))

# 20. hand swap (QC Nov 10): the nut passes from one closed hand to the other each step, each hand having rolled
#     170 degrees in free air while open, neither wrist turning while it holds: the free-air roll is not credited
new_trial()
step(tilt=0.0)
NUT_G = tuple(NUT_P + (0, 0.004, 0))
rolls, x, k = {"left_gripper": 0.0, "right_gripper": 0.0}, m.THREAD.start, 1
step(nut=x, grippers={"left_gripper": (NUT_G, ON, 0.0)})  # the left hand pinches the nut, nothing turned yet
while x < 360.0:
    h, o = ("left_gripper", "right_gripper")[k % 2], ("right_gripper", "left_gripper")[k % 2]
    rolls[h] -= 170.0  # rolled the tightening way while open, then lands on the nut as the other hand lets go
    x = min(360.0, x + 170.0)
    step(nut=x, grippers={h: (NUT_G, ON, rolls[h]), o: (NUT_G, 1.0, rolls[o])})
    k += 1
check("hand swap: free-air roll not credited", idle(10)[0], 0.0)

# 21. squared and drawn past contact in the same step (QC Nov 10): the nut was drawn up against a cocked clamp
new_trial()
regrip_turn(m.THREAD.start, 5.0)  # cocked, at contact + 5 (inside the band)
step(held=True)
step(nut=15.0, held=True, tilt=0.0)  # pressed square and drawn to contact + 15 in the same step
step(held=False)
regrip_turn(15.0, 360.0)
check("square and past contact in one step", idle(10)[0], 0.0)

print("all pass" if not bad else f"{bad} FAILED")
