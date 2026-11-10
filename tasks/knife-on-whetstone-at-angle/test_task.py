"""Offline checks of the whetstone scorer: hand-made knife poses and gripper positions step by step, no simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
HOME = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}


def axrot(a, t):
    return m._axrot(a, math.radians(t))


def quat(r):
    w = math.sqrt(max(0.0, 1 + r[0, 0] + r[1, 1] + r[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + r[0, 0] - r[1, 1] - r[2, 2])) / 2, r[2, 1] - r[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] + r[1, 1] - r[2, 2])) / 2, r[0, 2] - r[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - r[0, 0] - r[1, 1] + r[2, 2])) / 2, r[1, 0] - r[0, 1])
    return np.array([x, y, z, w])


def st(pos, q=(0, 0, 0, 1.0), vel=(0, 0, 0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.array(vel, float), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def knife_rot(heading, phi):
    """The knife turned to `heading`, then `phi` degrees about its own handle axis (+ tips the edge down on face A;
    180 - a lies face B down with the edge down by a)."""
    r0 = axrot((0, 0, 1), heading)
    return axrot(r0 @ np.array([0, 1.0, 0]), phi) @ r0


def frame(pos, r, grip="right", opening=0.0, grippers=None):
    """One control step: knife at (pos, r); the right gripper on the handle (grip="right"), or `grippers` given."""
    g = dict(HOME)
    if grippers is not None:
        g.update(grippers)
    elif grip == "right":
        g["right_gripper"] = tuple(np.asarray(pos) + r @ np.array([0, m.GRIP_Y, 0]))
    kit._OPENING["right_gripper"] = opening
    kit._OPENING["left_gripper"] = 1.0
    objs = {"knife": st(pos, quat(r)), "board": st((*m.BOARD_POS, m.TOP)), "stone": st((*m.STONE_POS, m.TOP)),
            **{k: st(v) for k, v in g.items()}}
    ok = m.success(objs)
    return m.score(objs), ok


def on_stone_pos(r, a, lift=0.001):
    """Knife position putting the edge point 5 cm along the blade at `a` along the stone, `lift` above its top."""
    target = np.r_[m.STONE_POS + a * m.S_U[:2], m.TOP + m.STONE_TOP + lift]
    p = target - r @ np.array([m.EDGE_X, m.STONE_V + m.GRIP_Y, 0.0])
    lows = [(p + r @ np.array([m.EDGE_X, y, s * m.BLADE_T / 2]))[2] for y in m.EDGE_SAMPLES for s in (-1, 1)]
    p[2] += (m.TOP + m.STONE_TOP + lift) - min(lows)
    return p


def strokes(face, n, angle=17.0, lead=True, lift=0.001, a0=None, travel=0.065, **kw):
    """n strokes on `face` ('A' pushes along +u, 'B' pulls), each lifted 2 cm between."""
    phi = angle if face == "A" else 180.0 - angle
    r = knife_rot(m.STONE_HEADING, phi)
    sign = (1 if face == "A" else -1) * (1 if lead else -1)
    a0 = (-0.005 * sign) if a0 is None else a0
    out = []
    for _ in range(n):
        for k in range(21):
            out.append(frame(on_stone_pos(r, a0 + sign * travel * k / 20, lift), r, **kw))
        p = on_stone_pos(r, a0 + sign * travel, lift) + [0, 0, 0.02]
        out.append(frame(p, r, **kw))
        out.append(frame(on_stone_pos(r, a0, lift) + [0, 0, 0.02], r, **kw))
    return out


def start_pos():
    return np.array(m.KNIFE_START), knife_rot(m.PARK_HEADING, 0.0)


def put_down(heading=m.PARK_HEADING, phi=0.0, release=True, home=True):
    r = knife_rot(heading, phi)
    p = np.array(m.KNIFE_START)
    for _ in range(12):  # carried over to the board, well above everything
        frame(p + [0, 0, 0.08], r)
    if phi:
        p[2] += 0.02
    frame(p, r)
    if release:
        frame(p, r, grippers=HOME if home else {"right_gripper": (0.30, -0.12, m.TOP + 0.15)})
        return frame(p, r, grippers=HOME if home else {"right_gripper": (0.30, -0.12, m.TOP + 0.15)})
    return frame(p, r)


def new_trial():
    kit._EPISODE["count"] += 1
    p, r = start_pos()
    frame(p, r, grippers=HOME)  # first step: arms home, knife on the board


def check(name, got, want):
    assert abs(got - want) < 1e-9, f"{name}: score {got}, expected {want}"
    print(f"ok  {name}: {got}")


# 1. Do nothing.
new_trial()
p, r = start_pos()
for _ in range(30):
    s, _ = frame(p, r, grippers=HOME)
check("do nothing", s, 0.0)

# 2. The full job: 4 strokes each face at 17 degrees, edge leading, put down flat edge away, arms home.
new_trial()
strokes("A", 4)
strokes("B", 4)
s, ok = put_down()
check("full job", s, 1.0)
assert ok, "success() should be true at the end of the full job"

# 3. Same, but the arm not back home: 0.9.
new_trial()
strokes("A", 4)
strokes("B", 4)
check("arms not home", put_down(home=False)[0], 0.9)

# 4. Knife still held at the end (no release): 0.75.
new_trial()
strokes("A", 4)
strokes("B", 4)
check("still held at the end", put_down(release=False)[0], 0.75)

# 5. Flat strokes (5 degrees, scratching the face): no strokes count.
new_trial()
strokes("A", 4, angle=5.0)
strokes("B", 4, angle=5.0)
check("flat strokes", put_down()[0], 0.0)

# 6. Steep strokes (30 degrees) after good ones: the edge is rounded, 0 for the trial (irreversible).
new_trial()
strokes("A", 4)
strokes("B", 4)
strokes("A", 1, angle=30.0)
strokes("B", 4)
check("steep stroke after good ones", put_down()[0], 0.0)

# 7. One face only: 0.25.
new_trial()
strokes("A", 5)
check("one side only", put_down()[0], 0.25)

# 8. Two strokes per face: not enough.
new_trial()
strokes("A", 2)
strokes("B", 2)
check("two strokes per side", put_down()[0], 0.0)

# 9. Edge-trailing strokes only: none count.
new_trial()
strokes("A", 4, lead=False)
strokes("B", 4, lead=False)
check("edge trailing", put_down()[0], 0.0)

# 10. Hovering 1 cm above the stone: no contact, no strokes.
new_trial()
strokes("A", 4, lift=0.01)
strokes("B", 4, lift=0.01)
check("hovering", put_down()[0], 0.0)

# 11. The knife dragged with the gripper open (pushed, not held), and with the other arm's open gripper.
new_trial()
strokes("A", 4, opening=1.0)
strokes("B", 4, opening=1.0)
check("dragged, gripper open", put_down()[0], 0.0)
new_trial()
strokes("A", 4, grip=None, grippers={"left_gripper": (0.40, -0.05, m.TOP + 0.05)})
strokes("B", 4, grip=None, grippers={"left_gripper": (0.40, -0.05, m.TOP + 0.05)})
check("dragged by the other arm", put_down()[0], 0.0)

# 12. Put down with the edge towards the robot, or left on the stone: 0.75.
new_trial()
strokes("A", 4)
strokes("B", 4)
check("edge towards the robot", put_down(heading=m.PARK_HEADING + 180.0)[0], 0.75)
new_trial()
strokes("A", 4)
strokes("B", 4)
r = knife_rot(m.STONE_HEADING, 17.0)
p = on_stone_pos(r, 0.0)
frame(p, r, grippers=HOME)
check("left on the stone", frame(p, r, grippers=HOME)[0], 0.75)

# 13. Rolled off the end: a stroke carried past the stone's far end with the edge dropping.
new_trial()
strokes("A", 4)
strokes("B", 4)
r = knife_rot(m.STONE_HEADING, 17.0)
for k in range(10):
    frame(on_stone_pos(r, 0.05 + 0.004 * k), r)
for k in range(1, 6):
    frame(on_stone_pos(r, 0.09 + 0.006 * k, lift=-0.002 * k), r)
check("rolled off the end", put_down()[0], 0.0)

# 14. A new trial (kit.episode() moves on) forgets the last one's strokes.
new_trial()
strokes("A", 4)
strokes("B", 4)
put_down()
new_trial()
p, r = start_pos()
check("reset after a full job", frame(p, r, grippers=HOME)[0], 0.0)

# 15. Pitched, not angled (reviewer's case): the knife rolled only 0.5 to 1 degree but pitched 15 to 17 degrees tip-down,
# dragged 7 cm edge first with only the tip on the stone. The old face-tilt angle read this as 15 to 17 degrees; the
# angle across the blade is about 1 degree, the length axis is not level and only the tip touches: no strokes.
def tip_pos(r, a, lift=0.001):
    """Knife position putting the lowest edge point at `a` along the stone's middle, `lift` above its top."""
    pts = [r @ np.array([m.EDGE_X, y, s * m.BLADE_T / 2]) for y in m.EDGE_SAMPLES for s in (-1, 1)]
    low = min(pts, key=lambda v: v[2])
    return np.r_[m.STONE_POS + a * m.S_U[:2], m.TOP + m.STONE_TOP + lift] - low


def pitched(face, roll, pitch, n=4, travel=0.07):
    """n edge-leading strokes on `face`, the knife rolled `roll` about its handle axis and pitched `pitch` tip-down."""
    r = knife_rot(m.STONE_HEADING, roll if face == "A" else 180.0 - roll)
    r = axrot(r @ np.array([1.0, 0, 0]), pitch) @ r
    sign = 1 if face == "A" else -1
    for _ in range(n):
        for k in range(21):
            frame(tip_pos(r, -0.005 * sign + sign * travel * k / 20), r)
        frame(tip_pos(r, sign * travel) + [0, 0, 0.02], r)
        frame(tip_pos(r, 0.0) + [0, 0, 0.02], r)


for roll, pitch in ((1.0, 17.0), (1.0, -17.0), (0.5, 15.0)):
    new_trial()
    pitched("A", roll, pitch)
    pitched("B", roll, pitch)
    check(f"pitched {pitch} deg, rolled {roll} deg, tip only", put_down()[0], 0.0)

# 16. Correct angle (17 deg across the blade) with the blade's length level or pitched 3 deg: the edge lies along
# the stone and every stroke counts.
for pitch in (0.0, 3.0):
    new_trial()
    pitched("A", 17.0, pitch)
    pitched("B", 17.0, pitch)
    check(f"17 deg across the blade, pitched {pitch} deg", put_down()[0], 1.0)

# 17. Correct angle but pitched 8 deg (only about 2 cm of edge on the stone, length not level): no strokes.
new_trial()
pitched("A", 17.0, 8.0)
pitched("B", 17.0, 8.0)
check("17 deg across the blade, pitched 8 deg", put_down()[0], 0.0)

print("all pass")
