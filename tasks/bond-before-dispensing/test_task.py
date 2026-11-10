"""Offline checks of the bonding task's scorer: step-by-step histories of the faucet lever, the clamp's attachment, the
can and the grippers, traps and near misses included. No simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, ck = m.kit, m.ck
ck.register(m.BOND)
TOP = m.TOP
PARK = {"left_gripper": (0.20, 0.27, TOP + 0.22), "right_gripper": (0.18, -0.30, TOP + 0.22)}
SHUT, OPEN = 0.0, m.LEVER_DOWN
HALF = math.radians(-11.0)  # pressed, but short of the 11.5-degree threshold
CAN_AT = np.array(m.CAN, float)
CAN_Q = (0.0, 0.0, math.sin(m.CAN_YAW / 2), math.cos(m.CAN_YAW / 2))


def st(pos, quat=(0.0, 0, 0, 1)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


_a = math.atan2(m.IN[1], m.IN[0])
CLAMP_Q = (0.0, 0.0, math.sin(_a / 2), math.cos(_a / 2))  # pointing into the can through the tab
CLAMP_REST = m.BOND.nodes()[-1]


def step(lever, on=None, can=None, can_q=CAN_Q, grippers=None, clamp=None, held=False):
    """One control step: the lever's angle, the target the clamp is on (None, "can" or "drum"), the can's position,
    gripper tool points (default: parked at home) and the clamp's position."""
    kit._JOINTS["drum/lever"] = kit.JointState(lever, 0.0, False)
    ck.set_attached(m.CLAMP, on)
    ck.set_held(m.CLAMP, held)
    g = dict(PARK, **(grippers or {}))
    objs = {"drum": st(m.DRUM), "can": st(CAN_AT if can is None else can, can_q),
            m.CLAMP: st(CLAMP_REST if clamp is None else clamp, CLAMP_Q),
            m.LEAD: ck.state_from_points(m.LEAD, m.BOND.nodes())}
    for k, v in g.items():
        objs[k] = st(v)
        kit._OPENING[k] = 1.0
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames, new_trial=True):
    """frames: (steps, lever, clamp on[, extra kwargs]). A fresh trial unless new_trial is False."""
    if new_trial:
        kit._EPISODE["count"] += 1
    out = step(SHUT)  # the trial's first scored step: the arms at their ready pose (PARK), so home is PARK
    for fr in frames:
        n, lever, on = fr[:3]
        kw = fr[3] if len(fr) > 3 else {}
        for _ in range(n):
            out = step(lever, on, **kw)
    return out


S = round(1 / m.DT)  # control steps per second
GOOD = 6 * S  # 6 s of flow: 75 %
moved = {"can": CAN_AT + np.array([0.0, 0.02, 0.0])}  # opening 2 cm off the spout's axis
tipped = {"can_q": (math.sin(math.radians(15)), 0.0, 0.0, math.cos(math.radians(15)))}  # leaning 30 degrees
on_lever = {"grippers": {"right_gripper": (0.235, m.TAP_Y, TOP + 0.16)}}
on_clamp = {"grippers": {"left_gripper": tuple(CLAMP_REST + np.array([0.0, -0.02, 0.0]))}}
away = {"grippers": {"right_gripper": (0.25, -0.25, TOP + 0.15)}}

CASES = [
    ("nothing done", [(300, SHUT, None)], 0.0),
    ("bonded only, never filled", [(300, SHUT, "can")], 0.0),
    ("filled without bonding", [(GOOD, OPEN, None), (50, SHUT, None)], 0.0),
    ("clamp on the drum's chime, then filled", [(GOOD, OPEN, "drum"), (50, SHUT, "drum")], 0.0),
    ("a 1-step squirt before bonding, then bonded and filled", [(1, OPEN, None), (20, SHUT, "can"),
                                                                 (GOOD, OPEN, "can"), (50, SHUT, "can")], 0.0),
    ("bonded, filled, clamp pulled off while still flowing", [(GOOD, OPEN, "can"), (5, OPEN, None),
                                                               (50, SHUT, None)], 0.0),
    ("lever pressed short of the threshold the whole run, unbonded", [(400, HALF, None)], 0.0),
    ("filled first, then bonded and left clipped on", [(GOOD, OPEN, None), (20, SHUT, "can"), (50, SHUT, "can")], 0.0),
    ("clamp squeezed open on the tab (held there, not biting) while it filled",
     [(GOOD, OPEN, None, {"held": True}), (50, SHUT, None)], 0.0),
    ("bonded, filled to 75 %, faucet still held open at the end", [(GOOD, OPEN, "can")], 0.25),
    ("bonded, overfilled to 95 % (above the 90 % line)", [(round(7.6 * S), OPEN, "can"), (50, SHUT, "can")], 0.25),
    ("bonded, can 2 cm off the spout: all spilled", [(GOOD, OPEN, "can", moved), (50, SHUT, "can", moved)], 0.0),
    ("bonded, 1 s spilled then the can moved under and filled",
     [(S, OPEN, "can", moved), (GOOD, OPEN, "can"), (50, SHUT, "can")], 0.25),
    ("bonded, filled, then the can knocked over", [(GOOD, OPEN, "can"), (50, SHUT, "can", tipped)], 0.25),
    ("bonded, filled to 50 %", [(4 * S, OPEN, "can"), (50, SHUT, "can")], 0.5),
    ("bonded, filled, a gripper still on the lever", [(GOOD, OPEN, "can"), (50, SHUT, "can", on_lever)], 0.5),
    ("bonded, filled, clamp removed after, still held", [(GOOD, OPEN, "can"), (50, SHUT, None, on_clamp)], 0.5),
    ("bonded, filled, an arm parked away from home", [(GOOD, OPEN, "can"), (50, SHUT, "can", away)], 0.9),
    ("bonded, filled to 75 %, shut, arms home", [(GOOD, OPEN, "can"), (50, SHUT, "can")], 1.0),
    ("same, clamp taken off after the flow stopped", [(GOOD, OPEN, "can"), (20, SHUT, "can"), (50, SHUT, None)], 1.0),
    ("filled in two spells, bonded throughout (60 % exactly)",
     [(round(2.4 * S), OPEN, "can"), (20, SHUT, "can"), (round(2.4 * S), OPEN, "can"), (50, SHUT, "can")], 1.0),
    ("0.5 s spilled (not more), then filled", [(S // 2, OPEN, "can", moved), (GOOD, OPEN, "can"),
                                               (50, SHUT, "can")], 1.0),
]

bad = 0
for name, frames, want in CASES:
    got, _ = run(frames)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

# success() needs the full score; a reset starts a clean history (a violation or fill is not carried over).
sc, su = run([(GOOD, OPEN, "can"), (50, SHUT, "can")])
sc2, su2 = run([(50, SHUT, None)])  # the next trial: nothing done
run([(GOOD, OPEN, None)])  # an unbonded fill in one trial ...
sc3, _ = run([(GOOD, OPEN, "can"), (50, SHUT, "can")])  # ... does not poison the next
sc4, _ = run([(50, SHUT, None)], new_trial=False)  # no reset: the same trial carries on (clamp off after: still 1.0)
m.REQUIRE_ARMS_HOME = False
sc5, su5 = run([(GOOD, OPEN, "can"), (50, SHUT, "can", away)])
m.REQUIRE_ARMS_HOME = True
for flag, name in ((sc == 1.0 and su, "done and arms home: 1.0 and success"),
                   (sc2 == 0.0 and not su2, "a reset clears the fill: the next trial doing nothing scores 0"),
                   (sc3 == 1.0, "a reset clears the unbonded-flow latch"),
                   (sc4 == 1.0, "without a reset the history carries on"),
                   (sc5 == 1.0 and su5, "with REQUIRE_ARMS_HOME off, the job alone is 1.0")):
    bad += not flag
    print(("ok  " if flag else "FAIL"), name)

# The kit's attach rule, offline: a clamp bites only when a squeeze that had its jaws on the tab (in the tab's zone,
# pointing in, jaws closing across the plate) ends, and it is still there or dropping straight down from it.
kit._EPISODE["count"] += 1
can = st(CAN_AT, CAN_Q)
R_CAN = ck._rotm_xyzw(CAN_Q)
edge = CAN_AT + R_CAN @ np.array(m.TAB_EDGE)
fit = m.BOND.end
SQ = 0.22  # the opening measured in the sim with the fingers pressing on the clamp's 2.2 cm handles
UP, FAR = np.array([0.0, 0.0, 0.02]), np.array([0.0, 0.0, 0.10])


def clamp_at(jaw_world, quat, opening, grip_off=(0.0, 0.0, 0.0)):
    """One control step: the clamp's jaw point and orientation, the left gripper's opening and its tool point (at the
    clamp's grip point plus grip_off)."""
    R = ck._rotm_xyzw(quat)
    pos = jaw_world - R @ fit.jaw_point
    grip = pos + R @ fit.grip_point + np.asarray(grip_off, float)
    objs = {"can": can, "drum": st(m.DRUM), m.CLAMP: st(pos, quat), "left_gripper": st(grip),
            "right_gripper": st(PARK["right_gripper"])}
    kit._OPENING["left_gripper"] = opening
    kit._OPENING["right_gripper"] = 1.0
    ck._control_step(objs)
    return ck.attached(objs, m.CLAMP)


def scenario(*frames):
    """A fresh trial: per control step (jaw point, orientation, opening[, gripper offset]); the clamp's target at the
    end. True/False for the opening mean squeezing (0.22) / let go (fully open)."""
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = 0
    out = None
    for k, fr in enumerate(frames):
        kit._EPISODE["last"] = k + 1
        jw, q, op = fr[:3]
        op = SQ if op is True else (1.0 if op is False else op)
        out = clamp_at(jw, q, op, *fr[3:])
    return out


def qmul(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)


AWAY = (0.0, 0.0, math.sin(_a / 2 + math.pi / 2), math.cos(_a / 2 + math.pi / 2))
_s = math.sin(math.pi / 4)
ROLLED = qmul((m.IN[0] * _s, m.IN[1] * _s, 0.0, math.cos(math.pi / 4)), CLAMP_Q)  # rolled 90 degrees about its axis
_w = m.CAN_YAW + math.radians(90 - 50)  # a point on the can's wall 50 degrees around from the tab, at its height
wall = CAN_AT + np.array([m.CAN_R * math.cos(_w), m.CAN_R * math.sin(_w), m.TAB[2]])
_wa = _w + math.pi  # the clamp pointing straight into the wall there
WALL_Q = (0.0, 0.0, math.sin(_wa / 2), math.cos(_wa / 2))
handle = CAN_AT + R_CAN @ np.array([-0.012, 0.0, m.CAN_H + 0.018])
side = R_CAN @ np.array([1.0, 0.0, 0.0])  # across the tab plate
checks = [(scenario((edge, CLAMP_Q, True), (edge, CLAMP_Q, True)) is None, "held (squeezed open) on the tab: not bitten"),
          (scenario((edge, CLAMP_Q, True), (edge, CLAMP_Q, False)) == "can", "squeezed on the tab, let go: bites"),
          (scenario((edge - 0.004 * m.IN, CLAMP_Q, True), (edge - 0.004 * m.IN, CLAMP_Q, False)) == "can",
           "squeezed 4 mm out along the clamp, let go: bites"),
          (scenario((edge - 0.008 * m.IN, CLAMP_Q, True), (edge - 0.008 * m.IN, CLAMP_Q, False)) is None,
           "squeezed 8 mm out (only the jaw tips past the plate's edge), let go: no bite"),
          (scenario((edge - 0.02 * m.IN, CLAMP_Q, True), (edge - 0.02 * m.IN, CLAMP_Q, False)) is None,
           "squeezed 2 cm short, let go: no bite"),
          (scenario((edge, AWAY, True), (edge, AWAY, False)) is None, "at the tab but pointing away: no bite"),
          (scenario((edge - 0.08 * m.IN, CLAMP_Q, False), (edge, CLAMP_Q, False), (edge, CLAMP_Q, False)) is None,
           "never squeezed, pushed or dropped onto the tab: no bite (its jaws are shut)"),
          (scenario((edge, CLAMP_Q, True), (edge, CLAMP_Q, True), (edge - 0.012 * m.IN, CLAMP_Q, False)) == "can",
           "let go on the tab while it moves 1.2 cm as the fingers open: bites"),
          (scenario((edge, CLAMP_Q, True), (edge + np.array([0, 0, -0.037]), CLAMP_Q, False)) == "can",
           "let go on the tab, drops 3.7 cm straight down out of the opening fingers before the next step: bites"),
          (scenario((edge, CLAMP_Q, True), (edge - 0.03 * m.IN, CLAMP_Q, True), (edge - 0.03 * m.IN, CLAMP_Q, False))
           is None, "squeezed on the tab, backed off 3 cm still squeezed, then let go: no bite"),
          (scenario((edge, CLAMP_Q, True), (edge - 0.06 * m.IN, CLAMP_Q, False)) is None,
           "pulled 6 cm away while letting go: no bite"),
          (scenario((edge, CLAMP_Q, True), (edge - 0.2 * m.IN, CLAMP_Q, False), (edge - 0.2 * m.IN, CLAMP_Q, False),
                    (edge, CLAMP_Q, False)) is None, "squeezed on the tab, taken away, then dropped back on later: no bite"),
          (scenario((wall, WALL_Q, True), (wall, WALL_Q, False)) is None,
           "squeezed onto the can's painted wall 4 cm from the tab, pointing in, let go: no bite (paint insulates)"),
          (scenario((handle, CLAMP_Q, True), (handle, CLAMP_Q, False)) is None,
           "squeezed onto the can's carry handle, let go: no bite (only the bare tab is a bonding point)"),
          (scenario((edge, CLAMP_Q, True), (edge, CLAMP_Q, False), (edge, CLAMP_Q, True)) is None,
           "bitten, then gripped again: squeezed open, it lets go"),
          # holes found by an independent review (H1-H6)
          (scenario((edge - 0.05 * m.IN, CLAMP_Q, 0.0, UP), (edge, CLAMP_Q, 0.0, UP), (edge, CLAMP_Q, 0.0, FAR)) is None,
           "H1 a closed empty hand pushes the clamp onto the tab and lifts away: no bite"),
          (scenario((edge, CLAMP_Q, True), (edge, CLAMP_Q, False, FAR), (edge, CLAMP_Q, False, FAR),
                    (edge, CLAMP_Q, 0.0, np.array([0.0, 0.0, 0.024]))) == "can",
           "H2 a bitten clamp stays on when a hand closes empty 2.4 cm above it"),
          (scenario((edge, CLAMP_Q, True), (edge, CLAMP_Q, False, FAR), (edge, CLAMP_Q, 0.0, 0.015 * side)) == "can",
           "H2b a bitten clamp stays on when a closed hand rests beside it (not straddling its handles)"),
          (scenario((edge + 0.011 * side, CLAMP_Q, True), (edge + 0.011 * side, CLAMP_Q, False, FAR)) is None,
           "H3 jaws 1.1 cm beside the 3 mm plate: no bite"),
          (scenario((edge + R_CAN @ np.array([0.008, -0.008, 0.0]), CLAMP_Q, True),
                    (edge + R_CAN @ np.array([0.008, -0.008, 0.0]), CLAMP_Q, False, FAR)) is None,
           "H3b jaws 8 mm beside the plate, tips at the painted wall: no bite"),
          (scenario((edge + 0.003 * side, CLAMP_Q, True), (edge + 0.003 * side, CLAMP_Q, False)) == "can",
           "3 mm across the plate (jaws still on it): bites"),
          (scenario((edge, ROLLED, True), (edge, ROLLED, False, FAR)) is None,
           "H4 clamp rolled 90 degrees (jaws closing up and down, along the plate): no bite"),
          (scenario((edge, CLAMP_Q, True), (edge + np.array([0, 0, 0.03]), CLAMP_Q, True),
                    (edge + np.array([0, 0, 0.06]), CLAMP_Q, False, FAR)) is None,
           "H5 squeezed on the tab, lifted 6 cm while squeezed, let go there: no bite"),
          (scenario((edge, CLAMP_Q, True), (edge + np.array([0, 0, 0.02]), CLAMP_Q, False, FAR)) is None,
           "H5b squeezed on the tab, let go while moving 2 cm up: no bite (more than 1.5 cm off, not dropping)")]
_nk = R_CAN @ np.array(m.NECK)
_col = m._can().colliders[1].center  # the neck's collider (the drawn neck shares its center)
drawn = R_CAN @ np.array([_col[0], _col[1], m.NECK[2]])
checks.append((np.allclose(_nk, drawn) and np.linalg.norm((CAN_AT + _nk)[:2] - m.SPOUT[:2]) < 1e-6,
               "H6 the scored fill opening is the drawn neck, and it is right under the spout"))
_off = np.array([0.006, 0.0, 0.0])  # the can moved 6 mm: the 5 mm stream still lands in the 1.8 cm opening
checks.append((m._catches({"can": st(CAN_AT + _off, CAN_Q)}) and not m._catches({"can": st(CAN_AT + 2 * _off, CAN_Q)}),
               "H6b the stream is caught 6 mm from the opening's center (all inside it) and not at 1.2 cm (it hits the rim)"))
for flag, name in checks:
    bad += not flag
    print(("ok  " if flag else "FAIL"), name)
print("all pass" if not bad else f"{bad} FAILED")
raise SystemExit(1 if bad else 0)
