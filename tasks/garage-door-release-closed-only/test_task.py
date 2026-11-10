"""Offline checks of the garage door's scorer: step-by-step joint histories, near misses included, no simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
S0 = m.DOOR_START


def st(pos):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def lift(q):
    """The tool point holding the door's lift handle with the door at q."""
    return (m.LIFT_X - 0.006, m.LIFT_Y, m.TOP + m.FLOOR + q + m.LIFT_Z + 0.004)


def step(door, pull=0.0, vel=None, right=None, left=None):
    """One control step: set the joints and grippers, call success() (which records the history), return score()."""
    v = 0.0 if vel is None else vel
    kit._JOINTS["garage/door"] = kit.JointState(door, v, False)
    kit._JOINTS["garage/release"] = kit.JointState(pull, 0.0, False)
    g = dict(PARK)
    if right is not None:
        g["right_gripper"] = right
    if left is not None:
        g["left_gripper"] = left
    objs = {"garage": st(m.BASE), **{k: st(p) for k, p in g.items()}}
    ok = m.success(objs)
    return m.score(objs), ok


def new_trial():
    kit._EPISODE["count"] += 1


def lower(q0, q1, secs, held=True):
    """The door moved from q0 to q1 over secs, held at its handle (or not), at the speed that implies."""
    n = max(1, int(round(secs / m.DT)))
    v = (q1 - q0) / (n * m.DT)
    out = None
    for k in range(1, n + 1):
        q = q0 + (q1 - q0) * k / n
        out = step(q, vel=v, right=lift(q) if held else None)
    return out


def pull_release(door=S0, held=True):
    for p in (-0.01, -0.02, -0.03, -0.035, -0.02, 0.0):
        step(door, p, right=lift(door) if held else None)


def rest(door, n=10, held=False, pull=None):
    out = None
    for _ in range(n):
        out = step(door, pull if pull is not None else door - S0, right=lift(door) if held else None)
    return out


def case_nothing():
    new_trial()
    return rest(S0, 30, pull=0.0)


def case_correct():
    """Hold the door, pull the release, lower it over 4 s, let go, arms home."""
    new_trial()
    rest(S0, 2, pull=0.0)
    for _ in range(5):
        step(S0, right=lift(S0))
    pull_release()
    lower(S0, 0.0, 4.0)
    rest(0.0, 3, held=True)
    return rest(0.0, 10)


def case_drop():
    """The tempting answer: pull the release with the door open and nobody holding it; the door falls 9 cm."""
    new_trial()
    rest(S0, 2, pull=0.0)
    pull_release(held=False)
    q, v = S0, 0.0
    while q > 0:
        v += 9.81 * m.DT
        q = max(0.0, q - v * m.DT)
        step(q, -0.02, vel=-v)
    return rest(0.0, 20)


def case_drop_then_reopen_and_lower():
    """Dropped, then lifted back up and lowered properly: the drop is irreversible."""
    case_drop()
    lower(0.0, S0, 3.0)
    lower(S0, 0.0, 4.0)
    return rest(0.0, 10)


def case_held_then_let_go_midway():
    """Released while held, lowered halfway, then let go: it falls the rest of the way."""
    new_trial()
    rest(S0, 2, pull=0.0)
    pull_release()
    lower(S0, 0.045, 2.0)
    q, v = 0.045, 0.0
    while q > 0:
        v += 9.81 * m.DT
        q = max(0.0, q - v * m.DT)
        step(q, vel=-v)
    return rest(0.0, 10)


def case_slammed():
    """Released while held, then pushed down fast (0.6 m/s): slammed counts as dropped."""
    new_trial()
    rest(S0, 2, pull=0.0)
    pull_release()
    lower(S0, 0.0, 0.15)
    return rest(0.0, 10)


def case_fast_hand_ok():
    """Lowered by hand at 0.2 m/s (under the 0.25 m/s line), then let go: full score."""
    new_trial()
    rest(S0, 2, pull=0.0)
    pull_release()
    lower(S0, 0.0, 0.45)
    return rest(0.0, 10)


def case_still_held_at_end():
    new_trial()
    rest(S0, 2, pull=0.0)
    pull_release()
    lower(S0, 0.0, 4.0)
    return rest(0.0, 10, held=True)


def case_released_not_closed():
    """Held, released, lowered to 4 cm and still held there at the end."""
    new_trial()
    rest(S0, 2, pull=0.0)
    pull_release()
    lower(S0, 0.04, 2.0)
    return rest(0.04, 10, held=True)


def case_released_held_open():
    """Held and released, then simply held up where it started: no closing done."""
    new_trial()
    rest(S0, 2, pull=0.0)
    pull_release()
    return rest(S0, 10, held=True)


def case_forced():
    """Pushed down 2 cm while the trolley still holds it, then released and lowered properly."""
    new_trial()
    rest(S0, 2, pull=0.0)
    for q in (0.085, 0.078, 0.070, 0.080, S0):
        step(q, 0.0, right=lift(q))
    pull_release()
    lower(S0, 0.0, 4.0)
    return rest(0.0, 10)


def case_nudged():
    """A 1 cm nudge while connected (under the 1.5 cm force line), then the proper procedure."""
    new_trial()
    rest(S0, 2, pull=0.0)
    for q in (0.085, 0.080, 0.085, S0):
        step(q, 0.0, right=lift(q))
    pull_release()
    lower(S0, 0.0, 4.0)
    return rest(0.0, 10)


def case_half_pull():
    """The red handle pulled 2 cm only (not tripped): the door stays connected and open."""
    new_trial()
    rest(S0, 2, pull=0.0)
    for p in (-0.01, -0.02, -0.01, 0.0):
        step(S0, p, right=lift(S0))
    return rest(S0, 10, pull=0.0)


def case_gap_4mm():
    """Lowered properly but left 4 mm above the floor (held there): not closed."""
    new_trial()
    rest(S0, 2, pull=0.0)
    pull_release()
    lower(S0, 0.004, 4.0)
    return rest(0.004, 10, held=True)


def case_hovering_hand():
    """Closed and let go, but a tool point left 4 cm from the lift handle: still counts as holding."""
    new_trial()
    rest(S0, 2, pull=0.0)
    pull_release()
    lower(S0, 0.0, 4.0)
    near = lift(0.0)
    near = (near[0] - 0.04, near[1], near[2])
    out = None
    for _ in range(10):
        out = step(0.0, -S0, right=near)
    return out


def case_unsupported_then_caught():
    """Release pulled with no hand on the door, then the door grabbed before it gained speed and lowered properly:
    pulling the release on an unsupported door is irreversible even if the door is caught."""
    new_trial()
    rest(S0, 2, pull=0.0)
    pull_release(held=False)
    for _ in range(3):
        step(S0, -0.02, right=lift(S0))
    lower(S0, 0.0, 4.0)
    return rest(0.0, 10)


def case_supported_from_below():
    """The door carried from under its bottom edge instead of by the lift handle, released, lowered, let go."""
    new_trial()
    rest(S0, 2, pull=0.0)
    under = lambda q: (m.DOOR_X - 0.02, 0.05, m.TOP + m.FLOOR + q - 0.01)  # noqa: E731
    for p in (-0.01, -0.02, -0.03, -0.02, 0.0):
        step(S0, p, right=under(S0))
    lower(S0, 0.0, 4.0)
    return rest(0.0, 10)


def case_law_flag_without_joint():
    """The door law's latched "released" flag set, but the red handle never actually pulled: the scorer trusts the
    joint, so the door counts as never released (and, closed by pushing, as forced)."""
    new_trial()
    kit.law_state("garage/door")["released"] = True
    rest(S0, 2, pull=0.0)
    lower(S0, 0.0, 4.0)
    out = rest(0.0, 10, pull=0.0)
    kit.law_state("garage/door").pop("released", None)
    return out


CASES = [
    ("nothing done", case_nothing, 0.0),
    ("held, released, lowered, let go, arms home", case_correct, 1.0),
    ("release pulled with the door open and unheld (the tempting answer)", case_drop, 0.0),
    ("dropped, then lifted and lowered properly (irreversible)", case_drop_then_reopen_and_lower, 0.0),
    ("held and released, let go halfway down", case_held_then_let_go_midway, 0.0),
    ("held and released, then pushed down fast", case_slammed, 0.0),
    ("held and released, lowered at 0.2 m/s", case_fast_hand_ok, 1.0),
    ("closed but still held at the end", case_still_held_at_end, 0.5),
    ("released safely, held 4 cm open at the end", case_released_not_closed, 0.25),
    ("released safely, held where it started", case_released_held_open, 0.25),
    ("forced 2 cm against the connected opener", case_forced, 0.0),
    ("nudged 1 cm while connected, then the proper procedure", case_nudged, 1.0),
    ("release handle pulled 2 cm only", case_half_pull, 0.0),
    ("left 4 mm above the floor", case_gap_4mm, 0.25),
    ("closed, a tool point hovering 4 cm from the handle", case_hovering_hand, 0.5),
    ("release pulled unsupported, door caught at once and lowered (irreversible)", case_unsupported_then_caught, 0.0),
    ("door carried from under its bottom edge, released, lowered", case_supported_from_below, 1.0),
    ("law flag says released but the handle joint never tripped", case_law_flag_without_joint, 0.0),
]

bad = 0
for name, fn, want in CASES:
    got, _ = fn()
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

checks = []
# a drop in one trial does not carry into the next
case_drop()
checks.append((case_correct()[0] == 1.0, "a new trial starts with a clean history"))
# success needs the full job and the arms home
got, ok = case_correct()
checks.append((ok, "the correct procedure counts as success"))
got, ok = case_still_held_at_end()
checks.append((not ok, "closed but still held is not success"))
# arms home: a finished job with a gripper parked away from home scores 0.9
new_trial()
rest(S0, 2, pull=0.0)
pull_release()
lower(S0, 0.0, 4.0)
away = (0.25, -0.15, m.TOP + 0.15)
step(0.0, -S0, right=away)
sc_away, su_away = step(0.0, -S0, right=away)
checks.append((sc_away == 0.9 and not su_away, "done, a gripper left away from home: 0.9, no success"))
m.REQUIRE_ARMS_HOME = False
step(0.0, -S0, right=away)
sc_off, su_off = step(0.0, -S0, right=away)
m.REQUIRE_ARMS_HOME = True
checks.append((sc_off == 1.0 and su_off, "with REQUIRE_ARMS_HOME off, the job alone is 1.0"))
# a door still moving when it reaches the floor is not yet at rest
new_trial()
rest(S0, 2, pull=0.0)
pull_release()
lower(S0, 0.001, 4.0)
sc_moving, _ = step(0.0, -S0, vel=-0.02)
checks.append((sc_moving == 0.5, "closed but still moving: 0.5"))
for flag, name in checks:
    bad += not flag
    print(("ok  " if flag else "FAIL"), name)
print("all pass" if not bad else f"{bad} FAILED")
raise SystemExit(1 if bad else 0)
