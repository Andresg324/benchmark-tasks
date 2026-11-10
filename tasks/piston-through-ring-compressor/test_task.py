"""Offline checks of the piston and ring compressor scorer: step-by-step joint histories, near misses included, no
simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
DECK = m.TOP + m.DECK
HOME = {"left_gripper": (0.323, 0.30, m.TOP + 0.26), "right_gripper": (0.323, -0.30, m.TOP + 0.26)}
AWAY = {"left_gripper": (0.25, 0.25, m.TOP + 0.20), "right_gripper": (0.25, -0.30, m.TOP + 0.20)}
ON_CROWN = lambda q: (m.BX, m.BY, DECK + q + 0.013)  # noqa: E731
_hp = m.HANDLE_DIR * m.PRESS_R
ON_HANDLES = (m.BX + _hp[0], m.BY + _hp[1], DECK + 0.02)


def st(pos):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0, 0, 0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def step(piston, lift=0.0, right=None, left=None, park=HOME):
    """One control step: set the joints and grippers, call success() (which records the history), return score()."""
    kit._JOINTS["engine/piston"] = kit.JointState(piston, 0.0, False)
    kit._JOINTS["engine/compressor"] = kit.JointState(lift, 0.0, False)
    g = dict(park)
    if right is not None:
        g["right_gripper"] = right
    if left is not None:
        g["left_gripper"] = left
    objs = {"engine": st((m.BX, m.BY, DECK)), **{k: st(p) for k, p in g.items()}}
    ok = m.success(objs)
    return m.score(objs), ok


def new_trial():
    kit._EPISODE["count"] += 1


def drive(q0, q1, held=True, lift_fn=None, n=60):
    """The right hand pushes the crown from q0 to q1; the left holds the handles (or not). lift_fn(q) gives the
    compressor's lift at each step (default: seated while held, the modeled 6.7 mm ride-up while a ring crosses
    otherwise)."""
    out = None
    for k in range(1, n + 1):
        q = q0 + (q1 - q0) * k / n
        if lift_fn is not None:
            up = lift_fn(q)
        else:
            up = 0.0 if held else (0.0067 if m.crossing(q) else 0.0)
        out = step(q, up, right=ON_CROWN(q), left=ON_HANDLES if held else None)
    return out


def finish(q, lift=0.0, n=30, park=HOME):
    out = None
    for _ in range(n):
        out = step(q, lift, park=park)
    return out


CASES = {}


def case(f):
    CASES[f.__name__] = f
    return f


@case
def nothing():
    new_trial()
    return finish(m.START, n=60), 0.0


@case
def correct():
    new_trial()
    finish(m.START, n=3)
    drive(m.START, m.SEAT)
    return finish(m.SEAT), 1.0


@case
def correct_arms_not_home():
    new_trial()
    finish(m.START, n=3)
    drive(m.START, m.SEAT)
    return finish(m.SEAT, park=AWAY), 0.9


@case
def unheld_push():
    """Drive it without holding the compressor: it rides up at the first ring."""
    new_trial()
    finish(m.START, n=3)
    drive(m.START, m.SEAT, held=False)
    return finish(m.SEAT), 0.0


@case
def held_until_last_ring():
    """Holding for the first two rings in (the oil ring enters first), letting go for the top ring."""
    new_trial()
    finish(m.START, n=3)
    drive(m.START, 0.0075)
    drive(0.0075, m.SEAT, held=False)
    return finish(m.SEAT), 0.0


@case
def escaped_then_fixed():
    """A ring escaped, then the band was pressed back down and the piston pushed home: still 0 (irreversible)."""
    new_trial()
    finish(m.START, n=3)
    drive(m.START, 0.0061, held=False)
    drive(0.0061, m.SEAT)
    return finish(m.SEAT), 0.0


@case
def lift_just_under_tol():
    """The band 0.9 mm up at every crossing: under the 1 mm a ring needs, so clean."""
    new_trial()
    finish(m.START, n=3)
    drive(m.START, m.SEAT, lift_fn=lambda q: 0.0009 if m.crossing(q) else 0.0)
    return finish(m.SEAT), 1.0


@case
def lift_just_over_tol():
    new_trial()
    finish(m.START, n=3)
    drive(m.START, m.SEAT, lift_fn=lambda q: 0.0012 if m.crossing(q) else 0.0)
    return finish(m.SEAT), 0.0


@case
def jump_across_deck_while_lifted():
    """The ring crosses the deck plane between two steps (never sampled in the gap) with the band lifted."""
    new_trial()
    step(0.0055 + 0.001, 0.0)
    step(0.0055 + 0.001, 0.003)
    step(0.0045 - 0.001, 0.003)
    finish(0.0045 - 0.001, n=5)
    return finish(m.SEAT), 0.0


@case
def one_handed_fast_between_zones():
    """Review, Nov 10: one hand drives at 8 cm/s (4 mm per step), the other at home; the samples fall between the ring
    zones, so the band is never seen crossing. Each ring goes from above the entry to below the deck in one step:
    slammed through, it escapes."""
    new_trial()
    finish(m.START, n=3)
    for q in [0.022 - 0.002 * k for k in range(4)] + [0.016, 0.012, 0.008, 0.004, 0.0, -0.004, -0.008, -0.010]:
        step(q, 0.0, right=ON_CROWN(q))
    return finish(-0.010), 0.0


@case
def compressor_pulled_up_early():
    """The compressor lifted off the piston before any ring entered: the rings spring out above the deck."""
    new_trial()
    finish(m.START, n=3)
    for up in np.linspace(0.0, 0.03, 20):
        step(m.START, up, left=ON_HANDLES)
    return finish(m.START, lift=0.0), 0.0


@case
def compressor_lifted_with_rings_in():
    """Lifting the empty compressor after all three rings are in harms nothing."""
    new_trial()
    finish(m.START, n=3)
    drive(m.START, m.SEAT)
    for up in np.linspace(0.0, 0.03, 20):
        step(m.SEAT, up, left=ON_HANDLES)
    for up in np.linspace(0.03, 0.0, 20):
        step(m.SEAT, up, left=ON_HANDLES)
    return finish(m.SEAT), 1.0


@case
def still_pushing_at_end():
    new_trial()
    finish(m.START, n=3)
    drive(m.START, m.SEAT)
    out = None
    for _ in range(30):
        out = step(m.SEAT, right=ON_CROWN(m.SEAT))
    return out, 0.0


@case
def still_holding_compressor_at_end():
    new_trial()
    finish(m.START, n=3)
    drive(m.START, m.SEAT)
    out = None
    for _ in range(30):
        out = step(m.SEAT, left=ON_HANDLES)
    return out, 0.0


@case
def one_ring():
    new_trial()
    finish(m.START, n=3)
    drive(m.START, 0.0115)
    return finish(0.0115), 0.2


@case
def two_rings():
    new_trial()
    finish(m.START, n=3)
    drive(m.START, 0.0075)
    return finish(0.0075), 0.4


@case
def three_rings_not_home():
    """All three rings in, crown 3 mm below the deck, 7 mm short of its seat."""
    new_trial()
    finish(m.START, n=3)
    drive(m.START, -0.003)
    return finish(-0.003), 0.6


@case
def just_short_of_seat():
    """Crown 2.5 mm above its seat: past the 2 mm tolerance."""
    new_trial()
    finish(m.START, n=3)
    drive(m.START, m.SEAT + 0.0025)
    return finish(m.SEAT + 0.0025), 0.6


@case
def ring_on_chamfer():
    """The top ring 0.5 mm below the deck, still on the chamfer: not counted as in."""
    new_trial()
    finish(m.START, n=3)
    drive(m.START, 0.0045)
    return finish(0.0045), 0.4


@case
def only_hold_compressor():
    """Pressing the compressor and never driving the piston: no credit."""
    new_trial()
    for _ in range(40):
        step(m.START, left=ON_HANDLES)
    return finish(m.START), 0.0


@case
def history_reset():
    """A failed trial, then a scene reset and a clean trial: the new trial is scored on its own."""
    new_trial()
    drive(m.START, m.SEAT, held=False)
    new_trial()
    finish(m.START, n=3)
    drive(m.START, m.SEAT)
    return finish(m.SEAT), 1.0


def law_checks():
    """The kit laws: the piston jams once a ring escapes, the compressor is pushed up only while a ring crosses."""
    pl, cl = m.PistonLaw(), m.CompressorLaw()

    class L:
        start = m.START

    mem = pl.fresh(L)
    t, k, e, d = pl.drive("engine/piston", m.START, 0.0, {"compressor": 0.0, "piston": m.START}, mem)
    assert k == 0.0 and e == 0.0, "no ring crossing at the start"
    q = 0.0055 + 0.001  # top ring 1.5 mm above the deck: crossing
    mem["prev"] = q + 0.001  # arrived there in 1 mm steps (a jump from the start would slam the lower rings through)
    t, k, e, d = pl.drive("engine/piston", q, 0.0, {"compressor": 0.0, "piston": q}, mem)
    assert e == m.RING_PUSH and mem["jam"] is None
    tgt, k_, _, _ = cl.drive("engine/compressor", 0.0, 0.0, {"piston": q, "compressor": 0.0}, {})
    assert abs(k_ * (tgt - 0.0) - m.LIFT_N) < 1e-9, (tgt, k_)
    _, k_, lift_e, _ = cl.drive("engine/compressor", 0.0, 0.0, {"piston": m.START, "compressor": 0.0}, {})
    assert lift_e == 0.0 and k_ == 0.0
    t, k, e, d = pl.drive("engine/piston", q, 0.0, {"compressor": 0.004, "piston": q}, mem)
    assert mem["jam"] == q and mem["escaped"] == [0] and k == m.JAM_HOLD
    # equilibrium ride-up of an unheld compressor: lift = weight
    up = (m.LIFT_N - m.COMP_MASS * 9.81) / m.LIFT_K
    assert 0.005 < up < 0.008, up


if __name__ == "__main__":
    bad = 0
    for name, f in CASES.items():
        (got, ok), want = f()
        flag = "ok " if abs(got - want) < 1e-9 else "BAD"
        bad += flag == "BAD"
        print(f"{flag} {name:34s} score {got:.2f} (want {want:.2f}) success {ok}")
    law_checks()
    print("law checks ok")
    print("all pass" if bad == 0 else f"{bad} FAILED")
