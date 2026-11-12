"""Offline checks of the piston and ring compressor scorer: step-by-step joint histories, near misses included, no
simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import os

import math

import numpy as np
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP as PAD_TIP
from yam_arena.demo import grip_rotation
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
DECK = m.TOP + m.DECK
HOME = {"left_gripper": (0.323, 0.30, m.TOP + 0.26), "right_gripper": (0.323, -0.30, m.TOP + 0.26)}
AWAY = {"left_gripper": (0.30, 0.25, m.TOP + 0.15), "right_gripper": (0.10, -0.35, m.TOP + 0.15)}  # 24 cm off the axis
ON_CROWN = lambda q, up=0.0: (m.BX, m.BY, DECK + q + PAD_TIP + up)  # noqa: E731  (fingertips on the crown, or `up` over it)
_hp = m.HANDLE_DIR * m.PRESS_R
HANDLE_TOP = m.HANDLE_Z + 0.005  # the handles' collider tops above the deck
ON_HANDLES = (m.BX + _hp[0], m.BY + _hp[1], DECK + HANDLE_TOP + PAD_TIP - 0.002)  # fingertips pressing the handles


def mat_quat(R):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


DOWN = mat_quat(grip_rotation(0.0))  # grippers point straight down, fingers closed (opening 0) unless set


def st(pos, q=(0, 0, 0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
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
    objs = {"engine": st((m.BX, m.BY, DECK)), **{k: st(p, DOWN) for k, p in g.items()}}
    kit._OPENING.update({k: OPEN.get(k, 0.0) for k in g})
    ok = m.success(objs)
    return m.score(objs), ok


OPEN = {}  # grippers' openings (default closed)


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
    """Arms parked away from home at the end: the job alone earns full credit."""
    new_trial()
    finish(m.START, n=3)
    drive(m.START, m.SEAT)
    return finish(m.SEAT, park=AWAY), 1.0


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


FAST = [0.022 - 0.002 * k for k in range(4)] + [0.016, 0.012, 0.008, 0.004, 0.0, -0.004, -0.008, -0.010]


def with_law(qs, held):
    """Play crown heights qs through the compressor's law: an unheld band is at its modeled 6.7 mm ride-up at the
    step after the law pushed it (the push ramps it up within one 50 ms step), a held one stays on the deck."""
    cl, mem, pushed, out = m.CompressorLaw(), {}, False, None
    for q in qs:
        up = 0.0067 if (pushed and not held) else 0.0
        out = step(q, up, right=ON_CROWN(q), left=ON_HANDLES if held else None)
        _, k_, _, _ = cl.drive("engine/compressor", up, 0.0, {"piston": q, "compressor": up}, mem)
        pushed = k_ > 0.0
    return out


@case
def fast_with_compressor_held():
    """Review, Nov 11: a fast stroke (8 cm/s, 4 mm per 50 ms step, each ring from above the entry to below the deck in
    one step) with the compressor held flat on the deck: no ring escapes, so the job counts."""
    new_trial()
    finish(m.START, n=3)
    with_law(FAST, held=True)
    return finish(m.SEAT), 1.0


@case
def one_handed_fast_between_zones():
    """The same fast stroke with nobody holding the compressor: the samples fall between the ring zones, but the
    compressor's law tests the ring's path, so the band rides up and the next ring's path crosses the open gap."""
    new_trial()
    finish(m.START, n=3)
    with_law(FAST, held=False)
    return finish(m.SEAT), 0.0


@case
def fast_band_up_between_samples():
    """A ring's whole path through a 3 mm gap falls between two samples (above the band's lip, then in the bore):
    still an escape."""
    new_trial()
    finish(m.START, n=3)
    step(0.0055 + 0.003, 0.0)  # top ring 3 mm above the deck, still in the band
    step(0.0055 + 0.003, 0.0029)  # band 2.9 mm up: its lower edge just under the ring, which is still in it
    step(0.0055 - 0.002, 0.0029)  # ring 2 mm into the bore: its path went through the gap
    return finish(m.SEAT), 0.0


@case
def band_lifted_ring_still_in_band():
    """The band 1.5 mm up while every ring is still 3 mm or more above the deck (inside the band, above its lower
    edge), then pressed back down before any ring reaches the gap: nothing escaped."""
    new_trial()
    finish(m.START, n=3)
    drive(m.START, 0.0085)  # top ring 3 mm above the deck
    for _ in range(5):
        step(0.0085, 0.0015, left=ON_HANDLES)
    step(0.0085, 0.0, left=ON_HANDLES)
    drive(0.0085, m.SEAT)
    return finish(m.SEAT), 1.0


@case
def piston_pulled_up_out_of_band():
    """The piston pulled up until the top ring's middle is past the band's top edge: it springs open above the band."""
    new_trial()
    finish(m.START, n=3)
    for q in np.linspace(m.START, m.START + 0.002, 5):
        step(q, 0.0, right=ON_CROWN(q))
    drive(m.START + 0.002, m.SEAT)
    return finish(m.SEAT), 0.0


@case
def piston_nudged_up_inside_band():
    """Pulled up 0.8 mm: the top ring's middle is still 0.2 mm under the band's top edge, so it stays squeezed."""
    new_trial()
    finish(m.START, n=3)
    for q in np.linspace(m.START, m.START + 0.0008, 5):
        step(q, 0.0, right=ON_CROWN(q))
    drive(m.START + 0.0008, m.SEAT)
    return finish(m.SEAT), 1.0


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


def end_with(q=None, right=None, left=None, n=30, lift=0.0):
    """The correct drive to the seat (or to q), then the hands left at these poses for the last n steps."""
    new_trial()
    finish(m.START, n=3)
    q = m.SEAT if q is None else q
    drive(m.START, q)
    out = None
    for _ in range(n):
        out = step(q, lift, right=right, left=left)
    return out


@case
def still_pushing_at_end():
    """Fingertips still on the crown: the rings are in, but the job isn't finished until the hand lets go."""
    return end_with(right=ON_CROWN(m.SEAT)), 0.6


@case
def still_holding_compressor_at_end():
    return end_with(left=ON_HANDLES), 0.6


@case
def both_hands_on_at_end():
    return end_with(right=ON_CROWN(m.SEAT), left=ON_HANDLES), 0.6


@case
def hand_grasping_a_handle_at_end():
    """Fingers closed around the handle bars from above (pads either side, tool point at the bars' height)."""
    p = m.HANDLE_DIR * m.PRESS_R
    return end_with(left=(m.BX + p[0], m.BY + p[1], DECK + m.HANDLE_Z)), 0.6


@case
def held_band_flat_over_a_crossing_ring_at_end():
    """The old loophole the hands-off rule guarded: the band held flat while the top ring is still crossing (2 mm above
    the deck). Only the two rings wholly in the bore count, and they stay in when the hand lets go."""
    return end_with(q=0.0075, left=ON_HANDLES, right=ON_CROWN(0.0075)), 0.4


@case
def fingertips_3mm_over_crown_at_end():
    """Let go: closed fingertips hovering 3 mm over the crown, touching nothing (the old 21 cm zone scored 0)."""
    return end_with(right=ON_CROWN(m.SEAT, 0.003)), 1.0


@case
def hand_hovering_over_bore_at_end():
    """Fingertips 2.5 cm above the band's top over the bore (the old zone scored this 0)."""
    return end_with(right=(m.BX, m.BY, DECK + m.BAND_H + 0.025 + PAD_TIP)), 1.0


@case
def fist_hovering_5mm_over_handles_at_end():
    p = m.HANDLE_DIR * m.PRESS_R
    return end_with(left=(m.BX + p[0], m.BY + p[1], DECK + HANDLE_TOP + PAD_TIP + 0.005)), 1.0


@case
def open_hand_beside_band_at_end():
    """An open hand parked with its finger 1 cm beside the band's lip, at the side away from the handles (the old zone
    scored this 0)."""
    OPEN["left_gripper"] = 1.0
    try:
        pos = (m.BX, m.BY - m.BAND_OUT - 0.0015 - 0.01 - (abs(FINGER_OPEN) + 0.0064), DECK + 0.03)  # tip 1 cm out
        out = end_with(left=pos)
        objs = {"left_gripper": st(pos, DOWN), "compressor": m._on_axis(0.0)}
        gap = m.gk.hand_gap(objs, "left_gripper", "compressor", m.COMPRESSOR_SHAPE)
        assert abs(gap - 0.01) < 1e-4, gap  # the pose is what it says: 1 cm from the band, not touching
        return out, 1.0
    finally:
        OPEN.clear()


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
    # a fast stroke: the top ring at 4.5 mm above the deck, moving 4 mm a step, will skip the 2.5 mm zone between
    # samples; the compressor's law still pushes the band up (and the piston's law adds the crossing drag and push)
    mem_c = {"piston": 0.0055 + 0.0085}
    tgt, k_, _, _ = cl.drive("engine/compressor", 0.0, 0.0, {"piston": 0.0055 + 0.0045, "compressor": 0.0}, mem_c)
    assert k_ == m.LIFT_K, "a ring about to cross between samples pushes the band"
    mem = pl.fresh(L)
    mem["prev"] = 0.0055 + 0.0085
    t, k, e, d = pl.drive("engine/piston", 0.0055 + 0.0045, 0.0, {"compressor": 0.0, "piston": 0.0}, mem)
    assert e == m.RING_PUSH and mem["jam"] is None
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
