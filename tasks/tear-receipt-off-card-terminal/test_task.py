"""Offline checks of the receipt-tear task, no simulator. The receipt's strips are posed by hand from the sheet's own
forward kinematics (each strip's lean), the terminal and tray from their poses, and the tear runs through task.py's
own Tear (tie loads from the poses, the teeth from the strip at the slot's contact with the bar's edge).

Covers: the start state (ties unloaded, paper off the teeth and the back lip, standing below its self-buckling length,
nothing scored); doing nothing; a pull over the teeth tears it off edge first (0.5), then flat in the tray, released
and still (1.0); held or touching the tray (0.75); the cheats: a straight pull out of the slot, a sideways rip without
the bar, a pull over the smooth back lip, pulling the roll out, pushing the attached receipt onto the tray, the
terminal moved off its mark or tipped over, the receipt dropped off the counter; a stalled half-tear (0.25); a new
trial; the tray check at 300 tray placements; and the "does a pinch load the ties" check (grip, arm and tear forces
along the demo's swing, pads on the paper, the swing drawing past the paper's length).

    python3 test_task.py
"""
import json
import math
from pathlib import Path

import numpy as np
from yam_arena.constants import GRIPPER
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import guardkit as gk  # noqa: E402
import yamgeom as yg  # noqa: E402
import sheetkit as sk  # noqa: E402
import yamkit as kit  # noqa: E402

AWAY = {"left_gripper": (0.15, 0.30, M.TOP + 0.30), "right_gripper": (0.15, -0.30, M.TOP + 0.30)}
SH = M.RECEIPT_SHEET
N = SH.n
DOWN = (1.0, 0.0, 0.0, 0.0)  # tool z pointing down (approach), xyzw


def check(cond, what):
    if not cond:
        raise AssertionError(what)


def st(pos, R=None, q=None, half=(0.01, 0.01, 0.01)):
    pos = np.asarray(pos, float)
    if q is None:
        q = sk.R_quat(np.eye(3) if R is None else R)
    return ObjectState(pos=pos, quat_xyzw=np.asarray(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=pos - np.asarray(half), high=pos + np.asarray(half))


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


# -- posing the receipt: absolute leans (rad, toward the terminal's printer end +x; negative = over the tear bar) -----
def chain_frames(phis, bottom_world, R_extra=np.eye(3)):
    """Strip frames with strip i leaning phis[i] about the width, the strip at the slot's bottom edge middle at
    bottom_world; R_extra turns the whole receipt about that point (terminal-frame rotation) before placing it."""
    phis = np.asarray(phis, float)
    r = SH.root
    q = np.array([(phis[j] - phis[j + 1]) if j < r else (phis[j + 1] - phis[j]) for j in range(N - 1)])
    base_R = M.R_TERM @ R_extra @ M.R_SHEET @ sk.rot_x(phis[r])
    fr = SH.forward(q, base_pos=np.zeros(3), base_R=base_R)
    p0, R0 = fr[0]
    b = p0 + R0 @ np.array([0.0, -M.S0_LEN, 0.0])
    shift = np.asarray(bottom_world, float) - b
    return [(p + shift, R) for p, R in fr], q


def anchor_world(local=M.TIE_MID, term_pos=M.TERM_POS, term_R=M.R_TERM):
    return np.asarray(term_pos) + term_R @ np.asarray(local)


def objects_for(fr, q=None, term_pos=M.TERM_POS, term_R=M.R_TERM, tray_pos=M.TRAY_POS, tray_R=np.eye(3),
                grippers=None, opening=1.0):
    o = {}
    for i, (p, R) in enumerate(fr):
        o[SH.key(i)] = st(p, R)
    o["terminal"] = st(term_pos, term_R)
    o["tray"] = st(tray_pos, tray_R)
    for g, p in (grippers or AWAY).items():
        o[g] = st(p, q=DOWN)
        kit._OPENING[g] = opening
    if q is not None:
        for j in range(N - 1):
            kit._JOINTS[f"receipt/s{SH.strip_of_hinge(j)}"] = kit.JointState(float(q[j]), 0.0, False)
    return o


def start_frames():
    return chain_frames(np.zeros(N), anchor_world())


def new_trial():
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None


def run(seq):
    """Score a sequence of objects dicts as consecutive control steps of one trial; returns the scores."""
    new_trial()
    out = []
    for k, o in enumerate(seq):
        kit._EPISODE["last"] = k
        M.success(o)
        out.append(M.score(o))
    return out


S0_UP = M.R_TERM @ np.array([0.0, 0.0, 1.0])
TEETH_ANGLE = None


def teeth_angle():
    """The smallest lean over the bar (rad, negative) at which the strip at the slot touches the teeth's edge."""
    for d in np.arange(0.0, 30.0, 0.05):
        fr, q = chain_frames(np.full(N, math.radians(d)), anchor_world())
        if M.on_teeth(objects_for(fr, q)).all():
            return math.radians(d)
    raise AssertionError("the paper never reaches the teeth")


def pulled_over(stretch, lean_deg=60.0, corner=0.0):
    """The receipt wrapped over the teeth (the strip at the slot on the edge, the rest bent lean_deg over the bar) and
    drawn `stretch` m along the strip at the slot's own direction; `corner` (rad) turns it in its plane about the
    far edge's tie, so the near edge is pulled first."""
    a = teeth_angle()
    phis = np.array([a] + [math.radians(lean_deg)] * (N - 1))
    fr, q = chain_frames(phis, anchor_world())
    d = fr[0][1] @ np.array([0.0, 1.0, 0.0])
    fr = [(p + stretch * d, R) for p, R in fr]
    if corner:
        pivot = fr[0][0] + fr[0][1] @ np.array(M.TIE_STRIP_PTS[-1])
        Rn = sk.quat_R(sk.R_quat(np.eye(3)))
        axis = fr[0][1] @ np.array([0.0, 0.0, 1.0])  # the paper's normal
        K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
        Rc = Rn + math.sin(corner) * K + (1 - math.cos(corner)) * K @ K
        fr = [(pivot + Rc @ (p - pivot), Rc @ R) for p, R in fr]
    return fr, q


def flat_in_tray_frames(tray_pos=M.TRAY_POS, tray_R=np.eye(3), yaw_in_tray=0.0, z=0.0006, shift=0.0):
    """The torn receipt lying flat on the tray floor, along the tray's length, turned yaw_in_tray in it."""
    fr = SH.forward(np.zeros(N - 1), base_pos=np.zeros(3), base_R=np.eye(3))
    R = tray_R @ sk.quat_R(sk.R_quat(M.rz(yaw_in_tray)))
    c = np.asarray(tray_pos) + tray_R @ np.array([0.0, shift, z])
    return [(c + R @ p, R @ Rr) for p, Rr in fr], np.zeros(N - 1)


GRIP_N = GRIPPER["stiffness"] * M.PAPER_T / 2  # N per finger: closed target, the paper's collider between the pads
PAD_MU = max(1.0, M.THERMAL.friction)  # sheetkit's rubber pads, combine "max"
GRIP_CAP = 2 * PAD_MU * GRIP_N  # N: the pull a closed pinch on the paper holds before it slips


# -- tests ------------------------------------------------------------------------------------------------------------
def test_paper_numbers():
    check(abs(M.TIE_TENSILE - 2100.0 * 0.058 / 8) < 1e-9, "intact tie = tensile MD x its share of the width")
    check(M.TEAR_CD == 0.220 and M.THERMAL.areal_mass == 0.048 and M.PAPER_TH == 55e-6, "Jujo KSND-48 values")
    check(M.N_TIES * M.TEAR_CD < GRIP_CAP < M.TIE_TENSILE, "an even pull over the teeth tears inside the grip; an "
          "intact tie holds far more than the grip")
    check(abs(sum(SH.widths) - M.RECEIPT_L) < 1e-12 and abs(SH.widths[0] - 2 * M.TIE_DEPTH) < 1e-12, "strips")
    check(max(SH.widths[1:]) <= sk.PITCH * 1.05, "uniform pitch above the slot strip")
    stab = sk.stability(SH)
    check(stab["omega_dt"] <= 1.0 + 1e-6, f"hinge stability {stab}")


def test_upright_under_self_buckling():
    """10 cm of this paper stands: the column's self-weight buckling length (Greenhill, 7.837 D / w)^(1/3) is longer,
    and a leaning receipt springs back upright (energy of the hinge chain with the slot's bending spring)."""
    w = M.THERMAL.areal_mass * sk.G
    lc = (7.837 * M.THERMAL.bending_stiffness / w) ** (1 / 3)
    check(lc > M.RECEIPT_L * 1.05, f"buckling length {lc * 1000:.1f} mm vs receipt {M.RECEIPT_L * 1000:.0f} mm")
    from scipy.optimize import minimize
    widths, n = SH.widths, SH.n
    m = [SH.mass(i) for i in range(n)]
    k = [SH.stiffness(j) for j in range(n - 1)]

    def energy(p):
        e = 0.5 * M.BEND_K * p[0] ** 2 + sum(0.5 * k[j] * (p[j + 1] - p[j]) ** 2 for j in range(n - 1))
        z = 0.0
        for i in range(n):
            e += m[i] * sk.G * (z + widths[i] / 2 * math.cos(p[i]))
            z += widths[i] * math.cos(p[i])
        return e

    r = minimize(energy, np.linspace(0.05, 0.4, n), method="L-BFGS-B", options=dict(ftol=1e-15, gtol=1e-12))
    check(np.max(np.abs(r.x)) < math.radians(0.5), f"a leaning receipt springs back upright ({np.degrees(r.x)})")


def test_start_state():
    fr, q = start_frames()
    o = objects_for(fr, q)
    check(np.max(M.tie_loads(o)) < 1e-6, "ties unloaded at the start")
    check(not M.on_teeth(o).any(), "the paper starts off the teeth")
    c, s = SH.box_local(0)
    p0, R0 = fr[0]
    back = anchor_world((M.LIP_FACE_X, 0.0, M.TEETH_Z))
    check(M._box_dist(R0.T @ (back - p0), c, s) > 0.0002, "and off the back lip")
    check(not M.lying_in_tray(o), "not in the tray")
    a = teeth_angle()
    check(math.radians(5) < a < math.radians(20), f"the paper reaches the teeth only bent over the bar ({math.degrees(a):.1f})")
    # the strips stand clear of the terminal's body, the lips and the stub (colliders)
    corners = SH.corners(fr).reshape(-1, 3)
    loc = (corners - np.asarray(M.TERM_POS)) @ M.R_TERM
    check(loc[:, 2].min() >= M.TIE_Z - 1e-9, "the receipt starts at the tie line, above the top face")


def test_doing_nothing():
    fr, q = start_frames()
    scores = run([objects_for(fr, q) for _ in range(60)])
    check(max(scores) == 0.0, f"doing nothing scores {max(scores)}")
    check(M._L.tear.count == 0, "nothing tore")


def test_tear_over_the_teeth_then_tray():
    seq = [objects_for(*start_frames())]
    for s in np.linspace(0.0, 0.0003, 8):
        seq.append(objects_for(*pulled_over(float(s), corner=-math.radians(0.3))))
    scores = run(seq)
    check(M._L.tear.clean, f"torn off along the bar {M._L.tear.broken}")
    order = [k for k in sorted(range(M.N_TIES), key=lambda k: M._L.tear.at[k])]
    check(M._L.tear.at[0] <= M._L.tear.at[-1], f"the near edge goes first {M._L.tear.at}")
    check(scores[-1] == 0.5, f"torn, not in the tray: {scores[-1]}")
    check(0.25 in scores or scores.index(0.5) > 0, "the tear runs across over steps")
    # continue the same trial: flat in the tray, hands away, still
    fr, q = flat_in_tray_frames()
    o = objects_for(fr, q)
    s = []
    for k in range(M.HOLD_STEPS + 3):
        kit._EPISODE["last"] = len(seq) + k
        s.append(M.score(o))
    check(s[0] == 0.75 and s[-1] == 1.0, f"flat in the tray, released and still: {s[0]} -> {s[-1]}")
    del order


def test_even_pull_over_teeth():
    seq = [objects_for(*pulled_over(float(s))) for s in np.linspace(0.0, 0.0002, 6)]
    run(seq)
    check(M._L.tear.off, "an even pull over the teeth tears it off")
    total = M.TIE_K * 0.0001 * M.N_TIES
    check(total < GRIP_CAP, f"needing {total:.2f} N, within the {GRIP_CAP:.1f} N pinch")


def _max_stretch_with_grip():
    return GRIP_CAP / (M.N_TIES * M.TIE_K)  # the most a pinch can draw the ties (all pulling together)


def test_cheat_straight_pull():
    """Pulled straight up out of the slot: the paper never touches the teeth, every tie is intact, and the pinch slips
    long before 8 intact ties give."""
    smax = _max_stretch_with_grip()
    fr0, q = start_frames()
    seq = [objects_for([(p + s * S0_UP, R) for p, R in fr0], q) for s in np.linspace(0, smax, 10)]
    scores = run(seq)
    check(max(scores) == 0.0 and M._L.tear.count == 0, "a straight pull tears nothing")
    check(max(M._L.tear.peak) < M.TIE_TENSILE / 10, "an intact tie is far from breaking")


def test_cheat_sideways_rip():
    """Ripped sideways along the width, or twisted in its plane, without the bar: no tooth, nothing tears."""
    smax = _max_stretch_with_grip()
    fr0, q = start_frames()
    side = M.R_TERM @ np.array([0.0, 1.0, 0.0])
    seq = [objects_for([(p + s * side, R) for p, R in fr0], q) for s in np.linspace(0, smax * 1.5, 10)]
    for ang in np.linspace(0, math.radians(2.0), 8):  # turned in its plane about the far edge (corner pull, no bar)
        fr, _ = chain_frames(np.zeros(N), anchor_world(), R_extra=ry(0) @ rx(0))
        pivot = anchor_world((M.SLOT_X, M.TIE_U[-1], M.TIE_Z))
        Rc = M.R_TERM @ np.array([[1, 0, 0], [0, math.cos(ang), -math.sin(ang)], [0, math.sin(ang), math.cos(ang)]]) \
            @ M.R_TERM.T
        loads_ok = [(pivot + Rc @ (p - pivot), Rc @ R) for p, R in fr]
        o = objects_for(loads_ok, q)
        if M.tie_loads(o).sum() <= GRIP_CAP:
            seq.append(o)
    scores = run(seq)
    check(max(scores) == 0.0 and M._L.tear.count == 0, f"a sideways rip tears nothing {M._L.tear.peak}")


def test_cheat_back_lip():
    """Pulled over the smooth front lip (toward the screen, away from the teeth): intact ties only, no tear."""
    for d in np.arange(0.0, 30.0, 0.05):
        fr, q = chain_frames(np.full(N, -math.radians(d)), anchor_world())
        p0, R0 = fr[0]
        c, s = SH.box_local(0)
        back = anchor_world((M.LIP_FACE_X, 0.0, M.TEETH_Z))
        if M._box_dist(R0.T @ (back - p0), c, s) <= M.TOOTH_CONTACT:
            break
    phis = np.array([-math.radians(d)] + [math.radians(-60.0)] * (N - 1))
    fr, q = chain_frames(phis, anchor_world())
    dvec = fr[0][1] @ np.array([0.0, 1.0, 0.0])
    smax = _max_stretch_with_grip()
    seq = [objects_for([(p + s * dvec, R) for p, R in fr], q) for s in np.linspace(0, smax, 8)]
    scores = run(seq)
    check(not M.on_teeth(seq[-1]).any(), "the back lip has no teeth")
    check(max(scores) == 0.0 and M._L.tear.count == 0, "over the back lip nothing tears")


def test_cheat_pull_roll_out():
    """Pulling the paper out of the printer (feeding more roll) needs every intact tie to give at once: far past the
    pinch. If it were forced out anyway, the break is not along the teeth: a ragged tear, never more than 0.25."""
    need = M.N_TIES * M.TIE_TENSILE
    check(need > 10 * GRIP_CAP, f"pulling the roll out needs {need:.0f} N against a {GRIP_CAP:.1f} N pinch")
    fr0, q = start_frames()
    seq = [objects_for([(p + s * S0_UP, R) for p, R in fr0], q) for s in (0.0, 0.004, 0.02)]
    scores = run(seq)
    check(M._L.tear.off and not M._L.tear.clean, "forced out: torn intact, not along the teeth")
    check(max(scores) <= 0.25, f"a ragged pull-out scores {max(scores)}")


def test_cheat_push_onto_tray_untorn():
    """The receipt bent over onto the counter toward the tray, still attached (no tear): 0. Dragged there in one
    step (the ties torn intact, not on the teeth): ragged, at most 0.25."""
    phis = np.array([math.radians(5.0)] + [math.radians(90.0)] * (N - 1))
    fr, q = chain_frames(phis, anchor_world())
    scores = run([objects_for(fr, q) for _ in range(30)])
    check(max(scores) == 0.0, f"bent over, untorn: {max(scores)}")
    fr2, q2 = flat_in_tray_frames()
    scores = run([objects_for(*start_frames())] + [objects_for(fr2, q2)] * (M.HOLD_STEPS + 3))
    check(max(scores) <= 0.25, f"ripped onto the tray without the bar: {max(scores)}")


def _torn_then(seq_after):
    seq = [objects_for(*start_frames())] + [objects_for(*pulled_over(float(s))) for s in np.linspace(0, 2e-4, 6)]
    return run(seq + seq_after)


def test_cheat_terminal_moved_or_tipped():
    fr, q = flat_in_tray_frames()
    moved = M.TERM_POS[0] + 0.03, M.TERM_POS[1], M.TERM_POS[2]
    s = _torn_then([objects_for(fr, q, term_pos=moved)] * (M.HOLD_STEPS + 3))
    check(s[-1] == 0.75, f"terminal 3 cm off its mark: {s[-1]}")
    turned = M.rz(M.TERM_YAW + math.radians(15))
    s = _torn_then([objects_for(fr, q, term_R=turned)] * (M.HOLD_STEPS + 3))
    check(s[-1] == 0.75, f"terminal turned 15 degrees: {s[-1]}")
    tipped = M.R_TERM @ rx(math.radians(40))
    s = _torn_then([objects_for(fr, q, term_R=tipped)] + [objects_for(fr, q)] * (M.HOLD_STEPS + 3))
    check(max(s[-M.HOLD_STEPS:]) == 0.0, "terminal tipped over once: 0 for the trial")


def test_cheat_dropped_off_counter():
    fr, q = flat_in_tray_frames(z=-0.2)
    s = _torn_then([objects_for(fr, q)] + [objects_for(*flat_in_tray_frames())] * (M.HOLD_STEPS + 3))
    check(s[-1] == 0.0, "receipt off the counter: 0 for the trial")


def test_held_or_touching():
    fr, q = flat_in_tray_frames()
    mid = fr[SH.root][0]
    held = {"left_gripper": AWAY["left_gripper"], "right_gripper": (mid[0], mid[1], mid[2] + 0.004)}
    s = _torn_then([objects_for(fr, q, grippers=held, opening=0.0)] * (M.HOLD_STEPS + 3))
    check(s[-1] == 0.75, f"still held: {s[-1]}")
    rim = (M.TRAY_POS[0] + M.TRAY_OUT[0] / 2, M.TRAY_POS[1], M.TRAY_POS[2] + 0.010)
    on_tray = {"left_gripper": AWAY["left_gripper"], "right_gripper": rim}
    s = _torn_then([objects_for(fr, q, grippers=on_tray, opening=0.3)] * (M.HOLD_STEPS + 3))
    check(s[-1] == 0.75, f"hand on the tray: {s[-1]}")


def test_cheat_receipt_under_tray():
    """The torn receipt laid flat on the counter and the tray set down on top of it (or the receipt slid fully under
    the tray): the receipt is under the tray, not in it, so at most 0.5. Lying on the floor, resting on its face or
    pressed 0.5 mm into it, it is in the tray: 1.0."""
    xy = M.TRAY_POS[:2]
    fr, q = flat_in_tray_frames(tray_pos=(xy[0], xy[1], M.TOP), z=M.PAPER_T / 2)  # flat on the counter, tray's xy
    for lift in (M.PAPER_T, M.PAPER_T + 0.0002):  # the tray's floor resting on the paper, or just over it
        tp = (xy[0], xy[1], M.TOP + lift + M.TRAY_FLOOR_T)
        o = objects_for(fr, q, tray_pos=tp)
        check(not M.lying_in_tray(o), f"receipt under the tray (floor {1000 * lift:.1f} mm up) is not in it")
        s = _torn_then([o] * (M.HOLD_STEPS + 3))
        check(s[-1] == 0.5, f"receipt under the tray: {s[-1]}")
    for z in (M.PAPER_T / 2, M.PAPER_T / 2 - 0.0005):  # resting on the floor's face; pressed 0.5 mm into it
        fr2, q2 = flat_in_tray_frames(z=z)
        s = _torn_then([objects_for(fr2, q2)] * (M.HOLD_STEPS + 3))
        check(s[-1] == 1.0, f"resting on the tray floor ({1000 * z:.2f} mm): {s[-1]}")


def _link_on_rim(link: str, over: float) -> dict:
    """LINK_POSES for one arm link (its own orientation unturned) with its lowest hull point `over` m above the top of
    the tray's far rim wall (negative: pressed into it)."""
    p = yg.points(link)
    low = p[np.argmin(p[:, 2])]
    rim = np.array([M.TRAY_POS[0] + M.TRAY_OUT[0] / 2 - M.TRAY_WALL / 2, M.TRAY_POS[1],
                    M.TRAY_POS[2] + M.TRAY_RIM_H - M.TRAY_FLOOR_T + over])
    return {f"right_robot/{link}": (rim - low, np.eye(3))}


def test_cheat_forearm_on_tray():
    """A wrist or forearm link left resting on the tray's rim (in the sim every arm link is checked by its collision
    hulls, not only the hand): still touching, 0.75. The same link 3 mm clear of everything: 1.0."""
    fr, q = flat_in_tray_frames()
    for link in ("link4", "link5"):
        for over, want in ((-0.0002, 0.75), (0.0, 0.75), (0.003, 1.0)):
            M.LINK_POSES.clear()
            M.LINK_POSES.update(_link_on_rim(link, over))
            try:
                s = _torn_then([objects_for(fr, q)] * (M.HOLD_STEPS + 3))
            finally:
                M.LINK_POSES.clear()
            check(s[-1] == want, f"{link} {1000 * over:+.1f} mm over the rim: {s[-1]} (want {want})")


class _LinkData:
    def __init__(self, n):
        import torch
        self.body_pos_w = torch.zeros((1, n, 3), dtype=torch.float64)
        self.body_quat_w = torch.tensor([[[0.0, 0.0, 0.0, 1.0]] * n], dtype=torch.float64)


class _LinkArt:
    def __init__(self, names):
        self.body_names = list(names)
        self.data = _LinkData(len(names))


class _LinkScene(dict):
    def __init__(self, arts):
        import torch
        super().__init__(arts)
        self.env_origins = torch.zeros((1, 3), dtype=torch.float64)


class _LinkBase:
    def __init__(self, arts):
        self.scene = _LinkScene(arts)


def test_sim_link_read_is_strict():
    """In the simulator the hands-clear check reads all 16 arm-link poses (8 links x 2 arms) or stops with an error: a
    missing robot or link, or no poses at all, never falls back to the offline hand boxes."""
    full = ["base"] + list(M.ARM_LINKS)
    ok = {r: _LinkArt(full) for r in M.ROBOTS}
    M.LINK_POSES.clear()
    M._read_links(_LinkBase(ok))
    check(len(M.LINK_POSES) == 16, f"{len(M.LINK_POSES)} link poses read")
    for arts, what in (({M.ROBOTS[0]: _LinkArt(full)}, "a robot missing"),
                       ({r: _LinkArt([n for n in full if n != "link4"]) for r in M.ROBOTS}, "link4 missing")):
        M.LINK_POSES.clear()
        try:
            M._read_links(_LinkBase(arts))
            raised = False
        except RuntimeError:
            raised = True
        check(raised, f"{what}: must raise, not fall back")
    M.LINK_POSES.clear()
    fr, q = flat_in_tray_frames()
    M._IN_SIM["on"] = True
    try:
        M.hands_on(objects_for(fr, q))
        raised = False
    except RuntimeError:
        raised = True
    finally:
        M._IN_SIM["on"] = False
    check(raised, "in the sim, no link poses must raise, not use the hand boxes")
    check(not M.hands_on(objects_for(fr, q)), "offline (tests only) the hand boxes are used")


def test_not_flat():
    """Torn and dropped standing or crumpled high, or hanging over the rim: 0.5."""
    fr, q = flat_in_tray_frames(z=0.020)
    s = _torn_then([objects_for(fr, q)] * 5)
    check(s[-1] == 0.5, "lying 2 cm over the floor is not lying in the tray")
    fr, q = flat_in_tray_frames(shift=0.035)
    s = _torn_then([objects_for(fr, q)] * 5)
    check(s[-1] == 0.5, "hanging over the tray's rim")


def test_stalled_half_tear():
    seq = [objects_for(*start_frames())]
    a = teeth_angle()
    phis = np.array([a] + [math.radians(60.0)] * (N - 1))
    fr, q = chain_frames(phis, anchor_world())
    # only the near edge's tie is drawn past the tear resistance, then the hand lets the paper go back
    o = objects_for(fr, q)
    pivot = fr[0][0] + fr[0][1] @ np.array(M.TIE_STRIP_PTS[-1])
    axis = fr[0][1] @ np.array([0.0, 0.0, 1.0])
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    ang = -0.00015 / (M.TIE_U[-1] - M.TIE_U[0])
    Rc = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
    seq.append(objects_for([(pivot + Rc @ (p - pivot), Rc @ R) for p, R in fr], q))
    seq += [objects_for(*start_frames())] * 10
    scores = run(seq)
    check(0 < M._L.tear.count < M.N_TIES, f"a part tear {M._L.tear.broken}")
    check(scores[-1] == 0.25, f"a ragged half-tear left in the printer: {scores[-1]}")
    del o


def test_new_trial():
    test_tear_over_the_teeth_then_tray()
    fr, q = start_frames()
    s = run([objects_for(fr, q)] * 3)
    check(s[-1] == 0.0 and M._L.tear.count == 0, "a new trial starts untorn")


def test_tray_placement_robust():
    rng = np.random.default_rng(3)
    for _ in range(300):
        yaw = rng.uniform(-math.pi, math.pi)
        tpos = (M.TRAY_POS[0] + rng.uniform(-0.05, 0.05), M.TRAY_POS[1] + rng.uniform(-0.05, 0.05), M.TRAY_POS[2])
        tR = M.rz(yaw)
        fr, q = flat_in_tray_frames(tpos, tR, yaw_in_tray=rng.uniform(-0.15, 0.15))
        check(M.lying_in_tray(objects_for(fr, q, tray_pos=tpos, tray_R=tR)), "flat in a moved tray")
        fr, q = flat_in_tray_frames(tpos, tR, shift=0.035)
        check(not M.lying_in_tray(objects_for(fr, q, tray_pos=tpos, tray_R=tR)), "over the rim of a moved tray")


# -- does a pinch load the ties? (sheetkit probe p3: a clamp that reached its target with no load tore nothing) ----
def _right_targets():
    import yam_arena.demo as demo
    plan = demo.ArmPlan(M.right_plan(), demo.RIGHT_BASE_POS)
    return plan, demo


def _arm_compliance(q, base):
    """The tool point's translational compliance (m/N) from the arm's joint stiffnesses (yam_arena constants)."""
    from yam_arena.constants import SHOULDER, WRIST
    import yam_arena.ik as ik
    J = np.zeros((3, 6))
    for j in range(6):
        dq = np.zeros(6)
        dq[j] = 1e-6
        J[:, j] = (ik.tcp_pose(q + dq, base)[0] - ik.tcp_pose(q - dq, base)[0]) / 2e-6
    Kq = np.diag([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)
    return J @ np.linalg.inv(Kq) @ J.T


def test_pinch_loads_ties():
    """Along the demo's swing: (1) the pads close on the paper (their boxes overlap the strips the pinch is on, and
    the paper's collider stops them short of closed, so they squeeze it); (2) the swing draws the pinch past the
    paper's length over the teeth, so the paper cannot follow without the ties stretching; (3) the arm, held that far
    past, pulls harder than the tear needs; (4) the pinch holds more than that pull (no slip first); the edge-first
    tear needs less than an even pull."""
    plan, demo = _right_targets()
    t_pinch = 3.5 + 2.0 + 1.5
    pos, rot, finger, label = plan.target(t_pinch - 0.01)
    check(label == "pinch it near its edge", label)
    fr, q = start_frames()
    o = objects_for(fr, q, grippers={"right_gripper": pos, "left_gripper": AWAY["left_gripper"]}, opening=0.0)
    o["right_gripper"] = st(pos, R=rot)
    on = [i for i in range(N) if gk.touching(o, SH.key(i), M.STRIP_PARTS[i], grippers=("right_gripper",))]
    check(len(on) >= 3, f"the closed pads lie on strips {on}")
    check(0 not in on, "the pinch is above the strip at the slot")
    # the palm clears the receipt's top edge, the fingertips the bar and the terminal
    top = fr[-1][0] + fr[-1][1] @ np.array([0.0, SH.widths[-1], 0.0])
    palm_low = pos[2] + (-gk.PALM.center[2] - gk.PALM.size[2] / 2)
    check(palm_low - top[2] > 0.004, f"palm {1000 * (palm_low - top[2]):.1f} mm over the receipt's top")
    # (2) how far past the paper's length over the teeth the swing commands the pinch
    edge = M.TEETH_W + M.PINCH_OFF * M.ACROSS
    paper_len = M.PINCH_UP  # paper from the teeth to the pinch (the tie line is 1.5 mm below, inside the slot)
    over = []
    for b, extra, across in M.SWING:
        p = M.pinch_point(b, extra, across)
        over.append(np.linalg.norm(p - edge) - paper_len)
    check(over[-1] > 50 * M.TIE_STRETCH, f"the swing ends {1000 * over[-1]:.1f} mm past the paper's length")
    # (3) the force the arm makes held that far past (its joint springs), along the pull
    t_end = t_pinch + len(M.SWING) * 1.0
    a, _ = plan.action7(t_end - 0.01)
    C = _arm_compliance(np.asarray(a[:6], float), demo.RIGHT_BASE_POS)
    p_end = M.pinch_point(*M.SWING[-1])
    d = (p_end - edge) / np.linalg.norm(p_end - edge)
    arm_force = over[-1] / float(d @ C @ d)
    even = M.N_TIES * M.TEAR_CD
    # (5) edge first: the pads' middle off the paper's middle by PINCH_OFF; a rigid strip shares a pull at u between
    # the remaining ties as force + moment; the tear runs tie by tie from the near edge
    left = list(range(M.N_TIES))
    worst = 0.0
    u_p = M.PINCH_OFF
    while left:
        us = np.array([M.TIE_U[k] for k in left])
        um = us.mean()
        frac = 1 / len(us) + (u_p - um) * (us - um) / max(np.sum((us - um) ** 2), 1e-12)
        F = M.TEAR_CD / frac.max()
        worst = max(worst, F)
        left.pop(int(np.argmax(frac)))
    check(worst < even, f"edge first ({worst:.2f} N) takes less than an even pull ({even:.2f} N)")
    check(arm_force > 2 * even, f"the arm pulls {arm_force:.1f} N, the tear needs at most {even:.2f} N")
    check(GRIP_CAP > 2 * even, f"the pinch holds {GRIP_CAP:.1f} N, twice the even pull {even:.2f} N")
    print(f"    pinch {GRIP_CAP:.1f} N (pads {GRIP_N:.1f} N each), arm {arm_force:.1f} N at {1000 * over[-1]:.0f} mm "
          f"past, tear edge-first {worst:.2f} N / even {even:.2f} N, pads on strips {on}")


def test_left_hand_steadies_terminal():
    """The left hand's closed fingertips press lightly on the keypad end's top face, nowhere near the receipt."""
    import yam_arena.demo as demo
    plan = demo.ArmPlan(M.left_plan(), demo.LEFT_BASE_POS)
    pos, rot, finger, label = plan.target(3.0 + 2.0 + 1.2 - 0.01)
    check(label == "steady the terminal", label)
    tip_z = pos[2] - M.TIP_PAST_TCP
    into = M.TOP + M.TERM_H - tip_z
    check(0.0005 < into <= 0.002, f"fingertips {1000 * into:.1f} mm into the top face")
    local = M.R_TERM.T @ (np.asarray(pos) - np.asarray(M.TERM_POS))
    check(abs(local[1]) < M.TERM_W / 2 - 0.015 and local[0] < M.SLOT_X - 0.06, "on the keypad end, away from the slot")

def test_text_matches():
    j = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    for s in ("220 mN", "2.1 kN/m", "48 g/m^2", "Jujo", "KSND-48", "15 mm", "1 mm", "2 cm", "10 degrees", "5 degrees",
              "30 degrees", "1 s"):
        check(s in readme, f"README: {s}")
    for s in ("220 mN", "2.1 kN/m", "1 s", "2 cm", "15 mm", "1 mm", "10 degrees", "5 degrees", "30 degrees"):
        check(s in j["scoring"] or s in j["fidelity"] or s in j["context"], f"task.json: {s}")
    for txt in (readme, json.dumps(j)):
        check("$" not in txt, "no prices")


def _normal_world(fr):
    return fr[0][1] @ np.array([0.0, 0.0, 1.0])


def test_cheat_press_onto_bar():
    """The standing receipt pressed sideways onto the bar (toward the screen), no pull: the slot takes the press,
    no tie is in tension, nothing tears, however hard."""
    fr0, q = start_frames()
    screen = M.R_TERM @ np.array([1.0, 0.0, 0.0])  # toward the bar
    seq = [objects_for([(p + d * screen, R) for p, R in fr0], q) for d in np.linspace(0, 0.003, 12)]
    scores = run(seq)
    check(max(scores) == 0.0 and M._L.tear.count == 0, f"pressed onto the bar: {max(scores)} {M._L.tear.peak}")


def test_cheat_push_down_into_slot():
    """Bent over onto the teeth, then pushed down into the slot (compression) or pressed flat over the bar with a
    palm: no tension, nothing tears."""
    a = teeth_angle()
    for lean in (a, 60.0 * math.pi / 180):
        phis = np.array([a] + [lean] * (N - 1))
        fr, q = chain_frames(phis, anchor_world())
        down = -(fr[0][1] @ np.array([0.0, 1.0, 0.0]))
        n = _normal_world(fr)
        seq = [objects_for([(p + d * down, R) for p, R in fr], q) for d in np.linspace(0, 0.003, 10)]
        seq += [objects_for([(p + d * n, R) for p, R in fr], q) for d in np.linspace(-0.003, 0.003, 10)]
        scores = run(seq)
        check(max(scores) == 0.0 and M._L.tear.count == 0, f"pushed into the slot / pressed: {M._L.tear.peak}")


def test_cheat_nick_then_pull_straight():
    """One tie started on the teeth, then the paper put back upright and pulled straight up or ripped sideways: off
    the teeth the running tear is not along the bar; nothing more breaks below the paper's full strength."""
    seq = [objects_for(*start_frames())]
    a = teeth_angle()
    fr, q = chain_frames(np.array([a] + [math.radians(60.0)] * (N - 1)), anchor_world())
    pivot = fr[0][0] + fr[0][1] @ np.array(M.TIE_STRIP_PTS[-1])
    axis = _normal_world(fr)
    K = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    ang = -0.00013 / (M.TIE_U[-1] - M.TIE_U[0])
    Rc = np.eye(3) + math.sin(ang) * K + (1 - math.cos(ang)) * K @ K
    seq.append(objects_for([(pivot + Rc @ (p - pivot), Rc @ R) for p, R in fr], q))
    fr0, q0 = start_frames()
    smax = GRIP_CAP / (M.N_TIES * M.TIE_K)
    seq += [objects_for([(p + s * S0_UP, R) for p, R in fr0], q0) for s in np.linspace(0, smax, 8)]
    side = M.R_TERM @ np.array([0.0, 1.0, 0.0])
    seq += [objects_for([(p + s * side, R) for p, R in fr0], q0) for s in np.linspace(0, smax, 8)]
    scores = run(seq)
    check(0 < M._L.tear.count < M.N_TIES, f"one nicked, the rest hold {M._L.tear.broken}")
    check(scores[-1] == 0.25, f"a nicked receipt pulled off the bar: {scores[-1]}")


def test_reset_placement_after_jitter():
    """The reset puts the receipt where the jittered terminal holds it: zero tie load, as at the layout start."""
    rng = np.random.default_rng(5)
    for _ in range(50):
        tp = np.asarray(M.TERM_POS) + np.array([*rng.uniform(-0.03, 0.03, 2), 0.0])
        tR = M.rz(M.TERM_YAW + rng.uniform(-0.2, 0.2))
        pos, R = M.receipt_root_pose(tp, tR)
        fr = SH.forward(np.zeros(N - 1), base_pos=pos, base_R=sk.quat_R(M.quat_from_R(R)))
        o = objects_for(fr, np.zeros(N - 1), term_pos=tp, term_R=tR)
        check(np.max(M.tie_loads(o)) < 1e-6 and not M.on_teeth(o).any(), "placed at the jittered slot, unloaded")
    for R in (M.rz(3.1), M.R_TERM @ M.R_SHEET, rx(3.0) @ ry(1.0)):
        check(np.allclose(sk.quat_R(M.quat_from_R(R)), R, atol=1e-9), "quaternion round trip")


def test_cheat_terminal_lifted_or_tilted():
    fr, q = flat_in_tray_frames()
    up = M.TERM_POS[0], M.TERM_POS[1], M.TERM_POS[2] + 0.10
    s = _torn_then([objects_for(fr, q, term_pos=up)] * (M.HOLD_STEPS + 3))
    check(s[-1] == 0.75, f"terminal held in the air: {s[-1]}")
    s = _torn_then([objects_for(fr, q, term_R=M.R_TERM @ rx(math.radians(20)))] * (M.HOLD_STEPS + 3))
    check(s[-1] == 0.75, f"terminal propped at 20 degrees: {s[-1]}")


class _FakeArt:
    """Just enough of an Isaac Lab articulation / rigid object to record what the reset hook writes."""

    def __init__(self, joint_names=(), pose=None):
        import torch
        self.joint_names = list(joint_names)
        self.calls = {}
        self.data = type("D", (), {})()
        self.data.joint_pos = torch.zeros((1, max(1, len(self.joint_names))))
        if pose is not None:
            self.data.root_pose_w = torch.tensor([pose], dtype=torch.float32)

    def __getattr__(self, name):
        if name.endswith("_index"):
            def rec(**kw):
                self.calls.setdefault(name, []).append(kw)
            return rec
        raise AttributeError(name)


def test_reset_springs_the_paper():
    """Probe p1 (48345ef): the reset's settle ran with every receipt hinge limp (yamkit applies the hinge laws only
    at control steps), the receipt flopped onto the bar before the trial and the pinch closed on air. The reset hook
    must put every hinge straight and drive it with its paper stiffness from the first physics step, and place the
    receipt at the terminal's slot."""
    q = sk.R_quat(M.R_TERM)
    term = _FakeArt(pose=[*M.TERM_POS, *q])
    rec = _FakeArt(joint_names=[f"{lk.name}_joint" for lk in M.RECEIPT.links])
    M.PIN.place({"terminal": term, "receipt": rec})
    stiff = rec.calls.get("write_joint_stiffness_to_sim_index", [])
    check(len(stiff) == len(M.RECEIPT.links), f"every hinge sprung at reset ({len(stiff)} of {len(M.RECEIPT.links)})")
    ks = sorted(float(c["stiffness"].item()) for c in stiff)
    want = sorted(lk.law.k for lk in M.RECEIPT.links)
    check(np.allclose(ks, want, rtol=1e-5) and min(ks) > 0, "with the paper's hinge stiffness")
    pos = rec.calls.get("write_joint_position_to_sim_index", [])
    check(len(pos) == len(M.RECEIPT.links) and all(abs(float(c["position"].item())) < 1e-9 for c in pos), "straight")
    pose = rec.calls["write_root_pose_to_sim_index"][0]["root_pose"][0].numpy()
    check(np.allclose(pose[:3], M.RECEIPT.pos, atol=1e-5), f"placed at the slot {pose[:3]} vs {M.RECEIPT.pos}")
    # and that stiffness holds the 10 cm receipt upright (each hinge's spring against the paper above it, leaning 1 deg)
    w, n = SH.widths, SH.n
    for j in range(n - 1):
        above = range(j + 1, n)
        m_above = sum(SH.mass(i) for i in above)
        h = sum(w[i] for i in above) / 2
        check(SH.stiffness(j) > m_above * sk.G * h, f"hinge {j} holds the paper above it upright")


class _FakeScene(dict):
    pass


def test_reset_order_survives_yamkit_and_jitter():
    """The env's reset writes the scene once inside itself; yamkit's reset wrapper (outside it, from the second trial)
    then sets every law-held joint limp, and yam_arena jitters the free objects; only then does the settle step. The
    receipt must end each reset placed at the JITTERED terminal and sprung, over several trials."""
    import torch
    q = sk.R_quat(M.R_TERM)
    for trial in range(3):
        term = _FakeArt(pose=[*M.TERM_POS, *q])
        rec = _FakeArt(joint_names=[f"{lk.name}_joint" for lk in M.RECEIPT.links])
        scene = _FakeScene(terminal=term, receipt=rec)

        def env_reset_body(env):
            M.scene_was_reset(scene)  # scene.reset
            M.before_physics(scene)   # ManagerBasedEnv.reset: scene.write_data_to_sim()

        env_reset = M.inside_env_reset(env_reset_body)
        env_reset(None)
        if trial > 0:  # yamkit's _watch_resets, outside the env reset: every law joint limp at its start
            for j in range(len(rec.joint_names)):
                kit._set_drive(rec, j, target=0.0, stiffness=0.0, effort=0.0)
        shift = np.array([0.02, -0.015, 0.0]) * (trial + 1)
        term.data.root_pose_w = torch.tensor([[*(np.asarray(M.TERM_POS) + shift), *q]], dtype=torch.float32)  # jitter
        M.before_physics(scene)  # the settle's first physics step
        M.before_physics(scene)  # later steps do nothing more
        stiff = rec.calls["write_joint_stiffness_to_sim_index"]
        last = {}
        for c in stiff:
            last[int(c["joint_ids"][0])] = float(c["stiffness"].item())
        check(all(last[j] > 0 for j in range(len(rec.joint_names))), f"trial {trial}: hinges limp after reset {last}")
        poses = rec.calls["write_root_pose_to_sim_index"]
        check(len(poses) == 1, f"placed once per reset ({len(poses)})")
        want, _ = M.receipt_root_pose(np.asarray(M.TERM_POS) + shift, M.R_TERM)
        check(np.allclose(poses[-1]["root_pose"][0, :3].numpy(), want, atol=1e-5), "placed at the jittered terminal")


def test_right_approach_clears_receipt():
    """Probe p2: at CARRY_Z the open fingers swept into the standing receipt's top on the way over and pushed it onto
    the bar, so the pinch closed on air. Along the right plan until the pinch closes, no part of the right hand
    (fingers at their commanded opening) touches the standing receipt, and at the pinch the paper lies between the
    open pads."""
    import yam_arena.demo as demo
    plan = demo.ArmPlan(M.right_plan(), demo.RIGHT_BASE_POS)
    fr, q = start_frames()
    t_close = 3.5 + 2.0
    for t in np.arange(0.0, t_close, 0.05):
        pos, rot, finger, label = plan.target(float(t))
        o = objects_for(fr, q, grippers={"left_gripper": AWAY["left_gripper"], "right_gripper": pos},
                        opening=float(finger / M.FINGER_OPEN))
        o["right_gripper"] = st(pos, R=rot)
        hit = [i for i in range(N) if gk.touching(o, SH.key(i), M.STRIP_PARTS[i], grippers=("right_gripper",))]
        check(not hit, f"t {t:.2f} ({label}): the open hand touches strips {hit}")
    pos, rot, finger, label = plan.target(t_close - 0.01)
    gap = abs(float((np.asarray(pos) - fr[4][0]) @ (M.R_TERM @ np.array([1.0, 0.0, 0.0]))))
    half = abs(finger) * 1.0
    check(gap + M.PAPER_T / 2 < half, f"paper between the open pads ({1000 * gap:.1f} mm off center, pads +-{1000 * half:.1f})")


def test_right_descent_tolerates_tracking_error():
    """Probe p3: with the fingers 12 mm a side the hand still met the receipt's top on the way down (the real hand
    tracks its plan only to some mm and a few degrees). Coming down fully open, it stays clear of the standing receipt
    with the tool point off the plan by up to 6 mm across the paper and along its width and the grip turned up to 10
    degrees, all the way to the pinch height."""
    import yam_arena.demo as demo
    plan = demo.ArmPlan(M.right_plan(), demo.RIGHT_BASE_POS)
    fr, q = start_frames()
    across_thick = M.R_TERM @ np.array([1.0, 0.0, 0.0])
    t_close = 3.5 + 2.0
    for t in np.arange(0.0, t_close, 0.1):
        pos, rot, finger, label = plan.target(float(t))
        for dx in (-0.006, 0.0, 0.006):
            for dw in (-0.006, 0.006):
                for yaw in (-10.0, 10.0):
                    p = np.asarray(pos) + dx * across_thick + dw * M.ACROSS
                    Rg = M.rz(math.radians(yaw)) @ rot
                    o = objects_for(fr, q, opening=float(finger / M.FINGER_OPEN))
                    o["right_gripper"] = st(p, R=Rg)
                    hit = [i for i in range(N) if gk.touching(o, SH.key(i), M.STRIP_PARTS[i],
                                                              grippers=("right_gripper",))]
                    check(not hit, f"t {t:.1f} {label} off {dx}/{dw} yaw {yaw}: touches strips {hit}")


def test_cheat_tray_flipped_or_moved():
    """Torn, then the tray turned upside down over the receipt (or held tilted): not lying in the tray. Torn and in
    the tray, but the tray carried off its outline: not 1.0."""
    fr, q = flat_in_tray_frames()
    flipped = rx(math.pi)
    tpos = (M.TRAY_POS[0], M.TRAY_POS[1], M.TRAY_POS[2] + 0.004)
    s = _torn_then([objects_for(fr, q, tray_pos=tpos, tray_R=flipped)] * (M.HOLD_STEPS + 3))
    check(s[-1] == 0.5, f"tray flipped over the receipt: {s[-1]}")
    tilted = rx(math.radians(40))
    fr2, q2 = flat_in_tray_frames(tray_R=tilted)
    s = _torn_then([objects_for(fr2, q2, tray_R=tilted)] * (M.HOLD_STEPS + 3))
    check(s[-1] == 0.5, f"tray held tilted: {s[-1]}")
    moved = (M.TRAY_POS[0] + 0.18, M.TRAY_POS[1], M.TRAY_POS[2])
    fr3, q3 = flat_in_tray_frames(tray_pos=moved)
    s = _torn_then([objects_for(fr3, q3, tray_pos=moved)] * (M.HOLD_STEPS + 3))
    check(s[-1] == 0.75, f"tray carried 18 cm off its outline: {s[-1]}")


def test_little_paper_above_the_pinch():
    """Probe p6: pinched 3.6 cm above the teeth, the 6 cm of paper above the pads folded over a finger, and the open
    hand carried the torn receipt away. The pinch is near the top edge: at most 15 mm of paper above the tool point
    (the pads' height), the palm well clear, and the torn receipt hangs below the pinch with its end just reaching the
    tray floor where the lay starts."""
    above = (M.RECEIPT_L - M.TIE_DEPTH) - M.PINCH_UP
    check(0.003 < above <= 0.015, f"{1000 * above:.1f} mm of paper above the tool point")
    palm_gap = -gk.PALM.center[2] - gk.PALM.size[2] / 2 - above
    check(palm_gap > 0.04, f"palm {1000 * palm_gap:.0f} mm over the top edge")
    hang = M.PINCH_UP - M.TIE_DEPTH  # the torn receipt below the pinch
    check(abs(hang - M.TRAY_TOUCH_Z) < 0.015, f"hanging {1000 * hang:.0f} mm vs touch-down height {1000 * M.TRAY_TOUCH_Z:.0f}")
    check(hang <= 2 * M.TRAY_LAY + 0.03 and hang < M.TRAY_IN[1], "it lays inside the tray's length")


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_") and callable(v)]
    for t in tests:
        t()
        print(f"ok {t.__name__}")
    print(f"{len(tests)} tests ok")
