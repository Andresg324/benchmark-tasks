"""A bench model of a small four-stroke engine (a mower or generator engine) with its dipstick seated in the oil fill
tube and a folded shop rag on the table. Checking the oil means pulling the dipstick, wiping the blade clean on the rag
(the first pull reads a false, smeared level because oil splashes up the tube while the engine runs), pushing it back
in until the cap seats, pulling it again and holding it still to read the level, then seating it again and putting the
rag down.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled engine and its sliding dipstick; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Check the oil level on the engine with the dipstick, then put everything back."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# The engine's frame is fixed to the table, with its origin on the table top under the fill tube's axis.
SX, SY = 0.25, -0.12  # the fill tube's axis in world x, y
BASE = (SX, SY, TOP)
MOUTH = 0.085  # the fill tube's mouth (top rim) above the table
BLADE = 0.08  # the dipstick blade, from the cap's underside to the tip
BLADE_W, BLADE_T = 0.010, 0.003  # blade width (along x) and thickness (along y)
CAP_R, CAP_H = 0.016, 0.010  # the cap's disc, which seats on the tube's rim
HANDLE = (0.018, 0.035, 0.035)  # the T-handle above the cap: thickness (x), width (y), height (z)
HANDLE_C = CAP_H + HANDLE[2] / 2  # the handle's middle above the link origin (the cap's underside)
TUBE_R = 0.012  # the fill tube's outside radius
CASE_TOP = 0.05  # the crankcase block's top
RAG = (0.04, 0.04, 0.025)  # the folded shop rag (pad): x, y, z
RAG_START = (0.26, 0.10, TOP + RAG[2] / 2)

# Scoring (the dipstick's joint position q is how far the cap is above its seat, m)
SEAT = 0.003  # m: q at or under this is seated (cap down on the rim; a 3 mm gap is a visible lift of the cap)
OUT = BLADE + 0.005  # m: the blade's tip at least 5 mm above the mouth: the dipstick is out of the tube
TIP_IN = BLADE - 0.005  # m: under this the tip is back inside the tube (it picks up the splash on the tube wall)
WIPE_ZONE = 0.06  # m: the lower 6 cm of the blade, the part the splash and the FULL/ADD crosshatch are on (drawn smear)
WIPE_SPAN = 0.04  # m: the stretch from the tip that the rag's middle must slide over: the pad, about 4 cm tall when held
# on edge, reaches 2 cm either side of its middle, so a middle drawn from 4 cm down to the tip rubs the whole 6 cm smear
WIPE_FRAC = 0.8  # the middle must slide over 80% of that span (3.2 cm) in contact: a stroke may stop up to 8 mm short
# (the arm lags its target about 1.6 cm), but a dab (no slide) or half a stroke (2 cm) leaves the marks smeared
GAP_STEPS = 3  # control steps (0.15 s): kept for the drawn oil only; the wipe credits only slides between two steps
# that are both in contact, so nothing is credited across a gap (a dab, a lift and a dab elsewhere add up to nothing)
SLIDE_MAX = 0.005  # m per control step (10 cm/s): a faster jump between two touching steps is not credited (the demo's
# stroke slides about 0.75 mm per step; a pad yanked along faster skips over the oil rather than wiping it)
BIN = 0.001  # m: the blade is tracked in 1 mm bins
RAG_TOUCH = 0.027  # m: rag middle within this of the blade's axis: half the pad (2 cm) plus the blade's half-thickness and
# the cloth pressing round it (about 0.5 cm); a rag hovering clear of the blade sits farther out than this
RAG_GAP = 0.002  # m: and the rag's outline within 2 mm of the blade's faces (the cloth's nap and the pad's give): a rigid
# pad held 5 mm clear of the blade can still have its middle within RAG_TOUCH, so the outline is checked as well
READ_STEPS = 20  # control steps (1 s at 20 Hz) the dipstick must be held out and still to read it
STILL = 0.01  # m/s: the dipstick moving slower than this (its travel per control step) counts as still
CONTROL_HZ = 20
HOLD_R = 0.03  # m: a tool point this close to the handle's middle, with the fingers closed on it, holds the dipstick
CLOSED = 0.75  # kit.opening below this: fingers closed on something
LET_GO = 0.06  # m: at the end each tool point is farther than this from the handle and from the rag
RAG_DOWN = 0.01  # m: the rag's bottom within 1 cm of the table, at rest
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving
GRIPPERS = ("left_gripper", "right_gripper")

ALU = kit.Look((0.62, 0.64, 0.66), 0.4, metallic=0.6)  # cast crankcase
DARK = kit.Look((0.14, 0.14, 0.15), 0.6)
RED = kit.Look((0.72, 0.12, 0.10), 0.5)  # engine shroud
YELLOW = kit.Look((0.95, 0.75, 0.10), 0.45)  # dipstick cap and handle (yellow, as on most small engines)
STEEL = kit.Look((0.82, 0.83, 0.85), 0.25, metallic=0.9)
BLACK = kit.Look((0.06, 0.06, 0.06), 0.7)
RAGLOOK = kit.Look((0.20, 0.35, 0.70), 0.9)  # blue shop towel
WOOD = kit.Look((0.55, 0.40, 0.25), 0.7)


def _box(lo, hi, look, solid=True):
    c = tuple((a + b) / 2 for a, b in zip(lo, hi))
    s = tuple(b - a for a, b in zip(lo, hi))
    return kit.box(c, s, look), (kit.Collider(c, size=s) if solid else None)


def _parts(boxes):
    meshes, colliders = [], []
    for lo, hi, look, solid in boxes:
        m, c = _box(lo, hi, look, solid)
        meshes.append(m)
        if c is not None:
            colliders.append(c)
    return tuple(meshes), tuple(colliders)


def _engine():
    body = [  # (low corner, high corner, look, collides), in the engine's frame
        ((-0.03, -0.10, 0.0), (0.13, 0.05, 0.008), WOOD, True),  # bench mounting board
        ((0.02, -0.09, 0.008), (0.12, 0.03, CASE_TOP + 0.02), ALU, True),  # crankcase and cylinder block
        ((0.03, -0.08, CASE_TOP + 0.02), (0.11, 0.02, CASE_TOP + 0.05), RED, True),  # shroud over the flywheel
        ((-0.02, -0.03, 0.008), (0.02, 0.03, CASE_TOP - 0.02), ALU, True),  # fill boss on the crankcase's front
        ((0.12, -0.06, 0.03), (0.125, -0.02, 0.06), BLACK, False),  # oil drain plug on the side
    ]
    meshes, colliders = _parts(body)
    meshes += (kit.cylinder((0.0, 0.0, (CASE_TOP - 0.02 + MOUTH) / 2), TUBE_R, MOUTH - CASE_TOP + 0.02, YELLOW),
               kit.cylinder((0.0, 0.0, MOUTH - 0.002), TUBE_R + 0.002, 0.004, DARK))  # fill tube and its rim
    # Dipstick (link frame at the cap's underside, on the tube's axis): blade below, cap disc and T-handle above.
    marks = 0.012  # the crosshatch between ADD and FULL near the tip
    s_meshes = (
        kit.box((0.0, 0.0, -BLADE / 2), (BLADE_W, BLADE_T, BLADE), STEEL),
        kit.box((0.0, 0.0, -BLADE + 0.008 + marks / 2), (BLADE_W * 1.02, BLADE_T * 1.2, marks),
                kit.Look((0.45, 0.46, 0.48), 0.5, metallic=0.6)),
        kit.cylinder((0.0, 0.0, CAP_H / 2), CAP_R, CAP_H, YELLOW),
        kit.box((0.0, 0.0, HANDLE_C), HANDLE, YELLOW),
    )
    s_colliders = (
        kit.Collider((0.0, 0.0, -BLADE / 2), size=(BLADE_W, BLADE_T, BLADE)),
        kit.Collider((0.0, 0.0, CAP_H / 2), size=(2 * CAP_R, 2 * CAP_R, CAP_H)),
        kit.Collider((0.0, 0.0, HANDLE_C), size=HANDLE),
    )
    stick = kit.Link("stick", "prismatic", "Z", (0.0, 0.0, MOUTH), lower=0.0, upper=0.13, start=0.0, mass=0.04,
                     meshes=s_meshes, colliders=s_colliders, damping=0.5)
    return kit.Mechanism("engine", pos=BASE, size=(0.16, 0.15, 0.11), meshes=meshes, colliders=colliders,
                         links=(stick,))


def _rag():
    folds = [kit.box((0.0, 0.0, 0.0), RAG, RAGLOOK),
             kit.box((0.0, 0.0, RAG[2] / 2 - 0.004), (RAG[0] * 1.01, RAG[1] * 1.01, 0.001),
                     kit.Look((0.16, 0.29, 0.60), 0.9))]
    return kit.Prop("rag", pos=RAG_START, size=RAG, mass=0.04, meshes=tuple(folds),
                    colliders=(kit.Collider((0.0, 0.0, 0.0), size=RAG),))


OBJECTS = [_engine(), _rag()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here; a scene reset (kit.episode()) starts a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.phase = "seated"  # seated (dirty) -> out (dirty, can be wiped) -> clean_out -> reseated -> second -> read
        self.swept = np.zeros(int(round(WIPE_SPAN / BIN)), bool)  # bins (from the tip) the rag's middle has slid over
        self.rubbed = np.zeros(int(round(WIPE_ZONE / BIN)), bool)  # bins the sliding pad has covered (oil drawing only)
        self.last_s, self.gap = None, 0  # the rag's middle along the blade at its last contact, steps since then
        self.read_steps = 0
        self.reseated = False  # wiped, then pushed back in by hand until the cap seated
        self.read = False  # pulled again after that seat and held out still for 1 s
        self.half_seat = False  # a wiped blade went back into the tube without seating and came out again
        self.final_seat = False  # after the reading, pushed back in by hand until the cap seated
        self.last_q, self.speed = None, 0.0
        self.home = None
        self.last = None
        self.still = False


_H = _History()


def _q(objects):
    return kit.joint(objects, "engine/stick").pos


def _handle(objects):
    return np.asarray(objects["engine/stick"].pos, float) + np.array([0.0, 0.0, HANDLE_C])


def _holding(objects):
    """A gripper's tool point within 3 cm of the handle's middle with its fingers closed (on the handle)."""
    h = _handle(objects)
    for g in GRIPPERS:
        if g in objects and np.linalg.norm(np.asarray(objects[g].pos) - h) < HOLD_R and kit.opening(objects, g) < CLOSED:
            return True
    return False


def _wiped():
    """The rag's middle has slid over at least 80% of the blade's lower 4 cm, in contact, during this pull."""
    return _H.swept.sum() >= WIPE_FRAC * len(_H.swept) - 1e-9


def _mark(arr, a, b):
    a, b = max(int(round(a / BIN)), 0), min(int(round(b / BIN)), len(arr))
    if b > a:
        arr[a:b] = True


def _rag_pass(objects, q):
    """Track one pull's wipe. The rag touches the blade when its middle is within 2.7 cm (RAG_TOUCH) of the blade's axis
    (horizontally) and its height overlaps the blade above the mouth. Between two touching steps (with no more than
    GAP_STEPS without contact between them) the stretch of blade its middle slid over is marked; a rag held still on
    the blade (a dab) marks nothing."""
    r = objects.get("rag")
    tip = TOP + MOUTH - BLADE + q  # the blade's tip in world z
    if r is not None:
        p, lo, hi = np.asarray(r.pos, float), float(r.low[2]), float(r.high[2])
        lx, ly, hx, hy = float(r.low[0]), float(r.low[1]), float(r.high[0]), float(r.high[1])
        gx = max(0.0, lx - (SX + BLADE_W / 2), (SX - BLADE_W / 2) - hx)  # the gap between the rag's outline and the
        gy = max(0.0, ly - (SY + BLADE_T / 2), (SY - BLADE_T / 2) - hy)  # blade's faces, across and along y
        touch = (np.hypot(p[0] - SX, p[1] - SY) <= RAG_TOUCH and math.hypot(gx, gy) <= RAG_GAP
                 and lo < tip + BLADE and hi > max(tip, TOP + MOUTH))
    if r is None or not touch:
        _H.gap += 1
        _H.last_s = None  # contact lost: the next touch starts a new slide, and the gap is not credited
        return
    s = p[2] - tip  # the rag's middle along the blade, from the tip
    if _H.last_s is not None and abs(s - _H.last_s) <= SLIDE_MAX:
        a, b = sorted((_H.last_s, s))
        _mark(_H.swept, a, b)
        if b - a >= BIN / 2:  # the oil is rubbed off wherever the sliding pad's face passed (drawn only)
            _mark(_H.rubbed, max(a - (p[2] - lo), TOP + MOUTH - tip), b + (hi - p[2]))
    _H.last_s, _H.gap = s, 0


def _reset_wipe():
    _H.swept[:] = False
    _H.rubbed[:] = False
    _H.last_s, _H.gap = None, 0


def _back_to_start():
    """Seated without a wipe before it (or dropped back in): the next pull is a first pull again."""
    _H.phase = "seated"
    _reset_wipe()


def _update(objects):
    global _H
    if _H.episode != kit.episode():
        _H = _History()
    js = kit.joint(objects, "engine/stick")
    q, held = js.pos, _holding(objects)
    speed = abs(q - _H.last_q) * CONTROL_HZ if _H.last_q is not None else 0.0  # from the joint's travel per step
    _H.last_q, _H.speed = q, speed
    ph = _H.phase
    if ph == "seated":  # seated, blade smeared (the start, or after a seat that did not follow a wipe)
        if q >= OUT:
            _H.phase = "out"
    elif ph == "out":  # out with a smeared blade: the rag can wipe it
        _rag_pass(objects, q)
        if _wiped():
            _H.phase = "clean_out"
        elif q <= SEAT:
            _back_to_start()
        elif q < TIP_IN:  # the tip back in the tube picks up the splash again: a part wipe before it is lost
            _reset_wipe()
    elif ph in ("clean_out", "clean_in", "second", "second_in"):
        if ph == "second" and q >= OUT and speed < STILL and held:  # the true reading: out, held, still
            _H.read_steps += 1
            if _H.read_steps >= READ_STEPS:
                _H.phase, _H.read = "after_read", True
        elif ph == "second":
            _H.read_steps = 0
        if _H.phase != "after_read":
            if q <= SEAT:  # a clean blade seated by hand reads true on the next pull
                if held:
                    _H.phase = "reseated"
                    _H.reseated = _H.reseated or ph in ("clean_out", "clean_in")
                else:
                    _back_to_start()
            elif q < TIP_IN and ph in ("clean_out", "second"):
                _H.phase = "clean_in" if ph == "clean_out" else "second_in"
            elif q >= OUT and ph in ("clean_in", "second_in"):  # back in part way and out again: smeared again
                _H.phase, _H.half_seat = "out", True
                _reset_wipe()
    elif ph == "reseated":
        if q >= OUT:
            _H.phase, _H.read_steps = "second", 0
    elif ph == "after_read":  # the end state is checked in _job; the closing seat must be pushed in by hand
        if q <= SEAT and held:
            _H.final_seat = True
        elif q > SEAT:
            _H.final_seat = False

    now = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    if _H.home is None and len(now) == 2:
        _H.home = now
    _H.still = _H.last is not None and all(g in _H.last and np.linalg.norm(now[g] - _H.last[g]) < HOME_STILL
                                           for g in now)
    _H.last = now
    _draw_oil(objects, q)
    _debug(objects, q, js.vel, held)


# --- The oil on the blade, drawn as markers (visual only; scoring never reads it). A smeared blade shows a ragged dark
# film over its lower 6 cm, which disappears where the rag has passed; after a wipe and a full seat the blade shows a
# clean oil band up to the true level, between the ADD and FULL marks.
OIL_LEVEL = 0.016  # m above the tip: the true level shown on a good reading
SMEAR = ((0.0, 0.060, 0.0013), (0.0, 0.045, 0.0016), (0.0, 0.030, 0.0018))  # (from, to, extra thickness) streaks
_VIS = {"markers": None, "failed": False}
_N_SEG = 24


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        oil = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in (0.30, 0.17, 0.04)),
                                          roughness=0.1, metallic=0.0)
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/World/Visuals/dipstick_oil", markers={"oil": sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0),
                                                                                        visual_material=oil)}))
    except Exception as e:  # visuals are optional
        print(f"dipstick: no oil visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _oil_segments():
    """(from, to, extra thickness) spans above the tip, in m, of oil to draw on the blade now."""
    if _H.phase in ("reseated", "second", "second_in", "after_read"):
        return [(0.0, OIL_LEVEL, 0.0012)]
    if _H.phase in ("clean_out", "clean_in"):
        return []
    segs = []
    for a, b, t in SMEAR:  # unwiped runs of each streak
        k0 = int(a / BIN)
        run = None
        for k in range(k0, int(b / BIN)):
            dirty = k >= len(_H.rubbed) or not _H.rubbed[k]
            if dirty and run is None:
                run = k
            if not dirty and run is not None:
                segs.append((run * BIN, k * BIN, t))
                run = None
        if run is not None:
            segs.append((run * BIN, b, t))
    return segs[:_N_SEG]


def _draw_oil(objects, q):
    if "isaaclab" not in sys.modules:
        return
    m = _markers()
    if m is None:
        return
    tip = TOP + MOUTH - BLADE + q
    tr, sc = [], []
    for a, b, t in _oil_segments():
        tr.append((SX, SY, tip + (a + b) / 2))
        sc.append((BLADE_W * 0.9, BLADE_T + t, max(b - a, 1e-4)))
    while len(tr) < _N_SEG:
        tr.append((0.0, 0.0, -10.0)), sc.append((1e-5, 1e-5, 1e-5))
    m.visualize(translations=np.array(tr, float), orientations=np.tile([0.0, 0.0, 0.0, 1.0], (_N_SEG, 1)),
                scales=np.array(sc, float), marker_indices=np.zeros(_N_SEG, int))


def _debug(objects, q, qd, held):
    path = os.environ.get("DIPLOG")
    if not path:
        return
    f = lambda v: ",".join(f"{x:.3f}" for x in v)  # noqa: E731
    r = objects.get("rag")
    line = (f"q={q:.4f} qd={qd:.3f} held={int(held)} ph={_H.phase} swept={_H.swept.sum() * BIN:.3f} "
            f"rs={_H.read_steps} rag={f(r.pos) if r else '-'} raglo={r.low[2] - TOP if r else 0:.3f} "
            f"ragq={f(r.quat_xyzw) if r else '-'} raghi={r.high[2] - TOP if r else 0:.3f} "
            f"L={f(objects['left_gripper'].pos)} Lq={f(objects['left_gripper'].quat_xyzw)} oL={kit.opening(objects, 'left_gripper'):.2f} "
            f"R={f(objects['right_gripper'].pos)} oR={kit.opening(objects, 'right_gripper'):.2f}")
    with open(path, "a") as fh:
        fh.write(line + "\n")


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still."""
    if _H.home is None or not _H.still:
        return False
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - _H.home[g]) < HOME_TOL for g in GRIPPERS)


def _clear_of(objects, p):
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - p) > LET_GO for g in GRIPPERS)


def _let_go(objects):
    """Both tool points more than 6 cm from the dipstick's handle."""
    return _clear_of(objects, _handle(objects))


def _rag_down(objects):
    """The rag at rest with its bottom within 1 cm of the table, both tool points more than 6 cm from it."""
    r = objects.get("rag")
    return (r is not None and float(r.low[2]) - TOP < RAG_DOWN and r.resting()
            and _clear_of(objects, np.asarray(r.pos, float)))


def _job(objects):
    if not _H.reseated:  # never wiped and pushed back in: a first-pull reading, or nothing
        return 0.0
    if not _H.read:  # wiped and reinserted, but no true second reading (half-seated, or not read)
        return 0.25
    closed = _H.final_seat and _q(objects) <= SEAT and _let_go(objects)
    if not closed:  # read, but the dipstick is left out, half in or still held
        return 0.5
    return 1.0 if _rag_down(objects) else 0.75


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """Job done (wiped, reseated, read, reseated, everything let go, rag down), dipstick still, and with
    REQUIRE_ARMS_HOME both arms home and still. Called first each step, so it also records the history."""
    _update(objects)
    still = _H.speed < STILL
    return _job(objects) == 1.0 and still and (_arms_home(objects) or not REQUIRE_ARMS_HOME)


def score(objects):
    """0 unless the blade was pulled out, wiped (the rag's middle slid, in contact, over at least 3.2 cm of the blade's
    lower 4 cm in that pull, so its face rubbed the 6 cm smear; a dab is not a wipe) and pushed back in by hand
    until the cap seated (within 3 mm). 0.25 for that without a true second reading (pulled again from a full seat and
    held out and still for 1 s). 0.5 for a true reading with the dipstick then left out, part way in or still held.
    0.75 for the dipstick seated and let go but the rag not put down on the table. 1.0 with the rag down and both
    tool points clear; 0.9 if the arms are not back home and still. Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1."""
    return _score(objects)


# --- Scripted demo: the right arm works the dipstick, the left arm the rag.
GRIP_Z = MOUTH + CAP_H + 0.004 + 0.013  # TCP with the pad tips 4 mm up the handle (PAD_TIP_PAST_TCP 13 mm)
PULL = BLADE + 0.02  # the second pull lifts the tip 2 cm clear of the mouth (for the reading)
DEMO_PUSH = -0.004  # push 4 mm past the seat so the cap is pressed down
RAG_Y = SY + 0.017  # the rag's middle 1.7 cm from the blade's axis: the pad, held on edge and tilted 15 deg, reaches
# about 1.8 cm out from its middle, so the arm presses it about 1 mm onto the blade's face (0.15 cm from the axis)
RAG_CARRY = 0.018  # a held object rides about this far toward -y of the tool point (straight-down grip)
ROLL = -1.3  # for the wipe the left wrist rolls about 75 deg: the gripper reaches in from the left side, away from the
# right gripper, and holds the pad on edge with its broad face to the blade
RAG_GRIP_DZ = 0.012  # the rag is picked with the tool point 1.2 cm above its middle, so the fingers hold only its upper
# half and the pad stands out past the fingertips (in the rolled grip the fingers then stay clear of the tube's rim)
ROLLED_CARRY = (-0.017, 0.017)  # in that grip the pad's middle rides this far along y and z of the tool point
PULL_WIPE = BLADE + 0.03  # for the wipe the first pull lifts the tip 3 cm clear of the mouth, so the pad can reach the
# tip without its lower edge catching on the tube's rim
WIPE_BOT, WIPE_TOP = TOP + MOUTH + 0.030, TOP + MOUTH + 0.075  # the rag's middle at the start and end of the stroke,
# which runs up the blade from beside the tip to past 4 cm above it; drawn upward, the pad's tilted leading edge rides
# off the blade instead of digging into it (a downward stroke jammed the pad after about 1 cm)


def demo():
    yaw = math.pi / 2
    R = lambda dz: (SX, SY, TOP + GRIP_Z + dz)  # noqa: E731
    right = [Waypoint(1.5, R(0.06), FINGER_OPEN, "over the handle", yaw),
             Waypoint(1.0, R(0.0), FINGER_OPEN, "down on the handle", yaw),
             Waypoint(0.6, R(0.0), FINGER_CLOSED, "grip", yaw),
             Waypoint(2.0, R(PULL_WIPE), FINGER_CLOSED, "first pull", yaw)]
    t_out = sum(w.duration for w in right)

    lyaw = -math.pi / 2  # the left grip turned the other way round, so its wrist has room to roll
    ry = RAG_START[1] + RAG_CARRY
    rz = RAG_START[2] - TOP
    left = [Waypoint(2.5, (RAG_START[0], ry, TOP + rz + 0.06), FINGER_OPEN, "over the rag", lyaw),
            Waypoint(1.0, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ), FINGER_OPEN, "down on the rag", lyaw),
            Waypoint(0.6, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ), FINGER_CLOSED, "pick up the rag", lyaw),
            Waypoint(1.0, (RAG_START[0], ry, TOP + 0.16), FINGER_CLOSED, "lift", lyaw)]
    wait = t_out - sum(w.duration for w in left)
    lx, ly = SX, RAG_Y - ROLLED_CARRY[0]  # tool point for the rag's middle at (SX, RAG_Y, z + ROLLED_CARRY[1])
    top, bot = WIPE_TOP - ROLLED_CARRY[1], WIPE_BOT - ROLLED_CARRY[1]
    left += [kit.Tilted(max(wait, 3.0), (lx, ly + 0.04, bot), FINGER_CLOSED, "beside the blade's tip", lyaw, roll=ROLL),
             kit.Tilted(1.0, (lx, ly, bot), FINGER_CLOSED, "rag on the blade", lyaw, roll=ROLL),
             kit.Tilted(3.0, (lx, ly, top), FINGER_CLOSED, "wipe up the blade", lyaw, roll=ROLL),
             kit.Tilted(0.8, (lx, ly + 0.05, top + 0.01), FINGER_CLOSED, "rag off", lyaw, roll=ROLL)]
    t_wiped = sum(w.duration for w in left)

    right += [Waypoint(t_wiped - t_out, R(PULL_WIPE), FINGER_CLOSED, "hold out while wiped", yaw),
              Waypoint(2.0, R(DEMO_PUSH), FINGER_CLOSED, "push back in to the seat", yaw),
              Waypoint(0.8, R(DEMO_PUSH), FINGER_CLOSED, "seated", yaw),
              Waypoint(2.0, R(PULL), FINGER_CLOSED, "second pull", yaw),
              Waypoint(2.5, R(PULL), FINGER_CLOSED, "hold still and read", yaw),
              Waypoint(2.0, R(DEMO_PUSH), FINGER_CLOSED, "push back in", yaw),
              Waypoint(0.8, R(DEMO_PUSH), FINGER_CLOSED, "seated", yaw),
              Waypoint(0.6, R(DEMO_PUSH), FINGER_OPEN, "let go", yaw),
              Waypoint(1.0, R(0.07), FINGER_OPEN, "hand up", yaw),
              kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    total = sum(w.duration for w in right)

    left += [Waypoint(1.5, (RAG_START[0], ry, TOP + rz + 0.05), FINGER_CLOSED, "rag back over its place", lyaw),
             Waypoint(1.0, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ + 0.002), FINGER_CLOSED, "rag down", lyaw),
             Waypoint(0.6, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ + 0.002), FINGER_OPEN, "let go of the rag", lyaw),
             Waypoint(0.8, (RAG_START[0], ry, TOP + rz + 0.07), FINGER_OPEN, "hand up", lyaw)]
    left += [kit.Home(max(total - 1.5 - sum(w.duration for w in left), 1.0)), kit.Home(1.5, label="hold at home")]
    return {"left": left, "right": right}
