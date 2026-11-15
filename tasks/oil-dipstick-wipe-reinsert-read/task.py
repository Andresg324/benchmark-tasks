"""A bench model of a small four-stroke engine (a mower or generator engine) with its dipstick seated in the oil fill
tube, a folded shop rag, and an oil check card with ADD, OK and OVER pockets and a marker puck. Checking the oil means
pulling the dipstick, wiping the blade clean on the rag (the first pull reads a false, smeared level because oil
splashes up the tube while the engine runs), pushing it back in until the cap seats, pulling it again and holding it
still to read the level, then seating it again, putting the rag down and reporting the level by leaving the puck in
the matching pocket. The level changes from trial to trial (below ADD, between the marks, or above FULL), so the report
can only be right if the blade is read.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled engine and its sliding dipstick; nothing on disk changes)
import guardkit as gk  # noqa: E402  (the gripper's own shape, for the let-go checks)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = ("Check the oil level on the engine with the dipstick, mark it on the check card with the puck, then put "
               "everything back.")

# The engine (a lawn tractor's single-cylinder engine, modeled as its crankcase side) is fixed to the table. Its oil fill
# tube rises out of the crankcase and slopes up toward the robot at 25 degrees; the dipstick slides along the tube's
# axis. The tube frame has its origin at the middle of the tube's mouth (top rim) and its z axis along the tube, out of
# the mouth; its y axis is world y, and its x axis lies in the vertical plane through the tube (up and away).
SX, SY = 0.38, -0.10  # the tube's mouth in world x, y
MOUTH = 0.125  # the mouth's middle above the table
TUBE_DEG = 25.0  # the tube's slope above horizontal
_TH = math.radians(TUBE_DEG)
U = np.array([-math.cos(_TH), 0.0, math.sin(_TH)])  # the tube's axis, out of the mouth: up and toward the robot
_PHI = _TH - math.pi / 2  # the tube frame is the world frame turned by this about y (its z axis then along U)
TILT = (0.0, float(math.sin(_PHI / 2)), 0.0, float(math.cos(_PHI / 2)))  # x, y, z, w (plain floats: Warp rejects float64)
R_TUBE = np.array([[math.cos(_PHI), 0.0, math.sin(_PHI)], [0.0, 1.0, 0.0], [-math.sin(_PHI), 0.0, math.cos(_PHI)]])
MOUTH_W = np.array([SX, SY, TOP + MOUTH])  # the mouth's middle in world coordinates
BLADE = 0.245  # the dipstick blade, from the cap's underside to the tip
BLADE_W, BLADE_T = 0.010, 0.003  # blade width (along the tube frame's x) and thickness (along y)
CAP_R, CAP_H = 0.016, 0.010  # the cap's disc, which seats on the tube's rim
HANDLE = (0.035, 0.018, 0.035)  # the T-handle above the cap: width (tube x), thickness (y), length along the tube (z)
HANDLE_C = CAP_H + HANDLE[2] / 2  # the handle's middle along the tube from the link origin (the cap's underside)
STICK_LEN = BLADE + CAP_H + HANDLE[2]  # 0.29 m overall (Kohler 32 038 08-S class, about 11-1/2 in)
TUBE_R = 0.012  # the fill tube's outside radius
TUBE_IN = 0.26  # the tube runs this far down into the crankcase below its mouth
RAG = (0.04, 0.04, 0.025)  # the folded shop rag (pad): x, y, z
RAG_START = (0.26, 0.10, TOP + RAG[2] / 2)

# The oil level, set per trial (a real trial sets it with a syringe before each run). ADD is 0.8 cm and FULL 2.0 cm
# above the tip; each case sits 4 mm or more from the nearest mark, so it reads the same by eye from any camera.
ADD_MARK, FULL_MARK = 0.008, 0.020
LEVELS = {"low": 0.004, "ok": 0.014, "over": 0.027}  # m above the tip: below ADD, between the marks, above FULL
CASES = ("low", "ok", "over")
# The oil check card (fixed to the bench, beside the rag) and its marker puck. The pockets run ADD, OK, OVER from the
# robot's left to its right (left to right in the top camera).
CARD_C = (0.35, 0.10)  # the card's middle, x, y
CARD = (0.06, 0.16, 0.010)  # x, y, z: 6 x 16 cm, 1 cm tall
FLOOR = 0.005  # each pocket is 5 mm deep: its floor 5 mm above the table, the card's top 1 cm
POCKET = 0.044  # pockets 4.4 cm square
POCKET_DY = 0.052  # pocket middles 5.2 cm apart along y (0.8 cm walls between them)
POCKETS = {"low": (CARD_C[0], CARD_C[1] + POCKET_DY), "ok": CARD_C, "over": (CARD_C[0], CARD_C[1] - POCKET_DY)}
POCKET_NAME = {"low": "ADD", "ok": "OK", "over": "OVER"}
PUCK_R, PUCK_H = 0.015, 0.02  # the marker puck: 3 cm across, 2 cm tall
PUCK_START = (0.20, 0.15, TOP + PUCK_H / 2)

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
# Held: a gripper with both fingers touching the T-handle or the blade (gk.gripping, contact as below) and their opening
# stopped by it: across the handle's 1.8 cm thickness or 3.5 cm width (from 1.8 cm less 15 % of full opening to 3.5 cm
# plus 3 %), or pinching the blade across its 3 mm thickness or 1 cm width. On a stick this long a hand pulls it out
# hand over hand, pinching the blade near the mouth, as a person does. An empty fist (0) or an open hand around the
# stick does not hold it.
HANDLE_GRIP = (gk.band(HANDLE[1])[0], gk.band(HANDLE[0])[1])
BLADE_GRIP = (gk.band(BLADE_T)[0], gk.band(BLADE_W)[1])
# Let go at the end: no gripper touching the dipstick's cap and handle, the rag or the puck, judged by the gripper's own
# shape (palm and both fingers at their present opening, gk.touching: within 2 mm, the solver's resting gap plus the box
# fit to the meshes). A hand hovering beside them, however close, has let go.
HANDLE_SHAPE = gk.Block((0.0, 0.0, HANDLE_C), HANDLE)  # in the dipstick's frame
BLADE_SHAPE = gk.Block((0.0, 0.0, -BLADE / 2), (BLADE_W, BLADE_T, BLADE))
STICK_SHAPE = (HANDLE_SHAPE, gk.Ring(0.0, CAP_R, 0.0, CAP_H), BLADE_SHAPE)
RAG_SHAPE = gk.Block((0.0, 0.0, 0.0), RAG)
PUCK_SHAPE = gk.Ring(0.0, PUCK_R, -PUCK_H / 2, PUCK_H / 2)
RAG_DOWN = 0.01  # m: the rag at rest with its bottom 1 cm or less above the table
IN_POCKET = 0.012  # m: the puck's middle within this of a pocket's middle along x and y (a puck lying flat in the
# 4.4 cm pocket can be at most 7 mm off; one hanging on a wall with its middle outside the pocket does not count)
SUNK = 0.002  # m: and its bottom at least 2 mm below the card's top (down in the 5 mm pocket, not lying on the rim)
DEBOUNCE = 2  # control steps: the puck in a pocket for 2 steps is a report made (one step can be a physics pop)

ALU = kit.Look((0.62, 0.64, 0.66), 0.4, metallic=0.6)  # cast crankcase
DARK = kit.Look((0.14, 0.14, 0.15), 0.6)
RED = kit.Look((0.72, 0.12, 0.10), 0.5)  # engine shroud
YELLOW = kit.Look((0.95, 0.75, 0.10), 0.45)  # dipstick cap and handle (yellow, as on most small engines)
STEEL = kit.Look((0.82, 0.83, 0.85), 0.25, metallic=0.9)
BLACK = kit.Look((0.06, 0.06, 0.06), 0.7)
RAGLOOK = kit.Look((0.20, 0.35, 0.70), 0.9)  # blue shop towel
WOOD = kit.Look((0.55, 0.40, 0.25), 0.7)
CARDLOOK = kit.Look((0.20, 0.21, 0.23), 0.6)  # dark gray card
POCKET_LOOK = {"low": kit.Look((0.95, 0.55, 0.05), 0.5), "ok": kit.Look((0.10, 0.65, 0.20), 0.5),
               "over": kit.Look((0.80, 0.08, 0.06), 0.5)}  # amber ADD, green OK, red OVER
PUCKLOOK = kit.Look((0.95, 0.95, 0.93), 0.4)  # white puck
LABEL = kit.Look((0.97, 0.97, 0.95), 0.5)  # the pockets' white label symbols
TICK = kit.Look((0.02, 0.02, 0.02), 0.6)  # the ADD and FULL marks


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


BLOCK_C = (0.555, SY)  # the crankcase model's middle on the table (world x, y)


def _block():
    """The engine's crankcase side, fixed to the table beyond the tube's mouth. The fill tube enters its near face about
    8 cm up and runs down inside it; no collider sits on the tube's line (the dipstick slides down it untouched)."""
    cx = BLOCK_C[0]
    body = [  # (low corner, high corner, look, collides), in the block's frame (origin on the table at BLOCK_C)
        ((0.45 - cx, -0.10, 0.0), (0.66 - cx, 0.10, 0.008), WOOD, True),  # bench mounting board
        ((0.47 - cx, -0.09, 0.008), (0.64 - cx, -0.02, 0.13), ALU, True),  # crankcase, right of the tube's line
        ((0.47 - cx, 0.02, 0.008), (0.64 - cx, 0.09, 0.13), ALU, True),  # crankcase, left of it
        ((0.47 - cx, -0.02, 0.10), (0.64 - cx, 0.02, 0.13), ALU, True),  # crankcase top over the tube's line
        ((0.47 - cx, -0.02, 0.008), (0.64 - cx, 0.02, 0.10), ALU, False),  # crankcase round the tube (seen only)
        ((0.49 - cx, -0.08, 0.13), (0.63 - cx, 0.08, 0.17), RED, True),  # shroud over the flywheel
        ((0.64 - cx, -0.04, 0.03), (0.645 - cx, 0.0, 0.06), BLACK, False),  # oil drain plug on the far side
    ]
    meshes, colliders = _parts(body)
    return kit.Prop("block", pos=(*BLOCK_C, TOP), size=(0.21, 0.20, 0.17), meshes=meshes, colliders=colliders)


def _engine():
    """The fill tube and its dipstick, in the tube frame (origin at the mouth's middle, z along the tube)."""
    meshes = (kit.cylinder((0.0, 0.0, -TUBE_IN / 2), TUBE_R, TUBE_IN, YELLOW),
              kit.cylinder((0.0, 0.0, -0.002), TUBE_R + 0.002, 0.004, DARK))  # fill tube and its rim
    colliders = (kit.Collider((0.0, 0.0, -TUBE_IN + 0.004), size=(0.02, 0.02, 0.006)),)  # the tube's buried end
    # Dipstick (link frame at the cap's underside, on the tube's axis): blade below, cap disc and T-handle above.
    marks = FULL_MARK - ADD_MARK  # the crosshatch between ADD and FULL near the tip
    s_meshes = (
        kit.box((0.0, 0.0, -BLADE / 2), (BLADE_W, BLADE_T, BLADE), STEEL),
        kit.box((0.0, 0.0, -BLADE + ADD_MARK + marks / 2), (BLADE_W * 1.02, BLADE_T * 1.2, marks),
                kit.Look((0.45, 0.46, 0.48), 0.5, metallic=0.6)),
        # the ADD and FULL marks: black ticks 1.5 mm long standing 2 mm proud of the blade's edges, so they show
        # beside the oil band from every side
        kit.box((0.0, 0.0, -BLADE + ADD_MARK), (BLADE_W + 0.004, BLADE_T * 1.5, 0.0015), TICK),
        kit.box((0.0, 0.0, -BLADE + FULL_MARK), (BLADE_W + 0.004, BLADE_T * 1.5, 0.0015), TICK),
        kit.cylinder((0.0, 0.0, CAP_H / 2), CAP_R, CAP_H, YELLOW),
        kit.box((0.0, 0.0, HANDLE_C), HANDLE, YELLOW),
    )
    s_colliders = (
        kit.Collider((0.0, 0.0, -BLADE / 2), size=(BLADE_W, BLADE_T, BLADE)),
        kit.Collider((0.0, 0.0, CAP_H / 2), size=(2 * CAP_R, 2 * CAP_R, CAP_H)),
        kit.Collider((0.0, 0.0, HANDLE_C), size=HANDLE),
    )
    # The stick stays where it is let go: in the real curved tube the bent blade's friction holds it against its 50 g
    # weight (about 0.2 N along this slope), so the sim switches its weight off (floating) rather than model the bend.
    stick = kit.Link("stick", "prismatic", "Z", (0.0, 0.0, 0.0), lower=0.0, upper=0.30, start=0.0, mass=0.05,
                     meshes=s_meshes, colliders=s_colliders, damping=0.5, floating=True)
    return kit.Mechanism("engine", pos=tuple(float(v) for v in MOUTH_W), size=(0.04, 0.04, 0.60), tilt_xyzw=TILT, meshes=meshes,
                         colliders=colliders, links=(stick,))


def _rag():
    folds = [kit.box((0.0, 0.0, 0.0), RAG, RAGLOOK),
             kit.box((0.0, 0.0, RAG[2] / 2 - 0.004), (RAG[0] * 1.01, RAG[1] * 1.01, 0.001),
                     kit.Look((0.16, 0.29, 0.60), 0.9))]
    return kit.Prop("rag", pos=RAG_START, size=RAG, mass=0.04, meshes=tuple(folds),
                    colliders=(kit.Collider((0.0, 0.0, 0.0), size=RAG),))


SYM_Z, SYM_T, SYM_W, SYM_L = FLOOR + 0.0013, 0.0006, 0.006, 0.028  # label strokes: height, thickness, width, length


def _symbol(case, y):
    """The pocket's label, drawn in white on its floor so every camera can tell the pockets apart without color: a
    minus sign for ADD (too little oil), a check mark for OK and a plus sign for OVER (too much). Strokes are 6 mm wide
    and 2.8 cm long; the minus runs across the robot's view (along y)."""
    if case == "low":
        return (kit.box((0.0, y, SYM_Z), (SYM_W, SYM_L, SYM_T), LABEL),)
    if case == "over":
        return (kit.box((0.0, y, SYM_Z), (SYM_W, SYM_L, SYM_T), LABEL),
                kit.box((0.0, y, SYM_Z), (SYM_L, SYM_W, SYM_T), LABEL))
    # check mark, seen from the robot (x away, y to its left): a short stroke down to the right, a long one up
    path = [(0.001, y + 0.011, SYM_Z), (-0.007, y + 0.004, SYM_Z), (0.010, y - 0.012, SYM_Z)]
    return (kit.band(path, SYM_T, SYM_W, LABEL, normal=(0, 0, 1)),)


def _card():
    """The oil check card: a 5 mm base and 5 mm walls round three 4.4 cm pockets, each floor colored and labelled
    with a white symbol (minus ADD, check mark OK, plus OVER)."""
    cx, cy, h = CARD[0] / 2, CARD[1] / 2, CARD[2]
    p = POCKET / 2
    parts = [((-cx, -cy, 0.0), (cx, cy, FLOOR), CARDLOOK, True),  # base
             ((-cx, -cy, FLOOR), (-p, cy, h), CARDLOOK, True),  # near rail
             ((p, -cy, FLOOR), (cx, cy, h), CARDLOOK, True)]  # far rail
    ys = sorted(POCKETS[k][1] - CARD_C[1] for k in CASES)
    edges = [-cy] + [y for m in ys for y in (m - p, m + p)] + [cy]
    for a, b in zip(edges[0::2], edges[1::2]):  # the end walls and the walls between pockets
        parts.append(((-p, a, FLOOR), (p, b, h), CARDLOOK, True))
    meshes, colliders = _parts(parts)
    for k in CASES:  # colored pocket floors (visual, 1 mm), each with its white label symbol on top
        y = POCKETS[k][1] - CARD_C[1]
        meshes += (kit.box((0.0, y, FLOOR + 0.0005), (POCKET * 0.98, POCKET * 0.98, 0.001), POCKET_LOOK[k]),)
        meshes += _symbol(k, y)
    return kit.Prop("card", pos=(*CARD_C, TOP), size=CARD, meshes=meshes, colliders=colliders)


def _puck():
    return kit.Prop("puck", pos=PUCK_START, size=(2 * PUCK_R, 2 * PUCK_R, PUCK_H), mass=0.015,
                    meshes=(kit.cylinder((0.0, 0.0, 0.0), PUCK_R, PUCK_H, PUCKLOOK),
                            kit.cylinder((0.0, 0.0, PUCK_H / 2), PUCK_R * 0.35, 0.0006, DARK)),
                    colliders=(kit.Collider((0.0, 0.0, 0.0), radius=PUCK_R, height=PUCK_H),))


OBJECTS = [_block(), _engine(), _rag(), _card(), _puck()]


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
        self.case = None  # this trial's oil level: "low", "ok" or "over" (set at the first step)
        self.in_steps = 0  # control steps in a row the puck has been down in a pocket
        self.early = False  # the puck went into a pocket before the true reading: a guess, not a report


_H = _History()


def _trial_case(rag_xy):
    """This trial's oil level. OIL_CASE=low|ok|over sets it (to run each case). Otherwise it comes from the seeded
    reset: every free object starts jittered by up to 3 cm, and the rag's offset from its place, in whole millimetres
    (x + y), modulo 3 picks the case, so it changes with the seed but the rag's position is no cue a camera could use.
    With no jitter (object_jitter=0) the rag is in place and the level is between the marks."""
    env = os.environ.get("OIL_CASE", "").strip().lower()
    if env in LEVELS:
        return env
    off = np.asarray(rag_xy, float)[:2] - np.asarray(RAG_START[:2])
    if np.linalg.norm(off) < 0.002:
        return "ok"
    return CASES[int(math.floor((off[0] + off[1]) / 0.001)) % 3]


def _pocket(objects):
    """The pocket the puck is down in (its middle within 1.2 cm of the pocket's middle, its bottom 2 mm or more below
    the card's top), or None."""
    pk = objects.get("puck")
    if pk is None or float(pk.low[2]) > TOP + CARD[2] - SUNK:
        return None
    p = np.asarray(pk.pos, float)
    for k, (x, y) in POCKETS.items():
        if abs(p[0] - x) <= IN_POCKET and abs(p[1] - y) <= IN_POCKET:
            return k
    return None


def _report(objects):
    """The level reported at the end: the pocket the puck rests in, let go (no gripper touching it)."""
    k = _pocket(objects)
    pk = objects.get("puck")
    if k is None or not pk.resting() or gk.touching(objects, "puck", PUCK_SHAPE):
        return None
    return k


def _q(objects):
    return kit.joint(objects, "engine/stick").pos


def _holding(objects):
    """A gripper holding the dipstick: both fingers touching its T-handle or its blade with their opening stopped by
    it, not an empty fist or an open hand beside it."""
    return bool(gk.gripping(objects, "engine/stick", HANDLE_SHAPE, HANDLE_GRIP)
                or gk.gripping(objects, "engine/stick", BLADE_SHAPE, BLADE_GRIP))


def _wiped():
    """The rag's middle has slid over at least 80% of the blade's lower 4 cm, in contact, during this pull."""
    return _H.swept.sum() >= WIPE_FRAC * len(_H.swept) - 1e-9


def _mark(arr, a, b):
    a, b = max(int(round(a / BIN)), 0), min(int(round(b / BIN)), len(arr))
    if b > a:
        arr[a:b] = True


def _tube(p):
    """World point p in the tube frame: (across, along y, along the tube from the mouth)."""
    return R_TUBE.T @ (np.asarray(p, float) - MOUTH_W)


def _rag_pass(objects, q):
    """Track one pull's wipe, in the tube frame. The rag touches the blade when its middle is within 2.7 cm (RAG_TOUCH)
    of the blade's axis, its own shape (the pad as it lies, turned with the rag) is within 2 mm (RAG_GAP) of the blade's
    faces, and its extent along the tube overlaps the blade out of the mouth. Between two touching steps the stretch of
    blade its middle slid over is marked; a rag held still on the blade (a dab) marks nothing."""
    r = objects.get("rag")
    tip = q - BLADE  # the blade's tip along the tube from the mouth
    touch = False
    if r is not None:
        c = _tube(r.pos)
        rr = gk._rot(r.quat_xyzw)
        half = float(np.abs(U @ rr) @ (np.asarray(RAG) / 2))  # the pad's half extent along the tube
        st = objects.get("engine/stick")
        gap = (gk._box_gap(np.asarray(r.pos, float), rr, gk.Block((0.0, 0.0, 0.0), RAG), np.asarray(st.pos, float),
                           gk._rot(st.quat_xyzw), BLADE_SHAPE) if st is not None else 1.0)
        touch = (math.hypot(c[0], c[1]) <= RAG_TOUCH and gap <= RAG_GAP and c[2] - half < q
                 and c[2] + half > max(tip, 0.0))
    if not touch:
        _H.gap += 1
        _H.last_s = None  # contact lost: the next touch starts a new slide, and the gap is not credited
        return
    s = c[2] - tip  # the rag's middle along the blade, from the tip
    if _H.last_s is not None and abs(s - _H.last_s) <= SLIDE_MAX:
        a, b = sorted((_H.last_s, s))
        _mark(_H.swept, a, b)
        if b - a >= BIN / 2:  # the oil is rubbed off wherever the sliding pad's face passed (drawn only)
            _mark(_H.rubbed, max(a - half, -tip), b + half)
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
    if _H.case is None:
        r = objects.get("rag")
        _H.case = _trial_case(r.pos if r is not None else RAG_START)
    # A report made before the true reading is a guess: the puck down in any pocket for 2 steps before then marks it.
    _H.in_steps = _H.in_steps + 1 if _pocket(objects) is not None else 0
    if _H.in_steps >= DEBOUNCE and not _H.read:
        _H.early = True
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

    _draw_oil(objects, q)


# --- The oil on the blade, drawn as markers (visual only; the scorer knows the level from the trial's case). A smeared
# blade shows a ragged amber film over its lower 6 cm, which disappears where the rag has passed; after a wipe and a full
# seat the blade shows a clean oil band from the tip up to this trial's level (below ADD, between the marks, or above FULL).
SMEAR = ((0.0, 0.060, 0.0013), (0.0, 0.045, 0.0016), (0.0, 0.030, 0.0018))  # (from, to, extra thickness) streaks
_VIS = {"markers": None, "failed": False}
_N_SEG = 24


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        oil = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in (0.85, 0.52, 0.06)),
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
        return [(0.0, LEVELS[_H.case or "ok"], 0.0012)]
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
    tip = q - BLADE  # along the tube from the mouth
    tr, sc = [], []
    for a, b, t in _oil_segments():
        tr.append(tuple(MOUTH_W + (tip + (a + b) / 2) * U))
        sc.append((BLADE_W + 0.001, BLADE_T + t, max(b - a, 1e-4)))  # the film wraps the blade's edges
    while len(tr) < _N_SEG:
        tr.append((0.0, 0.0, -10.0)), sc.append((1e-5, 1e-5, 1e-5))
    m.visualize(translations=np.array(tr, float), orientations=np.tile(TILT, (_N_SEG, 1)),
                scales=np.array(sc, float), marker_indices=np.zeros(_N_SEG, int))


def _let_go(objects):
    """No gripper touching the dipstick's cap or handle."""
    return not gk.touching(objects, "engine/stick", STICK_SHAPE)


def _rag_down(objects):
    """The rag at rest, its bottom 1 cm or less above the table, and no gripper touching it."""
    r = objects.get("rag")
    return (r is not None and float(r.low[2]) - TOP < RAG_DOWN and r.resting()
            and not gk.touching(objects, "rag", RAG_SHAPE))


def _job(objects):
    report = _report(objects)
    if _H.early or (_pocket(objects) is not None and not _H.read):  # reported before the true reading: a guess
        return 0.0
    if report is not None and report != _H.case:  # a wrong report: worse than none, so guessing never pays
        return 0.0
    if not _H.reseated:  # never wiped and pushed back in: a first-pull reading, or nothing
        return 0.0
    if not _H.read:  # wiped and reinserted, but no true second reading (half-seated, or not read)
        return 0.25
    if report is None:  # read, but no level reported (puck not in a pocket, still held, or on a wall)
        return 0.5
    closed = _H.final_seat and _q(objects) <= SEAT and _let_go(objects)
    return 1.0 if closed and _rag_down(objects) else 0.75


def success(objects):
    """Job done (wiped, reseated, read, reseated, the right level reported, everything let go, rag down) and the
    dipstick still. Where the arms end up does not matter. Called first each step, so it also records the history."""
    _update(objects)
    return _job(objects) == 1.0 and _H.speed < STILL


def score(objects):
    """0 unless the blade was pulled out, wiped (the rag's middle slid, in contact, over at least 3.2 cm of the blade's
    lower 4 cm in that pull, so its face rubbed the 6 cm smear; a dab is not a wipe) and pushed back in by hand
    until the cap seated (within 3 mm). Also 0 for a wrong report (the puck left in another level's pocket) or a
    report made before the true reading (the puck in any pocket before then). 0.25 for the wipe and reseat without a
    true second reading (pulled again from a full seat and held out and still for 1 s). 0.5 for a true reading with
    no report (the puck not let go in a pocket). 0.75 for the right report with the dipstick not seated by hand and
    let go, or the rag not put down. 1.0 for all of it. Possible scores: 0, 0.25, 0.5, 0.75, 1."""
    return _job(objects)


# --- Scripted demo: the right arm works the dipstick hand over hand along the sloping tube, the left arm the rag.
YAW = 0.0  # the right grip: straight down, fingers closing across world y (across the blade's and handle's thickness)
GRIP_UP = 0.003  # the tool point 3 mm above the gripped part's axis: the pad tips (12.8 mm past it) reach 1 cm below it
A_HANDLE = 0.030  # the handle is gripped 3 cm up the tube from the cap's underside (its middle is at 2.75 cm), so the
# pad tips stay over 1 cm clear of the cap's disc
A_LOW = 0.035  # the hand-over-hand regrasp pinches the blade 3.5 cm out of the mouth: the fingers stay clear of its rim
A_HIGH = 0.1775  # the farthest a stroke carries the hand up the tube (the arm's reach at this height, with margin)
Q_REGRASP = A_HIGH - A_HANDLE  # 0.1475: the first stroke, on the handle, pulls the cap this far out
Q_WIPE = 0.285  # each pull is planned to leave the tip 4 cm out of the mouth; in the sim the pinches give about 1.6 cm
Q_READ = Q_WIPE  # (the stick ends about 0.269, the tip 2.4 cm out), still well past "out" (0.25) for the reading
DEMO_PUSH = -0.004  # push 4 mm past the seat so the cap is pressed down
RAG_DY = 0.017  # the rag's middle 1.7 cm from the blade's axis (along +y): the pad, held on edge and tilted 15 deg,
# reaches about 1.8 cm out from its middle, so the arm presses it about 1 mm onto the blade's face
RAG_CARRY = 0.018  # a held object rides about this far toward -y of the tool point (straight-down grip)
ROLL = -1.3  # for the wipe the left wrist rolls about 75 deg: the gripper reaches in from the left side, away from the
# right gripper, and holds the pad on edge with its broad face to the blade
RAG_GRIP_DZ = 0.012  # the rag is picked with the tool point 1.2 cm above its middle, so the fingers hold only its upper
# half and the pad stands out past the fingertips
ROLLED_CARRY = (-0.017, 0.017)  # in that grip the pad's middle rides this far along y and z of the tool point
WIPE_A = (0.02, 0.085)  # the rag's middle drawn up the blade from 2 cm to 8.5 cm out of the mouth: from just past the
# tip (which ends 2.4-4 cm out) to over 4 cm above it, whatever the pinches' give. Drawn up the blade (toward the
# robot), the pad's tilted leading edge rides off the blade instead of digging into it


PUCK_CARRY = 0.0  # the puck is gripped round its middle (the rag is held near one edge)
PUCK_GRIP_Z = PUCK_H + 0.003  # tool point 3 mm above the puck's top: the pads (13 mm past it) hold its upper 1 cm
PUCK_DROP_Z = FLOOR + PUCK_GRIP_Z + 0.002  # let go with the puck's bottom 2 mm above the pocket's floor


def _on_tube(a, dy=0.0, dz=0.0):
    """The world point a m along the tube's axis from the mouth's middle, shifted dy along y and dz up."""
    p = MOUTH_W + a * U
    return (float(p[0]), float(p[1] + dy), float(p[2] + dz))


def _grip(a):
    return _on_tube(a, dz=GRIP_UP)


def _stroke(a0, a1, what, hover=0.015, t_move=1.4, t_stroke=2.0):
    """One hand-over-hand stroke of the right hand along the tube: over the spot, down, pinch, carry from a0 to a1
    along the tube. Between strokes the open hand slides along the tube 1.5 cm above the stick, its open fingers (9.4 cm
    apart) straddling the blade and handle, so it never knocks the stick."""
    up = lambda a: _on_tube(a, dz=GRIP_UP + hover)  # noqa: E731
    return [Waypoint(t_move, up(a0), FINGER_OPEN, f"over the {what}", YAW),
            Waypoint(0.8, _grip(a0), FINGER_OPEN, f"down on the {what}", YAW),
            Waypoint(0.6, _grip(a0), FINGER_CLOSED, f"pinch the {what}", YAW),
            Waypoint(t_stroke, _grip(a1), FINGER_CLOSED, f"{'pull' if a1 > a0 else 'push'} along the tube", YAW)]


def _release(a, hover=0.015):
    return [Waypoint(0.5, _grip(a), FINGER_OPEN, "let go", YAW),
            Waypoint(0.5, _on_tube(a, dz=GRIP_UP + hover), FINGER_OPEN, "hand up", YAW)]


def demo():
    # first pull: the handle up the tube as far as the hand reaches, let go (the stick stays), pinch the blade at the
    # mouth and draw it out the rest of the way
    right = (_stroke(A_HANDLE, A_HIGH, "handle", hover=0.03, t_move=1.5) + _release(A_HIGH)
             + _stroke(A_LOW, A_LOW + Q_WIPE - Q_REGRASP, "blade"))
    t_out = sum(w.duration for w in right)
    a_hold = A_LOW + Q_WIPE - Q_REGRASP  # where the hand holds the blade, 13.75 cm above its tip

    lyaw = -math.pi / 2  # the left grip turned the other way round, so its wrist has room to roll
    ry = RAG_START[1] + RAG_CARRY
    rz = RAG_START[2] - TOP
    left = [Waypoint(2.5, (RAG_START[0], ry, TOP + rz + 0.06), FINGER_OPEN, "over the rag", lyaw),
            Waypoint(1.0, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ), FINGER_OPEN, "down on the rag", lyaw),
            Waypoint(0.6, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ), FINGER_CLOSED, "pick up the rag", lyaw),
            Waypoint(1.0, (RAG_START[0], ry, TOP + 0.18), FINGER_CLOSED, "lift", lyaw)]
    wait = t_out - sum(w.duration for w in left)
    rag_tcp = lambda a, dy=0.0, dz=0.0: _on_tube(a, RAG_DY - ROLLED_CARRY[0] + dy,  # noqa: E731
                                                  -ROLLED_CARRY[1] + dz)
    # the rag waits up high until the right hand has drawn the blade out, then comes in beside the tip
    left += [Waypoint(max(wait - 3.0, 0.2), (RAG_START[0], ry, TOP + 0.18), FINGER_CLOSED, "wait with the rag", lyaw),
             kit.Tilted(3.0, rag_tcp(WIPE_A[0], 0.04), FINGER_CLOSED, "beside the blade's tip", lyaw, roll=ROLL),
             kit.Tilted(1.0, rag_tcp(WIPE_A[0]), FINGER_CLOSED, "rag on the blade", lyaw, roll=ROLL),
             kit.Tilted(4.0, rag_tcp(WIPE_A[1]), FINGER_CLOSED, "wipe up the blade", lyaw, roll=ROLL),
             kit.Tilted(0.8, rag_tcp(WIPE_A[1], 0.05, 0.01), FINGER_CLOSED, "rag off", lyaw, roll=ROLL)]
    t_wiped = sum(w.duration for w in left)

    # push back in until the cap seats: the blade in to the mouth, then the handle the rest of the way
    right += [Waypoint(max(t_wiped - t_out, 0.2), _grip(a_hold), FINGER_CLOSED, "hold out while wiped", YAW),
              Waypoint(2.0, _grip(A_LOW), FINGER_CLOSED, "push the blade back in", YAW)]
    right += _release(A_LOW)
    right += _stroke(A_HIGH, A_HANDLE + DEMO_PUSH, "handle")
    right += [Waypoint(0.8, _grip(A_HANDLE + DEMO_PUSH), FINGER_CLOSED, "seated", YAW)]
    right += _release(A_HANDLE + DEMO_PUSH)
    # second pull, held still to read
    a_read = A_LOW + Q_READ - Q_REGRASP
    right += _stroke(A_HANDLE, A_HIGH, "handle") + _release(A_HIGH) + _stroke(A_LOW, a_read, "blade")
    t_hold = sum(w.duration for w in right)  # the second pull is out
    right += [Waypoint(2.5, _grip(a_read), FINGER_CLOSED, "hold still and read", YAW)]
    t_read = sum(w.duration for w in right)  # the reading is taken
    # push back in until the cap seats, let go
    right += [Waypoint(2.0, _grip(A_LOW), FINGER_CLOSED, "push the blade back in", YAW)]
    right += _release(A_LOW)
    right += _stroke(A_HIGH, A_HANDLE + DEMO_PUSH, "handle")
    right += [Waypoint(0.8, _grip(A_HANDLE + DEMO_PUSH), FINGER_CLOSED, "seated", YAW)]
    right += _release(A_HANDLE + DEMO_PUSH, hover=0.03)
    right += [kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    total = sum(w.duration for w in right)

    left += [Waypoint(2.5, (RAG_START[0], ry, TOP + rz + 0.05), FINGER_CLOSED, "rag back over its place", lyaw),
             Waypoint(1.0, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ + 0.002), FINGER_CLOSED, "rag down", lyaw),
             Waypoint(0.6, (RAG_START[0], ry, TOP + rz + RAG_GRIP_DZ + 0.002), FINGER_OPEN, "let go of the rag", lyaw),
             Waypoint(0.8, (RAG_START[0], ry, TOP + rz + 0.07), FINGER_OPEN, "hand up", lyaw)]
    # The report. This scripted demonstration is told this trial's level (OIL_CASE, or between the marks with no
    # jitter); it does not read the blade, so it only shows the motions can be done. The scored policy is told nothing
    # and has to read the oil band off the blade.
    case = _trial_case(RAG_START[:2])
    px, py = PUCK_START[0], PUCK_START[1] + PUCK_CARRY
    kx, ky = POCKETS[case][0], POCKETS[case][1] + PUCK_CARRY
    left += [Waypoint(max(t_hold - 3.0 - sum(w.duration for w in left), 1.5), (px, py, TOP + PUCK_GRIP_Z + 0.06),
                      FINGER_OPEN, "over the puck", lyaw),
             Waypoint(1.0, (px, py, TOP + PUCK_GRIP_Z), FINGER_OPEN, "down on the puck", lyaw),
             Waypoint(0.6, (px, py, TOP + PUCK_GRIP_Z), FINGER_CLOSED, "pick up the puck", lyaw),
             Waypoint(1.0, (px, py, TOP + PUCK_GRIP_Z + 0.05), FINGER_CLOSED, "lift the puck", lyaw),
             Waypoint(1.5, (kx, ky, TOP + PUCK_GRIP_Z + 0.04), FINGER_CLOSED, f"over the {POCKET_NAME[case]} pocket",
                      lyaw)]
    left += [Waypoint(max(t_read + 0.5 - sum(w.duration for w in left), 0.2), (kx, ky, TOP + PUCK_GRIP_Z + 0.04),
                      FINGER_CLOSED, "wait for the reading", lyaw),
             Waypoint(1.0, (kx, ky, TOP + PUCK_DROP_Z), FINGER_CLOSED, "puck into the pocket", lyaw),
             Waypoint(0.6, (kx, ky, TOP + PUCK_DROP_Z), FINGER_OPEN, "let go of the puck", lyaw),
             Waypoint(0.8, (kx, ky, TOP + PUCK_DROP_Z + 0.07), FINGER_OPEN, "hand up", lyaw)]
    left += [kit.Home(max(total - 1.5 - sum(w.duration for w in left), 1.0)), kit.Home(1.5, label="hold at home")]
    return {"left": left, "right": right}
