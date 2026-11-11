"""A router table with a bearing-guided rabbeting bit and a steel starting pin in its insert plate, and a cherry board
(25 x 15 cm, 1.27 cm thick) with one long edge band-sawn to a gentle convex curve, lying on the table top on the
infeed (right) side of the bit, its curved edge toward the bit.

To rout a curved edge on a router table, where there is no fence to steady the work, a cabinetmaker first braces the
board against the starting pin, a steel pin in the insert plate a few centimetres from the bit, then pivots it about
the pin into the spinning bit until its edge rides the bit's bearing. Only then does the board come off the pin and
follow the bearing round, fed right to left against the bit's rotation. Fed straight into the bit without the pin,
the cutter's first bite on an unsupported edge grabs the board and throws it, or pulls the hands toward the cutter.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import pivotkit as pv  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled table, bit and board; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = ("Rout the rabbet along the board's curved edge on the router table, then leave the board on the "
               "table past the bit.")
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
# The robot's table is the router table's top. The bit turns counterclockwise seen from above (a router hangs
# upside down under the table), so its front (robot-side) cutting edge moves to the right: work is fed right to left
# (+y), against it, and the starting pin sits on the infeed side, to the bit's right.
BIT = np.array([0.330, 0.060])  # the bit's axis
RB = 0.00635  # m: the bearing's radius (a 1/2 in ball bearing on the bit's tip)
RC = 0.0159  # m: the cutter's radius (a 1-1/4 in rabbeting bit): the rabbet is RC - RB = 9.5 mm (3/8 in) wide
CUT_H = 0.0064  # m: the cutter stands 1/4 in above the table (the rabbet's depth); the bearing rides above it
BEAR_Z = (0.0066, 0.0114)  # m: the bearing's bottom and top above the table (3/16 in thick)
PIN = np.array([BIT[0] + 0.0075, BIT[1] - 0.050])  # the starting pin: 5 cm to the bit's right, in the insert plate
RP = 0.0032  # m: the pin's radius (a 1/4 in steel pin)
PIN_H = 0.030  # m: it stands 3 cm above the table

R = 0.300  # m: the curved edge's radius (a gentle arc: 2.7 cm of bow over the board's 25 cm)
HALF = 0.125  # m: half the board's length (25 cm)
DEPTH = 0.150  # m: the board's depth across its middle, back edge to the top of the curve
T = 0.0127  # m: the board's thickness (1/2 in cherry)
GAP = 0.001  # m: the board's underside rides 1 mm above the table on its joints (pivotkit carries it flat)
BOARD_MASS = 0.22  # kg: 25 x 15 x 1.27 cm of cherry (0.56 g/cm3) less the bow
ALPHA = math.asin(HALF / R)  # the half-angle the curve spans about its centre (24.6 deg)
ARC = 2 * ALPHA * R  # m: the curve's length, 25.8 cm
Z_MID = GAP + T / 2  # the board's mid-thickness above the table (its frame's height)
SLIDE_DAMP = 4.0  # N*s/m: 220 g of board sliding on a waxed table top (friction as damping: 0.2 N at 5 cm/s)
RG2 = (0.25 ** 2 + 0.15 ** 2) / 12  # m^2: the board's radius of gyration squared about its centre


def _circle_meet(c0, r0, c1, r1, pick):
    """The intersection of two circles (centres c0, c1, radii r0, r1); `pick` chooses between the two."""
    d = np.linalg.norm(c1 - c0)
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    mid = c0 + a * (c1 - c0) / d
    perp = np.array([-(c1 - c0)[1], (c1 - c0)[0]]) / d
    return min((mid + h * perp, mid - h * perp), key=pick)


# The board's frame (pivotkit): its origin is the pin's centre with the board in its "braced" pose, where the curve
# touches both the pin and the bearing, its long sides along y. The curve's centre in that frame:
C_L = _circle_meet(PIN, R + RP, BIT, R + RB, pick=lambda c: c[0]) - PIN
BACK = C_L[0] + R - DEPTH  # the back edge's x in the board's frame
ENDS = (C_L[1] - HALF, C_L[1] + HALF)  # the trailing (right, -y) and leading (left, +y) ends' y
_N_ARC = 64
OUTLINE = np.array([C_L + R * np.array([math.cos(a), math.sin(a)]) for a in np.linspace(-ALPHA, ALPHA, _N_ARC)]
                   + [(BACK, ENDS[1]), (BACK, ENDS[0])])  # convex, counterclockwise: the curve, then the back corners


def pose(J):
    """The board's turn (rad) and slides from the joints: a point p of the board's frame is at
    PIN + rot(turn) @ (p + slides) in the world."""
    return J["board_turn"], np.array([J["board_x"], J["board"]])


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def to_board(J, w):
    """A world (x, y) point in the board's frame."""
    th, s = pose(J)
    return _rot(-th) @ (np.asarray(w, float) - PIN) - s


def to_world(J, p):
    th, s = pose(J)
    return PIN + _rot(th) @ (np.asarray(p, float) + s)


def outline_dist(p):
    """Signed distance (m) from point p (board frame) to the board's outline: negative inside the board."""
    best, inside = 1e9, True
    n = len(OUTLINE)
    for i in range(n):
        a, b = OUTLINE[i], OUTLINE[(i + 1) % n]
        e = b - a
        t = min(max(np.dot(p - a, e) / np.dot(e, e), 0.0), 1.0)
        best = min(best, float(np.linalg.norm(p - (a + t * e))))
        if e[0] * (p - a)[1] - e[1] * (p - a)[0] < 0:  # right of a counterclockwise edge: outside
            inside = False
    return -best if inside else best


def curve_point(p):
    """(distance, angle): how far p (board frame) is from the curved edge and the angle (rad, about the curve's
    centre; +ALPHA is the leading end) of the curve's nearest point; None if the nearest point is past an end."""
    v = p - C_L
    ang = math.atan2(v[1], v[0])
    if abs(ang) > ALPHA:
        return None
    return float(np.linalg.norm(v) - R), ang


def pin_gap(J):
    """The gap (m) between the pin's surface and the board (negative: pressed into it)."""
    return outline_dist(to_board(J, PIN)) - RP


def bit_dist(J):
    """How far the bit's axis is from the board (negative: the axis is under the board)."""
    return outline_dist(to_board(J, BIT))


# --- Scoring thresholds.
BRACE = 0.0015  # m: the board is on the pin within 1.5 mm of its surface (the pin's collider keeps it from closer;
#                 1.5 mm is the play of a hand-held board against a 1/4 in pin)
ON_BEARING = 0.0015  # m: the curve is on the bearing within 1.5 mm of its surface (the rabbet then within 1.5 mm of
#                      full width; the bearing's collider keeps the curve from going deeper)
CLEAR = 0.005  # m: at the end, no part of the board within 5 mm of the cutter
REQUIRED = 0.180  # m: routed along at least 18 cm of the 25.8 cm curve (70%): a start braced on the pin, which is 5 cm
#                  from the bit, enters the curve 3 to 7 cm in from its leading end, and that short stretch is routed
#                  in a second pass from the other end; everything after the entry point must be routed in one feed
HALFWAY = REQUIRED / 2  # m: 9 cm, half the cut: real progress round the bearing
BIN = 0.002  # m: the routed length is kept as 2 mm stretches of the curve (129 of them)
N_BINS = int(round(ARC / BIN))
HANDS = 0.076  # m: no tool point within 3 in (7.6 cm) of the bit's axis while the pads are low (router-table manuals'
#                three-inch rule for hands near a bit)
HANDS_LOW = 0.05  # m: "low" is pad tips within 5 cm of the table top (the bit and the board's top are below 1.4 cm)
PRESS = 0.015  # m: pad tips within 1.5 cm above the board's top over it (grown 1 cm) are on it: still holding it
DT = 0.05  # s per control step (20 Hz)
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving
HOME_STEPS = round(SUCCESS_HOLD_S / DT)  # 20 steps: home and still for 1 s, also when the run ends at its time limit
GRIPPERS = ("left_gripper", "right_gripper")


def on_bearing(J):
    """The curved edge rides the bearing: the bit's axis within the bearing's radius plus 1.5 mm of the curve."""
    c = curve_point(to_board(J, BIT))
    return c is not None and c[0] <= RB + ON_BEARING


def braced(J):
    """The pivot condition: the board within 1.5 mm of the pin and its curve not yet on the bearing. While braced the
    board can only turn about the pin (pivotkit); once the curve rides the bearing it comes off the pin."""
    return pin_gap(J) <= BRACE and not on_bearing(J)


CENTRE = (float(C_L[0] + R - DEPTH / 2), float(C_L[1]))  # the board's middle in its frame (drag acts there)


FREE_TURN = 10.0  # off the pin the board's turn is damped 10 times harder (see drag)


def drag(J):
    """The board's drag on the table as joint damping. The chain's turn is about the pin, not the board's centre, so
    on the pin (slides locked) its damping is the slide damping times (the centre's distance from the pin squared
    plus the board's radius of gyration squared), what dragging the board round the pin costs. Off the pin, per-joint
    damping can't hold the coupling between that turn and the slides, so a push at the board's centre would swing it
    round the far-off pin instead of sliding it; the turn is damped 10 times harder there so pushes slide the board."""
    centre = to_world(J, CENTRE)
    turn = SLIDE_DAMP * (float(np.sum((centre - PIN) ** 2)) + RG2)
    return SLIDE_DAMP, turn if braced(J) else FREE_TURN * turn


PIVOT = pv.Pivot("board", engaged=braced, free="slide", resist=drag)

# The board starts on the table to the right of the bit, turned 14 deg counterclockwise from the braced pose (its
# leading end swung back toward the robot) and 1.5 cm nearer the robot and 2.5 cm further right: 2 cm off the pin.
START_TURN = math.radians(14.0)
START_SHIFT = np.array([-0.015, -0.025])
START = (*(float(v) for v in _rot(-START_TURN) @ START_SHIFT), START_TURN)  # (x slide, y slide, turn)

WOOD = kit.Look((0.56, 0.31, 0.19), 0.6)  # cherry, planed
END_GRAIN = kit.Look((0.44, 0.23, 0.14), 0.7)
SAWN = kit.Look((0.47, 0.26, 0.16), 0.8)  # the band-sawn curve, a little darker and rougher
FRESH = kit.Look((0.93, 0.74, 0.55), 0.6)  # freshly routed cherry (the rabbet), drawn by markers
LAMINATE = kit.Look((0.80, 0.81, 0.79), 0.4)
PLATE = kit.Look((0.12, 0.12, 0.13), 0.4)  # phenolic insert plate
STEEL = kit.Look((0.78, 0.79, 0.80), 0.25, metallic=0.8)
CARBIDE = kit.Look((0.80, 0.18, 0.10), 0.35, metallic=0.4)  # the bit's coated body and carbide wings
DARK = kit.Look((0.20, 0.20, 0.21), 0.5)


def _prism(poly, z0, z1, look):
    """A flat convex prism: polygon `poly` (counterclockwise (x, y) points) from z0 to z1."""
    n = len(poly)
    pts = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    counts, idx = [n, n], list(range(n))[::-1] + [n + i for i in range(n)]
    for i in range(n):
        j = (i + 1) % n
        counts.append(4)
        idx += [i, j, n + j, n + i]
    return kit._mesh(pts, counts, idx, look)


def _board_links():
    """The board on pivotkit's planar chain (frame: origin at the pin's centre in the braced pose, z at mid-board)."""
    h = T / 2
    m = [_prism([tuple(p) for p in OUTLINE], -h, h, WOOD)]
    # the band-sawn curve shows as a darker 1.5 mm strip along the top of the curve
    inner = [tuple(C_L + (R - 0.0015) * np.array([math.cos(a), math.sin(a)])) for a in np.linspace(-ALPHA, ALPHA, 48)]
    outer = [tuple(C_L + (R + 0.0002) * np.array([math.cos(a), math.sin(a)])) for a in np.linspace(-ALPHA, ALPHA, 48)]
    for k in range(47):
        m.append(_prism([inner[k], outer[k], outer[k + 1], inner[k + 1]], h - 0.0001, h + 0.0002, SAWN))
    for y in ENDS:  # end grain at both ends
        x1 = C_L[0] + math.sqrt(R * R - (y - C_L[1]) ** 2)
        m.append(kit.box(((BACK + x1) / 2, y + (0.0008 if y < C_L[1] else -0.0008), h + 0.00005),
                         (x1 - BACK, 0.0016, 0.0002), END_GRAIN))
    for k, dx in enumerate((0.03, 0.065, 0.10)):  # faint grain lines along the board
        m.append(kit.box((BACK + dx, C_L[1] + 0.01 * (k - 1), h + 0.00005), (0.0012, 2 * HALF - 0.03, 0.0001),
                         END_GRAIN))
    # colliders: the body behind the curve, then the curve as 1 cm chord boxes turned to follow it (smooth on the
    # bearing: each chord is within 0.04 mm of the true curve)
    x_in = C_L[0] + R * math.cos(ALPHA)  # where the curve meets the ends
    cols = [kit.Collider(((BACK + x_in) / 2, C_L[1], 0.0), size=(x_in - BACK, 2 * HALF, T))]
    n = int(math.ceil(ARC / 0.010))
    for k in range(n):
        a0, a1 = -ALPHA + 2 * ALPHA * k / n, -ALPHA + 2 * ALPHA * (k + 1) / n
        p0, p1 = (C_L + R * np.array([math.cos(a), math.sin(a)]) for a in (a0, a1))
        am, chord = (a0 + a1) / 2, float(np.linalg.norm(p1 - p0))
        cen = (p0 + p1) / 2 - 0.015 * np.array([math.cos(am), math.sin(am)])  # 3 cm thick, inward from the chord
        cols.append(kit.Collider((cen[0], cen[1], 0.0), size=(0.030, chord, T), yaw=am))
    return pv.planar(PIVOT, pin=(0.0, 0.0, Z_MID), reach=((-0.08, 0.08), (-0.10, 0.32)), turn=(-0.6, 0.8),
                     mass=BOARD_MASS, meshes=m, colliders=tuple(cols), start=START, slide_at=CENTRE)


def _table():
    """The router table's top (the base): laminate, with the insert plate around the bit drawn on it."""
    meshes = [kit.box((0.29 - PIN[0], 0.04 - PIN[1], 0.0003), (0.40, 0.62, 0.0006), LAMINATE)]
    cols = [kit.Collider((0.29 - PIN[0], 0.04 - PIN[1], 0.0003), size=(0.40, 0.62, 0.0006))]
    return kit.Mechanism("router_table", pos=(float(PIN[0]), float(PIN[1]), TOP), size=(0.40, 0.62, 0.0006), meshes=tuple(meshes),
                         colliders=tuple(cols), links=_board_links())


def _bit():
    """The insert plate with the bit and the starting pin (fixed; its own prop, so the board touches them)."""
    px, py = PIN - BIT
    meshes = [kit.box((0.0, 0.0, 0.0008), (0.235, 0.300, 0.0010), PLATE),  # a 9-1/4 x 11-3/4 in phenolic plate
              kit.cylinder((0.0, 0.0, 0.00135), 0.024, 0.0001, DARK),  # the insert ring's opening round the bit
              kit.cylinder((0.0, 0.0, CUT_H / 2), RC, CUT_H, CARBIDE, n=40),  # the cutter's swept cylinder
              kit.cylinder((0.0, 0.0, CUT_H + 0.0001), 0.0065, 0.0002, STEEL),
              kit.cylinder((0.0, 0.0, sum(BEAR_Z) / 2), RB, BEAR_Z[1] - BEAR_Z[0], STEEL),  # the bearing
              kit.cylinder((0.0, 0.0, BEAR_Z[1] + 0.0008), 0.0035, 0.0016, DARK),  # its retaining screw
              kit.cylinder((px, py, PIN_H / 2), RP, PIN_H, STEEL, n=20),  # the starting pin
              kit.cylinder((px, py, 0.0014), 0.0060, 0.0002, DARK)]  # its threaded hole's rim
    cols = [kit.Collider((0.0, 0.0, sum(BEAR_Z) / 2), radius=RB, height=BEAR_Z[1] - BEAR_Z[0], slick=True),  # it rolls
            kit.Collider((px, py, PIN_H / 2), radius=RP, height=PIN_H)]
    return kit.Prop("router_bit", pos=(float(BIT[0]), float(BIT[1]), TOP), size=(0.235, 0.300, PIN_H), meshes=tuple(meshes),
                    colliders=tuple(cols))


OBJECTS = [_table(), _bit()]
KEY = "router_table/board"


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.grab = False  # irreversible: the board met the cutter while not braced on the pin and turning about it
        self.hands = False  # irreversible: a tool point came within 3 in of the bit's axis with the pads low
        self.cutting = False  # the board is in the cutter after an entry made braced on the pin
        self.braced_prev = False  # braced at the last step
        self.started = False  # the curve reached the bearing in a cut begun on the pin
        self.angle = None  # the bearing's contact angle on the curve at the last step (None: not on the bearing)
        self.routed = np.zeros(N_BINS, bool)  # the curve's 2 mm stretches routed to full width
        self.home_steps = 0  # consecutive steps with both arms home and still
        self.home = None  # each gripper's tool point at the trial's first scored step
        self.last = None
        self.still = False


_H = _History()


def _joints(objects):
    return {n: kit.joint(objects, f"router_table/{n}").pos for n in ("board_turn", "board_x", "board")}


def _bin(ang):
    return min(max(int((ang + ALPHA) * R / BIN), 0), N_BINS - 1)


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    J = _joints(objects)
    now_braced = braced(J)
    in_cutter = bit_dist(J) < RC
    if in_cutter and not _H.cutting:
        if now_braced and _H.braced_prev:
            _H.cutting = True  # entered braced on the pin: the only motion there is turning about it
        else:
            _H.grab = True  # the cutter's first bite on an unbraced board: it grabs and throws it
    if not in_cutter:
        _H.cutting = False  # out of the cutter: a new entry needs the pin again
    _H.braced_prev = now_braced
    c = curve_point(to_board(J, BIT))
    ang = c[1] if (c is not None and c[0] <= RB + ON_BEARING and _H.cutting) else None
    if ang is not None:
        _H.started = True
        # routed: the curve's stretches the bearing passed over feeding right to left, the contact moving from the
        # leading end toward the trailing end (the angle falling); feeding the other way (a climb cut) routes nothing
        if _H.angle is not None and ang < _H.angle:
            _H.routed[_bin(ang):_bin(_H.angle) + 1] = True
        elif _H.angle is None:
            _H.routed[_bin(ang)] = True
    _H.angle = ang
    now = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    for g in now:  # hands near the bit
        low = float(np.asarray(objects[g].low)[2]) - TOP < HANDS_LOW
        if low and np.linalg.norm(now[g][:2] - BIT) < HANDS:
            _H.hands = True
    if _H.home is None and len(now) == 2:
        _H.home = now
    _H.still = _H.last is not None and all(g in _H.last and np.linalg.norm(now[g] - _H.last[g]) < HOME_STILL
                                           for g in now)
    _H.last = now
    home = _H.home is not None and _H.still and all(g in now and np.linalg.norm(now[g] - _H.home[g]) < HOME_TOL
                                                   for g in GRIPPERS)
    _H.home_steps = _H.home_steps + 1 if home else 0
    _draw(J)
    if os.environ.get("ROUTER_TRACE"):  # a per-step trace for debugging sim runs (off in normal runs)
        with open(os.environ["ROUTER_TRACE"], "a") as f:
            f.write(" ".join(f"{k}={v:.4f}" for k, v in J.items()) +
                    f" gap={pin_gap(J) * 1000:.1f} bit={bit_dist(J) * 1000:.1f} braced={now_braced}"
                    f" cut={_H.cutting} grab={_H.grab} hands={_H.hands} routed={routed_cm():.1f}"
                    f" turned={pv.turned_deg(objects, KEY):.1f} slip={pv.max_slip_mm(objects, KEY):.2f}"
                    f" score={_score(objects)} L={np.round(now.get('left_gripper', np.zeros(3)), 3)}"
                    f" R={np.round(now.get('right_gripper', np.zeros(3)), 3)}\n")


def routed_cm():
    return 100.0 * BIN * int(_H.routed.sum())


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still there for
    the last 1 s (so arms that only pass through home as the run hits its time limit don't count)."""
    return _H.home_steps >= HOME_STEPS


def _released(objects):
    """No gripper's pad tips over the board (grown 1 cm) within 1.5 cm above its top: nobody still holding it."""
    J = _joints(objects)
    for g in GRIPPERS:
        if g not in objects:
            continue
        p = to_board(J, np.asarray(objects[g].pos, float)[:2])
        tip_z = float(np.asarray(objects[g].low)[2]) - TOP
        if outline_dist(p) < 0.01 and tip_z < GAP + T + PRESS:
            return False
    return True


def _past(J):
    """The board clear of the cutter by 5 mm and pushed past the bit: its trailing end on the outfeed (left) side."""
    trail = [to_world(J, OUTLINE[0])[1], to_world(J, OUTLINE[-1])[1]]  # the trailing end's two corners
    return bit_dist(J) >= RC + CLEAR and min(trail) > BIT[1]


def _job(objects):
    J = _joints(objects)
    if _H.grab or _H.hands:
        return 0.0  # the bit grabbed the board, or a hand went near the bit
    if not _released(objects):
        return 0.0  # nothing counts while a hand is still on the board
    routed = BIN * int(_H.routed.sum())
    if routed >= REQUIRED:
        return 1.0 if _past(J) else 0.75
    if routed >= HALFWAY:
        return 0.5
    if _H.started:
        return 0.25  # braced on the pin and pivoted onto the bearing, but not followed round
    return 0.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """The curve routed over at least 18 cm, the board pushed past the bit and let go, both arms home and still for
    1 s. Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0, irreversibly, if the board ever met the cutter (any part of it within the cutter's 15.9 mm radius) other
    than by being turned into it about the starting pin while braced on it (within 1.5 mm), the curve not yet on the
    bearing: a board fed into the bit off the pin, or slid back into it after leaving it, is grabbed. 0,
    irreversibly, if a tool point came within 7.6 cm (3 in) of the bit's axis with the pads within 5 cm of the
    table. 0 while pad tips are within 1.5 cm above the board over it. Otherwise: routed length is the curve's
    stretch the bearing passed over (within 1.5 mm of it) in a cut begun on the pin, fed right to left. 1.0 with at
    least 18 cm routed, the board 5 mm clear of the cutter with its trailing end past the bit, and both arms home
    and still for 1 s (0.9 with them elsewhere); 0.75 with 18 cm routed but the board left on or before the bit; 0.5
    with 9 cm routed; 0.25 with the curve brought onto the bearing from the pin; 0 otherwise.
    Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1."""
    return _score(objects)


# --- The routed stretch, drawn as markers (visual only; scoring never reads it): a pale band of fresh-cut cherry
# along the board's top at the curve, as wide as the rabbet, wherever the bearing has routed it.
_VIS = {"markers": None, "failed": False}


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        mat = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in FRESH.color), roughness=0.6)
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/World/Visuals/routed_edge",
            markers={"cut": sim_utils.CuboidCfg(size=(RC - RB, BIN * 1.15, 0.0004), visual_material=mat)}))
    except Exception as e:  # visuals are optional
        print(f"router: no routed-edge visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _draw(J):
    if "isaaclab" not in sys.modules:
        return
    m = _markers()
    if m is None:
        return
    th = J["board_turn"]
    tr, qs = [], []
    for k in range(N_BINS):
        if not _H.routed[k]:
            tr.append((0.0, 0.0, -10.0))
            qs.append((0.0, 0.0, 0.0, 1.0))
            continue
        a = -ALPHA + (k + 0.5) * BIN / R
        p = to_world(J, C_L + (R - (RC - RB) / 2) * np.array([math.cos(a), math.sin(a)]))
        yaw = a + th
        tr.append((p[0], p[1], TOP + GAP + T + 0.0003))
        qs.append((0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
    m.visualize(translations=np.array(tr, float), orientations=np.array(qs, float),
                marker_indices=np.zeros(N_BINS, int))


# --- The scripted demo (left arm; the right arm stays home).
TIP = PAD_TIP_PAST_TCP  # the tool point is this far above the pad tips
PZ = TOP + GAP + T + TIP - 0.004  # the pads 4 mm into the board's top: pressing on it


def _board_world(turn, slides, p):
    return PIN + _rot(turn) @ (np.asarray(p, float) + np.asarray(slides, float))


def _press_at(turn, slides, z=PZ):
    w = _board_world(turn, slides, PRESS_PT)
    return (float(w[0]), float(w[1]), z)


# the pads press on the board's middle along its length, 1.2 cm behind its centre (8.7 cm in from the top of the
# curve): a push there barely turns the board (a board pushed at one point swings until its centre trails behind)
PRESS_PT = np.array([CENTRE[0] - 0.012, CENTRE[1]])


def _feed_slides(phi, into=0.0005):
    """The slides (turn 0) that put the curve's contact with the bearing at angle `phi` on the curve, `into` m into
    the bearing. Kept small: the bearing's push on the curve, behind the pads, would turn the board about them."""
    c_world = BIT - (R + RB - into) * np.array([math.cos(phi), math.sin(phi)])
    return c_world - PIN - C_L


def demo():
    """Left arm: press the pads on the board and slide it 2.9 cm until its curve is against the starting pin, swing it
    15 deg clockwise about the pin until the curve rides the bearing, then feed it right to left round the bearing,
    keeping the curve on it, to the trailing end and on past the bit; lift off and go home."""
    C, O = FINGER_CLOSED, FINGER_OPEN
    s0 = np.array(START[:2])
    th0 = START[2]
    braced_s = np.zeros(2)
    into_pin = _rot(-th0) @ (np.array([0.0015, 0.0025]))  # 3 mm on toward the pin (its collider stops the board)
    left = [Waypoint(2.2, _press_at(th0, s0, TOP + 0.10), C, "over the board"),
            Waypoint(1.2, _press_at(th0, s0, PZ + 0.01), C, "down onto it"),
            Waypoint(0.6, _press_at(th0, s0), C, "press")]
    n = 6
    for k in range(1, n + 1):  # slide it onto the pin
        s = s0 + (braced_s + into_pin - s0) * k / n
        left.append(Waypoint(0.5, _press_at(th0, s), C, "brace the board against the starting pin"))
    left.append(Waypoint(0.5, _press_at(th0, braced_s + into_pin), C, "settle on the pin"))
    n = 10
    th1 = -math.radians(1.0)  # a degree past the braced pose: into the bearing, which stops it
    for k in range(1, n + 1):  # pivot about the pin
        th = th0 + (th1 - th0) * k / n
        left.append(Waypoint(0.6, _press_at(th, braced_s), C, "pivot about the pin into the bit"))
    # feed: the contact runs from where the pivot put it down the curve to past the trailing end
    phi0 = math.atan2(*(BIT - PIN - C_L)[::-1])
    phis = np.linspace(phi0, -ALPHA - math.radians(4.0), 22)
    for phi in phis:
        left.append(Waypoint(0.7, _press_at(0.0, _feed_slides(phi)), C, "feed round the bearing"))
    s_end = _feed_slides(phis[-1], into=-0.004) + np.array([0.0, 0.030])
    left += [Waypoint(1.0, _press_at(0.0, s_end), C, "on past the bit"),
             Waypoint(0.8, _press_at(0.0, s_end, TOP + 0.10), C, "lift off"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    right = [kit.Home(1.0, label="stay home")]
    return {"left": left, "right": right}
