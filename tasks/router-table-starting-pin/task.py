"""A router table with a bearing-guided rabbeting bit and a steel starting pin in its insert plate, and a cherry board
(25 x 15 cm, 1.27 cm thick) with one long edge band-sawn to a gentle convex curve, lying on the table top on the
infeed (right) side of the bit, its curved edge toward the bit.

To rout a curved edge on a router table, where there is no fence to steady the work, a woodworker first braces the
board against the starting pin, a steel pin in the insert plate a few centimeters from the bit, then pivots it about
the pin into the spinning bit until its edge rides the bit's bearing. Only then does the board come off the pin and
follow the bearing round, fed right to left against the bit's rotation. Fed straight into the bit without the pin,
the cutter's first bite on an unsupported edge can grab the board and throw it, or pull the hands toward the cutter.
"""

import dataclasses
import functools
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled table, bit and board; nothing on disk changes)
import guardkit as gk  # noqa: E402  (after yamkit)
import yamgeom as yg  # noqa: E402  (the arm's collision hulls, read from the environment at run time)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, ObjectState  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = ("Rout the rabbet along the board's curved edge on the router table, then leave the board on the "
               "table past the bit.")

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
# The robot's table is the router table's top. The bit turns counterclockwise seen from above (a router hangs
# upside down under the table), so its front (robot-side) cutting edge moves to the right: work is fed right to left
# (+y), against it, and the starting pin sits on the infeed side, to the bit's right.
BIT = np.array([0.330, 0.060])  # the bit's axis
RB = 0.00635  # m: the bearing's radius (a 0.5 in ball bearing on the bit's tip)
RC = 0.0159  # m: the cutter's radius (a 1.25 in rabbeting bit), painted round the bearing as the cutter circle: the
#               rabbet it stands for is RC - RB = 9.5 mm (3/8 in) wide, 1/8 in high (one pass)
BEAR_Z = (0.0034, 0.0082)  # m: the bearing's bottom and top above the table (3/16 in thick, the bit's bearing's place)
SCREW_R = 0.0048  # m: the shoulder screw's 3/8 in head and the nylon spacers (3/8 in across) above and below the bearing
SCREW_TOP = 0.0193  # m: the screw head's top: a 1/4 in spacer over the bearing, then the 3/16 in tall head
PIN = np.array([BIT[0] + 0.0075, BIT[1] - 0.050])  # the starting pin: 5 cm to the bit's right, in the insert plate
RP = 0.0032  # m: the pin's radius (a 0.25 in steel pin)
PIN_H = 0.030  # m: it stands 3 cm above the table

R = 0.300  # m: the curved edge's radius (a gentle arc: 2.7 cm of bow over the board's 25 cm)
HALF = 0.125  # m: half the board's length (25 cm)
DEPTH = 0.150  # m: the board's depth across its middle, back edge to the top of the curve
T = 0.0127  # m: the board's thickness (0.5 in cherry)
GAP = 0.001  # m: the table top's friction surface (below) stands 1 mm proud of the drawn laminate; the board lies on it
BOARD_MASS = 0.25  # kg: 25 x 15 x 1.27 cm of cherry (0.561 g/cm3) with its 2.7 cm bow: 353 cm2 x 1.27 cm = 448 cm3
ALPHA = math.asin(HALF / R)  # the half-angle the curve spans about its center (24.6 deg)
ARC = 2 * ALPHA * R  # m: the curve's length, 25.8 cm
Z_MID = GAP + T / 2  # the board's mid-thickness above the table (its frame's height)
G = 9.81
# The board's friction on the table's top, a planed wooden slab in the sim as on the rig (README): wood on wood. The
# board is a free rigid body lying on the table, so its weight and the hands' press bear on the table top through a
# contact, and PhysX's Coulomb friction there resists its sliding and turning: mu 0.25 sliding, 0.3 at rest, the low
# end of clean, dry wood on wood (0.25-0.5, Engineering ToolBox, "Friction - friction coefficients"), for smooth
# planed faces. The rig's tilt test checks it (a test board starts sliding at 14-17 deg; paste wax if steeper).
MU_TABLE = 0.25  # sliding
MU_TABLE_STATIC = 0.30


def _circle_meet(c0, r0, c1, r1, pick):
    """The intersection of two circles (centers c0, c1, radii r0, r1); `pick` chooses between the two."""
    d = np.linalg.norm(c1 - c0)
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    mid = c0 + a * (c1 - c0) / d
    perp = np.array([-(c1 - c0)[1], (c1 - c0)[0]]) / d
    return min((mid + h * perp, mid - h * perp), key=pick)


# The board's frame: its origin is the pin's center with the board in its "braced" pose, where the curve
# touches both the pin and the bearing, its long sides along y. The curve's center in that frame:
C_L = _circle_meet(PIN, R + RP, BIT, R + RB, pick=lambda c: c[0]) - PIN
BACK = C_L[0] + R - DEPTH  # the back edge's x in the board's frame
ENDS = (C_L[1] - HALF, C_L[1] + HALF)  # the trailing (right, -y) and leading (left, +y) ends' y
_N_ARC = 64
OUTLINE = np.array([C_L + R * np.array([math.cos(a), math.sin(a)]) for a in np.linspace(-ALPHA, ALPHA, _N_ARC)]
                   + [(BACK, ENDS[1]), (BACK, ENDS[0])])  # convex, counterclockwise: the curve, then the back corners


def pose(J):
    """The board's turn (rad) and slides from its pose (_joints): a point p of the board's frame is at
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
    center; +ALPHA is the leading end) of the curve's nearest point; None if the nearest point is past an end."""
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
# Contact is contact: the board touches the pin, or its curve the bearing, when the gap between them is at most
# TOUCH. PhysX rests the board pressed 0.03-0.06 mm into a collider (the demo's run), so 0.2 mm is the solver's
# tolerance, not an air gap; a board hovering 0.3 mm or more away is not touching. And it is not pressed more than
# DEEP into it: a flat board can't be, so a deeper reading is a board lifted over the pin or the bearing.
TOUCH = 0.0002  # m
DEEP = 0.001  # m
BRACE = TOUCH  # the board on the pin: its edge within TOUCH of the pin's surface
ON_BEARING = TOUCH  # the curve on the bearing: within TOUCH of the bearing's surface
FLAT_UP = 0.001  # m: the board lies flat on the table: its middle at most 1 mm above where it rests,
FLAT_TILT = 2.0  # deg: and tipped at most 2 deg (a board lifted or tipped over the bearing or the pin traces nothing)
PIVOT_TURN = math.radians(0.01)  # rad per control step: the board turning about the pin as it enters the cutter
PIVOT_SLIP = 0.0005  # m per control step: the pin's place on the board moving at most this much meanwhile (it pivots
#                      about the pin, not slides along or past it)
CLEAR = 0.005  # m: at the end, no part of the board within 5 mm of the cutter circle
BIN = 0.002  # m: the traced length is kept as 2 mm stretches of the curve (129 of them)
N_BINS = int(round(ARC / BIN))
WHOLE = N_BINS - 2  # full credit: all but two of the curve's 129 2 mm stretches (4 mm in all), for the corners at its
#                    ends; in one feed or in passes, each begun braced on the pin
MOST = 0.180  # m: 18 cm, 70% of the 25.8 cm curve: most of it, as from a start a few cm in from the leading end
HALFWAY = MOST / 2  # m: 9 cm: real progress round the bearing
HANDS = 0.076  # m: the three-inch rule of router-table manuals for hands near a bit: no part of either gripper (its
#                own shape: palm, finger bodies, pads, tips and carriages, boxes around its collision hulls, at their
#                present opening) may touch the zone within 3 in (7.6 cm) of the bit's axis, up to HANDS_LOW above the
#                table. The zone is the hazard itself, since a slip or a grab carries a hand that close into the cutter.
HANDS_LOW = 0.05  # m: the zone's height above the table top (the bit and the board's top are below 1.4 cm)
GRIPPERS = ("left_gripper", "right_gripper")


def on_bearing(J):
    """The curved edge touches the bearing: the curve's gap to the bearing's surface between -DEEP and TOUCH."""
    c = curve_point(to_board(J, BIT))
    return c is not None and RB - DEEP <= c[0] <= RB + ON_BEARING


def braced(J):
    """The pivot condition: the board touching the pin (its gap to the pin's surface between -DEEP and TOUCH) and its
    curve not yet on the bearing. Nothing holds it there but whatever pushes it onto the pin (the pin is a plain
    collider); once the curve rides the bearing it comes off the pin."""
    return -DEEP <= pin_gap(J) <= BRACE and not on_bearing(J)


CENTRE = (float(C_L[0] + R - DEPTH / 2), float(C_L[1]))  # the board's middle in its frame: the board body's origin


def _centroid(poly):
    """Area and centroid of a convex polygon (counterclockwise points)."""
    x, y = poly[:, 0], poly[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cr = x * y1 - x1 * y
    a = cr.sum() / 2
    return a, np.array([((x + x1) * cr).sum(), ((y + y1) * cr).sum()]) / (6 * a)


COM = _centroid(OUTLINE)[1] - np.array(CENTRE)  # the board's center of mass from its middle (toward the curve's bow)


# --- The resistance: a one-way hanging-weight drag, the same in the sim as in the real trials (README, Resistance).
# Nothing at the bit cuts or turns: it is a smooth ball bearing on a fixed shoulder screw (the bit's own bearing's size
# and place), and the cutter's circle round it is only painted. The cut's resistance comes from a cord tied to a screw
# eye in the board's trailing end, which runs to a pulley off the table's right end, through a cam cleat, and down to
# a hanging weight. While the board moves away from the pulley (fed right to left) it draws cord and lifts the weight,
# a steady CORD_N pull at the eye toward the pulley; held there by the hands, it still carries the weight. Let go or
# drawn back, the board lets the cord run back until the cleat's cams grip (CATCH), and the cord goes slack: no pull, so a
# board left at rest is free. The cord is set snug with the board braced on the pin and its curve's leading end on the
# bearing, where a whole-curve feed starts, so the drag takes hold as the board is fed along the bearing; after a
# run-back it takes hold again only where the board draws out past where the cams gripped (the cleat keeps what was
# drawn, less the run-back the cams allow before they grip).
# Where 3 N comes from: cutting_force.py (Curti et al. 2021, Eur. J. Wood Prod. 79:667-678,
# doi:10.1007/s00107-021-01667-5, Eq. 4 and Table 5): the pass's mean cutting force against the feed (cherry; a 1.25 in
# two-flute rabbeting bit at 18,000 rpm; 9.5 mm wide, 1/8 in high) is 3.0-3.1 N at the demo's 0.3 m/min feed.
CORD_N = 3.0  # N: the cord's pull at the board: a 0.300 kg weight (2.94 N) trimmed with the pulley's and cleat's drag
PULLEY = np.array([0.360, -0.660])  # m: the pulley's sheave, clamped at the bench's right edge (y -0.65), its top
#                                     level with the eye (so the cord runs flat)
_X_IN = float(C_L[0] + R * math.cos(ALPHA))  # where the curve meets the ends (board frame x)
EYE = np.array([_X_IN - 0.010, ENDS[0]])  # the screw eye: in the trailing end face, at mid-thickness, 1 cm in from
#                                            the curve (board frame)
CATCH = 0.003  # m: the run-back the rig's cleat check allows before the cams grip (README build); the sim takes all of
#                it
_LEAD_TURN = math.atan2(*(BIT - PIN - C_L)[::-1]) - ALPHA  # braced on pin and bearing, the bearing at the leading end
SNUG = float(np.linalg.norm(PIN + _rot(_LEAD_TURN) @ (EYE + _rot(-_LEAD_TURN) @ C_L - C_L) - PULLEY))  # cord set


def cord_length(J):
    """The cord's run (m) from the board's screw eye to the pulley."""
    return float(np.linalg.norm(to_world(J, EYE) - PULLEY))


class _Drag:
    """Per trial: the cord drawn out so far (the cleat holds it), the pull now and where it acts."""

    def __init__(self):
        self.episode = kit.episode()
        self.drawn = SNUG  # the most cord drawn since the cleat last seated, or where it is seated
        self.seated = True  # the cleat's cams hold the cord (set snug at SNUG to start)
        self.key = None
        self.force = np.zeros(2)  # N, world (x, y)
        self.at = PULLEY.copy()  # where it acts (world): the screw eye
        self.load = (0.0, 0.0, 0.0)


_D = _Drag()


def drag_load(J):
    """The cord's pull on the board, as (its moment about the pin, its components along the board's x and y), and its
    bookkeeping; _push_board puts it on the board's body. CORD_N toward the pulley at the eye while the cord is taut,
    else 0: taut while the board draws cord out, and while it lets the cord back up to CATCH from the most drawn (the
    cams slipping); then the cams grip there and the cord is slack until the board draws out past that again. It
    computes once per control step (the same pose returns the same load)."""
    global _D
    if _D.episode != kit.episode():
        _D = _Drag()
    key = (round(J["board_turn"], 9), round(J["board_x"], 9), round(J["board"], 9))
    if key == _D.key:
        return _D.load
    _D.key = key
    eye = to_world(J, EYE)
    run = float(np.linalg.norm(eye - PULLEY))
    if run >= _D.drawn:  # drawing cord out through the cleat: the weight rises
        _D.drawn, _D.seated = run, False
        taut = True
    elif not _D.seated and run >= _D.drawn - CATCH:  # let back: the weight sinks, the cord slips back through the cams
        taut = True
    else:  # the cams have gripped (at most CATCH back from the most drawn): the cleat holds the weight
        if not _D.seated:
            _D.drawn, _D.seated = _D.drawn - CATCH, True
        taut = run >= _D.drawn
    _D.force = CORD_N * (PULLEY - eye) / run if taut else np.zeros(2)
    _D.at = eye
    f, th = _D.force, J["board_turn"]
    tau = float((eye - PIN)[0] * f[1] - (eye - PIN)[1] * f[0])
    ex, ey = _rot(th) @ np.array([1.0, 0.0]), _rot(th) @ np.array([0.0, 1.0])
    _D.load = (tau, float(f @ ex), float(f @ ey))
    return _D.load


def drag_force_now():
    """The cord's pull on the board at the last step, (x, y) in N (world), for logs and tests."""
    return _D.force.copy()


# Where the demo braces the board on the pin: turned 14 deg counterclockwise from square (its leading end swung back
# toward the robot), at the place along its curve where a pivot about the pin brings the bearing onto the curve a few
# mm in from its leading end (the woodworker's margin: braced any nearer the end, the bearing meets end grain). The demo
# finds this pose itself; the task starts the board square (START, below).
BRACE_TURN = math.radians(14.0)
_PHI0 = math.atan2(*(BIT - PIN - C_L)[::-1])
_BRACED_END = _rot(-(_PHI0 - ALPHA)) @ C_L - C_L  # braced where the pivot brings the curve's very end to the bearing
_ALONG = np.array([-0.0022, 0.0083]) / np.linalg.norm([-0.0022, 0.0083])  # along the pin, away from the curve's end
BRACE_AT = _BRACED_END + 0.001 * _ALONG  # the slides of the braced pose the demo makes (1 mm along)
PIN_OUT = (-BRACE_AT - C_L) / np.linalg.norm(-BRACE_AT - C_L)  # from the curve's center to the pin there (board frame)
# The board starts as a woodworker sets it down: flat on the table on the infeed (right) side of the bit, square to
# the table (its long sides along y, the curve toward the bit), 2.2 cm from the pin and 3.4 cm from the bit's axis.
START = (-0.015, -0.040, 0.0)  # (x slide, y slide, turn): 1.5 cm nearer the robot, 4 cm right of the pose at turn 0
#                                where the curve touches both pin and bearing
WOOD = kit.Look((0.56, 0.31, 0.19), 0.6)  # cherry, planed
END_GRAIN = kit.Look((0.44, 0.23, 0.14), 0.7)
SAWN = kit.Look((0.47, 0.26, 0.16), 0.8)  # the band-sawn curve, a little darker and rougher
TRACE = kit.Look((0.10, 0.30, 0.85), 0.5)  # the blue spotting paste the bearing leaves on the edge, drawn by markers
LAMINATE = kit.Look((0.80, 0.81, 0.79), 0.4)
PLATE = kit.Look((0.12, 0.12, 0.13), 0.4)  # phenolic insert plate
STEEL = kit.Look((0.78, 0.79, 0.80), 0.25, metallic=0.8)
NYLON = kit.Look((0.92, 0.91, 0.87), 0.5)  # the nylon spacers
PAINT = kit.Look((0.95, 0.95, 0.95), 0.6)  # the painted cutter circle and the drawn 3 in circle
CORD = kit.Look((0.85, 0.35, 0.10), 0.8)  # the drag's braided cord
PINE = kit.Look((0.80, 0.66, 0.45), 0.7)  # the 2x4 offcut the pulley and the cleat are screwed to
WASHERS = kit.Look((0.62, 0.63, 0.66), 0.4)  # the bag of steel washers (the hanging weight)
FLAG = kit.Look((0.98, 0.78, 0.05), 0.6)  # the bright tape flag on the cord above the weight
# The drag's rig as it is built (README, Real-world build), drawn for the cameras only: the drag law uses PULLEY alone.
# The cord runs level from the screw eye through the cam cleat to the pulley, clamped on a 2x4 offcut at the bench's
# right edge, over it and straight down to the hanging weight, a bag of washers, with a tape flag on the cord just
# above it. The weight hangs free below the bench's edge and rises as the board draws cord out.
SHEAVE_R = 0.019  # m: the pulley's 1-1/2 in sheave
WEIGHT_DROP = 0.36  # m: the bag's top below the table top with the cord set snug
BAG = (0.08, 0.04, 0.10)  # m: the bag of washers
_IN = np.array([0.06, -0.66]) / np.linalg.norm([0.06, -0.66])  # the cord's run onto the pulley (from the board)
DARK = kit.Look((0.20, 0.20, 0.21), 0.5)
POLYCARB = kit.Look((0.70, 0.84, 0.93), 0.05)  # the clear polycarbonate shield's edge (its face is clear: not drawn)
ALUMINUM = kit.Look((0.74, 0.75, 0.77), 0.35, metallic=0.8)  # its anodized aluminum body
# The plate's overhead pin guard, after the Bench Dog ProGuard (Bench Dog Tools catalog, 2008, p. 16: a polycarbonate
# shield on an aluminum body bolted to the plate, "1-1/2\" stock clearance under the guard", usable as a starting pin's
# guard): a clear 10 cm shield centered over the bit, its underside 1-1/2 in (38 mm) above the table, held by an
# aluminum arm from a post bolted to the plate's far side (away from the robot). The board passes under it; the 3 in
# rule already keeps the hands' low parts out from under it.
GUARD_R = 0.051  # m: the shield's radius (a 4 in disc; its height is the maker's)
GUARD_Z = (0.0381, 0.0445)  # m: the shield's underside and top above the table (1/4 in polycarbonate)
GUARD_POST_X = 0.100  # m: the post's center, from the bit's axis, on the far side (+x)


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


def _board():
    """The board: a free rigid body lying on the table. Drawn in its frame (origin at the pin's center in the braced
    pose, z at mid-board), then moved so the body's own origin is the board's middle (CENTRE)."""
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
    g = np.array([CENTRE[0], CENTRE[1], 0.0])
    m = [kit._mesh(np.asarray(k.points, float) - g, k.counts, k.indices, k.look) for k in m]
    cols = [dataclasses.replace(c, center=tuple(float(v) for v in np.asarray(c.center, float) - g)) for c in cols]
    th, sl = START[2], np.array(START[:2])
    at = PIN + _rot(th) @ (np.array(CENTRE) + sl)
    # its mass, center of mass and inertia from its own shape (the colliders overlap, so PhysX's would be off)
    lx, ly = DEPTH, 2 * HALF
    inertia = (BOARD_MASS * (ly ** 2 + T ** 2) / 12, BOARD_MASS * (lx ** 2 + T ** 2) / 12,
               BOARD_MASS * (lx ** 2 + ly ** 2) / 12)
    return kit.Prop("board", pos=(float(at[0]), float(at[1]), TOP + GAP + T / 2), yaw=float(th),
                    size=(DEPTH, 2 * HALF, T), mass=BOARD_MASS, meshes=tuple(m), colliders=tuple(cols),
                    com=(float(COM[0]), float(COM[1]), 0.0), inertia=inertia)


def _table():
    """The router table's top (drawn as a router table's laminate, with the insert plate around the bit; the board
    slides on a wood-on-wood surface, GAP above)."""
    meshes = [kit.box((0.29 - PIN[0], 0.04 - PIN[1], 0.0003), (0.40, 0.62, 0.0006), LAMINATE)]
    cols = [kit.Collider((0.29 - PIN[0], 0.04 - PIN[1], 0.0003), size=(0.40, 0.62, 0.0006))]
    return kit.Prop("router_table", pos=(float(PIN[0]), float(PIN[1]), TOP), size=(0.40, 0.62, 0.0006),
                    meshes=tuple(meshes), colliders=tuple(cols))


def _ring(r, w, z, look):
    """A flat painted line round the bit's axis: radius r (its outer edge), w wide, its top at z."""
    return [_prism([(rr * math.cos(a), rr * math.sin(a)) for rr, a in ((r - w, a0), (r, a0), (r, a0 + math.pi / 36),
                                                                      (r - w, a0 + math.pi / 36))],
                   z - 0.0001, z, look) for a0 in (k * math.pi / 36 for k in range(72))]


def _drag_rig():
    """The drag's pulley, cleat, offcut and clamp at the bench's edge (visual only), in the bit prop's frame."""
    p = np.array([PULLEY[0] - BIT[0], PULLEY[1] - BIT[1], 0.0])
    into = np.array([_IN[0], _IN[1], 0.0])
    axle = np.array([-_IN[1], _IN[0], 0.0])  # the pulley's axle, level and across the cord
    block = p - 0.05 * into + 0.045 * axle  # the 2x4 offcut beside the cord's run, just inside the edge
    return [kit.cylinder(p + (0.0, 0.0, Z_MID - SHEAVE_R), SHEAVE_R, 0.008, STEEL, axis=axle),  # the pulley's sheave
            kit.cylinder(p + (0.0, 0.0, Z_MID - SHEAVE_R) + 0.012 * axle, 0.003, 0.03, DARK, axis=axle),  # its axle
            kit.box(block + (0.0, 0.0, 0.019), (0.038, 0.089, 0.038), PINE),  # the 2x4 offcut on the bench
            kit.box(p - 0.05 * into + (0.0, 0.0, Z_MID), (0.012, 0.030, 0.012), DARK),  # the cam cleat on it
            kit.box((block[0], -0.645 - BIT[1], 0.0), (0.030, 0.012, 0.11), DARK)]  # the C-clamp holding it at the edge


def _bit():
    """The insert plate with the bit's stand-in and the starting pin (fixed; its own prop, so the board touches them).
    No cutter: at the bit's axis a ball bearing of the bit's bearing's size turns on a fixed shoulder screw, between
    nylon spacers, and the cutter's circle is painted round it (README, Real-world build)."""
    px, py = PIN - BIT
    spacer = BEAR_Z[0] - 0.0001
    meshes = [kit.box((0.0, 0.0, 0.0008), (0.235, 0.300, 0.0010), PLATE),  # the plate's outline, 9.25 x 11.75 in
              *_ring(RC, 0.0015, 0.0014, PAINT),  # the cutter circle, painted
              *_ring(HANDS, 0.0010, 0.0014, PAINT),  # the 3 in circle, drawn
              kit.cylinder((0.0, 0.0, spacer / 2 + 0.0001), SCREW_R, spacer, NYLON),  # 1/8 in spacer under it
              kit.cylinder((0.0, 0.0, sum(BEAR_Z) / 2), RB, BEAR_Z[1] - BEAR_Z[0], STEEL),  # the bearing
              kit.cylinder((0.0, 0.0, (BEAR_Z[1] + SCREW_TOP - 0.0048) / 2), SCREW_R, SCREW_TOP - 0.0048 - BEAR_Z[1],
                           NYLON),  # 1/4 in spacer over it
              kit.cylinder((0.0, 0.0, SCREW_TOP - 0.0024), SCREW_R, 0.0048, DARK),  # the shoulder screw's head
              *_drag_rig(),
              kit.cylinder((px, py, PIN_H / 2), RP, PIN_H, STEEL, n=20),  # the starting pin
              kit.cylinder((px, py, 0.0014), 0.0060, 0.0002, DARK),  # its threaded hole's rim
              # the overhead guard: shield, arm and post (see GUARD_R)
              # the shield is clear, so only its 3 mm edge is drawn (the renderer here draws no see-through surface);
              # its collider is the whole disc
              *[_prism([(r * math.cos(a), r * math.sin(a)) for r, a in ((GUARD_R - 0.003, a0), (GUARD_R, a0),
                                                                       (GUARD_R, a0 + math.pi / 24),
                                                                       (GUARD_R - 0.003, a0 + math.pi / 24))],
                       GUARD_Z[0], GUARD_Z[1], POLYCARB) for a0 in (k * math.pi / 24 for k in range(48))],
              kit.box(((GUARD_R - 0.02 + GUARD_POST_X) / 2, 0.0, GUARD_Z[1] + 0.004),
                      (GUARD_POST_X - GUARD_R + 0.02, 0.030, 0.008), ALUMINUM),
              kit.box((GUARD_POST_X, 0.0, (GUARD_Z[1] + 0.008) / 2), (0.025, 0.040, GUARD_Z[1] + 0.008), ALUMINUM)]
    tx, ty = 0.29 - BIT[0], 0.04 - BIT[1]  # the table top's middle, from the bit
    cols = [kit.Collider((0.0, 0.0, sum(BEAR_Z) / 2), radius=RB, height=BEAR_Z[1] - BEAR_Z[0], slick=True),  # it rolls
            kit.Collider((0.0, 0.0, (BEAR_Z[1] + SCREW_TOP) / 2), radius=SCREW_R, height=SCREW_TOP - BEAR_Z[1]),
            kit.Collider((px, py, PIN_H / 2), radius=RP, height=PIN_H),
            # the guard is solid: a thrown board or a hand meets it
            kit.Collider((0.0, 0.0, sum(GUARD_Z) / 2), radius=GUARD_R, height=GUARD_Z[1] - GUARD_Z[0]),
            kit.Collider(((GUARD_R - 0.02 + GUARD_POST_X) / 2, 0.0, GUARD_Z[1] + 0.004),
                         size=(GUARD_POST_X - GUARD_R + 0.02, 0.030, 0.008)),
            kit.Collider((GUARD_POST_X, 0.0, (GUARD_Z[1] + 0.008) / 2), size=(0.025, 0.040, GUARD_Z[1] + 0.008)),
            # the table top's surface under the board (its top at the board's underside, GAP above the drawn top):
            # planed hardwood on the planed wooden slab (wood on wood), MU_TABLE above
            kit.Collider((tx, ty, GAP / 2), size=(0.40, 0.62, GAP), friction=(MU_TABLE_STATIC, MU_TABLE))]
    return kit.Prop("router_bit", pos=(float(BIT[0]), float(BIT[1]), TOP), size=(0.235, 0.300, GUARD_Z[1] + 0.008), meshes=tuple(meshes),
                    colliders=tuple(cols))


OBJECTS = [_table(), _bit(), _board()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.grab = False  # irreversible: the board crossed the cutter circle while not braced on the pin and turning
        #                    about it
        self.hands = False  # irreversible: a gripper's shape, or a forearm or wrist link, touched the 3 in zone
        self.cord = False  # irreversible: a gripper or a forearm or wrist link touched the drag's cord
        self.entered = False  # the board is inside the cutter circle after an entry made braced on the pin
        self.braced_prev = False  # braced (and flat) at the last step
        self.prev = None  # the board's pose at the last step
        self.started = False  # the curve reached the bearing in a pass begun on the pin
        self.angle = None  # the bearing's contact angle on the curve at the last step (None: not on the bearing)
        self.traced = np.zeros(N_BINS, bool)  # the curve's 2 mm stretches the bearing has traced


_H = _History()


def _joints(objects):
    """The board's pose as (turn, slides) in the pin frame (see pose()), from the free body's position and heading."""
    b = objects["board"]
    x, y, z, w = (float(v) for v in b.quat_xyzw)
    th = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    sl = _rot(-th) @ (np.asarray(b.pos, float)[:2] - PIN) - np.array(CENTRE)
    return {"board_turn": th, "board_x": float(sl[0]), "board": float(sl[1])}


def board_rest(objects):
    """How far the board's middle is above where it lies flat (mm) and how far it is tipped (deg), for logs."""
    b = objects["board"]
    x, y, z, w = (float(v) for v in b.quat_xyzw)
    up = 1 - 2 * (x * x + y * y)  # the board's z axis's world z
    return 1000 * (float(b.pos[2]) - (TOP + GAP + T / 2)), math.degrees(math.acos(min(1.0, max(-1.0, up))))


def _push_board(base, objects):
    """Each control step (a yamkit step hook): the cord's pull (drag_load) put on the board's body, at its center of
    mass as a force and a turning moment about it from the screw eye, held until the next step."""
    if "board" not in objects:
        return
    import torch

    J = _joints(objects)
    drag_load(J)
    f, c = _D.force, _D.at
    th = J["board_turn"]
    com = np.asarray(objects["board"].pos, float)[:2] + _rot(th) @ COM
    tz = float((c - com)[0] * f[1] - (c - com)[1] * f[0])
    body = base.scene["board"]
    dev = base.scene.env_origins.device
    ids = torch.tensor([0], device=dev)
    comp = body.permanent_wrench_composer
    if not f.any():
        comp.reset(env_ids=ids)
        return
    comp.set_forces_and_torques_index(forces=torch.tensor([[[float(f[0]), float(f[1]), 0.0]]], device=dev),
                                      torques=torch.tensor([[[0.0, 0.0, tz]]], device=dev),
                                      body_ids=torch.tensor([0], device=dev), env_ids=ids, is_global=True)


kit.STEP_HOOKS[:] = [h for h in kit.STEP_HOOKS if getattr(h, "__name__", "") != "_push_board"] + [_push_board]


def _bin(ang):
    return min(max(int((ang + ALPHA) * R / BIN), 0), N_BINS - 1)


_DBG = {"n": 0}


def _cord_gap(objects, J):
    """ROUTER_DEBUG only: the smallest gap (m) between the cord's run and any gripper box or forearm or wrist box, and
    whose it is."""
    eye = to_world(J, EYE)
    seg = gk.Seg((float(eye[0]), float(eye[1]), TOP + Z_MID), (float(PULLEY[0]), float(PULLEY[1]), TOP + Z_MID),
                 CORD_R)
    objs = {**objects, "cord": _ORIGIN}
    co, ro = np.zeros(3), np.eye(3)
    best = (1.0, "")
    for g in GRIPPERS:
        if g not in objs:
            continue
        o, cg, rg, _, _ = gk._hand_frames(objs, g, "cord")
        for box in gk.hand_boxes(o):
            d = gk._box_gap(cg, rg, box, co, ro, seg)
            if d < best[0]:
                best = (d, g)
    for side in ("left", "right"):
        for ln, boxes in link_boxes().items():
            st = objects.get(f"{side}_{ln}")
            if st is None:
                continue
            cg, rg = np.asarray(st.pos, float), gk._rot(st.quat_xyzw)
            for box in boxes:
                d = gk._box_gap(cg, rg, box, co, ro, seg)
                if d < best[0]:
                    best = (d, f"{side}_{ln}")
    return best


def _dbg(J, ang, objects=None):
    """ROUTER_DEBUG=1: print the board's pose, the bearing contact, the traced length and the cord's pull every 20
    steps, and every step the tool points (to measure the hands' forces from the arms' give afterwards)."""
    if not os.environ.get("ROUTER_DEBUG"):
        return
    _DBG["n"] += 1
    if objects is not None:
        tp = [np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects]
        if len(tp) == 2:
            f = drag_force_now()
            sys.stderr.write(f"router tcp {_DBG['n']} " + " ".join(f"{v:.5f}" for v in (*tp[0], *tp[1])) +
                             f" turn {J['board_turn']:.5f} sl {J['board_x']:.5f} {J['board']:.5f} drag {f[0]:.3f} "
                             f"{f[1]:.3f}\n")
        if _DBG["n"] % 5 == 0:
            gap, who = _cord_gap(objects, J)
            sys.stderr.write(f"router cordgap {_DBG['n']} {gap * 1000:.1f} mm {who}\n")
    if _DBG["n"] % 20 == 0:
        f = drag_force_now()
        sys.stderr.write(f"router dbg {_DBG['n']}: turn {math.degrees(J['board_turn']):.1f} slides "
                         f"{J['board_x']*100:.2f} {J['board']*100:.2f} cm; pin gap {pin_gap(J)*1000:.1f} mm; bit dist "
                         f"{bit_dist(J)*1000:.1f} mm; on bearing {ang is not None}; traced {traced_cm():.1f} cm; drag "
                         f"({f[0]:.2f}, {f[1]:.2f}) N; grab {_H.grab} hands {_H.hands} cord {_H.cord}; up "
                         f"{_LIFT['z']:.2f} mm, tipped {_LIFT['tilt']:.2f} deg\n")
    sys.stderr.flush()


_LIFT = {"z": 0.0, "tilt": 0.0}


def _pivoting(a, b):
    """From pose a to pose b (one control step) the board turned about the pin: it turned at least PIVOT_TURN and the
    pin's place on the board moved at most PIVOT_SLIP (sliding along the pin or past it moves it)."""
    if a is None:
        return False
    slip = float(np.linalg.norm(to_board(b, PIN) - to_board(a, PIN)))
    return abs(b["board_turn"] - a["board_turn"]) >= PIVOT_TURN and slip <= PIVOT_SLIP


def _update(objects):
    global _H
    try:
        _LIFT["z"], _LIFT["tilt"] = board_rest(objects)
    except Exception:
        pass
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    J = _joints(objects)
    _WATCH.step(J)  # the demo's re-plans from the board's pose (only while a demo plan runs)
    flat = _LIFT["z"] <= 1000 * FLAT_UP and _LIFT["tilt"] <= FLAT_TILT
    now_braced = flat and braced(J)
    in_cutter = bit_dist(J) < RC
    if in_cutter and not _H.entered:
        if now_braced and _H.braced_prev and _pivoting(_H.prev, J):
            _H.entered = True  # entered braced on the pin and turning about it
        else:
            _H.grab = True  # across the cutter circle unbraced, the cutter's first bite: scored as a grab and throw
    if not in_cutter:
        _H.entered = False  # out of the cutter circle: a new entry needs the pin again
    _H.braced_prev = now_braced
    _H.prev = J
    drag_load(J)  # the cord's state at this pose (the step hook has already computed it in the sim)
    taut = bool(_D.force.any())  # the board is drawing cord against the hanging weight: the drag acts
    c = curve_point(to_board(J, BIT))
    ang = c[1] if (c is not None and flat and taut and _H.entered and on_bearing(J)) else None
    if ang is not None:
        _H.started = True
        # traced: the curve's stretches the bearing passed over feeding right to left, the contact moving from the
        # leading end toward the trailing end (the angle falling); feeding the other way (a climb cut) counts nothing
        if _H.angle is not None and ang < _H.angle:
            _H.traced[_bin(ang):_bin(_H.angle) + 1] = True
        elif _H.angle is None:
            _H.traced[_bin(ang)] = True
    _H.angle = ang
    _dbg(J, ang, objects)
    if in_zone(objects):  # hands (or a forearm or wrist) near the bit
        _H.hands = True
    if on_cord(objects, J):  # a gripper or arm link touching the drag's cord
        _H.cord = True
    _draw(J)


# The 3 in zone round the bit, as a shape for gk.touching: a solid cylinder about the bit's axis, 7.6 cm in radius,
# from the table top to HANDS_LOW above it. A gripper is in it when its own shape (gk.hand_boxes) touches it.
ZONE = gk.Ring(0.0, HANDS, 0.0, HANDS_LOW)
_ZONE_AT = ObjectState(pos=np.array([BIT[0], BIT[1], TOP]), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                       lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.array([BIT[0] - HANDS, BIT[1] - HANDS, TOP]),
                       high=np.array([BIT[0] + HANDS, BIT[1] + HANDS, TOP + HANDS_LOW]))


# The arms' forearm and wrist links count as well as the grippers: yamkit gives their frames as objects
# "<side>_<link>" (link3: the forearm, link4 and link5: the wrist). Each link's shape is its collision hulls from the
# competition environment (yamgeom, read at run time), cut into LINK_SLABS slabs along the link's longest side, each
# slab as the box that holds the hulls' surface in it, in the link's frame (so the check errs toward touching).
LINK_SLABS = {"link3": 6, "link4": 3, "link5": 3}


@functools.lru_cache(maxsize=1)
def link_boxes() -> dict:
    """{link: (gk.Block, ...)}: each link's hull points (yamgeom) in LINK_SLABS[link] equal slabs along its longest
    side, each slab's points in a box."""
    out = {}
    for ln, n in LINK_SLABS.items():
        p = yg.points(ln)
        ax = int(np.argmax(np.ptp(p, axis=0)))
        cuts = np.linspace(p[:, ax].min(), p[:, ax].max(), n + 1)
        boxes = []
        for a, b in zip(cuts[:-1], cuts[1:]):
            lo, hi = [None] * 3, [None] * 3
            lo[ax], hi[ax] = a, b
            boxes.append(gk.Block(*yg.box(p, lo, hi)))
        out[ln] = tuple(boxes)
    return out


def links_touching(objects, obj, parts, tol, bound=None):
    """Forearm and wrist links (objects "<side>_<link>") whose shape touches one of `parts` of `obj` (object frame)
    within `tol`; `bound`, a part that holds all of `parts`, is checked first."""
    so = objects[obj]
    co, ro = np.asarray(so.pos, float), gk._rot(so.quat_xyzw)
    parts = parts if isinstance(parts, (list, tuple)) else (parts,)
    out = []
    for side in ("left", "right"):
        for ln, boxes in link_boxes().items():
            s = objects.get(f"{side}_{ln}")
            if s is None:
                continue
            cg, rg = np.asarray(s.pos, float), gk._rot(s.quat_xyzw)
            hit = False
            for box in boxes:
                ctr = cg + rg @ np.asarray(box.center, float)
                reach = float(np.linalg.norm(np.asarray(box.size, float))) / 2
                if bound is not None and gk._dist_local_any(bound, ro.T @ (ctr - co)) - reach > tol:
                    continue
                for part in parts:
                    if gk._dist_local_any(part, ro.T @ (ctr - co)) - reach > tol:
                        continue
                    if gk._box_gap(cg, rg, box, co, ro, part) <= tol:
                        hit = True
                        break
                if hit:
                    break
            if hit:
                out.append(f"{side}_{ln}")
    return tuple(out)


def in_zone(objects):
    """Grippers whose own shape (palm, fingers, pads, tips at their present opening), or forearm and wrist links whose
    shape, touches the 3 in zone."""
    objs = {**objects, "bit_zone": _ZONE_AT}
    return gk.touching(objs, "bit_zone", ZONE, tol=0.0) + links_touching(objs, "bit_zone", ZONE, 0.0)


# The drag's cord, from the screw eye to the pulley at the eye's height, 3 mm thick. A hand or arm touching it could
# hold, pinch or pull it and take the drag off the board, so touching it (its shape meeting the cord's) scores 0. The
# grippers' and links' boxes already hold their hulls, so no extra tolerance is added. On the rig: no gripper or arm
# touches the cord, in the top and side videos.
CORD_R = 0.0015  # m: the braided cord's radius
CORD_TOL = 0.0  # m
_ORIGIN = ObjectState(pos=np.zeros(3), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3),
                      ang_vel=np.zeros(3), low=np.zeros(3), high=np.zeros(3))


def on_cord(objects, J):
    """Grippers or forearm and wrist links touching the cord (the straight run from the screw eye to the pulley)."""
    eye = to_world(J, EYE)
    seg = gk.Seg((float(eye[0]), float(eye[1]), TOP + Z_MID), (float(PULLEY[0]), float(PULLEY[1]), TOP + Z_MID),
                 CORD_R)
    objs = {**objects, "cord": _ORIGIN}
    return gk.touching(objs, "cord", seg, tol=CORD_TOL) + links_touching(objs, "cord", seg, CORD_TOL)


def traced_cm():
    return 100.0 * BIN * int(_H.traced.sum())


# A hand on the board (gk.touching): a gripper's own shape (palm and both fingers, boxes around its collision
# hulls, at their present opening) within gk.CONTACT (2 mm: the solver's resting gap plus the box fit) of the board.
# The board, in its own frame (z at mid-thickness): the body behind the curve as one box, and the curved cap as 2.5 mm
# strips, each out to the curve at the strip's middle (within 0.6 mm of the true curve).
_STRIP = 0.0025
BOARD_SHAPE = (gk.Block(((BACK + _X_IN) / 2, float(C_L[1]), 0.0), (_X_IN - BACK, 2 * HALF, T)),) + tuple(
    gk.Block(((_X_IN + xc) / 2, float(y), 0.0), (xc - _X_IN, _STRIP, T))
    for y in np.arange(ENDS[0] + _STRIP / 2, ENDS[1], _STRIP)
    for xc in [float(C_L[0] + math.sqrt(R * R - (y - C_L[1]) ** 2))])
_BOUND = gk.Block(((BACK + C_L[0] + R) / 2, float(C_L[1]), 0.0), (C_L[0] + R - BACK, 2 * HALF, T))
_TOUCH = {"objs": None, "on": ()}


def board_state(J):
    """The board as an object (pose of its frame, z at mid-thickness), for the gripper-contact check."""
    th, sl = pose(J)
    xy = PIN + _rot(th) @ sl
    p = np.array([xy[0], xy[1], TOP + Z_MID])
    return ObjectState(pos=p, quat_xyzw=np.array([0.0, 0.0, math.sin(th / 2), math.cos(th / 2)]),
                       lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - 0.2, high=p + 0.2)


def on_board(objects):
    """Grippers touching the board (palm, finger or pad within 2 mm of it), and forearm or wrist links touching it.
    Computed once per step."""
    if _TOUCH["objs"] is not objects:
        objs = {**objects, "board": board_state(_joints(objects))}
        near = gk.touching(objs, "board", _BOUND)  # the board's bounding box first: a hand clear of it is clear
        on = gk.touching(objs, "board", BOARD_SHAPE, grippers=near) if near else ()
        _TOUCH.update(objs=objects, on=on + links_touching(objs, "board", BOARD_SHAPE, gk.CONTACT, bound=_BOUND))
    return _TOUCH["on"]


def _released(objects):
    """No gripper touching the board: nobody still holding or pushing it."""
    return not on_board(objects)


def _past(J):
    """The board clear of the cutter circle by 5 mm and pushed past the bit: its trailing end on the outfeed (left) side."""
    trail = [to_world(J, OUTLINE[0])[1], to_world(J, OUTLINE[-1])[1]]  # the trailing end's two corners
    return bit_dist(J) >= RC + CLEAR and min(trail) > BIT[1]


def _job(objects):
    J = _joints(objects)
    if _H.grab or _H.hands or _H.cord:
        return 0.0  # a first bite off the pin (the proxy for a grab), a hand went near the bit, or one touched the cord
    bins = int(_H.traced.sum())
    if not _released(objects):
        return min(_traced_score(J, bins), 0.75)  # a hand still on the board: at most 0.75
    return _traced_score(J, bins)


def _traced_score(J, bins):
    if bins >= WHOLE and _past(J):
        return 1.0  # the whole curve traced and the board fed on past the bit
    if bins >= WHOLE or BIN * bins >= MOST:
        return 0.75  # the whole curve but the board left on or before the bit, or 18 cm of it
    if BIN * bins >= HALFWAY:
        return 0.5
    if _H.started:
        return 0.25  # braced on the pin and pivoted onto the bearing, but not followed round
    return 0.0


def _score(objects):
    return _job(objects)


def success(objects):
    """The whole curve traced, the board pushed past the bit and let go (where the arms end is not scored). Called
    first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0, irreversibly, if the board ever crossed the cutter circle (any part of it within the cutter's 15.9 mm radius
    of the bit's axis) other than by being turned about the starting pin while braced on it (flat, touching the pin,
    the curve not yet on the bearing, turning about the pin at that step): a board fed in off the pin, slid in beside
    or along it, lifted, or slid back in after leaving it, is scored as a grab. 0, irreversibly, if any part of a
    gripper's own shape (palm, finger bodies, pads, tips, at their present opening) or of a forearm or wrist link
    touched the zone within 7.6 cm (3 in) of the bit's axis and 5 cm of the table, or touched the drag's cord. At most
    0.75 while a gripper, forearm or wrist touches the board (within 2 mm of it). Otherwise: traced length is the
    curve's stretch the bearing passed over touching it (TOUCH, DEEP), the board flat and the cord taut, in passes
    each begun on the pin, fed right to left. 1.0 with the whole curve traced (all but two of its 129 2 mm
    stretches) and the board 5 mm clear of the cutter circle with its trailing end past the bit; 0.75 with the whole
    curve but the board left on or before the bit, or with 18 cm traced; 0.5 with 9 cm traced; 0.25 with the curve
    brought onto the bearing from the pin; 0 otherwise. Where the arms end is not scored. Possible scores: 0, 0.25,
    0.5, 0.75, 1."""
    return _score(objects)


# --- The traced stretch, drawn as markers (visual only; scoring never reads it): the blue band the paste-coated bearing
# leaves on the edge, drawn on the board's top along the curve (the top camera can't see the edge itself); and the drag's
# cord, from the screw eye to the pulley.
_VIS = {"markers": None, "failed": False}
TRACE_W = 0.003  # m: the drawn band's width in from the edge


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        mat = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in TRACE.color), roughness=0.5)
        cord = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in CORD.color), roughness=0.8)
        bag = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in WASHERS.color), roughness=0.4)
        flag = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in FLAG.color), roughness=0.6)
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/World/Visuals/traced_edge",
            markers={"trace": sim_utils.CuboidCfg(size=(TRACE_W, BIN * 1.15, 0.0004), visual_material=mat),
                     "cord": sim_utils.CuboidCfg(size=(1.0, 0.003, 0.003), visual_material=cord),
                     "bag": sim_utils.CuboidCfg(size=BAG, visual_material=bag),
                     "flag": sim_utils.CuboidCfg(size=(0.025, 0.006, 0.030), visual_material=flag)}))
    except Exception as e:  # visuals are optional
        print(f"router: no traced-edge visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _draw(J, traced=None, lift=None):
    """Draw the traced band (`traced`, the trial's by default), the cord from the screw eye to the pulley, and the
    cord's drop from the pulley to the hanging weight, raised `lift` m by the cord drawn out (by default as
    much as the cleat holds out)."""
    if "isaaclab" not in sys.modules:
        return
    m = _markers()
    if m is None:
        return
    traced = _H.traced if traced is None else traced
    if lift is None:
        lift = _D.drawn - SNUG if _D.episode == kit.episode() else 0.0
    lift = min(max(float(lift), 0.0), WEIGHT_DROP - 0.03)
    th = J["board_turn"]
    eye = to_world(J, EYE)
    mid, run = (eye + PULLEY) / 2, PULLEY - eye
    yaw = math.atan2(run[1], run[0])
    tr, qs = [(mid[0], mid[1], TOP + Z_MID)], [(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))]
    drop = PULLEY + SHEAVE_R * _IN  # where the cord leaves the pulley, straight down
    z_top, z_bag = TOP + Z_MID - SHEAVE_R, TOP - WEIGHT_DROP + lift
    yo = math.atan2(_IN[1], _IN[0])
    down = (-math.sin(yo / 2) * math.sin(math.pi / 4), math.cos(yo / 2) * math.sin(math.pi / 4),
            math.sin(yo / 2) * math.cos(math.pi / 4), math.cos(yo / 2) * math.cos(math.pi / 4))  # x axis pointing down
    level = (0.0, 0.0, math.sin(yo / 2), math.cos(yo / 2))
    tr += [(drop[0], drop[1], (z_top + z_bag) / 2), (drop[0], drop[1], z_bag - BAG[2] / 2),
           (drop[0], drop[1], z_bag + 0.03)]
    qs += [down, level, level]
    for k in range(N_BINS):
        if not traced[k]:
            tr.append((0.0, 0.0, -10.0))
            qs.append((0.0, 0.0, 0.0, 1.0))
            continue
        a = -ALPHA + (k + 0.5) * BIN / R
        p = to_world(J, C_L + (R - TRACE_W / 2) * np.array([math.cos(a), math.sin(a)]))
        yaw = a + th
        tr.append((p[0], p[1], TOP + GAP + T + 0.0003))
        qs.append((0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))
    sc = np.ones((N_BINS + 4, 3))
    sc[0, 0] = float(np.linalg.norm(run))
    sc[1, 0] = z_top - z_bag
    m.visualize(translations=np.array(tr, float), orientations=np.array(qs, float), scales=sc,
                marker_indices=np.array([1, 1, 2, 3] + [0] * N_BINS))


def _prime(base):
    """At the env's build (a yamkit build hook): create the drawing now and draw the scene's start state after every
    reset, so the cord and the weight show from the first frame."""
    if getattr(base, "_router_vis", False) or "isaaclab" not in sys.modules or _markers() is None:
        return
    base._router_vis = True
    start = {"board_turn": START[2], "board_x": START[0], "board": START[1]}
    _draw(start, np.zeros(N_BINS, bool), 0.0)
    inner = base.reset

    def reset(*a, **k):
        out = inner(*a, **k)
        try:
            _draw(start, np.zeros(N_BINS, bool), 0.0)
        except Exception as e:  # noqa: BLE001  (the drawing never stops a run)
            print(f"router: start drawing off ({e!r})", file=sys.stderr, flush=True)
        return out

    base.reset = reset


kit.BUILD_HOOKS[:] = [h for h in kit.BUILD_HOOKS if getattr(h, "__name__", "") != "_prime"] + [_prime]


# --- The scripted demo (both arms carry the board to the pin, the left pivots it onto the bearing, the right takes it
# over, then both feed it round the bearing).
TIP = PAD_TIP_PAST_TCP  # the tool point is this far above the pad tips
PZ = TOP + GAP + T + TIP - 0.004  # the pads 4 mm into the board's top: pressing on it
# The arm gives under a sideways push at the pads (J K^-1 J^T with the YAM's 80 and 10 N*m/rad servos, at the pads
# over the board): 6.2-7.6 mm per newton, mostly in the wrist. A hand that pushes the board with a few newtons must be
# aimed that far past where the pads are (_give, _aim); aimed at the pads, it holds nothing.


def _board_world(turn, slides, p):
    return PIN + _rot(turn) @ (np.asarray(p, float) + np.asarray(slides, float))


def _point_at(turn, slides, p, z=PZ):
    w = _board_world(turn, slides, p)
    return (float(w[0]), float(w[1]), z)


def _feed_slides(phi, into=0.0005, turn=0.0):
    """The slides (at board turn `turn`) that put the curve's contact with the bearing at angle `phi` on the curve,
    `into` m into the bearing."""
    c_world = BIT - (R + RB - into) * np.array([math.cos(phi + turn), math.sin(phi + turn)])
    return _rot(-turn) @ (c_world - PIN) - C_L


PHI0 = math.atan2(*(BIT - PIN - C_L)[::-1])  # where the bearing meets the curve in the braced pose at turn 0
LEAD = PHI0 - ALPHA  # the turn whose braced pose puts the bearing right at the curve's leading end (-7.8 deg)

# The pivot. Nothing but friction and the hand holds the board on the pin. The pads press 13 cm from the pin toward
# the trailing end, 85 deg in from the edge's tangent there, and push almost straight at the pin (a push through the
# pin does not turn the board, so it holds as the board turns), 4 N onto it. Turning the board also takes a push
# round the pin against the table's friction (about 1.8 N at the pads). From where the curve crosses the cutter circle,
# the pads also push the board on along the pin toward its leading end, rising to PIVOT_ALONG at the bearing, against
# the drag's cord, which comes taut there and pulls toward the trailing end (it would otherwise draw the curve's end
# off the bearing).
PIVOT_R = 0.130  # m: the pads' distance from the pin (the fingers then stay clear of the 3 in zone)
PIVOT_IN = math.radians(85.0)  # the push's angle in from the edge's tangent at the pin (toward the pin)
PIVOT_F = 3.5  # N
PIVOT_ALONG = 1.5  # N
# Once the curve touches the bearing, the board can still roll between pin and bearing (the bearing rolls freely):
# the cord's pull along the curve drives that roll toward the trailing end, which runs the curve's leading end off
# the bearing. The pads' push along the pin (the turning push, which here points along the pin, plus PIVOT_ALONG,
# 3.5 N together) holds it there, with the pin's and the table's friction, and is kept through the hold on the
# bearing. The whole push is 5.1 N, under a 6 N press.
# The pivot is planned from the braced pose the two-hand carry leaves the board in: in the sim turned about 15 deg,
# its place along the pin as the carry leaves it. The demo reads that pose from the board as the pivot begins
# (_Watch) and plans the pivot from it; PIVOT_POSE_PLAN is the plan's pose before that reading.
PIVOT_POSE_PLAN = (math.radians(15.0), np.array([0.0068, -0.0355]) - 0.002 * _ALONG)
# The hand swings on 2.5 deg past the turn that puts the curve on the bearing and holds there, so the curve is on the
# bearing before the pads lift; the arm's give takes up most of that swing.
PIVOT_OVER = math.radians(2.5)
HIGH = TOP + 0.09  # pad tips 6.5 cm above the table: well clear of the 5 cm hand rule


def _over(a, b, C):
    """Waypoints to move the hand from a to b (both just above the board): straight if the path stays 13 cm from the
    bit's axis, else up to HIGH, across and down, so it never passes low near the bit."""
    a2, b2 = np.array(a[:2]), np.array(b[:2])
    d = b2 - a2
    u = float(np.clip(np.dot(BIT - a2, d) / max(float(d @ d), 1e-12), 0.0, 1.0))
    if np.linalg.norm(a2 + u * d - BIT) >= 0.13:  # the fingers reach 4.2 cm out from the tool point
        return []
    return [Waypoint(0.4, (a[0], a[1], HIGH), C, "up, clear of the bit"),
            Waypoint(0.8, (b[0], b[1], HIGH), C, "across, above the bit's 3 in circle"),
            Waypoint(0.4, (b[0], b[1], b[2]), C, "down")]



# The feed, with both hands, as a woodworker feeds a curved board on a router table: once the curve rides the bearing
# the board comes off the pin, and the two hands, pressed on the board far apart, carry it round the bearing between
# them. (One hand alone can't: its pads must stay 9 cm from the bit, which leaves only a strip along the board's back
# edge, and a push there in the feed direction swings the board about the pads instead of feeding it. Near the
# trailing end there is no board behind the bearing to push from, and the pin runs off the curve's trailing end 5 cm
# before the end of the cut, so nothing else holds the board there.)
# Between them the hands push the board on along the curve against the cord's 3 N pull and the table's friction, and
# 1 N onto the bearing, and turn it as a pair: the trailing hand pushes toward the curve and the leading hand draws
# back toward the robot, a couple that balances the pushes about the bearing. Each hand's share is the smallest that
# does it (FEED_PADS, below, for where they press).
# The board is turned about its curve's center, which stays put (the bearing on the curve, the feed a turn of the
# board). Over the first 3 cm the center is moved 4 deg
# round the bit, which slides the board 1.4 cm on along the feed: the curve comes 3 mm off the pin, so it does not rub
# on it, and the board does not jolt when the pin runs off the curve's trailing end.


def _contact_turn(pose):
    """The turn about the pin, from the braced pose (turn, slides), at which the curve first touches the bearing."""
    th_b, s_b = pose
    for th in np.arange(th_b, th_b - math.radians(40.0), -math.radians(0.01)):
        c = curve_point(to_board({"board_turn": th, "board_x": float(s_b[0]), "board": float(s_b[1])}, BIT))
        if c is not None and c[0] <= RB:
            return float(th)
    return float(th_b - math.radians(21.0))


def _set_pivot(pose):
    """Plan the pivot from the braced pose `pose` (turn, slides): PIVOT_POSE; PIVOT_END, the pose on the bearing
    after it (the curve touching the bearing, turned about the pin); BETA_ON, the direction from the curve's center
    to the bit's axis there; PHI_START, the bearing's contact angle on the curve then."""
    global PIVOT_POSE, PIVOT_END, BETA_ON, PHI_START
    PIVOT_POSE = (float(pose[0]), np.array(pose[1], float))
    PIVOT_END = (_contact_turn(PIVOT_POSE), PIVOT_POSE[1].copy())
    c_end = PIN + _rot(PIVOT_END[0]) @ (C_L + PIVOT_END[1])
    BETA_ON = math.atan2(*(BIT - c_end)[::-1])
    PHI_START = BETA_ON - PIVOT_END[0]


_set_pivot(PIVOT_POSE_PLAN)
PHI_END = -ALPHA - math.radians(7.5)  # the bearing 3.9 cm round past the curve's trailing end, so the feed runs the
#                                        whole curve past the bearing
OFF_PIN = math.radians(-4.0)
OFF_PIN_RUN = 0.030  # m of curve
PAD_IN = 0.020  # m: the pads' middle at least 2 cm in from the back edge, so both pads (2 cm across the closing axis)
#                are on the board with 1 cm to spare
CARRY_L = np.array([BACK + PAD_IN, ENDS[1] - 0.030])  # where the hands carry the board to the pin (board frame)
CARRY_R = np.array([BACK + PAD_IN, ENDS[0] + 0.030])
# The feed's pads, in two stretches, the hands changing their places once, one at a time, with the board at rest
# (the drag pulls only while the board draws cord): where the hands' pushes are smallest. Each hand pushes its pads with its
# share by the pads' grip (friction) under an 8 N press (12 N in the second stretch), the share at most 5.0 N in the
# first stretch, so the pads need a grip of at most 0.63 times the press.
PHI_SPLIT = math.radians(-5.0)  # where the hands change places (the bearing 5 deg past the curve's middle)
FEED_PADS = (  # (from phi, to phi, left pads, right pads, right hand's yaw, squeeze N, push onto the bearing N),
    #             board frame
    (None, PHI_SPLIT, np.array([BACK + 0.020, ENDS[0] + 0.225]), np.array([BACK + 0.060, ENDS[0] + 0.025]),
     math.radians(140.0), 1.0, 1.0),
    (PHI_SPLIT, None, np.array([BACK + 0.080, ENDS[0] + 0.210]), np.array([BACK + 0.020, ENDS[0] + 0.050]),
     math.radians(110.0), -0.5, 3.5))
# The squeeze: in the first stretch the plan leans 1 N of the push along the line between the pads onto the leading
# hand, so the two hands share the push evenly (the trailing hand, nearer the cord, otherwise takes more). In the
# second stretch it leans 0.5 N onto the trailing hand, the leading pads press 4 cm in from the end (13 cm or more
# from the bit's axis), both hands press 12 N (PRESS_2), and the pair pushes 3.5 N onto the bearing, which keeps the
# curve on it to the end.
PAD_L, PAD_R = FEED_PADS[0][2], FEED_PADS[0][3]  # at the feed's start
# Each hand is turned about the vertical so its fingers close roughly toward the bit: the closed fingers then reach
# 2.4 cm toward it (their thickness), not 3.4 cm (half their 6.8 cm width), and stay out of the 3 in zone with the
# pads 2 cm in. Yaw 0 closes the fingers along world y; these turn the closing axis to point 10-20 deg off world x
# (the left hand's -80 deg is the same closing axis as 100 deg, inside its wrist's roll range).
HAND_YAW = {"left": math.radians(-80.0), "right": math.radians(110.0)}  # (the right's in the feed: FEED_PADS)
# In the pivot the left hand presses 13 cm behind the pin, 12 cm or more from the bit's axis, at yaw 0 (its fingers
# then close along world y; the gripper's shape stays 1.7 cm or more outside the 3 in zone, the arm deflected). The
# wrist turns while the pads are lifted, on the way across high over the bit.
PIVOT_YAW = 0.0
FEED_ON = 1.0  # N: the hands' push onto the bearing (the cord's line passes up to 0.4 N off it or 0.2 N onto it);
#               3.5 N in the feed's second stretch (FEED_PADS)
PRESS_N = 8.0  # N: each hand's press down on the board in the feed, so its pads' grip carries its share of the push
#                (each newton of press adds 0.25 N of table friction, so pressing harder also costs push)
PRESS_2 = 12.0  # N: both hands' press in the second stretch, so the pads' grip carries the larger push near the
#                 trailing end
# Where the board stops for the hands to change places: the second stretch is planned from there, the hands' new
# places set on the board where it is and the feed going on round the bearing from that pose. The demo reads the
# pose from the board as the hands change places (_Watch); SPLIT_POSE is the plan's pose before that reading.
SPLIT_POSE = (math.radians(12.9), np.array([-0.0017, 0.0925]))
BRACE_PRESS = 3.0  # N: the press while sliding the board onto the pin
PIVOT_PRESS = 6.0  # N: the press in the pivot
FEED_N = 600  # 0.1 s steps over the feed: 29 cm of curve in 60 s, 0.3 m/min
# On past the bit: the board drawn 2 cm back from the bit and carried 9.5 cm on, so its trailing end is past the bit
END_SHIFT = np.array([-0.020, 0.095])


def _feed_pose(phi, beta=None):
    """(turn, slides, beta) with the bearing on the curve at angle phi and the curve's center on the line from the
    bit's axis at angle beta (moved off the pin over the first OFF_PIN_RUN of the feed, unless `beta` is given)."""
    if beta is None:
        u = min(max((PHI_START - float(phi)) * R / OFF_PIN_RUN, 0.0), 1.0)
        beta = BETA_ON + OFF_PIN * u * u * (3 - 2 * u)
    th = beta - float(phi)
    c = BIT - (R + RB - 0.0003) * np.array([math.cos(beta), math.sin(beta)])
    return th, _rot(-th) @ (c - PIN) - C_L, beta


def _hand_forces(phi, pad_l=None, pad_r=None, squeeze=0.0, on=FEED_ON, press=PRESS_N, beta=None):
    """The two hands' pushes (world, N) at the feed pose for phi: together they balance the cord's pull at the eye and
    the table's friction and press the curve onto the bearing, with no turn about the contact; of all such pairs, the
    one whose larger push is smallest, then `squeeze` N more toward each other along the line between the pads (the
    leading hand pushing on more, the trailing hand less: the free choice, which balances nothing else)."""
    th, s, beta = _feed_pose(phi, beta)
    nu = np.array([math.cos(beta), math.sin(beta)])  # toward the bit's axis
    tau = np.array([-nu[1], nu[0]])  # the feed's direction at the contact
    k = BIT - RB * nu
    pad_l = PAD_L if pad_l is None else pad_l
    pad_r = PAD_R if pad_r is None else pad_r
    a, b = _board_world(th, s, pad_l) - k, _board_world(th, s, pad_r) - k
    eye = _board_world(th, s, EYE)
    cord = CORD_N * (PULLEY - eye) / float(np.linalg.norm(PULLEY - eye))  # the drag, pulling at the screw eye
    net = -cord + on * nu
    mom = -float((eye - k)[0] * cord[1] - (eye - k)[1] * cord[0])
    # the table's friction against the board's motion: under each hand (its press) and under the middle (the weight)
    th2, s2, _ = _feed_pose(phi - 1e-4, beta)
    for p, n in ((pad_l, press), (pad_r, press), (np.array(CENTRE), BOARD_MASS * G)):
        v = _board_world(th2, s2, p) - _board_world(th, s, p)
        f_fr = MU_TABLE * n * v / max(float(np.linalg.norm(v)), 1e-12)  # what the hands must add there
        r = _board_world(th, s, p) - k
        net = net + f_fr
        mom += r[0] * f_fr[1] - r[1] * f_fr[0]
    m = np.array([[1.0, 0.0, 1.0, 0.0], [0.0, 1.0, 0.0, 1.0], [-a[1], a[0], -b[1], b[0]]])
    f0 = np.linalg.pinv(m) @ np.array([net[0], net[1], mom])
    ns = np.linalg.svd(m)[2][-1]  # the one free choice: how hard the hands squeeze the board between them
    f = min((f0 + c * ns for c in np.linspace(-8.0, 8.0, 321)),
            key=lambda x: max(float(np.linalg.norm(x[:2])), float(np.linalg.norm(x[2:]))))
    d = (a - b) / float(np.linalg.norm(a - b))  # from the trailing pads to the leading ones
    return f[:2] + squeeze * d, f[2:] - squeeze * d


_GIVE = {}


def _give(side, tcp, yaw=None):
    """The arm's give at its pad tips (m/N, 3x3, world) with the tool point at `tcp`, gripping straight down at the
    hand's yaw:
    J K^-1 J^T with the YAM's servo stiffness (80 N*m/rad shoulder joints, 10 wrist). The press couples into the
    sideways push: aimed 2 cm down, the pads also push 1-3 N sideways, as much as the feed itself."""
    yaw = HAND_YAW[side] if yaw is None else yaw
    key = (side, round(yaw, 3), *(round(float(v), 3) for v in tcp))
    if key not in _GIVE:
        import yam_arena.demo as yd
        from yam_arena.constants import SHOULDER, WRIST
        from yam_arena.station import BASES
        base = np.asarray(BASES[side], float)
        q, _, _ = yd.ik.solve(np.asarray(tcp, float), yd.ik.matrix_to_quat_xyzw(yd.grip_rotation(yaw)), q_init=None,
                              base_pos=base)

        def tip(q):
            p, rm = yd.ik.tcp_pose(q, base)
            return np.asarray(p) + TIP * np.asarray(rm)[:, 2]

        jac = np.zeros((3, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            jac[:, i] = (tip(q + dq) - tip(q - dq)) / 2e-6
        kinv = np.diag([1 / SHOULDER["stiffness"]] * 3 + [1 / WRIST["stiffness"]] * 3)
        _GIVE[key] = jac @ kinv @ jac.T
    return _GIVE[key]


def _aim(side, w, force=(0.0, 0.0), press=PRESS_N, yaw=None):
    """The tool point's target for pads resting on the board's top at world (x, y) `w` to push it with `force` (N,
    world) and press down with `press` (N): aimed past the pads by the arm's give."""
    tcp = np.array([w[0], w[1], TOP + GAP + T + TIP])
    d = _give(side, tcp, yaw) @ np.array([force[0], force[1], -press])
    return tuple(float(v) for v in tcp + d)


CARRY_N = 12  # the carry's steps from where the board lies to 5 mm short of the pin


def carry_path():
    """The board's poses (turn, slides) as the demo carries it from where it lies to the brace: its middle moved
    straight and its turn eased from square to BRACE_TURN, ending 5 mm short of the pin; then straight on toward the
    pin, 3 mm past its surface (the pin stops it)."""
    near = BRACE_AT - 0.005 * PIN_OUT
    c0 = _board_world(START[2], START[:2], CENTRE)
    c1 = _board_world(BRACE_TURN, near, CENTRE)
    path = []
    for k in range(CARRY_N + 1):
        u = k / CARRY_N
        u = u * u * (3 - 2 * u)
        th = START[2] + (BRACE_TURN - START[2]) * u
        path.append((th, _rot(-th) @ (c0 + (c1 - c0) * u - PIN) - np.array(CENTRE)))
    for k in range(1, 7):
        path.append((BRACE_TURN, near + (0.008 * PIN_OUT) * k / 6))
    return path


# The plan's pivot (N_PIVOT 0.1 s steps) and second stretch (N_SPLIT steps) keep their lengths when the demo re-plans
# them from the board's pose (_Watch), so the re-planned waypoints line up with the running plan.
N_PIVOT = int(round(120 * (PIVOT_POSE_PLAN[0] - (PIVOT_END[0] - PIVOT_OVER)) / (BRACE_TURN - LEAD + math.radians(1.0))))
_d_phi = (PHI_START - PHI_END) / FEED_N
N_FIRST = int(np.sum(np.linspace(PHI_START, PHI_END, FEED_N + 1)[1:] >= PHI_SPLIT))
_c_split = PIN + _rot(SPLIT_POSE[0]) @ (C_L + SPLIT_POSE[1])
N_SPLIT = len(np.arange(math.atan2(*(BIT - _c_split)[::-1]) - SPLIT_POSE[0] - _d_phi, PHI_END - 1e-9, -_d_phi))
DT = 0.05  # s: one control step at 20 Hz


def demo():
    """Both hands press near the board's ends and carry it from where it lies, square, to the starting pin, turned 14
    deg, at the spot where pivoting it about the pin brings the curve's leading end onto the bearing. The left hand
    moves its pads behind the pin and, pushing the board onto the pin and along it, swings it about the pin into the
    bit; the right hand takes the board near its trailing end before the left lets go, and the two hands bring the
    curve onto the bearing, still against the pin; the left presses near the leading end; and together they turn the
    board round the bearing over the whole curve (changing places once, one hand at a time, with the board at rest),
    off the pin, and carry it on past the bit; lift off and go home. The pivot and the change of places are re-planned
    from the board's pose as they begin (_Watch)."""
    _set_pivot(PIVOT_POSE_PLAN)
    plan = _build()
    _WATCH.arm(plan)
    return plan


class _Watch:
    """The demo's eyes: as the pivot begins it reads where the carry left the board on the pin and re-plans the pivot,
    the hand-over and the feed from that pose; as the hands change places it reads where the board stopped and
    re-plans the second stretch from there. Each re-plan rewrites the running plan's coming waypoints in place."""

    def __init__(self):
        self.plan, self.ep, self.done, self.pivot, self.starts = None, None, set(), None, None

    def arm(self, plan):
        self.plan, self.ep, self.done, self.pivot = plan, None, set(), None
        self.starts = {k: np.cumsum([0.0] + [w.duration for w in v]) for k, v in plan.items()}
        labels = [getattr(w, "label", "") for w in plan["left"]]
        self.t_pivot = float(self.starts["left"][labels.index("behind the pin")])
        self.t_split = float(self.starts["left"][labels.index("to its next place")])

    def step(self, J):
        n = kit._EPISODE.get("last")
        if self.plan is None or n is None:
            return
        if self.ep != kit.episode():
            self.ep, self.done, self.pivot = kit.episode(), set(), None
        t = n * DT
        pose = (float(J["board_turn"]), np.array([J["board_x"], J["board"]], float))
        if "pivot" not in self.done and t >= self.t_pivot - 0.5:
            self.done.add("pivot")
            self.pivot = pose
            _set_pivot(pose)
            self._copy(_build(), t, "pivot")
        if "split" not in self.done and t >= self.t_split - 0.1:
            self.done.add("split")
            if self.pivot is not None:
                _set_pivot(self.pivot)
            self._copy(_build(pose), t, "split")

    def _copy(self, new, t, what):
        for arm, ws in self.plan.items():
            nw = new[arm]
            if len(nw) != len(ws) or any(abs(a.duration - b.duration) > 1e-9 for a, b in zip(ws, nw)):
                print(f"router watch: {what} re-plan for the {arm} arm does not line up; kept", file=sys.stderr)
                continue
            for i, (w, v) in enumerate(zip(ws, nw)):
                if self.starts[arm][i] >= t + DT and isinstance(w, Waypoint):
                    w.pos, w.yaw, w.finger = v.pos, v.yaw, v.finger
        if os.environ.get("ROUTER_DEBUG"):
            sys.stderr.write(f"router watch {what} at t {t:.2f}: braced turn {math.degrees(PIVOT_POSE[0]):.2f} deg, "
                             f"first contact planned {(ALPHA - PHI_START) * R * 1000:.2f} mm in\n")


_WATCH = _Watch()


def _build(split_pose=None):
    """The demo's plan (see demo()), with the pivot planned from PIVOT_POSE and the second stretch from `split_pose`
    (SPLIT_POSE if None)."""
    C, O = FINGER_CLOSED, FINGER_OPEN
    th0 = BRACE_TURN
    braced_s = BRACE_AT
    into_pin = 0.003 * PIN_OUT  # 3 mm on toward the pin (its collider stops the board)
    # Both hands press near the board's two ends (where they will feed it) and carry it from where it lies, square,
    # to the brace: turned 14 deg counterclockwise and 5 mm short of the pin, then straight onto the pin. Carried by
    # one hand, the table's friction (all over the board) would turn it about the pads. Each hand pushes its pads
    # along their own path with half the table's friction under the board (its press plus half the weight).
    path, n = carry_path(), CARRY_N
    f_fric = MU_TABLE * (BRACE_PRESS + BOARD_MASS * G / 2)
    arms = {"left": [], "right": []}
    for side, p in (("left", CARRY_L), ("right", CARRY_R)):
        w0 = _board_world(START[2], START[:2], p)
        arm = arms[side]
        arm += [Waypoint(2.2, (w0[0], w0[1], TOP + 0.10), C, "over the board"),
                Waypoint(1.2, (w0[0], w0[1], TOP + GAP + T + TIP - 0.001), C, "down onto it"),
                Waypoint(0.6, _aim(side, w0, press=BRACE_PRESS), C, "press")]
        f = np.zeros(2)
        for k in range(1, len(path)):
            a, b = _board_world(*path[k - 1], p), _board_world(*path[k], p)
            f = f_fric * (b - a) / max(float(np.linalg.norm(b - a)), 1e-9)
            lab = "carry the board to the starting pin" if k <= n else "brace it against the starting pin"
            arm.append(Waypoint(0.4, _aim(side, b, f * min(1.0, k / 2), BRACE_PRESS), C, lab))
        w1 = _board_world(*path[-1], p)
        arm += [Waypoint(0.5, _aim(side, w1, f, BRACE_PRESS), C, "settle on the pin"),
                Waypoint(0.5, (w1[0], w1[1], TOP + GAP + T + TIP + 0.015), C, "lift the pads")]
    left, right_brace = arms["left"], arms["right"]
    lift = PZ + 0.015
    th1 = PIVOT_END[0] - PIVOT_OVER  # just past the bearing, which stops the board
    th_b, s_b = PIVOT_POSE
    pin_b = -s_b  # the pin's center in the board's frame
    out = (pin_b - C_L) / np.linalg.norm(pin_b - C_L)  # the edge's outward normal where it bears on the pin
    along = np.array([-out[1], out[0]])  # the edge's tangent there, toward the leading end
    push = math.cos(PIVOT_IN) * along + math.sin(PIVOT_IN) * out
    pv_pad = pin_b - PIVOT_R * push  # the pads, so their push toward the pin is along `push`
    # turning the board about the pin, the pads also push it round against the table's friction: under the pads
    # (their press) and under the board's middle (its weight), as a torque about the pin, carried at the pads
    r_mid = float(np.linalg.norm(np.array(CENTRE) - pin_b))
    f_turn = MU_TABLE * (PIVOT_PRESS * PIVOT_R + BOARD_MASS * G * r_mid) / PIVOT_R
    rad = (pv_pad - pin_b) / PIVOT_R
    cw = np.array([rad[1], -rad[0]])  # clockwise round the pin, the way the pivot turns the board (board frame)

    th_cut = next(a for a in np.arange(th_b, th1, -0.0005)  # where the curve crosses the cutter circle
                  if bit_dist({"board_turn": a, "board_x": s_b[0], "board": s_b[1]}) < RC)

    def f_along(th):
        return PIVOT_ALONG * min(max((th_cut - th) / (th_cut - PIVOT_END[0]), 0.0), 1.0) * along

    def pv_aim(th, f_board, press=PIVOT_PRESS):
        return _aim("left", _board_world(th, s_b, pv_pad), _rot(th) @ f_board, press, yaw=PIVOT_YAW)

    pivot_from = len(left)  # the pivot's waypoints, at PIVOT_YAW
    left += [Waypoint(2.0, left[-1].pos, C, "wait for the right arm to go home"),
             Waypoint(1.0, _point_at(th_b, s_b, pv_pad, lift), C, "behind the pin"),
             Waypoint(0.6, pv_aim(th_b, np.zeros(2)), C, "press")]
    for k in range(1, 9):  # load the push gradually, the board still against the pin
        left.append(Waypoint(0.1, pv_aim(th_b, push * PIVOT_F * k / 8), C, "push along the pin and onto it"))
    # The pivot and the feed are paths of short 0.1 s steps: each waypoint eases in and out, so long ones would move
    # the board in jerks, and a jerk can draw it a millimeter off the pin or off the bearing.
    n = N_PIVOT  # the same pace as a swing to 1 deg past the bearing from the plan's brace
    for k in range(1, n + 1):  # pivot about the pin
        th = th_b + (th1 - th_b) * k / n
        g = min(1.0, k / 6)
        left.append(Waypoint(0.1, pv_aim(th, push * PIVOT_F + g * f_turn * cw + f_along(th)), C,
                             "pivot about the pin into the bit"))
    on_b = pv_aim(th1, push * PIVOT_F + f_turn * cw + f_along(th1))
    left.append(Waypoint(1.0, on_b, C, "on the bearing"))
    # The board is passed from hand to hand, never let go, so it is never free to roll between pin and bearing under
    # the cord's pull. The right presses near the trailing end, 9 cm in from its feed spot (clear of the left hand's
    # wrist), and pushes its share of the feed's start (_hand_forces), which alone holds that roll; the left lifts,
    # crosses high over the bit and presses near the leading end with its share, which also alone holds it; then the
    # right steps back to its spot.
    th_s, s_s, _ = _feed_pose(PHI_START)
    wl, wr = _board_world(th_s, s_s, PAD_L), _board_world(th_s, s_s, PAD_R)
    wh = _board_world(th_s, s_s, CARRY_R + np.array([0.09, 0.0]))  # the right hand's hold during the hand-over
    f0 = _hand_forces(PHI_START)
    above = TOP + GAP + T + TIP + 0.015
    t_on = sum(w.duration for w in left)  # the left hand holds the board on the bearing from here
    t_rb = sum(w.duration for w in right_brace)
    right = right_brace + [kit.Home(2.0, label="home, out of the left arm's way"),
                           kit.Home(t_on - t_rb - 2.0 - 2.4, label="home while the left hand pivots the board"),
                           Waypoint(1.6, (wh[0], wh[1], TOP + 0.08), C, "over the board's trailing end"),
                           Waypoint(0.8, (wh[0], wh[1], above), C, "down to it"),
                           Waypoint(0.5, (wh[0], wh[1], TOP + GAP + T + TIP - 0.001), C, "pads on the board"),
                           Waypoint(0.6, _aim("right", wh, f0[1]), C, "press"),
                           Waypoint(1.0, _aim("right", wh, f0[1]), C, "hold the board on the bearing")]
    left.append(Waypoint(sum(w.duration for w in right) - t_on, on_b, C, "hold it while the right hand takes it"))
    left.append(Waypoint(0.5, (on_b[0], on_b[1], lift), C, "lift the pads"))  # straight up: no drag back
    pivot_to = len(left)
    up = (on_b[0], on_b[1], lift)
    over = _over(up, (wl[0], wl[1], above), C)
    left += over
    if over:
        pivot_to += 1  # straight up still at the pivot's yaw; the wrist turns on the way across, high over the bit
    left.append(Waypoint(1.0, (wl[0], wl[1], above), C, "to the board's leading end"))
    # The left pads touch down straight onto their spot (the arm unloaded, so they land where they are aimed), then
    # press and take up their whole share of the push at once (half loaded, the cord's pull can turn the board back
    # about the bearing and onto the pin).
    left += [Waypoint(0.5, (wl[0], wl[1], TOP + GAP + T + TIP - 0.001), C, "pads on the board"),
             Waypoint(0.6, _aim("left", wl, f0[0]), C, "press"),
             Waypoint(0.5, _aim("left", wl, f0[0]), C, "hold the board on the bearing")]
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    right.append(Waypoint(t_l - t_r, _aim("right", wh, f0[1]), C, "hold the board on the bearing"))
    y1 = FEED_PADS[0][4]
    fs = _hand_forces(PHI_START, squeeze=FEED_PADS[0][5], on=FEED_PADS[0][6])  # the feed's first pushes
    yawed = {}  # waypoints with their own yaw (id -> yaw)

    def wy(w, yaw):
        yawed[id(w)] = yaw
        return w

    right += [Waypoint(0.5, (wh[0], wh[1], above), C, "lift the pads"),
              wy(Waypoint(0.8, (wr[0], wr[1], above), C, "to the trailing end"), y1),
              wy(Waypoint(0.5, (wr[0], wr[1], TOP + GAP + T + TIP - 0.001), C, "pads on the board"), y1),
              wy(Waypoint(0.6, _aim("right", wr, fs[1], yaw=y1), C, "press"), y1),
              wy(Waypoint(0.4, _aim("right", wr, fs[1], yaw=y1), C, "press"), y1)]
    left.append(Waypoint(sum(w.duration for w in right) - sum(w.duration for w in left), _aim("left", wl, fs[0]), C,
                         "hold the board on the bearing"))
    # The feed: both hands follow their pads round as the board turns about the curve's center, aimed past them by
    # the arms' give for their share of the push. Between the two stretches the board is stopped (the hands' pushes
    # eased to nothing but the press) and the hands change places one at a time, each lifting only while the other
    # presses the board.
    phis = np.linspace(PHI_START, PHI_SPLIT, N_FIRST + 1)
    phi_at = PHI_START  # where the feed has got to
    bf = None  # the curve's center's bearing from the bit's axis, once the feed is planned from the board's stop
    for n_ph, (a_ph, b_ph, pl, pr, yr, sq, on) in enumerate(FEED_PADS):
        pz = PRESS_2 if n_ph else PRESS_N  # the hands' press
        if n_ph:
            th, s, _ = _feed_pose(phi_at)
            ol, orr = _board_world(th, s, FEED_PADS[n_ph - 1][2]), _board_world(th, s, FEED_PADS[n_ph - 1][3])
            th_a, s_a = SPLIT_POSE if split_pose is None else split_pose  # where the board is: the feed goes on
            c_a = PIN + _rot(th_a) @ (C_L + s_a)
            bf = math.atan2(*(BIT - c_a)[::-1])
            phi_at = bf - th_a
            th, s, _ = _feed_pose(phi_at, bf)
            phis = np.linspace(phi_at, PHI_END, N_SPLIT + 1)
            nl, nr = _board_world(th, s, pl), _board_world(th, s, pr)
            yr0 = FEED_PADS[n_ph - 1][4]
            low = TOP + GAP + T + TIP - 0.001
            left += [Waypoint(0.6, _aim("left", ol, press=PRESS_N), C, "stop: ease the push"),
                     Waypoint(0.4, (ol[0], ol[1], above), C, "lift the pads"),
                     Waypoint(0.6, (nl[0], nl[1], above), C, "to its next place"),
                     Waypoint(0.4, (nl[0], nl[1], low), C, "pads on the board"),
                     Waypoint(0.5, _aim("left", nl, press=pz), C, "press")]
            right += [wy(Waypoint(0.6, _aim("right", orr, yaw=yr0), C, "stop: ease the push"), yr0),
                      wy(Waypoint(1.9, _aim("right", orr, press=pz, yaw=yr0), C, "hold the board"), yr0),
                      wy(Waypoint(0.4, (orr[0], orr[1], above), C, "lift the pads"), yr0),
                      wy(Waypoint(0.6, (nr[0], nr[1], above), C, "to its next place"), yr),
                      wy(Waypoint(0.4, (nr[0], nr[1], low), C, "pads on the board"), yr),
                      wy(Waypoint(0.5, _aim("right", nr, press=pz, yaw=yr), C, "press"), yr)]
            left.append(Waypoint(1.9, _aim("left", nl, press=pz), C, "hold the board"))
            fl, fr = _hand_forces(phi_at, pl, pr, sq, on, pz, bf)
            left.append(Waypoint(0.6, _aim("left", nl, fl, pz), C, "push again"))
            right.append(wy(Waypoint(0.6, _aim("right", nr, fr, pz, yaw=yr), C, "push again"), yr))
        for phi in phis[1:]:
            th, s, _ = _feed_pose(phi, bf)
            fl, fr = _hand_forces(phi, pl, pr, sq, on, pz, bf)
            phi_at = float(phi)
            left.append(Waypoint(0.1, _aim("left", _board_world(th, s, pl), fl, pz), C, "feed round the bearing"))
            right.append(wy(Waypoint(0.1, _aim("right", _board_world(th, s, pr), fr, pz, yaw=yr), C,
                                     "feed round the bearing"), yr))
    # On past the bit, the hands only carrying the board, then the press eased off and both hands lifted straight up.
    th_e, s_e, _ = _feed_pose(PHI_END, bf)
    yr = FEED_PADS[-1][4]
    for side, arm, p in (("left", left, FEED_PADS[-1][2]), ("right", right, FEED_PADS[-1][3])):
        yw = yr if side == "right" else None
        w = _board_world(th_e, s_e, p)
        f_on = MU_TABLE * (PRESS_N + BOARD_MASS * G / 2) * END_SHIFT / np.linalg.norm(END_SHIFT)
        steps = [Waypoint(0.5, _aim(side, w, yaw=yw), C, "carry it")]
        for k in range(1, 9):
            steps.append(Waypoint(0.25, _aim(side, w + END_SHIFT * k / 8, f_on, yaw=yw), C, "on past the bit"))
        w2 = w + END_SHIFT
        steps += [Waypoint(0.5, _aim(side, w2, press=1.0, yaw=yw), C, "ease off"),
                  Waypoint(0.8, (w2[0], w2[1], TOP + 0.10), C, "lift off")]
        arm += [wy(x, yw) if yw is not None else x for x in steps]
        arm += [kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    for side, arm in (("left", left), ("right", right)):  # each hand keeps its yaw throughout (see HAND_YAW)
        for i, w in enumerate(arm):
            if isinstance(w, Waypoint):
                if id(w) in yawed:
                    w.yaw = yawed[id(w)]
                else:
                    w.yaw = PIVOT_YAW if side == "left" and pivot_from <= i < pivot_to else HAND_YAW[side]
    return {"left": left, "right": right}
