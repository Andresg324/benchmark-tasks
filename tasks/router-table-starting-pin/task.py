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
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled table, bit and board; nothing on disk changes)
import guardkit as gk  # noqa: E402  (after yamkit)
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
RC = 0.0159  # m: the cutter's radius (a 1.25 in rabbeting bit): the rabbet is RC - RB = 9.5 mm (3/8 in) wide
CUT_H = 0.0032  # m: the cutter stands 1/8 in above the table: the first pass of a 1/4 in deep rabbet (the usual
#                 "no more than 1/8 in per pass"; a second pass at full height finishes it, outside this task)
BEAR_Z = (0.0034, 0.0082)  # m: the bearing's bottom and top above the table (3/16 in thick, on the bit's tip)
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
# The board's friction on the router table's top (phenolic or high-pressure laminate). The board is a free rigid body
# lying on the table, so its weight and the hands' press bear on the table top through a contact, and PhysX's Coulomb
# friction there resists its sliding and turning: mu 0.25 sliding, 0.3 at rest. No published value for planed hardwood on
# phenolic or laminate was found; 0.25 is the low end of clean, dry wood on wood (0.25-0.5) and inside wood on clean
# metal (0.2-0.6) (Engineering ToolBox, "Friction - friction coefficients"), and above wet or waxed wood (0.2 and
# less). The real trial's feed-force check measures it.
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
BRACE = 0.0015  # m: the board is on the pin within 1.5 mm of its surface (the pin's collider keeps it from closer;
#                 1.5 mm is the play of a hand-held board against a 0.25 in pin)
ON_BEARING = 0.0015  # m: the curve is on the bearing within 1.5 mm of its surface (the rabbet then within 1.5 mm of
#                      full width; the bearing's collider keeps the curve from going deeper)
CLEAR = 0.005  # m: at the end, no part of the board within 5 mm of the cutter
BIN = 0.002  # m: the routed length is kept as 2 mm stretches of the curve (129 of them)
N_BINS = int(round(ARC / BIN))
WHOLE = N_BINS - 2  # full credit: the whole curve routed, every 2 mm stretch but the one at each very end (the
#                    bearing rolls off the corner there); in one feed or in passes, each begun braced on the pin
MOST = 0.180  # m: 18 cm, 70% of the 25.8 cm curve: most of it, as from a start a few cm in from the leading end
HALFWAY = MOST / 2  # m: 9 cm: real progress round the bearing
HANDS = 0.076  # m: the three-inch rule of router-table manuals for hands near a bit: no part of either gripper (its
#                own shape: palm, finger bodies, pads and tips, boxes fitted to the YAM's collision meshes, at their
#                present opening) may touch the zone within 3 in (7.6 cm) of the bit's axis, up to HANDS_LOW above the
#                table. The zone is the hazard itself, since a slip or a grab carries a hand that close into the cutter.
HANDS_LOW = 0.05  # m: the zone's height above the table top (the bit and the board's top are below 1.4 cm)
GRIPPERS = ("left_gripper", "right_gripper")


def on_bearing(J):
    """The curved edge rides the bearing: the bit's axis within the bearing's radius plus 1.5 mm of the curve."""
    c = curve_point(to_board(J, BIT))
    return c is not None and c[0] <= RB + ON_BEARING


def braced(J):
    """The pivot condition: the board within 1.5 mm of the pin and its curve not yet on the bearing. Nothing holds it
    there but whatever pushes it onto the pin (the pin is a plain collider); once the curve rides the bearing it comes
    off the pin."""
    return pin_gap(J) <= BRACE and not on_bearing(J)


CENTRE = (float(C_L[0] + R - DEPTH / 2), float(C_L[1]))  # the board's middle in its frame: the board body's origin


def _centroid(poly):
    """Area and centroid of a convex polygon (counterclockwise points)."""
    x, y = poly[:, 0], poly[:, 1]
    x1, y1 = np.roll(x, -1), np.roll(y, -1)
    cr = x * y1 - x1 * y
    a = cr.sum() / 2
    return a, np.array([((x + x1) * cr).sum(), ((y + y1) * cr).sum()]) / (6 * a)


COM = _centroid(OUTLINE)[1] - np.array(CENTRE)  # the board's center of mass from its middle (toward the curve's bow)


# --- The cutter's force on the board. Curti, Marcon, Denaud, Togni, Furferi and Goli (2021), "Generalized cutting
# force model for peripheral milling of wood, based on the effect of density and fibre orientation", Eur. J. Wood
# Prod. 79:667-678, Table 5 (up-milling, straight edge): per tooth Ft = (Ks*h + Int)*ap, with Ks and Int per unit of
# wood density as functions of the grain angle GA (deg). Their model reproduces the router forces Goli et al. (2003)
# measured. Averaged over a turn of a 2-flute cutter engaged over phi_e = acos(1 - 2 ae / D), with chip thickness
# h = fz sin(phi): the mean force against the feed (along the cutting edge's motion at the contact) is
#   Fx = z ap / (2 pi) * (Ks fz sin^2(phi_e) / 2 + Int sin(phi_e))
# and the mean pull toward the bit's axis is
#   Fy = z ap / (2 pi) * (Ks fz (phi_e / 2 - sin(2 phi_e) / 4) + Int (1 - cos(phi_e))).
# For this pass (ae 9.5 mm, ap 3.2 mm, 22,000 rpm) that is about 3.0-3.3 N against the feed and 2.0-2.2 N toward the
# bit at 0.75-1 m/min, rising to 3.6-5.0 N at 5 m/min; a full 1/4 in pass would double it. Feed rates here
# (chip 10-60 um) are below the 40-100 um the model was fitted on, so the chip-independent term Int dominates.
RHO = 561.0  # kg/m3: black cherry at 12% moisture (AHEC)
RPM = 22000.0  # the router's speed
FLUTES = 2
DT = 0.05  # s: one control step at 20 Hz (the run's control rate)
HOLD_STEPS = 4  # a cut's force is held up to 0.2 s while the cutter stays in the wood without 10 um of fresh depth
_CUT_A = np.array([-ALPHA + (k + 0.5) * BIN / R for k in range(int(round(ARC / BIN)))])  # the curve's 2 mm bins
_CUT_P = np.stack([C_L + R * np.array([math.cos(a), math.sin(a)]) for a in _CUT_A])  # their points (board frame)


def cutting_force(ae, ap, vf, ga_deg):
    """(Fx, Fy) in N: the mean force against the feed and the pull toward the bit's axis, for a radial depth ae (m),
    cut height ap (m), feed speed vf (m/s) and grain angle ga_deg (deg), Curti et al. 2021 (see above)."""
    if ae <= 0.0:
        return 0.0, 0.0
    ga = min(abs(ga_deg), 90.0)
    ks = (-5e-6 * ga * ga + 1e-3 * ga + 26e-3) * RHO  # N/mm2
    it = (-1e-7 * ga * ga + 1e-5 * ga + 55e-4) * RHO  # N/mm
    fz = 1000.0 * max(vf, 0.0) / (RPM / 60.0 * FLUTES)  # mm per tooth
    pe = math.acos(max(-1.0, 1.0 - 2.0 * ae / (2 * RC)))
    k = FLUTES * 1000.0 * ap / (2 * math.pi)
    fx = k * (ks * fz * math.sin(pe) ** 2 / 2 + it * math.sin(pe))
    fy = k * (ks * fz * (pe / 2 - math.sin(2 * pe) / 4) + it * (1 - math.cos(pe)))
    return fx, fy


class _Cut:
    """Per trial: how deep (m, from the uncut curve) the cutter has reached at each 2 mm bin, the force now, and the
    last pose (to tell the board's speed)."""

    def __init__(self):
        self.episode = kit.episode()
        self.depth = np.zeros(len(_CUT_A))
        self.key = None
        self.prev = None  # the deepest-cutting bin's world point at the last step
        self.force = np.zeros(2)  # N, world (x, y), smoothed
        self.at = BIT.copy()  # where it acts (world)
        self.load = (0.0, 0.0, 0.0)
        self.target = np.zeros(2)  # N: the last fresh cut's force, held while the cutter stays in the wood
        self.held = 0  # steps it has been held


_C = _Cut()


def cut_load(J):
    """The cutter's force on the board, as (its moment about the pin, its components along the board's x and y), and
    its bookkeeping; _push_board puts it on the board's body. The cutter removes the board's curve inside its radius; each bin's fresh depth this step is what
    it cuts. The force acts at the deepest fresh bin along the cutting edge's motion there (the bit turns
    counterclockwise seen from above), which is against a right-to-left feed and with a climb cut, plus a pull toward
    the bit's axis that the bearing takes. It computes once per control step (the same pose returns the same load)."""
    global _C
    if _C.episode != kit.episode():
        _C = _Cut()
    key = (round(J["board_turn"], 9), round(J["board_x"], 9), round(J["board"], 9))
    if key == _C.key:
        return _C.load
    _C.key = key
    w = np.array([to_world(J, p) for p in _CUT_P])
    r = np.linalg.norm(w - BIT, axis=1)
    reach = np.clip(RC - r, 0.0, RC - RB)  # how deep the cutter reaches into the curve at each bin now
    fresh = reach > _C.depth + 1e-5
    if fresh.any():
        k = int(np.argmax(np.where(fresh, reach, -1.0)))
        ae = float(reach[k])
        vf = 0.0 if _C.prev is None else float(np.linalg.norm(w[k] - _C.prev)) / DT
        fx, fy = cutting_force(ae, CUT_H, min(vf, 0.2), math.degrees(_CUT_A[k]))
        u = (w[k] - BIT) / max(float(r[k]), 1e-6)  # out from the bit's axis to the contact
        edge = np.array([-u[1], u[0]])  # the cutting edge's motion there (counterclockwise)
        target = fx * edge - fy * u
        _C.at, _C.prev = w[k], w[k]
        _C.depth = np.maximum(_C.depth, reach)
        _C.target, _C.held = target, 0
    elif reach.max() > 0.0 and _C.held < HOLD_STEPS:
        target = _C.target  # still in the wood, under 10 um fresh this step: the cut goes on at its last force
        _C.held += 1
    else:
        target = np.zeros(2)
        _C.target, _C.prev = np.zeros(2), None
    _C.force = _C.force + 0.5 * (target - _C.force)  # a 0.1 s lag: the mean of the tooth pulses, not each pulse
    f, c = _C.force, _C.at
    th = J["board_turn"]
    tau = float((c - PIN)[0] * f[1] - (c - PIN)[1] * f[0])
    ex, ey = _rot(th) @ np.array([1.0, 0.0]), _rot(th) @ np.array([0.0, 1.0])
    _C.load = (tau, float(f @ ex), float(f @ ey))
    return _C.load


def cut_force_now():
    """The cutter's force on the board at the last step, (x, y) in N (world), for logs and tests."""
    return _C.force.copy()


# Where the demo braces the board on the pin: turned 14 deg counterclockwise from square (its leading end swung back
# toward the robot), at the place along its curve where a pivot about the pin brings the bearing onto the curve 3 mm in
# from its leading end (the woodworker's margin: braced any nearer the end, the bearing meets end grain). The demo
# finds this pose itself; the task starts the board square (START, below).
BRACE_TURN = math.radians(14.0)
_PHI0 = math.atan2(*(BIT - PIN - C_L)[::-1])
_BRACED_END = _rot(-(_PHI0 - ALPHA)) @ C_L - C_L  # braced where the pivot brings the curve's very end to the bearing
_ALONG = np.array([-0.0022, 0.0083]) / np.linalg.norm([-0.0022, 0.0083])  # along the pin, away from the curve's end
BRACE_AT = _BRACED_END + 0.003 * _ALONG  # the slides of the braced pose the demo makes (3 mm along)
PIN_OUT = (-BRACE_AT - C_L) / np.linalg.norm(-BRACE_AT - C_L)  # from the curve's center to the pin there (board frame)
# The board starts as a woodworker sets it down: flat on the table on the infeed (right) side of the bit, square to
# the table (its long sides along y, the curve toward the bit), 2.2 cm from the pin and 3.4 cm from the bit's axis.
START = (-0.015, -0.040, 0.0)  # (x slide, y slide, turn): 1.5 cm nearer the robot, 4 cm right of the pose at turn 0
#                                where the curve touches both pin and bearing
WOOD = kit.Look((0.56, 0.31, 0.19), 0.6)  # cherry, planed
END_GRAIN = kit.Look((0.44, 0.23, 0.14), 0.7)
SAWN = kit.Look((0.47, 0.26, 0.16), 0.8)  # the band-sawn curve, a little darker and rougher
FRESH = kit.Look((0.93, 0.74, 0.55), 0.6)  # freshly routed cherry (the rabbet), drawn by markers
LAMINATE = kit.Look((0.80, 0.81, 0.79), 0.4)
PLATE = kit.Look((0.12, 0.12, 0.13), 0.4)  # phenolic insert plate
STEEL = kit.Look((0.78, 0.79, 0.80), 0.25, metallic=0.8)
CARBIDE = kit.Look((0.80, 0.18, 0.10), 0.35, metallic=0.4)  # the bit's coated body and carbide wings
DARK = kit.Look((0.20, 0.20, 0.21), 0.5)
POLYCARB = kit.Look((0.86, 0.92, 0.95), 0.05, opacity=0.25)  # the guard's clear polycarbonate shield
ALUMINUM = kit.Look((0.74, 0.75, 0.77), 0.35, metallic=0.8)  # its anodized aluminum body
# The plate's overhead pin guard, after the Bench Dog ProGuard (Bench Dog Tools catalog, 2008, p. 16: a polycarbonate
# shield on an aluminum body bolted to the plate, "1-1/2\" stock clearance under the guard", usable as a starting pin's
# guard): a clear 10 cm shield centered over the bit, its underside 1-1/2 in (38 mm) above the table, held by an
# aluminum arm from a post bolted to the plate's far side (away from the robot). The board passes under it; the 3 in
# rule already keeps the hands' low parts out from under it.
GUARD_R = 0.051  # m: the shield's radius (a 4 in disc; the diameter is assumed, the height is the maker's)
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
    """The router table's top: laminate, with the insert plate around the bit drawn on it."""
    meshes = [kit.box((0.29 - PIN[0], 0.04 - PIN[1], 0.0003), (0.40, 0.62, 0.0006), LAMINATE)]
    cols = [kit.Collider((0.29 - PIN[0], 0.04 - PIN[1], 0.0003), size=(0.40, 0.62, 0.0006))]
    return kit.Prop("router_table", pos=(float(PIN[0]), float(PIN[1]), TOP), size=(0.40, 0.62, 0.0006),
                    meshes=tuple(meshes), colliders=tuple(cols))


def _bit():
    """The insert plate with the bit and the starting pin (fixed; its own prop, so the board touches them)."""
    px, py = PIN - BIT
    meshes = [kit.box((0.0, 0.0, 0.0008), (0.235, 0.300, 0.0010), PLATE),  # a 9.25 x 11.75 in phenolic plate
              kit.cylinder((0.0, 0.0, 0.00135), 0.024, 0.0001, DARK),  # the insert ring's opening round the bit
              kit.cylinder((0.0, 0.0, CUT_H / 2), RC, CUT_H, CARBIDE, n=40),  # the cutter's swept cylinder
              kit.cylinder((0.0, 0.0, CUT_H + 0.0001), 0.0065, 0.0002, STEEL),
              kit.cylinder((0.0, 0.0, sum(BEAR_Z) / 2), RB, BEAR_Z[1] - BEAR_Z[0], STEEL),  # the bearing
              kit.cylinder((0.0, 0.0, BEAR_Z[1] + 0.0008), 0.0035, 0.0016, DARK),  # its retaining screw
              kit.cylinder((px, py, PIN_H / 2), RP, PIN_H, STEEL, n=20),  # the starting pin
              kit.cylinder((px, py, 0.0014), 0.0060, 0.0002, DARK),  # its threaded hole's rim
              # the overhead guard: shield, arm and post (see GUARD_R)
              kit.cylinder((0.0, 0.0, sum(GUARD_Z) / 2), GUARD_R, GUARD_Z[1] - GUARD_Z[0], POLYCARB, n=48),
              kit.box(((GUARD_R - 0.02 + GUARD_POST_X) / 2, 0.0, GUARD_Z[1] + 0.004),
                      (GUARD_POST_X - GUARD_R + 0.02, 0.030, 0.008), ALUMINUM),
              kit.box((GUARD_POST_X, 0.0, (GUARD_Z[1] + 0.008) / 2), (0.025, 0.040, GUARD_Z[1] + 0.008), ALUMINUM)]
    tx, ty = 0.29 - BIT[0], 0.04 - BIT[1]  # the table top's middle, from the bit
    cols = [kit.Collider((0.0, 0.0, sum(BEAR_Z) / 2), radius=RB, height=BEAR_Z[1] - BEAR_Z[0], slick=True),  # it rolls
            kit.Collider((px, py, PIN_H / 2), radius=RP, height=PIN_H),
            # the guard is solid: a thrown board or a hand meets it
            kit.Collider((0.0, 0.0, sum(GUARD_Z) / 2), radius=GUARD_R, height=GUARD_Z[1] - GUARD_Z[0]),
            kit.Collider(((GUARD_R - 0.02 + GUARD_POST_X) / 2, 0.0, GUARD_Z[1] + 0.004),
                         size=(GUARD_POST_X - GUARD_R + 0.02, 0.030, 0.008)),
            kit.Collider((GUARD_POST_X, 0.0, (GUARD_Z[1] + 0.008) / 2), size=(0.025, 0.040, GUARD_Z[1] + 0.008)),
            # the table top's surface under the board (its top at the board's underside, GAP above the drawn top):
            # hardwood on phenolic or laminate, the board's sliding friction on it (MU_TABLE, above)
            kit.Collider((tx, ty, GAP / 2), size=(0.40, 0.62, GAP), friction=(MU_TABLE_STATIC, MU_TABLE))]
    return kit.Prop("router_bit", pos=(float(BIT[0]), float(BIT[1]), TOP), size=(0.235, 0.300, GUARD_Z[1] + 0.008), meshes=tuple(meshes),
                    colliders=tuple(cols))


OBJECTS = [_table(), _bit(), _board()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.grab = False  # irreversible: the board met the cutter while not braced on the pin and turning about it
        self.hands = False  # irreversible: a gripper's shape touched the 3 in zone round the bit
        self.cutting = False  # the board is in the cutter after an entry made braced on the pin
        self.braced_prev = False  # braced at the last step
        self.started = False  # the curve reached the bearing in a cut begun on the pin
        self.angle = None  # the bearing's contact angle on the curve at the last step (None: not on the bearing)
        self.routed = np.zeros(N_BINS, bool)  # the curve's 2 mm stretches routed to full width


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
    """Each control step (a yamkit step hook): the cutter's force (cut_load) put on the board's body, at its center
    of mass as a force and a turning moment, held until the next step."""
    if "board" not in objects:
        return
    import torch

    J = _joints(objects)
    cut_load(J)
    f, c = _C.force, _C.at
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


def _dbg(J, ang, objects=None):
    """ROUTER_DEBUG=1: print the board's pose, the bearing contact, the routed length and the cutter's force every 20
    steps, and every step the tool points (to measure the hands' forces from the arms' give afterwards)."""
    if not os.environ.get("ROUTER_DEBUG"):
        return
    _DBG["n"] += 1
    if objects is not None:
        tp = [np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects]
        if len(tp) == 2:
            f = cut_force_now()
            print(f"router tcp {_DBG['n']} " + " ".join(f"{v:.5f}" for v in (*tp[0], *tp[1])) +
                  f" turn {J['board_turn']:.5f} sl {J['board_x']:.5f} {J['board']:.5f} cut {f[0]:.3f} {f[1]:.3f}",
                  file=sys.stderr, flush=True)
    if _DBG["n"] % 20 == 0:
        f = cut_force_now()
        print(f"router dbg {_DBG['n']}: turn {math.degrees(J['board_turn']):.1f} slides {J['board_x']*100:.2f} "
              f"{J['board']*100:.2f} cm; pin gap {pin_gap(J)*1000:.1f} mm; bit dist {bit_dist(J)*1000:.1f} mm; "
              f"on bearing {ang is not None}; routed {routed_cm():.1f} cm; force ({f[0]:.2f}, {f[1]:.2f}) N; "
              f"grab {_H.grab} hands {_H.hands}; up {_LIFT['z']:.2f} mm, tipped {_LIFT['tilt']:.2f} deg", file=sys.stderr, flush=True)


_LIFT = {"z": 0.0, "tilt": 0.0}


def _update(objects):
    global _H
    try:
        _LIFT["z"], _LIFT["tilt"] = board_rest(objects)
    except Exception:
        pass
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    J = _joints(objects)
    now_braced = braced(J)
    in_cutter = bit_dist(J) < RC
    if in_cutter and not _H.cutting:
        if now_braced and _H.braced_prev:
            _H.cutting = True  # entered braced on the pin: the only motion there is turning about it
        else:
            _H.grab = True  # the cutter's first bite on an unbraced board: the proxy for a grab and throw
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
    _dbg(J, ang, objects)
    if in_zone(objects):  # hands near the bit
        _H.hands = True
    _draw(J)


# The 3 in zone round the bit, as a shape for gk.touching: a solid cylinder about the bit's axis, 7.6 cm in radius,
# from the table top to HANDS_LOW above it. A gripper is in it when its own shape (gk.hand_boxes) touches it.
ZONE = gk.Ring(0.0, HANDS, 0.0, HANDS_LOW)
_ZONE_AT = ObjectState(pos=np.array([BIT[0], BIT[1], TOP]), quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]),
                       lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=np.array([BIT[0] - HANDS, BIT[1] - HANDS, TOP]),
                       high=np.array([BIT[0] + HANDS, BIT[1] + HANDS, TOP + HANDS_LOW]))


def in_zone(objects):
    """Grippers whose own shape (palm, fingers, pads, tips at their present opening) touches the 3 in zone."""
    return gk.touching({**objects, "bit_zone": _ZONE_AT}, "bit_zone", ZONE, tol=0.0)


def routed_cm():
    return 100.0 * BIN * int(_H.routed.sum())


# A hand on the board (gk.touching): a gripper's own shape (palm and both fingers, boxes fitted to the YAM's collision
# meshes, at their present opening) within gk.CONTACT (2 mm: the solver's resting gap plus the box fit) of the board.
# The board, in its own frame (z at mid-thickness): the body behind the curve as one box, and the curved cap as 2.5 mm
# strips, each out to the curve at the strip's middle (within 0.6 mm of the true curve).
_X_IN = float(C_L[0] + R * math.cos(ALPHA))  # where the curve meets the ends
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
    """Grippers touching the board (palm, finger or pad within 2 mm of it). Computed once per step."""
    if _TOUCH["objs"] is not objects:
        objs = {**objects, "board": board_state(_joints(objects))}
        near = gk.touching(objs, "board", _BOUND)  # the board's bounding box first: a hand clear of it is clear
        _TOUCH.update(objs=objects, on=gk.touching(objs, "board", BOARD_SHAPE, grippers=near) if near else ())
    return _TOUCH["on"]


def _released(objects):
    """No gripper touching the board: nobody still holding or pushing it."""
    return not on_board(objects)


def _past(J):
    """The board clear of the cutter by 5 mm and pushed past the bit: its trailing end on the outfeed (left) side."""
    trail = [to_world(J, OUTLINE[0])[1], to_world(J, OUTLINE[-1])[1]]  # the trailing end's two corners
    return bit_dist(J) >= RC + CLEAR and min(trail) > BIT[1]


def _job(objects):
    J = _joints(objects)
    if _H.grab or _H.hands:
        return 0.0  # a first bite off the pin (the proxy for a grab), or a hand went near the bit
    bins = int(_H.routed.sum())
    if not _released(objects):
        return min(_routed_score(J, bins), 0.75)  # a hand still on the board: at most 0.75
    return _routed_score(J, bins)


def _routed_score(J, bins):
    if bins >= WHOLE and _past(J):
        return 1.0  # the whole curve routed and the board fed on past the bit
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
    """The whole curve routed, the board pushed past the bit and let go (where the arms end is not scored). Called
    first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0, irreversibly, if the board ever met the cutter (any part of it within the cutter's 15.9 mm radius) other
    than by being turned into it about the starting pin while braced on it (within 1.5 mm), the curve not yet on the
    bearing: a board fed into the bit off the pin, or slid back into it after leaving it, is scored as a grab. 0,
    irreversibly, if any part of a gripper's own shape (palm, finger bodies, pads, tips, at their present opening)
    touched the zone within 7.6 cm (3 in) of the bit's axis and 5 cm of the table. At most 0.75 while a gripper
    touches the board (its palm or fingers within 2 mm of it). Otherwise: routed length is the curve's stretch the
    bearing passed over (within 1.5 mm of it) in cuts each begun on the pin, fed right to left. 1.0 with the whole
    curve routed (every 2 mm stretch but the one at each end) and the board 5 mm clear of the cutter with its
    trailing end past the bit; 0.75 with the whole curve but the board left on or before the bit, or with 18 cm
    routed; 0.5 with 9 cm routed; 0.25 with the curve brought onto the bearing from the pin; 0 otherwise. Where the
    arms end is not scored. Possible scores: 0, 0.25, 0.5, 0.75, 1."""
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


# --- The scripted demo (both arms carry the board to the pin, the left pivots it onto the bearing, the right takes it
# over, then both feed it round the bearing).
TIP = PAD_TIP_PAST_TCP  # the tool point is this far above the pad tips
PZ = TOP + GAP + T + TIP - 0.004  # the pads 4 mm into the board's top: pressing on it
PZ_FIRM = TOP + GAP + T + TIP - 0.020  # aimed 20 mm in: a firm press (about 5-8 N, the arm giving 2.3-3.8 mm/N
#                                       downward there), so the pads' grip carries the 3-4 N push against the cutter
# The arm gives under a sideways push at the pads (J K^-1 J^T with the YAM's 80 and 10 N*m/rad servos, at the pads
# over the board): 6.2-7.6 mm per newton (here 6.7-7.1), mostly in the wrist. A hand that pushes the board with a few newtons must be
# aimed that far past where the pads are; aimed at the pads, it holds nothing.
GIVE = np.array([[6.8, -0.8], [-0.8, 7.0]]) * 1e-3  # m/N, world x and y


def _board_world(turn, slides, p):
    return PIN + _rot(turn) @ (np.asarray(p, float) + np.asarray(slides, float))


def _point_at(turn, slides, p, z=PZ):
    w = _board_world(turn, slides, p)
    return (float(w[0]), float(w[1]), z)


def _press_at(turn, slides, z=PZ):
    return _point_at(turn, slides, PRESS_PT, z)


def _lead(turn, force):
    """Where to aim the hand, past the pads (board frame, m), for it to push the board with `force` (board frame, N)."""
    return _rot(-turn) @ GIVE @ _rot(turn) @ np.asarray(force, float)


# the pads press on the board's middle along its length, 1.2 cm behind its center (8.7 cm in from the top of the
# curve): a push there barely turns the board (a board pushed at one point swings until its center trails behind)
PRESS_PT = np.array([CENTRE[0] - 0.012, CENTRE[1]])


def _feed_slides(phi, into=0.0005, turn=0.0):
    """The slides (at board turn `turn`) that put the curve's contact with the bearing at angle `phi` on the curve,
    `into` m into the bearing."""
    c_world = BIT - (R + RB - into) * np.array([math.cos(phi + turn), math.sin(phi + turn)])
    return _rot(-turn) @ (c_world - PIN) - C_L


PHI0 = math.atan2(*(BIT - PIN - C_L)[::-1])  # where the bearing meets the curve in the braced pose at turn 0
LEAD = PHI0 - ALPHA  # the turn whose braced pose puts the bearing right at the curve's leading end (-7.8 deg)
BRACED_LEAD = _rot(-LEAD) @ C_L - C_L  # the slides of that pose: the board 4.1 cm further along the pin

# The pivot. Nothing but friction and the hand holds the board on the pin. The pads press 13 cm from the pin toward
# the trailing end, 85 deg in from the edge's tangent there, and push almost straight at the pin (a push through the
# pin does not turn the board, so it holds as the board turns), 4 N onto it; pushed 60 deg in (2 N along the pin), the
# board slid 6.5 mm along the pin before it met the cutter. Turning the board also takes a push round the pin against
# the table's friction (about 1.8 N at the pads). From where the cutter starts to cut, the pads also push the board on
# along the pin toward its leading end, rising to PIVOT_ALONG at the bearing, against the cutter's first bite (about
# 2 N toward the trailing end), which would otherwise draw the curve's end off the bearing.
PIVOT_R = 0.130  # m: the pads' distance from the pin (at 11 cm a finger reached the 3 in zone)
PIVOT_IN = math.radians(85.0)  # the push's angle in from the edge's tangent at the pin (toward the pin)
PIVOT_F = 3.5  # N
PIVOT_ALONG = 1.5  # N
# Once the curve touches the bearing, the board can still roll between pin and bearing (the bearing rolls freely):
# the cutter's push along the curve drives that roll toward the trailing end, which runs the curve's leading end off
# the bearing. The pads' push along the pin (the turning push, which here points along the pin, plus PIVOT_ALONG,
# 3.5 N together) holds it there, with the pin's and the table's friction; it is kept through the hold on the bearing
# (dropped there, the board rolled off at once). The whole push is 5.1 N, under a 6 N press.
# The pivot is planned from the braced pose the two-hand carry leaves the board in, in the sim: turned 15 deg (1 deg
# past the plan), 2 mm on along the pin, where a pivot brings the bearing onto the curve 5 mm in from its leading end.
PIVOT_POSE = (math.radians(15.0), np.array([0.0068, -0.0355]))
# The hand swings on 1.5 deg past the turn that puts the curve on the bearing and holds there, so the curve is on the
# bearing before the pads lift. Free on the pin, the board can still roll between pin and bearing: a larger overshoot
# (6.8 deg, 1.8 cm of the hand's give) wound up enough push to roll the curve's leading end off the bearing.
PIVOT_OVER = math.radians(1.5)
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
# Between them the hands push the board on along the curve against the cutter's 3.3 N and the table's friction, and
# 0.5 N onto the bearing, and turn it as a pair: the trailing hand pushes toward the curve and the leading hand draws
# back toward the robot, a couple that balances the pushes about the bearing. Each hand's share is the smallest that
# does it (FEED_PADS, below, for where they press).
# The board is turned about its curve's center, which stays put (the bearing on the curve, the feed a turn of the
# board). Over the first 3 cm the center is moved 4 deg
# round the bit, which slides the board 1.4 cm on along the feed: the curve comes 3 mm off the pin, so it does not rub
# on it, and the board does not jolt when the pin runs off the curve's trailing end.
PIVOT_END = (math.radians(-6.6), PIVOT_POSE[1] + np.array([0.0006, -0.0010]))  # on the bearing after the pivot (the
#                       board stops within 0.4 deg of where the bearing first touches, slid 1-2 mm along the pin)
_C_END = PIN + _rot(PIVOT_END[0]) @ (C_L + PIVOT_END[1])
BETA_ON = math.atan2(*(BIT - _C_END)[::-1])  # the direction from the curve's center to the bit's axis
PHI_START = BETA_ON - PIVOT_END[0]  # the bearing's contact angle on the curve then (23.4 deg: the leading end routed)
PHI_END = -ALPHA - math.radians(7.5)  # the bearing 3.9 cm round past the curve's trailing end (the board lags the hands
#                                        by a few mm: planned 1.8 cm past, the bearing left the curve 6 mm short of its end)
OFF_PIN = math.radians(-4.0)
OFF_PIN_RUN = 0.030  # m of curve
PAD_IN = 0.020  # m: the pads' middle at least 2 cm in from the back edge, so both pads (2 cm across the closing axis)
#                are on the board with 1 cm to spare (at 0.5 cm in, half of each pad overhung the edge and slipped off)
CARRY_L = np.array([BACK + PAD_IN, ENDS[1] - 0.030])  # where the hands carry the board to the pin (board frame)
CARRY_R = np.array([BACK + PAD_IN, ENDS[0] + 0.030])
# The feed's pads, in two stretches, the hands changing their places once, one at a time, with the board at rest
# (the cutter pushes only while it cuts): where the hands' pushes are smallest. Each hand pushes its pads with its
# share by the pads' grip (friction) under an 8 N press, the share at most 5.0 N, so the pads need a grip of at least
# 0.63 times the press. (With one pair of places for the whole feed the larger push would be 5.7 N; at a 6 N press
# with the pads 2 cm in at both ends the grip needed is 0.84 of the press, more than the sim's pads have.)
PHI_SPLIT = math.radians(-5.0)  # where the hands change places (the bearing 5 deg past the curve's middle)
FEED_PADS = (  # (from phi, to phi, left pads, right pads, right hand's yaw, squeeze N), board frame
    (None, PHI_SPLIT, np.array([BACK + 0.020, ENDS[0] + 0.225]), np.array([BACK + 0.060, ENDS[0] + 0.025]),
     math.radians(140.0), 1.0),
    (PHI_SPLIT, None, np.array([BACK + 0.080, ENDS[0] + 0.225]), np.array([BACK + 0.020, ENDS[0] + 0.050]),
     math.radians(110.0), 0.0))
# The squeeze: planned with equal shares (4.7 N each), the first stretch ran with the trailing hand pushing 1.1-1.2 N
# more along the line between the pads and the leading hand that much less (measured from the arms' deflection,
# 5.2-5.9 N against 3.9-4.0 N); the plan leans 1 N the other way there so the run shares the push evenly.
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
FEED_BACK = 3.3  # N: the cutter's push back along the curve to plan for (Curti et al., above: 2.9 N at the demo's
#                  feed, 3.3 N at 1 m/min; the board's speed jumps as it breaks free of the table's friction)
FEED_ON = 0.5  # N: the hands' push onto the bearing, besides the cutter's own 1.9 N pull
PRESS_N = 8.0  # N: each hand's press down on the board in the feed, so its pads' grip carries its share of the push
#                (each newton of press adds 0.25 N of table friction, so pressing harder also costs push)
BRACE_PRESS = 3.0  # N: the press while sliding the board onto the pin
PIVOT_PRESS = 6.0  # N: the press in the pivot
FEED_N = 600  # 0.1 s steps over the feed: 29 cm of curve in 60 s, 0.3 m/min
# On past the bit: the board drawn 2 cm back from the bit and carried 9.5 cm on, so its trailing end is past the bit
END_SHIFT = np.array([-0.020, 0.095])


def _feed_pose(phi):
    """(turn, slides, beta) with the bearing on the curve at angle phi and the curve's center on the line from the
    bit's axis at angle beta (moved off the pin over the first OFF_PIN_RUN of the feed)."""
    u = min(max((PHI_START - float(phi)) * R / OFF_PIN_RUN, 0.0), 1.0)
    beta = BETA_ON + OFF_PIN * u * u * (3 - 2 * u)
    th = beta - float(phi)
    c = BIT - (R + RB - 0.0003) * np.array([math.cos(beta), math.sin(beta)])
    return th, _rot(-th) @ (c - PIN) - C_L, beta


def _hand_forces(phi, pad_l=None, pad_r=None, squeeze=0.0):
    """The two hands' pushes (world, N) at the feed pose for phi: together they balance the cutter's push back and
    the table's friction and press the curve onto the bearing, with no turn about the contact; of all such pairs, the
    one whose larger push is smallest, then `squeeze` N more toward each other along the line between the pads (the
    leading hand pushing on more, the trailing hand less: the free choice, which balances nothing else)."""
    th, s, beta = _feed_pose(phi)
    nu = np.array([math.cos(beta), math.sin(beta)])  # toward the bit's axis
    tau = np.array([-nu[1], nu[0]])  # the feed's direction at the contact
    k = BIT - RB * nu
    pad_l = PAD_L if pad_l is None else pad_l
    pad_r = PAD_R if pad_r is None else pad_r
    a, b = _board_world(th, s, pad_l) - k, _board_world(th, s, pad_r) - k
    net = FEED_BACK * tau + FEED_ON * nu
    # the table's friction against the board's motion: under each hand (its press) and under the middle (the weight)
    th2, s2, _ = _feed_pose(phi - 1e-4)
    mom = 0.0
    for p, n in ((pad_l, PRESS_N), (pad_r, PRESS_N), (np.array(CENTRE), BOARD_MASS * G)):
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


def demo():
    """Both hands press near the board's ends and carry it from where it lies, square, to the starting pin, turned 14
    deg, at the spot where pivoting it about the pin brings the curve's leading end onto the bearing. The left hand
    moves its pads behind the pin and, pushing the board onto the pin and along it, swings it about the pin until the
    curve rides the bearing, and holds it there. The right hand takes the board near its trailing end before the left
    lets go; the left presses near the leading end; and together they turn the board round the bearing over the whole
    curve (changing places once, one hand at a time, with the board at rest), off the pin, and carry it on past the
    bit; lift off and go home."""
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

    th_cut = next(a for a in np.arange(th_b, th1, -0.0005)  # where the cutter starts to cut
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
    n = int(round(120 * (th_b - th1) / (th0 - LEAD + math.radians(1.0))))  # the same pace as a swing to 1 deg past
    for k in range(1, n + 1):  # pivot about the pin
        th = th_b + (th1 - th_b) * k / n
        g = min(1.0, k / 6)
        left.append(Waypoint(0.1, pv_aim(th, push * PIVOT_F + g * f_turn * cw + f_along(th)), C,
                             "pivot about the pin into the bit"))
    on_b = pv_aim(th1, push * PIVOT_F + f_turn * cw + f_along(th1))
    left.append(Waypoint(1.0, on_b, C, "on the bearing"))
    # The board is passed from hand to hand, never let go: lifted, the left hand left the board free to roll between
    # pin and bearing under the cutter's push, off the curve's leading end. The right presses near the trailing end,
    # 9 cm in from its feed spot (at its feed spot its wrist came within 8.5 cm of the left's), and pushes its share of
    # the feed's start (_hand_forces), which alone holds that roll; the left lifts, crosses high over the bit and
    # presses near the leading end with its share, which also alone holds it; then the right steps back to its spot.
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
    # press and take up their whole share of the push at once (half loaded, the cutter's push can turn the board back
    # about the bearing and onto the pin).
    left += [Waypoint(0.5, (wl[0], wl[1], TOP + GAP + T + TIP - 0.001), C, "pads on the board"),
             Waypoint(0.6, _aim("left", wl, f0[0]), C, "press"),
             Waypoint(0.5, _aim("left", wl, f0[0]), C, "hold the board on the bearing")]
    t_l, t_r = sum(w.duration for w in left), sum(w.duration for w in right)
    right.append(Waypoint(t_l - t_r, _aim("right", wh, f0[1]), C, "hold the board on the bearing"))
    y1 = FEED_PADS[0][4]
    fs = _hand_forces(PHI_START, squeeze=FEED_PADS[0][5])  # the feed's first pushes
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
    phis = np.linspace(PHI_START, PHI_END, FEED_N + 1)
    phi_at = PHI_START  # where the feed has got to
    for n_ph, (a_ph, b_ph, pl, pr, yr, sq) in enumerate(FEED_PADS):
        if n_ph:
            th, s, _ = _feed_pose(phi_at)
            ol, orr = _board_world(th, s, FEED_PADS[n_ph - 1][2]), _board_world(th, s, FEED_PADS[n_ph - 1][3])
            nl, nr = _board_world(th, s, pl), _board_world(th, s, pr)
            yr0 = FEED_PADS[n_ph - 1][4]
            low = TOP + GAP + T + TIP - 0.001
            left += [Waypoint(0.6, _aim("left", ol), C, "stop: ease the push"),
                     Waypoint(0.4, (ol[0], ol[1], above), C, "lift the pads"),
                     Waypoint(0.6, (nl[0], nl[1], above), C, "to its next place"),
                     Waypoint(0.4, (nl[0], nl[1], low), C, "pads on the board"),
                     Waypoint(0.5, _aim("left", nl), C, "press")]
            right += [wy(Waypoint(0.6, _aim("right", orr, yaw=yr0), C, "stop: ease the push"), yr0),
                      wy(Waypoint(1.9, _aim("right", orr, yaw=yr0), C, "hold the board"), yr0),
                      wy(Waypoint(0.4, (orr[0], orr[1], above), C, "lift the pads"), yr0),
                      wy(Waypoint(0.6, (nr[0], nr[1], above), C, "to its next place"), yr),
                      wy(Waypoint(0.4, (nr[0], nr[1], low), C, "pads on the board"), yr),
                      wy(Waypoint(0.5, _aim("right", nr, yaw=yr), C, "press"), yr)]
            left.append(Waypoint(1.9, _aim("left", nl), C, "hold the board"))
            fl, fr = _hand_forces(phi_at, pl, pr, sq)
            left.append(Waypoint(0.6, _aim("left", nl, fl), C, "push again"))
            right.append(wy(Waypoint(0.6, _aim("right", nr, fr, yaw=yr), C, "push again"), yr))
        for phi in phis[1:]:
            if (a_ph is not None and phi > a_ph) or (b_ph is not None and phi < b_ph):
                continue
            th, s, _ = _feed_pose(phi)
            fl, fr = _hand_forces(phi, pl, pr, sq)
            phi_at = float(phi)
            left.append(Waypoint(0.1, _aim("left", _board_world(th, s, pl), fl), C, "feed round the bearing"))
            right.append(wy(Waypoint(0.1, _aim("right", _board_world(th, s, pr), fr, yaw=yr), C,
                                     "feed round the bearing"), yr))
    # On past the bit, the hands only carrying the board, then the press eased off and both hands lifted straight up.
    th_e, s_e, _ = _feed_pose(PHI_END)
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
