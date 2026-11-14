"""A router table with a bearing-guided rabbeting bit and a steel starting pin in its insert plate, and a cherry board
(25 x 15 cm, 1.27 cm thick) with one long edge band-sawn to a gentle convex curve, lying on the table top on the
infeed (right) side of the bit, its curved edge toward the bit.

To rout a curved edge on a router table, where there is no fence to steady the work, a woodworker first braces the
board against the starting pin, a steel pin in the insert plate a few centimeters from the bit, then pivots it about
the pin into the spinning bit until its edge rides the bit's bearing. Only then does the board come off the pin and
follow the bearing round, fed right to left against the bit's rotation. Fed straight into the bit without the pin,
the cutter's first bite on an unsupported edge can grab the board and throw it, or pull the hands toward the cutter.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import pivotkit as pv  # noqa: E402
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
GAP = 0.001  # m: the board's underside rides 1 mm above the table on its joints (pivotkit carries it flat)
BOARD_MASS = 0.22  # kg: 25 x 15 x 1.27 cm of cherry (0.56 g/cm3) less the bow
ALPHA = math.asin(HALF / R)  # the half-angle the curve spans about its center (24.6 deg)
ARC = 2 * ALPHA * R  # m: the curve's length, 25.8 cm
Z_MID = GAP + T / 2  # the board's mid-thickness above the table (its frame's height)
SLIDE_DAMP = 4.0  # N*s/m: 220 g of board sliding on a waxed table top (friction as damping: 0.2 N at 5 cm/s)
RG2 = (0.25 ** 2 + 0.15 ** 2) / 12  # m^2: the board's radius of gyration squared about its center


def _circle_meet(c0, r0, c1, r1, pick):
    """The intersection of two circles (centers c0, c1, radii r0, r1); `pick` chooses between the two."""
    d = np.linalg.norm(c1 - c0)
    a = (r0 * r0 - r1 * r1 + d * d) / (2 * d)
    h = math.sqrt(max(r0 * r0 - a * a, 0.0))
    mid = c0 + a * (c1 - c0) / d
    perp = np.array([-(c1 - c0)[1], (c1 - c0)[0]]) / d
    return min((mid + h * perp, mid - h * perp), key=pick)


# The board's frame (pivotkit): its origin is the pin's center with the board in its "braced" pose, where the curve
# touches both the pin and the bearing, its long sides along y. The curve's center in that frame:
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
HANDS = 0.076  # m: no tool point within 3 in (7.6 cm) of the bit's axis while the pads are low (router-table manuals'
#                three-inch rule for hands near a bit). This is a hazard zone, not a contact proxy: a hand that close to
#                a spinning bit is the hazard itself, since a slip or a grab carries it into the cutter.
HANDS_LOW = 0.05  # m: "low" is pad tips within 5 cm of the table top (the bit and the board's top are below 1.4 cm)
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
    """The board's drag on the table as joint damping. The chain's turn is about the pin, not the board's center, so
    on the pin (slides locked) its damping is the slide damping times (the center's distance from the pin squared
    plus the board's radius of gyration squared), what dragging the board round the pin costs. Off the pin, per-joint
    damping can't hold the coupling between that turn and the slides, so a push at the board's center would swing it
    round the far-off pin instead of sliding it; the turn is damped 10 times harder there so pushes slide the board."""
    centre = to_world(J, CENTRE)
    turn = SLIDE_DAMP * (float(np.sum((centre - PIN) ** 2)) + RG2)
    return SLIDE_DAMP, turn if braced(J) else FREE_TURN * turn


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
    last joints (to tell the board's speed)."""

    def __init__(self):
        self.episode = kit.episode()
        self.depth = np.zeros(len(_CUT_A))
        self.key = None
        self.prev = None  # the deepest-cutting bin's world point at the last step
        self.force = np.zeros(2)  # N, world (x, y), smoothed
        self.at = BIT.copy()  # where it acts (world)
        self.load = (0.0, 0.0, 0.0)


_C = _Cut()


def cut_load(J):
    """The cutter's force on the board as efforts on the board's joints (turn about the pin, x slide, y slide), and
    its bookkeeping. The cutter removes the board's curve inside its radius; each bin's fresh depth this step is what
    it cuts. The force acts at the deepest fresh bin along the cutting edge's motion there (the bit turns
    counterclockwise seen from above), which is against a right-to-left feed and with a climb cut, plus a pull toward
    the bit's axis that the bearing takes. Called once per control step per joint; it computes once per step."""
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
    else:
        target = np.zeros(2)
        _C.prev = None
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


PIVOT = pv.Pivot("board", engaged=braced, free="slide", resist=drag, load=cut_load)

# The board starts on the table to the right of the bit, turned 14 deg counterclockwise from the braced pose (its
# leading end swung back toward the robot) and 1.5 cm nearer the robot and 2.5 cm further right: 2.4 cm off the pin.
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
    """The board on pivotkit's planar chain (frame: origin at the pin's center in the braced pose, z at mid-board)."""
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
    meshes = [kit.box((0.0, 0.0, 0.0008), (0.235, 0.300, 0.0010), PLATE),  # a 9.25 x 11.75 in phenolic plate
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


_H = _History()


def _joints(objects):
    return {n: kit.joint(objects, f"router_table/{n}").pos for n in ("board_turn", "board_x", "board")}


def _bin(ang):
    return min(max(int((ang + ALPHA) * R / BIN), 0), N_BINS - 1)


_DBG = {"n": 0}


def _dbg(J, ang):
    """ROUTER_DEBUG=1: print the board's pose, the bearing contact, the routed length and the cutter's force every 20 steps."""
    if not os.environ.get("ROUTER_DEBUG"):
        return
    _DBG["n"] += 1
    if _DBG["n"] % 20 == 0:
        f = cut_force_now()
        print(f"router dbg {_DBG['n']}: turn {math.degrees(J['board_turn']):.1f} slides {J['board_x']*100:.2f} "
              f"{J['board']*100:.2f} cm; pin gap {pin_gap(J)*1000:.1f} mm; bit dist {bit_dist(J)*1000:.1f} mm; "
              f"on bearing {ang is not None}; routed {routed_cm():.1f} cm; force ({f[0]:.2f}, {f[1]:.2f}) N; "
              f"grab {_H.grab} hands {_H.hands}", file=sys.stderr, flush=True)


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
    _dbg(J, ang)
    now = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    for g in now:  # hands near the bit
        low = float(np.asarray(objects[g].low)[2]) - TOP < HANDS_LOW
        if low and np.linalg.norm(now[g][:2] - BIT) < HANDS:
            _H.hands = True
    _draw(J)


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
    if not _released(objects):
        return 0.0  # nothing counts while a hand is still touching the board
    bins = int(_H.routed.sum())
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
    irreversibly, if a tool point came within 7.6 cm (3 in) of the bit's axis with the pads within 5 cm of the
    table. 0 while a gripper touches the board (its palm or fingers, at their present opening, within 2 mm of
    it). Otherwise: routed length is the curve's stretch the bearing passed over (within 1.5 mm of it) in cuts each
    begun on the pin, fed right to left. 1.0 with
    the whole curve routed (every 2 mm stretch but the one at each end) and the board 5 mm clear of the cutter with its
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


# --- The scripted demo (left arm; the right arm stays home).
# --- The scripted demo (left arm; the right arm stays home).
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

# The pivot. The cutter's first bite pushes the board about 2 N toward its trailing end along the pin, and once the
# curve reaches the bearing the board is off the brace with only the hand to hold it there. So the pads push it on
# toward the leading end and onto the pin through the whole pivot: they press 9 cm from the pin toward the trailing
# end, 25 deg in from the edge's tangent there, and push straight at the pin (a push through the pin does not turn
# the board, so it holds as the board turns), 3.2 N: 2.9 N along the pin toward the leading end, 1.4 N onto it.
PIVOT_R = 0.090  # m: the pads' distance from the pin
PIVOT_IN = math.radians(25.0)  # the push's angle in from the edge's tangent at the pin (toward the pin)
PIVOT_F = 3.2  # N
# The feed. Fed right to left, the cutter pushes back on the curve at the bearing, about 3.3 N along it, and pulls
# it about 2 N toward the bit's axis, which the bearing takes. With nothing else to steady the board's turn, the
# pads' push must point through that contact, or the board swings about the pads. Where the board reaches far enough
# behind the stretch on the bearing (the hands kept 9 cm from the bit's axis), the pads press there and push toward the
# contact, 1.25 times the cutter's 3.3 N along the curve; the rest of their push presses the curve onto the bearing.
# Toward the trailing end there is no board that far behind, so the pads press ahead of the contact, near the curve,
# and pull the board on, straight away from the contact, 3.3 N along the curve: their pull off the bearing is then
# under 0.45 of that, well under the cutter's own pull onto it (0.66 of its push back).
FEED_FX = 1.25 * 3.3  # N: the push along the curve, from behind
FEED_PULL = 3.3  # N: the pull along the curve, from ahead
FEED_STRETCHES = 7
HAND_PAD = 0.090  # m: the pads' least distance from the bit's axis while feeding (the hazard zone is 7.6 cm)
HAND_AIM = 0.080  # m: and the hand's aim's (the pads plus the lead), should the pads slip


def _contact(phi):
    """The curve's point at angle phi (board frame), its tangent toward the leading end and its outward normal."""
    ph = max(float(phi), -ALPHA)
    return (C_L + R * np.array([math.cos(ph), math.sin(ph)]), np.array([-math.sin(ph), math.cos(ph)]),
            np.array([math.cos(ph), math.sin(ph)]))


def _feed_force(pad, phi, pull):
    """The pads' force on the board (board frame, N): along the line from the pads through the contact."""
    k_pt, along, _ = _contact(phi)
    u = (pad - k_pt) if pull else (k_pt - pad)
    u = u / np.linalg.norm(u)
    return u * (FEED_PULL if pull else FEED_FX) / float(u @ along)


def _feed_pad(part):
    """The pads for a stretch of contact angles (pads, pull): on the board, 2.5 cm in from its ends and back edge and
    2 cm in from the curve or more, HAND_PAD from the bit's axis through the stretch. Pushing from behind if that
    puts 0.6 of the push along the curve or more all through the stretch; else pulling from ahead, the pull off the
    bearing smallest."""
    push = pull = None
    for a_p in np.linspace(-ALPHA + 0.025 / R, ALPHA - 0.025 / R, 120):
        for h in np.arange(0.02, 0.121, 0.005):
            pad = C_L + (R - h) * np.array([math.cos(a_p), math.sin(a_p)])
            if pad[0] < BACK + 0.025:
                continue
            w_push, w_pull = 1.0, 0.0
            for phi in part:
                k_pt, along, out = _contact(phi)
                bit = k_pt + RB * out
                u = (k_pt - pad) / np.linalg.norm(k_pt - pad)  # toward the contact
                w_push = min(w_push, float(u @ along))
                # pulling: the pull off the bearing over the pull along the curve (u points back at the contact)
                w_pull = max(w_pull, float((u @ out) / -(u @ along)) if -(u @ along) > 0.05 else 9.0)
                near = np.linalg.norm(pad - bit) < HAND_PAD
                if near or (w_push > 0.05 and np.linalg.norm(pad + GIVE @ _feed_force(pad, phi, False) - bit) < HAND_AIM):
                    w_push = -1.0
                if near or (w_pull < 9.0 and np.linalg.norm(pad + GIVE @ _feed_force(pad, phi, True) - bit) < HAND_AIM):
                    w_pull = 9.0
            if push is None or w_push > push[0]:
                push = (w_push, pad)
            if pull is None or w_pull < pull[0]:
                pull = (w_pull, pad)
    if push[0] >= 0.6:
        return push[1], False
    return pull[1], True


CENTER_ON = PIN + _rot(LEAD) @ (C_L + BRACED_LEAD)  # the curve's center (world) with the curve on pin and bearing


def _wheel(phi):
    """The board's (turn, slides) with the curve's center at CENTER_ON and the bearing on the curve at angle phi:
    the board turned about that center. Capped at the board's 0.79 rad turn (its joint stops at 0.8)."""
    th = min(LEAD + ALPHA - float(phi), 0.79)
    return th, _rot(-th) @ (CENTER_ON - PIN) - C_L


SWEEPS = (1.6, 1.5, 1.4, 1.3, 1.2)  # each feed stretch's hand path over the stretch: the longest that suits the pads


def _stretch(part):
    """(hand path's contact angles, pads, pull) for a stretch: the longest sweep in SWEEPS whose pads push from behind
    with 0.6 or more of the push along the curve and at most 5.5 N in all, or pull from ahead with their pull off the
    bearing at most 0.45 of the pull along it (the cutter's own pull onto it is 0.66); else the shortest."""
    span = float(part[0] - part[-1])
    for sw in SWEEPS:
        sweep = np.linspace(part[0], part[0] - sw * span, int(round(len(part) * sw)))
        pad, pull = _feed_pad(sweep)
        ok = True
        for phi in sweep:
            f = _feed_force(pad, phi, pull)
            _, along, out = _contact(phi)
            if (pull and -(f @ out) > 0.45 * (f @ along)) or (not pull and np.linalg.norm(f) > 5.5):
                ok = False
                break
        if ok:
            return sweep, pad, pull
    return sweep, pad, pull
HIGH = TOP + 0.09  # pad tips 6.5 cm above the table: well clear of the 5 cm hand rule


def _over(a, b, C):
    """Waypoints to move the hand from a to b (both just above the board): straight if the path stays 10 cm from the
    bit's axis, else up to HIGH, across and down, so it never passes low near the bit."""
    a2, b2 = np.array(a[:2]), np.array(b[:2])
    d = b2 - a2
    u = float(np.clip(np.dot(BIT - a2, d) / max(float(d @ d), 1e-12), 0.0, 1.0))
    if np.linalg.norm(a2 + u * d - BIT) >= 0.10:
        return []
    return [Waypoint(0.4, (a[0], a[1], HIGH), C, "up, clear of the bit"),
            Waypoint(0.8, (b[0], b[1], HIGH), C, "across, above the bit's 3 in circle"),
            Waypoint(0.4, (b[0], b[1], b[2]), C, "down")]


def demo():
    """Left arm: press the pads on the board and slide it onto the starting pin at the spot where pivoting it about
    the pin brings the curve's leading end onto the bearing; move the pads behind the pin and, pushing the board on
    along the pin and onto it, swing it about the pin until the curve rides the bearing; then feed it right to left
    round the bearing over the whole curve and on past the bit, pushing from behind toward the contact with the
    bearing, then near the trailing end pulling from ahead, moving the pads along the board between seven stretches; lift off and go home. One braced feed routes the whole curve."""
    C, O = FINGER_CLOSED, FINGER_OPEN
    s0 = np.array(START[:2])
    th0 = START[2]
    braced_s = BRACED_LEAD
    into_pin = _rot(-th0) @ (np.array([0.0015, 0.0025]))  # 3 mm on toward the pin (its collider stops the board)
    left = [Waypoint(2.2, _press_at(th0, s0, TOP + 0.10), C, "over the board"),
            Waypoint(1.2, _press_at(th0, s0, PZ + 0.01), C, "down onto it"),
            Waypoint(0.6, _press_at(th0, s0), C, "press")]
    n = 8
    for k in range(1, n + 1):  # slide it onto the pin
        s = s0 + (braced_s + into_pin - s0) * k / n
        left.append(Waypoint(0.5, _press_at(th0, s), C, "brace the board against the starting pin"))
    left.append(Waypoint(0.5, _press_at(th0, braced_s + into_pin), C, "settle on the pin"))
    lift = PZ + 0.015
    th1 = LEAD - math.radians(1.0)  # a degree past the braced pose: into the bearing, which stops it
    pin_b = -braced_s  # the pin's center in the board's frame
    out = (pin_b - C_L) / np.linalg.norm(pin_b - C_L)  # the edge's outward normal where it bears on the pin
    along = np.array([-out[1], out[0]])  # the edge's tangent there, toward the leading end
    push = math.cos(PIVOT_IN) * along + math.sin(PIVOT_IN) * out
    pv_pad = pin_b - PIVOT_R * push  # the pads, so their push toward the pin is along `push`
    left += [Waypoint(0.5, _press_at(th0, braced_s, lift), C, "lift the pads"),
             Waypoint(1.0, _point_at(th0, braced_s, pv_pad, lift), C, "behind the pin"),
             Waypoint(0.6, _point_at(th0, braced_s, pv_pad, PZ_FIRM), C, "press")]
    for k in range(1, 9):  # load the push gradually, the board still against the pin
        left.append(Waypoint(0.1, _point_at(th0, braced_s, pv_pad + _lead(th0, push * PIVOT_F * k / 8), PZ_FIRM), C,
                             "push along the pin and onto it"))
    # The pivot and the feed are paths of short 0.1 s steps: each waypoint eases in and out, so long ones would move
    # the board in jerks, and a jerk can draw it a millimeter off the pin or off the bearing.
    n = 120
    for k in range(1, n + 1):  # pivot about the pin
        th = th0 + (th1 - th0) * k / n
        left.append(Waypoint(0.1, _point_at(th, braced_s, pv_pad + _lead(th, push * PIVOT_F), PZ_FIRM), C,
                             "pivot about the pin into the bit"))
    left += [Waypoint(0.5, _point_at(th1, braced_s, pv_pad + _lead(th1, push * PIVOT_F), PZ_FIRM), C,
                      "on the bearing"),
             Waypoint(0.5, _point_at(th1, braced_s, pv_pad, lift), C, "lift the pads")]
    # Feed in stretches, the pads moved along the board between them. On the bearing the curve also stays on the pin
    # (the cutter's pull and the pads' push hold it on both), so its center stays put and the board turns about it:
    # the curve runs round between the pin and the bearing like a wheel (seen in the sim, the center within 0.3 mm).
    # Each stretch's hand path sweeps up to 1.6 times the stretch: the board lags the hand (in the sim it turned 0.6 of each
    # sweep, held back by the cutter and by the curve's rub on the pin and bearing), and the next stretch starts from
    # where the board should be. A board that keeps up simply turns further along the bearing.
    phis = np.linspace(ALPHA, -ALPHA - math.radians(3.5), 210)
    last = None
    for k, part in enumerate(np.array_split(phis, FEED_STRETCHES)):
        sweep, pad, pull = _stretch(part)
        th_k, s_k = _wheel(part[0])
        if last is not None:
            left.append(Waypoint(0.4, (last[0], last[1], lift), C, "lift the pads"))
            left += _over((last[0], last[1], lift), _point_at(th_k, s_k, pad, lift), C)
        left += [Waypoint(0.8 if k else 1.0, _point_at(th_k, s_k, pad, lift), C, "move the pads along the board"),
                 Waypoint(0.5, _point_at(th_k, s_k, pad, PZ_FIRM), C, "press")]
        for j, phi in enumerate(sweep):
            th, sl = _wheel(phi)
            f = _feed_force(pad, phi, pull) * min(1.0, (j + 1) / 6)  # loaded over 0.6 s
            last = _point_at(th, sl, pad + _lead(th, f), PZ_FIRM)
            left.append(Waypoint(0.1, last, C, "pull the board round the bearing" if pull else "feed round the bearing"))
        pad_prev, f_prev = pad, f
    # on past the bit: the trailing end off the bearing, then the board drawn 2 cm back from the bit and 3 cm on
    th_e, s_e = _wheel(phis[-1] - math.radians(2.0))
    s_end = s_e + _rot(-th_e) @ np.array([-0.020, 0.030])
    left += [Waypoint(1.0, _point_at(th_e, s_end, pad_prev + _lead(th_e, f_prev), PZ_FIRM), C, "on past the bit"),
             Waypoint(0.8, _point_at(th_e, s_end, pad_prev, TOP + 0.10), C, "lift off"),
             kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    right = [kit.Home(1.0, label="stay home")]
    return {"left": left, "right": right}
