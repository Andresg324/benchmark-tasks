"""A GX200-class single-cylinder short block (68 mm bore, 54 mm stroke), head off, stands in a fixture on the bench with
its bore vertical and its deck 19.75 cm up; the crank is turned 44 degrees past top dead center. A new piston with
three rings is started in the bore: its skirt is in the bore and its ring pack is squeezed inside a pliers-type ring
compressor, a steel band that stands on the deck with its two handles reaching out to the robot's left. "Install the
piston in the bore." An engine builder holds the compressor down flat on the deck with one hand and drives the piston
down through it with the other (in a shop by tapping the crown with a hammer handle). If the compressor rides up off
the deck while a ring is crossing from the band into the bore, that ring can spring out into the gap, catch on the
deck's edge and jam the piston (pushed on, the ring can break). The rod hangs on its pin with its cap off; its big end
lands on the crank journal when the crown is 10.2 mm below the deck.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled engine block articulation; nothing on disk changes)
import guardkit as gk  # noqa: E402  (gripper contact: the hands let go at the end)
from dataclasses import dataclass  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.task_file import TABLE_TOP, ObjectState  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Install the piston in the bore."

# --- The engine (fixed): a GX200-class short block (68 mm bore, 54 mm stroke, 84 mm rod), head off, held in a fixture
# with its bore vertical. Heights are above the deck unless they say "table"; the bore's axis is vertical.
BX, BY = 0.26, -0.17  # the bore's axis
STROKE, ROD, PIN_DOWN = 0.054, 0.084, 0.026  # stroke; rod, pin center to big-end center; pin center below the crown
TDC_GAP = 0.0005  # the crown's depth below the deck at top dead center
CRANK_Z = 0.060  # the crank's axis above the table: the counterweights swing 4.5 cm below it, inside the crankcase
CRANK_DEG = 44.0  # the crank is turned 44 degrees past top dead center (reasons in the README)
DECK = CRANK_Z + STROKE / 2 + ROD + PIN_DOWN + TDC_GAP  # 19.75 cm: the deck's height above the table
_t, _r = math.radians(CRANK_DEG), STROKE / 2
DROP = _r * (1 - math.cos(_t)) + ROD - math.sqrt(ROD ** 2 - (_r * math.sin(_t)) ** 2)  # 9.7 mm below TDC at 44 deg
CASE_TOP = 0.115  # the crankcase's top above the table, where the finned cylinder barrel starts
FLANGE, FLANGE_T = 0.130, 0.015  # the deck flange: 13 cm square, 1.5 cm thick
BARREL_R, FIN_R = 0.047, 0.062  # the barrel and its cooling fins
BORE_R = 0.034  # 68 mm bore
LINER_R = 0.0515  # the liner's flange on the deck
HOLE = 0.036  # half-width of the square opening the frame boxes leave around the bore

# --- The piston (and rod, one sliding part). q = the crown's height above the deck.
PISTON_R = 0.0338  # 67.6 mm across: 0.4 mm skirt clearance
PISTON_H = 0.050  # crown to skirt bottom
RINGS = (0.005, 0.009, 0.013)  # each ring groove's depth below the crown: top compression, second, oil ring
RING_NAMES = ("top compression ring", "second ring", "oil ring")
START = 0.022  # crown 22 mm above the deck: skirt in the bore, all three rings inside the compressor's band
SEAT = -(TDC_GAP + DROP)  # -10.2 mm: the rod's big end (cap off) lands on the crank journal; the piston goes no further
PISTON_MASS = 0.25  # kg, piston, pin and rod

# --- The ring compressor (a free sliding sleeve on the bore's axis). q = its band's lower edge above the deck.
BAND_H = 0.018  # band height: the whole 8 mm ring pack fits inside with the crown 4 mm proud of it at the start
BAND_IN, BAND_OUT = 0.0342, 0.0385  # the band around the rings; a rolled stiffening lip to 4 cm at its top
HANDLE_DIR = np.array([-math.sin(math.radians(20)), math.cos(math.radians(20)), 0.0])  # to the left, 20 deg back
HANDLE_R0, HANDLE_R1 = 0.040, 0.190  # where the handles start and end, from the axis
HANDLE_Z = 0.008  # the handles' axis height above the band's lower edge
PRESS_R = 0.175  # where the demo's hand presses on the handles, from the axis
COMP_MASS = 0.20  # kg
LIFT_MAX = 0.030

# --- The modeled ring entry (reasons in the README). A ring "crossing" is within ZONE above the deck: squeezed in the
# band's lower lip and riding the deck's 1 mm bore chamfer. Each crossing ring pushes outward on the lip's taper, which
# pushes the compressor up (LIFT_N, falling off as it rises, so an unheld band rides up about 7 mm and hangs there) and
# pushes the piston back (RING_PUSH): the driving hand has to push through it.
ZONE = 0.0025  # m
LIFT_N = 6.0  # N up on the compressor while any ring is crossing (its weight is 2 N)
LIFT_K = 600.0  # N/m: the lift falls off as the band rises off the lip; (6 - 2) / 600 = 6.7 mm where it hangs
LIFT_DAMP = 15.0  # N*s/m: about critical for the 200 g band on that spring, so it rises without bouncing
RING_PUSH = 4.0  # N up on the piston while a ring is crossing
DAMP_BAND, DAMP_BORE, DAMP_CROSS = 60.0, 40.0, 200.0  # N*s/m: rings in the band, each ring in the bore, crossing
JAM_HOLD = 20000.0  # N/m: an escaped ring caught on the deck edge holds the piston (1 mm of push = 20 N)

# --- Scoring thresholds (reasons in score() and the README)
LIFT_TOL = 0.001  # m: a gap over 1 mm under the band lets a 1.2 mm-tall ring spring out of it
IN_DEPTH = 0.001  # m: a ring counts as in once it is 1 mm below the deck, past the chamfer, all inside the bore
HOME_TOL = 0.002  # m: the crown within 2 mm of its seat (rod on the journal); less than the 4 mm between grooves
SEATED_TOL = 0.001  # m: the compressor resting on the deck at the end
# Let go at the end: no gripper touching the piston, the band or its handles, judged by the gripper's own shape (palm
# and both fingers at their present opening, gk.touching: within 2 mm, the solver's resting gap plus the box fit). A
# hand hovering over the work more than 2 mm away has let go.

CAST = kit.Look((0.30, 0.33, 0.36), 0.7)  # painted cast iron
DECK_FACE = kit.Look((0.66, 0.67, 0.68), 0.3, metallic=0.9)  # the machined deck
LINER = kit.Look((0.58, 0.59, 0.60), 0.25, metallic=1.0)
BORE_DARK = kit.Look((0.05, 0.05, 0.06), 0.8)
ALU = kit.Look((0.80, 0.80, 0.78), 0.35, metallic=0.8)
CROWN = kit.Look((0.70, 0.70, 0.68), 0.45, metallic=0.7)
RING = kit.Look((0.15, 0.15, 0.16), 0.4, metallic=0.8)
BAND = kit.Look((0.74, 0.75, 0.77), 0.25, metallic=1.0)
GRIP = kit.Look((0.80, 0.12, 0.10), 0.6)  # red vinyl handle grips
STEEL = kit.Look((0.45, 0.46, 0.48), 0.3, metallic=1.0)
MARK = kit.Look((0.10, 0.10, 0.10), 0.6)


# --- The ring rule, used by both the piston's law (the jam) and the scorer (the history) ------------------------------
def ring_heights(piston_q):
    """Each ring's height above the deck."""
    return [piston_q - d for d in RINGS]


def crossing(piston_q):
    """Rings now crossing from the band into the bore (within 2.5 mm above the deck)."""
    return crossing_between(piston_q, piston_q)


def crossing_between(qa, qb):
    """Rings whose path, as the crown moves from qa to qb, passes through the 2.5 mm crossing zone. A fast stroke can
    carry a ring through the whole zone between two control steps, so the laws test the path, not one sample."""
    lo, hi = min(qa, qb), max(qa, qb)
    return [i for i, d in enumerate(RINGS) if lo - d <= ZONE and hi - d >= 0.0]


def escapes(r, r_prev, lift, lift_prev):
    """A ring escapes when it is out of the band and not in the bore. Below the band: its path during the step (from
    r_prev to r) passes through the gap between the deck and the band's lower edge while that gap is over 1 mm (a
    1.2 mm ring fits through it), so a fast stroke can't skip the gap between samples, while a ring that goes from
    the band straight into the bore with the band seated is in, however fast it went. Above the band: the piston
    pulled up until a ring's middle is past the band's top edge, where it springs open."""
    if r > lift + BAND_H:
        return True
    up = max(lift, lift_prev)
    if up <= LIFT_TOL:
        return False
    return max(r, r_prev) >= 0.0 and min(r, r_prev) <= up


@dataclass(frozen=True)
class PistonLaw:
    """The piston: held by ring friction where it is left (gravity off; the rings' drag is far more than its weight),
    pushed back by each crossing ring, and held for good once a ring escapes (the sim's stand-in for a jam)."""

    def fresh(self, lk):
        return {"prev": lk.start, "prev_lift": 0.0, "jam": None, "escaped": []}

    def drive(self, key, q, qd, J, mem):
        lift = J["compressor"]
        if mem["jam"] is None:
            for i, d in enumerate(RINGS):
                if escapes(q - d, mem["prev"] - d, lift, mem["prev_lift"]):
                    mem["escaped"].append(i)
                    mem["jam"] = q
        prev = mem["prev"]
        mem["prev"], mem["prev_lift"] = q, lift
        if mem["jam"] is not None:
            return mem["jam"], JAM_HOLD, 0.0, 50.0
        inside = sum(1 for r in ring_heights(q) if r < 0.0)
        cross = bool(crossing_between(prev, q + (q - prev)))  # last step's path and the next one's at this speed
        damp = DAMP_BAND + DAMP_BORE * inside + (DAMP_CROSS if cross else 0.0)
        return q, 0.0, RING_PUSH if cross else 0.0, damp


@dataclass(frozen=True)
class CompressorLaw:
    """The compressor: its weight holds it on the deck, but a crossing ring pushes it up off the deck unless a hand
    holds it down."""

    def drive(self, key, q, qd, J, mem):
        p = J["piston"]
        prev = mem.get("piston", p)
        mem["piston"] = p
        # A ring crossing during the last step or, at the piston's present speed, during the next one: a fast stroke
        # that carries a ring through the 2.5 mm zone between two control steps still pushes the band up.
        if crossing_between(prev, p + (p - prev)):  # LIFT_N - LIFT_K * q as an implicit spring (stable; an effort
            return LIFT_N / LIFT_K, LIFT_K, 0.0, LIFT_DAMP  # held for a whole control step overshoots to the stop)
        return q, 0.0, 0.0, 0.5


# --- The scene --------------------------------------------------------------------------------------------------------
def _frame(half_out, z0, z1, looks):
    """Four boxes around the square opening at the bore (half-width HOLE), from z0 up to z1."""
    h, b = HOLE, half_out
    rects = [((-(b + h) / 2, 0.0), (b - h, 2 * b)), (((b + h) / 2, 0.0), (b - h, 2 * b)),
             ((0.0, (b + h) / 2), (2 * h, b - h)), ((0.0, -(b + h) / 2), (2 * h, b - h))]
    meshes, colliders = [], []
    for (cx, cy), (sx, sy) in rects:
        if looks:
            body, face, top = looks
            meshes.append(kit.box((cx, cy, (z0 + z1 - top) / 2), (sx, sy, z1 - z0 - top), body))
            meshes.append(kit.box((cx, cy, z1 - top / 2), (sx, sy, top), face))
        colliders.append(kit.Collider((cx, cy, (z0 + z1) / 2), size=(sx, sy, z1 - z0)))
    return meshes, colliders


def _block():
    """The short block in its fixture, in the deck's frame (the table is at -DECK). The crank (axis along y, 6 cm above
    the table) and the rest of the bottom end are inside the crankcase and not drawn."""
    floor, case, flange = -DECK, CASE_TOP - DECK, -FLANGE_T
    meshes = [kit.box((0.0, 0.0, floor + 0.004), (0.20, 0.24, 0.008), STEEL),  # the fixture's base plate
              kit.box((0.0, 0.0, (floor + 0.008 + case) / 2), (0.15, 0.17, case - floor - 0.008), CAST),  # crankcase
              kit.cylinder((0.0, -0.086, CRANK_Z - DECK), 0.034, 0.004, CAST, axis="y"),  # main bearing boss
              kit.cylinder((0.0, -0.118, CRANK_Z - DECK), 0.0095, 0.060, STEEL, axis="y"),  # the crank's 19 mm end
              kit.lathe([(BORE_R, case), (BARREL_R, case), (BARREL_R, flange), (BORE_R, flange)], CAST)]
    for z in np.arange(case + 0.008, flange - 0.004, 0.010):  # the air-cooling fins
        meshes.append(kit.lathe([(BARREL_R, z), (FIN_R, z), (FIN_R, z + 0.003), (BARREL_R, z + 0.003)], CAST))
    colliders = [kit.Collider((0.0, 0.0, (floor + case) / 2), size=(0.15, 0.17, case - floor))]
    m, c = _frame(FLANGE / 2, flange, 0.0, (CAST, DECK_FACE, 0.004))  # the deck flange, machined on top
    meshes += m
    colliders += c
    colliders += _frame(BARREL_R, case, flange, None)[1]  # the barrel (drawn round, collides as a square)
    # The bore: a liner whose flange sits 0.3 mm proud of the deck, a 1 mm chamfer at its mouth, dark below.
    meshes.append(kit.lathe([(BORE_R, case), (BORE_R + 0.002, case), (BORE_R + 0.002, flange), (LINER_R, flange),
                             (LINER_R, 0.0003), (BORE_R + 0.001, 0.0003), (BORE_R, -0.0007)], LINER))
    meshes.append(kit.cylinder((0, 0, case + 0.002), BORE_R, 0.002, BORE_DARK))
    for sx in (-1, 1):  # the four head-bolt holes
        for sy in (-1, 1):
            meshes.append(kit.cylinder((sx * 0.050, sy * 0.050, 0.0001), 0.0045, 0.0006, MARK))
    return meshes, colliders


def _piston():
    m = [kit.cylinder((0, 0, -PISTON_H / 2), PISTON_R, PISTON_H, ALU),
         kit.cylinder((0, 0, -0.0005), PISTON_R - 0.0015, 0.0012, CROWN),  # the crown's raised face
         kit.cylinder((0.012, 0.0, 0.0003), 0.008, 0.0006, CROWN),  # the two valve reliefs
         kit.cylinder((-0.012, 0.0, 0.0003), 0.008, 0.0006, CROWN),
         kit.box((0.0, 0.022, 0.0004), (0.010, 0.003, 0.0008), MARK)]  # the arrow toward the front of the engine
    for d in RINGS:
        m.append(kit.lathe([(PISTON_R - 0.002, -d - 0.0006), (PISTON_R + 0.0002, -d - 0.0006),
                            (PISTON_R + 0.0002, -d + 0.0006), (PISTON_R - 0.002, -d + 0.0006)], RING))
    c = (kit.Collider((0, 0, -0.005), radius=0.030, height=0.010),)
    return kit.Link("piston", "prismatic", "Z", (0.0, 0.0, 0.0), lower=SEAT, upper=0.040, start=START,
                    mass=PISTON_MASS, meshes=tuple(m), colliders=c, floating=True, law=PistonLaw())


def _compressor():
    m = [kit.lathe([(BAND_IN, 0.0), (BAND_OUT - 0.002, 0.0), (BAND_OUT - 0.002, BAND_H - 0.003),
                    (BAND_OUT + 0.0015, BAND_H - 0.003), (BAND_OUT + 0.0015, BAND_H), (BAND_IN, BAND_H)], BAND)]
    c = []
    rc = (BAND_IN + BAND_OUT) / 2
    for k in range(8):  # the band's top as eight blocks around the ring, clear of the piston's 3 cm crown collider
        a = k * math.pi / 4
        size = (0.005, 0.022, BAND_H - 0.002) if k % 4 == 0 else (0.022, 0.005, BAND_H - 0.002) if k % 4 == 2 \
            else (0.006, 0.006, BAND_H - 0.002)
        c.append(kit.Collider((rc * math.cos(a), rc * math.sin(a), 0.001 + (BAND_H - 0.002) / 2), size=size))
    # The two handles: steel bars from the band's hinge lug, with red grips on their outer 9 cm, and the ratchet bar
    # that locks them squeezed.
    perp = np.array([-HANDLE_DIR[1], HANDLE_DIR[0], 0.0])
    lug = HANDLE_DIR * (BAND_OUT + 0.004)
    m.append(kit.box((lug[0], lug[1], HANDLE_Z), (0.012, 0.012, 0.014), STEEL))
    for s in (-1, 1):
        mid = HANDLE_DIR * (HANDLE_R0 + 0.031) + perp * 0.0065 * s
        grip = HANDLE_DIR * (HANDLE_R1 - 0.045) + perp * 0.0065 * s
        m.append(kit.cylinder((mid[0], mid[1], HANDLE_Z), 0.004, 0.062, STEEL, axis=HANDLE_DIR))
        m.append(kit.cylinder((grip[0], grip[1], HANDLE_Z), 0.0052, 0.090, GRIP, axis=HANDLE_DIR))
    rb = HANDLE_DIR * 0.060
    m.append(kit.cylinder((rb[0], rb[1], HANDLE_Z + 0.004), 0.002, 0.020, STEEL, axis=perp))
    for r in np.arange(0.06, HANDLE_R1, 0.02):  # the handles' colliders: 2.2 cm blocks along them, tops 1.3 cm up
        p = HANDLE_DIR * r
        c.append(kit.Collider((p[0], p[1], HANDLE_Z), size=(0.022, 0.022, 0.010)))
    return kit.Link("compressor", "prismatic", "Z", (0.0, 0.0, 0.0), lower=0.0, upper=LIFT_MAX, start=0.0,
                    mass=COMP_MASS, meshes=tuple(m), colliders=tuple(c), law=CompressorLaw())


def _engine():
    meshes, colliders = _block()
    return kit.Mechanism("engine", pos=(BX, BY, TOP + DECK), size=(0.30, 0.30, 2 * DECK), meshes=tuple(meshes),
                         colliders=tuple(colliders), links=(_piston(), _compressor()))


OBJECTS = [_engine()]


# --- Readouts ---------------------------------------------------------------------------------------------------------
def crown(objects):
    """The crown's height above the deck (m)."""
    return kit.joint(objects, "engine/piston").pos


def lift(objects):
    """The compressor band's lower edge above the deck (m)."""
    return kit.joint(objects, "engine/compressor").pos


GRIPPERS = ("left_gripper", "right_gripper")
PISTON_SHAPE = gk.Ring(0.0, PISTON_R, -PISTON_H, 0.0)  # in the piston's frame: the crown's center, on the bore's axis
_perp = np.array([-HANDLE_DIR[1], HANDLE_DIR[0], 0.0])
COMPRESSOR_SHAPE = (  # in the compressor's frame: the band's lower edge, on the bore's axis
    (gk.Ring(BAND_IN, BAND_OUT + 0.0015, 0.0, BAND_H),  # the band and its rolled lip
     gk.Block(tuple(HANDLE_DIR * (BAND_OUT + 0.004) + [0, 0, HANDLE_Z]), (0.012, 0.012, 0.014)))  # the hinge lug
    + tuple(gk.Seg(tuple(HANDLE_DIR * HANDLE_R0 + _perp * 0.0065 * s + [0, 0, HANDLE_Z]),
                   tuple(HANDLE_DIR * HANDLE_R1 + _perp * 0.0065 * s + [0, 0, HANDLE_Z]), 0.0052) for s in (-1, 1))
    + tuple(gk.Block(tuple(HANDLE_DIR * r + [0, 0, HANDLE_Z]), (0.022, 0.022, 0.010))  # the handles' colliders
            for r in np.arange(0.06, HANDLE_R1, 0.02)))


def _on_axis(q):
    """A frame on the bore's axis, q above the deck (the block is fixed and upright)."""
    p = np.array([BX, BY, TOP + DECK + q])
    return ObjectState(pos=p, quat_xyzw=np.array([0.0, 0.0, 0.0, 1.0]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p, high=p)


_SEEN = {}  # this step's answer: success() and score() both ask, with the same objects


def hands_on(objects):
    """A gripper (palm, finger or pad) still touching the piston, the compressor's band or its handles."""
    if _SEEN.get("objs") is not objects:
        objs = {**objects, "piston": _on_axis(crown(objects)), "compressor": _on_axis(lift(objects))}
        _SEEN.clear()
        _SEEN.update(objs=objects, on=bool(gk.touching(objs, "piston", PISTON_SHAPE)
                                           or gk.touching(objs, "compressor", COMPRESSOR_SHAPE)))
    return _SEEN["on"]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.prev = None  # (crown, lift) at the last step
        self.escaped = set()  # rings that sprang out (irreversible for the trial)


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    q, up = crown(objects), lift(objects)
    qp, upp = _H.prev if _H.prev is not None else (q, up)
    for i, d in enumerate(RINGS):
        if escapes(q - d, qp - d, up, upp):
            _H.escaped.add(i)
    _H.prev = (q, up)


def rings_in(objects):
    """Rings now fully in the bore (1 mm or more below the deck) that never escaped."""
    return sum(1 for i, r in enumerate(ring_heights(crown(objects))) if r <= -IN_DEPTH and i not in _H.escaped)


def _job(objects):
    if _H.escaped:  # a ring sprang out under a lifted compressor (the proxy for a jammed or broken ring), irreversible
        return 0.0
    n = rings_in(objects)
    home = crown(objects) <= SEAT + HOME_TOL
    if n == len(RINGS) and home and lift(objects) <= SEATED_TOL and not hands_on(objects):
        return 1.0
    # real progress only: each ring in cleanly, while the piston isn't home or a hand is still on the work (a ring
    # counted here is wholly in the bore, so it stays in when the hand lets go)
    return 0.2 * n


def success(objects):
    """The job done and both hands let go. Called first each step, so it also records the history."""
    _update(objects)
    return _job(objects) == 1.0


def score(objects):
    """Doing nothing scores 0. 0 if at any step a ring escaped (the sim's proxy for a ring caught on the deck's edge or
    broken, which a real trial sees in the bore), whatever happens after (irreversible): out of the band and not in the
    bore, either passing through the gap between the deck and the band while the band was more than 1 mm off the deck (a
    1.2 mm-tall ring fits through the gap), or pulled up past the band's top. Then, at the end: 1.0 if all three rings are in
    the bore (each 1 mm or more below the deck, past the bore's chamfer), the crown within 2 mm of its seat (rod on the
    journal, crown 10.2 mm below the deck; 2 mm is half the 4 mm between ring grooves), the compressor resting on the
    deck (within 1 mm) and both hands let go (no gripper touching the piston, the band or its handles, judged by the
    gripper's own shape; hovering near them is let go); otherwise 0.2 for each ring in cleanly (0, 0.2, 0.4 or 0.6),
    which is also the most a run gets with a hand still on the work. Possible: 0, 0.2, 0.4, 0.6, 1."""
    return _job(objects)


RIGHT_PITCH = -0.25  # rad: the right hand's fingers lean 14 deg forward, the left's 14 deg to the right: with the deck
LEFT_ROLL = -0.25  # at 19.75 cm a straight-down grip can't reach the crown's start height (the arms lift about 22 cm)


def _tips(roll, pitch):
    """From the pad tips to the tool point for a grip tilted by roll and pitch."""
    return -PAD_TIP_PAST_TCP * (kit._rot_xyz(roll, pitch) @ np.array([0.0, 0.0, -1.0]))


def demo():
    """Left arm: press the compressor's handles down onto the deck with closed fingers and keep pressing. Right arm:
    closed fingers onto the crown's middle, then a steady push down through all three rings to the seat. Then the
    right hand comes up off the crown, the left lets go, and both go home. Both grips lean 14 degrees (the deck is
    near the top of the arms' reach)."""
    deck = TOP + DECK
    hp = HANDLE_DIR * PRESS_R
    htop = HANDLE_Z + 0.005  # the handles' collider tops, above the deck

    def R(d, z, label):  # the right hand's pad tips on the bore's axis, z above the deck
        tcp = np.array([BX, BY, deck + z]) + _tips(0.0, RIGHT_PITCH)
        return kit.Tilted(d, tuple(tcp), FINGER_CLOSED, label, pitch=RIGHT_PITCH)

    def L(d, z, label):  # the left hand's pad tips over the handles' press point, z above the deck
        tcp = np.array([BX + hp[0], BY + hp[1], deck + z]) + _tips(LEFT_ROLL, 0.0)
        return kit.Tilted(d, tuple(tcp), FINGER_CLOSED, label, roll=LEFT_ROLL)

    left = [L(3.0, htop + 0.025, "over the compressor's handles"),
            L(1.5, htop + 0.002, "onto the handles"),
            L(1.0, htop - 0.020, "press them down")]
    t_hold = sum(w.duration for w in left)
    right = [R(3.0, START + 0.012, "over the crown"),
             R(t_hold - 3.0 + 0.5, START + 0.003, "onto the crown")]
    z0, z1 = START + 0.003, SEAT - 0.012  # the target leads the crown: the arm lags about 1.5 cm under load
    n = 12
    for k in range(1, n + 1):
        right.append(R(0.8, z0 + (z1 - z0) * k / n, "drive it down"))
    right += [R(1.0, z1, "seated"), R(1.0, START + 0.012, "hand up off the crown")]
    t_right = sum(w.duration for w in right)
    left += [L(t_right - t_hold, htop - 0.020, "keep holding it down"),
             L(1.0, htop + 0.025, "let go, up"),
             kit.Home(2.5), kit.Home(2.0, label="hold at home")]
    right += [R(1.0, START + 0.012, "wait"), kit.Home(2.5), kit.Home(2.0, label="hold at home")]
    return {"left": left, "right": right}
