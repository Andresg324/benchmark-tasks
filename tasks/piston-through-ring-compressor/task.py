"""A single-cylinder small-engine block (68 mm bore, the size of a 200 cc shop or generator engine) stands on the bench
with its deck up. A new piston with three rings is started in the bore: its skirt is in the bore and its ring pack is
squeezed inside a pliers-type ring compressor, a steel band that stands on the deck with its two handles reaching out
to the robot's left. "Install the piston in the bore." An engine builder holds the compressor down flat on
the deck with one hand and drives the piston down through it with the other (in a shop by tapping the crown with a
hammer handle). If the compressor rides up off the deck while a ring is crossing from the band into the bore, that
ring springs out into the gap, catches on the deck's edge and the piston jams (pushed on, the ring breaks).
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled engine block articulation; nothing on disk changes)
from dataclasses import dataclass  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Install the piston in the bore."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the bench is clear for
# the next cylinder. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# --- The block (fixed). Heights are above the deck unless they say "table"; the bore's axis is vertical.
BX, BY = 0.26, -0.17  # the bore's axis
DECK = 0.060  # the deck's height above the table
BLOCK = 0.150  # the block's footprint, x by y
BORE_R = 0.034  # 68 mm bore
LINER_R = 0.0515  # the liner's flange (covers the square opening cut in the block's frame boxes)
HOLE = 0.036  # half-width of the square opening the frame boxes leave around the bore

# --- The piston (and rod, one sliding part). q = the crown's height above the deck.
PISTON_R = 0.0338  # 67.6 mm across: 0.4 mm skirt clearance
PISTON_H = 0.050  # crown to skirt bottom
RINGS = (0.005, 0.009, 0.013)  # each ring groove's depth below the crown: top compression, second, oil ring
RING_NAMES = ("top compression ring", "second ring", "oil ring")
START = 0.022  # crown 22 mm above the deck: skirt in the bore, all three rings inside the compressor's band
SEAT = -0.010  # the rod's big end on the crank journal: the crown 10 mm below the deck, the piston can go no further
PISTON_MASS = 0.25  # kg, piston, pin and rod

# --- The ring compressor (a free sliding sleeve on the bore's axis). q = how far its band's lower edge is above the deck.
BAND_H = 0.018  # band height: the whole 8 mm ring pack fits inside with the crown 4 mm proud of it at the start
BAND_IN, BAND_OUT = 0.0342, 0.0385  # the band around the rings; a rolled stiffening lip to 4 cm at its top
HANDLE_DIR = np.array([0.5, math.sqrt(3.0) / 2, 0.0])  # the handles reach to the robot's left, 30 deg forward
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
DAMP_BAND, DAMP_BORE, DAMP_CROSS = 60.0, 40.0, 200.0  # N*s/m: rings sliding in the band, each ring in the bore, crossing
JAM_HOLD = 20000.0  # N/m: an escaped ring caught on the deck edge holds the piston (1 mm of push = 20 N)

# --- Scoring thresholds (reasons in score() and the README)
LIFT_TOL = 0.001  # m: a gap over 1 mm under the band lets a 1.2 mm-tall ring spring out of it
IN_DEPTH = 0.001  # m: a ring counts as in once it is 1 mm below the deck, past the chamfer, all inside the bore
HOME_TOL = 0.002  # m: the crown within 2 mm of its seat (rod on the journal); less than the 4 mm between grooves
SEATED_TOL = 0.001  # m: the compressor resting on the deck at the end
HANDS_R = 0.21  # m: a tool point this close to the bore's axis (the handles reach 19 cm) ...
HANDS_Z = BAND_H + 0.06  # ... and lower than 6 cm above the band's top is on the piston or the compressor
ARM_HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
ARM_STILL = 0.002  # m per control step

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
    return [i for i, r in enumerate(ring_heights(piston_q)) if 0.0 <= r <= ZONE]


def escapes(r, r_prev, lift, lift_prev):
    """A ring springs out when the band has lifted more than 1 mm off the deck while the ring is in the gap between
    the deck and the band's lower edge, or crossed the deck plane during a step in which the band was lifted."""
    if r_prev > ZONE and r < 0.0:  # driven through the whole 2.5 mm entry in one 50 ms step (over 5 cm/s): the ring is
        return True  # slammed past the band's lip instead of being squeezed in, and catches on the deck edge
    up = max(lift, lift_prev)
    if up <= LIFT_TOL:
        return False
    return (0.0 <= r <= lift) or (r_prev > 0.0 >= r)


@dataclass(frozen=True)
class PistonLaw:
    """The piston: held by ring friction where it is left (gravity off; the rings' drag is far more than its weight),
    pushed back by each crossing ring, and jammed for good once a ring escapes."""

    def fresh(self, lk):
        return {"prev": lk.start, "prev_lift": 0.0, "jam": None, "escaped": []}

    def drive(self, key, q, qd, J, mem):
        lift = J["compressor"]
        if mem["jam"] is None:
            for i, d in enumerate(RINGS):
                if escapes(q - d, mem["prev"] - d, lift, mem["prev_lift"]):
                    mem["escaped"].append(i)
                    mem["jam"] = q
        mem["prev"], mem["prev_lift"] = q, lift
        if mem["jam"] is not None:
            return mem["jam"], JAM_HOLD, 0.0, 50.0
        inside = sum(1 for r in ring_heights(q) if r < 0.0)
        cross = bool(crossing(q))
        damp = DAMP_BAND + DAMP_BORE * inside + (DAMP_CROSS if cross else 0.0)
        return q, 0.0, RING_PUSH if cross else 0.0, damp


@dataclass(frozen=True)
class CompressorLaw:
    """The compressor: its weight holds it on the deck, but a crossing ring pushes it up off the deck unless a hand
    holds it down."""

    def drive(self, key, q, qd, J, mem):
        if crossing(J["piston"]):  # LIFT_N - LIFT_K * q as an implicit spring (stable; an effort held for a whole
            return LIFT_N / LIFT_K, LIFT_K, 0.0, LIFT_DAMP  # control step overshoots to the band's stop)
        return q, 0.0, 0.0, 0.5


# --- The scene --------------------------------------------------------------------------------------------------------
def _block():
    meshes, colliders = [], []
    h, b = HOLE, BLOCK / 2
    frame = [((-(b + h) / 2, 0.0), (b - h, BLOCK)), (((b + h) / 2, 0.0), (b - h, BLOCK)),
             ((0.0, (b + h) / 2), (2 * h, b - h)), ((0.0, -(b + h) / 2), (2 * h, b - h))]
    face = 0.004  # the machined deck face's thickness
    for (cx, cy), (sx, sy) in frame:
        meshes.append(kit.box((cx, cy, -(DECK + face) / 2), (sx, sy, DECK - face), CAST))
        meshes.append(kit.box((cx, cy, -face / 2), (sx, sy, face), DECK_FACE))
        colliders.append(kit.Collider((cx, cy, -DECK / 2), size=(sx, sy, DECK)))
    # The bore: a liner whose flange sits 0.3 mm proud of the deck, a 1 mm chamfer at its mouth, dark below.
    meshes.append(kit.lathe([(BORE_R, -0.055), (LINER_R, -0.055), (LINER_R, 0.0003), (BORE_R + 0.001, 0.0003),
                             (BORE_R, -0.0007)], LINER))
    meshes.append(kit.cylinder((0, 0, -0.050), BORE_R, 0.002, BORE_DARK))
    for sx in (-1, 1):  # head-bolt holes and the two dowel holes on the deck
        for sy in (-1, 1):
            meshes.append(kit.cylinder((sx * 0.060, sy * 0.060, 0.0001), 0.0045, 0.0006, MARK))
    meshes.append(kit.cylinder((-0.060, 0.0, 0.0001), 0.0035, 0.0006, MARK))
    meshes.append(kit.cylinder((0.060, 0.0, 0.0001), 0.0035, 0.0006, MARK))
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


def hands_on(objects):
    """A tool point within 21 cm of the bore's axis and lower than 6 cm above the band's top: on the piston, the
    compressor or its handles (the handles reach 19 cm out)."""
    for g in GRIPPERS:
        if g not in objects:
            continue
        p = np.asarray(objects[g].pos, float)
        if math.hypot(p[0] - BX, p[1] - BY) < HANDS_R and p[2] - (TOP + DECK) < HANDS_Z:
            return True
    return False


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.prev = None  # (crown, lift) at the last step
        self.escaped = set()  # rings that sprang out (irreversible for the trial)
        self.home = None
        self.last = None
        self.still = False


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
    now = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    if _H.home is None and len(now) == 2:
        _H.home = now
    _H.still = _H.last is not None and all(g in _H.last and np.linalg.norm(now[g] - _H.last[g]) < ARM_STILL
                                           for g in now)
    _H.last = now


def rings_in(objects):
    """Rings now fully in the bore (1 mm or more below the deck) that never escaped."""
    return sum(1 for i, r in enumerate(ring_heights(crown(objects))) if r <= -IN_DEPTH and i not in _H.escaped)


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still."""
    if _H.home is None or not _H.still:
        return False
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - _H.home[g]) < ARM_HOME_TOL
               for g in GRIPPERS)


def _job(objects):
    if _H.escaped:  # a ring sprang out under a lifted compressor: jammed or broken, irreversible
        return 0.0
    if hands_on(objects):  # still holding or pushing at the end: nothing counts until both hands are off
        return 0.0
    n = rings_in(objects)
    home = crown(objects) <= SEAT + HOME_TOL
    if n == len(RINGS) and home and lift(objects) <= SEATED_TOL:
        return 1.0
    return 0.2 * n  # real progress only: each ring in cleanly, piston not home


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """The job done and, with REQUIRE_ARMS_HOME, both arms home and still. Called first each step, so it also records
    the history."""
    _update(objects)
    if os.environ.get("PISTON_DEBUG"):
        g = {s: np.round(np.asarray(objects[s].pos) - (BX, BY, TOP + DECK), 3) for s in GRIPPERS if s in objects}
        with open(os.environ["PISTON_DEBUG"], "a") as f:  # a file path: run_task.sh keeps only the last lines
            print("crown %.4f lift %.4f esc %s in %d" % (crown(objects), lift(objects), sorted(_H.escaped),
                                                        rings_in(objects)), g, file=f)
    return _job(objects) == 1.0 and (_arms_home(objects) or not REQUIRE_ARMS_HOME)


def score(objects):
    """0 if at any step a ring escaped, whatever happens after (irreversible): the compressor's band more than 1 mm
    off the deck (a 1.2 mm-tall ring fits through the gap) while that ring was in the gap between the deck and the
    band, or crossing the deck plane. Then, at the end: 0 if a tool point is still on the work (within 21 cm of the
    bore's axis and lower than 6 cm above the band's top), since nothing counts while held; 1.0 if all three rings
    are in the bore (each 1 mm or more below the deck, past the bore's chamfer), the crown within 2 mm of its seat
    (rod on the journal, crown 10 mm below the deck; 2 mm is half the 4 mm between ring grooves) and the compressor
    resting on the deck (within 1 mm); otherwise 0.2 for each ring in cleanly (0, 0.2, 0.4 or 0.6). With
    REQUIRE_ARMS_HOME, a finished job with the arms not back home and still scores 0.9.
    Possible: 0, 0.2, 0.4, 0.6, 0.9, 1."""
    return _score(objects)


def demo():
    """Left arm: press the compressor's handles down onto the deck with closed fingers and keep pressing. Right arm:
    closed fingers onto the crown's middle, then a steady push down through all three rings to the seat. Then the
    right hand comes up, the left lets go, and both go home."""
    deck = TOP + DECK
    tcp = PAD_TIP_PAST_TCP  # the tool point is this far above the pad tips
    hp = HANDLE_DIR * PRESS_R
    hx, hy = BX + hp[0], BY + hp[1]
    htop = deck + HANDLE_Z + 0.005  # the handles' collider tops
    left = [Waypoint(3.0, (hx, hy, htop + 0.025 + tcp), FINGER_CLOSED, "over the compressor's handles"),
            Waypoint(1.5, (hx, hy, htop + 0.002 + tcp), FINGER_CLOSED, "onto the handles"),
            Waypoint(1.0, (hx, hy, htop - 0.020 + tcp), FINGER_CLOSED, "press them down")]
    t_hold = sum(w.duration for w in left)
    right = [Waypoint(3.0, (BX, BY, deck + START + 0.06 + tcp), FINGER_CLOSED, "over the crown"),
             Waypoint(t_hold - 3.0 + 0.5, (BX, BY, deck + START + 0.003 + tcp), FINGER_CLOSED, "onto the crown")]
    z0, z1 = START + 0.003, SEAT - 0.012  # the target leads the crown: the arm lags about 1.5 cm under load
    n = 12
    for k in range(1, n + 1):
        right.append(Waypoint(0.8, (BX, BY, deck + z0 + (z1 - z0) * k / n + tcp), FINGER_CLOSED, "drive it down"))
    right += [Waypoint(1.0, (BX, BY, deck + z1 + tcp), FINGER_CLOSED, "seated"),
              Waypoint(1.5, (BX, BY, deck + BAND_H + 0.08 + tcp), FINGER_CLOSED, "hand up out of the band")]
    t_right = sum(w.duration for w in right)
    left += [Waypoint(t_right - t_hold, (hx, hy, htop - 0.020 + tcp), FINGER_CLOSED, "keep holding it down"),
             Waypoint(1.0, (hx, hy, htop + 0.025 + tcp), FINGER_CLOSED, "let go, up"),
             kit.Home(2.5), kit.Home(2.0, label="hold at home")]
    right += [Waypoint(1.0, (BX, BY, deck + BAND_H + 0.08 + tcp), FINGER_CLOSED, "wait"),
              kit.Home(2.5), kit.Home(2.0, label="hold at home")]
    if os.environ.get("PISTON_DEMO_NOHOLD"):  # a check of the trap: the right hand alone, the left stays home
        left = [kit.Home(sum(w.duration for w in right))]
    return {"left": left, "right": right}
