"""A manual tile cutter on the table in front of the robot, with a glazed 6-inch (15.2 cm) wall tile on its bed, squared
against the side fence so the cut line runs down the tile's middle. The scoring wheel hangs from a carriage that rides
two steel rails above the tile; the carriage starts parked at the near end, its wheel 1.5 cm short of the tile's near
edge, and has a black push knob on its right side. The breaker handle is hinged on the far rail post and held up by its
return spring; pressing it down sets its rubber-shod breaker foot on the tile just inside the far edge, astride the cut
line, which snaps a scored tile over the cutter's center ridge.

The tiler's rule: score the glaze in ONE continuous pass, edge to edge, then snap. Running the wheel back over the score
(a second pass) chips the glaze along the line and lets the wheel wander out of its first track, so the break comes out
ragged; pressing the breaker before the score is complete cracks the tile wherever it is weakest. The carriage stays
at the far stop for the snap (the breaker foot belongs over the end of the line), and the handle is let go after it so
its spring lifts it back clear of the cut halves.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled cutter; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Cut the tile in half on the tile cutter."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
BX, BY = 0.30, 0.0  # the cutter's base origin on the table
BASE_L, BASE_W, BASE_T = 0.30, 0.22, 0.016  # aluminum base plate: length (x), width (y), thickness
BED = 0.020  # the tile's underside: foam pads on the base, level with the steel breaking ridge under the cut line
TILE_X0, TILE_L, TILE_W, TILE_T = 0.225, 0.152, 0.152, 0.008  # near edge, length (x), width (y), thickness
TILE_X1 = TILE_X0 + TILE_L  # far edge (0.377)
TILE_TOP = BED + TILE_T  # 0.028
FENCE_Y = -TILE_W / 2  # the side fence: the tile's right edge sits against it, so the cut line is at y 0
RAIL_Y, RAIL_Z, RAIL_R = 0.025, 0.058, 0.006  # the two guide rails: |y|, height of their axes, radius
POST_X0, POST_X1 = 0.170, 0.435  # the rail posts' inner faces (near end, far end)
POST_T = 0.018  # rail post thickness along x

# The carriage (prismatic along x). Its joint position c is the wheel's travel from the start: wheel at WHEEL0 + c.
WHEEL0 = TILE_X0 - 0.015  # the wheel parked 1.5 cm short of the near edge
TRAVEL = (TILE_X1 + 0.015) - WHEEL0  # to the far stop, 1.5 cm past the far edge (0.182 m)
WHEEL_R = 0.011  # a 22 mm tungsten-carbide scoring wheel
CAR_HX, CAR_HY = 0.017, 0.040  # carriage body half length (x) and half width (y)
CAR_Z0, CAR_Z1 = 0.046, 0.068  # carriage body bottom and top (it wraps the rails)
KNOB = (0.025, 0.025, 0.050)  # the push knob: x, y, z size
KNOB_Y, KNOB_Z = -0.066, 0.073  # its center (y and height; at the wheel's x)
CARRIAGE_MASS = 0.20  # kg
CARRIAGE_DRAG = 3.0  # N per m/s: the carriage's linear bearings and the wheel's drag on the glaze (0.15 N at 5 cm/s)

# The breaker handle (revolute about y at the far post). q = 0: the breaker foot resting on the tile's glaze.
PIVOT = (0.424, 0.0, 0.090)
ARM_L = 0.090  # pivot to the handle grip
FOOT_X = -0.075  # the foot's position along the handle (m from the pivot): 2.8 cm inside the far edge
FOOT_DROP = PIVOT[2] - TILE_TOP  # 6.2 cm from the handle's bar down to the glaze
GRIP_Y0, GRIP_Y1, GRIP_R = 0.055, 0.100, 0.013  # the rubber grip on the handle's crossbar: y span and radius
RAISED = 0.55  # rad: the return spring holds the handle up at 31.5 degrees (foot 4.8 cm above the glaze)
PRESSED_STOP = -0.03  # rad: the handle's lower stop (the tile's halves give 2 mm under the foot once it breaks)

# --- Scoring thresholds and the reasons for them.
CELL = 0.001  # m: the score line is tracked in 1 mm cells, the width of the wheel's track
REVERSE_TOL = 0.003  # m: rolling back more than 3 mm, counted over the whole stroke, starts a second stroke; less is
# the hand's play at a pause (counted in total so a saw-like back-and-forth can't hide under it)
SNAP_Q = 0.0  # rad: the handle pressed until the foot bears on the glaze: on the ridge a scored tile snaps under it
PARKED = 0.008  # m: the wheel at least 8 mm past the far edge (off the tile, at the far stop) for a clean snap
RELEASED_Q = RAISED - 0.09  # rad: the handle back up within 5 degrees of its stop (let go, foot clear of the tile)
HANDS_OFF = 0.04  # m: no gripper's tool point within 4 cm of the grip or the push knob (nobody still holding them)
STILL = 0.005  # m/s: the carriage at rest
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving
GRIPPERS = ("left_gripper", "right_gripper")

# --- What the camera shows (drawing only; the score never reads it). The tile stays one fixed collider throughout.
LINE_W = 0.004  # m: the drawn score line, 4 mm wide (a real score is a hairline; the top camera sees about 5 mm per
# pixel, and a 2 mm line did not show there)
LINE_SEGS = 6  # the line is drawn in 6 lengths of 25 mm; each shows once the wheel has rolled over all of it
GAP = 0.006  # m: the drawn split between the two halves after the snap, 6 mm (over a pixel on the top camera)
LINE_MESHES, WHOLE_MESHES, SPLIT_MESHES = [], [], []  # indices into the cutter's base meshes (filled by _cutter)

ALU = kit.Look((0.74, 0.75, 0.77), 0.35, metallic=0.8)
STEEL = kit.Look((0.80, 0.81, 0.84), 0.2, metallic=1.0)
FOAM = kit.Look((0.12, 0.12, 0.13), 0.9)
RED = kit.Look((0.78, 0.10, 0.08), 0.45)
BLACK = kit.Look((0.06, 0.06, 0.07), 0.6)
SCALE = kit.Look((0.95, 0.80, 0.15), 0.5)
GLAZE = kit.Look((0.20, 0.42, 0.72), 0.15)  # a blue glazed wall tile
BISQUE = kit.Look((0.82, 0.70, 0.58), 0.9)  # the tile's unglazed body, seen at its edges
SCORED = kit.Look((0.04, 0.045, 0.05), 0.8)  # the scored line: crushed glaze, near black
CRACK = kit.Look((0.03, 0.03, 0.035), 0.9)  # the shadowed gap between the two halves
CARBIDE = kit.Look((0.35, 0.36, 0.38), 0.3, metallic=0.9)

BASE = (BX, BY, TOP)


def _w(c):
    """A point given in world coordinates (z above the table), in the cutter base's frame."""
    return (c[0] - BX, c[1] - BY, c[2])


class Carriage:
    """The carriage on its linear bearings: no spring, only a speed-opposing drag, so it moves while pushed and stays
    where it is left."""

    def drive(self, key, q, qd, J, mem):
        return q, 0.0, 0.0, CARRIAGE_DRAG


def _cutter():
    meshes, colliders = [], []

    def solid(c, s, look):
        meshes.append(kit.box(_w(c), s, look))
        colliders.append(kit.Collider(_w(c), size=s))

    solid((BX, BY, BASE_T / 2), (BASE_L, BASE_W, BASE_T), ALU)  # the base plate
    for sy in (1, -1):  # foam pads under the tile's two halves, either side of the ridge
        solid((BX, sy * 0.058, (BASE_T + BED) / 2), (BASE_L - 0.04, 0.10, BED - BASE_T), FOAM)
    meshes.append(kit.box(_w((BX, 0.0, (BASE_T + BED) / 2)), (BASE_L - 0.04, 0.008, BED - BASE_T), STEEL))  # ridge
    # the side fence on the right, with a printed scale
    solid((BX, FENCE_Y - 0.006, BED + 0.006), (0.20, 0.012, 0.012), RED)
    for k in range(9):
        meshes.append(kit.box(_w((0.24 + k * 0.015, FENCE_Y - 0.006, BED + 0.0122)), (0.0012, 0.008, 0.0004), BLACK))
    # the tile: glazed top, bisque body; one fixed piece (it sits against the fence on the foam)
    tile_c = (TILE_X0 + TILE_L / 2, 0.0, BED + TILE_T / 2)
    WHOLE_MESHES[:] = (len(meshes), len(meshes) + 1)
    meshes.append(kit.box(_w(tile_c), (TILE_L, TILE_W, TILE_T - 0.0012), BISQUE))
    meshes.append(kit.box(_w((tile_c[0], 0.0, TILE_TOP - 0.0007)), (TILE_L, TILE_W, 0.0014), GLAZE))
    colliders.append(kit.Collider(_w(tile_c), size=(TILE_L, TILE_W, TILE_T)))
    # drawn only, hidden at first (see _show): the score line in lengths along x on the glaze, and the snapped tile as
    # two halves either side of a dark gap along the line
    seg = TILE_L / LINE_SEGS
    for k in range(LINE_SEGS):
        LINE_MESHES.append(len(meshes))
        meshes.append(kit.box(_w((TILE_X0 + (k + 0.5) * seg, 0.0, TILE_TOP + 0.0002)), (seg, LINE_W, 0.0004), SCORED))
    half = (TILE_W - GAP) / 2
    for sy in (1, -1):
        y = sy * (GAP / 2 + half / 2)
        SPLIT_MESHES.extend((len(meshes), len(meshes) + 1))
        meshes.append(kit.box(_w((tile_c[0], y, tile_c[2])), (TILE_L, half, TILE_T - 0.0012), BISQUE))
        meshes.append(kit.box(_w((tile_c[0], y, TILE_TOP - 0.0007)), (TILE_L, half, 0.0014), GLAZE))
    SPLIT_MESHES.append(len(meshes))
    meshes.append(kit.box(_w((tile_c[0], 0.0, TILE_TOP - 0.003)), (TILE_L, GAP + 0.001, 0.002), CRACK))
    # the rail posts at both ends and the two rails
    for x in (POST_X0 - POST_T / 2, POST_X1 + POST_T / 2):
        solid((x, 0.0, (BASE_T + RAIL_Z + 0.014) / 2), (POST_T, 0.075, RAIL_Z + 0.014 - BASE_T), ALU)
    rail_len = POST_X1 - POST_X0 + 2 * POST_T
    for sy in (1, -1):
        meshes.append(kit.cylinder(_w(((POST_X0 + POST_X1) / 2, sy * RAIL_Y, RAIL_Z)), RAIL_R, rail_len, STEEL, axis="x"))
    # the handle's pivot block on top of the far post
    solid((PIVOT[0], 0.0, (RAIL_Z + 0.014 + PIVOT[2] + 0.008) / 2), (0.022, 0.03, PIVOT[2] + 0.008 - RAIL_Z - 0.014), ALU)

    # --- the carriage (link frame: at the wheel's axle height under the rails, x = the wheel's contact point)
    cz = lambda z: z - RAIL_Z  # noqa: E731  (a height above the table, in the carriage's frame)
    body_c = (0.0, 0.0, cz((CAR_Z0 + CAR_Z1) / 2))
    body = (2 * CAR_HX, 2 * CAR_HY, CAR_Z1 - CAR_Z0)
    knob_c = (0.0, KNOB_Y, cz(KNOB_Z))
    inner = -KNOB_Y - KNOB[1] / 2  # the knob's inner face (|y|)
    arm_c = (0.0, -(CAR_HY + inner) / 2, 0.0)  # the bracket from the carriage's side out to the knob
    arm_s = (0.012, inner - CAR_HY + 0.002, 0.012)
    car_meshes = (
        kit.box(body_c, body, RED),
        kit.box((0.0, 0.0, cz(CAR_Z1 + 0.002)), (2 * CAR_HX - 0.006, 0.05, 0.004), BLACK),  # top cap
        kit.box((0.0, 0.0, cz(0.5 * (CAR_Z0 + TILE_TOP + WHEEL_R))), (0.010, 0.012, CAR_Z0 - TILE_TOP - WHEEL_R + 0.004),
                ALU),  # the wheel's fork
        kit.cylinder((0.0, 0.0, cz(TILE_TOP + WHEEL_R)), WHEEL_R, 0.003, CARBIDE, axis="y"),  # the scoring wheel
        kit.cylinder((0.0, 0.0, cz(TILE_TOP + WHEEL_R)), 0.003, 0.012, STEEL, axis="y"),  # its axle
        kit.box(arm_c, arm_s, ALU),
        kit.box(knob_c, KNOB, BLACK),
        kit.box((0.0, KNOB_Y, cz(KNOB_Z + KNOB[2] / 2 + 0.001)), (KNOB[0] - 0.004, KNOB[1] - 0.004, 0.002), SCALE),
    )
    carriage = kit.Link("carriage", "prismatic", "X", _w((WHEEL0, 0.0, RAIL_Z)), lower=0.0, upper=TRAVEL, start=0.0,
                        mass=CARRIAGE_MASS, meshes=car_meshes,
                        colliders=(kit.Collider(body_c, size=body), kit.Collider(knob_c, size=KNOB)),
                        damping=0.5, floating=True, law=Carriage())

    # --- the breaker handle (link frame at the pivot; at q = 0 the bar is level and the foot is on the glaze)
    cross_y0 = -0.010
    handle_meshes = (
        kit.cylinder((0.0, 0.0, 0.0), 0.007, 0.034, STEEL, axis="y"),  # the pivot pin
        kit.box((-ARM_L / 2 + 0.004, 0.0, 0.0), (ARM_L + 0.008, 0.012, 0.012), RED),  # the bar
        kit.box((FOOT_X, 0.0, -FOOT_DROP / 2 + 0.006), (0.010, 0.010, FOOT_DROP - 0.012), RED),  # the foot's leg
        kit.box((FOOT_X, 0.0, -FOOT_DROP + 0.004), (0.014, 0.060, 0.008), BLACK),  # its rubber shoe, astride the line
        kit.box((-ARM_L, (cross_y0 + GRIP_Y0) / 2, 0.0), (0.012, GRIP_Y0 - cross_y0, 0.012), RED),  # the crossbar
        kit.cylinder((-ARM_L, (GRIP_Y0 + GRIP_Y1) / 2, 0.0), GRIP_R, GRIP_Y1 - GRIP_Y0, BLACK, axis="y"),  # the grip
    )
    grip_c = (-ARM_L, (GRIP_Y0 + GRIP_Y1) / 2, 0.0)
    handle = kit.Link("handle", "revolute", "Y", _w(PIVOT), lower=PRESSED_STOP, upper=RAISED, start=RAISED,
                      mass=0.10, meshes=handle_meshes,
                      colliders=(kit.Collider(grip_c, size=(2 * GRIP_R, GRIP_Y1 - GRIP_Y0, 2 * GRIP_R)),),
                      spring=0.6, rest=RAISED + 0.15, damping=0.05, floating=True)

    return kit.Mechanism("cutter", pos=BASE, size=(BASE_L, BASE_W, 0.11), meshes=tuple(meshes),
                         colliders=tuple(colliders), links=(carriage, handle))


OBJECTS = [_cutter()]
kit.set_visible("cutter", LINE_MESHES + SPLIT_MESHES, False, body="base")  # an unscored, unbroken tile at the start


def _rot_y(t):
    c, s = math.cos(t), math.sin(t)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def grip_at(q):
    """The grip's center (world, z absolute) with the handle at angle q."""
    return np.array([PIVOT[0], PIVOT[1], TOP + PIVOT[2]]) + _rot_y(q) @ np.array([-ARM_L, (GRIP_Y0 + GRIP_Y1) / 2, 0.0])


def knob_at(c):
    """The push knob's center (world, z absolute) with the carriage at travel c."""
    return np.array([WHEEL0 + c, KNOB_Y, TOP + KNOB_Z])


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
E0, E1 = TILE_X0 - WHEEL0, TILE_X1 - WHEEL0  # the tile's near and far edges in carriage travel
N_CELLS = int(round((E1 - E0) / CELL))


class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.cells = np.zeros(N_CELLS, dtype=int)  # the stroke (1, 2, ...) that first scored each 1 mm cell; 0 = none
        self.stroke = 1  # the current stroke of the wheel
        self.dir = 0  # its direction: +1 away from the robot, -1 back, 0 not yet moved
        self.extreme = None  # the furthest travel of the current stroke in its direction
        self.prev = None  # the carriage's travel at the last step
        self.back = 0.0  # total travel rolled back against the current stroke's direction (play at pauses)
        self.rescored = False  # irreversible: the wheel ran back over line it had already scored (a second pass)
        self.snapped = False  # the handle has been pressed to the breaking angle (the tile broke; irreversible)
        self.clean_snap = None  # at the snap: was there one full pass, no re-score, and the carriage at the far stop?
        self.home = None  # each gripper's tool point at the trial's first scored step
        self.last = None
        self.still = False


_H = _History()


def _sweep(a, b):
    """The wheel rolled from travel a to b in this step: mark the cells it crossed for the current stroke, and flag a
    re-score if any of them was scored by an earlier stroke."""
    lo, hi = sorted((a, b))
    i0 = max(int(math.floor((lo - E0) / CELL)), 0)
    i1 = min(int(math.ceil((hi - E0) / CELL)), N_CELLS)
    if i1 <= i0:
        return
    seg = _H.cells[i0:i1]
    if np.any((seg != 0) & (seg != _H.stroke)):
        _H.rescored = True
    seg[seg == 0] = _H.stroke


def _track(c):
    """Follow the wheel. A stroke continues while the carriage keeps going one way (pauses included); rolling back
    more than REVERSE_TOL from the stroke's furthest point starts a new stroke from there."""
    if _H.prev is None:
        _H.prev, _H.extreme = c, c
        return
    if _H.dir == 0:
        if abs(c - _H.extreme) > REVERSE_TOL:
            _H.dir = 1 if c > _H.extreme else -1
            _sweep(_H.extreme, c)
            _H.extreme = c
    else:
        _H.back += max(0.0, (_H.prev - c) * _H.dir)  # this step's roll-back against the stroke, added up
        if (c - _H.extreme) * _H.dir >= 0:  # still going the same way, past its furthest point so far
            _sweep(_H.extreme, c)
            _H.extreme = c
        elif abs(c - _H.extreme) > REVERSE_TOL or _H.back > REVERSE_TOL:  # rolled back: a new stroke from here
            _H.stroke += 1
            _H.dir = -_H.dir
            _H.back = 0.0
            _sweep(_H.extreme, c)
            _H.extreme = c
    _H.prev = c


def _full_pass():
    """Every cell of the line, edge to edge, scored, all by one stroke (one continuous pass). Strokes that never
    reached the tile (a nudge at the near end) don't matter."""
    return bool(_H.cells[0] != 0 and np.all(_H.cells == _H.cells[0]))


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    c = kit.joint(objects, "cutter/carriage").pos
    _track(c)
    q = kit.joint(objects, "cutter/handle").pos
    if not _H.snapped and q <= SNAP_Q:
        _H.snapped = True
        _H.clean_snap = _full_pass() and not _H.rescored and c >= E1 + PARKED
    now = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    if _H.home is None and len(now) == 2:
        _H.home = now
    _H.still = _H.last is not None and all(g in _H.last and np.linalg.norm(now[g] - _H.last[g]) < HOME_STILL
                                           for g in now)
    _H.last = now
    _show()


def _show():
    """Draw the tile's state from the history (visual only): each length of the score line once the wheel has rolled
    over all of it, and after the snap the two halves with the gap between them in place of the whole tile."""
    per = N_CELLS / LINE_SEGS
    for k, i in enumerate(LINE_MESHES):
        cells = _H.cells[int(round(k * per)):int(round((k + 1) * per))]
        kit.set_visible("cutter", [i], bool(np.all(cells != 0)) and not _H.snapped, body="base")
    kit.set_visible("cutter", WHOLE_MESHES, not _H.snapped, body="base")
    kit.set_visible("cutter", SPLIT_MESHES, _H.snapped, body="base")


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still."""
    if _H.home is None or not _H.still:
        return False
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - _H.home[g]) < HOME_TOL for g in GRIPPERS)


def _held(objects):
    """A gripper's tool point within 4 cm of the handle's grip or of the carriage's push knob."""
    parts = (grip_at(kit.joint(objects, "cutter/handle").pos), knob_at(kit.joint(objects, "cutter/carriage").pos))
    return any(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - p) < HANDS_OFF
               for g in GRIPPERS for p in parts)


def _job(objects):
    if _H.rescored:
        return 0.0  # a second pass over the score: chipped glaze, a wandering line
    if _H.snapped and not _H.clean_snap:
        return 0.0  # snapped before one full pass was finished (or with the carriage off its far stop): a random crack
    if not _full_pass() or _held(objects):
        return 0.0  # no complete score line yet, or something still held
    car = kit.joint(objects, "cutter/carriage")
    if car.pos < E1 + PARKED or abs(car.vel) > STILL:
        return 0.0  # unreachable without a re-score (the wheel would be back on the tile); kept as a guard
    if not _H.snapped:
        return 0.25  # one clean pass, edge to edge, but the tile not snapped
    if kit.joint(objects, "cutter/handle").pos < RELEASED_Q:
        return 0.0  # the handle still pressed down (it is only held there by something on it)
    return 1.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """One clean pass, then the snap, the handle back up and nothing held, and (with REQUIRE_ARMS_HOME) both arms home
    and still. Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """The score line is the wheel's travel over the tile, kept in 1 mm cells. A stroke is a run of the carriage one
    way; rolling back more than 3 mm in total starts a new one. 0 if any stroke ran the wheel over line an earlier stroke had
    scored (a second pass; irreversible), if the handle was pressed down to the breaking angle (the foot on the glaze)
    before one stroke had scored the whole line edge to edge, or with the carriage not parked at its far stop (wheel
    8 mm or more past the far edge), and 0 while a gripper's tool point is within 4 cm of the grip or the push knob at
    the end. 0.25: one clean pass edge to edge, carriage parked and still, no snap. 1.0: the clean pass, then the snap,
    and the handle let go and back up within 5 degrees of its stop. With REQUIRE_ARMS_HOME, a finished job with the arms
    not back home and still scores 0.9. Possible scores: 0, 0.25, 0.9, 1."""
    return _score(objects)


def demo():
    """Right arm: closed fingers down behind the carriage's push knob, one smooth push to the far stop (the single
    pass), straight up off the knob and home. Left arm, after that: closed fingers on top of the handle's grip, press it
    down along its arc past the breaking angle, lift off so the spring raises the handle, home."""
    lag = 0.016  # the arm trails its target by about 1.6 cm
    k0 = knob_at(0.0)
    behind = k0[0] - KNOB[0] / 2 - 0.012  # the fingers' center 1.2 cm behind the knob's near face
    zr = k0[2] + 0.010
    right = [Waypoint(1.8, (behind - 0.01, KNOB_Y, zr + 0.07), FINGER_CLOSED, "over the space behind the knob"),
             Waypoint(1.2, (behind - 0.005, KNOB_Y, zr), FINGER_CLOSED, "down behind the knob"),
             Waypoint(0.6, (behind, KNOB_Y, zr), FINGER_CLOSED, "touch the knob"),
             Waypoint(4.5, (behind + TRAVEL + lag, KNOB_Y, zr), FINGER_CLOSED, "one pass to the far stop"),
             Waypoint(0.8, (behind + TRAVEL + lag, KNOB_Y, zr), FINGER_CLOSED, "hold at the stop"),
             Waypoint(1.0, (behind + TRAVEL - 0.01, KNOB_Y, zr + 0.05), FINGER_CLOSED, "straight up off the knob"),
             kit.Home(2.0), kit.Home(1.0, label="hold at home")]
    t_right = sum(w.duration for w in right)
    top = lambda q: grip_at(q) + np.array([0.0, 0.0, GRIP_R + 0.013 - 0.002])  # noqa: E731  (TCP on top of the grip)
    s = top(RAISED)
    left = [Waypoint(t_right - 1.5, (0.22, 0.22, TOP + 0.20), FINGER_CLOSED, "wait for the pass"),
            Waypoint(1.8, tuple(s + np.array([-0.012, 0, 0.022])), FINGER_CLOSED, "over the grip"),
            Waypoint(1.0, tuple(s), FINGER_CLOSED, "on the grip")]
    for k in range(1, 9):  # press down along the handle's arc, a little past the stop so it bears firmly
        q = RAISED + (PRESSED_STOP - 0.06 - RAISED) * k / 8
        left.append(Waypoint(0.35, tuple(top(q) - np.array([0, 0, 0.004 * k / 8])), FINGER_CLOSED, "press down"))
    e = top(PRESSED_STOP)
    left += [Waypoint(1.0, tuple(e - np.array([0, 0, 0.012])), FINGER_CLOSED, "hold the snap"),
             Waypoint(1.2, tuple(e + np.array([-0.04, 0, 0.07])), FINGER_CLOSED, "lift off: the spring raises it"),
             kit.Home(2.0), kit.Home(2.0, label="hold at home")]
    right.insert(-1, kit.Home(sum(w.duration for w in left) - t_right))
    return {"left": left, "right": right}
