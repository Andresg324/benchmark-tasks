"""A 165 mm Japanese kitchen knife (a santoku with a round "wa" handle) lies on an end-grain cutting board next to a
1000-grit whetstone held in a non-slip rubber holder. Sharpening it the trained way means holding the blade at 15 to
20 degrees to the stone and drawing it across in edge-leading strokes, a few on each side, then laying the knife back
down flat on the board with its edge facing away.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled knife, stone and board; nothing on disk changes)
from yam_arena.constants import FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Sharpen the knife on the whetstone, then put it back on the chopping block."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)


def _dirs(deg):
    """(u, v) for a heading: v along the knife from handle to tip, u across it towards the edge (both horizontal)."""
    a = math.radians(deg)
    return np.array([math.cos(a), math.sin(a), 0.0]), np.array([-math.sin(a), math.cos(a), 0.0])


# --- The knife, in its own frame: lying flat, handle axis along +y (handle at -y, tip at +y), edge towards +x,
# origin on the handle axis at the bolster (where the blade meets the handle).
HANDLE_Y = (-0.095, 0.0)
HANDLE_W, HANDLE_T = 0.020, 0.018  # across (in the blade's plane) and thickness of the octagonal handle
BLADE_Y = (0.0, 0.165)  # 165 mm blade
SPINE_X, EDGE_X = -0.007, 0.040  # the blade is 47 mm tall; its edge 40 mm from the handle axis
BLADE_T = 0.003  # collider thickness (the steel is 2 mm at the spine)
GRIP_Y = -0.060  # where the demo holds the handle (6 cm behind the bolster)
KNIFE_MASS = 0.10  # kg (a real 165 mm santoku with a wa handle is about 140 g; see fidelity)
EDGE_SAMPLES = np.linspace(0.005, 0.155, 16)  # points along the edge the scorer checks

# --- The board and the knife's starting place: the knife lies flat on the board, edge away from the robot, turned
# 15 degrees, its handle over the board's edge so a hand can close round it.
PARK_HEADING = 15.0
PARK_TCP = np.array([0.20, -0.05])  # the handle's grip point at the start
BOARD = (0.12, 0.22, 0.08)  # an end-grain maple butcher block: across x along x height
P_U, P_V = _dirs(PARK_HEADING)
BOARD_POS = PARK_TCP + P_V[:2] * (-GRIP_Y - 0.03 + BOARD[1] / 2) + P_U[:2] * 0.0  # its edge under the handle, 3 cm
#                                                                               behind the bolster
KNIFE_START = np.r_[PARK_TCP - GRIP_Y * P_V[:2], TOP + BOARD[2] + BLADE_T / 2]

# --- The stone, fixed in its holder; its long side along the stroke (u at STONE_HEADING).
STONE_HEADING = -15.0
STONE = (0.18, 0.06, 0.025)  # a standard 180 x 60 x 25 mm combination stone
RISER = 0.06  # the wooden sharpening stand the holder sits on
HOLDER_H = 0.015  # the rubber holder under the stone
STONE_TOP = RISER + HOLDER_H + STONE[2]  # 10 cm above the bench
S_U, S_V = _dirs(STONE_HEADING)
STROKE_TCP_Y = -0.20  # where the hand works: the stone sits under the blade's middle
STONE_V = 0.15  # the stone's middle is under the blade 9 cm past the bolster
STONE_POS = np.array([0.355, STROKE_TCP_Y]) + STONE_V * S_V[:2]

# --- Scoring
BAND = (13.0, 22.0)  # deg: the 15-20 degree sharpening angle, plus 2 degrees each way for a hand-held angle
STEEP = 22.0  # deg: steeper than this the edge is rounded over (blunted), not sharpened
CONTACT = 0.003  # m: an edge point within 3 mm of the stone's top, over the stone, counts as on the stone
EDGE_ON = 3  # edge points (1 cm apart, each standing for 1 cm of edge) on the stone for travel to count as a stroke:
#               3 cm of edge, half the stone's 6 cm width; a tip or heel dragged on its corner sharpens nothing
LEVEL = 5.0  # deg: the blade's length (handle to tip) within this of level while stroking; a whetstone stroke keeps
#               the whole edge on the stone, and a pitch above 5 deg lifts the far end of a 6 cm contact 5 mm off it
STROKE = 0.05  # m: edge-leading travel on the stone, in band, that makes a stroke (most of the stone's 6 cm width
#                     crossed by each part of the edge; the demo strokes 7 cm)
STEEP_TRAVEL = 0.02  # m: 2 cm of edge dragged over the stone steeper than STEEP rounds that part of the edge
PER_SIDE = 3  # strokes on each face: "a few strokes per side"
OFF_END = 0.003  # m: an edge point this far below the stone's top just past an end, right after contact = rolled off
HELD_DIST = 0.02  # m: a gripper's tool point this close to the handle's axis, between its ends
HELD_OPEN = 0.6  # and its fingers at most this open (0 closed, 1 fully open) holds the knife
CLEAR = 0.05  # m: at the end every tool point at least this far from the handle (let go)
FLAT = 20.0  # deg: lying flat on a face (not on its spine or edge)
EDGE_AWAY = 0.5  # the edge's horizontal direction has at least this much +x (facing away from the robot, < 60 deg)
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving
GRIPPERS = ("left_gripper", "right_gripper")

STEEL = kit.Look((0.80, 0.82, 0.85), 0.25, metallic=0.9)
BEVEL = kit.Look((0.95, 0.96, 0.97), 0.15, metallic=1.0)  # the polished edge bevel
WOOD = kit.Look((0.55, 0.36, 0.20), 0.6)  # magnolia handle
HORN = kit.Look((0.10, 0.09, 0.09), 0.4)  # buffalo-horn ferrule
STONE_LOOK = kit.Look((0.85, 0.45, 0.22), 0.9)  # orange 1000-grit stone
RUBBER = kit.Look((0.08, 0.08, 0.09), 0.9)
MAPLE = kit.Look((0.86, 0.72, 0.52), 0.7)
MAPLE_DARK = kit.Look((0.74, 0.58, 0.40), 0.7)


def _knife():
    h0, h1 = HANDLE_Y
    meshes = (
        kit.cylinder((0.0, (h0 + h1 - 0.015) / 2, 0.0), HANDLE_T / 2, h1 - h0 - 0.015, WOOD, axis="y", n=8),
        kit.cylinder((0.0, -0.0075, 0.0), HANDLE_T / 2 + 0.0005, 0.015, HORN, axis="y", n=8),
        kit.box(((SPINE_X + EDGE_X) / 2, 0.065, 0.0), (EDGE_X - SPINE_X, 0.13, 0.002), STEEL),
        kit.box(((SPINE_X + 0.026) / 2, 0.1475, 0.0), (0.026 - SPINE_X, 0.035, 0.002), STEEL),  # towards the tip
        kit.box((EDGE_X - 0.003, 0.065, 0.0), (0.006, 0.13, 0.0024), BEVEL),
        kit.box((0.023, 0.1475, 0.0), (0.006, 0.035, 0.0024), BEVEL),
    )
    colliders = (
        kit.Collider((0.0, (h0 + h1) / 2, 0.0), size=(HANDLE_W, h1 - h0, HANDLE_T)),
        kit.Collider(((SPINE_X + EDGE_X) / 2, sum(BLADE_Y) / 2, 0.0), size=(EDGE_X - SPINE_X, BLADE_Y[1], BLADE_T)),
    )
    return kit.Prop("knife", pos=tuple(float(c) for c in KNIFE_START), size=(0.05, 0.26, HANDLE_T), mass=KNIFE_MASS,
                    yaw=math.radians(PARK_HEADING), meshes=meshes, colliders=colliders, com=(0.012, 0.02, 0.0))


def _stone():
    hx, hy = STONE[0] + 0.03, STONE[1] + 0.02
    lip = 0.03  # the holder's end clamps reach 3 cm up (1 cm below the stone's top)
    parts = [  # (centre, size, look)
        ((0, 0, RISER / 2), (hx, hy, RISER), WOOD),
        ((0, 0, RISER + HOLDER_H / 2), (hx, hy, HOLDER_H), RUBBER),
        ((0, 0, RISER + HOLDER_H + STONE[2] / 2), STONE, STONE_LOOK),
        (((hx - 0.012) / 2, 0, RISER + lip / 2), (0.012, hy, lip), RUBBER),
        ((-(hx - 0.012) / 2, 0, RISER + lip / 2), (0.012, hy, lip), RUBBER),
    ]
    meshes = tuple(kit.box(c, sz, lk) for c, sz, lk in parts)
    colliders = tuple(kit.Collider(c, size=sz) for c, sz, _ in parts)
    return kit.Prop("stone", pos=(float(STONE_POS[0]), float(STONE_POS[1]), TOP), size=(hx, hy, STONE_TOP),
                    yaw=math.radians(STONE_HEADING), meshes=meshes, colliders=colliders)


def _board():
    meshes = (kit.box((0, 0, BOARD[2] / 2), BOARD, MAPLE),
              kit.box((0, 0, BOARD[2] + 0.0002), (BOARD[0] - 0.02, BOARD[1] - 0.02, 0.0004), MAPLE_DARK))
    return kit.Prop("board", pos=(float(BOARD_POS[0]), float(BOARD_POS[1]), TOP), size=BOARD, yaw=math.radians(PARK_HEADING),
                    meshes=meshes, colliders=(kit.Collider((0, 0, BOARD[2] / 2), size=BOARD),))


OBJECTS = [_board(), _stone(), _knife()]


# --- Geometry from the knife's pose.
def _rot(q):
    x, y, z, w = (float(c) for c in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _pose(objects):
    k = objects["knife"]
    return np.asarray(k.pos, float), _rot(k.quat_xyzw)


def blade_angle(objects):
    """Degrees between the blade's faces and the horizontal stone top (0 = lying flat, 90 = on its edge)."""
    _, r = _pose(objects)
    return math.degrees(math.acos(min(1.0, abs(r[2, 2]))))


def sharpen_angle(objects):
    """The sharpening angle: degrees the blade's width (spine to edge, knife +x) tilts below the level stone top,
    edge down. Measured across the blade, so pitching the knife tip-down about its width does not count as angle."""
    _, r = _pose(objects)
    return math.degrees(math.asin(max(-1.0, min(1.0, -r[2, 0]))))


def level(objects):
    """The blade's length axis (knife +y, handle to tip) within LEVEL degrees of horizontal."""
    _, r = _pose(objects)
    return abs(r[2, 1]) < math.sin(math.radians(LEVEL))


def side(objects):
    """Which face is down: 'A' (the face that lies on the board at the start) or 'B'."""
    _, r = _pose(objects)
    return "A" if r[2, 2] > 0 else "B"


def _edge_points(objects):
    """World points along the edge's lower corner (both corners of the collider; the lower one is kept per point)."""
    p, r = _pose(objects)
    pts = []
    for y in EDGE_SAMPLES:
        a = p + r @ np.array([EDGE_X, y, BLADE_T / 2])
        b = p + r @ np.array([EDGE_X, y, -BLADE_T / 2])
        pts.append(a if a[2] < b[2] else b)
    return np.array(pts)


def _stone_local(pts):
    """(along the stone's length, across it, height above its top) for world points."""
    d = pts[:, :2] - STONE_POS
    return d @ S_U[:2], d @ S_V[:2], pts[:, 2] - (TOP + STONE_TOP)


def on_stone(objects):
    """Some edge point over the stone's top and within CONTACT of it, with the edge lower than the spine."""
    _, r = _pose(objects)
    if r[2, 0] >= 0:  # the edge (knife +x) is not the low side: the spine or the face is what touches
        return False
    a, c, h = _stone_local(_edge_points(objects))
    over = (np.abs(a) <= STONE[0] / 2) & (np.abs(c) <= STONE[1] / 2)
    return bool(np.any(over & (h <= CONTACT)))


def edge_on_stone(objects):
    """How many of the EDGE_SAMPLES points are over the stone's top and within CONTACT of it."""
    a, c, h = _stone_local(_edge_points(objects))
    return int(np.sum((np.abs(a) <= STONE[0] / 2) & (np.abs(c) <= STONE[1] / 2) & (h <= CONTACT)))


def _off_end(objects):
    """An edge point just past an end of the stone (within 4 cm), across its width, below its top by OFF_END."""
    a, c, h = _stone_local(_edge_points(objects))
    past = (np.abs(a) > STONE[0] / 2) & (np.abs(a) < STONE[0] / 2 + 0.04) & (np.abs(c) <= STONE[1] / 2)
    return bool(np.any(past & (h < -OFF_END)))


def _edge_dir(objects):
    """The horizontal unit direction the edge faces (knife +x projected on the table)."""
    _, r = _pose(objects)
    e = r[:, 0].copy()
    e[2] = 0.0
    n = np.linalg.norm(e)
    return e / n if n > 1e-6 else e


def held(objects):
    """A gripper at most HELD_OPEN open with its tool point within HELD_DIST of the handle's axis, between its ends."""
    p, r = _pose(objects)
    for g in GRIPPERS:
        if g not in objects:
            continue
        opening = kit.opening(objects, g)
        if not (opening <= HELD_OPEN):  # nan (no reading) counts as not holding
            continue
        local = r.T @ (np.asarray(objects[g].pos, float) - p)
        if HANDLE_Y[0] <= local[1] <= HANDLE_Y[1] and math.hypot(local[0], local[2]) <= HELD_DIST:
            return True
    return False


def _released(objects):
    p, r = _pose(objects)
    mid = p + r @ np.array([0.0, sum(HANDLE_Y) / 2, 0.0])
    return all(g not in objects or np.linalg.norm(np.asarray(objects[g].pos, float) - mid) >= CLEAR for g in GRIPPERS)


def on_board(objects):
    """The knife lying flat on the board: its middle over the board, its lowest edge point within 1 cm of the board's
    top, its faces within FLAT of level."""
    p, r = _pose(objects)
    mid = p + r @ np.array([0.0, 0.05, 0.0])
    d = mid[:2] - BOARD_POS
    over = abs(d @ P_U[:2]) <= BOARD[0] / 2 and abs(d @ P_V[:2]) <= BOARD[1] / 2
    low = _edge_points(objects)[:, 2].min() - (TOP + BOARD[2])
    return over and -0.005 <= low <= 0.01 and blade_angle(objects) <= FLAT


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.strokes = {"A": 0, "B": 0}  # strokes counted per face
        self.run = None  # the stroke in progress: dict(side, start, band, steep) or None
        self.since_contact = 99  # control steps since the edge was last on the stone
        self.damaged = None  # why the edge was damaged (irreversible), or None
        self.last_pos = None
        self.home = None
        self.last = None
        self.still = False


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    p, _ = _pose(objects)
    move = np.zeros(3) if _H.last_pos is None else p - _H.last_pos
    _H.last_pos = p
    contact = on_stone(objects) and held(objects)  # a knife pushed or dragged without a hand on it is not a stroke
    if contact:
        ang, s = sharpen_angle(objects), side(objects)
        lead = float(move @ _edge_dir(objects))  # this step's travel in the direction the edge faces
        if _H.run is None or _H.run["side"] != s:
            _H.run = {"side": s, "band": 0.0, "steep": 0.0, "counted": False}
        if ang > STEEP:
            _H.run["steep"] += abs(lead)
        elif BAND[0] <= ang and lead > 0 and level(objects) and edge_on_stone(objects) >= EDGE_ON:
            _H.run["band"] += lead
        if _H.run["steep"] >= STEEP_TRAVEL and _H.damaged is None:
            _H.damaged = "steep"
        if _H.run["band"] >= STROKE and not _H.run["counted"]:
            _H.run["counted"] = True
            _H.strokes[s] += 1
        _H.since_contact = 0
    else:
        _H.run = None  # lifting off ends the stroke
        _H.since_contact += 1
        if _H.since_contact <= 10 and _H.damaged is None and held(objects) and _off_end(objects):
            _H.damaged = "off the end"
    now = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    if _H.home is None and len(now) == 2:
        _H.home = now
    _H.still = _H.last is not None and all(g in _H.last and np.linalg.norm(now[g] - _H.last[g]) < HOME_STILL
                                           for g in now)
    _H.last = now
    _H.steps = getattr(_H, "steps", 0) + 1
    if os.environ.get("KNIFE_DEBUG") and _H.steps % 5 == 0:  # a trace for tuning the demo
        with open(os.environ["KNIFE_DEBUG"], "a") as f:
            a, c, h = _stone_local(_edge_points(objects))
            f.write(f"{_H.steps} pos {np.round(p, 3)} ang {sharpen_angle(objects):.1f} {side(objects)} "
                    f"level {level(objects)} edge_on {edge_on_stone(objects)} "
                    f"stone {on_stone(objects)} held {held(objects)} open {kit.opening(objects, 'right_gripper'):.2f} "
                    f"edge_h {h.min() * 1000:.1f}mm strokes {_H.strokes} dmg {_H.damaged} "
                    f"board {on_board(objects)} safe {put_down_safely(objects)} "
                    f"tcp {np.round(objects['right_gripper'].pos, 3)} local {np.round(_pose(objects)[1].T @ (objects['right_gripper'].pos - p), 3)}\n")


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still."""
    if _H.home is None or not _H.still:
        return False
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - _H.home[g]) < HOME_TOL for g in GRIPPERS)


def put_down_safely(objects):
    """Let go of, lying flat on the board, edge facing away from the robot."""
    return (on_board(objects) and _released(objects) and not held(objects)
            and float(_edge_dir(objects) @ np.array([1.0, 0.0, 0.0])) >= EDGE_AWAY)


def _job(objects):
    if _H.damaged is not None:  # a rounded or rolled edge needs regrinding: worse than unsharpened
        return 0.0
    a, b = _H.strokes["A"], _H.strokes["B"]
    if min(a, b) >= PER_SIDE:
        return 1.0 if put_down_safely(objects) else 0.75
    if max(a, b) >= PER_SIDE:
        return 0.25  # one side only: the edge is pushed over to one side (a wire edge), not sharp
    return 0.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """Both faces sharpened with no damage, the knife put down safely and still, and (REQUIRE_ARMS_HOME) both arms home
    and still. Called first each step, so it also records the history."""
    _update(objects)
    still = np.linalg.norm(np.asarray(objects["knife"].lin_vel, float)) < 0.01
    return _job(objects) == 1.0 and still and (_arms_home(objects) or not REQUIRE_ARMS_HOME)


def score(objects):
    """A stroke: the knife held, its edge on the stone (an edge point over the stone within 3 mm of its top, edge lower
    than spine) and moved 5 cm edge-leading with its faces 13 to 22 degrees to the stone, in one touch (lifting off
    ends it). 0 for the rest of the trial once the edge was damaged: 2 cm of edge rubbed on the stone steeper than 22
    degrees in one touch, or the edge dropped 3 mm below the stone's top just past an end within 0.5 s of touching.
    Otherwise 0.25 for 3+ strokes on one face only, 0.75 for 3+ on each face, 1.0 for that and the knife let go, lying
    flat on the board, edge facing away (0.9 if the arms are not back home and still). Possible scores: 0, 0.25, 0.75,
    0.9, 1."""
    return _score(objects)


# --- Scripted demo (right arm only; the left arm stays home).
def _axrot(a, t):
    a = np.asarray(a, float) / np.linalg.norm(a)
    k = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(t) * k + (1 - math.cos(t)) * k @ k


def _hand(heading, spin):
    """The right hand's rotation for a side grip on the handle: fingers closing across the handle, approach along the
    knife (v at `heading`), turned `spin` degrees about the handle; spin -90 holds the knife flat (face A down),
    -90 + a tilts the edge down by a degrees, +90 - a turns it over (face B down) with the edge down by a."""
    import yam_arena.demo as demo
    r0 = _axrot((0, 0, 1), math.radians(heading)) @ kit._rot_xyz(math.pi / 2, 0.0) @ demo.grip_rotation(math.pi / 2)
    return tuple((_axrot(r0 @ np.array([0.0, 0.0, 1.0]), math.radians(spin)) @ r0).ravel())


DEMO_ANGLE = 17.0  # deg: the angle the demo aims for on the stone
DEMO_TRIM = {"A": 2.5, "B": -2.5}  # deg: the knife sits about 2.5 deg low in the grip on face A and high on B (sim)
GRIP_OPEN = 0.45 * FINGER_OPEN  # part open: room round the 18 mm handle without the lower finger hitting the bench
DEMO_PRESS = 0.001  # m: the edge aimed this far into the stone


def demo():
    """Right arm: slide the open hand sideways round the handle (fingers above and below it), lift, sharpen face A with four push strokes (edge
    leading, away from the robot), turn the knife over and sharpen face B with four pull strokes, lay it back on the
    board edge-away, let go, back out and go home."""
    from yam_arena.constants import FINGER_CLOSED
    T = kit.Tilted
    z_board = TOP + BOARD[2] + BLADE_T / 2
    pu, pv = P_U, P_V
    grip = np.r_[PARK_TCP, z_board]
    beside = grip + 0.06 * pu  # past the spine-to-edge line, clear of the blade (which starts 6 cm further along)
    flat = _hand(PARK_HEADING, -90.0)
    right = [T(2.0, tuple(beside + [0, 0, 0.03]), GRIP_OPEN, "beside the handle", rot=flat),
             T(1.0, tuple(beside), GRIP_OPEN, "down beside it", rot=flat),
             T(1.2, tuple(grip), GRIP_OPEN, "slide round the handle", rot=flat),
             T(0.8, tuple(grip), FINGER_CLOSED, "grip", rot=flat),
             T(1.2, tuple(grip + [0, 0, 0.06]), FINGER_CLOSED, "lift", rot=flat)]
    su = S_U
    centre = np.r_[STONE_POS, 0.0] - STONE_V * np.r_[S_V[:2], 0.0]  # the grip point when the blade's middle is on the stone

    def strokes(sign, n=4):
        """n strokes edge-leading, each from the stone's middle to 6.5 cm past it: sign +1 pushes along +u (face A,
        edge towards +u, the far half of the stone), -1 pulls (face B, the near half)."""
        ang = DEMO_ANGLE + DEMO_TRIM["A" if sign > 0 else "B"]
        rot = _hand(STONE_HEADING, -90.0 + ang if sign > 0 else 90.0 - ang)
        reach = math.cos(math.radians(ang)) * EDGE_X
        z = TOP + STONE_TOP + math.sin(math.radians(ang)) * EDGE_X - DEMO_PRESS
        a = centre - sign * su * (reach + 0.005)  # the edge 5 mm short of the stone's middle
        a[2] = z
        b = a + sign * su * 0.065
        out = [T(1.5, tuple(a + [0, 0, 0.03]), FINGER_CLOSED, "over the stone", rot=rot)]
        for _ in range(n):
            out += [T(0.6, tuple(a), FINGER_CLOSED, "edge on the stone", rot=rot),
                    T(1.4, tuple(b), FINGER_CLOSED, "stroke", rot=rot),
                    T(0.5, tuple(b + [0, 0, 0.02]), FINGER_CLOSED, "lift", rot=rot),
                    T(0.8, tuple(a + [0, 0, 0.02]), FINGER_CLOSED, "back", rot=rot)]
        return out

    right += strokes(1.0)
    right += [T(1.5, tuple(centre + [0, 0, TOP + STONE_TOP + 0.07]), FINGER_CLOSED, "turn the knife over",
                rot=_hand(STONE_HEADING, 0.0))]
    right += strokes(-1.0)
    right += [T(1.0, tuple(centre + [0, 0, TOP + STONE_TOP + 0.07]), FINGER_CLOSED, "lift clear of the stone",
                rot=_hand(STONE_HEADING, 90.0 - DEMO_ANGLE - DEMO_TRIM["B"])),
              T(1.2, tuple(centre + [0, 0, TOP + STONE_TOP + 0.07]), FINGER_CLOSED, "turn back upright",
                rot=_hand(STONE_HEADING, 0.0))]
    right += [T(2.0, tuple(grip + [0, 0, 0.06]), FINGER_CLOSED, "back over the board", rot=flat),
              T(1.2, tuple(grip + [0, 0, 0.004]), FINGER_CLOSED, "lay it down", rot=flat),
              T(0.6, tuple(grip + [0, 0, 0.004]), GRIP_OPEN, "let go", rot=flat),
              T(1.0, tuple(beside + [0, 0, 0.004]), GRIP_OPEN, "slide out", rot=flat),
              T(0.8, tuple(beside + [0, 0, 0.06]), GRIP_OPEN, "up", rot=flat),
              kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    total = sum(w.duration for w in right)
    left = [kit.Home(total - 1.5), kit.Home(1.5, label="hold at home")]
    return {"left": left, "right": right}
