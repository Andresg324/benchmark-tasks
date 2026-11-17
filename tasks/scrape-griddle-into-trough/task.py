"""On the station table, in front of the arms: a range-top griddle's steel plate (56.8 x 31.1 cm cooking surface,
with a 7.6 cm grease tray across its near end, toward the robot), unheated. Near the lip, 22 separate pieces of black
modeling compound (Play-Doh), each pressed flat onto the steel, stand in for food burnt onto the plate. A 4 in.
(10.2 cm) griddle scraper lies on the bench to the right.

The job is to scrape the griddle clean: break the pieces free and push them along the plate with the scraper, over the
lip into the grease tray, without flinging them off the griddle or gouging the plate, then set the scraper down in the
taped square.

Each piece is a rigid body (debriskit) bonded to the plate by a breakable joint that the physics solver breaks when the
blade pushes it harder than its bond; a freed piece is a free body on the plate with real friction. The scraper's blade
has to meet the pieces, break them free and push them; they fall into the tray under gravity when pushed over the lip.
The score reads where they really are, by mass: in the tray, still on the griddle, or off it. A piece counts in the
tray only if the scraper put it there: it went over the lip while the held scraper's blade was pushing it (or pushing
pieces that pushed it), and the robot never touched it. The robot touching any piece at any time scores the trial 0.
The scraper starts in one place and has to be lifted, used and set down in a taped square elsewhere on the bench.
Every threshold below appears once, with the same value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import debriskit as dk  # noqa: E402
import guardkit as gk  # noqa: E402
import scrapekit as sk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Scrape the griddle clean into the grease tray with the scraper."

# -- the griddle (world frame: x away from the arms, y to the robot's left, z up; m) ----------------------------------
# A range-top add-on griddle (Assure Parts 177AOGT1227, WebstaurantStore: 27 x 12-1/4 x 4 in. overall, 22-3/8 x 12-1/4
# in. cooking surface, 7-gauge type 430 stainless plate, 1-1/2 in. rear and side edges, a 3 x 12-1/4 x 1-1/4 in. grease
# tray across the open front end), set on the bench on its own frame with the plate's top 5.0 cm above the bench, tray
# end toward the robot. Fixed (clamped to the bench in the real build) and never heated: the trial runs at room
# temperature.
PLATE_Z = TOP + 0.050  # the plate's top
LIP_X = 0.150  # the lip: the plate's near edge, over the tray
PLATE_LX = 0.568  # 22-3/8 in. along x
PLATE_W = 0.311  # 12-1/4 in. along y
PLATE_YC = -0.020
PLATE_Y0 = PLATE_YC - PLATE_W / 2
TRAY_X0 = LIP_X - 0.076  # 3 in. tray
TRAY_DEPTH = 0.032  # 1-1/4 in. below the plate's top
GUARD_H = 0.038  # splash guards 1-1/2 in. (3.8 cm) above the plate on both sides and the far end
GUARD_T = 0.003
STEEL = (0.72, 0.73, 0.74)
DARK_STEEL = (0.45, 0.46, 0.47)

TRAY = Box("tray", pos=((TRAY_X0 + LIP_X) / 2, PLATE_YC, (TOP + PLATE_Z - TRAY_DEPTH) / 2),
           size=(LIP_X - TRAY_X0, PLATE_W, PLATE_Z - TRAY_DEPTH - TOP), color=DARK_STEEL)  # tray floor on the frame
TRAY_WALL = Box("tray_wall", pos=(TRAY_X0 - GUARD_T / 2, PLATE_YC, (TOP + PLATE_Z) / 2),
                size=(GUARD_T, PLATE_W, PLATE_Z - TOP), color=STEEL)  # the tray's near wall, level with the plate
_GUARD_X0 = TRAY_X0 - GUARD_T
_GUARD_LEN = LIP_X + PLATE_LX + GUARD_T - _GUARD_X0
GUARD_R = Box("guard_right", pos=(_GUARD_X0 + _GUARD_LEN / 2, PLATE_Y0 - GUARD_T / 2, (TOP + PLATE_Z + GUARD_H) / 2),
              size=(_GUARD_LEN, GUARD_T, PLATE_Z + GUARD_H - TOP), color=STEEL)
GUARD_L = Box("guard_left", pos=(_GUARD_X0 + _GUARD_LEN / 2, PLATE_Y0 + PLATE_W + GUARD_T / 2,
                                 (TOP + PLATE_Z + GUARD_H) / 2),
              size=(_GUARD_LEN, GUARD_T, PLATE_Z + GUARD_H - TOP), color=STEEL)
GUARD_B = Box("guard_back", pos=(LIP_X + PLATE_LX + GUARD_T / 2, PLATE_YC, (TOP + PLATE_Z + GUARD_H) / 2),
              size=(GUARD_T, PLATE_W, PLATE_Z + GUARD_H - TOP), color=STEEL)

# -- the debris: 22 separate pieces of black modeling compound pressed onto the plate (debriskit) ---------------------
# Food burnt onto a griddle sits in separate stuck-on bits that a scraper has to break free one by one. Here each bit is
# a piece of black Play-Doh (Hasbro, from a freshly opened 4 oz can each trial): a 2.50 g ball pressed flat onto the
# plate through a square hole in a fixed template, so it is 1.9 cm square and 4.8 mm thick (the template's foam board),
# 22 pieces, 55.0 g in all (just under half a can). Each piece sticks only where it was pressed onto the steel; pieces
# do not touch one another. Laid out in a fixed staggered grid near the lip: four rows 3.5 cm apart (6, 5, 6 and 5
# pieces, 3.5 cm apart along a row, alternate rows offset by half), the near row's centers 2.5 cm from the lip.
PIECE_W = 0.019  # m: a pressed piece's side (the template's square hole)
PIECE_T = 0.0048  # m: its thickness (the template's 3/16 in. foam board)
PIECE_MASS = 0.0025  # kg: each piece weighed out to 2.50 g
PITCH = 0.035  # m: center to center, along a row and between rows
ROWS_X = (0.175, 0.210, 0.245, 0.280)  # row centers: the near row 2.5 cm from the lip (x 15.0)
ROW_COUNTS = (6, 5, 6, 5)
PATTERN_YC = -0.020  # the pattern's centerline (the plate's): pieces from y -11.70 to 7.70 cm
DOUGH_COLORS = ((0.05, 0.05, 0.06), (0.08, 0.07, 0.08), (0.11, 0.10, 0.10))
CHIP_LIFT = 0.0  # m: each piece lies on the plate, its underside on the steel
# Each piece is stuck to the plate (and only to the plate) by a breakable bond at the middle of its underside (a PhysX
# fixed joint that the solver breaks for good the first time holding the piece takes more than its break force or
# torque). Each piece holds under its own weight and breaks free under the scraper's edge; in real trials the same
# press (a 1 L water bottle on the measuring cup for 3 s) sets the bond, and qualification checks that the scraper frees
# each piece in one stroke. The scraper's edge driven under a piece's side lifts that side and peels the piece off (the
# break torque); a piece freed by anything but the blade is not credited.
BOND_FORCE = 0.8  # N per piece
BOND_PEEL_ARM = 0.0025  # m
# PhysX tests a joint's break force against its own internal measure of the joint's load, not the push on the piece,
# so the value written is the bond x BREAK_CAL.
BREAK_CAL = 32.0


def _corners2(cx, cy, sx, sy, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy) for dx, dy in
            ((-sx / 2, -sy / 2), (sx / 2, -sy / 2), (sx / 2, sy / 2), (-sx / 2, sy / 2))]


def overlap2(a, b, margin=0.0) -> bool:
    """Two pieces' footprints (oriented rectangles) overlap, or come closer than `margin` (separating-axis test)."""
    pa, pb = _corners2(a.pos[0], a.pos[1], a.size[0], a.size[1], a.yaw), _corners2(b.pos[0], b.pos[1], b.size[0],
                                                                                   b.size[1], b.yaw)
    for poly in (pa, pb):
        for i in range(4):
            (x0, y0), (x1, y1) = poly[i], poly[(i + 1) % 4]
            nx, ny = y0 - y1, x1 - x0
            n = math.hypot(nx, ny)
            nx, ny = nx / n, ny / n
            da = [nx * x + ny * y for x, y in pa]
            db = [nx * x + ny * y for x, y in pb]
            if max(da) + margin <= min(db) or max(db) + margin <= min(da):
                return False
    return True


def _layout():
    chips = []
    for i, (x, n) in enumerate(zip(ROWS_X, ROW_COUNTS)):
        for j in range(n):
            y = PATTERN_YC + (j - (n - 1) / 2) * PITCH
            col = DOUGH_COLORS[(i + j) % len(DOUGH_COLORS)]
            chips.append(dk.Chip(f"piece{i}_{j}", (x, float(y), float(PLATE_Z + PIECE_T / 2 + CHIP_LIFT)),
                                 (PIECE_W, PIECE_W, PIECE_T), 0.0, PIECE_MASS, col,
                                 (BOND_FORCE, BOND_FORCE * BOND_PEEL_ARM)))
    return tuple(chips)


CHIPS = _layout()
CHIP_MASS = np.array([c.mass for c in CHIPS])
TOTAL_MASS = float(CHIP_MASS.sum())

# -- the scraper (scraper frame: origin where the blade's neck meets the handle, +x toward the handle's end, y along the
# edge, z square to the blade). Dexter-Russell Basics P94850 4 in. griddle scraper (Wasserstrom): 4 in. (10.2 cm) stiff
# stainless blade, 4-1/2 in. (11.4 cm) polypropylene handle, 8-1/2 in. (21.6 cm) overall, 4.8 oz (136 g).
BLADE_W = 0.102
BLADE_L = 0.065  # the blade's flat, edge to neck
NECK_L = 0.037
BLADE_T = 0.0015
EDGE_LOCAL = np.array([-(BLADE_L + NECK_L), 0.0, -BLADE_T / 2])  # the middle of the edge, on the blade's underside
# (the face toward the plate when the edge is down: the line that meets the plate, 0.6 mm below the mid-thickness at 36
# degrees, so the 0.6 mm contact band is measured from the steel that touches)
HANDLE = ((0.057, 0.0, 0.0), (0.114, 0.030, 0.024))  # center, size: 11.4 cm long, 30 x 24 mm
BLADE_BOX = ((-(NECK_L + BLADE_L / 2), 0.0, 0.0), (BLADE_L, BLADE_W, BLADE_T))
NECK_BOX = ((-NECK_L / 2, 0.0, 0.0), (NECK_L, 0.030, BLADE_T))
SCRAPER_MASS = 0.136
GRIP_LOCAL = np.array([0.050, 0.0, 0.0])  # where the demo holds the handle, 15.2 cm behind the edge
# At rest the scraper lies on the bench on its edge and the handle's far end: the edge dips by REST_TILT.
REST_TILT = math.atan2(HANDLE[1][2] / 2 - BLADE_T / 2, HANDLE[0][0] + HANDLE[1][0] / 2 - EDGE_LOCAL[0])
SCRAPER_XY = (0.250, -0.255)
R_REST = np.array([[math.cos(-REST_TILT), 0.0, math.sin(-REST_TILT)], [0.0, 1.0, 0.0],
                   [-math.sin(-REST_TILT), 0.0, math.cos(-REST_TILT)]])  # Ry(-tilt): the edge end down
_rest_low = R_REST @ np.array([HANDLE[0][0] + HANDLE[1][0] / 2, 0.0, -HANDLE[1][2] / 2])
SCRAPER_START = (SCRAPER_XY[0], SCRAPER_XY[1], TOP - float(_rest_low[2]) + 0.0005)
WHITE = kit.Look((0.94, 0.94, 0.92), 0.5)
BLADE_LOOK = kit.Look((0.80, 0.81, 0.83), 0.25, metallic=0.9)
SCRAPER = kit.Prop(
    "scraper", pos=SCRAPER_START, mass=SCRAPER_MASS, size=(0.216, BLADE_W, HANDLE[1][2]),
    tilt_xyzw=tuple(float(v) for v in sk.quat(R_REST)),
    meshes=(kit.box(*HANDLE, WHITE), kit.box(*BLADE_BOX, BLADE_LOOK), kit.box(*NECK_BOX, BLADE_LOOK)),
    colliders=(kit.Collider(*HANDLE), kit.Collider(*BLADE_BOX), kit.Collider(*NECK_BOX)))
HANDLE_PARTS = (gk.Block(*HANDLE),)  # what a hand holds
PUSH_PARTS = (gk.Block(*BLADE_BOX), gk.Block(*NECK_BOX))  # what scrapes: the blade and its neck (not the handle)
SCRAPER_PARTS = (gk.Block(*HANDLE), gk.Block(*BLADE_BOX), gk.Block(*NECK_BOX))

# -- where the scraper goes when the job is done: a square of blue painter's tape on the bench, right of where the
# scraper starts (the start does not satisfy it: the scraper's origin starts 8.5 cm outside it, so the scraper has to be
# picked up, used and put down there). Set down means its origin (where the neck meets the handle) inside the tape's
# outer edge.
SPOT_XY = (0.250, -0.400)  # the square's center
SPOT_SIDE = 0.120  # outer edge, 12 cm square
TAPE_W = 0.019  # 3/4 in. tape
TAPE_T = 0.0003  # m: the tape's thickness on the bench
TAPE_BLUE = (0.25, 0.55, 0.85)
_sx, _sy, _h = SPOT_XY[0], SPOT_XY[1], SPOT_SIDE / 2
TAPE = tuple(Box(f"tape_{k}", pos=(px, py, TOP + TAPE_T / 2), size=(lx, ly, TAPE_T), color=TAPE_BLUE)
             for k, px, py, lx, ly in (("near", _sx - _h + TAPE_W / 2, _sy, TAPE_W, SPOT_SIDE),
                                       ("far", _sx + _h - TAPE_W / 2, _sy, TAPE_W, SPOT_SIDE),
                                       ("right", _sx, _sy - _h + TAPE_W / 2, SPOT_SIDE - 2 * TAPE_W, TAPE_W),
                                       ("left", _sx, _sy + _h - TAPE_W / 2, SPOT_SIDE - 2 * TAPE_W, TAPE_W)))
LIFT_MIN = 0.050  # m: picked up: the held scraper's origin rose 5 cm above where it lay (not dragged along the bench)


# -- the gouge check (scrapekit's blade-angle rule, no soil) and thresholds (each with its reason in the README) --------
# contact 0.6 mm (PhysX rest jitter; 1.0 mm once a stroke is on); slide 1.5 mm in 0.15 s; gouge > 70 deg or edge tilted
# > 10 deg over 1 cm
LAW = sk.Law(gap=0.0006, gap_hyst=0.0004)
BLADE = sk.Blade(center=tuple(EDGE_LOCAL), along=(0, 1, 0), normal=(0, 0, 1), width=BLADE_W, lanes=10)
GRID = sk.Plate(x0=LIP_X, y0=PLATE_Y0, lx=0.57, ly=0.31, z=PLATE_Z, cell=0.01, trough_x0=TRAY_X0,
                trough_y=(PLATE_Y0, PLATE_Y0 + PLATE_W))
TIER_TROUGH = (0.25, 0.60, 0.90)  # share of the debris (by mass) in the tray for 0.25, 0.5, 0.75
RESIDUE_MAX = 0.05  # 0.75 also needs at most 5 % of the debris left on the griddle (one piece, 4.5 %)
FULL_TROUGH = 0.98  # 1.0 needs at least 98 % in the tray (in the simulator every piece: one piece is 4.5 %) ...
FULL_RESIDUE = 0.0  # ... and nothing left on the griddle (no piece: the plate clean), plus the scraper set down

# The plate's surface: clean, dry, bare 430 stainless, as in a real trial. Dry steel on steel (the scraper's blade on
# the plate) is about 0.7 static and 0.6 sliding (common engineering tables give 0.5 to 0.8 for clean dry steel); a freed
# piece of dough on the dry plate takes the same 0.7 and 0.6 (the dough's own value, combine "min").
PLATE_FRICTION = (0.70, 0.60)  # static, sliding
_PLATE_SIZE = (PLATE_LX, PLATE_W, PLATE_Z - TOP)
PLATE = kit.Prop("plate", pos=(LIP_X + PLATE_LX / 2, PLATE_YC, (TOP + PLATE_Z) / 2), size=_PLATE_SIZE,
                 meshes=(kit.box((0.0, 0.0, 0.0), _PLATE_SIZE, kit.Look(STEEL, 0.4, metallic=0.8)),),
                 colliders=(kit.Collider((0.0, 0.0, 0.0), _PLATE_SIZE),),
                 friction=PLATE_FRICTION)  # plate on its frame, one fixed block

# A piece of dough on steel: 0.7 static, 0.6 sliding. Every piece is bonded to the plate (the anchor) until the blade
# breaks it free; with its damping, a freed piece stops when the blade stops, as dough does.
dk.setup(CHIPS, dk.Physics(static_friction=0.7, dynamic_friction=0.6, break_scale=BREAK_CAL, linear_damping=1.0,
                           angular_damping=1.0),
         anchor=(PLATE.name, PLATE.pos))

OBJECTS = [PLATE, TRAY, TRAY_WALL, GUARD_R, GUARD_L, GUARD_B, *TAPE, SCRAPER]

SPILL_CAP_SHARE = 0.05  # more than 5 % of the debris (more than one piece) off the griddle caps the score at 0.25
SPILL_CAP = 0.25
GOUGE_CAP = 0.5
SET_DOWN_Z = TOP + 0.030  # the scraper's origin below this: lying on the bench, not on the griddle (plate at 5.0 cm)
STILL_STEP = 0.0005  # m per control step (1 cm/s) ...
STILL_TURN = math.radians(0.5)  # ... and under 0.5 degree a step
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)  # 20 steps = 1 s
GRIPPERS = ("left_gripper", "right_gripper")
HANDS_CAP = 0.0  # the robot touching any piece (fingers, palm or arm) at any time: the trial scores 0
TOUCH_BLADE = 0.0005  # m: blade on a piece, or piece on piece: within 0.5 mm (exact boxes; PhysX rests them at 0)
TOUCH_HAND = 0.001  # m: the robot's own collision mesh within 1 mm of a piece, or overlapping it
MOVE = 0.0002  # m: a contact moves a piece if the piece moved 0.2 mm or more that step (4 mm/s)
DEPART = 0.0005  # m: a piece has left its place (its bond let go) once it is 0.5 mm from where it was pressed
FREE_WINDOW = 2  # steps: a piece freed by the blade leaves its place while the blade (or a pile it pushes) is in contact
# with it, or within 0.1 s of that contact
PUSH_WINDOW = 10  # steps (0.5 s): a piece is credited when it goes over the lip within 0.5 s of being pushed by the blade
PIECE_PART = gk.Block((0.0, 0.0, 0.0), (PIECE_W, PIECE_W, PIECE_T))


def _pose(o):
    return np.asarray(o.pos, float), np.asarray(o.quat_xyzw, float)


def in_footprint(x, y) -> np.ndarray:
    """Over the griddle (plate, tray and splash guards), in plan."""
    x, y = np.asarray(x, float), np.asarray(y, float)
    return ((x >= _GUARD_X0) & (x <= LIP_X + PLATE_LX + GUARD_T)
            & (y >= PLATE_Y0 - GUARD_T) & (y <= PLATE_Y0 + PLATE_W + GUARD_T))


def in_spot(p) -> bool:
    """The scraper's origin inside the taped square's outer edge (in plan)."""
    return abs(float(p[0]) - SPOT_XY[0]) <= SPOT_SIDE / 2 and abs(float(p[1]) - SPOT_XY[1]) <= SPOT_SIDE / 2


def where(debris) -> tuple:
    """Each piece's place from its real pose (center): in the tray (inside the tray's walls, below the plate's top),
    off the griddle (outside its footprint: on the bench, the floor, or carried off on the scraper), or else still on
    the griddle (on the plate, against a guard, on the blade above the plate, or falling). A piece still on the
    scraper once it is set down in the taped square is off the griddle; one knocked off the blade into the tray before the
    set-down is in the tray."""
    d = np.asarray(debris, float).reshape(-1, 7)
    x, y, z = d[:, 0], d[:, 1], d[:, 2]
    tray = (x >= TRAY_X0) & (x <= LIP_X) & (y >= PLATE_Y0) & (y <= PLATE_Y0 + PLATE_W) & (z < PLATE_Z)
    off = ~in_footprint(x, y)
    return tray, ~tray & ~off, off


def shares(debris) -> tuple:
    """Shares of the debris by mass: in the tray, left on the griddle, off it."""
    tray, left, off = where(debris)
    return (float(CHIP_MASS[tray].sum()) / TOTAL_MASS, float(CHIP_MASS[left].sum()) / TOTAL_MASS,
            float(CHIP_MASS[off].sum()) / TOTAL_MASS)


def scraper_held(objects) -> bool:
    """A gripper's own shape (palm and fingers, opened as they are) touches the scraper's handle (guardkit's contact
    check: the hand's boxes meet the handle, allowing for the boxes' fit to the gripper's meshes)."""
    return bool(gk.touching(objects, "scraper", HANDLE_PARTS, grippers=GRIPPERS))


def scraper_touched(objects) -> bool:
    """Any gripper touching any part of the scraper (blade included): it is not released."""
    return bool(gk.touching(objects, "scraper", SCRAPER_PARTS, grippers=GRIPPERS))


START_DEBRIS = np.array([[*c.pos, 0.0, 0.0, math.sin(c.yaw / 2), math.cos(c.yaw / 2)] for c in CHIPS])


class _Pose:
    """A piece's pose as guardkit reads an object (pos, quat_xyzw)."""

    def __init__(self, row):
        self.pos, self.quat_xyzw = np.asarray(row[:3], float), np.asarray(row[3:7], float)


_MESH: dict = {}


def mesh(name) -> tuple:
    """A YAM arm link's own collision mesh (yam_links.npz, from the arm's description: surface points about 0.5 mm
    apart, in the link's frame), with its centroid and the radius around it."""
    if not _MESH:
        d = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "yam_links.npz"))
        unit = float(d["unit"][0])  # stored as int16 in 0.05 mm steps
        for k in d.files:
            if k != "unit":
                pts = d[k].astype(float) * unit
                c = pts.mean(axis=0)
                _MESH[k] = (pts, c, float(np.linalg.norm(pts - c, axis=1).max()))
    return _MESH[name]


def robot_on(objects, debris, among) -> np.ndarray:
    """Pieces (of those in `among`) the robot's own shape touches now: any point of an arm link's collision mesh
    (upper arm, forearm, wrist, palm, both fingertips; each link where the simulator reports it this step) within
    TOUCH_HAND of the piece or inside it. Offline, where no link poses are given, the grippers' palm and finger boxes
    (guardkit) stand in."""
    out = np.zeros(len(debris), bool)
    links = [(name, pose) for g in GRIPPERS for name, pose in objects.get(g + kit.ARM_SUFFIX, {}).items()]
    half = np.array([PIECE_W, PIECE_W, PIECE_T]) / 2
    for i in np.nonzero(among)[0]:
        row = debris[i]
        rp = sk.rotm(row[3:7])
        if links:
            for name, (o, q) in links:
                pts, c, r = mesh(name)
                rl = sk.rotm(q)
                if float(np.linalg.norm(np.asarray(o) + rl @ c - row[:3])) - r > 0.02:
                    continue
                loc = ((np.asarray(o) + pts @ rl.T) - row[:3]) @ rp  # the link's points in the piece's frame
                gap = np.linalg.norm(np.maximum(np.abs(loc) - half, 0.0), axis=1)
                if float(gap.min()) <= TOUCH_HAND:
                    out[i] = True
                    break
            continue
        o = dict(objects)
        o["_piece"] = _Pose(row)
        out[i] = bool(gk.touching(o, "_piece", PIECE_PART, tol=TOUCH_HAND, grippers=GRIPPERS))
    return out


def _gap(pa, ra, part_a, pb, rb, part_b) -> float:
    """Gap (m, 0 if they overlap) between box part_a of a body at (pa, ra) and convex part_b of a body at (pb, rb)."""
    return gk._box_gap(pa, ra, part_a, pb, rb, part_b)


def blade_on(scraper, debris, among) -> np.ndarray:
    """Pieces (of those in `among`) the scraper's blade or neck touches now (within TOUCH_BLADE). The handle does not
    scrape."""
    sp, sr = np.asarray(scraper.pos, float), sk.rotm(scraper.quat_xyzw)
    out = np.zeros(len(debris), bool)
    for i in np.nonzero(among)[0]:
        row = debris[i]
        if np.linalg.norm(row[:3] - sp) > 0.16:
            continue
        rp = sk.rotm(row[3:7])
        out[i] = any(_gap(sp, sr, part, row[:3], rp, PIECE_PART) <= TOUCH_BLADE for part in PUSH_PARTS)
    return out


def chain(seed, debris, among) -> np.ndarray:
    """The pieces (of those in `among`) touching a seed piece, directly or through other such pieces (a pile pushed in
    front of the blade moves as one)."""
    seed = np.asarray(seed, bool).copy()
    if not seed.any():
        return seed
    rot = {}
    todo = list(np.nonzero(seed)[0])
    cand = list(np.nonzero(among)[0])
    while todo:
        i = todo.pop()
        for j in cand:
            if seed[j] or np.linalg.norm(debris[i, :3] - debris[j, :3]) > 0.035:
                continue
            ri = rot.setdefault(i, sk.rotm(debris[i, 3:7]))
            rj = rot.setdefault(j, sk.rotm(debris[j, 3:7]))
            if _gap(debris[i, :3], ri, PIECE_PART, debris[j, :3], rj, PIECE_PART) <= TOUCH_BLADE:
                seed[j] = True
                todo.append(j)
    return seed


def over_plate(debris) -> np.ndarray:
    d = np.asarray(debris, float).reshape(-1, 7)
    return (d[:, 0] >= LIP_X) & (d[:, 1] >= PLATE_Y0) & (d[:, 1] <= PLATE_Y0 + PLATE_W)


class Ledger:
    """One trial's history, advanced once per control step (success() and score() share it): the debris as last read,
    which pieces the robot has touched (fingers, palm or arm), when each piece was last pushed by the held scraper's
    blade, whether its bond let go under the blade, whether it went over the lip under the blade (credited), the gouge latch (scrapekit's blade-angle rule on the scraper's pose;
    no soil), whether the scraper has been picked up, and the set-down hold."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.n = 0
        self.s = sk.Scrape(GRID, BLADE, LAW, ())
        self.debris = START_DEBRIS.copy()
        n = len(CHIPS)
        self.hand = np.zeros(n, bool)  # a gripper has touched this piece (latched)
        self.bladed_at = np.full(n, -10 ** 9)  # last step the held blade touched it (directly or through a pile)
        self.pushed_at = np.full(n, -10 ** 9)  # ... and moved it
        self.departed = np.zeros(n, bool)  # it has left its place (its bond let go)
        self.freed = np.zeros(n, bool)  # ... while the held blade was pushing it
        self.credit = np.zeros(n, bool)  # freed by the blade, and its last trip over the lip was under the blade
        self.lifted = False  # the scraper has been held with its origin LIFT_MIN above where it lay
        self.last = None
        self.good = 0  # consecutive steps with the plate clean (the full shares) and the scraper set down and released

    def advance(self, objects, held: bool) -> None:
        """Contacts, releases and lip crossings for this step's debris. A contact counts as a push only when the
        touched piece moved 0.2 mm or more that step: a hand hovering or resting on a still piece does not count."""
        self.n += 1
        prev = self.debris
        if dk.KEY in objects:
            self.debris = np.asarray(objects[dk.KEY], float).reshape(-1, 7)
        d = self.debris
        moved = np.linalg.norm(d[:, :3] - prev[:, :3], axis=1) >= MOVE
        if moved.any():
            self.hand |= robot_on(objects, d, moved)
        if held:  # the blade on a piece, directly or through a pile of touching pieces
            on = chain(blade_on(objects["scraper"], d, np.ones(len(d), bool)), d, np.ones(len(d), bool))
            self.bladed_at[on] = self.n
            self.pushed_at[on & moved] = self.n
        left = ~self.departed & (np.linalg.norm(d[:, :3] - START_DEBRIS[:, :3], axis=1) >= DEPART)
        for i in np.nonzero(left)[0]:  # the first time a piece leaves its place: its bond let go
            self.departed[i] = True
            self.freed[i] = self.n - self.bladed_at[i] <= FREE_WINDOW
        went = over_plate(prev) & (d[:, 0] < LIP_X)  # over the lip this step
        for i in np.nonzero(went)[0]:
            self.credit[i] = self.freed[i] and self.n - self.pushed_at[i] <= PUSH_WINDOW
        self.credit &= ~self.hand

    @property
    def hands(self) -> bool:
        return bool(self.hand.any())

    def shares(self):
        """(credited in the tray, left on the griddle, off the griddle): a piece in the tray that the blade did not put
        there, or that a hand touched, is in none of them."""
        tray, left, off = where(self.debris)
        ok = tray & self.credit & ~self.hand
        return (float(CHIP_MASS[ok].sum()) / TOTAL_MASS, float(CHIP_MASS[left].sum()) / TOTAL_MASS,
                float(CHIP_MASS[off].sum()) / TOTAL_MASS)

    def uncredited(self) -> int:
        tray, _, _ = where(self.debris)
        return int((tray & ~(self.credit & ~self.hand)).sum())

    def tier(self) -> float:
        """The ladder by credited debris in the tray (and left on the griddle for 0.75), then the caps."""
        tr, res, sp = self.shares()
        t = 0.0
        if tr >= TIER_TROUGH[0] - 1e-12:
            t = 0.25
        if tr >= TIER_TROUGH[1] - 1e-12:
            t = 0.5
        if tr >= TIER_TROUGH[2] - 1e-12 and res <= RESIDUE_MAX + 1e-12:
            t = 0.75
        if self.s.gouged:
            t = min(t, GOUGE_CAP)
        if sp > SPILL_CAP_SHARE + 1e-12:
            t = min(t, SPILL_CAP)
        if self.hands:
            t = min(t, HANDS_CAP)
        return t

    def clean(self) -> bool:
        """The full state: at least 98 % credited in the tray, nothing left on the griddle, and no cap."""
        tr, res, sp = self.shares()
        return (tr >= FULL_TROUGH - 1e-12 and res <= FULL_RESIDUE + 1e-12 and not self.s.gouged
                and sp <= SPILL_CAP_SHARE + 1e-12 and not self.hands)


_L = Ledger()


def _update(objects):
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    held = scraper_held(objects)
    _L.advance(objects, held)
    sp, sq = _pose(objects["scraper"])
    if held and float(sp[2]) >= SCRAPER_START[2] + LIFT_MIN:
        _L.lifted = True
    _L.s.step(sp, sq, held)
    r = sk.rotm(sq)
    still = False
    if _L.last is not None:
        dp = float(np.linalg.norm(sp - _L.last[0]))
        dr = math.acos(max(-1.0, min(1.0, (float(np.trace(_L.last[1].T @ r)) - 1) / 2)))
        still = dp <= STILL_STEP and dr <= STILL_TURN
    _L.last = (sp, r)
    down = (_L.lifted and not scraper_touched(objects) and still and float(sp[2]) <= SET_DOWN_Z and in_spot(sp)
            and not _L.s.touching)
    _L.good = _L.good + 1 if (_L.clean() and down) else 0


def success(objects):
    """At least 98 % of the debris (by mass) put in the tray by the scraper, nothing left on the griddle, no robot
    contact with the debris, nothing gouged, at most 5 % off the griddle, and the scraper (picked up) set down in the taped square,
    released and still: this step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """By mass of the debris the scraper put in the grease tray (freed by the held blade and pushed over the lip by
    it, directly or through a pile, within 0.5 s; never touched by the robot): 0.25 with at least 25 %; 0.5 with at least 60 %; 0.75
    with at least 90 % and at most 5 % (one piece) left on the griddle; 1.0 with at least 98 %, nothing left on the
    griddle and the scraper, after being picked up, set down in the taped square, released and still for 1 s. Caps: the
    robot touching any piece at any time scores 0; a gouge (blade over 70 degrees, or the edge tilted over 10 degrees
    onto a corner, slid 1 cm) caps at 0.5; more than 5 % of the debris (more than one piece) off the griddle caps at
    0.25. Doing nothing scores 0."""
    _update(objects)
    s = _L.tier()
    if _L.good >= HOLD_STEPS:
        s = 1.0
    return s


# -- scripted demo: open loop from the fixed layout ---------------------------------------------------------------------
# The right hand takes the scraper by the handle from above (yaw 0: fingers close along world y, across the handle,
# which runs along x), lifts it and tilts it 40 degrees edge-down (Tilted pitch below zero: the approach leans forward),
# so the edge points at the robot and the handle rises away from it. Each stroke sets the edge on the plate beyond
# the debris, draws it toward the robot at 4 cm/s over the lip, lifts it slowly (5 cm/s) over the tray and moves
# back. Three lanes 5 cm apart (each stroke overlaps the last by half the blade), three times over, then a cleanup stroke
# along each splash guard, so the plate ends clean (nothing left). The scraper crosses the splash guard only while level
# and high (tilted, its edge would hang below the guard's top), so it is tilted over the plate and levelled over the
# tray. Then it is carried, level, to the taped square right of where it lay, laid down there and let go.
SCRAPE_DEG = 40.0  # blade-to-plate angle of the demo's strokes (inside the 20-60 degree band)
PRESS = 0.010  # m: the edge is aimed this far into the plate (the collider stops it). Holding the scraper, the hand
# sits 3.5-6.5 mm above its command along the stroke (more toward the robot; measured in the simulator), so a shallower
# aim lets the edge rise off the plate halfway down the stroke and the debris slip under it.
LANES_Y = (-0.070, -0.020, 0.030)  # stroke centers: the blade covers y -0.121 to 0.081
PASSES = 3
# A cleanup pass along both splash guards: pushed pieces spread sideways, and a few can come to rest beyond the outer
# lanes' ends, as far as the guards, anywhere from the debris to the lip. Centered at y -12.0 and 8.0, the blade covers
# y -17.1 to -6.9 and 2.9 to 13.1, 4.5 mm inside each guard (-17.55 and 13.55).
EDGE_LANES_Y = (-0.120, 0.080)
FINAL_X_START = 0.300  # 1 cm beyond the debris's far edge (x 28.95; the plate behind it is already clean): the right arm reaches
# the left guard no farther out
X_START, X_END = 0.350, 0.135  # edge from 6 cm beyond the debris to 1.5 cm past the lip. The hand lags about 1.5 cm,
# so the edge lands over 3 cm beyond the debris, on bare steel, and every stroke meets the pieces from behind
STROKE_SPEED = 0.04  # m/s
LIFT = 0.03  # m above the plate between strokes
LIFT_S = 0.6  # s for the lift at the end of a stroke (0.05 m/s: what rides on the blade slides off into the tray)
CARRY_Z = TOP + 0.15  # tool point height when carrying the scraper level (it clears the 3.8 cm splash guards)
GRIP_YAW = 0.0  # yaw 0: the fingers close along world y, across the handle (which lies along x)
# The handle turns about 6 degrees in the fingers as they close and lift it (measured in the simulator: the edge end swings
# toward -y); tilted 36-40 degrees, that turn would roll the edge 3.5 degrees, one end 6 mm above the plate. The tilted
# hand is turned back by the same amount, so the edge meets the plate along its length.
YAW_TRIM = math.radians(-6.0)
GRIP_DZ = 0.004  # m: the tool point 4 mm above the handle's axis, so the pad tips (13 mm past the tool point) stop
# 1 mm above the handle's underside and clear the bench
GRASP_LOCAL = GRIP_LOCAL + R_REST.T @ np.array([0.0, 0.0, GRIP_DZ])  # the tool point in the scraper's frame


def scraper_rot(pitch: float) -> np.ndarray:
    """The scraper's rotation once the hand that took it at rest pitches by `pitch` about world y (negative: the
    approach leans forward and the edge, pointing at the robot, goes down)."""
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]]) @ R_REST


def pitch_for(theta_deg: float) -> float:
    """Hand pitch that puts the blade at theta_deg to the plate (the rest tilt is already in the grasp)."""
    return REST_TILT - math.radians(theta_deg)


def blade_angle(pitch: float) -> float:
    return sk.blade_angle_deg(scraper_rot(pitch), BLADE)


def tcp_for_edge(edge, pitch: float) -> tuple:
    """The tool point that puts the middle of the scraper's edge at `edge`."""
    r = scraper_rot(pitch)
    return tuple(float(v) for v in np.asarray(edge, float) - r @ (EDGE_LOCAL - GRASP_LOCAL))


def _stroke(plan, y, label, x0=X_START, approach=1.5):
    p = pitch_for(SCRAPE_DEG)
    E = lambda x, dz: tcp_for_edge((x, y, PLATE_Z + dz), p)  # noqa: E731
    run = (x0 - X_END) / STROKE_SPEED
    plan += [kit.Tilted(approach, E(x0, LIFT), FINGER_CLOSED, f"over the start: {label}", GRIP_YAW + YAW_TRIM, pitch=p),
             kit.Tilted(0.8, E(x0, -PRESS), FINGER_CLOSED, f"edge on: {label}", GRIP_YAW + YAW_TRIM, pitch=p),
             kit.Tilted(run, E(X_END, -PRESS), FINGER_CLOSED, label, GRIP_YAW + YAW_TRIM, pitch=p),
             kit.Tilted(LIFT_S, E(X_END, LIFT), FINGER_CLOSED, f"lift over the tray: {label}", GRIP_YAW + YAW_TRIM, pitch=p)]


def demo():
    """Right hand: take the scraper, scrape three overlapping lanes three times over into the tray, lay the scraper down
    in the taped square."""
    g0 = np.asarray(SCRAPER_START) + R_REST @ GRIP_LOCAL
    gx, gy, gz = (float(v) for v in g0)
    gz += GRIP_DZ
    p = pitch_for(SCRAPE_DEG)
    s0 = tcp_for_edge((X_START, LANES_Y[0], PLATE_Z + LIFT), p)
    plan = [Waypoint(2.0, (gx, gy, gz + 0.08), FINGER_OPEN, "above the handle", GRIP_YAW),
            Waypoint(1.5, (gx, gy, gz), FINGER_OPEN, "around the handle", GRIP_YAW),
            Waypoint(0.8, (gx, gy, gz), FINGER_CLOSED, "grip", GRIP_YAW),
            Waypoint(1.2, (gx, gy, CARRY_Z), FINGER_CLOSED, "lift", GRIP_YAW),
            Waypoint(1.5, (0.300, LANES_Y[0] - 0.030, CARRY_Z), FINGER_CLOSED, "level over the plate", GRIP_YAW),
            kit.Tilted(2.0, s0, FINGER_CLOSED, "tilt the edge down over the plate", GRIP_YAW + YAW_TRIM, pitch=p)]
    for k in range(PASSES):
        for i, y in enumerate(LANES_Y):
            _stroke(plan, y, f"pass {k + 1} lane {i + 1}")
    for y, side in zip(EDGE_LANES_Y, ("right", "left")):  # 3 s to cross to each guard (20 cm across, joints slow)
        _stroke(plan, y, f"cleanup along the {side} guard", x0=FINAL_X_START, approach=3.0)
    e = tcp_for_edge((X_END, LANES_Y[-1], PLATE_Z + LIFT), p)  # levelled where the arm reaches CARRY_Z easily
    hx, hy = gx + SPOT_XY[0] - SCRAPER_XY[0], gy + SPOT_XY[1] - SCRAPER_XY[1]  # the same grasp over the taped square
    plan += [Waypoint(1.5, (e[0], e[1], CARRY_Z), FINGER_CLOSED, "level it over the tray", GRIP_YAW),
             Waypoint(2.0, (gx, gy, CARRY_Z), FINGER_CLOSED, "back over the bench", GRIP_YAW),
             Waypoint(1.5, (hx, hy, CARRY_Z), FINGER_CLOSED, "over the taped square", GRIP_YAW),
             Waypoint(1.5, (hx, hy, gz + 0.002), FINGER_CLOSED, "lay it down", GRIP_YAW),
             Waypoint(0.8, (hx, hy, gz + 0.002), FINGER_OPEN, "let go", GRIP_YAW),
             Waypoint(1.2, (hx, hy, gz + 0.08), FINGER_OPEN, "lift clear", GRIP_YAW),
             Waypoint(2.0, (hx, hy, gz + 0.08), FINGER_OPEN, "wait", GRIP_YAW)]
    return {"right": plan}
