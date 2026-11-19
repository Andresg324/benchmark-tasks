"""Open a compact vaccine refrigerator, put a vaccine delivery away the way CDC storage guidance says, then shut it.

On a clinic counter (the station table): an Accucold ARS1PV Pharma-Vac (1 cu ft; cabinet 55 cm high, 44 cm wide, 41 cm
deep without its door; interior 40 cm high, 36 cm wide, 20 cm deep) stands in front of the arms, its door hinged on the
right and shut (its keyed lock left unlocked), a bar handle down its left edge. Inside are its three wire shelves: the lower two where the maker's dimensioned
front view draws them, 5.7 and 18.1 cm above the interior floor; the top one in its slot at 31.75 cm (12.5 in). The
cabinet has a top and two sides: the only way in is through the front opening, between the shelves. Left of the
fridge, a delivery tote holds three cartons of vaccine lying flat as they were packed: 10-vial cartons of Gardasil 9
(8.6 x 3.95 x 4.45 cm).

The job: open the door, each carton on a shelf, at least 5 cm (2 in) from the walls, the floor, the door and the ceiling of the unit
(CDC: vaccines 2 to 3 inches from the walls, ceiling, floor and door), then the door shut.

What makes it hard is the enclosure. A hand cannot come down from above (the cabinet top is in the way); it carries
each carton in through the 36 cm opening, between two shelves 12 cm apart, and sets it down in a band 5 cm from the
door plane and the back wall, without the fingers or the carton catching the shelf above. The top shelf is too close
to the ceiling for any carton (a flat one would reach 36.2 cm, inside the 5 cm ceiling margin). The door's
self-closing hinge holds it shut, and pulls it shut once it is within 30 degrees of closed; wider open, it stays where
it is left. The door's slab covers the whole opening while it is shut: nothing gets in until it has been opened.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import guardkit as gk  # noqa: E402
import yamkit as kit  # noqa: E402
import yamgeom as yg  # noqa: E402  (the arm's collision hulls, read from the environment at run time)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
DT = 0.05  # s: one control step (20 Hz)
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
RUN_LIMIT_STEPS = 3600
INSTRUCTION = ("Open the fridge, put the three vaccine cartons from the tote away in it, each on a shelf at least 5 cm "
               "from the walls, floor, door and ceiling, then shut the fridge door.")

WHITE = kit.Look((0.93, 0.93, 0.92), 0.45)
LINER = kit.Look((0.96, 0.96, 0.96), 0.35)
WIRE_LOOK = kit.Look((0.82, 0.83, 0.84), 0.4)  # plastic-coated wire
GASKET = kit.Look((0.35, 0.36, 0.38), 0.7)
HANDLE_LOOK = kit.Look((0.20, 0.45, 0.75), 0.4)  # the blue handle
PANEL = kit.Look((0.30, 0.55, 0.80), 0.5)
CARTON_LOOK = kit.Look((0.95, 0.95, 0.96), 0.6)  # white carton ...
BAND_LOOK = kit.Look((0.42, 0.30, 0.62), 0.6)  # ... with a purple band
TOTE_LOOK = kit.Look((0.25, 0.42, 0.65), 0.5)

# -- the fridge (Accucold ARS1PV spec sheet). Its own frame: u into the fridge (world +x), v to its left (world +y),
#    w up; origin on the interior floor, at the middle of the door plane (the cabinet's front face). -----------------
FRONT_X = 0.50  # world x of the cabinet's front face: its back is flush with the table's far edge (x 0.91)
FRIDGE_Y = -0.04  # world y of the cabinet's middle
INT_D, INT_W, INT_H = 0.20, 0.36, 0.40  # interior depth (u), width (v), height (w): 8.0 x 14.0 x 15.75 in
EXT_W, EXT_H = 0.44, 0.55  # cabinet width and height: 17.5 x 21.5 in
BODY_D = 0.41  # cabinet depth without the door: 16 1/8 in
FLOOR_UP = 0.057  # interior floor above the counter (front view: 2.2 in)
ORIGIN = np.array([FRONT_X, FRIDGE_Y, TOP + FLOOR_UP])
SHELF_W = (0.057, 0.181, 0.3175)  # wire tops above the interior floor: 2.2 and 7.1 in (the maker's front view);
# the top shelf one 1/2 in slot above the drawn 12.0 in, at 12.5 in, so no carton fits under the 5 cm ceiling margin
WIRE_D = 0.004  # wire thickness
RAIL_D = 0.006  # the shelf's front and back rails
SHELF_U = (0.010, 0.190)  # the shelf's front and back rails' middles (u)
WIRE_PITCH = 0.025  # front-to-back wires, center to center
DOOR_T = 0.092  # door thickness: 19.75 in deep with the door, 16 1/8 in without
DOOR_OUT = 0.0064  # the open door's outer face stands this far outside the cabinet's right side (18.0 vs 17.5 in)
DOOR_Z = (0.019, 0.502)  # the door's bottom and top above the counter (under the 1 7/8 in control panel)
HANDLE_OUT = 0.051  # the handle (an Amerock BP54008SS bar pull, as on the mock) stands 2 in proud of the door
DOOR_KG = 4.0  # the mock door is weighted to this
GAP = 0.05  # m: CDC's 2 to 3 in from the walls, ceiling, floor and door: the 2 in (5 cm) end
ON_SHELF = (-0.002, 0.004)  # m: a carton's lowest corner within this band of a shelf's wire tops rests on that shelf
SQUARE_DEG = 10.0  # standing or lying square: one of the carton's axes within 10 degrees of vertical
DOOR_OPEN_CAP = 0.5  # the most a trial scores while the door is not shut
SHUT_DEG = 0.5  # door shut: within 0.5 degrees of closed (its free edge within 4 mm of the cabinet front)

FRIDGE_POS = (FRONT_X + BODY_D / 2, FRIDGE_Y, TOP)  # the cabinet's origin: on the counter under its middle
_HW = INT_W / 2
_SIDE_T = (EXT_W - INT_W) / 2
_HD = BODY_D / 2


def wire_vs() -> list:
    """The v of each front-to-back wire of a shelf: evenly spread, the outer ones 1 cm in from the side walls."""
    n = int(round((INT_W - 0.02) / WIRE_PITCH))
    return [-_HW + 0.01 + i * (INT_W - 0.02) / n for i in range(n + 1)]


def _shelf(w_top: float):
    """One wire shelf (cabinet frame): front and back rails across the width, wires front to back on top of them."""
    zr = FLOOR_UP + w_top - WIRE_D - RAIL_D / 2
    zw = FLOOR_UP + w_top - WIRE_D / 2
    meshes, cols = [], []
    for u in SHELF_U:
        c, s = (u - _HD, 0.0, zr), (RAIL_D, INT_W, RAIL_D)
        meshes.append(kit.box(c, s, WIRE_LOOK))
        cols.append(kit.Collider(c, size=s))
    L = SHELF_U[1] - SHELF_U[0] + RAIL_D
    for v in wire_vs():
        c, s = (sum(SHELF_U) / 2 - _HD, v, zw), (L, WIRE_D, WIRE_D)
        meshes.append(kit.box(c, s, WIRE_LOOK))
        cols.append(kit.Collider(c, hull=_WIRE_HULL, faces=_WIRE_FACES))  # a round wire (octagonal section)
    return meshes, cols


def _rod(length: float, d: float, n: int = 8):
    """A rod along x, `d` across (an n-sided section), as hull points about its middle and their triangles."""
    ring = [(0.0, d / 2 * math.cos(2 * math.pi * (k + 0.5) / n), d / 2 * math.sin(2 * math.pi * (k + 0.5) / n))
            for k in range(n)]
    pts = tuple((sx * length / 2, y, z) for sx in (-1, 1) for (_, y, z) in ring)
    faces = [(0, k + 1, k) for k in range(1, n - 1)] + [(n, n + k, n + k + 1) for k in range(1, n - 1)]
    for k in range(n):
        a, b = k, (k + 1) % n
        faces += [(a, b, n + b), (a, n + b, n + a)]
    return pts, tuple(faces)


_WIRE_HULL, _WIRE_FACES = _rod(SHELF_U[1] - SHELF_U[0] + RAIL_D, WIRE_D)


def _cabinet():
    """The cabinet's walls, top, back and floor (meshes and colliders alike), the control panel and the three shelves."""
    back_u = INT_D - _HD
    top_h = EXT_H - FLOOR_UP - INT_H
    walls = [((0.0, 0.0, FLOOR_UP / 2 + 0.0095), (BODY_D, EXT_W, FLOOR_UP - 0.019)),  # base and floor, on its feet
             ((0.0, 0.0, FLOOR_UP + INT_H + top_h / 2), (BODY_D, EXT_W, top_h)),
             ((0.0, _HW + _SIDE_T / 2, FLOOR_UP + INT_H / 2), (BODY_D, _SIDE_T, INT_H)),
             ((0.0, -_HW - _SIDE_T / 2, FLOOR_UP + INT_H / 2), (BODY_D, _SIDE_T, INT_H)),
             (((back_u + _HD) / 2, 0.0, FLOOR_UP + INT_H / 2), (_HD - back_u, INT_W, INT_H))]
    meshes = [kit.box(c, s, WHITE) for c, s in walls]
    meshes += [kit.box((-_HD + INT_D / 2, 0.0, FLOOR_UP + 0.0005), (INT_D, INT_W, 0.001), LINER),
               kit.box((back_u - 0.0005, 0.0, FLOOR_UP + INT_H / 2), (0.001, INT_W, INT_H), LINER),
               kit.box((-_HD - 0.0005, 0.0, EXT_H - 0.024), (0.001, EXT_W - 0.01, 0.040), PANEL)]
    ring = 0.012
    meshes += [kit.box((-_HD + 0.0015, 0.0, FLOOR_UP - ring / 2), (0.003, INT_W + 2 * ring, ring), GASKET),
               kit.box((-_HD + 0.0015, 0.0, FLOOR_UP + INT_H + ring / 2), (0.003, INT_W + 2 * ring, ring), GASKET),
               kit.box((-_HD + 0.0015, _HW + ring / 2, FLOOR_UP + INT_H / 2), (0.003, ring, INT_H), GASKET),
               kit.box((-_HD + 0.0015, -_HW - ring / 2, FLOOR_UP + INT_H / 2), (0.003, ring, INT_H), GASKET)]
    meshes += [kit.box((sx * (_HD - 0.03), sy * (EXT_W / 2 - 0.03), 0.0095), (0.03, 0.03, 0.019), GASKET)
               for sx in (-1, 1) for sy in (-1, 1)]  # feet
    colliders = [kit.Collider(c, size=s) for c, s in walls]
    for w in SHELF_W:
        m, c = _shelf(w)
        meshes += m
        colliders += c
    return tuple(meshes), tuple(colliders)


# The door: hinged at the cabinet's front right corner. Its link frame sits on the hinge axis; at 0 the door is shut
# (inner face 1 mm off the cabinet front), at +90 degrees it stands open along the fridge's right side, reaching back
# toward the arms, its outer face 0.64 cm outside the right side (46 cm wide with the door open).
HINGE = (-_HD - DOOR_T, -EXT_W / 2 - DOOR_OUT)  # cabinet frame (x, y) of the hinge axis
DOOR_START = 0.0  # the door starts shut, the self-closing hinge holding it
_DW = EXT_W + DOOR_OUT  # the door's width, hinge axis to its handle edge
_DOOR_BOX = ((DOOR_T - 0.001) / 2, DOOR_OUT + EXT_W / 2, (DOOR_Z[0] + DOOR_Z[1]) / 2)  # link frame
_DOOR_SIZE = (DOOR_T - 0.001, EXT_W, DOOR_Z[1] - DOOR_Z[0])
# The handle: a vertical bar on the door's free edge, standing off the door on a post at each end (the maker's photo),
# here an Amerock BP54008SS bar pull at its listed sizes: 3/4 in bar, 12 in between the posts' centres, 14 5/8 in long,
# 2 in projection.
BAR_D = 0.019  # the bar across
BAR_L = 0.371  # the bar's length overall
POST_CC = 0.305  # between the posts' centres
BAR_IN = 0.03  # the bar's centre line in from the door's free edge
BAR_X = -(HANDLE_OUT - BAR_D / 2)  # link frame x of the bar's centre line (the door's outer face at x 0)
BAR_ZC = (DOOR_Z[0] + DOOR_Z[1]) / 2
_BAR_C = (BAR_X, _DW - BAR_IN, BAR_ZC)
_POST_SIZE = (HANDLE_OUT - BAR_D / 2, 0.019, 0.019)  # each post, from the door's face to the bar's centre line
_POSTS = tuple(((BAR_X / 2, _DW - BAR_IN, BAR_ZC + sz * POST_CC / 2), _POST_SIZE) for sz in (-1, 1))
_HANDLE = ((-HANDLE_OUT / 2, _DW - BAR_IN, BAR_ZC), (HANDLE_OUT, BAR_D, BAR_L))  # the handle's bounding box
CLOSER_DEG = (30.0, 35.0)  # the self-closing hinge pulls the door shut once it is within 30 degrees of closed (full
# pull), fading out by 35 degrees; wider open, the door stays where it is left (the mock's Hettich 9073626 hinges
# list "a wide automatic closing angle of 35°")
CLOSER_K = 0.6  # N*m/rad: the closer's spring ...
CLOSER_REST = -0.5  # rad: ... toward this angle past shut, so it still presses the door shut at 0 (0.3 N*m)
CLOSER_DAMP = 1.0  # N*m*s/rad: the hinge's damping (the door swings shut in about a second from 30 degrees)
# The magnetic catch that holds the door shut (on the mock: an Everbilt 4.4 lb magnetic catch, 9235997, its counter
# plate on the door's inner face): it holds with up to its rated pull at contact, and its pull fades to nothing as the
# gap opens. A modeled law, against the robot: the door opens only once the pull at the catch passes the rating.
CATCH_N = 4.4 * 4.448  # N: the catch's rated pull (4.4 lb)
CATCH_R = (EXT_W + DOOR_OUT) - 0.020  # m: the catch's counter plate, this far from the hinge along the door (facing
# the catch recessed in the left side wall's front edge)
CATCH_GAP = 0.003  # m: the gap at the catch by which its pull has faded to nothing


def catch_torque(q: float) -> float:
    """The catch's pull on the door about the hinge (N*m, toward shut) with the door open q rad."""
    g = max(0.0, q) * CATCH_R  # the gap at the catch
    return CATCH_N * CATCH_R * max(0.0, 1.0 - g / CATCH_GAP)


class _Closer:
    """The ARS1PV's self-closing hinge, as PhysX's own (implicit) joint drive: a spring toward CLOSER_REST within
    CLOSER_DEG[0] of shut, faded to nothing by CLOSER_DEG[1]; beyond that the hinge only damps. Plus the magnetic
    catch: an effort toward shut, catch_torque(q), set each control step."""

    def drive(self, key, q, qd, J, mem):
        a, b = (math.radians(d) for d in CLOSER_DEG)
        f = 1.0 if q <= a else max(0.0, (b - q) / (b - a))
        return CLOSER_REST, CLOSER_K * f, -catch_torque(q), CLOSER_DAMP


_BAR_PTS, _BAR_FACES = _rod(BAR_L, BAR_D, 12)
_BAR_HULL = tuple((p[1], p[2], p[0]) for p in _BAR_PTS)  # the rod turned upright (x, y, z) -> (y, z, x)


def _door_parts():
    """The door slab, the handle's bar (a round 12-sided rod) and its two posts: meshes and colliders, link frame."""
    bar_mesh = kit._mesh([tuple(float(c + o) for c, o in zip(p, _BAR_C)) for p in _BAR_HULL], [3] * len(_BAR_FACES),
                         [j for f in _BAR_FACES for j in f], HANDLE_LOOK)
    meshes = [kit.box(_DOOR_BOX, _DOOR_SIZE, WHITE), bar_mesh] + [kit.box(c, sz, HANDLE_LOOK) for c, sz in _POSTS]
    colliders = [kit.Collider(_DOOR_BOX, size=_DOOR_SIZE), kit.Collider(_BAR_C, hull=_BAR_HULL, faces=_BAR_FACES)]
    colliders += [kit.Collider(c, size=sz) for c, sz in _POSTS]
    return tuple(meshes), tuple(colliders)


_DM, _DC = _door_parts()
DOOR = kit.Link(
    "door", "revolute", "Z", anchor=(HINGE[0], HINGE[1], 0.0), lower=0.0, upper=1.75, start=DOOR_START, mass=DOOR_KG,
    meshes=_DM, colliders=_DC, damping=CLOSER_DAMP, law=_Closer())

_M, _C = _cabinet()
FRIDGE = kit.Mechanism("fridge", pos=FRIDGE_POS, size=(BODY_D, EXT_W, EXT_H), meshes=_M, colliders=_C, links=(DOOR,),
                       friction=(0.5, 0.4))

# -- the cartons: Gardasil 9, carton of 10 single-dose vials, 8.6 x 3.95 x 4.45 cm. ------------------------------------
CARTON = (0.086, 0.0395, 0.0445)  # long (x), across (y), high (z), lying flat as packed (WHO PQ, PAHO pack)
CARTON_KG = 0.060
CARTON_FRICTION = (0.5, 0.4)  # paperboard on plastic and coated wire (static, dynamic)
CARTONS = ("carton_a", "carton_b", "carton_c")

# -- the tote, left of the fridge: a Sterilite 6 Qt storage box (inside 11.5 x 6 in at the bottom, 4.25 in deep;
#    outside 13.5 x 8 in, 4 5/8 in high), its long side across the robot, the lid off. The cartons lie flat in it in a
#    row, long side toward the fridge, 4 cm apart. Its walls flare out to the rim. -------------------------------------
TOTE_IN = (0.152, 0.292, 0.108)  # inside at the bottom (x, y) and depth
TOTE_OUT = (0.203, 0.343, 0.117)  # outside at the rim (x, y) and height
TOTE_T = 0.004  # wall and floor thickness
TOTE_POS = (0.33, 0.50, TOP)  # the middle of its underside
CARTON_START = tuple((TOTE_POS[0], TOTE_POS[1] + dy) for dy in (-0.08, 0.0, 0.08))  # (x, y), in pick order
_HEX_FACES = ((0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7),
              (2, 7, 6), (3, 0, 4), (3, 4, 7))  # a hexahedron's 12 triangles: bottom 0-3, top 4-7, same order


def _tote_walls():
    """Four flared walls (convex hexahedra) and the floor, in the tote's frame (origin under the middle)."""
    (lx, ly, h), (ox, oy, H), t = TOTE_IN, TOTE_OUT, TOTE_T
    ix, iy = ox / 2 - t, oy / 2 - t  # the inside half-sizes at the rim
    walls = []
    for sgn in (1, -1):  # the walls across x (at +-x) and along y (at +-y)
        walls.append([(sgn * lx / 2, -ly / 2, t), (sgn * (lx / 2 + t), -ly / 2 - t, t), (sgn * (lx / 2 + t), ly / 2 + t, t),
                      (sgn * lx / 2, ly / 2, t), (sgn * ix, -iy, H), (sgn * ox / 2, -oy / 2, H), (sgn * ox / 2, oy / 2, H),
                      (sgn * ix, iy, H)])
        walls.append([(-lx / 2, sgn * ly / 2, t), (-lx / 2 - t, sgn * (ly / 2 + t), t), (lx / 2 + t, sgn * (ly / 2 + t), t),
                      (lx / 2, sgn * ly / 2, t), (-ix, sgn * iy, H), (-ox / 2, sgn * oy / 2, H), (ox / 2, sgn * oy / 2, H),
                      (ix, sgn * iy, H)])
    return walls


def _tote() -> kit.Prop:
    (lx, ly, _), t = TOTE_IN, TOTE_T
    meshes = [kit.box((0.0, 0.0, t / 2), (lx + 2 * t, ly + 2 * t, t), TOTE_LOOK)]
    colliders = [kit.Collider((0.0, 0.0, t / 2), size=(lx + 2 * t, ly + 2 * t, t))]
    for pts in _tote_walls():
        meshes.append(kit._mesh(pts, [3] * 12, [j for f in _HEX_FACES for j in f], TOTE_LOOK))
        colliders.append(kit.Collider((0.0, 0.0, 0.0), hull=tuple(tuple(float(c) for c in p) for p in pts),
                                      faces=_HEX_FACES))
    return kit.Prop("tote", pos=TOTE_POS, size=(TOTE_OUT[0], TOTE_OUT[1], TOTE_OUT[2]), meshes=tuple(meshes),
                    colliders=tuple(colliders))


TOTE = _tote()


def _carton(name: str, xy) -> kit.Prop:
    return kit.Prop(name, pos=(xy[0], xy[1], TOP + TOTE_T + CARTON[2] / 2), size=CARTON, mass=CARTON_KG,
                    meshes=(kit.box((0.0, 0.0, 0.0), CARTON, CARTON_LOOK),
                            kit.box((-0.02, 0.0, 0.0), (0.025, CARTON[1] + 0.0006, CARTON[2] + 0.0006), BAND_LOOK)),
                    colliders=(kit.Collider((0.0, 0.0, 0.0), size=CARTON),))


OBJECTS = [FRIDGE, TOTE] + [_carton(n, xy) for n, xy in zip(CARTONS, CARTON_START)]
FRICTION = {n: CARTON_FRICTION for n in CARTONS}

# -- geometry helpers -------------------------------------------------------------------------------------------------
_CORNERS = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)], float) * np.array(CARTON) / 2
CARTON_PARTS = (gk.Block((0.0, 0.0, 0.0), CARTON),)


def rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def corners_world(objects, name: str) -> np.ndarray:
    s = objects[name]
    return np.asarray(s.pos, float) + _CORNERS @ rotm(s.quat_xyzw).T


def corners_fridge(objects, name: str) -> np.ndarray:
    """The carton's 8 corners in the fridge frame (u, v, w), from the cabinet's pose as the simulator reports it."""
    fr = objects["fridge"]
    R = rotm(fr.quat_xyzw)
    origin = np.asarray(fr.pos, float) + R @ np.array([-_HD, 0.0, FLOOR_UP])
    return (corners_world(objects, name) - origin) @ R


def clearances(objects, name: str) -> dict:
    """Gaps (m) from the carton to the door plane, back wall, side walls, ceiling and floor of the interior."""
    c = corners_fridge(objects, name)
    return {"door": float(c[:, 0].min()), "back": float(INT_D - c[:, 0].max()),
            "right": float(c[:, 1].min() + _HW), "left": float(_HW - c[:, 1].max()),
            "ceiling": float(INT_H - c[:, 2].max()), "floor": float(c[:, 2].min())}


def square(objects, name: str) -> bool:
    """One of the carton's axes within SQUARE_DEG of vertical: standing on end, on a side, or lying flat."""
    return float(np.abs(rotm(objects[name].quat_xyzw)[2, :]).max()) >= math.cos(math.radians(SQUARE_DEG))


def shelf_of(objects, name: str):
    """Which shelf (0 bottom, 1, 2 top) the carton rests on, or None: inside the interior, square, its lowest corner
    within ON_SHELF of that shelf's wire tops and its middle over the shelf (between its rails). A carton propped on
    another carton or on the shelf's rail edge, hanging under a shelf, or lying on the floor rests on none."""
    c = corners_fridge(objects, name)
    if not (c[:, 0].min() > 0.0 and c[:, 0].max() < INT_D and np.abs(c[:, 1]).max() < _HW and c[:, 2].max() < INT_H):
        return None
    mid = c.mean(axis=0)
    if not (SHELF_U[0] <= mid[0] <= SHELF_U[1]) or not square(objects, name):
        return None
    low = float(c[:, 2].min())
    for i, w in enumerate(SHELF_W):
        if ON_SHELF[0] <= low - w <= ON_SHELF[1]:
            return i
    return None


def compliant(objects, name: str) -> bool:
    """On a shelf with every gap at least GAP (5 cm): door plane, back wall, both side walls, ceiling and floor."""
    return shelf_of(objects, name) is not None and min(clearances(objects, name).values()) >= GAP


def door_angle(objects) -> float:
    """The door's opening (rad, 0 = shut)."""
    return kit.joint(objects, "fridge/door").pos


def door_shut(objects) -> bool:
    return door_angle(objects) <= math.radians(SHUT_DEG)


# -- the arms' own shapes: the collision hulls the simulator collides, every link from the shoulder to the fingertips
# (yamgeom: read at run time from the environment's robot model). The simulator reports each link's pose every step.
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}  # "left_robot/link5" -> (pos, R) in the env frame, this control step (empty offline)
CLEAR = 0.005  # m: a hand counts as off a carton when every arm link's hull is more than 5 mm from it
_HALF_DIAG = float(np.linalg.norm(CARTON)) / 2


def arm_gap(objects, name: str, upto: float = 0.02) -> float:
    """The smallest gap (m) between carton `name` and any arm link's own shape (exact up to `upto`)."""
    s = objects[name]
    oc, Rc = np.asarray(s.pos, float), rotm(s.quat_xyzw)
    lo, hi = -np.array(CARTON) / 2, np.array(CARTON) / 2
    best = math.inf
    for key, (o, R) in LINK_POSES.items():
        link = key.split("/")[1]
        _, c, r = yg.cloud(link)
        if float(np.linalg.norm(o + R @ c - oc)) - r - _HALF_DIAG > upto:  # link and carton bounds: too far apart
            continue
        q = yg.near(link, lambda x, o=o, R=R: ((o + x @ R.T) - oc) @ Rc,
                    lambda x: np.linalg.norm(np.maximum(np.maximum(lo - x, x - hi), 0.0), axis=1), upto)
        if not len(q):
            continue
        best = min(best, float(np.linalg.norm(np.maximum(np.maximum(lo - q, q - hi), 0.0), axis=1).min()))
    return best


def link_pose(p, q_xyzw) -> tuple:
    """A link's (position, rotation) from the simulator's body pose: Isaac Lab reports quaternions as (x, y, z, w)."""
    return np.asarray(p, float), rotm(q_xyzw)


def corner_speed(objects, name: str) -> float:
    """The fastest of the carton's 8 corners (m/s), from the simulator's linear and angular velocity (world frame)."""
    s = objects[name]
    v, w = np.asarray(s.lin_vel, float), np.asarray(s.ang_vel, float)
    r = _CORNERS @ rotm(s.quat_xyzw).T
    return float(np.linalg.norm(v + np.cross(w, r), axis=1).max())


def _door_boxes():
    """The door's slab, the handle's bar (its bounding box) and posts: (centre, size) in the door link's frame."""
    bar = np.asarray(_BAR_HULL)
    lo, hi = bar.min(axis=0), bar.max(axis=0)
    return [(np.asarray(_DOOR_BOX, float), np.asarray(_DOOR_SIZE, float)),
            (np.asarray(_BAR_C, float) + (lo + hi) / 2, hi - lo)] + [(np.asarray(c, float), np.asarray(sz, float))
                                                                   for c, sz in _POSTS]


DOOR_BOXES = _door_boxes()


def door_frame(objects):
    """The door link's frame in the env frame: (origin on the hinge axis, rotation), from the cabinet's pose as the
    simulator reports it and the door's angle."""
    fr = objects["fridge"]
    R = rotm(fr.quat_xyzw)
    a = door_angle(objects)
    Rz = np.array([[math.cos(a), -math.sin(a), 0.0], [math.sin(a), math.cos(a), 0.0], [0.0, 0.0, 1.0]])
    return np.asarray(fr.pos, float) + R @ np.array([HINGE[0], HINGE[1], 0.0]), R @ Rz


def _box_gap(x, c, sz):
    return np.linalg.norm(np.maximum(np.abs(x - c) - sz / 2, 0.0), axis=1)


def door_gap(objects, upto: float = 0.02) -> float:
    """The smallest gap (m) between the door (slab and handle) and any arm link's own shape (exact up to `upto`)."""
    o_d, R_d = door_frame(objects)
    centre = o_d + R_d @ np.asarray(_DOOR_BOX, float)
    reach = float(np.linalg.norm(_DOOR_SIZE)) / 2 + HANDLE_OUT
    best = math.inf
    for key, (o, R) in LINK_POSES.items():
        link = key.split("/")[1]
        _, c, r = yg.cloud(link)
        if float(np.linalg.norm(o + R @ c - centre)) - r - reach > upto:
            continue

        def dist(x):
            return np.min([_box_gap(x, bc, bs) for bc, bs in DOOR_BOXES], axis=0)

        q = yg.near(link, lambda x, o=o, R=R: ((o + x @ R.T) - o_d) @ R_d, dist, upto)
        if len(q):
            best = min(best, float(dist(q).min()))
    return best


HAND_R = 0.025  # m: offline tests only: a hand as a ball this big round its tool point


def door_hands_off(objects) -> bool:
    """No arm link within CLEAR of the door or its handle (the sim: every link's hulls; offline tests: each hand as a
    ball round its tool point)."""
    if LINK_POSES:
        return door_gap(objects) > CLEAR
    if _IN_SIM["on"]:
        raise RuntimeError("[vaccine] hands-off check: no arm link poses read from the simulator")
    o_d, R_d = door_frame(objects)
    for g in GRIPPERS:
        if g in objects:
            x = ((np.asarray(objects[g].pos, float) - o_d) @ R_d)[None, :]
            if min(float(_box_gap(x, bc, bs)[0]) for bc, bs in DOOR_BOXES) <= CLEAR + HAND_R:
                return False
    return True


def hands_off(objects, name: str) -> bool:
    """No arm link within CLEAR of the carton (the sim: every link's hulls; offline tests: guardkit's hand boxes)."""
    if LINK_POSES:
        return arm_gap(objects, name) > CLEAR
    if _IN_SIM["on"]:
        raise RuntimeError("[vaccine] hands-off check: no arm link poses read from the simulator")
    return not gk.touching(objects, name, CARTON_PARTS, CLEAR)


# -- one trial's record -------------------------------------------------------------------------------------------------
DROP_H = 0.010  # m: a carton whose middle comes to rest more than 1 cm below where a hand last carried it has been
# dropped
CARRY_SLACK = 0.002  # m per control step: how much faster than the hand a carton it touches may sink and still count
# as carried down (not slipping out of the fingers)
GRIPPERS = ("left_gripper", "right_gripper")  # the tool points the simulator reports each step
DROP_SPEED = 0.4  # m/s: a carton corner moving this fast with no hand on it has been thrown or knocked over
REST_SPEED = 0.02  # m/s: at rest, no corner moves faster than this (the simulator's velocities)
STILL_STEP = 0.0005  # m: per control step, the most a resting carton's corner moves
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)


class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.n = 0
        self.last = {}
        self.last_low = {}
        self.rest_n = {}  # carton -> control steps it has been at rest in a row (no hand on it)
        self.last_mid = {}  # carton -> its middle at the last control step
        self.last_hand = {}
        self.dropped = {}  # carton -> why (it then counts for nothing this trial)
        self.carried = {}  # carton -> its middle's height the last time a hand carried it (None once judged)
        self.good = 0
        self.door_max = 0.0  # the widest the door has been open this trial (rad)
        self.shut = True  # the door shut at the last control step
        self.door_free = True  # no arm link on the door at the last control step

    def step(self, objects) -> None:
        self.n += 1
        self.state = {}
        self.door_max = max(self.door_max, door_angle(objects))
        for c in CARTONS:
            pts = corners_world(objects, c)
            off = hands_off(objects, c)
            moved = math.inf
            speed = corner_speed(objects, c)  # the simulator's velocities, this physics step
            if c in self.last:
                moved = float(np.linalg.norm(pts - self.last[c], axis=1).max())
                speed = max(speed, moved / DT)  # and the average over the control step
            if off and self.n > 2 and speed > DROP_SPEED and c not in self.dropped:
                self.dropped[c] = f"moved {speed:.2f} m/s with no hand on it (dropped, thrown or knocked over)"
            low = float(pts[:, 2].mean())  # its middle's height (turning it in the hand barely moves its middle)
            if not off:  # a hand is on it: the carried height follows it up freely, and down only as far as the hand
                # goes down (plus CARRY_SLACK a step), so a carton slipping out of a still hand leaves it behind
                if self.carried.get(c) is None or low >= self.carried[c]:
                    self.carried[c] = low
                else:
                    g = min((h for h in GRIPPERS if h in objects),
                            key=lambda h: float(np.linalg.norm(np.asarray(objects[h].pos) - pts.mean(axis=0))),
                            default=None)
                    hand_down = 0.0  # how far the hand carried the carton's middle down this step, moving it rigidly
                    if g is not None and g in self.last_hand and c in self.last_mid:
                        p0, R0 = self.last_hand[g]
                        p1, R1 = np.asarray(objects[g].pos, float), rotm(objects[g].quat_xyzw)
                        pred = p1 + R1 @ R0.T @ (self.last_mid[c] - p0)
                        hand_down = max(0.0, float(self.last_mid[c][2] - pred[2]))
                    went = float(self.last_low.get(c, low)) - low
                    self.carried[c] = max(low, self.carried[c] - min(max(went, 0.0), hand_down + CARRY_SLACK))
            rest = moved <= STILL_STEP and corner_speed(objects, c) <= REST_SPEED
            self.rest_n[c] = self.rest_n.get(c, 0) + 1 if (rest and off) else 0
            if off and self.rest_n[c] >= 2 and self.carried.get(c) is not None:  # at rest again (two steps running:
                # a carton just let go can look still for one step), no hand on it
                fall = self.carried[c] - low
                if fall > DROP_H and c not in self.dropped:
                    self.dropped[c] = f"fell {fall * 1000:.0f} mm after the hand last carried it (dropped)"
                self.carried[c] = None
            self.last_low[c] = low
            self.last_mid[c] = pts.mean(axis=0)
            self.last[c] = pts
            self.state[c] = compliant(objects, c) and off and moved <= STILL_STEP and c not in self.dropped
        self.last_hand = {h: (np.asarray(objects[h].pos, float), rotm(objects[h].quat_xyzw)) for h in GRIPPERS
                          if h in objects}
        self.door_free = door_hands_off(objects)
        self.shut = door_shut(objects)
        done = all(self.state.values()) and door_shut(objects) and self.door_free
        self.good = self.good + 1 if done else 0
        if _TRACE:
            with open(_TRACE, "a") as f:
                f.write(f"{self.n} door_deg={math.degrees(door_angle(objects)):.2f} door_free={int(self.door_free)} "
                        f"door_max={math.degrees(self.door_max):.1f} " + " ".join(
                    f"{c}:shelf={shelf_of(objects, c)} ok={int(self.state[c])} gaps_mm="
                    f"{ {k: round(v * 1000) for k, v in clearances(objects, c).items()} }" for c in CARTONS)
                    + f" dropped={self.dropped} good={self.good}\n")

    def tier(self) -> float:
        if self.good >= HOLD_STEPS:
            return 1.0
        n = 0.25 * sum(self.state.values())
        if not self.shut:  # the door left open: at most 0.5, however many cartons are in
            n = min(n, DOOR_OPEN_CAP)
        return round(n, 4)


_L = Ledger()
_TRACE = os.environ.get("VACCINE_TRACE")


def _update(objects):
    """Advance the trial's record once per control step; success() and score() share it."""
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    _L.step(objects)


def success(objects):
    """All three cartons on a shelf at least 5 cm from the walls, floor, door and ceiling, none dropped, no hand on any
    of them, all still, and the door shut (within 0.5 degrees) with no arm link on it or its handle: held 1 s, the
    trial ends."""
    _update(objects)
    return _L.good >= HOLD_STEPS


def score(objects):
    """0.25 for each carton that rests square on a shelf at least 5 cm from the door plane, the back wall, both side
    walls, the ceiling and the floor, still, with no hand on it and never dropped; at most 0.5 while the door is not
    shut; 1.0 once all three are and the door is shut with no arm on it, held 1 s. A carton that was dropped, thrown or
    knocked over counts for nothing. Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Doing nothing scores 0."""
    _update(objects)
    return _L.tier()


# -- the simulator hooks -------------------------------------------------------------------------------------------------
_IN_SIM = {"on": False, "told": False}


def _patch(scene) -> None:
    if getattr(scene, "_vaccine", False):
        return
    scene._vaccine = True
    yg.warm(ARM_LINKS)  # read and check the hulls now, after the app is up (never when the task file loads)
    inner_object, inner_states = scene.task_object, scene.object_states

    def task_object(spec):
        obj = inner_object(spec)
        if spec.name in FRICTION:
            s, d = FRICTION[spec.name]
            obj.spawn_cfg_addon["physics_material"] = scene.sim_utils.RigidBodyMaterialCfg(static_friction=s,
                                                                                           dynamic_friction=d)
            obj.object_cfg = obj._init_object_cfg()
        return obj

    def object_states(env, names):
        out = inner_states(env, names)
        if all(c in out for c in CARTONS) and "fridge" in out:
            _IN_SIM["on"] = True
            _read_links(env.unwrapped)
        return out

    scene.task_object, scene.object_states = task_object, object_states


def _read_links(base) -> None:
    """Every arm link's pose (env frame) this control step, into LINK_POSES. A missing robot or link is an error."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"[vaccine] hands-off check: no robot {robot!r} in the scene") from e
        names = list(art.body_names)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"[vaccine] hands-off check: {robot} has no link(s) {missing} (bodies {names})")
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = link_pose(P[i] - origin, Q[i])
    if not _IN_SIM["told"]:
        _IN_SIM["told"] = True
        print(f"[vaccine] hands-off check: {len(LINK_POSES)} link poses from the sim, hull check on", file=sys.stderr,
              flush=True)


class _Hook(kit._Hook):
    def exec_module(self, module):
        self._inner.exec_module(module)
        kit._patch(module)
        _patch(module)


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:  # replace yamkit's import hook with one that adds ours after it
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook())


# -- scripted demo: open loop from the fixed layout -------------------------------------------------------------------
# Left hand, the door first: level, its open fingers pointing along the door toward the hinge, it slides them either
# side of the handle's bar (the inner one into the gap behind it), closes and pulls (the inner finger bears on the back
# of the bar); the catch lets go and the hand draws the door open along its arc to 40 degrees, then opens and slides
# its fingers back off the bar; wider open than the self-closing
# hinge's reach, the door stays. It reaches over the door's top to its inner face and pushes it open
# to 70 degrees with the edge of its closed fingertips, clear of the arm's way in. Then each carton: comes straight down over it in the tote, closes on its 3.95 cm sides (pads along its length),
# lifts it clear of the tote, turns to fingers-forward (the carton now stands on end, 4.45 cm front to back) at the
# shelf's height in front of the fridge, slides it in 8 mm above the wires to 6 cm past the door plane, sets it down,
# opens and backs straight out. Cartons a and b go on the bottom shelf, c on the middle one. Then the left arm parks
# out of the door's sweep, and the right hand, level and fingers closed, pushes the door round its arc with its
# fingertips from 70 to 10 degrees open and backs off; the self-closing hinge shuts it.
LEVEL = -math.pi / 2  # Tilted pitch: fingers forward, level
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open
WIDE = 0.032  # m: open half-gap round a 3.95 cm carton
LIFT = 0.008  # m: the carton's bottom this far above the wires while it slides in
PLACE_U = 0.06 + CARTON[2] / 2  # the standing carton's middle: its front face 6 cm in from the door plane
PLACES = ((0, 0.085), (0, -0.0121), (1, 0.0364))  # (shelf, v) for cartons a, b, c: each midway between two wires
STAND_U = -0.045  # tool point here (u) before sliding in: the carton's front 2.4 cm outside the door plane
PARK_L = (0.36, 0.42, TOP + 0.28)  # the left hand waits here, level, while the door closes (the door's sweep reaches
# the arm's start pose)
CARRY_Z = TOP + 0.15  # m: tool point height over the tote (the flat carton's bottom 1 cm over the rim)
TURN_Z = TOP + 0.24  # m: tool point height while the hand turns fingers-forward (the carton on end 8 cm over the rim)


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * max(0.0, half) / FINGER_HALF_OPEN


def _t(d, pos, f, label, pitch=0.0, yaw=0.0):
    return kit.Tilted(float(d), tuple(float(v) for v in pos), float(f), label, yaw, 0.0, pitch)


def place_tcp(i: int, lift: float = 0.002) -> np.ndarray:
    """World tool point when carton i stands in its place, its bottom `lift` above the wires (held at its middle)."""
    shelf, v = PLACES[i]
    return ORIGIN + np.array([PLACE_U, v, SHELF_W[shelf] + lift + CARTON[0] / 2])


OPEN_DEG = 40  # the left hand pulls the door open this far
GRIP_Z = TOP + 0.30  # the hand hooks the handle's bar at this height
HOOK_D = HANDLE_OUT - BAR_D / 2  # m: the tool point on the bar's centre line, its inner finger in the gap behind it
HOOK_WIDE = BAR_D / 2 + 0.012  # m: open half-gap round the bar
HOOK_LAT = 0.035  # m: the open hand comes in along the door from this far past the bar, toward its free edge
HOOK_TURN = math.radians(10)  # the fingers point along the door toward the hinge, turned 10 degrees off it


def bar_point(theta: float) -> np.ndarray:
    """World position of the handle bar's centre line at GRIP_Z with the door open theta."""
    hx, hy = FRIDGE_POS[0] + HINGE[0], FRIDGE_POS[1] + HINGE[1]
    along = np.array([-math.sin(theta), math.cos(theta)])
    inward = np.array([math.cos(theta), math.sin(theta)])  # into the door (its outer face's inward normal)
    p = np.array([hx, hy]) + (_DW - BAR_IN) * along + BAR_X * inward
    return np.array([p[0], p[1], GRIP_Z])


def hook_point(theta: float) -> np.ndarray:
    """World tool point of the hooking hand with the door open theta: in the finger gap behind the bar, level with it
    along the door."""
    hx, hy = FRIDGE_POS[0] + HINGE[0], FRIDGE_POS[1] + HINGE[1]
    along = np.array([-math.sin(theta), math.cos(theta)])
    inward = np.array([math.cos(theta), math.sin(theta)])
    p = np.array([hx, hy]) + (_DW - BAR_IN) * along - HOOK_D * inward
    return np.array([p[0], p[1], GRIP_Z])


def hook_turn(theta: float) -> float:
    return theta - math.pi / 2 + HOOK_TURN


def _hand(d, pos, f, label, turn):
    """Left hand level, fingers forward, pointed `turn` round from straight ahead."""
    return kit.Tilted(float(d), tuple(float(v) for v in pos), float(f), label, 0.0, 0.0, LEVEL, float(turn))


def door_open_plan() -> list:
    """Hook the handle's bar: the open hand, its fingers pointing along the door toward the hinge, slides them either
    side of the bar from the free-edge side (the inner finger into the gap behind the bar) and closes; it pulls the door
    open along its arc, the inner finger bearing on the back of the bar (a pull by contact, the grip's force holding
    the finger behind the bar), then opens and slides the fingers back off the bar."""
    t0 = hook_turn(0.0)
    wide = finger_for(HOOK_WIDE)
    beside = hook_point(0.0) + np.array([0.0, HOOK_LAT, 0.0])  # past the bar's free-edge side (shut door)
    out = [_hand(4.0, beside + np.array([-0.06, 0.0, 0.0]), wide, "in front of the handle", t0),
           _hand(2.0, beside, wide, "fingers level with the bar", t0),
           _hand(2.0, hook_point(0.0), wide, "fingers either side of the bar, one in behind it", t0),
           _hand(1.0, hook_point(0.0), FINGER_CLOSED, "close round the bar", t0),
           _hand(0.5, hook_point(0.0), FINGER_CLOSED, "hold", t0)]
    for deg in range(5, OPEN_DEG + 1, 5):
        th = math.radians(deg)
        out.append(_hand(1.5 if deg == 5 else 0.9, hook_point(th), FINGER_CLOSED, "pull the door open", hook_turn(th)))
    th = math.radians(OPEN_DEG)
    turn = hook_turn(th)
    back = hook_point(th) - 0.045 * np.array([math.cos(turn), math.sin(turn), 0.0])
    out += [_hand(0.8, hook_point(th), wide, "let go: the door stays open", turn),
            _hand(1.5, back, wide, "fingers back off the bar", turn),
            _hand(2.5, back + np.array([0.0, 0.0, OVER_Z - GRIP_Z]), FINGER_CLOSED, "up, over the door's top", turn)]
    # then round the door's free edge to its inner face, and push it open the rest of the way with the side of the
    # closed fingers (angled 30 degrees off the face, so the fingertips' edge touches it)
    def inner(theta, off):
        hx, hy = FRIDGE_POS[0] + HINGE[0], FRIDGE_POS[1] + HINGE[1]
        along = np.array([-math.sin(theta), math.cos(theta)])
        inward = np.array([math.cos(theta), math.sin(theta)])
        p = np.array([hx, hy]) + PUSH_R * along + (DOOR_T + off) * inward
        return np.array([p[0], p[1], GRIP_Z])

    def pturn(theta):
        return theta - math.pi / 2 + math.radians(PUSH_DTURN)

    out += [_hand(8.0, inner(th, 0.05) + np.array([0.0, 0.0, OVER_Z - GRIP_Z]), FINGER_CLOSED,
                  "over the door to its inner side", pturn(th)),
            _hand(6.0, inner(th, 0.04), FINGER_CLOSED, "down inside the door", pturn(th)),
            _hand(1.0, inner(th, PUSH_OFF), FINGER_CLOSED, "fingers on the door's inner face", pturn(th))]
    for deg in range(OPEN_DEG + 10, PUSH_OPEN_DEG + 1, 10 if (PUSH_OPEN_DEG - OPEN_DEG) % 10 == 0 else 5):
        t2 = math.radians(deg)
        out.append(_hand(0.8, inner(t2, PUSH_OFF), FINGER_CLOSED, "push the door open", pturn(t2)))
    t2 = math.radians(PUSH_OPEN_DEG)
    out += [_hand(1.5, inner(t2, 0.05), FINGER_CLOSED, "off the door", pturn(t2)),
            _hand(4.0, (0.25, 0.25, TOP + 0.30), FINGER_CLOSED, "toward the tote", 0.0)]
    return out


OVER_Z = TOP + 0.58  # the hand passes over the door's top (50.2 cm) at this height
PUSH_R = 0.31  # m: the left hand pushes the door open this far out from the hinge, on its inner face
PUSH_DTURN = -30  # degrees: the hand's turn off the door's face while it pushes
PUSH_OFF = 0.012  # m: tool point off the inner face while pushing (the fingertips' edge on it)
PUSH_OPEN_DEG = 70  # the left hand pushes the door open this far



def left_plan() -> list:
    wide, shut = finger_for(WIDE), FINGER_CLOSED
    out = door_open_plan()
    for i, (cx, cy) in enumerate(CARTON_START):
        grip = np.array([cx, cy, TOP + TOTE_T + CARTON[2] / 2])
        above = np.array([cx, cy, CARRY_Z])
        place = place_tcp(i)
        up = place_tcp(i, LIFT)
        front = ORIGIN + np.array([STAND_U, up[1] - ORIGIN[1], up[2] - ORIGIN[2]])
        side = np.array([front[0] - 0.02, 0.36, max(up[2], TURN_Z)])  # beside the fridge's left side, turned
        out += [_t(3.0, above, wide, f"over carton {i + 1}"),
                _t(2.0, grip, wide, "down round it"),
                _t(1.2, grip, shut, "grip it"),
                _t(0.6, grip, shut, "hold"),
                _t(2.0, above, shut, "lift it out of the tote"),
                _t(4.0, side, shut, "turn fingers forward beside the fridge", LEVEL),
                _t(2.0, front, shut, "to the opening, at shelf height", LEVEL),
                _t(2.5, up, shut, "slide it in between the shelves", LEVEL),
                _t(1.0, place, shut, "set it down", LEVEL),
                _t(1.0, place, wide, "let go", LEVEL),
                _t(2.0, front - np.array([0.02, 0.0, 0.0]), wide, "back out", LEVEL),
                _t(3.0, side + np.array([0.0, 0.0, 0.03]), wide, "clear of the fridge", LEVEL)]
    out += [_t(3.0, PARK_L, wide, "park left, out of the door's sweep", LEVEL)]
    return out


DOOR_R = 0.15  # m: the right hand pushes the door this far out from its hinge
PUSH_Z = TOP + 0.30  # tool point height while pushing (the fingertips on the door's outer face)
TIP_OFF = 0.015  # m: tool point to the door's outer face while pushing (the fingertips 1.3 cm past the tool point)
TURN_MAX = math.radians(60)  # the hand points at most 60 degrees round from straight ahead (the arm's reach)


def door_push_point(theta: float, off: float) -> np.ndarray:
    """World tool point `off` m outside the door's outer face, DOOR_R along it from the hinge, door open theta."""
    hx, hy = FRIDGE_POS[0] + HINGE[0], FRIDGE_POS[1] + HINGE[1]
    along = np.array([-math.sin(theta), math.cos(theta)])
    out = np.array([-math.cos(theta), -math.sin(theta)])  # the outer face's normal
    p = np.array([hx, hy]) + DOOR_R * along + off * out
    return np.array([p[0], p[1], PUSH_Z])


def _push(d, theta, off, label):
    """Right hand level, fingers closed and pointing into the door's outer face (turned with the door, at most
    TURN_MAX round), the tool point `off` outside the face with the door open theta."""
    return kit.Tilted(float(d), tuple(float(v) for v in door_push_point(theta, off)), FINGER_CLOSED, label, 0.0, 0.0,
                      LEVEL, min(theta, TURN_MAX))


def right_plan() -> list:
    wait = sum(w.duration for w in left_plan())
    out = [kit.Home(wait, FINGER_CLOSED, "wait for the cartons"),
           kit.Tilted(3.0, (0.32, -0.30, PUSH_Z), FINGER_CLOSED, "level, fingers forward", 0.0, 0.0, LEVEL, 0.0),
           _push(3.0, math.radians(PUSH_OPEN_DEG), TIP_OFF + 0.02, "behind the door's outer face"),
           _push(1.5, math.radians(PUSH_OPEN_DEG), TIP_OFF, "fingertips on the door")]
    for deg in (55, 40, 30, 20, 10):  # pushed to 10 degrees; the closer takes it from there
        out.append(_push(1.4, math.radians(deg), TIP_OFF, "push the door round"))
    out += [_push(1.5, math.radians(10), TIP_OFF + 0.08, "back off the door"), kit.Home(3.0, FINGER_CLOSED)]
    return out


def demo():
    """Left hand: the three cartons, two on the bottom shelf and one on the middle shelf, each 6 cm in; then the right
    hand pushes the door shut."""
    return {"left": left_plan(), "right": right_plan()}
