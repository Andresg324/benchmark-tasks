"""In a home garage, a ratcheting jack stand (a Big Red T43002 3-ton steel jack stand, one of its pair) stands on the
workbench (the station table) in front of the arms, its saddle at its lowest notch, its locking handle hanging down at
the front. The job: raise the saddle three notches, let it settle on the ratchet, then push the locking handle down to
check that the pawl is fully home, as the maker's manual says: "Adjust height by pulling up on ratchet bar. The weight
of the locking handle should secure the ratchet bar in desired position. To confirm this, simply push down on the
locking handle."

The stand is one articulation with a free base (its four-leg stamped-steel frame and square column head, which can
slide or tip on the bench), the ratchet bar with its saddle on a vertical slide in the column, and the locking handle
with its pawl on a pin through the column head. The bar's teeth and the pawl's tip are convex hulls on the two links,
so the bar is held at a notch only by the pawl's tip under a tooth: lifting the bar cams the pawl out on each tooth's
sloped face, and the handle's own weight swings it back in under the next tooth. Nothing else holds the bar up.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import gripkit  # noqa: E402
import yamgeom as yg  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = ("Raise the jack stand's saddle three notches from its lowest position, let it settle on the ratchet, "
               "then push the locking handle down to check that it is locked, then let go.")

# -- the stand (its own frame: origin on the bench under the column's axis, z up, x away from the robot; the locking
#    handle hangs at the front, toward the robot) --------------------------------------------------------------------
# Big Red T43002 (Torin), 3-ton steel jack stand: published lifting range 11-1/4 to 16-3/4 in (minimum and maximum
# saddle height, Torin's operator's manual). Its other sizes are estimated from maker photos: a four-leg pyramid frame
# about 9 in square at the feet, a square column head, a 26 mm square ratchet bar with eleven notch positions spread
# evenly over the published range, a U saddle, and a bent-rod locking handle on a pin through the column head.
IN = 0.0254
H_MIN, H_MAX = 11.25 * IN, 16.75 * IN  # the saddle's top above the floor at the lowest and the highest notch
N_NOTCH = 10  # notches above the lowest position (eleven positions)
PITCH = (H_MAX - H_MIN) / N_NOTCH  # 13.97 mm between notches
STEEL_KG_M3 = 7850.0  # density of steel: the masses below come from the parts' volumes
FOOT_HALF = 0.115  # the feet span 230 mm square
PLATE_T = 0.0025  # the frame's stamped sheet
COL_Z0, COL_TOP = 0.150, 0.240  # the column head, from the top of the legs to its rim
BAR_HALF = 0.013  # the bar is 26 mm square
TOOTH_D = 0.005  # each tooth stands 5 mm proud of the bar's front face
ROOT_X = -BAR_HALF + TOOTH_D  # the bottom of a notch (the bar's front face is at x = -BAR_HALF)
UNDERCUT_DEG = 20.0  # each tooth's load face leans back 20 degrees, so a load pulls the pawl in, not out
TOOTH_LAND = 0.0005  # each tooth's narrow flat outer land, below its sloped top
TOOTH_H = 0.0084  # each tooth's height at the root (load face to where its slope meets the root): the slope rises
# 58 degrees from horizontal, steep enough that a rising tooth pushes the pawl out rather than jamming against it
BAR_BOT = 0.020  # the bar's bottom end at the lowest notch
SAD_Z0 = COL_TOP + 0.0005  # the saddle's underside at the lowest notch, just over the column's rim: at its lowest
# position the bar rests on the pawl by its lowest tooth (its highest position is its slide joint's limit)
SAD_PLATE = 0.010
SAD_X, SAD_Y = 0.035, 0.028  # the saddle: 70 mm long (along its groove, x) and 56 mm wide over its two ears (y)
EAR_T = 0.008
COL_IN_Y, COL_IN_BACK, COL_IN_FRONT = BAR_HALF + 0.0015, BAR_HALF + 0.0015, -0.031  # the column head's inside
COL_WALL = 0.003
Z_PT = 0.165  # the pawl's top edge at the notch's root, at the lowest notch (where each tooth comes to rest)
# the handle's pin, through the column head's sides: high above the pawl's tip and only just in front of the teeth, so a
# rising tooth's push on the tip has a long lever about the pin and the bar lifts with little more than its own weight
# (a pin low and well forward of the tip lets friction on the ramp cancel half that lever: the bar then needs about
# 9 N more than its weight to cam the pawl out, and the cam's reaction shoves the stand)
PIVOT = (ROOT_X - 0.0115, 0.0, Z_PT + 0.055)
PAWL_GAP = 0.0005  # the seated pawl's tip stops this far short of the notch's root
PAWL_HALF_W = 0.006  # the pawl plate is 12 mm wide
ROD_R = 0.005  # the handle is 10 mm steel rod
SLEEVE_R, SLEEVE_HALF = 0.008, 0.025  # its grip sleeve, 50 mm long on the front bar
HANDLE_REACH, HANDLE_DROP = 0.100, 0.050  # the front bar is this far forward of and below the pin
HANDLE_HALF_W = 0.045  # the loop's front bar is 90 mm wide
HANDLE_UP = 0.45  # rad: the handle lifts this far (the pawl clear of the teeth from about 0.09 rad)
HANDLE_KG = 0.25  # the rod loop, sleeve and pawl plate
STAND_XY = (0.40, -0.06)
TAN_U = math.tan(math.radians(UNDERCUT_DEG))

# Friction (static, dynamic, combine). Painted steel on painted steel, dry: 0.4 / 0.35 (steel on steel, clean and dry,
# is about 0.5-0.8; paint lowers it), PhysX averaging the two surfaces. The YAM's finger pads are rubber-faced, 1.0 / 0.8
# (gripkit), so a pad grips the saddle at about 0.7 / 0.6 (averaged), and the stand's feet hold on the bench.
STEEL = (0.40, 0.35, "average")
PAINT = kit.Look((0.78, 0.08, 0.07), 0.45)  # Big Red
BAR_LOOK = kit.Look((0.80, 0.10, 0.08), 0.4)
ROD_LOOK = kit.Look((0.80, 0.10, 0.08), 0.35)
SLEEVE_LOOK = kit.Look((0.12, 0.12, 0.12), 0.7)
DARK = kit.Look((0.25, 0.04, 0.03), 0.6)


def hull(points) -> kit.Collider:
    """A convex-hull collider of `points` (center = their centroid, triangles from their convex hull)."""
    from scipy.spatial import ConvexHull

    p = np.asarray(points, float)
    c = p.mean(axis=0)
    rel = p - c
    faces = []
    for t in ConvexHull(p).simplices:
        a, b, d = rel[t[0]], rel[t[1]], rel[t[2]]
        n = np.cross(b - a, d - a)
        faces.append(tuple(int(i) for i in (t if n @ ((a + b + d) / 3) >= 0 else t[::-1])))
    return kit.Collider(tuple(float(v) for v in c), hull=tuple(tuple(float(v) for v in q) for q in rel),
                        faces=tuple(faces))


def hull_mesh(col: kit.Collider, look: kit.Look) -> kit.Mesh:
    """The visible mesh of a hull collider (its own triangles)."""
    c = np.asarray(col.center, float)
    pts = [tuple(c + np.asarray(q, float)) for q in col.hull]
    return kit._mesh(pts, [3] * len(col.faces), [i for f in col.faces for i in f], look)


def extrude_xz(poly, y0, y1):
    """Points of an xz polygon extruded across y0..y1."""
    return [(x, y, z) for (x, z) in poly for y in (y0, y1)]


def rod(a, b, r, n=8):
    """Points of an n-sided rod of radius r from a to b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    u = (b - a) / np.linalg.norm(b - a)
    v = np.cross(u, [0.0, 0.0, 1.0] if abs(u[2]) < 0.9 else [1.0, 0.0, 0.0])
    v /= np.linalg.norm(v)
    w = np.cross(u, v)
    ring = [r * (math.cos(2 * math.pi * k / n) * v + math.sin(2 * math.pi * k / n) * w) for k in range(n)]
    return [tuple(e + q) for e in (a, b) for q in ring]


def box_pts(lo, hi):
    return [(x, y, z) for x in (lo[0], hi[0]) for y in (lo[1], hi[1]) for z in (lo[2], hi[2])]


def hull_volume(col: kit.Collider) -> float:
    from scipy.spatial import ConvexHull

    return float(ConvexHull(np.asarray(col.hull, float)).volume)


# -- the frame: four sloped leg plates up to the column head, and the column head's four walls ---------------------------
_TOP_LO, _TOP_HI = (COL_IN_FRONT - COL_WALL, -COL_IN_Y - COL_WALL), (COL_IN_BACK + COL_WALL, COL_IN_Y + COL_WALL)


def _leg(side):
    """One leg plate: from a foot edge up to the matching edge of the column head's bottom, PLATE_T thick."""
    f = FOOT_HALF
    feet = {"front": [(-f, -f), (-f, f)], "back": [(f, -f), (f, f)], "left": [(-f, f), (f, f)],
            "right": [(-f, -f), (f, -f)]}[side]
    tops = {"front": [(_TOP_LO[0], _TOP_LO[1]), (_TOP_LO[0], _TOP_HI[1])],
            "back": [(_TOP_HI[0], _TOP_LO[1]), (_TOP_HI[0], _TOP_HI[1])],
            "left": [(_TOP_LO[0], _TOP_HI[1]), (_TOP_HI[0], _TOP_HI[1])],
            "right": [(_TOP_LO[0], _TOP_LO[1]), (_TOP_HI[0], _TOP_LO[1])]}[side]
    inward = {"front": (1, 0), "back": (-1, 0), "left": (0, -1), "right": (0, 1)}[side]
    pts = []
    for (x, y), z in [(p, 0.0) for p in feet] + [(p, COL_Z0) for p in tops]:
        pts += [(x, y, z), (x + inward[0] * PLATE_T * 1.4, y + inward[1] * PLATE_T * 1.4, z)]
    return hull(pts)


_FRAME_COLS = [_leg(s) for s in ("front", "back", "left", "right")]
_COL_WALLS = [
    hull(box_pts((COL_IN_FRONT - COL_WALL, -COL_IN_Y - COL_WALL, COL_Z0), (COL_IN_FRONT, COL_IN_Y + COL_WALL, COL_TOP))),
    hull(box_pts((COL_IN_BACK, -COL_IN_Y - COL_WALL, COL_Z0), (COL_IN_BACK + COL_WALL, COL_IN_Y + COL_WALL, COL_TOP))),
    hull(box_pts((COL_IN_FRONT, COL_IN_Y, COL_Z0), (COL_IN_BACK, COL_IN_Y + COL_WALL, COL_TOP))),
    hull(box_pts((COL_IN_FRONT, -COL_IN_Y - COL_WALL, COL_Z0), (COL_IN_BACK, -COL_IN_Y, COL_TOP))),
]
BASE_COLS = tuple(_FRAME_COLS + _COL_WALLS)
# the leg plates have large triangular cut-outs (about a third of each plate, from the photos): the frame's mass
BASE_KG = STEEL_KG_M3 * (sum(hull_volume(c) for c in _FRAME_COLS) * 0.67 / 1.4 + sum(hull_volume(c) for c in _COL_WALLS))
_BASE_MESHES = tuple(hull_mesh(c, PAINT) for c in BASE_COLS)


# -- the ratchet bar and its saddle (link "bar": a slide along z; its frame is the stand's at the lowest notch) -----------
def tooth_bottom(k: int) -> float:
    """Height (bar frame) of tooth k's load face at the root: tooth k rests on the pawl when the bar is k notches up."""
    return Z_PT - k * PITCH


def tooth_poly(k: int):
    b = tooth_bottom(k)
    return [(ROOT_X, b), (-BAR_HALF, b - TOOTH_D * TAN_U), (-BAR_HALF, b + TOOTH_LAND), (ROOT_X, b + TOOTH_H)]


_BAR_CORE = hull(box_pts((ROOT_X, -BAR_HALF, BAR_BOT), (BAR_HALF, BAR_HALF, SAD_Z0)))
_TEETH = [hull(extrude_xz(tooth_poly(k), -BAR_HALF, BAR_HALF)) for k in range(N_NOTCH + 1)]
_SAD_PLATE = hull(box_pts((-SAD_X, -SAD_Y, SAD_Z0), (SAD_X, SAD_Y, SAD_Z0 + SAD_PLATE)))
_EARS = [hull(box_pts((-SAD_X, s * SAD_Y - (EAR_T if s > 0 else 0), SAD_Z0 + SAD_PLATE),
                      (SAD_X, s * SAD_Y + (EAR_T if s < 0 else 0), H_MIN))) for s in (1, -1)]
BAR_COLS = tuple([_BAR_CORE] + _TEETH + [_SAD_PLATE] + _EARS)
BAR_KG = STEEL_KG_M3 * sum(hull_volume(c) for c in BAR_COLS)
BAR = kit.Link("bar", "prismatic", "Z", anchor=(0.0, 0.0, 0.0), lower=-0.002, upper=N_NOTCH * PITCH + 0.004, start=0.0,
               mass=BAR_KG, damping=0.5,
               meshes=tuple(hull_mesh(c, DARK if c in _TEETH else BAR_LOOK) for c in BAR_COLS),
               colliders=BAR_COLS)


# -- the locking handle and its pawl (link "handle": a hinge about y through the pin; joint 0 = the pawl fully home,
#    positive = the handle lifted, the pawl swung out of the teeth) -----------------------------------------------------
def pawl_top(x: float) -> float:
    """The pawl's top edge (stand frame, handle down): parallel to the teeth's undercut load faces."""
    return Z_PT - (ROOT_X - x) * TAN_U


_PX, _, _PZ = PIVOT
_TIP_X = ROOT_X - PAWL_GAP
_ARM_X0, _ARM_X1 = _PX - 0.0035, _PX + 0.0035
_PAWL_TIP = [(_TIP_X, pawl_top(_TIP_X)), (_TIP_X, Z_PT - 0.0045), (_ARM_X0, Z_PT - 0.016), (_ARM_X0, pawl_top(_ARM_X0))]
_PAWL_ARM = [(_ARM_X0, Z_PT - 0.016), (_ARM_X1, Z_PT - 0.016), (_ARM_X1, _PZ + 0.004), (_ARM_X0, _PZ + 0.004)]
_FX, _FZ = _PX - HANDLE_REACH, _PZ - HANDLE_DROP  # the loop's front bar (stand frame, handle down)
_ARM_Y0 = COL_IN_Y + COL_WALL + ROD_R  # the loop's ends ride on the pin just outside the column head's sides
HANDLE_SEGS = [((_PX, -_ARM_Y0, _PZ), (_PX, _ARM_Y0, _PZ), 0.004),  # the pin (turns with the handle)
               ((_PX, _ARM_Y0, _PZ), (_FX, HANDLE_HALF_W, _FZ), ROD_R),
               ((_PX, -_ARM_Y0, _PZ), (_FX, -HANDLE_HALF_W, _FZ), ROD_R),
               ((_FX, -HANDLE_HALF_W, _FZ), (_FX, HANDLE_HALF_W, _FZ), ROD_R),
               ((_FX, -SLEEVE_HALF, _FZ), (_FX, SLEEVE_HALF, _FZ), SLEEVE_R)]


def _to_handle(pts):
    return [tuple(np.asarray(p, float) - np.asarray(PIVOT, float)) for p in pts]


_HANDLE_COLS = [hull(_to_handle(extrude_xz(_PAWL_TIP, -PAWL_HALF_W, PAWL_HALF_W))),
                hull(_to_handle(extrude_xz(_PAWL_ARM, -PAWL_HALF_W, PAWL_HALF_W)))]
_HANDLE_COLS += [hull(_to_handle(rod(a, b, r))) for a, b, r in HANDLE_SEGS]
HANDLE = kit.Link("handle", "revolute", "Y", anchor=PIVOT, lower=0.0, upper=HANDLE_UP, start=0.0, mass=HANDLE_KG,
                  damping=0.02,
                  meshes=tuple(hull_mesh(c, SLEEVE_LOOK if i == len(_HANDLE_COLS) - 1 else ROD_LOOK)
                               for i, c in enumerate(_HANDLE_COLS)),
                  colliders=tuple(_HANDLE_COLS))

STAND = kit.Mechanism(
    "stand", pos=(STAND_XY[0], STAND_XY[1], TOP), mass=BASE_KG, size=(2 * FOOT_HALF, 2 * FOOT_HALF, H_MAX),
    meshes=_BASE_MESHES, colliders=BASE_COLS, links=(BAR, HANDLE), self_collide=True,
    # the bar slides in the column on its joint, and the handle's pin rides in the column's holes and its pawl through
    # a slot in the column's front wall: those parts do not collide with the frame (PhysX also never collides links
    # that share a joint). The bar's teeth and the pawl do collide, and they alone hold the bar up, at every position
    # including the lowest (the joint's lower limit, 2 mm under it, stands in for the saddle landing on the column's
    # rim: the bar reaches it only if the handle is held up and the bar dropped).
    no_collide=(("base", "bar"), ("base", "handle")),
    material=STEEL, contact_offset=0.001)

OBJECTS = [STAND]
gripkit.use_rubber_pads()  # the YAM's finger pads are rubber-faced

# -- scored thresholds ----------------------------------------------------------------------------------------------------
TARGET = 3  # notches up from the lowest position
AT_NOTCH = 0.0015  # m: the bar rests at notch k when within 1.5 mm of k notches up (the pawl's play is under 0.5 mm)
SEATED = 0.025  # rad: the pawl fully home: the handle within 1.4 degrees of its down stop (its tip within about
# 1.5 mm of its full 4.5 mm engagement, at least 3 mm still under the tooth)
TILT_MAX_DEG = 5.0  # the stand tipped: its column more than 5 degrees off vertical at any moment
TOUCH = 0.0006  # m: an arm link touches a part when within 0.6 mm of it (the arm's hull points lie within 0.5 mm of
# its surface)
PRESS_STEPS = 5  # control steps (0.25 s) in a row that a hand must stay on top of the grip for a push: a graze is not
# a push
CLEAR = 0.005  # m: hands clear: every arm link at least 5 mm from the bar, the saddle, the handle and the column head
STILL_STEP = 0.0005  # m per control step (1 cm/s) for the bar and the frame; the handle 0.005 rad per step
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}  # "right_robot/gripper" -> (pos, R) in the env frame at this control step (filled in the sim)


def rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


# -- reading the stand ------------------------------------------------------------------------------------------------------
def bar_q(objects) -> float:
    return float(kit.joint(objects, "stand/bar").pos)


def handle_q(objects) -> float:
    return float(kit.joint(objects, "stand/handle").pos)


def tilt_deg(objects) -> float:
    """Angle (degrees) between the stand's column and straight up."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rotm(objects["stand"].quat_xyzw)[2, 2])))))


def notch(q: float):
    """The notch k (1..10) the bar rests at, or None (at its bottom, or between notches)."""
    k = int(round(q / PITCH))
    return k if 1 <= k <= N_NOTCH and abs(q - k * PITCH) <= AT_NOTCH else None


def locked_notch(objects):
    """The notch the bar is locked at: resting at notch k with the pawl fully home under tooth k; else None."""
    k = notch(bar_q(objects))
    return k if k is not None and handle_q(objects) <= SEATED else None


# -- the outlines the arms are checked against (each in its part's frame) ---------------------------------------------------
_BAR_BOXES = [((-BAR_HALF, -BAR_HALF, BAR_BOT), (BAR_HALF, BAR_HALF, SAD_Z0)), ((-SAD_X, -SAD_Y, SAD_Z0), (SAD_X, SAD_Y, H_MIN))]
_COL_BOX = [(_TOP_LO + (COL_Z0,), _TOP_HI + (COL_TOP,))]
_H_SEGS = [(np.asarray(a, float) - PIVOT, np.asarray(b, float) - PIVOT, r) for a, b, r in HANDLE_SEGS[1:]]
_GRIP_SEGS = _H_SEGS[2:]  # the loop's front bar and its sleeve: where a hand pushes the handle down


def _box_gap(q, boxes):
    g = np.full(len(q), np.inf)
    for lo, hi in boxes:
        lo, hi = np.asarray(lo, float), np.asarray(hi, float)
        d = np.maximum(np.maximum(lo - q, q - hi), 0.0)
        g = np.minimum(g, np.linalg.norm(d, axis=1))
    return g


def _seg_gap(q, segs, top_only=False):
    """Distance from points q to rods (a, b, r); with top_only, only points above a rod's axis count (a hand on top)."""
    g = np.full(len(q), np.inf)
    for a, b, r in segs:
        u = b - a
        s = np.clip(((q - a) @ u) / (u @ u), 0.0, 1.0)
        near = a + s[:, None] * u
        d = np.maximum(np.linalg.norm(q - near, axis=1) - r, 0.0)
        if top_only:
            d = np.where(q[:, 2] >= near[:, 2], d, np.inf)
        g = np.minimum(g, d)
    return g


def part_frames(objects) -> dict:
    """(origin, rotation, gap function, reach) of each checked part, world (env) frame."""
    b, h, s = objects["stand/bar"], objects["stand/handle"], objects["stand"]
    return {"bar": (np.asarray(b.pos, float), rotm(b.quat_xyzw), lambda q: _box_gap(q, _BAR_BOXES), 0.32),
            "column": (np.asarray(s.pos, float), rotm(s.quat_xyzw), lambda q: _box_gap(q, _COL_BOX), 0.26),
            "handle": (np.asarray(h.pos, float), rotm(h.quat_xyzw), lambda q: _seg_gap(q, _H_SEGS), 0.13),
            "grip_top": (np.asarray(h.pos, float), rotm(h.quat_xyzw), lambda q: _seg_gap(q, _GRIP_SEGS, True), 0.13)}


def link_gap(key, frame) -> float:
    """Smallest gap (m) between arm link `key` ("right_robot/tip_left") as posed and a part; 0.02 when farther."""
    o, R = LINK_POSES[key]
    co, Rc, gap, reach = frame
    link = key.split("/")[1]
    _, cen, rad = yg.cloud(link)
    o_, R_ = np.asarray(o, float), np.asarray(R, float)
    centre = (o_ + R_ @ cen - co) @ Rc
    if float(np.linalg.norm(centre)) - rad > reach + 0.02:
        return 0.02
    q = yg.near(link, lambda x: (o_ + x @ R_.T - co) @ Rc, gap, 0.02)
    return min(0.02, float(gap(q).min())) if len(q) else 0.02


def arm_gaps(objects) -> dict:
    """Smallest gap (m) from any arm link of either arm to each part (0.02 when none is within 2 cm)."""
    frames = part_frames(objects)
    return {name: min((link_gap(k, f) for k in LINK_POSES), default=0.02) for name, f in frames.items()}


# -- the trial's ledger ------------------------------------------------------------------------------------------------------
class Ledger:
    """One trial: the tipped latch, the handle check (pushed down on top while the bar sat locked at notch 3), and the
    final state's still/hold count."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.tipped = False
        self.checked = False
        self.press = 0
        self.clear = False
        self.lock = None
        self.last = None
        self.good = 0

    def step(self, objects) -> None:
        q, th, tilt = bar_q(objects), handle_q(objects), tilt_deg(objects)
        if tilt > TILT_MAX_DEG:
            self.tipped = True
        gaps = arm_gaps(objects)
        self.lock = locked_notch(objects)
        if self.lock != TARGET:
            self.checked = False  # a check counts only for the bar's current rest at notch 3
            self.press = 0
        elif gaps["grip_top"] <= TOUCH and gaps["bar"] > CLEAR:
            self.press += 1  # a hand on top of the handle's grip, the handle at its stop, the bar locked at 3 and let go
            if self.press >= PRESS_STEPS:
                self.checked = True
        else:
            self.press = 0
        self.clear = min(gaps["bar"], gaps["column"], gaps["handle"]) > CLEAR
        base = np.asarray(objects["stand"].pos, float)
        still = self.last is not None and (abs(q - self.last[0]) <= STILL_STEP and abs(th - self.last[1]) <= 0.005
                                           and float(np.linalg.norm(base - self.last[2])) <= STILL_STEP)
        self.last = (q, th, base)
        self.good = self.good + 1 if (self.tier() == 0.5 and self.checked and still) else 0

    def tier(self) -> float:
        """The end state's tier before the handle check: 0.5 locked at notch 3, 0.25 locked at another notch, else 0;
        each with every arm link clear of the bar, saddle, handle and column head and the stand never tipped."""
        if self.tipped or not self.clear or self.lock is None:
            return 0.0
        return 0.5 if self.lock == TARGET else 0.25

    def score(self) -> float:
        s = self.tier()
        if s == 0.5 and self.checked and self.good >= HOLD_STEPS:
            return 1.0
        return s


_L = None


def _read_links(base) -> None:
    """Every arm link's pose (env frame) at this control step, into LINK_POSES. A missing arm or link, or finger pads
    left without their rubber, stops the trial: the contact checks and the grip would otherwise pass silently."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"jack stand: no {robot} in the scene for the contact checks") from e
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"jack stand: {robot} has no link(s) {missing} for the contact checks")
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
    if gripkit._STATE.get("bound", 0) < 0:
        raise RuntimeError("jack stand: the finger pads' rubber material could not be bound")
    if not _READ_ONCE:
        _READ_ONCE.append(1)
        print(f"jack stand: contact checks on {len(LINK_POSES)} arm links; pad shapes with rubber: "
              f"{gripkit._STATE.get('bound')}", file=sys.stderr, flush=True)


_READ_ONCE: list = []
kit.STATE_HOOKS.append(_read_links)


def _update(objects):
    global _L
    if _L is None or _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else ("obj", id(objects))
    if key == _L.key:
        return
    _L.key = key
    _L.step(objects)


def success(objects):
    """The bar locked at notch 3 (resting on the pawl, the pawl fully home), the handle pushed down to check it, every
    arm link clear of the bar, saddle, handle and column head, the stand never tipped, all still: this step counts
    toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """1.0: the bar locked three notches up (resting on the pawl's tip, the pawl fully home), the handle pushed down on
    top while the bar sat locked there and no hand held it, then every arm link at least 5 mm clear of the bar, saddle,
    handle and column head, the stand never tipped
    past 5 degrees, all still for 1 s. 0.5: locked at notch 3 and let go, but the handle never pushed down to check it
    (or not yet still for 1 s). 0.25: locked at another notch and let go. 0: anything else, including doing nothing (the
    bar starts at its lowest position), the bar held up by a hand or left between notches, the pawl not fully home, a
    hand still on the bar, saddle, handle or column head, or the stand tipped past 5 degrees at any moment."""
    _update(objects)
    return _L.score()


# -- scripted demo: open loop from the fixed layout ---------------------------------------------------------------------------
# Two hands, as a person does it: one steadies the stand, the other raises the saddle. The left hand comes in from the
# robot's left, pointing right, and grips the column head front to back, low on it (below the handle's arms), then
# bears down a little so the stand neither slides nor rocks on the bench while the bar is pulled up. The right hand,
# pointing forward (level, away from the robot) with its fingers closing across the saddle (world y), comes in from the
# robot's side over the handle, takes the saddle by its two ears and lifts the bar. Each tooth's sloped face pushes the
# pawl out as it passes and the handle's weight swings the pawl back in under it. It lifts until the third tooth is
# past the pawl (about half a notch more), lowers the bar onto the pawl, lets go and backs out; the left lets go of the
# stand and backs out too. Then the right, its fingers closed and turned so they lie along the handle's grip, pushes
# the handle down on top of its sleeve, holds a moment, lifts off and goes home.
GRIP_Z = 0.268  # m: the right's tool point this high above the bench, level with the middle of the saddle's ears
GRIP_DX = 0.005  # m: and this far past the saddle's middle (the pads' centers sit behind the tool point)
PITCH_FWD = -math.pi / 2  # a hand pointing forward (+x)
SQUEEZE = 0.008  # m a side: the right closes this far past the ears' outer faces (about 40 N a pad)
OVER = PITCH / 2  # m: lifted this far past the target notch (the pawl drops in under the tooth after about 0.5 mm)
SAG = 0.103  # m: the lift is aimed this far higher: the simulated arm droops under the bar's weight and the cams' push
LEAN = 0.010  # m: and this far back (toward the robot) and to the right while lifting: the loaded arm gives forward and
# to the left, and its pull would otherwise rock the stand as each tooth cams the pawl out
SET_PRESS = 0.003  # m: when setting the bar down, aimed this far below its notch (the arm's give presses it on)
PUSH = 0.0  # m: the closed fingertips' tip point aimed this far below the top of the handle's sleeve (at this pitch
# the fingertips' lowest corner reaches about 4 mm past that point, so they press about 4 mm into the sleeve's top)
PRESS_DX = 0.006  # m: aimed this far past the grip's axis, so the corner lands on the sleeve's top
PRESS_PITCH = -math.pi / 4  # the hand pointing forward and 45 degrees down while it pushes the handle
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open
# the left hand steadying the stand: pointing right (-y), level, its fingers closing front to back on the column head
STEADY_YAW, STEADY_ROLL = -math.pi / 2, -math.pi / 2
COL_XC = (COL_IN_FRONT - COL_WALL + COL_IN_BACK + COL_WALL) / 2  # the column head's middle, front to back
COL_HALF_X = (COL_IN_BACK - COL_IN_FRONT) / 2 + COL_WALL  # half its depth, 25.75 mm
STEADY_Z = 0.172  # m: the left's tool point this high above the bench, low on the column head (below the handle's arms)
STEADY_DY = 0.006  # m: and this far left of the column's middle (the fingertips just past it)
STEADY_SQ = 0.006  # m a side: the left closes this far past the column head's front and back faces
STEADY_OPEN = 0.006  # m a side: and comes in opened only this far past them, so its front finger passes under the
# handle's arm
STEADY_DOWN = 0.004  # m: once gripping, the left bears down this far: it presses the stand onto the bench


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * min(FINGER_HALF_OPEN, max(0.0, half)) / FINGER_HALF_OPEN


def _tw(duration, pos, finger, label, yaw=0.0, pitch=0.0, roll=0.0):
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label, yaw=float(yaw),
                      pitch=float(pitch), roll=float(roll))


def _steady(release_at: float) -> list:
    """The left hand's plan: grip the column head, bear down, hold until `release_at` s, let go, back out, go home."""
    sx, sy = STAND_XY
    x, y, z = sx + COL_XC, sy + STEADY_DY, TOP + STEADY_Z
    open_, hold = finger_for(COL_HALF_X + STEADY_OPEN), finger_for(COL_HALF_X - STEADY_SQ)
    a = dict(yaw=STEADY_YAW, roll=STEADY_ROLL)
    plan = [kit.Home(0.5, label="start"),
            _tw(3.0, (x, y + 0.12, z - 0.004), open_, "left of the stand's column head, pointing right", **a),
            _tw(2.0, (x, y, z), open_, "fingers either side of the column head, below the handle", **a),
            _tw(1.0, (x, y, z), hold, "grip the column head", **a),
            _tw(0.5, (x, y, z - STEADY_DOWN), hold, "bear down: the stand held on the bench", **a)]
    held = sum(w.duration for w in plan)
    plan += [_tw(max(0.5, release_at - held), (x, y, z - STEADY_DOWN), hold, "hold the stand steady", **a),
             _tw(1.0, (x, y, z), open_, "let go of the stand", **a),
             _tw(2.0, (x, y + 0.12, z - 0.004), open_, "back out to the left", **a),
             kit.Home(4.0, label="back to the ready pose"), kit.Home(3.0, label="wait")]
    return plan


def demo():
    """The left steadies the stand by its column head while the right lifts the saddle three notches and sets the bar
    on the pawl; both let go, then the right pushes the handle down.
    JACK_VARIANT=nothing: both arms stay home. JACK_VARIANT=nopush: no handle check. JACK_VARIANT=two: lifted two
    notches. JACK_VARIANT=brace: the bar lifted to notch 3 plus half a notch and held there by the hand to the end."""
    variant = os.environ.get("JACK_VARIANT", "")
    if variant == "nothing":
        return {"left": [kit.Home(10.0, label="stay at the ready pose")],
                "right": [kit.Home(10.0, label="stay at the ready pose")]}
    notches = 2 if variant == "two" else TARGET
    sx, sy = STAND_XY
    g0 = TOP + GRIP_Z
    open_, hold = finger_for(FINGER_HALF_OPEN), finger_for(SAD_Y - SQUEEZE)
    up = notches * PITCH
    gx0 = sx + GRIP_DX
    pf = PITCH_FWD
    right = [kit.Home(0.5, label="start"),
             _tw(3.0, (gx0 - 0.10, sy, g0 + 0.03), open_, "behind the saddle, pointing forward", pitch=pf),
             _tw(1.5, (gx0 - 0.10, sy, g0), open_, "level with its ears", pitch=pf),
             _tw(2.0, (gx0, sy, g0), open_, "fingers either side of the saddle's ears", pitch=pf),
             _tw(0.5, (gx0, sy, g0), open_, "settle", pitch=pf),
             _tw(1.3, (gx0, sy, g0), hold, "grip the saddle", pitch=pf),
             _tw(0.5, (gx0, sy, g0), hold, "settle", pitch=pf),
             _tw(4.0, (gx0 - LEAN, sy - LEAN, g0 + up + OVER + SAG), hold,
                 f"lift {notches} notches and half a notch more", pitch=pf),
             _tw(1.0, (gx0 - LEAN, sy - LEAN, g0 + up + OVER + SAG), hold, "the pawl drops in under the tooth",
                 pitch=pf)]
    lifted = sum(w.duration for w in right)
    release_at = lifted + 6.3  # the left lets go once the right has set the bar down, let go and backed out
    if variant == "brace":
        right.append(_tw(20.0, (gx0 - LEAN, sy - LEAN, g0 + up + OVER + SAG), hold, "hold the bar up", pitch=pf))
        return {"left": _steady(release_at), "right": right}
    right += [_tw(2.5, (gx0, sy, g0 + up - SET_PRESS), hold, "set the bar down on the pawl", pitch=pf),
              _tw(0.8, (gx0, sy, g0 + up - SET_PRESS), hold, "settle", pitch=pf),
              _tw(1.0, (gx0, sy, g0 + up - SET_PRESS), open_, "let go", pitch=pf),
              _tw(2.0, (gx0 - 0.10, sy, g0 + up), open_, "back out, clear of the saddle", pitch=pf)]
    if variant != "nopush":
        # the closed fingertips, the hand pointing forward and down at PRESS_PITCH, on top of the handle's sleeve
        ax = np.array([-math.sin(PRESS_PITCH), 0.0, -math.cos(PRESS_PITCH)])  # the hand's pointing direction

        def tips_at(z):
            return tuple(np.array([sx + _FX + PRESS_DX, sy, z]) - PAD_TIP_PAST_TCP * ax)

        top = TOP + _FZ + SLEEVE_R
        pp = PRESS_PITCH
        right += [_tw(3.0, tips_at(top + 0.04), FINGER_CLOSED, "over the handle's grip", pitch=pp),
                  _tw(1.5, tips_at(top + 0.005), FINGER_CLOSED, "down to the grip", pitch=pp),
                  _tw(1.5, tips_at(top - PUSH), FINGER_CLOSED, "push the handle down", pitch=pp),
                  _tw(1.0, tips_at(top - PUSH), FINGER_CLOSED, "hold", pitch=pp),
                  _tw(1.5, tips_at(top + 0.04), FINGER_CLOSED, "lift off", pitch=pp)]
    right += [kit.Home(4.0, label="back to the ready pose"), kit.Home(3.0, label="wait")]
    return {"left": _steady(release_at), "right": right}
