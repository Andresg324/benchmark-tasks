"""On a kitchen counter (the station table) stands a new, full 3 L box of wine: a Black Box 3 L carton, a tall
5 x 5 x 9.5 in tower of corrugated board with the wine in a sealed bag inside. Near the bottom of its front panel is a
perforated tab. Behind it, tucked inside, the bag's tap (a Vitop-style tap on the bag's spout) lies where it was
packed. To get the box ready to pour, the tab is pushed in until its perforations tear, the tap is drawn out through
the opening, and its neck is set down into the die-cut slot under the opening, so the tap stands outside the box,
pointing out and spout down, with the panel held between its two flanges.

The model:
- The carton stands free on the counter: a full box, 3.25 kg, that slides or tips if pushed hard enough. The full bag
  inside is rigid: it fills the carton except a pocket behind the opening, where the tap lies.
- The tab is a plate in the opening. It is held by its perforations: a stiff spring that lets it give 2 mm under an
  8 N push and tears (the spring is switched off for good) once the push reaches 8 N. Torn, it slides in freely,
  drops by its own weight into the box and can tip back about its bottom edge.
- The tap is a rigid body joined to the bag by a chain of slides and hinges, free within the reach of the bag's film:
  up to 2 cm further out than the seated position, 3.5 cm to either side, 8 cm down, 3 cm up, pitched up to 34
  degrees and rolled about its axis (no yaw joint). Nothing pulls it anywhere: it moves by contact and gravity only.
- The slot is a 24 mm wide notch under the opening. The tap's neck (20 mm) goes down into it; its flanges, 7 mm
  apart, straddle the 3 mm panel. Everything collides by contact; nothing snaps the tap into place.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import gripkit  # noqa: E402
import yamgeom as yg  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Get the box of wine ready to pour."
_PROBE = os.environ.get("WINEBOX_PROBE")  # a file: one line of readouts per control step (probe runs only)

# -- the carton (its own frame: origin on the front face's bottom edge, at the middle of its width, on the counter;
#    x into the box, away from the robot; z up) ------------------------------------------------------------------
# Black Box 3 L: a 5 x 5 x 9.5 in "tower" carton (Packaging World, "Boxed wine aims high"). 3 L of wine (about
# 0.99 kg/L) plus the bag, tap and carton: 3.25 kg. Corrugated board 3 mm thick (B flute). The opening's and the
# slot's sizes and positions are estimated from maker photos.
X_C, Y_C = 0.33, -0.08  # world x, y of the carton frame's origin
CD, CW, CH = 0.127, 0.127, 0.241
PT = 0.003  # the board's thickness
HOLE_Y = 0.048  # the opening: |y| <= 4.8 cm ...
HOLE_Z = (0.072, 0.148)  # ... from 7.2 to 14.8 cm over the counter
NOTCH_Y = 0.012  # the slot under the opening: 24 mm wide ...
NOTCH_Z0 = 0.054  # ... its bottom 5.4 cm over the counter (it runs up into the opening)
TAB_GAP = 0.0015  # m: the tab's clearance in the opening on every side (its perforations' cut, widened so nothing
# touches at spawn)
# Corrugated board on a laminate counter, and board on board: 0.45 static, 0.40 dynamic (estimated; paperboard runs
# 0.3-0.5). "min": the lower of the two touching surfaces.
BOARD = (0.45, 0.40, "min")
# The tap is polypropylene and HDPE (Vitop product sheet); its friction, 0.35 / 0.30, is estimated (plastics on board
# or rubber run about 0.2-0.5). "average" with the rubber finger pads.
PLASTIC = (0.35, 0.30, "average")
CARTON_LOOK = kit.Look((0.06, 0.06, 0.065), 0.6)
EDGE_LOOK = kit.Look((0.55, 0.50, 0.42), 0.9)  # the board's cut edge (kraft)
PERF_LOOK = kit.Look((0.80, 0.78, 0.72), 0.7)  # the perforation line printed round the tab
BAG_LOOK = kit.Look((0.62, 0.63, 0.66), 0.3, 0.6)  # the bag's metallised film
TAP_LOOK = kit.Look((0.05, 0.05, 0.05), 0.4)
RED = kit.Look((0.75, 0.08, 0.06), 0.45)
IVORY = kit.Look((0.92, 0.89, 0.80), 0.5)


def _bx(lo, hi, look=None, collide=True, material=None):
    """A drawn box (None: not drawn) and its box collider (None: no collider), from corner `lo` to corner `hi`."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c, s = tuple(float(v) for v in (lo + hi) / 2), tuple(float(v) for v in hi - lo)
    return (kit.box(c, s, look) if look is not None else None,
            kit.Collider(c, size=s, material=material) if collide else None)


_HW = CW / 2
_PANEL = [  # the front panel, round the opening and the slot
    _bx((0, -_HW, 0), (PT, -HOLE_Y, CH), CARTON_LOOK),
    _bx((0, HOLE_Y, 0), (PT, _HW, CH), CARTON_LOOK),
    _bx((0, -HOLE_Y, HOLE_Z[1]), (PT, HOLE_Y, CH), CARTON_LOOK),
    _bx((0, -HOLE_Y, 0), (PT, -NOTCH_Y, HOLE_Z[0]), CARTON_LOOK),
    _bx((0, NOTCH_Y, 0), (PT, HOLE_Y, HOLE_Z[0]), CARTON_LOOK),
    _bx((0, -NOTCH_Y, 0), (PT, NOTCH_Y, NOTCH_Z0), CARTON_LOOK),
    # the opening's and the slot's cut edges (drawn only)
    _bx((-0.0002, -HOLE_Y - 0.001, HOLE_Z[0] - 0.001), (0.0, -NOTCH_Y, HOLE_Z[0]), EDGE_LOOK, collide=False),
    _bx((-0.0002, NOTCH_Y, HOLE_Z[0] - 0.001), (0.0, HOLE_Y + 0.001, HOLE_Z[0]), EDGE_LOOK, collide=False),
    _bx((-0.0002, -HOLE_Y - 0.001, HOLE_Z[1]), (0.0, HOLE_Y + 0.001, HOLE_Z[1] + 0.001), EDGE_LOOK, collide=False),
    _bx((-0.0002, -HOLE_Y - 0.001, HOLE_Z[0]), (0.0, -HOLE_Y, HOLE_Z[1]), EDGE_LOOK, collide=False),
    _bx((-0.0002, HOLE_Y, HOLE_Z[0]), (0.0, HOLE_Y + 0.001, HOLE_Z[1]), EDGE_LOOK, collide=False),
    _bx((-0.0002, -NOTCH_Y - 0.001, NOTCH_Z0 - 0.001), (0.0, NOTCH_Y + 0.001, NOTCH_Z0), EDGE_LOOK, collide=False),
    _bx((-0.0002, -NOTCH_Y - 0.001, NOTCH_Z0), (0.0, -NOTCH_Y, HOLE_Z[0]), EDGE_LOOK, collide=False),
    _bx((-0.0002, NOTCH_Y, NOTCH_Z0), (0.0, NOTCH_Y + 0.001, HOLE_Z[0]), EDGE_LOOK, collide=False),
]
_SHELL = [  # the other five walls
    _bx((CD - PT, -_HW, 0), (CD, _HW, CH), CARTON_LOOK),
    _bx((PT, -_HW, 0), (CD - PT, -_HW + PT, CH), CARTON_LOOK),
    _bx((PT, _HW - PT, 0), (CD - PT, _HW, CH), CARTON_LOOK),
    _bx((PT, -_HW + PT, 0), (CD - PT, _HW - PT, PT), CARTON_LOOK),
    _bx((PT, -_HW + PT, CH - PT), (CD - PT, _HW - PT, CH), CARTON_LOOK),
]
_IN = _HW - PT
# The full bag fills the carton except a pocket behind the opening: under the tap's spout (from 4 cm in), under its
# flanges (from 8.85 cm in) and above the opening. Drawn as the bag's film where it shows through the opening.
BAG_LOW_X, BAG_LOW_Z = 0.040, 0.081
BAG_STEP_X, BAG_STEP_Z = 0.0885, 0.097
BAG_TOP_Z = 0.153
_BAG = [
    _bx((BAG_LOW_X, -_IN, PT), (CD - PT, _IN, BAG_LOW_Z), BAG_LOOK),
    _bx((BAG_STEP_X, -_IN, BAG_LOW_Z), (CD - PT, _IN, BAG_STEP_Z), BAG_LOOK),
    _bx((PT, -_IN, BAG_TOP_Z), (CD - PT, _IN, CH - PT), BAG_LOOK),
    _bx((CD - PT - 0.004, -_IN, BAG_STEP_Z), (CD - PT, _IN, BAG_TOP_Z), BAG_LOOK),
]

# -- the tab ---------------------------------------------------------------------------------------------------------
# The perforated tab fills the opening (1.5 mm cut all round), board 3 mm thick: 93 x 73 mm at about 0.45 kg/m^2
# (B-flute corrugated board), 3.1 g. Its perforations hold it until a push on it reaches F_TEAR (estimated: a firm
# finger push; no published figure), giving TAB_GIVE before they tear.
F_TEAR = 8.0  # N
TAB_GIVE = 0.002  # m
TAB_K = F_TEAR / TAB_GIVE  # N/m: the perforations' stiffness
TAB_KG = 0.0031
TAB_C = (PT / 2, 0.0, (HOLE_Z[0] + HOLE_Z[1]) / 2)  # the tab's middle (carton frame)
TAB_W, TAB_H = 2 * (HOLE_Y - TAB_GAP), HOLE_Z[1] - HOLE_Z[0] - 2 * TAB_GAP
TAB_IN = 0.060  # m: how far in it can slide (the bag's face stops it first)


class Perforation:
    """The tab's perforations as a law on its in-out slide: until torn, a stiff spring holding it flush (TAB_K, so a
    push of F_TEAR gives TAB_GIVE); once the push it carries (TAB_K x how far it has given) reaches F_TEAR, the
    perforations are torn for the rest of the trial and the tab slides freely (light damping only)."""

    def __init__(self, k, force, damp=4.0, free_damp=0.05):
        self.k, self.force, self.damp, self.free_damp = k, force, damp, free_damp

    def fresh(self, lk):
        return {"torn": False, "peak": 0.0}

    def drive(self, key, q, qd, J, mem):
        f = self.k * max(q, 0.0)
        mem["peak"] = max(mem["peak"], f)
        if not mem["torn"] and f >= self.force:
            mem["torn"] = True
        if mem["torn"]:
            return q, 0.0, 0.0, self.free_damp
        return 0.0, self.k, 0.0, self.damp


class TabHold:
    """The perforations also hold the tab square in the opening: until they are torn (the tab_x law), a stiff
    rotational spring keeps it from tipping; torn, it is free to fall back into the box about its bottom edge."""

    def __init__(self, k, damp=0.02, free_damp=0.0005):
        self.k, self.damp, self.free_damp = k, damp, free_damp

    def drive(self, key, q, qd, J, mem):
        if tab_torn():
            return q, 0.0, 0.0, self.free_damp
        return 0.0, self.k, 0.0, self.damp


TAB_TIP_K = 15.0  # N m/rad: the perforations' hold on the tab's tilt (an 8 N push at its middle tilts it 1 degree)
TAB_HINGE = (PT / 2, 0.0, HOLE_Z[0] + TAB_GAP)  # the tab's bottom edge (carton frame): torn, it tips back about it
_TZ = TAB_H / 2  # the tab's middle above its bottom edge
_TAB_M = [kit.box((0.0, 0.0, _TZ), (PT, TAB_W, TAB_H), CARTON_LOOK)]
for _s in (-1, 1):  # the perforation line round the tab's edge, printed on its outside face
    _TAB_M.append(kit.box((-PT / 2 - 0.0002, _s * (TAB_W / 2 - 0.002), _TZ), (0.0003, 0.002, TAB_H - 0.002), PERF_LOOK))
    _TAB_M.append(kit.box((-PT / 2 - 0.0002, 0.0, _TZ + _s * (TAB_H / 2 - 0.002)), (0.0003, TAB_W - 0.002, 0.002),
                          PERF_LOOK))
TAB_LINKS = (
    kit.Link("tab_x", "prismatic", "X", anchor=TAB_C, lower=0.0, upper=TAB_IN, start=0.0, mass=0.001, damping=0.0,
             law=Perforation(TAB_K, F_TEAR)),
    kit.Link("tab_z", "prismatic", "Z", anchor=TAB_C, lower=-0.080, upper=0.0, start=0.0, mass=0.001, damping=0.02,
             parent="tab_x"),
    # it tips back into the box (top inward) up to 80 degrees; forward it would meet the panel's inside
    kit.Link("tab", "revolute", "Y", anchor=TAB_HINGE, lower=0.0, upper=math.radians(80.0), start=0.0, mass=TAB_KG,
             damping=0.0, parent="tab_z", law=TabHold(TAB_TIP_K), meshes=tuple(_TAB_M),
             colliders=(kit.Collider((0.0, 0.0, _TZ), size=(PT, TAB_W, TAB_H)),)),
)

# -- the tap -----------------------------------------------------------------------------------------------------------
# Vitop tap (Smurfit Kappa product sheet, amkFa/VT): tap 10.2 g, spout 6.1 g, PP / HDPE / elastomer, LDPE spout;
# black with red side tabs, ivory spout. Its sizes are estimated from maker photos. In the tap's own frame (at the
# neck's middle; x along its axis, into the box; the tap points out along -x, its outlet down):
NECK_R, NECK_HALF = 0.010, 0.0035  # the neck between the flanges: 20 mm across, 7 mm long
FLANGE_HALF, FLANGE_T = 0.015, 0.0035  # the flanges: 30 mm squares (simple shapes: the real ones are round), 3.5 mm
BODY = ((-0.043, -NECK_HALF - FLANGE_T), (-0.016, 0.016), (-0.017, 0.017))  # the tap's body, x, y, z
SPOUT_X, SPOUT_R, SPOUT_Z = -0.032, 0.006, (-0.031, -0.017)  # the outlet under the body
GLAND_R, GLAND_X = 0.012, (NECK_HALF + FLANGE_T, NECK_HALF + FLANGE_T + 0.010)  # the bag's spout behind the flanges
TAP_KG = 0.0163
# the neck's middle at the start (carton frame): the tap's front 4.2 cm behind the panel's face, spawned 2 mm over
# the bag it then settles onto
SPIGOT_START = (0.085, 0.0, 0.114)
SEAT = (PT / 2, 0.0, NOTCH_Z0 + NECK_R)  # the neck's middle when seated: on the slot's bottom, in the panel's plane

_TAP_PARTS = [
    _bx((BODY[0][0], BODY[1][0], BODY[2][0]), (BODY[0][1], BODY[1][1], BODY[2][1]), TAP_LOOK, material=PLASTIC),
    _bx((-NECK_HALF - FLANGE_T, -FLANGE_HALF, -FLANGE_HALF), (-NECK_HALF, FLANGE_HALF, FLANGE_HALF), None,
        material=PLASTIC),
    _bx((NECK_HALF, -FLANGE_HALF, -FLANGE_HALF), (NECK_HALF + FLANGE_T, FLANGE_HALF, FLANGE_HALF), None,
        material=PLASTIC),
]
_TAP_M = [m for m, _ in _TAP_PARTS if m is not None] + [
    kit.box((-NECK_HALF - FLANGE_T / 2, 0, 0), (FLANGE_T, 2 * FLANGE_HALF, 2 * FLANGE_HALF), TAP_LOOK),
    kit.box((NECK_HALF + FLANGE_T / 2, 0, 0), (FLANGE_T, 2 * FLANGE_HALF, 2 * FLANGE_HALF), TAP_LOOK),
    kit.cylinder((0.0, 0, 0), NECK_R, 2 * NECK_HALF, IVORY, axis="x"),
    kit.cylinder(((GLAND_X[0] + GLAND_X[1]) / 2, 0, 0), GLAND_R, GLAND_X[1] - GLAND_X[0], IVORY, axis="x"),
    kit.cylinder((SPOUT_X, 0, (SPOUT_Z[0] + SPOUT_Z[1]) / 2), SPOUT_R, SPOUT_Z[1] - SPOUT_Z[0], IVORY),
    kit.box((-0.026, -0.01625, 0.004), (0.020, 0.0005, 0.012), RED),  # the red press tabs, flush on its sides
    kit.box((-0.026, 0.01625, 0.004), (0.020, 0.0005, 0.012), RED),
]
_TAP_C = [c for _, c in _TAP_PARTS] + [
    kit.Collider((0.0, 0.0, 0.0), radius=NECK_R, height=2 * NECK_HALF, axis="X", material=PLASTIC),
    kit.Collider(((GLAND_X[0] + GLAND_X[1]) / 2, 0, 0), radius=GLAND_R, height=GLAND_X[1] - GLAND_X[0], axis="X",
                 material=PLASTIC),
    kit.Collider((SPOUT_X, 0, (SPOUT_Z[0] + SPOUT_Z[1]) / 2), radius=SPOUT_R, height=SPOUT_Z[1] - SPOUT_Z[0],
                 material=PLASTIC),
]
# The bag's film: how far the tap can go from where it was packed (slides x, y, z, then pitch and roll at its neck;
# no yaw: a chain six joints deep blows up at spawn in this simulator, and seated, the flanges on the panel hold the
# tap square anyway). The film near the spout moves with it: 2 g on each of the four joints between bag and tap.
OUT_SLACK = 0.020  # m: the tap can come this much further out than its seated position
REACH = {"x": (-(SPIGOT_START[0] - SEAT[0]) - OUT_SLACK, 0.0), "y": (-0.035, 0.035), "z": (-0.080, 0.030),
         "pitch": (-0.6, 0.6), "roll": (-3.1, 3.1)}
FILM_KG = 0.002
SPIGOT_LINKS = (
    kit.Link("sp_x", "prismatic", "X", anchor=SPIGOT_START, lower=REACH["x"][0], upper=REACH["x"][1], mass=FILM_KG,
             damping=0.05),
    kit.Link("sp_y", "prismatic", "Y", anchor=SPIGOT_START, lower=REACH["y"][0], upper=REACH["y"][1], mass=FILM_KG,
             damping=0.05, parent="sp_x"),
    kit.Link("sp_z", "prismatic", "Z", anchor=SPIGOT_START, lower=REACH["z"][0], upper=REACH["z"][1], mass=FILM_KG,
             damping=0.05, parent="sp_y"),
    kit.Link("sp_pitch", "revolute", "Y", anchor=SPIGOT_START, lower=REACH["pitch"][0], upper=REACH["pitch"][1],
             mass=FILM_KG, damping=0.0005, parent="sp_z"),
    kit.Link("spigot", "revolute", "X", anchor=SPIGOT_START, lower=REACH["roll"][0], upper=REACH["roll"][1],
             mass=TAP_KG, damping=0.0005, parent="sp_pitch", meshes=tuple(_TAP_M), colliders=tuple(_TAP_C)),
)

_BASE = _PANEL + _SHELL + _BAG
# The carton is free on the counter: 3.25 kg (3 L of wine at about 0.99 kg/L, plus the bag, tap and carton), its mass
# spread over its walls and the full bag. With board on the counter at 0.45 it takes about 14 N to slide it; tipping it
# takes about 16 N pushed in at the tab's height. It spawns 1 mm over the counter and settles onto it.
CARTON_KG = 3.25
SPAWN_LIFT = 0.001
CARTON = kit.Mechanism(
    "carton", pos=(X_C, Y_C, TOP + SPAWN_LIFT), size=(CD, CW, CH), mass=CARTON_KG,
    meshes=tuple(m for m, _ in _BASE if m is not None), colliders=tuple(c for _, c in _BASE if c is not None),
    links=TAB_LINKS + SPIGOT_LINKS, self_collide=True, material=BOARD, contact_offset=0.001)
OBJECTS = [CARTON]
gripkit.use_rubber_pads()  # the YAM's finger pads are rubber-faced

# -- scored thresholds -----------------------------------------------------------------------------------------------
# m: the groove straddles the panel: its middle within 3 mm of the panel's
SEAT_X_TOL = (2 * NECK_HALF - PT) / 2 + 0.001
SEAT_Z_TOL = 0.003  # m: the neck down on the slot's bottom: its middle at most 3 mm above where it rests there
TILT_MAX = math.radians(15.0)  # the tap's axis within 15 degrees of level ...
AIM_MAX = math.radians(30.0)  # ... pointing out of the box within 30 degrees of straight out
UPRIGHT = math.cos(math.radians(30.0))  # its outlet down: its "up" within 30 degrees of the carton's up
TOUCH = 0.002  # m: a hand link within 2 mm of the tap holds or touches it
CLEAR = 0.005  # m: hands clear of the tap: every hand link at least 5 mm from it
STILL_STEP = 0.001  # m per control step (2 cm/s): the tap and the carton still
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
TAB_T, OUT_T = 0.25, 0.5
KNOCKED_OVER = math.radians(45.0)  # the carton leaning this far from upright at any time: knocked over (score 0)
SLID = 0.05  # m: the carton's footprint ending this far (horizontally) from where it stood at the start: shoved off
# its place (dragged or pushed across the counter instead of steadied)
STANDING = math.radians(10.0)  # at the end the carton must stand on the counter: leaning at most this much ...
LIFTED = 0.005  # m: ... and its lowest bottom corner at most this far above the start (not held up off the counter)
HAND_LINKS = ("link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}  # "right_robot/tip_left" -> (pos, R) in the env frame at this control step (filled in the sim)


def rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _quat(R):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    R = np.asarray(R, float)
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = 2 * math.sqrt(1 + tr)
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4)
    if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[0, 0] - R[1, 1] - R[2, 2])
        return (s / 4, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if R[1, 1] > R[2, 2]:
        s = 2 * math.sqrt(1 + R[1, 1] - R[0, 0] - R[2, 2])
        return ((R[0, 1] + R[1, 0]) / s, s / 4, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = 2 * math.sqrt(1 + R[2, 2] - R[0, 0] - R[1, 1])
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4, (R[1, 0] - R[0, 1]) / s)


# -- reading the scene -----------------------------------------------------------------------------------------------
def carton_pose(objects):
    o = objects["carton"]
    return np.asarray(o.pos, float), rotm(o.quat_xyzw)


def tap_pose(objects):
    """The tap's frame in the world: its neck's middle and its rotation."""
    o = objects["carton/spigot"]
    return np.asarray(o.pos, float), rotm(o.quat_xyzw)


def tap_in_carton(objects):
    """The tap's neck middle and its rotation in the carton's frame."""
    cp, cR = carton_pose(objects)
    tp, tR = tap_pose(objects)
    return cR.T @ (tp - cp), cR.T @ tR


def carton_lean(objects) -> float:
    """How far the carton leans from upright (rad)."""
    _, cR = carton_pose(objects)
    return math.acos(max(-1.0, min(1.0, float(cR[2, 2]))))


def carton_spot(objects) -> np.ndarray:
    """The middle of the carton's footprint (its bottom face's middle) in the world."""
    cp, cR = carton_pose(objects)
    return cp + cR @ np.array([CD / 2, 0.0, 0.0])


def carton_low(objects) -> float:
    """The height of the carton's lowest bottom corner (world z): on the counter even when it rests on an edge."""
    cp, cR = carton_pose(objects)
    return min(float((cp + cR @ np.array([x, y, 0.0]))[2]) for x in (0.0, CD) for y in (-CW / 2, CW / 2))


def tab_torn() -> bool:
    """The tab's perforations torn in this trial (the tear law's memory, read only if it is this trial's)."""
    ep, mem = kit._LAW_MEM.get("carton/tab_x", (None, {}))
    return ep == kit._EPISODE["count"] and bool(mem.get("torn", False))


def seated(objects) -> bool:
    """The tap's neck down in the slot with the panel between its flanges, the tap level within 15 degrees, pointing
    out of the box within 30 degrees, its outlet down (its up within 30 degrees of the carton's)."""
    n, R = tap_in_carton(objects)
    axis_out = -R[:, 0]  # where the tap points
    in_slot = (abs(n[1]) <= NOTCH_Y and SEAT[2] - 0.002 <= n[2] <= SEAT[2] + SEAT_Z_TOL
               and abs(n[0] - SEAT[0]) <= SEAT_X_TOL)
    level = abs(math.asin(max(-1.0, min(1.0, axis_out[2])))) <= TILT_MAX
    out = axis_out[0] <= -math.cos(AIM_MAX)
    return bool(in_slot and level and out and R[2, 2] >= UPRIGHT)


def tap_out(objects) -> bool:
    """The tap is out through the opening: the middle of its body's back face (where the body meets the front flange)
    is in front of the panel's outside face, so the whole body is outside the box, however the tap is turned."""
    n, R = tap_in_carton(objects)
    return bool((n + R @ np.array([BODY[0][1], 0.0, 0.0]))[0] < 0.0)


def tap_boxes(objects):
    """The tap's collider boxes (body, flanges, neck, gland and outlet as their bounding boxes) in the world:
    (center, R, half-size)."""
    tp, tR = tap_pose(objects)
    out = []
    for c in _TAP_C:
        if c.size is not None:
            h = np.asarray(c.size, float) / 2
        else:
            h = np.array([c.height / 2, c.radius, c.radius]) if c.axis.upper() == "X" else \
                np.array([c.radius, c.radius, c.height / 2])
        out.append((tp + tR @ np.asarray(c.center, float), tR, h))
    return out


def _box_gap(q, c, R, h):
    d = np.abs((q - c) @ R) - h
    return np.linalg.norm(np.maximum(d, 0.0), axis=1) + np.minimum(d.max(axis=1), 0.0)


def hand_gap(objects) -> float:
    """Smallest gap between either arm's links from the upper arm down (link2, forearm, wrist, gripper, fingertips) and
    the tap; 0.05 when none is within 5 cm. In the simulator, no link poses read is an error, not a pass."""
    if not LINK_POSES:
        if "isaaclab" in sys.modules:
            raise RuntimeError("winebox: no arm link poses for the hands-clear check")
        return 0.05
    boxes = tap_boxes(objects)
    tp, _ = tap_pose(objects)

    def gap(q):
        q = np.atleast_2d(np.asarray(q, float))
        return np.min([_box_gap(q, c, R, h) for c, R, h in boxes], axis=0)
    out = 0.05
    for k, (o, R) in LINK_POSES.items():
        if k.split("/")[1] not in HAND_LINKS:
            continue
        o, R = np.asarray(o, float), np.asarray(R, float)
        _, cen, rad = yg.cloud(k.split("/")[1])
        if float(np.linalg.norm(o + R @ cen - tp)) - rad > 0.12:
            continue  # the whole link is far from the tap (no part of the tap is 7 cm from its neck)
        q = yg.near(k.split("/")[1], lambda x, o=o, R=R: o + x @ R.T, gap, 0.05)
        if len(q):
            out = min(out, float(gap(q).min()))
    return out


# -- the trial's ledger ----------------------------------------------------------------------------------------------
class Ledger:
    """One trial: the tab torn, the tap out and seated, the hands' gap, the hold."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.n = 0
        self.torn = False
        self.out = False
        self.seat = False
        self.clear = False
        self.held = False
        self.last = None
        self.good = 0
        self.c0 = None  # the carton's place at the trial's first step
        self.c_last = None
        self.tipped = False  # the carton was knocked over at some point in the trial
        self.slid = False  # the carton is now more than SLID from where it started
        self.standing = True  # the carton stands on the counter now (upright, not lifted)

    def step(self, objects) -> None:
        self.n += 1
        self.torn = tab_torn()
        gap = hand_gap(objects)
        self.held = gap <= TOUCH
        self.clear = gap > CLEAR
        self.seat = seated(objects)
        self.out = tap_out(objects) or self.seat
        cp = carton_spot(objects)
        if self.c0 is None:
            self.c0, self.z0 = cp.copy(), carton_low(objects)
        lean = carton_lean(objects)
        self.tipped = self.tipped or lean > KNOCKED_OVER
        self.slid = float(np.linalg.norm((cp - self.c0)[:2])) > SLID
        self.standing = lean <= STANDING and carton_low(objects) - self.z0 <= LIFTED
        tp, _ = tap_pose(objects)
        still = (self.last is not None and float(np.abs(tp - self.last).max()) <= STILL_STEP
                 and float(np.abs(cp - self.c_last).max()) <= STILL_STEP)
        self.last, self.c_last = tp, cp.copy()
        done = self.tier() == 1.0 and self.clear and still
        self.good = self.good + 1 if done else 0
        if _PROBE:
            n, R = tap_in_carton(objects)
            js = {k: kit._JOINTS[f"carton/{k}"].pos for k in ("tab_x", "tab_z", "tab", "sp_x", "sp_y", "sp_z",
                                                               "sp_pitch", "spigot") if f"carton/{k}" in kit._JOINTS}
            peak = kit.law_state("carton/tab_x").get("peak", 0.0)
            with open(_PROBE, "a") as fh:
                lz = objects["left_gripper"].pos[2] - TOP - CH if "left_gripper" in objects else float("nan")
                fh.write("%d torn %d peakF %.2f out %d seat %d gap %.4f lean %.2f moved %.4f leftz %.4f n %s ax %s"
                         " up %.3f j %s\n"
                         % (self.n, int(self.torn), peak, int(self.out), int(self.seat), gap,
                            math.degrees(carton_lean(objects)), float(np.linalg.norm((cp - self.c0)[:2])), lz,
                            " ".join("%.4f" % v for v in n),
                            " ".join("%.3f" % v for v in -R[:, 0]), R[2, 2],
                            " ".join("%s=%.4f" % kv for kv in js.items())))

    def tier(self) -> float:
        if self.tipped:
            return 0.0
        if self.seat and not self.held and not self.slid and self.standing:
            return 1.0
        if self.out or self.seat:
            return OUT_T
        if self.torn:
            return TAB_T
        return 0.0

    def score(self) -> float:
        s = self.tier()
        if s == 1.0 and self.good < HOLD_STEPS:
            return OUT_T
        return s


_L = None


def _read_links(base) -> None:
    """Every hand link's pose (env frame) at this control step, into LINK_POSES. A missing arm or link, or finger pads
    left without their rubber, stops the trial: the hand checks would otherwise pass silently."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"winebox: no {robot} in the scene for the hand checks") from e
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in HAND_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"winebox: {robot} has no link(s) {missing} for the hand checks")
        for link in HAND_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
    if gripkit._STATE.get("bound", 0) < 0:
        raise RuntimeError("winebox: the finger pads' rubber material could not be bound")
    if _PROBE and not _READ_ONCE:
        _READ_ONCE.append(1)
        print(f"winebox: hand checks on {len(LINK_POSES)} links; pad shapes with rubber: "
              f"{gripkit._STATE.get('bound')}", file=sys.stderr, flush=True)


_READ_ONCE: list = []
kit.STATE_HOOKS.append(_read_links)
if "isaaclab" in sys.modules:  # in the simulator: build the hand links' points as the task loads
    yg.warm(HAND_LINKS)


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
    """The tap seated in the slot, level and pointing out, outlet down, hands clear, all still: this step counts
    toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """1.0: the tap's neck down in the slot under the opening with the panel between its flanges, the tap level within
    15 degrees, pointing out of the box within 30 degrees, its outlet down (its up within 30 degrees of the carton's);
    let go (no hand within 5 mm of the tap); all still for 1 s. 0.5: the tap out through the opening (its body in front
    of the panel) but not seated so (hanging, upside down, held in the slot), or seated but not yet held 1 s. 0.25: the
    tab's perforations torn, the tap still inside. 0: anything else."""
    _update(objects)
    return _L.score()


# -- scripted demo: open loop from the fixed layout ------------------------------------------------------------------
# The right hand, fingers closed and pointing into the box, pushes the tab in at its middle until the perforations
# tear and the tab slides in, then draws back; the tab drops into the box. It opens its fingers 4 cm apart, reaches in
# through the opening either side of the tap's body, squeezes it, draws it straight out until the flanges straddle the
# panel, lowers it into the slot, lets go and draws back.
PAD_TIP = 0.013  # m: the pads' tips past the tool point (yam_arena PAD_TIP_PAST_TCP)
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open
POINT = -math.pi / 2  # pitch: fingers pointing into the box (+x), closing across it (y)
PUSH_PAST = 0.030  # m: the push aims the fingertips this far past the tab's outside face (short of the tap, 4.2 cm in)
GRIP_IN = 0.060  # m: the tool point this far inside the panel's face when gripping the tap's body
GRIP_OPEN = 0.021  # m: the finger faces this far either side of the tool point going in (the body is 32 mm wide;
# the fingers' outsides then clear the 96 mm opening by 2-3 mm a side)
GRIP_LOW = 0.002  # m: the tool point this far below the tap's axis (the fingers' back is 68 mm tall in a 76 mm opening)
SEAT_PRESS = 0.003  # m: the neck aimed this far below the slot's bottom (the arm gives)
# The left hand steadies the carton: fingers closed and pointing into the box, laid flat on its top (the hand's
# underside is 34 mm below the tool point) over the back half, then pressed STEADY_PRESS further down, so the arm's own
# give holds the carton down while the right hand works.
STEADY_X, STEADY_Y = 0.115, 0.030  # the left tool point over the top (carton frame): fingertips at the back edge
HAND_UNDER = 0.034  # m: the closed hand's underside below its tool point (fingers pointing level)
STEADY_PRESS = 0.025  # m: aimed this far below the top
DANGLE_OUT = 0.018  # m: the 'dangling' variant draws the tap this much further out before letting go
VARIANT = os.environ.get("WINEBOX_VARIANT", "")


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * min(FINGER_HALF_OPEN, max(0.0, half)) / FINGER_HALF_OPEN


def cw(x, y, z):
    """Carton frame -> world (the carton as it starts)."""
    return (X_C + x, Y_C + y, TOP + z)


def _tw(duration, pos, finger, label, yaw=0.0, pitch=POINT):
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label, yaw=float(yaw),
                      pitch=float(pitch))


def _t(seq):
    return sum(w.duration for w in seq)


def demo():
    """Left: lay the hand flat on the carton's top and press down (steady it) until the right hand is done. Right: tear
    the tab in, draw the tap out, seat it, let go. WINEBOX_VARIANT=nothing: both arms stay at the ready pose; dangling:
    the tap is drawn 1.8 cm further out than seated and let go there (it hangs outside); upside: the hand turns the tap
    over (half a turn about its axis) before seating it; held: the tap is seated but never let go; nosteady: the left
    hand stays at the ready pose (nothing steadies the carton)."""
    over = cw(STEADY_X, STEADY_Y, CH + HAND_UNDER + 0.05)
    press = cw(STEADY_X, STEADY_Y, CH + HAND_UNDER - STEADY_PRESS)
    closed = finger_for(0.0)
    left = [kit.Home(0.5, label="start"),
            _tw(2.5, over, closed, "over the carton's top"),
            _tw(1.5, press, closed, "hand flat on the top, press down (steady the carton)")]
    if VARIANT == "nosteady":
        left = [kit.Home(4.5, label="stay at the ready pose")]
    if VARIANT == "nothing":
        return {"left": [kit.Home(10.0, label="stay at the ready pose")],
                "right": [kit.Home(10.0, label="stay at the ready pose")]}
    zc = SPIGOT_START[2]
    zg = zc - GRIP_LOW  # the tool point gripping the tap
    grip_x = GRIP_IN
    pull_x = grip_x - (SPIGOT_START[0] - SEAT[0])  # the tool point when the groove is in the panel's plane
    right = [kit.Home(0.5, label="start"),
             _tw(3.0, cw(-0.12, 0.0, zc), closed, "in front of the tab"),
             _tw(2.0, cw(-PAD_TIP - 0.015, 0.0, zc), closed, "fingertips just off the tab's middle"),
             _tw(3.5, cw(-PAD_TIP + PUSH_PAST, 0.0, zc), closed, "push the tab in until it tears"),
             _tw(0.5, cw(-PAD_TIP + PUSH_PAST, 0.0, zc), closed, "settle"),
             _tw(2.0, cw(-0.06, 0.0, zc), closed, "draw back out of the opening (the tab drops)"),
             _tw(1.0, cw(-0.06, 0.0, zc), finger_for(GRIP_OPEN), "open the fingers"),
             _tw(2.5, cw(grip_x, 0.0, zg), finger_for(GRIP_OPEN), "reach in either side of the tap's body"),
             _tw(0.5, cw(grip_x, 0.0, zg), finger_for(GRIP_OPEN), "settle"),
             _tw(1.5, cw(grip_x, 0.0, zg), closed, "squeeze the tap"),
             _tw(0.5, cw(grip_x, 0.0, zg), closed, "settle")]
    yaw = math.pi if VARIANT == "upside" else 0.0
    if VARIANT == "dangling":
        right += [_tw(4.0, cw(pull_x - DANGLE_OUT, 0.0, zg + 0.004), closed, "draw the tap out past the slot"),
                  _tw(0.5, cw(pull_x - DANGLE_OUT, 0.0, zg + 0.004), closed, "settle"),
                  _tw(1.0, cw(pull_x - DANGLE_OUT, 0.0, zg + 0.004), finger_for(0.024), "let go (it hangs)"),
                  _tw(2.0, cw(pull_x - 0.06, 0.0, zg + 0.004), finger_for(0.024), "draw back")]
    else:
        right += [_tw(4.0, cw(pull_x, 0.0, zg + 0.004), closed,
                      "draw the tap out until its flanges straddle the panel")]
        if VARIANT == "upside":
            right += [_tw(3.0, cw(pull_x - 0.005, 0.0, zg + 0.004), closed, "turn it over", yaw=yaw)]
            right += [_tw(1.0, cw(pull_x, 0.0, zg + 0.004), closed, "back in line", yaw=yaw)]
        right += [_tw(0.5, cw(pull_x, 0.0, zg + 0.004), closed, "settle", yaw=yaw),
                  _tw(3.0, cw(pull_x, 0.0, SEAT[2] - GRIP_LOW - SEAT_PRESS), closed, "lower its neck into the slot",
                      yaw=yaw),
                  _tw(0.5, cw(pull_x, 0.0, SEAT[2] - GRIP_LOW - SEAT_PRESS), closed, "settle", yaw=yaw)]
        if VARIANT == "held":
            right += [_tw(8.0, cw(pull_x, 0.0, SEAT[2] - GRIP_LOW - SEAT_PRESS), closed, "keep holding it", yaw=yaw)]
            return {"left": _left_rest(left, _t(right) - 5.5), "right": right}
        right += [_tw(1.0, cw(pull_x, 0.0, SEAT[2] - GRIP_LOW - SEAT_PRESS), finger_for(0.024), "let go", yaw=yaw),
                  _tw(2.0, cw(pull_x - 0.045, 0.0, SEAT[2] + 0.013), finger_for(0.024), "draw back", yaw=yaw)]
    right += [kit.Home(3.0, label="back to the ready pose"),
              kit.Home(2.5, label="wait")]
    return {"left": _left_rest(left, _t(right) - 5.5), "right": right}


def _left_rest(left, until):
    """The left hand keeps pressing until `until` s, then lifts off the top and goes back to the ready pose (or, if it
    never went to the carton, stays at the ready pose)."""
    if VARIANT == "nosteady":
        return left + [kit.Home(until + 5.5 - _t(left), label="stay")]
    press, over = left[-1].pos, left[-2].pos
    return left + [_tw(until - _t(left), press, left[-1].finger, "keep pressing"),
                   _tw(1.5, over, left[-1].finger, "lift off the top"),
                   kit.Home(3.0, label="back to the ready pose"),
                   kit.Home(1.0, label="wait")]
