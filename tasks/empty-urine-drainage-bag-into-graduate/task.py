"""At a bedside (the station table stands in for the floor beside the bed): a Bard 154002 2000 mL urinary drainage bag
hangs by its hook from a catheter-bag floor holder, its front toward the robot, holding 600 mL of clear tap water
standing in for urine (in the sim and the real trial). Its drain tube is folded up in front of the bag with the
twist-valve tip pushed into its holder clip, the valve closed. A blue absorbent underpad lies on the floor under the
bag; a clean, empty 1000 mL graduated intake/output container (the graduate) stands beside it on the robot's right.

The job is a nursing assistant's routine, "Measure and record food and liquid intake or urinary and fecal output"
(O*NET 31-1131.00), done the way the CDC's catheter guideline requires (III.B.3): empty the bag into a separate clean
container, avoid splashing, and never let the drain spout touch the container. As nurses do it: set the graduate on the
underpad under the drain; one hand takes the drain tube out of its clip and aims the spout into the graduate's mouth
without touching it; the other hand opens the valve; the aiming hand follows the stream as its arc shortens while the
bag empties; the valve is closed and the tube goes back into its clip. The real failures are kept possible: the spout
touches the rim, the stream misses the mouth (an angled spout throws it forward, and the throw changes as the bag
empties), the tube is dropped and hangs down draining onto the floor, the valve is left open, the graduate is knocked
over, the tube is left out of its holder.

The bag and tube are a yamkit articulation on the holder: the bag swings on its hook (a pull on the tube moves it), the
drain tube is two links (an 8 cm run off the outlet, then 11 cm) on hinges with a sideways hinge at the outlet, and the
tip is a 22 mm twist-valve collar that turns on the tube. The holder clip and the valve's stops are click laws on those
joints. The water is a lumped flow model, the way the IV-hub task moves its fluid: the bag is a pillow-shaped volume
(its section grows over the bottom 8 cm, so 600 mL stands 10 cm deep); it drains through the spout as an orifice
(Torricelli, Cd 0.61) with the head from the bag's level down to the spout, scaled by how far the valve is open; the
stream leaves the spout along its axis at sqrt(2 g h) and falls ballistically into the graduate's mouth or onto the
floor (counted as spilled); the graduate is a liquidkit vessel (its level, and a spill over its rim if tipped). Every
threshold below appears once, with the same value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import guardkit as gk  # noqa: E402
import liquidkit as lk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = ("Empty the drainage bag into the graduate without letting the drain spout touch the graduate, then "
               "close the drain and put the tube back in its holder.")
ML = 1e-6  # m^3 per mL
G = 9.81

# -- looks --------------------------------------------------------------------------------------------------------------
VINYL = kit.Look((0.90, 0.92, 0.93), 0.3)  # the bag's clear vinyl (drawn as its back panel and seams)
VINYL_EDGE = kit.Look((0.78, 0.80, 0.82), 0.4)
PRINT = kit.Look((0.35, 0.45, 0.70), 0.5)  # the bag's printed graduation panel
TUBE = kit.Look((0.86, 0.90, 0.92), 0.25)  # clear PVC drain tube
VALVE = kit.Look((0.20, 0.45, 0.80), 0.4)  # the blue twist-valve collar
STEEL = kit.Look((0.75, 0.76, 0.78), 0.3, metallic=0.8)
PP = kit.Look((0.93, 0.93, 0.90), 0.35)  # the graduate's translucent polypropylene (opaque in the renderer)
WATER_RGB = (0.66, 0.83, 0.96)  # clear tap water: light, slightly blue-tinted
WATER_OPACITY = 0.55  # drawn semi-transparent (RTX fractional cutout opacity)
JET_OPACITY = 1.0  # the thin jet drawn solid (a few pixels wide: partial opacity would only speckle it)
PAD_RGB = (0.55, 0.70, 0.88)  # the blue absorbent underpad on the floor

# -- the holder and the bag ---------------------------------------------------------------------------------------------
# Catheter bag floor holder for 2 L bags: 500 mm tall, 210 mm deep, 0.3 kg (Rosscare / Essential Aids listings). Its
# base sits on the floor (the table) behind the bag; the bag's hanger hangs on the hook at the top, 480 mm up.
STAND_X = 0.450
BAG_Y = -0.040
STAND_POS = (STAND_X, BAG_Y, TOP)  # the holder's base, middle, on the table: the articulation's fixed base frame
HOOK = (-0.030, 0.0, 0.480)  # the hook, in the base frame
# Bard 154002 urinary drainage bag, 2000 mL, vinyl, single hook (BD product page; retailer listings: twist valve
# drain tube). Inside modeled as a box 18 cm wide, 4.5 cm thick, 25 cm tall (2.03 L), hanging 2 cm below the hook on
# its hanger. [V: measure the real bag; the maker lists capacity, not dimensions.]
STRAP = 0.020
BAG_W, BAG_T, BAG_H = 0.180, 0.045, 0.250
BAG_AREA = BAG_W * BAG_T  # m^2: the full section (81 cm^2) above the bottom's bulge
# A filled vinyl bag is a pillow: welded flat along its bottom seam, it bulges to its full thickness a few cm up. The
# liquid's section grows as sqrt(height) over the bottom BAG_BULGE_H, then stays BAG_AREA: 600 mL stands 10 cm deep.
BAG_BULGE_H = 0.080  # m [V]
_BULGE_V = BAG_AREA * 2.0 / 3.0 * BAG_BULGE_H  # m^3 held in the bulge (432 mL)


def bag_depth(volume_m3: float) -> float:
    """Height of the urine's surface above the bag's bottom seam for this volume."""
    v = max(volume_m3, 0.0)
    if v <= _BULGE_V:
        return BAG_BULGE_H * (v / _BULGE_V) ** (2.0 / 3.0)
    return BAG_BULGE_H + (v - _BULGE_V) / BAG_AREA


def bag_thickness(z: float) -> float:
    """The filled bag's inside thickness at height z above its bottom seam."""
    return BAG_T * math.sqrt(min(max(z, 0.0), BAG_BULGE_H) / BAG_BULGE_H)


BAG_SHELL_KG = 0.070  # the empty bag and its 48 in inlet tubing [V]
START_ML = 600.0  # an adult's output over about 8 h (0.5-1 mL/kg/h for 70 kg): a bag due to be emptied
BAG_KG = BAG_SHELL_KG + START_ML * ML * 1000.0  # the bag's body mass (the liquid's weight is not changed as it drains)
BAG_SWING = (-0.02, 0.40)  # rad: the holder's frame stops it swinging back; it can swing out toward the robot
PORT = (HOOK[0], 0.0, HOOK[2] - STRAP - BAG_H)  # the outlet, at the middle of the bag's bottom (base frame)

# -- the drain tube and twist valve --------------------------------------------------------------------------------------
# Clear PVC drain tube, 9 mm across, ending in a 12 mm twist-valve collar 22 mm long (the spout) [V: measure].
TUBE_R = 0.0045
LA = 0.080  # the tube's run out from the outlet before it folds up: the folded tube stands 5.75 cm proud of the bag's
# front, held there by the holder tab (room for the gripper's 10 cm palm beside the bag; probes p3-p5) [V: the real tube
# folds closer to the bag]
LB = 0.110  # the rest of the tube [V]
LV = 0.022  # the twist-valve collar; its end is the spout
COLLAR_R = 0.006
TUBE_KG = (0.004, 0.008, 0.010, 0.010)  # sideways hinge, stub, tube, collar [raised from ~2 g for the solver]
STOW_A = math.pi / 2  # the stub folded out toward the robot, horizontal ...
STOW_B = math.pi / 2  # ... and the tube folded up in front of the bag: the spout in the holder clip, pointing up
CLIP_ZONE = 0.15  # rad: within this of both stowed angles the tube is in its holder (the clip pulls it in)
CLIP_PULL = (0.30, 0.10)  # N*m/rad on the stub and the tube: about 0.5 N at the spout to pull it out [estimate]
# The clip's laws act only from the first observation, after the embodiment's 0.5 s settle (no law runs then), so the
# folded tube would droop out of the clip zone under gravity first (probe p1). The clip's hinges carry this damping from
# spawn and keep it while clipped: the stub drifts < 0.02 rad in the settle; it only resists speed, not the pull-out.
CLIP_SETTLE_DAMP = 0.30  # N*m*s/rad
VALVE_OPEN = math.pi / 2  # rad: the collar's travel, closed stop to open stop [V: measure the real valve's turn]
VALVE_OPEN_Q = -VALVE_OPEN  # the collar's joint at its open stop: it opens turning clockwise seen from the spout (the
# direction the right wrist can turn it at the draining pose) [V: the real valve's direction]
VALVE_CLOSED_DEG = 10.0  # the valve counts as closed within 10 degrees of its closed stop (no flow there)
VALVE_PULL = 0.02  # N*m/rad toward the nearer stop when within 0.15 rad of it (the valve clicks at its stops)
OUTLET_R = 0.003  # m: the spout's 6 mm bore [V]
OUTLET_A = math.pi * OUTLET_R ** 2
CD = lk.ORIFICE_CD  # 0.61, a sharp-edged orifice


class Stops:
    """A joint with click positions (a tube clip, a valve's stops): within `zone` of one it is pulled into it with
    `pull`; between them it is free with a little drag. `setting` is the index of the stop it sits in (None between)."""

    def __init__(self, positions, zone: float, pull: float, damp: float = 0.02, drag: float = 0.01):
        self.positions, self.zone, self.pull, self.damp, self.drag = tuple(positions), zone, pull, damp, drag

    def drive(self, key, q, qd, J, mem):
        i = min(range(len(self.positions)), key=lambda k: abs(q - self.positions[k]))
        if abs(q - self.positions[i]) <= self.zone:
            mem["setting"] = i
            return self.positions[i], self.pull, 0.0, self.damp
        mem["setting"] = None
        return q, 0.0, 0.0, self.drag


def _below(p, d) -> tuple:
    return (p[0], p[1], p[2] - d)


def _cyl(center, r, h, look) -> kit.Mesh:
    return kit.cylinder(center, r, h, look)


_HOOK_TO_BAG = STRAP + BAG_H / 2
_BAG_C = (0.0, 0.0, -_HOOK_TO_BAG)  # the bag body's middle, in the bag link's frame (origin at the hook)
_PORT_L = (0.0, 0.0, -(STRAP + BAG_H))  # the outlet in the bag link's frame
_CLIP_Z = LB + LV - 0.006  # the holder clip's middle above the outlet, on the folded tube
BAG_LINK = kit.Link(
    "bag", "revolute", "Y", anchor=HOOK, lower=BAG_SWING[0], upper=BAG_SWING[1], start=0.0, mass=BAG_KG, damping=0.3,
    meshes=(kit.box((BAG_T / 2 - 0.0005, 0.0, _BAG_C[2]), (0.001, BAG_W, BAG_H), VINYL),  # back panel
            kit.box((-BAG_T / 2 + 0.0004, 0.0, _BAG_C[2] + 0.06), (0.0008, 0.10, 0.08), PRINT),  # front label
            kit.box((0.0, BAG_W / 2, _BAG_C[2]), (BAG_T, 0.004, BAG_H), VINYL_EDGE),  # side seams
            kit.box((0.0, -BAG_W / 2, _BAG_C[2]), (BAG_T, 0.004, BAG_H), VINYL_EDGE),
            kit.box((0.0, 0.0, -STRAP - 0.002), (BAG_T, BAG_W, 0.004), VINYL_EDGE),  # top seam
            kit.box((0.0, 0.0, -STRAP - BAG_H + 0.002), (BAG_T, BAG_W, 0.004), VINYL_EDGE),  # bottom seam
            kit.box((0.0, 0.0, -STRAP / 2), (0.003, 0.012, STRAP), VINYL_EDGE),  # the hanger
            _cyl(_below(_PORT_L, -0.004), 0.006, 0.008, TUBE),  # the outlet boss
            # the tube holder: a tab out from the bag's front ending in the clip the folded tube's spout end sits in
            kit.box((-(BAG_T / 2 + LA - COLLAR_R) / 2, 0.0, _PORT_L[2] + _CLIP_Z), (LA - COLLAR_R - BAG_T / 2, 0.010, 0.003),
                    VINYL_EDGE)),
    colliders=(kit.Collider(_BAG_C, size=(BAG_T, BAG_W, BAG_H)),))
# The tube's own bending stiffness keeps it from flopping sideways at the outlet: without it the folded-up tube is an
# inverted pendulum on this hinge and fell over to its stop (probe p3: the collar 48 mm off its clip).
TUBE_SIDE_PULL = 0.05  # N*m/rad toward straight (gravity's tipping stiffness folded up is ~0.015) [estimate]
TUBE_X = kit.Link("tube_x", "revolute", "X", anchor=PORT, lower=-0.5, upper=0.5, start=0.0, mass=TUBE_KG[0],
                  parent="bag", damping=CLIP_SETTLE_DAMP,
                  law=Stops((0.0,), 0.6, TUBE_SIDE_PULL, damp=CLIP_SETTLE_DAMP))
TUBE_A = kit.Link("tube_a", "revolute", "Y", anchor=PORT, lower=-0.6, upper=2.6, start=STOW_A, mass=TUBE_KG[1],
                  parent="tube_x", damping=CLIP_SETTLE_DAMP,
                  meshes=(_cyl((0.0, 0.0, -LA / 2), TUBE_R, LA, TUBE),),
                  colliders=(kit.Collider((0.0, 0.0, -LA / 2), radius=TUBE_R, height=LA - 0.004),),
                  law=Stops((STOW_A,), CLIP_ZONE, CLIP_PULL[0], damp=CLIP_SETTLE_DAMP))
TUBE_B = kit.Link("tube_b", "revolute", "Y", anchor=_below(PORT, LA), lower=-2.6, upper=2.6, start=STOW_B,
                  mass=TUBE_KG[2], parent="tube_a", damping=CLIP_SETTLE_DAMP,
                  meshes=(_cyl((0.0, 0.0, -LB / 2), TUBE_R, LB, TUBE),),
                  colliders=(kit.Collider((0.0, 0.0, -LB / 2), radius=TUBE_R, height=LB - 0.004),),
                  law=Stops((STOW_B,), CLIP_ZONE, CLIP_PULL[1], damp=CLIP_SETTLE_DAMP))
VALVE_LINK = kit.Link("valve", "revolute", "Z", anchor=_below(PORT, LA + LB), lower=min(0.0, VALVE_OPEN_Q),
                      upper=max(0.0, VALVE_OPEN_Q), start=0.0,
                      mass=TUBE_KG[3], parent="tube_b", damping=0.0,
                      meshes=(_cyl((0.0, 0.0, -LV / 2), COLLAR_R, LV, VALVE),
                              kit.box((COLLAR_R, 0.0, -LV / 2), (0.002, 0.003, LV * 0.8), TUBE)),  # its grip rib
                      colliders=(kit.Collider((0.0, 0.0, -LV / 2), radius=COLLAR_R, height=LV),),
                      law=Stops((0.0, VALVE_OPEN_Q), 0.15, VALVE_PULL))
_STAND_MESHES = (kit.box((0.0, 0.0, 0.004), (0.210, 0.220, 0.008), STEEL),  # the base frame on the floor
                 _cyl((0.020, 0.090, 0.245), 0.005, 0.490, STEEL), _cyl((0.020, -0.090, 0.245), 0.005, 0.490, STEEL),
                 kit.box((0.020, 0.0, 0.488), (0.010, 0.190, 0.010), STEEL),  # the top bar
                 kit.box(((HOOK[0] + 0.020) / 2, 0.0, 0.486), (0.050, 0.006, 0.006), STEEL))  # the hook arm
BAG = kit.Mechanism(
    "drainbag", pos=STAND_POS, size=(0.21, 0.22, 0.50), meshes=_STAND_MESHES,
    colliders=(kit.Collider((0.0, 0.0, 0.004), size=(0.210, 0.220, 0.008)),
               kit.Collider((0.020, 0.0, 0.245), size=(0.010, 0.190, 0.490))),
    links=(BAG_LINK, TUBE_X, TUBE_A, TUBE_B, VALVE_LINK))

# -- the graduate --------------------------------------------------------------------------------------------------------
# Medline 1000 mL (32 oz) graduated triangular intake/output container, polypropylene (DYND80417 translucent). Modeled
# round: a frustum 9.0 cm across the floor and 11.2 cm across the mouth inside, 13.5 cm deep (1.06 L to the brim), on
# a 3 mm floor, with its molded handle on the robot's right. [V: measure the real container; a round mouth of the
# same area stands in for the triangular one.]
GR_B, GR_T, GR_H = 0.045, 0.056, 0.135
GR_FLOOR, GR_WALL = 0.003, 0.002
GR_KG = 0.090  # [V]
GR_RIM_Z = GR_FLOOR + GR_H  # the rim's height above the table
GR_START = (0.230, -0.245)  # on the robot's right beside the underpad, its handle toward the right
GR_SIDE = -1.0  # the handle sticks out toward -y
HANDLE_ARM = (0.012, 0.030, 0.008)  # the handle's top arm (x across it, y out from the rim, z)
HANDLE_ARM_C = (0.0, GR_SIDE * (GR_T + HANDLE_ARM[1] / 2 - 0.002), GR_RIM_Z - 0.010)
HANDLE_BAR = (0.012, 0.008, 0.090)
HANDLE_BAR_C = (0.0, GR_SIDE * (GR_T + HANDLE_ARM[1] - 0.006), GR_RIM_Z - 0.010 - HANDLE_BAR[2] / 2)
HANDLE_GRIP = (0.0, GR_SIDE * (GR_T + 0.016), HANDLE_ARM_C[2])  # where the hand takes it: the top arm's middle


def _ring_posts(r_lo, r_hi, z_lo, z_hi, post=0.004):
    """A frustum wall as a ring of touching upright posts just outside the inside radius (colliders are convex)."""
    rc = (r_lo + r_hi) / 2 + post
    n = int(math.ceil(2 * math.pi * rc / (2 * post)))
    return [kit.Collider((rc * math.cos(2 * math.pi * k / n), rc * math.sin(2 * math.pi * k / n), (z_lo + z_hi) / 2),
                         radius=post, height=z_hi - z_lo) for k in range(n)]


GRADUATE = kit.Prop(
    "graduate", pos=(GR_START[0], GR_START[1], TOP), mass=GR_KG,
    size=(2 * (GR_T + 0.008), 2 * (GR_T + HANDLE_ARM[1]), GR_RIM_Z),
    meshes=(kit.lathe([(0.0, 0.0), (GR_B + GR_WALL, 0.0), (GR_T + GR_WALL, GR_RIM_Z), (GR_T, GR_RIM_Z),
                       (GR_B, GR_FLOOR), (0.0, GR_FLOOR)], PP, n=48),
            kit.box(HANDLE_ARM_C, HANDLE_ARM, PP), kit.box(HANDLE_BAR_C, HANDLE_BAR, PP)),
    colliders=(kit.Collider((0.0, 0.0, GR_FLOOR / 2), radius=GR_B + GR_WALL, height=GR_FLOOR),
               kit.Collider(HANDLE_ARM_C, size=HANDLE_ARM), kit.Collider(HANDLE_BAR_C, size=HANDLE_BAR))
    + tuple(_ring_posts(GR_B, GR_T, GR_FLOOR, GR_RIM_Z)))

UNDERPAD = Box("underpad", pos=(0.300, BAG_Y + 0.060, TOP + 0.0006), size=(0.30, 0.40, 0.001), color=PAD_RGB)  # 1 mm, on the floor
OBJECTS = [UNDERPAD, BAG, GRADUATE]

# -- the scored thresholds -----------------------------------------------------------------------------------------------
FULL_ML = 570.0  # in the graduate for full credit: 95 % of the 600 mL (the bag drains completely when open and aimed)
HALF_ML = 300.0  # half the bag
TRACE_ML = 100.0
CLEAN_ML = 5.0  # mL spilled for full credit: a few splashes on the pad
SPILL_MAX_ML = 20.0  # mL: more spilled scores 0 (a puddle on the floor)
TOUCH_TOL = 0.001  # m: the spout (collar and the tube's last 2 cm) within 1 mm of the graduate is a touch
SPOUT_TUBE = 0.020  # m of tube above the collar counted as the spout
MOUTH_MARGIN = 0.002  # m: a stream landing this close to the rim is taken to hit it and splash out
UPRIGHT_DEG = 10.0  # the graduate's axis within 10 degrees of vertical at the end
RELEASE_MARGIN = 0.010  # m: every tool point and pad tip this far from the spout, the tube and the graduate
STILL_STEP = 0.0005  # m per control step (1 cm/s)
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
TOUCH_CAP = 0.25  # a touch caps the trial at 0.25
GRIPPERS = ("left_gripper", "right_gripper")


# -- geometry ------------------------------------------------------------------------------------------------------------
def rot_axis(axis: str, a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    if axis == "X":
        return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])
    if axis == "Y":
        return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def axis_rot(u, a: float) -> np.ndarray:
    """Rotation by a about unit axis u (Rodrigues)."""
    u = np.asarray(u, float) / np.linalg.norm(u)
    k = np.array([[0, -u[2], u[1]], [u[2], 0, -u[0]], [-u[1], u[0], 0]])
    return np.eye(3) + math.sin(a) * k + (1 - math.cos(a)) * (k @ k)


def fk(swing=0.0, side=0.0, a=STOW_A, b=STOW_B, v=0.0) -> dict:
    """World pose (origin, rotation) of every link's frame for these joint angles: the bag on its hook, the sideways
    hinge, the stub, the tube and the valve collar; plus 'tip' (the spout's end, a point) and 'collar' (its middle)."""
    base = np.asarray(STAND_POS, float)
    hook = base + np.asarray(HOOK, float)
    Rb = rot_axis("Y", swing)
    port = hook + Rb @ (np.asarray(PORT, float) - np.asarray(HOOK, float))
    Rx = Rb @ rot_axis("X", side)
    Ra = Rx @ rot_axis("Y", a)
    pb = port + Ra @ np.array([0.0, 0.0, -LA])
    Rbt = Ra @ rot_axis("Y", b)
    pv = pb + Rbt @ np.array([0.0, 0.0, -LB])
    Rv = Rbt @ rot_axis("Z", v)
    return {"bag": (hook, Rb), "tube_x": (port, Rx), "tube_a": (port, Ra), "tube_b": (pb, Rbt), "valve": (pv, Rv),
            "tip": pv + Rv @ np.array([0.0, 0.0, -LV]), "collar": pv + Rv @ np.array([0.0, 0.0, -LV / 2])}


def _R(q) -> np.ndarray:
    return lk._rot(q)


def _state_pose(objects, key) -> tuple:
    s = objects[key]
    return np.asarray(s.pos, float), _R(np.asarray(s.quat_xyzw, float))


def spout(objects) -> tuple:
    """(spout end, its outward axis, the valve frame's origin, rotation) from the valve collar's state."""
    p, R = _state_pose(objects, "drainbag/valve")
    d = R @ np.array([0.0, 0.0, -1.0])
    return p + LV * d, d, p, R


def bag_level(objects, volume_m3: float) -> float:
    """World height of the urine's surface in the bag: the bottom's middle plus the lumped depth (the bag hangs within
    a few degrees of vertical)."""
    p, R = _state_pose(objects, "drainbag/bag")
    floor = p + R @ np.asarray(_PORT_L, float)
    return float(floor[2]) + bag_depth(volume_m3)


def joint_angle(key: str, default: float) -> float:
    try:
        return float(kit.joint(None, f"drainbag/{key}").pos)
    except KeyError:
        return default


def valve_open_fraction(v: float) -> float:
    """How far the valve is open: 0 within VALVE_CLOSED_DEG of its closed stop, 1 at its open stop."""
    lo = math.radians(VALVE_CLOSED_DEG)
    return float(min(max((v / VALVE_OPEN_Q * VALVE_OPEN - lo) / (VALVE_OPEN - lo), 0.0), 1.0))


def stowed(a: float, b: float) -> bool:
    """The tube is folded up in its holder clip: both of its hinges within CLIP_ZONE of their stowed angles."""
    return abs(a - STOW_A) <= CLIP_ZONE and abs(b - STOW_B) <= CLIP_ZONE


def graduate_frame(objects) -> tuple:
    """(origin at the inside floor's middle, rotation) of the graduate's liquidkit frame."""
    p, R = _state_pose(objects, "graduate")
    return p + R @ np.array([0.0, 0.0, GR_FLOOR]), R


def wall_distance(local) -> float:
    """Distance from a point (graduate frame, origin at the inside floor's middle) to the graduate's wall, rim or floor,
    in the (radius, height) half-plane of its axis."""
    rho, z = math.hypot(float(local[0]), float(local[1])), float(local[2])
    best = math.inf
    ro_b, ro_t = GR_B + GR_WALL / 2, GR_T + GR_WALL / 2
    for (r0, z0), (r1, z1), half in (((ro_b, 0.0), (ro_t, GR_H), GR_WALL / 2),
                                     ((0.0, -GR_FLOOR / 2), (GR_B, -GR_FLOOR / 2), GR_FLOOR / 2)):
        dr, dz = r1 - r0, z1 - z0
        t = min(max(((rho - r0) * dr + (z - z0) * dz) / (dr * dr + dz * dz), 0.0), 1.0)
        best = min(best, math.hypot(rho - r0 - t * dr, z - z0 - t * dz) - half)
    return best


def spout_points(objects) -> list:
    """(point, radius) samples along the spout: the collar and the last 2 cm of tube above it."""
    _tip, d, pv, _R_ = spout(objects)
    pts = [(pv + d * (LV * k / 4), COLLAR_R) for k in range(5)]
    pts += [(pv - d * (SPOUT_TUBE * k / 4), TUBE_R) for k in range(1, 5)]
    return pts


def touches(objects) -> bool:
    """The spout (collar or the tube's last 2 cm) within TOUCH_TOL of the graduate's wall, rim or floor."""
    o, R = graduate_frame(objects)
    return any(wall_distance(R.T @ (p - o)) <= r + TOUCH_TOL for p, r in spout_points(objects))


def graduate_depth(volume_m3: float) -> float:
    """Depth of this much water in the upright graduate (the frustum's exact volume solved for the height), for the
    drawing: the drawn level rises smoothly, not in the liquidkit sample layers' 5.6 mm steps."""
    def vol(h):
        r = GR_B + (GR_T - GR_B) * h / GR_H
        return math.pi * h / 3.0 * (GR_B * GR_B + GR_B * r + r * r)
    v = max(float(volume_m3), 0.0)
    if v >= vol(GR_H):
        return GR_H
    lo, hi = 0.0, GR_H
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if vol(mid) < v else (lo, mid)
    return (lo + hi) / 2


def tilt_deg(R) -> float:
    return math.degrees(math.acos(float(np.clip(R[2, 2], -1.0, 1.0))))


def arc_points(p0, v0, z_end: float, n: int = 14) -> list:
    """Points along the ballistic stream from p0 (velocity v0) down to height z_end."""
    p0, v0 = np.asarray(p0, float), np.asarray(v0, float)
    a, b, c = -G / 2, float(v0[2]), float(p0[2]) - z_end
    disc = b * b - 4 * a * c
    t_end = (-b - math.sqrt(disc)) / (2 * a) if disc >= 0 else 0.0
    ts = np.linspace(0.0, max(t_end, 0.0), n + 1)
    return [p0 + v0 * t + np.array([0.0, 0.0, -G / 2 * t * t]) for t in ts]


def land(p0, v0, z: float):
    """Where a stream leaving p0 with velocity v0 crosses height z on its way down (None if it never does)."""
    a, b, c = -G / 2, float(v0[2]), float(p0[2]) - z
    disc = b * b - 4 * a * c
    if disc < 0:
        return None
    t = (-b - math.sqrt(disc)) / (2 * a)
    if t < 0:
        return None
    return np.asarray(p0, float)[:2] + np.asarray(v0, float)[:2] * t


# -- one trial's record --------------------------------------------------------------------------------------------------
class Ledger:
    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.bag = START_ML * ML  # m^3 in the bag
        self.world = lk.World(vessels={"graduate": lk.Vessel("graduate", lk.bowl(GR_B, GR_T, GR_H), lip_width=0.03)},
                              table_z=TOP)
        self.stream_spill = 0.0  # m^3 that missed the graduate
        self.stream = None  # the stream's arc (points from the spout to where it lands) while it flows, for the drawing
        self.stream_frac = 0.0  # how far the valve was open for that stream (the drawn stream's thickness)
        self.stream_caught = False  # whether that stream went into the graduate's mouth
        self.touched = False
        self.good = 0
        self.last = None
        self.a, self.b, self.v = STOW_A, STOW_B, 0.0

    def graduate_ml(self) -> float:
        return self.world.vessels["graduate"].volume / ML

    def spilled_ml(self) -> float:
        return (self.stream_spill + self.world.spilled) / ML

    def flow(self, objects, dt: float) -> None:
        """The drain for one control step: orifice flow from the bag's level down to the spout, then the stream's
        landing (in the graduate's mouth, else on the floor), then the graduate's own spill if it is tipped."""
        self.stream = None
        frac = valve_open_fraction(self.v)
        tip, d, _pv, _Rv = spout(objects)
        head = bag_level(objects, self.bag) - float(tip[2])
        o, R = graduate_frame(objects)
        if frac > 0.0 and head > 0.0 and self.bag > 0.0:
            speed = math.sqrt(2 * G * head)
            q = min(self.bag, CD * OUTLET_A * frac * speed * dt)
            self.bag -= q
            mouth = o + R @ np.array([0.0, 0.0, GR_H])
            caught = False
            if tilt_deg(R) <= 30.0:
                if float(tip[2]) <= float(mouth[2]):  # the spout is below the rim: inside the mouth or outside it
                    loc = R.T @ (tip - o)
                    caught = 0.0 <= float(loc[2]) <= GR_H and math.hypot(loc[0], loc[1]) <= GR_T - MOUTH_MARGIN
                    end = tip
                else:
                    hit = land(tip, speed * d, float(mouth[2]))
                    caught = hit is not None and float(np.linalg.norm(hit - mouth[:2])) <= GR_T - MOUTH_MARGIN
                    end = np.array([*hit, float(mouth[2])]) if hit is not None else tip
            if caught:
                g = self.world.vessels["graduate"]
                room = max(g.capacity - g.volume, 0.0)
                g.volume += min(q, room)
                self.stream_spill += max(q - room, 0.0)
            else:
                self.stream_spill += q
                hit = land(tip, speed * d, TOP)
                end = np.array([*hit, TOP]) if hit is not None else tip
            # the drawing: the stream's arc from the spout down to the graduate's surface, or to the floor
            if caught:
                z_end = float((o + R @ np.array([0.0, 0.0, graduate_depth(self.world.vessels["graduate"].volume)]))[2])
            else:
                z_end = TOP
            self.stream = arc_points(tip, speed * d, min(z_end, float(tip[2])), n=STREAM_SEGMENTS)
            self.stream_frac = frac
            self.stream_caught = caught
        self.world.step({"graduate": (o, np.asarray(objects["graduate"].quat_xyzw, float))}, dt, {})

    def step(self, objects) -> None:
        self.a, self.b = joint_angle("tube_a", STOW_A), joint_angle("tube_b", STOW_B)
        self.v = joint_angle("valve", 0.0)
        self.flow(objects, DT)
        if touches(objects):
            self.touched = True
        gp, _ = _state_pose(objects, "graduate")
        vp, _ = _state_pose(objects, "drainbag/valve")
        now = np.concatenate([gp, vp])
        still = self.last is not None and float(np.abs(now - self.last).max()) <= STILL_STEP
        self.last = now
        self.good = self.good + 1 if (self.done_state(objects) and still) else 0

    def released(self, objects) -> bool:
        spout_parts = (gk.Seg((0.0, 0.0, 0.0), (0.0, 0.0, -LV), COLLAR_R),)
        tube_parts = (gk.Seg((0.0, 0.0, 0.0), (0.0, 0.0, -LB), TUBE_R),)
        return bool(gk.released_clear(objects, "drainbag/valve", spout_parts, RELEASE_MARGIN)
                    and gk.released_clear(objects, "drainbag/tube_b", tube_parts, RELEASE_MARGIN)
                    and gk.released_clear(objects, "graduate", None, RELEASE_MARGIN))

    def done_state(self, objects) -> bool:
        _o, R = graduate_frame(objects)
        return (self.graduate_ml() >= FULL_ML and self.spilled_ml() <= CLEAN_ML and not self.touched
                and valve_open_fraction(self.v) == 0.0 and stowed(self.a, self.b) and tilt_deg(R) <= UPRIGHT_DEG
                and self.released(objects))

    def tier(self, objects) -> float:
        g, s = self.graduate_ml(), self.spilled_ml()
        if s > SPILL_MAX_ML:
            return 0.0
        _o, R = graduate_frame(objects)
        upright = tilt_deg(R) <= UPRIGHT_DEG
        closed = valve_open_fraction(self.v) == 0.0
        if g >= FULL_ML and closed and upright:
            out = 0.75
            if self.good >= HOLD_STEPS:
                out = 1.0
        elif (g >= FULL_ML) or (g >= HALF_ML and closed and upright):
            out = 0.5
        elif g >= TRACE_ML:
            out = 0.25
        else:
            out = 0.0
        return min(out, TOUCH_CAP) if self.touched else out


_L = Ledger()
_DEBUG = int(os.environ.get("DRAINBAG_DEBUG", "0") or 0)


def _update(objects):
    """Advance the trial's record (and the urine) once per control step; success() and score() share it."""
    global _L
    if _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    _L.step(objects)
    if _DEBUG and n is not None and n % _DEBUG == 0:  # probe log only (DRAINBAG_DEBUG=<every n steps>)
        tip, _d, _p, _R_ = spout(objects)
        print(f"drain step {n}: bag {_L.bag / ML:.1f} mL, graduate {_L.graduate_ml():.1f} mL, spilled "
              f"{_L.spilled_ml():.2f} mL, valve {math.degrees(_L.v):.0f} deg, stub {math.degrees(_L.a):.0f} tube "
              f"{math.degrees(_L.b):.0f} deg, stowed {stowed(_L.a, _L.b)}, spout z {float(tip[2]) - TOP:.3f}, touched "
              f"{_L.touched}, tier {_L.tier(objects)}", flush=True)
        if _DEBUG and "left_gripper" in objects and "right_gripper" in objects:  # probe log only
            col = _p + _R_ @ np.array([0.0, 0.0, -LV / 2])  # (DRAINBAG_DEBUG)
            grip_l = _p + _R_ @ np.array([0.0, 0.0, TUBE_GRIP - LV / 2])  # (DRAINBAG_DEBUG)
            _go, Rg = graduate_frame(objects)  # (inside the DRAINBAG_DEBUG guard above)
            print(f"  left hand - tube grip mm {np.round((objects['left_gripper'].pos - grip_l) * 1000, 1)}, right hand - "
                  f"collar mm {np.round((objects['right_gripper'].pos - col) * 1000, 1)}, openings "
                  f"{kit.opening(objects, 'left_gripper'):.2f}/{kit.opening(objects, 'right_gripper'):.2f}, bag swing "
                  f"{math.degrees(joint_angle('bag', 0.0)):.1f} deg, graduate tilt {tilt_deg(Rg):.1f} deg", flush=True)


def graduate_ml(objects=None) -> float:
    """mL of urine in the graduate."""
    return _L.graduate_ml()


def spilled_ml(objects=None) -> float:
    """mL that landed outside the graduate (on the underpad or the floor)."""
    return _L.spilled_ml()


def success(objects):
    """At least 570 mL in the graduate, at most 5 mL spilled, the spout never touched the graduate, the valve closed,
    the tube back in its holder, the graduate upright, both hands clear and everything still: this step counts toward
    the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """0 when more than 20 mL is spilled. Otherwise: 1.0 for at least 570 mL in the graduate with the valve closed, the
    graduate upright, at most 5 mL spilled, the tube back in its holder and both hands clear, still, for 1 s; 0.75 for
    at least 570 mL with the valve closed and the graduate upright; 0.5 for at least 570 mL otherwise (valve left
    open, or the graduate tipped after), or at least 300 mL with the valve closed and the graduate upright; 0.25 for
    at least 100 mL. A spout touch at any time caps the trial at 0.25. Doing nothing scores 0."""
    _update(objects)
    return _L.tier(objects)


# -- drawing the water (Isaac Lab markers; never affects the score) ------------------------------------------------------
# Created when the env is built (so the renderer has compiled the material long before the first frame) and drawn in
# its start state on every scene reset, before the settle steps and the reset renders: frame 0 already shows the full
# bag. After that it is redrawn every control step from the trial's record.
_VIS = {"markers": None, "failed": False}
SURFACE_T = 0.002
STREAM_R = 0.004  # m: the drawn stream's radius with the valve fully open (the 6 mm bore's jet, a little widened so it
STREAM_R_MIN = 0.0015  # shows on the 320 px camera); thinner while the valve is only partly open, never below this
STREAM_SEGMENTS = 24
STREAM_INTO = 0.004  # m: the drawn stream starts this far inside the spout and ends this far below the surface it meets
BAG_SLICES = 12
_HIDDEN = (np.array([0.0, 0.0, -1.0]), (1.0, 0.0, 0.0, 0.0), (1e-4, 1e-4, 1e-4))  # under the floor, for an empty draw


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        try:  # partial opacity in RTX real-time (otherwise opacity < 1 is ignored and the water draws opaque)
            import carb
            carb.settings.get_settings().set_bool("/rtx/raytracing/fractionalCutoutOpacity", True)
        except Exception:  # noqa: BLE001
            pass
        lin = tuple(kit._linear(c) for c in WATER_RGB)
        mat = sim_utils.PreviewSurfaceCfg(diffuse_color=lin, roughness=0.05, opacity=WATER_OPACITY)
        jet = sim_utils.PreviewSurfaceCfg(diffuse_color=lin, roughness=0.05, opacity=JET_OPACITY)
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/World/Visuals/water",
            markers={"disc": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=mat),
                     "block": sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=mat),
                     "jet": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=jet)}))
    except Exception as e:  # noqa: BLE001  (visuals are optional: scoring never depends on them)
        sys.stderr.write(f"drainbag: no visuals ({e!r})\n")
        _VIS["failed"] = True
    return _VIS["markers"]


def _wxyz(R) -> tuple:
    x, y, z, w = lk._quat(R)
    return (w, x, y, z)


def _seg(p, q, r):
    """(center, quat wxyz, scale) of a unit cylinder drawn from p to q with radius r."""
    v = np.asarray(q, float) - np.asarray(p, float)
    n = float(np.linalg.norm(v))
    if n < 1e-5:
        return None
    zc = v / n
    ref = np.array([1.0, 0.0, 0.0]) if abs(zc[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
    xc = np.cross(ref, zc)
    xc /= np.linalg.norm(xc)
    Rs = np.stack([xc, np.cross(zc, xc), zc], 1)
    return (np.asarray(p) + np.asarray(q)) / 2, _wxyz(Rs), (r, r, n)


def stream_path(points) -> list:
    """The drawn stream's centre line: the arc, started a little inside the spout and run a little below the surface
    it lands on, so it meets both without a gap."""
    pts = [np.asarray(p, float) for p in points]
    if len(pts) < 2:
        return pts
    d0, d1 = pts[1] - pts[0], pts[-1] - pts[-2]
    n0, n1 = float(np.linalg.norm(d0)), float(np.linalg.norm(d1))
    if n0 > 1e-6:
        pts[0] = pts[0] - d0 / n0 * STREAM_INTO
    if n1 > 1e-6:
        pts[-1] = pts[-1] + d1 / n1 * STREAM_INTO
    return pts


def inside_path(points, o, R) -> list:
    """A caught stream's drawn path inside the graduate: the ballistic arc until it meets the inside wall (a fast jet
    reaches the near wall before the surface), then down that wall to the end height (it runs down the wall)."""
    pts = [np.asarray(p, float) for p in points]
    if len(pts) < 2:
        return pts
    loc = [R.T @ (p - o) for p in pts]

    def out_of(lp, margin=0.001 + STREAM_R):  # the drawn tube's surface stays inside the wall
        h = min(max(float(lp[2]), 0.0), GR_H)
        return float(lp[2]) < GR_H and math.hypot(lp[0], lp[1]) > GR_B + (GR_T - GR_B) * h / GR_H - margin

    for k in range(1, len(loc)):
        if out_of(loc[k]) and not out_of(loc[k - 1]):
            lo, hi = loc[k - 1], loc[k]
            for _ in range(30):
                mid = (lo + hi) / 2
                lo, hi = (lo, mid) if out_of(mid) else (mid, hi)
            hit = lo
            ang = math.atan2(hit[1], hit[0])
            z_end = float(loc[-1][2])
            run = []
            for h in np.linspace(float(hit[2]), z_end, 6)[1:]:
                rr = GR_B + (GR_T - GR_B) * min(max(h, 0.0), GR_H) / GR_H - 0.001 - STREAM_R
                run.append(np.array([rr * math.cos(ang), rr * math.sin(ang), h]))
            return [o + R @ lp for lp in loc[:k]] + [o + R @ hit] + [o + R @ lp for lp in run]
    return pts


def drawing(objects, L=None, segments: bool = True) -> list:
    """(prototype index, center, quat wxyz, scale) of what is drawn: the water in the bag (slices from its bottom seam
    to its level, each as thick as the filled bag there, so the bag shows full and visibly empties), the surface in the
    graduate (at the frustum's exact level for its volume) and, only where the stream's tube mesh is unavailable, the
    stream as end-to-end cylinders along stream_line()."""
    L = _L if L is None else L
    out = []
    p, R = _state_pose(objects, "drainbag/bag")
    depth = bag_depth(L.bag)
    if depth > 0.001:
        floor = p + R @ np.asarray(_PORT_L, float)
        h = depth / BAG_SLICES
        for k in range(BAG_SLICES):
            zc = (k + 0.5) * h
            t = max(bag_thickness(zc) - 0.004, 0.003)
            out.append((1, floor + R @ np.array([0.0, 0.0, zc]), _wxyz(R), (t, BAG_W - 0.010, h * 1.02)))
    o, Rg = graduate_frame(objects)
    vol = L.world.vessels["graduate"].volume
    if vol > 0.0:
        if tilt_deg(Rg) <= 15.0:  # upright: a level disc at the exact depth, just inside the wall
            hd = graduate_depth(vol)
            r = GR_B + (GR_T - GR_B) * hd / GR_H - 0.001 - hd * math.tan(math.radians(tilt_deg(Rg)))
            out.append((0, o + Rg @ np.array([0.0, 0.0, hd]), (1.0, 0.0, 0.0, 0.0), (r, r, SURFACE_T)))
        else:
            disc = L.world.vessels["graduate"].surface_disc(o, np.asarray(objects["graduate"].quat_xyzw, float))
            if disc is not None:
                c, (x, y, z, w), (a, b) = disc
                out.append((0, c, (w, x, y, z), (a, b, SURFACE_T)))
    line = stream_line(objects, L) if segments else None
    if line is not None:
        pts, r = line
        for p0, p1 in zip(pts[:-1], pts[1:]):
            seg = _seg(p0, p1, r)
            if seg is not None:
                out.append((2, *seg))
    return out


def stream_line(objects, L=None):
    """(centre line points, radius) of the drawn stream, or None while nothing flows."""
    L = _L if L is None else L
    if L.stream is None:
        return None
    r = max(STREAM_R * math.sqrt(max(L.stream_frac, 0.0)), STREAM_R_MIN)
    pts = L.stream
    if getattr(L, "stream_caught", False):
        o, Rg = graduate_frame(objects)
        pts = inside_path(pts, o, Rg)
    return stream_path(pts), r


JET_RINGS, JET_SIDES = 40, 12


def jet_tube(pts, r) -> np.ndarray:
    """Vertices (JET_RINGS x JET_SIDES) of a smooth tube of radius r along the polyline pts, resampled evenly by length
    (fixed topology, so the mesh only moves its points)."""
    pts = np.asarray(pts, float)
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    u = np.linspace(0.0, s[-1], JET_RINGS)
    c = np.stack([np.interp(u, s, pts[:, k]) for k in range(3)], 1)
    t = np.gradient(c, axis=0)
    t /= np.maximum(np.linalg.norm(t, axis=1, keepdims=True), 1e-9)
    side = np.cross(t[0], [0.0, 0.0, 1.0])
    side = side / np.linalg.norm(side) if np.linalg.norm(side) > 1e-6 else np.array([0.0, 1.0, 0.0])
    ang = np.linspace(0.0, 2 * math.pi, JET_SIDES, endpoint=False)
    out = np.zeros((JET_RINGS, JET_SIDES, 3))
    for k in range(JET_RINGS):
        a = side - (side @ t[k]) * t[k]
        a /= max(np.linalg.norm(a), 1e-9)
        b = np.cross(t[k], a)
        out[k] = c[k] + r * (np.outer(np.cos(ang), a) + np.outer(np.sin(ang), b))
    return out.reshape(-1, 3)


def _jet():
    """The stream's tube mesh (one smooth surface, no segment seams), made with the markers; None if it cannot be."""
    if "jet" in _VIS:
        return _VIS["jet"]
    _VIS["jet"] = None
    try:
        import isaaclab.sim as sim_utils
        import omni.usd
        from pxr import Gf, UsdGeom, UsdShade, Vt

        stage = omni.usd.get_context().get_stage()
        path, mat_path = "/World/Visuals/water_jet", "/World/Visuals/water_jet_material"
        lin = tuple(kit._linear(c) for c in WATER_RGB)
        cfg = sim_utils.PreviewSurfaceCfg(diffuse_color=lin, roughness=0.05, opacity=JET_OPACITY)
        cfg.func(mat_path, cfg)
        mesh = UsdGeom.Mesh.Define(stage, path)
        n, m = JET_RINGS, JET_SIDES
        idx = []
        for k in range(n - 1):
            for j in range(m):
                j2 = (j + 1) % m
                idx += [k * m + j, k * m + j2, (k + 1) * m + j2, (k + 1) * m + j]
        mesh.CreateFaceVertexCountsAttr(Vt.IntArray([4] * ((n - 1) * m)))
        mesh.CreateFaceVertexIndicesAttr(Vt.IntArray(idx))
        mesh.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        mesh.CreateDoubleSidedAttr(True)
        mesh.CreatePointsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, -1.0)] * (n * m)))
        mesh.CreateNormalsAttr(Vt.Vec3fArray([Gf.Vec3f(0.0, 0.0, 1.0)] * (n * m)))
        mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(UsdShade.Material(stage.GetPrimAtPath(mat_path)))
        _VIS["jet"] = mesh
        _VIS["jet_api"] = (Gf, Vt)
    except Exception as e:  # noqa: BLE001  (fall back to drawing the stream as marker segments)
        sys.stderr.write(f"drainbag: no stream mesh, segments instead ({e!r})\n")
    return _VIS["jet"]


def _draw_jet(mesh, line, origin) -> None:
    Gf, Vt = _VIS["jet_api"]
    if line is None:  # nothing flows: the tube shrinks to a hidden point under the floor
        pts = np.tile(np.array([0.0, 0.0, -1.0]) + origin, (JET_RINGS * JET_SIDES, 1))
        nrm = np.tile([0.0, 0.0, 1.0], (JET_RINGS * JET_SIDES, 1))
    else:
        c, r = line
        pts = jet_tube(c, r)
        nrm = (pts - jet_tube(c, 0.0)) / max(r, 1e-9)
        pts = pts + origin
    mesh.GetPointsAttr().Set(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
    mesh.GetNormalsAttr().Set(Vt.Vec3fArray.FromNumpy(nrm.astype(np.float32)))


def _draw(objects, origin, L=None) -> None:
    m = _markers()
    if m is None:
        return
    mesh = _jet()
    items = drawing(objects, L, segments=mesh is None) or [(0, *_HIDDEN)]
    if mesh is not None:
        _draw_jet(mesh, stream_line(objects, L), origin)
    m.visualize(translations=np.array([np.asarray(c, float) + origin for _, c, _, _ in items], float),
                orientations=np.array([q for _, _, q, _ in items], float),
                scales=np.array([s for _, _, _, s in items], float),
                marker_indices=np.array([i for i, _, _, _ in items], int))


def _start_objects(base) -> dict:
    """The bag's and the graduate's poses straight from the sim, read without yamkit's per-step joint laws (which must
    not act before the first control step), as states for drawing()."""
    from types import SimpleNamespace

    np_ = lambda t: getattr(t, "torch", t).detach().cpu().numpy().astype(np.float64)  # noqa: E731
    origin = np_(base.scene.env_origins)[0]
    art = base.scene[BAG.name]
    i = list(art.body_names).index(BAG_LINK.name)
    bag = SimpleNamespace(pos=np_(art.data.body_pos_w)[0][i] - origin, quat_xyzw=np_(art.data.body_quat_w)[0][i])
    gp = np_(base.arena_world.get_pose_w(GRADUATE.name))[0]
    return {"drainbag/bag": bag, "graduate": SimpleNamespace(pos=gp[:3] - origin, quat_xyzw=gp[3:])}


def _prime(env) -> None:
    """At env build: create the markers now and draw the start state after every scene reset."""
    base = env.unwrapped
    if getattr(base, "_drainbag_vis", False) or _markers() is None:
        return
    _jet()
    base._drainbag_vis = True
    inner = base.reset

    def reset(*a, **k):
        out = inner(*a, **k)
        if not _VIS["failed"]:
            try:
                _draw(_start_objects(base), base.scene.env_origins[0].detach().cpu().numpy(), Ledger())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                sys.stderr.write(f"drainbag: start drawing off ({e!r})\n")
        return out

    base.reset = reset


# -- the simulator hook: the drain steps once per control step, after yamkit's object_states ------------------------------
def _patch(scene) -> None:
    if getattr(scene, "_drainbag", False):
        return
    scene._drainbag = True
    inner = scene.object_states

    def object_states(env, names):
        out = inner(env, names)
        if all(n in out for n in ("drainbag/bag", "drainbag/valve", "graduate")):
            _update(out)
            if not _VIS["failed"]:
                try:
                    _draw(out, env.unwrapped.scene.env_origins[0].detach().cpu().numpy())
                except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                    sys.stderr.write(f"drainbag: visuals off ({e!r})\n")
                    _VIS["failed"] = True
        return out

    scene.object_states = object_states
    make = getattr(scene, "make_env", None)
    if make is not None:
        def make_env(*a, **k):
            env = make(*a, **k)
            try:
                _prime(env)
            except Exception as e:  # noqa: BLE001
                sys.stderr.write(f"drainbag: visuals not primed ({e!r})\n")
            return env

        scene.make_env = make_env


class _Hook(kit._Hook):
    def exec_module(self, module):
        self._inner.exec_module(module)
        kit._patch(module)
        _patch(module)


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:  # replace yamkit's import hook with one that also adds the drain step after yamkit's patch
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook())


# -- scripted demo: open loop from the fixed layout ------------------------------------------------------------------------
# The way a nursing assistant empties the bag, with the two hands split the same way. Right hand: takes the graduate by
# its handle's top arm (from above), carries it onto the underpad beneath the drain and lets go. Left hand (the aiming
# hand): takes the drain tube out of its holder clip, holding it 7 cm up from the valve, pulls it down out of the clip,
# swings it out and down until the spout points into the graduate's mouth without touching it, and holds it there.
# Right hand (the valve hand): takes the valve collar and twists it open, lets go (a twist valve stays where it is put)
# and stands by. As the bag empties, its level and so the stream's speed drop and the stream's arc shortens; the aiming
# hand follows it, moving the spout so the stream keeps landing on the same spot in the mouth. When the bag is empty the
# valve hand takes the collar again and twists it closed; the aiming hand swings the tube back up and pushes the spout
# into its clip; both hands let go and back away. (The right hand cannot hold the tube fingers-across at the clip: a
# global IK search finds no pose within 35 mm; the left one can.)
X_YAW = -math.pi / 2  # top-down fingers closing across world x (turned this way: the wrist stays inside its limits)
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open (guardkit's gripper model)
DRAIN_A = math.radians(120.0)  # the run out of the outlet raised 30 degrees above horizontal, out toward the robot
DRAIN_PHI = math.radians(60.0)  # the spout's axis 60 degrees from straight down, toward the robot
PULL_A = STOW_A - 0.35  # the stub lowered 20 degrees: the spout comes down out of the clip
RAISE_PHI = math.pi - 0.5  # the tube already leans out toward the robot as the stub rises: the palm keeps off the bag
TUBE_GRIP = 0.085  # m: the aiming hand holds the tube this far up from the collar's middle (room for the valve hand)
COLLAR_GRIP_UP = 0.004  # m: the valve hand pinches the collar this far toward the tube from its middle
APPROACH = 0.070  # m: a hand comes in to the tube along its fingers from this far back
CARRY_DZ = 0.040  # m: the graduate is carried this far above the table
SLIDE_OFF = 0.030  # m: the open fingers slide this far out along the handle's arm (past its end) before lifting
# Grips inside the measured envelope (1-20 mm parts, rubber-like pads): fingers aimed 2 mm inside each side, ~10 N.
TUBE_FINGER = None  # set below (needs finger_for)
TWIST_OPEN_PAST = math.radians(5.0)  # the valve hand turns this far past the open stop (it slips there; more is out of reach)
TWIST_CLOSE_PAST = 0.0  # ... and to the closed stop when closing (past it is out of the right wrist's reach)
# Across the fingers, the collar rolls in the pads by up to ~35 degrees of a 90-degree twist (probes p8-p10), so the
# plan assumes the valve ends 75 % open for its aim and holds long enough for 97 % of the bag at 60 % open.
PLAN_OPEN_FRAC = 0.75
HOLD_OPEN_FRAC = 0.6
AIM_STEPS = 6  # waypoints of the aiming hand's following move
AIM_TIP_RIM = (0.035, 0.060)  # m: the spout's end stays this high above the rim while it follows the stream


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * max(0.0, half) / FINGER_HALF_OPEN


TUBE_FINGER = finger_for(TUBE_R - 0.002)
GRASP_FINGER = finger_for(COLLAR_R - 0.002)
WIDE = finger_for(0.011)  # 5 mm each side of the collar: the far pad must fit between it and the bag's front
TUBE_WIDE = finger_for(0.0095)


def hand_angles(m) -> tuple:
    """(yaw, roll, pitch) of a kit.Tilted waypoint whose hand is turned by m from the straight-down grip, written as
    pitch(y) @ roll(x) @ yaw(z)."""
    m = np.asarray(m, float)
    r = math.asin(max(-1.0, min(1.0, -m[1, 2])))
    if math.cos(r) < 1e-6:
        # gimbal lock (roll +-90 degrees, which every untwisted tube grip is): only pitch -+ yaw is defined, so take
        # yaw 0 and put the whole turn in the pitch (probes p2-p7: atan2(0, 0) dropped both and the hand never rolled)
        s = 1.0 if r > 0 else -1.0
        return 0.0, r, math.atan2(s * m[0, 1], m[0, 0])
    return math.atan2(m[1, 0], m[1, 1]), r, math.atan2(m[0, 2], m[2, 2])


def tilted(duration: float, pos, finger: float, label: str, m) -> kit.Tilted:
    yaw, r, p = hand_angles(m)
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label, yaw=yaw, roll=r, pitch=p)


def tube_angles(a: float, phi: float) -> tuple:
    """(stub, tube) joint angles that put the stub at `a` and the tube's (and spout's) axis `phi` from straight down
    toward the robot (pi: straight up)."""
    return a, phi - a


def hand_turn(phi: float, twist: float, side: float) -> np.ndarray:
    """A hand's turn from the straight-down grip when it holds the tube (or collar) of a tube pointing `phi`: fingers
    pointing along `side` * y (square to the plane the tube swings in: -1 the left hand's, pointing right; +1 the right
    hand's, pointing left), closing across the tube in that plane, then turned `twist` about the tube's axis
    (negative turns the collar toward its open stop). Swinging the tube is then a roll of the wrist about the
    fingers' direction."""
    d = np.array([-math.sin(phi), 0.0, -math.cos(phi)])  # the spout's outward axis
    c = -np.array([math.cos(phi), 0.0, -math.sin(phi)])  # the closing direction, in the swing plane square to d
    a = np.array([0.0, side, 0.0])
    m = np.stack([np.cross(c, -a), c, -a], 1)  # takes the straight-down grip's x, y (closing), z (back up the arm)
    return axis_rot(-d, twist) @ m


def collar_turn(phi: float, twist: float) -> np.ndarray:
    """The valve hand's (right hand's) turn on the collar."""
    return hand_turn(phi, twist, 1.0)


def tube_turn(phi: float) -> np.ndarray:
    """The aiming hand's (left hand's) turn on the tube."""
    return hand_turn(phi, 0.0, -1.0)


def _axis(a: float, phi: float) -> tuple:
    f = fk(a=tube_angles(a, phi)[0], b=tube_angles(a, phi)[1])
    d = (f["tip"] - f["collar"]) / np.linalg.norm(f["tip"] - f["collar"])
    return f, d


def tube_wp(duration, a, phi, finger, label, back=0.0) -> kit.Tilted:
    """An aiming-hand waypoint on the tube (TUBE_GRIP up from the collar) with the tube at (stub a, axis phi), the hand
    `back` m behind it along its fingers."""
    f, d = _axis(a, phi)
    m = tube_turn(phi)
    return tilted(duration, f["collar"] - TUBE_GRIP * d + back * m[:, 2], finger, label, m)


def collar_wp(duration, a, phi, twist, finger, label, back=0.0) -> kit.Tilted:
    """A valve-hand waypoint on the collar with the tube at (stub a, axis phi), the hand `back` m behind it along its
    fingers."""
    f, d = _axis(a, phi)
    m = collar_turn(phi, twist)
    return tilted(duration, f["collar"] - COLLAR_GRIP_UP * d + back * m[:, 2], finger, label, m)


# -- the stream the aiming hand follows (the scorer's own drain model, run ahead offline) ---------------------------------
def tip_head(a: float, phi: float, volume_m3: float) -> tuple:
    """(spout end, outward axis, head) for the tube at (a, phi) and this much urine in the (unswung) bag."""
    f, d = _axis(a, phi)
    floor = float(f["bag"][0][2]) - STRAP - BAG_H
    return f["tip"], d, floor + bag_depth(volume_m3) - float(f["tip"][2])


def landing(a: float, phi: float, volume_m3: float):
    """Where the stream crosses the rim's plane for the tube at (a, phi) and this much in the bag (None: no flow)."""
    tip, d, head = tip_head(a, phi, volume_m3)
    if head <= 0.0:
        return None
    return land(tip, math.sqrt(2 * G * head) * d, TOP + GR_RIM_Z)


def aim_point() -> np.ndarray:
    """Where the stream is kept landing (on the rim's plane): where the full bag's stream lands at the draining pose."""
    return np.asarray(landing(DRAIN_A, DRAIN_PHI, START_ML * ML), float)


def graduate_place() -> tuple:
    """The graduate is set down centered between the spout's end at the draining pose and the aim point: the spout is
    over the mouth and the stream lands inside it."""
    f, _d = _axis(DRAIN_A, DRAIN_PHI)
    c = (f["tip"][:2] + aim_point()) / 2
    return float(c[0]), float(c[1])


def drain_seconds(fill_ml: float = START_ML, upto: float = 0.995, frac: float = 1.0) -> float:
    """How long the valve, `frac` open, takes to pass `upto` of the bag at the demo's draining pose (offline
    integration)."""
    v, t = fill_ml * ML, 0.0
    while v > (1 - upto) * fill_ml * ML and t < 600:
        _tip, _d, head = tip_head(DRAIN_A, DRAIN_PHI, v)
        if head <= 0:
            break
        v -= CD * OUTLET_A * frac * math.sqrt(2 * G * head) * DT
        t += DT
    return t


def predicted_volume(t: float, frac: float = PLAN_OPEN_FRAC) -> float:
    """m^3 left in the bag t s after the valve opens `frac` (at the draining pose)."""
    v = START_ML * ML
    for _ in range(int(round(t / DT))):
        _tip, _d, head = tip_head(DRAIN_A, DRAIN_PHI, v)
        if head <= 0:
            break
        v = max(v - CD * OUTLET_A * frac * math.sqrt(2 * G * head) * DT, 0.0)
    return v


def aim_pose(volume_m3: float, prev=(DRAIN_A, DRAIN_PHI)) -> tuple:
    """The tube pose (stub a, axis phi) near `prev` that lands the stream for this bag volume closest to the mouth's
    aim point, with the spout's end AIM_TIP_RIM above the rim."""
    goal = aim_point()
    best, out = math.inf, prev
    for da in np.radians(np.arange(-6.0, 6.01, 0.5)):
        for dp in np.radians(np.arange(-6.0, 6.01, 0.5)):
            a, phi = prev[0] + da, prev[1] + dp
            tip, _d, head = tip_head(a, phi, volume_m3)
            rise = float(tip[2]) - (TOP + GR_RIM_Z)
            if not AIM_TIP_RIM[0] <= rise <= AIM_TIP_RIM[1] or head <= 0:
                continue
            hit = landing(a, phi, volume_m3)
            cost = float(np.linalg.norm(hit - goal)) + 0.002 * math.hypot(da, dp)
            if cost < best:
                best, out = cost, (a, phi)
    return out


# -- the timeline ---------------------------------------------------------------------------------------------------------
GRADUATE_DONE_S = 2.0 + 1.5 + 0.8 + 1.2 + 3.0 + 1.5 + 0.8 + 1.2 + 1.5  # the graduate is down and the right hand clear
LEFT_DONE_S = GRADUATE_DONE_S  # (name kept for the tests)
T_TUBE_IN = GRADUATE_DONE_S + 0.5  # the aiming hand moves in to the tube
T_AIMED = T_TUBE_IN + 1.5 + 0.8 + 1.5 + 2 * 1.2 + 4 * 1.0 + 1.0  # the spout is over the mouth, steady
VALVE_IN_S, VALVE_GRIP_S, TWIST_S, LET_GO_S, BACK_S = 1.5, 0.8, 4.0, 0.8, 1.5
T_OPENING = T_AIMED + VALVE_IN_S + VALVE_GRIP_S  # the twist starts
T_FLOW = T_OPENING + TWIST_S / 4  # the valve passes 10 degrees about here: flow starts
T_FOLLOW = T_OPENING + TWIST_S + LET_GO_S + BACK_S  # the valve hand is clear: the aiming hand starts to follow
HOLD_S = math.ceil(drain_seconds(upto=0.97, frac=HOLD_OPEN_FRAC)) + 2.0
T_CLOSING = T_FLOW + HOLD_S  # the valve hand comes back in to close


def aim_schedule() -> list:
    """(time s, stub a, axis phi) of the aiming hand's following waypoints, from T_FOLLOW to T_CLOSING: each pose lands
    the stream the plan predicts for that moment (PLAN_OPEN_FRAC open) on the aim point."""
    out, prev = [], (DRAIN_A, DRAIN_PHI)
    for k in range(1, AIM_STEPS + 1):
        t = T_FOLLOW + (T_CLOSING - T_FOLLOW) * k / AIM_STEPS
        v = predicted_volume(min(t, T_CLOSING - 2.0) - T_FLOW)
        if v < 0.03 * START_ML * ML:  # the last trickle falls where it falls: keep the pose
            out.append((t, *prev))
            continue
        prev = aim_pose(v, prev)
        out.append((t, *prev))
    return out


def swing_path(a0, phi0, a1, phi1, n: int) -> list:
    return [(a0 + (a1 - a0) * k / n, phi0 + (phi1 - phi0) * k / n) for k in range(1, n + 1)]


def graduate_moves() -> list:
    gx, gy = GR_START
    px, py = graduate_place()
    hx, hy, hz = HANDLE_GRIP
    z = TOP + hz
    # a firm squeeze (fingers commanded shut): the graduate hangs 7 cm off the pinch, and a light one let it turn
    # about the closing axis and tip (probe p1)
    open_, hold = finger_for(0.016), FINGER_CLOSED
    from yam_arena.demo import Waypoint

    def w(d, pos, f, label):
        return Waypoint(float(d), tuple(float(v) for v in pos), float(f), label, X_YAW)

    return [w(2.0, (gx + hx, gy + hy, z + 0.08), open_, "over the graduate's handle"),
            w(1.5, (gx + hx, gy + hy, z), open_, "around the handle's top arm"),
            w(0.8, (gx + hx, gy + hy, z), hold, "grip"),
            w(1.2, (gx + hx, gy + hy, z + CARRY_DZ), hold, "lift"),
            w(3.0, (px + hx, py + hy, z + CARRY_DZ), hold, "carry it beneath the bag"),
            w(1.5, (px + hx, py + hy, z + 0.001), hold, "set it on the underpad"),
            w(0.8, (px + hx, py + hy, z + 0.001), open_, "let go"),
            w(1.2, (px + hx, py + hy + GR_SIDE * SLIDE_OFF, z + 0.001), open_, "slide off the end of the handle's arm"),
            w(1.5, (px + hx - 0.06, py + hy + GR_SIDE * (SLIDE_OFF + 0.04), z + 0.06), open_,
              "lift clear, back toward the robot")]


def right_plan() -> list:
    """The right hand: the graduate onto the underpad, then the valve: open, stand by, close."""
    plan = graduate_moves()
    a0, p0 = DRAIN_A, DRAIN_PHI
    _t, a1, p1 = aim_schedule()[-1]
    t_open = -VALVE_OPEN - TWIST_OPEN_PAST
    plan.append(collar_wp(max(T_AIMED - GRADUATE_DONE_S, 1.0), a0, p0, 0.0, WIDE,
                          "beside the valve collar, waiting for the spout to be aimed", back=APPROACH))
    plan += [collar_wp(VALVE_IN_S, a0, p0, 0.0, WIDE, "around the collar"),
             collar_wp(VALVE_GRIP_S, a0, p0, 0.0, GRASP_FINGER, "grip the collar"),
             collar_wp(TWIST_S, a0, p0, t_open, GRASP_FINGER, "twist the valve open"),
             collar_wp(LET_GO_S, a0, p0, t_open, WIDE, "let go: the valve stays open"),
             collar_wp(BACK_S, a0, p0, t_open, WIDE, "back away from the collar", back=APPROACH),
             collar_wp(max(T_CLOSING - T_FOLLOW - VALVE_IN_S, 1.0), a1, p1, t_open, WIDE,
                       "stand by while the bag empties", back=APPROACH),
             collar_wp(VALVE_IN_S, a1, p1, t_open, WIDE, "back around the collar"),
             collar_wp(VALVE_GRIP_S, a1, p1, t_open, GRASP_FINGER, "grip the collar"),
             collar_wp(TWIST_S, a1, p1, TWIST_CLOSE_PAST, GRASP_FINGER, "twist the valve closed"),
             collar_wp(LET_GO_S, a1, p1, TWIST_CLOSE_PAST, WIDE, "let go"),
             collar_wp(BACK_S, a1, p1, TWIST_CLOSE_PAST, WIDE, "back away from the collar", back=APPROACH)]
    from yam_arena.demo import Waypoint
    plan.append(Waypoint(4.0, (0.20, -0.22, TOP + 0.20), WIDE, "out of the way", X_YAW))
    plan.append(Waypoint(6.0, (0.20, -0.22, TOP + 0.20), WIDE, "wait", X_YAW))
    return plan


T_CLOSED = T_CLOSING + VALVE_IN_S + VALVE_GRIP_S + TWIST_S + LET_GO_S + BACK_S  # the valve hand is clear again


def left_plan() -> list:
    """The left hand: the tube out of its clip, the spout aimed and following the stream, then back into the clip."""
    plan = [tube_wp(7.0, STOW_A, math.pi, TUBE_WIDE, "behind the tube in its clip", back=APPROACH),
            tube_wp(max(T_TUBE_IN - 7.0, 0.5), STOW_A, math.pi, TUBE_WIDE, "wait for the graduate to be set down",
                    back=APPROACH),
            tube_wp(1.5, STOW_A, math.pi, TUBE_WIDE, "around the tube"),
            tube_wp(0.8, STOW_A, math.pi, TUBE_FINGER, "grip the tube"),
            tube_wp(1.5, PULL_A, math.pi, TUBE_FINGER, "pull the spout down out of the clip")]
    for i, (a, phi) in enumerate(swing_path(PULL_A, math.pi, DRAIN_A, RAISE_PHI, 2)):
        plan.append(tube_wp(1.2, a, phi, TUBE_FINGER, f"raise the stub {i + 1}"))
    for i, (a, phi) in enumerate(swing_path(DRAIN_A, RAISE_PHI, DRAIN_A, DRAIN_PHI, 4)):
        plan.append(tube_wp(1.0, a, phi, TUBE_FINGER, f"swing the spout down over the mouth {i + 1}"))
    plan.append(tube_wp(1.0, DRAIN_A, DRAIN_PHI, TUBE_FINGER, "steady over the mouth"))
    plan.append(tube_wp(T_FOLLOW - T_AIMED, DRAIN_A, DRAIN_PHI, TUBE_FINGER, "hold the aim while the valve is opened"))
    t_prev = T_FOLLOW
    for i, (t, a, phi) in enumerate(aim_schedule()):
        plan.append(tube_wp(t - t_prev, a, phi, TUBE_FINGER, f"follow the stream as it shortens {i + 1}"))
        t_prev = t
    _t, a1, p1 = aim_schedule()[-1]
    plan.append(tube_wp(T_CLOSED - t_prev + 0.5, a1, p1, TUBE_FINGER, "hold the aim while the valve is closed"))
    for i, (a, phi) in enumerate(swing_path(a1, p1, DRAIN_A, RAISE_PHI, 4)):
        plan.append(tube_wp(1.0, a, phi, TUBE_FINGER, f"swing the tube back up {i + 1}"))
    for i, (a, phi) in enumerate(swing_path(DRAIN_A, RAISE_PHI, PULL_A, math.pi, 2)):
        plan.append(tube_wp(1.2, a, phi, TUBE_FINGER, f"lower the stub {i + 1}"))
    plan += [tube_wp(1.5, STOW_A, math.pi, TUBE_FINGER, "push the spout up into the clip"),
             tube_wp(0.8, STOW_A, math.pi, TUBE_WIDE, "let go"),
             tube_wp(1.5, STOW_A, math.pi, TUBE_WIDE, "back away from the tube", back=APPROACH),
             tube_wp(4.0, STOW_A, math.pi, TUBE_WIDE, "wait", back=APPROACH)]
    return plan


def demo():
    """Right hand sets the graduate on the underpad beneath the drain; left hand takes the tube out of its clip and
    aims the spout into the graduate; right hand opens the valve; the left hand follows the shortening stream; the
    right hand closes the valve; the left hand clips the tube back."""
    return {"left": left_plan(), "right": right_plan()}
