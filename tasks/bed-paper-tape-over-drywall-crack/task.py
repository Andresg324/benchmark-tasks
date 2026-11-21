"""On the station table, in front of the arms: an offcut of half-inch drywall stands in a bench stand, leaning back 20
degrees, its paper face toward the robot. A 12 cm crack runs straight up it, and a bed of wet joint compound 3 mm deep
and 10 cm wide has been laid along it. An 18.8 cm strip of 2-1/16 in. paper joint tape lies on
a 2x4 offcut on the bench to the left, its top end over the offcut's end, and a 4 in. joint knife lies on the bench to
the right.

The job is to tape the crack: lay the tape along it and bed it with the knife, so the tape is centered on the crack,
embedded with a thin layer of compound under it across its width and compound squeezed out at both edges, with no
wrinkle and no tear; then set the knife down. Tape set on the wet bed with a light press of the fingers stays there
(the compound's tack) but is not bedded; only the knife beds it.

The tape is sheetkit's row of rigid strips (6.7 mm pitch) with the real tape's width, weight per length and bending
stiffness. The compound is bedkit's height field on the panel's face. A held knife sliding with its edge on a strip
that lies on the face presses the compound under it down to a depth set by the blade's angle and squeezes the rest out
at the tape's edges; a strip pressed with a thin layer left under it bonds to the bed (a joint to the wall switched
on: it no longer lifts or slides freely); with too little compound it does not (a blister). A strip a finger presses
onto the bed sticks weakly (a much weaker joint: the wet compound's tack); it never counts as bedded. Hands do not bed
tape; only the knife does. Every threshold appears once below, with the same value in task.json and the README.
"""
from __future__ import annotations

import dataclasses
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import yamkit as kit  # noqa: E402
import sheetkit as sk  # noqa: E402
import bedkit as bk  # noqa: E402
import guardkit as gk  # noqa: E402
import yamgeom as yg  # noqa: E402
from yam_arena import demo as _demo  # noqa: E402
from yam_arena import ik as _ik  # noqa: E402
from yam_arena import station as _station  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP, SHOULDER, WRIST  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Box  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Tape the crack in the wall panel with the paper tape and the joint knife, then lay the knife back on the bench."

# -- the panel (world frame: x away from the arms, y to the left, z up; m) --------------------------------------------
LEAN = math.radians(20.0)  # the panel leans back 20 degrees from vertical in its stand
PANEL_T = 0.0127  # half-inch drywall
PANEL_W = 0.40  # along y
PANEL_H = 0.40  # up the face
PANEL_YC = -0.04
FACE_X0 = 0.370  # x of the face's bottom edge
FACE_Z0 = TOP + PANEL_T * math.sin(LEAN)  # the bottom edge's back corner rests on the table
UP = np.array([math.sin(LEAN), 0.0, math.cos(LEAN)])  # up the face
NORMAL = np.array([-math.cos(LEAN), 0.0, math.sin(LEAN)])  # out of the face, toward the robot
R_PANEL = np.array([[math.cos(LEAN), 0.0, math.sin(LEAN)], [0.0, 1.0, 0.0], [-math.sin(LEAN), 0.0, math.cos(LEAN)]])
PANEL_POS = np.array([FACE_X0, PANEL_YC, FACE_Z0]) + R_PANEL @ np.array([PANEL_T / 2, 0.0, PANEL_H / 2])
FACE_V0 = PANEL_YC - PANEL_W / 2  # y of the face's right edge (face coordinate v = 0)


def face_point(s: float, y: float, d: float = 0.0) -> np.ndarray:
    """World point `s` m up the face from its bottom edge, at `y`, `d` m out from the face (negative: into it)."""
    return np.array([FACE_X0, y, FACE_Z0]) + s * UP + d * NORMAL


# -- the crack, the tape's line on it, and the bed of compound ---------------------------------------------------------
CRACK_LEN = 0.12  # m: a short settling crack (about 5 in.); see the README for why not longer
CRACK_OFF_FALL = 0.0  # the crack runs straight up the face (down the fall line)
CRACK_MID = (0.215, -0.02)  # (s up the face, y) of the crack's middle: its bottom end 14.5 cm up the face, so the knife
# (drawn down from the held top end, its handle hanging below the edge) ends its stroke clear of the bench
CRACK_UP = math.cos(CRACK_OFF_FALL) * UP + math.sin(CRACK_OFF_FALL) * np.array([0.0, 1.0, 0.0])  # along it, upward
DOWN = -CRACK_UP  # the knife's draw direction, down the tape
ACROSS = np.cross(NORMAL, DOWN) / np.linalg.norm(np.cross(NORMAL, DOWN))  # across the tape, in the face's plane
CRACK_MID_W = face_point(*CRACK_MID)
CRACK_BOT = CRACK_MID_W - CRACK_UP * CRACK_LEN / 2
CRACK_TOP = CRACK_MID_W + CRACK_UP * CRACK_LEN / 2
STATION = 0.01  # m between scored stations along the crack
STATIONS = np.array([CRACK_BOT + CRACK_UP * k * STATION for k in range(round(CRACK_LEN / STATION) + 1)])  # 13
HOLD_ON_FACE = 0.020  # m: the held tape leaves the face this far up the line past the crack's top end ...
HOLD_OUT = 0.025  # ... and its held top end stands this far out from the face (folded out, between the fingers)
TOP_OVER = HOLD_ON_FACE + HOLD_OUT  # m of tape past the crack's top end
TAPE_LEN = 0.1876  # 28 strips at 6.7 mm: 2.3 cm past the crack's bottom end
BED_W = 0.10  # m: the bed is about 4 in. wide (ProForm's sheet) ...
BED_T = 0.003  # ... and about 3 mm deep (Toupret's sheet: "a depth of about 3mm (1/8")")
BED_ENDS = (CRACK_LEN / 2 + (TAPE_LEN - TOP_OVER - CRACK_LEN) + 0.010, CRACK_LEN / 2 + HOLD_ON_FACE + 0.010)
PAPER_RGB = (0.90, 0.89, 0.86)  # drywall's face paper
MUD_RGB = (0.72, 0.73, 0.74)  # wet all-purpose compound (gray until it dries white)
PENCIL_RGB = (0.25, 0.25, 0.27)


def start_bed(U, V, depth: float = BED_T) -> np.ndarray:
    """The bed on the face's cells (u up the face, v from its right edge): `depth` within BED_W / 2 of the
    crack's line, from BED_ENDS[0] below its middle to BED_ENDS[1] above."""
    pts = (np.array([FACE_X0, FACE_V0, FACE_Z0])[None, None, :] + U[..., None] * UP[None, None, :]
           + V[..., None] * np.array([0.0, 1.0, 0.0])[None, None, :]) - CRACK_MID_W
    along, across = pts @ CRACK_UP, pts @ ACROSS
    return np.where((np.abs(across) <= BED_W / 2) & (along >= -BED_ENDS[0]) & (along <= BED_ENDS[1]), depth, 0.0)


def _slab(center, along, across, length, width, thick, look):
    """A flat slab on the face (visual only), in the panel's own frame."""
    c = R_PANEL.T @ (np.asarray(center, float) - PANEL_POS)
    a, w, n = (R_PANEL.T @ np.asarray(v, float) for v in (along, across, NORMAL))
    pts = [c + sa * a * length / 2 + sw * w * width / 2 + sn * n * thick for sn in (0, 1) for sw in (-1, 1)
           for sa in (-1, 1)]
    faces = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]
    return kit._mesh(pts, [4] * 6, [i for f in faces for i in f], look)


_BED_MID = CRACK_MID_W + CRACK_UP * (BED_ENDS[1] - BED_ENDS[0]) / 2
_FACE_LOCAL_X = -PANEL_T / 2
PANEL = kit.Prop(
    "panel", pos=tuple(PANEL_POS), size=(PANEL_T, PANEL_W, PANEL_H),
    tilt_xyzw=(0.0, math.sin(LEAN / 2), 0.0, math.cos(LEAN / 2)),
    meshes=(kit.box((0, 0, 0), (PANEL_T, PANEL_W, PANEL_H), kit.Look(PAPER_RGB, 0.9)),
            # the bed, drawn 0.4 mm proud of the face (the tape, 1 mm thick in the simulator, shows on it)
            _slab(_BED_MID, CRACK_UP, ACROSS, sum(BED_ENDS), BED_W, 0.0004, kit.Look(MUD_RGB, 0.5)),
            # pencil ticks on the crack's line, 2 cm past each end of the bed (marked before the bed is laid), so
            # the line can be seen
            *(_slab(CRACK_MID_W + CRACK_UP * e, CRACK_UP, ACROSS, 0.012, 0.0012, 0.0002, kit.Look(PENCIL_RGB, 0.8))
              for e in (-BED_ENDS[0] - 0.021, BED_ENDS[1] + 0.021))),
    colliders=(kit.Collider((0, 0, 0), size=(PANEL_T, PANEL_W, PANEL_H)),), friction=(0.3, 0.25))

# The stand: a pine 1x2 stop strip in front of the bottom edge, and a 38 x 38 mm (2x2) brace behind (fixed, like the
# stand's screws).
PINE = (0.80, 0.66, 0.46)
STOP_SIZE = (0.019, PANEL_W, 0.038)
STOP = Box("stop", pos=(FACE_X0 - STOP_SIZE[0] / 2 - 0.001, PANEL_YC, TOP + STOP_SIZE[2] / 2), size=STOP_SIZE,
           color=PINE)
_BRACE_TOP = face_point(0.30, PANEL_YC, -PANEL_T - 0.019)
_BRACE_BOT = np.array([_BRACE_TOP[0] + 0.14, PANEL_YC, TOP + 0.019])
_BRACE_LEN = float(np.linalg.norm(_BRACE_TOP - _BRACE_BOT))
_BRACE_ANG = math.atan2(_BRACE_TOP[2] - _BRACE_BOT[2], _BRACE_BOT[0] - _BRACE_TOP[0])
BRACE = kit.Prop(
    "brace", pos=tuple((_BRACE_TOP + _BRACE_BOT) / 2), size=(_BRACE_LEN, 0.038, 0.038),
    tilt_xyzw=(0.0, math.sin(_BRACE_ANG / 2), 0.0, math.cos(_BRACE_ANG / 2)),
    meshes=(kit.box((0, 0, 0), (_BRACE_LEN, 0.038, 0.038), kit.Look(PINE, 0.7)),),
    colliders=(kit.Collider((0, 0, 0), size=(_BRACE_LEN, 0.038, 0.038)),))

# -- the tape -----------------------------------------------------------------------------------------------------------
# Paper joint tape: 2-1/16 in. (52.4 mm) wide (ProForm, USG: "52mm x 76m"); 0.215 mm thick and 138 g/m^2 (Toupret's data
# sheet; USG ME's 550 g per 52 mm x 76 m roll is the same weight, 139 g/m^2 with the core). Bending stiffness from the
# thickness and a paper modulus of 3.7 GPa (Niskanen): D = E t^3 / 12 (1 - nu^2) = 3.4 mN m. The yield strain and
# springback set how sharply a hinge must bend before it keeps a set (a crease); the score does not read them. The wet
# tape slides freely on the compound and the knife's steel slides freely on it (0.3 / 0.25, the panel's face too); the
# finger pads grip it (sheetkit's pad material wins any pairing).
TAPE_W, TAPE_T, TAPE_GSM = 0.0524, 0.000215, 0.138
TAPE_PAPER = sk.Paper(
    "paper joint tape", areal_mass=TAPE_GSM, thickness=TAPE_T,
    bending_stiffness=3.7e9 * TAPE_T ** 3 / (12 * 0.91), yield_strain=0.004, springback=math.radians(35.0),
    friction=0.3, kinetic_friction=0.25, tensile_strength=4000.0,
    source="2-1/16 in. (ProForm, USG); 0.215 mm, 138 g/m^2, >4 N/mm (Toupret); E 3.7 GPa (Niskanen 2008)")
TAPE_COLLIDER_T = 0.001  # the strips' contact box: the tape's 0.215 mm plus the compound under it, so the knife rides
TAPE_RGB = (0.94, 0.92, 0.86)  # on it at the real height (0.215 + about 0.8 mm)
# On the bench: on a 2x4 offcut standing on edge (38 x 89 mm) on a scrap of 3/4 in. plywood (19 mm), the top end 3.5 cm
# past the offcut's end so it can be pinched (a strip lying flat on a surface cannot be picked up by its edge with these
# fingers, and a lower overhang puts the wrist on the bench). The tape points from its top end along TAPE_DIR (away from
# the robot, to the left); its strip 0 is the top end.
PLY_H, BLOCK_W = 0.019, 0.038
BLOCK_H = PLY_H + 0.089
TAPE_END = np.array([0.180, 0.300])  # the top end's middle (x, y)
TAPE_ALPHA = math.radians(30.0)
TAPE_DIR = np.array([math.cos(TAPE_ALPHA), math.sin(TAPE_ALPHA), 0.0])
OVERHANG = 0.035
_tape_mid = np.array([*TAPE_END, 0.0]) + TAPE_DIR * TAPE_LEN / 2
TAPE = sk.Sheet.uniform("tape", width=TAPE_W, length=TAPE_LEN, paper=TAPE_PAPER,
                        pos=(float(_tape_mid[0]), float(_tape_mid[1]), TOP + BLOCK_H + TAPE_COLLIDER_T / 2 + 0.0003),
                        collider_t=TAPE_COLLIDER_T, lines=False, look=kit.Look(TAPE_RGB, 0.9))
TAPE_YAW = TAPE_ALPHA - math.pi / 2  # the sheet's +y (from strip 0, its top end, toward the bottom end) along TAPE_DIR
_BLOCK_LEN = 0.30
_block_mid = np.array([*TAPE_END, 0.0]) + TAPE_DIR * (OVERHANG + _BLOCK_LEN / 2)
_PLY = ((0.0, 0.0, -BLOCK_H / 2 + PLY_H / 2), (0.14, _BLOCK_LEN, PLY_H))
_STUD = ((0.0, 0.0, PLY_H / 2), (BLOCK_W, _BLOCK_LEN, BLOCK_H - PLY_H))
BLOCK = kit.Prop(
    "offcut", pos=(float(_block_mid[0]), float(_block_mid[1]), TOP + BLOCK_H / 2), yaw=TAPE_YAW,
    size=(0.14, _BLOCK_LEN, BLOCK_H),
    meshes=(kit.box(*_PLY, kit.Look((0.78, 0.68, 0.52), 0.8)), kit.box(*_STUD, kit.Look(PINE, 0.7))),
    colliders=(kit.Collider(*_PLY), kit.Collider(*_STUD)))
STRIPS = tuple(bk.Strip(TAPE.key(i), *TAPE.box_local(i)) for i in range(TAPE.n))
# The hinges' springs from the first physics step. yamkit starts every mechanism joint with no stiffness (its drives are
# set at the first control step, because most parts start away from their drive targets); the tape starts flat at its
# springs' rest angle, so with no stiffness at first its overhang would sag freely for a moment and the hinges' plastic
# memory would keep that sag as a set (the held end would hang a few centimeters below the offcut's top). The tape's
# actuators get each hinge's own spring and damping at creation instead.
_TAPE_GAINS = {TAPE.link_name(TAPE.strip_of_hinge(j)): (TAPE.stiffness(j), TAPE.damping(j)) for j in range(TAPE.n - 1)}
_plain_mech_object = kit._mech_object


def _mech_object_tape(scene, spec):
    if spec.name != TAPE.name:
        return _plain_mech_object(scene, spec)
    import isaaclab.actuators as _act

    plain_cfg = _act.ImplicitActuatorCfg

    def _cfg(joint_names_expr, stiffness=None, damping=None, **kw):
        name = joint_names_expr[0][: -len("_joint")]
        if name in _TAPE_GAINS:
            stiffness, damping = _TAPE_GAINS[name]
        return plain_cfg(joint_names_expr=joint_names_expr, stiffness=stiffness, damping=damping, **kw)

    _act.ImplicitActuatorCfg = _cfg  # the object builds its articulation config (and actuators) as it is created
    try:
        return _plain_mech_object(scene, spec)
    finally:
        _act.ImplicitActuatorCfg = plain_cfg


kit._mech_object = _mech_object_tape

# -- the knife (knife frame: origin at the blade's root, +x along the blade to its edge, y across, z up) ----------------
# A 4 in. flexible joint knife: a steel blade 102 mm wide at the edge and 76 mm long (0.6 mm steel; 1 mm collider),
# narrowing to 60 mm at the ferrule; a 115 mm plastic handle, 30 x 22 mm; 110 g.
BLADE_LEN, BLADE_W = 0.076, 0.102
BLADE_T = 0.001
HANDLE = ((-0.0675, 0.0, 0.0), (0.115, 0.030, 0.022))
FERRULE = ((-0.005, 0.0, 0.0), (0.010, 0.034, 0.014))
BLADE_Z = -HANDLE[1][2] / 2 + BLADE_T / 2  # the blade's mid-plane: flush with the handle's underside
BLADE_FRONT = ((BLADE_LEN - 0.020, 0.0, BLADE_Z), (0.040, BLADE_W, BLADE_T))  # the wide part, to the edge ...
BLADE_NECK = ((0.018, 0.0, BLADE_Z), (0.036, 0.060, BLADE_T))  # ... and the neck, narrowing toward the ferrule
KNIFE_MASS = 0.110
# Steel on paper wet with compound: it slides freely (the same low friction as the tape on the wet face). The finger
# pads still grip the handle: their rubber material wins any pairing (sheetkit binds it with "max").
KNIFE_FRICTION = (0.3, 0.25)
GRIP_LOCAL = np.array([-0.090, 0.0, 0.0])  # where the hand holds the handle
KNIFE_START = (0.200, -0.230, TOP + HANDLE[1][2] / 2)  # lying flat on the bench, blade toward +y (left)
KNIFE_YAW = math.pi / 2
STEEL = kit.Look((0.74, 0.75, 0.77), 0.3, metallic=0.9)
KNIFE = kit.Prop(
    "knife", pos=KNIFE_START, yaw=KNIFE_YAW, mass=KNIFE_MASS, size=(0.20, BLADE_W, HANDLE[1][2]),
    meshes=(kit.box(*HANDLE, kit.Look((0.15, 0.30, 0.65), 0.5)), kit.box(*FERRULE, STEEL),
            kit.box(BLADE_FRONT[0], (0.040, BLADE_W, 0.0006), STEEL), kit.box(BLADE_NECK[0], (0.036, 0.060, 0.0006), STEEL)),
    colliders=(kit.Collider(*HANDLE), kit.Collider(*FERRULE), kit.Collider(*BLADE_FRONT), kit.Collider(*BLADE_NECK)),
    friction=KNIFE_FRICTION)
KNIFE_PARTS = (gk.Block(*HANDLE), gk.Block(*FERRULE))  # what a hand holds

OBJECTS = [PANEL, STOP, BRACE, BLOCK, KNIFE, dataclasses.replace(TAPE.mechanism(), yaw=TAPE_YAW)]

# -- the compound model and thresholds (each with its reason in the README and task.json) ------------------------------
LAW = bk.Law()  # the angle law: 0.8 mm left at <= 40 deg, 0.1 mm at >= 60 deg; bonds with 0.4-1.6 mm left
BLADE = bk.Blade(edge_x=BLADE_LEN, width=BLADE_W, thick=BLADE_T, z=BLADE_Z)
FACE = bk.Face("panel", origin=(_FACE_LOCAL_X, -PANEL_W / 2, -PANEL_H / 2), u=(0, 0, 1), v=(0, 1, 0),
               size=(PANEL_H, PANEL_W), cell=0.001)
CENTER_TOL = 0.003  # m: the tape's center line within 3 mm of the crack at every station
BEDDED = (0.0004, 0.0016)  # m of compound under the tape at a station (USG and ProForm: "about 1/32 in." = 0.7-0.8 mm)
BEDDED_SHARE = 0.9  # of the stations bedded
SQUEEZE_MIN = 100e-9  # m^3 squeezed out at each edge of the strip over a station (a quarter of a full pass's)
ACROSS_CHECK = 0.018  # m: the depth is also read this far either side of the tape's center line (8 mm in from its edges)
FLAT = 0.0007  # m: every strip over the crack lies with all its underside corners within this of the face (no wrinkle)
ON_BED = 0.0035  # m: a strip whose underside middle is within this of the face is on the bed (the bed is 3 mm deep)
KNIFE_WORKED = 0.5  # share of stations whose strip the held, sliding knife has pressed: no credit at all below it
TORN_CAP = 0.5  # a knife's corner dragged along the tape tears it: the score is capped here
SET_DOWN_Z = TOP + 0.06  # the knife's origin below this: lying on the bench
PANEL_CLEAR = 0.002  # m: every corner of the knife at least this far in front of the face
STILL_STEP = 0.0005  # m per control step (1 cm/s) for the knife and every strip ...
STILL_TURN = math.radians(0.5)  # ... and the knife under 0.5 degree a step
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)  # 20 steps = 1 s
# The bond (a strip bedded in wet compound): a joint from the wall to the strip, switched on when the strip is bedded
# (bedkit). Off the face it pulls back with a spring that reaches BOND_PEEL at BOND_STRETCH of lift, and it lets go
# there (peeled); along the face it resists sliding like a thin layer of wet compound sheared, up to BOND_SHEAR.
BOND_PEEL = 0.05  # N per strip
BOND_STRETCH = 0.0005  # m
BOND_SHEAR_DAMPING = 40.0  # N s/m per strip
BOND_SHEAR = 0.1  # N per strip
# The wet compound's tack (before any knife): paper tape set on the bed by hand stays there, "held in place by a thin
# layer of joint compound" (Fine Homebuilding), though it can still be lifted off and reset. A strip sticks only where
# it touches the mudded band (its underside middle within TACK_GAP of the face, over compound) while a fingertip is in
# contact with it (a pad or tip box within TACK_TOUCH): never from a distance, never off the bed, never without a
# finger on it. It is the same kind of joint as the bond, five to ten times weaker. No published figure for wet
# compound's tack was found; its strength is an assumption, about 20 times a strip's weight (about 0.5 mN; its shear
# cap is about 30 Pa over a strip, below the 200-1000 Pa yield stress of wall (spackling and joint) compounds in
# US 11,267,758 B2): its shear is a capped damper, so it does not hold a load still but slows the tape's sliding to
# under a millimeter a minute; it lets go when the strip is lifted past TACK_STRETCH. It never counts as bedded.
TACK_TOUCH = 0.001  # m: a fingertip's pad or tip box within this of the strip (contact, not nearness)
TACK_GAP = 0.001  # m: the strip's underside middle within this of the face (touching the bed)
TACK_MUD = 0.0005  # m of compound under the strip's middle (on the bed, not on bare face)
TACK_PEEL = 0.01  # N per strip
TACK_STRETCH = 0.002  # m
TACK_SHEAR = 0.01  # N per strip
GRIPPERS = ("left_gripper", "right_gripper")
CONTACT = 0.002  # m: a gripper's own shape (guardkit's boxes fitted to the finger meshes) within 2 mm touches a part
TAPE_PARTS = {s.key: (gk.Block(s.center, s.size),) for s in STRIPS}


def _pose(o):
    return np.asarray(o.pos, float), np.asarray(o.quat_xyzw, float)


FIXED_POSES = {"panel": (PANEL_POS, bk.quat(R_PANEL))}
_KNIFE_CORNERS = np.array([np.asarray(c) + np.asarray(sz) / 2 * np.array([sx, sy, sz_])
                           for c, sz in (HANDLE, FERRULE, BLADE_FRONT, BLADE_NECK)
                           for sx in (-1, 1) for sy in (-1, 1) for sz_ in (-1, 1)])


def in_sim() -> bool:
    """Running inside the simulator (its Kit runtime is loaded), not offline (where isaaclab may be imported alone)."""
    return "carb" in sys.modules


def knife_clear_of_panel(kp, kq) -> bool:
    """Every corner of the knife is at least PANEL_CLEAR in front of the panel's face."""
    pts = _KNIFE_CORNERS @ bk.rotm(kq).T + kp
    return bool(np.min((pts - face_point(0.0, PANEL_YC)) @ NORMAL) >= PANEL_CLEAR)


def knife_held(objects) -> bool:
    """A gripper's own shape touches the knife's handle or ferrule (guardkit's contact check): contact, not nearness."""
    return bool(gk.touching(objects, "knife", KNIFE_PARTS, tol=CONTACT, grippers=GRIPPERS))


# -- every arm link (its collision hulls, posed by the simulator) against the tape and the knife ---------------------
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}  # "right_robot/link5" -> (pos, R) in the env frame at this control step (filled in the sim)
LINK_FAR = 0.02  # m: link_gap computes gaps up to this exactly
STRIP_BOXES = {s.key: ((np.asarray(s.center, float), np.asarray(s.size, float) / 2),) for s in STRIPS}
KNIFE_BOXES = tuple((np.asarray(c, float), np.asarray(sz, float) / 2) for c, sz in (HANDLE, FERRULE, BLADE_FRONT,
                                                                                      BLADE_NECK))


def _box_gap(q, boxes):
    """Distance (m, 0 inside) from part-frame points q to the nearest of the part's boxes ((center, half size), ...)."""
    best = None
    for c, h in boxes:
        g = np.linalg.norm(np.maximum(np.abs(q - c) - h, 0.0), axis=1)
        best = g if best is None else np.minimum(best, g)
    return best


def link_gap(objects, key, obj, boxes) -> float:
    """Smallest gap (m) between arm link `key` ("left_robot/link5") as posed and the boxes of object `obj` (in its
    frame); LINK_FAR when over that."""
    o, R = (np.asarray(v, float) for v in LINK_POSES[key])
    co, Rc = np.asarray(objects[obj].pos, float), bk.rotm(objects[obj].quat_xyzw)
    link = key.split("/")[1]
    _, cen, rad = yg.cloud(link)
    reach = max(float(np.linalg.norm(np.abs(c) + h)) for c, h in boxes)
    if float(np.linalg.norm(o + R @ cen - co)) - rad > reach + LINK_FAR:
        return LINK_FAR  # the whole link far from the object

    def gap(q):
        return _box_gap(q, boxes)

    q = yg.near(link, lambda x: (o + x @ R.T - co) @ Rc, gap, LINK_FAR)
    return min(LINK_FAR, float(gap(q).min())) if len(q) else LINK_FAR


def arm_gap(objects, obj, boxes) -> float:
    """Smallest gap (m) between any link of either arm (link1 to the fingertips) and `obj`'s boxes."""
    return min((link_gap(objects, k, obj, boxes) for k in LINK_POSES), default=LINK_FAR)


def _arm_on_strip(objects, key) -> bool:
    """A gripper's own shape, or any arm link's hulls, within CONTACT of tape strip `key`."""
    return bool(gk.touching(objects, key, TAPE_PARTS[key], tol=CONTACT, grippers=GRIPPERS)
                or arm_gap(objects, key, STRIP_BOXES[key]) <= CONTACT)


def hands_on_tape(objects) -> bool:
    """Any part of either arm (gripper shapes and every link's hulls) touches any strip of the tape."""
    return any(_arm_on_strip(objects, s.key) for s in STRIPS)


def fingered_strips(objects, strips) -> list:
    """Which of `strips` any part of either arm touches now."""
    return [i for i in strips if _arm_on_strip(objects, STRIPS[i].key)]


def knife_released(objects) -> bool:
    """No hand holds the knife and every arm link is at least CONTACT from all of its parts (no wrist pinning it)."""
    return not knife_held(objects) and arm_gap(objects, "knife", KNIFE_BOXES) > CONTACT


# -- the bonds in the simulator: one joint per strip, authored switched off, switched on and off by the task -----------
_BONDS = {"paths": [], "on": set(), "tack_paths": [], "tack_on": set()}


def _author_bonds(prim) -> None:
    """One PhysX D6 joint per strip from the world to the strip's middle, outside the articulation, switched off.
    Frame 0 lies half the strip's box out from the face (where a lying strip's middle is, whichever side of the tape
    is on the wall: the demo's fold lays it bench-top side down) with its x into the wall: x is a spring that reaches
    BOND_PEEL at BOND_STRETCH and holds no more than that (a peel force); y and z (along the face) are dampers capped
    at BOND_SHEAR; rotations are free (a strip on a thin wet layer hinges as the tape bends). One environment
    (object_jitter=0, one env).
    """
    from pxr import Gf, UsdGeom, UsdPhysics

    stage = prim.GetStage()
    env_M = UsdGeom.XformCache().GetLocalToWorldTransform(prim.GetParent())
    rot = env_M.ExtractRotationQuat() * Gf.Quatd(*bk.quat(R_PANEL)[[3, 0, 1, 2]].tolist())
    anchor = env_M.Transform(Gf.Vec3d(*(float(v) for v in face_point(0.2, PANEL_YC) + TAPE_COLLIDER_T / 2 * NORMAL)))
    m = float(min(TAPE.mass(i) for i in range(TAPE.n)))
    root = str(prim.GetPath())
    for kind, peel, stretch, shear in (("bond", BOND_PEEL, BOND_STRETCH, BOND_SHEAR),
                                       ("tack", TACK_PEEL, TACK_STRETCH, TACK_SHEAR)):
        _BONDS["paths" if kind == "bond" else "tack_paths"] = _author_kind(stage, root, kind, anchor, rot, m, peel,
                                                                           stretch, shear)


def _author_kind(stage, root, kind, anchor, rot, m, peel, stretch, shear) -> list:
    from pxr import Gf, UsdPhysics

    k = peel / stretch
    paths = []
    for i in range(TAPE.n):
        ln = "base" if i == TAPE.root else TAPE.link_name(i)
        c, s = TAPE.box_local(i)
        jt = UsdPhysics.Joint.Define(stage, f"{root}/{kind}_{i}")
        jt.CreateBody1Rel().SetTargets([stage.GetPrimAtPath(f"{root}/{ln}").GetPath()])
        jt.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in anchor)))
        jt.CreateLocalRot0Attr(Gf.Quatf(float(rot.GetReal()), *(float(v) for v in rot.GetImaginary())))
        jt.CreateLocalPos1Attr(Gf.Vec3f(float(c[0]), float(c[1]), float(c[2])))
        jt.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        jt.CreateExcludeFromArticulationAttr(True)
        jt.CreateCollisionEnabledAttr(False)
        jt.CreateJointEnabledAttr(False)
        jp = jt.GetPrim()
        for ax, (stiff, damp, cap) in (("transX", (k, 2 * math.sqrt(k * m), peel)),
                                       ("transY", (0.0, BOND_SHEAR_DAMPING, shear)),
                                       ("transZ", (0.0, BOND_SHEAR_DAMPING, shear))):
            dr = UsdPhysics.DriveAPI.Apply(jp, ax)
            dr.CreateTypeAttr("force")
            dr.CreateStiffnessAttr(float(stiff))
            dr.CreateDampingAttr(float(damp))
            dr.CreateTargetPositionAttr(0.0)
            dr.CreateTargetVelocityAttr(0.0)
            dr.CreateMaxForceAttr(float(cap))
        paths.append(str(jp.GetPath()))
    return paths


_plain_author_ties = sk._author_ties


def _author_ties_and_bonds(prim, ties):
    n = _plain_author_ties(prim, ties)
    if prim.GetName() == TAPE.name:
        _author_bonds(prim)
    return n


sk._author_ties = _author_ties_and_bonds


def _set_bond(i: int, on: bool, kind: str = "bond") -> None:
    """Switch strip i's bond (or tack) joint on or off in the running simulator (USD physics:jointEnabled); offline a
    no-op."""
    key, pk = ("on", "paths") if kind == "bond" else ("tack_on", "tack_paths")
    if on:
        _BONDS[key].add(i)
    else:
        _BONDS[key].discard(i)
    if not in_sim():
        return  # offline: no joints to switch
    if not _BONDS[pk]:
        raise RuntimeError(f"tape: no {kind} joints were authored, so no {kind} could hold the tape")
    try:
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        UsdPhysics.Joint(stage.GetPrimAtPath(_BONDS[pk][i])).GetJointEnabledAttr().Set(bool(on))
    except Exception as e:  # noqa: BLE001 (the score's bond and the physics must agree)
        raise RuntimeError(f"tape: switching {kind} {i} {'on' if on else 'off'} failed") from e


# -- one trial ------------------------------------------------------------------------------------------------------------
class Ledger:
    """One trial's compound, bonds and history, advanced once per control step (success() and score() share it)."""

    def __init__(self, start=start_bed):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.bed = bk.Bed(FACE, STRIPS, BLADE, LAW, start)
        self.last = None
        self.tape_last = None
        self.good = 0
        self.peels = 0
        self.view = {}
        self.fingered = np.zeros(TAPE.n, bool)  # latched: a gripper touched this strip over the crack once bedded
        self.tacked = np.zeros(TAPE.n, bool)  # stuck to the wet bed by a finger's press (not bedded)
        self.tacks = 0  # tack events (telemetry)
        for i in list(_BONDS["on"]):
            _set_bond(i, False)
        for i in list(_BONDS["tack_on"]):
            _set_bond(i, False, "tack")

    def stations(self, frames) -> dict:
        """Per station: covered (a strip lying on the face holds the crack point), the tape's center line's offset (m),
        compound under the tape there (m), squeezed out at both edges, bonded, bedded."""
        fr = bk.face_frame(FACE, FIXED_POSES["panel"])
        lying = np.array([abs(self.bed.lift(fr, frames, i)) <= ON_BED for i in range(TAPE.n)])
        out = {k: [] for k in ("covered", "offset", "depth", "across", "squeezed", "bonded", "bedded", "strip")}
        for c in STATIONS:
            hit = self.bed.locate(frames, c, lying)
            cov = hit is not None
            i = hit[0] if cov else -1
            depth = self.bed.depth_under(fr, frames, i, c) if cov else 0.0
            across = []  # the depth a little in from each edge, across the tape from the crack point
            if cov:
                R = frames[i][1]
                for sgn in (-1.0, 1.0):
                    p = c + R @ np.array([sgn * ACROSS_CHECK - hit[1], 0.0, 0.0])  # from the tape's center line
                    h2 = self.bed.locate(frames, p, lying)
                    across.append(self.bed.depth_under(fr, frames, h2[0], p) if h2 is not None else 0.0)
            sq = bool(cov and self.bed.squeezed[i].min() >= SQUEEZE_MIN)
            bnd = bool(cov and self.bed.bonded[i] and not self.fingered[i])
            out["covered"].append(cov)
            out["offset"].append(abs(hit[1]) if cov else math.inf)
            out["depth"].append(depth)
            out["across"].append(tuple(across))
            out["squeezed"].append(sq)
            out["bonded"].append(bnd)
            out["bedded"].append(bool(cov and bnd and sq and all(BEDDED[0] <= v <= BEDDED[1] for v in [depth, *across])))
            out["strip"].append(i)
        out["lying"] = lying
        return out

    def wrinkled(self, frames, st) -> bool:
        """A wrinkle: a strip over the crack (between the first and last strip at a station) with any underside
        corner more than FLAT off the face (a ridge, a bubble, a fold lying on the tape)."""
        idx = [i for i in st["strip"] if i >= 0]
        if not idx:
            return True
        fr = bk.face_frame(FACE, FIXED_POSES["panel"])
        o, _, _, n = fr
        for i in range(min(idx), max(idx) + 1):
            if np.max((bk.underside(STRIPS[i], frames[i], n) - o) @ n) > FLAT:
                return True
        return False

    def tier(self, st) -> float:
        """0.25: the tape lies on the face over the crack at every station and the knife has pressed it at KNIFE_WORKED
        of them; 0.5: and centered within CENTER_TOL at every
        station; 0.75: and bedded at BEDDED_SHARE of them. Torn tape caps it at TORN_CAP."""
        if not all(st["covered"]):
            return 0.0
        if np.mean([bool(self.bed.pressed[i]) for i in st["strip"]]) < KNIFE_WORKED - 1e-9:
            return 0.0  # laid or set by hand only: no credit until the knife has been drawn over it
        if max(st["offset"]) > CENTER_TOL:
            return 0.25
        if np.mean(st["bedded"]) < BEDDED_SHARE - 1e-9:
            return 0.5
        return min(0.75, TORN_CAP) if self.bed.torn else 0.75


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
    if not LINK_POSES and in_sim():
        raise RuntimeError("tape: no arm link poses were read for this step")
    frames = sk.frames(objects, TAPE)
    held = knife_held(objects)
    kp, kq = _pose(objects["knife"])
    fr = bk.face_frame(FACE, FIXED_POSES["panel"])
    for i in _L.bed.step(kp, kq, held, FIXED_POSES["panel"], frames):
        _L.bed.set_bonded(i, True, fr, frames)
        _set_bond(i, True)
    for i in _L.bed.release(fr, frames, BOND_STRETCH):
        _L.bed.set_bonded(i, False)
        _set_bond(i, False)
        _L.peels += 1
    _tack(objects, fr, frames)
    st = _L.stations(frames)
    over = sorted({i for i in st["strip"] if i >= 0 and _L.bed.bonded[i]})
    if over:  # a finger on bedded tape over the crack: that stretch is not left as the knife bedded it
        for i in fingered_strips(objects, over):
            _L.fingered[i] = True
        st = _L.stations(frames)
    t = _L.tier(st)
    r = bk.rotm(kq)
    still = False
    if _L.last is not None:
        dp = float(np.linalg.norm(kp - _L.last[0]))
        dr = math.acos(max(-1.0, min(1.0, (float(np.trace(_L.last[1].T @ r)) - 1) / 2)))
        still = dp <= STILL_STEP and dr <= STILL_TURN
    _L.last = (kp, r)
    tp = np.array([f[0] for f in frames])
    tape_still = _L.tape_last is not None and float(np.max(np.linalg.norm(tp - _L.tape_last, axis=1))) <= STILL_STEP
    _L.tape_last = tp
    ok = False
    if t == 0.75 and still and tape_still and not held:
        down = float(kp[2]) <= SET_DOWN_Z and knife_clear_of_panel(kp, kq)
        ok = down and not _L.wrinkled(frames, st) and knife_released(objects) and not hands_on_tape(objects)
    _L.good = _L.good + 1 if ok else 0
    _L.view = {"tier": t, "st": st}
    _BEADS.draw(_L.bed, frames)


def _tack(objects, fr, frames) -> None:
    """The wet bed's tack. A strip that is not bonded sticks when it touches the mudded band while a finger presses it;
    it lets go when lifted past TACK_STRETCH; a bond replaces it. Scoring never reads it."""
    for i in np.flatnonzero(_L.tacked):
        i = int(i)
        if _L.bed.bonded[i] or _L.bed.lift(fr, frames, i) > TACK_STRETCH:
            _L.tacked[i] = False
            _set_bond(i, False, "tack")
    cand = []
    for i in range(TAPE.n):
        if _L.tacked[i] or _L.bed.bonded[i]:
            continue
        lift = _L.bed.lift(fr, frames, i)
        if -_L.bed.law.below <= lift <= TACK_GAP and _L.bed.depth_under(fr, frames, i) >= TACK_MUD:
            cand.append(i)
    pressed = [i for i in cand if gk.touching(objects, STRIPS[i].key, TAPE_PARTS[STRIPS[i].key], tol=TACK_TOUCH,
                                              grippers=GRIPPERS, boxes=gk.FINGERTIPS)]
    for i in pressed:
        _L.tacked[i] = True
        _L.tacks += 1
        _set_bond(i, True, "tack")


def success(objects):
    """The tape centered and bedded over the crack, no wrinkle, the knife set down, no hand on the tape, all still:
    this step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """0.25: the tape lies on the face over the crack end to end (at every 1 cm station) and the held knife has been
    drawn over it at half of them or more (tape set by hand alone scores 0); 0.5: and its center line is
    within 3 mm of the crack at every station; 0.75: and it is bedded at 90% of the stations (0.4-1.6 mm of compound
    under it, compound squeezed out at both edges, bonded); 1.0: and no wrinkle, the knife set down, no hand on the
    tape, and everything still for 1 s. Doing nothing scores 0."""
    _update(objects)
    s = _L.view.get("tier", 0.0)
    if s == 0.75 and _L.good >= HOLD_STEPS:
        s = 1.0
    return s


# -- the compound squeezed out, as the cameras see it --------------------------------------------------------------------
class _Beads:
    """Compound that changed from the starting bed (squeezed-out beads, struck-off bed), drawn as thin gray tiles just in
    front of the face with Isaac Lab's VisualizationMarkers. Only a picture: off with one warning on any error."""

    def __init__(self):
        self.markers, self.off, self.shown, self.origin = None, False, None, None

    def draw(self, bed, frames):
        if self.off:
            return
        fr = bk.face_frame(FACE, FIXED_POSES["panel"])
        blocks = bed.blocks(fr, frames, k=3)[:4000]
        sig = hash(tuple((round(a, 4), round(b, 4)) for a, b, _, _ in blocks))
        if sig == self.shown:
            return
        try:
            import isaaclab.sim as sim_utils
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

            if self.origin is None:
                import omni.usd
                from pxr import UsdGeom

                stage, cache = omni.usd.get_context().get_stage(), UsdGeom.XformCache()
                self.origin = np.zeros(3)
                for prim in stage.Traverse():
                    if prim.GetName() == "panel":
                        w = np.array(cache.GetLocalToWorldTransform(prim).ExtractTranslation(), float)
                        self.origin = w - PANEL_POS
                        break
            if self.markers is None:
                proto = sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=tuple(kit._linear(c) for c in MUD_RGB), roughness=0.5))
                self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/Visuals/compound",
                                                                            markers={"mud": proto}))
            self.shown = sig
            if not blocks:
                self.markers.set_visibility(False)
                return
            o, eu, ev, n = fr
            q = bk.quat(np.stack([eu, ev, n], axis=1))
            tr = [o + a * eu + b * ev + n * 0.0008 + self.origin for a, b, _, _ in blocks]
            self.markers.set_visibility(True)
            self.markers.visualize(translations=np.array(tr, np.float32),
                                   orientations=np.tile(q, (len(blocks), 1)).astype(np.float32),
                                   scales=np.array([(su * 0.95, sv * 0.95, 0.0012) for _, _, su, sv in blocks], np.float32))
        except ImportError:
            self.off = True  # no simulator: offline tests
        except Exception as e:  # noqa: BLE001 (cosmetic only)
            self.off = True
            print(f"[tape] compound tiles off ({e!r}); scoring is unaffected", flush=True)


_BEADS = _Beads()


def _read_links(base) -> None:
    """Every arm link's pose (env frame) at this control step, into LINK_POSES. A missing arm or link, or bond joints
    that were never authored, stops the trial: the contact checks and the bonds would otherwise fail silently."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"tape: no {robot} in the scene for the contact checks") from e
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"tape: {robot} has no link(s) {missing} for the contact checks")
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, bk.rotm(Q[i]))
    if not _READ_ONCE:
        if len(_BONDS["paths"]) != TAPE.n or len(_BONDS["tack_paths"]) != TAPE.n:
            raise RuntimeError(f"tape: {len(_BONDS['paths'])} bond and {len(_BONDS['tack_paths'])} tack joints "
                               f"authored, not {TAPE.n} of each")
        _READ_ONCE.append(1)
        print(f"tape: contact checks on {len(LINK_POSES)} arm links; {TAPE.n} bond and {TAPE.n} tack joints "
              f"authored (switched off)", file=sys.stderr, flush=True)


_READ_ONCE: list = []
kit.STATE_HOOKS.append(_read_links)
if in_sim():  # in the simulator: read and check the hulls as the task loads, not in a trial's step
    yg.warm(ARM_LINKS)

# -- scripted demo: open loop from the fixed layout ---------------------------------------------------------------------
# Left hand (the tape): it pinches the tape's top end where it overhangs the offcut, fingers level and pointing along
# the tape, closing through its thickness; lifts it; turns it to point at the face in front of the crack's top end; and
# moves in, so the tape's free part meets the bed and lays itself down the crack's line while the held end stays folded
# out between the fingers; it holds there while the knife beds the tape below, then lets go.
# Right hand (the knife): it takes the knife by the back of the handle from above, turns it so the blade points up the
# tape into the face at 35 degrees with the edge square across the tape, comes in below the held end, sets the edge on
# the tape just above the crack's top end, draws it down the tape past the bottom end at 1.5 cm/s, lifts off, and lays
# the knife back on the bench.


def _r_from(a, c) -> np.ndarray:
    """A grip rotation whose approach (tool z) is `a` and whose fingers close along `c` (tool x), made square to a."""
    a = np.asarray(a, float) / np.linalg.norm(a)
    c = np.asarray(c, float) - float(np.dot(c, a)) * a
    c /= np.linalg.norm(c)
    return np.column_stack([c, np.cross(a, c), a])


def _ypr(R) -> tuple:
    """(yaw, roll, pitch) with yamkit.Tilted's rotation ry(pitch) rx(roll) grip_rotation(yaw) equal to R."""
    M = np.asarray(R, float) @ np.asarray(_demo.grip_rotation(0.0), float).T
    roll = math.asin(max(-1.0, min(1.0, -M[1, 2])))
    return math.atan2(M[1, 0], M[1, 1]), roll, math.atan2(M[0, 2], M[2, 2])


def _wp(sec, pos, finger, label, R) -> kit.Tilted:
    yaw, roll, pitch = _ypr(R)
    return kit.Tilted(float(sec), tuple(float(v) for v in pos), finger, label, yaw=yaw, roll=roll, pitch=pitch)


# The hold: the fingers point into the face (approach -NORMAL) and close across the tape's thickness (along DOWN), on
# its top end, which stands out from the face between them; below the fingertips the tape bends down onto the face and
# runs along the crack's line. The same hold on the bench: fingers level, pointing along the tape, closing vertically.
PIN_SIDE = math.radians(0.0)  # the fingers turned this far toward the tape's left (+) or right from square to the face
R_PIN = _r_from(math.cos(PIN_SIDE) * -NORMAL - math.sin(PIN_SIDE) * ACROSS, DOWN)  # closing through the tape's top end
_A_PIN = np.column_stack([R_PIN[:, 0], -NORMAL, np.cross(R_PIN[:, 0], -NORMAL)])  # the tape runs from its end to the face
_Z = np.array([0.0, 0.0, 1.0])
_Y = np.array([0.0, 1.0, 0.0])
R_PICK = np.column_stack([-_Z, TAPE_DIR, np.cross(-_Z, TAPE_DIR)]) @ _A_PIN.T @ R_PIN  # the same hold, tape flat
TIPS_OFF_FACE = 0.008  # m: the fingertips stop this far out from the face
HOLD_FOLD = CRACK_TOP + HOLD_ON_FACE * CRACK_UP  # where the held tape leaves the face
TAPE_TOP_END = HOLD_FOLD + HOLD_OUT * NORMAL
PIN_TCP = HOLD_FOLD + TIPS_OFF_FACE * NORMAL - PAD_TIP_PAST_TCP * R_PIN[:, 2]
_G_END = R_PIN.T @ (TAPE_TOP_END - PIN_TCP)  # the tape's top end in the hand's frame (it keeps its place in the hand)
PICK_TCP = np.array([*TAPE_END, TOP + BLOCK_H + TAPE_COLLIDER_T / 2]) - R_PICK @ _G_END

KNIFE_DEG = 35.0  # the blade's angle to the face while it beds the tape: leaves 0.8 mm (bedkit's law)
R_KNIFE_REST = np.array([[math.cos(KNIFE_YAW), -math.sin(KNIFE_YAW), 0.0],
                         [math.sin(KNIFE_YAW), math.cos(KNIFE_YAW), 0.0], [0.0, 0.0, 1.0]])


def _pass_rotations(deg) -> tuple:
    """(knife, hand) rotations for a pass with the blade at `deg` to the face, pointing up the tape into the face, its
    edge square across the tape."""
    th = math.radians(deg)
    b = math.cos(th) * CRACK_UP - math.sin(th) * NORMAL  # along the blade toward its edge
    R_knife = np.column_stack([b, -ACROSS, np.cross(b, -ACROSS)])
    return R_knife, R_knife @ R_KNIFE_REST.T @ np.asarray(_demo.grip_rotation(KNIFE_YAW), float)


R_KNIFE_PASS = _pass_rotations(KNIFE_DEG)[0]
EDGE_LOCAL = np.array([BLADE_LEN, 0.0, BLADE_Z])
STROKE = (0.020, -0.021)  # m along the crack from its top end / bottom end: where the stroke starts and ends. The
# knife sags in the pinch by about a centimeter as it is pushed, so its edge lands about 1 cm short of each aim: the
# stroke is aimed 2 cm above the top end and 2.1 cm past the bottom end (further, the knife's lower corner nears the
# bench), so it still works both end stations
PRESS = 0.012  # m: the edge is aimed this far below the tape's top (the tape and the face stop it); the knife sags
# in the pinch by up to about a centimeter while it is pushed along
SPEED = 0.015  # m/s down the tape
OFF, BACK = 0.02, 0.06  # m out from the face before and after the stroke
APPROACH_BELOW = 0.04  # m down the tape from the stroke's start where the knife comes in
LIFT = 0.10  # m above the bench when carrying the knife
TURN_X = 0.23  # m: where the knife is turned toward the panel


def knife_tcp(edge, R_knife) -> tuple:
    """The tool point that puts the middle of the knife's edge at `edge` with the knife at rotation R_knife."""
    return tuple(float(v) for v in np.asarray(edge, float) - R_knife @ (EDGE_LOCAL - GRIP_LOCAL))


STAGE_OUT = 0.14  # m: the hand starts its move to the face this far out, the tape hanging toward the face below it
LAY_TIP = math.radians(20.0)  # ... with the fingers tipped this far down the crack's line, so the tape meets the bed low
# (pointing square at the face from further out, the stiff tape meets the face high and goes over the panel's top;
# tipped more this close to the face, the left wrist is past its reach)
CARRY_VIA = (0.08, 0.10)  # m out from the stage and to the left: the tape is turned toward the face out here, then
# brought in (turned on the way across from the bench, its free end would sweep into the panel's left edge and swivel
# out of the pinch)
_k = np.array([[0.0, -ACROSS[2], ACROSS[1]], [ACROSS[2], 0.0, -ACROSS[0]], [-ACROSS[1], ACROSS[0], 0.0]])
R_LAY = (np.eye(3) - math.sin(LAY_TIP) * _k + (1 - math.cos(LAY_TIP)) * _k @ _k) @ R_PIN  # turned -LAY_TIP about ACROSS


def _left() -> list:
    a = R_PICK[:, 2]
    stage = PIN_TCP + STAGE_OUT * NORMAL
    return [_wp(8.5, PICK_TCP - 0.03 * a + 0.012 * _Z, FINGER_OPEN, "behind the tape's top end", R_PICK),
            _wp(1.2, PICK_TCP - 0.03 * a, FINGER_OPEN, "fingers level with the overhang", R_PICK),
            _wp(1.5, PICK_TCP, FINGER_OPEN, "pads either side of the overhang", R_PICK),
            _wp(1.3, PICK_TCP, FINGER_CLOSED, "pinch the top end", R_PICK),
            _wp(1.2, PICK_TCP + 0.04 * _Z, FINGER_CLOSED, "lift the tape off the offcut", R_PICK),
            _wp(4.0, stage + CARRY_VIA[0] * NORMAL + CARRY_VIA[1] * _Y, FINGER_CLOSED,
                "turn it toward the face, fingers tipped down, out in front of the panel", R_LAY),
            _wp(2.5, stage, FINGER_CLOSED, "in front of the crack, the tape's free end toward the bed", R_LAY),
            _wp(2.5, PIN_TCP + 0.06 * NORMAL, FINGER_CLOSED, "toward the face: the tape's free end meets the bed low",
                R_LAY),
            _wp(2.5, PIN_TCP + 0.025 * NORMAL, FINGER_CLOSED, "tip the fingers up: the tape lays itself down the line",
                R_PIN),
            _wp(1.5, PIN_TCP, FINGER_CLOSED, "in to the face", R_PIN),
            _wp(1.0, PIN_TCP, FINGER_CLOSED, "hold the top end at the top of the bed", R_PIN)]


# Setting the tape (before the knife): closed fingertips pointing into the face, side by side across the tape, press it
# onto the bed in short dabs down its length ("lightly press it into place", Fine Homebuilding): in, out, along, in.
# They never slide along it while touching (run down the tape, the rubber pads would drag it, pull it taut off the face
# from the held end and peel the tacked strips). The right hand sets it from SET_START below the crack's top end to past
# the bottom end while the left hand keeps the top end (so the tape is pulled tight as it is pressed; the right starts
# that low so the arms stay clear of each other); then the left lets go and sets the top part the same way, from just
# below its fold down past where the right began, and goes home before the knife comes in. The wet bed's tack keeps the tape there.
R_SET = _r_from(-NORMAL, ACROSS)
SET_TIP = math.radians(30.0)  # the right hand's fingers tipped this far up the crack's line (its forearm lower, clear
# of the left arm: pointing square into the face its forearm passed within the left wrist's hulls)
SET_CORNER = 0.0035  # m: tipped, the fingertips' lower corners reach this much nearer the face than their middle


def _r_set_tipped(tip) -> np.ndarray:
    return (np.eye(3) + math.sin(tip) * _k + (1 - math.cos(tip)) * _k @ _k) @ R_SET


SET_EASE = 0.076  # m down the tape over which the right's fingers turn from SET_TIP to square to the face
SET_START = 0.060  # m below the crack's top end: where the right fingertips first press (clear of the left arm)
SET_OVERLAP = 0.015  # m: the left's top pass runs this far past where the right began
SET_END = 0.012  # m past the crack's bottom end
SET_PRESS = 0.002  # m: the fingertips are aimed this far below the tape's top (a light press; the face stops them)
SET_DAB = 0.015  # m down the tape between dabs (a closed fingertip presses about 2 cm of it)
SET_OUT = 0.008  # m: between dabs the fingertips back this far off the face, then move along
DAB_S = (0.5, 0.6, 0.6, 0.3)  # s: back off, move along, press in, hold
# The left's dabs on the top part, where the tape is still loose: a lighter press and fewer, wider dabs (each firm dab
# there slides the whole loose tape about half a millimeter to the robot's right).
LEFT_PRESS = 0.001  # m
LEFT_DAB = 0.022  # m: at most this far apart, evenly spaced
LEFT_DAB_S = (0.5, 0.6, 0.8, 0.3)  # s: back off, move along, press in, hold (a slower back-off slides it more)


def _dab_marks(run, step=SET_DAB, even=False) -> list:
    """Distances down the tape of the dabs after the first (at 0): every `step`, the last at `run` (even: evenly spaced
    at most `step` apart)."""
    if even:
        n = max(1, math.ceil(run / step - 1e-6))
        return [float(run * i / n) for i in range(1, n + 1)]
    out = [float(d) for d in np.arange(step, run - 1e-3, step)]
    return out + [float(run)]


def _dab_plan(at, run, label, step=SET_DAB, secs=DAB_S, even=False) -> list:
    """(sec, pos, R, label) of the dabs after the first one: `at(d, out)` gives the pose `d` down the tape from the first,
    pressing (out None) or `out` m off the face."""
    plan, prev = [], 0.0
    for d in _dab_marks(run, step, even):
        plan += [(secs[0], *at(prev, SET_OUT), "back off the tape"),
                 (secs[1], *at(d, SET_OUT), "along the tape, off it"),
                 (secs[2], *at(d, None), label),
                 (secs[3], *at(d, None), "hold the press")]
        prev = d
    return plan


def _set_tcp(p, d, R=R_SET, u=None, e=0.0) -> tuple:
    """The tool point that puts the closed fingertips `d` m out from the face at face point p, then `e` m further
    along u (u . n = 1; default the face normal)."""
    u = NORMAL if u is None else np.asarray(u, float)
    return tuple(float(v) for v in np.asarray(p, float) + d * NORMAL + e * u - PAD_TIP_PAST_TCP * R[:, 2])


# The arm is springy: its joints are PD springs (yam_arena: 80 N m/rad shoulder, 10 N m/rad wrist), so a fingertip
# aimed a little into the face gives way along C n (C = J K^-1 J^T at the fingertip), not straight back along n. For the
# left hand on the dab line C n leans 0.6-0.9 mm across and 0.6-0.8 mm up the tape per mm of press: aimed straight
# into the face, each dab slid the fingertip sideways on the face as the press built up and the rubber carried the
# loose tape with it (about 1 mm per dab; the right's dabs push the other way, but the left still holds the top end
# then). So the left's dabs come in and press along u = C n / (n . C n): the arm's give is
# then straight into the face and the fingertip stays where it landed.
K_ARM = np.diag([SHOULDER["stiffness"]] * 3 + [WRIST["stiffness"]] * 3)  # N m/rad, joints 1-6
_Q_SET_SEED = np.radians([-48.0, 111.0, 100.0, -15.0, -44.0, -21.0])  # the elbow branch the left's set pass rides


def tip_compliance(q, base) -> np.ndarray:
    """3x3 compliance (m/N) of the closed fingertips' middle for arm joints q: C = J K^-1 J^T."""
    def tip(qq):
        p, R = _ik.tcp_pose(qq, base)
        return p + PAD_TIP_PAST_TCP * R[:, 2]
    J = np.zeros((3, 6))
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = 1e-6
        J[:, i] = (tip(q + dq) - tip(q - dq)) / 2e-6
    return J @ np.linalg.inv(K_ARM) @ J.T


def press_dir(p, R, base=_station.BASES["left"], seed=_Q_SET_SEED) -> np.ndarray:
    """u = C n / (n . C n) at the left's fingertips pressing at face point p with rotation R (u . n = 1)."""
    q, err, _ = _ik.solve(_set_tcp(p, TAPE_COLLIDER_T, R), _ik.matrix_to_quat_xyzw(R), seed, base_pos=base, iters=200)
    if err > 0.002:
        return NORMAL.copy()
    c = tip_compliance(q, base) @ NORMAL
    return c / float(c @ NORMAL)


def _left_set() -> list:
    s0 = HOLD_FOLD - 0.008 * CRACK_UP
    s1 = CRACK_TOP - (SET_START + SET_OVERLAP) * CRACK_UP
    on = TAPE_COLLIDER_T - LEFT_PRESS
    run = float(np.linalg.norm(s1 - s0))
    u0, u1 = press_dir(s0, R_SET), press_dir(s1, R_SET)

    def at(d, out):
        """The left's dab pose d down the tape from s0: pressing (out None) or `out` m off the tape's top, along u."""
        p, f = s0 + d * DOWN, d / run
        u = (1 - f) * u0 + f * u1
        u /= float(u @ NORMAL)
        return _set_tcp(p, TAPE_COLLIDER_T, R_SET, u, (on if out is None else TAPE_COLLIDER_T + out)
                        - TAPE_COLLIDER_T), R_SET

    dabs = _dab_plan(at, run, "dab the top of the tape onto the bed", LEFT_DAB, LEFT_DAB_S, even=True)
    return ([_wp(1.0, PIN_TCP, FINGER_OPEN, "let go of the top end", R_PIN),
            _wp(1.2, PIN_TCP + 0.05 * NORMAL, FINGER_OPEN, "back off the face", R_PIN),
             _wp(2.0, _set_tcp(s0, 0.05), FINGER_CLOSED, "fingers closed, turned across the tape", R_SET),
             _wp(1.0, at(0.0, SET_OUT)[0], FINGER_CLOSED, "in front of the top of the tape", R_SET),
             _wp(0.6, at(0.0, None)[0], FINGER_CLOSED, "press the top of the tape onto the bed", R_SET),
             _wp(0.5, at(0.0, None)[0], FINGER_CLOSED, "settle", R_SET)]
            + [_wp(sec, pos, FINGER_CLOSED, label, R) for sec, pos, R, label in dabs]
            + [_wp(LEFT_DAB_S[0], at(run, SET_OUT)[0], FINGER_CLOSED, "back off the tape", R_SET),
               _wp(1.0, _set_tcp(s1, 0.06), FINGER_CLOSED, "off the tape", R_SET), kit.Home(4.0)])


def _right_set() -> list:
    """The right's set pass: tipped SET_TIP at the top (clear of the left arm), easing toward square to the face over
    SET_EASE down the tape (tipped low on the face the pose is out of reach; square near the top, its forearm meets the
    left wrist)."""
    s0 = CRACK_TOP - SET_START * CRACK_UP
    s1 = CRACK_BOT - SET_END * CRACK_UP
    on = TAPE_COLLIDER_T - SET_PRESS

    def at(p, tip, d=None):
        R = _r_set_tipped(tip)
        return _set_tcp(p, on + SET_CORNER * math.sin(tip) / math.sin(SET_TIP) if d is None else d, R), R

    plan = [(3.0, *at(s0, SET_TIP, 0.08), "fingers closed, out in front of the tape below the held end"),
            (2.0, *at(s0, SET_TIP, 0.06), "in front of the tape"),
            (2.0, *at(s0, SET_TIP), "press the tape onto the bed"),
            (0.5, *at(s0, SET_TIP), "settle")]
    run = float(np.linalg.norm(s1 - s0))
    tip_at = lambda d: SET_TIP * max(0.0, 1.0 - d / SET_EASE)  # noqa: E731
    plan += _dab_plan(lambda d, out: at(s0 + d * DOWN, tip_at(d), out), run, "dab the tape onto the bed")
    plan.append((1.0, *at(s1, tip_at(run), 0.06), "off the tape"))
    return [_wp(sec, pos, FINGER_CLOSED, label, R) for sec, pos, R, label in plan]


def _right(deg=KNIFE_DEG, keep=False, sets=True, start=STROKE[0]) -> list:
    """The right's plan: the set pass (`sets`), then the knife taken, drawn down the tape at `deg` to the face from
    `start` m above the crack's top end and laid back on the bench (`keep`: kept in hand with its edge on the wall at
    the stroke's end instead)."""
    R_knife, R_hand = _pass_rotations(deg)
    g0 = np.asarray(KNIFE_START) + R_KNIFE_REST @ GRIP_LOCAL
    gx, gy, gz = (float(v) for v in g0)
    plan = (_right_set() if sets else []) + [
            Waypoint(3.0, (gx, gy, gz + 0.08), FINGER_OPEN, "above the knife's handle", KNIFE_YAW),
            Waypoint(1.5, (gx, gy, gz + 0.003), FINGER_OPEN, "around the handle", KNIFE_YAW),
            Waypoint(1.3, (gx, gy, gz + 0.003), FINGER_CLOSED, "grip", KNIFE_YAW),
            Waypoint(1.2, (gx, gy, TOP + LIFT), FINGER_CLOSED, "lift", KNIFE_YAW)]
    e0 = CRACK_TOP + start * CRACK_UP
    e1 = CRACK_BOT + STROKE[1] * CRACK_UP
    on = TAPE_COLLIDER_T - PRESS
    below = e0 - APPROACH_BELOW * CRACK_UP  # the knife comes in below the held end, then slides up onto the start
    plan += [_wp(3.0, knife_tcp(below + BACK * NORMAL, R_knife), FINGER_CLOSED, "square the blade to the tape", R_hand),
             _wp(1.2, knife_tcp(below + OFF * NORMAL, R_knife), FINGER_CLOSED, "in close, below the held end", R_hand),
             _wp(2.0, knife_tcp(e0 + OFF * NORMAL, R_knife), FINGER_CLOSED, "up to the start of the stroke", R_hand),
             _wp(1.0, knife_tcp(e0 + on * NORMAL, R_knife), FINGER_CLOSED, "edge on the tape below the held end",
                 R_hand),
             _wp(1.0, knife_tcp(e0 + on * NORMAL, R_knife), FINGER_CLOSED, "settle the edge on the tape", R_hand),
             _wp(float(np.linalg.norm(e1 - e0)) / SPEED, knife_tcp(e1 + on * NORMAL, R_knife), FINGER_CLOSED,
                 "draw the knife down the tape", R_hand)]
    if keep:
        return plan + [_wp(5.0, knife_tcp(e1 + on * NORMAL, R_knife), FINGER_CLOSED, "keep the knife on the wall",
                           R_hand)]
    plan += [_wp(0.8, knife_tcp(e1 + OFF * NORMAL, R_knife), FINGER_CLOSED, "lift off", R_hand),
             _wp(1.2, knife_tcp(e1 + BACK * NORMAL, R_knife), FINGER_CLOSED, "back from the panel", R_hand),
             Waypoint(3.5, (TURN_X, gy, TOP + LIFT), FINGER_CLOSED, "over the bench", KNIFE_YAW),
             Waypoint(1.2, (gx, gy, TOP + LIFT), FINGER_CLOSED, "over its place", KNIFE_YAW),
             Waypoint(1.5, (gx, gy, gz + 0.001), FINGER_CLOSED, "lay it down", KNIFE_YAW),
             Waypoint(1.0, (gx, gy, gz + 0.001), FINGER_OPEN, "let go", KNIFE_YAW),
             Waypoint(1.2, (gx, gy, gz + 0.08), FINGER_OPEN, "lift clear", KNIFE_YAW),
             Waypoint(2.0, (gx, gy, gz + 0.08), FINGER_OPEN, "wait", KNIFE_YAW)]
    return plan


VARIANTS = ("", "hand_only", "bare_bed", "knife_left", "steep")
STEEP_DEG = 60.0  # the steep variant's blade angle (bedkit's law leaves 0.1 mm: a starved, unbonded stretch) ...
STEEP_START = -0.04  # ... from 4 cm below the crack's top end (that steep, the arm cannot reach higher): 9 of 13 stations
HANDS_OFF_WAIT = 14.0  # s both hands wait at home at the end of the hand_only variant (the tape on its own)


def demo():
    """Left hand: pick up the tape by its top end and lay it on the crack, holding the end. Right hand: press the tape
    onto the bed with the fingertips down its length below the held end. Left: let go, press the top part the same way,
    go home. Right: take the knife, draw it down the tape from above the crack's top end past its bottom end, and lay
    the knife down.

    DRYWALL_VARIANT runs a cheat instead (each scores its cap): hand_only (the tape laid and dabbed by hand, no knife:
    0), bare_bed (the tape left on the bench, the knife drawn down the bare bed and laid back: 0), knife_left (the
    knife kept in hand on the wall at the stroke's end: 0.75), steep (the pass at 60 degrees from 4 cm below the top end,
    starving the bed: 0.5)."""
    variant = os.environ.get("DRYWALL_VARIANT", "")
    if variant not in VARIANTS:
        raise ValueError(f"DRYWALL_VARIANT {variant!r} is not one of {VARIANTS}")
    if variant == "bare_bed":
        return {"left": [kit.Home(1.0)], "right": [kit.Home(1.0)] + _right(sets=False)}
    left = _left()
    right = (_right(STEEP_DEG, start=STEEP_START) if variant == "steep" else _right(keep=variant == "knife_left"))
    t_laid = sum(w.duration for w in left)
    right.insert(0, kit.Home(t_laid + 0.5))  # wait at home for the tape to be laid
    n_set = 1 + len(_right_set())
    t_set_done = sum(w.duration for w in right[:n_set])
    left.append(_wp(max(0.5, t_set_done - t_laid), PIN_TCP, FINGER_CLOSED,
                    "keep the top end while the right hand sets the tape below", R_PIN))
    left += _left_set()
    if variant == "hand_only":  # the right goes home after its dabs, out of the left's way; both then wait, hands off
        return {"left": left + [kit.Home(HANDS_OFF_WAIT)], "right": right[:n_set] + [kit.Home(4.0)]}
    t_left_home = sum(w.duration for w in left)
    k = next(i for i, w in enumerate(right) if w.label == "square the blade to the tape")
    t_k = sum(w.duration for w in right[:k])
    right.insert(k, Waypoint(max(0.0, t_left_home - t_k), right[k - 1].pos, FINGER_CLOSED,
                             "wait for the left hand to clear", KNIFE_YAW))
    return {"left": left, "right": right}
