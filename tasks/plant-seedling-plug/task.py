"""On the station table, in front of the arms: a wooden planter box (30 x 30 cm inside, 9 cm deep) of firmed potting
mix, with a 10.3 cm planting hole cored 6.2 cm deep in its middle. The loose mix (440 mL,
200 g, crumbled into clods) lies heaped round it. A pepper seedling, popped from a 36-cell insert with its root plug
(4.9 x 5.7 cm across the top, 5.7 cm deep, 70 g), stands on the bench to the left: a 9 cm stem 3.6 mm across with two
seed leaves and four true leaves.

The job is to plant it: the plug goes in the hole upright, its top level with the surrounding soil, and the loose mix
goes back in round it on every side and is firmed, so the plant stands by itself, without creasing the stem or
crushing a leaf.

Every clod of loose mix is its own rigid body (soilkit). The plug and plant are one free articulated body: the root
ball, and a springy stem (two segments, each on a two-axis spring joint) carrying the leaves, which the hands and the
soil collide with. The score reads the end state: the plug centered in the hole (its top's center within 0.5 cm of
the hole's axis, turned any way), the soil's height at 16 points set out from the plug's own faces, so they follow the
plug wherever it stands and however it is turned, read as a real trial reads them with a 9.5 mm pin set at each
point from the plug's faces (the fill, and the level on each of the plug's four sides), the plug's tilt and depth, the
hands off it, and a side push of the weight of 50 g on the stem (pushkit) that a firmed plug resists. A damaged seedling (the stem
bent past its kink angle, or the stem or a leaf squeezed between a hand's fingers) scores 0. Every threshold below appears once, with the same value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import guardkit as gk  # noqa: E402
import mechkit as mk  # noqa: E402
import pushkit as pk  # noqa: E402
import soilkit as sk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Plant the seedling in the planter box, level with the soil, and firm it in so it stands by itself."

# -- the planter box (world frame: x away from the arms, y to the robot's left, z up; m) ----------------------------------------
# A wooden planter box, 30 x 30 cm inside, 9 cm deep, fixed on the bench, filled with moist potting mix
# firmed level 1 cm under its top edge. The planting hole is cored 10.3 cm across with a food can (401 x 411 size) to 6.2 cm deep (twice
# the root ball's width, as planting guides say), so it is the same every trial.
POT = (0.280, 0.000)  # the hole's center
BED_X, BED_Y = 0.300, 0.300  # inside
BED_WALL = 0.018
RIM_Z = TOP + 0.090  # the box's top edge
Z_S = TOP + 0.080  # the firmed mix's surface
R_HOLE = 0.0515  # the gap beside the plug's 5.7 cm faces is 2.3 cm: two 1.1 cm clods side by side (at 10 cm it
# was 2.15 cm, one clod across, and that side's top layer stayed thin in the simulator)
HOLE_D = 0.062
HOLE_FLOOR = Z_S - HOLE_D
R_RING = 0.120  # the blocks round the hole reach this far out
MIX = kit.Look((0.24, 0.17, 0.11), 0.95)
WOOD = kit.Look((0.62, 0.45, 0.30), 0.8)


def _planter() -> kit.Prop:
    # colliders: the mix under the hole's floor (one block), a ring of 16 blocks round the hole (their inner faces make
    # a 16-sided hole 10.3 cm across), four blocks for the rest of the box, and the box's four walls
    hz = (Z_S - HOLE_D / 2) - TOP
    cols = [kit.Collider((0.0, 0.0, (HOLE_FLOOR - TOP) / 2), size=(BED_X, BED_Y, HOLE_FLOOR - TOP))]
    n = 16
    rc = (R_HOLE + R_RING) / 2
    for k in range(n):
        a = 2 * math.pi * k / n
        cols.append(kit.Collider((rc * math.cos(a), rc * math.sin(a), hz),
                                 size=(R_RING - R_HOLE, 2 * R_RING * math.tan(math.pi / n) + 0.004, HOLE_D), yaw=a))
    sq = 0.085  # the ring covers this square round the hole; four blocks cover the rest
    cols += [kit.Collider((0.0, s * (sq + BED_Y / 2) / 2, hz), size=(BED_X, BED_Y / 2 - sq, HOLE_D)) for s in (-1, 1)]
    cols += [kit.Collider((s * (sq + BED_X / 2) / 2, 0.0, hz), size=(BED_X / 2 - sq, 2 * sq, HOLE_D)) for s in (-1, 1)]
    wz = (RIM_Z - TOP) / 2
    walls = [((0.0, s * (BED_Y + BED_WALL) / 2, wz), (BED_X + 2 * BED_WALL, BED_WALL, RIM_Z - TOP)) for s in (-1, 1)]
    walls += [((s * (BED_X + BED_WALL) / 2, 0.0, wz), (BED_WALL, BED_Y, RIM_Z - TOP)) for s in (-1, 1)]
    cols += [kit.Collider(c, size=sz) for c, sz in walls]
    zs = Z_S - TOP
    meshes = [kit.box(c, sz, WOOD) for c, sz in walls]
    meshes += [kit.box((0.0, s * (sq + BED_Y / 2) / 2, zs - 0.001), (BED_X, BED_Y / 2 - sq, 0.002), MIX) for s in (-1, 1)]
    meshes += [kit.box((s * (sq + BED_X / 2) / 2, 0.0, zs - 0.001), (BED_X / 2 - sq, 2 * sq, 0.002), MIX)
               for s in (-1, 1)]
    meshes += [kit.lathe([(R_HOLE, zs), (sq * math.sqrt(2) + 0.001, zs)], MIX, n=48, caps=False),  # round the hole
               kit.lathe([(R_HOLE, HOLE_FLOOR - TOP), (R_HOLE, zs)], MIX, n=16, caps=False),  # the hole's wall
               kit.lathe([(0.0, HOLE_FLOOR - TOP), (R_HOLE, HOLE_FLOOR - TOP)], MIX, n=16, caps=False)]  # its floor
    return kit.Prop("planter", pos=(POT[0], POT[1], TOP), size=(BED_X + 2 * BED_WALL, BED_Y + 2 * BED_WALL, RIM_Z - TOP),
                    meshes=tuple(meshes), colliders=tuple(cols))


# -- the plug: a pepper seedling from a 3601 insert (cell 4.9 x 5.7 cm at the top, 5.7 cm deep, 118 mL), popped out
# with its root ball whole: tapered to about 3.6 x 4.4 cm at the bottom, 70 g moist. Its 4.9 cm side runs along x.
# The plant is a real body jointed to the root ball (one articulation, free on the bench): a compact 4-5 week
# transplant's 9 cm stem, 3.6 mm across, in two 4.5 cm segments, each on a two-axis springy joint, carrying two seed
# leaves and four true leaves. The hands, the soil and the planter collide with the stem and leaves.
PLUG_H = 0.057
PLUG_TOP = (0.049, 0.057)  # x, y
PLUG_BOT = (0.036, 0.044)
PLUG_MASS = 0.070
PLUG_START = (0.220, 0.260)  # on the bench, beside the box (its leaves clear of the box)
ROOT = kit.Look((0.30, 0.21, 0.13), 0.95)
LEAF = kit.Look((0.25, 0.55, 0.20), 0.6)
STEM_LOOK = kit.Look((0.36, 0.55, 0.24), 0.6)
PLUG_LAYERS = 3
# Stem: 3.6 mm across (a young plug's), 9 cm tall in two 4.5 cm segments, each on a
# two-axis springy joint (at the root ball and at mid-height). Bending stiffness EI = E x pi d^4 / 64 with Young's
# modulus falling from the woody base to the soft growing top (measured along young tomato stems, the nearest
# solanaceous seedling measured: about 600 MPa in the lignified hypocotyl, down to about 10 MPa in the young upper stem); each joint carries its segment's EI / length (N*m/rad), so the chain
# bends like the beam. Damping about critical (springs back without ringing).
STEM_D = 0.0036
STEM_SEG = 0.045
STEM_E = (600e6, 60e6)  # Pa: the lower segment (hypocotyl), the upper (young stem)
STEM_I = math.pi * STEM_D ** 4 / 64
STEM_K = tuple(e * STEM_I / STEM_SEG for e in STEM_E)  # 0.110, 0.011 N*m/rad
STEM_C = (0.0015, 0.0003)  # N*m*s/rad
STEM_SEG_MASS = 0.0006  # kg: a 4.5 cm length of moist stem (density about 1 g/mL) and its node
STEM_H = len(STEM_E) * STEM_SEG
JOINT_MASS = 0.0002  # kg: the part of a node between its two bend axes (a joint needs a body; it is part of the stem)
LINK_ARMATURE = 2e-6  # kg*m^2 on each plant joint: the solver needs it for gram-light links on a 70 g body; the plant
# still springs back in about 0.1 s
LINK_INERTIA = 4e-7  # kg*m^2 on every axis of every plant link (a 1 g, 6.5 cm leaf alone has about 3.5e-7): a floor for
# the stem, whose own shape gives 1e-10 about its axis, which the solver cannot step beside a 70 g body
# The shoot of a compact pepper transplant: simple, ovate true leaves, closely spaced up the stem above the two seed
# leaves. Leaf positions are given as height over the plug's top and direction in plan (degrees from +x).
# Seed leaves (cotyledons): a narrow pair, 1.8 x 0.5 cm, 2 cm up the stem, opposite each other on the diagonal
# (-30 and 150 degrees), part of the lower stem's body. They reach 2 cm out, inside the root ball's outline, so the
# fingers that hold the root ball by its x faces pass 6 mm clear of them.
COTYLEDON_H, COTYLEDON_LEN, COTYLEDON_W = 0.020, 0.018, 0.005
COTYLEDON_DEGS = (-30.0, 150.0)
# True leaves 1-3: a petiole and an ovate blade (widest below its middle), 6.5, 6.5 and 5 cm long in all, the blade up
# to 2.6 cm wide, 1.0 g each, on the lower stem at 3.2, 4.2 and 3.7 cm: each on its petiole's springy two-axis joint, so
# a hand brushing or pressing it swings it round the stem or folds it up or down (to 86 degrees either way), and it
# springs back (pushed end-on, it pushes the stem). They point to the robot's right (-75, -105 and -90 degrees from +x),
# clear of the fingers that hold the root ball by its x faces (a leaf along x was caught between them in the
# simulator). (On the upper segment, a leaf's two joints would make the articulation seven bodies deep, which the
# simulator cannot step: it blew up at spawn.)
LEAF_T = 0.002
LEAF_MASS = 0.0010
LEAF_K = 0.008  # N*m/rad, each axis: the petiole's return spring (a 1 g leaf held level by it sags about 2.5 degrees)
LEAF_C = 0.0002  # N*m*s/rad
LEAVES = (("leaf1", 0, 0.032, -75.0, 0.065), ("leaf2", 0, 0.042, -105.0, 0.065), ("leaf3", 0, 0.037, -90.0, 0.050))
# name, stem segment, height over the plug's top, direction (deg), length (m)
LEAF_SHAPE = ((0.30, 0.0025), (0.45, 0.018), (0.70, 0.026), (0.87, 0.020), (1.00, 0.009))  # the leaf from the petiole's
# base out, in pieces: (where the piece ends, as a share of the leaf's length past the petiole's base; its width): the
# petiole, then the blade
# True leaf 4, the youngest: 2.5 cm long, 1.2 cm wide, 0.3 g, 7 cm up (on the upper stem), toward -100 degrees, where
# no hand passes at that height. It hangs on one springy joint about the horizontal axis across it (it folds up or down
# when pressed; one joint keeps the articulation five links deep), and above it the stem ends in the growing tip, a bud
# 6 mm across.
TOP_LEAF = ("leaf4", 1, 0.070, -100.0, 0.025)
TOP_LEAF_MASS, TOP_LEAF_K = 0.0003, 0.004
TOP_LEAF_SHAPE = ((0.25, 0.002), (0.50, 0.009), (0.80, 0.012), (1.00, 0.006))
BUD_R, BUD_H = 0.003, 0.006
# Damage. Kinked: a stem joint bent more than KINK_DEG (a crease or break). Pinched: a stem segment or leaf touched by
# both fingers of one hand at once (squeezed between the pads).
KINK_DEG = 45.0
PINCH_MIN_GAP = 0.0005  # m: each pad this far from the tool point at least, i.e. something holds the fingers apart
DAMAGE_STEPS = 2  # control steps in a row that confirm it
NODE_GAP = 0.003  # m: each segment's collider stops this short of its joints, where the node's own small collider sits
NODE = (kit.Collider((0.0, 0.0, 0.0), size=(0.002, 0.002, 0.002)),)  # every body of the articulation needs a shape
LEAF_ROOT = 0.006  # m: the leaf's collider starts this far out from the stem's axis (the petiole's base)


def plug_width(frac_from_bottom: float) -> tuple:
    return tuple(b + (t - b) * frac_from_bottom for b, t in zip(PLUG_BOT, PLUG_TOP))


def _leaf_parts(deg: float, length: float, shape) -> list:
    """A leaf's flat pieces in its own frame (origin on the stem's axis at the leaf's node), as (center, size, turn):
    the petiole, then the blade in strips along it, each as wide as the blade there."""
    th = math.radians(deg)
    u = np.array([math.cos(th), math.sin(th), 0.0])
    out, r0 = [], LEAF_ROOT
    for f, w in shape:
        r1 = LEAF_ROOT + f * (length - LEAF_ROOT)
        out.append((tuple(float(x) for x in u * (r0 + r1) / 2), (r1 - r0, w, LEAF_T), th))
        r0 = r1
    return out


def _cotyledons() -> list:
    """The seed leaves as (center, size, turn) in the lower stem segment's frame (origin at the plug's top)."""
    out = []
    for deg in COTYLEDON_DEGS:
        th = math.radians(deg)
        r = STEM_D / 2 + COTYLEDON_LEN / 2
        out.append(((r * math.cos(th), r * math.sin(th), COTYLEDON_H), (COTYLEDON_LEN, COTYLEDON_W, LEAF_T), th))
    return out


def _lift_axis(deg: float) -> str:
    th = math.radians(deg)
    return "Y" if abs(math.cos(th)) > abs(math.sin(th)) else "X"  # the horizontal axis across the leaf


def _stem_links() -> list:
    top = PLUG_H / 2
    links, parent = [], None
    for k, (kk, c) in enumerate(zip(STEM_K, STEM_C)):
        a = (0.0, 0.0, top + k * STEM_SEG)
        meshes = [kit.cylinder((0.0, 0.0, STEM_SEG / 2), STEM_D / 2, STEM_SEG, STEM_LOOK, n=12)]
        cols = [kit.Collider((0.0, 0.0, STEM_SEG / 2), radius=STEM_D / 2, height=STEM_SEG - 2 * NODE_GAP)]
        if k == 0:  # the seed leaves, on the lower stem
            meshes += [kit.box(cc, sz, LEAF, yaw=th) for cc, sz, th in _cotyledons()]
            cols += [kit.Collider(cc, size=sz, yaw=th) for cc, sz, th in _cotyledons()]
        if k == len(STEM_K) - 1:  # the growing tip
            meshes.append(kit.cylinder((0.0, 0.0, STEM_SEG), BUD_R, BUD_H, LEAF, n=12))
            cols.append(kit.Collider((0.0, 0.0, STEM_SEG), radius=BUD_R, height=BUD_H))
        links.append(kit.Link(f"bend{k}", "revolute", "X", anchor=a, lower=-1.4, upper=1.4, mass=JOINT_MASS,
                              inertia=LINK_INERTIA, armature=LINK_ARMATURE, damping=c, colliders=NODE, parent=parent,
                              law=mk.Spring(0.0, kk, c)))
        links.append(kit.Link(f"stem{k}", "revolute", "Y", anchor=a, lower=-1.4, upper=1.4, mass=STEM_SEG_MASS,
                              inertia=LINK_INERTIA, armature=LINK_ARMATURE, damping=c, parent=f"bend{k}",
                              meshes=tuple(meshes), colliders=tuple(cols), law=mk.Spring(0.0, kk, c)))
        parent = f"stem{k}"
    for name, seg, h, deg, length in LEAVES:
        parts = _leaf_parts(deg, length, LEAF_SHAPE)
        a = (0.0, 0.0, top + h)
        links.append(kit.Link(f"{name}_swing", "revolute", "Z", anchor=a, lower=-2.2, upper=2.2, mass=JOINT_MASS,
                              inertia=LINK_INERTIA, armature=LINK_ARMATURE, damping=LEAF_C, colliders=NODE,
                              parent=f"stem{seg}", law=mk.Spring(0.0, LEAF_K, LEAF_C)))
        links.append(kit.Link(name, "revolute", _lift_axis(deg), anchor=a, lower=-1.5, upper=1.5, mass=LEAF_MASS,
                              inertia=LINK_INERTIA, armature=LINK_ARMATURE, damping=LEAF_C, parent=f"{name}_swing",
                              meshes=tuple(kit.box(cc, sz, LEAF, yaw=th) for cc, sz, th in parts),
                              colliders=tuple(kit.Collider(cc, size=sz, yaw=th) for cc, sz, th in parts),
                              law=mk.Spring(0.0, LEAF_K, LEAF_C)))
    name, seg, h, deg, length = TOP_LEAF
    parts = _leaf_parts(deg, length, TOP_LEAF_SHAPE)
    links.append(kit.Link(name, "revolute", _lift_axis(deg), anchor=(0.0, 0.0, top + h), lower=-1.5, upper=1.5,
                          mass=TOP_LEAF_MASS, inertia=LINK_INERTIA, armature=LINK_ARMATURE, damping=LEAF_C,
                          parent=f"stem{seg}", meshes=tuple(kit.box(cc, sz, LEAF, yaw=th) for cc, sz, th in parts),
                          colliders=tuple(kit.Collider(cc, size=sz, yaw=th) for cc, sz, th in parts),
                          law=mk.Spring(0.0, TOP_LEAF_K, LEAF_C)))
    return links


def _plug() -> kit.Mechanism:
    cols, meshes = [], []
    for i in range(PLUG_LAYERS):
        w = plug_width((i + 0.5) / PLUG_LAYERS)
        zc = -PLUG_H / 2 + (i + 0.5) * PLUG_H / PLUG_LAYERS
        cols.append(kit.Collider((0.0, 0.0, zc), size=(w[0], w[1], PLUG_H / PLUG_LAYERS)))
        meshes.append(kit.box((0.0, 0.0, zc), (w[0], w[1], PLUG_H / PLUG_LAYERS), ROOT))
    return kit.Mechanism("plug", pos=(PLUG_START[0], PLUG_START[1], TOP + PLUG_H / 2 + 0.0005), mass=PLUG_MASS,
                         size=(PLUG_TOP[0], PLUG_TOP[1], PLUG_H), meshes=tuple(meshes), colliders=tuple(cols),
                         links=tuple(_stem_links()))


# -- the loose mix: the mix dug out of the hole, crumbled into clods: 1.1 cm cubes of 1.0 g, 200 of them (200 g; about
# 440 mL loose). Clods, not balls: moist mix crumbles into lumps that do not roll, so a heap holds its slope at the
# hole's edge (a heap of 1.2 cm spheres slumped into the hole in the simulator before the trial began). The gap round a
# level plug (about 350 mL) is 2.3-2.7 cm wide at the top, two clods across, so the clods pack loosely in it; the
# 440 mL of loose mix is about 1.25 times its volume. They lie heaped round the hole, from 6 to 10 cm out, two clods deep, turned a little each (a fixed seed):
# the same every trial.
GRAIN_R = 0.0055  # half the clod's side
GRAIN_MASS = 0.0010
N_GRAINS = 200
GRAIN_BULK = (2 * GRAIN_R) ** 3 / 0.6  # m^3 of loose mix per clod
R_HEAP_IN = 0.060
R_HEAP_OUT = 0.100
HEAP_PITCH = 0.0135  # clod centers in a layer
HEAP_RISE = 0.0115  # layer to layer
MIX_COLORS = ((0.24, 0.17, 0.11), (0.20, 0.14, 0.09), (0.29, 0.21, 0.13))


def _grains() -> tuple:
    rng = np.random.default_rng(20261114)
    out, pitch = [], HEAP_PITCH
    z = Z_S + GRAIN_R + 0.0005
    layer = 0
    while len(out) < N_GRAINS:
        off = (pitch / 2) * (layer % 2)
        rin, rout = R_HEAP_IN + 0.004 * layer, R_HEAP_OUT - 0.005 * layer  # a heap, not a wall: each layer narrower
        if layer > 4:
            raise ValueError("the loose mix does not fit round the hole")
        for j in range(-9, 10):
            for i in range(-9, 10):
                x = i * pitch + off + (j % 2) * pitch / 2
                y = j * pitch * math.sqrt(3) / 2 + off * 0.5
                if rin <= math.hypot(x, y) <= rout and len(out) < N_GRAINS:
                    a = float(rng.uniform(-0.15, 0.15))
                    out.append(sk.Grain(f"clod_{len(out):03d}", (POT[0] + x + float(rng.uniform(-3e-4, 3e-4)),
                                                                 POT[1] + y + float(rng.uniform(-3e-4, 3e-4)), z),
                                        GRAIN_R, GRAIN_MASS, MIX_COLORS[int(rng.integers(len(MIX_COLORS)))],
                                        cube=True, quat_xyzw=(0.0, 0.0, math.sin(a / 2), math.cos(a / 2))))
        z += HEAP_RISE
        layer += 1
    return tuple(out)


GRAINS = _grains()
sk.setup(GRAINS)

OBJECTS = [_planter(), _plug()]

# -- scoring ------------------------------------------------------------------------------------------------------------
# The soil in the hole is read the way a real trial reads it: a probe lowered at 16 points until it rests on the mix (a
# pin profile meter). The points are set out from the plug, not from the box: measured from the plug's top edges with
# a square and rule (in the simulator, the plug's own frame at its top), there are four points beside each face, 1.0 cm out from the
# face, at 15 and 50 percent of the way from the face's middle toward either end. So wherever the plug stands and
# however it is turned, every probe lands in the gap: its 9.5 mm end 5.25 mm clear of the face (tilted 10 degrees, the
# plug's lower side comes at most 3.5 mm past its top's outline, 4.5 mm for the simulator's three stepped layers).
# The probe is a 9.5 mm steel dowel pin, flat end down; in the simulator it is a 9.5 mm disc that comes down until any part of it
# meets a clod (each clod's own turned cube) or the plug, else the hole's floor (the firmed surface beyond the wall).
# - Placement range: the plug's top centered within PLACE_R of the hole's axis, any turn about its axis, upright within
#   TILT_FULL. Inside it, every probe lands wholly inside the hole's wall (the farthest point is 4.04 cm from the plug's
#   center: 4.04 + 0.475 + 0.5 = 5.015 cm, 1.35 mm inside the 5.15 cm wall; with 0.5 mm for setting a pin by the rule
#   and 0.5 mm for reading the offset, 0.35 mm is left). Outside it the readings could fall on the hole's edge, so the trial is not scored as planted: 0.
# - Fill at a point: how far the soil there has risen from the hole's floor, as a share of the hole's 6.2 cm depth,
#   capped at 1: mix heaped above the surface counts as full, never more. The fill is the mean over the 16 points.
# - The soil level on a side of the plug: the mean height of that side's four points (each capped at the surface).
#   A side is filled when its level is within 1 cm of the plug's top.
# The plug's top is planted level when it is within 1 cm of the firmed surface (Z_S). Upright is within 10 degrees,
# for every tier: more lean would bring the plug's lower side under the probes.
DEPTH_TOL = 0.010
LEVEL_TOL = 0.010
TILT_FULL = 10.0  # deg
FILL_FULL = 0.85  # mean fill at the 16 probe points, for 1.0
FILL_HALF = 0.60
FILL_QUARTER = 0.40
PLACE_R = 0.005  # m: the plug's top center from the hole's axis, in plan
PROBE_OUT = 0.010  # m: each probe point's distance out from its face of the plug's top
PROBE_ALONG = (-0.5, -0.15, 0.15, 0.5)  # where along the face, as a share of its half-length from its middle
PROBE_SIDE = tuple(k // 4 for k in range(16))  # the side (+x, +y, -x, -y, in the plug's frame) each point is on
PROBE_D = 0.0095  # m: a 3/8 in steel dowel pin, set flat end down
PROBE_SAMPLES = 13  # points across the probe's end: its center, 6 at half its radius, 6 on its rim
SPILL_MAX = 0.10  # share of the clods out of the planter box (on the bench or the floor): more caps the score at 0.5
HANDS_CLEAR = 0.010  # m: every gripper's own shape at least 1 cm from the plug
STILL_STEP = 0.0005  # m per control step (1 cm/s)
STILL_TURN = math.radians(0.5)
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
# The firmness test: once the planted state has held for 1 s, a side push of 0.5 N (the weight of 50 g) at 5 cm above
# the plug's top (on the stem) for 1 s, first toward +y, then toward -y. Firmed: the plug turns less than 5 degrees under it and is upright
# again after it. A plug standing loose in the hole, or held on one side only, tips over. The push stops as soon as the
# plug has turned 5 degrees (the test has failed), so a failed test does not lay the plant over; it is not repeated
# until a hand has worked on the plug again.
PUSH_N = 0.5
PUSH_ABOVE = 0.050
PUSH_STEPS = 20
PUSH_TILT = 5.0  # deg
SETTLE_STEPS = 10  # 0.5 s after the push before reading the tilt again

PLUG_PARTS = tuple(gk.Block((0.0, 0.0, -PLUG_H / 2 + (i + 0.5) * PLUG_H / PLUG_LAYERS),
                            (*plug_width((i + 0.5) / PLUG_LAYERS), PLUG_H / PLUG_LAYERS)) for i in range(PLUG_LAYERS))
GRIPPERS = ("left_gripper", "right_gripper")
STEM_KEYS = tuple(f"plug/stem{k}" for k in range(len(STEM_E)))  # each stem segment's body, base to top
PLANT_PARTS = {key: [gk.Seg((0.0, 0.0, 0.0), (0.0, 0.0, STEM_SEG), STEM_D / 2)] for key in STEM_KEYS}


def _flat_segs(pieces) -> list:
    """Each flat leaf piece (center, size, turn) as three thin lines along it: its middle and both edges."""
    out = []
    for cc, sz, th in pieces:
        u = np.array([math.cos(th), math.sin(th), 0.0])
        v = np.array([-u[1], u[0], 0.0])
        c = np.asarray(cc, float)
        out += [gk.Seg(tuple(float(x) for x in c - u * sz[0] / 2 + v * w), tuple(float(x) for x in c + u * sz[0] / 2 + v * w),
                       LEAF_T / 2) for w in (-sz[1] / 2, 0.0, sz[1] / 2)]
    return out


PLANT_PARTS[STEM_KEYS[0]] += _flat_segs(_cotyledons())
PLANT_PARTS[STEM_KEYS[-1]].append(gk.Seg((0.0, 0.0, STEM_SEG - BUD_H / 2), (0.0, 0.0, STEM_SEG + BUD_H / 2), BUD_R))
for _name, _seg, _h, _deg, _len in LEAVES:
    PLANT_PARTS[f"plug/{_name}"] = _flat_segs(_leaf_parts(_deg, _len, LEAF_SHAPE))
PLANT_PARTS[f"plug/{TOP_LEAF[0]}"] = _flat_segs(_leaf_parts(TOP_LEAF[3], TOP_LEAF[4], TOP_LEAF_SHAPE))
PLANT_PARTS = {k: tuple(v) for k, v in PLANT_PARTS.items()}


def _void_volume() -> float:
    """The hole's volume round a level-planted plug, up to the surface (m^3): the hole minus the plug's part below the
    surface (the plug's top at the surface; its bottom 5 mm above the floor, a gap no clod reaches)."""
    hole = math.pi * R_HOLE ** 2 * HOLE_D
    w0, w1 = PLUG_BOT, PLUG_TOP
    plug = PLUG_H * (w0[0] * w0[1] + w1[0] * w1[1] + math.sqrt(w0[0] * w0[1] * w1[0] * w1[1])) / 3
    under = math.pi * R_HOLE ** 2 * (HOLE_D - PLUG_H)
    return hole - plug - under


VOID = _void_volume()


def rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def plug_frame(objects):
    p = objects["plug"]
    return np.asarray(p.pos, float), rotm(p.quat_xyzw)


def tilt_deg(r) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(r[2, 2])))))


def plug_top(objects) -> np.ndarray:
    c, r = plug_frame(objects)
    return c + r @ np.array([0.0, 0.0, PLUG_H / 2])


def offset(objects) -> float:
    """How far the plug's top center is from the hole's axis, in plan (m)."""
    top = plug_top(objects)
    return math.hypot(float(top[0]) - POT[0], float(top[1]) - POT[1])


def centered(objects) -> bool:
    """In the hole, inside the placement range: its top's center within PLACE_R of the hole's axis, its center below
    the surface."""
    c, _ = plug_frame(objects)
    return bool(offset(objects) <= PLACE_R + 1e-9 and c[2] < Z_S)  # 1e-9: float noise


def soil(objects) -> np.ndarray:
    s = objects.get(sk.KEY)
    if s is None:
        return np.array([g.pos for g in GRAINS], float)
    return np.asarray(s, float).reshape(-1, 3)


def soil_quats(objects, n: int) -> np.ndarray:
    q = objects.get(sk.KEY_QUAT)
    if q is None:  # no orientations given (offline checks): the clods lie square
        return np.tile(np.array([0.0, 0.0, 0.0, 1.0]), (n, 1))
    return np.asarray(q, float).reshape(-1, 4)


def probe_offsets() -> np.ndarray:
    """(16, 2): the probe points in the plug's frame, on its top (x, y from the stem's axis), side by side."""
    hx, hy = PLUG_TOP[0] / 2, PLUG_TOP[1] / 2
    out = []
    for sx, sy in ((1, 0), (0, 1), (-1, 0), (0, -1)):  # +x, +y, -x, -y faces
        for f in PROBE_ALONG:
            out.append((sx * (hx + PROBE_OUT) + sy * f * hx, sy * (hy + PROBE_OUT) + sx * f * hy))
    return np.array(out, float)


PROBE_OFF = probe_offsets()
PROBE_END = np.array([(0.0, 0.0)] + [(rr * math.cos(math.radians(a)), rr * math.sin(math.radians(a)))
                                     for rr in (PROBE_D / 4, PROBE_D / 2) for a in range(0, 360, 60)])  # across its end


def probe_points(objects) -> np.ndarray:
    """The 16 points (x, y), world frame: the probe points carried with the plug's top (its position and turn)."""
    top = plug_top(objects)
    _, r = plug_frame(objects)
    p = top[None, :] + np.column_stack([PROBE_OFF, np.zeros(16)]) @ r.T
    return p[:, :2]


def _drop_onto_boxes(xy, centers, rots, halves) -> np.ndarray:
    """For vertical lines coming down at each (x, y) in `xy` (M, 2): the height (m) where each first meets each box
    (center, rotation, half sizes); -inf where it misses. Returns (M, K)."""
    top = Z_S + 0.2
    o = np.concatenate([xy, np.full((len(xy), 1), top)], axis=1)  # (M, 3)
    rel = o[:, None, :] - centers[None, :, :]  # (M, K, 3)
    lo = np.einsum("mki,kij->mkj", rel, rots)  # the line's start in each box's frame
    ld = np.broadcast_to(-rots[:, 2, :], lo.shape)  # its direction (straight down) in each box's frame
    h = np.broadcast_to(halves, lo.shape)
    with np.errstate(divide="ignore", invalid="ignore"):
        t1 = (-h - lo) / ld
        t2 = (h - lo) / ld
    flat = np.abs(ld) < 1e-12
    inside = np.abs(lo) <= h
    tmin = np.where(flat, np.where(inside, -np.inf, np.inf), np.minimum(t1, t2)).max(axis=2)
    tmax = np.where(flat, np.where(inside, np.inf, -np.inf), np.maximum(t1, t2)).min(axis=2)
    hit = (tmin <= tmax) & (tmax >= 0.0)
    return np.where(hit, top - np.maximum(tmin, 0.0), -np.inf)


def probe_heights(objects, s=None) -> np.ndarray:
    """The 16 probe readings (m, world z): where a 9.5 mm flat-ended probe lowered at each probe point comes to
    rest, on the highest clod or plug face under any part of its end, else on the hole's floor (or the surface, for
    any part of its end beyond the hole's wall)."""
    s = soil(objects) if s is None else s
    q = soil_quats(objects, len(s))
    rots = [rotm(v) for v in q]
    centers, halves = [np.asarray(s, float)], [np.full((len(s), 3), GRAIN_R)]
    c, r = plug_frame(objects)
    for blk in PLUG_PARTS:  # the root ball's three layers
        centers.append((c + r @ np.asarray(blk.center, float))[None, :])
        halves.append((np.asarray(blk.size, float) / 2)[None, :])
        rots.append(r)
    xy = probe_points(objects)[:, None, :] + PROBE_END[None, :, :]  # (16, PROBE_SAMPLES, 2)
    z = _drop_onto_boxes(xy.reshape(-1, 2), np.concatenate(centers), np.array(rots), np.concatenate(halves))
    inside = np.hypot(xy[..., 0] - POT[0], xy[..., 1] - POT[1]) < R_HOLE
    floor = np.where(inside, HOLE_FLOOR, Z_S)  # the hole's floor, or the firmed surface beyond its wall
    return np.maximum(z.max(axis=1).reshape(xy.shape[:2]), floor).max(axis=1)


def point_fill(h) -> np.ndarray:
    """Each point's fill: the soil's rise from the hole's floor as a share of the hole's depth, 0 to 1 (heaped mix
    above the surface counts as full, never more)."""
    return np.clip((np.asarray(h, float) - HOLE_FLOOR) / HOLE_D, 0.0, 1.0)


def fill_share(h) -> float:
    """The mean fill over the 16 points."""
    return float(point_fill(h).mean())


def side_levels(h) -> list:
    """The soil level (m, world z) on each side of the plug (+x, +y, -x, -y): the mean of its four points' heights,
    each capped at the surface."""
    hc = np.minimum(np.asarray(h, float), Z_S)
    return [float(hc[[k for k in range(16) if PROBE_SIDE[k] == side]].mean()) for side in range(4)]


def spill_share(s) -> float:
    """Clods out of the planter box: beyond its walls in plan (on the bench, the floor, or carried off in a hand)."""
    out = (np.abs(s[:, 0] - POT[0]) > BED_X / 2 + BED_WALL) | (np.abs(s[:, 1] - POT[1]) > BED_Y / 2 + BED_WALL)
    return float(out.sum()) / len(s)


def hands_off(objects) -> bool:
    """Every gripper's own shape at least 1 cm from the root ball, the stem and the leaves."""
    for g in GRIPPERS:
        if gk.hand_gap(objects, g, "plug", PLUG_PARTS) < HANDS_CLEAR:
            return False
        if any(key in objects and gk.hand_gap(objects, g, key, parts) < HANDS_CLEAR for key, parts in PLANT_PARTS.items()):
            return False
    return True


def stem_bends(objects) -> list:
    """The bend at each stem joint, degrees: the turn between the root ball and the first segment, and between each
    segment and the next (both bending axes together), as a protractor on a photo of the stem would read it."""
    keys = ("plug",) + STEM_KEYS
    if any(k not in objects for k in keys):
        return []
    rs = [rotm(objects[k].quat_xyzw) for k in keys]
    return [math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(a.T @ b)) - 1) / 2)))) for a, b in zip(rs, rs[1:])]


def pinched(objects) -> tuple:
    """(gripper, part) for a stem segment or leaf squeezed by both fingers of one hand: the fingers held apart (not a
    closed fist) and each finger's own shape (body, pad and tip, at the present opening) within 2 mm of it at once. The YAM's fingers are a stiff position servo, so a
    part touched from both sides is pressed between them."""
    for g in GRIPPERS:
        if g not in objects:
            continue
        for key, parts in PLANT_PARTS.items():
            if key not in objects:
                continue
            o, cg, Rg, co, Ro = gk._hand_frames(objects, g, key)
            if o < PINCH_MIN_GAP:
                continue  # a closed fist: nothing between the pads
            boxes = gk.hand_boxes(o)
            sides = (boxes[1:4], boxes[4:7])
            if gk.hand_gap(objects, g, key, parts) > gk.CONTACT:
                continue
            if all(any(gk._box_gap(cg, Rg, b, co, Ro, part) <= gk.CONTACT for b in side for part in parts)
                   for side in sides):
                return g, key
    return ()


class Ledger:
    """One trial's history, advanced once per control step: the planted-state hold, the firmness push and its result."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.objs = None
        self.last = None
        self.good = 0  # consecutive steps in the planted state, hands off, still
        self.push = 0  # steps into the firmness test (0: not started)
        self.base_tilt = None
        self.peak = 0.0
        self.firm = False
        self.damage = None  # latched: "kinked" (a stem joint past KINK_DEG) or "pinched" (squeezed by a hand)
        self.kink_steps = 0
        self.pinch_steps = 0
        self.max_bend = 0.0
        self.failed = False  # the push turned it 5 degrees: not firm; not pushed again until a hand works on it again
        self.state = {}

    def planted(self, objects, s) -> dict:
        c, r = plug_frame(objects)
        top = plug_top(objects)
        h = probe_heights(objects, s)
        lv = side_levels(h)
        st = dict(tilt=tilt_deg(r), depth=float(top[2] - Z_S), levels=[v - float(top[2]) for v in lv],
                  fill=fill_share(h), spill=spill_share(s), hole=centered(objects), offset=offset(objects),
                  off=hands_off(objects))
        st["sides"] = sum(v >= -LEVEL_TOL for v in st["levels"])
        st["full"] = (st["hole"] and st["sides"] == 4 and st["fill"] >= FILL_FULL and st["tilt"] <= TILT_FULL
                      and abs(st["depth"]) <= DEPTH_TOL and st["spill"] <= SPILL_MAX and st["off"])
        return st

    def tier(self) -> float:
        st = self.state
        if self.damage is not None or not st or not st["hole"]:
            return 0.0
        t = 0.0
        if st["tilt"] <= TILT_FULL and st["fill"] >= FILL_QUARTER:
            t = 0.25
        if st["tilt"] <= TILT_FULL and st["fill"] >= FILL_HALF and st["sides"] >= 3 and st["off"]:
            t = 0.5
        if st["full"]:
            t = 0.75  # planted and filled, firmness not yet shown
        if st["spill"] > SPILL_MAX:
            t = min(t, 0.5)
        if self.firm and st["full"]:
            t = 1.0
        return t


_L = Ledger()


def _update(objects):
    global _L
    if _L.episode != kit.episode():
        pk.request("plug", None)
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else None
    if (key is not None and key == _L.key) or (key is None and objects is _L.objs):
        return
    _L.key, _L.objs = key, objects
    bends = stem_bends(objects)
    _L.max_bend = max([_L.max_bend] + bends)
    _L.kink_steps = _L.kink_steps + 1 if any(b > KINK_DEG for b in bends) else 0
    _L.pinch_steps = _L.pinch_steps + 1 if pinched(objects) else 0
    if _L.damage is None and _L.kink_steps >= DAMAGE_STEPS:
        _L.damage = "kinked"
    if _L.damage is None and _L.pinch_steps >= DAMAGE_STEPS:
        _L.damage = "pinched"
    s = soil(objects)
    st = _L.planted(objects, s)
    _L.state = st
    if os.environ.get("SEEDLING_TRACE") and n is not None and n % 40 == 0:  # a debugging trace of the scored state
        print(f"seedling {n}: offset {st['offset'] * 100:.2f} cm tilt {st['tilt']:.1f} "
              f"depth {st['depth'] * 100:.2f} cm levels "
              f"{[round(v * 100, 2) for v in st['levels']]} fill {st['fill']:.2f} spill {st['spill']:.2f} "
              f"off {st['off']} bends {[round(b) for b in bends]} damage {_L.damage} firm {_L.firm}", flush=True)
    c, r = plug_frame(objects)
    still = False
    if _L.last is not None:
        dp = float(np.linalg.norm(c - _L.last[0]))
        dr = math.acos(max(-1.0, min(1.0, (float(np.trace(_L.last[1].T @ r)) - 1) / 2)))
        still = dp <= STILL_STEP and dr <= STILL_TURN
    _L.last = (c, r)
    if _L.push:  # the firmness test is running: it ends early if a hand comes back to the plug
        if not st["off"] or not st["hole"]:
            pk.request("plug", None)
            _L.push, _L.good, _L.peak = 0, 0, 0.0
            return
        _L.push += 1
        _L.peak = max(_L.peak, abs(st["tilt"] - _L.base_tilt))
        if _L.peak >= PUSH_TILT:  # gave way: the test has its answer, so the push stops before it lays the plant over
            pk.request("plug", None)
            _L.push, _L.good, _L.failed = 0, 0, True
            return
        top = plug_top(objects)
        point = (float(top[0]), float(top[1]), float(top[2] + PUSH_ABOVE))
        if _L.push <= PUSH_STEPS:
            pk.request("plug", (0.0, PUSH_N, 0.0), point)
        elif _L.push <= 2 * PUSH_STEPS:
            pk.request("plug", (0.0, -PUSH_N, 0.0), point)
        else:
            pk.request("plug", None)
        if _L.push >= 2 * PUSH_STEPS + SETTLE_STEPS:
            _L.firm = _L.peak < PUSH_TILT and st["full"]
            _L.push = 0
            _L.good = 0
        return
    if _L.firm and not st["full"]:
        _L.firm = False  # disturbed after the test: it no longer stands
    if _L.failed and not st["off"]:
        _L.failed = False  # a hand is working on it again (firming it): it may be pushed again once let go
    _L.good = _L.good + 1 if (st["full"] and still) else 0
    if not _L.firm and not _L.failed and _L.good >= HOLD_STEPS:
        _L.push, _L.base_tilt, _L.peak = 1, st["tilt"], 0.0


def success(objects):
    """Planted: in the hole, its top's center within 0.5 cm of the hole's axis (any turn), upright within 10 degrees,
    its top within 1 cm of the surface, soil within 1 cm of its top on all four sides, the hole at least 85 % filled at
    the 16 probe points, at most 10 % of the mix spilled, hands off, it held through the side push of the weight of 50 g (turned under 5 degrees), and the seedling undamaged."""
    _update(objects)
    return bool(_L.firm and _L.state.get("full") and _L.damage is None)


def score(objects):
    """From the end state: 1.0 planted, filled on all four sides, hands off and firm under the side push; 0.75 the
    same but not yet pushed, or tipping under the push; 0.5 three sides filled and at least 60 % filled, hands off;
    0.25 at least 40 % filled. Every tier needs the plug inside the placement range: its top's center within 0.5 cm of
    the hole's axis (any turn about its own) and upright within 10 degrees; outside it, 0. More than 10 % of the mix
    spilled caps the score at 0.5. A damaged seedling (a stem joint bent past 45 degrees, or the stem or a leaf squeezed
    between one hand's fingers, either for 2 control steps running) scores 0 for the whole trial. Doing nothing
    scores 0."""
    _update(objects)
    return _L.tier()


# -- scripted demo: open loop from the fixed layout ---------------------------------------------------------------------
# Left hand: comes down beside the plant and moves in to pinch the plug across its 4.9 cm sides by the root ball (yaw
# 90: the fingers close along x), 1.5 cm under its top, lifts it, sets it in the hole with its top 2 mm under the
# surface, and holds it there while the right hand pushes the mix in on the -y half. Then it lets go, moves out
# sideways and pushes the mix in on the +y half and the -x side, and presses it down there; the right hand then does
# the +x side and presses the rest. The strokes run under the leaves, never through the stem. Both hands end clear.
GRIP_BELOW_TOP = 0.015
LEAN = 0.5  # rad: while both hands work at the hole, each hand leans its wrist away from the other (left toward +y,
# right toward -y), so the wrists stay apart. The left hand takes the plug already leaned, so the plug stays upright.
TILT_FIX = 0.135  # rad: the leaned pinch holds the plug 6-8 degrees off upright, its top toward -x (measured in the
# simulator); once the plug is in the hole the hand pitches back by this much about y, so the plug stands upright
# (1-2 degrees) while the mix goes in
GRASP_SHIFT = 0.002  # m: the plug's center sits this far along +x from the tool point once pinched (yaw 90), measured
# in the simulator with the hand coming in from the side of the plant (0.0-0.24 cm through the carry and hold; coming
# straight down it had been 2 cm: one finger then barely moved)
RELEASE_OPEN = 0.62 * FINGER_OPEN  # let go by opening about 1 cm past the plug's 4.9 cm width, not fully: a finger
# opened all the way swings out through the packed gap into the hole's wall, shoving the hand (measured: 1.2 cm
# sideways and 1 cm up) and tipping the plug about 28 degrees
APPROACH_SHIFT = 0.015  # m: going down beside the plug, the tool point this far toward -x of the plug's center, so the
# +x finger passes 5 mm outside the plug's face
YAW_PLUG = math.pi / 2
PLANT_SIDE = 0.025  # m: the left hand comes down this far to the +y side of the plug and then moves in round it, and
# leaves the same way. The leaned hand's finger mounts cross its middle 4-8 cm above the tool point, on the +y side of
# the stem once the hand is round the plug: coming straight down they would land on the stem, and lifting straight up
# from the plug they caught the stem and pulled the plug out of the hole (simulator); sideways they pass beside it
SET_DEPTH = 0.002  # the plug's top this far under the surface while held
PUSH_TIP = 0.002  # m: the pad tips this far over the firmed surface in a stroke
PAD = 0.013  # pad tips past the tool point
STROKE_OUT, STROKE_IN = 0.100, 0.050  # m from the hole's center: start and end of a push stroke (pad tips); the pads'
# leading face then stops about 1 cm inside the hole's edge, so the mix drops into the gap, not onto the plug's top
STROKE_S = 2.6  # s per stroke
FIRM_DEPTH = 0.008  # m: the pad tips aimed this far into the backfill when firming (the clods give way under them)
FIRM_R = (0.037, 0.039)  # m: where the tips press, from the hole's center, on the x and y sides: the middle of the gap
# (the plug's faces are 2.45 and 2.85 cm out, the hole's wall 5 cm)
FIRM_R_CORNER = 0.048  # m: the four corner presses, just outside the plug's corner (3.76 cm out; the pads reach 1 cm in)


def fold(yaw: float) -> float:
    """The same two-finger grip turned into [-90, 90) degrees (the fingers are symmetric), the wrist's easy range."""
    return (yaw + math.pi / 2) % math.pi - math.pi / 2


def _stroke(plan, deg, label, out=STROKE_OUT, inn=STROKE_IN, lift=0.06, roll=0.0, pitch=0.0):
    """A push from `out` to `inn` (m from the hole's center, measured at the pad tips) along the direction `deg`.
    With the hand leaned (roll), the tips sit off the tool point, so the tool point's path is shifted to keep the tips
    on the line: otherwise a stroke from the +y side would stop 6 mm short of the hole's edge."""
    th = math.radians(deg)
    ux, uy = math.cos(th), math.sin(th)
    yaw = fold(th - math.pi / 2)  # the fingers close along the stroke: the pads' 4 cm width pushes the mix
    tip = kit._rot_xyz(roll, pitch) @ np.array([0.0, 0.0, -PAD])  # pad tips from the tool point
    tz = Z_S + PUSH_TIP
    s0 = np.array([POT[0] + out * ux, POT[1] + out * uy, tz]) - tip
    s1 = np.array([POT[0] + inn * ux, POT[1] + inn * uy, tz]) - tip
    P = lambda v, dz=0.0: (float(v[0]), float(v[1]), float(v[2] + dz))  # noqa: E731
    W = lambda d, p, lb: kit.Tilted(d, p, FINGER_CLOSED, lb, yaw, roll=roll, pitch=pitch)  # noqa: E731
    plan += [W(0.8, P(s0, lift), f"over {label}"),  # moving between strokes 6 cm up: clear of the plug's top
             W(0.5, P(s0), f"down {label}"),
             W(1.0, P(s1), f"push {label}"),
             W(0.1, P(s1, 0.015), f"up {label}"),
             W(0.2, P(s1 + 0.6 * (s0 - s1), lift), f"up and out {label}")]  # out as it rises: clear of the leaves


# The strokes after the plug is let go, as (hand, direction deg, lean phi deg), and the firming presses as (hand,
# direction, radius m, lean): the left hand works the +y half and the -x side, then the right hand the +x side and the
# rest of the presses. Each lean is the one (of every 30 degrees, per arm) that the arm reaches with its hand farthest
# from the stem.
# Consecutive strokes and presses go round the hole in steps of at most 45 degrees: moving between strokes on far sides
# of the hole, the hand would pass through the plant (its stem stands 9 cm over the surface).
_LEFT_LEAN = {20: 90, 40: 60, 60: 60, 65: 60, 75: 60, 90: 90, 105: 90, 115: 90, 120: 90, 140: 90, 160: 120, 180: 150}
SECOND = tuple(("left", d, _LEFT_LEAN[d]) for d in (20, 40, 65, 90, 115, 140, 180, 180, 160, 120, 105, 90, 75, 60, 90, 90))
SECOND_RIGHT = ((0, 270), (0, 270))
# no presses at -90 and -135 degrees, under the leaves: there the hand came down on them and tipped the plug 11-13
# degrees (simulator); that side was packed by the strokes made while the plug was held
PRESSES = (("left", 45, FIRM_R_CORNER, 60), ("left", 90, FIRM_R[1], 90), ("left", 135, FIRM_R_CORNER, 120),
           ("left", 180, FIRM_R[0], 180), ("right", 0, FIRM_R[0], 300), ("right", -45, FIRM_R_CORNER, 300))


def lean(phi: float) -> tuple:
    """(roll, pitch) that tilt the hand by LEAN with its upper parts toward the direction `phi` (deg, plan view)."""
    th = math.radians(phi)
    return -LEAN * math.sin(th), LEAN * math.cos(th)


def _press(plan, deg, r, phi):
    """Firming: the closed pad tips pressed FIRM_DEPTH into the backfill `r` from the hole's center along `deg`, the
    pads' 4 cm width along the plug's face (never on the plug's top)."""
    th = math.radians(deg)
    roll, pitch = lean(phi)
    tip = kit._rot_xyz(roll, pitch) @ np.array([0.0, 0.0, -PAD])
    p = np.array([POT[0] + r * math.cos(th), POT[1] + r * math.sin(th), Z_S - FIRM_DEPTH]) - tip
    yaw = fold(th - math.pi / 2)  # the fingers close along the radius: the pads lie along the plug's face
    P = lambda dz=0.0: (float(p[0]), float(p[1]), float(p[2] + dz))  # noqa: E731
    plan += [kit.Tilted(0.9, P(0.04), FINGER_CLOSED, f"over firm {deg}", yaw, roll=roll, pitch=pitch),
             kit.Tilted(0.7, P(), FINGER_CLOSED, f"firm {deg}", yaw, roll=roll, pitch=pitch),
             kit.Tilted(0.5, P(0.04), FINGER_CLOSED, f"lift firm {deg}", yaw, roll=roll, pitch=pitch)]


def demo():
    px, py = PLUG_START
    gz = TOP + PLUG_H - GRIP_BELOW_TOP
    set_z = Z_S - SET_DEPTH - GRIP_BELOW_TOP
    first = (-40, -65, -90, -115, -140, -105, -90, -75, -90)  # strokes while the plug is held: the -y half (the left fingers are
    # on the x sides, so strokes toward them stop 1.2 cm further out)
    hold_s = len(first) * STROKE_S + 1.5
    # carried over the heap of loose mix (the plug's bottom hangs 4.2 cm under the tool point), then lowered into the
    # hole from straight above it
    L = lambda d, p, f, lb: kit.Tilted(d, p, f, lb, YAW_PLUG, roll=-LEAN)  # noqa: E731
    U = lambda d, p, f, lb: kit.Tilted(d, p, f, lb, YAW_PLUG, roll=-LEAN, pitch=TILT_FIX)  # noqa: E731
    hx, hy = POT[0] - GRASP_SHIFT, POT[1]
    bx = px - APPROACH_SHIFT
    left = [L(2.0, (bx, py + PLANT_SIDE, gz + 0.14), FINGER_OPEN, "over the plug, beside the plant"),
            L(2.0, (bx, py + PLANT_SIDE, gz), FINGER_OPEN, "down beside the plant"),
            L(1.0, (bx, py, gz), FINGER_OPEN, "round the plug"),
            L(1.0, (bx, py, gz), FINGER_CLOSED, "pinch"),
            L(1.5, (bx, py, TOP + 0.160), FINGER_CLOSED, "lift"),
            L(2.5, (hx, hy, TOP + 0.160), FINGER_CLOSED, "over the hole"),
            L(2.5, (hx, hy, set_z), FINGER_CLOSED, "in the hole"),
            U(2.0, (hx, hy, set_z), FINGER_CLOSED, "stand it upright"),
            U(hold_s, (hx, hy, set_z), FINGER_CLOSED, "hold upright"),
            U(1.0, (hx, hy, set_z), RELEASE_OPEN, "let go"),
            U(1.0, (hx, hy + PLANT_SIDE, set_z), RELEASE_OPEN, "out beside the plant"),
            U(1.5, (hx, hy + PLANT_SIDE, set_z + 0.08), RELEASE_OPEN, "straight up"),
            U(1.0, (hx, hy + STROKE_OUT, set_z + 0.08), RELEASE_OPEN, "away from the plant")]
    t_held = sum(w.duration for w in left[:8])  # the plug is in the hole and the left wrist leans away
    right = [Waypoint(t_held + 0.5 - 1.5, (0.200, -0.180, TOP + 0.17), FINGER_CLOSED, "wait", 0.0)]
    for deg in first:
        inn = STROKE_IN if abs(math.sin(math.radians(deg))) > 0.9 else STROKE_IN + 0.008
        _stroke(right, deg, f"{deg} deg", inn=inn, roll=LEAN)
    right.append(Waypoint(1.0, (0.200, -0.180, TOP + 0.17), FINGER_CLOSED, "wait", 0.0))
    # after letting go: each hand works the side it can reach leaning away from the stem (LEAN toward `phi`), so its
    # finger mounts and palm stay outside the plant while the pad tips work the mix; one hand at a time
    plans = {"left": left, "right": right}
    for hand, deg, phi in SECOND:
        roll, pitch = lean(phi)
        _stroke(plans[hand], deg, f"{deg} deg", roll=roll, pitch=pitch)
    for hand, deg, r, phi in PRESSES:
        if hand == "right" and plans["left"][-1].label != "away":
            last, th = plans["left"][-1], math.radians(PRESSES[[h for h, *_ in PRESSES].index("right") - 1][1])
            # first straight out from the hole along the last press's direction, still leaned, clear of the plant
            plans["left"].append(kit.Tilted(1.0, (last.pos[0] + 0.06 * math.cos(th), last.pos[1] + 0.06 * math.sin(th),
                                                  TOP + 0.17), FINGER_CLOSED, "clear of the plant", last.yaw,
                                            roll=last.roll, pitch=last.pitch))
            plans["left"].append(Waypoint(2.0, (0.200, 0.200, TOP + 0.17), FINGER_OPEN, "away", YAW_PLUG))
            t_l, t_r = (sum(w.duration for w in plans[h]) for h in ("left", "right"))
            right.append(Waypoint(max(1.0, t_l + 0.5 - t_r), (0.200, -0.180, TOP + 0.17), FINGER_CLOSED, "wait", 0.0))
            for d2, phi2 in SECOND_RIGHT:
                roll, pitch = lean(phi2)
                _stroke(right, d2, f"{d2} deg", roll=roll, pitch=pitch)
        _press(plans[hand], deg, r, phi)
    right += [Waypoint(2.0, (0.200, -0.180, TOP + 0.17), FINGER_CLOSED, "away", 0.0),
              Waypoint(4.5, (0.200, -0.180, TOP + 0.17), FINGER_CLOSED, "wait for the push test", 0.0)]
    left.append(Waypoint(max(1.0, sum(w.duration for w in right) - sum(w.duration for w in left)), (0.200, 0.200, TOP + 0.17),
                         FINGER_OPEN, "wait", YAW_PLUG))
    return {"left": left, "right": right}
