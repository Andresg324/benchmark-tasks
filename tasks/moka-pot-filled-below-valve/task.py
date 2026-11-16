"""A 3-cup aluminum stovetop moka pot, taken apart on the kitchen counter: its empty water chamber (the base, with the
brass safety valve on its side) stands free on the counter, its upper chamber stands upright to the left of it, a glass
carafe of water stands to the right, and the funnel basket, already filled with ground coffee, stands in an espresso cup
behind it. Making moka coffee means filling the base with water up to just below the safety valve, never over it (the
valve could then not vent the steam if the funnel clogs) and not far under it (weak, under-extracted coffee), dropping
the filled basket into the base, and screwing the upper chamber onto the base's thread, straight, until the gasket is
pressed snug, while the other hand steadies the base, so steam cannot escape at the gasket. The pot then goes on the
stove (not part of this task).
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import pourkit as pk  # noqa: E402  (the water: volumes, pours, spills)
import threadkit as thk  # noqa: E402  (the screw thread as collider segments, and the gasket)
import twistkit as tk  # noqa: E402  (signed wrist-roll helpers and the wrist's yaw range)
import yamkit as kit  # noqa: E402  (modeled props, gripper states, tilted grips)
import guardkit as gk  # noqa: E402  (gripper contact: holding and letting go)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Get the moka pot ready to brew."
D = math.radians

# Real size throughout (a 3-cup moka pot is 15.9 cm tall assembled). The base stands free on the counter.
BX, BY = 0.27, 0.05  # toward the left arm: it can lift its open hand over the whole upper chamber here (see twist)
FLOOR_Z = TOP

# The base (free, 0.30 kg empty: an estimate for the cast-aluminum 3-cup base): an aluminum water chamber, octagonal
# outside (8.0 cm across the flats, as the Moka Express's eight-sided body), 7.2 cm tall, with a threaded collar 6.6 cm
# across on top. Heights are in its own frame (origin on its foot, on its axis). Inside it steps in three times (cast
# wall); the water is modeled in these three sections.
B_FOOT_R, B_COLLAR_R, RIM = 0.041, 0.033, 0.072
B_FLATS = 0.080
B_MASS = 0.30
B_FLOOR = 0.004
SECTIONS = ((0.036, 0.020), (0.033, 0.020), (0.031, 0.028))  # (inside radius, height) from the floor up
COLLAR_H = 0.012  # the thread on the collar's outside, under the upper chamber's skirt when screwed on
VALVE_Z = 0.050  # the safety valve's center; its bore inside the wall is 8 mm across
VALVE_BORE_R = 0.004
TOP_LINE_Z = VALVE_Z - VALVE_BORE_R  # 4.6 cm: water above the bore's bottom edge stands in the valve




def ml_below(z):
    """mL the base holds up to height z (its own frame)."""
    v, z0 = 0.0, B_FLOOR
    for r, h in SECTIONS:
        dz = min(max(z - z0, 0.0), h)
        v += math.pi * r * r * dz * 1e6
        z0 += h
    return v


VALVE_ML = ml_below(TOP_LINE_Z)  # 155.8 mL: up to the valve bore's bottom edge
LOW_ML = ml_below(TOP_LINE_Z - 0.010)  # 122.5 mL: 1 cm below it (a fifth less water: weak, under-extracted coffee)
DRY_ML = 30.0  # under this nothing worth brewing went in (do-nothing included)


# The upper chamber (free, 0.20 kg): a 7.0 cm wide aluminum pot with a lid, knob, spout and a black handle, 8.7 cm
# tall, its skirt (female thread) dropping 1.2 cm over the base's collar. Frame origin: the center of its bottom
# face, which rests on the basket's flange (through the gasket) when it is screwed on.
T_R, T_TOP_R, T_H = 0.035, 0.040, 0.087
SKIRT_IN, SKIRT_OUT = 0.036, 0.0425  # the skirt (female thread) round the base's collar
GRIP_DZ = 0.033  # grip height on its body, above its bottom face (high, clear of the hand holding the base)
T_WIDTH = 2 * T_R
HANDLE_ANG = D(45)  # the handle points 45 degrees off the fingers' closing axis at the start (fingers clear of it)

# The thread (threadkit, a helix of collider boxes on each part, so the top really screws on and a crooked start can
# ride or bind on the wrong groove): 66 mm major diameter, single start, right hand, 3.0 mm pitch, 1.5 turns on each
# part, 0.3 mm flank clearance. The base's ridge sits 1.5 mm higher than threadkit's default (5.0 mm under the rim)
# because the upper chamber seats on the basket's 1.5 mm flange, not on the rim.
TH_MALE = thk.Thread(male_top=-0.0050)
TH_FEMALE = thk.MOKA
# The gasket: a ring of compliant boxes standing 0.8 mm proud of the upper chamber's bottom face (threadkit.Gasket).
GASKET = thk.Gasket(k_point=1.0e4)  # per contact point: 2,500 let a 50 N load punch through (probe p7)
AL_AL = (1.1, 1.0)  # aluminum on aluminum, dry (handbook static 1.05-1.35): the threads, the base on the counter
# m/s: PhysX's push-apart speed for the upper chamber, whose thin thread ridges can overlap the base's on impact. Only
# the upper chamber is capped: with the base capped too, the basket's flange sank through the base's rim at the
# thread's rate under the gasket's ~20 N and sprang back when unloaded (d33a-d37).
MAX_DEPEN = 0.05
SEAT_T = 0.003  # the upper chamber's bottom face as a ring of boxes this thick

# The carafe (free): a 4.9 cm glass carafe 13.5 cm tall holding 175 mL. Pouring all of it in brings the base to 175
# mL, 19 mL over the valve: the pour has to stop.
JUG_XY = (0.23, -0.12)
JUG_R, JUG_H, JUG_IN, JUG_BASE = 0.0245, 0.135, 0.022, 0.004
JUG0 = 175.0
JUG_POS = (*JUG_XY, TOP + JUG_H / 2)
FOOT = 0.001
TOP_XY = (0.24, 0.19)

# The funnel basket (free, 40 g: 25 g of stainless steel and 15 g of ground coffee), a stainless replacement funnel for
# a 3-cup pot: a flange 6.12 cm across and 1.5 mm thick, a cup 6.1 cm across at the top tapering to 4.6 cm at its
# perforated floor 2.0 cm down, and a stem 1.0 cm across and 3.8 cm long. Frame origin: the center of the flange's
# bottom face, which rests on the base's rim when it is in (the upper chamber's gasket then presses on the flange).
# The coffee is level with the flange. The flange has to pass inside the upper chamber's female thread (its crests are
# 6.44 cm across), so it is narrower than that thread.
# The flange is 6.12 cm across: its edge (with the cup's 0.2-0.5 mm play in the bore) must stay well inside the upper
# chamber's female crests (6.44 cm): at 6.24 cm, drawn as 24 boxes whose corners stand 0.3 mm proud, the crests caught
# it and the screwing dragged the basket 8-14 mm down into the base (d25, d26).
K_FLANGE_R, K_FLANGE_T = 0.0306, 0.0015
K_TOP_R, K_FLOOR_R, K_DEPTH = 0.0305, 0.023, 0.020
K_STEM_R, K_STEM_L = 0.005, 0.038
# It starts standing in a ceramic espresso cup (5.4 cm across, 5.0 cm tall, 4.2 cm across inside its mouth): the
# basket's floor rests on the cup's rim and its stem hangs inside, the way it is set down to be filled.
CUP_XY, CUP_R, CUP_IN, CUP_H = (0.36, -0.07), 0.027, 0.021, 0.050
BASKET_POS = (*CUP_XY, TOP + CUP_H + K_DEPTH + 0.0005)
# The base's wall inside the collar, as 24 posts round the opening (the cup and stem go down inside them, the flange
# rests on their tops at the rim), kept inside the thread's root: center radius and post radius
# The wall's outside at the rim (6.26 cm across) stands 0.9 mm inside the female thread's crests, so a top set on 0.9 mm
# off the axis still goes down round it onto the thread.
# The posts are 2.6 mm thick so the flange rests on a 2.5 mm ledge (on 1.5 mm, a basket lying 1.5 mm off center
# tipped into the opening when the top came down, d12).
WALL_R, POST_R, N_POSTS = 0.0300, 0.0013, 24  # 1.9 mm round the cup's 5.35 cm band (at 1.1 mm it hung on the posts, d3)
# The mouth: the collar's inside is turned to a short lead-in over the top 2 mm (r 28.7 mm at 2 mm down to 29.0 mm at
# the rim), and outside it the rim is the boiler wall's flat top, 2.3 mm wide and 3 mm deep (29.0-31.3 mm, 48 boxes like
# the flange's ring), that the flange (r 30.6 mm) bears on with its outer 1.6 mm. As on the real pot, the funnel drops
# into the boiler's mouth: its cup's centering band (r 28.5 mm, see K_BAND_Z) stands in the bore with 0.2 mm play (at
# most 0.5 mm in the lead-in), so the flange's edge stays within 31.1 mm of the base's axis, 1.1 mm inside the upper
# chamber's female crests (32.2 mm), and its bearing on the rim never runs out.
MOUTH_R0, MOUTH_R1, MOUTH_H, MOUTH_T = 0.0287, 0.0290, 0.002, 0.0008
LEDGE_R, LEDGE_H, LEDGE_N = (0.0290, 0.0313), 0.003, 48
# The band reaches 6.5 mm down, well into the bore under the 2 mm lead-in (0.2 mm play there): only 1 mm of a 2.6 mm
# band was below the lead-in, and under the screwing top's gasket the basket crept 1.1-1.3 mm sideways, its flange's
# 1.0 mm bearing on the ledge ran out on one side and it sank 2.1 mm (d33a, d34).
K_BAND_R, K_BAND_Z = 0.0285, (-0.0065, -0.0004)  # the basket's centering band, in its frame
# The basket and the base's mouth are bare metal: METAL_MU (static, dynamic), a handbook dry value for steel on
# aluminum, instead of yam_arena's rubber-like default (1.2, 1.0). With the rubber value a basket dropped crooked wedged
# on the posts that model the mouth (sim runs); with this one it slides down flat.
METAL_MU = (0.6, 0.47)

ALU = kit.Look((0.78, 0.79, 0.80), 0.35, metallic=1.0)
DARK_ALU = kit.Look((0.45, 0.46, 0.48), 0.5, metallic=1.0)
BRASS = kit.Look((0.86, 0.66, 0.22), 0.3, metallic=1.0)
BLACK = kit.Look((0.06, 0.06, 0.07), 0.5)
COFFEE = kit.Look((0.20, 0.12, 0.07), 0.9)
CERAMIC = kit.Look((0.93, 0.92, 0.89), 0.25)
GLASS_LOOK = kit.Look((0.86, 0.92, 0.95), 0.05, opacity=0.28)
WATER_RGB = (0.25, 0.52, 0.95)


def _yaw_quat(u):
    return (0.0, 0.0, math.sin(u / 2), math.cos(u / 2))


def _mouth(n=24):
    """The lead-in (n oriented slabs whose inner faces make the cone) and the flat ledge, in the base's frame."""
    dr, dz = MOUTH_R1 - MOUTH_R0, MOUTH_H
    L = math.hypot(dr, dz)
    th = math.atan2(dr, dz)  # the face's lean from vertical
    nr, nz = math.cos(th), -math.sin(th)  # into the metal: outward and down
    rm, zm = (MOUTH_R0 + MOUTH_R1) / 2 + nr * MOUTH_T / 2, RIM - dz / 2 + nz * MOUTH_T / 2
    w = 2 * (MOUTH_R1 + MOUTH_T) * math.tan(math.pi / n) * 1.04
    qp = (0.0, math.sin(th / 2), 0.0, math.cos(th / 2))  # about the tangent: x -> the normal, z -> up the face
    out = []
    for k in range(n):
        u = 2 * math.pi * k / n
        qy = _yaw_quat(u)
        x1, y1, z1, w1 = qy
        x2, y2, z2, w2 = qp
        q = (w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
             w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2)
        out.append(kit.Collider((rm * math.cos(u), rm * math.sin(u), zm), size=(MOUTH_T, w, L), quat=q))
    out += thk.seat_ring(LEDGE_R[0], LEDGE_R[1], RIM - LEDGE_H, RIM, n=LEDGE_N)
    return out


def _base():
    z1 = B_FLOOR + SECTIONS[0][1]
    z2 = z1 + SECTIONS[1][1]
    wall_top = RIM - COLLAR_H - 0.002
    apo = B_FLATS / 2
    corner = apo / math.cos(math.pi / 8)
    # the octagonal body (a lathe with 8 sides, its flats facing +-x and +-y), the collar round, the inside stepped
    outer = [(0.0, 0.0), (corner, 0.0), (corner, wall_top), (0.0, wall_top)]
    inner = [(B_COLLAR_R, wall_top), (B_COLLAR_R, RIM), (SECTIONS[2][0], RIM), (SECTIONS[2][0], z2),
             (SECTIONS[1][0], z2), (SECTIONS[1][0], z1), (SECTIONS[0][0], z1), (SECTIONS[0][0], B_FLOOR),
             (0.0, B_FLOOR)]
    body = kit.lathe(outer, ALU, n=8)  # corners at 0, 45, ... degrees: turned 22.5 degrees, its flats face the axes
    c8, s8 = math.cos(math.pi / 8), math.sin(math.pi / 8)
    body = kit.Mesh(tuple((c8 * p[0] - s8 * p[1], s8 * p[0] + c8 * p[1], p[2]) for p in body.points), body.counts,
                    body.indices, body.look)
    meshes = [body, kit.lathe([(0.0, wall_top)] + inner, ALU),
              thk.ridge_mesh(TH_MALE, TH_MALE.major_r - TH_MALE.depth / 2,
                             RIM + TH_MALE.male_top - TH_MALE.pitch * TH_MALE.male_turns, TH_MALE.male_turns, DARK_ALU),
              # the safety valve on the side toward the robot (-x): a brass hex body and a pin
              kit.cylinder((-apo - 0.004, 0, VALVE_Z), 0.0065, 0.008, BRASS, axis="x", n=6),
              kit.cylinder((-apo - 0.009, 0, VALVE_Z), 0.002, 0.004, BRASS, axis="x"),
              # its bore inside the wall, a brass ring you see when you look into the base
              kit.cylinder((-SECTIONS[2][0] + 0.0004, 0, VALVE_Z), VALVE_BORE_R, 0.001, BRASS, axis="x")]
    # a solid bottom 1 cm thick (the stem's end stops 2.5 mm over it), the eight flat sides as oriented boxes (the
    # steadying hand bears on two of them), the wall inside the collar as posts up to the rim, and the male thread
    cols = [kit.Collider((0, 0, 0.005), radius=0.039, height=0.010)]
    side = 2 * apo * math.tan(math.pi / 8) * 1.02
    for k in range(8):
        u = k * math.pi / 4
        # 8 mm thick (inside them only water, modeled apart): a hand's pads pressing on 4 mm walls went through
        # them and pushed the basket's cup up from inside (d17-d22)
        cols.append(kit.Collider(((apo - 0.004) * math.cos(u), (apo - 0.004) * math.sin(u), (0.010 + wall_top) / 2),
                                 size=(0.008, side, wall_top - 0.010), quat=_yaw_quat(u)))
    # the wall inside the collar as a closed ring of boxes (24 posts left 5 mm gaps a crooked flange's edge slipped
    # through, letting the basket drop 14 mm into the base, d24)
    cols += thk.seat_ring(WALL_R - POST_R, WALL_R + POST_R, 0.010, RIM - MOUTH_H, n=N_POSTS)
    cols += _mouth()
    cols += thk.colliders(thk.male(TH_MALE, rim_z=RIM))
    return kit.Prop("base", pos=(BX, BY, FLOOR_Z + 0.0005), size=(2 * B_FOOT_R, 2 * B_FOOT_R, RIM), mass=B_MASS,
                    meshes=tuple(meshes), colliders=tuple(cols), friction=AL_AL)


def _top():
    prof = [(0.0, 0.0), (SKIRT_IN, 0.0), (SKIRT_IN, -COLLAR_H), (SKIRT_OUT, -COLLAR_H), (SKIRT_OUT, 0.0),
            (T_R, 0.006), (T_R, 0.040), (0.039, 0.046),
            (T_TOP_R, T_H - 0.004), (0.036, T_H), (0.0, T_H)]
    c, s = math.cos(HANDLE_ANG), math.sin(HANDLE_ANG)
    meshes = [kit.lathe(prof, ALU),
              kit.cylinder((0, 0, T_H + 0.005), 0.008, 0.010, BLACK),  # lid knob
              kit.box((-c * 0.044, -s * 0.044, T_H - 0.008), (0.012, 0.012, 0.010), ALU)]  # spout (opposite the handle)
    cols = [kit.Collider((0, 0, (SEAT_T + 0.040) / 2), radius=T_R, height=0.040 - SEAT_T),
            kit.Collider((0, 0, (0.040 + T_H) / 2), radius=0.039, height=T_H - 0.040),
            kit.Collider((0, 0, T_H + 0.005), radius=0.008, height=0.010),
            # its bottom face as a ring of boxes (a flat box ring holds a screwing load; cylinder faces sank, probe p2)
            *thk.seat_ring(0.026, T_R, 0.0, SEAT_T, n=24),
            # the skirt's wall round the female thread
            *thk.seat_ring(TH_FEMALE.female_root_r + 0.0002, SKIRT_OUT, -COLLAR_H, 0.0, n=24),
            *thk.colliders(thk.female(TH_FEMALE)),
            *GASKET.colliders()]
    for k in range(3):  # the handle: three short blocks stepping out along its direction
        r = 0.046 + 0.010 * k
        p = (c * r, s * r, 0.060 - 0.012 * k)
        meshes.append(kit.box(p, (0.012, 0.012, 0.022), BLACK))
        cols.append(kit.Collider(p, size=(0.012, 0.012, 0.022)))
    return kit.Prop("top", pos=(*TOP_XY, TOP + COLLAR_H + 0.0005), size=(0.11, 0.11, T_H + COLLAR_H + 0.01),
                    mass=0.20, meshes=tuple(meshes), colliders=tuple(cols), friction=AL_AL,
                    max_depenetration=MAX_DEPEN)


def _jug():
    z0 = -JUG_H / 2
    shell = kit.lathe([(0.0, z0), (JUG_R, z0), (JUG_R, -z0), (JUG_IN, -z0), (JUG_IN, z0 + JUG_BASE),
                       (0.0, z0 + JUG_BASE)], GLASS_LOOK)
    return kit.Prop("jug", pos=JUG_POS, size=(2 * JUG_R, 2 * JUG_R, JUG_H), mass=0.20, meshes=(shell,),
                    colliders=(kit.Collider((0, 0, FOOT / 2), radius=JUG_R, height=JUG_H - FOOT),
                               kit.Collider((0, 0, z0 + 0.002), size=(0.032, 0.032, 0.004))))


def _basket():
    f, d = K_FLANGE_T, K_DEPTH
    prof = [(0.0, -d - K_STEM_L), (K_STEM_R, -d - K_STEM_L), (K_STEM_R, -d), (K_FLOOR_R, -d), (K_TOP_R, 0.0),
            (K_FLANGE_R, 0.0), (K_FLANGE_R, f), (0.029, f), (0.029, f - 0.001), (0.0, f - 0.001)]
    meshes = [kit.lathe(prof, ALU),
              kit.cylinder((0, 0, f - 0.0008), 0.0288, 0.0012, COFFEE)]  # the ground coffee, level with the flange
    # the tapered cup as two straight bands (its mean width over each), so a gripper's pads bear on a flat face
    # the flange's bearing ring as boxes (the gasket and the rim are box rings too): as a cylinder it sank through
    # the rim under the screwing load (d25), as cylinder faces did in the thread probes (threadkit.seat_ring)
    cols = [kit.Collider((0, 0, f / 2), radius=0.0255, height=f),
            *thk.seat_ring(0.0255, K_FLANGE_R, 0.0, f, n=48),  # 48 boxes: corners 0.07 mm proud
            kit.Collider((0, 0, -0.007), radius=0.0268, height=0.014),
            kit.Collider((0, 0, sum(K_BAND_Z) / 2), radius=K_BAND_R, height=K_BAND_Z[1] - K_BAND_Z[0]),
            kit.Collider((0, 0, -0.017), radius=0.0235, height=0.006),
            kit.Collider((0, 0, -d - K_STEM_L / 2), radius=K_STEM_R, height=K_STEM_L)]
    return kit.Prop("basket", pos=BASKET_POS, size=(2 * K_FLANGE_R, 2 * K_FLANGE_R, d + K_STEM_L + f), mass=0.04,
                    meshes=tuple(meshes), colliders=tuple(cols), friction=METAL_MU)


def _cup():
    h = CUP_H
    prof = [(0.0, 0.0), (0.022, 0.0), (CUP_R, 0.006), (CUP_R, h), (CUP_IN, h), (CUP_IN, 0.008), (0.0, 0.008)]
    cols = [kit.Collider((0, 0, 0.004), radius=CUP_R, height=0.008)]
    for k in range(12):  # its wall as posts, so the stem hangs inside
        u = 2 * math.pi * k / 12
        cols.append(kit.Collider((0.024 * math.cos(u), 0.024 * math.sin(u), (0.008 + h) / 2), radius=0.003,
                                 height=h - 0.008))
    return kit.Prop("cup", pos=(*CUP_XY, TOP), size=(2 * CUP_R, 2 * CUP_R, h), meshes=(kit.lathe(prof, CERAMIC),),
                    colliders=tuple(cols))


OBJECTS = [_base(), _top(), _jug(), _basket(), _cup()]


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _yaw(q):
    r = _rot(q)
    return math.atan2(r[1, 0], r[0, 0])


def _rim(objects):
    b = objects["base"]
    return np.asarray(b.pos, float) + _rot(b.quat_xyzw) @ np.array([0.0, 0.0, RIM])



# On the base, threaded straight: the upper chamber's bottom face within 4 mm of the rim's height, its axis within 2 mm
# of the base's and within 1.5 degrees of it. Down in the right groove the thread lets it wobble about 1 degree (its
# 0.3 mm flank clearance); a top that started crooked and rides or binds on the wrong groove stands at least one
# groove off on one side, 2.6 degrees over the 66 mm thread (cross-threaded), or rides up on the crests.
ON_DZ, ON_DXY, ON_TILT = 0.004, 0.002, 1.5


def _top_rel(objects):
    """(offset of the top's origin from the rim, in the base's frame; tilt between their axes in degrees)."""
    t, b = objects["top"], objects["base"]
    rb = _rot(b.quat_xyzw)
    d = rb.T @ (np.asarray(t.pos, float) - _rim(objects))
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(t.quat_xyzw)[:, 2] @ rb[:, 2])))))
    return d, tilt


def top_on(objects):
    d, tilt = _top_rel(objects)
    return abs(d[2]) < ON_DZ and math.hypot(d[0], d[1]) < ON_DXY and tilt < ON_TILT


def _lets_water_in(objects):
    """The base's mouth is open unless the upper chamber sits over it (within 3 cm above the rim and 5 cm across), or
    the basket is in it or just over it: a stream then lands on the grounds and runs off over the rim (spilled)."""
    rim = _rim(objects)
    d = np.asarray(objects["top"].pos, float) - rim
    k = np.asarray(objects["basket"].pos, float) - rim
    return not ((math.hypot(d[0], d[1]) < 0.05 and -0.005 < d[2] < 0.03)
                or (math.hypot(k[0], k[1]) < 0.035 and -0.01 < k[2] < 0.04))


# The basket is in: its flange's bottom within 2 mm of the rim (resting on it, not propped on something), its axis
# within 4 mm of the base's (the modeled cup, a band 5.35 cm across, can sit 1.9 mm off center in the 5.74 cm opening)
# and within 5 degrees of it.
SEAT_DZ, SEAT_DXY, SEAT_TILT = 0.002, 0.004, 5.0
COFFEE_TIP = 45.0  # degrees: tipped further, the grounds slide out of a 2 cm deep cup filled to its brim


def _tilt_deg(q, ref=(0.0, 0.0, 1.0)):
    return math.degrees(math.acos(max(-1.0, min(1.0, float(_rot(q)[:, 2] @ np.asarray(ref, float))))))


def basket_seated(objects):
    k, b = objects["basket"], objects["base"]
    d = np.asarray(k.pos, float) - _rim(objects)
    return (abs(d[2]) < SEAT_DZ and math.hypot(d[0], d[1]) < SEAT_DXY
            and _tilt_deg(k.quat_xyzw, _rot(b.quat_xyzw)[:, 2]) < SEAT_TILT)


pk.setup([pk.Vessel("base", sections=SECTIONS, floor=B_FLOOR, fill={}, accepts=_lets_water_in),
          pk.Vessel("jug", sections=((JUG_IN, JUG_H - JUG_BASE),), floor=-JUG_H / 2 + JUG_BASE, fill={"water": JUG0})],
         liquids={"water": WATER_RGB}, splash_drop=0.10, splash=0.25)
pk.set_floor(TOP)

# Scoring
SPILL_MAX = 5.0  # mL, a teaspoon on the counter
UPRIGHT = 10.0  # degrees
GRIPPERS = ("left_gripper", "right_gripper")
HOLD_MIN = 0.45  # the pads squeeze the 7 cm body to a reading of 0.55-0.65 in the sim (0.74 nominal)
# Snug: the gasket pressed at least C_SNUG (m) between the upper chamber's face and the basket's flange: the clamp that
# SNUG_NM of screwing torque gives through the modeled thread, over the gasket's estimated stiffness. Firm hand-tight
# is taken as 2.5 N*m for a 65 mm closure from published jar and closure data (see README). Through the modeled thread
# (square ridges, mean radius 32.4 mm, lead 0.84 degrees, aluminum friction 1.0) and the gasket (friction 0.5 at 28.5
# mm) 2.5 N*m clamps about 52 N, and at the gasket's estimated 5e5 N/m that is 0.10 mm. A top screwed down without
# that torque, or cross-threaded, does not press it that far.
SNUG_NM = 2.5
SNUG_N = 52.0
C_SNUG = SNUG_N / GASKET.k  # 0.10 mm


# Contact is judged with the gripper's own shape (palm and both fingers at their present opening, guardkit boxes fitted
# to the simulator's collision meshes) against these shapes in each object's frame: within 2 mm counts as touching
# (the solver's resting gap plus the box fit), so a hand hovering near a part is not on it.
TOP_SHAPE = (gk.Ring(0.0, T_R, 0.0, 0.040), gk.Ring(0.0, T_TOP_R, 0.040, T_H),  # the lower and upper body
             gk.Ring(SKIRT_IN, SKIRT_OUT, -COLLAR_H, 0.0),  # the skirt
             gk.Seg((0.0, 0.0, T_H), (0.0, 0.0, T_H + 0.010), 0.008),  # the lid knob
             *[gk.Block((math.cos(HANDLE_ANG) * (0.046 + 0.010 * k), math.sin(HANDLE_ANG) * (0.046 + 0.010 * k),
                         0.060 - 0.012 * k), (0.012, 0.012, 0.022)) for k in range(3)])  # the handle
BASE_SHAPE = (gk.Ring(0.0, B_FLATS / 2, 0.0, RIM - COLLAR_H), gk.Ring(0.0, B_COLLAR_R, RIM - COLLAR_H, RIM))
JUG_SHAPE = gk.Ring(0.0, JUG_R, -JUG_H / 2, JUG_H / 2)


def top_held_by(objects, g):
    """Fingers closed on the upper chamber: the gripper touching it (guardkit contact) with an opening between
    HOLD_MIN and its 7 cm width + 3 %, i.e. stopped by its body (an empty closed hand reads 0, an open one 1)."""
    if g not in objects:
        return False
    t = objects["top"]
    r = _rot(t.quat_xyzw)
    along = float((np.asarray(objects[g].pos, float) - np.asarray(t.pos, float)) @ r[:, 2])
    want = (T_WIDTH if along <= 0.040 else 2 * T_TOP_R) / (2 * abs(FINGER_OPEN))
    op = kit.opening(objects, g)
    return HOLD_MIN <= op <= want + tk.HOLD_OVER and bool(gk.touching(objects, "top", TOP_SHAPE, grippers=(g,)))


def top_held(objects):
    return any(top_held_by(objects, g) for g in GRIPPERS)


# --- What happened during the run (success() and score() run after every control step; pourkit has already moved the
# water). yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.on = False  # the upper chamber's skirt was round the collar at the last step
        self.under = False  # the basket was in the base when the upper chamber last went on
        self.tipped = 0  # consecutive steps the basket has been tipped past COFFEE_TIP
        self.coffee_lost = None  # where the grounds fell out (latched), else None


_H = _History()


def _engaged(objects):
    """The skirt round the collar: the top's bottom face less than the skirt's 1.2 cm above the rim, its axis within
    6 mm and 12 degrees of the base's (a crooked start counts too)."""
    d, tilt = _top_rel(objects)
    return -ON_DZ < d[2] < COLLAR_H and math.hypot(d[0], d[1]) < 0.006 and tilt < 12.0


def _watch_coffee(objects):
    """The grounds fall out if the basket is tipped past 45 degrees for two steps running (one step is a physics pop);
    latched, with where they fell."""
    k = objects["basket"]
    _H.tipped = _H.tipped + 1 if _tilt_deg(k.quat_xyzw) > COFFEE_TIP else 0
    if _H.tipped >= 2 and _H.coffee_lost is None:
        _H.coffee_lost = (float(k.pos[0]), float(k.pos[1]))


def _update(objects):
    global _H
    if _H.episode != kit.episode():
        _H = _History()
    _watch_coffee(objects)
    eng = _engaged(objects)
    if eng and not _H.on:  # it just went on: was the basket already in under it?
        _H.under = basket_seated(objects) and _H.coffee_lost is None
    _H.on = eng


def basket_ok(objects):
    """The basket in the base with its coffee still in it."""
    return basket_seated(objects) and _H.coffee_lost is None


def gasket_compression(objects):
    """How far (m) the gasket is pressed: its 0.8 mm standing proud of the upper chamber's face, less the gap between
    that face and the top of the basket's flange, along the base's axis."""
    b, k, t = objects["base"], objects["basket"], objects["top"]
    ax = _rot(b.quat_xyzw)[:, 2]
    flange_top = np.asarray(k.pos, float) + _rot(k.quat_xyzw)[:, 2] * K_FLANGE_T
    return GASKET.protrude - float((np.asarray(t.pos, float) - flange_top) @ ax)


def screwed_snug(objects):
    """Threaded on straight (top_on), put on after the basket went in, and the gasket pressed at least C_SNUG."""
    return top_on(objects) and _H.under and basket_ok(objects) and gasket_compression(objects) >= C_SNUG


def _jug_held(objects):
    """A gripper still touching the carafe (guardkit contact, within 2 mm)."""
    return bool(gk.touching(objects, "jug", JUG_SHAPE))


def _put_away(objects):
    """The carafe upright and let go; the base still standing upright; no gripper touching the pot."""
    jug_ok = pk.tilt(objects, "jug") < UPRIGHT and not _jug_held(objects)
    return (jug_ok and _tilt_deg(objects["base"].quat_xyzw) < UPRIGHT and not top_held(objects)
            and not _hand_on_pot(objects))


def _hand_on_pot(objects):
    """A gripper (palm or either finger) still touching the upper chamber or the base (guardkit contact, 2 mm)."""
    return bool(gk.touching(objects, "top", TOP_SHAPE)) or bool(gk.touching(objects, "base", BASE_SHAPE))


# The grounds, once fallen out, are drawn for the cameras: a disc of bare steel over the coffee in the basket and a
# brown patch where they fell (Isaac Lab visualization markers, drawn after pourkit's water; no physics).
_CV = {"markers": None, "failed": False}


def _coffee_markers():
    if _CV["markers"] is None and not _CV["failed"]:
        try:
            import isaaclab.sim as sim_utils
            from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

            def disc(look):
                mat = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in look.color),
                                                  roughness=look.roughness, metallic=look.metallic)
                return sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=mat)
            _CV["markers"] = VisualizationMarkers(VisualizationMarkersCfg(
                prim_path="/World/Visuals/moka_coffee", markers={"bare": disc(ALU), "grounds": disc(COFFEE)}))
        except Exception as e:  # noqa: BLE001  (visuals are optional: scoring never depends on them)
            print(f"moka: no coffee visuals ({e!r})", file=sys.stderr, flush=True)
            _CV["failed"] = True
    return _CV["markers"]


def _draw_coffee(objects, origin):
    lost = _H.coffee_lost if _H.episode == kit.episode() else None
    if lost is None and _CV["markers"] is None:
        return  # nothing to draw yet (markers are made on the first spill, so the first frames are untouched)
    m = _coffee_markers()
    if m is None:
        return
    hide = ((0.0, 0.0, -10.0), (0, 0, 0, 1), (1e-5, 1e-5, 1e-5))
    if lost is None:
        items = [hide, hide]
    else:
        k = objects["basket"]
        top = np.asarray(k.pos, float) + _rot(k.quat_xyzw) @ np.array([0.0, 0.0, K_FLANGE_T - 0.0001])
        z = TOP + 0.0005
        items = [(tuple(top), tuple(float(q) for q in k.quat_xyzw), (0.0292, 0.0292, 0.0006)),
                 ((lost[0], lost[1], z), (0, 0, 0, 1), (0.035, 0.028, 0.001))]
    m.visualize(translations=np.array([np.asarray(p, float) + origin for p, _, _ in items]),
                orientations=np.array([q for _, q, _ in items], float),
                scales=np.array([sc for _, _, sc in items], float), marker_indices=np.array([0, 1]))


_pk_draw = pk._draw


def _draw_all(objects, origin):
    _pk_draw(objects, origin)
    try:
        _draw_coffee(objects, origin)
    except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
        if not _CV["failed"]:
            print(f"moka: coffee visuals off ({e!r})", file=sys.stderr, flush=True)
        _CV["failed"] = True


pk._draw = _draw_all


def _job(objects):
    water = pk.volume(objects, "base")
    if water > VALVE_ML or water < DRY_ML:
        return 0.0
    basket, snug = basket_ok(objects), screwed_snug(objects)
    if water < LOW_ML:  # weak coffee: some credit only if the pot is otherwise put together
        return 0.25 if basket and snug else 0.0
    if not basket:
        return 0.25
    if not snug:
        return 0.5
    if pk.spilled(objects) > SPILL_MAX or not _put_away(objects):
        return 0.75
    return 1.0


def success(objects):
    """The job done; where the arms end up is not scored. Called first each step, so it also records the history."""
    _update(objects)
    _see(objects)
    return _job(objects) == 1.0


def score(objects):
    """0 with more than 155.8 mL in the base at the end (water above the bottom of the safety valve's bore) or under 30
    mL (nothing to brew; includes doing nothing), and with 30 to 122.5 mL unless the pot is otherwise ready (then
    0.25: weak coffee). With the base filled between the lines: 0.25 if the basket is not in it with its coffee
    (flange within 2 mm of the rim, axis within 4 mm and 5 degrees; tipped past 45 degrees on the way, the grounds
    fell out); 0.5 if the upper chamber is not screwed snug over it (put on after the basket went in, threaded on
    straight: face within 4 mm of the rim, axis within 2 mm and 1.5 degrees of the base's, and the gasket pressed at
    least 0.10 mm, what 2.5 N·m of screwing gives); 0.75 with more than 5 mL spilled, the carafe not upright and let
    go, the base tipped over 10 degrees, or a gripper still touching the pot. 1.0 otherwise. Possible scores: 0, 0.25,
    0.5, 0.75, 1."""
    return _job(objects)


# --- Demo
YAW, ROLL = math.pi / 2, 1.35  # the right hand comes in from -y, fingers closing along x, tipped about 13 degrees down
GRIP_Z = 0.065
LIP_ABOVE = 0.065  # the lip 6.5 cm over the rim: at 5 cm the tipped carafe pushed the free base 3 mm (d1)
POUR_PITCH = 1.40  # rad: tuned in the sim so the base ends between the lines (see README)
STROKE = 90.0  # the first turn, made holding the top as it was set on (the regrip strokes then run to the wrist's end)


def pour_plan(pitch=POUR_PITCH):
    sx, sy, _ = JUG_POS
    gz = TOP + GRIP_Z
    tcp = np.array([sx, sy, gz])
    lip0 = pk.lip("jug", JUG_POS, toward=(1, 0, 0))
    over = (BX - 0.008, BY, FLOOR_Z + RIM + LIP_ABOVE)
    t = lambda d, p, f, label: kit.Tilted(d, p, f, label, yaw=YAW, roll=ROLL)  # noqa: E731
    plan = [t(1.5, (sx, sy - 0.09, gz + 0.03), FINGER_OPEN, "beside the carafe"),
            t(1.0, (sx, sy - 0.09, gz), FINGER_OPEN, "down"),
            t(1.2, tuple(tcp), FINGER_OPEN, "around the carafe"),
            t(0.8, tuple(tcp), FINGER_CLOSED, "grip"),
            t(1.2, (sx, sy, gz + 0.10), FINGER_CLOSED, "lift")]
    plan += pk.pour(tcp, lip0, over, roll=ROLL, yaw=YAW, start_pitch=0.85, to_pitch=pitch, above=0.05, steps=8,
                    duration=6.0, hold=2.0, back=3.0, finger=FINGER_CLOSED)
    plan += [t(1.5, (sx, sy, gz + 0.10), FINGER_CLOSED, "back over its spot"),
             t(1.2, (sx, sy, gz + 0.003), FINGER_CLOSED, "set down"),
             t(1.0, (sx, sy, gz + 0.003), FINGER_OPEN, "let go"),
             t(1.0, (sx, sy - 0.06, gz + 0.016), FINGER_OPEN, "back out"),
             t(1.0, (sx, sy - 0.10, gz + 0.08), FINGER_OPEN, "up and away")]
    return plan


# The basket: gripped straight down across its cup (tool point 7.5 mm under the flange, the pad tips level with the
# cup's floor), carried over the base and let go with the flange 3.2 cm over the rim, the cup's floor 1.2 cm over it
# and its stem already inside; the tapered cup centers itself as it drops.
K_GRIP_DZ = -0.0075
K_RELEASE = 0.032
# The fingers close along the carry from the cup to the base (toward -x and +y, 45 degrees off each): a pinched basket
# swings about the line through the two pads, so a carry across that line tipped it 13 degrees in the grip (sim runs).
K_YAW = math.pi / 4
K_TIP = 0.0128  # the closed pad tips stand this far past the tool point (guardkit's gripper shape)
K_PRESS = 0.003  # m commanded past the seated basket's top: the arm's give turns it into a few newtons
# In the grip the basket sits 1.5 cm off the tool point along the fingers' line: the first finger to touch it pushes it
# along until its stem meets the espresso cup's wall, and the two pads then hold it wherever it is. Opened all at once,
# the fingers flick it back toward the tool point and it lands crooked on the rim (sim runs). So the tool point is
# carried over the base's axis and the grip is eased in two steps: the basket slides back between the pads to the axis,
# still held, and then drops in level.


def basket_plan():
    kx, ky, kz = BASKET_POS
    gz = kz + K_GRIP_DZ
    rz = FLOOR_Z + RIM + K_RELEASE + K_GRIP_DZ
    travel = TOP + 0.20
    y = K_YAW
    press_z = FLOOR_Z + RIM + K_FLANGE_T + K_TIP - K_PRESS  # tool point: the pad tips K_PRESS into a seated basket
    return [Waypoint(2.0, (kx, ky, travel), FINGER_OPEN, "over the basket", y),
            Waypoint(1.5, (kx, ky, gz + 0.03), FINGER_OPEN, "down to it", y),
            Waypoint(1.0, (kx, ky, gz), FINGER_OPEN, "around the cup", y),
            Waypoint(1.0, (kx, ky, gz), FINGER_CLOSED, "grip the basket", y),
            Waypoint(1.5, (kx, ky, rz + 0.06), FINGER_CLOSED, "lift it out of the cup", y),
            Waypoint(3.5, (BX, BY, rz + 0.06), FINGER_CLOSED, "over the base", y),
            Waypoint(1.0, (BX, BY, rz + 0.06), FINGER_CLOSED, "let it stop swinging", y),
            Waypoint(2.0, (BX, BY, rz), FINGER_CLOSED, "stem into the base", y),
            Waypoint(0.8, (BX, BY, rz), FINGER_CLOSED, "steady", y),
            Waypoint(1.0, (BX, BY, rz), 0.35 * FINGER_OPEN, "ease the grip", y),
            Waypoint(1.0, (BX, BY, rz), 0.45 * FINGER_OPEN, "let it slide over the axis", y),
            Waypoint(0.6, (BX, BY, rz), FINGER_OPEN, "let it drop in", y),
            Waypoint(1.0, (BX, BY, rz + 0.06), FINGER_OPEN, "up off it", y),
            # then press it home with the closed fingertips on its middle, as a cook pushes the funnel down: a basket
            # that dropped a few degrees crooked can hang with its flange's low edge on the mouth's lead-in, 6 mm up
            # and 13 degrees tipped (d16, d20)
            Waypoint(0.8, (BX, BY, rz + 0.06), FINGER_CLOSED, "close the hand over it", y),
            Waypoint(1.5, (BX, BY, press_z + 0.015), FINGER_CLOSED, "down to the coffee", y),
            Waypoint(1.0, (BX, BY, press_z), FINGER_CLOSED, "press the basket home", y),
            Waypoint(0.5, (BX, BY, press_z), FINGER_CLOSED, "hold", y),
            Waypoint(1.0, (BX, BY, rz + 0.06), FINGER_CLOSED, "up off it", y)]



# The right hand then steadies the base, as a cook's other hand does, once the upper chamber sits on the thread: it
# comes in from the robot's side (-x), its fingers pointing forward (+x) and closing along y across the base's two y
# flats, and holds it while the left arm screws the upper chamber on, so the screwing torque does not turn the base on
# the counter. Sim runs that shaped this: held from -y instead, the hand's palm led the sideways approach and shoved the
# free base 3.9 cm (d2), and its wrist and wrist camera stood beside the pot where the left hand brings the upper chamber
# down, so the arms touched and the top landed crooked (d4, d5). From -x the right wrist stays low and behind the pot,
# out of the left hand's way. The hand is turned to the side well behind the pot (beside it, it knocked the carafe over,
# d3), waits there while the left hand sets the upper chamber on, and slides in fingertips first. Only one finger of
# the YAM gripper closes: the other pad stays about 4.45 cm from the tool point (at yaw 0 on the -y side). The hand
# slides in with that pad 5 mm off the -y flat (with 1 mm it caught the base's corner and shoved it 8 mm, d6), then
# steps sideways until it stands 1 mm off, and the moving finger presses the base onto it.
# The fingers close only to a few mm inside the base's 8.0 cm across the flats (a firm hand's hold, not a crush):
# commanded fully shut, they met the flats at opening 0.86 and kept going to 0.04, through the base's walls (d17-d22).
STEADY_GRIP = 0.78 * FINGER_OPEN
BASE_GZ = 0.035  # grip height on the base, above the counter (low, so the hand stays under the left hand's fingers)
FIXED_PAD = 0.0445  # the gripper's fixed pad face, from the tool point along -y at yaw 0
STEADY_Y = BY - (B_FLATS / 2 + 0.001) + FIXED_PAD
SLIDE_Y = BY - (B_FLATS / 2 + 0.005) + FIXED_PAD
# The YAM's fingers are plates 6.8 cm wide (here standing up), so their bodies reach above the base's shoulder beside
# the upper chamber's skirt. The tool point stands 3 mm short of the base's axis, so the finger bodies stay at least
# 44.6 mm from the axis, outside the skirt (42.5 mm): with it 1 cm past the axis the fingers squeezed the skirt as
# well and the top could not turn freely (d9, d10).
STEADY_X = BX - 0.003
PITCH_IN = -1.50  # the hand tipped forward 86 degrees: fingers pointing +x, nearly level
BACK = 0.13  # where the hand turns and comes in from, behind the pot (-x)


def _steady_t(d, p, f, label):
    return kit.Tilted(d, p, f, label, yaw=0.0, pitch=PITCH_IN)


def wait_plan():
    """Out of the way behind the pot, turned to the side and down at the base's height, while the top is set on."""
    gz = FLOOR_Z + BASE_GZ
    return [Waypoint(1.5, (BX - BACK, SLIDE_Y, TOP + 0.14), FINGER_OPEN, "back behind the pot", 0.0),
            _steady_t(2.0, (BX - BACK, SLIDE_Y, gz + 0.03), FINGER_OPEN, "turn the hand forward, behind the pot"),
            _steady_t(1.2, (BX - BACK, SLIDE_Y, gz), FINGER_OPEN, "down to the base's height")]


def steady_plan():
    gz = FLOOR_Z + BASE_GZ
    return [_steady_t(2.0, (STEADY_X, SLIDE_Y, gz), FINGER_OPEN, "slide in, fingertips first, beside its flats"),
            _steady_t(1.0, (STEADY_X, STEADY_Y, gz), FINGER_OPEN, "the fixed pad up to the flat"),
            # closed slowly: shut in 0.8 s the grip jolted the free base (1.2 mm up, 2.9 deg) and popped the basket
            # 5 mm out of its seat (d21)
            _steady_t(2.0, (STEADY_X, STEADY_Y, gz), STEADY_GRIP, "steady the base")]


def release_plan():
    gz = FLOOR_Z + BASE_GZ
    t = lambda d, p, f, label: kit.Tilted(d, p, f, label, yaw=0.0, pitch=PITCH_IN)  # noqa: E731
    return [t(0.8, (STEADY_X, STEADY_Y, gz), FINGER_OPEN, "let go of the base"),
            t(0.8, (STEADY_X, SLIDE_Y, gz), FINGER_OPEN, "off the flat"),
            t(1.5, (BX - BACK, SLIDE_Y, gz), FINGER_OPEN, "back out"),
            t(1.0, (BX - BACK, SLIDE_Y, gz + 0.10), FINGER_OPEN, "up and away"),
            kit.Home(2.0), kit.Home(1.5, label="hold at home")]


def right_plan():
    """Pour, set the carafe down, put the basket in and wait behind the pot; returns the plan and when the hand has
    left the pot (s)."""
    plan = pour_plan() + basket_plan()
    plan += wait_plan()
    return plan, sum(w.duration for w in plan[:-2])


# Measured in the sim (demo runs): with a straight-down grip over the base the left arm's tool point settles about
# (-2.3, 0.9) mm off its command, and a gripped top standing on the thread has its axis (1.5, -9.7) mm off the tool
# point at wrist yaw 0 (turning with the wrist). Commands are offset so the top's axis, not the tool point, is over the
# base and the wrist turns about that axis.
# The regrip strokes grip lower than the carry: the finger pads' inner faces run from 13 mm under the tool point to
# 9 mm over it (yam.urdf tip meshes), and the body steps out from r 35 to r 39 mm at 40 mm up. Gripped 33 mm up, the
# pads' top 2 mm met that step's underside: closing, the finger lifted and tipped the top 1 mm / 2 deg off its groove,
# then the turn drove it a whole pitch down through the ridges and pushed the basket 2-5 mm into the base (d30).
# At 26 mm the pads (13-35 mm up the body) stay 5 mm under the step.
GRIP_TW = 0.026
ARM_ERR = np.array((-0.0023, 0.0009))
PAD_OFF = np.array((0.0015, -0.0097))
CREST = 0.0025  # m: a top set on straight rests on the thread's crests about this far above where it seats (probes)
# Carrying the upper chamber (gripped 3.3 cm up its body at wrist yaw 0, before it touches anything) its axis sat (0.2,
# -12.2) mm from the commanded tool point, and 2.2 mm lower than commanded (demo run d6: arm sag under the load and the
# top's place between the pads). The set-down is commanded from that offset, and let go 2 mm higher.
PLACE_OFF = np.array((0.0002, -0.0122))
PLACE_SAG = 0.002


# The demo looks where the parts are (closed loop, like a cook watching the skirt line up over the thread): after every
# control step the scorer's state read-out (success(), below) also records the base's rim and the upper chamber's
# bottom face here, and a _Live waypoint works its target out from them when the arm starts on it. The thread only
# catches a top set on within ~0.9 mm of the base's axis; open loop the carried top came down 0.6-4 mm off (d7-d25).
_SEEN = {}


def _see(objects):
    _SEEN["rim"] = _rim(objects)
    _SEEN["top"] = np.array(objects["top"].pos, float)
    _SEEN["comp"] = gasket_compression(objects) if top_on(objects) else -1.0  # a top knocked off reads nothing
    _SEEN["top_ax"] = _rot(objects["top"].quat_xyzw)[:, 2]
    _SEEN["top_yaw"] = _yaw(objects["top"].quat_xyzw)
    lg = objects.get("left_gripper")
    _SEEN["ltcp"] = None if lg is None else np.array(lg.pos, float)


def _seen(key, default):
    v = _SEEN.get(key)
    return np.array(default, float) if v is None else v


class _Live(kit.Tilted):
    """A demo waypoint whose (pos, yaw[, finger[, roll, pitch]]) is `fn()`, worked out once, when the arm starts on
    it (a yamkit Tilted waypoint, so the grip can also be tipped by `roll` about x and `pitch` about y)."""

    def __init__(self, duration, fn, finger, label):
        self.duration, self.label = duration, label
        self._fn, self._v, self._finger = fn, None, finger

    def _get(self):
        if self._v is None:
            r = tuple(self._fn())
            fing = float(r[2]) if len(r) > 2 and r[2] is not None else self._finger
            rp = (float(r[3]), float(r[4])) if len(r) > 4 else (0.0, 0.0)
            self._v = (tuple(float(v) for v in r[0]), float(r[1]), fing, *rp)
        return self._v

    @property
    def finger(self):
        return self._get()[2]

    @property
    def roll(self):
        return self._get()[3]

    @property
    def pitch(self):
        return self._get()[4]

    @property
    def pos(self):
        return self._get()[0]

    @property
    def yaw(self):
        return self._get()[1]


# Once the gasket is pressed the strokes still planned are skipped: each waypoint of them lasts one control step (the
# open hand holds over the lid), and the other hand's hold on the base is shortened by the same time, so both hands
# let go as soon as the top is snug instead of waiting out the unused strokes (about 30 s).
SKIP_S = 0.05
_SKIPPED = []  # the skippable waypoints


class _Skip(_Live):
    """A _Live waypoint that lasts SKIP_S once `skip()` is true (checked only before the arm reaches it)."""

    def __init__(self, duration, fn, finger, label, skip):
        self._skip = skip
        super().__init__(duration, fn, finger, label)
        _SKIPPED.append(self)

    @property
    def duration(self):
        return SKIP_S if self._skip() else self._dur

    @duration.setter
    def duration(self, v):
        self._dur = v


def skipped_s():
    """Seconds the skipped waypoints no longer take."""
    return sum(w._dur - SKIP_S for w in _SKIPPED if w._skip())


class _HoldBase(_Live):
    """The other hand's hold on the base: it ends `skipped_s()` sooner."""

    def __init__(self, duration, pos, finger, label):
        super().__init__(duration, lambda: (pos, 0.0, finger, 0.0, PITCH_IN), finger, label)

    @property
    def duration(self):
        return max(0.5, self._dur - skipped_s())

    @duration.setter
    def duration(self, v):
        self._dur = v


def _axis():
    """The base's axis (xy) as seen now."""
    return _seen("rim", (BX, BY, FLOOR_Z + RIM))[:2]


def _about_axis(yaw, pad_off=None, axis=None):
    """Tool point xy that puts a held top's axis on the base's axis (`axis`, default its modeled spot) at wrist yaw
    `yaw` (rad). `pad_off`: the top's axis from the commanded tool point at yaw 0 (default: the regrasp's PAD_OFF and
    ARM_ERR)."""
    po = PAD_OFF + ARM_ERR if pad_off is None else np.asarray(pad_off, float)
    c, s = math.cos(yaw), math.sin(yaw)
    off = np.array((c * po[0] - s * po[1], s * po[0] + c * po[1]))
    return tuple(np.array((BX, BY) if axis is None else axis, float) - off)


# The open hand rolls back above the whole upper chamber: a top that turns more than 45 degrees per stroke brings its
# handle under the fingers' path back, so they rise until the pad tips clear the handle (7.1 cm up), the spout (8.4 cm)
# and the lid (8.7 cm), with 1 cm to spare.
LIFT_OVER = T_H + 0.010 + PAD_TIP_PAST_TCP - GRIP_TW  # 7.7 cm over the grip height


def _centered(axis=None):
    """Tool point xy that puts the open hand's fingers evenly round the top's axis (each 4.7 cm out, 7 mm clear of
    its 4.0 cm upper body)."""
    return tuple(np.array((BX, BY) if axis is None else axis, float) - ARM_ERR)


# The strokes stop once the gasket is pressed this far (a margin over the 0.10 mm snug): turning a seated top further
# only makes the pads slip and wrenches the held base.
C_STOP = 0.00015
# Regrip: only one finger of the YAM closes; the other pad face stands FIXED_PAD from the tool point (-y at yaw 0). The
# open hand is brought (closed loop on the tool point it reaches) to where that pad just touches the body (r 35 mm),
# so the closing finger squeezes the top against it. Opened 4-8 mm short (open loop: the soft wrist settles a few mm
# off, and yields further to the closing finger), the finger alone shoved the threaded top sideways, tipped it 2.3 deg
# off its groove and it dropped a pitch onto the basket's flange, tipping and sinking the basket (d30, d31).
REGRIP_D = FIXED_PAD - T_R + 0.0001  # tool point from the top's axis, toward +y at yaw 0 (0.1 mm press)
REGRIP_FIXES = 2
SEAT_GZ_REL = K_FLANGE_T + GRIP_TW  # grip height over the rim with the top seated on the basket's flange


# Regrip yaw from the handle (closed loop on the top's seen yaw): the finger plates are 6.8 cm wide, so with the handle
# 39-45 degrees off the closing axis the fixed pad's plate landed on it (d32: the open hand stopped 3 mm short, and
# closing pushed the top over through the handle). Each stroke instead starts with the handle along the plates (90
# degrees off the closing axis, beside them) and turns as far as the wrist goes (to YAW_END): the handle turns with the
# hand, so it is beside the plates again at the next regrip, half a turn on.
YAW_HI, YAW_END, MIN_STROKE = math.radians(70.0), math.radians(-155.0), math.radians(60.0)  # wrist -160..75 at the seat
N_STROKES = 5  # about 104 + 4 x 180 degrees: past the ~630 the thread needs; the gasket stop ends it sooner
TURN_GAIN = 0.5  # of the seen tilt corrected at each 15-degree step
# The regrip squeezes the 7.0 cm body to a 6.3 cm gap (opening 0.68), not fully shut: commanded shut, the finger drove
# 1-2 cm into the body's collider (opening 0.08-0.17 on a 7 cm top), dragging the threaded top with it (d30-d32).
REGRIP_FINGER = 0.68 * FINGER_OPEN


def _regrip_yaw():
    """The start yaw that puts the handle along the plates, within the wrist's reach: the largest one (longest stroke)."""
    h = _SEEN.get("top_yaw", 0.0) + HANDLE_ANG
    c = [h + math.pi * m for m in range(-4, 5)]
    ok = [y for y in c if YAW_END + MIN_STROKE <= y <= YAW_HI]
    if ok:
        return max(ok)
    return min((min(max(y, YAW_END + MIN_STROKE), YAW_HI) for y in c), key=lambda y: min(abs(y - x) for x in c))


def twist(n_strokes, start_yaw, step_s=0.3, grip_s=0.7, held_at=None, open_s=1.4, n=12):
    """Regrasp loop (grip, roll clockwise in `n` steps to YAW_END, let go, lift the open hand over the top, roll back
    to the next start yaw, come down round it again, repeat). Each stroke is worked out from where the base's axis,
    the top and its handle are when it starts (see _Live): it grips the top GRIP_TW up its body with the handle along
    the plates (_regrip_yaw), turns about the base's axis and follows the thread down at its 3.0 mm a turn, no lower
    than the seat and never below the seen top. With `held_at` (a function giving the tool point) the hand already
    holds the top there at `start_yaw`: it opens first."""
    label = "screw the top on"
    pitch = TH_FEMALE.pitch
    st = {"y0": math.radians(start_yaw), "yn": None}

    def grip_z():
        rim_z = _seen("rim", (BX, BY, FLOOR_Z + RIM))[2]
        top_z = _seen("top", (BX, BY, FLOOR_Z + RIM + K_FLANGE_T + CREST))[2]
        return max(rim_z + SEAT_GZ_REL, top_z + GRIP_TW), rim_z + SEAT_GZ_REL

    def next_yaw():  # latched once per regrip, from the released top's handle
        if st["yn"] is None:
            st["yn"] = _regrip_yaw()
        return st["yn"]

    def start():  # a stroke's first waypoint: latch the axis, yaw and grip height, or stop once the gasket is pressed
        st["ax"] = _axis()
        st["z"], st["zmin"] = grip_z()
        st["y0"] = next_yaw()
        st["yn"], st["yf"], st["ylast"] = None, None, st["y0"]
        st["i"] = st.get("i", -1) + 1
        st.setdefault("stop", False)
        if not st["stop"] and _SEEN.get("comp", -1.0) >= C_STOP:
            st["stop"], st["stop_i"] = True, st["i"] - 1
        st["cmd"] = np.array(_about_axis(st["y0"], axis=st["ax"]), float)
        return hold() if st["stop"] else ((*st["cmd"], st["z"]), st["y0"])

    def want_tcp(yaw):  # where the tool point should be: the fixed pad on the body
        top = _seen("top", (BX, BY, 0.0))[:2]
        return top + np.array((-math.sin(yaw) * REGRIP_D, math.cos(yaw) * REGRIP_D))

    def fix_grip():  # move the commanded tool point by what the reached one is off where it should be
        if st["stop"]:
            return hold()
        tcp = _SEEN.get("ltcp")
        if tcp is not None:
            err = want_tcp(st["y0"]) - tcp[:2]
            st["cmd"] = st["cmd"] + 0.8 * np.clip(err, -0.006, 0.006)
        return (*st["cmd"], st["z"]), st["y0"]

    def held_xy(yaw):  # the gripped hand turns rigidly about the base's axis from where it gripped
        dl = yaw - st["y0"]
        v = st["cmd"] - st["ax"]
        return st["ax"] + np.array((math.cos(dl) * v[0] - math.sin(dl) * v[1], math.sin(dl) * v[0] + math.cos(dl) * v[1]))

    def hold():  # snug: the open hand waits over the top, clear of the handle
        return (*_centered(st["ax"]), st["z"] + LIFT_OVER), st.get("hy", st["y0"]), FINGER_OPEN

    def yaw_at(k):
        return st["y0"] + (YAW_END - st["y0"]) * k / n

    def turn(k, finger=None):
        def f():
            if st["stop"]:
                return hold()
            # stop turning the moment the gasket is pressed: turned on through the stroke, it was pressed 0.4-0.7
            # mm (some hundreds of newtons), and let go, the top sprang 2-3 mm back up its thread, 5 deg crooked (d38)
            if k > 0 and st["yf"] is None and _SEEN.get("comp", -1.0) >= C_STOP:
                st["yf"] = st["ylast"]
            yaw = yaw_at(k) if st["yf"] is None else st["yf"]
            st["ylast"] = yaw
            d = math.degrees(st["y0"] - yaw)
            # follow the thread down, but never below where the top is seen now: a top that lags the thread is
            # turned, not pushed (pushed, the ridges go through each other, d28, d30)
            top_z = _seen("top", (0.0, 0.0, 1.0))[2] + GRIP_TW
            # keep the gripped top upright (closed loop on its seen axis, world roll/pitch as in the line-up): turned
            # 1.3-1.8 deg crooked, its gasket bore on one side of the flange and dragged the basket 1.3 mm sideways
            # off its ledge and 2.1 mm down (d33a)
            if k == 0:
                st["r"], st["p"] = 0.0, 0.0
            elif finger is None:
                ax = _seen("top_ax", (0.0, 0.0, 1.0))
                st["r"] = float(np.clip(st["r"] + TURN_GAIN * ax[1], -0.05, 0.05))
                st["p"] = float(np.clip(st["p"] - TURN_GAIN * ax[0], -0.05, 0.05))
            return ((*held_xy(yaw), max(st["zmin"], min(st["z"] - pitch * d / 360.0, top_z))), yaw, finger,
                    st["r"], st["p"])
        return f

    def yend():  # where the stroke ended: at the wrist's end, or where the gasket got pressed
        return YAW_END if st.get("yf") is None else st["yf"]

    def up():  # let go: snug now? then no more strokes
        if st["stop"]:  # stopped before this stroke: the open hand stays over the top (it came down onto its
            return hold()  # shoulder from there and lifted the screwed-on top off with the basket, d39, d40)
        st["stop"] = st.get("yf") is not None or _SEEN.get("comp", -1.0) >= C_STOP
        if st["stop"]:
            st["stop_i"] = st["i"]
        ye = yend()
        d = math.degrees(st["y0"] - ye)
        if st["stop"]:
            st.setdefault("hy", ye)  # the open hand keeps its yaw from here on (rolled, it could catch the handle)
            return (*held_xy(ye), st["z"] + 0.025), ye
        return (*held_xy(ye), max(st["zmin"], st["z"] - pitch * d / 360.0) + 0.025), ye

    def cen(h, yaw_fn):
        return lambda: hold() if st["stop"] else ((*_centered(st["ax"]), st["z"] + h), yaw_fn())

    out = []
    if held_at is not None:
        # opened easing 2 mm down: opened in place, the arm springing back up dragged the top 6-12 mm up (d13, d25)
        def opening():
            x, y, z = held_at()
            return (x, y, z - 0.002), st["y0"]
        out.append(_Live(open_s, opening, FINGER_OPEN, f"{label}: open the hand"))
    def after(i):  # stroke i comes after the one the strokes stopped in
        return lambda: st.get("stop_i") is not None and i > st["stop_i"]

    def back(i):  # stroke i's roll back and descent are not needed: the strokes stopped in it or before
        return lambda: st.get("stop_i") is not None and i >= st["stop_i"]

    for i in range(n_strokes):
        sk = after(i)
        out.append(_Skip(0.8, start, FINGER_OPEN, f"{label}: to the grip offset", sk))
        for _ in range(REGRIP_FIXES):
            out.append(_Skip(0.8, fix_grip, FINGER_OPEN, f"{label}: fixed pad onto the body", sk))
        out.append(_Skip(grip_s, turn(0), REGRIP_FINGER, f"{label}: grip", sk))
        for k in range(1, n + 1):
            out.append(_Skip(step_s, turn(k), REGRIP_FINGER, f"{label}: turn", sk))
        out.append(_Skip(open_s, turn(n, FINGER_OPEN), FINGER_OPEN, f"{label}: let go", sk))
        # straight up off it first: moved sideways at once, the opened hand knocked a part-threaded top off (d12)
        out.append(_Skip(1.2, up, FINGER_OPEN, f"{label}: up off it", sk))
        out.append(_Skip(0.6, cen(0.025, yend), FINGER_OPEN, f"{label}: center the open hand", sk))
        if i < n_strokes - 1:  # up over the handle, roll back to the next start yaw there, and down round the body
            out.append(_Skip(1.0, cen(LIFT_OVER, yend), FINGER_OPEN, f"{label}: lift over", sk))
            for k in range(1, n + 1):
                out.append(_Skip(step_s * 0.8, cen(LIFT_OVER, lambda k=k: YAW_END + (next_yaw() - YAW_END) * k / n),
                                 FINGER_OPEN, f"{label}: roll back", back(i)))
            out.append(_Skip(1.0, cen(0.0, next_yaw), FINGER_OPEN, f"{label}: back down", back(i)))
    out.append(_Live(1.2, cen(LIFT_OVER, yend), FINGER_OPEN, f"{label}: lift clear"))
    # straight up well over the pot, then over towards the arm's side, before going home: straight home from over the
    # top, the open hand came down to the top's shoulder and lifted the screwed-on top off with the basket (d39)
    away = lambda dx, dy: lambda: ((*(np.array(_centered(st["ax"])) + (dx, dy)), st["z"] + LIFT_OVER + 0.12),  # noqa: E731
                                   st.get("hy", yend()))
    out.append(_Live(1.5, away(0.0, 0.0), FINGER_OPEN, f"{label}: up away from the pot"))
    out.append(_Live(1.5, away(0.05, 0.20), FINGER_OPEN, f"{label}: over to the side"))
    return out


PLACE_FIXES = 4  # looks and corrections before setting the top on
PLACE_GAIN = 0.8  # of each correction (the full step overshot: the arm was still settling from the last, d27)
PLACE_HOVER = 0.035  # lined up with the skirt's bottom edge well over the basket's flange (the top can sit 1-2 cm
#                      lower in the hand than carried: at 16 and 24 mm its skirt hung 0-2 mm over the rim, d27, d28)
PLACE_REST = 0.002  # m: set on with its bottom face this far over gasket touch (it rests on the crests 0.3-2.7 mm up)
HELD_PRESS = 0.002  # m the held first turn eases down: the thread's 0.75 mm a quarter turn, and a light press
# The left hand picks, carries and sets the upper chamber with its wrist turned 45 degrees, so the handle stands 90
# degrees off the fingers' closing axis, beside the finger plates: at 45 degrees off, the opening finger caught the
# handle when it let go after the first turn and tipped the top 13 degrees (d29). The regrip strokes then work at
# wrist yaw 0, where the handle is 45 degrees off the fingers at every regrip.
PLACE_YAW = math.radians(45.0)
STROKE_YAW = 0.0


def press_plan(at):
    """The left hand waits at home until `at` s, then presses the basket home with its closed fingertips on its middle
    (the base is held by then; holding it can unseat the basket a few mm, d17, d21)."""
    pz = FLOOR_Z + RIM + K_FLANGE_T + K_TIP - K_PRESS
    hi = FLOOR_Z + RIM + 0.10
    cx, cy = tuple(np.array((BX, BY)) - ARM_ERR)
    return [kit.Home(max(0.5, at - 2.0), label="wait while the base is filled and held"),
            Waypoint(2.0, (cx, cy, hi), FINGER_CLOSED, "over the basket", PLACE_YAW),
            Waypoint(1.5, (cx, cy, pz + 0.015), FINGER_CLOSED, "down to the coffee", PLACE_YAW),
            Waypoint(1.0, (cx, cy, pz), FINGER_CLOSED, "press the basket home", PLACE_YAW),
            Waypoint(0.5, (cx, cy, pz), FINGER_CLOSED, "hold", PLACE_YAW),
            Waypoint(1.0, (cx, cy, hi), FINGER_CLOSED, "up off it", PLACE_YAW)]


def left_plan(go, steady, press=None):
    """Pick the upper chamber up by its body and hold it aside until `go` s (the right hand has left the pot), set it
    on the collar straight and keep holding it there until `steady` s (the base is held; None: just set on), turn it a
    quarter turn clockwise still holding it, then screw it on with regrip strokes (see twist) and go home. Returns the
    plan, when the top is set on (s) and when it is done screwing (s)."""
    tx, ty = TOP_XY
    gz = TOP + COLLAR_H + 0.0005 + GRIP_DZ
    cx, cy = _about_axis(PLACE_YAW, PLACE_OFF)
    seat_gz = FLOOR_Z + RIM + K_FLANGE_T + GRIP_DZ  # the top seated on the basket's flange
    place_gz = seat_gz + CREST + PLACE_SAG  # set on the thread's crests
    plan = [] if press is None else press_plan(press)
    plan += [Waypoint(2.0, (tx, ty, gz + 0.10), FINGER_OPEN, "over the upper chamber", PLACE_YAW),
            Waypoint(1.0, (tx, ty, gz + 0.10), FINGER_OPEN, "steady", PLACE_YAW),
            Waypoint(2.0, (tx, ty, gz), FINGER_OPEN, "around it", PLACE_YAW),
            Waypoint(0.8, (tx, ty, gz), FINGER_CLOSED, "grip", PLACE_YAW),
            Waypoint(1.2, (tx, ty, seat_gz + 0.04), FINGER_CLOSED, "lift", PLACE_YAW)]
    # carried high: 3-4 cm over the seat the swinging top's skirt clipped the collar on the way in (d11)
    plan.append(Waypoint(max(0.5, go - sum(w.duration for w in plan)), (tx - 0.02, ty + 0.02, seat_gz + 0.07),
                         FINGER_CLOSED, "hold it aside while the base is filled", PLACE_YAW))
    plan += [Waypoint(2.0, (cx, cy, seat_gz + 0.06), FINGER_CLOSED, "over the base", PLACE_YAW),
             Waypoint(2.0, (cx, cy, place_gz + PLACE_HOVER), FINGER_CLOSED, "lower to the collar", PLACE_YAW),
             Waypoint(1.0, (cx, cy, place_gz + PLACE_HOVER), FINGER_CLOSED, "settle", PLACE_YAW)]
    # line the skirt up over the thread from where the top's bottom face and the base's rim are, and stand the top
    # upright with the wrist (closed loop: the arm sags, and the top hangs a few degrees crooked between the pads; set
    # on 3 degrees crooked it cocked to 7 degrees and cross-threaded, d27)
    fix = {"d": np.zeros(2), "r": 0.0, "p": 0.0, "dz": None}

    def line_up():
        rim, top, ax = _SEEN.get("rim"), _SEEN.get("top"), _SEEN.get("top_ax")
        if rim is not None and top is not None and ax is not None:
            fix["d"] = np.clip(fix["d"] + PLACE_GAIN * (rim[:2] - top[:2]), -0.015, 0.015)
            fix["r"] = float(np.clip(fix["r"] + PLACE_GAIN * ax[1], -0.15, 0.15))
            fix["p"] = float(np.clip(fix["p"] - PLACE_GAIN * ax[0], -0.15, 0.15))
            fix["dz"] = float(top[2] - (place_gz + PLACE_HOVER))  # the top's bottom face below the tool point
        return placed(PLACE_HOVER)()

    def on_z():
        """Tool height that sets the top's bottom face PLACE_REST over gasket touch (closed loop on its seen height
        in the hand: it can sit 1-2 cm lower between the pads than carried, and pushed down that much more the
        ridges were forced through each other, d28)."""
        if fix["dz"] is None:
            return place_gz
        rim_z = _seen("rim", (BX, BY, FLOOR_Z + RIM))[2]
        want = rim_z + K_FLANGE_T + GASKET.protrude + PLACE_REST
        return float(np.clip(want - fix["dz"], place_gz - 0.02, place_gz + PLACE_HOVER))

    def placed(dz=None):
        return lambda: ((cx + fix["d"][0], cy + fix["d"][1], on_z() if dz is None else place_gz + dz), PLACE_YAW,
                        None, fix["r"], fix["p"])

    for _ in range(PLACE_FIXES):
        plan += [_Live(0.6, line_up, FINGER_CLOSED, "line the top up over the thread"),
                 _Live(1.0, placed(PLACE_HOVER), FINGER_CLOSED, "settle")]
    plan.append(_Live(1.5, placed(), FINGER_CLOSED, "onto the thread"))
    plan.append(_Live(0.5, placed(), FINGER_CLOSED, "settle on the thread"))
    up = sum(w.duration for w in plan)
    if steady is not None:
        plan.append(_Live(max(0.5, steady - up), placed(), FINGER_CLOSED,
                          "hold it on the thread while the base is steadied"))
    lo, hi = tk.yaw_range("left", (BX, BY, seat_gz))
    if math.degrees(PLACE_YAW) - STROKE < lo or math.degrees(PLACE_YAW) > hi:
        raise ValueError(f"left wrist range {lo:.0f}..{hi:.0f} is too small for a {STROKE:.0f}-degree stroke")
    # the first stroke without letting go: the whole hand turns rigidly about the base's axis (its tilt correction
    # turning with it), pressing down a little, so the top finds its groove upright; let go on the crests, a top cocks
    # (probes, d27)
    hs = {}

    def held(k, n):
        def f():
            if not hs:
                hs["ax"] = _axis()
                hs["p0"] = np.array(placed()()[0])
            dlt = -math.radians(STROKE * k / n)
            c, sn = math.cos(dlt), math.sin(dlt)
            v = hs["p0"][:2] - hs["ax"]
            xy = hs["ax"] + np.array((c * v[0] - sn * v[1], sn * v[0] + c * v[1]))
            r, pp = fix["r"], fix["p"]
            z = hs["p0"][2] - HELD_PRESS * k / n
            return (xy[0], xy[1], z), PLACE_YAW + dlt, None, c * r - sn * pp, sn * r + c * pp
        return f

    n = max(1, math.ceil(STROKE / 10.0))
    for k in range(1, n + 1):
        plan.append(_Live(0.4, held(k, n), FINGER_CLOSED, "screw the top on: first turn, held upright"))
    plan.append(_Live(1.4, held(n, n), FINGER_OPEN, "screw the top on: let go"))
    yend = PLACE_YAW - math.radians(STROKE)

    def over_z():  # pad tips over the handle, spout and lid, from where the top is now
        hs["over"] = float(_seen("top", hs["p0"] - (0, 0, GRIP_TW))[2] + GRIP_TW + LIFT_OVER)
        return hs["over"]

    def lift(h, yaw):
        return lambda: ((*held(n, n)()[0][:2], hs["p0"][2] + h), yaw)

    plan += [_Live(1.2, lift(0.025, yend), FINGER_OPEN, "screw the top on: up off it"),
             _Live(1.0, lambda: ((*_centered(_axis()), over_z()), yend), FINGER_OPEN, "screw the top on: lift over")]
    def ry():  # the first regrip's yaw: the released top's handle along the plates
        if "ry" not in hs:
            hs["ry"] = _regrip_yaw()
        return hs["ry"]

    for k in range(1, 7):
        plan.append(_Live(0.25, lambda k=k: ((*_centered(_axis()), hs["over"]), yend + (ry() - yend) * k / 6),
                          FINGER_OPEN, "screw the top on: roll back"))
    plan.append(_Live(1.0, lambda: ((*_centered(_axis()), hs["over"] - LIFT_OVER), ry()), FINGER_OPEN,
                      "screw the top on: back down"))
    plan += twist(N_STROKES, math.degrees(STROKE_YAW))
    done = sum(w.duration for w in plan)
    return plan + [kit.Home(2.0), kit.Home(1.5, label="hold at home")], up, done


def demo():
    """The right arm pours from the carafe into the base, tipping it about its lip 6.5 cm over the rim, sets it back
    down, lifts the basket out of its cup and drops it into the base, and comes round behind the pot to hold the base by
    two of its flats. The left arm then sets the upper chamber on the collar, turns it a quarter turn still holding it,
    and screws it on with regrip strokes (each starting with the handle beside the finger plates and turning to the
    wrist's end) until the gasket is pressed. It lets go and lifts clear, and the right hand lets go of the base."""
    right, _ = right_plan()
    right += steady_plan()
    held = sum(w.duration for w in right)
    # the other hand takes hold of the free base before the upper chamber comes over it: gripping it with a top already
    # set on popped the basket up ~3 mm and the top's face pinned it there crooked (d15-d17); with nothing over it, the
    # basket settles back down the mouth's lead-in
    left, _, done = left_plan(held + 1.0, None, press=held)
    right.append(_HoldBase(max(0.5, done - held), (STEADY_X, STEADY_Y, FLOOR_Z + BASE_GZ), STEADY_GRIP,
                           "hold the base"))
    return {"right": right + release_plan(), "left": left}
