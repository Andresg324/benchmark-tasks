"""A 20 x 16 x 6 in steel wall-mount control enclosure stands on the bench with its door off, locked out and tested
dead. On its back pan, a three-pole Bussmann BMM603-3SQ midget fuse block sits on a horizontal DIN rail, holding three
Bussmann FNM 13/32 x 1-1/2 in (10.3 x 38.1 mm) fuses upright in open spring clips: F1, F2 and F3 from the left. A
yellow Ideal 34-015 tong fuse puller lies on the bench in front of the enclosure, and a parts tray beside it. "The
panel is locked out and tested dead. Use the fuse puller to take out the middle fuse, F2, and put it in the parts
tray." The puller is held pointing into the panel, perpendicular to the fuse, its jaws closed round the fuse's body
between its clips; the fuse is drawn out of both clips toward the robot, carried in the jaws and let go in the tray.
F1 and F3 stay seated.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props; nothing on disk changes)
import clipkit as ck  # noqa: E402  (the clips' spring, applied to the fuses' ferrules)
import guardkit as gk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = ("The panel is locked out and tested dead. Use the fuse puller to take out the middle fuse, F2, and put "
               "it in the parts tray.")

# --- The enclosure: 20 x 16 x 6 in (508 x 406 x 152 mm) steel, standing on the bench, opening toward the robot.
ENC_FRONT, ENC_DEPTH, ENC_W, ENC_H, WALL = 0.273, 0.152, 0.406, 0.508, 0.0015
ENC_BACK = ENC_FRONT + ENC_DEPTH
PAN_FRONT = 0.408  # the back pan's face (on standoffs off the back wall)
RAIL_TOP = PAN_FRONT - 0.0075  # a 35 x 7.5 mm top-hat DIN rail, horizontal
# --- The fuse block (Bussmann BMM603-3SQ, data sheet 10241: 77.0 mm long, 20.0 mm per pole, 33.0 mm high off the
# rail). The fuses stand upright, their axes 28 mm off the rail, in three poles 20 mm apart.
X_F = RAIL_TOP - 0.028
Z_C = TOP + 0.150  # the fuses' middles above the bench
POLES = {"fuse_f1": 0.020, "fuse_f2": 0.0, "fuse_f3": -0.020}  # F1 on the robot's left
BLOCK_L, BLOCK_H = 0.077, 0.033
# --- The fuses: Bussmann FNM, 13/32 x 1-1/2 in (10.3 x 38.1 mm), about 9 g.
FUSE_R, FUSE_L, FERRULE_L = 0.00515, 0.0381, 0.0095
END = (FUSE_L - FERRULE_L) / 2  # 14.3 mm: each ferrule's centre from the fuse's middle
BODY_HALF = 0.0048  # collider: the fibre tube as a 9.6 mm square prism inside its 10.3 mm round
FUSE_MASS = 0.009
# Rigid contacts have no rolling or spinning resistance, so a set-down fuse would roll or spin on the tray for
# seconds; a real fuse on a tray stops within a fraction of a second. Angular damping stands in for that.
FUSE_SPIN_DAMPING = 4.0  # 1/s
CLIP_W = 0.008  # each clip's width along the fuse
CLIP_HOLD = 6.0  # N: what each clip presses its ferrule onto the clip base with (assumed; see README)
RELEASE = 0.0025  # m: a ferrule's centre this far out of its seat has passed the clip fingers' lips
CLIPS = tuple(ck.Clip(name, (X_F, y, Z_C), ends=(-END, END), hold=CLIP_HOLD, release=RELEASE)
              for name, y in POLES.items())
ck.register(CLIPS)

# --- The puller: Ideal 34-015, one-piece yellow high-impact nylon tongs, 5 in (127 mm), for 1/4-1/2 in fuses. Its frame:
# origin at the loop end, on its underside, x toward the jaws, the two arms either side of x (z up when it lies flat).
HINGE_X = 0.008  # the arms flex at the loop
ARM_Y0, ARM_YH, ARM_Y1 = 0.0045, 0.008, 0.0115  # each arm's centre line off the middle: at the loop, along the
#                                               straight handles (x 0.025-0.085) and at the jaws (x 0.100)
ARM_T = 0.004  # arm thickness (across), 10 mm tall; the notched handles 12 mm tall
JAW_X0, JAW_X1 = 0.100, 0.118  # the jaws' gripping faces along the puller
JAW_IN = 0.0085  # each jaw's face off the middle at rest: 17 mm open, past a 1/2 in fuse
JAW_X = (JAW_X0 + JAW_X1) / 2
JAW_Z = 0.005  # the jaws' and arms' middle height
GRASP_X, GRASP_Z = 0.050, 0.0175  # where the demo pinches the handles (tool point, puller frame)
CLOSE_STOP = 0.045  # rad: each arm's closing stop
ARM_SPRING = 15.0  # N*m/rad: the nylon's flex (about 5.5 N at each jaw sprung shut on a 10 mm fuse); inferred
PULLER_START = (0.150, -0.100)
# --- The parts tray: 150 x 100 x 30 mm, on the bench to the right of the puller.
TRAY = (0.205, -0.260)
TRAY_SIZE = (0.150, 0.100, 0.030)

STEEL = kit.Look((0.80, 0.80, 0.78), 0.5)  # light grey painted enclosure (RAL 7035)
PAN = kit.Look((0.92, 0.92, 0.90), 0.6)  # white back pan
RAIL = kit.Look((0.75, 0.76, 0.78), 0.3, metallic=0.8)
BLOCK = kit.Look((0.08, 0.08, 0.09), 0.5)
TIN = kit.Look((0.78, 0.79, 0.80), 0.35, metallic=0.7)
SCREW = kit.Look((0.62, 0.63, 0.66), 0.4, metallic=0.8)
TUBE = kit.Look((0.45, 0.20, 0.12), 0.7)  # brown fibre tube
LABEL = kit.Look((0.95, 0.95, 0.92), 0.7)
YELLOW = kit.Look((0.95, 0.78, 0.08), 0.45)
TRAY_LOOK = kit.Look((0.15, 0.30, 0.65), 0.5)
WIRE = {"fuse_f1": kit.Look((0.75, 0.10, 0.10), 0.6), "fuse_f2": kit.Look((0.10, 0.10, 0.10), 0.6),
        "fuse_f3": kit.Look((0.95, 0.95, 0.95), 0.6)}


def _box(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return tuple(float(v) for v in (lo + hi) / 2), tuple(float(v) for v in hi - lo)


def _enclosure():
    """The enclosure, back pan, DIN rail (fixed). Frame: the enclosure's middle."""
    cx, cz = ENC_FRONT + ENC_DEPTH / 2, ENC_H / 2
    parts = [((ENC_BACK - WALL, -ENC_W / 2, 0), (ENC_BACK, ENC_W / 2, ENC_H)),  # back
             ((ENC_FRONT, -ENC_W / 2, 0), (ENC_BACK, -ENC_W / 2 + WALL, ENC_H)),  # right side
             ((ENC_FRONT, ENC_W / 2 - WALL, 0), (ENC_BACK, ENC_W / 2, ENC_H)),  # left side
             ((ENC_FRONT, -ENC_W / 2, 0), (ENC_BACK, ENC_W / 2, WALL)),  # bottom
             ((ENC_FRONT, -ENC_W / 2, ENC_H - WALL), (ENC_BACK, ENC_W / 2, ENC_H)),  # top
             ((PAN_FRONT, -0.175, 0.025), (PAN_FRONT + 0.0025, 0.175, 0.480))]  # back pan
    meshes, colliders = [], []
    for lo, hi in parts:
        c, s = _box((lo[0] - cx, lo[1], lo[2] - cz), (hi[0] - cx, hi[1], hi[2] - cz))
        meshes.append(kit.box(c, s, PAN if lo[0] == PAN_FRONT else STEEL))
        colliders.append(kit.Collider(c, size=s))
    # the door's flange lip round the opening (drawn), the rail and two more blocks' worth of terminal strip (drawn)
    for z0, z1 in ((0.0, 0.012), (ENC_H - 0.012, ENC_H)):
        c, s = _box((ENC_FRONT - cx, -ENC_W / 2, z0 - cz), (ENC_FRONT + 0.003 - cx, ENC_W / 2, z1 - cz))
        meshes.append(kit.box(c, s, STEEL))
    zr = Z_C - TOP
    c, s = _box((RAIL_TOP - cx, -0.160, zr - 0.0175 - cz), (PAN_FRONT - cx, 0.160, zr + 0.0175 - cz))
    meshes.append(kit.box(c, s, RAIL))
    colliders.append(kit.Collider(c, size=s))
    for k in range(6):  # grey feed-through terminal blocks on the same rail, left of the fuse block
        y0 = 0.045 + 0.0065 * k
        c, s = _box((RAIL_TOP - 0.045 - cx, y0, zr - 0.030 - cz), (RAIL_TOP - cx, y0 + 0.006, zr + 0.030 - cz))
        meshes.append(kit.box(c, s, kit.Look((0.55, 0.57, 0.60), 0.5)))
    return kit.Prop("enclosure", pos=(cx, 0.0, TOP + cz), size=(ENC_DEPTH, ENC_W, ENC_H), mass=None,
                    meshes=tuple(meshes), colliders=tuple(colliders))


def _block():
    """The BMM603-3SQ block (fixed on its rail). Frame: F2's seated middle, x toward the pan (into the clips)."""
    meshes, colliders = [], []
    back = RAIL_TOP - X_F  # 28 mm: the block's base on the rail, behind the fuses' axes
    front = back - BLOCK_H  # 5 mm in front of the axes: the clip lips and the barriers

    def solid(lo, hi, look=BLOCK, collide=True):
        c, s = _box(lo, hi)
        meshes.append(kit.box(c, s, look))
        if collide:
            colliders.append(kit.Collider(c, size=s))

    for y in POLES.values():
        # the floor under the fuse's body: a recess 12 mm behind the axis (the puller's jaws reach in round the body)
        solid((0.012, y - 0.009, -0.0095), (back, y + 0.009, 0.0095))
        for z in (-END, END):
            # each clip: its base touching the ferrule behind (tinned copper), two fingers up its sides to the
            # ferrule's widest point, drawn on to their lips 2.5 mm past it
            solid((FUSE_R + 0.0001, y - 0.004, z - CLIP_W / 2), (back, y + 0.004, z + CLIP_W / 2), TIN)
            for s in (-1, 1):
                yi = y + s * (FUSE_R + 0.0001)
                lo, hi = sorted((yi, yi + s * 0.0015))
                solid((0.0, lo, z - CLIP_W / 2), (0.012, hi, z + CLIP_W / 2), TIN)
                lo2, hi2 = sorted((y + s * 0.0040, yi + s * 0.0015))
                c, sz = _box((-0.0025, lo2, z - CLIP_W / 2), (0.0, hi2, z + CLIP_W / 2))
                meshes.append(kit.box(c, sz, TIN))  # the lip, drawn only (the clip's spring is clipkit's law)
            # the block's moulding round the clip, behind the fuse
            zz = sorted((z - math.copysign(0.0048, z), math.copysign(0.0385, z)))
            solid((0.012, y - 0.009, zz[0]), (back, y + 0.009, zz[1]))
        for z in (-1, 1):  # terminals beyond each fuse end: screw housings standing to the block's full height
            zz = sorted((z * 0.0205, z * BLOCK_L / 2))
            solid((front, y - 0.009, zz[0]), (back, y + 0.009, zz[1]))
            c, s = _box((front - 0.0005, y - 0.0035, z * 0.031 - 0.0035), (front, y + 0.0035, z * 0.031 + 0.0035))
            meshes.append(kit.box(c, s, SCREW))
            c, s = _box((front + 0.004, y - 0.0025, z * BLOCK_L / 2 - (0.003 if z > 0 else -0.003) - 0.001),
                        (front + 0.010, y + 0.0025, z * BLOCK_L / 2 + 0.002))
            meshes.append(kit.box(c, s, WIRE[[k for k, v in POLES.items() if v == y][0]]))
    for yb in (-0.030, -0.010, 0.010, 0.030):  # the barriers between the poles and at the block's sides
        lo, hi = (yb - 0.00075, yb + 0.00075)
        for z in (-1, 1):
            zz = sorted((z * 0.0095, z * BLOCK_L / 2))
            solid((front, lo, zz[0]), (back, hi, zz[1]))
    for name, y in POLES.items():  # circuit markers on the block's end, below each fuse (F1, F2, F3 left to right)
        c, s = _box((front - 0.0004, y - 0.006, -BLOCK_L / 2 + 0.002), (front, y + 0.006, -BLOCK_L / 2 + 0.012))
        meshes.append(kit.box(c, s, LABEL))
        n = int(name[-1])
        for k in range(n):  # one, two or three dark ticks: the number at a glance
            c, s = _box((front - 0.0008, y - 0.004 + 0.003 * k, -BLOCK_L / 2 + 0.004),
                        (front - 0.0004, y - 0.0028 + 0.003 * k, -BLOCK_L / 2 + 0.010))
            meshes.append(kit.box(c, s, BLOCK))
    return kit.Prop("block", pos=(X_F, 0.0, Z_C), size=(0.034, 0.062, BLOCK_L), mass=None, meshes=tuple(meshes),
                    colliders=tuple(colliders))


def _fuse(name, y):
    """A fuse standing in its clips. Frame: its middle, z along its axis."""
    body = FUSE_L - 2 * FERRULE_L
    meshes = [kit.cylinder((0, 0, 0), FUSE_R - 0.0002, body + 0.001, TUBE)]
    for z in (-END, END):
        meshes.append(kit.cylinder((0, 0, z), FUSE_R, FERRULE_L, TIN))
    meshes.append(kit.box((-FUSE_R + 0.0002, 0, 0), (0.0008, 0.004, body * 0.7), LABEL))  # printed band, front
    colliders = [kit.Collider((0, 0, 0), size=(2 * BODY_HALF, 2 * BODY_HALF, body))]
    colliders += [kit.Collider((0, 0, z), radius=FUSE_R, height=FERRULE_L) for z in (-END, END)]
    return kit.Prop(name, pos=(X_F, y, Z_C), size=(2 * FUSE_R, 2 * FUSE_R, FUSE_L), mass=FUSE_MASS,
                    meshes=tuple(meshes), colliders=tuple(colliders), angular_damping=FUSE_SPIN_DAMPING)


def arm_y(x):
    """An arm's centre line off the puller's middle at x (puller frame), at rest: out from the loop, straight and
    parallel along the notched handles, then out again to the jaws."""
    return float(np.interp(x, [HINGE_X, 0.025, 0.085, 0.100], [ARM_Y0, ARM_YH, ARM_YH, ARM_Y1]))


def _arm(s):
    """One arm of the tongs (s = +1 left, -1 right) on its flex hinge at the loop."""
    anchor = (HINGE_X, s * 0.0035, 0.0)
    meshes, colliders = [], []

    def part(lo, hi, collide=True):
        c, sz = _box(lo, hi)
        c = (c[0] - anchor[0], c[1] - anchor[1], c[2] - anchor[2])
        meshes.append(kit.box(c, sz, YELLOW))
        if collide:
            colliders.append(kit.Collider(c, size=sz))

    xs = [0.010 + 0.015 * k for k in range(7)][:-1] + [0.085, 0.100]
    for x0, x1 in zip(xs[:-1], xs[1:]):
        yc = s * arm_y((x0 + x1) / 2)
        h = 0.012 if 0.025 <= x0 < 0.085 else 0.010
        part((x0, yc - ARM_T / 2, 0.0), (x1, yc + ARM_T / 2, h))
    for k in range(9):  # the handle's notches (drawn)
        x = 0.030 + 0.006 * k
        yc = s * (arm_y(x) + ARM_T / 2)
        part((x, min(yc, yc + s * 0.0008), 0.002), (x + 0.002, max(yc, yc + s * 0.0008), 0.011), collide=False)
    ys = s * arm_y(0.095)  # finger stop: a ridge outward between the handle and the jaws
    part((0.093, min(ys, ys + s * 0.0055), 0.0), (0.097, max(ys, ys + s * 0.0055), 0.010))
    part((JAW_X0, min(s * JAW_IN, s * (JAW_IN + 0.003)), 0.0), (JAW_X1, max(s * JAW_IN, s * (JAW_IN + 0.003)), 0.010))
    part((JAW_X1, min(s * (JAW_IN + 0.0015), s * (JAW_IN + 0.003)), 0.001),
         (JAW_X1 + 0.004, max(s * (JAW_IN + 0.0015), s * (JAW_IN + 0.003)), 0.009))  # rounded nose
    name = "left" if s > 0 else "right"
    # closing is toward the middle; squeezed shut with nothing in the jaws, the handles' finger stops meet at 0.045
    # rad (jaws 8 mm apart), so a fuse in the jaws (9.6 mm prism) is always gripped before that
    lo, hi = (-CLOSE_STOP, 0.02) if s > 0 else (-0.02, CLOSE_STOP)
    return kit.Link(name, "revolute", "Z", anchor, lower=lo, upper=hi, start=0.0, mass=0.008, meshes=tuple(meshes),
                    colliders=tuple(colliders), law=ck.Spring(rest=0.0, k=ARM_SPRING))


def _puller():
    loop = kit.box((0.005, 0.0, 0.005), (0.010, 0.013, 0.010), YELLOW)
    return kit.Mechanism("puller", pos=(PULLER_START[0], PULLER_START[1], TOP + 0.0002), size=(0.25, 0.04, 0.03),
                         mass=0.004, meshes=(loop,),
                         colliders=(kit.Collider((0.005, 0.0, 0.005), size=(0.010, 0.013, 0.010)),),
                         links=(_arm(1), _arm(-1)))


def _tray():
    L, W, H = TRAY_SIZE
    t = 0.002
    parts = [((-L / 2, -W / 2, -H / 2), (L / 2, W / 2, -H / 2 + t)),
             ((-L / 2, -W / 2, -H / 2), (-L / 2 + t, W / 2, H / 2)), ((L / 2 - t, -W / 2, -H / 2), (L / 2, W / 2, H / 2)),
             ((-L / 2, -W / 2, -H / 2), (L / 2, -W / 2 + t, H / 2)), ((-L / 2, W / 2 - t, -H / 2), (L / 2, W / 2, H / 2))]
    meshes, colliders = [], []
    for lo, hi in parts:
        c, s = _box(lo, hi)
        meshes.append(kit.box(c, s, TRAY_LOOK))
        colliders.append(kit.Collider(c, size=s))
    return kit.Prop("tray", pos=(TRAY[0], TRAY[1], TOP + H / 2), size=TRAY_SIZE, mass=0.10, meshes=tuple(meshes),
                    colliders=tuple(colliders))


OBJECTS = [_enclosure(), _block()] + [_fuse(n, y) for n, y in POLES.items()] + [_puller(), _tray()]
GRIPPERS = ("left_gripper", "right_gripper")
CLIP = {c.name: c for c in CLIPS}

# --- Readouts ---------------------------------------------------------------------------------------------------------
FUSE_SHAPE = gk.Seg((0.0, 0.0, -FUSE_L / 2), (0.0, 0.0, FUSE_L / 2), FUSE_R)
JAW_REACH = 0.001  # m: a jaw's face within 1 mm of the tube's surface (closed, it presses the 9.6 mm prism at 4.8 mm)
STILL = 0.02  # m/s


def ends_out(objects, name):
    """[bottom, top]: whether each ferrule of a fuse is out of its clip (past the fingers' lips, or off it)."""
    s = objects[name]
    return ck.seat_state(s.pos, s.quat_xyzw, CLIP[name])[0]


def jaw_faces(objects):
    """World points: each jaw's gripping face as (start, end) along the puller, at the jaws' middle height (left,
    right)."""
    out = []
    for side, s in (("left", 1), ("right", -1)):
        st = objects.get(f"puller/{side}")
        if st is None:
            return []
        anchor = np.array([HINGE_X, s * 0.0035, 0.0])
        R, o = gk._rot(st.quat_xyzw), np.asarray(st.pos, float)
        out.append(tuple(o + R @ (np.array([x, s * JAW_IN, JAW_Z]) - anchor) for x in (JAW_X0, JAW_X1)))
    return out


def _face_near(objects, name):
    """Per jaw, in the fuse's frame: the point of its face nearest the fuse's axis (seen along the axis)."""
    res = []
    for a, b in jaw_faces(objects):
        qa, qb = gk.local(objects, name, a), gk.local(objects, name, b)
        d = qb - qa
        t = float(np.clip(-(qa[:2] @ d[:2]) / max(float(d[:2] @ d[:2]), 1e-12), 0.0, 1.0))
        res.append(qa + t * d)
    return res


def jaws_on(objects, name="fuse_f2"):
    """The puller's jaws closed round the fuse's body: each jaw's face within 1 mm of the tube's surface somewhere
    along its 18 mm, the two on opposite sides of the fuse, between its ferrules (the body, 19 mm long, plus 3 mm)."""
    near = _face_near(objects, name)
    if len(near) != 2:
        return False
    for q in near:
        if abs(q[2]) > (FUSE_L / 2 - FERRULE_L) + 0.003 or math.hypot(q[0], q[1]) > FUSE_R + JAW_REACH:
            return False
    a, b = (np.array([q[0], q[1]]) / max(math.hypot(q[0], q[1]), 1e-9) for q in near)
    return float(a @ b) < -0.5


def jaw_touching(objects, name="fuse_f2"):
    """Either jaw's face within 2 mm of the fuse anywhere along it (the jaws still on it)."""
    for q in _face_near(objects, name):
        if abs(q[2]) <= FUSE_L / 2 + 0.002 and math.hypot(q[0], q[1]) <= max(FUSE_R, BODY_HALF * math.sqrt(2)) + 0.002:
            return True
    return False


def hand_on(objects, name="fuse_f2"):
    """A gripper touching the fuse: its own shape (palm and fingers at their present opening) within 2 mm."""
    return bool(gk.touching(objects, name, FUSE_SHAPE, grippers=tuple(g for g in GRIPPERS if g in objects)))


def in_tray(objects, name="fuse_f2"):
    """The fuse's middle inside the tray's walls and below its rim."""
    q = gk.local(objects, "tray", objects[name].pos)
    L, W, H = TRAY_SIZE
    return abs(q[0]) < L / 2 - 0.002 and abs(q[1]) < W / 2 - 0.002 and -H / 2 < q[2] < H / 2 + 0.002


def in_enclosure(objects, name="fuse_f2"):
    p = np.asarray(objects[name].pos, float)
    return ENC_FRONT < p[0] < ENC_BACK and abs(p[1]) < ENC_W / 2 and TOP < p[2] < TOP + ENC_H


def still(objects, name="fuse_f2"):
    v = getattr(objects[name], "lin_vel", None)
    return v is None or float(np.linalg.norm(np.asarray(v, float))) < STILL


def _arm_box(s):
    """One arm of the puller, loop to jaw tip, as a block in its link's frame (for the hand-contact check)."""
    anchor = np.array([HINGE_X, s * 0.0035, 0.0])
    lo = np.array([0.010, s * 0.0025, 0.0]) - anchor
    hi = np.array([JAW_X1 + 0.004, s * (ARM_Y1 + ARM_T / 2), 0.012]) - anchor
    a, b = np.minimum(lo, hi), np.maximum(lo, hi)
    return gk.Block(tuple((a + b) / 2), tuple(b - a))


ARM_BOX = {"left": _arm_box(1), "right": _arm_box(-1)}


def puller_held(objects):
    """A gripper touching either arm of the puller: its own shape within 2 mm (gk.touching)."""
    grippers = tuple(g for g in GRIPPERS if g in objects)
    return any(gk.touching(objects, f"puller/{side}", box, grippers=grippers)
               for side, box in ARM_BOX.items() if f"puller/{side}" in objects)


def puller_down(objects):
    """The puller put down: lying flat on the bench (its underside within 1 cm of the bench top, tilted under 20
    degrees), outside the enclosure and the tray, and no gripper touching it."""
    st = objects.get("puller")
    if st is None:
        return False
    p = np.asarray(st.pos, float)
    flat = float(gk._rot(st.quat_xyzw)[2, 2]) > math.cos(math.radians(20.0))
    q = gk.local(objects, "tray", p) if "tray" in objects else np.full(3, 1.0)
    on_tray = abs(q[0]) < TRAY_SIZE[0] / 2 + 0.01 and abs(q[1]) < TRAY_SIZE[1] / 2 + 0.01
    inside = ENC_FRONT - 0.005 < p[0] and abs(p[1]) < ENC_W / 2 + 0.005
    return flat and p[2] < TOP + 0.010 and not on_tray and not inside and not puller_held(objects)


class _Watch(gk._PerStep):
    """Per control step: a neighbour knocked out of its clips (latched: score 0), and F2 leaving its clips without the
    puller's jaws round it (latched: score at most 0.5)."""

    def _fresh(self):
        self.prev_out, self.prev_jaws, self.bare, self.neighbour = False, False, False, False
        self.knock = gk.Latch(steps=2)

    def feed(self, objects):
        if self._tick(objects):
            hit = any(any(ends_out(objects, n)) for n in ("fuse_f1", "fuse_f3") if n in objects)
            self.neighbour = self.knock.feed(objects, hit)
            both = all(ends_out(objects, "fuse_f2"))
            jaws = jaws_on(objects)
            if both and not self.prev_out and not (jaws or self.prev_jaws):
                self.bare = True
            self.prev_out, self.prev_jaws = both, jaws
        return self


WATCH = _Watch()


def _job(objects):
    w = WATCH.feed(objects)
    if w.neighbour:
        return 0.0
    out = ends_out(objects, "fuse_f2")
    held = jaws_on(objects) or jaw_touching(objects) or hand_on(objects)
    if all(out):
        if in_tray(objects):
            done = 1.0 if (not held and still(objects) and puller_down(objects)) else 0.75
        elif in_enclosure(objects) and not held:
            done = 0.25  # dropped inside the panel: loose in the enclosure, it has to be found before power goes back
        else:
            done = 0.5
    elif any(out):
        done = 0.25  # one end out, the fuse cocked in the block
    else:
        done = 0.0
    if w.bare:
        done = min(done, 0.5)
    return done


def success(objects):
    return _job(objects) == 1.0


def score(objects):
    """F2's state at the end, after two checks kept over the whole run. 0 (irreversible) if F1 or F3 is ever out of
    either clip (a ferrule's centre more than 2.5 mm out of its seat, past the fingers' lips, or 3 mm off it sideways
    or 5 mm along it) for two control steps in a row. At most 0.5 (irreversible) if F2 ever leaves its second clip
    without the puller's jaws closed round its body (both jaw faces within 1 mm of its surface, on opposite sides,
    between its ferrules) at that step or the step before. Otherwise: 1.0 with F2 out of both clips, its middle inside
    the parts tray, nothing holding it (no jaw face within 2 mm of it, no gripper touching it: the gripper's own shape
    within 2 mm), still (under 2 cm/s), and the puller put down (lying flat on the bench outside the enclosure and the
    tray, no gripper touching it); 0.75 with it in the tray but still held or moving, or the puller not put down; 0.5 with it out of both
    clips anywhere else (held, or on the bench), except loose inside the enclosure, 0.25 (dropped into the panel);
    0.25 with one end out of its clip (cocked in the block); 0 with it still in both clips. Where the arms end up and
    where on the bench the puller is laid are not scored. Possible: 0, 0.25, 0.5, 0.75, 1."""
    return _job(objects)


# --- Demo -------------------------------------------------------------------------------------------------------------
W_GRIP = 0.0939  # m: the fingers' gap fully open (FINGER_OPEN)


def _finger(gap):
    """Finger target for a gap between the pads of `gap` m."""
    return FINGER_OPEN * min(max(gap, 0.0), W_GRIP) / W_GRIP


REST_W = 2 * (arm_y(GRASP_X) + ARM_T / 2)  # the handles' outside width at the pinch, at rest (22.1 mm)
HOLD_PITCH = math.radians(-10.0)  # the hand tilted, fingertips forward: the puller held level, pointing into the panel


def demo():
    """Right arm: pinches the puller's notched handles at its balance point where it lies, lifts it, brings it to F2
    pointing into the panel (perpendicular to the fuse) with its jaws open either side of the fuse's body, slides them
    on and squeezes, draws the fuse straight out of both clips toward the robot, carries it to the tray, sets it down on
    the tray's floor, eases the squeeze so the jaws spring open, lifts the puller clear, lays it back flat on its spot
    on the bench as it was found and lets go. Left arm stays at its start."""
    reach = JAW_X - GRASP_X  # the jaws' middle ahead of the tool point
    up = GRASP_Z - JAW_Z  # the tool point above the jaws' middle
    gx, gy = PULLER_START[0] + GRASP_X, PULLER_START[1]
    light = _finger(REST_W - 0.0015)  # a firm pinch: jaws still about 14 mm open
    wide = _finger(0.040)

    def w(dur, pos, finger, label):
        return kit.Tilted(dur, tuple(float(v) for v in pos), finger, label, pitch=HOLD_PITCH)

    # jaws' middle on F2's axis at its middle: the 10 mm jaws fill most of the 19 mm opening between the barriers
    at = (X_F - reach, 0.0, Z_C + up)
    r = [w(2.0, (gx, gy, TOP + 0.08), wide, "over the puller"),
         w(1.2, (gx, gy, TOP + GRASP_Z + 0.0005), wide, "down to the handles"),
         w(0.8, (gx, gy, TOP + GRASP_Z + 0.0005), light, "pinch the handles"),
         w(0.3, (gx, gy, TOP + GRASP_Z + 0.0005), light, "hold"),
         w(1.2, (gx, gy, TOP + 0.10), light, "lift"),
         w(2.2, (at[0] - 0.045, 0.0, at[2] + 0.02), light, "in front of F2"),
         w(1.0, (at[0] - 0.040, 0.0, at[2]), light, "jaws level with F2's body"),
         w(2.0, at, light, "jaws either side of the body"),
         w(1.0, at, FINGER_CLOSED, "squeeze"),
         w(0.4, at, FINGER_CLOSED, "hold")]
    tx, ty = TRAY[0] + 0.03 - reach, TRAY[1]  # the fuse set down 3 cm past the tray's middle, fingers clear of its wall
    down = TOP + 0.002 + FUSE_L / 2 + 0.002 + up  # the fuse's bottom 2 mm over the tray's floor
    p = at
    r += [w(2.0, (p[0] - 0.035, p[1], p[2]), FINGER_CLOSED, "draw it out of both clips"),
          w(1.2, (p[0] - 0.050, p[1], p[2] + 0.02), FINGER_CLOSED, "clear of the block"),
          w(2.0, (p[0] - 0.090, -0.06, TOP + 0.13), FINGER_CLOSED, "out of the enclosure"),
          w(3.0, (tx, ty, TOP + 0.10), FINGER_CLOSED, "over the tray"),
          w(1.5, (tx, ty, TOP + 0.050), FINGER_CLOSED, "lower into the tray"),
          w(1.0, (tx, ty, down), FINGER_CLOSED, "set it down on the tray's floor"),
          w(0.6, (tx, ty, down), light, "ease the squeeze: the jaws spring open"),
          w(0.6, (tx, ty, down), light, "let go"),
          w(1.2, (tx, ty, TOP + 0.12), light, "lift the puller clear"),
          w(2.0, (gx, gy, TOP + 0.08), light, "back over its spot on the bench"),
          w(1.2, (gx, gy, TOP + GRASP_Z + 0.002), light, "lay it down where it was"),
          w(0.6, (gx, gy, TOP + GRASP_Z + 0.002), wide, "let go of it"),
          w(1.0, (gx, gy, TOP + 0.10), wide, "lift clear"),
          w(1.5, (0.22, -0.02, TOP + 0.18), wide, "away, clear of the tray in the top camera's view"),
          w(3.0, (0.22, -0.02, TOP + 0.18), wide, "hold")]
    total = sum(w.duration for w in r)
    return {"left": [kit.Home(total)], "right": r}
