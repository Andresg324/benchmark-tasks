"""A 20 x 16 x 6 in steel wall-mount control enclosure stands on the bench with its door off, locked out and tested
dead. On its back pan, three Eaton Bussmann CHM1DU modular fuse holders (F1, F2, F3 from the left) sit on a DIN rail,
each with a hinged carrier holding a 10 x 38 mm fuse. F2's fuse has blown. On the bench lie two opened Bussmann
cartons of spare 10 x 38 mm fuses, KTK-6 (6 A, three left) on the right and KTK-10 (10 A, two left) on the left, and a
parts tray. "The panel is locked out and tested dead. F2's fuse has blown: replace it with a new 6 A fuse from its
box, put the blown fuse in the parts tray, and close F2." The robot pulls F2's carrier open by its grip tab, lifts the
blown fuse out of the carrier's clips and drops it in the tray, takes a KTK-6 out of its carton, lays it into the
carrier's clips, and swings the carrier shut until it latches. F1 and F3 stay shut; the 10 A fuse never goes in F2.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props; nothing on disk changes)
import carrierkit as ck  # noqa: E402  (the carrier's clips and latch)
import guardkit as gk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = ("The panel is locked out and tested dead. F2's fuse has blown: replace it with a new 6 A fuse from its "
               "box, put the blown fuse in the parts tray, and close F2.")

# --- The enclosure: 20 x 16 x 6 in (508 x 406 x 152 mm) steel, standing on the bench, opening toward the robot.
ENC_FRONT, ENC_DEPTH, ENC_W, ENC_H, WALL = 0.273, 0.152, 0.406, 0.508, 0.0015
ENC_BACK = ENC_FRONT + ENC_DEPTH
PAN_FRONT = 0.408  # the back pan's face (on standoffs off the back wall)
RAIL_TOP = PAN_FRONT - 0.0075  # a 35 x 7.5 mm top-hat DIN rail, horizontal
ZC = TOP + 0.150  # the rail's (and the holders') middle above the bench
# --- The holders: Eaton Bussmann CHM1DU, one module, for 10 x 38 mm fuses (Eaton Technical Data 720147, dimensional
# data: 17.5 mm module, 73.9 mm tall, 83.2 mm deep off the rail; the body 17.3 mm wide). Frame of the holder row:
# origin on F2's front face at the rail's height, x into the panel, z up.
HOLDER_W, HOLDER_H, HOLDER_D = 0.0173, 0.0739, 0.0832
X_F = RAIL_TOP - HOLDER_D  # the holders' front faces
POLES = {"f1": 0.045, "f2": 0.0, "f3": -0.045}  # F1 on the robot's left; 45 mm apart (a blank module's gap between)
# The carrier: the front 20 mm of the holder, 60 mm tall, hinged at its bottom front edge (z -30 mm); it swings
# forward and down to its open stop. Link frame: origin on the hinge, axes as the row's at joint 0 (shut).
CAR_H, CAR_D, PLATE = 0.060, 0.020, 0.002
HINGE_Z = -0.030
OPEN = math.radians(90.0)  # the open stop: the carrier lies level (assumed; the spec sheet gives no opening angle)
LATCH_ZONE = math.radians(6.0)  # within this of shut the latch spring pulls it shut
DETENT = math.radians(8.0)  # within this of the open stop a spring holds it open
SNAP = ck.Snap(open=OPEN, zone=LATCH_ZONE, detent=DETENT, k_latch=0.5, k_open=0.5)  # N*m/rad (assumed, light)
TAB = ((-0.010, 0.046), (0.0, CAR_H))  # the red grip tab on the carrier's top front: (x, z) low and high corners
# --- The fuses: Eaton Bussmann KTK (fast-acting, 600 V, 10 x 38 mm midget; 13/32 x 1-1/2 in = 10.3 x 38.1 mm,
# melamine tube, nickel-plated end caps, 0.02 lb = 9 g).
FUSE_R, FUSE_L, FERRULE_L = 0.00515, 0.0381, 0.0095
END = (FUSE_L - FERRULE_L) / 2  # 14.3 mm: each ferrule's centre from the fuse's middle
BODY_HALF = 0.0048  # collider: the tube as a 9.6 mm square prism inside its 10.3 mm round
FUSE_MASS = 0.009
FX, FZ = PLATE + FUSE_R + 0.0003, CAR_H / 2  # the fuse's seated middle in the carrier's frame
END_GAP = 0.0009  # the fuse's end play between the carrier's end blocks, each end
SEAT = ck.Seat("holders", "f2", seat=(FX, 0.0, FZ), ends=(-END, END), hold=0.8, release=0.0025)
CAR_MASS = 0.012
# --- The cartons: Bussmann boxes of 10, opened, tuck lid folded back flat on the bench. 108 x 43 x 13 mm outside
# (ten fuses side by side; assumed, the carton is not dimensioned by Eaton), 1 mm card.
BOX_L, BOX_D, BOX_H, CARD = 0.108, 0.043, 0.013, 0.001
BOXES = {"box_6a": (0.165, -0.120), "box_10a": (0.165, 0.120)}
SPARES = {"spare_6a_1": ("box_6a", -0.040), "spare_6a_2": ("box_6a", -0.012), "spare_6a_3": ("box_6a", 0.030),
          "spare_10a_1": ("box_10a", -0.030), "spare_10a_2": ("box_10a", 0.010)}
GOOD = tuple(n for n in SPARES if "6a" in n)
WRONG = tuple(n for n in SPARES if "10a" in n)
FUSES = ("fuse_old",) + tuple(SPARES)
# --- The parts tray: 150 x 100 x 30 mm, on the bench to the right.
TRAY = (0.200, -0.300)
TRAY_SIZE = (0.150, 0.100, 0.030)

STEEL = kit.Look((0.80, 0.80, 0.78), 0.5)  # light grey painted enclosure (RAL 7035)
PAN = kit.Look((0.92, 0.92, 0.90), 0.6)  # white back pan
RAIL = kit.Look((0.75, 0.76, 0.78), 0.3, metallic=0.8)
GREY = kit.Look((0.72, 0.72, 0.70), 0.6)  # the holders' grey moulding
RED = kit.Look((0.80, 0.10, 0.10), 0.5)  # the grip tab
SCREW = kit.Look((0.62, 0.63, 0.66), 0.4, metallic=0.8)
DARK = kit.Look((0.10, 0.10, 0.10), 0.6)
TUBE = kit.Look((0.88, 0.87, 0.82), 0.5)  # melamine tube
CAP = kit.Look((0.78, 0.79, 0.81), 0.35, metallic=0.7)
CARD_LOOK = kit.Look((0.93, 0.80, 0.20), 0.8)  # the carton's printed card
BAND = {"box_6a": kit.Look((0.15, 0.25, 0.60), 0.7), "box_10a": kit.Look((0.15, 0.15, 0.15), 0.7)}
TRAY_LOOK = kit.Look((0.15, 0.30, 0.65), 0.5)


def _box(lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return tuple(float(v) for v in (lo + hi) / 2), tuple(float(v) for v in hi - lo)


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


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
    for z0, z1 in ((0.0, 0.012), (ENC_H - 0.012, ENC_H)):  # the door's flange lip round the opening (drawn)
        c, s = _box((ENC_FRONT - cx, -ENC_W / 2, z0 - cz), (ENC_FRONT + 0.003 - cx, ENC_W / 2, z1 - cz))
        meshes.append(kit.box(c, s, STEEL))
    zr = ZC - TOP
    c, s = _box((RAIL_TOP - cx, -0.160, zr - 0.0175 - cz), (PAN_FRONT - cx, 0.160, zr + 0.0175 - cz))
    meshes.append(kit.box(c, s, RAIL))
    colliders.append(kit.Collider(c, size=s))
    for k in range(6):  # grey feed-through terminal blocks on the same rail, right of the holders (drawn)
        y0 = -0.120 + 0.0065 * k
        c, s = _box((RAIL_TOP - 0.045 - cx, y0, zr - 0.030 - cz), (RAIL_TOP - cx, y0 + 0.006, zr + 0.030 - cz))
        meshes.append(kit.box(c, s, kit.Look((0.55, 0.57, 0.60), 0.5)))
        colliders.append(kit.Collider(c, size=s))
    return kit.Prop("enclosure", pos=(cx, 0.0, TOP + cz), size=(ENC_DEPTH, ENC_W, ENC_H), mass=None,
                    meshes=tuple(meshes), colliders=tuple(colliders))


def _fuse_meshes(center=(0.0, 0.0, 0.0), axis="z"):
    """A KTK fuse's look: melamine tube, nickel caps, a printed band. `axis` its length."""
    c = np.asarray(center, float)
    k = "xyz".index(axis)
    body = FUSE_L - 2 * FERRULE_L
    out = [kit.cylinder(tuple(c), FUSE_R - 0.0002, body + 0.001, TUBE, axis=axis)]
    for e in (-END, END):
        p = c.copy()
        p[k] += e
        out.append(kit.cylinder(tuple(p), FUSE_R, FERRULE_L, CAP, axis=axis))
    return out


def _carrier(name, y, with_fuse):
    """One holder's hinged carrier (link frame at its hinge): front plate with the red grip tab on top, the two end
    blocks with the contact clips (the clip fingers reach the ferrules' widest point), and, for F1 and F3, the fuse
    they hold (part of the carrier: those carriers are never meant to open)."""
    meshes, colliders = [], []
    W = HOLDER_W / 2

    def part(lo, hi, look=GREY, collide=True):
        c, s = _box(lo, hi)
        meshes.append(kit.box(c, s, look))
        if collide:
            colliders.append(kit.Collider(c, size=s))

    part((0.0, -W, 0.0), (PLATE, W, CAR_H))  # front plate
    (tx0, tz0), (tx1, tz1) = TAB
    part((tx0, -W, tz0), (tx1, W, tz1), RED)  # grip tab
    lo_end, hi_end = FZ - FUSE_L / 2 - END_GAP, FZ + FUSE_L / 2 + END_GAP
    part((PLATE, -W, 0.0), (FX + 0.003, W, lo_end))  # the end blocks (clip housings), 3 mm past the fuse's axis
    part((PLATE, -W, hi_end), (FX + 0.003, W, CAR_H - 0.006))
    part((PLATE, -W, CAR_H - 0.006), (CAR_D, W, CAR_H))  # the carrier's top, under the tab, its full depth
    for z0 in (FZ - FUSE_L / 2 - END_GAP, FZ + END + 0.0002):  # the clips' fingers beside each ferrule's outer part,
        for s in (-1, 1):  # up to 2 mm short of its widest point (the clip's spring is carrierkit's law)
            lo, hi = sorted((s * (FUSE_R + 0.0006), s * W))
            part((PLATE, lo, z0), (FX - 0.002, hi, z0 + 0.0050 + END_GAP - 0.0004), CAP)
    for k in range(int(name[-1])):  # one, two or three dark ticks on the tab: the circuit number at a glance
        part((tx0 - 0.0004, -0.006 + 0.004 * k, tz0 + 0.004), (tx0, -0.004 + 0.004 * k, tz1 - 0.004), DARK, False)
    if with_fuse:
        meshes += _fuse_meshes((FX, 0.0, FZ))
        colliders.append(kit.Collider((FX, 0.0, FZ), radius=FUSE_R, height=FUSE_L))
    return kit.Link(name, "revolute", "Y", (0.0, y, HINGE_Z), lower=-OPEN, upper=0.0, start=0.0, mass=CAR_MASS,
                    meshes=tuple(meshes), colliders=tuple(colliders), damping=SNAP.damp, law=SNAP)


def _holders():
    """The three CHM1DU holders on the rail (fixed): each the moulding behind and above and below its carrier, with
    its two screw terminals; the carriers are links."""
    meshes, colliders = [], []
    W = HOLDER_W / 2

    def part(lo, hi, look=GREY, collide=True):
        c, s = _box(lo, hi)
        meshes.append(kit.box(c, s, look))
        if collide:
            colliders.append(kit.Collider(c, size=s))

    for y in POLES.values():
        part((CAR_D + 0.0015, y - W, -HOLDER_H / 2), (HOLDER_D, y + W, HOLDER_H / 2))  # behind the carrier
        part((0.0, y - W, HINGE_Z + CAR_H + 0.004), (CAR_D + 0.0015, y + W, HOLDER_H / 2))  # above it
        part((0.0, y - W, -HOLDER_H / 2), (CAR_D + 0.0015, y + W, HINGE_Z - 0.0005))  # below it
        for z in (HOLDER_H / 2 - 0.0015, -HOLDER_H / 2 + 0.0015):  # the screw terminals' openings (drawn)
            part((-0.0004, y - 0.0035, z - 0.0010), (0.0, y + 0.0035, z + 0.0010), SCREW, False)
    return kit.Mechanism("holders", pos=(X_F, 0.0, ZC), size=(HOLDER_D + 0.02, 0.11, HOLDER_H), mass=None,
                         meshes=tuple(meshes), colliders=tuple(colliders),
                         links=tuple(_carrier(n, y, n != "f2") for n, y in POLES.items()))


def _carton(name):
    """An opened carton (free on the bench). Frame: its middle; its length across the bench (y), the lid folded back
    flat behind it (+x)."""
    L, D, H, t = BOX_L, BOX_D, BOX_H, CARD
    parts = [((-D / 2, -L / 2, -H / 2), (D / 2, L / 2, -H / 2 + t)),
             ((-D / 2, -L / 2, -H / 2), (-D / 2 + t, L / 2, H / 2)), ((D / 2 - t, -L / 2, -H / 2), (D / 2, L / 2, H / 2)),
             ((-D / 2, -L / 2, -H / 2), (D / 2, -L / 2 + t, H / 2)), ((-D / 2, L / 2 - t, -H / 2), (D / 2, L / 2, H / 2)),
             ((D / 2, -L / 2, -H / 2), (D / 2 + D, L / 2, -H / 2 + t))]  # the lid, folded back flat
    meshes, colliders = [], []
    for lo, hi in parts:
        c, s = _box(lo, hi)
        meshes.append(kit.box(c, s, CARD_LOOK))
        colliders.append(kit.Collider(c, size=s))
    c, s = _box((-D / 2 - 0.0004, -L / 2 + 0.010, -H / 2 + 0.003), (-D / 2, L / 2 - 0.010, H / 2 - 0.003))
    meshes.append(kit.box(c, s, BAND[name]))  # the printed catalogue band on the carton's front
    c, s = _box((D / 2 + 0.006, -L / 2 + 0.010, -H / 2 + t), (D / 2 + D - 0.006, L / 2 - 0.010, -H / 2 + t + 0.0004))
    meshes.append(kit.box(c, s, BAND[name]))  # and on the lid's inside
    x, y = BOXES[name]
    return kit.Prop(name, pos=(x, y, TOP + H / 2), size=(3 * D, L, H), mass=0.012, meshes=tuple(meshes),
                    colliders=tuple(colliders))


def _fuse(name, pos, quat):
    """A loose fuse. Frame: its middle, z along its axis."""
    colliders = [kit.Collider((0, 0, 0), size=(2 * BODY_HALF, 2 * BODY_HALF, FUSE_L - 2 * FERRULE_L))]
    colliders += [kit.Collider((0, 0, z), radius=FUSE_R, height=FERRULE_L) for z in (-END, END)]
    meshes = _fuse_meshes()
    if name == "fuse_old":  # the blown fuse: marked with a black stripe round its middle
        meshes.append(kit.cylinder((0, 0, 0), FUSE_R - 0.0001, 0.006, DARK))
    return kit.Prop(name, pos=tuple(float(v) for v in pos), size=(2 * FUSE_R, 2 * FUSE_R, FUSE_L), mass=FUSE_MASS,
                    meshes=tuple(meshes), colliders=tuple(colliders), tilt_xyzw=tuple(quat))


LYING = (0.0, math.sin(math.pi / 4), 0.0, math.cos(math.pi / 4))  # axis along x (toward the panel)
HINGE_W = np.array([X_F, 0.0, ZC + HINGE_Z])  # F2's hinge, world


def spare_start(name):
    box, dy = SPARES[name]
    x, y = BOXES[box]
    return np.array([x, y + dy, TOP + CARD + FUSE_R + 0.0003])


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


OBJECTS = ([_enclosure(), _holders()] + [_carton(n) for n in BOXES]
           + [_fuse("fuse_old", HINGE_W + np.array([FX, 0.0, FZ]), (0.0, 0.0, 0.0, 1.0))]
           + [_fuse(n, spare_start(n), LYING) for n in SPARES] + [_tray()])
ck.register(SEAT, FUSES, start=("fuse_old",))
GRIPPERS = ("left_gripper", "right_gripper")

# --- Readouts ---------------------------------------------------------------------------------------------------------
FUSE_SHAPE = gk.Seg((0.0, 0.0, -FUSE_L / 2), (0.0, 0.0, FUSE_L / 2), FUSE_R)
CARRIER_SHAPE = gk.Block(((TAB[0][0] + CAR_D) / 2, 0.0, CAR_H / 2), (CAR_D - TAB[0][0], HOLDER_W, CAR_H))
SHUT = math.radians(2.0)  # a carrier within 2 degrees of its face is shut (flush, from a side photo)
OPENED = math.radians(30.0)  # F2 counts as opened past 30 degrees
NEIGHBOUR = math.radians(5.0)  # F1 or F3 opened 5 degrees or more: a neighbour opened
SEATED = 0.001  # m: both ferrules within 1 mm of the clip floor, on their clips
STILL = 0.02  # m/s


def opening(objects, n):
    """How far carrier `n` ("f1", "f2", "f3") is open (rad, 0 shut), from its link's orientation."""
    R = gk._rot(objects[f"holders/{n}"].quat_xyzw)
    return -math.atan2(R[0, 2], R[2, 2])


def _car(objects):
    s = objects["holders/f2"]
    return s.pos, s.quat_xyzw


def fuse_state(objects, name):
    """(out_of_clip [2], offsets [2]) of a fuse against F2's carrier clips."""
    s = objects[name]
    return ck.seat_state(s.pos, s.quat_xyzw, *_car(objects), SEAT)


def seated_in_f2(objects, name):
    """The fuse lies down in F2's two clips (both ferrules within 1 mm of the floor, on their clips)."""
    return all(ck.seated(d, SEAT, SEATED) for d in fuse_state(objects, name)[1])


def out_of_f2(objects, name="fuse_old"):
    """Both ferrules out of F2's clips."""
    return all(fuse_state(objects, name)[0])


def in_tray(objects, name):
    q = gk.local(objects, "tray", objects[name].pos)
    L, W, H = TRAY_SIZE
    return abs(q[0]) < L / 2 - 0.002 and abs(q[1]) < W / 2 - 0.002 and -H / 2 < q[2] < H / 2 + 0.002


def in_box(objects, name):
    """The fuse's middle inside its carton's walls and below its rim (2 mm over it counts)."""
    box = SPARES[name][0]
    q = gk.local(objects, box, objects[name].pos)
    return abs(q[0]) < BOX_D / 2 and abs(q[1]) < BOX_L / 2 and q[2] < BOX_H / 2 + 0.002


def hand_on(objects, name):
    """A gripper touching it: the gripper's own shape (palm and fingers at their present opening) within 2 mm."""
    shape = CARRIER_SHAPE if name == "holders/f2" else FUSE_SHAPE
    return bool(gk.touching(objects, name, shape, grippers=tuple(g for g in GRIPPERS if g in objects)))


def still(objects, name):
    v = getattr(objects[name], "lin_vel", None)
    return v is None or float(np.linalg.norm(np.asarray(v, float))) < STILL


class _Watch(gk._PerStep):
    """Per control step, latched for the trial (2 steps in a row, so a one-step physics pop does not count): F1 or F3
    opened 5 degrees or more; a 10 A fuse seated in F2's clips; the blown fuse seated in F2's clips again after it had
    been out of both. Any of them makes the trial's score 0."""

    def _fresh(self):
        self.neighbour, self.wrong, self.back = gk.Latch(2), gk.Latch(2), gk.Latch(2)
        self.old_was_out = False

    def feed(self, objects):
        if self._tick(objects):
            self.neighbour.feed(objects, any(opening(objects, n) >= NEIGHBOUR for n in ("f1", "f3")))
            self.wrong.feed(objects, any(seated_in_f2(objects, n) for n in WRONG))
            self.old_was_out = self.old_was_out or out_of_f2(objects)
            self.back.feed(objects, self.old_was_out and seated_in_f2(objects, "fuse_old"))
        return self

    @property
    def hazard(self):
        return self.neighbour.on or self.wrong.on or self.back.on


WATCH = _Watch()


def _job(objects):
    if WATCH.feed(objects).hazard:
        return 0.0
    old_out = out_of_f2(objects)
    new = [n for n in GOOD if seated_in_f2(objects, n)]
    if new and old_out:
        f2 = opening(objects, "f2")
        done = (f2 <= SHUT and in_tray(objects, "fuse_old")
                and not any(hand_on(objects, n) for n in ("holders/f2", new[0], "fuse_old"))
                and still(objects, new[0]) and still(objects, "fuse_old"))
        return 1.0 if done else 0.75
    if old_out and any(not in_box(objects, n) for n in GOOD):
        return 0.5
    if old_out or opening(objects, "f2") >= OPENED:
        return 0.25
    return 0.0


def success(objects):
    return _job(objects) == 1.0


def score(objects):
    """0 for the trial (irreversible, whatever was reached before or after) from the second control step in a row at
    which F1 or F3 is open 5 degrees or more, a 10 A spare lies seated in F2's clips, or the blown fuse lies seated in
    them again after it had been out. Otherwise, from the state now: 1.0 with a 6 A spare seated in F2's clips (both
    ferrules within 1 mm of the clip floor), F2 shut (within 2 degrees of flush), the blown fuse out of F2 and in the
    parts tray, no gripper touching F2's carrier, the new fuse or the blown one (the gripper's own shape within 2 mm),
    and both fuses still (under 2 cm/s); 0.75 with a 6 A spare seated in F2 and the blown fuse out, the rest not yet
    done; 0.5 with the blown fuse out of F2 and a 6 A spare out of its carton; 0.25 with the blown fuse out of F2, or
    F2 open 30 degrees or more; 0 otherwise. Where the arms end up is not scored. Possible: 0, 0.25, 0.5, 0.75, 1."""
    return _job(objects)


# --- Demo -------------------------------------------------------------------------------------------------------------
W_GRIP = 0.0939  # m: the fingers' gap fully open (FINGER_OPEN)


def _finger(gap):
    """Finger target for a gap between the pads of `gap` m."""
    return FINGER_OPEN * min(max(gap, 0.0), W_GRIP) / W_GRIP


def car_point(local, a):
    """World point of F2's carrier-frame point `local` with the carrier open `a` rad."""
    return HINGE_W + _rot_y(-a) @ np.asarray(local, float)


def _out_dir(a):
    """World direction out of F2's clips (from the floor to the open side) with the carrier open `a` rad."""
    return _rot_y(-a) @ np.array([1.0, 0.0, 0.0])


GRASP_UP = 0.0085  # the tool point 8.5 mm above a fuse's axis: the fingertips reach 4.3 mm past it, 1.2 mm off the floor
PRESS = FUSE_R + 0.0128 - 0.0010  # closed fingertips 1 mm into the top of a seated fuse: pressing it into its clips


def demo():
    """Left arm: pinches F2's grip tab from the front, pulls the carrier 35 degrees open and lets go (it drops to its
    open stop, lying level). Right arm: grips the blown fuse across its body from above, draws it out of the clips,
    drops it in the parts tray; picks the right-most KTK-6 out of its carton, lays it into the carrier's clips, lets
    go and presses it down into both clips with its closed fingertips. Left arm: pinches the open carrier's end the
    same way (fingertips into the panel), swings it back up along its arc until it is shut and the latch has caught,
    lets go, then presses its face home. Both arms end clear."""
    (tx0, tz0), (tx1, tz1) = TAB
    tab_w = _finger(HOLDER_W - 0.002)  # a light pinch on the 17.3 mm carrier: about 5 N
    wide = _finger(0.040)
    fwd = math.radians(-90.0)  # fingertips pointing into the panel

    def L(dur, pos, finger, label, pitch=fwd):
        return kit.Tilted(dur, tuple(float(v) for v in pos), finger, label, pitch=pitch)

    pinch = np.array([tx1 - 0.0131, 0.0, (tz0 + tz1) / 2])  # tool point, carrier frame: tips cover the tab's 10 mm
    park = (0.18, 0.20, ZC + 0.05)
    left = [L(1.5, (0.20, 0.24, ZC + 0.05), wide, "out in front of the enclosure"),
            L(2.0, car_point(pinch, 0.0) + np.array([-0.05, 0.0, 0.0]), wide, "in front of F2's tab"),
            L(1.2, car_point(pinch, 0.0), wide, "fingers either side of the tab"),
            L(0.6, car_point(pinch, 0.0), tab_w, "pinch the tab")]
    for k, deg in enumerate((10, 20, 35)):
        left.append(L(0.8 if k else 1.0, car_point(pinch, math.radians(deg)), tab_w, f"pull it open {deg} deg"))
    p = car_point(pinch, math.radians(35))
    left += [L(0.4, p, wide, "let go: it drops open"),
             L(1.0, p + np.array([-0.03, 0.0, 0.05]), wide, "clear"),
             L(1.5, park, wide, "out of the right hand's way")]
    t_left_open = sum(w.duration for w in left)

    # right arm: the blown fuse out, into the tray
    up = np.array([0.0, 0.0, 1.0])  # out of the open carrier's clips (it lies level)
    seat_w = car_point((FX, 0.0, FZ), OPEN)
    grip = _finger(0.0)
    open_f = _finger(0.024)
    lean = math.radians(-10.0)  # higher up, fingertips 10 degrees toward the panel (the arm's reach)

    def R(dur, pos, finger, label, pitch=0.0):
        return kit.Tilted(dur, tuple(float(v) for v in pos), finger, label, pitch=pitch)

    tray_drop = np.array([TRAY[0], TRAY[1], TOP + 0.04])  # the fuse let go about 2 cm over the tray's floor
    wait = (0.18, -0.20, TOP + 0.20)
    right = [R(t_left_open + 0.2, wait, open_f, "wait"),
             R(2.0, seat_w + 0.05 * up, open_f, "over the open carrier", lean),
             R(1.0, seat_w + 0.025 * up, open_f, "square over the blown fuse"),
             R(1.0, seat_w + GRASP_UP * up, open_f, "fingers either side of it"),
             R(0.6, seat_w + GRASP_UP * up, grip, "grip it"),
             R(1.2, seat_w + (GRASP_UP + 0.02) * up, grip, "draw it out of the clips"),
             R(1.0, seat_w + (GRASP_UP + 0.02) * up + np.array([-0.05, 0.0, 0.03]), grip, "clear of the holders", lean),
             R(1.5, (0.19, -0.15, TOP + 0.15), grip, "out of the enclosure", lean),
             R(2.0, tray_drop, grip, "over the tray", lean),
             R(0.6, tray_drop, wide, "drop it in the tray", lean),
             R(0.8, tray_drop + np.array([0.0, 0.0, 0.04]), wide, "up", lean)]
    # the new fuse out of the KTK-6 carton
    sp = spare_start("spare_6a_3")
    right += [R(1.6, sp + np.array([0.0, 0.0, 0.06]), open_f, "over the KTK-6 carton"),
              R(1.2, sp + GRASP_UP * up, open_f, "fingers either side of a fuse"),
              R(0.6, sp + GRASP_UP * up, grip, "grip it"),
              R(1.0, sp + 0.07 * up, grip, "lift it out of the carton"),
              R(2.0, seat_w + 0.05 * up, grip, "over the open carrier", lean),
              R(1.0, seat_w + 0.025 * up, grip, "square over the clips"),
              R(1.2, seat_w + (GRASP_UP + 0.002) * up, grip, "lay it into the clips"),
              R(0.6, seat_w + (GRASP_UP + 0.002) * up, open_f, "let go"),
              R(0.6, seat_w + 0.025 * up, grip, "lift, close the fingers"),
              R(0.8, seat_w + PRESS * up, grip, "press it down into both clips"),
              R(0.4, seat_w + PRESS * up, grip, "hold"),
              R(0.8, seat_w + 0.025 * up, open_f, "lift clear"),
              R(1.0, seat_w + 0.05 * up, open_f, "back out", lean),
              R(1.4, wait, open_f, "clear")]
    t_right = sum(w.duration for w in right)

    # left arm: the same pinch on the open carrier, swung back up along its arc
    left += [L(max(t_right - t_left_open, 0.5), park, wide, "wait for the right hand"),
             L(1.5, car_point(pinch, OPEN) + np.array([-0.05, 0.0, 0.0]), wide, "in front of the open carrier's end"),
             L(1.2, car_point(pinch, OPEN), wide, "fingers either side of it"),
             L(0.6, car_point(pinch, OPEN), tab_w, "pinch it")]
    for deg in (70, 50, 30, 15, 5, 0):
        left.append(L(0.9, car_point(pinch, math.radians(deg)), tab_w, f"swing it up to {deg} deg"))
    p = car_point(pinch, 0.0)
    left.append(L(0.5, p, tab_w, "hold it shut: the latch catches"))
    face = np.array([-0.0128 + 0.001, 0.0, CAR_H / 2 + 0.005])  # closed fingertips 1 mm into the shut face
    left += [L(0.4, p, wide, "let go"),
             L(0.8, p + np.array([-0.04, 0.0, 0.0]), wide, "back off"),
             L(1.0, car_point(face, 0.0) + np.array([-0.03, 0.0, 0.0]), FINGER_CLOSED, "in front of F2"),
             L(1.0, car_point(face, 0.0), FINGER_CLOSED, "press it home"),
             L(0.8, car_point(face, 0.0) + np.array([-0.05, 0.0, 0.0]), FINGER_CLOSED, "back off"),
             L(1.5, park, wide, "clear"),
             L(3.0, park, wide, "hold")]
    t_left = sum(w.duration for w in left)
    right.append(R(max(t_left - t_right, 0.5), wait, open_f, "hold"))
    return {"left": left, "right": right}
