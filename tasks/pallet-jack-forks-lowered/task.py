"""A loaded pallet sits on the forks of a manual pallet jack, and a bay is taped on the floor 17 cm ahead. Material
movers pump the handle to raise the forks until the pallet rides clear of the floor, roll the jack to the bay, then
hold the release pedal until the forks are fully down and leave the handle upright before walking away. Leaving the
forks raised (a trip hazard and a load that can roll off with the jack) or dragging the pallet along the floor with
the forks down is the wrong answer. 1:8 scale.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props and the jack; nothing on disk changes)
import mechkit as mk  # noqa: E402  (the pump, the release pedal and the handle's upright detent)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Move the loaded pallet into the taped bay with the pallet jack."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the aisle is clear for
# the next person or job. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# --- The pallet jack, in its own frame: origin on the floor under the handle's pivot, x toward the fork tips, y across
# the jack, z up. In the scene the forks point to -y, so its x is the world's -y and its y the world's x. 1:8 scale of
# a standard 2,500 kg hand pallet truck: 1,150 mm forks, 685 mm wide, 85 mm lowered, 200 mm raised.
PIVOT_Z = 0.044  # the handle's pivot on the pump head (350 mm)
HANDLE_L = 0.108  # pivot to the middle of the grip bar: the grip is 15.2 cm above the floor (1,220 mm)
RAIL_Y = 0.026  # the handle's loop: two rails 5.2 cm apart (42 cm)
GRIP = (0.02, 0.060, 0.02)  # the grip bar: 2 cm square, 6 cm across
GRIP_ANGLE = math.radians(25)  # the bar's square section is turned 25 degrees, its top face toward the operator
FORK_X0, FORK_X1 = 0.035, 0.179  # the forks run from the fork frame to the tips: 14.4 cm (1,150 mm)
FORK_Y, FORK_W = 0.033, 0.020  # fork center lines 6.6 cm apart, each 2 cm wide (160 mm): 8.6 cm overall (685 mm)
FORK_Z0, FORK_Z1 = 0.005, 0.0106  # lowered, the forks' undersides are 5 mm up and their tops 1.06 cm (85 mm)
LIFT = 0.0144  # full lift 1.44 cm (115 mm)
PER_STROKE = 0.0036  # each pump stroke raises the forks 3.6 mm (29 mm): four strokes for full lift
DROP = 0.0005  # m per control step with the pedal held: the forks come down at 1 cm/s
STEER_R, STEER_Y = 0.01125, 0.014  # two steer wheels 2.25 cm across (180 mm) under the pump
ROLLER_X, ROLLER_R = 0.165, 0.005  # a load roller 1 cm across (80 mm) under each fork, 1.4 cm from its tip
PEDAL_AT = (-0.012, -0.012, 0.024)  # the release pedal's hinge on the side of the pump unit, toward the robot
PEDAL_ARM = 0.052  # hinge to the middle of the pedal plate
HANDLE_ZONE = 0.12  # rad (7 degrees): the handle's return spring holds it upright within this; beyond it, it falls back
STROKE_LO, STROKE_HI = -0.45, -0.10  # rad: a stroke pulls the handle back past 26 degrees and returns it within 6
PEDAL_RELEASE = 0.15  # rad: the pedal pressed 8.6 degrees opens the release valve (8 mm at its plate)
JACK_MASS = 0.083  # with the forks, handle, pedal, wheels and lift flag 175 g: 90 kg at full size
TAG_OUT = 0.035  # the lift flag rises 3.5 cm as the forks rise their full 1.44 cm: 2 cm above the raised fork frame
TAG = (0.008, 0.008, 0.022)  # a green post 8 mm square and 2.2 cm tall

# --- The pallet: a single-deck stringer pallet 1,200 x 1,000 mm with a stack of cartons 480 mm high (about 150 kg).
PALLET = (0.150, 0.125)  # along the forks, across them
STRINGER_H, STRINGER_W = 0.0143, 0.0048  # three 2 x 5 stringers (38 x 114 mm)
DECK_T = 0.0024  # 19 mm deck boards
PALLET_H = STRINGER_H + DECK_T
LOAD = (0.130, 0.105, 0.060)
PALLET_MASS = 0.30
PALLET_X = FORK_X0 + 0.002 + PALLET[0] / 2  # the pallet's center along the forks, its back 2 mm from the fork frame
JACK_X = 0.28  # the jack's center line (world x)
PIVOT_Y0 = 0.21  # the handle's pivot at the start (world y)
JACK_YAW = -math.pi / 2
JACK_START = (JACK_X, PIVOT_Y0, TOP)
PALLET_START = (JACK_X, PIVOT_Y0 - PALLET_X, TOP + 0.0005)
# --- The bay: a rectangle of 1 cm yellow floor tape, 18 cm along the jack's travel and 15.5 cm across, inside: 1.5 cm
# (12 cm at full size) of room on each side of the pallet.
TRAVEL = 0.17
BAY = (JACK_X, PALLET_START[1] - TRAVEL)
BAY_IN = (0.155, 0.180)  # inside size along world x, world y
TAPE = 0.01

# --- Scoring
VIOLATION_STEPS = 2
CLEAR = 0.001  # m: the stringers' lowest point 1 mm (8 mm) off the floor: the whole load is on the forks, none on the
# floor. Two strokes leave the loaded pallet 2.4 mm up (19 mm) after the forks' 1.1 mm sag under the load.
NUDGE = 0.018  # m: before its first lift the pallet may shift this far on the floor: each pump stroke pulls the jack back
# toward the operator and the forks bear part of the load as they meet the deck, so the pallet rocks along with it
# (NUDGE_SEEN in the scripted demo, measured with PALLET_JACK_DEBUG=1), plus a 5 mm margin. 1.8 cm is 14 cm at full size
NUDGE_SEEN = 0.013
NUDGE_TOWARD = 0.005  # m: of that, at most 5 mm toward the bay: the pump's rocking is back toward the operator and
# sideways, so any real progress toward the bay before the first lift is the pallet being dragged on its stringers
SETTLE = 0.005  # m: after its first lift, all its slides on the floor added up (each landing's farthest shift from where it
# came down, summed, never reset by a later lift) may total 5 mm (4 cm), the forks setting it down
ROLL_START = 0.01  # m: carried 1 cm toward the bay: the lift is done and the roll has begun
MOVED = 0.03  # m: carried 3 cm closer to the bay (along the line from its start to the bay) counts as moving it
SEAT_X, SEAT_Y = 0.02, 0.02  # m: the pallet's center within 2 cm of its place on the forks, along and across them
TIPPED = 15.0  # degrees: the pallet tipped off the forks or over
IN_BAY = TAPE / 2  # m: every corner of the pallet's footprint inside the middle of the tape
DOWN_JOINT = 0.0015  # m: the forks within 1.5 mm (12 mm) of fully down
DOWN_Z = 0.002  # m: the pallet's lowest corner within 2 mm of the floor (resting on its stringers)
FLAT = 3.0  # degrees
PEDAL_UP = 0.10  # rad: the pedal let go (back within 6 degrees of its rest)
RELEASED_OPEN = 0.85
NEAR = 0.03  # m: a closed gripper within 3 cm of the jack's or the pallet's outline is holding them
STILL = 0.001  # m per control step (2 cm/s at 20 Hz)
HOME_TOL = 0.03
HOME_STILL = 0.001

RED = kit.Look((0.75, 0.09, 0.07), 0.45, 0.2)
DARK_RED = kit.Look((0.50, 0.05, 0.04), 0.5, 0.2)
CHROME = kit.Look((0.80, 0.80, 0.82), 0.2, 0.9)
BLACK = kit.Look((0.06, 0.06, 0.06), 0.7)
POLY = kit.Look((0.20, 0.20, 0.22), 0.8)  # gray polyurethane wheels
HUB = kit.Look((0.70, 0.70, 0.68), 0.4, 0.6)
WOOD = kit.Look((0.78, 0.62, 0.42), 0.85)
WOOD_DARK = kit.Look((0.62, 0.47, 0.30), 0.9)
KRAFT = kit.Look((0.52, 0.36, 0.20), 0.85)
TAPE_LOOK = kit.Look((0.74, 0.60, 0.42), 0.35)
LABEL = kit.Look((0.93, 0.93, 0.90), 0.7)
TAG_GREEN = kit.Look((0.15, 0.85, 0.20), 0.5)
YELLOW = kit.Look((0.95, 0.80, 0.10), 0.6)

ROLLING = kit.PhysMat(0.04, 0.03, "min")  # the load rollers (see fidelity): they slide like rolling wheels
GRIPPY = kit.GRIPPY

_BOX_FACES = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]


def _turned_box(center, size, angle):
    """The 8 corners of a box turned `angle` about the jack's y axis (its x axis tipping up toward z)."""
    c, h = np.asarray(center, float), np.asarray(size, float) / 2
    ca, sa = math.cos(angle), math.sin(angle)
    rot = np.array([[ca, 0, -sa], [0, 1, 0], [sa, 0, ca]])
    return [c + rot @ (h * np.array([sx, sy, sz])) for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]


def _jack():
    """A manual pallet jack: a red pump unit on two steer wheels, a hinged loop handle with a black grip bar, the forks
    (a link that the pump raises), a load roller under each fork tip, and a red release pedal on the pump unit."""
    # Base: the pump unit, its chrome ram, the steer-wheel yoke, the load rollers and their push rods.
    meshes = [kit.cylinder((0, 0, 0.029), 0.012, 0.026, RED),
              kit.cylinder((0, 0, 0.0415), 0.0065, 0.005, CHROME),
              kit.box((0, 0, 0.0185), (0.016, 2 * STEER_Y + 0.012, 0.005), DARK_RED),
              kit.cylinder((0, 0, PIVOT_Z), 0.004, 0.022, DARK_RED, axis="y")]
    cols = [kit.Collider((0, 0, 0.029), radius=0.012, height=0.026)]
    for s in (-1, 1):
        y = s * FORK_Y
        meshes += [kit.cylinder((ROLLER_X, y, ROLLER_R), ROLLER_R, 0.012, POLY, axis="y"),
                   kit.box((ROLLER_X, y, 0.007), (0.004, 0.014, 0.003), DARK_RED),  # roller yoke, inside the fork
                   kit.box((ROLLER_X - 0.006, y, 0.008), (0.012, 0.004, 0.002), DARK_RED)]  # push rod
        cols.append(kit.Collider((ROLLER_X, y, ROLLER_R), radius=ROLLER_R, shape="sphere", material=ROLLING))

    # The forks and the fork frame, raised by the pump.
    fm, fc = [], []
    fz, fh = (FORK_Z0 + FORK_Z1) / 2, FORK_Z1 - FORK_Z0
    for s in (-1, 1):
        y = s * FORK_Y
        c, sz = ((FORK_X0 + FORK_X1) / 2, y, fz), (FORK_X1 - FORK_X0, FORK_W, fh)
        fm.append(kit.box((c[0] - 0.002, y, FORK_Z1 - 0.001), (sz[0] - 0.004, FORK_W, 0.002), RED))  # top plate
        for w in (-1, 1):  # channel side walls (open underneath, over the roller linkage)
            fm.append(kit.box((c[0] - 0.002, y + w * (FORK_W / 2 - 0.001), fz), (sz[0] - 0.004, 0.002, fh), RED))
        fm.append(kit.box((FORK_X1 - 0.002, y, fz + 0.001), (0.004, FORK_W, fh - 0.002), DARK_RED))  # rounded tip
        fc.append(kit.Collider(c, size=sz))
    frame = ((0.0245, 0.0, 0.0175), (0.021, 2 * FORK_Y + FORK_W, 0.025))  # the fork frame behind the pallet
    fm.append(kit.box(*frame, RED))
    fc.append(kit.Collider(*frame))
    fm.append(kit.box((0.016, 0.0, 0.026), (0.010, 0.016, 0.008), DARK_RED))  # lift link to the pump ram
    forks = kit.Link("forks", "prismatic", "Z", (0.0, 0.0, 0.0), lower=0.0, upper=LIFT + 0.001, start=0.0, mass=0.045,
                     meshes=tuple(fm), colliders=tuple(fc), damping=0.0,
                     law=mk.Pump(handle="handle", stroke_lo=STROKE_LO, stroke_hi=STROKE_HI, per_stroke=PER_STROKE,
                                 top=LIFT, release="pedal", release_at=PEDAL_RELEASE, drop_rate=DROP, k=3000.0,
                                 damp=60.0))

    # The handle: a loop of red tube from the pivot to a black rubber grip bar, hinged across the jack.
    hm, hc = [kit.cylinder((0, 0, 0), 0.006, 0.014, RED, axis="y")], []
    for s in (-1, 1):
        y = s * RAIL_Y
        a = ((0, s * 0.005, 0.004), (0, y, 0.030))  # the neck splays out to the rails
        mid = ((a[0][1] + a[1][1]) / 2, (a[0][2] + a[1][2]) / 2)
        hm.append(kit.box((0, mid[0], mid[1]), (0.006, abs(a[1][1] - a[0][1]) + 0.006, 0.030), RED))
        c, sz = (0, y, (0.030 + HANDLE_L) / 2), (0.006, 0.006, HANDLE_L - 0.030)
        hm.append(kit.box(c, sz, RED))
        hc.append(kit.Collider(c, size=sz))
    grip = _turned_box((0, 0, HANDLE_L), GRIP, GRIP_ANGLE)
    hm.append(kit._mesh(grip, [4] * 6, [i for f in _BOX_FACES for i in f], BLACK))
    hc.append(kit.Collider((0, 0, 0), shape="hull", points=tuple(map(tuple, grip))))
    handle = kit.Link("handle", "revolute", "Y", (0.0, 0.0, PIVOT_Z), lower=-1.35, upper=0.05, start=0.0, mass=0.025,
                      meshes=tuple(hm), colliders=tuple(hc), damping=0.0,
                      law=mk.Detents(positions=(0.0,), labels=("UPRIGHT",), zone=HANDLE_ZONE, pull=0.6, damp=0.03,
                                     drag=0.05))

    # The release pedal: a red arm with a black ribbed plate, sticking out toward the robot, spring-returned up.
    pm = [kit.box((0, -0.022, 0.0), (0.006, 0.040, 0.004), RED),
          kit.box((0, -PEDAL_ARM, 0.0005), (0.020, 0.022, 0.003), RED)]
    pm += [kit.box((dx, -PEDAL_ARM, 0.0022), (0.0015, 0.020, 0.0008), BLACK) for dx in (-0.006, -0.002, 0.002, 0.006)]
    pc = (kit.Collider((0, -0.022, 0.0), size=(0.006, 0.040, 0.004)),
          kit.Collider((0, -PEDAL_ARM, 0.0005), size=(0.020, 0.022, 0.003)))
    pedal = kit.Link("pedal", "revolute", "X", PEDAL_AT, lower=-0.02, upper=0.40, start=0.0, mass=0.008, meshes=tuple(pm),
                     colliders=pc, damping=0.0, law=mk.Spring(rest=0.0, k=0.3, damp=0.02))

    # The lift flag (display only, for the overhead camera, which can't see a 1.44 cm lift): a green post hidden inside
    # the fork frame's far end (away from the robot) with the forks down, rising out of its top as they rise.
    ty = FORK_Y + FORK_W / 2 - TAG[1] / 2 - 0.002
    tag = kit.Link("tag", "prismatic", "Z", (frame[0][0], ty, FORK_Z0 + 0.002 + TAG[2] / 2), lower=0.0, upper=TAG_OUT, start=0.0, mass=0.002,
                   meshes=(kit.box((0, 0, 0), TAG, TAG_GREEN),), colliders=(kit.Collider((0, 0, 0), size=TAG),),
                   floating=True, damping=0.0,
                   law=mk.Follow(source="forks", ratio=TAG_OUT / LIFT, lo=0.0, hi=TAG_OUT, rate=0.006))

    def wheel(name, y):
        m = (kit.cylinder((0, 0, 0), STEER_R, 0.009, POLY, axis="y"),
             kit.cylinder((0, 0, 0), 0.006, 0.0105, HUB, axis="y"))
        lugs = tuple(kit.box((0.004 * math.cos(a), math.copysign(0.0055, y), 0.004 * math.sin(a)), (0.002, 0.001, 0.002),
                             BLACK) for a in (0, math.pi / 2, math.pi, 1.5 * math.pi))
        return kit.Link(name, "revolute", "Y", (0.0, y, STEER_R), lower=-math.inf, upper=math.inf, start=0.0,
                        mass=0.006, meshes=m + lugs, damping=0.0,
                        colliders=(kit.Collider((0, 0, 0), radius=STEER_R, shape="sphere"),),
                        law=mk.Free(drag=0.003))  # rolling resistance of the polyurethane tires

    size = (FORK_X1 + 0.02, 2 * FORK_Y + FORK_W + 0.04, PIVOT_Z + HANDLE_L + 0.02)
    return kit.Mechanism("jack", pos=JACK_START, size=size, mass=JACK_MASS, yaw=JACK_YAW, meshes=tuple(meshes),
                         colliders=tuple(cols),
                         links=(forks, handle, pedal, tag, wheel("wheel_a", -STEER_Y), wheel("wheel_b", STEER_Y)))


def _pallet():
    """A wooden single-deck stringer pallet with seven deck boards on three stringers, carrying a block of brown cartons
    (taped, with white labels on the side facing the robot). Origin at the middle of its footprint on the floor."""
    lx, ly = PALLET
    meshes, cols = [], []
    for y in (-(ly / 2 - STRINGER_W / 2), 0.0, ly / 2 - STRINGER_W / 2):
        c, sz = (0, y, STRINGER_H / 2), (lx, STRINGER_W, STRINGER_H)
        meshes.append(kit.box(c, sz, WOOD_DARK))
        cols.append(kit.Collider(c, size=sz))
    board = lx / 7 - 0.0025
    for i in range(7):
        x = -lx / 2 + board / 2 + i * (lx - board) / 6
        meshes.append(kit.box((x, 0, STRINGER_H + DECK_T / 2), (board, ly, DECK_T), WOOD))
    cols.append(kit.Collider((0, 0, STRINGER_H + DECK_T / 2), size=(lx, ly, DECK_T)))
    # The load: two layers of three cartons each way, as one stretch-wrapped block.
    z0 = PALLET_H
    c, sz = (0, 0, z0 + LOAD[2] / 2), LOAD
    meshes.append(kit.box(c, sz, KRAFT))
    cols.append(kit.Collider(c, size=sz))
    for x in (-LOAD[0] / 6, LOAD[0] / 6):  # seams between cartons on the top and sides
        meshes.append(kit.box((x, 0, z0 + LOAD[2] + 0.0003), (0.0008, LOAD[1] + 0.001, 0.0008), WOOD_DARK))
    for y in (-LOAD[1] / 6, LOAD[1] / 6):
        meshes.append(kit.box((0, y, z0 + LOAD[2] + 0.0003), (LOAD[0] + 0.001, 0.0008, 0.0008), WOOD_DARK))
    meshes.append(kit.box((0, 0, z0 + LOAD[2] / 2), (LOAD[0] + 0.0012, LOAD[1] + 0.0012, 0.0008), WOOD_DARK))
    for x in (-LOAD[0] / 3, 0.0, LOAD[0] / 3):  # tape over each top carton, and a label on each robot-facing carton
        meshes.append(kit.box((x, 0, z0 + LOAD[2] + 0.0006), (0.012, LOAD[1] + 0.0014, 0.0006), TAPE_LOOK))
        for z in (z0 + LOAD[2] / 4, z0 + 3 * LOAD[2] / 4):
            meshes.append(kit.box((x, LOAD[1] / 2 + 0.0005, z), (0.022, 0.001, 0.014), LABEL))
    return kit.Prop("pallet", pos=PALLET_START, size=(lx, ly, PALLET_H + LOAD[2]), mass=PALLET_MASS, yaw=JACK_YAW,
                    meshes=tuple(meshes), colliders=tuple(cols))


def _bay():
    """The bay: a rectangle of yellow floor tape, flat on the floor (nothing to bump into)."""
    ix, iy = BAY_IN
    hx, hy = ix / 2 + TAPE / 2, iy / 2 + TAPE / 2
    fx, fy = ix + 2 * TAPE, iy + 2 * TAPE
    strips = [((hx, 0, 0), (TAPE, fy, 0.001)), ((-hx, 0, 0), (TAPE, fy, 0.001)),
              ((0, hy, 0), (ix, TAPE, 0.001)), ((0, -hy, 0), (ix, TAPE, 0.001))]
    return kit.Prop("bay", pos=(BAY[0], BAY[1], TOP + 0.0005), size=(fx, fy, 0.001),
                    meshes=tuple(kit.box(c, s, YELLOW) for c, s in strips), colliders=())


OBJECTS = [_jack(), _pallet(), _bay()]


# --- Geometry the scorer reads


def _rot(quat_xyzw):
    x, y, z, w = (float(v) for v in quat_xyzw)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _tilt(quat_xyzw):
    """Degrees between the body's z axis and the vertical."""
    x, y = float(quat_xyzw[0]), float(quat_xyzw[1])
    return math.degrees(math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _corners(p):
    """The four corners of the pallet's footprint (the bottom of its stringers), in the world."""
    r = _rot(p.quat_xyzw)
    hx, hy = PALLET[0] / 2, PALLET[1] / 2
    return [np.asarray(p.pos, float) + r @ np.array([sx * hx, sy * hy, 0.0]) for sx in (-1, 1) for sy in (-1, 1)]


def _lowest(p):
    """Height of the pallet's lowest bottom corner."""
    return float(min(c[2] for c in _corners(p)))


def _footprint(p):
    """The middle of the pallet's footprint, (x, y)."""
    return np.asarray(p.pos, float)[:2]


def _on_forks(objects):
    """The pallet's center within 2 cm of its place on the forks, along and across them, and level with them."""
    j, p = objects["jack"], objects["pallet"]
    b = _rot(j.quat_xyzw).T @ (np.asarray(p.pos, float) - np.asarray(j.pos, float))
    return bool(abs(b[0] - PALLET_X) <= SEAT_X and abs(b[1]) <= SEAT_Y and _tilt(p.quat_xyzw) <= TIPPED)


def _carried(objects):
    """Riding clear of the floor on the raised forks: on the forks, with its lowest corner 1 mm off the floor, and the
    forks actually raised by the pump (kit level above 0 and the fork joint above DOWN_JOINT), so a pallet lifted by
    hand over forks that are still down does not count as carried by the jack."""
    return bool(_on_forks(objects) and _lowest(objects["pallet"]) >= TOP + CLEAR
                and mk.level(objects, "jack/forks") > 1e-9 and kit.joint(objects, "jack/forks").pos > DOWN_JOINT)


def _in_bay(objects):
    """Every corner of the pallet's footprint inside the middle of the bay's tape."""
    lim = np.asarray(BAY_IN) / 2 + IN_BAY
    return all(bool((np.abs(c[:2] - np.asarray(BAY)) <= lim).all()) for c in _corners(objects["pallet"]))


def _forks_down(objects):
    """The forks fully down: the pump's level back to 0 (the kit's memory), cross-checked with the fork joint within
    1.5 mm of its bottom, and the pallet resting flat on its stringers (lowest corner within 2 mm of the floor)."""
    p = objects["pallet"]
    return bool(mk.level(objects, "jack/forks") <= 1e-9 and kit.joint(objects, "jack/forks").pos <= DOWN_JOINT
                and _lowest(p) <= TOP + DOWN_Z and _tilt(p.quat_xyzw) <= FLAT)


def _parked(objects):
    """The handle upright (within the 7-degree zone its return spring holds) and the release pedal let go."""
    return bool(abs(kit.joint(objects, "jack/handle").pos) <= HANDLE_ZONE
                and kit.joint(objects, "jack/pedal").pos <= PEDAL_UP)


GRIPPERS = ("left_gripper", "right_gripper")
# The jack's outline in its own frame (low and high corners): handle back to fork tips, across the pedal, floor to grip.
JACK_BOX = ((-0.03, -0.075, 0.0), (FORK_X1, FORK_Y + FORK_W / 2, PIVOT_Z + HANDLE_L + 0.015))
PALLET_BOX = ((-PALLET[0] / 2, -PALLET[1] / 2, 0.0), (PALLET[0] / 2, PALLET[1] / 2, PALLET_H + LOAD[2]))


def _near(point, obj, low, high):
    """`point` within NEAR of the box from `low` to `high` in `obj`'s frame."""
    d = _rot(obj.quat_xyzw).T @ (np.asarray(point, float) - np.asarray(obj.pos, float))
    return bool(np.all(d >= np.asarray(low) - NEAR) and np.all(d <= np.asarray(high) + NEAR))


def _released(objects):
    """No gripper holds the jack or the pallet: each has its fingers open (past 0.85), or its tool point and pad tips
    more than 3 cm outside both outlines."""
    for g in GRIPPERS:
        if g not in objects or kit.opening(objects, g) > RELEASED_OPEN:
            continue
        for pt in (objects[g].low, objects[g].high, objects[g].pos):
            if _near(pt, objects["jack"], *JACK_BOX) or _near(pt, objects["pallet"], *PALLET_BOX):
                return False
    return True


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.


class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.start = None  # the pallet's footprint at the start
        self.pre_toward = 0.0  # m: the most it moved toward the bay on the floor before its first lift
        self.lifted = False  # has ridden clear of the floor 1 cm toward the bay (the roll has begun)
        self.pre = 0.0  # before the first lift: the footprint's farthest shift from its start
        self.landed = None  # after a lift: where the pallet came back down on the floor (None while riding clear)
        self.slid_done = 0.0  # floor slides after the first lift from earlier landings, summed
        self.slid_now = 0.0  # the farthest shift from the current landing point
        self.moved_ever = False  # carried 3 cm closer to the bay
        self.drag_run = self.tip_run = 0
        self.dragged = self.tipped = False  # irreversible
        self.last = None
        self.still = False
        self.home = None
        self.last_grip = None
        self.grips_still = False


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    grips = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    if _H.home is None:
        _H.home = grips
    _H.grips_still = _H.last_grip is not None and all(
        g in _H.last_grip and np.linalg.norm(p - _H.last_grip[g]) < HOME_STILL for g, p in grips.items())
    _H.last_grip = grips
    p, j = objects["pallet"], objects["jack"]
    pp, jp = np.asarray(p.pos, float), np.asarray(j.pos, float)
    foot = _footprint(p)
    if _H.start is None:
        _H.start = foot
    _H.still = _H.last is not None and (np.linalg.norm(pp - _H.last[0]) < STILL and np.linalg.norm(jp - _H.last[1]) < STILL)
    _H.last = (pp, jp)
    carried = _carried(objects)
    to_bay = np.asarray(BAY) - _H.start
    progress = float((foot - _H.start) @ to_bay) / float(np.linalg.norm(to_bay))  # m closer to the bay than at the start
    if carried:
        if progress >= ROLL_START:
            _H.lifted = True  # the roll has begun: from here on, floor slides count against SETTLE
        if _H.landed is not None:  # lifted again: that landing's slide stays counted
            _H.slid_done += _H.slid_now
            _H.landed, _H.slid_now = None, 0.0
        if progress >= MOVED:
            _H.moved_ever = True
    elif not _H.lifted:
        _H.pre = max(_H.pre, float(np.linalg.norm(foot - _H.start)))
        _H.pre_toward = max(_H.pre_toward, progress)
    else:
        if _H.landed is None:
            _H.landed = foot
        _H.slid_now = max(_H.slid_now, float(np.linalg.norm(foot - _H.landed)))
    # Dragged (or pushed) across the floor: more than NUDGE from its start before the first lift, or more than SETTLE of
    # floor slides in all after it. Never reset by lifting.
    over = _H.pre > NUDGE or _H.pre_toward > NUDGE_TOWARD or _H.slid_done + _H.slid_now > SETTLE
    _H.drag_run = _H.drag_run + 1 if over else 0
    _H.tip_run = _H.tip_run + 1 if _tilt(p.quat_xyzw) > TIPPED else 0
    if _H.drag_run >= VIOLATION_STEPS:
        _H.dragged = True
    if _H.tip_run >= VIOLATION_STEPS:
        _H.tipped = True
    if os.environ.get("PALLET_JACK_DEBUG"):
        print(f"pj jack {jp.round(4)} pallet {pp.round(4)} low {_lowest(p) - TOP:.4f} carried {carried} "
              f"forks {kit.joint(objects, 'jack/forks').pos:.4f} level {mk.level(objects, 'jack/forks'):.4f} "
              f"handle {kit.joint(objects, 'jack/handle').pos:.3f} pedal {kit.joint(objects, 'jack/pedal').pos:.3f} "
              f"pre {_H.pre:.4f} slid {_H.slid_done + _H.slid_now:.4f} moved {_H.moved_ever} dragged {_H.dragged} bay {_in_bay(objects)} score {_score(objects)} "
              f"grip {np.asarray(objects['left_gripper'].pos).round(3)} {kit.opening(objects, 'left_gripper'):.2f}", flush=True)


def _arms_home(objects):
    """Each gripper's tool point within 3 cm of where it was at the trial's first scored step, and still."""
    if not _H.home or not _H.grips_still:
        return False
    return all(np.linalg.norm(np.asarray(objects[g].pos, float) - p) < HOME_TOL for g, p in _H.home.items()
               if g in objects)


def _score(objects):
    if _H.dragged or _H.tipped:
        return 0.0
    if not _H.moved_ever:
        return 0.0  # nothing done, or the pallet never rode the raised forks toward the bay
    if not _in_bay(objects):
        return 0.25  # moved on the jack, but not inside the bay
    if not _forks_down(objects):
        return 0.25  # in the bay with the forks left up (the trap)
    if not (_parked(objects) and _released(objects) and _H.still):
        return 0.5  # forks down in the bay, but the handle not upright, the pedal or handle still held, or moving
    return 1.0 if (_arms_home(objects) or not REQUIRE_ARMS_HOME) else 0.9


def success(objects):
    """Full score: the pallet in the bay with the forks fully down, the handle upright and let go, and (with
    REQUIRE_ARMS_HOME) both arms home and still. Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if the pallet was dragged (2 steps in a row, irreversible): before the roll begins (carried 1 cm toward the
    bay) its footprint more than NUDGE (1.8 cm) from its start while on the floor, or after that its floor slides (each
    landing's farthest shift from where it came down, summed and never reset by a later lift) over 5 mm; 0 if it tipped
    past 15 degrees; 0 if it never rode the raised forks 3 cm closer to the bay (doing nothing, only pumping, or going
    the wrong way). Otherwise 0.25 with the pallet not fully inside the bay, 0.25 inside it with the forks not fully
    down, 0.5 with the forks down but the handle not upright, the pedal or the jack still held, or things moving, 0.9
    done with an arm not home and 1.0 done with both arms home. Possible scores: 0, 0.25, 0.5, 0.9, 1."""
    return _score(objects)


# --- The demo


def _tcp(pivot_y, theta):
    """The tool point at the grip bar's center with the handle's pivot at world y `pivot_y` and the handle pulled back
    `theta` radians toward the operator (toward +y)."""
    c, s = math.cos(theta), math.sin(theta)
    return (JACK_X, pivot_y + s * HANDLE_L, TOP + PIVOT_Z + c * HANDLE_L)


def demo():
    """Left arm: grip the handle's bar from above, pump the handle four times (pull it back 37 degrees and return it
    upright) to raise the forks fully, push the jack 17 cm forks-first into the bay, let go of the handle (it stays
    upright), press and hold the release pedal until the forks are down, let it up and return home. The right arm stays
    at home."""
    W = kit.Tilted
    g = GRIP_ANGLE
    y0, y1 = PIVOT_Y0, PIVOT_Y0 - TRAVEL
    stroke = 0.65

    def at(y, th, d, label, finger=FINGER_CLOSED):
        return W(d, _tcp(y, th), finger, label, roll=-(g + th))

    def back(p, dist):  # `dist` back out along the grip's approach (up and toward +y)
        return (p[0], p[1] + dist * math.sin(g), p[2] + dist * math.cos(g))

    p0 = _tcp(y0, 0.0)
    left = [W(1.5, back(p0, 0.05), FINGER_OPEN, "over the handle", roll=-g),
            W(1.0, p0, FINGER_OPEN, "down around the grip", roll=-g),
            W(0.8, p0, FINGER_CLOSED, "grip", roll=-g)]
    for _ in range(4):
        left += [at(y0, stroke * k / 3, 0.35, "pump: pull back") for k in range(1, 4)]
        left += [at(y0, stroke * (1 - k / 3), 0.35, "pump: return") for k in range(1, 4)]
    left += [at(y0, 0.0, 0.6, "raised")]
    left += [at(y0 - TRAVEL * k / 10, 0.0, 0.5, "push to the bay") for k in range(1, 11)]
    left += [at(y1, 0.0, 0.8, "stop")]
    p1 = _tcp(y1, 0.0)
    # The pedal's plate, in the world, with the jack where it stopped.
    px = JACK_X + PEDAL_AT[1] - PEDAL_ARM
    py = y1 - PEDAL_AT[0]
    pz = TOP + PEDAL_AT[2] + 0.002  # the plate's top
    high = TOP + 0.21  # clear above the handle's grip bar
    left += [W(0.8, p1, FINGER_OPEN, "let go", roll=-g),
             W(1.0, back(p1, 0.05), FINGER_OPEN, "back off the grip", roll=-g),
             W(1.0, (JACK_X, p1[1] + 0.04, high), FINGER_OPEN, "up, clear of the handle"),
             W(1.2, (px - 0.03, py, high), FINGER_CLOSED, "over, beside the pedal"),
             W(1.2, (px - 0.03, py, pz + PAD_TIP_PAST_TCP + 0.006), FINGER_CLOSED, "down beside the pedal"),
             W(0.8, (px, py, pz + PAD_TIP_PAST_TCP + 0.004), FINGER_CLOSED, "over the pedal's plate"),
             W(0.8, (px, py, pz + PAD_TIP_PAST_TCP - 0.013), FINGER_CLOSED, "press the pedal"),
             W(2.5, (px, py, pz + PAD_TIP_PAST_TCP - 0.013), FINGER_CLOSED, "hold: forks lowering"),
             W(0.8, (px, py, pz + PAD_TIP_PAST_TCP + 0.006), FINGER_CLOSED, "let the pedal up"),
             W(0.8, (px - 0.03, py, pz + PAD_TIP_PAST_TCP + 0.006), FINGER_CLOSED, "out, away from the handle"),
             W(1.0, (px - 0.03, py, high), FINGER_CLOSED, "up"),
             kit.Home(1.8),
             kit.Home(2.0, label="hold at home")]
    total = sum(w.duration for w in left)
    return {"left": left, "right": [kit.Home(total, label="stay home")]}
