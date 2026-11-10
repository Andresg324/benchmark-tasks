"""A boxed mini fridge stands on the floor, too wide for a hand to grip and too heavy to carry. A two-wheel hand truck
stands upright behind it, and a taped loading spot is marked on the floor 18 cm away. Material movers and delivery
drivers move a load like this with the hand truck: slide the nose plate fully under the carton (carton against the
frame), tilt the truck back so the weight rides on the axle, roll it to the spot, stand it up and slide the plate out.
Pushing or dragging the carton across the floor, lifting it, or tipping it over is the wrong answer. 1:5 scale.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props and the hand truck; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Move the boxed fridge onto the taped loading spot."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the workspace is clear for
# the next person or job. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# --- The hand truck, in its own frame: origin on the axle's center line, x forward (toward the nose plate's tip),
# y along the axle, z up when it stands upright. In the scene it faces -y, so its x is the world's -y and its y the
# world's x. 1:5 scale of a 100 cm light-duty hand truck with 10-inch wheels.
R = 0.025  # wheel radius: the axle stands 2.5 cm above the floor
WHEEL_Y, WHEEL_W = 0.052, 0.016  # wheel centers along the axle, and tire width
F = 0.018  # the frame's front face is 1.8 cm ahead of the axle
RAIL = (0.007, 0.008)  # rail section (x, y), square steel tube
RAIL_Y = 0.038  # rail center lines along the axle
RAIL_TOP = 0.169
PLATE_D, PLATE_W, PLATE_T = 0.070, 0.070, 0.003  # extended nose plate: 7 cm ahead of the frame (35 cm), 7 cm wide, 3 mm thick
BEVEL = 0.030  # its front 3 cm is beveled down to a 0.4 mm edge, so it slides under a carton
HANDLE = (-0.010, 0.0, 0.165)  # center of the grip bar: 2 cm square, 6.8 cm long, set 2.8 cm behind the frame face
GRIP = (0.020, 0.068, 0.020)
GRIP_ANGLE = math.radians(25)  # the grip's square section is turned 25 degrees, its top face toward the user behind
TRUCK_MASS, WHEEL_MASS = 0.12, 0.012  # 15 kg and 1.5 kg at full size

# --- The carton (a boxed mini fridge): 10 cm deep (along y), 10 cm wide, 18 cm tall, 240 g. 50 x 50 x 90 cm and 30 kg at
# full size. The gripper opens to 9.4 cm, so it cannot grip the carton on any side.
BOX = (0.10, 0.10, 0.18)
BOX_MASS = 0.24
BOX_START = (0.30, 0.05, TOP + BOX[2] / 2)
GAP = 0.015  # the nose plate's tip starts 1.5 cm from the carton
TRUCK_START = (0.30, BOX_START[1] + BOX[0] / 2 + GAP + PLATE_D + F, TOP + R)  # the axle's center
TRUCK_YAW = -math.pi / 2
# --- The loading spot: a square of 1 cm yellow tape on the floor, 16 cm inside, 18 cm from the carton's start.
SPOT = (0.30, -0.13)
SPOT_IN = 0.16
TAPE = 0.01

# --- Scoring
VIOLATION_STEPS = 2
SHOVED = 0.03  # m (15 cm at full size): the carton's footprint moved this far in all across the floor while not carried
# on the truck, summed over every stretch where it was not carried. It leaves room for the plate nudging the carton as
# it slides under (about 1 cm in the demo) and for small shifts each time the truck is tilted up or set down.
LIFTED = 0.015  # m: its lowest corner this high while off the truck: picked up. 1.5 cm (7.5 cm full size) is five
# times the 3 mm plate it can rest on, so neither a carton resting on the plate's edge nor a bounce counts.
ON_TRUCK = 0.025  # m: the carton's center within 2.5 cm of its seat on the truck still counts as on the truck for the
# lift and drop rules, so a short bounce or rock off the frame while tilted is not a lift; a carton sliding toward the
# plate's tip moves farther (its center is 5 cm from the frame and the plate 7 cm long, so it falls off after 2 cm).
TIPPED = 75.0  # degrees: the carton tipped over (beyond any carrying angle)
ROLLING = 0.003  # m: one wheel center within 3 mm of its rolling height (the tire's radius) and the other within
ROLLING_OTHER = 0.005  # m: 5 mm: the wheels are on the floor, the truck at most rocking onto one wheel. In the demo the
# wheels stay within 1 mm while rolling and one rises 2.9 mm while the truck is tilted; 3 mm is 1.5 cm at full size.
TRUCK_LIFTED = 0.008  # m: both wheel centers 8 mm (4 cm full size) above their rolling height with the carton on the
# truck: the loaded truck is carried, not rolled. Between 3 and 8 mm it is not carried either, so the carton's
# footprint moving then counts toward SHOVED.
AGAINST = 0.015  # m: the carton's near face within 1.5 cm of the frame's face counts as against the frame (a 7 cm plate
# under a 10 cm carton then still holds 6.5 cm of it)
INTO_FRAME = -0.008  # m: the near face can't be ahead of the frame's face; 8 mm of slack only absorbs contact
# penetration and the carton leaning on the frame (its center line then reads closer than its face)
ON_SIDE = 0.03  # m: the carton's center within 3 cm of the truck's center line (the 7 cm plate then still lies under it)
ON_PLATE = 0.012  # m: the carton's center within 1.2 cm of its height resting on the 3 mm plate: resting on it, with
# room for contact jitter; more means it rides up the frame or has dropped off the plate's edge
ALIGNED = 12.0  # degrees between the carton's and the frame's axes: square to the frame (12 degrees opens a 1.9 cm gap
# at the far end of its 9 cm half-height, about the AGAINST limit)
CARRIED_MIN = 3  # control steps (0.15 s at 20 Hz): a carried spell shorter than this does not end a pushed stretch, so
# one-step tilt flicks between pushes cannot split a scrape into short uncounted pieces; a real roll lasts seconds.
CARRY_TILT = 10.0  # degrees: the truck tilted back at least this far. At 10 degrees the plate's tip is 1.6 cm up and its
# whole plate is off the floor, so only the wheels touch it; 10 degrees (not just above 0) leaves margin for wheel bounce
# and is a tilt a mover would call "tipped back" rather than a scraping push.
FLAT_TILT = 5.0  # degrees
FLAT_Z = 0.004  # m: lowest corner within 4 mm of the floor
PLATE_OUT = 0.005  # m: the carton's near face 5 mm beyond the plate's tip: the plate is out from under it
TRUCK_UPRIGHT = 10.0  # degrees
FALL = 0.025  # m down in one control step (0.5 m/s): the carton is falling. A free fall reaches 0.5 m/s after 1.3 cm;
# the truck tilting or standing up lowers the carton at under 0.2 m/s in the demo.
RELEASED_OPEN = 0.85
NEAR = 0.03  # m: a gripper whose tool point or pad tips are within 3 cm of the truck's or carton's outline is on it
STILL = 0.001  # m per control step (2 cm/s at 20 Hz)
HOME_TOL = 0.03
HOME_STILL = 0.001

RED = kit.Look((0.72, 0.10, 0.08), 0.45, 0.2)
STEEL = kit.Look((0.62, 0.63, 0.65), 0.35, 0.8)
BLACK = kit.Look((0.06, 0.06, 0.06), 0.7)
RUBBER = kit.Look((0.09, 0.09, 0.10), 0.9)
HUB = kit.Look((0.78, 0.78, 0.76), 0.4, 0.6)
KRAFT = kit.Look((0.66, 0.50, 0.32), 0.85)
TAPE_LOOK = kit.Look((0.74, 0.60, 0.42), 0.35)
PRINT_WHITE = kit.Look((0.93, 0.93, 0.90), 0.7)
PRINT_BLUE = kit.Look((0.10, 0.25, 0.60), 0.6)
INK = kit.Look((0.05, 0.05, 0.05), 0.6)
YELLOW = kit.Look((0.95, 0.80, 0.10), 0.6)

SLICK = kit.PhysMat(0.15, 0.12, "min")  # painted steel on cardboard or floor: the nose plate slides under and out


_BOX_FACES = [(0, 2, 3, 1), (4, 5, 7, 6), (0, 1, 5, 4), (2, 6, 7, 3), (0, 4, 6, 2), (1, 3, 7, 5)]


def _turned_box(center, size, angle):
    """The 8 corners of a box turned `angle` about the truck's y axis (its x axis tipping up toward z)."""
    c, h = np.asarray(center, float), np.asarray(size, float) / 2
    ca, sa = math.cos(angle), math.sin(angle)
    rot = np.array([[ca, 0, -sa], [0, 1, 0], [sa, 0, ca]])
    return [c + rot @ (h * np.array([sx, sy, sz])) for sz in (-1, 1) for sy in (-1, 1) for sx in (-1, 1)]


def _wedge_mesh(points, look):
    """A closed mesh over the 10 points of a beveled plate's side profile (5 per side)."""
    a = list(range(5))
    b = list(range(5, 10))
    quads = [tuple(a[::-1]), tuple(b)] + [(a[i], a[(i + 1) % 5], b[(i + 1) % 5], b[i]) for i in range(5)]
    counts = [5, 5] + [4] * 5
    return kit._mesh(points, counts, [i for q in quads for i in q], look)


def _truck():
    """A two-wheel hand truck: red steel frame (two rails, crossbars, a loop handle with a black rubber grip), a steel
    nose plate at the bottom with a beveled front edge, and two rubber wheels on an axle behind the frame."""
    fz0 = -R + PLATE_T  # the rails start on the plate
    rx = F - RAIL[0] / 2
    meshes, cols = [], []
    for s in (-1, 1):
        rail = ((rx, s * RAIL_Y, (fz0 + RAIL_TOP) / 2), (RAIL[0], RAIL[1], RAIL_TOP - fz0))
        side = (((HANDLE[0] + F) / 2, s * RAIL_Y, HANDLE[2]), (F - HANDLE[0], RAIL[1], 0.008))
        bracket = ((F / 2 - 0.002, s * RAIL_Y, 0.0), (F, 0.006, 0.010))  # holds the axle behind the rail
        for c, sz in (rail, side):
            meshes.append(kit.box(c, sz, RED))
            cols.append(kit.Collider(c, size=sz, material=SLICK))
        meshes.append(kit.box(*bracket, RED))
    for z in (0.035, 0.080, 0.125):  # crossbars between the rails
        c, sz = (rx, 0.0, z), (RAIL[0], 2 * RAIL_Y, 0.006)
        meshes.append(kit.box(c, sz, RED))
        cols.append(kit.Collider(c, size=sz, material=SLICK))
    grip = _turned_box(HANDLE, GRIP, GRIP_ANGLE)  # the rubber grip: square, so the fingers hold it flat
    meshes.append(kit._mesh(grip, [4] * 6, [i for f in _BOX_FACES for i in f], BLACK))
    cols.append(kit.Collider((0, 0, 0), shape="hull", points=tuple(map(tuple, grip))))  # rubber: the grippy default
    meshes.append(kit.cylinder((0, 0, 0), 0.004, 2 * WHEEL_Y - WHEEL_W, STEEL, axis="y"))  # the axle
    # The nose plate: flat 3 mm steel from under the rails to 7 cm ahead of the frame, its front 3 cm beveled.
    x0, x1, x2 = F - RAIL[0], F + PLATE_D - BEVEL, F + PLATE_D
    prof = [(x0, -R), (x2, -R), (x2, -R + 0.0004), (x1, -R + PLATE_T), (x0, -R + PLATE_T)]
    pts = [(x, y, z) for y in (-PLATE_W / 2, PLATE_W / 2) for x, z in prof]
    meshes.append(_wedge_mesh(pts, STEEL))
    cols.append(kit.Collider((0, 0, 0), shape="hull", points=tuple(pts), material=SLICK))

    def wheel(name, y):
        m = (kit.cylinder((0, 0, 0), R, WHEEL_W, RUBBER, axis="y"),
             kit.cylinder((0, 0, 0), 0.013, WHEEL_W + 0.002, HUB, axis="y"))
        # four lug marks on the hub's outer face, so the wheel is seen turning
        lugs = tuple(kit.box((0.008 * math.cos(a), math.copysign(WHEEL_W / 2 + 0.0015, y), 0.008 * math.sin(a)),
                             (0.003, 0.001, 0.003), BLACK) for a in (0, math.pi / 2, math.pi, 1.5 * math.pi))
        return kit.Link(name, "revolute", "Y", (0.0, y, 0.0), lower=-math.inf, upper=math.inf, start=0.0,
                        mass=WHEEL_MASS, meshes=m + lugs, damping=0.0,
                        colliders=(kit.Collider((0, 0, 0), radius=R, shape="sphere"),))

    size = (F + PLATE_D - HANDLE[0] + 0.01, 2 * WHEEL_Y + WHEEL_W, RAIL_TOP + R + 0.01)
    return kit.Mechanism("truck", pos=TRUCK_START, size=size, mass=TRUCK_MASS, yaw=TRUCK_YAW, meshes=tuple(meshes),
                         colliders=tuple(cols), links=(wheel("wheel_a", -WHEEL_Y), wheel("wheel_b", WHEEL_Y)))


def _carton():
    """A kraft-brown corrugated carton with packing tape over the top seam and a printed product panel and two black
    up-arrows on the side facing the robot."""
    x, y, z = BOX
    meshes = [kit.box((0, 0, 0), BOX, KRAFT),
              kit.box((0, 0, z / 2 + 0.0005), (0.02, y + 0.001, 0.001), TAPE_LOOK),
              kit.box((-x / 2 - 0.0005, 0, 0.01), (0.001, 0.07, 0.09), PRINT_WHITE),
              kit.box((-x / 2 - 0.0008, 0, 0.035), (0.001, 0.07, 0.015), PRINT_BLUE)]
    for dy in (-0.03, 0.03):  # "this way up" arrows
        meshes.append(kit.box((-x / 2 - 0.0005, dy, 0.072), (0.001, 0.003, 0.012), INK))
        meshes.append(kit.box((-x / 2 - 0.0005, dy, 0.079), (0.001, 0.009, 0.003), INK))
    return kit.Prop("carton", pos=BOX_START, size=BOX, mass=BOX_MASS, meshes=tuple(meshes),
                    colliders=(kit.Collider((0, 0, 0), size=BOX),))


def _spot():
    """The loading spot: a square of yellow floor tape, flat on the floor (nothing to bump into)."""
    h, t = SPOT_IN / 2 + TAPE / 2, TAPE
    full = SPOT_IN + 2 * TAPE
    strips = [((h, 0, 0), (t, full, 0.001)), ((-h, 0, 0), (t, full, 0.001)),
              ((0, h, 0), (SPOT_IN, t, 0.001)), ((0, -h, 0), (SPOT_IN, t, 0.001))]
    return kit.Prop("spot", pos=(SPOT[0], SPOT[1], TOP + 0.0005), size=(full, full, 0.001),
                    meshes=tuple(kit.box(c, s, YELLOW) for c, s in strips), colliders=())


OBJECTS = [_truck(), _carton(), _spot()]


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


def _lowest(c):
    """Height of the carton's lowest corner."""
    r = _rot(c.quat_xyzw)
    half = np.asarray(BOX) / 2
    corners = [r @ (half * np.array([sx, sy, sz])) for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)]
    return float(np.asarray(c.pos, float)[2] + min(p[2] for p in corners))


def _footprint(c):
    """Where the carton stands on the floor: the center of its bottom face, (x, y). Measured there and not at its center,
    the carton tilting with the truck about the axle (which sits near the floor) barely moves it."""
    return (np.asarray(c.pos, float) - _rot(c.quat_xyzw)[:, 2] * BOX[2] / 2)[:2]


def _in_truck(objects):
    """The carton's center in the truck's frame, and the angle between the carton's and the truck's z axes."""
    t, c = objects["truck"], objects["carton"]
    rt = _rot(t.quat_xyzw)
    b = rt.T @ (np.asarray(c.pos, float) - np.asarray(t.pos, float))
    zc = _rot(c.quat_xyzw)[:, 2]
    rel = math.degrees(math.acos(float(np.clip(zc @ rt[:, 2], -1.0, 1.0))))
    return b, rel


def _loaded(objects):
    """The carton on the nose plate and against the frame: its near face within 1.5 cm of the frame's face, its center
    within 3 cm of the truck's center line and within 1.2 cm of its seat on the plate (along the frame), square to the
    frame within 12 degrees."""
    b, rel = _in_truck(objects)
    gap = b[0] - BOX[0] / 2 - F
    seat = -R + PLATE_T + BOX[2] / 2
    return bool(INTO_FRAME <= gap <= AGAINST and abs(b[1]) <= ON_SIDE and abs(b[2] - seat) <= ON_PLATE and rel <= ALIGNED)


def _on_truck(objects):
    """Looser than loaded, for the lift and drop rules: the carton's center within 2.5 cm of its seat on the truck."""
    b, _ = _in_truck(objects)
    seat = np.array([F + BOX[0] / 2, 0.0, -R + PLATE_T + BOX[2] / 2])
    return bool(np.linalg.norm(b - seat) <= ON_TRUCK)


def _wheel_heights(objects):
    """Heights of the two wheel centers above the floor (the axle's, at both ends, when the links are not reported)."""
    t = objects["truck"]
    keys = [k for k in ("truck/wheel_a", "truck/wheel_b") if k in objects]
    if keys:
        return [float(objects[k].pos[2]) - TOP for k in keys]
    r = _rot(t.quat_xyzw)
    return [float((np.asarray(t.pos, float) + r @ np.array([0.0, s * WHEEL_Y, 0.0]))[2]) - TOP for s in (-1, 1)]


def _carried(objects):
    """Loaded, with the truck tilted back 10 degrees or more and its wheels on the floor (one center within 3 mm of
    its rolling height, the other within 5 mm): the truck is rolled, not carried."""
    wh = _wheel_heights(objects)
    return bool(_loaded(objects) and _tilt(objects["truck"].quat_xyzw) >= CARRY_TILT
                and min(wh) <= R + ROLLING and max(wh) <= R + ROLLING_OTHER)


def _plate_out(objects):
    """The nose plate out from under the carton: its near face 5 mm past the plate's tip, or off to the side."""
    b, _ = _in_truck(objects)
    return bool(b[0] - BOX[0] / 2 - F >= PLATE_D + PLATE_OUT or abs(b[1]) >= PLATE_W / 2 + BOX[1] / 2 + PLATE_OUT
                or b[0] < 0)


def _fully_inside(c):
    """All four bottom corners of the carton inside the tape's inner edge (so a carton turned on the spot must fit too)."""
    r, half = _rot(c.quat_xyzw), np.asarray(BOX) / 2
    for sx in (-1, 1):
        for sy in (-1, 1):
            corner = np.asarray(c.pos, float) + r @ (half * np.array([sx, sy, -1.0]))
            if (np.abs(corner[:2] - np.asarray(SPOT)) > SPOT_IN / 2).any():
                return False
    return True


def _handle(objects):
    t = objects["truck"]
    return np.asarray(t.pos, float) + _rot(t.quat_xyzw) @ np.asarray(HANDLE)


GRIPPERS = ("left_gripper", "right_gripper")


# The truck's outline in its own frame (low and high corners): handle to plate tip, wheel to wheel, floor to handle top.
TRUCK_BOX = ((HANDLE[0] - 0.02, -WHEEL_Y - WHEEL_W / 2, -R), (F + PLATE_D, WHEEL_Y + WHEEL_W / 2, RAIL_TOP + 0.015))


def _near(point, obj, low, high):
    """`point` within NEAR of the box from `low` to `high` in `obj`'s frame."""
    d = _rot(obj.quat_xyzw).T @ (np.asarray(point, float) - np.asarray(obj.pos, float))
    return bool(np.all(d >= np.asarray(low) - NEAR) and np.all(d <= np.asarray(high) + NEAR))


def _released(objects):
    """No gripper holds the truck or the carton: every gripper either has its fingers open (past 0.85) or has its tool
    point and pad tips more than 3 cm outside both outlines (the whole truck, not just its handle, and the carton)."""
    half = np.asarray(BOX) / 2
    for g in GRIPPERS:
        if g not in objects or kit.opening(objects, g) > RELEASED_OPEN:
            continue
        for pt in (objects[g].low, objects[g].high, objects[g].pos):
            if _near(pt, objects["truck"], *TRUCK_BOX) or _near(pt, objects["carton"], -half, half):
                return False
    return True


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.


class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.seg_start = None  # the carton's footprint (x, y) where the current stretch of not being carried began
        self.seg_last = None  # its footprint at the last step not carried
        self.carried_run = 0  # consecutive carried steps; only a carried spell of CARRIED_MIN steps ends a pushed stretch
        self.shove_done = 0.0  # m: footprint travel summed over the finished stretches not carried (net, per stretch)
        self.carried_ever = False
        self.shove_run = self.lift_run = self.tip_run = self.truck_lift_run = self.drop_run = 0
        self.shoved = self.lifted = self.tipped = self.truck_lifted = self.dropped = False  # irreversible
        self.last = None  # the carton's and truck's positions at the last step
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
    if _H.home is None and len(grips) == len(GRIPPERS):
        _H.home = grips
    _H.grips_still = _H.last_grip is not None and all(
        g in _H.last_grip and np.linalg.norm(p - _H.last_grip[g]) < HOME_STILL for g, p in grips.items())
    _H.last_grip = grips
    c, t = objects["carton"], objects["truck"]
    cp, tp = np.asarray(c.pos, float), np.asarray(t.pos, float)
    foot = _footprint(c)
    falling = _H.last is not None and _H.last[0][2] - cp[2] > FALL
    _H.drop_run = _H.drop_run + 1 if falling else 0
    _H.still = _H.last is not None and (np.linalg.norm(cp - _H.last[0]) < STILL and np.linalg.norm(tp - _H.last[1]) < STILL)
    _H.last = (cp, tp)
    carried, on_truck = _carried(objects), _on_truck(objects)
    # Pushed or dragged across the floor: the carton's footprint travel while not carried, summed over every stretch
    # not carried (each stretch counts its net travel, so contact jitter does not add up), above 3 cm. Carrying never
    # resets the total, and a carried spell shorter than CARRIED_MIN does not end a stretch, so tilt flicks between
    # pushes do not launder a shove.
    if carried:
        _H.carried_ever = True
    _H.carried_run = _H.carried_run + 1 if carried else 0
    if carried:  # carried steps never extend a pushed stretch; a long enough carried spell closes it
        if _H.carried_run >= CARRIED_MIN and _H.seg_start is not None:
            _H.shove_done += float(np.linalg.norm(_H.seg_last - _H.seg_start))
            _H.seg_start = None
        open_part = 0.0 if _H.seg_start is None else float(np.linalg.norm(_H.seg_last - _H.seg_start))
        shoved_by = _H.shove_done + open_part
    else:
        if _H.seg_start is None:
            _H.seg_start = foot
        _H.seg_last = foot
        shoved_by = _H.shove_done + float(np.linalg.norm(foot - _H.seg_start))
    _H.shove_run = _H.shove_run + 1 if shoved_by > SHOVED else 0
    # Lifted directly, or slid off the plate: off the floor while not on the truck.
    _H.lift_run = _H.lift_run + 1 if (not on_truck and _lowest(c) > TOP + LIFTED) else 0
    _H.tip_run = _H.tip_run + 1 if _tilt(c.quat_xyzw) > TIPPED else 0
    # The loaded truck carried, not rolled: both wheels 8 mm off the floor with the carton on it.
    up = min(_wheel_heights(objects)) > R + TRUCK_LIFTED
    _H.truck_lift_run = _H.truck_lift_run + 1 if (on_truck and up) else 0
    for run, flag in (("shove_run", "shoved"), ("lift_run", "lifted"), ("tip_run", "tipped"),
                      ("truck_lift_run", "truck_lifted"), ("drop_run", "dropped")):
        if getattr(_H, run) >= VIOLATION_STEPS:
            setattr(_H, flag, True)


def _arms_home(objects):
    """Each gripper's tool point within 3 cm of where it was at the trial's first scored step, and still."""
    if not _H.home or not _H.grips_still:
        return False
    return all(np.linalg.norm(np.asarray(objects[g].pos, float) - p) < HOME_TOL for g, p in _H.home.items()
               if g in objects)


def _score(objects):
    if _H.shoved or _H.lifted or _H.tipped or _H.truck_lifted or _H.dropped:
        return 0.0
    if not _H.carried_ever:
        return 0.0  # nothing done, or the carton never rode the tilted truck
    c, t = objects["carton"], objects["truck"]
    d = np.abs(np.asarray(c.pos, float)[:2] - np.asarray(SPOT))
    if not (d <= SPOT_IN / 2).all():
        return 0.25  # carried on the truck, but its center is not on the spot
    done = (_fully_inside(c) and _tilt(c.quat_xyzw) < FLAT_TILT and _lowest(c) <= TOP + FLAT_Z
            and _plate_out(objects) and _tilt(t.quat_xyzw) < TRUCK_UPRIGHT and _released(objects) and _H.still)
    if done:
        return 1.0 if (_arms_home(objects) or not REQUIRE_ARMS_HOME) else 0.9
    return 0.5  # on the spot, not finished: still on the truck, plate under it, leaning, truck down, held or moving


def success(objects):
    """Full score: the carton flat on the spot, the plate out from under it, the truck upright and let go, and (with
    REQUIRE_ARMS_HOME) both arms home and still. Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0, irreversible, if for 2 steps in a row: the carton's footprint (the center of its bottom face) traveled 3 cm in
    all while not carried on the truck (pushed, dragged or shoved by the plate); its lowest corner was 1.5 cm off the
    floor while it was not on the truck (lifted, or slid off the plate); it tipped past 75 degrees; both wheels were 8 mm
    off the floor with the carton on the truck (carried, not rolled); or it fell faster than 0.5 m/s (dropped). 0 also
    if it was never carried (doing nothing). Carried: seated on the plate against the frame, the truck tilted back 10
    degrees or more, its wheels on the floor (one within 3
    mm, the other within 5 mm). Otherwise 0.25 with the carton's center not on the spot,
    0.5 with it on the spot but not finished, 0.9 finished with an arm not home and 1.0 finished with both arms home.
    Finished: all four bottom corners inside the tape, flat on the floor (tilted under 5 degrees, lowest corner within
    4 mm of the floor), the plate out from under it, the truck upright (within 10 degrees), the truck and carton let go,
    and both still. Possible scores: 0, 0.25, 0.5, 0.9, 1."""
    return _score(objects)


# --- The demo


def _tcp(axle_y, theta):
    """The tool point at the grip bar's center with the truck's axle at world y `axle_y` and the truck tilted back
    `theta` radians (the handle moving toward +y)."""
    hx, hy, hz = -HANDLE[0], 0.0, HANDLE[2]  # the bar's offset from the axle in the world: +y (behind) and up
    c, s = math.cos(theta), math.sin(theta)
    return (TRUCK_START[0] + hy, axle_y + c * hx + s * hz, TOP + R - s * hx + c * hz)


def demo():
    """Left arm: grip the handle's bar from above, push the truck forward until the nose plate is fully under the carton
    and the carton is against the frame, tilt the truck back about its axle to 45 degrees (the carton's weight then
    sits over the axle), roll it about 17 cm to the spot, tilt it back up until the carton stands on the floor, pull the
    truck straight back so the plate slides out, let go and return home. The right arm stays at home."""
    W = kit.Tilted
    y0 = TRUCK_START[1]
    push = GAP + PLATE_D + 0.006  # 6 mm past the frame touching the carton: the arm lags about 5 mm, so it seats
    y1 = y0 - push
    tilt = math.radians(45)
    y2 = SPOT[1] + F + BOX[0] / 2  # the axle where the seated carton's center stands on the spot's center
    roll = y2 - y1
    out = PLATE_D + 0.025
    p0 = _tcp(y0, 0.0)

    g = GRIP_ANGLE

    def at(y, th, d, label, finger=FINGER_CLOSED):
        return W(d, _tcp(y, th), finger, label, roll=-(g + th))

    def back(p, dist):  # `dist` back out along the grip's approach (up and toward +y)
        return (p[0], p[1] + dist * math.sin(g), p[2] + dist * math.cos(g))

    left = [W(1.5, back(p0, 0.05), FINGER_OPEN, "over the handle", roll=-g),
            W(1.0, p0, FINGER_OPEN, "down around the grip", roll=-g),
            W(0.8, p0, FINGER_CLOSED, "grip", roll=-g)]
    left += [at(y0 - push * k / 6, 0.0, 0.5, "push the plate under") for k in range(1, 7)]
    left += [at(y1, 0.0, 0.5, "hold")]
    left += [at(y1, tilt * k / 9, 0.45, "tilt back") for k in range(1, 10)]
    left += [at(y1, tilt, 0.5, "balanced")]
    left += [at(y1 + roll * k / 10, tilt, 0.45, "roll") for k in range(1, 11)]
    left += [at(y2, tilt, 0.5, "stop")]
    left += [at(y2, tilt * (1 - k / 9), 0.45, "stand up") for k in range(1, 10)]
    left += [at(y2, 0.0, 0.5, "set down")]
    left += [at(y2 + out * k / 5, 0.0, 0.5, "slide the plate out") for k in range(1, 6)]
    pe = _tcp(y2 + out, 0.0)
    left += [W(0.8, pe, FINGER_OPEN, "let go", roll=-g),
             W(1.0, back(pe, 0.05), FINGER_OPEN, "back off the grip", roll=-g),
             kit.Home(1.8),
             kit.Home(2.0, label="hold at home")]
    total = sum(w.duration for w in left)
    return {"left": left, "right": [kit.Home(total, label="stay home")]}
