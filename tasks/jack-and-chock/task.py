"""A scale model of a small hatchback (about 1:10) stands on the bench, side-on to the robot, its nose to the robot's
left. Its left side, the driver's side in a left-hand-drive car, faces the arms. The front has round headlights, a
grille and a number plate; the back has red tail lights. Under each sill, behind the front wheel and ahead of the
rear wheel, is a jacking point: a bright reinforced patch on the pinch-weld flange with a small triangle on the sill
above it. A red trolley jack stands in front of the car's nose, and two yellow wheel chocks with black handles stand
behind its tail.

The job is to lift the front left wheel. A trained technician chocks the wheel diagonally opposite (the rear right,
on the far side) front and back, and sets the jack's saddle under the front left jacking point, never under the
floor pan or the suspension.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props below; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Get the car ready to lift its front left wheel."

# The car: nose toward +y, so its left side is the near side (smaller x).
CAR_X, CAR_LEN, CAR_W = 0.295, 0.36, 0.16
NEAR, FAR = CAR_X - CAR_W / 2, CAR_X + CAR_W / 2  # 0.215 and 0.375
SILL_Z = 0.025  # underside of the body above the bench
WHEEL_Y, WHEEL_R, WHEEL_W = 0.115, 0.029, 0.019
ARCH_R = 0.034
WHEEL_X = {"left": NEAR + WHEEL_W / 2 + 0.0005, "right": FAR - WHEEL_W / 2 - 0.0005}
WHEELS = {  # name: (x, y) of the tire's center
    "front_left": (WHEEL_X["left"], WHEEL_Y), "rear_left": (WHEEL_X["left"], -WHEEL_Y),
    "front_right": (WHEEL_X["right"], WHEEL_Y), "rear_right": (WHEEL_X["right"], -WHEEL_Y),
}
JP_IN, JP_Y = 0.007, WHEEL_Y - ARCH_R - 0.020  # jacking points: 7 mm in from the sill's face, 2 cm from the arch
JACK_POINTS = {
    "front_left": (NEAR + JP_IN, JP_Y), "rear_left": (NEAR + JP_IN, -JP_Y),
    "front_right": (FAR - JP_IN, JP_Y), "rear_right": (FAR - JP_IN, -JP_Y),
}

# Jack: its origin is the saddle's center; the body trails 5 cm behind it (toward -x when aimed at the car).
JACK_SIZE = (0.070, 0.036, 0.034)
JACK_Z = TOP + 0.010  # origin height: chassis bottom 1 cm below
JACK_START = (0.265, 0.245, JACK_Z)
PUMP_DX, PUMP_R, PUMP_TOP = -0.043, 0.0115, 0.024  # the hydraulic unit the jack is carried by (local frame)
SADDLE_TOP = 0.010  # local; 2 cm above the bench, 5 mm under the sill

# Chocks: origin at the wedge's center. "A" has its toe toward -y, "B" toward +y. The handle post is 4.2 cm out
# along +x from the wedge's center, so the gripper can hold it clear of the car's side.
CHOCK_H, CHOCK_D, CHOCK_L = 0.015, 0.025, 0.026
POST_DX, POST_R, POST_TOP = 0.042, 0.009, 0.025  # post top: local z (wedge mid-height is 0)
CHOCK_Z = TOP + CHOCK_H / 2
CHOCK_START = {"chock_a": (0.33, -0.27, CHOCK_Z), "chock_b": (0.22, -0.27, CHOCK_Z)}
SNUG_D = 0.0294  # center-to-tire distance (along y) at which a chock's colliders touch the tire's
SNUG_SLACK = 0.010  # a chock counts as snug up to 1 cm short of touching
CHOCK_X_TOL = 0.015
JACK_TOL_X, JACK_TOL_Y = 0.005, 0.015  # saddle center vs. the jacking point: across the sill, along it
MAX_TILT = math.radians(15)

# -- looks -----------------------------------------------------------------------------------

PAINT = kit.Look((0.13, 0.33, 0.62), 0.3, metallic=0.4)
GLASS = kit.Look((0.10, 0.12, 0.15), 0.1, metallic=0.2)
TRIM = kit.Look((0.12, 0.12, 0.13), 0.6)
BUMPER = kit.Look((0.55, 0.57, 0.60), 0.5)
TIRE = kit.Look((0.06, 0.06, 0.06), 0.85)
RIM = kit.Look((0.78, 0.79, 0.82), 0.25, metallic=1.0)
LAMP = kit.Look((0.97, 0.97, 0.90), 0.1)
TAIL = kit.Look((0.80, 0.05, 0.05), 0.2)
PLATE = kit.Look((0.95, 0.95, 0.92), 0.5)
UNDER = kit.Look((0.20, 0.20, 0.21), 0.8)
STEEL = kit.Look((0.86, 0.87, 0.90), 0.2, metallic=1.0)
MARK = kit.Look((0.95, 0.95, 0.95), 0.4)
JACK_RED = kit.Look((0.78, 0.08, 0.06), 0.45)
CHROME = kit.Look((0.85, 0.86, 0.88), 0.15, metallic=1.0)
RUBBER_Y = kit.Look((0.96, 0.75, 0.08), 0.7)
RUBBER_K = kit.Look((0.08, 0.08, 0.08), 0.8)


def _poly_x(poly_yz, x0, x1, look):
    """A convex (y, z) outline extruded along x from x0 to x1."""
    n = len(poly_yz)
    pts = [(x0, y, z) for y, z in poly_yz] + [(x1, y, z) for y, z in poly_yz]
    counts, idx = [n, n], list(range(n))[::-1] + list(range(n, 2 * n))
    for i in range(n):
        j = (i + 1) % n
        counts.append(4)
        idx += [i, j, n + j, n + i]
    return kit._mesh(pts, counts, idx, look)


def _strip_x(ys, bottoms, tops, x0, x1, look):
    """A solid between bottom(y) and top(y), sampled at ys, extruded along x from x0 to x1 (quads only, so concave
    outlines such as wheel arches render cleanly)."""
    pts = []
    for y, b, t in zip(ys, bottoms, tops):
        pts += [(x0, y, b), (x1, y, b), (x1, y, t), (x0, y, t)]
    counts, idx = [], []
    for i in range(len(ys) - 1):
        a, c = 4 * i, 4 * (i + 1)
        for e0, e1 in ((0, 1), (1, 2), (2, 3), (3, 0)):
            counts.append(4)
            idx += [a + e0, a + e1, c + e1, c + e0]
    last = 4 * (len(ys) - 1)
    counts += [4, 4]
    idx += [3, 2, 1, 0, last, last + 1, last + 2, last + 3]
    return kit._mesh(pts, counts, idx, look)


def _interp(y, table):
    ys, zs = zip(*table)
    return float(np.interp(y, ys, zs))


def _car():
    """The car is one fixed prop centered at (CAR_X, 0) on the bench. Local frame = world frame shifted by CAR_X."""
    hx = CAR_W / 2
    ys = np.linspace(-CAR_LEN / 2, CAR_LEN / 2, 121)
    belt = [(-0.180, 0.050), (-0.178, 0.068), (-0.172, 0.080), (0.100, 0.080), (0.150, 0.071), (0.172, 0.062),
            (0.180, 0.048)]

    def bottom(y):
        b = SILL_Z
        for yc in (WHEEL_Y, -WHEEL_Y):
            if abs(y - yc) < ARCH_R:
                b = max(b, WHEEL_R + math.sqrt(ARCH_R ** 2 - (y - yc) ** 2))
        if abs(y) > CAR_LEN / 2 - 0.006:
            b = max(b, 0.030)
        return b

    lower = _strip_x(ys, [bottom(y) for y in ys], [_interp(y, belt) for y in ys], -hx, hx, PAINT)
    gy = np.linspace(-0.170, 0.100, 61)
    green = [(-0.170, 0.080), (-0.150, 0.138), (-0.120, 0.146), (0.030, 0.146), (0.100, 0.080)]
    inset = 0.011
    glass = _strip_x(gy, [0.080] * len(gy), [_interp(y, green) - 0.004 for y in gy], -hx + inset, hx - inset, GLASS)
    ry = np.linspace(-0.150, 0.040, 30)
    roof = _strip_x(ry, [_interp(y, green) - 0.005 for y in ry], [_interp(y, green) + 0.001 for y in ry],
                    -hx + inset - 0.001, hx - inset + 0.001, PAINT)
    meshes = [lower, glass, roof]
    # Pillars between the side windows (B pillar) and an underbody tray.
    for sx in (-1, 1):
        meshes.append(kit.box((sx * (hx - inset + 0.0005), -0.035, 0.112), (0.002, 0.012, 0.064), PAINT))
    meshes.append(kit.box((0, 0, SILL_Z + 0.002), (CAR_W - 0.02, CAR_LEN - 0.03, 0.004), UNDER))
    # Front: round headlights, grille, plate, bumper. Back: tail lights, plate, bumper.
    for sx in (-1, 1):
        meshes.append(kit.cylinder((sx * 0.052, 0.1755, 0.060), 0.0095, 0.006, LAMP, axis="y"))
        meshes.append(kit.cylinder((sx * 0.052, 0.1735, 0.060), 0.0115, 0.004, TRIM, axis="y"))
        meshes.append(kit.box((sx * 0.060, -0.1795, 0.070), (0.016, 0.004, 0.016), TAIL))
    meshes.append(kit.box((0, 0.1805, 0.043), (0.060, 0.004, 0.010), TRIM))
    meshes.append(kit.box((0, 0.1830, 0.032), (0.036, 0.002, 0.009), PLATE))
    meshes.append(kit.box((0, -0.1805, 0.058), (0.036, 0.002, 0.009), PLATE))
    meshes.append(kit.box((0, 0.180, 0.033), (CAR_W - 0.004, 0.006, 0.014), BUMPER))
    meshes.append(kit.box((0, -0.180, 0.037), (CAR_W - 0.004, 0.006, 0.014), BUMPER))
    # Mirrors at the windshield base.
    for sx in (-1, 1):
        meshes.append(kit.box((sx * (hx + 0.006), 0.085, 0.090), (0.010, 0.006, 0.008), PAINT))
    # Wheels.
    for (wx, wy) in WHEELS.values():
        lx = wx - CAR_X
        meshes.append(kit.cylinder((lx, wy, WHEEL_R), WHEEL_R, WHEEL_W, TIRE, axis="x", n=40))
        out = 1 if lx > 0 else -1
        meshes.append(kit.cylinder((lx + out * 0.0005, wy, WHEEL_R), 0.0185, WHEEL_W, RIM, axis="x", n=32))
        meshes.append(kit.cylinder((lx + out * 0.0012, wy, WHEEL_R), 0.006, WHEEL_W, TRIM, axis="x", n=20))
    # Pinch-weld flanges under both sills, with the four jacking points (bright patch + triangle above).
    for sx in (-1, 1):
        fx = sx * (hx - JP_IN)
        meshes.append(kit.box((fx, 0, SILL_Z - 0.0025), (0.002, 2 * (WHEEL_Y - ARCH_R) - 0.004, 0.005), UNDER))
        for sy in (-1, 1):
            meshes.append(kit.box((fx, sy * JP_Y, SILL_Z - 0.0028), (0.0035, 0.014, 0.0058), STEEL))
            tri = [(sy * JP_Y - 0.004, 0.036), (sy * JP_Y + 0.004, 0.036), (sy * JP_Y, 0.029)]
            meshes.append(_poly_x(tri, sx * hx - 0.0003 * sx, sx * hx + 0.0008 * sx, MARK))

    colliders = [kit.Collider((0, 0, 0.5 * (SILL_Z + 0.146)), size=(CAR_W, CAR_LEN, 0.146 - SILL_Z))]
    # Tires: stacked slabs that follow the round tread, finest near the ground where a chock meets it.
    bands = [(0.000, 0.002), (0.002, 0.004), (0.004, 0.006), (0.006, 0.008), (0.008, 0.010), (0.010, 0.012),
             (0.012, 0.014), (0.014, 0.016), (0.016, 0.042), (0.042, 0.050), (0.050, 0.058)]
    for (wx, wy) in WHEELS.values():
        for z0, z1 in bands:
            zc = z1 if z1 <= WHEEL_R else (z0 if z0 >= WHEEL_R else WHEEL_R)  # the band's widest height
            hw = math.sqrt(max(WHEEL_R ** 2 - (WHEEL_R - zc) ** 2, 0.0))
            colliders.append(kit.Collider((wx - CAR_X, wy, 0.5 * (z0 + z1)), size=(WHEEL_W, 2 * hw, z1 - z0)))
    return kit.Prop("car", pos=(CAR_X, 0.0, TOP), size=(CAR_W + 0.02, CAR_LEN + 0.01, 0.15), meshes=tuple(meshes),
                    colliders=tuple(colliders))


def _jack():
    m = [
        kit.box((-0.025, 0.0125, -0.006), (0.060, 0.004, 0.008), JACK_RED),
        kit.box((-0.025, -0.0125, -0.006), (0.060, 0.004, 0.008), JACK_RED),
        kit.box((-0.040, 0.0, -0.008), (0.026, 0.025, 0.003), JACK_RED),
    ]
    for wx, r in ((0.002, 0.0045), (-0.051, 0.004)):
        for sy in (-1, 1):
            m.append(kit.cylinder((wx, sy * 0.0165, -0.010 + r), r, 0.003, TIRE, axis="y", n=16))
    arm = np.array([0.030, 0.0, 0.010])
    arm = arm / np.linalg.norm(arm)
    m.append(kit.cylinder((-0.015, 0.0, 0.000), 0.0045, 0.033, JACK_RED, axis=arm))
    m.append(kit.lathe([(0.0035, -0.002), (0.0035, 0.005), (0.0090, 0.0065), (0.0090, 0.0095), (0.0075, 0.010),
                        (0.0, 0.0098)], CHROME, center=(0.0, 0.0, 0.0)))
    m.append(kit.lathe([(PUMP_R, -0.006), (PUMP_R, 0.018), (0.0095, 0.020), (0.0, 0.020)], JACK_RED,
                       center=(PUMP_DX, 0.0, 0.0)))
    m.append(kit.cylinder((PUMP_DX, 0.0, 0.022), 0.006, 0.004, CHROME))
    sock = np.array([-0.7, 0.0, 0.7])
    m.append(kit.cylinder((PUMP_DX - 0.010, 0.0, 0.016), 0.0035, 0.016, CHROME, axis=sock))
    col = (
        kit.Collider((-0.025, 0.0, -0.006), size=(0.062, 0.033, 0.008)),
        kit.Collider((0.0, 0.0, 0.004), size=(0.018, 0.018, 0.012)),
        kit.Collider((PUMP_DX, 0.0, 0.0075), radius=PUMP_R, height=0.027),
    )
    return kit.Prop("jack", pos=JACK_START, size=JACK_SIZE, mass=0.08, meshes=tuple(m), colliders=col)


def _chock(name, toe):
    """A rubber wheel chock: a wedge whose toe (sign `toe` along y) slides under the tire, and a black handle post
    standing on the bench just outside it, which is what the gripper holds."""
    s, h, hl = CHOCK_D / 2, CHOCK_H / 2, CHOCK_L / 2
    prof = [(-s, -h), (-s + 0.004, -h + 0.003), (-s + 0.008, -h + 0.008), (-s + 0.012, h), (s, h), (s, -h)]
    prof = [(-toe * y, z) for y, z in prof]
    meshes = [_poly_x(prof, -hl, hl, RUBBER_Y)]
    meshes.append(kit.cylinder((POST_DX, 0.0, (POST_TOP - h) / 2), POST_R, POST_TOP + h, RUBBER_K))
    meshes.append(kit.box(((hl + POST_DX) / 2, 0.0, 0.0), (POST_DX - hl, 0.008, 0.008), RUBBER_K))
    cols = []
    for k in range(6):  # stepped boxes under the sloped face
        z0, z1 = -h + k * 0.0025, -h + (k + 1) * 0.0025
        front = -s + 0.002 * k  # where the slope is at this step's bottom, toe-side y (toe toward -y)
        cols.append(kit.Collider((0.0, -toe * (front + s) / 2, (z0 + z1) / 2), size=(CHOCK_L, s - front, z1 - z0)))
    cols.append(kit.Collider((POST_DX, 0.0, (POST_TOP - h) / 2), radius=POST_R, height=POST_TOP + h))
    cols.append(kit.Collider(((hl + POST_DX) / 2, 0.0, 0.0), size=(POST_DX - hl, 0.008, 0.008)))
    return kit.Prop(name, pos=CHOCK_START[name], size=(CHOCK_L + POST_DX + POST_R, CHOCK_D, CHOCK_H + POST_TOP),
                    mass=0.04, meshes=tuple(meshes), colliders=tuple(cols))


OBJECTS = [_car(), _jack(), _chock("chock_a", toe=-1), _chock("chock_b", toe=+1)]
_TOE = {"chock_a": -1, "chock_b": +1}


# -- scoring -----------------------------------------------------------------------------------

def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _upright(s):
    qx, qy = s.quat_xyzw[0], s.quat_xyzw[1]
    return bool(1 - 2 * (qx * qx + qy * qy) > math.cos(MAX_TILT))


def _saddle(s):
    return np.asarray(s.pos, float)[:2]


def _jack_set(s):
    """Where the jack is: a jacking point's name when it stands upright and still on the bench with its saddle under
    that point's pinch-weld patch (within 5 mm across the sill and 1.5 cm along it); "under" when its saddle is under
    the car's footprint in any other way, upright or not, on or near the bench; otherwise None."""
    p = _saddle(s)
    if _upright(s) and s.resting() and s.pos[2] < JACK_Z + 0.01:
        for name, (jx, jy) in JACK_POINTS.items():
            if abs(p[0] - jx) < JACK_TOL_X and abs(p[1] - jy) < JACK_TOL_Y:
                return name
    if NEAR - 0.003 < p[0] < FAR + 0.003 and abs(p[1]) < CAR_LEN / 2 and s.pos[2] < JACK_Z + 0.02:
        return "under"
    return None


def _chock_at(name, s):
    """The wheel and side ("front" or "rear") a chock is snug against, or None. Snug: upright and still on the bench,
    its wedge across the tire's tread (center within 1.5 cm of the tire's center line), its toe facing the tire,
    and its center no more than 1 cm farther from the tire than touching."""
    if not (_upright(s) and s.resting() and s.pos[2] < CHOCK_Z + 0.008):
        return None
    toe_dir = _rot(s.quat_xyzw) @ np.array([0.0, float(_TOE[name]), 0.0])
    for wheel, (wx, wy) in WHEELS.items():
        if abs(s.pos[0] - wx) > CHOCK_X_TOL:
            continue
        dy = s.pos[1] - wy
        if abs(dy) > SNUG_D + SNUG_SLACK or abs(dy) < SNUG_D - 0.006:
            continue
        if toe_dir[1] * -np.sign(dy) < 0.7:  # the toe must point at the tire
            continue
        return wheel, ("front" if dy > 0 else "rear")
    return None


def _parts(objects):
    jack = _jack_set(objects["jack"])
    spots = [_chock_at(n, objects[n]) for n in ("chock_a", "chock_b")]
    sides = {side for spot in spots if spot and spot[0] == "rear_right" for side in [spot[1]]}
    on_lifted = any(spot and spot[0] == "front_left" for spot in spots)
    return jack, len(sides), on_lifted


def _score(objects):
    jack, sides, on_lifted = _parts(objects)
    if on_lifted or jack not in (None, "front_left"):
        return 0.0
    j = jack == "front_left"
    if j and sides == 2:
        return 1.0
    if (j and sides == 1) or (not j and sides == 2):
        return 0.5
    if j or sides == 1:
        return 0.25
    return 0.0


def success(objects):
    """Jack saddle under the front left jacking point and the rear right tire chocked front and back."""
    return _score(objects) == 1.0


def score(objects):
    """Scored at the end. 0 if the jack is under the car (upright or tipped) other than set at the front left jacking
    point, or a chock is against the front left tire. Otherwise: 1.0 for the jack at the front left jacking point with the rear right tire
    chocked on both sides; 0.5 for the jack right and one side chocked, or both sides chocked without the jack;
    0.25 for the jack right alone or one side chocked alone; 0 otherwise. Chocks at any other wheel count for
    nothing. Possible scores: 0, 0.25, 0.5, 1.0."""
    return _score(objects)


# -- demo --------------------------------------------------------------------------------------

def _grip(top, below=0.010):  # TCP height that puts the finger pads `below` an object's top
    return top - below + 0.013


def demo():
    """Left arm: carry the jack by its hydraulic unit from in front of the car to beside the front left sill, set it
    down 2 cm out and roll it in until the saddle is under the jacking point. Right arm: carry each chock by its
    handle to just outside the rear right tire, set it down and push it in against the tread, front then rear."""
    travel = TOP + 0.22
    c = np.array(DEMO_TUNE["lag"])

    jp = np.array(JACK_POINTS["front_left"])
    pump0 = np.array(JACK_START[:2]) + (PUMP_DX, 0.0)
    pump1 = jp + (PUMP_DX, 0.0) + DEMO_TUNE["jack_fix"]
    z_pump = _grip(JACK_Z + PUMP_TOP, 0.012)
    out = DEMO_TUNE["jack_out"]
    left = [
        Waypoint(1.5, (*pump0, travel), FINGER_OPEN, "to the jack"),
        Waypoint(1.5, (*pump0, z_pump), FINGER_OPEN, "descend"),
        Waypoint(0.8, (*pump0, z_pump), FINGER_CLOSED, "grip"),
        Waypoint(1.2, (*pump0, travel), FINGER_CLOSED, "lift"),
        Waypoint(2.0, (pump1[0] - out, pump1[1], travel), FINGER_CLOSED, "carry"),
        Waypoint(1.5, (pump1[0] - out, pump1[1], z_pump + 0.002), FINGER_CLOSED, "set down beside the sill"),
        Waypoint(2.0, (pump1[0], pump1[1], z_pump + 0.002), FINGER_CLOSED, "roll it in"),
        Waypoint(0.6, (pump1[0], pump1[1], z_pump + 0.002), FINGER_OPEN, "release"),
        Waypoint(1.0, (pump1[0] - 0.01, pump1[1], travel), FINGER_OPEN, "retreat"),
    ]
    left = [Waypoint(w.duration, tuple(np.array(w.pos) + (*c, 0.0)) if i > 3 else w.pos, w.finger, w.label)
            for i, w in enumerate(left)]

    right = []
    z_post = _grip(CHOCK_Z + POST_TOP, 0.012)
    push = DEMO_TUNE["chock_out"]
    for name, sign in (("chock_a", +1), ("chock_b", -1)):
        s0 = np.array(CHOCK_START[name][:2]) + (POST_DX, 0.0)
        wx, wy = WHEELS["rear_right"]
        goal = np.array([wx + DEMO_TUNE["chock_dx"] + POST_DX, wy + sign * (SNUG_D + DEMO_TUNE["chock_gap"][name])])
        goal = goal + DEMO_TUNE["chock_fix"][name]
        right += [
            Waypoint(1.5, (*s0, travel), FINGER_OPEN, "to the chock"),
            Waypoint(1.5, (*s0, z_post), FINGER_OPEN, "descend"),
            Waypoint(0.8, (*s0, z_post), FINGER_CLOSED, "grip"),
            Waypoint(1.2, (*s0, travel), FINGER_CLOSED, "lift"),
            Waypoint(2.0, (goal[0] + push, goal[1], travel), FINGER_CLOSED, "carry"),
            Waypoint(1.5, (goal[0] + push, goal[1], z_post + 0.002), FINGER_CLOSED, "set down outside"),
            Waypoint(1.5, (goal[0], goal[1], z_post + 0.002), FINGER_CLOSED, "push in"),
            Waypoint(0.6, (goal[0], goal[1], z_post + 0.002), FINGER_OPEN, "release"),
            Waypoint(1.0, (goal[0] + 0.01, goal[1], travel), FINGER_OPEN, "retreat"),
        ]
    park_l, park_r = (0.16, 0.26, TOP + 0.22), (0.16, -0.26, TOP + 0.22)
    left += [Waypoint(1.2, park_l, FINGER_OPEN, "park"), Waypoint(10.0, park_l, FINGER_OPEN, "hold")]
    right += [Waypoint(1.2, park_r, FINGER_OPEN, "park"), Waypoint(2.0, park_r, FINGER_OPEN, "hold")]
    return {"left": left, "right": right}


DEMO_TUNE = {
    "lag": (0.0, 0.0),
    "jack_fix": np.array([0.0, 0.0]),
    "jack_out": 0.020,
    "chock_out": 0.020,
    "chock_dx": 0.0055,
    "chock_gap": {"chock_a": 0.007, "chock_b": 0.003},
    "chock_fix": {"chock_a": np.array([0.0, 0.0]), "chock_b": np.array([0.0, 0.0])},
}
