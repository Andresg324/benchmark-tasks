"""A pH meter bench after a measurement. A glass pH electrode hangs in the ring of an electrode stand, its bulb in a
small beaker of sample. On the robot's right are a beaker of deionized (DI) water with a squeeze wash bottle behind
it, and the electrode's storage bottle: a wide-mouth bottle of 3 M KCl storage solution with a green label and a
white cap that has a hole with a black rubber grommet for the electrode. The electrode is put away by lowering it
through that hole until its head rests on the cap.
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
INSTRUCTION = "Put the pH probe away."

# The electrode. Its origin is the middle of its head; the glass shaft hangs below the head.
HEAD_R, HEAD_H = 0.014, 0.030
SHAFT_R, SHAFT_L = 0.006, 0.080
TIP_BELOW = HEAD_H / 2 + SHAFT_L  # from the origin down to the bulb's tip
JUNCTION_ABOVE_TIP = 0.015  # the ceramic junction, which must be wet too

HOLE = 0.022  # the square opening in the stand's ring and in the storage cap (the head is 2.8 cm across)
RING_TOP = 0.090  # height of the ring's top above the table
STAND_RING = (0.27, -0.08)
SAMPLE = dict(xy=STAND_RING, r=0.025, h=0.068, surface=0.045)  # 100 mL beaker
RINSE = dict(xy=(0.22, -0.20), r=0.029, h=0.080, surface=0.055)  # 150 mL beaker of DI water
STORE = dict(xy=(0.30, -0.22), r=0.028, h=0.074, surface=0.060)  # storage bottle body; its cap top is at RING_TOP
GLASS_T = 0.0025  # beaker wall
FLOOR = 0.004  # beaker and bottle floor thickness

PROBE_START = (STAND_RING[0], STAND_RING[1], TOP + RING_TOP + HEAD_H / 2)
SUBMERGED = 0.020  # the bulb and the junction count as wet with the tip at least 2 cm under the surface
RINSE_STEPS = 10  # 0.5 s at 20 Hz in the DI water counts as a rinse
DROP_TILT = math.radians(45)
DROP_STEPS = 2

# -- looks ------------------------------------------------------------------------------------

GLASS = kit.Look((0.86, 0.92, 0.95), 0.05, opacity=0.25)
PROBE_GLASS = kit.Look((0.80, 0.88, 0.92), 0.05, opacity=0.45)
FILL = kit.Look((0.93, 0.93, 0.88), 0.4, opacity=0.6)  # the electrode's KCl filling, seen through its glass
SILVER = kit.Look((0.78, 0.78, 0.80), 0.25, metallic=1.0)
EPOXY = kit.Look((0.08, 0.08, 0.09), 0.5)
CERAMIC = kit.Look((0.95, 0.94, 0.90), 0.8)
DI_WATER = kit.Look((0.80, 0.90, 1.00), 0.05, opacity=0.30)
SAMPLE_LIQ = kit.Look((0.95, 0.80, 0.35), 0.05, opacity=0.45)
KCL = kit.Look((0.85, 0.93, 0.95), 0.05, opacity=0.35)
HDPE = kit.Look((0.94, 0.94, 0.92), 0.6, opacity=0.85)
CAP_WHITE = kit.Look((0.97, 0.97, 0.97), 0.5)
RUBBER = kit.Look((0.06, 0.06, 0.06), 0.9)
LABEL_GREEN = kit.Look((0.15, 0.62, 0.30), 0.6)
LABEL_WHITE = kit.Look((0.98, 0.98, 0.96), 0.7)
STAND_METAL = kit.Look((0.70, 0.71, 0.73), 0.3, metallic=0.8)
STAND_BASE = kit.Look((0.20, 0.22, 0.26), 0.6)
WASH_BOTTLE = kit.Look((0.95, 0.95, 0.93), 0.6, opacity=0.9)
WASH_CAP = kit.Look((0.15, 0.35, 0.80), 0.5)
MARK = kit.Look((0.25, 0.25, 0.25), 0.6)


def annulus(center, r_in, r_out, height, look, n=40):
    """A flat ring (a short tube with a hole) about the vertical through `center`."""
    cx, cy, cz = center
    pts, counts, idx = [], [], []
    for z in (cz - height / 2, cz + height / 2):
        for r in (r_in, r_out):
            for k in range(n):
                t = 2 * math.pi * k / n
                pts.append((cx + r * math.cos(t), cy + r * math.sin(t), z))
    ring = lambda layer, k: layer * n + k % n  # noqa: E731  (layers: 0 bottom-in, 1 bottom-out, 2 top-in, 3 top-out)
    for k in range(n):
        for a, b in ((0, 1), (2, 3), (0, 2), (1, 3)):
            counts.append(4)
            idx += [ring(a, k), ring(a, k + 1), ring(b, k + 1), ring(b, k)]
    return kit._mesh(pts, counts, idx, look)


def glass_vessel(r, h, floor, surface, glass, liquid, wall=0.0025, n=48):
    """An open glass vessel holding liquid, drawn so the renderer shows both: the glass is drawn only above the liquid
    line (an open band, so no cap closes the mouth), the liquid's own surface forms the vessel's sides below that line,
    and a thin glass floor sits under it. See-through layers drawn over each other hide one another."""
    ring = [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n), (surface + h) / 2) for k in range(n + 1)]
    return [kit.band(ring, h - surface, wall, glass, normal=(0, 0, 1)),
            kit.cylinder((0, 0, floor / 2), r, floor, glass, n=n),
            kit.cylinder((0, 0, (floor + surface) / 2), r, surface - floor, liquid, n=n)]


def octagon_walls(r_in, height, z0, wall):
    """Eight box colliders standing round a circle of radius r_in (their inner faces touch it)."""
    # Colliders can't be turned, so four walls face the main directions and four square posts fill the diagonals,
    # each post's inner corner on the circle.
    zc, side, a = z0 + height / 2, 2 * r_in * math.tan(math.pi / 8) + 0.004, wall / 2
    out = [kit.Collider((sx * (r_in + a), sy * (r_in + a), zc), size=(wall, side, height) if sx else (side, wall, height))
           for sx, sy in ((1, 0), (-1, 0), (0, 1), (0, -1))]
    d = (r_in + a * math.sqrt(2)) / math.sqrt(2)
    out += [kit.Collider((sx * d, sy * d, zc), size=(wall, wall, height)) for sx in (1, -1) for sy in (1, -1)]
    return out


def square_hole(center_z, hole, outer, thick):
    """Four box colliders round a square opening `hole` wide, in a plate `outer` wide."""
    s = (outer - hole) / 2
    o = hole / 2 + s / 2
    return [kit.Collider((o, 0, center_z), size=(s, outer, thick)), kit.Collider((-o, 0, center_z), size=(s, outer, thick)),
            kit.Collider((0, o, center_z), size=(hole, s, thick)), kit.Collider((0, -o, center_z), size=(hole, s, thick))]


def _beaker(name, d, liquid, marks=True):
    r, h, s = d["r"], d["h"], d["surface"]
    ri = r - GLASS_T
    meshes = glass_vessel(r - GLASS_T / 2, h, FLOOR, s, GLASS, liquid, wall=GLASS_T)
    if marks:  # white graduation ticks on the side facing the arms
        for k, z in enumerate(np.arange(0.02, h - 0.01, 0.01)):
            w = 0.010 if k % 2 == 0 else 0.006
            meshes.append(kit.box((-r - 0.0003, 0, z), (0.0006, w, 0.0012), LABEL_WHITE))
    colliders = [kit.Collider((0, 0, FLOOR / 2), size=(2 * ri, 2 * ri, FLOOR))] + octagon_walls(ri, h - FLOOR, FLOOR, 0.004)
    return kit.Prop(name, pos=(d["xy"][0], d["xy"][1], TOP), size=(2 * r, 2 * r, h),
                    meshes=tuple(meshes), colliders=tuple(colliders))


def _storage_bottle():
    """A wide-mouth bottle of storage solution: body, shoulder, neck and a white cap with a grommeted hole."""
    r, h, s = STORE["r"], STORE["h"], STORE["surface"]
    ri = r - 0.002
    neck_r = 0.020
    profile = [(0.0, 0.0), (r - 0.002, 0.0), (r, 0.003), (r, h - 0.010), (neck_r + 0.002, h - 0.002),
               (neck_r, h - 0.002), (neck_r, h), ]
    cap_h = RING_TOP - h
    meshes = [kit.lathe(profile, HDPE, n=48),
              kit.cylinder((0, 0, (FLOOR + s) / 2), ri - 0.0006, s - FLOOR, KCL, n=48),
              kit.cylinder((0, 0, 0.032), r + 0.0006, 0.034, LABEL_WHITE, n=48),
              kit.cylinder((0, 0, 0.040), r + 0.0009, 0.012, LABEL_GREEN, n=48),
              annulus((0, 0, h + cap_h / 2), 0.0118, neck_r + 0.003, cap_h, CAP_WHITE),
              annulus((0, 0, RING_TOP + 0.0006), HOLE / 2, HOLE / 2 + 0.0025, 0.0012, RUBBER)]
    for k in range(10):  # ribs on the cap
        t = 2 * math.pi * k / 10
        meshes.append(kit.box(((neck_r + 0.003) * math.cos(t), (neck_r + 0.003) * math.sin(t), h + cap_h / 2),
                              (0.002, 0.002, cap_h * 0.9), CAP_WHITE))
    colliders = ([kit.Collider((0, 0, FLOOR / 2), size=(2 * ri, 2 * ri, FLOOR))]
                 + octagon_walls(ri, h - 0.012 - FLOOR, FLOOR, 0.003)
                 + square_hole(h - 0.006, 0.026, 2 * r, 0.012)  # the shoulder, round the neck
                 + square_hole(RING_TOP - cap_h / 2, HOLE, 2 * neck_r + 0.006, cap_h))
    return kit.Prop("storage_bottle", pos=(STORE["xy"][0], STORE["xy"][1], TOP), size=(2 * r, 2 * r, RING_TOP),
                    meshes=tuple(meshes), colliders=tuple(colliders))


def _stand():
    """An electrode stand: a heavy base behind the sample, a rod, and an arm ending in a ring over the beaker."""
    rx = STAND_RING[0] + 0.11
    ry = STAND_RING[1]
    base = (0.10, 0.12, 0.012)
    rod_h = 0.18
    arm_z = RING_TOP - 0.005
    arm_len = rx - STAND_RING[0] - 0.016
    meshes = (kit.box((rx - STAND_RING[0] + 0.01, 0, base[2] / 2), base, STAND_BASE),
              kit.cylinder((rx - STAND_RING[0], 0, rod_h / 2), 0.006, rod_h, STAND_METAL),
              kit.box((rx - STAND_RING[0] - 0.006, 0, arm_z), (0.03, 0.022, 0.016), STAND_BASE),  # the clamp boss
              kit.box((arm_len / 2 + 0.016, 0, arm_z), (arm_len, 0.012, 0.010), STAND_METAL),
              annulus((0, 0, arm_z), 0.0115, 0.0175, 0.010, STAND_BASE),
              kit.cylinder((rx - STAND_RING[0], 0, rod_h + 0.003), 0.007, 0.006, STAND_BASE))
    colliders = ([kit.Collider((rx - STAND_RING[0] + 0.01, 0, base[2] / 2), size=base),
                  kit.Collider((rx - STAND_RING[0], 0, rod_h / 2), size=(0.012, 0.012, rod_h)),
                  kit.Collider((arm_len / 2 + 0.016, 0, arm_z), size=(arm_len, 0.012, 0.010))]
                 + square_hole(arm_z, HOLE, 0.035, 0.010))
    return kit.Prop("stand", pos=(STAND_RING[0], ry, TOP), size=(0.15, 0.12, rod_h + 0.006),
                    meshes=meshes, colliders=tuple(colliders))


def _wash_bottle():
    x, y = 0.38, -0.29
    profile = [(0.0, 0.0), (0.028, 0.0), (0.030, 0.004), (0.030, 0.115), (0.016, 0.135), (0.016, 0.140)]
    meshes = (kit.lathe(profile, WASH_BOTTLE, n=40),
              kit.cylinder((0, 0, 0.145), 0.018, 0.012, WASH_CAP),
              kit.band([(0, 0, 0.150), (0, 0, 0.17), (-0.02, 0, 0.183), (-0.045, 0, 0.182)], 0.004, 0.004, WASH_BOTTLE,
                       normal=(0, 1, 0)),
              kit.cylinder((0, 0, 0.07), 0.0305, 0.03, LABEL_WHITE),
              kit.cylinder((0, 0, 0.078), 0.0308, 0.008, WASH_CAP))
    return kit.Prop("wash_bottle", pos=(x, y, TOP), size=(0.06, 0.06, 0.15), meshes=meshes,
                    colliders=(kit.Collider((0, 0, 0.07), size=(0.06, 0.06, 0.14)),))


def _probe():
    """A combination pH electrode with a screw connector head (its cable is unplugged for storage): black head,
    clear glass shaft with the white filling and the silver reference wire inside, a ceramic junction near the tip
    and the round glass bulb."""
    top_shaft = -HEAD_H / 2
    tip = -TIP_BELOW
    meshes = (kit.cylinder((0, 0, 0), HEAD_R, HEAD_H, EPOXY),
              kit.cylinder((0, 0, HEAD_H / 2 + 0.006), 0.008, 0.012, SILVER),  # the screw connector
              kit.cylinder((0, 0, HEAD_H / 2 + 0.013), 0.004, 0.002, EPOXY),
              kit.cylinder((0, 0, top_shaft - 0.004), 0.0075, 0.008, EPOXY),  # the collar under the head
              kit.cylinder((0, 0, (top_shaft + tip) / 2 + 0.003), SHAFT_R, SHAFT_L - 0.010, PROBE_GLASS),
              kit.cylinder((0, 0, (top_shaft + tip) / 2 + 0.006), SHAFT_R - 0.0012, SHAFT_L - 0.030, FILL),
              kit.cylinder((0, 0, (top_shaft + tip) / 2), 0.0007, SHAFT_L - 0.02, SILVER),
              kit.lathe([(0.0, tip), (0.0035, tip + 0.0008), (0.0052, tip + 0.0030), (SHAFT_R, tip + 0.0065),
                         (SHAFT_R, tip + 0.008)], PROBE_GLASS),
              kit.box((-SHAFT_R - 0.0002, 0, tip + JUNCTION_ABOVE_TIP), (0.0014, 0.003, 0.003), CERAMIC))
    colliders = (kit.Collider((0, 0, 0), radius=HEAD_R, height=HEAD_H),
                 kit.Collider((0, 0, top_shaft - SHAFT_L / 2), size=(2 * SHAFT_R, 2 * SHAFT_R, SHAFT_L)))
    return kit.Prop("probe", pos=PROBE_START, size=(2 * HEAD_R, 2 * HEAD_R, HEAD_H + SHAFT_L), mass=0.04,
                    meshes=meshes, colliders=colliders)


OBJECTS = [_stand(), _beaker("sample_beaker", SAMPLE, SAMPLE_LIQ), _beaker("rinse_beaker", RINSE, DI_WATER),
           _storage_bottle(), _wash_bottle(), _probe()]
_START = {o.name: np.array(o.pos) for o in OBJECTS if o.mass is not None}


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
GRIPPERS = ("left_gripper", "right_gripper")
class _History:
    def __init__(self):
        self.episode = kit.episode()  # which trial this history belongs to
        self.last = None
        self.in_rinse = 0
        self.rinsed = False
        self.tipped = 0
        self.dropped = False  # never forgiven


_H = _History()


def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _tilt(s):
    x, y = s.quat_xyzw[0], s.quat_xyzw[1]
    return math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0)))


def _tip(s):
    return np.asarray(s.pos, float) + _rot(s.quat_xyzw) @ np.array([0.0, 0.0, -TIP_BELOW])


def _wet_in(s, d):
    """The bulb and the junction under the surface of vessel `d`: the tip inside the vessel's wall and at least 2 cm
    below its liquid surface, with the electrode within 15 degrees of upright."""
    t = _tip(s)
    inside = np.hypot(t[0] - d["xy"][0], t[1] - d["xy"][1]) < d["r"] - GLASS_T
    return bool(inside and t[2] < TOP + d["surface"] - SUBMERGED and _tilt(s) < math.radians(15))


def _stored(s):
    """Put away: wet in the storage bottle, hanging by its head in the cap (the head within 1 cm of resting on the cap),
    and still."""
    seated = s.pos[2] < TOP + RING_TOP + HEAD_H / 2 + 0.010
    return bool(_wet_in(s, STORE) and seated and s.resting(max_speed=0.02))


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    tracked = list(_START) + [g for g in GRIPPERS if g in objects]
    now = {n: np.asarray(objects[n].pos, dtype=float) for n in tracked}
    _H.last = now
    p = objects["probe"]
    _H.in_rinse = _H.in_rinse + 1 if _wet_in(p, RINSE) else 0
    if _H.in_rinse >= RINSE_STEPS:
        _H.rinsed = True
    _H.tipped = _H.tipped + 1 if _tilt(p) > DROP_TILT else 0
    if _H.tipped >= DROP_STEPS:
        _H.dropped = True


def _score(objects):
    if _H.dropped or not _stored(objects["probe"]):
        return 0.0
    return 1.0 if _H.rinsed else 0.5


def success(objects):
    """1.0: the electrode rinsed in the DI water, then hanging in the storage bottle with its bulb wet. Called first
    each step, so it also records the step's history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if the electrode ever tipped more than 45 degrees for 2 control steps in a row (a dropped glass electrode). Otherwise, at the end: 1.0 if it
    hangs in the storage bottle with its bulb and junction in the solution and it was dipped in the DI water first
    (at least 0.5 s with the same depth); 0.5 if it is stored but was never rinsed; 0 if it is anywhere else, including
    left in the sample, left in the DI water or lying dry. Possible scores: 0, 0.5, 1."""
    return _score(objects)


def demo():
    """Right arm: lift the electrode out of the stand's ring, dip it in the DI water for 2 s, lift it out, and lower it
    through the storage cap until its head rests on the cap."""
    travel = TOP + 0.212
    grip_dz = 0.005  # TCP above the head's middle when gripping
    lag = 0.0  # measured: at rest the arm reaches these targets within 2 mm, so no offset
    swing = 0.004  # the electrode hangs about 2 degrees back in the grip, so its tip trails 4 mm in -x
    sx, sy = STAND_RING
    rx, ry = RINSE["xy"]
    bx, by = STORE["xy"]
    pick_z = PROBE_START[2] + grip_dz
    dip_z = TOP + 0.022 + TIP_BELOW + grip_dz  # tip 2.2 cm above the table: 3 cm under the DI water
    seat_z = TOP + RING_TOP + HEAD_H / 2 + 0.003 + grip_dz
    w = []
    w.append(Waypoint(1.5, (sx, sy + lag, travel), FINGER_OPEN, "over the stand"))
    w.append(Waypoint(1.5, (sx, sy + lag, pick_z), FINGER_OPEN, "down to the head"))
    w.append(Waypoint(0.8, (sx, sy + lag, pick_z), FINGER_CLOSED, "grip"))
    w.append(Waypoint(1.5, (sx, sy + lag, travel), FINGER_CLOSED, "lift out of the ring"))
    w.append(Waypoint(1.5, (rx, ry + lag, travel), FINGER_CLOSED, "over the DI water"))
    w.append(Waypoint(1.2, (rx, ry + lag, dip_z), FINGER_CLOSED, "dip"))
    w.append(Waypoint(2.0, (rx, ry + lag, dip_z), FINGER_CLOSED, "rinse"))
    w.append(Waypoint(1.2, (rx, ry + lag, travel), FINGER_CLOSED, "lift out"))
    w.append(Waypoint(1.5, (bx + swing, by + lag, travel), FINGER_CLOSED, "over the storage bottle"))
    w.append(Waypoint(1.0, (bx + swing, by + lag, travel), FINGER_CLOSED, "settle"))
    w.append(Waypoint(2.5, (bx + swing, by + lag, seat_z), FINGER_CLOSED, "lower through the cap"))
    w.append(Waypoint(0.6, (bx + swing, by + lag, seat_z), FINGER_OPEN, "let go"))
    w.append(Waypoint(1.0, (bx + swing, by + lag, travel), FINGER_OPEN, "up"))
    w.append(Waypoint(1.0, (0.16, -0.26, TOP + 0.20), FINGER_OPEN, "park"))
    w.append(Waypoint(2.0, (0.16, -0.26, TOP + 0.20), FINGER_OPEN, "hold"))
    return {"right": w}
