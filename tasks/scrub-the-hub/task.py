"""An IV flush on a training arm. The patient's forearm (skin-colored, about 7 cm thick and 32 cm long, tapering to the
wrist) lies on a dark padded armrest, pointing at the robot. A peripheral IV catheter is taped in the forearm under a
clear film dressing. A short clear extension line runs from it down the arm's right side to a needleless connector (a
clear valve with a blue cap ring) that lies on the bench beside the armrest, taped down, its open end pointing to the
robot's right. On the right, a light-blue tray holds an opened alcohol prep pad, folded, on its torn foil packet, and a
prefilled 10 mL saline flush syringe lying with its tip toward the connector.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props below; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Flush the IV line on the training arm."

# Armrest and forearm (fixed). The forearm collides as a box whose top is the top of the arm.
REST_X, REST_L, REST_W, REST_H = 0.29, 0.36, 0.12, 0.01
ARM_X, ARM_L, ARM_R = 0.29, 0.32, 0.035
ARM_Z = TOP + REST_H + ARM_R
ARM_TOP = ARM_Z + ARM_R
CATH = (0.31, -0.012)  # where the catheter enters the skin

# The needleless connector lies on the bench along y, its open end (the hub) facing -y.
AXIS_Z = TOP + 0.009  # the connector's and the syringe's common axis height
HUB_X, HUB_BACK_Y, HUB_TIP_Y, HUB_R = 0.27, -0.074, -0.100, 0.0055
HUB = np.array([HUB_X, HUB_TIP_Y, AXIS_Z])  # the hub's open end: the surface that must be scrubbed

PAD = (0.035, 0.035, 0.012)  # the folded prep pad
TRAY_X, TRAY_Y, TRAY_H = 0.35, -0.20, 0.008
TRAY_SIZE = (0.15, 0.15, TRAY_H)
PAD_START = (0.315, -0.20, TOP + TRAY_H + 0.002 + PAD[2] / 2)  # on its foil packet
SYR_L, SYR_W = 0.12, 0.018  # the syringe collides as a box along y, its tip end at +y
SYR_START = (0.385, -0.20, TOP + TRAY_H + SYR_W / 2)

HUB_TOP = AXIS_Z + HUB_R  # the connector's top surface
SCRUB_END = 0.010  # the scrubbed part: the last 1 cm of the connector before its open end
CONTACT_BELOW, CONTACT_ABOVE = 0.003, 0.002  # pad underside within 3 mm below / 2 mm above the connector's top
SCRUB_SPEED = 0.010  # and moving at least 1 cm/s: a scrub, not a pad left lying there
HELD_OPENING = 0.6  # a gripper closed at least this far (0 closed, 1 open) ...
HELD_DIST = 0.03  # ... with its tool point within 3 cm of the pad's center holds the pad
PAD_TILT = 20.0  # degrees; a tumbling pad doesn't scrub
SCRUB_STEPS = 300  # 15 s at 20 Hz, counted over the whole run
TOUCH = 0.015  # the syringe's tip end within 1.5 cm of the hub counts as touching it
CONNECTED = 0.008  # within 8 mm at rest, lined up with the connector, counts as connected

# -- looks and models -------------------------------------------------------------------------

SKIN = kit.Look((0.87, 0.68, 0.55), 0.65)
REST_LOOK = kit.Look((0.16, 0.18, 0.24), 0.9)
CLEAR = kit.Look((0.88, 0.92, 0.96), 0.15, opacity=0.45)
FILM = kit.Look((0.95, 0.96, 0.97), 0.2, opacity=0.35)
TUBE = kit.Look((0.90, 0.93, 0.95), 0.2, opacity=0.6)
CATH_PINK = kit.Look((0.95, 0.55, 0.70), 0.4)
BLUE = kit.Look((0.15, 0.40, 0.85), 0.4)
WHITE = kit.Look((0.96, 0.96, 0.95), 0.6)
TAPE = kit.Look((0.92, 0.91, 0.86), 0.9)
PAD_LOOK = kit.Look((0.98, 0.98, 0.97), 0.95)
FOIL = kit.Look((0.82, 0.82, 0.86), 0.25, metallic=0.8)
FOIL_PRINT = kit.Look((0.75, 0.10, 0.25), 0.5)
TRAY_LOOK = kit.Look((0.62, 0.76, 0.90), 0.5)
BARREL = kit.Look((0.94, 0.96, 0.98), 0.15, opacity=0.5)
SALINE = kit.Look((0.86, 0.92, 0.98), 0.1, opacity=0.35)
PLUNGER = kit.Look((0.97, 0.97, 0.97), 0.5)
STOPPER = kit.Look((0.18, 0.18, 0.20), 0.6)
LABEL = kit.Look((0.95, 0.95, 0.85), 0.6)


def _forearm():
    """The training arm with the IV site: catheter hub and wings under a clear film, and the extension line running
    down the arm's right side onto the bench to the connector."""
    profile = [(0.0, -0.165), (0.022, -0.163), (0.027, -0.155), (0.030, -0.10), (0.034, -0.02), (0.036, 0.06),
               (0.037, 0.15), (0.033, 0.16), (0.0, 0.162)]
    cx, cy = CATH[0] - ARM_X, CATH[1]
    top = ARM_R + 0.001
    line = [(cx - 0.012, cy - 0.002, top + 0.004), (cx - 0.025, cy - 0.016, top - 0.004),
            (cx - 0.035, -0.032, -0.010), (cx - 0.040, -0.042, -0.030), (HUB_X - ARM_X, -0.055, AXIS_Z - ARM_Z + 0.002),
            (HUB_X - ARM_X, HUB_BACK_Y + 0.002, AXIS_Z - ARM_Z)]
    meshes = (kit.lathe(profile, SKIN, axis="x", n=48),
              kit.box((cx, cy, top + 0.0005), (0.045, 0.035, 0.001), FILM),
              kit.cylinder((cx - 0.006, cy, top + 0.004), 0.004, 0.014, CATH_PINK, axis="x"),
              kit.box((cx - 0.004, cy, top + 0.0015), (0.008, 0.022, 0.002), CATH_PINK),
              kit.box((cx - 0.030, -0.030, -0.006), (0.012, 0.012, 0.001), TAPE),
              kit.band(line, 0.004, 0.004, TUBE, normal=(0, 0, 1)))
    return kit.Prop("forearm", pos=(ARM_X, 0.0, ARM_Z), size=(ARM_L, 2 * ARM_R, 2 * ARM_R), meshes=meshes,
                    colliders=(kit.Collider((0, 0, 0), size=(ARM_L, 2 * ARM_R, 2 * ARM_R)),))


def _connector():
    """A needleless connector: clear valve body, blue collar, white luer end; taped to the bench."""
    length = HUB_BACK_Y - HUB_TIP_Y
    yc = (HUB_BACK_Y + HUB_TIP_Y) / 2
    body = [(0.0, -length / 2), (0.003, -length / 2), (0.0045, -0.008), (HUB_R, -0.004), (HUB_R, 0.006),
            (0.0045, 0.009), (0.004, length / 2), (0.0, length / 2)]
    tape_z = -(AXIS_Z - TOP) + 0.0005
    return kit.Prop("connector", pos=(HUB_X, yc, AXIS_Z), size=(0.012, length, 0.012),
                    meshes=(kit.lathe(body, CLEAR, axis=(0, -1, 0), n=32),
                            kit.cylinder((0, -0.004, 0), HUB_R + 0.0008, 0.005, BLUE, axis="y"),
                            kit.cylinder((0, -length / 2 + 0.002, 0), 0.0042, 0.004, WHITE, axis="y"),
                            kit.box((0, 0.008, tape_z), (0.022, 0.012, 0.001), TAPE)),
                    colliders=(kit.Collider((0, 0, 0), size=(0.011, length, 0.011)),))


def _pad():
    """An alcohol prep pad folded in four into a soft square."""
    w, h = PAD[0], PAD[2]
    return kit.Prop("pad", pos=PAD_START, size=PAD, mass=0.025,
                    meshes=(kit.box((0, 0, -h / 4), (w, w, h / 2), PAD_LOOK),
                            kit.box((0.001, -0.001, h / 4), (w - 0.003, w - 0.002, h / 2), PAD_LOOK)),
                    colliders=(kit.Collider((0, 0, 0), size=PAD),))


def _syringe():
    """A prefilled saline flush: clear barrel with saline, white plunger and flange, luer tip at +y."""
    r = SYR_W / 2 - 0.001
    half = SYR_L / 2
    return kit.Prop("syringe", pos=SYR_START, size=(SYR_W, SYR_L, SYR_W), mass=0.04,
                    meshes=(kit.cylinder((0, 0.012, 0), r, 0.075, BARREL, axis="y"),
                            kit.cylinder((0, 0.015, 0), r - 0.0015, 0.064, SALINE, axis="y"),
                            kit.cylinder((0, 0.012, 0), r + 0.0003, 0.03, LABEL, axis="y"),
                            kit.lathe([(r, 0), (0.003, 0.006), (0.002, 0.012), (0.0, 0.012)], BARREL,
                                      center=(0, 0.0495, 0), axis="y"),
                            kit.cylinder((0, -0.0165, 0), r - 0.0012, 0.004, STOPPER, axis="y"),
                            kit.box((0, -0.0275, 0), (SYR_W, 0.0025, 0.012), WHITE),
                            kit.box((0, -0.0435, 0), (0.004, 0.030, 0.010), PLUNGER),
                            kit.cylinder((0, -half + 0.002, 0), r, 0.003, PLUNGER, axis="y")),
                    colliders=(kit.Collider((0, 0, 0), size=(SYR_W, SYR_L, SYR_W)),))


def _tray():
    foil = kit.box((PAD_START[0] - TRAY_X, 0, TRAY_H / 2 + 0.001), (0.05, 0.06, 0.002), FOIL)
    foil_print = kit.box((PAD_START[0] - TRAY_X, 0.022, TRAY_H / 2 + 0.0021), (0.04, 0.012, 0.0002), FOIL_PRINT)
    return kit.Prop("tray", pos=(TRAY_X, TRAY_Y, TOP + TRAY_H / 2), size=TRAY_SIZE,
                    meshes=(kit.box((0, 0, 0), TRAY_SIZE, TRAY_LOOK), foil, foil_print),
                    colliders=(kit.Collider((0, 0, 0), size=TRAY_SIZE),
                               kit.Collider((PAD_START[0] - TRAY_X, 0, TRAY_H / 2 + 0.001), size=(0.05, 0.06, 0.002))))


OBJECTS = [
    kit.Prop("armrest", pos=(REST_X, 0.0, TOP + REST_H / 2), size=(REST_L, REST_W, REST_H),
             meshes=(kit.box((0, 0, 0), (REST_L, REST_W, REST_H), REST_LOOK),),
             colliders=(kit.Collider((0, 0, 0), size=(REST_L, REST_W, REST_H)),)),
    _forearm(),
    _connector(),
    _tray(),
    _pad(),
    _syringe(),
]
_START = {o.name: np.array(o.pos) for o in OBJECTS if o.mass is not None}


def _rot(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _syringe_tip(s):
    """The syringe's tip end (the end of its luer, where it meets the connector) and its long axis."""
    r = _rot(np.asarray(s.quat_xyzw, dtype=float))
    axis = r @ np.array([0.0, 1.0, 0.0])
    return np.asarray(s.pos, dtype=float) + axis * SYR_L / 2, axis


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()  # which trial this history belongs to
        self.last = None
        self.scrub = 0  # control steps the pad was scrubbing the hub
        self.touched_unscrubbed = False  # never forgiven


_H = _History()


def _held(objects, name):
    """Some gripper is closed on the object: closed past HELD_OPENING with its tool point near the object's center."""
    pos = np.asarray(objects[name].pos, dtype=float)
    for side in ("left_gripper", "right_gripper"):
        g = objects.get(side)
        if g is None:
            continue
        if kit.opening(objects, side) < HELD_OPENING and np.linalg.norm(np.asarray(g.pos) - pos) < HELD_DIST:
            return True
    return False


def _scrubbing(objects):
    """A scrub step: the pad held in a gripper, lying flat (within 20 degrees), its underside on the connector's top
    (within 3 mm below to 2 mm above it) over the connector's last centimeter before the open end, and moving at
    1 cm/s or more."""
    p = objects["pad"]
    qx, qy = p.quat_xyzw[0], p.quat_xyzw[1]
    flat = 1 - 2 * (qx * qx + qy * qy) > math.cos(math.radians(PAD_TILT))
    half = PAD[0] / 2
    over = (abs(p.pos[0] - HUB_X) < half + HUB_R
            and p.pos[1] - half < HUB_TIP_Y + SCRUB_END and p.pos[1] + half > HUB_TIP_Y)
    bottom = p.pos[2] - PAD[2] / 2
    on_top = HUB_TOP - CONTACT_BELOW < bottom < HUB_TOP + CONTACT_ABOVE
    return bool(flat and over and on_top and np.linalg.norm(p.lin_vel) > SCRUB_SPEED and _held(objects, "pad"))


def _touching(s):
    tip, _ = _syringe_tip(s)
    return bool(np.linalg.norm(tip - HUB) < TOUCH)


def _connected(s):
    tip, axis = _syringe_tip(s)
    return bool(np.linalg.norm(tip - HUB) < CONNECTED and axis[1] > math.cos(math.radians(15)) and s.resting())


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    now = {n: np.asarray(objects[n].pos, dtype=float) for n in _START}
    _H.last = now
    if _scrubbing(objects):
        _H.scrub += 1
    if _touching(objects["syringe"]) and _H.scrub < SCRUB_STEPS:
        _H.touched_unscrubbed = True


def _score(objects):
    if _H.touched_unscrubbed or _H.scrub < SCRUB_STEPS:
        return 0.0
    return 1.0 if _connected(objects["syringe"]) else 0.5


def success(objects):
    """1.0: the hub scrubbed for 15 s before the syringe touched it, and the syringe connected. Called first each
    step, so it also records the step's history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if the syringe's tip came within 1.5 cm of the hub before the hub had been scrubbed for 15 s in all, even
    if it was scrubbed later. 0 also if the hub was never scrubbed for 15 s. Otherwise 0.5, or 1.0 with the syringe
    connected: its tip end at rest within 8 mm of the hub, lined up with the connector within 15 degrees.
    A scrub step is a control step in which a gripper holds the pad flat, rubbing its underside on the connector's
    top over the last centimeter before the open end, moving at 1 cm/s or more (see _scrubbing). Possible scores: 0,
    0.5, 1."""
    return _score(objects)


def demo():
    """Right arm: pick up the prep pad, scrub the hub's open end with small back-and-forth strokes for 17 s, drop the
    pad on the tray, then pick up the syringe, lay it in line with the connector and push its tip home."""

    def grip(top, below=0.012):  # TCP height that puts the finger pads `below` an object's top
        return top - below + 0.013

    travel = TOP + 0.20
    lag = 0.012  # the arm falls ~1.5 cm short of a target near the middle of the bench, toward its own base
    o, c = FINGER_OPEN, FINGER_CLOSED
    pad_top = PAD_START[2] + PAD[2] / 2
    held_below = 0.006  # the pads grip the pad 6 mm below its top
    pick_pad = pick_place_waypoints((PAD_START[0], PAD_START[1], grip(pad_top, held_below)),
                                    (0.0, 0.0, 0.0), travel_z=travel)[:4]
    # Scrub: the pad's underside rubs over the top and the open end of the connector.
    pad_z = AXIS_Z + HUB_R + PAD[2] / 2 - 0.002
    tcp_z = grip(pad_z + PAD[2] / 2, held_below)
    hx, hy = HUB_X, HUB[1] + 0.006 + lag
    scrub = [Waypoint(1.8, (hx, hy, tcp_z + 0.05), c, "over the hub"),
             Waypoint(1.0, (hx, hy, tcp_z), c, "pad on the hub")]
    for k in range(17):
        scrub += [Waypoint(0.5, (hx + 0.010, hy - 0.004, tcp_z), c, "scrub"),
                  Waypoint(0.5, (hx - 0.010, hy + 0.004, tcp_z), c, "scrub")]
    drop = [Waypoint(1.0, (hx, hy, travel), c, "lift the pad"),
            Waypoint(1.5, (0.24, -0.24, travel - 0.08), c, "to the discard spot"),
            Waypoint(0.8, (0.24, -0.24, TOP + 0.05), c, "lower"),
            Waypoint(0.5, (0.24, -0.24, TOP + 0.05), o, "drop the pad"),
            Waypoint(0.8, (0.24, -0.24, travel), o, "up")]
    yaw = np.pi / 2  # fingers close along x, across the syringe
    syr_top = SYR_START[2] + SYR_W / 2
    sy_home = HUB[1] - SYR_L / 2  # syringe center when its tip meets the hub
    held = grip(TOP + SYR_W, 0.008)
    syringe = pick_place_waypoints((SYR_START[0], SYR_START[1], grip(syr_top, 0.008)),
                                   (HUB_X, sy_home - 0.02 + lag, held + 0.003), travel_z=travel, yaw=yaw)[:6]
    syringe += [Waypoint(1.5, (HUB_X, sy_home + lag + 0.006, held + 0.003), c, "push the tip home", yaw),
                Waypoint(0.8, (HUB_X, sy_home + lag + 0.006, held + 0.003), c, "seat", yaw),
                Waypoint(0.6, (HUB_X, sy_home + lag + 0.006, held + 0.003), o, "let go", yaw),
                Waypoint(1.0, (HUB_X, sy_home + lag, travel), o, "up", yaw)]
    park = (0.16, -0.26, TOP + 0.22)
    right = pick_pad + scrub + drop + syringe + [Waypoint(1.2, park, o, "park"), Waypoint(2.0, park, o, "hold")]
    return {"right": right}
