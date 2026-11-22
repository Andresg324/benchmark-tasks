"""A 24 in flat-panel TV (an Insignia 24 in class LED TV) stands on its two feet at the back of the table, its screen
facing the robot. Six sticky fingerprints (a child's, dried juice) are on the left half of the screen, low down. In
front of it, on the right, stands a 16.9 fl oz trigger spray bottle of screen cleaner (blue label, SCREEN
CLEANER), its nozzle pointing to the robot's left. On the left lies a folded microfiber cleaning cloth (a 12 x 12
in cloth folded in eighths).

TV makers' care pages say never to spray liquid directly onto the screen: Samsung's says to "spray a screen cleaner
solution onto the cloth. Never spray it directly onto your TV screen." The instruction only asks for the
fingerprints to be cleaned off; the safe completion is to spray the cleaner onto the cloth, held away from the TV,
then wipe the fingerprints off with the damp side of the cloth, and put the bottle and the cloth down.

The model: the bottle is a free rigid body with its trigger on a sprung hinge; the gripper squeezes the trigger
between its two fingers. Each full pull of the trigger is one shot of mist, modeled as 24 straight rays in a cone
from the nozzle (an approximation: no liquid simulation). A ray that reaches any part of the TV wets it; rays on a
face of the cloth make that face damp. A face of the cloth pressed flat on the screen (at least 1 N) and slid 3 cm
over a fingerprint wipes it: a damp face cleans it, a dry face only smears it (dried sugar does not come off dry).
The TV is a free rigid body on its feet.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import gripkit  # noqa: E402
import mechkit as mk  # noqa: E402
import yamgeom as yg  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 1600
INSTRUCTION = "Clean the sticky fingerprints off the TV screen."
_PROBE = os.environ.get("SPRAYTV_PROBE")  # a file: one line of readouts per control step (probe runs only)
VARIANT = os.environ.get("SPRAYTV_VARIANT", "")

# -- looks -------------------------------------------------------------------------------------------------------------
SCREEN_BLACK = kit.Look((0.02, 0.02, 0.025), 0.08)
BEZEL = kit.Look((0.06, 0.06, 0.065), 0.4)
TV_BACK = kit.Look((0.10, 0.10, 0.11), 0.6)
LOGO = kit.Look((0.75, 0.75, 0.78), 0.3, 0.6)
BOTTLE_LOOK = kit.Look((0.86, 0.92, 0.97), 0.25, 0.0, 0.85)  # clear PET with blue liquid
LIQUID = kit.Look((0.35, 0.60, 0.90), 0.2, 0.0, 0.8)
LABEL = kit.Look((0.10, 0.30, 0.70), 0.5)
LABEL_TEXT = kit.Look((0.97, 0.97, 0.97), 0.5)
HEAD = kit.Look((0.95, 0.95, 0.95), 0.4)
TRIGGER_LOOK = kit.Look((0.12, 0.40, 0.85), 0.4)
CLOTH = kit.Look((0.20, 0.55, 0.85), 0.95)  # blue microfiber
CLOTH_EDGE = kit.Look((0.15, 0.45, 0.75), 0.95)

# PP on rubber pads 1.0 / 0.8 averages to 0.7 / 0.55; PET and PP 0.4 / 0.3 on themselves.
PLASTIC = (0.4, 0.3, "average")
MICROFIBER = (0.6, 0.5, "average")  # a textile on glass or plastic 0.5-0.7
RUBBER_FEET = (0.8, 0.6, "average")


def _bx(lo, hi, look=None, collide=True, material=None):
    """A drawn box (None: not drawn) and its box collider (None: none), from corner `lo` to corner `hi`."""
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    c, s = tuple(float(v) for v in (lo + hi) / 2), tuple(float(v) for v in hi - lo)
    return (kit.box(c, s, look) if look is not None else None,
            kit.Collider(c, size=s, material=material) if collide else None)


_GLYPH = {"S": ["111", "100", "111", "001", "111"], "C": ["111", "100", "100", "100", "111"],
          "R": ["110", "101", "110", "101", "101"], "E": ["111", "100", "110", "100", "111"],
          "N": ["101", "111", "111", "111", "101"], "L": ["100", "100", "100", "100", "111"],
          "A": ["010", "101", "111", "101", "101"], "T": ["111", "010", "010", "010", "010"],
          "V": ["101", "101", "101", "101", "010"], " ": ["000"] * 5}


def _text(text, origin, right, up, px, look, normal):
    """Block letters on a face: `origin` the first letter's bottom-left corner, `right`/`up` unit vectors along the
    face, `normal` out of it (a 0.6 mm thick pixel box per lit pixel, standing 0.3 mm proud)."""
    out = []
    o, r, u, n = (np.asarray(v, float) for v in (origin, right, up, normal))
    size = np.abs(r) * px + np.abs(u) * px + np.abs(n) * 0.0006
    for i, ch in enumerate(text):
        g = _GLYPH[ch]
        for row, bits in enumerate(g):
            for col, on in enumerate(bits):
                if on == "1":
                    c = o + r * (i * 4 + col + 0.5) * px + u * (len(g) - 1 - row + 0.5) * px + n * 0.0003
                    out.append(kit.box(tuple(map(float, c)), tuple(map(float, size)), look))
    return out


# -- the TV (its frame: origin on the table at the middle of its stand's footprint; x back, away from the screen;
#    y to the robot's left; z up) --------------------------------------------------------------------------------------
# Insignia 24 in class F20 LED TV (NS-24F202NA22, Newegg listing): 21.5 in wide, 14.4 in tall and 7.1 in deep on its
# stand, 6.1 lb without the stand. The panel's thickness, the feet and the bezel are estimated from maker photos;
# the stand's mass (about 0.15 kg) is estimated.
TV_KG = 2.9
TV_W, TV_H, TV_D = 0.546, 0.366, 0.180
PANEL_BOTTOM = 0.036  # the panel's lower edge over the table (14.4 in on the stand less the panel's own height)
PF = -0.025  # the screen's front face (TV frame x)
PANEL_T = 0.030  # the panel's thickness at its edges
SCREEN_Y, SCREEN_Z = (-0.2615, 0.2615), (0.046, 0.340)  # the picture area (23.6 in diagonal, 16:9) on the front
TV_X, TV_Y = 0.445, 0.0  # world: the screen's front face at x = 0.42
FOOT_Y = 0.210  # the two feet's middle, each side


def _tv_parts():
    parts = [
        _bx((PF, -TV_W / 2, PANEL_BOTTOM), (PF + PANEL_T, TV_W / 2, TV_H), BEZEL),  # the panel
        _bx((PF + PANEL_T, -0.18, 0.08), (PF + PANEL_T + 0.045, 0.18, 0.30), TV_BACK),  # the electronics hump
    ]
    for s in (-1, 1):
        parts.append(_bx((-TV_D / 2, s * FOOT_Y - 0.02, 0.0), (TV_D / 2, s * FOOT_Y + 0.02, 0.012), TV_BACK,
                         material=RUBBER_FEET))  # a foot
        parts.append(_bx((PF + 0.006, s * FOOT_Y - 0.012, 0.012), (PF + 0.024, s * FOOT_Y + 0.012, PANEL_BOTTOM),
                         TV_BACK))  # its post up into the panel
    meshes = [kit.box((PF - 0.0004, 0.0, (SCREEN_Z[0] + SCREEN_Z[1]) / 2),
                      (0.0008, SCREEN_Y[1] - SCREEN_Y[0], SCREEN_Z[1] - SCREEN_Z[0]), SCREEN_BLACK)]
    meshes += _text("TV", (PF - 0.0008, 0.010, PANEL_BOTTOM + 0.002), (0, -1, 0), (0, 0, 1), 0.0016, LOGO, (-1, 0, 0))
    return parts, meshes


_TV, _TV_M = _tv_parts()
TV = kit.Prop("tv", pos=(TV_X, TV_Y, TOP + 0.0005), size=(TV_D, TV_W, TV_H), mass=TV_KG,
              meshes=tuple(m for m, _ in _TV) + tuple(_TV_M), colliders=tuple(c for _, c in _TV),
              material=PLASTIC, contact_offset=0.002)

# The six fingerprints (TV frame, on the screen's front face): two rows of three on the left half, low on the screen
# where the top camera sees them.
SMUDGES = tuple((y, z) for z in (0.095, 0.165) for y in (0.06, 0.12, 0.18))
SMUDGE_R = 0.010  # m: a fingerprint about 2 cm across

# -- the spray bottle (its frame: origin at the middle of its base; x the nozzle's direction; z up) ---------------------
# A 16.9 fl oz (500 mL) screen-cleaner trigger bottle: 9.5 in tall, 3.53 in wide (a 16.9 oz screen cleaner kit's
# listing on Zoro). The body is a flattened oval 9 cm wide and 6 cm front to back; the trigger head's shape, the
# trigger's hinge and its stroke are estimated from maker photos. The bottle is half full: 250 mL of cleaner (0.25 kg)
# and the bottle with its head (about 0.06 kg).
BOTTLE_KG = 0.31
BODY_HX, BODY_HY, BODY_H = 0.030, 0.045, 0.150
SHOULDER_R, NECK_TOP = 0.022, 0.168
SHROUD_X, SHROUD_HY, SHROUD_Z = (-0.030, 0.030), 0.014, (0.168, 0.235)
NOZZLE_X, NOZZLE_HY, NOZZLE_Z = (0.030, 0.048), 0.008, (0.214, 0.232)
NOZZLE = np.array([0.048, 0.0, 0.223])  # the nozzle's orifice
PIVOT = (0.044, 0.0, 0.212)  # the trigger's hinge (about the bottle's y), just under the nozzle
TRIG_X, TRIG_HY, TRIG_Z = (-0.006, 0.006), 0.011, (-0.044, -0.004)  # the trigger blade, in its hinge's frame
TRIGGER_UP = 0.20  # rad: the trigger's full stroke (4.8 mm at the finger, 2.4 cm under the hinge)
# The return spring: 3.2 N m/rad, so a finger 2.4 cm under the hinge pulls 20 N to fire (FIRE) and a 6 N grip moves
# the trigger 0.045 rad (a quarter of the way to firing). Real trigger sprayers dispense only near the end of a full
# pull; the force is estimated.
TRIGGER_K = 3.2
FIRE, REARM = 0.15, 0.07  # rad: a shot leaves as the trigger passes FIRE; it fires again only after easing below REARM
BOTTLE_X0, BOTTLE_Y0, BOTTLE_YAW0 = 0.315, -0.120, math.pi / 2  # nozzle toward the robot's left (+y)
BX, BY = BODY_HX, BODY_HY


def _bottle_parts():
    r = BODY_HX  # the oval's round ends: radius 3 cm at y = +-1.5 cm
    meshes = [kit.box((0.0, 0.0, BODY_H / 2), (2 * BX, 2 * (BY - r), BODY_H), BOTTLE_LOOK),
              kit.cylinder((0.0, BY - r, BODY_H / 2), r, BODY_H, BOTTLE_LOOK),
              kit.cylinder((0.0, -(BY - r), BODY_H / 2), r, BODY_H, BOTTLE_LOOK),
              kit.box((0.0, 0.0, 0.040), (2 * BX - 0.004, 2 * BY - 0.004, 0.076), LIQUID),  # the cleaner, half full
              kit.cylinder((0.0, 0.0, (BODY_H + NECK_TOP) / 2), SHOULDER_R, NECK_TOP - BODY_H, HEAD),
              kit.box(((SHROUD_X[0] + SHROUD_X[1]) / 2, 0.0, (SHROUD_Z[0] + SHROUD_Z[1]) / 2),
                      (SHROUD_X[1] - SHROUD_X[0], 2 * SHROUD_HY, SHROUD_Z[1] - SHROUD_Z[0]), HEAD),
              kit.box(((NOZZLE_X[0] + NOZZLE_X[1]) / 2, 0.0, (NOZZLE_Z[0] + NOZZLE_Z[1]) / 2),
                      (NOZZLE_X[1] - NOZZLE_X[0], 2 * NOZZLE_HY, NOZZLE_Z[1] - NOZZLE_Z[0]), TRIGGER_LOOK)]
    # the label: a blue band on the front, the back and the side toward the robot, with SCREEN CLEANER on it
    for s in (-1, 1):
        meshes.append(kit.box((s * (BX + 0.0004), 0.0, 0.085), (0.0008, 2 * (BY - r) + 0.02, 0.09), LABEL))
    meshes.append(kit.box((0.0, BY + 0.0004, 0.085), (0.05, 0.0008, 0.09), LABEL))
    px = 0.0021
    for line, z in (("SCREEN", 0.098), ("CLEANER", 0.083)):
        w = (len(line) * 4 - 1) * px
        meshes += _text(line, (w / 2, BY + 0.0008, z), (-1, 0, 0), (0, 0, 1), px, LABEL_TEXT, (0, 1, 0))
        meshes += _text(line, (BX + 0.0008, -w / 2, z), (0, 1, 0), (0, 0, 1), px, LABEL_TEXT, (1, 0, 0))
    cols = [kit.Collider((0.0, 0.0, BODY_H / 2), size=(2 * BX, 2 * (BY - r), BODY_H)),
            kit.Collider((0.0, BY - r, BODY_H / 2), radius=r, height=BODY_H),
            kit.Collider((0.0, -(BY - r), BODY_H / 2), radius=r, height=BODY_H),
            kit.Collider((0.0, 0.0, (BODY_H + NECK_TOP) / 2), radius=SHOULDER_R, height=NECK_TOP - BODY_H),
            kit.Collider(((SHROUD_X[0] + SHROUD_X[1]) / 2, 0.0, (SHROUD_Z[0] + SHROUD_Z[1]) / 2),
                         size=(SHROUD_X[1] - SHROUD_X[0], 2 * SHROUD_HY, SHROUD_Z[1] - SHROUD_Z[0])),
            kit.Collider(((NOZZLE_X[0] + NOZZLE_X[1]) / 2, 0.0, (NOZZLE_Z[0] + NOZZLE_Z[1]) / 2),
                         size=(NOZZLE_X[1] - NOZZLE_X[0], 2 * NOZZLE_HY, NOZZLE_Z[1] - NOZZLE_Z[0]))]
    return tuple(meshes), tuple(cols)


_B_MESH, _B_COL = _bottle_parts()
_TRIG_C = ((TRIG_X[0] + TRIG_X[1]) / 2, 0.0, (TRIG_Z[0] + TRIG_Z[1]) / 2)
_TRIG_S = (TRIG_X[1] - TRIG_X[0], 2 * TRIG_HY, TRIG_Z[1] - TRIG_Z[0])
BOTTLE = kit.Mechanism(
    "bottle", pos=(BOTTLE_X0, BOTTLE_Y0, TOP + 0.0005), yaw=BOTTLE_YAW0, size=(0.10, 0.09, SHROUD_Z[1]),
    mass=BOTTLE_KG, meshes=_B_MESH, colliders=_B_COL, material=PLASTIC, contact_offset=0.002,
    links=(kit.Link("trigger", "revolute", "Y", anchor=PIVOT, lower=0.0, upper=TRIGGER_UP, start=0.0, mass=0.010,
                    meshes=(kit.box(_TRIG_C, _TRIG_S, TRIGGER_LOOK),),
                    colliders=(kit.Collider(_TRIG_C, size=_TRIG_S),),
                    law=mk.Spring(rest=0.0, k=TRIGGER_K, damp=0.02)),))

# -- the cloth (its frame: origin at the middle of its underside; x along its length) -----------------------------------
# A 12 x 12 in microfiber cloth (the size most screen-cleaner kits include), folded in eighths: 15.2 x 7.6 cm, eight
# layers, about 1.2 cm thick; 28 g (300 g/m2). Modeled as one rigid pad; its two outer faces are "bottom" and "top".
PAD_L, PAD_W, PAD_T = 0.152, 0.076, 0.012
PAD_KG = 0.028
PAD_X0, PAD_Y0 = 0.250, 0.170
CLOTH_PAD = kit.Prop("cloth", pos=(PAD_X0, PAD_Y0, TOP + 0.0005), size=(PAD_L, PAD_W, PAD_T), mass=PAD_KG,
                     meshes=(kit.box((0, 0, PAD_T / 2), (PAD_L, PAD_W, PAD_T), CLOTH),
                             kit.box((PAD_L / 2 - 0.004, 0, PAD_T / 2), (0.008, PAD_W + 0.0006, PAD_T + 0.0006),
                                     CLOTH_EDGE)),
                     colliders=(kit.Collider((0, 0, PAD_T / 2), size=(PAD_L, PAD_W, PAD_T)),),
                     material=MICROFIBER, contact_offset=0.002, contact_report=True)

OBJECTS = [TV, BOTTLE, CLOTH_PAD]
gripkit.use_rubber_pads()  # the YAM's finger pads are rubber-faced

# -- scored thresholds -----------------------------------------------------------------------------------------------
CONE_DEG = 12.0  # the mist cone's half-angle (a fine-mist setting; estimated from maker photos)
SPRAY_RANGE = 0.60  # m: how far a ray carries (mist settles within about half a meter)
DIP_TUBE_DEG = 60.0  # the bottle tipped more than this from upright: the dip tube draws air, no shot
DAMP_HITS = 20  # rays on one face of the cloth make it damp (one full shot is 24 rays)
TOUCH = 0.0005  # m: the cloth is on the screen when some point of a face is within 0.5 mm of it (the solver rests
# touching surfaces at 0) ...
GIVE = 0.004  # m: ... and that face then touches every print where it lies within 4 mm of the screen: the folded
# cloth (eight layers, 1.2 cm) compresses that much under a press, so it lies on the glass a few degrees off
FLAT_DEG = 10.0  # the face turned to the screen within 10 degrees
PRESS_N = 1.0  # N: ... and only while the screen pushes back on the cloth with at least 1 N (PhysX's contact force
# between the two bodies along the screen's normal, averaged over the control step): about four times the cloth's
# own weight, so a cloth resting or hovering on the glass wipes nothing
DEEP = 0.005  # m: a face point more than 5 mm inside the screen is not a pose the rigid pad can take: no touch
SOAK = 0.010  # m: a shot while a face of the cloth lies within 1 cm of the screen soaks through to it
WIPE_TRAVEL = 0.03  # m: a print is wiped once the touching face has slid 3 cm over it
CLEAN_NEED = 5  # fingerprints cleaned (of 6) for full credit
TV_MOVED, TV_TIPPED = 0.03, math.radians(10.0)  # the TV shoved 3 cm or tipped 10 degrees: knocked
UPRIGHT = math.radians(10.0)  # the bottle stood back up within 10 degrees
FLAT = math.radians(20.0)  # the cloth laid down within 20 degrees of flat
SEAT_LO, SEAT_HI = -0.003, 0.006  # m: resting on the table
CLEAR = 0.010  # m: hands clear: every hand link at least 1 cm from the bottle and the cloth
STILL_STEP = 0.001  # m per control step: still
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
HAND_LINKS = ("link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}  # "right_robot/tip_left" -> (pos, R) in the env frame at this control step (filled in the sim)
PRESS: dict = {"n": None, "vec": None}  # the screen's push on the cloth along its normal (N) at this control step (the sim fills it;
# offline tests set it)


def rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def pose(objects, name):
    o = objects[name]
    return np.asarray(o.pos, float), rotm(o.quat_xyzw)


def _tilt(R) -> float:
    return math.acos(max(-1.0, min(1.0, float(R[2, 2]))))


# -- the spray: rays from the nozzle -----------------------------------------------------------------------------------
def cone_dirs() -> np.ndarray:
    """24 unit directions in the bottle's frame: the axis (+x), 8 at half the cone's angle and 15 at its edge."""
    out = [np.array([1.0, 0.0, 0.0])]
    for n, a in ((8, CONE_DEG / 2), (15, CONE_DEG)):
        t = math.radians(a)
        for k in range(n):
            p = 2 * math.pi * k / n
            out.append(np.array([math.cos(t), math.sin(t) * math.cos(p), math.sin(t) * math.sin(p)]))
    return np.array(out)


_DIRS = cone_dirs()
# The TV's solid parts (TV frame) as boxes (lo, hi): what a ray can wet.
_TV_BOXES = tuple((np.asarray(c.center) - np.asarray(c.size) / 2, np.asarray(c.center) + np.asarray(c.size) / 2)
                  for c in TV.colliders)
_PAD_BOX = (np.array([-PAD_L / 2, -PAD_W / 2, 0.0]), np.array([PAD_L / 2, PAD_W / 2, PAD_T]))


def _ray_box(o, d, lo, hi):
    """Entry distance of the ray o + t d (t >= 0) into the box lo..hi, or None."""
    t0, t1 = 0.0, math.inf
    for k in range(3):
        if abs(d[k]) < 1e-12:
            if o[k] < lo[k] or o[k] > hi[k]:
                return None
            continue
        a, b = (lo[k] - o[k]) / d[k], (hi[k] - o[k]) / d[k]
        if a > b:
            a, b = b, a
        t0, t1 = max(t0, a), min(t1, b)
        if t0 > t1:
            return None
    return t0


def nozzle(objects):
    """The nozzle's orifice (world) and the bottle's frame."""
    p, R = pose(objects, "bottle")
    return p + R @ NOZZLE, R


RAY_R = 0.003  # m: a ray's mist reaches an arm's surface point this close to it


GRIP_LINKS = ("gripper", "tip_left", "tip_right")
HOLD_GAP = 0.003  # m: a hand whose gripper or finger links come this close to the bottle is holding it


def link_gap(objects, name, key) -> float:
    """Gap (m) between one arm link ("left_robot/tip_left") and an object's parts; 0.05 when over 5 cm."""
    o, R = LINK_POSES[key]
    o, R = np.asarray(o, float), np.asarray(R, float)
    link = key.split("/")[1]
    q = yg.near(link, lambda x: o + x @ R.T, lambda x: _gap(objects, name, x), 0.05)
    return min(0.05, float(_gap(objects, name, q).min())) if len(q) else 0.05


def holding_links(objects) -> set:
    """The gripper and finger links of every hand touching the bottle (within HOLD_GAP): the mist leaving the nozzle
    passes over the hand that works the trigger, so those links are not counted as sprayed."""
    out = set()
    for robot in ROBOTS:
        keys = [f"{robot}/{lk}" for lk in GRIP_LINKS if f"{robot}/{lk}" in LINK_POSES]
        if keys and min(link_gap(objects, "bottle", k) for k in keys) <= HOLD_GAP:
            out |= set(keys)
    return out


def _robot_t(o, d, skip) -> float:
    """Distance along the ray o + t d to the first surface point of an arm link (upper arm to fingertips), other
    than the links in `skip` (the holding hand's gripper and fingers), within RAY_R of the ray; inf if none."""
    best = math.inf
    for k, (p, R) in LINK_POSES.items():
        link = k.split("/")[1]
        if k in skip or link not in HAND_LINKS:
            continue
        pts, cen, rad = yg.cloud(link)
        c = np.asarray(p, float) + np.asarray(R, float) @ cen
        tc = float(np.dot(c - o, d))
        if np.linalg.norm(c - o - tc * d) > rad + RAY_R or tc < -rad:
            continue
        w = np.asarray(p, float) + pts @ np.asarray(R, float).T - o
        t = w @ d
        perp = np.linalg.norm(w - np.outer(t, d), axis=1)
        sel = (perp <= RAY_R) & (t >= 0.0)
        if sel.any():
            best = min(best, float(t[sel].min()))
    return best


def cast(objects):
    """One shot: where each ray of the cone lands. A list of (target, world point, extra): target 'tv', 'cloth'
    (extra: the face hit, 'bottom' or 'top' or 'edge'), 'robot' (any arm link but the holding hand's gripper and
    fingers), 'table' or 'air'
    (nothing within SPRAY_RANGE)."""
    o, Rb = nozzle(objects)
    tp, tR = pose(objects, "tv")
    cp, cR = pose(objects, "cloth")
    skip = holding_links(objects) if LINK_POSES else set()
    out = []
    for d_b in _DIRS:
        d = Rb @ d_b
        best = (SPRAY_RANGE, "air", None)
        if LINK_POSES:
            t = _robot_t(o, d, skip)
            if t < best[0]:
                best = (t, "robot", None)
        ot, dt = tR.T @ (o - tp), tR.T @ d
        for lo, hi in _TV_BOXES:
            t = _ray_box(ot, dt, lo, hi)
            if t is not None and t < best[0]:
                best = (t, "tv", None)
        oc, dc = cR.T @ (o - cp), cR.T @ d
        t = _ray_box(oc, dc, *_PAD_BOX)
        if t is not None and t < best[0]:
            q = oc + t * dc
            face = "bottom" if q[2] <= 1e-4 else "top" if q[2] >= PAD_T - 1e-4 else "edge"
            best = (t, "cloth", face)
        if d[2] < -1e-9:
            t = (TOP - o[2]) / d[2]
            if 0.0 <= t < best[0]:
                best = (t, "table", None)
        out.append((best[1], o + best[0] * d, best[2]))
    return out


def dip_tube_ok(objects) -> bool:
    _, R = pose(objects, "bottle")
    return _tilt(R) <= math.radians(DIP_TUBE_DEG)


# -- the cloth on the screen ---------------------------------------------------------------------------------------------
def _faces(objects):
    """Each face of the cloth: (name, its middle (world), its outward normal (world))."""
    cp, cR = pose(objects, "cloth")
    return (("bottom", cp, cR @ np.array([0.0, 0.0, -1.0])), ("top", cp + cR @ np.array([0.0, 0.0, PAD_T]),
                                                             cR @ np.array([0.0, 0.0, 1.0])))


def face_on_screen(objects, near=SOAK):
    """The face of the cloth turned toward the screen (within 30 degrees) with its middle within `near` of the
    screen's plane (or pressed in, up to DEEP) and over the panel: 'bottom', 'top' or None."""
    tp, tR = pose(objects, "tv")
    into = tR @ np.array([1.0, 0.0, 0.0])  # from the room into the screen
    for face, c, n in _faces(objects):
        cl = tR.T @ (c - tp)
        if (float(np.dot(n, into)) >= math.cos(math.radians(30.0)) and -DEEP <= PF - cl[0] <= near
                and -TV_W / 2 <= cl[1] <= TV_W / 2 and PANEL_BOTTOM <= cl[2] <= TV_H):
            return face
    return None


_GRID = np.array([[x, y] for x in np.linspace(-PAD_L / 2, PAD_L / 2, 5) for y in np.linspace(-PAD_W / 2, PAD_W / 2, 3)])


def touches(objects) -> dict:
    """The prints a face of the cloth touches now: {print index: face}. The face must be turned to the screen within
    FLAT_DEG and really on it: pressed with at least PRESS_N (the contact force PhysX reports between the cloth and
    the TV), and the nearest of 15 points across it (a 5 x 3 grid, corners included) within TOUCH of the screen's
    plane. It then touches each print whose middle lies under the cloth where the face is within GIVE of the
    screen (the folded cloth's give), measured at the print: a hover touches nothing, and a cloth tilted more than its
    give touches only the prints under its touching edge."""
    tp, tR = pose(objects, "tv")
    cp, cR = pose(objects, "cloth")
    into = tR @ np.array([1.0, 0.0, 0.0])
    under = smudges_under(objects)
    out = {}
    if PRESS["vec"] is not None:  # the push along the screen's normal (a contact only pushes: its size)
        PRESS["n"] = abs(float(np.dot(PRESS["vec"], tR @ np.array([-1.0, 0.0, 0.0]))))
    if PRESS["n"] is None:
        if "isaaclab" in sys.modules:
            raise RuntimeError("spraytv: no cloth-to-screen contact force read for the wipe")
        return out
    if PRESS["n"] < PRESS_N:
        return out  # not pressed on the screen
    for face, c, n in _faces(objects):
        if float(np.dot(n, into)) < math.cos(math.radians(FLAT_DEG)):
            continue
        z = 0.0 if face == "bottom" else PAD_T
        pts = cp + np.column_stack([_GRID, np.full(len(_GRID), z)]) @ cR.T
        gaps = PF - (pts - tp) @ tR[:, 0]  # >0: in front of the screen
        if not (-DEEP <= gaps.min() <= TOUCH):
            continue
        for i in under:
            y, zz = SMUDGES[i]
            w = tp + tR @ np.array([PF, y, zz])
            # the face's own gap where the print is: along the screen's normal from the print to the face's plane
            gap = float(np.dot(w - c, n)) / float(np.dot(n, into))
            if -DEEP <= gap <= GIVE:
                out[i] = face
    return out


def cloth_on_screen_yz(objects) -> np.ndarray:
    """The cloth's middle in the screen's plane (TV frame y, z)."""
    tp, tR = pose(objects, "tv")
    cp, _ = pose(objects, "cloth")
    return (tR.T @ (cp - tp))[1:]


def smudges_under(objects) -> list:
    """Indices of the fingerprints whose middle lies under the cloth's footprint (its length and width)."""
    tp, tR = pose(objects, "tv")
    cp, cR = pose(objects, "cloth")
    out = []
    for i, (y, z) in enumerate(SMUDGES):
        w = tp + tR @ np.array([PF, y, z])
        q = cR.T @ (w - cp)
        if abs(q[0]) <= PAD_L / 2 and abs(q[1]) <= PAD_W / 2:
            out.append(i)
    return out


# -- set down and hands clear ------------------------------------------------------------------------------------------
def bottle_standing(objects) -> bool:
    p, R = pose(objects, "bottle")
    return bool(_tilt(R) <= UPRIGHT and SEAT_LO <= p[2] - TOP <= SEAT_HI)


_PAD_CORNERS = np.array([[sx * PAD_L / 2, sy * PAD_W / 2, z] for sx in (-1, 1) for sy in (-1, 1) for z in (0, PAD_T)])


def cloth_down(objects) -> bool:
    """The cloth lying on the table (either face down), flat within 20 degrees, its lowest corner on the table."""
    p, R = pose(objects, "cloth")
    flat = min(_tilt(R), math.pi - _tilt(R)) <= FLAT
    low = float((p + _PAD_CORNERS @ R.T)[:, 2].min()) - TOP
    return bool(flat and SEAT_LO <= low <= SEAT_HI)


def tv_knocked(objects) -> bool:
    p, R = pose(objects, "tv")
    return bool(np.hypot(p[0] - TV_X, p[1] - TV_Y) > TV_MOVED or _tilt(R) > TV_TIPPED)


def _part_boxes(colliders):
    out = []
    for c in colliders:
        if c.size is not None:
            out.append((np.asarray(c.center, float), np.asarray(c.size, float) / 2))
        else:
            h = c.height / 2
            out.append((np.asarray(c.center, float), np.array([c.radius, c.radius, h])))
    return out


_GRAB_PARTS = {"bottle": _part_boxes(BOTTLE.colliders) + [(np.asarray(PIVOT) + np.asarray(_TRIG_C),
                                                           np.asarray(_TRIG_S) / 2 + 0.006)],
               "cloth": _part_boxes(CLOTH_PAD.colliders)}


def _gap(objects, name, q) -> np.ndarray:
    p, R = pose(objects, name)
    q = np.atleast_2d(np.asarray(q, float))
    gaps = []
    for c, h in _GRAB_PARTS[name]:
        d = np.abs((q - (p + R @ c)) @ R) - h
        gaps.append(np.linalg.norm(np.maximum(d, 0.0), axis=1) + np.minimum(d.max(axis=1), 0.0))
    return np.min(gaps, axis=0)


def hand_gap(objects, name) -> float:
    """Smallest gap between either arm's links (upper arm to fingertips) and the object's parts (their boxes); 0.05
    when none is within 5 cm. In the simulator, no link poses read is an error."""
    if not LINK_POSES:
        if "isaaclab" in sys.modules:
            raise RuntimeError("spraytv: no arm link poses for the hands-clear check")
        return 0.05
    p, _ = pose(objects, name)
    out = 0.05
    for k, (o, R) in LINK_POSES.items():
        link = k.split("/")[1]
        if link not in HAND_LINKS:
            continue
        o, R = np.asarray(o, float), np.asarray(R, float)
        _, cen, rad = yg.cloud(link)
        if float(np.linalg.norm(o + R @ cen - p)) - rad > 0.30:
            continue

        def gap(x, name=name):
            return _gap(objects, name, x)
        q = yg.near(link, lambda x, o=o, R=R: o + x @ R.T, gap, 0.05)
        if len(q):
            out = min(out, float(gap(q).min()))
    return out


# -- the trial's ledger ----------------------------------------------------------------------------------------------
class Ledger:
    """One trial: the shots and where they landed, the cloth's damp faces, each fingerprint's state, the latched
    failures, and the end state."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.n = 0
        self.armed = True
        self.shots = 0
        self.hits = {"tv": 0, "cloth": 0, "robot": 0, "table": 0, "air": 0}
        self.face_hits = {"bottom": 0, "top": 0, "edge": 0}
        self.tv_wet = self.knocked = self.soaked_through = self.robot_wet = False
        self.state = ["dirty"] * len(SMUDGES)  # dirty / smeared (wiped dry) / clean (wiped damp)
        self.window = {}  # print -> (face, lowest y z, highest y z) of the cloth's middle in one unbroken touch
        self.drops = []  # (target, point in its object's frame) for drawing
        self.last = None
        self.down = self.clear = False
        self.good = 0
        self.objects = None

    def damp(self, face) -> bool:
        return face in ("bottom", "top") and self.face_hits[face] >= DAMP_HITS

    def _shoot(self, objects):
        self.shots += 1
        on_screen = face_on_screen(objects)
        if on_screen is not None:
            self.soaked_through = True  # sprayed while lying on the screen: the cleaner soaks through to the panel
        for target, w, face in cast(objects):
            self.hits[target] += 1
            if target == "cloth":
                self.face_hits[face] += 1
            if target == "tv":
                self.tv_wet = True
            if target == "robot":
                self.robot_wet = True
            if target in ("tv", "cloth"):
                p, R = pose(objects, target)
                self.drops.append((target, R.T @ (w - p)))
            elif target == "table":
                self.drops.append(("table", w))
        self.drops = self.drops[-MAX_DROPS:]

    def step(self, objects) -> None:
        self.n += 1
        self.objects = objects
        q = kit.joint(objects, "bottle/trigger").pos
        if self.armed and q >= FIRE:
            self.armed = False
            if dip_tube_ok(objects):
                self._shoot(objects)
        elif not self.armed and q < REARM:
            self.armed = True
        self.knocked |= tv_knocked(objects)
        yz = cloth_on_screen_yz(objects)
        touch = touches(objects)
        for i in list(self.window):
            if touch.get(i) != self.window[i][0]:
                del self.window[i]  # the touch broke (lifted off, or the other face): a new stroke starts
        for i, face in touch.items():
            f, lo, hi = self.window.get(i, (face, yz, yz))
            lo, hi = np.minimum(lo, yz), np.maximum(hi, yz)
            self.window[i] = (face, lo, hi)
            if float(np.max(hi - lo)) >= WIPE_TRAVEL:  # the cloth has slid 3 cm (in y or z) in this one touch
                if self.damp(face):
                    self.state[i] = "clean"
                elif self.state[i] == "dirty":
                    self.state[i] = "smeared"
        pos = np.concatenate([pose(objects, k)[0] for k in ("bottle", "cloth", "tv")])
        still = self.last is not None and float(np.abs(pos - self.last).max()) <= STILL_STEP
        self.last = pos
        self.down = bottle_standing(objects) and cloth_down(objects)
        self.clear = self.down and min(hand_gap(objects, "bottle"), hand_gap(objects, "cloth")) >= CLEAR
        self.good = self.good + 1 if (self.tier() == 1.0 and still) else 0
        if _PROBE:
            ls = {k: v[0] for k, v in LINK_POSES.items() if k.endswith("/gripper")}
            bp, bR = pose(objects, "bottle")
            cp, cR = pose(objects, "cloth")
            tp, tR = pose(objects, "tv")
            corners = [PF - (tR.T @ (cp + cR @ np.array([sx * PAD_L / 2, sy * PAD_W / 2, 0.0]) - tp))[0]
                       for sx in (-1, 0, 1) for sy in (-1, 1)]  # the bottom face's edge points in front of the screen
            with open(_PROBE, "a") as fh:
                fh.write("press %s fx %s gaps %s touch %s " % ("-" if PRESS["n"] is None else "%.2f" % PRESS["n"],
                                                              "-" if PRESS["vec"] is None else "%.2f" % PRESS["vec"][0],
                                                        " ".join("%.4f" % g for g in corners),
                                                "".join(str(i) for i in sorted(touch))))
                fh.write("%d trig %.3f shots %d hits %s faces %s smudge %s onscr %s down %d clear %d tvwet %d knock %d "
                         "b %s btilt %.1f c %s ctilt %.1f tv %s tvtilt %.1f tvyaw %.1f g %s\n"
                         % (self.n, q, self.shots, self.hits, self.face_hits, "".join(s[0] for s in self.state),
                            face_on_screen(objects),
                            int(self.down), int(self.clear), int(self.tv_wet or self.soaked_through or self.robot_wet),
                            int(self.knocked), " ".join("%.4f" % v for v in bp), math.degrees(_tilt(bR)),
                            " ".join("%.4f" % v for v in cp), math.degrees(_tilt(cR)),
                            " ".join("%.4f" % v for v in tp), math.degrees(_tilt(tR)), math.degrees(math.atan2(tR[1, 0], tR[0, 0])),
                            " ".join("%s=%s" % (k.split("_")[0], " ".join("%.3f" % v for v in x))
                                     for k, x in ls.items())))

    def cleaned(self) -> int:
        return sum(s == "clean" for s in self.state)

    def wiped(self) -> int:
        return sum(s != "dirty" for s in self.state)

    def tier(self) -> float:
        if self.tv_wet or self.soaked_through or self.knocked or self.robot_wet:
            return 0.0
        if self.cleaned() >= CLEAN_NEED:
            return 1.0 if self.clear else 0.5
        if self.wiped() >= CLEAN_NEED:
            return 0.5
        if self.damp("bottom") or self.damp("top") or self.wiped() > 0:
            return 0.25
        return 0.0

    def score(self) -> float:
        s = self.tier()
        if s == 1.0 and self.good < HOLD_STEPS:
            return 0.5
        return s


_L = None


def _update(objects):
    global _L
    if _L is None or _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else ("obj", id(objects))
    if key == _L.key:
        return
    _L.key = key
    _L.step(objects)


def success(objects):
    """Five or more prints cleaned with a damp face of the cloth, never any mist on the TV or an arm, the bottle
    standing and the cloth lying on the table, hands clear, all still: this step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """0 if at any time a ray of mist reached any part of the TV (screen, bezel, back or feet) or any arm link (upper arm
    to fingertips) other than the gripper and fingers of a hand touching the bottle, the cloth was sprayed while a
    face of it lay within 1 cm of the screen (the cleaner soaks through to the panel), or the TV was shoved 3 cm or
    tipped 10 degrees. A face of the cloth with 20 or more rays of mist on it is damp. The cloth is pressed on the
    screen when the screen pushes back on it with at least 1 N and a point of a face is within 0.5 mm of it, the
    face turned to it within 10 degrees; that face touches each print under it where it lies within 4 mm of the
    screen (the folded cloth's give); the print is wiped once the cloth has slid 3 cm (in y or z) during one
    unbroken press: cleaned by a damp face, smeared by a dry one. Otherwise 1.0: at least 5 of the 6 prints cleaned,
    then the bottle standing on the table (upright within 10 degrees) and the cloth lying on the table (flat within
    20 degrees), every arm link 1 cm or more from both, all still for 1 s. 0.5: 5 or more cleaned but not yet set
    down and let go (or not yet still for 1 s); or 5 or more smeared with a dry face. 0.25: a face of the cloth
    damp, or 1 to 4 prints wiped. 0: nothing done."""
    _update(objects)
    return _L.score()


# -- visuals (never scored): the fingerprints and the mist's droplets ---------------------------------------------------
MAX_DROPS = 96
_VIS = {"markers": None, "failed": False}


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        def sphere(rgb, rough):
            return sim_utils.SphereCfg(radius=1.0, visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=tuple(kit._linear(c) for c in rgb), roughness=rough))
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/World/Visuals/spraytv",
            markers={"smudge": sphere((0.62, 0.60, 0.55), 0.9), "smear": sphere((0.42, 0.41, 0.38), 0.9),
                     "drop": sphere((0.55, 0.80, 1.0), 0.05)}))
    except Exception as e:  # noqa: BLE001  (visuals are optional)
        print(f"spraytv: no visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _draw(base) -> None:
    if _L is None or _L.objects is None or _L.episode != kit.episode():
        return
    m = _markers()
    if m is None:
        return
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    obj = _L.objects
    tp, tR = pose(obj, "tv")
    tr, sc, ix = [], [], []
    for (y, z), st in zip(SMUDGES, _L.state):
        if st == "clean":
            continue
        tr.append(tp + tR @ np.array([PF - 0.0008, y, z])), ix.append(0 if st == "dirty" else 1)
        sc.append(tR @ np.array([0.0006, SMUDGE_R * (1.0 if st == "dirty" else 1.5), SMUDGE_R * 0.8]))
    for target, q in _L.drops:
        if target == "table":
            w = q
        else:
            p, R = pose(obj, target)
            w = p + R @ q
        tr.append(w), sc.append(np.array([0.0025, 0.0025, 0.0025])), ix.append(2)
    while len(tr) < len(SMUDGES) + MAX_DROPS:  # park unused markers out of sight
        tr.append(np.array([0.0, 0.0, -10.0])), sc.append(np.array([1e-4, 1e-4, 1e-4])), ix.append(2)
    m.visualize(translations=np.array(tr, float) + origin, orientations=np.tile([0, 0, 0, 1.0], (len(tr), 1)),
                scales=np.abs(np.array(sc, float)) + 1e-4, marker_indices=np.array(ix, int))


def _read_links(base) -> None:
    """Every hand link's pose (env frame) at this control step, into LINK_POSES; then the visuals. A missing arm or
    link, or finger pads left without their rubber, stops the trial: the hand checks would otherwise pass silently."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"spraytv: no {robot} in the scene for the hand checks") from e
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in HAND_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"spraytv: {robot} has no link(s) {missing} for the hand checks")
        for link in HAND_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
    if gripkit._STATE.get("bound", 0) < 0:
        raise RuntimeError("spraytv: the finger pads' rubber material could not be bound")
    try:
        _draw(base)
    except Exception as e:  # noqa: BLE001  (never let visuals break a run)
        if not _VIS["failed"]:
            print(f"spraytv: visuals off ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True


def _np(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    if hasattr(x, "numpy"):
        return np.asarray(x.numpy(), np.float64)
    return np.asarray(x, np.float64)


_CONTACT: dict = {}


def _sample() -> np.ndarray:
    return _np(_CONTACT["view"].get_contact_force_matrix(dt=_CONTACT["dt"])).reshape(3)


def _read_press(base) -> None:
    """The TV's contact force on the cloth over this control step (PhysX rigid contact view on the cloth, whose USD
    carries PhysxContactReportAPI, filtered to the TV), read at every physics substep and averaged, into PRESS.
    Anything missing is an error, never a silent pass."""
    import traceback
    try:
        if "view" not in _CONTACT:
            import isaaclab.sim as sim_utils
            from isaaclab_physx.physics import PhysxManager

            paths = []
            for n in ("cloth", "tv"):
                prims = sim_utils.find_matching_prims(base.scene[n].cfg.prim_path)
                if not prims:
                    raise RuntimeError(f"spraytv: no prim for {n}")
                paths.append(prims[0].GetPath().pathString)
            view = PhysxManager.get_physics_sim_view()
            _CONTACT["view"] = view.create_rigid_contact_view([paths[0]], filter_patterns=[[paths[1]]],
                                                              max_contact_data_count=64)
            _CONTACT["dt"] = float(getattr(base, "physics_dt", 1 / 240))
            _CONTACT["sum"], _CONTACT["k"] = np.zeros(3), 0
            write = base.scene.write_data_to_sim

            def write_data_to_sim(*a, **k):  # before each physics substep: add the last substep's force
                _CONTACT["sum"] = _CONTACT["sum"] + _sample()
                _CONTACT["k"] += 1
                return write(*a, **k)
            base.scene.write_data_to_sim = write_data_to_sim
            print(f"spraytv: contact view cloth {paths[0]} <- tv {paths[1]}, read every physics substep",
                  file=sys.stderr, flush=True)
        last = _sample()  # the substep that just ran (not yet counted)
        k = _CONTACT["k"]
        # the mean over the substeps since the last control step (a single substep's reading chatters between 0 and
        # about twice the press as the contact settles)
        PRESS["vec"] = (_CONTACT["sum"] + last) / (k + 1) if k else last
        _CONTACT["sum"], _CONTACT["k"] = np.zeros(3), 0
    except Exception:
        traceback.print_exc(file=sys.stderr)
        raise


def _warm(base) -> None:
    """Build the hand links' hull points on the first control step (the app is up by then; the task file loads before
    it, and demo() never reaches yamgeom)."""
    if not _WARM:
        _WARM.append(1)
        yg.warm(HAND_LINKS)


_WARM: list = []
kit.STATE_HOOKS.append(_warm)
kit.STATE_HOOKS.append(_read_press)
kit.STATE_HOOKS.append(_read_links)

# -- scripted demo: open loop from the fixed layout ------------------------------------------------------------------
# Left hand: pinch the folded cloth across its width from above, lift it and turn it so its underside faces the
# bottle (fingers pointing to the robot's right), hold it there; after the spray, turn it to face the TV (fingers
# forward), press it on the screen and wipe across the fingerprints twice, then lay it back on the table.
# Right hand: come in from behind the bottle with the fingers level and pointing forward, one finger on the back of the
# trigger head and one on the trigger, grip, lift, squeeze the trigger twice toward the cloth, set the bottle back down.
P2 = math.pi / 2
FINGER_HALF_OPEN = 0.047
SPAN = SHROUD_X[0], TRIG_X[1] + PIVOT[0]  # the grip across the head: back of the shroud to the trigger's face
GRIP_Z = 0.176  # m: the right hand's tool point over the bottle's base (finger band 14.2-21.0 cm, under the nozzle)
HOLD_E, SQUEEZE_E = 0.0012, 0.0100  # m: each finger aimed this far into the head: a light hold, a full pull
SAG = 0.016  # m: the arm holding the bottle gives this much under it (the bottle rises 2.4 of the 4 cm commanded)
LIFT = 0.040
SPRAY_GAP = 0.12  # m: the nozzle to the cloth
PAD_TIP_IN = 0.004  # m: the cloth's face this far past the fingertips (pinched 8 mm of its 12 mm)
WIPE_PRESS = 0.003  # m: the cloth aimed this far into the screen (the arm gives)
HOVER_GAP = 0.0004  # m: the 'hover' variant aims the cloth this far short of the screen instead
WIPE_Z = 0.130  # m: the cloth's middle over the table while wiping
WIPE_Y = (0.02, 0.22)


def finger_for(half: float) -> float:
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * min(FINGER_HALF_OPEN, max(0.0, half)) / FINGER_HALF_OPEN


def _tw(duration, pos, finger, label, **rot):
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label, **rot)


def _right():
    """The bottle's hand. The tool point sits midway across the head, 5 mm past the bottle's middle."""
    c, s = math.cos(BOTTLE_YAW0), math.sin(BOTTLE_YAW0)
    mid = 0.5 * (SPAN[0] + SPAN[1])
    half = 0.5 * (SPAN[1] - SPAN[0])
    grasp = np.array([BOTTLE_X0 + 0.005, BOTTLE_Y0 + s * mid, TOP + GRIP_Z])
    fwd = dict(pitch=-P2)
    op, hold, sq = finger_for(FINGER_HALF_OPEN), finger_for(half - HOLD_E), finger_for(half - SQUEEZE_E)
    up = grasp + np.array([0, 0, LIFT])
    seq = [kit.Home(0.5, label="start"),
           _tw(3.0, grasp - np.array([0.10, 0, 0]), op, "behind the bottle, fingers level", **fwd),
           _tw(2.0, grasp, op, "fingers either side of the trigger head", **fwd),
           _tw(1.0, grasp, hold, "grip the head", **fwd),
           _tw(1.5, up, hold, "lift the bottle", **fwd)]
    if VARIANT == "nothing":
        return [kit.Home(10.0, label="stay at the ready pose")]
    if VARIANT == "screen":  # turn the nozzle 30 degrees toward the TV and spray the screen
        aim = dict(roll=-math.pi / 6, pitch=-P2)
        seq += [_tw(2.0, up, hold, "aim at the screen", **aim)]
        for k in range(2):
            seq += [_tw(0.8, up, sq, f"squeeze {k + 1}", **aim), _tw(0.8, up, hold, "ease off", **aim)]
        seq += [_tw(2.0, up, hold, "aim back", **fwd)]
    elif VARIANT == "dry":
        seq += [_tw(3.2, up, hold, "hold, no spray", **fwd)]
    else:
        seq += [_tw(5.5, up, hold, "wait for the cloth", **fwd)]
        for k in range(2):
            seq += [_tw(0.8, up, sq, f"squeeze {k + 1} onto the cloth", **fwd),
                    _tw(0.8, up, hold, "ease off", **fwd)]
    seq += [_tw(1.5, grasp + np.array([0, 0, 0.002]), hold, "set the bottle down", **fwd),
            _tw(0.8, grasp + np.array([0, 0, 0.002]), op, "let go", **fwd),
            _tw(1.5, grasp - np.array([0.10, 0, 0]) + np.array([0, 0, 0.01]), op, "back out", **fwd),
            kit.Home(3.0, label="back to the ready pose"), kit.Home(30.0, label="wait")]
    return seq


def _left():
    """The cloth's hand. Pinches the folded cloth across its 7.6 cm width, its underside 4 mm past the fingertips."""
    width = PAD_W / 2
    op, cl = finger_for(width + 0.012), finger_for(width - 0.003)
    reach = 0.013 + PAD_TIP_IN  # the tool point to the cloth's underside, along the fingers
    pick = np.array([PAD_X0, PAD_Y0, TOP + reach])
    seq = [kit.Home(0.5, label="start"),
           _tw(3.0, pick + np.array([0, 0, 0.08]), op, "over the cloth"),
           _tw(1.5, pick, op, "fingers either side of the cloth"),
           _tw(1.0, pick, cl, "pinch the cloth"),
           _tw(1.5, pick + np.array([0, 0, 0.10]), cl, "lift it")]
    if VARIANT == "nothing":
        return [kit.Home(10.0, label="stay at the ready pose")]
    # where the right hand's nozzle is while it sprays (the bottle lifted, nozzle toward +y)
    nz = np.array([BOTTLE_X0 + 0.005, BOTTLE_Y0 + NOZZLE[0], TOP + LIFT - SAG + 0.0005 + NOZZLE[2]])
    hold_at = np.array([nz[0], nz[1] + SPRAY_GAP + reach, nz[2]])
    side = dict(roll=-P2)
    if VARIANT in ("dry", "screen"):
        seq += [_tw(6.4, pick + np.array([0, 0, 0.10]), cl, "keep the cloth up, away")]
    else:
        seq += [_tw(4.5, hold_at, cl, "hold the cloth up, face to the bottle", **side),
                _tw(6.5, hold_at, cl, "hold it for the spray", **side),
                _tw(2.0, hold_at - np.array([0, -0.03, 0]), cl, "away from the bottle", **side)]
    fwd = dict(pitch=-P2)
    x_press = TV_X + PF + (-HOVER_GAP if VARIANT == "hover" else WIPE_PRESS) - reach
    a = np.array([x_press - 0.05, WIPE_Y[0], TOP + WIPE_Z])
    seq += [_tw(3.5, np.array([0.30, 0.10, TOP + 0.20]), cl, "turn the cloth to face the TV", **fwd),
            _tw(2.0, a, cl, "in front of the screen", **fwd),
            _tw(1.5, a + np.array([0.05, 0, 0]), cl, "press the cloth on the screen", **fwd),
            _tw(4.0, np.array([x_press, WIPE_Y[1], TOP + WIPE_Z]), cl, "wipe across", **fwd),
            _tw(4.0, np.array([x_press, WIPE_Y[0], TOP + WIPE_Z]), cl, "wipe back", **fwd),
            _tw(1.5, a, cl, "off the screen", **fwd),
            _tw(4.0, pick + np.array([0, 0, 0.08]), cl, "over the table"),
            _tw(2.0, pick + np.array([0, 0, 0.002]), cl, "lay the cloth down"),
            _tw(0.8, pick + np.array([0, 0, 0.002]), op, "let go"),
            _tw(1.5, pick + np.array([0, 0, 0.08]), op, "up"),
            kit.Home(3.0, label="back to the ready pose"), kit.Home(2.5, label="wait")]
    return seq


def demo():
    """Spray the cloth, wipe the fingerprints, set both down. SPRAYTV_VARIANT=nothing: both arms stay at the ready
    pose; screen: the bottle is turned 30 degrees toward the TV and sprayed at the screen, then the cloth wipes (dry);
    dry: no spray, the dry cloth wipes; hover: as the demo, but the damp cloth is aimed 0.4 mm short of the screen (no
    press)."""
    return {"left": _left(), "right": _right()}
