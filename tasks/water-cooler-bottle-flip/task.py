"""A top-load water cooler (scaled, its body cut short so the arms can reach over it) stands near the robot with its
reservoir crown open and nearly empty; behind it a full 3-gallon cooler bottle stands on the table, uncapped, with a
clamp-on lifting collar that has a handle on each side. The worker swaps in the full bottle: both hands lift it, turn
it neck-down in one quick motion right over the crown and sit its neck in the receptacle. Turned over slowly, or held
neck-down anywhere but in the receptacle, the open neck glugs water onto the floor.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import pourkit as pk  # noqa: E402  (the water: the bottle's outflow, the reservoir, the spill total)
import tracekit  # noqa: E402  (per-trial memory: the irreversible spill and drop rules)
import yamkit as kit  # noqa: E402  (modeled props, gripper states, tilted demo grips)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Put the full bottle on the water cooler."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# The bottle: a 3-gallon (11.4 L) cooler bottle, about 26.5 cm across and 36 cm tall with a 5.5 cm neck, at 1:2.5
# scale. Frame origin on the handles' axis, 6.5 cm above its base. Clear blue PET.
BODY_R, BOTTOM, BODY_TOP = 0.053, -0.065, 0.040  # 10.6 cm across: wider than the gripper's 9.4 cm full opening
SH1_R, SH1_TOP = 0.040, 0.049  # the shoulder, in two steps
SH2_R, SH2_TOP = 0.025, 0.058  # it rests on this ring when seated neck-down
NECK_R, NECK_TOP = 0.011, 0.078  # 2.2 cm neck (5.5 cm real), 2 cm long, open (the cap's tear tab already pulled)
WALL = 0.003
# The lifting collar's two handles stick out 4.5 cm on each side (y), 4 cm long and 2.5 cm thick: the size one
# gripper holds reliably.
LUG_Y, LUG = 0.0755, (0.04, 0.045, 0.025)
WATER0 = 727.0  # mL: 11.36 L / 2.5^3
JUG_MASS = 0.80  # kg: 727 g of water and 73 g of bottle and collar (a real one is about 12.5 kg)
JUG_XY = (0.335, 0.0)
JUG_POS = (*JUG_XY, TOP - BOTTOM + 0.001)

# The cooler (fixed): a 12 cm square cabinet whose top (the crown) is 9 cm above the table; a real cooler's crown is
# about 95 cm up, which no YAM arm can reach over with a bottle, so only its top is to scale. The receptacle is a 3.3
# cm square opening (the real collar takes a 5.5 cm neck with about 1.4 cm to spare) running 3.5 cm down into the
# reservoir.
COOLER_XY = (0.205, 0.0)
COOLER_W, COOLER_H = 0.12, 0.09
BORE = 0.0165  # half side of the receptacle opening
BORE_D = 0.035
COOLER_POS = (*COOLER_XY, TOP)
CROWN_Z = TOP + COOLER_H
# The reservoir: 3 L real (192 mL scaled), with 60 mL left in it from the old bottle. A seated bottle feeds it until
# the water reaches its neck's tip (the cooler's air lock), 194 mL in all.
RES_R, RES_H = 0.045, 0.030
RES0 = 60.0
RES_FLOOR, BORE_LEN = 0.035, 0.025  # the reservoir's floor (cooler frame) and the receptacle tube above it
NECK_IN = NECK_TOP - SH2_TOP  # 2 cm: how far a seated neck reaches below the crown
SEAL_ML = (math.pi * RES_R ** 2 * RES_H + math.pi * BORE ** 2 * (BORE_LEN - NECK_IN)) * 1e6  # 195 mL

# How fast the open neck lets water out: a real 3-gallon bottle turned neck-down glugs empty in about 10 s (about
# 1.1 L/s through a 5.5 cm neck); the scaled bottle keeps those 10 s, 73 mL/s, so time in the trial is real time.
GLUG = 73.0

RIB = kit.Look((0.55, 0.72, 0.92), 0.2)
GRIP = kit.Look((0.15, 0.17, 0.20), 0.6)
WHITE = kit.Look((0.94, 0.94, 0.92), 0.5)
DARK = kit.Look((0.08, 0.08, 0.09), 0.7)
CROWN = kit.Look((0.62, 0.65, 0.70), 0.4)
WATER_RGB = (0.45, 0.70, 0.97)


def _ring(r, z, width, look, thick=0.0008):
    pts = [(r * math.cos(t), r * math.sin(t), z) for t in np.linspace(0, 2 * math.pi, 41)]
    return kit.band(pts, width, thick, look, normal=(0, 0, 1))


def _jug():
    # The renderer hides the water drawn inside behind any see-through surface, so the clear bottle is drawn only as
    # its edges and molded ribs, thin pale blue rings, and the water column shows its shape.
    rings = [(BODY_R, BOTTOM + 0.0015, 0.003), (BODY_R, -0.040, 0.0025), (BODY_R, 0.025, 0.0025),
             (BODY_R, BODY_TOP - 0.001, 0.002), (SH1_R + 0.002, SH1_TOP - 0.005, 0.002), (SH2_R, SH2_TOP, 0.002),
             (NECK_R, NECK_TOP - 0.004, 0.005), (NECK_R, SH2_TOP + 0.004, 0.002)]
    meshes = [_ring(r + 0.0005, z, w, RIB) for r, z, w in rings]
    meshes.append(kit.lathe([(BODY_R + 0.002, -0.016), (BODY_R + 0.002, 0.016)], GRIP))  # the lifting collar
    # A round base can roll on its rim in PhysX: a flat square foot 1 mm proud keeps it standing.
    cols = [kit.Collider((0, 0, (BOTTOM + 0.001 + BODY_TOP) / 2), radius=BODY_R, height=BODY_TOP - BOTTOM - 0.001),
            kit.Collider((0, 0, BOTTOM + 0.002), size=(0.07, 0.07, 0.004)),
            kit.Collider((0, 0, (BODY_TOP + SH1_TOP) / 2), radius=SH1_R, height=SH1_TOP - BODY_TOP),
            kit.Collider((0, 0, (SH1_TOP + SH2_TOP) / 2), radius=SH2_R, height=SH2_TOP - SH1_TOP),
            kit.Collider((0, 0, (SH2_TOP + NECK_TOP) / 2), radius=NECK_R, height=NECK_TOP - SH2_TOP)]
    for c in _lugs_local():
        meshes.append(kit.box(c, LUG, GRIP))
        cols.append(kit.Collider(c, size=LUG))
    return kit.Prop("jug", pos=JUG_POS, size=(2 * BODY_R, 2 * (LUG_Y + LUG[1] / 2), NECK_TOP - BOTTOM),
                    mass=JUG_MASS, meshes=tuple(meshes), colliders=tuple(cols))


def _lugs_local():
    return [(0.0, s * LUG_Y, 0.0) for s in (-1, 1)]


def _cooler():
    w, h, b, d = COOLER_W, COOLER_H, BORE, BORE_D
    side = (w - 2 * b) / 2
    parts = [((0, 0, (h - d) / 2), (w, w, h - d))]  # the cabinet below the receptacle's floor
    for s in (-1, 1):  # the top layer, built around the square receptacle opening
        parts.append(((s * (b + side / 2), 0, h - d / 2), (side, w, d)))
        parts.append(((0, s * (b + side / 2), h - d / 2), (2 * b, side, d)))
    meshes = [kit.box(c, s, WHITE) for c, s in parts]
    for s in (-1, 1):  # the gray crown plate around the opening (visual, 1 mm proud)
        meshes.append(kit.box((s * (b + 0.02), 0, h + 0.001), (0.04, 0.08 + 2 * b, 0.002), CROWN))
        meshes.append(kit.box((0, s * (b + 0.02), h + 0.001), (2 * b, 0.04, 0.002), CROWN))
    meshes.append(kit.box((0, 0, h - d + 0.001), (2 * b, 2 * b, 0.002), DARK))  # the dark receptacle floor
    meshes.append(kit.box((w / 2 + 0.001, 0.025, 0.025), (0.002, 0.012, 0.02), kit.Look((0.85, 0.15, 0.12), 0.5)))
    meshes.append(kit.box((w / 2 + 0.001, -0.025, 0.025), (0.002, 0.012, 0.02), kit.Look((0.15, 0.35, 0.85), 0.5)))
    return kit.Prop("cooler", pos=COOLER_POS, size=(w, w, h), meshes=tuple(meshes),
                    colliders=tuple(kit.Collider(c, size=s) for c, s in parts))


OBJECTS = [_cooler(), _jug()]


# --- Geometry the scorer and the water model share.
def _rot(objects, name):
    return pk._rotm(objects[name].quat_xyzw)


def _neck_tip(objects):
    return np.asarray(objects["jug"].pos, float) + _rot(objects, "jug") @ np.array([0.0, 0.0, NECK_TOP])


def _in_receptacle(objects):
    """The neck points down and its tip is inside the receptacle opening, at least 5 mm below the crown."""
    tip = _neck_tip(objects)
    off = np.abs(tip[:2] - np.asarray(COOLER_XY))
    return (pk.tilt(objects, "jug") > 150.0 and bool(np.all(off < BORE - NECK_R + 0.002))
            and tip[2] < CROWN_Z - 0.005)


def _dock(objects):
    return ("cooler", SEAL_ML) if _in_receptacle(objects) else None


pk.setup([pk.Vessel("jug", sections=((BODY_R - WALL, BODY_TOP - BOTTOM - WALL), (SH1_R - WALL, SH1_TOP - BODY_TOP),
                                     (SH2_R - WALL, SH2_TOP - SH1_TOP), (NECK_R - 0.0015, NECK_TOP - SH2_TOP)),
                    floor=BOTTOM + WALL, fill={"water": WATER0}, max_flow=GLUG),
          pk.Vessel("cooler", sections=((RES_R, RES_H), (BORE, BORE_LEN)), floor=RES_FLOOR, mouth=BORE,
                    fill={"water": RES0})],
         liquids={"water": WATER_RGB}, docks={"jug": _dock}, limits={"cooler": SEAL_ML})
# Water a stream puts in the reservoir past the air-lock level counts as spill: the seated neck and shoulder push it
# up and over the crown.
pk.set_floor(TOP)


# --- Scoring
# Spill limit: 73 mL, 10 % of the bottle (1.1 L real), a puddle that needs a mop and a wet-floor sign. Turned
# neck-down in under a second right over the crown, the bottle loses about half that (the demo: 35 to 45 mL) while
# its neck swings past the crown's edge; turned over in 3 s, or held on its side for a second, it loses more, at
# 73 mL a second.
SPILL_MAX = 73.0
SEAT_TILT = 5.0  # degrees from straight down: past this the bottle is propped on its neck or the crown's edge, not
#                  sitting on its shoulder ring (5 degrees lifts one side of the 5 cm ring 4.4 mm off the crown)
SEAT_DEPTH = 0.015  # m: the neck's tip at least 1.5 of its 2 cm below the crown, so the shoulder ring is down on it
FELL_TILT = 45.0  # degrees: tipped this far with no hand on a handle and its neck not in the receptacle, it fell
FELL_STEPS = 3  # 0.15 s in a row, so a one-step physics pop does not count
# One hand: a 12.5 kg bottle turned neck-down by one handle twists out of the hand or swings its open neck sideways,
# and bottled-water suppliers' instructions say to flip it with both hands; turned past 90 degrees (neck below
# level) with only one gripper on a handle, outside the receptacle, for 3 steps in all scores 0 (irreversible).
# A gripper holds a handle when its fingers are stopped by it, 10 to 60 % open (the 2.5 cm handle stops the 9.4 cm
# opening at 18 to 34 % in the sim as it squeezes; an empty closed fist reads 0), with its tool point inside the
# handle's box grown by 1 cm.
HOLD_OPEN = (0.10, 0.6)
HOLD_PAD = 0.01
CLEAR = 0.01  # m: for 0.9 and 1.0 both tool points must be this far clear of the bottle's outline (let go)
HOME_TOL, HOME_STILL = 0.03, 0.002  # m; m per control step
GRIPPERS = ("left_gripper", "right_gripper")

T = tracekit.Trace()
_HOME = {"episode": None, "home": None, "last": None, "still": False}


def _local(objects, p):
    """Point `p` in the bottle's own frame."""
    return _rot(objects, "jug").T @ (np.asarray(p, float) - np.asarray(objects["jug"].pos, float))


def _in_box(p, center, half):
    return bool(np.all(np.abs(p - np.asarray(center)) <= np.asarray(half)))


def _hands_on(objects):
    """How many grippers hold a handle of the bottle."""
    half = np.asarray(LUG) / 2 + HOLD_PAD
    n = 0
    for g in GRIPPERS:
        if g not in objects or not HOLD_OPEN[0] <= kit.opening(objects, g) <= HOLD_OPEN[1]:
            continue
        p = _local(objects, objects[g].pos)
        n += any(_in_box(p, c, half) for c in _lugs_local())
    return n


def _clear(objects):
    """Both tool points at least 1 cm outside the bottle's outline (body, neck and handles)."""
    half = np.asarray(LUG) / 2 + CLEAR
    for g in GRIPPERS:
        if g not in objects:
            continue
        p = _local(objects, objects[g].pos)
        if math.hypot(p[0], p[1]) < BODY_R + CLEAR and BOTTOM - CLEAR < p[2] < NECK_TOP + CLEAR:
            return False
        if any(_in_box(p, c, half) for c in _lugs_local()):
            return False
    return True


def _seated(objects):
    """Neck-down within 5 degrees, neck tip 1.5 cm or more down in the receptacle, still."""
    tip = _neck_tip(objects)
    return (_in_receptacle(objects) and pk.tilt(objects, "jug") >= 180.0 - SEAT_TILT
            and tip[2] <= CROWN_Z - SEAT_DEPTH and objects["jug"].resting(max_speed=0.02, max_spin=1.0))


def _watch(objects):
    if not T.new_step(objects):
        return
    T.track("spilled", pk.spilled() > SPILL_MAX)  # irreversible: the water is on the floor
    T.track("fell", pk.tilt(objects, "jug") > FELL_TILT and _hands_on(objects) == 0 and not _in_receptacle(objects),
            steps=FELL_STEPS)  # irreversible: a full bottle that fell over or was dropped
    T.track("one_hand", pk.tilt(objects, "jug") > 90.0 and _hands_on(objects) == 1 and not _in_receptacle(objects))
    # irreversible: turned neck-down on one handle for 3 steps in all, so swapping hands every 2 steps does not dodge it
    if _HOME["episode"] != kit.episode():
        _HOME.update(episode=kit.episode(), home=None, last=None, still=False)
    now = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    if _HOME["home"] is None and len(now) == 2:
        _HOME["home"] = now
    last = _HOME["last"]
    _HOME["still"] = last is not None and all(g in last and np.linalg.norm(now[g] - last[g]) < HOME_STILL for g in now)
    _HOME["last"] = now


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still."""
    home = _HOME["home"]
    if home is None or not _HOME["still"]:
        return False
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - home[g]) < HOME_TOL for g in GRIPPERS)


def _job(objects):
    if T.ever("fell") or T.total_s("one_hand") >= FELL_STEPS * T.dt - 1e-9 or not _seated(objects):
        return 0.0
    if T.ever("spilled"):
        return 0.25
    if _hands_on(objects) or not _clear(objects):
        return 0.5
    return 1.0


def success(objects):
    """The bottle seated, dry floor, let go and, with REQUIRE_ARMS_HOME, both arms home and still."""
    _watch(objects)
    return _job(objects) == 1.0 and (_arms_home(objects) or not REQUIRE_ARMS_HOME)


def score(objects):
    """0 if the bottle ever fell: tipped past 45 degrees for 3 steps with no gripper on a handle and its neck not in
    the receptacle (irreversible). 0 if it was ever turned past 90 degrees with only one gripper on a handle, neck
    outside the receptacle, for 3 steps in all (irreversible). 0 unless it ends seated: neck-down within 5
    degrees, the neck's tip in the receptacle opening and at least 1.5 cm below the crown, still. Seated, 0.25 if more than 73 mL (10 % of the
    bottle) was ever spilled, counting water past the reservoir's 195 mL air-lock level (irreversible); 0.5 if a
    gripper still holds a handle or either tool point is within 1 cm of the bottle; 1.0 let go with both arms home
    and still, 0.9 with an arm not home. Possible scores: 0, 0.25, 0.5, 0.9, 1."""
    _watch(objects)
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


# --- Demo: both arms take a handle from the side (approach along y, fingers closing vertically), lift the bottle,
# carry it over the crown, turn it neck-down in 0.8 s by rolling both wrists together (joint6, -90 to +90
# degrees, about the handles' axis), lower its neck into the receptacle, let go and go home.
YAW, ROLL = math.pi / 2, math.pi / 2
GRIP_Y = LUG_Y + 0.005  # tool point 5 mm outboard of the handle's centre
OUT_Y = 0.12  # clear of the handle's end with the fingers open
AIM_DX = 0.004  # m: the held bottle trails the tool points about 4 mm toward the robot
FLIP_S = 0.8  # s for the half turn
UP = TOP + 0.195  # handles' height while carrying and turning: the bottle's base clears the 8.4 cm turning radius


def _w(side, d, pos, finger, label, pitch):
    s = 1.0 if side == "left" else -1.0
    x, y, z = pos
    return kit.Tilted(d, (x, s * y, z), finger, label, yaw=-s * YAW, roll=-s * ROLL, pitch=pitch)


def arm_plan(side, flip_s=FLIP_S, pause=None, aim=(0.0, 0.0)):
    """One arm's plan. `pause` = (degrees, seconds) stops the turn part-way; `aim` shifts the seat (m, x and y)."""
    jx, _, jz = JUG_POS
    cx, cy = COOLER_XY[0] + AIM_DX + aim[0], aim[1]
    seat = CROWN_Z + SH2_TOP + 0.002
    p0 = -math.pi / 2
    w = lambda d, pos, f, label, p=p0: _w(side, d, (pos[0], pos[1] + (1 if side == "left" else -1) * cy, pos[2]),  # noqa: E731
                                          f, label, p)
    plan = [_w(side, 2.0, (jx, OUT_Y, TOP + 0.15), FINGER_OPEN, "beside the handle", p0),
            _w(side, 1.5, (jx, OUT_Y, jz), FINGER_OPEN, "down", p0),
            _w(side, 1.5, (jx, GRIP_Y, jz), FINGER_OPEN, "onto the handle", p0),
            _w(side, 1.0, (jx, GRIP_Y, jz), FINGER_CLOSED, "grip", p0),
            _w(side, 2.0, (jx, GRIP_Y, UP), FINGER_CLOSED, "lift", p0),
            w(2.0, (cx, GRIP_Y, UP), FINGER_CLOSED, "over the crown")]
    n = 6
    for k in range(1, n + 1):
        p = p0 + math.pi * k / n
        plan.append(w(flip_s / n, (cx, GRIP_Y, UP), FINGER_CLOSED, "turn neck-down", p))
        if pause and abs(math.degrees(math.pi * k / n) - pause[0]) < 1:
            plan.append(w(pause[1], (cx, GRIP_Y, UP), FINGER_CLOSED, "pause", p))
    plan += [w(0.8, (cx, GRIP_Y, seat), FINGER_CLOSED, "neck into the receptacle", -p0),
             w(0.8, (cx, GRIP_Y, seat), FINGER_CLOSED, "settle", -p0),
             w(1.0, (cx, GRIP_Y, seat), FINGER_OPEN, "let go", -p0),
             w(1.2, (cx, OUT_Y, seat + 0.01), FINGER_OPEN, "out", -p0),
             w(1.5, (cx, OUT_Y, TOP + 0.20), FINGER_OPEN, "up", 0.0),
             kit.Home(2.5), kit.Home(1.5, label="hold at home")]
    return plan


def demo():
    """Both arms take the bottle by its handles, lift it, carry it over the cooler, turn it neck-down in 0.8 s
    and seat its neck in the receptacle, let go and go home."""
    return {"left": arm_plan("left"), "right": arm_plan("right")}
