"""Splitting a field sample of fine aggregate in a riffle splitter. A 12-chute splitter with 1/2 in (12.7 mm) chutes
stands on the table with its two catch pans slid under the chute outlets, one on each side of it; beyond it (farther
from the robot) a full-width steel pour pan holds 667 g of dry concrete sand, open along the side that faces the
splitter.

ASTM C702 (reducing samples of aggregate to testing size) has the sample poured from a pan as wide as the chute
assembly, along its full width, so it flows freely through the chutes; Iowa DOT IM 336 asks that it be spread across
the full width so approximately equal amounts flow through each chute. The chutes alternate between the two catch
pans, so a pour that favors one end of the row, a corner or a single spot feeds a few chutes and gives a biased half
whose gradation is not the stockpile's.

The sand is rifflekit's scored bulk model (rifflekit.py, on dosekit, pourkit and yamkit): from the pan's pose each
step it works out how high the sand stands over each 7.2 mm strip of the pan's lip, pours each strip by a weir law
(drifting toward the low end of a pan tipped sideways), and drops each stream into the chute under it (or onto the
table). The catch pans show one ridge of sand per chute.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (modeled props, gripper states, tilted demo grips; nothing on disk changes)
import dosekit as dk  # noqa: E402  (the sand's density and angle of repose)
import guardkit as gk  # noqa: E402  (grip and release checks, arms home)
import rifflekit as rk  # noqa: E402  (import last: its hook runs after the others')
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Split the sand sample in the pan into two halves with the riffle splitter."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still

# --- The splitter (x forward, y to the robot's left, from the midpoint between the arm bases; z above the table).
# 12 chutes 12.7 mm wide with 1.6 mm aluminum vanes: a 17.4 cm chute row along y. Chute k (k = 0 at -y) discharges
# toward -x (the back catch pan) when k is even and toward +x (the front pan) when it is odd.
SPLIT_X = 0.22
CHUTES, CHUTE_W, VANE = 12, 0.0127, 0.0016
ROW_W = CHUTES * CHUTE_W + (CHUTES + 1) * VANE
HOPPER_D = 0.060  # the hopper's mouth across x
HOPPER_TOP = 0.10  # above the table (a bench splitter's is about 20 cm; lowered so the arms reach over it)
HOPPER_BOT = 0.06
OUTLET_Z = 0.045
SHEET = 0.0015
CATCH_IN = (0.050, 0.190, 0.035)  # catch pans inside: x, y, wall height
CATCH_X = (SPLIT_X - 0.072, SPLIT_X + 0.072)  # back, front

# --- The pour pan: 17.2 cm wide inside (the chute row less 2.4 mm), 10 cm front to back, 4.5 cm walls on three sides
# and a 6 mm lip along the side it pours from (-x, toward the splitter). Its origin is the middle of its underside.
PAN_W, PAN_D, PAN_H = 0.172, 0.100, 0.045
FLOOR_T = 0.003
LIP_H = 0.006
# The end handles are flat bars 25 x 20 mm running out to y +-0.16 (a stock pan's are short tabs; these are lengthened
# so the two grippers stay 28 cm apart).
HANDLE_Y0, HANDLE_Y1 = PAN_W / 2 + SHEET, 0.160
HANDLE_X, HANDLE_Z, HANDLE_ZC = 0.025, 0.020, 0.030
GRIP_Y = 0.140
PAN_MASS = 0.97  # kg: 300 g of steel plus the 667 g sample (the sand's flow is modeled, its weight is in the pan)
PAN0 = (0.38, 0.0)
SAMPLE_G = 667.0  # g: two halves of at least 300 g each, ASTM C136's minimum for a fine aggregate sieve analysis
SAND = dk.MATERIALS["sand"]  # dry sand: 1.45 g/mL loose, 34 degrees angle of repose

STEEL = kit.Look((0.70, 0.71, 0.72), 0.35, metallic=1.0)
ALU = kit.Look((0.80, 0.81, 0.83), 0.4, metallic=1.0)
CHUTE_A = kit.Look((0.62, 0.64, 0.67), 0.5, metallic=1.0)
CHUTE_B = kit.Look((0.42, 0.44, 0.47), 0.5, metallic=1.0)
CATCH = kit.Look((0.55, 0.57, 0.60), 0.4, metallic=1.0)


def _boxes(parts, look):
    return [kit.box(c, s, look) for c, s in parts], [kit.Collider(c, size=s) for c, s in parts]


def _pan():
    z = FLOOR_T
    hy = (HANDLE_Y0 + HANDLE_Y1) / 2
    parts = [((0, 0, FLOOR_T / 2), (PAN_D + 2 * SHEET, PAN_W + 2 * SHEET, FLOOR_T)),
             ((PAN_D / 2 + SHEET / 2, 0, z + PAN_H / 2), (SHEET, PAN_W + 2 * SHEET, PAN_H)),
             ((-PAN_D / 2 - SHEET / 2, 0, z + LIP_H / 2), (SHEET, PAN_W + 2 * SHEET, LIP_H)),
             ((0, PAN_W / 2 + SHEET / 2, z + PAN_H / 2), (PAN_D, SHEET, PAN_H)),
             ((0, -PAN_W / 2 - SHEET / 2, z + PAN_H / 2), (PAN_D, SHEET, PAN_H)),
             ((0, hy, HANDLE_ZC), (HANDLE_X, HANDLE_Y1 - HANDLE_Y0, HANDLE_Z)),
             ((0, -hy, HANDLE_ZC), (HANDLE_X, HANDLE_Y1 - HANDLE_Y0, HANDLE_Z))]
    meshes, colliders = _boxes(parts, STEEL)
    return kit.Prop("pan", pos=(*PAN0, TOP), size=(PAN_D + 2 * SHEET, 2 * HANDLE_Y1, FLOOR_T + PAN_H), mass=PAN_MASS,
                    meshes=tuple(meshes), colliders=tuple(colliders))


def _splitter():
    """The splitter and its two catch pans, fixed, in the frame of the hopper's center at table level."""
    meshes, colliders = [], []
    hz = (HOPPER_BOT + HOPPER_TOP) / 2
    frame = [((s * (HOPPER_D / 2 + SHEET / 2), 0, hz), (SHEET, ROW_W, HOPPER_TOP - HOPPER_BOT)) for s in (1, -1)]
    frame += [((0, s * (ROW_W / 2 + SHEET), HOPPER_TOP / 2), (0.08, 2 * SHEET, HOPPER_TOP)) for s in (1, -1)]
    m, c = _boxes(frame, ALU)
    meshes += m
    colliders += c
    for s in (1, -1):  # the end plates' top flanges, sloping in over the end chutes (drawn only)
        meshes.append(kit.box((0, s * (ROW_W / 2 + rk.END_FLANGE / 2), HOPPER_TOP - 0.001), (HOPPER_D, rk.END_FLANGE, 0.002),
                              ALU))
    y0 = -ROW_W / 2
    for k in range(CHUTES + 1):  # the vanes, drawn only
        meshes.append(kit.box((0, y0 + VANE / 2 + k * (CHUTE_W + VANE), hz),
                              (HOPPER_D, VANE, HOPPER_TOP - HOPPER_BOT), ALU))
    for k in range(CHUTES):  # each chute's floor slopes from the hopper's far side down to its outlet
        a, b = y0 + VANE + k * (CHUTE_W + VANE), y0 + (k + 1) * (CHUTE_W + VANE)
        s = -1 if k % 2 == 0 else 1
        x0, x1 = -s * HOPPER_D / 2, s * (HOPPER_D / 2 + 0.012)
        pts = [(x0, a, HOPPER_BOT + 0.01), (x0, b, HOPPER_BOT + 0.01), (x1, b, OUTLET_Z), (x1, a, OUTLET_Z)]
        meshes.append(kit._mesh(pts, [4], [0, 1, 2, 3] if s > 0 else [0, 3, 2, 1], CHUTE_A if s > 0 else CHUTE_B))
    for cx in CATCH_X:
        dx, dy, h = CATCH_IN
        parts = [((cx - SPLIT_X, 0, FLOOR_T / 2), (dx + 2 * SHEET, dy + 2 * SHEET, FLOOR_T))]
        parts += [((cx - SPLIT_X + s * (dx / 2 + SHEET / 2), 0, h / 2), (SHEET, dy + 2 * SHEET, h)) for s in (1, -1)]
        parts += [((cx - SPLIT_X, s * (dy / 2 + SHEET / 2), h / 2), (dx, SHEET, h)) for s in (1, -1)]
        m, c = _boxes(parts, CATCH)
        meshes += m
        colliders += c
    return kit.Prop("splitter", pos=(SPLIT_X, 0.0, TOP), size=(0.25, 0.20, HOPPER_TOP), meshes=tuple(meshes),
                    colliders=tuple(colliders))


OBJECTS = [_splitter(), _pan()]

rk.setup(rk.Pan("pan", depth=PAN_D, width=PAN_W, floor=FLOOR_T, lip=LIP_H, wall=PAN_H, fill=SAMPLE_G,
                material="sand"),
         rk.Splitter(center=(SPLIT_X, 0.0), mouth=HOPPER_D, top=TOP + HOPPER_TOP, chutes=CHUTES, chute_w=CHUTE_W,
                     vane=VANE, catch=(rk.CatchPan("back", (CATCH_X[0], 0.0), CATCH_IN[:2], TOP + FLOOR_T),
                                       rk.CatchPan("front", (CATCH_X[1], 0.0), CATCH_IN[:2], TOP + FLOOR_T))),
         floor_z=TOP)

# --- Scoring
THROUGH_FULL = 0.97  # of the sample down the chutes: at most 20 g left in the pan or lost
THROUGH_PART = 0.90
THROUGH_ANY = 0.50
LOST_FULL = 0.02  # spilled on the table or dropped straight into a catch pan past the hopper: 13 g
LOST_PART = 0.05
EVEN = 0.35  # every chute within 35 % of the mean chute (Iowa IM 336: about equal amounts through each chute)
FED = 0.60  # every chute within 60 % of the mean: the pour reached the whole row, unevenly
HALF_FULL = (0.45, 0.55)  # each catch pan's share of the sample: 300 to 367 g, so each half meets C136's 300 g
HALF_PART = (0.40, 0.60)
REST_Z = 0.008  # m: the pan's underside within 8 mm of the table top
REST_TILT = 10.0  # degrees
PAN_PARTS = (gk.Block((0, 0, (FLOOR_T + PAN_H) / 2), (PAN_D + 2 * SHEET, PAN_W + 2 * SHEET, FLOOR_T + PAN_H)),
             gk.Block((0, (HANDLE_Y0 + HANDLE_Y1) / 2, HANDLE_ZC), (HANDLE_X, HANDLE_Y1 - HANDLE_Y0, HANDLE_Z)),
             gk.Block((0, -(HANDLE_Y0 + HANDLE_Y1) / 2, HANDLE_ZC), (HANDLE_X, HANDLE_Y1 - HANDLE_Y0, HANDLE_Z)))
HOME = gk.ArmsHome()
# fingers closing along x on the 25 mm handle bar: 0.13 to 0.35 open (on the bar they read 0.19 to 0.32 in the sim, the
# loaded pan prying them a little apart at full tip; an empty fist reads 0, an open hand 1)
GRIP_BAND = gk.band(HANDLE_X, over=0.08)
TWO_HANDS = 0.90  # of the grams that go down the chutes, the share poured with a hand gripping each handle


class _TwoHands(gk._PerStep):
    """Grams that went down the chutes during steps when each handle bar had its own gripper closed on it."""

    def _fresh(self):
        self.last, self.held = 0.0, 0.0

    def feed(self, objects) -> None:
        if not self._tick(objects):
            return
        thr = rk.through(objects)
        if thr > self.last and both_handles(objects):
            self.held += thr - self.last
        self.last = thr


def both_handles(objects) -> bool:
    a = gk.grip_on(objects, "pan", PAN_PARTS[1], GRIP_BAND)
    b = gk.grip_on(objects, "pan", PAN_PARTS[2], GRIP_BAND)
    return any(g != h for g in a for h in b)


HANDS = _TwoHands()


def _set_down(objects):
    """The pan back on the table, flat and still, let go (every tool point and pad tip 1 cm clear of it)."""
    o = objects["pan"]
    q = np.asarray(o.quat_xyzw, float)
    upright = 1 - 2 * (q[0] ** 2 + q[1] ** 2) > math.cos(math.radians(REST_TILT))
    return (float(o.pos[2]) - TOP < REST_Z and upright and o.resting()
            and gk.released_clear(objects, "pan", PAN_PARTS, 0.01))


def _job(objects):
    """The split's grade, before the arms-home check."""
    if not _set_down(objects):
        return 0.0
    m = rk.sample()
    ch = rk.chutes(objects)
    thr = float(ch.sum())
    if thr < THROUGH_ANY * m or HANDS.held < TWO_HANDS * thr:
        return 0.0
    lost = rk.spilled(objects) + rk.bypass(objects)
    halves = [g / m for g in rk.catch(objects).values()]
    spread = float(np.abs(ch / ch.mean() - 1.0).max())
    if (thr >= THROUGH_FULL * m and lost <= LOST_FULL * m and spread <= EVEN
            and all(HALF_FULL[0] <= h <= HALF_FULL[1] for h in halves)):
        return 1.0
    if (thr >= THROUGH_PART * m and lost <= LOST_PART * m and spread <= FED
            and all(HALF_PART[0] <= h <= HALF_PART[1] for h in halves)):
        return 0.5
    return 0.25


def _score(objects):
    HOME.feed(objects)
    HANDS.feed(objects)
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not HOME.home(objects):
        return 0.9
    return job


def success(objects):
    """A full score, both arms home included."""
    return _score(objects) == 1.0


def score(objects):
    """0 unless the pour pan ends back on the table (its underside within 8 mm of the table top, level within 10
    degrees, still) and let go (every tool point and pad tip 1 cm clear of it), and 0 unless at least 90 % of what
    went down the chutes was poured with each handle bar gripped by its own hand (fingers stopped by the 25 mm bar, tool
    point within 1 cm of it). Then by where the 667 g sample went: 0 if less than half of it went down the chutes;
    1.0 if at least 97 % did, at most 2 % (13 g) was spilled or fell past the hopper straight into a catch pan, every
    chute carried within 35 % of the mean chute's load and each catch pan holds 45 to 55 % of the sample, with both
    arms home and still (0.9 if not home); 0.5 if at least 90 % went down the chutes, at most 5 % was lost, every chute
    within 60 % of the mean and each catch pan 40 to 60 %; else 0.25.
    Possible scores: 0, 0.25, 0.5, 0.9, 1."""
    return _score(objects)


# --- The demo. Both arms take the pan's end handles in the side grip (each hand pointing inward along y, tipped 13
# degrees down, fingers closing along x), lift it over the splitter and tip it toward -x by turning both wrists about
# world y together, keeping the lip on a fixed line over the hopper's middle.
P = math.pi
SIDE_L = dict(yaw=-P / 2, roll=-1.35)
SIDE_R = dict(yaw=P / 2, roll=1.35)
TILT = math.radians(60.0)  # 26 degrees past the sand's 34-degree repose: all but about 1 % of the sample runs out
LIP_LOCAL = np.array([-PAN_D / 2 - SHEET, 0.0, FLOOR_T + LIP_H])  # the lip's top, pan frame (at y = 0)
HANDLE_LOCAL = np.array([0.0, 0.0, HANDLE_ZC])
LIP_AT = np.array([SPLIT_X + 0.005, 0.0, TOP + HOPPER_TOP + 0.022])  # where the lip is held while it pours
LIFT = TOP + HOPPER_TOP + 0.075  # the tool points' height while the pan is carried level
X0 = PAN0[0]
BACK = 0.012  # m: the set-down is aimed 12 mm past the pan's spot, as the loaded pan trails the hands by about that


def _ry(p):
    c, s = math.cos(p), math.sin(p)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _tcp_for_lip(p):
    """Tool point (at y = 0) that keeps the lip at LIP_AT with the pan pitched p (negative: lip down)."""
    return LIP_AT - _ry(p) @ (LIP_LOCAL - HANDLE_LOCAL)


def _both(d, xyz, f, label, pitch=0.0, out=0.0):
    x, _, z = (float(v) for v in xyz)
    return (kit.Tilted(d, (x, GRIP_Y + out, z), f, label, pitch=pitch, **SIDE_L),
            kit.Tilted(d, (x, -GRIP_Y - out, z), f, label, pitch=pitch, **SIDE_R))


def demo():
    """Both arms come in beside the handles, close on them, lift the pan 15 cm and carry it back over the splitter,
    lower the lip to 2.2 cm over the hopper's middle, tip the pan to 60 degrees in 12 steps over 6 s turning about the
    lip, hold 4 s while the last of the sand runs out, tip it back level, carry it forward to where it started, set it
    down, let go, back out sideways and go home."""
    z0 = TOP + HANDLE_ZC
    over0 = _tcp_for_lip(0.0)
    plan = [_both(2.5, (X0, 0, z0 + 0.05), FINGER_OPEN, "beside the handles", out=0.06),
            _both(1.0, (X0, 0, z0), FINGER_OPEN, "down beside the handles", out=0.06),
            _both(1.2, (X0, 0, z0), FINGER_OPEN, "onto the handles"),
            _both(0.8, (X0, 0, z0), FINGER_CLOSED, "grip both handles"),
            _both(1.5, (X0, 0, LIFT), FINGER_CLOSED, "lift level"),
            _both(2.0, (over0[0], 0, LIFT), FINGER_CLOSED, "carry over the splitter"),
            _both(1.0, over0, FINGER_CLOSED, "lip down over the hopper")]
    n = 12
    for k in range(1, n + 1):
        p = -TILT * k / n
        plan.append(_both(0.5, _tcp_for_lip(p), FINGER_CLOSED, "tip together", pitch=p))
    plan += [_both(4.0, _tcp_for_lip(-TILT), FINGER_CLOSED, "hold while it runs out", pitch=-TILT),
             _both(2.0, over0, FINGER_CLOSED, "tip back level"),
             _both(1.0, (over0[0], 0, LIFT), FINGER_CLOSED, "up off the hopper"),
             _both(2.0, (X0 + BACK, 0, LIFT), FINGER_CLOSED, "carry forward"),
             _both(1.5, (X0 + BACK, 0, z0 + 0.003), FINGER_CLOSED, "set down"),
             _both(0.8, (X0 + BACK, 0, z0 + 0.003), FINGER_OPEN, "let go"),
             _both(1.0, (X0 + BACK, 0, z0 + 0.003), FINGER_OPEN, "out sideways", out=0.06),
             _both(1.0, (X0 + BACK, 0, z0 + 0.05), FINGER_OPEN, "up", out=0.06)]
    left = [w[0] for w in plan] + [kit.Home(2.5), kit.Home(1.5, label="hold at home")]
    right = [w[1] for w in plan] + [kit.Home(2.5), kit.Home(1.5, label="hold at home")]
    return {"left": left, "right": right}
