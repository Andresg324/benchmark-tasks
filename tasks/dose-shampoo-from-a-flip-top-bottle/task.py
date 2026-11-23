"""On a bathroom counter (the station table) in front of the arms: a half-used 12.6 fl oz bottle of Suave Professionals
shampoo stands upright with its flip-top cap snapped shut, and a small dish stands beside it.

The job is a coin-sized dose of shampoo in the dish and the bottle shut again: the flip cap thumbed open, the bottle
tipped cap-down over the dish and squeezed, set back upright, and the cap snapped shut. The dose is 1.5 to 4.0 g (a
quarter-sized dollop), nothing dripped outside the dish, and the cap clicked shut, not left open or resting on its
catch.

The cap's lid is a hinged link with a snap detent: it is held shut until it is pushed past its catch, it then springs
to resting on the catch (11.5 degrees open), and its living hinge flips it closed or open either side of upright; the
open lid stops at 115 degrees. A hand moves it only by contact. The shampoo is squeezekit's flow model: the bottle's two
broad walls are sprung panels whose push (read from their joints) raises the pressure inside, and shampoo flows out of
the cap's orifice only while the lid is unclicked, the bottle is tipped cap-down and the pressure at the orifice
(squeeze plus the shampoo's own weight) exceeds the orifice's opening pressure. The shampoo's weight alone stays well
under it at any tilt, so the open bottle held cap-down does not drip until it is squeezed, and a light squeeze does not
open it: the hand sets the dose by how hard and how long it squeezes. While the lid is opened less than 61 degrees it
lies over the orifice and the shampoo runs onto it and off the cap; past that it falls plumb from the orifice into the
dish or outside it. Every threshold below appears once, with the same value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import sys
import warnings

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import gripkit  # noqa: E402
import squeezekit as sq  # noqa: E402
import yamgeom as yg  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Put a coin-sized dose of shampoo in the dish, then close the bottle."

# -- the bottle (its own frame: origin at the middle of its base, z up its axis, x along its 71 mm width, y across its
#    43 mm depth; the lid's hinge is on the +y side, its thumb tab on the -y side) ------------------------------------------
W, D = 0.0714, 0.0428  # m: 2.812 x 1.687 in (the listing's size; 8.375 in tall)
BASE_Z, BODY_Z, SHOULDER_Z, CAP_Z, LID_H = 0.030, 0.165, 0.180, 0.2007, 0.0120  # m: heights of the part tops
HEIGHT = CAP_Z + LID_H  # 212.7 mm overall with the cap shut
CAP_R = 0.017  # m: the flip cap, 34 mm across
SHOULDER = (0.050, 0.036)
PANEL = (0.056, 0.003, 0.110)  # each broad wall's sprung panel: 56 mm wide, 3 mm thick, 110 mm tall
PANEL_Z = 0.0975  # the panels' middle height (they run from 42.5 to 152.5 mm up)
PANEL_TRAVEL = 0.010
CORE_D = D - 2 * (PANEL[1] + PANEL_TRAVEL)
PANEL_SIGN = {"wall_l": -1.0, "wall_r": 1.0}  # inward push = sign * joint position (wall_l on +y, wall_r on -y)
K_WALL = 1500.0  # N/m per wall: 1.5 N per mm of push (a half-used bottle's walls; README: checking it)
AREA_ML_PER_CM = 15.0  # mL displaced per cm of one wall's push (a finger pad dents the wall over about 15 cm^2)
CAPACITY_ML = 400.0  # mL brim-full (12.6 fl oz = 373 mL of shampoo when new, plus the headspace)
FILL_ML = 200.0  # mL of shampoo at the start of each trial (about half used)
DENSITY = 1.03  # g/mL: shampoo
BOTTLE_G = 28.0  # g: the empty bottle and cap
LID_KG, PANEL_KG = 0.004, 0.003
BOTTLE_KG = (BOTTLE_G + FILL_ML * DENSITY) / 1000 - LID_KG - 2 * PANEL_KG
CROSS_CM2 = 22.0  # cm^2: the body's inside cross-section (an oval about 6.7 x 3.9 cm)
LIQUID_TOP = FILL_ML / CROSS_CM2 / 100.0  # m: the shampoo stands 9.1 cm deep in the upright bottle
BOTTLE_COM = (0.0, 0.0, (FILL_ML * DENSITY * LIQUID_TOP / 2 + BOTTLE_G * 0.105) / (FILL_ML * DENSITY + BOTTLE_G))
ORIFICE = (0.0, 0.0, CAP_Z)  # the cap's orifice (4 mm across), at the top of the cap's base under the lid
FLOW = 0.5  # mL/s per kPa: the orifice's conductance (see the README: Poiseuille at a shear-thinned viscosity)
SETTLE_S = 2.0  # s: tipped cap-down, the shampoo takes this long to run down into the cap
DOWN_DEG = 70.0  # the orifice must point within 70 degrees of straight down for shampoo to reach it
VALVE_KPA = 3.06  # kPa: the orifice opens only above this pressure (12.3 inches of water, the threshold opening
# pressure of the shampoo bottle's dispensing valve in US patent 4,728,006; a model choice for this cap). The shampoo's
# own weight gives at most 1.16 kPa (straight down, 11.2 cm of shampoo), so held cap-down unsqueezed it never opens.
AIR_IN = 0.05  # mL/s per kPa of vacuum: air drawn back in through the closed orifice after a squeeze is let go
HINGE = (0.0, CAP_R, CAP_Z)  # the lid's hinge: across the cap's +y edge at the top of its base
LIP = 0.005  # m: the thumb tab sticks out 5 mm past the cap's side, opposite the hinge
LID_REACH = 2 * CAP_R + LIP  # 39 mm from the hinge to the tab's edge
ORIFICE_R = 0.002  # m: the orifice, 4 mm across, in the middle of the cap
# The lid's underside crosses the orifice's line (its far edge, 2 mm past the middle) until the lid stands open by
# acos((CAP_R + ORIFICE_R) / LID_REACH) = 61 degrees: below that the lid lies over the orifice and catches the stream.
LID_CLEAR_A = math.acos((CAP_R + ORIFICE_R) / LID_REACH)
BOTTLE_XY = (0.300, 0.050)
SPAWN_GAP = 0.0002
BOTTLE_START = (BOTTLE_XY[0], BOTTLE_XY[1], TOP + SPAWN_GAP)

# -- the snap detent of the lid (opening angle a = -joint position, rad; 0 = shut) -------------------------------------------
SHUT_A = 0.05  # rad (2.9 deg): within this the lid is clicked shut, held toward 0 ...
HOLD_K = 1.0  # N*m/rad: ... by this stiffness: it takes 0.05 N*m (1.3 N at the tab) to pull it past the catch
BEAD_A = 0.20  # rad (11.5 deg): past the catch the lid springs open to rest on it here ...
BEAD_K = 0.267  # N*m/rad: ... and pressing it shut takes 0.04 N*m (2.1 N at the lid's middle) to pass the catch
OVER_A = math.pi / 2  # the living hinge's over-center point: below it the lid falls toward the catch, above it open
HINGE_K = 0.01  # N*m/rad: the living hinge's spring
OPEN_A = math.radians(115.0)  # the open stop
LID_DAMP = 0.002  # N*m*s/rad


class SnapLid:
    """yamkit law for the flip lid's hinge: a conservative spring that depends only on the lid's angle, so contact
    alone moves the lid and it stays wherever its springs balance. Shut (under SHUT_A): pulled to 0. On the catch
    (SHUT_A to BEAD_A): pushed open to BEAD_A. Free (BEAD_A to OVER_A): the living hinge pulls it back to BEAD_A.
    Past OVER_A: the living hinge pushes it to the open stop."""

    def drive(self, key, q, qd, J, mem):
        a = -q
        if a < SHUT_A:
            goal, k = 0.0, HOLD_K
        elif a < BEAD_A:
            goal, k = BEAD_A, BEAD_K
        elif a < OVER_A:
            goal, k = BEAD_A, HINGE_K
        else:
            goal, k = OPEN_A, HINGE_K
        mem["open"] = a
        return -goal, k, 0.0, LID_DAMP


# -- what the cameras see of the bottle (visual meshes only: the colliders below are unchanged boxes and cylinders) ----------
BODYLOOK = kit.Look((0.88, 0.64, 0.69), 0.3)  # a rose-tinted bottle
LABEL = kit.Look((0.97, 0.94, 0.92), 0.6)  # a plain cream label on each broad wall (no text or logo drawn)
BAND = kit.Look((0.62, 0.28, 0.38), 0.5)  # a plain band across the label
CAPLOOK = kit.Look((0.90, 0.90, 0.88), 0.35)  # the cap's base
LIDLOOK = kit.Look((0.97, 0.97, 0.96), 0.3)  # the lid, a shade lighter so its seam shows
HOLE = kit.Look((0.25, 0.25, 0.25), 0.6)  # the orifice, seen when the lid is open
OVAL_N = 4.0  # the body's cross-section is a superellipse |x/a|^4 + |y/b|^4 = 1: a rounded oval within the 71 x 43 mm box
VIS_A, VIS_B = W / 2 - 0.0005, D / 2 - 0.001  # its half-axes (a hair inside the colliders' box, so the labels show)


def _oval_y(x: float, a: float = VIS_A, b: float = VIS_B) -> float:
    return b * max(0.0, 1.0 - abs(x / a) ** OVAL_N) ** (1.0 / OVAL_N)


def _loft(rings, look, m: int = 72):
    """A closed visual surface through superellipse rings (half-x, half-y, z, exponent), bottom to top."""
    pts, counts, idx = [], [], []
    for a, b, z, n in rings:
        for k in range(m):
            t = 2 * math.pi * k / m
            c, s = math.cos(t), math.sin(t)
            pts.append((a * math.copysign(abs(c) ** (2 / n), c), b * math.copysign(abs(s) ** (2 / n), s), z))
    for i in range(len(rings) - 1):
        for k in range(m):
            p, q = i * m + k, i * m + (k + 1) % m
            counts.append(4)
            idx += [p, q, q + m, p + m]
    counts += [m, m]
    idx += list(range(m))[::-1] + list(range((len(rings) - 1) * m, len(rings) * m))
    return kit._mesh(pts, counts, idx, look)


def _patch(side: float, half_w: float, z0: float, z1: float, out: float, look, nx: int = 24):
    """A label patch (visual) lying on the oval broad wall on side `side` (+1: +y), in its wall link's frame: a thin
    curved slab from `out` - 1.5 mm to `out` outside the drawn oval, over |x| <= half_w and z0..z1 (link frame)."""
    ya = side * (D / 2 - PANEL[1] / 2)
    xs = [-half_w + 2 * half_w * i / nx for i in range(nx + 1)]
    pts = []
    for depth in (out, out - 0.0015):
        for z in (z0, z1):
            for x in xs:
                pts.append((x, side * (_oval_y(x) + depth) - ya, z))
    n1 = nx + 1
    counts, idx = [], []

    def quad(a, b, c, d):
        counts.append(4)
        idx.extend([a, b, c, d])

    for i in range(nx):
        quad(i, i + 1, n1 + i + 1, n1 + i)  # outer face
        quad(2 * n1 + i, 3 * n1 + i, 3 * n1 + i + 1, 2 * n1 + i + 1)  # inner face
        quad(i, 2 * n1 + i, 2 * n1 + i + 1, i + 1)  # bottom edge
        quad(n1 + i, n1 + i + 1, 3 * n1 + i + 1, 3 * n1 + i)  # top edge
    for e in (0, nx):
        quad(e, n1 + e, 3 * n1 + e, 2 * n1 + e)
    return kit._mesh(pts, counts, idx, look)


def _wall_looks(side: float):
    h = PANEL[2] / 2 - 0.005
    return (_patch(side, 0.026, -h, h, 0.0004, LABEL),
            _patch(side, 0.026, -0.010, 0.010, 0.0008, BAND))


def _wall(name: str, side: float):
    y = side * (D / 2 - PANEL[1] / 2)
    lo, hi = (-PANEL_TRAVEL, 0.0) if side > 0 else (0.0, PANEL_TRAVEL)
    return kit.Link(name, "prismatic", "Y", anchor=(0.0, y, PANEL_Z), lower=lo, upper=hi, start=0.0, mass=PANEL_KG,
                    meshes=_wall_looks(side), colliders=(kit.Collider((0, 0, 0), size=PANEL),),
                    damping=0.0, law=sq.Wall("bottle", -side, k=K_WALL, area_m2=AREA_ML_PER_CM * 1e-4))


# the lid in its link frame (origin on the hinge line, axes as the bottle's when shut)
LID_DISC = ((0.0, -CAP_R, LID_H / 2), CAP_R, LID_H)
LID_TAB = ((0.0, -2 * CAP_R - LIP / 2 + 0.0005, 0.0015), (0.012, LIP + 0.001, 0.003))
LID = kit.Link("lid", "revolute", "X", anchor=HINGE, lower=-OPEN_A, upper=0.0, start=0.0, mass=LID_KG,
               meshes=(kit.lathe([(0.0, 0.0), (CAP_R, 0.0), (CAP_R, LID_H - 0.003), (CAP_R - 0.002, LID_H),
                                  (0.0, LID_H)], LIDLOOK, center=(0.0, -CAP_R, 0.0), n=64),
                       kit.box(*LID_TAB, LIDLOOK)),
               colliders=(kit.Collider(LID_DISC[0], radius=CAP_R, height=LID_H),
                          kit.Collider(*LID_TAB)),
               damping=0.0, law=SnapLid())

_BASE_PARTS = (((0, 0, BASE_Z / 2), (W, D, BASE_Z)),
               ((0, 0, (BASE_Z + BODY_Z) / 2), (W, CORE_D, BODY_Z - BASE_Z)),
               ((0, 0, (PANEL_Z + PANEL[2] / 2 + BODY_Z) / 2), (W, D, BODY_Z - PANEL_Z - PANEL[2] / 2)),
               ((0, 0, (BODY_Z + SHOULDER_Z) / 2), (SHOULDER[0], SHOULDER[1], SHOULDER_Z - BODY_Z)))
BOTTLE = sq.free_body(kit.Mechanism(
    "bottle", pos=BOTTLE_START, yaw=0.0, mass=BOTTLE_KG, size=(W, D, HEIGHT),
    meshes=(_loft([(VIS_A - 0.003, VIS_B - 0.003, 0.0, OVAL_N), (VIS_A - 0.0008, VIS_B - 0.0008, 0.002, OVAL_N),
                   (VIS_A, VIS_B, 0.006, OVAL_N), (VIS_A, VIS_B, BODY_Z - 0.004, OVAL_N),
                   (VIS_A - 0.003, VIS_B - 0.002, BODY_Z + 0.002, OVAL_N), (0.026, 0.019, 0.173, 3.0),
                   (CAP_R + 0.002, CAP_R + 0.001, SHOULDER_Z, 2.0)], BODYLOOK),
            kit.lathe([(CAP_R + 0.0008, SHOULDER_Z - 0.001), (CAP_R + 0.0008, CAP_Z - 0.001), (CAP_R, CAP_Z),
                       (0.0, CAP_Z)], CAPLOOK, n=64),
            kit.cylinder((0.0, 0.0, CAP_Z + 0.0002), ORIFICE_R + 0.0005, 0.0004, HOLE, n=24)),
    colliders=tuple(kit.Collider(c, size=s) for c, s in _BASE_PARTS)
    + (kit.Collider((0, 0, (SHOULDER_Z + CAP_Z) / 2), radius=CAP_R, height=CAP_Z - SHOULDER_Z),),
    links=(_wall("wall_l", 1.0), _wall("wall_r", -1.0), LID)), com=BOTTLE_COM)

# -- the dish: a 3.5 in round porcelain dip bowl, 89 mm across, 25 mm tall, 110 g -------------------------------------------
DISH_R, DISH_H, DISH_WALL, DISH_FLOOR = 0.0445, 0.025, 0.004, 0.005
DISH_IN_R = DISH_R - DISH_WALL  # 40.5 mm: shampoo falling inside this lands in the dish
DISH_XY = (0.300, -0.050)
DISH_START = (DISH_XY[0], DISH_XY[1], TOP + SPAWN_GAP)
PORCELAIN = kit.Look((0.97, 0.97, 0.98), 0.25)
_N_WALL = 16
_SEG = 2 * DISH_R * math.tan(math.pi / _N_WALL) + 0.001


def _dish_wall():
    out = []
    for i in range(_N_WALL):
        t = 2 * math.pi * i / _N_WALL
        r = DISH_R - DISH_WALL / 2
        out.append(((r * math.cos(t), r * math.sin(t), DISH_H / 2), t))
    return out


_BOX_FACES = ((0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7), (0, 1, 5), (0, 5, 4), (1, 2, 6), (1, 6, 5), (2, 3, 7),
              (2, 7, 6), (3, 0, 4), (3, 4, 7))


def _yawed_box(center, yaw, size):
    """A convex-hull box collider turned by `yaw` about z (yamkit's box colliders are axis-aligned)."""
    c, s = math.cos(yaw), math.sin(yaw)
    hx, hy, hz = (v / 2 for v in size)
    pts = []
    for dz in (-hz, hz):
        for dx, dy in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            pts.append((c * dx - s * dy, s * dx + c * dy, dz))
    return kit.Collider(tuple(center), hull=tuple(pts), faces=_BOX_FACES)


DISH = kit.Prop(
    "dish", pos=DISH_START, mass=0.110, size=(2 * DISH_R, 2 * DISH_R, DISH_H),
    meshes=(kit.lathe([(0.0, 0.0), (DISH_R - 0.004, 0.0), (DISH_R, 0.004), (DISH_R, DISH_H),
                       (DISH_IN_R, DISH_H), (DISH_IN_R, DISH_FLOOR + 0.003), (DISH_IN_R - 0.003, DISH_FLOOR),
                       (0.0, DISH_FLOOR)], PORCELAIN, n=64),),
    colliders=(kit.Collider((0, 0, DISH_FLOOR / 2), radius=DISH_R, height=DISH_FLOOR),)
    + tuple(_yawed_box(c, t, (DISH_WALL, _SEG, DISH_H)) for c, t in _dish_wall()))

OBJECTS = [BOTTLE, DISH]
gripkit.use_rubber_pads()  # the YAM's finger pads are rubber-faced

# -- the shampoo model and the scored thresholds -------------------------------------------------------------------------
FLOWMODEL = sq.Bottle("bottle", tip=ORIFICE, axis=(0.0, 0.0, 1.0), panels=tuple(PANEL_SIGN.items()),
                      capacity=CAPACITY_ML, fill=FILL_ML, area=AREA_ML_PER_CM, cross=CROSS_CM2, neck=2.5, neck_ml=8.0,
                      p_yield=VALVE_KPA, flow=FLOW, air_in=AIR_IN, down_deg=DOWN_DEG, settle=SETTLE_S,
                      start_settled=False, glug=False)
DOSE_G = (1.5, 4.0)  # g in the dish: a coin-sized dose (a quarter-sized dollop, about 24 mm across)
LOW_G = 0.5  # g: under this the dish holds no real dose
HIGH_G = 15.0  # g: over this (half again the 10 mL, two-teaspoon general guide) the dose is dumped
OFF_G = 0.3  # g: at most this much outside the dish (drips on the counter, the dish's rim or the bottle's outside)
DISH_SPILL_DEG = 45.0  # the dish tipped past this with shampoo in it: its shampoo is spilled (counted outside)
UPRIGHT_DEG = 15.0  # at the end the bottle and the dish stand within 15 degrees of upright ...
STAND_GAP = 0.005  # ... resting on the counter (their bases within 5 mm of it)
CLEAR = 0.005  # m: and every arm link at least 5 mm from the bottle (lid included) and the dish
STILL_STEP = 0.0005  # m per control step (1 cm/s)
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}  # "left_robot/link3" -> (origin, rotation) this control step (env frame)


def rotm(q) -> np.ndarray:
    return sq.rotm(q)


def tilt_deg(q) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rotm(q)[2, 2])))))


def _joint(objects, key: str) -> float:
    """A bottle joint's position. A joint that cannot be read stops the trial: a lid read as shut, or walls read as
    unpushed, would otherwise score every trial 0 without a word."""
    try:
        return float(kit.joint(objects, key).pos)
    except KeyError as e:
        sys.stderr.write(f"shampoo: no joint state for {key}\n")
        raise RuntimeError(f"shampoo: no joint state for {key} (the bottle's lid and walls are scored from them)") from e


def lid_open(objects) -> float:
    """The lid's opening angle (rad, 0 = shut), read from its hinge joint."""
    return -_joint(objects, "bottle/lid")


def clicked(objects) -> bool:
    return lid_open(objects) < SHUT_A


def pushes(objects) -> list:
    """Each broad wall's inward push (m), read from its joint: what the hand did to the bottle."""
    return [sign * _joint(objects, f"bottle/{link}") for link, sign in PANEL_SIGN.items()]


class Shampoo(sq.Sauce):
    """squeezekit's sauce with the cap's lid as a seal and the dish as a moving catch. With the lid clicked shut
    nothing leaves the bottle and no air enters: a squeeze only raises the pressure inside. With it unclicked the flow
    is squeezekit's, but while the lid is opened less than LID_CLEAR_A it lies across the orifice's line and the
    shampoo runs onto the lid and the cap and drips off them: all of it is outside the dish. Past that, shampoo falls
    plumb from the orifice; inside the dish's inner wall (in the dish's own frame, above its floor, the dish upright)
    it is in the dish, anywhere else it is outside."""

    def __init__(self):
        super().__init__(FLOWMODEL)
        self.dish_ml = 0.0
        self.on_lid = 0.0  # mL that ran onto the lid and cap (part of `off`)
        self.dish = (np.array(DISH_START, float), np.eye(3))
        self.blocked = False

    def step(self, pos, quat_xyzw, deflections, dt=DT, sub=10, sealed=False, blocked=False):
        self.blocked = blocked
        if not sealed:
            return super().step(pos, quat_xyzw, deflections, dt, sub)
        self.last_tip = self.tip_world(pos, quat_xyzw)
        c = self.cos_down(quat_xyzw)
        down = c >= math.cos(math.radians(self.b.down_deg))
        self.t_down = self.t_down + dt if down else 0.0
        self.squeeze_ml = self.displaced(deflections)
        self.p = sq.P_ATM * (self.air / self.v_air(self.squeeze_ml) - 1.0)
        self.q_now = 0.0
        return 0.0

    def _land(self, x, y, z, ml) -> None:
        if self.blocked:  # onto the lid lying over the orifice: smeared on the cap and dripped off it
            self.on_lid += ml
            self.off += ml
            return
        dp, dr = self.dish
        local = dr.T @ (np.array([x, y, z]) - dp)
        if dr[2, 2] >= math.cos(math.radians(DISH_SPILL_DEG)) and math.hypot(local[0], local[1]) <= DISH_IN_R \
                and local[2] >= DISH_FLOOR:
            self.dish_ml += ml
        else:
            self.off += ml

    def spill_dish(self) -> None:
        self.off += self.dish_ml
        self.dish_ml = 0.0

    def dish_g(self) -> float:
        return self.dish_ml * DENSITY

    def off_g(self) -> float:
        return self.off * DENSITY

    def balance(self) -> float:
        return self.dispensed - self.dish_ml - self.off


# -- the outlines the arms are checked against -----------------------------------------------------------------------------
def _box_gap(q, center, size):
    d = np.abs(q - np.asarray(center, float)) - np.asarray(size, float) / 2
    return np.linalg.norm(np.maximum(d, 0.0), axis=1)


def _cyl_gap(q, center, r, h):
    d = q - np.asarray(center, float)
    radial = np.hypot(d[:, 0], d[:, 1])
    return np.hypot(np.maximum(radial - r, 0.0), np.maximum(np.abs(d[:, 2]) - h / 2, 0.0))


def bottle_gap(qb):
    g = np.minimum(_box_gap(qb, (0, 0, BODY_Z / 2), (W, D, BODY_Z)), _box_gap(qb, *_BASE_PARTS[3]))
    return np.minimum(g, _cyl_gap(qb, (0, 0, (SHOULDER_Z + CAP_Z) / 2), CAP_R, CAP_Z - SHOULDER_Z))


def lid_gap(ql):
    return np.minimum(_cyl_gap(ql, LID_DISC[0], CAP_R, LID_H), _box_gap(ql, *LID_TAB))


def dish_gap(qd):
    return _cyl_gap(qd, (0, 0, DISH_H / 2), DISH_R, DISH_H)


def _parts(objects):
    b, d = objects["bottle"], objects["dish"]
    out = {"bottle": (np.asarray(b.pos, float), rotm(b.quat_xyzw), bottle_gap, 0.22),
           "dish": (np.asarray(d.pos, float), rotm(d.quat_xyzw), dish_gap, 0.05)}
    lid = objects.get("bottle/lid")
    if lid is not None:
        out["lid"] = (np.asarray(lid.pos, float), rotm(lid.quat_xyzw), lid_gap, 0.05)
    return out


_GAP_MEMO: dict = {}


def link_gap(parts, key, part) -> float:
    """Smallest gap (m) between arm link `key` ("right_robot/tip_left") as posed and `part`; 0.02 for a link more than
    2 cm away (per-hull, from the robot's own collision hulls)."""
    o, R = LINK_POSES[key]
    co, Rc, gap, reach = parts[part]
    memo = (key, part, np.asarray(o).tobytes(), np.asarray(R).tobytes(), co.tobytes(), Rc.tobytes())
    if memo in _GAP_MEMO:
        return _GAP_MEMO[memo]
    link = key.split("/")[1]
    _, cen, rad = yg.cloud(link)
    o_, R_ = np.asarray(o, float), np.asarray(R, float)
    centre = (o_ + R_ @ cen - co) @ Rc
    if float(np.linalg.norm(centre)) - rad > reach + 0.02:
        out = 0.02
    else:
        q = yg.near(link, lambda x: (o_ + x @ R_.T - co) @ Rc, gap, 0.02)
        out = min(0.02, float(gap(q).min())) if len(q) else 0.02
    if len(_GAP_MEMO) > 2048:
        _GAP_MEMO.clear()
    _GAP_MEMO[memo] = out
    return out


def arm_gap(objects) -> float:
    """Smallest gap (m) between any arm link (link1 to the fingertips) and the bottle, its lid or the dish."""
    parts = _parts(objects)
    return min((link_gap(parts, k, p) for k in LINK_POSES for p in parts), default=0.02)


def standing(state, gap=STAND_GAP) -> bool:
    """Upright within 15 degrees and resting on the counter (its origin, the base's middle, within `gap` of it)."""
    return tilt_deg(state.quat_xyzw) <= UPRIGHT_DEG and float(state.pos[2]) - TOP <= gap


def dose_tier(dish_g: float, off_g: float) -> float:
    """The dose's cap on the score: 0 for no dose or a dumped one, 0.25 for the wrong amount or drips outside the dish,
    else 1.0 (the full score then depends on the finish)."""
    if dish_g < LOW_G - 1e-12 or dish_g > HIGH_G + 1e-12:
        return 0.0
    if not DOSE_G[0] - 1e-12 <= dish_g <= DOSE_G[1] + 1e-12 or off_g > OFF_G + 1e-12:
        return 0.25
    return 1.0


class Ledger:
    """One trial: the shampoo, the dish's spill, and the still/hold count."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.shampoo = Shampoo()
        self.finished = False
        self.last = None
        self.good = 0
        self.n = 0

    def step(self, objects) -> None:
        self.n += 1
        b, d = objects["bottle"], objects["dish"]
        bp, dp = np.asarray(b.pos, float), np.asarray(d.pos, float)
        s = self.shampoo
        s.dish = (dp, rotm(d.quat_xyzw))
        sq.register(s)
        shut = clicked(objects)
        s.step(b.pos, b.quat_xyzw, pushes(objects), dt=DT, sealed=shut, blocked=lid_open(objects) < LID_CLEAR_A)
        if tilt_deg(d.quat_xyzw) > DISH_SPILL_DEG and s.dish_ml > 0.0:
            s.spill_dish()
        still = self.last is not None and max(float(np.linalg.norm(bp - self.last[0])),
                                              float(np.linalg.norm(dp - self.last[1]))) <= STILL_STEP
        self.last = (bp, dp)
        self.finished = shut and standing(b) and standing(d) and arm_gap(objects) > CLEAR
        done = self.tier() == 0.5 and self.finished and still
        self.good = self.good + 1 if done else 0
        if _PROBE:
            with open(_PROBE, "a") as fh:
                fh.write("%d lid %.1f p %.2f disp %.2f dish %.2f off %.2f push %s tilt %.1f bottle %s dish %s fin %d "
                         "gap %.4f\n" % (self.n, math.degrees(lid_open(objects)), s.p, s.dispensed * DENSITY,
                                         s.dish_g(), s.off_g(), " ".join("%.4f" % v for v in pushes(objects)),
                                         tilt_deg(b.quat_xyzw), " ".join("%.4f" % v for v in bp),
                                         " ".join("%.4f" % v for v in dp), int(self.finished),
                                         arm_gap(objects)))

    def tier(self) -> float:
        t = dose_tier(self.shampoo.dish_g(), self.shampoo.off_g())
        return min(t, 0.5)

    def score(self) -> float:
        t = self.tier()
        return 1.0 if t == 0.5 and self.good >= HOLD_STEPS else t


_L = None
_PROBE = os.environ.get("SHAMPOO_PROBE")


def _read_links(base) -> None:
    """Every arm link's pose (env frame) at this control step, into LINK_POSES. A missing arm or link, or finger pads
    left without their rubber, stops the trial: the hands-clear check and the grip would otherwise pass silently."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"shampoo: no {robot} in the scene for the hands-clear check") from e
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"shampoo: {robot} has no link(s) {missing} for the hands-clear check")
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
    if gripkit._STATE.get("bound", 0) < 0:
        raise RuntimeError("shampoo: the finger pads' rubber material could not be bound")
    if not _READ_ONCE:
        _READ_ONCE.append(1)
        sys.stderr.write(f"shampoo: hands-clear check on {len(LINK_POSES)} arm links; pad shapes with rubber: "
                         f"{gripkit._STATE.get('bound')}\n")


_READ_ONCE: list = []
kit.STATE_HOOKS.append(_read_links)
if "isaaclab" in sys.modules:  # in the simulator: read and check the hulls as the task loads, not in a trial's step
    yg.warm(ARM_LINKS)


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
    _VIS.draw(_L.shampoo, objects)


def success(objects):
    """A coin-sized dose in the dish, nothing outside it, the cap clicked shut, the bottle and dish standing on the
    counter with every arm link clear of them, all still: this step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """1.0: 1.5 to 4.0 g of shampoo in the dish, at most 0.3 g outside it, the lid clicked shut, the bottle and the
    dish standing upright on the counter, every arm link at least 5 mm clear of them, all still (under 1 cm/s) for 1 s.
    0.5: the dose and no drips, but not that finish (the cap left open or resting on its catch, the bottle still held
    or not standing). 0.25: 0.5 to 1.5 g or 4.0 to 15 g in the dish, or a dose with more than 0.3 g outside it. 0:
    under 0.5 g or over 15 g in the dish, including doing nothing (the dish starts empty) and squeezing with the cap
    shut."""
    _update(objects)
    return _L.score()


# -- what the cameras see -----------------------------------------------------------------------------------------------------
class _Visuals:
    """The shampoo in the dish as a pink pool (its depth from the volume), the stream while it flows, and drips
    outside the dish as small drops where they landed, drawn with Isaac Lab's visualization markers. Only a picture: on
    any error it turns itself off and scoring carries on."""

    COLOR = (0.90, 0.62, 0.70)

    def __init__(self):
        self.markers, self.off, self.shown, self.drops, self.episode = None, False, None, [], None

    def draw(self, s: Shampoo, objects) -> None:
        if self.off:
            return
        if self.episode != kit.episode():
            self.episode, self.drops, self.seen_off = kit.episode(), [], 0.0
        tip = s.last_tip
        if s.off > self.seen_off + 1e-6 and tip is not None and len(self.drops) < 60:
            self.drops.append((float(tip[0]), float(tip[1]), TOP))
            self.seen_off = s.off
        tr, sc, ix = [], [], []
        dp, dr = s.dish
        if s.dish_ml > 1e-4:
            depth = s.dish_ml * 1e-6 / (math.pi * (DISH_IN_R - 0.006) ** 2)
            c = dp + dr @ np.array([0.0, 0.0, DISH_FLOOR + depth / 2])
            tr.append(c)
            sc.append((DISH_IN_R - 0.006, DISH_IN_R - 0.006, max(depth, 3e-4)))
            ix.append(0)
        for p in self.drops:
            tr.append((p[0], p[1], p[2] + 0.001))
            sc.append((0.006, 0.006, 0.002))
            ix.append(1)
        if s.q_now > 0.0 and tip is not None:
            z0 = float(dp[2]) + DISH_FLOOR if math.hypot(tip[0] - dp[0], tip[1] - dp[1]) <= DISH_IN_R else TOP
            if tip[2] > z0:
                tr.append((tip[0], tip[1], (tip[2] + z0) / 2))
                sc.append((0.003, 0.003, float(tip[2] - z0)))
                ix.append(1)
        sig = (len(tr), round(float(np.sum(sc)) if sc else 0.0, 7))
        if sig == self.shown:
            return
        try:
            if self.markers is None:
                import isaaclab.sim as sim_utils
                from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

                mat = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(sq._linear(x) for x in self.COLOR),
                                                  roughness=0.3)
                self.markers = VisualizationMarkers(VisualizationMarkersCfg(
                    prim_path="/Visuals/shampoo",
                    markers={"pool": sim_utils.CylinderCfg(radius=1.0, height=1.0, visual_material=mat),
                             "drop": sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=mat)}))
            self.shown = sig
            if not tr:
                self.markers.set_visibility(False)
                return
            self.markers.set_visibility(True)
            origin = np.asarray(_ORIGIN, np.float32)
            self.markers.visualize(translations=np.array(tr, np.float32) + origin,
                                   orientations=np.tile(np.array([0, 0, 0, 1], np.float32), (len(tr), 1)),
                                   scales=np.array(sc, np.float32), marker_indices=np.array(ix, int))
        except ImportError:
            self.off = True
        except Exception as e:  # noqa: BLE001 (cosmetic only)
            self.off = True
            warnings.warn(f"shampoo visuals off ({type(e).__name__}: {e}); scoring is unaffected", RuntimeWarning)
            print(f"shampoo: visuals off ({e!r})", file=sys.stderr, flush=True)


_VIS = _Visuals()
_ORIGIN = [0.0, 0.0, 0.0]


def _read_origin(base) -> None:
    _ORIGIN[:] = [float(v) for v in base.scene.env_origins[0].detach().cpu().numpy()]


kit.STATE_HOOKS.append(_read_origin)

# -- scripted demo: open loop from the fixed layout ----------------------------------------------------------------------------
# The left hand points forward and takes the standing bottle across its broad walls with a holding grip. The right hand
# points across the bottle (fingers closed, pointing +y), sets its fingertips' top edge under the lid's thumb tab and
# lifts it along the lid's arc, over upright, so the living hinge flips it open; the right hand then waits out of the
# way. The left lifts the bottle and rolls it 118 degrees about its forward axis, cap-down over the dish, waits for the
# shampoo to run into the cap, squeezes, eases back to the holding grip, rolls it upright, sets it back on its mark, lets
# go and goes home. The right hand, pointing forward with its fingers closed, pushes the open lid back over upright from
# behind (it falls onto its catch) and presses it down past the catch until it clicks, then goes home.
DEMO_VARIANT = os.environ.get("SHAMPOO_VARIANT", "")  # "": the full demo; see demo() for the cheats
GRIP_Z = 0.100  # m: the left's tool point this high up the bottle (the pads on the walls' middle, nearer the COM)
PAD_BACK = 0.012  # m: the pads' middle sits this far behind the tool point (guardkit's hand model)
TIP_PAST = 0.0128  # m: the closed fingertips reach this far past the tool point
TIP_HALF = 0.0064  # m: the closed fingertips' half-thickness across the closing axis
TIP_HALF_W = 0.0132  # m: the fingertips' half-width
HOLD_PUSH = 0.0035  # m a side: the holding grip (the bottle weighs 0.24 kg; a lighter grip let it swing 6 degrees)
DOSE_PUSH = 0.0090  # m a side: the dosing squeeze (5.5 mm past the holding grip)
DOSE_S = 2.5  # s held
WEAK_PUSH = 0.0055  # m a side: the "weak" variant's squeeze (2 mm past the holding grip: under the opening pressure)
WEAK_S = 5.0  # s held
DUMP_PUSH, DUMP_S = 0.0140, 10.0  # the "dump" variant: as hard as the walls go, for 10 s
WAIT_LONG_S = 45.0  # s cap-down unsqueezed in the "wait" variant
TAIL_S = 1.0  # s back at the holding grip before rolling upright
POUR_DEG = 118.0  # roll about the forward axis (the wrist's limit is 120)
LIFT_AIM = 0.002  # m: the right's fingertip edge aims this far into the lid while lifting it (the arm gives)
PRESS_AIM = 0.025  # m: and this far past the lid's shut top when pressing it (the arm gives: a 4 mm aim stalled at 6 deg)
FINGER_HALF_OPEN = 0.047
PUSH_DX = -0.016  # m: shutting the lid, the right's tool point this far behind the bottle's middle (x): its
# fingertips (3 mm behind to 13 mm ahead of the tool point) meet the lid, its pads stay 2 mm behind the lid's edge
Z_PUSH = 0.219  # m above the counter: the height the open lid is pushed back at (about 25 mm out along it)


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * max(0.0, half) / FINGER_HALF_OPEN


def bottle_grip(push: float) -> float:
    return finger_for(D / 2 - push)


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0, 0], [0, c, -s], [0, s, c]])


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1.0, 0], [-s, 0, c]])


_DOWN = np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])  # yam_arena's straight-down grip (yaw 0)
R_LEFT = _ry(-math.pi / 2) @ _DOWN  # pointing forward (+x), closing across y
R_ACROSS = _rx(math.pi / 2) @ _DOWN  # pointing +y, closing along z (the fingertips' edge runs along x)
R_FWD_V = _rx(math.pi / 2) @ R_LEFT  # pointing forward, closing along z (the fingertips' 26 mm width runs along y)

BX, BY = BOTTLE_XY
LEFT_AT = np.array([BX + PAD_BACK, BY, TOP + GRIP_Z])  # the left's tool point on the standing bottle
_GRIP_IN_BOTTLE = np.array([PAD_BACK, 0.0, GRIP_Z])


def pour_tcp() -> np.ndarray:
    """The left's tool point that puts the rolled bottle's orifice 6 cm over the dish's middle floor."""
    off = _rx(math.radians(POUR_DEG)) @ (np.asarray(ORIFICE) - _GRIP_IN_BOTTLE)
    want = np.array([DISH_XY[0], DISH_XY[1], TOP + DISH_FLOOR + 0.060])
    return want - off


def _hinge_world():
    return np.array([BX, BY + HINGE[1], TOP + HINGE[2]])


def lift_tcp(a_deg: float, r: float = 0.0365, aim: float = LIFT_AIM) -> np.ndarray:
    """The right's tool point (pointing +y) that puts its fingertips' top front edge on the lid's underside, `r` from
    the hinge, with the lid opened `a_deg`, pressed `aim` into it."""
    a = math.radians(a_deg)
    d = np.array([0.0, -math.cos(a), math.sin(a)])  # along the lid, hinge to tab
    n_in = np.array([0.0, -math.sin(a), -math.cos(a)])  # the lid's underside normal (away from the lid)
    push = aim if a_deg > 0 else -0.0005  # at 0: 0.5 mm under the tab, not yet pushing
    edge = _hinge_world() + r * d - push * n_in
    return edge - np.array([0.0, TIP_PAST, TIP_HALF])


def push_tcp(a_deg: float, z: float, aim: float = LIFT_AIM) -> np.ndarray:
    """The right's tool point (pointing forward, closing vertical) whose fingertips' -y face touches the open lid's
    outer face at height `z` (above the table), the lid opened `a_deg`, pressed `aim` into it."""
    a = math.radians(a_deg)
    h = _hinge_world()
    d = np.array([0.0, -math.cos(a), math.sin(a)])
    n_out = np.array([0.0, math.sin(a), math.cos(a)])
    s = (TOP + z - h[2] - LID_H * n_out[2]) / d[2]  # where the outer face crosses that height
    face = h + s * d + LID_H * n_out
    return np.array([BX + PUSH_DX, face[1] + TIP_HALF_W - aim, TOP + z])


def _w(duration, pos, finger, label, rot):
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label, rot=np.asarray(rot))


def demo():
    """The left holds the bottle and doses; the right opens and shuts the cap. SHAMPOO_VARIANT picks a cheat instead:
    "open" (the cap left open), "dump" (a long hard squeeze), "closed" (squeezed with the cap shut), "held" (the lid
    pushed back onto its catch and the hand left resting on it, never clicked), "nothing", "wait" (held open cap-down
    WAIT_LONG_S with no squeeze, then shut) and "weak" (a light squeeze, WEAK_PUSH for WEAK_S, then shut)."""
    v = DEMO_VARIANT
    if v == "nothing":
        return {"left": [kit.Home(2.0)], "right": [kit.Home(2.0)]}
    shut_f, wide = FINGER_CLOSED, finger_for(0.032)
    push, dose_s = {"dump": (DUMP_PUSH, DUMP_S), "weak": (WEAK_PUSH, WEAK_S)}.get(v, (DOSE_PUSH, DOSE_S))
    hold, dose = bottle_grip(HOLD_PUSH), bottle_grip(push)
    behind = LEFT_AT - np.array([0.085, 0.0, 0.0])
    up = LEFT_AT + np.array([0.0, 0.0, 0.090])
    down = LEFT_AT + np.array([0.0, 0.0, 0.0008])
    pour = pour_tcp()
    r_pour = _rx(math.radians(POUR_DEG)) @ R_LEFT
    park = np.array([0.25, -0.10, TOP + 0.25])  # the right waits here (both its hand rotations reach it)

    # the right opens the lid along its arc
    start = lift_tcp(0.0)
    opening = [_w(1.5, start - np.array([0.0, 0.030, 0.0]), shut_f, "in front of the cap's tab", R_ACROSS),
               _w(1.0, start, shut_f, "under the tab", R_ACROSS)]
    for a in (8, 20, 35, 50, 65, 80, 95, 105):
        opening.append(_w(0.6, lift_tcp(a), shut_f, f"lift the lid to {a} deg", R_ACROSS))
    opening += [_w(0.8, lift_tcp(105) + np.array([0.0, -0.040, 0.015]), shut_f, "back off", R_ACROSS),
                _w(1.5, park, shut_f, "out of the way", R_ACROSS)]
    if v == "closed":
        opening = []
    # the right pushes the open lid back over and presses it shut
    over = np.array([BX + PUSH_DX, BY + 0.075, TOP + 0.275])
    lip_side = np.array([BX + PUSH_DX, BY - 0.012, TOP + HEIGHT + TIP_HALF])  # over the lid's tab end (longest lever)
    closing = [_w(4.0, park, shut_f, "turn to point forward", R_FWD_V),
               _w(2.0, over, shut_f, "above, beyond the open lid", R_FWD_V),
               _w(1.5, push_tcp(115, Z_PUSH, -0.006), shut_f, "behind the lid", R_FWD_V)]
    for a in (105, 95, 85, 75, 60):
        closing.append(_w(0.7, push_tcp(a, Z_PUSH), shut_f, f"push it back to {a} deg", R_FWD_V))
    closing += [_w(1.0, push_tcp(60, Z_PUSH) + np.array([0.0, 0.0, 0.030]), shut_f, "up off it", R_FWD_V),
                _w(1.0, lip_side + np.array([0.0, 0.0, 0.025]), shut_f, "over the lid", R_FWD_V)]
    if v == "held":
        closing += [_w(1.0, lip_side + np.array([0.0, 0.0, 0.003]), shut_f, "onto it", R_FWD_V),
                    _w(8.0, lip_side + np.array([0.0, 0.0, 0.003]), shut_f, "rest on the lid", R_FWD_V)]
    else:
        closing += [_w(1.0, lip_side + np.array([0.0, 0.0, 0.010]), shut_f, "onto it", R_FWD_V),
                    _w(1.5, lip_side - np.array([0.0, 0.0, PRESS_AIM]), shut_f, "press it shut", R_FWD_V),
                    _w(1.0, lip_side - np.array([0.0, 0.0, PRESS_AIM]), shut_f, "click", R_FWD_V),
                    _w(1.0, lip_side + np.array([0.0, 0.0, 0.040]), shut_f, "lift off", R_FWD_V),
                    _w(2.0, park, shut_f, "away", R_FWD_V),
                    kit.Home(4.0)]
    if v == "open":
        closing = [kit.Home(4.0)]
    t_open = sum(w.duration for w in opening)

    left = [_w(3.0, behind, wide, "behind the bottle", R_LEFT),
            _w(1.5, LEFT_AT, wide, "around its walls", R_LEFT),
            _w(0.8, LEFT_AT, hold, "hold it", R_LEFT)]
    t_grip = sum(w.duration for w in left)
    left += [_w(t_open + 0.5 if opening else 1.0, LEFT_AT, hold, "hold it while the cap is opened", R_LEFT),
             _w(2.5, up, hold, "lift", R_LEFT),
             _w(2.5, pour, hold, "cap-down over the dish", r_pour),
             _w(SETTLE_S + 0.5, pour, hold, "let the shampoo run into the cap", r_pour)]
    if v == "wait":  # no squeeze: only the holding grip, cap-down
        left.append(_w(WAIT_LONG_S, pour, hold, "wait, no squeeze", r_pour))
    else:
        left += [_w(0.4, pour, dose, "squeeze", r_pour),
                 _w(dose_s, pour, dose, "hold the squeeze", r_pour),
                 _w(0.4, pour, hold, "ease off", r_pour),
                 _w(TAIL_S, pour, hold, "let it stop", r_pour)]
    left += [_w(2.0, up, hold, "upright", R_LEFT),
             _w(1.5, LEFT_AT + np.array([0.0, 0.0, 0.010]), hold, "over its mark", R_LEFT),
             _w(1.0, down, hold, "set it down", R_LEFT)]
    left += [_w(0.8, down, wide, "let go", R_LEFT),
             _w(1.5, behind, wide, "back out", R_LEFT),
             kit.Home(4.0)]
    t_clear = sum(w.duration for w in left) - 2.0  # the left is well on its way home (its arm out of the right's way)
    right = [_w(t_grip, park, shut_f, "wait for the grip", R_ACROSS)] + opening
    right += [_w(max(0.5, t_clear - t_grip - t_open), park, shut_f, "wait for the bottle", R_ACROSS)] + closing
    return {"left": left, "right": right}
