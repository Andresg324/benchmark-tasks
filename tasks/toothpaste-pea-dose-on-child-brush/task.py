"""On a bathroom counter (the station table), in front of the arms: a child's manual toothbrush lies on its back,
bristles up, on the robot's left, and a kids' fluoride toothpaste tube (a Crest Kid's 4.6 oz / 130 g tube, partly
used, its flip cap flipped open) lies on its side on the right.

The job is to get the brush ready for a 4-year-old: a pea-sized dab of toothpaste (0.25 g; 0.15 to 0.35 g scores full)
on the bristles, nothing off the bristles, the tube put down. The US guidance for ages 3 to 6 is no more than a
pea-sized amount, and over 38% of children that age use more (CDC MMWR 2019;68(4):87-90). Too much is the real failure.

The paste is squeezekit's flow model. The tube's body is four sprung wall panels round a rigid core: a hand dispenses
only by pushing the panels in (read from their joints), and the paste flows through the nozzle only while the
pressure inside beats its yield pressure, then keeps flowing briefly after the squeeze stops (the air inside
re-expands). So the dose is set by how far and how long the hand squeezes. The paste drops plumb from the nozzle and
lands on the bristle field, a 1 mm grid that moves with the brush (the brush's own frame), or off it (the counter, the
head, the handle). Every threshold below appears once, with the same value in task.json and the README.
"""
from __future__ import annotations

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import guardkit as gk  # noqa: E402
import squeezekit as sq  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 1200
INSTRUCTION = "Get the toothbrush ready for a 4-year-old."

# -- the toothbrush (its own frame: origin at the middle of the bristle field's top, x along the brush toward the
#    head's tip, z out of the bristles; lying on its back its frame matches the world's) -------------------------------
BRISTLE = (0.020, 0.009, 0.009)  # m: the bristle field, 20 x 9 mm, 9 mm tall (a child's brush; verify on the trial brush)
HEAD = (0.026, 0.012, 0.005)  # the head under the bristles: 26 x 12 x 5 mm
NECK = (0.030, 0.006, 0.005)
HANDLE = (0.095, 0.016, 0.012)  # a chunky child's handle: 95 x 16 x 12 mm
_HEAD_Z = -BRISTLE[2] - HEAD[2] / 2
_FLOOR = -BRISTLE[2] - HEAD[2]  # the brush's underside (lying on its back, all three rest on the counter)
BRUSH_PARTS_BOX = (((0.0, 0.0, -BRISTLE[2] / 2), BRISTLE),
                   ((0.0, 0.0, _HEAD_Z), HEAD),
                   ((-(HEAD[0] + NECK[0]) / 2, 0.0, _FLOOR + NECK[2] / 2), NECK),
                   ((-HEAD[0] / 2 - NECK[0] - HANDLE[0] / 2, 0.0, _FLOOR + HANDLE[2] / 2), HANDLE))
BRUSH_G = 14.0  # g
BRUSH_START = (0.320, 0.080, TOP - _FLOOR)  # the bristle top's middle: 14 mm above the counter, head away from the robot
# Where the demo holds the handle (its middle), the tool point 14 mm over the brush's underside: the fingertips stand
# 13 mm below the tool point and stop 1 mm over the counter (aimed at the handle's mid-height, the brush hung 8 mm low
# in the hand in the third probe).
BRUSH_GRIP_LOCAL = np.array([-0.100, 0.0, _FLOOR + 0.014])
BRUSH = kit.Prop(
    "brush", pos=BRUSH_START, mass=BRUSH_G / 1000, size=(0.151, HANDLE[1], -_FLOOR),
    meshes=(kit.box(*BRUSH_PARTS_BOX[0], kit.Look((0.95, 0.95, 0.97), 0.8)),
            *(kit.box(c, s, kit.Look((0.30, 0.65, 0.90), 0.5)) for c, s in BRUSH_PARTS_BOX[1:])),
    colliders=tuple(kit.Collider(c, size=s) for c, s in BRUSH_PARTS_BOX))

# -- the tube (its own frame: origin at the middle of the body, x along its axis from the nozzle (-x) to the crimp (+x),
#    z up while it lies on the counter) ----------------------------------------------------------------------------------
BODY_R = 0.0175  # m: the round part of the body, 35 mm across (the standard 35 mm laminate size; caliper the trial tube)
BODY_X = (-0.050, 0.050)  # 10 cm of round body
# The used end, rolled flat up to the paste (as the reset rolls it): a flat roll 55 mm wide (the 35 mm tube laid flat),
# 10 mm thick and 2.4 cm long, lying on the counter. It is the tube's wide, rigid foot: lying on its sprung bottom
# panel alone, the tube rocked over 40 degrees in the second probe.
CRIMP = ((0.062, 0.0, -0.018 + 0.005), (0.024, 0.055, 0.010))  # its underside level with the cap's (18 mm)
SHOULDER_R, SHOULDER_X = 0.010, (-0.058, -0.050)
# The flip cap: 36 mm across (a stand-up cap, a little wider than the body [M]) and 13 mm long. With the rolled foot
# it carries the lying tube 0.5 mm clear of its wall panels: in the fifth probe the bottom panel bore the tube's
# weight, bounced on the paste's pressure and pumped paste out of the open nozzle.
CAP_R, CAP_X = 0.018, (-0.071, -0.058)
LID_T = 0.005  # the flip lid: a 5 mm disc over the cap's face, hinged at one edge, with a thumb tab at the other
NOZZLE_D, NOZZLE_L = 0.0075, 0.009  # m: the orifice, 7.5 mm across (generic tubes 0.28-0.32 in, US 9567138), 9 mm long
TIP = (CAP_X[0], 0.0, 0.0)  # the nozzle's mouth at the cap's face, under the lid
AXIS_TO_TIP = (-1.0, 0.0, 0.0)
PANEL_X = 0.0  # the middle of each wall panel along the tube (the body's middle)
PANEL = (0.060, 0.003, 0.020)  # each panel: 6 cm along the tube, 3 mm thick, 2 cm wide
PANEL_TRAVEL = 0.010
_CORE_W = 2 * (BODY_R - PANEL[1] - PANEL_TRAVEL)
CORE = (BODY_X[1] - BODY_X[0], _CORE_W, _CORE_W)
PANEL_SIGN = {"panel_l": -1.0, "panel_r": 1.0, "panel_t": -1.0, "panel_b": 1.0}  # inward push = sign * joint pos
PASTE_ML = 46.0  # mL left: 60 g of paste, the middle of the 50-70 g reset fill (a 4.6 oz / 130 g tube, partly used)
AIR_ML = 8.0  # mL of air inside at rest (a laminate tube springs back and draws some in; calibrate, see the README)
TUBE_G = 12.0  # g: the empty laminate tube and cap
DENSITY = 1.30  # g/mL: a silica toothpaste (pastes run 1.3-1.6, silica low, chalk high); real trials weigh grams
PANEL_KG = 0.0015
TUBE_KG = (TUBE_G + PASTE_ML * DENSITY) / 1000 - 4 * PANEL_KG
K_WALL = 400.0  # N/m per panel: 0.4 N per mm of push (verify on the trial tube)
PANEL_AREA_M2 = PANEL[0] * PANEL[2]
AREA_ML_PER_CM = 1.0  # mL displaced per cm of one panel's push (calibrate, see the README)
TAU_Y_PA = 300.0  # Pa: the paste's yield stress (measured 100 Pa on a model paste, arXiv 1903.10888; ~500 Pa on a
# commercial-style formula, US 5202112): the middle of that range. The demo's dose is checked at 100, 300 and 500 Pa.


def p_yield_kpa(tau_pa: float = TAU_Y_PA) -> float:
    """Pressure (kPa) to push a yield-stress plug through the nozzle: 4 tau L / D."""
    return 4.0 * tau_pa * NOZZLE_L / NOZZLE_D / 1000.0


P_YIELD_KPA = p_yield_kpa()  # 1.44 kPa
FLOW = 0.10  # mL/s per kPa past the yield: a firm squeeze (5 kPa) runs a 7.5 mm ribbon about 1 cm/s (calibrate)
TUBE_START = (0.260, -0.150, TOP + CAP_R)  # lying on its side on cap and foot, nozzle toward the robot, hinge on top
LAMINATE = kit.Look((0.95, 0.85, 0.20), 0.5)
PASTE_RGB = (0.35, 0.75, 0.95)  # blue kids' gel


class _Wall(sq.Wall):
    """squeezekit's wall spring, with the panel's travel opened only once the spring acts (the first control step).
    The arena settles the scene before its first step with no spring yet: an open panel under the lying tube sags to
    its stop (10 mm, seen in the first probe) and throws the tube on the first step. (As in instill-one-eye-drop.)"""

    sets_limits = True

    def __init__(self, bottle, sign, k, area_m2, lo, hi):
        super().__init__(bottle, sign, k=k, area_m2=area_m2)
        object.__setattr__(self, "lim", (lo, hi))

    def drive(self, key, q, qd, J, mem):
        mem["limits"] = self.lim
        return super().drive(key, q, qd, J, mem)


def _panel(link: str, axis: str, side: float):
    off = side * (BODY_R - PANEL[1] / 2)
    anchor = (PANEL_X, off, 0.0) if axis == "Y" else (PANEL_X, 0.0, off)
    size = PANEL if axis == "Y" else (PANEL[0], PANEL[2], PANEL[1])
    lo, hi = (-PANEL_TRAVEL, 0.0) if side > 0 else (0.0, PANEL_TRAVEL)
    return kit.Link(link, "prismatic", axis, anchor=anchor, lower=-1e-4, upper=1e-4, start=0.0, mass=PANEL_KG,
                    meshes=(kit.box((0, 0, 0), size, LAMINATE),), colliders=(kit.Collider((0, 0, 0), size=size),),
                    damping=0.0, law=_Wall("tube", -side, k=K_WALL, area_m2=PANEL_AREA_M2, lo=lo, hi=hi))


# The flip lid: a hinge along the tube's y at the cap face's upper edge (lying on the counter the hinge is on top).
# Opening swings the lid's free edge out past the nozzle and up (joint position positive). A snap hinge: within
# LID_SHUT_RAD of shut it is pulled shut (0.03 N m at the edge of that zone, about 1 N on the thumb tab); past
# LID_SNAP_RAD it is pulled to its open rest; between, it swings free. The nozzle is open (paste can leave) once the
# lid is LID_OPEN_RAD off it. The trial starts with the lid at its open rest (see the README: why the cap starts open).
LID_SHUT_RAD, LID_SNAP_RAD, LID_REST_RAD, LID_MAX_RAD = 0.5, 1.1, 2.5, 2.7
LID_OPEN_RAD = 1.0
K_LID = 0.06  # N m/rad
TAB = ((-LID_T / 2, 0.0, -2 * CAP_R - 0.002), (0.003, 0.010, 0.004))  # the thumb tab, 4 mm proud at the free edge


class Snap:
    """The flip lid's snap hinge (a mechkit-style law; joint position q >= 0, opening = q)."""

    sets_limits = True  # held at its start until the law acts (the arena settles the scene before the first step)

    def drive(self, key, q, qd, J, mem):
        mem["limits"] = (0.0, LID_MAX_RAD)
        mem["open"] = q >= LID_OPEN_RAD
        if q <= LID_SHUT_RAD:
            return 0.0, K_LID, 0.0, 2e-4
        if q >= LID_SNAP_RAD:
            return LID_REST_RAD, K_LID, 0.0, 2e-4
        return q, 0.0, 0.0, 2e-4


_LID_LOOK = kit.Look((0.20, 0.45, 0.85), 0.5)
LID = kit.Link("lid", "revolute", "Y", anchor=(CAP_X[0], 0.0, CAP_R), lower=LID_REST_RAD - 1e-4,
               upper=LID_REST_RAD + 1e-4, start=LID_REST_RAD, mass=0.004,
               meshes=(kit.cylinder((-LID_T / 2, 0.0, -CAP_R), CAP_R, LID_T, _LID_LOOK, axis="x"),
                       kit.box(*TAB, _LID_LOOK)),
               colliders=(kit.Collider((-LID_T / 2, 0.0, -CAP_R), radius=CAP_R, height=LID_T, axis="X"),
                          kit.Collider(*TAB)),
               law=Snap())

TUBE = sq.free_body(kit.Mechanism(
    "tube", pos=TUBE_START, yaw=0.0, mass=TUBE_KG, size=(CRIMP[0][0] + CRIMP[1][0] / 2 - CAP_X[0] + 2 * CAP_R, 2 * BODY_R, 2 * BODY_R + 2 * CAP_R),
    meshes=(kit.box((PANEL_X, 0, 0), CORE, LAMINATE),
            kit.cylinder((0, 0, 0), BODY_R - 0.001, BODY_X[1] - BODY_X[0], LAMINATE, axis="x"),
            kit.box(*CRIMP, LAMINATE),
            kit.cylinder(((SHOULDER_X[0] + SHOULDER_X[1]) / 2, 0, 0), SHOULDER_R, SHOULDER_X[1] - SHOULDER_X[0],
                         LAMINATE, axis="x"),
            kit.cylinder(((CAP_X[0] + CAP_X[1]) / 2, 0, 0), CAP_R, CAP_X[1] - CAP_X[0],
                         kit.Look((0.20, 0.45, 0.85), 0.5), axis="x")),
    colliders=(kit.Collider((PANEL_X, 0, 0), size=CORE), kit.Collider(*CRIMP),
               kit.Collider(((SHOULDER_X[0] + SHOULDER_X[1]) / 2, 0, 0), radius=SHOULDER_R,
                            height=SHOULDER_X[1] - SHOULDER_X[0], axis="X"),
               kit.Collider(((CAP_X[0] + CAP_X[1]) / 2, 0, 0), radius=CAP_R, height=CAP_X[1] - CAP_X[0], axis="X")),
    links=(_panel("panel_l", "Y", 1.0), _panel("panel_r", "Y", -1.0),
           _panel("panel_t", "Z", 1.0), _panel("panel_b", "Z", -1.0), LID)),
    com=(0.0, 0.0, 0.0))
TUBE_PARTS = (gk.Seg((CAP_X[0] + CAP_R, 0, 0), (BODY_X[1], 0, 0), BODY_R), gk.Block(*CRIMP))

OBJECTS = [BRUSH, TUBE]

# -- the paste model (squeezekit) and the scored thresholds ---------------------------------------------------------------
# The paste does not drip from an open tube turned nozzle-down (its yield stress holds it), so its own weight is left
# out (neck 0, a huge cross-section); it is fed at the nozzle whatever way the nozzle points short of straight up.
PASTE = sq.Bottle("tube", tip=TIP, axis=AXIS_TO_TIP, panels=tuple(PANEL_SIGN.items()), capacity=PASTE_ML + AIR_ML,
                  fill=PASTE_ML, area=AREA_ML_PER_CM, cross=1e9, neck=0.0, neck_ml=1.0, p_yield=P_YIELD_KPA,
                  flow=FLOW, air_in=2.0, down_deg=90.0, settle=0.0)
import dataclasses  # noqa: E402

_SEALED = dataclasses.replace(PASTE, p_yield=1e9, air_in=0.0, down_deg=180.0)  # shut: no flow, no venting
BEAD_CM = 0.45  # cm: a cell of bristle field holds a 4.5 mm tall bead before it slumps (a pea dab stands 4-5 mm)
BRISTLES = sq.Face("bristles", center=(0.0, 0.0, 0.0), size=BRISTLE[:2], cell=0.001, rest=BEAD_CM)
LAND_DEG = 60.0  # paste lands on the bristles only while they face within 60 degrees of straight up
PEA_G = (0.15, 0.35)  # g on the bristles: a pea-sized dab (0.25 g) for ages 3-6 ...
SMEAR_G = 0.05  # ... at least this much (under 0.15 g) is a smear or rice grain, the under-3 dose ...
OVER_G = 0.70  # ... above 0.35 g is too much (a half load), above 0.70 g a full adult strip
OFF_G = 0.05  # g of paste off the bristles (counter, head, neck, handle) allowed for 0.75 and 1.0
UP_DEG = 45.0  # at the end the bristles face within 45 degrees of straight up ...
TUBE_DOWN_Z = TOP + 0.030  # ... and the tube lies on the counter (its middle below this), released
FLIP_DEG = 90.0  # the bristles turned past sideways with paste on them: the dab is counted as fallen off
STILL_STEP = 0.0005  # m per control step (1 cm/s)
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
GRIPPERS = ("left_gripper", "right_gripper")


def rotm(q) -> np.ndarray:
    return sq.rotm(q)


def bristles_up_deg(q) -> float:
    """Angle (degrees) between the bristles' facing (the brush's z) and straight up."""
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rotm(q)[2, 2])))))


class BrushPaste(sq.Sauce):
    """squeezekit's Sauce with its landing face carried by the brush: the nozzle's plumb line is cut with the bristle
    plane and the hit point is given in the brush's frame, so the dab stays where it landed on the bristles wherever
    the brush is held or laid afterwards. With the bristles more than LAND_DEG from facing up, nothing lands on them."""

    def __init__(self):
        super().__init__(PASTE, [BRISTLES])
        self.brush = (np.array(BRUSH_START, float), np.eye(3))

    def tip_world(self, pos, quat_xyzw) -> np.ndarray:
        t = super().tip_world(pos, quat_xyzw)
        bp, r = self.brush
        n = r[:, 2]
        if n[2] < math.cos(math.radians(LAND_DEG)):
            return np.array([1.0, 1.0, -1.0])  # far off the field: lands off
        s = float(n @ (t - bp)) / float(n[2])  # height of the nozzle above the bristle plane, plumb
        local = r.T @ (t - np.array([0.0, 0.0, s]) - bp)
        return np.array([local[0], local[1], s])

    def step(self, pos, quat_xyzw, deflections, dt: float = DT, sub: int = 10, open_: bool = True) -> float:
        """squeezekit's step; with the lid shut the nozzle is sealed (pressure builds, nothing leaves)."""
        if open_:
            return super().step(pos, quat_xyzw, deflections, dt, sub)
        b, self.b = self.b, _SEALED
        try:
            return super().step(pos, quat_xyzw, deflections, dt, sub)
        finally:
            self.b = b

    def on_g(self) -> float:
        return self.on("bristles") * DENSITY

    def off_g(self) -> float:
        return self.off_target() * DENSITY


def pushes(objects) -> list:
    """Each wall panel's inward push (m), read from its joint: what the hand did to the tube."""
    out = []
    for link, sign in PANEL_SIGN.items():
        try:
            out.append(sign * kit.joint(objects, f"tube/{link}").pos)
        except KeyError:  # no joint state (not in the sim): no squeeze
            out.append(0.0)
    return out


def lid_open(objects) -> bool:
    """The flip lid swung at least LID_OPEN_RAD off the nozzle (read from its hinge)."""
    try:
        return kit.joint(objects, "tube/lid").pos >= LID_OPEN_RAD
    except KeyError:  # no joint state (offline tests): treated as open
        return True


def tube_held(objects) -> bool:
    return bool(gk.touching(objects, "tube", TUBE_PARTS, grippers=GRIPPERS))


def dose_tier(on_g: float, off_g: float) -> float:
    """The paste part of the score from the grams on the bristles and off them."""
    if on_g > OVER_G + 1e-12 or on_g < SMEAR_G - 1e-12:
        return 0.0
    if not PEA_G[0] - 1e-12 <= on_g <= PEA_G[1] + 1e-12:
        return 0.25  # a smear (under 0.15 g) or too much (0.35 to 0.70 g)
    return 0.75 if off_g <= OFF_G + 1e-12 else 0.5


class Ledger:
    """One trial: the paste, the flip latch and the still/hold count."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.paste = BrushPaste()
        self.flipped = False
        self.last = None
        self.good = 0

    def step(self, objects) -> None:
        b, t = objects["brush"], objects["tube"]
        bp, bq = np.asarray(b.pos, float), np.asarray(b.quat_xyzw, float)
        self.paste.brush = (bp, rotm(bq))
        sq.register(self.paste)
        self.paste.step(t.pos, t.quat_xyzw, pushes(objects), dt=DT, open_=lid_open(objects))
        if _PROBE:
            self.n = getattr(self, "n", 0) + 1
            try:
                lid = kit.joint(objects, "tube/lid").pos
            except KeyError:
                lid = float("nan")
            with open(_PROBE, "a") as f:
                f.write("%d %.3f %s %.3f %.4f %.4f %.1f %s %s %s" % (
                    self.n, lid, " ".join("%.5f" % v for v in pushes(objects)), self.paste.p, self.paste.on_g(),
                    self.paste.off_g(), bristles_up_deg(bq), " ".join("%.4f" % v for v in t.pos),
                    " ".join("%.4f" % v for v in bp), int(tube_held(objects)))
                        + " Q " + " ".join("%.3f" % v for v in t.quat_xyzw)
                        + " L " + " ".join("%.4f" % v for v in objects["left_gripper"].pos)
                        + " R " + " ".join("%.4f" % v for v in objects["right_gripper"].pos)
                        + " P " + " ".join("%.5f" % kit.joint(objects, f"tube/{k}").pos for k in PANEL_SIGN) + "\n")
        up = bristles_up_deg(bq)
        if up > FLIP_DEG and self.paste.on_g() > 0.0:
            self.flipped = True
        tp = np.asarray(t.pos, float)
        still = self.last is not None and max(float(np.linalg.norm(bp - self.last[0])),
                                              float(np.linalg.norm(tp - self.last[1]))) <= STILL_STEP
        self.last = (bp, tp)
        done = (self.tier() == 0.75 and up <= UP_DEG and float(tp[2]) <= TUBE_DOWN_Z and not tube_held(objects)
                and still)
        self.good = self.good + 1 if done else 0

    def tier(self) -> float:
        t = dose_tier(self.paste.on_g(), self.paste.off_g())
        return min(t, 0.25) if self.flipped else t


_L = None
_PROBE = os.environ.get("TOOTH_PROBE")


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
    _TILES.draw(objects, _L)


def success(objects):
    """A pea-sized dab on the bristles, nothing off them, the bristles up, the tube down and released, all still: this
    step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """Grams on the bristles (mL x 1.30 g/mL): 0.15 to 0.35 g with at most 0.05 g off the bristles scores 0.75, with
    more off them 0.5; 0.05 to 0.15 g (a smear) or 0.35 to 0.70 g (too much) 0.25; under 0.05 g or over 0.70 g 0.
    1.0: the 0.75 state with the bristles within 45 degrees of facing up and the tube lying on the counter, released,
    both still for 1 s. Turning the bristles past sideways with paste on them caps the score at 0.25. Doing nothing
    scores 0."""
    _update(objects)
    s = _L.tier()
    if s == 0.75 and _L.good >= HOLD_STEPS:
        s = 1.0
    return s


# -- the paste the cameras see ---------------------------------------------------------------------------------------------
class _Tiles(sq.Tiles):
    """The dab drawn on the bristles as tiles in the brush's own frame (turned and carried with it every step), with
    Isaac Lab's VisualizationMarkers; the env origin is read once from the brush's prim. Only a picture: on any error
    it turns itself off and scoring carries on."""

    def __init__(self):
        super().__init__(color=PASTE_RGB, min_ml=2e-5)
        self.origin = None

    def draw(self, objects, ledger):  # noqa: D102  (overrides squeezekit's world-fixed draw)
        if self.off:
            return
        if self.origin is None:
            try:
                import omni.usd
                from pxr import UsdGeom

                stage, cache = omni.usd.get_context().get_stage(), UsdGeom.XformCache()
                self.origin = np.zeros(3)
                for prim in stage.Traverse():
                    if prim.GetName() == "brush":
                        w = np.array(cache.GetLocalToWorldTransform(prim).ExtractTranslation(), float)
                        self.origin = w - np.asarray(objects["brush"].pos, float)
                        break
            except Exception:  # noqa: BLE001 (no simulator: unit tests)
                self.off = True
                return
        g = ledger.paste.grids["bristles"]
        bp, r = ledger.paste.brush
        xs, ys = g.centers()
        ii, jj = np.nonzero(g.v > self.min_ml)
        tr, sc = [], []
        for i, j in zip(ii, jj):
            h = float(g.v[i, j]) / (g.f.cell * 100.0) ** 2 / 100.0  # m
            tr.append(bp + r @ np.array([xs[i], ys[j], h / 2]))
            sc.append((g.f.cell, g.f.cell, max(h, 2e-4)))
        sig = (len(tr), round(float(np.sum(sc)) if sc else 0.0, 7), tuple(np.round(bp, 4)),
               tuple(np.round(r[:, 2], 3)))
        if sig == self.shown:
            return
        try:
            if self.markers is None:
                if not tr:
                    return
                import isaaclab.sim as sim_utils
                from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

                proto = sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
                    diffuse_color=tuple(sq._linear(x) for x in self.color), roughness=0.5))
                self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path=self.path,
                                                                            markers={"paste": proto}))
            self.shown = sig
            if not tr:
                self.markers.set_visibility(False)
                return
            q = np.asarray(sq.quat(r), np.float32)
            self.markers.set_visibility(True)
            self.markers.visualize(translations=np.array(tr, np.float32) + np.asarray(self.origin, np.float32),
                                   orientations=np.tile(q, (len(tr), 1)), scales=np.array(sc, np.float32),
                                   marker_indices=np.zeros(len(tr), int))
        except Exception as e:  # noqa: BLE001 (cosmetic only)
            self.off = True
            print(f"toothpaste: tiles off ({e!r}); scoring is unaffected", file=sys.stderr, flush=True)


_TILES = _Tiles()

# -- scripted demo: open loop from the fixed layout --------------------------------------------------------------------------
# The left hand takes the brush by the handle from above (fingers across its 16 mm width), lifts it and holds it level,
# bristles up, over the middle of the counter. The right hand takes the tube across its body (a light holding grip on
# two wall panels), lifts it and turns it nozzle-down (pitched 80 degrees: 10 degrees off plumb) with the nozzle 4 mm
# over the middle of the bristles, squeezes 1.5 mm a side for 1.2 s, eases back to the holding grip, waits 1.5 s for
# the flow to die, lifts 3 cm and lays the tube back where it was. The left hand lays the brush back down.
HOLD_BRUSH_AT = (0.300, 0.020, TOP + 0.100)  # the bristle top's middle while the brush is held up
HOLD_YAW = -math.pi / 2  # the held brush turned so its handle points left (+y), the left hand clear of the right
NOZZLE_GAP = 0.004  # m: the nozzle's mouth over the bristle top while squeezing
PITCH = -math.radians(80.0)
HOLD_PUSH = 0.0004  # m a side: holds the tube (1.4 N a side, 1.0 kPa inside, under the 1.44 kPa yield)
DOSE_PUSH = 0.00175  # m a side: the dosing squeeze (about 0.25 g in the offline model at 300 Pa)
DOSE_S = 1.2  # s held
# Variants for checking the scorer in the simulator (TOOTH_VARIANT=much | smear): a firmer, longer squeeze (too much, a
# half load) and a short light one (a smear). The demo itself uses DOSE_PUSH for DOSE_S.
VARIANTS = {"much": (0.0025, 1.5), "smear": (0.0012, 0.6)}
TAIL_S = 1.5  # s for the flow to die after easing off
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open (guardkit's gripper model)
WIDE = 0.030
GRIP_YAW = 0.0  # fingers closing across world y (yaw 0 on the real arm, GLOBAL-FIXES A1)
CARRY_DZ = 0.080
SIDE_Y = -0.060  # m: the tube is turned nozzle-down here, 6 cm right of the held brush's head
PAD_TIP = 0.013  # m: the closed fingertips stand this far below the tool point (yam_arena PAD_TIP_PAST_TCP)


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * max(0.0, half) / FINGER_HALF_OPEN


K_FINGER = 5000.0  # N/m: the YAM finger's position servo (yam_arena GRIPPER stiffness), in series with the wall


def push_force(push: float) -> float:
    """N on one wall panel pushed in `push` m, both side panels pushed alike: its spring plus the air's pressure."""
    ml = 2 * AREA_ML_PER_CM * push * 100.0
    p = sq.P_ATM * (AIR_ML / max(1e-3, AIR_ML - ml) - 1.0)
    return K_WALL * push + p * 1000.0 * PANEL_AREA_M2


def tube_grip(push: float) -> float:
    """Gripper command that pushes each side panel `push` m in: the finger's servo is a spring in series with the
    panel, so it is commanded past the panel by the force it must make over its stiffness."""
    return finger_for(BODY_R - push - push_force(push) / K_FINGER)


def nozzle_dir(pitch: float) -> np.ndarray:
    """The nozzle's direction (world) when the grip is pitched by `pitch` about y from straight down."""
    c, s = math.cos(pitch), math.sin(pitch)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]]) @ np.asarray(AXIS_TO_TIP)


def dose_tcp(gap: float = NOZZLE_GAP) -> tuple:
    """Tool point that puts the nozzle's mouth `gap` m over the middle of the held brush's bristles."""
    mouth = np.array([HOLD_BRUSH_AT[0], HOLD_BRUSH_AT[1], HOLD_BRUSH_AT[2] + gap])
    return tuple(float(v) for v in mouth - (GRASP_X - TIP[0]) * nozzle_dir(PITCH))


def _tw(duration, pos, finger, label, pitch=0.0, yaw=None):
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label,
                      yaw=GRIP_YAW if yaw is None else float(yaw),
                      roll=0.0, pitch=float(pitch))


GRASP_X = 0.015  # m: the right hand holds the body 1.5 cm behind its middle (toward the crimp), on the wall panels
def demo():
    """Left hand holds the brush up, level; right hand doses it; both put things back."""
    tx, ty, tz = TUBE_START
    gx = tx + GRASP_X
    push, dose_s = VARIANTS.get(os.environ.get("TOOTH_VARIANT", ""), (DOSE_PUSH, DOSE_S))
    hold, dose = tube_grip(HOLD_PUSH), tube_grip(push)
    over, at = dose_tcp(0.040), dose_tcp()
    lifted = (gx, ty, tz + CARRY_DZ)
    right = [_tw(1.5, (gx, ty, tz + 0.08), finger_for(WIDE), "above the tube"),
             _tw(1.5, (gx, ty, tz), finger_for(WIDE), "around the tube"),
             _tw(0.8, (gx, ty, tz), hold, "hold it"),
             _tw(1.5, lifted, hold, "lift")]
    left = []
    bx, by, bz = (float(v) for v in np.asarray(BRUSH_START) + BRUSH_GRIP_LOCAL)
    c, sn = math.cos(HOLD_YAW), math.sin(HOLD_YAW)
    hx, hy, hz = (float(v) for v in np.asarray(HOLD_BRUSH_AT) + np.array([[c, -sn, 0], [sn, c, 0], [0, 0, 1]]) @ BRUSH_GRIP_LOCAL)
    b_open, b_hold = finger_for(WIDE), finger_for(HANDLE[1] / 2 - 0.001)
    left += [Waypoint(2.0, (bx, by, bz + 0.08), b_open, "above the brush handle", GRIP_YAW),
             Waypoint(1.5, (bx, by, bz), b_open, "around the handle", GRIP_YAW),
             Waypoint(0.8, (bx, by, bz), b_hold, "grip", GRIP_YAW),
             Waypoint(1.5, (bx, by, bz + 0.08), b_hold, "lift", GRIP_YAW),
             Waypoint(1.5, (hx, hy, hz), b_hold, "hold it bristles-up", HOLD_YAW)]
    t_brush = sum(w.duration for w in left)
    right.append(_tw(max(0.5, t_brush - sum(w.duration for w in right)), lifted, hold, "wait for the brush"))
    side = (over[0], SIDE_Y, TOP + 0.180)  # beside the held brush, clear of it, where the arm can still hold it level
    right += [_tw(2.0, side, hold, "carry it level beside the brush"),
              _tw(1.5, side, hold, "turn it nozzle-down beside the brush", PITCH),
              _tw(1.5, over, hold, "nozzle 4 cm over the bristles", PITCH),
              _tw(1.2, at, hold, "nozzle 4 mm over the bristles", PITCH),
              _tw(0.3, at, dose, "squeeze", PITCH),
              _tw(dose_s, at, dose, "hold the squeeze", PITCH),
              _tw(0.3, at, hold, "ease off", PITCH),
              _tw(TAIL_S, at, hold, "let the flow die", PITCH),
              _tw(1.0, over, hold, "lift off the dab", PITCH),
              _tw(1.5, side, hold, "beside the brush again", PITCH),
              _tw(1.5, side, hold, "turn it level"),
              _tw(2.0, (gx, ty, tz + CARRY_DZ), hold, "back over its place"),
              _tw(1.5, (gx, ty, tz + 0.002), hold, "lay it down"),
              _tw(0.8, (gx, ty, tz + 0.002), finger_for(WIDE), "let go"),
              _tw(1.0, (gx, ty, tz + 0.08), finger_for(WIDE), "lift clear"),
              _tw(6.0, (gx, ty, tz + 0.08), finger_for(WIDE), "wait")]
    t_dosed = sum(w.duration for w in right[:[w.label for w in right].index("lift off the dab") + 1])
    left += [Waypoint(max(0.5, t_dosed - sum(w.duration for w in left)), (hx, hy, hz), b_hold,
                      "hold still while it is dosed", HOLD_YAW),
             Waypoint(1.5, (bx, by, bz + 0.08), b_hold, "back over its place", GRIP_YAW),
             Waypoint(1.5, (bx, by, bz + 0.001), b_hold, "lay it down", GRIP_YAW),
             Waypoint(0.8, (bx, by, bz + 0.001), b_open, "let go", GRIP_YAW),
             Waypoint(1.0, (bx, by, bz + 0.08), b_open, "lift clear", GRIP_YAW),
             Waypoint(4.0, (bx, by, bz + 0.08), b_open, "wait", GRIP_YAW)]
    return {"left": left, "right": right}
