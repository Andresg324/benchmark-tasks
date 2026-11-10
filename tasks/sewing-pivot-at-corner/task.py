"""A single-needle lockstitch sewing machine set flush in its table, stopped at a corner of a topstitched leather
panel. The machine's bed is the table top; its arm reaches over the bed from the pillar on the robot's right to the
needle, with the handwheel on the pillar's right side and the presser-foot lifter lever on the front of the head. The
panel (16 x 16 cm, 3 mm thick, tan leather) lies on the bed with the needle 1.5 cm in from its front-right corner:
this is its third corner, so the back and right sides of the topstitching are sewn (cream thread, ending at the
needle) and the front and left sides are marked by a scored stitching groove. The needle is up, just above the leather, and the foot is
down on it.

To turn a corner a machine operator turns the handwheel by hand until the needle is down through the work, raises the
presser foot, pivots the work 90 degrees about the needle so the next side lines up with the feed, and lowers the foot
again. Turning with the needle up shifts the corner and breaks the seam line (and in leather every needle hole is
permanent); turning with the foot down drags and scuffs the work and bends the needle.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import mechkit as mk  # noqa: E402
import pivotkit as pv  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled machine and its moving parts; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Turn the corner of the leather panel on the sewing machine, ready to sew its next side."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
NX, NY = 0.250, 0.030  # the needle's line (where it comes down through the bed)
BASE = (NX, NY, TOP)  # the machine's base origin: on the table, under the needle
SEAM = 0.015  # m: the topstitching runs 1.5 cm in from the panel's edges
PANEL = 0.160  # m: the panel is 16 cm square
THICK = 0.003  # m: 3 mm of leather (two layers of 1.5 mm, glued)
GAP = 0.001  # m: the panel's underside rides 1 mm above the table (the bed) on its joints
PANEL_MASS = 0.080  # kg: 16 x 16 cm of 3 mm leather (density about 1 g/cm3)
UNDER, OVER = GAP, GAP + THICK  # the panel's underside and top above the table

# The needle: one handwheel turn is one stitch. The wheel's angle `a` is the crank knob's angle above the front
# horizontal (turning the wheel's top toward the operator lowers it). The needle's phase is 120 deg - a: 0 at its top,
# 180 at its bottom. Its tip runs from 18 mm above the table (14 mm above the leather) to 12 mm below the table top
# (through the throat plate into the bobbin case), a 30 mm stroke as on industrial lockstitch machines.
WHEEL_START = math.radians(60.0)  # stopped with the needle up (phase 60 deg: tip 6.5 mm above the leather)
TIP_LOW, STROKE = -0.012, 0.030


def needle_tip(a):
    """The needle tip's height above the table with the handwheel at `a` (rad)."""
    return TIP_LOW + STROKE * (1 + math.cos(math.radians(120.0) - a)) / 2


THROUGH = 0.001  # m: the needle is in once its tip is 1 mm below the leather's underside (its point fully through)
WHEEL = (0.255, -0.212, 0.120)  # the handwheel's center
WHEEL_R, KNOB_AT, KNOB_R, KNOB_L = 0.045, 0.033, 0.011, 0.025  # wheel radius, knob radius on the wheel, knob size
LIFTER = (0.218, -0.014, 0.120)  # the lifter lever's pivot on the head's front face
LEVER_L, TAB_R, TAB_L = 0.060, 0.0125, 0.042  # lever length to its grip, the grip's radius and length (along y)
TAB_Y = -0.024  # the grip sticks out to the right (-y) from the lever's end: its middle is 2.4 cm off the lever
FOOT_DOWN_Q, FOOT_UP_Q = -0.20, 0.30  # rad: the lever's two detents (grip 1.2 cm below / 1.8 cm above its pivot)
DETENT_ZONE = 0.20  # rad: within this of a detent the lever is pulled into it
# The foot counts as down only with the lever inside the down detent's zone (q <= 0, where a released lever springs
# back down onto the leather), plus 0.002 rad (0.03 mm of foot lift): the lever sits at -0.2000 +/- 0.0001 rad in its
# detent in the sim, so this is 20 times its resting noise. Between the detents (a lever left at q 0 to 0.1, foot
# 2.8 to 4.2 mm up) it is not down.
FOOT_DOWN_MAX = FOOT_DOWN_Q + DETENT_ZONE + 0.002
FOOT_LIFT = 0.007  # m: the foot rises 7 mm when lifted (a hand lifter's usual lift)

# The panel's drag on the bed: light when the foot is up (leather sliding on a table), heavy under the foot.
FREE_DAMP = (3.0, 0.015)  # N*s/m and N*m*s/rad: 80 g of leather sliding on the bed
FOOT_DAMP = (150.0, 1.5)  # under the presser foot (a ~40 N foot spring on the feed dogs): it drags

# --- Scoring thresholds.
TURN_GOAL, TURN_TOL = 90.0, 8.0  # deg: a square corner; 8 deg is 1.8 cm over the 13 cm next side, which the
#                                  operator still steers back onto the groove in the first stitches
OFF_TURN = 8.0  # deg: turning more than the corner's own tolerance off the needle (or under the foot) is a turn
JITTER = 0.05  # deg: turn under the foot is counted through this dead band (as pivotkit's free turn), so the sim's
#                resting jitter (under 0.005 deg a step) never adds up over a long run; 0.05 deg is 0.1 mm at the
#                panel's far corner, and a real turn is counted in full once it passes it
STILL_STEP = 0.001  # m per step: the panel still (under 2 cm/s) when the needle meets it. A panel moved more than
#                     this in the step it meets an already lowered needle was pushed onto the point (a torn hole)
CORNER_TOL = 0.002  # m: the needle within 2 mm of the corner (half a 4 mm stitch): a hole further off shows in leather
PROGRESS = 45.0  # deg: halfway round, the least turn that is real progress
HANDS_OFF = 0.03  # m: no tool point within 3 cm of the lifter lever (its arm or its grip) or the handwheel knob
PRESS = 0.015  # m: no pad tip within 1.5 cm above the panel's top over the panel (nobody still pressing on it)
DT = 0.05  # s per control step (20 Hz)
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving
HOME_STEPS = round(SUCCESS_HOLD_S / DT)  # 20 steps: home and still for 1 s, also when the run ends at its time limit
GRIPPERS = ("left_gripper", "right_gripper")

ENAMEL = kit.Look((0.90, 0.88, 0.82), 0.35)
DARK = kit.Look((0.18, 0.19, 0.20), 0.45)
STEEL = kit.Look((0.78, 0.79, 0.80), 0.25, metallic=0.8)
BED = kit.Look((0.62, 0.64, 0.62), 0.5)
LEATHER = kit.Look((0.62, 0.40, 0.22), 0.7)
EDGE = kit.Look((0.40, 0.24, 0.12), 0.7)
GROOVE = kit.Look((0.44, 0.27, 0.13), 0.8)
THREAD = kit.Look((0.96, 0.94, 0.86), 0.6)
BLACK = kit.Look((0.06, 0.06, 0.07), 0.5)
RED = kit.Look((0.80, 0.10, 0.08), 0.5)
SPOOL = kit.Look((0.95, 0.93, 0.84), 0.6)


def _w(c):
    """A point given in world coordinates (z above the table), in the machine base's frame."""
    return (c[0] - BASE[0], c[1] - BASE[1], c[2])


def _wbox(lo, hi, look, meshes, colliders=None):
    """A box between two world corners, added to meshes (and colliders)."""
    c = tuple((a + b) / 2 for a, b in zip(lo, hi))
    s = tuple(b - a for a, b in zip(lo, hi))
    meshes.append(kit.box(_w(c), s, look))
    if colliders is not None:
        colliders.append(kit.Collider(_w(c), size=s))


def foot_down(J):
    return J["lifter"] <= FOOT_DOWN_MAX


def _pin_in_panel(J):
    """The needle's line passes through the panel (it may have been slid off it)."""
    px, py = -J["panel_x"], -J["panel"]  # the needle's point in the panel's frame
    lo, hi = -SEAM, PANEL - SEAM
    return lo < px < hi and lo < py < hi


def needle_in(J):
    """The needle's point is through the leather (the pivot condition)."""
    return needle_tip(J["wheel"]) <= UNDER - THROUGH and _pin_in_panel(J)


PIVOT = pv.Pivot("panel", engaged=needle_in, free="slide", slide_damp=FREE_DAMP[0], turn_damp=FREE_DAMP[1],
                 resist=lambda J: FOOT_DAMP if foot_down(J) else None)


def _panel_links():
    """The leather panel, pinned by the needle while it is in (pivotkit). Its frame's origin is the corner of the
    stitching under the needle, mid-thickness; +x runs back along the sewn side, +y left along the next side."""
    lo, hi = -SEAM, PANEL - SEAM
    mid = (lo + hi) / 2
    m = [kit.box((mid, mid, 0.0), (PANEL, PANEL, THICK), LEATHER)]
    top = THICK / 2 + 0.0003
    for k in range(4):  # a darker burnished edge all round
        c = [(mid, lo + 0.002), (mid, hi - 0.002), (lo + 0.002, mid), (hi - 0.002, mid)][k]
        s = (PANEL, 0.004) if k < 2 else (0.004, PANEL)
        m.append(kit.box((c[0], c[1], top - 0.0001), (s[0], s[1], 0.0004), EDGE))
    far = PANEL - 2 * SEAM  # 13 cm between corners
    # this is the panel's third corner: the back side and the right side are sewn (cream thread, 4 mm stitches with
    # 3.5 mm of thread on top), the front and left sides are still to sew (a scored groove marks the line)
    for c, s in (((0.0, far / 2, top), (0.0012, far, 0.0004)), ((far / 2, far, top), (far, 0.0012, 0.0004))):
        m.append(kit.box(c, s, GROOVE))
    n = int(round(far / 0.004))
    for k in range(n):
        u = 0.004 * k + 0.002
        m.append(kit.box((far - u, 0.0, top + 0.0002), (0.0035, 0.0018, 0.0006), THREAD))  # right side, to the needle
        m.append(kit.box((far, far - u, top + 0.0002), (0.0018, 0.0035, 0.0006), THREAD))  # back side
    col = kit.Collider((mid, mid, 0.0), size=(PANEL, PANEL, THICK))
    return pv.planar(PIVOT, pin=(0.0, 0.0, GAP + THICK / 2), reach=((-0.06, 0.06), (-0.06, 0.06)),
                     turn=(-0.6, 2.4), mass=PANEL_MASS, meshes=m, colliders=(col,))


class NeedleBar:
    """The needle bar follows the handwheel (a crank): its drop below the top of the stroke."""

    @staticmethod
    def fn(J, mem):
        return needle_tip(J["wheel"]) - (TIP_LOW + STROKE)


def _machine():
    meshes, colliders = [], []
    # the machine bed, flush with the table, and its throat plate around the needle
    _wbox((0.170, -0.240, 0.0), (0.330, 0.075, 0.0006), BED, meshes)
    _wbox((NX - 0.020, NY - 0.016, 0.0), (NX + 0.030, NY + 0.016, 0.0008), STEEL, meshes)
    for dy in (-0.005, 0.005):  # feed-dog slots
        _wbox((NX - 0.010, NY + dy - 0.001, 0.0), (NX + 0.012, NY + dy + 0.001, 0.0009), BLACK, meshes)
    # the pillar on the right, the arm across, the head over the needle
    _wbox((0.222, -0.204, 0.0), (0.288, -0.140, 0.150), ENAMEL, meshes, colliders)
    _wbox((0.226, -0.204, 0.098), (0.284, 0.004, 0.150), ENAMEL, meshes, colliders)
    _wbox((0.222, 0.004, 0.072), (0.288, 0.062, 0.156), ENAMEL, meshes, colliders)
    _wbox((0.2215, 0.010, 0.080), (0.2225, 0.056, 0.148), STEEL, meshes)  # face plate
    _wbox((0.2235, -0.200, 0.140), (0.2245, -0.150, 0.146), DARK, meshes)  # a dark band on the arm
    # the thread spool on its pin on top of the arm
    meshes.append(kit.cylinder(_w((0.255, -0.060, 0.162)), 0.010, 0.024, SPOOL))
    meshes.append(kit.cylinder(_w((0.255, -0.060, 0.152)), 0.013, 0.003, DARK))
    meshes.append(kit.cylinder(_w((0.255, -0.060, 0.172)), 0.013, 0.003, DARK))

    links = []
    # --- the handwheel (link frame at its center; at joint 0 the crank knob points to the front)
    wy = 0.007
    wmesh = [kit.cylinder((0, 0, 0), WHEEL_R, 2 * wy, DARK, axis="y", n=40),
             kit.cylinder((0, -wy - 0.0005, 0), WHEEL_R - 0.006, 0.001, STEEL, axis="y", n=40),
             kit.cylinder((0, 0.010, 0), 0.012, 0.012, STEEL, axis="y"),  # hub into the pillar
             kit.box((0, -wy - 0.001, WHEEL_R - 0.009), (0.004, 0.0015, 0.014), RED),  # index mark at the top
             kit.cylinder((-KNOB_AT, -wy - KNOB_L / 2, 0), KNOB_R, KNOB_L, BLACK, axis="y")]
    wcol = (kit.Collider((0, 0, 0), radius=WHEEL_R, height=2 * wy, axis="Y"),
            kit.Collider((-KNOB_AT, -wy - KNOB_L / 2, 0), radius=KNOB_R, height=KNOB_L, axis="Y"))
    links.append(kit.Link("wheel", "revolute", "Y", _w(WHEEL), lower=math.radians(-95), upper=math.radians(100),
                          start=WHEEL_START, mass=0.15, meshes=tuple(wmesh), colliders=wcol, damping=0.0,
                          floating=True, law=mk.Free(drag=0.03)))
    # --- the needle bar and needle (link frame at the needle's tip at the top of its stroke)
    top_tip = TIP_LOW + STROKE
    nmesh = (kit.cylinder((0, 0, 0.015), 0.0011, 0.030, STEEL),  # the needle (a 2.2 mm shank, scaled up to show)
             kit.box((0, 0, 0.033), (0.009, 0.009, 0.007), STEEL),  # needle clamp
             kit.cylinder((0, 0, 0.065), 0.0035, 0.060, STEEL))  # needle bar, running up into the head
    links.append(kit.Link("needle", "prismatic", "Z", (0.0, 0.0, top_tip), lower=-STROKE, upper=0.0,
                          start=NeedleBar.fn({"wheel": WHEEL_START}, None), mass=0.02, meshes=nmesh, floating=True,
                          damping=0.0, law=mk.Follow(fn=NeedleBar.fn, lo=-STROKE, hi=0.0, rate=0.01)))
    # --- the presser foot on its bar, just behind the needle (link frame at the sole's underside, foot down)
    fmesh = (kit.box((-0.004, 0.0045, 0.0015), (0.020, 0.005, 0.003), STEEL),  # two toes either side of the needle
             kit.box((-0.004, -0.0045, 0.0015), (0.020, 0.005, 0.003), STEEL),
             kit.box((0.008, 0.0, 0.0025), (0.006, 0.014, 0.005), STEEL),  # heel joining them
             kit.cylinder((0.009, 0.0, 0.040), 0.003, 0.070, STEEL))  # presser bar
    links.append(kit.Link("foot", "prismatic", "Z", (0.0, 0.0, OVER), lower=0.0, upper=FOOT_LIFT, start=0.0,
                          mass=0.02, meshes=fmesh, floating=True, damping=0.0,
                          law=mk.Follow(source="lifter", ratio=FOOT_LIFT / (FOOT_UP_Q - FOOT_DOWN_Q),
                                        offset=-FOOT_DOWN_Q * FOOT_LIFT / (FOOT_UP_Q - FOOT_DOWN_Q),
                                        lo=0.0, hi=FOOT_LIFT, rate=0.002)))
    # --- the presser-foot lifter lever (link frame at its pivot; at joint 0 it points straight at the robot)
    lmesh = (kit.box((-LEVER_L / 2, 0, 0), (LEVER_L, 0.006, 0.009), STEEL),
             kit.cylinder((0, 0.004, 0), 0.008, 0.010, STEEL, axis="y"))
    links.append(kit.Link("lifter", "revolute", "Y", _w(LIFTER), lower=FOOT_DOWN_Q - 0.03, upper=FOOT_UP_Q + 0.04,
                          start=FOOT_DOWN_Q, mass=0.05, meshes=lmesh,
                          colliders=(kit.Collider((-LEVER_L / 2 + 0.008, 0, 0), size=(LEVER_L - 0.03, 0.006, 0.009)),),
                          damping=0.0, floating=True,
                          law=mk.Detents(positions=(FOOT_DOWN_Q, FOOT_UP_Q), labels=("down", "up"), zone=DETENT_ZONE,
                                         pull=0.8, damp=0.03, drag=0.03)))
    # its grip: a black plastic sleeve on a pin sticking out to the right from the lever's end, free to spin (so a
    # held grip doesn't twist the lever)
    grip_at = (LIFTER[0] - LEVER_L, LIFTER[1], LIFTER[2])
    links.append(kit.Link("lifter_grip", "revolute", "Y", _w(grip_at), lower=-3.0, upper=3.0, start=0.0, mass=0.02,
                          meshes=(kit.cylinder((0, TAB_Y, 0), TAB_R, TAB_L, BLACK, axis="y"),),
                          colliders=(kit.Collider((0, TAB_Y, 0), radius=TAB_R, height=TAB_L, axis="Y"),),
                          damping=0.0, floating=True, parent="lifter", law=mk.Free(drag=0.002)))
    links += list(_panel_links())
    return kit.Mechanism("machine", pos=BASE, size=(0.2, 0.34, 0.2), meshes=tuple(meshes),
                         colliders=tuple(colliders), links=tuple(links))


OBJECTS = [_machine()]
KEY = "machine/panel"


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.angle = None  # the panel's turn at the last step (deg)
        self.foot_turn = 0.0  # deg turned with the presser foot down (irreversible once past OFF_TURN)
        self.foot_ref = None  # the turn (deg) last counted into foot_turn (the dead band's reference)
        self.off_hole = False  # irreversible: the needle went into the leather off the corner (a stray hole)
        self.tip = None  # the needle tip's height at the last step
        self.into = False  # the needle tip was below the leather's top over the panel at the last step
        self.s = None  # the panel's two slides at the last step
        self.home_steps = 0  # consecutive steps with both arms home and still
        self.lifted_in = False  # the foot was up while the needle was in at the corner (the turn was set up)
        self.home = None  # each gripper's tool point at the trial's first scored step
        self.last = None
        self.still = False


_H = _History()


def _joints(objects):
    return {n: kit.joint(objects, f"machine/{n}").pos for n in ("wheel", "lifter", "panel_turn", "panel_x", "panel")}


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    J = _joints(objects)
    ang = pv.angle_deg(objects, KEY)
    if _H.angle is not None and foot_down(J):  # turn under the foot, through the JITTER dead band
        if _H.foot_ref is None:
            _H.foot_ref = _H.angle
        if abs(ang - _H.foot_ref) > JITTER:
            _H.foot_turn += abs(ang - _H.foot_ref)
            _H.foot_ref = ang
    else:
        _H.foot_ref = None
    _H.angle = ang
    # needle holes: the tip below the leather's top over the panel is making one (even before it is through)
    tip, s = needle_tip(J["wheel"]), (J["panel_x"], J["panel"])
    into = tip < OVER and _pin_in_panel(J)
    if into and not _H.into:
        descended = _H.tip is None or _H.tip >= OVER
        moved = 0.0 if _H.s is None else math.hypot(s[0] - _H.s[0], s[1] - _H.s[1])
        if not descended and moved >= STILL_STEP:
            _H.off_hole = True  # the panel was swept onto the lowered needle: torn, wherever it ended up
    if into and math.hypot(*pv.pin_offset(objects, KEY)) > CORNER_TOL:
        _H.off_hole = True  # a hole (or a hole dragged) more than 2 mm from the corner
    _H.tip, _H.into, _H.s = tip, into, s
    at = pv.entered_at(objects, KEY)
    if at is not None and math.hypot(*at) > CORNER_TOL:
        _H.off_hole = True
    if pv.engaged(objects, KEY) and not foot_down(J) and at is not None and math.hypot(*at) <= CORNER_TOL:
        _H.lifted_in = True
    now = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    if _H.home is None and len(now) == 2:
        _H.home = now
    _H.still = _H.last is not None and all(g in _H.last and np.linalg.norm(now[g] - _H.last[g]) < HOME_STILL
                                           for g in now)
    _H.last = now
    home = _H.home is not None and _H.still and all(g in now and np.linalg.norm(now[g] - _H.home[g]) < HOME_TOL for g in GRIPPERS)
    _H.home_steps = _H.home_steps + 1 if home else 0
    if os.environ.get("SEWING_TRACE"):  # a per-step trace for debugging sim runs (off in normal runs)
        with open(os.environ["SEWING_TRACE"], "a") as f:
            f.write(" ".join(f"{k}={v:.4f}" for k, v in J.items()) + f" ang={ang:.2f} in={pv.engaged(objects, KEY)}"
                    f" free={pv.free_turn_deg(objects, KEY):.2f} slip={pv.max_slip_mm(objects, KEY):.2f}"
                    f" foot_turn={_H.foot_turn:.2f} score={_score(objects)}"
                    f" L={np.round(now.get('left_gripper', np.zeros(3)), 3)} R={np.round(now.get('right_gripper', np.zeros(3)), 3)}\n")


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still there for
    the last 1 s (so arms that only pass through home as the run hits its time limit don't count)."""
    return _H.home_steps >= HOME_STEPS


def _seg_dist(p, a, b):
    """The distance from point p to the segment a-b."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    t = np.clip(np.dot(p - a, b - a) / max(np.dot(b - a, b - a), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(p - (a + t * (b - a))))


def _lever_dist(objects, p):
    """How far point p is from the lifter lever: its arm (pivot to end) or its grip (along y from the end)."""
    q = kit.joint(objects, "machine/lifter").pos
    pivot = np.array([LIFTER[0], LIFTER[1], TOP + LIFTER[2]])
    end = pivot + np.array([-LEVER_L * math.cos(q), 0.0, LEVER_L * math.sin(q)])
    g0, g1 = end + (0.0, TAB_Y - TAB_L / 2, 0.0), end + (0.0, TAB_Y + TAB_L / 2, 0.0)
    return min(_seg_dist(p, pivot, end), _seg_dist(p, g0, g1))


def _knob_point(objects):
    a = kit.joint(objects, "machine/wheel").pos
    return np.array([WHEEL[0] - KNOB_AT * math.cos(a), WHEEL[1] - 0.007 - KNOB_L / 2, TOP + WHEEL[2] + KNOB_AT * math.sin(a)])


def _on_panel(objects, g):
    """A gripper's pad tips over the panel (grown 1 cm) and within 1.5 cm above its top: pressing on it."""
    J = _joints(objects)
    th = J["panel_turn"]
    p = np.asarray(objects[g].pos, float)
    tip_z = float(np.asarray(objects[g].low)[2]) - TOP
    d = p[:2] - np.array([NX, NY])
    c, s = math.cos(th), math.sin(th)
    lx, ly = c * d[0] + s * d[1] - J["panel_x"], -s * d[0] + c * d[1] - J["panel"]  # in the panel's frame
    inside = all(-SEAM - 0.01 < v < PANEL - SEAM + 0.01 for v in (lx, ly))
    return inside and tip_z < OVER + PRESS


def _released(objects):
    """Nobody still holding the lifter lever or the handwheel knob, or pressing on the panel."""
    for g in GRIPPERS:
        if g not in objects:
            continue
        p = np.asarray(objects[g].pos, float)
        if _lever_dist(objects, p) < HANDS_OFF or np.linalg.norm(p - _knob_point(objects)) < HANDS_OFF:
            return False
        if _on_panel(objects, g):
            return False
    return True


def _job(objects):
    J = _joints(objects)
    if (pv.free_turn_deg(objects, KEY) > OFF_TURN or _H.foot_turn > OFF_TURN or _H.off_hole
            or pv.slid(objects, KEY)):
        return 0.0  # turned off the needle or under the foot, a stray hole, or forced sideways on the needle
    if not _released(objects):
        return 0.0  # nothing counts while a hand is still on the work or the machine
    ang = pv.angle_deg(objects, KEY)
    corner = math.hypot(*pv.pin_offset(objects, KEY)) <= CORNER_TOL
    if corner and abs(ang - TURN_GOAL) <= TURN_TOL:
        return 1.0 if foot_down(J) else 0.75
    if corner and pv.turned_deg(objects, KEY) >= PROGRESS and ang >= PROGRESS:
        return 0.5
    if _H.lifted_in and pv.engaged(objects, KEY) and corner:
        return 0.25  # set up (foot raised with the needle in at the corner) and the needle still in: raising the
        #              needle again undoes it, and a run back at its start state scores like doing nothing
    return 0.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """The corner turned 90 +/- 8 deg about the needle, the foot back down, nothing held, and (with REQUIRE_ARMS_HOME)
    both arms home and still for 1 s. Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0, irreversibly, if the panel turned more than 8 deg in all while the needle was out of it, or more than 8 deg
    while the presser foot was down (lever in its down detent's zone), both counted through a 0.05 deg dead band, if
    the needle tip went below the leather's top more than 2 mm from the corner or the panel was pushed onto a lowered
    needle (moving 1 mm or more in that step), or if the panel was forced sideways more than 2 mm while the needle was
    in. 0 while a tool point is within 3 cm of the lifter lever (arm or grip) or the handwheel knob, or pad tips within
    1.5 cm above the panel over it. Otherwise: 1.0 with the panel turned 90 +/- 8 deg, the needle still at the corner
    (within 2 mm) and the foot down; 0.75 the same with the foot still up; 0.5 turned at least 45 deg about the needle
    at the corner; 0.25 the foot was raised with the needle in at the corner and the needle is still in there; 0
    otherwise. With REQUIRE_ARMS_HOME, a finished job without both arms home and still for the last 1 s scores 0.9.
    Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1."""
    return _score(objects)


# --- The scripted demo.
TIP = PAD_TIP_PAST_TCP  # the tool point is this far above the pad tips


def _knob_tcp(a, lift=0.0):
    """The tool point with the closed pad tips on top of the crank knob, the wheel at `a`."""
    return (WHEEL[0] - KNOB_AT * math.cos(a), WHEEL[1] - 0.007 - KNOB_L / 2,
            TOP + WHEEL[2] + KNOB_AT * math.sin(a) + KNOB_R + TIP + lift)


def _grip_tcp(q, dz=0.0):
    """The tool point at the lifter grip's center (fingers closing along x around it), the lever at q."""
    return (LIFTER[0] - LEVER_L * math.cos(q), LIFTER[1] + TAB_Y, TOP + LIFTER[2] + LEVER_L * math.sin(q) + dz)


def demo():
    """Right arm: press the handwheel's crank knob down through 118 deg (needle from 6.5 mm above the leather to the
    bottom of its stroke), then grip the lifter lever and raise it to its upper detent (foot up), let go. Left arm:
    press the closed pads on the panel 10 cm from the needle and sweep them 90 deg round the needle, lift off. Right
    arm: press the lifter grip back down to its lower detent (foot down). Both home."""
    C, O = FINGER_CLOSED, FINGER_OPEN
    a0, a1 = WHEEL_START, math.radians(-58.0)
    right = [Waypoint(1.5, _knob_tcp(a0, 0.04), C, "over the handwheel knob"),
             Waypoint(1.0, _knob_tcp(a0, 0.002), C, "on the knob")]
    n = 8
    for k in range(1, n + 1):
        a = a0 + (a1 - a0) * k / n
        right.append(Waypoint(0.6, _knob_tcp(a, -0.004), C, "turn the handwheel toward you"))
    right += [Waypoint(0.8, _knob_tcp(a1, 0.04), C, "lift off the knob"),
              Waypoint(1.5, _grip_tcp(FOOT_DOWN_Q, 0.05), 0.6 * O, "over the lifter grip", math.pi / 2),
              Waypoint(1.0, _grip_tcp(FOOT_DOWN_Q), 0.6 * O, "around it", math.pi / 2),
              Waypoint(0.6, _grip_tcp(FOOT_DOWN_Q), C, "grip it", math.pi / 2),
              Waypoint(1.2, _grip_tcp(FOOT_UP_Q + 0.03), C, "raise the presser foot", math.pi / 2),
              Waypoint(0.6, _grip_tcp(FOOT_UP_Q + 0.03), O, "let go", math.pi / 2),
              Waypoint(1.0, (0.20, -0.14, TOP + 0.16), O, "out of the way", math.pi / 2)]
    t_right = sum(w.duration for w in right)

    # left arm: pads on the panel 10 cm out from the needle, swept 90 deg round it
    r, b0, b1 = 0.10, math.radians(60.0), math.radians(150.0)
    pz = TOP + OVER + TIP - 0.004  # the pads 4 mm into the leather's top: pressing it
    pt = lambda b, z: (NX + r * math.cos(b), NY + r * math.sin(b), z)  # noqa: E731
    left = [Waypoint(t_right, pt(b0, TOP + 0.12), C, "wait over the panel"),
            Waypoint(1.2, pt(b0, pz + 0.01), C, "down onto the panel"),
            Waypoint(0.6, pt(b0, pz), C, "press")]
    n = 9
    for k in range(1, n + 1):
        left.append(Waypoint(0.7, pt(b0 + (b1 - b0) * k / n, pz), C, "pivot the panel about the needle"))
    left += [Waypoint(0.8, pt(b1, TOP + 0.10), C, "lift off"), kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    t_left = sum(w.duration for w in left[:-2])

    right += [Waypoint(max(0.5, t_left - t_right), (0.20, -0.14, TOP + 0.16), O, "wait", math.pi / 2),
              Waypoint(1.2, _grip_tcp(FOOT_UP_Q, 0.04), C, "over the lifter grip", math.pi / 2),
              Waypoint(0.8, _grip_tcp(FOOT_UP_Q, TAB_R + TIP), C, "on top of it", math.pi / 2),
              Waypoint(1.2, _grip_tcp(FOOT_DOWN_Q - 0.02, TAB_R + TIP), C, "lower the presser foot", math.pi / 2),
              Waypoint(0.8, _grip_tcp(FOOT_DOWN_Q, 0.06), C, "lift off", math.pi / 2),
              kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    return {"left": left, "right": right}
