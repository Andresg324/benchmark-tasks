"""A steel plate on the bench, coated with blue layout dye, with two scribed center lines crossing at a center-punch
mark: the center of a 60 mm hole to be laid out. A pair of 100 mm spring dividers, already set to a 30 mm radius,
stands with its points stuck in a cork block beside the plate.

To lay out the hole a machinist or sheet-metal worker sets the point of one leg in the punch mark, presses lightly on
the dividers' handle to keep it seated, and swings the other leg all the way round so its point scribes the circle,
then lifts the dividers off and sticks them back in the cork. The punch mark is what holds the pivot leg: if the
pressure comes off mid-swing or the hand pushes sideways, the point skates out of the mark and the second half of the
line comes out doubled or off-center, which spoils the layout (a scribed line on steel can't be rubbed out); starting
off the mark scribes a circle with the wrong center, and a swing short of a full turn leaves a gap.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import mechkit as mk  # noqa: E402
import pivotkit as pv  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled plate and dividers; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = ("Scribe the full circle for the hole on the blue steel plate with the dividers, centered on the punch "
               "mark, then stand the dividers back in the cork.")
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
MX, MY = 0.260, -0.100  # the center-punch mark (the hole's center)
BASE = (MX, MY, TOP)  # the mechanism's base origin: on the table under the mark
PLATE_T = 0.006  # m: 6 mm (1/4 in) mild steel plate
PLATE = ((MX - 0.060, MY - 0.055), (MX + 0.070, MY + 0.060))  # its corners (13 x 11.5 cm)
CORK_T = 0.006  # m: a 6 mm cork block, its top level with the plate's
CX, CY = MX, MY - 0.115  # the cork block's center (6 x 6 cm), 3 cm to the right of the plate
CORK = ((CX - 0.030, CY - 0.030), (CX + 0.030, CY + 0.030))

# The dividers (100 mm spring dividers), in their own frame: the origin is the pivot leg's point, +x runs to the
# scribing leg's point R away; z up from the points. Set to R = 30 mm (a 60 mm hole).
R = 0.030
LEG_TOP = 0.096  # m: the legs meet the spring bow's ends 9.6 cm up
BOW_R = 0.009  # m: the spring bow's radius
KNOB_Z0, KNOB_H, KNOB_R = 0.112, 0.024, 0.013  # the handle: bottom above the points, height, radius
# (a real handle is about 8 mm across; this one is 2.6 cm so the gripper can hold it, see fidelity)
PART_MASS, KNOB_MASS = 0.080, 0.020  # kg: steel legs, bow and screw; the handle
SPRING_PRE, SPRING_K, KNOB_TRAVEL = 1.5, 500.0, 0.004  # N, N/m, m: the handle rides 4 mm down on a light spring

# --- Scoring thresholds.
SEAT_IN = 0.0005  # m: the point drops into the punch mark's cone once within 0.5 mm of its center (a center-punch
#                   mark struck lightly is about 2 mm across, so 0.5 mm is well down its 90-degree cone)
SEAT_OUT = 0.0015  # m: and stays seated until it is 1.5 mm off: the mark's rim (1 mm) plus 0.5 mm for the play the sim
#                    lets the seated point shift (0.83 mm measured against 0.3 mm stops); past it, it has skated out
PLAY = 0.0003  # m: how far the seated point can shift in the cone either way (its stops in the pivot kit)
PRESS = 0.0015  # m: the handle pressed 1.5 mm down its spring (2.25 N, a light fingertip press) holds the point in
RIDE_OUT = 1.0  # deg: turned this far without that press, the pivot point rides out of the mark (0.5 mm of travel at
#                 the scribing point, about the depth of the punch mark's cone)
POINT_DRAG = 30.0  # N*s/m: the points' drag on the steel while down and unseated (two hardened points on dyed steel)
DOWN = 0.0005  # m: the points within 0.5 mm of the steel are on it (they would scratch it if moved)
STRAY = 0.0010  # m: a 1 mm stray scratch (a leg dragged with the point unseated) shows on blued steel
FULL_ARC = 355.0  # deg: a 5-degree gap is 2.6 mm on the 60 mm circle, about the width of the saw's kerf when the
#                   hole is cut, so a circle closed within it reads as complete
HALF_ARC = 180.0  # deg: half the circle scribed cleanly, the least that is real progress
HANDS_OFF = 0.030  # m: a tool point within 3 cm of the handle's center is still on it (it is 2.6 cm across)
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving
GRIPPERS = ("left_gripper", "right_gripper")

STEEL_BLUE = kit.Look((0.10, 0.20, 0.52), 0.45, metallic=0.4)  # layout dye on steel
PLATE_EDGE = kit.Look((0.55, 0.56, 0.58), 0.35, metallic=0.8)
LINE = kit.Look((0.86, 0.88, 0.92), 0.25, metallic=0.8)  # bright steel where the dye is scribed through
PUNCH = kit.Look((0.08, 0.08, 0.10), 0.6)
CORK_LOOK = kit.Look((0.42, 0.27, 0.15), 0.95)
DIV = kit.Look((0.80, 0.81, 0.83), 0.25, metallic=0.9)
DARK = kit.Look((0.20, 0.21, 0.23), 0.4, metallic=0.7)


def _w(c):
    """A point given in world coordinates (z above the table), in the base's frame."""
    return (c[0] - BASE[0], c[1] - BASE[1], c[2])


def _wbox(lo, hi, look, meshes, colliders=None):
    """A box between two world corners, added to meshes (and colliders)."""
    c = tuple((a + b) / 2 for a, b in zip(lo, hi))
    s = tuple(b - a for a, b in zip(lo, hi))
    meshes.append(kit.box(_w(c), s, look))
    if colliders is not None:
        colliders.append(kit.Collider(_w(c), size=s))


# --- The dividers' seat in the punch mark (the pivot condition). The chain's turn is about the mark, so the pivot
# point sits at (sx, sy) from it in the turned frame: its distance from the mark is hypot(sx, sy).
_SEATED = {"episode": None, "in": False, "free_at": None, "last": None}


def point_off(J):
    """The pivot leg's point's distance from the punch mark's center (m)."""
    return math.hypot(J["dv_x"], J["dv"])


def pressed(J):
    return J["knob"] <= -PRESS


def seated(J):
    """The pivot point in the punch mark's cone with the points on the steel. It drops in within SEAT_IN of the mark's
    center and stays in until it is pushed out to the rim (SEAT_OUT), or until the dividers have turned RIDE_OUT away
    from where the press on the handle came off: the scribing point's drag, unopposed by a press, lifts the pivot point
    up out of its cone. Called once per control step (by the pivot kit)."""
    st = _SEATED
    if st["episode"] != kit.episode():
        st.update(episode=kit.episode(), free_at=None, last=None, **{"in": False})
    th = J["dv_turn"]
    if pressed(J) or J["lift"] > DOWN:
        st["free_at"] = None
    elif st["free_at"] is None:
        st["free_at"] = th if st["last"] is None else st["last"]  # the press came off at the last step's turn
    st["last"] = th
    loose = 0.0 if st["free_at"] is None else abs(th - st["free_at"])
    lim = SEAT_OUT if st["in"] else SEAT_IN
    st["in"] = J["lift"] <= DOWN and point_off(J) <= lim and loose <= math.radians(RIDE_OUT)
    return st["in"]


def _resist(J):
    """The points' friction on the steel while down and not seated (the turn still scribes freely when seated)."""
    return (POINT_DRAG, 0.01) if J["lift"] <= DOWN and not _SEATED["in"] else None


PIVOT = pv.Pivot("dv", engaged=seated, free="slide", tol=PLAY, slip=SEAT_OUT, slide_damp=4.0, turn_damp=0.01,
                 resist=_resist)


class HandleSpring:
    """The handle's spring: a preloaded push up against its top stop (a force, as a stiffness drive won't move a
    slide), growing 500 N/m as it is pressed down."""

    @staticmethod
    def drive(key, q, qd, J, mem):
        return q, 0.0, SPRING_PRE + SPRING_K * max(0.0, -q), 0.3


def _leg(foot, top, r, look):
    """A leg from its point (foot) to its top, as a tapered steel rod with a fine point."""
    foot, top = np.asarray(foot, float), np.asarray(top, float)
    d = top - foot
    n = d / np.linalg.norm(d)
    tip = 0.010
    return [kit.cylinder(tuple(foot + n * (tip + (np.linalg.norm(d) - tip) / 2)), r, np.linalg.norm(d) - tip, look,
                         axis=n, n=16),
            kit.lathe([(0.0001, 0.0), (r * 0.8, tip)], look, tuple(foot), axis=n, n=16)]


def _divider_links():
    mid = R / 2
    m = []
    m += _leg((0.0, 0.0, 0.0), (mid - 0.006, 0.0, LEG_TOP), 0.0022, DIV)
    m += _leg((R, 0.0, 0.0), (mid + 0.006, 0.0, LEG_TOP), 0.0022, DIV)
    bow = [(mid + BOW_R * math.cos(a), 0.0, LEG_TOP + 0.002 + BOW_R * (1 + math.sin(a)))
           for a in np.linspace(-math.pi * 0.85, math.pi * 1.85, 40)]
    m.append(kit.band(bow, 0.004, 0.0012, DIV, normal=(0, 1, 0)))  # the flat spring bow over the legs
    m.append(kit.cylinder((mid, 0.0, KNOB_Z0 - 0.003), 0.002, 0.006, DIV))  # the handle's stem
    # the adjusting screw across the legs, with its knurled nut
    m.append(kit.cylinder((mid, 0.0, 0.055), 0.0012, 0.022, DIV, axis="x"))
    m.append(kit.cylinder((mid + 0.004, 0.0, 0.055), 0.0045, 0.003, DARK, axis="x", n=20))
    col = kit.Collider((mid, 0.0, LEG_TOP + 0.010), size=(0.020, 0.005, 0.020))
    links = list(pv.planar(PIVOT, pin=_w((MX, MY, PLATE_T)), reach=((-0.14, 0.14), (-0.14, 0.14)),
                           turn=(-7.5, 7.5), mass=PART_MASS, meshes=m, colliders=(col,), parent="lift",
                           start=(CX - mid - MX, CY - MY, 0.0)))
    knob_mesh = (kit.cylinder((0, 0, KNOB_H / 2), KNOB_R, KNOB_H, DARK, n=24),
                 kit.cylinder((0, 0, KNOB_H + 0.0005), KNOB_R - 0.002, 0.001, DIV, n=24))
    knob = kit.Link("knob", "prismatic", "Z", _w((MX + mid, MY, PLATE_T + KNOB_Z0)), lower=-KNOB_TRAVEL, upper=0.0,
                    start=0.0, mass=KNOB_MASS, meshes=knob_mesh, parent="dv", damping=0.0,
                    colliders=(kit.Collider((0, 0, KNOB_H / 2), radius=KNOB_R, height=KNOB_H),), law=HandleSpring())
    lift = kit.Link("lift", "prismatic", "Z", _w((MX, MY, PLATE_T)), lower=0.0, upper=0.08, start=0.0, mass=0.005,
                    damping=0.0, law=mk.Free(drag=0.5))
    return [lift] + links + [knob]


def _bench():
    meshes, colliders = [], []
    (x0, y0), (x1, y1) = PLATE
    _wbox((x0, y0, 0.0), (x1, y1, PLATE_T - 0.0002), PLATE_EDGE, meshes, colliders)
    _wbox((x0 + 0.0008, y0 + 0.0008, PLATE_T - 0.0003), (x1 - 0.0008, y1 - 0.0008, PLATE_T), STEEL_BLUE, meshes)
    z = PLATE_T + 0.0001
    _wbox((MX - 0.045, MY - 0.0005, PLATE_T - 0.0002), (MX + 0.045, MY + 0.0005, z), LINE, meshes)  # center lines
    _wbox((MX - 0.0005, MY - 0.045, PLATE_T - 0.0002), (MX + 0.0005, MY + 0.045, z), LINE, meshes)
    meshes.append(kit.cylinder(_w((MX, MY, z + 0.0001)), 0.0011, 0.0003, PUNCH, n=20))  # the punch mark
    (a0, b0), (a1, b1) = CORK
    _wbox((a0, b0, 0.0), (a1, b1, CORK_T), CORK_LOOK, meshes, colliders)
    return meshes, colliders


def _mechanism():
    meshes, colliders = _bench()
    return kit.Mechanism("bench", pos=BASE, size=(0.13, 0.25, 0.15), meshes=tuple(meshes),
                         colliders=tuple(colliders), links=tuple(_divider_links()))


OBJECTS = [_mechanism()]
KEY = "bench/dv"
NAMES = ("lift", "dv_turn", "dv_x", "dv", "knob")


# --- Geometry from the joints.
def _points(J):
    """World (x, y) of the pivot point and the scribing point."""
    th = J["dv_turn"]
    c, s = math.cos(th), math.sin(th)
    sx, sy = J["dv_x"], J["dv"]
    p0 = np.array([MX + c * sx - s * sy, MY + s * sx + c * sy])
    p1 = p0 + R * np.array([c, s])
    return p0, p1


def _inside(p, rect, inset=0.0):
    (x0, y0), (x1, y1) = rect
    return x0 + inset <= p[0] <= x1 - inset and y0 + inset <= p[1] <= y1 - inset


def handle_center(J):
    p0, p1 = _points(J)
    xy = (p0 + p1) / 2
    return np.array([xy[0], xy[1], TOP + PLATE_T + J["lift"] + KNOB_Z0 + J["knob"] + KNOB_H / 2])


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.pts = None  # the two points' (x, y) at the last step
        self.theta = None  # the dividers' turn at the last step (rad)
        self.was_seated = False
        self.stray = [0.0, 0.0]  # m scratched by each point while down on the plate and not seated
        self.skated = False  # irreversible: a stray scratch (started off the mark, or skated out of it)
        self.arc = np.zeros(3600, bool)  # 0.1-degree bins of the circle scribed with the pivot seated (fine bins, so
        # the two end bins over-count the arc by at most 0.2 degrees)
        self.lifted = 0.0  # the dividers' highest lift since the last scribing (m)
        self.line = []  # scribed dots (world x, y) for the drawing
        self.home = None
        self.last = None
        self.still = False


_H = _History()


def _joints(objects):
    return {n: kit.joint(objects, f"bench/{n}").pos for n in NAMES}


def _cover(a0, a1):
    """Mark the 1-degree bins between two turns (rad) as scribed."""
    d0, d1 = sorted((math.degrees(a0) * 10.0, math.degrees(a1) * 10.0))
    k0, k1 = int(math.floor(d0)), int(math.floor(d1))
    if k1 - k0 >= 3600:
        _H.arc[:] = True
        return
    for k in range(k0, k1 + 1):
        _H.arc[k % 3600] = True


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    J = _joints(objects)
    pts = _points(J)
    seat = pv.engaged(objects, KEY)
    down = J["lift"] <= DOWN
    if _H.pts is not None:
        if down and not seat:
            for i in range(2):
                if _inside(pts[i], PLATE) or _inside(_H.pts[i], PLATE):
                    _H.stray[i] += float(np.linalg.norm(pts[i] - _H.pts[i]))
            if max(_H.stray) > STRAY or pv.slid(objects, KEY):
                _H.skated = True
        if seat and _H.was_seated:
            _cover(_H.theta, J["dv_turn"])
            _H.lifted = 0.0
        if down and _inside(pts[1], PLATE) and (not _H.line or np.linalg.norm(pts[1] - _H.line[-1]) > 0.0006):
            _H.line.append(pts[1].copy())
    _H.lifted = max(_H.lifted, J["lift"])
    _H.pts, _H.theta, _H.was_seated = pts, J["dv_turn"], seat
    now = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    if _H.home is None and len(now) == 2:
        _H.home = now
    _H.still = _H.last is not None and all(g in _H.last and np.linalg.norm(now[g] - _H.last[g]) < HOME_STILL
                                           for g in now)
    _H.last = now
    _draw()
    if os.environ.get("DIVIDERS_TRACE"):  # a per-step trace for debugging sim runs (off in normal runs)
        with open(os.environ["DIVIDERS_TRACE"], "a") as f:
            f.write(" ".join(f"{k}={v:.5f}" for k, v in J.items()) + f" seat={seat} off={point_off(J) * 1000:.2f}"
                    f" stray={_H.stray[0] * 1000:.2f},{_H.stray[1] * 1000:.2f} arc={int(_H.arc.sum())}"
                    f" score={_score(objects)} R={np.round(now.get('right_gripper', np.zeros(3)), 3)}\n")


def arc_deg():
    return float(_H.arc.sum()) / 10.0


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still."""
    if _H.home is None or not _H.still:
        return False
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - _H.home[g]) < HOME_TOL for g in GRIPPERS)


def _released(objects):
    """No tool point within 3 cm of the dividers' handle."""
    hc = handle_center(_joints(objects))
    return all(np.linalg.norm(np.asarray(objects[g].pos, float) - hc) >= HANDS_OFF for g in GRIPPERS if g in objects)


def in_cork(J):
    """The dividers standing in the cork block: points down, both at least 3 mm inside its edges."""
    p0, p1 = _points(J)
    return J["lift"] <= DOWN and _inside(p0, CORK, 0.003) and _inside(p1, CORK, 0.003)


def _job(objects):
    if _H.skated or pv.slid(objects, KEY):
        return 0.0  # a stray scratch: started off the mark, or the point skated out of it
    if not _released(objects):
        return 0.0  # nothing counts while the dividers are still in hand
    arc = arc_deg()
    if arc >= FULL_ARC:
        return 1.0 if in_cork(_joints(objects)) else 0.75
    if arc >= HALF_ARC:
        return 0.5
    return 0.0


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """At least 355 degrees of the circle scribed with the pivot point seated, no stray scratch, the dividers back in
    the cork and let go, and (with REQUIRE_ARMS_HOME) both arms home and still. Called first each step, so it also
    records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0, irreversibly, if either divider point ever moved more than 1 mm in all over the plate while down on it
    without the pivot point seated in the punch mark under pressure (starting off the mark, or skating out of it
    mid-swing). 0 while a tool point is within 3 cm of the dividers' handle. Otherwise: 1.0 with at least 355 degrees
    of the circle scribed with the pivot seated and the dividers standing back in the cork; 0.75 the same with the
    dividers left elsewhere; 0.5 with at least 180 degrees scribed; 0 otherwise. With REQUIRE_ARMS_HOME, a finished
    job with the arms not back home and still scores 0.9. Possible scores: 0, 0.5, 0.75, 0.9, 1."""
    return _score(objects)


# --- The scribed line, drawn as markers (visual only; scoring never reads it): a bright dot wherever the scribing
# leg's point has been while down on the plate, so a skated swing shows as a doubled or offset arc.
_VIS = {"markers": None, "failed": False}
_N_DOTS = 640


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        mat = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in LINE.color), roughness=0.25,
                                          metallic=0.8)
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(
            prim_path="/World/Visuals/scribed_line",
            markers={"dot": sim_utils.CylinderCfg(radius=0.0008, height=0.0002, visual_material=mat)}))
    except Exception as e:  # visuals are optional
        print(f"dividers: no scribed-line visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _draw():
    if "isaaclab" not in sys.modules:
        return
    m = _markers()
    if m is None:
        return
    tr = [(p[0], p[1], TOP + PLATE_T + 0.0002) for p in _H.line[:_N_DOTS]]
    tr += [(0.0, 0.0, -10.0)] * (_N_DOTS - len(tr))
    m.visualize(translations=np.array(tr, float), orientations=np.tile([0.0, 0.0, 0.0, 1.0], (_N_DOTS, 1)),
                marker_indices=np.zeros(_N_DOTS, int))


# --- The scripted demo (right arm; the left arm stays home).
GRIP_Z = TOP + PLATE_T + KNOB_Z0 + 0.015  # the tool point gripping the handle (pad tips 2 mm above its bottom)
PRESS_Z = GRIP_Z - 0.006  # pressing: 4 mm down the spring and 2 mm more against the points
CARRY = 0.035  # m: the points carried 3.5 cm above the steel


def _handle_xy(px, py, th):
    """The handle's (x, y) with the pivot point at (px, py) and the dividers turned th (rad)."""
    return px + R / 2 * math.cos(th), py + R / 2 * math.sin(th)


STROKES = (0.0, 125.0, 250.0, 375.0)  # deg: three strokes of 125 deg, a full turn and 15 deg of overlap
YAW0 = -80.0  # deg: the grip's yaw at the start of each stroke (wrist roll 100 deg, inside its +-120 deg limit)


def demo():
    """Right arm: grip the dividers' handle in the cork, lift them 3.5 cm, carry them over the punch mark and lower
    the pivot point into it, press, and swing the handle round the mark in three 125-degree strokes, letting go and
    rolling the wrist back between strokes (the point stays seated: nothing turns it while the hand is off). Then lift
    the dividers off, carry them back to the cork, set them down, let go, home."""
    C, O = FINGER_CLOSED, FINGER_OPEN
    y0 = math.radians(YAW0)
    hx, hy = CX, CY
    right = [Waypoint(2.6, (hx, hy, GRIP_Z + 0.05), O, "over the dividers' handle", y0),
             Waypoint(1.0, (hx, hy, GRIP_Z), O, "around it", y0),
             Waypoint(0.6, (hx, hy, GRIP_Z), C, "grip it", y0),
             Waypoint(0.8, (hx, hy, GRIP_Z + CARRY), C, "lift the points out of the cork", y0)]
    sx, sy = _handle_xy(MX, MY, 0.0)
    right += [Waypoint(1.6, (sx, sy, GRIP_Z + CARRY), C, "carry them over the punch mark", y0),
              Waypoint(1.2, (sx, sy, GRIP_Z + 0.006), C, "lower the points to just above the steel", y0),
              Waypoint(0.8, (sx, sy, GRIP_Z + 0.006), C, "settle over the mark", y0),
              Waypoint(1.0, (sx, sy, GRIP_Z), C, "set the point in the mark", y0),
              Waypoint(0.8, (sx, sy, PRESS_Z), C, "press", y0)]
    for i in range(len(STROKES) - 1):
        a0, a1 = STROKES[i], STROKES[i + 1]
        n = 10
        for k in range(1, n + 1):
            a = a0 + (a1 - a0) * k / n
            x, y = _handle_xy(MX, MY, math.radians(a))
            right.append(Waypoint(0.45, (x, y, PRESS_Z), C, "swing the scribing leg round", y0 + math.radians(a - a0)))
        x, y = _handle_xy(MX, MY, math.radians(a1))
        right.append(Waypoint(0.5, (x, y, PRESS_Z), C, "hold the press at the end of the stroke",
                              y0 + math.radians(a1 - a0)))
        if i < len(STROKES) - 2:  # regrip: let go, roll the wrist back, grip again and press
            right += [Waypoint(0.5, (x, y, GRIP_Z), C, "ease off", y0 + math.radians(a1 - a0)),
                      Waypoint(0.5, (x, y, GRIP_Z), O, "let go", y0 + math.radians(a1 - a0)),
                      Waypoint(0.6, (x, y, GRIP_Z + 0.04), O, "up off the handle", y0 + math.radians(a1 - a0)),
                      Waypoint(2.2, (x, y, GRIP_Z + 0.04), O, "roll the wrist back", y0),
                      Waypoint(0.8, (x, y, GRIP_Z), O, "around the handle", y0),
                      Waypoint(0.6, (x, y, GRIP_Z), C, "grip it", y0),
                      Waypoint(0.5, (x, y, PRESS_Z), C, "press", y0)]
        else:
            yaw = y0 + math.radians(a1 - a0)
            th = math.radians(a1)
            right += [Waypoint(0.5, (x, y, GRIP_Z), C, "ease off", yaw),
                      Waypoint(0.8, (x, y, GRIP_Z + CARRY), C, "lift the dividers off", yaw)]
    # back to the cork, the handle over its center (the dividers keep their last turn)
    right += [Waypoint(1.6, (CX, CY, GRIP_Z + CARRY), C, "carry them to the cork", yaw),
              Waypoint(1.2, (CX, CY, GRIP_Z - 0.002), C, "stick the points in the cork", yaw),
              Waypoint(0.6, (CX, CY, GRIP_Z - 0.002), O, "let go", yaw),
              Waypoint(0.8, (CX, CY, GRIP_Z + 0.05), O, "up off the handle", yaw),
              kit.Home(2.0), kit.Home(1.5, label="hold at home")]
    t = sum(w.duration for w in right)
    left = [kit.Home(t, label="stay home")]
    return {"left": left, "right": right}
