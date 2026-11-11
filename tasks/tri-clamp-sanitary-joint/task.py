"""A 1.5-inch sanitary (tri-clamp) joint on a vertical stainless line: the lower ferrule is welded to a tube stub on a
stand, the upper ferrule ends a short spool held by a bracket, and a white gasket sits between the two ferrule faces.
A hinged clamp ring has been put round the joint and its swing bolt dropped into the slot, but it went on cocked: its
hinge side (toward the robot) rides up on top of the flanges instead of around them, and the wing nut is 1.5 turns
short of touching the ear.

The job is a process technician's: press the clamp's high side down so the channel takes both flanges and the gasket
all round (it drops square once its lip clears the rim), then turn the wing nut snug by hand, half a turn to one and a
half turns past contact. A clamp drawn up while cocked bites the flange rim on one side and leaves the joint open on
the other: it leaks, can blow off under pressure, and the gasket has to be replaced. A nut left at contact leaks.

Models: the stand, ferrules and gasket are fixed; the clamp is a rigid ring on a hinge about its swing-bolt axis
(tilt), held by a seating law (cocked on the rim, square once pressed over it, locked once the nut is past contact);
the wing nut is a twistkit cap on the swing bolt (M8 x 1.25).
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import twistkit as tk  # noqa: E402  (threaded parts on top of yamkit; nothing on disk changes)
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Tighten the clamp on the pipe joint."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the workspace is clear
# for the next person or job. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# --- The joint (m). Stand frame: origin on the pipe's axis at the table top (world (X0, Y0, TOP)), x forward, z up.
X0, Y0 = 0.26, 0.045
ZJ = 0.090  # the joint's mid-plane (between the two ferrule faces) above the table
TUBE_R, TUBE_IR = 0.01905, 0.01740  # 1.5-inch OD sanitary tube, 1.65 mm (16 gauge) wall
FL_R = 0.02525  # ferrule flange: 50.5 mm OD
FL_T = 0.00285  # flange thickness at its rim
TAPER_H = 0.0023  # the flange's back face tapers down to the tube over 2.3 mm (about 20 degrees)
GAP = 0.0010  # the gasket's visible lip between the two faces
Z_TOP = ZJ + 0.040  # the upper spool's cut end (kept low: a hand pressing the clamp's hinge side sits over it)

# The clamp (13MHH-style single-hinge clamp for 1.5-inch ferrules): a channel with lips over both flanges' tapers.
CL_RIN = 0.0220  # lip bore (44 mm): over the tube, under the flanges' 50.5 mm
CL_RCH = 0.0265  # channel back wall (53 mm)
CL_ROUT = 0.0310  # outer surface (62 mm)
CL_H = 0.0070  # half its height (14 mm wide band)
CL_LIP = 0.0025  # lip thickness: the channel inside is 2 x (7.0 - 2.5) = 9.0 mm tall
R_B = 0.050  # the swing bolt's axis from the pipe's axis (tangent to the ring, along y, at the ears)
EAR_T = 0.006  # each ear 6 mm thick; ears 3 mm apart at contact
EAR_GAP = 0.003
HINGE_R = 0.0345  # the hinge knuckle's axis from the pipe's axis, on the near side (-x)

# The wing nut on the +y end of the swing bolt: M8 x 1.25 right-hand, 1.5 turns short of the ear at the start.
NUT_Y0 = EAR_GAP / 2 + EAR_T + 0.005  # the hub's center at contact (hub 10 mm long against the +y ear's face)
NUT_SPAN, NUT_LEN, NUT_WING_T, NUT_HUB_R = 0.036, 0.010, 0.010, 0.0065
THREAD = tk.Thread(pitch=0.00125, turns=4.0, start=-540.0, max_tight=1080.0, hand=1, free_lift=0.0)
NUT = "stand/wingnut"
CLAMP = "stand/clamp"

# --- The clamp's seating (degrees of tilt about the swing-bolt axis; + lifts the hinge side).
Q_COCK = 6.0  # cocked: the hinge side's lower lip sits on top of the upper flange's rim (7.9 mm up, 75 mm from the
#              bolt axis)
Q_SLIP = 4.5  # pressed down past this the lip slips over the rims (2 mm down at the rim) and the clamp drops square
Q_LIFT = 5.5  # a square clamp lifted past this rides back up on the rim
K_SEAT, D_SEAT = 10.0, 0.12  # N*m/rad, N*m*s/rad: a 2 N push on the hinge side moves it about one degree
K_LOCK, D_LOCK = 15.0, 0.12  # held where it is once the nut is past contact (the drive's torque is capped at 20 N*m)

# --- Scoring thresholds (each with its reason).
SEAT_DEG = 0.8  # seated square: tilt under 0.8 degrees, the hinge side within 1.05 mm of square, inside the channel's
#                 1.15 mm of axial play over the flange pair, so both flanges and the gasket are in the channel all round
TIGHT_DEG = 10.0  # past contact: the nut more than 10 degrees (0.035 mm of bolt travel) past touching the ear
SNUG_DEG = 180.0  # snug: at least half a turn past contact (0.6 mm of bolt travel). Less, the halves have not yet drawn
#                  the ferrules onto the gasket, and the joint seeps once the line is under pressure
OVER_DEG = 540.0  # overtight: more than 1.5 turns past contact is past firm hand tight for a wing-nut clamp; it
#                  squeezes the gasket into the bore (a sanitary defect: product lodges there) even if backed off
GRIP_SLACK = 90.0  # all but 90 degrees of the nut's tightening must be done by a gripper holding it (twistkit)
HANDS_R = 0.06  # let go: no gripper's tool point within 6 cm of the joint's center (the clamp's 3.1 cm radius plus a
NUT_R = 0.04  # finger's length) or within 4 cm of the wing nut
HOME_TOL = 0.03  # a gripper is home within 3 cm of where its tool point was at the trial's first scored step
HOME_STILL = 0.001  # and still: its tool point moved less than 1 mm since the last step (2 cm/s)
_DEBUG = os.environ.get("TRICLAMP_DEBUG")  # a file path: per-step state is appended there

STEEL = kit.Look((0.78, 0.79, 0.81), 0.30, metallic=0.75)  # 316L tube and ferrules, polished
CLAMP_LOOK = kit.Look((0.66, 0.67, 0.70), 0.40, metallic=0.6)  # cast 304 clamp, a shade darker than the ferrules
NUT_LOOK = kit.Look((0.86, 0.72, 0.30), 0.40, metallic=0.5)  # brass wing nut
GASKET = kit.Look((0.97, 0.97, 0.95), 0.6)  # white PTFE gasket
DARK = kit.Look((0.20, 0.21, 0.23), 0.6, metallic=0.3)  # painted stand
BORE = kit.Look((0.35, 0.36, 0.38), 0.5, metallic=0.5)


def _arc(r0, r1, z0, z1, a0, a1, look, cx=0.0, n=24):
    """A rectangular-section ring segment: radii r0..r1, heights z0..z1, angles a0..a1 (rad, ccw from +x), its axis
    vertical through (cx, 0)."""
    pts, counts, idx = [], [], []
    sec = [(r0, z0), (r1, z0), (r1, z1), (r0, z1)]
    for k in range(n + 1):
        a = a0 + (a1 - a0) * k / n
        for r, z in sec:
            pts.append(np.array([cx + r * math.cos(a), r * math.sin(a), z]))
    for k in range(n):
        for e in range(4):
            a, b = 4 * k + e, 4 * k + (e + 1) % 4
            counts.append(4)
            idx += [a, b, b + 4, a + 4]
    counts += [4, 4]
    idx += [3, 2, 1, 0, 4 * n, 4 * n + 1, 4 * n + 2, 4 * n + 3]
    return kit._mesh(pts, counts, idx, look)


# --- The stand: base plate, lower tube and ferrule, gasket, upper ferrule and spool, its bracket (all fixed).
def _stand():
    zf = ZJ - GAP / 2  # lower ferrule face
    zu = ZJ + GAP / 2  # upper ferrule face
    m = [kit.box((0.035, 0.0, 0.006), (0.17, 0.10, 0.012), DARK),
         kit.lathe([(TUBE_R, 0.012), (TUBE_R, zf - FL_T - TAPER_H), (FL_R, zf - FL_T), (FL_R, zf), (0.0, zf)],
                   STEEL, n=48),
         kit.lathe([(0.0175, zf), (FL_R + 0.0003, zf), (FL_R + 0.0003, zu), (0.0175, zu)], GASKET, n=48),
         kit.lathe([(0.0, zu), (FL_R, zu), (FL_R, zu + FL_T), (TUBE_R, zu + FL_T + TAPER_H), (TUBE_R, Z_TOP),
                    (TUBE_IR, Z_TOP), (TUBE_IR, Z_TOP - 0.025)], STEEL, n=48),
         kit.cylinder((0.0, 0.0, Z_TOP - 0.0245), TUBE_IR, 0.001, BORE, n=32),
         # the pipe support: a square post behind the joint and an arm with a strap round the upper spool
         kit.box((0.105, 0.0, (0.012 + Z_TOP - 0.007) / 2), (0.025, 0.025, Z_TOP - 0.019), DARK),
         kit.box((0.060, 0.0, Z_TOP - 0.012), (0.090, 0.020, 0.010), DARK),
         _arc(TUBE_R, TUBE_R + 0.002, Z_TOP - 0.019, Z_TOP - 0.005, 0.0, 2 * math.pi, DARK, n=32)]
    c = [kit.Collider((0.035, 0.0, 0.006), size=(0.17, 0.10, 0.012)),
         kit.Collider((0.0, 0.0, (0.012 + zf) / 2), radius=TUBE_R, height=zf - 0.012),
         kit.Collider((0.0, 0.0, ZJ), radius=FL_R, height=2 * FL_T + GAP),
         kit.Collider((0.0, 0.0, (zu + Z_TOP) / 2), radius=TUBE_R, height=Z_TOP - zu),
         kit.Collider((0.105, 0.0, (0.012 + Z_TOP - 0.007) / 2), size=(0.025, 0.025, Z_TOP - 0.019)),
         kit.Collider((0.060, 0.0, Z_TOP - 0.012), size=(0.090, 0.020, 0.010))]
    return m, c


# --- The clamp: two half rings meeting at the hinge (-x) and at the ears (+x), on a hinge about the swing bolt's axis.
def _clamp_meshes():
    """In the clamp link's frame: origin on the swing bolt's axis at the ears (stand (R_B, 0, ZJ)), so the ring's
    center is at (-R_B, 0, 0)."""
    cx = -R_B
    m = []
    for a0, a1 in ((math.radians(6), math.radians(174)), (math.radians(-174), math.radians(-6))):
        m.append(_arc(CL_RCH, CL_ROUT, -CL_H, CL_H, a0, a1, CLAMP_LOOK, cx))  # back wall
        m.append(_arc(CL_RIN, CL_RCH, CL_H - CL_LIP, CL_H, a0, a1, CLAMP_LOOK, cx))  # upper lip
        m.append(_arc(CL_RIN, CL_RCH, -CL_H, -CL_H + CL_LIP, a0, a1, CLAMP_LOOK, cx))  # lower lip
    # the hinge: knuckles of both halves on a vertical pin
    m.append(kit.cylinder((cx - HINGE_R, 0.0, 0.0), 0.0045, 2 * CL_H, CLAMP_LOOK, n=20))
    m.append(kit.cylinder((cx - HINGE_R, 0.0, 0.0), 0.0022, 2 * CL_H + 0.003, STEEL, n=12))
    # the ears: the +y one slotted for the bolt (the nut bears on its outer face), the -y one carrying the bolt's eye
    ear_x0 = CL_ROUT - 0.003 - R_B  # from just inside the ring's outer surface out past the bolt
    ear_len = R_B + 0.008 - (CL_ROUT - 0.003)
    for s in (-1, 1):
        m.append(kit.box((ear_x0 + ear_len / 2, s * (EAR_GAP / 2 + EAR_T / 2), 0.0), (ear_len, EAR_T, 2 * CL_H - 0.002),
                         CLAMP_LOOK))
    # the swing bolt along y through both ears and out past the nut's starting place
    m.append(kit.cylinder((0.0, 0.010, 0.0), 0.004, 0.044, STEEL, axis="y", n=16))
    m.append(kit.cylinder((0.0, -EAR_GAP / 2 - EAR_T - 0.002, 0.0), 0.0065, 0.008, STEEL, axis="y", n=16))  # its eye
    return tuple(m)


class ClampSeat:
    """The clamp's seating law (a yamkit law; angles in rad). Cocked, the hinge side's lip rests on the upper flange's
    rim: a spring toward Q_COCK. Pressed down past Q_SLIP the lip slips over and the clamp drops square: a spring
    toward 0 (lifted past Q_LIFT it rides back up). Once the wing nut is past contact the halves are drawn onto the
    flanges and the clamp is held where it is, cocked or square, until the nut is backed off again."""

    def fresh(self, lk):
        return {"mode": "cocked", "lock": None}

    def drive(self, key, q, qd, J, mem):
        past = tk.past_seat_deg(None, NUT) if NUT in tk._SPECS else -1e9
        if past > TIGHT_DEG:
            if mem["lock"] is None:
                mem["lock"] = q
            return mem["lock"], K_LOCK, 0.0, D_LOCK
        mem["lock"] = None
        if mem["mode"] == "cocked" and q < math.radians(Q_SLIP):
            mem["mode"] = "seated"
        elif mem["mode"] == "seated" and q > math.radians(Q_LIFT):
            mem["mode"] = "cocked"
        target = math.radians(Q_COCK) if mem["mode"] == "cocked" else 0.0
        return target, K_SEAT, 0.0, D_SEAT


def _clamp_link():
    # the pressing face: the hinge side's top, where a hand pushes the high side down (the rest of the clamp has no
    # collider, so fingers at the wing nut do not shove the ring)
    press = kit.Collider((-R_B - 0.031, 0.0, CL_H - 0.004), size=(0.018, 0.045, 0.008))
    return kit.Link("clamp", "revolute", "Y", (R_B, 0.0, ZJ), lower=math.radians(-0.3), upper=math.radians(9.0),
                    start=math.radians(Q_COCK), mass=0.15, meshes=_clamp_meshes(), colliders=(press,), damping=0.0,
                    floating=True, law=ClampSeat())


def _wing_nut():
    """A brass wing nut on the swing bolt, axis along y: a hub and two wings (lying flat at the start)."""
    meshes = [kit.cylinder((0, 0, 0), NUT_HUB_R, NUT_LEN, NUT_LOOK, axis="y", n=20)]
    for s in (-1, 1):
        meshes.append(kit.box((s * (NUT_SPAN / 4 + NUT_HUB_R * 0.3), 0, 0),
                              (NUT_SPAN / 2 - NUT_HUB_R * 0.6, NUT_LEN * 0.8, NUT_WING_T), NUT_LOOK))
    cols = (kit.Collider((0, 0, 0), size=(NUT_SPAN, NUT_LEN, NUT_WING_T)),)
    return tk.Cap("wingnut", anchor=(R_B, NUT_Y0, ZJ), mass=0.02, meshes=tuple(meshes), colliders=cols,
                  grip=(0.0, 0.0, 0.0), width=NUT_WING_T)


def _stand_mech():
    m, c = _stand()
    return tk.Threaded("stand", pos=(X0, Y0, TOP), size=(0.17, 0.10, Z_TOP), meshes=tuple(m), colliders=tuple(c),
                       cap=_wing_nut(), thread=THREAD, axis="Y", extra=(_clamp_link(),))


OBJECTS = [_stand_mech()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.cocked_tight = False  # the nut was ever past contact with the clamp not square (irreversible)
        self.max_past = -1e9  # the most the nut was ever past contact (a squeezed gasket stays squeezed)
        self.was_seated = None  # whether the clamp was square at the last scored step
        self.home = None
        self.last_grip = None
        self.grips_still = False


_H = _History()


def _tilt_deg(objects):
    return math.degrees(kit.joint(objects, CLAMP).pos)


def _seated(objects):
    """Square on both flanges: the clamp's tilt (hinge side up) under SEAT_DEG. The joint's lower limit is -0.3
    degrees (the far lips on the flanges), so the reading is the joint itself, not the law's bookkeeping."""
    return abs(_tilt_deg(objects)) < SEAT_DEG


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    grips = {g: np.asarray(objects[g].pos, float) for g in ("left_gripper", "right_gripper") if g in objects}
    if _H.home is None:
        _H.home = grips
    _H.grips_still = _H.last_grip is not None and all(
        g in _H.last_grip and np.linalg.norm(p - _H.last_grip[g]) < HOME_STILL for g, p in grips.items())
    _H.last_grip = grips
    past = tk.past_seat_deg(objects, NUT)
    _H.max_past = max(_H.max_past, past)
    seated = _seated(objects)
    # cocked at this step or the one before: a nut drawn past contact in the same step the clamp drops square was
    # still drawn up against a cocked clamp
    if past > TIGHT_DEG and not (seated and _H.was_seated in (None, True)):
        _H.cocked_tight = True
    _H.was_seated = seated
    if _DEBUG:
        lg = objects.get("left_gripper")
        print(f"[triclamp] tilt={_tilt_deg(objects):.2f} past={past:.0f} grip={-tk.gripped_deg(objects, NUT):.0f} "
              f"unheld={tk.unheld_deg(objects, NUT):.0f} held={tk.held(objects, NUT)} "
              f"lp={None if lg is None else np.round(np.asarray(lg.pos, float) - NUT_XYZ, 3)} "
              f"op={kit.opening(objects, 'left_gripper'):.2f} mode={kit.law_state(CLAMP).get('mode')} "
              f"score={_score(objects)}", file=open(_DEBUG, "a"))


def _hands_on(objects):
    """A gripper still at the joint: holding the wing nut, or its tool point within 6 cm of the joint's center or 4 cm
    of the nut (pressing the clamp, or hovering on the nut)."""
    if tk.held(objects, NUT):
        return True
    centre = np.array([X0, Y0, TOP + ZJ])
    nut = np.asarray(objects[NUT].pos, float)
    for g in ("left_gripper", "right_gripper"):
        if g in objects:
            p = np.asarray(objects[g].pos, float)
            if np.linalg.norm(p - centre) < HANDS_R or np.linalg.norm(p - nut) < NUT_R:
                return True
    return False


def _arms_home(objects):
    """Each gripper's tool point within 3 cm of where it was at the trial's first scored step, and still."""
    if not _H.home or not _H.grips_still:
        return False
    return all(np.linalg.norm(np.asarray(objects[g].pos, float) - p) < HOME_TOL for g, p in _H.home.items()
               if g in objects)


def _score(objects):
    # drawn up while cocked at any point: the lip bit the flange rim and the joint is open on the far side; it leaks
    # or blows off under pressure, and the gasket must be replaced, so backing off and reseating does not undo it
    if _H.cocked_tight:
        return 0.0
    if not _seated(objects):
        return 0.0  # the clamp is not square on both flanges
    if _hands_on(objects):
        return 0.0  # still holding the nut or pressing the clamp: the job isn't finished
    past = tk.past_seat_deg(objects, NUT)
    if past <= TIGHT_DEG:
        return 0.0  # square but never drawn up: the nut is loose and nothing holds the joint together
    if (-tk.gripped_deg(objects, NUT) < (past - THREAD.start) - GRIP_SLACK
            or tk.unheld_deg(objects, NUT, tightening=True) > GRIP_SLACK):
        return 0.0  # the nut was spun by a knock or a swipe, not turned with a grip
    if _H.max_past > OVER_DEG:
        return 0.25  # past 1.5 turns at some point: the gasket was squeezed into the bore
    if past < SNUG_DEG:
        return 0.5  # past contact but under half a turn: loose, it seeps under pressure
    return 1.0 if (_arms_home(objects) or not REQUIRE_ARMS_HOME) else 0.9


def success(objects):
    """Full score: the clamp square on both flanges, the wing nut 0.5 to 1.5 turns past contact, let go, and (with
    REQUIRE_ARMS_HOME) both arms home and still. Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if the wing nut was ever more than 10 degrees past contact while the clamp was tilted 0.8 degrees or more
    at that step or the one before (drawn up cocked; irreversible). 0 unless the clamp is square now (tilt under 0.8 degrees), no gripper holds the nut
    or is within 6 cm of the joint or 4 cm of the nut, and the nut is past contact. 0 if more than 90 degrees of the
    tightening was done without a gripper holding the nut and turning with it. Then 0.25 if the nut was ever more than
    1.5 turns past contact (overtight), 0.5 if it is under half a turn past contact (loose), and 1.0 from half a turn to
    1.5 turns with both arms home and still (0.9 if not, with REQUIRE_ARMS_HOME). Possible scores: 0, 0.25, 0.5, 0.9,
    1."""
    return _score(objects)


# --- Demo (left arm; the right stays home) ---------------------------------------------------------------------
NUT_XYZ = (X0 + R_B, Y0 + NUT_Y0, TOP + ZJ)
PRESS_XY = (X0 - HINGE_R - 0.0025, Y0)  # the hinge knuckle's outer half, the clamp's high side (clear of the spool)
PRESS_TOP = TOP + ZJ + CL_H  # the clamp's top when square
NUT_ROLL = -math.pi / 2  # the left hand's grip turned to point along -y, end-on to the wing nut
TURN = 900.0  # 1.5 turns to contact, then one turn past it (mid-window)
STROKE = 180.0
F_LOOSE = -0.012  # fingers 24 mm apart at the nut: clear of the 10 mm wings, but open fingers rolled flat stay off
#                    the flanges 2.5 cm away (fully open, the lower finger jams on them; sim pilot, Nov 10)
HI = 0.0  # grip yaw at the start of each stroke: wings flat match fingers closing vertically at yaw 0 or 180


def _press():
    x, y = PRESS_XY
    cocked_top = TOP + ZJ + (R_B + HINGE_R) * math.sin(math.radians(Q_COCK)) + CL_H
    return [Waypoint(2.5, (x, y, cocked_top + 0.06), FINGER_CLOSED, "over the clamp's high side", 0.0),
            Waypoint(1.5, (x, y, cocked_top + 0.013 + 0.003), FINGER_CLOSED, "fingertips on the hinge", 0.0),
            Waypoint(2.0, (x, y, PRESS_TOP + 0.013 - 0.004), FINGER_CLOSED, "press it down square", 0.0),
            Waypoint(0.8, (x, y, PRESS_TOP + 0.013 - 0.004), FINGER_CLOSED, "hold", 0.0),
            Waypoint(1.5, (x, y, PRESS_TOP + 0.06), FINGER_CLOSED, "lift off", 0.0),
            Waypoint(1.5, (x, y, PRESS_TOP + 0.06), FINGER_CLOSED, "turn the wrist", math.pi / 2)]


def _nut_strokes():
    """Left arm: point the hand along -y at the wing nut, pinch the wings (flat at the start, so the fingers close top
    and bottom), roll clockwise seen from the nut's outside (the grip's yaw going down) 180 degrees, let go, roll back
    open, and repeat; then back out along +y. A parallel grip matches the wings every 180 degrees."""
    x, y, z = NUT_XYZ
    y += THREAD.pitch * (-THREAD.start) / 360.0 + 0.004  # where the nut starts, plus 4 mm: pad tips off the ear
    out = [kit.Tilted(2.5, (x + 0.02, y + 0.025, z + 0.05), F_LOOSE, "turn the hand toward the nut",
                      math.radians(HI), NUT_ROLL),
           kit.Tilted(2.0, (x, y + 0.025, z), F_LOOSE, "in line with the wing nut", math.radians(HI), NUT_ROLL),
           kit.Tilted(1.5, (x, y, z), F_LOOSE, "fingers around the wings", math.radians(HI), NUT_ROLL)]
    left = TURN
    step = 15.0
    while left > 1e-6:
        s = min(STROKE, left)
        out.append(kit.Tilted(0.7, (x, y, z), FINGER_CLOSED, "pinch the wings", math.radians(HI), NUT_ROLL))
        n = max(1, math.ceil(s / step))
        for k in range(1, n + 1):
            out.append(kit.Tilted(0.3, (x, y, z), FINGER_CLOSED, "turn the wing nut",
                                  math.radians(HI - s * k / n), NUT_ROLL))
        out.append(kit.Tilted(0.6, (x, y, z), F_LOOSE, "let go", math.radians(HI - s), NUT_ROLL))
        left -= s
        y -= THREAD.pitch * s / 360.0  # the nut runs in along the bolt
        if left > 1e-6:
            # the open pads sit inside the wings' 36 mm span, so draw the hand back off the nut along the bolt, roll
            # back in free air, and slide back on (rolled back in place they drag the wings back with them)
            out.append(kit.Tilted(0.5, (x, y + 0.025, z), F_LOOSE, "draw back off the wings", math.radians(HI - s),
                                  NUT_ROLL))
            for k in range(1, n + 1):
                out.append(kit.Tilted(0.25, (x, y + 0.025, z), F_LOOSE, "roll back",
                                      math.radians(HI - s + s * k / n), NUT_ROLL))
            out.append(kit.Tilted(0.6, (x, y, z), F_LOOSE, "back onto the wings", math.radians(HI), NUT_ROLL))
        else:
            out.append(kit.Tilted(1.0, (x, y + 0.025, z), F_LOOSE, "back off the nut", math.radians(HI - s),
                                  NUT_ROLL))
            out.append(kit.Tilted(1.5, (x + 0.02, y + 0.025, z + 0.05), F_LOOSE, "up clear of the joint",
                                  math.radians(HI - s), NUT_ROLL))
    return out


def demo():
    """Left arm: press the clamp's hinge side down until it drops square on the flanges, then tighten the wing nut 900
    degrees (1.5 turns to contact, one past it) with an end-on regrasp loop, back off and go home. The right arm stays
    home."""
    left = _press() + _nut_strokes() + [kit.Home(2.5), kit.Home(2.0, label="hold at home")]
    total = sum(w.duration for w in left)
    return {"left": left, "right": [kit.Home(total - 2.0), kit.Home(2.0, label="hold at home")]}
