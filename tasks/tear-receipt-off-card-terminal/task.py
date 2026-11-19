"""At a shop counter (the station table), in front of the arms: a handheld card terminal (Ingenico Move/5000) lying
screen up, its printer end toward the robot's right, with the customer's receipt just printed: 10 cm of 58 mm thermal
paper standing up out of the printer, past the serrated tear bar. On the right of the counter is a black tip tray
(Winco TT-46, 4-1/2 x 6-1/2 in).

The job is a cashier's or server's: "Issue receipts, refunds, credits, or change due to customers" (O*NET 41-2011.00).
The receipt has to be torn off against the tear bar and laid in the tip tray, hands clear. Manuals for printers with
a tear bar give the technique: "Pull one edge of the paper against the tear bar at an angle as shown below, and then
tear down and across against the tear bar to tear the paper." (Printek MT2), and "Pulling up and pulling sideways
without using the tear bar can cause a paper jam." (Intermec PB3).

THE TEAR. The paper bends with its own stiffness everywhere. Along the line where it leaves the printer, just below
the bar's teeth (inside the slot; the paper in the printer is held there), it parts at its tear resistance (ISO 1974,
cross direction) where the bar's teeth notch it: where the paper at the slot touches the toothed edge and the paper
above it bends back over the bar. A tear already running continues at that resistance while the paper is still drawn
over the teeth. Elsewhere along the line it holds its tensile strength (ISO 1924-3); paper that gives at that strength
is a ragged rip, not a tear along the bar. Only tension counts: the paper drawn out of the slot or sheared along its
width; a push into the slot or a press across the paper loads nothing. The paper is Jujo KSND-48 thermal paper
(datasheet: 48 g/m^2, 55 um, tensile at least 2.1 kN/m MD, tear at least 220 mN CD). In the simulator the line is a
row of elastic ties across the width (sheetkit's tie pattern: translation springs to the terminal, read every control
step from the paper's stretch and switched off past the strength; PhysX's own breakForce is not used).

THE RECEIPT. The paper's bending stiffness and real mass on a row of thin strips across the width, hinged to bend
along its length, standing in the slot. The bending across the tear line has the same stiffness until the receipt is
off. Where it lies is measured in the tip tray's own frame, so the check is the same wherever the tray sits or turns.
Every threshold below appears once, with the same value in task.json and the README.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import guardkit as gk  # noqa: E402
import yamgeom as yg  # noqa: E402  (the arm's collision hulls, read from the environment at run time)
import yamkit as kit  # noqa: E402
import sheetkit as sk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 1200
INSTRUCTION = "Give the customer their receipt."
VERBOSE = bool(os.environ.get("RECEIPT_VERBOSE"))


def _log(msg: str) -> None:
    """Probe telemetry, only with RECEIPT_VERBOSE=1."""
    if os.environ.get("RECEIPT_VERBOSE"):
        print(msg, flush=True)

# -- the terminal: Ingenico Move/5000, 169 x 78 x 57 mm, 310 g without paper roll and battery (Ingenico user guide).
#    Its own frame: origin at its center, x along its length toward the printer end, y across, z up out of the screen.
TERM_L, TERM_W, TERM_H = 0.169, 0.078, 0.057
TERM_KG = 0.37  # 310 g bare (Ingenico user guide) plus its battery and a 40 mm roll
TERM_YAW = -math.pi / 2  # printer end toward world -y (the robot's right)
TERM_XY = (0.240, 0.030)
TERM_POS = (TERM_XY[0], TERM_XY[1], TOP + TERM_H / 2)
SLOT_X = TERM_L / 2 - 0.016  # the paper's mid-plane across the top face, 16 mm in from the printer end
LIP_T = 0.002  # m: the tear bar (printer-end side) and the plain front lip (screen side), each 2 mm thick ...
LIP_H = 0.006  # ... standing 6 mm above the top face; the tear bar's teeth are its top edge
PAPER_T = 0.001  # m: the strips' collider thickness (sheetkit; the paper itself is 55 um)
SLOT_GAP = 0.0005  # m: between each lip and the paper's collider face
TIE_DEPTH = 0.0015  # m: the line where the paper in the printer meets the receipt, below the teeth (inside the slot)
FIRST_STRIP = 2 * TIE_DEPTH  # m: the strip at the slot reaches as far above the teeth as the tie line is below them
DARK = kit.Look((0.12, 0.12, 0.13), 0.6)
SCREEN = kit.Look((0.03, 0.04, 0.05), 0.2)
KEYS = kit.Look((0.30, 0.30, 0.32), 0.6)
STEEL = kit.Look((0.70, 0.70, 0.72), 0.35, 0.8)
TEETH_Z = TERM_H / 2 + LIP_H  # the teeth, terminal frame
TIE_Z = TEETH_Z - TIE_DEPTH
# The paper leaves the printer "back over the top of the printer, not ... over the screen" (Chase, Move/5000 help)
# and is unrolled "3 cm über die Abrisskante" (Payone quick start guide): the tear edge is on the printer-end side.
BAR_FACE_X = SLOT_X + PAPER_T / 2 + SLOT_GAP  # the tear bar's face toward the paper (printer-end side, +x)
LIP_FACE_X = SLOT_X - PAPER_T / 2 - SLOT_GAP  # the plain lip's face (screen side, -x)
BAR_X = BAR_FACE_X + LIP_T / 2
LIP_X = LIP_FACE_X - LIP_T / 2
TOOTH_PITCH = 0.0025  # m: the bar's serration (visual only; the collider is the bar's straight top edge)

# -- the paper: Jujo Thermal JujoP-KSND-48 datasheet (possupply.com): grammage 48 +- 4 g/m^2 (ISO 536), thickness
#    55 +- 4 um (ISO 534), tensile strength MD >= 2.1 kN/m, CD >= 1.1 kN/m (ISO 1924-3), tear strength MD >= 200 mN,
#    CD >= 220 mN (ISO 1974). Bending stiffness D = E t^3 / 12 (1 - nu^2), E 5.5 GPa (office paper, machine
#    direction: Niskanen (ed.), Paper Physics, 2008); yield strain, springback and friction: sheetkit's SOURCES.
PAPER_W = 0.058  # Ingenico: "Width: 58 mm"
RECEIPT_L = 0.100  # m of paper above the tie line (the printed receipt)
E_MD = 5.5e9
PAPER_TH = 55e-6
THERMAL = sk.Paper(
    "48 g/m^2 thermal (Jujo KSND-48)", areal_mass=0.048, thickness=PAPER_TH,
    bending_stiffness=E_MD * PAPER_TH ** 3 / (12 * 0.91), yield_strain=0.004, springback=math.radians(35.0),
    friction=0.45, kinetic_friction=0.35, tensile_strength=2100.0,
    source="Jujo KSND-48: 48 g/m^2, 55 um, tensile MD 2.1 kN/m, tear CD 220 mN; E 5.5 GPa MD (Niskanen 2008)")
TEAR_CD = 0.220  # N: tear resistance, the tear running across the web (ISO 1974)
N_TIES = 8  # ties across the 58 mm width, one per 7.25 mm
TIE_SHARE = PAPER_W / N_TIES
TIE_TENSILE = THERMAL.tensile_strength * TIE_SHARE  # N: an intact tie (no tooth on it, no tear running into it)
TIE_STRETCH = 0.0001  # m: a tie's spring reaches the tear resistance at this stretch (k = TEAR_CD / TIE_STRETCH)
TIE_K = TEAR_CD / TIE_STRETCH
TEETH_LEAN = math.atan2(SLOT_GAP, TIE_DEPTH)  # rad: the slot strip's lean over the bar when it meets the teeth
TOOTH_CONTACT = 0.0002  # m: the strip at the slot within this of the teeth's edge: the paper is pressed onto the teeth
_n_rest = round((RECEIPT_L - FIRST_STRIP) / sk.PITCH)
HINGES = tuple(FIRST_STRIP + (RECEIPT_L - FIRST_STRIP) * k / _n_rest for k in range(_n_rest))

# Sheet frame -> terminal frame: the sheet's width (x) runs across the terminal (y), its length (y) up (z), its normal
# (z) toward the printer end (x). The receipt stands straight: 10 cm is under its self-buckling length (test_task).
R_SHEET = np.array([[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]])


def rz(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


R_TERM = rz(TERM_YAW)
TIE_MID = np.array([SLOT_X, 0.0, TIE_Z])  # the tie line's middle, terminal frame
RECEIPT_CENTER = np.asarray(TERM_POS) + R_TERM @ (TIE_MID + np.array([0.0, 0.0, RECEIPT_L / 2]))
RECEIPT_SHEET = sk.Sheet("receipt", PAPER_W, RECEIPT_L, HINGES, THERMAL, tuple(float(v) for v in RECEIPT_CENTER),
                         collider_t=PAPER_T, line_pitch=0.005, margin=0.008)
N_STRIPS = RECEIPT_SHEET.n
STRIP_KEYS = tuple(RECEIPT_SHEET.key(i) for i in range(N_STRIPS))
S0_KEY = STRIP_KEYS[0]


def _quat_xyzw(R) -> tuple:
    return tuple(float(v) for v in sk.R_quat(R))


_mech = RECEIPT_SHEET.mechanism()
RECEIPT = kit.Mechanism(_mech.name, pos=_mech.pos, size=_mech.size, mass=_mech.mass, meshes=_mech.meshes,
                        colliders=_mech.colliders, links=_mech.links, yaw=TERM_YAW, tilt_xyzw=_quat_xyzw(R_SHEET))

# Each tie: the point on the strip at the slot (its own frame: at its hinge, the top edge; the strip runs down -y) and
# the same point in the terminal's frame with the receipt as it starts.
TIE_U = tuple((k + 0.5) * TIE_SHARE - PAPER_W / 2 for k in range(N_TIES))  # across the width
S0_LEN = float(RECEIPT_SHEET.widths[0])
TIE_STRIP_PTS = tuple((u, -S0_LEN, 0.0) for u in TIE_U)
TIE_ANCHORS = tuple((SLOT_X, u, TIE_Z) for u in TIE_U)
BEND_K = THERMAL.bending_stiffness * PAPER_W / S0_LEN  # N*m/rad: the paper's bending across the tie line


def _terminal_meshes() -> tuple:
    stub = kit.Look((0.96, 0.96, 0.94), 0.8)
    return (kit.box((0.0, 0.0, 0.0), (TERM_L, TERM_W, TERM_H), DARK),
            kit.box((-0.010, 0.0, TERM_H / 2 + 0.0002), (0.070, 0.060, 0.0004), SCREEN),
            kit.box((-0.062, 0.0, TERM_H / 2 + 0.0002), (0.036, 0.060, 0.0004), KEYS),
            kit.box((BAR_X, 0.0, TERM_H / 2 + (LIP_H - 0.001) / 2), (LIP_T, PAPER_W + 0.006, LIP_H - 0.001), STEEL),
            *(kit.box((BAR_X, float(y), TEETH_Z - 0.0005), (LIP_T, TOOTH_PITCH / 2, 0.001), STEEL)
              for y in np.arange(-(PAPER_W + 0.006) / 2 + TOOTH_PITCH / 2, (PAPER_W + 0.006) / 2, TOOTH_PITCH)),
            kit.box((LIP_X, 0.0, TERM_H / 2 + LIP_H / 2), (LIP_T, PAPER_W + 0.006, LIP_H), DARK),
            # the paper still in the printer, up to the tie line (it stays when the receipt is torn off)
            kit.box((SLOT_X, 0.0, (TERM_H / 2 + TIE_Z) / 2), (0.0002, PAPER_W, TIE_Z - TERM_H / 2), stub))


TERMINAL = kit.Prop(
    "terminal", pos=TERM_POS, size=(TERM_L, TERM_W, TERM_H + 2 * LIP_H), mass=TERM_KG, yaw=TERM_YAW,
    meshes=_terminal_meshes(),
    colliders=(kit.Collider((0.0, 0.0, 0.0), size=(TERM_L, TERM_W, TERM_H)),
               kit.Collider((BAR_X, 0.0, TERM_H / 2 + LIP_H / 2), size=(LIP_T, PAPER_W + 0.006, LIP_H)),
               kit.Collider((LIP_X, 0.0, TERM_H / 2 + LIP_H / 2), size=(LIP_T, PAPER_W + 0.006, LIP_H))))

# -- the tip tray: Winco TT-46, "4-1/2 x 6-1/2" (114 x 165 mm), black plastic, 1/4 in (6 mm) high. Its own frame:
#    origin at the middle of its floor's top face, x across (114 mm), y along. ------------------------------------
TRAY_OUT = (0.114, 0.165)
TRAY_WALL = 0.002
TRAY_IN = (TRAY_OUT[0] - 2 * TRAY_WALL, TRAY_OUT[1] - 2 * TRAY_WALL)
TRAY_RIM_H = 0.00635
TRAY_FLOOR_T = 0.002
TRAY_KG = 0.08
TRAY_XY = (0.200, -0.205)
TRAY_YAW = 0.0
TRAY_POS = (TRAY_XY[0], TRAY_XY[1], TOP + TRAY_FLOOR_T)
BLACK = kit.Look((0.05, 0.05, 0.05), 0.4)


def _tray_parts() -> tuple:
    ox, oy = TRAY_OUT[0] / 2, TRAY_OUT[1] / 2
    zc = TRAY_RIM_H / 2 - TRAY_FLOOR_T / 2
    floor = ((0.0, 0.0, -TRAY_FLOOR_T / 2), (TRAY_OUT[0], TRAY_OUT[1], TRAY_FLOOR_T))
    walls = (((ox - TRAY_WALL / 2, 0.0, zc), (TRAY_WALL, TRAY_OUT[1], TRAY_RIM_H)),
             ((-ox + TRAY_WALL / 2, 0.0, zc), (TRAY_WALL, TRAY_OUT[1], TRAY_RIM_H)),
             ((0.0, oy - TRAY_WALL / 2, zc), (TRAY_OUT[0], TRAY_WALL, TRAY_RIM_H)),
             ((0.0, -oy + TRAY_WALL / 2, zc), (TRAY_OUT[0], TRAY_WALL, TRAY_RIM_H)))
    return (floor,) + walls


TRAY = kit.Prop(
    "tray", pos=TRAY_POS, size=(TRAY_OUT[0], TRAY_OUT[1], TRAY_RIM_H), mass=TRAY_KG, yaw=TRAY_YAW,
    meshes=tuple(kit.box(c, s, BLACK) for c, s in _tray_parts()),
    colliders=tuple(kit.Collider(c, size=s) for c, s in _tray_parts()))

OBJECTS = [TERMINAL, RECEIPT, TRAY]

# -- the scored thresholds ---------------------------------------------------------------------------------------------
LIE_Z = 0.015  # m: lying in the tray: every strip corner inside the rim and at most 15 mm over the floor (a receipt's
#                roll curl may lift it a little; standing up, folded over the rim or crumpled high does not pass)
OFF_COUNTER_Z = TOP - 0.05  # any strip below this has fallen off the counter
SCREEN_UP_DEG = 30.0  # the terminal knocked over: its screen past 30 degrees from level at any time
MARK_MOVE = 0.02  # m: the terminal left on its mark: its center within 2 cm of where it started ...
MARK_TURN_DEG = 10.0  # ... turned at most 10 degrees ...
MARK_REST = 0.003  # m: ... and resting on the counter: its center within 3 mm of its resting height, ...
#                     tilted at most 5 deg
MARK_TILT_DEG = 5.0
STILL_STEP = 0.0005  # m per control step (1 cm/s)
HAND_CONTACT = 0.001  # m: any part of either arm within 1 mm of the receipt or the tray touches it
FLOOR_TOL = 0.001  # m: lying in the tray: no strip corner more than 1 mm below the tray floor's top face. Paper resting
#                    on the floor has its lowest corners at the face (rest offset 0); a receipt under the tray, with
#                    the tray set down on it, has every corner at least the floor's 2 mm thickness below the face.
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
GRIPPERS = ("left_gripper", "right_gripper")


def rotm(q) -> np.ndarray:
    return sk.quat_R(q)


def strip_parts(i: int) -> tuple:
    c, s = RECEIPT_SHEET.box_local(i)
    return (gk.Block(c, s),)


STRIP_PARTS = tuple(strip_parts(i) for i in range(N_STRIPS))
TRAY_PARTS = tuple(gk.Block(c, s) for c, s in _tray_parts())


def frames(objects) -> list:
    return sk.frames(objects, RECEIPT_SHEET)


def strip_corners(objects) -> np.ndarray:
    return RECEIPT_SHEET.corners(frames(objects))


# -- the tear line: the paper's strengths across the slot, from its stretch there ----------------------------------
def _term_frame(objects):
    t = objects["terminal"]
    return np.asarray(t.pos, float), rotm(t.quat_xyzw)


def tie_loads(objects) -> np.ndarray:
    """Each tie's tension (N): spring x the tensile part of its stretch, in the strip at the slot's own frame: drawn
    out of the slot along the paper (+y) and sheared along the width (x). A push into the slot (compression) or a
    press across the paper's thickness (z) is carried by the slot and the printer, not by the paper: it loads no
    tie."""
    pt, Rt = _term_frame(objects)
    s0 = objects[S0_KEY]
    ps, Rs = np.asarray(s0.pos, float), rotm(s0.quat_xyzw)
    out = np.zeros(N_TIES)
    for k in range(N_TIES):
        a = pt + Rt @ np.asarray(TIE_ANCHORS[k])
        p = ps + Rs @ np.asarray(TIE_STRIP_PTS[k])
        d = Rs.T @ (p - a)
        out[k] = TIE_K * math.hypot(max(0.0, float(d[1])), float(d[0]))
    return out


def tie_stretch(objects) -> list:
    """Each tie's full 3-D stretch (m), tension or not (telemetry: a connected tie stays within microns at rest)."""
    pt, Rt = _term_frame(objects)
    s0 = objects[S0_KEY]
    ps, Rs = np.asarray(s0.pos, float), rotm(s0.quat_xyzw)
    return [float(np.linalg.norm(ps + Rs @ np.asarray(TIE_STRIP_PTS[k]) - pt - Rt @ np.asarray(TIE_ANCHORS[k])))
            for k in range(N_TIES)]


def _box_dist(p_local, center, size) -> float:
    d = np.abs(np.asarray(p_local) - np.asarray(center)) - np.asarray(size) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def teeth_gaps(objects) -> np.ndarray:
    """Per tie: the gap (m) between the strip at the slot and the tear bar's toothed edge there."""
    pt, Rt = _term_frame(objects)
    s0 = objects[S0_KEY]
    ps, Rs = np.asarray(s0.pos, float), rotm(s0.quat_xyzw)
    c, s = RECEIPT_SHEET.box_local(0)
    return np.array([_box_dist(Rs.T @ (pt + Rt @ np.array([BAR_FACE_X, u, TEETH_Z]) - ps), c, s) for u in TIE_U])


def wrapped(objects) -> bool:
    """The paper above the slot strip is drawn over the bar: strip s1 leans toward the bar past the angle at which
    the slot strip first meets the teeth (the paper bends over the bar's edge, not just leans on it)."""
    _, Rt = _term_frame(objects)
    d = Rt.T @ rotm(objects[STRIP_KEYS[1]].quat_xyzw) @ np.array([0.0, 1.0, 0.0])
    return math.atan2(float(d[0]), float(d[2])) >= TEETH_LEAN


def on_teeth(objects) -> np.ndarray:
    """Per tie: the paper is drawn over the teeth there: the strip at the slot touches the tear bar's toothed edge
    (within TOOTH_CONTACT; the edge is the bar's top inner corner, along the width) and the paper above it is wrapped
    over the bar (`wrapped`)."""
    return (teeth_gaps(objects) <= TOOTH_CONTACT) & wrapped(objects)


class Tear:
    """The ties' state in one trial: broken flags, peak loads, the step each broke, and whether it broke notched (on
    the teeth or at a running tear's tip). `disable(k)` switches tie k's joint off in the simulator."""

    def __init__(self):
        self.episode = kit.episode()
        self.broken = [False] * N_TIES
        self.peak = [0.0] * N_TIES
        self.at = [None] * N_TIES
        self.notched_at_break = [False] * N_TIES
        self.n = 0

    def strength(self, k: int, teeth) -> tuple:
        """A tie gives at the tear resistance while the paper there is drawn over the teeth, or while it is at the tip
        of a tear already running AND the paper is still drawn over the teeth (a started tear pulled any other way
        wanders off the bar: it stays at full strength here, and the trial cannot score a tear along the bar)."""
        tip = (k > 0 and self.broken[k - 1]) or (k < N_TIES - 1 and self.broken[k + 1])
        notched = bool(teeth[k]) or (tip and bool(np.any(teeth)))
        return (TEAR_CD if notched else TIE_TENSILE), notched

    def step(self, objects, disable=None) -> None:
        self.n += 1
        loads, teeth = tie_loads(objects), on_teeth(objects)
        for k in sorted(range(N_TIES), key=lambda i: -loads[i]):
            if self.broken[k]:
                continue
            self.peak[k] = max(self.peak[k], float(loads[k]))
            limit, notched = self.strength(k, teeth)
            if loads[k] > limit:
                self.broken[k], self.at[k], self.notched_at_break[k] = True, self.n, notched
                if disable is not None:
                    disable(k)
                _log(f"[receipt] tie {k} broke at {loads[k]:.3g} N ({'notched' if notched else 'intact'})")

    @property
    def count(self) -> int:
        return sum(self.broken)

    @property
    def off(self) -> bool:
        return all(self.broken)

    @property
    def clean(self) -> bool:
        """Off along the bar: every tie broke on the teeth or at the running tear's tip (a tie torn intact is a
        ragged rip, not a tear along the bar)."""
        return self.off and all(self.notched_at_break)


# -- the trial's score -------------------------------------------------------------------------------------------------
def lying_in_tray(objects) -> bool:
    """The tray resting flat, right side up, and every strip corner inside its rim's inner outline, no more than
    FLOOR_TOL below its floor's top face and at most LIE_Z over it (the tray's own frame): the receipt lies in the tray,
    not under it."""
    t = objects["tray"]
    Rt, pt = rotm(t.quat_xyzw), np.asarray(t.pos, float)
    if float(Rt[2, 2]) < math.cos(math.radians(MARK_TILT_DEG)):
        return False
    q = (strip_corners(objects).reshape(-1, 3) - pt) @ Rt
    return bool(np.all(np.abs(q[:, 0]) <= TRAY_IN[0] / 2) and np.all(np.abs(q[:, 1]) <= TRAY_IN[1] / 2)
                and np.all(q[:, 2] >= -FLOOR_TOL) and np.all(q[:, 2] <= LIE_Z))


def terminal_up(objects) -> bool:
    t = objects["terminal"]
    return float(rotm(t.quat_xyzw)[2, 2]) >= math.cos(math.radians(SCREEN_UP_DEG)) and float(t.pos[2]) > OFF_COUNTER_Z


def terminal_on_mark(objects) -> bool:
    t = objects["terminal"]
    p = np.asarray(t.pos, float)
    R = rotm(t.quat_xyzw)
    turn = abs(math.degrees(math.atan2(R[1, 0], R[0, 0]) - TERM_YAW)) % 360.0
    turn = min(turn, 360.0 - turn)
    resting = abs(float(p[2]) - TERM_POS[2]) <= MARK_REST and float(R[2, 2]) >= math.cos(math.radians(MARK_TILT_DEG))
    return float(np.hypot(p[0] - TERM_POS[0], p[1] - TERM_POS[1])) <= MARK_MOVE and turn <= MARK_TURN_DEG and resting


def tray_on_mark(objects) -> bool:
    """The tip tray within MARK_MOVE of where it started (its outline on the counter)."""
    p = np.asarray(objects["tray"].pos, float)
    return float(np.hypot(p[0] - TRAY_POS[0], p[1] - TRAY_POS[1])) <= MARK_MOVE


# -- hands clear: every arm link's collision hulls (shoulder to fingertips, both arms), read at run time from the
# competition environment's robot model (yamgeom: surface points 0.5 mm apart in each link's own frame), posed each
# control step from the simulator's link poses, against the receipt's strips and the tray's colliders. guardkit's box
# model of the hand (palm and fingers) is used only offline (the tests), where there are no link poses; in the
# simulator a missing robot or link is an error, never a fallback.
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = tuple(kit.GRIPPERS.values())
LINK_POSES: dict = {}  # "right_robot/link4" -> (pos, R) in the env frame, this control step (empty offline)
_IN_SIM = {"on": False, "told": False}  # on: this module is running in the simulator (set when the scene is patched)


def _blocks_gap(q: np.ndarray, parts) -> np.ndarray:
    """Distance (m, 0 inside) from object-frame points q to the nearest of the boxes `parts`."""
    out = np.full(len(q), np.inf)
    for b in parts:
        d = np.maximum(np.abs(q - np.asarray(b.center, float)) - np.asarray(b.size, float) / 2, 0.0)
        out = np.minimum(out, np.linalg.norm(d, axis=1))
    return out


def link_gap(objects, key: str, obj: str, parts, poses=None) -> float:
    """The smallest gap (m, 0 for overlap) between arm link `key` ("right_robot/link4") as posed and object `obj`'s
    boxes `parts`; anything over 1 cm reads as just over 1 cm."""
    poses = LINK_POSES if poses is None else poses
    o, R = (np.asarray(v, float) for v in poses[key])
    co, Ro = np.asarray(objects[obj].pos, float), rotm(objects[obj].quat_xyzw)
    _, c, r = yg.cloud(key.split("/")[1])
    centre = (o + R @ c - co) @ Ro
    if float(_blocks_gap(centre[None, :], parts)[0]) - r > 0.01:
        return 0.01 + 1e-6
    q = yg.near(key.split("/")[1], lambda x: (o + x @ R.T - co) @ Ro, lambda x: _blocks_gap(x, parts), 0.01)
    return float(_blocks_gap(q, parts).min()) if len(q) else 0.01 + 1e-6


def hands_on(objects, poses=None) -> bool:
    """Some part of either arm (in the sim any link from the shoulder to the fingertips, by its collision hulls;
    offline the hand's boxes) touches the receipt or the tray (within HAND_CONTACT)."""
    poses = LINK_POSES if poses is None else poses
    if not poses:
        if _IN_SIM["on"]:
            raise RuntimeError("[receipt] hands-clear: no arm link poses read from the simulator")
        return any(gk.touching(objects, STRIP_KEYS[i], STRIP_PARTS[i], tol=HAND_CONTACT, grippers=GRIPPERS)
                   for i in range(N_STRIPS)) or \
            bool(gk.touching(objects, "tray", TRAY_PARTS, tol=HAND_CONTACT, grippers=GRIPPERS))
    targets = [(STRIP_KEYS[i], STRIP_PARTS[i]) for i in range(N_STRIPS) if STRIP_KEYS[i] in objects]
    targets.append(("tray", TRAY_PARTS))
    return any(link_gap(objects, k, obj, parts, poses) <= HAND_CONTACT for k in poses for obj, parts in targets)


def _read_links(base) -> None:
    """Every arm link's pose (env frame) this control step, into LINK_POSES. A robot or link the simulator does not
    have is an error: the hands-clear check never falls back to the hand boxes in a run."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"[receipt] hands-clear: no robot {robot!r} in the scene") from e
        names = list(art.body_names)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"[receipt] hands-clear: {robot} has no link(s) {missing} (bodies {names})")
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
    want = {f"{r}/{k}" for r in ROBOTS for k in ARM_LINKS}
    if set(LINK_POSES) != want:
        raise RuntimeError(f"[receipt] hands-clear: link poses {sorted(LINK_POSES)} != {sorted(want)}")
    if not _IN_SIM["told"]:
        _IN_SIM["told"] = True
        sys.stderr.write(f"[receipt] hands-clear: {len(LINK_POSES)} link poses from the sim, hull check on\n")
        sys.stderr.flush()


def tier(clean: bool, started: bool, lying: bool) -> float:
    """0.75 torn off along the bar and lying in the tray, 0.5 torn off along the bar, 0.25 a tear started but not
    through, or a ragged rip (a tie torn intact), 0 nothing torn."""
    if clean:
        return 0.75 if lying else 0.5
    return 0.25 if started else 0.0


class Ledger:
    """One trial: the tear, the latches and the still/hold count."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.tear = Tear()
        self.fell = False
        self.knocked = False
        self.last = None
        self.good = 0
        self.lying = False
        self.mark = True

    def step(self, objects) -> None:
        self.tear.step(objects, PIN.disable if PIN.live else None)
        corners = strip_corners(objects).reshape(-1, 3)
        if float(corners[:, 2].min()) < OFF_COUNTER_Z:
            self.fell = True
        if not terminal_up(objects):
            self.knocked = True
        if self.tear.off:
            PIN.release()
        self.lying = self.tear.clean and lying_in_tray(objects)
        self.mark = terminal_on_mark(objects) and tray_on_mark(objects)
        com = corners.mean(axis=0)
        still = self.last is not None and float(np.linalg.norm(com - self.last)) <= STILL_STEP
        self.last = com
        done = self.lying and self.mark and still and not self.fell and not self.knocked and not hands_on(objects)
        self.good = self.good + 1 if done else 0

    def tier(self) -> float:
        if self.fell or self.knocked:
            return 0.0
        return tier(self.tear.clean, self.tear.count > 0, self.lying)


_L = None


def _update(objects):
    global _L
    if _L is None or _L.episode != kit.episode():
        _L = Ledger()
        PIN.new_trial()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else ("obj", id(objects))
    if key == _L.key:
        return
    _L.key = key
    PIN.capture()
    _L.step(objects)
    if VERBOSE and _L.tear.n % 5 == 0:
        d0 = _term_frame(objects)[1].T @ rotm(objects[S0_KEY].quat_xyzw) @ np.array([0.0, 1.0, 0.0])
        lean = math.degrees(math.atan2(float(d0[0]), float(d0[2])))
        on = [i for i in range(N_STRIPS)
              if gk.touching(objects, STRIP_KEYS[i], STRIP_PARTS[i], grippers=("right_gripper",))]
        moved = np.round(np.asarray(objects["terminal"].pos) - np.asarray(TERM_POS), 4).tolist()
        top = 1000 * (float(strip_corners(objects)[-1][:, 2].max()) - TOP)
        _log(f"[receipt] step {_L.tear.n}: ties broken {_L.tear.count}/{N_TIES}, max 3-D tie stretch "
             f"{1e6 * max(tie_stretch(objects)):.0f} um, tension loads {[round(v, 3) for v in tie_loads(objects)]}, "
             f"teeth gap mm {[round(1000 * v, 2) for v in teeth_gaps(objects)]}, s0 lean {lean:.1f} deg over the bar, "
             f"right opening {kit.opening(objects, 'right_gripper'):.3f} {_right_pose(objects)} on strips {on}, "
             f"terminal {moved}, top strip z {top:.0f} mm, lying {_L.lying}, mark {_L.mark}, score {_L.tier()}")


def _right_pose(objects) -> str:
    """Telemetry: the right tool point in the terminal's frame relative to the paper's top middle (mm: x across the
    paper's thickness, y along its width, z), and the grip's yaw about the vertical (deg)."""
    pt, Rt = _term_frame(objects)
    g = objects["right_gripper"]
    d = Rt.T @ (np.asarray(g.pos, float) - pt) - np.array([SLOT_X, 0.0, TIE_Z + RECEIPT_L])
    Rg = rotm(g.quat_xyzw)
    yaw = math.degrees(math.atan2(Rg[1, 0], Rg[0, 0]))
    return f"tcp vs paper top mm {np.round(1000 * d, 1).tolist()} yaw {yaw:.1f}"


def success(objects):
    """The receipt torn off along the bar and lying in the tip tray, both arms clear of it and the tray, still,
    the terminal on its mark screen up: this step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1 and _L.tier() == 0.75


def score(objects):
    """1.0: the receipt torn off along the tear bar (every tie across its width broken; nothing left hanging from the
    printer) and lying in the tip tray (right side up, every strip corner inside the rim, none more than 15 mm over
    its floor or more than 1 mm below it), both arms clear of the receipt and the tray, still for 1 s, the terminal
    and the tray on their marks. 0.75: torn off and lying in the tray otherwise. 0.5: torn off, not lying in the
    tray. 0.25: a tear started and stalled part-way across (a ragged half-tear left in the printer). 0: nothing torn,
    including doing nothing. The whole trial scores 0 if the receipt falls off the counter or the terminal is knocked
    over."""
    _update(objects)
    s = _L.tier()
    if s == 0.75 and _L.mark and _L.good >= HOLD_STEPS:
        s = 1.0
    return s


# -- simulator side: author the ties and the slot's bending joint, switch ties off as they break ---------------------
class _Slot:
    """The receipt's ties to the terminal (one D6 joint per tie, terminal body to the strip at the slot, translations
    on implicit springs of TIE_K, rotations free) and the paper's bending across the tie line (one D6 joint, a spring
    of BEND_K about the width toward the paper standing straight out of the slot, translations free). Authored once
    on the stage just before physics starts; a broken tie is switched off (physics:jointEnabled), the bending joint
    when the last tie goes. A new trial switches them all back on. Offline (no simulator) none of this runs."""

    def __init__(self):
        self.paths, self.bend, self.live, self.bend_off = [], None, False, False

    def author(self, stage) -> None:
        from pxr import Gf, UsdPhysics

        terms = [prim for prim in stage.Traverse() if str(prim.GetPath()).endswith("/terminal")
                 and prim.HasAPI(UsdPhysics.RigidBodyAPI)]
        terms.sort(key=lambda prim: ("/env_0/" not in str(prim.GetPath()), str(prim.GetPath())))  # env_0 first
        term = terms[0] if terms else None
        rec = stage.GetPrimAtPath(str(term.GetPath()).rsplit("/", 1)[0] + "/receipt/s0") if term else None
        if rec is not None and not (rec.IsValid() and rec.HasAPI(UsdPhysics.RigidBodyAPI)):
            rec = None
        if term is None or rec is None:
            print(f"[receipt] slot joints NOT authored (terminal {term}, strip {rec})", file=sys.stderr, flush=True)
            return
        q = sk.R_quat(R_SHEET)  # x, y, z, w
        rot0 = Gf.Quatf(float(q[3]), float(q[0]), float(q[1]), float(q[2]))
        root = str(term.GetPath()).rsplit("/", 1)[0]
        for k in range(N_TIES):
            jt = UsdPhysics.Joint.Define(stage, f"{root}/receipt_tie_{k}")
            self._frames(jt, term, rec, TIE_ANCHORS[k], rot0, TIE_STRIP_PTS[k])
            for ax in ("transX", "transY", "transZ"):
                self._drive(jt.GetPrim(), ax, TIE_K, 2 * math.sqrt(TIE_K * RECEIPT_SHEET.mass(0)))
            self.paths.append(str(jt.GetPath()))
        bj = UsdPhysics.Joint.Define(stage, f"{root}/receipt_bend")
        self._frames(bj, term, rec, TIE_MID, rot0, (0.0, -S0_LEN, 0.0))
        inertia = RECEIPT_SHEET.total_mass * RECEIPT_L ** 2 / 3
        # USD angular drives are per degree
        self._drive(bj.GetPrim(), "rotX", BEND_K * math.pi / 180, 2 * math.sqrt(BEND_K * inertia) * math.pi / 180)
        self.bend = str(bj.GetPath())
        _log(f"[receipt] slot: {N_TIES} ties (k {TIE_K:.4g} N/m, notched {TEAR_CD} N, intact {TIE_TENSILE:.3g} N) "
              f"and the bending joint (k {BEND_K:.3g} N*m/rad) on {rec.GetPath()}")

    @staticmethod
    def _frames(jt, term, rec, anchor, rot0, pt) -> None:
        from pxr import Gf

        jt.CreateBody0Rel().SetTargets([term.GetPath()])
        jt.CreateBody1Rel().SetTargets([rec.GetPath()])
        jt.CreateLocalPos0Attr(Gf.Vec3f(*(float(v) for v in anchor)))
        jt.CreateLocalRot0Attr(rot0)  # the strip at the slot standing straight, in the terminal's frame
        jt.CreateLocalPos1Attr(Gf.Vec3f(*(float(v) for v in pt)))
        jt.CreateLocalRot1Attr(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
        jt.CreateExcludeFromArticulationAttr(True)
        jt.CreateCollisionEnabledAttr(False)
        jt.CreateJointEnabledAttr(True)

    @staticmethod
    def _drive(prim, axis, k, c) -> None:
        from pxr import UsdPhysics

        dr = UsdPhysics.DriveAPI.Apply(prim, axis)
        dr.CreateTypeAttr("force")
        dr.CreateStiffnessAttr(float(k))
        dr.CreateDampingAttr(float(c))
        dr.CreateTargetPositionAttr(0.0)
        dr.CreateMaxForceAttr(1e6)

    @staticmethod
    def _enable(path, on) -> None:
        import omni.usd
        from pxr import UsdPhysics

        stage = omni.usd.get_context().get_stage()
        UsdPhysics.Joint(stage.GetPrimAtPath(path)).GetJointEnabledAttr().Set(bool(on))

    def disable(self, k: int) -> None:
        self._enable(self.paths[k], False)

    def release(self) -> None:
        if self.live and self.bend and not self.bend_off:
            self._enable(self.bend, False)
            self.bend_off = True
            _log("[receipt] torn off: bending joint off")

    def new_trial(self) -> None:
        if self.live:
            for p in self.paths:
                self._enable(p, True)
            if self.bend:
                self._enable(self.bend, True)
        self.bend_off = False

    def capture(self) -> None:
        self.live = bool(self.paths)

    def place(self, scene) -> None:
        """At a scene reset, before the first physics step (after any start jitter): rejoin every tie and the bending
        joint, and put the receipt where the terminal now holds it (`receipt_root_pose`), at rest. A start placement,
        not a pull during the trial."""
        import torch

        if self.paths:
            for p in self.paths:
                self._enable(p, True)
            self._enable(self.bend, True)
        self.bend_off = False
        term, rec = scene["terminal"], scene["receipt"]
        tp = getattr(term.data.root_pose_w, "torch", term.data.root_pose_w)  # (1, 7): position, quaternion xyzw
        v = tp[0].detach().cpu().numpy().astype(np.float64)
        pos, R = receipt_root_pose(v[:3], rotm(v[3:7]))
        pose = torch.tensor([[*pos, *quat_from_R(R)]], dtype=tp.dtype, device=tp.device)
        rec.write_root_pose_to_sim_index(root_pose=pose)
        rec.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), dtype=tp.dtype, device=tp.device))
        spring_hinges(rec)
        _log(f"[receipt] reset: receipt placed at the terminal's slot, ties rejoined (terminal {np.round(v[:3], 4)})")


def spring_hinges(art) -> None:
    """Every hinge of the receipt straight and sprung with its paper stiffness (its crease law's drive toward flat)
    from the first physics step after a reset. The scene's settle runs before any control step, and yamkit applies
    the hinge laws only at control steps: without this the hinges are limp through the settle and the receipt flops
    over and takes a set before the trial starts."""
    names = list(art.joint_names)
    for lk in RECEIPT.links:
        j = names.index(f"{lk.name}_joint")
        kit._put_joint(art, j, lk.start)
        kit._set_drive(art, j, target=0.0, stiffness=lk.law.k, effort=0.0, damping=lk.law.c)


def quat_from_R(R) -> tuple:
    """Rotation matrix to (x, y, z, w), Shepperd's method (stable near half turns)."""
    R = np.asarray(R, float)
    t = float(np.trace(R))
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        q = ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, s / 4)
    else:
        i = int(np.argmax(np.diag(R)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = math.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k]) * 2
        v = [0.0, 0.0, 0.0]
        v[i] = s / 4
        v[j] = (R[j, i] + R[i, j]) / s
        v[k] = (R[k, i] + R[i, k]) / s
        q = (v[0], v[1], v[2], (R[k, j] - R[j, k]) / s)
    return tuple(float(x) for x in q)


ROOT_IN_TERM = R_TERM.T @ (np.asarray(RECEIPT_CENTER) - np.asarray(TERM_POS))  # the receipt's root, terminal frame


def receipt_root_pose(term_pos, term_R) -> tuple:
    """Where the receipt's root strip starts for a terminal at (term_pos, term_R): standing straight in the slot."""
    term_R = np.asarray(term_R, float)
    return np.asarray(term_pos, float) + term_R @ ROOT_IN_TERM, term_R @ R_SHEET


PIN = _Slot()


def _hook_sim_reset() -> None:
    try:
        from isaaclab.sim import SimulationContext as cls
    except Exception:  # noqa: BLE001  (offline: no simulator)
        return
    if getattr(cls.reset, "_receipt", False):
        return
    plain = cls.reset

    def reset(self, *args, **kwargs):
        if not PIN.paths:
            try:
                PIN.author(self.stage)
            except Exception as e:  # noqa: BLE001  (the log shows it; the trial then cannot tear)
                print(f"[receipt] authoring the slot joints failed: {e!r}", flush=True)
        return plain(self, *args, **kwargs)

    reset._receipt = True
    cls.reset = reset
    try:
        from isaaclab.scene import InteractiveScene
    except Exception:  # noqa: BLE001
        return
    plain_scene_reset, plain_write = InteractiveScene.reset, InteractiveScene.write_data_to_sim

    def scene_reset(self, *args, **kwargs):
        out = plain_scene_reset(self, *args, **kwargs)
        scene_was_reset(self)
        return out

    def write_data_to_sim(self, *args, **kwargs):
        before_physics(self)
        return plain_write(self, *args, **kwargs)

    InteractiveScene.reset, InteractiveScene.write_data_to_sim = scene_reset, write_data_to_sim
    try:
        from isaaclab.envs import DirectRLEnv, ManagerBasedEnv
    except Exception:  # noqa: BLE001
        return
    for env_cls in (ManagerBasedEnv, DirectRLEnv):
        env_cls.reset = inside_env_reset(env_cls.reset)


# The reset placement must run at the first physics step AFTER the env's reset has returned: Isaac Lab's env reset
# itself writes the scene once (ManagerBasedEnv.reset: scene.reset, then scene.write_data_to_sim), and only after it
# returns do yamkit's reset wrapper (installed lazily at the first control step; it sets every law-held joint limp)
# and yam_arena's start jitter run. Placing and springing inside the env reset would be undone on every later trial
# and would ignore the jitter. So: armed by the scene reset, skipped while an env reset is running, done at the next
# write (the settle's first physics step), whatever wrappers sit around the env's reset.
_RESET = {"depth": 0}


def inside_env_reset(plain):
    """Wrap an env class's reset so the scene writes inside it are known to be inside it."""
    if getattr(plain, "_receipt", False):
        return plain

    def reset(self, *args, **kwargs):
        _RESET["depth"] += 1
        try:
            return plain(self, *args, **kwargs)
        finally:
            _RESET["depth"] -= 1

    reset._receipt = True
    return reset


def scene_was_reset(scene) -> None:
    scene._receipt_pending = True


def before_physics(scene) -> None:
    """At a scene write: if a reset is pending and no env reset is running, place and spring the receipt now."""
    if getattr(scene, "_receipt_pending", False) and _RESET["depth"] == 0:
        scene._receipt_pending = False
        try:
            PIN.place(scene)
        except Exception as e:  # noqa: BLE001
            print(f"[receipt] reset placement failed: {e!r}", file=sys.stderr, flush=True)


_kit_patch = kit._patch


def _patch_capture(scene):
    _kit_patch(scene)
    _hook_sim_reset()
    if not getattr(scene, "_receipt_links", False):  # read the arm links' poses with each step's object states
        scene._receipt_links = True
        _IN_SIM["on"] = True
        yg.warm(ARM_LINKS)  # read and check the hulls now (the app is up), not inside a trial's step
        states = scene.object_states

        def object_states(env, names):
            out = states(env, names)
            _read_links(env.unwrapped)
            return out

        scene.object_states = object_states


kit._patch = _patch_capture
if "yam_arena.scene" in sys.modules:
    _patch_capture(sys.modules["yam_arena.scene"])


# -- scripted demo: open loop from the fixed layout -------------------------------------------------------------------
# Left hand: fingers closed into one blunt tip, presses lightly on the terminal's keypad end until the receipt is
# off. Right hand: fingers fully open across the paper's thickness, comes over the standing receipt above its top edge,
# straight down over it, and pinches it near its top edge, off-center toward its near edge; then swings the pinch
# over the teeth toward the printer end and down, drawing the paper tight over the bar, and on down and across: the
# tear starts at the near edge and runs across. It carries the receipt to the tip tray, touches its hanging end down at
# one end of the tray and lays it along the tray while lowering, lets go and lifts clear.
WORLD_R = R_TERM
TEETH_W = np.asarray(TERM_POS) + WORLD_R @ np.array([BAR_FACE_X, 0.0, TEETH_Z])  # the teeth's edge, middle, world
TO_BAR = WORLD_R @ np.array([1.0, 0.0, 0.0])  # world direction from the slot over the bar, toward the printer end
ACROSS = WORLD_R @ np.array([0.0, 1.0, 0.0])  # world direction along the paper's width (+x here)
PINCH_UP = 0.085  # m: the pinch's height above the teeth, near the receipt's top edge: little paper above the pads
#                   to fold over a finger, and the torn receipt hangs below the pinch to be laid down
PINCH_OFF = -0.012  # m along ACROSS: the pads' middle toward the paper's near edge
PAPER_YAW = 0.0  # yam_arena: yaw 0 closes the fingers along world y, across the paper's thickness here
CARRY_Z = TOP + 0.150
TIP_PAST_TCP = 0.0128  # m: the fingertips reach this far past the tool point (guardkit's tip box)
RECEIPT_TOP_Z = TOP + TERM_H + TIE_Z - TERM_H / 2 + RECEIPT_L  # the standing receipt's top edge
APPROACH_Z = RECEIPT_TOP_Z + TIP_PAST_TCP + 0.025  # the right hand comes over the receipt with its fingertips 25 mm
#                                                   above its top edge, clear of it on the way over
PRESS_INTO = 0.0015  # m: the left hand's closed fingertips aim this far into the terminal's top face (a light press)
TRAY_DROP_Z = 0.017  # m over the tray floor: where the pinch opens (fingertips 4 mm over the floor)
TRAY_LAY = 0.050  # m: the receipt is laid along the tray's length, from +y of its middle toward -y
TRAY_TOUCH_Z = 0.095  # m over the tray floor: the hanging receipt's end just reaches the floor
# The swing: (degrees over the teeth toward the printer end, extra radius past the paper's length, meters across).
# The last two draw past the paper's length: the paper cannot follow, so the arm pulls it tight across the tear
# line.
SWING = ((-30.0, 0.0, 0.000), (-55.0, 0.004, 0.006), (-62.0, 0.012, 0.016), (-64.0, 0.020, 0.030))


def pinch_point(bend_deg: float, extra: float = 0.0, across: float = 0.0) -> np.ndarray:
    """The pinch with the paper straight from the teeth, bent bend_deg over the bar (negative = toward the printer end),
    PINCH_UP + extra long, shifted `across` along the width."""
    b = math.radians(-bend_deg)
    r = PINCH_UP + extra
    return (TEETH_W + r * (math.cos(b) * np.array([0.0, 0.0, 1.0]) + math.sin(b) * TO_BAR)
            + (PINCH_OFF + across) * ACROSS)


def _w(d, pos, f, label, yaw=PAPER_YAW):
    return Waypoint(float(d), tuple(float(v) for v in pos), float(f), label, float(yaw))


def right_plan() -> list:
    p0 = pinch_point(0.0) + (SLOT_X - BAR_FACE_X) * (WORLD_R @ np.array([1.0, 0.0, 0.0]))  # on the paper's mid-plane
    wide, shut = FINGER_OPEN, FINGER_CLOSED  # fully open over the receipt: room for the hand's tracking error
    tray_hi = np.array([TRAY_XY[0], TRAY_XY[1] + TRAY_LAY, TOP + TRAY_FLOOR_T + TRAY_TOUCH_Z])
    tray_lo = np.array([TRAY_XY[0], TRAY_XY[1] - TRAY_LAY + 0.02, TOP + TRAY_FLOOR_T + TRAY_DROP_Z])
    plan = [_w(3.5, (p0[0], p0[1], APPROACH_Z), wide, "over the receipt"),
            _w(2.0, p0, wide, "down over it, fingers either side"),
            _w(1.5, p0, shut, "pinch it near its edge")]
    labels = ("over the teeth", "down over the bar", "down and across: tear", "on across")
    for (b, extra, across), lab in zip(SWING, labels):
        plan.append(_w(1.0, pinch_point(b, extra, across), shut, lab))
    pe = pinch_point(*SWING[-1])
    plan += [_w(0.5, pe, shut, "let the tear run out"),
             _w(1.5, (pe[0], pe[1], CARRY_Z), shut, "lift the receipt"),
             _w(2.0, (tray_hi[0], tray_hi[1], CARRY_Z), shut, "over the tip tray's far end"),
             _w(1.5, tray_hi, shut, "touch its hanging end down"),
             _w(2.0, tray_lo, shut, "lay it along the tray"),
             _w(1.2, tray_lo, wide, "let go"),
             _w(1.2, (tray_lo[0], tray_lo[1], CARRY_Z), wide, "lift clear"),
             _w(1.5, (0.20, -0.32, CARRY_Z), wide, "out of the way"),
             _w(3.0, (0.20, -0.32, CARRY_Z), wide, "wait")]
    return plan


TEAR_DONE_S = 3.5 + 2.0 + 1.5 + 1.0 * len(SWING) + 0.5  # the right hand's plan up to "let the tear run out"
KEYPAD_W = np.asarray(TERM_POS) + WORLD_R @ np.array([-0.060, 0.0, 0.0])  # over the keypad end


def left_plan() -> list:
    """Fingers closed into one blunt tip, a light press on the keypad end, as a thumb steadies it: a press holds the
    terminal down without pushing it sideways."""
    x, y = float(KEYPAD_W[0]), float(KEYPAD_W[1])
    zp = TOP + TERM_H + TIP_PAST_TCP - PRESS_INTO
    shut = FINGER_CLOSED
    return [_w(3.0, (x, y, CARRY_Z), shut, "over the keypad end"),
            _w(2.0, (x, y, zp + 0.010), shut, "down to it"),
            _w(1.2, (x, y, zp), shut, "steady the terminal"),
            _w(TEAR_DONE_S + 0.5 - 6.2, (x, y, zp), shut, "steady it through the tear"),
            _w(1.5, (x, y, CARRY_Z), shut, "lift away"),
            _w(12.0, (x, y, CARRY_Z), shut, "wait")]


def demo():
    """Left hand steadies the terminal; right hand tears the receipt over the bar and lays it in the tip tray."""
    return {"left": left_plan(), "right": right_plan()}
