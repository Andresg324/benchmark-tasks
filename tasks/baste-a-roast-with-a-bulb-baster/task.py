"""On a kitchen counter (the station table), in front of the arms: a roast sits in the middle of a 13 x 9 in roasting
pan with 3 cups of juices around it, and a bulb baster (OXO, 40 mL) lies on the counter to the left of the pan.

The job is to baste the roast: squeeze the bulb, dip the tip into the juices, let the bulb open so it draws them up,
carry the baster over the roast and squeeze the juices out onto its top, without dripping or squirting them onto the
counter, then set the baster back down on the counter.

The baster is a rigid tube with a bulb whose wall is four sprung panels on joints; a hand squeezes the bulb only by
pushing those panels in (read from their joints). The suction is bulbkit's stated law (bulbkit.py): Boyle's law for the
air trapped in the bulb and the tube, and the sharp-edged orifice law at the tip's opening, driven by the pressure
across it: the inside pressure set by the panels' push and the air left inside, and outside the opening, the
juices' depth over it. The bulb draws juices up only when the air inside is below the pressure at the submerged
opening, which takes air pushed out first (a squeeze) and the bulb opening again (a release) with the tip under the
surface. What leaves the tip in the air falls as a jet from the tip (liquidkit.ballistic) onto the roast's top, back
into the pan, or onto the counter.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402

import bulbkit as bk  # noqa: E402
import capkit as ck  # noqa: E402
import gripkit  # noqa: E402
import squeezekit as sq  # noqa: E402
import yamgeom as yg  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402



def _oval(rows, look, t0=-math.pi, t1=math.pi, m=48, shift=(0.0, 0.0, 0.0)) -> "kit.Mesh":
    """A drawn-only surface (no collider) through rings of rounded-rectangle sections: `rows` are (u, a, b, n) with u
    the position along the shape's axis and the ring a superellipse of half widths a, b and exponent n (2: an ellipse,
    higher: squarer), along x, in y and z. From angle t0 to t1 (a full turn closes the ring); `shift` is subtracted
    from every point (a link's anchor)."""
    full = abs((t1 - t0) - 2 * math.pi) < 1e-9
    k = m if full else m + 1
    pts = []
    for u, a, b, n in rows:
        for j in range(k):
            t = t0 + (t1 - t0) * j / m
            c, sn = math.cos(t), math.sin(t)
            pts.append((u, a * math.copysign(abs(c) ** (2.0 / n), c), b * math.copysign(abs(sn) ** (2.0 / n), sn)))
    idx, counts = [], []
    for r in range(len(rows) - 1):
        for j in range(m):
            a0, a1 = r * k + j, r * k + (j + 1) % k
            idx += [a0, a1, a1 + k, a0 + k]
            counts.append(4)
    return kit._mesh(np.asarray(pts, float) - np.asarray(shift, float), counts, idx, look)


SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 2400
INSTRUCTION = "Baste the roast with the juices from the pan, then put the baster down on the counter."
IN = 0.0254

# -- the roasting pan: Chicago Metallic 13 x 9 in roast pan, 2.25 in deep (rack left out). Its long side runs across
#    the robot (along y). Its own frame: origin at the middle of the inside floor -------------------------------------
PAN_C = (0.360, 0.000)
PAN_OUT = (9 * IN, 13 * IN, 2.25 * IN)  # x, y, z outside
WALL_T = 0.002
FLOOR_T = 0.002
PAN_IN = (PAN_OUT[0] - 2 * WALL_T, PAN_OUT[1] - 2 * WALL_T)  # inside floor, x by y
RIM_H = PAN_OUT[2] - FLOOR_T  # rim over the inside floor
FLOOR_Z = TOP + FLOOR_T  # world height of the inside floor
PAN_KG = 0.90  # the steel pan
RIM_R = 0.002  # its rolled rim, drawn
_BALL = [(RIM_R * math.sin(a), -RIM_R * math.cos(a)) for a in np.linspace(0.0, math.pi, 9)]  # a rim corner
JUICE_ML = 710.0  # 3 US cups
PAN_LOOK = kit.Look((0.16, 0.16, 0.17), 0.45, 0.5)  # dark nonstick steel
JUICE_LOOK = kit.Look((0.55, 0.30, 0.12), 0.15, 0.0, 0.85)
# -- the roast stand-in: a 9 in length of 4x4 lumber (dressed 3.5 x 3.5 in, the American Softwood Lumber Standard's
#    size for a nominal 4x4), wrapped in plastic film, lying in the pan's middle. Long side along y; its straight
#    sides make its top exactly its section -----------------------------------------------------------------------
ROAST = (3.5 * IN, 9.0 * IN, 3.5 * IN)  # x, y, z
ROAST_KG = 0.80  # 1.8 L of pine
ROAST_LOOK = kit.Look((0.80, 0.66, 0.45), 0.7)
TOP_LOOK = kit.Look((0.93, 0.86, 0.66), 0.9)  # its top face, where the juices are aimed
ROAST_TOP_Z = FLOOR_Z + ROAST[2]
FREE_AREA = PAN_IN[0] * PAN_IN[1] - ROAST[0] * ROAST[1]  # m^2 of floor the juices stand on
LEVEL0 = JUICE_ML * 1e-6 / FREE_AREA  # m: the juices' depth at the start (about 13 mm)

PAN = kit.Prop(
    "pan", pos=(PAN_C[0], PAN_C[1], FLOOR_Z), mass=PAN_KG + JUICE_ML / 1000.0, size=PAN_OUT,
    meshes=(kit.box((0, 0, -FLOOR_T / 2), (PAN_OUT[0], PAN_OUT[1], FLOOR_T), PAN_LOOK),
            *(kit.box((sx * (PAN_OUT[0] - WALL_T) / 2, 0, RIM_H / 2), (WALL_T, PAN_OUT[1], RIM_H), PAN_LOOK)
              for sx in (-1, 1)),
            *(kit.box((0, sy * (PAN_OUT[1] - WALL_T) / 2, RIM_H / 2), (PAN_IN[0], WALL_T, RIM_H), PAN_LOOK)
              for sy in (-1, 1)),
            kit.box((0, 0, LEVEL0 / 2), (PAN_IN[0], PAN_IN[1], LEVEL0), JUICE_LOOK),
            # drawn only: the pan's rolled rim along the walls' tops, meeting at rounded corners
            *(kit.cylinder((0, sy * (PAN_OUT[1] - WALL_T) / 2, RIM_H), RIM_R, PAN_OUT[0] - WALL_T, PAN_LOOK, axis="x")
              for sy in (-1, 1)),
            *(kit.cylinder((sx * (PAN_OUT[0] - WALL_T) / 2, 0, RIM_H), RIM_R, PAN_OUT[1] - WALL_T, PAN_LOOK,
                           axis="y") for sx in (-1, 1)),
            *(kit.lathe(_BALL, PAN_LOOK, center=(sx * (PAN_OUT[0] - WALL_T) / 2, sy * (PAN_OUT[1] - WALL_T) / 2, RIM_H),
                        n=24) for sx in (-1, 1) for sy in (-1, 1))),
    colliders=(kit.Collider((0, 0, -FLOOR_T / 2), size=(PAN_OUT[0], PAN_OUT[1], FLOOR_T)),
               *(kit.Collider((sx * (PAN_OUT[0] - WALL_T) / 2, 0, RIM_H / 2), size=(WALL_T, PAN_OUT[1], RIM_H))
                 for sx in (-1, 1)),
               *(kit.Collider((0, sy * (PAN_OUT[1] - WALL_T) / 2, RIM_H / 2), size=(PAN_IN[0], WALL_T, RIM_H))
                 for sy in (-1, 1))),
    material=(0.25, 0.20, "average"))
ROAST_PROP = kit.Prop(
    "roast", pos=(PAN_C[0], PAN_C[1], FLOOR_Z + ROAST[2] / 2), mass=ROAST_KG, size=ROAST,
    meshes=(kit.box((0, 0, -0.0015), (ROAST[0], ROAST[1], ROAST[2] - 0.003), ROAST_LOOK),
            kit.box((0, 0, ROAST[2] / 2 - 0.0015), (ROAST[0], ROAST[1], 0.003), TOP_LOOK)),
    colliders=(kit.Collider((0, 0, 0), size=ROAST),), material=(0.4, 0.35, "average"))

# -- the baster: OXO Good Grips baster (oxo.com: 12 x 2.5 x 2 in, a flat bulb; 1.5 oz / 40 mL; silicone bulb, Tritan
#    tube; 109 g listed). Its own frame: x along it from the tip's opening (x = 0) to the bulb's top, y across the flat
#    bulb's width (2.5 in), z through its thickness (2 in) ----------------------------------------------------------
BASTER_L = 12 * IN
BULB_W, BULB_T = 2.5 * IN, 2.0 * IN  # the flat bulb's width and thickness
TUBE_L = 0.215
TUBE_R = (0.005, 0.0135)  # outside radius at the tip and at the bulb
BORE = ((0.0, 0.003), (TUBE_L, 0.0116))  # inside radius: the opening, and the bore's top: 40 mL, the markings' 1.5 oz
COLLAR = (0.211, 0.223, 0.016)  # x0, x1, radius: the bulb's neck over the tube
PANEL_X = (0.224, 0.284)  # the bulb's squeezable walls, 6 cm along it
PANEL_W = 0.024  # each of the four panels 2.4 cm wide, so fingers closing on the two flat bulb's narrow sides pass
# between the broad sides' edges
PANEL_T = 0.003
PANEL_INSET = 0.0005
TRAVEL_LR = 0.016  # how far the narrow sides (y) can be squeezed in ...
TRAVEL_TB = 0.012  # ... and the broad ones (z)
DOME = (0.285, BASTER_L, 0.012)  # the bulb's closed top
BULB_ML = 100.0  # mL inside the bulb at rest
BULB_AREA_CM2 = 21.0  # its inside cross-section
PANEL_CM2 = (PANEL_X[1] - PANEL_X[0]) * PANEL_W * 1e4  # 14.4 cm^2: mL displaced per cm of a panel's push
K_WALL = 2000.0  # N/m per panel: the silicone wall's spring
TUBE_G, BULB_G, PANEL_G = 55.0, 38.0, 4.0  # 109 g in all
# Balance points along the baster: the tube's wall thickens with its radius (a tapered shell's centre of mass sits at
# L (r0 + 2 r1) / (3 (r0 + r1)) from the tip), the bulb and its panels at the bulb's middle.
_TUBE_COM = TUBE_L * (TUBE_R[0] + 2 * TUBE_R[1]) / (3 * (TUBE_R[0] + TUBE_R[1]))
PANEL_MID = 0.5 * (PANEL_X[0] + PANEL_X[1])
PANEL_SIGN = {"panel_l": -1.0, "panel_r": 1.0, "panel_t": -1.0, "panel_b": 1.0}  # inward push = sign * joint pos
BASTER_KIT = bk.Baster(bore=BORE, bulb_ml=BULB_ML, bulb_area_cm2=BULB_AREA_CM2, panel_cm2=PANEL_CM2, tip_d=0.006,
                       k_wall=K_WALL)
_BASE_COM = ((TUBE_G * _TUBE_COM + BULB_G * PANEL_MID) / (TUBE_G + BULB_G), 0.0, 0.0)
TUBE_LOOK = kit.Look((0.86, 0.90, 0.94), 0.12)  # clear Tritan, drawn opaque (the renderer draws a see-through
# surface as solid); the juices inside are drawn over it (_Drawer)
MARK_LOOK = kit.Look((0.25, 0.27, 0.30), 0.5)  # the tube's printed volume markings
MARK_ML = (10.0, 20.0, 30.0, 40.0)
BULB_LOOK = kit.Look((0.80, 0.10, 0.12), 0.6)  # red silicone
PLASTIC = (0.30, 0.25, "average")  # the Tritan tube, taken as a hard plastic
SILICONE = (1.16, 1.16, "average")  # the bulb: rubber on rubber, clean and dry, 1.16 (Engineering ToolBox, friction
# coefficients); with the YAM's rubber-faced pads (1.0 / 0.8, gripkit) a pinch on the bulb grips at about 1.1 / 1.0


class _Wall(sq.Wall):
    """A bulb wall panel: a spring of K_WALL back to rest; in the simulator only what pushes on it (a finger) moves
    it. The air's pressure on the panels is taken in the suction law (bulbkit: a panel under a vacuum counts as drawn
    in until its spring balances it), not through the joints: fed through a joint law once per control step, the stiff
    trapped air makes the panels oscillate. The panel's travel opens only once the spring acts (the first control
    step), as in squeezekit."""

    sets_limits = True

    def __init__(self, sign, lo, hi):
        super().__init__("baster_air_not_on_panels", sign, k=K_WALL, area_m2=0.0)
        object.__setattr__(self, "lim", (lo, hi))

    def drive(self, key, q, qd, J, mem):
        mem["limits"] = self.lim
        mem["deflection"] = self.sign * q
        return 0.0, self.k, 0.0, self.damp


BULB_A, BULB_B, BULB_N = BULB_W / 2 - PANEL_INSET, BULB_T / 2 - PANEL_INSET, 3.0  # the drawn bulb's outline: half
# width, half thickness and a squarish superellipse, through the panels' outer faces


def _panel(link: str, axis: str, side: float):
    half, travel = (BULB_W / 2, TRAVEL_LR) if axis == "Y" else (BULB_T / 2, TRAVEL_TB)
    off = side * (half - PANEL_INSET - PANEL_T / 2)
    anchor = (PANEL_MID, off, 0.0) if axis == "Y" else (PANEL_MID, 0.0, off)
    size = (PANEL_X[1] - PANEL_X[0], PANEL_T, PANEL_W) if axis == "Y" else (PANEL_X[1] - PANEL_X[0], PANEL_W, PANEL_T)
    lo, hi = (-travel, 0.0) if side > 0 else (0.0, travel)
    t = {("Y", 1.0): 0.0, ("Z", 1.0): math.pi / 2, ("Y", -1.0): math.pi, ("Z", -1.0): -math.pi / 2}[(axis, side)]
    look = _oval([(x, BULB_A, BULB_B, BULB_N) for x in PANEL_X], BULB_LOOK, t - math.pi / 4, t + math.pi / 4, m=12,
                 shift=anchor)  # drawn: its quarter of the bulb's rounded outline, which moves with the panel
    return kit.Link(link, "prismatic", axis, anchor=anchor, lower=-1e-4, upper=1e-4, start=0.0, mass=PANEL_G / 1000,
                    meshes=(look,),
                    colliders=(kit.Collider((0, 0, 0), size=size, material=SILICONE),),
                    damping=0.0, law=_Wall(-side, lo, hi))


def _along_x(pts):
    p = np.asarray(pts, float)
    return np.stack([p[:, 2], p[:, 1], p[:, 0]], axis=1)


_CORE = (BULB_W / 2 - PANEL_INSET - PANEL_T - TRAVEL_LR - 0.0005, BULB_T / 2 - PANEL_INSET - PANEL_T - TRAVEL_TB - 0.0005)
# the core's half width and half thickness: what the panels close down onto
_BASTER_COLS = (ck.colliders([_along_x(ck.frustum(TUBE_R[0], 0.0, TUBE_R[1], TUBE_L, 16)),
                              _along_x(ck.frustum(COLLAR[2], COLLAR[0], COLLAR[2], COLLAR[1], 16)),
                              _along_x(ck.frustum(DOME[2], DOME[0], DOME[2] * 0.6, DOME[1], 16))])
                + (kit.Collider(((PANEL_X[0] + DOME[0]) / 2, 0, 0),
                                size=(DOME[0] - PANEL_X[0] + 0.004, 2 * _CORE[0], 2 * _CORE[1])),))
_TUBE_PROFILE = [(TUBE_R[0], 0.0), (TUBE_R[1], TUBE_L)]
BASTER_KG = (TUBE_G + BULB_G) / 1000.0


def _tube_r(x: float) -> float:
    """The tube's outside radius at x from the tip."""
    return TUBE_R[0] + (TUBE_R[1] - TUBE_R[0]) * min(max(x, 0.0), TUBE_L) / TUBE_L


# The drawn bulb's rounded outline below and above the panels (drawn only: the collision shapes there are the neck's
# collar, the rigid core and the dome): a short flare from the collar, and a rounded top.
_SHOULDER = [(COLLAR[1] - 0.006 + 0.006 * u, COLLAR[2] + (BULB_A - COLLAR[2]) * math.sin(u * math.pi / 2),
              COLLAR[2] + (BULB_B - COLLAR[2]) * math.sin(u * math.pi / 2), 2.0 + (BULB_N - 2.0) * u)
             for u in (0.0, 0.25, 0.5, 0.75, 1.0)] + [(PANEL_X[0], BULB_A, BULB_B, BULB_N)]
_TOP = [(PANEL_X[1] + (BASTER_L - PANEL_X[1]) * u, BULB_A * (1 - u ** 3) ** (1 / 3), BULB_B * (1 - u ** 3) ** (1 / 3),
         BULB_N) for u in (0.0, 0.3, 0.5, 0.65, 0.78, 0.88, 0.95, 1.0)]
# Lying on the counter it rests on its tip and its flat bulb's broad side, tilted up toward the bulb.
LIE_TILT = math.atan2(BULB_T / 2 - PANEL_INSET - TUBE_R[0], PANEL_MID)
BASTER_START = (0.090, 0.250, TOP + TUBE_R[0] + 0.0002)  # the tip's opening; the tip toward the robot
R_LIE = np.array([[math.cos(LIE_TILT), 0, -math.sin(LIE_TILT)], [0, 1, 0], [math.sin(LIE_TILT), 0, math.cos(LIE_TILT)]])
BASTER = sq.free_body(kit.Mechanism(
    "baster", pos=BASTER_START, yaw=0.0, mass=BASTER_KG, tilt_xyzw=ck.quat(R_LIE),
    size=(BASTER_L, BULB_W, BULB_T),
    meshes=(kit.lathe(_TUBE_PROFILE, TUBE_LOOK, axis=(1.0, 0.0, 0.0), n=48),
            *(kit.lathe([(_tube_r(x) + 0.0004, -0.0006), (_tube_r(x) + 0.0004, 0.0006)], MARK_LOOK, center=(x, 0, 0),
                        axis=(1.0, 0.0, 0.0), n=48) for x in (BASTER_KIT.column_length(v) for v in MARK_ML)),
            kit.lathe([(COLLAR[2], 0.0), (COLLAR[2], COLLAR[1] - COLLAR[0])], BULB_LOOK, center=(COLLAR[0], 0, 0),
                      axis=(1.0, 0.0, 0.0), n=48),
            kit.box(((PANEL_X[0] + DOME[0]) / 2, 0, 0), (DOME[0] - PANEL_X[0], 2 * _CORE[0], 2 * _CORE[1]),
                    BULB_LOOK),
            _oval(_SHOULDER, BULB_LOOK), _oval(_TOP, BULB_LOOK)),
    colliders=_BASTER_COLS,
    links=(_panel("panel_l", "Y", 1.0), _panel("panel_r", "Y", -1.0),
           _panel("panel_t", "Z", 1.0), _panel("panel_b", "Z", -1.0)),
    material=PLASTIC, contact_offset=0.001),
    com=_BASE_COM)

OBJECTS = [PAN, ROAST_PROP, BASTER]
gripkit.use_rubber_pads()

# -- scoring thresholds ---------------------------------------------------------------------------------------------------
GOOD_ML = 20.0  # mL on the roast's top for a basting (half the baster's 40 mL) ...
SOME_ML = 8.0  # ... at least this much (under 20 mL) is a weak one
SPILL_OK_ML = 2.0  # mL on the counter allowed for 0.75 and 1.0
SPILL_BAD_ML = 10.0  # more on the counter caps the score at 0.25
PAN_TILT_DEG = 8.0  # the pan tipped further than this at any time: 0 (its juices would slop)
TOP_UP_DEG = 20.0  # juices land on the roast's top only while it faces within 20 degrees of up
TOUCH = 0.001
CLEAR = 0.005
PLACE_GAP = 0.005  # the baster let go resting on the counter: lowest point within 5 mm of it ...
PLACE_V = 0.30  # ... moving under 0.3 m/s
STILL_STEP = 0.0005
HOLD_STEPS = round(SUCCESS_HOLD_S / DT)
ARM_LINKS = ("link1", "link2", "link3", "link4", "link5", "gripper", "tip_left", "tip_right")
ROBOTS = ("left_robot", "right_robot")
LINK_POSES: dict = {}


def rotm(q) -> np.ndarray:
    return ck.rotm(q)


def tilt_deg(q) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(rotm(q)[2, 2])))))


# -- geometry readers -------------------------------------------------------------------------------------------------------
def tip_world(objects) -> np.ndarray:
    b = objects["baster"]
    return np.asarray(b.pos, float)  # the frame's origin is the tip's opening


def axis_world(objects) -> np.ndarray:
    """Unit vector along the baster from its tip toward its bulb."""
    return rotm(objects["baster"].quat_xyzw)[:, 0]


def in_rect(p_local, half, margin=0.0) -> bool:
    return abs(float(p_local[0])) <= half[0] - margin and abs(float(p_local[1])) <= half[1] - margin


def pan_local(objects, p):
    pan = objects["pan"]
    return rotm(pan.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(pan.pos, float))


def roast_local(objects, p):
    r = objects["roast"]
    return rotm(r.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(r.pos, float))


def in_juice_area(objects, p) -> bool:
    """Over the pan's floor where the juices stand: inside its walls, outside the roast's footprint."""
    return (in_rect(pan_local(objects, p), (PAN_IN[0] / 2, PAN_IN[1] / 2))
            and not in_rect(roast_local(objects, p), (ROAST[0] / 2, ROAST[1] / 2)))


def _seg_gap(q, a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    u = (b - a) / L
    d = q - a
    s = d @ u
    radial = np.linalg.norm(d - s[:, None] * u, axis=1)
    return np.hypot(np.maximum(radial - r, 0.0), np.maximum(np.maximum(-s, s - L), 0.0))


def baster_gap(qb):
    """Distance from baster-frame points to its outline (the tube as its widest cylinder, the flat bulb at rest as a
    cylinder as wide as it, a little generous through its thickness)."""
    g = _seg_gap(qb, (0, 0, 0), (COLLAR[0], 0, 0), TUBE_R[1])
    return np.minimum(g, _seg_gap(qb, (COLLAR[0], 0, 0), (BASTER_L, 0, 0), BULB_W / 2))


_GAP_MEMO: dict = {}


def link_gap(objects, key) -> float:
    """Smallest gap (m) between arm link `key` and the baster's outline; 0.02 for a link more than 2 cm away."""
    o, R = LINK_POSES[key]
    b = objects["baster"]
    bo, Rb = np.asarray(b.pos, float), rotm(b.quat_xyzw)
    memo = (key, np.asarray(o).tobytes(), np.asarray(R).tobytes(), bo.tobytes(), Rb.tobytes())
    if memo in _GAP_MEMO:
        return _GAP_MEMO[memo]
    link = key.split("/")[1]
    _, cen, rad = yg.cloud(link)
    o_, R_ = np.asarray(o, float), np.asarray(R, float)
    centre = (o_ + R_ @ cen - bo) @ Rb
    if float(np.linalg.norm(centre - np.array([BASTER_L / 2, 0, 0]))) - rad > BASTER_L / 2 + 0.03:
        out = 0.02
    else:
        q = yg.near(link, lambda x: (o_ + x @ R_.T - bo) @ Rb, baster_gap, 0.02)
        out = min(0.02, float(baster_gap(q).min())) if len(q) else 0.02
    if len(_GAP_MEMO) > 1024:
        _GAP_MEMO.clear()
    _GAP_MEMO[memo] = out
    return out


def arm_gap(objects, robots=ROBOTS) -> float:
    keys = [k for k in LINK_POSES if k.split("/")[0] in robots]
    return min((link_gap(objects, k) for k in keys), default=0.02)


def baster_low_z(objects) -> float:
    """World height of the baster's lowest point (rings of the tube's ends, the flat bulb's oval at rest, its top)."""
    b = objects["baster"]
    R = rotm(b.quat_xyzw)
    a = np.linspace(0, 2 * math.pi, 24, endpoint=False)
    ring = np.stack([np.zeros_like(a), np.cos(a), np.sin(a)], axis=1)
    pts = [np.asarray(b.pos, float) + (np.array([x, 0, 0]) + ring * np.array([0, ry_, rz_])) @ R.T
           for x, ry_, rz_ in ((0.0, TUBE_R[0], TUBE_R[0]), (TUBE_L, TUBE_R[1], TUBE_R[1]),
                               (PANEL_MID, BULB_W / 2, BULB_T / 2), (BASTER_L, DOME[2], DOME[2]))]
    return float(np.min(np.concatenate(pts)[:, 2]))


def pushes(objects) -> list:
    """Each bulb panel's inward push (m), read from its joint: what a hand did to the bulb."""
    out = []
    for link, sign in PANEL_SIGN.items():
        try:
            out.append(sign * kit.joint(objects, f"baster/{link}").pos)
        except KeyError as e:  # no joint state: offline (unit tests) no squeeze; in the simulator a missing panel stops
            if "isaaclab" in sys.modules:  # the trial, since the squeeze would otherwise read as none, silently
                raise RuntimeError(f"baster: no joint state for the bulb panel {link}") from e
            out.append(0.0)
    return out


def level_z(objects, pan_ml: float) -> float:
    """World height of the juices' surface (the pan level on the counter)."""
    return float(objects["pan"].pos[2]) + pan_ml * 1e-6 / FREE_AREA


def land(objects, p0, v0, pan_ml):
    """Where juices leaving the tip at p0 with velocity v0 come down: 'roast' (its top), 'pan', or 'counter'."""
    r = objects["roast"]
    rtop = np.asarray(r.pos, float) + rotm(r.quat_xyzw) @ np.array([0, 0, ROAST[2] / 2])
    rim = np.asarray(objects["pan"].pos, float) + np.array([0, 0, RIM_H])
    up = np.array([0.0, 0.0, 1.0])
    # the roast's top counts while it faces up and the roast is still in the pan (basting it lifted out does not count)
    top_ok = tilt_deg(r.quat_xyzw) <= TOP_UP_DEG and in_rect(pan_local(objects, r.pos), (PAN_IN[0] / 2, PAN_IN[1] / 2))
    planes = [("roast", rtop, up, lambda p: top_ok and in_rect(roast_local(objects, p), (ROAST[0] / 2, ROAST[1] / 2))),
              ("pan", rim, up, lambda p: in_rect(pan_local(objects, p), (PAN_IN[0] / 2, PAN_IN[1] / 2))),
              ("pan", np.array([0, 0, level_z(objects, pan_ml)]), up,
               lambda p: in_rect(pan_local(objects, p), (PAN_IN[0] / 2, PAN_IN[1] / 2))),
              ("counter", np.array([0, 0, TOP]), up, lambda p: True)]
    if float(p0[2]) < rtop[2]:
        planes = planes[1:]
    name, pt = bk.land(p0, v0, planes)
    return name or "counter", pt


class Ledger:
    """One trial: the baster's inside, the pan's juices, where every mL went, the let-go and still/hold checks."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.bulb = bk.Bulb(BASTER_KIT)
        self.pan_ml = JUICE_ML
        self.on_roast = 0.0
        self.counter = 0.0
        self.in_pan = 0.0
        self.tipped = False
        self.last = None
        self.last_tip = None
        self.held = False
        self.placed = True
        self.clear = False
        self.good = 0
        self.n = 0
        self.jet = None  # (tip, landing point) of this step's jet, for the drawing
        self.patches = {}  # (x, y) on the roast's top, 1 cm cells -> mL landed there, for the drawing

    def step(self, objects) -> None:
        self.n += 1
        b = objects["baster"]
        bp = np.asarray(b.pos, float)
        if tilt_deg(objects["pan"].quat_xyzw) > PAN_TILT_DEG:
            self.tipped = True
        tip = tip_world(objects)
        ax = axis_world(objects)
        down = float(ax[2])  # tip-to-bulb axis up = tip pointing down
        sub_depth = level_z(objects, self.pan_ml) - float(tip[2]) if in_juice_area(objects, tip) else -1.0
        if float(tip[2]) < float(objects["pan"].pos[2]) - 0.002:  # under the pan's floor: outside it
            sub_depth = -1.0
        drawn0, back0 = self.bulb.drawn, self.bulb.back
        out = self.bulb.step(pushes(objects), down, sub_depth, DT)
        self.pan_ml += (self.bulb.back - back0) - (self.bulb.drawn - drawn0)
        if out > 0.0:
            vtip = np.zeros(3) if self.last_tip is None else (tip - self.last_tip) / DT
            v0 = -ax * self.bulb.jet_speed(out / DT) + vtip
            where, pt = land(objects, tip, v0, self.pan_ml)
            if where == "roast":
                self.on_roast += out
            elif where == "pan":
                self.in_pan += out
                self.pan_ml += out
            else:
                self.counter += out
            self.jet = (tip, np.asarray(pt, float))
            if where == "roast":
                loc = roast_local(objects, pt)
                cell = (round(float(loc[0]), 2), round(float(loc[1]), 2))
                self.patches[cell] = self.patches.get(cell, 0.0) + out
        else:
            self.jet = None
        self.last_tip = tip
        gap = arm_gap(objects)
        held = gap <= TOUCH
        low = baster_low_z(objects) - TOP
        if self.held and not held:  # let go this step: set down, or dropped or thrown?
            v = 0.0 if self.last is None else float(np.linalg.norm(bp - self.last)) / DT
            self.placed = -0.01 <= low <= PLACE_GAP and v <= PLACE_V and not in_rect(
                pan_local(objects, bp), (PAN_OUT[0] / 2, PAN_OUT[1] / 2))
        self.held = held
        self.clear = gap > CLEAR
        still = self.last is not None and float(np.linalg.norm(bp - self.last)) <= STILL_STEP
        self.last = bp
        on_counter = -0.01 <= low <= PLACE_GAP and not in_rect(pan_local(objects, bp), (PAN_OUT[0] / 2, PAN_OUT[1] / 2))
        done = self.tier() == 0.75 and self.clear and on_counter and self.placed and still
        self.good = self.good + 1 if done else 0
        if _PROBE:
            fing = FINGERS.get("right_robot", (float("nan"),) * 2)
            with open(_PROBE, "a") as fh:
                fh.write("%d tip %s ax %s sub %.4f P %s p %.2f liq %.2f drawn %.2f back %.2f roast %.2f pan %.2f "
                         "counter %.2f gap %.4f low %.4f fingR %.4f %.4f\n" % (
                             self.n, " ".join("%.4f" % v for v in tip), " ".join("%.3f" % v for v in ax), sub_depth,
                             " ".join("%.4f" % v for v in pushes(objects)), self.bulb.p, self.bulb.liquid,
                             self.bulb.drawn, self.bulb.back, self.on_roast, self.in_pan, self.counter, gap, low,
                             fing[0], fing[1]))

    def tier(self) -> float:
        if self.tipped or self.on_roast < SOME_ML - 1e-9:
            return 0.0
        if self.on_roast < GOOD_ML - 1e-9 or self.counter > SPILL_BAD_ML + 1e-9:
            return 0.25
        return 0.75 if self.counter <= SPILL_OK_ML + 1e-9 else 0.5

    def score(self) -> float:
        s = self.tier()
        return 1.0 if s == 0.75 and self.good >= HOLD_STEPS else s


_L = None
_PROBE = os.environ.get("BASTER_PROBE")
FINGERS: dict = {}


def _read_links(base) -> None:
    """Every arm link's pose (env frame) at this control step, into LINK_POSES. A missing arm or link, or finger pads
    left without their rubber, stops the trial: the contact checks would otherwise pass silently."""
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    for robot in ROBOTS:
        try:
            art = base.scene[robot]
        except KeyError as e:
            raise RuntimeError(f"baster: no {robot} in the scene for the contact checks") from e
        names = list(art.body_names)
        P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
        Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
        missing = [link for link in ARM_LINKS if link not in names]
        if missing:
            raise RuntimeError(f"baster: {robot} has no link(s) {missing} for the contact checks")
        for link in ARM_LINKS:
            i = names.index(link)
            LINK_POSES[f"{robot}/{link}"] = (P[i] - origin, rotm(Q[i]))
        jn = list(art.joint_names)
        q = getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0].detach().cpu().numpy()
        FINGERS[robot] = (float(q[jn.index("joint7")]), float(q[jn.index("joint8")]))
    if gripkit._STATE.get("bound", 0) < 0:
        raise RuntimeError("baster: the finger pads' rubber material could not be bound")
    if _PROBE and not _READ_ONCE:
        _READ_ONCE.append(1)
        print(f"baster: contact checks on {len(LINK_POSES)} arm links; pad shapes with rubber: "
              f"{gripkit._STATE.get('bound')}", file=sys.stderr, flush=True)


_READ_ONCE: list = []
kit.STATE_HOOKS.append(_read_links)


# -- the juices' weight on the baster ---------------------------------------------------------------------------------------
def juice_wrench(tip, R, ml: float, com):
    """(force, torque about `com`), world frame: the weight of `ml` of juices inside the baster, acting at the middle
    of the column they fill from the tip (tip and R: the baster's opening and rotation; com: its centre of mass)."""
    F = np.array([0.0, 0.0, -max(0.0, ml) * 1e-6 * bk.RHO * 9.80665])
    c = np.asarray(tip, float) + np.asarray(R, float)[:, 0] * BASTER_KIT.column_length(ml) / 2
    return F, np.cross(c - np.asarray(com, float), F)


_WRENCH: dict = {"mode": None}


def _apply_juice_weight(base) -> None:
    """Each control step, put the juices now inside the baster on its body as an external force (they are not part of
    its rigid mass). A missing baster or no way to apply a force stops the trial."""
    import torch

    try:
        art = base.scene["baster"]
    except KeyError as e:
        raise RuntimeError("baster: no baster in the scene for the juices' weight") from e
    names = list(art.body_names)
    idx = [i for i, n in enumerate(names) if not n.startswith("panel")]
    if not idx:
        raise RuntimeError(f"baster: no base body among {names}")
    i = idx[0]
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    P = getattr(art.data.body_pos_w, "torch", art.data.body_pos_w)[0].detach().cpu().numpy().astype(float)
    Q = getattr(art.data.body_quat_w, "torch", art.data.body_quat_w)[0].detach().cpu().numpy().astype(float)
    tip, R = P[i] - origin, rotm(Q[i])
    ml = _L.bulb.liquid if (_L is not None and _L.episode == kit.episode()) else 0.0
    F, tq = juice_wrench(tip, R, ml, tip + R @ np.asarray(_BASE_COM))
    f = torch.tensor(np.asarray(F, np.float32).reshape(1, 1, 3), device=art.device)
    t = torch.tensor(np.asarray(tq, np.float32).reshape(1, 1, 3), device=art.device)
    tries = []
    comp = getattr(art, "permanent_wrench_composer", None)
    if comp is not None:
        tries.append(("wrench_composer", lambda: comp.set_forces_and_torques(
            forces=f, torques=t, body_ids=[i], env_ids=[0], is_global=True)))
    if hasattr(art, "set_external_force_and_torque"):
        tries.append(("external_global", lambda: art.set_external_force_and_torque(
            f, t, body_ids=[i], env_ids=[0], is_global=True)))
    for name, fn in tries:
        if _WRENCH["mode"] not in (None, name):
            continue
        try:
            fn()
        except Exception as e:  # noqa: BLE001 (try the next API)
            if _WRENCH["mode"] == name:
                raise
            print(f"baster: {name} failed for the juices' weight: {e!r}", file=sys.stderr, flush=True)
            continue
        if _WRENCH["mode"] is None:
            _WRENCH["mode"] = name
            if _PROBE:
                print(f"baster: the juices' weight goes on body {names[i]} through {name}", file=sys.stderr,
                      flush=True)
        return
    raise RuntimeError("baster: no way to put the juices' weight on the baster")


kit.STATE_HOOKS.append(_apply_juice_weight)
if "isaaclab" in sys.modules:
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
    _DRAW.draw(objects, _L)


# -- the juices the cameras see -----------------------------------------------------------------------------------------------
JUICE_RGB = (0.55, 0.30, 0.12)


class _Drawer:
    """The juices inside the clear tube (a column from the tip), the jet while it runs, and what landed on the roast's
    top (a dark wet patch per landing spot, carried with the roast), drawn with Isaac Lab's VisualizationMarkers; the
    env origin is read once from the pan's prim. Only a picture: on any error it turns itself off and scoring carries
    on."""

    path = "/Visuals/baster_juices"

    def __init__(self):
        self.off = False
        self.origin = None
        self.markers = None

    def _origin(self, objects):
        import omni.usd
        from pxr import UsdGeom

        stage, cache = omni.usd.get_context().get_stage(), UsdGeom.XformCache()
        for prim in stage.Traverse():
            if prim.GetName() == "pan":
                w = np.array(cache.GetLocalToWorldTransform(prim).ExtractTranslation(), float)
                return w - np.asarray(objects["pan"].pos, float)
        return np.zeros(3)

    def draw(self, objects, ledger):
        if self.off:
            return
        try:
            if self.origin is None:
                self.origin = self._origin(objects)
            if self.markers is None:
                import isaaclab.sim as sim_utils
                from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

                look = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(sq._linear(x) for x in JUICE_RGB), roughness=0.2)
                wet = sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(sq._linear(x * 0.75) for x in JUICE_RGB),
                                                  roughness=0.3)
                self.markers = VisualizationMarkers(VisualizationMarkersCfg(prim_path=self.path, markers={
                    "column": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=look),
                    "patch": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=wet)}))
            tr, rq, sc, idx = [], [], [], []
            b = objects["baster"]
            R = rotm(b.quat_xyzw)
            tip = np.asarray(b.pos, float)
            L = BASTER_KIT.column_length(ledger.bulb.liquid)
            if ledger.bulb.liquid > 0.05 and L > 0.001:  # drawn over the (opaque-drawn) tube, under its markings
                Lb = min(L, TUBE_L)
                nseg = max(1, math.ceil(Lb / 0.015))
                for k in range(nseg):
                    a, b = Lb * k / nseg, Lb * (k + 1) / nseg
                    r = _tube_r((a + b) / 2) + 0.0002
                    tr.append(tip + R[:, 0] * (a + b) / 2)
                    rq.append(sq.quat(R @ np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float)))
                    sc.append((r, r, b - a))
                    idx.append(0)
            if ledger.jet is not None:
                p0, p1 = ledger.jet
                d = p1 - p0
                n = float(np.linalg.norm(d))
                if n > 1e-3:
                    z = d / n
                    x = np.cross(z, [0.0, 1.0, 0.0])
                    x = x / np.linalg.norm(x) if np.linalg.norm(x) > 1e-6 else np.array([1.0, 0, 0])
                    tr.append(p0 + d / 2)
                    rq.append(sq.quat(np.stack([x, np.cross(z, x), z], axis=1)))
                    sc.append((0.002, 0.002, n))
                    idx.append(0)
            r_ = objects["roast"]
            Rr, pr = rotm(r_.quat_xyzw), np.asarray(r_.pos, float)
            for (lx, ly), ml in ledger.patches.items():
                rad = min(0.03, math.sqrt(ml * 1e-6 / (math.pi * 0.001)))  # drawn as a 1 mm film of juices
                tr.append(pr + Rr @ np.array([lx, ly, ROAST[2] / 2 + 0.0008]))
                rq.append(sq.quat(Rr))
                sc.append((rad, rad, 0.001))
                idx.append(1)
            if not tr:
                self.markers.set_visibility(False)
                return
            self.markers.set_visibility(True)
            self.markers.visualize(translations=np.array(tr, np.float32) + np.asarray(self.origin, np.float32),
                                   orientations=np.array(rq, np.float32), scales=np.array(sc, np.float32),
                                   marker_indices=np.array(idx, int))
        except Exception as e:  # noqa: BLE001 (no simulator, or cosmetic only)
            self.off = True
            if "isaaclab" in sys.modules:
                print(f"baster: juices drawing off ({e!r}); scoring is unaffected", file=sys.stderr, flush=True)


_DRAW = _Drawer()


def success(objects):
    """At least 20 mL on the roast's top, at most 2 mL on the counter, the baster let go resting on the counter with
    both arms clear of it, all still: this step counts toward the 1 s hold."""
    _update(objects)
    return _L.good >= 1


def score(objects):
    """mL of juices landed on the roast's top and on the counter: 20 mL or more on the roast with at most 2 mL on the
    counter scores 0.75, with more on the counter 0.5; 8 to 20 mL on the roast 0.25; under 8 mL 0. Over 10 mL on the
    counter caps the score at 0.25. The pan tipped past 8 degrees scores 0. 1.0: the 0.75 state with the baster let go
    resting on the counter (not dropped or thrown), both arms at least 5 mm clear of it, still for 1 s."""
    _update(objects)
    return _L.score()


# -- scripted demo -----------------------------------------------------------------------------------------------------------
# One hand, as a person bastes: the left takes the lying baster by its bulb from above, a light squeeze across two of
# its wall panels (the panels' grip carries the baster), and lifts it, turning its hand to point forward in steps so the
# baster hangs tip-down below the bulb with its tip always clear of the counter. Over the pan's front strip of juices it
# squeezes the bulb, lowers the tip under the surface to 4 mm off the floor, eases back to the light hold (the bulb
# opens and draws the juices up) and waits, lifts the baster out over the roast, tip 2.5 cm above its top, squeezes the
# bulb (the juices go onto the roast) and eases off, then carries the baster back and lays it down where it was, lets
# go and goes home. The right arm is not needed and stays at its ready pose.
FINGER_HALF_OPEN = 0.047
K_FINGER = 5000.0
PITCH_FWD = -math.pi / 2
HOLD = 0.002  # m a side: the light squeeze that holds the baster by its bulb (4 N a finger on the silicone)
TURN_HOLD = 0.005  # m a side: a firm squeeze while the baster is picked up and turned, so it turns with the hand and
# does not swing past plumb in the pinch (a pinch turns freely about its closing line under a light squeeze)
SQUEEZE = 0.0145  # m a side: emptying the bulb before the dip
SQUEEZE_OUT = 0.015  # m a side: emptying it over the roast
HELD_DZ = -0.002  # m: the held baster sits this much under the hand's command (the arm gives under its load)
DIP_XY = (PAN_C[0] - 0.020, PAN_C[1] - (PAN_IN[1] + ROAST[1]) / 4)  # the pan's right-hand strip of juices, between
# the roast's end and the pan's wall (it runs front to back, so the hanging baster's small lean along x does not matter)
TIP_OFF_FLOOR = 0.004
OVER_ROAST = 0.025
CARRY_Z = ROAST_TOP_Z + 0.050  # the tip's height whenever the baster crosses the pan (5 cm over the roast's top)
R_DOWN = np.array([[0.0, 0.0, -1.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0]])  # baster x (tip to bulb) to world up
TURN = ((0.0, 0.10), (-math.pi / 6, 0.20), (-math.pi / 3, 0.30), (PITCH_FWD, 0.36))  # (hand pitch, bulb height over
# the counter): the lift-and-turn from lying to tip-down, the tip at least 1 cm over the counter at each step even if
# the pinch held the baster rigidly


def finger_for(half: float) -> float:
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * min(FINGER_HALF_OPEN, max(0.0, half)) / FINGER_HALF_OPEN


def bulb_grip(push: float) -> float:
    """Gripper command pushing each gripped panel `push` m in (the finger's servo in series with the panel's spring)."""
    return finger_for(BULB_W / 2 - PANEL_INSET - push - K_WALL * push / K_FINGER)


def _tw(duration, pos, finger, label, yaw=0.0, pitch=0.0, roll=0.0):
    return kit.Tilted(float(duration), tuple(float(v) for v in pos), float(finger), label, yaw=float(yaw),
                      pitch=float(pitch), roll=float(roll))


def _t(seq):
    return sum(w.duration for w in seq)


def held_tcp(tip_xyz):
    """The hand's tool point on the bulb's middle, holding the baster tip-down with its tip's opening at `tip_xyz`."""
    p = np.asarray(tip_xyz, float) + R_DOWN @ np.array([PANEL_MID, 0.0, 0.0]) - np.array([0, 0, HELD_DZ])
    return tuple(float(v) for v in p)


def demo():
    """The left bastes with the baster held by its bulb; the right stays home. (BASTER_VARIANT, the scorer's checks in
    the simulator: nothing; no_squeeze; air_release, squeezed and let go in the air before the dip; wrong_way, the
    squeeze made only once the tip is under the juices and let go after it is lifted out; near_miss, all done right
    but squeezed out 1.5 cm off the roast's near edge, into the pan.)"""
    var = os.environ.get("BASTER_VARIANT", "")
    stay = [kit.Home(10.0, label="stay at the ready pose")]
    if var == "nothing":
        return {"left": list(stay), "right": list(stay)}
    open_ = finger_for(BULB_W / 2 + 0.012)
    hold = bulb_grip(HOLD)
    firm = bulb_grip(TURN_HOLD)
    fill = hold if var == "no_squeeze" else bulb_grip(SQUEEZE)
    empty = bulb_grip(SQUEEZE_OUT)
    lean = -LIE_TILT  # the hand tilted with the lying baster, square to it
    o0 = np.asarray(BASTER_START, float)
    bulb0 = o0 + R_LIE @ np.array([PANEL_MID, 0.0, 0.0])
    P = PITCH_FWD
    level = FLOOR_Z + LEVEL0
    tip_hi = (DIP_XY[0], DIP_XY[1], level + 0.040)
    tip_dip = (DIP_XY[0], DIP_XY[1], FLOOR_Z + TIP_OFF_FLOOR)
    tip_up = (DIP_XY[0], DIP_XY[1], CARRY_Z)
    tip_over = (PAN_C[0], PAN_C[1], CARRY_Z)
    aim_x = PAN_C[0] - ROAST[0] / 2 - 0.015 if var == "near_miss" else PAN_C[0]
    tip_aim = (aim_x, PAN_C[1], ROAST_TOP_Z + OVER_ROAST)
    at = lambda z: (bulb0[0], bulb0[1], TOP + z)  # noqa: E731
    left = [_tw(3.0, (bulb0[0], bulb0[1], bulb0[2] + 0.10), open_, "over the bulb", pitch=lean),
            _tw(2.0, bulb0, open_, "fingers either side of the bulb", pitch=lean),
            _tw(0.5, bulb0, open_, "settle", pitch=lean),
            _tw(1.3, bulb0, firm, "hold the bulb firmly", pitch=lean),
            _tw(0.3, bulb0, firm, "settle", pitch=lean)]
    for k, (pitch, z) in enumerate(TURN):
        left.append(_tw(3.0, at(z), firm, "lift it, turning it tip-down" if k < len(TURN) - 1 else "hanging tip-down",
                        pitch=pitch))
    left += [_tw(1.0, at(TURN[-1][1]), firm, "let it settle", pitch=P),
             _tw(1.0, at(TURN[-1][1]), hold, "ease to the light hold", pitch=P),
             _tw(2.0, held_tcp((bulb0[0], bulb0[1], CARRY_Z)), hold, "the tip down to 5 cm over the roast's height",
                 pitch=P),
             _tw(5.0, held_tcp(tip_up), hold, "over the pan's right-hand strip", pitch=P),
             _tw(1.5, held_tcp(tip_hi), hold, "tip 4 cm over the juices", pitch=P),
             _tw(1.5, held_tcp(tip_hi), hold, "let it settle", pitch=P)]
    if var == "air_release":  # squeeze and let go in the air, then dip: it should draw nothing
        left += [_tw(1.5, held_tcp(tip_hi), fill, "squeeze the bulb", pitch=P),
                 _tw(1.5, held_tcp(tip_hi), hold, "let it open in the air", pitch=P)]
    elif var == "wrong_way":  # no squeeze in the air: it comes under the surface, and the let-go after the lift
        left += [_tw(2.0, held_tcp(tip_hi), hold, "hold", pitch=P)]
    else:
        left += [_tw(1.5, held_tcp(tip_hi), fill, "squeeze the bulb", pitch=P),
                 _tw(0.5, held_tcp(tip_hi), fill, "hold the squeeze", pitch=P)]
    fin = left[-1].finger
    under, lifted = (fill, hold) if var == "wrong_way" else (hold, hold)
    left += [_tw(2.5, held_tcp(tip_dip), fin, "tip under the juices, 4 mm off the floor", pitch=P),
             _tw(0.5, held_tcp(tip_dip), fin, "settle", pitch=P),
             _tw(1.5, held_tcp(tip_dip), under, "squeeze under the juices" if var == "wrong_way" else
                 "ease off: the bulb opens and draws the juices up", pitch=P),
             _tw(1.5, held_tcp(tip_dip), under, "wait", pitch=P),
             _tw(2.0, held_tcp(tip_up), under, "lift it out", pitch=P),
             *([_tw(1.5, held_tcp(tip_up), lifted, "let the bulb open in the air", pitch=P)] if var == "wrong_way"
               else []),
             _tw(4.0, held_tcp(tip_over), hold, "over the roast", pitch=P),
             _tw(1.0, held_tcp(tip_over), hold, "let it settle", pitch=P),
             _tw(1.5, held_tcp(tip_aim), hold, "tip 2.5 cm over its top", pitch=P),
             _tw(0.5, held_tcp(tip_aim), hold, "settle", pitch=P),
             _tw(1.5, held_tcp(tip_aim), empty, "squeeze it out onto the roast", pitch=P),
             _tw(1.5, held_tcp(tip_aim), empty, "hold the squeeze", pitch=P),
             _tw(1.0, held_tcp(tip_aim), hold, "ease off", pitch=P),
             _tw(2.0, held_tcp(tip_over), hold, "lift it", pitch=P),
             _tw(5.0, held_tcp((bulb0[0], bulb0[1], CARRY_Z)), hold, "back over its place", pitch=P),
             _tw(2.0, at(TURN[-1][1]), hold, "up", pitch=P),
             _tw(1.0, at(TURN[-1][1]), firm, "hold it firmly again", pitch=P)]
    for pitch, z in reversed(TURN[:-1]):
        left.append(_tw(3.0, at(z), firm, "lower it, turning it level", pitch=pitch))
    down = bulb0 + np.array([0.0, 0.0, 0.002])
    left += [_tw(3.0, (down[0], down[1], down[2] + 0.03), firm, "down to the counter", pitch=lean),
             _tw(1.5, down, firm, "lay it down", pitch=lean),
             _tw(0.3, down, firm, "settle", pitch=lean),
             _tw(1.0, down, open_, "let go", pitch=lean),
             _tw(1.5, (down[0], down[1], down[2] + 0.08), open_, "lift clear", pitch=lean),
             kit.Home(3.0, label="back to the ready pose"),
             kit.Home(3.0, label="wait")]
    return {"left": left, "right": [kit.Home(_t(left), label="stay at the ready pose")]}
