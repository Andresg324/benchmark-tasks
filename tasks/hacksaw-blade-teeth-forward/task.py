"""A 12-inch hacksaw frame is clamped by its back bar in a bench vise, lying flat with its handle to the robot's left.
Its old blade has been taken out and its wing nut backed off half a turn, and a new 12-inch bi-metal blade lies across
two wooden blocks in front of it. Fitting a hacksaw blade means putting its two holes over the frame's two pins with
the teeth pointing away from the handle, so that the saw cuts on the push stroke, then turning the wing nut until the
blade is snug and one to two and a half turns more, so it is taut. A blade fitted backwards cuts on the pull stroke:
it chatters, grabs and dulls, and a slack blade wanders and snaps in the cut.
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
INSTRUCTION = "Fit the new blade to the hacksaw."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the workspace is clear
# for the next person or job. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# --- The saw (fixed: its back bar is clamped in the vise). Frame coordinates: origin on the blade's center line at the
# middle between the pins, on the table top (world (SAW_X, SAW_Y, TOP)); x toward the back bar, y toward the handle
# (the robot's left), z up.
SAW_X, SAW_Y = 0.31, 0.095  # the blade line's x on the table and the y of the middle between the pins
H = 0.090  # the frame lies flat with the blade plane 9 cm above the table (the vise jaws' top)
PIN_Y = 0.1524  # the pins are 12 inches (304.8 mm) apart, the blade's nominal length
PIN_R = 0.00175  # 3.5 mm pins
THROAT = 0.100  # 4-inch throat: blade line to the back bar's inner edge
BAR_W, BAR_T = 0.022, 0.012  # the frame's steel channel, 22 mm deep in the frame's plane and 12 mm thick
POST_Y = 0.172  # the front (-y) and rear (+y) posts' center lines
NUT_Y = -0.205  # the wing nut's center, on the tension screw outside the front post
NUT_HUB_R, NUT_SPAN, NUT_LEN, NUT_WING_T = 0.005, 0.046, 0.012, 0.010

# The blade: 12-inch (305 mm between hole centers, 318 mm long) by 1/2-inch (12.7 mm) bi-metal blade, 0.65 mm thick,
# 18 teeth per inch, painted blue with a white arrow printed on both faces pointing the way the teeth cut.
BLADE_L, BLADE_W, BLADE_T = 0.318, 0.0127, 0.00065
BLADE_COL_T = 0.003  # its collider is 3 mm thick so the fingers can pinch its edges
HOLE_GAP = 0.010  # the colliders leave a 10 mm slot at each hole (see fidelity): a drop within +-3 mm goes over a pin
BLADE_START = (0.215, 0.07, TOP + 0.030 + BLADE_COL_T / 2)  # across two 3 cm wooden blocks
BLADE_YAW = math.radians(-30.0)  # lying askew, its arrow toward the handle's side (+y) and away from the robot

# The tension screw: M6 x 1 mm. Snug = the blade's slack taken up; the trial starts half a turn short of that.
THREAD = tk.Thread(pitch=0.001, turns=6.0, start=-180.0, max_tight=1440.0, hand=-1, free_lift=0.0)
NUT = "saw/wingnut"

# --- Scoring thresholds
SEAT_XY = 0.00325  # a hole's center within 3.25 mm of its pin's axis (the slot's 5 mm half-width less the pin's 1.75 mm)
SEAT_Z = 0.002  # and the blade down on the spigots: its mid-plane within 2 mm of the seated height (a blade resting
# on top of a pin sits 7.5 mm high)
FLAT_DEG = 5.0  # and flat: its face within 5 degrees of the frame's plane
TEETH_DEG = 20.0  # teeth direction: the printed arrow within 20 degrees of pointing away from the handle and the
# toothed edge within 20 degrees of facing away from the back bar (a reversed blade is 180 degrees off)
TAUT_DEG = 360.0  # taut: at least one full turn past snug. Under it the blade still flexes sideways and wanders
OVER_DEG = 900.0  # overtight past 2.5 turns: the pins start to tear the blade's holes and the frame bows
OFF_STEPS = 10  # past snug, the blade counts as off the pins only once it has been off for 10 steps in a row (a
# one-step knock is a jolt, not a blade taken off)
GRIP_SLACK = 90.0  # all but 90 degrees of the wing nut's tightening must be done by a gripper holding it
HOLD_OPEN = 0.6  # a gripper holds the blade or the nut when its fingers are under 60 % open with its tool point
HOLD_MARGIN = 0.02  # within 2 cm of the blade's center line or of the nut
HOME_TOL = 0.03  # a gripper is home within 3 cm of where its tool point was at the trial's first scored step
HOME_STILL = 0.001  # and still: its tool point moved less than 1 mm since the last step (2 cm/s)
_DEBUG = bool(os.environ.get("HACKSAW_DEBUG"))

STEEL = kit.Look((0.74, 0.75, 0.77), 0.35, metallic=0.6)
CHROME = kit.Look((0.86, 0.87, 0.89), 0.2, metallic=0.85)
FRAME = kit.Look((0.80, 0.12, 0.08), 0.45, metallic=0.2)  # red enamel frame
GRIP = kit.Look((0.08, 0.08, 0.09), 0.7)
BLUE = kit.Look((0.10, 0.30, 0.75), 0.45, metallic=0.2)
WHITE = kit.Look((0.97, 0.97, 0.95), 0.5)
TEETH = kit.Look((0.80, 0.80, 0.82), 0.25, metallic=0.9)
VISE = kit.Look((0.22, 0.36, 0.52), 0.55, metallic=0.2)
DARK = kit.Look((0.15, 0.15, 0.16), 0.6, metallic=0.4)
WOOD = kit.Look((0.72, 0.56, 0.36), 0.8)


def _prism(poly, z0, z1, look):
    """A flat polygon `poly` ((x, y) points, counterclockwise seen from +z) extruded from z0 to z1."""
    n = len(poly)
    pts = [(x, y, z0) for x, y in poly] + [(x, y, z1) for x, y in poly]
    counts, idx = [n, n], list(range(n - 1, -1, -1)) + list(range(n, 2 * n))
    for i in range(n):
        j = (i + 1) % n
        counts.append(4)
        idx += [i, j, n + j, n + i]
    return kit._mesh([np.array(p, float) for p in pts], counts, idx, look)


def _merge(meshes):
    """Meshes with the same look joined into one (fewer prims for the many small teeth)."""
    pts, counts, idx = [], [], []
    for m in meshes:
        off = len(pts)
        pts += [np.array(p, float) for p in m.points]
        counts += list(m.counts)
        idx += [i + off for i in m.indices]
    return kit._mesh(pts, counts, idx, meshes[0].look)


# --- The blade (a loose prop). Its frame: origin at its center, +y the way the teeth cut (the printed arrow), +x its
# toothed edge, +z its printed face.
def _arrow(cx, cy, z, look, length=0.050):
    """A printed arrow pointing +y, centered at (cx, cy) on the face at height z (z > 0 the top face, z < 0 the
    bottom one)."""
    t = 0.0002
    z0, z1 = (z, z + t) if z > 0 else (z - t, z)
    y0, y1, yh = cy - length / 2, cy + length / 2, cy + length / 2 - 0.012
    shaft = [(cx - 0.0015, y0), (cx + 0.0015, y0), (cx + 0.0015, yh), (cx - 0.0015, yh)]
    head = [(cx - 0.0045, yh), (cx + 0.0045, yh), (cx, y1)]
    return [_prism(shaft, z0, z1, look), _prism(head, z0, z1, look)]


def _blade():
    w, t = BLADE_W, BLADE_T
    edge = w / 2 - 0.0015  # the painted back runs to here; the last 1.5 mm is the bare tooth edge
    meshes = []
    hy = 0.002  # the holes are 4 mm across
    for y0, y1 in ((-BLADE_L / 2, -PIN_Y - hy), (-PIN_Y + hy, PIN_Y - hy), (PIN_Y + hy, BLADE_L / 2)):
        meshes.append(kit.box(((edge - w / 2) / 2, (y0 + y1) / 2, 0), (edge + w / 2, y1 - y0, t), BLUE))
    for py in (-PIN_Y, PIN_Y):  # either side of each hole
        for x0, x1 in ((-w / 2, -hy), (hy, edge)):
            meshes.append(kit.box(((x0 + x1) / 2, py, 0), (x1 - x0, 2 * hy, t), BLUE))
    # the bare tooth edge and its teeth: 18 per inch (1.41 mm), each raked toward +y (the way the blade cuts)
    pitch = 0.0254 / 18
    meshes.append(kit.box((edge + 0.0004, 0, 0), (0.0008, BLADE_L, t), TEETH))
    teeth = []
    for k in range(int(BLADE_L / pitch)):
        y = -BLADE_L / 2 + k * pitch
        teeth.append(_prism([(edge + 0.0008, y), (edge + 0.0008, y + pitch), (edge + 0.0020, y + pitch)],
                            -t / 2, t / 2, TEETH))
    meshes.append(_merge(teeth))
    arrows = []
    for cy in (-0.095, 0.0, 0.095):  # three printed arrows on each face, as the makers print them
        for z in (t / 2, -t / 2):
            arrows += _arrow(-0.0012, cy, z, WHITE)
    meshes.append(_merge(arrows))
    g, end = HOLE_GAP / 2, BLADE_L / 2 - PIN_Y - HOLE_GAP / 2
    cols = (kit.Collider((0, 0, 0), size=(w, 2 * (PIN_Y - g), BLADE_COL_T)),
            kit.Collider((0, PIN_Y + g + end / 2, 0), size=(w, end, BLADE_COL_T)),
            kit.Collider((0, -PIN_Y - g - end / 2, 0), size=(w, end, BLADE_COL_T)))
    return kit.Prop("blade", pos=BLADE_START, yaw=BLADE_YAW, size=(w, BLADE_L, BLADE_COL_T), mass=0.04,
                    meshes=tuple(meshes), colliders=cols)


# --- The frame, the vise and the blocks (all one fixed mechanism: the frame is clamped), in the frame's coordinates.
def _frame():
    m, c = [], []

    def part(center, size, look, collide=True):
        m.append(kit.box(center, size, look))
        if collide:
            c.append(kit.Collider(center, size=size))

    bx = THROAT + BAR_W / 2  # the back bar's center line
    part((bx, 0, H), (BAR_W, 2 * POST_Y + BAR_W, BAR_T), FRAME)
    for s in (-1, 1):  # the front (-y) and rear (+y) posts, from the back bar to the blade line
        part(((bx - 0.010) / 2, s * POST_Y, H), (bx + 0.010, BAR_W, BAR_T), FRAME)
        # the spigot on the post's inner face, with its pin standing up through the blade's hole
        part((0, s * (PIN_Y + 0.004), H - BLADE_COL_T / 2 - 0.004), (0.016, 0.020, 0.008), STEEL)
        m.append(kit.cylinder((0, s * PIN_Y, H + 0.0015), PIN_R, 0.009, CHROME, n=16))
        c.append(kit.Collider((0, s * PIN_Y, H + 0.0015), size=(2 * PIN_R, 2 * PIN_R, 0.009)))
    # the tension screw (M6) from the front spigot out through the front post, carrying the wing nut
    y_in, y_out = -PIN_Y - 0.008, NUT_Y - NUT_LEN / 2 - 0.004  # from the spigot to 4 mm past the nut
    m.append(kit.cylinder((0, (y_in + y_out) / 2, H), 0.003, y_in - y_out, CHROME, axis="y", n=16))
    # the pistol grip on the handle end, lying flat: from the rear post's back corner, raked toward the blade side
    grip = [(bx + BAR_W / 2, POST_Y - BAR_W / 2), (bx + 0.005, POST_Y + 0.060), (0.050, POST_Y + 0.135),
            (0.012, POST_Y + 0.120), (0.045, POST_Y + 0.040), (0.060, POST_Y - BAR_W / 2)]
    m.append(_prism(grip, H - 0.013, H + 0.013, GRIP))
    # the bench vise holding the back bar: 4-inch jaws along y, fixed jaw behind the bar, movable jaw in front
    jx0, jx1 = bx - BAR_W / 2, bx + BAR_W / 2
    part((jx1 + 0.012, 0, (H + 0.004) / 2 + 0.015), (0.024, 0.115, H + 0.004 - 0.030), VISE)  # fixed jaw body
    part((jx0 - 0.012, 0, (H + 0.004) / 2 + 0.015), (0.024, 0.115, H + 0.004 - 0.030), VISE)  # movable jaw body
    part((jx1 + 0.002, 0, H - 0.002), (0.004, 0.100, 0.014), DARK, collide=False)  # serrated steel jaw faces
    part((jx0 - 0.002, 0, H - 0.002), (0.004, 0.100, 0.014), DARK, collide=False)
    part((bx + 0.040, 0, 0.022), (0.110, 0.090, 0.034), VISE)  # the body and slide
    part((bx + 0.055, 0, 0.006), (0.150, 0.140, 0.012), VISE)  # the swivel base on the bench
    m.append(kit.cylinder((jx0 - 0.030, 0, 0.040), 0.007, 0.030, DARK, axis="x", n=16))  # the screw boss
    m.append(kit.cylinder((jx0 - 0.048, 0, 0.040), 0.0045, 0.110, CHROME, axis="y", n=12))  # the T-handle
    # two hardwood blocks for the new blade, under it 7 cm either side of its middle
    for s in (-1, 1):
        cx = BLADE_START[0] - SAW_X - s * 0.07 * math.sin(BLADE_YAW)
        cy = BLADE_START[1] - SAW_Y + s * 0.07 * math.cos(BLADE_YAW)
        part((cx, cy, 0.015), (0.045, 0.040, 0.030), WOOD)
    return m, c


def _wing_nut():
    """A cast wing nut on the tension screw, axis along y: a hub and two wings (lying flat at the start)."""
    meshes = [kit.cylinder((0, 0, 0), NUT_HUB_R, NUT_LEN, STEEL, axis="y", n=20)]
    for s in (-1, 1):
        meshes.append(kit.box((s * (NUT_SPAN / 4 + NUT_HUB_R * 0.3), 0, 0),
                              (NUT_SPAN / 2 - NUT_HUB_R * 0.6, NUT_LEN * 0.9, NUT_WING_T), STEEL))
    # one box: the hub is no thicker than the wings, so the fingers close on the wings' flats
    cols = (kit.Collider((0, 0, 0), size=(NUT_SPAN, NUT_LEN, NUT_WING_T)),)
    return tk.Cap("wingnut", anchor=(0.0, NUT_Y, H), mass=0.02, meshes=tuple(meshes), colliders=cols,
                  grip=(0.0, 0.0, 0.0), width=NUT_WING_T)


def _saw():
    m, c = _frame()
    return tk.Threaded("saw", pos=(SAW_X, SAW_Y, TOP), size=(0.25, 0.50, H + 0.01), meshes=tuple(m),
                       colliders=tuple(c), cap=_wing_nut(), thread=THREAD, axis="Y")


OBJECTS = [_saw(), _blade()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.fitted_slack = False  # the blade was on both pins while the wing nut was still short of snug
        self.home = None
        self.last_grip = None
        self.grips_still = False
        self.max_past = 0.0  # the most the nut was ever past snug with the blade on the pins (torn holes stay torn)
        self.off = 0  # steps in a row the blade has been off the pins


_H = _History()


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _on_pins(objects):
    """The blade is on both pins: each hole's center within 3.25 mm of a pin's axis, the blade down on the spigots
    (mid-plane within 2 mm of the seated height) and lying flat (within 5 degrees). Either end may be on either pin."""
    b = objects["blade"]
    r = _rot(b.quat_xyzw)
    if r[2, 2] < math.cos(math.radians(FLAT_DEG)):
        return False
    p = np.asarray(b.pos, float)
    holes = [p + r @ np.array([0.0, s * PIN_Y, 0.0]) for s in (-1, 1)]
    pins = [np.array([SAW_X, SAW_Y + s * PIN_Y, TOP + H]) for s in (-1, 1)]
    for order in ((0, 1), (1, 0)):
        if all(np.linalg.norm((holes[i] - pins[j])[:2]) < SEAT_XY and abs(holes[i][2] - pins[j][2]) < SEAT_Z
               for i, j in zip((0, 1), order)):
            return True
    return False


def _teeth_forward(objects):
    """The printed arrow (the way the teeth cut) points away from the handle (-y) and the toothed edge faces away from
    the back bar (-x), each within 20 degrees."""
    r = _rot(objects["blade"].quat_xyzw)
    lim = math.cos(math.radians(TEETH_DEG))
    return float(-r[1, 1]) > lim and float(-r[0, 0]) > lim


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
    on = _on_pins(objects)
    if _DEBUG:
        b = objects["blade"]
        print(f"[hacksaw] blade pos={np.round(np.asarray(b.pos, float) - (SAW_X, SAW_Y, TOP + H), 4)} "
              f"q={np.round(np.asarray(b.quat_xyzw, float), 3)} on={on} fwd={_teeth_forward(objects)} "
              f"past={tk.past_seat_deg(objects, NUT):.0f} grip={-tk.gripped_deg(objects, NUT):.0f} "
              f"unheld={tk.unheld_deg(objects, NUT):.0f} held={_held(objects)} score={_score(objects)} "
              f"rg={math.degrees(tk._grip_yaw(objects['right_gripper'], (0.0, 1.0, 0.0))):.0f} "
              f"rp={np.round(np.asarray(objects['right_gripper'].pos, float) - NUT_XYZ, 3)} "
              f"op={kit.opening(objects, 'right_gripper'):.2f} th={tk.held(objects, NUT)}", flush=True)
    past = tk.past_seat_deg(objects, NUT)
    _H.off = 0 if on else _H.off + 1
    if on:
        _H.max_past = max(_H.max_past, past)
    if past <= 0.0:
        # slack: the blade can go on or come off freely, so what counts is whether it is on now
        _H.fitted_slack = on
    elif _H.off >= OFF_STEPS:
        # past snug the blade is clamped between the pins: if it isn't on them (it was never fitted, or was fitted
        # after the nut went past snug, or came off), the nut was tightened on nothing, and only backing it off to
        # slack and fitting the blade then counts again
        _H.fitted_slack = False


def _near_segment(p, a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    t = float(np.clip(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0))
    return np.linalg.norm(p - (a + t * (b - a))) < r


def _held(objects):
    """A gripper with its fingers under 60 % open and its tool point within 2 cm of the blade's center line or of the
    wing nut (plus its half-span)."""
    b = objects["blade"]
    r = _rot(b.quat_xyzw)
    p = np.asarray(b.pos, float)
    a, e = p - r[:, 1] * BLADE_L / 2, p + r[:, 1] * BLADE_L / 2
    nut = np.asarray(objects[NUT].pos, float)
    for g in ("left_gripper", "right_gripper"):
        if g not in objects or kit.opening(objects, g) >= HOLD_OPEN:
            continue
        q = np.asarray(objects[g].pos, float)
        if _near_segment(q, a, e, HOLD_MARGIN) or np.linalg.norm(q - nut) < HOLD_MARGIN + NUT_SPAN / 2:
            return True
    return False


def _arms_home(objects):
    """Each gripper's tool point within 3 cm of where it was at the trial's first scored step, and still."""
    if not _H.home or not _H.grips_still:
        return False
    return all(np.linalg.norm(np.asarray(objects[g].pos, float) - p) < HOME_TOL for g, p in _H.home.items()
               if g in objects)


def _score(objects):
    past = tk.past_seat_deg(objects, NUT)
    # the blade isn't on both pins: nothing is fitted
    if not _on_pins(objects):
        return 0.0
    # fitted the wrong way round (teeth toward the handle or toward the back bar): the saw would cut on the pull
    if not _teeth_forward(objects):
        return 0.0
    # still holding the blade or the wing nut: the job isn't finished
    if _held(objects):
        return 0.0
    if past <= 0.0:
        return 0.25  # on the pins the right way, but slack: the wing nut was never turned past snug
    # tensioned without the blade on the pins (it was put on after the nut went past snug), or by knocking or swiping
    # the nut round rather than turning it with a grip
    if not _H.fitted_slack:
        return 0.0
    if (-tk.gripped_deg(objects, NUT) < (past - THREAD.start) - GRIP_SLACK
            or tk.unheld_deg(objects, NUT, tightening=True) > GRIP_SLACK):
        return 0.0
    # past 2.5 turns at any point with the blade on: the pins tore its holes, and backing the nut off doesn't undo that
    if past > OVER_DEG or _H.max_past > OVER_DEG:
        return 0.25
    if past < TAUT_DEG:
        return 0.5  # under a full turn past snug: slack
    return 1.0 if (_arms_home(objects) or not REQUIRE_ARMS_HOME) else 0.9


def success(objects):
    """Full score: the blade on both pins with its teeth pointing away from the handle, the wing nut 1 to 2.5 turns
    past snug, let go, and (with REQUIRE_ARMS_HOME) both arms home and still. Called first each step, so it also
    records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 unless the blade ends on both pins (each hole within 3.25 mm of its pin, down on the spigots, flat) with its
    printed arrow pointing away from the handle and its teeth facing away from the back bar (within 20 degrees), and
    no gripper still holding the blade or the wing nut. 0.25 if the wing nut was never turned past snug. 0 if the nut
    went past snug while the blade was not on both pins (fitting it afterward does not count), or if more than 90
    degrees of the nut's tightening was done without a gripper holding it and turning with it (twistkit). Otherwise
    0.5 if under one turn past snug (slack), 0.25 if it was ever over 2.5 turns past snug with the blade on (overtight,
    even if backed off after), and 1.0 from one to 2.5
    turns with both arms home and still (0.9 if not, with REQUIRE_ARMS_HOME).
    Possible scores: 0, 0.25, 0.5, 0.9, 1."""
    return _score(objects)


# --- Demo ----------------------------------------------------------------------------------------
PICK_Z = BLADE_START[2]  # tool point at the blade's mid-plane: the pads close on its edges
PLACE_Z = TOP + H + 0.004  # let go 4 mm above the seated height; the blade drops onto the spigots
CARRY_Z = TOP + 0.17
RIDE = (-0.0007, 0.0016)  # where the blade's middle ends up relative to the tool point when let go (from a run)
NUT_XYZ = (SAW_X, SAW_Y + NUT_Y, TOP + H)
NUT_ROLL = math.pi / 2  # the right hand's grip turned to point along +y, end-on to the wing nut
TURN = 760.0  # half a turn to snug, then about 1.5 turns past it (mid-window); a run ended 576 degrees past snug
STROKE = 180.0


def _nut_strokes(t_wait):
    """Right arm: from its side, point the hand along +y at the wing nut, pinch the wings (flat at the start, so the
    fingers close top and bottom), roll clockwise seen from the nut's outside (the grip's yaw going down) 180 degrees,
    let go, roll back open, and repeat; then back out along -y and go home. A parallel grip matches the wings every
    180 degrees, so each stroke starts at the same yaw."""
    x, y, z = NUT_XYZ
    hi = 180.0  # wings flat match the fingers closing vertically at grip yaw 0 or 180 (pointing along +y)
    out = [Waypoint(t_wait, (0.24, -0.24, TOP + 0.18), FINGER_OPEN, "wait for the blade"),
           kit.Tilted(2.0, (x - 0.03, y - 0.03, z + 0.08), FINGER_OPEN, "turn the hand toward the nut",
                      math.radians(hi), NUT_ROLL),
           kit.Tilted(2.0, (x, y - 0.025, z), FINGER_OPEN, "behind the wing nut", math.radians(hi), NUT_ROLL),
           kit.Tilted(1.5, (x, y, z), FINGER_OPEN, "fingers around the wings", math.radians(hi), NUT_ROLL)]
    left = TURN
    step = 15.0
    while left > 1e-6:
        s = min(STROKE, left)
        out.append(kit.Tilted(0.7, (x, y, z), FINGER_CLOSED, "pinch the wings", math.radians(hi), NUT_ROLL))
        n = max(1, math.ceil(s / step))
        for k in range(1, n + 1):
            out.append(kit.Tilted(0.3, (x, y, z), FINGER_CLOSED, "turn the wing nut",
                                  math.radians(hi - s * k / n), NUT_ROLL))
        out.append(kit.Tilted(0.6, (x, y, z), FINGER_OPEN, "let go", math.radians(hi - s), NUT_ROLL))
        left -= s
        if left > 1e-6:
            for k in range(1, n + 1):
                out.append(kit.Tilted(0.25, (x, y, z), FINGER_OPEN, "roll back", math.radians(hi - s + s * k / n),
                                      NUT_ROLL))
        else:
            out.append(kit.Tilted(1.0, (x, y - 0.025, z), FINGER_OPEN, "back off the nut", math.radians(hi - s),
                                  NUT_ROLL))
            out.append(kit.Tilted(1.5, (x - 0.03, y - 0.03, z + 0.08), FINGER_OPEN, "up clear of the saw",
                                  math.radians(hi - s), NUT_ROLL))
    return out


def demo():
    """Left arm: pinch the new blade at its middle across its width, lift it, turn it 150 degrees as it is carried
    over the frame so its arrow points away from the handle with its teeth toward the robot, lower it over the two
    pins and let go, then go home. Right arm, once the left is home: tighten the wing nut 760 degrees with an end-on
    regrasp loop, let go and go home. Both end with a hold at home."""
    bx, by, _ = BLADE_START
    px, py = SAW_X - RIDE[0], SAW_Y - RIDE[1]
    y0, y1 = math.radians(60.0), math.radians(-90.9)  # fingers across the blade at the pick and at the pins (the
    # blade lags the wrist by 0.9 degrees as it is carried; from a run)
    left = [Waypoint(2.0, (bx, by, PICK_Z + 0.06), FINGER_OPEN, "over the new blade", y0),
            Waypoint(1.5, (bx, by, PICK_Z), FINGER_OPEN, "around the blade's middle", y0),
            Waypoint(0.8, (bx, by, PICK_Z), FINGER_CLOSED, "pinch the blade", y0),
            Waypoint(1.5, (bx, by, CARRY_Z), FINGER_CLOSED, "lift it", y0),
            Waypoint(1.5, ((bx + px) / 2, (by + py) / 2, CARRY_Z), FINGER_CLOSED, "turn it end for end", (y0 + y1) / 2),
            Waypoint(1.5, (px, py, CARRY_Z), FINGER_CLOSED, "over the pins", y1),
            Waypoint(2.0, (px, py, PLACE_Z + 0.015), FINGER_CLOSED, "lower it", y1),
            Waypoint(1.5, (px, py, PLACE_Z), FINGER_CLOSED, "holes over the pins", y1),
            Waypoint(0.7, (px, py, PLACE_Z), FINGER_OPEN, "let go", y1),
            Waypoint(1.0, (px, py, PLACE_Z + 0.06), FINGER_OPEN, "lift clear", y1),
            kit.Home(2.0)]
    t_left = sum(w.duration for w in left)
    right = _nut_strokes(t_left) + [kit.Home(2.5), kit.Home(2.0, label="hold at home")]
    total = sum(w.duration for w in right)
    left.append(kit.Home(max(total - t_left, 0.5), label="hold at home"))
    return {"left": left, "right": right}
