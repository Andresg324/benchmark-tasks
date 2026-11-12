"""A reservoir water level: a clear 1-liter jar of water with a carry handle on its lid (the reservoir) and a clear end
tube taped to a wooden stick, joined by a clear hose coiled on the table, each standing in a stand. A datum post on the
left carries a black datum line; a far post on the right, standing on a pad 2.5 cm higher than the table (the floor
there is not level), carries a red sliding marker and a small shelf for the end tube.

The job is to carry the datum line's height across to the far post (Fine Homebuilding, "Making and Using Water
Levels", 1994; JLC, "Making and Using a Water Level"): the water in the jar and in the end tube always stands at one
height, so the end tube is set at the far post, the jar is held beside the datum post and raised or lowered until its
water sits on the line, and the marker is slid to the end tube's water surface. The jar has 55 times the end tube's
cross-section, so moving the end tube barely shifts the water (4 cm moves it 0.7 mm) while the water follows the jar
almost 1:1; the jar has nowhere to stand at the line's height, so one hand holds it on the line while the other, 45 cm
away at the far post, slides the marker. The end tube's mouth is 12 cm above its stand's floor and the line is at 13
cm, so the jar can only go up to the line once the end tube is up on the far post's shelf, and must come down again
before the end tube goes back into its stand.

Model (levelhose): the water is not simulated. Fixed volume, one surface height wherever both ends are open, found
from the ends' poses every step; a mouth below that height overflows (Torricelli) and what runs out is gone for good,
drawn as a stream and a puddle.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props; nothing on disk changes)
import guardkit as gk  # noqa: E402  (grip, release and arms-home checks)
import levelhose as lh  # noqa: E402  (the water in the level, updated every control step)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = ("Use the water level to mark the height of the datum post's black line on the far post with the red "
               "marker, then put the jar and the hose end back in their stands.")
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
POST_X = 0.30  # both posts' centers (x)
DATUM_Y, FAR_Y = 0.235, -0.235  # the datum post (left) and the far post (right)
POST = 0.04  # posts 4 cm square ...
POST_H = 0.18  # ... and 18 cm tall above the table (their tops under the arms as they reach in)
PAD = (0.12, 0.12, 0.025)  # the far post stands on a 2.5 cm pad: the floor there is higher
DATUM_Z = TOP + 0.130  # the datum line's height (13.0 cm above the table)

# the reservoir: a clear round jar 10 cm across and 10 cm tall (9.4 cm inside), a vented lid with a carry handle
# across it, the hose leaving through a fitting in its bottom
JAR_R, JAR_H, JAR_IN = 0.05, 0.10, 0.047  # outside radius, height, inside radius
JAR_FLOOR = 0.003  # its bottom wall
HANDLE = ((0.0, 0.0, JAR_H + 0.015), (0.05, 0.025, 0.03))  # (center, size): a 2.5 cm wide grip bar 3 cm tall
JAR_MASS = 0.15  # kg in the sim (a real jar with 0.4 L of water about 0.5 kg)

# the far end: a 2.5 cm square pine stick 15 cm long on a 4 cm square foot, a clear tube taped to its front face
STICK = (0.025, 0.025, 0.15)
FOOT = (0.04, 0.04, 0.008)
TUBE_R = 0.008  # clear acrylic tube, 16 mm outside ...
BORE_R = 0.00635  # ... 12.7 mm (1/2 in) bore: 1.27 mL per cm
TUBE_X = -(STICK[0] / 2 + TUBE_R + 0.0045)  # the tube's axis in the stick's frame: on its front face (-x)
TUBE_Z = (0.03, 0.12)  # the tube from 3 cm to 12 cm above the stick's bottom
END_MASS = 0.08  # kg: stick, foot, tube and its water
HOSE = lh.Hose(radius=0.0048, length=1.2)  # 3/8 in bore clear vinyl, 1.2 m
FILL = TOP + 0.060  # the water 6 cm above the table with both in their stands

# stands on the table (start and finish) and the far post's shelf: floors with a 1 cm rim
RIM_H, RIM_T = 0.01, 0.004
JAR_STAND = (0.22, 0.14)  # the jar's stand, 12 cm square inside
JAR_IN_SQ = (0.12, 0.12)
END_STAND = (0.22, -0.14)  # the end's stand, 8 cm square inside
END_IN_SQ = (0.08, 0.08)
SHELF_IN = (0.052, 0.056)
SHELF_TOP = TOP + 0.04  # the far post's shelf floor: 4 cm above the table
HOLD_Y = POST / 2 + SHELF_IN[1] / 2  # the end on the shelf: its stick's center this far inboard of the post's
JAR_SPOT = np.array([POST_X, DATUM_Y - POST / 2 - 0.01 - JAR_R])  # the jar held 1 cm from the datum post

# the red marker on the far post: a collar on the post's front face, a grip tab, and a pointer across the end's tube;
# its pointer's top edge is the mark (link origin = the mark)
MARK_START = TOP + 0.092  # 3.8 cm below the datum height
MARK_FACE = POST_X - POST / 2 - 0.0015  # world x of the marker's back
TAB = ((-0.0285, -0.008, -0.01), (0.045, 0.024, 0.02))  # (center, size) in the marker's frame: 4.5 cm long, on
# the outboard side, away from the tube

PINE = kit.Look((0.86, 0.72, 0.50), 0.7)
PLY = kit.Look((0.70, 0.62, 0.50), 0.8)
DARK = kit.Look((0.12, 0.12, 0.12), 0.6)
RED = kit.Look((0.85, 0.12, 0.10), 0.5)
CLEAR = kit.Look((0.90, 0.95, 1.00), 0.05, 0.0, 0.25)
TUBING = kit.Look((0.55, 0.75, 0.95), 0.3)
GREY = kit.Look((0.55, 0.55, 0.55), 0.8)
BLACK = kit.Look((0.08, 0.08, 0.08), 0.5)


def _rim(cx, cy, z0, inside, meshes, colliders, open_side=None):
    """A 1 cm rim around a floor `inside` (x, y) at height z0 (in the prop's frame) centered at (cx, cy); `open_side`
    "-y" leaves that side for a post's face."""
    ix, iy = inside
    parts = [((cx - ix / 2 - RIM_T / 2, cy), (RIM_T, iy + 2 * RIM_T)), ((cx + ix / 2 + RIM_T / 2, cy), (RIM_T, iy + 2 * RIM_T))]
    for s in (-1, 1):
        if open_side != ("+y" if s > 0 else "-y"):
            parts.append(((cx, cy + s * (iy / 2 + RIM_T / 2)), (ix, RIM_T)))
    for (x, y), (sx, sy) in parts:
        c, size = (x, y, z0 + RIM_H / 2), (sx, sy, RIM_H)
        meshes.append(kit.box(c, size, PLY))
        colliders.append(kit.Collider(c, size=size))


def _stands():
    m, c = [], []
    _rim(0.0, JAR_STAND[1], 0.0, JAR_IN_SQ, m, c)
    _rim(0.0, END_STAND[1], 0.0, END_IN_SQ, m, c)
    for r in (0.040, 0.050, 0.060):  # the hose coil (no collider: the arms pass over it)
        path = [(r * math.cos(t), r * math.sin(t), 0.0055) for t in np.linspace(0, 2 * math.pi, 49)]
        m.append(kit.band(path, 0.011, 0.011, TUBING, normal=(0, 0, 1)))
    return kit.Prop("stands", pos=(JAR_STAND[0], 0.0, TOP), size=(0.12, 0.40, RIM_H), meshes=tuple(m),
                    colliders=tuple(c))


def _datum_post():
    h = POST_H
    m = [kit.box((0, 0, h / 2), (POST, POST, h), PINE),
         kit.box((0, 0, DATUM_Z - TOP), (POST + 0.001, POST + 0.001, 0.002), DARK)]  # the datum line, all round
    return kit.Prop("datum_post", pos=(POST_X, DATUM_Y, TOP), size=(POST, POST, h), meshes=tuple(m),
                    colliders=(kit.Collider((0, 0, h / 2), size=(POST, POST, h)),))


def _far_post():
    m, c = [], []
    pad = ((0, 0, PAD[2] / 2), PAD)
    post = ((0, 0, PAD[2] + (POST_H - PAD[2]) / 2), (POST, POST, POST_H - PAD[2]))
    shelf_y = POST / 2 + (SHELF_IN[1] + RIM_T) / 2  # the shelf on the post's inboard (+y) face
    shelf = ((0, shelf_y, SHELF_TOP - TOP - 0.005), (SHELF_IN[0] + 2 * RIM_T, SHELF_IN[1] + RIM_T, 0.01))
    for (cen, size), look in ((pad, GREY), (post, PINE), (shelf, PLY)):
        m.append(kit.box(cen, size, look))
        c.append(kit.Collider(cen, size=size))
    _rim(0.0, POST / 2 + SHELF_IN[1] / 2, SHELF_TOP - TOP, SHELF_IN, m, c, open_side="-y")
    return kit.Prop("far_post", pos=(POST_X, FAR_Y, TOP), size=(PAD[0], 0.17, POST_H), meshes=tuple(m),
                    colliders=tuple(c))


def _marker():
    """The red marker on the far post's front face (a fixed-base mechanism whose base is a small block inside the
    post: the post, pad and shelf are a fixed prop, so they stop what is set on them)."""
    fx = MARK_FACE - POST_X  # 1.5 mm in front of the post's front face: it slides without rubbing
    ptr_x = TUBE_X + POST / 2 - TUBE_R - 0.0015  # the pointer passes just in front of the end's tube
    marker_m = [kit.box((-0.003, 0.0, -0.0075), (0.006, POST + 0.006, 0.015), RED),  # collar plate on the front face
                kit.box(*TAB, RED),
                kit.box((ptr_x, POST / 2 + 0.0175, -0.0015), (0.002, 0.045, 0.003), RED),  # pointer; top edge = mark
                kit.box(((ptr_x - 0.006) / 2, POST / 2 + 0.003, -0.0015), (-0.006 - ptr_x, 0.006, 0.003), RED)]
    marker_c = [kit.Collider((-0.003, 0.0, -0.0075), size=(0.006, POST + 0.006, 0.015)), kit.Collider(*TAB)]
    marker = kit.Link("marker", "prismatic", "Z", (fx, 0.0, DATUM_Z - TOP), lower=0.07 - (DATUM_Z - TOP),
                      upper=0.165 - (DATUM_Z - TOP), start=MARK_START - DATUM_Z, mass=0.10, meshes=tuple(marker_m),
                      colliders=tuple(marker_c), damping=2.0, floating=True, drag=1.6)
    core = ((0, 0, POST_H - 0.02), (0.02, 0.02, 0.02))  # hidden inside the post's top
    return kit.Mechanism("rail", pos=(POST_X, FAR_Y, TOP), size=(0.02, 0.02, POST_H), meshes=(),
                         colliders=(kit.Collider(*core),), links=(marker,))


def _jar():
    m = [kit.cylinder((0, 0, JAR_H / 2), JAR_R, JAR_H, CLEAR),
         kit.cylinder((0, 0, JAR_H - 0.004), JAR_R + 0.001, 0.008, CLEAR),  # the lid
         kit.box(*HANDLE, BLACK),
         kit.cylinder((0, 0, -0.002), 0.008, 0.004, TUBING)]  # the hose fitting under its bottom
    c = [kit.Collider((0, 0, JAR_H / 2), radius=JAR_R, height=JAR_H, axis="Z"), kit.Collider(*HANDLE)]
    return kit.Prop("jar", pos=(JAR_STAND[0], JAR_STAND[1], TOP), size=(2 * JAR_R, 2 * JAR_R, JAR_H + 0.03),
                    mass=JAR_MASS, meshes=tuple(m), colliders=tuple(c))


def _end():
    sx, sy, sz = STICK
    tz = (TUBE_Z[0] + TUBE_Z[1]) / 2
    m = [kit.box((0, 0, sz / 2), STICK, PINE), kit.box((0, 0, FOOT[2] / 2), FOOT, PINE),
         kit.cylinder((TUBE_X, 0, tz), TUBE_R, TUBE_Z[1] - TUBE_Z[0], CLEAR),
         kit.cylinder((TUBE_X, 0, TUBE_Z[0] - 0.004), TUBE_R * 0.75, 0.008, TUBING)]  # the hose barb
    for z in (TUBE_Z[0] + 0.012, TUBE_Z[1] - 0.012):  # two wraps of black tape
        m.append(kit.box(((TUBE_X - TUBE_R) / 2, 0, z), (sx / 2 - TUBE_X + TUBE_R + 0.001, sy + 0.002, 0.006), DARK))
    c = [kit.Collider((0, 0, sz / 2), size=STICK), kit.Collider((0, 0, FOOT[2] / 2), size=FOOT),
         kit.Collider((TUBE_X, 0, tz), size=(2 * TUBE_R, 2 * TUBE_R, TUBE_Z[1] - TUBE_Z[0]))]
    return kit.Prop("far_end", pos=(END_STAND[0], END_STAND[1], TOP), size=(FOOT[0] - TUBE_X + TUBE_R, FOOT[1], sz),
                    mass=END_MASS, meshes=tuple(m), colliders=tuple(c))


JAR, END = "jar", "far_end"
OBJECTS = [_stands(), _datum_post(), _far_post(), _marker(), _jar(), _end()]

lh.setup([lh.End(JAR, bottom=(0.0, 0.0, JAR_FLOOR), top=(0.0, 0.0, JAR_H), radius=JAR_IN),
          lh.End(END, bottom=(TUBE_X, 0.0, TUBE_Z[0]), top=(TUBE_X, 0.0, TUBE_Z[1]), radius=BORE_R)],
         hose=HOSE, fill_to=FILL, floor_z=TOP)

# --- The rule's numbers (reasons in score() and the README).
ON_LINE = 0.0015  # m: the jar's water within 1.5 mm of the line while the marker moves (a meniscus read on a 2 mm line)
ON_SURFACE = 0.0015  # m: the marker's edge within 1.5 mm of the end's water surface when it moves
FULL = 0.0015  # m: the mark within 1.5 mm (a sixteenth of an inch) of the datum height: carpentry tolerance
MOVED = 0.0005  # m: the marker counts as moved once its edge is 0.5 mm from where it was last counted
SPILL_MAX = 1.0  # mL: about 20 drops; a careful transfer spills none
AT_JAR = 0.03  # m: the jar's axis within 3 cm of its spot 1 cm off the datum post (its water read against the line)
AT_END = 0.025  # m: the end's tube within 2.5 cm of its spot behind the marker's pointer
UPRIGHT_READ = 15.0  # degrees from vertical: the jar or the end held up to be read
UPRIGHT_DOWN = 10.0  # degrees: standing set down
IN_STAND = 0.025  # m: set back in its stand: within 2.5 cm of the stand's center, its bottom within 5 mm of the table
STILL = 0.01  # m/s

END_SPOT = np.array([POST_X + TUBE_X, FAR_Y + HOLD_Y])
JAR_PARTS = (gk.Seg((0, 0, 0), (0, 0, JAR_H), JAR_R), gk.Block(*HANDLE))
HANDLE_PART = gk.Block(*HANDLE)
HANDLE_BAND = gk.band(HANDLE[1][1])  # fingers closed on the 2.5 cm bar
END_PARTS = (gk.Block((0, 0, STICK[2] / 2), STICK), gk.Block((0, 0, FOOT[2] / 2), FOOT),
             gk.Seg((TUBE_X, 0, TUBE_Z[0]), (TUBE_X, 0, TUBE_Z[1]), TUBE_R))
MARKER = "rail/marker"
MARKER_PARTS = (gk.Block(*TAB), gk.Block((-0.003, 0.0, -0.0075), (0.006, POST + 0.006, 0.015)))
HOME = gk.ArmsHome()
TAUT = gk.Latch(steps=2)


def _tilt(objects, name):
    """Degrees of a prop's axis from vertical."""
    z = lh._rotm(objects[name].quat_xyzw)[:, 2]
    return math.degrees(math.acos(max(-1.0, min(1.0, float(z[2])))))


def mark(objects):
    """World z of the marker's edge (the mark)."""
    return float(objects[MARKER].pos[2])


def _end_tube_xy(objects):
    o = objects[END]
    return (np.asarray(o.pos, float) + lh._rotm(o.quat_xyzw) @ np.array([TUBE_X, 0.0, 0.0]))[:2]


def reading(objects):
    """This step is a transfer: the jar beside the datum post, upright, gripped by its handle, its water on the line;
    the end at the far post, upright; the marker's edge on the end's water surface; the hand on the jar not the one on
    the marker."""
    jar = objects[JAR]
    hands = gk.grip_on(objects, JAR, HANDLE_PART, HANDLE_BAND)
    on_marker = gk.holding(objects, MARKER, MARKER_PARTS)
    return bool(any(h not in on_marker for h in hands)
                and float(np.linalg.norm(np.asarray(jar.pos, float)[:2] - JAR_SPOT)) <= AT_JAR
                and _tilt(objects, JAR) <= UPRIGHT_READ
                and abs(lh.surface_at(objects, JAR) - DATUM_Z) <= ON_LINE
                and float(np.linalg.norm(_end_tube_xy(objects) - END_SPOT)) <= AT_END
                and _tilt(objects, END) <= UPRIGHT_READ
                and abs(lh.surface_at(objects, END) - mark(objects)) <= ON_SURFACE)


class _Marking(gk._PerStep):
    """Per trial, latched: whether the step at which the marker last moved was a transfer."""

    def _fresh(self):
        self.ref, self.read = None, False

    def feed(self, objects):
        if not self._tick(objects):
            return
        z = mark(objects)
        if self.ref is None:
            self.ref = z
        elif abs(z - self.ref) > MOVED:
            self.ref, self.read = z, reading(objects)


MARKING = _Marking()


def in_stand(objects, name, stand):
    """`name` let go (every tool point and pad tip 1 cm clear), standing in its stand within 10 degrees of upright,
    still."""
    o = objects[name]
    parts = JAR_PARTS if name == JAR else END_PARTS
    p = np.asarray(o.pos, float)
    return (float(np.linalg.norm(p[:2] - np.array(stand))) <= IN_STAND and abs(p[2] - TOP) <= 0.005
            and gk.released_clear(objects, name, parts, 0.01) and _tilt(objects, name) <= UPRIGHT_DOWN
            and float(np.linalg.norm(o.lin_vel)) < STILL)


def _job(objects):
    MARKING.feed(objects)
    TAUT.feed(objects, lh.taut(objects))
    if lh.spilled(objects) > SPILL_MAX or TAUT.on:
        return 0.0  # water out of a mouth (an end below the water, the jar tipped) or the hose pulled taut
    if not (in_stand(objects, JAR, JAR_STAND) and in_stand(objects, END, END_STAND)):
        return 0.0  # the jar or the end still held, not back in its stand, lying down or moving
    if not gk.released_clear(objects, MARKER, MARKER_PARTS, 0.01):
        return 0.0  # the marker still held
    if not MARKING.read:
        return 0.0  # never moved, or last moved without a transfer (the jar off its line or not held, the end away)
    return 1.0 if abs(mark(objects) - DATUM_Z) <= FULL else 0.0


def _score(objects):
    HOME.feed(objects)
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not HOME.home(objects):
        return 0.9
    return job


def success(objects):
    return _score(objects) == 1.0


def score(objects):
    """1.0 if the red marker's edge ends within 1.5 mm (a sixteenth of an inch, carpentry tolerance) of the datum line's height and it
    was set from a transfer: at the last step it moved (by more than 0.5 mm), the jar stood within 3 cm of its spot
    beside the datum post, within 15 degrees of upright, gripped by its handle by a hand other than one on the marker,
    with its water within 1.5 mm of the line; the end's tube stood within 2.5 cm of its spot behind the marker's
    pointer, within 15 degrees of upright; and the marker's edge was within 1.5 mm of the end's water surface. Also no
    more than 1 mL spilled, the hose never pulled taut, the jar and the end back in their stands (within 2.5 cm),
    upright within 10 degrees, still and let go (every tool point and pad tip 1 cm clear), the marker let go, and both
    arms back home and still (0.9 if not home). Otherwise 0: nothing done, the marker last moved without such a
    transfer (the jar off its line, set down or in the marker hand, the end not at the far post), the mark more than
    1.5 mm off, more than 1 mL spilled (the end left in its stand or set back while the jar is up, the jar tipped), or
    anything still held or not back in its stand. Possible scores: 0, 0.9, 1.0."""
    return _score(objects)


# --- Demo: the right hand lifts the end out of its stand onto the far post's shelf; the left lifts the jar from its
# stand and holds it beside the datum post with its water on the line; the right slides the marker up to the end's
# water surface and lets go; the left sets the jar back in its stand; the right puts the end back in its stand; home.
GRIP_DZ = 0.138  # tool point above the stick's bottom when gripping its top (pad tips 1.3 cm lower, above the mouth)
JAR_DZ = JAR_H + 0.018  # tool point above the jar's bottom when gripping its handle (pad tips 5 mm over the lid)
CARRY = 0.058  # stick bottom while carrying (1.8 cm over the shelf's floor, 8 mm over its rim)
UP = 0.04
A_RATIO = (BORE_R / JAR_IN) ** 2  # the end tube's cross-section over the jar's
# the jar's bottom where its water sits on the line with the end on the shelf: the jar's water depth changes by what
# the end tube's column gains or loses between the stand and the shelf
JAR_HOLD = (DATUM_Z - FILL) + A_RATIO * ((DATUM_Z - SHELF_TOP - TUBE_Z[0]) - (FILL - TOP - TUBE_Z[0]))
JAR_CARRY = JAR_HOLD + 0.01  # carried 1 cm higher (the water 2 cm under the end's mouth on the shelf)
JAR_TRIM = 0.0032  # m: correction to the jar's hold height, from the sim (a held load rides low)
MARK_TRIM = 0.0  # m: correction to the marker grip's lift, from the sim


def _p(x, y, z_bottom, dz=GRIP_DZ):
    return (x, y, TOP + z_bottom + dz)


def _j(x, y, z_bottom):
    return _p(x, y, z_bottom, JAR_DZ)


def demo():
    """End to the far shelf; jar up beside the datum post, its water on the line; marker set; jar back; end back; home."""
    yr = FAR_Y + HOLD_Y
    jx, jy = JAR_SPOT
    sx, sy = JAR_STAND
    ex, ey = END_STAND
    tx = MARK_FACE + TAB[0][0] - 0.012  # 1 cm from the tab's front end: the hand's body clears the post's face
    z_tab = MARK_START - 0.01  # tool point at the tab's middle
    z_set = z_tab + DATUM_Z - MARK_START + MARK_TRIM
    hold = JAR_HOLD + JAR_TRIM
    z_high = TOP + CARRY + 0.02 + GRIP_DZ  # the pad tips 1 cm over the post's top and 4 cm over the end's tube
    HALF = FINGER_OPEN * 0.42  # fingers 3.9 cm apart around the 2.4 cm tab: fully open they reach the end's tube
    ty = FAR_Y + TAB[0][1]  # the tab's center (y)
    left = [Waypoint(9.0, _j(sx, sy, UP), FINGER_OPEN, "wait for the end on the shelf, over the jar"),
            Waypoint(1.2, _j(sx, sy, 0.0), FINGER_OPEN, "around its handle"),
            Waypoint(0.6, _j(sx, sy, 0.0), FINGER_CLOSED, "grip"),
            Waypoint(1.5, _j(sx, sy, JAR_CARRY), FINGER_CLOSED, "lift"),
            Waypoint(2.5, _j(jx, jy, JAR_CARRY), FINGER_CLOSED, "beside the datum post"),
            Waypoint(1.5, _j(jx, jy, hold), FINGER_CLOSED, "water down to the line"),
            Waypoint(9.0, _j(jx, jy, hold), FINGER_CLOSED, "hold on the line"),
            Waypoint(2.5, _j(sx, sy, JAR_CARRY), FINGER_CLOSED, "back over its stand"),
            Waypoint(2.5, _j(sx, sy, 0.0), FINGER_CLOSED, "down into the stand"),
            Waypoint(0.6, _j(sx, sy, 0.0), FINGER_OPEN, "let go"),
            Waypoint(1.0, _j(sx, sy, UP), FINGER_OPEN, "clear")]
    q = math.pi / 2  # the end is gripped with the fingers closing along x: opened along y beside the post they hit it
    right = [Waypoint(2.5, _p(ex, ey, UP), FINGER_OPEN, "over the stick", q),
             Waypoint(1.2, _p(ex, ey, 0.0), FINGER_OPEN, "around the stick's top", q),
             Waypoint(0.6, _p(ex, ey, 0.0), FINGER_CLOSED, "grip", q),
             Waypoint(1.5, _p(ex, ey, CARRY), FINGER_CLOSED, "lift", q),
             Waypoint(2.0, _p(POST_X, yr, CARRY), FINGER_CLOSED, "over the far post's shelf", q),
             Waypoint(1.2, _p(POST_X, yr, SHELF_TOP - TOP), FINGER_CLOSED, "onto the shelf", q),
             Waypoint(0.6, _p(POST_X, yr, SHELF_TOP - TOP), FINGER_OPEN, "let go", q),
             Waypoint(1.0, _p(POST_X, yr, CARRY + 0.02), FINGER_OPEN, "up off the stick", q),
             Waypoint(2.0, (tx - 0.015, yr, z_high), HALF, "out in front, over the tube"),
             Waypoint(1.0, (tx - 0.015, ty, z_high), HALF, "over the marker's tab"),
             Waypoint(1.5, (tx, ty, z_tab), HALF, "down around the tab"),
             Waypoint(0.6, (tx, ty, z_tab), FINGER_CLOSED, "grip the tab"),
             Waypoint(2.0, (tx, ty, z_tab), FINGER_CLOSED, "wait for the jar on the line"),
             Waypoint(3.0, (tx, ty, z_set), FINGER_CLOSED, "slide it to the water"),
             Waypoint(1.5, (tx, ty, z_set), FINGER_CLOSED, "hold on the water"),
             Waypoint(0.6, (tx, ty, z_set), HALF, "let go"),
             Waypoint(1.0, (tx - 0.015, ty, z_high), HALF, "up off the tab"),
             Waypoint(4.0, _p(POST_X, yr, SHELF_TOP - TOP + UP), FINGER_OPEN, "over the end, the jar going down", q),
             Waypoint(1.2, _p(POST_X, yr, SHELF_TOP - TOP), FINGER_OPEN, "around its stick", q),
             Waypoint(0.6, _p(POST_X, yr, SHELF_TOP - TOP), FINGER_CLOSED, "grip", q),
             Waypoint(1.5, _p(POST_X, yr, CARRY), FINGER_CLOSED, "lift", q),
             Waypoint(2.0, _p(ex, ey, CARRY), FINGER_CLOSED, "back over its stand", q),
             Waypoint(1.5, _p(ex, ey, 0.0), FINGER_CLOSED, "down into the stand", q),
             Waypoint(0.6, _p(ex, ey, 0.0), FINGER_OPEN, "let go", q),
             Waypoint(1.0, _p(ex, ey, UP), FINGER_OPEN, "clear", q)]
    tail = [kit.Home(2.0), kit.Home(SUCCESS_HOLD_S + 1.0, label="hold home")]
    return {"left": left + tail, "right": right + list(tail)}
