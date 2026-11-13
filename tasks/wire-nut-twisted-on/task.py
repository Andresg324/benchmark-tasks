"""Two black 12 AWG copper conductors stand up out of an open 4-11/16-inch-square steel junction box, stripped 16 mm at
their ends and bent so that their tips meet side by side. A red twist-on wire connector (a wire nut) has been pushed
onto the tips by hand but not turned, and the right-hand wire has slipped 12 mm back into the box, so only the left
wire's tip is inside the connector. An electrician making this splice holds the two wires side by side with their tips
even, so the connector's coil grabs both, and turns the connector clockwise until it is snug and no bare copper shows
below its skirt. A wire that is short is not gripped by the coil and pulls out or arcs; a connector that is only started
is loose and leaves copper exposed.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import twistkit as tk  # noqa: E402  (threaded caps on top of yamkit; nothing on disk changes)
import yamkit as kit  # noqa: E402
import guardkit as gk  # noqa: E402  (the gripper's own shape, for holding and let-go)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Finish this wire splice."

# The junction box (fixed): a 4-11/16-inch-square, 2-1/8-inch-deep galvanized steel box, open face up, centered at
# BOX_C.
# Heights below are above the table top; y offsets are from the box's center line.
BOX_C = (0.30, 0.0)
BOX_W, BOX_H, BOX_T = 0.119, 0.054, 0.0016  # outside width, depth, sheet thickness
# The conductors: 12 AWG solid copper (2.05 mm), THHN insulation 3.3 mm across. Each comes up through a knockout in
# the box's back (5.3 cm along x and 3 cm along y from its center), rises to JOG, runs across to the middle and
# stands up beside the other.
WIRE_R, CU_R = 0.00165, 0.001025
KNOCKOUT_X, KNOCKOUT_Y = 0.053, 0.030  # the right wire's knockout toward the robot (-x), the left's away
JOG = 0.090  # height of each wire's horizontal run
STRIP = 0.120  # the left wire's strip line (insulation ends); its tip is 16 mm above
STRIP_LEN = 0.016  # 5/8 inch, the strip length printed on the connector's carton
SHORT = 0.012  # the right wire starts 12 mm back into the box (its tip 12 mm below the left one's)

# The connector: a red twist-on wire connector for two to four 12 AWG wires, 2.0 cm across the skirt, 2.8 cm long,
# with a square-wire coil inside. Its "thread" is the coil biting on the wires.
NUT_R, NUT_L = 0.010, 0.028
SEAT = STRIP - 0.002  # the skirt's height when snug: it covers the last 2 mm of insulation
THREAD = tk.Thread(pitch=0.004, turns=2.5, start=-915.0, max_tight=60.0)  # see BITE_DEG and the README
NUT = "box/nut"
WIRE_B = "box/wire_b"

# Scoring (angles in degrees back from the seat, i.e. the connector turned that much short of snug)
BITE_DEG = 540.0  # the coil has gripped the wires once the connector is a full turn on (900 is where it first bites)
SNUG_DEG = 45.0  # snug: within 45 degrees of the seat (0.5 mm of travel at 4 mm per turn)
COVER_DEG = 180.0  # bare copper shows below the skirt more than 180 degrees back (2 mm of overlap at 4 mm per turn)
GRIP_SLACK = 90.0  # all but 90 degrees of the connector's turning on must be done with a gripper holding it
EVEN_TOL = 0.003  # tips even within 3 mm (under one 4 mm turn of the coil) when the coil grips the wires
MOVE_TOL = 0.002  # the sim's coil does not hold the wire, so a wire moved more than 2 mm after the bite is counted
# as pulled out of the splice (a real trial checks retention with a tug test instead; see the README)
# A gripper holds the connector or the right wire when both its fingers touch it (gk.gripping: the gripper's own shape,
# palm and fingers at their present opening, within 2 mm, the solver's resting gap plus the fit of its boxes to the
# meshes) with their opening stopped by it: the connector's 2.0 cm or the wire's 3.3 mm, from 15 % of full opening
# narrower (never below half of it) to 3 % wider. An open hand around it or a closed fist resting on it does not hold it.
NUT_GRIP = gk.band(2 * NUT_R)
WIRE_GRIP = gk.band(2 * WIRE_R)
STEEL = kit.Look((0.78, 0.79, 0.80), 0.4, metallic=0.35)
DARK = kit.Look((0.10, 0.10, 0.11), 0.7)
BLACK = kit.Look((0.06, 0.06, 0.07), 0.45)
COPPER = kit.Look((0.85, 0.52, 0.30), 0.3, metallic=0.9)
RED = kit.Look((0.78, 0.10, 0.08), 0.45)
GREEN = kit.Look((0.15, 0.55, 0.20), 0.5)
WHITE = kit.Look((0.93, 0.93, 0.90), 0.6)


def _rod(a, b, r, look):
    """A cylinder from point a to point b, overlapping by r at each end so bends close up."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    d = b - a
    L = float(np.linalg.norm(d))
    return kit.cylinder(tuple((a + b) / 2), r, L + 2 * r, look, axis=tuple(d / L), n=12)


def _wire(side):
    """One conductor's meshes and colliders (side +1: the left wire, -1: the right), drawn with its tip even with the
    left wire's. It comes up through its knockout, runs toward the box's center line along x (the right wire on the
    robot's side), then along y to the middle, and stands up. Colliders cover the insulation above the box rim and
    stop 3 mm under the strip line, clear of the connector."""
    s = side
    xk, yk, yc = s * KNOCKOUT_X, s * KNOCKOUT_Y, s * WIRE_R
    pts = [(xk, yk, -0.010), (xk, yk, JOG), (0, yk, JOG), (0, yc, JOG), (0, yc, STRIP)]
    meshes = [_rod(p, q, WIRE_R, BLACK) for p, q in zip(pts, pts[1:])]
    meshes.append(kit.cylinder((0, yc, STRIP + STRIP_LEN / 2), CU_R, STRIP_LEN, COPPER, n=12))
    d, top = 2 * WIRE_R, STRIP - 0.003
    cols = (kit.Collider((xk, yk, (BOX_H + 0.004 + JOG) / 2), size=(d, d, JOG - BOX_H - 0.004)),
            kit.Collider((xk / 2, yk, JOG), size=(abs(xk) + d, d, d)),
            kit.Collider((0, (yk + yc) / 2, JOG), size=(d, abs(yk - yc) + d, d)),
            kit.Collider((0, yc, (JOG + WIRE_R + top) / 2), size=(d, d, top - JOG - WIRE_R)))
    return meshes, cols


def _box():
    w, h, t = BOX_W, BOX_H, BOX_T
    parts = [((0, 0, t / 2), (w, w, t)),
             ((w / 2 - t / 2, 0, h / 2), (t, w, h)), ((-w / 2 + t / 2, 0, h / 2), (t, w, h)),
             ((0, w / 2 - t / 2, h / 2), (w, t, h)), ((0, -w / 2 + t / 2, h / 2), (w, t, h))]
    meshes = [kit.box(c, s, STEEL) for c, s in parts]
    for sx, sy in ((1, 1), (-1, -1)):  # the two corner screw tabs for a mud ring, with their tapped holes
        c = (sx * (w / 2 - 0.009), sy * (w / 2 - 0.009), h - 0.004)
        meshes.append(kit.box(c, (0.014, 0.014, 0.0016), STEEL))
        meshes.append(kit.cylinder((c[0], c[1], h - 0.0031), 0.0018, 0.0004, DARK, n=12))
    for s in (1, -1):  # the knockouts the wires come through, each with a plastic bushing
        meshes.append(kit.cylinder((s * KNOCKOUT_X, s * KNOCKOUT_Y, t + 0.0002), 0.0045, 0.0004, DARK, n=20))
        meshes.append(kit.lathe([(0.0045, t), (0.0045, t + 0.003), (0.0035, t + 0.003)], WHITE,
                                center=(s * KNOCKOUT_X, s * KNOCKOUT_Y, 0)))
    meshes.append(kit.cylinder((0.030, 0.0, t + 0.002), 0.0030, 0.004, GREEN, n=12))  # the green ground screw
    cols = [kit.Collider(c, size=s) for c, s in parts]
    return meshes, cols


def _nut():
    prof = [(0.0, 0.0), (NUT_R - 0.0004, 0.0), (NUT_R, 0.002), (NUT_R - 0.0002, 0.016), (0.0085, 0.023),
            (0.0060, NUT_L - 0.0005), (0.0, NUT_L)]
    meshes = [kit.lathe(prof, RED, n=32)]
    for k in range(8):  # molded ribs up the side, one of them white so the turning shows
        a = 2 * math.pi * k / 8
        meshes.append(kit.cylinder((NUT_R * math.cos(a), NUT_R * math.sin(a), 0.010), 0.0009, 0.014,
                                   WHITE if k == 0 else RED, n=6))
    return tk.Cap("nut", anchor=(0.0, 0.0, SEAT), mass=0.01, meshes=tuple(meshes),
                  colliders=(kit.Collider((0, 0, NUT_L / 2), radius=NUT_R, height=NUT_L),),
                  grip=(0.0, 0.0, 0.014), width=2 * NUT_R)


def _splice():
    box_meshes, box_cols = _box()
    a_meshes, a_cols = _wire(+1)
    b_meshes, b_cols = _wire(-1)
    # The right wire slides along its own length (straight up and down through its knockout), drawn with its tip even;
    # its joint reads how far it is from even (negative: short). It stays where it is left, like a stiff wire in a box.
    wire_b = kit.Link("wire_b", "prismatic", "Z", (0.0, 0.0, 0.0), lower=-0.016, upper=0.010, start=-SHORT,
                      mass=0.01, meshes=tuple(b_meshes), colliders=b_cols, floating=True, stay=300.0, damping=2.0)
    return tk.Threaded("box", pos=(BOX_C[0], BOX_C[1], TOP), size=(BOX_W, BOX_W, STRIP + STRIP_LEN),
                       meshes=tuple(box_meshes + a_meshes), colliders=tuple(box_cols) + a_cols, cap=_nut(),
                       thread=THREAD, extra=(wire_b,))


OBJECTS = [_splice()]


def _hold_thread_while_settling():
    """Hold the connector on its coil from the very first physics step. The simulator settles the scene for 0.5 s after
    each reset, before the first camera frame, and twistkit sets the coil's drives only from the first control step
    on; until then the connector's slide has no drive and the connector dropped onto the slide's stop, then was lifted
    back to its start height in the first second of the video. This applies the same drives before every settling
    step, so the first frame shows the connector where it starts and nothing moves before a hand touches it."""
    ie = sys.modules.get("yam_arena.inspect_embodiment")
    cls = getattr(ie, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_wire_nut_hold", False):
        return
    base_reset = cls.reset

    def reset(self, *a, **k):
        self._wire_nut_fresh = True  # a new trial: the coil's record starts over at the start angle

        def step(action):
            env = self._env
            if getattr(self, "_wire_nut_fresh", False):
                tk._TRACK.pop(NUT, None)
                self._wire_nut_fresh = False
            spec = kit._MECHS.get(NUT.split("/")[0])
            if spec is not None:
                tk._drive_thread(env.unwrapped, spec)
            return env._wire_nut_step(action)

        orig_ensure = self._ensure_sim

        def ensure(goal):
            orig_ensure(goal)
            env = self._env
            if env is not None and not hasattr(env, "_wire_nut_step"):
                env._wire_nut_step = env.step
                env.step = step

        self._ensure_sim = ensure
        try:
            return base_reset(self, *a, **k)
        finally:
            del self._ensure_sim
            if self._env is not None and hasattr(self._env, "_wire_nut_step"):
                self._env.step = self._env._wire_nut_step  # control steps run as before
                del self._env._wire_nut_step

    cls.reset = reset
    cls._wire_nut_hold = True


_hold_thread_while_settling()


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.bitten = False  # the connector is a full turn or more onto the wires (the coil has gripped them)
        self.b_at_bite = None  # the right wire's offset from even when the coil last gripped
        self.pulled = False  # the right wire moved more than 2 mm while the coil gripped it


_H = _History()


def _offset_b(objects):
    """How far the right wire's tip is from the left one's (m, negative: short)."""
    return kit.joint(objects, WIRE_B).pos


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    b = _offset_b(objects)
    bitten = tk.backed_off_deg(objects, NUT) <= BITE_DEG
    if bitten and not _H.bitten:  # the coil grips the wires now: where the right wire is decides whether it's caught
        _H.b_at_bite, _H.pulled = b, False
    if bitten and abs(b - _H.b_at_bite) > MOVE_TOL:
        _H.pulled = True
    _H.bitten = bitten


def _captured():
    """Both wires in the coil: the right wire's tip within 3 mm of the left one's when the coil gripped them, and not
    moved more than 2 mm since. (The left wire is fixed under the connector's axis, so it is always in.)"""
    return _H.bitten and abs(_H.b_at_bite) <= EVEN_TOL and not _H.pulled


NUT_SHAPE = gk.Ring(0.0, NUT_R, 0.0, NUT_L)  # the connector's collider, in its own frame (origin at the skirt)


def _wire_shapes(objects, sides=(+1, -1)):
    """Each conductor's path as capsules in the box's frame (insulation from its bushing up, then its bare tip): the
    left wire (+1) fixed, the right one (-1) shifted along z by its slide."""
    out = []
    for side in sides:
        dz = 0.0 if side > 0 else _offset_b(objects)
        xk, yk, yc = side * KNOCKOUT_X, side * KNOCKOUT_Y, side * WIRE_R
        pts = [(xk, yk, BOX_T + dz), (xk, yk, JOG + dz), (0, yk, JOG + dz), (0, yc, JOG + dz),
               (0, yc, STRIP + STRIP_LEN + dz)]
        out += [gk.Seg(a, b, WIRE_R) for a, b in zip(pts, pts[1:])]
    return tuple(out)


def _held(objects):
    """A gripper holding the connector or the right wire: both fingers touching it with their opening stopped by it."""
    return bool(gk.gripping(objects, NUT, NUT_SHAPE, NUT_GRIP)
                or gk.gripping(objects, "box", _wire_shapes(objects, (-1,)), WIRE_GRIP))


def _clear(objects):
    """Let go and clear: no gripper touching the connector or either wire (gk.touching: the gripper's own shape, palm and
    fingers at their present opening, within 2 mm of them), so the finished splice is seen to stay together on its own.
    A hand resting on it could still be holding it together or pressing it down; a hand off it, however near, is not."""
    return not (gk.touching(objects, NUT, NUT_SHAPE) or gk.touching(objects, "box", _wire_shapes(objects)))


def _score(objects):
    back = tk.backed_off_deg(objects, NUT)
    # the connector not a full turn on (or never turned): nothing is spliced
    if back > BITE_DEG:
        return 0.0
    # turned partly without a grip on it (knocked or swiped round): not a splice made by hand
    if (-tk.gripped_deg(objects, NUT) < (-THREAD.start - back) - GRIP_SLACK
            or tk.unheld_deg(objects, NUT, tightening=True) > GRIP_SLACK):
        return 0.0
    # the right wire wasn't level with the left when the coil gripped them (or was pulled after): it isn't spliced
    if not _captured():
        return 0.0
    if back > COVER_DEG:
        return 0.25  # both wires in, but the connector is loose and bare copper shows below its skirt
    if back > SNUG_DEG:
        return 0.5  # copper covered, but the connector isn't snug
    if _held(objects) or not _clear(objects):
        return 0.5  # snug, but a gripper still holds it or a hand rests on it: not shown to stay together on its own
    return 1.0


def success(objects):
    """Full score: the connector snug (within 45 degrees of its seat) on both wires, let go (no gripper touching it or
    the wires). Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 unless the connector ends at least a full turn onto the wires (540 degrees or less back from snug; it first
    bites 900 degrees back), with all but 90 degrees of its net turning credited (twistkit: a gripper grips the connector
    and its wrist rolls), and no more than 90 degrees of clockwise turning in all done without such a grip.
    0 if the right wire's tip was more than 3 mm from the left one's when the connector came a full turn on, or the
    wire moved more than 2 mm after that (re-judged if the connector is backed off and put on again). Otherwise 0.25
    if the connector is more than 180 degrees from snug (bare copper below the skirt), 0.5 if 45 to 180 degrees from
    snug (covered but loose), and 1.0 within 45 degrees of snug and let go (no gripper touching the connector or either
    wire; 0.5 if a gripper still holds the connector or the right wire, both fingers touching it with their opening
    stopped by it, or a hand still touches them).
    Where the arms end up does not matter. Possible scores: 0, 0.25, 0.5, 1.
    These angles and the 2 mm limit are the sim's stand-ins for what a real trial observes (copper below the skirt,
    a snug check by hand and a tug test; see the README)."""
    return _score(objects)


# --- Demo ----------------------------------------------------------------------------------------
NUT_XY = BOX_C
B_GRIP = (BOX_C[0] - 0.050, BOX_C[1] - KNOCKOUT_Y)  # near the outer end of the right wire's run along x, 5 cm from
# the connector's axis: the YAM's fingers are 6.8 cm wide, so this keeps them clear of the connector
PULL_EXTRA = 0.001  # how far past even the right arm lifts (it stopped 0.8 mm short in a run)
NUT_GRIP_Z = TOP + SEAT + 0.014  # tool point at the connector's middle when snug
TURN = 960.0  # from 915 degrees back (just off the coil) to 45 degrees past snug (the slip between strokes takes some)
STROKE = 150.0


def demo():
    """Right arm: grip the right wire's horizontal run with the fingers across it, draw the wire 12 mm up out of the
    box until its tip is level with the left one's, let go and go home. Left arm, once the right arm is home: turn the
    connector clockwise (seen from above) with a regrasp loop (grip, roll the wrist 150 degrees, let go, roll back,
    repeat; 960 degrees of roll in all), riding down the wires as it threads on, let go, back off 10 cm above the
    connector, pause there and go home."""
    x, y = B_GRIP
    z0 = TOP + JOG - SHORT
    up = z0 + SHORT + PULL_EXTRA
    yaw = 0.0  # fingers close along y, across the run
    right = [Waypoint(1.5, (x, y, z0 + 0.05), FINGER_OPEN, "over the short wire", yaw),
             Waypoint(1.5, (x, y, z0), FINGER_OPEN, "around the short wire", yaw),
             Waypoint(0.8, (x, y, z0), FINGER_CLOSED, "pinch the wire", yaw),
             Waypoint(1.5, (x, y, up), FINGER_CLOSED, "draw it up level with the other", yaw),
             Waypoint(0.6, (x, y, up), FINGER_OPEN, "let go", yaw),
             Waypoint(1.0, (x, y, up + 0.05), FINGER_OPEN, "lift clear", yaw),
             kit.Home(2.0)]
    t_right = sum(w.duration for w in right)
    park = (0.24, 0.20, TOP + 0.20)
    left = [Waypoint(t_right, park, FINGER_OPEN, "wait for the right arm")]
    z_bite = NUT_GRIP_Z + THREAD.pitch * 915.0 / 360.0
    left += tk.twist_waypoints(NUT_XY, z_bite, -TURN, side="left", stroke=STROKE, pitch=THREAD.pitch,
                               label="twist the connector on")
    last = left[-1]  # opened and lifted 5 cm: back off well clear of the splice and pause there before going home
    left += [Waypoint(1.0, (last.pos[0], last.pos[1], NUT_GRIP_Z + 0.10), FINGER_OPEN, "back off, clear of the splice",
                      last.yaw),
             Waypoint(1.0, (last.pos[0], last.pos[1], NUT_GRIP_Z + 0.10), FINGER_OPEN, "pause clear", last.yaw),
             kit.Home(2.0), kit.Home(2.0, label="hold at home")]
    total = sum(w.duration for w in left)
    right.append(kit.Home(max(total - t_right, 0.5), label="hold at home"))
    return {"left": left, "right": right}
