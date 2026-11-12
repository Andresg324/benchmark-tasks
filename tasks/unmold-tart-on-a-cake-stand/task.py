"""A baked tart has cooled in its loose-bottom fluted tart pan on the counter, and a small cake stand stands behind
it. A tart cannot be turned out upside down like a cake (its crust and topping can break), and lifting the
pan by its rim lifts everything, because the ring's inward bottom lip holds the loose base. The usual way out is to
stand the base on a cake stand narrower than the base and lower the ring away level with both hands, so it drops to
the table around the stand and leaves the tart on its base, on the stand, ready to serve.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import guardkit as gk  # noqa: E402  (per-trial latches, gripper contact)
import yamkit as kit  # noqa: E402  (modeled props, gripper states, tilted demo grips)
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Take the tart out of its pan."

# The pan: a 24 cm (9.5-inch) fluted tart pan with a removable bottom, tinned steel, full size. Ring frame origin at
# the center of the ring's bottom edge.
WALL_R, WALL_T, WALL_H = 0.120, 0.003, 0.026  # fluted wall, mid-radius 12 cm, 2.6 cm tall
FLUTES, FLUTE_D = 30, 0.002  # 30 flutes, 2 mm deep (visual; the collider wall is round)
LEDGE_IN, LEDGE_H = 0.106, 0.004  # the ring's inward bottom lip that the base rests on (1.25 cm wide)
LIP_OUT, LIP_H = 0.1335, 0.004  # the rolled top rim, widened to a 1.2 cm flat lip the grippers can pinch
LIP_Z = WALL_H - LIP_H / 2  # lip center height in the ring frame
SEG = 24  # collider segments around the ring
RING_MASS = 0.15
BASE_R, BASE_T, BASE_MASS = 0.117, 0.003, 0.10  # the loose base: a 23.4 cm steel disk
TART_R, TART_H, TART_MASS = 0.1165, 0.020, 0.45  # the baked tart: crust and filling
# The cake stand: a small plain white pedestal cake stand 7 cm tall with an 11 cm top, narrower than the 23.4 cm base
# and the ring's 21.2 cm bottom opening, so the ring can drop past it; a 5 cm stem on a 9 cm foot.
STAND_R, STAND_H = 0.055, 0.07
PLATE_T, STEM_R, FOOT_R, FOOT_T = 0.008, 0.025, 0.045, 0.008

PAN_XY = (0.225, 0.0)
STAND_XY = (0.415, 0.0)
RING_POS = (*PAN_XY, TOP + 0.0005)
BASE_POS = (*PAN_XY, RING_POS[2] + LEDGE_H + BASE_T / 2 + 0.0003)
TART_POS = (*PAN_XY, BASE_POS[2] + BASE_T / 2 + TART_H / 2 + 0.0003)

TIN = kit.Look((0.74, 0.75, 0.77), 0.35, 0.8)
DARK_TIN = kit.Look((0.40, 0.41, 0.43), 0.45, 0.7)
CRUST = kit.Look((0.80, 0.58, 0.30), 0.8)
CRUST_DARK = kit.Look((0.62, 0.40, 0.18), 0.85)
FILLING = kit.Look((0.95, 0.80, 0.25), 0.5)
STAND = kit.Look((0.93, 0.92, 0.89), 0.4)


def _circle(r_of, z, n=180):
    return [(r_of(t) * math.cos(t), r_of(t) * math.sin(t), z) for t in np.linspace(0, 2 * math.pi, n + 1)]


def _flat_ring(r_in, r_out, z, look, thick=0.001):
    """A flat annulus (a strap along the circle, `thick` tall)."""
    return kit.band(_circle(lambda t: (r_in + r_out) / 2, z), thick, r_out - r_in, look, normal=(0, 0, 1))


def _ring():
    flute = lambda t: WALL_R + FLUTE_D / 2 * math.cos(FLUTES * t)  # noqa: E731
    meshes = [kit.band(_circle(flute, WALL_H / 2), WALL_H, WALL_T, TIN, normal=(0, 0, 1)),  # the fluted wall
              _flat_ring(WALL_R, LIP_OUT, LIP_Z, TIN, LIP_H),  # the rolled rim's flat lip
              _flat_ring(LEDGE_IN, WALL_R, LEDGE_H / 2, DARK_TIN, LEDGE_H)]  # the inward bottom lip
    cols = []
    for k in range(SEG):
        a = 2 * math.pi * k / SEG
        c, s = math.cos(a), math.sin(a)
        deg = math.degrees(a)
        rr = (LEDGE_IN + WALL_R + WALL_T / 2) / 2
        chord = lambda r: 2 * r * math.tan(math.pi / SEG) + 0.002  # noqa: E731  (2 mm overlap between segments)
        cols.append(kit.Collider((rr * c, rr * s, LEDGE_H / 2), size=(WALL_R + WALL_T / 2 - LEDGE_IN, chord(rr),
                                                                     LEDGE_H), yaw=deg))
        cols.append(kit.Collider((WALL_R * c, WALL_R * s, WALL_H / 2), size=(WALL_T, chord(WALL_R), WALL_H), yaw=deg))
        rl = (WALL_R + LIP_OUT) / 2
        cols.append(kit.Collider((rl * c, rl * s, LIP_Z), size=(LIP_OUT - WALL_R, chord(rl), LIP_H), yaw=deg))
    return kit.Prop("ring", pos=RING_POS, size=(2 * LIP_OUT, 2 * LIP_OUT, WALL_H), mass=RING_MASS,
                    meshes=tuple(meshes), colliders=tuple(cols))


def _base():
    return kit.Prop("base", pos=BASE_POS, size=(2 * BASE_R, 2 * BASE_R, BASE_T), mass=BASE_MASS,
                    meshes=(kit.cylinder((0, 0, 0), BASE_R, BASE_T, DARK_TIN, n=64),),
                    colliders=(kit.Collider((0, 0, 0), radius=BASE_R, height=BASE_T),))


def _tart():
    h = TART_H / 2
    flute = lambda t: TART_R - 0.0015 + FLUTE_D / 2 * math.cos(FLUTES * t)  # noqa: E731
    meshes = [kit.lathe([(TART_R - 0.004, -h), (TART_R - 0.004, h - 0.002), (TART_R - 0.012, h - 0.001)], CRUST,
                        n=64),
              kit.band(_circle(flute, 0.0), TART_H, 0.003, CRUST, normal=(0, 0, 1)),  # the fluted crust edge
              kit.band(_circle(lambda t: TART_R - 0.006, h - 0.0005), 0.002, 0.008, CRUST_DARK, normal=(0, 0, 1)),
              kit.cylinder((0, 0, h - 0.0025), TART_R - 0.010, 0.003, FILLING, n=64)]
    return kit.Prop("tart", pos=TART_POS, size=(2 * TART_R, 2 * TART_R, TART_H), mass=TART_MASS,
                    meshes=tuple(meshes), colliders=(kit.Collider((0, 0, 0), radius=TART_R, height=TART_H),))


def _stand():
    parts = [((0, 0, STAND_H - PLATE_T / 2), STAND_R, PLATE_T),  # top
             ((0, 0, STAND_H / 2), STEM_R, STAND_H - 2 * PLATE_T),  # stem
             ((0, 0, FOOT_T / 2), FOOT_R, FOOT_T)]  # foot
    return kit.Prop("stand", pos=(*STAND_XY, TOP), size=(2 * STAND_R, 2 * STAND_R, STAND_H),
                    meshes=tuple(kit.cylinder(c, r, h, STAND, n=64) for c, r, h in parts),
                    colliders=tuple(kit.Collider(c, radius=r, height=h) for c, r, h in parts))


OBJECTS = [_stand(), _ring(), _base(), _tart()]


# --- Scoring
# Thresholds and why:
COCK_DEG = 6.0  # ring tilt while it slides down past the crust: 6 degrees leans the top of the 2.6 cm wall 2.7 mm
#                 inward, deeper than the 2 mm flutes, so the high side's wall can lever into the baked fluted
#                 edge (the tilt is the proxy; a real trial reads it from an inclinometer and photographs the edge)
SHOVE = 0.005  # m: the tart pushed 5 mm across its base has been shoved by a hand or the ring (it sits there by
#                friction and nothing else moves it), which can dent its crust edge (the sim's tart is rigid: the
#                shift is the proxy; a real trial photographs the edge before and after)
TIP_DEG = 20.0  # the tart tilted this far has slid on the base or fallen against the stand's edge, where it can
#                 break (a proxy for a ruined tart; a real trial reads the tilt from the video)
LEVEL_DEG = 5.0  # base (with the tart) and ring "level": 5 degrees drops one edge of the 23.4 cm base 1 cm
CENTER = 0.03  # m: base center within 3 cm of the stand's axis, so the tart's weight stays well inside its 5.5 cm top radius
ON = 0.005  # m: "resting on": the base's bottom within 5 mm of the stand's top; the ring's within 5 mm of the table
# Touching (gk.touching): a gripper's own shape (palm and both fingers, boxes fitted to the YAM's collision meshes, at
# their present opening) within gk.CONTACT (2 mm: the solver's resting gap plus the box fit) of the part's shape. A
# hand hovering over the tart or beside the pan more than 2 mm away is not touching it.
TART = gk.Ring(0.0, TART_R, -TART_H / 2, TART_H / 2)
BASE = gk.Ring(0.0, BASE_R, -BASE_T / 2, BASE_T / 2)
RING = (gk.Ring(LEDGE_IN, WALL_R + WALL_T / 2, 0.0, LEDGE_H),  # the bottom lip
        gk.Ring(WALL_R - WALL_T / 2, WALL_R + WALL_T / 2, 0.0, WALL_H),  # the wall
        gk.Ring(WALL_R, LIP_OUT, LIP_Z - LIP_H / 2, LIP_Z + LIP_H / 2))  # the top lip

COCKED = gk.Latch(steps=1)  # irreversible: the ring cocked while sliding off the crust (the two-step guard
#                                against physics pops is in _Slide: tilted at both ends of a step)
TOUCHED = gk.Latch(steps=2)  # irreversible: a gripper touched the tart
RUINED = gk.Latch(steps=2)  # irreversible: the tart tipped or slid off its base (the proxy for a ruined tart)


def _tilt(objects, name):
    x, y, _, _ = (float(v) for v in objects[name].quat_xyzw)
    return math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y)))))


_SEEN = {}  # this step's contact results: success() and score() both ask, with the same objects


def _contact(objects, name, parts):
    if _SEEN.get("objs") is not objects:
        _SEEN.clear()
        _SEEN["objs"] = objects
    if name not in _SEEN:
        _SEEN[name] = bool(gk.touching(objects, name, parts))
    return _SEEN[name]


def _on_tart(objects):
    """A gripper (palm, finger or pad) touching the tart's top, side or the crust's bottom edge."""
    return _contact(objects, "tart", TART)


def _let_go(objects):
    """No gripper touching the ring, the base or the tart (an open hand resting against the ring still counts as on
    it; one hovering beside it does not)."""
    return not (_contact(objects, "ring", RING) or _contact(objects, "base", BASE) or _on_tart(objects))


def _bottom(objects, name, half):
    return float(objects[name].pos[2]) - half


def _stand_top(objects):
    return float(objects["stand"].pos[2]) + STAND_H


def _off(objects, name):
    """Horizontal distance from the stand's axis (m)."""
    return float(np.hypot(*(np.asarray(objects[name].pos[:2]) - np.asarray(objects["stand"].pos[:2]))))


def _tart_on_base(objects):
    q = gk.local(objects, "base", objects["tart"].pos)
    return math.hypot(q[0], q[1]) < SHOVE and abs(q[2] - (BASE_T + TART_H) / 2) < ON


def _base_on_stand(objects):
    """The base (with the tart on it) resting level and centered on the stand's top."""
    return (_off(objects, "base") <= CENTER and _tilt(objects, "base") <= LEVEL_DEG
            and abs(_bottom(objects, "base", BASE_T / 2) - _stand_top(objects)) <= ON
            and _tart_on_base(objects) and objects["base"].resting(max_speed=0.02, max_spin=1.0))


def _ring_down(objects):
    """The ring flat on the table around the stand (the stand inside its 10.6 cm inner lip)."""
    return (float(objects["ring"].pos[2]) - TOP <= ON and _tilt(objects, "ring") <= LEVEL_DEG
            and _off(objects, "ring") <= LEDGE_IN - STAND_R and objects["ring"].resting(max_speed=0.02, max_spin=1.0))


def _ring_on(objects):
    """The base still in the ring, on its bottom lip."""
    q = gk.local(objects, "ring", objects["base"].pos)
    return math.hypot(q[0], q[1]) < LEDGE_IN and abs(q[2] - (LEDGE_H + BASE_T / 2)) < ON


SLIDE = (LEDGE_H + 0.005, WALL_H)  # the ring's drop below the base while its wall still stands beside the crust


def _drop(objects):
    """How far the ring's bottom is below where it holds the base."""
    return _bottom(objects, "base", BASE_T / 2) - float(objects["ring"].pos[2])


class _Slide(gk._PerStep):
    """Cocked while sliding, judged over each step's motion rather than at single samples: the ring's drop from the
    last step to this one overlaps the 1.7 cm sliding stretch and the ring is tilted past 6 degrees at both ends of
    that step. A ring moving faster than the stretch per step (34 cm/s) is still checked, and a one-step tilt pop
    is not counted (the smaller of the two tilts is used)."""

    def _fresh(self):
        self.prev, self.hit = None, False

    def cocked(self, objects) -> bool:
        if self._tick(objects):
            now = (_drop(objects), _tilt(objects, "ring"))
            prev, self.prev = self.prev or now, now
            lo, hi = min(prev[0], now[0]), max(prev[0], now[0])
            self.hit = lo < SLIDE[1] and hi >= SLIDE[0] and min(prev[1], now[1]) > COCK_DEG
        return self.hit


SLID = _Slide()


def _watch(objects):
    COCKED.feed(objects, SLID.cocked(objects))
    TOUCHED.feed(objects, _on_tart(objects))
    RUINED.feed(objects, _tilt(objects, "tart") > TIP_DEG or not _tart_on_base(objects))


def _job(objects):
    if TOUCHED.on or RUINED.on or not _base_on_stand(objects):
        return 0.0
    if not _let_go(objects) or _ring_on(objects):
        return 0.25
    if not _ring_down(objects):
        return 0.0
    return 0.5 if COCKED.on else 1.0


def success(objects):
    """Unmolded level and let go (where the arms end once clear does not matter)."""
    _watch(objects)
    return _job(objects) == 1.0


def score(objects):
    """Doing nothing scores 0. 0 if a gripper ever touched the tart (its palm or fingers in contact with the tart's
    top, side or crust edge, by the gripper's own shape; hovering does not count) or the tart ever tipped past 20
    degrees or slid off its base (irreversible, 2 steps in a row). 0 unless the base, with the tart on it, ends
    resting on the stand: within 3 cm of its axis, within 5 degrees of level, its bottom within
    5 mm of the stand's top. Then 0.25 if the ring is still on (the base on its bottom lip) or a gripper is still
    touching the ring or the base; 0 if the ring is anywhere but flat on the table around the stand; 0.5 if the ring
    ever tilted past 6 degrees while sliding down past the crust (irreversible); otherwise 1.0. Possible scores: 0,
    0.25, 0.5, 1.0."""
    _watch(objects)
    return _job(objects)


# --- Demo: each arm pinches the pan's lip from its own side (a side grip: approach along y tilted 25 degrees down,
# fingers closing nearly vertically), both lift the pan together off the table, carry it over the stand, and lower it
# level: the base meets the stand and stops while the ring goes on down to the table. Then let go, back out and go home.
TILT = math.radians(25)  # a level side grip's hand body meets the table with the tool point 5.1 cm up; tilted 25
#                          degrees it gets down to 2.4 cm, below the lip of the pan on the table (2.4 cm)
GRIP_Y = LIP_OUT + 0.0025  # tool point 2.5 mm outside the lip, so the pad tips (1.3 cm past it) stop short of the wall
GRIP_DZ = 0.003  # tool point 3 mm above the lip's mid-plane: the tilted pads meet the lip 8 mm further in, where it
#                  lies 3.8 mm above the jaw axis, so this centers the lip between the pads
OUT_Y = GRIP_Y + 0.02
NEAR = -0.012  # fingers 2.4 cm apart around the 4 mm lip (wide open they would reach the table)
LIFT = 0.135  # m: ring bottom while carrying, 6.5 cm above the stand's top


def _w(side, d, pos, finger, label):
    s = 1.0 if side == "left" else -1.0
    x, y, z = pos
    return kit.Tilted(d, (x, s * y, z), finger, label, roll=-s * (math.pi / 2 - TILT))


def arm_plan(side, put=STAND_XY, lower_s=4.0, down_to=0.003):
    """One arm's plan. `put` is where the pan's center goes; `down_to` is the ring bottom's final height."""
    px = PAN_XY[0]
    z0 = RING_POS[2] - TOP + LIP_Z + GRIP_DZ
    zc = LIFT + LIP_Z + GRIP_DZ
    zd = down_to + LIP_Z + GRIP_DZ
    cx, cy = put
    s = 1.0 if side == "left" else -1.0
    w = lambda d, x, y, z, f, label: _w(side, d, (x, y + s * cy, TOP + z), f, label)  # noqa: E731
    return [_w(side, 2.0, (px, OUT_Y, TOP + z0 + 0.05), NEAR, "beside the lip"),
            _w(side, 1.5, (px, OUT_Y, TOP + z0), NEAR, "down"),
            _w(side, 1.5, (px, GRIP_Y, TOP + z0), NEAR, "onto the lip"),
            _w(side, 1.0, (px, GRIP_Y, TOP + z0), FINGER_CLOSED, "pinch"),
            _w(side, 2.0, (px, GRIP_Y, TOP + zc), FINGER_CLOSED, "lift"),
            w(2.5, cx, GRIP_Y, zc, FINGER_CLOSED, "over the stand"),
            w(lower_s, cx, GRIP_Y, zd, FINGER_CLOSED, "lower the ring away"),
            w(1.0, cx, GRIP_Y, zd, NEAR, "let go"),
            w(1.2, cx, OUT_Y, zd, NEAR, "out"),
            w(1.5, cx, OUT_Y, zd + 0.15, NEAR, "up"),
            kit.Home(2.5), kit.Home(1.5, label="hold at home")]


def demo():
    """Both arms pinch the pan's lip at opposite sides, lift it together, center it over the stand and
    lower it level until the ring rests on the table around the stand, then let go and go home."""
    return {"left": arm_plan("left"), "right": arm_plan("right")}
