"""A concrete footing with three rebar stubs sticking straight up out of it, at one-third scale. The footing is a grey
block in front of the arms; the three ribbed steel bars stand in a row across it. On each side of the footing, three
orange rebar caps stand on the ground in a row: two kinds, mixed. The square caps, with a flat 4 cm plate on top of
a sleeve, are impalement-rated caps (a steel plate is molded into the plastic). The round mushroom caps, a
2.6 cm dome on a short stem, only cover the bar's cut end.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props below; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Make the rebar safe before the crew walks through."

# Footing and bars (one-third scale: a #6 bar, 1.9 cm, is 0.64 cm here).
FOOT_X, FOOT_LX, FOOT_LY, FOOT_H = 0.31, 0.12, 0.34, 0.04
BAR_R, BAR_UP = 0.0032, 0.07  # bar radius; how far each bar sticks up out of the footing
FOOT_Y = -0.02
BARS = {"bar_right": -0.13, "bar_middle": -0.02, "bar_left": 0.09}  # name: y of the bar, all at x = FOOT_X
BAR_TOP = TOP + FOOT_H + BAR_UP

# Impalement cap: square plate on a sleeve. Origin at the middle of its height.
PLATE, PLATE_H, SLEEVE_OUT, SLEEVE_IN, SLEEVE_H = 0.040, 0.016, 0.020, 0.012, 0.032
CAP_H = PLATE_H + SLEEVE_H
# Mushroom cap: dome on a stem. Origin at the middle of its height.
DOME_R, DOME_H, STEM_OUT, STEM_IN, STEM_H = 0.013, 0.011, 0.012, 0.008, 0.016
MUSH_H = DOME_H + STEM_H

SQUARE_CAPS = {"cap_a": (0.28, 0.24), "cap_b": (0.22, -0.26), "cap_c": (0.36, -0.26)}
MUSHROOM_CAPS = {"mushroom_a": (0.21, 0.24), "mushroom_b": (0.35, 0.24), "mushroom_c": (0.29, -0.26)}

SEAT_XY, SEAT_Z, MAX_TILT = 0.008, 0.008, math.radians(20)

# -- looks and models -------------------------------------------------------------------------

CONCRETE = kit.Look((0.63, 0.62, 0.59), 0.95)
CONCRETE_EDGE = kit.Look((0.55, 0.54, 0.51), 0.95)
REBAR = kit.Look((0.42, 0.29, 0.21), 0.75, metallic=0.4)
SAFETY_ORANGE = kit.Look((0.98, 0.42, 0.05), 0.55)
ORANGE_DARK = kit.Look((0.85, 0.33, 0.03), 0.6)
STEEL = kit.Look((0.70, 0.71, 0.74), 0.35, metallic=0.9)


def _ribbed_bar(y):
    """A deformed bar: alternating ribs every 6 mm, from inside the footing to its cut end."""
    z0, z1 = FOOT_H / 2 - 0.01, FOOT_H / 2 + BAR_UP
    prof, z, k = [(0.0, z0)], z0, 0
    while z < z1 - 0.003:
        prof.append((BAR_R if k % 2 == 0 else BAR_R + 0.0005, z))
        z += 0.0025
        k += 1
    prof += [(BAR_R, z1 - 0.001), (BAR_R - 0.0008, z1), (0.0, z1)]
    return kit.lathe(prof, REBAR, center=(0, y, 0), n=20)


def _footing():
    meshes = [kit.box((0, 0, 0), (FOOT_LX, FOOT_LY, FOOT_H), CONCRETE),
              kit.box((0, 0, FOOT_H / 2 - 0.001), (FOOT_LX - 0.004, FOOT_LY - 0.004, 0.0025), CONCRETE_EDGE)]
    colliders = [kit.Collider((0, 0, 0), size=(FOOT_LX, FOOT_LY, FOOT_H))]
    for y in BARS.values():
        meshes.append(_ribbed_bar(y - FOOT_Y))
        colliders.append(kit.Collider((0, y - FOOT_Y, FOOT_H / 2 + BAR_UP / 2), radius=BAR_R, height=BAR_UP))
    return kit.Prop("footing", pos=(FOOT_X, FOOT_Y, TOP + FOOT_H / 2), size=(FOOT_LX, FOOT_LY, FOOT_H + 2 * BAR_UP),
                    meshes=tuple(meshes), colliders=tuple(colliders))


def _ring(z, h, outer, inner):
    """Four box walls around a square bore: the sleeve's colliders."""
    w = (outer - inner) / 2
    return [kit.Collider(((outer - w) / 2, 0, z), size=(w, outer, h)),
            kit.Collider((-(outer - w) / 2, 0, z), size=(w, outer, h)),
            kit.Collider((0, (outer - w) / 2, z), size=(inner, w, h)),
            kit.Collider((0, -(outer - w) / 2, z), size=(inner, w, h))]


def _square_cap(name, xy):
    zs = -CAP_H / 2 + SLEEVE_H  # the plate's underside, top of the sleeve
    zp = zs + PLATE_H / 2
    sleeve = kit.lathe([(SLEEVE_IN / 2, zs), (SLEEVE_IN / 2, -CAP_H / 2), (SLEEVE_OUT / 2, -CAP_H / 2),
                        (SLEEVE_OUT / 2, zs)], ORANGE_DARK, n=28)
    meshes = (kit.box((0, 0, zp - 0.002), (PLATE, PLATE, PLATE_H - 0.004), SAFETY_ORANGE),
              kit.box((0, 0, zs + PLATE_H - 0.002), (PLATE - 0.006, PLATE - 0.006, 0.004), SAFETY_ORANGE),
              kit.box((0.0, 0, zs + PLATE_H - 0.0005), (0.009, PLATE - 0.009, 0.0012), ORANGE_DARK),
              kit.box((0, 0.0, zs + PLATE_H - 0.0005), (PLATE - 0.009, 0.009, 0.0012), ORANGE_DARK),
              kit.box((0, -PLATE / 2 - 0.0004, zp), (0.022, 0.001, 0.005), STEEL),  # the plate's edge shows
              sleeve)
    colliders = (kit.Collider((0, 0, zp), size=(PLATE, PLATE, PLATE_H)),
                 *_ring(-CAP_H / 2 + SLEEVE_H / 2, SLEEVE_H, SLEEVE_OUT, SLEEVE_IN))
    return kit.Prop(name, pos=(xy[0], xy[1], TOP + CAP_H / 2), size=(PLATE, PLATE, CAP_H), mass=0.04,
                    meshes=meshes, colliders=colliders)


def _mushroom_cap(name, xy):
    zs = -MUSH_H / 2 + STEM_H  # the dome's underside
    dome = kit.lathe([(STEM_OUT / 2, zs), (DOME_R, zs), (DOME_R, zs + 0.004), (DOME_R - 0.002, zs + 0.0072),
                      (DOME_R - 0.006, zs + 0.0100), (0.0, zs + DOME_H)], SAFETY_ORANGE, n=36)
    stem = kit.lathe([(STEM_IN / 2, zs), (STEM_IN / 2, -MUSH_H / 2), (STEM_OUT / 2, -MUSH_H / 2),
                      (STEM_OUT / 2, zs)], ORANGE_DARK, n=24)
    colliders = (kit.Collider((0, 0, zs + DOME_H / 2), radius=DOME_R, height=DOME_H),
                 *_ring(-MUSH_H / 2 + STEM_H / 2, STEM_H, STEM_OUT, STEM_IN))
    return kit.Prop(name, pos=(xy[0], xy[1], TOP + MUSH_H / 2), size=(2 * DOME_R, 2 * DOME_R, MUSH_H), mass=0.02,
                    meshes=(dome, stem), colliders=colliders)


OBJECTS = [_footing()]
OBJECTS += [_square_cap(n, xy) for n, xy in SQUARE_CAPS.items()]
OBJECTS += [_mushroom_cap(n, xy) for n, xy in MUSHROOM_CAPS.items()]


def _upright(s):
    qx, qy = s.quat_xyzw[0], s.quat_xyzw[1]
    return 1 - 2 * (qx * qx + qy * qy) > math.cos(MAX_TILT)


def _seated(s, bar_y):
    """A square cap seated on a bar: its sleeve centered on the bar within 0.8 cm, the plate's underside resting on
    the bar's cut end (height within 0.8 cm of that), upright within 20 degrees, and still."""
    seat_z = BAR_TOP + CAP_H / 2 - SLEEVE_H
    return bool(np.hypot(s.pos[0] - FOOT_X, s.pos[1] - bar_y) < SEAT_XY and abs(s.pos[2] - seat_z) < SEAT_Z
                and _upright(s) and s.resting())


def _covered(objects):
    """The bars with an impalement-rated (square) cap seated on them."""
    return [b for b, y in BARS.items() if any(_seated(objects[c], y) for c in SQUARE_CAPS)]


def success(objects):
    """All three bars carry a seated impalement-rated cap."""
    return len(_covered(objects)) == 3


def score(objects):
    """One third for each bar with a square, impalement-rated cap seated on it. A mushroom cap on a bar counts for
    nothing: it does not stop impalement. Possible scores: 0, 0.33, 0.67, 1."""
    return len(_covered(objects)) / 3


def demo():
    """Left arm: the square cap on its side onto the left bar. Right arm: its two square caps onto the right bar, then
    the middle bar (after the left arm has parked). Each cap is held by its plate and lowered over the bar's end."""

    def grip(top, below=0.013):  # TCP height that puts the finger pads `below` an object's top
        return top - below + 0.013

    travel = TOP + 0.22
    cap_top = TOP + CAP_H
    seat_top = BAR_TOP - SLEEVE_H + CAP_H  # the plate's top once seated
    drop = 0.006  # let go this far above the seat
    lx, ly = LAG_LEFT
    rx, ry = LAG_RIGHT

    def put(cap, bar, lag):
        """Pick the cap by its plate, carry it high, stop with the sleeve 1 cm above the bar's end until the arm has
        settled, then slide it down over the bar. A held cap rides a measured offset from the tool point, and the arm
        settles a little short of its target; HELD_OFF corrects for both."""
        cx, cy = SQUARE_CAPS[cap]
        pick = pick_place_waypoints((cx, cy, grip(cap_top)), (cx, cy, grip(cap_top)), travel_z=travel, speed=0.8)[:4]
        x, y = FOOT_X + lag[0], BARS[bar] + HELD_OFF[bar] + lag[1]
        above, seat = grip(seat_top) + 0.03, grip(seat_top) + drop
        return pick + [Waypoint(2.2, (x, y, travel), FINGER_CLOSED, "carry"),
                       Waypoint(1.5, (x, y, above), FINGER_CLOSED, "over the bar"),
                       Waypoint(2.0, (x, y, above), FINGER_CLOSED, "settle"),
                       Waypoint(1.5, (x, y, seat), FINGER_CLOSED, "slide on"),
                       Waypoint(0.6, (x, y, seat), FINGER_OPEN, "let go"),
                       Waypoint(1.0, (x, y, travel), FINGER_OPEN, "up")]

    left = put("cap_a", "bar_left", (lx, ly))
    left_park = (0.18, 0.26, TOP + 0.22)
    left += [Waypoint(1.0, left_park, FINGER_OPEN, "park"), Waypoint(20.0, left_park, FINGER_OPEN, "hold")]
    right = put("cap_b", "bar_right", (rx, ry)) + put("cap_c", "bar_middle", (rx - 0.0025, ry))
    right_park = (0.18, -0.26, TOP + 0.22)
    right += [Waypoint(1.0, right_park, FINGER_OPEN, "park"), Waypoint(2.0, right_park, FINGER_OPEN, "hold")]
    return {"left": left, "right": right}


# Where the arms settle short of a target near the footing (measured in the demo runs), added to the targets.
LAG_LEFT = (0.0, 0.0)
LAG_RIGHT = (0.0, 0.0)
HELD_OFF = {"bar_left": 0.0185, "bar_right": 0.0185, "bar_middle": 0.0164}  # measured
