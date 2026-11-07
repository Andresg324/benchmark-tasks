"""Packing for a flight. Two hard-shell suitcases lie open on the bed-side table, each with its lid folded flat behind
it. On the robot's left is a small silver carry-on. On the right is a large navy suitcase with an airline checked-bag
tag (a white strip with a green band and a barcode) looped through its side handle. Between them lie four things to
pack: a dark grey power bank and a spare camera battery in a clear plastic case (nearer the big suitcase), and a
rolled light-blue shirt and a black toiletry bag (nearer the carry-on).
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props below; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Pack the bags for the flight."

# Suitcase bases: (center x, center y, inside x, inside y, wall height). Walls 8 mm thick.
CARRY = (0.28, 0.195, 0.13, 0.14, 0.045)
CHECKED = (0.29, -0.175, 0.16, 0.19, 0.05)
WALL = 0.008

BANK = (0.095, 0.045, 0.025)
BATT = (0.045, 0.040, 0.025)
SHIRT = (0.10, 0.044, 0.044)
KIT = (0.08, 0.045, 0.045)
START = {"power_bank": (0.245, -0.035), "battery": (0.35, -0.035), "shirt": (0.24, 0.065), "toiletries": (0.355, 0.065)}
LITHIUM = ("power_bank", "battery")
OTHER = ("shirt", "toiletries")

# -- looks ---------------------------------------------------------------------------------------

NAVY = kit.Look((0.10, 0.16, 0.32), 0.35)
NAVY_RIB = kit.Look((0.07, 0.11, 0.24), 0.35)
SILVER = kit.Look((0.78, 0.79, 0.81), 0.25, metallic=0.6)
SILVER_RIB = kit.Look((0.66, 0.67, 0.70), 0.25, metallic=0.6)
LINING = kit.Look((0.30, 0.31, 0.34), 0.9)
STRAP = kit.Look((0.15, 0.15, 0.16), 0.8)
TAG = kit.Look((0.98, 0.98, 0.96), 0.7)
TAG_BAND = kit.Look((0.10, 0.55, 0.30), 0.6)
BLACK = kit.Look((0.06, 0.06, 0.07), 0.6)
BANK_GREY = kit.Look((0.20, 0.21, 0.23), 0.45)
LED = kit.Look((0.25, 0.55, 1.0), 0.3)
PORT = kit.Look((0.70, 0.72, 0.75), 0.3, metallic=0.8)
CASE = kit.Look((0.90, 0.93, 0.96), 0.15, opacity=0.4)
CELL = kit.Look((0.12, 0.12, 0.13), 0.4)
GOLD = kit.Look((0.85, 0.68, 0.25), 0.3, metallic=1.0)
CELL_LABEL = kit.Look((0.85, 0.25, 0.15), 0.5)
SHIRT_BLUE = kit.Look((0.62, 0.75, 0.90), 0.95)
SHIRT_EDGE = kit.Look((0.52, 0.65, 0.82), 0.95)
ZIP = kit.Look((0.80, 0.65, 0.30), 0.4, metallic=0.7)


def _suitcase(name, spec, shell, rib, tagged):
    """An open hard-shell suitcase: the base shell (walls and lining) with two packing straps, and the lid shell
    lying open flat behind it. The big one carries an airline checked-bag tag through the handle on its near side."""
    cx, cy, ix, iy, h = spec
    w = WALL
    walls = [((ix / 2 + w / 2, 0, h / 2), (w, iy + 2 * w, h)), ((-ix / 2 - w / 2, 0, h / 2), (w, iy + 2 * w, h)),
             ((0, iy / 2 + w / 2, h / 2), (ix, w, h)), ((0, -iy / 2 - w / 2, h / 2), (ix, w, h))]
    m = [kit.box(c, s, shell) for c, s in walls] + [kit.box((0, 0, 0.002), (ix, iy, 0.004), LINING)]
    for k in (-1, 1):  # ribs moulded into the walls
        m.append(kit.box((-ix / 2 - w - 0.0006, k * iy / 4, h / 2), (0.001, 0.008, h * 0.8), rib))
    m += [kit.band([(-ix / 2, k * iy / 3, 0.004), (-ix / 2 + 0.01, k * iy / 3, 0.010)], 0.012, 0.002, STRAP,
                   normal=(0, 1, 0)) for k in (-1, 1)]
    lx = ix + 2 * w  # the lid: same footprint, lying open just beyond the hinge
    lid_c = (ix / 2 + w + 0.004 + lx / 2, 0, 0.0)
    lh = 0.025
    lid = [((lid_c[0] + lx / 2 - w / 2, 0, lh / 2), (w, iy + 2 * w, lh)),
           ((lid_c[0] - lx / 2 + w / 2, 0, lh / 2), (w, iy + 2 * w, lh)),
           ((lid_c[0], iy / 2 + w / 2, lh / 2), (lx - 2 * w, w, lh)),
           ((lid_c[0], -iy / 2 - w / 2, lh / 2), (lx - 2 * w, w, lh))]
    m += [kit.box(c, s, shell) for c, s in lid] + [kit.box((lid_c[0], 0, 0.002), (lx - 2 * w, iy, 0.004), LINING)]
    m.append(kit.box((lid_c[0], 0, 0.006), (lx - 2 * w - 0.02, iy - 0.02, 0.003), STRAP))  # the lid's mesh divider
    # the side handle on the near wall
    hx = -ix / 2 - w - 0.006
    m.append(kit.band([(hx + 0.006, -0.03, h * 0.6), (hx - 0.002, -0.022, h * 0.6), (hx - 0.004, 0, h * 0.6),
                       (hx - 0.002, 0.022, h * 0.6), (hx + 0.006, 0.03, h * 0.6)], 0.012, 0.004, BLACK,
                      normal=(0, 0, 1)))
    if tagged:  # airline checked-bag tag: a long strip looped through the handle, hanging down the wall
        tx = hx - 0.006
        m += [kit.box((tx, 0.0, h * 0.40), (0.0015, 0.036, h * 0.75), TAG),
              kit.box((tx - 0.0008, 0.0, h * 0.62), (0.0008, 0.036, 0.009), TAG_BAND)]
        for k in range(11):
            m.append(kit.box((tx - 0.0008, -0.014 + 0.0028 * k, h * 0.33), (0.0008, 0.0013, 0.014), BLACK))
        # the tag's stub, folded over the rim onto the lining
        m += [kit.box((-ix / 2 + 0.012, 0.0, 0.0045), (0.024, 0.036, 0.001), TAG),
              kit.box((-ix / 2 + 0.018, 0.0, 0.0052), (0.008, 0.036, 0.0006), TAG_BAND)]
    cols = [kit.Collider(c, size=s) for c, s in walls + lid]
    return kit.Prop(name, pos=(cx, cy, TOP), size=(2 * lx + 0.004, iy + 2 * w, h), meshes=tuple(m),
                    colliders=tuple(cols))


def _power_bank():
    lx, ly, lz = BANK
    r = lz / 2
    m = [kit.box((0, 0, 0), (lx, ly - 2 * r, lz), BANK_GREY),
         kit.cylinder((0, ly / 2 - r, 0), r, lx, BANK_GREY, axis="x"),
         kit.cylinder((0, -ly / 2 + r, 0), r, lx, BANK_GREY, axis="x")]
    m += [kit.cylinder((-lx / 2 + 0.012 + 0.006 * k, -0.012, lz / 2), 0.0012, 0.001, LED) for k in range(4)]
    m += [kit.box((-lx / 2 - 0.0005, k, 0), (0.002, 0.009, 0.0035), PORT) for k in (-0.01, 0.01)]
    return kit.Prop("power_bank", pos=(*START["power_bank"], TOP + lz / 2), size=BANK, mass=0.06, meshes=tuple(m),
                    colliders=(kit.Collider((0, 0, 0), size=BANK),))


def _battery():
    """A spare camera battery in a clear protective case."""
    lx, ly, lz = BATT
    m = [kit.box((0, 0, -0.001), (0.038, 0.030, 0.016), CELL),
         kit.box((0.0, 0.0, 0.0072), (0.022, 0.024, 0.0006), CELL_LABEL),
         kit.box((0.0195, -0.008, -0.001), (0.0006, 0.004, 0.006), GOLD),
         kit.box((0.0195, 0.0, -0.001), (0.0006, 0.004, 0.006), GOLD),
         kit.box((0.0195, 0.008, -0.001), (0.0006, 0.004, 0.006), GOLD),
         kit.box((0, 0, 0), BATT, CASE),
         kit.box((0, 0, lz / 2 - 0.001), (lx + 0.002, ly + 0.002, 0.004), CASE)]
    return kit.Prop("battery", pos=(*START["battery"], TOP + lz / 2), size=BATT, mass=0.04, meshes=tuple(m),
                    colliders=(kit.Collider((0, 0, 0), size=BATT),))


def _shirt():
    """A rolled shirt lying along x; collides as a box of the roll's size so it can't roll away."""
    lx, ly, lz = SHIRT
    r = ly / 2
    m = [kit.cylinder((0, 0, 0), r, lx, SHIRT_BLUE, axis="x", n=40)]
    spiral = [(-lx / 2 + 0.0008, (r - 0.004 * t) * np.cos(6 * t), (r - 0.004 * t) * np.sin(6 * t))
              for t in np.linspace(0, 2.6, 30)]
    m.append(kit.band(spiral, 0.0015, 0.0015, SHIRT_EDGE, normal=(1, 0, 0)))
    m.append(kit.box((0.01, 0, r - 0.0005), (0.06, 0.012, 0.002), SHIRT_EDGE))  # the folded hem along the top
    return kit.Prop("shirt", pos=(*START["shirt"], TOP + lz / 2), size=SHIRT, mass=0.05, meshes=tuple(m),
                    colliders=(kit.Collider((0, 0, 0), size=SHIRT),))


def _toiletries():
    lx, ly, lz = KIT
    r = 0.01
    m = [kit.box((0, 0, -0.002), (lx, ly, lz - 0.004), BLACK),
         kit.box((0, 0, 0), (lx - 2 * r, ly - 2 * r, lz), BLACK),
         kit.cylinder((0, ly / 2 - r, lz / 2 - r), r, lx, BLACK, axis="x"),
         kit.cylinder((0, -ly / 2 + r, lz / 2 - r), r, lx, BLACK, axis="x"),
         kit.box((0, 0, lz / 2 + 0.0005), (lx - 0.01, 0.003, 0.001), ZIP),
         kit.box((lx / 2 - 0.012, 0.004, lz / 2 + 0.001), (0.004, 0.006, 0.002), ZIP),
         kit.band([(-lx / 2, -0.008, 0.008), (-lx / 2 - 0.008, 0, 0.010), (-lx / 2, 0.008, 0.008)], 0.006, 0.002,
                  STRAP, normal=(0, 0, 1))]
    return kit.Prop("toiletries", pos=(*START["toiletries"], TOP + lz / 2), size=KIT, mass=0.05, meshes=tuple(m),
                    colliders=(kit.Collider((0, 0, 0), size=KIT),))


OBJECTS = [
    _suitcase("carry_on", CARRY, SILVER, SILVER_RIB, False),
    _suitcase("checked_bag", CHECKED, NAVY, NAVY_RIB, True),
    _power_bank(),
    _battery(),
    _shirt(),
    _toiletries(),
]
_START = {o.name: np.array(o.pos) for o in OBJECTS if o.mass is not None}


def _inside(s, spec):
    """In a suitcase, at rest or not: its center over the inside of the base and nothing above the rim (+5 mm)."""
    cx, cy, ix, iy, h = spec
    return bool(abs(s.pos[0] - cx) < ix / 2 and abs(s.pos[1] - cy) < iy / 2 and s.high[2] < TOP + h + 0.005)


def _in(s, spec):
    """Packed in a suitcase: inside it and at rest, so the lid would close."""
    return bool(s.resting() and _inside(s, spec))


# Packing is reversible until the bag is checked in, so only the end state counts; nothing needs remembering
# between steps.
def _score(objects):
    if any(_inside(objects[n], CHECKED) for n in LITHIUM):  # at rest or not
        return 0.0
    carried = sum(_in(objects[n], CARRY) for n in LITHIUM)
    if carried == 2:
        return 0.5 + 0.25 * sum(_in(objects[n], CARRY) or _in(objects[n], CHECKED) for n in OTHER)
    return 0.25 if carried == 1 else 0.0


def success(objects):
    """1.0: the power bank and the spare battery in the carry-on, the shirt and the toiletry bag packed in either
    suitcase."""
    return _score(objects) == 1.0


def score(objects):
    """0 if the power bank or the spare battery is in the checked suitcase at the end, at rest or not. Otherwise 0.5 with both in the
    carry-on, plus 0.25 for each of the shirt and the toiletry bag packed in either suitcase; 0.25 with only one of
    them in the carry-on (the other left out); 0 with neither. Possible scores: 0, 0.25, 0.5, 0.75, 1."""
    return _score(objects)


def _carry(pick, place, travel, yaw=0.0, place_yaw=None, opening=0.62):
    """Top-down pick and place with the fingers opened only part way (`opening` of full), so they fit beside the
    suitcase walls."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    part = FINGER_OPEN * opening
    return [Waypoint(1.5, (px, py, travel), part, "to the item", yaw),
            Waypoint(1.5, (px, py, pz), part, "descend", yaw),
            Waypoint(0.8, (px, py, pz), FINGER_CLOSED, "close", yaw),
            Waypoint(1.2, (px, py, travel), FINGER_CLOSED, "lift", yaw),
            Waypoint(1.8, (qx, qy, travel), FINGER_CLOSED, "carry", end),
            Waypoint(1.2, (qx, qy, qz), FINGER_CLOSED, "lower", end),
            Waypoint(0.6, (qx, qy, qz), part, "release", end),
            Waypoint(1.0, (qx, qy, travel), part, "retreat", end)]


def demo():
    """Left arm: the power bank and the battery into the carry-on. Then the right arm: the shirt and the toiletry bag
    into the big suitcase. One arm at a time, since the items lie close together."""

    def grip(top, below=0.012):  # TCP height that puts the finger pads `below` an object's top
        return top - below + 0.013

    travel = TOP + 0.20
    lag = 0.012
    cx, cy, _, _, ch = CARRY
    left = _carry((*START["power_bank"], grip(TOP + BANK[2])), (cx - 0.005, cy - 0.03, grip(TOP + BANK[2]) + 0.012),
                  travel)
    left += _carry((*START["battery"], grip(TOP + BATT[2])), (cx + 0.005, cy + 0.035, grip(TOP + BATT[2]) + 0.012),
                   travel)
    park_l = (0.14, 0.27, TOP + 0.22)
    left += [Waypoint(1.2, park_l, FINGER_OPEN, "park"), Waypoint(22.0, park_l, FINGER_OPEN, "wait")]
    kx, ky, _, _, kh = CHECKED
    right = [Waypoint(20.5, (0.14, -0.27, TOP + 0.22), FINGER_OPEN, "wait")]
    right += _carry((START["shirt"][0], START["shirt"][1] + lag, grip(TOP + SHIRT[2], 0.016)),
                    (kx - 0.01, ky + 0.045, grip(TOP + SHIRT[2], 0.016) + 0.012), travel)
    right += _carry((START["toiletries"][0], START["toiletries"][1] + lag, grip(TOP + KIT[2], 0.016)),
                    (kx - 0.01, ky - 0.04, grip(TOP + KIT[2], 0.016) + 0.012), travel)
    park_r = (0.14, -0.27, TOP + 0.22)
    right += [Waypoint(1.2, park_r, FINGER_OPEN, "park"), Waypoint(2.0, park_r, FINGER_OPEN, "hold")]
    return {"left": left, "right": right}
