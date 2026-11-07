"""A hospital blood bank. A blue transport cooler from the blood supplier stands open in front of the robot with
three units lying in it: red blood cells (a dark red bag), apheresis platelets (an opaque straw-colored bag) and frozen
plasma (a flat pale yellow bag, white with frost). Each bag has a white label, two ports at one end and a hanging tab
at the other. Three storage units stand around it. On the left is a chest-type blood bank refrigerator with its lid
open, two red cell units already inside and a green temperature display. On the right is a chest freezer with its lid
open, frosted inside, two frozen plasma units already inside and a blue display. Straight ahead is a benchtop platelet
agitator: a grey base with an amber display and a flat platform with low rails, one platelet bag already on it.
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
INSTRUCTION = "Put away the blood products from the delivery."

BAG = (0.10, 0.045, 0.025)  # every unit's collider: 10 cm long (x), 4.5 cm wide (y), 2.5 cm thick
PLASMA = (0.10, 0.045, 0.022)  # frozen plasma is flatter

# Transport cooler (open top), centered in front of the robot.
COOL_X, COOL_Y, COOL_IX, COOL_IY, COOL_H, COOL_W = 0.235, 0.0, 0.13, 0.25, 0.04, 0.008
UNIT_Y = {"red_cells": 0.085, "plasma": 0.0, "platelets": -0.085}
# Chest refrigerator (left) and chest freezer (right): open tops with the lids standing open behind.
CHEST_X, CHEST_IX, CHEST_IY, CHEST_H, CHEST_W = 0.30, 0.17, 0.13, 0.07, 0.008
FRIDGE_Y, FREEZER_Y = 0.215, -0.215
# Platelet agitator, straight ahead.
AG_X, AG_Y, AG_BASE = 0.375, 0.0, (0.10, 0.17, 0.035)
AG_PLAT = (0.12, 0.17, 0.008)  # the rocking platform on the base
AG_TOP = TOP + AG_BASE[2] + AG_PLAT[2]  # the platform's surface
STORE_STEPS = 20  # 1 s at 20 Hz resting in a storage unit counts as stored there

# -- looks ---------------------------------------------------------------------------------------

FILM = kit.Look((0.92, 0.94, 0.96), 0.15, opacity=0.12)
RBC = kit.Look((0.42, 0.03, 0.05), 0.35)
PLT = kit.Look((0.93, 0.78, 0.48), 0.45)
FFP = kit.Look((0.97, 0.92, 0.66), 0.55)
FROST = kit.Look((0.95, 0.97, 1.0), 0.95, opacity=0.3)
LABEL = kit.Look((0.98, 0.98, 0.97), 0.8)
BARS = kit.Look((0.10, 0.10, 0.10), 0.8)
PORT = kit.Look((0.85, 0.87, 0.90), 0.3)
COOLER_BLUE = kit.Look((0.12, 0.35, 0.72), 0.5)
COOLER_IN = kit.Look((0.88, 0.90, 0.92), 0.6)
WHITE = kit.Look((0.94, 0.95, 0.95), 0.35)
FRIDGE_IN = kit.Look((0.80, 0.86, 0.90), 0.5)
FREEZER_IN = kit.Look((0.90, 0.95, 1.0), 0.95)
GASKET = kit.Look((0.25, 0.27, 0.30), 0.7)
GREY = kit.Look((0.55, 0.57, 0.60), 0.45)
DARK = kit.Look((0.08, 0.09, 0.10), 0.4)
GREEN_LED = kit.Look((0.20, 0.95, 0.35), 0.3)
BLUE_LED = kit.Look((0.30, 0.65, 1.0), 0.3)
AMBER_LED = kit.Look((1.0, 0.65, 0.15), 0.3)
RED_TAG = kit.Look((0.80, 0.10, 0.10), 0.5)


def _bag_meshes(size, fill, frosted=False):
    """A blood bag lying flat along x: a rounded pouch of product inside clear film, a white label with a barcode on
    top, two ports at the +x end and a hanging tab at the -x end."""
    lx, ly, lz = size
    r = lz / 2 - 0.001
    m = [kit.box((0, 0, 0), (lx - 0.012, ly - 2 * r, lz - 0.002), fill),
         kit.cylinder((0, ly / 2 - r, 0), r, lx - 0.012, fill, axis="x"),
         kit.cylinder((0, -ly / 2 + r, 0), r, lx - 0.012, fill, axis="x"),
         kit.box((0, 0, 0), (lx - 0.004, ly, lz), FILM),
         kit.box((-0.012, 0, lz / 2 + 0.0004), (0.032, 0.022, 0.001), LABEL)]
    for k in range(6):
        m.append(kit.box((-0.022 + 0.0035 * k, 0.003, lz / 2 + 0.001), (0.0015, 0.010, 0.0006), BARS))
    m += [kit.cylinder((lx / 2 + 0.002, 0.010, 0), 0.0035, 0.010, PORT, axis="x"),
          kit.cylinder((lx / 2 + 0.002, -0.010, 0), 0.0035, 0.010, PORT, axis="x"),
          kit.box((-lx / 2 - 0.002, 0, 0), (0.008, 0.030, 0.002), FILM)]
    if frosted:
        m.append(kit.box((0, 0, 0), (lx - 0.002, ly + 0.001, lz + 0.001), FROST))
    return tuple(m)


def _unit(name, x, y, size, fill, frosted=False, mass=0.06):
    z = TOP + size[2] / 2
    return kit.Prop(name, pos=(x, y, z), size=size, mass=mass, meshes=_bag_meshes(size, fill, frosted),
                    colliders=(kit.Collider((0, 0, 0), size=size),))


def _rim(ix, iy, h, w, look):
    """A rim strip 4 mm tall along the top of each wall."""
    return [kit.box((ix / 2 + w / 2, 0, h + 0.002), (w + 0.002, iy + 2 * w + 0.002, 0.004), look),
            kit.box((-ix / 2 - w / 2, 0, h + 0.002), (w + 0.002, iy + 2 * w + 0.002, 0.004), look),
            kit.box((0, iy / 2 + w / 2, h + 0.002), (ix, w + 0.002, 0.004), look),
            kit.box((0, -iy / 2 - w / 2, h + 0.002), (ix, w + 0.002, 0.004), look)]


def _open_box(ix, iy, h, w, outer, inner):
    """Four walls (colliders and meshes) of an open box around an ix x iy floor, plus its floor."""
    walls = [((ix / 2 + w / 2, 0, h / 2), (w, iy + 2 * w, h)), ((-ix / 2 - w / 2, 0, h / 2), (w, iy + 2 * w, h)),
             ((0, iy / 2 + w / 2, h / 2), (ix, w, h)), ((0, -iy / 2 - w / 2, h / 2), (ix, w, h))]
    meshes = [kit.box(c, s, outer) for c, s in walls] + [kit.box((0, 0, 0.001), (ix, iy, 0.002), inner)]
    return walls, meshes


def _cooler():
    walls, meshes = _open_box(COOL_IX, COOL_IY, COOL_H, COOL_W, COOLER_BLUE, COOLER_IN)
    meshes += _rim(COOL_IX, COOL_IY, COOL_H, COOL_W, WHITE) + [
               kit.box((-COOL_IX / 2 - COOL_W - 0.0006, 0.05, COOL_H / 2), (0.001, 0.05, 0.02), WHITE)]
    return kit.Prop("cooler", pos=(COOL_X, COOL_Y, TOP), size=(COOL_IX + 2 * COOL_W, COOL_IY + 2 * COOL_W, COOL_H),
                    meshes=tuple(meshes), colliders=tuple(kit.Collider(c, size=s) for c, s in walls))


def _chest(name, y, inner, led, stock, frosted):
    """A chest unit: white cabinet walls, dark gasket rim, the lid standing open behind it, a display on the front,
    and two units of stock lying at the back of the well."""
    ix, iy, h, w = CHEST_IX, CHEST_IY, CHEST_H, CHEST_W
    walls, meshes = _open_box(ix, iy, h, w, WHITE, inner)
    meshes += _rim(ix, iy, h, w, GASKET) + [
               kit.box((ix / 2 + w + 0.006, 0, h + 0.06), (0.012, iy + 2 * w, 0.12), WHITE),  # the open lid
               kit.box((ix / 2 + w - 0.0005, 0, h + 0.06), (0.002, iy - 0.01, 0.11), inner),
               kit.box((-ix / 2 - w - 0.0008, 0, h * 0.55), (0.0015, 0.05, 0.018), DARK),
               kit.box((-ix / 2 - w - 0.0018, 0, h * 0.55), (0.0008, 0.03, 0.009), led)]
    if frosted:  # frost on the inside of the walls
        meshes += [kit.box((ix / 2 - 0.002, 0, h / 2), (0.004, iy, h), FREEZER_IN),
                   kit.box((-ix / 2 + 0.002, 0, h / 2), (0.004, iy, h), FREEZER_IN),
                   kit.box((0, iy / 2 - 0.002, h / 2), (ix - 0.008, 0.004, h), FREEZER_IN),
                   kit.box((0, -iy / 2 + 0.002, h / 2), (ix - 0.008, 0.004, h), FREEZER_IN)]
    for k, dy in enumerate((0.03, -0.03)):  # stock already inside: lying across the back of the well
        c = np.array((ix / 2 - 0.028, dy, 0.012))
        for mesh in _bag_meshes((0.045, 0.055, 0.02), stock, frosted):
            pts = tuple(tuple(np.array(p) + c) for p in mesh.points)
            meshes.append(kit.Mesh(pts, mesh.counts, mesh.indices, mesh.look))
    cols = [kit.Collider(c, size=s) for c, s in walls]
    cols.append(kit.Collider((ix / 2 - 0.028, 0, 0.013), size=(0.05, iy, 0.02)))  # the stock, as one block
    return kit.Prop(name, pos=(CHEST_X, y, TOP), size=(ix + 2 * w + 0.03, iy + 2 * w, h + 0.12),
                    meshes=tuple(meshes), colliders=tuple(cols))


def _agitator():
    bx, by, bz = AG_BASE
    px, py, pz = AG_PLAT
    rail = 0.008
    meshes = [kit.box((0, 0, bz / 2), AG_BASE, GREY),
              kit.box((-bx / 2 - 0.0008, 0.05, bz / 2), (0.0015, 0.04, 0.016), DARK),
              kit.box((-bx / 2 - 0.0018, 0.05, bz / 2), (0.0008, 0.026, 0.008), AMBER_LED),
              kit.box((-bx / 2 - 0.0012, -0.04, bz / 2), (0.0015, 0.012, 0.012), GREEN_LED),
              kit.box((0, 0, bz + pz / 2), AG_PLAT, WHITE)]
    rails = [((px / 2 - 0.002, 0, bz + pz + rail / 2), (0.004, py, rail)),
             ((-px / 2 + 0.002, 0, bz + pz + rail / 2), (0.004, py, rail))]
    meshes += [kit.box(c, s, GREY) for c, s in rails]
    c = np.array((0.025, 0.0, bz + pz + 0.011))  # one platelet unit already on the platform, across its far half
    for mesh in _bag_meshes((0.045, 0.10, 0.02), PLT):
        meshes.append(kit.Mesh(tuple(tuple(np.array(p) + c) for p in mesh.points), mesh.counts, mesh.indices,
                               mesh.look))
    cols = [kit.Collider((0, 0, bz / 2), size=AG_BASE), kit.Collider((0, 0, bz + pz / 2), size=AG_PLAT),
            kit.Collider((0.025, 0, bz + pz + 0.01), size=(0.045, 0.10, 0.02))]
    cols += [kit.Collider(c, size=s) for c, s in rails]
    return kit.Prop("agitator", pos=(AG_X, AG_Y, TOP), size=(px, py, bz + pz + rail), meshes=tuple(meshes),
                    colliders=tuple(cols))


OBJECTS = [
    _cooler(),
    _chest("fridge", FRIDGE_Y, FRIDGE_IN, GREEN_LED, RBC, False),
    _chest("freezer", FREEZER_Y, FREEZER_IN, BLUE_LED, FFP, True),
    _agitator(),
    _unit("red_cells", COOL_X, UNIT_Y["red_cells"], BAG, RBC, mass=0.06),
    _unit("plasma", COOL_X, UNIT_Y["plasma"], PLASMA, FFP, frosted=True, mass=0.05),
    _unit("platelets", COOL_X, UNIT_Y["platelets"], BAG, PLT, mass=0.06),
]
UNITS = ("red_cells", "platelets", "plasma")
_START = {o.name: np.array(o.pos) for o in OBJECTS if o.mass is not None}
CORRECT = {"red_cells": "fridge", "platelets": "agitator", "plasma": "freezer"}


def _where(s):
    """Which storage unit a bag is in, at rest or not: 'fridge', 'freezer', 'agitator' or None. In a chest: its
    center inside the well and its top below the rim. On the agitator: its center over the platform and its bottom no
    more than 2.5 cm above the platform's surface, so it may lie on the platform or on the platelet bag already
    there."""
    x, y, _ = s.pos
    for name, cy in (("fridge", FRIDGE_Y), ("freezer", FREEZER_Y)):
        if abs(x - CHEST_X) < CHEST_IX / 2 and abs(y - cy) < CHEST_IY / 2 and s.high[2] < TOP + CHEST_H:
            return name
    if (abs(x - AG_X) < AG_PLAT[0] / 2 and abs(y - AG_Y) < AG_PLAT[1] / 2
            and AG_TOP - 0.01 < s.low[2] < AG_TOP + 0.025):
        return "agitator"
    return None


def _stored(s):
    """Where a bag is stored: _where() for a bag at rest, None for one still moving."""
    return _where(s) if s.resting() else None


def _quat_angle(q, ref):
    """Angle in degrees between two (x, y, z, w) quaternions."""
    d = abs(float(np.dot(np.asarray(q, float), np.asarray(ref, float))))
    return float(np.degrees(2 * np.arccos(min(1.0, d))))


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()  # which trial this history belongs to
        self.runs = {u: (None, 0) for u in UNITS}  # (storage, consecutive steps resting there)
        self.ruined = set()  # units that were stored at the wrong temperature: never forgiven


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    for u in UNITS:
        where = _stored(objects[u])
        prev, n = _H.runs[u]
        n = n + 1 if where == prev else 1
        _H.runs[u] = (where, n)
        if where is not None and where != CORRECT[u] and n >= STORE_STEPS:
            _H.ruined.add(u)


def _score(objects):
    if _H.ruined:
        return 0.0
    if any(_where(objects[u]) not in (None, CORRECT[u]) for u in UNITS):  # in the wrong unit at the end, even moving
        return 0.0
    return sum(_stored(objects[u]) == CORRECT[u] for u in UNITS) / 3


def success(objects):
    """1.0: red cells in the refrigerator, platelets on the agitator, plasma in the freezer. Called first each step,
    so it also records the step's history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if any unit was ever stored at the wrong temperature (resting 1 s in the wrong storage unit), even if it was
    moved on afterwards, since that unit must be discarded. 0 also if any unit is in the wrong storage unit at the end,
    at rest or not. Otherwise one third for each unit resting in its own storage unit at the end.
    Possible scores: 0, 1/3, 2/3, 1."""
    return _score(objects)


def _carry(pick, place, travel, yaw=0.0, place_yaw=None, opening=0.62):
    """Top-down pick and place with the fingers opened only part way (`opening` of full), so they fit between the
    units in the cooler."""
    (px, py, pz), (qx, qy, qz) = pick, place
    end = yaw if place_yaw is None else place_yaw
    part = FINGER_OPEN * opening
    return [Waypoint(1.5, (px, py, travel), part, "to the unit", yaw),
            Waypoint(1.5, (px, py, pz), part, "descend", yaw),
            Waypoint(0.8, (px, py, pz), FINGER_CLOSED, "close", yaw),
            Waypoint(1.2, (px, py, travel), FINGER_CLOSED, "lift", yaw),
            Waypoint(1.8, (qx, qy, travel), FINGER_CLOSED, "carry", end),
            Waypoint(1.2, (qx, qy, qz), FINGER_CLOSED, "lower", end),
            Waypoint(0.6, (qx, qy, qz), part, "release", end),
            Waypoint(1.0, (qx, qy, travel), part, "retreat", end)]


def demo():
    """Left arm: the red cells into the refrigerator. Then the right arm: the plasma into the freezer and the
    platelets onto the agitator's platform, turned across it. One arm at a time, since the units lie close together."""

    def grip(top, below=0.012):  # TCP height that puts the finger pads `below` an object's top
        return top - below + 0.013

    travel = TOP + 0.20
    lag = 0.012
    rc = _carry((COOL_X, UNIT_Y["red_cells"] - lag, grip(TOP + BAG[2])),
                (CHEST_X - 0.012, FRIDGE_Y, grip(TOP + BAG[2] + 0.004) + 0.02), travel)
    park_l = (0.14, 0.27, TOP + 0.22)
    left = rc + [Waypoint(1.2, park_l, FINGER_OPEN, "park"), Waypoint(16.0, park_l, FINGER_OPEN, "wait")]
    ffp = _carry((COOL_X, UNIT_Y["plasma"] + lag, grip(TOP + PLASMA[2], 0.011)),
                 (CHEST_X - 0.012, FREEZER_Y, grip(TOP + PLASMA[2] + 0.004) + 0.02), travel)
    plt = _carry((COOL_X, UNIT_Y["platelets"] + lag, grip(TOP + BAG[2])),
                 (AG_X - 0.03, AG_Y + lag, grip(AG_TOP + BAG[2]) + 0.012), travel, place_yaw=np.pi / 2)
    park_r = (0.14, -0.27, TOP + 0.22)
    right = [Waypoint(10.5, park_r, FINGER_OPEN, "wait")] + ffp + plt + [Waypoint(1.2, park_r, FINGER_OPEN, "park"),
                                                                       Waypoint(2.0, park_r, FINGER_OPEN, "hold")]
    return {"left": left, "right": right}
