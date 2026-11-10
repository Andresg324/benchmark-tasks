"""A small woodshop's dust collection, as a 1:2.5 bench model. One dust collector draws through a trunk duct that runs
across the bench; three branch ducts lead from it to a table saw, a planer and a disc sander, and each branch has a
sliding blast gate. A rough board lies on the planer's infeed table: the planer is the machine about to run.

Before switching on a machine, a woodworker slides that machine's gate fully open and the other two fully shut. A
small collector only has the airflow for one 4-inch branch at a time: an idle gate left open, or part open, splits the
suction, and the running machine's chips and fine dust go into the air instead of the bag. The shop was left with the
saw's gate open from the last cut and the sander's half open, and the planer's shut.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props and the gate slides; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Get the dust collection ready to plane this board."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# --- Scene, in world coordinates (m): x forward from the arm bases, y to the robot's left, z up from the table top.
X_G = 0.310  # the three blast gates stand in one row across the bench at this x
TRUNK_X = 0.220  # the trunk duct runs along y here, to the collector on the right
MACHINE_X = 0.470  # the machines' front faces, where the branch ducts end
R_IN, R_OUT = 0.020, 0.022  # branch duct bore and outside radius (a 4-inch duct at 1:2.5 is 4 cm)
R_TRUNK = 0.028  # trunk outside radius (a 6-inch main at 1:2.5)
DUCT_Z = 0.030  # duct centerline height
BRANCHES = {"saw": 0.150, "planer": 0.0, "sander": -0.150}  # each branch's centerline y
TARGET = "planer"  # the machine about to run: the board lies on its infeed table
COLLECTOR = (0.300, -0.430)  # the collector's drum, on the bench to the right

# --- The blast gates. Each blade slides along y in its housing; 0 = shut, TRAVEL = pulled out to its stop.
TRAVEL = 0.046  # m: the stop; the blade's leading edge then sits 4 mm clear of the bore
EDGE = -0.022  # the blade's leading edge (link frame) when shut: 2 mm past the bore's far side, in the seat groove
BLADE_T = 0.003  # blade thickness (x)
BLADE_Y = (EDGE, 0.050)  # the blade's extent along y (link frame); its tail sticks out of the housing's open side
BLADE_Z = (0.008, 0.052)
HOUSING_Y, HOUSING_Z = 0.030, (0.004, 0.058)  # housing half width (y) and height span
KNOB = (0.030, 0.025, 0.028)  # the pull handle on a stem above the blade's tail: x, y, z (grip across its 2.5 cm)
KNOB_Y, KNOB_Z = 0.045, 0.074  # its center, link frame (6 to 8.8 cm above the table, clear over the housings)
BLADE_MASS = 0.045  # kg
GATE_START = {"saw": TRAVEL, "planer": 0.0, "sander": 0.022}  # left from the last job: saw open, sander half open
STAY_K, DRAG = 60.0, 2.0  # N/m, N*s/m: the blade's friction in its housing (it stays where it is left)

# --- Scoring thresholds (reasons in score() and the README).
OPEN_MIN = 0.040  # m: open = leading edge at most 2 mm into the 40 mm bore, which blocks 1.9% of its area
SHUT_MAX = 0.004  # m: shut = withdrawn at most 4 mm, which uncovers a 2 mm sliver, 1.9% of the bore's area
GATE_STILL = 0.005  # m/s: a gate counts only at rest
HANDS_OFF = 0.040  # m: a tool point within 4 cm of a gate's handle center is still on it (the handle is 2.5 to 3 cm)
DT = 0.05  # s per control step (20 Hz)
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving
GRIPPERS = ("left_gripper", "right_gripper")

GALV = kit.Look((0.74, 0.76, 0.78), 0.35, metallic=0.7)
DUCT = kit.Look((0.80, 0.82, 0.84), 0.3, metallic=0.6)
BLADE = kit.Look((0.66, 0.68, 0.70), 0.3, metallic=0.8)
KNOBL = kit.Look((0.08, 0.08, 0.09), 0.6)
SADDLE = kit.Look((0.30, 0.30, 0.31), 0.7)
SAW = kit.Look((0.20, 0.32, 0.55), 0.5)
PLANER = kit.Look((0.80, 0.55, 0.12), 0.5)
SANDER = kit.Look((0.25, 0.45, 0.30), 0.5)
IRON = kit.Look((0.62, 0.63, 0.64), 0.35, metallic=0.6)
DARK = kit.Look((0.14, 0.14, 0.15), 0.5)
STEEL = kit.Look((0.82, 0.82, 0.84), 0.25, metallic=0.9)
RED = kit.Look((0.80, 0.10, 0.08), 0.4)
PAPER = kit.Look((0.62, 0.48, 0.30), 0.9)
OAK = kit.Look((0.66, 0.50, 0.32), 0.85)
DRUM = kit.Look((0.18, 0.30, 0.52), 0.5)
BAG = kit.Look((0.93, 0.93, 0.90), 0.9)

SHOP = (0.400, 0.0, TOP)  # the shop prop's origin
GATES = (X_G, 0.0, TOP)  # the gate row's origin


def _rel(o, c):
    return (c[0] - o[0], c[1] - o[1], c[2])


def _xduct(o, x0, x1, y, meshes, colliders, r=R_OUT):
    """A branch duct section along x from x0 to x1 at y, with its collider."""
    c = _rel(o, ((x0 + x1) / 2, y, DUCT_Z))
    meshes.append(kit.cylinder(c, r, x1 - x0, DUCT, axis="x"))
    colliders.append(kit.Collider(c, size=(x1 - x0, 2 * r, 2 * r)))


def _box(o, center, size, look, meshes, colliders=None):
    c = _rel(o, center)
    meshes.append(kit.box(c, size, look))
    if colliders is not None:
        colliders.append(kit.Collider(c, size=size))


def _shop():
    """Everything fixed except the gate blades: ducts, machines, the board and the collector."""
    o, m, c = SHOP, [], []
    # trunk along y, from past the saw branch to the collector, with an end cap; branches join it
    y0, y1 = BRANCHES["saw"] + 0.040, COLLECTOR[1] - R_TRUNK
    tc = _rel(o, (TRUNK_X, (y0 + y1) / 2, DUCT_Z + 0.002))
    m.append(kit.cylinder(tc, R_TRUNK, y0 - y1, DUCT, axis="y"))
    c.append(kit.Collider(tc, size=(2 * R_TRUNK, y0 - y1, 2 * R_TRUNK)))
    m.append(kit.cylinder(_rel(o, (TRUNK_X, y0 + 0.002, DUCT_Z + 0.002)), R_TRUNK + 0.002, 0.006, GALV, axis="y"))
    for y in np.arange(y1 + 0.03, y0, 0.11):  # saddles under the trunk
        _box(o, (TRUNK_X, y, 0.003), (0.05, 0.018, 0.006), SADDLE, m)
    for name, y in BRANCHES.items():
        _xduct(o, TRUNK_X, X_G - 0.016, y, m, c)
        _xduct(o, X_G + 0.016, MACHINE_X + 0.01, y, m, c)
        for x in (X_G - 0.016, X_G + 0.016):  # the gate's collars, clamped onto the duct
            m.append(kit.cylinder(_rel(o, (x + (0.004 if x < X_G else -0.004), y, DUCT_Z)), R_OUT + 0.002, 0.012,
                                  GALV, axis="x"))
        for x in (0.265, 0.410):  # saddles
            _box(o, (x, y, 0.004), (0.016, 0.040, 0.008), SADDLE, m)
    # the collector: a drum on the bench with a filter bag on top, its inlet facing the trunk
    m.append(kit.cylinder(_rel(o, (*COLLECTOR, 0.055)), 0.065, 0.11, DRUM))
    c.append(kit.Collider(_rel(o, (*COLLECTOR, 0.055)), radius=0.065, height=0.11))
    m.append(kit.cylinder(_rel(o, (*COLLECTOR, 0.113)), 0.067, 0.006, DARK))
    m.append(kit.lathe([(0.055, 0.0), (0.060, 0.05), (0.055, 0.10), (0.03, 0.115), (0.0, 0.118)], BAG,
                       _rel(o, (*COLLECTOR, 0.116))))
    stub = _rel(o, ((TRUNK_X + COLLECTOR[0] - 0.06) / 2, COLLECTOR[1], DUCT_Z + 0.002))  # the trunk's elbow in
    m.append(kit.cylinder(stub, R_TRUNK, COLLECTOR[0] - 0.06 - TRUNK_X + R_TRUNK, DUCT, axis="x"))
    # table saw: blue cabinet, cast-iron top, blade, rip fence, red switch
    ys = BRANCHES["saw"]
    _box(o, (0.530, ys, 0.042), (0.12, 0.11, 0.084), SAW, m, c)
    _box(o, (0.530, ys, 0.087), (0.13, 0.13, 0.006), IRON, m, c)
    m.append(kit.cylinder(_rel(o, (0.535, ys, 0.078)), 0.030, 0.0025, STEEL, axis="y"))
    _box(o, (0.530, ys + 0.045, 0.098), (0.13, 0.010, 0.016), DARK, m)
    _box(o, (0.468, ys - 0.035, 0.060), (0.006, 0.020, 0.012), RED, m)
    # planer: orange stand, cast-iron beds, the cutter head on its columns, the board on the infeed bed
    yp = BRANCHES["planer"]
    _box(o, (0.535, yp, 0.025), (0.13, 0.10, 0.050), PLANER, m, c)
    _box(o, (0.535, yp, 0.053), (0.15, 0.08, 0.006), IRON, m, c)  # beds, infeed toward the robot
    for sy in (-1, 1):
        _box(o, (0.540, yp + sy * 0.045, 0.085), (0.040, 0.012, 0.058), PLANER, m, c)  # columns
    _box(o, (0.540, yp, 0.103), (0.050, 0.102, 0.024), PLANER, m, c)  # the head, over the bed
    _box(o, (0.540, yp, 0.118), (0.035, 0.050, 0.008), DARK, m)  # its dust hood
    _box(o, (0.468, yp - 0.040, 0.040), (0.006, 0.020, 0.012), RED, m)
    _box(o, (0.500, yp, 0.0625), (0.090, 0.055, 0.013), OAK, m, c)  # the rough board, its end at the cutter head
    for k in range(4):  # saw marks across the rough board
        _box(o, (0.468 + 0.018 * k, yp, 0.0692), (0.003, 0.055, 0.0005), kit.Look((0.55, 0.40, 0.25), 0.9), m)
    # disc sander: green base, a vertical sanding disc facing the robot, motor behind, small work table in front
    yd = BRANCHES["sander"]
    _box(o, (0.530, yd, 0.022), (0.12, 0.10, 0.044), SANDER, m, c)
    m.append(kit.cylinder(_rel(o, (0.560, yd, 0.085)), 0.030, 0.050, SANDER, axis="x"))
    c.append(kit.Collider(_rel(o, (0.560, yd, 0.085)), size=(0.05, 0.06, 0.06)))
    m.append(kit.cylinder(_rel(o, (0.528, yd, 0.085)), 0.042, 0.010, DARK, axis="x"))
    m.append(kit.cylinder(_rel(o, (0.522, yd, 0.085)), 0.040, 0.002, PAPER, axis="x"))
    _box(o, (0.500, yd, 0.050), (0.035, 0.080, 0.005), IRON, m, c)
    _box(o, (0.468, yd - 0.035, 0.030), (0.006, 0.020, 0.012), RED, m)
    return kit.Prop("shop", pos=SHOP, size=(0.40, 0.70, 0.24), meshes=tuple(m), colliders=tuple(c))


class FrictionSlide:
    """A blast gate's blade in its housing: no spring and no detent, only the friction of the plate in its slot, so it
    stays wherever it is left. Modeled as a soft hold at the blade's current position (STAY_K) plus a drag (DRAG):
    about 0.1 N at the demo's 3 cm/s, so it moves under any push and stops when the push stops."""

    def drive(self, key, q, qd, J, mem):
        return q, STAY_K, 0.0, DRAG


def _gates():
    o, m, c = GATES, [], []
    links = []
    for name, y in BRANCHES.items():
        for sx in (-1, 1):  # the housing: two galvanized plates either side of the blade's slot, closed on the -y side
            cx = X_G + sx * (BLADE_T / 2 + 0.0003 + 0.004)
            size = (0.008, 2 * HOUSING_Y, HOUSING_Z[1] - HOUSING_Z[0])
            _box(o, (cx, y, sum(HOUSING_Z) / 2), size, GALV, m, c)
        _box(o, (X_G, y - HOUSING_Y + 0.002, sum(HOUSING_Z) / 2), (0.0036, 0.004, HOUSING_Z[1] - HOUSING_Z[0]),
             GALV, m)
        blade_c = (0.0, sum(BLADE_Y) / 2, sum(BLADE_Z) / 2)
        blade_s = (BLADE_T, BLADE_Y[1] - BLADE_Y[0], BLADE_Z[1] - BLADE_Z[0])
        knob_c = (0.0, KNOB_Y, KNOB_Z)
        meshes = (kit.box(blade_c, blade_s, BLADE), kit.box(knob_c, KNOB, KNOBL),
                  kit.box((0.0, KNOB_Y, (BLADE_Z[1] + KNOB_Z) / 2), (0.006, 0.010, KNOB_Z - BLADE_Z[1]), KNOBL),
                  kit.box((0.0, 0.034, BLADE_Z[1] - 0.004), (BLADE_T + 0.001, 0.006, 0.004), BLADE))
        links.append(kit.Link(name, "prismatic", "Y", (0.0, y, 0.0), lower=0.0, upper=TRAVEL, start=GATE_START[name],
                              mass=BLADE_MASS, meshes=meshes,
                              colliders=(kit.Collider(blade_c, size=blade_s), kit.Collider(knob_c, size=KNOB)),
                              damping=0.5, floating=True, law=FrictionSlide()))
    return kit.Mechanism("gates", pos=GATES, size=(0.03, 0.50, 0.08), meshes=tuple(m), colliders=tuple(c),
                         links=tuple(links))


OBJECTS = [_shop(), _gates()]


def knob(objects, name):
    """The world position of gate `name`'s pull handle."""
    q = kit.joint(objects, f"gates/{name}").pos
    return np.array([X_G, BRANCHES[name] + q + KNOB_Y, TOP + KNOB_Z])


# --- What happened during the run. success() and score() are called after every control step; the history (the arms'
# start, for the home check) is kept here and started afresh on every scene reset (kit.episode()).
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.home = None  # each gripper's tool point at the trial's first scored step
        self.last = None
        self.still = False


_H = _History()


def _update(objects):
    global _H
    if _H.episode != kit.episode():  # the simulator reset the scene: a new trial, a clean history
        _H = _History()
    now = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    if _H.home is None and len(now) == 2:
        _H.home = now
    _H.still = _H.last is not None and all(g in _H.last and np.linalg.norm(now[g] - _H.last[g]) < HOME_STILL
                                           for g in now)
    _H.last = now


def _arms_home(objects):
    """Both grippers' tool points back within 3 cm of where they were at the trial's first step, and still."""
    if _H.home is None or not _H.still:
        return False
    return all(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - _H.home[g]) < HOME_TOL for g in GRIPPERS)


def _held(objects):
    """Some tool point within 4 cm of some gate's handle: a hand still on a gate."""
    return any(g in objects and np.linalg.norm(np.asarray(objects[g].pos) - knob(objects, n)) < HANDS_OFF
               for g in GRIPPERS for n in BRANCHES)


def gate_state(objects, name):
    """'open', 'shut' or 'part' for gate `name`."""
    q = kit.joint(objects, f"gates/{name}").pos
    return "open" if q >= OPEN_MIN else "shut" if q <= SHUT_MAX else "part"


def _job(objects):
    if _held(objects):
        return 0.0  # nothing counts while a hand is still on a gate
    if any(abs(kit.joint(objects, f"gates/{n}").vel) > GATE_STILL for n in BRANCHES):
        return 0.0  # a gate still sliding
    if gate_state(objects, TARGET) != "open":
        return 0.0  # the planer's gate not fully open: the planer would run starved of suction
    shut = sum(gate_state(objects, n) == "shut" for n in BRANCHES if n != TARGET)
    return {2: 1.0, 1: 0.25, 0: 0.0}[shut]


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """The planer's gate open, the saw's and sander's shut, no hand on a gate, both arms home and still. Called first
    each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """Scored on the final state. A gate is open when its blade is pulled out at least 40 mm of its 46 mm travel
    (leading edge at most 2 mm into the 40 mm bore, blocking 1.9% of its area), shut when withdrawn at most 4 mm
    (a 2 mm sliver of bore uncovered past the seat, 1.9% of its area; as an estimate, a sliver that size passes about
    2% of the collector's flow, within what a small collector can spare), and part open otherwise. 0 if any tool
    point is within 4 cm of a gate's handle (still held) or a gate is still sliding faster than 5 mm/s. 0 if the
    planer's gate is not open. With it open: 1.0 with both the saw's and the sander's gates shut; 0.25 with one of
    them shut (the suction still splits two ways); 0 with neither shut (opening all three, or only the planer's,
    leaves the start's three-way split). With REQUIRE_ARMS_HOME, a finished job with the arms not back home and still
    scores 0.9. Possible scores: 0, 0.25, 0.9, 1.0."""
    return _score(objects)


def demo():
    """Right arm: grip the sander gate's handle and push the half-open blade shut, let go, go home. Left arm: grip the
    saw gate's handle and push its blade shut, let go; once the right arm is home, grip the planer gate's handle and
    pull its blade open to the stop, let go, go home. Grips close across the handle (along y), straight down."""
    gz = TOP + KNOB_Z + KNOB[2] / 2 - 0.016 + 0.013  # TCP height with the finger pads 1.6 cm down the handle
    up = gz + 0.07
    half = FINGER_OPEN * 0.7

    def at(name, q):
        return (X_G, BRANCHES[name] + q + KNOB_Y)

    def slide(name, q0, q1, label, lag=0.004):
        """Over the handle, down around it, grip, slide it from q0 to q1 (aiming `lag` past q1), let go, lift."""
        s = math.copysign(lag, q1 - q0)
        out = [Waypoint(1.2, (*at(name, q0), up), half, f"over the {name} gate"),
               Waypoint(0.8, (*at(name, q0), gz), half, "around its handle"),
               Waypoint(0.6, (*at(name, q0), gz), FINGER_CLOSED, "grip")]
        for k in range(1, 7):
            out.append(Waypoint(0.3, (*at(name, q0 + (q1 + s - q0) * k / 6), gz), FINGER_CLOSED, label))
        out += [Waypoint(0.5, (*at(name, q1 + s), gz), FINGER_CLOSED, "hold at the stop"),
                Waypoint(0.6, (*at(name, q1 + s), gz), half, "let go"),
                Waypoint(0.8, (*at(name, q1 + s), up), half, "lift off")]
        return out

    right = slide("sander", GATE_START["sander"], 0.0, "push the sander gate shut")
    right += [kit.Home(1.8), kit.Home(0.5, label="stay home")]
    left = slide("saw", GATE_START["saw"], 0.0, "push the saw gate shut")
    wait = max(sum(w.duration for w in right) - sum(w.duration for w in left), 0.0) + 0.5
    left += [Waypoint(wait, (*at("saw", 0.0), up + 0.03), half, "wait for the right arm")]
    left += slide("planer", 0.0, TRAVEL, "pull the planer gate open")
    left += [kit.Home(1.8), kit.Home(2.0, label="hold at home")]
    pad = sum(w.duration for w in left) - sum(w.duration for w in right)
    right += [kit.Home(max(pad, 0.5), label="stay home")]
    return {"left": left, "right": right}
