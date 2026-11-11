"""A 1:2 model of a bathroom sink's trap arm being roughed in: a 1.25 inch PVC drain pipe from the sink's trap to the
vertical stack, carried on three adjustable pipe stands. Each stand is a base pad on the floor and a top-adjust leveling
screw (a real 1/2-13 thread, hex head on top, turned from above) with a steel bracket threaded on it that holds a
V-saddle under the pipe; turning a head raises or lowers its bracket. A torpedo level lies on the pipe.

A horizontal drain has to fall toward the stack inside a window, and the plumbing code sets both ends of it. Too
little fall (or none, or back-fall) and the line holds standing water and solids that build into a clog: the
International Plumbing Code (Table 704.1) requires at least 1/4 inch per foot (20.8 mm per m) for pipe 2.5 inches and
smaller. Too much, and on a trap arm the outlet end drops below the trap's weir, the arm runs full and siphons the trap
dry, letting sewer gas into the room: the IPC limits the total fall of a fixture drain from slope to one pipe diameter
(1.25 in over this arm's 1.0 m from trap weir to stack, 31.75 mm per m). The pipe also has to bear on every stand: a
stand left low is no support, and the pipe sags into a belly there. The arm was set up dead level (the torpedo level's
bubble centered, which looks right) with the middle stand half a turn (1 mm) low.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled props; nothing on disk changes)
import twistkit as tk  # noqa: E402,F401  (the stands' leveling screws are twistkit threads)
import levelkit as lv  # noqa: E402  (import last: its per-step model runs after yamkit's and twistkit's)
from yam_arena.constants import FINGER_OPEN, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.task_file import TABLE_TOP, Cylinder  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Adjust the pipe stands so the sink drain is ready to connect."
REQUIRE_ARMS_HOME = True  # full score also needs both arms back where they started, still (see _arms_home)

# --- The drain, in its own frame (m): origin on the table under the pipe's axis at y = 0, x away from the robot, y to
# the robot's left. The body's bottom plane (z = 0) is the underside of the stand brackets, which ride the screws.
PIPE = "drain"
PIPE_POS = (0.300, 0.0, TOP)
PIPE_R = 0.0105  # 1.25 inch PVC (1.66 in = 42.2 mm OD) at 1:2: 2.1 cm across
PIPE_Y = (-0.215, 0.215)  # 43 cm long: 1.9 cm short of the trap's side (sink end, +y), 1.3 cm short of the stack's (-y)
BRACKET_T, SADDLE_T = 0.008, 0.004  # bracket plate and saddle thickness under the pipe
PIPE_ZC = BRACKET_T + SADDLE_T + PIPE_R  # pipe axis 2.25 cm above the brackets' underside
PIPE_TOP = PIPE_ZC + PIPE_R
SCREW_X = -0.060  # the screws stand 6 cm in front of the pipe (toward the robot), clear of it from above
STAND_Y = {"sink": 0.125, "mid": 0.0, "stack": -0.125}  # 12.5 cm apart (25 cm full size); the end stands are more
# than 12 cm from y = 0, so both arms can turn them at once
SPAN = STAND_Y["sink"] - STAND_Y["stack"]  # 0.25 m between the end stands
H0 = 0.050  # m: the brackets' underside at the start (dead level): pipe axis 7.25 cm above the floor
LEAD = 0.0254 / 13  # m per turn: a 1/2-13 UNC screw (13 threads per inch, 1.954 mm per turn), the thread of common
# 1/2 inch leveling mounts; the model's screws are that real thread
TRAVEL = {"sink": 2.1, "mid": 1.6, "stack": 2.1}  # turns from fully in (0) to each screw's stop (4.1 / 3.1 mm)
SEAT = {"sink": 1.0, "mid": 0.5, "stack": 1.1}  # turns out at which each head's hinge zero sits (lv.Foot seat_turns),
# one turn from each end's start: hinge limits -365/+426 (sink), -185/+426 (middle) and -401/+390 degrees (stack), all
# within the 1.25 turns (450 degrees) proven in the sim; the ends start 360 degrees from their zero, the middle 180
START = {"sink": 0.0, "mid": 0.0, "stack": 2.1}  # turns out at the start: sink end fully in, stack end at its stop
START_Z = {"sink": H0, "mid": H0 - 0.5 * LEAD, "stack": H0}  # bracket heights at the start: level, middle 1 mm low
FEET = tuple(lv.Foot(n, (SCREW_X, STAND_Y[n]), kind="screw", ext=START_Z[n] - LEAD * START[n], pitch=LEAD,
                     start_turns=START[n], travel_turns=TRAVEL[n], stop=True, seat_turns=SEAT[n])
             for n in ("sink", "mid", "stack"))
HEAD_Z, HEAD_R, HEAD_H = 0.100, 0.014, 0.016  # hex heads 2.4 cm across flats, 9.2 to 10.8 cm up (above the pipe)
SPARE_TURNS = 0.0  # no hinge margin past the stop beyond twistkit's own 30 degrees (keeps the upper limit at 426)

# --- Scoring thresholds (reasons in score() and the README).
INCH_PER_FT = 1000.0 / 12.0  # 1 inch per foot = 83.3 mm per m
FALL_MIN = 0.25 * INCH_PER_FT  # 20.8 mm/m: IPC Table 704.1, 1/4 inch per foot for drains 2.5 inches and smaller
FALL_HALF = 0.125 * INCH_PER_FT  # 10.4 mm/m: 1/8 inch per foot, the code minimum only for 3 inch pipe and larger
TRAP_TO_STACK = 0.500  # m (model): developed length from the trap's weir (y +25) to the stack's axis (y -25); 1.0 m
# full size, inside the IPC's 5 ft trap-to-vent limit for a 1.25 inch trap
PIPE_DIAM = 0.03175 / 2  # m (model): the arm's 1.25 inch nominal diameter at 1:2
FALL_MAX = 1000.0 * PIPE_DIAM / TRAP_TO_STACK  # 31.75 mm/m: total fall from slope at most one pipe diameter
# How a real trial reads each threshold: a digital angle gauge laid on the pipe (degrees), or a steel rule at the two
# end stands (difference of the pipe-center heights over their 25 cm spacing, mm). 10.4 / 20.8 / 31.75 mm/m read
# 0.60 / 1.19 / 1.82 degrees and 2.6 / 5.2 / 7.9 mm.
GAUGE_DEG = {k: math.degrees(math.atan(v / 1000.0)) for k, v in
             (("half", FALL_HALF), ("min", FALL_MIN), ("max", FALL_MAX))}
RULE_MM = {k: math.tan(math.radians(v)) * SPAN * 1000.0 for k, v in GAUGE_DEG.items()}
SLACK_MM = 0.5  # the middle saddle may sit at most 0.5 mm below a string line pulled between the end saddles (a 0.5
# mm feeler gauge does not go in between it and the pipe) ...
HUMP_MM = 0.25  # ... and at most 0.25 mm above it: the pipe then rocks on the middle and lifts an end saddle by twice
# that (the middle stand is halfway between the ends), 0.5 mm, the same feeler gauge
HOME_TOL = 0.03  # m: a gripper's tool point this close to where it was at the trial's first step counts as home
HOME_STILL = 0.002  # m per control step: the tool point not moving
GRIPPERS = ("left_gripper", "right_gripper")
HANDS_OFF = 0.03  # m: a tool point within 3 cm of the drain assembly (pipe, level, brackets, screw heads) is a hand
# still on the job, not let go; the arms' home tool points sit 8.5 cm past the pipe's ends and 15 cm above the heads

PVC = kit.Look((0.93, 0.93, 0.92), 0.5)
STEEL = kit.Look((0.62, 0.64, 0.67), 0.35, metallic=0.7)
SADDLE = kit.Look((0.12, 0.12, 0.13), 0.6)
LEVEL_BODY = kit.Look((0.95, 0.45, 0.08), 0.45)  # an orange torpedo level
FITTING = kit.Look((0.55, 0.56, 0.58), 0.5)

# The torpedo level: a 23 cm (9 inch) level at 1:2, lying on the pipe between the sink and middle stands. Its vial reads
# level (bubble between the two lines) at 0 fall; the bubble runs toward the high end, so at the code's fall it sits
# toward the sink end, about three quarters of the way to the vial's end.
LEVEL_C = (0.0, 0.075)  # body frame
LEVEL_SIZE = (0.016, 0.115, 0.016)
LEVEL = lv.Vial(at=(LEVEL_C[0], LEVEL_C[1], PIPE_TOP + LEVEL_SIZE[2] + 0.001), radius=0.030, circle_deg=0.25,
                full_deg=2.0, kind="tube", axis=(0.0, 1.0), bubble_r=0.008)


def _drain():
    """The pipe, the three stand brackets with their V-saddles, and the torpedo level, all on the tilting link. The
    brackets have no colliders (the screws pass through them; the heads and pads are the twistkit threads')."""
    m, c = [], []
    y0, y1 = PIPE_Y
    m.append(kit.cylinder((0, (y0 + y1) / 2, PIPE_ZC), PIPE_R, y1 - y0, PVC, axis="y"))
    c.append(kit.Collider((0, (y0 + y1) / 2, PIPE_ZC), size=(2 * PIPE_R, y1 - y0, 2 * PIPE_R)))
    for f in FEET:
        y = f.xy[1]
        # bracket plate from around the screw to under the pipe, a boss around the screw, a V-saddle under the pipe
        m.append(kit.box(((SCREW_X - 0.012 + 0.014) / 2, y, BRACKET_T / 2), (0.014 - SCREW_X + 0.012, 0.020, BRACKET_T),
                         STEEL))
        m.append(kit.cylinder((SCREW_X, y, BRACKET_T / 2 + 0.002), 0.010, BRACKET_T + 0.004, STEEL))
        m.append(kit.box((0, y, BRACKET_T + SADDLE_T / 2), (0.030, 0.020, SADDLE_T), SADDLE))
        for s in (-1, 1):  # the V's cheeks either side of the pipe
            m.append(kit.box((s * (PIPE_R + 0.003), y, BRACKET_T + SADDLE_T + 0.005), (0.005, 0.020, 0.010), SADDLE))
    lc = (LEVEL_C[0], LEVEL_C[1], PIPE_TOP + LEVEL_SIZE[2] / 2)
    m.append(kit.box(lc, LEVEL_SIZE, LEVEL_BODY))
    c.append(kit.Collider(lc, size=LEVEL_SIZE))
    m.extend(lv.vial_meshes(LEVEL))
    return lv.Leveled(PIPE, pos=PIPE_POS, size=(2 * PIPE_R, y1 - y0, PIPE_TOP), meshes=tuple(m), colliders=tuple(c),
                      feet=FEET, com=(0.0, 0.0), load_kg=0.3, top=PIPE_TOP, vials=(LEVEL,))


DRAIN = _drain()
SCREWS = lv.screw_feet(DRAIN, knob_z=HEAD_Z, knob_r=HEAD_R, knob_h=HEAD_H, sides=6, spare_turns=SPARE_TURNS)
TRAP = "sink_trap"  # the sink's trap and tailpiece (fixed): its outlet is where the arm will be glued in
STACK = "stack"  # the 3 inch vertical stack (fixed): the arm will go into its sanitary tee
OBJECTS = [DRAIN, *SCREWS,
           Cylinder(TRAP, pos=(PIPE_POS[0], 0.250, TOP + 0.050), radius=0.016, height=0.100, color=(0.55, 0.56, 0.58)),
           Cylinder(STACK, pos=(PIPE_POS[0], -0.250, TOP + 0.080), radius=0.022, height=0.160,
                    color=(0.93, 0.93, 0.92))]


# --- What happened during the run. success() and score() are called after every control step; the history is kept
# here and started afresh on every scene reset (kit.episode()). Nothing in this task is irreversible (the pipe is not
# glued yet), so only the arms' home position is remembered.
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


def _saddles_mm(objects):
    """Each stand's saddle height (its bracket's underside, mm above the table), read from its screw's turns."""
    st = lv.state(objects, PIPE)
    return {f: st.ext_mm[f] for f in STAND_Y}


def fall(objects):
    """The fall toward the stack (-y), mm per m, between the two end saddles (the pipe-center heights at the end stands
    over their 25 cm spacing); negative = back-fall toward the sink. The score also checks the pipe's own fall
    (pipe_fall), so a pipe rocked onto a high middle stand passes neither way unless both agree."""
    z = _saddles_mm(objects)
    return (z["sink"] - z["stack"]) / SPAN


def middle_off_line_mm(objects):
    """How far the middle saddle sits off the string line between the end saddles, mm (+ above: the pipe humps over
    it and an end lifts off; - below: it hangs slack and the pipe bellies there)."""
    z = _saddles_mm(objects)
    t = (STAND_Y["mid"] - STAND_Y["stack"]) / SPAN
    return z["mid"] - (z["stack"] + t * (z["sink"] - z["stack"]))


def all_bearing(objects):
    """Every stand carries the pipe: the middle saddle at most 0.5 mm below the end-to-end line (a belly) and at most
    0.25 mm above it (a hump that lifts an end 0.5 mm off its saddle)."""
    return -SLACK_MM <= middle_off_line_mm(objects) <= HUMP_MM


def pipe_fall(objects):
    """The pipe's own fall toward the stack, mm per m, as it rests on the stands that carry it (an angle gauge laid on
    the pipe). It differs from the saddle fall when the pipe rocks on a high middle stand."""
    return lv.slope_mm_per_m(objects, PIPE, toward=(0.0, -1.0))


def hands_near(objects):
    """A gripper's tool point within 3 cm of the drain assembly's bounds: x from the screw heads' front to the level's
    far side, y over the pipe's length, z from the table to the heads' tops plus the pad tips' reach below the tool
    point."""
    x0, x1 = PIPE_POS[0] + SCREW_X - HEAD_R, PIPE_POS[0] + PIPE_R + 0.008
    y0, y1 = PIPE_POS[1] + PIPE_Y[0], PIPE_POS[1] + PIPE_Y[1]
    ztop = TOP + HEAD_Z + HEAD_H / 2 + PAD_TIP_PAST_TCP
    for g in GRIPPERS:
        if g in objects:
            x, y, z = np.asarray(objects[g].pos, float)
            if (x0 - HANDS_OFF <= x <= x1 + HANDS_OFF and y0 - HANDS_OFF <= y <= y1 + HANDS_OFF
                    and z <= ztop + HANDS_OFF):
                return True
    return False


def _job(objects):
    st = lv.state(objects, PIPE)
    if lv.held(objects, PIPE) or st.pressed or hands_near(objects):
        return 0.0  # nothing counts while a hand is still on the job: fingers on a head, pressing, or resting nearby
    falls = (fall(objects), pipe_fall(objects))  # the saddles' fall and the pipe's own: both must pass
    bearing = all_bearing(objects)
    if all(FALL_MIN <= f <= FALL_MAX for f in falls):
        # inside the code's window: full credit only with every stand carrying the pipe; a slack stand is a belly
        return 1.0 if bearing else 0.25
    if all(FALL_HALF <= f <= FALL_MAX for f in falls) and bearing:
        return 0.25  # falls the right way on every stand, but under 1/4 inch per foot (legal only for 3 inch and up)
    return 0.0  # level or nearly so, back-fall, steeper than one diameter over the arm (siphons the trap), or worse


def _score(objects):
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not _arms_home(objects):
        return 0.9
    return job


def success(objects):
    """Fall inside the code's window, every stand bearing, hands off, both arms home and still. Called first each step,
    so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """Scored on the final state (nothing is glued, so any setting can still be changed). The fall is the pipe's slope
    toward the stack in mm per m, both from the two end stands' saddle heights over their 25 cm spacing and from the
    pipe itself as it rests (an angle gauge on it); both must pass. Every stand bears when the middle saddle is at most
    0.5 mm below a string line between the end saddles (a belly) and at most 0.25 mm above it (a hump lifting an end
    0.5 mm).
    0 while a gripper's fingers are closed on a screw head, a hand presses down on the pipe, or a tool point is within
    3 cm of the drain assembly.
    1.0: fall from 20.8 mm/m (1/4 inch per foot, IPC Table 704.1's minimum for drains 2.5 inches and smaller) to 31.75
    mm/m (one 1.25 inch pipe diameter of total fall over the 1.0 m from the trap's weir to the stack, the IPC's limit
    for a fixture drain; steeper, the arm's outlet drops below the weir and siphons the trap), every stand bearing, and
    both arms back home and still (0.9 if not home). 0.25: a fall in that window with the middle off the line (a belly
    or a hump), or every stand bearing with a fall from 10.4 mm/m (1/8 inch per foot, allowed only for 3 inch pipe and
    larger) up to 20.8. 0: anything else, including doing nothing (dead level, middle stand 1 mm low), back-fall toward
    the sink, and a fall steeper than 31.75 mm/m. Possible scores: 0, 0.25, 0.9, 1.0."""
    return _score(objects)


# --- Demo -------------------------------------------------------------------------------------------------------------
PAD_DOWN_HEAD = 0.011  # the finger pads reach 1.1 cm down the 1.6 cm head (tips 9.7 cm up, 1.1 cm over the pipe's top)
HALF = 0.5 * FINGER_OPEN  # open only half way near the heads
DEMO_TURN = {"sink": 1.6 * 360.0, "mid": 0.5 * 360.0, "stack": -1.6 * 360.0}  # degrees (+ = run the screw out)
# sink end up 3.13 mm, stack end down 3.13 mm (6.25 mm over the 25 cm span = 25.0 mm/m, 0.30 inch per foot), middle up
# 1 mm to the line between them
STROKE = 120.0  # degrees of wrist roll per grip (from -60 to +60 degrees or back): 1.6 turns take five strokes


def _turn(foot, degrees, label):
    """Twistkit regrasp strokes on a stand's hex head (lv.turn_foot -> tk.twist_waypoints): straight down, fingers half
    open, each 120-degree stroke starting at -60 degrees of yaw (or +60 when turning the other way)."""
    gz = TOP + HEAD_Z + HEAD_H / 2 - PAD_DOWN_HEAD + PAD_TIP_PAST_TCP
    return lv.turn_foot(DRAIN, foot, degrees, z=gz, open_to=HALF, start_yaw=-math.copysign(STROKE / 2, degrees),
                        stroke=STROKE, step=15.0, label=label)


def demo():
    """The left arm runs the sink-end screw out 1.6 turns (bracket up 3.1 mm) while the right arm runs the stack-end
    screw in 1.6 turns (down 3.1 mm), five strokes each; the right arm goes home, and the left runs the middle screw out
    half a turn (up 1 mm, onto the line between the ends) and goes home. The arms are never both near y = 0 (the
    middle stand)."""
    left = _turn("sink", DEMO_TURN["sink"], "raise the sink-end stand")
    left += _turn("mid", DEMO_TURN["mid"], "bring the middle stand up to the pipe")
    left += [kit.Home(1.8)]
    right = _turn("stack", DEMO_TURN["stack"], "lower the stack-end stand") + [kit.Home(1.8)]
    tl, tr = (sum(w.duration for w in a) for a in (left, right))
    left += [kit.Home(max(tr - tl, 0.0) + 0.5, label="stay home")]
    right += [kit.Home(max(tl - tr, 0.0) + 0.5, label="stay home")]
    return {"left": left, "right": right}
