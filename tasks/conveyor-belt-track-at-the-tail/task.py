"""A benchtop belt conveyor (a 1:2 model of a light-duty slider-bed conveyor) runs with its belt drifting toward the far
side frame. A maintenance mechanic tracks it at the tail pulley with the two take-up screws: a small turn on one
screw, then several belt revolutions of watching before the next, until the belt runs centered and steady, with the
belt's tension left in its working range. Cranking a screw several turns at once (or the wrong one) runs the belt
into a side frame, where its edge frays.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import beltkit as bk  # noqa: E402  (the belt's tracking law and its drawing, on twistkit take-up screws)
import guardkit as gk  # noqa: E402
import twistkit as tk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "The belt on this conveyor is running off to one side. Fix it before it rubs the frame."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the workspace is clear for
# the next person or job. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# --- The conveyor (m, world: x forward, y to the robot's left, z above the table top). The belt runs along +y on top.
XC = 0.28  # the belt's center line
Y_TAIL, Y_HEAD = -0.035, 0.27  # tail and head pulley axes
Z_BELT = 0.10  # the belt's top run (the slider bed's top)
PULLEY_R = 0.02  # 40 mm pulleys (1:2 of a 3.2-inch pulley), their axes 2 cm below the belt
BELT_W = 0.13  # 130 mm belt (1:2 of a 10-inch belt)
FRAME_IN = (0.205, 0.355)  # the side frames' inner faces: 150 mm apart, so 10 mm of clearance a side
FRAME_T = 0.010  # side frame thickness; the frames stand 10 mm above the belt as edge guides
SCREW_X = {"near": 0.19, "far": 0.37}  # the take-up screws, just outside the side frames, on the pulley's axis line
SCREW_SPAN = SCREW_X["far"] - SCREW_X["near"]  # 0.18 m between them
Y_PLATE = -0.073  # the take-up end plates' outer faces; each screw's knob sits outside its plate
KNOB_L, KNOB_HUB, KNOB_LOBE = 0.028, 0.011, 0.006  # four-lobe knob: 28 mm long, 34 mm across the lobes
KNOB_IN = Y_PLATE - 0.004  # the knob's inner face (4 mm off the plate)
KNOB_MID = KNOB_IN - KNOB_L / 2
START_DEG = 720.0  # twistkit angle at the start: two turns of slot travel each way

# The take-up screws: M10 x 1.5 (1.5 mm of pulley travel per turn); the knob turns in place (captive screw), so the
# twistkit thread's own axial rise is set to nothing and beltkit reads the turns. Turning a knob clockwise, seen from
# its outer end, draws that end of the pulley outward (tightens that side). Slot ends 2 turns either way; past 2 turns
# loosened the screw leaves its bearing block's thread.
THREAD = tk.Thread(pitch=1e-6, turns=0.05, start=START_DEG, max_tight=START_DEG + 720.0, hand=-1, free_lift=0.0,
                   resist=0.05, tight_resist=0.15)
KNOBS = {"near": "takeup_near/knob", "far": "takeup_far/knob"}

# The belt law (see beltkit): 0.2 m/s (40 ft/min, a common light-duty package speed) on a 0.74 m loop, one revolution
# every 3.7 s; the belt answers a pulley change after a lag of one revolution and settles over about 12 (3 time
# constants of 16 s). Crowned-pulley centering length 3.2 m: a quarter turn of one screw (0.375 mm, 2.1 milliradian of
# skew) moves where the belt settles by 6.7 mm. The tail pulley starts 0.75 mm out of square (half a turn on the
# far screw), which settles the belt 13.3 mm toward the far frame: left alone it rubs that frame about 20 s in.
BELT = bk.Belt(near=KNOBS["near"], far=KNOBS["far"], start_deg=START_DEG, skew0=-0.00075 / SCREW_SPAN, e0=0.002,
               speed=0.20, loop=0.74, span=SCREW_SPAN, pitch=0.0015, travel=0.003, center_len=3.2, lag_revs=1.0,
               clearance=(FRAME_IN[1] - FRAME_IN[0] - BELT_W) / 2, stretch0=0.5, x_center=XC, y_tail=Y_TAIL,
               y_head=Y_HEAD, z_top=TOP + Z_BELT, width=BELT_W, frame_x=FRAME_IN, bar=(Y_TAIL, TOP + Z_BELT + 0.0185))
bk.setup(BELT)

# --- Scoring thresholds (each with its reason).
CENTERED = 0.003  # centered: within 3 mm, Hytrol's "within a quarter-inch of the center" (6.35 mm) at 1:2 scale
NEAR_CENTER = 0.006  # running clear but off center: within 6 mm (half an inch at 1:2), 0.25
STEADY_RATE = 0.0001  # steady: the edge moving under 0.1 mm/s at every step of the last 10 s, i.e. under 1 mm in 10 s
STEADY_S = 10.0  # (about 2.7 revolutions), less than a ruler at the tail can show
# Small turns, each waited out (Texas Belting: "Adjust no more than a quarter turn at a time, then run several revolutions
# before adjusting again"): more than 0.3 turn of adjustment (both screws together) within any 7.4 s, two revolutions of
# the 0.74 m loop at 0.2 m/s, latches 0. 0.3 is a quarter turn plus 0.05 turn (18 degrees) for the knob's slip and coast
# under a full quarter-turn stroke.
MAX_ADJUST = 0.30
ADJUST_WINDOW_S = 7.4
STRETCH_LO, STRETCH_HI = 0.3, 0.8  # belt stretch window (%): set at 0.5; under 0.3 the drive pulley slips under load,
#                                    over 0.8 the splice and the pulley bearings are overloaded (model window)
HANDS_OFF = 0.04  # let go: every tool point and pad tip 4 cm clear of a knob (a finger's width plus the arm's lag)
HOME_TOL = 0.03  # a gripper is home within 3 cm of where its tool point was at the trial's first scored step
HOME_STILL = 0.001  # and still: its tool point moved less than 1 mm since the last step (2 cm/s)

ALU = kit.Look((0.72, 0.74, 0.76), 0.35, metallic=0.7)  # extruded aluminum side frames
STEEL = kit.Look((0.55, 0.56, 0.58), 0.4, metallic=0.6)
DARK = kit.Look((0.20, 0.21, 0.23), 0.6, metallic=0.3)  # painted legs, motor
BELT_LOOK = kit.Look((0.07, 0.07, 0.08), 0.8)  # the belt where it wraps the pulleys and the return run
GUARD = kit.Look((0.92, 0.75, 0.10), 0.5)  # yellow pulley guards
KNOB_LOOK = kit.Look((0.13, 0.30, 0.70), 0.45)  # blue four-lobe knobs
WHITE = kit.Look((0.95, 0.95, 0.93), 0.6)


def _conveyor():
    """The fixed conveyor: legs, two side frames standing 10 mm above the belt as edge guides, the slider bed, the
    pulleys with the belt wrapped on them, the return run, the drive motor at the head, yellow guards over both pulley
    nips, and a white center mark on the tail guard."""
    x0, x1 = FRAME_IN
    yl, yh = Y_TAIL - 0.012, Y_HEAD + 0.012
    ym, ly = (yl + yh) / 2, yh - yl
    xm = (x0 + x1) / 2
    zp = Z_BELT - PULLEY_R
    parts = []  # (center, size, look, collide)
    for x in (x0 - FRAME_T / 2, x1 + FRAME_T / 2):  # side frames: z 0.05 to 0.11
        parts.append(((x, ym, 0.08), (FRAME_T, ly, 0.06), ALU, True))
        for y in (yl + 0.03, yh - 0.03):  # legs
            parts.append(((x, y, 0.025), (0.02, 0.02, 0.05), DARK, True))
    parts.append(((xm, (Y_TAIL + Y_HEAD) / 2, Z_BELT - 0.004), (x1 - x0, Y_HEAD - Y_TAIL - 2 * PULLEY_R, 0.008), STEEL,
                  True))  # slider bed
    parts.append(((xm, (Y_TAIL + Y_HEAD) / 2, zp - PULLEY_R + 0.0015), (BELT_W, Y_HEAD - Y_TAIL, 0.003), BELT_LOOK,
                  False))  # the return run
    parts.append(((x1 + FRAME_T + 0.03, Y_HEAD, 0.075), (0.06, 0.06, 0.05), DARK, True))  # gear motor at the head
    for y in (Y_TAIL, Y_HEAD):  # pulley guards: a bent plate over each nip, across the frames
        parts.append(((xm, y, Z_BELT + 0.016), (x1 - x0 + 2 * FRAME_T, 0.045, 0.003), GUARD, True))
        for x in (x0 - FRAME_T / 2, x1 + FRAME_T / 2):
            parts.append(((x, y, Z_BELT + 0.0075), (FRAME_T, 0.045, 0.014), GUARD, True))
    parts.append(((xm, Y_TAIL - 0.0225 + 0.003, Z_BELT + 0.0178), (0.003, 0.012, 0.0006), WHITE, False))  # center mark
    meshes = [kit.box(c, s, lk) for c, s, lk, _ in parts]
    for y in (Y_TAIL, Y_HEAD):  # pulleys, the belt wrapped on them
        meshes.append(kit.cylinder((xm, y, zp), PULLEY_R, BELT_W, BELT_LOOK, axis="x"))
        meshes.append(kit.cylinder((xm, y, zp), 0.006, x1 - x0, STEEL, axis="x"))
    meshes.append(kit.cylinder((x1 + FRAME_T + 0.03, Y_HEAD, 0.075 + 0.025 + 0.02), 0.022, 0.06, DARK, axis="y"))
    cols = [kit.Collider(c, size=s) for c, s, _, col in parts if col]
    return kit.Prop("conveyor", pos=(0.0, 0.0, TOP), size=(0.3, 0.45, 0.13), meshes=tuple(meshes),
                    colliders=tuple(cols))


def _takeup(side):
    """One tail take-up: a bearing block on the pulley shaft, an end plate bolted to the side frame, the M10 screw
    between them and a blue four-lobe knob on the screw's outer end with a white index mark on its top lobe."""
    x = SCREW_X[side]
    zp = Z_BELT - PULLEY_R
    meshes = [kit.box((0, Y_TAIL, zp), (0.016, 0.03, 0.026), STEEL),  # bearing block on the pulley shaft
              kit.box((0, Y_PLATE + 0.003, zp), (0.024, 0.006, 0.05), STEEL),  # end plate
              kit.box((0, (Y_PLATE + Y_TAIL) / 2, zp + 0.019), (0.012, Y_TAIL - Y_PLATE + 0.03, 0.006), STEEL),  # strap
              kit.cylinder((0, (Y_PLATE + Y_TAIL - 0.015) / 2, zp), 0.005, Y_TAIL - 0.015 - Y_PLATE, STEEL, axis="y")]
    cols = [kit.Collider((0, Y_TAIL, zp), size=(0.016, 0.03, 0.026)),
            kit.Collider((0, Y_PLATE + 0.003, zp), size=(0.024, 0.006, 0.05))]
    # the knob, in its own frame: origin on the screw axis at the knob's inner face, the knob along -y from it
    km = [kit.cylinder((0, -KNOB_L / 2, 0), KNOB_HUB, KNOB_L, KNOB_LOOK, axis="y"),
          kit.cylinder((0, 0.002, 0), 0.005, 0.004, STEEL, axis="y")]  # screw shank into the plate
    for k in range(4):
        a = k * math.pi / 2
        km.append(kit.cylinder((KNOB_HUB * math.cos(a), -KNOB_L / 2, KNOB_HUB * math.sin(a)), KNOB_LOBE, KNOB_L,
                               KNOB_LOOK, axis="y", n=16))
    km.append(kit.box((0, -KNOB_L - 0.0003, KNOB_HUB + 0.002), (0.004, 0.0008, 0.010), WHITE))  # index mark, outer face
    km.append(kit.box((0, -KNOB_L / 2, KNOB_HUB + KNOB_LOBE + 0.0003), (0.004, 0.02, 0.0008), WHITE))  # and over the top
    w = 2 * (KNOB_HUB + KNOB_LOBE)  # 34 mm across opposite lobes
    kc = (kit.Collider((0, -KNOB_L / 2, 0), size=(w, KNOB_L, 0.012)),
          kit.Collider((0, -KNOB_L / 2, 0), size=(0.012, KNOB_L, w)))
    cap = tk.Cap("knob", anchor=(0.0, KNOB_IN, zp), mass=0.03, meshes=tuple(km), colliders=kc,
                 grip=(0.0, -KNOB_L / 2, 0.0), width=w)
    return tk.Threaded(f"takeup_{side}", pos=(x, 0.0, TOP), size=(0.03, 0.1, 0.1), meshes=tuple(meshes),
                       colliders=tuple(cols), cap=cap, thread=THREAD, axis="Y")


OBJECTS = [_conveyor(), _takeup("near"), _takeup("far")]
KNOB_XYZ = {s: np.array([SCREW_X[s], KNOB_MID, TOP + Z_BELT - PULLEY_R]) for s in SCREW_X}


# The whole knob in its own frame (origin on the screw axis at its inner face, the knob along -y), for the let-go check.
KNOB_PARTS = (gk.Seg((0.0, 0.0, 0.0), (0.0, -KNOB_L, 0.0), KNOB_HUB + KNOB_LOBE),)
# Both arms back at their start pose and still (guardkit keeps it per trial, reset with kit.episode(); beltkit keeps
# the belt's own history the same way).
HOME = gk.ArmsHome(tol=HOME_TOL, still=HOME_STILL)
RUSHED = gk.Latch(steps=2)  # irreversible: more than 0.3 turn within 7.4 s (two steps in a row confirm it)


def _hands_off(objects):
    """No gripper holds a knob (twistkit: fingers closed on it) and every tool point and pad tip is 4 cm clear of
    every part of both knobs."""
    if any(tk.held(objects, k) for k in KNOBS.values()):
        return False
    return all(gk.released_clear(objects, k, KNOB_PARTS, HANDS_OFF) for k in KNOBS.values())


def _score(objects):
    # irreversible: the belt's edge touched a side frame at any moment (a frayed edge stays frayed), or a screw was
    # backed out of its bearing block (the pulley end let go)
    rushed = RUSHED.feed(objects, bk.adjusted(ADJUST_WINDOW_S) > MAX_ADJUST)  # fed every step
    if bk.rubbed() or any(tk.came_off(objects, k) for k in KNOBS.values()):
        return 0.0
    if rushed:
        return 0.0  # a turn of more than a quarter, or turns not waited out over two belt revolutions
    if not _hands_off(objects):
        return 0.0  # still holding (or touching) a knob
    if bk.max_rate(STEADY_S) >= STEADY_RATE:
        return 0.0  # still drifting
    off = max(abs(bk.offset(objects)), abs(bk.settle()))  # where it runs now and where it settles at these settings
    if off > NEAR_CENTER:
        return 0.0
    tension_ok = STRETCH_LO <= bk.stretch(objects) <= STRETCH_HI
    if off > CENTERED or not tension_ok:
        return 0.25  # running clear but off center, or centered with the belt too tight or too slack
    return 1.0 if (HOME.home(objects) or not REQUIRE_ARMS_HOME) else 0.9


def success(objects):
    """Full score: centered and steady, no rub, tension in its window, knobs let go, arms home. Called first each step,
    so it also records where the arms are (beltkit integrates the belt in the simulator's hook, before this)."""
    HOME.feed(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if the belt's edge reached a side frame (10 mm off center) at any moment, even if tracked back after; or a
    take-up screw was backed out of its block (more than 2 turns loosened); or more than 0.3 turn of adjustment (both
    screws together, out and back counted) was made within any 7.4 s (two belt revolutions). 0 if a gripper holds a knob or a tool
    point or pad tip is within 4 cm of one; 0 if the belt's edge moved 0.1 mm/s or more at any step of the last 10 s (still
    drifting). Then e is the larger of the belt's offset now and the offset it settles at with the screws as left:
    0 if e > 6 mm; 0.25 if 3 mm < e <= 6 mm, or if e <= 3 mm with the belt's stretch outside 0.3 to 0.8 percent;
    1.0 if e <= 3 mm with the stretch in the window and both arms home and still (0.9 if not).
    Possible scores: 0, 0.25, 0.9, 1."""
    return _score(objects)


# --- Demo (right arm; the left stays home) ---------------------------------------------------------------------
ROLL = math.pi / 2  # the right hand's grip turned to point along +y, end-on to a knob
F_LOOSE = -0.023  # fingers 46 mm apart: clear of the knob's 34 mm lobes
STROKE = 90.0  # a quarter turn per adjustment
HI = 180.0  # grip yaw at the start of a stroke: fingers closing vertically onto the top and bottom lobes


def _quarter(side, wait):
    """Tighten the knob on `side` a quarter turn (clockwise seen from its outer end: the grip's yaw going down), let
    go, back the hand 3 cm off along the screw and wait `wait` seconds watching the belt."""
    x, y, z = KNOB_XYZ[side]
    y -= 0.002  # pad tips 1.1 cm past the tool point stay 5 mm off the end plate
    out = [kit.Tilted(1.5, (x, y - 0.03, z), F_LOOSE, "in line with the knob", math.radians(HI), ROLL),
           kit.Tilted(1.0, (x, y, z), F_LOOSE, "fingers around the lobes", math.radians(HI), ROLL),
           kit.Tilted(0.6, (x, y, z), FINGER_CLOSED, "grip the knob", math.radians(HI), ROLL)]
    n = 6
    for k in range(1, n + 1):
        out.append(kit.Tilted(0.3, (x, y, z), FINGER_CLOSED, "a quarter turn", math.radians(HI - STROKE * k / n), ROLL))
    out.append(kit.Tilted(0.6, (x, y, z), F_LOOSE, "let go", math.radians(HI - STROKE), ROLL))
    out.append(kit.Tilted(1.0, (x, y - 0.03, z), F_LOOSE, "back off the knob", math.radians(HI - STROKE), ROLL))
    out.append(kit.Tilted(1.5, (x, y - 0.03, z + 0.03), F_LOOSE, "watch the belt", math.radians(HI), ROLL))
    out.append(kit.Tilted(max(wait - 1.5, 0.1), (x, y - 0.03, z + 0.03), F_LOOSE, "watch the belt", math.radians(HI),
                          ROLL))
    return out


def demo():
    """Right arm: the belt runs toward the far frame, so tighten the far take-up a quarter turn, let go and watch the
    belt for about 2 revolutions (it keeps drifting out for one, then turns back); tighten it a second quarter turn,
    let go and go home while the belt settles on the center line. The left arm stays home."""
    approach = [kit.Tilted(3.0, (KNOB_XYZ["far"][0], KNOB_XYZ["far"][1] - 0.03, KNOB_XYZ["far"][2] + 0.04), F_LOOSE,
                           "toward the far take-up", math.radians(HI), ROLL)]
    right = approach + _quarter("far", 8.0) + _quarter("far", 0.5)[:-1] + [kit.Home(2.5),
                                                                            kit.Home(40.0, label="hold at home")]
    total = sum(w.duration for w in right)
    return {"left": [kit.Home(total)], "right": right}
