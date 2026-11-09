"""Three 50 mL glass media bottles of freshly made liquid media stand on the bench, their blue screw caps done up tight,
next to an empty polypropylene autoclave tray. Before liquids in screw-cap bottles go into an autoclave, a lab worker
loosens each cap about a quarter to a half turn but leaves it on, and stands the bottles in a tray with a raised rim.
A tight cap traps the steam pressure and the bottle can burst; a cap taken off lets the medium boil over and be
contaminated once it comes out; the tray catches anything that boils over or breaks.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import twistkit as tk  # noqa: E402  (threaded caps on top of yamkit; nothing on disk changes)
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint, pick_place_waypoints  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Get these bottles ready for the autoclave."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the workspace is clear for
# the next person or job. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# The bottles: 50 mL borosilicate media bottles with GL 32 screw caps (true size). Heights are above the bottle's base.
BODY_R, BODY_H = 0.023, 0.068  # cylindrical body, then a shoulder up to the neck
SHOULDER_TOP = 0.080
NECK_R = 0.0145
RING_R, RING_TOP = 0.017, 0.084  # the pouring ring under the cap
CAP_R, CAP_H = 0.0185, 0.020  # the cap sits on the ring: from 8.4 to 10.4 cm
SEAT = RING_TOP  # the cap's seat (its lower edge when screwed down)
FILL = 0.045  # liquid height (50 mL)
BOTTLE_MASS, CAP_MASS = 0.12, 0.01
THREAD = tk.Thread(pitch=0.004, turns=1.25, start=20.0)  # GL 32: 4 mm pitch, off after 1.25 turns; starts snug

# The autoclave tray (fixed): polypropylene, its outside TRAY_X x TRAY_Y, rim TRAY_H tall, centered at TRAY_C.
TRAY_C = (0.33, 0.0)
TRAY_X, TRAY_Y, TRAY_H = 0.14, 0.36, 0.035
TRAY_WALL, TRAY_FLOOR = 0.006, 0.004

# Where each bottle starts (on the bench) and the tray slot the demo puts it in.
BOTTLES = {  # name: (start xy, demo slot xy, medium color)
    "broth": ((0.22, 0.24), (0.33, 0.11), (0.80, 0.55, 0.20)),  # nutrient broth, amber
    "saline": ((0.25, -0.24), (0.33, -0.11), (0.93, 0.92, 0.80)),  # buffered saline, nearly clear
    "medium": ((0.34, 0.25), (0.33, 0.0), (0.88, 0.32, 0.42)),  # cell-culture medium with phenol red, pink
}
CAPS = [f"{n}/cap" for n in BOTTLES]

# Scoring
LOOSE_MIN = 45.0  # a cap counts as loosened when backed off at least 45 degrees past its seat
LOOSE_MAX = 270.0  # and at most 270 (three quarters of a turn), so at least half a turn of thread still holds it
GRIPPED_MIN = 45.0  # and at least 45 degrees of that turning was done with a gripper holding the cap
TIP_TILT = 45.0  # a bottle tilted past 45 degrees (for 2 steps) has tipped over: irreversible
UPRIGHT = 15.0  # standing upright within 15 degrees
IN_TRAY_MARGIN = 0.0  # a bottle is in the tray with its whole base inside the rim, on the tray floor
VIOLATION_STEPS = 2
HOME_TOL = 0.03  # a gripper is home within 3 cm of where its tool point was at the trial's first scored step
HOME_STILL = 0.001  # and still: its tool point moved less than 1 mm since the last step (2 cm/s)
HOLD_MARGIN = 0.02  # a gripper holds a bottle when its fingers are closed (opening under 0.6) with its tool point
HOLD_OPEN = 0.6  # within 2 cm of the bottle's or cap's outside
STILL = 0.001  # a bottle is still when its base moved less than 1 mm since the last step (2 cm/s)

GLASS = kit.Look((0.80, 0.86, 0.88), 0.05, opacity=0.35)
CAP_BLUE = kit.Look((0.13, 0.33, 0.70), 0.45)
WHITE = kit.Look((0.95, 0.95, 0.93), 0.6)
TAPE = kit.Look((0.90, 0.86, 0.72), 0.8)
STRIPE = kit.Look((0.15, 0.15, 0.15), 0.8)
TRAY = kit.Look((0.86, 0.87, 0.86), 0.55)


def _bottle(name, xy, color):
    liquid = kit.Look(color, 0.25)
    body = [  # (radius, height) from the base up, glass above the liquid line only (stacked see-through layers wash out)
        (BODY_R, FILL), (BODY_R, BODY_H), (BODY_R - 0.003, BODY_H + 0.006), (0.016, SHOULDER_TOP - 0.002),
        (NECK_R, SHOULDER_TOP), (NECK_R, SEAT - 0.004), (0.0, SEAT - 0.004)]
    meshes = [kit.lathe([(0.0, 0.0), (BODY_R - 0.001, 0.0), (BODY_R - 0.0005, 0.002), (BODY_R - 0.0005, FILL),
                         (0.0, FILL)], liquid),
              kit.lathe(body, GLASS),
              kit.lathe([(RING_R, SEAT - 0.004), (RING_R, SEAT), (0.0, SEAT)], CAP_BLUE)]  # pouring ring
    for k in range(1, 5):  # white graduation marks at 10 mL steps, on the side facing the robot
        h = FILL * k / 5
        meshes.append(kit.box((-BODY_R - 0.0003, 0.0, h), (0.0008, 0.008 if k % 2 == 0 else 0.005, 0.0012), WHITE))
    meshes.append(kit.box((0.0, BODY_R + 0.0003, 0.036), (0.016, 0.0008, 0.020), WHITE))  # label on the left side
    # a strip of autoclave indicator tape across the label (stripes show when it has been through a cycle)
    meshes.append(kit.box((0.0, BODY_R + 0.0008, 0.036), (0.020, 0.0008, 0.006), TAPE))
    colliders = (kit.Collider((0, 0, BODY_H / 2), radius=BODY_R, height=BODY_H),
                 kit.Collider((0, 0, (BODY_H + SHOULDER_TOP) / 2), radius=0.017, height=SHOULDER_TOP - BODY_H),
                 kit.Collider((0, 0, (SHOULDER_TOP + SEAT) / 2), radius=RING_R, height=SEAT - SHOULDER_TOP))
    # the cap: ribbed blue GL 32 cap, a white marker line across its top (shows how far it has turned)
    cap_meshes = [kit.lathe([(CAP_R - 0.0008, 0.0), (CAP_R - 0.0008, CAP_H - 0.002), (CAP_R - 0.0025, CAP_H),
                             (0.0, CAP_H)], CAP_BLUE),
                  kit.box((0.006, 0.0, CAP_H + 0.0003), (0.020, 0.0025, 0.0008), WHITE)]
    for k in range(24):
        a = 2 * math.pi * k / 24
        cap_meshes.append(kit.cylinder(((CAP_R - 0.0008) * math.cos(a), (CAP_R - 0.0008) * math.sin(a),
                                        CAP_H / 2 - 0.001), 0.0010, CAP_H - 0.004, CAP_BLUE, n=6))
    cap = tk.Cap("cap", anchor=(0.0, 0.0, SEAT), mass=CAP_MASS, meshes=tuple(cap_meshes),
                 colliders=(kit.Collider((0, 0, CAP_H / 2), radius=CAP_R, height=CAP_H),), grip=(0, 0, CAP_H / 2), width=2 * CAP_R)
    return tk.Threaded(name, pos=(xy[0], xy[1], TOP + 0.001), size=(2 * BODY_R, 2 * BODY_R, SEAT + CAP_H),
                       mass=BOTTLE_MASS, meshes=tuple(meshes), colliders=colliders, cap=cap, thread=THREAD)


def _tray():
    """A polypropylene autoclave tray with a raised rim, fixed on the bench."""
    lx, ly, h, t, f = TRAY_X, TRAY_Y, TRAY_H, TRAY_WALL, TRAY_FLOOR
    parts = [((0, 0, f / 2), (lx, ly, f)),
             ((lx / 2 - t / 2, 0, h / 2), (t, ly, h)), ((-lx / 2 + t / 2, 0, h / 2), (t, ly, h)),
             ((0, ly / 2 - t / 2, h / 2), (lx, t, h)), ((0, -ly / 2 + t / 2, h / 2), (lx, t, h))]
    meshes = [kit.box(c, s, TRAY) for c, s in parts]
    meshes.append(kit.box((lx / 2 - 0.02, -ly / 2 + 0.05, f + 0.0003), (0.02, 0.06, 0.0006), TAPE))  # a tape label
    return kit.Prop("tray", pos=(TRAY_C[0], TRAY_C[1], TOP), size=(lx, ly, h), meshes=tuple(meshes),
                    colliders=tuple(kit.Collider(c, size=s) for c, s in parts))


OBJECTS = [_tray()] + [_bottle(n, s, c) for n, (s, _, c) in BOTTLES.items()]


# --- What happened during the run. success() and score() are called after every control step, so the run's history is
# kept here. yamkit counts the simulator's scene resets (kit.episode()), so a new trial starts with a clean history.
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.tip_run = {n: 0 for n in BOTTLES}
        self.tipped = False  # a bottle tipped over: irreversible
        self.home = None
        self.last_grip = None
        self.grips_still = False
        self.last_bottle = {}
        self.bottle_still = {n: False for n in BOTTLES}


_H = _History()


def _tilt(s):
    x, y = s.quat_xyzw[0], s.quat_xyzw[1]
    return math.degrees(math.acos(float(np.clip(1 - 2 * (x * x + y * y), -1.0, 1.0))))


def _bottle_held(objects, name):
    """A gripper's fingers closed (opening under 0.6) with its tool point anywhere on the bottle or its cap: within 2 cm
    of its outside, from its base to 2 cm above the cap."""
    s = objects[name]
    p = np.asarray(s.pos, float)
    for g in ("left_gripper", "right_gripper"):
        if g not in objects or kit.opening(objects, g) >= HOLD_OPEN:
            continue
        d = np.asarray(objects[g].pos, float) - p
        if np.hypot(d[0], d[1]) < BODY_R + HOLD_MARGIN and -HOLD_MARGIN < d[2] < SEAT + CAP_H + HOLD_MARGIN:
            return True
    return False


def _in_tray(name, s):
    """Standing upright (within 15 degrees) on the tray floor with its whole base inside the rim, and still."""
    p = np.asarray(s.pos, float)
    half_x = TRAY_X / 2 - TRAY_WALL - BODY_R + IN_TRAY_MARGIN
    half_y = TRAY_Y / 2 - TRAY_WALL - BODY_R + IN_TRAY_MARGIN
    return bool(abs(p[0] - TRAY_C[0]) < half_x and abs(p[1] - TRAY_C[1]) < half_y
                and abs(p[2] - (TOP + TRAY_FLOOR)) < 0.01 and _tilt(s) < UPRIGHT and _H.bottle_still[name])


def _loosened(objects, key):
    """Backed off at least 45 degrees past the seat, with at least 45 degrees of turning done by a gripper, and on."""
    return (tk.backed_off_deg(objects, key) >= LOOSE_MIN and tk.gripped_deg(objects, key) >= GRIPPED_MIN
            and not tk.is_off(objects, key))


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
    for n in BOTTLES:  # yamkit reports no velocities for mechanisms, so stillness comes from the step-to-step motion
        p = np.asarray(objects[n].pos, float)
        _H.bottle_still[n] = n in _H.last_bottle and np.linalg.norm(p - _H.last_bottle[n]) < STILL
        _H.last_bottle[n] = p
        _H.tip_run[n] = _H.tip_run[n] + 1 if _tilt(objects[n]) > TIP_TILT else 0
        if _H.tip_run[n] >= VIOLATION_STEPS:
            _H.tipped = True


def _arms_home(objects):
    """Each gripper's tool point within 3 cm of where it was at the trial's first scored step, and still."""
    if not _H.home or not _H.grips_still:
        return False
    return all(np.linalg.norm(np.asarray(objects[g].pos, float) - p) < HOME_TOL for g, p in _H.home.items()
               if g in objects)


def _score(objects):
    # irreversible: a cap came off its thread (even if screwed back on), or a bottle tipped over
    if _H.tipped or any(tk.came_off(objects, k) for k in CAPS):
        return 0.0
    # any cap still tight (or loosened less than 45 degrees, or not by a gripper): that bottle can burst
    if not all(_loosened(objects, k) for k in CAPS):
        return 0.0
    if any(tk.backed_off_deg(objects, k) > LOOSE_MAX for k in CAPS):
        return 0.25  # a cap hanging on by less than half a turn of thread
    if not all(_in_tray(n, objects[n]) and not _bottle_held(objects, n) for n in BOTTLES):
        return 0.5  # caps right, but a bottle not standing in the tray (or still held)
    return 1.0 if (_arms_home(objects) or not REQUIRE_ARMS_HOME) else 0.9


def success(objects):
    """Full score: every cap loosened 45 to 270 degrees and still on, every bottle upright in the tray and let go, and
    (with REQUIRE_ARMS_HOME) both arms home and still. Called first each step, so it also records the history."""
    _update(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 if any cap came off its thread at any point (turned back more than 1.25 turns from its seat), even if it was
    screwed back on, or if any bottle tipped past 45 degrees for 2 steps; both are irreversible. 0 if any cap ends less
    than 45 degrees back from its seat (still tight), or was turned less than 45 degrees in total by a gripper holding
    it (fingers closed on the cap, centered on its axis: see twistkit.held). With every cap loosened: 0.25 if any cap
    is backed off more than 270 degrees (on by less than half a turn); otherwise 0.5 unless every bottle stands upright
    (within 15 degrees) and still (moved under 1 mm in the last step) on the tray floor, wholly inside its rim, with no
    gripper holding it; then 1.0 with both arms home and still (0.9 if not, with REQUIRE_ARMS_HOME).
    Possible scores: 0, 0.25, 0.5, 0.9, 1."""
    return _score(objects)


# --- Demo ----------------------------------------------------------------------------------------
GRIP_BODY = 0.045  # TCP height above a bottle's base for a grip around its body
GRIP_CAP = SEAT + 0.013  # TCP height above a bottle's base for a grip around its cap (pad tips 1.2 cm above the seat)
TURN = 150.0  # the demo loosens each cap 150 degrees, in two strokes (90 then 60)
LAG = np.array([0.0, 0.0, 0.0])  # where the arms fall short of their targets (tuned from runs)


def _move_and_loosen(name, side):
    start, slot, _ = BOTTLES[name]
    travel = TOP + 0.20
    pick = (start[0], start[1], TOP + 0.001 + GRIP_BODY)
    place = (slot[0], slot[1], TOP + TRAY_FLOOR + GRIP_BODY + 0.004)
    w = pick_place_waypoints(pick, place, travel_z=travel)
    w += tk.twist_waypoints(slot, TOP + TRAY_FLOOR + GRIP_CAP, TURN, side=side, stroke=90.0, pitch=THREAD.pitch,
                            label=f"loosen the {name} cap")
    return w


def demo():
    """Right arm: carry the saline bottle by its body into the tray's right slot, then loosen its cap 150 degrees in
    two wrist strokes (grip, roll, let go, roll back, grip, roll), lift clear and go home. Left arm, at the same time:
    the broth bottle into the left slot and its cap loosened the same way; then, once the right arm is home, the
    medium bottle into the middle slot and its cap loosened. Both arms end at home."""
    right = _move_and_loosen("saline", "right") + [kit.Home(2.0)]
    left = _move_and_loosen("broth", "left")
    t_right = sum(w.duration for w in right)
    t_left = sum(w.duration for w in left)
    park = (0.22, 0.22, TOP + 0.20)
    if t_left < t_right:
        left.append(Waypoint(1.0, park, FINGER_OPEN, "wait for the right arm"))
        left.append(Waypoint(t_right - t_left, park, FINGER_OPEN, "wait for the right arm"))
    left += _move_and_loosen("medium", "left") + [kit.Home(2.0), kit.Home(2.0, label="hold at home")]
    total = sum(w.duration for w in left)
    right.append(kit.Home(max(total - sum(w.duration for w in right), 0.5), label="hold at home"))
    return {"left": left, "right": right}
