"""Offline checks of the jack-stand task, no simulator. Stand states are built by hand from the same geometry the task
spawns; the arms' links are posed by the robot's forward kinematics along the scripted demo's own plan and checked with
their own collision hulls.

Covers: the ratchet's geometry (the bar resting on the pawl at every notch, the tooth below clear, the pawl swung out
clears every tooth, a bar let go between notches falls to the notch below, a load on the undercut face seats the pawl);
the reading (notch, locked, seated); doing nothing; the ladder (locked at 3 and checked, at 3 unchecked, other notches,
held up by a hand, braced between notches, the pawl lifted, tipped and set back, a hand still on the stand); the
handle check (a press on top counts, a hover 2.5 mm above or a touch from below does not, a press while the bar is not
locked does not); the demo plan's reach and its hands' clearance; no pxr at load or in demo(); the text's numbers.

    python3 test_task.py
"""
import math
import os
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import yamkit as kit  # noqa: E402

N_OK = [0]


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)
    N_OK[0] += 1


for _v in ("", "nothing", "nopush", "two", "brace"):
    os.environ["JACK_VARIANT"] = _v
    M.demo()
os.environ.pop("JACK_VARIANT", None)
_PXR = [m for m in sys.modules if m.split(".")[0] in ("pxr", "omni", "isaaclab")]


def quat_y(a):
    return np.array([0.0, math.sin(a / 2), 0.0, math.cos(a / 2)])


def st(pos, q=(0.0, 0.0, 0.0, 1.0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


S0 = np.array([M.STAND_XY[0], M.STAND_XY[1], M.TOP])


def objects(q_bar, th, tilt_deg=0.0):
    """The stand at its start place, tilted about y by tilt_deg (about its base origin), the bar q_bar up, the handle
    th lifted."""
    a = math.radians(tilt_deg)
    R = M.rotm(quat_y(a))
    qb = quat_y(a)
    kit._JOINTS["stand/bar"] = kit.JointState(float(q_bar), 0.0, False)
    kit._JOINTS["stand/handle"] = kit.JointState(float(th), 0.0, False)
    Rh = R @ M.rotm(quat_y(th))
    from scipy.spatial.transform import Rotation
    return {"stand": st(S0, qb), "stand/bar": st(S0 + R @ np.array([0, 0, q_bar]), qb),
            "stand/handle": st(S0 + R @ np.asarray(M.PIVOT), Rotation.from_matrix(Rh).as_quat()),
            "_joints": {"stand/bar": kit._JOINTS["stand/bar"], "stand/handle": kit._JOINTS["stand/handle"]}}


# -- 2D geometry of the ratchet (xz section) ---------------------------------------------------------------------------
def poly_overlap(a, b, tol=1e-6):
    """Separating-axis test of two convex xz polygons; True when they overlap by more than tol."""
    for P in (a, b):
        n = len(P)
        for i in range(n):
            e = np.subtract(P[(i + 1) % n], P[i])
            ax = np.array([-e[1], e[0]]) / (np.linalg.norm(e) + 1e-12)
            pa, pb = np.asarray(a) @ ax, np.asarray(b) @ ax
            if pa.max() <= pb.min() + tol or pb.max() <= pa.min() + tol:
                return False
    return True


def pawl_polys(th):
    """The pawl's two xz polygons (stand frame) with the handle lifted th (rad)."""
    px, pz = M.PIVOT[0], M.PIVOT[2]
    c, s = math.cos(th), math.sin(th)

    def turn(p):  # about +y: x' = x c + z s, z' = -x s + z c (relative to the pin)
        x, z = p[0] - px, p[1] - pz
        return (px + x * c + z * s, pz - x * s + z * c)

    return [[turn(p) for p in M._PAWL_TIP], [turn(p) for p in M._PAWL_ARM]]


def teeth_at(q):
    return [[(x, z + q) for x, z in M.tooth_poly(k)] for k in range(M.N_NOTCH + 1)]


def any_overlap(q, th, tol=1e-6):
    return any(poly_overlap(p, t, tol) for p in pawl_polys(th) for t in teeth_at(q))


check(not _PXR, "no pxr/omni/isaaclab imported by the task, its kits or demo() (any variant)")
P = M.PITCH
for k in range(1, M.N_NOTCH + 1):
    q = k * P
    check(not any_overlap(q, 0.0, 2e-5), f"notch {k}: the bar resting on the seated pawl, no overlap")
    check(any_overlap(q - 0.0003, 0.0), f"notch {k}: 0.3 mm lower the tooth is into the pawl (it rests on it)")
check(not any_overlap(0.0, 0.0), "lowest position: the seated pawl clear of every tooth")
for q in np.linspace(0.0, M.N_NOTCH * P + 0.004, 400):
    if any_overlap(q, 0.115):
        check(False, f"the pawl swung out 0.115 rad clears the teeth at q {q:.4f}")
check(True, "the pawl swung out 0.115 rad clears every tooth over the whole travel (the bar lifts freely)")
check(any_overlap(3 * P - 0.0003, 0.08), "with the pawl only part out (0.08 rad) tooth 3 still lands on it")
# a bar let go between notches falls to the notch below: at every height on the way down some pawl angle clears the
# teeth (the pawl rides the receding slope of the tooth below and swings home), and it is home by the time the
# tooth above lands on it


def theta_min(q):
    for th in np.linspace(0.0, 0.2, 201):
        if not any_overlap(q, th, 2e-5):
            return th
    return None


for k in (1, 3, 6):
    path = [theta_min(z) for z in np.linspace(k * P + P - 0.0007, k * P + 1e-5, 60)]
    check(all(t is not None for t in path) and path[-1] == 0.0,
          f"let go anywhere up to a notch above notch {k}: it falls to notch {k}, the pawl home when it lands")
# lifted past the tooth, the seated pawl is cammed out: the slope of the next tooth meets the pawl's lower edge
check(any_overlap(3 * P + P - 0.0045, 0.0), "lifting toward notch 4, tooth 4's slope meets the seated pawl (cams it out)")


# a load on the undercut face seats the pawl: torque about the pin (stand frame, y axis) of the face's normal force
def load_torque(x_contact):
    n = np.array([math.sin(math.radians(M.UNDERCUT_DEG)), -math.cos(math.radians(M.UNDERCUT_DEG))])  # x, z on pawl
    r = np.array([x_contact - M.PIVOT[0], M.pawl_top(x_contact) - M.PIVOT[2]])
    return r[1] * n[0] - r[0] * n[1]  # tau_y = z Fx - x Fz


for x in np.linspace(-M.BAR_HALF, M.ROOT_X - M.PAWL_GAP, 9):
    check(load_torque(x) < 0.0, f"the bar's weight on the pawl at x {x * 1000:.1f} mm turns it in (toward its stop)")
# a rising tooth pushes the pawl out, not jams it: the angle between the ramp's normal and the pawl tip's direction of
# travel, plus the friction angle (steel on steel 0.4), stays well under 90 degrees
_al = math.atan2(M.TOOTH_H - M.TOOTH_LAND, M.TOOTH_D)
_n = np.array([-math.sin(_al), math.cos(_al)])
_r = np.array([M._TIP_X - M.PIVOT[0], M.Z_PT - 0.0045 - M.PIVOT[2]])
_mv = np.array([_r[1], -_r[0]]) / np.linalg.norm(_r)
_ang = math.degrees(math.acos(float(_n @ _mv))) + math.degrees(math.atan(M.STEEL[0]))
check(_ang < 75.0, f"a rising tooth cams the pawl out (ramp normal vs pawl travel plus friction angle {_ang:.0f} deg)")
check(M.HANDLE_SEGS[3][0][0] < M.PIVOT[0], "the handle's front bar hangs in front of the pin: its weight swings the "
      "pawl in")


# lifting cams the pawl out with little more than the bar's weight (with the pin far in front of and low over the tip,
# the cam takes several newtons more, the pads slide up the ears and the cam's reaction shoves the stand).
# Force balance about the pin as the rising ramp first meets the pawl's lower tip corner, sliding (friction mu up the
# ramp on the pawl), against the handle's weight at its centroid (the colliders' volumes, as PhysX computes it).
def _handle_com():
    from scipy.spatial import ConvexHull
    vol, mom = 0.0, np.zeros(3)
    for c in M._HANDLE_COLS:
        p = np.asarray(c.center) + np.asarray(c.hull)
        o = p.mean(axis=0)
        for s in ConvexHull(p).simplices:
            a, b, d = p[s]
            v = abs(np.dot(a - o, np.cross(b - o, d - o))) / 6
            vol, mom = vol + v, mom + v * (a + b + d + o) / 4
    return mom / vol


G = 9.81
MU = M.STEEL[0]
_t = np.array([math.cos(_al), math.sin(_al)])  # up the ramp
TAU_G = M.HANDLE_KG * G * abs(float(_handle_com()[0]))  # the handle's weight about the pin
LEVER = (_r[1] * _n[0] - _r[0] * _n[1]) + MU * (_r[1] * _t[0] - _r[0] * _t[1])  # opening torque per newton on the tip
N_CAM = TAU_G / LEVER if LEVER > 0 else math.inf  # the tooth's push that just lifts the pawl off its stop
UP = N_CAM * (_n[1] + MU * _t[1])  # its pull down on the bar (and up on the frame, through the pin)
SIDE = abs(N_CAM * (_n[0] + MU * _t[0]))  # and its push on the frame toward the robot
W_BAR = M.BAR_KG * G
check(N_CAM < 0.4 * W_BAR, f"camming the pawl out takes {N_CAM:.1f} N at the tooth (under 0.4 x the bar's weight, "
      f"{W_BAR:.1f} N): net lever {LEVER * 1000:.1f} mm against the handle's {TAU_G:.3f} N m")
_frame_w = (M.BASE_KG + M.HANDLE_KG) * G
check(SIDE < M.STEEL[1] * (_frame_w - UP), f"the cam's side push on the frame ({SIDE:.1f} N) is under its sliding "
      f"friction on the bench ({M.STEEL[1] * (_frame_w - UP):.1f} N): the stand does not walk while the bar lifts")
import gripkit  # noqa: E402
from yam_arena.constants import GRIPPER  # noqa: E402

_pad_mu = (gripkit.RUBBER_PAD[1] + M.STEEL[1]) / 2  # sliding, averaged
_hold = 2 * _pad_mu * min(GRIPPER["effort"], GRIPPER["stiffness"] * M.SQUEEZE)
check(_hold > 1.5 * (W_BAR + UP), f"the pads' sliding hold on the ears ({_hold:.0f} N at {M.SQUEEZE * 1000:.0f} mm "
      f"squeeze) is over 1.5 x the lift ({W_BAR + UP:.0f} N: the bar plus the cam)")

# -- the reading ------------------------------------------------------------------------------------------------------
check(M.notch(0.0) is None and M.notch(3 * P) == 3 and M.notch(3 * P + 0.0014) == 3 and M.notch(3 * P + 0.002) is None,
      "notch(): 3 within 1.5 mm, none between notches or at the bottom")
check(M.locked_notch(objects(3 * P, 0.0)) == 3 and M.locked_notch(objects(3 * P, 0.03)) is None,
      "locked only with the pawl fully home")
check(abs(M.H_MIN - 0.28575) < 1e-9 and abs(M.H_MIN + M.N_NOTCH * P - 0.42545) < 1e-9,
      "saddle 11.25 in at the lowest notch, 16.75 in at the highest")
check(abs(M.H_MIN + 3 * P - 0.32766) < 1e-4, "notch 3: saddle 12.9 in (327.7 mm) above the floor")


# -- the arms, posed along the demo's plan -----------------------------------------------------------------------------
def arm_poses(t, variant="", stop=None):
    """FK link poses along the demo's plan at time t; `stop` (m): the right's finger faces stopped this far from the
    tool point (on a part they close on), instead of where they are commanded."""
    import yam_arena.demo as demo
    from yam_arena import station
    from yam_arena.analysis import fk_link_poses_world
    from yam_arena.constants import FINGER_OPEN

    os.environ["JACK_VARIANT"] = variant
    try:
        d = M.demo()
    finally:
        os.environ.pop("JACK_VARIANT", None)
    out = {}
    for side in ("left", "right"):
        plan = demo.ArmPlan(d[side], station.BASES[side])
        a, _ = plan.action7(min(t, plan.duration - 1e-6))
        q = {f"joint{i + 1}": float(a[i]) for i in range(6)}
        frac = float(a[6]) if stop is None or side != "right" else stop / M.FINGER_HALF_OPEN
        q["joint7"] = q["joint8"] = frac * FINGER_OPEN  # opening fraction -> URDF metres
        poses = fk_link_poses_world(q, station.BASES[side])
        for ln in M.ARM_LINKS:
            out[f"{side}_robot/{ln}"] = (poses[ln][:3, 3].copy(), poses[ln][:3, :3].copy())
    return out


def t_of(label, variant="", side="right", frac=0.5):
    os.environ["JACK_VARIANT"] = variant
    try:
        seq = M.demo()[side]
    finally:
        os.environ.pop("JACK_VARIANT", None)
    t = 0.0
    for w in seq:
        if w.label == label:
            return t + frac * w.duration
        t += w.duration
    raise KeyError(label)


READY = arm_poses(0.0)
GRIP = arm_poses(t_of("grip the saddle", frac=1.0))
PRESS = arm_poses(t_of("hold"))


def run(states, _unused=None):
    """A fresh trial: feed (objects, link poses) per control step; returns the ledger."""
    kit._EPISODE["count"] += 1
    M._L = None
    for i, (o, lp) in enumerate(states):
        kit._EPISODE["last"] = i
        kit._JOINTS.update(o["_joints"])  # the simulator updates the joints every step
        M.LINK_POSES.clear()
        M.LINK_POSES.update(lp)
        M.score(o)
    return M._L


def hold(o, lp, n):
    return [(o, lp)] * n


L3 = objects(3 * P, 0.0)
n = M.HOLD_STEPS + 2
K = M.PRESS_STEPS  # a push: this many steps in a row on top of the grip
# doing nothing
led = run(hold(objects(0.0, 0.0), READY, 60), None)
check(led.score() == 0.0, "doing nothing: 0 (the bar starts at its lowest position)")
# locked at 3, checked, let go, still
led = run(hold(L3, PRESS, K) + hold(L3, READY, n), None)
check(led.checked and led.score() == 1.0, f"locked at notch 3, handle pushed down on top, hands clear, still: 1.0 "
      f"({led.checked} {led.score()} good {led.good} clear {led.clear} lock {led.lock})")
led = run(hold(L3, PRESS, K) + hold(L3, READY, M.HOLD_STEPS - K), None)
check(led.score() == 0.5, "checked but not yet still for 1 s: 0.5")
led = run(hold(L3, PRESS, K - 1) + hold(L3, READY, n), None)
check(not led.checked and led.score() == 0.5, f"a graze on top of the grip ({K - 1} steps, under 0.25 s): 0.5")
led = run(hold(L3, PRESS, K - 1) + hold(L3, READY, 1) + hold(L3, PRESS, K - 1) + hold(L3, READY, n), None)
check(not led.checked and led.score() == 0.5, "two short taps that do not add up to 0.25 s in a row: 0.5")
led = run(hold(L3, READY, 60), None)
check(not led.checked and led.score() == 0.5, "locked at notch 3, handle never pushed: 0.5")
for k in (1, 2, 4, 10):
    o = objects(k * P, 0.0)
    led = run(hold(o, PRESS, K) + hold(o, READY, n), None)
    check(led.score() == 0.25, f"locked at notch {k} (pushed or not): 0.25")
# the hand still on the stand at the end
led = run(hold(L3, PRESS, K) + hold(L3, PRESS, n), None)
check(led.score() == 0.0, "a hand still pressing the handle at the end: 0")
led = run(hold(L3, GRIP, n), None)
check(led.score() == 0.0, "the hand still holding the saddle at notch 3: 0")
# braced between notches (pawl home, bar off it), and held at notch 3 with the pawl lifted
for o, what in ((objects(3 * P + 0.007, 0.0), "the bar held half a notch over notch 3, pawl home"),
                (objects(3 * P, 0.15), "the bar at notch-3 height with the pawl swung out (handle lifted)"),
                (objects(3 * P, 0.04), "the bar at notch 3, the pawl only part home")):
    led = run(hold(o, PRESS, K) + hold(o, READY, n), None)
    check(led.score() == 0.0, what + ", hands clear: 0")
    led = run(hold(o, GRIP, n), None)
    check(led.score() == 0.0, what + ", held by the hand: 0")
# tipped
led = run(hold(objects(3 * P, 0.0, 6.0), READY, 3) + hold(L3, PRESS, K) + hold(L3, READY, n), None)
check(led.tipped and led.score() == 0.0, "the stand tipped 6 degrees at any moment, then set back and finished: 0")
led = run(hold(L3, PRESS, K) + hold(objects(3 * P, 0.0, 4.5), READY, n), None)
check(not led.tipped and led.score() == 1.0, "leaning 4.5 degrees (under the 5 degree limit): still scored")
# the handle check: hover, from below, before the bar is locked
M.LINK_POSES.clear()
M.LINK_POSES.update(PRESS)
kit._JOINTS.update(L3["_joints"])
check(M.arm_gaps(L3)["grip_top"] <= M.TOUCH, "the demo's press puts a fingertip on top of the grip (within 0.6 mm)")


def shifted(poses, dz):
    return {k: (p + np.array([0, 0, dz]), R) for k, (p, R) in poses.items()}


import yamgeom as yg  # noqa: E402

_p, _R = PRESS["right_robot/tip_left"]
SINK = M.TOP + M._FZ + M.SLEEVE_R - float((_p + yg.points("tip_left") @ _R.T)[:, 2].min())
check(0.002 < SINK < 0.008, f"the demo's press: the fingertips' lowest point {SINK * 1000:.1f} mm below the sleeve's top")
LIFT_OFF = shifted(PRESS, SINK + 0.0025)  # the tips 2.5 mm above the grip's top
M.LINK_POSES.clear()
M.LINK_POSES.update(LIFT_OFF)
g = M.arm_gaps(L3)["grip_top"]
check(g > 0.002, f"hovering 2.5 mm over the grip reads {g * 1000:.1f} mm (not a touch)")
led = run(hold(L3, LIFT_OFF, 5) + hold(L3, READY, n), None)
check(not led.checked and led.score() == 0.5, "a hover over the handle does not count as the check: 0.5")
HOVER1 = shifted(PRESS, SINK + 0.001)  # the tips 1 mm above the grip's top
led = run(hold(L3, HOVER1, 5) + hold(L3, READY, n), None)
check(not led.checked and led.score() == 0.5, "a hover 1 mm over the handle does not count as the check: 0.5")
# the check while a hand still holds the bar (the other arm on the saddle) does not count
BOTH = dict(PRESS)
BOTH.update({k.replace("right_robot", "left_robot"): v for k, v in GRIP.items() if k.startswith("right_robot")})
led = run(hold(L3, BOTH, 5) + hold(L3, READY, n), None)
check(not led.checked and led.score() == 0.5, "a press while a hand still holds the saddle does not count: 0.5")
# a check, then the bar lifted off notch 3 and set back: the old check no longer counts
led = run(hold(L3, PRESS, K) + hold(objects(3 * P + 0.007, 0.0), READY, 3) + hold(L3, READY, n), None)
check(not led.checked and led.score() == 0.5, "checked, then the bar lifted and set back at notch 3: 0.5")
a_, b_, r_ = M._GRIP_SEGS[1]
mid = (a_ + b_) / 2
under = np.array([mid + [0, 0, -r_ - 0.0005], mid + [0.004, 0, -r_ + 0.001]])
over = np.array([mid + [0, 0, r_ + 0.0005]])
check(np.isinf(M._seg_gap(under, M._GRIP_SEGS, True)).all() and M._seg_gap(over, M._GRIP_SEGS, True)[0] < M.TOUCH,
      "a touch on the grip's underside does not read as a push on top; one on top does")
led = run(hold(objects(3 * P + 0.007, 0.0), PRESS, 5) + hold(L3, READY, n), None)
check(not led.checked and led.score() == 0.5, "a press while the bar is not yet locked at 3 does not count: 0.5")
led = run(hold(objects(2 * P, 0.0), PRESS, 5) + hold(L3, READY, n), None)
check(not led.checked and led.score() == 0.5, "a press while the bar is locked at notch 2 does not count for notch 3")

# -- the demo plan: reach (station bases) and the hands' clearance from the column and the handle -----------------------
import yam_arena.demo as demo  # noqa: E402
from yam_arena import station  # noqa: E402

for variant in ("", "two", "nopush", "brace"):
    os.environ["JACK_VARIANT"] = variant
    try:
        plan = demo.ArmPlan(M.demo()["right"], station.BASES["right"])
    finally:
        os.environ.pop("JACK_VARIANT", None)
    worst = 0.0
    for k in range(int(plan.duration / 0.05)):
        a, _ = plan.action7(k * 0.05)
        pos = plan.target(k * 0.05)[0]
        p, _ = demo.ik.tcp_pose(a[:6], station.BASES["right"])
        worst = max(worst, float(np.linalg.norm(p - pos)))
    check(worst < 0.003, f"demo{'/' + variant if variant else ''}: every waypoint in reach (IK within "
          f"{worst * 1000:.1f} mm)")
M.LINK_POSES.clear()
M.LINK_POSES.update({k: v for k, v in arm_poses(t_of("grip the saddle", frac=1.0), stop=M.SAD_Y).items()
                     if k.startswith("right_robot")})  # the right's pads stopped on the ears
gg = M.arm_gaps(objects(0.0, 0.0))
check(gg["column"] > 0.005 and gg["handle"] > 0.01, f"gripping the saddle (pads on the ears): the right's fingers "
      f"{gg['column'] * 1000:.1f} mm from the column head, {gg['handle'] * 1000:.1f} mm from the handle")

# the left hand steadying the stand: in reach, its fingers on the column head's front and back faces, clear of the
# handle while it grips, comes in and backs out; it lets go before the right pushes the handle
for variant in ("", "two", "nopush", "brace"):
    os.environ["JACK_VARIANT"] = variant
    try:
        plan = demo.ArmPlan(M.demo()["left"], station.BASES["left"])
    finally:
        os.environ.pop("JACK_VARIANT", None)
    worst = 0.0
    for k in range(int(plan.duration / 0.05)):
        a, _ = plan.action7(k * 0.05)
        p, _ = demo.ik.tcp_pose(a[:6], station.BASES["left"])
        worst = max(worst, float(np.linalg.norm(p - plan.target(k * 0.05)[0])))
    check(worst < 0.003, f"demo{'/' + variant if variant else ''}: the left's waypoints in reach (IK within "
          f"{worst * 1000:.1f} mm)")
_t_hold = t_of("hold the stand steady", side="left")
STEADY = {k: v for k, v in arm_poses(_t_hold).items() if k.startswith("left_robot")}
M.LINK_POSES.clear()
M.LINK_POSES.update(STEADY)
gs = M.arm_gaps(objects(0.0, 0.0))
check(gs["column"] <= M.TOUCH and gs["handle"] > 0.005 and gs["grip_top"] > 0.01,
      f"the left gripping the column head: on its faces, {gs['handle'] * 1000:.1f} mm below the handle's arm")
_worst_h = 1.0
for _lab in ("left of the stand's column head, pointing right", "fingers either side of the column head, below the handle",
             "let go of the stand", "back out to the left"):
    for _f in (0.0, 0.5, 1.0):
        M.LINK_POSES.clear()
        M.LINK_POSES.update({k: v for k, v in arm_poses(t_of(_lab, side="left", frac=_f)).items()
                             if k.startswith("left_robot")})
        _worst_h = min(_worst_h, M.arm_gaps(objects(0.0, 0.0))["handle"])
check(_worst_h > 0.002, f"the left comes in and backs out {_worst_h * 1000:.1f} mm or more from the handle")
_t_left_out = t_of("back out to the left", side="left", frac=1.0)
check(_t_left_out < t_of("push the handle down", frac=0.0) and t_of("let go of the stand", side="left", frac=0.0)
      >= t_of("back out, clear of the saddle", frac=1.0) - 1e-6,
      "the left lets go after the right has set the bar down and let go, and is clear before the handle push")
check(abs(M.tooth_bottom(0) - M.Z_PT) < 1e-9 and M.BAR.lower <= -0.002 and M.SAD_Z0 > M.COL_TOP,
      "at its lowest position the bar rests on the pawl by its lowest tooth (contact), 2 mm above its joint's lower "
      "limit, its saddle clear of the column's rim")

# -- the USD: every hull collider written as a convex hull ------------------------------------------------------------------
u = kit._mech_usda(M.STAND)
ncol = len(M.STAND.colliders) + sum(len(lk.colliders) for lk in M.STAND.links)
check(u.count('physics:approximation = "convexHull"') == ncol and "radius = 0.0" not in u,
      f"the stand's USD has {ncol} convex-hull colliders, no empty ones")
check(all(len(o.meshes) > 0 for o in M.OBJECTS), "every object has visible meshes")
check(1.4 < M.BAR_KG < 1.7 and M.BAR_KG < 2.0, f"the bar and saddle weigh {M.BAR_KG:.2f} kg (under the arm's 2 kg "
      "payload)")

# -- the text's numbers -----------------------------------------------------------------------------------------------------
txt = (HERE / "task.json").read_text() + (HERE / "README.md").read_text() if (HERE / "README.md").exists() else ""
if txt:
    for s in ("11-1/4", "16-3/4", "1.5 mm", "5 degrees", "5 mm", "0.6 mm", "1 s", "13.97", "T43002"):
        check(s in txt, f"text states {s!r}")
    check(re.search(r"1\.4 degrees", txt) is not None, "text states the 1.4 degree seated limit")
print(f"all {N_OK[0]} checks passed")
