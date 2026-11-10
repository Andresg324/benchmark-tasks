"""Offline checks of the hand-truck task's scorer: step-by-step histories of the truck (axle position, tilt, wheels), the
carton (on the truck or loose) and the grippers, with near misses and cheap tricks. No simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
TOP = m.TOP
PARK = {"left_gripper": (0.32, 0.305, TOP + 0.26), "right_gripper": (0.32, -0.305, TOP + 0.26)}
X = m.TRUCK_START[0]
Y0 = m.TRUCK_START[1]
Y_IN = Y0 - (m.GAP + m.PLATE_D)  # the axle once the plate is fully under the carton
SEAT = np.array([m.F + m.BOX[0] / 2, 0.0, -m.R + m.PLATE_T + m.BOX[2] / 2])  # the carton's center on the plate
C0 = np.array(m.BOX_START)


def quat(tilt_deg, yaw=0.0):
    """Yaw about z, then tilt back about the world x axis (top toward +y for a positive tilt)."""
    a, b = -math.radians(tilt_deg) / 2, yaw / 2
    qx = np.array([math.sin(a), 0, 0, math.cos(a)])
    qz = np.array([0, 0, math.sin(b), math.cos(b)])
    return np.array(kit._quat_mul(tuple(qx), tuple(qz)))


def st(pos, q, half=0.01):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - half,
                       high=p + half)


def truck_pose(axle_y, tilt, lift=0.0):
    p = np.array([X, axle_y, TOP + m.R + lift])
    return p, quat(tilt, m.TRUCK_YAW)


def handle(axle_y, tilt, lift=0.0):
    p, q = truck_pose(axle_y, tilt, lift)
    return p + m._rot(q) @ np.asarray(m.HANDLE)


def step(axle_y, tilt, carton, grips=None, lift=0.0):
    """carton: "load" (seated on the plate) or (position, tilt in degrees). grips: name -> position (fingers open),
    (position, opening), or "hold" (on the handle, closed on the 2 cm grip)."""
    tp, tq = truck_pose(axle_y, tilt, lift)
    if isinstance(carton, str):
        cp, cq = tp + m._rot(tq) @ SEAT, quat(tilt)
    else:
        cp = np.asarray(carton[0], float)
        cq = quat(carton[1]) if len(carton) < 3 else np.array([0, 0, math.sin(math.radians(carton[2]) / 2),
                                                               math.cos(math.radians(carton[2]) / 2)])
    rt = m._rot(tq)
    objs = {"truck": st(tp, tq), "carton": st(cp, cq), "spot": st((*m.SPOT, TOP), quat(0)),
            "truck/wheel_a": st(tp + rt @ np.array([0, -m.WHEEL_Y, 0]), tq),
            "truck/wheel_b": st(tp + rt @ np.array([0, m.WHEEL_Y, 0]), tq)}
    g = dict(PARK, **(grips or {}))
    for k, v in g.items():
        if isinstance(v, str):
            pos, op = tuple(handle(axle_y, tilt, lift)), 0.21
        elif isinstance(v[0], tuple):
            pos, op = v
        else:
            pos, op = v, 1.0
        p = np.array(pos, float)
        objs[k] = ObjectState(pos=p, quat_xyzw=quat(0), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                              low=p - [0, 0, 0.013], high=p)
        kit._OPENING[k] = op
    ok = m.success(objs)
    return m.score(objs), ok


def run(frames):
    """frames: (steps, axle_y, tilt, carton[, grips[, lift]]). A fresh trial each time."""
    kit._EPISODE["count"] += 1
    out = step(Y0, 0.0, (C0, 0.0))  # the trial's first scored step: the arms at their ready pose
    for fr in frames:
        n, y, th, c = fr[:4]
        g = fr[4] if len(fr) > 4 else None
        lift = fr[5] if len(fr) > 5 else 0.0
        for _ in range(n):
            out = step(y, th, c, g, lift)
    return out


H = {"left_gripper": "hold"}
ROLL = m.SPOT[1] - m.BOX_START[1]
Y_END = Y_IN + ROLL
Y_OUT = Y_END + m.PLATE_D + 0.02
END_C = (np.array([X, m.SPOT[1], C0[2]]), 0.0)


def ramp(a, b, n=10):
    return [a + (b - a) * k / n for k in range(1, n + 1)]


# The right way: push the plate under (the carton stays put), tilt back to 45, roll, stand up, slide out, let go.
INSERT = [(1, y, 0.0, (C0, 0.0), H) for y in ramp(Y0, Y_IN + 0.071)] + [(5, Y_IN, 0.0, "load", H)]
TILT = [(1, Y_IN, th, "load", H) for th in ramp(0.0, 45.0)]
ROLLING = [(1, y, 45.0, "load", H) for y in ramp(Y_IN, Y_END, 20)]
STAND = [(1, Y_END, th, "load", H) for th in ramp(45.0, 0.0)] + [(3, Y_END, 0.0, END_C, H)]
SLIDE = [(1, y, 0.0, END_C, H) for y in ramp(Y_END, Y_OUT, 5)]
LETGO = [(40, Y_OUT, 0.0, END_C)]
GOOD = INSERT + TILT + ROLLING + STAND + SLIDE + LETGO
NEAR_OPEN = {"left_gripper": (tuple(handle(Y_OUT, 0.0) + [0, 0.0, 0.05]), 1.0)}
NEAR_SHUT = {"left_gripper": (tuple(handle(Y_OUT, 0.0) + [0, 0.03, 0.0]), 0.0)}
RAIL_SHUT = {"left_gripper": ((X - 0.038, Y_OUT - 0.02, TOP + 0.10), 0.0)}
ON_CARTON = {"right_gripper": ((X, m.SPOT[1], TOP + m.BOX[2] + 0.01), 0.0)}


def shove(c_from, c_to, n=10, y_from=Y0, y_to=None, tilt=0.0, grips=H):
    """The truck pushed along with the carton loose in front of it (the plate against or partly under it)."""
    y_to = y_from + (c_to[1] - c_from[1]) if y_to is None else y_to
    return [(1, y, tilt, (np.array(c), 0.0), grips) for y, c in zip(ramp(y_from, y_to, n), ramp(np.array(c_from), np.array(c_to), n))]


def tilt_roll(th, lift=0.0):
    """Load, tilt to `th` degrees, roll to the spot with the wheels `lift` off the floor, stand up, slide out, let go."""
    return (INSERT + [(1, Y_IN, t, "load", H, lift) for t in ramp(0, th)]
            + [(1, y, th, "load", H, lift) for y in ramp(Y_IN, Y_END, 20)]
            + [(1, Y_END, t, "load", H) for t in ramp(th, 0.0)] + [(3, Y_END, 0.0, END_C, H)] + SLIDE + LETGO)


def ratchet(hop, rock):
    """Scrape the loaded, upright truck `hop` at a time to the spot, rocking it through the tilts `rock` between hops."""
    fr, y = list(INSERT), Y_IN
    while y > Y_END + 1e-9:
        fr += [(1, y, t, "load", H) for t in rock]
        ny = max(Y_END, y - hop)
        fr += [(1, yy, 0.0, "load", H) for yy in ramp(y, ny, 5)]
        y = ny
    return fr + [(1, Y_END, t, "load", H) for t in (10, 20, 10, 0)] + [(3, Y_END, 0.0, END_C, H)] + SLIDE + LETGO


def on_truck(axle_y, tilt, offset):
    """The carton's center on the truck, `offset` (truck frame) from its seat."""
    tp, tq = truck_pose(axle_y, tilt)
    return tp + m._rot(tq) @ (SEAT + np.asarray(offset, float))


def turned_off(yaw, dy):
    """The whole job, but the truck set down `dy` past the spot's center and the carton left turned `yaw` degrees."""
    ye = Y_END - dy
    c = (END_C[0] + [0, -dy, 0], 0.0, yaw)
    return (INSERT + TILT + [(1, y, 45.0, "load", H) for y in ramp(Y_IN, ye, 20)]
            + [(1, ye, th, "load", H) for th in ramp(45.0, 0.0)] + [(3, ye, 0.0, c, H)]
            + [(1, y, 0.0, c, H) for y in ramp(ye, ye + m.PLATE_D + 0.02, 5)] + [(40, ye + m.PLATE_D + 0.02, 0.0, c)])


cases = [
    ("nothing", [(100, Y0, 0.0, (C0, 0.0))], 0.0),
    ("the whole job, arms home", GOOD, 1.0),
    ("the whole job, arm parked over the handle, fingers open", GOOD[:-1] + [(40, Y_OUT, 0.0, END_C, NEAR_OPEN)], 0.9),
    ("the whole job, fingers closed beside the grip", GOOD[:-1] + [(40, Y_OUT, 0.0, END_C, NEAR_SHUT)], 0.5),
    ("the whole job, fingers closed on a rail", GOOD[:-1] + [(40, Y_OUT, 0.0, END_C, RAIL_SHUT)], 0.5),
    ("the whole job, a closed gripper pressing on the carton's top", GOOD[:-1] + [(40, Y_OUT, 0.0, END_C, ON_CARTON)],
     0.5),
    ("plate still under the carton on the spot, let go", INSERT + TILT + ROLLING + STAND + [(40, Y_END, 0.0, END_C)],
     0.5),
    ("plate slid only 3 cm out", INSERT + TILT + ROLLING + STAND + [(40, Y_END + 0.03, 0.0, END_C)], 0.5),
    ("still tilted and held on the spot", INSERT + TILT + ROLLING + [(40, Y_END, 45.0, "load", H)], 0.5),
    ("loaded and tilted, never rolled, stood back up", INSERT + TILT + [(1, Y_IN, th, "load", H) for th in
                                                                         ramp(45.0, 0.0)] + [(40, Y_IN, 0.0, "load")],
     0.25),
    ("rolled halfway, stood up there and slid out", INSERT + TILT + ROLLING[:10] + [
        (1, Y_IN + ROLL / 2, th, "load", H) for th in ramp(45.0, 0.0)]
     + [(40, Y_IN + ROLL / 2 + 0.1, 0.0, (C0 + [0, ROLL / 2, 0], 0.0))], 0.25),
    ("plate pushed under, never tilted", INSERT + [(40, Y_IN, 0.0, "load")], 0.0),
    ("tilted only 5 degrees", INSERT + [(1, Y_IN, th, "load", H) for th in ramp(0, 5)] + [(40, Y_IN, 5.0, "load", H)],
     0.0),
    ("the plate shoves the carton 18 cm to the spot without tilting", shove(C0, (X, m.SPOT[1], C0[2]), 20)
     + [(40, Y0 + ROLL + 0.05, 0.0, END_C)], 0.0),
    ("the plate shoves the carton 4 cm, then the whole job", shove(C0, C0 + [0, -0.04, 0], 5)
     + [(1, Y_IN - 0.04, th, "load", H) for th in ramp(0, 45)]
     + [(1, y, 45.0, "load", H) for y in ramp(Y_IN - 0.04, Y_END, 20)] + STAND + SLIDE + LETGO, 0.0),
    ("the plate nudges the carton 2 cm before sliding under, then the whole job", shove(C0, C0 + [0, -0.02, 0], 5)
     + GOOD, 1.0),
    ("carton dragged on the plate at 0 degrees (not lifted) to the spot", INSERT
     + [(1, y, 0.0, "load", H) for y in ramp(Y_IN, Y_END, 20)] + SLIDE + LETGO, 0.0),
    ("carton dragged at a 5 degree tilt (still on the floor)", INSERT
     + [(1, Y_IN, th, "load", H) for th in ramp(0, 5, 3)]
     + [(1, y, 5.0, "load", H) for y in ramp(Y_IN, Y_END, 20)] + SLIDE + LETGO, 0.0),
    ("carton lifted 3 cm by two grippers squeezing it", [(5, Y0, 0.0, (C0 + [0, 0, 0.03], 0.0))] + GOOD, 0.0),
    ("carton lifted 1 cm (rocked), set back, then the whole job", [(5, Y0, 0.0, (C0 + [0, 0, 0.01], 0.0))] + GOOD, 1.0),
    ("carton tipped onto its side", [(40, Y0, 0.0, (C0 + [0, -0.04, -0.04], 90.0))], 0.0),
    ("over-tilted to 80 degrees while rolling", INSERT + [(1, Y_IN, th, "load", H) for th in ramp(0, 80)] + [(3, Y_IN, 80.0, "load", H)] + ROLLING
     + STAND + SLIDE + LETGO, 0.0),
    ("loaded truck lifted 3 cm and carried", INSERT + TILT + [(1, y, 45.0, "load", H, 0.03) for y in
                                                             ramp(Y_IN, Y_END, 20)] + STAND + SLIDE + LETGO, 0.0),
    ("loaded truck held 1 cm off the floor for the whole roll (carried)", INSERT + TILT
     + [(1, y, 45.0, "load", H, 0.01) for y in ramp(Y_IN, Y_END, 20)] + STAND + SLIDE + LETGO, 0.0),
    ("wheels bump 5 mm off the floor for 2 steps while rolling", INSERT + TILT + ROLLING[:10]
     + [(1, Y_IN + ROLL / 2, 45.0, "load", H, 0.005)] * 2 + ROLLING[10:] + STAND + SLIDE + LETGO, 1.0),
    ("carton dropped from the truck onto the spot", INSERT + TILT + ROLLING
     + [(1, Y_END, 45.0, (np.array([X, m.SPOT[1], C0[2] + 0.12 - 0.04 * k]), 0.0), H) for k in range(1, 4)]
     + [(40, Y_END + 0.15, 0.0, END_C)], 0.0),
    ("on the spot, leaning 10 degrees, let go", INSERT + TILT + ROLLING + STAND + SLIDE
     + [(40, Y_OUT, 0.0, (END_C[0] + [0, 0, 0.005], 10.0))], 0.5),
    ("on the spot but 5 cm off center (tape crossed)", INSERT + TILT
     + [(1, y, 45.0, "load", H) for y in ramp(Y_IN, Y_END + 0.05, 20)]
     + [(1, Y_END + 0.05, th, "load", H) for th in ramp(45.0, 0.0)]
     + [(40, Y_OUT + 0.05, 0.0, (END_C[0] + [0, 0.05, 0], 0.0))], 0.5),
    ("set down 10 cm short of the spot", INSERT + TILT
     + [(1, y, 45.0, "load", H) for y in ramp(Y_IN, Y_END + 0.10, 20)]
     + [(1, Y_END + 0.10, th, "load", H) for th in ramp(45.0, 0.0)]
     + [(40, Y_OUT + 0.10, 0.0, (END_C[0] + [0, 0.10, 0], 0.0))], 0.25),
    ("truck left lying down after the job", INSERT + TILT + ROLLING + STAND + SLIDE + [(40, Y_OUT, 85.0, END_C)], 0.5),
    ("pushed the set-down carton 4 cm across the spot after sliding out", GOOD[:-1]
     + [(1, Y_OUT, 0.0, (END_C[0] + [0, -0.01 * k, 0], 0.0)) for k in range(1, 5)] + [(40, Y_OUT, 0.0, (END_C[0] + [0, -0.04, 0], 0.0))],
     0.0),
    ("carton still moving at the end", GOOD[:-1]
     + [(1, Y_OUT, 0.0, (END_C[0] + [0, 0.002 * (k % 2), 0], 0.0)) for k in range(40)], 0.5),
    ("load slid off the nose plate's tip while rolling (5 cm up), landed on the spot", INSERT + TILT + ROLLING[:15]
     + [(1, Y_IN + ROLL * 0.75, 45.0, (np.array([X, m.SPOT[1] + 0.02 - 0.01 * k, C0[2] + 0.05 - 0.01 * k]), 0.0), H)
        for k in range(5)]
     + [(1, Y_END, th, END_C, H) for th in ramp(45.0, 0.0)] + SLIDE + LETGO, 0.0),
    ("load slid 3 cm down the plate off the frame while tilted", INSERT + TILT + ROLLING[:10]
     + [(5, Y_IN + ROLL / 2, 45.0, (np.array(truck_pose(Y_IN + ROLL / 2, 45.0)[0]) + m._rot(quat(45.0, m.TRUCK_YAW))
                                     @ (SEAT + [0.03, 0, 0]), 45.0), H)] + ROLLING[10:] + STAND + SLIDE + LETGO, 0.0),
    ("truck left tilted 15 degrees after sliding out, let go", INSERT + TILT + ROLLING + STAND + SLIDE
     + [(40, Y_OUT, 15.0, END_C)], 0.5),
    ("truck stood up with the plate under the carton, then released tilted 20 degrees", INSERT + TILT + ROLLING
     + [(1, Y_END, th, "load", H) for th in ramp(45.0, 20.0)] + [(40, Y_END, 20.0, "load")], 0.5),
    # Independent review cases (Nov 9):
    ("rolled at a shallow 12 degree tilt, wheels on the floor", tilt_roll(12), 1.0),
    ("rolled at 15 degrees", tilt_roll(15), 1.0),
    ("rolled at 18 degrees", tilt_roll(18), 1.0),
    ("truck held 1.4 cm in the air at 10 degrees and carried", tilt_roll(10, 0.014), 0.0),
    ("truck held 4 mm in the air at 10 degrees (hovered, not rolled)", tilt_roll(10, 0.004), 0.0),
    ("one-step 20 degree flick, then the upright truck held 1.4 cm up and carried", INSERT
     + [(1, Y_IN, 20.0, "load", H)] + [(1, y, 0.0, "load", H, 0.014) for y in ramp(Y_IN, Y_END, 20)]
     + [(3, Y_END, 0.0, END_C, H)] + SLIDE + LETGO, 0.0),
    ("upright truck scraped 2.5 cm at a time with one-step 20 degree flicks between", ratchet(0.025, (20,)), 0.0),
    ("upright truck scraped 2.8 cm at a time with 0-20-0 degree rocks between", ratchet(0.028, (7, 14, 20, 14, 7, 0)),
     0.0),
    ("carton turned 45 degrees on the spot, center 2.9 cm off (corners over the tape)", turned_off(45.0, 0.029), 0.5),
    ("carton turned 30 degrees on the spot, center 2.9 cm off", turned_off(30.0, 0.029), 0.5),
    ("carton turned 3 degrees on the spot, center 2.5 cm off (fits inside)", turned_off(3.0, 0.025), 1.0),
    ("carton turned 10 degrees, centered on the spot (fits inside)", GOOD[:-1]
     + [(40, Y_OUT, 0.0, (END_C[0], 0.0, 10.0))], 1.0),
    ("load bounces 1.3 cm up off the plate for 2 steps at 45 degrees", INSERT + TILT + ROLLING[:10]
     + [(2, Y_IN + ROLL / 2, 45.0, (on_truck(Y_IN + ROLL / 2, 45.0, [0, 0, 0.013]), 45.0), H)] + ROLLING[10:] + STAND
     + SLIDE + LETGO, 1.0),
    ("load rocks 1 cm off the frame, 13 degrees, for 2 steps at 35 degrees", INSERT
     + [(1, Y_IN, t, "load", H) for t in ramp(0, 35)] + [(2, Y_IN, 35.0, (on_truck(Y_IN, 35.0, [0.01, 0, 0]), 22.0), H)]
     + [(1, Y_IN, t, "load", H) for t in ramp(35, 45)] + ROLLING + STAND + SLIDE + LETGO, 1.0),
    ("off its spot for 1 step only (a glitch)", [(1, Y0, 0.0, (C0 + [0, 0.05, 0], 0.0))] + GOOD, 1.0),
]


def check():
    bad = 0
    for name, frames, want in cases:
        got, ok = run(frames)
        flag = "ok " if abs(got - want) < 1e-9 else "BAD"
        bad += flag == "BAD"
        print(f"{flag} {got:4.2f} (want {want:4.2f}) success={ok!s:5} {name}")
    # a new trial (scene reset) starts with a clean history: a violation in one trial does not carry over
    run([(5, Y0, 0.0, (C0 + [0, -0.05, 0], 0.0))])
    got, _ = run(GOOD)
    flag = "ok " if got == 1.0 else "BAD"
    bad += flag == "BAD"
    print(f"{flag} {got:4.2f} (want 1.00) after a failed trial, a reset trial done right")
    print("all pass" if not bad else f"{bad} FAILED")
    return bad


if __name__ == "__main__":
    raise SystemExit(1 if check() else 0)
