"""Offline checks of the vaccine-fridge scorer, layout and demo, no simulator. Each control step is built by hand: the
cartons' poses as the simulator reports them, the fridge's door angle, the hands where given.

Covers: the start scores 0; the full job scores 1.0 and ends the trial; a door left open, a carton on the top shelf
(inside the ceiling margin), a carton stacked on another, leaning on a wall, lying on the floor or under a shelf, propped
on a rail, too near the door plane or the back wall, tilted, moving, or with a hand on it, does not count; a carton
dropped or thrown onto a shelf counts for nothing for the rest of the trial even once it rests in a good place; a
door shut on an empty fridge scores 0; a sweep of carton poses on every shelf against the 5 cm rule computed
independently; a new trial starts afresh; the demo's set-down poses comply; no pxr or Isaac module is imported by
loading the task or by demo(); the thresholds in task.json and the README match the code.

    python3 test_task.py
"""
import json
import math
import random
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import guardkit as gk  # noqa: E402,F401
import yamkit as kit  # noqa: E402

AWAY = {"left_gripper": (0.0, 0.45, M.TOP + 0.30), "right_gripper": (0.0, -0.45, M.TOP + 0.30)}


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def quat(R):
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        return np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    v = np.zeros(4)
    v[i] = math.sqrt(max(0.0, 1.0 + R[i, i] - R[j, j] - R[k, k])) / 2
    v[j] = (R[j, i] + R[i, j]) / (4 * v[i])
    v[k] = (R[k, i] + R[i, k]) / (4 * v[i])
    v[3] = (R[k, j] - R[j, k]) / (4 * v[i])
    return v


def rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


STAND = ry(math.pi / 2)
from yam_arena.demo import grip_rotation as _grip  # noqa: E402

LEVEL_R = kit._rot_xyz(0.0, -math.pi / 2) @ _grip(0.0)  # the carton's long axis vertical (standing on end)


def st(pos, R=np.eye(3), half=0.03, vel=(0.0, 0.0, 0.0), spin=(0.0, 0.0, 0.0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=quat(R), lin_vel=np.asarray(vel, float), ang_vel=np.asarray(spin, float),
                       low=p - half, high=p + half)


def world(u, v, w):
    return M.ORIGIN + np.array([u, v, w])


def on_shelf(shelf, u, v, R=STAND):
    """A carton turned by R whose lowest corner rests on shelf `shelf`'s wires, its middle at (u, v)."""
    low = float((np.abs(R) @ (np.array(M.CARTON) / 2))[2])
    return world(u, v, M.SHELF_W[shelf] + low)


TOTE = {n: (np.array([xy[0], xy[1], M.TOP + M.TOTE_T + M.CARTON[2] / 2]), np.eye(3))
        for n, xy in zip(M.CARTONS, M.CARTON_START)}
GOOD = {"carton_a": (on_shelf(0, 0.08, 0.07), STAND), "carton_b": (on_shelf(0, 0.08, -0.03), STAND),
        "carton_c": (on_shelf(1, 0.08, 0.03), STAND)}


def frame(cartons, door_deg=90.0, hands=AWAY, vel=None, grip=0.42, hand_R=None):
    kit._JOINTS["fridge/door"] = kit.JointState(pos=math.radians(door_deg), vel=0.0, latched=False)
    objs = {"fridge": st(M.FRIDGE_POS, half=0.3), "tote": st(M.TOTE_POS)}
    for n, (p, R) in cartons.items():
        v = (vel or {}).get(n, ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0)))
        objs[n] = st(p, R, vel=v[0], spin=v[1])
    for g, p in hands.items():
        objs[g] = st(p, M.rotm((1.0, 0.0, 0.0, 0.0)) if hands[g] == AWAY[g] else
                     (LEVEL_R if hand_R is None else hand_R), 0.01)  # away: straight
        # down; on a carton: level, fingers forward, as the hand holds a carton in the fridge
        kit._OPENING[g] = grip if g == "left_gripper" and hands[g] != AWAY[g] else 1.0  # a hand on a carton: closed on it
    objs["_open"] = dict(kit._OPENING)  # this frame's finger openings (run() restores them step by step)
    return objs


def run(frames):
    """Fresh trial; score each frame (one control step each); the last score and success."""
    M._L = M.Ledger()
    s = ok = None
    for f in frames:
        kit._OPENING.update(f.get("_open", {}))
        s, ok = M.score(f), M.success(f)
    return s, ok


def hold(cartons, door_deg=0.0, n=30, hands=AWAY):
    return [frame(cartons, door_deg, hands) for _ in range(n)]


def with_(**kw):
    out = dict(GOOD)
    out.update(kw)
    return out


def main():
    # -- the layout and its published sizes
    check((M.INT_D, M.INT_W, M.INT_H) == (0.20, 0.36, 0.40), "interior 20 x 36 x 40 cm (8.0 x 14.0 x 15.75 in)")
    check(M.CARTON == (0.086, 0.0395, 0.0445), "carton 8.6 x 3.95 x 4.45 cm")
    check(M.SHELF_W[2] + M.CARTON[2] > M.INT_H - M.GAP + 0.01, "even a flat carton on the top shelf breaks the ceiling margin by over 1 cm")
    for i in range(2):
        check(M.SHELF_W[i] + M.CARTON[0] < M.INT_H - M.GAP - 0.01, f"a standing carton on shelf {i} clears the ceiling margin by over 1 cm")
    check(M.SHELF_W[0] >= M.GAP, "the bottom shelf's wires are at least 5 cm off the floor")
    check(all(M.SHELF_W[i + 1] - M.SHELF_W[i] - M.WIRE_D - M.RAIL_D > M.CARTON[0] + 0.02 for i in range(2)),
          "a standing carton plus 2 cm fits under the shelf above (bottom and middle shelves)")

    # -- the start, and doing nothing
    check(M.DOOR_START == 0.0, "the door starts shut")
    s, ok = run(hold(TOTE, 0.0))
    check(s == 0.0 and not ok, f"start state (cartons in the tote, door shut) scores 0 ({s})")
    s, ok = run(hold(TOTE, 0.0, n=5) + hold(TOTE, M.OPEN_DEG, n=20) + hold(TOTE, 0.0))
    check(s == 0.0 and not ok, f"door opened and shut again, nothing stored: 0 ({s})")

    # -- the order: open, store, shut (the cartons stand in their places from the start here: offline frames cannot
    # carry them in; the simulator's door is what keeps them out until it is opened)
    seq = hold(GOOD, 0.0, n=5) + hold(GOOD, M.OPEN_DEG, n=20)
    s, ok = run(seq + hold(GOOD, 0.0))
    check(s == 1.0 and ok, f"opened, three stored, shut: 1.0 ({s})")
    s, ok = run(seq + hold(GOOD, M.OPEN_DEG))
    check(s == 0.5 and not ok and abs(M._L.door_max - math.radians(M.OPEN_DEG)) < 1e-9,
          f"stored, then the door left open: capped at 0.5; the widest opening on record ({s})")
    s, ok = run(seq + hold(GOOD, 25.0))
    check(s == 0.5 and not ok, f"stored, door pushed only part way (25 degrees): 0.5 ({s})")
    s, ok = run(hold(GOOD, 0.0, n=M.HOLD_STEPS - 2) + hold(GOOD, M.OPEN_DEG, n=10))
    check(s == 0.5 and not ok, f"shut, then opened again before the 1 s hold: 0.5 ({s})")
    s, _ = run(hold(with_(carton_c=TOTE["carton_c"], carton_b=TOTE["carton_b"]), M.OPEN_DEG))
    check(s == 0.25, f"one stored, door open: 0.25 (under the cap) ({s})")

    # -- shut means shut, with no arm on the door: a hand holding it, a fingertip in its edge gap
    bar0 = tuple(M.bar_point(0.0))
    s, ok = run(hold(GOOD, 0.4, hands=dict(AWAY, left_gripper=bar0)))
    check(s == 0.75 and not ok, f"a hand on the handle holds the door at 0.4 degrees: 0.75, no success ({s})")
    o_d, R_d = M.door_frame(frame(GOOD, 0.3))
    edge = o_d + R_d @ np.array([M.DOOR_T / 2, M._DW + 0.01, 0.30])
    s, ok = run(hold(GOOD, 0.3, hands=dict(AWAY, right_gripper=tuple(edge))))
    check(s == 0.75 and not ok, f"a fingertip left in the door's free-edge gap: 0.75, no success ({s})")
    s, ok = run(hold(GOOD, 0.0, hands=dict(AWAY, left_gripper=tuple(np.array(bar0) - np.array([0.06, 0, 0])))))
    check(s == 1.0 and ok, f"a hand 6 cm in front of the handle, off it: 1.0 ({s})")
    s, ok = run(hold(TOTE, 0.0))
    check(s == 0.0 and not ok, f"door shut on an empty fridge scores 0 ({s})")

    # -- the full job
    s, ok = run(hold(GOOD, 0.0))
    check(s == 1.0 and ok, f"three cartons stored, door shut, held 1 s: 1.0 and success ({s})")
    s, ok = run(hold(GOOD, 0.0, n=M.HOLD_STEPS - 1))
    check(s == 0.75 and not ok, f"... held less than 1 s: 0.75, not yet success ({s})")
    s, ok = run(hold(GOOD, 90.0))
    check(s == 0.5 and not ok, f"door left open: 0.5, no success ({s})")
    s, ok = run(hold(GOOD, 0.8))
    check(s == 0.5 and not ok, f"door 0.8 degrees ajar (6 mm at its free edge): not shut, 0.5 ({s})")
    s, _ = run(hold(GOOD, 0.4))
    check(s == 1.0, f"door 0.4 degrees (within 0.5): shut ({s})")
    s, _ = run(hold(with_(carton_c=TOTE["carton_c"]), 0.0))
    check(s == 0.5, f"two stored, one left in the tote, door shut: 0.5 ({s})")

    # -- every side bounded
    bad = {
        "top shelf, standing (inside the ceiling margin)": (on_shelf(2, 0.08, 0.0), STAND),
        "top shelf, lying flat (inside the ceiling margin)": (on_shelf(2, 0.10, 0.0, np.eye(3)), np.eye(3)),
        "stacked on carton b": (on_shelf(0, 0.08, -0.03) + np.array([0, 0, M.CARTON[0]]), STAND),
        "on the interior floor": (world(0.10, 0.0, M.CARTON[0] / 2), STAND),
        "under the bottom shelf, lying": (world(0.10, 0.0, M.CARTON[2] / 2), np.eye(3)),
        "propped 1 cm up on a rail": (on_shelf(0, 0.08, 0.07) + np.array([0, 0, 0.01]), STAND),
        "1 cm into the shelf (below the wires)": (on_shelf(0, 0.08, 0.07) - np.array([0, 0, 0.01]), STAND),
        "leaning 15 deg on the left wall": (on_shelf(0, 0.08, 0.16, rx(math.radians(15)) @ STAND), rx(math.radians(15)) @ STAND),
        "4.5 cm from the door plane": (on_shelf(0, 0.045 + M.CARTON[2] / 2, 0.07), STAND),
        "4.5 cm from the back wall": (on_shelf(0, M.INT_D - 0.045 - M.CARTON[2] / 2, 0.07), STAND),
        "4.5 cm from the left wall": (on_shelf(0, 0.08, M.INT_W / 2 - 0.045 - M.CARTON[1] / 2), STAND),
        "4.5 cm from the right wall": (on_shelf(0, 0.08, -M.INT_W / 2 + 0.045 + M.CARTON[1] / 2), STAND),
        "outside, in front of the opening": (on_shelf(0, -0.06, 0.07), STAND),
        "on the cabinet top": (world(-0.10 + 0.205, 0.0, M.INT_H + 0.1 + M.CARTON[0] / 2), STAND),
        "tilted 12 deg on the shelf": (on_shelf(0, 0.08, 0.07, rx(math.radians(12)) @ STAND), rx(math.radians(12)) @ STAND),
    }
    for what, pose in bad.items():
        s, _ = run(hold(with_(carton_a=pose), 0.0))
        check(s == 0.5, f"carton a {what}: not counted ({s})")
    for what, pose in {"lying flat on the middle shelf": (on_shelf(1, 0.10, 0.0, np.eye(3)), np.eye(3)),
                       "on its side on the bottom shelf": (on_shelf(0, 0.10, 0.07, rx(math.pi / 2)), rx(math.pi / 2)),
                       "standing, turned 30 deg about the vertical": (on_shelf(0, 0.09, 0.07, rz(0.5) @ STAND), rz(0.5) @ STAND),
                       "exactly 5.05 cm from the door plane": (on_shelf(0, 0.0505 + M.CARTON[2] / 2, 0.07), STAND)}.items():
        s, _ = run(hold(with_(carton_a=pose), 0.0))
        check(s == 1.0, f"carton a {what}: counted ({s})")

    # -- motion and hands
    a = GOOD["carton_a"]
    frames = [frame(with_(carton_a=(a[0] + np.array([0, 0, 0.001 * (k % 2)]), a[1])), 0.0) for k in range(40)]
    s, ok = run(frames)
    check(s == 0.5 and not ok, f"carton a jiggling 1 mm a step: not still, not counted ({s})")
    hands = dict(AWAY, left_gripper=tuple(a[0]))
    s, _ = run(hold(GOOD, 0.0, hands=hands))
    check(s == 0.5, f"a hand holding carton a: not counted ({s})")
    hands = dict(AWAY, left_gripper=tuple(a[0] + np.array([-0.30, 0.0, 0.0])))
    s, _ = run(hold(GOOD, 0.0, hands=hands))
    check(s == 1.0, f"hand 30 cm away: counted ({s})")

    # -- dropped: held by a hand over its place, let go, falls (sampled at 20 Hz, the sim's velocities), lands
    hand = lambda p: dict(AWAY, left_gripper=tuple(p))  # noqa: E731

    def fall_frames(drop, touching_steps=0):
        """Held `drop` over its place for 5 steps, then a free fall; the hand still reads on for `touching_steps`."""
        top = a[0] + np.array([0, 0, drop])
        out = [frame(with_(carton_a=(top, a[1])), 90.0, hands=hand(top)) for _ in range(5)]
        t, k = 0.0, 0
        while True:
            t += M.DT
            z = drop - 0.5 * 9.81 * t * t
            if z <= 0:
                break
            p = a[0] + np.array([0, 0, z])
            hands = hand(top) if k < touching_steps else AWAY  # the hand stays; the carton slides out
            out.append(frame(with_(carton_a=(p, a[1])), 90.0, hands=hands, vel={"carton_a": ((0, 0, -9.81 * t), (0, 0, 0))}))
            k += 1
        return out + hold(GOOD, 0.0, n=40)

    for drop, touch, want in ((0.002, 0, 1.0), (0.008, 0, 1.0), (0.015, 0, 0.5), (0.05, 0, 0.5), (0.03, 2, 0.5),
                              (0.02, 1, 0.5)):
        s, _ = run(fall_frames(drop, touch))
        check(s == want, f"carton a let go {drop * 1000:.0f} mm over its place (hand on for {touch} steps of the fall): "
                         f"{want} ({s}, {M._L.dropped})")
    # slid out of loose fingers: the hand stays where it was and lingers 0-2 steps after the carton lands
    for drop in (0.03, 0.05, 0.08):
        for linger in (0, 1, 2):
            top = a[0] + np.array([0, 0, drop])
            fr = [frame(with_(carton_a=(top, a[1])), 90.0, hands=hand(top)) for _ in range(5)]
            t = 0.0
            while drop - 0.5 * 9.81 * t * t > 0:
                t += M.DT
                z = max(0.0, drop - 0.5 * 9.81 * t * t)
                fr.append(frame(with_(carton_a=(a[0] + np.array([0, 0, z]), a[1])), 90.0, hands=hand(top),
                                vel={"carton_a": ((0, 0, -9.81 * t if z > 0 else 0.0), (0, 0, 0))}))
            fr += [frame(GOOD, 90.0, hands=hand(top)) for _ in range(linger)] + hold(GOOD, 0.0, n=30)
            s, _ = run(fr)
            check(s == 0.5 and "carton_a" in M._L.dropped,
                  f"slid {drop * 100:.0f} cm out of a still hand, hand lingering {linger} steps: dropped ({s})")
    # let go late in a control step (the carton has barely moved at the first hands-off step), 2 cm up: dropped
    top = a[0] + np.array([0, 0, 0.02])
    fr = [frame(with_(carton_a=(top, a[1])), 0.0, hands=hand(top)) for _ in range(5)]
    t = 0.0025
    while 0.02 - 0.5 * 9.81 * t * t > 0:
        fr.append(frame(with_(carton_a=(a[0] + np.array([0, 0, 0.02 - 0.5 * 9.81 * t * t]), a[1])), 0.0))
        t += M.DT
    s, _ = run(fr + hold(GOOD, 0.0, n=30))
    check(s == 0.5 and "carton_a" in M._L.dropped, f"let go 2 cm up late in a control step: dropped ({s})")
    # turned on end in a still hand (its lowest corner drops 2 cm, its middle stays), then lowered by hand: not a drop
    flat_p = a[0] + np.array([0, 0, 0.10])
    fr = [frame(with_(carton_a=(flat_p, np.eye(3))), 90.0, hands=hand(flat_p)) for _ in range(3)]
    fr += [frame(with_(carton_a=(flat_p, ry(math.pi / 2 * k / 6))), 90.0, hands=hand(flat_p)) for k in range(7)]
    fr += [frame(with_(carton_a=(flat_p + (a[0] - flat_p) * k / 10, STAND)), 90.0,
                 hands=hand(flat_p + (a[0] - flat_p) * k / 10)) for k in range(11)]
    s, _ = run(fr + hold(GOOD, 0.0, n=30))
    check(s == 1.0 and not M._L.dropped, f"turned on end in a still hand, then lowered by hand: counted ({s})")
    # gripped 3 cm off its middle, the hand turns 90 degrees in place in 3 steps (0.15 s), then lowers it: not a drop
    tcp0 = a[0] + np.array([0, 0, 0.10])
    fr = []
    for k in range(4):
        Rk = ry(math.pi / 2 * k / 3)  # turns the 3 cm offset from level to straight down
        fr.append(frame(with_(carton_a=(tcp0 + Rk @ np.array([0.03, 0, 0]), a[1])), 0.0, hands=hand(tcp0),
                        hand_R=Rk @ LEVEL_R))
    mid0 = tcp0 + ry(math.pi / 2) @ np.array([0.03, 0, 0])
    for k in range(1, 21):
        p = mid0 + (a[0] + np.array([0, 0, 0.002]) - mid0) * k / 20
        fr.append(frame(with_(carton_a=(p, a[1])), 0.0, hands=hand(p - (mid0 - tcp0)), hand_R=ry(math.pi / 2) @ LEVEL_R))
    s, _ = run(fr + hold(GOOD, 0.0, n=30))
    check(s == 1.0 and not M._L.dropped, f"off-centre grip, fast 90-degree turn in the hand, lowered: counted ({s}, {M._L.dropped})")
    # set down on the cabinet top, then carried to its place: not a drop
    roof = world(0.10, 0.0, M.INT_H + 0.093 + M.CARTON[0] / 2)
    frames = [frame(with_(carton_a=(roof + np.array([0, 0, 0.002]), STAND)), 90.0, hands=hand(roof)) for _ in range(4)]
    frames += hold(with_(carton_a=(roof, STAND)), 90.0, n=5)
    frames += [frame(with_(carton_a=(roof, STAND)), 90.0, hands=hand(roof)) for _ in range(3)]
    frames += [frame(with_(carton_a=(roof + (a[0] - roof) * k / 20, STAND)), 90.0, hands=hand(roof + (a[0] - roof) * k / 20))
               for k in range(21)]
    frames += hold(GOOD, 0.0, n=30)
    s, _ = run(frames)
    check(s == 1.0 and not M._L.dropped, f"carton set on the cabinet top, then carried to its place: counted ({s})")
    frames = hold(with_(carton_a=TOTE["carton_a"]), 90.0, n=5) + hold(GOOD, 0.0, n=40)  # teleported (thrown) in
    s, _ = run(frames)
    check(s == 0.5, f"carton a arriving 30 cm in one step with no hand (thrown): nothing ({s})")
    # set down by hand: arrives while a hand is on it, released at rest
    frames = [frame(with_(carton_a=(a[0] + np.array([0, 0, 0.002 * (5 - k)]), a[1])), 0.0,
                    hands=dict(AWAY, left_gripper=tuple(a[0]))) for k in range(6)] + hold(GOOD, 0.0, n=30)
    s, ok = run(frames)
    check(s == 1.0 and ok and not M._L.dropped, f"set down by hand, then let go: counted ({s})")

    # -- knocked by the door: a carton left sticking out of the opening is shoved 3 cm in one step by the closing door
    out = (on_shelf(0, 0.01, 0.07), STAND)
    frames = hold(with_(carton_a=out), 20.0, n=5) + [frame(with_(carton_a=(out[0] + np.array([0.03, 0, 0]), STAND)), 5.0)] \
        + hold(with_(carton_a=(out[0] + np.array([0.06, 0, 0]), STAND)), 0.0, n=30)
    s, _ = run(frames)
    check(s == 0.5 and "carton_a" in M._L.dropped, f"carton knocked in by the closing door: counts for nothing ({s})")

    # -- the self-closing hinge's law: pulls shut within 30 degrees, nothing past 35
    law = M.DOOR.law
    k30 = law.drive("fridge/door", math.radians(29), 0.0, {}, {})[1]
    k40 = law.drive("fridge/door", math.radians(40), 0.0, {}, {})[1]
    k0 = law.drive("fridge/door", 0.0, 0.0, {}, {})
    check(k30 == M.CLOSER_K and k40 == 0.0 and k0[1] * (0.0 - k0[0]) > 0.2,
          "closer: full spring within 30 deg, none past 35, still pressing shut at 0")

    # -- the magnetic catch: its rated pull at contact (4.4 lb at the counter plate), nothing once the gap passes 3 mm
    t0 = M.catch_torque(0.0)
    check(abs(t0 - 4.4 * 4.448 * M.CATCH_R) < 1e-9 and abs(k0[2] + t0) < 1e-9,
          f"catch holds the shut door with its rated pull ({t0:.2f} N*m, an effort toward shut)")
    q3 = M.CATCH_GAP / M.CATCH_R
    check(M.catch_torque(q3) == 0.0 and M.catch_torque(q3 / 2) < t0 and law.drive("fridge/door", math.radians(5), 0.0,
          {}, {})[2] == 0.0, "catch: pull fades to nothing by a 3 mm gap; none at 5 degrees")
    r_bar = M._DW - M.BAR_IN
    need = (t0 + k0[1] * (0.0 - k0[0])) / r_bar
    check(15.0 < need < 25.0, f"pull needed at the bar to open the shut door: {need:.1f} N (catch plus closer)")
    # the score reads only where the cartons and the door end up: how the door was opened (bar, edge, slab) is not
    # scored, and nothing gets past the shut door's slab
    src = (HERE / "task.py").read_text()
    body = src[src.index("class Ledger"):src.index("_L = Ledger()")]
    check(not any(w in body for w in ("catch", "BAR_", "_HANDLE", "hook")),
          "the scorer does not read the catch or the handle")

    # -- the simulator's own velocities: a 1 cm free fall (0.44 m/s at the shelf) latches; a 4 mm set-down does not
    frames = hold(GOOD, 90.0, n=5) + [frame(GOOD, 90.0, vel={"carton_a": ((0, 0, -0.44), (0, 0, 0))})] + hold(GOOD, 0.0, n=30)
    s, _ = run(frames)
    check(s == 0.5 and "carton_a" in M._L.dropped, f"carton a landing at 0.44 m/s (a 1 cm fall), no hand: nothing ({s})")
    frames = hold(GOOD, 90.0, n=5) + [frame(GOOD, 90.0, vel={"carton_a": ((0, 0, -0.28), (0, 0, 0))})] + hold(GOOD, 0.0, n=30)
    s, _ = run(frames)
    check(s == 1.0, f"carton a settling at 0.28 m/s (a 4 mm drop off the fingers): counted ({s})")
    frames = hold(GOOD, 90.0, n=5) + [frame(GOOD, 90.0, vel={"carton_a": ((0, 0, 0), (10.0, 0, 0))})] + hold(GOOD, 0.0, n=30)
    s, _ = run(frames)
    check(s == 0.5, f"carton a tipping (corner over 0.4 m/s from spin alone), no hand: nothing ({s})")

    # -- hands-off in the simulator: link poses from body quaternions in Isaac Lab's (x, y, z, w) order
    from yam_arena import ik
    from yam_arena.analysis import fk_link_poses_world
    from yam_arena.constants import ARM_JOINTS
    from yam_arena.demo import LEFT_BASE_POS
    tcp = M.place_tcp(0)
    Rh = kit._rot_xyz(0.0, M.LEVEL) @ __import__("yam_arena.demo", fromlist=["x"]).grip_rotation(0.0)
    q, pe, _ = ik.solve(tcp, ik.matrix_to_quat_xyzw(Rh), base_pos=LEFT_BASE_POS, iters=400)
    qd = dict(zip(ARM_JOINTS, q))
    qd["joint7"] = qd["joint8"] = -(M.CARTON[1] / 2) / M.FINGER_HALF_OPEN * abs(M.FINGER_OPEN) * 0.98
    P = fk_link_poses_world(qd, LEFT_BASE_POS)
    M.LINK_POSES.clear()
    for link in M.ARM_LINKS:
        M.LINK_POSES[f"left_robot/{link}"] = M.link_pose(P[link][:3, 3], quat(P[link][:3, :3]))  # (x, y, z, w)
    held = frame(with_(carton_a=(tcp, STAND)), 90.0)
    g = M.arm_gap(held, "carton_a")
    check(pe < 0.002 and g <= M.CLEAR, f"a carton held at its place reads as touched by the hand's hulls (gap {g * 1000:.1f} mm)")
    away = frame(GOOD, 90.0)
    check(M.arm_gap(away, "carton_b") > M.CLEAR, "... and the carton beside it reads as clear")
    M.LINK_POSES.clear()

    # -- each demo place stands on two wires with its middle between them
    for i, (shelf, v) in enumerate(M.PLACES):
        under = [w for w in M.wire_vs() if abs(w - v) < M.CARTON[1] / 2 - M.WIRE_D / 2]
        check(len(under) >= 2 and min(under) < v < max(under), f"demo place {i}: wires under it at {[round(w, 4) for w in under]}")

    # -- a new trial starts afresh
    kit._EPISODE["count"] += 1
    s = M.score(frame(TOTE, 90.0))
    check(s == 0.0 and not M._L.dropped, "a new trial starts afresh")

    # -- sweep against an independent 5 cm rule
    rng = random.Random(7)
    n_ok = n_bad = 0
    for _ in range(1500):
        shelf = rng.randrange(3)
        R = [STAND, np.eye(3), rx(math.pi / 2)][rng.randrange(3)] @ np.eye(3)
        R = rz(rng.uniform(-math.pi, math.pi)) @ R
        u, v = rng.uniform(0.03, M.INT_D - 0.03), rng.uniform(-0.15, 0.15)
        p = on_shelf(shelf, u, v, R)
        corners = p + M._CORNERS @ R.T - M.ORIGIN
        inside = corners[:, 0].min() > 0 and corners[:, 0].max() < M.INT_D and np.abs(corners[:, 1]).max() < M.INT_W / 2
        want = (inside and M.SHELF_U[0] <= u <= M.SHELF_U[1] and corners[:, 0].min() >= 0.05
                and corners[:, 0].max() <= M.INT_D - 0.05 and np.abs(corners[:, 1]).max() <= M.INT_W / 2 - 0.05
                and corners[:, 2].max() <= M.INT_H - 0.05 and corners[:, 2].min() >= 0.05)
        objs = frame(with_(carton_a=(p, R)), 0.0)
        got = M.compliant(objs, "carton_a")
        if got != want:
            check(False, f"sweep: shelf {shelf} u {u:.3f} v {v:.3f}: scorer {got} vs rule {want}")
        n_ok += want
        n_bad += not want
    check(n_ok > 50 and n_bad > 50, f"sweep of 1500 poses agrees with the 5 cm rule ({n_ok} in, {n_bad} out)")

    # -- the demo's set-down poses comply
    for i in range(3):
        tcp = M.place_tcp(i)
        objs = frame(with_(**{M.CARTONS[i]: (tcp - np.array([0, 0, 0.002]), STAND)}), 0.0)
        check(M.compliant(objs, M.CARTONS[i]), f"demo place {i}: compliant, gaps "
              f"{ {k: round(v * 100, 1) for k, v in M.clearances(objs, M.CARTONS[i]).items()} } cm")

    # -- the door's whole swing (slab and handle, 0 to 90 degrees) clears the tote by at least 1 cm (plan view)
    hx, hy = M.FRIDGE_POS[0] + M.HINGE[0], M.FRIDGE_POS[1] + M.HINGE[1]
    lo = np.array([M.TOTE_POS[0] - M.TOTE_OUT[0] / 2, M.TOTE_POS[1] - M.TOTE_OUT[1] / 2])
    hi = np.array([M.TOTE_POS[0] + M.TOTE_OUT[0] / 2, M.TOTE_POS[1] + M.TOTE_OUT[1] / 2])
    gap = math.inf
    for deg in np.linspace(0, 90, 91):
        th = math.radians(deg)
        Rz = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        for c, sz in ((M._DOOR_BOX, M._DOOR_SIZE), M._HANDLE):
            for sx in np.linspace(-0.5, 0.5, 9):
                for sy in np.linspace(-0.5, 0.5, 41):
                    w = np.array([hx, hy]) + Rz @ np.array([c[0] + sx * sz[0], c[1] + sy * sz[1]])
                    gap = min(gap, float(np.linalg.norm(np.maximum(np.maximum(lo - w, w - hi), 0.0))))
    check(gap >= 0.01, f"the door's swing clears the tote ({gap * 1000:.1f} mm)")

    # -- the shelf wires are convex-hull colliders in the fridge's USD (written for a mechanism's base too)
    usda = kit._mech_usda(M.FRIDGE)
    n_wires = 3 * len(M.wire_vs())
    check(usda.count('physics:approximation = "convexHull"') == n_wires + 1 and "radius = 0.0" not in usda,
          f"{n_wires} wire hulls and the handle's bar hull in the fridge's USD, no empty colliders")

    # -- the handle: a round bar on two posts, a finger gap behind it, 4.4 cm proud overall
    bar = np.asarray(M._BAR_HULL) + np.asarray(M._BAR_C)
    check(abs(bar[:, 0].min() + M.HANDLE_OUT) < 1e-3 and bar[:, 0].max() < -0.018,
          f"bar 4.4 cm proud, {(-bar[:, 0].max()) * 1000:.0f} mm gap behind it")
    check(all(abs(c[0] + sz[0] / 2) < 1e-9 for c, sz in M._POSTS), "the posts run from the door's face to the bar")
    check(abs(bar[:, 2].max() - bar[:, 2].min() - M.BAR_L) < 1e-6, "the bar spans post to post")
    # the demo's grip point is the bar's centre line, wherever the door stands
    hx, hy = M.FRIDGE_POS[0] + M.HINGE[0], M.FRIDGE_POS[1] + M.HINGE[1]
    for deg in (0, 20, M.OPEN_DEG):
        th = math.radians(deg)
        Rz2 = np.array([[math.cos(th), -math.sin(th)], [math.sin(th), math.cos(th)]])
        want = np.array([hx, hy]) + Rz2 @ np.array(M._BAR_C[:2])
        check(np.linalg.norm(M.bar_point(th)[:2] - want) < 1e-9, f"grip point on the bar at {deg} degrees")
    check(M.DOOR.law.drive("fridge/door", math.radians(M.OPEN_DEG), 0.0, {}, {})[1] == 0.0,
          f"pulled open to {M.OPEN_DEG} degrees, past the closer's reach: the door stays")

    # -- no pxr / Isaac at load or in demo()
    M.demo()
    loaded = [m for m in sys.modules if m == "pxr" or m.startswith("pxr.") or m.startswith("isaaclab") or m.startswith("omni")]
    check(not loaded, f"no pxr/Isaac module imported by loading the task or demo() ({loaded[:3]})")

    # -- thresholds in the text
    tj = json.loads((HERE / "task.json").read_text()) if (HERE / "task.json").exists() else None
    if tj:
        text = tj["scoring"] + tj["setup"] + (HERE / "README.md").read_text()
        for needle in ("5 cm", "0.25", "1 s", "0.5 degrees", "1 cm", "at most 0.5"):
            check(needle in text, f"text states {needle!r}")
        check(len(tj["scoring"]) <= 4000 and len(tj["setup"]) <= 4000 and len(tj["instruction"]) <= 1000,
              "task.json fields within the bot's limits")
        check(tj["instruction"] == M.INSTRUCTION, "task.json instruction == task.py INSTRUCTION")
        check(not re.search(r"\$\s?\d", text), "no prices in the text")
    print("all passed")


if __name__ == "__main__":
    main()
