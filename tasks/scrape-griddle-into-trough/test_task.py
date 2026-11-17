"""Offline checks of the griddle scorer and demo, no simulator. The debris is given as the simulator gives it (each
piece's pose in objects["debris"]); scraper states are built from the same geometry the demo uses (task.scraper_rot,
task.tcp_for_edge), with the right gripper closed on the handle (fingers across world y, the real yaw-0 convention:
yam_arena.demo.grip_rotation) when the scraper is held. Covers the debris layout, every tier, both caps, where a piece
counts (tray, griddle, off it), the scraper left in hand, on the griddle, in the tray, dropped on the bench, set down
but moving, the 1 s hold, a new trial, the thresholds in the text, and a kinematic playback of the demo plan (the
pushing itself needs the simulator).

    python3 test_task.py
"""
import json
import math
import re
import sys
from pathlib import Path

import numpy as np
from yam_arena.demo import grip_rotation
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import scrapekit as sk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.constants import FINGER_OPEN  # noqa: E402

AWAY = {"left_gripper": (0.15, 0.30, M.TOP + 0.30), "right_gripper": (0.15, -0.30, M.TOP + 0.30)}
GRIP_OPEN = (M.HANDLE[1][1] / 2) / abs(FINGER_OPEN)  # pads on the 30 mm handle's sides


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=sk.quat(np.asarray(rot)), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def fresh():
    kit._EPISODE["count"] += 1
    M._L = M.Ledger()


FIXED = {b.name: st(b.pos) for b in M.OBJECTS if b.name != "scraper"}


DEBRIS = [M.START_DEBRIS]  # the debris the next states carry


def objs(spos, srot, held=True, pitch=0.0, debris=None):
    """Objects for one step: the scraper at spos/srot; the right hand closed on its handle if held, else both away."""
    o = dict(FIXED)
    o["debris"] = (DEBRIS[0] if debris is None else debris).copy()
    o["scraper"] = st(spos, srot)
    opening = {}
    for g, p in AWAY.items():
        o[g] = st(p, grip_rotation(0.0))
        opening[g] = 1.0
    if held:
        o["right_gripper"] = st(np.asarray(spos) + srot @ M.GRASP_LOCAL, ry(pitch) @ grip_rotation(M.GRIP_YAW))
        opening["right_gripper"] = GRIP_OPEN
    return o, opening


def feed(states):
    """Each state as the sim gives it: a fresh dict per step, the finger openings of that step."""
    s = 0.0
    for o, opening in states:
        kit._OPENING.update(opening)
        o = dict(o)
        M.success(o)
        s = M.score(o)
    return s


def at_edge(edge, rot):
    return np.asarray(edge, float) - rot @ M.EDGE_LOCAL


SINK = 0.002  # m: where a pressed edge actually sits (the collider stops it; the demo aims PRESS deeper)


def stroke(y, theta=M.SCRAPE_DEG, x0=M.X_START, x1=M.X_END, held=True, dz=-SINK, speed=M.STROKE_SPEED,
           lift_s=M.LIFT_S, skew=0.0, roll=0.0):
    """States for one stroke: settle, draw the edge from x0 to x1 at `speed`, lift LIFT (3 cm) in `lift_s`."""
    p = M.pitch_for(theta)
    r = M.scraper_rot(p)
    if roll:
        c, s = math.cos(roll), math.sin(roll)
        r = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) @ r
    out = []
    z = M.PLATE_Z + dz
    for _ in range(4):
        out.append(objs(at_edge((x0, y, z), r), r, held, p))
    n = max(2, round(abs(x0 - x1) / speed / M.DT))
    for k in range(1, n + 1):
        s_ = k / n
        out.append(objs(at_edge((x0 + s_ * (x1 - x0), y + s_ * skew, z), r), r, held, p))
    m = max(1, round(lift_s / M.DT))
    for k in range(1, m + 1):
        out.append(objs(at_edge((x1, y + skew, z + M.LIFT * k / m), r), r, held, p))
    for _ in range(5):
        out.append(objs(at_edge((x1, y + skew, z + M.LIFT), r), r, held, p))
    return out


def job(passes=M.PASSES, lanes=M.LANES_Y, **kw):
    out = []
    for _ in range(passes):
        for y in lanes:
            out += stroke(y, **kw)
    return out


REST_POS = np.asarray(M.SCRAPER_START)


def set_down(n=25, pos=REST_POS, rot=None, wobble=False):
    rot = M.R_REST if rot is None else rot
    out = []
    for k in range(n):
        p = np.asarray(pos, float) + (np.array([0.002 * (k % 2), 0, 0]) if wobble else 0.0)
        out.append(objs(p, rot, False))
    return out


def held_at_rest(n=25):
    return [objs(REST_POS, M.R_REST, True)] * n


def moved(frac_tray, frac_off=0.0):
    """The debris with pieces moved into the tray (in layout order, skipping any that would pass `frac_tray` of the
    mass), then others onto the bench up to `frac_off`; the rest where they lay. Mass shares land just under the
    fractions asked for (within one piece, 4.5 %)."""
    d = M.START_DEBRIS.copy()
    used = set()
    for frac, place in ((frac_tray, lambda i: (0.11, -0.10 + 0.002 * (i % 100), M.PLATE_Z - M.TRAY_DEPTH + 0.002)),
                        (frac_off, lambda i: (0.40, -0.26, M.TOP + 0.0015))):
        acc = 0.0
        for i in range(len(d)):
            if i not in used and acc + M.CHIP_MASS[i] <= frac * M.TOTAL_MASS + 1e-12:
                d[i, :3] = place(i)
                acc += M.CHIP_MASS[i]
                used.add(i)
    return d


def with_debris(d, states):
    DEBRIS[0] = d
    out = [(dict(o, debris=d.copy()), op) for o, op in states]
    DEBRIS[0] = M.START_DEBRIS
    return out

SPOT_POS = np.array([M.SPOT_XY[0], M.SPOT_XY[1], M.SCRAPER_START[2]])
P40 = M.pitch_for(M.SCRAPE_DEG)
R40 = M.scraper_rot(P40)


def tray_place(i):
    """A resting place on the tray's floor for piece i (pieces spread across the tray)."""
    return (0.090 + 0.035 * (i % 2), M.PLATE_Y0 + 0.015 + 0.013 * i, M.PLATE_Z - M.TRAY_DEPTH + M.PIECE_T / 2)


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def pusher_state(d, how, x_back, y, hand="right_gripper"):
    """One step with piece(s) in d and the pusher's front at x_back (the pushed piece's +x face), centered on y.
    how: "blade" (held scraper, edge on the plate at 40 degrees), "loose" (the same pose, nobody holding it),
    "handle" (held, turned round: the handle's end pushes), "hand" (a gripper's fingertips on the piece, the scraper
    lying where it started), "none" (nothing touches: the arm's body, a tipped griddle, a moved tray)."""
    if how in ("blade", "loose"):
        o, op = objs(at_edge((x_back + 0.0005, y, M.PLATE_Z - 0.001), R40), R40, how == "blade", P40, debris=d)
    elif how == "handle":
        r = rz(math.pi) @ M.R_REST
        org = np.array([x_back + 0.0005 + M.HANDLE[0][0] + M.HANDLE[1][0] / 2, y, M.PLATE_Z + 0.013])
        o, op = objs(org, r, True, 0.0, debris=d)
    else:
        o, op = objs(REST_POS, M.R_REST, False, debris=d)
        if how == "hand":
            o[hand] = st((x_back - M.PIECE_W / 2, y, M.PLATE_Z + M.PIECE_T + 0.006), grip_rotation(0.0))
            op = dict(op, **{hand: 0.0})
    return o, op


def scrape(ids, how="blade", d0=None, step=0.008):
    """Each piece in turn pushed along the plate by `how` from where it lies over the lip, then resting in the tray.
    Returns (states, the debris at the end)."""
    d = (M.START_DEBRIS if d0 is None else d0).copy()
    out = []
    for i in ids:
        x0, y = float(d[i, 0]), float(d[i, 1])
        xs = list(np.arange(x0, M.LIP_X - 0.012, -step)) + [M.LIP_X - 0.012]
        for x in xs:
            d[i, :3] = (x, y, M.PLATE_Z + M.PIECE_T / 2 + M.CHIP_LIFT)
            out.append(pusher_state(d.copy(), how, x + M.PIECE_W / 2, y))
        d[i, :3] = tray_place(i)
        out.append(pusher_state(d.copy(), how, M.LIP_X - 0.012 + M.PIECE_W / 2, y))
    return out, d


def run(states, d_end, down):
    return feed(states + with_debris(d_end, down))


ALL = list(range(22))
JOB, FULL = scrape(ALL)


def main():
    import itertools

    C = M.CHIPS
    check(len(C) == 22, f"{len(C)} debris pieces (rigid bodies)")
    check(not any(M.overlap2(a, b, 0.010) for a, b in itertools.combinations(C, 2)),
          "no two pieces touch at the start (at least 1 cm apart: separate pieces, each bonded to the plate only)")
    check(all(c.size == (0.019, 0.019, 0.0048) for c in C), "every piece 1.9 cm square and 4.8 mm thick (the template)")
    check(all(abs(c.pos[2] - c.size[2] / 2 - M.PLATE_Z - M.CHIP_LIFT) < 1e-9 for c in C), "every piece on the plate")
    check([sum(1 for c in C if c.pos[0] == x) for x in M.ROWS_X] == [6, 5, 6, 5], "rows of 6, 5, 6 and 5 pieces")
    check(abs(M.ROWS_X[0] - M.LIP_X - 0.025) < 1e-9 and all(abs(b - a - 0.035) < 1e-9 for a, b in
                                                            zip(M.ROWS_X, M.ROWS_X[1:])),
          "near row 2.5 cm from the lip, rows 3.5 cm apart")
    corners = [p for c in C for p in M._corners2(c.pos[0], c.pos[1], c.size[0], c.size[1], c.yaw)]
    check(all(M.LIP_X + 0.010 <= p[0] and M.PLATE_Y0 + 0.05 <= p[1] <= M.PLATE_Y0 + M.PLATE_W - 0.05 for p in corners),
          "every piece on the plate at least 1 cm behind the lip, 5 cm or more inside the side guards")
    reach = (min(M.LANES_Y) - M.BLADE_W / 2, max(M.LANES_Y) + M.BLADE_W / 2)
    check(all(reach[0] <= p[1] <= reach[1] for p in corners) and max(p[0] for p in corners) < M.FINAL_X_START,
          "every piece lies within the demo's three lanes and short of where its strokes land")
    T = M.TOTAL_MASS
    check(abs(T * 1000 - 55.0) < 1e-6 and all(abs(m - 0.0025) < 1e-12 for m in M.CHIP_MASS),
          f"debris {T * 1000:.2f} g: 22 pieces of 2.50 g")
    # the bonds: every piece stuck to the plate on its own; it comes free under the blade
    check(all(c.bond is not None for c in C), "every piece is bonded to the plate")
    check(all(c.bond == (M.BOND_FORCE, M.BOND_FORCE * M.BOND_PEEL_ARM) for c in C),
          "break force 0.8 N, break torque 0.8 N x 2.5 mm")
    check(M.CHIP_LIFT == 0.0, "every piece lies on the plate (no gap under it)")
    check(M.PLATE_FRICTION == (0.70, 0.60) and M.dk._SPEC["physics"].static_friction == 0.70
          and M.dk._SPEC["physics"].dynamic_friction == 0.60, "clean dry steel: 0.7 static, 0.6 sliding, for the "
                                                              "blade and for a freed piece")
    check(min(c.bond[0] / (c.mass * 9.81) for c in C) >= 20, "every bond holds at least 20 times its piece's weight")
    check(M.dk._SPEC["anchor"] == (M.PLATE.name, M.PLATE.pos) and M.PLATE.mass is None,
          "the bonds anchor on the plate, a fixed body")
    p0, q0, p1 = M.dk.bond_frames(C[0])
    check(abs(M.PLATE.pos[2] + p0[2] - (C[0].pos[2] - C[0].size[2] / 2)) < 1e-9 and p1 == (0.0, 0.0, -C[0].size[2] / 2),
          "each bond sits at the middle of its piece's underside")
    check(abs(M.blade_angle(M.pitch_for(40.0)) - 40.0) < 1e-6, "demo pitch gives a 40 degree blade")
    check(M.shares(M.START_DEBRIS) == (0.0, 1.0, 0.0), "at the start everything is on the griddle")

    # -- where a piece counts ---------------------------------------------------------------------------------------
    d = M.START_DEBRIS[:1].copy()
    probe = lambda x, y, z: [bool(v[0]) for v in M.where(np.array([[x, y, z, 0, 0, 0, 1]]))]  # noqa: E731
    check(probe(0.11, 0.0, M.PLATE_Z - M.TRAY_DEPTH + 0.001) == [True, False, False], "on the tray floor: in the tray")
    check(probe(0.11, 0.0, M.PLATE_Z + 0.02) == [False, True, False], "above the tray (falling, or on the blade): not yet")
    check(probe(0.30, 0.0, M.PLATE_Z + 0.001) == [False, True, False], "on the plate: left")
    check(probe(0.30, M.PLATE_Y0 - 0.05, M.TOP + 0.001) == [False, False, True], "on the bench beside it: off")
    check(probe(0.05, 0.0, M.TOP + 0.001) == [False, False, True], "on the bench in front of the tray: off")
    del d

    # -- nothing, the scraper's start, fixed griddle -------------------------------------------------------------------
    fresh()
    check(feed(set_down(60)) == 0.0, "doing nothing: 0")
    fresh()
    check(feed(held_at_rest(60)) == 0.0 and not M._L.lifted, "taking hold of the scraper where it lies: 0, not lifted")
    check(not M.in_spot(M.SCRAPER_START) and math.dist(M.SCRAPER_START[:2], M.SPOT_XY) >= 0.10,
          "the scraper's start is outside the taped square, its origin 10 cm or more from the square's center")
    check(max(abs(M.SCRAPER_XY[0] - M.SPOT_XY[0]), abs(M.SCRAPER_XY[1] - M.SPOT_XY[1])) - M.SPOT_SIDE / 2 >= 0.08,
          "the start origin lies 8 cm or more outside the tape's outer edge (a 3 cm placement error stays outside)")
    lo = REST_POS + M.R_REST @ np.array([-(M.BLADE_L + M.NECK_L), -M.BLADE_W / 2, 0.0])
    check(lo[1] - (M.SPOT_XY[1] + M.SPOT_SIDE / 2) >= 0.02, "the scraper at its start lies clear of the tape (2 cm)")
    check(all(o.mass is None for o in (M.PLATE, M.TRAY, M.TRAY_WALL, M.GUARD_R, M.GUARD_L, M.GUARD_B, *M.TAPE)),
          "plate, tray, tray wall, guards and tape are fixed bodies: the griddle cannot be tipped nor the tray moved")

    # -- tiers, every piece put in the tray by the held blade ---------------------------------------------------------
    for k, want in ((5, 0.0), (6, 0.25), (13, 0.25), (14, 0.5), (20, 0.5), (21, 0.75)):
        fresh()
        sts, d = scrape(ALL[:k])
        s = run(sts, d, set_down(40, pos=SPOT_POS))
        tr, res, _ = M._L.shares()
        check(s == want, f"{k} pieces scraped into the tray ({tr:.1%}), the rest on the plate ({res:.1%}): {want} ({s})")
    fresh()
    s = run(JOB, FULL, set_down(40, pos=SPOT_POS))
    check(s == 1.0 and M._L.credit.all(), "every piece scraped over the lip by the held blade, scraper picked up and "
                                         "set down in the taped square for 2 s: 1.0")
    fresh()
    sts, d = scrape(ALL[1:])
    loose = d.copy()
    loose[0, :3] = (0.30, 0.0, M.PLATE_Z + M.PIECE_T / 2)
    s = run(sts, loose, set_down(40, pos=SPOT_POS))
    check(s == 0.75, f"a piece freed but left loose on the plate: 0.75 ({s})")
    fresh()
    sts, d = scrape(ALL[2:])
    s = run(sts, d, set_down(40, pos=SPOT_POS))
    check(s == 0.5, f"two pieces left on the plate: 0.5 ({s})")
    fresh()
    sts, d = scrape(ALL[1:])
    flung = d.copy()
    flung[0, :3] = (0.40, -0.26, M.TOP + M.PIECE_T / 2)
    s = run(sts, flung, set_down(40, pos=SPOT_POS))
    check(s == 0.75, f"one piece flung off the griddle, the rest scraped in: 0.75, not 1.0 ({s})")
    fresh()
    sts, d = scrape(ALL[2:])
    two = d.copy()
    two[0, :3] = (0.40, -0.26, M.TOP + M.PIECE_T / 2)
    two[1, :3] = (0.05, 0.0, M.TOP + M.PIECE_T / 2)
    s = run(sts, two, set_down(40, pos=SPOT_POS))
    check(s == 0.25, f"two pieces (9.1 %) off the griddle, the rest scraped in: capped at 0.25 ({s})")
    blade = (SPOT_POS + M.R_REST @ M.EDGE_LOCAL)[:2]
    fresh()
    sts, d = scrape(ALL[1:])
    carried = d.copy()
    carried[0, :3] = (blade[0] + 0.02, blade[1], M.TOP + 0.006)
    s = run(sts, carried, set_down(40, pos=SPOT_POS))
    check(s == 0.75 and M._L.shares()[2] > 0, f"one piece carried off on the blade to the bench: off, 0.75 ({s})")
    riding = FULL.copy()
    riding[0, :3] = (0.11, -0.02, M.PLATE_Z + 0.015)
    check(M.shares(riding)[1] > 0, "a piece on the blade held over the tray, above the plate's top: still on the griddle")

    # -- the set-down --------------------------------------------------------------------------------------------------
    for what, down, want in (
            ("set down only 0.5 s", set_down(10, pos=SPOT_POS), 0.75),
            ("laid in the square but still in hand", [objs(SPOT_POS, M.R_REST, True)] * 40, 0.75),
            ("set down in the square but still moving", set_down(40, pos=SPOT_POS, wobble=True), 0.75),
            ("put back where it started (outside the square)", set_down(40), 0.75),
            ("left lying on the griddle", set_down(40, pos=np.array([0.40, 0.0, M.PLATE_Z + 0.012]), rot=np.eye(3)),
             0.75),
            ("dropped into the grease tray", set_down(40, pos=np.array([0.11, 0.0, M.PLATE_Z - M.TRAY_DEPTH + 0.006]),
                                                       rot=np.eye(3)), 0.75),
            ("dropped flat on the bench outside the square", set_down(40, pos=np.array([0.30, -0.24, M.TOP + 0.012]),
                                                                      rot=np.eye(3)), 0.75),
            ("dropped flat inside the square", set_down(40, pos=SPOT_POS + np.array([0.03, -0.02, 0.0]),
                                                         rot=rz(0.3)), 1.0)):
        fresh()
        s = run(JOB, FULL, down)
        check(s == want, f"everything scraped in, scraper {what}: {want} ({s})")
    fresh()
    drag = [objs(REST_POS + (SPOT_POS - REST_POS) * k / 30, M.R_REST, True) for k in range(31)]
    s = feed(drag + set_down(40, pos=SPOT_POS))
    check(s == 0.0 and not M._L.lifted, "the scraper dragged along the bench into the square, never lifted: not picked "
                                        "up, 0")

    # -- cheats: anything but the held blade putting debris in the tray ------------------------------------------------
    for how, what in (("hand", "pinched and carried over the lip by a gripper"),
                      ("none", "moved into the tray with no blade touching it (the arm's body, a tipped griddle or a "
                               "moved tray)"),
                      ("loose", "pushed by the scraper while nobody holds it (thrown or let go)"),
                      ("handle", "pushed with the scraper's handle end (the blade not touching)")):
        fresh()
        sts, d = scrape(ALL, how=how)
        s = run(sts, d, set_down(40, pos=SPOT_POS))
        check(s == 0.0 and not M._L.credit.any(), f"every piece {what}: none credited, 0 ({s})")
    fresh()
    sts, d = scrape(ALL[:1], how="hand")
    sts2, d = scrape(ALL[1:], d0=d)
    s = run(sts + sts2, d, set_down(40, pos=SPOT_POS))
    check(s == 0.0 and M._L.hands, f"one piece moved by a gripper, the other 21 scraped in: a hand on the debris, 0 ({s})")
    fresh()
    touch, _ = scrape(ALL[:1], how="hand")
    sts, d = scrape(ALL)
    s = run(touch[:2] + sts, d, set_down(40, pos=SPOT_POS))
    check(s == 0.0 and M._L.hands, f"a gripper pressing one piece for 0.1 s (its bond broken by hand), then every piece "
                                   f"scraped in: 0 ({s})")
    fresh()
    o, op = JOB[40]
    o = dict(o)
    dd = o["debris"].copy()
    dd[21, 0] -= 0.003
    o["debris"] = dd
    o["left_gripper"] = st(dd[21, :3] + np.array([0.0, 0.0, M.PIECE_T / 2 + 0.006]), grip_rotation(0.0))
    s = run(JOB[:40] + [(o, dict(op, left_gripper=0.0))] + JOB[41:], FULL, set_down(40, pos=SPOT_POS))
    check(s == 0.0, f"the free hand moving one piece 3 mm once while the other hand scrapes: 0 ({s})")
    fresh()
    o, op = JOB[-1]
    o = dict(o)
    dd = o["debris"].copy()
    dd[3, 2] += 0.010
    o["debris"] = dd
    o["left_gripper"] = st(dd[3, :3] + np.array([0.0, 0.0, M.PIECE_T / 2 + 0.006]), grip_rotation(0.0))
    s = run(JOB + [(o, dict(op, left_gripper=0.0))], FULL, set_down(40, pos=SPOT_POS))
    check(s == 0.0, f"a gripper lifting a piece already in the tray: 0 ({s})")
    # pushed to the lip by the blade, left there, then over the lip 2 s later with nothing touching it
    fresh()
    sts, d = scrape(ALL[1:])
    x0, y0 = float(d[0, 0]), float(d[0, 1])
    up = []
    for x in np.arange(x0, M.LIP_X + M.PIECE_W / 2 - 0.002, -0.008):
        d[0, 0] = x
        up.append(pusher_state(d.copy(), "blade", x + M.PIECE_W / 2, y0))
    up += [pusher_state(d.copy(), "none", 0, 0)] * 40
    d[0, 0] = M.LIP_X - 0.005
    up.append(pusher_state(d.copy(), "none", 0, 0))
    d[0, :3] = tray_place(0)
    s = run(sts + up, d, set_down(40, pos=SPOT_POS))
    check(s == 0.75 and not M._L.credit[0], f"a piece pushed to the lip by the blade, then over it 2 s later with "
                                            f"nothing touching it: not credited, 0.75 ({s})")
    # a pile: the blade pushes piece A along its lane into piece B (same lane, nearer the lip); A pushes B over the lip
    fresh()
    d = M.START_DEBRIS.copy()
    a_, b_ = 11, 0  # row 3 (x 24.5) and row 1 (x 17.5), both at y -10.75
    y = float(d[a_, 1])
    xb0 = float(d[b_, 0])
    pile = []
    for x in np.arange(float(d[a_, 0]), M.LIP_X - 0.030, -0.008):
        d[a_, :3] = (x, y, M.PLATE_Z + M.PIECE_T / 2)
        xb = min(xb0, x - M.PIECE_W - 0.0003)
        d[b_, :3] = (xb, y, M.PLATE_Z + M.PIECE_T / 2)
        pile.append(pusher_state(d.copy(), "blade", x + M.PIECE_W / 2, y))
    d[a_, :3], d[b_, :3] = tray_place(a_), tray_place(b_)
    pile.append(pusher_state(d.copy(), "blade", M.LIP_X - 0.02, y))
    feed(pile)
    check(M._L.credit[a_] and M._L.credit[b_], "a piece pushed over the lip by another piece the blade pushes: both "
                                               "credited (a pile in front of the blade)")

    # the blade 2 mm above a piece (no contact) while something else carries it over the lip: not credited
    fresh()
    d = M.START_DEBRIS.copy()
    hover = []
    y = float(d[0, 1])
    for x in np.arange(float(d[0, 0]), M.LIP_X - 0.012, -0.008):
        d[0, :3] = (x, y, M.PLATE_Z + M.PIECE_T / 2)
        o, op = objs(at_edge((x + M.PIECE_W / 2 + 0.0025, y, M.PLATE_Z - 0.001), R40), R40, True, P40, debris=d.copy())
        hover.append((o, op))
    feed(hover)
    check(not M._L.credit[0] and not M._L.freed[0], "the blade 2 mm behind a piece that something else moves over the "
                                                    "lip (not touching it): not freed by the blade, not credited")
    # a piece first moved 3 mm by something other than the blade, then scraped in by the blade: not credited
    fresh()
    d = M.START_DEBRIS.copy()
    d[0, 0] -= 0.003
    nudged = [objs(REST_POS, M.R_REST, False, debris=d.copy())]
    sts, d = scrape(ALL, d0=d)
    s = run(nudged + sts, d, set_down(40, pos=SPOT_POS))
    check(not M._L.freed[0] and not M._L.credit[0] and s == 0.75, f"a piece knocked loose by something other than the "
                                                                  f"blade, then scraped in: not credited, 0.75 ({s})")
    # a bond that gives slowly under the blade (the piece creeps 0.15 mm a step before it lets go): freed by the blade
    fresh()
    d = M.START_DEBRIS.copy()
    y = float(d[0, 1])
    creep = []
    for k in range(1, 6):
        d[0, 0] = M.START_DEBRIS[0, 0] - 0.00015 * k
        creep.append(pusher_state(d.copy(), "blade", float(d[0, 0]) + M.PIECE_W / 2, y))
    sts, d = scrape(ALL, d0=d)
    s = run(creep + sts, d, set_down(40, pos=SPOT_POS))
    check(M._L.freed[0] and s == 1.0, f"a piece that creeps off its place under the blade before its bond lets go: "
                                      f"freed by the blade, 1.0 ({s})")
    # a hand pushing a piece slowly (5 mm/s, 0.25 mm a step) still touches the dough
    fresh()
    sts, d = scrape(ALL[:1], how="hand", step=0.00025)
    feed(sts[:40])
    check(M._L.hands, "a gripper pushing a piece slowly (5 mm/s): a hand on the dough")
    # near misses: a finger 2 mm above a still piece, or 2 mm off a piece the blade moves, is not a touch
    fresh()
    o, op = pusher_state(M.START_DEBRIS.copy(), "hand", 0, 0)
    o = dict(o)
    o["right_gripper"] = st(M.START_DEBRIS[0, :3] + np.array([0.0, 0.0, M.PIECE_T / 2 + 0.0128 + 0.002]),
                            grip_rotation(0.0))
    feed([(o, dict(op, right_gripper=0.0))] * 10)
    check(not M._L.hands, "fingertips 2 mm above a piece (not touching): no hand on the debris")
    fresh()
    o2 = []
    for o_, op_ in JOB[:60]:
        o_ = dict(o_)
        dd = o_["debris"]
        o_["left_gripper"] = st(dd[0, :3] + np.array([0.0, 0.0, M.PIECE_T / 2 + 0.0128 + 0.002]), grip_rotation(0.0))
        o2.append((o_, dict(op_, left_gripper=0.0)))
    feed(o2)
    check(not M._L.hands and M._L.credit[0], "a free hand following 2 mm above a piece the blade pushes in: no touch, "
                                             "credited")
    fresh()
    still = M.START_DEBRIS.copy()
    o, op = pusher_state(still, "hand", float(still[0, 0]) + M.PIECE_W / 2, float(still[0, 1]))
    feed([(dict(o), op)] * 10)
    check(not M._L.hands, "a fingertip resting on a bonded piece that does not move: not counted (nothing moved)")
    # the arm's own collision meshes (as the simulator reports each link's pose): a forearm or a fingertip moving a
    # piece is the robot on the dough; the same link 2 mm clear of it is not
    def link_sweep(link, gap_m, axis):
        """Piece 0 pushed along the plate toward the lip; `link` (identity turn) placed so its nearest mesh point is
        gap_m behind the piece's far side (axis 0) or above its top (axis 2)."""
        pts = M.mesh(link)[0]
        k = int(np.argmin(pts[:, axis]))
        fresh()
        d = M.START_DEBRIS.copy()
        out = []
        y = float(d[0, 1])
        for x in list(np.arange(float(d[0, 0]), M.LIP_X - 0.012, -0.008)):
            d[0, :3] = (x, y, M.PLATE_Z + M.PIECE_T / 2)
            o, op = objs(REST_POS, M.R_REST, False, debris=d.copy())
            o = dict(o)
            target = (np.array([x + M.PIECE_W / 2 + gap_m, y, M.PLATE_Z + M.PIECE_T / 2]) if axis == 0 else
                      np.array([x, y, M.PLATE_Z + M.PIECE_T + gap_m]))
            o["right_gripper" + kit.ARM_SUFFIX] = {link: (target - pts[k], np.array([0.0, 0.0, 0.0, 1.0]))}
            out.append((o, op))
        feed(out)
        return M._L.hands
    check(link_sweep("link3", -0.0003, 0), "the upper arm's mesh sweeping a piece toward the lip: the robot on the "
                                           "dough, 0")
    check(link_sweep("link4", -0.0003, 0), "the forearm's mesh sweeping a piece: the robot on the dough")
    check(link_sweep("tip_left", -0.0003, 2), "a fingertip's mesh pressing on a piece as it moves: the robot on the "
                                              "dough")
    check(not link_sweep("link4", 0.002, 0), "the forearm's mesh 2 mm clear of a moving piece: not a touch")
    check(not link_sweep("tip_left", 0.002, 2), "a fingertip's mesh 2 mm above a moving piece: not a touch")
    # a piece on the top of a splash guard is still on the griddle
    check([bool(v[0]) for v in M.where(np.array([[0.30, M.PLATE_Y0 - 0.0015, M.PLATE_Z + M.GUARD_H + 0.003, 0, 0, 0,
                                                  1]]))] == [False, True, False],
          "a piece resting on a splash guard's top: left on the griddle")
    # the bond peels under an edge driven beneath a piece; each piece lies on the steel
    check(M.BOND_PEEL_ARM == 0.0025 and M.CHIP_LIFT == 0.0, "break torque = bond x 2.5 mm (an edge under a piece "
                                                            "peels it); each piece lies on the steel")
    # a piece pressed loose from above by the handle, then pushed over by the blade: not freed by the blade
    fresh()
    d = M.START_DEBRIS.copy()
    d[0, 2] -= 0.0006  # pressed down 0.6 mm: its bond lets go under the handle
    r = rz(math.pi) @ M.R_REST
    org = np.array([d[0, 0] + M.HANDLE[0][0], d[0, 1], d[0, 2] + M.PIECE_T / 2 + M.HANDLE[1][2] / 2])
    pressed = [objs(org, r, True, 0.0, debris=d.copy())]
    sts, d = scrape(ALL, d0=d)
    s = run(pressed + sts, d, set_down(40, pos=SPOT_POS))
    check(not M._L.freed[0] and s == 0.75, f"a piece pressed loose with the handle, then scraped in: not credited, "
                                           f"0.75 ({s})")

    # -- gouge cap ----------------------------------------------------------------------------------------------------
    fresh()
    s = run(stroke(0.0, theta=78.0) + JOB, FULL, set_down(40, pos=SPOT_POS))
    check(s == 0.5 and M._L.s.gouged, "a stroke at 78 degrees, then everything scraped in: gouge cap 0.5")
    fresh()
    s = run(stroke(0.0, roll=math.radians(15)) + JOB, FULL, set_down(40, pos=SPOT_POS))
    check(s == 0.5 and M._L.s.gouged, "a stroke on the corner (edge tilted 15 degrees): 0.5")
    fresh()
    s = run(stroke(0.0, roll=math.radians(8)) + JOB, FULL, set_down(40, pos=SPOT_POS))
    check(s == 1.0 and not M._L.s.gouged, "edge tilted 8 degrees is not a gouge: 1.0")
    fresh()
    s = feed(job() + set_down(40, pos=SPOT_POS))
    check(s == 0.0 and not M._L.s.gouged, "the demo's strokes with the debris not moving (nothing pushed): 0")

    # -- a new trial resets -------------------------------------------------------------------------------------------
    fresh()
    s = feed(set_down(5))
    check(s == 0.0 and not M._L.s.gouged, "new trial: latches cleared")

    # -- text matches code --------------------------------------------------------------------------------------------
    d = json.loads((HERE / "task.json").read_text())
    readme = (HERE / "README.md").read_text()
    check(d["instruction"] == M.INSTRUCTION, "task.json instruction == task.INSTRUCTION")
    text = " ".join(str(d[k]) for k in ("setup", "scoring", "fidelity", "context")) + readme
    for frag in ("25 %", "60 %", "90 %", "98 %", "5 %", "70 degrees", "10 degrees", "1 cm", "1 s", "55.0 g", "22 ",
                 "2.50 g", "1.9 cm", "4.8 mm", "measuring cup", "water bottle", "taped square", "white nitrile", "capped at 0.5", "caps the score at 0.25", "136 g",
                 "10.2 cm", "5.0 cm", "0.6 mm", "0.7", "0.6", "0.5 mm", "1 mm", "0.5 s", "x 17.5, 21.0, 24.5 and 28.0",
                 "-10.75 to 6.75", "-9.0 to 5.0", "Play-Doh", "177AOGT1227", "P94850", "BOMATA", "0.40 g",
                 "tray's rim", "3 minutes", "5 minutes"):
        check(frag in text or frag.replace(" %", "%") in text, f"text states {frag!r}")
    limits = dict(title=120, instruction=1000, context=1000, setup=4000, scoring=4000, fidelity=500, scores=300)
    for k, n in limits.items():
        check(len(d.get(k, "")) <= n, f"{k} within {n} chars ({len(d.get(k, ''))})")
    check(not re.search(r"\b(colour|centre|grey|metre|litre|aluminium)\b", json.dumps(d) + readme), "American spelling")

    # -- the demo plan: limits and a kinematic playback ----------------------------------------------------------------
    plan = M.demo()["right"]
    steps = round(sum(w.duration for w in plan) / M.DT)
    check(steps <= 2300, f"demo plan {steps} steps (fits the README's 2,400-step demo run)")
    xs, ys, zs = ([w.pos[i] for w in plan] for i in range(3))
    check(min(xs) >= 0.15 and max(xs) <= 0.48, f"tool points within x 0.15-0.48 m ({min(xs):.3f}-{max(xs):.3f})")
    check(max(zs) - M.TOP <= 0.19, f"highest tool point {max(zs) - M.TOP:.3f} m above the table (wpcheck passes to 0.19)")
    check(all(-0.45 < y < 0.30 for y in ys),
          "tool points within y -0.45 to 0.30 m (the taped square lies 10 cm outboard of the right arm's base)")
    check(all(w.yaw == 0.0 or (getattr(w, "pitch", 0.0) != 0.0 and w.yaw == M.YAW_TRIM) for w in plan),
          "every waypoint at yaw 0 (fingers across world y, across the handle); tilted ones trimmed by YAW_TRIM")
    fins = {float(w.finger) for w in plan}
    check(fins <= {float(M.FINGER_OPEN), float(M.FINGER_CLOSED)}, "finger targets are FINGER_OPEN / FINGER_CLOSED only")
    fresh()
    states = []
    prev = None
    attached = False
    held_poses = []
    for w in plan:
        pitch = getattr(w, "pitch", 0.0)
        n = max(1, round(w.duration / M.DT))
        a = prev if prev is not None else (np.asarray(w.pos, float), pitch)
        for k in range(1, n + 1):
            s_ = k / n
            pos = a[0] + s_ * (np.asarray(w.pos, float) - a[0])
            pt = a[1] + s_ * (pitch - a[1])
            if attached:
                r = M.scraper_rot(pt)
                o = pos - r @ M.GRASP_LOCAL
                e = o + r @ M.EDGE_LOCAL  # the plate's collider stops an edge aimed into it (PRESS) at SINK
                if e[0] >= M.LIP_X and e[2] < M.PLATE_Z - SINK:
                    o = o + np.array([0.0, 0.0, M.PLATE_Z - SINK - e[2]])
                states.append(objs(o, r, True, pt))
                held_poses.append((o, r))
            else:
                states.append(objs(REST_POS, M.R_REST, False))
        if w.label == "grip":
            attached = True
        if w.label == "let go":
            attached = False
        prev = (np.asarray(w.pos, float), pitch)
    # the held scraper crosses the right splash guard only above its top (the playback has no colliders)
    boxes = (M.HANDLE, M.BLADE_BOX, M.NECK_BOX)
    corners = np.array([np.asarray(c) + np.asarray(sz) / 2 * np.array([i, j, k]) for c, sz in boxes
                        for i in (-1, 1) for j in (-1, 1) for k in (-1, 1)])
    g_top, g_in, g_out = M.PLATE_Z + M.GUARD_H, M.PLATE_Y0, M.PLATE_Y0 - M.GUARD_T
    worst = 1.0
    for o, r in held_poses:
        w_ = o[None, :] + corners @ r.T
        if not (np.all(w_[:, 1] > g_in) or np.all(w_[:, 1] < g_out)):
            worst = min(worst, float(w_[:, 2].min()) - g_top)
    check(worst >= 0.005, f"the held scraper crosses the splash guard at least 5 mm above it ({worst * 1000:.1f} mm)")
    g_left = M.PLATE_Y0 + M.PLATE_W
    ymax = max(float((o[None, :] + corners @ r.T)[:, 1].max()) for o, r in held_poses
               if float((o[None, :] + corners @ r.T)[:, 2].min()) < g_top)
    check(ymax <= g_left - 0.004, f"below the guards' top the scraper stays at least 4 mm inside the left guard "
                                  f"({(g_left - ymax) * 1000:.1f} mm)")
    lanes = sorted([*M.LANES_Y, *M.EDGE_LANES_Y])
    reach = (lanes[0] - M.BLADE_W / 2, lanes[-1] + M.BLADE_W / 2)
    check(M.PLATE_Y0 + 0.004 <= reach[0] <= M.PLATE_Y0 + 0.006 and g_left - 0.006 <= reach[1] <= g_left - 0.004,
          f"the cleanup strokes reach within 4 to 6 mm of both guards (y {reach[0] * 100:.2f} to {reach[1] * 100:.2f} cm)")
    touched = 0
    for o, opening in states:
        kit._OPENING.update(opening)
        M.success(dict(o))
        touched += int(M._L.s.touching)
    check(not M._L.s.gouged and touched >= 9 * 60 + 60, f"kinematic playback of the demo plan: the edge on the plate for "
                                                     f"{touched} steps (11 strokes), no gouge")
    print("all pass")


def test_all():
    main()


if __name__ == "__main__":
    sys.exit(main())
