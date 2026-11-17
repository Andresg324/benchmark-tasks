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
    fractions asked for (within one flake, under 1.5 %)."""
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


def main():
    import itertools

    C = M.CHIPS
    check(len(C) == 88, f"{len(C)} debris fragments (rigid bodies)")
    check(not any(M.overlap2(a, b, 0.0005) for a, b in itertools.combinations(C, 2)),
          "no two fragments overlap or touch at the start (cracks at least 0.5 mm)")
    check(min(c.size[2] for c in C) >= 0.002, "every fragment at least 2 mm thick (no tunnelling through the blade)")
    check(all(abs(c.pos[2] - c.size[2] / 2 - M.PLATE_Z - M.CHIP_LIFT) < 1e-9 for c in C), "every fragment on the plate")
    r = M.RING_D / 2
    for k, (cx, cy, _) in enumerate(M.CAKES):
        mine = [c for c in C if c.name.startswith(f"cake{k}_")]
        far = max(math.hypot(p[0] - cx, p[1] - cy) for c in mine
                  for p in M._corners2(c.pos[0], c.pos[1], c.size[0], c.size[1], c.yaw))
        area = sum(c.size[0] * c.size[1] for c in mine)
        check(len(mine) == 22 and far <= r + 0.004, f"pancake {k + 1}: 22 fragments, corners within 4 mm of its 7 cm ring "
                                                   f"(farthest corner {far * 1000:.1f} mm from the center)")
        check(0.75 <= area / (math.pi * r * r) <= 1.0, f"pancake {k + 1}: fragments cover {area / (math.pi * r * r):.0%}"
                                                       f" of the disc (the rest is cracks)")
    for a, b in itertools.combinations(M.CAKES, 2):
        check(math.hypot(a[0] - b[0], a[1] - b[1]) - M.RING_D >= 0.008, "rings at least 8 mm apart")
    check(all(M.LIP_X + 0.0005 <= p[0] and M.PLATE_Y0 + 0.05 <= p[1] <= M.PLATE_Y0 + M.PLATE_W - 0.05 for c in C
              for p in M._corners2(c.pos[0], c.pos[1], c.size[0], c.size[1], c.yaw)),
          "every fragment on the plate behind the lip, 5 cm or more inside the side guards")
    T = M.TOTAL_MASS
    check(abs(T * 1000 - 55.5) < 0.05, f"debris {T * 1000:.2f} g")
    sh = M.CHIP_MASS / T
    check(0.007 <= sh.min() and sh.max() <= 0.018, f"each fragment {sh.min():.1%} to {sh.max():.1%} of the mass")
    # the bonds: every fragment stuck to the plate, break force from its width across the stroke, far above its weight
    check(all(c.bond is not None for c in C), "every fragment is bonded to the plate")
    check(all(abs(c.bond[0] - M.BOND_EDGE * M.across(c)) < 1e-9 for c in C), "break force = 12 N per m of width")
    check(all(abs(c.bond[1] - c.bond[0] * M.BOND_PEEL_ARM) < 1e-12 for c in C), "break torque = break force x 2 mm")
    fb = [c.bond[0] for c in C]
    check(0.10 <= min(fb) and max(fb) <= 0.22, f"fragments break free at {min(fb):.2f}-{max(fb):.2f} N")
    check(min(c.bond[0] / (c.mass * 9.81) for c in C) >= 20, "every bond holds at least 20 times its fragment's weight")
    check(M.dk._SPEC["anchor"] == (M.PLATE.name, M.PLATE.pos) and M.PLATE.mass is None,
          "the bonds anchor on the plate, a fixed body")
    p0, q0, p1 = M.dk.bond_frames(C[0])
    check(abs(M.PLATE.pos[2] + p0[2] - (C[0].pos[2] - C[0].size[2] / 2)) < 1e-9 and p1 == (0.0, 0.0, -C[0].size[2] / 2),
          "each bond sits at the middle of its fragment's underside")
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

    # -- nothing, hand, tiers -----------------------------------------------------------------------------------------
    fresh()
    check(feed(set_down(60)) == 0.0, "doing nothing: 0")
    fresh()
    check(feed(held_at_rest(60)) == 0.0, "picking the scraper up and holding it: 0")
    for frac, want in ((0.24, 0.0), (0.265, 0.25), (0.615, 0.5), (0.93, 0.5)):
        fresh()
        s = feed(with_debris(moved(frac), set_down(40)))
        tr, res, _ = M.shares(moved(frac))
        check(s == want, f"{tr:.1%} in the tray, the rest on the plate ({res:.1%}): {want} ({s})")
    # -- the full job, the hold, the hand -----------------------------------------------------------------------------
    full = moved(1.0)
    fresh()
    s = feed(with_debris(full, set_down(40)))
    check(s == 1.0, "everything in the tray, scraper set down 2 s: 1.0")
    one = moved(1.0)
    i_big = int(np.argmax(M.CHIP_MASS))
    one[i_big] = M.START_DEBRIS[i_big]
    fresh()
    s = feed(with_debris(one, set_down(40)))
    check(s == 0.75, f"a single fragment ({M.CHIP_MASS[i_big] / T:.1%}) left on the plate, the rest in the tray, "
                     f"scraper set down: 0.75, below 1.0 ({s})")
    one = moved(1.0)
    i_small = int(np.argmin(M.CHIP_MASS))
    one[i_small] = M.START_DEBRIS[i_small]
    fresh()
    s = feed(with_debris(one, set_down(40)))
    check(s == 0.75, f"even the smallest fragment ({M.CHIP_MASS[i_small] / T:.1%}) left on the plate: 0.75 ({s})")
    loose = moved(1.0)
    loose[i_small, :3] = (0.30, 0.0, M.PLATE_Z + 0.0026)
    fresh()
    s = feed(with_debris(loose, set_down(40)))
    check(s == 0.75, f"a fragment freed but left loose on the plate: 0.75 ({s})")
    two = moved(1.0)
    big = np.argsort(M.CHIP_MASS)[-2:]
    two[big] = M.START_DEBRIS[big]
    fresh()
    s = feed(with_debris(two, set_down(40)))
    check(s == 0.5, f"the two largest fragments ({M.CHIP_MASS[big].sum() / T:.1%}) left on the plate: 0.5 ({s})")
    flung = moved(1.0)
    flung[i_big, :3] = (0.40, -0.26, M.TOP + 0.0026)
    fresh()
    s = feed(with_debris(flung, set_down(40)))
    check(s == 1.0, f"the rest in the tray and one fragment ({M.CHIP_MASS[i_big] / T:.1%}) off the griddle (under the "
                    f"2 % cap), nothing left on it: 1.0 ({s})")
    fresh()
    s = feed(with_debris(full, set_down(10)))
    check(s == 0.75, "set down only 0.5 s: 0.75 (hold not met)")
    fresh()
    s = feed(with_debris(full, held_at_rest(40)))
    check(s == 0.75, "scraper laid on the bench but still in hand: 0.75")
    fresh()
    s = feed(with_debris(full, set_down(40, wobble=True)))
    check(s == 0.75, "scraper set down but still moving: 0.75")
    fresh()
    s = feed(with_debris(full, set_down(40, pos=np.array([0.40, 0.0, M.PLATE_Z + 0.012]), rot=np.eye(3))))
    check(s == 0.75, "scraper left lying on the griddle: 0.75")
    fresh()
    s = feed(with_debris(full, set_down(40, pos=np.array([0.11, 0.0, M.PLATE_Z - M.TRAY_DEPTH + 0.006]),
                                        rot=np.eye(3))))
    check(s == 0.75, "scraper dropped into the grease tray: 0.75, not set down")
    fresh()
    s = feed(with_debris(full, set_down(40, pos=np.array([0.30, -0.24, M.TOP + 0.012]), rot=np.eye(3))))
    check(s == 1.0, "scraper dropped flat elsewhere on the bench: set down, 1.0")

    # -- caps ---------------------------------------------------------------------------------------------------------
    fresh()
    s = feed(with_debris(moved(0.96, 0.035), set_down(40)))
    off = M.shares(moved(0.96, 0.035))[2]
    check(s == 0.25 and off > 0.02, f"{off:.1%} of the debris pushed off the griddle onto the bench: capped at 0.25 ({s})")
    fresh()
    s = feed(with_debris(moved(0.70, 0.035), set_down(40)))
    check(s == 0.25, "70 % in the tray but over 2 % off the griddle: 0.25")
    fresh()
    s = feed(stroke(0.0, theta=78.0) + with_debris(full, set_down(40)))
    check(s == 0.5 and M._L.s.gouged, "a stroke at 78 degrees, then everything in the tray: gouge cap 0.5")
    fresh()
    s = feed(stroke(0.0, roll=math.radians(15)) + with_debris(full, set_down(40)))
    check(s == 0.5 and M._L.s.gouged, "a stroke on the corner (edge tilted 15 degrees): 0.5")
    fresh()
    s = feed(stroke(0.0, roll=math.radians(8)) + with_debris(full, set_down(40)))
    check(s == 1.0 and not M._L.s.gouged, "edge tilted 8 degrees is not a gouge: 1.0")
    fresh()
    s = feed(job() + set_down(40))
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
    for frag in ("25 %", "60 %", "90 %", "98 %", "2 %", "70 degrees", "10 degrees", "1 cm", "1 s",
                 "55.5 g", "88", "22 fragments", "four", "136 g", "10.2 cm", "5.0 cm", "0.6 mm", "0.15 to 0.2",
                 "0.4 to 0.5", "0.9 g per cubic cm", "capped at 0.5", "caps the score at 0.25", "7.0 cm", "5 mm",
                 "12 N per m", "0.11 to 0.21 N", "0.8 to 1.7", "x 19.5, y -7.5 and 0.5", "x 26.4, y -3.5 and 4.5",
                 "1.0 cm from the lip", "6.55 cm", "Norpro 957R", "Zen Principle", "177AOGT1227", "P94850", "14.0 g",
                 "USS-DBS66-20", "3 minutes"):
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
    check(all(abs(y) < 0.30 for y in ys), "tool points within |y| < 0.30 m")
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
