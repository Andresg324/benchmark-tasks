"""Offline checks of the tile cutter's scorer: hand-made trajectories of the carriage and the handle, and gripper
positions, stepped through success() as the simulator would. No simulator needed.

    python test_task.py        (with the yam_arena package importable)
"""

import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit

HOME = {"left_gripper": (0.20, 0.27, 0.95), "right_gripper": (0.18, -0.26, 0.95)}
END = m.TRAVEL  # the carriage at its far stop
UP = m.RAISED


def st(pos):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array([0.0, 0, 0, 1]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def ramp(a, b, n=20):
    return list(np.linspace(a, b, n))


def run(path, grippers=None, end_vel=0.0, extra_end=4):
    """One trial: `path` is a list of (carriage travel, handle angle) per step; the grippers stay home unless
    `grippers` (a dict side -> position) is given for the final steps. Returns (score, success) at the end."""
    kit._EPISODE["count"] += 1
    path = list(path) + [path[-1]] * extra_end
    su = False
    for k, (c, q) in enumerate(path):
        last = k == len(path) - 1
        kit._JOINTS["cutter/carriage"] = kit.JointState(pos=float(c), vel=end_vel if last else 0.0, latched=False)
        kit._JOINTS["cutter/handle"] = kit.JointState(pos=float(q), vel=0.0, latched=False)
        g = dict(HOME)
        if grippers and k >= len(path) - extra_end:
            g.update(grippers)
        objs = {"cutter": st((m.BX, m.BY, m.TOP)), "cutter/carriage": st(m.knob_at(c)),
                "cutter/handle": st(m.grip_at(q)), **{s: st(p) for s, p in g.items()}}
        su = m.success(objs)
    return m.score(objs), su


def P(cs, q=UP):
    return [(c, q) for c in cs]


PASS = P(ramp(0.0, END, 40))  # one smooth pass, edge to edge, to the far stop


def snap_at(c):
    """Press the handle past the breaking angle with the carriage at travel c, then let go."""
    return P([c] * 6, q=0.3) + P([c] * 4, q=-0.02) + P([c] * 6, q=UP)


SNAP = snap_at(END)


def sawing(fwd=0.005, back=0.0029):
    """Saw across the tile: forward `fwd`, back `back` (each back under the 3 mm play), repeat to the far stop."""
    cs, c = [0.0], 0.0
    while c < END:
        c = min(c + fwd, END)
        cs.append(c)
        if c < END:
            c -= back
            cs.append(c)
    return P(cs)


CASES = [
    ("sawing back and forth across the tile (5 mm on, 2.9 mm back), then snap", sawing() + SNAP, 0.0),
    ("one pause with 2.9 mm of play mid-pass, then on and snap",
     P(ramp(0.0, END / 2, 20) + [END / 2 - 0.0029] + ramp(END / 2 - 0.0029, END, 20)) + SNAP, 1.0),
    ("nothing done", P([0.0] * 10), 0.0),
    ("snap with no score (pressing the breaker first)", P([0.0] * 5) + snap_at(0.0), 0.0),
    ("snap first, then a full pass", P([0.0] * 5) + snap_at(0.0) + PASS, 0.0),
    ("half pass, then snap", P(ramp(0.0, 0.09, 20)) + snap_at(0.09), 0.0),
    ("half pass, snap, then finish the pass", P(ramp(0.0, 0.09, 20)) + snap_at(0.09) + P(ramp(0.09, END, 20)), 0.0),
    ("half pass only", P(ramp(0.0, 0.09, 20)), 0.0),
    ("pass stopping 2 mm short of the far edge", P(ramp(0.0, m.E1 - 0.002, 30)), 0.0),
    ("one clean pass, no snap", PASS, 0.25),
    ("one clean pass with a pause half way", P(ramp(0, 0.08, 15) + [0.08] * 10 + ramp(0.08, END, 20)), 0.25),
    ("one clean pass with a 2 mm wobble back half way",
     P(ramp(0, 0.08, 15) + [0.078] * 3 + ramp(0.078, END, 20)), 0.25),
    ("pass, roll back 5 mm over the tile, finish: re-scored", P(ramp(0, 0.08, 15) + [0.075] + ramp(0.075, END, 20)), 0.0),
    ("pass, then back to the start (second pass)", PASS + P(ramp(END, 0.0, 30)), 0.0),
    ("pass, back to the start, snap", PASS + P(ramp(END, 0.0, 30)) + P([0.0] * 3, q=-0.02) + P([0.0] * 5), 0.0),
    ("pass, a second full pass, back at the far stop, snap", PASS + P(ramp(END, 0.0, 30)) + PASS + SNAP, 0.0),
    ("pass, then roll back 1 cm at the far stop (off the tile), snap", PASS + P(ramp(END, END - 0.01, 5)) +
     P(ramp(END - 0.01, END, 5)) + SNAP, 1.0),
    ("nudge 5 mm at the near end and back, then one pass, snap", P(ramp(0, 0.005, 3) + ramp(0.005, 0, 3)) + PASS + SNAP,
     1.0),
    ("clean pass, snap, handle let go: done", PASS + SNAP, 1.0),
    ("clean pass, handle pressed only part way (not snapped)", PASS + P([END] * 5, q=0.10) + P([END] * 5), 0.25),
    ("clean pass, snap, then a second pass", PASS + SNAP + P(ramp(END, 0.0, 30)), 0.0),
    ("clean pass, snap, handle still pressed at the end", PASS + P([END] * 6, q=-0.02), 0.0),
    ("clean pass, snap, handle let go but not yet back up", PASS + SNAP[:-6] + P([END] * 6, q=0.3), 0.0),
]

bad = 0
for name, path, want in CASES:
    got, _ = run(path)
    ok = abs(got - want) < 1e-9
    bad += not ok
    print(("ok  " if ok else "FAIL"), f"{got:.2f} (want {want})", name)

# Held at the end: a tool point on the grip (handle up) or on the push knob gets no credit.
grip_up = tuple(m.grip_at(UP))
knob_end = tuple(m.knob_at(END))
checks = []
sc, su = run(PASS + SNAP)
checks.append((sc == 1.0 and su, "done, arms home: 1.0 and success"))
sc, su = run(PASS + SNAP, grippers={"left_gripper": grip_up})
checks.append((sc == 0.0 and not su, "done but a gripper still on the grip: 0"))
sc, _ = run(PASS, grippers={"right_gripper": knob_end})
checks.append((sc == 0.0, "clean pass but a gripper still at the push knob: 0"))
sc, _ = run(PASS, end_vel=0.05)
checks.append((sc == 0.0, "clean pass, carriage still rolling: 0"))
away = {"right_gripper": (0.30, -0.15, 0.90)}
sc, su = run(PASS + SNAP, grippers=away)
checks.append((sc == 0.9 and not su, "done, a gripper away from home (and off the parts): 0.9"))
sc, _ = run(PASS + SNAP, grippers={"right_gripper": (0.18, -0.24, 0.95)})
checks.append((sc == 1.0, "done, a gripper 2 cm from its start: home"))
# A reset clears the history: a re-scored trial followed by a clean one.
run(PASS + P(ramp(END, 0.0, 30)))
sc, _ = run(PASS + SNAP)
checks.append((sc == 1.0, "a reset (kit.episode) clears the history"))
m.REQUIRE_ARMS_HOME = False
sc, su = run(PASS + SNAP, grippers=away)
m.REQUIRE_ARMS_HOME = True
checks.append((sc == 1.0 and su, "with REQUIRE_ARMS_HOME off, done alone is 1.0"))
# The drawing (visual only): line lengths appear as the wheel crosses them, the split replaces the tile after the snap.
vis = lambda ids: [kit.visible("cutter", i, body="base") for i in ids]  # noqa: E731
run(P(ramp(0.0, m.E0 + m.TILE_L / 2 + 0.001, 20)))
checks.append((vis(m.LINE_MESHES) == [True] * 3 + [False] * 3 and all(vis(m.WHOLE_MESHES))
               and not any(vis(m.SPLIT_MESHES)), "drawing: half pass shows half the score line, tile whole"))
run(PASS)
checks.append((all(vis(m.LINE_MESHES)) and not any(vis(m.SPLIT_MESHES)), "drawing: full pass shows the whole line"))
run(PASS + SNAP)
checks.append((not any(vis(m.WHOLE_MESHES + m.LINE_MESHES)) and all(vis(m.SPLIT_MESHES)),
               "drawing: after the snap, two halves and the gap"))
run(P([0.0] * 5))
checks.append((not any(vis(m.LINE_MESHES + m.SPLIT_MESHES)) and all(vis(m.WHOLE_MESHES)),
               "drawing: a reset shows a whole, unscored tile again"))
for flag, name in checks:
    bad += not flag
    print(("ok  " if flag else "FAIL"), name)
print("all pass" if not bad else f"{bad} FAILED")
raise SystemExit(1 if bad else 0)
