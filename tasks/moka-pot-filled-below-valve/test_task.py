"""Offline checks of the moka pot scorer: hand-made step-by-step states of the pot, the carafe and the grippers, no
simulator. The water in the base is set directly (pourkit's pour model is tested by its pilot, acid-into-water).

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit, pk = m.kit, m.pk
HOME = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
BASE = np.array((m.BX, m.BY, m.FLOOR_Z))
RIM = BASE + (0, 0, m.RIM)
TOP0 = np.array((*m.TOP_XY, m.TOP + m.COLLAR_H + 0.0005))
GRIP_OPEN = 0.58  # fingers closed on the top's body (as read in the sim)


def q_yaw(deg, tilt=0.0):
    """Yaw about z (degrees), then optionally tilted about x."""
    a, t = math.radians(deg) / 2, math.radians(tilt) / 2
    qz = np.array([0.0, 0.0, math.sin(a), math.cos(a)])
    qx = np.array([math.sin(t), 0.0, 0.0, math.cos(t)])
    x1, y1, z1, w1 = qz
    x2, y2, z2, w2 = qx
    return np.array([w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2, w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
                     w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2, w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2])


def st(pos, yaw=0.0, tilt=0.0, half=0.02):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=q_yaw(yaw, tilt), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p - half,
                       high=p + half)


def frame(top=None, top_yaw=0.0, top_tilt=0.0, jug=None, left=None, left_yaw=0.0, left_open=1.0, right=None,
          right_yaw=0.0, right_open=1.0):
    g = {"left_gripper": (left if left is not None else HOME["left_gripper"], left_yaw, left_open),
         "right_gripper": (right if right is not None else HOME["right_gripper"], right_yaw, right_open)}
    objs = {"stove": st((m.BX, m.BY, m.TOP)), "base": st(BASE),
            "top": st(TOP0 if top is None else top, top_yaw, top_tilt),
            "jug": st(m.JUG_POS if jug is None else jug)}
    objs.update({k: st(p, y) for k, (p, y, _) in g.items()})
    return objs, {k: o for k, (_, _, o) in g.items()}


def run(frames, water=0.0, spill=0.0):
    """frames: list of (steps, (objects, openings)). A fresh trial each time (the kit's reset counter moves on); the
    base is given `water` mL and the table `spill` mL after the first step."""
    kit._EPISODE["count"] += 1
    out = (0.0, False)
    first = True
    for n, (objs, opens) in [(2, frame())] + frames:
        kit._OPENING.update(opens)
        for _ in range(n):
            pk.update(objs)
            if first:
                pk._S.contents["base"] = {"water": water} if water else {}
                pk._S.contents["jug"] = {"water": max(m.JUG0 - water - spill, 0.0)}
                pk._S.spilled = {"water": spill} if spill else {}
                first = False
            ok = m.success(objs)
            out = (m.score(objs), ok)
    return out


GRIP_Z = RIM[2] + m.GRIP_DZ


def place(yaw=0.0, holder="left"):
    """Carry the top over and set it on the rim, held."""
    g = (RIM[0], RIM[1], GRIP_Z)
    return [(3, frame(top=RIM + (0, 0, 0.03), top_yaw=yaw, **{holder: g + np.array((0, 0, 0.03)),
                                                              f"{holder}_open": GRIP_OPEN}))] + \
           [(3, frame(top=RIM, top_yaw=yaw, **{holder: g, f"{holder}_open": GRIP_OPEN}))]


def screw(deg, start=0.0, holder="left", grip_turns=True, other=None, step=10.0, open_=None, tilt=0.0, hover=None):
    """Turn the seated top clockwise (seen from above) by `deg` in `step`-degree steps. With `grip_turns` the holder's
    wrist rolls with it (regrasping every 90 degrees, as the demo does); otherwise the holder's wrist stays put (or
    wiggles +-10 degrees with `hover="wiggle"`) while something else turns the top."""
    f, yaw, w0 = [], start, 0.0
    g = (RIM[0], RIM[1], GRIP_Z)
    op = GRIP_OPEN if open_ is None else open_
    done = 0.0
    while done < deg - 1e-9:
        s = min(step, deg - done)
        done += s
        yaw -= s
        if grip_turns:
            w = w0 - (done % 90.0 if done % 90.0 else 90.0)
        elif hover == "wiggle":
            w = 10.0 if int(done / step) % 2 else -10.0
        else:
            w = 0.0
        kw = {holder: g, f"{holder}_open": op, f"{holder}_yaw": w}
        if other:
            kw.update(other)
        f.append((1, frame(top=RIM, top_yaw=yaw, top_tilt=tilt, **kw)))
        if grip_turns and done % 90.0 == 0:  # let go, roll back open, grip again
            f.append((1, frame(top=RIM, top_yaw=yaw, top_tilt=tilt, **{holder: g, f"{holder}_open": 1.0,
                                                                         f"{holder}_yaw": w})))
            f.append((1, frame(top=RIM, top_yaw=yaw, top_tilt=tilt, **{holder: g, f"{holder}_open": 1.0,
                                                                         f"{holder}_yaw": w0})))
            f.append((1, frame(top=RIM, top_yaw=yaw, top_tilt=tilt, **{holder: g, f"{holder}_open": op,
                                                                         f"{holder}_yaw": w0})))
    return f, yaw


def let_go(yaw, home=True, tilt=0.0):
    return [(3, frame(top=RIM, top_yaw=yaw, top_tilt=tilt, left=(RIM[0], RIM[1], GRIP_Z + 0.05)))] + \
           ([(5, frame(top=RIM, top_yaw=yaw, top_tilt=tilt))] if home else [])


GOOD = 140.0


def good(deg=585.0, water=GOOD, spill=0.0, home=True):
    f, yaw = screw(deg)
    return run(place() + f + let_go(yaw, home), water=water, spill=spill)


def check(name, got, want):
    assert abs(got[0] - want) < 1e-9, f"{name}: score {got[0]}, wanted {want}"
    print(f"ok  {name}: {got[0]}")


def high_grip_screw(deg_per_stroke=90.0, strokes=7, opening=0.83):
    """Screwed on by the wider upper body (tool point 6.5 cm up, fingers at 0.83 for its 8 cm), regripping each stroke."""
    g, f, yaw = (RIM[0], RIM[1], RIM[2] + 0.065), [], 0.0
    for _ in range(strokes):
        for k in range(1, int(deg_per_stroke / 10) + 1):
            yaw -= 10
            f.append((1, frame(top=RIM, top_yaw=yaw, left=g, left_open=opening, left_yaw=-10 * k)))
        f.append((1, frame(top=RIM, top_yaw=yaw, left=g, left_open=1.0, left_yaw=-deg_per_stroke)))
        f.append((1, frame(top=RIM, top_yaw=yaw, left=g, left_open=1.0, left_yaw=0)))
    return run(place() + f + let_go(yaw), water=GOOD)


def main():
    check("full job", good(), 1.0)
    # review, Nov 10
    check("screwed on gripping the upper body (630 degrees)", high_grip_screw(), 1.0)
    f2, y2 = screw(585.0)
    check("job done, left hand left clamped on the upper body", run(place() + f2 + [(10, frame(
        top=RIM, top_yaw=y2, left=(RIM[0], RIM[1], RIM[2] + 0.065), left_open=0.83))], water=GOOD), 0.5)
    check("job done, left hand left closed on the lid knob", run(place() + f2 + [(10, frame(
        top=RIM, top_yaw=y2, left=(RIM[0], RIM[1], RIM[2] + m.T_H + 0.005), left_open=0.17))], water=GOOD), 0.5)
    assert good()[1], "success() should be true"
    check("do nothing", run([]), 0.0)
    check("top screwed on an empty base", good(water=0.0), 0.0)
    check("water above the valve", good(water=m.VALVE_ML + 2), 0.0)
    check("carafe emptied in (175 mL)", good(water=m.JUG0), 0.0)
    check("weak: 1.5 cm under the valve", good(water=m.ml_below(m.TOP_LINE_Z - 0.015)), 0.25)
    check("just under the valve", good(water=m.VALVE_ML - 1), 1.0)
    check("just above the low line", good(water=m.LOW_ML + 1), 1.0)
    check("filled, top left off", run([], water=GOOD), 0.25)
    check("a dribble poured in, top left off", run([], water=60.0), 0.0)
    check("weak fill, top left off", run([], water=m.LOW_ML - 1), 0.0)
    check("filled, top set on but not turned", run(place() + let_go(0.0), water=GOOD), 0.25)
    check("loose: a turn", good(deg=360.0), 0.25)
    check("snug band low end", good(deg=460.0), 1.0)
    check("over-turned: 2.5 turns (not biting)", good(deg=900.0), 0.25)
    check("spilled 10 mL", good(spill=10.0), 0.5)
    check("arms not home", good(home=False), 0.9)
    # an empty closed fist resting on the top while it is swiped round earns nothing
    f, yaw = screw(585.0, grip_turns=True, open_=0.05)
    check("empty fist rolling on the top", run(place() + f + let_go(yaw), water=GOOD), 0.25)
    # screwed 585 by hand, then dragged back 200 degrees by an open hand: looser by that much
    f, yaw = screw(585.0)
    drag = [(1, frame(top=RIM, top_yaw=yaw + 10.0 * k, right=(RIM[0], RIM[1] - 0.04, GRIP_Z), right_open=1.0))
            for k in range(1, 21)]
    check("screwed, then dragged back 200 unheld", run(place() + f + drag + let_go(yaw + 200.0), water=GOOD), 0.25)
    # still holding the pot at the end
    f, yaw = screw(585.0)
    held = [(5, frame(top=RIM, top_yaw=yaw, left=(RIM[0], RIM[1], GRIP_Z), left_open=GRIP_OPEN, left_yaw=-45.0))]
    check("pot left held", run(place() + f + held, water=GOOD), 0.5)
    # carafe left on the stove plate, or still held
    f, yaw = screw(585.0)
    end = let_go(yaw)[:1] + [(5, frame(top=RIM, top_yaw=yaw, jug=(m.BX + 0.05, m.BY - 0.05, m.TOP + m.JUG_H / 2)))]
    check("carafe left on the stove", run(place() + f + end, water=GOOD), 0.5)
    # twist holes: the other arm swipes the top round while the left hand hovers open around it
    swipe = {"right": (RIM[0], RIM[1] - 0.04, GRIP_Z), "right_open": 0.0}
    f, yaw = screw(585.0, grip_turns=False, open_=1.0, other=swipe)
    check("swiped while a hand hovers open", run(place() + f + let_go(yaw), water=GOOD), 0.25)
    # ... or while the left hand is closed on it but only wiggles its wrist +-10 degrees
    f, yaw = screw(585.0, grip_turns=False, other=swipe, hover="wiggle")
    check("swiped while a closed hand wiggles", run(place() + f + let_go(yaw), water=GOOD), 0.25)
    f, yaw = screw(1500.0, grip_turns=False, other=swipe, hover="wiggle")
    check("swiped 4 turns while a closed hand wiggles", run(place() + f + let_go(yaw), water=GOOD), 0.25)
    # ... or while the closed hand stays still
    f, yaw = screw(585.0, grip_turns=False, other=swipe)
    check("swiped under a still closed hand", run(place() + f + let_go(yaw), water=GOOD), 0.25)
    # screwed, then lifted off and set back without turning: the count starts again
    f, yaw = screw(585.0)
    off = [(3, frame(top=RIM + (0, 0, 0.02), top_yaw=yaw, left=(RIM[0], RIM[1], GRIP_Z + 0.02),
                     left_open=GRIP_OPEN))]
    check("screwed, lifted off, set back", run(place() + f + off + place(yaw) + let_go(yaw), water=GOOD), 0.25)
    # cross-threaded: the top crooked 6 degrees on the rim while it is turned
    f, yaw = screw(585.0, tilt=6.0)
    check("cross-threaded (6 degrees crooked)", run(place() + f + let_go(yaw, tilt=6.0), water=GOOD), 0.25)
    # unscrewing takes the credit back: on 585, back 360
    f, yaw = screw(585.0)
    back = []
    for k in range(1, 37):
        w = (10.0 * k) % 90.0
        back.append((1, frame(top=RIM, top_yaw=yaw + 10.0 * k, left=(RIM[0], RIM[1], GRIP_Z), left_open=GRIP_OPEN,
                              left_yaw=w if w else 90.0)))
        if w == 0:
            back += [(1, frame(top=RIM, top_yaw=yaw + 10.0 * k, left=(RIM[0], RIM[1], GRIP_Z), left_yaw=90.0)),
                     (1, frame(top=RIM, top_yaw=yaw + 10.0 * k, left=(RIM[0], RIM[1], GRIP_Z), left_yaw=0.0))]
    check("screwed 585 then backed off 360", run(place() + f + back + let_go(yaw + 360.0), water=GOOD), 0.25)
    # held to the end while the score is taken: no credit for the job until it is let go
    f, yaw = screw(585.0)
    held = [(20, frame(top=RIM, top_yaw=yaw, left=(RIM[0], RIM[1], GRIP_Z), left_open=GRIP_OPEN, left_yaw=0.0))]
    check("job done but the top still gripped at the end", run(place() + f + held, water=GOOD), 0.5)
    # the carafe still in the right hand at the end
    f, yaw = screw(585.0)
    end = let_go(yaw)[:1] + [(5, frame(top=RIM, top_yaw=yaw, jug=(0.23, -0.20, m.TOP + 0.10),
                                         right=(0.23, -0.20, m.TOP + 0.10), right_open=0.3))]
    check("carafe still held at the end", run(place() + f + end, water=GOOD), 0.5)
    # the top knocked off the base after being screwed (lying beside it): the count is gone
    f, yaw = screw(585.0)
    knocked = [(5, frame(top=RIM + (0.09, 0.0, -0.05), top_yaw=yaw, top_tilt=80.0))]
    check("screwed, then knocked off", run(place() + f + let_go(yaw)[:1] + knocked, water=GOOD), 0.25)
    # the top only hovering just over the collar at the end (not down on the rim)
    f, yaw = screw(585.0)
    hover = [(5, frame(top=RIM + (0, 0, 0.008), top_yaw=yaw))]
    check("screwed, then left raised 8 mm", run(place() + f + let_go(yaw)[:1] + hover, water=GOOD), 0.25)
    # reset: a full job, then a new trial where nothing happens
    assert good()[0] == 1.0
    check("reset after a full job, then nothing", run([]), 0.0)
    print("all pass")


if __name__ == "__main__":
    main()
