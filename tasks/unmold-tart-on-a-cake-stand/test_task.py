"""Offline checks of the tart-unmolding scorer: step-by-step poses, no simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import math
import os

import numpy as np
from yam_arena.constants import PAD_TIP_PAST_TCP as PAD_TIP
from yam_arena.demo import grip_rotation
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
HOME = {"left_gripper": (0.20, 0.27, 0.95), "right_gripper": (0.20, -0.27, 0.95)}
TOP = m.TOP
STAND = np.array([*m.STAND_XY, TOP])
STAND_TOP = TOP + m.STAND_H
RING0 = np.array(m.RING_POS)
SEAT = m.LEDGE_H + m.BASE_T / 2  # base center above the ring's bottom when the ring holds it
TART_UP = (m.BASE_T + m.TART_H) / 2  # tart center above the base center
HELD = 0.05  # opening of fingers pinching the 4 mm lip


def quat(deg_x=0.0):
    a = math.radians(deg_x) / 2
    return np.array([math.sin(a), 0.0, 0.0, math.cos(a)])


def st(pos, deg=0.0, half=(0.12, 0.12, 0.013), speed=0.0, q=None):
    p = np.array(pos, float)
    h = np.array(half)
    return ObjectState(pos=p, quat_xyzw=quat(deg) if q is None else np.asarray(q, float),
                       lin_vel=np.array([speed, 0, 0]), ang_vel=np.zeros(3), low=p - h, high=p + h)


def mat_quat(R):
    """Rotation matrix -> quaternion (x, y, z, w)."""
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return np.array([x, y, z, w])


DOWN = mat_quat(grip_rotation(0.0))  # a straight-down grip: fingers pointing at the table
SIDE = {g: mat_quat(kit._rot_xyz(-s * (math.pi / 2 - m.TILT), 0.0) @ grip_rotation(0.0))  # the demo's tilted pinch
        for g, s in (("left_gripper", 1.0), ("right_gripper", -1.0))}


def frame(ring, base=None, tart=None, ring_deg=0.0, base_deg=0.0, tart_deg=None, hands="home", opening=1.0,
          grip=None):
    """Ring bottom center at `ring`; base and tart default to riding in the ring. hands: "home", "lip" (both tool
    points where the demo pinches the lip, with its tilted grip) or a dict of tool points (a straight-down grip) or of
    (tool point, quaternion)."""
    ring = np.asarray(ring, float)
    base = ring + [0, 0, SEAT] if base is None else np.asarray(base, float)
    tart = base + [0, 0, TART_UP] if tart is None else np.asarray(tart, float)
    tart_deg = base_deg if tart_deg is None else tart_deg
    if hands == "home":
        g, op = dict(HOME), {k: 1.0 for k in HOME}
    elif hands == "lip":
        g = {"left_gripper": (ring + [0, m.GRIP_Y, m.LIP_Z + m.GRIP_DZ], SIDE["left_gripper"]),
             "right_gripper": (ring + [0, -m.GRIP_Y, m.LIP_Z + m.GRIP_DZ], SIDE["right_gripper"])}
        op = {k: opening for k in g}
    else:
        g = {**HOME, **hands}
        op = {k: (opening if k in hands else 1.0) for k in g}
    objs = {"_opening": op, "stand": st(STAND),
            "ring": st(ring, ring_deg), "base": st(base, base_deg), "tart": st(tart, tart_deg)}
    for n, p in g.items():
        pos, q = p if isinstance(p, tuple) and len(p) == 2 and np.ndim(p[1]) == 1 else (p, DOWN)
        objs[n] = st(pos, half=(0.01, 0.01, 0.01), q=q)
    return objs


def run(frames):
    """frames: list of (steps, objects). A fresh trial each time (the kit's reset counter moves on)."""
    kit._EPISODE["count"] += 1
    kit._EPISODE["last"] = None
    out = (0.0, False)
    for n, objs in [(2, frame(RING0))] + frames:  # each trial starts with the pan on the table and the arms home
        kit._OPENING.update(objs["_opening"])
        objs = {k: v for k, v in objs.items() if k != "_opening"}
        for _ in range(n):
            objs = dict(objs)  # a new dict each step: offline, the kit tells steps apart by the objects dict
            ok = m.success(objs)
            out = (m.score(objs), ok)
    return out


def lerp(a, b, s):
    return np.asarray(a, float) + s * (np.asarray(b, float) - np.asarray(a, float))


ON_STAND_RING = np.array([*m.STAND_XY, STAND_TOP - SEAT + m.BASE_T / 2])  # ring bottom when the base just meets the stand
BASE_ON = np.array([*m.STAND_XY, STAND_TOP + m.BASE_T / 2])
RING_DOWN = np.array([*m.STAND_XY, TOP + 0.0005])
UP = np.array([*m.PAN_XY, TOP + m.LIFT])
OVER = np.array([*m.STAND_XY, TOP + m.LIFT])


def move(a, b, steps, **kw):
    return [(1, frame(lerp(a, b, (k + 1) / steps), **kw)) for k in range(steps)]


def lower(steps=60, tilt=lambda s: 0.0, **kw):
    """From over the stand down to the table: the base stops on the stand, the ring goes on."""
    out = []
    for k in range(steps):
        s = (k + 1) / steps
        r = lerp(OVER, RING_DOWN, s)
        base = r + [0, 0, SEAT] if r[2] + SEAT >= BASE_ON[2] else BASE_ON
        out.append((1, frame(r, base, ring_deg=tilt(s), hands="lip", opening=HELD, **kw)))
    return out


def unmolded(hands="home", opening=1.0, **kw):
    return frame(RING_DOWN, BASE_ON, hands=hands, opening=opening, **kw)


CARRY = ([(10, frame(RING0, hands="lip", opening=HELD))] + move(RING0, UP, 30, hands="lip", opening=HELD)
         + move(UP, OVER, 40, hands="lip", opening=HELD))
OUT = {"left_gripper": (RING_DOWN + [0, m.OUT_Y, m.LIP_Z + m.GRIP_DZ], SIDE["left_gripper"]),
       "right_gripper": (RING_DOWN + [0, -m.OUT_Y, m.LIP_Z + m.GRIP_DZ], SIDE["right_gripper"])}
DEMO = CARRY + lower() + [(5, unmolded(hands="lip", opening=0.3)), (5, unmolded(hands=OUT, opening=0.3))]


def gap(objs, g, name, parts):
    kit._OPENING.update(objs["_opening"])
    return m.gk.hand_gap({k: v for k, v in objs.items() if k != "_opening"}, g, name, parts)


def place(g, q, opening, start, direction, name, parts, want):
    """The end state with gripper g (orientation q, opened `opening`) moved from `start` along `direction` until its
    own shape is `want` m from `name`'s parts (0: just touching), found by halving to 0.05 mm."""
    at = lambda s: unmolded(hands={g: (np.asarray(start, float) + np.asarray(direction, float) * s, q)},  # noqa: E731
                            opening=opening)
    lo, hi = 0.0, 0.06
    assert gap(at(lo), g, name, parts) > want >= gap(at(hi), g, name, parts) - 1e-9
    while hi - lo > 0.00005:
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if gap(at(mid), g, name, parts) > want else (lo, mid)
    return at(hi)


def check(name, got, want):
    ok = abs(got - want) < 1e-9
    print(f"{'ok  ' if ok else 'FAIL'} {name}: {got} (want {want})")
    return ok


results = []
c = lambda name, frames, want: results.append(check(name, run(frames)[0], want))  # noqa: E731

# The demo path and the end states.
c("demo path, arms home", DEMO + [(20, unmolded())], 1.0)
results.append(run(DEMO + [(30, unmolded())])[1] is True)
c("demo path, arms left out of the way, not home (arm position is not scored)",
  DEMO + [(20, unmolded(hands={"left_gripper": (0.3, 0.3, 0.95)}))], 1.0)
results.append(run(DEMO + [(30, unmolded(hands={"left_gripper": (0.3, 0.3, 0.95)}))])[1] is True)
c("do nothing", [(100, frame(RING0))], 0.0)
c("still pinching the lip at the end", DEMO + [(20, unmolded(hands="lip", opening=HELD))], 0.25)
c("empty closed fist resting at the lip at the end", DEMO + [(20, unmolded(hands="lip", opening=0.0))], 0.25)
LET_GO = 0.2556  # the demo's let-go opening: fingers 2.4 cm apart around the 4 mm lip
ring_in = lambda want, op=LET_GO: place("left_gripper", SIDE["left_gripper"], op,  # noqa: E731
                                       RING_DOWN + [0, m.OUT_Y + 0.03, m.LIP_Z + m.GRIP_DZ], (0, -1, 0), "ring",
                                       m.RING, want)
c("open hand resting against the ring's lip at the end", DEMO + [(20, ring_in(0.0))], 0.25)
c("closed hand pushed in against the ring at the end", DEMO + [(20, ring_in(0.0, 0.0))], 0.25)
c("open hand 5 mm from the ring's lip at the end, not touching (the old 1 cm zone gave 0.25)",
  DEMO + [(20, ring_in(0.005))], 1.0)
c("open hand 3 mm from the ring's lip at the end (1 mm past the 2 mm contact tolerance), not touching", DEMO + [(20, ring_in(0.001 + m.gk.CONTACT))], 1.0)
c("whole pan set on the stand, ring still on, let go",
  CARRY + move(OVER, ON_STAND_RING, 20, hands="lip", opening=HELD) + [(20, frame(ON_STAND_RING))], 0.25)
c("whole pan lifted and set back down on the table",
  CARRY + move(OVER, [m.STAND_XY[0], 0.0, TOP + 0.0005], 20, hands="lip", opening=HELD)
  + [(20, frame([m.STAND_XY[0], 0.0, TOP + 0.0005]))], 0.0)
c("whole pan set back down where it was", CARRY + move(OVER, RING0, 40, hands="lip", opening=HELD) + [(20, frame(RING0))], 0.0)
off = BASE_ON + [0.04, 0, 0]
c("base 4 cm off center on the stand (level)", CARRY + lower() + [(20, frame(RING_DOWN, off))], 0.0)
c("base off center, tart tipped over", CARRY + lower() + [(5, frame(RING_DOWN, off, base_deg=30.0))]
  + [(20, frame(RING_DOWN, BASE_ON))], 0.0)
c("tart slid off its base onto the table, base on the stand",
  CARRY + lower() + [(20, frame(RING_DOWN, BASE_ON, tart=[m.STAND_XY[0] + 0.15, 0, TOP + m.TART_H / 2]))], 0.0)
c("tart tipped 25 degrees for 2 steps, then level again", CARRY + [(2, frame(OVER, base_deg=25.0))] + lower()
  + [(20, unmolded())], 0.0)
c("one-step physics pop of the tart's tilt", CARRY + [(1, frame(OVER, base_deg=25.0))] + lower() + [(20, unmolded())],
  1.0)
# The ring cocked while it slides down past the crust.
cock = lambda s: 12.0 if 0.60 < s < 0.70 else 0.0  # noqa: E731  (the ring slides past the base for s 0.59 to 0.72)
c("ring cocked 12 degrees while sliding off (one arm lowering), ends flat", CARRY + lower(tilt=cock)
  + [(20, unmolded())], 0.5)
flick = lambda s: 12.0 if 0.64 < s < 0.657 else 0.0  # noqa: E731
c("ring cocked for only 2 steps, then level", CARRY + lower(120, tilt=flick) + [(20, unmolded())], 0.5)
c("ring tilted 5 degrees while sliding (within 6)", CARRY + lower(tilt=lambda s: 5.0) + [(20, unmolded())], 1.0)
c("whole pan tilted 15 degrees while carried (not sliding)", CARRY + [(10, frame(OVER, ring_deg=15.0, base_deg=15.0,
                                                                                 hands="lip", opening=HELD))]
  + lower() + [(20, unmolded())], 1.0)
c("fast correct drop: the ring falls clear in 2 steps", CARRY + lower(4) + [(20, unmolded())], 1.0)
yank = [(1, frame(ON_STAND_RING, BASE_ON, ring_deg=12.0, hands="lip", opening=HELD)),
        (1, frame(RING_DOWN + [0, 0, 0.02], BASE_ON, ring_deg=12.0, hands="lip", opening=HELD))]
c("ring yanked down cocked past the whole crust within one step", CARRY + move(OVER, ON_STAND_RING, 10, hands="lip",
                                                                               opening=HELD)
  + yank + [(20, unmolded())], 0.5)
c("ring yanked down level within one step", CARRY + move(OVER, ON_STAND_RING, 10, hands="lip", opening=HELD)
  + [(1, frame(ON_STAND_RING, BASE_ON, hands="lip", opening=HELD)), (1, frame(RING_DOWN, BASE_ON, hands="lip",
                                                                          opening=HELD))] + [(20, unmolded())], 1.0)
ON_STAND = CARRY + move(OVER, ON_STAND_RING, 10, hands="lip", opening=HELD)
mid, past = ON_STAND_RING - [0, 0, 0.015], ON_STAND_RING - [0, 0, 0.04]
c("ring cocked 30 degrees with exactly one sample inside the sliding stretch", ON_STAND
  + [(1, frame(mid, BASE_ON, ring_deg=30.0, hands="lip", opening=HELD)),
     (1, frame(past, BASE_ON, ring_deg=30.0, hands="lip", opening=HELD))] + [(20, unmolded())], 0.5)
c("ring lowered cocked 25 degrees at 30 cm/s (1.5 cm a step)", ON_STAND
  + [(1, frame(np.maximum(ON_STAND_RING - [0, 0, 0.015 * (k + 1)], RING_DOWN), BASE_ON, ring_deg=25.0, hands="lip",
               opening=HELD)) for k in range(8)] + [(20, unmolded())], 0.5)
c("one-step tilt pop at 30 degrees inside the sliding stretch", ON_STAND
  + [(1, frame(ON_STAND_RING - [0, 0, 0.005 * (k + 1)], BASE_ON, ring_deg=30.0 if k == 2 else 0.0, hands="lip",
               opening=HELD)) for k in range(8)] + [(20, unmolded())], 1.0)
c("ring left leaning on the stand, not flat", CARRY + lower() + [(20, frame(RING_DOWN + [0, 0, 0.02], BASE_ON,
                                                                          ring_deg=12.0))], 0.0)
c("ring flat on the table but beside the stand", CARRY + lower() + [(20, frame(RING_DOWN + [0.08, 0, 0], BASE_ON))],
  0.0)
# Fingers on the tart: contact by the gripper's own shape, not a zone over it.
tart_top = lambda up: {"left_gripper": BASE_ON + [0.0, 0.05, TART_UP + m.TART_H / 2 + PAD_TIP + up]}  # noqa: E731
c("closed fingertips resting on the tart's top for 2 steps, otherwise the demo",
  CARRY + lower() + [(2, unmolded(hands=tart_top(0.0), opening=0.0))] + [(20, unmolded())], 0.0)
c("fingertips pressed 3 mm into the tart's top for 2 steps",
  CARRY + lower() + [(2, unmolded(hands=tart_top(-0.003), opening=0.0))] + [(20, unmolded())], 0.0)
c("open hand's fingertips resting on the tart's top for 2 steps",
  CARRY + lower() + [(2, unmolded(hands=tart_top(0.0), opening=1.0))] + [(20, unmolded())], 0.0)
c("fingertips touching the tart's top for 1 step only (a physics pop)",
  CARRY + lower() + [(1, unmolded(hands=tart_top(0.0), opening=0.0))] + [(20, unmolded())], 1.0)
c("open hand's fingertips 5 mm over the tart for 20 steps, not touching (the old zone scored 0)",
  CARRY + lower() + [(20, unmolded(hands=tart_top(0.005), opening=1.0))] + [(20, unmolded())], 1.0)
c("closed fingertips 3 mm over the tart for 20 steps, not touching",
  CARRY + lower() + [(20, unmolded(hands=tart_top(0.003), opening=0.0))] + [(20, unmolded())], 1.0)
side_tart = lambda want: place("left_gripper", SIDE["left_gripper"], LET_GO,  # noqa: E731
                              BASE_ON + [0, m.TART_R + 0.05, TART_UP], (0, -1, 0), "tart", m.TART, want)
c("open hand's fingers against the tart's side for 2 steps", CARRY + lower() + [(2, side_tart(0.0))]
  + [(20, unmolded())], 0.0)
c("open hand 4 mm from the tart's side for 20 steps, not touching", CARRY + lower() + [(20, side_tart(0.004))]
  + [(20, unmolded())], 1.0)
c("gripping the tart's edge to carry the pan",
  [(10, frame(RING0, hands={"left_gripper": RING0 + [0, m.TART_R - 0.005, SEAT + TART_UP],
                            "right_gripper": RING0 + [0, -m.TART_R + 0.005, SEAT + TART_UP]}, opening=0.2))]
  + CARRY + lower() + [(20, unmolded())], 0.0)
# The sim's grab variant: a tilted pinch from outside whose pad tip (1.3 cm in from the tool point, toward -y) is under
# the crust's bottom edge, at base level.
pinch = unmolded()
tip_at = BASE_ON + [0, m.TART_R - 0.004, m.BASE_T / 2 - 0.001]
pinch["left_gripper"] = st(tip_at + [0, 0.013, 0], 90.0, half=(0.01, 0.01, 0.01))
pinch["_opening"] = {**pinch["_opening"], "left_gripper": 0.04}
c("pad tip pinching the crust's bottom edge for 2 steps (sim grab variant)", CARRY + lower() + [(2, pinch)]
  + [(20, unmolded())], 0.0)
c("tart shoved 1 cm across its base by a hand's side (sim grab variant)", CARRY + lower()
  + [(20, frame(RING_DOWN, BASE_ON, tart=BASE_ON + [0, -0.01, TART_UP]))], 0.0)
c("tart 4 mm off center on its base is still fine", CARRY + lower()
  + [(20, frame(RING_DOWN, BASE_ON, tart=BASE_ON + [0, -0.004, TART_UP]))], 1.0)
c("pinching the lip is not a finger in the tart", CARRY + lower() + [(20, unmolded())], 1.0)
c("hand 2 cm above the tart is not on it", CARRY + lower()
  + [(5, unmolded(hands=tart_top(0.02), opening=1.0))] + [(20, unmolded())], 1.0)
# A new trial starts afresh (latches cleared by kit.episode()).
run(CARRY + [(3, frame(OVER, base_deg=30.0))])
c("reset after a ruined trial", DEMO + [(20, unmolded())], 1.0)
# The numbers the text states.
results.append(m.COCK_DEG == 6.0 and m.TIP_DEG == 20.0 and m.LEVEL_DEG == 5.0 and m.CENTER == 0.03
               and m.ON == 0.005 and m.gk.CONTACT == 0.002)
# The test poses touch (or clear) what they say they do.
results.append(gap(ring_in(0.0), "left_gripper", "ring", m.RING) <= m.gk.CONTACT)
results.append(abs(gap(ring_in(0.005), "left_gripper", "ring", m.RING) - 0.005) < 0.0006)
results.append(gap(ring_in(0.0), "left_gripper", "tart", m.TART) > 0.01)
results.append(gap(unmolded(hands=tart_top(0.005), opening=1.0), "left_gripper", "tart", m.TART) > 0.0045)
results.append(gap(unmolded(hands="lip", opening=HELD), "left_gripper", "tart", m.TART) > m.gk.CONTACT)
results.append(abs(2 * m.WALL_R - 0.24) < 1e-9 and abs(2 * m.STAND_R - 0.11) < 1e-9 and m.STAND_H == 0.07)
lean = m.WALL_H * math.sin(math.radians(m.COCK_DEG)) * 1000
results.append(abs(lean - 2.7) < 0.05 and lean > m.FLUTE_D * 1000)  # "leans the wall top 2.7 mm, past the 2 mm flutes"
results.append(abs(m.BASE_R * math.sin(math.radians(m.LEVEL_DEG)) - 0.010) < 0.0005)  # "5 degrees: 1 cm at its edge"
total = m.RING_MASS + m.BASE_MASS + m.TART_MASS
results.append(abs(total - 0.70) < 1e-9)

print("all pass" if all(results) else f"FAILED {results.count(False)}")
