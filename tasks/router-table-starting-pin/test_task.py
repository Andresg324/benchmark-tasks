"""Offline checks of the router-table scorer: step-by-step joint histories, near misses included, no simulator.

    python test_task.py        (with the yam_arena package importable; prints "all pass")
"""

import math
import os

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = os.path.dirname(os.path.abspath(__file__))
m = load(os.path.join(HERE, "task.py")).module
kit = m.kit
PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
TH0 = m.BRACE_TURN  # the turn the board is braced at
PHI0 = math.atan2(*(m.BIT - m.PIN - m.C_L)[::-1])  # where the pivot puts the bearing on the curve


DOWN = (1.0, 0.0, 0.0, 0.0)  # a gripper pointing straight down (its pad tips below the tool point)


def st(pos, tip=0.0, q=(0, 0, 0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - np.array([0.01, 0.01, tip]), high=p + 0.01)


class Trial:
    """One trial's joints; each call to step() is one control step through the kit's record and the scorer."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.j = {"board_turn": m.START[2], "board_x": m.START[0], "board": m.START[1]}

    def step(self, left=None, right=None, opening=0.0, **j):
        self.j.update(j)
        g = dict(PARK)
        if left is not None:
            g["left_gripper"] = left
        if right is not None:
            g["right_gripper"] = right
        for k in g:
            kit._OPENING[k] = opening  # closed, as in the demo, unless given
        objs = {"router_table": st((m.PIN[0], m.PIN[1], m.TOP)), "board": board(self.j),
                **{k: st(p, m.PAD_TIP_PAST_TCP, DOWN) for k, p in g.items()}}
        ok = m.success(objs)
        return m.score(objs), ok

    def press(self):
        """The pads on the board now, at the middle of its back edge (clear of the 3 in zone however it lies)."""
        th, s = self.j["board_turn"], np.array([self.j["board_x"], self.j["board"]])
        return m._point_at(th, s, (m.BACK + 0.005, float(m.C_L[1])))

    def move(self, n=10, press=True, **target):
        """Joints moved linearly to `target` over n steps, the pads on the board throughout."""
        a = dict(self.j)
        out = None
        for k in range(1, n + 1):
            self.j.update({key: a[key] + (v - a[key]) * k / n for key, v in target.items()})
            out = self.step(left=self.press() if press else None)
        return out

    def slides_for(self, world_shift, turn=None):
        """Slides that move the board by a world shift from where it is (at turn `turn`)."""
        th = self.j["board_turn"] if turn is None else turn
        return np.array([self.j["board_x"], self.j["board"]]) + m._rot(-th) @ np.asarray(world_shift, float)

    def rest(self, n=25, **kw):
        out = None
        for _ in range(n):
            out = self.step(**kw)
        return out


def board(j, up=0.0):
    """The board's body state for joints-style pose j (turn, slides), lying flat (or `up` m higher)."""
    th = j["board_turn"]
    c = m.to_world(j, m.CENTRE)
    return st((c[0], c[1], m.TOP + m.GAP + m.T / 2 + up), q=(0.0, 0.0, math.sin(th / 2), math.cos(th / 2)))


def check(got, want, what):
    if abs(got - want) > 1e-9:
        raise AssertionError(f"{what}: got {got}, want {want}")
    print(f"ok   {what}: {got}")


def brace(t):
    """Slide the board from its start to 5 mm off the pin, then straight onto it (pure slide, start turn)."""
    out = -m.C_L / np.linalg.norm(m.C_L)  # from the curve's center to the pin, braced at slides (0, 0)
    off = -0.005 * out
    t.rest(2)
    t.move(12, board_turn=TH0, board_x=float(off[0]), board=float(off[1]))
    t.move(6, board_x=0.0, board=0.0)


def pivot(t, to=0.0):
    t.move(14, board_turn=to)


def feed(t, phi_end=-m.ALPHA - math.radians(4.0), n=60, into=0.001):
    for phi in np.linspace(PHI0, phi_end, n):
        s = m._feed_slides(phi, into=into)
        t.j.update(board_turn=0.0, board_x=float(s[0]), board=float(s[1]))
        t.step(left=t.press())


def braced_slides(turn):
    """Slides that keep the curve on both the pin and the bearing at board turn `turn` (the board turned about the
    curve's center): the bearing then meets the curve at angle PHI0 - turn."""
    return m._rot(-turn) @ m.C_L - m.C_L


LEAD = PHI0 - m.ALPHA  # the turn whose braced pose puts the bearing right at the curve's leading end


def slides_at(phi, turn, into=0.001):
    """Slides (at board turn `turn`) that put the curve's contact with the bearing at angle `phi` on the curve."""
    c = m.BIT - (m.R + m.RB - into) * np.array([math.cos(phi + turn), math.sin(phi + turn)])
    return m._rot(-turn) @ (c - m.PIN) - m.C_L


def brace_at(t, turn):
    """Slide the board onto the pin (at turn + 14 deg) where a pivot to `turn` brings the bearing on, then pivot."""
    s = braced_slides(turn)
    t.rest(2)
    t.move(12, board_turn=turn + TH0, board_x=float(s[0]), board=float(s[1]))
    t.move(14, board_turn=turn)


def feed_at(t, turn, phi_from, phi_to, n=60, into=0.001):
    for phi in np.linspace(phi_from, phi_to, n):
        s = slides_at(phi, turn, into)
        t.j.update(board_turn=turn, board_x=float(s[0]), board=float(s[1]))
        t.step(left=t.press())


def finish_at(t, turn, home=True):
    s = slides_at(-m.ALPHA - math.radians(4.0), turn, into=-0.004) + m._rot(-turn) @ np.array([0.0, 0.03])
    t.move(10, board_x=float(s[0]), board=float(s[1]))
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))
    return t.rest(25) if home else t.rest(25, left=(0.30, 0.25, m.TOP + 0.15))


def whole_one_pass(home=True):
    t = Trial()
    brace_at(t, LEAD)
    feed_at(t, LEAD, m.ALPHA, -m.ALPHA - math.radians(4.0))
    return t, finish_at(t, LEAD, home)


def finish(t, home=True, past=True):
    if past:
        s = m._feed_slides(-m.ALPHA - math.radians(4.0), into=-0.004) + np.array([0.0, 0.03])
        t.move(10, board_x=float(s[0]), board=float(s[1]))
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))  # lift off
    return t.rest(25) if home else t.rest(25, left=(0.30, 0.25, m.TOP + 0.15))


def test_geometry():
    j = {"board_turn": TH0, "board_x": 0.0, "board": 0.0}
    check(float(m.braced(j)), 1.0, "braced pose at the start turn is on the pin")
    check(float(m.bit_dist(j) > m.RC), 1.0, "braced pose at the start turn is clear of the cutter circle")
    j0 = {"board_turn": 0.0, "board_x": 0.0, "board": 0.0}
    check(float(m.on_bearing(j0) and not m.braced(j0)), 1.0, "turned to 0 the curve rides the bearing, off the pin")
    js = {"board_turn": m.START[2], "board_x": m.START[0], "board": m.START[1]}
    check(float(m.START[2] == 0.0 and 0.02 < m.pin_gap(js) < 0.025 and m.bit_dist(js) > 0.033), 1.0,
          "start: square to the table, 2.2 cm off the pin, 3.4 cm off the bit")
    check(float(abs(m.ARC - 0.2579) < 0.001), 1.0, "curve is 25.8 cm long")


def test_do_nothing():
    t = Trial()
    check(t.rest(60)[0], 0.0, "do nothing")


def test_start_state():
    """Anti-gaming: the start state meets no end condition: not braced, not on the bearing, nothing traced, 0."""
    t = Trial()
    sc, ok = t.step()
    check(float(sc == 0.0 and not ok and not m.braced(t.j) and not m.on_bearing(t.j) and m.traced_cm() == 0.0), 1.0,
          "the start: not braced, curve off the bearing, nothing traced, score 0")


def test_pass_off_the_bearing():
    """Anti-gaming: braced, pivoted, then fed round 1 mm off the bearing (never touching it) and on past the bit: the
    bearing passed over nothing, so nothing is traced (a curve within 0.5 mm of the bearing is touching it)."""
    t = Trial()
    brace_at(t, LEAD)
    feed_at(t, LEAD, m.ALPHA - 0.004 / m.R, -m.ALPHA - math.radians(4.0), into=-0.001)
    sc, _ = finish_at(t, LEAD)
    check(float(m.traced_cm() <= 0.4 and sc <= 0.25), 1.0,
          f"fed round 1 mm off the bearing: {m.traced_cm():.1f} cm traced, score {sc}")


def test_demo_plan_offline():
    """The demo's waypoints (the board assumed to follow the pads exactly): no grab, no hands near the bit, the whole
    curve traced, past the bit: 1.0. The sim decides whether the board really follows; this checks the plan."""
    plan = m.demo()["left"]
    t = Trial()
    t.rest(2)
    s_pin = None
    for w in plan:
        lab = getattr(w, "label", "")
        if lab in ("brace it against the starting pin", "settle on the pin"):
            s_pin = m.BRACED_LEAD
        if lab == "brace it against the starting pin":
            t.move(6, board_turn=TH0, board_x=float(m.BRACED_LEAD[0]), board=float(m.BRACED_LEAD[1]))
        elif lab == "pivot about the pin into the bit":
            t.move(3, board_turn=max(m.LEAD, t.j["board_turn"] - (TH0 - m.LEAD) / 14))
    assert s_pin is not None
    feed_at(t, m.LEAD, m.ALPHA, -m.ALPHA - math.radians(4.0), n=400)  # the sim feeds ~0.6 mm a step
    sc, ok = finish_at(t, m.LEAD)
    check(sc, 1.0, f"demo plan through the scorer (traced {m.traced_cm():.1f} cm)")
    check(float(m.bit_dist({"board_turn": TH0, "board_x": float(m.BRACED_LEAD[0]), "board": float(m.BRACED_LEAD[1])})
                > m.RC), 1.0, "braced at the start turn, before the pivot, the board is clear of the cutter circle")
    check(float(abs(m.LEAD - LEAD) < 1e-12), 1.0, "task and test agree on the leading-end turn")


def test_old_demo_start_is_not_whole():
    """review, Nov 12: the old demo's start (bearing on 4.1 cm in from the leading end) leaves that stretch untraced."""
    t = Trial()
    brace(t)
    pivot(t)
    feed(t)
    sc, ok = finish(t)
    check(sc, 0.75, f"braced, pivoted, fed round and past, but {m.ARC * 100 - m.traced_cm():.1f} cm untraced at the "
                    f"leading end (traced {m.traced_cm():.1f} cm)")
    check(float(ok), 0.0, "no success")


def test_whole_curve_one_pass():
    t, (sc, ok) = whole_one_pass()
    check(sc, 1.0, f"braced so the bearing meets the leading end, pivoted, fed round and past (traced "
                   f"{m.traced_cm():.1f} of {m.ARC * 100:.1f} cm)")
    check(float(ok), 1.0, "success() agrees")
    check(float(m._H.hands or m._H.grab), 0.0, "no hands near the bit, no grab")


def test_whole_curve_two_passes():
    """review, Nov 12: a safe second pass braced on the pin is allowed: the old start, pulled back off the bit, then
    braced again at the leading end and fed through."""
    t = Trial()
    brace(t), pivot(t), feed(t)
    finish(t)
    first = m.traced_cm()
    back = t.slides_for((-0.06, 0.0))  # pulled 6 cm straight back toward the robot, off the bit
    t.move(8, board_x=float(back[0]), board=float(back[1]))
    s = braced_slides(LEAD)
    away = s + m._rot(-(LEAD + TH0)) @ np.array([-0.06, 0.0])
    t.move(20, board_turn=LEAD + TH0, board_x=float(away[0]), board=float(away[1]))  # slid round, 6 cm back
    check(float(m._H.grab), 0.0, "carried back round the bit without touching it")
    t.move(12, board_x=float(s[0]), board=float(s[1]))  # onto the pin
    t.move(14, board_turn=LEAD)  # pivoted in about it
    feed_at(t, LEAD, m.ALPHA, -m.ALPHA - math.radians(4.0))
    sc, ok = finish_at(t, LEAD)
    check(sc, 1.0, f"first pass {first:.1f} cm, second pass braced on the pin at the leading end: whole curve")
    check(float(m._H.grab), 0.0, "the second, braced entry is not a grab")


def test_second_pass_off_pin_is_a_grab():
    t = Trial()
    brace(t), pivot(t), feed(t)
    finish(t)
    s = slides_at(m.ALPHA - 0.01, LEAD, into=-0.03)
    t.move(5, press=False, board_turn=LEAD, board_x=float(s[0]), board=float(s[1]))
    s = slides_at(m.ALPHA - 0.01, LEAD)
    t.move(8, board_x=float(s[0]), board=float(s[1]))  # straight into the bit at the leading end, off the pin
    check(float(m._H.grab), 1.0, "second pass fed in off the pin: grab")
    check(t.rest(25)[0], 0.0, "and the whole run scores 0")


def test_whole_curve_left_on_bit():
    t = Trial()
    brace_at(t, LEAD)
    feed_at(t, LEAD, m.ALPHA, -m.ALPHA + 0.003)
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))
    check(t.rest(25)[0], 0.75, f"traced {m.traced_cm():.1f} cm, board left on the bit")


def test_not_home():
    t, (sc, ok) = whole_one_pass(home=False)
    check(sc, 1.0, "done right, arms not home (not scored)")
    check(float(ok), 1.0, "success() agrees")
    check(float(hasattr(m, "REQUIRE_ARMS_HOME") or hasattr(m, "REQUIRED")), 0.0, "no arm-home rule, no 18 cm full")


def test_still_pressing():
    t = Trial()
    brace(t), pivot(t), feed(t)
    s = m._feed_slides(-m.ALPHA - math.radians(4.0), into=-0.004) + np.array([0.0, 0.03])
    t.move(10, board_x=float(s[0]), board=float(s[1]))
    check(t.rest(25, left=t.press())[0], 0.75, "board traced and past but still pressed at the end: 0.75")
    p = t.press()
    p = (p[0], p[1], p[2] + (m.TOP + m.GAP + m.T + m.TIP - 0.004) - m.PZ)  # the pad tips 4 mm below the board's top
    away = t.rest(25, left=(p[0], p[1], p[2] + 0.10))[0]  # what the board earns with the hand lifted away
    check(float(away > 0), 1.0, f"hand lifted 10 cm: {away}")
    check(t.rest(25, left=(p[0], p[1], p[2] + 0.0045))[0], 0.75,
          "pads resting on the board's top (0.5 mm over it, within the 2 mm contact tolerance): still on it, 0.75")
    check(t.rest(25, left=(p[0], p[1], p[2] + 0.012))[0], away,
          "pads hovering 8 mm over the board, touching nothing: let go (the old 1.5 cm zone scored 0)")
    check(t.rest(25, left=(p[0], p[1], p[2] + 0.007), opening=1.0)[0], away,
          "open hand hovering 3 mm over the board, touching nothing: let go")


def test_hand_against_the_side():
    """A hand pushing the board's back edge from behind (fingers down beside it) is on it; 3 mm off it is not."""
    t = Trial()
    brace(t), pivot(t), feed(t)
    s = m._feed_slides(-m.ALPHA - math.radians(4.0), into=-0.004) + np.array([0.0, 0.03])
    t.move(10, board_x=float(s[0]), board=float(s[1]))
    th, sl = t.j["board_turn"], np.array([t.j["board_x"], t.j["board"]])
    z = m.TOP + m.GAP + m.T / 2 + m.PAD_TIP_PAST_TCP  # pad tips at the board's mid-thickness
    half_x = 0.0032 * 2  # a closed hand's tip boxes reach 6.4 mm out along x each way from the tool point
    away = t.rest(25)[0]
    for gap, want, what in ((0.001, 0.75, "fingers 1 mm off the back edge: touching, still on it (0.75)"),
                            (0.003, away, "fingers 3 mm off the back edge: not touching, let go")):
        w = m._board_world(th, sl, (m.BACK - half_x - gap, float(m.C_L[1])))
        check(t.rest(25, left=(float(w[0]), float(w[1]), z))[0], want, what)
    check(float(len(m.on_board(m._TOUCH["objs"]))), 0.0, "on_board() agrees")


def test_straight_in():
    """The trap: the curve fed straight into the bit, never touching the pin."""
    t = Trial()
    t.rest(2)
    # turn to 0 first, well away from both, then slide the curve straight onto the bearing from the robot's side
    t.move(10, board_turn=0.0, board_x=-0.04, board=0.0)
    check(float(m.braced(t.j)), 0.0, "clear of the pin")
    t.move(10, board_x=0.0)
    check(float(m._H.grab), 1.0, "fed into the bit off the pin is a grab")
    feed(t)
    check(finish(t)[0], 0.0, "grab, then fed round and past perfectly: still 0 (irreversible)")


def test_feed_along_then_in():
    """Slide the board along to the left first, then into the bit beside the pin (off it)."""
    t = Trial()
    t.rest(2)
    t.move(10, board_turn=0.0, board_x=-0.03, board=0.06)
    t.move(10, board_x=0.004)
    check(float(m._H.grab), 1.0, "curve pushed into the bit to the left of the pin: grab")
    check(t.rest(30)[0], 0.0, "scores 0")


def test_pivot_only():
    t = Trial()
    brace(t)
    pivot(t)
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))
    check(t.rest(25)[0], 0.25, "braced and pivoted onto the bearing, then let go")


def test_brace_no_pivot():
    t = Trial()
    brace(t)
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))
    check(t.rest(25)[0], 0.0, "braced on the pin only")
    t2 = Trial()
    brace(t2)
    pivot(t2, to=math.radians(4.0))  # inside the cutter circle but not yet on the bearing
    p = t2.press()
    t2.step(left=(p[0], p[1], p[2] + 0.10))
    check(t2.rest(25)[0], 0.0, "pivoted into the cutter circle but short of the bearing")


def test_pivot_away():
    t = Trial()
    brace(t)
    pivot(t, to=math.radians(30.0))  # swung the wrong way about the pin
    check(t.rest(25)[0], 0.0, "turned the wrong way about the pin")


def test_halfway():
    t = Trial()
    brace(t), pivot(t)
    feed(t, phi_end=PHI0 - 0.11 / m.R, n=30)
    s = m._feed_slides(PHI0 - 0.11 / m.R) - np.array([0.03, 0.0])  # pulled back off the bit
    t.move(10, board_x=float(s[0]), board=float(s[1]))
    p = t.press()
    t.step(left=(p[0], p[1], p[2] + 0.10))
    sc = t.rest(25)[0]
    check(sc, 0.5, f"traced {m.traced_cm():.1f} cm then pulled back")


def test_full_not_past():
    t = Trial()
    brace(t), pivot(t)
    feed(t, phi_end=-m.ALPHA + 0.01, n=60)
    check(finish(t, past=False)[0], 0.75, f"traced {m.traced_cm():.1f} cm (over 18), board left on the bit")


def test_climb():
    """Pivoted onto the bearing at the trailing end and fed the wrong way (left to right): traces nothing."""
    t = Trial()
    brace(t), pivot(t)
    phi = PHI0
    for phi in np.linspace(PHI0, PHI0 + 0.03 / m.R, 15):  # the contact moving toward the leading end
        s = m._feed_slides(phi)
        t.j.update(board_turn=0.0, board_x=float(s[0]), board=float(s[1]))
        t.step(left=t.press())
    check(float(m.traced_cm() <= 0.2), 1.0, f"climb feed traces nothing ({m.traced_cm():.1f} cm)")


def test_reentry_off_pin():
    """A proper start, then the board pulled off the bit and pushed back in further along, off the pin."""
    t = Trial()
    brace(t), pivot(t)
    feed(t, phi_end=PHI0 - 0.05 / m.R, n=20)
    s = m._feed_slides(PHI0 - 0.05 / m.R) - np.array([0.03, 0.0])
    t.move(8, board_x=float(s[0]), board=float(s[1]))
    check(float(m._H.entered), 0.0, "out of the cutter circle")
    s2 = m._feed_slides(PHI0 - 0.07 / m.R)
    t.move(8, board_x=float(s2[0]), board=float(s2[1]))
    check(float(m._H.grab), 1.0, "pushed back into the bit off the pin: grab")
    feed(t)
    check(finish(t)[0], 0.0, "and stays 0")


def test_shove_onto_pin_and_bit():
    """Pin contact and cutter-circle entry in the same step (a shove) is not a braced entry."""
    t = Trial()
    t.rest(2)
    t.move(1, board_x=0.0, board=0.0, board_turn=math.radians(2.0))
    check(float(m._H.grab), 1.0, "shoved onto pin and bit at once")


def test_hands():
    t = Trial()
    t.rest(2)
    t.step(left=(m.BIT[0] - 0.06, m.BIT[1], m.TOP + 0.04))
    check(float(m._H.hands), 1.0, "tool point 6 cm from the bit, pads low")
    brace(t), pivot(t), feed(t)
    check(finish(t)[0], 0.0, "hands near the bit is irreversible")
    t2 = Trial()
    t2.rest(2)
    t2.step(left=(m.BIT[0] - 0.06, m.BIT[1], m.TOP + 0.10))
    check(float(m._H.hands), 0.0, "high over the bit is fine")
    # reviews of other tasks (a slicer, a drill press): a tool-point-only check lets a finger in while the tool point
    # stays out. Here the gripper's own shape counts: the tool point 9 cm out, pads on the board, but the closed
    # fingers' 6.8 cm-wide bodies reach 3.4 cm toward the bit (yaw 0: their width along x) and their pads 2 cm
    tz = m.TOP + m.GAP + m.T + m.PAD_TIP_PAST_TCP
    for dist, want in ((0.090, 1.0), (0.120, 0.0)):
        t3 = Trial()
        t3.rest(2)
        t3.step(left=(m.BIT[0] - dist, m.BIT[1], tz))
        check(float(m._H.hands), want, f"tool point {dist * 100:.0f} cm from the bit's axis, pads on the board: "
              f"{'a finger in the 3 in zone' if want else 'clear'}")
        check(float(dist > m.HANDS), 1.0, "the tool point itself is outside 3 in")


def test_reset():
    t = Trial()
    t.rest(2)
    t.move(1, board_x=0.0, board=0.0, board_turn=math.radians(2.0))
    check(float(m._H.grab), 1.0, "grab in one trial")
    t2 = Trial()
    t2.rest(2)
    check(float(m._H.grab or m._H.started or m._H.traced.any()), 0.0, "next trial starts clean (kit.episode())")
    brace_at(t2, LEAD)
    feed_at(t2, LEAD, m.ALPHA, -m.ALPHA - math.radians(4.0))
    check(finish_at(t2, LEAD)[0], 1.0, "and can score 1.0")


def test_carry_to_the_pin():
    """The demo carries the board from where it lies (square) to the brace: clear of the pin and the cutter circle all the
    way, braced only at the end, and never a grab."""
    path = m.carry_path()
    t = Trial()
    t.rest(2)
    for k, (th, sl) in enumerate(path):
        t.step(board_turn=th, board_x=float(sl[0]), board=float(sl[1]))
        j = dict(t.j)
        if k <= m.CARRY_N:
            check(float(m.pin_gap(j) > 0.004 and m.bit_dist(j) > m.RC + 0.010), 1.0,
                  f"carry step {k}: {m.pin_gap(j) * 1000:.1f} mm off the pin, {m.bit_dist(j) * 1000:.1f} mm off the bit's axis")
    check(float(m.braced(dict(t.j)) and not m._H.grab), 1.0, "ends braced on the pin, no grab")
    check(float(abs(path[0][0]) < 1e-12 and abs(path[m.CARRY_N][0] - TH0) < 1e-12), 1.0, "from square to the brace turn")


def test_model():
    """The model choices the demo relies on: the bearing's collider is frictionless (it rolls), the pin's is not; the
    board is a free body (no joints, nothing holds it on the pin); the scorer reads its pose from the body."""
    usd = kit._usda(m.OBJECTS[1])
    check(float(usd.count("material:binding:physics") == 2 and usd.count("Looks/Slick>") == 1), 1.0,
          "only the bearing is slick (the table surface has its own friction)")
    on = {"board_turn": TH0, "board_x": 0.0, "board": 0.0}
    off = {"board_turn": m.START[2], "board_x": m.START[0], "board": m.START[1]}
    check(float(m.braced(on) and not m.braced(off)), 1.0, "braced on the pin, not at the start")
    names = [o.name for o in m.OBJECTS]
    check(float(names == ["router_table", "router_bit", "board"]
                and not any(isinstance(o, kit.Mechanism) for o in m.OBJECTS)), 1.0,
          "table, bit and board are plain props: no joints anywhere")
    b = m.OBJECTS[2]
    check(float(b.mass == m.BOARD_MASS and abs(b.yaw - m.START[2]) < 1e-12), 1.0, "the board is a free 250 g body")
    j = {"board_turn": 0.3, "board_x": 0.012, "board": -0.034}
    back = m._joints({"board": board(j)})
    check(float(max(abs(back[k] - j[k]) for k in j) < 1e-9), 1.0, "the scorer's pose read back from the body")


def test_table_friction():
    """The table's friction: the board lies on the table top's surface (mu 0.25 sliding, 0.3 static, "min" combine so
    the board's grippy material doesn't raise it); its 250 g and the hands' press bear on it through the contact."""
    usd = kit._usda(m.OBJECTS[1])
    check(float("physics:dynamicFriction = 0.25" in usd and "physics:staticFriction = 0.3" in usd
                and usd.count('frictionCombineMode = "min"') == 2), 1.0, "table surface: mu 0.25 / 0.3, min combine")
    surf = m.OBJECTS[1].colliders[-1]
    top = m.OBJECTS[1].pos[2] + surf.center[2] + surf.size[2] / 2
    b = m.OBJECTS[2]
    bottom = b.pos[2] + min(c.center[2] - c.size[2] / 2 for c in b.colliders)
    check(float(abs(top - bottom) < 1e-6), 1.0, f"the board's underside rests on the surface ({(top - bottom) * 1e3:.3f} mm)")
    busd = kit._usda(b)
    check(float("physics:mass = 0.25" in busd and "physics:centerOfMass" in busd and "diagonalInertia" in busd), 1.0,
          "the board's mass, center of mass and inertia are its own (not from overlapping colliders)")
    lo = np.array(m.OBJECTS[1].pos[:2]) + np.array(surf.center[:2]) - np.array(surf.size[:2]) / 2
    hi = lo + np.array(surf.size[:2])
    tab = m.OBJECTS[0]
    check(float(np.allclose(hi - lo, tab.size[:2])), 1.0, "the friction surface is the whole table top")


def drag_at(j):
    """The cord's pull (world, N) after the board moves to joints-style pose j."""
    m.drag_load({"board_turn": float(j[0]), "board_x": float(j[1][0]), "board": float(j[1][1])})
    return m.drag_force_now()


def test_drag():
    """The drag, the same law in the sim as on the rig: the cord from the screw eye in the board's trailing end to the
    pulley pulls 3.0 N toward the pulley while it is taut, and 0 once the board has let it run back past the cam
    cleat's catch (the cleat holds the weight): slack at the start and through the carry and pivot to the snug pose,
    3.0 N along the feed (2.97-3.00 N against it) and while the board is held there, 0 once it runs back 0.5 mm, back
    on only near the most cord drawn."""
    check(float(abs(m.CORD_N - 3.0) < 1e-12 and abs(0.300 * m.G - 2.94) < 0.005), 1.0,
          "0.300 kg hanging weight (2.94 N), trimmed with the pulley's and cleat's drag to 3.0 N at the board")
    kit._EPISODE["count"] += 1
    check(float(np.linalg.norm(drag_at((m.START[2], np.array(m.START[:2])))) == 0.0), 1.0, "at the start: slack, 0 N")
    carry = max(float(np.linalg.norm(drag_at(p))) for p in m.carry_path())
    check(float(carry == 0.0), 1.0, "carried square to the brace on the pin: slack all the way")
    s = braced_slides(LEAD)
    piv = [float(np.linalg.norm(drag_at((LEAD + TH0 * (1 - k / 20), s)))) for k in range(21)]
    check(float(max(piv[:-1]) == 0.0 and abs(piv[-1] - 3.0) < 1e-9), 1.0, "pivoted about the pin: slack until the "
          "leading end meets the bearing, where the cord is set snug and takes hold")
    against, angle, on, size = [], 0.0, 0, []
    phis = np.linspace(m.ALPHA, -m.ALPHA - math.radians(4.0), 400)
    for phi in phis:  # a feed round the bearing, the board turning about its curve's center (the demo's plan)
        th, sl, beta = m._feed_pose(min(phi, m.PHI_START))
        f = drag_at((th, sl))
        if not f.any():
            continue
        on += 1
        tau = np.array([-math.sin(beta), math.cos(beta)])  # the feed's direction at the bearing (right to left)
        against.append(float(-f @ tau))
        size.append(float(np.linalg.norm(f)))
        angle = max(angle, math.degrees(math.acos(min(1.0, -f @ tau / np.linalg.norm(f)))))
    check(float(on >= 0.95 * len(phis) and max(abs(v - 3.0) for v in size) < 1e-9 and min(against) >= 2.96
                and angle < 9.0), 1.0,
          f"fed round the bearing: 3.0 N on {on} of {len(phis)} steps, {min(against):.2f}-{max(against):.2f} N against "
          f"the feed (the cord within {angle:.1f} deg of it)")
    eye = m.to_world({"board_turn": th, "board_x": float(sl[0]), "board": float(sl[1])}, m.EYE)
    f = m.drag_force_now()
    check(float(np.allclose(m._D.at, eye) and np.allclose(f / 3.0, (m.PULLEY - eye) / np.linalg.norm(m.PULLEY - eye))),
          1.0, "it acts at the screw eye, toward the pulley")
    tau_pin = (eye - m.PIN)[0] * f[1] - (eye - m.PIN)[1] * f[0]
    check(float(abs(m._D.load[0] - tau_pin) < 1e-12), 1.0, "its moment is the cord's line's, about the eye")
    # stationary and backward: none
    last = (th, sl)
    check(float(abs(np.linalg.norm(drag_at((last[0], last[1] + np.array([0.0, 2e-6])))) - 3.0) < 1e-9), 1.0,
          "board held at rest, the cord taut: 3.0 N (the hands hold the weight up)")
    back = [float(np.linalg.norm(drag_at(m._feed_pose(p)[:2]))) for p in phis[::-1][1:200]]
    check(float(max(back) == 0.0), 1.0, "drawn back toward the pulley: slack, 0 N")
    more = phis[-1] + (phis[1] - phis[0]) * np.arange(1, 11)  # on past where the first feed stopped
    fwd = [float(np.linalg.norm(drag_at(m._feed_pose(p)[:2]))) for p in np.concatenate([phis[200:], more])]
    first = next(i for i, v in enumerate(fwd) if v > 0)
    check(float(195 <= first <= 200 and max(fwd[:first]) == 0.0 and min(fwd[first:]) > 3.0 - 1e-9), 1.0,
          f"fed again: no pull until the board is back within 0.5 mm of the most cord drawn (step {first} of "
          f"{len(fwd)})")
    # near miss: just under the cord drawn per step that counts as moving
    kit._EPISODE["count"] += 1
    base = (LEAD, slides_at(m.ALPHA - 0.02, LEAD))
    drag_at(base)
    j = {"board_turn": LEAD, "board_x": float(base[1][0]), "board": float(base[1][1])}
    u = m.to_world(j, m.EYE) - m.PULLEY
    u = m._rot(-LEAD) @ (u / np.linalg.norm(u))  # away from the pulley, in the board's frame
    run0 = m.cord_length(j)
    for step, want in ((-0.0004, 3.0), (-0.0006, 0.0), (0.0, 3.0), (0.0001, 3.0)):
        kit._EPISODE["count"] += 1
        drag_at(base)
        m._D.drawn = run0  # the cord drawn out to here
        got = float(np.linalg.norm(drag_at((LEAD, base[1] + step * u))))
        check(float(abs(got - want) < 1e-9), 1.0, f"eye {step * 1e3:+.1f} mm from the most cord drawn: {got:.1f} N "
              f"(the cleat catches after 0.5 mm of run-back)")
    # the trap: a board fed straight in off the pin meets the same drag (and scores 0 as a grab)
    kit._EPISODE["count"] += 1
    t = Trial()
    t.rest(2)
    t.move(10, board_turn=0.0, board_x=-0.04, board=0.06)
    for k in range(1, 11):
        t.step(board_x=-0.04 + 0.0044 * k)
    check(float(m._H.grab), 1.0, "a board pushed straight in off the pin is a grab, whatever the drag does")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
    print("all pass")
