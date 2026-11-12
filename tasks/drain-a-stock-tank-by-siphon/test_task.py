"""Offline checks of the siphon model and the scorer: step-by-step hose-end and gripper histories at 20 Hz, no
simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import importlib.util
import math
import os

import numpy as np
from yam_arena import demo as yd
from yam_arena import ik
from yam_arena.task_file import ObjectState

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("stock_tank_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit, pk, sk, gk = m.kit, m.pk, m.sk, m.gk

T = m.TOP
PARK = {"left_gripper": np.array([0.20, 0.27, T + 0.20]), "right_gripper": np.array([0.20, -0.27, T + 0.20])}
ON_STICK = 0.266  # opening closed on the 2.5 cm stick
ON_HOSE = 0.415  # closed across the stick and the taped hose (3.9 cm)
PLAIN = math.pi / 2  # fingers along x: the intake's plain faces
PINCH = m.OUTLET_YAW  # fingers square across the outlet's taped hose
OUT_OPEN = m.OUTLET_YAW - math.pi / 2  # fingers on the outlet's plain faces
OUT_Q = (0.0, 0.0, math.sin(m.OUTLET_YAW / 2), math.cos(m.OUTLET_YAW / 2))


def quat(yaw):
    return np.asarray(ik.matrix_to_quat_xyzw(yd.grip_rotation(yaw)), float)


def st(pos, q=(0, 0, 0, 1.0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(q, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


class Trial:
    """A fresh trial: ends standing at their starts, arms parked open. An end gripped by a hand rides with it."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.end = {"intake": np.array([*m.INTAKE0, T]), "outlet": np.array([*m.OUTLET0, T])}
        self.g = {k: v.copy() for k, v in PARK.items()}
        self.yaw = {"left_gripper": PLAIN, "right_gripper": PLAIN}
        self.open = {g: 1.0 for g in PARK}
        self.held = {}  # gripper -> (end, offset of the end's origin from the tool point)
        self.n = 0
        self.last = None
        self.step()

    def objects(self):
        o = {"tank": st((*m.TANK, T)), "tub": st((*m.TUB, T))}
        for n, p in self.end.items():
            o[n] = st(p, OUT_Q if n == "outlet" else (0, 0, 0, 1.0))
        for g, p in self.g.items():
            o[g] = st(p, quat(self.yaw[g]))
            kit._OPENING[g] = self.open[g]
        return o

    def step(self, k=1):
        for _ in range(k):
            for g, (n, off) in self.held.items():
                self.end[n] = self.g[g] + off
            o = self.objects()
            pk.update(o)
            sk.update(o)
            m.score(o)
            self.last = o
            self.n += 1
        return self.last

    def score(self):
        return m.score(self.last)

    def move(self, g, to, secs, yaw=None):
        """Straight-line move of a gripper's tool point (interpolated per 20 Hz step)."""
        a, b = self.g[g].copy(), np.asarray(to, float)
        n = max(1, int(round(secs * 20)))
        y0 = self.yaw[g]
        for i in range(1, n + 1):
            self.g[g] = a + (b - a) * i / n
            if yaw is not None:
                self.yaw[g] = y0 + (yaw - y0) * i / n
            self.step()

    def grip(self, g, end, opening, secs=0.5):
        self.open[g] = opening
        self.held[g] = (end, self.end[end] - self.g[g])
        self.step(int(secs * 20))

    def let_go(self, g, secs=0.5, opening=1.0):
        self.held.pop(g, None)
        self.open[g] = opening
        self.step(int(secs * 20))


def tcp(xy, bottom, d=0.0):
    """Tool point gripping an end standing at xy; d: offset toward the outlet's taped hose."""
    a = m.OUTLET_YAW
    return np.array([xy[0] - d * math.sin(a), xy[1] + d * math.cos(a), T + bottom + m.GRIP_Z])


def place_intake(t, g="left_gripper", bed=m.TANK_BED, hold=True):
    t.move(g, tcp(m.INTAKE0, 0.04), 1.5, PLAIN)
    t.move(g, tcp(m.INTAKE0, 0.0), 0.8)
    t.grip(g, "intake", ON_STICK)
    t.move(g, tcp(m.INTAKE0, m.OVER_RIM), 1.0)
    t.move(g, tcp(m.IN_INTAKE, m.OVER_RIM), 1.5)
    t.move(g, tcp(m.IN_INTAKE, bed), 1.0)
    if not hold:
        t.let_go(g, opening=0.6)
        t.move(g, tcp(m.IN_INTAKE, m.OVER_RIM), 0.8)


def sink_outlet(t, g="right_gripper"):
    """As the demo: picked up pinched, sunk, let go (open) while the hose fills."""
    c = m.COL_R
    t.move(g, tcp(m.OUTLET0, 0.04, c), 1.5, PINCH)
    t.move(g, tcp(m.OUTLET0, 0.0, c), 0.8)
    t.grip(g, "outlet", ON_HOSE)
    t.move(g, tcp(m.OUTLET0, m.OVER_RIM, c), 1.0)
    t.move(g, tcp(m.IN_OUTLET, m.OVER_RIM, c), 1.5)
    t.move(g, tcp(m.IN_OUTLET, m.TANK_BED, c), 1.0)
    check(sk.fill() == 0.0, "sunk while pinched: no fill yet")
    t.let_go(g, opening=0.6)
    t.step(50)  # the hose fills (2 s)


def pinch_outlet(t, g="right_gripper"):
    t.move(g, tcp(m.IN_OUTLET, m.TANK_BED + 0.015, m.COL_R), 1.0, PINCH)
    t.move(g, tcp(m.IN_OUTLET, m.TANK_BED, m.COL_R), 0.6)
    t.grip(g, "outlet", ON_HOSE)


def carry_out(t, g="right_gripper", secs=(1.2, 2.0, 1.2), dy=m.COL_R, low=m.TUB_FLOOR):
    t.move(g, tcp(m.IN_OUTLET, m.OVER_RIM, dy), secs[0])
    t.move(g, tcp(m.IN_TUB, m.OVER_RIM, dy), secs[1])
    t.move(g, tcp(m.IN_TUB, low, dy), secs[2])


def finish(t, release_intake=True, drain_s=40):
    """As the demo: the outlet let go in the tub and the right arm home; the left keeps the intake down while the tank
    drains, then lets go and goes home."""
    t.let_go("right_gripper")
    t.move("right_gripper", tcp(m.IN_TUB, 0.09), 0.8)
    t.move("right_gripper", PARK["right_gripper"], 1.5)
    t.step(int(drain_s * 20))
    if release_intake:
        t.let_go("left_gripper", opening=0.6)
        t.move("left_gripper", tcp(m.IN_INTAKE, m.OVER_RIM), 0.8)
        t.move("left_gripper", PARK["left_gripper"], 1.5)
        t.step(30)


def demo_run(**kw):
    t = Trial()
    place_intake(t)
    sink_outlet(t)
    pinch_outlet(t)
    carry_out(t, **kw)
    finish(t)
    return t


def test_numbers_match_text():
    s = sk._CFG["siphon"]
    check(abs(m.DRAIN_ML - 1162) < 1, f"1,571 mL to start, 1,162 mL above the drawn-down level ({m.DRAIN_ML:.0f})")
    check(abs(math.pi * m.TANK_R ** 2 * m.WATER * 1e6 - 1571) < 1, "starting volume 1,571 mL")
    check(abs(m.SURFACE0 - T - 0.092) < 1e-9 and abs(m.DRAWN_Z - T - 0.055) < 1e-9, "surface 9.2 cm, drawn down at 5.5 cm")
    check(abs(s.hose_ml - 76.0) < 0.1 and abs(s.k - 283) < 1, f"hose {s.hose_ml:.1f} mL, k {s.k:.0f} mL/s per sqrt(m)")
    head = m.SURFACE0 - (T + m.TUB_FLOOR + m.MOUTH[2])
    check(abs(head - 0.081) < 1e-9, f"starting head 8.1 cm, flow {s.k * math.sqrt(head):.0f} mL/s")
    check(m.SPILL_MAX == 20.0 and m.FULL == 0.90 and m.SOME == 0.25 and m.AIR_MAX == 10.0, "thresholds as stated")
    check(m.TANK_BED + m.GRIP_Z - 0.013 > m.SURFACE0 - T, "pad tips stay above the water while holding an end down")
    check(m.TUBE_TOP[2] + m.TANK_BED < m.SURFACE0 - T, "an end's hose joint is under the water when it stands sunk")


def test_do_nothing():
    t = Trial()
    t.step(400)
    check(t.score() == 0.0 and not sk.primed(), "do nothing: 0")


def test_demo_full():
    t = demo_run()
    o = t.last
    check(sk.lost_prime(o, "drawn_down") and not sk.lost_prime(o, "intake_out") and not sk.lost_prime(o, "outlet_high"),
          f"drawn down, no other loss ({sk.losses()})")
    check(m.into_tub(o) >= 0.9 * m.DRAIN_ML, f"{m.into_tub(o):.0f} mL of {m.DRAIN_ML:.0f} in the tub")
    check(pk.spilled() < 1e-6, "nothing spilled")
    check(t.score() == 1.0, f"demo path scores 1.0 (got {t.score()})")


def test_fast_carry():
    t = demo_run(secs=(0.25, 0.4, 0.25))  # 4-5 cm per step
    check(t.score() == 1.0, f"fast but correct carry: 1.0 (got {t.score()})")


def test_outlet_carried_open():
    t = Trial()
    place_intake(t)
    t.move("right_gripper", tcp(m.OUTLET0, 0.04), 1.5, OUT_OPEN)
    t.move("right_gripper", tcp(m.OUTLET0, 0.0), 0.8)
    t.grip("right_gripper", "outlet", ON_STICK)
    t.move("right_gripper", tcp(m.OUTLET0, m.OVER_RIM), 1.0)
    t.move("right_gripper", tcp(m.IN_OUTLET, m.OVER_RIM), 1.5)
    t.move("right_gripper", tcp(m.IN_OUTLET, m.TANK_BED), 1.0)
    t.step(50)  # held on its plain faces: open, the hose fills
    check(sk.primed() and not m.capped(t.last), "held on the plain faces it fills and is not capped")
    carry_out(t, dy=0.0)
    check(sk.lost_prime(t.last, "outlet_high"), "carried out open: outlet_high")
    finish(t)
    check(t.score() == 0.0, f"outlet carried out open: 0 (got {t.score()})")


def test_pinched_while_sinking_never_fills():
    t = Trial()
    place_intake(t)
    t.move("right_gripper", tcp(m.OUTLET0, 0.04, m.COL_R), 1.5, PINCH)
    t.move("right_gripper", tcp(m.OUTLET0, 0.0, m.COL_R), 0.8)
    t.grip("right_gripper", "outlet", ON_HOSE)
    check(m.capped(t.last), "pinched across the hose: capped")
    t.move("right_gripper", tcp(m.OUTLET0, m.OVER_RIM, m.COL_R), 1.0)
    t.move("right_gripper", tcp(m.IN_OUTLET, m.OVER_RIM, m.COL_R), 1.5)
    t.move("right_gripper", tcp(m.IN_OUTLET, m.TANK_BED, m.COL_R), 1.0)
    t.step(60)
    check(sk.fill() == 0.0 and not sk.primed(), "sunk with the outlet pinched: air trapped, no fill")


def test_intake_let_go_during_carry():
    t = Trial()
    place_intake(t, hold=False)
    sink_outlet(t)
    pinch_outlet(t)
    carry_out(t)
    check(sk.lost_prime(t.last, "intake_out"), "intake unheld while the outlet is carried out: it springs up, intake_out")
    finish(t, release_intake=False)
    check(t.score() == 0.0, f"intake let go during the carry: 0 (got {t.score()})")


def test_one_gripper_only():
    t = Trial()
    place_intake(t, g="right_gripper", hold=False)
    sink_outlet(t)
    pinch_outlet(t)
    carry_out(t)
    finish(t, release_intake=False)
    check(t.score() == 0.0 and sk.lost_prime(t.last, "intake_out"), f"one gripper does it all: 0 (got {t.score()})")


def test_one_arm_low_carry_attack():
    """QC attack: one hand stands the intake toward the tub side and lets go, sinks the outlet on the far side, pinches
    it and carries it low into the tub: the intake comes up as soon as the outlet leaves the tank."""
    t = Trial()
    near = (m.TANK[0] - 0.02, m.TANK[1] - 0.06)
    far = (m.TANK[0] + 0.03, m.TANK[1] + 0.05)
    g = "right_gripper"
    t.move(g, tcp(m.INTAKE0, 0.04), 1.0, PLAIN)
    t.move(g, tcp(m.INTAKE0, 0.0), 0.5)
    t.grip(g, "intake", ON_STICK)
    t.move(g, tcp(m.INTAKE0, m.OVER_RIM), 0.8)
    t.move(g, tcp(near, m.OVER_RIM), 1.0)
    t.move(g, tcp(near, m.TANK_BED), 0.8)
    t.let_go(g, opening=0.6)
    c = m.COL_R
    t.move(g, tcp(m.OUTLET0, 0.04, c), 1.5, PINCH)
    t.move(g, tcp(m.OUTLET0, 0.0, c), 0.8)
    t.grip(g, "outlet", ON_HOSE)
    t.move(g, tcp(m.OUTLET0, m.OVER_RIM, c), 1.0)
    t.move(g, tcp(far, m.OVER_RIM, c), 1.5)
    t.move(g, tcp(far, m.TANK_BED, c), 1.0)
    t.let_go(g, opening=0.6)
    t.step(50)
    check(sk.primed(), "one hand can sink both ends and fill the hose")
    t.grip(g, "outlet", ON_HOSE)
    t.move(g, tcp(far, m.OVER_RIM, c), 1.0)
    into = (m.TUB[0] + 0.03, m.TUB[1] + 0.07)
    t.move(g, tcp(into, m.OVER_RIM, c), 1.5)
    t.move(g, tcp(into, m.TUB_FLOOR, c), 1.0)
    t.let_go(g)
    t.move(g, PARK[g], 1.5)
    t.step(800)
    check(sk.lost_prime(t.last, "intake_out") and t.score() == 0.0, f"one-arm low carry: 0 (got {t.score()})")


def test_trapped_air_blocks_fill():
    """The outlet's taped run standing well above the water (the outlet held up, mouth just under) traps more air than
    the flow can sweep out: the hose does not fill."""
    t = Trial()
    place_intake(t)
    c = m.COL_R
    g = "right_gripper"
    t.move(g, tcp(m.OUTLET0, 0.04, c), 1.5, PINCH)
    t.move(g, tcp(m.OUTLET0, 0.0, c), 0.8)
    t.grip(g, "outlet", ON_HOSE)
    t.move(g, tcp(m.OUTLET0, m.OVER_RIM, c), 1.0)
    t.move(g, tcp(m.IN_OUTLET, m.OVER_RIM, c), 1.5)
    t.move(g, tcp(m.IN_OUTLET, m.TANK_BED + 0.012, c), 1.0)  # tube top still under, run 1.2 cm higher
    t.open[g] = 0.6  # eased open (not pinched), still holding it up
    t.step(60)
    air = m.trapped_air(t.last)
    check(air > m.AIR_MAX and sk.fill() == 0.0, f"{air:.0f} mL of air in the run: no fill")
    t.g[g] = tcp(m.IN_OUTLET, m.TANK_BED, c)
    t.end["outlet"] = np.array([*m.IN_OUTLET, m.TOP + m.TANK_BED])
    t.held[g] = ("outlet", t.end["outlet"] - t.g[g])
    t.step(60)
    air = m.trapped_air(t.last)
    check(air <= m.AIR_MAX and sk.primed(), f"standing on the floor: {air:.1f} mL of air in the run, fills and primes")


def test_outlet_left_high():
    t = Trial()
    place_intake(t)
    sink_outlet(t)
    pinch_outlet(t)
    carry_out(t, low=m.SURFACE0 - T + 0.01)  # mouth 1 cm over the tank's surface, over the tub
    finish(t)
    check(sk.lost_prime(t.last, "outlet_high") and t.score() == 0.0, f"opened above the tank's surface: 0 (got {t.score()})")


def test_short_open_flick_latches():
    t = Trial()
    place_intake(t)
    sink_outlet(t)
    pinch_outlet(t)
    t.move("right_gripper", tcp(m.IN_OUTLET, m.OVER_RIM, m.COL_R), 1.2)
    t.open["right_gripper"] = 0.6  # the pinch eased for two steps, out over the water
    t.step(2)
    t.open["right_gripper"] = ON_HOSE
    t.step(2)
    check(sk.lost_prime(t.last, "outlet_high"), "a two-step slip of the pinch above the surface is a lost prime")
    t.move("right_gripper", tcp(m.IN_TUB, m.OVER_RIM, m.COL_R), 2.0)
    t.move("right_gripper", tcp(m.IN_TUB, m.TUB_FLOOR, m.COL_R), 1.2)
    finish(t)
    check(t.score() == 0.0, f"and then nothing runs: 0 (got {t.score()})")


def test_lost_and_restarted():
    t = Trial()
    place_intake(t)
    sink_outlet(t)
    pinch_outlet(t)
    t.move("right_gripper", tcp(m.IN_OUTLET, m.OVER_RIM, m.COL_R), 1.2)
    t.let_go("right_gripper", 0.1, opening=0.6)  # dropped back in, open, above the surface for a moment
    t.held["right_gripper"] = ("outlet", t.end["outlet"] - t.g["right_gripper"])
    t.move("right_gripper", tcp(m.IN_OUTLET, m.TANK_BED, m.COL_R), 0.6)
    t.held.pop("right_gripper")
    check(sk.lost_prime(t.last, "outlet_high"), "lost once")
    t.step(50)
    pinch_outlet(t)
    carry_out(t)
    finish(t)
    check(t.score() == 0.25, f"lost, sunk again and drained: 0.25 (got {t.score()})")


def test_intake_off_the_bottom():
    t = Trial()
    place_intake(t, bed=m.TANK_BED + 0.010)
    sink_outlet(t)
    pinch_outlet(t)
    carry_out(t)
    finish(t)  # (offline the let-go intake stays where it was held)
    o = t.last
    check(sk.lost_prime(o, "drawn_down") and m.into_tub(o) < 0.9 * m.DRAIN_ML,
          f"intake held 1 cm up: stops at {m.into_tub(o):.0f} mL")
    check(t.score() == 0.5, f"stopped short: 0.5 (got {t.score()})")


def test_still_running_at_the_end():
    t = Trial()
    place_intake(t)
    sink_outlet(t)
    pinch_outlet(t)
    carry_out(t)
    finish(t, release_intake=False, drain_s=6)
    check(sk.primed() and t.score() == 0.0, f"still running, intake still held at the end: 0 (got {t.score()})")
    t.let_go("left_gripper", opening=0.6)
    t.move("left_gripper", PARK["left_gripper"], 1.0)
    check(sk.lost_prime(t.last, "intake_out") and t.score() == 0.25,
          f"let go of the intake while it still runs: it comes up, 0.25 (got {t.score()})")


def test_held_at_end():
    t = Trial()
    place_intake(t)
    sink_outlet(t)
    pinch_outlet(t)
    carry_out(t)
    t.let_go("right_gripper", opening=ON_HOSE)  # opened just enough to let it run, fingers still on it
    t.held["right_gripper"] = ("outlet", t.end["outlet"] - t.g["right_gripper"])
    t.open["right_gripper"] = 0.6
    t.step(900)
    check(sk.lost_prime(t.last, "drawn_down"), "drained")
    check(t.score() == 0.0, f"outlet still in the hand at the end (open, 0.6): 0 (got {t.score()})")
    t.open["right_gripper"] = 0.0
    t.step(5)
    check(t.score() == 0.0, "closed fist at the stick: 0")
    t.held.pop("right_gripper")
    t.g["right_gripper"] = t.end["outlet"] + np.array([0.0, 0.0, m.END_H + 0.005])
    t.open["right_gripper"] = 1.0
    t.step(5)
    check(t.score() == 0.0, "open hand hovering 5 mm over the stick's top (pad tips inside its outline): 0")


def test_empty_fist_is_not_a_pinch():
    t = Trial()
    place_intake(t)
    sink_outlet(t)
    t.move("right_gripper", tcp(m.IN_OUTLET, m.TANK_BED, m.COL_R), 1.0, PINCH)
    t.open["right_gripper"] = 0.0
    t.step(3)
    check(not m.capped(t.last) and not m.gripping(t.last, "outlet"), "an empty fist at the hose is not a pinch")
    t.open["right_gripper"] = 0.9
    t.step(3)
    check(not m.capped(t.last), "an open hand around it is not a pinch")
    t.yaw["right_gripper"] = OUT_OPEN
    t.open["right_gripper"] = ON_HOSE
    t.step(3)
    check(not m.capped(t.last), "fingers on the plain faces are not a pinch, whatever the opening")
    t.yaw["right_gripper"] = PINCH
    t.g["right_gripper"] = tcp(m.IN_OUTLET, m.TANK_BED - 0.03, m.COL_R)
    t.step(3)
    check(not m.capped(t.last), "across the end tube (below the hose run) is not a pinch")


def test_spill():
    t = Trial()
    place_intake(t)
    sink_outlet(t)
    pinch_outlet(t)
    t.move("right_gripper", tcp(m.IN_OUTLET, m.OVER_RIM, m.COL_R), 1.2)
    beside = (m.TUB[0] - 0.12, m.TUB[1] + 0.03)  # on the table beside the tub
    t.move("right_gripper", tcp(beside, m.OVER_RIM, m.COL_R), 1.5)
    t.move("right_gripper", tcp(beside, 0.0, m.COL_R), 1.0)
    t.open["right_gripper"] = 0.6
    t.step(10)
    check(pk.spilled() > 20, f"opened on the table: {pk.spilled():.0f} mL on the floor")
    t.grip("right_gripper", "outlet", ON_HOSE)
    t.move("right_gripper", tcp(beside, m.OVER_RIM, m.COL_R), 0.8)
    t.move("right_gripper", tcp(m.IN_TUB, m.OVER_RIM, m.COL_R), 1.0)
    t.move("right_gripper", tcp(m.IN_TUB, m.TUB_FLOOR, m.COL_R), 0.8)
    finish(t)
    check(t.score() == 0.0, f"more than 20 mL spilled: 0 (got {t.score()})")


def test_episode_reset():
    t = demo_run()
    check(t.score() == 1.0, "a full trial")
    t2 = Trial()
    t2.step(5)
    check(sk.flowed() == 0.0 and not sk.losses() and not m.FLOAT.up and t2.score() == 0.0,
          "a reset trial starts with nothing flowed or latched")


def test_not_home():
    t = Trial()
    place_intake(t)
    sink_outlet(t)
    pinch_outlet(t)
    carry_out(t)
    t.let_go("right_gripper")
    t.move("right_gripper", tcp(m.IN_TUB, 0.09), 0.8)
    t.step(800)
    t.let_go("left_gripper", opening=0.6)
    t.move("left_gripper", tcp(m.IN_INTAKE, m.OVER_RIM + 0.03), 0.8)
    t.step(30)
    check(t.score() == 0.9, f"done, arms not home: 0.9 (got {t.score()})")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            print(f"-- {name}")
            fn()
    print("all pass")
