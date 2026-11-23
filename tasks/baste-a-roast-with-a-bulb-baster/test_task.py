"""Offline checks of the baster task, no simulator. The pan, roast and baster states are built by hand from the same
geometry the demo uses; the bulb panels' pushes are set directly (in the simulator they come from the panels' joints).

Covers: the suction law (a squeeze in the air, a dip and a release draw about the squeezed volume; no squeeze, a
squeeze let go in the air, a squeeze never let go, and random squeeze/dip sequences with no release under the surface
draw no more than the passive rise in the tip; volume is conserved); the start state and doing nothing score 0; the
demo's baste; set down vs still held; juices squeezed onto the counter; a tilted squirt; squeezed from the pan's side
strip back into the pan; a near miss beside the roast; the wrong way round (squeezed under the
surface, let go in the air); a weak baste; the pan tipped; the roast knocked over; a new trial; the demo plan's reach.

    python3 test_task.py
"""
import math
import random
import sys
from pathlib import Path

import numpy as np
from yam_arena.task_file import ObjectState, load

HERE = Path(__file__).parent
M = load(HERE / "task.py").module
import bulbkit as bk  # noqa: E402
import capkit as ck  # noqa: E402
import yamkit as kit  # noqa: E402


def check(cond, what):
    print(("ok   " if cond else "FAIL ") + what)
    if not cond:
        raise AssertionError(what)


def st(pos, rot=np.eye(3)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(ck.quat(np.asarray(rot)), float), lin_vel=np.zeros(3),
                       ang_vel=np.zeros(3), low=p - 0.01, high=p + 0.01)


def ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


PUSH = {"v": (0.0, 0.0, 0.0, 0.0)}
M.pushes = lambda objects: list(PUSH["v"])
PAN0 = (M.PAN_C[0], M.PAN_C[1], M.FLOOR_Z)
ROAST0 = (M.PAN_C[0], M.PAN_C[1], M.FLOOR_Z + M.ROAST[2] / 2)
DIP = (M.DIP_XY[0], M.DIP_XY[1], M.FLOOR_Z + 0.004)
HI = (DIP[0], DIP[1], DIP[2] + 0.05)
AIM = (M.PAN_C[0], M.PAN_C[1], M.ROAST_TOP_Z + 0.025)
LIE = (M.BASTER_START, M.R_LIE)


class Trial:
    def __init__(self):
        kit._EPISODE["n"] = kit._EPISODE.get("n", 0) + 1
        M._L = None
        M.LINK_POSES.clear()
        self.k = 0
        self.pan, self.panR, self.roast, self.roastR = PAN0, np.eye(3), ROAST0, np.eye(3)
        self.score = None

    def run(self, steps, tip, R, push=0.0, both=False):
        PUSH["v"] = (push, push, push if both else 0.0, push if both else 0.0)
        for _ in range(int(steps)):
            self.k += 1
            kit._EPISODE["last"] = self.k
            o = {"pan": st(self.pan, self.panR), "roast": st(self.roast, self.roastR), "baster": st(tip, R)}
            self.score = M.score(o)
        return self

    @property
    def L(self):
        return M._L


def baste(t, squeeze=0.012, out=0.014, aim=AIM, R_out=M.R_DOWN):
    t.run(10, *LIE).run(20, HI, M.R_DOWN).run(30, HI, M.R_DOWN, squeeze).run(20, DIP, M.R_DOWN, squeeze)
    t.run(40, DIP, M.R_DOWN, 0.0).run(20, HI, M.R_DOWN).run(30, aim, R_out).run(40, aim, R_out, out)
    return t.run(20, aim, R_out)


def total(t):
    L = t.L
    return L.pan_ml + L.bulb.liquid + L.on_roast + L.counter


# -- the law ---------------------------------------------------------------------------------------------------------------
B = M.BASTER_KIT


def law(seq):
    s = bk.Bulb(B)
    for dur, push, depth in seq:
        for _ in range(int(round(dur / M.DT))):
            s.step([push, push, 0.0, 0.0], 1.0, depth, M.DT)
    return s


D = 0.007
s = law([(1.5, M.SQUEEZE, -0.03), (1.0, M.SQUEEZE, D), (2.0, M.HOLD, D), (1.0, M.HOLD, -0.1)])
check(28.0 <= s.liquid <= 36.0, f"the demo's squeeze in the air, dip, ease to the hold: draws {s.liquid:.1f} mL (28-36)")
check(abs(s.liquid - s.drawn + s.back + s.out_air) < 1e-6, "law: inside = drawn - back - out")
passive = law([(2.0, 0.0, D), (1.0, 0.0, -0.1)]).drawn
check(passive < 0.2, f"never squeezed, dipped: only the passive rise in the tip ({passive:.3f} mL)")
s = law([(1.5, 0.012, -0.03), (1.0, 0.0, -0.03), (2.0, 0.0, D), (1.0, 0.0, -0.1)])
check(s.drawn < 0.2, f"squeezed and let go in the air, then dipped: nothing drawn ({s.drawn:.3f} mL)")
s = law([(1.5, 0.012, -0.03), (3.0, 0.012, D), (1.0, 0.012, -0.1)])
check(s.drawn < 0.2, f"squeeze held while dipped, never let go: nothing drawn ({s.drawn:.3f} mL)")
s = law([(1.0, 0.0, D), (1.5, 0.012, D), (2.0, 0.0, D), (1.0, 0.0, -0.1)])
check(24.0 <= s.liquid <= 30.0, f"squeezed under the surface (air bubbles out), released: draws {s.liquid:.1f} mL")
s = law([(1.0, 0.0, D), (1.5, M.SQUEEZE, D), (2.0, M.SQUEEZE, -0.05), (1.5, M.HOLD, -0.05), (1.0, M.HOLD, -0.1)])
check(s.liquid < 0.2, f"wrong way (squeezed under the surface, let go after the lift): nothing kept ({s.liquid:.3f} mL)")
s = law([(1.0, 0.0, D)] + [(1.0, 0.0, D), (1.0, 0.0, -0.05)] * 20)
check(s.drawn - s.out_air - s.back < 0.2, "20 dips with no squeeze: nothing kept")
bound = B.column_ml(0.012) * 1.05  # the bore over the deepest dip: the passive rise in the tip
HELD_MAX = 2.0  # mL: what re-dips can stack in the tip (the meniscus's surface tension holds a few mm more column
# between dips, and a tilted dip compresses the trapped air a little); far under the 8 mL the score's lowest step needs
DOWNS = [1.0, 0.7, 0.2, 0.06, 0.04, 0.0, -0.5, -1.0]


def fuzz(seed, n, tilts):
    """Random squeeze / dip / tilt sequences with no release under the surface (a push never eases while the tip
    stays under). Returns the most ever inside at once, and the most kept over the trial per dip."""
    rng = random.Random(seed)
    most, per_dip, kept = 0.0, 0.0, 0.0
    for _ in range(n):
        s = bk.Bulb(B)
        push, p3, under, dips = 0.0, 0.0, False, 0
        for _ in range(15):
            nu = rng.random() < 0.5
            nxt = rng.choice([0.0, 0.004, 0.008, 0.012, 0.016])
            n3 = rng.choice([0.0, nxt])
            if under and nu:  # still under: no easing
                nxt, n3 = max(nxt, push), max(n3, p3)
            if nu and not under:  # dipping keeps the squeeze as it was
                nxt, n3 = push, p3
                dips += 1
            push, p3, under = nxt, n3, nu
            down = rng.choice(DOWNS) if tilts else 1.0
            depth = rng.uniform(0.002, 0.012) if under else -0.05
            for _ in range(rng.randint(1, 30)):
                s.step([push, push, p3, 0.0], down, depth, M.DT)
                most = max(most, s.liquid)
        per_dip = max(per_dip, (s.drawn - s.back) / max(1, dips))
        kept = max(kept, s.drawn - s.back)
    return most, per_dip, kept


most, per_dip, kept = fuzz(7, 120, False)
check(most < HELD_MAX and per_dip < bound and kept < M.SOME_ML, f"120 random squeeze/dip sequences, tip-down, no release under the surface: at "
      f"most {most:.3f} mL inside at once, {per_dip:.3f} mL kept per dip (passive rise {bound:.3f}), {kept:.2f} mL kept in all")
most, per_dip, kept = fuzz(11, 200, True)
check(most < HELD_MAX and per_dip < bound and kept < M.SOME_ML, f"200 such sequences also held level, tilted and tip-up: at most {most:.3f} mL "
      f"inside, {per_dip:.3f} mL kept per dip, {kept:.2f} mL kept in all")
# the red team's loophole: squeeze, dip still squeezed, lift out, turn level and let go in the air, lower the level
# tip into the juices, tilt the bulb up: the air vented while level, so nothing is drawn


def axis_down(down):
    s_ = math.sqrt(max(0.0, 1 - down * down))
    x = np.array([s_, 0.0, down])
    y = np.array([0.0, 1.0, 0.0])
    return np.stack([x, y, np.cross(x, y)], axis=1)


s = bk.Bulb(B)
for dur, push, down, depth in ((1.5, 0.013, 1.0, -0.03), (1.0, 0.013, 1.0, D), (1.0, 0.013, 1.0, -0.05),
                               (0.5, 0.013, 0.0, -0.05), (1.0, 0.0, 0.0, -0.05), (1.0, 0.0, 0.0, D),
                               (2.0, 0.0, 0.3, D), (1.0, 0.0, 1.0, D)):
    for _ in range(int(round(dur / M.DT))):
        s.step([push, push, 0.0, 0.0], down, depth, M.DT)
check(s.drawn - s.back < bound, f"red-team loophole (let go in the air held level, then dipped): {s.drawn - s.back:.3f} mL")
s = bk.Bulb(B)
for dur, push, down, depth in ((1.0, 0.0, 1.0, D), (1.0, 0.0, 1.0, -0.05), (1.0, 0.013, -1.0, -0.05),
                               (0.2, 0.013, 1.0, -0.05)):
    for _ in range(int(round(dur / M.DT))):
        s.step([push, push, 0.0, 0.0], down, depth, M.DT)
check(abs(s.p) < 0.5, f"squeezed tip-up in the air: no stored pressure ({s.p:.2f} kPa)")
# the bulb's wall limits the vacuum: let go at once with the tip under, the inside never goes below what the panels'
# springs hold at full travel
s = bk.Bulb(B)
low = 0.0
for push, depth, n in ((0.016, -0.03, 20), (0.016, D, 10), (0.0, D, 40)):
    for _ in range(n):
        s.step([push, push, 0.0, 0.0], 1.0, depth, M.DT, sub=40)
        low = min(low, s.p)
lim = M.K_WALL * M.TRAVEL_LR / (M.PANEL_CM2 * 1e-4) / 1000.0
check(low > -lim, f"let go at once under the surface: the vacuum stays within what the wall holds ({low:.1f} > -{lim:.0f} kPa)")
F, tq = M.juice_wrench((0.3, 0.0, 0.8), M.R_DOWN, 25.0, (0.3, 0.0, 0.8 + 0.2))
check(abs(F[2] + 0.025 * 9.81 * 0.998) < 2e-3 and np.linalg.norm(tq) < 1e-9,
      f"25 mL inside weighs {-F[2]:.3f} N, straight under the balance point when hanging (no torque)")
s = law([(1.5, 0.012, -0.03), (1.0, 0.012, D), (2.0, 0.0, D), (3.0, 0.0, -0.1)])
check(s.out_air < 0.5, f"a filled baster held tip-down in the air for 3 s drips {s.out_air:.2f} mL (held by its vacuum)")
s = law([(1.5, 0.012, -0.03), (1.0, 0.012, D), (2.0, 0.0, D), (1.0, 0.0, -0.1), (2.0, 0.014, -0.1)])
check(s.liquid < 0.5 and s.out_air > 24.0, f"squeezed out in the air: {s.out_air:.1f} mL out")

# -- the scorer -------------------------------------------------------------------------------------------------------
t = Trial().run(30, *LIE)
check(t.score == 0.0, "start state: 0")
t = Trial().run(200, *LIE)
check(t.score == 0.0, "doing nothing: 0")
t = baste(Trial())
check(abs(total(t) - M.JUICE_ML) < 1e-6, "volume conserved (pan + baster + roast + counter)")
check(t.L.on_roast >= M.GOOD_ML and t.L.counter <= M.SPILL_OK_ML, f"demo baste: {t.L.on_roast:.1f} mL on the roast")
check(t.score == 0.75, "basted, baster still held over the roast: 0.75")
t.run(40, *LIE)
check(t.score == 1.0, "basted, then the baster set down on the counter and still for 1 s: 1.0")
t = baste(Trial(), squeeze=0.0)
check(t.score == 0.0, "no squeeze before the dip: 0")
t = baste(Trial(), aim=(0.30, 0.30, 0.90))
check(t.score == 0.0 and t.L.counter > 20, f"squeezed out over the counter: 0 ({t.L.counter:.1f} mL on it)")
t = baste(Trial(), aim=DIP[:2] + (M.ROAST_TOP_Z + 0.025,))
check(t.score == 0.0 and t.L.on_roast < 1.0, "squeezed back into the pan's juices beside the roast: 0")
NEAR = (M.PAN_C[0] - M.ROAST[0] / 2 - 0.015, M.PAN_C[1], M.ROAST_TOP_Z + 0.025)
t = baste(Trial(), aim=NEAR)
check(t.score == 0.0 and t.L.on_roast < 1.0, "near miss (squeezed out 1.5 cm off the roast's near edge): 0")
o = {"pan": st(PAN0), "roast": st(ROAST0), "baster": st(AIM, M.R_DOWN)}
up15 = np.array([M.PAN_C[0], M.PAN_C[1], M.ROAST_TOP_Z + 0.15])
check(M.land(o, up15, np.array([0, 0, -1.0]), M.JUICE_ML)[0] == "roast", "a jet straight down from 15 cm lands on the top")
w, p = M.land(o, up15, np.array([2.0, 0, -0.5]), M.JUICE_ML)
check(w != "roast", f"a jet squirted sideways at 2 m/s from 15 cm overshoots the top (lands: {w})")
w, p = M.land(o, up15, np.array([0, 3.0, 0.0]), M.JUICE_ML)
check(w == "counter", f"a jet squirted sideways at 3 m/s from 15 cm flies over the rim onto the counter (lands: {w})")
t = baste(Trial(), squeeze=0.005)
check(t.score == 0.25 and M.SOME_ML <= t.L.on_roast < M.GOOD_ML, f"a weak squeeze: 0.25 ({t.L.on_roast:.1f} mL)")
t = Trial()
t.panR = ry(math.radians(12))
baste(t)
check(t.score == 0.0, "the pan tipped 12 degrees: 0")
t = Trial()
t.roastR = ry(math.radians(90))
baste(t)
check(t.L.on_roast == 0.0, "the roast knocked onto its side: nothing lands on its top")
t = baste(Trial())
t.run(5, *LIE)
t2 = Trial().run(10, *LIE)
check(t2.L.on_roast == 0.0 and t2.score == 0.0, "a new trial starts clean")

# -- the start layout and the demo -----------------------------------------------------------------------------------------
check(11.0 < M.LEVEL0 * 1000 < 16.0, f"3 cups stand {M.LEVEL0 * 1000:.1f} mm deep around the roast")
check(M.DIP_XY[1] - M.TUBE_R[0] > M.PAN_C[1] - M.PAN_IN[1] / 2 + 0.015
      and M.DIP_XY[1] + M.TUBE_R[0] < M.PAN_C[1] - M.ROAST[1] / 2 - 0.015, "the dip spot lies in the side strip of juices")
check(39.0 < B.bore_ml() < 42.0, f"the tube's bore holds {B.bore_ml():.1f} mL (40 mL markings)")
d = M.demo()
check(abs(sum(w.duration for w in d["left"]) - sum(w.duration for w in d["right"])) < 1e-6, "demo: arms end together")
check(sum(w.duration for w in d["left"]) / M.DT < M.RUN_LIMIT_STEPS, "demo fits the run limit")
for var in ("nothing", "no_squeeze", "air_release", "wrong_way", "near_miss"):
    M.os.environ["BASTER_VARIANT"] = var
    dv = M.demo()
    check(sum(w.duration for w in dv["left"]) / M.DT < M.RUN_LIMIT_STEPS, f"variant {var} fits the run limit")
M.os.environ.pop("BASTER_VARIANT")
# the left's tip path along the demo (straight lines between its tip-down waypoints) clears the roast and the walls
worst_roast, worst_wall, prev = 1.0, 1.0, None
for w in d["left"]:
    if not isinstance(w, kit.Tilted) or abs(w.pitch - M.PITCH_FWD) > 1e-6:
        prev = None
        continue
    tip = np.asarray(w.pos) - np.array([0, 0, M.PANEL_MID - M.HELD_DZ])
    if prev is not None:
        for f in np.linspace(0, 1, 60):
            p = prev + (tip - prev) * f
            dx, dy = abs(p[0] - M.PAN_C[0]), abs(p[1] - M.PAN_C[1])
            if dx <= M.ROAST[0] / 2 + M.TUBE_R[0] and dy <= M.ROAST[1] / 2 + M.TUBE_R[0]:
                worst_roast = min(worst_roast, p[2] - M.ROAST_TOP_Z)
            near_wall = (abs(dx - M.PAN_IN[0] / 2) <= M.WALL_T + M.TUBE_R[0] and dy <= M.PAN_OUT[1] / 2) or (
                abs(dy - M.PAN_IN[1] / 2) <= M.WALL_T + M.TUBE_R[0] and dx <= M.PAN_OUT[0] / 2)
            if near_wall:
                worst_wall = min(worst_wall, p[2] - (M.FLOOR_Z + M.RIM_H))
    prev = tip
check(worst_roast >= 0.02, f"demo: the tip passes at least 2 cm over the roast's top ({worst_roast * 1000:.0f} mm)")
check(worst_wall >= 0.02, f"demo: the tip crosses the pan's walls at least 2 cm over the rim ({worst_wall * 1000:.0f} mm)")
print("all ok")
