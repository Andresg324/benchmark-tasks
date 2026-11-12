"""Offline checks of the concrete-cylinder scorer: step-by-step scoop, rod and float poses through dosekit's bulk
model, near misses included, and a replay of the scripted demo's waypoints for both arms (perfect tracking), no
simulator.

    PY=R/venv/bin/python R/yam-kits/chat/offline/runtest.sh test_task.py      (prints "all pass")
"""

import importlib.util
import math
import os

import numpy as np
from yam_arena.task_file import ObjectState

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("cylinder_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit, dk, pk = m.kit, m.dk, m.pk

TOP = m.TOP
PARK = {"left_gripper": np.array([0.20, 0.27, TOP + 0.20]), "right_gripper": np.array([0.20, -0.27, TOP + 0.20])}
MX, MY = m.MOLD
RIM = m.RIM
FLOOR = TOP + m.MOLD_FLOOR
SCOOP0, ROD0, FLOAT0 = m.HANDLE0.copy(), np.asarray(m.ROD0, float).copy(), np.array([*m.FLOAT, TOP])
TIP = np.asarray(m.TIP, float)
PLATE_TOP = m.PLATE[2]


def qy(deg):
    a = math.radians(deg) / 2
    return np.array([0.0, math.sin(a), 0.0, math.cos(a)])


DOWN = (0.70711, 0.70711, 0.0, 0.0)  # the straight-down grip (yam_arena's DOWN_XYZW): fingers closing along world y


def q_of(rm):
    """A rotation matrix as an x, y, z, w quaternion."""
    w = math.sqrt(max(0.0, 1 + rm[0, 0] + rm[1, 1] + rm[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + rm[0, 0] - rm[1, 1] - rm[2, 2])) / 2, rm[2, 1] - rm[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - rm[0, 0] + rm[1, 1] - rm[2, 2])) / 2, rm[0, 2] - rm[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - rm[0, 0] - rm[1, 1] + rm[2, 2])) / 2, rm[1, 0] - rm[0, 1])
    return (x, y, z, w)


def leaned(deg):
    """The straight-down hand leaned `deg` about world x, its top away from the mold (toward -y)."""
    a = math.radians(deg)
    rx = np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])
    return q_of(rx @ pk._rotm(DOWN))


def reach(z, q=DOWN):
    """How far off the mold's wall (on its -y side) the tool point is when the hand's outline below the rim, turned to
    q, just touches the wall."""
    rot = pk._rotm(q)
    d = 0.03
    for _ in range(6):
        tcp = np.array([MX, MY - m.MOLD_R_OUT - d, z])
        pts = tcp + (rot @ m._HAND_PTS.T).T
        pts = pts[pts[:, 2] <= m.RIM - 0.001]
        d -= float(np.min(np.hypot(pts[:, 0] - MX, pts[:, 1] - MY))) - m.MOLD_R_OUT
    return d


def st(pos, q=(0, 0, 0, 1.0), vel=(0, 0, 0), half=0.01):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(q, float), lin_vel=np.array(vel, float), ang_vel=np.zeros(3),
                       low=p - half, high=p + half)


class Trial:
    """A fresh trial: the scene at its start, the arms parked at home. Tools are moved by hand (teleported); while a
    tool moves, the arm holding it follows its grip."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.scoop, self.q = SCOOP0.copy(), np.array([0, 0, 0, 1.0])
        self.rod, self.flt = ROD0.copy(), FLOAT0.copy()
        self.g = {k: v.copy() for k, v in PARK.items()}
        self.vel = {"scoop": np.zeros(3), "rod": np.zeros(3), "float": np.zeros(3)}
        self.gq = {}  # gripper orientations other than straight down
        self.out = None
        self.step()

    def objs(self):
        return {"mold": st((MX, MY, TOP)), "pan": st((*m.PAN, TOP)), "stand": st((*m.STAND, TOP)),
                "rack": st((*m.RACK, TOP)), "scoop": st(self.scoop, self.q, self.vel["scoop"]),
                "rod": st(self.rod, vel=self.vel["rod"]), "float": st(self.flt, vel=self.vel["float"]),
                **{k: st(v, self.gq.get(k, DOWN)) for k, v in self.g.items()}}

    def step(self, n=1):
        for _ in range(n):
            o = self.objs()
            dk.update(o)
            ok = m.success(o)
            self.out = (m.score(o), ok)
        return self.out

    # -- the scoop (right hand follows its handle) --
    def put(self, mouth, tilt=0.0, n=1):
        q = qy(tilt)
        target = np.asarray(mouth, float) - pk._rotm(q) @ m.MOUTH_OFF
        start = self.scoop.copy()
        for k in range(1, n + 1):
            self.scoop = start + (target - start) * k / n
            self.q = q
            self.g["right_gripper"] = self.scoop + [0, m.GRIP_DY, 0]
            self.step()

    def put_lip(self, xy, z, tilt):
        q = qy(tilt)
        fr = dk._frame({"scoop": st((0, 0, 0), q)}, m.SCOOP_BIN)
        self.scoop = np.array([xy[0], xy[1], z]) - fr["lip"]
        self.q = q
        self.g["right_gripper"] = self.scoop + [0, m.GRIP_DY, 0]
        self.step()

    def scoopful(self, xy=(MX, MY), heaped=True):
        surf = dk.surface_z(self.objs(), "pan")
        self.put((*m.DIP_XY, m.CARRY_Z), n=4)
        self.put((*m.DIP_XY, surf - 0.008), n=8)
        self.step(10)
        self.put((*m.DIP_XY, m.CARRY_Z), n=6)
        self.put((xy[0] - 0.02, xy[1], m.CARRY_Z), n=10)
        for t in np.linspace(0, 129, 20):
            self.put_lip(xy, m.POUR_LIP_Z, t)
        self.step(20)
        for t in np.linspace(129, 0, 10):
            self.put_lip(xy, m.POUR_LIP_Z, t)

    def scoop_down(self, home=True):
        self.put(SCOOP0 + m.MOUTH_OFF, n=8)
        self.g["right_gripper"] = PARK["right_gripper"].copy() if home else np.array([0.25, -0.15, TOP + 0.12])

    # -- the rod (left hand on its block) --
    def tip_at(self, p):
        self.rod = np.asarray(p, float) - TIP
        self.g["left_gripper"] = self.rod + [0, 0, 0.0095]
        self.step()

    def rod_strokes(self, n, lo, hi, pts=m.FIRST_PTS):
        for k in range(n):
            dx, dy = pts[k % len(pts)]
            self.tip_at((MX + dx, MY + dy, hi))
            self.tip_at((MX + dx, MY + dy, (hi + lo) / 2))
            self.tip_at((MX + dx, MY + dy, lo))
        self.tip_at((MX, MY, hi))

    def layer1_top(self):
        return FLOOR + dk.lifts(self.objs(), "mold")[0].top

    def rod_layer1(self, n=25, lo=FLOOR + 0.002, taps=12, pts=m.FIRST_PTS):
        self.rod_strokes(n, lo, FLOOR + 0.054, pts)
        self.taps(taps)

    def rod_layer2(self, n=25, into=0.011, taps=12, pts=m.SPOKES * 3 + m.FIRST_PTS * 3):
        self.rod_strokes(n, self.layer1_top() - into, RIM + 0.012, pts)
        self.taps(taps)

    # -- taps on the mold's side (right hand, the rod held up out of the mold) --
    def taps(self, n, gap=-0.001, z=m.TAP_Z, q=DOWN):
        """n taps from the mold's -y side, the hand's outline stopping `gap` m off the wall's outer face (negative:
        pressed into it), with the hand turned to `q`."""
        y_in = MY - m.MOLD_R_OUT - reach(z, q) - gap
        self.gq["right_gripper"] = np.array(q, float)
        for _ in range(n):
            self.g["right_gripper"] = np.array([MX, y_in - 0.025, z])
            self.step()
            self.g["right_gripper"] = np.array([MX, y_in, z])
            self.step()
        self.g["right_gripper"] = PARK["right_gripper"].copy()
        self.gq.pop("right_gripper")
        self.step()

    def air_taps(self, n, r_extra, z=m.TAP_Z):
        self.taps(n, gap=r_extra, z=z)

    def tcp_taps(self, n, off, q=DOWN, z=m.TAP_Z):
        """n taps with the tool point stopping `off` m off the wall's outer face, the hand turned to `q`."""
        self.gq["right_gripper"] = np.array(q, float)
        for _ in range(n):
            self.g["right_gripper"] = np.array([MX, MY - m.MOLD_R_OUT - off - 0.025, z])
            self.step()
            self.g["right_gripper"] = np.array([MX, MY - m.MOLD_R_OUT - off, z])
            self.step()
        self.g["right_gripper"] = PARK["right_gripper"].copy()
        self.gq.pop("right_gripper")
        self.step()

    def rim_taps(self, n):
        for _ in range(n):
            self.g["right_gripper"] = np.array([MX, MY - 0.030, RIM + 0.025])
            self.step()
            self.g["right_gripper"] = np.array([MX, MY - 0.030, RIM + 0.012])
            self.step()
        self.g["right_gripper"] = PARK["right_gripper"].copy()
        self.step()

    def pour_lift(self, h):
        """One scoop poured slowly over the mold until a new lift `h` m thick is on top (or the mold is at its brim),
        then laid back."""
        surf = dk.surface_z(self.objs(), "pan")
        self.put((*m.DIP_XY, m.CARRY_Z), n=4)
        self.put((*m.DIP_XY, surf - 0.008), n=8)
        self.step(10)
        self.put((*m.DIP_XY, m.CARRY_Z), n=6)
        self.put((MX - 0.02, MY, m.CARRY_Z), n=10)
        n0, tilt = len(dk.lifts(self.objs(), "mold")), 0
        while tilt < 129:
            L = dk.lifts(self.objs(), "mold")
            if (len(L) > n0 and L[-1].top - L[-1].bottom >= h) or dk.volume(self.objs(), "mold") >= m.MOLD_BIN.brim:
                break
            tilt += 1
            self.put_lip((MX, MY), m.POUR_LIP_Z, tilt)
        for tt in np.linspace(tilt, 0, 5):
            self.put_lip((MX, MY), m.POUR_LIP_Z, tt)
        self.put(SCOOP0 + m.MOUTH_OFF, n=4)

    def pour_until(self, frac):
        """One scoop, poured slowly over the mold until it holds `frac` of its brim volume."""
        surf = dk.surface_z(self.objs(), "pan")
        self.put((*m.DIP_XY, m.CARRY_Z), n=4)
        self.put((*m.DIP_XY, surf - 0.008), n=8)
        self.step(10)
        self.put((*m.DIP_XY, m.CARRY_Z), n=6)
        self.put((MX - 0.02, MY, m.CARRY_Z), n=10)
        tilt = 0
        while tilt < 129 and dk.volume(self.objs(), "mold") < frac * m.MOLD_BIN.brim:
            tilt += 1
            self.put_lip((MX, MY), m.POUR_LIP_Z, tilt)
        for tt in np.linspace(tilt, 0, 5):
            self.put_lip((MX, MY), m.POUR_LIP_Z, tt)

    def rod_back(self):
        self.rod = ROD0.copy()
        self.g["left_gripper"] = PARK["left_gripper"].copy()
        self.step()

    # -- the float (left hand on its knob) --
    def strike(self, dz=0.001, n=40, y0=0.065, y1=-0.065):
        for k in range(n + 1):
            self.flt = np.array([MX, MY + y0 + (y1 - y0) * k / n, RIM + dz])
            self.g["left_gripper"] = self.flt + m.KNOB_C
            self.step()
        self.flt = FLOAT0.copy()
        self.g["left_gripper"] = PARK["left_gripper"].copy()
        self.step()

    def finish(self, n=30):
        return self.step(n)

    def full(self, strike=True):
        self.scoopful()
        self.scoopful()
        self.rod_layer1()
        self.scoopful()
        self.scoopful()
        self.rod_layer2()
        self.rod_back()
        self.scoop_down()
        if strike:
            self.strike()


fails = 0


def expect(name, got, want, extra=""):
    global fails
    ok = abs(got[0] - want) < 1e-9 and got[1] == (want == 1.0)
    fails += not ok
    print(f"{'ok  ' if ok else 'FAIL'} {got[0]:.2f} (want {want:.2f}) {name} {extra}")


def info(t):
    o = t.objs()
    L = dk.lifts(o, "mold")
    return (f"[{dk.amount(o, 'mold'):.0f} g, lifts " + ", ".join(f"{x.bottom * 100:.1f}-{x.top * 100:.1f} cm"
                                                                  for x in L)
            + f", heap {dk.heap(o, 'mold') * 1000:.1f} mm, vol {dk.volume(o, 'mold') / m.MOLD_BIN.brim:.3f} brim; "
            + m.layers_ok(o)[1] + "]")


print(f"mold {m.MOLD_BIN.brim:.1f} mL; scoop level {m.SCOOP_BIN.brim:.1f} mL; layer 1 window "
      f"{m.L1_LO * 100:.2f}-{m.L1_HI * 100:.2f} cm; rod tip to tool point {m.ROD_DZ * 100:.1f} cm")

t = Trial()
expect("do nothing", t.finish(40), 0.0, info(t))

t = Trial()
t.full()
expect("two layers, 25 strokes each, struck off, tools down, arms home", t.finish(), 1.0, info(t))

t = Trial()
t.full(strike=False)
t.g["left_gripper"] = PARK["left_gripper"].copy()
expect("layers right, top left heaped (not struck)", t.finish(), 0.25, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.scoopful(), t.rod_layer2(), t.rod_back()
t.scoop_down(home=False)
t.strike()
expect("full job, right arm not home", t.finish(), 0.9, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.scoopful(), t.rod_layer2(), t.rod_back()
t.put(SCOOP0 + m.MOUTH_OFF, n=8)
t.strike()
expect("full job, still holding the scoop", t.finish(), 0.0, info(t))
t.g["right_gripper"] = t.scoop + [0, 0, 0.041]
expect("... hand lifted 4.1 cm off the handle (let go)", t.finish(), 0.9)

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.scoopful(), t.rod_layer2()
t.scoop_down()
t.g["left_gripper"] = PARK["left_gripper"].copy()
t.strike()
t.g["left_gripper"] = t.rod + [0, 0, 0.0095]
expect("full job, rod still in hand over the mold", t.finish(), 0.0, info(t))
t.g["left_gripper"] = PARK["left_gripper"].copy()
expect("... the rod let go standing in the mold", t.finish(), 0.0)

# held by any part, not only its grip (Nov 11 review rule): fingers closed on the rod's shaft or the scoop's bowl
t = Trial()
t.full()
expect("full job (control)", t.finish(), 1.0)
t.g["left_gripper"] = t.rod + np.array([0.0, 0.0, -0.06])  # 6 cm down the shaft, 5 cm below the block's center
expect("full job, left hand closed on the rod's shaft at the end", t.finish(), 0.0)
t = Trial()
t.full()
t.g["right_gripper"] = t.scoop + m.MOUTH_OFF + np.array([0.0, 0.0, -0.01])  # in the bowl, 5.6 cm from the handle
expect("full job, right hand closed on the scoop's bowl at the end", t.finish(), 0.0)
t = Trial()
t.full()
t.g["left_gripper"] = t.flt + np.array([0.045, 0.0, 0.01])  # on the blade's end, 4.6 cm from the knob
expect("full job, left hand on the float's blade end at the end", t.finish(), 0.0)

t = Trial()
t.full()
t.vel["float"] = np.array([0.0, 0.05, 0.0])
expect("full job, float still sliding", t.step(), 0.0)
t = Trial()
t.full()
t.g["right_gripper"] = t.scoop + np.array([0.0, 0.0, 0.045])  # a hand left 4.5 cm over the handle (not on it)
t.g["left_gripper"] = np.array([0.24, 0.27, TOP + 0.20])  # left arm parked 4 cm off its start
expect("full job, left arm 4 cm off its start pose", t.finish(), 0.9)
t = Trial()
t.full()
t.g["left_gripper"] = t.flt + np.array([0.0, 0.0, PLATE_TOP + 0.005])  # a hand lying on the blade, 5 mm over it
expect("full job, left hand resting on the float's blade (not let go)", t.finish(), 0.0)

# fast but correct motions still count (the kit tests the band crossed between steps)
t = Trial()
t.scoopful()
t.scoopful()
for k in range(25):  # each stroke one step down, one step up
    dx, dy = m.FIRST_PTS[k % 5]
    t.tip_at((MX + dx, MY + dy, FLOOR + 0.054))
    t.tip_at((MX + dx, MY + dy, FLOOR + 0.002))
t.tip_at((MX, MY, FLOOR + 0.054))
t.taps(12)
t.scoopful()
t.scoopful()
t.rod_layer2()
t.rod_back()
t.scoop_down()
t.strike(n=4)  # the float drawn across the 4.1 cm mouth in 4 steps (0.2 s)
expect("fast strokes (one step each way) and a fast strike-off", t.finish(), 1.0, info(t))

# the core mistakes
t = Trial()
for _ in range(4):
    t.scoopful()
t.rod_strokes(50, FLOOR + 0.002, RIM + 0.012)
t.rod_back(), t.scoop_down(), t.strike()
expect("filled in one go, rodded 50 times to the bottom, struck", t.finish(), 0.0, info(t))

t = Trial()
for _ in range(4):
    t.scoopful()
t.rod_back(), t.scoop_down(), t.strike()
expect("filled in one go, not rodded, struck", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.scoopful(), t.scoopful()
t.rod_layer2()
t.rod_back(), t.scoop_down(), t.strike()
expect("filled in one go, rodded only the top half", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(n=24), t.scoopful(), t.scoopful(), t.rod_layer2(), t.rod_back()
t.scoop_down(), t.strike()
expect("first layer rodded 24 times", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(n=10), t.scoopful(), t.scoopful()
t.rod_strokes(40, FLOOR + 0.002, RIM + 0.012)
t.rod_back(), t.scoop_down(), t.strike()
expect("first layer rodded 10 times, then 40 strokes to the floor through both", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(lo=FLOOR + 0.020), t.scoopful(), t.scoopful(), t.rod_layer2(), t.rod_back()
t.scoop_down(), t.strike()
expect("first layer rodded 25 times, stopping 2 cm off the floor", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.scoopful(), t.rod_layer2(into=-0.002), t.rod_back()
t.scoop_down(), t.strike()
expect("second layer rodded 25 times, stopping 2 mm above the first", t.finish(), 0.0, info(t))

for into, want in ((0.011, 1.0), (0.007, 0.0), (0.004, 0.0)):
    t = Trial()
    t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.scoopful(), t.rod_layer2(into=into), t.rod_back()
    t.scoop_down(), t.strike()
    expect(f"second layer's strokes {into * 1000:.0f} mm into the first", t.finish(), want, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.scoopful(), t.rod_layer2(n=24), t.rod_back()
t.scoop_down(), t.strike()
expect("second layer rodded 24 times", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.rod_layer1(), t.scoopful(), t.rod_layer2(), t.scoopful(), t.scoopful()
t.rod_strokes(25, RIM - 0.03, RIM + 0.012)
t.rod_back(), t.scoop_down(), t.strike()
expect("three layers (one, one, two scoops), each rodded", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.rod_layer1(), t.scoopful(), t.scoopful(), t.scoopful(), t.rod_layer2(), t.rod_back()
t.scoop_down(), t.strike()
expect("first layer one scoop (a quarter of the mold), second three", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.scoopful(), t.rod_strokes(25, FLOOR + 0.002, RIM + 0.012), t.scoopful()
t.rod_layer2(), t.rod_back()
t.scoop_down(), t.strike()
expect("first layer three scoops (over two thirds), second one", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.rod_layer2(), t.rod_back(), t.scoop_down()
t.strike()
expect("two layers rodded right, the mold left a scoop short (sunk top)", t.finish(), 0.25, info(t))
t.scoopful()
t.scoop_down()
t.strike()
expect("... topped up with a whole scoop (1.7 cm unrodded on top) and struck", t.finish(), 0.0, info(t))

# a small top-up after the second layer's rodding (C31 allows it), then struck: full credit
t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.pour_until(1.0), t.rod_layer2()
t.pour_until(1.01)
t.rod_back(), t.scoop_down(), t.strike()
o = t.objs()
expect("second layer to the brim, rodded, a small top-up (under 5 mm) struck", t.finish(), 1.0,
       info(t) + f" top-up {(dk.lifts(o, 'mold')[-1].top - dk.lifts(o, 'mold')[-1].bottom) * 1000:.1f} mm")

# QC attacks (Nov 11): a careful top-up stopped just under the brim with no strike-off
t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.rod_layer2()
t.pour_until(0.985)
t.rod_back(), t.scoop_down()
expect("three scoops rodded, a slow top-up to 98.5% of the brim, never struck", t.finish(), 0.0, info(t))
# struck first, then a little more concrete added and left unstruck
t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.pour_until(1.0), t.rod_layer2()
t.strike()
t.pour_until(1.0)
t.rod_back(), t.scoop_down()
o = t.objs()
expect("struck, then a small top-up added and not struck again", t.finish(), 0.25,
       info(t) + f" swept after {m.struck_after_last(o):.2f}")
# every stroke within 5.2 mm of the axis (a 1 cm circle in a 4.1 cm bore)
C = [(0.0052, 0.0), (0.0, 0.0052), (-0.0052, 0.0), (0.0, -0.0052), (0.0, 0.0)]
t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(pts=C), t.scoopful(), t.scoopful(), t.rod_layer2(pts=C)
t.rod_back(), t.scoop_down(), t.strike()
expect("all strokes within 5.2 mm of the axis in both layers", t.finish(), 0.0, info(t))
for r_extra, want in ((0.004, 0.0), (0.002, 0.0)):
    t = Trial()
    t.scoopful(), t.scoopful(), t.rod_layer1(taps=0), t.air_taps(12, r_extra)
    t.scoopful(), t.scoopful(), t.rod_layer2(taps=0), t.air_taps(12, r_extra)
    t.rod_back(), t.scoop_down(), t.strike()
    expect(f"taps stopping {r_extra * 1000:.0f} mm short of the wall (no contact)", t.finish(), want, info(t))
# QC 2 (Nov 11): taps judged on the hand's real outline, turned with the hand
print(f"straight-down hand at the tap height: meets the wall with the tool point {reach(m.TAP_Z) * 1000:.1f} mm "
      f"off it; leaned 15 deg {reach(m.TAP_Z, leaned(15)) * 1000:.1f} mm")
for gap, want in ((0.0015, 1.0), (0.0025, 0.0)):
    t = Trial()
    t.scoopful(), t.scoopful(), t.rod_layer1(taps=0), t.taps(12, gap=gap)
    t.scoopful(), t.scoopful(), t.rod_layer2(taps=0), t.taps(12, gap=gap)
    t.rod_back(), t.scoop_down(), t.strike()
    expect(f"taps with the hand's outline stopping {gap * 1000:.1f} mm off the wall", t.finish(), want, info(t))
for off, q, want, name in ((reach(m.TAP_Z), DOWN, 1.0, "upright hand, light tap (outline just touching)"),
                           (0.016, leaned(15), 0.0, "hand leaned 15 deg, tool point 16 mm off the wall (air gap)"),
                           (reach(m.TAP_Z) + 0.0025, DOWN, 0.0, "upright hand stopping 2.5 mm short of the wall")):
    t = Trial()
    t.scoopful(), t.scoopful(), t.rod_layer1(taps=0), t.tcp_taps(12, off, q)
    t.scoopful(), t.scoopful(), t.rod_layer2(taps=0), t.tcp_taps(12, off, q)
    t.rod_back(), t.scoop_down(), t.strike()
    expect(f"{name}", t.finish(), want, info(t) + f" tool point {off * 1000:.1f} mm off")
t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(taps=0), t.taps(12, gap=-0.001, q=leaned(15))
t.scoopful(), t.scoopful(), t.rod_layer2(taps=0), t.taps(12, gap=-0.001, q=leaned(15))
t.rod_back(), t.scoop_down(), t.strike()
expect("hand leaned 15 deg, its fingertips meeting the wall", t.finish(), 1.0, info(t))
# top-ups of 4.5 mm each, a rod poke between them (each a lift of its own): 5 mm in all at most
t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.rod_layer2()
for _ in range(6):
    if dk.volume(t.objs(), "mold") >= 0.995 * m.MOLD_BIN.brim:
        break
    t.pour_lift(0.0045)
    top = FLOOR + dk.lifts(t.objs(), "mold")[-1].top
    t.tip_at((MX, MY, RIM + 0.01)), t.tip_at((MX, MY, top - 0.005)), t.tip_at((MX, MY, RIM + 0.01))
t.rod_back(), t.scoop_down(), t.strike()
o = t.objs()
expect("chained top-ups under 5 mm each with a poke between, struck", t.finish(), 0.0,
       info(t) + f" {len(dk.lifts(o, 'mold')) - 2} top-ups")
t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(taps=0), t.rim_taps(12)
t.scoopful(), t.scoopful(), t.rod_layer2(taps=0), t.rim_taps(12)
t.rod_back(), t.scoop_down(), t.strike()
expect("hand bobbing 1.2 to 2.5 cm over the rim instead of tapping", t.finish(), 0.0, info(t))
t = Trial()
t.full()
t.flt = np.array([MX + 0.03, MY, RIM])  # blade lying across the rim, the hand gone
t.step()
expect("full job, float left lying on the mold", t.finish(), 0.0, info(t))

t = Trial()
t.full(strike=False)
t.strike(dz=0.006)
expect("float drawn 6 mm above the rim: top still heaped", t.finish(), 0.25, info(t))

# C31's distribution and taps
for name, pts in (("all 25 strokes of each layer in one spot (on the axis)", [(0.0, 0.0)]),
                  ("all 25 strokes of each layer in one spot (8 mm off the axis)", [(0.008, 0.0)]),
                  ("strokes in two spots", [(0.008, 0.0), (-0.008, 0.0)]),
                  ("strokes in three zones", [(0.008, 0.0), (-0.008, 0.0), (0.0, 0.0)])):
    t = Trial()
    t.scoopful(), t.scoopful(), t.rod_layer1(pts=pts), t.scoopful(), t.scoopful(), t.rod_layer2(pts=pts)
    t.rod_back(), t.scoop_down(), t.strike()
    expect(name, t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(pts=m.SPOKES), t.scoopful(), t.scoopful(), t.rod_layer2(pts=m.SPOKES)
t.rod_back(), t.scoop_down(), t.strike()
expect("strokes in the four sectors, none on the axis (4 of 5 zones)", t.finish(), 1.0, info(t))

t = Trial()
t.scoopful(), t.scoopful()
t.rod_strokes(50, FLOOR + 0.002, FLOOR + 0.054, [(0.0, 0.0)])
t.taps(12)
t.scoopful(), t.scoopful()
t.rod_strokes(50, t.layer1_top() - 0.011, RIM + 0.012, [(0.0, 0.0)])
t.taps(12), t.rod_back(), t.scoop_down(), t.strike()
expect("50 strokes per layer, all in one spot", t.finish(), 0.0, info(t))

for n1, n2, want in ((10, 12, 1.0), (15, 15, 1.0), (9, 12, 0.0), (12, 9, 0.0), (16, 12, 0.0), (12, 0, 0.0)):
    t = Trial()
    t.scoopful(), t.scoopful(), t.rod_layer1(taps=n1), t.scoopful(), t.scoopful(), t.rod_layer2(taps=n2)
    t.rod_back(), t.scoop_down(), t.strike()
    expect(f"taps {n1} and {n2} after rodding the layers", t.finish(), want, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.taps(12), t.rod_layer1(taps=0), t.scoopful(), t.scoopful(), t.rod_layer2()
t.rod_back(), t.scoop_down(), t.strike()
expect("first layer tapped before rodding instead of after", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(taps=0), t.scoopful(), t.scoopful(), t.taps(12), t.rod_layer2()
t.rod_back(), t.scoop_down(), t.strike()
expect("first layer's taps made after the second layer went in", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful(), t.rod_layer1(), t.scoopful(), t.scoopful(), t.rod_layer2(taps=0)
t.taps(12, gap=0.010)
t.rod_back(), t.scoop_down(), t.strike()
expect("second layer's taps stopping 1 cm short of the mold (hovering)", t.finish(), 0.0, info(t))

# cheap tricks
t = Trial()
t.rod_strokes(60, FLOOR + 0.002, RIM + 0.012)
t.rod_back()
expect("rodding the empty mold", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful()
for k in range(25):
    t.tip_at((MX, MY, t.layer1_top() + 0.02))
    t.tip_at((MX, MY, t.layer1_top() + 0.001))
t.tip_at((MX, MY, RIM + 0.02))
t.scoopful(), t.scoopful(), t.rod_layer2(), t.rod_back(), t.scoop_down(), t.strike()
expect("first layer: rod hovered and grazed the surface 25 times", t.finish(), 0.0, info(t))

t = Trial()
t.scoopful(), t.scoopful()
t.g["left_gripper"] = np.array([MX, MY, FLOOR + 0.01])
for _ in range(25):
    t.g["left_gripper"] = np.array([MX, MY, FLOOR + 0.01])
    t.step(2)
    t.g["left_gripper"] = np.array([MX, MY, RIM + 0.02])
    t.step(2)
t.g["left_gripper"] = PARK["left_gripper"].copy()
t.scoopful(), t.scoopful(), t.rod_layer2(), t.rod_back(), t.scoop_down(), t.strike()
expect("first layer poked 25 times with the fingers, not the rod", t.finish(), 0.0, info(t))

t = Trial()
t.rod = np.array([0.22, 0.10, TOP + 0.003])
t.flt = np.array([0.30, 0.25, TOP - 0.30])
expect("rod knocked off its rack, float off the table", t.finish(30), 0.0, info(t))

t = Trial()
t.strike()
expect("striking off the empty mold", t.finish(), 0.0, info(t))

# history reset with kit.episode(): a one-go trial does not carry its record into the next one
t = Trial()
for _ in range(4):
    t.scoopful()
t.rod_strokes(50, FLOOR + 0.002, RIM + 0.012)
t.finish(5)
t = Trial()
t.full()
expect("new trial after a one-go trial", t.finish(), 1.0, info(t))


# --- the scripted demo, replayed with perfect tracking: each arm's tool point follows its waypoints (smoothstep, as
# yamkit's planner), in lockstep; a tool rides in the hand from its "grip" waypoint to its "let go" waypoint.
def track(ws, park, roll0=0.0, pitch0=0.0):
    """Per-step (tool point, roll, pitch, label of the waypoint being reached)."""
    out, tcp, roll, pitch = [], park.copy(), roll0, pitch0
    for wp in ws:
        if isinstance(wp, kit.Home):
            goal, groll, gpitch = park, roll, pitch
        else:
            goal, groll, gpitch = np.array(wp.pos, float), wp.roll, wp.pitch
        n = max(1, int(round(wp.duration / m.DT)))
        start, r0, p0 = tcp.copy(), roll, pitch
        for i in range(1, n + 1):
            s = i / n
            s = s * s * (3 - 2 * s)
            tcp, roll, pitch = start + s * (goal - start), r0 + s * (groll - r0), p0 + s * (gpitch - p0)
            out.append((tcp.copy(), roll, pitch, wp.label if i == n else ""))
    return out


def replay():
    d = m.demo()
    right = track(d["right"], PARK["right_gripper"], m.ROLL, m.P0)
    left = track(d["left"], PARK["left_gripper"])
    n = max(len(right), len(left))
    right += [right[-1][:3] + ("",)] * (n - len(right))
    left += [left[-1][:3] + ("",)] * (n - len(left))
    t = Trial()
    grip_rot = kit._rot_xyz(m.ROLL, m.P0)
    hand = {"scoop": False, "rod": False, "float": False}
    near_arms = 1.0
    for (rt, rr, rp, rl), (lt, _, _, ll) in zip(right, left):
        t.g["right_gripper"], t.g["left_gripper"] = rt.copy(), lt.copy()
        near_arms = min(near_arms, abs(rt[1] - lt[1]) if np.linalg.norm(rt[:2] - lt[:2]) < 0.15 else 1.0)
        if hand["scoop"]:
            rot = kit._rot_xyz(rr, rp) @ grip_rot.T
            prev = t.scoop.copy()
            t.scoop = rt + rot @ (SCOOP0 - m.TCP0)
            t.vel["scoop"] = (t.scoop - prev) / m.DT
            t.q = qy(math.degrees(math.atan2(rot[0, 2], rot[2, 2])))
        if hand["rod"]:
            aim = m.ROD_AIM if ll.endswith(("down", "bottom")) else (0.0, 0.0)  # the sim's lag at a stroke's bottom
            t.rod = lt + (ROD0 - m.ROD_TCP) - np.array([*aim, 0.0])
        if hand["float"]:
            t.flt = lt + (FLOAT0 - m.FLOAT_TCP)
        t.step()
        if rl == "grip":
            hand["scoop"] = True
        if rl == "let go":
            hand["scoop"] = False
            t.scoop, t.q, t.vel["scoop"] = SCOOP0.copy(), np.array([0, 0, 0, 1.0]), np.zeros(3)
        if ll == "grip rod":
            hand["rod"] = True
        if ll == "let go of rod":
            hand["rod"] = False
            t.rod = ROD0.copy()
        if ll == "grip float":
            hand["float"] = True
        if ll == "let go of float":
            hand["float"] = False
            t.flt = FLOAT0.copy()
    return t, n, near_arms


t, steps, near_arms = replay()
log = dk.stroke_log(t.objs(), "mold")
L = dk.lifts(t.objs(), "mold")
expect("scripted demo replayed (perfect tracking)", t.step(20), 1.0,
       f"{info(t)}, {steps} steps, strokes per lift {[sum(s.lift == x.index for s in log) for x in L]}, "
       f"taps per lift {[sum(lf == x.index for lf, _ in m.TAPS.log) for x in L]}, "
       f"spilled {dk.spilled():.0f} g")
ok = steps <= 6400 - 400
fails += not ok
print(("ok  " if ok else "FAIL") + f" demo ({steps} steps) leaves at least 20 s (400 steps) of the 6,400-step limit")
ok = near_arms > 0.12
fails += not ok
print(("ok  " if ok else "FAIL") + " the two hands never come within 12 cm of each other in y while within 15 cm")

print("all pass" if not fails else f"{fails} FAILED")
