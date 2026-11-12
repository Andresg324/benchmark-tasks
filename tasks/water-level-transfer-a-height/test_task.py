"""Offline checks of the water-level model and the scorer: step-by-step jar, end, marker and gripper histories at
20 Hz, no simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import importlib.util
import math
import os

import numpy as np
from yam_arena.task_file import ObjectState

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("water_level_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit, lh, gk = m.kit, m.lh, m.gk

PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
ON_STICK = 0.266  # finger opening closed on the 2.5 cm stick
ON_HANDLE = 0.266  # on the 2.5 cm handle bar
ON_TAB = 0.255  # on the 2.4 cm tab
STAND = {m.JAR: m.JAR_STAND, m.END: m.END_STAND}
DZ = {m.JAR: m.JAR_DZ, m.END: m.GRIP_DZ}
JAR_SPOT = tuple(m.JAR_SPOT)
END_SPOT = (m.POST_X, m.FAR_Y + m.HOLD_Y)
SHELF = m.SHELF_TOP - m.TOP
TAB_X = m.MARK_FACE + m.TAB[0][0] - 0.012
TAB_Y = m.FAR_Y + m.TAB[0][1]


def st(pos, quat=(0, 0, 0, 1.0), vel=(0, 0, 0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.array(vel, float), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def qx(deg):
    a = math.radians(deg) / 2
    return (math.sin(a), 0.0, 0.0, math.cos(a))


class Trial:
    """A fresh trial: the jar and the end in their stands, the marker at its start, arms at PARK."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.obj = {n: [np.array([*STAND[n], m.TOP]), (0, 0, 0, 1.0)] for n in (m.JAR, m.END)}
        self.marker = m.MARK_START
        self.g = dict(PARK)
        self.open = {g: 1.0 for g in PARK}
        self.last = None
        self.step()

    def objects(self):
        objs = {"stands": st((m.JAR_STAND[0], 0, m.TOP)), "datum_post": st((m.POST_X, m.DATUM_Y, m.TOP)),
                "far_post": st((m.POST_X, m.FAR_Y, m.TOP)),
                m.MARKER: st((m.MARK_FACE, m.FAR_Y, self.marker))}
        for n, (p, q) in self.obj.items():
            objs[n] = st(p, q)
        objs.update({g: st(p, (1.0, 0.0, 0.0, 0.0)) for g, p in self.g.items()})  # pointing down
        return objs

    def step(self, n=1):
        for _ in range(n):
            kit._OPENING.update(self.open)
            objs = self.objects()
            lh.update(objs)
            ok = m.success(objs)
            self.last = (m.score(objs), ok)
        return self.last

    def surface(self, name):
        return lh.surface_at(end=name)

    # --- moves
    def hold(self, name, g):
        p = self.obj[name][0]
        self.g[g] = (float(p[0]), float(p[1]), float(p[2]) + DZ[name])
        self.open[g] = ON_HANDLE if name == m.JAR else ON_STICK

    def put(self, name, xy, z_bottom, g=None, tilt=0.0):
        self.obj[name] = [np.array([xy[0], xy[1], m.TOP + z_bottom]), qx(tilt)]
        if g:
            self.hold(name, g)

    def move(self, moves, secs=1.5):
        """moves: {name: (xy, z_bottom, gripper or None)}; all moved together in a straight line over `secs`."""
        n = max(1, int(round(secs / 0.05)))
        start = {k: self.obj[k][0].copy() for k in moves}
        for i in range(1, n + 1):
            for k, (xy, zb, g) in moves.items():
                goal = np.array([xy[0], xy[1], m.TOP + zb])
                p = start[k] + (goal - start[k]) * i / n
                self.put(k, p[:2], p[2] - m.TOP, g)
            self.step()

    def let_go(self, g, park=True):
        self.open[g] = 1.0
        self.step()
        if park:
            self.g[g] = PARK[g]
            self.step()

    def grip_tab(self, g):
        self.g[g] = (TAB_X, TAB_Y, self.marker - 0.01)
        self.open[g] = ON_TAB
        self.step()

    def slide(self, z, g, secs=2.0):
        n = max(1, int(round(secs / 0.05)))
        z0 = self.marker
        for i in range(1, n + 1):
            self.marker = z0 + (z - z0) * i / n
            self.g[g] = (TAB_X, TAB_Y, self.marker - 0.01)
            self.step()

    def home(self, n=10):
        self.g = dict(PARK)
        self.open = {g: 1.0 for g in PARK}
        return self.step(n)


def end_up(t):
    """The right hand: the end out of its stand onto the far post's shelf, let go."""
    t.hold(m.END, "right_gripper")
    t.move({m.END: (m.END_STAND, m.CARRY, "right_gripper")})
    t.move({m.END: (END_SPOT, m.CARRY, "right_gripper")}, 2.0)
    t.move({m.END: (END_SPOT, SHELF, "right_gripper")})
    t.let_go("right_gripper", park=False)


def jar_up(t, off=0.0, spot=JAR_SPOT):
    """The left hand: the jar out of its stand to beside the datum post, its water `off` m above the line."""
    t.hold(m.JAR, "left_gripper")
    t.move({m.JAR: (m.JAR_STAND, m.JAR_CARRY, "left_gripper")})
    t.move({m.JAR: (spot, m.JAR_CARRY, "left_gripper")}, 2.5)
    t.move({m.JAR: (spot, m.JAR_HOLD + off, "left_gripper")})


def jar_back(t):
    t.move({m.JAR: (m.JAR_STAND, m.JAR_CARRY, "left_gripper")}, 2.5)
    t.move({m.JAR: (m.JAR_STAND, 0.0, "left_gripper")})
    t.let_go("left_gripper")


def end_back(t):
    t.hold(m.END, "right_gripper")
    t.move({m.END: (END_SPOT, m.CARRY, "right_gripper")})
    t.move({m.END: (m.END_STAND, m.CARRY, "right_gripper")}, 2.0)
    t.move({m.END: (m.END_STAND, 0.0, "right_gripper")})
    t.let_go("right_gripper")


def full(off=0.0, to=None, secs=2.0, back=True):
    """The whole job; the marker slid to the end's surface (or to `to`) over `secs`."""
    t = Trial()
    end_up(t)
    jar_up(t, off)
    t.grip_tab("right_gripper")
    t.slide(t.surface(m.END) if to is None else to, "right_gripper", secs)
    t.step(10)
    t.let_go("right_gripper")
    if back:
        jar_back(t)
        end_back(t)
    return t


def check(name, got, want):
    assert got == want, f"{name}: got {got}, want {want}"
    print(f"ok  {name}: {got}")


# --- the model
t = Trial()
s0 = t.surface(m.JAR)
assert abs(s0 - m.FILL) < 1e-6 and abs(t.surface(m.END) - s0) < 1e-9
t.step(200)
assert abs(t.surface(m.JAR) - s0) < 1e-9 and lh.spilled() == 0.0
print(f"ok  at rest in the stands: water at {(s0 - m.TOP) * 100:.1f} cm in both, nothing spills")

t = Trial()
end_up(t)
d_end = t.surface(m.END) - s0
assert lh.spilled() == 0.0 and abs(d_end) < 0.001
print(f"ok  end raised 4 cm onto the shelf: the water moves {d_end * 1000:.2f} mm (the jar holds it)")
jar_up(t)
o = t.objects()
assert abs(t.surface(m.JAR) - m.DATUM_Z) < 1e-4 and abs(t.surface(m.END) - m.DATUM_Z) < 1e-4 and lh.spilled() == 0
print(f"ok  jar held at JAR_HOLD ({m.JAR_HOLD * 100:.2f} cm): its water and the end's on the line")
z0 = t.surface(m.END)
t.move({m.JAR: (JAR_SPOT, m.JAR_HOLD + 0.005, "left_gripper")}, 0.5)
dj = t.surface(m.END) - z0
assert abs(dj - 0.005) < 0.0002
print(f"ok  jar raised 5 mm: the water rises {dj * 1000:.2f} mm (follows the jar ~1:1)")

t = Trial()  # the jar raised to the line with the end still in its stand: the end overflows
jar_up(t)
t.step(40)
assert lh.spilled() > 5.0, lh.spilled()
print(f"ok  jar up with the end still in its stand: the end overflows, {lh.spilled():.1f} mL")

# --- the scorer
t = Trial()
check("do nothing", t.step(40)[0], 0.0)
check("do nothing, arms home", t.home()[0], 0.0)

t = full()
check("full transfer, all back, arms not home (arm position is not scored)", t.last[0], 1.0)
check("full transfer, arms home", t.home()[0], 1.0)

t = full(secs=0.15)
check("fast but correct: marker slid 3.8 cm in 0.15 s", t.home()[0], 1.0)

t = full(back=False)
check("marked to 1.5 mm, the jar and the end not put back (jar still held on the line)", t.home()[0], 0.75)

t = full(to=m.DATUM_Z + m.PAD[2])
check("level set up, but marker set 2.5 cm high (the datum's height above the far post's pad)", t.home()[0], 0.25)

t = full(off=0.01)
check("jar 1 cm above the line, marker slid to the end's water", t.home()[0], 0.0)

t = full(off=0.01, to=m.DATUM_Z)
check("jar 1 cm above the line, marker slid to the datum height", t.home()[0], 0.0)

t = full(off=0.0012)
check("jar 1.2 mm above the line, marker on the water", t.home()[0], 1.0)

t = full(off=0.0012, to=m.DATUM_Z + 0.0022)
check("jar 1.2 mm high, marker 1 mm over the water (2.2 mm off the datum): within 3 mm", t.home()[0], 0.5)


def end_dropped():
    """The end lowered back to its stand while the jar is up (it overflows), raised again, then marked."""
    t = Trial()
    end_up(t)
    jar_up(t)
    t.hold(m.END, "right_gripper")
    t.move({m.END: (END_SPOT, m.CARRY, "right_gripper")})
    t.move({m.END: (m.END_STAND, m.CARRY, "right_gripper")}, 2.0)
    t.move({m.END: (m.END_STAND, 0.0, "right_gripper")})
    t.step(10)
    t.move({m.END: (m.END_STAND, m.CARRY, "right_gripper")})
    t.move({m.END: (END_SPOT, m.CARRY, "right_gripper")}, 2.0)
    t.move({m.END: (END_SPOT, SHELF, "right_gripper")})
    t.let_go("right_gripper", park=False)
    t.move({m.JAR: (JAR_SPOT, m.JAR_HOLD + (m.DATUM_Z - t.surface(m.JAR)) + 0.0, "left_gripper")}, 0.5)
    t.grip_tab("right_gripper")
    t.slide(t.surface(m.END), "right_gripper")
    t.let_go("right_gripper")
    jar_back(t)
    end_back(t)
    return t


t = end_dropped()
check(f"end let down while the jar is up ({lh.spilled():.1f} mL out), then marked", t.home()[0], 0.0)

t = Trial()  # put back in the wrong order: the end into its stand while the jar is still on the line
end_up(t)
jar_up(t)
t.grip_tab("right_gripper")
t.slide(t.surface(m.END), "right_gripper")
t.let_go("right_gripper")
end_back(t)
jar_back(t)
check(f"end put back before the jar ({lh.spilled():.1f} mL out)", t.home()[0], 0.0)


def one_gripper_parked():
    """One gripper: the jar left in its stand, the end onto the shelf, the marker slid to the datum height."""
    t = Trial()
    end_up(t)
    t.grip_tab("right_gripper")
    t.slide(m.DATUM_Z, "right_gripper")
    t.let_go("right_gripper")
    end_back(t)
    return t


check("one gripper: jar parked in its stand, marker slid to the datum height", one_gripper_parked().home()[0], 0.0)

t = Trial()  # one gripper: the jar left in its stand, marker slid to the end's water
end_up(t)
t.grip_tab("right_gripper")
t.slide(t.surface(m.END), "right_gripper")
t.let_go("right_gripper")
end_back(t)
check("one gripper: jar parked, marker slid to the end's water (7 cm low)", t.home()[0], 0.0)


def jar_on_line_unheld():
    """The jar put at the line's height and let go (no shelf there in the scene, but the scorer must still want a
    hand on it), then the same gripper slides the marker."""
    t = Trial()
    end_up(t)
    jar_up(t)
    t.let_go("left_gripper")
    t.grip_tab("right_gripper")
    t.slide(t.surface(m.END), "right_gripper")
    t.let_go("right_gripper")
    jar_back_from = t.obj[m.JAR][0]
    t.hold(m.JAR, "left_gripper")
    assert np.allclose(t.obj[m.JAR][0], jar_back_from)
    jar_back(t)
    end_back(t)
    return t


check("jar left standing at the line without a hand", jar_on_line_unheld().home()[0], 0.0)


def same_hand():
    """The hand holding the jar is also counted on the marker (it cannot reach; the scorer still refuses it)."""
    t = Trial()
    end_up(t)
    jar_up(t)
    t.g["right_gripper"] = PARK["right_gripper"]
    z0 = t.marker
    for i in range(1, 21):
        t.marker = z0 + (t.surface(m.END) - z0) * i / 20
        t.g["left_gripper"] = (TAB_X, TAB_Y, t.marker - 0.01)  # the left on the tab ...
        t.open["left_gripper"] = ON_TAB
        o = t.objects()
        assert not m.reading(o)  # ... is not on the jar's handle
        t.step()
    t.open["left_gripper"] = 1.0
    t.step()
    t.hold(m.JAR, "left_gripper")
    jar_back(t)
    end_back(t)
    return t


check("one hand on the marker, none on the jar", same_hand().home()[0], 0.0)

t = Trial()  # the jar held beside the far post instead (off its spot)
end_up(t)
jar_up(t, spot=(m.POST_X - 0.10, m.JAR_SPOT[1]))
t.grip_tab("right_gripper")
t.slide(t.surface(m.END), "right_gripper")
t.let_go("right_gripper")
jar_back(t)
end_back(t)
check("jar held on the water's height 10 cm in front of the datum post", t.home()[0], 0.0)

t = Trial()  # the end left on the table at the far post, not behind the pointer
end_up(t)
t.hold(m.END, "right_gripper")
t.move({m.END: ((m.POST_X - 0.08, m.FAR_Y + 0.07), SHELF, "right_gripper")})
t.let_go("right_gripper", park=False)
jar_up(t)
t.grip_tab("right_gripper")
t.slide(t.surface(m.END), "right_gripper")
t.let_go("right_gripper")
jar_back(t)
t.hold(m.END, "right_gripper")
t.move({m.END: (m.END_STAND, m.CARRY, "right_gripper")}, 2.0)
t.move({m.END: (m.END_STAND, 0.0, "right_gripper")})
t.let_go("right_gripper")
check("end 8 cm away from the far post while marking", t.home()[0], 0.0)

t = full()
t.home()
t.marker += 0.0025
t.step(3)
check("marker nudged 2.5 mm after everything was put back (setup credit only)", t.home()[0], 0.25)

# a correct transfer, then a quick wrong flick of the marker (undone) with the jar already down
t2 = full()
t2.home()
t2.grip_tab("right_gripper")
z = t2.marker
t2.slide(z + 0.02, "right_gripper", 0.1)
t2.slide(z, "right_gripper", 0.1)
t2.let_go("right_gripper")
check("marker flicked 2 cm and back after the jar was down (setup credit only)", t2.home()[0], 0.25)

t = full()
t.home()
t.g["right_gripper"] = (TAB_X, TAB_Y, t.marker - 0.01)
t.open["right_gripper"] = 1.0
check("open hand left around the marker's tab (marker not let go)", t.step(3)[0], 0.25)
t.open["right_gripper"] = 0.0
check("empty closed fist on the marker's tab", t.step(3)[0], 0.25)

t = full()
t.home()
t.g["left_gripper"] = (m.JAR_STAND[0] + 0.03, m.JAR_STAND[1], m.TOP + 0.05)
t.open["left_gripper"] = 0.0
check("closed fist resting against the jar's side (jar not let go)", t.step(3)[0], 0.75)

t = full()
t.home()
t.g["left_gripper"] = (m.JAR_STAND[0], m.JAR_STAND[1], m.TOP + m.JAR_DZ)
t.open["left_gripper"] = 1.0
check("open hand around the jar's handle at the end", t.step(3)[0], 0.75)

t = Trial()  # a fist on the handle without closing on it is not a grip on the jar
end_up(t)
jar_up(t)
t.open["left_gripper"] = 0.0
kit._OPENING.update(t.open)
o = t.objects()
assert not gk.grip_on(o, m.JAR, m.HANDLE_PART, m.HANDLE_BAND)
t.open["left_gripper"] = ON_HANDLE
print("ok  an empty closed fist at the handle is not a grip")

t = full()
t.obj[m.END][0] = t.obj[m.END][0] + np.array([0.03, 0.0, 0.0])
check("end set down 3 cm off its stand", t.home()[0], 0.75)

t = full()
t.obj[m.JAR][1] = qx(30)
check("jar left leaning 30 degrees in its stand", t.home()[0], 0.75)

# --- partial credit (review, Nov 12)
t = Trial()  # the level set up and held on the line for 2 s, then put back without marking
end_up(t)
jar_up(t)
t.step(40)
jar_back(t)
end_back(t)
check("level set up and held on the line 2 s, never marked, all put back", t.home()[0], 0.25)

t = Trial()  # the same held only 0.5 s: not yet a setup
end_up(t)
t.hold(m.JAR, "left_gripper")
t.move({m.JAR: (m.JAR_STAND, m.JAR_CARRY, "left_gripper")})
t.move({m.JAR: (JAR_SPOT, m.JAR_CARRY, "left_gripper")}, 2.5)
t.move({m.JAR: (JAR_SPOT, m.JAR_HOLD + 0.0035, "left_gripper")}, 0.5)
t.move({m.JAR: (JAR_SPOT, m.JAR_HOLD, "left_gripper")}, 0.25)
t.step(4)
t.move({m.JAR: (JAR_SPOT, m.JAR_HOLD + 0.0035, "left_gripper")}, 0.25)
jar_back(t)
end_back(t)
check("jar on the line only 0.6 s, never marked", t.home()[0], 0.0)

t = Trial()  # the jar swept up past the line and straight back down: passing through is not a setup
end_up(t)
t.hold(m.JAR, "left_gripper")
t.move({m.JAR: (m.JAR_STAND, m.JAR_CARRY, "left_gripper")})
t.move({m.JAR: (JAR_SPOT, m.JAR_CARRY, "left_gripper")}, 2.5)
t.move({m.JAR: (JAR_SPOT, m.JAR_HOLD - 0.01, "left_gripper")}, 0.5)
t.move({m.JAR: (JAR_SPOT, m.JAR_CARRY, "left_gripper")}, 0.5)
jar_back(t)
end_back(t)
check("jar swept through the line height and back, never held there", t.home()[0], 0.0)

t = full(off=0.0025)
check("jar 2.5 mm above the line, marker on the water (mark 2.5 mm off), all put back", t.home()[0], 0.5)
t = full(off=0.0025, back=False)
check("the same, jar and end not put back", t.home()[0], 0.5)
t = full(off=-0.0028)
check("jar 2.8 mm below the line, marker on the water", t.home()[0], 0.5)
t = full(off=0.005)
check("jar 5 mm above the line, marker on the water (mark 5 mm off)", t.home()[0], 0.0)
t = full(off=0.005, to=m.DATUM_Z)
check("jar 5 mm above the line, marker slid to the datum height by eye (not on the water)", t.home()[0], 0.0)
t = full(off=0.0025, to=m.DATUM_Z)
check("jar 2.5 mm high, marker 2.5 mm under the water on the datum height (lucky, not a 1.5 mm read)", t.home()[0],
      0.5)
t = full(off=0.0012, back=False)
t.home()
check("1.5 mm mark, then the marker still pinched at the end", (t.grip_tab("right_gripper") or t.step(3))[0], 0.25)

# the text's numbers match the code
assert m.WIDE == 0.003 and m.SET_S == 1.0 and m.SET_UP.steps == 20
assert m.FULL == 0.0015 and m.ON_LINE == 0.0015 and m.ON_SURFACE == 0.0015 and m.SPILL_MAX == 1.0
assert m.MOVED == 0.0005 and m.AT_JAR == 0.03 and m.AT_END == 0.025 and m.IN_STAND == 0.025
assert abs(m.DATUM_Z - m.TOP - 0.130) < 1e-9 and abs(m.SHELF_TOP - m.TOP - 0.04) < 1e-9 and m.PAD[2] == 0.025
assert abs(math.pi * m.BORE_R ** 2 * 1e6 * 0.01 - 1.267) < 0.01  # 1.27 mL per cm of end tube
assert abs((m.JAR_IN / m.BORE_R) ** 2 - 54.8) < 0.1  # the jar's cross-section 55 times the end tube's
print("all pass")
