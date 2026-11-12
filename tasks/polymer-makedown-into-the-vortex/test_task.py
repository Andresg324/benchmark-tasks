"""Offline checks of the powder model and scorer: step-by-step rod, shaker and gripper histories at 20 Hz, no
simulator.

    python test_task.py        (with the yam_arena package importable)
"""

import importlib.util
import math
import os
from types import SimpleNamespace

import numpy as np
from yam_arena.task_file import ObjectState

HERE = os.path.dirname(os.path.abspath(__file__))
_spec = importlib.util.spec_from_file_location("polymer_task", os.path.join(HERE, "task.py"))
m = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(m)
kit, dk, gk = m.kit, m.dk, m.gk

PARK = {"left_gripper": (0.20, 0.27, m.TOP + 0.20), "right_gripper": (0.20, -0.27, m.TOP + 0.20)}
CAP_OVER = np.array(m.CAP_AT)  # the cap over the water, as in the demo
GRIPPED = 0.15  # finger opening on the rod's sleeve or the shaker's handle
STIR = 1 / m.REV_S  # the demo's turns per second
TILT = m.HELD_DEG  # the demo's sprinkling tilt, as held in the sim
TIP_R = 0.008  # the rod tip's circle in the sim (the 2 cm command, shrunk by the arm's lag)


def st(pos, quat=(0, 0, 0, 1.0)):
    p = np.array(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.array(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                       low=p - 0.01, high=p + 0.01)


def qy(deg):
    """Turned `deg` about world y with the top going toward -x (negative rotation about y)."""
    a = -math.radians(deg) / 2
    return (0.0, math.sin(a), 0.0, math.cos(a))


def left_in_shaker():
    return dk._state().left["shaker"]


class Trial:
    """A fresh trial: rod in its rest, shaker on the bench, arms at PARK. step() runs one control step as the hook
    does: the kit's update, then success() and score()."""

    def __init__(self):
        kit._EPISODE["count"] += 1
        self.rod = np.array(m.ROD_START, float)
        self.rod_q = (0, 0, 0, 1.0)
        self.shaker = np.array([*m.SHAKER_AT, m.TOP])
        self.shaker_q = (0, 0, 0, 1.0)
        self.g = dict(PARK)
        self.open = {g: 1.0 for g in PARK}
        self.last = None
        self.a = 0.0
        self.step()

    def objects(self):
        objs = {"beaker": st((*m.BEAKER, m.TOP)), "rod_rest": st((*m.ROD_BLOCK, m.TOP)),
                "rod": st(self.rod, self.rod_q), "shaker": st(self.shaker, self.shaker_q)}
        objs.update({g: st(p, (1.0, 0.0, 0.0, 0.0)) for g, p in self.g.items()})
        return objs

    def step(self, n=1):
        for _ in range(n):
            kit._OPENING.update(self.open)
            objs = self.objects()
            dk.update(objs)
            ok = m.success(objs)
            self.last = (m.score(objs), ok)
        return self.last

    # -- the rod (left hand) --
    def hold_rod(self, hand="left_gripper"):
        self.g[hand] = tuple(self.rod)
        self.open[hand] = GRIPPED

    def rod_tip_at(self, p, hand="left_gripper"):
        self.rod = np.asarray(p, float) + [0, 0, m.ROD_LEN / 2]
        if hand:
            self.hold_rod(hand)

    def stir_pos(self, a, r=TIP_R):
        return (m.STIR[0] + r * math.cos(a), m.STIR[1] + r * math.sin(a), m.TOP + m.STIR_Z)

    # -- the shaker (right hand) --
    def hold_shaker(self, hand="right_gripper"):
        R = dk._rotm(self.shaker_q)
        self.g[hand] = tuple(self.shaker + R @ m.GRIP)
        self.open[hand] = GRIPPED

    def shaker_at(self, tilt, cap=CAP_OVER, hand="right_gripper"):
        self.shaker_q = qy(tilt)
        R = dk._rotm(self.shaker_q)
        self.shaker = np.asarray(cap, float) - R @ np.asarray(m.CAP, float)
        if hand:
            self.hold_shaker(hand)

    def run(self, seconds, stir=None, tilt=None, rod_hand="left_gripper", shaker_hand="right_gripper",
            stir_r=TIP_R, cap=CAP_OVER):
        """`stir`: turns per second of the rod's tip round its circle (None: rod untouched). `tilt`: the shaker held
        at that tilt with the cap at `cap` (None: shaker untouched)."""
        for _ in range(int(round(seconds / 0.05))):
            if stir is not None:
                self.a += 2 * math.pi * stir * 0.05
                self.rod_tip_at(self.stir_pos(self.a, stir_r), rod_hand)
            if tilt is not None:
                self.shaker_at(tilt, cap=cap, hand=shaker_hand)
            self.step()
        return self.last

    def put_down(self, home=True, rod="rest"):
        """Shaker stood back on its spot, rod stood in its rest (or left in the beaker), hands away (home)."""
        self.shaker_q = (0, 0, 0, 1.0)
        self.shaker = np.array([*m.SHAKER_AT, m.TOP])
        if rod == "rest":
            self.rod = np.array(m.ROD_START, float) + [0, 0, 0.004]
        else:
            self.rod_tip_at((m.STIR[0], m.STIR[1], m.TOP + 0.006), hand=None)
        for g in PARK:
            self.open[g] = 1.0
            self.g[g] = PARK[g] if home else (0.25, 0.15 if g == "left_gripper" else -0.15, m.TOP + 0.15)
        return self.step(30)


def correct(trial=None, stir=STIR, tilt=TILT, stir_r=TIP_R):
    """The demo's pattern: stir 3 s, trickle the shaker empty while stirring, stir 3 s more."""
    t = trial or Trial()
    t.run(3.0, stir=stir, stir_r=stir_r)
    t.run(m.SPRINKLE_S, stir=stir, tilt=tilt, stir_r=stir_r)
    t.run(3.0, stir=stir, stir_r=stir_r)
    return t


def totals():
    return dk.added(), dk.lumped(), dk.surface(), dk.spilled()


# -- the model ---------------------------------------------------------------------------------------------------------

def test_dose_conserved():
    correct()
    total = left_in_shaker() + dk.dispersed() + dk.lumped() + dk.surface() + dk.spilled()
    assert abs(total - m.DOSE) < 1e-9, total


def test_no_flow_until_past_95_degrees():
    t = Trial()
    t.run(3.0, stir=STIR, tilt=94.0)
    assert left_in_shaker() == m.DOSE


def test_rate_grows_with_tilt():
    got = []
    for tilt in (100.0, 105.0, 120.0):
        Trial().run(1.0, stir=STIR, tilt=tilt)
        got.append(m.DOSE - left_in_shaker())
    assert 0 < got[0] < got[1] < got[2], got
    assert 0.05 < got[1] < 0.09, got  # about 0.07 g/s at 105 degrees, as the task text says


def test_powder_on_still_water_lumps_after_one_second():
    t = Trial()
    t.run(0.3, tilt=110.0)
    g = m.DOSE - left_in_shaker()
    assert g > 0 and dk.lumped() == 0
    t.shaker_at(0.0, cap=(*m.SHAKER_AT, m.TOP + m.CAP[2]))
    t.run(0.6)
    assert dk.lumped() == 0  # under a second: not yet
    t.run(0.6)
    assert abs(dk.lumped() - g) < 1e-9


def test_lumps_irreversible():
    """Fish-eyes formed during a 1.5 s stop stay when stirring resumes."""
    t = Trial()
    t.run(2.0, stir=STIR)
    t.run(2.0)
    t.run(0.5, tilt=110.0)
    t.shaker_at(0.0, cap=(*m.SHAKER_AT, m.TOP + m.CAP[2]))
    t.run(1.5)
    lumped = dk.lumped()
    assert lumped > 0
    t.run(5.0, stir=STIR)
    assert dk.lumped() == lumped


def test_short_pause_forgiven():
    """A 0.4 s hitch in the stirring while trickling: no fish-eyes."""
    t = Trial()
    t.run(3.0, stir=STIR)
    t.run(3.0, stir=STIR, tilt=TILT)
    t.run(0.4, tilt=TILT)
    t.run(m.SPRINKLE_S, stir=STIR, tilt=TILT)
    t.run(3.0, stir=STIR)
    assert dk.lumped() / m.DOSE < 0.01, dk.lumped()
    assert t.put_down()[0] == 1.0


def test_rod_above_the_water_does_not_stir():
    t = Trial()
    t.rod_tip_at((m.STIR[0], m.STIR[1], m.SURFACE + 0.04))
    t.step(20)
    top = 0.0
    for k in range(60):
        t.rod_tip_at((m.STIR[0] + 0.01 * math.cos(k), m.STIR[1] + 0.01 * math.sin(k), m.SURFACE + 0.005))
        t.step()
        top = max(top, dk.stir_speed())
    assert top == 0, top


def test_slow_stir_does_not_disperse():
    """A rod drifting round at a third of a turn per second (2 cm/s) is no swirl: powder lumps."""
    t = Trial()
    t.run(2.0, stir=0.33)
    t.run(m.SPRINKLE_S, stir=0.33, tilt=TILT)
    t.run(2.0, stir=0.33)
    assert dk.lumped() > 0.9 * m.DOSE
    assert t.put_down()[0] == 0.0


# -- the score ---------------------------------------------------------------------------------------------------------

def test_do_nothing_zero():
    t = Trial()
    assert t.step(60)[0] == 0.0
    assert t.put_down()[0] == 0.0  # tools back where they were, nothing added


def test_demo_pattern_full():
    t = correct()
    s = t.put_down()
    assert s == (1.0, True), (s, totals())
    assert dk.lumped() / m.DOSE < 0.01


def test_not_home_09():
    t = correct()
    assert t.put_down(home=False)[0] == 0.9


def test_rod_left_in_the_beaker_zero():
    t = correct()
    assert t.put_down(rod="beaker")[0] == 0.0


def test_fast_stir_full():
    """Stirring at 4 turns a second round a 2 cm circle (50 cm/s at the tip): interpolated, just as good."""
    t = correct(stir=4.0, stir_r=0.02)
    assert t.put_down()[0] == 1.0


def test_tools_still_held_zero():
    """A closed hand on any part of either tool at the end (the rod's tip end, the shaker's cap) is still holding."""
    for where in ("rod_sleeve", "rod_end", "shaker_handle", "shaker_cap"):
        t = correct()
        t.put_down()
        if where.startswith("rod"):
            p = np.array(t.rod) + ([0, 0, 0] if where == "rod_sleeve" else [0, 0, -0.06])
            t.g["left_gripper"], t.open["left_gripper"] = tuple(p), GRIPPED
        else:
            p = t.shaker + (m.GRIP if where == "shaker_handle" else [0, 0, m.CAP[2] - 0.004])
            t.g["right_gripper"], t.open["right_gripper"] = tuple(p), GRIPPED
        assert t.step(5)[0] == 0.0, where


def test_open_hand_round_the_tool_zero():
    """An open hand left around the rod's sleeve, or an empty fist beside it, has not let go (1 cm clearance)."""
    for opening in (1.0, 0.0):
        t = correct()
        t.put_down()
        t.g["left_gripper"] = tuple(np.array(t.rod) + [0.006, 0, 0])
        t.open["left_gripper"] = opening
        assert t.step(5)[0] == 0.0, opening


def test_shaker_left_tipped_zero():
    t = correct()
    t.put_down()
    t.shaker_at(30.0, cap=(m.SHAKER_AT[0], m.SHAKER_AT[1], m.TOP + 0.08), hand=None)
    assert t.step(5)[0] == 0.0


def test_powder_before_stirring_zero():
    t = Trial()
    t.run(m.SPRINKLE_S, tilt=TILT)
    t.run(5.0, stir=STIR)
    assert dk.lumped() > 0.9 * m.DOSE
    assert t.put_down()[0] == 0.0


def test_stir_pause_mid_sprinkle_zero():
    """The rod stops for 1.5 s while the powder keeps trickling in."""
    t = Trial()
    t.run(3.0, stir=STIR)
    t.run(3.0, stir=STIR, tilt=TILT)
    t.run(1.5, tilt=TILT)
    t.run(m.SPRINKLE_S, stir=STIR, tilt=TILT)
    t.run(3.0, stir=STIR)
    assert dk.lumped() / m.DOSE > m.LUMP_MAX
    assert t.put_down()[0] == 0.0


def test_turned_over_zero():
    """The shaker turned cap-down in one step into the demo's swirl, and into a 50 cm/s one: the dose lands in a
    tenth of a second and heaps."""
    for stir, r in ((STIR, TIP_R), (4.0, 0.02)):
        t = Trial()
        t.run(3.0, stir=stir, stir_r=r)
        t.run(1.0, stir=stir, tilt=180.0, stir_r=r)
        t.run(4.0, stir=stir, stir_r=r)
        assert left_in_shaker() == 0
        assert dk.lumped() / m.DOSE > 0.1, (stir, dk.lumped())
        assert t.put_down()[0] == 0.0


def test_quick_flick_latches():
    """Turned over for two steps and straight back: the fish-eyes from that flick stay, even though the rest is then
    trickled in right."""
    t = Trial()
    t.run(3.0, stir=STIR)
    t.run(0.1, stir=STIR, tilt=170.0)
    t.run(m.SPRINKLE_S, stir=STIR, tilt=TILT)
    t.run(3.0, stir=STIR)
    assert dk.lumped() / m.DOSE > m.LUMP_MAX
    assert t.put_down()[0] == 0.0


def test_shaken_too_fast_zero():
    """Held at 135 degrees (1.1 g/s, the dose in under half a second) into the demo's stir: faster than the swirl
    spreads it."""
    t = Trial()
    t.run(3.0, stir=STIR)
    t.run(3.0, stir=STIR, tilt=135.0)
    t.run(3.0, stir=STIR)
    assert dk.lumped() / m.DOSE > m.LUMP_MAX
    assert t.put_down()[0] == 0.0


def test_one_arm_alternating_zero():
    """One arm: trickle for 1 s, 2.5 s to set the shaker down and take up the rod, stir 3 s, 2.5 s back to the
    shaker, again and again. Each second's powder lands on still water and lumps."""
    t = Trial()
    for _ in range(30):
        t.g["left_gripper"] = PARK["left_gripper"]
        t.run(1.0, tilt=TILT)
        t.shaker_q, t.shaker = (0, 0, 0, 1.0), np.array([*m.SHAKER_AT, m.TOP])  # stood down to free the hand
        t.run(2.5)
        t.run(3.0, stir=STIR, rod_hand="right_gripper")
        t.run(2.5)
        if left_in_shaker() <= 0:
            break
    assert left_in_shaker() == 0
    assert dk.lumped() > 0.9 * m.DOSE
    assert t.put_down()[0] == 0.0


def test_one_gripper_only_zero():
    """One gripper doing one job: stirring only adds nothing; trickling only lumps everything; stirring with the
    shaker itself (the cap dunked in the water) stirs nothing and the powder that comes out under water is lost."""
    t = Trial()
    t.run(20.0, stir=STIR)
    assert t.put_down()[0] == 0.0
    t = Trial()
    t.run(m.SPRINKLE_S, tilt=TILT)
    assert t.put_down()[0] == 0.0
    t = Trial()
    for k in range(200):
        a = k * 0.5
        t.shaker_at(150.0, cap=(m.BEAKER[0] + 0.012 * math.cos(a), m.BEAKER[1] + 0.012 * math.sin(a),
                                m.SURFACE - 0.01))
        t.step()
    assert dk.stir_speed() == 0 and dk.spilled() > 0.4
    assert t.put_down()[0] == 0.0


def test_spilled_beside_the_beaker_zero():
    """Trickled with the cap 6 cm past the beaker's axis, or swung in already tipped (a short spill on the way)."""
    t = Trial()
    t.run(2.0, stir=STIR)
    t.run(m.SPRINKLE_S, stir=STIR, tilt=TILT, cap=CAP_OVER + [0.04, 0.0, 0.0])
    assert dk.spilled() > 0.4
    assert t.put_down()[0] == 0.0
    t = Trial()
    t.run(3.0, stir=STIR)
    t.run(0.5, stir=STIR, tilt=115.0, cap=CAP_OVER + [0.05, 0.0, 0.0])  # 0.13 g on the bench on the way in
    t.run(m.SPRINKLE_S, stir=STIR, tilt=TILT)
    t.run(3.0, stir=STIR)
    assert m.SPILL_MAX < dk.spilled() and dk.lumped() / m.DOSE <= m.LUMP_MAX
    assert t.put_down()[0] == 0.0


def test_one_arm_parked_dredger_zero():
    """One arm sets the dredger down tipped just past 95 degrees with its cap over the water, lets go, then takes the
    rod and stirs while it doses itself: 0 (powder came out of a dredger nobody held), at any parking tilt."""
    for tilt, park_s in ((98.0, 2.0), (100.0, 1.0), (102.0, 0.5), (105.0, 0.5)):
        t = Trial()
        t.g["right_gripper"] = PARK["right_gripper"]
        for _ in range(int(park_s / 0.05)):
            t.shaker_at(tilt, hand="left_gripper")
            t.step()
        t.shaker_at(tilt, hand=None)
        t.g["left_gripper"], t.open["left_gripper"] = PARK["left_gripper"], 1.0
        t.run(80.0, stir=STIR, rod_hand="left_gripper")
        t.shaker_q, t.shaker = (0, 0, 0, 1.0), np.array([*m.SHAKER_AT, m.TOP])
        assert dk.added() > 0.99 * m.DOSE and dk.spilled() == 0  # it all went in ...
        assert t.put_down()[0] == 0.0, tilt  # ... but not with two hands


def test_rod_stirred_without_a_hand_zero():
    """The rod swirls round while the dredger trickles but no other hand holds it (pushed round by the dredger, or
    rocking on its own): the stirring is not credited."""
    t = Trial()
    t.run(3.0, stir=STIR)
    t.g["left_gripper"], t.open["left_gripper"] = PARK["left_gripper"], 1.0
    t.run(m.SPRINKLE_S, stir=STIR, tilt=TILT, rod_hand=None)
    t.run(3.0, stir=STIR)
    assert t.put_down()[0] == 0.0


def test_parked_with_a_hand_hovering_zero():
    """The dredger parked tipped while a half-open hand (0.5) hovers 2 cm over its handle, or a closed fist sits 2 cm
    off it, and the other hand stirs: neither hand grips the handle, so 0."""
    for opening, off in ((0.5, 0.02), (0.15, 0.025), (0.0, 0.005)):
        t = Trial()
        t.run(3.0, stir=STIR)
        t.shaker_at(TILT, hand=None)
        h = t.shaker + dk._rotm(t.shaker_q) @ np.asarray(m.HANDLE)
        t.g["right_gripper"], t.open["right_gripper"] = tuple(h + [0, 0, off]), opening
        t.run(m.SPRINKLE_S, stir=STIR)
        t.shaker_q, t.shaker = (0, 0, 0, 1.0), np.array([*m.SHAKER_AT, m.TOP])
        t.run(3.0, stir=STIR)
        assert m.UNMANNED.on and t.put_down()[0] == 0.0, (opening, off)


def test_grip_flickers_tolerated_but_not_a_letting_go():
    """The demo's pattern with two separate one-step and one two-step slips of the dredger hand (a grip flicker in the
    physics): still 1.0; the hand off the handle for 3 steps in a row while powder flows: 0."""
    for gaps, want in (((40, 80, 120, 121), 1.0), ((60, 61, 62), 0.0)):
        t = Trial()
        t.run(3.0, stir=STIR)
        for k in range(int(m.SPRINKLE_S / 0.05)):
            t.a += 2 * math.pi * STIR * 0.05
            t.rod_tip_at(t.stir_pos(t.a))
            t.shaker_at(TILT)
            if k in gaps:
                t.open["right_gripper"] = 0.5
            t.step()
        t.run(3.0, stir=STIR)
        assert t.put_down()[0] == want, (gaps, want)


def test_trickle_on_still_water_lumps_after_one_second():
    """A steady trickle onto one spot of still water: the patch gels 1 s after the first grains land (fresh powder
    joins the gelling patch, it does not make it younger)."""
    t = Trial()
    t.run(0.5, tilt=110.0)
    assert dk.lumped() == 0
    t.run(0.6, tilt=110.0)
    assert dk.lumped() > 0


def test_new_trial_resets():
    correct()
    Trial()
    assert left_in_shaker() == m.DOSE and dk.lumped() == 0 and dk.dispersed() == 0 and dk.spilled() == 0


def test_text_matches_code():
    doc = m.score.__doc__.replace("\n    ", " ")
    assert f"{100 * m.ADDED_MIN:.0f} % of the {m.DOSE} g dose" in doc
    assert f"{100 * m.LUMP_MAX:.0f} % of the dose ({m.LUMP_MAX * m.DOSE:.3f} g)" in doc
    assert f"at most {m.SPILL_MAX} g" in doc
    assert f"within {m.UPRIGHT_DEG:.0f} degrees" in doc and f"within {1000 * m.DOWN:.0f} mm" in doc
    assert f"{100 * m.CLEAR:.0f} cm clear" in doc
    head = m.__doc__.replace("\n", " ")
    assert f"{m.SHAKE_FROM:.0f} degrees" in head and f"{m.SHAKE_RATE:.0f} g/s" in head
    rate = m.SHAKE_RATE * ((m.HELD_DEG - m.SHAKE_FROM) / m.SHAKE_RAMP) ** 2
    assert f"about {rate:.2f} g/s at {m.HELD_DEG:.0f} degrees" in head, rate
    assert m.SHAKE_FROM + m.SHAKE_RAMP == 180.0  # full rate cap straight down


class _Pose:
    def __init__(self, a):
        self.a = a

    def detach(self):
        return self

    def cpu(self):
        return self

    def numpy(self):
        return self.a


class _FakeMarkers:
    def __init__(self):
        self.calls = []

    def visualize(self, **kw):
        self.calls.append(kw)


def test_drawn_at_reset():
    """Right after a reset, before the first frame: the water column, no powder or lumps (from an old lumpy trial)."""
    t = Trial()
    t.run(3.0, tilt=110.0)
    assert dk.lumped() > 0
    fake = _FakeMarkers()
    dk._VIS.update(markers=fake, failed=False)
    try:
        pose = np.array([*m.BEAKER, m.TOP, 0.0, 0.0, 0.0, 1.0])
        base = SimpleNamespace(scene=SimpleNamespace(env_origins=[_Pose(np.zeros(3))]),
                               arena_world=SimpleNamespace(get_pose_w=lambda n: [_Pose(pose)] if n == "beaker"
                                                           else 1 / 0))
        dk._draw_start(SimpleNamespace(unwrapped=base))
        ids, sc = fake.calls[0]["marker_indices"], fake.calls[0]["scales"]
        shown = sc[:, 2] > 1e-4
        assert shown[ids == 0].sum() == 1 and shown[ids == 1].sum() == 0 and shown[ids == 2].sum() == 0
    finally:
        dk._VIS.update(markers=None, failed=True)


if __name__ == "__main__":
    tests = [(n, f) for n, f in globals().items() if n.startswith("test_") and callable(f)]
    for n, f in tests:
        f()
        print("ok", n)
    print("all pass")
