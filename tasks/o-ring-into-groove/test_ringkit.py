"""Offline tests for ringkit (no simulator): /data/IsaacLab-Arena/.venv/bin/python test_ringkit.py -> "all pass"."""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402
import ringkit as rk  # noqa: E402
import yamkit as kit  # noqa: E402
from yam_arena.task_file import ObjectState  # noqa: E402

TOP = 0.75
FIX = (0.36, -0.10, TOP)  # fixture base origin; the shaft's end face 10.5 cm up
RING = rk.Ring("clip", end=(0.0, 0.0, 0.105), axis="-Z", shaft_d=0.040, free_id=0.0365, thickness=0.00175,
               width=0.0045, length=0.06, grooves=(rk.Groove(at=0.0381, width=0.00185, diameter=0.0375),),
               stop=0.03903, limit_id=0.041, lug_gap=0.012, lug_out=0.0058)
TIPS = rk.TipPair(("pliers", (0.0, 0.0, 0.0)), ("pliers/jaw", (0.0, 0.0, 0.0)))
KEY = rk.attach("fixture", RING, TIPS)
END_Z = FIX[2] + 0.105


def st(pos, quat=(0.0, 0.0, 0.0, 1.0)):
    p = np.asarray(pos, float)
    return ObjectState(pos=p, quat_xyzw=np.asarray(quat, float), lin_vel=np.zeros(3), ang_vel=np.zeros(3), low=p,
                       high=p)


def scene(tip_s=None, gap=0.012, theta=0.0, radial=None, dz=0.0, quat=(0.0, 0.0, 0.0, 1.0)):
    """Objects with the pliers' two tip ends `gap` apart, centered on the ring's lug side (angle pi + theta about the
    shaft), at s = tip_s (None: far away). `radial` is the tips' distance from the axis (default: on the lug holes)."""
    o = {"fixture": st(FIX)}
    if tip_s is None:
        o["pliers"], o["pliers/jaw"] = st((0.2, -0.3, TOP + 0.03)), st((0.2, -0.3, TOP + 0.03))
        return o
    s = rk.state(KEY)
    rad = (radial if radial is not None else s.r + RING.lug_out)
    ang = math.pi + theta
    c = np.array([FIX[0] + rad * math.cos(ang), FIX[1] + rad * math.sin(ang), END_Z - tip_s])
    t = np.array([-math.sin(ang), math.cos(ang), 0.0])
    o["pliers"] = st(c - t * gap / 2 + np.array([0, 0, -dz / 2]), quat)
    o["pliers/jaw"] = st(c + t * gap / 2 + np.array([0, 0, dz / 2]), quat)
    return o


def run(n, **kw):
    for _ in range(n):
        o = scene(**kw)
        rk.update(o)
    return o


def fresh():
    kit._EPISODE["count"] += 1


def gap_for(inner_d):
    return RING.lug_gap + math.pi * (inner_d - RING.free_id)


def near_face():
    return rk.state(KEY).s - RING.thickness / 2


def install(spread_d=0.0406, to_s=None, release=True):
    """Tips into the lugs, spread to `spread_d`, push down to `to_s` (default: the stop), let the ring close."""
    fresh()
    o = run(3)
    assert not rk.engaged(o, KEY)
    s_in = near_face() + 0.0015
    o = run(2, tip_s=s_in, gap=gap_for(RING.free_id))
    assert rk.engaged(o, KEY), "tips in the lug holes engage"
    o = run(5, tip_s=s_in, gap=gap_for(spread_d))
    target = (RING.stop if to_s is None else to_s + RING.thickness / 2)
    for k in range(1, 41):
        o = run(1, tip_s=s_in + (target - s_in) * k / 40, gap=gap_for(spread_d))
    if release:
        o = run(5, tip_s=target, gap=gap_for(RING.free_id))
    return o


def test_start():
    fresh()
    o = run(3)
    assert abs(rk.position(o, KEY) + RING.thickness / 2) < 1e-9
    assert abs(rk.inner_d(o, KEY) - RING.free_id) < 1e-9
    assert not rk.seated(o, KEY) and rk.groove(o, KEY) is None and not rk.engaged(o, KEY)
    assert not rk.overspread(o, KEY) and rk.turned_deg(o, KEY) == 0


def test_unspread_push_stays():
    fresh()
    s_in = near_face() + 0.0015
    run(2, tip_s=s_in, gap=gap_for(RING.free_id))
    for k in range(20):  # pushing down without spreading: the ring grips (here: rests on the end), tips slip out
        o = run(1, tip_s=s_in + 0.002 * k, gap=gap_for(0.0385))
    assert abs(rk.position(o, KEY) + RING.thickness / 2) < 1e-6
    assert not rk.engaged(o, KEY)


def test_full_install_and_turn():
    o = install()
    assert rk.groove(o, KEY) == 0
    assert abs(rk.inner_d(o, KEY) - 0.0375) < 1e-9, rk.inner_d(o, KEY)
    assert rk.engaged(o, KEY) and not rk.seated(o, KEY), "still held by the tips"
    assert abs(rk.max_d(o, KEY) - 0.0406) < 1e-6 and not rk.overspread(o, KEY)
    for k in range(1, 21):  # turn the pliers 25 degrees about the shaft: the seated ring turns in its groove
        o = run(1, tip_s=RING.stop, gap=gap_for(RING.free_id), theta=math.radians(1.25 * k))
    assert rk.turned_deg(o, KEY) > 24, rk.turned_deg(o, KEY)
    o = run(5)  # lift the pliers away
    assert rk.seated(o, KEY) and not rk.engaged(o, KEY) and not rk.cocked(o, KEY)
    assert rk.groove(o, KEY) == 0


def test_lift_out_of_groove_disengages():
    o = install()
    for k in range(10):  # pull up: the groove holds the closed ring; the tips leave the holes
        o = run(1, tip_s=RING.stop - 0.0008 * k, gap=gap_for(RING.free_id))
    assert rk.groove(o, KEY) == 0 and not rk.engaged(o, KEY) and rk.seated(o, KEY)


def test_overspread():
    o = install(spread_d=0.0418)
    assert rk.overspread(o, KEY) and rk.set_mm(o, KEY) > 0.3
    o = install(spread_d=0.0435)  # a large set: the ring's free bore is now bigger than the groove bottom
    o = run(3)
    assert rk.overspread(o, KEY) and not rk.seated(o, KEY), "a loose ring does not close onto the groove bottom"


def test_short_of_groove():
    o = install(to_s=0.030)
    o = run(5)
    assert abs(rk.position(o, KEY) - 0.030) < 3e-4, rk.position(o, KEY)
    assert rk.groove(o, KEY) is None and not rk.seated(o, KEY)
    assert abs(rk.inner_d(o, KEY) - 0.040) < 1e-9, "grips the land"


def test_snap_when_over_groove():
    o = install(to_s=0.0381 - 0.0008)  # mid-plane within half the groove width: its spring pushes it in
    o = run(5)
    assert rk.seated(o, KEY)


def test_cocked():
    tilt = (math.sin(math.radians(3)), 0.0, 0.0, math.cos(math.radians(3)))  # 6 degrees about x
    fresh()
    run(2)
    s_in = near_face() + 0.0015
    run(2, tip_s=s_in, gap=gap_for(RING.free_id), quat=tilt)
    for k in range(1, 41):
        o = run(1, tip_s=s_in + (RING.stop - s_in) * k / 40, gap=gap_for(0.0406), quat=tilt)
    assert rk.max_tilt_deg(o, KEY) > 5.9
    o = run(3, tip_s=RING.stop, gap=gap_for(RING.free_id), quat=tilt)
    o = run(3)
    assert rk.cocked(o, KEY) and not rk.seated(o, KEY)
    th = rk.state(KEY).theta
    # spread it again square, push, release: it seats
    run(2, tip_s=RING.stop - 0.0005, gap=gap_for(RING.free_id))
    o = run(4, tip_s=RING.stop - 0.0005, gap=gap_for(0.0406))
    assert not rk.cocked(o, KEY)
    o = run(3, tip_s=RING.stop, gap=gap_for(RING.free_id))
    o = run(3)
    assert rk.seated(o, KEY), rk.state(KEY)
    assert abs(rk.state(KEY).theta - th) < 1e-6


def test_twist():
    fresh()
    run(2)
    s_in = near_face() + 0.0015
    o = run(3, tip_s=s_in, gap=gap_for(0.0370), dz=0.0012)
    assert rk.engaged(o, KEY)
    assert rk.max_twist_deg(o, KEY) > 4, rk.max_twist_deg(o, KEY)


def test_off_axis_disengages():
    fresh()
    run(2)
    s_in = near_face() + 0.0015
    o = run(2, tip_s=s_in, gap=gap_for(RING.free_id), radial=rk.state(KEY).r + RING.lug_out + 0.006)
    assert not rk.engaged(o, KEY)
    o = run(2, tip_s=near_face() - 0.002, gap=gap_for(RING.free_id))
    assert not rk.engaged(o, KEY), "tips above the ring are not in it"


def test_episode_reset():
    install()
    fresh()
    o = run(1)
    assert not rk.seated(o, KEY) and rk.max_d(o, KEY) == RING.free_id and rk.turned_deg(o, KEY) == 0


def test_links_and_usd():
    lks = rk.links(RING)
    assert [lk.name for lk in lks] == ["clip_axial", "clip_turn", "clip", "clip_shown"]
    assert lks[3].meshes and lks[3].lower <= -RING.hide
    assert lks[0].axis == "Z" and lks[0].lower < -RING.stop + RING.thickness and lks[0].upper > RING.thickness / 2
    fixture = kit.Mechanism("fixture", pos=FIX, size=(0.1, 0.1, 0.1), links=lks)
    text = kit._mech_usda(fixture)
    assert text.count("physics:diagonalInertia") == 4
    jaw = kit.Link("jaw", "prismatic", "Y", (0, 0, 0), 0.0, 0.018, law=rk.HandleSpring(-0.01, 200.0),
                   colliders=(kit.Collider((0, 0, 0), size=(0.01, 0.01, 0.01)),))
    tool = rk.free_tool(kit.Mechanism("tool", pos=FIX, size=(0.1, 0.1, 0.1), mass=0.07, links=(jaw,),
                                      colliders=(kit.Collider((0, 0, 0), size=(0.01, 0.01, 0.01)),)),
                        com=(0.0, 0.0, 0.001))
    t2 = kit._mech_usda(tool)
    root = t2.index('def Xform "Root"\n{')
    base = t2.index('def Xform "base"')
    assert "PhysicsArticulationRootAPI" not in t2[root:base]
    assert "PhysicsArticulationRootAPI" in t2[base:base + 300]
    assert "physxArticulation:sleepThreshold" in t2[base:base + 900]
    assert "point3f physics:centerOfMass = (0, 0, 0.001)" in t2[base:base + 900]
    assert "fixed" not in t2.lower() or "PhysicsFixedJoint" not in t2
    jv = rk._joint_values(RING, rk.state(KEY))
    assert set(jv) == {"clip_axial", "clip_turn", "clip", "clip_shown"}


def test_pieces():
    o = install()
    ps = rk.pieces(KEY, o)
    assert len(ps) == rk.SEGMENTS + 2
    c = np.array([FIX[0], FIX[1], END_Z - rk.position(o, KEY)])
    rad = [np.linalg.norm((p - c)[:2]) for p, R, s in ps[:rk.SEGMENTS]]
    rm = rk.inner_d(o, KEY) / 2 + RING.width / 2
    assert max(abs(r - rm) for r in rad) < 1e-6
    for p, R, s in ps:
        assert abs(np.linalg.det(R) - 1) < 1e-9


if __name__ == "__main__":
    tests = [v for k, v in sorted(globals().items()) if k.startswith("test_")]
    for t in tests:
        t()
        print("ok", t.__name__)
    print("all pass")
