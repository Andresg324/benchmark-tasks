"""Offline checks of pivotkit (no simulator): the record kept per step, and the joint limits its laws ask for.

    /data/IsaacLab-Arena/.venv/bin/python test_pivotkit.py      (with yamkit.py next to it; prints "all pass")
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import pivotkit as pv  # noqa: E402
import yamkit as kit  # noqa: E402

KEY = "m/part"
PIN_IN = {"v": False}
PV = pv.Pivot("part", engaged=lambda J: PIN_IN["v"], resist=lambda J: (50.0, 0.5) if J.get("foot", 0) < 0 else None)
LINKS = pv.planar(PV, pin=(0, 0, 0.002), reach=((-0.05, 0.05), (-0.04, 0.04)), turn=(-1.0, 2.0), mass=0.05)
PVF = pv.Pivot("leg", engaged=lambda J: PIN_IN["v"], free="fixed")
LINKS_F = pv.planar(PVF, pin=(0, 0, 0.0), reach=((-0.05, 0.05), (-0.05, 0.05)), turn=(-3.0, 3.0), mass=0.05)


def new_trial():
    kit._EPISODE["count"] += 1


def drive(links, J):
    """One control step through the three laws, in chain order, as the kit calls them; returns each joint's
    (damping, limits)."""
    out = []
    for lk in links:
        mem = {}
        r = lk.law.drive(f"m/{lk.name}", J[lk.name], 0.0, J, mem)
        out.append((r[3], mem["limits"]))
    return out


def J(th=0.0, x=0.0, y=0.0, foot=1.0, name="part"):
    return {f"{name}_turn": th, f"{name}_x": x, name: y, "foot": foot}


def check(cond, what):
    if not cond:
        raise AssertionError(what)
    print("ok  ", what)


def test_chain():
    a, b, c = LINKS
    check((a.kind, a.axis, a.parent) == ("revolute", "Z", None) and (b.kind, b.axis, b.parent) == (
        "prismatic", "X", "part_turn") and (c.kind, c.axis, c.parent) == ("prismatic", "Y", "part_x"),
        "planar() builds turn -> x -> y, the part last")
    usd = kit._mech_usda(kit.Mechanism("m", pos=(0, 0, 0), size=(0.1, 0.1, 0.1), links=LINKS))
    check("body0 = </Root/part_x>" in usd and "body0 = </Root/part_turn>" in usd, "the USD chains the joints")
    check("physics:diagonalInertia" in usd, "collider-less intermediate links get an explicit inertia")
    check(usd.count("PhysicsDriveAPI:") == 3, "each chain joint gets a PhysX drive (implicit damping)")


def test_free_slide_and_turn():
    new_trial()
    PIN_IN["v"] = False
    r = drive(LINKS, J())
    check(all(lim is None for _, lim in r), "pin out: no narrowed stops (free='slide')")
    drive(LINKS, J(0.1, 0.01, 0.0))
    check(abs(pv.free_turn_deg(None, KEY) - math.degrees(0.1)) < 1e-9 and pv.turned_deg(None, KEY) == 0.0,
          "turning with the pin out counts as free turn, not as a pivot")
    check(not pv.engaged(None, KEY) and pv.entries(None, KEY) == 0, "never engaged")
    check(abs(pv.pin_offset(None, KEY)[0] + 0.01) < 1e-12, "pin offset follows the slide (part moved +x: pin at -x)")


def test_engaged_turn_only():
    new_trial()
    PIN_IN["v"] = False
    drive(LINKS, J())
    drive(LINKS, J(0.0, 0.003, -0.002))
    PIN_IN["v"] = True
    r = drive(LINKS, J(0.0, 0.003, -0.002))
    check(r[0][1] is None, "pin in: the turn stays free")
    check(abs(r[1][1][0] - 0.0025) < 1e-9 and abs(r[1][1][1] - 0.0035) < 1e-9, "x stops narrowed to +/- tol")
    check(abs(r[2][1][0] + 0.0025) < 1e-9 and abs(r[2][1][1] + 0.0015) < 1e-9, "y stops narrowed to +/- tol")
    check(pv.entered_at(None, KEY) == (-0.003, 0.002), "entered_at records where on the part the pin went in")
    for k in range(1, 31):
        drive(LINKS, J(math.radians(3 * k), 0.003, -0.002))
    check(abs(pv.turned_deg(None, KEY) - 90) < 1e-6 and abs(pv.angle_deg(None, KEY) - 90) < 1e-6
          and abs(pv.pin_offset(None, KEY)[0] + 0.003) < 1e-12,
          "a 90 deg turn while in is counted as pivoting")
    check(pv.free_turn_deg(None, KEY) == 0.0 and not pv.slid(None, KEY), "no free turn, no slide")
    drive(LINKS, J(math.radians(90), 0.0045, -0.002))
    check(not pv.slid(None, KEY) and abs(pv.max_slip_mm(None, KEY) - 1.5) < 1e-6, "1.5 mm of play is not a slide")
    drive(LINKS, J(math.radians(90), 0.0055, -0.002))
    check(pv.slid(None, KEY), "2.5 mm forced through the stops counts as slid")
    PIN_IN["v"] = False
    r = drive(LINKS, J(math.radians(90), 0.003, -0.002))
    check(pv.slid(None, KEY) and all(lim is None for _, lim in r), "slid stays recorded after the pin comes out")


def test_partial_entry_counts_free():
    new_trial()
    PIN_IN["v"] = False
    drive(LINKS, J())
    PIN_IN["v"] = True
    drive(LINKS, J(0.2))  # the step it went in: the turn may have happened before
    check(pv.turned_deg(None, KEY) == 0.0 and pv.free_turn_deg(None, KEY) > 11, "a turn on the entry step is free")
    PIN_IN["v"] = False
    drive(LINKS, J(0.25))
    check(abs(pv.free_turn_deg(None, KEY) - math.degrees(0.25)) < 1e-9, "a turn on the exit step is free too")
    check(pv.entries(None, KEY) == 1, "one entry")


def test_resist():
    new_trial()
    PIN_IN["v"] = False
    r = drive(LINKS, J(foot=-1))
    check(r[0][0] == 0.5 and r[1][0] == 50.0, "resist() raises the damping (foot down)")
    r = drive(LINKS, J(foot=1))
    check(r[0][0] == PV.turn_damp and r[2][0] == PV.slide_damp, "and the defaults return when it doesn't")


def test_fixed_mode():
    new_trial()
    PIN_IN["v"] = False
    r = drive(LINKS_F, J(0.3, 0.01, 0.02, name="leg"))
    check(r[0][1] is not None and abs(r[1][1][0] - 0.0095) < 1e-9 and abs(r[2][1][1] - 0.0205) < 1e-9,
          "free='fixed': all three joints held while the pin is out")
    PIN_IN["v"] = True
    r = drive(LINKS_F, J(0.3, 0.01, 0.02, name="leg"))
    check(r[0][1] is None and r[1][1] is not None, "pin in: only the turn is free")
    PIN_IN["v"] = False
    r = drive(LINKS_F, J(1.3, 0.01, 0.02, name="leg"))
    check(abs(sum(r[0][1]) / 2 - 1.3) < 1e-9, "pin out again: held where it was left")


def test_reset():
    new_trial()
    PIN_IN["v"] = True
    drive(LINKS, J())
    drive(LINKS, J(1.0))
    new_trial()
    check(pv.turned_deg(None, KEY) == 0.0 and pv.entries(None, KEY) == 0 and not pv.engaged(None, KEY),
          "kit.episode() changing gives a fresh record")


if __name__ == "__main__":
    for name, fn in list(globals().items()):
        if name.startswith("test_"):
            fn()
    print("all pass")
