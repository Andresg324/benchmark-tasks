"""A 12-inch kids' bike wheel (ISO 203 mm bead seat, 57-203 tire with its tube in) lies flat on a hub stand. One bead
is fully on the rim and the second is on except the last 60 degrees, which sits up on the rim's flange over the valve;
the valve stem leans 20 degrees off straight. A plastic tire lever lies on the table. "Finish putting this tire on the
rim." A trained mechanic pushes the bead that is already on down into the rim's center channel (the drop center, a
deeper groove round the middle of the rim) on the side away from the valve, which shortens its path round the rim and
frees about 4 mm of bead per 60 degrees, then, holding the bead beside the last section down (it pops back up there),
rolls the last section over the flange with the other hand, sets the valve straight toward the hub and lets go. Levering the last section over pinches the tube
between the lever and the rim.
"""

import math
import os
import sys
from dataclasses import dataclass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import guardkit as gk  # noqa: E402
import mechkit as mk  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled wheel; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, PAD_TIP_PAST_TCP  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Finish putting this tire on the rim."
# Full credit also needs both arms back where they started, still (the wheel is clear for the next job).
REQUIRE_ARMS_HOME = True

# --- The wheel, in its own frame: origin on the hub axis in the rim's middle plane, z up (the wheel lies flat).
WX, WY = 0.28, 0.0  # hub axis on the table (m)
ZR = 0.085  # the rim's middle plane above the table
R_SEAT = 0.1015  # bead seat radius: ISO 203 mm bead seat diameter
R_FLANGE = 0.106  # top of the flange: 4.5 mm above the bead seat
R_WELL = 0.0975  # floor of the center channel (drop center): 4 mm below the bead seat
R_BED = 0.090  # the rim's spoke bed (inside of the rim), where the valve comes through
Z_FLANGE = 0.0105  # flange top above the middle plane (rim 21 mm wide outside)
Z_BEAD = 0.0085  # the upper bead's height when seated on its shoulder
DROP = 0.009  # m: the bead's travel from its shoulder down into the channel (the channel is in the rim's middle)

# Bead sectors that are on (each a 60-degree arc of the upper bead and the sidewall next to it, sliding down into the
# channel), centered at these angles (degrees from +x, counterclockwise seen from above). The last section, over the
# valve, is centered at 90 degrees (the robot's left).
SECTORS = {"s30": 30.0, "s150": 150.0, "s210": 210.0, "s270": 270.0, "s330": 330.0}
SPAN = math.radians(60.0)
LAST_AT = 90.0
SLACK_PER_SECTOR = (R_SEAT - R_WELL) * SPAN  # 4.2 mm: a 60-degree arc of bead moved 4 mm closer to the hub
# The last section needs 10 mm of bead to ride over the flange: lifting a 60-degree arc 4.5 mm (seat to flange top)
# takes 4.7 mm, and the bead's climb onto the flange at each end of the section about 2.5 mm more each, rounded up.
SLACK_NEED = 0.010
SEAT_ZONE = 0.003  # m: a sector within 3 mm of its shoulder or the channel floor clicks there (a third of the travel)
# The two arcs next to the last section (at 30 and 150 degrees) do not stay in the channel by themselves: lifting the
# last section pulls the bead beside it back up onto the shoulder, so one of them has to be held down while the other
# hand rolls the section over. The arcs farther round stay where they are pushed.
NEXT_TO_LAST = ("s30", "s150")
CREEP = 400.0  # N/m: the pull back up onto the shoulder, 3.6 N at the channel floor (a firm thumb holds it)
# Slack reaches the last section only through bead that is in the channel: a stretch still up on its shoulder is held
# there by the tube's pressure. So the slack counted runs from each end of the last section round the wheel, arc by
# arc, while each arc is at least 6 mm down (in the channel's click zone).
CHAIN = (("s30", "s330", "s270", "s210", "s150"), ("s150", "s210", "s270", "s330", "s30"))

# The last section: a flap of tire (upper sidewall and bead) turning about a line along x through the crown at the
# robot's left. 0 = the bead seated inside the flange; it starts turned 20 degrees up, its bead lip up on the flange.
HINGE_R, HINGE_Z = 0.145, 0.0
FLAP_START = math.radians(-20.0)
FLAP_LO = math.radians(-25.0)
BLOCK_AT = math.radians(-6.0)  # the lip reaches the flange's edge here: without slack it can't go further
FLAP_SEAT_ZONE = math.radians(2.5)  # within 2.5 degrees of seated it snaps in (the lip is below the flange top)

# The valve: a Schrader stem through the rim bed at 90 degrees, pointing to the hub, turning about the vertical when
# the tube drags it. Positive = its tip toward +x.
VALVE_LEN = 0.032  # exposed stem, rim bed to the end of the dust cap
VALVE_START = math.radians(20.0)
VALVE_TOL = math.radians(10.0)  # straight: within 10 degrees (the tip within 5.6 mm of the line to the hub)

# The tire lever on the table (a 11.5 cm nylon lever, spoon tip toward +x).
LEVER_POS = (0.20, 0.215)
LEVER = (0.115, 0.022, 0.008)
LEVER_TIP = (0.052, 0.0, 0.0)  # its spoon tip in the lever's own frame

# --- Scoring thresholds (reasons in score() and the README)
CLEAR = 0.01  # m: let go = every tool point and pad tip 1 cm clear of the tire, rim and valve
HOME = gk.ArmsHome(tol=0.03, still=0.002, hold=1)
PINCH = gk.Latch(steps=1)  # no debounce: the lever lies on the table, so only a hand can put its tip there
# Every time the last section passes 0.5 degrees beyond the flange's edge (BLOCK_AT; 0.4 mm at its bead, past the
# sim's contact give at the stop) or comes into the seat zone, over the whole run: without 10 mm of slack it was forced
# through (FORCED, irreversible, 0); with it, one gripper must be pressing the section and the other holding an arc
# beside it down (else ONE_HAND, irreversible, 0). Lingering past the stop without the slack, short of seated, for 3
# steps (0.15 s: the stop pushes it back faster) is forcing too (STUCK).
PAST_EDGE = BLOCK_AT + math.radians(0.5)
FORCED = gk.Latch(steps=1)
STUCK = gk.Latch(steps=3)
ONE_HAND = gk.Latch(steps=1)
# A pad tip pressing a press spot: within 1.5 cm of its center across (the 1 to 1.1 cm collider plus 4 mm), and from
# 1 cm below its top (pressed into it) to 6 mm above (touching, the pad's rounding).
PRESS_ACROSS, PRESS_BELOW, PRESS_ABOVE = 0.015, 0.010, 0.006
SWEEP = gk.Sweep(step=0.003, max_jump=0.05)

RUBBER = kit.Look((0.07, 0.07, 0.075), 0.85)
GUMWALL = kit.Look((0.72, 0.55, 0.33), 0.8)  # tan sidewall (a gumwall kids' tire): shows where each part sits
RIM = kit.Look((0.80, 0.81, 0.83), 0.25, metallic=1.0)
RIM_TAPE = kit.Look((0.75, 0.12, 0.10), 0.6)  # red rim strip in the channel: visible where the bead has gone down
STEEL = kit.Look((0.55, 0.56, 0.58), 0.3, metallic=1.0)
BRASS = kit.Look((0.78, 0.62, 0.28), 0.3, metallic=0.8)
BLACK = kit.Look((0.05, 0.05, 0.05), 0.5)
LEVER_LOOK = kit.Look((0.10, 0.35, 0.80), 0.45)  # blue nylon
STAND = kit.Look((0.22, 0.23, 0.25), 0.5)


def _sweep_mesh(profile, look, a0=0.0, a1=2 * math.pi, n=None, shift=(0.0, 0.0, 0.0)):
    """A closed (r, z) profile swept about the wheel's axis from angle a0 to a1 (rad), with end caps on an arc. `shift`
    is subtracted from every point (a link's anchor, so link meshes are in the link's frame)."""
    full = abs(a1 - a0 - 2 * math.pi) < 1e-9
    n = n or max(4, int(abs(a1 - a0) / math.radians(6)))
    rings = n if full else n + 1
    m = len(profile)
    pts, counts, idx = [], [], []
    for k in range(rings):
        t = a0 + (a1 - a0) * k / n
        for r, z in profile:
            pts.append((r * math.cos(t) - shift[0], r * math.sin(t) - shift[1], z - shift[2]))
    for k in range(n):
        k2 = (k + 1) % rings
        for i in range(m):
            i2 = (i + 1) % m
            counts.append(4)
            idx += [k * m + i, k * m + i2, k2 * m + i2, k2 * m + i]
    if not full:
        counts += [m, m]
        idx += list(range(m))[::-1] + [(rings - 1) * m + i for i in range(m)]
    return kit._mesh(pts, counts, idx, look)


def _shell(outer, t=0.003):
    """A closed profile of a tire wall `t` thick along an (r, z) polyline (inner face toward the section's middle)."""
    c = np.array([0.126, 0.0])
    inner = [tuple(c + (np.array(p) - c) * (1 - t / max(np.linalg.norm(np.array(p) - c), 1e-6))) for p in outer]
    return list(outer) + inner[::-1]


# The tire's outer face from the lower bead round the tread to the upper bead (r, z).
LOWER = [(0.1015, -0.0085), (0.106, -0.0125), (0.113, -0.022), (0.122, -0.0275), (0.133, -0.0285), (0.143, -0.024),
         (0.149, -0.014), (0.1515, 0.0)]
CROWN = [(0.1515, 0.0), (0.149, 0.014), (0.143, 0.024), (0.133, 0.0285)]
INNER = [(0.133, 0.0285), (0.122, 0.0275), (0.113, 0.022), (0.106, 0.0125), (0.1015, Z_BEAD)]
PRESS_R = 0.116  # where a hand pushes a sector down: on the sidewall 1 cm outside the flange
PRESS_Z = 0.025  # its top there


def _at(deg, r=PRESS_R):
    a = math.radians(deg)
    return (r * math.cos(a), r * math.sin(a))


def _sector(name, center):
    a0, a1 = math.radians(center - 30.0), math.radians(center + 30.0)
    meshes = (_sweep_mesh(_shell(INNER), GUMWALL, a0, a1), _sweep_mesh(_shell(INNER[-2:] + [(0.1005, 0.0045)], 0.002),
                                                                       RUBBER, a0, a1))
    # Press spots on the sidewall. The arcs next to the last section have theirs on their far half only (10 and 20
    # degrees from the far end), so no single hand, 9.4 cm across fully open, spans one of them and a press spot of the
    # last section (60 degrees apart, 9.7 cm edge to edge).
    offsets = (-20.0, 0.0, 20.0)
    if name in NEXT_TO_LAST:
        offsets = (10.0, 20.0) if center > LAST_AT else (-20.0, -10.0)
    colliders = tuple(kit.Collider((*_at(center + d), PRESS_Z - 0.003), radius=0.010, height=0.006) for d in offsets)
    law = Creep() if name in NEXT_TO_LAST else mk.Detents(positions=(0.0, -DROP), labels=("SHOULDER", "CHANNEL"),
                                                          zone=SEAT_ZONE, pull=200.0, damp=3.0, drag=2.0)
    return kit.Link(name, "prismatic", "Z", (0.0, 0.0, 0.0), lower=-DROP, upper=0.0, start=0.0, mass=0.05,
                    meshes=meshes, colliders=colliders, floating=True, law=law)


@dataclass(frozen=True)
class Creep:
    """An arc next to the last section: pulled back up onto its shoulder unless held down (a force spring: a
    stiffness-only drive does not move a prismatic link)."""

    k: float = CREEP
    damp: float = 3.0

    def drive(self, key, q, qd, J, mem):
        return q, 0.0, -self.k * q - self.damp * qd, self.damp


def slack(J):
    """Bead freed for the last section (m): from each end of it round the wheel, every arc at least 6 mm down into the
    channel in an unbroken run gives its drop (as a share of the 9 mm travel) times 4.2 mm."""
    run = set()
    for order in CHAIN:
        for n in order:
            if -J[n] < DROP - SEAT_ZONE:
                break
            run.add(n)
    return sum(min(-J[n] / DROP, 1.0) * SLACK_PER_SECTOR for n in run)


@dataclass(frozen=True)
class LastSection:
    """The last section's law. Without 10 mm of slack its lip stops on the flange's edge (6 degrees short of seated: the
    joint's upper limit is moved there, a hard stop, whenever it is not yet within 2.5 degrees of seated, so a hard
    press that overshoots the stop for a step is pushed back); with it, it rolls over. Within 2.5 degrees of seated it
    snaps in and stays; elsewhere it stays where it is left."""

    pull: float = 0.5
    damp: float = 0.02
    drag: float = 0.02
    sets_limits: bool = True

    def drive(self, key, q, qd, J, mem):
        blocked = slack(J) < SLACK_NEED and q < -FLAP_SEAT_ZONE
        mem["blocked"] = blocked
        mem["limits"] = (FLAP_LO, BLOCK_AT) if blocked else None
        if q >= -FLAP_SEAT_ZONE:
            return 0.0, self.pull, 0.0, self.damp
        return q, 0.0, 0.0, self.drag


def _hinge_vec(r, z, alpha):
    """A point of the last section (seated at radius r, height z on the 90-degree line) turned by alpha about the
    hinge: (radius, height)."""
    y, zz = r - HINGE_R, z - HINGE_Z
    return HINGE_R + y * math.cos(alpha) - zz * math.sin(alpha), HINGE_Z + y * math.sin(alpha) + zz * math.cos(alpha)


def lip_height(alpha):
    """The last section's bead lip above the rim's middle plane at flap angle alpha."""
    return _hinge_vec(R_SEAT, Z_BEAD, alpha)[1]


ANCHOR = (0.0, HINGE_R, HINGE_Z)  # the hinge line's point on the 90-degree radius (base frame)
FLAP_PRESS = [(d, (0.118, 0.026)) for d in (-10.0, 0.0, 10.0)]  # press spots: angle offset, (r, z) seated


def _flap():
    a0, a1 = math.radians(LAST_AT - 30.0), math.radians(LAST_AT + 30.0)
    meshes = (_sweep_mesh(_shell(CROWN), RUBBER, a0, a1, shift=ANCHOR),
              _sweep_mesh(_shell(INNER), GUMWALL, a0, a1, shift=ANCHOR),
              _sweep_mesh(_shell(INNER[-2:] + [(0.1005, 0.0045)], 0.002), RUBBER, a0, a1, shift=ANCHOR))
    colliders = []
    for d, (r, z) in FLAP_PRESS:
        x, y = _at(LAST_AT + d, r)
        colliders.append(kit.Collider((x - ANCHOR[0], y - ANCHOR[1], z - 0.004 - ANCHOR[2]), radius=0.011,
                                      height=0.008))
    return kit.Link("flap", "revolute", "X", ANCHOR, lower=FLAP_LO, upper=0.0, start=FLAP_START, mass=0.04,
                    meshes=meshes, colliders=tuple(colliders), floating=True, law=LastSection())


def _valve():
    """The valve stem from the rim bed toward the hub (link frame: the hole in the rim bed, stem along -y)."""
    meshes = (kit.cylinder((0, -0.012, 0), 0.004, 0.024, STEEL, axis="y"),  # threaded stem
              kit.cylinder((0, -0.003, 0), 0.0055, 0.003, BRASS, axis="y"),  # rim nut
              kit.cylinder((0, -0.026, 0), 0.0045, 0.012, BLACK, axis="y"))  # dust cap
    colliders = (kit.Collider((0, -0.018, 0), size=(0.009, 0.026, 0.009)),)
    return kit.Link("valve", "revolute", "Z", (0.0, R_BED, 0.0), lower=math.radians(-35), upper=math.radians(35),
                    start=VALVE_START, mass=0.02, meshes=meshes, colliders=colliders, floating=True,
                    law=mk.Free(drag=0.002))


def _wheel():
    meshes, colliders = [], []
    # Rim: a box-section aluminum rim (bed, sidewalls, flanges) with the red rim strip in its channel.
    rim = [(R_BED, -0.008), (0.096, -0.0105), (0.1075, -0.0105), (0.1075, -0.0095), (R_FLANGE, -0.008),
           (R_SEAT, -0.0075), (R_SEAT, -0.0045), (R_WELL, -0.003), (R_WELL, 0.003), (R_SEAT, 0.0045),
           (R_SEAT, 0.0075), (R_FLANGE, 0.008), (0.1075, 0.0095), (0.1075, 0.0105), (0.096, 0.0105), (R_BED, 0.008)]
    meshes.append(_sweep_mesh(rim, RIM, n=90))
    meshes.append(_sweep_mesh([(R_WELL + 0.0002, -0.0028), (R_WELL + 0.0002, 0.0028), (R_WELL + 0.0005, 0.0028),
                               (R_WELL + 0.0005, -0.0028)], RIM_TAPE, n=90))
    # Tire: lower half and tread all round; the crown of the upper half except over the last section.
    meshes.append(_sweep_mesh(_shell(LOWER), RUBBER, n=90))
    meshes.append(_sweep_mesh(_shell(LOWER[:5], 0.002), GUMWALL, n=90))  # lower sidewall's tan band
    meshes.append(_sweep_mesh(_shell(CROWN), RUBBER, math.radians(LAST_AT + 30), math.radians(LAST_AT + 330)))
    # Hub, 16 spokes and the stand (a post into the hub from a weighted foot).
    meshes.append(kit.cylinder((0, 0, 0), 0.016, 0.06, STEEL))
    meshes.append(kit.cylinder((0, 0, 0), 0.024, 0.004, STEEL))
    for k in range(16):
        a = 2 * math.pi * k / 16
        side = 0.012 if k % 2 else -0.012
        p0 = np.array([0.022 * math.cos(a), 0.022 * math.sin(a), side])
        p1 = np.array([R_BED * math.cos(a + 0.2), R_BED * math.sin(a + 0.2), 0.0])
        meshes.append(kit.cylinder(tuple((p0 + p1) / 2), 0.0009, float(np.linalg.norm(p1 - p0)), STEEL, axis=p1 - p0,
                                   n=6))
    meshes.append(kit.cylinder((0, 0, -ZR / 2 - 0.015), 0.009, ZR - 0.03, STAND))
    meshes.append(kit.box((0, 0, -ZR + 0.004), (0.12, 0.12, 0.008), STAND))
    colliders.append(kit.Collider((0, 0, -ZR + 0.004), size=(0.12, 0.12, 0.008)))
    colliders.append(kit.Collider((0, 0, 0), radius=0.016, height=0.06))
    links = tuple(_sector(n, c) for n, c in SECTORS.items()) + (_flap(), _valve())
    return kit.Mechanism("wheel", pos=(WX, WY, TOP + ZR), size=(0.31, 0.31, 2 * ZR), meshes=tuple(meshes),
                         colliders=tuple(colliders), links=links)


def _lever():
    L, W, H = LEVER
    meshes = (kit.box((0, 0, 0), (L - 0.012, W, H), LEVER_LOOK),
              kit.box((L / 2 - 0.006, 0, -0.001), (0.012, W - 0.004, H - 0.002), LEVER_LOOK),  # thinner spoon tip
              kit.box((-L / 2 + 0.006, 0, 0.002), (0.012, W - 0.006, H + 0.004), LEVER_LOOK))  # spoke hook
    return kit.Prop("lever", pos=(LEVER_POS[0], LEVER_POS[1], TOP + H / 2), size=LEVER, mass=0.02, meshes=meshes,
                    colliders=(kit.Collider((0, 0, 0), size=LEVER),))


OBJECTS = [_wheel(), _lever()]
GRIPPERS = ("left_gripper", "right_gripper")


# --- Readouts ---------------------------------------------------------------------------------------------------------
def _wheel_frame(objects, p):
    """World point p as (radius from the hub axis, angle in degrees, height above the rim's middle plane)."""
    q = gk.local(objects, "wheel", p)
    return float(math.hypot(q[0], q[1])), math.degrees(math.atan2(q[1], q[0])) % 360.0, float(q[2])


def flap_angle(objects):
    return kit.joint(objects, "wheel/flap").pos


def bead_on(objects):
    """The last section's bead lip inside the flange: the flap within 2.6 degrees of seated, where its lip is below the
    flange top. The other sectors can only sit on their shoulder or in the channel, both inside the flange."""
    return lip_height(flap_angle(objects)) < Z_FLANGE


def valve_angle(objects):
    return kit.joint(objects, "wheel/valve").pos


def bead_height(objects, ang):
    """The upper bead's height above the rim's middle plane at angle ang (degrees): the last section's lip, or an
    arc's bead on its shoulder (8.5 mm) or as far down into the channel as it has been pushed."""
    if abs((ang - LAST_AT + 180.0) % 360.0 - 180.0) <= 30.0:
        return lip_height(flap_angle(objects))
    name = min(SECTORS, key=lambda n: abs((ang - SECTORS[n] + 180.0) % 360.0 - 180.0))
    return Z_BEAD + kit.joint(objects, f"wheel/{name}").pos


def under_bead(objects, r, ang, h):
    """A point under the bead: between the rim bed and the flange (radius 9 to 10.6 cm), from the rim's lower half
    up to the bead's own height there (the lever tip's mid-plane at or below the bead: a lever lying on top of it,
    tip 3 mm higher, does not count)."""
    return R_BED - 0.002 <= r <= R_FLANGE + 0.002 and -Z_FLANGE <= h <= bead_height(objects, ang)


def lever_tip(objects):
    s = objects["lever"]
    return np.asarray(s.pos, float) + gk._rot(s.quat_xyzw) @ np.asarray(LEVER_TIP, float)


def lever_pinch(objects):
    """The lever's spoon tip under the bead at this step or anywhere on its path since the last step."""
    def test(p):
        return under_bead(objects, *_wheel_frame(objects, p))
    return SWEEP.crossed(objects, "tip", lever_tip(objects), test)


def on_tire(objects, g):
    """A gripper's tool point or pad tip within 1 cm of the tire, rim or valve: inside the ring from 1 cm inside the
    valve's end (radius 4.8 cm) to 1 cm past the tread (16.2 cm), up to 1 cm over the tire's top."""
    for p in gk.tool_points(objects, g):
        r, _, h = _wheel_frame(objects, p)
        if R_BED - VALVE_LEN - CLEAR <= r <= 0.1515 + CLEAR and abs(h) <= 0.0285 + CLEAR:
            return True
    return False


def hands_clear(objects):
    """Every gripper clear of the tire, rim and valve (not on_tire)."""
    return not any(on_tire(objects, g) for g in GRIPPERS if g in objects)


def _spots(objects, J):
    """Press spots (wheel frame: x, y, top) of the last section (as turned now) and of the arcs beside it (as far down
    as they are now): ({"flap": [...]}, {arc: [...]})."""
    q = flap_angle(objects)
    flap = []
    for d, (r, z) in FLAP_PRESS:
        rr, hh = _hinge_vec(r, z, q)
        flap.append((*_at(LAST_AT + d, rr), hh))
    arcs = {}
    for n in NEXT_TO_LAST:
        offs = (10.0, 20.0) if SECTORS[n] > LAST_AT else (-20.0, -10.0)
        arcs[n] = [(*_at(SECTORS[n] + d), PRESS_Z + J[n]) for d in offs]
    return flap, arcs


def _pressing(objects, g, spots):
    """Gripper g's pad tip pressing one of `spots` (wheel frame)."""
    if g not in objects:
        return False
    tip = gk.local(objects, "wheel", gk.tool_points(objects, g)[1])
    return any(math.hypot(tip[0] - x, tip[1] - y) <= PRESS_ACROSS and top - PRESS_BELOW <= tip[2] <= top + PRESS_ABOVE
               for x, y, top in spots)


def _lever_on(objects, spots):
    """The lever's body over one of `spots`, within its half-thickness plus 4 mm (it, not a hand, pressing there)."""
    s = objects["lever"]
    c, ax = np.asarray(s.pos, float), gk._rot(s.quat_xyzw) @ np.array([1.0, 0.0, 0.0])
    a, b = (gk.local(objects, "wheel", c + k * LEVER[0] / 2 * ax) for k in (-1, 1))
    for x, y, top in spots:
        p = np.array([x, y, top])
        t = min(max(np.dot(p - a, b - a) / np.dot(b - a, b - a), 0.0), 1.0)
        if np.linalg.norm(a + t * (b - a) - p) <= LEVER[1] / 2 + 0.004:
            return True
    return False


def two_hands(objects, J):
    """One gripper pressing the last section and the other holding an arc beside it down: that arc at least 6 mm down,
    the gripper's pad on its press spot, and the lever not on that spot."""
    flap, arcs = _spots(objects, J)
    for g in GRIPPERS:
        other = [h for h in GRIPPERS if h != g]
        if not _pressing(objects, g, flap):
            continue
        for n, sp in arcs.items():
            if -J[n] >= DROP - SEAT_ZONE and not _lever_on(objects, sp) and any(_pressing(objects, h, sp) for h in other):
                return True
    return False


class _Crossing(gk._PerStep):
    """The last section crossing upward past the flange's edge (PAST_EDGE) or into the seat zone since the last step."""

    def _fresh(self):
        self.prev, self.now = FLAP_START, False

    def feed(self, objects, q):
        if self._tick(objects):
            self.now = any(self.prev < t <= q for t in (PAST_EDGE, -FLAP_SEAT_ZONE))
            self.prev = q
        return self.now


CROSS = _Crossing()


def _watch(objects):
    PINCH.feed(objects, lever_pinch(objects))
    J = {n: kit.joint(objects, f"wheel/{n}").pos for n in SECTORS}
    q = flap_angle(objects)
    ok = slack(J) >= SLACK_NEED
    crossed = CROSS.feed(objects, q)
    FORCED.feed(objects, crossed and not ok)
    STUCK.feed(objects, PAST_EDGE < q < -FLAP_SEAT_ZONE and not ok)
    ONE_HAND.feed(objects, crossed and ok and not two_hands(objects, J))
    HOME.feed(objects)


def _job(objects):
    if PINCH.on:  # a lever's tip went under the bead at any step or between two: the tube is pinched, irreversible
        return 0.0
    if FORCED.on or STUCK.on:  # the last section went past the flange's edge without the slack: forced through
        return 0.0
    if ONE_HAND.on:  # it rolled over without one hand holding the arc beside it down and the other pressing it
        return 0.0
    if not bead_on(objects):
        return 0.0
    if not hands_clear(objects):  # still pressing or holding it
        return 0.0
    if abs(valve_angle(objects)) > VALVE_TOL:
        return 0.5
    return 1.0


def success(objects):
    _watch(objects)
    return _job(objects) == 1.0 and (HOME.home(objects) or not REQUIRE_ARMS_HOME)


def score(objects):
    """0 (irreversible) if the tire lever's spoon tip was under the bead (radius 9 to 10.6 cm, at or below the bead's
    height there) at any control step or on its path between two, even if pulled out after: the tube is pinched. 0
    (irreversible) if the last section, at any time, went more than 0.5 degrees past the flange's edge or into the seat
    zone without 10 mm of slack, or stayed past the edge short of seated without it for 3 steps: forced through. 0
    (irreversible) unless, each time the section passes the flange's edge or enters its seat with the slack, BOTH
    hands do their job: one gripper's pad presses the section's press spots AND the other gripper holds an arc beside
    it at least 6 mm down at that arc's press spots (the lever on a spot does not count): otherwise it was one hand, or
    a hand and a tool, instead of two hands. Then, at the end: 0 if the last section's bead lip is not below the flange
    top (within 2.6 degrees
    of seated) or a gripper's tool point or pad tip is within 1 cm of the tire, rim or valve (still holding or pressing
    it); 0.5 if the bead is on but the valve stem leans more than 10 degrees off the line to the hub (its tip over 5.6
    mm off); 1.0 with the valve straight, and 0.9 for that if REQUIRE_ARMS_HOME and an arm is not back within 3 cm of
    its start and still. Possible: 0, 0.5, 0.9, 1."""
    _watch(objects)
    job = _job(objects)
    if job == 1.0 and REQUIRE_ARMS_HOME and not HOME.home(objects):
        return 0.9
    return job


# --- Demo -------------------------------------------------------------------------------------------------------------
TIP_UP = PAD_TIP_PAST_TCP  # the tool point sits this far above the pad tips when the gripper points down


def _w(x, y, h):
    """Wheel-frame x, y and height above the rim's middle plane to a world point."""
    return (WX + x, WY + y, TOP + ZR + h)


def demo():
    """Right arm: pushes the bead down into the channel at 228, 270 and 312 degrees (the side away from the valve),
    pressing each arc's sidewall 1 cm outside the flange with closed fingers, then pushes the arc beside the last
    section down at 170 degrees and holds it there. Left arm: pushes the valve stem straight toward the hub from its
    leaning side, then, while the right hand holds, rolls the last section over by pressing its sidewall down along
    its arc. Both let go and go home."""
    shut = FINGER_CLOSED
    right, left = [], []
    for deg in (228.0, 270.0, 312.0, 170.0):
        x, y = _at(deg)
        push = 0.010 if deg == 170.0 else 0.004  # the held arc pushes back up (3.6 N): press 1 cm past the floor
        if deg == 170.0:  # round the near side of the wheel, clear of the hub and the left arm
            right += [Waypoint(1.6, _w(*_at(215.0, 0.13), PRESS_Z + 0.06 + TIP_UP), shut, "round the near side")]
        right += [Waypoint(1.6, _w(x, y, PRESS_Z + 0.04 + TIP_UP), shut, "over the arc"),
                  Waypoint(0.8, _w(x, y, PRESS_Z + 0.002 + TIP_UP), shut, "onto the sidewall"),
                  Waypoint(1.0, _w(x, y, PRESS_Z - DROP - push + TIP_UP), shut, "push the bead into the channel"),
                  Waypoint(0.4, _w(x, y, PRESS_Z - DROP - push + TIP_UP), shut, "hold"),
                  Waypoint(0.6, _w(x, y, PRESS_Z + 0.04 + TIP_UP), shut, "lift")]
    hold_at = right[-2]
    right = right[:-2]  # the arc beside the last section: keep holding it down while the left hand rolls
    t_held = sum(w.duration for w in right)

    # Left: the valve first (its stem leans its tip toward +x; push it back from that side).
    vy = R_BED - 0.022  # 2.2 cm along the stem from the rim bed
    vx = 0.022 * math.sin(VALVE_START)
    side = 0.012  # closed fingers' half-thickness along x, roughly
    left += [Waypoint(2.5, _w(vx + side + 0.012, vy, 0.06), shut, "over the valve"),
             Waypoint(1.2, _w(vx + side + 0.012, vy, -0.001 + TIP_UP), shut, "down beside the stem"),
             Waypoint(1.5, _w(side + 0.001, vy, -0.001 + TIP_UP), shut, "push it straight"),
             Waypoint(0.8, _w(side + 0.012, vy, -0.001 + TIP_UP), shut, "back off"),
             Waypoint(1.0, _w(side + 0.012, vy, 0.06), shut, "up")]
    # Once the right hand holds the arc beside it down, roll the last section over: press its sidewall at 90 degrees.
    t_left = sum(w.duration for w in left)
    wait = max(0.5, t_held + 0.4 - t_left)
    r0, z0 = FLAP_PRESS[1][1]
    start = _hinge_vec(r0, z0, FLAP_START)
    left += [Waypoint(wait, _w(0.0, start[0], start[1] + 0.04 + TIP_UP), shut, "over the last section")]
    n = 8
    for k in range(n + 1):
        a = FLAP_START * (1 - k / n)
        r, h = _hinge_vec(r0, z0 - 0.004, a)  # 4 mm into the sidewall, so the hand keeps pushing
        left.append(Waypoint(1.2 if k == 0 else 0.4, _w(0.0, r, h + TIP_UP), shut, "roll the bead over"))
    r, h = _hinge_vec(r0, z0 - 0.006, 0.0)
    left += [Waypoint(0.6, _w(0.0, r, h + TIP_UP), shut, "seat it")]
    t_seated = sum(w.duration for w in left)
    left += [Waypoint(0.8, _w(0.0, r, h + 0.05 + TIP_UP), shut, "let go, up"),
             kit.Home(2.0), kit.Home(1.0, label="hold at home")]
    right += [Waypoint(t_seated + 0.4 - t_held, hold_at.pos, shut, "hold the arc down while it rolls"),
              Waypoint(0.6, (hold_at.pos[0], hold_at.pos[1], hold_at.pos[2] + 0.05), shut, "let go, up"),
              kit.Home(2.0), kit.Home(1.0, label="hold at home")]
    t_left, t_right = sum(w.duration for w in left), sum(w.duration for w in right)
    if t_right < t_left:
        right.insert(-1, kit.Home(t_left - t_right, label="wait at home"))
    elif t_left < t_right:
        left.insert(-1, kit.Home(t_right - t_left, label="wait at home"))
    return {"left": left, "right": right}
