"""A moka pot's screw thread as collider segments, so the upper chamber really threads onto the base, and a crooked
start can cross-thread (the ridges land in the wrong groove and jam) instead of being scored as crooked.

Geometry (the base's male thread on its collar, the upper chamber's female thread in its skirt), all ESTIMATES until
measured on a real 3-cup Moka Express (Bialetti publishes no thread data; see ~/work/status/rev-1506-moka.md):
- major diameter 66 mm (the task's collar; a 3D-print remake of the 3-cup funnel gives 65 mm for the thread),
- single start, right hand, pitch 3.0 mm, flank-to-flank ridge width 1.2 mm on each part (0.3 mm flank clearance a
  side), depth 1.2 mm, 0.4 mm radial clearance at crest and root,
- male thread 1.5 turns, female 1.5 turns: from first catch to the seat about 1.4 turns.

Each ridge is a helix of short boxes (one per `SEG_DEG` of turn), each tilted to the helix's lead angle, so a box's
ends meet its neighbors' within ~0.05 mm. Boxes are straight chords: at 10 degrees a chord's corners stand 0.13 mm
proud of the arc, inside the 0.4 mm radial clearance.

Frames: the base's thread is built in a frame whose origin is on the axis at the rim (z up); the top's in the top's
own frame (origin at its bottom face, which rests on the rim at the seat). Clockwise seen from above tightens.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import yamkit as kit


@dataclass(frozen=True)
class Thread:
    pitch: float = 0.0030
    major_r: float = 0.0330  # male crest radius
    depth: float = 0.0012  # ridge depth (both parts)
    radial_clear: float = 0.0004  # crest-to-root clearance
    ridge_w: float = 0.0012  # ridge width along the axis (each part)
    male_turns: float = 1.5
    female_turns: float = 1.5
    male_top: float = -0.0065  # height of the male ridge's topmost center below the rim
    seg_deg: float = 10.0

    @property
    def male_root_r(self):
        return self.major_r - self.depth

    @property
    def female_root_r(self):
        return self.major_r + self.radial_clear

    @property
    def female_crest_r(self):
        return self.female_root_r - self.depth

    def lead(self, r):
        return math.atan2(self.pitch, 2 * math.pi * r)


MOKA = Thread()


def _quat_axis(axis, ang):
    s = math.sin(ang / 2)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, math.cos(ang / 2)])


def _qmul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by, aw * by - ax * bz + ay * bw + az * bx,
                     aw * bz + ax * by - ay * bx + az * bw, aw * bw - ax * bx - ay * by - az * bz])


def segments(th: Thread, r_in, r_out, z0, turns):
    """Boxes along a right-hand helix from angle 0 (height z0) up `turns` turns: (center, size, quat_xyzw)."""
    n = max(1, int(round(turns * 360.0 / th.seg_deg)))
    du = math.radians(th.seg_deg)
    rc = (r_in + r_out) / 2
    lead = th.lead(rc)
    length = 2 * r_out * math.tan(du / 2) * 1.02  # chord at the outer face, 2 % overlap
    out = []
    for k in range(n):
        u = (k + 0.5) * du
        z = z0 + th.pitch * u / (2 * math.pi)
        c = (rc * math.cos(u), rc * math.sin(u), z)
        # box axes: x radial, y tangential, z along the axis; turned by u about z, then tilted by the lead angle about
        # its radial axis so its length follows the helix (rising counterclockwise: right hand)
        q = _qmul(_quat_axis((0, 0, 1), u), _quat_axis((1, 0, 0), lead))
        out.append((c, (r_out - r_in, length, th.ridge_w / math.cos(lead)), tuple(float(v) for v in q)))
    return out


def male(th: Thread = MOKA, rim_z=0.0, phase_deg=0.0):
    """The base's ridge, in a frame with the rim at `rim_z`."""
    z0 = rim_z + th.male_top - th.pitch * th.male_turns
    return _turn(segments(th, th.male_root_r - 0.0002, th.major_r, z0, th.male_turns), phase_deg)


def female(th: Thread = MOKA, phase_deg=0.0):
    """The top's ridge in its own frame (bottom face at z 0): at the seat each female ridge sits half a pitch above
    the male ridge at the same angle."""
    z0 = th.male_top - th.pitch * th.male_turns + th.pitch / 2
    return _turn(segments(th, th.female_crest_r, th.female_root_r + 0.0002, z0, th.female_turns), phase_deg)


def _turn(segs, phase_deg):
    if not phase_deg:
        return segs
    q0 = _quat_axis((0, 0, 1), math.radians(phase_deg))
    c, s = math.cos(math.radians(phase_deg)), math.sin(math.radians(phase_deg))
    return [((c * p[0] - s * p[1], s * p[0] + c * p[1], p[2]), size, tuple(float(v) for v in _qmul(q0, q)))
            for p, size, q in segs]


def colliders(segs):
    return tuple(kit.Collider(tuple(float(v) for v in p), size=tuple(float(v) for v in size), quat=q)
                 for p, size, q in segs)


def ridge_mesh(th: Thread, r, z0, turns, look, phase_deg=0.0):
    """A visible ridge along the same helix (a band)."""
    u = np.linspace(0, 2 * math.pi * turns, int(turns * 72) + 1) + math.radians(phase_deg)
    pts = [(r * math.cos(a), r * math.sin(a), z0 + th.pitch * (a - math.radians(phase_deg)) / (2 * math.pi))
           for a in u]
    return kit.band(pts, th.depth, th.ridge_w, look, normal=(0, 0, 1))


def seat_ring(r_in, r_out, z0, z1, n=36):
    """A flat annular face as `n` oriented boxes (a collider tuple). Use it for both faces of the seat (the base's rim,
    the upper chamber's bottom face or gasket): in probe p2 a cylinder resting on a cylinder sank 1.1 mm at 1 N*m and
    2.9 mm at 3 N*m of screwing torque (the thread's wedge load, ~60 N per N*m); box rings held 0.07 mm and 0.24 mm
    (probe p3). Keep n high enough that the chord corners (r_out / cos(pi / n)) stay clear of the thread."""
    rc = (r_in + r_out) / 2
    w = 2 * r_out * math.tan(math.pi / n) * 1.02
    return tuple(kit.Collider((rc * math.cos(2 * math.pi * k / n), rc * math.sin(2 * math.pi * k / n), (z0 + z1) / 2),
                              size=(r_out - r_in, w, z1 - z0),
                              quat=(0.0, 0.0, math.sin(math.pi * k / n), math.cos(math.pi * k / n)))
                 for k in range(n))


# -- the gasket: a sprung seat ------------------------------------------------------------------------------------------

@dataclass(frozen=True)
class Gasket:
    """The rubber gasket between the upper chamber and the base, as a ring of boxes on the upper chamber with PhysX
    compliant contact (a sprung seat), so that tightening past touch compresses it and the screwing torque rises with
    the turn instead of stopping dead on a rigid seat.

    Stiffness, all ESTIMATES (no published data on Moka Express gaskets): food-grade rubber ~65 Shore A, E ~4.4 MPa
    (Gent 1958's Shore-A-to-modulus relation); it bears on the funnel's flange over a ~2 mm wide, ~57 mm mean diameter
    band (A ~3.6e-4 m^2) through ~3 mm of rubber: k = E A / t ~ 5e5 N/m (range ~2e5-1.4e6 for 1-4 mm of band and E 3-6
    MPa). At k = 5e5 the 82-156 N clamp of firm hand-tight (2.9-5.5 N*m, Voorbij & Steenbekkers 2002, through the
    thread mechanics in the status file) is 0.16-0.31 mm of compression, 20-37 degrees of turn past touch at the 3.0 mm
    pitch. `protrude` is how far the gasket stands proud of the upper chamber's metal face: past that the metal seats
    (bottoms out) on the rim.

    PhysX's compliant stiffness acts per contact point, so `k_point` is a guess at k / (number of points): calibrate it
    with known axial loads in the sim (probe p4) and set it so the measured stiffness matches `k`."""

    k: float = 5.0e5  # N/m, the gasket as a whole (ESTIMATE, see above)
    k_point: float = 2.5e3  # N/m per PhysX contact point (calibrated in the sim)
    damping_point: float = 5.0  # N*s/m per contact point
    r_in: float = 0.026
    r_out: float = 0.031
    thick: float = 0.002  # the collider's thickness; only `protrude` of it stands below the metal face
    protrude: float = 0.0008
    friction: tuple = (0.6, 0.5)  # rubber on aluminum, ESTIMATE; combined by "min" so it wins over the metal's
    n: int = 24

    def surface(self):
        return kit.Surface(friction=self.friction, stiffness=self.k_point, damping=self.damping_point, combine="min")

    def colliders(self):
        """The ring in the upper chamber's frame (its metal bottom face at z 0): it spans z -protrude .. thick -
        protrude."""
        ring = seat_ring(self.r_in, self.r_out, -self.protrude, self.thick - self.protrude, n=self.n)
        sf = self.surface()
        return tuple(kit.Collider(c.center, size=c.size, quat=c.quat, surface=sf) for c in ring)

    def clamp(self, compression):
        """The clamp force (N) at a compression (m), linear, until the metal bottoms out."""
        return self.k * max(0.0, min(compression, self.protrude))


GASKET = Gasket()


# -- offline geometry checks (no sim) ----------------------------------------------------------------------------------

def _rotm(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _surface_pts(size, n=5):
    h = np.asarray(size) / 2
    g = np.linspace(-1, 1, n)
    pts = []
    for ax in range(3):
        for sgn in (-1, 1):
            for a in g:
                for b in g:
                    p = np.zeros(3)
                    p[ax] = sgn
                    o = [i for i in range(3) if i != ax]
                    p[o[0]], p[o[1]] = a, b
                    pts.append(p * h)
    return np.array(pts)


def min_gap(segs_a, segs_b, pose_b=(np.zeros(3), np.eye(3))):
    """Approximate least signed distance (m) from b's box surfaces (posed by (t, R) into a's frame) into a's boxes;
    negative = interpenetration."""
    t, R = pose_b
    best = 1.0
    boxes_a = [(np.asarray(c), _rotm(q), np.asarray(s) / 2) for c, s, q in segs_a]
    for c, s, q in segs_b:
        pts = (R @ (_rotm(q) @ _surface_pts(s).T + np.asarray(c)[:, None])).T + t
        for ca, Ra, ha in boxes_a:
            if np.min(np.linalg.norm(pts - ca, axis=1)) > 0.012:
                continue
            local = (pts - ca) @ Ra
            d = np.abs(local) - ha
            out = np.linalg.norm(np.maximum(d, 0), axis=1) + np.minimum(np.max(d, axis=1), 0)
            best = min(best, float(out.min()))
    return best


def cylinder_gap(segs_b, radius, z_lo, z_hi, pose_b=(np.zeros(3), np.eye(3))):
    """Approximate least signed distance (m) from b's box surfaces (posed into the cylinder's frame) to a solid
    cylinder on the z axis (the collar's core); negative = inside it."""
    t, R = pose_b
    best = 1.0
    for c, s, q in segs_b:
        pts = (R @ (_rotm(q) @ _surface_pts(s).T + np.asarray(c)[:, None])).T + t
        dr = np.hypot(pts[:, 0], pts[:, 1]) - radius
        dz = np.maximum(z_lo - pts[:, 2], pts[:, 2] - z_hi)
        d = np.where((dr > 0) | (dz > 0), np.hypot(np.maximum(dr, 0), np.maximum(dz, 0)), np.maximum(dr, dz))
        best = min(best, float(d.min()))
    return best
