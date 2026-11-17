"""liquidkit: a lumped, documented liquid model for YAM tasks (no particles).

Each rigid vessel (pot, bowl, ladle, cup, ice tray cell) carries a volume of liquid. Every control step the kit
works out, from the vessel's pose and motion:
  - where the free surface is (it stays normal to the effective gravity, g minus the vessel's acceleration, and
    lags it like the vessel's first slosh mode: a damped oscillator at the mode's natural frequency);
  - how much the vessel can hold at that attitude (the liquid below the lowest rim point);
  - how fast liquid leaves over the lowest rim point or a spout (broad-crested weir) or through a hole (orifice);
  - where that stream lands (straight down from the lip, with the lip's own velocity), and which receiving vessel
    catches it; what no vessel catches is counted as spilled, with its landing point.
It does not draw anything or touch the simulator: a task reads `surface_pose()` / `level()` to place a visual liquid
surface and calls `step()` from a yamkit step hook with each vessel's world pose.

Sources for the formulas (cite them in a task's fidelity text):
  - weir: Q = Cd * (2/3) * sqrt(2 g) * b * H^1.5, Cd ~ 0.6 (broad-crested/sharp lip; e.g. White, Fluid Mechanics,
    ch. 10 "weirs");
  - orifice: Q = Cd * A * sqrt(2 g h), Cd ~ 0.61 (Torricelli with vena contracta; White ch. 6/10);
  - first slosh mode of an upright circular tank: omega^2 = (g * 1.841 / R) * tanh(1.841 * h / R) (Ibrahim,
    Liquid Sloshing Dynamics, 2005, ch. 2); a rectangular tank uses pi/L in place of 1.841/R. Damping ratio of
    water in small smooth tanks ~0.005-0.05 (same source); the default 0.05 is the upper end, for a calmer model.
Units: SI (m, m^3, s). A vessel's local frame: z up its axis, origin at the inside bottom centre.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

G = 9.81
WEIR_CD = 0.6
ORIFICE_CD = 0.61


def _rot(q):
    """3x3 rotation matrix from a quaternion (x, y, z, w)."""
    x, y, z, w = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def _quat(R) -> tuple:
    """Rotation matrix to a quaternion (x, y, z, w)."""
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (x, y, z, w)


@dataclass
class Shape:
    """The inside of a vessel as a stack of horizontal slices (local frame). `radius(z)` for round vessels, or
    `half_xy(z)` -> (half-length x, half-width y) for rectangular ones. Sampled once into volume points."""
    height: float
    radius: object = None          # callable z -> r, or a number (cylinder)
    half_xy: object = None         # callable z -> (hx, hy), or a pair (box)
    n: int = 24                    # samples per axis (n^3 grid; 24 -> ~0.1 % volume error on a cylinder)

    def points(self):
        """Grid points inside the vessel and the volume each stands for."""
        zs = (np.arange(self.n) + 0.5) / self.n * self.height
        dz = self.height / self.n
        pts, w = [], []
        for z in zs:
            if self.radius is not None:
                r = self.radius(z) if callable(self.radius) else float(self.radius)
                u = (np.arange(self.n) + 0.5) / self.n * 2 - 1
                gx, gy = np.meshgrid(u * r, u * r)
                m = gx ** 2 + gy ** 2 <= r * r
                cell = (2 * r / self.n) ** 2 * dz
            else:
                hx, hy = self.half_xy(z) if callable(self.half_xy) else self.half_xy
                ux = (np.arange(self.n) + 0.5) / self.n * 2 - 1
                gx, gy = np.meshgrid(ux * hx, ux * hy)
                m = np.ones_like(gx, bool)
                cell = (2 * hx / self.n) * (2 * hy / self.n) * dz
            pts.append(np.stack([gx[m], gy[m], np.full(m.sum(), z)], 1))
            w.append(np.full(m.sum(), cell))
        return np.concatenate(pts), np.concatenate(w)

    def rim(self, k: int = 72):
        """Points on the rim (top edge), local frame."""
        a = np.linspace(0, 2 * math.pi, k, endpoint=False)
        if self.radius is not None:
            r = self.radius(self.height) if callable(self.radius) else float(self.radius)
            return np.stack([r * np.cos(a), r * np.sin(a), np.full(k, self.height)], 1)
        hx, hy = self.half_xy(self.height) if callable(self.half_xy) else self.half_xy
        s = np.stack([np.cos(a), np.sin(a)], 1)
        s = s / np.max(np.abs(s) / np.array([hx, hy]), axis=1, keepdims=True)
        return np.stack([s[:, 0], s[:, 1], np.full(k, self.height)], 1)

    def mode_omega(self, depth: float) -> float:
        """First slosh mode's angular frequency at this fill depth (upright)."""
        depth = max(depth, 1e-4)
        if self.radius is not None:
            r = self.radius(depth) if callable(self.radius) else float(self.radius)
            k = 1.841 / r
        else:
            hx, _ = self.half_xy(depth) if callable(self.half_xy) else self.half_xy
            k = math.pi / (2 * hx)
        return math.sqrt(G * k * math.tanh(k * depth))


@dataclass
class Vessel:
    name: str
    shape: Shape
    volume: float = 0.0                  # m^3 of liquid now
    lip_width: float = 0.02              # m: width of the stream over the rim (or the spout's width)
    spout: object = None                 # optional local point (x, y, z): liquid leaves here only (a spout/lip)
    hole: object = None                  # optional (local point, area m^2): an orifice (drain, nozzle)
    damping: float = 0.05                # slosh damping ratio
    jump_accel: float = 50.0             # m/s^2 (~5 g): above this a pose change is a jump (teleport), not a motion;
                                         # a held vessel never accelerates this hard (a sharp robot start is ~5 m/s^2)
    _pts: np.ndarray = field(default=None, repr=False)
    _w: np.ndarray = field(default=None, repr=False)
    _rim: np.ndarray = field(default=None, repr=False)
    _n: np.ndarray = field(default=None, repr=False)      # surface normal (world), slosh state
    _ndot: np.ndarray = field(default=None, repr=False)
    _pose: tuple = field(default=None, repr=False)
    _v: np.ndarray = field(default=None, repr=False)
    out_rate: float = 0.0                # m^3/s leaving last step
    out_point: np.ndarray = None         # world point it left from

    def __post_init__(self):
        self._pts, self._w = self.shape.points()
        self._rim = self.shape.rim() if self.spout is None else np.array([self.spout], float)
        self._n = np.array([0.0, 0.0, 1.0])
        self._ndot = np.zeros(3)

    @property
    def capacity(self) -> float:
        return float(self._w.sum())

    # --- geometry at a pose -------------------------------------------------------------------------------------
    def _world(self, pos, quat):
        R = _rot(quat)
        return R, np.asarray(pos, float)

    def held_at(self, pos, quat, normal=None) -> float:
        """Most liquid the vessel keeps at this pose: the volume below its lowest rim (or spout) point, measured
        along the surface normal."""
        R, p = self._world(pos, quat)
        n = self._n if normal is None else normal
        hp = (self._pts @ R.T + p) @ n
        lip = ((self._rim @ R.T + p) @ n).min()
        return float(self._w[hp <= lip].sum())

    def level(self, pos, quat) -> float:
        """Height of the free surface along its normal (world), i.e. the height of the volume-quantile point."""
        R, p = self._world(pos, quat)
        hp = (self._pts @ R.T + p) @ self._n
        if self.volume <= 0:
            return float(hp.min())
        o = np.argsort(hp)
        c = np.cumsum(self._w[o])
        i = min(int(np.searchsorted(c, self.volume)), len(o) - 1)
        return float(hp[o][i])

    def surface_pose(self, pos, quat):
        """(centre point, normal) of the free surface in world coordinates, for drawing it (None if empty)."""
        if self.volume <= 0:
            return None
        R, p = self._world(pos, quat)
        h = self.level(pos, quat)
        wp = self._pts @ R.T + p
        near = np.abs(wp @ self._n - h) < (self.shape.height / self.shape.n)
        c = wp[near].mean(0) if near.any() else wp.mean(0)
        return c, self._n.copy()

    def inside_local(self, local_pts, tol: float = 0.0) -> np.ndarray:
        """Which local points lie inside the vessel's hollow (walls shrunk by `tol`)."""
        p = np.atleast_2d(np.asarray(local_pts, float))
        z = p[:, 2]
        ok = (z >= -1e-9) & (z <= self.shape.height + 1e-9)
        zc = np.clip(z, 0.0, self.shape.height)
        if self.shape.radius is not None:
            r = np.array([self.shape.radius(v) for v in zc]) if callable(self.shape.radius) else \
                np.full(len(zc), float(self.shape.radius))
            return ok & (np.hypot(p[:, 0], p[:, 1]) <= r - tol)
        hxy = np.array([self.shape.half_xy(v) for v in zc]) if callable(self.shape.half_xy) else \
            np.tile(np.asarray(self.shape.half_xy, float), (len(zc), 1))
        return ok & (np.abs(p[:, 0]) <= hxy[:, 0] - tol) & (np.abs(p[:, 1]) <= hxy[:, 1] - tol)

    def surface_disc(self, pos, quat, k: int = 32):
        """Drawing hook: an ellipse lying in the free surface and kept inside the vessel, as (centre, quat xyzw,
        (a, b)) with the ellipse's axes along the quat's local x and y and its plane normal to local z; None if empty.
        Draw it as a flat cylinder of radius 1 scaled (a, b, thickness). The ellipse is fitted to the wetted section
        (the volume points at the surface height) and shrunk until its outline is inside the walls and above the
        floor, so a pool in a tipped pot's corner is drawn as a pool, not a disc through the wall."""
        if self.volume <= 0:
            return None
        R, p = self._world(pos, quat)
        n = self._n / np.linalg.norm(self._n)
        h = self.level(pos, quat)
        wp = self._pts @ R.T + p
        band = self.shape.height / self.shape.n
        near = np.abs(wp @ n - h) < band
        sec = wp[near] if near.sum() >= 3 else wp[np.argsort(np.abs(wp @ n - h))[:3]]
        c = sec.mean(0)
        c = c - (c @ n - h) * n                       # on the surface plane
        # in-plane axes: principal directions of the wetted section
        ref = np.array([1.0, 0.0, 0.0]) if abs(n[0]) < 0.9 else np.array([0.0, 1.0, 0.0])
        e1 = np.cross(n, np.cross(ref, n)); e1 /= np.linalg.norm(e1)
        e2 = np.cross(n, e1)
        uv = np.stack([(sec - c) @ e1, (sec - c) @ e2], 1)
        if len(uv) >= 3:
            w, V = np.linalg.eigh(np.cov(uv.T) + 1e-12 * np.eye(2))
            d = V[:, 1]
            e1, e2 = d[0] * e1 + d[1] * e2, -d[1] * e1 + d[0] * e2
            uv = np.stack([(sec - c) @ e1, (sec - c) @ e2], 1)
        lo, hi = uv.min(0), uv.max(0)
        mid = (lo + hi) / 2
        c = c + mid[0] * e1 + mid[1] * e2
        ab = np.maximum((hi - lo) / 2 + band / 2, 1e-4)
        a = np.linspace(0, 2 * math.pi, k, endpoint=False)
        for _ in range(60):
            ring = c + np.outer(ab[0] * np.cos(a), e1) + np.outer(ab[1] * np.sin(a), e2)
            if self.inside_local((ring - p) @ R).all():
                break
            ab = ab * 0.95
        Rm = np.stack([e1, e2, n], 1)
        if np.linalg.det(Rm) < 0:
            Rm[:, 1] *= -1
        return c, _quat(Rm), (float(ab[0]), float(ab[1]))

    # --- dynamics -----------------------------------------------------------------------------------------------
    def _slosh(self, pos, dt):
        """Update the surface normal: it follows the effective gravity (g - a) with the first mode's lag."""
        pos = np.asarray(pos, float)
        if self._pose is None:
            self._pose, self._v = pos, np.zeros(3)
            return
        v = (pos - self._pose) / dt
        a = (v - self._v) / dt
        if np.linalg.norm(a) > self.jump_accel:
            # a pose jump (a reset, a respawn, a solver pop), not a motion: restart the motion state, no slosh kick
            self._pose, self._v = pos, np.zeros(3)
            return
        self._pose, self._v = pos, v
        geff = np.array([0.0, 0.0, G]) + a          # "up" is opposite (gravity - acceleration)
        target = geff / max(np.linalg.norm(geff), 1e-9)
        R0 = self.shape.height
        depth = max(self.volume / max(self.capacity, 1e-12) * R0, 1e-3)
        w = self.shape.mode_omega(depth)
        acc = w * w * (target - self._n) - 2 * self.damping * w * self._ndot
        self._ndot = self._ndot + acc * dt
        n = self._n + self._ndot * dt
        self._n = n / np.linalg.norm(n)

    def step(self, pos, quat, dt: float):
        """Advance one control step at this world pose. Returns the volume (m^3) that left this step and the world
        point it left from (or (0, None))."""
        self._slosh(pos, dt)
        self.out_rate, self.out_point = 0.0, None
        if self.volume <= 0:
            return 0.0, None
        R, p = self._world(pos, quat)
        left = 0.0
        # over the rim / spout: a weir with head = how far the surface stands above the lowest lip point
        rim_w = self._rim @ R.T + p
        heights = rim_w @ self._n
        i = int(np.argmin(heights))
        head = self.level(pos, quat) - heights[i]
        if head > 0:
            q = WEIR_CD * (2.0 / 3.0) * math.sqrt(2 * G) * self.lip_width * head ** 1.5
            excess = self.volume - self.held_at(pos, quat)
            left = min(excess, q * dt) if excess > 0 else 0.0
            if left > 0:
                self.out_point = rim_w[i]
        # through a hole
        if self.hole is not None:
            hp_local, area = self.hole
            hp = np.asarray(hp_local, float) @ R.T + p
            hh = self.level(pos, quat) - hp @ self._n
            if hh > 0:
                qh = min(ORIFICE_CD * area * math.sqrt(2 * G * hh) * dt, self.volume - left)
                left += qh
                self.out_point = hp if self.out_point is None else self.out_point
        left = min(left, self.volume)
        self.volume -= left
        self.out_rate = left / dt
        return left, self.out_point


@dataclass
class World:
    """Vessels plus where streams land. `opening(name)` -> (centre world point, radius) of a receiver's mouth;
    filled by the task each step (from the receiver's pose). Spills are logged with their landing points."""
    vessels: dict
    table_z: float = 0.0
    spilled: float = 0.0
    spill_log: list = field(default_factory=list)

    def total(self) -> float:
        """All the liquid in the scene (every vessel plus what was spilled): constant over a trial."""
        return float(sum(v.volume for v in self.vessels.values()) + self.spilled)

    def step(self, poses: dict, dt: float, openings: dict):
        """poses: name -> (pos, quat) for every vessel. openings: name -> (centre, radius) for vessels that can
        catch a stream (their rim's centre and inner radius in world)."""
        for name, v in self.vessels.items():
            pos, quat = poses[name]
            out, pt = v.step(pos, quat, dt)
            if out <= 0:
                continue
            # the stream falls from the lip straight down, carried by the lip's own velocity
            vel = v._v if v._v is not None else np.zeros(3)
            caught = None
            for rname, (c, r) in openings.items():
                if rname == name or c[2] >= pt[2]:
                    continue
                t = math.sqrt(max(2 * (pt[2] - c[2]) / G, 0.0))
                land = pt[:2] + vel[:2] * t
                if np.linalg.norm(land - np.asarray(c[:2])) <= r:
                    caught = rname
                    break
            if caught is not None:
                rv = self.vessels[caught]
                room = rv.capacity - rv.volume
                rv.volume += min(out, room)
                out -= min(out, room)
            if out > 0:
                t = math.sqrt(max(2 * (pt[2] - self.table_z) / G, 0.0))
                land = pt[:2] + vel[:2] * t
                self.spilled += out
                self.spill_log.append((float(out), (float(land[0]), float(land[1]))))


def dip(ladle: Vessel, ladle_pose, pot: Vessel, pot_pose, dt: float, fill_time: float = 0.5) -> float:
    """A ladle (or scoop) whose bowl is below the pot's surface fills toward what it can hold at its attitude,
    over ~fill_time seconds. Returns the volume moved. The caller checks the bowl is inside the pot."""
    lpos, lq = ladle_pose
    R, p = ladle._world(lpos, lq)
    bowl_top = ((ladle._rim @ R.T + p) @ pot._n).min()
    if pot.volume <= 0 or bowl_top >= pot.level(*pot_pose):
        return 0.0
    room = max(ladle.held_at(lpos, lq, normal=pot._n) - ladle.volume, 0.0)
    move = min(room, pot.volume, room * dt / max(fill_time, dt))
    ladle.volume += move
    pot.volume -= move
    return move


def cylinder(radius: float, height: float, n: int = 24) -> Shape:
    return Shape(height=height, radius=radius, n=n)


def bowl(r_bottom: float, r_top: float, height: float, n: int = 24) -> Shape:
    """A frustum (straight-sided bowl, ladle, cup)."""
    return Shape(height=height, radius=lambda z: r_bottom + (r_top - r_bottom) * z / height, n=n)


def box(lx: float, ly: float, height: float, n: int = 24) -> Shape:
    return Shape(height=height, half_xy=(lx / 2, ly / 2), n=n)
