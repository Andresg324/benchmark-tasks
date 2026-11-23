"""bedkit: a strip of paper joint tape bedded in wet joint compound by a taping knife, for a yam_arena task file.
Plain Python and NumPy (the simulator side, switching the bond joints on and off, is in the task file).

The simulator has no paste, so the compound is a scored model computed from object poses each control step:

- The compound is a height field on the wall's face (`Face`, cells `cell` square): `h` is the compound's depth over
  each cell (m). The task puts the starting bed on it.
- The tape is a row of rigid strips (sheetkit): the task passes each strip's frame and box every step. A strip "lies"
  when all four corners of its underside are within `Law.lie` of the face. The compound is not a collider: the tape
  lies on the face, and the depth of compound under it is the field under its footprint.
- The knife is a straight edge in its own frame (`Blade`), sampled at `n_pts` points on both faces of the blade.

The law (each number a `Law` field; the task states its reason):

1. Contact and force, not proximity. The task passes, each control step, the knife's contact force on each strip
   as the physics engine reports it (PhysX contact reporting: the normal force pushing the strip into the face, N).
   A strip is pressed only while the knife touches it (`touch_force` or more on that strip) AND the knife's total
   push on the tape is at least `press_force`, the force needed to squeeze the compound out from under the tape (the
   task states its derivation). A knife near the tape but not touching it, or touching it lightly, presses nothing.
   Where on a pushed strip the edge is (which cells it works) is found from the poses: an edge sample is over a
   strip when it is inside the strip's box or no more than `gap` above its outer face (whichever side faces out),
   over its footprint (`gap` only locates the edge on a strip the engine says it pushes; it never presses by
   itself); it touches the bare face (or the compound on it) when it is no more than `gap` in front of the face or up
   to `below` behind it. Only a HELD knife (a gripper's own shape touching its handle, decided by the task) whose edge
   SLIDES (its middle moved at least `slide_min` along the face over `slide_steps` control steps) works the compound:
   a knife pressed and held still, or a dab, does nothing; so do hands.
2. Each face cell is worked once per entry under the edge (moving off and coming back is a new entry); the knife's
   pose is interpolated between control steps in sub-steps of at most half a cell.
3. The blade's angle sets what it leaves, as in drywall finishing ("less angle lays on, more angle takes off"):
   film(theta) = `film_low` at theta <= `low_deg`, `film_high` at theta >= `high_deg`, linear between.
4. Bedding. A knife edge sliding over a lying strip presses the compound under it down to film(theta). The excess is
   squeezed out sideways: each cell's excess goes to the nearer edge of the tape and comes out just beyond it (a bead,
   drawn there). `squeezed[i]` keeps, per strip, the volume that came out at its -x and +x edges. Where there is no
   more compound under a cell than film(theta) (a starved, dry bed), nothing comes out.
5. Off the tape, the edge strikes the compound above its own height (or above film(theta), whichever is higher) off
   onto the blade. Compound is never laid by the knife here (the task does not score spreading).
6. Bond. A strip bonds (the task switches its bond joint on) when the knife has just pressed it while it lies on the
   face with between `bond_min` and `bond_max` of compound left under its middle. Less is a starved strip: it does not
   bond (a blister). An unpressed strip floats on the full bed and does not bond. A bonded strip lets go when its
   middle is lifted off the face past the joint's stretch (a peel), or when it has slid more than `slide_release`
   along the face from where it bonded (moving wet tape that far undoes the bedding; it needs a fresh pass).
7. Torn tape. A knife pushing on the tape (rule 1) whose edge is turned more than `gouge_tilt_deg` out of the face's
   plane presses with one corner;
   sliding that corner `gouge_len` along the tape tears wet paper tape (`torn`, latched for the trial).

What it does not model (say so in the task's text): the compound does not flow, slump or dry, and does not push back
on the knife or the tape (the push that squeezes it is a threshold on the knife's measured contact force, not a
pressure field); the tape lies on the wall's face with the compound's depth kept as numbers; above `press_force` the
film left is set by the blade's angle alone (a harder push does not leave less), and below it nothing is pressed (a
light pass that would leave thick compound is not modelled: it is simply not bedded); the direction of the stroke
does not change what a pass leaves.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np



# -- geometry helpers ---------------------------------------------------------------------------------------------------


def rotm(q) -> np.ndarray:
    """Rotation matrix from a quaternion (x, y, z, w)."""
    x, y, z, w = (float(c) for c in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def quat(r) -> np.ndarray:
    """Quaternion (x, y, z, w) from a rotation matrix."""
    r = np.asarray(r, float)
    t = float(np.trace(r))
    if t > 0:
        s = 2.0 * math.sqrt(t + 1.0)
        q = [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s, (r[1, 0] - r[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i + 1) % 3, (i + 2) % 3
        s = 2.0 * math.sqrt(1.0 + r[i, i] - r[j, j] - r[k, k])
        q = [0.0, 0.0, 0.0, (r[k, j] - r[j, k]) / s]
        q[i] = 0.25 * s
        q[j] = (r[j, i] + r[i, j]) / s
        q[k] = (r[k, i] + r[i, k]) / s
    q = np.array(q)
    return q if q[3] >= 0 else -q


def _nlerp(a, b, s: float) -> np.ndarray:
    a, b = np.asarray(a, float), np.asarray(b, float)
    if np.dot(a, b) < 0:
        b = -b
    q = (1 - s) * a + s * b
    return q / (np.linalg.norm(q) or 1.0)


def blade_angle_deg(rot, normal) -> float:
    """Angle between the blade's plane (knife x-y) and a face with unit `normal` (deg, 0..90)."""
    return math.degrees(math.acos(min(1.0, abs(float(np.dot(np.asarray(rot)[:, 2], normal))))))


def edge_tilt_deg(rot, normal) -> float:
    """How far the edge line (knife y) is turned out of a face's plane (deg, 0..90)."""
    return math.degrees(math.asin(min(1.0, abs(float(np.dot(np.asarray(rot)[:, 1], normal))))))


def edge_points(b: "Blade", n_pts: int) -> np.ndarray:
    """Edge samples in the knife frame: (2, n, 3), on the blade's two faces."""
    ys = np.linspace(-b.width / 2, b.width / 2, n_pts)
    return np.array([np.stack([np.full_like(ys, b.edge_x), ys, np.full_like(ys, b.z + dz)], axis=1)
                     for dz in (-b.thick / 2, b.thick / 2)])


@dataclass(frozen=True)
class Law:
    gap: float = 0.0006  # m above the strip's top / the face that still touches
    gap_hyst: float = 0.0004  # m extra while a stroke goes on
    below: float = 0.005  # m behind the face (a pressed collider sinking in)
    slide_min: float = 0.0015  # m the edge's middle must move along the face ...
    slide_steps: int = 3  # ... over this many control steps
    lift_steps: int = 3  # steps without contact that end a stroke
    low_deg: float = 40.0
    high_deg: float = 60.0
    film_low: float = 0.0008  # m left by a low blade
    film_high: float = 0.0001  # m left by a steep blade
    lie: float = 0.0015  # m: a strip's underside corners within this of the face: it lies on the face
    bond_min: float = 0.0004  # m of compound left under a pressed strip's middle for it to bond ...
    bond_max: float = 0.0016  # ... and no more than this
    bead_cells: int = 3  # cells beyond the tape's edge the squeezed bead is spread over
    gouge_tilt_deg: float = 15.0  # the edge turned this far out of the face's plane: only a corner presses ...
    gouge_len: float = 0.005  # ... and sliding this far along the tape tears it
    slide_release: float = 0.003  # m a bonded strip may slide along the face before its bond lets go
    press_force: float = 5.0  # N: the knife's total push on the tape (contact normal force) that squeezes compound
    touch_force: float = 0.05  # N: a strip with less contact force from the knife is not touched by it

    def film(self, theta_deg: float) -> float:
        if theta_deg <= self.low_deg:
            return self.film_low
        if theta_deg >= self.high_deg:
            return self.film_high
        s = (theta_deg - self.low_deg) / (self.high_deg - self.low_deg)
        return self.film_low + s * (self.film_high - self.film_low)


@dataclass(frozen=True)
class Blade:
    edge_x: float  # m: the edge line in the knife's frame (x along the blade to its edge)
    width: float  # m across (knife y)
    thick: float = 0.001
    z: float = 0.0  # m: the blade's mid-plane
    n_pts: int = 0  # samples along the edge; 0: every half cell (set by Bed)


@dataclass(frozen=True)
class Face:
    """The wall's face on a fixed object: a rectangle `size` = (lu, lv) from `origin` along unit axes `u`, `v` (object
    frame); outward normal u x v."""

    target: str
    origin: tuple
    u: tuple
    v: tuple
    size: tuple
    cell: float = 0.001


@dataclass(frozen=True)
class Strip:
    """One tape strip: its `key` in objects and its collider box (center, size) in that frame (x across the tape, y
    along it, z through it)."""

    key: str
    center: tuple
    size: tuple


def face_frame(face: Face, pose) -> tuple:
    p, q = pose
    r = rotm(q)
    o = np.asarray(p, float) + r @ np.asarray(face.origin, float)
    u, v = r @ np.asarray(face.u, float), r @ np.asarray(face.v, float)
    n = np.cross(u, v)
    return o, u, v, n / np.linalg.norm(n)


def strip_world(s: Strip, frame) -> tuple:
    """(center, R) of a strip's box in world from its body frame (pos, R)."""
    pos, R = frame
    R = np.asarray(R, float)
    return np.asarray(pos, float) + R @ np.asarray(s.center, float), R


def wall_side(R, n) -> float:
    """The local z sign (+1 or -1) of a strip's side that faces the wall (normal n out of it). Paper tape has no right
    side: laid by its folded-out end it lies with its bench-top side on the wall (+z into the wall), so the underside
    is whichever side faces the wall, never a fixed -z (read from -z, a flat flipped strip
    would read 1 mm off)."""
    return 1.0 if float(np.asarray(R, float)[:, 2] @ np.asarray(n, float)) < 0 else -1.0


def underside(s: Strip, frame, n) -> np.ndarray:
    """The four corners of the strip's underside (its side facing the wall whose normal is n), world, (4, 3)."""
    c, R = strip_world(s, frame)
    hx, hy, hz = (v / 2 for v in s.size)
    hz *= wall_side(R, n)
    return np.array([c + R @ np.array([sx * hx, sy * hy, hz]) for sx in (-1, 1) for sy in (-1, 1)])


class Bed:
    """One trial's compound and bond state for one tape on one face, worked by one knife."""

    def __init__(self, face: Face, strips, blade: Blade, law: Law = Law(), start=None):
        self.face, self.law, self.blade = face, law, blade
        self.strips = list(strips)
        self.nu, self.nv = round(face.size[0] / face.cell), round(face.size[1] / face.cell)
        uc = (np.arange(self.nu) + 0.5) * face.cell
        vc = (np.arange(self.nv) + 0.5) * face.cell
        self.U, self.V = np.meshgrid(uc, vc, indexing="ij")
        self.h = np.zeros((self.nu, self.nv)) if start is None else np.asarray(start(self.U, self.V), float).copy()
        self.h0 = self.h.copy()
        n = len(self.strips)
        self.bonded = np.zeros(n, bool)
        self.pressed = np.zeros(n, bool)  # the knife has passed over the strip while it lay on the face
        self.starved = np.zeros(n, bool)  # pressed with too little compound under it to bond
        self.squeezed = np.zeros((n, 2))  # m^3 squeezed out at the strip's -x and +x edges
        self.bond_at = np.zeros((n, 3))  # where each strip's underside middle was when it bonded
        self.torn = False  # latched: a knife corner dragged along the tape (it tears wet paper tape)
        self.corner_run = 0.0
        self.n = 0
        self.prev = None
        self.mids = []
        self.under = set()
        self.stroke = False
        self.miss = 0
        self.scraped = 0.0
        self.touch_tape = False
        self.touch_face = False
        self.push = 0.0  # N: the knife's total contact push on the tape at the last step
        n_pts = blade.n_pts or int(math.ceil(blade.width / (face.cell / 2))) + 1  # no cell between two samples
        self._edge = edge_points(blade, n_pts)

    @property
    def area(self) -> float:
        return self.face.cell ** 2

    # -- geometry -----------------------------------------------------------------------------------------------
    def cell_of(self, fr, p):
        """(iu, iv) of the face cell under world point p, or None off the face."""
        o, eu, ev, _ = fr
        u, v = float((p - o) @ eu), float((p - o) @ ev)
        if not (0 <= u < self.face.size[0] and 0 <= v < self.face.size[1]):
            return None
        return min(int(u / self.face.cell), self.nu - 1), min(int(v / self.face.cell), self.nv - 1)

    def cell_point(self, fr, iu, iv) -> np.ndarray:
        o, eu, ev, _ = fr
        return o + (iu + 0.5) * self.face.cell * eu + (iv + 0.5) * self.face.cell * ev

    def lies(self, fr, frames) -> np.ndarray:
        """Each strip lies on the face: all four underside corners within `lie` of it (and over it)."""
        o, eu, ev, n = fr
        out = np.zeros(len(self.strips), bool)
        for i, s in enumerate(self.strips):
            c = underside(s, frames[i], n)
            d = (c - o) @ n
            uu, vv = (c - o) @ eu, (c - o) @ ev
            out[i] = bool(np.all(d <= self.law.lie) and np.all(d >= -self.law.below)
                          and np.all((uu >= 0) & (uu < self.face.size[0]) & (vv >= 0) & (vv < self.face.size[1])))
        return out

    def lift(self, fr, frames, i: int) -> float:
        """How far strip i's underside middle is off the face (m)."""
        o, _, _, n = fr
        return float((self._mid(fr, frames, i) - o) @ n)

    def locate(self, frames, p, lying=None):
        """The strip whose footprint holds world point p (projected along the strip's own z): (i, local x, local y), or
        None. Only strips that lie on the face count when `lying` is given."""
        best = None
        for i, s in enumerate(self.strips):
            if lying is not None and not lying[i]:
                continue
            c, R = strip_world(s, frames[i])
            loc = R.T @ (np.asarray(p, float) - c)
            hx, hy = s.size[0] / 2, s.size[1] / 2
            if abs(loc[0]) <= hx and abs(loc[1]) <= hy + 0.0003:  # the hinge gaps (0.4 mm) count as covered
                if best is None or abs(loc[1]) < abs(best[2]):
                    best = (i, float(loc[0]), float(loc[1]))
        return best

    # -- the step -----------------------------------------------------------------------------------------------
    def step(self, knife_pos, knife_quat, held: bool, face_pose, frames, force) -> list:
        """One control step. `frames`: each strip's body frame (pos, R) now; `force`: the knife's contact normal force
        on each strip (N, pushing it into the face) at this step, from the physics engine. Returns the strips to bond
        now."""
        L = self.law
        force = np.asarray(force, float).reshape(-1)
        if force.shape != (len(self.strips),) or not np.all(np.isfinite(force)):
            raise ValueError(f"bedkit: need the knife's contact force on each of {len(self.strips)} strips, got "
                             f"{force.shape}")
        touched = force >= L.touch_force
        self.push = float(force[touched].sum())
        pushing = touched & (self.push >= L.press_force)
        self.n += 1
        fr = face_frame(self.face, face_pose)
        o, eu, ev, nrm = fr
        pos, q = np.asarray(knife_pos, float), np.asarray(knife_quat, float)
        r_now = rotm(q)
        mid_local = np.array([self.blade.edge_x, 0.0, self.blade.z])
        mid = pos + r_now @ mid_local
        self.mids = (self.mids + [mid])[-(L.slide_steps + 1):]
        sliding = False
        if len(self.mids) > L.slide_steps:
            dsp = self.mids[-1] - self.mids[0]
            sliding = float(np.linalg.norm(dsp - float(dsp @ nrm) * nrm)) >= L.slide_min
        prev = self.prev if self.prev is not None else (pos, q)
        self.prev = (pos, q)
        prev_mid = prev[0] + rotm(prev[1]) @ mid_local
        span = float(np.linalg.norm(mid - prev_mid))
        rot_span = math.acos(max(-1.0, min(1.0, (float(np.trace(rotm(prev[1]).T @ r_now)) - 1) / 2)))
        reach = self.blade.width / 2 + abs(self.blade.edge_x)
        n_sub = int(min(200, max(1, math.ceil(max(span, rot_span * reach) / (self.face.cell / 2)))))
        lying = self.lies(fr, frames)
        boxes = [strip_world(s, frames[i]) for i, s in enumerate(self.strips)]
        theta = blade_angle_deg(r_now, nrm)
        film = L.film(theta)
        pressed_now = set()
        self.touch_tape = self.touch_face = False
        gap = L.gap + (L.gap_hyst if self.stroke else 0.0)
        near = abs(float((mid - o) @ nrm)) < 0.05 or abs(float((prev_mid - o) @ nrm)) < 0.05
        live = np.flatnonzero(lying)
        C = np.array([boxes[i][0] for i in live]).reshape(-1, 3)
        RS = np.array([boxes[i][1] for i in live]).reshape(-1, 3, 3)
        HS = np.array([self.strips[i].size for i in live]).reshape(-1, 3) / 2
        for k in range(1, (n_sub if near else 0) + 1):
            sk = k / n_sub
            p_k = prev[0] + sk * (pos - prev[0])
            r_k = r_now if n_sub == 1 else rotm(_nlerp(prev[1], q, sk))
            both = self._edge @ r_k.T + p_k  # (2 faces, n, 3)
            hb = (both - o) @ nrm
            pts = both[np.argmin(hb, axis=0), np.arange(both.shape[1])]  # per column the face nearer the wall
            hs_all = (pts - o) @ nrm
            uu, vv = (pts - o) @ eu, (pts - o) @ ev
            onf = (uu >= 0) & (uu < self.face.size[0]) & (vv >= 0) & (vv < self.face.size[1])
            iu = np.clip((uu / self.face.cell).astype(int), 0, self.nu - 1)
            iv = np.clip((vv / self.face.cell).astype(int), 0, self.nv - 1)
            # every sample in every lying strip's frame at once: (strips, samples, 3)
            loc = np.einsum("smj,sjk->smk", pts[None, :, :] - C[:, None, :], RS) if len(live) else np.zeros((0, len(pts), 3))
            inside = ((np.abs(loc[..., 0]) <= HS[:, None, 0]) & (np.abs(loc[..., 1]) <= HS[:, None, 1] + 0.0003)
                      & (np.abs(loc[..., 2]) <= HS[:, None, 2] + gap))  # either side may face out
            any_in = inside.any(axis=0) if len(live) else np.zeros(len(pts), bool)
            first = np.argmax(inside, axis=0) if len(live) else np.zeros(len(pts), int)
            hcell = self.h[iu, iv]
            face_hit = ~any_in & (hs_all >= -L.below) & (hs_all <= np.maximum(gap, hcell))
            if np.any(any_in & onf):
                self.touch_tape = True
            if np.any(face_hit & onf & (hs_all <= gap)):
                self.touch_face = True
            now = set()
            work = []
            for m in np.flatnonzero(onf & (any_in | face_hit)):
                cell = (int(iu[m]), int(iv[m]))
                if any_in[m]:
                    j = int(first[m])
                    if not pushing[int(live[j])]:
                        continue  # over the strip, but the knife does not touch it or push hard enough: no work
                    key = ("t", int(live[j]), cell)
                    work.append((key, (int(live[j]), loc[j, m]), float(hs_all[m])))
                else:
                    key = ("f", -1, cell)
                    work.append((key, None, float(hs_all[m])))
                now.add(key)
            if not (held and sliding):
                continue  # cells under a resting or unheld edge stay unworked until it slides over them
            new = now - self.under
            self.under = now
            if any(k[0] == "t" for k in now) and edge_tilt_deg(r_k, nrm) > L.gouge_tilt_deg:
                self.corner_run += span / n_sub
                if self.corner_run >= L.gouge_len:
                    self.torn = True
            if not new:
                continue
            for key, hit, hs in work:
                if key not in new:
                    continue
                new.discard(key)
                kind, i, cell = key
                if kind == "t":
                    pressed_now.add(i)
                    ex = float(self.h[cell]) - film
                    if ex > 0:
                        self.h[cell] = film
                        self._bead(fr, i, boxes[i], hit[1], ex * self.area)
                else:
                    level = max(film, hs)
                    ex = float(self.h[cell]) - level
                    if ex > 0:
                        self.h[cell] = level
                        self.scraped += ex * self.area
        if self.touch_tape or self.touch_face:
            self.stroke, self.miss = True, 0
        elif self.stroke:
            self.miss += 1
            if self.miss > L.lift_steps:
                self.stroke, self.under, self.corner_run = False, set(), 0.0
        bond = []
        for i in sorted(pressed_now):
            self.pressed[i] = True
            c, R = boxes[i]
            mid_cell = self.cell_of(fr, self._mid(fr, frames, i))
            d = float(self.h[mid_cell]) if mid_cell is not None else 0.0
            if d < L.bond_min:
                self.starved[i] = True
            elif d <= L.bond_max and not self.bonded[i]:
                bond.append(i)
        return bond

    def _bead(self, fr, i, box, loc, vol) -> None:
        """Squeeze `vol` out from under strip i at the edge nearer `loc` (strip-local point), as a bead just beyond it."""
        c, R = box
        s = self.strips[i].size
        side = 1 if loc[0] >= 0 else 0
        self.squeezed[i, side] += vol
        sgn = 1.0 if side else -1.0
        n = self.law.bead_cells
        for k in range(n):
            p = c + R @ np.array([sgn * (s[0] / 2 + (k + 1.0) * self.face.cell), loc[1],
                                  wall_side(R, fr[3]) * s[2] / 2])
            cell = self.cell_of(fr, p)
            if cell is not None:
                self.h[cell] += vol / n / self.area

    # -- bonds ----------------------------------------------------------------------------------------------------
    def set_bonded(self, i: int, on: bool, fr=None, frames=None) -> None:
        self.bonded[i] = bool(on)
        if on and fr is not None:
            self.bond_at[i] = self._mid(fr, frames, i)

    def _mid(self, fr, frames, i: int) -> np.ndarray:
        c, R = strip_world(self.strips[i], frames[i])
        return c + R @ np.array([0.0, 0.0, wall_side(R, fr[3]) * self.strips[i].size[2] / 2])

    def release(self, fr, frames, stretch: float) -> list:
        """Bonded strips that let go: lifted off the face past `stretch` (a peel: the joint's spring then carries its
        strength), or slid along the face more than `slide_release` from where they bonded (moving wet-bedded tape
        that far undoes the bedding: the strip needs a fresh pass)."""
        out = []
        n = fr[3]
        for i in np.flatnonzero(self.bonded):
            i = int(i)
            dm = self._mid(fr, frames, i) - self.bond_at[i]
            slid = float(np.linalg.norm(dm - float(dm @ n) * n))
            if self.lift(fr, frames, i) > stretch or slid > self.law.slide_release:
                out.append(i)
                if slid > self.law.slide_release:
                    self.pressed[i] = False
                    self.squeezed[i] = 0.0
        return out

    def depth_under(self, fr, frames, i: int, p=None) -> float:
        """Compound under strip i at world point p (default: the strip's middle), m."""
        if p is None:
            c, R = strip_world(self.strips[i], frames[i])
            p = c
        cell = self.cell_of(fr, np.asarray(p, float))
        return float(self.h[cell]) if cell is not None else 0.0

    def blocks(self, fr, frames, k: int = 3, show: float = 0.0002) -> list:
        """Drawable blocks of k x k cells of compound NOT under a lying strip and deeper than `show`, changed from the
        start (the starting bed is drawn once, by the task): [(u, v, su, sv)]."""
        c = self.face.cell
        nu, nv = self.nu, self.nv
        diff = (self.h > show) & (np.abs(self.h - self.h0) > 1e-6)
        if not diff.any():
            return []
        out = []
        bi, bj = np.nonzero(diff)
        seen = set()
        for a, b in zip(bi // k, bj // k):
            if (a, b) in seen:
                continue
            seen.add((a, b))
            su, sv = min(k, nu - a * k) * c, min(k, nv - b * k) * c
            out.append((a * k * c + su / 2, b * k * c + sv / 2, su, sv))
        return out
