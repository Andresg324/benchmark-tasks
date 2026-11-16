"""carrierkit: the fuse clips inside a hinged fuse carrier, and the carrier's latch, as laws. Import after yamkit:
    import carrierkit as ck

A modular fuse holder's carrier is a hinged drawer: the fuse lies in it between two end blocks, each ferrule held by a
contact clip whose fingers reach to the ferrule's widest point. The carrier's walls, clip fingers and end blocks are
real colliders (task.py); what they cannot do is spring. This module adds two things:

- `Seat`: while a ferrule sits in its clip, the clip presses it toward the carrier's floor and the clip's middle
  (its fingers centre it) with a set force (`Seat.hold`), applied at the ferrule's centre before every physics
  step, and the equal and opposite force on the carrier at the same point. A ferrule drawn out of its clip past
  the fingers' lips (`Seat.release` out from its seat), or pushed sideways or along the axis off the clip, is
  free. A free ferrule pushed back down between the clip's fingers is held again. Any fuse can be caught this way
  (the old one, a spare, a wrong one): which fuse sits in the carrier is read from the poses alone (`seat_state`).
  Nothing attaches a fuse to the carrier or to a hand.
- `Snap`: the carrier's latch and open stop as a yamkit link law. Within `zone` of shut a spring pulls the carrier
  shut; within `detent` of its open stop a spring holds it there; between them it swings free (gravity and whatever
  pushes it).

The force is constant while it acts (no stiffness), so it cannot ring on a 9 g fuse at the simulator's 240 Hz.
`seat_state`, `offsets` and `Holds` are pure NumPy; the scorer and the offline tests use them on poses.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass

import numpy as np
import yamkit as kit


@dataclass(frozen=True)
class Seat:
    """The two clips of one carrier, in the carrier link's own frame (origin at its hinge). `mech`/`link`: the
    mechanism and link names. `seat`: the fuse's centre seated. `out`: unit vector out of the clips, from the floor
    toward the open side. `axis`: the fuse's axis seated (unit). `ends`: the two ferrule centres along the fuse's own z
    axis (m). `hold`: the force (N) each clip presses its ferrule toward the floor with. `release`: how far out of its
    seat (m) a ferrule's centre is past the clip fingers' lips. `side`/`along`: how far (m) a ferrule's centre can go
    sideways or along the axis before it is off its clip. `reseat`: within this (m) of the seat a free ferrule is
    caught again."""

    mech: str
    link: str
    seat: tuple
    out: tuple = (1.0, 0.0, 0.0)
    axis: tuple = (0.0, 0.0, 1.0)
    ends: tuple = (-0.0143, 0.0143)
    hold: float = 0.8
    release: float = 0.0025
    side: float = 0.003
    along: float = 0.005
    reseat: float = 0.0006

    @property
    def key(self) -> str:
        return f"{self.mech}/{self.link}"


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def offsets(fuse_pos, fuse_quat, car_pos, car_quat, seat: Seat):
    """Per ferrule: (out, side, along) of its centre from its seat, in the carrier's frame (m). `out` is along
    seat.out (positive = lifted toward the open side), `side` across the clip, `along` along the seated axis."""
    Rc, Rf = _rot(car_quat), _rot(fuse_quat)
    o, a = np.asarray(seat.out, float), np.asarray(seat.axis, float)
    s = np.cross(a, o)
    # a fuse is the same either way round: a fuse lying end for end has each ferrule measured against the other clip
    flip = 1.0 if float((Rc.T @ Rf[:, 2]) @ a) >= 0.0 else -1.0
    res = []
    for e in seat.ends:
        p = Rc.T @ (np.asarray(fuse_pos, float) + Rf @ np.array([0.0, 0.0, e]) - np.asarray(car_pos, float))
        d = p - (np.asarray(seat.seat, float) + flip * e * a)
        res.append((float(d @ o), float(abs(d @ s)), float(d @ a)))
    return res


def off_clip(offset, seat: Seat) -> bool:
    """A ferrule out of its clip: lifted past the lips, or off it sideways or along the axis."""
    out, side, along = offset
    return out > seat.release or side > seat.side or abs(along) > seat.along


def seated(offset, seat: Seat, tol: float | None = None) -> bool:
    """A ferrule down in its clip, within `tol` (default `reseat`) of the floor."""
    out, side, along = offset
    t = seat.reseat if tol is None else tol
    return abs(out) < t and side < seat.side / 3 and abs(along) < seat.along / 3


def caught(offset, seat: Seat) -> bool:
    """A free ferrule pushed down into its clip, between its fingers: the clip takes hold of it (within 1 mm of the
    floor, 2 mm of the middle)."""
    out, side, along = offset
    return abs(out) < 0.001 and side < 0.002 and abs(along) < seat.along / 3


def seat_state(fuse_pos, fuse_quat, car_pos, car_quat, seat: Seat):
    """(out_of_clip, offsets) for the fuse's two ferrules, from the poses alone."""
    offs = offsets(fuse_pos, fuse_quat, car_pos, car_quat, seat)
    return [off_clip(d, seat) for d in offs], offs


class Holds:
    """Which ferrules of which fuses the carrier's clips hold, step by step: those in `start` held at the start; let go
    once off the clip; caught again (any fuse) once pushed back down between its fingers."""

    def __init__(self, seat: Seat, fuses, start=()):
        self.seat, self.fuses, self.start = seat, tuple(fuses), tuple(start)
        self.reset()

    def reset(self):
        self.held = {n: [n in self.start] * 2 for n in self.fuses}

    def update(self, poses, car_pose):
        """poses: {fuse: (pos, quat_xyzw)}; car_pose: (pos, quat_xyzw). Returns {fuse: [(point, force), ...]}: the
        world point (each held ferrule's centre) and the force on the fuse there; the carrier gets the opposite."""
        cp, cq = car_pose
        Rc = _rot(cq)
        o = np.asarray(self.seat.out, float)
        across = np.cross(np.asarray(self.seat.axis, float), o)
        out = {}
        for n in self.fuses:
            if n not in poses:
                continue
            pos, quat = poses[n]
            offs = offsets(pos, quat, cp, cq, self.seat)
            Rf = _rot(quat)
            push = []
            for i, (d, e) in enumerate(zip(offs, self.seat.ends)):
                if self.held[n][i] and off_clip(d, self.seat):
                    self.held[n][i] = False
                elif not self.held[n][i] and caught(d, self.seat):
                    self.held[n][i] = True
                if self.held[n][i]:
                    # the clip's fingers press the ferrule toward the floor and toward their middle (they centre it)
                    p = np.asarray(pos, float) + Rf @ np.array([0.0, 0.0, e])
                    side = float((Rc.T @ (p - np.asarray(cp, float)) - np.asarray(self.seat.seat, float)) @ across)
                    v = -(max(d[0], 0.0005) * o + side * across)
                    push.append((p, self.seat.hold * (Rc @ (v / np.linalg.norm(v)))))
            out[n] = push
        return out


@dataclass(frozen=True)
class Snap:
    """A yamkit link law for a carrier hinge whose joint is 0 shut and negative open (to `-open`, its stop). Within
    `zone` (rad) of shut a spring of `k_latch` (N*m/rad) pulls it shut; within `detent` of the open stop one of
    `k_open` holds it open; in between it swings free. `damp` (N*m*s/rad) is the hinge's damping."""

    open: float
    zone: float
    detent: float
    k_latch: float
    k_open: float
    damp: float = 0.002

    def drive(self, key, q, qd, J, mem):
        a = -q
        if a < self.zone:
            return 0.0, self.k_latch, 0.0, self.damp
        if a > self.open - self.detent:
            return -self.open, self.k_open, 0.0, self.damp
        return q, 0.0, 0.0, self.damp


_SEATS: list = []
_RT: dict = {}


def register(seat: Seat, fuses, start=()) -> None:
    """The carrier whose clips act, the fuses they can hold, and those held at the start (task.py calls this once)."""
    _SEATS[:] = [(seat, tuple(fuses), tuple(start))]
    _RT.pop("holds", None)


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x, dtype=np.float64)


class _Runtime:
    def __init__(self, scene):
        seat, fuses, start = _SEATS[0]
        self.seat = seat
        self.assets = {n: scene[n] for n in fuses}
        self.art = scene[seat.mech]
        self.body = list(self.art.body_names).index(seat.link)
        self.origin = _arr(scene.env_origins)[0]
        _RT.setdefault("holds", Holds(seat, fuses, start))

    def physics_step(self) -> None:
        import torch

        d = self.art.data
        cp = _arr(d.body_pos_w)[0, self.body] - self.origin
        cq = _arr(d.body_quat_w)[0, self.body]
        ccom = _arr(getattr(d, "body_com_pos_w", d.body_pos_w))[0, self.body] - self.origin
        poses, coms = {}, {}
        for name, asset in self.assets.items():
            a = asset.data
            poses[name] = (_arr(a.root_pos_w)[0] - self.origin, _arr(a.root_quat_w)[0])
            coms[name] = _arr(getattr(a, "root_com_pos_w", a.root_pos_w))[0] - self.origin
        pushes = _RT["holds"].update(poses, (cp, cq))
        fc, tc = np.zeros(3), np.zeros(3)
        for name, asset in self.assets.items():
            f, t = np.zeros(3), np.zeros(3)
            for p, force in pushes.get(name, []):
                f += force
                t += np.cross(p - coms[name], force)
                fc -= force
                tc -= np.cross(p - ccom, force)
            dev = getattr(asset.data.root_pos_w, "torch", asset.data.root_pos_w).device
            asset.instantaneous_wrench_composer.set_forces_and_torques_index(
                forces=torch.tensor(f.astype(np.float32).reshape(1, 1, 3), device=dev),
                torques=torch.tensor(t.astype(np.float32).reshape(1, 1, 3), device=dev),
                body_ids=[0], env_ids=[0], is_global=True)
        dev = getattr(d.body_pos_w, "torch", d.body_pos_w).device
        self.art.instantaneous_wrench_composer.set_forces_and_torques_index(
            forces=torch.tensor(fc.astype(np.float32).reshape(1, 1, 3), device=dev),
            torques=torch.tensor(tc.astype(np.float32).reshape(1, 1, 3), device=dev),
            body_ids=[self.body], env_ids=[0], is_global=True)


def held() -> dict:
    """{fuse: [held, held]} as of the last physics step (empty before the first)."""
    h = _RT.get("holds")
    return {k: list(v) for k, v in h.held.items()} if h else {}


def _patch_isaaclab() -> None:
    from isaaclab.scene import InteractiveScene

    if getattr(InteractiveScene, "_carrierkit", False):
        return
    InteractiveScene._carrierkit = True
    write, reset_ = InteractiveScene.write_data_to_sim, InteractiveScene.reset

    def write_data_to_sim(self):
        if _SEATS:
            try:
                if "rt" not in _RT:
                    _RT["rt"] = _Runtime(self)
                _RT["rt"].physics_step()
            except KeyError:
                pass  # the scene is still being built
        return write(self)

    def reset_scene(self, env_ids=None):
        out = reset_(self, env_ids)
        if "holds" in _RT:
            _RT["holds"].reset()
        return out

    InteractiveScene.write_data_to_sim, InteractiveScene.reset = write_data_to_sim, reset_scene


_orig_patch = kit._patch


def _patch(scene) -> None:
    _orig_patch(scene)
    if getattr(scene, "_carrierkit", False):
        return
    scene._carrierkit = True
    _patch_isaaclab()


kit._patch = _patch
if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
