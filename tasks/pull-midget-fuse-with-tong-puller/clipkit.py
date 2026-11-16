"""clipkit: the spring clips of an open fuse block, as a law on each fuse's two ferrules. Import after yamkit:
    import clipkit as ck

A cartridge fuse in an open-clip block is held at each ferrule by a clip whose two fingers wrap a little past the
ferrule's widest point and press it onto the clip's base. The fingers, the clip base and the block are real colliders
(task.py); what they cannot do is spring open. This module adds that: while a ferrule sits in its clip, the clip
presses it toward the clip's base with a set force (`Clip.hold`), applied at the ferrule's centre before every
physics step. A ferrule drawn out of its clip far enough to pass the fingers' lips (`Clip.release` out from its seat)
or pushed sideways or along its axis off the clip is free: no force. A ferrule pushed back fully into its clip is held
again. Nothing else holds a fuse: no joint, no attachment to the hand or the puller.

The force is constant while it acts (no stiffness), so it cannot ring on a light body (a spring on a 9 g fuse at the
simulator's 240 Hz would; see the kit's tests).

`seat_state(fuse_pos, fuse_quat, clip)` is pure NumPy; the scorer and the offline tests use it on poses.
"""

from __future__ import annotations

import sys
from dataclasses import dataclass

import numpy as np
import yamkit as kit


@dataclass(frozen=True)
class Clip:
    """One fuse's pair of clips. `name`: the fuse prop. `seat`: the fuse's centre seated (world, m, env frame).
    `out`: unit vector out of the clips (from the clip base toward the open side). `axis`: the fuse's axis seated
    (unit). `ends`: the two ferrule centres along the fuse's own z axis (m). `hold`: the force (N) each clip presses
    its ferrule toward the clip base with. `release`: how far out of its seat (m) a ferrule's centre is past the
    fingers' lips. `side` and `along`: how far (m) a ferrule's centre can be pushed sideways or along the axis before
    it is off its clip."""

    name: str
    seat: tuple
    out: tuple = (-1.0, 0.0, 0.0)
    axis: tuple = (0.0, 0.0, 1.0)
    ends: tuple = (-0.0143, 0.0143)
    hold: float = 6.0
    release: float = 0.0025
    side: float = 0.003
    along: float = 0.005
    reseat: float = 0.0006


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def ferrules(pos, quat, clip: Clip):
    """World centres of the fuse's two ferrules."""
    r = _rot(quat)
    return [np.asarray(pos, float) + r @ np.array([0.0, 0.0, e]) for e in clip.ends]


def offsets(pos, quat, clip: Clip):
    """Per ferrule: (out, side, along) of its centre from its seat (m): `out` along clip.out (positive = drawn out),
    `side` the sideways distance across the clip, `along` along the fuse's seated axis."""
    o, a = np.asarray(clip.out, float), np.asarray(clip.axis, float)
    s = np.cross(a, o)
    res = []
    for e, p in zip(clip.ends, ferrules(pos, quat, clip)):
        d = p - (np.asarray(clip.seat, float) + e * a)
        res.append((float(d @ o), float(abs(d @ s)), float(abs(d @ a))))
    return res


def off_clip(offset, clip: Clip) -> bool:
    """A ferrule out of its clip: drawn out past the lips, or off it sideways or along the axis."""
    out, side, along = offset
    return out > clip.release or side > clip.side or along > clip.along


def seated(offset, clip: Clip) -> bool:
    """A ferrule fully back in its clip (where a free one is caught again)."""
    out, side, along = offset
    return abs(out) < clip.reseat and side < clip.side / 3 and along < clip.along / 3


def seat_state(pos, quat, clip: Clip):
    """(out_of_clip, offsets) for the two ferrules, from the pose alone."""
    offs = offsets(pos, quat, clip)
    return [off_clip(d, clip) for d in offs], offs


class Holds:
    """Which ferrules their clips hold, step by step: held at the start; let go once off the clip; caught again once
    pushed fully back in."""

    def __init__(self, clips):
        self.clips = list(clips)
        self.reset()

    def reset(self):
        self.held = {c.name: [True, True] for c in self.clips}

    def update(self, poses):
        """poses: {name: (pos, quat_xyzw)}. Returns {name: [(point, force), ...]} for the held ferrules."""
        out = {}
        for c in self.clips:
            if c.name not in poses:
                continue
            pos, quat = poses[c.name]
            offs = offsets(pos, quat, c)
            pts = ferrules(pos, quat, c)
            push = []
            for i, d in enumerate(offs):
                if self.held[c.name][i] and off_clip(d, c):
                    self.held[c.name][i] = False
                elif not self.held[c.name][i] and seated(d, c):
                    self.held[c.name][i] = True
                if self.held[c.name][i]:
                    push.append((pts[i], -c.hold * np.asarray(c.out, float)))
            out[c.name] = push
        return out


_CLIPS: list = []
_RT: dict = {}


def register(clips) -> None:
    """The fuses' clips (task.py calls this once at import)."""
    _CLIPS[:] = list(clips)
    _RT.pop("holds", None)


def _arr(x) -> np.ndarray:
    x = getattr(x, "torch", x)
    if hasattr(x, "detach"):
        return x.detach().cpu().numpy().astype(np.float64)
    return np.asarray(x, dtype=np.float64)


class _Runtime:
    def __init__(self, scene):
        self.scene = scene
        self.assets = {c.name: scene[c.name] for c in _CLIPS}
        self.origin = _arr(scene.env_origins)[0]
        _RT.setdefault("holds", Holds(_CLIPS))

    def physics_step(self) -> None:
        import torch

        poses, coms = {}, {}
        for name, asset in self.assets.items():
            d = asset.data
            poses[name] = (_arr(d.root_pos_w)[0] - self.origin, _arr(d.root_quat_w)[0])
            coms[name] = _arr(getattr(d, "root_com_pos_w", d.root_pos_w))[0] - self.origin
        pushes = _RT["holds"].update(poses)
        for name, asset in self.assets.items():
            f, t = np.zeros(3), np.zeros(3)
            for p, force in pushes.get(name, []):
                f += force
                t += np.cross(p - coms[name], force)
            dev = getattr(asset.data.root_pos_w, "torch", asset.data.root_pos_w).device
            ft = torch.tensor(f.astype(np.float32).reshape(1, 1, 3), device=dev)
            tt = torch.tensor(t.astype(np.float32).reshape(1, 1, 3), device=dev)
            asset.instantaneous_wrench_composer.set_forces_and_torques_index(forces=ft, torques=tt, body_ids=[0],
                                                                             env_ids=[0], is_global=True)


def held() -> dict:
    """{fuse: [held, held]} as of the last physics step (empty before the first)."""
    h = _RT.get("holds")
    return {k: list(v) for k, v in h.held.items()} if h else {}


def _patch_isaaclab() -> None:
    from isaaclab.scene import InteractiveScene

    if getattr(InteractiveScene, "_clipkit", False):
        return
    InteractiveScene._clipkit = True
    write, reset_ = InteractiveScene.write_data_to_sim, InteractiveScene.reset

    def write_data_to_sim(self):
        if _CLIPS:
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
    if getattr(scene, "_clipkit", False):
        return
    scene._clipkit = True
    _patch_isaaclab()


kit._patch = _patch
if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])



@dataclass(frozen=True)
class Spring:
    """A yamkit link law: the tong's own spring, pulling an arm back to `rest` (rad) with stiffness `k` (N*m/rad) and
    damping `damp`, both applied by PhysX's implicit joint drive every physics step (stable on a light arm)."""

    rest: float = 0.0
    k: float = 8.0
    damp: float = 0.02

    def drive(self, key, q, qd, J, mem):
        return self.rest, self.k, 0.0, self.damp
