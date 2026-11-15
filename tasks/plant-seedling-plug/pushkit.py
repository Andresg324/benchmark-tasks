"""pushkit: a side push on one free object, asked for by the scorer (a spring-gauge firmness test).

The scorer calls `request(name, force, point)` with a force (N, world frame) and the world point it acts at, or
`request(name, None)` to stop. After every control step this module writes the latest request into that object's
external wrench in the simulator, so the force acts through the next control step (a one-step lag, 50 ms at 20 Hz).
For an articulated object (a yamkit Mechanism) the wrench goes on its base body: for the seedling, the root ball,
with the force and its moment about the root ball exactly as a push on the stem at that point hands them on once the
stem has taken up its bend. Nothing else is moved or drawn: the physics engine decides whether the object gives way.

The module hooks yam_arena.scene's object_states when the simulator imports it (as yamkit does). Plain Python at
import time; offline tests read `pending()` instead.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys

_REQ: dict = {}
_APPLIED: dict = {}


def request(name: str, force=None, point=None) -> None:
    _REQ[name] = None if force is None else (tuple(float(v) for v in force), tuple(float(v) for v in point))


def pending() -> dict:
    return dict(_REQ)


def _apply(env) -> None:
    import torch

    base = env.unwrapped
    for name, req in list(_REQ.items()):
        if _APPLIED.get(name) == req:
            continue
        try:
            obj = base.scene[name]
        except KeyError:
            continue
        names = list(getattr(obj, "body_names", []) or [])
        body = names.index("base") if "base" in names else 0  # a mechanism: its base (the root ball)
        dev = base.scene.env_origins.device
        comp = obj.permanent_wrench_composer
        if req is None:
            comp.reset(env_ids=torch.tensor([0], device=dev))
        else:
            f, p = req
            origin = base.scene.env_origins[0]
            force = torch.tensor([[f]], dtype=torch.float32, device=dev)
            pos = (torch.tensor([[p]], dtype=torch.float32, device=dev) + origin[None, None, :]).float()
            comp.set_forces_and_torques_index(forces=force, torques=torch.zeros_like(force), positions=pos,
                                              body_ids=torch.tensor([body], device=dev),
                                              env_ids=torch.tensor([0], device=dev), is_global=True)
        _APPLIED[name] = req


def _patch(scene) -> None:
    if getattr(scene, "_pushkit", False):
        return
    scene._pushkit = True
    states = scene.object_states

    def object_states(env, names):
        out = states(env, names)
        try:
            _apply(env)
        except Exception as e:  # a failed push must be visible in the log, never silent
            print(f"pushkit: could not apply the push: {e!r}", flush=True)
        return out

    scene.object_states = object_states


class _Hook(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    def find_spec(self, fullname, path, target=None):
        if fullname != "yam_arena.scene":
            return None
        sys.meta_path.remove(self)
        try:
            spec = importlib.util.find_spec(fullname)
        finally:
            sys.meta_path.insert(0, self)
        if spec is None:
            return None
        self._inner = spec.loader
        spec.loader = self
        return spec

    def create_module(self, spec):
        return self._inner.create_module(spec)

    def exec_module(self, module):
        self._inner.exec_module(module)
        _patch(module)


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _Hook) for f in sys.meta_path):
    sys.meta_path.insert(0, _Hook())
