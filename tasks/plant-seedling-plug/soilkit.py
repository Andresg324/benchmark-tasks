"""soilkit: loose potting-mix crumbs as real rigid bodies (spheres) for a yam_arena task file.

A task declares its crumbs as `Grain`s (center, radius, mass, color) and calls `setup(grains, ...)` at import. When the
simulator builds the scene, each grain is spawned as its own free rigid body (a sphere collider and the same visible
sphere) with its own friction, rolling damping and solver settings. The grains are not in the task's OBJECTS:
yam_arena's object jitter moves every free object independently by up to 3 cm, which would pile a dense bed of grains
into each other, so they keep the layout given here (the scene's reset puts them back).

After every control step `objects["soil"]` holds an (N, 3) array: each grain's position (x, y, z, in the same frame as
the other objects), in the order of `grains`, and `objects["soil_quat"]` an (N, 4) array of their orientations
(x, y, z, w). The scorer reads only these: where the crumbs really are and how they lie. Nothing is
drawn or moved by this module; the physics engine moves the grains.

Nothing on disk changes. The module hooks yam_arena.scene when the simulator imports it (as yamkit does), wrapping
task_room (to add the grains to the scene) and object_states (to read them). Plain Python and NumPy at import time.
"""

from __future__ import annotations

import importlib.abc
import importlib.util
import sys
from dataclasses import dataclass

import numpy as np

KEY = "soil"
KEY_QUAT = "soil_quat"


@dataclass(frozen=True)
class Grain:
    """One crumb centered at `pos` (world frame, m): a sphere of `radius` (m), or with `cube` a cube of side 2 x
    `radius` turned by `quat_xyzw` (a clod)."""

    name: str
    pos: tuple
    radius: float
    mass: float  # kg
    color: tuple  # sRGB 0..1
    cube: bool = False
    quat_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)


@dataclass(frozen=True)
class Physics:
    """The grains' contact properties. Moist potting mix is grippy and does not roll: friction 1.0 (combined by
    "average" with what it touches) and angular damping, which stands in for the rolling resistance a lumpy crumb has.
    Measured in the simulator (Nov 14): with these settings a heap of 1.1 cm cube clods holds its slope at a hole's
    edge (1 of 87 fell in while the scene settled), while 1.2 cm spheres slump into it (36 of 93) even so."""

    static_friction: float = 1.0
    dynamic_friction: float = 0.9
    angular_damping: float = 30.0  # 1/s
    contact_offset: float = 0.002  # m
    rest_offset: float = 0.0
    solver_position_iterations: int = 8
    solver_velocity_iterations: int = 1
    max_depenetration_velocity: float = 0.3  # m/s


_SPEC: dict = {"grains": (), "physics": Physics()}


def setup(grains, physics: Physics = Physics()) -> None:
    names = [g.name for g in grains]
    if len(set(names)) != len(names):
        raise ValueError("soil grain names must be unique")
    _SPEC["grains"], _SPEC["physics"] = tuple(grains), physics


def grains() -> tuple:
    return _SPEC["grains"]


def _linear(c: float) -> float:
    return c / 12.92 if c <= 0.04045 else ((c + 0.055) / 1.055) ** 2.4


def _objects(scene) -> list:
    ph = _SPEC["physics"]
    su = scene.sim_utils
    out = []
    for g in _SPEC["grains"]:
        shape = dict(size=(2 * float(g.radius),) * 3) if g.cube else dict(radius=float(g.radius))
        cfg = (su.CuboidCfg if g.cube else su.SphereCfg)(
            **shape,
            rigid_props=scene.PhysxRigidBodyPropertiesCfg(
                solver_position_iteration_count=ph.solver_position_iterations,
                solver_velocity_iteration_count=ph.solver_velocity_iterations,
                max_depenetration_velocity=ph.max_depenetration_velocity,
                angular_damping=ph.angular_damping),
            mass_props=su.MassPropertiesCfg(mass=float(g.mass)),
            collision_props=scene.PhysxCollisionPropertiesCfg(contact_offset=ph.contact_offset,
                                                              rest_offset=ph.rest_offset),
            physics_material=su.RigidBodyMaterialCfg(static_friction=ph.static_friction,
                                                     dynamic_friction=ph.dynamic_friction),
            visual_material=su.PreviewSurfaceCfg(diffuse_color=tuple(_linear(v) for v in g.color), roughness=0.95),
        )
        out.append(scene.Object(name=g.name, object_type=scene.ObjectType.RIGID, spawner_cfg=cfg,
                                initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in g.pos),
                                                        rotation_xyzw=tuple(float(v) for v in g.quat_xyzw))))
    return out


def read(env) -> tuple:
    """The grains' positions now, (N, 3) minus the env origin, and their orientations, (N, 4) x, y, z, w."""
    import torch

    names = [g.name for g in _SPEC["grains"]]
    if not names:
        return np.zeros((0, 3)), np.zeros((0, 4))
    base = env.unwrapped
    world = base.arena_world
    origin = base.scene.env_origins[0]
    pose = torch.stack([world.get_pose_w(n)[0, :7] for n in names])
    pos = (pose[:, :3] - origin[None, :]).detach().cpu().numpy().astype(np.float64)
    return pos, pose[:, 3:7].detach().cpu().numpy().astype(np.float64)


def _patch(scene) -> None:
    if getattr(scene, "_soilkit", False):
        return
    scene._soilkit = True
    room, states = scene.task_room, scene.object_states

    def task_room(*a, **k):
        return [*room(*a, **k), *_objects(scene)]

    def object_states(env, names):
        out = states(env, [n for n in names if n not in (KEY, KEY_QUAT)])
        if _SPEC["grains"]:
            out[KEY], out[KEY_QUAT] = read(env)
        return out

    scene.task_room, scene.object_states = task_room, object_states


class _Hook(importlib.abc.MetaPathFinder, importlib.abc.Loader):
    """Patches yam_arena.scene as soon as it is imported (after any other hook's patch)."""

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
