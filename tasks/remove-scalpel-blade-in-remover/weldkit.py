"""weldkit: fixed joints between yamkit bodies, switched on and off while a trial runs.

A part fixed to another until a mechanism lets it go (a scalpel blade on its handle's lug), and a part caught by a
mechanism once it is let go (the blade held by a remover's teeth), are PhysX fixed joints between two rigid bodies:

    WELDS = weldkit.Welds()
    WELDS.joint("lug", parent="handle", child="blade", at_spawn=True)          # on from the start
    WELDS.joint("teeth", parent="remover/base", child="blade", at_spawn=False)  # authored when switched on

`parent` and `child` name task objects (a yamkit Prop is one rigid body; "name/base" is a yamkit Mechanism's base
link). A joint on at spawn is written into the stage as the second of its bodies spawns, with the frames of the two
bodies' spawn poses (`spawn_pose`), so the pair starts fixed as laid out. `switch(name, on)` asks for a change; the
change is written at the next control step (`apply`): off sets the joint's `physics:jointEnabled` false; on authors a
new joint prim from the two bodies' live poses (a disabled PhysX joint is not re-used), enabled last. Every reset
removes the joints switched on during the trial and turns the spawn joints back on (`reset`). The pair of a joint never
collides while it is on.

Pure Python at import time; pxr and Isaac Lab are imported inside functions. Offline, `switch` only records the
request (`state(name)` reports it), so a scorer test can follow the same sequence without a simulator.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


def _quat_xyzw(R) -> np.ndarray:
    R = np.asarray(R, float)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    if w > 1e-6:
        return np.array([(R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w), w])
    i = int(np.argmax(np.diag(R)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(max(0.0, 1 + R[i, i] - R[j, j] - R[k, k])) * 2
    q = [0.0, 0.0, 0.0, (R[k, j] - R[j, k]) / s]
    q[i], q[j], q[k] = s / 4, (R[j, i] + R[i, j]) / s, (R[k, i] + R[i, k]) / s
    return np.array(q)


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


@dataclass
class Joint:
    name: str
    parent: str
    child: str
    at_spawn: bool
    on: bool = False
    path: str | None = None
    generation: int = 0
    pending: list = field(default_factory=list)


class Welds:
    """The task's switchable fixed joints."""

    def __init__(self):
        self.joints: dict[str, Joint] = {}
        self.paths: dict[str, str] = {}  # task object name -> its prim path in the stage (env 0)
        self.spawn_poses: dict[str, tuple] = {}  # task object name -> (pos, rotation matrix) at spawn
        self.log: list = []

    # -- declaration ---------------------------------------------------------------------------------------------------
    def joint(self, name: str, parent: str, child: str, at_spawn: bool) -> Joint:
        j = Joint(name, parent, child, at_spawn, on=at_spawn)
        self.joints[name] = j
        return j

    def spawn_pose(self, obj: str, pos, R) -> None:
        self.spawn_poses[obj] = (np.asarray(pos, float), np.asarray(R, float))

    # -- run time ------------------------------------------------------------------------------------------------------
    def switch(self, name: str, on: bool) -> None:
        j = self.joints[name]
        if j.on != bool(on):
            j.on = bool(on)
            j.pending.append(bool(on))

    def state(self, name: str) -> bool:
        return self.joints[name].on

    def offline_reset(self) -> None:
        for j in self.joints.values():
            j.on, j.pending = j.at_spawn, []

    # -- simulator side ------------------------------------------------------------------------------------------------
    def _body(self, ref: str) -> str:
        obj, _, link = ref.partition("/")
        return self.paths[obj] + (f"/{link}" if link else "")

    def _author(self, stage, j: Joint, p_pose, c_pose) -> None:
        """A new fixed joint prim for j: disabled while written, frames from the two poses (world), enabled last."""
        from pxr import Gf, Sdf, UsdPhysics

        if j.path is not None and stage.GetPrimAtPath(j.path).IsValid():
            stage.RemovePrim(j.path)
        j.generation += 1
        j.path = f"{self.paths[j.child.partition('/')[0]]}/weld_{j.name}_{j.generation}"
        (pp, Rp), (pc, Rc) = p_pose, c_pose
        local_pos = Rp.T @ (pc - pp)  # the child's origin in the parent's frame
        q = _quat_xyzw(Rp.T @ Rc)
        jp = UsdPhysics.FixedJoint.Define(stage, j.path)
        jp.CreateJointEnabledAttr().Set(False)
        jp.CreateBody0Rel().SetTargets([Sdf.Path(self._body(j.parent))])
        jp.CreateBody1Rel().SetTargets([Sdf.Path(self._body(j.child))])
        jp.CreateLocalPos0Attr().Set(Gf.Vec3f(*(float(v) for v in local_pos)))
        jp.CreateLocalRot0Attr().Set(Gf.Quatf(float(q[3]), Gf.Vec3f(float(q[0]), float(q[1]), float(q[2]))))
        jp.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
        jp.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
        jp.CreateCollisionEnabledAttr().Set(False)
        jp.CreateExcludeFromArticulationAttr().Set(True)
        jp.CreateJointEnabledAttr().Set(True)  # the last write
        self.log.append(f"authored {j.path}")

    def _enable(self, stage, j: Joint, on: bool) -> None:
        from pxr import UsdPhysics

        if j.path is not None and stage.GetPrimAtPath(j.path).IsValid():
            UsdPhysics.Joint(stage.GetPrimAtPath(j.path)).GetJointEnabledAttr().Set(bool(on))
            self.log.append(f"{'enabled' if on else 'disabled'} {j.path}")

    def spawned(self, name: str, prim_path: str, stage) -> None:
        """Called by the spawner of each welded object: once both bodies of a spawn joint exist, write it."""
        self.paths[name] = str(prim_path)
        for j in self.joints.values():
            objs = (j.parent.partition("/")[0], j.child.partition("/")[0])
            if j.at_spawn and j.path is None and all(o in self.paths for o in objs) \
                    and all(o in self.spawn_poses for o in objs):
                self._author(stage, j, self.spawn_poses[objs[0]], self.spawn_poses[objs[1]])

    def apply(self, env, poses) -> None:
        """Write the requested switches; `poses(ref)` gives a body's live (pos, R) in the env frame."""
        if not any(j.pending for j in self.joints.values()):
            return
        from isaaclab.sim.utils.stage import get_current_stage

        stage = get_current_stage()
        for j in self.joints.values():
            if not j.pending:
                continue
            on = j.pending[-1]
            j.pending.clear()
            if on and not j.at_spawn:
                self._author(stage, j, poses(j.parent), poses(j.child))
            else:
                self._enable(stage, j, on)

    def reset(self, before: bool) -> None:
        """Around a scene reset: before it, remove the joints switched on during the trial (they would pull their
        pair together while the reset puts the parts back); after it, turn the spawn joints back on."""
        try:
            from isaaclab.sim.utils.stage import get_current_stage

            stage = get_current_stage()
        except Exception:  # noqa: BLE001 - no simulator
            stage = None
        for j in self.joints.values():
            j.pending.clear()
            if before and not j.at_spawn:
                if stage is not None and j.path is not None and stage.GetPrimAtPath(j.path).IsValid():
                    stage.RemovePrim(j.path)
                    self.log.append(f"removed {j.path}")
                j.path, j.on = None, False
            if not before and j.at_spawn:
                if stage is not None:
                    self._enable(stage, j, True)
                j.on = True


def pose_of(state) -> tuple:
    """(pos, R) of a task object's state."""
    return np.asarray(state.pos, float), _rot(state.quat_xyzw)
