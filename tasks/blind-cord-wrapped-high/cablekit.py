"""cablekit: cables, cords, straps and hoses as chains of short linked segments, for a yam_arena task file (with yamkit).

A cable is N rigid segments in a chain. Each joint bends about two axes (it neither stretches nor twists), with a
bending stiffness and damping that pull it back toward the shape the cable started in (cables keep their set). One end
can be anchored to the world (a ground lead bolted to a drum, a cord leaving a wall box), and the free end can carry a
fitting: a spring clamp, a plug or a hook, which attaches to named target points on other objects.

    import yamkit as kit
    import cablekit as ck

    LEAD = ck.cable("bond_cable", path=[(0.33, 0.09, TOP + 0.13), (0.33, 0.16, TOP + 0.004), (0.22, 0.24, TOP + 0.004)],
                    radius=0.0035, segment=0.03, mass=0.04, anchored=True,
                    end=ck.Fitting("clamp", kind="clamp", targets=(ck.Target("can", body="can", point=(0, 0.05, 0.05)),)))
    OBJECTS = [..., LEAD]

The cable is built along `path` (a polyline in world coordinates, metres): the first point is the anchor (or the free
start), and segments of about `segment` metres follow the path; past the path's end the cable carries on straight.
With `anchored=True` the first point is fixed to the world (put it on the object the cable hangs from). The fitting
sits on the last segment's end, pointing along the cable.

Fittings and attachment. A `Fitting` has a `jaw` point (the business end: between a clamp's jaws, a plug's prongs, a
hook's bend) and a `grip` point (where a gripper holds it), both in the fitting's frame (origin at the cable's end, +x
out along the cable, z up when the cable lies flat). Its `targets` are points on other objects (`Target(name, body,
point, radius, facing, cone)`: `body` is a task object's name, "mech/link" for a yamkit Mechanism link, or None for a
world point; `point` and `facing` are in that body's frame). It attaches when its jaw comes inside the target's
catch box (`size`, in the target body's frame; default a cube 2 * `radius` on a side), and, if `facing` is given, with the fitting pointing (its +x) within `cone` degrees of `facing` (a clamp
must come at a tab edge-on, not from inside the wall):
- kind "clamp" (a spring clamp or alligator clip): it bites only when not squeezed, i.e. not held by a gripper (see `held`
  below); a gripper that holds it while it is on squeezes
  it open, and it lets go. So a clamp is put on by pressing it on and letting go, and taken off by gripping it.
- kind "plug": it seats once its jaw has stayed within the radius for `dwell` seconds, held or not.
- kind "hook": like a clamp it engages when let go within the radius; gripping does not release it.
Any fitting also comes off when pulled harder than `release_force` newtons (a tug on the cable, a gripper pulling it
away). While attached, a stiff spring-damper holds the jaw point on the target point and the fitting's orientation at
what it was when it attached (relative to the target), applied at every physics substep (240 Hz). A movable target
(a rigid object with mass) feels the reaction force. Nothing attaches to anything that is not a listed target.

Scorer API (`objects` as score() gets it; cable name "bond_cable", fitting key "bond_cable/clamp"):
- `objects["bond_cable"]` is a `CableState` (an ObjectState with `points`, the joint points from start to end);
  `objects["bond_cable/clamp"]` the fitting's body (pos = its frame origin, at the cable's end).
- `points(objects, cable)`: the polyline, (segments + 1, 3). `jaw(objects, key)`: the jaw point in world coordinates.
- `attached(objects, key)`: the target's name, or None. `attached_since(objects, key)`: the control step it attached
  at (this trial), or None. `attach_log(objects, key)`: every attachment this trial, as (target, start step, end step
  or None).
- `held(objects, key)`: a gripper holds the fitting now (fingers closed to within 1.2 cm of its width, tool point within
  2.5 cm of its axis anywhere along its length). `near(objects, key, p, r)`: some point of the fitting's axis is within
  r of world point p (for "hands off" checks).
- `length_between(objects, cable, a, b)`: the length along the cable between its closest points to world points a, b.
- `closest(objects, cable, p)`: (distance from p to the cable, arc length from the start at the closest point).
- `span(objects, cable)`: the straight distance between the cable's two ends. `length(cable)`: its full length.
All follow the trial: the simulator's scene reset (yamkit's `kit.episode()`) puts the cable back in its starting
shape, detaches every fitting (or reattaches one with `start=`) and clears the logs.

Demo helpers (plain NumPy, no simulator): `spec.nodes()` gives the starting joint points; `spec.fitting_point(local)`
the starting world position of a point in the fitting's frame (e.g. its `grip`); `spec.fitting_axis()` the direction the
fitting points at the start. A straight-down grip with yaw `ck.grip_yaw_across(axis)` closes the fingers across it.

Tests without a simulator: `register(*cables)`, then `set_attached(key, target)` / `set_held(key, bool)` and build a
CableState with `state_from_points(cable, points)`.

How it works. The cable is written as a USD articulation at start-up (as yamkit does for its props): a fixed anchor
body (or a free first segment as the articulation root), box-collider segments with capsule-shaped visual meshes,
D6 joints with the three translations and the twist locked and both bending axes limited to `max_bend` and driven
(implicit PhysX drives: `bend` N*m/rad toward the starting shape, `bend_damping`, plus `armature` for stability), and
the fitting welded on. The module hooks yam_arena.scene through yamkit's patch (yamkit must be imported first, which
importing this module does): task_object spawns a `Cable`, object_states adds the cable and its fitting and runs the
attach logic once per control step, and Isaac Lab's InteractiveScene.write_data_to_sim / reset are wrapped so the
attachment springs act every physics substep and a scene reset restores the cable. Plain Python and NumPy at import.

Stability and cost (measured at 20 Hz control, 240 Hz physics): see the "Cable kit" section of CLAUDE-CONTEXT.md.
Defaults: segments 2.5-4 cm, 2-8 g each, bend 0.004-0.02 N*m/rad, armature 1e-5. Self-collision is off by default
(segments then pass through each other; adjacent ones never collide).
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field

import numpy as np
import yamkit as kit
from yam_arena.task_file import Box, ObjectState

HOLD_R = 0.025  # a gripper holds a fitting when its tool point is within 2.5 cm of the fitting's axis (end to end)
FULL_OPEN = 0.094  # m between the fingers fully open (yam_arena's FINGER_OPEN on each of two fingers)
GRIPPERS = ("left_gripper", "right_gripper")
HOLD_MIN_OPEN = 0.5  # a gripper holds a fitting only if open at least half the fitting's width (the part sits between
# the pads; a fist closed on nothing reads 0)

COPPER = kit.Look((0.80, 0.50, 0.28), 0.35, metallic=0.9)
STEEL = kit.Look((0.72, 0.73, 0.75), 0.35, metallic=0.85)
BLACK = kit.Look((0.08, 0.08, 0.09), 0.6)


# -- specs -------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Target:
    """A point a fitting can attach to: `point` in the frame of `body` (a task object's name, "mech/link" for a yamkit
    mechanism's link, or None for the world), caught within `radius` metres of the fitting's jaw point. With `facing`
    (a direction in the body's frame) the fitting's +x must also point within `cone` degrees of it."""

    name: str
    body: str | None
    point: tuple
    radius: float = 0.012
    facing: tuple | None = None
    cone: float = 50.0
    size: tuple | None = None  # the catch zone: a box this size (x, y, z, metres) centred on `point`, in the body's
    # frame (local fix: the master used a sphere of `radius`, which let a jaw bite from the far side of a thin part);
    # None = a cube 2 * radius on a side

    def half(self) -> np.ndarray:
        return np.asarray(self.size, float) / 2 if self.size is not None else np.full(3, self.radius)


def in_target(t: Target, jaw, goal, R=None) -> bool:
    """The jaw point lies inside the target's catch box (goal = the target point in world coordinates, R = the target
    body's rotation; the box turns with its body)."""
    d = np.asarray(jaw, float) - np.asarray(goal, float)
    if R is not None:
        d = np.asarray(R, float).T @ d
    return bool(np.all(np.abs(d) <= t.half() + 1e-12))


@dataclass(frozen=True)
class Fitting:
    """What sits on the cable's free end. Frame: origin at the cable's end, +x out along the cable, z up when the
    cable lies flat. `size` = (length, width, height) of its box collider, which runs from x = 0 to x = `collider_len`
    (default: the length minus 1.2 cm, leaving room for the jaws). `meshes` default to a drawing for the kind. `jaw`
    and `grip` are points in this frame. `kind`: "clamp", "plug" or "hook" (see the module docstring). `start`: the
    name of a target it is attached to when each trial begins. `stiffness` (N/m) and `damping` (N*s/m) hold the jaw on
    the target; `turn_stiffness` (N*m/rad) and `turn_damping` hold its orientation; `release_force` (N) pulls it off."""

    name: str = "clamp"
    kind: str = "clamp"
    size: tuple = (0.056, 0.022, 0.016)
    mass: float = 0.04
    jaw: tuple | None = None
    grip: tuple | None = None
    collider_len: float | None = None
    meshes: tuple | None = None
    targets: tuple = ()
    dwell: float = 0.25
    release_force: float = 6.0
    stiffness: float = 400.0
    damping: float | None = None
    turn_stiffness: float = 0.25
    turn_damping: float = 0.004
    start: str | None = None
    raise_z: float | None = None  # its center line above the cable's axis (cable() sets it so both rest on a table)

    @property
    def dz(self) -> np.ndarray:
        return np.array([0.0, 0.0, self.raise_z or 0.0])

    @property
    def body_len(self) -> float:
        return self.collider_len if self.collider_len is not None else max(self.size[0] - 0.012, 0.6 * self.size[0])

    @property
    def jaw_point(self) -> np.ndarray:
        return np.asarray(self.jaw if self.jaw is not None else (self.size[0] - 0.005, 0.0, 0.0), float) + self.dz

    @property
    def grip_point(self) -> np.ndarray:
        return np.asarray(self.grip if self.grip is not None else (0.45 * self.body_len, 0.0, 0.0), float) + self.dz

    def drawing(self) -> tuple:
        """Its meshes (default: a drawing for the kind), raised by `raise_z`."""
        return tuple(_shift(m, self.dz) for m in self._drawing())

    def _drawing(self) -> tuple:
        if self.meshes is not None:
            return self.meshes
        L, w, h = self.size
        b = self.body_len
        if self.kind == "clamp":  # a spring bonding clamp: two steel handles, a coil, copper jaws past the body
            jl = L - b + 0.006
            return (kit.box((b * 0.45, 0, h * 0.30), (b * 0.9, w, h * 0.22), STEEL),
                    kit.box((b * 0.45, 0, -h * 0.30), (b * 0.9, w, h * 0.22), STEEL),
                    kit.cylinder((b * 0.55, 0, 0), h * 0.32, w * 0.9, STEEL, axis="y"),
                    kit.box((b * 0.12, 0, 0), (b * 0.24, w * 0.5, h * 0.7), BLACK),  # the cable's crimp
                    kit.box((L - jl / 2, 0, 0.0055), (jl, w * 0.8, 0.003), COPPER),
                    kit.box((L - jl / 2, 0, -0.0055), (jl, w * 0.8, 0.003), COPPER),
                    kit.box((b - 0.002, 0, 0), (0.006, w * 0.8, 0.014), STEEL))
        if self.kind == "plug":
            return (kit.box((b / 2, 0, 0), (b, w, h), BLACK),
                    kit.box((L - (L - b) / 2, 0.004, 0), (L - b, 0.002, 0.006), STEEL),
                    kit.box((L - (L - b) / 2, -0.004, 0), (L - b, 0.002, 0.006), STEEL))
        return (kit.box((b / 2, 0, 0), (b, w, h), STEEL),  # hook: a shank and an upturned bend
                kit.band([(b, 0, 0), (L, 0, 0), (L, 0, 0.012), (L - 0.008, 0, 0.016)], 0.004, 0.003, STEEL,
                         normal=(0, 1, 0)))


@dataclass(frozen=True)
class Cable(Box):
    """A cable. Build it with `cable(...)`. It subclasses `Box` only so yam_arena's task-file checks accept it; `pos`
    and `size` are the bounds of its starting path, `mass` stays None (yam_arena never jitters it)."""

    path: tuple = ()
    radius: float = 0.0035
    segments: int = 12
    seg_len: float = 0.03
    cable_mass: float = 0.04
    look: kit.Look = kit.Look((0.10, 0.45, 0.20), 0.5)
    anchored: bool = True
    anchor_meshes: tuple | None = None
    bend: float = 0.01
    bend_damping: float = 0.002
    max_bend: float = 1.3
    armature: float = 1e-5
    lin_damping: float = 0.4
    ang_damping: float = 1.0
    self_collide: bool = False
    end: Fitting | None = None

    def nodes(self) -> np.ndarray:
        """The starting joint points, start to end: (segments + 1, 3), world coordinates."""
        return _walk(np.asarray(self.path, float), self.seg_len, self.segments)

    def frames(self) -> list[np.ndarray]:
        """Each segment's starting rotation (columns: along the cable, sideways, up-ish)."""
        n = self.nodes()
        return [_frame(n[i + 1] - n[i]) for i in range(self.segments)]

    def fitting_axis(self) -> np.ndarray:
        return self.frames()[-1][:, 0]

    def fitting_point(self, local) -> np.ndarray:
        """Starting world position of a point given in the fitting's frame."""
        return self.nodes()[-1] + self.frames()[-1] @ np.asarray(local, float)

    @property
    def length(self) -> float:
        return self.segments * self.seg_len


def cable(name: str, path, *, radius: float = 0.0035, segment: float = 0.03, length: float | None = None,
          mass: float = 0.04, look: kit.Look | None = None, anchored: bool = True, end: Fitting | None = None,
          **kwargs) -> Cable:
    """A cable along `path` (world points), `length` long (default: the path's length) in segments of about
    `segment` metres. See the module docstring."""
    p = np.asarray(path, float)
    if len(p) < 2:
        raise ValueError(f"{name}: a cable path needs at least two points")
    total = length if length is not None else float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
    n = max(2, round(total / segment))
    seg = total / n
    if seg <= 2 * radius:
        raise ValueError(f"{name}: segments ({seg:.4f} m) must be longer than the cable is thick")
    tmp = Cable(name=name, pos=(0.0, 0.0, 0.0), size=(1.0, 1.0, 1.0), path=tuple(map(tuple, p)), segments=n,
                seg_len=seg)
    pts = tmp.nodes()
    lo, hi = pts.min(0) - radius, pts.max(0) + radius
    extra = dict(look=look) if look is not None else {}
    if end is not None and end.raise_z is None:
        import dataclasses

        end = dataclasses.replace(end, raise_z=max(0.0, end.size[2] / 2 - radius))
    return Cable(name=name, pos=tuple(((lo + hi) / 2).tolist()), size=tuple(np.maximum(hi - lo, 1e-3).tolist()),
                 color=(0.1, 0.45, 0.2), mass=None, path=tuple(map(tuple, p)), radius=radius, segments=n, seg_len=seg,
                 cable_mass=mass, anchored=anchored, end=end, **extra, **kwargs)


def _shift(m: kit.Mesh, d) -> kit.Mesh:
    return kit.Mesh(tuple(tuple(float(a + b) for a, b in zip(p, d)) for p in m.points), m.counts, m.indices, m.look)


# -- geometry ------------------------------------------------------------------------------------


def _frame(d) -> np.ndarray:
    """A rotation whose x axis is along d, with y horizontal where it can be (z then points up-ish)."""
    x = np.asarray(d, float)
    x = x / np.linalg.norm(x)
    y = np.cross([0.0, 0.0, 1.0], x)
    if np.linalg.norm(y) < 1e-6:  # vertical: y along world y
        y = np.array([0.0, 1.0, 0.0])
    y = y / np.linalg.norm(y)
    return np.column_stack([x, y, np.cross(x, y)])


def _walk(path: np.ndarray, L: float, n: int) -> np.ndarray:
    """n + 1 points along the polyline `path`, each exactly L from the one before (straight on past its end)."""
    nodes = [path[0]]
    k, t = 0, 0.0  # current polyline piece and the parameter on it
    for _ in range(n):
        c = nodes[-1]
        found = None
        while k < len(path) - 1:
            a, b = path[k], path[k + 1]
            d = b - a
            # |a + s d - c| = L, s in [t, 1], take the root ahead
            A = d @ d
            B = 2 * d @ (a - c)
            C = (a - c) @ (a - c) - L * L
            disc = B * B - 4 * A * C
            if A > 1e-12 and disc >= 0:
                s = (-B + math.sqrt(disc)) / (2 * A)
                if t - 1e-9 <= s <= 1.0:
                    found = a + s * d
                    t = s
                    break
            k, t = k + 1, 0.0
        if found is None:  # past the end: straight on
            d = path[-1] - path[-2] if len(nodes) < 2 else nodes[-1] - nodes[-2]
            found = c + L * d / np.linalg.norm(d)
            k = len(path)
        nodes.append(found)
    return np.array(nodes)


def _quat_wxyz(R) -> tuple:
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return (w, x, y, z)


def _rotm_xyzw(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _rot_vec(R) -> np.ndarray:
    """The rotation vector (axis * angle) of rotation matrix R."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1) / 2))
    a = math.acos(c)
    v = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    if a < 1e-6:
        return v / 2
    if math.pi - a < 1e-4:  # near 180 degrees: the axis from the symmetric part
        k = int(np.argmax(np.diag(R)))
        ax = R[:, k] + np.eye(3)[k]
        return a * ax / np.linalg.norm(ax)
    return a * v / (2 * math.sin(a))


# -- USD --------------------------------------------------------------------------------------------


def _mesh_usda(m: kit.Mesh, looks, ind: str, tag: str) -> str:
    return (f'{ind}def Mesh "{tag}" (\n{ind}    prepend apiSchemas = ["MaterialBindingAPI"]\n{ind})\n{ind}{{\n'
            f'{ind}    point3f[] points = [{", ".join(kit._fmt(p) for p in m.points)}]\n'
            f'{ind}    int[] faceVertexCounts = [{", ".join(map(str, m.counts))}]\n'
            f'{ind}    int[] faceVertexIndices = [{", ".join(map(str, m.indices))}]\n'
            f'{ind}    uniform token subdivisionScheme = "none"\n{ind}    uniform bool doubleSided = 1\n'
            f'{ind}    rel material:binding = </Root/Looks/M{looks.index(m.look)}>\n{ind}}}\n')


def _box_collider(center, size, ind: str, tag: str) -> str:
    return (f'{ind}def Cube "{tag}" (\n{ind}    prepend apiSchemas = ["PhysicsCollisionAPI", "PhysxCollisionAPI"]\n'
            f'{ind})\n{ind}{{\n{ind}    uniform token purpose = "guide"\n{ind}    double size = 1\n'
            f'{ind}    float physxCollision:contactOffset = 0.002\n{ind}    float physxCollision:restOffset = 0\n'
            f'{ind}    double3 xformOp:translate = {kit._fmt(center)}\n{ind}    float3 xformOp:scale = {kit._fmt(size)}\n'
            f'{ind}    uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:scale"]\n{ind}}}\n')


def _body(name, pos, R, mass, meshes, colliders, looks, spec: Cable, extra_api="", settings="") -> str:
    q = _quat_wxyz(R)
    out = [f'    def Xform "{name}" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", '
           f'"PhysxRigidBodyAPI"{extra_api}]\n    )\n    {{\n',
           settings,
           f"        float physics:mass = {mass:.6g}\n",
           f"        float physxRigidBody:linearDamping = {spec.lin_damping}\n",
           f"        float physxRigidBody:angularDamping = {spec.ang_damping}\n",
           "        float physxRigidBody:maxDepenetrationVelocity = 0.5\n",
           f"        double3 xformOp:translate = {kit._fmt(pos)}\n",
           f"        quatf xformOp:orient = ({q[0]:.7g}, {q[1]:.7g}, {q[2]:.7g}, {q[3]:.7g})\n",
           '        uniform token[] xformOpOrder = ["xformOp:translate", "xformOp:orient"]\n']
    for i, m in enumerate(meshes):
        out.append(_mesh_usda(m, looks, "        ", f"V{i}"))
    for i, (c, s) in enumerate(colliders):
        out.append(_box_collider(c, s, "        ", f"C{i}"))
    out.append("    }\n")
    return "".join(out)


def _quat_str(R) -> str:
    q = _quat_wxyz(R)
    return f"({q[0]:.7g}, {q[1]:.7g}, {q[2]:.7g}, {q[3]:.7g})"


def _bend_joint(name, body0, body1, pos0, R0, pos1, spec: Cable) -> str:
    lim = math.degrees(spec.max_bend)
    api = ", ".join([f'"PhysicsLimitAPI:{d}"' for d in ("transX", "transY", "transZ", "rotX", "rotY", "rotZ")]
                    + ['"PhysicsDriveAPI:rotY"', '"PhysicsDriveAPI:rotZ"', '"PhysxJointAPI"'])
    out = [f'    def PhysicsJoint "{name}" (\n        prepend apiSchemas = [{api}]\n    )\n    {{\n',
           f"        rel physics:body0 = </Root/{body0}>\n        rel physics:body1 = </Root/{body1}>\n",
           f"        point3f physics:localPos0 = {kit._fmt(pos0)}\n        quatf physics:localRot0 = {_quat_str(R0)}\n",
           f"        point3f physics:localPos1 = {kit._fmt(pos1)}\n        quatf physics:localRot1 = (1, 0, 0, 0)\n"]
    for d in ("transX", "transY", "transZ", "rotX"):  # locked: no stretch, no twist
        out.append(f"        float limit:{d}:physics:low = 1\n        float limit:{d}:physics:high = -1\n")
    for d in ("rotY", "rotZ"):
        out.append(f"        float limit:{d}:physics:low = {-lim:.6g}\n        float limit:{d}:physics:high = {lim:.6g}\n"
                   f'        uniform token drive:{d}:physics:type = "force"\n'
                   f"        float drive:{d}:physics:stiffness = {spec.bend * math.pi / 180:.6g}\n"
                   f"        float drive:{d}:physics:damping = {spec.bend_damping * math.pi / 180:.6g}\n"
                   f"        float drive:{d}:physics:maxForce = 1000\n"
                   f"        float drive:{d}:physics:targetPosition = 0\n")
    out.append("    }\n")
    return "".join(out)


def _frame0(spec: Cable):
    """The articulation's frame: the anchor (identity rotation at the path's start) or the first segment."""
    n = spec.nodes()
    if spec.anchored:
        return n[0], np.eye(3)
    return (n[0] + n[1]) / 2, spec.frames()[0]


def _cable_usda(spec: Cable) -> str:
    n, Rs = spec.nodes(), spec.frames()
    L, r = spec.seg_len, spec.radius
    o, R0 = _frame0(spec)

    def local(p, R):
        return R0.T @ (np.asarray(p) - o), R0.T @ R

    seg_mesh = kit.lathe([(0.0, -L / 2 - 0.8 * r), (0.6 * r, -L / 2 - 0.65 * r), (r, -L / 2), (r, L / 2),
                          (0.6 * r, L / 2 + 0.65 * r), (0.0, L / 2 + 0.8 * r)], spec.look, axis="x", n=14)
    fit = spec.end
    fit_meshes = fit.drawing() if fit else ()
    anchor_meshes = spec.anchor_meshes if spec.anchor_meshes is not None else (
        kit.box((0, 0, 0), (2.4 * r, 2.4 * r, 2.4 * r), STEEL),)
    looks = list(dict.fromkeys([spec.look] + [m.look for m in fit_meshes] +
                               ([m.look for m in anchor_meshes] if spec.anchored else [])))
    root_api = "" if not spec.anchored else ' (\n    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n)'
    out = ['#usda 1.0\n(\n    defaultPrim = "Root"\n    metersPerUnit = 1\n    upAxis = "Z"\n)\n',
           f'def Xform "Root"{root_api}\n{{\n']
    art_settings = (f"    bool physxArticulation:enabledSelfCollisions = {1 if spec.self_collide else 0}\n"
                    "    int physxArticulation:solverPositionIterationCount = 32\n"
                    "    int physxArticulation:solverVelocityIterationCount = 4\n")
    if spec.anchored:
        out.append(art_settings)
    out.append('    def Scope "Looks"\n    {\n')
    for i, lk in enumerate(looks):
        col = kit._fmt(kit._linear(c) for c in lk.color)
        out.append(f'        def Material "M{i}"\n        {{\n'
                   f'            token outputs:surface.connect = </Root/Looks/M{i}/S.outputs:surface>\n'
                   f'            def Shader "S"\n            {{\n'
                   f'                uniform token info:id = "UsdPreviewSurface"\n'
                   f'                color3f inputs:diffuseColor = {col}\n'
                   f'                float inputs:roughness = {lk.roughness}\n'
                   f'                float inputs:metallic = {lk.metallic}\n'
                   f'                float inputs:opacity = {lk.opacity}\n'
                   f'                token outputs:surface\n            }}\n        }}\n')
    out.append("    }\n")
    seg_m = spec.cable_mass / spec.segments
    if spec.anchored:
        out.append(_body("anchor", (0, 0, 0), np.eye(3), 0.1, anchor_meshes, [((0, 0, 0), (2 * r, 2 * r, 2 * r))],
                         looks, spec))
        out.append('    def PhysicsFixedJoint "root_joint"\n    {\n        rel physics:body1 = </Root/anchor>\n    }\n')
    for i in range(spec.segments):
        p, R = local((n[i] + n[i + 1]) / 2, Rs[i])
        root = i == 0 and not spec.anchored  # a free cable: the first segment is the articulation's root
        api = ', "PhysicsArticulationRootAPI", "PhysxArticulationAPI"' if root else ""
        sets = art_settings.replace("    ", "        ") if root else ""
        out.append(_body(f"s{i}", p, R, seg_m, (seg_mesh,), [((0, 0, 0), (L, 2 * r, 2 * r))], looks, spec, api, sets))
    for i in range(spec.segments):
        if i == 0 and not spec.anchored:
            continue
        if i == 0:
            body0, Rp, pp = "anchor", np.eye(3), np.zeros(3)  # anchor frame = articulation frame at the start node
            pos0 = R0.T @ (n[0] - o)
        else:
            Rp, pp = Rs[i - 1], (n[i - 1] + n[i]) / 2
            body0, pos0 = f"s{i - 1}", np.array([L / 2, 0, 0])
        out.append(_bend_joint(f"j{i}", body0, f"s{i}", pos0, Rp.T @ Rs[i], (-L / 2, 0, 0), spec))
    if fit is not None:
        p, R = local(n[-1], Rs[-1])
        b = fit.body_len
        out.append(_body(fit.name, p, R, fit.mass, fit_meshes, [((b / 2, 0, fit.dz[2]), (b, fit.size[1], fit.size[2]))],
                         looks, spec))
        out.append(f'    def PhysicsFixedJoint "{fit.name}_weld"\n    {{\n'
                   f"        rel physics:body0 = </Root/s{spec.segments - 1}>\n        rel physics:body1 = </Root/{fit.name}>\n"
                   f"        point3f physics:localPos0 = ({L / 2:.6g}, 0, 0)\n        quatf physics:localRot0 = (1, 0, 0, 0)\n"
                   "        point3f physics:localPos1 = (0, 0, 0)\n        quatf physics:localRot1 = (1, 0, 0, 0)\n    }\n")
    out.append("}\n")
    return "".join(out)


# -- state ------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class CableState(ObjectState):
    """An ObjectState for a cable (pos = the mean of its joint points; low/high bound them), plus `points`."""

    points: np.ndarray = None


@dataclass
class _Att:
    """A fitting's attachment, kept per trial."""

    episode: int
    target: Target | None = None
    since: int | None = None
    rel: np.ndarray | None = None  # fitting rotation relative to the target body's at the bite
    log: list = field(default_factory=list)
    dwell: int = 0
    held: bool = False
    pull: int = 0  # substeps in a row over the release force
    force: float = 0.0  # last spring force (N)


_CABLES: dict[str, Cable] = {}
_ATT: dict[str, _Att] = {}  # "cable/fitting" -> attachment
_JAW: dict[str, np.ndarray] = {}  # "cable/fitting" -> jaw point (world, env frame) at the last control step
_STATS = {"substeps": 0}
_ORIGIN = np.zeros(3)  # env 0's origin in the world (visual markers are placed in world coordinates)
_MOVABLE: set[str] = set()  # task objects with a mass (they feel the reaction of an attachment)


def _key(spec: Cable) -> str:
    return f"{spec.name}/{spec.end.name}"


def _target(spec: Cable, name: str) -> Target:
    for t in spec.end.targets:
        if t.name == name:
            return t
    raise KeyError(f"{spec.name}: no target {name!r}")


def _att(key: str) -> _Att:
    a = _ATT.get(key)
    if a is None or a.episode != kit.episode():
        a = _ATT[key] = _Att(kit.episode())
        spec = _CABLES.get(key.split("/")[0])
        if spec is not None and spec.end is not None and spec.end.start:
            a.target, a.since = _target(spec, spec.end.start), 0
            a.log.append([a.target.name, 0, None])
    return a


def _step_no() -> int:
    return int(kit._EPISODE["last"] or 0)


def _attach(a: _Att, t: Target | None) -> None:
    if t is None:
        if a.target is not None and a.log:
            a.log[-1][2] = _step_no()
        a.target, a.since, a.rel, a.pull = None, None, None, 0
    else:
        a.target, a.since, a.rel, a.pull = t, _step_no(), None, 0
        a.log.append([t.name, a.since, None])
    if os.environ.get("CABLEKIT_DEBUG"):
        print(f"cablekit step {_step_no()}: {'attached to ' + t.name if t else 'detached'}", flush=True)


# -- simulator side -------------------------------------------------------------------------------


def _arena_object(scene, spec: Cable):
    from isaaclab.actuators import ImplicitActuatorCfg

    class _CableObject(scene.Object):
        def _generate_articulation_cfg(self):
            cfg = super()._generate_articulation_cfg()
            cfg.actuators = {"bend": ImplicitActuatorCfg(joint_names_expr=[".*"], stiffness=spec.bend,
                                                         damping=spec.bend_damping, armature=spec.armature,
                                                         effort_limit_sim=1000.0)}
            cfg.init_state.joint_pos = {".*": 0.0}
            cfg.init_state.joint_vel = {".*": 0.0}
            return cfg

    o, R0 = _frame0(spec)
    w, x, y, z = _quat_wxyz(R0)
    return _CableObject(name=spec.name, usd_path=kit._write_text(spec.name, _cable_usda(spec)),
                        object_type=scene.ObjectType.ARTICULATION,
                        initial_pose=scene.Pose(position_xyz=tuple(float(v) for v in o), rotation_xyzw=(x, y, z, w)),
                        spawn_cfg_addon={"physics_material": scene._grippy()})


def _t(x):
    return getattr(x, "torch", x)


def _np(x) -> np.ndarray:
    return _t(x).detach().cpu().numpy().astype(np.float64)


def _body_pose(scene, body: str | None):
    """(pos, rot, lin vel, ang vel, asset or None, body index) of a target body, world frame (env origin included)."""
    if body is None:
        return np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), None, 0
    name, _, link = body.partition("/")
    asset = scene[name]
    names = list(getattr(asset, "body_names", []) or [])
    i = names.index(link) if link else (names.index("base") if "base" in names else 0)
    d = asset.data
    pos, quat = _np(d.body_pos_w)[0, i], _np(d.body_quat_w)[0, i]
    v, w = _np(d.body_lin_vel_w)[0, i], _np(d.body_ang_vel_w)[0, i]
    return pos, _rotm_xyzw(quat), v, w, asset, i


def _substep(scene) -> None:
    """Every physics substep: the attachment springs (jaw point to target point, orientation to its bite)."""
    import torch

    _STATS["substeps"] += 1
    origin = None
    for name, spec in _CABLES.items():
        if spec.end is None or name not in scene.articulations:
            continue
        key = _key(spec)
        a = _att(key)
        if a.target is None:
            continue
        fit = spec.end
        art = scene[name]
        i = list(art.body_names).index(fit.name)
        d = art.data
        p, R = _np(d.body_pos_w)[0, i], _rotm_xyzw(_np(d.body_quat_w)[0, i])
        com = _np(getattr(d, "body_com_pos_w", d.body_pos_w))[0, i]
        v, w = _np(d.body_lin_vel_w)[0, i], _np(d.body_ang_vel_w)[0, i]
        if origin is None:
            origin = _np(scene.env_origins)[0]
        tp, tR, tv, tw, tasset, ti = _body_pose(scene, a.target.body)
        if a.target.body is None:
            tp = origin
        jaw = p + R @ fit.jaw_point
        goal = tp + tR @ np.asarray(a.target.point, float)
        if a.rel is None:
            a.rel = tR.T @ R
        vj = v + np.cross(w, jaw - com)
        vg = tv + np.cross(tw, goal - tp)
        m = fit.mass + spec.cable_mass / spec.segments
        c = fit.damping if fit.damping is not None else 1.4 * math.sqrt(fit.stiffness * m)
        spring = fit.stiffness * (goal - jaw)
        F = spring - c * (vj - vg)
        a.force = float(np.linalg.norm(spring))
        a.pull = a.pull + 1 if a.force > fit.release_force else 0
        if a.pull >= 6:  # pulled harder than the release force for 6 substeps (25 ms): it comes off
            _attach(a, None)
            continue
        err = _rot_vec((tR @ a.rel) @ R.T)
        tau = np.cross(jaw - com, F) + fit.turn_stiffness * err - fit.turn_damping * (w - tw)
        dev = _t(d.body_pos_w).device
        art.instantaneous_wrench_composer.set_forces_and_torques_index(
            forces=torch.tensor(F, dtype=torch.float32, device=dev).view(1, 1, 3),
            torques=torch.tensor(tau, dtype=torch.float32, device=dev).view(1, 1, 3),
            body_ids=[i], env_ids=[0], is_global=True)
        if a.target.body is not None and a.target.body.partition("/")[0] in _MOVABLE:  # a loose target: the reaction
            tcom = _np(getattr(tasset.data, "body_com_pos_w", tasset.data.body_pos_w))[0, ti]
            tasset.instantaneous_wrench_composer.add_forces_and_torques_index(
                forces=torch.tensor(-F, dtype=torch.float32, device=dev).view(1, 1, 3),
                torques=torch.tensor(np.cross(goal - tcom, -F) - fit.turn_stiffness * err, dtype=torch.float32,
                                     device=dev).view(1, 1, 3),
                body_ids=[ti], env_ids=[0], is_global=True)


def _reset_cables(scene) -> None:
    """After a scene reset: each cable back in its starting shape, at rest; fittings detached (or on their start)."""
    import torch

    for name, spec in _CABLES.items():
        if name not in scene.articulations:
            continue
        art = scene[name]
        dev = _t(art.data.joint_pos).device
        nj = len(art.joint_names)
        ids = list(range(nj))
        art.write_joint_position_to_sim_index(position=torch.zeros((1, nj), device=dev), joint_ids=ids)
        art.write_joint_velocity_to_sim_index(velocity=torch.zeros((1, nj), device=dev), joint_ids=ids)
        if not spec.anchored:
            pose = _t(art.data.default_root_pose).clone()
            pose[:, :3] += _t(scene.env_origins)[:1]
            art.write_root_pose_to_sim_index(root_pose=pose)
            art.write_root_velocity_to_sim_index(root_velocity=torch.zeros((1, 6), device=dev))
        if spec.end is not None:
            _ATT.pop(_key(spec), None)


def _cable_states(base, name: str, origin) -> dict:
    spec = _CABLES[name]
    art = base.scene[name]
    names = list(art.body_names)
    if os.environ.get("CABLEKIT_DEBUG") and not _STATS.get(f"told_{name}"):
        _STATS[f"told_{name}"] = True
        print(f"cablekit {name}: bodies {names}, joints {list(art.joint_names)}", flush=True)
    pos, quat = _np(art.data.body_pos_w)[0], _np(art.data.body_quat_w)[0]
    vel = _np(art.data.body_lin_vel_w)[0]
    L = spec.seg_len
    idx = [names.index(f"s{i}") for i in range(spec.segments)]
    pts = []
    for k, i in enumerate(idx):
        x = _rotm_xyzw(quat[i])[:, 0]
        if k == 0:
            pts.append(pos[i] - x * L / 2 - origin)
        pts.append(pos[i] + x * L / 2 - origin)
    pts = np.array(pts)
    lo, hi = pts.min(0) - spec.radius, pts.max(0) + spec.radius
    out = {name: CableState(pos=pts.mean(0), quat_xyzw=np.array([0.0, 0, 0, 1]), lin_vel=vel[idx].mean(0),
                            ang_vel=np.zeros(3), low=lo, high=hi, points=pts)}
    if spec.end is not None:
        i = names.index(spec.end.name)
        p = pos[i] - origin
        half = np.asarray(spec.end.size, float) / 2
        out[_key(spec)] = ObjectState(pos=p, quat_xyzw=quat[i], lin_vel=vel[i], ang_vel=np.zeros(3), low=p - half,
                                      high=p + half)
    return out


def _target_world(objects, base, t: Target, origin) -> np.ndarray | None:
    """A target's point in the env frame (as the scorer's positions are)."""
    if t.body is None:
        return np.asarray(t.point, float)
    if base is not None:
        p, R, *_ = _body_pose(base.scene, t.body)
        return p - origin + R @ np.asarray(t.point, float)
    s = objects.get(t.body)
    if s is None:
        return None
    return np.asarray(s.pos, float) + _rotm_xyzw(s.quat_xyzw) @ np.asarray(t.point, float)


def _seg_dist(p, a, b) -> float:
    d = b - a
    u = max(0.0, min(1.0, float((p - a) @ d) / float(d @ d)))
    return float(np.linalg.norm(a + u * d - p))


def _holds(objects, g: str, s, fit: Fitting) -> bool:
    """Gripper g holds (squeezes) the fitting: its fingers closed to within 1.2 cm of the fitting's width, and its
    tool point within 2.5 cm of the fitting's axis anywhere from its back to its jaw tip."""
    op = kit.opening(objects, g)
    if op > min(0.6, (fit.size[1] + 0.012) / FULL_OPEN):
        return False
    if op < HOLD_MIN_OPEN * fit.size[1] / FULL_OPEN:  # closed tighter than the fitting is wide: an empty fist
        return False  # (local fix for this task: the master counts a closed empty gripper next to it as holding)
    R = _rotm_xyzw(s.quat_xyzw)
    a = np.asarray(s.pos, float) + R @ fit.dz
    b = np.asarray(s.pos, float) + R @ (np.array([fit.size[0], 0.0, 0.0]) + fit.dz)
    return _seg_dist(np.asarray(objects[g].pos, float), a, b) < HOLD_R


def _target_rot(objects, base, t: Target) -> np.ndarray:
    if base is not None:
        return _body_pose(base.scene, t.body)[1]
    return _rotm_xyzw(objects[t.body].quat_xyzw)


def _control_step(objects, base=None, origin=None) -> None:
    """Once per control step: who holds each fitting, and whether a free one bites onto a target."""
    for name, spec in _CABLES.items():
        fit = spec.end
        if fit is None or _key(spec) not in objects:
            continue
        key = _key(spec)
        a = _att(key)
        s = objects[key]
        R = _rotm_xyzw(s.quat_xyzw)
        jaw = np.asarray(s.pos, float) + R @ fit.jaw_point
        _JAW[key] = jaw
        a.held = any(g in objects and _holds(objects, g, s, fit) for g in GRIPPERS)
        if a.target is not None:
            if a.held and fit.kind == "clamp":  # squeezed open: it lets go
                _attach(a, None)
            continue
        near = None
        for t in fit.targets:
            g = _target_world(objects, base, t, origin)
            if g is None or not in_target(t, jaw, g, None if t.body is None else _target_rot(objects, base, t)):
                continue
            if t.facing is not None:
                f = np.asarray(t.facing, float)
                if t.body is not None:
                    f = _target_rot(objects, base, t) @ f
                if R[:, 0] @ f < math.cos(math.radians(t.cone)) * np.linalg.norm(f):
                    continue
            near = t
            break
        if near is None:
            a.dwell = 0
            continue
        a.dwell += 1
        if fit.kind == "plug":
            if a.dwell * 0.05 >= fit.dwell - 1e-9:
                _attach(a, near)
        elif not a.held:
            _attach(a, near)
    if os.environ.get("CABLEKIT_ARMS") and base is not None and _step_no() in (150, 200):
        for robot in ("left_robot", "right_robot"):
            art = base.scene[robot]
            bp = _np(art.data.body_pos_w)[0] - origin
            print(f"cablekit arm {robot} step {_step_no()}: " + ", ".join(
                f"{n}={np.round(bp[i], 3).tolist()}" for i, n in enumerate(art.body_names)), flush=True)
    if os.environ.get("CABLEKIT_DEBUG") and _step_no() % 10 == 0:
        for name, spec in _CABLES.items():
            if spec.end is not None and _key(spec) in _JAW:
                a = _att(_key(spec))
                fs = objects[_key(spec)]
                gp = {g[0]: np.round(objects[g].pos, 3).tolist() for g in GRIPPERS if g in objects}
                print(f"cablekit step {_step_no()} {name}: jaw={np.round(_JAW[_key(spec)], 3).tolist()} "
                      f"axis={np.round(_rotm_xyzw(fs.quat_xyzw)[:, 0], 2).tolist()} "
                      f"held={a.held} on={a.target.name if a.target else None} force={a.force:.2f} "
                      f"substeps={_STATS['substeps']} tcp={gp}", flush=True)


# -- hooks ------------------------------------------------------------------------------------------


def _patch_isaaclab() -> None:
    from isaaclab.scene import InteractiveScene

    if getattr(InteractiveScene, "_cablekit", False):
        return
    InteractiveScene._cablekit = True
    write, reset = InteractiveScene.write_data_to_sim, InteractiveScene.reset

    def write_data_to_sim(self):
        if _CABLES:
            _substep(self)
        return write(self)

    def reset_scene(self, env_ids=None):
        out = reset(self, env_ids)
        if _CABLES:
            _reset_cables(self)
        return out

    InteractiveScene.write_data_to_sim, InteractiveScene.reset = write_data_to_sim, reset_scene


_orig_patch = kit._patch


def _patch(scene) -> None:
    _orig_patch(scene)
    if getattr(scene, "_cablekit", False):
        return
    scene._cablekit = True
    _patch_isaaclab()
    inner_task_object, inner_states = scene.task_object, scene.object_states

    def task_object(spec):
        if isinstance(spec, Cable):
            _CABLES[spec.name] = spec
            return _arena_object(scene, spec)
        if getattr(spec, "mass", None) is not None:
            _MOVABLE.add(spec.name)
        return inner_task_object(spec)

    def object_states(env, names):
        out = inner_states(env, [n for n in names if n not in _CABLES])
        base = env.unwrapped
        origin = _np(base.scene.env_origins)[0]
        _ORIGIN[:] = origin
        for n in names:
            if n in _CABLES:
                out.update(_cable_states(base, n, origin))
        _control_step(out, base, origin)
        return out

    scene.task_object, scene.object_states = task_object, object_states


kit._patch = _patch
if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])


# -- scorer API ------------------------------------------------------------------------------------


def points(objects, cable: str) -> np.ndarray:
    """The cable's joint points from start to end, (segments + 1, 3)."""
    return np.asarray(objects[cable].points, float)


def jaw(objects, key: str) -> np.ndarray:
    """The fitting's jaw point (world, as the scorer's positions)."""
    spec = _CABLES[key.split("/")[0]]
    s = objects[key]
    return np.asarray(s.pos, float) + _rotm_xyzw(s.quat_xyzw) @ spec.end.jaw_point


def attached(objects, key: str) -> str | None:
    """The name of the target the fitting is attached to now, or None."""
    t = _att(key).target
    return None if t is None else t.name


def attached_since(objects, key: str) -> int | None:
    """The control step (this trial) the fitting's current attachment began, or None."""
    return _att(key).since


def attach_log(objects, key: str) -> list:
    """Every attachment this trial: [target, start step, end step or None]."""
    return [tuple(e) for e in _att(key).log]


def held(objects, key: str) -> bool:
    """A gripper holds the fitting (see `_holds`)."""
    return _att(key).held


def near(objects, key: str, p, r: float) -> bool:
    """Some point of the fitting's axis, from its back to its jaw tip, is within r of world point p."""
    fit = _CABLES[key.split("/")[0]].end
    s = objects[key]
    R = _rotm_xyzw(s.quat_xyzw)
    a = np.asarray(s.pos, float) + R @ fit.dz
    b = np.asarray(s.pos, float) + R @ (np.array([fit.size[0], 0.0, 0.0]) + fit.dz)
    return _seg_dist(np.asarray(p, float), a, b) < r


def origin() -> np.ndarray:
    """Env 0's origin in world coordinates (add it to scorer positions to place a visual marker)."""
    return _ORIGIN.copy()


def length(cable: str) -> float:
    return _CABLES[cable].length


def _project(pts: np.ndarray, p) -> tuple[float, float]:
    p = np.asarray(p, float)
    best, s_best, s0 = float("inf"), 0.0, 0.0
    for a, b in zip(pts[:-1], pts[1:]):
        d = b - a
        L = float(np.linalg.norm(d))
        u = 0.0 if L < 1e-12 else max(0.0, min(1.0, float((p - a) @ d) / (L * L)))
        dist = float(np.linalg.norm(a + u * d - p))
        if dist < best:
            best, s_best = dist, s0 + u * L
        s0 += L
    return best, s_best


def closest(objects, cable: str, p) -> tuple[float, float]:
    """(distance from world point p to the cable, arc length from the cable's start to the closest point)."""
    return _project(points(objects, cable), p)


def length_between(objects, cable: str, a, b) -> float:
    """The length along the cable between its closest points to world points a and b."""
    pts = points(objects, cable)
    return abs(_project(pts, a)[1] - _project(pts, b)[1])


def span(objects, cable: str) -> float:
    """The straight distance between the cable's two ends."""
    pts = points(objects, cable)
    return float(np.linalg.norm(pts[-1] - pts[0]))


# -- offline tests -------------------------------------------------------------------------------------


def register(*cables: Cable) -> None:
    """Make cables known to the scorer API without a simulator."""
    for c in cables:
        _CABLES[c.name] = c


def set_attached(key: str, target: str | None) -> None:
    """Offline tests: attach the fitting to a target (by name) or detach it, as of now."""
    spec = _CABLES[key.split("/")[0]]
    a = _att(key)
    if (a.target.name if a.target else None) != target:
        _attach(a, None if target is None else _target(spec, target))


def set_held(key: str, value: bool) -> None:
    _att(key).held = value


def state_from_points(cable: str, pts) -> CableState:
    pts = np.asarray(pts, float)
    return CableState(pos=pts.mean(0), quat_xyzw=np.array([0.0, 0, 0, 1]), lin_vel=np.zeros(3), ang_vel=np.zeros(3),
                      low=pts.min(0), high=pts.max(0), points=pts)


# -- demo helpers -------------------------------------------------------------------------------------


def grip_yaw_across(axis) -> float:
    """The yaw of a straight-down grip whose fingers close across a part lying along `axis` (yaw 0 closes along y)."""
    return math.atan2(float(axis[1]), float(axis[0]))
