"""handkit: contact between the YAM grippers and a part, on the robot's own collision hulls.

This folder ships no robot geometry. The hulls come from the competition environment at run time:
yam_arena.analysis.link_hulls() reads every <collision> hull of the robot model the simulator loads. Here the gripper
link's hulls and both fingertips' are sampled into surface points (at most SPACING apart on each face) once per run,
thinned to one per SPACING cube and kept in memory, then placed each step from the gripper's pose and its fingers'
opening (URDF kinematics of the finger joints).

    hk.Cyl(center, axis, r, z0, z1): a solid cylinder in a part's own frame (axis a unit vector through center).
    hk.Box(center, size): a box in a part's own frame.
    hk.gap(objects, g, part, shapes, links): the least distance (m) from gripper g's hulls (all its links, or the
        named ones) to any of `shapes` on objects[part]; negative when they overlap.
    hk.touching(objects, g, part, shapes): within CONTACT (1 mm).
    hk.pinching(objects, g, part, shapes): both fingertips within CONTACT.
"""

from __future__ import annotations

import functools
from dataclasses import dataclass

import numpy as np
import yamkit as kit

CONTACT = 0.001  # m: touching
SPACING = 0.002  # m: surface sample step on each hull face
LINKS = ("gripper", "tip_left", "tip_right")


@dataclass(frozen=True)
class Cyl:
    center: tuple
    axis: tuple
    r: float
    z0: float  # along `axis` from `center`
    z1: float


@dataclass(frozen=True)
class Box:
    center: tuple
    size: tuple


def _sample(tri: np.ndarray) -> np.ndarray:
    """Points on triangles (f x 3 x 3) at most about SPACING apart, corners included."""
    out = [tri.reshape(-1, 3)]
    for a, b, c in tri:
        n = int(np.ceil(max(np.linalg.norm(b - a), np.linalg.norm(c - a), np.linalg.norm(c - b)) / SPACING))
        if n < 1:
            continue
        i, j = np.meshgrid(np.arange(n + 1), np.arange(n + 1), indexing="ij")
        keep = (i + j) <= n
        u, v = i[keep] / n, j[keep] / n
        out.append(a + np.outer(u, b - a) + np.outer(v, c - a))
    return np.concatenate(out)


@functools.lru_cache(maxsize=None)
def link_points(link: str) -> np.ndarray:
    """Surface points (n x 3, the link's frame) of `link`'s collision hulls, from the environment at run time."""
    from scipy.spatial import ConvexHull
    from yam_arena.analysis import link_hulls

    pts = []
    for h in link_hulls()[link]:
        v = np.asarray(h.vertices, float)
        pts.append(_sample(v[ConvexHull(v).simplices]))
    p = np.concatenate(pts)
    _, keep = np.unique(np.floor(p / SPACING).astype(np.int64), axis=0, return_index=True)  # one per SPACING cell
    return p[np.sort(keep)]


@functools.lru_cache(maxsize=256)
def _fingers(q_um: int) -> dict:
    """The fingertips' 4x4 poses in the gripper link's frame with both finger joints at q_um micrometres."""
    from yam_arena import YAM_URDF
    from yam_arena.urdf import forward_kinematics

    q = q_um * 1e-6
    fk = forward_kinematics(YAM_URDF, {"joint7": q, "joint8": q}, root="gripper")
    return {k: fk[k] for k in ("tip_left", "tip_right")}


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def hand_points(objects, g: str, links=LINKS) -> dict:
    """{link: world points} of gripper g's hulls, placed from objects[g] (its tool point and the gripper's rotation)
    and its fingers' opening."""
    from yam_arena.constants import FINGER_OPEN, TCP_OFFSET

    s = objects[g]
    R = _rot(s.quat_xyzw)
    origin = np.asarray(s.pos, float) - R @ np.asarray(TCP_OFFSET, float)
    op = kit.opening(objects, g)
    op = 1.0 if not np.isfinite(op) else float(np.clip(op, 0.0, 1.0))
    tips = _fingers(int(round(op * FINGER_OPEN * 1e6)))
    out = {}
    for link in links:
        p = link_points(link)
        if link != "gripper":
            T = tips[link]
            p = p @ T[:3, :3].T + T[:3, 3]
        out[link] = p @ R.T + origin
    return out


def _dist(p: np.ndarray, shape) -> np.ndarray:
    """Signed distance of points p (n x 3, the part's frame) to a shape (negative inside)."""
    if isinstance(shape, Box):
        q = np.abs(p - np.asarray(shape.center, float)) - np.asarray(shape.size, float) / 2
        return np.linalg.norm(np.maximum(q, 0.0), axis=1) + np.minimum(q.max(axis=1), 0.0)
    a = np.asarray(shape.axis, float)
    d = p - np.asarray(shape.center, float)
    t = d @ a
    rho = np.linalg.norm(d - np.outer(t, a), axis=1)
    dr = rho - shape.r
    dz = np.maximum(shape.z0 - t, t - shape.z1)
    return np.hypot(np.maximum(dr, 0.0), np.maximum(dz, 0.0)) + np.minimum(np.maximum(dr, dz), 0.0)


def gap(objects, g: str, part: str, shapes, links=LINKS, reach: float = 0.75) -> dict:
    """{link: least distance (m) from that link's hulls to any of `shapes` on objects[part]} for gripper g; a link
    whose tool point is more than `reach` from the part is not measured (inf)."""
    if g not in objects or part not in objects:
        return {k: np.inf for k in links}
    so = objects[part]
    co, Ro = np.asarray(so.pos, float), _rot(so.quat_xyzw)
    if np.linalg.norm(np.asarray(objects[g].pos, float) - co) > reach:
        return {k: np.inf for k in links}
    out = {}
    for link, p in hand_points(objects, g, links).items():
        local = (p - co) @ Ro
        out[link] = float(min(_dist(local, s).min() for s in shapes))
    return out


def touching(objects, g: str, part: str, shapes, tol: float = CONTACT) -> bool:
    return min(gap(objects, g, part, shapes).values()) <= tol


def pinching(objects, g: str, part: str, shapes, tol: float = CONTACT) -> bool:
    gp = gap(objects, g, part, shapes, links=("tip_left", "tip_right"))
    return gp["tip_left"] <= tol and gp["tip_right"] <= tol
