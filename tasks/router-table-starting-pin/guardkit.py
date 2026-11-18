"""guardkit: shared scoring guards, trimmed to the helpers this task uses. Import after yamkit:
    import guardkit as gk

Parts are shapes in an object's own frame (origin at objects[obj].pos, turning with its quat), so a check
follows the object however it lies.

    gk.Block: A box centered at `center` with full `size` (object frame, m).
    gk.touching: Grippers in contact with `obj`: their own shape (palm and both fingers, opened as they are now)
        touches or overlaps one of `parts` (within `tol`, 2 mm by default). A hand hovering more than `tol` from a
        part is not touching it.
"""

import functools
from dataclasses import dataclass
import numpy as np
import yamkit as kit


GRIPPERS = ("left_gripper", "right_gripper")


@dataclass(frozen=True)
class Seg:
    """A capsule from `a` to `b` (object frame, m) with radius `r`: a handle, neck, rim or rod. a == b is a ball."""

    a: tuple
    b: tuple
    r: float = 0.0


@dataclass(frozen=True)
class Ring:
    """A solid ring about the object's z axis (object frame, m): radii `r_in`..`r_out`, heights `z0`..`z1`, centered on
    `center`. r_in = 0 gives a solid cylinder (a tart, a disk, a piston)."""

    r_in: float
    r_out: float
    z0: float
    z1: float
    center: tuple = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Block:
    """A box centered at `center` with full `size` (object frame, m)."""

    center: tuple
    size: tuple


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _dist_local(part, p) -> float:
    if isinstance(part, Seg):
        a, b = np.asarray(part.a, float), np.asarray(part.b, float)
        ab = b - a
        L2 = float(ab @ ab)
        t = 0.0 if L2 < 1e-18 else float(np.clip((p - a) @ ab / L2, 0.0, 1.0))
        return max(0.0, float(np.linalg.norm(p - (a + t * ab))) - part.r)
    d = np.abs(p - np.asarray(part.center, float)) - np.asarray(part.size, float) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def _proj_local(part, q: np.ndarray) -> np.ndarray:
    """The points of `part` nearest to object-frame points q (n x 3)."""
    if isinstance(part, Block):
        c, h = np.asarray(part.center, float), np.asarray(part.size, float) / 2
        return c + np.clip(q - c, -h, h)
    if isinstance(part, Ring):
        c = np.asarray(part.center, float)
        d = q - c
        rho = np.hypot(d[:, 0], d[:, 1])
        safe = np.where(rho > 1e-12, rho, 1.0)
        ux, uy = np.where(rho > 1e-12, d[:, 0] / safe, 1.0), np.where(rho > 1e-12, d[:, 1] / safe, 0.0)
        r = np.clip(rho, part.r_in, part.r_out)
        return c + np.stack([r * ux, r * uy, np.clip(d[:, 2], part.z0, part.z1)], axis=1)
    a, b = np.asarray(part.a, float), np.asarray(part.b, float)
    ab = b - a
    L2 = float(ab @ ab)
    t = np.zeros(len(q)) if L2 < 1e-18 else np.clip((q - a) @ ab / L2, 0.0, 1.0)
    x = a + t[:, None] * ab
    v = q - x
    n = np.linalg.norm(v, axis=1)
    return x + v * np.minimum(1.0, part.r / np.maximum(n, 1e-12))[:, None]


def _dist_local_any(part, p) -> float:
    if isinstance(part, Ring):
        return float(np.linalg.norm(p - _proj_local(part, p[None, :])[0]))
    return _dist_local(part, p)


# The YAM gripper's own shape in its tool-point frame (x: the fingers' closing axis, z: the approach, toward the pad
# tips), as boxes fitted at run time to the robot's collision hulls from the competition environment (yamgeom,
# yam_arm.urdf's gripper, tip_left and tip_right): the palm, and for each finger its body, its pad and its narrower
# tip, and its carriage. `o` is each finger's inner face's distance from the tool point: 0 closed, 47 mm fully open.
FINGER_SLABS = ((None, -0.021), (-0.021, -0.003), (-0.003, None))  # m along z: the finger's body, pad and tip


CONTACT = 0.002  # m: touching: the shapes within 2 mm (the solver's resting gap plus the box fit to the meshes)


@functools.lru_cache(maxsize=1)
def _hand_fit() -> tuple:
    """(palm, ((side, (body, pad, tip, carriage)), ...)) with the gripper closed: the palm is the box around the
    gripper link's hull points (102 x 65 x 74 mm); each finger's body, pad and tip hold its hull points in the
    FINGER_SLABS on its own side of the tool point (24 x 68 x 46, 10 x 40 x 18 and 6 x 26 x 16 mm), and its carriage
    the part that reaches across the tool point near the palm (48 x 26 x 20 mm), so every hull point is in a box."""
    import yamgeom as yg
    from yam_arena.constants import TCP_OFFSET

    palm = Block(*yg.box(yg.points("gripper") - np.asarray(TCP_OFFSET, float)))
    fingers = []
    for tip in ("tip_left", "tip_right"):
        p = yg.tcp_points(tip)
        s = 1.0 if yg.box(p, (None, None, FINGER_SLABS[2][0]))[0][0] > 0 else -1.0  # the side its tip is on
        own = p[p[:, 0] * s >= 0.0]
        boxes = [yg.box(own, (None, None, lo), (None, None, hi)) for lo, hi in FINGER_SLABS]
        if (p[:, 0] * s < 0.0).any():
            boxes.append(yg.box(p[p[:, 0] * s < 0.0]))  # the carriage
        fingers.append((s, tuple(boxes)))
    return palm, tuple(sorted(fingers))


def hand_boxes(o: float) -> list:
    palm, fingers = _hand_fit()
    out = [palm]
    for s, boxes in fingers:
        out += [Block((c[0] + s * o, c[1], c[2]), size) for c, size in boxes]
    return out


def _box_gap(cg, Rg, box, co, Ro, part, iters: int = 40) -> float:
    """Gap (m, 0 if they overlap) between a gripper box (gripper frame cg, Rg) and an object part (object frame co,
    Ro), by alternating projection from the box's center and 8 corners (exact for convex parts; the corners keep a
    ring's hollow from trapping it)."""
    c, h = np.asarray(box.center, float), np.asarray(box.size, float) / 2
    corners = np.array([[sx, sy, sz] for sx in (-1, 1) for sy in (-1, 1) for sz in (-1, 1)] + [[0, 0, 0]], float)
    p = cg + (c + corners * h) @ Rg.T  # world points in the gripper box
    q = p
    for _ in range(iters):
        q = co + _proj_local(part, (p - co) @ Ro) @ Ro.T  # nearest on the part
        p_new = cg + (c + np.clip((q - cg) @ Rg - c, -h, h)) @ Rg.T  # nearest on the box
        if np.max(np.abs(p_new - p)) < 1e-6:
            p = p_new
            break
        p = p_new
    return float(np.min(np.linalg.norm(p - q, axis=1)))


def _hand_frames(objects, g: str, obj: str):
    from yam_arena.constants import FINGER_OPEN

    op = kit.opening(objects, g)
    o = abs(FINGER_OPEN) * (1.0 if not np.isfinite(op) else float(np.clip(op, 0.0, 1.0)))
    s, so = objects[g], objects[obj]
    return (o, np.asarray(s.pos, float), _rot(s.quat_xyzw), np.asarray(so.pos, float), _rot(so.quat_xyzw))


def touching(objects, obj: str, parts, tol: float = CONTACT, grippers=GRIPPERS) -> tuple:
    """Grippers in contact with `obj`: their own shape (palm and both fingers, opened as they are now) touches or
    overlaps one of `parts` (within `tol`, 2 mm by default). A hand hovering more than `tol` from a part is not
    touching it."""
    out = []
    for g in grippers:
        if g not in objects:
            continue
        o, cg, Rg, co, Ro = _hand_frames(objects, g, obj)
        hit = False
        for part in (parts if isinstance(parts, (list, tuple)) else (parts,)):
            for box in hand_boxes(o):
                ctr = cg + Rg @ np.asarray(box.center, float)
                reach = float(np.linalg.norm(np.asarray(box.size, float))) / 2
                if _dist_local_any(part, Ro.T @ (ctr - co)) - reach > tol:
                    continue
                if _box_gap(cg, Rg, box, co, Ro, part) <= tol:
                    hit = True
                    break
            if hit:
                break
        if hit:
            out.append(g)
    return tuple(out)
