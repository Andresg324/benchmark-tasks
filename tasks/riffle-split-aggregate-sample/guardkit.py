"""guardkit: shared scoring guards for the holes reviewers kept finding (Nov 11). Import after yamkit:
    import guardkit as gk

Parts are shapes in an object's own frame (origin at objects[obj].pos, turning with its quat), so a check follows the
object however it lies: gk.Seg(a, b, r) is a capsule (a handle, neck or rod of radius r; a == b gives a ball) and
gk.Block(center, size) a box. With parts=None a check uses the object's world box (low/high), which over-covers a
tilted object (strict for release checks, loose for grip checks: pass parts for those).

Stateless checks (call any time; they read only this step's objects and kit.opening):
    gk.holding(o, "scoop", PARTS)            -> grippers with the tool point within 4 cm of ANY listed part and the
                                                fingers closed on something (not wide open, not an empty fist)
    gk.grip_on(o, "jug", HANDLE, gk.band(0.025))  -> grippers whose opening is stopped by that part (a real grip)
    gk.released_clear(o, "jug", PARTS, 0.01) -> True when every tool point and pad tip is 1 cm clear of the outline
Per-trial memory (reset when kit.episode() changes; updated once per control step even though success() and score()
both call them; pass a fresh objects dict per step in offline tests):
    FLIP = gk.Latch(steps=2)                 -> FLIP.feed(o, cond): an irreversible event, 2 steps in a row confirm it
    ONE = gk.Latch(steps=3, total=True)      -> counts steps in all: swapping hands every 2 steps doesn't dodge it
    HOME = gk.ArmsHome()                     -> HOME.feed(o) every step; HOME.home(o): back at the start pose, still
    SW = gk.Sweep()                          -> SW.crossed(o, "knife", tip, test, obj="board"): tests the points
                                                between last step's and this step's tip (fast motions)
Typical end-of-run rule: full credit only if gk.released_clear(...) and HOME.home(o) and not FAIL.on.
Scored geometry is the thing a real gauge reads (the pipe's own slope, the deck's centre): put that in the task.
"""

import math
from dataclasses import dataclass

import numpy as np

import yamkit as kit

GRIPPERS = ("left_gripper", "right_gripper")
NEAR = 0.04  # m: tool point within 4 cm of a part = a hand still on it (the tool point sits between the pads)
HELD_OPEN = (0.02, 0.85)  # opening (0 closed .. 1 open) of fingers closed on something: an empty fist reads 0
GRIP_PAD = 0.01  # m: for grip_on, the tool point within 1 cm of the gripped part
BAND = 0.15  # band(): fingers may squeeze up to 15 % of full opening narrower than the part ...
OVER = 0.03  # ... and be at most 3 % wider (an open hand hovering around it is not a grip)


# -- geometry ----------------------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Seg:
    """A capsule from `a` to `b` (object frame, m) with radius `r`: a handle, neck, rim or rod. a == b is a ball."""

    a: tuple
    b: tuple
    r: float = 0.0


@dataclass(frozen=True)
class Block:
    """A box centred at `center` with full `size` (object frame, m)."""

    center: tuple
    size: tuple


def _rot(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def local(objects, obj: str, p) -> np.ndarray:
    """World point `p` in object `obj`'s own frame."""
    s = objects[obj]
    return _rot(s.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(s.pos, float))


def _dist_local(part, p) -> float:
    if isinstance(part, Seg):
        a, b = np.asarray(part.a, float), np.asarray(part.b, float)
        ab = b - a
        L2 = float(ab @ ab)
        t = 0.0 if L2 < 1e-18 else float(np.clip((p - a) @ ab / L2, 0.0, 1.0))
        return max(0.0, float(np.linalg.norm(p - (a + t * ab))) - part.r)
    d = np.abs(p - np.asarray(part.center, float)) - np.asarray(part.size, float) / 2
    return float(np.linalg.norm(np.maximum(d, 0.0)))


def dist(objects, obj: str, p, parts=None) -> float:
    """Distance (m) from world point `p` to the nearest of `parts` of `obj` (0 inside); parts=None: the object's
    world box (low/high)."""
    if parts is None:
        s = objects[obj]
        d = np.maximum(np.asarray(s.low, float) - p, np.asarray(p, float) - np.asarray(s.high, float))
        return float(np.linalg.norm(np.maximum(d, 0.0)))
    q = local(objects, obj, p)
    return min(_dist_local(part, q) for part in (parts if isinstance(parts, (list, tuple)) else (parts,)))


def tool_points(objects, g: str) -> tuple:
    """A gripper's tool point (between the pads) and its pad tips' point."""
    from yam_arena.constants import PAD_TIP_PAST_TCP

    s = objects[g]
    tcp = np.asarray(s.pos, float)
    return tcp, tcp + _rot(s.quat_xyzw) @ np.array([0.0, 0.0, PAD_TIP_PAST_TCP])


def band(width: float, squeeze: float = BAND, over: float = OVER) -> tuple:
    """The opening band of fingers closed on a part `width` m across: from the width's opening minus `squeeze`
    (never below half of it, so an empty fist next to a thin part is not a grip) to plus `over`."""
    from yam_arena.constants import FINGER_OPEN

    want = width / (2 * abs(FINGER_OPEN))
    return want - min(squeeze, want / 2), want + over


# -- stateless checks --------------------------------------------------------------------------------------------------


def holding(objects, obj: str, parts=None, near: float = NEAR, open_band: tuple = HELD_OPEN, grippers=GRIPPERS) -> tuple:
    """Grippers holding `obj`: the tool point within `near` of ANY of `parts` (not just the handle's centre: a hand
    on the neck or bowl holds it too) and the fingers closed on something (opening inside `open_band`). Falsy if none."""
    return tuple(g for g in grippers if g in objects and open_band[0] <= kit.opening(objects, g) <= open_band[1]
                 and dist(objects, obj, objects[g].pos, parts) < near)


def grip_on(objects, obj: str, part, open_band: tuple, pad: float = GRIP_PAD, grippers=GRIPPERS) -> tuple:
    """Grippers with a real grip on `part` of `obj`: the opening is stopped by the part (inside `open_band`, see
    band(); a closed fist reads 0, an open hand ~1) and the tool point within `pad` of it. Falsy if none."""
    return tuple(g for g in grippers if g in objects and open_band[0] <= kit.opening(objects, g) <= open_band[1]
                 and dist(objects, obj, objects[g].pos, part) <= pad)


def released_clear(objects, obj: str, parts=None, margin: float = 0.01, open_ok: float | None = None,
                   grippers=GRIPPERS) -> bool:
    """Let go: every gripper's tool point and pad tip at least `margin` clear of `obj`'s outline (its parts, or its
    world box). With `open_ok` (e.g. 0.85) a gripper open wider than that is not counted (a wide-open hand resting
    against the object); by default even an open hand must be clear."""
    for g in grippers:
        if g not in objects or (open_ok is not None and kit.opening(objects, g) > open_ok):
            continue
        if any(dist(objects, obj, p, parts) < margin for p in tool_points(objects, g)):
            return False
    return True


# -- per-trial memory --------------------------------------------------------------------------------------------------


class _PerStep:
    """Resets when kit.episode() changes; _tick() is True the first time it is called in a control step (keyed on
    yamkit's step index in the sim, offline on the objects dict)."""

    def __init__(self):
        self._ep, self._key, self._objs, self.step = None, None, None, 0
        self._fresh()

    def _fresh(self):
        pass

    def _tick(self, objects) -> bool:
        ep, n = kit.episode(), kit._EPISODE.get("last")
        if ep != self._ep:
            self._ep, self._key, self._objs, self.step = ep, None, None, 0
            self._fresh()
        key = (ep, n) if n is not None else None
        if (key is not None and key == self._key) or (key is None and objects is self._objs):
            return False
        self._key, self._objs = key, objects
        self.step += 1
        return True


class Latch(_PerStep):
    """An irreversible event: once `cond` holds for `steps` control steps in a row (default 2: a one-step physics pop
    does not count, a quick flick undone at once does), it stays on for the rest of the trial. total=True counts the
    steps in all instead of in a row (swapping hands every 2 steps, or toggling, can't dodge it)."""

    def __init__(self, steps: int = 2, total: bool = False):
        self.steps, self.total = steps, total
        super().__init__()

    def _fresh(self):
        self.run, self.count, self.on, self.when = 0, 0, False, None

    def feed(self, objects, cond: bool) -> bool:
        """Feed this step's condition (once per step; repeats within a step are ignored). Returns whether latched."""
        if self._tick(objects):
            self.run = self.run + 1 if cond else 0
            self.count += bool(cond)
            if not self.on and (self.count if self.total else self.run) >= self.steps:
                self.on, self.when = True, self.step
        return self.on


class ArmsHome(_PerStep):
    """Both arms back where they started: each gripper's tool point within `tol` of its pose at the trial's first
    step and moving less than `still` m per step for `hold` steps in a row (so an arm passing through home or still
    retreating does not count)."""

    def __init__(self, tol: float = 0.03, still: float = 0.002, hold: int = 1, grippers=GRIPPERS):
        self.tol, self.still, self.hold, self.grippers = tol, still, hold, grippers
        super().__init__()

    def _fresh(self):
        self.start, self.last, self.still_run = None, None, 0

    def feed(self, objects) -> None:
        if not self._tick(objects):
            return
        now = {g: np.asarray(objects[g].pos, float) for g in self.grippers if g in objects}
        if self.start is None and len(now) == len(self.grippers):
            self.start = now
        moved = self.last is None or any(g not in self.last or np.linalg.norm(p - self.last[g]) >= self.still
                                         for g, p in now.items())
        self.still_run = 0 if moved else self.still_run + 1
        self.last = now

    def home(self, objects) -> bool:
        self.feed(objects)
        return (self.start is not None and self.still_run >= self.hold
                and all(g in objects and np.linalg.norm(np.asarray(objects[g].pos, float) - self.start[g]) < self.tol
                        for g in self.grippers))


def swept(p0, p1, test, step: float = 0.005) -> bool:
    """True if test(point) holds anywhere on the segment p0 -> p1, checked every `step` m (both ends included)."""
    p0, p1 = np.asarray(p0, float), np.asarray(p1, float)
    n = max(1, int(math.ceil(np.linalg.norm(p1 - p0) / step)))
    return any(test(p0 + (p1 - p0) * k / n) for k in range(n + 1))


class Sweep(_PerStep):
    """Between-step contact for fast motions: a tool point moving 3 cm in one 0.05 s step can jump over a thin target.
    crossed() tests the points between where the point was last step and where it is now. With `obj`, both are taken
    in that object's frame (motion relative to a moving object; `test` then gets object-frame points). A jump longer
    than `max_jump` (a reset, a teleport, a physics pop) tests only the current point."""

    def __init__(self, step: float = 0.005, max_jump: float = 0.05):
        self.res, self.max_jump = step, max_jump
        super().__init__()

    def _fresh(self):
        self.prev, self.cur, self.hit = {}, {}, {}

    def crossed(self, objects, key: str, point, test, obj: str | None = None) -> bool:
        if self._tick(objects):
            self.prev, self.hit = self.cur, {}
            self.cur = {}
        if key in self.hit:
            return self.hit[key]
        p = local(objects, obj, point) if obj is not None else np.asarray(point, float)
        self.cur[key] = p
        q = self.prev.get(key)
        if q is None or np.linalg.norm(p - q) > self.max_jump:
            q = p
        self.hit[key] = swept(q, p, test, self.res)
        return self.hit[key]
