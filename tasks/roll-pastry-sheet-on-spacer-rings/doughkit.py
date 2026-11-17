"""doughkit: a piece of dough rolled out under a rolling pin with spacer rings (Nov 11; Nov 15: a sheet, read on a
2 cm grid). Import after yamkit:
    import doughkit as dk

The dough is modeled, not simulated: it has no collider, so the pin rides on its rings on the board and the kit works
out what the barrel does to the dough under it. The sheet is a row of strips along y (one per `cell` of width); each
strip is a chain of thin slices along x, each with a fixed volume (per unit width) and a thickness, so its length is
volume / thickness and the total volume never changes.

Each control step, for each strip, the kit finds where the barrel's lowest line crosses that strip (x) and how high
it is above the board there (the gap: the rings' clearance, plus however far that end of the pin is lifted, so a
tilted pin leaves a wedge). A slice whose center the line rolls across, with the gap below the slice's thickness, is
pressed as the barrel's face reaches it (its lowest line still sqrt(2 R (t - gap)) short of the slice's center,
the contact patch of a round barrel on thick dough).

Press law (Nov 17: force-gated, elastic then rigid-plastic): the dough is elastic up to a yield strain YIELD_STRAIN
(5 %, an estimate) and then flows at its flow stress `stress`. A press squeezing h = t - gap off a slice t thick
loads that slice's strip (one `cell` wide) with F = p w a: a = sqrt(2 R h) the barrel's contact length, w the cell,
and p = stress x min(1, h / (YIELD_STRAIN t)) the contact pressure (elastic below the yield squeeze, the flow stress
above it). Below the yield load F_y = stress w sqrt(2 R YIELD_STRAIN t) the strain is elastic and the slice springs
fully back: no permanent thinning, however many passes. Above it the slice thins by
    dt = (1 - RATIO) (h - YIELD_STRAIN t),   h = F^2 / (2 R stress^2 w^2)
(RATIO of the plastic squeeze springs back, so it takes several passes). The thinning per pass is set by the load the
barrel puts on the strip: in the simulator the barrel's height is where the pin settles under the arms' push, its
weight, the rings on the board and this push back, so a pin that is not pushed down stays high and presses less. A
sheet rolled level with the rings on the board settles at gap / (1 - YIELD_STRAIN), 3.37 mm for the 3.2 mm rings. A pressed slice lengthens; the slices on its far side from the strip's anchor (the piece's
middle, held by the bulk of the dough) move outward to make room. A slice is pressed again only once the line has
gone back REARM behind where the barrel's face meets it (one press per crossing, however the arm jitters). Rolling
past a strip's end presses the end slice like any other: the rings hold the barrel at the gap there too.

Resistance (Nov 16; Nov 17 elastic below yield): the dough pushes back on the pin. For each strip the barrel is in
contact with (dough thicker than the gap within the barrel's contact reach of its lowest line), with h the largest
excess there, t that slice's thickness and a = sqrt(2 R h) the contact length: an upward force p x cell x a (p as in
the press law: elastic below the yield squeeze, the flow stress above it) and, while the line moves, a force against
the motion STRESS x cell x (h - YIELD_STRAIN t) (the plastic work of squeezing the dough down, per meter rolled; none
while it is only elastic). They act at the contact, so a pin pressed harder at one end feels the thicker
side push back harder, and they are worked out from the pin's pose and applied to it as an external force and torque
before every physics step (the press itself still happens once per control step).
`stress` is set by the task (Pin.stress, Pa; 0 turns it off).

    dk.setup(dk.Sheet(...), dk.Pin(...))    once, at import time of task.py
    dk.update(objects)                       each control step (the hook calls it; offline tests call it)
    dk.grid() -> t                           thickness (m, 0 = no dough) at the grid's cell centers
    dk.lengths() -> per-strip lengths (m); dk.volume() total volume (m^3); dk.ends() per-strip (x_min, x_max)
All of it resets when kit.episode() changes.

Visuals: one abutting slab per covered grid cell, as thick as the dough there, all one dough color (Isaac Lab
VisualizationMarkers, no physics), so the top camera shows the sheet growing.
"""

import math
import sys
from dataclasses import dataclass

import numpy as np

import yamkit as kit

RATIO = 0.5  # fraction of the plastic squeeze (beyond the yield strain) that springs back after one press
YIELD_STRAIN = 0.05  # the dough's elastic limit in compression (an estimate): a squeeze under 5 % of the thickness
#                      springs back fully
REARM = 0.01  # m: the line must go this far back past a slice before it can press it again


@dataclass(frozen=True)
class Sheet:
    """The dough: a piece `size` = (length along x, width along y, thickness) centered at (cx, cy) on a board whose top
    is at world z `board_z`; displayed and scored on `cell` cells over x in `x_range` (world m)."""

    center: tuple
    size: tuple
    board_z: float
    x_range: tuple
    cell: float = 0.01
    slice: float = 0.0025  # m: initial slice length along x


@dataclass(frozen=True)
class Pin:
    """The rolling pin `obj`: a barrel of `radius` along the object's local `axis` (unit), centered at its origin, `length`
    long between the rings."""

    obj: str
    radius: float
    length: float
    axis: tuple = (0.0, 1.0, 0.0)
    stress: float = 0.0  # Pa: the dough's flow stress against the barrel (0: no resistance)


_CFG = {}
_S = None
_VIS = {"markers": None, "failed": False, "index": None}


class _State:
    def __init__(self):
        sh = _CFG["sheet"]
        self.episode = kit.episode()
        n = max(1, int(round(sh.size[0] / sh.slice)))
        ny = max(1, int(round(sh.size[1] / sh.cell)))
        self.ys = sh.center[1] - sh.size[1] / 2 + (np.arange(ny) + 0.5) * sh.cell
        self.t = np.full((ny, n), float(sh.size[2]))
        self.vol = np.full((ny, n), sh.size[0] / n * sh.size[2])  # volume per unit width (m^2)
        self.armed = np.ones((2, ny, n), bool)  # [0]: may be pressed moving +x; [1]: moving -x
        self.anchor = n // 2  # slice boundary held in place (the piece's middle)
        self.anchor_x = float(sh.center[0])
        self.line = None  # per strip: (x, gap) of the barrel's lowest line last step, or None
        self.step = 0
        self.presses = 0  # control steps in which some slice thinned
        self.wrench = (np.zeros(3), np.zeros(3))  # the dough's force and torque on the pin (world, about its center)
        self.peak = np.zeros(3)  # largest |Fx|, |Fy|, Fz seen this trial (N)


def setup(sheet: Sheet, pin: Pin, before=None) -> None:
    """`before(base)`: called in the simulator each control step before the dough step (e.g. to read link poses)."""
    global _S
    _CFG.update(sheet=sheet, pin=pin, before=before)
    _S = None


def _state() -> _State:
    global _S
    if _S is None or _S.episode != kit.episode():
        _S = _State()
    return _S


def _rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def pin_line(objects, y: float):
    """(x, gap) of the barrel's lowest line where it passes over world y, or None (pin absent or not over y). gap is
    the height of that line above the board top (m)."""
    p, sh = _CFG["pin"], _CFG["sheet"]
    o = objects.get(p.obj) if objects is not None else None
    if o is None:
        return None
    a = _rotm(o.quat_xyzw) @ np.asarray(p.axis, float)
    a = a / np.linalg.norm(a)
    if abs(a[1]) < 0.2:  # pin turned more than ~78 degrees off the y axis: not rolling across the strips
        return None
    s = (y - float(o.pos[1])) / a[1]
    if abs(s) > p.length / 2:
        return None
    c = np.asarray(o.pos, float) + s * a
    down = np.array([0.0, 0.0, -1.0]) - a * (-a[2])  # straight down, less its part along the axis
    nd = np.linalg.norm(down)
    low = c + p.radius * down / nd if nd > 1e-9 else c
    return float(low[0]), float(low[2] - sh.board_z)


def _edges(S, j) -> np.ndarray:
    """Slice boundaries (world x) of strip j, n + 1 of them."""
    ln = S.vol[j] / S.t[j]
    e = np.zeros(len(ln) + 1)
    e[1:] = np.cumsum(ln)
    return e - e[S.anchor] + S.anchor_x


def update(objects) -> None:
    """One control step: press what the pin's line rolled across since the last step."""
    S = _state()
    S.step += 1
    lines = [pin_line(objects, float(y)) for y in S.ys]
    if S.line is None:
        S.line = lines
        _rearm(S, lines)
        return
    t0 = S.t.copy()
    S.wrench = _reaction(S, objects, lines, S.line)
    S.peak = np.maximum(S.peak, np.abs(S.wrench[0]))
    R = _CFG["pin"].radius
    for j, (now, before) in enumerate(zip(lines, S.line)):
        if now is None or before is None:
            continue
        x0, g0 = before
        x1, g1 = now
        g = max(g0, g1)  # the squeeze held through both control steps (a one-step spike, a drop, does not count)
        n = S.t.shape[1]
        e = _edges(S, j)  # recomputed only after a press moves them
        if x1 != x0:  # rolling: each slice is pressed as the barrel's face reaches it
            d = 1 if x1 > x0 else -1
            k = 0 if d > 0 else 1
            for i in (range(n) if d > 0 else range(n - 1, -1, -1)):
                if not S.armed[k, j, i] or g >= S.t[j, i]:
                    continue
                c = 0.5 * (e[i] + e[i + 1])
                hit = c - d * _reach(S.t[j, i], g)  # where the line is when the barrel's face meets the slice's top
                if not (min(x0, x1) < hit <= max(x0, x1)):
                    continue
                S.armed[k, j, i] = False
                dt = thinning(S.t[j, i], S.t[j, i] - g)
                if dt > 0.0:
                    S.t[j, i] -= dt
                    e = _edges(S, j)
        # pressing down in place: a slice under the barrel's face that no crossing has pressed since the barrel last
        # came to it is squeezed by the face's depth over it (less off the lowest line: t - g - s^2 / 2R at s)
        c = 0.5 * (e[:-1] + e[1:])
        h = S.t[j] - g - (c - x1) ** 2 / (2 * R)
        for i in np.flatnonzero((h > 0) & S.armed[0, j] & S.armed[1, j]):
            S.armed[:, j, i] = False
            S.t[j, i] -= thinning(S.t[j, i], float(h[i]))
    if not np.array_equal(S.t, t0):
        S.presses += 1
    S.line = lines
    _rearm(S, lines)


def thinning(t: float, h: float) -> float:
    """Permanent thinning (m) of a slice `t` thick squeezed by `h` (see the press law): none up to the yield strain."""
    return (1.0 - RATIO) * max(0.0, h - YIELD_STRAIN * t)


def pressure(t: float, h: float, stress: float) -> float:
    """The contact pressure (Pa) under the barrel squeezing `h` off dough `t` thick: elastic, rising to the flow stress
    at the yield strain, then the flow stress."""
    if h <= 0.0:
        return 0.0
    return stress * min(1.0, h / (YIELD_STRAIN * t))


def yield_load(t: float, stress: float = None) -> float:
    """The load (N) on one cell-wide strip `t` thick at which the dough under the barrel starts to yield."""
    p, sh = _CFG["pin"], _CFG["sheet"]
    s = p.stress if stress is None else stress
    return s * sh.cell * math.sqrt(2 * p.radius * YIELD_STRAIN * t)


def _reaction(S, objects, lines, before):
    """The dough's force and torque on the pin this step (world frame, torque about the pin's center), from the
    thickness before this step's presses (see Resistance in the module docstring)."""
    p, sh = _CFG["pin"], _CFG["sheet"]
    F, T = np.zeros(3), np.zeros(3)
    o = objects.get(p.obj) if objects is not None else None
    if not p.stress or o is None:
        return F, T
    center = np.asarray(o.pos, float)
    for j, now in enumerate(lines):
        if now is None:
            continue
        x, g = now
        e = _edges(S, j)
        c = 0.5 * (e[:-1] + e[1:])
        ex = S.t[j] - g
        r = np.array([_reach(t, g) for t in S.t[j]])
        touch = (ex > 0) & (np.abs(c - x) <= r + 0.5 * (e[1:] - e[:-1]))
        if not touch.any():
            continue
        i = int(np.flatnonzero(touch)[np.argmax(ex[touch])])
        h, t = float(ex[i]), float(S.t[j, i])
        a = math.sqrt(max(0.0, 2 * p.radius * h - h * h))
        d = 0.0
        if before is not None and before[j] is not None and now[0] != before[j][0]:
            d = 1.0 if now[0] > before[j][0] else -1.0
        plastic = max(0.0, h - YIELD_STRAIN * t)
        f = np.array([-d * p.stress * sh.cell * plastic, 0.0, pressure(t, h, p.stress) * sh.cell * a])
        at = np.array([x + d * 0.5 * a, float(S.ys[j]), sh.board_z + g])  # the middle of the contact, ahead of the line
        F += f
        T += np.cross(at - center, f)
    return F, T


def wrench():
    """The dough's (force, torque) on the pin from the last control step (N, N m; world frame, about its center)."""
    return _state().wrench


def peak():
    """The largest |Fx|, |Fy| and upward Fz the dough put on the pin this trial (N)."""
    return _state().peak.copy()


def _reach(t: float, g: float) -> float:
    """How far ahead of its lowest line the barrel's face first touches dough `t` thick with the line at gap `g`."""
    h = t - g
    if h <= 0:
        return 0.0
    r = _CFG["pin"].radius
    return math.sqrt(max(0.0, 2 * r * h - h * h)) if h < r else r


def _rearm(S, lines) -> None:
    for j, ln in enumerate(lines):
        if ln is None:  # the pin off this strip: the next time it comes down is a new press
            S.armed[:, j] = True
            continue
        S.armed[:, j] |= ln[1] >= S.t[j]  # the barrel lifted clear of a slice: likewise
        e = _edges(S, j)
        c = 0.5 * (e[:-1] + e[1:])
        r = np.array([_reach(t, ln[1]) for t in S.t[j]])  # back behind where the barrel's face meets each slice
        S.armed[0, j] |= ln[0] < c - r - REARM
        S.armed[1, j] |= ln[0] > c + r + REARM


def volume() -> float:
    """Total dough volume (m^3)."""
    S = _state()
    return float(S.vol.sum() * _CFG["sheet"].cell)


def ends() -> np.ndarray:
    """Per strip, (x_min, x_max) of the dough (world m)."""
    S = _state()
    return np.array([[_edges(S, j)[0], _edges(S, j)[-1]] for j in range(S.t.shape[0])])


def lengths() -> np.ndarray:
    e = ends()
    return e[:, 1] - e[:, 0]


def strips():
    """Per strip: (edges, thickness) arrays (copies)."""
    S = _state()
    return [(_edges(S, j), S.t[j].copy()) for j in range(S.t.shape[0])]


def grid():
    """Thickness on the `cell` grid: rows = strips (y), columns = cells along x over Sheet.x_range; t is the thickness
    (m) of the slice under the cell's center, 0 where there is no dough."""
    sh = _CFG["sheet"]
    S = _state()
    nx = int(round((sh.x_range[1] - sh.x_range[0]) / sh.cell))
    xc = sh.x_range[0] + (np.arange(nx) + 0.5) * sh.cell
    t = np.zeros((S.t.shape[0], nx))
    for j in range(S.t.shape[0]):
        e = _edges(S, j)
        i = np.searchsorted(e, xc, side="right") - 1
        ok = (i >= 0) & (i < S.t.shape[1])
        t[j, ok] = S.t[j, i[ok]]
    return t


def cell_x() -> np.ndarray:
    sh = _CFG["sheet"]
    nx = int(round((sh.x_range[1] - sh.x_range[0]) / sh.cell))
    return sh.x_range[0] + (np.arange(nx) + 0.5) * sh.cell


# -- visuals --------------------------------------------------------------------------------------------------------

# thickness bands (mm, upper bound) and their colors: thin dark, on spec (2.9 to 3.5 mm) golden, thicker paler
BANDS = ((2.5, (0.45, 0.25, 0.10)), (2.9, (0.70, 0.45, 0.20)), (3.5, (0.90, 0.68, 0.30)),
         (3.7, (0.95, 0.82, 0.55)), (4.2, (0.97, 0.90, 0.72)), (1e9, (0.99, 0.97, 0.90)))


def band(t_m: float) -> int:
    mm = t_m * 1000.0
    for i, (hi, _) in enumerate(BANDS):
        if mm < hi:
            return i
    return len(BANDS) - 1


DOUGH_BAND = 2  # one dough color for every slice: the video shows the sheet, not a thickness map


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        protos = {f"b{i}": sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=tuple(kit._linear(x) for x in col), roughness=0.8)) for i, (_, col) in enumerate(BANDS)}
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/doughkit",
                                                                       markers=protos))
    except Exception as e:  # visuals are optional: scoring never depends on them
        print(f"doughkit: no visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _draw(origin) -> None:
    m = _markers()
    if m is None:
        return
    sh = _CFG["sheet"]
    t = grid()
    xc = cell_x()
    S = _state()
    tr, sc, ids = [], [], []
    for j in range(t.shape[0]):
        for i in range(t.shape[1]):
            if t[j, i] <= 0:
                tr.append((0.0, 0.0, -10.0)), sc.append((1e-5,) * 3), ids.append(0)
                continue
            tr.append((xc[i] + origin[0], S.ys[j] + origin[1], sh.board_z + t[j, i] / 2 + origin[2]))
            sc.append((sh.cell * 1.002, sh.cell * 1.002, t[j, i])), ids.append(DOUGH_BAND)
    m.visualize(translations=np.array(tr, float), orientations=np.tile([0.0, 0.0, 0.0, 1.0], (len(tr), 1)),
                scales=np.array(sc, float), marker_indices=np.array(ids, int))


# -- hook into yam_arena.scene (after yamkit's) ----------------------------------------------------------------------


def _patch_embodiment() -> None:
    """Draw the dough as it starts the trial right after each scene reset, before the first frame is taken (the
    step hook only runs from the first control step on, so the dough would pop in). Wraps the embodiment's
    observation at t = 0 (the task file is loaded inside the first reset, so its reset can't be wrapped in time)."""
    ie = sys.modules.get("yam_arena.inspect_embodiment")
    cls = getattr(ie, "YamArenaEmbodiment", None)
    if cls is None or getattr(cls, "_doughkit", False):
        return
    cls._doughkit = True
    original = cls._observation

    def _observation(self):
        global _S
        if getattr(self, "_t", None) == 0.0 and getattr(self, "_base", None) is not None:
            try:
                _S = None  # fresh dough for the new trial
                _draw(self._base.scene.env_origins[0].detach().cpu().numpy())
                self._env.step(self._ready)  # one more settle step at the ready pose, so the cameras update
                for _ in range(getattr(ie, "RESET_RENDERS", 4)):
                    self._base.sim.render()
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                print(f"doughkit: no reset draw ({e!r})", file=sys.stderr, flush=True)
        return original(self)

    cls._observation = _observation


def _prime(env) -> None:
    """At env build: create the markers now (so the renderer has compiled their material long before the first frame)
    and draw the start sheet after every scene reset, before the settle steps and the reset renders: frame 0 already
    shows the dough in its final colour and shape (GLOBAL-FIXES, Nov 13 frame-0 rule)."""
    global _S
    base = env.unwrapped
    if getattr(base, "_doughkit_vis", False) or _markers() is None:
        return
    base._doughkit_vis = True
    _S = None
    _draw(base.scene.env_origins[0].detach().cpu().numpy())
    inner = base.reset

    def reset(*a, **k):
        global _S
        out = inner(*a, **k)
        if not _VIS["failed"]:
            try:
                _S = None  # a fresh sheet for the new trial
                _draw(base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                print(f"doughkit: start drawing off ({e!r})", file=sys.stderr, flush=True)
        return out

    base.reset = reset


def _patch(scene) -> None:
    _patch_embodiment()
    if getattr(scene, "_doughkit", False):
        return
    scene._doughkit = True
    make = getattr(scene, "make_env", None)
    if make is not None:
        def make_env(*a, **k):
            env = make(*a, **k)
            try:
                _prime(env)
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                print(f"doughkit: visuals not primed ({e!r})", file=sys.stderr, flush=True)
            return env

        scene.make_env = make_env
    inner = scene.object_states
    last = {"key": None}

    def object_states(env, names):
        out = inner(env, names)
        base = env.unwrapped
        buf = getattr(base, "episode_length_buf", None)
        key = (kit.episode(), int(buf[0].item()) if buf is not None else None)
        if key[1] is None or key != last["key"]:  # once per control step
            last["key"] = key
            if _CFG.get("before"):
                try:
                    _CFG["before"](base)
                except Exception as e:  # noqa: BLE001  (reported; the task's checks fall back to their offline form)
                    if not _VIS.get("before_failed"):
                        print(f"doughkit: before-step hook failed ({e!r})", file=sys.stderr, flush=True)
                    _VIS["before_failed"] = True
            update(out)
            _apply(base)
            try:
                _draw(base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                if not _VIS["failed"]:
                    print(f"doughkit: visuals off ({e!r})", file=sys.stderr, flush=True)
                _VIS["failed"] = True
        return out

    scene.object_states = object_states


_APPLY = {"mode": None, "failed": False, "base": None, "S": None, "lines": None}


class _Pose:
    def __init__(self, pos, quat_xyzw):
        self.pos, self.quat_xyzw = pos, quat_xyzw


def _apply(base) -> None:
    """Once: hook the scene's write to the simulator, so the dough's push on the pin is worked out from the pin's
    pose and put on it before every physics step (a push held for a whole control step makes the pin bounce)."""
    if not _CFG["pin"].stress or _APPLY["failed"] or _APPLY["base"] is base:
        return
    _APPLY["base"] = base
    write = base.scene.write_data_to_sim

    def write_data_to_sim(*a, **k):
        try:
            _physics_step(base)
        except Exception as error:  # noqa: BLE001  (never let the push break a run)
            _APPLY["failed"] = True
            print(f"doughkit: dough push off ({error!r})", file=sys.stderr, flush=True)
        return write(*a, **k)

    base.scene.write_data_to_sim = write_data_to_sim


def _physics_step(base) -> None:
    import torch

    S = _state()
    if _APPLY["S"] is not S:  # a new trial: no previous line
        _APPLY["S"], _APPLY["lines"] = S, None
    asset = base.scene[_CFG["pin"].obj]
    d = asset.data
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    pos = d.root_pos_w[0].detach().cpu().numpy().astype(np.float64) - origin
    quat = d.root_quat_w[0].detach().cpu().numpy().astype(np.float64)  # (x, y, z, w), as yamkit reads it here
    objs = {_CFG["pin"].obj: _Pose(pos, quat)}
    lines = [pin_line(objs, float(y)) for y in S.ys]
    F, T = _reaction(S, objs, lines, _APPLY["lines"])
    _APPLY["lines"] = lines
    f = torch.tensor(np.asarray(F, np.float32).reshape(1, 1, 3), device=asset.device)
    tq = torch.tensor(np.asarray(T, np.float32).reshape(1, 1, 3), device=asset.device)
    tries = []
    comp = getattr(asset, "permanent_wrench_composer", None)
    if comp is not None:
        tries.append(("wrench_composer", lambda: comp.set_forces_and_torques(
            forces=f, torques=tq, body_ids=[0], env_ids=[0], is_global=True)))
    if hasattr(asset, "set_external_force_and_torque"):
        tries.append(("external_global", lambda: asset.set_external_force_and_torque(
            f, tq, body_ids=[0], env_ids=[0], is_global=True)))
    for name, fn in tries:
        if _APPLY["mode"] not in (None, name):
            continue
        try:
            fn()
            _APPLY["mode"] = name
            return
        except Exception as error:  # noqa: BLE001  (try the next API)
            print(f"doughkit: {name} failed ({error!r})", file=sys.stderr, flush=True)
    raise RuntimeError("no way to put the dough's push on the pin")


class _Hook(kit._Hook):
    """Runs the hook it replaced (yamkit's), then adds the dough step."""

    def __init__(self, prev=None):
        self._prev = prev

    def exec_module(self, module):
        if self._prev is not None:
            self._prev._inner = self._inner
            self._prev.exec_module(module)
        else:
            self._inner.exec_module(module)
            kit._patch(module)
        _patch(module)


_patch_embodiment()
if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
elif not any(isinstance(f, _Hook) for f in sys.meta_path):
    _prev = next((f for f in sys.meta_path if isinstance(f, kit._Hook)), None)
    sys.meta_path[:] = [f for f in sys.meta_path if f is not _prev]
    sys.meta_path.insert(0, _Hook(_prev))
