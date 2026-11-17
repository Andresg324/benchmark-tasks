"""doughkit: a piece of pastry rolled out under a rolling pin with spacer rings (Nov 11; Nov 15: a sheet, read on a
2 cm grid). Import after yamkit:
    import doughkit as dk

The pastry is modeled, not simulated: it has no collider, so the pin rides on its rings on the board and the kit works
out what the barrel does to the pastry under it. The sheet is a row of strips along y (one per `cell` of width); each
strip is a chain of thin slices along x, each with a fixed volume (per unit width) and a thickness, so its length is
volume / thickness and the total volume never changes.

Each control step, for each strip, the kit finds where the barrel's lowest line crosses that strip (x) and how high
it is above the board there (the gap: the rings' clearance, plus however far that end of the pin is lifted, so a
tilted pin leaves a wedge). A slice whose center the line rolls across, with the gap below the slice's thickness, is
pressed as the barrel's face reaches it (its lowest line still sqrt(2 R (t - gap)) short of the slice's center,
the contact patch of a round barrel on thick pastry): t <- gap + (t - gap) * RATIO (the pastry springs back by RATIO of the excess, so it takes several passes to
come down to the gap). A pressed slice lengthens; the slices on its far side from the strip's anchor (the piece's
middle, held by the bulk of the dough) move outward to make room. A slice is pressed again only once the line has
gone back REARM behind where the barrel's face meets it (one press per crossing, however the arm jitters). Rolling
past a strip's end presses the end slice like any other: the rings hold the barrel at the gap there too.

Hands (the task's `gate(objects) -> bool`): presses count only while it is True (e.g. the pin driven by both handles).

Resistance (Nov 16): the pastry pushes back on the pin. For each strip the barrel is in contact
with (pastry thicker than the gap within the barrel's contact reach of its lowest line), with h the largest excess
there and a = sqrt(2 R h) the contact length: an upward force STRESS x cell x a (the pastry's flow stress over the
contact's footprint) and, while the line moves, a force against the motion STRESS x cell x h (the work of squeezing h
off a cell-wide strip, per meter rolled). They act at the contact, so a pin pressed harder at one end feels the thicker
side push back harder, and they are worked out from the pin's pose and applied to it as an external force and torque
before every physics step (the press itself still happens once per control step).
`stress` is set by the task (Pin.stress, Pa; 0 turns it off).

    dk.setup(dk.Sheet(...), dk.Pin(...))    once, at import time of task.py
    dk.update(objects)                       each control step (the hook calls it; offline tests call it)
    dk.grid() -> t                           thickness (m, 0 = no pastry) at the grid's cell centers
    dk.lengths() -> per-strip lengths (m); dk.volume() total volume (m^3); dk.ends() per-strip (x_min, x_max)
All of it resets when kit.episode() changes.

Visuals: one abutting slab per covered grid cell, as thick as the pastry there, all one pastry color (Isaac Lab
VisualizationMarkers, no physics), so the top camera shows the sheet growing.
"""

import math
import sys
from dataclasses import dataclass

import numpy as np

import yamkit as kit

RATIO = 0.5  # fraction of the excess over the gap left after one press (springback)
REARM = 0.01  # m: the line must go this far back past a slice before it can press it again


@dataclass(frozen=True)
class Sheet:
    """The pastry: a piece `size` = (length along x, width along y, thickness) centered at (cx, cy) on a board whose top
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
    stress: float = 0.0  # Pa: the pastry's flow stress against the barrel (0: no resistance)


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
        self.presses = 0  # presses that counted (both handles held)
        self.wrench = (np.zeros(3), np.zeros(3))  # the pastry's force and torque on the pin (world, about its center)
        self.peak = np.zeros(3)  # largest |Fx|, |Fy|, Fz seen this trial (N)


def setup(sheet: Sheet, pin: Pin, gate=None) -> None:
    """`gate(objects) -> bool`: presses count only while it is True (None: always)."""
    global _S
    _CFG.update(sheet=sheet, pin=pin, gate=gate)
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
    held = bool(_CFG["gate"](objects)) if _CFG.get("gate") else True
    t0, armed0 = S.t.copy(), S.armed.copy()
    S.wrench = _reaction(S, objects, lines, S.line)
    S.peak = np.maximum(S.peak, np.abs(S.wrench[0]))
    for j, (now, before) in enumerate(zip(lines, S.line)):
        if now is None or before is None:
            continue
        x0, g0 = before
        x1, g1 = now
        if x1 == x0:
            continue
        d = 1 if x1 > x0 else -1
        k = 0 if d > 0 else 1
        n = S.t.shape[1]
        order = range(n) if d > 0 else range(n - 1, -1, -1)
        e = _edges(S, j)  # recomputed only after a press moves them
        for i in order:
            if not S.armed[k, j, i]:
                continue
            c = 0.5 * (e[i] + e[i + 1])
            g = min(g0, g1)
            hit = c - d * _reach(S.t[j, i], g)  # where the line is when the barrel's face meets the slice's top
            if not (min(x0, x1) < hit <= max(x0, x1)):
                continue
            f = (hit - x0) / (x1 - x0)
            g = g0 + f * (g1 - g0)  # the gap there
            if g < S.t[j, i]:
                S.t[j, i] = g + (S.t[j, i] - g) * RATIO
                S.armed[k, j, i] = False
                e = _edges(S, j)
    if not held:  # the pin moved without the gate's hold does not count
        S.t, S.armed = t0, armed0
    elif not np.array_equal(S.t, t0):
        S.presses += 1
    S.line = lines
    _rearm(S, lines)


def _reaction(S, objects, lines, before):
    """The pastry's force and torque on the pin this step (world frame, torque about the pin's center), from the
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
        h = float(ex[touch].max())
        a = math.sqrt(max(0.0, 2 * p.radius * h - h * h))
        d = 0.0
        if before is not None and before[j] is not None and now[0] != before[j][0]:
            d = 1.0 if now[0] > before[j][0] else -1.0
        f = np.array([-d * p.stress * sh.cell * h, 0.0, p.stress * sh.cell * a])
        at = np.array([x + d * 0.5 * a, float(S.ys[j]), sh.board_z + g])  # the middle of the contact, ahead of the line
        F += f
        T += np.cross(at - center, f)
    return F, T


def wrench():
    """The pastry's (force, torque) on the pin from the last control step (N, N m; world frame, about its center)."""
    return _state().wrench


def peak():
    """The largest |Fx|, |Fy| and upward Fz the pastry put on the pin this trial (N)."""
    return _state().peak.copy()


def _reach(t: float, g: float) -> float:
    """How far ahead of its lowest line the barrel's face first touches pastry `t` thick with the line at gap `g`."""
    h = t - g
    if h <= 0:
        return 0.0
    r = _CFG["pin"].radius
    return math.sqrt(max(0.0, 2 * r * h - h * h)) if h < r else r


def _rearm(S, lines) -> None:
    for j, ln in enumerate(lines):
        if ln is None:
            continue
        e = _edges(S, j)
        c = 0.5 * (e[:-1] + e[1:])
        r = np.array([_reach(t, ln[1]) for t in S.t[j]])  # back behind where the barrel's face meets each slice
        S.armed[0, j] |= ln[0] < c - r - REARM
        S.armed[1, j] |= ln[0] > c + r + REARM


def volume() -> float:
    """Total pastry volume (m^3)."""
    S = _state()
    return float(S.vol.sum() * _CFG["sheet"].cell)


def ends() -> np.ndarray:
    """Per strip, (x_min, x_max) of the pastry (world m)."""
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
    (m) of the slice under the cell's center, 0 where there is no pastry."""
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


PASTRY_BAND = 2  # one pastry color for every slice: the video shows the sheet, not a thickness map


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
            sc.append((sh.cell * 1.002, sh.cell * 1.002, t[j, i])), ids.append(PASTRY_BAND)
    m.visualize(translations=np.array(tr, float), orientations=np.tile([0.0, 0.0, 0.0, 1.0], (len(tr), 1)),
                scales=np.array(sc, float), marker_indices=np.array(ids, int))


# -- hook into yam_arena.scene (after yamkit's) ----------------------------------------------------------------------


def _patch_embodiment() -> None:
    """Draw the pastry as it starts the trial right after each scene reset, before the first frame is taken (the
    step hook only runs from the first control step on, so the pastry would pop in). Wraps the embodiment's
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
                _S = None  # fresh pastry for the new trial
                _draw(self._base.scene.env_origins[0].detach().cpu().numpy())
                self._env.step(self._ready)  # one more settle step at the ready pose, so the cameras update
                for _ in range(getattr(ie, "RESET_RENDERS", 4)):
                    self._base.sim.render()
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                print(f"doughkit: no reset draw ({e!r})", file=sys.stderr, flush=True)
        return original(self)

    cls._observation = _observation


def _patch(scene) -> None:
    _patch_embodiment()
    if getattr(scene, "_doughkit", False):
        return
    scene._doughkit = True
    inner = scene.object_states
    last = {"key": None}

    def object_states(env, names):
        out = inner(env, names)
        base = env.unwrapped
        buf = getattr(base, "episode_length_buf", None)
        key = (kit.episode(), int(buf[0].item()) if buf is not None else None)
        if key[1] is None or key != last["key"]:  # once per control step
            last["key"] = key
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
    """Once: hook the scene's write to the simulator, so the pastry's push on the pin is worked out from the pin's
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
            print(f"doughkit: pastry push off ({error!r})", file=sys.stderr, flush=True)
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
    raise RuntimeError("no way to put the pastry's push on the pin")


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
