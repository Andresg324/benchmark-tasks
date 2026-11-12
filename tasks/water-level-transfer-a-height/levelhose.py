"""levelhose: a water level (clear hose of water between two upright end tubes: communicating vessels), for a
yam_arena task file (built on yamkit; edits nothing).

No fluids in the sim, so this is a scored model read from the end props' poses after every control step.

State (per trial, all reset when kit.episode() changes): the water volume in the hose (fixed, except what spills),
the common surface height h, spilled mL (latched), each end's frozen column while an end is capped.

Geometry. Each `End` is a clear rigid tube on a held prop: `bottom` (where the hose joins) and `top` (the open rim) in
the prop's frame, inner `radius`. The `Hose` (inner `radius`, `length` from one tube bottom to the other) hangs
straight down from each tube bottom to the table (`floor_z`) and lies on it between them. Lifting an end draws hose up
off the table, so the hose length below any h > both tube bottoms is the whole hose.

Laws (per control step):
- Level. With every end open, the water stands at one height h everywhere: h solves wetted(h) = volume, where
  wetted(h) = hose cross-section x (hose length below h) + each tube's cross-section x its length below h (by
  bisection; wetted is monotone in h). Consequence (real, and the reason a water-level user re-sets the datum end):
  raising one end by d raises h by d x A_that / (A_1 + A_2) (half of d with equal tubes), because that tube's
  water moves to the other one. A wide datum end (a reservoir jug, `radius` 5 cm) keeps h within about 1 mm.
- Spill. An open end whose rim is below h overflows at Q = CD x A_tube x sqrt(2 g (h - rim)) (Torricelli, CD = 0.6),
  until h falls to the rim. What spills is gone for the trial (latched): h stays lower when the end is raised again.
  Since h follows a lowered end down (by the same share as above), "below h" means below where the water re-settles:
  with two equal 20 cm tubes half full, an end spills only once its rim drops below about the other tube's bottom.
  Shorter or fuller tubes spill sooner.
- Cap. An end whose `capped` is true (a bool, or a function of the step's objects: a thumb over the end) seals the
  hose: while any end is capped no air can get in, so each tube's column is frozen in that tube (it moves with
  the tube) and nothing spills. Uncapped, the water re-levels at once.
- Taut. If the hose is too short to drop to the table from both ends and still span the gap between them, `taut()`
  is true (the model still solves as if it reached; a task may treat it as a pulled hose).

Scorer API (after the kit updated for the step): height() (m, task frame z; while capped, the first open end's
meniscus), surface_at(end) (meniscus z at that end), spilled() mL, volume() mL left, spilling(), taut(), step().
`objects` is optional on all of them.

Drawn (Isaac Lab VisualizationMarkers; cameras see them, never affect scoring): the blue water column in each end
tube (along the tube, up to the meniscus), the hose as blue where it is full and pale where it is dry, a stream from a
spilling rim to the table, and a puddle that grows with what spilled. Drawn right after every scene reset, before the
first rendered frame (a fresh trial at the start poses), and after every step.

Constants: g = 9.81; CD = 0.6 (sharp-edged outflow); typical clear vinyl level hose 3/8" ID (radius 4.8 mm), end
tubes 1/2" ID (radius 6.3 mm). The hose shape is a simplification: real hose sags in curves, but only how much hose
lies below h matters, and that is the whole hose whenever h is above both tube bottoms (the normal case).

Limits: no meniscus or surface-tension effects (a narrow tube tipped over still spills); re-leveling is instant (real
water sloshes for about a second); a capped column does not shift as the hose moves; the hose is not a physical body.

Usage in task.py (after yamkit; import this kit after any other kit that hooks the scene):
    import levelhose as lh
    lh.setup([lh.End("datum", bottom=(0, 0, -0.10), top=(0, 0, 0.10)),
              lh.End("far", bottom=(0, 0, -0.10), top=(0, 0, 0.10), capped=lambda o: thumb_on(o))],
             hose=lh.Hose(radius=0.0048, length=1.2), fill_to=TOP + 0.20, floor_z=TOP)
Tests call update(objects) per step with hand-made ObjectStates.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

import yamkit as kit

ML, G, CD = 1e6, 9.81, 0.6


@dataclass(frozen=True)
class End:
    """A clear end tube on task object `name`: `bottom` (hose side) and `top` (open rim) in its frame, inner
    `radius` m. `capped`: bool or function(objects) -> bool."""

    name: str
    bottom: tuple
    top: tuple
    radius: float = 0.0063
    capped: object = False


@dataclass(frozen=True)
class Hose:
    radius: float = 0.0048
    length: float = 1.2


_CFG = {"ends": (), "hose": Hose(), "volume": None, "fill_to": None, "floor_z": 0.0, "dt": 0.05,
        "color": (0.30, 0.55, 0.95)}


class _State:
    def __init__(self):
        self.episode = kit.episode()
        self.step = 0
        self.volume = _CFG["volume"]  # m^3; None until the first step when set by fill_to
        self.h = float("nan")
        self.spilled = 0.0  # m^3
        self.spilling = None  # (rim xyz) while an end overflows
        self.frozen = None  # per end: wetted length along its tube while capped
        self.cols = None  # per end: wetted length along its tube at the last open step
        self.surf = {}
        self.taut = False
        self.geo = []  # per end: (bottom, top) world, for the visuals


_S = _State()


def setup(ends, hose: Hose = Hose(), volume: float | None = None, fill_to: float | None = None, floor_z: float = 0.0,
          control_hz: float = 20.0, color=(0.30, 0.55, 0.95)) -> None:
    """Two `End`s and the `Hose`. Give the water as `volume` mL, or as `fill_to` (task-frame z of the surface at the
    start poses, worked out at each trial's first step). `floor_z` is the table top."""
    global _S
    if (volume is None) == (fill_to is None):
        raise ValueError("give exactly one of volume (mL) or fill_to (z)")
    _CFG.update(ends=tuple(ends), hose=hose, volume=None if volume is None else volume / ML, fill_to=fill_to,
                floor_z=float(floor_z), dt=1.0 / control_hz, color=color)
    _S = _State()


def _state() -> _State:
    global _S
    if _S.episode != kit.episode():
        _S = _State()
    return _S


def _rotm(q) -> np.ndarray:
    x, y, z, w = (float(c) for c in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _ends(objects):
    """[(bottom, top) world] per end."""
    out = []
    for e in _CFG["ends"]:
        o = objects[e.name]
        R, p = _rotm(o.quat_xyzw), np.asarray(o.pos, float)
        out.append((p + R @ np.asarray(e.bottom, float), p + R @ np.asarray(e.top, float)))
    return out


def _below(a, b, h) -> float:
    """Length of segment a-b below height h."""
    L = float(np.linalg.norm(b - a))
    za, zb = a[2], b[2]
    if abs(zb - za) < 1e-9:
        return L if za < h else 0.0
    lo, hi = min(za, zb), max(za, zb)
    return L * min(1.0, max(0.0, (h - lo) / (hi - lo)))


def _wetted(geo, h) -> float:
    """m^3 of water below height h with every end open."""
    hose, fz = _CFG["hose"], _CFG["floor_z"]
    drops = [max(0.0, b[2] - fz) for b, _ in geo]
    floor_run = max(0.0, hose.length - sum(drops))
    wet = (floor_run if h > fz else 0.0) + sum(min(max(h, fz), b[2]) - fz for b, _ in geo if b[2] > fz)
    out = math.pi * hose.radius ** 2 * wet
    for e, (b, t) in zip(_CFG["ends"], geo):
        out += math.pi * e.radius ** 2 * _below(b, t, h)
    return out


def _solve(geo, vol) -> float:
    lo = _CFG["floor_z"]
    hi = max(max(b[2], t[2]) for b, t in geo) + 1.0
    if _wetted(geo, hi) <= vol:
        return hi
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if _wetted(geo, mid) < vol else (lo, mid)
    return (lo + hi) / 2


def _capped(e: End, objects) -> bool:
    return bool(e.capped(objects) if callable(e.capped) else e.capped)


def update(objects) -> None:
    """Advance one control step from the end props' poses in `objects`."""
    st, ends = _state(), _CFG["ends"]
    if not ends or any(e.name not in objects for e in ends):
        return
    st.step += 1
    geo = _ends(objects)
    st.geo = geo
    if st.volume is None:  # fill_to: the water that stands at that height at the start poses
        st.volume = _wetted(geo, _CFG["fill_to"])
    hose, fz = _CFG["hose"], _CFG["floor_z"]
    gap = float(np.linalg.norm((geo[0][0] - geo[1][0])[:2]))
    st.taut = hose.length - sum(max(0.0, b[2] - fz) for b, _ in geo) < gap
    st.spilling = None
    caps = [_capped(e, objects) for e in ends]
    if any(caps):
        if st.frozen is None:  # sealed now: keep each tube's column where it was at the last open step
            st.frozen = st.cols or [_below(b, t, _solve(geo, st.volume)) for b, t in geo]
        st.surf = {}
        for e, (b, t), s in zip(ends, geo, st.frozen):
            L = float(np.linalg.norm(t - b))
            st.surf[e.name] = float(b[2] + (t[2] - b[2]) * min(1.0, s / L)) if L > 0 else float(b[2])
        opens = [st.surf[e.name] for e, c in zip(ends, caps) if not c]
        st.h = opens[0] if opens else float(np.mean(list(st.surf.values())))
        return
    st.frozen = None
    h = _solve(geo, st.volume)
    rims = [(t[2], e, t) for e, (b, t) in zip(ends, geo)]
    rz, e, t = min(rims, key=lambda r: r[0])
    if h > rz + 1e-6:  # the lowest open rim is under the water: it overflows (Torricelli), latched
        excess = st.volume - _wetted(geo, rz)
        q = CD * math.pi * e.radius ** 2 * math.sqrt(2 * G * (h - rz)) * _CFG["dt"]
        lost = min(excess, q)
        st.volume -= lost
        st.spilled += lost
        st.spilling = t
        h = _solve(geo, st.volume)
    st.h = h
    st.surf = {e.name: h for e in ends}
    st.cols = [_below(b, t, h) for b, t in geo]


# -- scorer API --------------------------------------------------------------------------------


def height(objects=None) -> float:
    """The common water surface height (task frame z, m). While an end is capped: the open end's meniscus."""
    return float(_state().h)


def surface_at(objects=None, end: str | None = None) -> float:
    """Meniscus height at `end` (task frame z, m). Call as surface_at(objects, "far") or surface_at(end="far")."""
    if end is None and isinstance(objects, str):
        objects, end = None, objects
    return float(_state().surf.get(end, float("nan")))


def spilled(objects=None) -> float:
    """mL spilled this trial (latched)."""
    return float(_state().spilled * ML)


def volume(objects=None) -> float:
    """mL left in the level."""
    v = _state().volume
    return float("nan") if v is None else float(v * ML)


def spilling(objects=None) -> bool:
    return _state().spilling is not None


def taut(objects=None) -> bool:
    return bool(_state().taut)


def step() -> int:
    return _state().step


# -- visuals -----------------------------------------------------------------------------------

_VIS = {"markers": None, "failed": False}
PROTOS = ("water", "hose")
SLOTS = {"water": 7, "hose": 2}  # water: 2 tube columns, 2 hose drops, the floor run, a stream, a puddle
PUDDLE_T, PUDDLE_MAX = 0.002, 0.08


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        def mat(rgb, rough):
            return sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in rgb), roughness=rough)

        protos = {"water": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=mat(_CFG["color"], 0.15)),
                  "hose": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=mat((0.85, 0.88, 0.88), 0.3))}
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/levelhose", markers=protos))
    except Exception as e:  # visuals are optional: scoring never depends on them
        print(f"levelhose: no visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _quat_z_to(d) -> tuple:
    """Quaternion (x, y, z, w) turning +z onto direction d."""
    d = np.asarray(d, float) / np.linalg.norm(d)
    c = float(d[2])
    if c < -0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    ax = np.cross([0.0, 0.0, 1.0], d)
    w = math.sqrt((1 + c) / 2)
    ax = ax / (2 * w)
    return (float(ax[0]), float(ax[1]), float(ax[2]), w)


def _seg(a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    if L < 1e-5:
        return None
    return ((a + b) / 2, _quat_z_to(b - a), (r, r, L))


def _items(objects) -> dict:
    st, hose, fz = _state(), _CFG["hose"], _CFG["floor_z"]
    out = {"water": [], "hose": []}
    if not st.geo or math.isnan(st.h):
        return out
    feet = []
    for e, (b, t) in zip(_CFG["ends"], st.geo):
        s = st.surf.get(e.name, st.h)
        L = _below(b, t, s) if st.frozen is None else st.frozen[_CFG["ends"].index(e)]
        L = min(L, float(np.linalg.norm(t - b)))
        if L > 1e-4:
            out["water"].append(_seg(b, b + (t - b) * L / np.linalg.norm(t - b), e.radius * 0.95))
        foot = np.array([b[0], b[1], fz + hose.radius])
        feet.append(foot)
        wet_top = min(b[2], max(s, fz))
        if wet_top > foot[2] + 1e-4:
            out["water"].append(_seg(foot, (b[0], b[1], wet_top), hose.radius))
        if b[2] > wet_top + 1e-4:
            out["hose"].append(_seg((b[0], b[1], wet_top), b, hose.radius))
    run = _seg(feet[0], feet[1], hose.radius)
    if run is not None:
        out["water"].append(run)
    if st.spilling is not None:
        t = st.spilling
        out["water"].append(_seg((t[0], t[1], fz), t, 0.002))
    if st.spilled > 0.2e-6:
        r = min(PUDDLE_MAX, math.sqrt(st.spilled / (math.pi * PUDDLE_T)))
        b = st.geo[0][0] if st.spilling is None else st.spilling
        out["water"].append(((b[0], b[1], fz + PUDDLE_T / 2), (0, 0, 0, 1), (r, r, PUDDLE_T)))
    return {k: [i for i in v if i is not None] for k, v in out.items()}


def _draw(objects, origin) -> None:
    m = _markers()
    if m is None:
        return
    items = _items(objects)
    tr, qs, sc, ids = [], [], [], []
    for i, p in enumerate(PROTOS):
        lst = items[p][:SLOTS[p]]
        for pos, q, s in lst:
            tr.append(tuple(np.asarray(pos, float) + origin)), qs.append(tuple(q)), sc.append(tuple(s)), ids.append(i)
        for _ in range(SLOTS[p] - len(lst)):
            tr.append((0.0, 0.0, -10.0)), qs.append((0, 0, 0, 1)), sc.append((1e-5,) * 3), ids.append(i)
    m.visualize(translations=np.array(tr, float), orientations=np.array(qs, float), scales=np.array(sc, float),
                marker_indices=np.array(ids, int))


# -- hooks: the step (scene.object_states) and the reset draw (the embodiment's env.reset) ---------


def _patch(scene) -> None:
    if getattr(scene, "_levelhose", False):
        return
    scene._levelhose = True
    inner, last = scene.object_states, {"key": None}

    def object_states(env, names):
        out = inner(env, names)
        base = env.unwrapped
        buf = getattr(base, "episode_length_buf", None)
        key = (kit.episode(), int(buf[0].item()) if buf is not None else None)
        if _CFG["ends"] and (key[1] is None or key != last["key"]):  # once per control step
            last["key"] = key
            update(out)
            try:
                _draw(out, base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                if not _VIS["failed"]:
                    print(f"levelhose: visuals off ({e!r})", file=sys.stderr, flush=True)
                _VIS["failed"] = True
        return out

    scene.object_states = object_states


def _start_objects(env, names):
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    out = {}
    for n in names:
        try:
            pose = base.arena_world.get_pose_w(n)[0].detach().cpu().numpy().astype(np.float64)
        except Exception:  # noqa: BLE001
            continue
        out[n] = SimpleNamespace(pos=pose[:3] - origin, quat_xyzw=pose[3:])
    return out, origin


def _draw_start(env) -> None:
    """Draw a fresh trial (water levelled at the start poses) right after a scene reset, before the first rendered
    frame, without touching the trial's own state."""
    global _S
    if not _CFG["ends"]:
        return
    objs, origin = _start_objects(env, [e.name for e in _CFG["ends"]])
    keep, _S = _S, _State()
    try:
        update(objs)
        _draw(objs, origin)
    finally:
        _S = keep


def _patch_embodiment() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline: no embodiment)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_levelhose", False):
        return
    cls._levelhose = True
    ensure = cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        ensure(self, *a, **k)
        env = self._env
        if env is None or getattr(env, "_levelhose_reset", False):
            return
        reset = env.reset

        def reset_and_draw(*ra, **rk):
            out = reset(*ra, **rk)
            try:
                _draw_start(env)
            except Exception as e:  # noqa: BLE001
                print(f"levelhose: no start visuals ({e!r})", file=sys.stderr, flush=True)
            return out

        env.reset, env._levelhose_reset = reset_and_draw, True

    cls._ensure_sim = _ensure_sim


class _Hook(kit._Hook):
    """Runs the import hook that was first in line (yamkit's, or another kit's chain), then adds this kit's step."""

    def __init__(self, prev):
        self.prev = prev

    def exec_module(self, module):
        if self.prev is not None:
            self.prev._inner = self._inner
            self.prev.exec_module(module)
        else:
            self._inner.exec_module(module)
            kit._patch(module)
        _patch(module)


_patch_embodiment()
if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:
    _prev = next((f for f in sys.meta_path if isinstance(f, kit._Hook)), None)
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook(_prev))
