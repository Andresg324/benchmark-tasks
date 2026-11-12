"""dispersekit: dispersing a powder into a stirred liquid (polymer make-down, thickeners, gums, CMC, xanthan,
polyacrylamide) and the "fish-eyes" that form when it is added wrong, for a yam_arena task file (built on yamkit;
edits nothing).

No particles or fluids in the sim, so this is a scored model read from poses after every control step.

State (per trial, all reset when kit.episode() changes):
- The beaker's liquid (fixed volume, drawn as a column; the surface is level with the beaker assumed upright).
- Surface powder: parcels {x, y (beaker frame), grams, age, layer}. Each step's landed grams merge into a parcel
  within MERGE_R of where they land (still liquid piles them up in one spot) or start a new one. A parcel's `layer`
  is the most of it that can lie one grain deep: SIGMA x the strip it was laid on, `spread` wide (the stream's
  width) and spread + the surface's travel that step long (a stirred surface moving under the stream spreads it).
- Agitation u (m/s): what the liquid's surface is doing, driven by the stir rod.
- Lumps (latched, irreversible) and totals: added, dispersed, lumped, spilled.

Laws (per control step, dt = 1/control_hz):
- Adding. A `Shaker` held tipped past `tilt` degrees (its spout axis from straight up) releases `rate` g/s (ramping
  in over `ramp` degrees, 20 by default, as the `power` of the share of it) until its `load` is gone. Grams fall
  plumb from the spout: inside the beaker's mouth they land on the surface at that point, else they are spilled.
  `add(objects, grams, xy)` adds by hand (another kit's pour).
- Stirring. The rod tip (`Stirrer`: a prop or gripper and a tip offset in its frame) is read in the beaker's frame;
  the path from last step's tip to this step's is sampled at K_SUB points (interpolated between steps), and the
  share of them inside the liquid (inside the radius, between floor and surface) times the tip speed is the drive
  a. u follows a: up with TAU_UP, down with TAU_DOWN (turbulence behind a rod dies within about half a second; the
  slow bulk swirl that remains carries powder round but does not wet it apart). Tip speed is capped at V_CAP so a
  teleport (reset) is not a stir. Parcels and lumps circle the axis at u / (0.6 R) rad/s.
- Dispersing. With u >= V_MIN each parcel wets apart at LAM x (u / V_REF) per second (capped at 3 x LAM), acting on
  at most its `layer` at a time (separate grains wet only one layer deep; a heap's inside stays dry). A parcel
  under 1e-4 g counts as dispersed.
- Lumping (fish-eyes). A parcel's age grows while it sits on still liquid (u < V_MIN), and while it holds more than
  its layer (a dry core). Age past T_GEL: on still liquid the whole parcel latches as a lump; in a stirred one the
  part above its layer does (the gelled skin seals the core). Lumps never disperse.

Constants (reasoned; tune per powder): T_GEL = 1.0 s (gum/polymer grains swell a sticky skin within about a second
of wetting: the make-down rule "add slowly into the shoulder of the vortex, never onto still water" exists because
of it); V_MIN = 0.05 m/s (a rod barely moving); V_REF = 0.15 m/s (a steady stir: a 2 cm circle at about 1.2 rev/s);
LAM = 2.0 /s (a stirred vortex wets a thin dusting in about 0.5 s); SIGMA = 0.006 g/cm^2 (one layer of ~100 um
grains at a bulk 0.6 g/mL); spread = 1 cm (a shaker's stream). Measured offline (400 mL, 4 cm radius, tip circling
2 cm at 2 rev/s): a sprinkle up to 0.3 g/s into that stir disperses fully; 0.5 g/s lumps ~20 %, 1 g/s ~60 %,
5 g/s ~90 %; powder put in before the stirring, or during a > 1 s pause, lumps (a 0.4 s hiccup does not).

Scorer API (after the kit updated for the step, inside object_states): added(), dispersed(), lumped(), surface(),
spilled() (all grams), stir_speed() (u, m/s), lumps() [(x, y, grams) beaker frame], step(). `objects` is optional.

Drawn (Isaac Lab VisualizationMarkers, cameras see them, never affect scoring): the liquid column; each surface
parcel as a flat powder-colored dot (radius grows with its grams); each lump as a white ball. Drawn right after
every scene reset, before the first rendered frame (a fresh trial: full beaker, no powder), and after every step.

Limits: the beaker is taken as upright (its tilt is ignored); the powder does not change the liquid's volume or
color; the shaker needs no shaking; a stir is any tip motion in the liquid (a rod held still in a moving beaker
counts too, since it is read in the beaker frame).

Usage in task.py (after yamkit; import this kit after any other kit that hooks the scene):
    import dispersekit as dk2
    dk2.setup(dk2.Beaker("beaker", radius=0.04, floor=0.005, fill=400.0),
              shakers=[dk2.Shaker("shaker", spout=(0, 0, 0.06), load=5.0)],
              stirrers=[dk2.Stirrer("rod", tip=(0, 0, -0.12))])
Tests call update(objects) per step with hand-made ObjectStates.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from types import SimpleNamespace

import numpy as np

import yamkit as kit

ML = 1e6
T_GEL, V_MIN, V_REF, LAM, SIGMA = 1.0, 0.05, 0.15, 2.0, 0.006  # SIGMA in g/cm^2
TAU_UP, TAU_DOWN, V_CAP, K_SUB = 0.2, 0.5, 1.0, 8
MERGE_R, LUMP_MERGE_R, MAX_PARCELS, MAX_LUMPS = 0.006, 0.008, 200, 24


@dataclass(frozen=True)
class Beaker:
    """The liquid in task object `name`: inner `radius` (m), inside bottom `floor` (m above the object's pos, its
    frame), `fill` mL."""

    name: str
    radius: float
    floor: float
    fill: float
    color: tuple = (0.62, 0.78, 0.95)

    @property
    def level(self) -> float:
        return self.floor + self.fill / ML / (math.pi * self.radius ** 2)


@dataclass(frozen=True)
class Shaker:
    """A powder source on task object `name`: `spout` point and `axis` (the way the powder leaves) in its frame."""

    name: str
    spout: tuple = (0.0, 0.0, 0.05)
    axis: tuple = (0.0, 0.0, 1.0)
    rate: float = 0.3  # g/s fully tipped
    load: float = 5.0  # g
    tilt: float = 110.0  # degrees of the spout axis from straight up before anything comes out
    spread: float = 0.01  # m: the stream's width where it lands
    ramp: float = 20.0  # degrees past `tilt` to the full rate ...
    power: float = 1.0  # ... rising as ((tilt past the start) / ramp) ** power (2: a trickle near the start)


@dataclass(frozen=True)
class Stirrer:
    """A stir rod: task object (or "left_gripper"/"right_gripper") `name` with its `tip` in that frame."""

    name: str
    tip: tuple = (0.0, 0.0, 0.0)


_CFG = {"beaker": None, "shakers": (), "stirrers": (), "dt": 0.05, "powder": (0.93, 0.90, 0.80)}


class _State:
    def __init__(self):
        self.episode = kit.episode()
        self.step = 0
        self.parcels = []  # [x, y, grams, age, layer grams]
        self.lumps = []  # [x, y, grams]
        self.u = 0.0
        self.tips = {}  # stirrer -> last tip, beaker frame
        self.left = {s.name: s.load for s in _CFG["shakers"]}
        self.added = self.dispersed = self.lumped = self.spilled = 0.0
        self.streams = []  # (spout xyz world, landing z) for the visuals


_S = _State()


def setup(beaker: Beaker, shakers=(), stirrers=(), control_hz: float = 20.0, powder=(0.93, 0.90, 0.80)) -> None:
    global _S
    _CFG.update(beaker=beaker, shakers=tuple(shakers), stirrers=tuple(stirrers), dt=1.0 / control_hz, powder=powder)
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


def _local(objects, name, p):
    """Point p (world) in the beaker's frame."""
    b = objects[name]
    return _rotm(b.quat_xyzw).T @ (np.asarray(p, float) - np.asarray(b.pos, float))


def _world(obj, p):
    return np.asarray(obj.pos, float) + _rotm(obj.quat_xyzw) @ np.asarray(p, float)


def _inside(b: Beaker, p) -> bool:
    return math.hypot(p[0], p[1]) < b.radius and b.floor <= p[2] <= b.level


def _layer(st: _State, x: float, y: float, spread: float) -> float:
    travel = st.u / (0.6 * _CFG["beaker"].radius) * math.hypot(x, y) * _CFG["dt"]
    return SIGMA * 1e4 * spread * (spread + travel)


def _land(st: _State, grams: float, x: float, y: float, spread: float = 0.01) -> None:
    st.added += grams
    layer = _layer(st, x, y, spread)
    best = min(st.parcels, key=lambda q: math.hypot(q[0] - x, q[1] - y), default=None)
    near = best is not None and math.hypot(best[0] - x, best[1] - y) < MERGE_R
    if near or len(st.parcels) >= MAX_PARCELS:
        if st.u >= V_MIN:  # on stirred liquid fresh powder dilutes the age; on still liquid it joins the gelling patch
            best[3] = (best[3] * best[2]) / (best[2] + grams)
        best[2] += grams
        best[4] = max(best[4], layer)
    else:
        st.parcels.append([x, y, grams, 0.0, layer])


def _lump(st: _State, x: float, y: float, grams: float) -> None:
    st.lumped += grams
    best = min(st.lumps, key=lambda q: math.hypot(q[0] - x, q[1] - y), default=None)
    if best is not None and (math.hypot(best[0] - x, best[1] - y) < LUMP_MERGE_R or len(st.lumps) >= MAX_LUMPS):
        best[2] += grams
    else:
        st.lumps.append([x, y, grams])


def add(objects, grams: float, xy=(0.0, 0.0), spread: float = 0.01) -> None:
    """Land `grams` on the surface at `xy` (beaker frame) this step: for powder delivered by other means."""
    _land(_state(), float(grams), float(xy[0]), float(xy[1]), spread)


def update(objects) -> None:
    """Advance one control step from the poses in `objects`. The scene hook calls it once per step."""
    st, b, dt = _state(), _CFG["beaker"], _CFG["dt"]
    if b is None or b.name not in objects:
        return
    st.step += 1
    st.streams = []
    surf_w = _world(objects[b.name], (0.0, 0.0, b.level))[2]
    # adding: shakers tipped past their tilt pour plumb from the spout
    for sh in _CFG["shakers"]:
        o = objects.get(sh.name)
        if o is None or st.left[sh.name] <= 1e-9:
            continue
        ax = _rotm(o.quat_xyzw) @ np.asarray(sh.axis, float)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, ax[2] / np.linalg.norm(ax)))))
        if tilt <= sh.tilt:
            continue
        frac = min(1.0, (tilt - sh.tilt) / sh.ramp) ** sh.power
        g = min(st.left[sh.name], sh.rate * frac * dt)
        st.left[sh.name] -= g
        spout = _world(o, sh.spout)
        p = _local(objects, b.name, spout)
        if math.hypot(p[0], p[1]) < b.radius and p[2] > b.level:
            _land(st, g, p[0], p[1], sh.spread)
            st.streams.append((spout, surf_w))
        else:
            st.spilled += g
            st.streams.append((spout, 0.0))
    # stirring: interpolated tip path inside the liquid drives the agitation u
    drive = 0.0
    for sr in _CFG["stirrers"]:
        o = objects.get(sr.name)
        if o is None:
            st.tips.pop(sr.name, None)
            continue
        p1 = _local(objects, b.name, _world(o, sr.tip))
        p0 = st.tips.get(sr.name)
        st.tips[sr.name] = p1
        if p0 is None:
            continue
        v = min(V_CAP, float(np.linalg.norm(p1 - p0)) / dt)
        f = sum(_inside(b, p0 + (k + 0.5) / K_SUB * (p1 - p0)) for k in range(K_SUB)) / K_SUB
        drive = max(drive, v * f)
    tau = TAU_UP if drive > st.u else TAU_DOWN
    st.u = drive + (st.u - drive) * math.exp(-dt / tau)
    # swirl carries parcels and lumps round the axis
    w = st.u / (0.6 * b.radius) * dt
    c, s = math.cos(w), math.sin(w)
    for q in st.parcels + st.lumps:
        q[0], q[1] = c * q[0] - s * q[1], s * q[0] + c * q[1]
    # disperse or gel
    stirred = st.u >= V_MIN
    lam = min(3.0 * LAM, LAM * st.u / V_REF) if stirred else 0.0
    keep = []
    for q in st.parcels:
        x, y, m, age, layer = q
        if not stirred or m > layer:
            age += dt
        if age > T_GEL:
            if not stirred:
                _lump(st, x, y, m)
                continue
            if m > layer:
                _lump(st, x, y, m - layer)
                m, age = layer, 0.0
        wet = min(m, min(m, layer) * (1.0 - math.exp(-lam * dt)))
        m -= wet
        st.dispersed += wet
        if m < 1e-4:
            st.dispersed += m
            continue
        keep.append([x, y, m, age, layer])
    st.parcels = keep


# -- scorer API --------------------------------------------------------------------------------


def added(objects=None) -> float:
    """Grams that landed on the liquid this trial."""
    return float(_state().added)


def dispersed(objects=None) -> float:
    """Grams wetted apart into the liquid (the good outcome)."""
    return float(_state().dispersed)


def lumped(objects=None) -> float:
    """Grams latched into lumps (fish-eyes); irreversible."""
    return float(_state().lumped)


def surface(objects=None) -> float:
    """Grams still dry on the surface (neither dispersed nor lumped yet)."""
    return float(sum(q[2] for q in _state().parcels))


def spilled(objects=None) -> float:
    """Grams that missed the beaker."""
    return float(_state().spilled)


def stir_speed(objects=None) -> float:
    """The liquid's agitation u, m/s."""
    return float(_state().u)


def lumps(objects=None) -> list:
    return [tuple(q) for q in _state().lumps]


def step() -> int:
    return _state().step


# -- visuals -----------------------------------------------------------------------------------

_VIS = {"markers": None, "failed": False}
PROTOS = ("liquid", "powder", "lump")
SLOTS = {"liquid": 1, "powder": 48, "lump": MAX_LUMPS}
STREAM_SLOTS = 2  # drawn with the powder prototype


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        def mat(rgb, rough=0.6):
            return sim_utils.PreviewSurfaceCfg(diffuse_color=tuple(kit._linear(c) for c in rgb), roughness=rough)

        protos = {"liquid": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z",
                                                  visual_material=mat(_CFG["beaker"].color, 0.15)),
                  "powder": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=mat(_CFG["powder"], 0.9)),
                  "lump": sim_utils.SphereCfg(radius=1.0, visual_material=mat((0.98, 0.98, 0.97), 0.4))}
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/dispersekit",
                                                                       markers=protos))
    except Exception as e:  # visuals are optional: scoring never depends on them
        print(f"dispersekit: no visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _items(objects) -> dict:
    """proto -> [(world pos, quat_xyzw, scale)] for this step's state."""
    st, b = _state(), _CFG["beaker"]
    o = objects.get(b.name) if b else None
    out = {p: [] for p in PROTOS}
    if o is None:
        return out
    q = tuple(float(c) for c in o.quat_xyzw)
    depth = b.level - b.floor
    out["liquid"].append((_world(o, (0.0, 0.0, b.floor + depth / 2)), q, (b.radius * 0.97, b.radius * 0.97, depth)))
    for x, y, m, *_ in sorted(st.parcels, key=lambda p: -p[2])[:SLOTS["powder"] - STREAM_SLOTS]:
        r = min(b.radius * 0.6, 0.002 + 0.008 * math.sqrt(m))
        out["powder"].append((_world(o, (x, y, b.level + 0.0008)), q, (r, r, 0.0015)))
    for spout, z in st.streams[:STREAM_SLOTS]:
        length = max(1e-3, float(spout[2] - z))
        out["powder"].append((np.array([spout[0], spout[1], spout[2] - length / 2]), (0, 0, 0, 1), (0.0012, 0.0012, length)))
    for x, y, m in st.lumps:
        r = min(0.012, (3 * m / (4 * math.pi)) ** (1 / 3) / 100)  # a ~1 g/mL gel ball (cm -> m)
        out["lump"].append((_world(o, (x, y, b.level + 0.3 * r)), (0, 0, 0, 1), (r, r, r * 0.8)))
    return out


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


def _names():
    b = _CFG["beaker"]
    return [b.name] + [s.name for s in _CFG["shakers"]] + [s.name for s in _CFG["stirrers"]] if b else []


def _patch(scene) -> None:
    if getattr(scene, "_dispersekit", False):
        return
    scene._dispersekit = True
    inner, last = scene.object_states, {"key": None}

    def object_states(env, names):
        out = inner(env, names)
        base = env.unwrapped
        buf = getattr(base, "episode_length_buf", None)
        key = (kit.episode(), int(buf[0].item()) if buf is not None else None)
        if _CFG["beaker"] is not None and (key[1] is None or key != last["key"]):  # once per control step
            last["key"] = key
            update(out)
            try:
                _draw(out, base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                if not _VIS["failed"]:
                    print(f"dispersekit: visuals off ({e!r})", file=sys.stderr, flush=True)
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
        except Exception:  # noqa: BLE001  (grippers and missing objects are skipped)
            continue
        out[n] = SimpleNamespace(pos=pose[:3] - origin, quat_xyzw=pose[3:])
    return out, origin


def _draw_start(env) -> None:
    """Draw a fresh trial (full beaker, no powder) right after a scene reset, before the first rendered frame, without
    touching the trial's own state."""
    global _S
    if _CFG["beaker"] is None:
        return
    objs, origin = _start_objects(env, _names())
    keep, _S = _S, _State()
    try:
        _draw(objs, origin)
    finally:
        _S = keep


def _patch_embodiment() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline: no embodiment)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_dispersekit", False):
        return
    cls._dispersekit = True
    ensure = cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        ensure(self, *a, **k)
        env = self._env
        if env is None or getattr(env, "_dispersekit_reset", False):
            return
        reset = env.reset

        def reset_and_draw(*ra, **rk):
            out = reset(*ra, **rk)
            try:
                _draw_start(env)
            except Exception as e:  # noqa: BLE001
                print(f"dispersekit: no start visuals ({e!r})", file=sys.stderr, flush=True)
            return out

        env.reset, env._dispersekit_reset = reset_and_draw, True

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
