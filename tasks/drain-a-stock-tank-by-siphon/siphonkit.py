"""siphonkit: starting a siphon and keeping its prime, for a yam_arena task file (built on yamkit and pourkit: the
source and receiver are pourkit `Vessel`s; edits neither).

No fluids in the sim, so this is a scored model read from poses after every control step, after pourkit's own step.

State (per trial, all reset when kit.episode() changes): `fill` (mL of water in the hose), `primed`, the prime
losses (latched, with step and reason), mL flowed, the step's flow rate.

Geometry. A `Siphon` names the pourkit source and receiver vessels and three kinds of `Point` (a task object or
gripper and an offset in its frame): the hose's `intake` and `outlet` ends and optional `via` points along the hose
(where it crosses a rim; used for drawing and for the submerge check). The hose has inner `radius` and `length`.
A vessel's surface is its pourkit volume laid up its stacked sections from the floor (taken as upright).

Laws (per control step, dt = 1/control_hz):
- In the water. The intake is "in" when it is inside the source's sections and at least `margin` below its surface
  (shallower, a siphon draws a vortex and gulps air). The outlet's level is the surface of whichever vessel's liquid
  it is under (it is submerged there), else its own height.
- Filling the hose (two stated priming actions; a task picks one):
  * prime="bulb": a squeeze bulb at the intake. `strokes(objects)` returns the bulb's stroke count so far (e.g. a
    mechkit Pump link: lambda o: mechkit.state("bulb/plunger").get("strokes", 0)). Each new stroke while the intake
    is in pushes `per_stroke` mL into the hose.
  * prime="submerge": the whole hose (intake, outlet and every via point) under the source's surface fills it in
    `submerge_s` seconds (linear).
  An unprimed, partly or fully filled hose drains back at once if the intake is out, or if the outlet is above the
  source's surface, uncapped, on a step with no new stroke.
- Primed. A full hose (fill >= hose volume) with the intake in and the outlet's level at or below the source's
  surface (or capped) is primed. A hose filled by submerging is primed while still under; lifting its outlet out
  over the rim uncapped then loses the prime (the real reason for the thumb over the end).
- Flow. While primed, uncapped, and the intake in: Q = k sqrt(max(0, z_src - z_out)) mL/s from the source, well
  mixed, through pourkit's transfer log (pk.pours / pk.poured see it), into the vessel the outlet is inside (source
  or receiver, under or above its liquid), or the one its stream falls into (pourkit's plumb landing), else spilled. k = CD A sqrt(2 g), CD = 1/sqrt(1 + K_MINOR + F L/D)
  (Darcy friction F = 0.03 for smooth vinyl at these flows, entry plus exit K_MINOR = 1.5): a 3/8" ID, 1 m hose
  gives k ~ 135 mL/s per sqrt(m), 60 mL/s at 20 cm head (about 3.6 L/min, in line with aquarium siphons).
- Losing the prime (latched in losses(); the hose empties, fill = 0): the intake out of the water ("intake_out" when
  the intake moved, "drawn_down" when the source fell to the intake by itself: the normal end of a drain), or the
  outlet above the source's surface while uncapped ("outlet_high"; flow reverses and air gets in). A capped outlet
  (`capped(objects)` true: a thumb or a pinch clamp) holds the column: no flow and no loss from its height.
  A lost siphon can be primed again; lost_prime() stays true.

Scorer API (after the kit updated for the step): primed(), flowed() mL this trial, flow() mL/s this step,
lost_prime(reason=None) bool (reasons "intake_out", "drawn_down", "outlet_high"), losses() [(step, reason)],
fill() mL in the hose, step(). Volumes in the vessels come from pourkit (pk.volume, pk.poured).

Drawn (Isaac Lab VisualizationMarkers; cameras see them, never affect scoring): the hose along intake -> vias ->
outlet, blue where it holds water (filled from the intake while priming, all of it when primed), pale where dry, and
the stream from the outlet to where it lands while it flows. Drawn right after every scene reset, before the first
rendered frame, together with pourkit's vessels (pourkit alone draws only from the first step), and after every step.

Limits: vessels taken as upright for their surfaces; the hose is not a physical body (give the intake cane and
outlet tube props of their own); no flow inertia (starts and stops at once); a bulb stroke counts whole.

Usage in task.py (after yamkit and pourkit's setup; import this kit last):
    import siphonkit as sk
    sk.setup(sk.Siphon(source="tank", receiver="bucket", intake=sk.Point("cane", (0, 0, -0.15)),
                       outlet=sk.Point("outlet", (0, 0, -0.05)), via=(sk.Point("cane", (0, 0, 0.05)),),
                       prime="bulb", strokes=lambda o: mk.state("bulb/plunger").get("strokes", 0)))
Tests call pk.update(objects) then sk.update(objects) per step with hand-made ObjectStates.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

import pourkit as pk
import yamkit as kit

ML, G, F, K_MINOR = 1e6, 9.81, 0.03, 1.5


@dataclass(frozen=True)
class Point:
    """A point `at` in task object `name`'s frame."""

    name: str
    at: tuple = (0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Siphon:
    source: str
    intake: Point
    outlet: Point
    receiver: str | None = None
    via: tuple = ()
    radius: float = 0.0048  # 3/8" ID vinyl
    length: float = 1.0
    prime: str = "bulb"  # or "submerge"
    strokes: object = None  # function(objects) -> cumulative bulb strokes
    per_stroke: float = 25.0  # mL per squeeze (a siphon-starter bulb)
    submerge_s: float = 1.0
    capped: object = None  # function(objects) -> outlet sealed (thumb, clamp)
    margin: float = 0.005  # m the intake must be under the surface
    # (from drain-a-stock-tank-by-siphon) `sink`: the points that must be under the source's surface to fill the hose
    # by submerging (default: intake, outlet and every via); a capped outlet traps the air, so it never fills then.
    # `rim`: a via used (drawn) only while the outlet is outside the source, where the hose crosses its rim.
    # `pulled(objects)`: true while the intake end is taken as dragged up to the rim by the hose (e.g. let go while
    # the other end is carried away): the intake is then out of the water, at `rim`.
    sink: tuple | None = None
    # `trapped(objects)`: mL of air left in a high point of a sunk hose (e.g. a hose run that stands above the surface);
    # the hose fills by submerging only while that is at most `trapped_max` (what the starting flow sweeps out).
    trapped: object = None
    trapped_max: float = 0.0
    rim: Point | None = None
    pulled: object = None

    @property
    def hose_ml(self) -> float:
        return math.pi * self.radius ** 2 * self.length * ML

    @property
    def k(self) -> float:
        """mL/s per sqrt(m) of head."""
        cd = 1.0 / math.sqrt(1.0 + K_MINOR + F * self.length / (2 * self.radius))
        return cd * math.pi * self.radius ** 2 * math.sqrt(2 * G) * ML


_CFG = {"siphon": None, "dt": 0.05, "color": (0.45, 0.70, 0.95), "draw_vessels": True}


class _State:
    def __init__(self):
        self.episode = kit.episode()
        self.step = 0
        self.fill = 0.0
        self.primed = False
        self.losses = []
        self.flowed = 0.0
        self.flow = 0.0
        self.strokes = None
        self.under_s = 0.0
        self.last_intake_z = None
        self.path = []  # world points intake .. outlet, for the visuals
        self.stream = None  # (outlet xyz, landing z)


_S = _State()


def setup(siphon: Siphon, control_hz: float = 20.0, color=(0.45, 0.70, 0.95), draw_vessels: bool = True) -> None:
    """`draw_vessels`: also draw pourkit's vessels at each reset (pourkit draws them only from the first step)."""
    global _S
    if siphon.prime not in ("bulb", "submerge"):
        raise ValueError("prime is 'bulb' or 'submerge'")
    _CFG.update(siphon=siphon, dt=1.0 / control_hz, color=color, draw_vessels=draw_vessels)
    _S = _State()


def _state() -> _State:
    global _S
    if _S.episode != kit.episode():
        _S = _State()
    return _S


def _world(objects, p: Point):
    o = objects.get(p.name)
    if o is None:
        return None
    return np.asarray(o.pos, float) + pk._rotm(o.quat_xyzw) @ np.asarray(p.at, float)


def _vessel(name):
    return pk._CFG["vessels"][name]


def _level(v, ml: float) -> float:
    """Height of `ml` above the floor along the vessel's axis."""
    left, z = ml / ML, 0.0
    for r, h in v.sections:
        a = math.pi * r * r
        if left <= a * h:
            return z + left / a
        left -= a * h
        z += h
    return z


def _surface_z(objects, name) -> float:
    v, o = _vessel(name), objects[name]
    return float((np.asarray(o.pos, float) + pk._rotm(o.quat_xyzw) @ np.array([0, 0, v.floor + _level(v, pk.volume(objects, name))]))[2])


def _inside(objects, name, p) -> bool:
    """Point p (world) inside vessel `name`'s sections."""
    if p is None or name is None or name not in objects:
        return False
    v, o = _vessel(name), objects[name]
    q = pk._rotm(o.quat_xyzw).T @ (p - np.asarray(o.pos, float))
    a, z0 = q[2] - v.floor, 0.0
    for r, h in v.sections:
        if z0 <= a <= z0 + h:
            return math.hypot(q[0], q[1]) < r
        z0 += h
    return False


def _under(objects, name, p, margin=0.0) -> bool:
    return _inside(objects, name, p) and p[2] < _surface_z(objects, name) - margin


def update(objects) -> None:
    """Advance one control step (call after pourkit's update for the step)."""
    st, s = _state(), _CFG["siphon"]
    if s is None or s.source not in objects:
        return
    st.step += 1
    dt = _CFG["dt"]
    pi, po = _world(objects, s.intake), _world(objects, s.outlet)
    pulled = bool(s.pulled(objects)) if s.pulled is not None else False
    rim = _world(objects, s.rim) if s.rim is not None else None
    if pulled and rim is not None:
        pi = rim
    vias = [_world(objects, p) for p in s.via]
    k = len(vias) // 2  # vias before the rim are on the intake's side
    if pulled and rim is not None:
        vias = vias[k:]  # the intake hangs at the rim
    elif rim is not None and po is not None and not _inside(objects, s.source, po) and _inside(objects, s.source, pi):
        vias = vias[:k] + [rim] + vias[k:]
    st.path = [p for p in [pi, *vias, po] if p is not None]
    st.flow, st.stream = 0.0, None
    if pi is None or po is None:
        return
    zs = _surface_z(objects, s.source)
    intake_in = not pulled and _under(objects, s.source, pi, s.margin)
    capped = bool(s.capped(objects)) if s.capped is not None else False
    out_in = next((n for n in (s.source, s.receiver) if n and _under(objects, n, po)), None)
    z_out = _surface_z(objects, out_in) if out_in else float(po[2])
    moved = st.last_intake_z is not None and abs(pi[2] - st.last_intake_z) > 0.002
    st.last_intake_z = float(pi[2])
    n_new = 0
    if s.prime == "bulb" and s.strokes is not None:
        total = int(s.strokes(objects))
        n_new = 0 if st.strokes is None else max(0, total - st.strokes)
        st.strokes = total

    def lose(reason):
        st.primed, st.fill = False, 0.0
        st.losses.append((st.step, reason))

    if st.primed:
        if not intake_in:
            lose("intake_out" if moved or pulled or not _inside(objects, s.source, pi) else "drawn_down")
            return
        if z_out > zs and not capped:
            lose("outlet_high")
            return
        if capped:
            return
        have = pk.volume(objects, s.source)
        q = min(have, s.k * math.sqrt(max(0.0, zs - z_out)) * dt)
        dest = out_in or next((n for n in (s.source, s.receiver) if n and _inside(objects, n, po)), None)
        if q <= 1e-9 or dest == s.source:
            return
        if dest is None:
            dest, _ = pk._landing(objects, s.source, po)
            dest = dest if dest is not None else pk.SPILL
        share = {k: v * q / have for k, v in pk.contents(objects, s.source).items() if v > 0}
        pk._apply([(s.source, dest, share)])
        st.flowed += q
        st.flow = q / dt
        land = po[2] if out_in else (_surface_z(objects, dest) if dest != pk.SPILL else pk._floor_z(po))
        # (a vessel's surface: its liquid's top, or its floor when empty)
        st.stream = (po, min(float(po[2]), land))
        return
    # not primed: fill the hose
    if s.prime == "bulb":
        if intake_in and n_new:
            st.fill = min(s.hose_ml, st.fill + n_new * s.per_stroke)
    else:
        sink = [_world(objects, p) for p in s.sink] if s.sink is not None else [po, *vias]
        air_ok = s.trapped is None or float(s.trapped(objects)) <= s.trapped_max
        if intake_in and not capped and air_ok and all(_under(objects, s.source, p) for p in sink if p is not None):
            st.fill = min(s.hose_ml, st.fill + s.hose_ml * dt / s.submerge_s)
    if st.fill > 0 and not capped and (not intake_in or (z_out > zs and n_new == 0)):
        st.fill = 0.0  # drains back
    if st.fill >= s.hose_ml - 1e-9 and intake_in and (z_out <= zs or capped):
        st.primed = True


# -- scorer API --------------------------------------------------------------------------------


def primed(objects=None) -> bool:
    return bool(_state().primed)


def flowed(objects=None) -> float:
    """mL siphoned out of the source this trial."""
    return float(_state().flowed)


def flow(objects=None) -> float:
    """mL/s this step."""
    return float(_state().flow)


def lost_prime(objects=None, reason: str | None = None) -> bool:
    """Whether the prime was lost this trial (latched); `reason` limits it to one cause. lost_prime(objects),
    lost_prime(objects, "intake_out") and lost_prime("intake_out") all work."""
    if isinstance(objects, str):
        objects, reason = None, objects
    return any(r == reason or reason is None for _, r in _state().losses)


def losses(objects=None) -> list:
    return list(_state().losses)


def fill(objects=None) -> float:
    return float(_state().fill)


def step() -> int:
    return _state().step


# -- visuals -----------------------------------------------------------------------------------

_VIS = {"markers": None, "failed": False}
PROTOS = ("water", "hose")
MAXSEG = 5
SLOTS = {"water": MAXSEG + 1, "hose": MAXSEG}


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
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/siphonkit", markers=protos))
    except Exception as e:  # visuals are optional: scoring never depends on them
        print(f"siphonkit: no visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _quat_z_to(d) -> tuple:
    d = np.asarray(d, float) / np.linalg.norm(d)
    c = float(d[2])
    if c < -0.999999:
        return (1.0, 0.0, 0.0, 0.0)
    w = math.sqrt((1 + c) / 2)
    ax = np.cross([0.0, 0.0, 1.0], d) / (2 * w)
    return (float(ax[0]), float(ax[1]), float(ax[2]), w)


def _seg(a, b, r):
    a, b = np.asarray(a, float), np.asarray(b, float)
    L = float(np.linalg.norm(b - a))
    return None if L < 1e-5 else ((a + b) / 2, _quat_z_to(b - a), (r, r, L))


def _items(objects) -> dict:
    st, s = _state(), _CFG["siphon"]
    out = {"water": [], "hose": []}
    path = st.path[:MAXSEG + 1]
    if s is None or len(path) < 2:
        return out
    lens = [float(np.linalg.norm(b - a)) for a, b in zip(path, path[1:])]
    wet = sum(lens) * (1.0 if st.primed else min(1.0, st.fill / s.hose_ml))
    for (a, b), L in zip(zip(path, path[1:]), lens):  # filled from the intake end
        w = min(L, max(0.0, wet))
        wet -= L
        mid = a + (b - a) * (w / L) if L > 0 else a
        out["water"].append(_seg(a, mid, s.radius))
        out["hose"].append(_seg(mid, b, s.radius))
    if st.stream is not None:
        p, z = st.stream
        out["water"].append(_seg((p[0], p[1], z), p, 0.003))
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


# -- hooks: the step (scene.object_states, after pourkit's) and the reset draw ------------------


def _patch(scene) -> None:
    if getattr(scene, "_siphonkit", False):
        return
    scene._siphonkit = True
    inner, last = scene.object_states, {"key": None}

    def object_states(env, names):
        out = inner(env, names)
        base = env.unwrapped
        buf = getattr(base, "episode_length_buf", None)
        key = (kit.episode(), int(buf[0].item()) if buf is not None else None)
        if _CFG["siphon"] is not None and (key[1] is None or key != last["key"]):  # once per control step
            last["key"] = key
            update(out)
            try:
                _draw(out, base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                if not _VIS["failed"]:
                    print(f"siphonkit: visuals off ({e!r})", file=sys.stderr, flush=True)
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
    """Right after a scene reset, before the first rendered frame: pourkit's vessels at their starting contents and an
    empty hose, without touching either kit's trial state."""
    global _S
    s = _CFG["siphon"]
    if s is None:
        return
    names = list(pk._CFG["vessels"]) + [p.name for p in (s.intake, s.outlet, *s.via)]
    objs, origin = _start_objects(env, dict.fromkeys(names))
    if _CFG["draw_vessels"]:
        keep_pk, pk._S = pk._S, pk._State()
        try:
            pk._draw(objs, origin)
        finally:
            pk._S = keep_pk
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
    if getattr(cls, "_siphonkit", False):
        return
    cls._siphonkit = True
    ensure = cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        ensure(self, *a, **k)
        env = self._env
        if env is None or getattr(env, "_siphonkit_reset", False):
            return
        reset = env.reset

        def reset_and_draw(*ra, **rk):
            out = reset(*ra, **rk)
            try:
                _draw_start(env)
            except Exception as e:  # noqa: BLE001
                print(f"siphonkit: no start visuals ({e!r})", file=sys.stderr, flush=True)
            return out

        env.reset, env._siphonkit_reset = reset_and_draw, True

    cls._ensure_sim = _ensure_sim


class _Hook(kit._Hook):
    """Runs the import hook that was first in line (pourkit's chain), then adds this kit's step after it."""

    def __init__(self, prev):
        self.prev = prev

    def exec_module(self, module):
        if self.prev is not None:
            self.prev._inner = self._inner
            self.prev.exec_module(module)
        else:
            self._inner.exec_module(module)
            kit._patch(module)
            pk._patch(module)
        _patch(module)


_patch_embodiment()
if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:
    _prev = next((f for f in sys.meta_path if isinstance(f, kit._Hook)), None)
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook(_prev))
