"""rifflekit: a pan of dry granular material poured into a riffle splitter, for a yam_arena task file (built on
dosekit's materials and its scene hook, which sits on pourkit's and yamkit's; edits none of them).

The simulator has no particles, so the sample is a scored bulk model read from the pan's pose after every control
step. The pan is a rectangular tray (inside `depth` along its x, `width` along its y) with tall walls on three sides
and a low lip along its -x side, the side it pours from. Its contents are kept as `strips` side-by-side strips across
its width, each running from the lip to the back wall.

- What a strip holds. Sand stands at its angle of repose: inward from the lip's top, the steepest stable surface rises
  at the repose angle from the horizontal, so in the pan's frame it slopes by (repose - the pan's tilt toward the lip).
  A strip holds what fits under that surface (and its walls); the rest stands above it by a head h at the lip. The
  same is done against the back wall for a pan tipped the other way.
- Flow over the lip. Each strip pours over its own width w of lip at a weir law, q = C sqrt(g) w h^1.5: the flow of a
  granular layer over an edge, like its flow out of a slot (Beverloo), goes as the head to the 1.5 power; C is
  Beverloo's 0.58 (`C_WEIR`). Never more than stands above the stable surface leaves in a step.
- Sideways. Sand moving down a pan that is also tipped sideways runs along the floor's steepest downhill line, so each
  strip's stream leaves the lip shifted toward the low end by its travel to the lip (a third of the pan's depth) times
  the ratio of the sideways to the forward slope. Between strips, the surface is held up to the angle of repose; past
  it the higher strip slides into the lower one (a pan lifted by one end empties toward that corner).
- Where it lands. Each stream falls plumb, spread across its strip's width. Inside the splitter's hopper it goes down
  the chute under it (up to `END_FLANGE` past either end of the row, on the hopper's end flange, the end chute): the
  chutes sit side by side along y and discharge alternately to the two catch pans. A stream
  that misses the hopper but falls inside a catch pan goes straight into that pan unsplit (`bypass`); anything else
  is spilled on the table at that spot.

Scorer API (after the kit updated for the step, inside the scene's object_states): `chutes(objects)` grams that went
down each chute, `catch(objects)` grams in each catch pan (chutes plus bypass), `bypass(objects)`, `spilled(objects)`,
`in_pan(objects)`, `through(objects)` (sum of the chutes), `sample()` (grams at the start), `flowing(objects)`.
All reset with kit.episode(). Tests call `update(objects)` per step with hand-made ObjectStates.

Visuals (Isaac Lab VisualizationMarkers; they show in the cameras, never affect scoring): the sand in the pan as columns
tilting with it, one stream per pouring strip, the sand in each catch pan as one ridge per chute outlet (its height
the grams that chute delivered), spill piles as flat disks.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from types import SimpleNamespace

import numpy as np

import dosekit as dk
import pourkit as pk
import yamkit as kit

ML = 1e6  # mL per cubic meter
G = 9.81
C_WEIR = 0.58  # Beverloo's discharge coefficient (flow out of a slot or over an edge)
NX = 20  # cells along a strip, for the volume under a surface
DUMP_GZ = 0.15  # a pan turned past about 81 degrees: everything runs out over its lowest edge at DUMP_RATE
DUMP_RATE = 400.0  # mL/s
END_FLANGE = 0.006  # m: a stream up to this far past either end of the chute row lands on the hopper's end flange
# and slides into the end chute (a hopper's end plates funnel stray sand in, so only a pour past them spills)
LAND_ABOVE = 0.03  # m: a stream let go up to this far below the hopper's top, inside its mouth, still lands in it
PILE_MERGE = 0.015  # m
MAX_PILES = 12
SUB = (-0.4, -0.2, 0.0, 0.2, 0.4)  # sub-streams across a strip's width, as fractions of it
TRAVEL = 1.0 / 3.0  # of the pan's depth: how far the sand that pours has slid along the floor to the lip
SLIDE_SWEEPS = 6  # sideways relaxation passes per step


@dataclass(frozen=True)
class Pan:
    """`name` the pan's object; `depth` x `width` its inside (along its x and y), floor at `floor` above its origin;
    `lip` the height of its -x edge above the floor, `wall` that of the other three; `fill` grams of `material` at the
    start, spread evenly; `strips` across its width."""

    name: str
    depth: float
    width: float
    floor: float
    lip: float
    wall: float
    fill: float
    material: str = "sand"
    strips: int = 24


@dataclass(frozen=True)
class CatchPan:
    """A fixed receptacle: inside `size` (x, y) centered at `center` (world x, y), inside floor at world `floor`."""

    name: str
    center: tuple
    size: tuple
    floor: float


@dataclass(frozen=True)
class Splitter:
    """A fixed riffle splitter: hopper mouth centered at `center` (world x, y), `mouth` m across x, its top at world
    `top`; `chutes` chutes of `chute_w` with `vane` thick dividers side by side along y; chute k discharges into
    catch[k % 2] (the first chute is the one at -y)."""

    center: tuple
    mouth: float
    top: float
    chutes: int
    chute_w: float
    vane: float
    catch: tuple

    @property
    def pitch(self) -> float:
        return self.chute_w + self.vane

    @property
    def row(self) -> float:
        return self.chutes * self.chute_w + (self.chutes + 1) * self.vane


_CFG: dict = {"pan": None, "splitter": None, "dt": 0.05, "floor_z": 0.0}
_S = None


def setup(pan: Pan, splitter: Splitter, control_hz: float = 20.0, floor_z: float = 0.0) -> None:
    global _S
    if pan.material not in dk.MATERIALS:
        raise ValueError(f"unknown material {pan.material!r}")
    _CFG.update(pan=pan, splitter=splitter, dt=1.0 / control_hz, floor_z=float(floor_z),
                mat=dk.MATERIALS[pan.material])
    _CFG["d"] = (np.arange(NX) + 0.5) / NX * pan.depth  # distance of each cell from the edge
    _CFG["ws"] = pan.width / pan.strips
    _CFG["y"] = (np.arange(pan.strips) + 0.5) * _CFG["ws"] - pan.width / 2
    _S = _State()


class _State:
    def __init__(self):
        sp, pan = _CFG["splitter"], _CFG["pan"]
        self.episode = kit.episode()
        self.step = 0
        self.v = np.full(pan.strips, pan.fill / _CFG["mat"].loose / pan.strips) if pan else np.zeros(0)  # mL
        self.chutes = np.zeros(sp.chutes if sp else 0)
        self.catch = {c.name: 0.0 for c in sp.catch} if sp else {}
        self.bypass = {c.name: 0.0 for c in sp.catch} if sp else {}
        self.spill = 0.0
        self.piles = []  # [x, y, grams]
        self.streams = []  # (top xyz, bottom z, mL/s)
        self.profile = None  # (edge sign, heights NX x strips) of the surface drawn
        self.flowing = False


def _state():
    global _S
    if _S is None or _S.episode != kit.episode():
        _S = _State()
    return _S


# -- the model ---------------------------------------------------------------------------------


def _rotm(q) -> np.ndarray:
    return pk._rotm(q)


def _strip_vol(base: np.ndarray, s: np.ndarray, cap: float) -> np.ndarray:
    """mL in each strip under the surface base + s (base: NX cells; s: one raise per strip)."""
    pan = _CFG["pan"]
    h = np.clip(base[:, None] + s[None, :], 0.0, cap)
    return h.sum(axis=0) * (pan.depth / NX) * _CFG["ws"] * ML


def _raise(base: np.ndarray, v: np.ndarray, cap: float) -> np.ndarray:
    lo, hi = np.full(v.shape, -0.3), np.full(v.shape, 0.3)
    for _ in range(36):
        mid = (lo + hi) / 2
        low = _strip_vol(base, mid, cap) < v
        lo, hi = np.where(low, mid, lo), np.where(low, hi, mid)
    return (lo + hi) / 2


def _land(p) -> tuple:
    """Where material let go at world point p ends up: ("chute", k), ("bypass", name) or ("spill", None)."""
    sp = _CFG["splitter"]
    x, y, z = float(p[0]), float(p[1]), float(p[2])
    cx, cy = sp.center
    if abs(x - cx) <= sp.mouth / 2 and abs(y - cy) <= sp.row / 2 + END_FLANGE and z >= sp.top - LAND_ABOVE:
        k = int((y - (cy - sp.row / 2)) // sp.pitch)
        return "chute", min(max(k, 0), sp.chutes - 1)
    for c in sp.catch:
        if abs(x - c.center[0]) <= c.size[0] / 2 and abs(y - c.center[1]) <= c.size[1] / 2 and z > c.floor:
            return "bypass", c.name
    return "spill", None


def _deliver(st: _State, p, ml: float) -> float:
    """Move `ml` let go at world point p; returns the z it falls to (for the stream's drawing)."""
    sp, rho = _CFG["splitter"], _CFG["mat"].loose
    grams = ml * rho
    where, key = _land(p)
    if where == "chute":
        st.chutes[key] += grams
        st.catch[sp.catch[key % 2].name] += grams
        return sp.top
    if where == "bypass":
        st.bypass[key] += grams
        st.catch[key] += grams
        return next(c.floor for c in sp.catch if c.name == key)
    st.spill += grams
    for pile in st.piles:
        if math.hypot(pile[0] - p[0], pile[1] - p[1]) < PILE_MERGE:
            pile[2] += grams
            break
    else:
        st.piles.append([float(p[0]), float(p[1]), grams])
    return _CFG["floor_z"]


def _slide(st: _State, g: np.ndarray, phi: float) -> None:
    """Sideways: where the surface between two neighboring strips leans past the angle of repose in the world, sand
    slides from the higher strip into the lower one until it no longer does."""
    pan, ws = _CFG["pan"], _CFG["ws"]
    area = pan.depth * ws * ML  # mL per m of mean depth in a strip
    tan_phi = math.tan(phi)
    for _ in range(SLIDE_SWEEPS):
        z = st.v / area  # each strip's mean depth (pan frame)
        hw = _CFG["y"] * g[1] + z * g[2]  # its surface's height in the world (the y part; x is shared)
        moved = False
        for j in range(len(z) - 1):
            dh = hw[j + 1] - hw[j]
            over = abs(dh) - tan_phi * ws * max(g[2], 1e-3)
            if over <= 1e-6:
                continue
            a, b = (j + 1, j) if dh > 0 else (j, j + 1)  # from the higher to the lower
            t = min(st.v[a], over / max(g[2], 1e-3) / 2 * area)
            st.v[a] -= t
            st.v[b] += t
            moved = moved or t > 1e-6
        if not moved:
            break
    cap = pan.wall * area  # nothing stands above the walls
    st.v = np.minimum(st.v, cap)


def update(objects) -> None:
    """Advance the sample by one control step from the pan's pose in `objects`."""
    st = _state()
    st.step += 1
    st.streams = []
    st.flowing = False
    pan, dt = _CFG["pan"], _CFG["dt"]
    o = objects.get(pan.name)
    if o is None:
        return
    if st.v.sum() <= 1e-6:
        st.profile = None
        return
    R = _rotm(o.quat_xyzw)
    org = np.asarray(o.pos, float)
    g = R.T @ np.array([0.0, 0.0, 1.0])  # world up in the pan's frame
    phi = math.radians(_CFG["mat"].repose)
    ws = _CFG["ws"]

    def world(sign, y, h):  # a point on the -x (sign -1) or +x (sign 1) edge's top
        return org + R @ np.array([sign * pan.depth / 2, y, pan.floor + h])

    if g[2] < DUMP_GZ:  # on its side or upside down: it all runs out over its lowest edge at DUMP_RATE
        ml = min(float(st.v.sum()), DUMP_RATE * dt)
        share = st.v * (ml / float(st.v.sum()))
        st.v = st.v - share
        mids = {(-1, 0): world(-1, 0.0, pan.lip), (1, 0): world(1, 0.0, pan.wall),
                (0, -1): org + R @ np.array([0.0, -pan.width / 2, pan.floor + pan.wall]),
                (0, 1): org + R @ np.array([0.0, pan.width / 2, pan.floor + pan.wall])}
        sx, sy = min(mids, key=lambda k: float(mids[k][2]))
        if sx:  # over a long edge: each strip where it meets that edge
            for j in np.nonzero(share > 1e-9)[0]:
                p = world(sx, float(_CFG["y"][j]), pan.lip if sx < 0 else pan.wall)
                st.streams.append((p, _deliver(st, p, float(share[j])), float(share[j]) / dt))
        else:  # over an end wall: all of it off that end
            p = mids[(0, sy)]
            st.streams.append((p, _deliver(st, p, ml), ml / dt))
        st.flowing = True
        st.profile = None
        return
    before = st.v.copy()
    _slide(st, g, phi)
    over_wall = float(before.sum() - st.v.sum())
    if over_wall > 1e-9:  # a strip filled past the walls: over the low end's side wall
        end = 1 if g[1] > 0 else -1  # the low end is the side world-down points to (-g_y)
        p = org + R @ np.array([0.0, -end * pan.width / 2, pan.floor + pan.wall])
        _deliver(st, p, over_wall)
    best = None
    for sign, h_edge in ((-1, pan.lip), (1, pan.wall)):
        n = np.array([sign, 0.0, 0.0])
        alpha = math.atan2(-float(g @ n), float(g[2]))  # tilt toward this edge (positive: the edge is down)
        base = h_edge + math.tan(max(-1.48, min(1.48, phi - alpha))) * _CFG["d"]
        s = _raise(base, st.v, pan.wall)
        if best is None or s.max() > best[2].max():
            best = (sign, base, s)
        if s.max() <= 0.0:
            continue
        excess = st.v - _strip_vol(base, np.zeros_like(s), pan.wall)
        q = C_WEIR * math.sqrt(G) * ws * np.clip(s, 0.0, None) ** 1.5 * dt * ML
        q = np.minimum(q, np.clip(excess, 0.0, None))
        fwd = -float(g @ n)  # downhill slope toward the edge
        shift = 0.0 if fwd <= 1e-6 else max(-pan.depth, min(pan.depth, float(g[1]) / -fwd * TRAVEL * pan.depth))
        # downhill along y is -g_y: a strip's sand leaves the edge shifted that way by its travel x the slope ratio
        for j in np.nonzero(q > 1e-9)[0]:
            ml = float(q[j])
            st.v[j] -= ml
            y = float(np.clip(_CFG["y"][j] + shift, -pan.width / 2 + ws / 2, pan.width / 2 - ws / 2))
            for f in SUB:
                zb = _deliver(st, world(sign, y + f * ws, h_edge), ml / len(SUB))
            st.streams.append((world(sign, y, h_edge), zb, ml / dt))
            st.flowing = True
    st.v = np.maximum(st.v, 0.0)
    sign, base, s = best
    st.profile = (sign, np.clip(base[:, None] + s[None, :], 0.0, pan.wall))


# -- scorer API ----------------------------------------------------------------------------------


def sample() -> float:
    """Grams in the pan at the start."""
    return float(_CFG["pan"].fill)


def chutes(objects=None) -> np.ndarray:
    return _state().chutes.copy()


def catch(objects=None) -> dict:
    return dict(_state().catch)


def bypass(objects=None) -> float:
    return float(sum(_state().bypass.values()))


def spilled(objects=None) -> float:
    return float(_state().spill)


def in_pan(objects=None) -> float:
    return float(_state().v.sum() * _CFG["mat"].loose)


def through(objects=None) -> float:
    return float(_state().chutes.sum())


def flowing(objects=None) -> bool:
    return bool(_state().flowing)


def step() -> int:
    return _state().step


# -- visuals -------------------------------------------------------------------------------------

_VIS = {"markers": None, "failed": False}
VX = 5  # the sand in the pan drawn as 5 columns along each pair of strips


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        rgb = tuple(kit._linear(c) for c in _CFG["mat"].color)
        surf = sim_utils.PreviewSurfaceCfg(diffuse_color=rgb, roughness=0.9)
        protos = {"grain": sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=surf),
                  "pile": sim_utils.CylinderCfg(radius=1.0, height=1.0, axis="Z", visual_material=surf)}
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/rifflekit",
                                                                       markers=protos))
    except Exception as e:  # visuals are optional: scoring never depends on them
        print(f"rifflekit: no visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _draw(objects, origin) -> None:
    m = _markers()
    if m is None:
        return
    st, pan, sp = _state(), _CFG["pan"], _CFG["splitter"]
    rho = _CFG["mat"].loose
    tr, qs, sc, ids = [], [], [], []

    def put(c, q, s, i=0):
        tr.append(tuple(np.asarray(c, float) + origin)), qs.append(tuple(q)), sc.append(tuple(s)), ids.append(i)

    def hide(i=0):
        put((0.0, 0.0, -10.0), (0, 0, 0, 1), (1e-5,) * 3, i)

    o = objects.get(pan.name)
    pairs = pan.strips // 2
    if o is not None and st.profile is not None:  # the sand in the pan: columns over blocks of cells, two strips wide
        R = _rotm(o.quat_xyzw)
        q = dk._quat_from_rot(R)
        sign, h = st.profile
        bx = NX // VX
        for i in range(VX):
            for j in range(pairs):
                hh = float(h[i * bx:(i + 1) * bx, 2 * j:2 * j + 2].mean())
                if hh < 5e-4:
                    hide()
                    continue
                lx = sign * (pan.depth / 2 - (i + 0.5) / VX * pan.depth)
                ly = (j + 0.5) * 2 * _CFG["ws"] - pan.width / 2
                c = np.asarray(o.pos, float) + R @ np.array([lx, ly, pan.floor + hh / 2])
                put(c, q, (pan.depth / VX * 0.98, 2 * _CFG["ws"] * 0.98, hh))
    else:
        for _ in range(VX * pairs):
            hide()
    # streams: one per pouring segment, from the edge to where it lands; thicker for more flow
    nseg = 2 * pan.strips + 1
    for k in range(nseg):
        if k < len(st.streams):
            p, zb, rate = st.streams[k]
            t = min(0.008, 0.0015 + 0.0006 * math.sqrt(rate))
            length = max(float(p[2]) - zb, 1e-4)
            put((p[0], p[1], zb + length / 2), (0, 0, 0, 1), (t, 0.006, length))
        else:
            hide()
    # each catch pan: a ridge under each chute that feeds it, plus what bypassed the chutes spread over its floor
    for ci, c in enumerate(sp.catch):
        area = c.size[0] * 0.9 * sp.pitch * 1.8
        for k in range(ci, sp.chutes, 2):
            hh = st.chutes[k] / rho / ML / area
            if hh < 2e-4:
                hide()
                continue
            y = sp.center[1] - sp.row / 2 + sp.vane + (k + 0.5) * sp.pitch
            put((c.center[0], y, c.floor + hh / 2), (0, 0, 0, 1), (c.size[0] * 0.9, sp.pitch * 1.8, hh))
        hb = st.bypass[c.name] / rho / ML / (c.size[0] * c.size[1])
        if hb < 2e-4:
            hide()
        else:
            put((*c.center, c.floor + hb / 2), (0, 0, 0, 1), (c.size[0] * 0.95, c.size[1] * 0.95, hb))
    for k in range(MAX_PILES):
        if k < len(st.piles):
            x, y, grams = st.piles[k]
            r = min(0.04, math.sqrt(grams / rho / ML / (math.pi * 0.003)))
            put((x, y, _CFG["floor_z"] + 0.0015), (0, 0, 0, 1), (r, r, 0.003), 1)
        else:
            hide(1)
    m.visualize(translations=np.array(tr, float), orientations=np.array(qs, float), scales=np.array(sc, float),
                marker_indices=np.array(ids, int))


# -- hook into yam_arena.scene (after yamkit's, pourkit's and dosekit's) ----------------------------


def _patch(scene) -> None:
    if getattr(scene, "_rifflekit", False):
        return
    scene._rifflekit = True
    inner = scene.object_states
    last = {"key": None}

    def object_states(env, names):
        out = inner(env, names)
        base = env.unwrapped
        buf = getattr(base, "episode_length_buf", None)
        key = (kit.episode(), int(buf[0].item()) if buf is not None else None)
        if _CFG["pan"] is None:
            return out
        if key[1] is None or key != last["key"]:  # once per control step
            last["key"] = key
            update(out)
            try:
                _draw(out, base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                if not _VIS["failed"]:
                    print(f"rifflekit: visuals off ({e!r})", file=sys.stderr, flush=True)
                _VIS["failed"] = True
        return out

    scene.object_states = object_states


def _draw_start(env) -> None:
    """Draw the full pan (and empty catch pans) right after a scene reset, before the settle steps and the first
    rendered frame: the step hook only draws after a control step's render, so without this the first two frames
    show an empty pan."""
    global _S
    pan = _CFG["pan"]
    if pan is None:
        return
    base = env.unwrapped
    origin = base.scene.env_origins[0].detach().cpu().numpy()
    pose = base.arena_world.get_pose_w(pan.name)[0].detach().cpu().numpy().astype(np.float64)
    o = SimpleNamespace(pos=pose[:3] - origin, quat_xyzw=pose[3:])
    keep, _S = _S, _State()  # a fresh sample, drawn without touching the trial's own state
    try:
        update({pan.name: o})
        _draw({pan.name: o}, origin)
    finally:
        _S = keep


def _patch_embodiment() -> None:
    try:
        from yam_arena import inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline tests: no embodiment)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_rifflekit", False):
        return
    cls._rifflekit = True
    ensure = cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        ensure(self, *a, **k)
        env = self._env
        if env is None or getattr(env, "_rifflekit_reset", False):
            return
        reset = env.reset

        def reset_and_draw(*ra, **rk):
            out = reset(*ra, **rk)
            try:
                _draw_start(env)
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                print(f"rifflekit: no start visuals ({e!r})", file=sys.stderr, flush=True)
            return out

        env.reset = reset_and_draw
        env._rifflekit_reset = True

    cls._ensure_sim = _ensure_sim


_patch_embodiment()


class _Hook(dk._Hook):
    def exec_module(self, module):
        super().exec_module(module)
        _patch(module)


if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
else:  # replace dosekit's import hook with one that also adds the riffle step after theirs
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook())
