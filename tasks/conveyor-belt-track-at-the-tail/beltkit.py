"""beltkit: a flat belt conveyor's belt tracking (sideways drift) for a yam_arena task file, on twistkit take-up screws.

The belt is not a physical body. Its sideways offset `e` (m, + toward world +x) follows a scored law each control step,
driven by how square the tail pulley sits, which the robot sets with the two take-up screws at the tail (twistkit
caps turning about the conveyor's length axis). The belt is drawn with Isaac Lab visualization markers.

    import twistkit as tk
    import beltkit as bk
    bk.setup(bk.Belt(near="takeup_near/knob", far="takeup_far/knob", skew0=-0.0055, e0=0.003, ...))

The law (all constants are fields of `Belt`):
- Take-up: turning a screw's knob clockwise, seen from its outer end, draws that end of the tail pulley outward by
  `pitch` per turn (a right-hand screw pulling the bearing block toward the knob). `advance(side)` is that travel from
  the start, clamped to the slot's +-`travel`.
- Skew: sigma = (advance_far - advance_near) / `span` (rad), the tail pulley's angle out of square that the robot
  adds; `skew0` is the frame's own error at the start (a pulley not square to the belt).
- Steering, the classic rule: a belt walks toward the end of a skewed pulley that it meets first, which is the end
  with less tension (the side that was moved in, or not moved out), at about v * tan(skew) (Habasit and ERO tracking
  guides: "the belt tracks toward the side with least tension"; tightening the far side therefore sends it toward the
  near side). So de/dt = -v * (sigma_eff + skew0) - (v / `center_len`) * e. The second term is the crowned pulleys'
  self-centering: with it, a belt on a fixed skew settles at e_ss = -center_len * (sigma + skew0) instead of running
  off for ever, with a time constant T = center_len / v.
- Lag: the belt does not answer a pulley change at once; the effective skew follows the set skew with a first-order
  lag of `lag_revs` belt revolutions (the belt must run over the pulley before the new angle is wrapped into it).
- Rub: the belt touches a side frame when |e| >= `clearance`; `rubbed()` latches for the trial (a frayed edge stays
  frayed) with the side and the step.
- Tension: the belt's stretch (%) is `stretch0` plus the loop length the two take-ups add, (a_near + a_far) / `loop`.

Each control step the kit integrates the law over 10 substeps with the knob angles interpolated between the last
step's and this step's, so a knob spun fast between two steps still steers (and can rub) in proportion.

Scorer API: `offset(objects)` (m), `rate()` (m/s, this step), `settle()` (m: where the belt settles at the current
screw settings, e_ss), `rubbed()`, `rub_side()`, `stretch()` (%), `turns(side)` (turns tightened from the start,
"near"/"far"), `max_rate(seconds)` (largest |de/dt| over the last `seconds`), `adjusted(seconds)` (turns of adjustment made over the
last `seconds`, both screws), `steps()`. All reset with
`kit.episode()`. Offline tests call `update(objects)` per step after `tk.set_angle(...)` with hand-made states.
"""

from __future__ import annotations

import math
import os
import sys
from collections import deque
from dataclasses import dataclass

import numpy as np
import twistkit as tk
import yamkit as kit

DT = 0.05  # s per control step (20 Hz)
SUB = 10  # integration substeps per control step


@dataclass(frozen=True)
class Belt:
    near: str  # twistkit key of the near (robot-side, lower x) take-up knob
    far: str  # twistkit key of the far take-up knob
    start_deg: float  # both knobs' twistkit start (degrees past the seat), the take-ups' start position
    skew0: float  # rad: the tail pulley's own error at the start (negative sends the belt toward +x)
    e0: float = 0.0  # m: the belt's offset at the start
    speed: float = 0.12  # m/s belt speed
    loop: float = 0.85  # m belt loop length
    span: float = 0.17  # m between the two take-up screws
    pitch: float = 0.0015  # m of take-up per screw turn
    travel: float = 0.003  # m: each take-up's slot allows this much either way from the start
    center_len: float = 2.6  # m: crowned-pulley centering length (settled offset per rad of skew)
    lag_revs: float = 1.0  # belt revolutions of lag between a pulley change and the belt's answer
    clearance: float = 0.010  # m between a belt edge and a side frame when centered
    stretch0: float = 0.5  # % stretch at the start
    # drawing (world, m): the belt's top run
    x_center: float = 0.28
    y_tail: float = -0.12
    y_head: float = 0.24
    z_top: float = 0.10
    width: float = 0.12
    frame_x: tuple = (0.205, 0.355)  # the side frames' inner faces (where a wear mark is drawn)
    # the belt-edge sensor's LED bar (world y, z of its face; None: no bar): one LED per mm of offset, +-clearance,
    # centered on x_center, the lit LED at the belt's offset rounded to the mm
    bar: tuple | None = None
    bar_pitch: float = 0.006  # m between LEDs

    @property
    def tau(self) -> float:
        return self.lag_revs * self.loop / self.speed

    def advance(self, deg_past_seat: float) -> float:
        a = self.pitch * (deg_past_seat - self.start_deg) / 360.0
        return min(max(a, -self.travel), self.travel)

    def settle_at(self, sigma: float) -> float:
        return -self.center_len * (sigma + self.skew0)


class _State:
    def __init__(self, b: Belt):
        self.episode = kit.episode()
        self.e = b.e0
        self.sigma_eff = 0.0
        self.sigma = 0.0
        self.deg = None  # (near, far) knob angles past the seat at the last step
        self.rate = 0.0
        self.rates = deque(maxlen=2000)
        self.knobs = deque(maxlen=2000)  # (near, far) knob angles past the seat (deg) at each step
        self.rub = False
        self.rub_side = 0
        self.rub_step = None
        self.step = 0
        self.phase = 0.0  # m the belt has run (for drawing)


_CFG: dict = {"belt": None}
_S: _State | None = None


def setup(belt: Belt) -> None:
    global _S
    _CFG["belt"] = belt
    _S = None


def _state() -> _State:
    global _S
    b = _CFG["belt"]
    if _S is None or _S.episode != kit.episode():
        _S = _State(b)
    return _S


def _sigma(b: Belt, near_deg: float, far_deg: float) -> float:
    return (b.advance(far_deg) - b.advance(near_deg)) / b.span


def update(objects) -> None:
    """One control step of the law (the simulator's hook calls it once per step; offline tests call it directly)."""
    b = _CFG["belt"]
    s = _state()
    now = (tk.past_seat_deg(objects, b.near), tk.past_seat_deg(objects, b.far))
    before = s.deg if s.deg is not None else now
    s.deg = now
    s.step += 1
    s.knobs.append(now)
    h = DT / SUB
    keep = math.exp(-h / b.tau)
    e0 = s.e
    for k in range(1, SUB + 1):
        f = k / SUB
        sig = _sigma(b, before[0] + f * (now[0] - before[0]), before[1] + f * (now[1] - before[1]))
        s.sigma_eff = sig + (s.sigma_eff - sig) * keep
        s.e += h * (-b.speed * (s.sigma_eff + b.skew0) - b.speed * s.e / b.center_len)
        if abs(s.e) >= b.clearance and not s.rub:
            s.rub, s.rub_side, s.rub_step = True, (1 if s.e > 0 else -1), s.step
    s.sigma = _sigma(b, *now)
    s.rate = (s.e - e0) / DT
    s.rates.append(abs(s.rate))
    s.phase = (s.phase + b.speed * DT) % 1000.0


# -- scorer API ---------------------------------------------------------------------------------


def offset(objects=None) -> float:
    """The belt's sideways offset from the frame's center line (m, + toward +x)."""
    return _state().e


def rate() -> float:
    return _state().rate


def max_rate(seconds: float) -> float:
    """The largest |de/dt| (m/s) over the last `seconds` (inf if fewer steps than that have run)."""
    s = _state()
    n = int(round(seconds / DT))
    if len(s.rates) < n:
        return float("inf")
    return max(list(s.rates)[-n:])


def adjusted(seconds: float) -> float:
    """Turns of adjustment made over the last `seconds` (the window's span of each knob's angle, near plus far): a
    screw turned out and back within the window counts its whole excursion."""
    s = _state()
    n = max(1, int(round(seconds / DT)) + 1)
    last = list(s.knobs)[-n:]
    if not last:
        return 0.0
    return sum(max(k[i] for k in last) - min(k[i] for k in last) for i in (0, 1)) / 360.0


def settle() -> float:
    """Where the belt settles (m) if the screws stay as they are now."""
    b = _CFG["belt"]
    return b.settle_at(_state().sigma)


def rubbed() -> bool:
    return _state().rub


def rub_side() -> int:
    return _state().rub_side


def turns(objects, side: str) -> float:
    """Turns the take-up on `side` ("near"/"far") has been tightened from its start (negative: loosened)."""
    b = _CFG["belt"]
    return (tk.past_seat_deg(objects, b.near if side == "near" else b.far) - b.start_deg) / 360.0


def stretch(objects) -> float:
    """The belt's stretch (%) at the current take-up settings."""
    b = _CFG["belt"]
    a = b.advance(tk.past_seat_deg(objects, b.near)) + b.advance(tk.past_seat_deg(objects, b.far))
    return b.stretch0 + 100.0 * a / b.loop


def steps() -> int:
    return _state().step


# -- visuals: the belt's top run, yellow chevrons moving with it, a white center stripe, a red wear mark ----------

_VIS = {"markers": None, "failed": False}
CHEVRON_PITCH = 0.06  # m between chevrons along the belt
CHEVRON_LEG = 0.035  # m: each arm of a chevron


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        def box(rgb, rough=0.7):
            return sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
                diffuse_color=tuple(kit._linear(c) for c in rgb), roughness=rough))

        protos = {"belt": box((0.07, 0.07, 0.08), 0.8), "chevron": box((0.95, 0.78, 0.10), 0.5),
                  "stripe": box((0.95, 0.95, 0.93), 0.6), "wear": box((0.85, 0.10, 0.08), 0.6),
                  "bar": box((0.04, 0.04, 0.05), 0.5), "led": box((0.10, 0.95, 0.20), 0.3)}
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/beltkit",
                                                                       markers=protos))
        _VIS["index"] = {n: i for i, n in enumerate(protos)}
    except Exception as e:  # visuals are optional: scoring never depends on them
        print(f"beltkit: no belt visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _yawq(a):
    return (0.0, 0.0, math.sin(a / 2), math.cos(a / 2))


def _draw(origin, s=None) -> None:
    m = _markers()
    if m is None:
        return
    b, idx = _CFG["belt"], _VIS["index"]
    s = s if s is not None else _state()
    xc = b.x_center + s.e
    length = b.y_head - b.y_tail
    ym = (b.y_head + b.y_tail) / 2
    tr, qs, sc, ids = [], [], [], []

    def add(kind, p, q, size):
        tr.append(tuple(np.asarray(p, float) + origin)), qs.append(q), sc.append(size), ids.append(idx[kind])

    add("belt", (xc, ym, b.z_top + 0.0015), (0, 0, 0, 1), (b.width, length, 0.003))
    add("stripe", (xc, ym, b.z_top + 0.0032), (0, 0, 0, 1), (0.003, length, 0.0004))
    # chevrons pointing along the belt's travel (+y), spaced CHEVRON_PITCH, moving with the belt
    off = s.phase % CHEVRON_PITCH
    ang = math.radians(35.0)
    n = int(length / CHEVRON_PITCH) + 1
    for k in range(n):
        yc = b.y_tail + 0.02 + off + k * CHEVRON_PITCH
        if yc > b.y_head - 0.02:
            yc = -10.0  # off the run: parked out of sight
        for sgn in (-1, 1):
            cx = xc + sgn * (CHEVRON_LEG / 2) * math.cos(ang)
            cy = yc - (CHEVRON_LEG / 2) * math.sin(ang)
            add("chevron", (cx, cy, b.z_top + 0.0034), _yawq(sgn * ang), (CHEVRON_LEG, 0.006, 0.0004))
    if b.bar is not None:  # the edge sensor's LED bar: a dark strip and the lit LED
        n_mm = int(round(b.clearance * 1000))
        yb, zb = b.bar
        add("bar", (b.x_center, yb, zb), (0, 0, 0, 1), ((2 * n_mm + 1) * b.bar_pitch + 0.004, 0.012, 0.002))
        k = max(-n_mm, min(n_mm, int(round(s.e * 1000))))
        add("led", (b.x_center + k * b.bar_pitch, yb, zb + 0.0012), (0, 0, 0, 1), (0.0045, 0.008, 0.0006))
    if s.rub:  # the frayed belt edge's rub mark on the side frame's inner face, along the tail end
        x = b.frame_x[1] if s.rub_side > 0 else b.frame_x[0]
        add("wear", (x - s.rub_side * 0.001, b.y_tail + 0.08, b.z_top + 0.005), (0, 0, 0, 1), (0.003, 0.12, 0.008))
    else:
        add("wear", (0.0, 0.0, -10.0), (0, 0, 0, 1), (1e-5, 1e-5, 1e-5))
    m.visualize(translations=np.array(tr, float), orientations=np.array(qs, float), scales=np.array(sc, float),
                marker_indices=np.array(ids, int))


# -- hook into yam_arena.scene (after yamkit's and twistkit's) ------------------------------------

_orig_patch = kit._patch


def _patch(scene) -> None:
    _orig_patch(scene)
    if getattr(scene, "_beltkit", False):
        return
    scene._beltkit = True
    inner = scene.object_states
    last = {"key": None}

    def object_states(env, names):
        out = inner(env, names)
        if _CFG["belt"] is None:
            return out
        base = env.unwrapped
        buf = getattr(base, "episode_length_buf", None)
        key = (kit.episode(), int(buf[0].item()) if buf is not None else None)
        if key[1] is None or key != last["key"]:  # once per control step
            last["key"] = key
            update(out)
            if os.environ.get("BELTKIT_DEBUG"):
                st = _state()
                print(f"beltkit step {st.step} e={st.e * 1000:.2f}mm rate={st.rate * 1000:.3f}mm/s "
                      f"settle={settle() * 1000:.1f}mm rub={st.rub} near={turns(out, 'near'):.3f} "
                      f"far={turns(out, 'far'):.3f} adj7={adjusted(7.4):.3f}", flush=True)
            try:
                _draw(base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                if not _VIS["failed"]:
                    print(f"beltkit: visuals off ({e!r})", file=sys.stderr, flush=True)
                _VIS["failed"] = True
        return out

    scene.object_states = object_states


kit._patch = _patch


def _hook_reset() -> None:
    """Draw the belt as a new trial starts (offset e0, LED at its mm, no wear mark) right after the env's reset, so
    the first camera frame already shows it; the step hook takes over from the first control step."""
    try:
        import yam_arena.inspect_embodiment as ie
    except Exception:  # noqa: BLE001  (offline tests: no simulator)
        return
    cls = ie.YamArenaEmbodiment
    if getattr(cls, "_beltkit_reset", False):
        return
    cls._beltkit_reset = True
    ensure = cls._ensure_sim

    def _ensure_sim(self, *a, **k):
        ensure(self, *a, **k)
        env = getattr(self, "_env", None)
        if env is None or getattr(env, "_beltkit_reset", False):
            return
        env._beltkit_reset = True
        reset = env.reset

        def reset_and_draw(*ra, **rk):
            out = reset(*ra, **rk)
            try:
                if _CFG["belt"] is not None and not _VIS["failed"]:
                    _draw(self._base.scene.env_origins[0].detach().cpu().numpy(), _State(_CFG["belt"]))
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                print(f"beltkit: no belt at reset ({e!r})", file=sys.stderr, flush=True)
            return out

        env.reset = reset_and_draw

    cls._ensure_sim = _ensure_sim


_hook_reset()
if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
