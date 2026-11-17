"""filterkit: a loose spin-on filter that is threaded onto its nipple, and a free cap wrench on a ratchet (on yamkit).

The filter is a free part: a yamkit `Mechanism` fixed at the nipple whose filter link sits on six joints with no drive
(three slides, then two tilts and the spin about its own axis), so it falls, tips, is carried and set down like any
loose part. Its joints are the chain's coordinates, not a constraint: the filter can be anywhere within reach, at any
tilt up to well past lying on its side. The base frame has its z axis along the nipple, pointing out of the engine,
with the origin at the center of the gasket land; the filter's own frame has its origin on its axis at the gasket's
face, its z axis toward the closed end.

The thread (FilterModel, one control step at a time):
- Free: no drive on any joint. The nipple is a collider and the filter's base plate has a hole around it, so the
  filter can be pushed onto the nipple only centered on it.
- On the nipple (gasket face within NIP_LAT of its axis, tilted under NIP_TILT, pushed on until it is within NIP_REACH
  of the thread's start): the threads' crests stop it at `d0` from the land, where the first thread meets the
  nipple's (a stop on the axial slide); pushing harder does not take it further.
- Started: turned clockwise (seen from its closed end) by START_TURN there. If its axis was within ENGAGE_TILT of the
  nipple's, the thread catches: from then on the slide follows pitch x turns, the side slides and tilts hold it on the
  axis, and the filter turns only about its axis. If it was tilted more, it is cross-threaded: it jams where it is
  (all joints locked) and the trial has failed (a cross-threaded filter damages the nipple's thread).
- Engaged: turning it costs `run_on` (N*m) before gasket contact and the gasket's torque past it (`gasket(past)`), as
  a stick-slip hold: it stays where it is left, and gives way only to that torque, either way (the drive's force cap
  is the torque, its target the last place it slipped to). Unscrewed past the thread's start it is free again.

The wrench: a `Mechanism` with a mass (a floating base: the ratchet's handle and head), and one link, the drive with
its adapter and the cap, on a hinge about the drive's axis. The ratchet is a stop: turned to tighten, the handle
drives the cap (the hinge's upper limit, the pawl, follows the cap back in 3-degree clicks when the handle swings
back, and a drag of `PAWL_DRAG` opposes the freewheeling). Nothing ties the cap to the filter: the cap's flutes (14
teeth inside the cup) drive the filter's 14 flutes by contact, and the cup stays on only while it is pushed over the
filter's end.

Plain Python and NumPy at import time.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np
import yamkit as kit

FREE, ENGAGED, CROSSED = "free", "engaged", "crossed"
NIP_LAT = 0.006  # m: the gasket face's center within 6 mm of the nipple's axis (its hole on the nipple's chamfered end)
NIP_TILT = math.radians(8.0)  # and the filter's axis within 8 degrees of the nipple's
NIP_REACH = 0.003  # m: within 3 mm of where the first threads meet: the crests stop it there
START_TURN = math.radians(20.0)  # turned this far clockwise against the crests: the thread starts (or crosses)
ENGAGE_TILT = math.radians(4.0)  # the axes within 4 degrees: the thread catches; more: it cross-threads
CENTER_RATE = 0.0006  # m per control step: an engaged filter's side slides are brought onto the axis this fast
TILT_RATE = math.radians(1.0)  # rad per control step: and its tilts
K_HOLD = 2000.0  # N*m/rad: the stick-slip hold's stiffness (0.5 degrees of give at 18 N*m)
AX_K = 20000.0  # N/m: the thread's axial stiffness (a drive on top of the stop)
SIDE_K = 20000.0  # N/m and N*m/rad: holding an engaged filter on the axis
CLICK = math.radians(3.0)  # the ratchet's 120 teeth
PAWL_DRAG = 0.2  # N*m


def _wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass(frozen=True)
class Thread:
    pitch: float  # m per turn
    turns_to_seat: float  # turns from the first thread catching to gasket contact
    run_on: float  # N*m to turn it before gasket contact
    gasket_rate: float  # N*m per rad past contact
    max_past: float  # rad past contact: a hard stop (the gasket squeezed flat)
    seat_travel: float = 0.0005  # m the gasket compresses past contact

    @property
    def d0(self) -> float:
        """The gasket face's distance from the land when the first thread catches (m)."""
        return self.pitch * self.turns_to_seat

    def torque(self, past: float) -> float:
        """N*m to turn it further at `past` rad past gasket contact."""
        return self.run_on if past <= 0 else max(self.gasket_rate * past, self.run_on)


class _Free:
    """A placeholder law: gives the link a PhysX drive (set each step by this module's driver) and lets the kit
    restore its limits after a reset."""

    sets_limits = True

    def __init__(self, damping: float = 0.0):
        self.damping = damping

    def drive(self, key, q, qd, J, mem):
        return q, 0.0, 0.0, self.damping


@dataclass(frozen=True)
class LooseFilter(kit.Mechanism):
    """A filter on six joints from its nipple (see the module docstring). `filter_meshes`, `filter_colliders` and
    `filter_mass` are the filter's own (its frame: origin on the axis at the gasket's face, z toward its closed end);
    `start` is (up, side, out, pitch, roll) for the chain: the gasket face's offset from the land's center in the
    base frame (x, y, z) and its tilts (rad, about y then x)."""

    thread: Thread | None = None
    filter_meshes: tuple = ()
    filter_colliders: tuple = ()
    filter_mass: float = 0.25
    start: tuple = (0.0, 0.0, 0.05, 0.0, 0.0)
    reach: tuple = ((-0.15, 0.10), (-0.10, 0.30), (-0.001, 0.25))

    def __post_init__(self):
        o = (0.0, 0.0, 0.0)
        (ul, uh), (sl, sh), (al, ah) = self.reach
        up, side, out, pitch, roll = self.start
        links = (
            kit.Link("f_up", "prismatic", "X", o, lower=ul, upper=uh, start=up, mass=0.005, damping=0.0, law=_Free()),
            kit.Link("f_side", "prismatic", "Y", o, lower=sl, upper=sh, start=side, mass=0.005, damping=0.0,
                     parent="f_up", law=_Free()),
            kit.Link("f_out", "prismatic", "Z", o, lower=al, upper=ah, start=out, mass=0.005, damping=0.0,
                     parent="f_side", law=_Free()),
            kit.Link("f_pitch", "revolute", "Y", o, lower=-1.2, upper=2.2, start=pitch, mass=0.005, damping=0.0,
                     parent="f_out", law=_Free()),
            kit.Link("f_roll", "revolute", "X", o, lower=-1.2, upper=1.2, start=roll, mass=0.005, damping=0.0,
                     parent="f_pitch", law=_Free()),
            kit.Link("filter", "revolute", "Z", o, lower=-60.0, upper=60.0, start=0.0, mass=self.filter_mass,
                     meshes=self.filter_meshes, colliders=self.filter_colliders, damping=0.0, parent="f_roll",
                     law=_Free()),
        )
        object.__setattr__(self, "links", links)


CHAIN = ("f_up", "f_side", "f_out", "f_pitch", "f_roll", "filter")


@dataclass
class FilterModel:
    """The thread's state this trial (see the module docstring). Angles in rad; `turned` is the tightening angle
    since the thread caught (clockwise seen from the closed end), so `turned - seat` is the angle past gasket
    contact."""

    episode: int
    thread: Thread
    state: str = FREE
    s_last: float | None = None
    turned: float = 0.0
    push: float = 0.0  # tightening turn against the crests (before the thread catches)
    anchor: float | None = None  # the spin joint's stick-slip target
    came_off: bool = False  # unscrewed off the thread after it caught
    crossed_tilt: float = 0.0
    on: bool = False  # on the nipple against the threads' crests (not yet started)
    peak_past: float = -1e9  # rad
    lock: dict = field(default_factory=dict)

    @property
    def seat(self) -> float:
        return 2 * math.pi * self.thread.turns_to_seat

    @property
    def past(self) -> float:
        """Rad past gasket contact (-inf while the thread has not caught)."""
        return self.turned - self.seat if self.state == ENGAGED else -math.inf

    def step(self, J: dict) -> dict:
        """Advance one control step from the chain's joints J (by link name). Returns what each joint should do:
        {link: (target, stiffness, effort, damping, effort_limit, (lo, hi) or None)}."""
        th = self.thread
        up, side, out = J["f_up"], J["f_side"], J["f_out"]
        pitch, roll, s = J["f_pitch"], J["f_roll"], J["filter"]
        ds = 0.0 if self.s_last is None else _wrap(s - self.s_last)
        self.s_last = s
        tighten = -ds  # the spin joint turns negative about the nipple's out-pointing axis when tightened
        lat = math.hypot(up, side)
        tilt = math.acos(max(-1.0, min(1.0, math.cos(pitch) * math.cos(roll))))
        free = {n: (J[n], 0.0, 0.0, 0.002 if n == "filter" else 0.01, 1000.0, None) for n in CHAIN}
        if self.state == CROSSED:
            return {n: (self.lock[n], 2000.0, 0.0, 5.0, 1000.0, (self.lock[n] - 1e-3, self.lock[n] + 1e-3))
                    for n in CHAIN}
        if self.state == FREE:
            on = lat < NIP_LAT and tilt < NIP_TILT and out < th.d0 + NIP_REACH
            self.on = on
            if not on:
                self.push = 0.0
                return free
            out_cmd = free["f_out"]
            free["f_out"] = (out_cmd[0], 0.0, 0.0, 0.01, 1000.0, (th.d0, 0.25))  # the crests: no closer than d0
            if out <= th.d0 + 0.0008:
                self.push = max(self.push + tighten, 0.0)
            else:
                self.push = 0.0
            if self.push >= START_TURN:
                if tilt <= ENGAGE_TILT:
                    self.state, self.turned, self.anchor = ENGAGED, 0.0, s
                else:
                    self.state, self.crossed_tilt = CROSSED, tilt
                    self.lock = dict(J)
                    return {n: (J[n], 2000.0, 0.0, 5.0, 1000.0, (J[n] - 1e-3, J[n] + 1e-3)) for n in CHAIN}
            return free
        # engaged
        self.turned += tighten
        if self.turned < -math.radians(10.0):  # unscrewed past where the thread caught: loose again
            self.state, self.came_off, self.turned, self.push = FREE, True, 0.0, 0.0
            return free
        past = self.turned - self.seat
        self.peak_past = max(self.peak_past, past)
        d = max(th.d0 - th.pitch * self.turned / (2 * math.pi), -th.seat_travel)
        out_lim = (min(d, out) - 0.0003, max(d, out) + 0.0003)
        cmd = {"f_out": (d, AX_K, 0.0, 20.0, 400.0, out_lim)}
        for n, rate in (("f_up", CENTER_RATE), ("f_side", CENTER_RATE), ("f_pitch", TILT_RATE),
                        ("f_roll", TILT_RATE)):
            q = J[n]
            lim = max(abs(q) - rate, 0.0002)  # the stop closes on the axis a little each step
            cmd[n] = (0.0, SIDE_K, 0.0, 20.0, 400.0, (-lim, lim))
        tau = th.torque(past)
        if self.anchor is None:
            self.anchor = s
        self.anchor = min(max(self.anchor, s - tau / K_HOLD), s + tau / K_HOLD)
        lo_spin = None
        if past >= th.max_past:  # the gasket squeezed flat: a hard stop
            lo_spin = (s - 1e-3, 60.0)
        cmd["filter"] = (self.anchor, K_HOLD, 0.0, 0.05, tau, lo_spin)
        return cmd


@dataclass
class RatchetModel:
    """The ratchet's pawl this trial: the cap's hinge (cap relative to handle, rad) may not rise above `pawl`."""

    episode: int
    pawl: float | None = None
    clicks: int = 0

    def step(self, q: float, qd: float):
        """-> (target, stiffness, effort, damping, (lo, hi))."""
        tooth = CLICK * math.ceil(q / CLICK - 1e-6)
        if self.pawl is None:
            self.pawl = tooth
        elif tooth < self.pawl:  # freewheeled back: the pawl drops into the next tooth (or several)
            self.clicks += int(round((self.pawl - tooth) / CLICK))
            self.pawl = tooth
        drag = -PAWL_DRAG if qd > 0.02 else (PAWL_DRAG if qd < -0.02 else 0.0)
        return q, 0.0, drag, 0.02, (-60.0, self.pawl + 1e-4)


# -- USD: a floating base named as the articulation's root ----------------------------------------------------------


def float_root(text: str) -> str:
    """A floating articulation's root is otherwise its most central body; name the base."""
    root_api = ('def Xform "Root" (\n    prepend apiSchemas = ["PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n'
                ')\n{\n')
    settings = ("    bool physxArticulation:enabledSelfCollisions = 0\n"
                "    int physxArticulation:solverPositionIterationCount = 32\n"
                "    int physxArticulation:solverVelocityIterationCount = 4\n")
    if root_api + settings not in text:
        raise RuntimeError("filterkit: unexpected yamkit USD layout for the root")
    text = text.replace(root_api + settings, 'def Xform "Root"\n{\n', 1)
    old = ('    def Xform "base" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", '
           '"PhysxRigidBodyAPI"]\n    )\n    {\n')
    if old not in text:
        raise RuntimeError("filterkit: unexpected yamkit USD layout for the base")
    new = ('    def Xform "base" (\n        prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI", '
           '"PhysxRigidBodyAPI", "PhysicsArticulationRootAPI", "PhysxArticulationAPI"]\n    )\n    {\n'
           + settings.replace("    ", "        "))
    return text.replace(old, new, 1)


_FLOATING: set = set()
_orig_mech_usda = kit._mech_usda


def _mech_usda(mech):
    text = _orig_mech_usda(mech)
    return float_root(text) if mech.name in _FLOATING else text


kit._mech_usda = _mech_usda


def floating(name: str) -> None:
    """Mark the mechanism `name` (one with a mass) as a free body: its base is the articulation's root."""
    _FLOATING.add(name)


# -- per step: set the drives ---------------------------------------------------------------------------------------

_MODELS: dict = {}  # name -> ("filter", Thread) or ("ratchet", link)
_STATE: dict = {}


def filter_model(name: str) -> FilterModel:
    m = _STATE.get(name)
    if m is None or m.episode != kit.episode():
        m = _STATE[name] = FilterModel(kit.episode(), _MODELS[name][1])
    return m


def ratchet_model(name: str) -> RatchetModel:
    m = _STATE.get(name)
    if m is None or m.episode != kit.episode():
        m = _STATE[name] = RatchetModel(kit.episode())
    return m


def register_filter(spec: LooseFilter) -> None:
    _MODELS[spec.name] = ("filter", spec.thread)


def register_ratchet(name: str, link: str) -> None:
    _MODELS[name] = ("ratchet", link)


def _apply(art, j, target, stiff, effort, damp, effort_limit, lim, mem):
    import torch

    kit._set_drive(art, j, target=target, stiffness=stiff, effort=effort, damping=damp)
    dev = getattr(art.data.joint_pos, "torch", art.data.joint_pos).device
    if mem.get(("eff", j)) != effort_limit:
        art.write_joint_effort_limit_to_sim_index(limits=torch.tensor([[effort_limit]], dtype=torch.float32,
                                                                      device=dev), joint_ids=[j])
        mem[("eff", j)] = effort_limit
    if lim is not None or mem.get(("lim", j)) is not None:
        want = lim if lim is not None else mem[("deflim", j)]
        if mem.get(("lim", j)) != want:
            kit._set_limits(art, j, want[0], want[1])
            mem[("lim", j)] = want if lim is not None else None


_MEM: dict = {}


def _drive(base, name: str) -> None:
    kind, arg = _MODELS[name]
    art = base.scene[name]
    np_ = lambda t: getattr(t, "torch", t).detach().cpu().numpy().astype(np.float64)  # noqa: E731
    q, qd = np_(art.data.joint_pos)[0], np_(art.data.joint_vel)[0]
    jn = list(art.joint_names)
    spec = kit._MECHS[name]
    ep, mem = _MEM.get(name, (None, None))
    if ep != kit.episode():
        mem = {}
        for lk in spec.links:
            mem[("deflim", jn.index(f"{lk.name}_joint"))] = (lk.lower - 1e-4, lk.upper + 1e-4)
        _MEM[name] = (kit.episode(), mem)
    if kind == "filter":
        J = {n: float(q[jn.index(f"{n}_joint")]) for n in CHAIN}
        cmd = filter_model(name).step(J)
        for n, (t, k, e, d, el, lim) in cmd.items():
            _apply(art, jn.index(f"{n}_joint"), t, k, e, d, el, lim, mem)
    else:
        j = jn.index(f"{arg}_joint")
        t, k, e, d, lim = ratchet_model(name).step(float(q[j]), float(qd[j]))
        _apply(art, j, t, k, e, d, 50.0, lim, mem)


_orig_mech_states = kit._mech_states


def _mech_states(base, name, origin):
    out = _orig_mech_states(base, name, origin)
    if name in _MODELS:
        _drive(base, name)
    return out


kit._mech_states = _mech_states
