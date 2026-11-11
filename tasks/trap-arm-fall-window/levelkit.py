"""levelkit: bodies that stand on adjustable supports, their resting tilt, rocking, bubble levels and pipe fall
(built on yamkit and twistkit).

A body (an appliance, a lab balance, a tribrach, a cabinet, a rack, a pipe run) stands on 3 or more supports: screw
feet turned by a knob (twistkit threads), sliding legs (a yamkit prismatic link), fixed feet, and loose shims or wedges
pushed under plain feet. Every control step this module computes, from the support heights alone and deterministically,
the plane the body rests on (which three supports carry it, which are off the ground), and drives the body to that
pose. Bubble levels drawn on the body show the tilt. Collinear supports (pipe stands or hangers along a run) give the
slope mode: a rigid pipe resting on its two highest-in-line supports, its fall read in mm per m.

    import yamkit as kit
    import twistkit as tk
    import levelkit as lv

    FEET = (lv.Foot("fl", (0.06, 0.07), kind="screw", ext=0.004, pitch=0.00175, start_turns=1.0, travel_turns=2.5,
                    lock=True),
            lv.Foot("fr", (0.06, -0.07), kind="screw", ext=0.004, pitch=0.00175, start_turns=2.2, travel_turns=2.5,
                    lock=True),
            lv.Foot("rl", (-0.06, 0.07), ext=0.0075), lv.Foot("rr", (-0.06, -0.07), ext=0.0075))
    # (a front foot could also stand in a floor dip: Foot(..., floor=-0.003))
    washer = lv.Leveled("washer", pos=(0.30, 0.0, TOP), size=(0.15, 0.17, 0.22), meshes=(...), colliders=(...),
                        feet=FEET, com=(0.0, 0.0), load_kg=4.0, top=0.21,
                        vials=(lv.Vial(at=(0.0, 0.0, 0.212), radius=0.025, circle_deg=0.5, full_deg=2.0),))
    OBJECTS = [washer, *lv.screw_feet(washer, knob_z=0.03), *lv.lock_nuts(washer, nut_z=0.017)]

The model (all lengths in m above the table top, angles in degrees; body frame = the body's own frame, origin on its
bottom plane, x forward, y left; feet `xy` are the contact points in that frame):
- Support height of foot j: s_j = base_j + ext_j. base_j = max(floor_j, top of the flat shims or wedges lying at rest
  under the contact point). ext_j = `ext` + `pitch` x turns out (screw: twistkit turns backed off from fully in, i.e.
  positive = extended), `ext` + `ratio` x joint position (slide), or `ext` (fixed). A screw foot's travel is 0 to
  `travel_turns`; past it the foot is out of its thread (`foot_out`, latched).
- The load point is the centre of mass `com` (body frame), pulled toward any gripper pressing down on the body's top
  (pad tips over the footprint and within 3 mm of the top): load = (W com + F press) / (W + F), with W = `load_kg`
  x g and F = `press_n` (15 N, a firm press with one hand). This is how a technician checks for rock: press each
  corner in turn.
- Resting plane: the face of the upper convex hull of the support points (x, y, s) whose triangle contains the load
  point. Every other support is at or below that plane; its gap is plane - s. With 4+ supports, a body with any gap
  over `rock_tol` (0.5 mm default) rocks: pressing over the off foot flips it onto the other triangle (counted in
  `rocks`). On a tie (the load right over the rocking diagonal) the body stays on the triangle it was on. If the load
  point is outside every face, the body has tipped over (`tipped`, latched; the pose then stays at the last face).
- Collinear supports (all within 1 mm of one line): the upper hull in 1D along that line; no roll about it (a pipe is
  held in its saddles); hangers below the line are slack (`slack`).
- Pose: pitch theta = -atan(gx) about y, then roll phi = atan(gy cos theta) about x (yamkit chain z -> pitch -> roll),
  where z = h + gx x + gy y is the plane; h is fitted on the contact supports. Tilt = acos(cos theta cos phi).
- Lock nuts: a jam nut on a screw foot's stem (its own twistkit thread). Locked when tightened `lock_deg` (15) past its
  seat. While locked, the foot is jammed: its extension stays where it was when the nut locked (turning it is booked in
  `forced_deg`, and the knob is held stiffly in the sim). A loose nut leaves the foot free.

Simplifications (say them in fidelity): the body is a rigid plane on point supports (no frame flex, no soil or rubber
compliance); small-angle geometry (foot points do not shift sideways as the body tilts; < 0.2 mm at 3 degrees over
15 cm); the body is posed kinematically, so it does not slide, bounce or fall over in the sim (a tipped body is flagged,
not toppled); shims count only under plain (fixed) feet and only at rest; a screw foot's knob, stem and pad stay put
while the body's bracket rides up and down the stem (as on a top-adjust levelling mount); the bubble's travel is
exaggerated so that the circle tolerance reads at 320x176 (the circle is drawn at the task's tolerance, not at a real
vial's 8 arcmin per 2 mm).

Scorer API (`objects` is what score() gets; `name` is the Leveled body):
- `state(objects, name)` -> `LevelState` (all of the below for this step).
- `tilt_deg`, `pitch_deg` (rise toward the body's +x, degrees: front up is positive), `roll_deg` (rise toward +y),
  `rise_deg(objects, name, toward)` (rise toward a body-frame direction).
- `slope_mm_per_m(objects, name, toward=(1, 0))`: fall going toward `toward` in mm per m (positive = downhill that way).
- `bubble(objects, name, i=0)`: the vial's bubble offset as a fraction of its travel (2-vector, body frame);
  `bubble_centered(objects, name, i=0)`: tilt (less the vial's own zero) within its circle.
- `rocking(objects, name)`, `gaps_mm(objects, name)` {foot: mm}, `unloaded(objects, name)` (feet off the ground),
  `slack(objects, name)` (same, for hangers), `rocks(objects, name)` (times it flipped this trial), `tipped`.
- `ext_mm(objects, name, foot)`, `foot_out(objects, name)`, `locked(objects, name, foot)`,
  `feet_locked(objects, name)` (every foot with a lock nut is locked), `forced_deg(objects, name, foot)`.
- `max_tilt_deg(objects, name)` (this trial), `touched_feet(objects, name)` (feet whose screw was turned this trial).
- `held(objects, name)`: a gripper's fingers closed on any of the body's knobs or nuts (twistkit's grip test).
All of them follow the trial: a new kit.episode() starts a fresh record.

Demo helpers: `plan(name_or_body, target=(0, 0), objects=None)` -> {foot: degrees to turn (positive = extend)} that
brings the body to `target` (pitch_deg, roll_deg) with every fixed foot bearing; `foot_grip(body, foot)` -> (x, y, z)
of the knob's grip point; `turn_foot(body, foot, degrees, side=..., open_to=...)` -> twistkit regrasp waypoints for
that knob (optionally opening the fingers only part way); `knob_limit_deg(threads)` -> the largest knob hinge limit in
degrees (keep it well under 450: PhysX revolute limits past 1.25 turns are untested in our tasks; choose the pitch,
`travel_turns` and `screw_feet(spare_turns=...)` so they stay small). `Foot(stop=True)` ends a foot's travel at a hard
stop instead of letting it come out of its thread.

Offline tests: `register(*bodies)` and `tk.register(*threads)`, then `update(objects)` once per step with hand-made
ObjectStates (`objects[name]` = the base: pos and quat give position and yaw; grippers optional); turn feet with
`tk.set_angle(key, past_seat_deg)`. Plain Python and NumPy at import time; Isaac code runs only inside the hooks.
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field, replace
from itertools import combinations

import numpy as np
import twistkit as tk
import yamkit as kit

G = 9.81
PRESS_DZ = 0.003  # m: pad tips within 3 mm of the body's top (or into it) over its footprint are pressing on it
COLLINEAR = 0.001  # m: supports all within this of one line are a run (slope mode)
SHIM_MARGIN = 0.002  # m: a contact point must be this far inside a shim's footprint to stand on it
SHIM_FLAT = 5.0  # degrees: a shim tilted more than this (on edge, propped) does not count
SHIM_STILL = 0.02  # m/s: a shim still moving faster than this (being pushed or carried) does not count
GRIPPERS = ("left_gripper", "right_gripper")


# -- specs -------------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Foot:
    """A support. `xy` is its contact point in the body frame (bottom plane, z = 0). `kind`: "fixed" (constant `ext`),
    "screw" (a twistkit knob: ext + pitch x turns out; `key` defaults to "<body>_<name>/knob"), or "slide" (a yamkit
    prismatic link `key`: ext + ratio x its joint position). `floor` is the floor's height under the foot (a dip is
    negative, a raised plate positive). `lock=True` gives a screw foot a jam nut ("<body>_<name>_nut/nut"), locked
    `lock_deg` past its seat. `start_turns` / `travel_turns`: a screw's turns out at the start and its full travel.
    `stop=True`: past `travel_turns` the foot sits at a hard stop (its extension clamped there) instead of coming out of
    its thread (`foot_out`, which latches `tipped`); use it where running a foot to its end is no failure (a stop
    foot is also clamped at fully in). `seat_turns`: where the knob hinge's zero sits, in turns out from fully in;
    putting it mid-travel centers the hinge's range on zero, so a foot with 2 turns of travel needs hinge limits of only
    about +-1 turn (see `screw_feet` and `knob_limit_deg`)."""

    name: str
    xy: tuple
    kind: str = "fixed"
    ext: float = 0.0
    floor: float = 0.0
    pitch: float = 0.00175  # m per turn: an M12 coarse thread, a common levelling-foot size
    start_turns: float = 0.0
    travel_turns: float = 3.0
    key: str | None = None
    ratio: float = 1.0
    lock: bool = False
    lock_deg: float = 15.0
    lock_start_deg: float = -180.0  # the nut's start, degrees past its seat (negative = backed off)
    shims: bool = True
    stop: bool = False  # True: the screw's travel ends at a hard stop (clamped there), never out of its thread
    seat_turns: float = 0.0  # turns out at which the knob's twistkit seat (hinge zero) sits; see screw_feet


@dataclass(frozen=True)
class Shim:
    """A loose shim or wedge (a separate object in OBJECTS named `name`). Its footprint is `length` (local x) by
    `width` (local y); its thickness runs from `t0` at local x = -length/2 to `t1` at +length/2 (equal for a flat
    shim). `bottom` is the offset from the object's reported `pos` down to its underside along its local z (default:
    minus half the mean thickness, a centred Box)."""

    name: str
    length: float
    width: float
    t0: float
    t1: float | None = None
    bottom: float | None = None

    def thick(self, u: float) -> float:
        t1 = self.t0 if self.t1 is None else self.t1
        f = min(max(u / self.length + 0.5, 0.0), 1.0)
        return self.t0 + (t1 - self.t0) * f

    def drop(self) -> float:
        t1 = self.t0 if self.t1 is None else self.t1
        return -(self.t0 + t1) / 4 if self.bottom is None else self.bottom


@dataclass(frozen=True)
class Vial:
    """A level vial on the body: `at` (body frame, its face centre), `kind` "bullseye" (2-axis) or "tube" (reads the
    tilt along `axis`, a body-frame unit 2-vector). The bubble reaches the rim at `full_deg` of tilt; the circle (or the
    tube's two lines) marks `circle_deg`. `zero` is the (pitch_deg, roll_deg) at which the bubble centres (a fall vial
    reads 0 at the design slope). Drawn with radius `radius`, bubble radius `bubble_r`."""

    at: tuple
    radius: float = 0.030
    circle_deg: float = 0.5
    full_deg: float = 2.0
    kind: str = "bullseye"
    axis: tuple = (1.0, 0.0)
    zero: tuple = (0.0, 0.0)
    bubble_r: float = 0.009
    line: float = 0.003  # m: width of the drawn circle or lines (about 1 px at 320x176)
    bubble_rgb: tuple = (0.97, 0.97, 0.92)

    def travel(self) -> float:
        return self.radius - self.bubble_r

    def ring_r(self) -> float:
        """Radius of the drawn circle: the bubble's edge touches it at `circle_deg`."""
        return self.bubble_r + self.travel() * min(1.0, self.circle_deg / self.full_deg)


class _PoseLaw:
    """yamkit law for one of the body's three pose joints: set kinematically to the model's pose every step (rate
    limited), held softly in between (the links float, so gravity does not sag them)."""

    kinematic = True

    def __init__(self, body: str, dof: int, rate: float, hold: float):
        self.body, self.dof, self.rate, self.hold = body, dof, rate, hold

    def drive(self, key, q, qd, J, mem):
        m = _MODEL.get(self.body)
        t = m.pose[self.dof] if m is not None and m.episode == kit.episode() else q
        mem["target"] = t
        return t, self.hold, 0.0, None


@dataclass(frozen=True)
class Leveled(kit.Mechanism):
    """A body on supports. Give it `pos` (its frame's origin, on the table top under the body's bottom-plane centre,
    z = TOP), `size`, the body's `meshes` and `colliders` (in its own frame, bottom plane at z = 0; they ride on the
    tilting link), `feet`, `com` (body-frame xy of the centre of mass), `load_kg`, `top` (height of its top face above
    its bottom plane, for presses), `shims` (Shim specs of loose objects), `vials`, and `extra` links (doors, lids)
    that ride on the body (their `parent` defaults to the body link). Fixed base: `mass` stays None."""

    feet: tuple = ()
    com: tuple = (0.0, 0.0)
    load_kg: float = 2.0
    top: float = 0.05
    shims: tuple = ()
    vials: tuple = ()
    extra: tuple = ()
    rock_tol: float = 0.0005
    press_n: float = 15.0
    rate_z: float = 0.002  # m per control step the body's pose may move (a 2 mm/step settle, sub-mm while turning)
    rate_ang: float = math.radians(0.5)

    def __post_init__(self):
        if self.mass is not None:
            raise ValueError(f"{self.name}: a Leveled body has a fixed (hidden) base; leave mass None")
        if len(self.feet) < 2:
            raise ValueError(f"{self.name}: needs at least 2 supports")
        start = _solve_spec(self)
        h, th, ph = start.pose
        b = self.name
        lz = kit.Link(f"{b}_z", "prismatic", "Z", (0.0, 0.0, 0.0), lower=-0.05, upper=0.15, start=h, mass=0.01,
                      floating=True, law=_PoseLaw(b, 0, self.rate_z, 2000.0))
        lp = kit.Link(f"{b}_pitch", "revolute", "Y", (0.0, 0.0, 0.0), lower=-0.5, upper=0.5, start=th, mass=0.01,
                      floating=True, parent=f"{b}_z", law=_PoseLaw(b, 1, self.rate_ang, 50.0))
        lr = kit.Link(f"{b}_body", "revolute", "X", (0.0, 0.0, 0.0), lower=-0.5, upper=0.5, start=ph,
                      mass=max(0.05, self.load_kg), floating=True, parent=f"{b}_pitch", meshes=self.meshes,
                      colliders=self.colliders, law=_PoseLaw(b, 2, self.rate_ang, 50.0))
        extra = tuple(lk if lk.parent is not None else replace(lk, parent=f"{b}_body") for lk in self.extra)
        object.__setattr__(self, "links", (lz, lp, lr) + extra)
        object.__setattr__(self, "meshes", ())
        object.__setattr__(self, "colliders", ())


# -- the pure model --------------------------------------------------------------------------------------


@dataclass(frozen=True)
class Rest:
    """Where a body rests: plane z = h + gx x + gy y (body frame), the supports carrying it, each support's gap below
    the plane (m), whether the load point is outside the supports (tipped), and whether the supports are a run."""

    h: float
    g: tuple
    contacts: tuple
    gaps: tuple
    tipped: bool
    run: bool

    @property
    def pose(self) -> tuple:
        th = -math.atan(self.g[0])
        ph = math.atan(self.g[1] * math.cos(th))
        return (self.h, th, ph)


def _line(P):
    """(origin, unit direction) if all points lie within COLLINEAR of one line, else None."""
    c = P.mean(axis=0)
    d = P - c
    if len(P) == 2:
        u = d[1] - d[0]
        return c, u / np.linalg.norm(u)
    _, s, vt = np.linalg.svd(d)
    u = vt[0]
    off = np.abs(d @ np.array([-u[1], u[0]]))
    return (c, u) if off.max() <= COLLINEAR else None


def rest(P, Z, load, prev: tuple | None = None) -> Rest:
    """The resting plane of a rigid body on point supports at body-frame xy `P` (n x 2) with heights `Z` (n), loaded at
    `load` (xy). Pure function. `prev`: the contacts it rested on last step (kept on a tie)."""
    P, Z, load = np.asarray(P, float), np.asarray(Z, float), np.asarray(load, float)
    n = len(P)
    ln = _line(P)
    if ln is not None:
        c, u = ln
        s = (P - c) @ u
        L = float((load - c) @ u)
        best = None
        for i, j in combinations(range(n), 2):
            if abs(s[j] - s[i]) < 1e-9:
                continue
            k = (Z[j] - Z[i]) / (s[j] - s[i])
            line = Z[i] + k * (s - s[i])
            if np.any(line - Z < -1e-9):
                continue
            lo, hi = sorted((s[i], s[j]))
            inside = min(L - lo, hi - L)
            pair = tuple(sorted((i, j)))
            cand = (inside >= -1e-12, pair == prev, inside, pair, k, line)
            if best is None or cand[:3] > best[:3]:
                best = cand
        ok, _, _, pair, k, line = best
        g = (k * u[0], k * u[1])
        h = float(np.mean([Z[i] - (g[0] * P[i, 0] + g[1] * P[i, 1]) for i in pair]))
        gaps = tuple(float(v) for v in np.maximum(line - Z, 0.0))
        return Rest(h, g, pair, gaps, not ok, True)
    best = None
    for tri in combinations(range(n), 3):
        A = np.column_stack([np.ones(3), P[list(tri)]])
        if abs(np.linalg.det(A)) < 1e-12:
            continue
        coef = np.linalg.solve(A, Z[list(tri)])
        plane = coef[0] + P @ coef[1:]
        if np.any(plane - Z < -1e-9):
            continue  # another support pokes up through this plane: not a resting face
        bary = np.linalg.solve(A.T, np.array([1.0, load[0], load[1]]))
        inside = float(bary.min())
        cand = (inside >= -1e-12, tuple(tri) == prev, inside, tuple(tri), coef, plane)
        if best is None or cand[:3] > best[:3]:
            best = cand
    if best is None:
        raise ValueError("levelkit: supports are degenerate")
    ok, _, _, tri, coef, plane = best
    gaps = tuple(float(v) for v in np.maximum(plane - Z, 0.0))
    return Rest(float(coef[0]), (float(coef[1]), float(coef[2])), tri, gaps, not ok, False)


def tilt_of(g) -> float:
    """Tilt (degrees) of a plane with gradient g from horizontal."""
    return math.degrees(math.atan(math.hypot(g[0], g[1])))


def _solve_spec(body: Leveled) -> Rest:
    P = [f.xy for f in body.feet]
    Z = [f.floor + f.ext + (f.pitch * f.start_turns if f.kind == "screw" else 0.0) for f in body.feet]
    return rest(P, Z, body.com)


# -- per-trial state ---------------------------------------------------------------------------------------


@dataclass(frozen=True)
class LevelState:
    pitch_deg: float
    roll_deg: float
    tilt_deg: float
    g: tuple
    contacts: tuple  # names of the supports carrying the body
    gaps_mm: dict
    rocking: bool
    tipped: bool
    ext_mm: dict
    locked: dict
    pressed: bool


class _Model:
    def __init__(self, body: Leveled):
        self.body = body
        self.episode = kit.episode()
        self.rest = _solve_spec(body)
        self.pose = self.rest.pose
        self.prev = self.rest.contacts
        self.tipped = False
        self.foot_out = False
        self.rocks = 0
        self.max_tilt = tilt_of(self.rest.g)
        self.lock_ext: dict = {}  # foot -> extension when its nut locked
        self.lock_turns: dict = {}
        self.forced: dict = {f.name: 0.0 for f in body.feet}
        self.touched: set = set()
        self.ext = {f.name: f.ext + (f.pitch * f.start_turns if f.kind == "screw" else 0.0) for f in body.feet}
        self.locked = {f.name: f.lock and f.lock_start_deg >= f.lock_deg for f in body.feet}
        self.pressed = False
        self.state = None


_BODIES: dict[str, Leveled] = {}
_MODEL: dict[str, _Model] = {}


def register(*bodies: Leveled) -> None:
    """Make bodies known to the model without a simulator (offline tests); the sim registers them when it builds."""
    for b in bodies:
        _BODIES[b.name] = b


def screw_key(body: Leveled | str, foot: Foot) -> str:
    b = body if isinstance(body, str) else body.name
    return foot.key or f"{b}_{foot.name}/knob"


def nut_key(body: Leveled | str, foot: Foot) -> str:
    b = body if isinstance(body, str) else body.name
    return f"{b}_{foot.name}_nut/nut"


def _model(name: str) -> _Model:
    m = _MODEL.get(name)
    if m is None or m.episode != kit.episode():
        m = _MODEL[name] = _Model(_BODIES[name])
    return m


def _quat_rot(q) -> np.ndarray:
    return tk._quat_rot(q)


def _yaw_of(state) -> float:
    R = _quat_rot(state.quat_xyzw)
    return math.atan2(R[1, 0], R[0, 0])


def _turns(key: str) -> float | None:
    if key not in tk._SPECS:
        return None
    return tk.turns(None, key)


def _foot_ext(m: _Model, body: Leveled, f: Foot, objects) -> float:
    """The foot's extension now, with its lock nut's jam applied."""
    if f.kind == "fixed":
        return f.ext
    if f.kind == "slide":
        try:
            q = kit.joint(objects, f.key).pos
        except KeyError:
            return m.ext[f.name]
        lk = next((lk for b in kit._MECHS.values() for lk in b.links if f"{b.name}/{lk.name}" == f.key), None)
        q0 = lk.start if lk is not None else 0.0
        return f.ext + f.ratio * (q - q0)
    k = screw_key(body, f)
    t = _turns(k)
    if t is None:
        return m.ext[f.name]
    t += f.seat_turns  # turns out from fully in
    if abs(t - f.start_turns) > 1e-6:
        m.touched.add(f.name)
    if not f.stop and (t > f.travel_turns or tk.came_off(None, k)):
        m.foot_out = True
    locked = False
    if f.lock and nut_key(body, f) in tk._SPECS:
        locked = tk.past_seat_deg(None, nut_key(body, f)) >= f.lock_deg
    elif f.lock:
        locked = m.locked.get(f.name, False)
    if locked:
        if f.name not in m.lock_ext:
            m.lock_ext[f.name], m.lock_turns[f.name] = m.ext[f.name], t
        m.forced[f.name] = max(m.forced[f.name], abs(t - m.lock_turns[f.name]) * 360.0)
        m.locked[f.name] = True
        return m.lock_ext[f.name]
    m.lock_ext.pop(f.name, None)
    m.lock_turns.pop(f.name, None)
    m.locked[f.name] = False
    tt = min(max(t, 0.0 if f.stop else -0.1), f.travel_turns)
    return f.ext + f.pitch * tt


def _shim_top(body: Leveled, objects, w_xy) -> float | None:
    """Highest top of the resting, flat shims under world point `w_xy` (m above the table), or None."""
    best = None
    for sh in body.shims:
        s = objects.get(sh.name)
        if s is None:
            continue
        R = _quat_rot(s.quat_xyzw)
        if R[2, 2] < math.cos(math.radians(SHIM_FLAT)):
            continue
        if float(np.linalg.norm(np.asarray(s.lin_vel, float))) > SHIM_STILL:
            continue
        p = np.asarray(s.pos, float)
        d = R.T @ (np.array([w_xy[0], w_xy[1], p[2]]) - p)
        if abs(d[0]) > sh.length / 2 - SHIM_MARGIN or abs(d[1]) > sh.width / 2 - SHIM_MARGIN:
            continue
        top = p[2] + R[2, 2] * (sh.drop() + sh.thick(float(d[0])))
        best = top if best is None else max(best, top)
    return best


def update(objects) -> None:
    """Compute every registered body's resting pose for this step (called by the hook once per control step, after
    yamkit's and twistkit's own updates; tests call it directly)."""
    for name, body in _BODIES.items():
        if name not in objects:
            continue
        m = _model(name)
        base = objects[name]
        o = np.asarray(base.pos, float)
        yaw = _yaw_of(base)
        c, s = math.cos(yaw), math.sin(yaw)
        P, Z = [], []
        for f in body.feet:
            ext = _foot_ext(m, body, f, objects)
            m.ext[f.name] = ext
            wx, wy = o[0] + c * f.xy[0] - s * f.xy[1], o[1] + s * f.xy[0] + c * f.xy[1]
            floor = f.floor
            if f.shims and f.kind == "fixed" and body.shims:
                top = _shim_top(body, objects, (wx, wy))
                if top is not None:
                    floor = max(floor, top - o[2])
            P.append(f.xy)
            Z.append(floor + ext)
        # the load point: the centre of mass, pulled toward a hand pressing down on the top
        load = np.asarray(body.com, float) * body.load_kg * G
        wsum = body.load_kg * G
        m.pressed = False
        top_z = o[2] + m.pose[0] + body.top
        for g in GRIPPERS:
            gs = objects.get(g)
            if gs is None:
                continue
            d = np.asarray(gs.pos, float) - o
            bx, by = c * d[0] + s * d[1], -s * d[0] + c * d[1]
            hx, hy = body.size[0] / 2, body.size[1] / 2
            tips = float(gs.low[2]) if gs.low is not None else float(gs.pos[2])  # the pad tips
            if abs(bx) <= hx and abs(by) <= hy and top_z - 0.5 * body.top <= tips <= top_z + PRESS_DZ:
                load = load + body.press_n * np.array([bx, by])
                wsum += body.press_n
                m.pressed = True
        load = load / wsum
        r = rest(P, Z, load, m.prev)
        rocking = (not r.run) and len(P) >= 4 and max(r.gaps) > body.rock_tol
        if r.contacts != m.prev and max(r.gaps) > body.rock_tol and not r.run:
            m.rocks += 1
        m.prev = r.contacts
        if r.tipped or m.foot_out:
            m.tipped = True
        if not m.tipped:
            m.rest = r
            m.pose = r.pose
        rr = m.rest
        m.max_tilt = max(m.max_tilt, tilt_of(rr.g))
        names = [f.name for f in body.feet]
        m.state = LevelState(
            pitch_deg=math.degrees(math.atan(rr.g[0])), roll_deg=math.degrees(math.atan(rr.g[1])),
            tilt_deg=tilt_of(rr.g), g=rr.g, contacts=tuple(names[i] for i in rr.contacts),
            gaps_mm={n: 1000.0 * gp for n, gp in zip(names, rr.gaps)}, rocking=bool(rocking and not m.tipped),
            tipped=m.tipped, ext_mm={n: 1000.0 * m.ext[n] for n in names}, locked=dict(m.locked), pressed=m.pressed)


# -- scorer API ---------------------------------------------------------------------------------------------


def state(objects, name: str) -> LevelState:
    """This step's LevelState (after update)."""
    m = _model(name)
    if m.state is None:
        update(objects)
    return m.state


def tilt_deg(objects, name: str) -> float:
    return state(objects, name).tilt_deg


def pitch_deg(objects, name: str) -> float:
    """Rise toward the body's +x (front up positive), degrees."""
    return state(objects, name).pitch_deg


def roll_deg(objects, name: str) -> float:
    """Rise toward the body's +y (left side up positive), degrees."""
    return state(objects, name).roll_deg


def rise_deg(objects, name: str, toward=(1.0, 0.0)) -> float:
    g = state(objects, name).g
    d = np.asarray(toward, float) / np.linalg.norm(toward)
    return math.degrees(math.atan(g[0] * d[0] + g[1] * d[1]))


def slope_mm_per_m(objects, name: str, toward=(1.0, 0.0)) -> float:
    """Fall going toward `toward` (body frame), mm per m: positive = runs downhill that way."""
    g = state(objects, name).g
    d = np.asarray(toward, float) / np.linalg.norm(toward)
    return -1000.0 * (g[0] * d[0] + g[1] * d[1])


def bubble(objects, name: str, i: int = 0) -> np.ndarray:
    """Vial i's bubble offset from its centre as a fraction of its travel (body frame; the bubble runs uphill)."""
    v = _BODIES[name].vials[i]
    st = state(objects, name)
    t = np.array([st.pitch_deg - v.zero[0], st.roll_deg - v.zero[1]])
    if v.kind == "tube":
        a = np.asarray(v.axis, float) / np.linalg.norm(v.axis)
        t = a * float(t @ a)
    mag = float(np.linalg.norm(t))
    if mag < 1e-12:
        return np.zeros(2)
    return t / mag * min(1.0, mag / v.full_deg)


def bubble_centered(objects, name: str, i: int = 0) -> bool:
    """The bubble is inside vial i's circle (tilt from the vial's zero within `circle_deg`)."""
    v = _BODIES[name].vials[i]
    return float(np.linalg.norm(bubble(objects, name, i))) * v.full_deg <= v.circle_deg + 1e-9


def rocking(objects, name: str) -> bool:
    """4+ supports, not a run, and some support more than `rock_tol` below the resting plane."""
    return state(objects, name).rocking


def gaps_mm(objects, name: str) -> dict:
    return dict(state(objects, name).gaps_mm)


def unloaded(objects, name: str) -> tuple:
    """Supports more than `rock_tol` below the plane: off the ground (a foot) or slack (a hanger)."""
    tol = 1000.0 * _BODIES[name].rock_tol
    return tuple(n for n, gp in state(objects, name).gaps_mm.items() if gp > tol)


slack = unloaded


def rocks(objects, name: str) -> int:
    """Times the body flipped from one resting triangle to another this trial (a press over the off foot)."""
    return _model(name).rocks


def tipped(objects, name: str) -> bool:
    """The load point left the supports (the body would have fallen over) or a foot came out of its thread: latched."""
    return _model(name).tipped


def foot_out(objects, name: str) -> bool:
    return _model(name).foot_out


def ext_mm(objects, name: str, foot: str) -> float:
    return state(objects, name).ext_mm[foot]


def locked(objects, name: str, foot: str) -> bool:
    return bool(state(objects, name).locked.get(foot, False))


def feet_locked(objects, name: str) -> bool:
    """Every foot that has a lock nut has it tightened against the body."""
    st = state(objects, name)
    return all(st.locked.get(f.name, False) for f in _BODIES[name].feet if f.lock)


def forced_deg(objects, name: str, foot: str) -> float:
    """Most a foot was turned against its locked nut this trial (degrees)."""
    return _model(name).forced.get(foot, 0.0)


def max_tilt_deg(objects, name: str) -> float:
    return _model(name).max_tilt


def touched_feet(objects, name: str) -> tuple:
    return tuple(sorted(_model(name).touched))


def held(objects, name: str) -> bool:
    """A gripper's fingers closed on any of the body's knobs or lock nuts (twistkit's grip test), as of the last step."""
    body = _BODIES[name]
    keys = [screw_key(body, f) for f in body.feet if f.kind == "screw"]
    keys += [nut_key(body, f) for f in body.feet if f.lock]
    return any(k in tk._SPECS and tk.held(objects, k) for k in keys)


# -- builders: screw feet and lock nuts as twistkit threads --------------------------------------------------

STEEL = kit.Look((0.78, 0.79, 0.80), 0.3, metallic=0.8)
KNOB = kit.Look((0.10, 0.10, 0.11), 0.6)
PAD = kit.Look((0.12, 0.12, 0.12), 0.8)
NUT = kit.Look((0.85, 0.70, 0.20), 0.35, metallic=0.6)  # brass-colored jam nut: reads apart from the black knob


def _world(body: Leveled, xy) -> tuple:
    c, s = math.cos(body.yaw), math.sin(body.yaw)
    return (body.pos[0] + c * xy[0] - s * xy[1], body.pos[1] + s * xy[0] + c * xy[1])


def screw_feet(body: Leveled, knob_z: float, knob_r: float = 0.014, knob_h: float = 0.014, pad_r: float = 0.016,
               stem_r: float = 0.004, rides: bool = False, resist: float = 0.08, sides: int = 0,
               spare_turns: float = 0.25) -> list:
    """One fixed twistkit `Threaded` per screw foot of `body`: a swivel pad on the floor (base, no collider, so it never
    fights the body), and the turning part: a threaded stem up from the pad and a knurled knob centred `knob_z` above
    the table top (body.pos[2]; world z = body.pos[2] + knob_z), `knob_r` x `knob_h` (grip it straight down across its diameter). With `rides=False` (a top-adjust
    levelling mount) the knob stays at its height and the body's bracket rides the stem; `rides=True` uses the foot's
    real pitch so the knob climbs as it turns (a nut on a hanger rod carrying a pipe saddle). `sides=6` draws the knob
    as a hex head (`knob_r` to its corners) for a mount turned with a wrench. `spare_turns`: how far past
    `travel_turns` the knob's hinge may turn before its joint limit (the hinge's upper limit is (travel_turns +
    spare_turns) x 360 + 30 degrees; keep it small, see `knob_limit_deg`: PhysX revolute limits past about 480 degrees
    are untested in our tasks). A foot with `seat_turns` > 0 has its hinge zero that far out: the hinge then runs from
    -(seat_turns x 360 + 5) to (travel_turns - seat_turns + spare_turns) x 360 + 30 degrees, with the same resistance
    both ways."""
    out = []
    for f in body.feet:
        if f.kind != "screw":
            continue
        x, y = _world(body, f.xy)
        z0 = body.pos[2] + f.floor
        kz = knob_z - f.floor  # the knob's height above its pad (knob_z is above the table top, body.pos[2])
        head = (kit.lathe([(knob_r, -knob_h / 2), (knob_r, knob_h / 2)], KNOB, (0, 0, kz), n=sides) if sides
                else kit.cylinder((0, 0, kz), knob_r, knob_h, KNOB))
        cap_m = (head,
                 kit.cylinder((0, 0, (kz) / 2), stem_r, kz, STEEL))
        cap_c = (kit.Collider((0, 0, kz), radius=knob_r, height=knob_h),)
        seat = f.seat_turns  # the hinge's zero; below it (running the foot in) is twistkit's "past the seat"
        th = tk.Thread(pitch=f.pitch if rides else 1e-5, turns=f.travel_turns - seat + spare_turns,
                       start=-360.0 * (f.start_turns - seat), max_tight=max(20.0, 360.0 * seat + 5.0), resist=resist,
                       tight_resist=resist if seat > 0 else resist * 3, free_lift=0.0)
        out.append(tk.Threaded(f"{body.name}_{f.name}", pos=(x, y, z0), size=(2 * pad_r, 2 * pad_r, kz + knob_h),
                               meshes=(kit.cylinder((0, 0, 0.002), pad_r, 0.004, PAD),),
                               colliders=(), cap=tk.Cap("knob", anchor=(0, 0, 0), mass=0.03, meshes=cap_m,
                                                        colliders=cap_c, grip=(0, 0, kz), width=2 * knob_r),
                               thread=th))
    return out


def knob_limit_deg(threads) -> float:
    """The largest hinge limit (degrees from the seat, either side) among twistkit `Threaded` parts, e.g. the list
    `screw_feet` returns: the upper limit is turns x 360 + 30 (twistkit's margin), the lower one max_tight. Check it
    stays well under 450 (1.25 turns, the most our sim tasks have run)."""
    return max(max(t.thread.turns * 360.0 + 30.0, t.thread.max_tight) for t in threads)


def lock_nuts(body: Leveled, nut_z: float, across: float = 0.030, nut_h: float = 0.010) -> list:
    """One fixed twistkit `Threaded` per locking screw foot: a hex jam nut (`across` flats, `nut_h` tall, centred
    `nut_z` above the table top, i.e. body.pos[2] + nut_z in the world) on the stem below the knob. Tightening runs it down onto the body's bracket; it starts
    `lock_start_deg` past its seat (negative = backed off) and locks at `lock_deg`. Make it wider than the knob so a
    straight-down grip can close on it past the knob."""
    out = []
    for f in body.feet:
        if f.kind != "screw" or not f.lock:
            continue
        x, y = _world(body, f.xy)
        r = across / 2 / math.cos(math.pi / 6)
        hexm = kit.lathe([(r, -nut_h / 2), (r, nut_h / 2)], NUT, (0, 0, 0), n=6)
        th = tk.Thread(pitch=1e-5, turns=2.0, start=f.lock_start_deg, max_tight=60.0, resist=0.05, tight_resist=0.4,
                       free_lift=0.0, hand=-1)  # tightening a jam nut down onto the bracket: clockwise from the top
        out.append(tk.Threaded(f"{body.name}_{f.name}_nut", pos=(x, y, body.pos[2] + nut_z), size=(2 * r, 2 * r, nut_h), meshes=(),
                               colliders=(), cap=tk.Cap("nut", anchor=(0, 0, 0), mass=0.02, meshes=(hexm,),
                                                        colliders=(kit.Collider((0, 0, 0), size=(across, across, nut_h)),),
                                                        width=across), thread=th))
    return out


def vial_meshes(v: Vial, housing=kit.Look((0.12, 0.12, 0.13), 0.5), liquid=kit.Look((0.72, 0.90, 0.18), 0.2),
                ring=kit.Look((0.05, 0.05, 0.05), 0.6)) -> tuple:
    """Body-frame meshes for vial `v`: a dark housing, a bright yellow-green liquid face and a black circle (bullseye)
    or two black lines (tube) at the tolerance. Add them to the body's meshes; the bubble is drawn by the kit."""
    x, y, z = v.at
    out = [kit.cylinder((x, y, z - 0.003), v.radius + 0.003, 0.006, housing),
           kit.cylinder((x, y, z + 0.0004), v.radius, 0.0008, liquid)]
    if v.kind == "bullseye":
        rr = v.ring_r()
        pts = [(x + rr * math.cos(2 * math.pi * k / 48), y + rr * math.sin(2 * math.pi * k / 48), z + 0.0012)
               for k in range(49)]
        out.append(kit.band(pts, 0.0008, v.line, ring, normal=(0, 0, 1)))
    else:
        a = np.asarray(v.axis, float) / np.linalg.norm(v.axis)
        d = v.ring_r()
        for sgn in (-1, 1):
            cx, cy = x + sgn * d * a[0], y + sgn * d * a[1]
            out.append(kit.box((cx, cy, z + 0.0012), (v.line if abs(a[0]) > 0.5 else 2 * v.radius,
                                                       2 * v.radius if abs(a[0]) > 0.5 else v.line, 0.0008), ring))
    return tuple(out)


# -- demo helpers ---------------------------------------------------------------------------------------------


def plan(body: Leveled | str, target=(0.0, 0.0), objects=None) -> dict:
    """Degrees to turn each screw foot (positive = extend) to bring the body to `target` = (pitch_deg, roll_deg) with
    every non-screw support on the plane (all bearing). From the start state, or from this step's if `objects`. Feet
    that a lock nut jams must be unlocked first (not handled here)."""
    b = _BODIES[body] if isinstance(body, str) else body
    if b.name not in _BODIES:
        register(b)
    ext = _model(b.name).ext if objects is not None else \
        {f.name: f.ext + (f.pitch * f.start_turns if f.kind == "screw" else 0.0) for f in b.feet}
    g = (math.tan(math.radians(target[0])), math.tan(math.radians(target[1])))
    plane = lambda f, h: h + g[0] * f.xy[0] + g[1] * f.xy[1]  # noqa: E731
    fixed = [f for f in b.feet if f.kind != "screw"]
    if fixed:
        h = max(f.floor + ext[f.name] - (g[0] * f.xy[0] + g[1] * f.xy[1]) for f in fixed)
    else:
        h = float(np.mean([f.floor + ext[f.name] - (g[0] * f.xy[0] + g[1] * f.xy[1]) for f in b.feet]))
    return {f.name: 360.0 * (plane(f, h) - f.floor - ext[f.name]) / f.pitch for f in b.feet if f.kind == "screw"}


def past_seat_for(foot: Foot, turns_out: float) -> float:
    """The twistkit angle (degrees past the seat) of a screw foot set `turns_out` turns out from fully in; for
    tk.set_angle in offline tests."""
    return -360.0 * (turns_out - foot.seat_turns)


def foot_grip(body: Leveled, foot: str) -> tuple:
    """World (x, y, z) of a screw foot's knob grip point at the start."""
    f = next(f for f in body.feet if f.name == foot)
    spec = tk._SPECS.get(screw_key(body, f))
    x, y = _world(body, f.xy)
    z = spec.pos[2] + spec.cap.grip[2] if spec is not None else body.pos[2] + 0.03
    return (x, y, z)


def turn_foot(body: Leveled, foot: str, degrees: float, side: str | None = None, z: float | None = None,
              open_to: float | None = None, **kw) -> list:
    """Twistkit regrasp waypoints that turn a screw foot's knob `degrees` (positive = extend: counterclockwise seen
    from the top for these right-hand feet). `z`: the tool point's height while gripping (default: the knob's grip
    point). `open_to`: open the fingers only this far (a finger value, e.g. 0.5 x FINGER_OPEN) instead of fully, so
    open fingers clear a wall or the body next to the knob."""
    x, y, zg = foot_grip(body, foot)
    wps = tk.twist_waypoints((x, y), zg if z is None else z, degrees, side=side, pitch=0.0, **kw)
    if open_to is None:
        return wps
    from yam_arena.constants import FINGER_OPEN
    from yam_arena.demo import Waypoint
    return [Waypoint(w.duration, w.pos, open_to if w.finger == FINGER_OPEN else w.finger, w.label, w.yaw)
            for w in wps]


# -- sim glue: model step, lock pins, bubble markers -----------------------------------------------------------

_VIS = {"markers": None, "failed": False}


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        protos, idx = {}, {}
        for b in _BODIES.values():
            for v in b.vials:
                key = "bubble_%02x%02x%02x" % tuple(int(255 * c) for c in v.bubble_rgb)
                if key not in protos:
                    idx[key] = len(protos)
                    protos[key] = sim_utils.SphereCfg(radius=1.0, visual_material=sim_utils.PreviewSurfaceCfg(
                        diffuse_color=tuple(kit._linear(c) for c in v.bubble_rgb), roughness=0.2))
        if not protos:
            _VIS["failed"] = True
            return None
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/levelkit",
                                                                       markers=protos))
        _VIS["index"] = idx
    except Exception as e:  # noqa: BLE001  (visuals are optional; scoring never depends on them)
        print(f"levelkit: no bubble visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def bubble_world(objects, name: str, i: int = 0) -> np.ndarray:
    """World position (env frame) of vial i's bubble centre, on the body link's actual pose."""
    b = _BODIES[name]
    v = b.vials[i]
    link = objects.get(f"{name}/{name}_body")
    off = bubble(objects, name, i) * v.travel()
    local = np.array([v.at[0] + off[0], v.at[1] + off[1], v.at[2] + 0.0012])
    if link is None:
        return np.asarray(objects[name].pos, float) + local
    return np.asarray(link.pos, float) + _quat_rot(link.quat_xyzw) @ local


def _draw(objects, origin) -> None:
    m = _markers()
    if m is None:
        return
    tr, qs, sc, ids = [], [], [], []
    for name, b in _BODIES.items():
        for i, v in enumerate(b.vials):
            key = "bubble_%02x%02x%02x" % tuple(int(255 * c) for c in v.bubble_rgb)
            if name in objects:
                p = bubble_world(objects, name, i) + origin
                link = objects.get(f"{name}/{name}_body", objects[name])
                tr.append(tuple(p)), qs.append(tuple(float(q) for q in link.quat_xyzw))
                sc.append((v.bubble_r, v.bubble_r, 0.0012))
            else:
                tr.append((0.0, 0.0, -10.0)), qs.append((0, 0, 0, 1)), sc.append((1e-5,) * 3)
            ids.append(_VIS["index"][key])
    m.visualize(translations=np.array(tr, float), orientations=np.array(qs, float), scales=np.array(sc, float),
                marker_indices=np.array(ids, int))


def _pin_locked(base) -> None:
    """A foot whose jam nut is tight does not turn: hold its knob stiffly where it was when the nut locked (best
    effort; the model already ignores turning a jammed foot)."""
    for name, b in _BODIES.items():
        m = _MODEL.get(name)
        if m is None:
            continue
        for f in b.feet:
            if f.kind != "screw" or not m.locked.get(f.name):
                continue
            k = screw_key(b, f)
            mech = k.split("/")[0]
            try:
                art = base.scene[mech]
            except KeyError:
                continue
            j = list(art.joint_names).index("knob_joint")
            q = float(getattr(art.data.joint_pos, "torch", art.data.joint_pos)[0, j].item())
            hold = m.__dict__.setdefault("pin_q", {}).setdefault(f.name, q)
            kit._set_drive(art, j, target=hold, stiffness=5.0, effort=0.0)
        for f in b.feet:
            if not m.locked.get(f.name):
                m.__dict__.get("pin_q", {}).pop(f.name, None)


_orig_patch = kit._patch


def _patch(scene) -> None:
    _orig_patch(scene)  # yamkit's (and twistkit's) hooks first
    if getattr(scene, "_levelkit", False):
        return
    scene._levelkit = True
    inner_task_object, inner_states = scene.task_object, scene.object_states
    last = {"key": None}

    def task_object(spec):
        if isinstance(spec, Leveled):
            _BODIES[spec.name] = spec
        return inner_task_object(spec)

    def object_states(env, names):
        out = inner_states(env, names)
        base = env.unwrapped
        buf = getattr(base, "episode_length_buf", None)
        key = (kit.episode(), int(buf[0].item()) if buf is not None else None)
        if key[1] is None or key != last["key"]:  # once per control step
            last["key"] = key
            update(out)
            try:
                _pin_locked(base)
            except Exception as e:  # noqa: BLE001
                print(f"levelkit: lock pin failed ({e!r})", file=sys.stderr, flush=True)
            try:
                _draw(out, base.scene.env_origins[0].detach().cpu().numpy())
            except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
                if not _VIS["failed"]:
                    print(f"levelkit: visuals off ({e!r})", file=sys.stderr, flush=True)
                _VIS["failed"] = True
        return out

    scene.task_object, scene.object_states = task_object, object_states


kit._patch = _patch
if "yam_arena.scene" in sys.modules:
    _patch(sys.modules["yam_arena.scene"])
