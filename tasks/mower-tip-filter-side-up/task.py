"""A walk-behind rotary mower stands on the bench, front away from the robot, fuel run dry and the spark-plug wire off, waiting for its blade to be
changed. To reach the blade it has to be tipped onto its side, and which side goes down is decided by the engine: the
crankcase breather sits under the air filter, so with the filter and carburetor side (or the handle) down, crankcase
oil runs through the breather into the filter and intake, which soaks the filter and makes the engine smoke and refuse
to start. Tipped the other way, filter side up, nothing reaches it. 1:4 scale.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import tracekit  # noqa: E402
import yamkit as kit  # noqa: E402  (spawns the modeled mower; nothing on disk changes)
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Tip the mower over so the blade can be changed."
# Arms home: with this on, full credit also needs both arms back where they started, still, so the bench is clear for
# the person who changes the blade. Off, the job alone earns full credit.
REQUIRE_ARMS_HOME = True

# --- The mower, in its own frame: origin on the table under the deck's center, x toward the air-filter side, y toward
# the front, z up. In the scene it is turned so its front faces away from the robot (+x) and its filter side is on the
# robot's right (-y). 1:4 model of a 21 inch (53 cm) push mower.
X0, Y0 = 0.26, 0.0  # the deck's center on the table
YAW = -math.pi / 2  # mower x -> world -y, mower y -> world +x
DECK = (0.14, 0.14, 0.03)  # 56 x 56 cm full size; its underside 2 cm (8 cm) above the table
DECK_Z = 0.02
WHEEL_X, WHEEL_Y, WHEEL_R, WHEEL_W = 0.0775, 0.05, 0.02, 0.015  # 8 inch wheels outboard of the deck's sides
ENGINE = ((0.0, -0.005, 0.0775), (0.075, 0.075, 0.055))  # center, size: the engine's shroud on the deck
CRANK = np.array([0.0, -0.005, 0.065])  # the crankcase's center, low in the engine
# The crankcase breather: the port where the breather tube leaves the crankcase, toward the filter side (+x) and the
# rear (-y, the handle), 18.2 degrees above the crankcase's center. With the crankcase half full of oil the oil's
# surface passes through the center at any tilt, so the port goes under the oil once it drops below the center: at 25
# degrees with the filter side down or the handle down, and at 18 degrees toward the corner between them.
PORT_ELEV = math.atan(math.tan(math.radians(25)) * math.sqrt(0.5))
PORT = np.array([math.cos(PORT_ELEV) * math.sqrt(0.5), -math.cos(PORT_ELEV) * math.sqrt(0.5), math.sin(PORT_ELEV)])
FILTER = ((0.049, -0.012, 0.085), (0.022, 0.040, 0.030))  # the air filter's cover on the engine's filter side
LUG = (0.025, 0.030, 0.025)  # the two lift lugs on the deck's side edges, toward the front
LUG_C = (0.0575, 0.055, DECK_Z + DECK[2] + LUG[2] / 2)  # the filter-side lug; the muffler-side one is at -x
FRONT_BAR = ((0.0, 0.083, 0.056), (0.07, 0.012, 0.012))  # the front carry handle
H_ANGLE, H_LEN, H_X = math.radians(50), 0.16, 0.0725  # the handle: two bars up and back from the rear corners
H_START = (-0.07, DECK_Z + DECK[2])  # (y, z) where the bars leave the deck
H_END = (H_START[0] - H_LEN * math.cos(H_ANGLE), H_START[1] + H_LEN * math.sin(H_ANGLE))
MASS = 0.30  # kg (19 kg full size would be 0.30 x 64 = 19.2 kg)

# --- Scoring
OIL_RATE = 4.0  # mL per second through the breather while the port is under the oil
CRANKCASE_ML = 7.0  # the model's oil: a 450 mL (15 oz) push-mower crankcase at 1:4 (450 / 64)
STAIN_FULL = 2.0  # mL: the drawn stain covers the whole filter element (visual only)
UNDER_STEPS = 2  # control steps in a row (0.1 s) before the port counts as under, so a one-step PhysX pop doesn't.
# Any confirmed spell under the oil soaks the filter: oil that reaches the breather tube runs on into the filter, as
# dyed water does in the real mock's tube, so a quick dip past 25 degrees is not forgiven.
ON_SIDE = 70.0  # degrees the filter side is raised: at 70 the blade's bolt faces sideways and a wrench reaches it
OVER = 120.0  # degrees: past this the mower is upside down on its engine (and the port is under the oil anyway)
TWIST = 20.0  # degrees: the deck's front-to-back line within 20 degrees of level, on its side and not on the handle
MOVED = 0.20  # m: the deck's center more than 20 cm from its start: dragged or knocked. A clean tip moves it about
# 12 cm (it swings about a wheel edge 8.5 cm out from the center), so 20 cm is a tip plus 8 cm of shoving.
DECK_C = np.array([0.0, 0.0, DECK_Z + DECK[2] / 2])  # the deck's center in the mower's frame
RELEASED_OPEN = 0.85
NEAR = 0.02  # m: closed fingers within 2 cm of the mower's outline are holding it
HOME_TOL = 0.03
STILL = 0.001  # m per control step

GREEN = kit.Look((0.13, 0.42, 0.16), 0.45, 0.3)  # painted steel deck
DARK = kit.Look((0.20, 0.20, 0.21), 0.6, 0.2)
BLACK = kit.Look((0.05, 0.05, 0.05), 0.7)
RUBBER = kit.Look((0.08, 0.08, 0.09), 0.9)
HUB = kit.Look((0.80, 0.78, 0.70), 0.5, 0.2)
STEEL = kit.Look((0.66, 0.67, 0.69), 0.35, 0.8)
YELLOW = kit.Look((0.95, 0.75, 0.10), 0.5)
PAPER = kit.Look((0.95, 0.93, 0.86), 0.9)
RED = kit.Look((0.75, 0.10, 0.08), 0.5)
WHITE = kit.Look((0.92, 0.92, 0.90), 0.4)
BLADE = kit.Look((0.75, 0.76, 0.78), 0.3, 0.9)


def _handle_points(n):
    """Points along one handle bar from the deck to the crossbar, (y, z)."""
    return [(H_START[0] + (H_END[0] - H_START[0]) * k / n, H_START[1] + (H_END[1] - H_START[1]) * k / n)
            for k in range(n + 1)]


def _mower():
    """The deck with its four wheels, the blade under it, the engine (filter, carburetor and spark plug on the +x side,
    muffler on the -x side), two lift lugs, a front carry handle and the folded-down handle."""
    meshes, cols = [], []

    def part(c, s, look, solid=True):
        meshes.append(kit.box(c, s, look))
        if solid:
            cols.append(kit.Collider(c, size=s))

    part((0, 0, DECK_Z + DECK[2] / 2), DECK, GREEN)
    meshes.append(kit.box((0, 0, DECK_Z + DECK[2] + 0.001), (0.10, 0.10, 0.002), GREEN))
    meshes.append(kit.box((0, 0, DECK_Z - 0.001), (0.12, 0.016, 0.002), BLADE))  # the blade, seen from below
    meshes.append(kit.cylinder((0, 0, DECK_Z - 0.002), 0.008, 0.003, STEEL))  # its center bolt
    for sx in (-1, 1):
        for sy in (-1, 1):
            c = (sx * WHEEL_X, sy * WHEEL_Y, WHEEL_R)
            meshes.append(kit.cylinder(c, WHEEL_R, WHEEL_W, RUBBER, axis="x"))
            meshes.append(kit.cylinder((c[0] + sx * 0.0005, c[1], c[2]), 0.011, WHEEL_W, HUB, axis="x"))
            cols.append(kit.Collider(c, size=(WHEEL_W, 0.03, 2 * WHEEL_R)))
    ec, es = ENGINE
    part(ec, es, DARK)
    meshes.append(kit.cylinder((0, -0.005, ec[2] + es[2] / 2 + 0.005), 0.03, 0.01, BLACK))  # recoil starter cover
    meshes.append(kit.cylinder((-0.02, 0.022, ec[2] + es[2] / 2 + 0.006), 0.008, 0.012, RED))  # fuel cap
    meshes.append(kit.cylinder((-0.02, -0.025, ec[2] + es[2] / 2 + 0.004), 0.005, 0.008, YELLOW))  # oil fill cap
    fc, fs = FILTER
    part(fc, fs, YELLOW)  # air filter cover, with the pleated paper element showing through three slots
    for dz in (-0.008, 0.0, 0.008):
        meshes.append(kit.box((fc[0] + fs[0] / 2 + 0.0005, fc[1], fc[2] + dz), (0.001, 0.032, 0.004), PAPER))
    part((0.044, -0.012, 0.062), (0.012, 0.024, 0.014), STEEL)  # carburetor under the filter
    meshes.append(kit.cylinder((0.040, -0.030, 0.068), 0.003, 0.016, BLACK, axis=(1, 0, 1)))  # breather tube
    meshes.append(kit.cylinder((0.045, 0.024, 0.074), 0.004, 0.016, WHITE, axis="x"))  # spark plug
    meshes.append(kit.cylinder((0.050, 0.030, 0.090), 0.005, 0.014, BLACK, axis="z"))  # its boot, taped back up
    part((-0.045, -0.010, 0.075), (0.015, 0.035, 0.025), BLACK)  # muffler
    meshes.append(kit.box((-0.053, -0.010, 0.075), (0.002, 0.030, 0.020), STEEL))  # its heat shield
    for sx in (-1, 1):
        part((sx * LUG_C[0], LUG_C[1], LUG_C[2]), LUG, STEEL)
    bc, bs = FRONT_BAR  # carry handle on two posts
    part(bc, bs, BLACK)
    for sx in (-1, 1):
        part((sx * 0.03, 0.073, 0.052), (0.008, 0.008, 0.012), BLACK)
    pts = _handle_points(6)
    d = np.array([0.0, H_END[0] - H_START[0], H_END[1] - H_START[1]])
    for sx in (-1, 1):
        a, b = np.array([sx * H_X, *H_START]), np.array([sx * H_X, *H_END])
        meshes.append(kit.cylinder((a + b) / 2, 0.005, H_LEN, DARK, axis=d))
        for (y0, z0), (y1, z1) in zip(pts[:-1], pts[1:]):
            cols.append(kit.Collider((sx * H_X, (y0 + y1) / 2, (z0 + z1) / 2), size=(0.012, abs(y1 - y0) + 0.004,
                                                                                    abs(z1 - z0) + 0.004)))
    meshes.append(kit.cylinder((0, *H_END), 0.006, 2 * H_X + 0.012, BLACK, axis="x"))  # the crossbar grip
    cols.append(kit.Collider((0, *H_END), size=(2 * H_X + 0.012, 0.012, 0.012)))
    size = (2 * (WHEEL_X + WHEEL_W / 2), DECK[1] / 2 - H_END[0] + 0.01, H_END[1] + 0.01)
    return kit.Prop("mower", pos=(X0, Y0, TOP), size=size, mass=MASS, yaw=YAW, meshes=tuple(meshes), colliders=tuple(cols))


OBJECTS = [_mower()]


# --- Geometry the scorer reads


def _rot(q):
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def up_in_mower(m):
    """The world's up direction in the mower's frame: (0, 0, 1) on its wheels, (1, 0, 0) on its side filter-side up."""
    return _rot(m.quat_xyzw)[2, :]


def filter_side_up_deg(m):
    """How far the filter side (+x) is raised, degrees: 0 on its wheels, 90 lying on its muffler side, negative with
    the filter side down."""
    u = up_in_mower(m)
    return math.degrees(math.atan2(u[0], u[2]))


def twist_deg(m):
    """The deck's front-to-back line from level, degrees: positive with the front up (tipped back on its handle)."""
    return math.degrees(math.asin(float(np.clip(up_in_mower(m)[1], -1.0, 1.0))))


def port_under(m):
    """The breather port below the oil's surface (the crankcase's center, half full): oil runs to the filter."""
    return float(PORT @ up_in_mower(m)) < 0.0


GRIPPERS = ("left_gripper", "right_gripper")


def _released(objects):
    """No gripper holds the mower: every gripper has its fingers open (past 0.85), or its tool point and pad tips more
    than 2 cm outside the mower's outline."""
    m = objects["mower"]
    lo, hi = np.asarray(m.low, float) - NEAR, np.asarray(m.high, float) + NEAR
    for g in GRIPPERS:
        if g not in objects or kit.opening(objects, g) > RELEASED_OPEN:
            continue
        for p in (objects[g].low, objects[g].high, objects[g].pos):
            if np.all(np.asarray(p) >= lo) and np.all(np.asarray(p) <= hi):
                return False
    return True


# --- The run's history (tracekit): how long the port has been under the oil, the arms' start poses.
T = tracekit.Trace()
_MEM = {}


def _watch(objects):
    if not T.new_step(objects):
        return
    if T.step == 1:
        _MEM.clear()
        _MEM["home"] = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
        _MEM["start"] = deck_center(objects["mower"])
    grips = {g: np.asarray(objects[g].pos, float) for g in GRIPPERS if g in objects}
    last = _MEM.get("grips")
    _MEM["still"] = last is not None and all(np.linalg.norm(p - last[g]) < STILL for g, p in grips.items() if g in last)
    _MEM["grips"] = grips
    T.track("port_under", port_under(objects["mower"]), steps=UNDER_STEPS)


def oil_in_filter():
    """mL of crankcase oil that has run into the air filter this trial (it never drains back out)."""
    return min(CRANKCASE_ML, T.total_s("port_under") * OIL_RATE)


def flooded():
    """Oil reached the filter: the port was under the oil for 2 steps in a row at any time this trial (latched)."""
    return T.ever("port_under")


def deck_center(m):
    return np.asarray(m.pos, float) + _rot(m.quat_xyzw) @ DECK_C


def _arms_home(objects):
    home = _MEM.get("home") or {}
    if not home or not _MEM.get("still"):
        return False
    return all(np.linalg.norm(np.asarray(objects[g].pos, float) - p) < HOME_TOL for g, p in home.items() if g in objects)


def _score(objects):
    m = objects["mower"]
    if flooded():
        return 0.0  # oil in the filter: it must be replaced, and the engine smokes until the oil burns off
    side, twist = filter_side_up_deg(m), twist_deg(m)
    moved = np.linalg.norm(deck_center(m)[:2] - _MEM.get("start", deck_center(m))[:2])
    on_side = ON_SIDE <= side <= OVER and abs(twist) <= TWIST
    if not (on_side and moved <= MOVED and m.resting() and _released(objects)):
        return 0.0  # still on its wheels, partly tipped, on its handle, upside down, dragged, moving or still held
    return 1.0 if (_arms_home(objects) or not REQUIRE_ARMS_HOME) else 0.9


def success(objects):
    """Full score: the mower resting on its muffler side with the filter side up, no oil in the filter, let go and
    both arms home. Called first each step, so it also records the history."""
    _watch(objects)
    return _score(objects) == 1.0


def score(objects):
    """0 once oil has reached the air filter (irreversible): the breather port below the oil's surface for 2 control
    steps in a row at any time, which takes 25 degrees of tilt with the filter side or the handle down. Otherwise 1.0
    with the mower resting with its filter side raised 70 to 120 degrees, its front-to-back line within 20 degrees of
    level, its deck's center within 20 cm of where it started, no gripper holding it and both arms home and still;
    0.9 for the same with an arm not home; 0 for anything else (on its wheels, partly tipped, tipped back on its
    handle, held, moving). Possible scores: 0, 0.9, 1."""
    _watch(objects)
    return _score(objects)


# --- Oil visuals: a dark stain over the filter's paper element that grows with the oil in it, and a drip under the
# filter cover once it is soaked. Isaac Lab visualization markers; they never affect scoring.
_VIS = {"markers": None, "failed": False}
OIL_LOOK = (0.16, 0.09, 0.02)


def _markers():
    if _VIS["markers"] is not None or _VIS["failed"]:
        return _VIS["markers"]
    try:
        import isaaclab.sim as sim_utils
        from isaaclab.markers import VisualizationMarkers, VisualizationMarkersCfg

        lin = tuple(kit._linear(v) for v in OIL_LOOK)
        protos = {"oil": sim_utils.CuboidCfg(size=(1.0, 1.0, 1.0), visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=lin, roughness=0.15, metallic=0.1))}
        _VIS["markers"] = VisualizationMarkers(VisualizationMarkersCfg(prim_path="/World/Visuals/mower", markers=protos))
    except Exception as e:  # noqa: BLE001  (visuals are optional)
        print(f"mower: no oil visuals ({e!r})", file=sys.stderr, flush=True)
        _VIS["failed"] = True
    return _VIS["markers"]


def _draw(objects, origin):
    mk = _markers()
    m = objects.get("mower")
    if mk is None or m is None:
        return
    r, p = _rot(m.quat_xyzw), np.asarray(m.pos, float)
    q = tuple(float(v) for v in m.quat_xyzw)
    frac = max(0.5, min(1.0, oil_in_filter() / STAIN_FULL)) if flooded() else 0.0
    fc, fs = FILTER
    tr, qs, sc = [], [], []
    if frac > 0:  # the stain spreads up the filter's face from the breather inlet at its bottom
        h = (fs[2] + 0.002) * frac
        local = np.array([fc[0] + 0.0005, fc[1], fc[2] - fs[2] / 2 - 0.001 + h / 2])
        tr.append(p + r @ local + origin), qs.append(q), sc.append((fs[0] + 0.003, fs[1] + 0.003, h))
    else:
        tr.append(np.array([0, 0, -10.0]) + origin), qs.append((0, 0, 0, 1)), sc.append((1e-5,) * 3)
    if frac >= 1.0:  # oil dripping from the filter cover's seam
        local = np.array([fc[0] + fs[0] / 2 + 0.002, fc[1], fc[2] - fs[2] / 2 - 0.006])
        tr.append(p + r @ local + origin), qs.append(q), sc.append((0.004, 0.006, 0.012))
    else:
        tr.append(np.array([0, 0, -10.0]) + origin), qs.append((0, 0, 0, 1)), sc.append((1e-5,) * 3)
    mk.visualize(translations=np.array(tr, float), orientations=np.array(qs, float), scales=np.array(sc, float),
                 marker_indices=np.zeros(len(tr), int))


def _patch_scene(scene):
    if getattr(scene, "_mower", False):
        return
    scene._mower = True
    inner = scene.object_states

    def object_states(env, names):
        out = inner(env, names)
        try:
            _draw(out, env.unwrapped.scene.env_origins[0].detach().cpu().numpy())
        except Exception as e:  # noqa: BLE001  (never let the visuals break a run)
            if not _VIS["failed"]:
                print(f"mower: oil visuals off ({e!r})", file=sys.stderr, flush=True)
            _VIS["failed"] = True
        return out

    scene.object_states = object_states


class _Hook(kit._Hook):
    def exec_module(self, module):
        self._inner.exec_module(module)
        kit._patch(module)
        _patch_scene(module)


if "yam_arena.scene" in sys.modules:
    _patch_scene(sys.modules["yam_arena.scene"])
else:  # replace yamkit's import hook with one that also draws the oil after yamkit's patch
    sys.meta_path[:] = [f for f in sys.meta_path if not isinstance(f, kit._Hook)]
    sys.meta_path.insert(0, _Hook())


# --- The demo


def to_world(p):
    """A point in the mower's frame (standing at its start) in the world."""
    return (X0 + p[1], Y0 - p[0], TOP + p[2])


def _turn(v, th):
    """(a, z) turned by `th` the way that raises +a."""
    a, z = v
    return (a * math.cos(th) - z * math.sin(th), a * math.sin(th) + z * math.cos(th))


GRIP_Z = LUG_C[2] + 0.003  # tool point 3 mm above the lug's center: pad tips 7 mm above the deck
PIVOT_X = -(WHEEL_X + WHEEL_W / 2)  # the muffler-side wheels' outer edge, the line the mower tips about


def lug_tcp(th, sx=1):
    """The tool point on the lug on side `sx` (+1 filter side) with the mower tipped `th` radians about the other
    side's wheel edge (filter side raised for sx = +1)."""
    dx, dz = _turn((LUG_C[0] - PIVOT_X, GRIP_Z), th)
    return to_world((sx * (PIVOT_X + dx), LUG_C[1], dz))


TIP_TO = math.radians(78)  # 20 degrees past the 58 degree balance point, so it settles onto its side


def demo():
    """Left arm: grip the filter-side lug from above (fingers closing across it), lift it on an arc about the
    muffler-side wheels, turning the grip with the mower, past the balance point (58 degrees), let go so the mower
    settles onto its muffler side, back off along the grip and return home. The right arm stays home."""
    W = kit.Tilted
    p0 = lug_tcp(0.0)
    left = [W(2.6, (p0[0], p0[1], p0[2] + 0.04), FINGER_OPEN, "over the filter-side lug"),
            W(1.2, p0, FINGER_OPEN, "down around the lug"),
            W(0.8, p0, FINGER_CLOSED, "grip")]
    n = 14
    for k in range(1, n + 1):
        th = TIP_TO * k / n
        left.append(W(0.45, lug_tcp(th), FINGER_CLOSED, "tip filter side up", roll=-th))
    pe = lug_tcp(TIP_TO)
    left.append(W(0.6, pe, FINGER_CLOSED, "past balance", roll=-TIP_TO))
    left.append(W(0.8, pe, FINGER_OPEN, "let go", roll=-TIP_TO))
    back = (pe[0], pe[1] + 0.05 * math.sin(TIP_TO), pe[2] + 0.05 * math.cos(TIP_TO))  # out along the fingers
    up = (back[0], back[1] + 0.03, TOP + 0.21)  # straight up clear of the lying mower before heading home
    left += [W(1.6, back, FINGER_OPEN, "back off", roll=-TIP_TO),
             W(1.6, up, FINGER_OPEN, "lift clear", roll=-TIP_TO / 2), kit.Home(3.0), kit.Home(2.0, label="hold")]
    total = sum(w.duration for w in left)
    return {"left": left, "right": [kit.Home(total, label="stay home")]}
