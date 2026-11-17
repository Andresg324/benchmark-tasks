"""On a bar's back counter (the station table): a sixtel beer keg (sixth barrel, 23.5 cm across) standing loose, valve
up; to its right a chrome Sankey D coupler standing on a drip tray, its handle up (OFF) and pointing at the robot, its
beer line and gas line connected all the time and running to the cooler wall in front of the robot (the beer shank
and the gas regulator).

The job is a bartender helper's or bartender's: change the keg ("Stock refrigerating units with wines or bottled beer
or replace empty beer kegs", O*NET 35-9011.00). The maker's steps (Micro Matic, "How do I tap a keg", D system): handle upright (OFF); align the
two coupler lugs with the openings in the keg valve; insert the coupler and turn it clockwise about 90 degrees until
it locks; press the handle down (ON). The keg is not clamped: it stands loose on the counter, and the turn twists it
unless it is held, so one hand steadies the keg by its chime while the other seats and turns the coupler.

The bayonet is real contact geometry: the valve's top flange has two openings; the coupler's two inward lugs pass
down through them and, turned clockwise, slide under the flange until they meet the valve's stops at 90 degrees. The
handle is an over-center lever on the coupler (mechkit OverCenter): it stays up until pushed past level, then snaps
down and stays down (the handle also drives the probe into the valve). Both lines are
cablekit chains anchored at the wall and attached to the coupler's outlet and inlet for the whole trial; pulling a
line off its fitting scores 0.

Scoring (see score()): latched 0 for the handle pushed past level while the coupler is not locked (on a pressurised
keg the probe then lifts the coupler off and sprays), the keg tipped over, the coupler dropped or knocked over, or a
line pulled off its fitting; else 0.25 for the coupler seated on the valve (lugs through the openings), 0.5 for it
turned and locked, 0.75 for the handle also pressed down and locked, 1.0 for that with both hands let go and the keg
and coupler still for 1 s.
"""
import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import numpy as np  # noqa: E402

import yamkit as kit  # noqa: E402
import mechkit as mk  # noqa: E402
import cablekit as ck  # noqa: E402
import guardkit as gk  # noqa: E402
from yam_arena.constants import FINGER_CLOSED, FINGER_OPEN  # noqa: E402
from yam_arena.demo import Waypoint  # noqa: E402
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
DT = 1.0 / 20  # control step, s
RUN_LIMIT_STEPS = 1200
INSTRUCTION = "Tap the keg with the coupler."

STEEL = kit.Look((0.78, 0.79, 0.80), 0.3, 0.9)
DOME = kit.Look((0.70, 0.71, 0.72), 0.35, 0.9)
VALVE = kit.Look((0.55, 0.56, 0.58), 0.35, 0.8)
CHROME = kit.Look((0.86, 0.87, 0.89), 0.15, 1.0)
BLACK = kit.Look((0.06, 0.06, 0.07), 0.6)
RED = kit.Look((0.75, 0.10, 0.08), 0.5)
CLEAR = kit.Look((0.80, 0.84, 0.86), 0.2)  # clear vinyl beer line (drawn pale: the renderer ignores opacity)
WALL = kit.Look((0.85, 0.86, 0.84), 0.6)
TRAY_LOOK = kit.Look((0.08, 0.08, 0.09), 0.7)
MARK = kit.Look((0.95, 0.80, 0.10), 0.6)  # paint marks on the valve and the coupler (read on the top video)


def _sector(r_in, r_out, a0, a1, z0, z1, look, chamfer=(0.0, 0.0)):
    """An annular sector about the z axis (radii r_in..r_out, angles a0..a1 in degrees, heights z0..z1) as a convex
    hull collider and its mesh: rings built of these have no gaps and no corners poking in or out. `chamfer` (dr, dz)
    bevels the outer top edge (a lead-in)."""
    dr, dz = chamfer
    prof = [(r_in, z0), (r_out, z0)] + ([(r_out, z1 - dz), (r_out - dr, z1)] if dr > 0 else [(r_out, z1)]) + [(r_in, z1)]
    n = len(prof)
    raw = [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), z) for a in (a0, a1) for r, z in prof]
    cx, cy, cz = (sum(p[k] for p in raw) / len(raw) for k in range(3))
    pts = tuple((x - cx, y - cy, z - cz) for x, y, z in raw)
    faces = tuple((0, i + 1, i) for i in range(1, n - 1)) + tuple((n, n + i, n + i + 1) for i in range(1, n - 1)) + \
        tuple(f for i in range(n) for f in ((i, (i + 1) % n, n + (i + 1) % n), (i, n + (i + 1) % n, n + i)))
    mesh = kit._mesh(raw, [3] * len(faces), [j for f in faces for j in f], look)
    return kit.Collider((cx, cy, cz), hull=pts, faces=faces), mesh


def _arcs(spans, n_per_deg, r_in, r_out, z0, z1, look, chamfer=(0.0, 0.0)):
    """Sectors covering each (a0, a1) span in degrees, about `n_per_deg` degrees each."""
    out = []
    for a0, a1 in spans:
        k = max(1, round((a1 - a0) / n_per_deg))
        out += [_sector(r_in, r_out, a0 + (a1 - a0) * i / k, a0 + (a1 - a0) * (i + 1) / k, z0, z1, look, chamfer)
                for i in range(k)]
    return out


# -- the keg: a sixtel (sixth barrel, Sankey D spear). 23.5 cm across (9-1/4 in) and 59.4 cm tall (23-3/8 in); 7.5 kg empty
#    (16.5 lb), 26.3 kg full (58 lb) (KegWorks keg size guide). Simulated: its top 10 cm standing on the counter, at its
#    real width and its empty mass (the real trial uses an empty, vented keg). Frame: origin on the counter under the
#    valve, z up; the valve's openings face +-y in the keg's frame. -------------------------------------------------
KEG_C = (0.260, 0.000)
KEG_R = 0.1175
KEG_KG = 7.5
DOME_Z = 0.060  # the top plate (dome) above the counter, keg frame
CHIME_T, CHIME_H = 0.004, 0.040  # the rolled top rim: 4 mm steel, 4 cm above the dome
CHIME_TOP = DOME_Z + CHIME_H
CORE_R = 0.017  # the valve body under its flange
FLANGE_IN, FLANGE_OUT = CORE_R, 0.027  # the valve's top flange (the bayonet's ledge)
GROOVE = (DOME_Z, DOME_Z + 0.022)  # under the flange: where the lugs turn
FLANGE_Z = (DOME_Z + 0.022, DOME_Z + 0.030)
FLANGE_LEAD = (0.0015, 0.0015)  # m: the flange's top edge broken by a 1.5 mm bevel (a pressed steel edge, not a
#   lead-in: the coupler has to be set down centered)
FLANGE_TOP = FLANGE_Z[1]
NOTCH_HALF = 22.0  # deg: each opening in the flange (16 mm wide at its middle radius), centered on +-y
STOP_DEG = (-20.0, 160.0)  # the valve's two stops under the flange: the clockwise-turned lugs end against them
STOP_HALF = 8.0  # deg


def _keg():
    m = [kit.cylinder((0.0, 0.0, DOME_Z / 2), KEG_R, DOME_Z, STEEL),
         kit.cylinder((0.0, 0.0, DOME_Z + 0.0015), KEG_R - CHIME_T, 0.003, DOME),
         kit.cylinder((0.0, 0.0, (DOME_Z + FLANGE_TOP) / 2), CORE_R, FLANGE_TOP - DOME_Z, VALVE),
         kit.cylinder((0.0, 0.0, FLANGE_TOP + 0.0005), 0.010, 0.001, BLACK),  # the valve's seal ring
         kit.box((FLANGE_OUT - 0.003, 0.0, FLANGE_TOP + 0.0005), (0.005, 0.003, 0.001), MARK)]  # paint mark on +x
    c = [kit.Collider((0.0, 0.0, DOME_Z / 2), radius=KEG_R, height=DOME_Z),
         kit.Collider((0.0, 0.0, (DOME_Z + FLANGE_TOP) / 2), size=(1.4 * CORE_R, 1.4 * CORE_R, FLANGE_TOP - DOME_Z))]
    flange = [(90 + NOTCH_HALF, 270 - NOTCH_HALF), (270 + NOTCH_HALF, 450 - NOTCH_HALF)]
    stops = [(d - STOP_HALF, d + STOP_HALF) for d in STOP_DEG]
    parts = (_arcs(flange, 12, FLANGE_IN, FLANGE_OUT, *FLANGE_Z, VALVE, FLANGE_LEAD)
             + _arcs(stops, 16, CORE_R - 0.002, FLANGE_OUT, GROOVE[0] + 0.001, GROOVE[1], VALVE)
             + _arcs([(0, 360)], 11.25, KEG_R - CHIME_T, KEG_R, DOME_Z, CHIME_TOP, STEEL))
    c += [cc for cc, _ in parts]
    m += [mm for _, mm in parts]
    return tuple(m), tuple(c)


_KM, _KC = _keg()
KEG_FRICTION = (0.4, 0.4)  # static, dynamic: clean dry metal on wood, 0.2-0.6 (Engineering ToolBox, "Friction and
#   Friction Coefficients for some Common Materials"); the middle of that range, on the counter and on the fingers
KEG = kit.Prop("keg", pos=(KEG_C[0], KEG_C[1], TOP + 0.001), size=(2 * KEG_R, 2 * KEG_R, CHIME_TOP), mass=KEG_KG,
               meshes=_KM, colliders=_KC, friction=KEG_FRICTION)

# -- the coupler: a chrome-plated brass Sankey D coupler (Micro Matic 7485BB). Frame: origin at the underside of its roof (which rests on the valve's flange when it is
#    seated), z up; its two lugs point inward on its local +-x; its handle is on its -x side, the beer outlet on top
#    and the gas inlet on its +x side. -------------------------------------------------------------------------------
SKIRT_IN, SKIRT_T, SKIRT_H = 0.030, 0.004, 0.024  # square skirt round the flange: 3 mm clear of it on each side; it
#   reaches 6 mm below the lugs, so the coupler is already held round the flange while its lugs rest on top
LUG_IN, LUG_W, LUG_Z = 0.0195, 0.010, (-0.018, -0.010)  # lugs: 2.5 mm clear of the valve body, 3 mm each side of an opening
BODY_W, BODY_Z = 0.046, (0.006, 0.066)
BEER_PORT = (0.020, 0.0, 0.083)  # the outlet elbow's open end (faces +x)
GAS_PORT = (0.037, 0.0, 0.030)  # the gas inlet's open end (faces +x)
PIVOT = (-0.027, 0.0, 0.060)
UP_Q, DOWN_Q = math.radians(35.0), math.radians(-25.0)  # handle raised (OFF) / at its down stop (ON)
CENTER_Q = 0.0  # level: the over-center point
LOCK_Q = math.radians(-17.0)  # the handle pushed at least this far down is locked over center (ON)
EARLY_Q = math.radians(-3.0)  # past level by this much while the coupler is not locked: pressed too early
HANDLE_PUSH = 1.2  # N*m: the over-center spring (about 20 N at the grip, 6 cm out)
COUPLER_KG, HANDLE_KG = 0.80, 0.10


def _coupler_base():
    h = SKIRT_IN + SKIRT_T / 2
    zc = -SKIRT_H / 2
    walls = [((s * h, 0.0, zc), (SKIRT_T, 2 * (SKIRT_IN + SKIRT_T), SKIRT_H)) for s in (-1, 1)]
    walls += [((0.0, s * h, zc), (2 * SKIRT_IN, SKIRT_T, SKIRT_H)) for s in (-1, 1)]
    lugs = [((s * (LUG_IN + SKIRT_IN) / 2, 0.0, sum(LUG_Z) / 2), (SKIRT_IN - LUG_IN + 0.001, LUG_W, LUG_Z[1] - LUG_Z[0]))
            for s in (-1, 1)]
    roof = [((0.0, 0.0, 0.003), (2 * (SKIRT_IN + SKIRT_T), 2 * (SKIRT_IN + SKIRT_T), 0.006))]
    body = [((0.0, 0.0, sum(BODY_Z) / 2), (BODY_W, BODY_W, BODY_Z[1] - BODY_Z[0]))]
    beer = [((0.0, 0.0, 0.078), (0.016, 0.016, 0.024)), ((0.012, 0.0, 0.083), (0.016, 0.012, 0.012))]
    gas = [((0.030, 0.0, 0.030), (0.014, 0.012, 0.012))]
    boss = [((-0.025, 0.0, PIVOT[2]), (0.006, 0.020, 0.016))]
    meshes = ([kit.box(c, s, CHROME) for c, s in walls + lugs + roof + body + boss]
              + [kit.box(c, s, BLACK) for c, s in beer] + [kit.box(c, s, RED) for c, s in gas]
              + [kit.box((BODY_W / 2 + 0.0005, 0.0, 0.050), (0.001, 0.004, 0.012), MARK)])  # paint mark on +x
    colliders = [kit.Collider(c, size=s) for c, s in walls + lugs + roof + body + beer + gas]
    return tuple(meshes), tuple(colliders)


_CM, _CC = _coupler_base()
HANDLE = kit.Link(
    "handle", "revolute", "Y", anchor=PIVOT, lower=DOWN_Q, upper=UP_Q, start=UP_Q, mass=HANDLE_KG, friction=0.08,
    damping=0.05,
    meshes=(kit.box((-0.043, 0.0, 0.0), (0.074, 0.012, 0.010), CHROME),
            kit.box((-0.065, 0.0, 0.0), (0.032, 0.014, 0.012), BLACK)),
    colliders=(kit.Collider((-0.043, 0.0, 0.0), size=(0.074, 0.012, 0.010)),),
    law=mk.OverCenter(center=CENTER_Q, lock_at=LOCK_Q, push=HANDLE_PUSH, dead=0.03, damp=0.05, sign=-1))
TRAY_C, TRAY_SIZE = (0.200, -0.260), (0.120, 0.120, 0.008)  # black rubber drip tray, fixed
TRAY_TOP = TOP + TRAY_SIZE[2]
COUPLER_START = (TRAY_C[0], TRAY_C[1], TRAY_TOP + SKIRT_H + 0.001)
COUPLER = kit.Mechanism("coupler", pos=COUPLER_START, size=(0.20, 0.08, 0.12), mass=COUPLER_KG, meshes=_CM,
                        colliders=_CC, links=(HANDLE,))
TRAY = kit.Prop("drip_tray", pos=(TRAY_C[0], TRAY_C[1], TOP), size=TRAY_SIZE,
                meshes=(kit.box((0, 0, TRAY_SIZE[2] / 2), TRAY_SIZE, TRAY_LOOK),),
                colliders=(kit.Collider((0, 0, TRAY_SIZE[2] / 2), size=TRAY_SIZE),))

# -- the cooler wall (fixed) with the beer shank and the gas regulator, and the two lines. Beer line: 3/16 in ID clear
#    vinyl (11 mm across); gas line: 5/16 in ID red vinyl (14 mm). Both are pushed onto barbs and clamped at the
#    coupler, so they stay on through the trial unless pulled hard. ----------------------------------------------------
WALL_X = 0.560
BEER_ANCHOR = (WALL_X - 0.015, 0.100, TOP + 0.150)
GAS_ANCHOR = (WALL_X - 0.015, 0.220, TOP + 0.080)
WALL = kit.Prop("cooler_wall", pos=(WALL_X + 0.010, 0.150, TOP), size=(0.020, 0.400, 0.250),
                meshes=(kit.box((0, 0, 0.125), (0.020, 0.400, 0.250), WALL),
                        kit.cylinder((-0.0125, BEER_ANCHOR[1] - 0.150, BEER_ANCHOR[2] - TOP), 0.012, 0.005, CHROME,
                                     axis="x"),
                        kit.box((-0.020, GAS_ANCHOR[1] - 0.150, GAS_ANCHOR[2] - TOP + 0.030), (0.020, 0.040, 0.070),
                                CHROME)),
                colliders=(kit.Collider((0, 0, 0.125), size=(0.020, 0.400, 0.250)),))
LINE_RELEASE_N = 25.0  # a clamped line pulled harder than this comes off its barb (sim; the real trial watches for it)
FIT_L = 0.040  # each line's barb ferrule and clamp, on its last 4 cm (50 g with the line's end)
# Each line is held on its port by a point spring only (applied explicitly every 240 Hz substep). Its orientation
# spring is off: on the fitting's small roll inertia (about 3e-6 kg*m^2) any orientation spring in the cable kit's
# explicit form tipped the coupler over at spawn. The line's bending at the barb comes from its own
# bend joints.
FIT_TURN_K, FIT_TURN_C = 0.0, 0.0


def _line_end(port_local, name, look):
    jaw = (FIT_L, 0.0, 0.0)
    return ck.Fitting(name=name, kind="plug", size=(FIT_L, 0.020, 0.020), mass=0.050, jaw=jaw, collider_len=0.024,
                      grip=(0.012, 0.0, 0.0), raise_z=0.0, release_force=LINE_RELEASE_N, stiffness=500.0,
                      turn_stiffness=FIT_TURN_K, turn_damping=FIT_TURN_C, start="port",
                      meshes=(kit.box((FIT_L / 2, 0, 0), (FIT_L, 0.020, 0.020), look),),
                      targets=(ck.Target("port", body="coupler", point=port_local, radius=0.010),))


def _start_port(local) -> tuple:
    return (COUPLER_START[0] + local[0], COUPLER_START[1] + local[1], COUPLER_START[2] + local[2])


def _line_path(anchor, port_local, via):
    """From the wall down to the counter, along it, and up to the port, ending with the fitting pointing -x into it."""
    p = _start_port(port_local)
    end = (p[0] + FIT_L, p[1], p[2])
    return [anchor] + list(via) + [(end[0] + 0.070, end[1], end[2]), end]


def _line(name, path, segment, **kw):
    """A cable along `path` whose last joint lands on the path's end (the segments are chords, so the cable is a
    little shorter than the polyline): its segment length is searched, at a fixed count, so the fitting sits on its
    port at the start."""
    p = np.asarray(path, float)
    full = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
    best = None
    for n in range(round(full / segment) - 2, round(full / segment) + 3):  # the walk can jump at a corner: try counts
        lo, hi = 0.8 * full / n, 1.2 * full / n
        for _ in range(40):
            mid = (lo + hi) / 2
            end = ck.cable(name, path, length=n * mid, segment=mid, **kw).nodes()[-1]
            if (end - p[-1]) @ (p[-1] - p[-2]) > 0:  # past the end
                hi = mid
            else:
                lo = mid
        c = ck.cable(name, path, length=n * lo, segment=lo, **kw)
        miss = float(np.linalg.norm(c.nodes()[-1] - p[-1]))
        if best is None or miss < best[0]:
            best = (miss, c)
    return best[1]


BEER_R, GAS_R = 0.0055, 0.0071
BEER_LINE = _line("beer_line", _line_path(BEER_ANCHOR, BEER_PORT,
                                         [(0.490, 0.150, TOP + BEER_R), (0.465, 0.030, TOP + BEER_R), (0.460, -0.080, TOP + BEER_R),
                                          (0.440, -0.200, TOP + BEER_R), (0.415, -0.245, TOP + 0.030),
                                          (0.400, -0.262, TOP + 0.080)]),
                     radius=BEER_R, segment=0.030, mass=0.060, look=CLEAR, anchored=True,
                     end=_line_end(BEER_PORT, "beer_fit", BLACK), bend=0.01, bend_damping=0.004, max_bend=1.2,
                     lin_damping=1.0, ang_damping=1.5, self_collide=True)
GAS_LINE = _line("gas_line", _line_path(GAS_ANCHOR, GAS_PORT,
                                           [(0.530, 0.290, TOP + GAS_R), (0.535, 0.170, TOP + GAS_R),
                                            (0.530, -0.020, TOP + GAS_R),
                                            (0.505, -0.200, TOP + GAS_R), (0.455, -0.315, TOP + GAS_R),
                                            (0.400, -0.310, TOP + GAS_R), (0.385, -0.272, TOP + 0.035)]),
                    radius=GAS_R, segment=0.030, mass=0.080, look=RED, anchored=True,
                    end=_line_end(GAS_PORT, "gas_fit", RED), bend=0.015, bend_damping=0.006, max_bend=1.2,
                    lin_damping=1.0, ang_damping=1.5, self_collide=True)
OBJECTS = [KEG, TRAY, WALL, COUPLER, BEER_LINE, GAS_LINE]

# -- scoring ----------------------------------------------------------------------------------------------------------
SEAT_XY = 0.004  # m: the coupler's axis within 4 mm of the valve's
SEAT_DOWN, SEAT_UP = 0.002, 0.002  # m: its roof within 2 mm below / 2 mm above the flange's top (the lugs then sit
#   below the flange, which lets the locked coupler rise 2 mm; resting on the flange instead, it would stand 18 mm up)
SEAT_TILT = 6.0  # deg: the coupler upright on the valve
LOCK_DEG = 10.0  # deg: turned to within 10 degrees of a locked position (lugs under the flange, against the stops)
TIP_DEG = 10.0  # deg: the keg tilted more than this has tipped
DROP_DEG = 45.0  # deg: the coupler tilted more than this has been dropped or knocked over
OFF_TABLE_Z = TOP - 0.05  # m: the keg or coupler below this has fallen off the counter
STILL_STEP = 0.0005  # m per control step (1 cm/s)
CLEAR = 0.010  # m: let go: each whole hand (palm, both fingers, pads) 1 cm clear
SUCCESS_STEPS = round(SUCCESS_HOLD_S / DT)
COUPLER_PARTS = (gk.Block((0.0, 0.0, -SKIRT_H / 2 + 0.003), (2 * (SKIRT_IN + SKIRT_T), 2 * (SKIRT_IN + SKIRT_T),
                                                              SKIRT_H + 0.006)),
                 gk.Block((0.0, 0.0, sum(BODY_Z) / 2), (BODY_W, BODY_W, BODY_Z[1] - BODY_Z[0])),
                 gk.Block((0.008, 0.0, 0.078), (0.032, 0.016, 0.024)),
                 gk.Block((0.030, 0.0, 0.030), (0.014, 0.012, 0.012)))
HANDLE_PARTS = (gk.Block((-0.043, 0.0, 0.0), (0.074, 0.014, 0.012)),)
KEG_PARTS = (gk.Ring(KEG_R - CHIME_T, KEG_R, DOME_Z, CHIME_TOP), gk.Ring(0.0, KEG_R, 0.0, DOME_Z),
             gk.Ring(0.0, FLANGE_OUT, DOME_Z, FLANGE_TOP))
LINES = (("beer_line", "beer_line/beer_fit"), ("gas_line", "gas_line/gas_fit"))
GRIPPERS = ("left_gripper", "right_gripper")


def rotm(q) -> np.ndarray:
    x, y, z, w = (float(v) for v in q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def _tilt_deg(R) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))


def coupler_in_keg(objects):
    """(position, rotation) of the coupler's frame in the keg's frame."""
    k, c = objects["keg"], objects["coupler"]
    Rk = rotm(k.quat_xyzw)
    p = Rk.T @ (np.asarray(c.pos, float) - np.asarray(k.pos, float))
    return p, Rk.T @ rotm(c.quat_xyzw)


def turn_deg(objects) -> float:
    """The coupler's yaw on the valve (keg frame, degrees, -180..180): +90 or -90 with its lugs in the openings, 0 or
    180 turned clockwise a quarter turn to the locked positions."""
    _, R = coupler_in_keg(objects)
    return math.degrees(math.atan2(R[1, 0], R[0, 0]))


def seated(objects) -> bool:
    """The coupler down on the valve with its lugs through the openings (below the flange), upright."""
    p, R = coupler_in_keg(objects)
    dz = float(p[2]) - FLANGE_TOP
    return (math.hypot(float(p[0]), float(p[1])) <= SEAT_XY and -SEAT_DOWN <= dz <= SEAT_UP
            and _tilt_deg(R) <= SEAT_TILT)


def locked(objects) -> bool:
    """Seated and turned to a locked position: its lugs under the flange."""
    if not seated(objects):
        return False
    a = turn_deg(objects)
    return min(abs(a), 180.0 - abs(a)) <= LOCK_DEG


def handle_q(objects) -> float:
    return float(kit.joint(objects, "coupler/handle").pos)


def lines_on(objects) -> bool:
    return all(ck.attached(objects, key) is not None for _, key in LINES)


def let_go(objects) -> bool:
    """Both whole hands (palm, both fingers and their pads, at their present opening) 1 cm clear of the coupler, its
    handle and the keg: an open hand resting on any of them has not let go."""
    for g in GRIPPERS:
        if g not in objects:
            continue
        for obj, parts in (("coupler", COUPLER_PARTS), ("coupler/handle", HANDLE_PARTS), ("keg", KEG_PARTS)):
            if gk.hand_gap(objects, g, obj, parts, upto=CLEAR) < CLEAR:
                return False
    return True


class Ledger:
    """One trial: the fault latches (pressed early, keg tipped, coupler dropped, a line pulled off), the best stage
    reached now, and how long the full end state has held."""

    def __init__(self):
        self.episode = kit.episode()
        self.key = None
        self.early = gk.Latch(steps=1)  # any single step pressed past level while unlocked
        self.tipped = gk.Latch(steps=2)
        self.dropped = gk.Latch(steps=2)
        self.off = gk.Latch(steps=2)
        self.last = None
        self.good = 0
        self.stage = 0.0

    def step(self, objects) -> None:
        k, c = objects["keg"], objects["coupler"]
        lock = locked(objects)
        q = handle_q(objects)
        self.early.feed(objects, q <= EARLY_Q and not lock)
        self.tipped.feed(objects, _tilt_deg(rotm(k.quat_xyzw)) > TIP_DEG or float(k.pos[2]) < OFF_TABLE_Z)
        self.dropped.feed(objects, _tilt_deg(rotm(c.quat_xyzw)) > DROP_DEG or float(c.pos[2]) < OFF_TABLE_Z)
        self.off.feed(objects, not lines_on(objects))
        if lock and q <= LOCK_Q:
            self.stage = 0.75
        elif lock:
            self.stage = 0.5
        elif seated(objects):
            self.stage = 0.25
        else:
            self.stage = 0.0
        p = np.concatenate([np.asarray(k.pos, float), np.asarray(c.pos, float)])
        still = self.last is not None and float(np.max(np.abs(p - self.last))) <= STILL_STEP
        self.last = p
        self.good = self.good + 1 if (self.stage == 0.75 and still and let_go(objects)) else 0

    def faulted(self) -> bool:
        return self.early.on or self.tipped.on or self.dropped.on or self.off.on


_L = None


def _update(objects):
    global _L
    if _L is None or _L.episode != kit.episode():
        _L = Ledger()
    n = kit._EPISODE.get("last")
    key = (kit.episode(), n) if n is not None else ("obj", id(objects))
    if key == _L.key:
        return
    _L.key = key
    _L.step(objects)
    _SEAT.step(objects, n)


def success(objects):
    """Locked on the valve, handle down over center, both hands let go, keg and coupler still: this step counts toward
    the 1 s hold."""
    _update(objects)
    return not _L.faulted() and _L.good >= 1


def score(objects):
    """1.0: the coupler seated on the valve, turned clockwise to its locked position, its handle pressed down and
    locked over center, both hands 1 cm clear of the coupler, its handle and the keg, and keg and coupler still for 1
    s. 0.75: locked with the handle down, not yet let go or still. 0.5: seated and turned to lock, handle still up.
    0.25: seated on the valve with its lugs through the openings, not turned. Otherwise 0, including doing nothing.
    The whole trial scores 0 if the handle is pushed past level while the coupler is not locked, the keg tips more
    than 10 degrees, the coupler is dropped or knocked over (tilted more than 45 degrees) or falls off the counter, or
    a line is pulled off its fitting."""
    _update(objects)
    if _L.faulted():
        return 0.0
    s = _L.stage
    if s == 0.75 and _L.good >= SUCCESS_STEPS:
        s = 1.0
    return s


# -- scripted demo: from the fixed layout, closed loop over the valve -------------------------------------------------
# Left hand: pinches the keg's chime in front of the valve on the left (fingers across the 4 mm rim, leaned out over
# it) and holds the keg steady until the coupler is on, then lets go. Right hand: takes the coupler by its body
# (fingers across it), lifts it over the chime, turns it a quarter turn counter-clockwise on the way so its lugs line
# up with the valve's openings, lines it up over the valve by eye (_Seat: re-aimed and re-levelled from where it
# actually hangs), lowers it still held until the lugs are through the openings, lets go, grips it again where it
# sits, turns it clockwise a quarter turn into the stops, lets go, and pushes the handle down where it actually is
# with closed fingers until it locks.
FINGER_HALF_OPEN = 0.047  # m: each finger's inner face from the tool point when fully open
GRIP_UP = 0.046  # m: the tool point this far above the coupler's origin: between its two ports, tips 2.7 cm over its roof
SEAT_PUSH = 0.006  # m: commanded this far below seated
HOVER = 0.060  # m: the coupler's roof commanded this far above the flange as it comes over the valve (the loaded arm
#   sags 2-3 cm there and keeps settling, so the line-up then holds its measured height)
CARRY_UP = 0.130  # m: lifted this far: its skirt passes 4.4 cm over the chime (2.9 cm with the 1.5 cm the loaded arm sags)
RIM_IN = 0.075  # m: it is carried in over the rim to this far from the valve's axis on the right before it comes down
#   (the arm cannot lift that high right over the valve; at the rim it can)
PRESS_X = -0.090  # m: the handle is pushed on its black grip (coupler frame x), 6.3 cm out from its pivot
PRESS_PAST = 0.030  # m: the fingertips commanded this far below the handle's top at its down stop
TIP_PAST_TCP = 0.0128  # m: closed fingertips reach this far below the tool point
STEADY_DEG = 150.0  # deg (keg frame): the left hand pinches the chime here, in front of the valve on the left: clear of
#   the handle's sweep (front right, while it turns from -y to -x) and of the lines' sweep (back left, from +y to +x)
STEADY_LEAN = 35.0  # deg: that hand leans out over the rim, fingertips in, so its palm stays clear of the right hand turning
CHIME_GRIP = 0.010  # m: the left tool point this far below the chime's top (pads on the rim, tips above the dome)
# Over the valve, with the coupler and both lines on it and the grip leaned, the right arm settles short of its target
# toward its base (y), the coupler sits off-center along the fingers' closing axis, and the wrist sags under the load.
# The plan starts aimed that much the other way; the closed-loop line-up (_Seat) then works from the coupler's pose.
HOVER_LAG = (0.0, -0.050)  # m, world: where the leaned, loaded hand ends up minus where it was sent, hovering
SEAT_LAG = (0.0, -0.033)  # m: the same once it is down on the valve (the arm settles closer as it comes down)
GRIP_OFF = (0.020, 0.0)  # m, world, at grip yaw 90 deg: the coupler's axis minus the tool point
SAG_ROLL = 0.35  # rad: over the valve the loaded wrist sags the coupler about 20 deg about x (its top toward +y); the
#   plan leans the grip the other way while it hovers, so the coupler hangs level, and levels the grip again as the
#   valve takes the coupler's weight
TURN_PAST = math.radians(10.0)  # the grip turns this far past square, so the lugs end firmly against the valve's stops
#   (the hand turns through its grip's give)
SAFE_Z = TOP + 0.215
PARK_L, PARK_R = (0.180, 0.300, TOP + 0.150), (0.180, -0.300, TOP + 0.150)  # where each arm waits, out of the way


def finger_for(half: float) -> float:
    """Gripper command whose finger faces stand `half` m from the tool point."""
    return FINGER_CLOSED + (FINGER_OPEN - FINGER_CLOSED) * max(0.0, half) / FINGER_HALF_OPEN


def _w(d, pos, f, label, yaw=0.0):
    return Waypoint(float(d), tuple(float(v) for v in pos), float(f), label, float(yaw))


KEG_Z0 = TOP + 0.001  # the keg's origin (it stands 1 mm proud at the start)
VALVE_XY = KEG_C
SEAT_TCP_Z = KEG_Z0 + FLANGE_TOP + GRIP_UP
L_TIMES = (2.5, 1.5, 0.8, 1.5, 2.5, 1.5, 0.5, 0.8, 0.8, 2.5, 0.6, 0.8, 1.0, 2.2, 2.0, 1.0)
# Closed-loop line-up (a person watches the coupler over the valve): rounds of SERVO_S, each re-aimed from where the
# coupler actually hangs, then a held lowering until its skirt is round the flange's lead-in, then it is let go.
SERVO_ROUNDS, SERVO_S = 10, 0.5
LOWER_ROUNDS, LOWER_S = 10, 0.5
SERVO_GAIN = 0.5  # of the coupler's measured miss taken out each round
SERVO_STEP = 0.008  # m: at most this much re-aim per round
CENTER_TOL, LEVEL_TOL = 0.002, 5.0  # m, deg: centered and level (a lug has 3 mm each side in its opening)
LEVEL_GAIN, LEVEL_STEP = 0.6, 0.08  # of the measured tilt (rad) taken out of the grip's lean each round, at most
HANG_UP = 0.030  # m: while it is lined up, the coupler's roof is held this far over the flange (skirt 6 mm clear),
#   measured: the loaded arm sags 2-3 cm and by how much varies, so the height is read too
RELEASE_UP = 0.010  # m: lowered, still held, until the coupler's roof is this far over the flange (lugs through the
#   openings), then let go
LINE_UP_S = SERVO_ROUNDS * SERVO_S + LOWER_ROUNDS * LOWER_S
STEADY_OFF_S = 2.2  # s: the right hand waits over the locked coupler while the left lets go of the keg and lifts
COUPLER_DONE_S = sum(L_TIMES[:12]) - L_TIMES[6] + LINE_UP_S + 1.8  # the coupler is turned to lock and let go: the
#   steady hand lets go of the keg before the handle is pressed (pressing down does not turn the keg)


def _handle_top_z(q: float) -> float:
    """World height of the handle's top face at PRESS_X with the handle at angle q (coupler seated, keg level)."""
    s = (PIVOT[0] - PRESS_X) / math.cos(q)
    return KEG_Z0 + FLANGE_TOP + PIVOT[2] + s * math.sin(q) + 0.005 / math.cos(q)


def _aim(x, y, yaw, loaded=True, lag=SEAT_LAG):
    """Where to send the tool point so the held coupler's axis lands on (x, y) at grip yaw `yaw` (loaded=False: where
    the tool point already is once the coupler is locked, so the unloaded arm does not spring sideways on letting go)."""
    c, s = math.cos(yaw - math.pi / 2), math.sin(yaw - math.pi / 2)
    ox, oy = c * GRIP_OFF[0] - s * GRIP_OFF[1], s * GRIP_OFF[0] + c * GRIP_OFF[1]
    lag = lag if loaded else (0.0, 0.0)
    return x - lag[0] - ox, y - lag[1] - oy


def _lean(d, pos, f, label, yaw):
    """A waypoint whose grip leans SAG_ROLL about x, against the loaded wrist's sag."""
    return kit.Tilted(float(d), tuple(float(v) for v in pos), float(f), label, float(yaw), SAG_ROLL, 0.0)


def coupler_plan() -> list:
    """The right hand: the coupler on, turned to lock, the handle pressed."""
    x0, y0, z0 = COUPLER_START
    vx, vy = VALVE_XY
    wide, shut = finger_for(0.034), FINGER_CLOSED
    grip = z0 + GRIP_UP
    lifted = grip + CARRY_UP
    hover = SEAT_TCP_Z + HOVER
    seat = SEAT_TCP_Z - SEAT_PUSH
    quarter = math.pi / 2
    px = vx + PRESS_X
    lifted_off = SEAT_TCP_Z + 0.045  # fingers still astride the beer outlet (the arm reaches no higher over the valve)
    over_handle = _handle_top_z(UP_Q) + TIP_PAST_TCP + 0.010
    press = _handle_top_z(DOWN_Q) + TIP_PAST_TCP - PRESS_PAST
    t = L_TIMES
    return [_w(t[0], (x0, y0, SAFE_Z), wide, "over the coupler"),
            _w(t[1], (x0, y0, grip), wide, "down round its body"),
            _w(t[2], (x0, y0, grip), shut, "grip it"),
            _w(t[3], (x0, y0, lifted), shut, "lift it, lines trailing"),
            _w(t[4], (vx, vy - RIM_IN, lifted), shut, "in over the rim, turned a quarter turn left", quarter),
            _lean(t[5], (*_aim(vx, vy, quarter, lag=HOVER_LAG), hover), shut, "down, lugs over the openings", quarter),
            *[_lean(SERVO_S, (*_aim(vx, vy, quarter, lag=HOVER_LAG), hover), shut, f"line up {k + 1}", quarter)
              for k in range(SERVO_ROUNDS)],
            *[_lean(LOWER_S, (*_aim(vx, vy, quarter, lag=HOVER_LAG), hover), shut, f"lower it, still held {k + 1}",
                    quarter) for k in range(LOWER_ROUNDS)],
            kit.Tilted(t[7], (*_aim(vx, vy, quarter, loaded=False), hover), FINGER_OPEN, "let it go onto the valve",
                       quarter),
            _w(t[8], (*_aim(vx, vy, quarter, loaded=False), hover), FINGER_OPEN, "let it settle on its own weight",
               quarter),
            _w(1.0, (*_aim(vx, vy, quarter, loaded=False), SEAT_TCP_Z), FINGER_OPEN, "down round it again", quarter),
            _w(0.8, (*_aim(vx, vy, quarter, loaded=False), seat), shut, "grip it again", quarter),
            _w(t[9], (*_aim(vx, vy, -TURN_PAST, loaded=False), seat), shut,
               "turn it clockwise a quarter turn to the stops", -TURN_PAST),
            _w(t[10], (*_aim(vx, vy, -TURN_PAST, loaded=False), seat), shut, "hold", -TURN_PAST),
            _w(t[11], (*_aim(vx, vy, 0.0, loaded=False), seat), FINGER_OPEN, "let go, fingers wide"),  # back to square
            _w(t[12], (*_aim(vx - 0.020, vy, 0.0, loaded=False), lifted_off), FINGER_OPEN,
               "lift clear, back toward the handle"),
            _w(STEADY_OFF_S, (*_aim(vx - 0.020, vy, 0.0, loaded=False), lifted_off), FINGER_OPEN,
               "wait while the keg hand moves off"),
            _w(1.0, (px, vy, lifted_off), FINGER_OPEN, "back over the handle, fingers still wide"),
            _w(t[13] - 1.0, (px, vy, over_handle), shut, "over the handle, fingers closed"),
            _w(t[14], (px, vy, press), shut, "push the handle down past level until it locks"),
            _w(t[15], (px, vy, press), shut, "hold"),
            _w(1.0, (px, vy, over_handle), shut, "lift off"),
            _w(1.5, PARK_R, wide, "out of the way"),
            _w(20.0, PARK_R, wide, "wait")]


def _tw(d, pos, f, label, yaw, tilt):
    """A waypoint whose grip is also leaned by `tilt` about the horizontal axis at `yaw` (the rim's tangent)."""
    return kit.Tilted(float(d), tuple(float(v) for v in pos), float(f), label, float(yaw),
                      float(tilt * math.cos(yaw)), float(tilt * math.sin(yaw)))


def steady_plan() -> list:
    """The left hand: the keg held steady by its chime."""
    a = math.radians(STEADY_DEG)
    r = KEG_R - CHIME_T / 2
    cx, cy = KEG_C[0] + r * math.cos(a), KEG_C[1] + r * math.sin(a)
    z = KEG_Z0 + CHIME_TOP - CHIME_GRIP
    yaw = math.atan2(math.sin(a + math.pi / 2), math.cos(a + math.pi / 2))  # fingers close along the radius
    lean = math.radians(STEADY_LEAN)
    wide = finger_for(0.022)  # 2 cm each side of the 4 mm rim while the leaned fingers come down over it
    above = z + 0.050
    head = [_w(2.0, (0.200, 0.150, above), wide, "toward the keg, turning the grip", yaw),
            _tw(1.5, (cx, cy, above), wide, "over the chime, in front of the valve", yaw, lean),
            _tw(1.5, (cx, cy, z), wide, "down astride the rim", yaw, lean),
            _tw(0.8, (cx, cy, z), FINGER_CLOSED, "pinch the chime", yaw, lean)]
    return head + [_tw(COUPLER_DONE_S - sum(w.duration for w in head), (cx, cy, z), FINGER_CLOSED,
                       "hold the keg steady", yaw, lean),
                   _tw(0.8, (cx, cy, z), finger_for(0.024), "let go", yaw, lean),
                   _tw(1.2, (cx, cy, above), finger_for(0.024), "lift clear", yaw, lean),
                   _w(3.0, PARK_L, wide, "out of the way"),
                   _w(20.0, PARK_L, wide, "wait")]


class _Seat:
    """The demo's eyes over the valve (closed loop, as a person lines a coupler up by eye): before each line-up round
    it reads where the held coupler's skirt hangs and how it tilts, to 1 mm and 1 degree, and re-aims that round's
    waypoint and lean; once centered and level it holds there for SERVO_S, then lowers it, still held and still
    re-aimed, until the skirt is round the flange; it lets go where the hand actually is; and it grips the settled
    coupler again where it actually sits. It only moves the hand's targets: nothing acts on the coupler but contact."""

    def __init__(self):
        self.W, self.starts, self.ep, self.done = None, None, None, set()

    def arm(self, w: list) -> None:
        self.W, self.starts, self.ep, self.done = w, np.cumsum([0.0] + [x.duration for x in w]), None, set()
        self.idx = {x.label: i for i, x in enumerate(w)}
        self.ok = 0

    def _look(self, objects):
        c, k = objects["coupler"], objects["keg"]
        Rc = rotm(c.quat_xyzw)
        skirt = np.asarray(c.pos, float) + Rc @ np.array([0.0, 0.0, -SKIRT_H / 2])
        miss = np.round((skirt[:2] - np.asarray(k.pos, float)[:2]) * 1000.0) / 1000.0
        p, _ = coupler_in_keg(objects)
        up = round(float(p[2]) - FLANGE_TOP, 3)
        return miss, np.round(Rc[:2, 2], 2), up

    def _say(self, msg):
        if os.environ.get("KEG_DEBUG"):  # probe telemetry only
            print(f"[seat] {msg}", flush=True)

    def step(self, objects, n) -> None:
        if self.W is None or n is None or "coupler" not in objects:
            return
        if self.ep != kit.episode():
            self.ep, self.done, self.ok = kit.episode(), set(), 0
        t = n * DT
        for j, w in enumerate(self.W):
            if j in self.done or t < self.starts[j] - 0.1:
                continue
            if t >= self.starts[j]:  # already under way: too late to aim it
                self.done.add(j)
                continue
            lab = w.label
            if not (lab.startswith("line up") or lab.startswith("lower it") or lab == "let it go onto the valve"
                    or lab == "down round it again" or lab == "back over the handle, fingers still wide"):
                self.done.add(j)
                continue
            self.done.add(j)
            prev = self.W[j - 1]
            miss, cz, up = self._look(objects)
            tilt = math.degrees(math.asin(min(1.0, float(np.hypot(*cz)))))
            if lab.startswith("line up") or lab.startswith("lower it"):
                good = float(np.hypot(*miss)) <= CENTER_TOL and tilt <= LEVEL_TOL
                self.ok = self.ok + 1 if good else 0
                fix = np.clip(-SERVO_GAIN * miss, -SERVO_STEP, SERVO_STEP)
                z = prev.pos[2]
                if lab.startswith("line up") or (lab.startswith("lower it") and self.ok < 1 and up > RELEASE_UP + 0.006):
                    # hang it clear of the flange while it is centered (and while it drifts off center again)
                    z = z + float(np.clip(HANG_UP - up, -0.006, 0.010))
                if lab.startswith("lower it") and self.ok >= 1:  # only while centered and level: otherwise hold
                    z = z - float(np.clip(up - RELEASE_UP, 0.0, 0.010))  # down, still held and re-aimed
                w.pos = (prev.pos[0] + float(fix[0]), prev.pos[1] + float(fix[1]), z)
                w.roll = float(np.clip(prev.roll + np.clip(LEVEL_GAIN * cz[1], -LEVEL_STEP, LEVEL_STEP), -0.1, 0.7))
                w.pitch = float(np.clip(prev.pitch - np.clip(LEVEL_GAIN * cz[0], -LEVEL_STEP, LEVEL_STEP), -0.3, 0.3))
                self._say(f"t={t:.2f} {lab}: miss {np.round(miss * 1000).tolist()} mm tilt {tilt:.0f} deg up "
                          f"{up * 1000:.0f} mm -> roll {math.degrees(w.roll):.0f} pitch {math.degrees(w.pitch):.0f}")
            elif lab == "let it go onto the valve":  # open where the hand is now, so the unloaded arm stays put
                g = np.asarray(objects["right_gripper"].pos, float)
                w.pos, w.roll, w.pitch = (float(g[0]), float(g[1]), float(g[2])), prev.roll, prev.pitch
                for i in range(j + 1, self.idx["down round it again"]):
                    self.W[i].pos = w.pos
                self._say(f"t={t:.2f} let go at {np.round(g * 1000).tolist()}, miss {np.round(miss * 1000).tolist()}")
            elif lab == "back over the handle, fingers still wide":  # press the handle where it actually is
                c = objects["coupler"]
                Rc = rotm(c.quat_xyzw)
                h = np.asarray(c.pos, float) + Rc @ np.array([PRESS_X, 0.0, 0.0])
                hx, hy = round(float(h[0]), 3), round(float(h[1]), 3)
                i0 = self.idx["push the handle down past level until it locks"]
                for i in (j, self.idx["over the handle, fingers closed"], i0, i0 + 1, self.idx["lift off"]):
                    self.W[i].pos = (hx, hy, self.W[i].pos[2])
                self._say(f"t={t:.2f} press the handle at {[hx * 1000, hy * 1000]}")
            else:  # grip the settled coupler again where it is
                c = np.asarray(objects["coupler"].pos, float)
                cx, cy = round(float(c[0]), 3), round(float(c[1]), 3)
                quarter = math.pi / 2
                for lab2, yaw, dx in (("down round it again", quarter, 0.0), ("grip it again", quarter, 0.0),
                                      ("turn it clockwise a quarter turn to the stops", -TURN_PAST, 0.0),
                                      ("hold", -TURN_PAST, 0.0), ("let go, fingers wide", 0.0, 0.0),
                                      ("wait while the keg hand moves off", 0.0, -0.020),
                                      ("lift clear, back toward the handle", 0.0, -0.020)):
                    i = self.idx[lab2] if lab2 != "hold" else self.idx["turn it clockwise a quarter turn to the stops"] + 1
                    ax, ay = _aim(cx + dx, cy, yaw, loaded=False)
                    self.W[i].pos = (ax, ay, self.W[i].pos[2])
                self._say(f"t={t:.2f} grip again at coupler {[cx * 1000, cy * 1000]} up {up * 1000:.0f} mm")


_SEAT = _Seat()


def demo():
    """Left hand steadies the keg by its chime; right hand seats the coupler, turns it to lock and presses the handle."""
    r = coupler_plan()
    _SEAT.arm(r)
    return {"left": steady_plan(), "right": r}
