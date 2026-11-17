"""A new spin-on oil filter (Kawasaki 16097-0007, the Teryx and Teryx4 750 filter) stands on the bench beside its
mount: the M20 x 1.5 nipple low on the front of the crankcase, its axis level and pointing at the robot, with the
frame's cross tube under it and the CVT intake duct beside it, as on the vehicle with its bottom guard removed. The
tool Kawasaki's torque figure assumes lies on the bench: a 65 mm, 14-flute cap-type filter wrench on a 24-inch
ratchet. Fitting it is the engine maker's procedure: start the filter square on the thread by hand (started crooked it
cross-threads and ruins the nipple's thread), spin it on until its gasket contacts the mount, then tighten it with the
wrench to the specified 18 N*m, about three quarters of a turn past contact, and take the wrench off. Left at contact
or a little past it, the filter can work loose under vibration and leak; cranked much past it, the gasket can be
crushed and blow out under oil pressure.
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np  # noqa: E402
import yamkit as kit  # noqa: E402
import guardkit as gk  # noqa: E402  (after yamkit)
import filterkit as fk  # noqa: E402  (after yamkit)
from yam_arena.task_file import TABLE_TOP  # noqa: E402

SCENE = "station"
TOP = TABLE_TOP[SCENE]
SUCCESS_HOLD_S = 1.0
SCORE_OVER = "end"
INSTRUCTION = "Fit the new oil filter."


def _rot_x(a):
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def _rot_y(deg):
    a = math.radians(deg)
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


def _rot_z(a):
    return np.array([[math.cos(a), -math.sin(a), 0], [math.sin(a), math.cos(a), 0], [0, 0, 1]])


def _quat_to_m(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                     [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                     [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def _m_to_quat(r):
    from yam_arena.ik import matrix_to_quat_xyzw
    return tuple(float(v) for v in matrix_to_quat_xyzw(np.asarray(r, float)))


def _qz_wxyz(a):  # a box turned `a` rad about its frame's z axis (USD orient: w, x, y, z)
    return (math.cos(a / 2), 0.0, 0.0, math.sin(a / 2))


# --- The installed location ----------------------------------------------------------------------------------------
# Positions in metres in the task frame (x forward from the arms, y to the robot's left, z up). The nipple sticks out
# of the crankcase's front face toward the robot, its axis level: Kawasaki's parts diagram (E1710, Teryx4 750 "Oil
# Pump/Oil Filter") draws the filter along the vehicle's FRONT arrow, low on the front of the engine, reached from
# below once the bottom guard is off. The gasket land's center:
X_LAND, Y_F, Z_F = 0.455, 0.05, TOP + 0.13
BOSS_R, BOSS_L = 0.038, 0.008  # the machined boss the gasket seals on, 7.6 cm across, 8 mm proud of the case
# The nipple's thread and the filter's, ISO 965-1/-2 M20 x 1.5, the general-purpose fit (6H internal on the filter,
# 6g external on the nipple), in mm. Nipple (6g): major 19.732-19.968, pitch diameter 18.844-18.994, root 18.128 at
# most; ISO 4753: its end is chamfered (about 45 degrees) down to no more than the root diameter. Filter (6H): minor
# (its crests) 18.376-18.676, pitch diameter 19.026-19.226. Modeled at the middle of each band: the filter's crests
# (18.526) over the nipple's chamfered end (18.128) leave 0.20 mm all round (0.12 at the tightest, 0.27 at the
# loosest); the pitch diameters, 0.207 mm (0.032 to 0.382).
NIP_CREST_R = 0.019968 / 2  # the nipple's crests, drawn
NIP_ROOT_R = 0.018128 / 2  # its root and its chamfered end: its collider
BORE_R = 0.018526 / 2  # the filter's crests: the bore's colliders
BORE_Z0, BORE_Z1 = 0.003, 0.0105  # the threaded bore, from the gasket's face (the filter's frame)
LEAD = NIP_CREST_R - NIP_ROOT_R  # 0.92 mm: the chamfer, axially
PD_CLEAR = (19.126 - 18.919) / 1000  # the pitch diameters' clearance (middles of the bands)
CASE_X0 = X_LAND + BOSS_L  # the crankcase's front face
CASE = ((CASE_X0, CASE_X0 + 0.20), (Y_F - 0.17, Y_F + 0.17), (TOP, TOP + 0.24))  # its lower front (x, y, z spans)
TUBE_R, TUBE_X = 0.016, 0.418  # the frame's lower cross tube (32 mm steel), across under the filter
TUBE_Y = (Y_F - 0.17, Y_F + 0.15)  # its span (cut off at the frame's lower rails)
TUBE_Z = Z_F - 0.033 - 0.045 - TUBE_R  # 4.5 cm of air under the can
DUCT_R, DUCT_X = 0.030, 0.428  # the CVT intake duct (60 mm), standing beside the filter on its right
DUCT_Y = Y_F - 0.033 - 0.030 - DUCT_R  # 3 cm of air between the can and the duct
DUCT_Z0 = TUBE_Z + TUBE_R + 0.002  # it comes down to the cross tube
PAN = ((0.400, 1.010), (Y_F - 0.457, Y_F + 0.457), 0.038)  # the drip pan (Lisle 19722, 24 x 36 x 1.5 in)

# --- The filter: Kawasaki 16097-0007 (the HF204 / KN-204 size: 66 mm across, 71 mm tall, M20 x 1.5) ----------------
CAN_R, CAN_H = 0.033, 0.071
FILTER_MASS = 0.25
FLUTE_L = 0.014  # the fluted end the cap wrench fits over
FLUTE_N = 14
FLUTE_IN, FLUTE_W = 0.031, 0.005  # the flutes' roots (radius) and their width; their crests are the can's 33 mm
# The thread: M20 x 1.5, right-handed; three turns (4.5 mm, an estimate, measured on the rig) from the first thread
# catching to gasket contact. Before contact it runs on with about 0.3 N*m. Past contact the squeezed gasket resists
# with a torque that rises linearly to Kawasaki's 18 N*m at three quarters of a turn (the makers' equivalence: three
# quarters of a turn past contact gives the specified torque) and on at the same slope (29 N*m at 1.25 turns); a hard
# stop at 1.5 turns (the gasket flat). Kawasaki (Teryx4 750 owner's manual, torque table, p. 101): "Oil Filter 18 N*m
# (1.8 kgf*m, 13 ft*lb)".
TORQUE_SPEC = 18.0
THREAD = fk.Thread(pitch=0.0015, turns_to_seat=3.0, run_on=0.3, gasket_rate=TORQUE_SPEC / math.radians(270.0),
                   max_past=math.radians(540.0), root_r=NIP_ROOT_R, crest_r=BORE_R, bore_z0=BORE_Z0, bore_z1=BORE_Z1,
                   collar_r=0.0125, lead=LEAD,
                   d2=0.019026, pd_clear=PD_CLEAR)
D0 = THREAD.d0  # 4.5 mm: the gasket face's distance from the land when the thread catches
NIP_L = THREAD.nip_len  # the nipple stands 8.4 mm out of the land: its chamfered end inside the bore at the catch
CROSS_DEG = math.degrees(THREAD.cross_tilt)  # 2.6 degrees: started more crooked than this, it cross-threads

# Scoring: degrees past gasket contact.
LOOSE, SNUG = 90.0, 180.0
GOOD_LO, GOOD_HI = 240.0, 360.0
CRUSH = 450.0
GRIP_SLACK = 45.0  # at most this much of the turning past contact done without a hand
CARRY = 30.0  # a holding wrist's roll not yet matched by the filter, kept at most 30 degrees

ALU = kit.Look((0.62, 0.63, 0.64), 0.5, metallic=0.5)  # cast aluminum crankcase
LAND = kit.Look((0.85, 0.86, 0.87), 0.25, metallic=0.8)
STEEL = kit.Look((0.55, 0.56, 0.58), 0.35, metallic=0.7)
FRAME = kit.Look((0.10, 0.10, 0.11), 0.5, metallic=0.3)  # black powder-coated frame tube
DUCT = kit.Look((0.14, 0.14, 0.15), 0.85)  # black rubber duct
PAN_LOOK = kit.Look((0.08, 0.08, 0.09), 0.7)  # black polyethylene pan
CAN = kit.Look((0.07, 0.07, 0.08), 0.35, metallic=0.2)  # black painted can
WHITE = kit.Look((0.95, 0.95, 0.93), 0.6)
RUBBER = kit.Look((0.06, 0.06, 0.06), 0.8)
BLACK = kit.Look((0.05, 0.05, 0.05), 0.6)
CHROME = kit.Look((0.80, 0.81, 0.83), 0.2, metallic=0.95)
CAP_LOOK = kit.Look((0.30, 0.31, 0.33), 0.4, metallic=0.8)


def _ring(r_in, r_out, z0, z1, n=12):
    """A ring of n boxes about z: the colliders of a hollow part."""
    chord = 2 * r_out * math.tan(math.pi / n) + 0.001
    out = []
    for k in range(n):
        a = 2 * math.pi * k / n
        rm = (r_in + r_out) / 2
        out.append(kit.Collider((rm * math.cos(a), rm * math.sin(a), (z0 + z1) / 2),
                                size=(r_out - r_in, chord, z1 - z0), quat_wxyz=_qz_wxyz(a)))
    return out


def _qmul(a, b):  # (w, x, y, z) quaternion product
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return (aw * bw - ax * bx - ay * by - az * bz, aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx, aw * bz + ax * by - ay * bx + az * bw)


def _ribs(r_in, r_out, width, z0, z1, n, offset=0.0, look=None, point=0):
    """n radial ribs about z: their colliders, and their meshes if `look`. `point` (+1 or -1) makes the rib's end at
    z1 (+1) or z0 (-1) pointed, a 90-degree ridge across its width, as the flutes' rounded ends are: pushed end to end
    they slide off each other into the gaps."""
    cols, meshes = [], []
    for k in range(n):
        a = offset + 2 * math.pi * k / n
        rm = (r_in + r_out) / 2
        zz0, zz1 = (z0, z1 - width / 2) if point > 0 else ((z0 + width / 2, z1) if point < 0 else (z0, z1))
        c = (rm * math.cos(a), rm * math.sin(a), (zz0 + zz1) / 2)
        cols.append(kit.Collider(c, size=(r_out - r_in, width, zz1 - zz0), quat_wxyz=_qz_wxyz(a)))
        if point:
            zt = zz1 if point > 0 else zz0
            d = width / math.sqrt(2)
            q45 = (math.cos(math.pi / 8), math.sin(math.pi / 8), 0.0, 0.0)  # 45 degrees about the rib's radial axis
            cols.append(kit.Collider((rm * math.cos(a), rm * math.sin(a), zt), size=(r_out - r_in, d, d),
                                     quat_wxyz=_qmul(_qz_wxyz(a), q45)))
        if look is not None:
            m = kit.box((0, 0, 0), (r_out - r_in, width, z1 - z0), look)
            rot = _rot_z(a)
            pts = tuple(tuple(float(v) for v in rot @ np.asarray(p) + np.asarray(c)) for p in m.points)
            meshes.append(kit.Mesh(pts, m.counts, m.indices, m.look))
    return cols, meshes


def _filter_parts():
    """The filter in its own frame: origin on the axis at the gasket's face, z toward the closed end."""
    body_top = CAN_H - FLUTE_L
    meshes = [
        kit.lathe([(0.022, 0.0), (0.0285, 0.0), (0.0285, 0.004), (0.022, 0.004)], RUBBER),  # the gasket
        kit.lathe([(BORE_R, BORE_Z0), (0.031, BORE_Z0), (0.031, 0.006), (BORE_R, 0.006)], STEEL),  # the base plate
        kit.lathe([(BORE_R, BORE_Z0 + 0.001), (0.0125, BORE_Z0 + 0.001), (0.0125, BORE_Z1), (BORE_R, BORE_Z1)],
                  STEEL),  # its threaded bore
        kit.lathe([(CAN_R - 0.001, 0.005), (CAN_R, 0.007), (CAN_R, body_top), (FLUTE_IN, body_top + 0.002),
                   (FLUTE_IN, CAN_H - 0.002), (FLUTE_IN - 0.002, CAN_H), (0.0, CAN_H)], CAN),  # can, fluted end
        kit.lathe([(CAN_R + 0.0004, 0.020), (CAN_R + 0.0004, 0.045), (CAN_R - 0.002, 0.045), (CAN_R - 0.002, 0.020)],
                  WHITE),  # the printed band
    ]
    # an index stripe down the side (to read its angle in the video) and three quarter-turn ticks on the band
    meshes.append(kit.box((CAN_R + 0.0006, 0.0, (0.008 + body_top) / 2), (0.0012, 0.006, body_top - 0.008), WHITE))
    for k in range(1, 4):
        a = math.pi / 2 * k
        meshes.append(kit.box(((CAN_R + 0.0008) * math.cos(a), (CAN_R + 0.0008) * math.sin(a), 0.032),
                              (0.004, 0.004, 0.010), BLACK))
    flute_cols, flute_meshes = _ribs(FLUTE_IN - 0.0005, CAN_R, FLUTE_W, body_top, CAN_H - 0.0005, FLUTE_N, look=CAN,
                                     point=1)
    meshes += flute_meshes
    cols = (_ring(0.022, 0.0285, 0.0, 0.004, n=12)  # the gasket
            + _ring(BORE_R, 0.0125, BORE_Z0, BORE_Z1, n=36)  # the threaded collar, its bore at its thread's crests
            + _ring(0.0122, 0.031, BORE_Z0, 0.006, n=12)  # the base plate round it
            + [kit.Collider((0, 0, (0.0105 + body_top) / 2), radius=CAN_R, height=body_top - 0.0105),
               kit.Collider((0, 0, (body_top + CAN_H) / 2), radius=FLUTE_IN, height=FLUTE_L)]
            + flute_cols)
    return tuple(meshes), tuple(cols)


# The filter starts standing on its gasket on the bench, at the robot's left (as a new filter is set down).
FILTER_P0 = (0.300, 0.300, TOP + 0.001)  # on the bench, left of the wrench and clear of the crankcase
R_BASE = _rot_y(-90.0)  # the mount's frame: origin at the land's center, z out along the nipple (world -x), x up
_b = R_BASE.T @ (np.array(FILTER_P0) - np.array((X_LAND, Y_F, Z_F)))
FILTER_START = (float(_b[0]), float(_b[1]), float(_b[2]), math.radians(90.0), 0.0)  # standing: its axis up


def _mount():
    body = NIP_L - LEAD
    meshes = [kit.cylinder((0, 0, NIP_L / 2), NIP_ROOT_R, NIP_L, STEEL, n=32),
              kit.lathe([(NIP_ROOT_R, body), (NIP_CREST_R, body), (NIP_ROOT_R, NIP_L)], STEEL, n=32)]  # the chamfer
    z = THREAD.pitch / 2
    while z < body - 0.0003:  # the thread's crests, drawn
        meshes.append(kit.cylinder((0, 0, z), NIP_CREST_R, 0.0004, STEEL, n=32))
        z += THREAD.pitch
    cols = ()  # the nipple's collider is the engine's (a link of this articulation would not collide with it)
    fm, fc = _filter_parts()
    return fk.LooseFilter("mount", pos=(X_LAND, Y_F, Z_F), size=(0.08, 0.08, 0.08), mass=None, meshes=tuple(meshes),
                          colliders=cols, tilt_xyzw=_m_to_quat(R_BASE), thread=THREAD, filter_meshes=fm,
                          filter_colliders=fc, filter_mass=FILTER_MASS, start=FILTER_START,
                          reach=((-0.14, 0.12), (-0.12, 0.30), (-0.001, 0.26)))


def _engine():
    """The lower front of the crankcase with the filter's boss, the frame's cross tube, the intake duct and the drip
    pan: fixed, in the task frame."""
    (x0, x1), (y0, y1), (z0, z1) = CASE
    (px0, px1), (py0, py1), ph = PAN
    meshes = [kit.box(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), (x1 - x0, y1 - y0, z1 - z0), ALU),
              kit.cylinder((X_LAND + BOSS_L / 2, Y_F, Z_F), BOSS_R, BOSS_L, LAND, axis="x"),
              kit.cylinder((TUBE_X, sum(TUBE_Y) / 2, TUBE_Z), TUBE_R, TUBE_Y[1] - TUBE_Y[0], FRAME, axis="y"),
              kit.cylinder((DUCT_X, DUCT_Y, (DUCT_Z0 + z1) / 2), DUCT_R, z1 - DUCT_Z0, DUCT)]
    for k in range(6):  # bolt bosses along the case's top edge, drawn
        meshes.append(kit.cylinder((x0 - 0.003, y0 + 0.03 + k * 0.055, z1 - 0.03), 0.007, 0.006, ALU, axis="x"))
    for ty in TUBE_Y:  # the cross tube's two feet (the frame's lower rails, cut off)
        meshes.append(kit.box((TUBE_X, ty, (TOP + TUBE_Z) / 2), (0.03, 0.03, TUBE_Z - TOP), FRAME))
    meshes.append(kit.box(((px0 + px1) / 2, (py0 + py1) / 2, TOP + 0.0015), (px1 - px0, py1 - py0, 0.003), PAN_LOOK))
    rims = (((px0 + px1) / 2, py0, px1 - px0, 0.004), ((px0 + px1) / 2, py1, px1 - px0, 0.004),
            (px0, (py0 + py1) / 2, 0.004, py1 - py0), (px1, (py0 + py1) / 2, 0.004, py1 - py0))
    for cx, cy, sx, sy in rims:
        meshes.append(kit.box((cx, cy, TOP + ph / 2), (sx, sy, ph), PAN_LOOK))
    cols = [kit.Collider(((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2), size=(x1 - x0, y1 - y0, z1 - z0)),
            kit.Collider((X_LAND + BOSS_L / 2, Y_F, Z_F), radius=BOSS_R, height=BOSS_L, axis="X"),
            kit.Collider((X_LAND - NIP_L / 2, Y_F, Z_F), radius=NIP_ROOT_R, height=NIP_L, axis="X"),  # the nipple
            kit.Collider((TUBE_X, sum(TUBE_Y) / 2, TUBE_Z), radius=TUBE_R, height=TUBE_Y[1] - TUBE_Y[0], axis="Y"),
            kit.Collider((DUCT_X, DUCT_Y, (DUCT_Z0 + z1) / 2), radius=DUCT_R, height=z1 - DUCT_Z0),
            kit.Collider(((px0 + px1) / 2, (py0 + py1) / 2, TOP + 0.0015), size=(px1 - px0, py1 - py0, 0.003))]
    for cx, cy, sx, sy in rims:
        cols.append(kit.Collider((cx, cy, TOP + ph / 2), size=(sx, sy, ph)))
    return kit.Prop("engine", pos=(0.0, 0.0, 0.0), size=(2.0, 1.0, 0.70), mass=None, meshes=tuple(meshes),
                    colliders=tuple(cols))


# --- The wrench: a 65 mm, 14-flute, 3/8-inch-drive cap-type filter wrench on a 24-inch, 1/2-inch-drive ratchet,
# through a 1/2 to 3/8-inch adapter. A free body: the ratchet (handle and head) is the floating base; the drive (the
# adapter and cap) turns on it about the drive's axis, held by the pawl one way. Its frame: origin on the drive's axis
# at the cup's inside floor, z out of the cup's back (the cup opens toward -z), the handle along -x.
# Dimensions: P = the maker's published figure; E = estimated (not published; measured on the rig).
CUP_IN = 0.0334  # E: the cup's wall inside (radius), 0.4 mm over the filter's flute crests (P: 65 mm, 14 flutes)
CUP_R = CUP_IN + 0.003  # E: a 3 mm wall
CUP_DEPTH = 0.013  # E: the wall's depth (the filter's fluted end is 14 mm long)
TEETH_D, TEETH_IN, TEETH_W = 0.012, 0.0315, 0.004  # E: the cup's 14 flutes, 12 mm deep, in to 31.5 mm, 4 mm wide
CAP_END = 0.004  # E: the cup's end plate
ADAPTER_L, ADAPTER_R = 0.0351, 0.0095  # P: GearWrench 81354, 1.38 in (35.1 mm) long; E: 19 mm across
ADAPTER_IN_CAP = 0.0095  # E
RATCHET_L = 0.6106  # P: GearWrench 81364XP, 24.04 in overall
HEAD_W, HEAD_T = 0.0368, 0.0175  # P: its head 1.45 in wide, 0.69 in thick
HANDLE_R, GRIP_L = 0.011, 0.15  # E: the round grip about 22 mm thick on its last 15 cm
HEAD_BOT = CAP_END + ADAPTER_L - ADAPTER_IN_CAP
HEAD_TOP = HEAD_BOT + HEAD_T
HEAD_R = HEAD_W / 2
HANDLE_Z = HEAD_BOT + HEAD_T / 2
HANDLE_X = (-(RATCHET_L - HEAD_R), -HEAD_R)
RATCHET_KG, ADAPTER_KG, CAP_KG = 1.361, 0.027, 0.060  # P: 3.000 lb; P: 0.06 lb; E: the steel cap
TOOL_MASS = RATCHET_KG + ADAPTER_KG + CAP_KG
# the wrench lies flat on the bench, cup down, in front of the mount (clear of the left arm's way to it), its handle
# toward the robot's right
TOOL_P0 = (0.300, 0.140, TOP + CUP_DEPTH + 0.0015)
R_FLAT = _rot_z(math.radians(90.0))  # local x -> +y (the handle toward -y), local z up
R_MOUNT = _rot_y(-90.0) @ R_FLAT  # on the filter, handle level: local z -> -x (the cup opens toward the engine)


def _tool_parts():
    hl = HANDLE_X[1] - HANDLE_X[0]
    mid = (HANDLE_X[0] + HANDLE_X[1]) / 2
    base_meshes = [kit.cylinder((0, 0, HANDLE_Z), HEAD_R, HEAD_T, CHROME),
                   kit.cylinder((0.004, 0.0, HEAD_TOP + 0.0015), 0.006, 0.003, CHROME),  # the direction lever
                   kit.box((mid, 0.0, HANDLE_Z), (hl, 0.016, 0.012), CHROME),
                   kit.cylinder((HANDLE_X[0] + GRIP_L / 2, 0.0, HANDLE_Z), HANDLE_R, GRIP_L, CHROME, axis="x")]
    base_cols = (kit.Collider((0, 0, HANDLE_Z), radius=HEAD_R, height=HEAD_T),
                 kit.Collider(((HANDLE_X[1] + HANDLE_X[0] + GRIP_L) / 2, 0.0, HANDLE_Z),
                              size=(hl - GRIP_L, 0.016, 0.012)),
                 kit.Collider((HANDLE_X[0] + GRIP_L / 2, 0.0, HANDLE_Z), radius=HANDLE_R, height=GRIP_L, axis="X"))
    cap_meshes = [kit.lathe([(CUP_IN, -CUP_DEPTH), (CUP_R, -CUP_DEPTH), (CUP_R, CAP_END), (0.0, CAP_END), (0.0, 0.0),
                             (CUP_IN, 0.0)], CAP_LOOK, n=28),
                  kit.cylinder((0, 0, (CAP_END + HEAD_BOT) / 2), ADAPTER_R, HEAD_BOT - CAP_END, CHROME)]
    teeth, teeth_meshes = _ribs(TEETH_IN, CUP_IN + 0.0008, TEETH_W, -TEETH_D, 0.0, FLUTE_N, offset=math.pi / FLUTE_N,
                                look=CAP_LOOK, point=-1)
    cap_meshes += teeth_meshes
    cap_cols = tuple([kit.Collider((0, 0, CAP_END / 2), radius=CUP_R, height=CAP_END),
                      kit.Collider((0, 0, (CAP_END + HEAD_BOT) / 2), radius=ADAPTER_R, height=HEAD_BOT - CAP_END)]
                     + _ring(CUP_IN, CUP_R, -CUP_DEPTH, 0.0, n=16) + teeth)
    return tuple(base_meshes), base_cols, tuple(cap_meshes), cap_cols


def _tool():
    bm, bc, cm, cc = _tool_parts()
    cap = kit.Link("cap", "revolute", "Z", (0.0, 0.0, 0.0), lower=-60.0, upper=60.0, start=0.0,
                   mass=ADAPTER_KG + CAP_KG, meshes=cm, colliders=cc, damping=0.0, law=fk._Free())
    return kit.Mechanism("wrench", pos=TOOL_P0, size=(0.66, 0.08, 0.08), mass=RATCHET_KG, meshes=bm, colliders=bc,
                         tilt_xyzw=_m_to_quat(R_FLAT), links=(cap,))


MOUNT = _mount()
OBJECTS = [_engine(), MOUNT, _tool()]
fk.register_filter(MOUNT)
fk.floating("wrench")
fk.register_ratchet("wrench", "cap")
KEY = "mount/filter"
TOOL, CAP = "wrench", "wrench/cap"
GRIPPERS = ("left_gripper", "right_gripper")
FILTER_SHAPE = gk.Ring(0.0, CAN_R, 0.0, CAN_H)  # in the filter's frame


def fmodel() -> fk.FilterModel:
    return fk.filter_model("mount")


def past_deg() -> float:
    return math.degrees(fmodel().past)


# --- Reading the scene -------------------------------------------------------------------------------------------
def _pose(objects, name):
    s = objects[name]
    return np.asarray(s.pos, float), _quat_to_m(np.asarray(s.quat_xyzw, float))


def filter_axis(objects):
    """(gasket face center, axis toward the closed end) of the filter, world."""
    p, r = _pose(objects, KEY)
    return p, r[:, 2]


def cap_seated(objects) -> bool:
    """The cap down over the filter's fluted end: its axis within 3 mm and 5 degrees of the filter's, its floor
    within 4 mm of the filter's end."""
    if CAP not in objects or KEY not in objects:
        return False
    fp, fa = filter_axis(objects)
    cp, cr = _pose(objects, CAP)
    off = cp - (fp + fa * CAN_H)
    along = float(np.dot(off, fa))
    lateral = off - along * fa
    return (float(np.linalg.norm(lateral)) < 0.003 and abs(along) < 0.004
            and float(np.dot(cr[:, 2], fa)) > math.cos(math.radians(5.0)))


def tool_holders(objects) -> set:
    """Grippers holding the wrench: fingers under 60 % open, tool point within 3 cm of the handle's axis."""
    if TOOL not in objects:
        return set()
    p, r = _pose(objects, TOOL)
    a = p + r @ np.array((HANDLE_X[0], 0.0, HANDLE_Z))
    b = p + r @ np.array((HANDLE_X[1], 0.0, HANDLE_Z))
    out = set()
    for g in GRIPPERS:
        if g not in objects or kit.opening(objects, g) >= 0.6:
            continue
        q = np.asarray(objects[g].pos, float)
        u = float(np.clip(np.dot(q - a, b - a) / np.dot(b - a, b - a), 0.0, 1.0))
        if np.linalg.norm(q - (a + u * (b - a))) < 0.03:
            out.add(g)
    return out


HOLD_TIGHT, HOLD_LOOSE = 0.15, 0.03


def filter_holders(objects) -> set:
    """Grippers closed on the can: the opening stopped by its 66 mm (from 15 % of full closed below to 3 % above),
    the tool point within 1.5 cm of its axis, from 1 cm above its gasket face to 1.5 cm past its end."""
    from yam_arena.constants import FINGER_OPEN
    if KEY not in objects:
        return set()
    fp, fa = filter_axis(objects)
    w = 2 * CAN_R / (2 * abs(FINGER_OPEN))
    out = set()
    for g in GRIPPERS:
        if g not in objects:
            continue
        op = kit.opening(objects, g)
        if not (w - HOLD_TIGHT <= op <= w + HOLD_LOOSE):
            continue
        d = np.asarray(objects[g].pos, float) - fp
        along = float(np.dot(d, fa))
        if 0.010 <= along <= CAN_H + 0.015 and np.linalg.norm(d - along * fa) < 0.015:
            out.add(g)
    return out


def _twist(q0, q1, axis):
    """The rotation from orientation q0 to q1 (x, y, z, w) about `axis` (world, unit), in degrees."""
    from yam_arena.ik import matrix_to_quat_xyzw
    q = matrix_to_quat_xyzw(_quat_to_m(q1) @ _quat_to_m(q0).T)
    s = float(np.dot(q[:3], axis))
    a = math.degrees(2 * math.atan2(s, q[3])) if (s or q[3]) else 0.0
    return (a + 180.0) % 360.0 - 180.0


# --- What happened during the run ---------------------------------------------------------------------------------
class _History:
    def __init__(self):
        self.episode = kit.episode()
        self.high = 0.0  # the furthest past contact so far (degrees, from 0): only new ground counts as turning
        self.prev_holders = set()
        self.last_quat = {}
        self.budget = {g: 0.0 for g in GRIPPERS}  # each holding gripper's wrist roll not yet matched by the filter
        self.unheld = 0.0  # new ground past contact gained with neither a holding hand nor the held wrench (degrees)


_H = _History()
_SEEN = {}


def _update(objects):
    global _H
    if _H.episode != kit.episode():
        _H = _History()
    fm = fmodel()
    now = filter_holders(objects)
    holders = now | _H.prev_holders
    quats = {g: np.asarray(objects[g].quat_xyzw, float) for g in GRIPPERS if g in objects}
    tighten = -filter_axis(objects)[1] if KEY in objects else np.array((1.0, 0.0, 0.0))  # clockwise from its end
    for g in GRIPPERS:
        if g in holders and g in quats and g in _H.last_quat:
            _H.budget[g] = min(_H.budget[g] + _twist(_H.last_quat[g], quats[g], tighten), CARRY)
        elif g not in holders:
            _H.budget[g] = 0.0
    if fm.state == fk.ENGAGED:
        past = math.degrees(fm.past)
        gain = max(past - _H.high, 0.0)
        if gain > 0:
            if cap_seated(objects) and tool_holders(objects):
                credit = gain  # turned by the wrench, held
            else:
                credit = min(gain, max([_H.budget[g] for g in holders] + [0.0]))
                for g in holders:
                    _H.budget[g] = max(_H.budget[g] - credit, 0.0)
            _H.unheld += gain - credit
            _H.high = past
    _H.prev_holders = now
    _H.last_quat = quats


def _hand_on_filter(objects):
    return bool(filter_holders(objects) or gk.touching(objects, KEY, FILTER_SHAPE))


def _score(objects):
    fm = fmodel()
    if fm.state == fk.CROSSED or fm.came_off or fm.state != fk.ENGAGED:
        return 0.0  # cross-threaded, unscrewed off after it was started, or not on the thread
    past, peak = math.degrees(fm.past), math.degrees(fm.peak_past)
    if peak > CRUSH or past < LOOSE or _H.unheld > GRIP_SLACK:
        return 0.0
    if past < SNUG:
        s = 0.25
    elif past < GOOD_LO or peak > GOOD_HI:
        s = 0.5
    else:
        s = 1.0
    if _hand_on_filter(objects) or cap_seated(objects) or tool_holders(objects):
        return min(s, 0.5)  # a hand still on the filter, the wrench still on it or still in a hand
    return s


def success(objects):
    """Full score (see score()), held 1 s. Called first each step, so it also records the history."""
    _update(objects)
    _SEEN["objects"] = objects
    return _score(objects) == 1.0


def score(objects):
    """Angles are degrees past gasket contact. 0 if the filter was cross-threaded (its thread started with its axis
    more than 4 degrees off the nipple's), was unscrewed off the thread after it was started, is not on the thread,
    was ever more than 450 degrees past contact, ends under 90 past contact, or if more than 45 degrees of the new
    ground it gained past contact was turned neither by a holding hand's own wrist roll (a gripper closed on the can,
    its roll about the filter's axis, at most 30 degrees ahead) nor by the wrench while its cap was on the filter and
    a hand held it. Otherwise 0.25 for 90 to 180; 0.5 for 180 to 240, or if it was ever past 360; 1.0 for 240 to 360
    and never past 360. Capped at 0.5 while a hand is on the filter (closed on it, or touching it within 2 mm), and
    while the wrench's cap is on the filter or a hand holds the wrench. Possible scores: 0, 0.25, 0.5, 1."""
    return _score(objects)


# --- Demo ----------------------------------------------------------------------------------------------------------
def _obj():
    return _SEEN.get("objects")


def _filter_now():
    """(gasket face center, axis to the closed end, rotation) of the filter seen at the last step."""
    o = _obj()
    if o is not None and KEY in o:
        p, r = _pose(o, KEY)
        return p, r[:, 2], r
    if "filter" in _CMD:  # offline checks: a filter on the nipple
        return _CMD["filter"]
    return np.array(FILTER_P0), np.array((0.0, 0.0, 1.0)), np.eye(3)


def _tool_now():
    """(origin, rotation) of the wrench and the cap's rotation, seen at the last step."""
    o = _obj()
    if o is not None and TOOL in o and CAP in o:
        p, r = _pose(o, TOOL)
        return p, r, _pose(o, CAP)[1]
    if "tool" in _CMD:  # offline (no simulator): where the demo last put it
        p, r = _CMD["tool"]
        return np.asarray(p), np.asarray(r), np.asarray(r)
    return np.array(TOOL_P0), R_FLAT.copy(), R_FLAT.copy()


_CMD = {}  # offline checks only: the wrench pose the demo last commanded (read when the simulator reports nothing)


class _Live:
    """A demo waypoint (kit.Posed fields) whose (pos, rot, finger) is fn(), worked out once, when the arm starts on
    it; keep() false makes it take no time (a step the demo skips)."""

    def __init__(self, duration, fn, label, keep=None):
        self._d, self._fn, self.label, self._keep, self._v = duration, fn, label, keep, None
        self.yaw = 0.0

    @property
    def duration(self):
        return self._d if self._keep is None or self._keep() else 0.0

    def _get(self):
        if self._v is None:
            pos, rot, finger = self._fn()
            self._v = (tuple(float(v) for v in pos), np.asarray(rot, float), float(finger))
        return self._v

    @property
    def pos(self):
        return self._get()[0]

    @property
    def rot(self):
        return self._get()[1]

    @property
    def finger(self):
        return self._get()[2]


class _Wait(kit.Home):
    """The right arm waits at home for as long as `fn()` says."""

    def __init__(self, fn, label):
        from yam_arena.constants import FINGER_OPEN
        self._fn, self.finger, self.label, self.pos, self.yaw = fn, FINGER_OPEN, label, (0.0, 0.0, 0.0), 0.0

    @property
    def duration(self):
        return self._fn()


def _td(yaw):
    import yam_arena.demo as yd
    return yd.grip_rotation(yaw)


def _squeeze(width_m, squeeze_m):
    """The finger target that closes the pads to `width - squeeze` (gripkit.squeeze_target): about 2.5 N per mm of
    squeeze on each finger, not the full-close 50 N that drives the pads into a wide part."""
    from yam_arena.constants import FINGER_OPEN
    return max(FINGER_OPEN, -max(0.0, width_m - squeeze_m) / 2)


F_CAN = _squeeze(2 * CAN_R, 0.008)  # about 20 N a finger on the can
F_GRIP = _squeeze(2 * HANDLE_R, 0.012)  # about 30 N a finger on the round grip
F_BEAM = _squeeze(HEAD_W, 0.008)  # about 20 N a finger across the ratchet's 36.8 mm head

_GIVE = {}


def _give(side, tcp, rot):
    """The arm's give at its pad tips (m/N, world): J K^-1 J^T with the YAM's servo stiffness."""
    key = (side, *(round(float(v), 3) for v in tcp), *(round(float(v), 2) for v in np.ravel(rot)))
    if key not in _GIVE:
        import yam_arena.demo as yd
        from yam_arena.constants import PAD_TIP_PAST_TCP, SHOULDER, WRIST
        from yam_arena.station import BASES
        base = np.asarray(BASES[side], float)
        q, _, _ = yd.ik.solve(np.asarray(tcp, float), yd.ik.matrix_to_quat_xyzw(rot), q_init=None, base_pos=base)

        def tip(q):
            p, rm = yd.ik.tcp_pose(q, base)
            return np.asarray(p) + PAD_TIP_PAST_TCP * np.asarray(rm)[:, 2]

        jac = np.zeros((3, 6))
        for i in range(6):
            dq = np.zeros(6)
            dq[i] = 1e-6
            jac[:, i] = (tip(q + dq) - tip(q - dq)) / 2e-6
        kinv = np.diag([1 / SHOULDER["stiffness"]] * 3 + [1 / WRIST["stiffness"]] * 3)
        _GIVE[key] = jac @ kinv @ jac.T
    return _GIVE[key]


# -- the left hand: pick the filter up, start it square on the thread, spin it on until hand-tight ------------------
PICK_YAW = 90.0  # the top-down pick's yaw (degrees): the fingers close along x, so turning the filter over
#                 (about y) cannot pivot it between the pads
GRASP_UP = CAN_H - 0.025 + 0.013  # the tool point this far along the axis from the gasket face (pads 2.5 cm in from
#                                   the closed end)
SPIN_FROM, SPIN_TO, SPIN_STEP = -60.0, 90.0, 15.0  # each stroke rolls the wrist from -60 to +90 about +x (tightening)
HAND_STALL = 8.0  # a stroke past contact that turns it less than this: hand-tight
SPIN_MAX = 12
CARRY_X = 0.38  # the filter is carried across here, clear of the crankcase, the cross tube and the wrench
PRESS = 0.0015  # the hand aims 1.5 mm past where the threads' crests meet (the arm's give presses it on)
OUT_NEAR = NIP_L - BORE_Z0 + 0.0015  # lining up closely: the bore's entry 1.5 mm short of the nipple's end
FINE_MM, FINE_DEG = 0.06, 0.4  # lined up: within 0.06 mm of the axis (the bore's clearance is 0.2) and 0.4 degrees
FINE_LOOKS = 3
PRESS_LOOKS = 10  # pressed on the nipple's end, re-aimed each second until the bore drops over it


def _horiz(roll_deg):
    """The left hand pointing at the engine (+x), rolled `roll_deg` about +x."""
    return _rot_x(math.radians(roll_deg)) @ _rot_y(-90.0) @ _td(0.0)


def _tcp_for_gasket(xg, fix=(0.0, 0.0)):
    """The left tool point that puts the filter's gasket face at x `xg` on the nipple's axis."""
    return np.array((xg - GRASP_UP, Y_F + fix[0], Z_F + fix[1]))


def _hand_phase(st):
    from yam_arena.constants import FINGER_OPEN
    x0, y0 = FILTER_P0[0], FILTER_P0[1]
    zg = TOP + GRASP_UP
    down = _td(math.radians(PICK_YAW))
    r_pick = _horiz(-PICK_YAW)
    out = [_Live(1.6, lambda: ((x0, y0, zg + 0.06), down, FINGER_OPEN), "filter: over it"),
           _Live(1.2, lambda: ((x0, y0, zg), down, FINGER_OPEN), "filter: around it"),
           _Live(0.8, lambda: ((x0, y0, zg), down, F_CAN), "filter: grip"),
           _Live(1.2, lambda: ((x0, y0, zg + 0.085), down, F_CAN), "filter: lift"),
           _Live(1.0, lambda: ((CARRY_X, y0, zg + 0.085), down, F_CAN), "filter: out from the engine"),
           _Live(1.6, lambda: ((CARRY_X, Y_F, zg + 0.07), down, F_CAN), "filter: across to the nipple"),
           _Live(2.0, lambda: (_tcp_for_gasket(X_LAND - D0 - 0.050), r_pick, F_CAN),
                 "filter: turn it to the nipple"),
           _Live(1.2, lambda: (_tcp_for_gasket(X_LAND - D0 - 0.010), r_pick, F_CAN), "filter: in line with the nipple")]
    st["fix"] = [0.0, 0.0]
    st["rfix"] = np.eye(3)

    def align(out, gain=0.7, clip=0.015):
        """Where is the bore's entry, and which way does the filter point? Aim it onto the nipple's axis, its gasket
        face `out` from the land."""
        def f():
            from yam_arena.urdf import axis_angle_matrix
            p, a, _ = _filter_now()
            b = p + BORE_Z0 * np.asarray(a, float)  # the bore's entry, where the nipple's end goes in
            # (the arm sags 2-3 cm under the filter in this pose: the aim takes it up, at most 4 cm)
            st["fix"][0] = float(np.clip(st["fix"][0] - gain * np.clip(b[1] - Y_F, -clip, clip), -0.04, 0.04))
            st["fix"][1] = float(np.clip(st["fix"][1] - gain * np.clip(b[2] - Z_F, -clip, clip), -0.04, 0.04))
            want, have = np.array((-1.0, 0.0, 0.0)), np.asarray(a, float)  # its axis to the closed end: -x
            c = np.cross(have, want)
            s = float(np.linalg.norm(c))
            if s > 1e-6:
                st["rfix"] = axis_angle_matrix(c / s, 0.8 * math.atan2(s, float(np.dot(have, want)))) @ st["rfix"]
            return _tcp_for_gasket(X_LAND - out, st["fix"]), st["rfix"] @ r_pick, F_CAN
        return f

    def off_axis():
        """(mm off the axis at the bore's entry, degrees of tilt) seen now."""
        p, a, _ = _filter_now()
        b = p + BORE_Z0 * np.asarray(a, float)
        tilt = math.degrees(math.acos(float(np.clip(-a[0], -1.0, 1.0))))
        return 1000 * math.hypot(b[1] - Y_F, b[2] - Z_F), tilt

    def fine(label, gate=lambda: True):
        """Up to FINE_LOOKS close looks, each a correction and then a hold while the arm settles (it lags its target
        by most of a second). Each one, when it starts, checks whether the last brought the bore's entry within
        FINE_MM of the axis and FINE_DEG of square; once one has, the rest are skipped (decided as each starts, so a
        look already made keeps its time)."""
        g = {"done": None}
        out_ = []

        def look(i):
            def f():
                e, t = off_axis()
                if g["done"] is None and e <= FINE_MM and t <= FINE_DEG:
                    g["done"] = i
                g["last"] = align(OUT_NEAR, 0.7, 0.002)()
                return g["last"]
            return f
        for i in range(FINE_LOOKS):
            keep = (lambda i=i: gate() and (g["done"] is None or i <= g["done"]))
            out_.append(_Live(0.6, look(i), label, keep))
            out_.append(_Live(1.0, lambda: g["last"], label + " (settle)", keep))
        return out_

    def onto():
        return _tcp_for_gasket(X_LAND - D0 + PRESS, st["fix"]), st["rfix"] @ r_pick, F_CAN

    def is_on():
        return fmodel().on or fmodel().state != fk.FREE

    def press(label, gate=lambda: True):
        """Pushed on, the base plate against the nipple's end if the bore is not yet over it: up to PRESS_LOOKS
        looks a second apart, each reading where the bore's entry is and re-aiming it (gently: the plate is held
        on the end's face by the push), as a hand feels for the start; once the bore has gone over the nipple's end
        the rest are skipped (decided as each starts)."""
        g = {"done": None}

        def look(i):
            def f():
                if g["done"] is None and is_on():
                    g["done"] = i
                align(OUT_NEAR, 0.5, 0.001)()
                return onto()
            return f
        return [_Live(1.6 if i == 0 else 1.0, look(i), label,
                      lambda i=i: gate() and (g["done"] is None or i <= g["done"])) for i in range(PRESS_LOOKS)]

    def off_nipple():  # not on it (the base plate met the nipple's end): back off and look again
        return _tcp_for_gasket(X_LAND - OUT_NEAR - 0.003, st["fix"]), st["rfix"] @ r_pick, F_CAN

    for _ in range(5):  # coarse: 8 mm out
        out.append(_Live(0.6, align(D0 + 0.008), "filter: line it up"))
    out += fine("filter: line it up closely")  # the bore's entry 1.5 mm short of the nipple's end
    out += press("filter: onto the nipple")

    def miss():  # not on the nipple (the base plate met the nipple's end); once on it, never again (latched)
        if is_on():
            st["was_on"] = True
        return not st.get("was_on", False)

    for _ in range(2):  # at most two more tries
        out.append(_Live(0.8, off_nipple, "filter: missed the nipple, back off", miss))
        out += fine("filter: line it up again", miss)
        out += press("filter: onto the nipple again", miss)

    def xg_now(extra=0.0):
        fm = fmodel()
        t = (fm.turned if fm.state == fk.ENGAGED else 0.0) + math.radians(extra)
        return X_LAND - max(D0 - THREAD.pitch * t / (2 * math.pi), -0.0004) + PRESS

    def runs(i):
        return lambda: st["stop"] is None or st["stop"] >= i

    n_back = int((SPIN_TO - SPIN_FROM) / SPIN_STEP)
    for i in range(SPIN_MAX):
        r0 = -PICK_YAW if i == 0 else SPIN_FROM
        steps = int((SPIN_TO - r0) / SPIN_STEP)

        def grip(r0=r0):
            st["p0"] = past_deg() if fmodel().state == fk.ENGAGED else -1e9
            return _tcp_for_gasket(xg_now(), st["fix"]), st["rfix"] @ _horiz(r0), F_CAN
        out.append(_Live(0.5, grip, "spin it on: grip", runs(i)))
        for k in range(1, steps + 1):
            out.append(_Live(0.12, lambda k=k, r0=r0: (_tcp_for_gasket(xg_now(SPIN_STEP), st["fix"]),
                                                       st["rfix"] @ _horiz(r0 + SPIN_STEP * k), F_CAN),
                             "spin it on: roll", runs(i)))
        out.append(_Live(0.4, lambda: (_tcp_for_gasket(xg_now(), st["fix"]) + (-0.003, 0, 0),
                                       st["rfix"] @ _horiz(SPIN_TO), FINGER_OPEN), "spin it on: let go", runs(i)))

        def look(i=i):  # how far did that stroke turn it? (a skipped step's target is read too: act only once)
            if st["stop"] is None:
                fm = fmodel()
                if fm.state != fk.ENGAGED:  # cross-threaded, or the thread never caught: no use spinning on
                    st["stop"] = i
                elif fm.state == fk.ENGAGED and fm.past > 0:
                    st["gains"].append(past_deg() - max(st["p0"], 0.0))
                    if st["gains"][-1] < HAND_STALL:
                        st["stop"] = i
                if i == SPIN_MAX - 1:
                    st["stop"] = i
            return _tcp_for_gasket(xg_now(), st["fix"]) + (-0.07, 0, 0.02), st["rfix"] @ _horiz(SPIN_TO), FINGER_OPEN
        out.append(_Live(1.0, look, "hand-tight: back off", lambda i=i: st["stop"] == i))
        for k in range(1, n_back + 1):
            out.append(_Live(0.06, lambda k=k: (_tcp_for_gasket(xg_now(), st["fix"]) + (-0.003, 0, 0),
                                                st["rfix"] @ _horiz(SPIN_TO - SPIN_STEP * k), FINGER_OPEN),
                             "spin it on: roll back", lambda i=i: st["stop"] is None or st["stop"] > i))
    return out


# -- both hands: the wrench ----------------------------------------------------------------------------------------
R_L, R_R = 0.0, 0.55  # where each hand holds the wrench, from the drive's axis (m): the left on the ratchet's head
TD_YAW = -math.pi / 2  # top-down grips with the fingers closing along x, across a handle that runs along y
PAD_BELOW = 0.006  # the pad tips this far below the handle's axis: the fingers close round it
PAD_BELOW_GRIP = 0.011  # on the round grip, which lies on the bench: the pads reach its middle, tips at the bench
STROKE_UP, STROKE_DOWN = 6.0, -10.0  # each stroke swings the handle from 6 degrees above level to 10 below
SEAT_BAND = (-1.0, 3.5)  # the handle's angle while the cup goes on (the right hand's reach with the handle carried)
SHARE_R = 1.0  # the right hand swings the handle; the left, on the head, keeps the cap pressed on
MARGIN = 1.0  # N*m over what the gasket takes
LEFT_PRESS = 8.0  # N: the left hand presses the cap onto the filter (toward the engine) while the right pushes
DONE_PAST, AIM_PAST = 270.0, 282.0
WRENCH_MAX = 22
WRENCH_STALL = 2.0
TURN_STEPS = 4
LIFT = 0.07  # the wrench lifted this far before it is turned (the left arm's reach)
TAKE_UP = 4.0  # degrees of the stroke taken up before the filter turns (the flutes' play and the hold's give)
W_SHARE = {"left": 0.5, "right": 0.5}  # each hand's share of the wrench's weight while carrying it


def _hand_on_tool(side, tool_p, tool_r, force=(0.0, 0.0, 0.0), top_down=False):
    """(tcp, rot) of a hand on the handle of a wrench at (tool_p, tool_r), aimed past the pads by the arm's give so
    that they push `force` (N, world) on it. The grip is the one taken on the flat wrench, turned with it; or
    (`top_down`) a top-down grip on the handle wherever it is."""
    from yam_arena.constants import PAD_TIP_PAST_TCP
    r = R_L if side == "left" else R_R
    h = tool_p + tool_r @ np.array((-r, 0.0, HANDLE_Z))
    if top_down:
        tcp = h + np.array((0.0, 0.0, PAD_TIP_PAST_TCP - PAD_BELOW))
        hd = tool_r @ np.array((-1.0, 0.0, 0.0))  # the handle's direction from the drive
        phi = math.atan2(hd[2], -hd[1])  # its angle above level, toward -y
        rot = _rot_x(-phi) @ _td(TD_YAW)
    else:
        below = PAD_BELOW_GRIP if side == "right" else PAD_BELOW
        tcp = tool_p + tool_r @ np.array((-r, 0.0, HANDLE_Z + PAD_TIP_PAST_TCP - below))
        rot = tool_r @ R_FLAT.T @ _td(TD_YAW)
    f = np.asarray(force, float)
    if f.any():
        tcp = tcp + _give(side, tuple(tcp), rot) @ f
    return tcp, rot


def _tool_pose_on(angle_deg, out=0.0, fix=(0.0, 0.0, 0.0)):
    """The wrench's (origin, rotation) on the filter's axis: its handle `angle_deg` above level (toward -y), its cup
    floor `out` m off the filter's end."""
    p, a, _ = _filter_now()
    end = p + a * CAN_H
    r = _rot_x(-math.radians(angle_deg)) @ R_MOUNT  # raising the handle turns it the loosening way about +x
    return end + a * (out + 0.0005) + np.asarray(fix, float), r


def _align_angle(st, near=STROKE_UP):
    """A handle angle near `near` that puts the cap's teeth in the gaps between the filter's flutes."""
    _, rt, rc = _tool_now()
    _, _, rf = _filter_now()
    rel = rt.T @ rc  # the cap on the handle, about the drive
    c = math.atan2(rel[1, 0], rel[0, 0])
    m = rf.T @ R_MOUNT  # the mounted wrench's frame (handle level) in the filter's
    g = math.atan2(m[1, 0], m[0, 0])
    pitch = 2 * math.pi / FLUTE_N
    best = None
    for k in range(-8, 9):
        ang = math.degrees(-g - c + k * pitch)
        if ang <= near and (best is None or ang > best):  # the highest at or below `near`
            best = ang
    lo, hi = SEAT_BAND
    if not lo <= best <= hi:  # out of reach: go on at mid-band, and let the flutes' pointed ends turn the cap in
        best = (lo + hi) / 2
    st["align"] = best
    return best


def _wrench_phase(st):
    from yam_arena.constants import FINGER_OPEN
    steps = []

    def add(label, dur, spec, keep=None):
        steps.append((label, dur, spec, keep))

    def fgrip(s):
        return F_BEAM if s == "left" else F_GRIP

    def at_tool(finger, up=0.0):
        def f():
            p, r, _ = _tool_now()
            return {s: ((_hand_on_tool(s, p, r)[0] + (0, 0, up)), _hand_on_tool(s, p, r)[1], finger(s))
                    for s in ("left", "right")}
        return f

    add("wrench: over the handle", 2.0, at_tool(lambda s: FINGER_OPEN, 0.06))
    add("wrench: onto the handle", 1.4, at_tool(lambda s: FINGER_OPEN))
    add("wrench: onto the handle", 0.3, at_tool(lambda s: FINGER_OPEN))
    add("wrench: grip the handle", 0.8, at_tool(fgrip))

    def check():  # is each hand on it? (read once, when this step starts)
        o = _obj()
        held = tool_holders(o) if o is not None else {"left_gripper", "right_gripper"}
        st["regrip"] = [g.split("_")[0] for g in ("left_gripper", "right_gripper") if g not in held]
        return at_tool(fgrip)()
    add("wrench: check the grip", 0.1, check)

    def again(finger, dz):
        def f():
            p, r, _ = _tool_now()
            return {s_: ((_hand_on_tool(s_, p, r)[0] + (0, 0, dz)) if s_ in st.get("regrip", []) else
                         _hand_on_tool(s_, p, r)[0], _hand_on_tool(s_, p, r)[1],
                         (finger if finger is not None else fgrip(s_)) if s_ in st.get("regrip", []) else fgrip(s_))
                    for s_ in ("left", "right")}
        return f
    missed = (lambda: bool(st.get("regrip", True)))
    add("wrench: open the hand that missed", 0.5, again(FINGER_OPEN, 0.01), missed)
    add("wrench: lower", 0.6, again(FINGER_OPEN, -0.008), missed)
    add("wrench: grip again", 0.8, again(None, -0.008), missed)

    def carried(p, r):
        _CMD["tool"] = (p, r)
        return {s: (*_hand_on_tool(s, p, r, (0.0, 0.0, W_SHARE[s] * TOOL_MASS * 9.81)), fgrip(s))
                for s in ("left", "right")}

    def lifted():
        p, r, _ = _tool_now()
        st["held0"] = (p + np.array((0.0, 0.0, LIFT)), r)
        return carried(*st["held0"])
    add("wrench: lift", 1.4, lifted)
    for k in range(1, TURN_STEPS + 1):  # a quarter turn about the handle: the cup from facing down to the engine
        def turn(k=k):
            p0, r0 = st["held0"]
            pivot = p0 + r0 @ np.array((0.0, 0.0, HANDLE_Z))
            rr = _rot_y(-90.0 * k / TURN_STEPS)
            shift = np.array((0.0, (Y_F - p0[1]) * k / TURN_STEPS, 0.0))  # and over to the filter's side
            return carried(pivot + rr @ (p0 - pivot) + shift, rr @ r0)
        add("wrench: turn the cup to the filter", 0.8, turn)

    def on(out, realign):
        def f():
            ang = _align_angle(st, SEAT_BAND[1]) if realign or "align" not in st else st["align"]
            return carried(*_tool_pose_on(ang, out, st.get("tfix", np.zeros(3))))
        return f
    add("wrench: the cup in front of the filter", 2.2, on(0.03, True))
    st["tfix"] = np.zeros(3)
    for _ in range(2):
        def center():  # where is the cap? aim it onto the filter's axis
            fp, fa, _ = _filter_now()
            o = _obj()
            if o is not None and CAP in o:
                off = _pose(o, CAP)[0] - (fp + fa * (CAN_H + 0.0305))
                st["tfix"] = st["tfix"] - 0.8 * np.clip(off - np.dot(off, fa) * fa, -0.01, 0.01)
            return on(0.03, True)()
        add("wrench: line the cap up", 0.8, center)
    add("wrench: cup over the flutes", 1.4, on(-0.002, False))
    add("wrench: seated", 0.6, on(-0.002, False))

    def regrip(finger, up):
        def f():
            p, r, _ = _tool_now()
            rt, rr = _hand_on_tool("right", p, r, top_down=True)
            return {"left": (*_hand_on_tool("left", p, r, (LEFT_PRESS, 0, 0)), F_BEAM),
                    "right": (rt + (0, 0, up), rr, finger)}
        return f
    add("wrench: right hand off", 0.7, lambda: {"left": (*_hand_on_tool("left", *_tool_now()[:2]), F_BEAM),
                                                 "right": (*_hand_on_tool("right", *_tool_now()[:2]), FINGER_OPEN)})
    add("wrench: right hand over the grip", 1.6, regrip(FINGER_OPEN, 0.035))
    add("wrench: right hand onto the grip", 1.0, regrip(FINGER_OPEN, 0.0))
    add("wrench: right hand grips", 0.8, regrip(F_GRIP, 0.0))

    def pose_at(ang):
        return _tool_pose_on(ang, -0.002)

    def hands(ang, tau, press=LEFT_PRESS):
        pt, rt = pose_at(ang)
        _CMD["tool"] = (pt, rt)
        a = math.radians(ang)
        tang = np.array((0.0, -math.sin(a), -math.cos(a)))  # down the swing at the handle (tightening)
        f_r = SHARE_R * tau / R_R * tang
        f_l = np.array((press, 0.0, 0.0))
        return {"left": (*_hand_on_tool("left", pt, rt, f_l), F_BEAM),
                "right": (*_hand_on_tool("right", pt, rt, f_r, top_down=True), F_GRIP)}

    sub = 5
    for n in range(WRENCH_MAX):
        runs = (lambda n=n: st["wstop"] is None or st["wstop"] >= n)

        def plan():  # latched at the stroke's start, from where the filter and the handle are
            if st["wstop"] is not None:
                return
            st["p"] = past_deg()
            _, r_tool, _ = _tool_now()
            hd = r_tool @ np.array((-1.0, 0.0, 0.0))
            st["a0"] = math.degrees(math.atan2(hd[2], -hd[1]))
            st["len"] = max(min(st["a0"] - STROKE_DOWN, max(AIM_PAST - st["p"], 0.0) + TAKE_UP), 2.0)

        for k in range(1, sub + 1):
            def spec(k=k):
                if k == 1:
                    plan()
                f = st["len"] * k / sub
                tau = THREAD.torque(math.radians(max(st["p"], 0.0) + max(f - TAKE_UP, 0.0))) + MARGIN
                return hands(st["a0"] - f, tau)
            add(f"stroke {n + 1}", 0.25, spec, runs)
        add(f"stroke {n + 1}: ease off", 0.2, lambda: hands(st["a0"] - st["len"], 0.0), runs)

        def back(n=n):  # tight now? then no more strokes; otherwise swing the handle back up (the ratchet clicks)
            if st["wstop"] is None:
                st["stalls"] = st.get("stalls", 0) + 1 if past_deg() - st["p"] < WRENCH_STALL else 0
                if past_deg() >= DONE_PAST or st["stalls"] >= 2 or n == WRENCH_MAX - 1:
                    st["wstop"] = n
            return hands(STROKE_UP if st["wstop"] is None else st["a0"] - st["len"], 0.0)
        add(f"stroke {n + 1}: ratchet back", 0.7, back, lambda n=n: st["wstop"] is None or st["wstop"] > n)

    # back up to where the right hand can hold the handle as it carried it, and move it back to that grip
    def back_up(right, dx=0.0, up=0.0):
        def f():
            pt, rt = pose_at(sum(SEAT_BAND) / 2)
            _CMD["tool"] = (pt, rt)
            if right is None:  # the carrying grip
                rt_, rr_ = _hand_on_tool("right", pt, rt)
                return {"left": (*_hand_on_tool("left", pt, rt, (LEFT_PRESS, 0, 0)), F_BEAM),
                        "right": (rt_ + (dx, 0, up), rr_, F_GRIP if dx == 0.0 else FINGER_OPEN)}
            return {"left": (*_hand_on_tool("left", pt, rt, (LEFT_PRESS, 0, 0)), F_BEAM),
                    "right": (_hand_on_tool("right", pt, rt, top_down=True)[0] + (0, 0, up),
                              _hand_on_tool("right", pt, rt, top_down=True)[1], right)}
        return f
    add("wrench: handle back up", 1.0, back_up(F_GRIP))
    add("wrench: right hand off", 0.7, back_up(FINGER_OPEN, up=0.035))
    add("wrench: right hand round to the carrying grip", 1.6, back_up(None, dx=-0.05))
    add("wrench: right hand onto the handle", 0.9, back_up(None, dx=-0.0001))
    add("wrench: right hand grips", 0.7, back_up(None))

    def away(out, lift=0.0):
        def f():
            pt, rt = _tool_pose_on(sum(SEAT_BAND) / 2, out)
            st["held1"] = (pt + np.array((0.0, 0.0, lift)), rt)
            return carried(*st["held1"])
        return f
    add("wrench: pull the cup off", 1.2, away(0.035))
    add("wrench: back from the filter", 1.0, away(0.07, 0.02))
    for k in range(1, TURN_STEPS + 1):  # the quarter turn back: the cup facing down again
        def unturn(k=k):
            p0, r0 = st["held1"]
            pivot = p0 + r0 @ np.array((0.0, 0.0, HANDLE_Z))
            rr = _rot_y(90.0 * k / TURN_STEPS)
            return carried(pivot + rr @ (p0 - pivot), rr @ r0)
        add("wrench: turn the cup down", 0.8, unturn)

    def lay(finger, up=0.0):
        def f():
            p0, r0 = st["held1"]
            pivot = p0 + r0 @ np.array((0.0, 0.0, HANDLE_Z))
            rr = _rot_y(90.0)
            p, r = pivot + rr @ (p0 - pivot), rr @ r0
            p = np.array((p[0], p[1], TOP + CUP_DEPTH + 0.003 + up))
            _CMD["tool"] = (p, r)
            return {s_: (np.asarray(_hand_on_tool(s_, p, r)[0]) + (0, 0, up), _hand_on_tool(s_, p, r)[1],
                         finger if finger is not None else fgrip(s_)) for s_ in ("left", "right")}
        return f
    add("wrench: lay it on the bench", 1.6, lay(None))
    add("wrench: let go", 0.7, lay(FINGER_OPEN))
    add("wrench: hands up", 0.9, lay(FINGER_OPEN, 0.06))

    def clear():  # up and clear of the wrench, before heading home
        return {"left": ((0.25, 0.10, TOP + 0.20), _td(0.0), FINGER_OPEN),
                "right": ((0.25, -0.30, TOP + 0.20), _td(0.0), FINGER_OPEN)}
    add("hands clear", 1.4, clear)

    out = {"left": [], "right": []}
    for label, dur, spec, keep in steps:
        cache = {}

        def both(spec=spec, cache=cache):
            if not cache:
                cache.update(spec())
            return cache

        for arm in ("left", "right"):
            out[arm].append(_Live(dur, lambda arm=arm, both=both: both()[arm], label, keep))
    return out


def demo():
    """Closed loop on the filter's thread and the wrench's pose, as a mechanic feels and sees them.

    Left arm: picks the filter up off the bench by its closed end (top-down, the pads 2.5 cm in from its end, the
    fingers closing along x so the filter cannot pivot between them), carries it out and across to the nipple and
    turns it to point at it, reads where its gasket face is and which way it points and corrects both (five looks:
    the arm sags 2-3 cm under the filter in this pose), and pushes it onto the nipple. If it is not on the nipple it
    backs off and tries again (twice at most). It spins the filter on clockwise (seen from its end) in wrist rolls of
    up to 150 degrees, following the thread in and regripping between them, until a roll past gasket contact turns it
    under 8 degrees (the pads slipping: hand-tight); then it lets go and backs off. The right arm waits at home.

    Both arms: take the wrench where it lies flat, top-down, the left across the ratchet's head and the right on the
    grip 55 cm out (each hand checks it is holding, and tries once more 8 mm lower if not), lift it 7 cm and turn it a
    quarter turn about the handle while bringing it across, so the cup faces the engine, hold it in front of the
    filter with the handle at an angle the right arm reaches (lining the cap's flutes up with the filter's when that
    angle allows; the flutes' pointed ends turn the cap in otherwise), read where the cap is and correct (two looks),
    and push the cup over the fluted end. The right hand moves to a top-down grip on the grip. Strokes: the handle
    swung down from 6 degrees above level to 10 below by the right hand, pushing what the gasket takes (plus 1 N*m),
    aimed past the handle by its arm's give, while the left presses the cap on (8 N); then swung back up with the
    ratchet clicking. Each stroke is sized from the filter's angle when it starts; they stop once it is 270 degrees
    past contact (18 N*m) or after two strokes that turn it under 2 degrees. The right hand goes back to the grip it
    carried with, the cup is pulled off, the wrench turned back cup down and laid flat on the bench; both let go, lift
    clear and go home."""
    st = {"stop": None, "gains": [], "wstop": None}
    left = _hand_phase(st)
    hand = list(left)
    right = [_Wait(lambda: sum(w.duration for w in hand), "wait at home")]
    w = _wrench_phase(st)
    left += w["left"] + [kit.Home(2.5), kit.Home(2.0, label="hold at home")]
    right += w["right"] + [kit.Home(2.5), kit.Home(2.0, label="hold at home")]
    return {"left": left, "right": right}
