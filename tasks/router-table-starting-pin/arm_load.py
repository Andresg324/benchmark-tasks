"""The arms' joint load in the scripted demo's feed: the one method behind the README's arm-load figures.

    python arm_load.py        (with the yam_arena package importable; no simulator)

At every tenth step of the planned feed, each hand's planned push and press (task.py _hand_forces, FEED_PADS,
PRESS_N and PRESS_2) are taken through the arm's Jacobian at its pads (the YAM's URDF, the pose from the package's own
IK at the planned pads with the hand's yaw), and the torque that holds the arm up is added (the URDF's link masses and
centers of mass). Each joint's torque is compared with its servo's rated (continuous) and peak torque: Damiao
DM-J4340, 9 N*m rated and 27 N*m peak, on joints 1-3, and DM-J4310, 3 and 7 N*m, on joints 4-6.
"""

import math
import os
import xml.etree.ElementTree as ET

import numpy as np
import yam_arena.demo as yd
from yam_arena import YAM_URDF
from yam_arena.constants import ARM_JOINTS
from yam_arena.station import BASES
from yam_arena.task_file import load
from yam_arena.urdf import forward_kinematics

m = load(os.path.join(os.path.dirname(os.path.abspath(__file__)), "task.py")).module
RATED = np.array([9.0, 9.0, 9.0, 3.0, 3.0, 3.0])
PEAK = np.array([27.0, 27.0, 27.0, 7.0, 7.0, 7.0])
_INERT = {}
for link in ET.parse(YAM_URDF).getroot().findall("link"):
    i = link.find("inertial")
    if i is not None:
        o = i.find("origin")
        c = np.array([float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()])
        _INERT[link.get("name")] = (float(i.find("mass").get("value")), c)


def _height_energy(q):
    poses = forward_kinematics(YAM_URDF, {n: float(v) for n, v in zip(ARM_JOINTS, q)})
    return sum(mass * m.G * (poses[k][:3, :3] @ c + poses[k][:3, 3])[2] for k, (mass, c) in _INERT.items()
               if k in poses)


def load_ratio(side, w, push, press, yaw):
    """(rated share, peak share): the largest joint torque over its rating, for the hand's pads on the board's top at
    world (x, y) `w` pushing it with `push` (N, world x and y) and pressing down with `press` (N)."""
    base = np.asarray(BASES[side], float)
    tcp = np.array([w[0], w[1], m.TOP + m.GAP + m.T + m.TIP])
    q, _, _ = yd.ik.solve(tcp, yd.ik.matrix_to_quat_xyzw(yd.grip_rotation(yaw)), q_init=None, base_pos=base)

    def tip(q):
        p, rm = yd.ik.tcp_pose(q, base)
        return np.asarray(p) + m.TIP * np.asarray(rm)[:, 2]

    jac, grav = np.zeros((3, 6)), np.zeros(6)
    for i in range(6):
        dq = np.zeros(6)
        dq[i] = 1e-6
        jac[:, i] = (tip(q + dq) - tip(q - dq)) / 2e-6
        grav[i] = (_height_energy(q + dq) - _height_energy(q - dq)) / 2e-6
    tau = grav + jac.T @ np.array([push[0], push[1], -press])  # the hand's force on the board, held by the joints
    return float(np.max(np.abs(tau) / RATED)), float(np.max(np.abs(tau) / PEAK))


def feed_steps():
    """(stretch, phi, beta) along the demo's feed, as demo() plans it."""
    phis = np.linspace(m.PHI_START, m.PHI_END, m.FEED_N + 1)
    th_a, s_a = m.SPLIT_POSE
    c_a = m.PIN + m._rot(th_a) @ (m.C_L + s_a)
    bf = math.atan2(*(m.BIT - c_a)[::-1])
    d_phi = (m.PHI_START - m.PHI_END) / m.FEED_N
    return [(0, p, None) for p in phis if p >= m.PHI_SPLIT] + \
        [(1, p, bf) for p in np.arange(bf - th_a, m.PHI_END, -d_phi)]


if __name__ == "__main__":
    worst = {}
    for k, (n, phi, beta) in enumerate(feed_steps()):
        if k % 10:
            continue
        _, _, pl, pr, yr, sq, on = m.FEED_PADS[n]
        press = m.PRESS_2 if n else m.PRESS_N
        th, s, _ = m._feed_pose(phi, beta)
        fl, fr = m._hand_forces(phi, pl, pr, sq, on, press, beta)
        for side, p, f, yaw in (("left", pl, fl, m.HAND_YAW["left"]), ("right", pr, fr, yr)):
            r = load_ratio(side, m._board_world(th, s, p), f, press, yaw)
            key = (n + 1, side)
            if key not in worst or r[0] > worst[key][0]:
                worst[key] = (r[0], r[1], float(np.linalg.norm(f)), press)
    for (n, side), (rated, peak, push, press) in sorted(worst.items()):
        print(f"stretch {n}, {side} hand: up to {push:.1f} N of push under a {press:.0f} N press; at most "
              f"{100 * rated:.0f}% of rated and {100 * peak:.0f}% of peak joint torque")
