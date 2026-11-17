"""Where the drag's 3 N comes from: the cutting force of the pass this rig stands in for.

    python cutting_force.py        (NumPy only; prints the mean forces of the pass at hand-feed speeds)

Curti, Marcon, Denaud, Togni, Furferi and Goli (2021), "Generalized cutting force model for peripheral milling of
wood, based on the effect of density and fibre orientation", Eur. J. Wood Prod. 79:667-678, Table 5 (up-milling,
straight edge): per tooth Ft = (Ks*h + Int)*ap, with Ks and Int per unit of wood density as functions of the grain
angle GA (deg). Averaged over a turn of a 2-flute cutter engaged over phi_e = acos(1 - 2 ae / D), with chip thickness
h = fz sin(phi), the mean force against the feed is
    Fx = z ap / (2 pi) * (Ks fz sin^2(phi_e) / 2 + Int sin(phi_e))
and the mean pull toward the bit's axis is
    Fy = z ap / (2 pi) * (Ks fz (phi_e / 2 - sin(2 phi_e) / 4) + Int (1 - cos(phi_e))).
The pass: black cherry (561 kg/m3 at 12% moisture, AHEC), a 1.25 in two-flute rabbeting bit at 22,000 rpm with a
0.5 in bearing (9.55 mm of radial engagement), 1/8 in (3.2 mm) high. The curved edge meets the board's grain at
0-25 deg (the curve spans 24.6 deg either side of the board's length). Fed by hand at 0.25-1 m/min, the pass takes
2.9-3.2 N against the feed over those grain angles; the rig's cord pulls 3.0 N.
"""

import math

RHO = 561.0  # kg/m3: black cherry at 12% moisture (AHEC)
RPM = 22000.0  # the router's speed
FLUTES = 2
RC = 0.0159  # m: the cutter's radius (1.25 in)
RB = 0.00635  # m: the bearing's radius (0.5 in)
CUT_H = 0.0032  # m: the pass's height (1/8 in)


def cutting_force(ae, ap, vf, ga_deg):
    """(Fx, Fy) in N: the mean force against the feed and the pull toward the bit's axis, for a radial depth ae (m),
    cut height ap (m), feed speed vf (m/s) and grain angle ga_deg (deg)."""
    if ae <= 0.0:
        return 0.0, 0.0
    ga = min(abs(ga_deg), 90.0)
    ks = (-5e-6 * ga * ga + 1e-3 * ga + 26e-3) * RHO  # N/mm2
    it = (-1e-7 * ga * ga + 1e-5 * ga + 55e-4) * RHO  # N/mm
    fz = 1000.0 * max(vf, 0.0) / (RPM / 60.0 * FLUTES)  # mm per tooth
    pe = math.acos(max(-1.0, 1.0 - 2.0 * ae / (2 * RC)))
    k = FLUTES * 1000.0 * ap / (2 * math.pi)
    fx = k * (ks * fz * math.sin(pe) ** 2 / 2 + it * math.sin(pe))
    fy = k * (ks * fz * (pe / 2 - math.sin(2 * pe) / 4) + it * (1 - math.cos(pe)))
    return fx, fy


if __name__ == "__main__":
    print("feed m/min  chip mm/tooth  against the feed N (grain angle 0 / 12 / 25 deg)  toward the bit N (0 / 25 deg)")
    for vf_min in (0.25, 0.5, 0.75, 1.0):
        vf = vf_min / 60.0
        fz = 1e3 * vf / (RPM / 60.0 * FLUTES)
        fx = [cutting_force(RC - RB, CUT_H, vf, ga)[0] for ga in (0.0, 12.0, 25.0)]
        fy = [cutting_force(RC - RB, CUT_H, vf, ga)[1] for ga in (0.0, 25.0)]
        print(f"{vf_min:10.2f}  {fz:13.4f}  {fx[0]:6.2f} / {fx[1]:5.2f} / {fx[2]:5.2f}{'':26s}{fy[0]:5.2f} / {fy[1]:5.2f}")
