"""Where the drag's 3 N comes from: the mean cutting force against the feed of the pass this rig stands in for.

    python cutting_force.py        (standard library only; prints the mean force against the feed at the demo's feed)

Curti, Marcon, Denaud, Togni, Furferi and Goli (2021), "Generalized cutting force model for peripheral milling of
wood, based on the effect of density, uncut chip cross section, grain orientation and tool helix angle", European
Journal of Wood and Wood Products 79:667-678, doi:10.1007/s00107-021-01667-5. Eq. 4 and Table 5 (up-milling, straight
blade): the cutting force per tooth is Fc = (Ks(GA) h + Int(GA)) ap rho, with
    Ks = -5e-6 GA^2 + 1e-3 GA + 26e-3 and Int = -1e-7 GA^2 + 1e-5 GA + 55e-4,
GA the grain angle (deg) at the mean chip position. Averaged over a turn of a 2-flute cutter engaged over
phi_e = acos(1 - 2 ae / D), with chip thickness h = fz sin(phi), the mean force against the feed is
    Fx = z ap rho / (2 pi) * (Ks fz sin^2(phi_e) / 2 + Int sin(phi_e)).
The pass: black cherry (561 kg/m3 at 12% moisture, AHEC), a 1.25 in two-flute rabbeting bit with a 0.5 in bearing
(9.55 mm of radial engagement), 1/8 in (3.2 mm) high, at 18,000 rpm (Freud's maximum speed for 1-1/4 in bits). Over
the 66 deg of engagement the mean chip position lies about 33 deg round from the edge, and the curved edge meets the
board's grain at 0-25 deg, so the grain angle at the mean chip position is 10-60 deg.
"""

import math

RHO = 561.0  # kg/m3: black cherry at 12% moisture (AHEC)
RPM = 18000.0  # Freud's maximum speed for 1-1/4 in router bits
FLUTES = 2
RC = 0.0159  # m: the cutter's radius (1.25 in)
RB = 0.00635  # m: the bearing's radius (0.5 in)
CUT_H = 0.0032  # m: the pass's height (1/8 in)
DEMO_FEED = 0.29  # m/min: the scripted demo's feed (29 cm of curve in 60 s)


def engagement(ae):
    """The cutter's engagement angle (rad) for a radial depth ae (m)."""
    return math.acos(max(-1.0, 1.0 - 2.0 * ae / (2 * RC)))


def feed_per_tooth(vf):
    """fz (mm) at a feed speed vf (m/s)."""
    return 1000.0 * max(vf, 0.0) / (RPM / 60.0 * FLUTES)


def force_against_feed(ae, ap, vf, ga_deg):
    """The mean cutting force against the feed (N), for a radial depth ae (m), cut height ap (m), feed speed vf
    (m/s) and grain angle ga_deg (deg) at the mean chip position."""
    if ae <= 0.0:
        return 0.0
    ga = min(abs(ga_deg), 90.0)
    ks = (-5e-6 * ga * ga + 1e-3 * ga + 26e-3) * RHO  # N/mm2
    it = (-1e-7 * ga * ga + 1e-5 * ga + 55e-4) * RHO  # N/mm
    fz = feed_per_tooth(vf)
    pe = engagement(ae)
    k = FLUTES * 1000.0 * ap / (2 * math.pi)
    return k * (ks * fz * math.sin(pe) ** 2 / 2 + it * math.sin(pe))


if __name__ == "__main__":
    vf = DEMO_FEED / 60.0
    print(f"at the demo's feed ({DEMO_FEED:.2f} m/min), mean cutting force against the feed:")
    for ga in (10.0, 35.0, 60.0):
        print(f"  grain angle {ga:4.0f} deg: {force_against_feed(RC - RB, CUT_H, vf, ga):.2f} N")
