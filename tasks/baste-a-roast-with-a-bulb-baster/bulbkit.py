"""bulbkit: a bulb baster's suction, as a stated law driven only by physics outputs (no particles, no assists).

The simulator moves rigid bodies only. The baster is a rigid tube with a bulb whose wall is four sprung panels on
prismatic joints (a yamkit Mechanism). This module keeps, for one baster, the liquid inside it (mL) and the air
trapped above that liquid (mL at atmospheric pressure), and every control step works out from the panels' joint
positions and the tip's pose alone:

  * the air's gauge pressure, isothermal (Boyle): p = P_ATM (air / V_gas - 1), with V_gas = the bulb's and the bore's
    inside volume - the volume the panels displace (each panel's face area times how far it is in) - the liquid
    inside. A panel is in as far as a finger pushes it (read from its joint), or, under a vacuum, as far as the vacuum
    draws it against its spring (k x = |p| A), whichever is further: the bulb can only hold the vacuum its wall
    resists, and a release draws only as fast as the wall springs back;
  * the flow through the tip's opening, the sharp-edged orifice law Q = Cd A sqrt(2 dp / rho) (liquidkit), driven by
    the pressure difference across the opening:
      - tip under the pan's liquid surface by s: outside the opening is P_ATM + rho g s, inside is P_ATM + p + rho g h
        (h the liquid column's height above the tip). Liquid flows in only while rho g s > p + rho g h, liquid
        flows out (back into the pan) while it is the other way round, and with no liquid left inside, air bubbles out
        while p > rho g s;
      - tip in the air: liquid inside runs out while p + rho g h beats the surface tension that holds a meniscus in
        the opening (4 sigma / d); with no liquid inside, or the baster held level or tip-up so the liquid lies away
        from the opening, air flows in or out until p = 0 (through the open tip that is settled at once). So a vacuum
        is kept only while liquid seals the opening.
    So the bulb draws liquid up only when the air inside is below the pressure at the submerged opening, which needs
    air pushed out of it first (a squeeze) and the bulb then opening up again (a release) while the tip is under the
    surface. A baster that was never squeezed only lets liquid rise in the tip to the outside level (communicating
    vessels). Squeezing and releasing in the air pulls in air, not liquid.
  * what leaves the tip in the air is a jet: it leaves along the tube's axis at the orifice's jet speed, plus the
    tip's own velocity, and falls ballistically (liquidkit.ballistic). The task decides where it lands.

Limits (state them in the task's fidelity text): one lumped air volume (no bubbles kept inside); no surface wetting or film inside the tube; the pan's liquid stays level
(the pan is not moved in the task).

Units: mL and kPa inside the model, m and s for poses and time.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

import liquidkit as lk

P_ATM = 101.325  # kPa, standard atmosphere
RHO = lk.RHO_WATER  # kg/m^3 (water: the task's juices are water with food colouring)
RHO_G = RHO * lk.G / 1000.0  # kPa per m of water column (9.79)


@dataclass(frozen=True)
class Baster:
    """A baster's inside. Its own frame: x along the tube from the tip's opening (x = 0) toward the bulb.
    `bore` is ((x, inner radius), ...) of the tube's bore from the tip to the bulb's neck (m); `bulb_ml` the bulb's
    inside volume at rest (mL); `bulb_area_cm2` its inside cross-section (for a column that reaches into it);
    `panel_cm2` each wall panel's face area (cm^2: mL displaced per cm of push); `tip_d` the opening's diameter (m);
    `k_wall` each panel's spring (N/m). Under a vacuum p a panel is drawn in until its spring balances it,
    k_wall x = |p| A; a finger holding it further in keeps it there."""

    bore: tuple
    bulb_ml: float
    bulb_area_cm2: float
    panel_cm2: float
    tip_d: float
    k_wall: float = 2000.0  # N/m: each panel's spring back to rest (what a finger pushes against)
    cd: float = lk.ORIFICE_CD
    sigma: float = lk.SIGMA_WATER

    @property
    def tip_area(self) -> float:
        return math.pi * self.tip_d ** 2 / 4.0

    def bore_ml(self) -> float:
        """The tube's bore volume, tip to bulb (mL)."""
        return self.column_ml(self.bore[-1][0])

    def inside_ml(self) -> float:
        return self.bore_ml() + self.bulb_ml

    def column_ml(self, length: float) -> float:
        """mL of bore from the tip up to `length` m along the tube (a frustum per bore segment)."""
        out = 0.0
        for (x0, r0), (x1, r1) in zip(self.bore[:-1], self.bore[1:]):
            if length <= x0:
                break
            x = min(length, x1)
            r = r0 + (r1 - r0) * (x - x0) / (x1 - x0)
            out += math.pi * (x - x0) * (r0 * r0 + r0 * r + r * r) / 3.0
        return out * 1e6

    def column_length(self, ml: float) -> float:
        """How far (m) along the tube from the tip `ml` of liquid reaches (into the bulb past the bore)."""
        if ml <= 0.0:
            return 0.0
        full = self.bore_ml()
        top = self.bore[-1][0]
        if ml >= full:
            return top + (ml - full) / self.bulb_area_cm2 / 100.0
        xs, mls = _table(self)
        return float(np.interp(ml, mls, xs))

    def cap_kpa(self) -> float:
        """Surface tension's hold on a meniscus in the opening (kPa): 4 sigma / d."""
        return 4.0 * self.sigma / self.tip_d / 1000.0


_TABLES: dict = {}


def _table(b: Baster):
    """(lengths, mL) along the bore, 0.1 mm apart: column_length's inverse by interpolation."""
    if b not in _TABLES:
        xs = np.linspace(0.0, b.bore[-1][0], 2801)
        _TABLES[b] = (xs, np.array([b.column_ml(float(x)) for x in xs]))
    return _TABLES[b]


class Bulb:
    """One trial's state of a baster: `liquid` (mL inside), `air` (mL at atmospheric pressure inside), `p` (gauge kPa).
    Starts empty, at atmospheric pressure, the panels at rest. Totals for the books: `drawn` (mL in through the tip),
    `out_air` (mL of liquid that left the tip in the air), `back` (mL that left it under the surface)."""

    def __init__(self, b: Baster):
        self.b = b
        self.liquid = 0.0
        self.air = b.inside_ml()
        self.p = 0.0
        self.drawn = 0.0
        self.out_air = 0.0
        self.back = 0.0
        self.pushes = (0.0, 0.0, 0.0, 0.0)

    # -- the air -----------------------------------------------------------------------------------------------------
    def displaced(self, pushes, gauge: float = 0.0) -> float:
        """mL the panels displace: each is in by its finger's push (m; outward counts as 0, the panels stop at rest), or,
        under a vacuum, by as far as the vacuum draws it against its spring, whichever is further (a quasi-static
        wall: the air's push on the panels is taken here, in the law, not through the simulator's joints)."""
        pull = max(0.0, -gauge) * 1000.0 * self.b.panel_cm2 * 1e-4 / self.b.k_wall  # m
        return sum(max(max(0.0, float(d)), pull) for d in pushes) * 100.0 * self.b.panel_cm2

    def gas_ml(self, gauge: float = 0.0, liquid=None) -> float:
        liquid = self.liquid if liquid is None else liquid
        return max(0.5, self.b.inside_ml() - self.displaced(self.pushes, gauge) - liquid)

    def pressure(self, liquid=None, air=None) -> float:
        """The inside gauge pressure (kPa): Boyle's law with the panels where the fingers and the vacuum put them."""
        liquid = self.liquid if liquid is None else liquid
        air = self.air if air is None else air
        p0 = P_ATM * (air / self.gas_ml(0.0, liquid) - 1.0)
        if p0 >= 0.0:
            return p0
        lo, hi = -P_ATM, 0.0  # f(p) = Boyle(p) - p falls as p rises: one root in (lo, hi)
        for _ in range(40):
            mid = 0.5 * (lo + hi)
            if P_ATM * (air / self.gas_ml(mid, liquid) - 1.0) - mid > 0.0:
                lo = mid
            else:
                hi = mid
        return 0.5 * (lo + hi)

    def head(self, liquid: float, down: float) -> float:
        """kPa of the liquid column standing over the opening; `down` = cos of the tube's angle from tip-straight-down."""
        return RHO_G * self.b.column_length(liquid) * down

    # -- one control step ------------------------------------------------------------------------------------------------
    def step(self, pushes, down: float, sub_depth: float, dt: float, sub: int = 40) -> float:
        """Advance by dt. `pushes`: each panel's inward push (m, from its joint); `down`: cos of the tube's angle from
        pointing straight down (1 tip-down, 0 level, < 0 tip-up); `sub_depth`: how far (m) the opening is under the
        pan's liquid surface (<= 0: in the air). Returns the mL that left the tip into the air this step (the caller
        lands it); liquid that leaves under the surface goes back to the pan (`back`), and what comes in is `drawn`
        (the caller takes it from the pan)."""
        self.pushes = tuple(float(d) for d in pushes)
        out = 0.0
        h = dt / sub
        cap = self.b.cap_kpa()
        for _ in range(sub):
            submerged = sub_depth > 0.0
            if not submerged and (down <= 0.05 or self.liquid <= 1e-9):
                # in the air with nothing over the opening (empty, or held level or tip-up so the liquid lies away from
                # it): the opening is open to the air and the inside is at atmospheric pressure
                self.air = self._air_for(0.0)
                continue
            dn = max(down, 0.0)
            if submerged:
                outside = RHO_G * sub_depth
                if self.liquid <= 1e-9 and self.pressure() > outside:  # air bubbles out until it matches
                    self.air = self._air_for(outside)
                    continue
                dv = self._flow(outside - self.pressure() - self.head(self.liquid, dn), h, dn, outside, 0.0)
                if dv > 0:
                    self.drawn += dv
                else:
                    self.back += -dv
                self.liquid = max(0.0, self.liquid + dv)
            else:
                drive = self.pressure() + self.head(self.liquid, dn) - cap
                if drive > 0:  # liquid runs out of the opening
                    dv = min(self._flow(drive, h, dn, 0.0, cap, sign=-1.0), self.liquid)
                    self.liquid -= dv
                    self.out_air += dv
                    out += dv
                elif self.pressure() + self.head(self.liquid, dn) < -cap:
                    # under vacuum the meniscus is drawn in: air enters and the column rides up the tube, until it holds
                    self.air = max(self.air, self._air_for(-cap - self.head(self.liquid, dn)))
        self.p = self.pressure()
        return out

    def _air_for(self, gauge: float) -> float:
        """The air (mL at atmosphere) that puts the inside at `gauge` kPa with the panels and liquid as they are."""
        return self.gas_ml(gauge) * (1.0 + gauge / P_ATM)

    def _flow(self, drive, h, down, outside, cap, sign=1.0) -> float:
        """mL through the opening in h s for a pressure difference `drive` (kPa; > 0 in when sign = 1), by the orifice
        law, never past the point where the difference is spent (it falls as liquid moves: the air expands or
        compresses and the column changes)."""
        d = abs(drive)
        if d <= 0.0:
            return 0.0
        q = self.b.cd * self.b.tip_area * math.sqrt(2.0 * d * 1000.0 / RHO) * 1e6  # mL/s
        dv = q * h * (1.0 if drive > 0 else -1.0)
        # stiffness of the drive to moving liquid: d(drive)/d(liquid) (numerical), so a step never overshoots
        e = 1e-3
        if sign > 0:
            f = lambda v: outside - self.pressure(liquid=v) - self.head(v, down)  # noqa: E731
            k = abs((f(self.liquid + e) - f(self.liquid)) / e)
            lim = d / k if k > 1e-9 else abs(dv)
            return max(-lim, min(lim, dv))
        f = lambda v: self.pressure(liquid=v) + self.head(v, down) - cap  # noqa: E731
        k = abs((f(self.liquid) - f(self.liquid - e)) / e) if self.liquid > e else 0.0
        lim = d / k if k > 1e-9 else abs(dv)
        return max(-lim, min(lim, dv))

    def jet_speed(self, rate_ml_s: float) -> float:
        """m/s of the jet leaving the opening carrying `rate_ml_s` (continuity through the contracted jet, Cd A)."""
        return rate_ml_s * 1e-6 / (self.b.cd * self.b.tip_area)


def land(p0, v0, planes):
    """Where a jet leaving p0 at v0 first comes down through one of `planes` [(name, point, normal, inside(pt)) ...]:
    each plane is tried highest first; returns (name, point) of the first whose `inside` accepts the crossing point,
    else (None, point on the last plane)."""
    last = None
    for name, pt, n, inside in planes:
        hit = lk.ballistic(p0, v0, pt, n)
        if hit is None:
            continue
        last = hit[1]
        if inside(hit[1]):
            return name, hit[1]
    return None, last
