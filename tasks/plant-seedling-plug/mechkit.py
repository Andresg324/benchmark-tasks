"""mechkit: joint behaviors ("laws") for yamkit Mechanism links. This task uses one: a return spring with damping.

    stem = kit.Link("stem0", "revolute", "Y", anchor=(0, 0, 0.03), lower=-1.4, upper=1.4, law=mk.Spring(0.0, 0.165, 0.002))

A law sets the joint's drive each control step from the joint's position `q` and speed `qd` and returns
(target, stiffness, effort, damping): PhysX's own (implicit) joint drive applies stiffness * (target - q) + effort -
damping * qd at every physics substep. Gains must be implicit like this: a spring computed once per 20 Hz control step
and held as an effort is unstable on light links. yamkit gives every link with a law a PhysX drive for this.
Units: rad for hinges; torques N*m.
"""
from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Spring:
    """A plain return spring with damping: pulls the joint to `rest` with stiffness `k` (N*m/rad) and damping `damp`
    (N*m*s/rad)."""

    rest: float
    k: float = 0.3
    damp: float = 0.02

    def drive(self, key, q, qd, J, mem):
        return self.rest, self.k, 0.0, self.damp

