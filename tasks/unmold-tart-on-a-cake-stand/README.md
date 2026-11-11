# Unmold a tart from a loose-bottom pan: stand it on a cake stand and lower the ring away with both hands

A baked tart has cooled in its 24 cm loose-bottom fluted tart pan on the counter, and a small cake stand stands on the
table behind it. "Take the tart out of its pan." means what a baker does with every tart: a tart cannot be turned
out upside down like a cake (the crust and topping would break), and lifting the pan by its rim lifts everything,
because the ring's inward bottom lip holds the loose base. So the baker holds the pan by the outer rim, stands the
base on a cake stand narrower than the base, and lowers the ring away level, so it drops to the counter around the
stand and leaves the tart on its base, on the stand, ready to serve (America's Test Kitchen does the same over a can;
a narrow stand serves it directly). Lowered unevenly, the ring cocks and its fluted wall levers into the
baked crust edge and breaks it.

## Spec

| Item | What it is |
|---|---|
| Instruction | "Take the tart out of its pan." (the stand as the support, the two-hand level lowering and the hands-off-the-tart rule are never stated) |
| Job | Bakers (O*NET 51-3011.00); the method is the standard one taught by America's Test Kitchen |
| Rule | Hold the pan only by its outer rim; stand the base, centered, on the cake stand; lower the ring away level with both hands; never touch the tart; let go |
| Physical core | The ring's 1.25 cm inward bottom lip carries the loose base: lifted by the rim, the whole pan comes up; only a support under the base (the 11 cm cake stand, inside the ring's 21.2 cm bottom opening) lets the ring go down without it |
| Two arms | The 700 g pan is 26.7 cm across at its rim and is held at two opposite points of its lip: carried by one pinch it tips and the tart slides off its base; lowering one side alone cocks the ring. Once the base rests on the stand, letting both sides go at once and letting the ring drop to the counter is also correct |
| Trap | Lifting the pan and setting it down whole (0, or 0.25 if left whole on the stand), setting the base off center on the stand (0), lowering one side first (cocked: 0.5), any finger in the tart (0) |
| Kits | `yamkit.py` (props, tilted grips; local change: box colliders can turn about z), `guardkit.py` (per-trial latches, arms home) |

### How it differs from neighboring tasks

- Turning a cake out of its pan (a two-hand flip onto a plate) inverts the pan; a tart cannot be inverted, and here
  the pan goes down while the tart stays up.
- Springform tasks open a side clasp; this pan has no clasp, and the base is freed only by supporting it from below
  while the ring is lowered past it.

### Scene (cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0)

| Object | Type and size | Material | Position |
|---|---|---|---|
| Tart pan ring | 24 cm (9.5-inch) fluted ring with a removable bottom: wall 2.6 cm tall with 30 flutes 2 mm deep; inward bottom lip from 10.6 cm to 12 cm radius (4 mm); top rim widened to a flat lip 1.2 cm wide out to 13.35 cm radius (4 mm thick); 150 g | tinned steel | on the table, center (22.5, 0) |
| Pan base | 23.4 cm disk, 3 mm thick, 100 g, resting on the ring's bottom lip | tinned steel | in the ring |
| Tart | baked tart, 23.3 cm across, 2 cm tall, fluted golden crust edge and yellow filling, 450 g | pastry and filling | on the base |
| Cake stand | small pedestal, 7 cm tall: 11 cm top on a 5 cm stem and 9 cm foot, fixed | white ceramic | standing at (41.5, 0) |

## Scoring

The scorer watches the whole run; three rules are irreversible once they hold for 2 control steps in a row (so a
one-step physics pop does not count, but a quick slip undone at once does).

| Outcome | Score |
|---|---|
| A tool point or pad tip over the tart's footprint (or 3 mm into its side), from the base's bottom (a tip that low is pinching the crust's edge) to 1 cm above its top | 0, irreversible |
| The tart tilted past 20 degrees or off its base (pushed more than 5 mm across it, which only a hand or the ring shoving its crust edge does, or lifted off it) | 0, irreversible |
| The base, with the tart, not resting on the stand at the end: center within 3 cm of the stand's axis, within 5 degrees of level, its bottom within 5 mm of the stand's top, still (includes doing nothing and setting the pan down whole anywhere else) | 0 |
| On the stand, but the ring is still on (the base on its bottom lip), or a tool point or pad tip is within 1 cm of the ring, base or tart | 0.25 |
| On the stand, let go, but the ring is not flat on the table around the stand (bottom within 5 mm of the table, within 5 degrees of level, the stand inside its bottom opening) | 0 |
| Unmolded and let go, but the ring tilted past 6 degrees while sliding down past the crust | 0.5, irreversible |
| Unmolded level, let go, both arms home and still | 1.0 |
| The same with an arm not home | 0.9 |

"Sliding down past the crust" is the stretch where the ring's bottom is 5 mm or more below where it holds the base
and its top is still above the base's bottom (1.7 cm of travel). The cocked rule is judged over each control step's
motion: it counts when the ring's travel between two steps overlaps that stretch and the ring is tilted past 6 degrees
at both ends of the step, so a ring moved faster than 1.7 cm a step (34 cm/s) is still checked and a one-step tilt pop
is not.

Why these numbers:

- 6 degrees: tilting the ring 6 degrees leans the top of its 2.6 cm wall 2.7 mm inward, deeper than the 2 mm flutes,
  so the high side's wall levers into the baked fluted edge; level lowering keeps it under 2 degrees in the demo.
- 3 cm off center: the stand's top is 5.5 cm in radius; within 3 cm the weight of the base and tart stays well inside
  it (5 cm off, the base tipped off the edge in the sim).
- 5 degrees: drops the edge of the base 1 cm, or one side of the ring 1 cm; the demo ends under 1 degree.
- 5 mm: resting on the stand (the base) or the table (the ring), with room for the reported pose.
- 20 degrees: a tart tilted this far has slid on its base or broken against the stand's edge.
- 1 cm: fingers within 1 cm of the tart's top over its footprint are in the crust or filling; 1 cm clear of the
  ring, base and tart counts as let go.

Possible scores: 0, 0.25, 0.5, 0.9, 1.0. In a real trial: a baked tart shell (or a 450 g weighted card disk with a
fluted paper edge) in the pan; check from the video that no finger touched the tart, measure the ring's tilt during
the lowering from the side camera (or a phone inclinometer taped to the lip), measure the base's offset on the stand
with a ruler, and photograph the fluted edge before and after for breaks.

## Run

```
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/unmold-tart-on-a-cake-stand/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the tart out of its pan." --scorer task_score --max-steps 1000 --log-dir ~/yam-eval
```

Dream-1: the same with `--policy dream`. Runtime per trial: about 50 s simulated (1,000 steps at 20 Hz). No seeds:
`object_jitter=0`, one fixed layout. Offline scorer tests: `python test_task.py` (prints "all pass").

## Real-world build

A 24 cm (9.5-inch) fluted tart pan with a removable bottom (tinned steel, about 2.5 cm deep) and a small cake stand
with an 11 cm top, about 7 cm tall (a 28-oz can does the same job). Real pans have a 3 mm rolled rim; the sim widens it to a 1.2 cm flat lip so a parallel
gripper can pinch it, which a real trial reproduces with a 1.2 cm steel strip ring bonded to the rim (or by trialing
with a pan with a flat rim). Use a baked tart shell for the tart, or a 450 g weighted card disk with a fluted paper
edge so a break or a finger mark shows.
