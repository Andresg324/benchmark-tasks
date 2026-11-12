# Unmold a tart from a loose-bottom pan: stand it on a cake stand and lower the ring away level

A baked tart has cooled in its 24 cm loose-bottom fluted tart pan on the counter, and a small cake stand stands on the
table behind it. "Take the tart out of its pan." means what a home baker does with every tart: a tart cannot be turned
out upside down like a cake (the crust and topping would break), and lifting the pan by its rim lifts everything,
because the ring's inward bottom lip holds the loose base. So the cook holds the pan by the outer rim, stands the
base on a cake stand narrower than the base, and lowers the ring away level, so it drops to the counter around the
stand and leaves the tart on its base, on the stand, ready to serve (America's Test Kitchen does the same over a can;
a narrow stand serves it directly). Lowered unevenly, the ring cocks and its fluted wall can lever into the
baked crust edge and break it.

## Spec

| Item | What it is |
|---|---|
| Instruction | "Take the tart out of its pan." (the stand as the support, the two-hand level lowering and the hands-off-the-tart rule are never stated) |
| Category | Societal relatability: baking at home is one of the most common household activities, and unmolding is a step every tart baked in a loose-bottom pan needs; the method is the standard one taught by America's Test Kitchen |
| Rule | Hold the pan only by its outer rim; stand the base, centered, on the cake stand; lower the ring away level with both hands; never touch the tart; let go |
| Physical core | The ring's 1.25 cm inward bottom lip carries the loose base: lifted by the rim, the whole pan comes up; only a support under the base (the 11 cm cake stand, inside the ring's 21.2 cm bottom opening) lets the ring go down without it |
| Two arms | The pan with its tart, 700 g, is 26.7 cm across at its rim and is held at two opposite points of its lip: carried by one pinch it tips and the tart slides off its base; lowering one side alone cocks the ring. Once the base rests on the stand, letting both sides go at once and letting the ring drop to the counter is also correct |
| Trap | Lifting the pan and setting it down whole (0, or 0.25 if left whole on the stand), setting the base off center on the stand (0), lowering one side first (cocked: 0.5), any finger touching the tart (0) |
| Kits | `yamkit.py` (props, tilted grips; local change: box colliders can turn about z), `guardkit.py` (per-trial latches; local addition: `touching()`, contact by the gripper's own shape) |

### How it differs from neighboring tasks

- Turning a cake out of its pan (a two-hand flip onto a plate) inverts the pan; a tart cannot be inverted, and here
  the pan goes down while the tart stays up.
- Springform tasks open a side clasp; this pan has no clasp, and the base is freed only by supporting it from below
  while the ring is lowered past it.

### Scene (cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0)

| Object | Type and size | Material | Position |
|---|---|---|---|
| Tart pan ring | 24 cm (9.5-inch) fluted ring with a removable bottom: wall 2.6 cm tall with 30 flutes 2 mm deep; inward bottom lip from 10.6 cm radius out to the wall, 1.25 cm wide (4 mm thick); top rim widened to a flat lip 1.2 cm wide out to 13.35 cm radius (4 mm thick); 150 g | tinned steel | on the table, center (22.5, 0) |
| Pan base | 23.4 cm disk, 3 mm thick, 100 g, resting on the ring's bottom lip | tinned steel | in the ring |
| Tart | baked tart, 23.3 cm across, 2 cm tall, fluted golden crust edge and yellow filling, 450 g | pastry and filling | on the base |
| Cake stand | small pedestal, 7 cm tall: 11 cm top on a 5 cm stem and 9 cm foot, fixed | white ceramic | standing at (41.5, 0) |

## Scoring

Three rules are irreversible once they hold for 2 control steps in a row (so a one-step physics pop does not count,
but a quick slip undone at once does).

| Outcome | Score |
|---|---|
| A gripper touching the tart: its palm or a finger in contact with the tart's top, side or crust edge (see "Contact" below); a hand hovering over it more than 2 mm away does not count. A finger can leave a mark in the filling or a dent in the crust | 0, irreversible |
| The tart tilted past 20 degrees or off its base (pushed more than 5 mm across it, which only a hand or the ring shoving its crust edge does, or lifted off it): proxy for a slid, dented or broken tart | 0, irreversible |
| The base, with the tart, not resting on the stand at the end: center within 3 cm of the stand's axis, within 5 degrees of level, its bottom within 5 mm of the stand's top, still (includes doing nothing and setting the pan down whole anywhere else) | 0 |
| On the stand, but the ring is still on (the base on its bottom lip), or a gripper is still touching the ring or the base: the base is centered on the stand, but the job is not finished, since a hand still holding the pan can lift the ring back or drag the base off center as it moves away, and whether the ring and base stay put is only known once they are let go | 0.25 |
| On the stand, let go, but the ring is not flat on the table around the stand (bottom within 5 mm of the table, within 5 degrees of level, the stand inside its bottom opening): the ring was dropped partway and is caught against the stand or the crust | 0 |
| Unmolded and let go, but the ring tilted past 6 degrees while sliding down past the crust: the tart is out, but its edge was put at risk of breaking | 0.5, irreversible |
| Unmolded level and let go, no gripper touching the ring, base or tart (the job ends with the tart standing free on its stand; where the arms end is not scored) | 1.0 |

Doing nothing scores 0: the base never reaches the stand.

"Sliding down past the crust" is the stretch where the ring's bottom is 5 mm or more below where it holds the base
and its top is still above the base's bottom (1.7 cm of travel). The cocked rule is judged over each control step's
motion: it counts when the ring's travel between two steps overlaps that stretch and the ring is tilted past 6 degrees
at both ends of the step, so a ring moved faster than 1.7 cm a step (34 cm/s) is still checked and a one-step tilt pop
is not.

Why these numbers:

- 6 degrees: tilting the ring 6 degrees leans the top of its 2.6 cm wall 2.7 mm inward, deeper than the 2 mm flutes,
  so the high side's wall can lever into the baked fluted edge.
  The sim's tart is rigid, so the tilt is a proxy for that break. A real trial reads the tilt from a phone
  inclinometer taped to the lip and checks the edge photos for a break.
- 3 cm off center: the stand's top is 5.5 cm in radius; within 3 cm the weight of the base and tart stays well inside
  it. Measured with a ruler from the stand's edge.
- 5 degrees: drops the edge of the base 1 cm, or one side of the ring 1 cm. Measured
  with a phone inclinometer.
- 5 mm: resting on the stand (the base) or the table (the ring), not held above it. Checked by eye or with a ruler.
- 20 degrees: a tart tilted this far has slid on its base or fallen against the stand's edge (read from the video).
- Contact, 2 mm: a gripper touches a part when its own shape comes within 2 mm of the part's shape. The gripper's
  shape is the palm and both fingers as boxes fitted to the YAM's collision meshes (each finger a 6 cm body, a pad
  1 cm thick and 4 cm wide, and a narrower tip reaching 1.3 cm past the tool point), placed at the fingers' present
  opening. The 2 mm allows for the solver's resting gap and the box fit, so it is contact, not nearness: an open hand
  5 mm over the tart, or 5 mm from the ring, is not touching. The sim's tart is rigid, so touching it is the proxy for
  a finger mark; the real trial checks the video for any finger on the tart and the tart for marks, and the video
  shows whether the hands are off the pan at the end.
- 5 mm across the base: the tart sits on the base by friction, so a shift this large is a hand or the ring shoving
  its crust edge (a pencil mark on the base under the crust shows it in a real trial).

Possible scores: 0, 0.25, 0.5, 1.0. In a real trial: a baked tart shell (or a 450 g weighted card disk with a
fluted paper edge) in the pan; check from the video that no finger touched the tart, measure the ring's tilt during
the lowering from the side camera (or a phone inclinometer taped to the lip), measure the base's offset on the stand
with a ruler, and photograph the fluted edge before and after for breaks.

## Run

```
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/unmold-tart-on-a-cake-stand/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Take the tart out of its pan." --scorer task_score --max-steps 1000 --log-dir ~/yam-eval
```

Dream-1: the same with `--policy dream`. Runtime per trial: up to 50 s simulated (1,000 steps at 20 Hz); the demo finishes in 321 steps. No seeds:
`object_jitter=0`, one fixed layout. Offline scorer tests: `python test_task.py` (prints "all pass").

## Real-world build

A 24 cm (9.5-inch) fluted tart pan with a removable bottom (tinned steel, about 2.5 cm deep) and a small cake stand
with an 11 cm top, about 7 cm tall (a 28-oz can does the same job). Real pans have a 3 mm rolled rim; the sim widens it to a 1.2 cm flat lip so a parallel
gripper can pinch it, which a real trial reproduces with a 1.2 cm steel strip ring bonded to the rim (or by trialing
with a pan with a flat rim). Use a baked tart shell for the tart, or a 450 g weighted card disk with a fluted paper
edge so a break or a finger mark shows.
