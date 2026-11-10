# Sharpen a kitchen knife on a whetstone at the right angle

A 165 mm santoku lies on an end-grain chopping block next to a 1000-grit whetstone held in a non-slip rubber holder
on a sharpening stand. "Sharpen the knife on the whetstone, then put it back on the chopping block." means what a cook
or butcher does without thinking about it: hold the blade 15 to 20 degrees off the stone, draw it across edge first, a
few strokes on each face, then lay the knife back down flat with its edge facing away. Rubbing it flat on the stone
(the easy pose for a gripper that picked it up flat), holding it steep, or doing one face only are the tempting wrong
answers.

## Spec

### Scene (cm from the midpoint between the arm bases, x forward, y to the robot's left, table top z = 0)

| Part | What it is | Where |
|---|---|---|
| Chopping block | End-grain maple butcher block, 12 x 22 x 8 cm, fixed | center (16.4, 8.5), long side at heading 105 deg, top z 8 |
| Knife | 165 mm santoku, 100 g: steel blade 16.5 x 4.7 cm with a polished edge bevel, octagonal magnolia wa handle 9.5 cm with a horn ferrule; balance point 2 cm in front of the bolster | flat on the block, face A down, edge away (heading 15 deg), bolster at (18.4, 0.8), handle reaching past the block's edge to (20.9, -8.4) |
| Whetstone | Orange 1000-grit stone 18 x 6 x 2.5 cm in a black rubber holder (21 x 8 x 1.5 cm, 3 cm end clamps) on a 6 cm wooden stand, all fixed | center (39.4, -5.5), long side at heading -15 deg, stone top z 10 |

### Rule and scoring

| Term | Definition (all thresholds in task.py) |
|---|---|
| Sharpening angle | Measured across the blade: how far its width (spine to edge) tilts below the stone's level top, edge down. A knife pitched tip-down about its width has no sharpening angle |
| Level | The blade's length (handle to tip) within 5 deg of horizontal: a stone stroke keeps the edge lying along the stone, and a 5 deg pitch already lifts the far end of a 6 cm contact 5 mm off it |
| Held | A gripper at most 60 % open, tool point within 2 cm of the handle's axis between its ends |
| On the stone | Held, edge lower than spine, some edge point over the stone and within 3 mm of its top |
| Stroke | One touch with 5 cm of edge-leading travel at 13 to 22 deg (15 to 20 deg, plus 2 deg for a hand-held angle), the blade level and at least 3 cm of edge (3 scorer points 1 cm apart, half the stone's 6 cm width) within 3 mm of the stone; a tip or heel dragged alone sharpens nothing |
| Damage (irreversible) | 2 cm rubbed steeper than 22 deg in one touch (edge rounded), or the edge dropping 3 mm below the stone's top just past an end within 0.5 s of contact (rolled off, chips the edge) |
| Put down safely | Let go (tool points 5 cm from the handle), flat (within 20 deg) on the block, edge within 60 deg of +x (away) |

| Outcome | Score |
|---|---|
| Damage, or fewer than 3 strokes on both faces (do nothing, flat strokes, hovering, dragging without a grip, edge-trailing) | 0 |
| 3+ strokes on one face only | 0.25 |
| 3+ strokes on each face, knife still held, left on the stone or edge towards the robot | 0.75 |
| Both faces, put down safely, arms not home | 0.9 |
| Both faces, put down safely, arms home and still for 1 s | 1.0 |

Why edge leading: an edge-first stroke cuts into the stone like slicing a thin layer off it and raises less of a wire
burr than a trailing stroke; on face A the demo pushes (edge away from the robot), on face B it pulls.

## How it differs

Unlike the board task #772 (sand an edge with a block held flat on it), the skill here is holding a tool tilted into a
narrow band rather than flat: the knife is held off the stone at 13 to 22 degrees measured across the blade, with its
length level so 3 cm or more of the edge is on the stone, and moved edge leading (pushed on one face, pulled on the
other), then flipped over to sharpen the second face, since one face alone only raises a wire. Going too steep is
irreversible damage (the edge is rounded over and the trial scores 0 even if later strokes are good), whereas the
sanding block has no wrong side and no angle that ruins the work.

## Files

- `task.py`: scene, scorer and scripted demo (right arm only). `yamkit.py`: local copy of the kit, with two additions
  (a `com` center of mass on `Prop`, and an explicit `rot` matrix on `Tilted`).
- `test_task.py`: offline scorer checks (prints "all pass").

## Run a trial

```
inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/knife-on-whetstone-at-angle/task.py \
  -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Sharpen the knife on the whetstone, then put it back on the chopping block." \
  --scorer task_score --max-steps 2400 --log-dir ~/yam-eval
```

## The demo

The right arm slides its open hand sideways round the handle (fingers above and below it, approach along the
handle), lifts, turns the knife to 17 degrees edge-down over the stone and makes four push strokes on face A, rolls
the knife over through upright and makes four pull strokes on face B, lays it back on the block edge-away, lets go and
goes home. The wrist roll (joint 6) covers the 146 degrees between the two faces; the hand is turned 90 degrees round
the handle at the pick so both faces fall inside its range. The demo trims its aim by 2.5 degrees per face for the
knife's sag in the grip.

## Runtime and seeds

About 46 s of demo (920 steps); one fixed layout, `object_jitter=0`.

## Fidelity

Nothing is ground: sharpness is scored from the knife's pose while on the stone. The stone is rigid and dry. The knife
is 100 g (real about 140 g). The sharpening stand is 6 cm tall so the wrist housing clears the bench during strokes.

## Real-world build

Any 165 to 180 mm santoku or gyuto with a wa handle; a 1000-grit 180 x 60 x 25 mm stone in a rubber holder screwed
to a 6 cm hardwood block; a 12 x 22 x 8 cm end-grain butcher block. Fix the stand and block to the bench with
double-sided tape or clamps at the positions above. Check a trial from the top and side video with 15 and 20 degree
angle cards, and by coloring the bevel with a marker: the right angle takes the ink off at the very edge.

## Licenses

All geometry is built from boxes and cylinders in task.py; no external assets.
