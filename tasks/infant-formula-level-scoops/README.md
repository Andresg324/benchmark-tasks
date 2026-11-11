# Measure powdered formula into a 4-ounce bottle

A wide-neck bottle holds 120 mL (4 fl oz) of water, next to an opened tin of formula powder with a leveling bar
across its opening; the tin's scoop lies on a stand. "Measure the formula into this bottle." (Capping and shaking
come after and are not part of the task.) US standard formulas are
mixed at one unpacked, level scoop per 2 fl oz of water, so the bottle takes exactly two level scoops (17.8 g). A
scoop dipped into the powder comes out heaped (about 29% over); the worker draws it up against the leveling bar so the
heap falls back into the tin, then tips it into the bottle. Heaped, pressed-down or extra scoops make the feed too
strong, and powder in water cannot be taken back out.

Run (from the repo root, Isaac Lab-Arena installed; the kit files yamkit.py, pourkit.py and dosekit.py sit next to
task.py):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/infant-formula-level-scoops/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Measure the formula into this bottle." --scorer task_score --max-steps 1400
    python tasks/infant-formula-level-scoops/test_task.py      # offline scorer checks, prints "all pass"

Use `--policy dream --max-steps 2400` for a Dream-1 trial. The scene is deterministic (no seeds, object_jitter=0).
The scripted demo plans 64.5 s (1,290 steps) and holds success 1 s at the end; a full trial is 2,400 steps. Set
`DOSEKIT_DEBUG=1` to print grams per bin, packing and spill each step.

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Formula tin (fixed) | white steel, 14.6 cm across (14 cm inside), 10 cm tall, lid off, plain pale-blue label band; 700 g of pale cream powder, top about 0.65 cm below the rim | axis at x 31, y 11 |
| Leveling bar (fixed, with a collider) | gray, 1 cm wide, 0.4 cm thick, across the opening along x; its underside at the rim is the cutting edge | center line y 12.5, z 10 to 10.4 |
| Bottle (fixed) | clear 270 mL wide-neck plastic bottle, 6 cm across, 11.5 cm tall, 5 cm neck opening, in a gray holder ring 9 cm across; 120 mL of water, the 120 mL line marked | axis at x 40, y -3 |
| Scoop (loose, 45 g) | dark-blue plastic; round handle 2.5 cm across, 7 cm long, along y; neck down to a round bowl 3.9 cm across outside, 3.6 cm inside, 1.75 cm deep (17.8 mL); rim 3.9 cm below the handle's axis | handle center x 23, y -12.5, z 8; bowl center y -4.75 |
| Scoop stand (fixed) | white; post 1.6 x 6.6 cm, 6.75 cm tall under the handle; pad 3.5 x 3.5 x 2.2 cm under the bowl | x 23, y -12.5 |

Powder (dosekit `Material`): loose 0.497 g/mL (as poured), 0.50 when dipped (labels give the scoop's weight dipped
and leveled), 0.64 packed; angle of repose 42 degrees. One level scoop: 17.8 mL x 0.50 = 8.9 g (US labels: 8.6 to 8.8
g). A dipped scoop is heaped to 11.4 g; a fingertip pressed into it packs the top 2 cm.

How the powder moves (dosekit, from the poses each control step): the scoop's mouth more than 4 mm under the
powder's surface fills it to the brim plus a cone at the angle of repose; the leveling bar is a `dk.Edge` on the tin
along its underside (`below=0.002`: a scoop can only be drawn up against the bar, and one carried over its top cuts
nothing), and wherever the scoop's heap passes within 4 mm of it, the heap is cut down to the bar and falls back
into the tin (with the rim pressed up to the underside or up to 1 mm short of it the result is within the dose
window; 3 mm or more short, enough heap stays to go over it); the kit also records which of the scoop's mouth cells
the bar has passed over since each dip (`dk.swept_since`), counting the whole band the mouth crossed between control
steps, so a fast draw (1 cm a step, 20 cm/s) is not skipped over (a local dosekit fix); the scoop tipped past 45 degrees pours from its lowest rim point straight down, into the bottle
if over its neck, else onto the table.

## Scoring

| Outcome | Score |
|---|---|
| A scoopful reached the bottle without the bar having passed over all of the scoop's mouth since it was dipped (latched) | 0 |
| The bottle has held more than 19.1 g at any time (latched) | 0 |
| A tool point within 4 cm of the scoop's handle, neck or bowl (still holding it) | 0 |
| Scoop still moving (over 1 cm/s) or off the table | 0 |
| 1 g or more of powder on the table | 0 |
| 16.6 to 19.1 g in the bottle, both arms home and still, held 1 s at the end | 1.0 |
| The same, arms not home | 0.9 |
| 8.3 to 9.5 g in the bottle (one level scoop) | 0.25 |
| Anything else (nothing, a partial scoop, one heaped scoop) | 0 |

Possible scores: 0, 0.25, 0.9, 1.0. Doing nothing scores 0.

Why the bar is required: the label's level scoop is leveled with the tin's leveler (or by drawing the scoop up the
side of the container), and the model's powder, being weightless and non-solid, sheds its heap exactly when the scoop
is tipped just under 45 degrees; a real cohesive powder sheds unevenly, so that shortcut is not a level scoop in the
real world. A load that was tipped and then also drawn along the bar is level and counts.

Why the numbers: the dose is two level scoops, 17.8 g, at the label's one scoop per 2 fl oz. The window is plus or
minus 7%, because a scoop drawn 1 mm short of the bar's underside keeps about 0.5 g (5.7% of a scoop) more; one heaped
scoop with a level one is 20.3 g (+14%) when the whole heap reaches the bottle, but part of a heap can fall off on
the carry (the sim's heaped-then-level run ended at 19.0 g, inside the window), so the strike-off rule, not the
grams, is what catches an unleveled scoop. The amount
latches because powder in water cannot be taken back out. Under 1 g on the table is a clean job (a level scoop is
8.9 g). The hand grips 1.5 cm from the handle's center, so a hand on the scoop, by its handle, neck or bowl, is well
inside 4 cm of the scoop's body line (the tool point sits between the fingertips, within about 2 cm of what they
hold) and one that has let go and backed out 6 cm is outside. Partial credit 0.25 goes only to one exact level scoop, which a second scoop can still complete.

In a real trial: weigh the bottle with its water on a 0.1 g kitchen scale before and after, weigh any powder brushed
off the table, check on the video that each scoop was drawn along the bar before it went to the bottle, and check
that the arms are back where they started.

Offline tests (`test_task.py`): do nothing 0; two level scoops 1.0 (0.9 not home); still holding 0, let go 0.9; scoop
sliding 0; one level 0.25; one heaped 0; heaped + level 0 (latched); two heaped 0; pressed with a finger then leveled
0; three level 0; a third level scoop partly tipped in 0 (latched); a third dipped and dumped back into the tin 1.0; scoop held still in the air by its neck or bowl 0; the draw along the bar done in 10, 5 or 3 steps (up to 1 cm a step) 1.0, and fast draws over the bar's top or 1 cm under it 0; one heaped then one level 0; drawn 1 cm under the bar 0; rim at the bar, 1 mm under it and pressed 3.5 mm into it (stopped by
its collider) 1.0, 3 mm under 0; carried over the bar's top 0; heap shed by tipping, no bar 0 (also when only the
second scoop is tipped); tipped and then drawn along the bar 1.0; poured
beside the bottle 0 (spill); held level over the bottle, never tipped 0; scoop knocked off the stand or off the table
0; fingers poked into the tin and hovering at the bottle 0; a new trial after an over-strong one 1.0; and the scripted
demo replayed with perfect tracking (less the 5.5 mm the loaded arm and grip sag in the sim), the bar's collider
emulated, 1.0 (17.5 g in the bottle; 18.1 g in the sim).

## Real-world build

- A 12.4 to 30.8 oz powdered formula tin with its own scoop (any US brand; cover the brand with a plain label band),
  the scoop's handle wrapped in a printed or heat-shrink sleeve to 2.5 cm round with a few washers inside for weight.
- If the tin has no built-in leveler, a 10 x 4 mm aluminum flat bar across the opening, epoxied to the rim.
- A 9 oz wide-neck bottle in a clamp-on bottle holder or a weighted ring; 120 mL of room-temperature water.
- A scoop stand: a 1.6 cm wood post and a 2.2 cm block, glued to a base.
- Calibrate before trials with each tin or lot: the sim's 8.9 g level scoop is the nominal label value (US labels
  8.6 to 8.8 g), not the real tin's. Weigh 10 level scoops of the product used, each leveled against the bar, take
  their mean m (check all 10 fall within 3% of m), and score 2m +- 7% for 1.0, m +- 7% for 0.25 and latch 0 above
  2m + 7%. The 7% is the 1 mm leveling error, wider than hand-leveled scatter.
- A 0.1 g kitchen scale for scoring. Formula powder is food: safe for the arms and the people around them.

## How it differs from nearby board tasks

- #1150 (warm a baby's bottle in a water bath) and #1061 (hang a washed bottle to dry): bottle handling, no
  measuring. BEHAVIOR-1K's 1,016 activity definitions have no formula or bottle-preparation activity (nearest:
  wash_baby_bottles, filling_sugar, make_chocolate_milk, none measured by level scoops).
- #355 (choose a medicine dosing tool): picks the tool, measures nothing.
- #621 (weigh 1.00 g on a balance), #666 (scoop beads into a bowl), #1373 (screed a fill to rails): weighing to a
  readout, moving loose solids, or striking off a surface. Here the amount is set by the fill method of a fixed
  measure: level versus heaped versus packed, with no readout to watch.
