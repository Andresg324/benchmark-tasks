# Scribe a full circle with spring dividers, keeping the point seated in the center-punch mark

A mild steel plate coated with blue layout dye lies on the bench, with two scribed center lines crossing at a
center-punch mark: the center of a 60 mm hole. A pair of 100 mm spring dividers, already set to a 30 mm radius,
stands with its points stuck in a cork block beside the plate. "Scribe the full circle for the hole on the blue steel
plate with the dividers, centered on the punch mark, then stand the dividers back in the cork." A machinist sets one
point in the punch mark, presses lightly on the handle so it stays seated, swings the other leg a full turn so its
point scribes the circle through the dye, then lifts the dividers off and sticks them back in the cork. If the point
skates out of the mark mid-swing the line comes out doubled or off-center; starting off the mark gives a circle with
the wrong center; a short swing leaves a gap. A scribed line in steel can't be rubbed out.

## Spec

| Object | What it is | Where (cm, table frame) |
|---|---|---|
| Plate | mild steel, 13 x 11.5 x 0.6 cm, blue layout dye, fixed | x 20 to 33, y -15.5 to -4 |
| Center lines | two bright scribed lines 9 cm long, crossing at the mark | through x 26, y -10 |
| Punch mark | a center-punch mark, about 2 mm across | x 26, y -10 |
| Cork block | 6 x 6 x 0.6 cm, top level with the plate's, fixed | x 23 to 29, y -24.5 to -18.5 |
| Dividers | 100 mm spring dividers set to 30 mm: two steel legs on a spring bow, adjusting screw and nut, 80 g | standing in the cork, legs along x, handle over its center (x 26, y -21.5) |
| Handle | black knurled steel, 2.6 cm across, 2.4 cm tall, 11.2 to 13.6 cm above the points, 20 g, on a light spring (4 mm travel) | on the dividers' top |

Mechanism (`task.py`, built with `pivotkit.py`, `mechkit.py` and `yamkit.py`):

- The dividers ride upright on a chain of joints: a vertical slide (they lift off the steel and fall back under their
  own weight), then `pivotkit.planar`: a turn about the vertical through the punch mark and two slides, so they can
  be carried anywhere over the bench and turned. The handle slides 4 mm down its stem against a spring (1.5 N preload,
  500 N/m).
- The pivot point is seated in the mark when the points are on the steel (within 0.5 mm) and the pivot point drops
  within 0.5 mm of the mark's center; it stays in until pushed 1.5 mm off (the mark's 1 mm rim plus 0.5 mm
  for the sim's play in the seated point), or until the dividers
  have turned 1 degree without the handle pressed at least 1.5 mm (2.25 N): unopposed by a press, the scribing
  point's drag lifts the pivot point out of the mark. While seated, the kit narrows both slides' stops to 0.3 mm
  either side of where the point went in, so the dividers can only turn about the mark.
- Down and unseated, the points drag on the steel (heavy slide damping).

## Scoring (whole-run history, recorded each step by `success()`)

| Outcome | Score |
|---|---|
| Either point moved more than 1 mm in all over the plate while down on it with the pivot not seated (an off-mark start, a skate out of the mark, or dragging the dividers across the plate) | 0, irreversible |
| A tool point within 3 cm of the handle's center (still holding the dividers) | 0 while so |
| At least 355 degrees of the circle scribed with the pivot seated, the dividers standing back in the cork, both arms home and still for 1 s | 1.0 |
| The same, arms not home | 0.9 |
| At least 355 degrees scribed, the dividers left elsewhere | 0.75 |
| At least 180 degrees scribed | 0.5 |
| Anything else, including doing nothing | 0 |

Possible scores: 0, 0.5, 0.75, 0.9, 1.0. The arc counts each 0.1-degree sector of the circle once, so swinging back and
forth over the same arc does not add up.

Why these numbers:
- 0.5 mm to seat, 1 mm to skate out: a lightly struck center-punch mark is about 2 mm across with a 90-degree cone;
  a point within 0.5 mm of its center slides down into it, and one pushed out to the rim has left it.
- 0.3 mm play: how far the seated point can shift in the cone.
- 1.5 mm press (2.25 N): a light fingertip press, enough to hold a hardened point in the mark against the scribing
  leg's drag; 1 degree unpressed is 0.5 mm at the scribing point, about the depth of the mark.
- 1 mm stray: a stray scratch that long shows plainly in blue layout dye and reads as a second line.
- 355 degrees: a 5-degree gap is 2.6 mm on the 60 mm circle, about a saw kerf, so the circle reads as closed.
- 180 degrees: half the circle cleanly scribed, the least that is real progress.
- 3 cm hands off: the handle is 2.6 cm across, so a tool point that close is still on it.
- Arms home: each tool point within 3 cm of where it started, and still.

In a real trial: film from above, then inspect the plate under a loupe: one clean circle centered on the punch mark
(a doubled or offset arc, or a scratch near the mark, is a fail), measure the gap with a scale, and check the
dividers stand in the cork.

## How it differs from nearby board tasks

#1205 (automatic center punch) makes the punch mark this task starts from: one shot on a cross, judged by where the
dent lands. #1374 files a block down to a scribed line. #1273 opens a folding fan about its rivet with one hand holding
a guard stick. Our own sewing-pivot-at-corner turns a panel 90 degrees about a needle after working two machine
controls in order. Here the scored skill is keeping a pivot point seated under steady pressure through a full
360-degree swing that the wrist can't make in one stroke, so it has to regrip without the point moving.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, laws, scorer (`score()`, `success()`) and scripted demo (`demo()`). |
| `pivotkit.py` | A pin pivot that holds only while a condition holds (planar part on a turn + two slides; limits narrowed while in). |
| `yamkit.py` | Local copy with `Link.parent` (chained links), cylinder `axis`, law-set joint limits, inertia for collider-less links. |
| `mechkit.py` | Joint laws (Free). |
| `test_task.py`, `test_pivotkit.py` | Offline checks; no simulator needed. Each prints `all pass`. |
| `task.json` | The task card. |

## Run a trial

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/dividers-scribe-circle/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Scribe the full circle for the hole on the blue steel plate with the dividers, centered on the punch mark, then stand the dividers back in the cork." \
      --scorer task_score --max-steps 2400

Model run (Dream-1): the same with `--policy dream`. Tests: `python test_task.py`, `python test_pivotkit.py`.

## The demo

1. Right arm grips the dividers' handle in the cork, lifts the points 3.5 cm and carries them over the punch mark.
2. Lowers the points to just above the steel, settles, sets the pivot point in the mark and presses the handle.
3. Swings the handle round the mark in three 125-degree strokes (375 degrees in all), easing off, letting go, rolling
   the wrist back and gripping and pressing again between strokes; the point stays seated since nothing turns while
   the hand is off.
4. Lifts the dividers off, carries them back, sticks the points in the cork, lets go and goes home. The left arm
   stays home.

## Runtime and seeds

- Demo: 905 control steps (45.3 s simulated). Model run limit: 2,400 steps (1 minute at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

The dividers are rigid and ride upright on joints: they can't tip or fall, and their spread is fixed at 30 mm. The
handle is enlarged from about 8 mm to 2.6 cm across so the gripper can hold it. The points don't cut: the seat in the
punch mark is a condition on the joints (above), and the scribed line is drawn as bright dots wherever the scribing
point has been while down on the plate (drawn 1.6 mm wide to show on camera; a real line is about 0.2 mm), so a skated
swing shows as a doubled or offset arc. Friction of the points on the steel is damping. The plate and cork are fixed.

## Real-world build

A 13 x 11.5 cm offcut of 6 mm mild steel plate, coated with layout dye, with two center lines scribed and the
crossing center-punched; a 6 x 6 cm cork tile beside it; a pair of 100 mm spring dividers set to 30 mm, with a
2.6 cm knurled sleeve pushed over its handle. Stand the dividers in the cork, legs along the plate's long side, before
each trial. Each trial needs a fresh plate or a re-dyed one. Divider points are sharp; the YAM arms move slowly, so the
trial is safe with hands kept clear.

## Licenses

All code here was written for this task; no third-party code or assets are included. Every shape is generated by
`yamkit.py`.
