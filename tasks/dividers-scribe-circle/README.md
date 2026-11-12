# Scribe a full circle with spring dividers, keeping the point seated in the center-punch mark

A mild steel plate coated with blue layout dye lies on the bench, with two scribed center lines crossing at a
center-punch mark: the center of a 60 mm hole. A pair of 100 mm spring dividers, already set to a 30 mm radius,
stands with its points stuck in a cork block beside the plate. "Scribe the full circle for the hole on the blue steel
plate with the dividers, centered on the punch mark, then stand the dividers back in the cork." A machinist sets one
point in the punch mark, presses lightly on the handle so it stays seated, swings the other leg a full turn so its
point scribes the circle through the dye, then lifts the dividers off and sticks them back in the cork. If the point
skates out of the mark mid-swing the line comes out doubled or off-center; starting off the mark gives a circle with
the wrong center; a short swing leaves a gap. A scribed line in steel can't be rubbed out.

Category: Societal relatability. Most people have drawn a circle with a school compass and met the same catch: the
point has to stay in its hole while the other leg goes all the way round, and a hand that lets the pressure off or
turns too far lets it slip. It is not filed under "Impact on jobs" because the trades that scribe with dividers are smaller than the board's
500,000-worker bar: the BLS Occupational Outlook Handbook counts about 288,300 machinists, 56,700 tool and die
makers and 131,100 sheet metal workers in 2025, 476,100 together.)

## Spec

| Object | What it is | Where (cm, table frame) |
|---|---|---|
| Plate | mild steel, 13 x 11.5 x 0.6 cm, blue layout dye, fixed | x 20 to 33, y -15.5 to -4 |
| Center lines | two bright scribed lines 9 cm long, crossing at the mark | through x 26, y -10 |
| Punch mark | a center-punch mark, about 2 mm across | x 26, y -10 |
| Cork block | 6 x 6 x 0.6 cm, top level with the plate's, fixed | x 23 to 29, y -24.5 to -18.5 |
| Dividers | 100 mm spring dividers set to 30 mm: two steel legs on a spring bow, adjusting screw and nut, 80 g | standing in the cork, legs along x, handle over its center (x 26, y -21.5) |
| Handle | black knurled sleeve, 2.6 cm across, 2.4 cm tall, 11.2 to 13.6 cm above the points, 20 g; turns freely on its stem and slides 4 mm down it on a light spring | on the dividers' top |

Mechanism (`task.py`, built with `pivotkit.py`, `mechkit.py` and `yamkit.py`; contact from `guardkit.py`):

- The dividers ride on a chain of joints: a vertical slide (they lift off the steel and fall back under their own
  weight), then `pivotkit.planar`: a turn about the vertical through the punch mark and two slides, so they can be
  carried anywhere over the bench and turned. On that sits the dividers' body on a hinge along the line through their
  two points (`tip`, up to 80 degrees either way, lying on their side): gravity on the legs and bow tips them over as
  soon as nothing holds them, helped by a 1 mm lean to one side (0.8 mN*m) so a let-go never stays balanced in the
  sim's exact equilibrium. Stuck in the cork, the lean is held where it was (the points can't rock in cork); off the
  steel, carried, the hand's grip holds them upright (0.3 N*m/rad). The
  handle is a sleeve that turns freely on its stem (`swivel`) and slides 4 mm down it against a spring (1.5 N preload,
  500 N/m), so a hand can keep hold of it while the legs swing a full turn underneath.
- The pivot point is seated in the mark when the points are on the steel (within 0.5 mm) and the pivot point drops
  within 0.5 mm of the mark's center; it stays in until pushed 1.5 mm off (the mark's 1 mm rim plus 0.5 mm
  for the sim's play in the seated point), or until the dividers
  have turned 1 degree without the handle pressed at least 1.5 mm (2.25 N): unopposed by a press, the scribing
  point's drag lifts the pivot point out of the mark. While seated, the kit narrows both slides' stops to 0.3 mm
  either side of where the point went in, so the dividers can only turn about the mark. This seat lock is a stand-in:
  a center-punch mark physically captures a divider point under light pressure (that is what the mark is for), and
  the sim has no cone-shaped dent for the point to sit in, so the lock does the mark's job. The robot still has to
  find the mark, press, keep pressing all the way round and not push sideways; a real trial checks that the point
  stayed seated by the circle being concentric with the punch mark (below).
- Down and unseated, the points drag on the steel (heavy slide damping).
- Nothing holds the dividers up but a hand or the cork: let go with the points on the plate, they tip over in the
  sim as they would on the bench. The scorer counts the fall at once (below) rather than waiting for them to land.

## Scoring (whole-run history, recorded each step by `success()`)

| Outcome | Score |
|---|---|
| Either point moved more than 1 mm in all over the plate while down on it with the pivot not seated (an off-mark start, a skate out of the mark, or dragging the dividers across the plate) | 0, irreversible |
| Let go standing on the plate: no hand holding the handle (the gripper's shape touching it, fingers stopped by it) for 2 steps in a row with the points on the plate, or leaning more than 10 degrees there | they fell over; nothing scribed after counts |
| A hand still touching the dividers (any part of them) | 0 while so |
| At least 355 degrees of the circle scribed with the pivot seated, the dividers standing back in the cork and let go | 1.0 |
| At least 355 degrees scribed, the dividers left elsewhere | 0.75 |
| At least 180 degrees scribed | 0.5 |
| Anything else, including doing nothing | 0 |

Possible scores: 0, 0.5, 0.75, 1.0. The arc counts each 0.1-degree sector of the circle once, so swinging back and
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
- Touching: the gripper's own shape (palm and both fingers as boxes fitted to the YAM's collision meshes, at their
  present opening) within 2 mm of the dividers' shape, which allows for the solver's resting gap and the box fit. A
  hand hovering 5 mm off is not on them. The job ends with the dividers let go, so a hand still on them earns nothing.
- Fall: dividers stand on two points in a line and tip as soon as the hand is off; 2 steps (0.1 s) is only a
  debounce against a flicker of the fingers. A 10-degree lean puts the handle 2 cm over, past where a hand still
  holding it would keep the points from rocking out of the mark. Holding means the hand touches the handle with its
  fingers stopped by the 2.6 cm handle (opening 0.12 to 0.45 of full; closed on it reads about 0.28), so an empty
  closed fist resting on it or an open hand around it is not a hold.

In a real trial: film from above, then inspect the plate under a loupe: one clean circle centered on the punch mark
(a doubled or offset arc, or a scratch near the mark, is a fail). The point stayed seated if the circle is concentric
with the mark: measure its radius from the punch mark with a scale at 8 places round it, which should all read 30 mm
(a point that slipped shifts the center and makes them differ). Measure the gap with the scale, and check the
dividers stand in the cork.

## How it differs from nearby board tasks

An automatic center-punch task makes the punch mark this task starts from: one shot on a cross, judged by where the
dent lands. Another files a block down to a scribed line, and another opens a folding fan about its rivet with one
hand holding a guard stick. Our own sewing-pivot-at-corner turns a panel 90 degrees about a needle after working two
machine controls in order. Here the scored skill is keeping a pivot point seated under steady pressure through a full
360-degree swing: the hand has to carry the handle round a 15 mm circle about the mark, pressing the whole time and
without pushing the pivot point sideways, and can't let go partway round.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, laws, scorer (`score()`, `success()`) and scripted demo (`demo()`). |
| `guardkit.py` | Gripper contact (`touching`): the gripper's own shape at its present opening against the dividers' shape. |
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
3. Carries the handle round the mark in one stroke, pressing and holding it the whole way at one wrist angle: the
   sleeve turns on its stem while the legs swing a full turn under it (about 363 degrees). It holds the press until
   the legs stop, then eases off over them. The dividers are never let go on the plate.
4. Lifts the dividers off, carries them back, sticks the points in the cork, lets go and moves away. The left arm
   stays home.

## Runtime and seeds

- Demo: 747 control steps (37.4 s at 20 Hz). Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

The dividers are rigid with their spread fixed at 30 mm, and tip only about the line through their two points (the
only way a pair standing on two points can fall). Carried in the air, the grip holds them upright as a hand does (the
gripper's two pads on a round handle would otherwise let them swing like a pendulum); on the steel nothing does. The
handle is enlarged from about 8 mm to 2.6 cm across so the gripper can hold it. The points don't cut, and the plate has no cone-shaped
dent: a real punch mark captures a lightly pressed point, and the sim's seat lock (above) stands in for that; a real
trial checks it by the circle being concentric with the mark at 8 measured radii. The scribed line is drawn as bright dots wherever the scribing
point has been while down on the plate (drawn 1.6 mm wide to show on camera; a real line is about 0.2 mm), so a skated
swing shows as a doubled or offset arc. Friction of the points on the steel is damping. The plate and cork are fixed.

## Real-world build

A 13 x 11.5 cm offcut of 6 mm mild steel plate, coated with layout dye, with two center lines scribed and the
crossing center-punched; a 6 x 6 cm cork tile beside it; a pair of 100 mm spring dividers set to 30 mm. The sliding
handle: a 3D-printed (PETG) knurled sleeve, 26 mm across and 24 mm tall, with a plain bore that slides and turns on
the dividers' handle (about 8 mm across, a drop of light oil in the bore), sitting on a compression spring of about 0.5 N/mm (10 mm inside, 12 mm free length)
that rests on a shaft collar clamped to the handle's base and is squeezed 3 mm at rest (1.5 N); a second collar
above the sleeve keeps it on and leaves it 4 mm of travel. Check the travel with a caliper's depth rod from the top
collar to the sleeve: 4 mm, and 1.5 N and 3.5 N on a small spring scale at the top and bottom of it. The press is not measured in a real trial: it is how the
sim models what keeps the point in the mark, and the line on the plate shows the outcome directly. Stand the dividers in the cork, legs along the plate's long side, before
each trial. Each trial needs a fresh plate or a re-dyed one. Divider points are sharp; the YAM arms move slowly, so the
trial is safe with hands kept clear.

## Licenses

All code here was written for this task; no third-party code or assets are included. Every shape is generated by
`yamkit.py`.
