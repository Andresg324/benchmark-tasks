# Turn a topstitched corner on a sewing machine: needle down, foot up, pivot, foot down

A single-needle lockstitch machine, set flush in its table, has stopped at a corner of a topstitched leather panel:
its third corner: the back and right sides are sewn and the needle sits over the corner of the stitching, up, with the presser foot down.
"Turn the corner of the leather panel on the sewing machine, ready to sew its next side." An operator turns the handwheel
by hand until the needle is down through the leather, raises the presser foot, pivots the panel 90 degrees about the
needle so the next side lines up with the feed, and lowers the foot. The needle is the pivot pin, so the corner stays
where the two lines of stitching meet. Turning with the needle up shifts the corner (and in leather a stray needle
hole is permanent); turning with the foot down drags and scuffs the work and bends the needle; forgetting to lower the
foot leaves the next stitches unclamped.

## Spec

| Object | What it is | Where (cm, table frame) |
|---|---|---|
| Machine bed | gray plate flush with the table, steel throat plate 5 x 3.2 cm, two feed-dog slots | x 17 to 33, y -24 to 7.5 |
| Machine body | off-white: pillar 15 cm tall, arm (z 9.8 to 15), head over the needle (underside z 7.2), spool on top | pillar x 22.2 to 28.8, y -20.4 to -14; head y 0.4 to 6.2 |
| Handwheel | black, 9 cm across, 1.4 cm thick, axis along y; red index mark | center x 25.5, y -21.2, z 12 |
| Crank knob | black, 2.2 cm across, 2.5 cm long, on the wheel's outer face 3.3 cm from its center | starts 60 deg above front horizontal (x 23.9, z 14.9) |
| Needle | steel, 30 mm stroke, crank-driven by the wheel | line x 25, y 3; tip starts 6.5 mm above the leather |
| Presser foot | steel, two toes either side of the needle, 7 mm lift | on the leather at the needle |
| Lifter lever | steel, 6 cm, pivot on the head's front face; detents down (start) and up | pivot x 21.8, y -1.4, z 12 |
| Lifter grip | black plastic sleeve 2.5 cm across, 4.2 cm long along y, free to spin, sticking out right from the lever's end | center x 15.9, y -3.8, z 10.8 |
| Leather panel | tan, 16 x 16 cm, 3 mm (two glued 1.5 mm layers), 80 g; cream topstitching sewn along the back and right sides, scored groove on the front and left, 1.5 cm in | x 23.5 to 39.5, y 1.5 to 17.5 |

Mechanism (`task.py`, built with `pivotkit.py`, `mechkit.py` and `yamkit.py`):

- Handwheel: a free hinge with light drag (`mk.Free`). The needle phase is 120 deg minus the knob's angle above the
  front horizontal; the needle bar follows it (`mk.Follow`, kinematic). Pushing the front knob down turns the wheel's
  top toward the operator, the correct direction.
- Lifter: two detents (`mk.Detents`, each pulling the lever in from within 0.2 rad), the foot follows it (7 mm).
- Panel: `pivotkit.planar`, a turn about the needle's line then two slides. While the needle tip is at least 1 mm
  below the leather's underside (and the needle is over the panel), both slides' joint limits are narrowed to
  +/- 0.5 mm around where it went in, so the panel only turns about the needle. With the foot down the panel's damping
  rises 50-fold (it drags).

## Scoring (whole-run history, recorded each step by `success()`)

| Outcome | Score |
|---|---|
| Panel turned more than 8 deg in all with the needle out | 0, irreversible |
| Panel turned more than 8 deg in all with the foot down | 0, irreversible |
| Needle tip below the leather's top more than 2 mm from the corner (even part way in) | 0, irreversible |
| Panel pushed onto an already lowered needle (moved 1 mm or more in that step) | 0, irreversible |
| Panel forced more than 2 mm sideways with the needle in | 0, irreversible |
| A tool point within 3 cm of the lifter lever (arm or grip) or knob, or pad tips over the panel within 1.5 cm above it | 0 while so |
| Turned 90 +/- 8 deg, needle within 2 mm of the corner, foot down, both arms home and still for 1 s | 1.0 |
| The same, arms not home | 0.9 |
| Turned 90 +/- 8 deg, foot left up | 0.75 |
| Turned at least 45 deg about the needle at the corner | 0.5 |
| Foot raised with the needle in at the corner, and the needle still in there | 0.25 |
| Anything else, including doing nothing | 0 |

Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1.0.

Why these numbers:
- 8 deg turn tolerance: 1.8 cm over the 13 cm next side, which an operator still steers onto the groove in the first
  stitches; the same 8 deg is the most turning allowed off the needle or under the foot, so a turn the scorer could not
  tell from no turn isn't counted as one.
- 2 mm at the corner: half a 4 mm stitch; a needle hole further off shows in leather and can't be closed.
- 2 mm forced sideways: four times the 0.5 mm play the needle allows (the demo shows 0.85 mm), so only a real push
  against the needle counts.
- 1 mm through: the needle's point is fully through the leather and holds it. A hole, though, is made as soon as the
  tip is below the leather's top, so that is where a stray hole counts.
- 1 mm per step (2 cm/s): a panel moving faster than this in the step it meets an already lowered needle was pushed
  onto the point (a torn edge); a needle that comes down into a still panel, or into the corner, is the real move.
- Foot down: the lifter lever within its down detent's zone (q <= 0 rad, plus 0.002 rad, 20 times the lever's
  resting noise in the sim), where a released lever springs back onto the leather. A lever left between the detents
  holds the foot 2.8 mm or more up, so it is not down.
- 0.05 deg dead band: turns under the foot and off the needle are counted only once the panel moves 0.05 deg from
  where it was last counted (0.1 mm at the far corner), so the sim's resting jitter (under 0.005 deg a step) never
  adds up over a long run, while a slow real turn is still counted in full.
- 0.25 needs the needle still in: raising the needle again undoes the set-up, and a run put back to its start state
  should score like doing nothing. Lowering the foot after raising it doesn't undo it.
- 45 deg: halfway round, the least turn that is real progress.
- 3 cm / 1.5 cm hands-off: a tool point that close to the lifter lever (anywhere on its arm or grip) or the knob is
  still on it; pads that low over the panel are on it.
- Arms home: each tool point within 3 cm of where it started and still for the last 1 s, so arms that only pass
  through home as the run hits its time limit score 0.9.

In a real trial: film from above, read the turn on a protractor sheet under the bed, check the hole sits on the corner
of the stitching, and film the handwheel and lifter to see the needle was down and the foot up through the turn.

## How it differs from nearby board tasks

#678 (sew past pins) hand-feeds a framed seam past a fixed needle and pulls pins just in time; nothing there turns,
lowers the needle or works the foot. Here the scored skill is the corner-pivot procedure: work two machine controls
(handwheel, foot lifter) in the right order around a pivot turn about a pin that holds only while the needle is down.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, laws, scorer (`score()`, `success()`) and scripted demo (`demo()`). |
| `pivotkit.py` | A pin pivot that holds only while a condition holds (planar part on a turn + two slides; limits narrowed while in). |
| `yamkit.py` | Local copy with `Link.parent` (chained links), cylinder `axis`, law-set joint limits, inertia for collider-less links. |
| `mechkit.py` | Joint laws (Free, Follow, Detents). |
| `test_task.py`, `test_pivotkit.py` | Offline checks; no simulator needed. Each prints `all pass`. |
| `task.json` | The task card. |

## Run a trial

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/sewing-pivot-at-corner/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Turn the corner of the leather panel on the sewing machine, ready to sew its next side." \
      --scorer task_score --max-steps 1500

Model run (Dream-1): the same with `--policy dream --max-steps 2400`. Tests: `python test_task.py`, `python test_pivotkit.py`.

## The demo

1. Right arm, closed fingers: presses the crank knob down from 60 deg above to 58 deg below horizontal (the needle
   goes through the leather and down to the bottom of its stroke).
2. Right arm: grips the lifter grip and raises it to its upper detent (foot up), lets go and moves aside.
3. Left arm: presses its closed pads on the panel 10 cm from the needle and sweeps them 90 deg round the needle.
4. Right arm: presses the lifter grip back down to its lower detent (foot down). Both arms go home.

## Runtime and seeds

- Demo: 617 control steps (30.9 s simulated). Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

The leather is one stiff 3 mm panel that slides and turns flat on the bed on joints and can't bend, lift or tear. The needle
doesn't stitch or carry thread; while it is through the leather the panel's slides are locked to within 0.5 mm. The
foot's clamp is heavy damping, not friction. Feed dogs, motor and treadle are not modeled. The lifter, behind the head
on most machines, is moved to the front for reach, and the handwheel has stops at 100 deg above and 95 deg below
horizontal (half a turn), where a real one turns on.

## Real-world build

A real flatbed lockstitch machine (or a head set in a sewing table) with its motor unplugged, a 16 x 16 cm panel of
3 mm vegetable-tanned leather (or two glued 1.5 mm layers) topstitched along two sides and grooved on the other two, a
large knob bolted to the handwheel's face 3.3 cm from its center, and a sleeve pushed onto the hand lifter. Set the
needle 6.5 mm above the leather and the foot down before each trial. The YAM arms move slowly and the machine is off,
so the trial is safe; keep fingers clear of the needle.

## Licenses

All code here was written for this task; no third-party code or assets are included. Every shape is generated by
`yamkit.py`.
