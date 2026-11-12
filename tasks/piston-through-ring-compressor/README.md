# Drive a piston into its bore through a ring compressor while the other hand holds the compressor flat on the deck

A single-cylinder GX200-class short block (68 mm bore, 54 mm stroke, the 200 cc engine on shop equipment and
generators), head off, stands in a fixture on the bench with its bore vertical and its deck about 20 cm up. A new
piston with three rings is started in the bore: its skirt is in the bore and its ring pack is squeezed inside a
pliers-type ring compressor, a steel band standing on the deck with its two red-gripped handles reaching toward the
robot's left. "Install the piston in the bore." An engine builder holds the compressor down flat on the deck with one
hand and drives the piston down through it with the other (in a shop by tapping the crown with a hammer handle). If
the band rides up off the deck while a ring is crossing from the band into the bore, that ring can spring out into the
gap and catch on the deck's edge, jamming the piston; pushed harder, the ring can break. Driving the piston one handed is
the tempting wrong answer, and the band does ride up then, because each ring pushes outward on the band's tapered
lower lip as it is squeezed past it.

## Spec

Coordinates in cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0.

| Object | What it is | Where |
|---|---|---|
| Engine | GX200-class short block (68 mm bore, 54 mm stroke, 84 mm rod), head off, in a steel fixture (20 x 24 cm base plate) with its bore vertical: gray cast crankcase 15 x 17 cm and 11.5 cm tall, crankshaft's 19 mm end out toward the robot's right on the crank's axis, finned barrel, 13 x 13 cm deck flange with four head-bolt holes; crank turned 44 degrees past top dead center | centered x 26, y -17; crank axis at z 6; deck at z 19.75 |
| Bore | 68 mm steel liner with a 1 mm chamfer at its mouth, 0.3 mm proud of the deck; head-bolt and dowel holes around it | axis x 26, y -17 |
| Piston | aluminum, 67.6 mm across, 5 cm tall, on its rod (piston, pin and rod 250 g), slides on the bore's axis; ring grooves 5 mm (top compression), 9 mm (second) and 13 mm (oil ring) below the crown; arrow on the crown; the rod hangs on its pin with its cap off, not attached to the crank | starts with the crown 2.2 cm above the deck (z 21.95), skirt in the bore, the rod's big end 3.2 cm above the journal; seats with the crown 10.2 mm below the deck |
| Ring compressor | pliers type: spring-steel band 1.8 cm tall around the ring pack (68.4 mm inside, rolled lip to 8 cm across), two steel handles locked squeezed by a ratchet bar, red vinyl grips on their outer 9 cm, 200 g; slides on the bore's axis | standing on the deck; handles from 4 to 19 cm out from the axis toward the robot's left, 20 degrees back from the y axis (ends near x 19.5, y 0.9), 0.8 cm above the deck |

Mechanism (`PistonLaw`, `CompressorLaw` in `task.py`, through the yamkit law hook):

- Piston: a slide along the bore's axis from 4 cm above the deck down to its seat, the crown 10.2 mm below the deck,
  where the rod's big end (cap off) lands on the crank journal; the journal is a stop, and the rod's swing onto it
  isn't modeled. Gravity is off for it (ring friction is far more than its weight),
  so it stays where it is left; drag of 60 N·s/m in the band, plus 40 N·s/m for each ring in the bore.
- A ring is "crossing" while it is within 2.5 mm above the deck: squeezed in the band's lower lip and riding the bore's
  chamfer. The laws run once per 50 ms control step, so they test each ring's path over the last step and, at the
  piston's present speed, the next one: a fast stroke can't carry a ring through the zone unseen. Each crossing ring
  pushes the piston back up with 4 N (and adds 200 N·s/m of drag), so the driving hand
  has to push through it, and pushes the compressor up with 6 N, falling off at 600 N/m as the band rises off the
  ring. The band weighs 2 N, so a band nobody holds rides up about 6.7 mm and hangs there until the ring has gone in.
- A ring escapes when it is out of the band and not in the bore: its path during a step passes through the gap
  between the deck and the band's lower edge while the band is more than 1 mm off the deck, or the piston is pulled
  up until the ring's middle is past the band's top edge. Speed alone is never an escape: a ring driven fast from the
  band into the bore with the band held flat is in. The sim then holds the piston where it is (a 20000 N/m hold,
  standing in for a ring caught on the deck's edge or the band's top).

Scoring (whole-run history, recorded each step by `success()`; `SCORE_OVER = "end"`):

| Outcome | Score |
|---|---|
| Doing nothing | 0 |
| Any ring escaped at any step (irreversible: pressing the band back down and pushing on doesn't undo it) | 0 |
| All three rings in the bore, crown within 2 mm of its seat, compressor resting on the deck, both hands let go (no gripper touching the piston, the band or its handles) | 1.0 |
| Otherwise (piston not home, or a hand still on the work at the end): 0.2 for each ring in cleanly | 0, 0.2, 0.4, 0.6 |

Possible scores: 0, 0.2, 0.4, 0.6, 1.0. The compressor is left standing empty on the deck, released: a mechanic
unlocks it and lifts it off afterward, and lifting the empty band once all rings are in harms nothing.

Why these numbers:
- 1 mm lift: a top compression ring is about 1.2 mm tall, so a gap over 1 mm lets it spring out of the band.
- 2.5 mm crossing zone: the band's lower lip (about 1.5 mm) plus the bore's 1 mm chamfer.
- A ring is in once it is 1 mm below the deck, past the chamfer and wholly inside the bore.
- 2 mm home tolerance: half the 4 mm between ring grooves; the seat is where the rod's big end meets the crank
  journal.
- Let go, judged by contact within 2 mm (`touching()` in the local `guardkit.py`): a gripper is on the work when its
  own shape (the palm and both fingers, as boxes fitted to the YAM's collision meshes, at the fingers' present
  opening) comes within 2 mm of the piston, the band or the handles. The 2 mm allows for the solver's resting gap and
  the box fit, so it is contact, not nearness: closed fingertips 3 mm over the crown, a fist 5 mm over the handles or
  an open hand 1 cm beside the band have let go. A real trial sees it in the video: both hands off the piston and the
  compressor at the end.
- Escape is a proxy: the sim models the rings' push on the band, not the ring springing out, so it counts a ring
  whose path crosses a gap over 1 mm under the band. A real trial checks the outcome: the band's lift with the feeler
  gauge or dial indicator, and any ring caught on the deck's edge, protruding or broken, by looking into the bore.
- A hand still on the work at the end caps the score at the per-ring credit (at most 0.6): the job ends with the
  piston seated and the compressor left standing free for the mechanic to unlock and lift off, and only then is it
  known that the band sits flat on its own. The rings that are credited stay in when the hand lets go: each is wholly
  in the bore, 1 mm below the deck, where neither the band's lift nor the piston's push-back can bring it back, and a
  ring still crossing under a band held flat is not credited.
- 0.2 per ring: each ring through the hard step without escaping is work the mechanic doesn't redo, but with the
  piston short of its seat the rod cap can't go on, so three rings in earn 0.6, well short of the finished 1.0.
- 6 N lift and 4 N push-back: these are estimates, not measurements. Our rough estimate is that a small-engine top
  ring presses outward with 10 to 20 N; on the band's 15 to 20 degree lip that gives a few newtons up. The real
  trial does not depend on them: it checks the band's lift and the rings directly.

In a real trial: a dial indicator or a 1 mm feeler gauge under the band's edge at the handle side shows whether the
band lifted (or watch for a ring end showing above the deck); look into the bore for a broken or protruding ring, and
measure the crown's depth below the deck with a depth gauge.

Why the deck is 19.75 cm up and the crank is 44 degrees past top dead center: the deck sits above the crank's axis by
half the stroke (27 mm), the rod (84 mm), the crown's height over the pin (26 mm) and the crown's 0.5 mm below the
deck at top dead center, 137.5 mm in all, and the crank's axis is 6 cm above the table so the counterweights clear the
crankcase floor. With the crank 44 degrees past top dead center the piston is 9.7 mm below its top position when the
big end meets the journal, so the crown seats 10.2 mm below the deck with all three rings 15 to 23 mm into the bore,
and at the start the big end hangs 3.2 cm clear of the journal. Shops usually turn the journal to bottom dead center
instead, where the crown seats 54.5 mm down the bore and is reached with a hammer handle; the robot pushes with its
fingertips, which reach about a centimeter below the deck, so the crank is set where that push lands the rod on
the journal. The rod cap goes on afterward, with the crankcase's side cover off. The deck height is near the
top of the arms' reach (they lift about 22 cm above the table), so the demo leans both grips 14 degrees.

## Category

Impact on jobs: automotive service technicians and mechanics (about 800,000 US jobs, BLS) and small-engine mechanics
fit pistons through a ring compressor in every engine rebuild, a standard job in engine repair.

## How it differs from nearby board tasks

- Driving a bearing through its inner ring with a sleeve is about which face carries the load; here the load path is
  fixed and the skill is one hand holding a loose guide flush while the other drives through it.
- Finding top dead center with a piston stop works an assembled engine; sliding a repair coupling and pressing a
  bearing past a detent are one-handed slides with no guide that can lift.
- Tasks that hold a part with one hand while the other works have nothing pushed off by the other hand's own force.

## Run

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/piston-through-ring-compressor/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Install the piston in the bore." --scorer task_score --max-steps 600

Dream-1: the same with `--policy dream --max-steps 2400`. Offline scorer checks: `python test_task.py` (prints
`all pass`). A one-handed run of the demo (the left arm left at home) was checked in the sim: the band rides up
6.9 mm, the oil ring escapes, the piston jams, score 0.
Runtime per trial: the demo finishes in 383 steps (19.2 s at 20 Hz); Dream-1 runs 2,400 steps (2 minutes). No seeds
(object_jitter = 0).

## Real-world build

- A GX200-class short block (68 mm bore, 54 mm stroke, 84 mm rod) with its crankshaft installed and its head off,
  bolted to a 20 x 24 cm steel base plate clamped to the table, on a wedge spacer so its bore is vertical (the
  GX200's cylinder is inclined 25 degrees on its own base). The crank's axis is 6 cm above the table and the deck
  19.75 cm; check the deck with a height gauge and shim the spacer to match. Set the crank 44 degrees past top dead
  center with a degree wheel on its end, then take the wheel off.
- A new 68 mm piston and ring set on its rod (rod cap and its bolts off), oiled, rings gapped and staggered; confirm
  on the purchased piston that the pin is 26 mm below the crown.
- A pliers-type piston ring compressor (2 to 5 inch band) squeezed around the rings and locked by its ratchet, standing
  on the deck with its handles toward the robot's left.
- The robot pushes with its closed fingertips on the crown (a soft plastic pad on the fingertip protects the crown).
