# Install a piston through a ring compressor, holding the compressor down on the deck

A single-cylinder small-engine block (68 mm bore, the size of a 200 cc shop or generator engine) stands deck-up on the
bench. A new piston with three rings is started in the bore: its skirt is in the bore and its ring pack is squeezed
inside a pliers-type ring compressor, a steel band standing on the deck with its two red-gripped handles reaching
toward the robot's left. "Install the piston in the bore." An engine builder holds the compressor down flat on the
deck with one hand and drives the piston down through it with the other (in a shop by tapping the crown with a hammer
handle). If the band rides up off the deck while a ring is crossing from the band into the bore, that ring springs out
into the gap, catches on the deck's edge and the piston jams; pushed harder, the ring breaks. Driving the piston one
handed is the tempting wrong answer, and the band does ride up then, because each ring pushes outward on the band's
tapered lower lip as it is squeezed past it.

## Spec

Coordinates in cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0.

| Object | What it is | Where |
|---|---|---|
| Engine block | gray painted cast iron, 15 x 15 cm, 6 cm tall, machined deck on top, fixed to the table | centered x 26, y -17; deck at z 6 |
| Bore | 68 mm steel liner with a 1 mm chamfer at its mouth, 0.3 mm proud of the deck; head-bolt and dowel holes around it | axis x 26, y -17 |
| Piston | aluminum, 67.6 mm across, 5 cm tall, with rod (250 g), slides on the bore's axis; ring grooves 5 mm (top compression), 9 mm (second) and 13 mm (oil ring) below the crown; arrow on the crown | starts with the crown 2.2 cm above the deck (z 8.2), skirt in the bore |
| Ring compressor | pliers type: spring-steel band 1.8 cm tall around the ring pack (68.4 mm inside, rolled lip to 8 cm across), two steel handles locked squeezed by a ratchet bar, red vinyl grips on their outer 9 cm, 200 g; slides on the bore's axis | standing on the deck; handles from 4 to 19 cm out from the axis toward the robot's left, 30 degrees forward of the y axis (ends near x 35.5, y -0.5), 0.8 cm above the deck |

Mechanism (`PistonLaw`, `CompressorLaw` in `task.py`, through the yamkit law hook):

- Piston: a slide along the bore's axis from 4 cm above the deck down to its seat, the crown 10 mm below the deck,
  where the rod's big end sits on the crank journal. Gravity is off for it (ring friction is far more than its weight),
  so it stays where it is left; drag of 60 N·s/m in the band, plus 40 N·s/m for each ring in the bore.
- A ring is "crossing" while it is within 2.5 mm above the deck: squeezed in the band's lower lip and riding the bore's
  chamfer. Each crossing ring pushes the piston back up with 4 N (and adds 200 N·s/m of drag), so the driving hand
  has to push through it, and pushes the compressor up with 6 N, falling off at 600 N/m as the band rises off the
  ring. The band weighs 2 N, so a band nobody holds rides up about 6.7 mm and hangs there until the ring has gone in.
- A ring escapes if the band is more than 1 mm off the deck while that ring is in the gap between the deck and the
  band's lower edge, or crosses the deck plane during a step in which the band is up. The piston then jams where it
  is (a 20000 N/m hold: the ring caught on the deck's edge).

Scoring (whole-run history, recorded each step by `success()`; `SCORE_OVER = "end"`):

| Outcome | Score |
|---|---|
| Any ring escaped at any step (irreversible: pressing the band back down and pushing on doesn't undo it) | 0 |
| At the end a tool point is still on the work (within 21 cm of the bore's axis and lower than 6 cm above the band's top) | 0 |
| All three rings in the bore, crown within 2 mm of its seat, compressor resting on the deck, hands off, both arms home | 1.0 |
| The same, arms not home and still | 0.9 |
| Hands off, piston not home: 0.2 for each ring in cleanly | 0, 0.2, 0.4, 0.6 |

Possible scores: 0, 0.2, 0.4, 0.6, 0.9, 1.0. The compressor is left standing empty on the deck, released: a mechanic
unlocks it and lifts it off afterward, and lifting the empty band once all rings are in harms nothing.

Why these numbers:
- 1 mm lift: a top compression ring is about 1.2 mm tall, so a gap over 1 mm lets it spring out of the band.
- 2.5 mm crossing zone: the band's lower lip (about 1.5 mm) plus the bore's 1 mm chamfer.
- A ring is in once it is 1 mm below the deck, past the chamfer and wholly inside the bore.
- 2 mm home tolerance: half the 4 mm between ring grooves, and short of the 10 mm seat by less than the arm's
  tracking lag; the seat is where the rod's big end meets the crank journal.
- 21 cm and 6 cm hands-off zone: the handles reach 19 cm from the axis; a tool point inside it is on the piston,
  the band or the handles.
- 6 N lift and 4 N push-back: a small-engine top ring presses outward with roughly 10 to 20 N; on the band's
  15 to 20 degree lip that gives a few newtons up.
- Arms home: each tool point within 3 cm of where it started and still, held 1 s.

In a real trial: a dial indicator or a 1 mm feeler gauge under the band's edge at the handle side shows whether the
band lifted (or watch for a ring end showing above the deck); look into the bore for a broken or protruding ring, and
measure the crown's depth below the deck with a depth gauge.

## How it differs from nearby board tasks

- #995 (drive a bearing through its inner ring with a sleeve) is about which face carries the load; here the load path
  is fixed and the skill is one hand holding a loose guide flush while the other drives through it.
- #828 (find top dead center with a piston stop) works an assembled engine; #901 (slide a repair coupling) and #40
  (press a bearing past a detent) are one-handed slides with no guide that can lift.
- #692, #767, #618 hold a part with one hand while the other works, but nothing there is pushed off by the other
  hand's own force.

## Run

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/piston-through-ring-compressor/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Install the piston in the bore." --scorer task_score --max-steps 600 --log-dir ~/yam-eval

Dream-1: the same with `--policy dream --max-steps 2400`. Offline scorer checks: `python test_task.py` (prints
`all pass`). `PISTON_DEBUG=<file>` appends the crown, the band's lift and the gripper positions every step; `PISTON_DEMO_NOHOLD=1` runs the demo one-handed (checked in the sim: the band rides up 6.7 mm, the oil ring escapes, the piston jams, score 0).
Runtime per trial: about 25 s of sim time for the demo; Dream-1 runs 2,400 steps. No seeds (object_jitter = 0).

## Real-world build

- A single-cylinder small-engine short block (a GX200-class block with its crank and rod installed, head off), bolted
  deck-up to a plate clamped to the table, crank turned to bottom dead center.
- A new 68 mm piston and ring set on its rod, oiled, rings gapped and staggered.
- A pliers-type piston ring compressor (2 to 5 inch band) squeezed around the rings and locked by its ratchet, standing
  on the deck with its handles toward the robot's left.
- The robot pushes with its closed fingertips on the crown (a soft plastic pad on the fingertip protects the crown).
