# Rout a curved edge on a router table: brace on the starting pin, pivot onto the bearing, then feed

A router table carries a bearing-guided rabbeting bit and a steel starting pin in its insert plate, 5 cm to the
bit's right. A cherry board with one long edge band-sawn to a gentle curve lies on the table to the right of the
bit, its curve toward it. "Rout the rabbet along the board's curved edge on the router table, then leave the board
on the table past the bit." With no fence to steady a curved part, a cabinetmaker first braces the board against the
starting pin, pivots it about the pin into the spinning bit until its edge rides the bearing, and only then comes off
the pin and feeds it right to left round the bearing, against the bit's rotation, until it is clear of the bit. Fed
straight into the bit without the pin, the cutter's first bite grabs the board and throws it, or pulls the hands
toward the cutter.

Run: `~/yam-tools/run_task.sh tasks/router-table-starting-pin demo 900` (scripted demo), `... dream 2400` (Dream-1).
Scripted demo (left arm): slides the board 2.9 cm onto the pin, pivots it 13.8 deg about the pin onto the bearing, feeds it round the bearing (21.4 cm routed) and past the bit, lets go and goes home: 1.0 in 692 steps.
Offline scorer tests: `python test_task.py` (prints "all pass"). One trial is 2,400 steps at most (2 minutes at 20 Hz);
object jitter 0, one fixed layout.

## Spec

Coordinates in cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0.

| Object | What it is | Where |
|---|---|---|
| Router table top | the robot's table, a light gray laminate area 40 x 62 cm | x 9 to 49, y -27 to 35 |
| Insert plate | black phenolic, 23.5 x 30 cm (9-1/4 x 11-3/4 in), 1 mm proud in the drawing, centered on the bit | x 21.3 to 44.8, y -9 to 21 |
| Bit | 3/8 in rabbeting bit: cutter 3.18 cm across (1-1/4 in), standing 6.4 mm above the table, red coated; 1/2 in ball bearing (1.27 cm across) from 6.6 to 11.4 mm above the table; turns counterclockwise seen from above | axis x 33, y 6 |
| Starting pin | steel, 6.4 mm across (1/4 in), 3 cm tall, screwed into the insert plate | x 33.75, y 1 |
| Board | cherry, 25 x 15 cm, 1.27 cm thick (1/2 in), 220 g; one long side band-sawn to a 30 cm radius (2.7 cm of bow), toward the bit | curve's ends at (33.5, -18.1) and (27.4, 6.2); back corners (21.6, -21.0) and (15.5, 3.2): turned 14 deg from square, 2 cm off the pin |

Mechanism (`task.py`, built with `pivotkit.py` and `yamkit.py`, local copies):

- The board is `pivotkit.planar`: a turn about the pin's axis, then two slides, carried flat 1 mm above the table.
  While it is braced (any part of it within 1.5 mm of the pin's surface, its curve not yet on the bearing) both slides
  are locked within 0.5 mm, so it can only turn about the pin. Once its curve rides the bearing it slides and turns
  freely. Its drag on the table is damping: 4 N s/m sliding (0.2 N at 5 cm/s), applied at the board's center (local
  pivotkit addition `slide_at`: PhysX applies a slide's damping at its joint frame); its turn about the pin is damped
  as dragging the board round the pin costs (4 N s/m times the center's distance from the pin squared plus the
  board's radius of gyration squared). Off the pin that turn is damped 10 times harder, because per-joint damping
  can't couple the far-off turn with the slides and a push at the center would otherwise swing the board round the
  pin instead of sliding it; the board then turns stiffer than a real one.
- The bearing and the pin are a separate fixed prop with real colliders, so the board stops on them. The bearing's
  collider is frictionless (local yamkit addition `Collider.slick`), since a real bearing rolls along the edge. The cutter has no
  collider: the board passes into it, which is how the scorer sees a cut. The board's curve collider is 26 chord boxes
  1 cm long, each within 0.04 mm of the true curve (local yamkit addition: yawed box colliders).
- The routed stretch is drawn as a pale band of fresh-cut cherry on the board's top along the curve, as wide as the
  rabbet (visual only; the real rabbet is on the board's underside).

## Scoring (whole-run history, recorded each step by `success()`)

| Outcome | Score |
|---|---|
| Any part of the board meets the cutter (within its 15.9 mm radius) other than by being turned into it about the pin while braced on it: fed into the bit off the pin, shoved onto pin and bit in one step, or pushed back into the bit after leaving it | 0, irreversible |
| A tool point within 7.6 cm (3 in) of the bit's axis with the pads within 5 cm of the table | 0, irreversible |
| Pad tips over the board (grown 1 cm) within 1.5 cm above its top at the end | 0 while so |
| At least 18 cm of the curve routed, the board 5 mm clear of the cutter with its trailing end past the bit, both arms home and still for 1 s | 1.0 |
| The same, arms not home | 0.9 |
| At least 18 cm routed, the board left on or before the bit | 0.75 |
| At least 9 cm routed | 0.5 |
| The curve brought onto the bearing from the pin, not fed round | 0.25 |
| Anything else, including doing nothing | 0 |

Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1.0.

Routed length: the curve's 2 mm stretches that the bearing passed over (the curve within 1.5 mm of the bearing) in a
cut begun on the pin, while fed right to left (the contact moving from the leading end toward the trailing end).
Feeding the other way, a climb cut, routes nothing.

Why these numbers:
- 15.9 mm: the cutter's radius (a 1-1/4 in rabbeting bit). Any contact with it is a cut; the first one must come
  while braced on the pin, the only motion there being the turn about it.
- 1.5 mm braced and on the bearing: the play of a hand-held board against a 1/4 in pin or a 1/2 in bearing; the
  colliders keep the board from coming closer, so within 1.5 mm is touching.
- 18 cm: 70% of the 25.8 cm curve. A start braced on this pin, 5 cm from the bit, enters the curve 3 to 7 cm in from
  its leading end (4.1 cm in the demo); that short stretch is routed in a second pass from the other end, and the
  rest of the curve must be routed in one feed. 9 cm is half of it, real progress round the bearing.
- 5 mm clear, trailing end past the bit: the board fed on through and off the bit, not left touching the cutter.
- 7.6 cm: router-table manuals' three-inch rule for hands near a bit; 5 cm above the table is well above the bit
  (1.2 cm) and the board's top (1.4 cm), so arms passing high over it are fine.
- 1.5 mm above the board: pads that low over it are still on it.
- Arms home: each tool point within 3 cm of where it started and still for the last 1 s.

A real trial: the router unplugged and its bit swapped for a dummy, filmed from above; the board's edge
pencil-marked every centimeter; check from the video that the board touched the pin before it crossed the painted
cutter circle and stayed on it until the edge met the bearing, then read off the marks that passed the bearing.

## Real-world build

A bench-top router table top (or 19 mm laminated MDF with a 9-1/4 x 11-3/4 in phenolic insert plate), the plate's
1/4 in steel starting pin, and a 1/2 in cherry board 25 x 15 cm band-sawn to a 30 cm radius on one long side. In
place of the bit, a dummy: a 1/2 in ball bearing on a 1/4 in bolt in the router's collet, its bottom 6.6 mm above the
table, with the 1-1/4 in cutter's circle painted on the insert plate around it. A real rabbeting bit (e.g. a Freud
32-104) can't be used in robot trials: standing still, its carbide wings stop the board's lower half 9.5 mm short of
the bearing, so the edge could never ride it, and its edges are sharp next to the grippers. With the dummy nothing is
cut, so one board serves every trial.
