# Roll puff pastry into an even 3 mm sheet on a pin with spacer rings

A block of puff pastry on a floured maple pastry board, and a wooden rolling pin with 3 mm spacer rings on both ends of
its barrel. "Roll this puff pastry out into a 3 mm sheet at least 20 cm long." The rings only set the thickness if
both of them ride on the board, so the pin is driven by both handles and kept level: with one end lifted the barrel
leaves a wedge, thick on that side. The pastry springs back, so it takes several passes. And a baker rolls from the
middle out to each end and stops short of the edge: rolled off the end, the cut edge is pressed thin and puff
pastry's layers seal there, so that edge will not rise.

Run (from the repo root, Isaac Lab-Arena installed):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/roll-pastry-sheet-on-spacer-rings/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Roll this puff pastry out into a 3 mm sheet at least 20 cm long." --scorer task_score --max-steps 1300
    python tasks/roll-pastry-sheet-on-spacer-rings/test_task.py      # offline model and scorer checks, prints "all pass"

Use `--policy dream --max-steps 2400` for a Dream-1 trial. The scene is deterministic (no seeds, object_jitter=0).
Set `DOUGHKIT_DEBUG=1` to print, each step, the barrel's line (x, gap in mm) over the outermost strips, the sheet's
ends and its thinnest and thickest cell.

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Pastry board | oiled maple (mid brown), 31 (x) x 28 (y) x 1.5 cm, floured (a wooden pin slides on it with friction 0.15 static, 0.10 dynamic); fixed | x 14 to 45, y -14 to 14, top z 1.5 |
| Puff pastry | a chilled block 6.5 (x) x 20 (y) x 1.0 cm (130 mL, about 130 g); modeled, no collider | centered x 30, y 0 |
| Rolling pin | wood, 43 cm overall, 450 g: barrel 5 cm across and 26 cm long; on each end of the barrel a blue silicone spacer ring 5.6 cm across (3 mm proud of the barrel) and 1.5 cm wide (\|y\| 11.5 to 13); handles 2.5 cm across and 8.5 cm long (\|y\| 13 to 21.5) | in its rest behind the pastry, axis along y at x 41.5, z 5 (rings 7 mm above the board) |
| Pin rest | two dark wooden cradles, fixed on the table just outside the board, one under the outer end of each handle: a block 4.5 (x) x 2 (y) cm whose top carries the handle at z 3.75, with a 5 mm lip 1 cm tall in front and behind | centered x 41.5, \|y\| 19.5 to 21.5 |

Model (`doughkit.py`): the sheet is 20 strips along x, one per 1 cm of width; each strip is a chain of slices (2.5 mm
long at the start) with a fixed volume each, so a slice's length is its volume over its thickness and the total volume
never changes.

| Quantity | Value | Why |
|---|---|---|
| Gap | the barrel's lowest line above the board where it crosses each strip, from the pin's pose | 3 mm with both rings on the board; one ring lifted 4 mm (0.9 degrees) gives about 3.5 mm at one edge of the pastry and 6.5 mm at the other |
| Press | a slice the barrel rolls across with the gap below its top goes to gap + 0.5 x (thickness - gap), once per crossing | the dough springs back: half the excess per pass is an estimate (measure on the real build: thickness after one pass of a 10 mm block on 3 mm rings). From 10 mm it takes four presses to come within 0.5 mm of the gap |
| Contact | a slice is pressed as the barrel's face reaches its top, sqrt(2 R (t - gap)) ahead of the lowest line (R = 2.5 cm) | a round barrel meets thick pastry ahead of its lowest point (19 mm ahead on the 10 mm block, 5 mm on a 3.5 mm sheet), so stopping a little short of an end still thins it |
| Lengthening | a pressed slice grows longer; the pastry beyond it, away from the block's middle, moves out | the middle of the block is held by its own weight on the board; the sheet grows at both ends |
| Rolled off an end | the barrel's line more than 5 mm past a strip's end while pressing, and still past it the next step (a one-step pop does not count): that end's last 1 cm is pressed to 0.6 x the gap (1.8 mm) and marked sealed, for good | 5 mm is about the barrel's contact patch on a 3.5 mm sheet, so the pin has left the pastry; the edge it drops off is pressed and dragged against the board (Food Republic, on puff pastry: flattened or crimped edges have difficulty rising) |
| Pin set down | axis level within 3 degrees, and back in its rest (axis within 5 mm of the seat height, within 1 cm of the rest's x) or lying on its rings on the board (their bottoms within 5 mm of it) | resting, not held in the air; a round pin left on the board rolls, hence the rest |
| Both handles | presses and roll-offs count only while each handle has a hand closed on it (fingers stopped by the 2.5 cm handle, tool point within 1 cm of it) | the pin is the standard handled type: its barrel turns on an axle between the handles, so it rolls over the pastry only when driven by them |
| Torn | pressing pastry, two steps in a row, with a tool point or pad tip within 1 cm of the barrel or rings: latched | a hand on the barrel stops it turning; the skidding barrel drags and tears laminated pastry |
| Let go | every tool point and pad tip 1 cm clear of the pin's barrel, rings and handles | a hand closed or open around a handle has not set it down |

Visuals (markers, no physics, never scored): one slab per covered 1 cm cell, as thick as the pastry there, colored by
thickness band: cream (thicker than 6.5 mm), pale gold (4.5 to 6.5), light gold (3.5 to 4.5), golden (2.5 to 3.5, on
spec), tan (2.0 to 2.5) and dark brown (under 2 mm, a sealed edge). The top camera shows the outline growing and any
thick band or wedge.

## Scoring

The pastry is read on the board's 1 cm grid (rows: the 20 strips; columns: 1 cm along x).

| Outcome | Score |
|---|---|
| The pin still held or touched (a tool point or pad tip within 1 cm of it), or not set down level (in its rest, or on its rings on the board) | 0 |
| An end rolled over (sealed), or the pastry torn (pressed with a hand on the barrel), whatever else was done | 0 |
| Every strip at least 20 cm long, at least 90 % of the sheet's cells 3 mm within 0.5 mm, pin set down, hands clear, both arms home and still | 1.0 |
| The same, arms not home and still (each tool point within 3 cm of where it started) | 0.9 |
| Every strip at least 18 cm long, but short of 20 cm or fewer than 90 % of cells on spec (about one pass to go) | 0.5 |
| Any strip shorter than 18 cm (nothing done, one or two passes, a wedge from a tilted pin) | 0 |

Possible scores: 0, 0.5, 0.9, 1.0. Doing nothing scores 0.

Why the numbers: 20 cm is the floor of a 20 cm (8 inch) square pan. 3 mm within 0.5 mm: the ring's 3 mm (sold as 0.125
inch) plus half a millimeter of springback a baker accepts; puff pastry much thicker in one place bakes raw there, much
thinner burns. 90 %: a 21 cm sheet loses about 1 cm at each end to trimming (10 %). 18 cm (90 % of the length) is one
pass short. The 5 mm and 3-degree pin checks are resting tolerances for a 5.6 cm ring on a flat board.

Volume ties length to thickness: the 130 mL block makes 21.7 cm at exactly 3 mm, so a sheet 20 cm long averages at most
3.25 mm. A tilted pin leaves the high side thick, and that strip short (one ring 4 mm up, eight passes: the outermost strip on
that side averages 5.6 mm and is 11.6 cm long), so a wedge scores 0, not 0.5.

The demo: both arms grip the pin's handles from above (fingers closing along x), lift it out of its rest, over the
middle of the block, and set it down on its rings, then five passes, each from the middle out to the far end and back,
then out to the near end and back, aimed 4 mm short of the end as it then lies (worked out with doughkit; the ends move
out as the sheet grows, and the pin trails the hands by about 1 cm at each turn, so it stops about 1.4 cm short); then
the pin is lifted off the pastry and put back in its rest, let go, home. In the sim it scores 1.0 at step 1,260 (sheet
20.5 cm, no end sealed; both rings on the board through every stroke, the far-end turns lifting the right ring at most
0.5 mm).

Offline (test_task.py, the same strokes without the arm's lag): three passes leave 19.8 cm with 85 % on spec (0.5), four
make 21.1 cm and 100 % on spec (1.0). The same strokes at 0.04 to 1 m/s give the same sheet. Mistakes: one end 4 mm high
(one hand) 0; straight off the end 0; a quick 1.2 cm overshoot past the end and straight back 0 (sealed), a one-step 7 mm pop not sealed; one gripper
closed on the middle of the barrel (pin level by symmetry) 0 (nothing counts, torn); one hand on a handle and one on the
barrel 0; open hands pushing the handles, nothing pressed; pin hung in
the air at the end 0; hands still on the handles, or resting on the barrel, 0.

Run in the sim: three passes 0.5 (19.3 cm, nothing sealed); doing nothing 0; one pass 0 (11.6 cm); full-length strokes over both ends from the start 0
(120 cells sealed, though the sheet reached 24.5 cm); one arm rolling by the left handle 0 (the pin skews 3.6 cm and its
free end rides 4 to 10 mm up: a wedge, sealed ends, the pin left tilted); one arm closed on the middle of the barrel 0 (nothing
counts without both handles held, and pressing with a hand on the barrel tears the pastry).

In a real trial: lay a 1 cm grid transparency over the sheet and photograph it from above to read its length per 1 cm
strip (a ruler along both long edges and the middle is enough: every strip must reach 20 cm). Read thickness with a
needle depth gauge pushed through the pastry to the board at the center of each 2 cm square (about 100 points; a cell
is on spec at 2.5 to 3.5 mm), and along the last 1 cm at each end (under 2 mm there, or pressed flat and shiny, is a
sealed edge: 0).

## Real-world build

- A maple pastry board about 31 x 28 x 1.5 cm, floured.
- A wooden rolling pin with handles, barrel 5 cm across, and a pair of silicone spacer rings 3 mm (0.125 inch) proud of
  the barrel (rolling-pin ring sets such as Regency's four pairs include 0.0625, 0.125, 0.25 and 0.375 inch).
- Store-bought puff pastry, cut and pressed into a 6.5 x 20 x 1 cm block (130 g); one block per trial. Roll it at
  about 18 C (20 to 30 minutes out of the fridge, when a fingertip leaves a dent): straight from the fridge, laminated
  pastry takes a baker leaning on the pin, more than the arms can push, while at 18 C the pin's weight and a light
  press from each hand thin it. If the first build shows the arms stalling on the 10 mm block, start from a 7 mm
  block 9.3 cm long (the same 130 mL) and say so in the setup.
- A 1 cm grid transparency, a needle depth gauge (or a toothpick marked in mm), a ruler, a phone camera.

## What the task tests

Bimanual control of one tool's level: the thickness is set only while both spacer rings ride the board, so two hands
must keep a 43 cm pin level as it travels. The work changes under the tool: the sheet lengthens and springs back, so
it takes several passes and the ends move out as it grows, and each stroke must stop short of where the end now
lies. Sealing an end is irreversible. Roller and flattening tasks that score coverage, or hold a springy sheet flat at
a fixed size, test none of these.
