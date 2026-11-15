# Roll store-bought puff pastry to an even 1/8 inch with a spacer-ring rolling pin

Half a sheet of store-bought puff pastry, thawed cold and unfolded on a floured maple board, and a handled maple rolling
pin with 1/8 inch silicone spacer rings on both ends of its barrel, lying on the board behind the pastry. "Roll this
puff pastry out to an even 1/8 inch (3.2 mm) thick." The pastry starts at the maker's 3/16 inch (4.8 mm). The rings set
the thickness only while both of them ride on the board, so the pin has to be kept level along its length: with one
end lifted the barrel leaves a wedge, thick on that side. The pastry springs back, so it takes several passes.
Category: Societal relatability. Rolling dough out with a rolling pin, for a pie crust, cookies or pastry, is something
most people have done at home; store-bought puff pastry sheets are a supermarket staple, and spacer rings that slip
onto a rolling pin are sold by ordinary kitchen retailers for an even thickness.

Run (from the repo root, Isaac Lab-Arena installed):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/roll-pastry-sheet-on-spacer-rings/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Roll this puff pastry out to an even 1/8 inch (3.2 mm) thick." --scorer task_score --max-steps 1300
    python tasks/roll-pastry-sheet-on-spacer-rings/test_task.py      # offline model and scorer checks, prints "all pass"

Use `--policy dream --max-steps 3000` for a Dream-1 trial (the run limit). The scene is deterministic (no seeds, object_jitter=0).
Runtime per trial: the demo's waypoints add up to about 50 s (about 1,000 steps at 20 Hz); a Dream-1 trial runs up to
3,000 steps (2.5 minutes).

## The pastry: what people buy, and what it is rolled to

- **The product.** The puff pastry most US supermarkets stock is Pepperidge Farm Puff Pastry Sheets: a 17.3 oz (490 g)
  frozen box of two sheets, each about 245 g, each folded in thirds (sold at Target, ShopRite, Stop & Shop, Giant,
  Hannaford, Publix and others). The maker gives the frozen sheet as about 9.75 x 10.5 inches and **3/16 inch (4.8 mm)
  thick**, folded in thirds (puffpastry.com FAQ, Pepperidge Farm's puff pastry site). America's Test Kitchen measured
  the sheets at 9.5 x 9 inches (24 x 23 cm) in its taste test of store-bought puff pastry. The other US products are
  less typical: Trader Joe's All Butter Puff Pastry (two 12.5 x 10 inch sheets, 18 oz) is seasonal, sold only in late
  fall and early winter; Dufour Classic (one 14 x 9 inch sheet, 14 oz, about $12) is mostly in specialty stores (both
  from the same ATK test). Block formats (Jus-Rol 500 g in the UK, Tenderflake in Canada) are not what a US shopper
  finds, so the task no longer starts from a block.
- **The start.** One sheet as the box directs: thawed until it unfolds easily, still cold, unfolded on a lightly floured
  surface (the package directions, quoted on the retailers' pages). Cut in half with a knife, it gives two pieces 11.5 x
  24 cm, 4.8 mm thick; one is the start of a trial (about 122 g, 132.5 mL). The whole sheet is not used: rolled to
  3.2 mm it becomes about 38 x 25 cm, longer than the arms can roll from one side of the board (see "Reach" below).
- **The target.** 1/8 inch (3.2 mm). Pepperidge Farm's own recipes roll one sheet "into a 15 x 10-inch rectangle"
  (Puff Pastry Pepperoni Pizza, pepperidgefarm.com): from about 9.75 x 10.5 inches at 3/16 inch, that is about 1/8 inch
  (4.8 mm x 102 / 150 = 3.3 mm). 1/8 inch is also the spacer ring sold for it (below). So there is real rolling to do: a
  third of the thickness comes off.
- **Cold.** The package says the thawed sheet "should still be cold", to thaw it no more than 40 minutes, and if the
  sheets get warm or sticky to put them in the freezer for 5 minutes or the refrigerator for 10 to 15. A real trial
  rolls the piece as soon as it is cut, and chills it as the package says if it turns sticky before the trial starts.

Volume (rolling keeps it): 11.5 x 24 x 0.48 = 132.5 mL. At 3.2 mm over the same 24 cm width that is 17.3 cm long
(132.5 / (24 x 0.32)); at the on-spec band's ends, 15.8 cm at 3.5 mm and 19.0 cm at 2.9 mm. Centered at x 24 cm, even
the 19.0 cm sheet runs x 14.5 to 33.5, on the 30 cm board (x 10 to 40) and inside the strokes' reach. The demo's sheet
ends 17.2 cm long (x 15.4 to 32.6) at 3.2 to 3.3 mm. A real sheet also spreads sideways a little as it is rolled, which
makes it shorter, not longer; the rings sit 1.75 cm outside each long side of the 24 cm piece (27.5 cm between them),
so a sheet up to 27.5 cm wide still rolls between them. The score has no length rule, so the volume never caps it:
the target thickness is reachable whatever the length.

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Pastry board | oiled maple (mid brown), 30 (x) x 40 (y) x 1.5 cm (a 12 x 16 inch board), floured; fixed | x 10 to 40, y -20 to 20, top z 1.5 |
| Puff pastry | half of a thawed Pepperidge Farm sheet: 11.5 (x) x 24 (y) cm, 4.8 mm (3/16 inch) thick, 132.5 mL, about 122 g; modeled, no collider | centered x 24, y 0 |
| Rolling pin | maple, a handled pin with a 12 inch barrel (Frieling Classic Rolling Pin with Handles: 2.4 inch barrel, 12 inch barrel, 21 inch overall, 1.6 lb): barrel 6.1 cm across and 30.5 cm long; on each end of the barrel a blue silicone 1/8 inch spacer ring 6.74 cm across (3.2 mm proud of the barrel) and 1.5 cm wide (\|y\| 13.75 to 15.25); handles 2.5 cm across and 11.4 cm long (\|y\| 15.25 to 26.65); 0.73 kg | lying on its rings on the board behind the pastry, axis along y at x 35, z 4.87 |

The pin lies on the board where a cook leaves it, not in a rest. Its barrel's front is 2.2 cm behind the pastry's far
edge, and its handles reach past the board's sides (they are thinner than the rings, so they touch nothing).

Model (`doughkit.py`): the sheet is 12 strips along x, one per 2 cm of width; each strip is a chain of slices (2.5 mm
long at the start) with a fixed volume each, so a slice's length is its volume over its thickness and the total volume
never changes.

| Quantity | Value | Why |
|---|---|---|
| Gap | the barrel's lowest line above the board where it crosses each strip, from the pin's pose | 3.2 mm with both rings on the board; one ring lifted 1 mm (0.2 degrees over the 29 cm between the rings) gives about 3.3 mm at one edge of the pastry and 4.1 mm at the other |
| Press | a slice the barrel rolls across with the gap below its top goes to gap + 0.5 x (thickness - gap), once per crossing | the dough springs back: half the excess per crossing is an estimate (measure on the real build: thickness after one crossing). From 4.8 mm: 4.0, 3.6, 3.4, 3.3 mm, so three crossings put a point on spec |
| Contact | a slice is pressed as the barrel's face reaches its top, sqrt(2 R h - h^2) ahead of the lowest line (h = t - gap, R = 3.05 cm) | a round barrel meets thick pastry ahead of its lowest point (10 mm ahead on the 4.8 mm sheet) |
| Lengthening | a pressed slice grows longer; the pastry beyond it, away from the piece's middle, moves out | the middle is held by its own weight on the board; the sheet grows at both ends. Sideways spread is not modeled |
| Rolled past an end | the end slice is pressed like any other the barrel crosses | the rings hold the barrel 3.2 mm up there too, so nothing goes thinner than the gap; no damage is assumed |
| Pin set down | axis level within 3 degrees, lying on its rings on the board or on the table (their bottoms within 5 mm of it) | resting, not held in the air or propped on something |
| Both hands on the pin (sim only, a stand-in) | presses count only while both grippers touch the pin: each gripper's own shape (palm and fingers, boxes fitted to the YAM's collision meshes, at their present opening) within 2 mm of a handle, the barrel or a ring (flat-ended cylinders as the colliders), whether it grips a handle or presses on the barrel with a closed hand | the sim's pin slides on its rings instead of turning, so the model cannot tell rolling from pushing the pin along; two hands on the pin can hold both ends down, so both rings can stay on the board. The instruction names no grip, so handles and palms on the barrel both count. A hand hovering over the pin does not count. A stroke with one hand or none on the pin is not penalized, and nothing is assumed torn: that stroke just does not count. A real trial has no hand rule: it scores the measured sheet and the final pin state |
| Let go | no gripper touching the pin's barrel, rings or handles (its own shape more than 2 mm away) | a hand still on the pin has not set it down; an open hand near the pin but not touching it has |
| Board friction | 0.15 static, 0.10 dynamic (ring on floured board) | a stand-in for a barrel that turns. The named pin's barrel turns on nylon bushings around its axle, so a real pin rolls under still handles and its travel is resisted by rolling, not sliding. The sim's gripped pin cannot turn and slides on its rings instead, so its sliding friction is set low: at 0.10, the 0.73 kg pin pressed down with 10 N drags with about 1.7 N. No measured coefficient is claimed |

What the model stands in for, and what a real trial checks instead: the thickness comes from the press rule above,
which follows the real behavior (the rings set the floor, the dough springs back by a share of the excess, volume is
kept). The springback share is an estimate, so a real trial measures the sheet itself. The model cannot tear the sheet,
stick it to the pin or press it below 3.2 mm; in the sim a stroke without both hands on the pin simply does not count.

The dough model also gives the pin no rolling resistance: the pastry pushes back on nothing, so in the sim the rings
keep the pin level on the flat board however it is held, and the robot only has to drive it back and forth. Cold puff
pastry pushes back hardest under the thicker side, so a pin pressed harder by one hand can ride up on the other; the
sim cannot show that. The real trial checks the result directly: the bands' average thicknesses across the width, at
the same points the sim reads.

Visuals (markers, no physics, never scored): one slab per covered 2 cm cell, as thick as the pastry there, colored by
thickness band: cream (thicker than 4.2 mm, the start), pale gold (3.7 to 4.2), light gold (3.5 to 3.7), golden (2.9 to
3.5, on spec), tan (2.5 to 2.9) and dark brown (under 2.5 mm). Tan and dark brown never appear in the sim: the rings
hold the barrel 3.2 mm up, so the model cannot press thinner.

## Scoring

The pastry's thickness is read at fixed points: the centers of the 2 cm squares of a grid lined up with the board's
front edge and center line, wherever the pastry covers the point (96 points for the demo's sheet: 12 bands of 8). The
sim reads exactly these points (its strips are the 2 cm bands, its cells the 2 cm squares); a real trial reads them
with a depth gauge.

| Outcome | Score |
|---|---|
| At least 90 % of the points 3.2 mm within 0.3 mm (2.9 to 3.5 mm), the 2 cm bands' average thicknesses within 0.3 mm of each other (even across the width), all the pastry on the board, pin set down level on its rings, no hand touching it | 1.0 |
| At least 90 % of the points at most 3.7 mm, but the sheet not finished (fewer than 90 % on spec, or uneven across the width; about one pass to go) | 0.5 |
| The sheet done, but a hand still touching the pin (within 2 mm) or the pin not set down level | 0.5 |
| At least 90 % of the points at most 4.2 mm and the bands within 0.3 mm of each other (one level pass, about two to go) | 0.25 |
| Anything else: nothing done (4.8 mm everywhere), less than one pass, a wedge from a tilted pin, pastry pushed off the board | 0 |

Where the arms end is not scored. Possible scores: 0, 0.25, 0.5, 1.0. Doing nothing scores 0. The score is the sheet the
robot leaves: no penalty is assumed for a stroke with one hand off the pin (in the sim it just does not count) or for
rolling past an end (the rings keep the barrel 3.2 mm up there as everywhere). There is no length rule: the sheet is
as long as its volume makes it at the thickness reached.

Sim and real trial differ in one rule. In the sim, the dough model presses the pastry only while both grippers touch
the pin; this is a documented approximation, because the sim's pin slides on its rings instead of turning, so the
model cannot tell rolling from pushing it along. The real trial has no hand rule at all: it scores only the measured
sheet (thickness and evenness at the grid points, coverage) and the final pin state (set down level, let go). A sheet
rolled partly or wholly with one hand scores exactly what the same sheet rolled with two hands scores.

Why each rule: an even 1/8 inch is the job itself, a sheet that bakes evenly. Evenness across the width is checked on
its own because it is the one fault the rings exist to prevent: rolled with one end lifted, the pin leaves a wedge,
thicker on that side, and puff pastry rises higher and bakes slower where it is thicker, so it can come out lopsided.
The pastry must stay on the board, where it is measured and can be used. The pin must be set down level and let go,
because a pin still held or hung in the air is not finished with. In the sim only, strokes count while both hands are
on the pin (on its handles or on the barrel): the sim's pin slides rather than turns, so two hands on it are the
stand-in for rolling it. The real trial does not check the hands.

Why each level is worth what it is: 1.0 is the job as asked. 0.5 is a sheet within 0.5 mm of the target nearly
everywhere, with about one pass to go or a slight wedge, or a finished sheet with the pin not put away; either is half
the job. 0.25 is one level pass: at least 0.6 mm off the 4.8 mm start nearly everywhere and even across the width, so
the pastry is safely started and rolled the right way, but still two passes from done. Below that (nothing done, a
single stroke out and back, or a wedge from a lifted end) there is no usable progress, so 0.

Why the numbers: 3.2 mm is 1/8 inch, the thickness the maker's recipes roll this pastry to and the ring sold for it.
0.3 mm (about 10 % of 3.2 mm) either side: enough that a level sheet
three crossings in (3.4 mm) is on spec while the start (4.8 mm) and one crossing (4.0 mm) are not; it is ten times the
gauge's 0.01 mm reading. 90 %: the outer row of points, near the cut and rolled edges where a sheet is pinched or
ragged, is about a fifth of the points, and 10 % lets about half of that row be off. 3.7 mm is within 0.5 mm, about one
pass to go; 4.2 mm is 0.6 mm off the start, what one level pass reaches at its ends (4.0 mm). 0.3 mm across the width is
the same tolerance a point is held to; each band is averaged over its length, so the check reads a wedge, not a single
thick point. A level pin presses every band alike (0 mm offline for the demo's strokes); one end 0.5 mm up through
eight passes leaves 0.36 mm (0.5), and 1 mm up leaves 0.72 mm (0). The 5 mm and 3-degree pin checks are resting
tolerances for a 6.74 cm ring on a flat surface (3 degrees over the 29 cm between the rings is a 15 mm rise, so a pin
past it is propped on something); a real trial reads them with a ruler and a level app. Whether a hand still touches
the pin is read from the video.

The demo: both arms grip the pin's handles from above (fingers closing along x) where it lies, lift it over the middle
of the pastry and set it down on its rings, then four passes, each from the middle out to the far end and back, then
out to the near end and back, aimed about 6 mm past the end as it then lies (worked out with doughkit; the pin trails
the hands by about 1 cm at each turn, and rolling past an end presses the end like any other slice). Then the pin is
lifted off the pastry, set back down on its rings where it lay, and both hands open. Offline the strokes give 1.0 with
the pin stopping anywhere from at to 2 cm short of each commanded end; the first sim run must confirm it (see the
run script).

Reach: the strokes run x 15 to 33 cm with the hands on the handles at |y| 19.5 cm, 11 cm in from the arm bases at |y|
30.5 cm, so the hands stay 0.19 to 0.35 m from their arm's base. The pin is picked up and put down at x 35 (0.37 m),
lifting its 0.73 kg weight. The YAM's capability comes from its servo ratings (Damiao DM-J4340 on joints 1 to 3, 9 N·m
rated and 27 N·m peak; DM-J4310 on joints 4 to 6, 3 N·m rated and 7 N·m peak) through the arm's Jacobian from the URDF,
gravity included: about 52 N continuous pressing down per hand (about 104 N with both), about 11 N continuous along the
board in its weakest horizontal direction (about 17 N typical, about 28 N peak). These continuous figures hold within
about 0.35 to 0.37 m of the arm's base at table height; farther out, holding the arm up already takes the shoulder to
its rated torque. The strokes stay inside that; only the pick-up and put-down at 0.37 m sit at its edge. The force a
person needs to roll this pastry has not been measured.

Offline (test_task.py, the same strokes without the arm's lag): a single stroke out to the far end and back leaves the
near half at 4.8 mm (0); one pass, 3.6 to 4.0 mm everywhere and even (0.25); two passes, 3.3 to 3.4 mm (1.0); the
demo's four, 3.2 to 3.3 mm (1.0). One end 0.5 mm up through eight passes 0.5; 1 mm up 0; 4 mm up (one hand) 0. A level
sheet at 3.6 mm everywhere 0.5. A sheet all on spec but 0.35 mm thicker on one side 0.5, at 0.25 mm 1.0. Two outer
bands at 2.5 mm (thin or torn) take a finished sheet below 1.0. One gripper closed on the middle of the barrel 0
(nothing counts); one hand on a handle and one closed on the barrel 1.0; both closed hands pressing on top of the
barrel 1.0; one hand on a handle and the other hovering 5 mm over the barrel, nothing pressed; open hands around the
handles touching nothing, nothing pressed; one hand lifted off the pin for part of the work, then proper strokes, 1.0;
full-length strokes over both ends 1.0 (nothing below 3.2 mm); the strokes at 0.4 m/s 1.0; pin set down where it lay
or on the table beside the board 1.0; pin hung in the air, fingers closed on the handles, fingertips resting on the
barrel (or 1 mm over it) at the end, or the pin propped 5 degrees off level, 0.5; hands opened wide around the handles
touching nothing, or closed fingertips 5 mm over the barrel, 1.0; fingers hovering 9 mm over both handles through every
stroke, nothing counts.

In a real trial (no hand rule; the score is the measured sheet and the final pin state): lay a 2 cm grid transparency
over the sheet, lined up with the board's front edge and its center line, so the points are the same every trial and
the same as the sim's. At the center of each 2 cm square the pastry covers (about 96 points), read the thickness with a
digital tire tread depth gauge (zeroed on the bare board, its foot resting on the pastry without pressing, its pin
pushed through to the board). A point is on spec at 2.9 to 3.5 mm, and at least 90 % must be. For evenness, average the
readings in each 2 cm band running the sheet's length: the thickest band may be at most 0.3 mm above the thinnest. A
point under 2.9 mm, a hole where the board shows through, a tear, or a spot where pastry stuck to the pin and pulled
away counts as off spec (a ringed pin cannot press below 3.2 mm, so anything thinner is damage). Pastry pushed off the
board scores 0. The sim cannot produce these, so they matter only in a real trial. For the pin, read its level with a
level app and its rest on the board with a ruler, and check the last frames of the video that no finger touches it.

## Real-world build

- A maple board about 12 x 16 inches (30 x 40 cm, 1.5 cm thick), lightly floured with all-purpose flour.
- A handled maple rolling pin with a 12 inch barrel, for example the Frieling Classic Rolling Pin with Handles (2.4
  inch barrel, 21 inch overall, 1.6 lb; the barrel turns on nylon bushings), and a pair of silicone spacer rings, for
  example the 1/8 inch pair from Mrs. Anderson's Baking silicone rolling pin rings (an 8-piece set, two rings each in
  four thicknesses from 0.0625 to 0.375 inch, sold for pins up to about 2.5 inches across). 1/8 inch is 3.18 mm; the sim
  uses 3.2 mm. Check the rings' height over the barrel with calipers.
- Pepperidge Farm Puff Pastry Sheets (17.3 oz box, two sheets). For each trial, thaw one sheet as the box directs, still
  cold, unfold it on the floured board, measure its thickness with calipers (record it; the maker gives 3/16 inch), cut
  it in half into two 11.5 x 24 cm pieces, set one centered at x 24, y 0, and start the trial at once. The other half
  is the next trial's, kept in the refrigerator.
- Measuring tools, all from a hardware or auto parts store: a digital tire tread depth gauge (the common 0 to 25 mm
  kind reading 0.01 mm, a thin pin sliding out of a flat foot), digital calipers, a 30 cm ruler, a 2 cm grid
  transparency (printed on an overhead sheet), a kitchen scale (about 122 g per half sheet), and a phone with a camera
  and a level app. No lab equipment is needed.
- Reset between trials: throw the rolled sheet away, scrape the board clean and re-flour it, set a fresh half sheet
  centered on the board, lay the pin on its rings at x 35 (axis along y), and check that both rings sit on the
  barrel's ends.

## What the task tests

Control of one tool's level: the thickness is set only while both spacer rings ride the board, so a 53 cm pin must be
kept level as it travels (the natural way is with both hands, but the real trial scores only the sheet). The work
changes under the tool: the sheet lengthens and springs back, so it takes several passes and the ends move out as it
grows.

## Sources

- Pepperidge Farm Puff Pastry Sheets, 17.3 oz, 2 sheets: retailer listings (e.g. target.com/p/-/A-13393722,
  shoprite.com product 00051000078742), which also quote the package's thawing and handling directions.
- Sheet size and thickness: puffpastry.com/faq ("approximately 9.75 x 10.5 x 3/16 inch ... folded in thirds").
- Measured sizes, other brands and availability: America's Test Kitchen, "The Best Store-Bought Puff Pastry"
  (americastestkitchen.com/taste_tests/2701-the-best-store-bought-puff-pastry).
- Rolled size: Pepperidge Farm, Puff Pastry Pepperoni Pizza (pepperidgefarm.com/recipe/puff-pastry-pepperoni-pizza):
  "Roll ... into a 15 x 10-inch rectangle."
- Pin: Frieling Classic Rolling Pin with Handles, model 3033 (e.g. everythingkitchens.com).
- Rings: Mrs. Anderson's Baking silicone rolling pin rings (Target and other kitchen retailers).

## Files

- `task.py`: the scene, the scorer and the scripted demo.
- `doughkit.py`: the pastry model (the press, springback and lengthening rules above).
- `guardkit.py` and `yamkit.py`: the task's helper modules (gripper contact shapes, scene and arm helpers).
- `test_task.py`: offline model and scorer checks; run `python tasks/roll-pastry-sheet-on-spacer-rings/test_task.py`.
