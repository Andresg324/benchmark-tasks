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

Use `--policy dream --max-steps 2400` for a Dream-1 trial (the run limit). The scene is deterministic (no seeds, object_jitter=0).
Runtime per trial: the demo runs 1,167 steps (58.4 s at 20 Hz); a Dream-1 trial runs up to 2,400 steps (2 minutes).

Results on this task.py (sha256 a8f7b3ef9491...): the scripted demo scores **1.0 in 1,167 steps**. Dream-1: PENDING.

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
  surface (the package directions, quoted on the retailers' pages). Cut to half its size with a ruler and a pizza
  wheel, it gives a piece 11.5 x 24 cm, 4.8 mm thick, the start of a trial (about 122 g, 132.5 mL). The whole sheet
  is not used: rolled to 3.2 mm it becomes about 38 x 25 cm, longer than the arms can roll from one side of the
  board (see "Reach" below).
- **The target.** 1/8 inch (3.2 mm). Pepperidge Farm's own recipes roll one sheet "into a 15 x 10-inch rectangle"
  (Puff Pastry Pepperoni Pizza, pepperidgefarm.com): from about 9.75 x 10.5 inches at 3/16 inch, that is about 1/8 inch
  (4.8 mm x 102 / 150 = 3.3 mm). 1/8 inch is also the spacer ring sold for it (below). So there is real rolling to do: a
  third of the thickness comes off.
- **Cold.** The package says the thawed sheet "should still be cold", to thaw it no more than 40 minutes, and if the
  sheets get warm or sticky to put them in the freezer for 5 minutes or the refrigerator for 10 to 15. A real trial
  thaws the sheet 35 minutes at room temperature (20 to 24 degrees C) and starts within 5 minutes of the end of the
  thaw, inside the box's 40-minute limit (see "Real-world evaluation" below).

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
| Pastry board | maple (mid brown): the J.K. Adams Williston board, 16 x 12 x 3/4 inch (40.6 x 30.5 x 1.9 cm), modeled 30 (x) x 40 (y) x 1.9 cm; floured; fixed | x 10 to 40, y -20 to 20, top z 1.9 |
| Puff pastry | half of a thawed Pepperidge Farm sheet: 11.5 (x) x 24 (y) cm, 4.8 mm (3/16 inch) thick, 132.5 mL, about 122 g; modeled, no collider | centered x 24, y 0 |
| Rolling pin | maple, a handled pin with a 12 inch barrel (Frieling Classic Rolling Pin with Handles: 2.4 inch barrel, 12 inch barrel, 21 inch overall, 1.6 lb): barrel 6.1 cm across and 30.5 cm long; on each end of the barrel a silicone 1/8 inch spacer ring 6.74 cm across (3.2 mm proud of the barrel) and 1.5 cm wide (\|y\| 13.75 to 15.25); handles 2.5 cm across and 11.4 cm long (\|y\| 15.25 to 26.65); 0.73 kg | lying on its rings on the board behind the pastry, axis along y at x 35, z 5.27 |

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
| Resistance | the pastry pushes the pin up by STRESS x 2 cm x sqrt(2 R h) and back against its motion by STRESS x 2 cm x h for each 2 cm strip in contact (h the largest excess there), STRESS 15 kPa; applied to the pin as a force and torque at the contact before every physics step | cold pastry resists the barrel; the force is an estimate (above), inside the arms' ratings |
| Pin set down | axis level within 3 degrees, lying on its rings on the board or on the table (their bottoms within 5 mm of it) | resting, not held in the air or propped on something |
| Both hands on the pin (sim only, a stand-in) | presses count only while both grippers touch the pin: each gripper's own shape (palm and fingers, boxes fitted to the YAM's collision meshes, at their present opening) within 2 mm of a handle, the barrel or a ring (flat-ended cylinders as the colliders), whether it grips a handle or presses on the barrel with a closed hand | the sim's pin slides on its rings instead of turning, so the model cannot tell rolling from pushing the pin along; two hands on the pin can hold both ends down, so both rings can stay on the board. The instruction names no grip, so handles and palms on the barrel both count. A hand hovering over the pin does not count. A stroke with one hand or none on the pin is not penalized, and nothing is assumed torn: that stroke just does not count. A real trial has no hand rule: it scores the measured sheet and the final pin state |
| Let go | no gripper touching the pin's barrel, rings or handles (its own shape more than 2 mm away) | a hand still on the pin has not set it down; an open hand near the pin but not touching it has |
| Board friction | 0.15 static, 0.10 dynamic (ring on floured board) | a stand-in for a barrel that turns. The named pin's barrel turns on nylon bushings around its axle, so a real pin rolls under still handles and its travel is resisted by rolling, not sliding. The sim's gripped pin cannot turn and slides on its rings instead, so its sliding friction is set low: at 0.10, the 0.73 kg pin pressed down with 10 N drags with about 1.7 N. No measured coefficient is claimed |

What the model stands in for, and what a real trial checks instead: the thickness comes from the press rule above,
which follows the real behavior (the rings set the floor, the dough springs back by a share of the excess, volume is
kept). The springback share is an estimate, so a real trial measures the sheet itself. The model cannot tear the sheet,
stick it to the pin or press it below 3.2 mm; in the sim a stroke without both hands on the pin simply does not count.

The pastry pushes back on the pin. Each control step, wherever the barrel is in contact with pastry thicker than the gap
(within its contact reach of the lowest line), the model takes h, the largest excess in each 2 cm strip, and a = sqrt(2
R h), the contact's length along x, and puts two forces on the pin at the contact: upward, the pastry's flow stress over
the contact's footprint (STRESS x 2 cm x a), and, while the barrel moves, against the motion, the work of squeezing h
off a 2 cm strip per meter rolled (STRESS x 2 cm x h). They are worked out from the pin's pose and applied to the pin
body as an external force and torque before every physics step (240 a second), so the arms have to hold the pin down and
drive it through the pastry, and a pin pressed harder at one end feels the thicker side push back harder. STRESS is 15
kPa, an estimate: no published compression or sheeting force for chilled store-bought puff pastry was found (sheeting
studies report bread dough), so it is set so that the first pass over the 4.8 mm sheet takes about 35 N of pressing down
(about 3.6 kg on the pin, a firm push by a person rolling cold pastry) and about 6 N of pushing along, falling as the
sheet thins (offline, for the demo's strokes). Both are inside the YAM's continuous ratings: per arm about 18 N of the
52 N down and 3 N of the 11 N along the board. The real trial measures the outcome, the sheet, so the estimate affects
only how hard the sim's arms work. The demo's hands are commanded 1 cm below the pin's axis while rolling, so the arms
hold the rings down on the board against the push instead of letting the pin ride up on the pastry.

Visuals (markers, no physics, never scored): the pastry is drawn as one golden sheet, built from abutting slabs, one
per covered 2 cm cell, each as thick as the pastry there, so the top camera shows the sheet growing as it is rolled.
Nothing about the scoring points is drawn.

## Scoring

The pastry's thickness is read in six bands along x, centered 3, 7 and 11 cm either side of the board's center line (y
-11, -7, -3, 3, 7, 11). In each band it is read at two kinds of point: row points every 4 cm along x from 1 cm behind
the board's front edge (x 11, 15, 19, ... 39 cm) wherever the pastry covers them, and two end points, 1 cm inside the
band's rolled ends, found from where the sheet's ends lie after rolling. A row point within 2 cm of an end point is left
to the end point. The demo's sheet gives 30 points (5 a band: 3 row points and 2 end points). The end points make an
unrolled end count: a sheet with its ends left at the start's thickness has its end points off spec. The sim reads
exactly these points (the row points are centers of the model's 2 cm cells; the end points are read on each band's own
slices); a real trial reads them with the depth rod of a digital caliper. Both readings are local: the sim reads the one
model slice under the point (a 2 cm band, a slice 2.5 to 3.7 mm long along x), and the caliper's foot, the end of its
beam (a few millimeters along x, its width across the band), rests on the pastry only around the point. No reading
covers more than its own point, and no two points overlap. The evenness check compares the bands' averages.

| Outcome | Score |
|---|---|
| At least 90 % of the points 3.2 mm within 0.3 mm (2.9 to 3.5 mm), the bands' average thicknesses within 0.3 mm of each other (even across the width), all the pastry on the board, pin set down level on its rings, no hand touching it | 1.0 |
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

Why the numbers: 3.2 mm is 1/8 inch, the thickness the maker's recipes roll this pastry to and the ring sold for it. 0.3
mm (about 10 % of 3.2 mm) either side, a protocol setting rather than a measured baking tolerance: enough that a level
sheet three crossings in (3.4 mm) is on spec while the start (4.8 mm) and one crossing (4.0 mm) are not; it is ten times
the caliper's 0.01 mm reading. 90 %: most of the points are in the outer bands or at the ends, near the cut and rolled
edges where a sheet is pinched or ragged, and 10 % lets about three of the 30 be off. 3.7 mm is within 0.5 mm, about one
pass to go; 4.2 mm is 0.6 mm off the start, what one level pass reaches at its ends (4.0 mm). 0.3 mm across the width is
the same tolerance a point is held to; each band is averaged over its points, so the check reads a wedge, not a single
thick point. A level pin presses every band alike (0 mm offline for the demo's strokes); one end 0.5 mm up through eight
passes leaves 0.36 mm (0.5), and 1 mm up leaves 0.72 mm (0). The 5 mm and 3-degree pin checks are resting tolerances for
a 6.74 cm ring on a flat surface (3 degrees over the 29 cm between the rings is a 15 mm rise, so a pin past it is
propped on something); a real trial reads them with a ruler and a level app. Whether a hand still touches the pin is
read from the video.

The demo: both arms grip the pin's handles from above (fingers closing along x) where it lies, lift it over the middle
of the pastry and set it down on its rings, then four passes, each from the middle out to the far end and back, then
out to the near end and back, aimed about 6 mm past the end as it then lies (worked out with doughkit; the pin trails
the hands by about 1 cm at each turn, and rolling past an end presses the end like any other slice). Then the pin is
lifted off the pastry, set back down on its rings where it lay, and both hands open. Offline the strokes give 1.0 with
the pin stopping anywhere from at to 2 cm short of each commanded end; in the simulator the demo scores 1.0 in 1,167
steps.

Reach: the strokes run x 15 to 33 cm with the hands on the handles at |y| 19.5 cm, 11 cm in from the arm bases at |y|
30.5 cm, so the hands stay 0.19 to 0.35 m from their arm's base. The pin is picked up and put down at x 35 (0.37 m),
lifting its 0.73 kg weight. The YAM's capability comes from its servo ratings (Damiao DM-J4340 on joints 1 to 3, 9 N·m
rated and 27 N·m peak; DM-J4310 on joints 4 to 6, 3 N·m rated and 7 N·m peak) through the arm's Jacobian from the URDF,
gravity included: about 52 N continuous pressing down per hand (about 104 N with both), about 11 N continuous along the
board in its weakest horizontal direction (about 17 N typical, about 28 N peak). These continuous figures hold within
about 0.35 to 0.37 m of the arm's base at table height; farther out, holding the arm up already takes the shoulder to
its rated torque. The strokes stay inside that; only the pick-up and put-down at 0.37 m sit at its edge.

Offline (test_task.py, the same strokes without the arm's lag): a single stroke out to the far end and back leaves the
near half at 4.8 mm (0); one pass, 3.6 to 4.0 mm everywhere and even (0.25); two passes, 3.3 to 3.4 mm (1.0); the demo's
four, 3.2 to 3.3 mm (1.0). One end 0.5 mm up through eight passes 0.5; 1 mm up 0; 4 mm up (one hand) 0. A level sheet at
3.6 mm everywhere 0.5. A sheet all on spec but 0.35 mm thicker on one side 0.5, at 0.25 mm 1.0. Two outer bands at 2.5
mm (thin or torn) take a finished sheet below 1.0. One gripper closed on the middle of the barrel 0 (nothing counts);
one hand on a handle and one closed on the barrel 1.0; both closed hands pressing on top of the barrel 1.0; one hand on
a handle and the other hovering 5 mm over the barrel, nothing pressed; open hands around the handles touching nothing,
nothing pressed; one hand lifted off the pin for part of the work, then proper strokes, 1.0; full-length strokes over
both ends 1.0 (nothing below 3.2 mm); the strokes at 0.4 m/s 1.0; pin set down where it lay or on the table beside the
board 1.0; pin hung in the air, fingers closed on the handles, fingertips resting on the barrel (or 1 mm over it) at the
end, or the pin propped 5 degrees off level, 0.5; hands opened wide around the handles touching nothing, or closed
fingertips 5 mm over the barrel, 1.0; fingers hovering 9 mm over both handles through every stroke, nothing counts.
Strokes stopping 3 cm short of the near end (x 19.5 cm) leave that end at 4.8 mm while every row point is on spec; the
end points read it, so the sheet scores below 1.0.

In a real trial the score is the measured sheet and the final pin state, with the hands free. The points are the sim's,
located along each band with a steel tape (row points from the board's front edge, end points 1 cm inside each band's
rolled edge), and each is read with the depth rod of a Neiko 01407A digital caliper: the rod pushed through the
pastry to the board, the beam's end lowered until it just touches the pastry's top. A point under 2.9 mm, or a spot
where pastry stuck to the pin and pulled away, counts as off spec (a ringed pin stops at 3.2 mm, so anything thinner is
damage). A hole or tear seen on the after photo, taken before the rod goes in, caps the score at 0.5. These arise only
in a real trial; the sim's sheet stays whole. The pin's level and rest are read with a phone level app and a steel tape,
and the last second of the video shows whether a finger still touches it. The full procedure, from the pastry's lot to
the reset, is in "Real-world evaluation" below.

## Real-world build

Everything is bought off the shelf; nothing is made, printed or drilled for the task. The same board, pin, rings, ruler
and gauge are used for every trial.

- Board: the J.K. Adams Williston Large maple cutting board with juice groove, 16 x 12 x 3/4 inch (WLS-1612; 40.6 x
  30.5 x 1.9 cm), laid juice-groove side down so the plain face is up. The sim's board is 40 x 30 x 1.9 cm (its plan
  rounded by 3 to 6 mm). It is held to the table with double-sided carpet tape.
- Pin: the Frieling Classic Rolling Pin with Handles (12 inch barrel, 2.4 inch across, 21 inch overall, 1.6 lb; the
  barrel turns on nylon bushings). Spacer rings: the 1/8 inch pair from Mrs. Anderson's Baking silicone rolling pin
  rings (an 8-piece set, two rings each in four thicknesses from 0.0625 to 0.375 inch, sold for pins up to about 2.5
  inches across). 1/8 inch is 3.18 mm; the sim uses 3.2 mm.
- Pastry and flour: Pepperidge Farm Puff Pastry Sheets (17.3 oz box, two sheets); all-purpose flour and a small
  fine-mesh sieve; a pizza wheel.
- Ruler: the Prym Omnigrid Universal Ruler, cm scale, 15 x 60 cm (article 611308), a clear quilting ruler with a
  metric grid. It guides the cut.
- Calipers, also the thickness gauge: the Neiko 01407A digital caliper (0 to 150 mm, 0.01 mm display, accuracy
  +-0.02 mm per the maker; about $25), a common budget caliper with a depth rod. The rod slides out of the end of its
  beam; the beam's end face (a few millimeters thick; measured once and recorded) is the foot that rests on the pastry
  around a point. Check leaves: a 20-leaf metric feeler gauge set, 0.05 to 1.00 mm (for example Draper 51701, a few
  dollars), stacked to about 3.00 mm (1.00 + 0.95 + 0.90 + 0.15); the stack is read once with the caliper on a bench
  and that value recorded as its reference.
- Gripper pad covers: Glad Press'n Seal food wrap, a food-safe wrap that seals to itself under finger pressure.
- Scale: American Weigh Scales Fast Weigh MS-500-BLK pocket scale (500 g x 0.1 g), for the flour.
- Thermometers: a ThermoPro TP50 digital room thermometer (+-0.5 degrees C) at the bench; a Taylor 5924 dial
  refrigerator/freezer thermometer (-29 to 27 degrees C, -20 to 80 degrees F) in the freezer.
- Also: a steel tape; masking tape; double-sided carpet tape; a timer; a phone with a level app and a camera on an
  overhead phone stand.

Rough cost of the rig, at common US retail prices: board about $45, pin about $40, rings about $12, caliper about
$25, feeler set about $10, scale about $20, the two thermometers about $20, ruler about $40, sieve, pizza wheel, tapes,
wrap and phone stand about $40: about $250 in all, plus about $7 of pastry a box (two trials).

**Budget build.** Two items have cheaper equivalents that still meet the protocol's tolerances. The cut only needs a
straight edge with 2 mm marks for its 3 mm tolerance, so a Johnson 40-0564 24 inch aluminum inch/metric straightedge
(about $14) can replace the Omnigrid ruler. The flour is weighed to 5.0 +- 0.5 g, so any 500 g x 0.1 g pocket scale
(about $10 to $12, an estimate) can replace the AWS scale. The thickness tools are already budget items (the Neiko
caliper, +-0.02 mm, checked on a feeler stack within 0.03 mm, for a 0.3 mm band). The board and pin stay as named,
because the sim models their sizes. Totals: primary about $250, budget about $215.

## Real-world evaluation (reproducible protocol)

Every real trial starts from the same measured state and is scored on the same points as the simulator (30 on the demo's
sheet), read with a caliper's depth rod. The protocol fixes the pastry (one product, one lot, the box's thaw, a measured
cut), the room (20 to 24 degrees C), the positions (tape marks on the table and board), the flour (weighed), the
measurement (the points located with a steel tape, a checked caliper, all points read within 5 minutes, plus the pin
check and photos), the gripper pads' covers and the reset. Where a step comes from the maker's directions it is quoted;
every other tolerance is the protocol's own setting, chosen to be well inside the scoring bands. They are collected in
the table at the end. Positions are in cm from the midpoint between the arm bases (x forward, y to the robot's left),
measured with the steel tape.

**The room (every trial).** Everything is done at room temperature: 20 to 24 degrees C (68 to 75 degrees F), read on
the room thermometer at the bench at the start of each trial and recorded. Trials run only within that range.

**Marks (once).**
- Board: masking-tape L marks on the table at the board's four corners put its front edge at x 10.0 and its center
  line at y 0 within 2 mm, square to the robot within 0.5 degree. The board is taped down inside them with
  double-sided carpet tape. The table and board are level within 0.5 degree (the phone's level app on the bare board,
  both directions).
- Ticks: short masking-tape ticks on the board's vertical edge faces, leaving its top clear for the rings: on the
  front and back faces at y 0 (the center line), y -12 and 12 (the pastry's sides) and y -11, -7, -3, 3, 7 and 11 (the
  bands); on both side faces at x 18.25
  and 29.75 (the pastry's near and far edges), at x 35 (the pin's axis), and at x 11, 15, 19, 23, 27, 31, 35 and 39
  (the rows of measuring points, every 4 cm from 1 cm behind the front edge).
- Camera: the phone on its overhead stand about 60 cm above the board, the whole board in frame, the same zoom and
  position every trial.

**The measuring points.** The same points the sim reads, in six bands at y -11, -7, -3, 3, 7 and 11 (the front- and
back-face ticks). Row points: at x 11, 15, 19, ... 39 (the side-face ticks) wherever the pastry covers them. End
points: in each band, 1 cm inside the pastry's near and far edges as they lie after rolling, measured from the edge
with the steel tape. A row point within 2 cm of an end point is left to the end point. The demo's sheet gives 30 (3
row points and 2 end points a band); a longer sheet adds row points, a shorter one has fewer.

**The pastry: product and lot (every series).** One product, Pepperidge Farm Puff Pastry Sheets, 17.3 oz, two sheets
per box. A series of trials uses boxes of one lot (the code and best-by date printed on the box, recorded for each
trial), bought together, inside their best-by date, and kept frozen ("Keep frozen until ready to thaw", the package)
at -18 degrees C or colder (the freezer thermometer). A sheet is thawed once. If a series needs
more boxes than one lot provides, the lot change is recorded and the next trial is flagged as the first of a new lot.
Each trial uses one sheet, cut to half its size on the board.

**Setting up a trial.** The package's directions are followed and quoted (the product listing at giantfoodstores.com
and the maker's recipes at pepperidgefarm.com, e.g. Neapolitan Napoleons).
1. Thaw: take one sheet from the box and return the other to the freezer at once. Remove its wrapping and thaw it at
   room temperature, as the box directs: "Thaw pastry sheet at room temperature 40 minutes or until easy to handle";
   "Thaw pastry sheet(s) until the pastry unfolds easily no more than 40 minutes"; "Pastry should unfold easily but
   still be cold after thawing." The maker's recipes add: "At room temperature, thaw unwrapped pastry for no more
   than 40 minutes." Start the timer when it comes out of the freezer: it thaws 35 minutes, so with at most 5 minutes of
   set-up after it the sheet is out of the freezer at most 40 minutes, inside the box's limit. The thaw is staggered so
   that it never holds up a trial: one sheet comes out of the freezer every 15 minutes on a fixed schedule, each on its
   own labeled kitchen timer, so a thawed sheet is ready as each trial's turn comes. A trial cycle then takes about 15
   minutes: about 3 minutes of prep for the trial itself (steps 2 to 8: flour, pin, unfold, check, cut, caliper, pad
   covers, photo), at most 2 minutes of rolling, at most 5 minutes of measuring and about 3 minutes of reset (below).
2. Board: clean and dry, inside its marks. Weigh 5.0 g of all-purpose flour (within 0.5 g) and sift it evenly over the
   board: the box's "lightly floured surface", by weight. Lay the pin on its rings with its axis over the x 35 ticks
   on both side faces (within 5 mm) and the rings' outer faces 5 cm in from the board's sides (within 5 mm each), so
   the pin is centered and square. Check the rings first: both pushed fully onto the barrel's ends (outer face flush
   with the barrel's end within 1 mm).
3. Unfold: "Unfold pastry sheet on lightly floured surface." "Remove parchment paper." (the box), here on the board,
   centered on it, with its fold seams running along x. Press the two fold seams together along their length with the
   fingertips, and "If pastry cracks when unfolding, sprinkle the crack with water and press the edges together to seal"
   (the box). A sheet with a corner missing is set aside and replaced from the same lot.
4. Measure: read the sheet's thickness with the calipers at the middle of each of its four edges (the jaws closed just
   to touch). The average is recorded; the maker gives 3/16 inch (4.8 mm), and a sheet averaging outside 4.3 to 5.3
   mm is set aside.
5. Cut: lay the ruler across the sheet, its long edge on the x 18.25 ticks, and run the pizza wheel along it; then on
   the x 29.75 ticks; then lay it along x on the y -12 and y 12 ticks (the front and back faces) for the sides. Lift
   the trimmings away. The piece is 11.5 x 24.0 cm, each edge within 3 mm of its ticks (checked with the steel tape),
   with both fold seams inside it running along x. The maker gives the sheet as about 9.75 x 10.5 inches (24.8 x 26.7
   cm), so the 24.0 cm cut fits; a sheet under 24.0 cm across (along y) is set aside and replaced from the same lot.
6. Check the caliper: close it fully and zero it; stand its beam's end on the bare board's margin with the rod pushed
   down to the board: 0.00. Then stand the beam's end on the leaf stack laid on the board, the rod
   beside the stack, down on the board: it reads the stack's recorded value within 0.03 mm (about the caliper's
   accuracy), five times; otherwise it is cleaned, zeroed and checked again.
7. Cover the gripper pads: tear a 10 x 10 cm piece of Glad Press'n Seal for each of the four fingers, lay it over the
   finger's pad and tip, sticky side in, and press it to seal around the finger's back, smoothing it flat over the pad's
   face (one layer there, no folds). The covers keep flour and pastry off the shared pads; the grippers hold the pin
   only by its handles, which stay off the board. Wrap on the lacquered handles grips less than bare pads (the sim keeps
   the pads' friction, an approximation); the grip's 50 N leaves a wide margin over the 0.73 kg pin and the pastry's few
   newtons along the board.
8. Take the before photo (a card in frame with the trial number, lot code, thaw time, average thickness and room
   temperature) and start the trial within 5 minutes of the end of the thaw. The run itself is limited to 2,400 steps
   (2 minutes).

**Measuring (every trial).** All points are read within 5 minutes of the pin's set-down (the protocol's fixed window, a
setting rather than a measured rest time), so every sheet is read at the same age. Reading takes about 6 s a point and
10 s a band to lay the steel tape, so the demo's 30 points take about 4 minutes. Start the timer when the pin comes to
rest.
1. Video: the last 1 s of the trial video shows no gripper finger touching the pin (let go).
2. Pin: lay the phone along the top of the barrel, steadied by a fingertip on the phone (not on the pin), and read the
   level app (zeroed on the bare board before the trial): within 3 degrees. Hold the steel tape upright beside each
   ring: its lowest point is within 5 mm of the surface under it (the board, or the table if the pin was put down
   beside the board). Take the after photo with the pin in place, and lift the pin straight up off the board. The photo
   records any pastry beyond the board's edges (pastry off the board scores 0) and any hole or tear (caps the score at
   0.5).
3. Read each band in turn: lay the steel tape along the band's line (sighted between its front- and back-face ticks),
   hooked on the board's front edge. At each point (row points at x 11, 15, 19, ... where the pastry covers them, then
   the end points 1 cm inside the pastry's near and far edges on that line), hold the caliper upright with its beam's 16
   mm width across the band, push the depth rod through the pastry until it stops on the board, then slide the beam down
   until its end just touches the pastry's top (the lightest touch; the hand holds the caliper's weight), and read it to
   0.01 mm: the pastry's thickness at that point. Row points within 2 cm of an end point are left to the end points.
   About 6 s a point and 10 s a band, so the demo's 30 points take about 4 minutes. A helper writes every reading into
   the readings table by band (or the phone records the reader's voice); a spot where pastry stuck to the pin and pulled
   away, at a point, is recorded as off spec. The time of the last reading is recorded: a trial is scored when it is
   within 5 minutes of the set-down.
4. Repeatability (a check after the scored readings): re-read three points (the middle two of the middle row and
   one end point) in their first holes. Each repeat is within 0.05 mm of its first reading; otherwise the caliper is
   checked on the leaves again and the trial is flagged.
5. Photo the sheet (the record of the sheet that was read). Unwrap the gripper pads' covers and throw them away.
6. Score with the simulator's rules: at least 90 % of the read points at 2.9 to 3.5 mm, each band's points averaged
   within 0.3 mm of every other band's average, all pastry on the board, and the pin down and let go for 1.0; the 0.5
   and 0.25 tiers as above; a hole or tear on the after photo caps the score at 0.5. The score uses the sheet and the
   pin only: the hands and the sheet's length are free.

**Reset (every trial, about 3 minutes).** Throw the rolled sheet away: each trial rolls a fresh sheet once. Scrape the
board with a bench scraper, wipe it with a dry cloth and let it dry; it is re-floured with weighed flour at the next
set-up. Brush flour and pastry off the pin's barrel and rings and off the ruler with a dry pastry brush. Check each ring
with the calipers at three places around it: half the difference between the ring's diameter and the barrel's beside it
is 3.2 mm within 0.1 mm (1/8 inch is 3.18 mm); a ring outside that, torn or stretched loose is replaced as a pair. Check
the board is still inside its marks and the camera stand unmoved.

**Tolerances.**

| What | Target | Tolerance | How checked | Source |
|---|---|---|---|---|
| Room | room temperature | 20 to 24 degrees C (68 to 75 degrees F) | room thermometer at the bench | protocol |
| Product | Pepperidge Farm Puff Pastry Sheets, 17.3 oz | one lot per series, inside best-by | box label recorded | protocol |
| Storage | frozen | -18 degrees C or colder; thawed once | freezer thermometer | package ("keep frozen"); temperature protocol |
| Thaw | unwrapped, at room temperature, 35 min | thaw plus set-up at most 40 min ("no more than 40 minutes") | timer | package limit; 35 min protocol |
| Seams and cracks | pressed together | cracks sprinkled with water and pressed to seal | by eye | package |
| Sheet thickness | 4.8 mm (3/16 inch) | average 4.3 to 5.3 mm | calipers, 4 edges | maker's size; tolerance protocol |
| Piece | 11.5 x 24.0 cm at x 18.25 to 29.75, y -12 to 12 | 3 mm each edge | ruler on the ticks, steel tape | protocol |
| Thaw end to start | within 5 min | | timer | protocol |
| Thaw schedule | one sheet out of the freezer every 15 min | each on its own labeled timer | timers | protocol |
| Per-trial prep | about 3 min (flour, pin, unfold, check, cut, caliper check, pad covers, photo) | at most 5 min from the end of the thaw | timer | protocol |
| Reset | about 3 min (sheet out, board scraped and dried, pin brushed, rings checked) | before the next trial's prep | timer | protocol |
| Flour on the board | 5.0 g | 0.5 g | scale, sieve | package ("lightly floured"); weight protocol |
| Board | front edge x 10.0, center y 0 | 2 mm; square and level within 0.5 degree | tape marks, steel tape, level app | protocol |
| Pin start | axis at x 35, centered | 5 mm; rings flush on the barrel's ends within 1 mm | ticks, steel tape | protocol |
| Ring height | 3.2 mm (1/8 inch = 3.18 mm) | 0.1 mm | calipers, 3 places per ring, every trial | protocol |
| Caliper zero | 0.00 with the beam's end and rod on the bare board | 0.00 | board margin | protocol |
| Caliper check | about 3.00 mm stack of feeler leaves | its recorded value within 0.03 mm, five times | feeler gauge set | caliper's stated accuracy; tolerance protocol |
| Pad covers | one layer of Glad Press'n Seal over each finger's pad and tip | fresh every trial, no folds on the pad face | by eye | protocol |
| Reading window | all points, about 8 s each (30 on the demo's sheet) | within 5 min of the pin's set-down | timer | protocol setting |
| Repeat readings | three points re-read in their holes | within 0.05 mm | caliper | protocol |
| Pin down (scored) | level, on its rings | within 3 degrees; rings within 5 mm of the surface | level app, steel tape | scoring |
| Let go (scored) | no finger on the pin | last 1 s of the video | video | scoring |
| Thickness points (scored) | 3.2 mm | 2.9 to 3.5 mm at 90 % of points; bands within 0.3 mm | caliper depth rod, 0.01 mm | scoring (band a protocol setting) |
| Holes and tears (scored) | none | a hole or tear on the after photo caps the score at 0.5 | after photo | scoring |
| End points | 1 cm inside each band's rolled ends | 3 mm | steel tape, band ticks | protocol |

## What the task tests

Control of one tool's level: the thickness is set only while both spacer rings ride the board, so a 53 cm pin must be
kept level as it travels (the natural way is with both hands, but the real trial scores only the sheet). The work
changes under the tool: the sheet lengthens and springs back, so it takes several passes and the ends move out as it
grows.

## Sources

- Pepperidge Farm Puff Pastry Sheets, 17.3 oz, 2 sheets, with the package's thawing and handling directions quoted
  above: Giant Food Stores product listing
  (giantfoodstores.com/groceries/product/pepperidge-farm-puff-pastry-sheets-2-ct-frozen-17-3-oz-box/30027).
- Thawing unwrapped: Pepperidge Farm, Neapolitan Napoleons (pepperidgefarm.com/recipe/neapolitan-napoleons): "At room
  temperature, thaw unwrapped pastry for no more than 40 minutes."
- Sheet size and thickness: puffpastry.com/faq ("approximately 9.75 x 10.5 x 3/16 inch ... folded in thirds").
- Measured sizes, other brands and availability: America's Test Kitchen, "The Best Store-Bought Puff Pastry"
  (americastestkitchen.com/taste_tests/2701-the-best-store-bought-puff-pastry).
- Rolled size: Pepperidge Farm, Puff Pastry Pepperoni Pizza (pepperidgefarm.com/recipe/puff-pastry-pepperoni-pizza):
  "Roll ... into a 15 x 10-inch rectangle."
- Board: J.K. Adams Williston Large maple cutting board with groove, 16 x 12 x 0.75 inch, WLS-1612
  (chefsupplies.ca/products/j-k-adams-16-x-12-x-0-75-maple-large-williston-cutting-board-with-groove-wls-1612).
- Pin: Frieling Classic Rolling Pin with Handles, model 3033, 2.4 inch x 12 inch barrel, 21 inch overall, nylon
  bushings (everythingkitchens.com/frieling-classic-rolling-pin-with-handles.html).
- Rings: Mrs. Anderson's Baking silicone rolling pin rings, 8-piece set, 0.0625, 0.125, 0.25 and 0.375 inch, "fits
  rolling pin cylinders up to 2.5-inches in diameter"
  (vermontkitchensupply.com/products/silicone-rolling-pin-rings-8pc-set; also target.com/p/-/A-1006725924).
- Ruler: Prym Omnigrid Universal ruler, cm scale, 15 x 60 cm, article 611308
  (prym.com/en/universal-ruler-15x60cm-16113080).
- Gripper pad covers: Glad Press'n Seal food plastic wrap (target.com/p/-/A-14771700).
- Calipers: Neiko 01407A electronic digital caliper, 0-6 in / 0-150 mm, 0.01 mm resolution, +-0.02 mm accuracy,
  with depth rod (zoro.com, item G321697191; machinistguides.com, Neiko 01407A review).
- Check leaves: Draper 51701 20-blade metric feeler gauge set, 0.05 to 1.0 mm
  (drapertools.com/product/51701/20-blade-metric-feeler-gauge-set).
- Scale: American Weigh Scales Fast Weigh MS-500-BLK, 500 g x 0.1 g (homebrewfinds.com review listing).
- Room thermometer: ThermoPro (TempPro) TP50, -50 to 70 degrees C, +-0.5 degrees C (temppro.com, TP50).
- Freezer thermometer: Taylor 5924 refrigerator/freezer dial thermometer, -20 to 80 degrees F (katom.com, 383-5924).

## Files

- `task.py`: the scene, the scorer and the scripted demo.
- `doughkit.py`: the pastry model (the press, springback and lengthening rules above).
- `guardkit.py` and `yamkit.py`: the task's helper modules (gripper contact shapes, scene and arm helpers).
- `test_task.py`: offline model and scorer checks; run `python tasks/roll-pastry-sheet-on-spacer-rings/test_task.py`.
