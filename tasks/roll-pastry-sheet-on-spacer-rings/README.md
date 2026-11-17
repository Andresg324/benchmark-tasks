# Roll store-bought crescent dough to an even 1/8 inch with a spacer-ring rolling pin

A sheet of store-bought crescent roll dough on a floured maple board, and a handled maple rolling pin with 1/8 inch
silicone spacer rings on both ends of its barrel, lying on the board behind the dough. "Roll this crescent dough out to
an even 1/8 inch (3.2 mm) thick, then set the rolling pin down off the dough." The sheet starts at 3/16 inch (4.8 mm),
set by rolling the dough between guide strips. The rings set the thickness only while both of them ride on the board,
so the pin has to be kept level along its length: with one end lifted the barrel leaves a wedge, thick on that side. A
light press springs back; passes past the dough's yield thin it a little each time, so it takes several. Category:
Societal relatability. Rolling dough out flat with a rolling pin, for a pie crust, cookies or a pizza base, is a
common home-baking job; the dough here is a store-bought refrigerated one (Pillsbury Original Crescent, an 8 oz can),
and spacer rings that slip onto a rolling pin are sold by ordinary kitchen retailers for an even thickness.

Run (from the repo root, Isaac Lab-Arena installed):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/roll-pastry-sheet-on-spacer-rings/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Roll this crescent dough out to an even 1/8 inch (3.2 mm) thick, then set the rolling pin down off the dough." \
      --scorer task_score --max-steps 2400
    python tasks/roll-pastry-sheet-on-spacer-rings/test_task.py      # offline model and scorer checks, prints "all pass"

Use `--policy dream --max-steps 2400` for a Dream-1 trial (the run limit). The scene is deterministic (no seeds, object_jitter=0).
Runtime per trial: the demo runs 1,420 steps (71.0 s at 20 Hz); a Dream-1 trial runs up to 2,400 steps (2 minutes).

Results on this task.py (sha256 6d12ed359a95...): the scripted demo scores **1.0 in 1,420 steps**. Dream-1 scored **0 in 2,400 steps** on the earlier task.py (a8f7b3ef9491); it has not been re-run on this one.

## The dough: what people buy, and the start sheet

- **The product.** Pillsbury Original Crescent Rolls, the refrigerated 8 oz (226 g) can sold in most US supermarkets.
  The can holds one perforated sheet of dough.
- **The start sheet is set by the protocol, not by the package.** The can's dough is balled, rolled with the pin
  between two 3/16 inch guide strips and trimmed with a straightedge and a pizza wheel to 11.5 x 24 cm, 4.8 mm thick
  (132.5 mL). One can makes that piece with dough to trim, and serves a session: after each trial the rolled sheet is
  balled with its trimmings and re-formed the same way. A caliper reads the trimmed sheet at 3 points before each
  trial: 4.6 to 5.0 mm, or it is rolled again.
- **Temperature.** The dough is used at room temperature (20 to 24 C): the can is opened and the dough left out 40
  minutes before the first trial. Nothing is chilled.
- **The target.** 1/8 inch (3.2 mm), the spacer ring sold for an even sheet (below). A third of the thickness comes off.

Volume (rolling keeps it): 11.5 x 24 x 0.48 = 132.5 mL. Over the same 24 cm width, at the on-spec band's ends, 15.8 cm at 3.5 mm and 19.0 cm at 2.9 mm.
Centered at x 24 cm, even the 19.0 cm sheet runs x 14.5 to 33.5, on the 30 cm board (x 10 to 40) and inside the strokes'
reach. The demo's sheet ends about 16.4 cm long (x 15.8 to 32.2). In the model the sheet's width is held at 24 cm
and its volume is conserved by lengthening (below); the rings sit 1.75 cm outside each long side of the piece. The
score has no length rule.

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Board | maple (mid brown): the J.K. Adams Williston board, 16 x 12 x 3/4 inch (40.6 x 30.5 x 1.9 cm), modeled 30 (x) x 40 (y) x 1.9 cm; lightly floured; fixed | x 10 to 40, y -20 to 20, top z 1.9 |
| Crescent dough | Pillsbury Original Crescent, balled, rolled between 3/16 inch guide strips and trimmed: 11.5 (x) x 24 (y) cm, 4.8 mm thick, 132.5 mL; modeled, no collider | centered x 24, y 0 |
| Rolling pin | maple, a handled pin with a 12 inch barrel (Frieling Classic Rolling Pin with Handles: 2.4 inch barrel, 12 inch barrel, 21 inch overall, 1.6 lb): barrel 6.1 cm across and 30.5 cm long; on each end of the barrel a silicone 1/8 inch spacer ring 6.74 cm across (3.2 mm proud of the barrel) and 1.5 cm wide (\|y\| 13.75 to 15.25); handles 2.5 cm across and 11.4 cm long (\|y\| 15.25 to 26.65); 0.73 kg | lying on its rings on the board behind the dough, axis along y at x 35, z 5.27 |

The pin lies on the board where a cook leaves it, not in a rest. Its barrel's front is 2.2 cm behind the dough's far
edge, and its handles reach past the board's sides (they are thinner than the rings, so they touch nothing).

Model (`doughkit.py`): the sheet is 12 strips along x, one per 2 cm of width; each strip is a chain of slices (2.5 mm
long at the start) with a fixed volume each, so a slice's length is its volume over its thickness and the total volume
never changes.

| Quantity | Value | Why |
|---|---|---|
| Gap | the barrel's lowest line above the board where it crosses each strip, from the pin's pose | 3.2 mm with both rings on the board; one ring lifted 1 mm (0.2 degrees over the 29 cm between the rings) gives about 3.3 mm at one edge of the dough and 4.1 mm at the other |
| Squeeze | h = t - gap, the barrel's depth into a slice, counted only while it is held (not for a momentary contact) | a pin dropped onto the dough or bouncing on it does not press |
| Press law | the dough springs back from light presses and thins once pressed past its yield, a share of the squeeze beyond the yield staying in each pass (below; the constants are in doughkit.py) | rigid-plastic dough with an elastic threshold, so it takes several passes; level rolling settles just above the rings' gap, inside the on-spec band (offline, test_task.py) |
| Rolling | a slice is pressed as the barrel's face reaches its top, sqrt(2 R h - h^2) ahead of the lowest line (R = 3.05 cm), once per crossing | a round barrel meets thick dough ahead of its lowest point (10 mm ahead on the 4.8 mm sheet) |
| Pressing in place | a slice under the barrel's face that no crossing has pressed since the barrel last came down on it is pressed by the face's depth over it (h - s^2 / 2R at a distance s from the lowest line), once until the barrel lifts off it or rolls on | a straight-down press thins dough too, most under the line |
| Lengthening | a pressed slice grows longer; the dough beyond it, away from the piece's middle, moves out | the middle is held by its own weight on the board; the sheet grows at both ends; its width is held at 24 cm, so the volume goes into length |
| Rolled past an end | the end slice is pressed like any other the barrel crosses | the rings hold the barrel 3.2 mm up there too, so nothing goes thinner than the gap, and the model does not tear the sheet there |
| Resistance | for each 2 cm strip in contact (h the largest squeeze there): up, the contact pressure over the contact's footprint, p x 2 cm x sqrt(2 R h - h^2), with p rising to the flow stress (13 kPa) at the yield and equal to it beyond; back against the motion, the plastic work per meter rolled; applied to the pin as a force and torque at the contact before every physics step | the dough's push on the barrel: elastic below its yield, its flow stress above. 13 kPa is set from a hand check on this dough (below) |
| Put away | the pin set down (axis level within 3 degrees, lying on its rings on the board or on the table, their bottoms within 5 mm of it), its barrel's lowest line at least 1 cm clear of the rolled sheet's outline along its whole length, and let go | resting, not held, not left lying on the dough, not propped on something |
| Let go | no part of either arm, from the upper arm (link3) to the fingertips, touching the pin: their real collision meshes (yam_links.npz, surface points every 1.5 mm, posed each control step from the simulator's link poses) overlap none of the pin's colliders (a gap of 0 or less) | contact, not nearness: a finger 0.3 mm off the pin has let go. guardkit's box model of the gripper is off by several mm against the real meshes, so it is used only in the offline tests (also as overlap) |
| Ring-board contact | 0.25 static, 0.20 dynamic friction | the pin rides on its rings; the ring-board contact uses a low sliding friction (the low end of wood on wood, Serway and Jewett, Physics for Scientists and Engineers, Table 5.1) |

**How the load governs thinning.** The dough is modeled as rigid-plastic with an elastic threshold: it springs back
from light presses and thins once pressed past its yield. Below the yield the contact pressure rises with the squeeze
and the slice springs fully back when the barrel passes; past it the pressure is the flow stress (13 kPa) and part of
the squeeze stays. The squeeze and the load on a strip go together: a strip one cell (2 cm) wide carries F = p x 2 cm
x sqrt(2 R h - h^2), so the dough yields only where the pin's load on it reaches its yield load. In the simulator the
barrel's height is where the pin settles under the arms' push, its own weight, the rings on the board and this push
back, so the thinning per pass is set by the load the hands put through the pin: a pin that is not pushed down rides up
on the dough and presses less. One hand or two may roll: what counts is the load the pin carries, not which hands carry
it. Tested offline (test_task.py): the pin's own weight (7.2 N) stays under the yield load across the sheet at every
thickness from the start to the rings' gap, so a pin resting on the dough, or rolled without a push, does not thin it;
passes that skim the start sheet (the barrel 0.1 or 0.2 mm into it, end to end) thin nothing however many are made; a
pin dropped onto the sheet and bouncing off thins nothing; passes past the yield thin it a little each time; and level
rolling with both rings on the board ends inside the 2.9 to 3.5 mm band.

**The resistance, from a hand check on this dough.** Pillsbury Original Crescent dough, balled and re-rolled, at room
temperature, on a bare cutting board set on a kitchen scale, rolled with a rolling pin without rings from about 3/16
inch to about 1/8 inch. The pin was pushed with the normal working pressure a person uses to roll dough flat: enough
that the dough stays flat, no extra weight. The scale under the board read about 3.0 to 3.5 kg at its peak (29 to
34 N). After enough passes the rolled dough stayed flat. The sim's flow stress, 13 kPa, is set so that its first pass
carries the force read in the hand check: on the sim's first pass
(the 4.8 mm sheet, 24 cm wide, h = 1.6 mm, a = sqrt(2 x 30.5 x 1.6 - 1.6^2) = 9.75 mm), F = stress x 24 cm x 9.75 mm,
so 29 to 34 N gives 12.4 to 14.5 kPa; the task uses 13 kPa, which gives 30.4 N up (3.1 kg) and 4.2 N back on the
first pass. A press too light to pass the dough's yield springs back; a lighter push past it thins less a pass and
needs more passes. Both forces are inside the YAM's continuous ratings: per arm about 15 N of the 52 N down and
2 N of the 11 N along the board with both hands, and one arm alone can carry the whole first pass. The demo's hands are
commanded 1 cm below the pin's axis while rolling, so the arms hold the rings down on the board against the push.

Visuals (markers, no physics, never scored): the dough is drawn as one golden sheet, built from abutting slabs, one per
covered 2 cm cell, each as thick as the dough there, so the top camera shows the sheet growing as it is rolled.
Nothing about the scoring points is drawn.

## Scoring

The dough's thickness is read in six bands along x, centered 3, 7 and 11 cm either side of the board's center line (y
-11, -7, -3, 3, 7, 11). In each band it is read at two kinds of point: row points every 4 cm along x from 1 cm behind
the board's front edge (x 11, 15, 19, ... 39 cm) wherever the dough covers them, and two end points, 1 cm inside the
band's rolled ends, found from where the sheet's ends lie after rolling. A row point within 2 cm of an end point is left
to the end point. The demo's sheet gives 30 points (5 a band: 3 row points and 2 end points). The end points make an
unrolled end count: a sheet with its ends left thick has its end points off spec. The sim reads exactly these points
(the row points are centers of the model's 2 cm cells; the end points are read on each band's own slices); a real
trial reads them with the depth rod of a digital caliper. Both readings are local: the sim reads the one model slice
under the point (a 2 cm band, a slice 2.5 to 3.7 mm long along x), and the caliper's foot, the end face of its beam,
rests on the dough only around the point. No reading covers more than its own point, and no two points overlap. The
evenness check compares the bands' averages.

| Outcome | Score |
|---|---|
| At least 90 % of the points 3.2 mm within 0.3 mm (2.9 to 3.5 mm), the bands' average thicknesses within 0.3 mm of each other (even across the width), all the dough on the board, the pin put away (set down level on its rings, its barrel at least 1 cm clear of the dough, let go) | 1.0 |
| At least 90 % of the points at most 3.7 mm, but the sheet not finished (fewer than 90 % on spec, or uneven across the width; about one pass to go) | 0.5 |
| The sheet done, but the pin not put away (still held, left lying on the dough, or not set down level) | 0.5 |
| At least 90 % of the points at most 4.2 mm and the bands within 0.3 mm of each other (one level pass, about two to go) | 0.25 |
| Anything else: nothing done (4.8 mm everywhere), less than one pass, a wedge from a tilted pin, dough pushed off the board | 0 |

Where the arms end is not scored. Possible scores: 0, 0.25, 0.5, 1.0. Doing nothing scores 0. The score is the sheet the
robot leaves and the pin's final state; the hands are free (one or two may roll, in the sim and on the board), and
there is no penalty for rolling past an end (the rings keep the barrel 3.2 mm up there as everywhere). There is no
length rule: the sheet is as long as its volume makes it at the thickness reached.

Why each rule: an even 1/8 inch is the job itself, a sheet that bakes evenly. Evenness across the width is checked on
its own because it is the one fault the rings exist to prevent: rolled with one end lifted, the pin leaves a wedge,
thicker on that side, and a sheet bakes unevenly, browning where it is thin before it is done where it is thick. The
dough must stay on the board, where it is measured and can be used. The pin must be put away, set down off the dough
and let go, because a pin still held, hung in the air or left lying on the sheet (it is in the way and has not
been put away) is not finished with.

Why each level is worth what it is: 1.0 is the job as asked. 0.5 is a sheet within 0.5 mm of the target nearly
everywhere, with about one pass to go or a slight wedge, or a finished sheet with the pin not put away; either is half
the job. 0.25 is one level pass: at least 0.6 mm off the 4.8 mm start nearly everywhere and even across the width, so
the dough is safely started and rolled the right way, but still two passes from done. Below that (nothing done, a
single stroke out and back, or a wedge from a lifted end) there is no usable progress, so 0.

Why the numbers: 3.2 mm is 1/8 inch, the ring sold for it. 0.3 mm (about 10 % of 3.2 mm) either side, the protocol's
setting: enough that a level sheet three crossings in (3.47 mm offline) is on spec while the start (4.8 mm) and one crossing
(4.12 mm) are not; it is thirty times the caliper's 0.01 mm reading. 90 %: most of the points are in the outer bands or
at the ends, near the cut and rolled edges where a sheet is pinched or ragged, and 10 % lets about three of the 30 be
off. 3.7 mm is within 0.5 mm, about one pass to go; 4.2 mm is 0.6 mm off the start, what one level pass reaches (3.76
to 4.12 mm). 0.3 mm across the width is the same tolerance a point is held to; each band is averaged over its points,
so the check reads a wedge, not a single thick point. A level pin presses every band alike (0 mm offline for the demo's
strokes); one end 0.3 mm up through ten passes leaves 0.23 mm and its high side 3.5 to 3.7 mm (0.5), and 1 mm up a wedge
(0). The 5 mm and 3-degree pin checks are resting tolerances for a 6.74 cm ring on a flat surface (3 degrees over the
29 cm between the rings is a 15 mm rise, so a pin past it is propped on something); 1 cm clear of the dough is a gap a
person sees at a glance. A real trial reads them with a ruler and a level app; whether a hand still touches the pin is
read from the video.

The demo: both arms come down on the pin's handles from 15 cm above them and grip them from above (fingers closing
along x) where it lies, lift it over the middle of the dough and set it down on its rings, then five passes, each from the middle out to the far end and back, then out
to the near end and back, aimed about 6 to 8 mm past the end as it then lies (worked out with doughkit; the pin trails
the hands by about 1 cm at each turn, and rolling past an end presses the end like any other slice). Then the pin is
lifted off the dough, set back down on its rings where it lay (x 35, about 3 cm clear of the sheet's far end), and both
hands open. In the simulator the loaded right arm tracks about 9 mm ahead of the left, turning the pin, so its targets
from the grip on are set 9 mm back. Offline the strokes give 1.0 with the pin stopping anywhere from at to 2 cm short of each commanded end; in
the simulator the demo scores 1.0 in 1,420 steps.

Reach: the strokes run x 15 to 33 cm with the hands on the handles at |y| 19.5 cm, 11 cm in from the arm bases at |y|
30.5 cm, so the hands stay 0.19 to 0.35 m from their arm's base. The pin is picked up and put down at x 35 (0.37 m),
lifting its 0.73 kg weight. The YAM's capability comes from its servo ratings (Damiao DM-J4340 on joints 1 to 3, 9 N·m
rated and 27 N·m peak; DM-J4310 on joints 4 to 6, 3 N·m rated and 7 N·m peak) through the arm's Jacobian from the URDF,
gravity included: about 52 N continuous pressing down per hand (about 104 N with both), about 11 N continuous along the
board in its weakest horizontal direction (about 17 N typical, about 28 N peak). These continuous figures hold within
about 0.35 to 0.37 m of the arm's base at table height; farther out, holding the arm up already takes the shoulder to
its rated torque. The strokes stay inside that; only the pick-up and put-down at 0.37 m sit at its edge.

Offline (test_task.py, the same strokes without the arm's lag): a single stroke out to the far end and back leaves the
near half at 4.8 mm (0); one pass, 3.76 to 4.12 mm everywhere and even (0.25); two passes leave the ends at 3.5 to 3.6
mm (0.5); three passes, 3.4 to 3.5 mm (1.0); the demo's five, 3.37 to 3.38 mm (1.0). One end 0.3 mm up through ten
passes 0.5; 1 mm up 0; 4 mm up (one hand) 0. A level sheet at 3.6 mm everywhere 0.5. A sheet all on spec but 0.35 mm
thicker on one side 0.5, at 0.25 mm 1.0. Two outer bands at 2.5 mm (thin or torn) take a finished sheet below 1.0.
One hand on one handle, the other off the pin, 1.0; one gripper closed on the middle of the barrel 1.0; one hand on a
handle and one closed on the barrel 1.0; both closed hands pressing on top of the barrel 1.0; full-length strokes over
both ends 1.0 (nothing below 3.2 mm); the strokes at 0.4 m/s 1.0. Light skims, the barrel held 0.1 or 0.2 mm into the
sheet's top, end to end: nothing thins (0). The pin dropped onto the sheet and bouncing back (down for one control
step), five times at each of four spots: nothing thins. Pressed straight down in place: the dough under the line thins,
holding it there does not press again, lifting it and setting it down again does. Pin set down where it lay or on the
table beside the board 1.0; pin let go lying on the finished sheet, or 5 mm past its end, 0.5, and 1.5 cm clear 1.0;
pin hung in the air, fingers closed on the handles, fingertips pressed on the barrel at the end, or the pin propped 5
degrees off level, 0.5; hands opened wide around the handles touching nothing, closed fingertips 5 mm or 0.3 mm over
the barrel, 1.0. On the real meshes: a fingertip 1.0 or 0.3 mm over the barrel has let go, one 0.2 mm into it has not,
and neither has a wrist resting on a handle. Strokes stopping short of the near end (x 19 cm) leave that end at 4.1 mm
while every row point is on spec; the end points read it, so the sheet scores below 1.0.

In a real trial the score is the measured sheet and the final pin state, with the hands free. The points are the sim's,
located along each band with a steel tape (row points from the board's front edge, end points 1 cm inside each band's
rolled edge), and each is read with the depth rod of a Neiko 01407A digital caliper: the rod pushed through the dough to
the board, the beam's end face lowered until it just touches the dough's top. A point under 2.9 mm, or a spot where
dough stuck to the pin and pulled away, counts as off spec (a ringed pin stops at 3.2 mm, so anything thinner is
damage). A hole or tear seen on the after photo, taken before the rod goes in, caps the score at 0.5. These are read
on the real sheet; the sim's sheet is whole by construction. The pin's level, rest and clearance from the dough are read with a phone level app and a steel tape, and
the last second of the video shows whether a finger still touches it. The full procedure is in "Real-world evaluation"
below.

## Real-world build

The rig is a low-cost setup of common kitchen stock at the sim's sizes. The same board, pin, rings, ruler and caliper are
used for every trial. Where a product is named, any equivalent that meets the stated sizes serves.

Parts:

- Board: a flat wooden cutting board, 40 x 30 cm (16 x 12 inch) in plan and 1.9 cm (3/4 inch) thick, with a plain
  face. Any board 40 to 41 cm across, 30 to 31 cm deep and 1.6 to 2.2 cm thick serves; the sim's board is
  40 x 30 x 1.9 cm. Example: the J.K. Adams Williston Large cutting board (WLS-1612; 40.6 x 30.5 x 1.9 cm), laid
  juice-groove side down so the plain face is up.
- Pin: a handled rolling pin whose barrel turns on its axle, the barrel 30.5 cm (12 inch) long and 6.1 cm
  (2.4 inch) across, handles about 2.5 cm across and 11 cm long, about 53 cm (21 inch) overall and 0.73 kg (1.6 lb),
  as in the sim. Any pin with a barrel 30 to 31 cm long and 5.8 to 6.4 cm across (inside the spacer rings' rated
  size), handles 2 to 3 cm across and at least 9 cm long, and a mass of 0.6 to 0.9 kg serves. Example: the Frieling
  Classic Rolling Pin with Handles (12 inch barrel, 2.4 inch across, 21 inch overall, 1.6 lb; the barrel turns on
  nylon bushings).
- Spacer rings: the 1/8 inch pair from Mrs. Anderson's Baking silicone rolling pin rings (an 8-piece set, two rings
  each in four thicknesses from 0.0625 to 0.375 inch, sold for pins up to about 2.5 inches across). 1/8 inch is
  3.18 mm; the sim uses 3.2 mm.
- Dough and flour: Pillsbury Original Crescent Rolls (refrigerated 8 oz can; one can serves a session); all-purpose
  flour.
- Guide strips: two 3/16 inch x 1/2 inch x 24 inch basswood strips (Midwest Products 4059; 3/16 inch is 4.76 mm), for
  re-forming the start sheet before each trial.
- Ruler: the Johnson 40-0564 24 inch aluminum inch/metric straightedge (2 mm graduations). It guides the cut, whose
  tolerance is 3 mm an edge.
- Check leaves: a 20-leaf metric feeler gauge set, 0.05 to 1.00 mm (the Draper 51701 set, or any metric set with
  these leaves), stacked to about 3.00 mm (1.00 + 0.95 + 0.90 + 0.15).
- Fingertip marks: Scotch 35 vinyl electrical tape, red, a band round each fingertip's back over the wrap.
- Gripper pad covers: Glad Press'n Seal food wrap, a food-safe wrap that seals to itself under finger pressure, over
  the robot's finger pads, tips and palm faces.
- Gloves (for people): AMMEX X3 disposable nitrile gloves (powder-free, made from materials that comply with FDA
  food-contact rules), worn by anyone handling the dough.
- Also: masking tape, double-sided carpet tape, a dry cloth, a bench scraper and a dry pastry brush.

Tools:

- Use a digital caliper with a depth rod as the thickness gauge: the Neiko 01407A (0 to 150 mm, 0.01 mm display,
  accuracy +-0.02 mm per the maker). The rod slides out of the end of its beam; the beam's end face is the foot that
  rests on the dough around a point (its size is measured once with the caliper's jaws and recorded as the footprint).
- Use a pizza wheel, a steel tape, a plate, and a phone with a timer, a level app and a camera on an overhead phone
  stand.

Build steps (details and tolerances in the protocol below):

1. Read the feeler stack once with the caliper on a bench and record that value as its reference. Measure the beam's
   end face with the jaws and record it.
2. Lay masking-tape L marks on the table for the board's corners (front edge at x 10.0, center line at y 0), and tape
   the board down inside them with double-sided carpet tape, plain face up. Check it level with the phone's level app.
3. Put the tape ticks on the board's edge faces for the dough's edges, the pin's axis, the bands and the rows of
   measuring points (Marks, below).
4. Slide a 1/8 inch spacer ring onto each end of the pin's barrel, flush with the barrel's ends, and check each ring
   with the caliper (3.2 mm within 0.1 mm).
5. Set the phone on its overhead stand about 60 cm above the board, the whole board in frame.

## Real-world evaluation (reproducible protocol)

Every real trial starts from the same measured sheet (11.5 x 24 cm, 4.6 to 5.0 mm) and is scored on the same points as
the simulator (30 on the demo's sheet), read with a caliper's depth rod. The steps below are the whole protocol; the
tolerances are collected in the table at the end. Positions are in cm from the midpoint between the arm bases (x
forward, y to the robot's left), measured with the steel tape. Everything is done at room temperature (20 to 24 C).
Anyone handling the dough wears AMMEX X3 disposable nitrile gloves (a fresh pair each session and whenever one tears);
the robot's grippers wear their Press'n Seal covers.

Food safety: raw flour and raw dough can carry harmful bacteria, and the FDA and CDC advise not eating raw flour or raw
dough. The trial dough is never eaten: it is thrown away at the end of the session. Handlers wear gloves, and the
board, pin, strips, straightedge and pizza wheel are washed with hot soapy water after the session.

**Marks (once per rig).**
- Board: masking-tape L marks on the table at the board's four corners put its front edge at x 10.0 and its center
  line at y 0 within 2 mm, square to the robot within 0.5 degree. The board is taped down inside them with
  double-sided carpet tape. The table and board are level within 0.5 degree (the phone's level app on the bare board,
  both directions).
- Ticks: short masking-tape ticks on the board's vertical edge faces, leaving its top clear for the rings: on the
  front and back faces at y 0 (the center line), y -12 and 12 (the dough's sides) and y -11, -7, -3, 3, 7 and 11 (the
  bands); on both side faces at x 18.25 and 29.75 (the dough's near and far edges), at x 35 (the pin's axis), and at
  x 11, 15, 19, 23, 27, 31, 35 and 39 (the rows of measuring points, every 4 cm from 1 cm behind the front edge).
- Camera: the phone on its overhead stand about 60 cm above the board, the whole board in frame, the same zoom and
  position every trial.
- Grip through the wrap (once per rig): close both covered grippers on the handles, lift the pin 10 cm off the board
  and shake it gently side to side for 5 s. It stays in the grip without slipping. The sim uses the bare pads'
  friction.

**The measuring points.** The same points the sim reads, in six bands at y -11, -7, -3, 3, 7 and 11 (the front- and
back-face ticks). Row points: at x 11, 15, 19, ... 39 (the side-face ticks) wherever the dough covers them. End
points: in each band, 1 cm inside the dough's near and far edges as they lie after rolling, measured from the edge
with the steel tape. A row point within 2 cm of an end point is left to the end point, so no two points overlap. The
demo's sheet gives 30 (3 row points and 2 end points a band); a longer sheet adds row points, a shorter one has fewer.

**Session start (once per session).**
1. Open one can of Pillsbury Original Crescent Rolls; record its lot code. Leave the dough out at room temperature for
   40 minutes before the first trial.
2. Check the caliper: zeroed, it reads 0.00 with the beam's end and rod on the bare board, and the feeler-leaf stack's
   recorded value within 0.03 mm, five times. Check each ring with the caliper at three places: 3.2 mm within 0.1 mm,
   both flush with the barrel's ends within 1 mm (a ring outside that, torn or loose is replaced as a pair).

**Each trial.**
1. Form the start sheet. Lightly flour the rolling pin and the board. Ball the dough (for the session's first trial
   the can's dough; after that the last trial's rolled sheet and all the trimmings), set it centered on x 24, y 0, and
   roll it out in all directions with the pin to a rough rectangle a little over 12 cm along x and 25 cm along y. Lay
   the two guide strips along x on the board, one beside each long side of the dough, and set the pin on them, so the
   barrel rests on both strips and the rings hang just outside them (the strips' outer edges against the rings' inner
   faces, about 24.8 cm between the strips). Finish rolling along x until the barrel rides on both strips along the
   whole sheet: 3/16 inch (4.76 mm). Lift the strips away. Lay the straightedge on the x 18.25 ticks and run the pizza wheel along it, then on the x
   29.75 ticks, then along x on the y -12 and 12 ticks. The sheet is 11.5 x 24.0 cm, each edge within 3 mm; the
   trimmings go on the plate for the next trial. Start check: read the thickness with the caliper's depth rod at 3
   points (x 24 on y -7, 0 and 7): each 4.6 to 5.0 mm, otherwise ball it and roll it again.
2. Set up. Lay the pin on its rings with its axis over the x 35 ticks (within 5 mm), centered. Fit fresh covers on the
   grippers: a 10 x 10 cm piece of Glad Press'n Seal over each of the four fingers' pad and tip, sticky side in, pressed
   to seal around the finger's back and smooth over the pad's face (one layer, no folds), one more piece over each
   palm face between the fingers, and a band of red vinyl electrical tape (Scotch 35, red) round each fingertip's back
   over the wrap, so the overhead video shows each finger against the pin. Take the before photo, with a card in frame giving the trial number, the lot code
   and the start check's readings.
3. Run the trial: at most 2,400 steps.
4. Measure (all points read within 5 minutes of the pin's set-down; start the phone's timer when the pin comes to rest).
   - Let go: the last 1 s of the overhead video shows no part of the robot touching the pin (the red tape bands mark
     the fingertips).
   - Pin: lay the phone along the top of the barrel, steadied by a fingertip on the phone, and read the level app
     (zeroed on the bare board): within 3 degrees. Hold the steel tape upright beside each ring: its lowest point is
     within 5 mm of the surface under it. With the steel tape, the barrel's lowest line is at least 1 cm clear of the
     dough. Take the after photo with the pin in place (it records dough beyond the board's edges, which scores 0, and
     any hole or tear, which caps the score at 0.5), then lift the pin straight up.
   - Points: band by band, lay the steel tape along the band's line, hooked on the board's front edge. At each point,
     hold the caliper upright with its beam across the band, push the depth rod through the dough to the board, slide
     the beam down until its end face just touches the dough's top (the hand holds the caliper's weight, so the face
     does not sink in), and read it to 0.01 mm. A helper writes the readings down (or the phone records the reader's
     voice). A spot where dough stuck to the pin and pulled away, at a point, is off spec.
   - Repeats: re-read three points (the middle two of the middle row and one end point) in their holes: each within
     0.05 mm of its first reading; otherwise the caliper is checked on the leaves again and the trial is flagged.
5. Score with the simulator's rules: at least 90 % of the points at 2.9 to 3.5 mm, the bands' averages within 0.3 mm
   of each other, all dough on the board, and the pin put away for 1.0; the 0.5 and 0.25 tiers as above; a hole or
   tear on the after photo caps the score at 0.5. The score uses the sheet and the pin only; the hands and the sheet's
   length are free.
6. Reset: unwrap the grippers' covers and throw them away, wipe each pad, tip and palm face with a dry cloth and check
   by eye that they are clean and dry; brush the pin's barrel and rings with a dry pastry brush; ball the rolled sheet
   together with the trimmings for the next trial and scrape the board.

The dough is re-worked between trials, as in the hand check. At the end of the session it is thrown away (Food safety,
above).

**Tolerances.**

| What | Target | Tolerance | How checked | Source |
|---|---|---|---|---|
| Product | Pillsbury Original Crescent Rolls, 8 oz can | one can per session; lot code recorded | can label | protocol |
| Dough temperature | room temperature (20 to 24 C); 40 min out of the can before the first trial | | phone timer | protocol |
| Start sheet | 11.5 x 24.0 cm at x 18.25 to 29.75, y -12 to 12, rolled between 3/16 inch strips | 3 mm each edge; 4.6 to 5.0 mm at 3 points | straightedge on the ticks, steel tape; caliper depth rod | protocol |
| Dough reuse | the session's dough, balled and re-formed each trial | | before photo | protocol |
| Flour | a light dusting on the rolling pin and the board | | by eye | protocol |
| Board | front edge x 10.0, center y 0 | 2 mm; square and level within 0.5 degree | tape marks, steel tape, level app | protocol |
| Pin start | axis at x 35, centered | 5 mm | ticks, steel tape | protocol |
| Rings | 3.2 mm (1/8 inch = 3.18 mm), flush on the barrel's ends | 0.1 mm; flush within 1 mm | caliper, 3 places per ring, once per session | protocol |
| Caliper | 0.00 on the bare board; the leaf stack's recorded value (about 3.00 mm) | 0.00; within 0.03 mm, five times | board, feeler leaves, once per session | caliper's stated accuracy; tolerance protocol |
| Gloves (people) | AMMEX X3 nitrile on anyone handling the dough | fresh pair each session and when torn | by eye | protocol |
| Gripper covers | one layer of Glad Press'n Seal over each finger's pad and tip and each palm face; a red tape band round each fingertip | fresh every trial, no folds on the pad face; pads and palms wiped and inspected after every trial | by eye | protocol |
| Grip through the wrap | pin held by its handles, lifted 10 cm and shaken 5 s | no slip | by eye, once per rig | protocol |
| Reading window | all points (30 on the demo's sheet) | within 5 min of the pin's set-down | phone timer | protocol setting |
| Repeat readings | three points re-read in their holes | within 0.05 mm | caliper | protocol |
| Pin put away (scored) | level, on its rings, barrel clear of the dough | within 3 degrees; rings within 5 mm of the surface; at least 1 cm clear | level app, steel tape | scoring |
| Let go (scored) | nothing of the robot on the pin | last 1 s of the video | video | scoring |
| Thickness points (scored) | 3.2 mm | 2.9 to 3.5 mm at 90 % of points; bands within 0.3 mm | caliper depth rod, 0.01 mm | scoring (band a protocol setting) |
| Holes and tears (scored) | none | a hole or tear on the after photo caps the score at 0.5 | after photo | scoring |
| End points | 1 cm inside each band's rolled ends | 3 mm | steel tape, band ticks | protocol |

## What the task tests

Control of one tool's level and load: the thickness is set only while both spacer rings ride the board, so a 53 cm pin
must be kept level as it travels, and the dough thins only under a working push, so the pin must be held down through
it (the natural way is with both hands, but one may do). The work changes under the tool: the sheet lengthens and
springs back, so it takes several passes and the ends move out as it grows. Then the pin is put away off the dough.

## Sources

- Pillsbury Original Crescent Rolls, refrigerated dough, 8 oz can (pillsbury.com product page).
- Board: J.K. Adams Williston Large maple cutting board with groove, 16 x 12 x 0.75 inch, WLS-1612
  (chefsupplies.ca/products/j-k-adams-16-x-12-x-0-75-maple-large-williston-cutting-board-with-groove-wls-1612).
- Pin: Frieling Classic Rolling Pin with Handles, model 3033, 2.4 inch x 12 inch barrel, 21 inch overall, nylon
  bushings (everythingkitchens.com/frieling-classic-rolling-pin-with-handles.html).
- Rings: Mrs. Anderson's Baking silicone rolling pin rings, 8-piece set, 0.0625, 0.125, 0.25 and 0.375 inch, "fits
  rolling pin cylinders up to 2.5-inches in diameter"
  (vermontkitchensupply.com/products/silicone-rolling-pin-rings-8pc-set; also target.com/p/-/A-1006725924).
- Ruler: Johnson 40-0564 24 inch aluminum inch/metric straightedge, graduations in 1/8 inch, 1/16 inch and 2 mm
  (johnsonlevel.com/P/2096/24AluminumInchMetricStraightEdge).
- Guide strips: Midwest Products basswood strip 4059, 0.1875 x 0.5 x 24 inch (whistle-stop.com, MID4059).
- Gloves: AMMEX X3 nitrile gloves, powder-free, 3 mil, "component materials that comply with FDA regulations for food
  contact" (staples.com listing, X344100).
- Raw flour and raw dough: U.S. FDA, "Handling Flour Safely: What You Need to Know" (fda.gov), and CDC, "Raw Flour and
  Raw Dough" food-safety guidance (cdc.gov): do not eat raw flour or raw dough; wash hands, surfaces and utensils after
  contact.
- Gripper pad covers: Glad Press'n Seal food plastic wrap (target.com/p/-/A-14771700).
- Calipers: Neiko 01407A electronic digital caliper, 0-6 in / 0-150 mm, 0.01 mm resolution, +-0.02 mm accuracy,
  with depth rod (zoro.com, item G321697191; machinistguides.com, Neiko 01407A review).
- Check leaves: Draper 51701 20-blade metric feeler gauge set, 0.05 to 1.0 mm
  (drapertools.com/product/51701/20-blade-metric-feeler-gauge-set).
- Friction, wood on wood, 0.25 to 0.5 static and 0.2 kinetic (the ring-board contact uses its low end): R. A. Serway and J. W. Jewett, Physics for Scientists
  and Engineers, Table 5.1 (coefficients of friction).

## Files

- `task.py`: the scene, the scorer, the let-go contact check and the scripted demo.
- `doughkit.py`: the dough model (the press law, springback, lengthening and the push back on the pin).
- `yam_links.npz`: the YAM arm's collision meshes (from yam_arena's arm description, link3 to the fingertips) as surface
  points every 1.5 mm, for the let-go contact check.
- `guardkit.py` and `yamkit.py`: the task's helper modules (gripper contact shapes for the offline tests, scene and arm
  helpers).
- `test_task.py`: offline model and scorer checks; run `python tasks/roll-pastry-sheet-on-spacer-rings/test_task.py`.
