# Cast a concrete test cylinder in two rodded layers

A 1:2.5 bench model of casting a 4 x 8 in concrete test cylinder: an empty cylinder mold, a steel sample pan of fresh
concrete, a scoop on its stand, a tamping rod hanging in a rack and a hand float. "Make a test cylinder from this
batch of concrete." ASTM C31/C31M fills a 4 x 8 in cylinder consolidated by rodding in two layers: about half the
mold, rodded 25 times throughout its depth with the strokes spread over the cross-section, the mold's outside tapped
10 to 15 times, then the rest, rodded 25 times with each stroke reaching about 1 in into the first layer, tapped
again, and the top struck off flat. The tempting wrong answers are filling the mold in one go and rodding
it (or not rodding at all), and rodding the top layer without reaching into the one below; all of them score 0
however good the top looks.

Run (from the repo root, Isaac Lab-Arena installed; the kit files yamkit.py, pourkit.py, dosekit.py and guardkit.py sit next to
task.py):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/concrete-test-cylinder-two-layers/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Make a test cylinder from this batch of concrete." --scorer task_score --max-steps 3500
    python tasks/concrete-test-cylinder-two-layers/test_task.py      # offline scorer checks, prints "all pass"

Use `--policy dream --max-steps 6400` for a Dream-1 trial. The scene is deterministic (no seeds, object_jitter=0).
The scripted demo plans 164 s (3,274 steps) and holds success 1 s at the end. A full trial is 6,400 steps (5 minutes
20 seconds at 20 Hz), about twice the demo's four scoops, 50 counted strokes, 24 taps and strike-off. In the sim the
demo scores 1.0 and ends at step 3,252: the first layer 4.1 cm deep, its 25 strokes 0.1 to 0.3 mm off the floor; the
second layer's 25 strokes 19.5 to 19.9 mm into the first; 5 strokes in each zone in both layers (sector strokes 9.1
to 13.0 mm off the axis, center ones within 2.5 mm); 12 taps per layer; the struck top 0.4 mm over the rim, full to
1.002 of the brim; 3 g left in the scoop and 5 g spilled. Set `DOSEKIT_DEBUG=1` to print grams per bin,
packing and spill each step.

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0. Scale
1:2.5 (a 4 x 8 in mold, 101.6 x 203.2 mm, is 40.6 x 81.3 mm here; 1 in is 1.0 cm).

| Object | What it is | Where |
|---|---|---|
| Cylinder mold (fixed) | black plastic, 4.1 cm across and 8.1 cm deep inside (107 mL), 4.7 cm across outside, 8.4 cm tall, empty | axis at x 31, y 5 |
| Sample pan (fixed) | round steel pan, 13 cm across inside, 5.5 cm deep; 1,300 g of fresh concrete, 4.5 cm deep, top 4.75 cm up | axis at x 32, y -10.5 |
| Scoop (loose, 50 g) | steel bowl 3.9 cm across outside, 3.6 cm inside, 2.6 cm deep (26.5 mL level, about 31 mL or 69 g heaped), on a neck up to a red round handle 2.5 cm across and 7 cm long along y | handle center x 21, y -16, z 8; bowl center y -8.25 |
| Scoop stand (fixed) | white; post 1.6 x 6.6 cm under the handle, pad 3.5 x 3.5 cm under the bowl | x 21, y -16 |
| Tamping rod (loose, 60 g) | steel rod 5 mm across with a rounded tip, 8.6 cm below a black grip block 3 x 3 x 2.5 cm (11.1 cm overall); yellow tape band 8.1 cm above the tip | hangs by its block in the rack's slot, tip 1.2 cm above the table |
| Rod rack (fixed) | wood: a fork of two prongs 9.8 cm up, slot 1.2 cm wide and open toward the robot (-x), on a post behind | x 20.5, y 15 |
| Hand float (loose, 50 g) | aluminum blade 10 x 4 cm, 4 mm thick, long side along x; gray post 1.2 cm square, 2 cm tall; black knob 3 x 3 x 2.5 cm | lying flat at x 32, y 22 |

Concrete (dosekit `MATERIALS["concrete"]`): 2.20 g/mL as placed, 2.22 when dipped, 2.40 fully consolidated (fresh
normal-weight concrete is about 2,400 kg/m3); angle of repose 38 degrees; each rod stroke closes 12% of the gap to
consolidated in the band it reaches, so 25 strokes take a layer 96% of the way (a model value, chosen so 25 strokes is
"done" and 10 is clearly not).

How the concrete moves (dosekit, from the poses each control step): the scoop's mouth more than 4 mm under the pan's
surface fills it heaped; tipped past about 41 degrees over the mold it pours in, landing loose; a heap over the
mold's rim steeper than 38 degrees runs off over the rim (a spill). The rod is a `dk.Presser`: a stroke counts when
its tip goes 3 mm or more under the surface and comes back out; it consolidates from 6 mm below the tip up to the
surface. Concrete added after a stroke starts a new lift, so the mold keeps a record of its lifts (layers), and the
kit logs each stroke with the lift on top at the time and how deep it went (`dk.stroke_log`, added for this task).
Taps are judged on the hand's real outline (`_Taps` in task.py): the closed gripper's outer surface, a 4 mm grid
taken from the YAM finger and gripper meshes (the fingers form an open frame, 4 to 8 mm out from the tool point near
it and 10 to 20 mm out on side rails 1.6 to 4.4 cm up), turned with the hand each step. A tap counts when any point of
it below the rim comes within 1.5 mm of the mold's outer wall, the tool point outside the wall, and counts again only
after the whole hand draws back 1 cm; each is logged with the lift on top. In the sim a straight-down hand pressed
slowly on the mold stopped with its tool point 10.7 mm off the wall, where this outline just meets it.
Taps change nothing in the model. The float's blade is a `dk.Edge` along its underside's long edge; drawn across the rim it cuts any heap down to its
height and drops the excess off the rim on the side it moves toward.

## Scoring

Scored on the final state (SCORE_OVER = "end"; success must hold 1 s), using the kit's lift and stroke records.

| Outcome | Score |
|---|---|
| A tool point within 4 cm of the scoop handle's center, the rod's grip block or the float's knob, or 2.5 cm of any other part of them, or a tool point or fingertip within 1 cm of a tool's outline (still held) | 0 |
| A tool still moving (over 1 cm/s) or off the table | 0 |
| The rod's tip or the scoop's bowl left in the mold, or the float left lying on it | 0 |
| Not exactly two layers (one go; three or more), or more than 5 mm added in all after the second layer | 0 |
| First layer's top outside 2.8 to 5.3 cm above the floor (35% to 65% of the depth) | 0 |
| Fewer than 25 strokes, made while the first layer was on top, reaching within 8 mm of the floor | 0 |
| Fewer than 25 strokes, made while the second layer was on top, reaching 9 mm or more into the first | 0 |
| A layer's 25 strokes not spread: fewer than 4 of the 5 zones (within 5 mm of the axis; +x, +y, -x, -y sectors 9 mm or more off it) with 3 strokes each | 0 |
| Fewer than 10 or more than 15 taps on the mold after a layer's 25th stroke and before the next layer | 0 |
| Layers and rodding right, top struck off (the float's edge drawn over at least 90% of the mouth after the last stroke and the last concrete added; nothing over 2 mm above the rim, filled to within 2%), both arms home and still | 1.0 |
| The same, arms not home | 0.9 |
| Layers and rodding right, top heaped, sunk or not struck off | 0.25 |

Possible scores: 0, 0.25, 0.9, 1.0. Doing nothing scores 0 (no layers).

Why the numbers:
- Two layers, 25 strokes each, the first throughout its depth, the second about 1 in into the first, about half the
  mold first: ASTM C31/C31M for 4 x 8 in cylinders consolidated by rodding (see Sources). Over-rodding is not
  penalized.
- A layer is at least a quarter of the mold deep (2 cm). C31 allows adding a small quantity to an underfilled mold
  before the strike-off; here top-ups after the second layer may add up to 5 mm in all (1.25 cm at full scale), so a thick
  unrodded cap is not passed off as one, even split into thin pours with a rod poke between them.
- 35% to 65%: C31's "approximately half", with 15% of the depth (1.2 cm) either way; a scoop is about 27% of the
  mold, so one scoop more or fewer than two lands outside the window.
- Within 8 mm of the floor (2 cm at full scale): C31 rods the first layer throughout its depth without damaging the
  mold's bottom, so the rod stops just short of it; the kit's stroke consolidates 6 mm below the tip, so a stroke
  that deep reaches the whole layer.
- 9 mm into the first layer: C31's "approximately 1 in" is 1.0 cm at 1:2.5; 9 mm allows 1 mm for the rod's give at
  the bottom of a stroke. A stroke that stops at the first layer's top does not count.
- Spread: C31 distributes the roddings uniformly over the cross-section. The zones are the center (within 5 mm of the
  axis) and four 90-degree sectors at 9 mm or more from it (just under half the 2.05 cm bore radius; strokes between
  5 and 9 mm count in no zone, so a ring of strokes around the axis is not "uniformly over the cross-section"); 4 of 5 zones with at least 3 strokes each allows a technician who
  works around the edge and skips the center, and fails strokes in one, two or three spots.
- Taps: C31 taps the outside of the mold lightly 10 to 15 times after rodding each layer (a mallet, or an open hand
  on a mold a mallet would dent, as these plastic ones would), to close the rod holes and release trapped air. Taps
  made before the layer's 25th stroke, or after the next layer goes in, do not count for it. A tap is the hand
  meeting the wall below the rim (some point of its outline within 1.5 mm of the wall; the outline is rounded to 1 mm) and
  drawing back 1 cm, so a light upright tap counts, while a hand stopping 2.5 mm short, a leaned hand whose tool point
  is near the wall but whose outline is not, or a hand bobbing over the rim does not.
- Struck off: the float's edge passes over at least 90% of the mouth after the last stroke and the last concrete
  added (a careful pour that stops at the brim is not struck), a straightedge across the rim then shows no bump over
  2 mm, and the mold is full to within 2% of its volume (2 mL, 1.7 mm of depth), so a sunk top is not "struck".
- The 4 cm hands-off radius is just over the grips' size (2.5 to 3 cm); a gripper holding a tool is within it. A tool
  point within 2.5 cm of any other part of a tool (the rod's shaft, the scoop's bowl, the float's blade) also counts as
  holding it, and so does a tool point or fingertip within 1 cm of a tool's outline (a hand resting on the blade).
- Strokes made into a layer after the next layer is on top do not count for it: the first layer's 25 strokes must
  be made before the second layer goes in.

In a real trial: count the strokes per layer and where each goes in on a top-down video, and count the taps per
layer; check each stroke's depth on a side-on video (at 320x176 the top camera cannot show depth) against two tape
bands on the rod (one at the mold's depth, which reaches the rim when the tip is on the floor, and one at half the depth plus
1 cm); check the first layer against a mark at half height on the outside of the mold; lay a straightedge across the
rim (no gap or bump over 2 mm).

Offline tests (`test_task.py`, 72 checks; each "right" case below also taps 12 times after each layer): do nothing 0; the full job 1.0 (0.9 with the right arm not home); still
holding the scoop 0, let go 0.9; the rod still in hand 0, left standing in the mold 0; a hand left closed on the rod's shaft, the scoop's bowl or the float's blade 0, or a hand resting on the blade 0; the left arm 4 cm off its start 0.9; float still sliding 0;
strokes made one step down and one step up, and a strike-off drawn in 4 steps (fast but correct) 1.0; filled
in one go and rodded 50 times to the floor 0; one go, not rodded 0; one go, only the top half rodded 0; first layer
rodded 24 times 0; first layer rodded 10 times then 40 strokes to the floor through both layers 0; first layer
strokes stopping 2 cm off the floor 0; second layer strokes stopping 2 mm above the first 0, reaching 4 or 7 mm into it 0,
11 mm into it 1.0; second layer rodded 24 times 0; three layers 0; a first layer of one scoop (a quarter) 0, of three
scoops 0; a scoop short (sunk top) 0.25, then topped up with a whole scoop (1.7 cm
unrodded) and struck 0; the second layer to the brim, rodded, a 3.7 mm top-up and struck 1.0; a slow top-up to 98.5%
of the brim on three rodded scoops, never struck 0; struck, then a small top-up left unstruck 0.25; all strokes within
5.2 mm of the axis 0; taps with the hand's outline stopping 1.5 mm off the wall 1.0, 2.5 mm off 0; an upright hand's light tap (the outline just touching, tool point 10.7 mm off) 1.0, stopping 2.5 mm short 0; a hand leaned 15 deg with its tool point 16 mm off the wall (outline 9 mm off) 0, leaned with its fingertips on the wall 1.0; top-ups of 4.5 mm each with a poke between, struck, 0; a hand bobbing 1.2 to 2.5 cm over the rim 0; the
float left lying on the mold 0; heaped top (float drawn 6 mm high) 0.25;
rodding the empty mold 0; the rod hovering and grazing the first layer 0; fingers poked into the first layer 0; rod
knocked off the rack and float off the table 0; striking off the empty mold 0; a new trial after a one-go trial 1.0;
all strokes in one spot (on the axis, or 8 mm off it) 0, 50 strokes per layer in one spot 0, two spots 0, three zones
0, the four sectors without the center 1.0; taps 10/12 and 15/15 1.0, 9/12, 12/9, 16/12 and 12/0 0; first layer
tapped before rodding 0, or after the second layer went in 0; taps hovering 1 cm short of the mold 0; the scripted
demo replayed for both arms with perfect tracking 1.0 (lifts 0 to 4.1 and 4.1 to 8.2 cm, 25 strokes and 12 taps
each, 3,274 steps, at least 20 s under the limit; the held rod's tip at a stroke's bottom modeled 9 mm and 8 mm off where the hand aims, as in the sim), and the two hands never within 12 cm of each other in y while
within 15 cm.

## Real-world build

- Mold: 1-1/2 in Schedule 40 PVC pipe (40.9 mm inside) cut to 8.1 cm and solvent-welded to a PVC cap or a flat
  3 mm PVC disc, painted black; or a 2 x 4 in plastic cylinder mold (sold for grout and mortar specimens) if the
  task is rebuilt at 1:2.
- Rod: 3/16 in (4.8 mm) smooth steel rod, one end ground round, 8.6 cm below a printed or wooden 3 x 3 x 2.5 cm
  grip block; two bands of yellow tape as above.
- Rack: a wooden post with two 12 mm wide hardwood prongs 9.8 cm up, a 12 mm slot between them.
- Pan: a 13 cm stainless steel bowl or round cake pan; scoop: a small stainless steel measuring scoop (about 30 mL)
  with its handle sleeved to 2.5 cm and weighted to 50 g.
- Float: a 10 x 4 cm aluminum strip, 4 mm thick, with a post and a printed knob glued on.
- "Concrete": a non-setting stand-in, not cement (wet cement is alkaline and abrasive and would set in the
  grippers' joints): damp masonry sand with about 10% bentonite or kaolin clay by weight, gray-tinted, mixed with
  water to a stiff-plastic consistency once and then made to the same recipe by weight every trial (write the water
  grams down on the first day). The score is procedural (layers, strokes, taps, strike-off), so the stand-in does not
  change it; the real check can also weigh the filled mold (a well-rodded specimen is heavier). A plastic sheet under
  everything. Safe for the arms: no sharp or powered tools, nothing that sets.

## What makes it distinct

The ability scored is consolidating a specimen by a counted procedure: placing material in two lifts of the right
depth, rodding each lift a set number of times spread over its cross-section and to a set depth (the second into the
first), tapping between lifts, and only then striking off. Striking off a surface, tamping a single dose, setting
concrete-trade parts flush, or loading a finished specimen into a test machine each exercise one step at most; here the
strike-off is the last step, and a flat, full top scores nothing unless the filling and rodding before it were right.

## Sources

- NICET, "Performance Examination - Concrete: Standard Practice for Making and Curing Concrete Test Specimens in the
  Field (ASTM C31/C31M-17) [AASHTO T 23-18]", checklist, 09/2020: for a 4 x 8 in mold the first layer is about half
  the volume, rodded 25 times throughout its depth without damaging the bottom; the second fills the mold and is
  rodded 25 times penetrating about 1 in into the layer below; roddings distributed uniformly; 10 to 15 light taps
  per layer; under- and overfilled molds adjusted; excess struck off with the rod, a float or a trowel to an even,
  flat surface; a 3/8 in rod for molds under 6 in.
- Iowa DOT Instructional Memorandum 315 (AASHTO T 23): 4 x 8 in cylinders in 2 equal layers, a 3/8 in rod, 25
  roddings per layer, the bottom layer rodded through its depth, each upper layer about 1 in into the one below.
