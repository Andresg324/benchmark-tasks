# Plant a pepper seedling's root plug in a planter box, backfilled level on every side and firmed so it stands alone

A task for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). `task.py` builds the scene,
scores it and holds the scripted demo; its docstrings explain every rule. The task's helper modules:

- `yamkit.py` spawns the modeled props as visible meshes over simple box and cylinder colliders: the planter box with
  its planting hole, and the seedling as one articulated body (the root ball, and the stem and leaves on spring joints).
- `mechkit.py` holds the spring law that drives the stem's joints.
- `soilkit.py` spawns the loose potting mix as 200 separate rigid clods and gives the scorer their positions and
  orientations.
- `pushkit.py` applies the firmness test's side push to the plug when the scorer asks for it.
- `guardkit.py` checks whether a gripper touches the plug.

Each patches `yam_arena.scene` in memory at import and changes no files.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/plant-seedling-plug/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Plant the seedling in the planter box, level with the soil, and firm it in so it stands by itself." \
      --scorer task_score --max-steps 2400

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Results on this task.py: scripted demo 1.0 in 2,234 steps (success; the plug ended 0.12 cm off the hole's axis and
1.1 degrees off upright, inside the placement range); Dream-1 scores 0 (2,400 steps) on the same task.py.

Render the videos of a run with `inspect-robots video <log.json>`.

Offline checks (no simulator): `python3 test_task.py`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the end of
the run. Everything is read from the end state:

| Measure | How | Full credit | Why this much |
|---|---|---|---|
| Plug placed in range (every tier) | its top's center from the hole's axis, in plan; any turn about its own axis | within 0.5 cm, else 0 | inside it every probe lands in the gap (see Measuring the soil) |
| Upright (every tier) | tilt of the plug's axis | 10 degrees or less, else 0 | a lean you see at a glance; more would bring the plug's lower side under the probes |
| Depth | plug top vs the firmed surface | within 1 cm | deeper rots the stem, proud dries the plug |
| Soil level, each side | mean height of that side's 4 probe points (each capped at the surface) | within 1 cm of the plug's top on all 4 sides (3 for 0.5) | the root ball covered all round, no air pocket |
| Fill | mean over the 16 probe points of the soil's rise from the hole's floor / the hole's 6.2 cm depth, each capped at 1 | 85% (60% for 0.5, 40% for 0.25) | the gap filled up, not one side heaped |
| Spill | clods out of the planter box | 10% or less, else capped at 0.5 | |
| Hands off | each gripper's own shape at least 1 cm from the plug | yes (for 1.0, 0.75, 0.5) | planted means let go |
| Firm | a side push of the weight of 50 g, 5 cm above the plug's top, 1 s each way, after 1 s standing still | turns under 5 degrees | a gardener's light check nudge (protocol setting); a packed backfill held it in the simulator, a loose one did not |
| Undamaged (whole trial) | stem bend at each of its two joints; the stem or a leaf touched by both fingers of one hand at once | never past 45 degrees, never squeezed (2 control steps running confirm either) | a creased stem or crushed leaf sets the plant back or kills it |

Possible scores: 0, 0.25, 0.5, 0.75 (planted and filled, not firm), 1.0. Doing nothing scores 0. A damaged seedling
scores 0 for the whole trial, however well it is planted.

## Measuring the soil: 16 probe points set out from the plug

The fill and the side levels are read the same way in the simulator and in a real trial: the height of the soil at 16
points round the plug, found by lowering a probe until it rests on the mix. This is a pin profile meter, the usual tool
for soil-surface heights: "a series of equally spaced pins that are lowered onto the soil surface until contact is
made" (Raper, Grift and Tekeste 2004, A portable tillage profiler for measuring subsoiling disruption, Transactions of
the ASAE), with an accuracy "better than plus or minus 1 mm" for purpose-built meters (Hirschi, Barfield, Moore and
Colliver 1987, Profile meters for detailed measurement of soil surface heights, Applied Engineering in Agriculture
3(1):47-51). Pinboards in soil-roughness work use pins of about 3 mm on 18.5 mm spacing (Thomsen et al. 2015,
SOIL 1:399-410); ours is a single 9.5 mm (3/8 in) steel dowel pin set on the mix at each point in turn, wider so it
rests on the crumbs of loose mix instead of slipping between them, and its top read with a digital depth gauge from a
pair of steel flat bars laid across the box's top edge (tools below). Every tool is a catalog item: nothing is made,
printed or drilled.

**The points follow the plug.** They are set out from the plug's own faces, not from the box, so wherever the plug
stands and however it is turned they land in the gap beside it:

- Four points beside each face, 1.0 cm out from the face (the probe's 9.5 mm end 5.25 mm clear of it), at 15% and 50%
  of the way from the face's middle toward either end. Points on a face are 0.7-1.0 cm apart.
- In the plug's frame (cm from the stem's axis, x across the 4.9 cm width): (+-3.45, +-0.43), (+-3.45, +-1.43),
  (+-0.37, +-3.85), (+-1.23, +-3.85).
- Three more points over the root ball, 1.8 cm from the stem at 15, 60 and 240 degrees from the plug's +x: always on
  the root ball (their 9.5 mm ends at most 2.3 cm from the stem; the root ball's top reaches 2.45 cm).
- A tilted plug: up to 10 degrees, its lower side comes at most 3.5 mm past its top's outline (4.5 mm for the
  simulator's three stepped layers), inside the 5.25 mm clearance.

**The placement range.** Every tier needs the plug's top centered within 0.5 cm of the hole's axis (in plan, turned
any way about its own axis) and upright within 10 degrees; outside it the trial scores 0, whatever the soil. Inside
it, every probe lands wholly inside the hole's wall, never on its edge. The hole is 10.3 cm across (5.15 cm radius),
in the simulator and in a real trial (the can that cores it). The farthest point is 4.04 cm from the plug's center,
so 4.04 + 0.475 (the probe's radius) + 0.5 = 5.015 cm: 1.35 mm inside the wall. Setting a pin by the rule's 0.5 mm
marks takes 0.5 mm of that, and reading the offset 0.5 mm more (a plug read as 0.5 cm off may be 0.55 cm off), so in
the worst case the pin's edge is still 0.35 mm inside the wall.

Checked offline for 8,424 placements in the range: the top's center 0, 0.25 cm, or 0.5 cm off the axis in 24
directions; turned every 10 degrees; upright or tilted 10 degrees eight ways. In every one, an empty hole reads its
floor at all 16 points, and every probe's end stays 1.35 mm or more inside the hole's wall: no probe touches the plug
or the hole's edge. A hole filled round a plug 0.5 cm off-center and turned 0-137 degrees scores the same as a
centered one (1.0 after the push); one bare side is still caught (0.5); a plug 0.55 cm or more off the axis, or tilted
11 degrees, scores 0.

**The tools** (all off the shelf, low cost; one set serves every trial; see Cost and accuracy below):

| Tool | Product (checked on a maker's or dealer's page) | Use | Resolution / accuracy |
|---|---|---|---|
| Height reference, x2 | Everbilt plain steel flat bar, 1-1/2 x 36 in, 1/4 in thick (38 x 914 x 6.35 mm), model 801827 (Home Depot) | laid flat and parallel across the box's top edge, 12 mm apart | sag 0.05 mm at most, own weight plus the gauge (computed: steel, E 200 GPa, 38 x 6.35 mm, span up to 42 cm on a diagonal); bow: see Cost and accuracy |
| Depth gauge | iGaging OriginCal IP54 digital depth caliper, 0-6 in (150 mm), model 100-700-34 (Penn Tool Co.) | base across both bars, rod lowered onto the pin's top | 0.01 mm display, +-0.025 mm (0.001 in, maker); recorded to 0.1 mm |
| Probe foot | 0.375 x 0.375 in (9.5 x 9.5 mm) alloy steel dowel pin to ASME B18.8.2 (AFT Fasteners PINDOW030003P; the B18.8.2 chamfered end has a flat face 0.350-0.365 in across, Aspen Fasteners 0.375 in listing) | set chamfered (flat) end down on the mix at each point; about 5 g, so it rests without sinking in | its 9.5 mm face is the simulator's 9.5 mm probe |
| Rule | any flush-ended 300 mm steel rule with 0.5 mm marks (e.g. Shinwa 24612, Workshop Heaven listing) | sets each point from the plug's faces; measures the plug's offset and the bars' gap | 0.5 mm |
| Square | any 3 in (75 mm) machinist's engineer square | laid flat on the plug's top, beam on a face's top edge, blade square to the face | 0.5 mm over the 20 mm it is used |
| Brush | any soft 2.5 cm paintbrush | brushes mix off the plug's top edges onto its top | |
| Hole corer | an empty 401 x 411 (No. 2.5) food can, lid cut off | cores the planting hole | 10.3 cm across, by the can-size code |
| Scale | American Weigh Scales SC-2KG (maker's SC-series manual) | weighs the water, the loose mix, the spill and the seedlings | 0.1 g |
| Push | a 100 g x 1 g tubular spring scale (e.g. Pesola 20100 Micro-Line), and a cotton thread | the firmness check: drawn level to 50 g; checked once at 50 g against the scale with 50 g of water | 1 g |
| Gripper covers | Dukal Tech-Med 4404XL nitrile finger cots, powder-free, about 0.08 mm thick, 144 per box | one rolled over each gripper finger's pad, new each trial (see Keeping the robot clean) | |
| Drop cloth | a clear plastic painter's drop cloth, 0.7 mil | under the planter box and round it on the bench | |

The reference for every height is the bars' top, resting on the box's top edge, which does not move as the mix is
firmed. With the pin on the firmed surface the gauge reads about 6.9 mm (6.35 + 10 - 9.5) and on the hole's floor
about 68.9 mm, inside the gauge's 150 mm. The pin's top stands at most 6.9 mm above the firmed surface before it meets
the gauge's base; a point where the mix is higher than that is full (fill 1, level at the surface), which is what the
cap gives it anyway. A 12 mm gap passes the gauge's rod and the 3.6 mm stem; the gauge's base needs only to reach
1 cm onto each bar (32 mm in all), which any depth caliper's base does.

**Setting a point.** Lay the square flat on the plug's top, its beam along the face's top edge (brush loose mix off
that edge first, onto the plug's top, never into the gap), its blade out over the gap. Slide it until the blade's edge
is 4.75 mm (half the pin) beyond the point along the face: 8.5 and 17.0 mm each side of the face's middle on the
4.9 cm faces, 9.0 and 19.0 mm on the 5.7 cm faces (the middle from the face's length on the rule). Set the pin against
the blade on the middle's side, its near side 5.25 mm out from the face (the rule laid along the blade, its flush end
at the face: the pin spans 5.25-14.75 mm), and let go. The three root-ball points are set the same way from two faces:
7.1 mm in from one 5.7 cm face (call it +x) and 4.7 mm from its middle toward +y; 12.9 mm in from the +y face and
9.0 mm from its middle toward +x; 12.9 mm in from the -y face and 9.0 mm from its middle toward -x.

**Reading a point.** Lay the two bars flat across the box's top edge, parallel and 12 mm apart (checked on the rule at
both walls), with the stem and the pin on the line midway between them (the stem passes up through the gap, the leaves
stay above the 6.35 mm steel; hold a leaf aside by hand if it hangs in the way). Lay each bar the same face up every
time (a marker dot on its end). Set the gauge's base across
both, over the pin, and lower its rod until it just touches the pin's top; never push the pin. Lift the pin off and go
on: set it at the next point, and pivot the pair about the stem to it. One pin is used, one point at a time (adjacent
points on the 4.9 cm faces are 7.4-8.6 mm apart, closer than the pin's 9.5 mm, so pins cannot stand at all of them at
once). A point takes about 25 s: 10 s to set the pin, 10 s to lay the pair and read, 5 s to lift it. A pin that a bar rests on, or that stops the gauge's base, stands
in mix heaped above the surface: that point is full.

**A real reading, step by step** (the same tools, references, order and tolerances in every trial):

1. At reset: measure the hole's center, the gap from each wall's inner face to the hole's edge (square and rule):
   hx = (h-x - h+x) / 2, likewise hy, recorded. Read S, the firmed surface, at 4 points 7.5 cm out from the hole's
   center along the box's x and y, and the hole's floor at 4 points 3.5 cm from its center, between them. Each floor
   reading must be S + 62 mm within 3 mm, else re-core; their mean is E. About 2 minutes.
2. After the trial: photograph the seedling. The readings start within 2 minutes of the trial's end and are done
   within 12 minutes of it (about 9 minutes of work), before the firmness push.
3. Placement: lay the square flat on the firmed surface, its beam against a wall's inner face, and the rule along its
   blade; slide them along the wall to find the plug's top edge or corner nearest that wall and read the gap. Do it
   from all four walls: the plug's center is px = (g-x - g+x) / 2, py likewise (a rectangle's center is midway between
   its extremes, at any turn), and its offset is sqrt((px - hx)^2 + (py - hy)^2). About 1 minute.
4. The 19 points (16 soil points, 3 on the root ball), one at a time: set the pin, read once, lift it. About 8 minutes.
5. The three root-ball readings: their mean P is the plug's top; the plane through them gives the tilt (1 mm on one of
   them is about 2 degrees). An offset over 0.5 cm or a tilt over 10 degrees: the trial scores 0. Point fill =
   (E - reading) / (E - S), capped at 0 and 1 (a reading at or under S is full). The fill is the mean of the 16.
   Thresholds in backfill height: 40% = 2.5 cm, 60% = 3.7 cm, 85% = 5.3 cm (0.9 cm under the surface).
6. Depth and sides: the plug's top is within 1 cm of the surface if P - S is within +-10 mm. A side's level is the mean
   of its four readings, each taken as S if under it; the side is level if it is within 10 mm of P.

**Tolerances.**

| What | Target | Tolerance | How checked | Source |
|---|---|---|---|---|
| Room | room temperature | 20 to 24 degrees C (68 to 75 degrees F) | room thermometer at the bench | protocol setting |
| Mix moisture | 350 g water per liter of bagged mix, rested 1 h | 1 g of water | the scale | protocol setting |
| Loose mix | 200 g, heaped round the hole | 1 g | the scale | protocol setting |
| Seedling | a store-bought transplant with one upright stem (primary: a Bonnie Plants pepper six-pack; any vegetable start that passes the checks), in a six-pack of the 3601 or 606 insert, one flat bought on one day per series, used within 2 weeks, a fresh plant every trial | cell: one plug per flat 4.9 x 5.7 cm and 5.7 cm deep within 3 mm; stem 3-4 mm across, 8-10 cm above the plug; 3-6 true leaves, each 8 cm or shorter; 60-80 g | the rule, the scale | cell size (maker); stem, leaves and weight from the simulated plant (protocol setting) |
| Hole | 10.3 cm across (the can), 62 mm deep | each of two diameters at least 10.3 cm; depth 3 mm at each of 4 floor points | depth gauge, step 1 | can-size code; depth protocol setting |
| Hole center | recorded, not set | 0.5 mm as read | square and rule, step 1 | protocol setting |
| Pin placement | the points above | 0.5 mm (inside the 1.35 mm margin) | square and rule | protocol setting; rule's marks (maker) |
| Plug offset | at most 0.5 cm | 0.5 mm (two 0.5 mm gaps, halved) | square and rule, step 3 | the placement range (scorer) |
| Heights | as read | 0.1 mm recorded; error 0.3 mm at most (see Cost and accuracy) | depth gauge | gauge +-0.025 mm (maker); sag computed; bow measured (at most 1 mm, once per series) |
| Reading window | all 19 points, one at a time, and the offset | start within 2 min, done within 12 min of the trial's end | timer | protocol setting |
| Reset | core, measure, heap, set the plant, clean the grippers | about 6 min | timer | protocol setting |

These are small against the 1 cm side-level and depth tolerances and the 0.6 mm of backfill per 1% of fill.

Why heights, not weights: the volume a weighed mass of moist mix fills depends on how hard it was firmed, which the
robot sets. A height at a point needs no density: it is where the soil surface is. Mix heaped above the surface
counts as full and no more, so a mound on one side cannot make up for a hollow on another.

Resolution. With a 9.5 mm probe on 1.1 cm clods, a clod beside an empty pocket reaches 5.5 mm into it. So in the
simulator a 2 cm pocket along a face can be bridged, but any pocket 2.4 cm long, anywhere along any face, drops a point
and fails that side (offline check). Real mix crumbs are smaller. With the plug's top at the surface, three filled
sides already give a fill over 60%, so the 60% threshold binds only for a plug set deeper.

In the simulator the probe is a 9.5 mm disc at the same 16 points, carried with the plug's top (its position and turn).
It comes down until any part of its end meets a clod (each clod's own turned cube) or the plug, else the hole's floor.
The plug's tilt, top and offset come from its pose.

Watering in, the next step of planting, is out of scope; the push test stands in for "stands by itself". The push test starts by itself once the planted state has held for 1 s with both hands off; if a hand comes back to
the plug during it, it stops and starts over later. The score is the end state's: no step of how the robot got there
is scored.

## Why the plug has to be held

In the simulator, 210 clods dropped into the gap round a plug standing loose in the hole knocked it over to 36
degrees: mix that lands on one side first shoves it over. Held upright until the mix is in round it, it stands by
itself afterwards (0-2 degrees after release in the demo runs).

## The seedling model

- Species and planting depth: a pepper transplant, which is set with its stem at the soil line as it grew ("Set pepper
  seedlings out in the garden so that the shoots are at the soil line as they were before transplanting": UMN Extension, Growing peppers), so the plug's top goes level with the soil. (Tomatoes, by contrast, are
  usually buried deeper.)
  The simulator models a pepper; the real trial uses any store-bought transplant of the same build and size
  (primary: a Bonnie Plants pepper; see Real-world build), set at the same level, the plug's top level with the soil.
- Stem: 9 cm above the plug, 3.6 mm across (a young plug's; garden-center plants reach a pencil's width, UMN
  Extension), in two 4.5 cm segments. Each joint (at the root ball and at mid-height) bends about both horizontal axes
  with stiffness EI / L, EI = E x pi d^4 / 64: E 600 MPa for the lower segment (the lignified hypocotyl) and 60 MPa for
  the upper (young stem), from the 600 MPa to 10 MPa range measured along young tomato stems (Coutand and Moulia 2000,
  J. Exp. Bot. 51:1825), the nearest seedling stem of the same family we found measured. That is 0.110 and 0.011
  N m/rad; damped about critically.
- Shape: four true leaves, the low end of the 4-6 young true leaves of a transplant-ready pepper (UMass Extension,
  Peppers: growing tips), closely spaced up the stem as on a sturdy transplant (UMN Extension). Pepper leaves are
  simple: a petiole and an ovate blade. Real leaves alternate round the stem; here all four face the robot's right, the
  side the hands do not hold the plug from.
- Seed leaves (cotyledons): two narrow plates, 1.8 x 0.5 cm, 2 cm up the stem on opposite sides (toward -30 and 150
  degrees from +x), part of the lower stem's body; they stay inside the root ball's outline, 6 mm clear of the fingers
  that hold it.
- True leaves 1-3: a petiole about 1.8 cm long and an ovate blade up to 2.6 cm wide, widest below its middle; 6.5,
  6.5 and 5 cm long in all, 1 g each, 3.2, 4.2 and 3.7 cm up the stem toward -75, -105 and -90
  degrees. Each hangs on
  a two-axis spring joint at its stalk (0.008 N m/rad each way: a level leaf sags 2.5 degrees; folding up to 86
  degrees either way), so a hand brushing or pressing it swings it aside or folds it down; a leaf pushed end-on pushes
  the stem.
- True leaf 4 (the youngest): 2.5 cm long and 1.2 cm wide, 0.3 g, 7 cm up the stem toward -100 degrees, on one spring
  joint about the axis across it (0.004 N m/rad), so it folds up or down when pressed. The simulator cannot step a
  two-axis leaf joint on the upper stem (seven bodies deep, it blew up at spawn), so the three larger leaves sit on the
  lower segment. Above it the stem ends in a 6 mm growing tip.
- The stem and leaves weigh about 5.5 g.
- Kink limit: 45 degrees at a joint, i.e. within 4.5 cm of stem: a bend radius of 5.7 cm, which stretches the stem's
  surface about 3% (computed from the stem's 3.6 mm width), sharper than a seedling bends in a breeze or under
  brushing, which it springs back from. The firmness push (the weight of 50 g) bends it about 13 degrees at each joint.
- Squeeze: the YAM gripper is a stiff position servo (5000 N/m per finger), so the stem or a leaf touching both fingers
  at once is pressed between them with far more than a light touch.
- The side push acts on the root ball, with the force and the moment that a push on the stem 5 cm above the plug's top
  hands on to it.

## Fidelity

- Real size and mass. The plug is a rigid tapered block; the stem and all its leaves are real bodies on it that the hands
  and the soil hit.
- Stem and leaves: the stem is two segments on two-axis spring joints, with stiffness from the bending stiffness measured
  along young tomato stems, the nearest measured relative (see The seedling model). Each leaf is made of flat plates (petiole and blade strips). The three lower true leaves
  swing round the stem and fold up or down on their stalks' springs; the young top leaf only folds; the seed leaves are rigid on the stem. All
  the leaves face one side, clear of the holding hand (a real plant's alternate round the stem).
- Loose mix: 200 rigid 1.1 cm clods (1 g each, grippy, damped so they do not roll), so a heap holds its slope. The
  firmed mix and the hole are fixed colliders. The clods cannot compact, so firmness in the simulator comes from
  packing the gap; the real trial with real mix is the true test.
- Root ball: rigid; it cannot break apart in the simulator (the real trial checks it on the photos).
- Soil readings: the probe is a 9.5 mm disc dropped onto the clods' cubes at the 16 points (see Measuring the soil).
- Squeeze: scored as both fingers touching the stem or a leaf at once (the gripper is a stiff position servo, so that
  is more than a light touch).
- Kink limit: set from the stretch at the stem's surface: 45 degrees within 4.5 cm stretches it about 3%.
- Side push: applied to the root ball with the force and moment a push on the stem hands on.

## Real-world build

- Room: 20 to 24 degrees C (68 to 75 degrees F).
- Planter box: a wooden planter box, 30 x 30 cm inside and 9 cm deep.
- Potting mix: PRO-MIX HP Mycorrhizae (PT Horticulture: 65-75% Canadian sphagnum peat by volume, perlite, limestone,
  a wetting agent). Moisture, the protocol's setting: 350 g of water per liter of mix as it comes from the bag, stirred
  in a lidded tub and left 1 h before use; the same tub serves the series. Firmed level 1 cm under the box's top edge.
- Planting hole: cored with an empty 401 x 411 food can (No. 2.5 size, 28-29 oz; the can-size code gives its outside
  diameter, 4 1/16 in = 10.3 cm), its lid cut off with a can opener: pressed open end down to 62 mm (within 3 mm, the
  depth gauge), turned once and lifted out with its core. The hole is 10.3 cm across, as in the simulator: each of two
  diameters at least 10.3 cm on the rule, else re-core. Centered by eye; its actual center is measured and recorded (Measuring the soil, step 1), so centering
  needs no tight tolerance.
- Loose mix: the cored mix, 200 g (about 440 mL), weighed on the scale, crumbled and heaped round the hole. The gap
  round a level plug is about 350 mL.
- Scale: American Weigh Scales SC-2KG, 2000 g x 0.1 g (for the mix, the spill and the seedlings).
- Seedling (held by the root ball, never the stem): a store-bought transplant, bought ready to plant (nothing is
  grown for the task). What the simulator needs is the plug and the shoot's size, not the species: a root ball the size
  of the standard 3601 cell (T.O. Plastics 3601: 1.94 x 2.23 in at the top, 2.23 in deep, 4.05 fl oz, recommended for
  bedding plants; Hummert listing), i.e. 4.9 x 5.7 cm, 5.7 cm deep, 118 mL, the simulator's plug, under one upright
  stem 8-10 cm tall.
  - Primary: a Bonnie Plants pepper six-pack, the plant the simulator models (e.g. Bonnie Plants Green Bell Sweet
    Pepper 6-pack, model 2100, Home Depot listing). A young pepper transplant has the simulated plant's build: one
    upright stem with a few simple true leaves on stalks.
  - Accepted equivalents: any vegetable start in the same six-pack cell (the 3601 insert, or the 606 with the same
    cell, 1.94 x 2.35 x 2.23 in, Greenhouse Megastore) that meets the measurements below. Common ones: Bonnie Plants
    cabbage and collards six-packs (fall and winter), Bonnie Plants tomato or pepper six-packs (spring and summer), and
    broccoli. Between them they cover the year: at least one of these vegetables is normally on sale as six-pack starts
    at garden centers in every season, warm-season starts in spring and summer and cole crops (cabbage, collards,
    broccoli) in late winter and spring and again from late summer into fall, through winter where winters are mild
    (NC State Extension, "Vegetables You Can Plant for Fall": "September is prime time to set out transplants of
    cabbage, kale, collards, broccoli, and cauliflower ... Young plants are readily available from local garden
    centers and nurseries").
  - A fresh plant every trial, popped from its cell in under a minute; a planted seedling is not reused. A series uses
    plants of one variety and grower bought on one day, kept watered and used within 2 weeks.
  - Cell check, every flat: one plug measured with the rule, 4.9 x 5.7 cm across the top and 5.7 cm deep, each within
    3 mm. A flat that fails is not used.
  - Plant check, every trial (rule and scale, under a minute): one upright stem 3-4 mm across at its base and 8-10 cm
    tall above the plug; 3-6 true leaves, none longer than 8 cm with its stalk; the plant with its root ball 60-80 g.
    Set with its largest leaves toward the robot's right. A plant that fails is replaced by the next one in the flat.
- Measuring tools: two Everbilt 38 x 6.35 mm steel flat bars, an iGaging 100-700-34 digital depth caliper, a
  9.5 x 9.5 mm steel dowel pin (0.375 x 0.375 in), a 300 mm steel rule, a 3 in engineer square and a soft 2.5 cm
  paintbrush (see Measuring the soil: The tools).
- Scoring a real trial (the same end state): photograph the seedling; read the placement, the three root-ball points and the
  16 points within the fixed window (Measuring the soil, steps 2-6); then push a 3 mm bamboo skewer 3 cm into
  the root ball 1.5 cm from the stem, zero a phone inclinometer on it, loop a cotton thread round the stem 5 cm above the
  plug's top and draw it level with the 100 g spring scale to a reading of 50 g, 1 s each
  way, reading the turn; gather the mix off the bench and floor and
  weigh it on the scale against the 200 g loaded (spill: a direct mass, no conversion; mix left on the
  surface outside the hole is neither spill nor fill); photograph again: a creased, kinked or broken stem, a crushed
  or torn leaf, or a root ball broken apart, scores 0.
- Safety: nothing hazardous; wear gloves if the mix contains fertilizer.

### Keeping the robot clean

Damp mix must not reach the shared robot's joints, and nothing here changes the manipulation.

- Before each trial: roll a new nitrile finger cot (Dukal Tech-Med 4404XL, powder-free, about 0.08 mm thick) over the
  pad of each gripper finger, four in all. Lay the drop cloth under the planter box and round it on the bench.
- After each trial (after scoring): peel the cots off into a bin; brush any mix off the fingers and wrists with the
  paintbrush onto the drop cloth; wipe the fingers with a damp lint-free wipe and dry them. About 1 minute, part of
  the reset; the mix on the drop cloth is weighed with the spill.
- Grip: a cot adds 0.08 mm to each finger's face, nothing against the gripper's position control and its 50 N limit,
  and nitrile is a high-friction glove material, so the hold on the damp root ball is not weaker (checked once per
  series: a plug held by covered fingers is lifted and shaken gently, without slipping). The simulator models bare
  pads; the scorer reads nothing the covers change.

### Cost and accuracy

The tools above are common low-cost products (two Everbilt flat bars, an iGaging 100-700-34 depth caliper, a dowel
pin, a steel rule, a 3 in square, an American Weigh Scales SC-2KG, a Pesola-type spring scale, Dukal finger cots, a drop
cloth, a brush and an empty can): about $260 in all, excluding the robot, a phone and the consumables (mix, plants).

They clear the thresholds with room to spare:

- Heights: the caliper reads within +-0.025 mm; the bars sag 0.05 mm at most; a bar bowed 1 mm over its 914 mm (measured: once per series each bar is stood on edge on the
  flat table and the largest gap under it read on the rule held upright beside it; a bar with more than 1 mm is
  replaced; laid the same face up each time) moves the reading at the middle by at
  most 0.21 mm on a 42 cm diagonal span and 0.11 mm on a 30 cm one, so 0.1 mm between orientations once S is read the
  same way. In all, about 0.3 mm at most. Against that: a side level and the plug's depth have 10 mm tolerances (3%
  used), and 1% of fill is 0.62 mm of backfill, so a point's fill moves by under 0.5%, against fill thresholds of 40%,
  60% and 85%.
- Placement: the rule and square set a pin within 0.5 mm, inside the 1.35 mm clearance to the hole's wall; the plug's
  offset is read within 0.5 mm against the 5 mm range (see The placement range).
- Firmness: the spring scale reads 1 g at 50 g (2%), checked against the 0.1 g scale.

## Runtime and seeds

- Demo: 2,234 control steps (112 s simulated); it ends once the full score (with the firmness push passed) has held
  for 1 s. The plan is 113 s (2,266 steps).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- One fixed layout (`object_jitter=0`); the clods' layout comes from a fixed seed. The clods are spawned outside
  the task's object list so object jitter never piles them together.

## Files

- `task.py`: scene, scorer, scripted demo.
- `yamkit.py`, `mechkit.py`, `soilkit.py`, `pushkit.py`, `guardkit.py`: the task's helper modules.
- `test_task.py`: offline checks of the layout and the probe points. The points are checked in the gap for 8,424
  placements across the range, off-center, turned and tilted. A filled hole round an off-center, turned plug scores
  full; out of range scores 0. Also checked: an empty hole, heaped mix, one side only, a pocket between points, and the
  40/60/85% boundaries. Then the scorer and the demo plan.
- `task.json`: the task's text.
