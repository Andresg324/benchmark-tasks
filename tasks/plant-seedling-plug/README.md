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

Render the videos of a run with `inspect-robots video <log.json>`.

Offline checks (no simulator): `python3 test_task.py`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the end of
the run. Everything is read from the end state:

| Measure | How | Full credit | Why this much |
|---|---|---|---|
| Plug in the hole | plug center within the hole's 5 cm radius, below the surface | yes | |
| Upright | tilt of the plug's axis | 10 degrees or less (20 for 0.5 and 0.25) | a lean you see at a glance |
| Depth | plug top vs the firmed surface | within 1 cm | deeper rots the stem, proud dries the plug |
| Soil level, each side | mean height of that side's 4 probe points (each capped at the surface) | within 1 cm of the plug's top on all 4 sides (3 for 0.5) | the root ball covered all round, no air pocket |
| Fill | mean over the 16 probe points of the soil's rise from the hole's floor / the hole's 6.2 cm depth, each capped at 1 | 85% (60% for 0.5, 40% for 0.25) | the gap filled up, not one side heaped |
| Spill | clods out of the planter box | 10% or less, else capped at 0.5 | |
| Hands off | each gripper's own shape at least 1 cm from the plug | yes (for 1.0, 0.75, 0.5) | planted means let go |
| Firm | 0.5 N side push 5 cm above the plug's top, 1 s each way, after 1 s standing still | turns under 5 degrees | our estimate of a gardener's check nudge (the weight of 50 g); a packed backfill held it in the simulator, a loose one did not |
| Undamaged (whole trial) | stem bend at each of its two joints; the stem or a leaf touched by both fingers of one hand at once | never past 45 degrees, never squeezed (2 control steps running confirm either) | a creased stem or crushed leaf sets the plant back or kills it |

Possible scores: 0, 0.25, 0.5, 0.75 (planted and filled, not firm), 1.0. Doing nothing scores 0. A damaged seedling
scores 0 for the whole trial, however well it is planted.

## Measuring the soil: 16 probe points

The fill and the side levels are read the same way in the simulator and in a real trial: the height of the soil at 16
fixed points, found by lowering a probe until it rests on the mix. This is a pin profile meter, the usual tool for
soil-surface heights: "a series of equally spaced pins that are lowered onto the soil surface until contact is made"
(Raper, Grift and Tekeste 2004, A portable tillage profiler for measuring subsoiling disruption, Transactions of the
ASAE), with an accuracy "better than plus or minus 1 mm" for purpose-built meters (Hirschi, Barfield, Moore and
Colliver 1987, Profile meters for detailed measurement of soil surface heights, Applied Engineering in Agriculture
3(1):47-51). Pinboards in soil-roughness work use pins of about 3 mm on 18.5 mm spacing (Thomsen et al. 2015,
SOIL 1:399-410); ours is a single 9.5 mm (3/8 in) flat-ended dowel moved from hole to hole, wider so it rests on the
crumbs of loose mix instead of slipping between them, and read against a steel rule with 0.5 mm marks (a steel rule's
resolution and accuracy are both 0.5 mm: NPL, UNCERT Manual 2000, Section 3, Kandil).

- **Points.** A template on the box's top edge (a stiff card 33.6 cm square, the box's outside size, slotted from one
  edge to the center so it slides in round the stem under the leaves) has 16 holes on a ring 4.4 cm from the hole's
  center: four beside each face of the plug, 10 and 30 degrees either side of the face's middle. Each lands 1.0-1.9 cm
  out from the plug's face and 0.8 cm in from the hole's wall, 1.5 cm from its nearest neighbor. Three more holes sit
  over the root ball, 1.8 cm from the stem (at 15, 60 and 240 degrees from +x, clear of the seed leaves).
- **Reference.** Every reading is a depth below the template, i.e. below the box's top edge, which does not move. The
  firmed surface is 1.0 cm below the edge (set at reset) and the hole's floor 7.2 cm below it (the former's depth
  stop). Before each trial the 16 holes are read in the empty hole; a reading off by more than 3 mm means re-dig.
- **Reading.** Lower the dowel through the hole until it rests on the mix under its own weight; never push. Hold any
  leaf aside by hand. Read the depth to 0.5 mm, lift, lower again, read again, and average.
- **Fill.** Point fill = (empty reading - final reading) / (empty reading - 1.0 cm), capped at 0 and 1. The fill is the
  mean of the 16. Thresholds in backfill height: 40% = 2.5 cm, 60% = 3.7 cm, 85% = 5.3 cm (0.9 cm under the surface).
- **Sides.** A side's level is the mean of its four points (each no higher than the surface), compared with the plug's
  top, the mean of the three root-ball readings (mix brushed off there). The plane through those three gives the
  tilt: 1 mm on one of them is about 2 degrees.

Why heights, not weights: the volume a weighed mass of moist mix fills depends on how hard it was firmed, which the
robot sets. A height at a point needs no density: it is where the soil surface is. Mix heaped above the surface
counts as full and no more, so a mound on one side cannot make up for a hollow on another. The readings land in the
gap, not on the plug, for any plug within 4 mm of the hole's center.

Resolution. With a 9.5 mm probe on 1.1 cm clods, a clod beside an empty pocket reaches 5.5 mm into it, so in the
simulator a pocket shorter than about 3.5 cm along a face can be bridged; any pocket 3.6 cm long, anywhere along any
face, drops a point and fails that side (offline check). Real mix crumbs are smaller. With the plug's top at the
surface, three filled sides already give a fill over 60%, so the 60% threshold binds only for a plug set deeper.

In the simulator the probe is a 9.5 mm disc at the same 16 points that comes down until any part of its end meets a
clod (each clod's own turned cube) or the plug, else the hole's floor. The plug's tilt and top come from its pose.

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
- Kink (an estimate derived from the stretch at the stem's surface, not a measured kink angle): 45 degrees at a
  joint, i.e. within 4.5 cm of stem: a bend radius of 5.7 cm, which stretches the stem's surface
  about 3%, sharper than a seedling bends in a breeze or under brushing, which it springs back from. The 0.5 N push
  bends it about 13 degrees at each joint.
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
- Kink limit: derived from the stretch at the stem's surface, not a measured kink angle.
- Side push: applied to the root ball with the force and moment a push on the stem hands on.

## Real-world build

- Planter box: a wooden planter box, 30 x 30 cm inside and 9 cm deep, filled with moist peat-perlite potting mix firmed level
  1 cm under its top edge. Dig the hole (as a gardener would with a trowel; the former makes it repeatable) with a 10.4 cm round former (a printed cylinder with a depth stop at 6.2 cm).
- Loose mix: the dug mix, 200 g (about 440 mL), weighed, crumbled and heaped round the hole. The gap round a level
  plug is about 360 mL.
- Seedling (held by the root ball, never the stem): a sweet pepper transplant grown in a 3601 insert (cell 4.9 x
  5.7 cm at the top, 5.7 cm deep, 118 mL), the stem 3-4 mm across and 8-10 cm tall above the plug, with its seed
  leaves and four true leaves; weigh five and use those within 60-80 g. Set it with its leaves toward the robot's
  right.
- Template and probe: a stiff card 33.6 cm square with the 19 holes (see Measuring the soil; print it at 1:1), a
  9.5 mm (3/8 in) hardwood dowel 20 cm long with a square-cut end, and a 15 cm steel rule with 0.5 mm marks.
- Scoring a real trial (the same end state): photograph the seedling; read the 16 points and the three root-ball
  points through the template (fill, side levels, the plug's top and tilt); then push a 3 mm bamboo skewer 3 cm into
  the root ball 1.5 cm from the stem, zero a phone inclinometer on it, and push with a 1 N spring gauge (0.01 N
  divisions) 5 cm above the plug's top, 1 s each way, reading the turn; gather the mix off the bench and floor and
  weigh it on a kitchen scale (1 g) against the 200 g loaded (spill: a direct mass, no conversion; mix left on the
  surface outside the hole is neither spill nor fill); photograph again: a creased, kinked or broken stem, a crushed
  or torn leaf, or a root ball broken apart, scores 0.
- Safety: nothing hazardous; wear gloves if the mix contains fertilizer.

## Runtime and seeds

- Demo: 2,234 control steps (112 s simulated); it ends once the full score (with the firmness push passed) has held
  for 1 s. The plan is 113 s (2,266 steps).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- One fixed layout (`object_jitter=0`); the clods' layout comes from a fixed seed. The clods are spawned outside
  the task's object list so object jitter never piles them together.

## Files

- `task.py`: scene, scorer, scripted demo.
- `yamkit.py`, `mechkit.py`, `soilkit.py`, `pushkit.py`, `guardkit.py`: the task's helper modules.
- `test_task.py`: offline checks of the layout, the probe points (empty hole, heaped mix, one side only, a pocket
  between points, the 40/60/85% boundaries), the scorer and the demo plan.
- `task.json`: the task's text.
