# Plant a pepper seedling's root plug in a planter box, backfilled level on every side and firmed so it stands alone

A task for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). `task.py` builds the scene,
scores it and holds the scripted demo; its docstrings explain every rule. The task's helper modules:

- `yamkit.py` spawns the modeled props as visible meshes over simple box and cylinder colliders: the planter box with
  its planting hole, and the seedling as one articulated body (the root ball, and the stem and leaves on spring joints).
- `mechkit.py` holds the spring law that drives the stem's joints.
- `soilkit.py` spawns the loose potting mix as 200 separate rigid clods and gives the scorer their positions.
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
| Soil level, each side | top of the 3rd highest clod in that side's quarter of the gap | within 1 cm of the plug's top on all 4 sides (3 for 0.5) | the root ball covered all round, no air pocket |
| Fill | clods in the hole / what the filled hole holds (105: 94 measured by dropping clods round the plug in a 10 cm hole, scaled by gap area) | 85% (60% for 0.5, 40% for 0.25) | one tall column of clods is not a fill |
| Spill | clods out of the planter box | 10% or less, else capped at 0.5 | |
| Hands off | each gripper's own shape at least 1 cm from the plug | yes (for 1.0, 0.75, 0.5) | planted means let go |
| Firm | 0.5 N side push 5 cm above the plug's top, 1 s each way, after 1 s standing still | turns under 5 degrees | a breeze or a watering-can stream |
| Undamaged (whole trial) | stem bend at each of its two joints; the stem or a leaf touched by both fingers of one hand at once | never past 45 degrees, never squeezed (2 control steps running confirm either) | a creased stem or crushed leaf sets the plant back or kills it |

Possible scores: 0, 0.25, 0.5, 0.75 (planted and filled, not firm), 1.0. Doing nothing scores 0. A damaged seedling
scores 0 for the whole trial, however well it is planted.

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
- Scoring a real trial (the same end state): mark the four sides with plant labels and read the soil level against the plug's
  top at each with a steel rule; read the tilt with a phone inclinometer on a 15 cm bamboo skewer pushed into the
  plug's center before the trial; push with a 1 N spring gauge 5 cm above the plug's top, each way, and read the
  turn on the inclinometer; collect and weigh the mix on the bench (spill); scoop up and weigh the loose mix left on
  the surface outside the hole and subtract it and the spill from 200 g (fill); photograph the seedling before and
  after: a creased, kinked or broken stem, or a crushed or torn leaf, scores 0.
- Safety: nothing hazardous; wear gloves if the mix contains fertilizer.

## Runtime and seeds

- Demo: 2,233 control steps (112 s simulated); it ends once the full score (with the firmness push passed) has held
  for 1 s. The plan is 113 s (2,266 steps).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- One fixed layout (`object_jitter=0`); the clods' layout comes from a fixed seed. The clods are spawned outside
  the task's object list so object jitter never piles them together.

## Files

- `task.py`: scene, scorer, scripted demo.
- `yamkit.py`, `mechkit.py`, `soilkit.py`, `pushkit.py`, `guardkit.py`: the task's helper modules.
- `test_task.py`: offline checks of the layout, the scorer and the demo plan.
- `task.json`: the task's text.
