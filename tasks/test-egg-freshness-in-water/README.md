# Float-test eggs: lower each into a bowl of water without cracking it, then lift the floating one out into a dish

A clear glass mixing bowl, half full of cold water, stands in front of the robot. Three eggs lie on a folded tea towel
to its left, and an empty glass ramekin stands to its right. The eggs look alike: one is fresh, one is a few weeks
old, one is old enough to float. The real trial uses two fresh eggs and a brown plastic egg weighted to float like
the old one (see Real-world build).

**Instruction:** `Lower each egg into the bowl of water without cracking it, then lift any egg that floats out into the small dish and leave the others in the bowl.`

Runtime per trial: up to 2,400 control steps (2 minutes at 20 Hz). The scripted demo (`EGG_SEED=0`) finishes in
about 45 s of simulated time, which takes about 25-30 minutes on one GPU (the water forces are computed for every egg
at 240 Hz).

## Why this task

- **The handling is the hard part.** An egg is fragile and slippery. It must be picked off the towel with a light
  grip, carried over the rim, and let go only at the water's surface, not dropped onto the glass. The floater must
  then be picked out of the water: a wet, smooth egg standing upright with a few millimeters showing, gripped lightly
  enough not to crack it and firmly enough not to slip as it comes out, and set down in a small dish without a drop.
  An egg can crack if it is let fall onto a hard surface from about a centimeter (Sutanto et al. 2025: more than half
  of eggs dropped 8 mm on end onto a hard support cracked) or squeezed past about 38-45 N (Krivanek et al. 2016).
- **Most people have done it or seen it.** The float test is the usual kitchen check for eggs of unknown age (USU
  Extension: "Sinks and lies flat: The egg is likely fresh." / "Sinks but stands upright: The egg is older but may
  still be safe to eat." / "Floats to the surface: The egg is older"). The floater is set aside.
- **What to take out is decided by the eggs, not by a label.** In the sim the eggs look alike, so the robot finds
  out which egg floats only once it is in the water; the score needs all three tested in the water before any
  lift-out. An egg's shell keeps its volume
  while water evaporates through it, so its air cell at the blunt end grows (USDA Egg Grading Manual: "The air cell is
  normally at the large end of the egg"; "Further increase in the size of the air cell ... is due to evaporation of
  water from the egg"). The egg gets lighter for the same size, and its center of mass moves toward the narrow end.
- **Two hands.** The left hand puts the eggs in from the towel; the right hand takes the floater out to the ramekin.

### Distinct from

- **Egg placement and packing tasks** (eggs into a carton or basket, an egg cracked into a bowl): those move eggs
  between fixed places. Here each egg is let go at a water surface, and one is then picked out of the water: a wet egg
  standing upright with a few millimeters showing, gripped across its breadth under the surface.
- **Retrieving cooked food from a pot** (lifting floating pieces out of water with a spoon): that takes out every
  piece that rises, in a batch, with a tool. Here the robot puts each egg in itself, gently, and lifts out only the
  one that floats, by hand.
- **Sorting by density in industry**: no machinery or sensors here, a bowl of tap water in a home kitchen.
- **External benchmarks**: their egg tasks place or cook eggs (BEHAVIOR-1K hard_boil_an_egg and RoboCasa boil_eggs
  put them in a pot of water); none float-tests eggs or lifts an egg out of water by hand.

Category: **Societal relatability**.

## Real-world build

A low-cost setup: the bowl, the ramekin, a thin non-slip silicone mat under both, a tea towel, a carton of eggs, a
plastic egg, 4.5 mm steel airgun BBs (Daisy .177), modelling clay, nitrile gloves, and common tools (tape, a ruler,
a bamboo skewer, an egg carton).

| Item | What | Size, mass |
|---|---|---|
| Sinkers (2) | US Large hen eggs, brown shells, from a new carton (USDA Large: 24 oz per dozen) | about 56.7 mm long, 42.1 mm across, 57 g |
| Floater (1) | BestPysanky brown hollow plastic dummy nest egg, 2.25 in, weighted inside with steel BBs | 57 mm long, 40 mm across (maker's listing); about 46.4 g |
| Bowl | Anchor Hocking 2.5 qt clear glass mixing bowl (81575L20) | top 8 5/8 in, bottom 4 5/8 in, 4 1/4 in high; 1.2 L of cold tap water, 7.5 cm deep |
| Discard dish | Duralex Ovenchef 8.5 oz clear glass ramekin | top 3 7/8 in, 2 in high |
| Towel | Cotton tea towel, folded | 13 x 22 cm, 4 mm thick |

Positions (cm from the midpoint between the arm bases, x forward, y left): bowl centered at x 32, y 0; towel
centered at x 30, y 23, the eggs' middles at y 16, 23 and 30, long axes along x; ramekin at x 30, y -20.5.
Which towel place holds the floater is shuffled per trial; the operator knows it by feel.

**Why a plastic floater.** Real eggs that float take weeks of aging and keep changing, so the floater is a brown
plastic egg weighted to the sim floater's specific gravity (0.972). Like the sim's floater it floats blunt end up
with about 5 mm showing (sim: 5.2 mm). The two sinkers are real fresh eggs. The plastic egg looks alike (brown,
egg-shaped, the same length) but is plastic: it may look a little glossier, and it cannot crack (see Scoring). Telling
it by look does not skip anything: the score needs all three eggs tested in the water first.

**Preparing the eggs.** Sinkers: two fresh eggs from a new carton, kept in the refrigerator between sessions. A
cracked one is replaced from the carton. Floater, once (about 15 minutes):

1. Stand the plastic egg in an egg carton with its hole up. The hole should be at the blunt end; if it is at the
   narrow end, plug it with clay and tape and make a hole at the blunt end. If a BB will not pass, open the hole a
   little with a drill bit turned by hand.
2. Drop in about 80 BBs, then more a few at a time, floating the egg in a bowl of cold tap water after each few,
   blunt end up with the hole above the water. Stop when it floats with about 10 mm showing.
3. Narrow end down, push small pieces of modelling clay in through the hole with a skewer and press them onto the
   BBs so they cannot shift. Add more small pieces of clay, floating it after each, until 5 to 7 mm shows against a
   ruler. Fill the hole with clay, tape over it and check once more: 4 to 7 mm showing (about 46.4 g in all).

Check before each session (1 minute): put the three eggs in a bowl of cold tap water. Both real eggs sink, and the
floater floats blunt end up with 4 to 7 mm showing. A floater that sits lower has let in water: replace it.

**Protecting the robot.** Lifting the floater puts the fingers about 3 cm into the water. Cover each finger, from
the pad up to the gripper body, with a finger cut from a large nitrile glove or a thin silicone sleeve (if a glove
finger is too short to reach the body), taped to the finger just below the gripper body, clear of the finger's
slide so the fingers still open and close fully. Dry the covers between trials. The operator stops the trial if
water reaches the gripper body.

**Reset.** Lift the three eggs out, dry them and lay them back on the towel at their marks; top the bowl up to
1.2 L of cold water.

## How the eggs are modeled

There is no fluid in the simulator. `floatkit.py` adds the water's forces to each egg before every physics step
(240 Hz). Buoyancy is the weight of the water displaced by the submerged part of the shell, applied at that part's
centroid. The water's level is computed from the bowl's sloped sides and what is in it. Drag is quadratic, with a
drag coefficient of 0.5 (a smooth ellipsoid near Reynolds number 10^4). PhysX does every contact and applies each
egg's weight at its center of mass.

Each egg (`eggmodel.py`) follows Hugelschaffer's egg curve, the egg-shape model used by Narushin et al. 2021:
r(x) = B/2 sqrt((L^2 - 4x^2)/(L^2 - 8wx + 4w^2)). Its length is L 56.7 mm and its breadth B 42.1 mm, the measured means
of 56.7 g eggs in Krivanek et al. 2016, with the widest section set 2.5 mm toward the blunt end. That gives a volume
of 52.54 cm3.

The contents fill the shell except the air cell, which takes the top `cell` mm of the blunt end. One contents density
makes the fresh egg weigh 57.0 g, with a 3 mm air cell (grade AA: 1/8 in or less) and a specific gravity of 1.085
(UF/IFAS VM044: most fresh eggs fall between 1.075 and 1.085). The older egg has an 8 mm air cell (grade B: over
3/16 in), 53.9 g, specific gravity 1.027. The floater has an 11 mm air cell, 51.1 g, specific gravity 0.972. Mass
and center of mass follow from the air cell; nothing else is set per egg.

What each egg does in the water is not scripted. Offline, the least potential energy of weight and buoyancy on a flat
floor under water puts the fresh egg on its side with its blunt end raised about 12 degrees, and the older egg
standing on its narrow end. The floater floats blunt end up with a few millimeters above the surface. The tilt grows
with the air cell, and the egg stands once the cell is about 6 mm deep. In the simulator the eggs do the same, to
within the leaning that contact with a neighbor or the bowl's sloping side gives.

Each egg's collider is a smooth convex hull of 242 points on that curve, in 10 rings of 24. The center of mass and
inertia of its contents are set on the body. Eggshell friction is 0.4 static and 0.35 dynamic.

## Scoring

`score()` and `success()` in `task.py`:

- **0 if at any time** an egg that is not held moves faster than **0.40 m/s** in any direction, or an egg, held or not,
  first touches the glass, the table, the towel or another egg faster than that, or, while touching one of them, has its
  velocity change by more than that in one physics step (rammed into it). 0.40 m/s is the speed of a free fall of 8 mm,
  so this covers an egg let fall, flung, or knocked or rammed into something while held. An egg is held when both
  fingertips of one hand press it from opposite sides. Real trial: the video shows it let go more than 8 mm above the
  water, the glass or the table, thrown, or knocked onto something.
- **0 also if** an egg is pressed harder than **38 N** in 3 of any 4 physics steps (1/80 s), or harder than 76 N in any
  single physics step (1/240 s). 38 N is the lowest rupture force across the egg's equator in Krivanek et al. 2016
  (37.96-45.02 N); Sutanto et al. 2025 report 45 N. The press is the largest of these forces: any one robot link's force
  on the egg, either arm's links together, both arms' links together, and the glass, table, towel and other eggs
  together (what a hand pushing the egg into the glass loads it with). A single step's reading is the impulse averaged over 1/240 s, so
  a sharp knock that averages twice the rupture force over a step peaked higher still. Real trial: these force limits
  are scored in the sim only; a real egg with a visibly cracked shell scores 0. The plastic floater cannot crack, so
  a hard squeeze on it is not caught in the real trial.
- **0 also if** an egg leaves the table, or (real trial) the bowl is knocked off its mat or water spills over its rim.
- Otherwise **0** until every egg has been **tested**: each has sat in the bowl's water, not touched by the robot,
  for 1 s.
- **0.25** once all three have been tested.
- **0.5** and the floater out of the bowl, with the other two lying or standing on the bowl's floor (lowest point
  within 1.2 cm of it) and not touched.
- **0.75** and the floater on the ramekin's floor (lowest point within 1.2 cm of it, its middle inside the ramekin) and
  not touched.
- **1.0** and every egg at rest (under 1 cm/s), with every link of both arms (link1 to the fingertips) 2 cm or more
  from every egg, held 1 s.

A good egg taken out of the bowl, or the floater left in it, stays at 0.25. Touching and clearance are read on the
arms' own collision shapes.

The let-fall and crush checks are read from the simulator every physics step (240 Hz). The speed is each egg's own
velocity, which catches a fall from 8.2 to 10 mm depending on when the fall starts relative to the step. The forces
are the contact force vectors PhysX reports between that egg and each link of each arm, and the egg's net contact
force.

Real trial, with a tape measure and a phone video: watch each egg go in and come out (release height), check the two
real eggs for cracks afterward, measure the floater in the ramekin and the others on the bowl's floor, and time 1 s with the
hands away.

## Run it

```
EGG_SEED=0 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/test-egg-freshness-in-water/task.py -E object_jitter=0 \
  --instruction "Lower each egg into the bowl of water without cracking it, then lift any egg that floats out into the small dish and leave the others in the bowl." \
  --scorer task_score --max-steps 2400
```

`EGG_SEED=<n>` repeats a shuffle of the eggs (0 is the canonical run); unset, each run draws and prints its own. The
scripted demo is a fixed plan for the canonical run (`EGG_SEED=0`) and reads nothing from the simulator: the left hand
puts the three eggs in from the towel, and at 31 s the right hand goes down where that run's floater has settled
(x 34.5, y -4.1 cm), grips it across its breadth just under the surface and sets it in the ramekin. With another
seed the floater goes in elsewhere, and the fixed plan is not written for it. Offline checks: `python test_task.py` (or `python -m pytest -q
test_task.py`).

## Files and kits

- `task.py`: scene, scorer, scripted demo.
- `eggmodel.py`: egg shape, mass, center of mass and inertia from the air cell.
- `floatkit.py`: water forces (buoyancy, drag, a sloped bowl's level).
- `yamkit.py`: modeled props (meshes, hull colliders, center of mass, contact reporting) and gripper states.
- `hullkit.py` and `guardkit.py`: contact and clearance on the arm's own collision shapes.
- `test_task.py`: offline tests of the physics, the scorer, the cheats and the detectors.

## Fidelity

Approximate:
- There are no waves, slosh, added mass or surface tension, and the fingers do not push the water. A real floater
  drifts away from approaching fingers more than it does here.
- Drag uses a drag coefficient of 0.5 (the sinking regime). At slow drift speeds a real egg's drag is higher, so a
  released floater drifts longer in the sim than in a real bowl.
- The bowl and ramekin are rigid glass fixed in place, built from a floor disc and flat sloped panels.
- A crack is not simulated. It is judged from the egg's speed and the contact force on it.
- The contents of each egg are one uniform density, with the shell's mass spread through them.
- Real trial: both sinkers are fresh eggs that lie on their side, while the sim's second sinker is an older egg that
  stands on its narrow end; only floater versus sinker matters for the score. The plastic floater is 5% narrower
  than the sim's (40 vs 42.1 mm) and lighter (about 46.4 vs 51.1 g) at the same specific gravity. Its weight sits low
  in the BBs, so it rights itself more stiffly than a real egg when tipped. Smooth plastic may be more slippery than
  eggshell. The finger covers are not modeled.

## Sources

- USDA AMS, Egg Grading Manual: weight classes (Large: 24 oz per dozen), air cell depth by grade, the air cell at the
  large end, and its growth by evaporation.
- USU Extension (Utah State), "How can I tell if eggs have gone bad?"; Egg Safety Center, "Floating eggs".
- UF/IFAS VM044: egg specific gravity by flotation in salt solutions (1.070-1.090).
- Krivanek et al. 2016, Acta Universitatis Agriculturae et Silviculturae Mendelianae Brunensis 64:1191: egg size and
  rupture forces.
- Narushin, Romanov and Griffin 2021, Annals of the New York Academy of Sciences: egg shape formulas built on
  Hugelschaffer's model.
- Sutanto et al. 2025, Communications Physics 8:182: egg compression and drop tests.
- BestPysanky, "Set of 12 Brown Hollow Plastic Dummy Fake Nest Eggs 2.25 Inches": 57 mm (H) x 40 mm (W), hollow,
  with a hole at one end (https://www.bestpysanky.com/products/2-25-set-of-12-solid-brown-brown-dummy-plastic-eggs;
  also https://www.amazon.com/dp/B00NMT24YG).
