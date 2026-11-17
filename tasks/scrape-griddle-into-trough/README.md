# Scrape burnt pancakes off a griddle over the lip into its grease tray with a scraper

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). A range-top add-on griddle
stands on the bench with its grease tray toward the robot. Near the lip, four silver-dollar pancakes were left on the
heat and burnt onto the steel (in real trials, a standard substitute made in about 5 minutes: a black charcoal and
flour paste pressed into the same rings on the plate). The robot takes a 4 in. griddle scraper from the bench,
breaks the pancakes up and pushes them over the lip into the tray, and sets the scraper down. The setup, scoring and fidelity notes are in
`task.json`.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/scrape-griddle-into-trough/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Scrape the griddle clean into the grease tray with the scraper." \
      --scorer task_score --max-steps 2400 --store-frames

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`. Offline checks (no simulator): `python3 test_task.py`.

## Scoring

`--scorer task_score` gives 1.0 when the task succeeds (the full condition held for 1 s) and otherwise `score()` at
the end of the trial. Every share is by mass, of the debris (55.5 g in the simulator), read from where the 88
fragments really are (their poses in the simulator; in a real trial, the three fractions weighed on a 0.01 g balance:
see Real-world evaluation).

| Where a fragment is | Counts as | Why |
|---|---|---|
| Inside the tray's walls (x 7.4 to 15.0, across the plate's width), below the plate's top | in the tray | delivered |
| Outside the griddle's footprint (plate, tray and splash guards) | off the griddle | on the bench or floor, or carried off on the scraper |
| Anywhere else: on the plate, against a guard, on the blade over the griddle, falling into the tray | left on the griddle | not yet delivered |

- **In the tray, by tiers.** 0.25 at 25 % in the tray (about one pancake), 0.5 at 60 %, 0.75 at 90 % with at most
  2 % left on the griddle (about one fragment), and 1.0 (with the set-down below) at 98 % with nothing left on the
  griddle: every fragment is off the plate. A fragment is 0.8 to 1.7 % of the mass, so a single fragment left behind
  scores 0.75. Debris pushed loose but left on the plate is not clean: it is the same debris in a pile.
- **Set down.** 1.0 needs the clean plate and the scraper set down on the bench off the griddle (its origin within
  3.0 cm of the bench top and outside the griddle's footprint: a scraper lying on the plate or dropped in the grease
  tray does not count), released (no gripper's palm or fingers touching it: `touching()` in `guardkit.py`, a contact
  check, not a distance zone) and still (under 1 cm/s and 0.5 degree per step, 10 degrees/s) for 1 s. The job is not
  finished while the scraper is in hand or lying on the plate. The arm's pose is not scored.
- **Off the griddle: capped at 0.25.** More than 2 % of the debris (a fragment or two) off the griddle caps the score
  at 0.25: food thrown onto the line or the floor is a slip and fire hazard, which is why cooks push the pile over the
  lip and lift the blade slowly. Judged from where the fragments are, like the tiers.
- **Gouge: a proxy, capped at 0.5.** While the scraper is held and sliding with its edge on the plate (within 0.6 mm
  of the plate's top, about the simulator's resting gap; 1.0 mm once a stroke has begun), with the blade steeper than
  70 degrees to the plate, or its edge tilted more than 10 degrees out of the plate's plane so only one corner touches,
  sliding 1 cm or more in a stroke, the score is capped at 0.5 for the rest of the trial (`scrapekit.py`, with no soil
  in it: only its blade-angle rule is used). The simulation has no scratches; this stands in for the chatter marks
  and scored lines a steep or cornered blade leaves in a griddle's surface. In a real trial the cap is judged on the
  bare steel plate itself: a new scratch in the steel 1 cm or longer, against a photo taken before the trial under a
  raking light.
- Doing nothing scores 0. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

## Real-world evaluation (reproducible protocol)

Every real trial starts from the same measured state and is scored on the same shares as the simulator, read by
weighing. The protocol fixes four things: the debris (a standard substitute made in about 5 minutes: a black charcoal
paste mixed by measuring spoons from two named powders and tap water, weighed into the same four cavities of one named
mold and pressed onto the bare plate, checked disc by disc for mass and area), the positions (stops and marks on the
bench and the plate), the outcome measurement (three weighed fractions plus photos) and the reset. The tolerances, each
with its source, are collected in the table at the end; the equipment, with its makers' or retailers' pages, is under
Real-world build.

**The scene.** At breakfast a cook pours silver-dollar pancakes in a pancake mold on the griddle; four are left on
the heat while an order goes out and burn onto the plate. A cook clears them before the next order by setting the
scraper's edge flat on the plate behind the burnt food and pushing it toward the grease tray in overlapping strokes,
then over the lip (WebstaurantStore's griddle-cleaning guide: scrape debris into the griddle's waste drawer,
https://www.webstaurantstore.com/article/142/how-to-clean-a-griddle.html). The trial runs cold: the robot and the
operator work next to a plate at room temperature, and every trial starts at the same temperature.

**The debris: a standard substitute for burnt pancakes, made in minutes.** Real burnt pancakes take about 2 hours a
batch to cook, burn and cool. Each trial instead uses four discs of a black charcoal paste pressed onto the plate in
the same four cavities of the same mold. Nothing is heated, cooked or left to cool, so a fresh set takes about 5
minutes:
- The recipe, every trial: 6 level US tablespoons of Gold Medal all-purpose flour, 2 level tablespoons of Zen Charcoal
  activated charcoal powder (food grade, from coconut shells) and 3 tablespoons of cold tap water, measured with one
  set of OXO Good Grips measuring spoons (powders spooned in and leveled with the back of a knife, not packed) and
  stirred in a small bowl with a silicone spatula for 60 s, until no dry powder or white streak is left. This makes
  about 100 g, enough for the four 14.0 g discs. It is made fresh for each trial and the rest is thrown away.
- Why it sticks: wet wheat flour is a starch and gluten paste, the same paste that glues posters to walls and makes
  dough stick to a steel work table. Pressed onto clean, dry, bare steel it wets the steel and holds on: it does not
  slide when the plate is touched or brushed, and it comes off only when a blade's edge is driven under it, as burnt
  food does. The charcoal makes it black (its area can be measured on the photo, and it looks like char) and short:
  the paste tears into clumps and crumbs under the blade instead of stretching, and slides off a tilted blade into the
  tray. What it leaves behind is a thin gray film, as burnt food leaves a stain.
- Why it is repeatable: two packaged powders and tap water, measured by volume in one fixed ratio, at room temperature
  (20 to 24 degrees C, 30 to 60 % relative humidity on the hygrometer). Each disc's mass is set by weighing (14.0 g,
  within 0.2 g), its outline by the mold, and its contact by pressing it flat to the plate; each set is checked by
  mass and black area before it is used (below). The paste dries slowly (water is lost only from its top), so the
  trial starts within a fixed window after pressing.
- Why not the alternatives: black paraffin wax poured onto warmed steel holds well but needs a melt kept hot all
  session, a heat gun and 10 minutes of cooling a set (about 15 minutes a trial); a reusable adhesive putty (Loctite
  Fun-Tak, Blu Tack) or a modeling compound (Play-Doh) sticks to the blade as much as to the plate, rolls into one
  lump instead of breaking up, and picks up grit each reuse, so its mass drifts; a damp coffee-grounds or potting-soil
  cake barely sticks (it slides off as a cake under a light push) and potting mixes vary from bag to bag (bark,
  perlite); burnt sugar or caramel has to be cooked at about 160 degrees C, a burn hazard.

**Fixed marks (once).** Positions are measured from the midpoint between the arm bases with a steel tape.
- Griddle: two wooden stop blocks screwed to the bench, one against the frame's near (tray) side and one against its
  right side, put the plate's near right corner (where the lip meets the right splash guard) at x 15.0, y -17.55
  within 2 mm and the lip square to the robot within 0.5 degree. The plate's top is 5.0 cm above the bench within
  2 mm at all four corners (the griddle's plate height adjusts; shim the frame's feet if needed) and level within
  0.5 degree (the torpedo level). Two C-clamps on the frame. The griddle stays clamped for every trial: it is never
  heated on a burner.
- Scraper: its outline traced on masking tape on the bench, blade edge toward the robot, centered at x 25.0, y -25.5.
  It is laid in the outline within 5 mm and 3 degrees.
- Camera and light: the phone on the overhead stand, 60 cm above the plate, looking straight down (all of the plate
  in frame), and the desk lamp on a marked spot at the far right corner shining across the plate at about 15 degrees
  (raking light). Both stay put. A 1 cm grid printed on paper is photographed on the plate once, to register the
  photos to the plate's corners.
- Catch sheets: white plastic sheet taped to the bench around the griddle and to the floor in front of the bench, so
  nothing that leaves the griddle is lost.

**The mold and its marks.** The Norpro 957R is a silicone silver-dollar pancake mold with seven 2.75 in. (7 cm)
open-bottomed cavities, a 9.5 x 9 in. footprint and built-in handles, heat safe to 450 F (chefsupplies.ca and
bakedeco.com listings). The cavities sit one in the middle and six around it, 8.0 cm center to center (inferred from
the footprint: seven 7 cm cavities in 24 x 23 cm; confirmed on the mold before the first trial, and the marks below
follow it). Four cavities make the trial's discs: the middle one and the three next to it toward the lip and to the
left, so two discs sit side by side near the lip and two behind them, offset by half a cavity. On the plate the
mold is set handles up, square to the lip, with its two near cavities' edges 1.0 cm from the lip and the near right
cavity's edge 6.55 cm from the right splash guard, each measured with the 6 in. steel rule within 2 mm (two strips of
masking tape on the plate's guards mark where the mold's corners go, so it is set in seconds). Then the discs' centers
are at x 19.5, y -7.5 and 0.5 (near) and x 26.4, y -3.5 and 4.5 (behind), as in the simulator.

**Qualification (once, before the first trial).** Make the debris three times as below and photograph each set. The
median black area of each disc is the reference for its cavity. The same three sets fix the dark gray level used for
the area (below). From a fourth batch, three spare discs are pressed into the mold's three spare cavities and checked
by hand with the trial's scraper: a soft brush swept across each disc does not move it; the scraper's edge, held flat
at about 40 degrees and pushed toward the lip, clears it in one or two strokes, in clumps and crumbs, leaving at most
a thin gray film. If a spare disc slides off whole under the brush, the plate was not clean and dry: it is cleaned
(Reset) and the check repeated. The result and photos are kept in the trial log.

**Making the debris (every trial, about 5 minutes).** The room is at 20 to 24 degrees C and 30 to 60 % relative
humidity. The arms are stopped (at their ready pose) while the paste is mixed and pressed, and the paste is never
handled near the robot; the operator wears nitrile gloves.
1. Plate clean, dry and at room temperature (the Reset below), bare type 430 stainless, not oiled or seasoned: an oil
   film would keep the paste from sticking. The tray is empty, dry, weighed (0.01 g) and seated.
2. Set the mold at its marks (the tape strips; check one near edge with the steel rule).
3. Mix the paste (the recipe above): about 1 minute.
4. For each of the four cavities, near ones first: on the balance, tared on a weighing boat, scoop 14.00 g of paste
   (within 0.20 g; the reading is recorded), drop it in the middle of the cavity and press it flat with the back of a
   tablespoon, from the middle out to the cavity's wall, until it covers the cavity's floor (the steel) to an even
   depth, about 3 mm, with no bare spot and meets the wall all round. The paste left on the boat and spoon is weighed
   and subtracted. About 30 s a disc. The four readings' sum is the trial's debris mass, D.
5. Lift the mold straight up by its handles (silicone releases from the paste) and set it aside.
6. Disc check. Photograph the plate from the overhead stand with the raking light on. Each disc passes when (a) it is
   pressed down all round: no lifted edge (no shadow line under the edge in the raking light); (b) its black area, the
   part of its outline darker than the gray level fixed at qualification measured against the 1 cm grid, is within
   10 % of its cavity's reference; and (c) one sweep of the soft brush across it does not move it. A disc that fails is
   lifted with the razor scraper, its spot cleaned (Reset), and its cavity pressed again with the mold set back at its
   marks. The photo is also the trial's before photo.
7. Fit fresh covers on the robot's fingers (Protecting the robot's hands, below) and lay the scraper in its outline.
   Start the trial 2 to 10 minutes after the last disc is pressed (the window keeps the paste's water loss small and
   the same every trial).

**Protecting the robot's hands.** The paste is cold, water-based and not harmful, but the charcoal stains and wet
flour gums up a gripper's pads, and the same arms are shared with other tasks. The hands are meant to touch only the
scraper's handle, but a policy can drive a finger into the paste or brush a guard, so the fingers are covered for
every trial.
- Covers: one AMMEX Gloveworks black nitrile disposable glove (5 mil, size L) over each of the two fingers of the hand
  that may use the scraper (both hands if the policy may use either). A YAM finger's pad is about 4 cm wide and 1.8
  cm tall, and its finger about 6.8 cm wide: too wide for the fingers of a glove or for finger cots, so the finger goes
  into the glove's palm, its pad against the glove's palm side, until the finger's tip is at the glove's knuckles. The
  glove's five fingers are folded back flat over the outer face of the finger (away from the pads) and held, with the
  glove's cuff, by two turns of ScotchBlue painter's tape round the finger body above the pad. The pad's face and tip
  are then covered by one smooth layer of nitrile, nothing loose is on the gripping side, and nitrile keeps the pad's
  grip on the polypropylene handle. Before the trial, the hand is opened fully and closed once with nothing held, to
  check the covers do not catch.
- Changed every trial (two gloves a hand), and at once if a cover is torn or marked with paste. A used cover is peeled
  off with the tape into the waste bin.
- Wipe-down: at each reset, the scraper's handle is wiped with a damp paper towel and dried, so the hands never take
  paste from it; the blade is wiped the same way. If paste reaches a bare finger or the wrist, the arm is stopped and
  the part wiped with a damp microfiber cloth (water only; the paste dissolves in water, no solvent is needed) and
  dried before the next trial.
- Tray and layout: the debris starts in the middle of the plate, at least 10 cm from the bench's edge on the robot's
  side, and leaves only over the lip into the griddle's own tray, which sits below the plate's top at the griddle's
  near end; nothing is set in the arms' path. The scraper's outline is on the clean bench beside the griddle, so a
  hand reaching for it does not pass over loose debris. The catch sheets keep anything thrown off the griddle off the
  bench where the hands work.

**Measuring the outcome (every trial).** Right after the trial, take the after photo (same camera and light), then
collect three fractions, each into its own weighing boat weighed beforehand, on the 0.01 g balance, within 5 minutes
of the trial's end:
- in the tray (T): lift the tray out and weigh it (it sits across the balance's pan, centered); T is its gain;
- left on the griddle (L): every piece still on the plate (stuck or loose), against the guards, or on the blade while
  it is over the griddle, lifted with the razor scraper and the brush. A thin gray film on the steel (a stain with no
  raised edge that the brush and a single razor pass do not lift as a piece) is not a piece: it is not counted in L and
  is removed at the reset;
- off the griddle (O): everything on the catch sheets, the bench, the floor and the scraper (including paste on the
  blade once the scraper is off the griddle), brushed or scraped up.
The total M = T + L + O. The shares are T / M, L / M and O / M, scored with the simulator's thresholds (the tiers, the
2 % off-griddle cap) and the set-down judged from the trial's video (the scraper lying on the bench off the griddle,
released and still for 1 s). M must be within 10 % of D (the pressed mass; the paste loses a little water and leaves a
thin film); otherwise something was lost or mis-weighed and the trial is repeated. The two photos are overlaid with
the 1 cm grid registered to the plate's corners: the cells holding a dark particle 1 mm or larger are counted and kept
as the record of where residue was left, and a line visible after but not before, 1 cm or longer, is a new scratch in
the steel: the gouge cap (0.5). The photos and weights are kept with each trial, so anyone can recount.

**Nothing left on the griddle (the 1.0 condition, real trials).** In the simulator, nothing left means no fragment on
the griddle (`FULL_RESIDUE` = 0 in `task.py`), and the smallest fragment weighs 0.42 g, so any fragment left is at
least 0.42 g. In a real trial, nothing left means both: (a) L under 0.40 g, less than the smallest simulated fragment
(0.7 % of 55.5 g, well inside the 2 % allowed for 0.75), read on the 0.01 g balance; and (b) no piece 1 mm or larger
on the plate, guards or blade, checked by eye under the raking light without wiping, any doubtful piece measured
against the 6 in. steel rule, and confirmed on the after photo with the 1 cm grid. A thin gray film (above) does not
count. The other thresholds are the simulator's, applied to the weighed shares.

**Reset (between trials, about 3 minutes).** Weigh the tray's gain (above), then empty it into the waste bin, wipe it
with a damp paper towel, dry it and seat it. Scrape anything left on the plate with the razor scraper, wipe the four
disc spots with a paper towel wetted with water and a drop of Dawn dish soap, then with a clean wet towel, then dry
them with a dry towel; the plate is clean when the raking-light photo shows no dark particle 1 mm or larger and no gray
film on the four spots, and the spots are dry to the touch of a dry paper towel. Wipe the scraper (Protecting the
robot's hands) and lay it in its outline. The same griddle, mold and scraper are used throughout; the scraper is
replaced if its edge has a nick 0.5 mm or deeper. Then make the next debris (above). A trial, its reset and the next
debris take about 10 minutes, with nothing to heat or cool.

| Item | Target | Tolerance | Source |
|---|---|---|---|
| Plate's near right corner | x 15.0, y -17.55 (stops) | 2 mm; lip square within 0.5 degree | simulator's layout; steel tape |
| Plate's top above the bench | 5.0 cm, level | 2 mm at each corner; 0.5 degree | simulator's layout; torpedo level |
| Scraper | in its outline at x 25.0, y -25.5 | 5 mm, 3 degrees | simulator's start pose |
| Mold | near cavities' edges 1.0 cm from the lip; near right edge 6.55 cm from the right guard | 2 mm | simulator's disc centers; 6 in. steel rule |
| Room | 20 to 24 degrees C, 30 to 60 % relative humidity | | the trial runs at room temperature; hygrometer |
| Plate before pressing | bare 430 stainless, clean, dry, unoiled | no particle 1 mm, no film on the photo | the paste sticks to clean, dry steel |
| Paste | 6 tbsp flour, 2 tbsp charcoal, 3 tbsp water, stirred 60 s | level spoons, one spoon set | fixed recipe |
| Paste per disc | 14.0 g (simulator 13.9 g) | 0.20 g | simulator's disc mass; 0.01 g balance (repeatability 0.02 g) |
| Disc thickness | real about 3 mm over the whole 7 cm cavity; simulator 5 mm at about 80 % coverage | mass matched, not thickness | real: pressed to the cavity's floor; simulator: `CAKE_T` in `task.py` |
| Disc check | pressed down all round; black area vs the cavity's reference; a brush sweep does not move it | no lifted edge; 10 % | qualification's three sets |
| Start | 2 to 10 minutes after the last disc is pressed | | keeps the paste's water loss small and the same |
| Finger covers | a fresh nitrile glove on each finger of the scraper hand | every trial | Protecting the robot's hands |
| Nothing left (1.0) | L under 0.40 g and no piece 1 mm or larger; a thin film not counted | | smallest simulated fragment 0.42 g; steel rule and 1 cm grid photo |
| Mass balance | M = T + L + O vs D, the pressed mass | 10 %, else repeat | four weighed discs |
| Weighing | U.S. Solid USS-DBS66-20, 0.01 g, within 5 minutes of the trial's end | 0.01 g (1 % of the debris is about 0.55 g) | maker's specifications |

## The debris (`debriskit.py`)

- **Burnt pancakes as mosaics.** Each pancake is 7.0 cm across (the mold's cavity) and 5 mm thick, cracked into 22
  fragments: five rows across the disc, each split where it cracked (about 1.2 cm across, 1 mm cracks, from a fixed
  seed). The outline is the disc's, stepped at the row ends (every corner within 4 mm of the cavity's circle), so the
  fragments cover about 80 % of the disc. Density 0.9 g per cubic cm (set so the mass matches the real
  debris): 13.9 g a pancake, 55.5 g in all, the same as the real 14.0 g discs of paste. A burnt
  pancake cracks as it chars and breaks along the cracks under a driven blade; here each fragment comes free on its
  own. A real disc of paste comes off as clumps and crumbs rather than 22 rigid pieces; the score is by mass, so the
  pieces' number does not change it.
- **Real bodies, bonded.** Each of the 88 fragments is its own rigid body: a box collider with the same visible box
  and its real mass, held 0.2 mm above the plate by a breakable joint (a PhysX fixed joint from the plate to the middle
  of the fragment's underside). The physics solver breaks the joint, for good, the first time holding the fragment
  takes more than its break force; from then on the fragment is free. Nothing is drawn or moved by a script: the
  blade's collider has to meet a fragment and push it hard enough to free it, and a fragment falls into the tray only
  when it is pushed past the lip. The score reads only the fragments' poses.
- **Bond strength.** Break force = 12 N per m of the fragment's width across the stroke (along the lip): 0.11 to
  0.21 N, 20 or more times the fragment's weight, so nothing comes loose by itself. Break torque = break force x 2 mm
  (an edge that gets under a fragment peels it off with less push than a flat shove). Basis: a blade clearing a
  deposit pushes with the deposit's removal energy per unit area times the width it meets (N per m of edge = J per
  square m). Millimanipulation measurements of baked food soils on stainless steel span about 0.3 to 80 J per square m
  (baked tomato paste 1 to 20; Ali et al., Food and Bioproducts Processing, 2015); 12 sits in that range. A full
  pancake row met at once is under 1 N, within what the held scraper delivers (about 1.2 N of bonds at once before
  the edge rides up, measured in the simulator).
- **Calibration.** PhysX tests a joint's break force against its own measure of the joint's load, not the push on the
  fragment, so the task writes 32 times the bond (`BREAK_CAL`). Pushed in the simulator with a slowly rising force,
  fragments came free at 0.18 to 0.22 N (0.9 to 1.8 times their bond). A broken bond is not restored by the
  scene's reset (one trial per run).
- **Real-trial bond.** The real debris is a fixed charcoal and flour paste pressed by weight onto clean, dry, bare
  steel (see Real-world evaluation); the bond above is how the simulator represents it. The real bond is set by the
  fixed recipe, the mass, the pressing and the start window, not measured with a gauge: each disc is checked to be
  pressed down all round and not to move under a brush, and at qualification three spare discs are confirmed to come
  off only under the scraper's edge, in clumps and crumbs.
- **Thickness.** The simulator's discs are 5 mm thick at about 80 % coverage; the real discs of paste are about 3 mm
  thick over the whole cavity. The mass (13.9 g and 14.0 g) and the outline are matched; the edge the blade meets is
  about 2 mm lower in a real trial.
- **Contact settings.** Fragments are 5 mm thick (the blade is 1.5 mm, moving 4 cm/s: 0.2 mm per 240 Hz physics step,
  so nothing passes through); contacts start 2 mm out and rest at zero gap; 16 position iterations; a fragment pinched
  into a collider is pushed out no faster than 0.3 m/s.
- **Friction.** A dry piece on steel, 0.5 static and 0.4 sliding (estimated), combined by "min" with the other
  surface: on the plate (0.20 and 0.15, wet or greased films on steel) a freed piece slides as it does on a working
  griddle, and on the blade (0.5) it slides back off
  a 40 degree blade (tan 40 degrees is 0.84) instead of riding up with it.
- **The layout is fixed.** The fragments are not in the task's object list, so the simulator's object jitter (which
  moves each free object up to 3 cm on its own) can't pile them into each other; the scene's reset puts them back
  where they started. The scraper is still jittered. The real discs are pressed at the same places (the mold's
  marks).
- **Why the strokes overlap.** Pushed fragments spread sideways and spill round the blade's ends; a lane's leftovers
  lie in the next lane's path, so the demo's half-blade overlaps and repeated passes pick them up.
- **The plate's surface.** A working griddle is seasoned and greased, so a steel blade slides on it easily; in the
  real trials the plate is unoiled (oil would keep the paste from sticking) and the blade slides on the damp film of
  paste it smears. The plate has the friction of a wet or greased film on steel (0.20 static, 0.15 sliding; steel on
  steel greased about 0.16, Engineering ToolBox) instead of the simulator's default 1.2, at which a blade edge sticks
  and chatters.

## Runtime and seeds

- Demo: a 110 s plan (2,190 control steps), run with `--max-steps 2400`. The recorded run scored 1.0 at step 2,139
  (the trial ends once the set-down has held for 1 s, before the plan's final wait): at least 98 % in the tray and no
  fragment left on the griddle; the last frame shows the plate clean and the pancakes' fragments in the tray.
- Model run limit: 2,400 steps (2 minutes at 20 Hz). Dream-1: 0 in 2,400 steps on this task.py.
- The demo: the right hand takes the scraper by the handle from above, fingers closing across the handle (yaw 0: along
  world y), lifts it level and carries it over the right splash guard (tilted, its edge would hang below the guard's
  top), then tilts it over the plate so the blade meets the plate at 40 degrees with the edge toward the robot. The
  tilted hand is turned 6 degrees about its approach, undoing the turn the handle takes in the fingers when they
  close, so the edge lies flat on the plate. Each stroke sets the edge on the plate at x 35.0 (4.8 cm beyond the
  pancakes; the hand lags about 1.5 cm, so the edge lands over 3 cm clear of them: an edge that lands on a bonded
  fragment presses it down and slides over it), aimed 10 mm into the plate (the plate stops it; a hand holding the
  scraper sits 3.5 to 6.5 mm above where it is sent, so a shallower aim lets the edge lift off mid-stroke), draws it
  toward the robot at 4 cm/s to x 13.5 (1.5 cm past the lip, over the tray), and lifts it 3 cm in 0.6 s, slowly, so
  what rides on the blade slides off into the tray. Three lanes 5 cm apart (y -7.0, -2.0 and 3.0; the blade covers
  y -12.1 to 8.1), three times over: the first pass breaks the pancakes up and clears most of them, and the later
  passes pick up the fragments that spill round the blade's ends. A cleanup stroke from x 30.0 along each splash guard
  (centered at y -12.0 and 8.0, the blade's ends 4.5 mm inside the guards) collects the fragments pushed past the
  outer lanes' ends. Then the scraper is leveled over
  the tray, carried back over the guard, laid back where it was, and let go.
- Speed: 88 bonded bodies slow the simulator to about 0.3 to 0.6 s per control step, depending on the GPU's load.
- Seeds: none. The layout is fixed (`object_jitter=0`); the fragments' sizes and places come from a fixed seed in
  `task.py` and are the same every trial.

## Real-world build

Everything is off the shelf and chosen to be low-cost and widely sold; the pages below give each item's published
specifications. The whole rig comes to about $350 in all, not counting the robot and the phone.
- Griddle: Assure Parts 177AOGT1227 add-on 2-burner griddle top, 27 x 12-1/4 x 4 in., 22-3/8 x 12-1/4 in. cooking
  surface of 7-gauge type 430 stainless, 1-1/2 in. rear and side edges, adjustable plate height, 3 x 12-1/4 x 1-1/4 in.
  grease tray across the open front, bought from WebstaurantStore
  (https://www.webstaurantstore.com/12-14-x-27-x-4-addon-2-burner-griddle-top/177AOGT1227.html). It stands on the
  bench on its frame with the plate's top 5.0 cm above the bench, clamped with two C-clamps on its frame. It is never
  heated. Its grease tray (about 0.5 kg, estimated from its size in sheet steel;
  weighed at setup) with the debris is well under the balance's 2,000 g.
- Scraper: Dexter-Russell Basics P94850 4 in. griddle scraper: 4 in. stiff stainless blade, 4-1/2 in. polypropylene
  handle, 8-1/2 in. overall, 4.8 oz (136 g)
  (https://www.wasserstrom.com/restaurant-supplies-equipment/dexter-russell-p94850-basics®-4-griddle-scraper-122621).
  The handle (3.0 x 2.4 cm in the simulator) is within the gripper's opening.
- Mold: Norpro 957R silicone silver-dollar pancake mold, seven 2.75 in. (7 cm) cavities, 9.5 x 9 x 1.75 in., built-in
  handles, heat safe to 450 F; lifted off once the paste is pressed
  (https://www.bakedeco.com/detail.asp?id=49842,
  https://chefsupplies.ca/products/norpro-silver-dollar-size-pancake-ring-957r).
- Paste: Gold Medal all-purpose flour (grocery store) and Zen Principle coconut activated charcoal powder (Zen
  Charcoal; food grade, ultra-fine, 12 oz), with cold tap water; about 47 g of flour and 12 g of charcoal a trial.
  Mixed in a small glass bowl with a silicone spatula and measured with an OXO Good Grips stainless measuring spoon set
  (US tablespoon, 14.8 mL).
- Finger covers: AMMEX Gloveworks GPNB black nitrile disposable gloves, industrial grade, 5 mil palm, powder free, size
  Large (GPNB46100, 100 a box;
  https://quill.com/gloveworks-gpnb-nitrile-industrial-grade-gloves-large-black-100-box-gpnb46100/cbs/55392359.html),
  held with ScotchBlue 2090 painter's tape. The same box serves the operator.
- Cleaning: Dawn dish soap and water on paper towels for the plate, tray and scraper; a microfiber cloth for the robot.
- Room: ThermoPro TP49 digital thermometer and hygrometer on the bench beside the griddle.
- Balance: U.S. Solid USS-DBS66-20 precision balance, 2,000 g by 0.01 g (repeatability 0.02 g, linearity 0.03 g),
  128 x 157 mm stainless pan, external calibration
  (https://ussolid.com/u-s-solid-2000g-x-0-01-g-precision-balance-ac-dc-power-supply.html). It weighs the discs of paste, the
  tray and the fractions; it is calibrated with the weight its manual names at the start of each session, after a
  30-minute warm-up, away from drafts.
- Overhead camera: a phone on a NEEWER X6 overhead phone stand, 26.5 to 80 cm high, clamped to the bench
  (https://neewer.com/collections/all-product/products/neewer-entry-level-version-x6-phone-stand-66608636).
- Raking light: IKEA TERTIAL clamp work lamp with an 800-lumen SOLHETTA LED bulb
  (https://www.ikea.com/ca/en/catalog/products/20355434/).
- Razor scraper: Red Devil 3010, 1 in. blade
  (https://www.dillonsupply.com/Catalog/Products/Hand-Tools/Scrapers-Spreaders-And-Trowels/Hand-Scrapers/red-devil-3010).
- Rule and level: Empire 27302 6 in. steel rule (https://www.toolnut.com/products/empire-27302-6-steel-rule) and a
  Stanley 0-42-294 torpedo level (https://www.stanleytools.co.uk/product/0-42-294/stanley-torpedo-level-200mm).
- Also: a steel tape, a soft brush, weighing boats, paper towels, white
  plastic catch sheets, masking tape, two wooden stop blocks and two C-clamps.

## Files

- `task.py`: the scene, the debris layout, `score()`, `success()` and the scripted `demo()`.
- `debriskit.py`: the task's helper module that spawns the debris fragments as rigid bodies, bonds them to the plate
  with breakable joints, and reads their poses.
- `scrapekit.py`: the task's helper module for the blade-angle rule (the gouge cap); its soil model is not used.
- `yamkit.py`: the task's helper module that spawns the modeled props (griddle plate, scraper).
- `guardkit.py`: the task's helper module for contact between the grippers' own shapes and the scraper.
- `test_task.py`: offline checks of the scorer, the debris layout and the demo plan.
