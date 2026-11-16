# Scrape burnt pancakes off a griddle over the lip into its grease tray with a scraper

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). A range-top add-on griddle
stands on the bench with its grease tray toward the robot. Near the lip, four silver-dollar pancakes were left on the
heat and burnt onto the steel. The robot takes a 4 in. griddle scraper from the bench, breaks the pancakes up and
pushes them over the lip into the tray, and sets the scraper down. The setup, scoring and fidelity notes are in
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
the end of the trial. Every share is by mass, of the debris (30.8 g in the simulator), read from where the 88
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
  scores 0.75. Debris pushed loose but left on the plate is not clean: it is the same char in a pile.
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
  in it: only its blade-angle rule is used). The simulation has no seasoning or scratches; this stands in for the
  chatter marks and scored lines a steep or cornered blade leaves in a griddle's seasoning. In a real trial the cap
  is judged on the plate itself: a new scratch 1 cm or longer, against a photo taken before the trial under a
  raking light.
- Doing nothing scores 0. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

## Real-world evaluation (reproducible protocol)

Every real trial starts from the same measured state and is scored on the same shares as the simulator, read by
weighing. The protocol fixes four things: the debris (one named mix, weighed into one named mold, one set and burn,
checked per batch by mass and black area), the positions (stops and marks on the bench and the plate), the outcome
measurement (three weighed fractions plus photos) and the reset. The tolerances are collected in the table at the end;
the equipment, with its makers' or retailers' pages, is under Real-world build.

**The scene.** At breakfast a cook pours silver-dollar pancakes in a pancake mold on the griddle; four are left on
the heat while an order goes out and burn onto the plate. A cook clears them before the next order by setting the
scraper's edge flat on the plate behind the burnt food and pushing it toward the grease tray in overlapping strokes,
then over the lip (WebstaurantStore's griddle-cleaning guide: scrape debris into the griddle's waste drawer,
https://www.webstaurantstore.com/article/142/how-to-clean-a-griddle.html). Silicone and steel pancake and egg rings are
sold for flat-tops for this use: set on the hot, oiled plate, batter poured in. The trial runs cold: the robot and the
operator work next to a plate at room temperature, and every trial starts at the same temperature. Cooks scrape the
plate warm; the burnt food is the same.

**The method.** It follows how cleaning products are tested on baked-on soil: a soil made to a fixed recipe by weight,
applied at a fixed amount, baked on a fixed schedule, and its removal measured by weighing (the US federal
specification A-A-7 for oven and grill cleaners, and CSPA DCC-12 for oven cleaners, bake a standard soil on coupons
and score removal from the coupons' weights). Here the soil is the burnt food a griddle collects, pancakes, made from
one named, widely sold mix, weighed into one named mold, set and burnt on a fixed schedule.

**Fixed marks (once).** Positions are measured from the midpoint between the arm bases with a steel tape.
- Griddle: two wooden stop blocks screwed to the bench, one against the frame's near (tray) side and one against its
  right side, put the plate's near right corner (where the lip meets the right splash guard) at x 15.0, y -17.55
  within 2 mm and the lip square to the robot within 0.5 degree. The plate's top is 5.0 cm above the bench within
  2 mm at all four corners (the griddle's plate height adjusts; shim the frame's feet if needed) and level within
  0.5 degree (the torpedo level). Two C-clamps on the frame.
- Scraper: its outline traced on masking tape on the bench, blade edge toward the robot, centered at x 25.0, y -25.5.
  It is laid in the outline within 5 mm and 3 degrees.
- Camera and light: the phone on the overhead stand, 60 cm above the plate, looking straight down (all of the plate
  in frame), and the desk lamp on a marked spot at the far right corner shining across the plate at about 15 degrees
  (raking light). Both stay put. A 1 cm grid printed on paper is photographed on the plate once, to register the
  photos to the plate's corners.
- Catch sheets: white plastic sheet taped to the bench around the griddle and to the floor in front of the bench, so
  nothing that leaves the griddle is lost.

**The mold and its marks.** The Norpro 957R is a silicone silver-dollar pancake mold with seven 2.75 in. (7 cm)
cavities, a 9.5 x 9 in. footprint and built-in handles, heat safe to 450 F (chefsupplies.ca and bakedeco.com listings).
The cavities sit one in the middle and six around it, 8.0 cm center to center (inferred from the footprint: seven
7 cm cavities in 24 x 23 cm; confirmed on the mold before the first trial, and the marks below follow it). Four cavities make the trial's
pancakes: the middle one and the three next to it toward the lip and to the left, so two pancakes sit side by side near
the lip and two behind them, offset by half a cavity. A fifth, the cavity behind the middle one on the left, makes the
witness pancake for the batch check. On the cold plate the mold is set handles up, square to the lip, with its two
near cavities' edges 1.0 cm from the lip and the near right cavity's edge 6.55 cm from the right splash guard, each
measured with the 6 in. steel rule within 2 mm. Then the pancakes' centers are at x 19.5, y -7.5 and 0.5 (near) and
x 26.4, y -3.5 and 4.5 (behind), as in the simulator.

**Qualification (once, before the first trial).** Cook three batches as below. For each, record the witness
pancake's mass and every pancake's black area (below). Their medians are the reference for every later batch.

**Making the debris (every trial).**
1. Plate clean, dry and cold (under 40 degrees C), the tray empty, dry, weighed (0.01 g) and seated.
2. Grease: 1.00 g of vegetable oil wiped over the plate with a paper towel, then buffed off with two dry paper towels
   weighed beforehand until they pick up no more. The film left (oil put on less the towels' gain) is at most 0.10 g:
   the plate is seasoned and lightly greased, as on a working griddle. Wipe the mold's cavities with the same towel.
3. Set the mold at its marks (above). Move the griddle to a two-burner range under a vented hood, away from the
   robot's bench, with the Class K extinguisher at hand. Heat until the surface thermometer reads 190 degrees C
   (375 degrees F, the mix box's griddle temperature) within 10 at six points marked on the plate (the four corners,
   the middle of the mold, and beside it).
4. Batter: Krusteaz Light & Fluffy Buttermilk Complete Pancake Mix, mixed by the box's directions: 1 cup of mix
   (spooned into the cup, leveled and weighed; each batch within 5 % of the qualification's weight) whisked with
   two-thirds of a cup (158 g) of cold water until just combined, rested 2 minutes, used within 30 minutes. The box's
   lot number is recorded with each trial.
5. Pour: 15.0 g of batter (within 0.3 g, about the tablespoon the mold's instructions give per cavity), weighed into a
   cup on the balance, into each of the five cavities. Set for 2 minutes (within 15 s), until the edges look dry and
   the tops stop bubbling. Lift the mold straight up by its handles (oven mitts on) and set it aside.
6. Burn: raise the heat until all six points read 232 degrees C (450 degrees F) within 10, hold for 20 minutes
   (within 15 s), then turn the burners off and let the griddle cool on the range until all six points read under 40
   degrees C. The pancakes are not turned: they burn onto the plate from below, as a pancake left on the heat does.
   Nothing hot goes to the robot's bench. One batch, with cooling, takes about 2 hours; a lab running many trials
   uses two identical griddles in rotation.
7. Batch check (mass and black area). Photograph the plate from the overhead stand. For each pancake, its black area
   is the part of its outline darker than the gray level fixed at qualification, measured against the 1 cm grid.
   Then lift the witness pancake with the razor scraper and brush (cut-resistant gloves on) into a weighing boat and
   weigh it (0.01 g); wipe its spot clean. The batch is used when the witness's mass is within 15 % and every
   pancake's black area within 20 % of the reference medians; otherwise the plate is cleaned (Reset) and a fresh
   batch cooked.
8. Set the griddle back against its stops and clamp it; lay the scraper in its outline; take the before photo (top
   camera, raking light on). Start the trial within 2 hours of the cook.

The same mix, mass of batter, mold, set and burn give the same pancakes from batch to batch, and the witness and
black areas confirm each batch before it is used. Burnt pancakes are lighter than the batter (their water cooks off):
about 7 g each, estimated; the witness weighs it every batch.

**Measuring the outcome (every trial).** After the trial, take the after photo (same camera and light), then collect
three fractions, each into its own weighing boat weighed beforehand, on the 0.01 g balance:
- in the tray (T): lift the tray out and weigh it; T is its gain;
- left on the griddle (L): everything still on the plate (stuck or loose), against the guards, or on the blade while
  it is over the griddle, lifted with the razor scraper and the brush;
- off the griddle (O): everything on the catch sheets, the bench, the floor and the scraper, brushed up.
The total M = T + L + O. The shares are T / M, L / M and O / M, scored with the simulator's thresholds (the tiers, the
2 % off-griddle cap) and the set-down judged from the trial's video (the scraper lying on the bench off the griddle,
released and still for 1 s). M must be within 10 % of four times the batch's witness pancake; otherwise something was
lost or mis-weighed and the trial is repeated. The two photos are overlaid with the 1 cm grid registered to the plate's
corners: the cells holding a dark particle 1 mm or larger are counted and kept as the record of where residue was
left, and a line visible after but not before, 1 cm or longer, is a new scratch: the gouge cap (0.5). The photos and
weights are kept with each trial, so anyone can recount.

**Reset (between trials).** Empty the tray, wash, dry and weigh it. Scrape anything left on the plate with the razor
scraper, wipe it with a damp cloth and dry it; the plate is clean when the grid photo shows no dark particle 1 mm or
larger. Every 10 trials, or when bare or rusty steel shows, re-season the plate (a thin coat of vegetable oil, 15
minutes at 232 degrees C, cooled) and take a new reference photo. The same griddle, mold and scraper are used
throughout; the scraper is replaced if its edge has a nick 0.5 mm or deeper. Then make a fresh batch (above).

| Item | Target | Tolerance |
|---|---|---|
| Plate's near right corner | x 15.0, y -17.55 (stops) | 2 mm; lip square within 0.5 degree |
| Plate's top above the bench | 5.0 cm, level | 2 mm at each corner; 0.5 degree |
| Scraper | in its outline at x 25.0, y -25.5 | 5 mm, 3 degrees |
| Mold | near cavities' edges 1.0 cm from the lip; near right edge 6.55 cm from the right guard | 2 mm |
| Oil film after buffing | at most 0.10 g | towels weighed (0.01 g) |
| Mix | 1 cup, the qualification's weight | 5 % |
| Batter per cavity | 15.0 g | 0.3 g |
| Set | 190 degrees C at six points, 2 minutes | 10 degrees C; 15 s |
| Burn | 232 degrees C at six points, 20 minutes | 10 degrees C; 15 s |
| Cool | under 40 degrees C at six points | trial within 2 h |
| Batch check | witness mass; each pancake's black area | 15 %; 20 % of the reference medians |
| Mass balance | M = T + L + O vs four witnesses | 10 %, else repeat |
| Weighing | 0.01 g balance | 0.01 g (1 % is about 0.3 g) |

## The debris (`debriskit.py`)

- **Burnt pancakes as mosaics.** Each pancake is 7.0 cm across (the mold's cavity) and 5 mm thick, cracked into 22
  fragments: five rows across the disc, each split where it cracked (about 1.2 cm across, 1 mm cracks, from a fixed
  seed). The outline is the disc's, stepped at the row ends (every corner within 4 mm of the cavity's circle). Density
  0.5 g per cubic cm (porous char): 7.7 g a pancake, 30.8 g in all. A burnt pancake cracks as it chars and breaks up
  along its cracks under a scraper; here each fragment comes free on its own.
- **Real bodies, bonded.** Each of the 88 fragments is its own rigid body: a box collider with the same visible box
  and its real mass, held 0.2 mm above the plate by a breakable joint (a PhysX fixed joint from the plate to the middle
  of the fragment's underside). The physics solver breaks the joint, for good, the first time holding the fragment
  takes more than its break force; from then on the fragment is free. Nothing is drawn or moved by a script: the
  blade's collider has to meet a fragment and push it hard enough to free it, and a fragment falls into the tray only
  when it is pushed past the lip. The score reads only the fragments' poses.
- **Bond strength.** Break force = 12 N per m of the fragment's width across the stroke (along the lip): 0.11 to
  0.21 N, 25 or more times the fragment's weight, so nothing comes loose by itself. Break torque = break force x 2 mm
  (an edge that gets under a fragment peels it off with less push than a flat shove). Basis: a blade clearing a
  deposit pushes with the deposit's removal energy per unit area times the width it meets (N per m of edge = J per
  square m). Millimanipulation measurements of baked food soils on stainless steel span about 0.3 to 80 J per square m
  (baked tomato paste 1 to 20; Ali et al., Food and Bioproducts Processing, 2015); 12 sits in that range. A full
  pancake row met at once is under 1 N, within what the held scraper delivers (about 1.2 N of bonds at once before
  the edge rides up, measured in the simulator on an earlier layout).
- **Calibration.** PhysX tests a joint's break force against its own measure of the joint's load, not the push on the
  fragment, so the task writes 32 times the bond (`BREAK_CAL`). Pushed in the simulator with a slowly rising force,
  fragments came free at 0.18 to 0.22 N (0.9 to 1.8 times their bond). A broken bond is not restored by the
  scene's reset (one trial per run).
- **Real-trial bond.** The real debris is burnt pancakes (see Real-world evaluation); the bond above is how the
  simulator represents them. The real bond comes from the fixed mix, batter mass, set and burn, and each batch is
  checked by the witness's mass and the pancakes' black areas before it is used.
- **Contact settings.** Fragments are 5 mm thick (the blade is 1.5 mm, moving 4 cm/s: 0.2 mm per 240 Hz physics step,
  so nothing passes through); contacts start 2 mm out and rest at zero gap; 16 position iterations; a fragment pinched
  into a collider is pushed out no faster than 0.3 m/s.
- **Friction.** Char on steel, 0.5 static and 0.4 sliding, combined by "min" with the other surface: on the greased
  plate (0.20 and 0.15) a fragment slides as easily as on a working griddle, and on the blade (0.5) it slides back off
  a 40 degree blade (tan 40 degrees is 0.84) instead of riding up with it.
- **The layout is fixed.** The fragments are not in the task's object list, so the simulator's object jitter (which
  moves each free object up to 3 cm on its own) can't pile them into each other; the scene's reset puts them back
  where they started. The scraper is still jittered. The real pancakes are cooked at the same places (the mold's
  marks).
- **Why the strokes overlap.** Pushed fragments spread sideways and spill round the blade's ends; a lane's leftovers
  lie in the next lane's path, so the demo's half-blade overlaps and repeated passes pick them up.
- **The plate's surface.** A working griddle is seasoned and greased, so a steel blade slides on it easily. The plate
  has the friction of greased steel on steel (0.20 static, 0.15 sliding) instead of the simulator's default 1.2, at
  which a blade edge sticks and chatters.

## Runtime and seeds

- Demo: a 110 s plan (2,190 control steps), run with `--max-steps 2400`. The recorded run scored 1.0 at step 2,139
  (the trial ends once the set-down has held for 1 s, before the plan's final wait): at least 98 % in the tray and no
  fragment left on the griddle; the last frame shows the plate clean and the pancakes' fragments in the tray.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
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

Everything is off the shelf; the pages below give each item's published specifications.
- Griddle: Assure Parts 177AOGT1227 add-on 2-burner griddle top, 27 x 12-1/4 x 4 in., 22-3/8 x 12-1/4 in. cooking
  surface of 7-gauge type 430 stainless, 1-1/2 in. rear and side edges, adjustable plate height, 3 x 12-1/4 x 1-1/4 in.
  grease tray across the open front
  (https://www.webstaurantstore.com/12-14-x-27-x-4-addon-2-burner-griddle-top/177AOGT1227.html). It stands on the
  bench on its frame with the plate's top 5.0 cm above the bench, clamped with two C-clamps on its frame. It is heated
  only to make the debris, on a two-burner range under a vented hood away from the bench, and is under 40 degrees C
  and off the range when it is clamped down.
- Scraper: Dexter-Russell Basics P94850 4 in. griddle scraper: 4 in. stiff stainless blade, 4-1/2 in. polypropylene
  handle, 8-1/2 in. overall, 4.8 oz (136 g)
  (https://www.wasserstrom.com/restaurant-supplies-equipment/dexter-russell-p94850-basics®-4-griddle-scraper-122621).
  The handle (3.0 x 2.4 cm in the simulator) is within the gripper's opening.
- Mold: Norpro 957R silicone silver-dollar pancake mold, seven 2.75 in. (7 cm) cavities, 9.5 x 9 x 1.75 in., built-in
  handles, heat safe to 450 F; lifted off before the burn (https://www.bakedeco.com/detail.asp?id=49842,
  https://chefsupplies.ca/products/norpro-silver-dollar-size-pancake-ring-957r).
- Mix: Krusteaz Light & Fluffy Buttermilk Complete Pancake Mix (box directions: 1 cup of mix to two-thirds of a cup of water,
  stand 2 minutes, griddle at 375 F; https://www.krusteaz.com/products/pancakes-waffles/buttermilk-pancake/).
- Balance: Ohaus Scout STX2202, 2,200 g by 0.01 g, 140 x 170 mm pan
  (https://www.transcat.com/ohaus-scale-30253011-stx2202-292080).
- Surface thermometer: ThermoWorks Pro-Surface Thermapen (THS-231-279), type K surface probe, -50 to 300 degrees C
  (https://www.thermoworks.com/products/ths-231-279). A contact probe suits bare stainless, which an infrared
  thermometer reads low (ThermoWorks, https://blog.thermoworks.com/emissivity/).
- Overhead camera: a phone on a NEEWER X6 overhead phone stand, 26.5 to 80 cm high, clamped to the bench
  (https://neewer.com/collections/all-product/products/neewer-entry-level-version-x6-phone-stand-66608636).
- Raking light: BenQ e-Reading LED desk lamp
  (https://www.benq.eu/en-uk/lighting/e-reading-desk-lamp/e-reading/spec.html).
- Razor scraper: Red Devil 3010, 1 in. blade
  (https://www.dillonsupply.com/Catalog/Products/Hand-Tools/Scrapers-Spreaders-And-Trowels/Hand-Scrapers/red-devil-3010).
- Fire extinguisher: Amerex B260, 6 L wet chemical, rated 2A:K, for cooking-oil fires
  (https://tigermedical.com/products/6-liter-stored-pressure-wet-chemical-fire-extinguisher-class-k-amrb260-oo).
- Rule and level: Empire 27302 6 in. steel rule (https://www.toolnut.com/products/empire-27302-6-steel-rule) and a
  Stanley 0-42-294 torpedo level (https://www.stanleytools.co.uk/product/0-42-294/stanley-torpedo-level-200mm).
- Also: a steel tape, a soft brush, weighing boats, paper towels, oven mitts, cut-resistant gloves, white plastic
  catch sheets, masking tape, two wooden stop blocks and two C-clamps.

## Files

- `task.py`: the scene, the debris layout, `score()`, `success()` and the scripted `demo()`.
- `debriskit.py`: the task's helper module that spawns the debris fragments as rigid bodies, bonds them to the plate
  with breakable joints, and reads their poses.
- `scrapekit.py`: the task's helper module for the blade-angle rule (the gouge cap); its soil model is not used.
- `yamkit.py`: the task's helper module that spawns the modeled props (griddle plate, scraper).
- `guardkit.py`: the task's helper module for contact between the grippers' own shapes and the scraper.
- `test_task.py`: offline checks of the scorer, the debris layout and the demo plan.
