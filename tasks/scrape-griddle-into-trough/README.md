# Scrape burnt pancakes off a griddle over the lip into its grease tray with a scraper

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). A range-top add-on griddle
stands on the bench with its grease tray toward the robot. Near the lip, four silver-dollar pancakes were left on the
heat and burnt onto the steel (in real trials, a fixed-recipe substitute made in minutes: black paraffin wax poured
into the same rings on the plate). The robot takes a 4 in. griddle scraper from the bench, breaks the pancakes up and
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
weighing. The protocol fixes four things: the debris (a fixed-recipe substitute made in minutes: one named wax,
poured by weight at a set temperature into one named mold on a warmed plate, checked disc by disc for area and hold), the positions (stops and marks on the bench and
the plate), the outcome measurement (three weighed fractions plus photos) and the reset. The tolerances, each with its
source, are collected in the table at the end; the equipment, with its makers' or retailers' pages, is under
Real-world build.

**The scene.** At breakfast a cook pours silver-dollar pancakes in a pancake mold on the griddle; four are left on
the heat while an order goes out and burn onto the plate. A cook clears them before the next order by setting the
scraper's edge flat on the plate behind the burnt food and pushing it toward the grease tray in overlapping strokes,
then over the lip (WebstaurantStore's griddle-cleaning guide: scrape debris into the griddle's waste drawer,
https://www.webstaurantstore.com/article/142/how-to-clean-a-griddle.html). The trial runs cold: the robot and the
operator work next to a plate at room temperature, and every trial starts at the same temperature.

**The debris: a fixed-recipe substitute for burnt pancakes.** Real burnt pancakes take about 2 hours a batch to
cook, burn and cool. Each trial instead uses four discs of black-dyed paraffin wax, poured into the same four cavities
of the same mold on the plate. The pour is set up so the wax wets the steel before it freezes: household and candle
paraffins melt at about 54 to 58 degrees C (a household paraffin spec sheet gives 130 F, 54 degrees C; CandleScience's
paraffin pillar wax MP-137 gives 135 F, 57.2 degrees C, https://candlescience.com/wax/paraffin-pillar-wax-mp-137), and
candle makers pour paraffin well above its melt point (the same CandleScience page: pour at 180 F, 82.2 degrees C).
Here the wax is poured at 75 to 85 degrees C onto spots of the plate warmed to 40 to 45 degrees C, below the melt
point, so the wax spreads and wets the steel for a moment, then freezes onto it in place; the disc then cools to room
temperature. Each disc is a dark, flat, 7 cm deposit bonded to the plate that has to be scraped off, like the burnt
pancake, and each one is checked to hold a weighed proof load before the trial. The four are made the same way every
time in about 15 minutes, most of it waiting while they cool:
- Wax: Gulf Wax household paraffin (food-grade paraffin sold for sealing jams and for candles; melted over boiling
  water, as in a double boiler, as its directions say). Paraffin is inert at room temperature, does not take up
  water, and weighs cleanly: what is scraped off is solid pieces, not a paste.
- Color: CandleScience black liquid candle dye, 12 drops per pound of wax (twice the maker's 6 drops for a dark
  shade, as dark as it goes), stirred into the melt, so the discs are black on the steel like char and their area can
  be measured on the photo. The same drops per pound for every melt.
- Amount: 14.0 g a disc (4.0 mm thick on average over the 7 cm cavity), poured by weight. The simulator's discs
  weigh 13.9 g (wax, 0.9 g per cubic cm); the total of the four pours is the trial's mass, so nothing has to be
  estimated.
- Why not the alternatives: a reusable adhesive putty (Loctite Fun-Tak, Blu Tack) pressed on in weighed pieces sticks
  but smears and rolls under a blade, sticks to the blade itself, never breaks up, and gains grease and grit each
  reuse, so its mass drifts; isomalt or hard caramel sets as fast and breaks up well but is poured at about 160
  degrees C (a burn hazard) and takes up water from the air, so its bond changes with the humidity. Paraffin pours at
  75 to 85 degrees C from a double boiler, which cannot overheat it.

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

**Melting (once per session, about 20 minutes).** 1 lb (454 g) of Gulf Wax and 12 drops of the dye in the aluminum
pouring pitcher, standing in a saucepan of water on the single-burner hot plate (a double boiler: the wax cannot pass
100 degrees C). Held at 75 to 85 degrees C (the infrared thermometer on the melt's surface) for the whole
session; topped up with wax and dye in the same ratio. One pound makes about seven trials.

**Qualification (once, before the first trial).** Make the debris three times as below and photograph each set. The
median black area of each disc is the reference for its cavity. The same three sets fix the dark gray level used for
the area (below). Then the bond is qualified on three spare discs, poured exactly as below into the mold's three
spare cavities: each is loaded with the proof-load rig (below) and water is added to the cup from a squeeze bottle,
about 5 g at a time with 5 s between additions, until the disc moves; the cup with its water is then weighed. The
three release masses and their median are recorded in the trial log beside the simulator's whole-disc value, 320 to
415 g (the sum of each disc's 22 bonds in `task.py`: 3.13 to 4.07 N, divided by 9.81 m/s2). Each spare disc must
hold at least the 86 g proof load; if one releases below it, the plate is cleaned (Reset) and the qualification is
repeated.

**Making the debris (every trial, about 15 minutes).** The room is at 20 to 24 degrees C. The arms are stopped
while wax is poured, and molten wax never goes on the robot or the scraper.
1. Plate clean, dry and at room temperature (20 to 24 degrees C on the masking-tape marks beside the cavities: tape
   reads true on an infrared thermometer, bare steel does not), the tray empty, dry, weighed (0.01 g) and seated.
   The plate is bare type 430 stainless, not oiled or seasoned: an oil film would weaken the wax's hold and make it
   vary.
2. Set the mold at its marks (the tape strips; check one near edge with the steel rule).
3. Warm: the Wagner HT1000 heat gun on its low setting, held about 15 cm above the plate and kept moving over one
   cavity's open bottom (never held on the silicone), until the tape mark beside that cavity reads 40 to 45 degrees C
   on the infrared thermometer, about 30 s a cavity.
4. Pour: the melt at 75 to 85 degrees C (the infrared thermometer on its surface); the pitcher, on a cork trivet on
   the balance, tared; pour into the warmed cavity until the balance reads -14.00 g (within 0.20 g; the reading is
   recorded). Warm and pour each of the four cavities in turn, near ones first, each pour within 30 s of its cavity
   reading 40 to 45 degrees C. About 3 minutes. The four readings' sum is the trial's debris mass, D.
5. Set and cool: after 3 minutes (within 15 s) every disc's gloss has gone matte; lift the mold straight up by its
   handles and set it aside (silicone releases from wax). Leave the discs to cool for 10 minutes from the last pour,
   and longer if needed, until every disc's top and every tape mark reads 20 to 24 degrees C.
6. Disc check. Photograph the plate from the overhead stand with the raking light on. Each disc passes when (a) it
   is bonded all round: no lifted edge (no shadow line under the edge in the raking light); (b) its black area, the
   part of its outline darker than the gray level fixed at qualification measured against the 1 cm grid, is within
   10 % of its cavity's reference; and (c) it holds the proof load (below) for 10 s without moving. A disc that fails
   is lifted with the razor scraper, its spot cleaned (Reset), and its cavity warmed and poured again with the mold
   set back at its marks. The photo, taken before the proof load, is also the trial's before photo.
7. Lay the scraper in its outline. Start the trial within 30 minutes of the pour (the wax does not change at room
   temperature, so the start does not have to be immediate).

**Proof load (every disc, every trial; mass only).** The load is the simulator's bond for one full row of a disc met
at once by the blade: 12 N per m (`BOND_EDGE` in `task.py`) x 7.0 cm (the disc's width) = 0.84 N, and 0.84 N / 9.81
m/s2 = 0.0856 kg, so 86 g. (The middle rows in `task.py` sum to 0.86 to 1.14 N, as each disc's cracks are turned a
little to the stroke; 86 g is the nominal 7 cm row.) A 1 in. nylon pulley on a small spring clamp is clipped to the
lip in line with the disc, its groove level with the plate's top. A 7.0 cm piece of wooden paint stirrer stands on edge
on the plate against the disc's far edge, square to the lip. A string tied to the middle of the stick runs along the
plate toward the lip, over the pulley, and down to a 9 oz plastic cup filled with water to 86 g (cup, water and string
loop weighed together on the balance, within 1 g). The cup is lowered onto the string by hand and let go; the disc
passes when it does not move for 10 s (the stick does not advance past a pencil mark on a strip of masking tape beside
it). The same rig, with water added until the disc moves, gives the release mass at qualification (above).

**Measuring the outcome (every trial).** After the trial, take the after photo (same camera and light), then collect
three fractions, each into its own weighing boat weighed beforehand, on the 0.01 g balance:
- in the tray (T): lift the tray out and weigh it (it sits across the balance's pan, centered); T is its gain;
- left on the griddle (L): every piece still on the plate (stuck or loose), against the guards, or on the blade while
  it is over the griddle, lifted with the razor scraper and the brush. A thin film of wax smeared on the steel (a
  dull patch with no raised edge that the brush and a single razor pass do not lift as a piece) is not a piece: it is
  not counted in L and is removed at the reset;
- off the griddle (O): everything on the catch sheets, the bench, the floor and the scraper, brushed up.
The total M = T + L + O. The shares are T / M, L / M and O / M, scored with the simulator's thresholds (the tiers, the
2 % off-griddle cap) and the set-down judged from the trial's video (the scraper lying on the bench off the griddle,
released and still for 1 s). M must be within 5 % of D (the poured mass); otherwise something was lost or mis-weighed
and the trial is repeated. The two photos are overlaid with the 1 cm grid registered to the plate's corners: the
cells holding a dark particle 1 mm or larger are counted and kept as the record of where residue was left, and a line
visible after but not before, 1 cm or longer, is a new scratch in the steel: the gouge cap (0.5). The photos and
weights are kept with each trial, so anyone can recount.

**Nothing left on the griddle (the 1.0 condition, real trials).** In the simulator, nothing left means no fragment on
the griddle (`FULL_RESIDUE` = 0 in `task.py`), and the smallest fragment weighs 0.42 g, so any fragment left is at
least 0.42 g. In a real trial, nothing left means both: (a) L under 0.40 g, less than the smallest simulated fragment
(0.7 % of 55.5 g, well inside the 2 % allowed for 0.75), read on the 0.01 g balance; and (b) no piece 1 mm or larger
on the plate, guards or blade, checked by eye under the raking light without wiping, any doubtful piece measured
against the 6 in. steel rule, and confirmed on the after photo with the 1 cm grid. A thin film (above) does not count.
The other thresholds are the simulator's, applied to the weighed shares.

**Reset (between trials, about 5 minutes).** Weigh the tray's gain (above), then empty it into the waste bin, wipe it
with a paper towel and seat it. Scrape anything left on the plate with the razor scraper, wipe the four cavity spots
with a paper towel wetted with odorless mineral spirits (paraffin is a hydrocarbon and dissolves in it), then with
a dry towel; the plate is clean when the raking-light photo shows no dark particle 1 mm or larger and no wax film
(no dull patch) on the four spots. Wait until the spirits have flashed off and the spots are dry (a minute or
two). Lay the scraper in its outline. The same griddle, mold and scraper are used throughout; the scraper is
replaced if its edge has a nick 0.5 mm or deeper. Then make the next debris (above). A trial, its reset and the next
debris take about 25 minutes, with the wax kept melted all session.

| Item | Target | Tolerance | Source |
|---|---|---|---|
| Plate's near right corner | x 15.0, y -17.55 (stops) | 2 mm; lip square within 0.5 degree | simulator's layout; steel tape |
| Plate's top above the bench | 5.0 cm, level | 2 mm at each corner; 0.5 degree | simulator's layout; torpedo level |
| Scraper | in its outline at x 25.0, y -25.5 | 5 mm, 3 degrees | simulator's start pose |
| Mold | near cavities' edges 1.0 cm from the lip; near right edge 6.55 cm from the right guard | 2 mm | simulator's disc centers; 6 in. steel rule |
| Room | 20 to 24 degrees C | | the trial runs at room temperature |
| Plate before the pour | bare 430 stainless, clean, dry, unoiled; 20 to 24 degrees C | no particle 1 mm, no film on the photo | wax bonds to clean steel; infrared thermometer on the tape marks |
| Plate spot at the pour | 40 to 45 degrees C (heat gun, low) | pour within 30 s | below paraffin's 54 to 58 degrees C melt point, so the wax wets the steel then freezes; infrared thermometer on the tape mark |
| Melt (pour temperature) | Gulf Wax, 12 drops of dye per lb, 75 to 85 degrees C | double boiler (at most 100 degrees C) | Gulf Wax directions (melt over boiling water); CandleScience pour guidance (180 F, 82 degrees C, for paraffin); CandleScience dye rate |
| Wax per disc | 14.0 g (simulator 13.9 g) | 0.20 g | simulator's disc mass; 0.01 g balance (repeatability 0.02 g) |
| Disc thickness | real 4.0 mm over the whole 7 cm cavity; simulator 5 mm at about 80 % coverage | mass matched, not thickness | real: 14.0 g / 0.9 g per cubic cm / 38.5 square cm; simulator: `CAKE_T` in `task.py` |
| Set and cool | mold off at 3 minutes (matte); 10 minutes from the last pour; disc tops 20 to 24 degrees C | 15 s on the mold | infrared thermometer (wax reads true at emissivity 0.95) |
| Disc check | bonded all round; black area vs the cavity's reference; proof load | no lifted edge; 10 %; 86 g held 10 s | qualification's three sets; `BOND_EDGE` x 7 cm / 9.81 m/s2 |
| Bond qualification | release mass of 3 spare discs, beside the simulator's 320 to 415 g a disc | each at least 86 g | sum of each disc's bonds in `task.py` |
| Nothing left (1.0) | L under 0.40 g and no piece 1 mm or larger; a thin film not counted | | smallest simulated fragment 0.42 g; steel rule and 1 cm grid photo |
| Start | within 30 minutes of the pour | | wax is stable at room temperature |
| Mass balance | M = T + L + O vs D, the poured mass | 5 %, else repeat | four weighed pours |
| Weighing | U.S. Solid USS-DBS66-20, 0.01 g | 0.01 g (1 % of the debris is about 0.55 g) | maker's specifications |

## The debris (`debriskit.py`)

- **Burnt pancakes as mosaics.** Each pancake is 7.0 cm across (the mold's cavity) and 5 mm thick, cracked into 22
  fragments: five rows across the disc, each split where it cracked (about 1.2 cm across, 1 mm cracks, from a fixed
  seed). The outline is the disc's, stepped at the row ends (every corner within 4 mm of the cavity's circle), so the
  fragments cover about 80 % of the disc. Density 0.9 g per cubic cm (paraffin wax, the real trials' debris): 13.9 g
  a pancake, 55.5 g in all, the same as the real 14.0 g pours. A burnt pancake cracks as it chars, and a cold wax disc
  cracks and chips under a driven blade; here each fragment comes free on its own. A real disc may come off in fewer,
  larger pieces than 22; the score is by mass, so the pieces' number does not change it.
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
- **Real-trial bond.** The real debris is wax discs poured onto plate spots warmed to 40 to 45 degrees C and cooled
  to room temperature (see Real-world evaluation); the bond above is how the simulator represents them. The real bond
  comes from the fixed wax, pour temperature and mass, plate temperature and cooling time. Every disc is checked to
  hold 86 g pulled across the plate (the simulator's bond for one 7 cm row, 0.84 N) for 10 s, and the release mass of
  three spare discs is recorded at qualification beside the simulator's 320 to 415 g a disc (3.13 to 4.07 N).
- **Thickness.** The simulator's discs are 5 mm thick at about 80 % coverage; the real pours are 4.0 mm thick over the
  whole cavity. The mass (13.9 g and 14.0 g) and the outline are matched; the edge the blade meets is 1 mm lower in a
  real trial.
- **Contact settings.** Fragments are 5 mm thick (the blade is 1.5 mm, moving 4 cm/s: 0.2 mm per 240 Hz physics step,
  so nothing passes through); contacts start 2 mm out and rest at zero gap; 16 position iterations; a fragment pinched
  into a collider is pushed out no faster than 0.3 m/s.
- **Friction.** A dry piece on steel, 0.5 static and 0.4 sliding (estimated), combined by "min" with the other
  surface: on the plate (0.20 and 0.15, waxy or greased films on steel) a freed piece slides as it does on a working
  griddle, and on the blade (0.5) it slides back off
  a 40 degree blade (tan 40 degrees is 0.84) instead of riding up with it.
- **The layout is fixed.** The fragments are not in the task's object list, so the simulator's object jitter (which
  moves each free object up to 3 cm on its own) can't pile them into each other; the scene's reset puts them back
  where they started. The scraper is still jittered. The real discs are poured at the same places (the mold's
  marks).
- **Why the strokes overlap.** Pushed fragments spread sideways and spill round the blade's ends; a lane's leftovers
  lie in the next lane's path, so the demo's half-blade overlaps and repeated passes pick them up.
- **The plate's surface.** A working griddle is seasoned and greased, so a steel blade slides on it easily; in the
  real trials the plate is unoiled (oil would weaken the wax's hold) and the blade slides on the wax it smears. The plate
  has the friction of a waxy or greased film on steel (0.20 static, 0.15 sliding; steel on steel greased about 0.16,
  Engineering ToolBox) instead of the simulator's default 1.2, at
  which a blade edge sticks and chatters.

## Runtime and seeds

- Demo: a 110 s plan (2,190 control steps), run with `--max-steps 2400`. The recorded run scored 1.0 at step 2,139
  (the trial ends once the set-down has held for 1 s, before the plan's final wait): at least 98 % in the tray and no
  fragment left on the griddle; the last frame shows the plate clean and the pancakes' fragments in the tray.
- Model run limit: 2,400 steps (2 minutes at 20 Hz). Dream-1: pending on this task.py.
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

Everything is off the shelf and chosen to be cheap and widely sold; the pages below give each item's published
specifications, and the prices are retail listings seen in November 2026 (rounded).

**Rough rig cost: about $530** (griddle $105 to $150, balance $66, hot plate $75, heat gun about $25, the other
items below about $155, and about $40 of tape, boats, sheets, clamps and blocks), not counting the robot and the phone. Consumables
are about $1 a trial (56 g of wax and a little dye and spirits).
- Griddle: Assure Parts 177AOGT1227 add-on 2-burner griddle top, 27 x 12-1/4 x 4 in., 22-3/8 x 12-1/4 in. cooking
  surface of 7-gauge type 430 stainless, 1-1/2 in. rear and side edges, adjustable plate height, 3 x 12-1/4 x 1-1/4 in.
  grease tray across the open front, about $105 to $150
  (https://www.webstaurantstore.com/12-14-x-27-x-4-addon-2-burner-griddle-top/177AOGT1227.html). It stands on the
  bench on its frame with the plate's top 5.0 cm above the bench, clamped with two C-clamps on its frame. It is never
  heated on a burner: only the four cavity spots are warmed to 40 to 45 degrees C with the heat gun before the pour. Its grease tray (about 0.5 kg, estimated from its size in sheet steel;
  weighed at setup) with the debris is well under the balance's 2,000 g.
- Scraper: Dexter-Russell Basics P94850 4 in. griddle scraper: 4 in. stiff stainless blade, 4-1/2 in. polypropylene
  handle, 8-1/2 in. overall, 4.8 oz (136 g), about $11 to $25
  (https://www.wasserstrom.com/restaurant-supplies-equipment/dexter-russell-p94850-basics®-4-griddle-scraper-122621).
  The handle (3.0 x 2.4 cm in the simulator) is within the gripper's opening.
- Mold: Norpro 957R silicone silver-dollar pancake mold, seven 2.75 in. (7 cm) cavities, 9.5 x 9 x 1.75 in., built-in
  handles, heat safe to 450 F, about $11 to $15; lifted off once the wax has set
  (https://www.bakedeco.com/detail.asp?id=49842,
  https://chefsupplies.ca/products/norpro-silver-dollar-size-pancake-ring-957r).
- Wax: Gulf Wax household paraffin, 16 oz, a few dollars at grocery stores (Royal Oak Enterprises; food-grade, for
  canning and candles; melt over boiling water, as in a double boiler, never directly on a burner;
  https://hannaford.com/groceries/home-office/kitchen-dining/gulf-wax-household-paraffin-wax-for-canning-candlemaking-16-oz-pkg.html).
  About 56 g a trial.
- Dye: CandleScience black liquid candle dye, 1 oz, $5.43 (3 drops per lb for a light shade, 6 for a dark one; works in
  paraffin; https://candlescience.com/color/black-liquid-dye).
- Melting: CandleScience small aluminum pouring pitcher, 1 lb (454 g), $10.75
  (https://candlescience.com/equipment/small-pouring-pitcher), standing in a 2 qt saucepan of water on a Cuisinart
  CB-30 1,300 W single-burner hot plate, about $75 (any thermostatic hot plate will do: the water bath sets the
  temperature; https://www.macys.com/shop/product/cuisinart-cb-30-countertop-single-burner?ID=8299114),
  and a cork trivet for the pitcher on the balance.
- Cleaning: Klean-Strip odorless mineral spirits, about $10 a quart (removes grease and oil; https://kleanstrip.com/?p=366) on paper
  towels, for the wax film left on the plate.
- Balance: U.S. Solid USS-DBS66-20 precision balance, 2,000 g by 0.01 g (repeatability 0.02 g, linearity 0.03 g),
  128 x 157 mm stainless pan, external calibration, $65.79
  (https://ussolid.com/u-s-solid-2000g-x-0-01-g-precision-balance-ac-dc-power-supply.html). It weighs the pours, the
  tray and the fractions; it is calibrated with the weight its manual names at the start of each session, after a
  30-minute warm-up, away from drafts.
- Heat gun: Wagner HT1000, two settings (750 F and 1,000 F; the low one is used), about $22 to $25
  (https://www.wagnerspraytech.com/product/ht-heat-gun/ht1000-heat-gun/). It warms the four cavity spots to 40 to 45
  degrees C before the pour.
- Proof-load rig: a 1 in. nylon pulley and a small spring clamp (a few dollars at a hardware store), a wooden paint
  stirrer cut to 7.0 cm, string, a 9 oz plastic cup and a squeeze bottle of water.
- Infrared thermometer: Etekcity Lasergrip 774, -50 to 450 degrees C, adjustable emissivity (set to 0.95), $18.99
  (https://etekcity.com/products/lasergrip-774-upgrade). It is meant for high-emissivity surfaces and reads shiny
  metal low, so it is aimed at the wax, the melt and the masking-tape marks on the plate, never at bare steel.
- Overhead camera: a phone on a NEEWER X6 overhead phone stand, 26.5 to 80 cm high, clamped to the bench, $25.99
  (https://neewer.com/collections/all-product/products/neewer-entry-level-version-x6-phone-stand-66608636).
- Raking light: IKEA TERTIAL clamp work lamp with an 800-lumen SOLHETTA LED bulb, about $20 with the bulb
  (https://www.ikea.com/ca/en/catalog/products/20355434/).
- Razor scraper: Red Devil 3010, 1 in. blade, a few dollars
  (https://www.dillonsupply.com/Catalog/Products/Hand-Tools/Scrapers-Spreaders-And-Trowels/Hand-Scrapers/red-devil-3010).
- Rule and level: Empire 27302 6 in. steel rule (https://www.toolnut.com/products/empire-27302-6-steel-rule) and a
  Stanley 0-42-294 torpedo level (https://www.stanleytools.co.uk/product/0-42-294/stanley-torpedo-level-200mm).
- Also: a steel tape, a soft brush, weighing boats, paper towels, oven mitts (for the pitcher), nitrile gloves, white
  plastic catch sheets, masking tape, two wooden stop blocks and two C-clamps.

### Budget build

The named items above stay as they are, and nothing changes for the sim or the scoring. These cheaper equivalents do
the same jobs:

- Griddle: buy the same Assure Parts 177AOGT1227 from WebstaurantStore, $104.49 (other sellers list it at up to
  $406.99). It is the sim's griddle, so only the source changes.
- Heat: the lab's own stove ($0) or a single electric burner such as the Elite Gourmet ESB-301BF (about $20) in place of
  the Cuisinart CB-30 hot plate ($75). The water bath caps the melt at 100 degrees C and the IR thermometer holds it at
  75 to 85 degrees C, so the burner only has to boil water.
- Scale: an American Weigh Scales AWS-100-CAL pocket scale, 100 g by 0.01 g, with its 100 g calibration weight
  ($17 to $20), in place of the $66 balance. It has the same 0.01 g readability. Each fraction is under 56 g and is
  weighed in a pre-weighed boat, so it stays under 100 g: T is the debris brushed out of the tray into a boat (the
  ~0.5 kg tray is over the scale's capacity), and each 14 g pour is weighed into a small pre-weighed paper cup, poured
  into the cavity and the emptied cup re-weighed. The 5 % check of M against D still catches anything lost in the
  brushing. The 86 g proof-load cup is under 100 g; at qualification the release cup's water is weighed in portions
  under 100 g and summed.
- Camera and light: a clamp-on overhead phone arm (about $12 to $15) in place of the NEEWER X6, and any clamp lamp the
  lab has in place of the TERTIAL; the camera only has to look straight down and the lamp only has to rake the plate.

Primary rig about $530; budget build about $350 (excluding the robot and a phone).

## Files

- `task.py`: the scene, the debris layout, `score()`, `success()` and the scripted `demo()`.
- `debriskit.py`: the task's helper module that spawns the debris fragments as rigid bodies, bonds them to the plate
  with breakable joints, and reads their poses.
- `scrapekit.py`: the task's helper module for the blade-angle rule (the gouge cap); its soil model is not used.
- `yamkit.py`: the task's helper module that spawns the modeled props (griddle plate, scraper).
- `guardkit.py`: the task's helper module for contact between the grippers' own shapes and the scraper.
- `test_task.py`: offline checks of the scorer, the debris layout and the demo plan.
