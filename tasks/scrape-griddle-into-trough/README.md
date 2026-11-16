# Scrape charred bits off a griddle over the lip into its grease tray with a scraper

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). A range-top add-on griddle
stands on the bench with its grease tray toward the robot, the near end of its plate covered with charred flakes and
crusts stuck to the steel. The robot takes a 4 in. griddle scraper from the bench, breaks the debris free and pushes it
over the lip into the tray, and sets the scraper down. The setup, scoring and fidelity notes are in `task.json`.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/scrape-griddle-into-trough/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Scrape the griddle clean into the grease tray with the scraper." \
      --scorer task_score --max-steps 2050 --store-frames

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`. Offline checks (no simulator): `python3 test_task.py`.

## Scoring

`--scorer task_score` gives 1.0 when the task succeeds (the full condition held for 1 s) and otherwise `score()` at
the end of the trial. Every share is by mass, of the 22.6 g of debris, read from where the 88 pieces really are (their
poses in the simulator; in a real trial, the three fractions weighed on a 0.01 g scale: see Real-world evaluation).

| Where a piece is | Counts as | Why |
|---|---|---|
| Inside the tray's walls (x 7.4 to 15.0, across the plate's width), below the plate's top | in the tray | delivered |
| Outside the griddle's footprint (plate, tray and splash guards) | off the griddle | on the bench or floor, or carried off on the scraper |
| Anywhere else: on the plate, against a guard, on the blade over the griddle, falling into the tray | left on the griddle | not yet delivered |

- **In the tray, by tiers.** 0.25 at 25 % in the tray (a first strip cleared), 0.5 at 60 % (most of the area done),
  0.75 at 90 % with at most 5 % left on the griddle, and 1.0 (with the set-down below) at 98 % with at most 1 % left:
  the plate is clean. A flake is 0.5 to 1.1 % of the mass and a crust 4.0 to 4.9 %, so 1 % allows one small flake to
  stay behind and 5 % about one crust. Debris pushed loose but left on the plate is not clean: it is the same carbon in
  a pile.
- **Set down.** 1.0 needs the clean plate and the scraper set down on the bench off the griddle (its origin within
  3.0 cm of the bench top and outside the griddle's footprint: a scraper lying on the plate or dropped in the grease
  tray does not count), released (no gripper's palm or fingers touching it: `touching()` in `guardkit.py`, a contact
  check, not a distance zone) and still (under 1 cm/s and 0.5 degree per step, 10 degrees/s) for 1 s. The job is not
  finished while the scraper is in hand or lying on the plate. The arm's pose is not scored.
- **Off the griddle: capped at 0.25.** More than 2 % of the debris (about 0.45 g, a few flakes) off the griddle caps
  the score at 0.25: grease and carbon thrown onto the line or the floor is a slip and fire hazard, which is why cooks
  push the pile over the lip and lift the blade slowly. Judged from where the pieces are, like the tiers.
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
weighing. The protocol fixes four things: the debris (one recipe, one stencil, one cook, checked per batch), the
positions (stops and marks on the bench), the outcome measurement (three weighed fractions plus photos) and the reset.
The tolerances are collected in the table at the end.

**Fixed marks (once).** Positions are measured from the midpoint between the arm bases with a steel rule.
- Griddle: two wooden stop blocks screwed to the bench, one against the frame's near (tray) side and one against its
  right side, put the plate's near right corner (where the lip meets the right splash guard) at x 15.0, y -17.55
  within 2 mm and the lip square to the robot within 0.5 degree. The plate's top is 5.0 cm above the bench within
  2 mm at all four corners (shim the feet) and level within 0.5 degree (a bubble level). Two C-clamps on the frame.
- Scraper: its outline traced on masking tape on the bench, blade edge toward the robot, centered at x 25.0, y -25.5.
  It is laid in the outline within 5 mm and 3 degrees.
- Camera and light: a phone on a fixed overhead mount 60 cm above the plate (all of the plate in frame) and a lamp on a
  marked stand at the far right corner shining across the plate at about 15 degrees (raking light). Both stay put.
- Catch sheets: white plastic sheet taped to the bench around the griddle and to the floor in front of the bench, so
  nothing that leaves the griddle is lost.

**The stencil (once).** The 88 pieces' outlines (positions, sizes and turns) are those in `task.py`, the same every
trial. Print them 1:1 with the script below (a 250 x 150 mm sheet: its bottom edge is the lip, its right edge the
right splash guard) and have them laser-cut through a 3 mm acrylic sheet of that size. A second, 60 x 60 mm, 3 mm
stencil carries the witness pieces: five 8 x 8 mm holes and one 17.5 x 17.5 mm hole.

    python3 - <<'EOF'
    import math
    from yam_arena.task_file import load
    m = load("tasks/scrape-griddle-into-trough/task.py").module
    svg = ['<svg xmlns="http://www.w3.org/2000/svg" width="250mm" height="150mm" viewBox="0 0 250 150">',
           '<rect width="250" height="150" fill="none" stroke="red" stroke-width="0.2"/>']
    for c in m.CHIPS:  # page right = the robot's right (-y), page up = away from the robot (+x)
        (x, y, _), (sx, sy, _) = c.pos, c.size
        u, v = (m.PLATE_Y0 + 0.250 - y) * 1000, (0.300 - x) * 1000
        svg.append(f'<rect x="{u - sy * 500:.2f}" y="{v - sx * 500:.2f}" width="{sy * 1000:.2f}" '
                   f'height="{sx * 1000:.2f}" transform="rotate({-math.degrees(c.yaw):.2f} {u:.2f} {v:.2f})" '
                   'fill="none" stroke="black" stroke-width="0.2"/>')
    open("griddle_stencil.svg", "w").write("\n".join(svg + ["</svg>"]))
    EOF

The approach follows how cleaning products are tested on baked-on soil: a soil made to a fixed recipe by weight,
applied at a fixed amount, baked on a fixed schedule, and its removal measured by weighing (for example the US federal
specification A-A-7 for oven and grill cleaners, and CSPA DCC-12 for oven cleaners, which bake a standard soil on
coupons and score removal from the coupons' weights). Those soils are spread as a thin film; this task needs separate
pieces that keep their outlines, so it uses a paste that holds the stencil's shape when cooked.

**Qualification (once, before the first trial).** Make the paste and cook the witness stencil's pieces on the plate
(below) for 6, 8, 10 and 12 minutes at 230 degrees C, one set per time. Fix the cook time as the shortest one at which
the pieces are black through (a crust broken in half shows no pale center). For the first three batches at that time,
record the witness pieces' masses and their black areas (measured on the top photo against the 1 cm grid): their
medians are the reference for every later batch. Pieces that are black through and match the reference mass and area
are the same debris from batch to batch.

**Making the debris (every trial).**
1. Plate clean, dry and cold (under 40 degrees C), the tray empty, dry and weighed (0.01 g) and seated.
2. Grease: 1.00 g of vegetable oil wiped over the plate with a paper towel, then buffed off with two dry paper towels
   weighed beforehand until they pick up no more. The film left (oil put on less the towels' gain) is at most 0.10 g:
   the plate is seasoned and lightly greased, as on a working griddle, with too little free oil to weigh.
3. Paste: 100 g all-purpose (plain) wheat flour and 100 g water at room temperature, whisked smooth and rested 10
   minutes; used within 1 hour. No other ingredients, so any lab can make the same paste.
4. Stencil: set the main stencil flush with the lip and against the right splash guard, and the witness stencil
   against the right guard at x 62 to 68. Weigh the bowl of paste, fill every hole level with a palette knife (each
   piece's paste is its hole's volume: 23 mL in all) and weigh the bowl again: the paste used is within 5 % of the
   qualification's. Lift the stencils straight up.
5. Cook: move the griddle to a two-burner range away from the robot's bench. Heat until a contact surface thermometer
   reads 230 degrees C within 10 at all five points marked on the plate (four corners and the middle of the stencil
   area), then hold it there for the fixed cook time. Turn the burners off and let the griddle cool on the range until
   all five points read under 40 degrees C. Unplug or turn off the range; nothing hot goes to the robot's bench.
6. Batch check: photograph the witness area from the overhead mount and measure each witness piece's black area
   against the grid; break the crust in half to check it is black through; lift the witness pieces with the razor
   scraper into a weighing boat and weigh the five flakes and the crust (0.01 g). The batch is accepted when the
   flakes' median area and mass and the crust's area and mass are within 25 % of the reference medians and no
   piece shows a pale center. If not, the plate is cleaned (Reset) and a fresh batch cooked. Then wipe the witness
   area clean.

   If a lab cannot make the paste pieces pass this check reliably, a fallback debris is allowed and recorded with the
   trial: hardwood lump charcoal broken and sanded into flakes and crusts of the stencil's sizes (sorted by mass to
   the same per-piece masses within 25 %), each set into its stencil hole on a level skim of the same paste, 0.5 mm
   (a 0.5 mm shim stencil), and cooked as above so the skim bonds it to the plate. The pieces then look and break
   like char and keep exact sizes, while the bond still comes from cooked-on paste. The batch check is the same.
7. Set the griddle back against its stops and clamp it; lay the scraper in its outline; take the before photo (top
   camera, raking light on). Start the trial within 2 hours of the cook.

The cooked residue is charred flour (its water cooks off), estimated at about 12 g in all; the batch check measures
it. Because each piece's paste is its stencil hole's volume, the pieces' shares of the total are close to the
simulator's: a flake 0.7 to 1.2 % of the mass (simulator 0.5 to 1.1 %), a crust 3.5 to 4.3 % (simulator 4.0 to
4.9 %), so the thresholds keep their meaning: 1 % is about one flake left, 5 % about one crust.

**Measuring the outcome (every trial).** After the trial, take the after photo (same camera and light), then collect
three fractions, each into its own weighing boat weighed beforehand, on the 0.01 g scale:
- in the tray (T): lift the tray out and weigh it; T is its gain;
- left on the griddle (L): everything still on the plate (stuck or loose), against the guards, or on the blade while
  it is over the griddle, lifted with a razor-blade scraper and a soft brush;
- off the griddle (O): everything on the catch sheets, the bench, the floor and the scraper, brushed up.
The total M = T + L + O. The shares are T / M, L / M and O / M, scored with the simulator's thresholds (the tiers, the
2 % off-griddle cap) and the set-down judged from the trial's video (the scraper lying on the bench off the griddle,
released and still for 1 s). M must be within 10 % of the mass expected from the batch check (the witnesses' total mass x
12.4, the ratio of the two stencils' hole areas); otherwise something was lost or mis-weighed and the trial is repeated. The two
photos are overlaid with a 1 cm grid registered to the plate's corners: the cells holding a dark particle 1 mm or
larger are counted and kept as the record of where residue was left, and a line visible after but not before, 1 cm or
longer, is a new scratch: the gouge cap (0.5). The photos and weights are kept with each trial, so anyone can recount.

**Reset (between trials).** Empty the tray, wash, dry and weigh it. Scrape anything left on the plate with the razor
scraper, wipe it with a damp cloth and dry it; the plate is clean when the grid photo shows no dark particle 1 mm or
larger. Every 10 trials, or when bare or rusty steel shows, re-season the plate (a thin coat of vegetable oil, 15
minutes at 250 degrees C, cooled) and take a new reference photo. The same griddle and scraper are used throughout; the
scraper is replaced if its edge has a nick 0.5 mm or deeper. Then make a fresh batch (above).

| Item | Target | Tolerance |
|---|---|---|
| Plate's near right corner | x 15.0, y -17.55 (stops) | 2 mm; lip square within 0.5 degree |
| Plate's top above the bench | 5.0 cm, level | 2 mm at each corner; 0.5 degree |
| Scraper | in its outline at x 25.0, y -25.5 | 5 mm, 3 degrees |
| Stencil | flush with the lip and the right guard | 1 mm |
| Oil film after buffing | at most 0.10 g | towels weighed (0.01 g) |
| Paste used | the qualification's mass | 5 % |
| Cook | 230 degrees C at five points, for the fixed time | 10 degrees C; 15 s |
| Cool | under 40 degrees C at five points | trial within 2 h |
| Batch check | flake and crust masses and black areas; black through | within 25 % of the reference medians |
| Mass balance | M = T + L + O vs the batch's expected mass | 10 %, else repeat |
| Weighing | 0.01 g scale | 0.01 g (1 % is about 0.12 g) |

## The debris (`debriskit.py`)

- **Real bodies, bonded.** Each of the 88 pieces is its own rigid body: a box collider with the same visible box and
  its real mass, held 0.2 mm above the plate by a breakable joint (a PhysX fixed joint from the plate to the middle of
  the piece's underside). The physics solver breaks the joint, for good, the first time holding the piece takes more
  than its break force; from then on the piece is free. Nothing is drawn or moved by a script: the blade's collider has
  to meet a piece and push it hard enough to free it, and a piece falls into the tray only when it is pushed past the
  lip. The score reads only the pieces' poses.
- **Bond strength.** Break force = 0.15 N per square cm of the piece's footprint: 0.08 to 0.13 N for a flake, 0.41 to
  0.50 N for a crust, 46 to 68 times the piece's weight, so nothing comes loose by itself. Break torque = break force
  x 2 mm (an edge that gets under a piece peels it off with less push than a flat shove). Real-world basis: a blade
  clearing a deposit pushes with the deposit's removal energy per unit area times the width it meets (N per m of edge
  = J per square m). Here that is 11 to 14 N per m for a flake and 24 to 28 for a crust. Millimanipulation
  measurements of baked food soils on stainless steel span about 0.3 to 80 J per square m (baked tomato paste 1 to
  20); heavily baked, polymerized lard is about 420 (Ali et al., Food and Bioproducts Processing, 2015). So the pieces
  are freshly charred bits from the last order: this bond is how the simulator represents the real trials' charred flour paste.
  Aged, polymerized carbon (years of it, which would need several times the arm's push) is out of scope. The value also sits below what the arm delivers: through the held scraper it frees a full row of
  about eight flakes at once only up to roughly 1.2 N in all; at 0.25 N per square cm the edge rode up over the first
  full row it met, pass after pass (measured in the simulator).
- **Calibration.** PhysX tests a joint's break force against its own measure of the joint's load, not the push on the
  piece. Pushed in the simulator with a slowly rising force, pieces came free at about 0.031 of the value written
  (a thirty-second; flakes 0.028, crusts 0.034 to 0.038), so the task writes 32 times the bond (`BREAK_CAL`): flakes come free about 11 %
  under the values above and crusts 10 to 23 % over. A broken bond is not restored by the scene's reset (one trial per
  run).
- **Real-trial bond.** The real debris is charred flour paste (see Real-world evaluation); the forces above are how
  the simulator represents it. The real bond comes from the fixed recipe, amount and cook, and every batch's witness pieces must match the qualification's masses and black areas within
  25 % and be black through before the batch is used.
- **Contact settings.** Pieces are at least 2 mm thick (the blade is 1.5 mm, moving 4 cm/s: 0.2 mm per 240 Hz
  physics step, so nothing passes through); contacts start 2 mm out and rest at zero gap; 16 position iterations; a
  piece pinched into a collider is pushed out no faster than 0.3 m/s.
- **Friction.** Carbon on steel, 0.5 static and 0.4 sliding, combined by "min" with the other surface: on the greased
  plate (0.20 and 0.15) a flake slides as easily as on a working griddle, and on the blade (0.5) it slides back off a
  40 degree blade (tan 40 degrees is 0.84) instead of riding up with it.
- **The layout is fixed.** The pieces are not in the task's object list, so the simulator's object jitter (which moves
  each free object up to 3 cm on its own) can't pile them into each other; the scene's reset puts them back where they
  started. The scraper is still jittered. The real trials cut their stencil from the same layout (Real-world
  evaluation), so the real pieces sit where the simulated ones do.
- **Why the strokes overlap.** Pushed flakes spread sideways and spill round the blade's ends; a lane's leftovers lie
  in the next lane's path, so the demo's half-blade overlaps and repeated passes pick them up.
- **The plate's surface.** A working griddle is seasoned and greased, so a steel blade slides on it easily. The plate
  has the friction of greased steel on steel (0.20 static, 0.15 sliding) instead of the simulator's default 1.2, at
  which a blade edge sticks and chatters.
- **Speed.** 88 extra bodies and their bonds slow the simulator to about 0.28 s per control step (about twice the
  scene without them).

## Runtime and seeds

- Demo: a 92 s plan (1,838 control steps), run with `--max-steps 2050`. The recorded run scored 1.0 at step 1,786
  (the trial ends once the set-down has held for 1 s, before the plan's final wait): at least 98 % in the tray and at
  most 1 % left on the griddle; the last frame shows the plate clean.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- The demo: the right hand takes the scraper by the handle from above, fingers closing across the handle (yaw 0: along
  world y), lifts it level and carries it over the right splash guard (tilted, its edge would hang below the guard's
  top), then tilts it over the plate so the blade meets the plate at 40 degrees with the edge toward the robot. The
  tilted hand is turned 6 degrees about its approach, undoing the turn the handle takes in the fingers when they
  close, so the edge lies flat on the plate. Each stroke sets the edge on the plate at x 32.5 (4.5 cm beyond the
  debris; the hand lags about 1.5 cm, so the edge lands about 3 cm clear of every piece: an edge that lands on a
  bonded piece presses it down and slides over it), aimed 10 mm into the plate (the plate stops it; a hand holding the
  scraper sits 3.5 to 6.5 mm above where it is sent, so a shallower aim lets the edge lift off mid-stroke and the
  flakes slip under it), draws it toward the robot at 4 cm/s to x 13.5 (1.5 cm past the lip, over the tray), and lifts
  it 3 cm in 0.6 s, slowly, so what rides on the blade slides off into the tray. Three lanes 5 cm apart (y -7.0, -2.0
  and 3.0; the blade covers y -12.1 to 8.1), three times over: the later passes pick up the flakes that spill round
  the blade's ends. Then the scraper is leveled over the tray, carried back over the guard, laid back where it was,
  and let go.
- Seeds: none. The layout is fixed (`object_jitter=0`); the pieces' sizes and places come from a fixed seed in
  `task.py` and are the same every trial.

## Real-world build

Everything is off the shelf. The griddle is a range-top add-on griddle top of the size above (69 x 31 cm overall,
7-gauge stainless, end grease tray), standing on its own frame with the plate's top 5.0 cm above the bench (shim the
frame's feet if needed), clamped to the bench with two C-clamps on its frame. It is heated only to make the debris, on a
two-burner range away from the bench, and is under 40 degrees C and unplugged when it is clamped down: nothing at the
bench is hot, so the trial is safe for the robot and the operator. The scraper is a 4 in. stiff griddle scraper with a
stainless blade and a polypropylene handle in line with it (150 g, handle about 3.0 x 2.4 cm, within the gripper's
opening). The debris is flour-and-water paste charred onto the plate through a stencil and cooled (see
Real-world evaluation). Also needed: the two 3 mm acrylic stencils (laser-cut from the script's drawing), a palette
knife, a contact surface thermometer, a razor-blade
scraper, a soft brush, weighing boats, paper towels, white plastic catch sheets, two wooden stop blocks, a bubble level,
a phone on an overhead mount, a desk lamp, and a kitchen scale (0.01 g).

## Files

- `task.py`: the scene, the debris layout, `score()`, `success()` and the scripted `demo()`.
- `debriskit.py`: the task's helper module that spawns the debris pieces as rigid bodies, bonds them to the plate with
  breakable joints, and reads their poses.
- `scrapekit.py`: the task's helper module for the blade-angle rule (the gouge cap); its soil model is not used.
- `yamkit.py`: the task's helper module that spawns the modeled props (griddle plate, scraper).
- `guardkit.py`: the task's helper module for contact between the grippers' own shapes and the scraper.
- `test_task.py`: offline checks of the scorer, the debris layout and the demo plan.
