# Scrape stuck-on debris off a griddle over the lip into its grease tray with a scraper

Files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). A range-top add-on griddle
stands on the bench with its grease tray toward the robot. Near the lip, 22 separate pieces of black Play-Doh are
pressed flat onto the steel plate, each stuck where it was pressed, standing in for food burnt onto a griddle (a
standard substitute made in about 5 minutes from a freshly opened can, nothing heated). The robot takes a 4 in.
griddle scraper from the bench, breaks the pieces free with its blade and pushes them over the lip into the tray, and
sets the scraper down in a taped square on the bench. Only the scraper may move the dough. The setup, scoring and fidelity notes are in `task.json`.

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
the end of the trial. Every share is by mass, of the debris (22 pieces of 2.50 g, 55.0 g, in the simulator and in a
real trial), read from where the pieces really are (their poses in the simulator; in a real trial, the three
fractions weighed on a 0.01 g scale: see Real-world evaluation). The definitions are the same in both.

| Where a piece is | Counts as | Why |
|---|---|---|
| Inside the tray's walls (x 7.4 to 15.0, across the plate's width), below the plate's top | in the tray | delivered |
| Outside the griddle's footprint (plate, tray and splash guards), including on the scraper once it is set down on the bench | off the griddle | on the bench or floor, or carried off on the scraper |
| Anywhere else: on the plate, against a guard or on its top, on the blade over the griddle, falling into the tray | left on the griddle | not yet delivered |

- **Only the scraper counts.** A piece in the tray is credited only if the held scraper put it there:
  1. its bond let go under the blade: when the piece first leaves its place (0.5 mm from where it was pressed), the
     held blade or its neck is touching it, directly or through pieces the blade is pushing (within 0.1 s);
  2. it went over the lip under the blade: within 0.5 s of the blade's last push;
  3. no part of the robot ever touched it.
  Touching is the blade within 0.5 mm of the piece (the blade and the pieces are exact boxes, and the solver rests
  touching bodies at zero gap); pushing is touching while the piece moves 0.2 mm or more that step (4 mm/s); pieces in a pile
  pass the push on the same way. The handle does not scrape: a piece pushed or flicked with the handle is not
  credited. A piece in the tray that is not credited counts in no share (`Ledger` in `task.py`). Scooping a piece up
  on the blade and tipping it into the tray is scraper use and counts.
- **Hands off the dough: 0.** The robot's own shape touching any piece at any time scores the whole trial 0: either
  any point of the arms' own collision meshes (upper arm, forearm, wrist, palm and both fingertips, from the arm's
  description, as the simulator poses each link every step) within 1 mm of a piece or inside it, while that piece
  moves 0.2 mm or more that step (a slow push counts too). A cook does not clear a griddle by hand (in service the plate is hot), and the task is the
  scraper's job; a 0 removes any reason to mix hand moves into a run. A hand passing near a piece, or resting on a
  piece that does not move, is not a touch. In a real trial a finger, palm or arm pushing, pinching or lifting any
  piece on the video, or dough on a white finger cover after the trial, scores 0.
- **In the tray, by tiers.** Shares in the tray are credited shares (below). 0.25 at 25 % in the tray (about six pieces), 0.5 at 60 %, 0.75 at 90 % with at most
  5 % left on the griddle (one piece, 4.5 %), and 1.0 (with the set-down below) at 98 % with nothing left on the
  griddle: every piece is off the plate. In the simulator 98 % means all 22 pieces in the tray; one piece left behind
  scores 0.75, two score 0.5. A piece pushed loose but left on the plate is not clean: it is the same debris moved.
- **Dough on the scraper.** Dough left on the scraper may be knocked into the tray before the set-down, for example by
  drawing the blade across the tray's rim; it then counts in the tray. Only what is on the blade at the set-down
  counts as off the griddle.
- **Picked up, then set down in the taped square.** The scraper starts on the bench with its origin (where the neck
  meets the handle) 8.5 cm outside a 12 cm square of blue painter's tape (outer edge, centered at x 25.0, y -40.0), so
  its start never satisfies the set-down. 1.0 needs the clean plate and the scraper picked up (held, its origin 5 cm
  above where it lay; a scraper dragged along the bench is not picked up) and then set down in the square (its origin
  inside the tape's outer edge and within 3.0 cm of the bench top: a scraper left on the plate, dropped in the tray,
  put back where it started or anywhere else on the bench does not count), released (no gripper's palm or fingers
  touching it: `touching()` in `guardkit.py`, a contact check, not a distance zone) and still (under 1 cm/s and 0.5
  degree per step, 10 degrees/s) for 1 s. The job is finished once the scraper is out of hand in its place. A scraper
  left on the plate can still reach 0.75. The arm's pose is not scored.
- **Off the griddle: capped at 0.25.** More than 5 % of the debris (more than one piece) off the griddle caps the score
  at 0.25: food thrown onto the line or the floor is a slip and fire hazard, which is why cooks push the pile over the
  lip and lift the blade slowly. Judged from where the pieces are, like the tiers.
- **Gouge: a proxy, capped at 0.5.** While the scraper is held and sliding with its edge on the plate (within 0.6 mm
  of the plate's top, about the simulator's resting gap; 1.0 mm once a stroke has begun), with the blade steeper than
  70 degrees to the plate, or its edge tilted more than 10 degrees out of the plate's plane so only one corner touches,
  sliding 1 cm or more in a stroke, the score is capped at 0.5 for the rest of the trial (`scrapekit.py`, with no soil
  in it: only its blade-angle rule is used). The simulation has no scratches; this stands in for the chatter marks
  and scored lines a steep or cornered blade leaves in a griddle's surface. In a real trial the cap is judged on the
  bare steel plate itself: a new scratch in the steel 1 cm or longer, against a photo taken before the trial under a
  raking light.
- Doing nothing scores 0. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

**Ways to game it, and what stops each** (each has a check in `test_task.py`):

| Strategy | Result |
|---|---|
| Push, pinch or carry pieces with a gripper, with or without the scraper in the other hand | 0 (hands off the dough) |
| Break a bond with a finger, then scrape the piece in | 0 |
| Sweep pieces with the forearm or wrist | 0 |
| Pieces reaching the tray with no blade on them (a tipped griddle or moved tray: both are fixed bodies here and clamped in real trials) | not credited |
| Blade hovering near a piece (2 mm) while something else moves it | not freed by the blade, not credited |
| A piece knocked loose by anything but the blade, then scraped in | not credited |
| Push with the handle, or with a scraper nobody holds (thrown, let go) | not credited |
| Push a piece to the lip, then let it go over later (0.5 s or more) by itself | not credited |
| Leave the scraper where it started, drag it into the square, leave it on the plate or in the tray | no 1.0 |
| Fling pieces off the griddle | over 5 % off caps at 0.25 |
| Scrape at a steep angle or on a corner | caps at 0.5 |
| Do nothing, or only pick up the scraper | 0 |

## Real-world evaluation (reproducible protocol)

Every real trial starts from the same measured state and is scored on the same shares as the simulator, read by
weighing. The protocol fixes four things: the debris (22 weighed pieces of black Play-Doh from a freshly opened can,
pressed onto the bare plate through a fixed template, each checked by mass and by eye), the positions (stops and
marks on the bench, the template on the plate), the outcome measurement (three weighed fractions plus photos) and the
reset. The tolerances are collected in the table at the end; the equipment, with its makers' or retailers' pages, is
under Real-world build.

**The scene.** On a working griddle, food left on the heat burns onto the plate in separate stuck-on bits. A cook
clears them before the next order by setting the scraper's edge flat on the plate behind them and pushing toward the
grease tray in overlapping strokes, then over the lip (WebstaurantStore's griddle-cleaning guide: scrape debris into
the griddle's waste drawer, https://www.webstaurantstore.com/article/142/how-to-clean-a-griddle.html). The trial
runs cold: the griddle is never heated, so the robot and the operator work next to a plate at room temperature, and
every trial starts at the same temperature.

**The debris: a standard substitute, made in about 5 minutes.** Each trial uses 22 pieces of black Play-Doh (Hasbro)
from a freshly opened, sealed 4 oz (113 g) can, each piece weighed to 2.50 g and pressed onto the clean, dry steel
through a template, so every piece has the same mass, size and place every trial.
- Why it works as stuck-on food: Play-Doh pressed onto clean, dry, bare steel sticks to the steel where it was pressed,
  so each piece has to be broken free by the scraper's edge driven under it at a low
  angle, as burnt-on food does. It sticks only where it was pressed on, so a blade sliding under a piece pushes the
  freed piece along the plate and over the lip. The pieces are separate (1.6 cm apart), so each
  is freed on its own and strokes have to overlap to collect them all.
- Why it is repeatable: one named product made to one formula, from a sealed can opened for that trial (the can's
  printed lot code is recorded), at room temperature; each piece's mass is set by weighing, its outline and thickness by
  the template's hole, its contact by a firm press, checked by eye below. Nothing is mixed, heated,
  cooked or cooled. The dough left in the can is thrown away, so every trial starts from fresh dough.
- Black: the pieces stand out against the steel on the overhead photos, so what is left is easy to see and count.

**Fixed marks (once).** Positions are measured from the midpoint between the arm bases with a steel tape.
- Griddle: two wooden stop blocks screwed to the bench, one against the frame's near (tray) side and one against its
  right side, put the plate's near right corner (where the lip meets the right splash guard) at x 15.0, y -17.55
  within 2 mm and the lip square to the robot within 0.5 degree. The plate's top is 5.0 cm above the bench within
  2 mm at all four corners (the griddle's plate height adjusts; shim the frame's feet if needed) and level within
  0.5 degree (the torpedo level). Two C-clamps on the frame. The griddle stays clamped for every trial and is never
  heated. An equivalent plain steel plate of the same size, with the same 3.8 cm guards and a tray of the same size
  across its near end, set at the same marks, serves as well.
- Scraper: its outline traced on masking tape on the bench, blade edge toward the robot, centered at x 25.0, y -25.5.
  It is laid in the outline within 5 mm and 3 degrees.
- Set-down square: four strips of ScotchBlue painter's tape (3/4 in.) on the bench forming a square 12 cm on a side
  (outer edge), centered at x 25.0, y -40.0, right of the scraper's outline, measured with the steel tape.
- Cameras and light: the phone on the overhead stand, 60 cm above the plate, looking straight down (all of the plate,
  the scraper's outline and the taped square in frame), records video of the whole trial and takes the photos. A
  second phone on a small tabletop tripod at the plate's near left corner, at plate height, looks along the lip and
  records the same trial: it shows the blade at the pieces and every piece going over the lip, and a finger near the
  dough from the side. The work lamp, clamped on a marked spot at the far right corner, shines across the plate at
  about 15 degrees (raking light). The cameras and the lamp stay put. A 1 cm grid printed on paper is photographed on the plate once, to register the
  photos to the plate's corners.
- Catch sheets: white plastic sheet taped to the bench around the griddle and to the floor in front of the bench, so
  nothing that leaves the griddle is lost.

**The template (made once).** The layout is printed at 100 % on one US Letter sheet (landscape): 22 squares 1.9 cm on
a side, in four rows 3.5 cm apart (6, 5, 6 and 5 squares; 3.5 cm center to center along a row; the 5-square rows
offset by half), and two edge lines. Measured from the near edge line, the rows' centers are at 2.5, 6.0, 9.5 and
13.0 cm; measured from the right edge line, the 6-square rows' centers are at 6.80, 10.30, 13.80, 17.30, 20.80 and
24.30 cm and the 5-square rows' at 8.55, 12.05, 15.55, 19.05 and 22.55 cm. The sheet is glued with a glue stick onto
Elmer's 3/16 in. (4.8 mm) white foam board, the board is cut along the two edge lines and around the sheet, and the 22
squares are cut out with an X-ACTO knife against the steel rule. The print is checked with the steel rule (each
square 1.9 cm within 0.5 mm, centers within 1 mm). On the plate the template's near edge sits along the lip and its
right edge against the right splash guard, so the pieces' centers fall at x 17.5, 21.0, 24.5 and 28.0 and at y -10.75
to 6.75 (6-piece rows) and -9.0 to 5.0 (5-piece rows), as in the simulator.

**Qualification (once, before the first trial).** From one can, make the debris as below on a clean plate and check by
hand: each 2.50 g ball pressed in its hole fills it level with the template's top (about 4.8 mm thick); each piece
lies flat and fully in contact with the plate, with no lifted edges; and the trial's scraper, held at about 30 degrees with its edge on the plate and pushed toward
the lip, breaks each piece free in one stroke and carries it over the lip, the piece leaving the blade as it drops
into the tray. The result and photos are kept in the trial log.

**Making the debris (every trial, about 5 minutes).** The room is at 20 to 24 degrees C (the ThermoPro TP49). The
arms are stopped at their ready pose while the debris is made; the operator wears nitrile gloves.
1. Plate clean, dry and at room temperature (the Reset below), bare type 430 stainless, not oiled or seasoned. The
   tray is empty, dry, weighed (0.01 g) and seated.
2. Lay the template on the plate, its near edge along the lip and its right edge against the right splash guard.
3. Open a fresh, sealed 4 oz can of black Play-Doh and record the lot code printed on it.
4. Weigh the pieces: on the scale, tared on a weighing boat, pinch off dough and roll it into a ball between the
   gloved palms, adding or pinching off until it reads 2.50 g within 0.05 g; set it aside on a sheet of Reynolds
   parchment paper. 22 balls, about 6 s each. Then weigh all 22 together on the boat: 55.00 g within 0.30 g (the
   total mass check). The total is the trial's debris mass, D.
5. Press the pieces: drop one ball into each hole, lay a square of parchment paper over it, set the measuring cup on
   it base down, and stand a full 1 L water bottle (1 kg) on the cup for 3 seconds. Every piece gets the same press,
   which sets how hard it sticks. The cup's base is wider than the hole and stops on the
   template, which sets each piece's thickness, and the parchment lets the cup lift clean.
6. Lift the template straight up and set it aside. The pieces stay on the steel where they were pressed; a piece that
   lifts with the template or tears is removed, its spot wiped dry, and a new 2.50 g ball pressed through the template
   laid back in place.
7. Contact check, every piece, by eye: each piece is flat and fully in contact with the plate, with no lifted edges.
   A piece that is not is removed, its spot wiped dry, and a new piece pressed in its place through the template.
8. Photograph the plate from the overhead stand with the raking light on: 22 pieces, each in its place. This is the
   trial's before photo.
9. Fit fresh covers on the robot's fingers (Protecting the robot's hands, below) and lay the scraper in its outline.
   Start the trial within 10 minutes of pressing the last piece.

**Batch checks, every trial.** The can's printed lot code (one fresh can a trial); each piece 2.50 g within 0.05 g; the
22 together 55.00 g within 0.30 g; a visual check that every piece is flat and fully in contact with the plate, with
no lifted edges; the before photo shows all 22 in place. They are
written in the trial log with the room temperature.

**Protecting the robot's hands.** Play-Doh is non-toxic and water-based, but it is salty, and crumbs pressed into a
gripper's pads would change their grip; the same arms are shared with other tasks. The hands are meant to touch only
the scraper's handle, but a policy can drive a finger into the dough or brush a guard, so the fingers are covered for
every trial.
- Covers: one Kimtech Pure G3 white nitrile glove (Kimberly-Clark, size L; white, so black dough on a cover shows) over each of the two fingers of the hand
  that may use the scraper (both hands if the policy may use either). A YAM finger's pad is about 4 cm wide and 1.8
  cm tall, and its finger about 6.8 cm wide: too wide for the fingers of a glove or for finger cots, so the finger goes
  into the glove's palm, its pad against the glove's palm side, until the finger's tip is at the glove's knuckles. The
  glove's five fingers are folded back flat over the outer face of the finger (away from the pads) and held, with the
  glove's cuff, by two turns of ScotchBlue painter's tape round the finger body above the pad. The pad's face and tip
  are then covered by one smooth layer of nitrile, nothing loose is on the gripping side, and nitrile keeps the pad's
  grip on the polypropylene handle. Before the trial, the hand is opened fully and closed once with nothing held, to
  check the covers move freely.
- Changed every trial (two gloves a hand), and at once if a cover is torn. After each trial the covers are looked at
  under the raking light before they come off: dough on a cover means a finger touched the dough, and the trial
  scores 0. A used cover is peeled off with the tape into the waste bin.
- Wipe-down: at each reset, the scraper's handle and blade are wiped with a damp microfiber cloth and dried, so the
  hands only ever take a clean handle. If dough reaches a bare finger or the wrist, the arm is stopped and the part
  wiped with a damp microfiber cloth (water only; Play-Doh dissolves in water) and dried before the next trial.
- Tray and layout: the debris starts in the middle of the plate, at least 10 cm from the bench's edge on the robot's
  side, and leaves only over the lip into the griddle's own tray, which sits below the plate's top at the griddle's
  near end; the arms' path is clear. The scraper's outline is on the clean bench beside the griddle, so a hand
  reaching for it passes over clean bench only. The catch sheets keep anything thrown off the griddle off the bench
  where the hands work.

**Measuring the outcome (every trial).** Right after the trial, take the after photo (same camera and light), then
collect three fractions, each into its own weighing boat weighed beforehand, on the 0.01 g scale, within 5 minutes
of the trial's end:
- in the tray (T): lift the tray out and weigh it (it sits across the scale's platform, centered); T is its gain;
- left on the griddle (L): every piece still on the plate (stuck or loose), against the guards, or on the blade while
  it is over the griddle, lifted with the razor scraper. A thin smear on the steel (a stain with no raised edge that a
  single razor pass does not lift as a piece) is not a piece: it is not counted in L and is wiped off at the reset;
- off the griddle (O): everything on the catch sheets, the bench, the floor and the scraper once it is set down
  (dough still on the blade at the set-down is off the griddle; dough knocked into the tray before then is in T).
Then the two videos are watched back: each piece that reached the tray other than broken free and pushed over the
lip by the held blade (or by pieces the blade was pushing) is counted (n pieces); a finger, palm or arm pushing,
pinching or lifting any piece scores the trial 0. The total M = T + L + O, with T as weighed. The shares are
(T - 2.50 g x n) / M in the tray (an uncredited piece counts in no share), L / M and O / M, scored with the
simulator's thresholds (the tiers, the 5 % off-griddle cap) and the pick-up and set-down judged from the overhead
video (the scraper lifted, then lying in the taped square, released and still for 1 s). M is within 5 % of D (the weighed pieces) for a valid trial; otherwise the trial is
repeated. The two photos are overlaid with the 1 cm grid registered to the plate's corners: the cells holding a dark
particle 1 mm or larger are counted and kept as the record of where residue was left, and a line visible after but
not before, 1 cm or longer, is a new scratch in the steel: the gouge cap (0.5). The photos and weights are kept with
each trial, so anyone can recount.

**Nothing left on the griddle (the 1.0 condition, real trials).** In the simulator, nothing left means no piece on the
griddle (`FULL_RESIDUE` = 0 in `task.py`), and every piece weighs 2.50 g. In a real trial, nothing left means both:
(a) L under 0.40 g, under a sixth of one piece (0.7 % of 55.0 g, well inside the 5 % allowed for 0.75), read on the
0.01 g scale; and (b) no piece 1 mm or larger on the plate, guards or blade, checked by eye under the raking light
without wiping, any doubtful piece measured against the 6 in. steel rule, and confirmed on the after photo with the
1 cm grid. A thin smear (above) is not a piece. The other thresholds are the simulator's, applied to the weighed
shares.

**Reset (between trials, about 3 minutes).** Weigh the tray's gain (above), then empty the tray's contents into the
waste bin (used dough is not reused), wipe the tray with a damp microfiber cloth, dry it with a paper towel and seat
it. Lift anything left on the plate with the razor scraper, wipe the plate where the pieces were with a damp
microfiber cloth (water only), then dry it with a paper towel; the plate is ready when the raking-light photo shows no
dark particle 1 mm or larger and no smear, and the steel is dry to the touch of a dry paper towel. Wipe the scraper
(Protecting the robot's hands) and lay it in its outline (not in the taped square). The same griddle, template and scraper are used throughout;
the scraper is replaced if its edge has a nick 0.5 mm or deeper, the template if a hole's edge is torn. Then make the
next debris (above). A trial, its reset and the next debris take about 10 minutes.

| Item | Target | Tolerance | Source |
|---|---|---|---|
| Plate's near right corner | x 15.0, y -17.55 (stops) | 2 mm; lip square within 0.5 degree | simulator's layout; steel tape |
| Plate's top above the bench | 5.0 cm, level | 2 mm at each corner; 0.5 degree | simulator's layout; torpedo level |
| Scraper | in its outline at x 25.0, y -25.5 | 5 mm, 3 degrees | simulator's start pose |
| Set-down square | 12 cm tape square (outer edge) centered at x 25.0, y -40.0 | 5 mm | simulator's square; steel tape |
| Template | near edge along the lip, right edge against the right guard; squares 1.9 cm | squares 0.5 mm, centers 1 mm | simulator's piece layout; 6 in. steel rule |
| Room | 20 to 24 degrees C | | the trial runs at room temperature; thermometer |
| Plate before pressing | bare 430 stainless, clean, dry, unoiled | no particle 1 mm, no smear on the photo | Play-Doh sticks to clean, dry steel |
| Dough | black Play-Doh, a fresh sealed 4 oz can each trial | can lot code recorded | one product, fresh each trial |
| Each piece | 2.50 g, 1.9 cm square, about 4.8 mm thick | 0.05 g | simulator's piece; 0.01 g scale |
| All 22 pieces | 55.00 g | 0.30 g | simulator's debris mass |
| Press | base of a measuring cup with a full 1 L water bottle (1 kg) on it, 3 s, down to the template | | the template's thickness sets the piece's; the bottle sets the press |
| Contact check | by eye: every piece flat and fully in contact with the plate, no lifted edges | every piece | a lifted piece is remade |
| Start | within 10 minutes of the last piece pressed | | fresh dough |
| Finger covers | a fresh white nitrile glove on each finger | every trial; no dough on them after | Protecting the robot's hands |
| Video | overhead phone (plate, outline and square in frame) and a phone at plate height at the near left corner | whole trial | credit and hands-off judged from both |
| Nothing left (1.0) | L under 0.40 g and no piece 1 mm or larger; a thin smear not counted | | a sixth of a 2.50 g piece; steel rule and 1 cm grid photo |
| Mass balance | M = T + L + O vs D, the weighed pieces | 5 %, else repeat | 22 weighed pieces |
| Weighing | BOMATA 1 kg x 0.01 g scale, within 5 minutes of the trial's end; tray plus contents within its 1,000 g | 0.01 g (1 % of the debris is 0.55 g); session check 100 g within 0.03 g | listed specifications; 100 g calibration weight |

## The debris (`debriskit.py`)

- **22 separate pieces.** Each piece is 1.9 cm square and 4.8 mm thick (the template's hole), 2.50 g, 55.0 g in all
  (just under half a 4 oz can). They lie in four rows 3.5 cm apart (6, 5, 6 and 5 pieces, 3.5 cm apart along a row,
  alternate rows offset by half), the near row's centers 2.5 cm from the lip, 1.6 cm of bare steel between
  neighbors. Why 22 pieces of 2.50 g: under half of one can, in pieces large enough to weigh to 2 % on a 0.01 g
  scale and to press by hand in seconds, small enough that the 10.2 cm blade meets two or three at a time, and
  enough of them that a lane's leftovers matter (one piece is 4.5 % of the mass).
- **Real bodies, bonded.** Each piece is its own rigid body: a box collider with the same visible box and its real
  mass, lying on the plate and held by a breakable joint (a PhysX fixed joint from the plate to the middle of the
  piece's underside). Pieces are bonded to the plate only, not to one another. The physics solver breaks the joint,
  for good, the first time holding the piece takes more than its break force; from then on the piece is free.
  Nothing is drawn or moved by a script: the blade's collider has to meet a piece and push it hard enough to free it,
  and a piece falls into the tray only when it is pushed past the lip. The score reads only the pieces' poses.
- **Bond strength.** Each piece holds under its own weight and breaks free under the scraper's edge; in real trials
  the same press (a 1 L water bottle on the cup for 3 s) sets the bond, and qualification checks that the scraper
  frees each piece in one stroke. The scraper's edge driven under a piece's side lifts that side and peels the piece
  off, as a cook's scraper does. A piece freed by anything but the blade (a hand, the arm, the handle) is not
  credited, and a hand on the dough scores the trial 0.
- **Break force in PhysX.** PhysX tests a joint's break force against its own measure of the joint's load, not the
  push on the piece, so the task scales the bond it writes (`BREAK_CAL`). A broken bond is not restored by the scene's
  reset (one trial per run).
- **Rigid pieces.** Pieces are rigid in the sim; the score is by mass in both, so dough smeared or left on the steel
  counts as left on the griddle.
- **Contact settings.** Pieces are 4.8 mm thick (the blade is 1.5 mm, moving 4 cm/s: 0.2 mm per 240 Hz physics step,
  so nothing passes through); contacts are generated from 2 mm out and rest at zero gap; 16 position iterations; a
  piece pinched into a collider is pushed out no faster than 0.3 m/s. A freed piece stops when the blade stops, as
  dough does.
- **Friction.** The plate is clean, dry, bare steel, as in a real trial: 0.7 static and 0.6 sliding for the blade on
  the plate (dry steel on steel; common engineering tables give 0.5 to 0.8) and for a freed piece of dough on it
  (combined by "min"). A freed piece drags on the plate and stops when the blade stops.
- **The layout is fixed.** The pieces are not in the task's object list, so the simulator's object jitter (which
  moves each free object up to 3 cm on its own) leaves them where the template puts them; the scene's reset puts them
  back where they started. The scraper is still jittered.
- **Why the strokes overlap.** Pushed pieces spread sideways and spill round the blade's ends; a lane's leftovers lie
  in the next lane's path, so the demo's half-blade overlaps and repeated passes pick them up.

## Runtime and seeds

- Demo: a 111 s plan (2,220 control steps), run with `--max-steps 2400`. The recorded run scored 1.0 at
  step 2,169 (the trial ends once the set-down has held for 1 s, before the plan's final wait): every piece freed
  by the blade and pushed by it over the lip into the tray (all 22 credited), no hand on the dough, none left on the
  griddle, and the scraper in the taped square.
- Model run limit: 2,400 steps (2 minutes at 20 Hz). Dream-1: 0 in 2,400 steps on this task.py.
- The demo: the right hand takes the scraper by the handle from above, fingers closing across the handle (yaw 0: along
  world y), lifts it level and carries it over the right splash guard (tilted, its edge would hang below the guard's
  top), then tilts it over the plate so the blade meets the plate at 40 degrees with the edge toward the robot. The
  tilted hand is turned 6 degrees about its approach, undoing the turn the handle takes in the fingers when they
  close, so the edge lies flat on the plate. Each stroke sets the edge on the plate at x 35.0 (6 cm beyond the
  pieces; the hand lags about 1.5 cm, so the edge lands over 4 cm clear of them: an edge that lands on a bonded piece
  presses it down and slides over it), aimed 10 mm into the plate (the plate stops it; a hand holding the scraper
  sits 3.5 to 6.5 mm above where it is sent, so a shallower aim lets the edge lift off mid-stroke), draws it toward
  the robot at 4 cm/s to x 13.5 (1.5 cm past the lip, over the tray), and lifts it 3 cm in 0.6 s, slowly, so what
  rides on the blade slides off into the tray. Three lanes 5 cm apart (y -7.0, -2.0 and 3.0; the blade covers y -12.1
  to 8.1, every piece inside), three times over: the first pass breaks the pieces free and clears most of them, and
  the later passes pick up the pieces that spill round the blade's ends. A cleanup stroke from x 30.0 along each
  splash guard (centered at y -12.0 and 8.0, the blade's ends 4.5 mm inside the guards) collects the pieces pushed
  past the outer lanes' ends. Then the scraper is leveled over the tray, carried back over the guard to the taped
  square, laid down in it and let go.
- Seeds: none. The layout is fixed (`object_jitter=0`) and the same every trial.

## Real-world build

Everything is off the shelf and chosen to be low-cost and widely sold; the pages below give each item's published
specifications.
- Griddle: use an Assure Parts 177AOGT1227 add-on 2-burner griddle top, 27 x 12-1/4 x 4 in., 22-3/8 x 12-1/4 in.
  cooking surface of 7-gauge type 430 stainless, 1-1/2 in. rear and side edges, adjustable plate height, 3 x 12-1/4 x
  1-1/4 in. grease tray across the open front, from WebstaurantStore
  (https://www.webstaurantstore.com/12-14-x-27-x-4-addon-2-burner-griddle-top/177AOGT1227.html). It stands on the
  bench on its frame with the plate's top 5.0 cm above the bench, clamped with two C-clamps on its frame, and is never
  heated. Its grease tray (about 0.5 kg; weighed at setup) with the debris is
  well within the scale's 1,000 g.
- Scraper: use a Dexter-Russell Basics P94850 4 in. griddle scraper: 4 in. stiff stainless blade, 4-1/2 in.
  polypropylene handle, 8-1/2 in. overall, 4.8 oz (136 g)
  (https://www.wasserstrom.com/restaurant-supplies-equipment/dexter-russell-p94850-basics®-4-griddle-scraper-122621).
  The simulator's scraper is this tool, at its size and mass; the handle (3.0 x 2.4 cm in the simulator) is within
  the gripper's opening.
- Debris: use black Play-Doh modeling compound (Hasbro), sealed 4 oz (113 g) cans, one opened for each trial.
- Template: use Elmer's 3/16 in. white foam board, one US Letter sheet printed at 100 %, an Elmer's glue stick, and an
  X-ACTO #1 knife to cut the squares.
- Press: use a measuring cup (its flat base) and Reynolds Kitchen parchment paper.
- Scale: use a BOMATA 0.01 g / 1 kg compact digital scale (model BS2000-001B as listed; 1,000 g by 0.01 g, stainless
  platform, tare, USB rechargeable), a common low-cost 0.01 g scale whose capacity covers the tray with its contents.
  It weighs the pieces, the tray and the fractions. At the start of each session
  it is checked, after a 10-minute warm-up and away from drafts, against a 100 g calibration weight (100.00 g within
  0.03 g), and recalibrated by its manual if it reads outside that.
- Finger covers: use Kimtech Pure G3 white nitrile gloves (Kimberly-Clark), size Large, held with ScotchBlue 2090
  painter's tape. The same box serves the operator. White, so dough on a cover shows.
- Cleaning: use microfiber cloths and water for the plate, tray, scraper and robot; paper towels to dry.
- Room: use a ThermoPro TP49 digital thermometer and hygrometer on the bench beside the griddle.
- Overhead camera: use a phone on a NEEWER X6 overhead phone stand, 26.5 to 80 cm high, clamped to the bench
  (https://neewer.com/collections/all-product/products/neewer-entry-level-version-x6-phone-stand-66608636).
- Side camera: use a second phone on a small tabletop phone tripod.
- Set-down square: ScotchBlue 2090 painter's tape, 3/4 in.
- Raking light: use an IKEA TERTIAL clamp work lamp with an 800-lumen SOLHETTA LED bulb
  (https://www.ikea.com/ca/en/catalog/products/20355434/).
- Razor scraper: use a Red Devil 3010, 1 in. blade
  (https://www.dillonsupply.com/Catalog/Products/Hand-Tools/Scrapers-Spreaders-And-Trowels/Hand-Scrapers/red-devil-3010).
- Rule and level: use an Empire 27302 6 in. steel rule (https://www.toolnut.com/products/empire-27302-6-steel-rule) and a
  Stanley 0-42-294 torpedo level (https://www.stanleytools.co.uk/product/0-42-294/stanley-torpedo-level-200mm).
- Also: use a steel tape, weighing boats, white plastic catch sheets, masking tape, two wooden stop blocks and two
  C-clamps.

## Files

- `task.py`: the scene, the debris layout, `score()`, `success()` and the scripted `demo()`.
- `debriskit.py`: the task's helper module that spawns the debris pieces as rigid bodies, bonds them to the plate
  with breakable joints, and reads their poses.
- `scrapekit.py`: the task's helper module for the blade-angle rule (the gouge cap); its soil model is not used.
- `yamkit.py`: the task's helper module that spawns the modeled props (griddle plate, scraper) and reports each arm
  link's pose to the scorer.
- `guardkit.py`: the task's helper module for contact between the grippers and the scraper, and for the exact gap
  between two boxes.
- `yam_links.npz`: the YAM arm's collision meshes (from the arm's description, upper arm to fingertips) as surface
  points about 0.5 mm apart, for the check that the robot never touches the dough.
- `test_task.py`: offline checks of the scorer, the debris layout and the demo plan.

## Sources and licenses

- Arm collision geometry: the I2RT Robotics YAM model ([i2rt-robotics/i2rt](https://github.com/i2rt-robotics/i2rt)), MIT License, Copyright (c) I2RT Robotics; the copy used here is vendored in yam-isaac-lab-arena (commit 120c3c8). `yam_links.npz` holds surface points sampled from its link collision meshes; the MIT license text is in `LICENSE.i2rt` in this folder.
