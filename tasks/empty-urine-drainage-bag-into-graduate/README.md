# Empty a urinary drainage bag into a graduate, spout never touching it, then close the valve and clip the tube back

A Bard 154002 2000 mL urinary drainage bag hangs from a catheter-bag floor holder beside the bed. It holds 600 mL of
clear tap water standing in for urine. The robot sets a 1000 mL graduate under the drain, takes the drain tube out of
its clip, aims the spout into the graduate's mouth without touching it, opens the twist valve and lets the bag empty,
following the stream as its arc shortens, then closes the valve and clips the tube back.

## Why it matters

**Who does it, and how many.** Emptying the drainage bag into a graduate is how urine output is measured for a patient
with an indwelling (Foley) catheter, and intake and output are on the task lists of two large occupations:

| Occupation (O*NET) | US jobs (O*NET OnLine, 2025) | O*NET task line (verbatim) |
|---|---|---|
| Nursing Assistants (31-1131.00) | 1,505,900 | "Measure and record food and liquid intake or urinary and fecal output, reporting changes to medical or nursing staff." |
| Licensed Practical and Licensed Vocational Nurses (29-2061.00) | 666,900 | "Record food and fluid intake and output." |

Together that is about 2.2 million US workers. Registered nurses do it too, but it is not on their O*NET list, so they
are not counted. Home health and personal care aides empty leg and night bags at home, but their O*NET task lists say
only "Assist patients with toileting and incontinent care", so they are not counted either.

**How often.**
- Between 15 % and 25 % of hospitalized patients may receive a short-term indwelling urinary catheter (CDC, CAUTI
  guideline background); about 5 % of US long-term-care residents have one at any time.
- Each bag is emptied at least once a shift and whenever it is half full. One US academic hospital's nursing
  guideline (2025; done by "Registered Nurse (RN), or Licensed Practical Nurse (LPN), or Nursing Assistant (NA), or
  Patient Care Technician (PCT)"): "Empty drainage bag, so it remains less than half full. Empty bag at a minimum each
  shift." The open nursing-skills textbook (WTCS / Open RN) gives half to two-thirds full.
- So every catheterized patient's bag is emptied at least 2-3 times a day (8 or 12 h shifts), every day, by the aide or nurse on shift.

**Why it is core, not ancillary.** It is the measurement that the O*NET lines above describe (urine output), and the
step most tied to infection control. It is a graded skill in nurse-aide competency testing: the Arizona skills
checklist "Catheter Care - Emptying a Urinary Drainage Bag" scores, in this order, "Places a barrier on the floor under
the drainage bag", "Places the graduate on the previously placed barrier", "Opens the drain to allow the urine to flow
into the graduate", "Completely empties drainage bag", "Avoids touching the graduate with any part of the tubing",
"Closes the drain" and "Replaces drain in holder". The CDC's catheter-associated UTI (CAUTI) guideline (2009),
recommendation III.B.3 (Category IB, kept in the 2022 SHEA/IDSA/APIC update), sets the method: "Empty the collecting bag
regularly using a separate, clean collecting container for each patient; avoid splashing, and prevent contact of the
drainage spigot with the nonsterile collecting container." The same guideline keeps the bag below the bladder and off
the floor and the system closed; CDC estimates that up to 380,000 CAUTIs and 9,000 related deaths a year could be
prevented.

**Why it is hard for a robot.** The spout hangs on soft tubing. One hand has to hold it 4-5 cm above an 11 cm mouth
without touching the rim, while the other twists a 22 mm valve collar on that same tube. The stream leaves the full bag
at 1.6 m/s and throws forward, and as the bag empties it slows and its arc shortens, so the aiming hand has to follow it.
A picture of the end state does not show any of this.

## The scene and how this matches the real job

The real items (each is in the sim at its real size; see Real-world build):
- **The bag:** Bard (BD) 154002 urinary drainage bag, 2000 mL vinyl, anti-reflux chamber, single-hook hanger, 48 in
  drainage tube, drain port "Twist Valve". Its 9 mm clear drain tube leaves the middle of the bag's bottom, runs out
  8 cm toward the robot and folds up 11 cm in front of the bag; the 22 mm blue twist-valve collar at its end sits
  closed in the holder clip (a tab out from the bag's front) with the spout end pointing up. A quarter turn opens it.
- **The hanging height:** the bag hangs from a 500 mm catheter-bag floor holder, hook 48 cm up, bag bottom 21 cm up:
  below the bladder and off the floor, as the CDC asks, at the height of a bed-frame hook. The station table stands in
  for the floor beside the bed.
- **The fill:** 600 mL, 30 % of the bag's 2000 mL and about 8 h of an adult's urine output: a bag due to be emptied.
- **The liquid:** clear tap water, in the sim and in the real trial: hygienic and the same every trial, and it drains
  like urine (urine is 95 % water, density 1.005-1.030 g/mL), so the flow, the stream's throw and the volumes are those
  of a real bag. The sim draws it light and slightly blue-tinted.
- **The graduate:** Medline DYND80417 graduated triangular intake/output container, 1000 mL (32 oz), with a handle,
  standing empty beside the underpad on the robot's right.
- **The barrier:** a blue disposable underpad (30 x 40 cm) on the floor under the bag.

| Step of the real procedure (Arizona checklist, CDC III.B.3) | What the robot does | What the scorer checks |
|---|---|---|
| Place a barrier on the floor under the bag | the underpad is already down (setup) | spill landing on it counts as spilled |
| Place the graduate on the barrier, under the drain | carries the graduate by its handle onto the underpad (it is not pre-placed) | graduate upright at the end (within 10 degrees) |
| Take the drain out of its holder | one hand pinches the tube and pulls the spout down out of its clip | tube back in clip at the end (1.0 only) |
| Hold the spout over the graduate, not touching it | aims the spout into the mouth, 4-5 cm above the rim | spout within 1 mm of the graduate at any time caps the trial at 0.25 |
| Open the drain | the other hand twists the valve collar a quarter turn open | flow follows the valve's opening |
| Empty the bag completely, no splashing | aiming hand follows the stream as it slows and its arc shortens | at least 570 mL (95 %) in the graduate; at most 5 mL spilled for 1.0, over 20 mL scores 0 |
| Close the drain | twists the collar back to its closed stop | valve within 10 degrees of closed (0.75 and up) |
| Replace the drain in its holder | swings the tube up and pushes the spout into its clip | tube in clip, both hands clear, everything still for 1 s (1.0) |

Not in the task: gloves, hand hygiene, wiping the spout with alcohol, reading the level at eye level and recording it
(the scale gives the exact volume instead), and pouring the urine away.

Real failures stay possible: the spout touches the rim (the CDC violation), the stream lands beyond the mouth (or, as
it shortens, short of it), the tube is dropped and hangs down with the valve open (draining onto the floor), the
graduate is knocked over, the valve is left open, the tube is left out of its clip.

## Run a trial

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/empty-urine-drainage-bag-into-graduate/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Empty the drainage bag into the graduate without letting the drain spout touch the graduate, then close the drain and put the tube back in its holder." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/drainbag-demo
```

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.
Video: `inspect-robots video logs/drainbag-demo/*.json`. The scorer (`task_score`) reports the score each step; the
trial's score is the one at its end. `DRAINBAG_DEBUG=<n>` prints the drain's state every n control steps (bag,
graduate, spilled, valve, tube angles, clipped, spout height, touch). Offline checks: `python3 test_task.py` in this
folder.

## Scoring

"In the graduate" is the water that reached it. "Spilled" is what landed anywhere else (underpad, table). A touch is the
spout (the valve collar and the last 2 cm of tube above it) within 1 mm of the graduate's wall, rim or floor at any
time.

| Score | Condition |
|---|---|
| 0 | more than 20 mL spilled (a puddle on the floor), or nothing done |
| 1.0 | at least 570 mL in the graduate (95 % of the 600 mL), at most 5 mL spilled, the valve closed (within 10 degrees of its closed stop), the tube back in its clip, the graduate upright (within 10 degrees of vertical), both hands clear (every tool point and pad tip 1 cm from the spout, the tube and the graduate) and everything still, held 1 s |
| 0.75 | at least 570 mL in the graduate, the valve closed and the graduate upright |
| 0.5 | at least 570 mL otherwise (valve left open, graduate tipped after), or at least 300 mL with the valve closed and the graduate upright |
| 0.25 | at least 100 mL in the graduate |
| cap 0.25 | a touch at any time caps the trial at 0.25 |

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Run limit: 2,400 steps (2 minutes at 20 Hz). Every quantity is read in the
graduate's own frame, so the score is the same wherever on the underpad, and turned whichever way, the graduate is
set down (`test_task.py` checks four turns and five positions).

## Real-world build

A low-cost setup of named, widely available products; nothing has to be made, and one set serves the whole session.

- Use a Bard (BD) 154002 urinary drainage bag, 2000 mL, with its anti-reflux chamber, twist-valve drain tube and
  holder clip (a stock hospital-supply item; one bag lasts the session, as it only ever holds tap water).
- Use a catheter-bag floor holder for 2 L bags, 500 mm tall (e.g. the Rosscare or Essential Aids urine catheter bag
  floor holder), its base held to the table with two strips of 3M duct tape. It puts the hook at the sim's 48 cm.
- Use a Medline DYND80417 1000 mL (32 oz) graduated triangular intake/output container.
- Use a Medline disposable underpad (23 x 36 in), folded to 30 x 40 cm.
- Use clear tap water, 600 mL per trial (no coloring: it stays hygienic and identical trial to trial), and a small
  plastic kitchen funnel (e.g. the smallest of an OXO Good Grips 3-piece funnel set) to fill the bag.
- Use a basic digital kitchen scale reading 1 g to 5 kg (e.g. Etekcity EK6015 or OXO Good Grips 11 lb food scale). The
  fill is 600 g within 2 g and the spill limits are 5 mL and 20 mL, so 1 g is enough; check it once against 500 mL of
  water measured in the graduate. The scale also measures the fill, so no measuring jug is needed.
- Use a Crayola Washable Broad Line marker (blue) and white facial tissues (Kleenex) for the touch check.
- Use ScotchBlue 2090 painter's tape and a 30 cm steel ruler (e.g. Westcott) for the setup marks, and a phone on a
  tripod mount for the side video.
- Gripper protection: none needed. The hands touch only the outside of the drain tube, the valve collar and the
  graduate's dry handle, and the stream leaves the spout away from the fingers; a stray drop on a pad is wiped off
  with a paper towel during the reset.
- Safety: tap water only; nothing sharp, hot or heavy (the full bag weighs 0.67 kg).

Times: first setup about 15 minutes (marks, holder, weighing the underpad); each reset about 4 minutes (drain and
refill the bag 2 minutes, rehang, close and clip 30 s, empty, dry and re-mark the graduate 1 minute, new underpad
30 s); the measurement 3 minutes.

## Real-world evaluation (reproducible protocol)

The products and tools are those listed under Real-world build.

**Setup (marks and tolerances; about 15 minutes the first time)**
1. Tape the holder's base outline so its hook sits at x 42, y -4 cm from the midpoint between the arm bases (within
   5 mm), 48 cm above the table (within 5 mm; check with the ruler).
2. Pre-weigh the dry underpad (1 g) and tape its corners flat centered at x 30, y 2 cm (within 1 cm).
3. Tare the dry graduate on the scale and weigh 600 g of tap water into it (within 2 g). Pour it through the funnel
   into the bag's inlet tube held above the bag, then dry and tare the graduate again (write down its empty mass).
4. Hang the bag on the hook, front toward the robot. Twist the valve to its closed stop and push the spout into the
   holder clip.
5. Color a 1 cm band round the graduate's rim, inside and out, with the washable marker; let it dry 1 minute. Stand
   the graduate on its tape circle at x 23, y -24.5 cm (within 5 mm), handle toward the robot's right.
6. Check the trial parts against the model dimensions under Fidelity with the ruler (bag, tube and graduate within
   5 mm; the valve's quarter turn by eye) and weigh the empty graduate (90 g within 10 g).

**Measurement (3 minutes, five readings)**
1. Graduate: weigh it; net mass in g = mL in the graduate.
2. Spilled: weigh the underpad; its gain in g, plus any drops on the table wiped up with a pre-weighed tissue, = mL
   spilled.
3. Touch: wipe the spout end and the collar with a white tissue. Any blue on it is a touch (photo of the tissue).
4. Valve and clip: photo of the collar at its closed stop and the spout in the clip; no drip from the spout for 30 s
   (video).
5. Graduate upright and both hands clear at the end: the side video.

**Reset (about 4 minutes):** unhook the bag and drain what is left into the sink; refill it as in step 3; rehang, close and clip it;
empty, dry and tare the graduate, renew the marker band, put it back on its circle; lay a fresh pre-weighed underpad
(or the same one once it is dry and back to its start mass).

**Tolerances**

| Item | Value | Tolerance |
|---|---|---|
| Hook position and height | x 42, y -4 cm; 48 cm up | 5 mm |
| Bag fill | 600 g (600 mL) | 2 g |
| Graduate start | x 23, y -24.5 cm, handle right | 5 mm |
| Underpad | 30 x 40 cm at x 30, y 2 | 1 cm |
| Scale | 1 g reading | 1 g |
| Full drain | 570 mL in the graduate | scale |
| Spill | 5 mL full credit, 20 mL limit | scale |
| Touch | 1 mm in sim; any marker transfer in the real trial | tissue |
| Valve closed | within 10 degrees of its closed stop; no drip in 30 s | photo, video |
| Graduate upright | within 10 degrees of vertical | video |

## Design notes

- **The holder and the bag.** One articulation fixed to the table at the holder's base. The bag hangs on a hinge at
  the hook: a pull on the tube swings it, as on a real holder; the holder's frame stops it swinging back.
- **The drain tube.** A sideways hinge at the outlet (it can be led a little to either side; a straightening law
  stands in for the tube's bending stiffness), an 8 cm run and an 11 cm tube on hinges in the bag's plane, and the
  22 mm valve collar turning on the tube's axis. The tube hangs straight down from the outlet when nothing holds it:
  dropped with the valve open, it drains onto the floor.
- **The holder clip and the valve.** Click laws: within 0.15 rad (8.6 degrees) of the folded angles both tube hinges
  are pulled in (0.5 N at the spout to pull it out); the collar has stops at closed and at a quarter turn open
  and clicks into the nearer one within 0.15 rad. It is closed within 10 degrees of its closed stop and passes flow in
  proportion beyond that.
- **The drain.** A lumped flow model, as in our IV-hub task: no particles, but every quantity follows the physics a
  nurse sees. The bag is a pillow: welded flat along its bottom seam, its section grows as sqrt(height) over the
  bottom 8 cm, then stays 18 x 4.5 cm, so 600 mL stands 10 cm deep. It drains as an orifice, Q = Cd A sqrt(2 g h)
  with Cd 0.61 and the 6 mm bore, h from the bag's level down to the spout, scaled by how far the valve is open. At
  the demo's pose the full bag starts at 27 mL/s and empties in 28 s fully open. Folded up in its clip, the
  spout is 3.1 cm above the full bag's level, so nothing flows even if the valve is opened there.
- **The stream.** It leaves the spout along its axis at sqrt(2 g h) and falls ballistically. It is caught if it
  crosses the rim's plane within the mouth (2 mm in from the rim), or if the spout is already inside the mouth below
  the rim; otherwise it lands on the floor and is spilled. As h falls the stream slows and its arc shortens. The
  graduate is a liquidkit vessel; tipped, it spills over its rim.
- **The drawing** (never part of the score). The water is drawn as clear water: light and slightly blue-tinted,
  semi-transparent in the bag and the graduate (the stream, a few pixels wide on the camera, is drawn solid). In the
  bag it is drawn as slices as thick as the filled bag at each height (full at the start, falling as it drains); in
  the graduate as a surface at the exact depth for its volume, so it rises smoothly; the stream as one smooth tube
  mesh from just inside the spout along its arc into the mouth and, where the fast jet meets the graduate's inside
  wall before the surface, down that wall to the surface (the arc is never drawn through the wall). The stream is
  drawn 8 mm thick fully open (a little wider than the 6 mm bore, so it shows on the 320 px camera), thinner while the
  valve is only partly open. The drawing is created when the scene is built and drawn in its start state on every
  reset, so the first frame already shows the full bag.
- **The demo** (open loop, 99 s). Right hand: takes the graduate's handle from above, carries it onto the underpad
  under the drain and slides off the handle's arm. Left hand (the aiming hand; the right one cannot hold the tube
  fingers-across at the clip): comes in from the robot's left with its fingers pointing right and pinches the tube
  8.5 cm up from the collar (fingers 2 mm inside each side, 10 N, inside the measured 1-20 mm grip envelope),
  pulls the spout down out of the clip, raises the run to 30 degrees above horizontal and swings the spout to 60
  degrees from vertical, 4.6 cm above the rim. Right hand (the valve hand): pinches the collar from the robot's right
  and twists it a quarter turn open, lets go and stands by 7 cm off. The aiming hand then follows the stream: six
  waypoints, from the scorer's own drain model run ahead (planned at 75 % open), move the spout so the shortening
  stream keeps landing on the same spot in the mouth (offline: within 45 mm of the mouth's center from 40 % to 100 %
  open). After 47 s the valve hand takes the collar again and twists it closed; the aiming hand swings the tube back
  up and pushes the spout into its clip; both hands let go and back away.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes).
- Scripted demo: the plan lasts 99 s; it scored 1.0 in 1,899 steps (600 mL in the graduate, 0 mL spilled, no
  touch, valve closed, tube clipped, hands clear). The demo is open loop.
- No randomness: `-E object_jitter=0` keeps the graduate on its mark; the bag starts with 600 mL every trial.

## Files

- `task.py`: the scene, the drain model, the scorer and the scripted demo.
- `yamkit.py`: spawns the holder, bag, tube and valve as one articulation (its click laws for the clip and the valve)
  and the graduate as a rigid prop, and gives the scorer the grippers and joint states.
- `liquidkit.py`: the graduate's liquid (volume, surface, spill over the rim when tipped).
- `guardkit.py`: checks that both hands are clear of the spout, the tube and the graduate.
- `test_task.py`: the offline checks.

## Fidelity

- The water is a lumped flow model, as in our accepted IV-hub task: the bag's volume drains through the spout as an
  orifice (Torricelli) with the real head, the stream follows the ballistic arc that head gives it, and the graduate
  (liquidkit) holds the volume, shows its level and spills over its rim if tipped. Volumes are conserved to the
  microliter and the timing (27 mL/s at the start, slowing as the bag empties) is the real one; what it leaves out is
  splashing, dripping and surface tension.
- The vinyl bag is a rigid pillow on its hook hinge, and its mass stays at full (0.67 kg) as it drains.
- The flexible PVC tube is two rigid links on hinges. Folded, it stands 5.75 cm off the bag's front on the holder tab
  (room for the gripper's 10 cm palm beside the bag); a real tube folds closer. The holder clip and the valve are
  click laws: the spout pulls out of the clip at 0.5 N, and the collar clicks into its stops within 0.15 rad.
- The triangular graduate is modeled round with the same mouth area. Masses of the tube parts and the graduate are
  raised slightly for the solver.
- Spout contact is a geometric check (1 mm).
- Model dimensions (the makers list capacities only): the bag 18 x 4.5 x 25 cm with an 8 cm bottom bulge; the drain
  tube and collar 9 mm, 8 + 11 cm, 22 mm, 6 mm bore; the valve a quarter turn, clockwise seen from the spout; the
  graduate 9.0 / 11.2 cm across, 13.5 cm deep, 90 g. Setup step 6 checks the trial parts against them.

## Sources

- O*NET 29-2061.00 Licensed Practical and Licensed Vocational Nurses, task list and 2025 employment:
  https://www.onetonline.org/link/summary/29-2061.00 ; O*NET 31-1121.00 / 31-1122.00 (home health and personal care
  aides, not counted): https://www.onetonline.org/link/details/31-1121.00
- CDC, CAUTI guideline background (15-25 % of hospitalized patients; about 5 % of long-term-care residents; up to
  380,000 infections and 9,000 deaths a year preventable): https://www.cdc.gov/infection-control/hcp/cauti/background.html
- A US academic medical center's Nursing Services Guideline, Indwelling Foley Catheter Care (2025):
  https://www.utoledo.edu/policies/utmc/nursing/guidelines/general/pdfs/Guideline%20General%20-%20Indwelling%20Foley%20Catheter%20Care%20with%20Nurse%20Driven%20Protocol.pdf
- Open RN / WTCS Nursing Skills, 21.2 Basic Concepts (empty at half to two-thirds full):
  https://wtcs.pressbooks.pub/nursingskills/chapter/21-2-basic-concepts/
- Arizona nurse-aide skills checklist, Catheter Care - Emptying a Urinary Drainage Bag:
  https://src.bryanuniversity.edu/wp-content/uploads/Catheter-Care_-Emptying-a-Urinary-Drainage-Bag_AZ-Skills-Checklist-.pdf

- O*NET 31-1131.00 Nursing Assistants, task list and 2025 employment: https://www.onetonline.org/link/summary/31-1131.00 ;
  BLS OEWS 31-1131: https://www.bls.gov/oes/current/oes311131.htm
- CDC, Guideline for Prevention of Catheter-Associated Urinary Tract Infections (2009), recommendation III.B.3:
  https://www.cdc.gov/infectioncontrol/guidelines/cauti/recommendations.html
- Bard 154002 urinary drainage bag, 2000 mL: https://www.bd.com/en-us/products-and-solutions/products/product-page.154002 ;
  twist-valve drain tube, single hook: https://www.wholesalepoint.com/products/bard-154002-cs.html
- Medline DYND80417 / DYND80419 graduated triangular container, 1000 mL:
  https://www.bettymills.com/medline-graduated-triangular-intake-output-container-black-1000-ml-dynd80419-meddynd80419z
- Catheter bag floor holder, 500 mm tall, 210 mm deep, 0.3 kg: https://rosscare.co.uk/products/urine-catheter-bag-floor-holder ;
  https://www.essentialaids.com/bedroom-aids/bedroom-aids-bed-pans-portable-urinals/urine-catheter-bag-floor-holder.html
