# Check a lawn tractor engine's oil (Kohler KT725) with its dipstick, wiping and re-reading it, and mark the level on the check card

A lawn tractor engine's oil check, on a low-cost rig built round the engine's own oil parts: the genuine
yellow-handled dipstick and oil fill tube of a Kohler 7000 Series KT725 V-twin (its manual, p.42, names "lawn
tractors, mowers" among its applications), the tube standing in a quart can of oil at the front corner of a black
plywood box that takes the place of the engine's blower housing. The tube's mouth, the housing's footprint and top and
the tube's corner are where Kohler's dimension drawing puts them on the engine, and the same rig is built for the real
trials. A folded blue shop rag lies in front of it, next to an oil check card with ADD, OK and OVER pockets and a white
marker puck. "Check the oil level on the engine
with the dipstick, mark it on the check card with the puck, then put everything back." means the procedure Kohler's
manual gives: pull the dipstick out by its handle, wipe the oil off the blade, push it back in until the cap seats,
pull it again and hold it still to read the level, push it back in until it seats, put the rag down, and record what
the blade showed. The level changes from trial to trial (below ADD, between the marks, or above FULL), so the puck
lands in the right pocket only if the blade was read. The tempting wrong answer is to read the first pull, which shows
a false smeared level past FULL from oil splashed up the tube.

## Spec

### Scene (cm from the midpoint between the arm bases, x forward, y to the robot's left, table top z = 0)

The scene is the low-cost real build (Build B under Real-world build). Its sizes and places come from Kohler Service
Manual 32 690 03 Rev. H (KT610-KT745), dimension drawing on p.5 ("the drawing"), turned so the drawing's oil-filter
side faces the robot and its "OIL FILL & DIPSTICK (YELLOW)" corner is the robot's near right corner.

| Part | What it is | Where |
|---|---|---|
| Base | Plywood, 47 x 50 x 0.8 cm, unpainted, fixed to the table; it stands for the engine's mounting surface | x 27.7 to 74.7, y -18.8 to 31.2 |
| Housing box | 12 mm plywood painted plain black, 41 x 41 cm, 30 cm tall (the engine's blower housing: its footprint and flat top from the drawing), closed on top, standing on the base. Its near right corner, where the tube stands, is cut off as two flat faces 15.2 cm wide, from 20 cm along the front and the right side to a point 6 cm in from both; they run from the top down to 20 cm above the base and are open below. The other three corners are cut at 45 degrees, 13 cm along each side (12 cm at the far right corner). Colliders are boxes inside it | x 30.7 to 71.7, y -14.5 to 26.5 |
| Oil can | Unlined steel 1 quart paint can, 10.8 cm across, 12.4 cm tall, with its lid, standing on the base in the open corner under the tube; the tube goes down through a hole in the lid. Its colliders leave a channel round the blade | middle x 36, y -12 |
| Oil fill tube | Kohler 32 123 14-S: black tube 2.1 cm across, straight up, with a 3.6 cm collar over its top 5 cm that the cap seats on; its mouth (the collar's top) 27.5 cm above the base, the engine's height from the drawing. A steel tab holds it to the corner face 3.5 cm behind it | mouth at x 36, y -12, z 28.3 |
| Dipstick | Kohler 32 038 14-S, a prismatic link sliding along the tube: steel blade 1.0 x 0.3 x 24.5 cm with black ADD and FULL marks 0.8 and 2.0 cm from the tip and a crosshatch between them, a yellow ribbed cap 3.2 cm across and 1 cm thick, and a yellow T-handle (a round-cornered tab) 3.5 cm wide, 1.8 cm thick, 3.5 cm tall. 29 cm overall, 50 g, with its weight: let go off its seat, it slides back down onto it. Seated, its tip is 3 cm above the base, in the can's oil, and the handle's top 32.8 cm above the table. The blade's flat faces look toward and away from the robot | in the tube |
| Shop rag | Folded blue shop towel pad, 4 x 4 x 2.5 cm, 40 g, loose | centered x 20, y 0 |
| Oil check card | Dark gray tray 6 x 16 x 1 cm, fixed: three pockets 4.4 cm square and 5 mm deep. Each floor carries a white symbol (2.8 cm, 6 mm strokes) so the pockets read without color: ADD amber with a minus sign, OK green with a check mark, OVER red with a plus sign | centered x 20, y 13; pockets at y 18.2 (ADD), 13 (OK), 7.8 (OVER) |
| Marker puck | White puck, 3 cm across, 2 cm tall, 15 g, loose | x 14, y 22 |

Kohler's drawing gives the engine's outline, heights and the fill tube's place. The blade's 24.5 cm runs from the
collar's top at the drawing's mouth height to 3 cm above the base, reaching into the oil, and its ADD and FULL marks
are 0.8 and 2.0 cm from the tip; the genuine dipstick's length and marks are recorded on arrival (see Real-world build). The blade carries a drawn oil film (visual
only): at the start, a ragged smear over its lower 6 cm (the false first reading); the smear disappears wherever the
rag passes; after a wipe and a full seat, the blade shows a clean oil band from the tip up to the trial's level:

| Level (`OIL_CASE`) | Oil band from the tip | Reads | Pocket |
|---|---|---|---|
| `low` | 0.4 cm, 4 mm under the ADD mark | below ADD | ADD (minus sign) |
| `ok` | 1.4 cm, 6 mm from either mark | between the marks | OK (check mark) |
| `over` | 2.7 cm, 7 mm above the FULL mark | above FULL | OVER (plus sign) |

Each level sits 4 mm or more from a mark, so it reads the same by eye in any camera that sees the blade. The
seeded reset picks the level: every free object is jittered by up to 3 cm, and the rag's offset from its place in
whole millimeters (x + y), modulo 3, picks low, ok or over (the rag's offset is noise, not a cue a camera could use).
With no jitter (`object_jitter=0`) the level is between the marks, and the environment variable `OIL_CASE` sets it
for a run. The band appears while the blade is inside the tube (after the full seat), so it never pops in on camera.

### Instruction

"Check the oil level on the engine with the dipstick, mark it on the check card with the puck, then put everything
back." The wipe-and-reread rule, the full seat and the closing seat are not stated. The instruction names the card
and the puck because a robot cannot speak its reading; the card stands in for writing the level down.

### Rule and why

Kohler's manual for this engine, p.41, "CHECK OIL LEVEL": "Ensure engine is cool. Clean oil fill/dipstick areas of
any debris. 1. Remove dipstick; wipe oil off. 2. Reinsert dipstick into tube; press completely down. 3. Remove
dipstick; check oil level. Level should be at top of indicator on dipstick. 4. If oil is low on indicator, add oil up
to top of indicator mark. 5. Reinstall and secure dipstick." and "never run engine with oil level below or above
operating range indicator on dipstick." Oil splashed up the fill tube while the engine ran coats the blade above the
true level, so a first-pull reading overstates the oil and a low engine gets run. A dipstick pushed only part way in
before the second pull reads low. It goes back in until its cap seats, so dirt cannot get into the crankcase, and the
rag goes back down. The level found decides what comes next: top up a low engine, mow, or drain an overfilled one.

The dipstick is handled by its handle: the blade carries the oil, and a hand on the blade smears the band it is about
to read. The wipe takes the oil off the whole oiled length of the blade, from the tip up past the highest level and
the splash, in one or more strokes, not a dab in one spot.

Category: Societal relatability. Checking the oil before mowing is the first maintenance step in a lawn tractor's
manuals, and the owner does it. In a 2018 Harris Poll of over 2,000 US adults for the Outdoor Power Equipment
Institute, 63% owned outdoor power equipment.

Reach: the dipstick is drawn straight up from a cap 28-29 cm above the table until its tip clears the mouth, which
takes the hand to about 59 cm. Offline IK on the YAM's URDF: a level grip (the gripper horizontal) reaches a vertical
line from 20 cm to 59 cm above the table at the tube's place; a straight-down grip reaches only about 18 cm there. So
the arm draws the whole stick out in one stroke by the handle, with the level grip. The stick's 50 g weight is about
0.5 N, far under the arm's ratings (DM-J4340 joints 1-3: 9 N m rated, 27 N m peak; DM-J4310 joints 4-6: 3 N m rated,
7 N m peak; grip capped at 50 N by i2rt's driver; at the hand about 11 N continuous horizontally in the weakest
direction, about 28 N peak, about 52 N continuous pressing down, from the URDF Jacobian with gravity).

### Scoring (thresholds and reasons)

Everything is scored from object states: the dipstick's joint position q (how far the cap is above its seat), the
rag's and the puck's positions and bounding boxes, the grippers' poses and finger openings (for contact with their own
shape), and the trial's level.

- Out = q at least the blade's length plus 5 mm (the tip clear of the mouth).
- Seated = q of 3 mm or less (a 3 mm lift of the cap is visible against the collar).
- Held = a gripper with both fingers touching the T-handle (contact, as for let go below) and their opening stopped
  by it: across the handle's 1.8 cm thickness or 3.5 cm width (the 1.8 cm less 15 % of full opening, to the 3.5 cm
  plus 3 %). An empty fist, an open hand round the handle, one finger on it or a hand hovering over it does not hold
  it, so a dipstick slid home beside a hand that is not holding it has not been pushed home by hand.
- Blade handled = any gripper's own shape within 2 mm of the blade while the dipstick is off its seat. The trial
  scores 0: the dipstick is drawn and pushed by its handle only, and a pinched blade smears the band and the reading.
- Wipe = while the blade is out and still smeared, the rag touches it when its middle is within 2.7 cm of the blade's
  axis (half the pad, 2 cm, plus the blade's half-thickness and about 0.5 cm of cloth pressed round it), its own
  shape, turned as it lies, is within 2 mm of the blade's faces (a rigid pad held 5 mm clear does not count) and its
  extent along the tube overlaps the blade out of the mouth. The scorer follows the rag's middle along the blade in
  1 mm bins from the tip and marks the stretch it slides over between two touching steps, at most 5 mm a step;
  nothing is credited across a step without contact. The blade is wiped once the middle has slid over 90% (5.4 cm)
  of the blade's 6 cm oiled length in that pull. The oiled length runs from the tip up past the highest level set
  (2.7 cm) and the crosshatch, plus the splash film from the running engine. The pad reaches 2 cm either side of
  its middle, so a stroke over the whole length rubs cloth from below the tip to 2 cm above the oil. A dab, dabs at
  several heights, a spot wipe or a stroke over part of the length (3 or 4.5 cm) does not count. Strokes add up within
  one pull, so two half strokes that together cover the length count; the tip going back into the tube clears the
  count. Either hand may move.
- Reseat = the wiped blade pushed back in, by the handle (held), until seated. A dipstick dropped back in, or seated
  without a wipe, starts the procedure again. A wiped blade that goes back into the tube part way and comes out again
  is smeared again and needs a new wipe.
- Reading = after the reseat, the dipstick pulled out again and held by the handle, out and still (moving under
  1 cm/s), for 1 s. It is read tip down, as it hangs.
- Report = the puck down in a pocket: its middle within 1.2 cm of the pocket's middle along x and y (a puck lying flat
  in the 4.4 cm pocket can be at most 7 mm off) and its bottom at least 2 mm below the card's top (in the pocket, not
  on a rim or a wall). It counts at the end, with the puck at rest and let go. The pocket it ends in is the report;
  moving it after the reading is allowed.
- Guess = the puck down in any pocket for 2 control steps (0.1 s) in a row before the reading has been taken. A
  guess scores 0 even if the puck later ends in the right pocket. One step is allowed for a physics pop.
- Closed up = after the reading, the dipstick pushed back in by the handle until seated, and let go. Rag down = the
  rag at rest on the table (its bottom 1 cm or less above it) and let go.
- Let go = no gripper touching the object, judged by contact with the gripper's own shape (its palm and both fingers
  at their present opening, as boxes fitted to its collision meshes): a gap of 2 mm or less, the solver's resting gap
  plus the fit of the boxes to the meshes.

| Outcome | Score | Why it is worth that |
|---|---|---|
| A gripper touched the blade off its seat, a wrong report, or a guess, whatever else was done | 0 | A handled blade gives no true reading; a wrong level on the checklist sends the next job the wrong way, and a wrong report must never pay more than none |
| Nothing done, or a first-pull reading without a wipe, or a wipe never followed by putting the dipstick back | 0 | No true reading is possible without the wipe and a full reseat |
| Wiped and seated by hand, but the second pull never held out still for 1 s | 0.25 | The procedure that makes a true reading possible is done; the reading is not |
| True reading, but no report (the puck not in a pocket, on a wall or rim, or still held) | 0.5 | The level was looked at but not recorded |
| The right report, but the dipstick left out, part way in, dropped back or still held, or the rag not put down | 0.75 | The check is done and recorded, but the engine is left open to dirt or the bench is not put back |
| All of it (where the arms end up is not scored) | 1.0 | |

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. A policy that always puts the puck in one pocket averages about 0.33 over the
three levels, below the 0.5 for reading and not reporting, and a report taken from the smeared first pull (always
past FULL) is wrong two times in three. A real trial is scored from the video in the same way (see Real-world build).

### Self-QC (each case is a test in `test_task.py`)

Do nothing (0); first pull read and put back (0); wipe skipped (0); rag held 6 cm off the blade (0) or hovering 3 cm
from it (0); rigid pad 4.5 mm clear (0); 3 cm and 4.5 cm wipes of the 6 cm length (0); a dab (0); dabs at three
heights (0); half the length, 6 to 3 cm (0), and scrubbed back and forth over that half (0); touch at the top, lift,
touch at the bottom (0); a one-step jump (0); half a stroke, the tip back in the tube, then the other half (0); a full
stroke down, up, or stopping 6 mm short (1.0); two half strokes covering the length (1.0); a full stroke with a 2-step
contact gap (1.0); rag rubbed on the cap with the dipstick seated (0); half-seated reinsert before the second pull (0);
second pull not held still 1 s (0.25); dropped back in after the wipe (0); left out, left half in, still held at the
end with the right report (0.75); rag still in hand (0.75); arms left away from home (1.0); dipstick let fall to its
seat beside an empty closed fist (0) and the same push with the handle gripped (1.0) or gripped across its width
(1.0); reseated with an open hand round the handle (0), one finger on it (0) or a closed hand hovering 5 mm over it
(0); drawn out hand over hand with the blade pinched (0, the blade handled); an empty fist at the blade during the
reading (0); rag drawn down the still blade by the other arm (1.0); half-dipped, then re-wiped and done right (1.0);
history reset by a new trial (0). The report, at each level: the right pocket (1.0); each wrong pocket (0); no report
(0.5); the right report with the rag in hand (0.75); at the low level, OVER from the first pull (0); not let go (0.5);
let go with the open hand hovering 1 cm over it (1.0), then closed on it again (0.5); straddling a wall (0.5); on a
rim (0.5); a wrong pocket after the reading, then the right one (1.0); the right pocket, then a wrong one (0); the
right pocket before the second pull (0); dipped in a pocket before the reading (0); a one-step pop before the reading
(1.0); the right pocket without a wipe (0); always guessing one pocket averages about 0.33; the levels spread evenly under
seeded jitter; the demo aims for the pocket of the level it is given; the drawn band follows the level.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `yamkit.py` | Helper that spawns the fill tube as an articulation (the sliding dipstick), the housing box with its can, the rag, the card and the puck, gives the scorer the joint state and the grippers, and adds tilted demo waypoints. It patches `yam_arena` in memory at import; no files change. |
| `guardkit.py` | Contact with the gripper's own shape (palm and fingers at their present opening): the let-go and blade checks use its `touching()`, the held check its `gripping()`, the wipe its box gap between the rag and the blade. |
| `test_task.py` | Offline scorer checks (the self-QC cases above). No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score; the level is between the marks with no jitter, and `OIL_CASE=low` or
`OIL_CASE=over` in the environment runs the other two levels):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/oil-dipstick-wipe-reinsert-read/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Check the oil level on the engine with the dipstick, mark it on the check card with the puck, then put everything back." \
      --scorer task_score --max-steps 2400

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`, run at all three levels (`OIL_CASE=low`,
`ok` and `over`, or seeded jitter), since at one fixed level always marking the same pocket would pass. Videos:
`inspect-robots video <log.json>`. Scorer tests: `python tasks/oil-dipstick-wipe-reinsert-read/test_task.py` (prints
`all pass`).

## The demo

1. Right arm: comes over the handle with its gripper level, pointing into the corner from the box's open front
   side, lowers its open fingers either side of the handle, grips it across its 1.8 cm thickness and draws the
   dipstick straight up in one stroke until the tip is 3 cm clear of the mouth (the hand about 59 cm above the table),
   and holds it there.
2. Left arm: picks up the rag, waits, then pitches its wrist about 75 degrees so the gripper reaches in level from the
   robot's side, holding the pad on edge with its broad face to the blade's near face, sets it on the blade 5 mm above
   the tip and draws it up the blade over 5 s until its middle is 8 cm above the tip (7.5 cm slid, past the 6 cm oiled
   length with 2 cm to spare), then takes it away. (A pad brought up from under the tip catches on its end, and one
   drawn downward digs its leading edge into the blade.)
3. Right arm: pushes the dipstick back down by the handle until the cap seats (4 mm past, so it is pressed down), lets
   go and lifts its hand; takes the handle again and draws it out the same way, and holds it out and still (3.4 s)
   while the left arm reads it.
4. Left arm: puts the rag back where it was, then reads the level with its wrist camera: it waits 7 cm back, and once
   the blade is out and still brings the camera to 10 cm from the blade's oiled end (aimed 1.6 cm above the tip,
   looking down about 32 degrees from the robot's side, the open gripper 4.5 cm or more from the blade), holds it
   there 1.5 s with the oil band and the ADD and FULL marks in view, and backs off.
5. Right arm: pushes the dipstick back down until it seats and lets go. Left arm: only after the reading, picks up
   the puck, sets it in the pocket and lets go. Both arms go home and hold.

The demo is a scripted demonstration, not a policy. It wipes a planned stroke; the scorer checks, from the rag's and
the blade's states, that the oiled bins were actually slid over in contact. It shows the blade to the left wrist
camera as a reader would, but it is told the trial's level (`OIL_CASE`, or between the marks with no jitter) and aims
for that pocket; it does not interpret the image. The scored policy is told nothing and has to read the oil band off
the blade against the ADD and FULL marks. The left wrist camera, held up to the blade as in the demo, shows the band
and both marks plainly (in the demo's left wrist camera video: the blade about 18 px wide and the 3 cm from the tip about
45 px tall at 320 x 176); the top camera sees the blade only as a thin line and the right wrist camera sees the
handle.

## Runtime and seeds

- Demo: 853 control steps (42.65 s simulated) at each of the three levels, ended on success.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- One fixed layout (`object_jitter=0`); the oil level is the one thing that changes between trials. Score a model on
  all three levels (`OIL_CASE=low`, `ok`, `over`), or on seeded resets, which also pick the level.

## Fidelity

The sim models the low-cost real build (Build B under Real-world build): the genuine dipstick and fill tube at the
engine's mouth height and corner from Kohler's drawing, in a quart can, at the corner of the black plywood box with its
cut corners, on the plywood base, all at the built sizes. The used-engine route (Build A) gives the robot the same tube,
handle and housing footprint (see Real-world build). The tube is modeled straight and vertical; the genuine tube bends
just under the cap. The dipstick has its weight and slides on the tube's axis even when out of the tube (a guide), so
it cannot be laid down or tilted, and the sim never asks the robot to line the tip up with the bore on the way back
in; a real trial uses the free dipstick, so it does test that. The blade runs from the drawing's mouth height to 3 cm above
the base, with its marks 0.8 and 2.0 cm from the tip, and the genuine stick's length and marks are recorded. The real
trials record whether the free dipstick goes back into the tube unaided (see Real-world build). The oil is drawn, not
simulated: the smear, its removal by the rag and the trial's level are visuals driven by the scorer's state. The rag
is a rigid pad. The gripper covers, drip pan and towels of the real rig are not modeled.
Clearances were checked offline along the demo at all three levels (forward kinematics of the demo's joint commands,
the arms' collision meshes with the fingers at their commanded opening, against the box, the can, the tab and the
base): apart from the intended contacts (the grip on the handle, the blade in its tube, the rag on the blade, the held
rag and puck), the nearest pass is 13.9 mm (right fingertip over the box's top edge by the corner), and the can is
never nearer than 6.3 cm.

## Real-world build

The rig needs a real dipstick in a real fill tube over a real oil reservoir, with the handle at the engine's height
and corner. The sim models the low-cost build, Build B; a used engine, Build A, is the alternative. The robot's task,
the instruction and the scoring are the same on both. Build B is a low-cost setup of widely available
materials: the two genuine Kohler parts, a quart paint can, plywood offcuts, black paint and engine oil, with the gripper
covers, towels, drip pan and waste can below.

**Build B, the low-cost rig the sim models.**

| Part | What to use | Note |
|---|---|---|
| Dipstick | Kohler 32 038 14-S, dipstick assembly (Russo Power, special order) | listed for the KT715-KT745 (L&S Engineers; eReplacementParts' KT725 cross-reference) |
| Fill tube | Kohler 32 123 14-S, tube assembly, oil fill (Russo Power, special order) | on the KT725 crankcase parts lists (Rehlko engine parts, KT725-3037; Jack's Small Engines, KT725-3048) |
| Oil can | An empty 1 quart unlined steel paint can with lid, about 10.8 cm across and 12.4 cm tall: Polar Tech HAZ1025 (Zoro), or any hardware store's empty quart paint can | the tube goes down through a hole drilled in the lid; the oil stays below the lid, so a plain hole serves |
| Housing box | 12 mm plywood offcuts, black spray paint | 41 x 41 cm, 30 cm tall, open at the bottom, its corners cut (below) |
| Base | An 8 mm plywood offcut, 47 x 50 cm | the box and the can stand on it |
| Oil | 10W-30 or SAE 30 engine oil, one quart | grades from Kohler's manual, p.4 |

The reservoir is a quart can, not a gallon can: it stands in the open corner under the tube, where a 19 cm tall gallon
can would come up beside the left hand's path.

Lead time: Russo Power lists both Kohler parts as special order, which for Kohler parts takes about 2 to 6 weeks
(Partswarehouse's special-order lead time). A used engine (Build A) is the in-stock route: it ships with its own
dipstick and tube.

Assembly, once, about 30 minutes plus the paint's drying time (Build A: about an hour to drain the fuel and bolt the
engine to the plate). Tools: use a drill with a step bit, a jigsaw, a screwdriver, a tape measure and a steel rule, black
spray paint.

1. Cut the box: four 30 cm sides and a top, 41 x 41 cm outside. Cut its near right corner (the robot's near right, where
   the tube stands) off as two flat faces, each about 15 cm wide: from 20 cm along the front side to a point 6 cm in
   from both the front and the right side, then to 20 cm along the right side. The two corner faces run from the top
   down to 20 cm above the base and are open below, so the can stands in the opening. Cut the other three corners off
   at 45 degrees, 13 cm along each side (12 cm at the far right corner); a square corner would stand 5 cm farther out,
   in the left arm's path home. Paint it black and screw it to the base, at x 30.7 to 71.7, y -14.5 to 26.5.
2. Drill a hole in the middle of the can's lid to the tube's outside diameter, measured with the steel rule.
3. Stand the can on the base under the tube's place (its middle at x 36, y -12), on a spacer if needed, so the seated
   dipstick's tip is about 2-3 cm above the can's floor.
4. Fit the tube's O-ring end down through the lid into the can and screw the tube's tab to the corner face (or a block
   behind it) with an M5 screw, as Kohler's manual (p.82) secures it to the engine's backing plate, with the mouth 27.5
   cm above the base and the tube upright.
5. Pour in oil to the trial's level (see Each trial) and seat the dipstick.

The robot meets what it meets on the engine: a T-handle over a cap at 28-29 cm above the bench, at the corner of a
housing it has to work beside, the long blade drawn straight up, and an oil level that shows only after the wipe and
reseat. Measure the genuine dipstick's length and mark positions on arrival and record them. The sim's stick is 29 cm
overall, with its marks 0.8 and 2.0 cm from the tip; if the genuine blade differs from
24.5 cm by more than 1 cm, record the difference with the results. The real tube bends just under the cap; the sim's
is straight.

**Build A, a used engine (alternative).** A used Kohler 7000 Series engine (KT715 to KT745, the KT725's series, which
share its dipstick part) from eBay's Multi-Purpose Engines category (Kohler, vertical shaft) or a local lawn tractor
parts seller. It is the in-stock route. The rig engine stays cold and
unfueled, so a non-running engine serves as well as a running one: it needs its crankcase, oil pan, fill tube and
dipstick, and it holds oil overnight on the drip tray with the tray staying dry. Drain the fuel and bolt it by its
mounting surface to a steel plate, turned as the drawing is turned here, so the fill tube's mouth is at x 36, y -12,
27.5 cm above the plate. If its dipstick or tube is missing, fit the genuine parts above.

The robot meets the same tube and handle on the engine: the mouth's height and place, the T-handle and the blade are
the same, and the blower housing has the box's footprint and top. Its corner at the tube is rounded (radius 20 cm, a
skirt down to 20 cm above the plate), never more than 1.5 cm out from the box's cut faces, so the tube's axis is 2.8 cm
from the housing against 3.5 cm on the box; under that corner the engine has its crankcase, oil pan, oil filter and the
tube's boss where the box has the can. Checked along the demo (forward kinematics of its joint commands and the arms'
collision meshes, against the box with its can and against a model of the engine sized from the drawing):
over the three levels, the nearest pass to the box is 14.4 mm for the left arm (its forearm, setting the puck in a
pocket) and 13.9 mm for the right arm (a fingertip lifting off the handle, over the box's top edge by the corner),
against 10.5 mm and 14.8 mm to the engine; the box's square top edge stands about 1 mm closer there than the engine's
rounded one. The nearest pass to the can is 6.3 cm (the left forearm picking up the rag). So the arms work the same
on both builds.

**Dimensions, sim against real** (the sim is Build B; the real build is checked against these with a tape and a steel
rule):

| Part | Dimension | Sim (Build B) | Real build tolerance | Source |
|---|---|---|---|---|
| Fill tube mouth | height above the base | 27.5 cm | +-1 cm | Kohler service manual 32 690 03 Rev. H, p.5 drawing |
| Fill tube | place | the box's front corner, x 36, y -12 | +-1 cm | drawing p.5 ("OIL FILL & DIPSTICK (YELLOW)") |
| Housing box | footprint, top height | 41 x 41 cm, 30 cm above the base | +-1 cm (Build A: the engine's own housing) | drawing p.5 |
| Housing box | corner at the tube | two flat faces, 20 cm along each side to 6 cm in, down to 20 cm above the base, open below; tube axis 3.5 cm from them | +-1 cm (Build A: rounded, radius 20 cm, within 1.5 cm of the faces; tube axis 2.8 cm from it) | drawing p.5 (top view) |
| Housing box | other corners | cut at 45 degrees, 13 cm along each side (12 cm at the far right) | +-1 cm | built, inside the engine's rounded corners |
| Oil can | size, place | 1 quart, 10.8 cm across, 12.4 cm tall, under the tube | any quart paint can (Build A: the engine's crankcase and oil pan) | built |
| Base | size | 47 x 50 x 0.8 cm | +-2 cm, thickness +-2 mm (the mouth height is measured from it) | built |
| Dipstick | overall length, blade | 29 cm, 24.5 cm | as on the genuine stick, recorded | drawing p.5 (mouth height to 3 cm above the base) |
| Dipstick | ADD and FULL marks from the tip | 0.8 and 2.0 cm | as on the genuine stick, recorded | the sim's marks |
| Fill tube | outside diameter | 2.1 cm | as supplied | drawing p.5 |
| Oil band | levels set | 0.4, 1.4, 2.7 cm from the tip | 3 mm or more from a mark, else reset | set by syringe |
| Shop rag | folded pad | 4 x 4 x 2.5 cm | +-0.5 cm | a folded shop towel |
| Check card | pockets | 4.4 cm square, 5 mm deep | +-1 mm | built (3D print or foam board; no stock tray has these pockets) |
| Puck | size | 3 cm across, 2 cm tall | +-2 mm | two 1-1/4 in wooden checkers, glued |
| Room | temperature | oil drawn | 20-24 degrees C | set so the oil's thickness on the blade is the same every trial |

**Keeping the oil off the robot.** The shared rig stays clean with these, fitted before each trial and replaced
after it:

- Each gripper finger wears a disposable nitrile glove as a cover: GloveWorks HD GWON48100 (Ammex), 8 mil, orange,
  diamond-textured on both sides, size XL (a box of 100). Pull it over the finger like a sock, the finger in the
  glove's palm and the glove's own fingers folded back along its outside, and close the cuff round the finger's base
  with one turn of tape, away from the grip faces. One layer of nitrile then covers the pad, the tip and the inner
  face. Four gloves a trial; the used ones go in the oily waste can (below). A glove fits the YAM's finger, about 6.8
  cm wide and 2.4 cm thick with a 4 cm pad, which is larger than the largest nitrile finger cots (Uline S-15365, size
  XL, made for a human finger); it covers the finger's sides as well as the pads, where the oily rag touches; and its
  nitrile keeps the grip firm.
- The base (or the engine's plate) stands in a Lisle 19722 Catch-All Drip Pan, 24 x 36 x 1.5 in, polyethylene, 8 quart,
  with a folded blue shop towel (Scott Shop Towels, the same roll as the rag) in it under the fill tube, where a drop
  from the blade lands. Two more folded towels cover the bench under the rag's place and the card. A fresh towel for
  the rag each trial; the used ones go in the oily waste can.
- Used gloves and towels go straight into a closed metal oily waste can: an empty 1 gallon metal paint can with its
  lid (BEHR 96601, Home Depot), its lid pressed on. It is emptied into the shop's oily-waste disposal at the end of
  each session, never left full overnight.
- Grip: the sim's grasped objects have a rubber-like contact (static friction 1.2, dynamic 1.0, `yam_arena`'s
  `GRIPPY_FRICTION`), and the covers keep a rubber (nitrile) face on the grip, textured, at 0.2 mm thick; the handle's
  1.8 cm grip is unchanged within that. The loads held are light (the 50 g dipstick, the 40 g rag, the 15 g puck), far
  under the 50 N grip cap. Before each session, with the covers on, the arm holds the dipstick out by its handle for
  10 s and lifts the rag and the puck without a slip; a cover that shifts on the finger is taped again.

**Free withdrawal and reinsertion.** The sim's dipstick slides on the tube's axis even when out; the real one is
free, so the real trials score the guided part directly: in each trial the tester records whether the tip went back
into the mouth without help on each push (the reseat and the closing seat) and how many tries it took. A dipstick that
the robot drops or lays down, or that the tester has to guide, ends the trial, scored on what was done up to that
moment. The results report this separately as the free-reinsertion rate, the measure of the gap between the guided
sim stick and the free real one.

**Time.** Building the rig is once: about 30 minutes for Build B (above). Resetting between trials takes about 8
minutes: the measurement after the trial (pull, wipe, reseat, pull, measure the band and
photograph it, about 2 minutes); new gloves on all four fingers and a new towel (about 2 minutes); setting the next
level by syringe and checking it with a pull and a wipe (about 3 minutes); smearing the blade and the tube, seating
the dipstick and putting the rag and the puck back (about 1 minute). With the trial itself (at most 2 minutes), a
block of three trials takes about 30 minutes.

**Each trial.** Trials run in blocks of three, one at each level, in an order shuffled by a die roll, so every level is
tested equally often. Before each trial, set the oil with a syringe to just under the indicator's bottom, mid-range, or
just over its top, check it with a pull and a wipe out of the camera's view, wipe the blade, smear oil on its lower 6
cm and up the inside of the tube, and seat it. No stock tray has three 4.4 cm square pockets 5 mm
deep (the nearest three-compartment trays have pockets about 10 cm across), so the check card is built: a 3D print of
the sim's card, or two layers of 5 mm foam board, 6 x 16 cm, with three 4.4 cm square holes in the top layer; the
floors painted amber, green and red, labeled ADD, OK and OVER and marked with the same white symbols as the sim; taped
to the bench at x 20, y 13. The puck is two standard 1-1/4 in (3.2 cm) wooden checkers glued together and painted
white (about 2 cm tall); no stock disc is 3 cm across and 2 cm tall. Score from the video as under Scoring: the
dipstick handled only by its handle, the rag drawn along at least 5.4 cm of the blade's lower 6 cm in contact, the gap
under the cap after each push (a 3 mm feeler strip), the second pull's 1 s hold (the video's timer), the frame the
puck goes into a pocket and the pocket it ends in, and the rag back on the table.

## Results by level

Every trial records the level it was set to and the level the robot reported, so results can be compared level by
level.

Sim (scripted demo, `OIL_CASE` set per run; the demo is told the level, see The demo):

| Level | Oil band from the tip | Pocket reported | Score | Steps |
|---|---|---|---|---|
| below ADD (`low`) | 0.4 cm | ADD | 1.0 | 853 |
| between the marks (`ok`) | 1.4 cm | OK | 1.0 | 853 |
| above FULL (`over`) | 2.7 cm | OVER | 1.0 | 853 |

A scored model is run at all three levels and reported the same way, one row per trial, with the per-level count of
right reports.

Real trial: after each trial the tester pulls the dipstick, wipes it, reseats it, pulls it again and measures the oil
band from the tip with a steel rule to the millimeter, photographed against the blade. One row per trial:

| Trial | Block | Level set (die) | Band measured (mm from tip) | Level by the marks | Tip back in unaided (reseat, closing seat; tries) | Pocket the puck is in | Puck in before the 1 s hold? | Score |
|---|---|---|---|---|---|---|---|---|

The measured band, not the planned one, decides the right pocket (against the dipstick's own marks); a set level that
lands within 3 mm of a mark is reset and the trial rerun.

## Licenses

All code here was written for this task; no third-party code or assets are included. Every shape is generated by
`yamkit.py` and `task.py`.
