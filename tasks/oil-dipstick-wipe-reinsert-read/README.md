# Check a lawn tractor's oil with its 29 cm dipstick, wiping and re-reading it, and mark the level on the check card

A bench model of a lawn tractor's engine stands on the table with its yellow 29 cm dipstick seated in the oil fill
tube, which slopes up out of the crankcase toward the robot. A folded blue shop rag lies beside it, next to an oil check card
with ADD, OK and OVER pockets and a white marker puck. "Check the oil level on the engine with the dipstick, mark it
on the check card with the puck, then put everything back." means the procedure every lawn tractor manual gives before
mowing: pull the dipstick (hand over hand: it is longer than one arm's stroke), wipe the blade clean on the rag, push it back in until the cap seats, pull it
again and hold it still to read the level, push it back in until it seats, put the rag down, and record what the
blade showed. The level changes from trial to trial (below ADD, between the marks, or above FULL), so the puck lands
in the right pocket only if the blade was read. The tempting wrong answer is to read the first pull, which shows a
false smeared level past FULL from oil splashed up the tube.

## Spec

### Scene (cm from the midpoint between the arm bases, x forward, y to the robot's left, table top z = 0)

| Part | What it is | Where |
|---|---|---|
| Engine | Fixed bench model of a lawn tractor engine's crankcase side: wooden mounting board (21 x 20 x 0.8 cm), aluminum crankcase (17 x 18 x 12 cm) with a red flywheel shroud on top (to z 17) | board x 45 to 66, y -20 to 0 |
| Oil fill tube | Yellow tube, 2.4 cm across, sloping up at 25 degrees toward the robot: it leaves the crankcase's near face about 8 cm up and its mouth is 10 cm out, at z 12.5; it runs 26 cm down inside the crankcase | mouth at x 38, y -10 |
| Dipstick | Prismatic link sliding along the tube's axis: steel blade 1.0 x 0.3 x 24.5 cm with black ADD and FULL marks 0.8 and 2.0 cm from the tip and a crosshatch between them, a yellow cap disc 3.2 cm across and 1 cm thick that seats on the tube's rim, and a yellow T-handle 3.5 cm wide, 1.8 cm thick, 3.5 cm long. 29 cm overall, 50 g, the length of a Kohler SV-series lawn tractor engine's dipstick. Starts seated; it stays where it is let go | cap on the rim at the mouth |
| Shop rag | Folded blue shop towel pad, 4 x 4 x 2.5 cm, 40 g, loose | centered x 26, y 10 |
| Oil check card | Dark gray tray 6 x 16 x 1 cm, fixed: three pockets 4.4 cm square and 5 mm deep, 0.8 cm walls between them. Each floor carries a white symbol (2.8 cm, 6 mm strokes) so the pockets read without color: ADD amber with a minus sign, OK green with a check mark, OVER red with a plus sign | centered x 35, y 10; pockets at y 15.2 (ADD), 10 (OK), 4.8 (OVER) |
| Marker puck | White puck, 3 cm across, 2 cm tall, 15 g, loose | x 20, y 15 |

The blade carries black ADD and FULL marks 0.8 and 2.0 cm from the tip (2 mm proud of its edges) and a drawn oil
film (visual only): at the start, a ragged smear over its lower 6 cm (the false first reading); the smear disappears
wherever the rag passes; after a wipe and a full seat, the blade shows a clean oil band from the tip up to the
trial's level, which is one of three:

| Level (`OIL_CASE`) | Oil band from the tip | Reads | Pocket |
|---|---|---|---|
| `low` | 0.4 cm, 4 mm under the ADD mark | below ADD | ADD (minus sign) |
| `ok` | 1.4 cm, 6 mm from either mark | between the marks | OK (check mark) |
| `over` | 2.7 cm, 7 mm above the FULL mark | above FULL | OVER (plus sign) |

Each level sits 4 mm or more from a mark, so it reads the same by eye in any camera that sees the blade. The
seeded reset picks the level: every free object is jittered by up to 3 cm, and the rag's offset from its place in
whole millimetres (x + y), modulo 3, picks low, ok or over (the rag's offset is noise, not a cue a camera could use).
With no jitter (`object_jitter=0`) the level is between the marks, and the environment variable `OIL_CASE` sets it
for a run. The band appears while the blade is inside the tube (after the full seat), so it never pops in on camera.

### Instruction

"Check the oil level on the engine with the dipstick, mark it on the check card with the puck, then put everything
back." The wipe-and-reread rule, the full seat and the closing seat are not stated. The instruction names the card
and the puck because a robot cannot speak its reading; the card stands in for writing the level down.

### Rule and why

Engine manuals (Briggs & Stratton, Honda, Kohler) say to remove the dipstick, wipe it
clean, reinsert it fully and remove it again to read. Oil splashed up the fill tube while the engine ran (or while it
was carried) coats the blade above the true level, so a first-pull reading overstates the oil and a low engine gets
run. A dipstick pushed only part way in before the second pull reads low or picks up the tube's splash again. The
dipstick goes back in until its cap seats, so dirt cannot get into the crankcase, and the rag goes back down. The
level found decides what comes next: top up a low engine, mow, or drain an overfilled one.

Category: Societal relatability. Checking the oil before mowing is the first maintenance step in a lawn tractor's
manuals, and the owner does it. Kohler's owner's manual for its Courage SV470-SV620 engines (single-cylinder
vertical-shaft engines on lawn tractors and riding mowers) gives exactly this procedure: "Remove dipstick; wipe oil off.
Reinsert dipstick into tube; press completely down. Remove dipstick; check oil level. Level should be at top of
indicator on dipstick," and "Never run engine with oil level below or above operating range indicator on dipstick."
In a 2018 Harris Poll of over 2,000 US adults for the Outdoor Power Equipment Institute, 63% owned outdoor power
equipment, and a lawn tractor's owner checks its oil the same way before each mowing.
Automotive technicians and small-engine mechanics are not claimed.

The dipstick is a lawn tractor's, at its real size: 29 cm overall (24.5 cm blade, 1 cm cap, 3.5 cm T-handle), sliding
in a fill tube that leaves the crankcase low and slopes up to a mouth at the engine's side. Kohler sells it as a part
(20 040 08-S, "Dipstick, bent tube", for SV470-SV620 in Kohler's accessories guide; 32 038 08-S for SV715-SV840) but
publishes no length; an SV720 owner measured his at about 11-1/2 in (29 cm). The real trial uses the Kohler part and
records its measured length. A walk-behind mower's stick is shorter (Honda's GX gauge sticks are 3.3 and 5.4 cm) and a
car's is longer (50 to 63 cm, Dorman 917-390 and 917-322); a car's tube drops 35 to 50 cm to the oil pan, past the
reach of arms that grip no higher than about 25 cm above the table, so the task uses the lawn tractor's.

A stick this long is longer than one arm's stroke, so it comes out hand over hand, as a person draws one out: grip the
handle and draw it as far as the hand goes, then pinch the blade at the mouth and draw it the rest of the way; going
back in, push the blade in to the mouth, then the handle home. The scorer counts a pinch on the blade as holding the
dipstick, as it does a grip on the handle.

### Scoring (thresholds and reasons)

Everything is scored from object states: the dipstick's joint position q (how far the cap is above its seat), the
rag's and the puck's positions and bounding boxes, the grippers' poses and finger openings (for contact with their own
shape), and the trial's level.

- Out = q at least the blade's length plus 5 mm (the tip clear of the mouth).
- Seated = q of 3 mm or less (a 3 mm lift of the cap is visible against the rim).
- Held = a gripper with both fingers touching the T-handle or the blade (contact, as for let go below) and their
  opening stopped by it: across the handle's 1.8 cm thickness or 3.5 cm width (the 1.8 cm less 15 % of full opening,
  to the 3.5 cm plus 3 %), or pinching the blade across its 3 mm thickness or 1 cm width (hand over hand). An empty
  fist, an open hand round the stick, one finger on it or a hand hovering over it does not hold it, so a dipstick
  slid home beside a hand that is not holding it has not been pushed home by hand.
- Wipe = while the blade is out and still smeared, the rag touches it when its middle is within 2.7 cm of the blade's
  axis (half the pad, 2 cm, plus the blade's half-thickness and about 0.5 cm of cloth pressed round it; a rag hovering
  clear sits farther out), its own shape, turned as it lies, is within 2 mm of the blade's faces (a rigid pad held 5 mm clear
  does not count) and its extent along the tube overlaps the blade out of the mouth. All of this is measured in the
  tube's frame, along and across its 25 degree slope. The scorer follows the rag's middle along the blade in
  1 mm bins from the tip and marks the stretch it slides over between two touching steps, at most 5 mm a step;
  nothing is credited across a step without contact, so a lift and a touch elsewhere adds nothing. The blade is wiped once the middle has slid over 80% (3.2 cm) of the blade's lower 4 cm in that pull: the pad
  reaches 2 cm either side of its middle, so a stroke between 4 cm and the tip drags cloth over the whole 6 cm smear
  and the crosshatch, and 80% lets a stroke start or stop 8 mm short (the arm lags its target about 1.6 cm).
  Touching is not wiping: a dab, or dabs at several heights, slide over nothing, and half a stroke (2 cm) leaves the
  marks smeared. Strokes add up within one pull, so two half strokes that together cover the span count, since a
  person who wipes in two passes also ends with a clean blade; the tip going back into the tube clears the count.
  Either hand may move: drawing the rag along a blade held still is how most people do it.
- Reseat = the wiped blade pushed back in, by hand (held), until seated. A dipstick dropped back in, or seated without
  a wipe, starts the procedure again. A wiped blade that goes back into the tube part way and comes out again is
  smeared again and needs a new wipe.
- Reading = after the reseat, the dipstick pulled out again and held, out and still (moving under 1 cm/s), for 1 s.
  The dipstick stays on the tube's axis, so it is read tip down, as manuals advise (tipping it up runs oil up the
  blade).
- Report = the puck down in a pocket. The robot cannot speak its reading, so it marks it the way one ticks a
  checklist box. Down in a pocket means: its middle within 1.2 cm of the pocket's middle along x and y (a puck lying
  flat in the 4.4 cm pocket can be at most 7 mm off; one hanging on a wall with its middle outside does not count)
  and its bottom at least 2 mm below the card's top (in the 5 mm pocket, not lying on the rim or on a wall between
  two pockets). The report counts at the end, with the puck at rest and let go (no gripper touching it),
  because a puck still in the hand has not marked anything. The pocket it ends in is the report; moving it after
  the reading is allowed, as anyone may correct a note. A real trial reads the pocket off a top photo and
  checks from the side that the puck sits below the card's top.
- Guess = the puck down in any pocket for 2 control steps (0.1 s) in a row before the reading has been taken: the
  level was not known yet, so whatever pocket it went into was chosen without looking. A guess scores 0 even if the
  puck later ends in the right pocket. One step is allowed for a physics pop. A real trial compares the frame the puck
  goes in with the end of the second pull's 1 s hold.
- Closed up = after the reading, the dipstick pushed back in by hand until seated, and let go. Rag down = the rag at
  rest on the table (its bottom 1 cm or less above it) and let go.
- Let go = no gripper touching the object, judged by contact with the gripper's own shape (its palm and both fingers
  at their present opening, as boxes fitted to its collision meshes): touching means a gap of 2 mm or less, the
  solver's resting gap plus the fit of the boxes to the meshes. A hand
  hovering beside the object, however close, has let go; fingers still closed on it have not.

| Outcome | Score | Why it is worth that |
|---|---|---|
| A wrong report (the puck left in another level's pocket), or a guess (the puck in a pocket before the reading), whatever else was done | 0 | A wrong level on the checklist sends the next job the wrong way (oil added to a full engine, or a low one left running), and a wrong report must never pay more than none |
| Nothing done, or a first-pull reading (pulled, read, put back) without a wipe, or a wipe never followed by putting the dipstick back | 0 | No true reading is possible without the wipe and a full reseat |
| Wiped and seated by hand, but the second pull never held out still for 1 s | 0.25 | The procedure that makes a true reading possible is done; the reading is not |
| True reading, but no report (the puck not in a pocket, on a wall or rim, or still held) | 0.5 | The level was looked at but not recorded, so nobody downstream can act on it |
| The right report, but the dipstick left out, part way in, dropped back or still held, or the rag not put down | 0.75 | The check is done and recorded, but the engine is left open to dirt or the bench is not put back as the instruction asks |
| All of it (where the arms end up is not scored) | 1.0 | |

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. A wrong report scores below no report, so guessing never pays: a policy
that always puts the puck in one pocket averages 1/3 over the three levels (1.0 once, 0 twice), below the 0.5 for
reading and not reporting, and a report taken from the smeared first pull (always past FULL) is wrong two times in
three. A real trial is checked from the video: count the pulls, check the rag is drawn along the lower blade (over
at least 3.2 cm of its lower 4 cm) between them, look for the gap under the cap after each push (a 3 mm feeler
strip), time the second pull's hold, note the frame the puck goes into a pocket and which pocket it ends in, and
check the rag is back on the table.

### Self-QC (each case is a test in `test_task.py`)

Do nothing (0); first pull read and put back (0); wipe skipped, pull-reseat-pull-read (0); rag held 6 cm off the blade
(0); 3 cm wipe (0); a dab, rag held still on the blade (0); dabs at 4, 2.5 and 0.5 cm, lifted between (0); half
the length, 4 to 2 cm (0), and scrubbed back and forth over that half (0); touch at the top, lift, touch at the bottom
(0); half a stroke, the tip back in the tube, then the other half (0); a full stroke down, up, or stopping 6 mm short
of the tip (1.0); two half strokes covering the span (1.0); a full stroke with a 2-step contact gap (1.0); rag rubbed on the cap with the dipstick seated (0); half-seated reinsert before the second pull
(0); second pull not held still 1 s (0.25); dropped back in after the wipe (0); left out, left half in, still
held at the end with the right report (0.75); rag still in hand (0.75); arms left away from home (1.0: where the arms end is not scored); dipstick let
fall to its seat beside an empty closed fist (0) and the same push with the handle gripped (1.0) or gripped across
its width (1.0); reseated with an open hand round the handle (0), with one finger on it (0) or by a closed hand
hovering 5 mm over it (0); second pull
"held" by an empty fist (0.25), then a report (0, a guess); rag drawn down the still blade by the other arm
(1.0, a real wipe); half-dipped, then re-wiped and done right (1.0); history reset by a new trial (0).
The report, at each level: the right pocket (1.0, and success); each wrong pocket (0); no report (0.5); the right
report with the rag in hand (0.75); at the low level, the first pull read and OVER reported (0); the puck in the right
pocket but not let go (0.5); let go with the open hand hovering 1 cm over it, not touching (1.0), then closed on it again (0.5); straddling the wall between ADD and OK (0.5); on the right pocket's rim (0.5); a wrong
pocket after the reading, then moved to the right one (1.0); the right pocket, then moved to a wrong one (0); the right
pocket before the second pull (0); dipped in a pocket before the reading, taken out, then reported right (0); a
one-step pop into a pocket before the reading, then done right (1.0); the right pocket without a wipe (0); a wrong
report with the dipstick left out (0); always guessing one pocket averages 1/3; with no jitter the level is between
the marks and seeded jitter spreads the three levels about evenly; the demo aims for the pocket of the level it is
given; the drawn band follows the level.

## How it differs from nearby tasks

- Checking a pressure gauge before opening a filter housing, or a fire extinguisher's pin and gauge, reads an
  instrument once and then chooses an action. Here the reading is only valid after a procedure: a
  two-pull reading in which the first reading must be discarded, and the report is scored on that second reading.
- Hand-pumping to a dial's pressure is a repeated stroke against a gauge, with no wipe or reseat.
- Cleaning a fiber connector's end face and then plugging it in is a single wipe-then-insert. Here the wipe sits between
  two pulls and is followed by a full reseat, a second pull held still, and a closing seat.
- Our kettle task (fill between the MIN and MAX lines) scores a level set by pouring; here the level is not changed,
  only read by the right procedure.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `yamkit.py` | Helper that spawns the fill tube as a tilted articulation (the sliding dipstick), the crankcase, the rag, the card and the puck, gives the scorer the joint state and the grippers, and adds tilted demo waypoints. It patches `yam_arena` in memory at import; no files change. |
| `guardkit.py` | Contact with the gripper's own shape (palm and fingers at their present opening): the let-go checks use its `touching()`, the held check its `gripping()` (both fingers on the handle or the blade, the opening stopped by it), the wipe its box gap between the rag and the blade. Trimmed to these two checks. |
| `test_task.py` | Offline scorer checks (the self-QC cases above). No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score; the level is between the marks with no jitter, and `OIL_CASE=low` or
`OIL_CASE=over` in the environment runs the other two levels):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/oil-dipstick-wipe-reinsert-read/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Check the oil level on the engine with the dipstick, mark it on the check card with the puck, then put everything back." \
      --scorer task_score --max-steps 1500

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`, run at all three levels (`OIL_CASE=low`, `ok`
and `over`, or seeded jitter), since at one fixed level always marking the same pocket would pass. Videos: `inspect-robots video <log.json>`.
Scorer tests: `python tasks/oil-dipstick-wipe-reinsert-read/test_task.py` (prints `all pass`).

## The demo

1. Right arm, hand over hand: grips the dipstick's handle from above (fingers across its 1.8 cm thickness) and draws
   it 14.75 cm up the tube, as far as the hand reaches; lets go (the stick stays), slides back down the stick with the
   fingers open, pinches the blade 3.5 cm out of the mouth and draws it on until the tip is 4 cm clear of the mouth.
2. Left arm: picks up the rag and waits; once the blade is out it rolls its wrist about 75 degrees so the gripper
   reaches in from the left side (away from the right hand) holding the pad on edge, presses the pad's broad face onto
   the blade beside the tip and draws it up the blade over 3 s until its middle is 4.5 cm above the tip, then takes it
   away. (Drawn downward, the tilted pad's leading edge dug into the blade and stalled after about 1 cm.)
3. Right arm: pushes the blade back in to the mouth, lets go, takes the handle and pushes it home until the cap seats;
   pulls it out again the same way (tip 2.5 cm clear), holds it still 2.5 s (the reading), pushes it back in until it
   seats and lets go.
4. Left arm: puts the rag back down where it was, picks up the puck, waits over the pocket until the reading has been
   taken, sets the puck in the pocket and lets go. Both arms go home and hold.

The demo is a scripted demonstration, not a policy: it is told the trial's level (`OIL_CASE`, or between the marks
with no jitter) and aims for that pocket; it does not read the blade. The scored policy is told nothing and has to
read the oil band off the blade against the ADD and FULL marks. The demo shows that every motion, including the report, can be done within the run
limit. The top camera sees the blade as a thin line, too small to read the band at 320 x 176; a policy has to bring a
wrist camera to the blade during the second pull and read the level itself. A demo that reads the level from
its own camera would be future work; it is not needed to show the task can be done.

## Runtime and seeds

- Demo: 981 control steps (49 s simulated) at each of the three levels, ended on success.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- One fixed layout (`object_jitter=0`); the oil level is the one thing that changes between trials. Score a model on
  all three levels (`OIL_CASE=low`, `ok`, `over`), or on seeded resets, which also pick the level.

## Fidelity

The dipstick is a lawn tractor's at its real size (29 cm overall; see Rule and why). It slides on the tube's axis even
when out of the tube (a guide), so it cannot be laid down or tilted. The guide means the sim never asks the robot to
line the tip up with the tube's bore on the way back in (the blade, 1.0 x 0.3 cm, has a few mm of play); a real trial
uses a free dipstick, so it does test that reinsertion, and a dipstick dropped on the bench there is left out (0.75
after a right report). The real fill tube curves on its way down to the crankcase, and the bent blade's friction in it
holds the stick where it is let go (its 50 g weight pulls about 0.2 N along the slope); the sim's tube is straight and
the stick's weight is switched off, so it stays where it is let go and moves only when pushed. The oil is drawn, not
simulated: the smear, its removal by the rag and the trial's level are visuals driven by the scorer's state. The rag is
a rigid pad.

## Real-world build

A lawn tractor or riding mower engine (cold, on the machine or on a stand) with its Kohler-type tube dipstick (for
example Kohler 20 040 08-S on a Courage SV470-SV620, or 32 038 08-S on an SV715-SV840; measure and record its length),
or a bench model with the same tube slope and a real 29 cm stick, and a folded shop towel. The fill tube's mouth sits
12.5 cm above the robot's bench, 38 cm in front of the arms' base line, the tube sloping up toward the robot at about
25 degrees. Trials run in blocks of three, one at each level, in an order shuffled by a die roll, so every level is
tested equally often. Before each trial, set the oil to 0.4, 1.4 or 2.7 cm on the blade (above the tip, against the
stick's own marks scaled as in the sim, or the real marks with the levels set just under ADD, mid-range and just over
FULL) with a syringe, check it with a pull and a wipe out of the camera's view, wipe the blade, smear oil on its lower
6 cm and up the inside of the tube, and seat it. The check card is a tray of two layers of 5 mm foam board (or a 3D
print), 6 x 16 cm, with three 4.4 cm square holes cut in the top layer, the floors painted amber, green and red,
labelled ADD, OK and OVER and marked with the same white minus sign, check mark and plus sign as the sim, taped to the
bench at x 35, y 10. The puck is a white checkers piece glued to a second one (3 cm across, about 2 cm tall). Score
from the video as described under Scoring: the level a real trial shows on the blade is the oil itself, and the report
is which pocket the puck ends in.

## Results by level

Every trial records the level it was set to and the level the robot reported, so results can be compared level by
level.

Sim (scripted demo, `OIL_CASE` set per run; the demo is told the level, see The demo):

| Level | Oil band from the tip | Pocket reported | Score | Steps |
|---|---|---|---|---|
| below ADD (`low`) | 0.4 cm | ADD | 1.0 | 981 |
| between the marks (`ok`) | 1.4 cm | OK | 1.0 | 981 |
| above FULL (`over`) | 2.7 cm | OVER | 1.0 | 981 |

A scored model is run at all three levels and reported the same way, one row per trial, with the per-level count of
right reports.

Real trial: after each trial the tester pulls the dipstick, wipes it, reseats it, pulls it again and measures the
oil band from the tip with a steel rule to the millimetre, photographed against the blade. One row per trial:

| Trial | Block | Level set (die) | Band measured (mm from tip) | Level by the marks | Pocket the puck is in | Puck in before the 1 s hold? | Score |
|---|---|---|---|---|---|---|---|

The measured band, not the planned one, decides the right pocket (ADD under 8 mm, OK 8-20 mm, OVER over 20 mm);
a set level that lands within 3 mm of a mark is reset and the trial rerun.

## Licenses

All code here was written for this task; no third-party code or assets are included. Every shape is generated by
`yamkit.py`.
