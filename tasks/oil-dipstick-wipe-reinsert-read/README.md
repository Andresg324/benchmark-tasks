# Check a small engine's oil: pull, wipe, reseat, pull again to read

A bench model of a small four-stroke engine (the kind on a lawn mower or a portable generator) stands on the table
with its yellow dipstick seated in the oil fill tube, and a folded blue shop rag lies beside it. "Check the oil level
on the engine with the dipstick, then put everything back." means the procedure every small-engine and quick-lube
technician follows: pull the dipstick, wipe the blade clean on the rag, push it back in until the cap seats, pull it
again and hold it still to read the level, push it back in until it seats, and put the rag down. The tempting wrong
answer is to read the first pull, which shows a false smeared level from oil splashed up the tube, and put the
dipstick back.

## Spec

### Scene (cm from the midpoint between the arm bases, x forward, y to the robot's left, table top z = 0)

| Part | What it is | Where |
|---|---|---|
| Engine | Fixed bench model: wooden mounting board (16 x 15 x 0.8 cm), aluminum crankcase and cylinder block (10 x 12 x 6 cm) with a red flywheel shroud on top (z 7 to 10), and a fill boss on the crankcase's front | board x 22 to 38, y -22 to -7 |
| Oil fill tube | Yellow tube, 2.4 cm across, rising from the fill boss to its mouth at z 8.5 | axis at x 25, y -12 |
| Dipstick | Prismatic link sliding along the tube's axis: steel blade 1.0 x 0.3 x 8 cm with a crosshatch (ADD to FULL) 0.8 to 2.0 cm from the tip, a yellow cap disc 3.2 cm across and 1 cm tall that seats on the tube's rim, and a yellow T-handle 3.5 cm wide, 1.8 cm thick, 3.5 cm tall. 13 cm overall, 40 g. Starts seated; it falls back down the tube if let go | cap on the rim at z 8.5 |
| Shop rag | Folded blue shop towel pad, 4 x 4 x 2.5 cm, 40 g, loose | centered x 26, y 10 |

The blade carries a drawn oil film (visual only): at the start, a ragged smear over its lower 6 cm (the false first
reading); the smear disappears wherever the rag passes; after a wipe and a full seat, the blade shows a clean oil
band up to 1.6 cm, between the ADD and FULL marks (the true reading).

### Instruction

"Check the oil level on the engine with the dipstick, then put everything back." The wipe-and-reread rule, the full
seat and the closing seat are not stated.

### Rule and why

Engine manuals (Briggs & Stratton, Honda, Kohler) and quick-lube procedures say to remove the dipstick, wipe it
clean, reinsert it fully and remove it again to read. Oil splashed up the fill tube while the engine ran (or while it
was carried) coats the blade above the true level, so a first-pull reading overstates the oil and a low engine gets
run. A dipstick pushed only part way in before the second pull reads low or picks up the tube's splash again. The
dipstick goes back in until its cap seats, so dirt cannot get into the crankcase, and the rag goes back down.

### Scoring (thresholds and reasons)

Everything is scored from object states: the dipstick's joint position q (how far the cap is above its seat), the
rag's position and bounding box, and the grippers' tool points and finger openings.

- Out = q at least the blade's length plus 5 mm (the tip clear of the mouth).
- Seated = q of 3 mm or less (a 3 mm lift of the cap is visible against the rim).
- Held = a tool point within 3 cm of the handle's middle with its fingers closed (opening under 0.75).
- Wipe = while the blade is out and still smeared, the rag touches it when its middle is within 2.7 cm of the blade's
  axis (half the pad, 2 cm, plus the blade's half-thickness and about 0.5 cm of cloth pressed round it; a rag hovering
  clear sits farther out) and its height overlaps the blade above the mouth. The scorer follows the rag's middle along
  the blade in 1 mm bins from the tip and marks the stretch it slides over between touching steps; contact lost for
  up to 3 steps (0.15 s, the pad chattering on the blade's face) keeps one stroke, and a longer gap does not count the
  jump. The blade is wiped once the middle has slid over 80% (3.2 cm) of the blade's lower 4 cm in that pull: the pad
  reaches 2 cm either side of its middle, so a stroke between 4 cm and the tip drags cloth over the whole 6 cm smear
  and the crosshatch, and 80% lets a stroke start or stop 8 mm short (the arm lags its target about 1.6 cm).
  Touching is not wiping: a dab, or dabs at several heights, slide over nothing, and half a stroke (2 cm) leaves the
  marks smeared. Strokes add up within one pull, so two half strokes that together cover the span count, since a
  mechanic who wipes in two passes also ends with a clean blade; the tip going back into the tube clears the count.
  Either hand may move: drawing the rag along a blade held still is how most mechanics do it.
- Reseat = the wiped blade pushed back in, by hand (held), until seated. A dipstick dropped back in, or seated without
  a wipe, starts the procedure again. A wiped blade that goes back into the tube part way and comes out again is
  smeared again and needs a new wipe.
- Reading = after the reseat, the dipstick pulled out again and held, out and still (moving under 1 cm/s), for 1 s.
  The dipstick stays on the tube's axis, so it is read tip down, as manuals advise (tipping it up runs oil up the
  blade).
- Closed up = after the reading, the dipstick pushed back in by hand until seated, and both tool points more than
  6 cm from the handle. Rag down = the rag at rest with its bottom within 1 cm of the table and both tool points more
  than 6 cm from it.

| Outcome | Score |
|---|---|
| Nothing done, or a first-pull reading (pulled, read, put back) without a wipe, or a wipe never followed by putting the dipstick back | 0 |
| Wiped and seated by hand, but the second pull never held out still for 1 s | 0.25 |
| True reading, then the dipstick left out, part way in, dropped back, or still held | 0.5 |
| True reading, dipstick seated and let go, rag not put down | 0.75 |
| All of it, arms not back home and still | 0.9 |
| All of it, both arms home and still for 1 s | 1.0 |

Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1.0. A real trial is checked from the video: count the pulls, check the rag
is drawn along the lower blade (over at least 3.2 cm of its lower 4 cm) between them, look for the gap under the cap after each push (a 3 mm feeler strip), time
the second pull's hold, and check the rag is back on the table.

### Self-QC (each case is a test in `test_task.py`)

Do nothing (0); first pull read and put back (0); wipe skipped, pull-reseat-pull-read (0); rag held 6 cm off the blade
(0); 3 cm wipe (0); a dab, rag held still on the blade (0); dabs at 4, 2.5 and 0.5 cm, lifted between (0); half
the length, 4 to 2 cm (0), and scrubbed back and forth over that half (0); touch at the top, lift, touch at the bottom
(0); half a stroke, the tip back in the tube, then the other half (0); a full stroke down, up, or stopping 6 mm short
of the tip (1.0); two half strokes covering the span (1.0); a full stroke with a 2-step contact gap (1.0); rag rubbed on the cap with the dipstick seated (0); half-seated reinsert before the second pull
(0); second pull not held still 1 s (0.25); dropped back in after the wipe (0); left out, left half in, still
held at the end (0.5); rag still in hand (0.75); arm not home (0.9); rag drawn down the still blade by the other arm
(1.0, a real wipe); half-dipped, then re-wiped and done right (1.0); history reset by a new trial (0).

## How it differs from nearby tasks

- #391 (check the pressure gauge before opening a filter housing) and #1241 (extinguisher pin and gauge check; faucet
  test) read an instrument once and then choose an action. Here nothing is chosen from the reading: the rule is the
  measuring procedure itself, a two-pull reading in which the first reading must be discarded.
- #256 (hand pump to the dial's pressure) is a repeated stroke against a gauge, with no wipe or reseat.
- #253 (clean a fiber connector's end face, then plug it in) is a single wipe-then-insert. Here the wipe sits between
  two pulls and is followed by a full reseat, a second pull held still, and a closing seat.
- Our kettle task (fill between the MIN and MAX lines) scores a level set by pouring; here the level is not changed,
  only read by the right procedure.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `yamkit.py` | Helper that spawns the engine as an articulation (the sliding dipstick) and the rag, gives the scorer the joint state and the grippers, and adds tilted demo waypoints. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline scorer checks (the self-QC cases above). No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/oil-dipstick-wipe-reinsert-read/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Check the oil level on the engine with the dipstick, then put everything back." \
      --scorer task_score --max-steps 600

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`. Videos: `inspect-robots video <log.json>`.
Scorer tests: `python tasks/oil-dipstick-wipe-reinsert-read/test_task.py` (prints `all pass`).

## The demo

1. Right arm: grips the dipstick's handle from above and pulls it until the tip is 3 cm clear of the mouth.
2. Left arm: picks up the rag, rolls its wrist about 75 degrees so the gripper reaches in from the left side (away
   from the right hand) holding the pad on edge, presses the pad's broad face onto the blade beside the tip and
   draws it up the blade over 3 s until its middle is about 4 cm above the tip, then takes it away. (Drawn downward,
   the tilted pad's leading edge dug into the blade and stalled after about 1 cm.)
3. Right arm: pushes the dipstick back in until the cap seats, pulls it again (tip 2 cm clear) and holds it still
   2.5 s (the reading),
   pushes it back in until it seats and lets go.
4. Left arm: puts the rag back down where it was. Both arms go home and hold.

## Runtime and seeds

- Demo: 541 control steps (27.1 s simulated).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

The dipstick is real size for a mower or generator engine (13 cm overall); a longer one cannot be pulled clear by
arms that lift about 22 cm above the table. It slides on the tube's axis even when out of the tube (a guide), so it
cannot be laid down or tilted; it falls back down the tube if let go. The oil is drawn, not simulated: the smear,
its removal by the rag and the true level are visuals driven by the scorer's state. The rag is a rigid pad.

## Real-world build

A small-engine bench model or a real mower or generator engine (drained, cold) with a push-in dipstick and fill tube,
oil at a level between ADD and FULL, a smear of oil wiped up the inside of the tube before each trial, and a folded
shop towel. Score from the video as described under Scoring.

## Licenses

All code here was written for this task; no third-party code or assets are included. Every shape is generated by
`yamkit.py`.
