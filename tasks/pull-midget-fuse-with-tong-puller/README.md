# Pull a midget fuse from its open-clip block with a tong puller held across it, and put it in the parts tray

A control enclosure stands on the bench with its door off, locked out and tested dead. On its back pan a three-pole
Bussmann BMM603-3SQ midget fuse block holds three Bussmann FNM 13/32 x 1-1/2 in (10.3 x 38.1 mm) fuses upright in open
spring clips, 20 mm apart. The robot picks up an Ideal 34-015 nylon tong fuse puller from the bench, holds it pointing
into the panel, square to the fuse, closes its jaws round the middle fuse's body between the clips, draws the fuse out
of both clips without disturbing its neighbors, and lets it go in a parts tray. The task file is `task.py` for the
`yam_arena` embodiment (`station` scene); `yamkit.py` spawns the modeled props, `clipkit.py` gives the clips their
spring, and `guardkit.py` has the scoring's contact check.

## Spec

- Category: Impact on jobs. Electricians (O*NET 47-2111.00, about 821,000 US jobs in 2025:
  [BLS OOH](https://www.bls.gov/ooh/construction-and-extraction/electricians.htm)); O*NET task statements "Repair or
  replace wiring, equipment, or fixtures, using hand tools or power tools" and "Diagnose malfunctioning systems,
  apparatus, or components, using test equipment and hand tools to locate the cause of a breakdown and correct the
  problem" ([O*NET 47-2111.00](https://www.onetonline.org/link/details/47-2111.00)). Pulling a control fuse to test
  or replace it is routine troubleshooting in any control panel.
- The puller is held square to the fuse. A tong puller's jaws are shaped to close round a cartridge fuse's body from
  the sides, so the tongs point into the panel, perpendicular to the fuse, and the jaws grip the body between the two
  clips, not a ferrule. The fuse is then drawn straight out toward the person (or one end first), carried in the jaws
  and put down in a tray, not left loose in the panel.
- Why a puller, though the panel is dead: the fuses are 9.7 mm apart tube to tube (20 mm pole pitch), with barriers
  beside the clips and terminal housings beyond each end, so fingers cannot get round one; the YAM's 10 mm pads
  cannot either. The puller's 3 mm jaws can.

| Part | Named product | Dimensions used | Source |
|---|---|---|---|
| Fuses | Bussmann FNM (time-delay, 250 Vac, 13/32 x 1-1/2 in midget) | 10.3 x 38.1 mm; ferrules 9.5 mm long (typical, not published); about 9 g (typical, not published) | Eaton Bussmann FNM data sheet 2028; size from the BMM block's fuse list (data sheet 10241) |
| Fuse block | Bussmann BMM603-3SQ, 3-pole, 10x38 (13/32 x 1-1/2 in) | 77.0 mm long, 20.0 mm per pole (60 mm), 33.0 mm off the rail; tin-plated copper clips | Eaton Bussmann Technical Data 10241, "BCM and BMM modular Class CC and supplemental ferrule fuse blocks" (dimension drawing; copy at https://prochiller.com/wp-content/uploads/2023/10/Bus_Ele_DS_1104_BM_Series-477773.pdf) |
| Puller | Ideal 34-015 small fuse puller | 5 in (127 mm), yellow high-impact nylon, for 1/4-1/2 in fuses (13/32 x 1-1/2 in listed); notched handles and finger stops | Ideal product page [34-015](https://www.idealind.com/eu/en/shop/34-015.html) and distributor listings; jaw and arm sections are inferred (no drawing is published) |
| Enclosure | 20 x 16 x 6 in (508 x 406 x 152 mm) steel wall-mount enclosure, door off, back pan on standoffs, 35 mm top-hat DIN rail | as named | standard sizes |
| Tray | plastic parts tray | 150 x 100 x 30 mm | any |

## Run a trial

Scripted demo:

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/pull-midget-fuse-with-tong-puller/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "The panel is locked out and tested dead. Use the fuse puller to take out the middle fuse, F2, and put it in the parts tray." \
      --scorer task_score --max-steps 2400

Model run (Dream-1): the same command with `--policy dream`. Videos: `inspect-robots video <log.json>`. Scorer and
clip-law tests (no simulator): `python tasks/pull-midget-fuse-with-tong-puller/test_task.py` (prints `all pass`).

## Scoring

`task_score` reports `score()`; the run ends early on `success()`, a full score held for 1 s.

| Outcome | Score |
|---|---|
| F2 out of both clips, its middle inside the tray, released (a release check: neither jaw nor any gripper in contact with it, contact meaning within 2 mm), still (under 2 cm/s), and the puller put down: lying flat on the bench outside the enclosure and the tray, no gripper touching it (a tool still in hand means the job is not finished) | 1.0 |
| F2 in the tray but still held in the jaws or by a hand, or still moving, or the puller still in hand or not put down | 0.75 |
| F2 out of both clips anywhere else (still held, or dropped on the bench) | 0.5 |
| F2 dropped loose inside the enclosure (it has to be found before power goes back on) | 0.25 |
| One end of F2 out of its clip, the fuse cocked in the block | 0.25 |
| F2 still in both clips, or nothing done | 0 |
| F1 or F3 out of either clip at any time (two control steps in a row) | 0 for the trial |
| F2 leaves its second clip without the puller's jaws closed round its body (at that step or the one before), at any time | at most 0.5 for the trial, even if it is later put back and pulled properly |

- Out of a clip: a ferrule's center more than 2.5 mm out of its seat (past the fingers' lips), or 3 mm off the clip
  sideways or 5 mm along the fuse.
- Jaws closed round the body: both jaw faces within 1 mm of the tube's surface, on opposite sides of it, and between
  the ferrules (the 19 mm body plus 3 mm).
- Holding (gripper): the gripper's own shape (palm and fingers at their present opening, boxes fitted to the YAM's
  collision meshes) within 2 mm of the fuse.
- Not scored: how the fuse comes out (one end first or straight), where on the bench the puller is laid, where the
  arms end up.
- Rocking one end out first needs only one clip's grip at a time, but the 10 mm jaws fill most of the 19 mm opening
  between the barriers, so they grip the body at its middle; drawn from there both clips let go together. The
  demo draws straight (about 12 N); a policy may rock or tilt the fuse as it likes.

Real trial, with the same rules, checked the same way wherever the scorer allows things to end up:

| Check | How, with what | Works for |
|---|---|---|
| F2 in the tray | final overhead photo: F2's middle inside the tray's walls | any position or way up in the tray (lying or standing); a fuse balanced on the rim with its middle outside does not count |
| F2 released and still | last second of the overhead video: nothing touches it and it does not move; if a jaw or finger is unclear on the photo, the tester lifts the puller or hand straight up and F2 must not move with it | any place in the tray |
| Puller put down | final photo: lying flat on the bench with both arms on the bench, outside the enclosure and the tray, no hand on it | anywhere on the bench, any way round |
| F1 and F3 seated | after the run, a caliper's depth rod from the front of the barrier beside each ferrule to the ferrule's front: more than 2.6 mm proud means out of its clip (seated reads about 0.2 mm); the overhead video shows when it happened | any of the four ferrules, whatever moved it |
| Puller used | the robot's wrist-camera frames at the moment F2's second ferrule leaves its clip: the puller's two jaws either side of F2's tube, between the ferrules | the moment is found on the video, wherever the fuse goes next |
| F2 dropped into the panel, or cocked | final photo into the enclosure; a cocked fuse has one ferrule visibly out of its clip (caliper as above) | any place inside the enclosure |

## Forces

- Clip grip: Bussmann publishes no pull-out force for these clips. The sim's clip presses each ferrule onto its base
  with 6 N while it is in the clip (assumed), so a straight draw takes about 12 N (both clips at once), one end at a
  time about 6 N. The jaws carry it by friction: about 15 N each on the body (below).
- The YAM (Damiao DM-J4340 on joints 1-3, 9 N*m rated; DM-J4310 on joints 4-6, 3 N*m rated; grip capped at 50 N by
  i2rt's driver) gives about 11 N continuous horizontally at the hand in its weakest direction (about 17 N typical,
  28 N peak), so a straight draw of 12 N is within its reach and pulling one end first is well within it.
- Squeezed by the 50 N grip 42 mm from where the arms flex, the jaws 101 mm from it close on the body with about
  15 N each, after the tong's own spring (about 5.5 N at each jaw to close it on a 10 mm fuse; the nylon's stiffness
  is inferred).

## Runtime and seeds

The scripted demo scores 1.0 in 596 steps (about 30 s at 20 Hz); a run is limited to 2,400 steps (2 minutes).
Dream-1 (`--policy dream`, same task.py) scores 0 in 2,400 steps: the arms stay near their start, the right hand
moves toward the tray and the puller, and the puller is never picked up; no fuse is touched. One
fixed layout (`-E object_jitter=0`): the fuses must start seated in their clips.

## The demo

The right arm, its hand tilted 10 degrees (fingertips forward) so it can reach the panel, pinches the puller's
notched handles 50 mm from the loop end (its balance point) where it lies, lifting it with the jaws still about 14 mm
open. It brings the puller level, pointing into the panel, slides the open jaws either side of F2's body, squeezes,
and draws the fuse straight out of both clips toward the robot. It carries the fuse out of the enclosure to the tray,
sets it down standing on the tray's floor, eases the squeeze so the jaws spring open, lifts the puller clear, lays
it back flat on its spot on the bench as it was found, lets go and moves away. The left arm stays at its start. The top camera looks over the robot's shoulder into the
enclosure, so while the jaws close on F2 and draw it out the right wrist and gripper are between it and the block; the
right wrist camera shows that part.

## Real-world build

- Layout: positions are measured from the midpoint between the arm bases (task.json `setup`), set with a tape measure
  from marks on the bench to within 2 mm.
- Enclosure: a 20 x 16 x 6 in steel wall-mount enclosure bolted upright to the bench through its mounting feet (a
  bench stand for a panel normally hung on a wall), front at x 27.3, door off, back pan on its standoffs. It does not
  move between trials.
- Rail and block: a horizontal 35 mm top-hat DIN rail centered across the pan 15 cm above the bench; on it, centered
  at y 0 against end stops, a BMM603-3SQ block with three FNM fuses (any amp rating), F1 to F3 from the left, marked on
  the block's marker strip below each pole.
- Puller: lying flat on a tape outline on the bench, loop end at x 15, y -10, jaw tips toward the panel. Tray: loose on
  a tape outline centered at x 20.5, y -26.
- Reset after each trial: push F2 back into its clips by hand until both ferrules seat; check all three with the
  caliper's depth rod (each ferrule front no more than 0.5 mm proud of the barrier fronts); puller and tray back on
  their outlines; tray emptied.
- Tools: a tape measure, a caliper with a depth rod, the robot's overhead and wrist cameras.
- Lockout: the panel is never connected to power for the trial.
- The puller: Ideal sells the 34-015 as 5 in (127 mm) overall; the sim's tongs measure 122 mm from the loop to the jaw
  tips. The loop end's shape is inferred, since Ideal publishes no drawing.
