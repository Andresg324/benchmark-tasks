# Swap a blown fuse in a hinged DIN-rail fuse holder for a new one taken from its carton, and latch the holder shut

A control enclosure stands on the bench with its door off, locked out and tested dead. On its DIN rail, three Eaton
Bussmann CHM1DU modular fuse holders (F1, F2, F3) each hold a 10 x 38 mm fuse in a hinged carrier. F2's fuse has blown.
Two opened Bussmann cartons of spare fuses lie on the bench, KTK-6 (6 A) and KTK-10 (10 A), with a parts tray. The
robot pulls F2's carrier open by its grip tab, lifts the blown fuse out of the carrier's clips and drops it in the
tray, takes a KTK-6 out of its carton, lays it into the carrier's clips, and swings the carrier shut until it latches,
without opening F1 or F3 and without putting a 10 A fuse in F2. The task file is `task.py` for the `yam_arena`
embodiment (`station` scene); `yamkit.py` spawns the modeled props, `carrierkit.py` gives the carrier its clips and
latch, and `guardkit.py` has the scoring's contact check.

## Spec

- Category: Impact on jobs. Electricians (O*NET 47-2111.00, about 821,000 US jobs in 2025:
  [BLS OOH](https://www.bls.gov/ooh/construction-and-extraction/electricians.htm)); O*NET task statement "Repair or
  replace wiring, equipment, or fixtures, using hand tools or power tools"
  ([O*NET 47-2111.00](https://www.onetonline.org/link/details/47-2111.00)). A blown control fuse is replaced from
  stock as routine troubleshooting in any control panel; modern panels use finger-safe modular holders like the
  CHM, opened by hand without a tool.
- What the task tests: operating a hinged, latching part by its small grip tab (pull it open, swing it shut until it
  latches) in a row of identical holders that must stay shut; taking a single fuse out of an opened retail carton
  where the other fuses lie loose beside it; laying a fuse into a carrier's contact clips so both ends seat; and the
  two-handed sequence of the swap (one hand works the carrier, the other the fuses). No fuse-puller tool is used: the
  holder is finger-safe and its carrier presents the fuse for a direct grip.
- The new fuse starts in its own carton, as it comes from stock: a partly used box of 10 with its tuck lid folded
  back. The rating to use is on the instruction (an electrician reads it off the holder's legend); the 10 A carton
  beside it is the same size and make.

| Part | Named product | Dimensions used | Source |
|---|---|---|---|
| Fuse holders | Eaton Bussmann CHM1DU, 1-pole modular holder for 10 x 38 mm fuses, 35 mm DIN rail | 17.5 mm module (body 17.3 mm), 73.9 mm tall, 83.2 mm deep off the rail, 50 g (0.11 lb) | Eaton Technical Data 720147, "10 x 38 mm Modular fuse holders", dimensional data (the same 2.909 x 0.689 x 3.275 in on distributor listings; Eaton's catalogue page swaps height and depth); weight from the catalogue page. Carrier height (60 mm), hinge position and opening stop are not dimensioned: read from product photos and assumed |
| Fuses | Eaton Bussmann KTK-6 and KTK-10, fast-acting 600 V midget fuses, 0.41 x 1.5 in | 10.3 x 38.1 mm; ferrules 9.5 mm long (typical, not published); 9 g (0.02 lb) | Eaton KTK product pages; weight from distributor listings (Elliott Electric, KTK10) |
| Cartons | Bussmann box of 10 (standard package quantity 10) | 108 x 43 x 13 mm outside, 1 mm card (assumed: ten fuses side by side; the carton is not dimensioned by Eaton) | distributor listings give the package quantity |
| Enclosure | 20 x 16 x 6 in (508 x 406 x 152 mm) steel wall-mount enclosure, door off, back pan on standoffs, 35 mm top-hat DIN rail | as named | standard sizes |
| Tray | plastic parts tray | 150 x 100 x 30 mm | any |

## Run a trial

Scripted demo:

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/swap-blown-fuse-in-din-holder-from-new-box/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "The panel is locked out and tested dead. F2's fuse has blown: replace it with a new 6 A fuse from its box, put the blown fuse in the parts tray, and close F2." \
      --scorer task_score --max-steps 2400

Model run (Dream-1): the same command with `--policy dream`. Videos: `inspect-robots video <log.json>`. Scorer, law
and adversarial tests (no simulator): `python tasks/swap-blown-fuse-in-din-holder-from-new-box/test_task.py` (prints
`all pass`).

## Scoring

`task_score` reports `score()`; the run ends early on `success()`, a full score held for 1 s.

| Outcome | Score |
|---|---|
| F1 or F3 open 5 degrees or more, for two control steps in a row, at any time | 0 for the trial |
| A 10 A fuse seated in F2's clips (two steps in a row), at any time, carrier open or shut | 0 for the trial |
| The blown fuse seated in F2's clips again after it had been out of them (two steps in a row) | 0 for the trial |
| A 6 A fuse seated in F2, F2 shut, the blown fuse in the tray, released (no gripper in contact with F2's carrier or either fuse) and both fuses still | 1.0 |
| A 6 A fuse seated in F2 and the blown fuse out, the rest not done (F2 open, still held, or the blown fuse elsewhere) | 0.75 |
| The blown fuse out of F2 and a 6 A fuse out of its carton (held, or dropped anywhere) | 0.5 |
| The blown fuse out of F2 with no new fuse out of its carton, or F2 open 30 degrees or more | 0.25 |
| Nothing done, or F2 opened and shut again with nothing swapped | 0 |

- The three zero rules are latched the step they occur and override every other line for the rest of the trial:
  milestones reached before or after them do not count (`score()` checks them first, every step).
- Seated: both ferrules within 1 mm of the carrier's clip floor and on their clips (within 1 mm sideways, 1.7 mm along
  the axis); a fuse turned end for end seats the same. Out: a ferrule lifted more than 2.5 mm off its clip floor (past
  the clip fingers), or 3 mm off sideways or 5 mm along.
- Shut: within 2 degrees of flush with the holder's face. Open: read from the carrier's angle on its hinge.
- Released: the gripper's own shape (palm and fingers at their present opening, boxes fitted to the YAM's collision
  meshes) not within 2 mm of F2's carrier, the new fuse or the blown fuse. Still: under 2 cm/s.
- Not scored: which hand does what, the order of the steps, where the arms and the cartons end up.

Real trial, with the same rules. Two fixed cameras record the whole trial: an overhead camera seeing the bench
(cartons, tray) and the enclosure's opening, and a side camera level with the rail, looking along the holders' faces.
Each check works for any final state the scorer accepts:

| What | Tool and method | Works for |
|---|---|---|
| F1, F2, F3 shut or open | a steel straightedge laid across the three holders' front faces after the trial, and a 2 mm feeler (or a 2 mm card): a carrier that takes the feeler under the straightedge at its tab is not shut (2 degrees is 2.1 mm at the 60 mm carrier's top; 5 degrees is 5.2 mm, also plain on the side video) | any carrier angle |
| When a neighbor opened, or a wrong fuse went in | the side video: the first frame where F1 or F3 stands proud, or where F2's open carrier holds the striped or a KTK-10 fuse down in its clips | any order of events |
| Which fuse is in F2, and seated | after the photos, F2 is opened by hand by its tab (the fuse is not touched); the catalogue number printed on the fuse is read (KTK-6, KTK-10, or the blown fuse's black stripe); seated means both ferrules sit down between their clip's fingers: each ferrule pressed lightly toward the carrier with a fingertip does not move further in (a ferrule riding a finger snaps down) | any fuse, either way round |
| Blown fuse in the tray | the final overhead photo: its middle (the stripe) inside the tray's walls, anywhere, any orientation | any position in the tray |
| A 6 A fuse out of its carton | the final overhead photo: the fuse's middle outside the carton's walls | anywhere on the bench, the tray or the panel |
| Released and still | the last second of the overhead and side video: no finger touching F2's carrier or either fuse, nothing moving | any arm position |

## Forces

- Everything in the task is light. The fuses weigh 9 g, the cartons about 12 g. The carrier's latch and clip forces
  are not published by Eaton; the sim uses deliberately light values (latch: 0.05 N*m at 6 degrees, about 1 N pulled
  at the tab; clips: 0.8 N per ferrule, toward the floor and the clip's middle), which the scoring does not depend on: every score line is a position, an angle
  or an identity. A real CHM's snap is a fingertip action.
- The YAM (Damiao DM-J4340 on joints 1-3, 9 N*m rated; DM-J4310 on joints 4-6, 3 N*m rated; grip capped at 50 N by
  i2rt's driver) gives about 11 N continuous horizontally at the hand in its weakest direction (about 17 N typical,
  28 N peak), well above anything here.

## Runtime and seeds

The scripted demo scores 1.0 in 944 steps (47 s at 20 Hz; two clean runs of the same task.py, both 1.0 in 944
steps). Dream-1 (`--policy dream`) scores 0 in the full 2,400 steps: the right hand moves to the parts tray and
nudges it, the left hand stays near its start, and no carrier is opened. A run is limited to 2,400 steps (2 minutes). One
fixed layout (`-E object_jitter=0`).

## The demo

The left arm, fingertips pointing into the panel, pinches F2's red grip tab between its fingertips, pulls the
carrier 35 degrees open along its arc and lets go; the carrier drops to its stop, lying level. The right arm, its hand
pitched 10 degrees, grips the blown fuse across its body from above, draws it straight up out of the carrier's clips,
carries it out of the enclosure and drops it in the tray. It then picks the right-most fuse out of the KTK-6 carton,
carries it to the open carrier, lays it into the clips, lets go and presses it down into both clips with its closed
fingertips. The left arm pinches the open carrier's end the same way it pinched the tab (fingertips into the panel), swings it
back up along its arc until it is shut and the latch has caught, lets go, and presses the carrier's face home with
its closed fingertips. Both arms end clear.

## Layout choices and assumptions

- Spacing: CHM holders are usually ganged side by side on the rail, 17.5 mm apart (Technical Data 720147 shows
  toolless ganging and the JV-L ganging kit). Here they stand 45 mm apart, with 27.5 mm of bare rail between
  neighbors. The YAM pinches F2's tab between 6.4 mm fingertips that stand beside the 17.3 mm tab; with ganged
  holders (17.5 mm) or a half-module spacer (26.5 mm) those fingertips overlap the neighbors' tabs (checked against
  the gripper's collision boxes along the demo's path), and a person pulls a ganged tab with one fingertip instead.
  The gap is a layout choice made for the robot's hand.
- Carrier: its height (60 mm of the 73.9 mm holder), hinge at its bottom front edge and opening stop (lying level,
  90 degrees) are assumed: Technical Data 720147 draws the carrier swung out by its top but does not dimension it.
  The latch (a spring within 6 degrees of shut) and the clips (0.8 N per ferrule, toward the clip floor and middle)
  are deliberately light, assumed values; Eaton publishes neither.
- Carton: Bussmann sells KTK and other midget fuses in boxes of 10; the carton is not dimensioned by Eaton, so it
  is modeled as ten fuses side by side (108 x 43 x 13 mm outside, 1 mm card). Three fuses are left in it, lying
  where they rolled, apart enough for a fingertip each side of one.
- The blown fuse is let go about 2 cm above the tray's floor and may roll in the tray; full credit waits until it
  is still.

## Real-world build and reset

Positions are measured from the midpoint between the two arm bases, x forward, y to the robot's left; tolerance
+-2 mm unless stated, set with a tape measure and marked with tape on the bench.

- Enclosure: a 20 x 16 x 6 in hinged steel wall-mount enclosure (508 x 406 x 152 mm), door removed, standing upright
  on its bottom side on the bench, opening toward the robot, front edge at x 27.3 cm, centered on y 0
  (tape outline round its footprint). Back pan on its standoffs, its face 40.8 cm from the arm-base line.
- Rail and holders: a horizontal 35 x 7.5 mm top-hat DIN rail on the back pan, its middle 15.0 cm above the bench.
  Three CHM1DU holders clipped on it, F1, F2, F3 from the robot's left, centered at y 4.5, 0 and -4.5 cm (45 mm
  apart, 27.5 mm of bare rail between neighbors; a 27.5 mm wooden spacer laid on the rail between them sets the gaps
  and is removed). One, two and three marker ticks on the tabs. Six terminal blocks on the rail further right, from
  y -12 cm. F1 and F3 hold any 10 x 38 mm fuse; F2 holds a KTK fuse with a black marker stripe round its middle (the
  blown one).
- Cartons: an opened Bussmann carton of KTK-6 (6 A) centered at x 16.5, y -12 cm and one of KTK-10 (10 A) at x 16.5,
  y 12 cm, each with its long side across the bench, lid folded back flat behind it (away from the robot), on a tape
  outline. Inside each, pencil marks on the floor at the fuses' places: KTK-6 at -4.0, -1.2 and +3.0 cm along the
  carton from its middle (+ toward the robot's left), KTK-10 at -3.0 and +1.0 cm; each fuse lies on its mark, pointing
  at the panel.
- Tray: 150 x 100 x 30 mm, centered at x 20, y -30 cm, long side pointing at the panel, on a tape outline.
- Arms: both at their ready pose.
- Lockout: the panel is never connected to power for the trial.
- Reset after a trial: open F2 by its tab, lift out whatever fuse is in it, put the striped (blown) fuse back in its
  clips (both ferrules pressed down onto the clip floor) and swing F2 shut until it latches; check all three carriers
  flush with the straightedge; put every fuse back on its pencil mark in its own carton (by its printed catalogue
  number); put the cartons and tray back on their outlines; arms to ready.
