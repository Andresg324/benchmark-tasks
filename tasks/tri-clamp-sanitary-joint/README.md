# tri-clamp-sanitary-joint

Make up a 1.5-inch sanitary (tri-clamp) joint on a vertical stainless line. The clamp has gone on cocked, its hinge
side riding on the flange rims, and its wing nut is 1.5 turns loose. Press the clamp square so its channel takes both
ferrule flanges and the gasket all round, then turn the wing nut hand tight: half a turn to 1.5 turns past contact.
Drawing the nut up with the clamp cocked scores 0 for good.

## Run

From the repository root, on a yam_arena install (Isaac Lab-Arena, inspect-robots):

    # scripted demo (scores 1.0)
    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/tri-clamp-sanitary-joint/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Tighten the clamp on the pipe joint." \
      --scorer task_score --max-steps 1300 --log-dir ~/yam-eval
    # Dream-1: the same with --policy dream --max-steps 2400

Offline scorer tests (no simulator): `python test_task.py` prints "all pass". `TRICLAMP_DEBUG=<file>` appends the
clamp's tilt, the nut's angle past contact and the grip state per step to that file.

- Runtime: the demo succeeds at control step 1,258 (63 s at 20 Hz); a 2,400-step run takes about 12 minutes of wall time
  on one L40S. No seeds: the layout is fixed (`object_jitter=0`); the demo is deterministic.
- Files: `task.py` (scene, scorer, demo), `twistkit.py` (the wing nut's thread), `yamkit.py` (modeled props,
  mechanisms and gripper states). Both kits hook yam_arena's scene builder in memory; nothing on disk changes.

## Spec

Frame: cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0.

| Object | Description | Position |
|---|---|---|
| Stand | painted steel base plate 17 x 10 x 1.2 cm; 2.5 cm square post 11.1 cm tall with a 9 x 2 x 1 cm arm strapped round the upper spool | plate centered (29.5, 4.5); post at (36.5, 4.5) |
| Line | 1.5-inch sanitary 316L tube, 38.1 mm OD, 1.65 mm wall, vertical | axis (26, 4.5) |
| Lower ferrule | 50.5 mm flange, 2.85 mm thick at the rim, 20-degree tapered back, on a tube stub from the plate | face at z 8.95 |
| Upper ferrule | the same, ending a short spool held by the bracket; spool cut open at the top | face at z 9.05, top z 13.0 |
| Gasket | white PTFE tri-clamp gasket, 50.5 mm OD, 35 mm bore | centered between the faces |
| Clamp | single-hinge 1.5-inch tri-clamp, cast stainless: ring 62 mm OD, 14 mm tall; channel 53 mm across, 9 mm tall inside; lip bore 44 mm; hinge on the robot's side, two 6 mm ears 3 mm apart (at contact) on the far side | hinge knuckle at x 22.55; tilted 6 degrees about the swing bolt's axis, hinge side 8.7 mm high (lower lip on the upper ferrule's rim) |
| Swing bolt | M8 x 1.25, pinned in the -y ear, through the slotted +y ear, along y | axis x 31.0, z 9.0 |
| Wing nut | brass, 36 mm across the wings, wings 10 mm thick, hub 13 mm across and 10 mm long; wings flat | on the bolt's +y end, 1.5 turns short of the ear (hub center y 5.94; at contact y 5.75) |

Scoring (`score()` in task.py, evaluated at the end of the run, with the run's history):

| Outcome | Score |
|---|---|
| The nut ever more than 10 degrees past contact while the clamp was tilted 0.8 degrees or more at that step or the one before (drawn up cocked; irreversible) | 0 |
| Clamp not square at the end (tilt 0.8 degrees or more) | 0 |
| A gripper still holding the nut, or any tool point within 6 cm of the joint's center or 4 cm of the nut | 0 |
| Nut not past contact (10 degrees or less) | 0 |
| More than 90 degrees of the tightening done without a gripper holding the nut and turning with it | 0 |
| Nut ever more than 1.5 turns past contact (overtight, even if backed off) | 0.25 |
| Nut past contact but under half a turn (loose) | 0.5 |
| Square, nut 0.5 to 1.5 turns past contact, let go, arms not home | 0.9 |
| All of that with both arms home and still (tool points within 3 cm of their start) | 1.0 |

Thresholds and reasons: 0.8 degrees of tilt about the swing bolt puts the hinge side 1.05 mm off square, inside
the channel's 1.15 mm of axial play over the flange pair (9.0 mm channel, two 2.85 mm flanges and the gasket), so
both flanges and the gasket are in the channel all round; 10 degrees past contact is 0.035 mm of bolt travel, the
noise band at contact; half a turn (0.6 mm of bolt travel) is the least that draws the ferrules onto the gasket, less
seeps under line pressure; 1.5 turns is firm hand tight for a wing-nut clamp, beyond it the gasket is squeezed into
the bore (a sanitary defect) and stays squeezed; 90 degrees of unheld turning allows for slip at the start and end of
each grip but not a swipe; 6 cm from the joint is the clamp's 3.1 cm radius plus a finger's reach.

## Real-world build

Off the shelf: a 1.5-inch tri-clamp ferrule pair (14MPW, 50.5 mm flange) welded to short lengths of 1.5-inch 316L
tube, a 1.5-inch PTFE tri-clamp gasket, a 1.5-inch single-hinge clamp with a wing nut (13MHH style), and a pipe stand
(base plate, square post and a U-bolt bracket) holding the lower stub upright and the upper spool above it with the
joint 9 cm above the table. Set the clamp cocked by closing it with its hinge side lifted onto the upper ferrule's rim
and the nut 1.5 turns short of the ear (paint marks on the nut and bolt).

A real trial is checked with a feeler gauge between the clamp's lips and the flange back faces at the hinge and at
the ears (equal within 1 mm means square), the paint mark on the wing nut counted in half turns from the contact mark,
and the top video for whether the clamp was square before the nut reached the ear.

Reset: back the nut off to 1.5 turns short of the ear, lift the hinge side back onto the rim, and replace the gasket if
its bead is crushed or it bulges into the bore.
