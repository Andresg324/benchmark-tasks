# circlip-pliers-into-groove

Fit an external retaining ring (circlip) into its groove on a shaft with circlip pliers: spread it just enough to
clear the shaft, slide it down to the groove, let it close, turn it in the groove to check it is seated, and put the
pliers down. A ring spread past its elastic limit is scrap; one let go short of the groove or tilted is not seated.

## Run

From the repository root, on a yam_arena install (Isaac Lab-Arena, inspect-robots):

    # scripted demo (scores 1.0)
    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/circlip-pliers-into-groove/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Fit the retaining ring into the groove on the shaft with the circlip pliers, then put the pliers down." \
      --scorer task_score --max-steps 1500 --log-dir ~/yam-eval
    # Dream-1: the same with --policy dream --max-steps 2400

Offline scorer tests (no simulator): `python test_task.py` prints "all pass".

- Runtime: the demo finishes in 625 control steps (31 s at 20 Hz); a 2,400-step run takes about 10 minutes
  of wall time on one L40S. No seeds: the layout is fixed (`object_jitter=0`); the demo is deterministic.
- Files: `task.py` (scene, scorer, demo), `ringkit.py` (the ring-on-a-shaft model), `yamkit.py` (modeled props,
  mechanisms and gripper states). Both kits hook yam_arena's scene builder in memory; nothing on disk changes.

## Spec

Frame: cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0.

| Object | Description | Position |
|---|---|---|
| Fixture | cast-iron base plate 12 x 12 x 1.2 cm; bearing housing 6.4 cm across, 3.8 cm tall | centered (36, -10) |
| Shaft | steel, 40 mm, upright; end face 10.5 cm up with a 1.5 mm chamfer | axis (36, -10) |
| Gear | steel spur gear, 56 mm, 30 teeth, 1.6 cm thick, on the shaft | z 5.0 to 6.6 |
| Groove | 1.85 mm wide, 37.5 mm bottom diameter (DIN 471 for 40 mm); lower wall flush with the gear face | z 6.6 to 6.78 |
| Ring | DIN 471 A40 external retaining ring, spring steel, yellow zinc-chromate plated, 1.75 mm thick, free bore 36.5 mm, lug holes 12 mm apart | flat on the shaft's end face, lugs toward the robot |
| Pliers | external circlip pliers with 90-degree bent tips, 13 cm long, red handle grips 8 x 1.0 x 1.2 cm, handles 4.0 cm apart at rest, tips 1.7 cm long, 12 mm apart at rest; squeezing the handles 1 mm spreads the tips 1 mm (handle stop at 18 mm); 130 g | lying on the rest, grip point (26.1, -22, 3.6), tips toward +x |
| Rest | black rubber block, 8 x 3 x 3 cm | center (25.6, -22, 1.5) |

The ring's bore grows by (lug spacing - 12 mm) / pi: it clears the shaft from 40.2 mm (lug spacing 23.6 mm) and
reaches its elastic limit at 41.0 mm (26.1 mm).

Scoring (`score()` in task.py, evaluated at the end of the run):

| Outcome | Score |
|---|---|
| Ring ever spread past 41.0 mm bore, or its ends ever twisted more than 10 degrees out of plane for 2 control steps in a row | 0 |
| Ring not seated (short of the groove, cocked, still spread or still in the pliers) | 0 |
| Pliers still held (a closed gripper within 3 cm of them, handle butt to tips), still in the ring, moving, off the bench's footprint, or not between 1 cm below and 5 cm above its top | 0 |
| Seated and pliers down, but its net turn in the groove under 15 degrees | 0.5 |
| Seated, turned at least 15 degrees in the groove, pliers down, arms not home | 0.9 |
| All of that with both arms home and still | 1.0 |

Thresholds and reasons: 41.0 mm, the usual maximum installation spread for a 40 mm DIN 471 ring (1 mm, 2.5%, over
the shaft), past which spring steel takes a set; 0.2 mm diametral clearance to slide over the shaft; 4 degrees of
tilt at release, at which a 49 mm-wide ring's far side sits 3.4 mm off the groove plane, nearly twice the groove
width; 10 degrees of twist, 2.1 mm across the lug spacing (ring makers say never twist a retaining ring); 15 degrees
of net turn in the groove, 5 mm at the ring's rim, the check that it is free all round. The turn is net: the span
between the smallest and largest angle the ring reached while closed in the groove, so a back-and-forth wiggle or
jitter counts only its width. A ring that closes into the groove (however slowly) while tilted more than 4 degrees
lands cocked. The pliers are put down only lying on the bench (grip point no more than 1 cm below its top, over its
footprint), not on the floor.

## Real-world build

All parts are off the shelf: a 40 mm ground steel shaft (or a 40 mm bolt-on shaft stub) turned with a DIN 471
groove 1.85 mm wide to 37.5 mm, a 40 mm-bore spur gear (or a 40 mm-bore collar) as the part it retains, a DIN 471
A40 ring, a pair of 90-degree external circlip pliers for 19 to 60 mm rings, and a rubber block. Clamp the shaft
upright in a bench vise or a V-block on a base plate so the groove is 6.6 cm above the table. The robot's gripper
closes on the pliers' handles; check that the gripper can squeeze them through their working range (handle gap
40 mm to 27 mm) and fit the pliers with stiffer grips if needed.

A real trial is checked from the top camera and by hand: the largest lug-hole spacing during the trial (against a
mm scale on the base plate; 26.1 mm = 41.0 mm bore), the ring fully in the groove all round (no part of it above
the groove's upper edge, checked with a straightedge on the gear face at four points) and closed on the groove bottom
(it turns freely but does not rock), a paint mark on the ring turned at least 15 degrees (5 mm at the rim) against a
mark on the shaft, and the pliers lying on the bench or rest.

Reset: take the ring off with the pliers (spread no more than 41 mm), check its free bore is still 36.5 mm with
calipers (replace it if it measures over 36.6 mm), lay it on the shaft's end face with the lugs toward the robot,
and put the pliers back on the rest.
