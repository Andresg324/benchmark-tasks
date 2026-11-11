# o-ring-into-groove

Fit an O-ring into its groove on a hydraulic O-ring boss (ORB) plug: put two fingertips into the ring, stretch it
just enough to pass over the thread, carry it down square to the groove at the hex, and draw the fingers out so it
closes into the groove. A ring stretched past 50% is scrap; one pushed down the thread unstretched rolls and is left
twisted; one let go tilted lands with a side on the thread.

## Run

From the repository root, on a yam_arena install (Isaac Lab-Arena, inspect-robots):

    # scripted demo (scores 1.0)
    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/o-ring-into-groove/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Fit the O-ring into the groove at the base of the plug's thread." \
      --scorer task_score --max-steps 1500 --log-dir ~/yam-eval
    # Dream-1: the same with --policy dream --max-steps 2400

Offline scorer tests (no simulator): `python test_task.py` prints "all pass".

- Runtime: the demo finishes in 339 control steps (17 s at 20 Hz); a 2,400-step run takes about 10 minutes of wall time on
  one L40S. No seeds: the layout is fixed (`object_jitter=0`); the demo is deterministic.
- Files: `task.py` (scene, scorer, demo), `ringkit.py` (the ring-on-a-shaft model; this copy adds `FingerTips`, a
  gripper's two fingertips stretching an elastomer ring, and rolling when it is pushed instead of stretched),
  `yamkit.py` (modeled props, mechanisms and gripper states). Both kits hook yam_arena's scene builder in memory;
  nothing on disk changes.

## Spec

Frame: cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0.

| Object | Description | Position |
|---|---|---|
| Holder | blue anodized aluminum block, 9 x 9 x 4 cm, fixed | centered (34, -10) |
| Plug | zinc-plated steel SAE J1926 -24 O-ring boss hex plug, standing hex-down: 2.25 inch hex (57.15 mm across flats) | axis (34, -10), hex z 4.0 to 5.4 |
| Groove | the undercut between hex and thread, 3.6 mm long, 44.2 mm diameter, lower wall the hex face | z 5.4 to 5.76 |
| Thread | 1.875-12 UN, 47.6 mm across the crests, flat end face with a 1.5 mm chamfer | z 5.76 to 7.66 |
| O-ring | AS568-924, bright blue 90-durometer fluorocarbon (FKM), 43.7 mm inside diameter, 3.0 mm section | flat and centered on the end face, z 7.66 to 7.96 |

The ring's stretch is its inner circumference: the loop round the two fingertips' outer faces and, once both
fingers are outside the thread, round the thread as well. With the YAM fingertips (4 mm thick, 14 mm wide) a pad
gap of 57 mm (4.7 mm clear of the thread each side) stretches it 28.7% (56.2 mm); the 50% limit is reached at a pad
gap of about 74 mm, and fully open (94 mm) is 77%.

Scoring (`score()` in task.py, evaluated at the end of the run):

| Outcome | Score |
|---|---|
| Ring ever stretched past 65.5 mm inside diameter (50%) | 0 |
| Ring not seated: on the end face or the thread, landed tilted (cocked), or a fingertip still in it or pressing on it | 0 |
| Seated, but more than 90 degrees of roll left in its section (pushed down the thread unstretched) | 0 |
| Seated, untwisted, never over-stretched, but not both arms back home and still | 0.9 |
| All of that, and both arms back home and still (tool points within 3 cm of their start) | 1.0 |

Possible scores: 0, 0.9, 1.0. There is no partial credit short of seated: a ring anywhere else than in the groove
is either a leak (twisted, thinned) or cut on assembly (on the thread), so a fitter does it again.

Thresholds and reasons: 65.5 mm (50% stretch), seal makers' installation limit for O-rings, half the 100% minimum
ultimate elongation of the 90-durometer FKM (ASTM D2000 M2HK910) used for O-ring boss seals; past it the section
thins for good. 0.2 mm diametral clearance to slide over the thread. 4 degrees of tilt at release, measured as the
fingertips' height difference over their spacing (atan(dz / gap)) when the ring is let go, including the moment one
fingertip leaves it before the other: the 50 mm ring's far side then sits 3.5 mm off the groove plane, more than its
3.0 mm section, so that side lands on the thread. 90 degrees of twist: a ring pushed along the thread without being
stretched clear of it rolls 38 degrees per mm (its 3.0 mm section rolling without slipping); 90 degrees is a quarter
turn (2.4 mm of rolling), when the mold parting line, which a fitter checks runs round the ring's outside, has moved
onto its face and visibly spirals. Only a fingertip arriving on the ring's near face pushes (rolls) it; a ring
already in the groove is held by the groove's wall and does not roll. A ring stretched clear of the thread again
springs back untwisted. The ring is seated when it is in the groove, closed onto its 44.2 mm bottom (1.2% installed
stretch), did not land tilted, and no fingertip is in it or on it.

## Real-world build

All parts are off the shelf: a -24 SAE J1926 (1.875-12 UN) O-ring boss hex plug, AS568-924 90-durometer FKM
O-rings in blue (as sold for ORB fittings; the color shows the ring and its parting line on camera), and a 9 x 9 x 4 cm aluminum block with a 2.25 inch hex pocket 2 mm deep (or a
bench vise with soft jaws holding the plug's hex so the hex face is 5.4 cm above the table). The robot's own
fingertips stretch the ring, so no tool is needed; add a printed mm scale on the holder in view of the top camera.

A real trial is checked from the top camera and by hand: the largest fingertip spacing while the ring was on them
(against the mm scale; a pad gap over 71 mm means over 50%), the ring fully in the groove all round with no part of
it on the thread (fingertip and eye round it), the parting line straight (not spiraling), and the ring's section
measured with calipers after the trial (3.0 mm; a thinned ring is evidence of over-stretch).

Reset: roll the ring out of the groove with a plastic O-ring pick, check its section (3.0 mm) and parting line,
replace it if either is off, and lay it flat and centered on the plug's end face.
