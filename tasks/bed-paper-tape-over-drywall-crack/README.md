# Tape a drywall crack: lay paper joint tape centered along it and bed it in the wet compound with a joint knife

Two YAM arms tape a 12 cm crack in a half-inch drywall offcut that leans back 20 degrees in a bench stand. A bed of
wet joint compound (3 mm deep, 10 cm wide) is already laid along the crack. The arms lay an 18.8 cm strip of
2-1/16 in. paper joint tape centered on it, press it onto the wet bed, and bed it with a 4 in. joint knife, then set
the knife down. `task.py` is the task for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena); `sheetkit.py` (the tape), `bedkit.py` (the
compound, bedding and bond), `guardkit.py` (gripper contact), `yamgeom.py` (the arm links' collision hulls) and
`yamkit.py` (props, demo waypoints) are its helpers.

The tape makers' order: compound along the joint, the tape centered on it and embedded while wet, the excess taken
off with a broad knife "leaving about 1/32" of compound under the tape" (National Gypsum ProForm). The crack is short,
like those that "radiate from the corners of doors and windows" (Family Handyman). task.json gives every size and
position, and the real-world trial.

What it tests: handling a thin, flexible paper strip, carried by one end and laid centered along a hidden line, and a
two-arm steady-and-act stroke: one hand holds the tape's top end while the other draws the knife down it, pressing at
a shallow angle so the compound squeezes out from under it, without dragging, wrinkling or tearing the tape. The knife
spreads no compound; the bed is laid before the trial.

The offcut leans back so its face is within both arms' reach at bench height; this is not the same as a vertical
wall. The bed is laid first because the task begins at the taping step of the makers' order, and the strip is cut to
length beforehand, as for any short crack. The bed hides the crack, so pencil ticks on its line 2 cm past each end of
the bed show where it runs (as a finisher marks it before bedding).

## Requirements

`yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh` (Isaac
Lab-Arena `release/0.3.1`, `inspect-robots` 1.0.0, `inspect-robots-yam` 0.38.0), and `source $ARENA_DIR/yam.env`.

## Run and score a trial

`--scorer task_score` gives 1.0 when `success()` holds for 1 s, otherwise `score()` at the end of the trial. The
crack is checked at 13 stations, 1 cm apart. Each threshold is defined once in `task.py` or bedkit's `Law`.

| Score | Rule |
|---|---|
| 0.25 | The tape covers the crack at every station, and the held knife has been drawn over it, pressing with 2 N or more (below), at half the stations or more. Tape set by hand alone scores 0. |
| 0.5 | And centered: its center line within 3 mm of the crack at every station. |
| 0.75 | And bedded at 12 of 13 stations (90%): 0.4-1.6 mm of compound under it on the crack and 18 mm to either side, compound squeezed out at both edges, bonded, and no part of either arm on it after it was bedded. |
| 1.0 | And no wrinkle (every strip over the crack flat, corners within 0.7 mm), the knife released on the bench at least 2 mm clear of the face, no part of either arm on the tape or within 2 mm of the knife, knife and tape still for 1 s. |
| cap 0.5 | Torn tape: the knife pressing with one corner (edge turned over 15 degrees out of the face) dragged 5 mm or more. |

"Part of an arm" is every link from the shoulder to the fingertips, checked against each link's collision hulls as
the simulator poses them.

**Bedding needs contact force, not nearness.** A strip is pressed only while PhysX's contact report says the knife
touches it and the knife's total contact force on the tape, normal to the face, is at least 2 N (`PRESS_FORCE`). The
force is read once a control step (the last physics step's contact force between the knife's colliders and each
strip). The poses only locate the edge on a strip the engine says it pushes (within 0.6 mm, 1.0 mm once a stroke has
begun); nearness alone beds nothing. No measured force for a taping-knife stroke was found, so the force needed is
estimated from plastic squeeze flow of the compound out from under the tape: F = 2 tau_y a^2 b / h, with a = 26.2 mm
(half the tape's width), h = 0.8 mm (the film left) and b = 2.6-3.1 mm (the wedge where a blade at 35-40 degrees thins
the 3 mm bed to 0.8 mm). A commercially available joint compound's yield stress is about 200 Pa (US 12,054,627 B2,
Proform Finishing Products: "approximately 2,000 dynes/cm2"; the patent's own lightweight ready-mix is about 50 Pa),
which gives F = 0.9-1.1 N: about 1 N is the estimate of what bedding needs. The rule asks for 2 N, twice that
estimate, a conservative margin. This is an approximation: the compound does not push back on the knife, and above the
threshold the film left is set by the blade's angle alone. In the scripted demo the knife pushes the tape with 2.3-4.5
N (10th percentile to maximum over both passes, in two runs; 2 N or more in 304 of 309 contact steps in each; PhysX
contact report).
Variant `hover` is a careless pass: the whole demo, but the knife's edge aimed 4 mm off the tape. Held by the back of
its handle, the knife sags in the fingers, so its edge still grazes the tape, with 0.3 N (median) and at most 1.8 N:
below the threshold, it beds nothing and scores 0. Variant `light` aims the edge at the tape's surface instead of
pressing it in: the sagging knife still pushes the tape with 2.2 N (median; 1.8-2.8 N, 10th-90th percentile), a real
press above the 1 N estimate, so it beds only what it passes while pushing 2 N or more (fully bedded at 6 of 13 stations) and scores 0.5. Doing nothing scores 0. Tape laid off the line can be lifted and laid again before it is
bedded.

Model run (Dream-1): the command below with `--policy dream`. Scripted demo (open loop):

1. The left hand pinches the tape's top end where it overhangs the offcut, carries it out in front of the panel with
   the fingers tipped down the crack's line, then tips them square to the face and moves in: the tape lays itself
   down the line, its top end held folded out 2 cm above the crack.
2. The right hand's closed fingertips press the tape onto the bed in dabs 1.5 cm apart (never sliding on the tape),
   from 6 cm below the crack's top end to past its bottom end.
3. The left keeps hold of the tape's top end. As finishers do on the first pass, the held end keeps the knife from
   dragging the tape down the wall.
4. The right takes the knife by its handle and turns the blade up the tape at 35 degrees to the face. It embeds the
   tape from the middle out, in two passes at 1.5 cm/s. The first starts 6.5 cm below the crack's top end, clear of the
   left hand, and runs to 2.1 cm past the bottom end. The left then lets go and goes home. The second pass starts 2 cm
   above the crack's top end and runs to 9 cm below it, over the start of the first; the bedded lower half now holds
   the tape. The right lays the knife back.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/bed-paper-tape-over-drywall-crack/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Tape the crack in the wall panel with the paper tape and the joint knife, then lay the knife back on the bench." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/tape-demo
```

Video: `inspect-robots video logs/tape-demo/*.json`. `DRYWALL_VARIANT=hand_only`, `bare_bed`, `knife_left`, `steep`,
`hover` or `light` runs a cheat instead of the demo (rows marked "variant" below).

## Failure modes

| Attempt | Score |
|---|---|
| Nothing done, or the start state | 0 |
| Tape laid and pressed on by hand with no knife (variant `hand_only`), or dropped loosely on the crack | 0 |
| Knife drawn down the bare bed, tape left on the bench (variant `bare_bed`) | 0 |
| Knife pushed with no hand on it, or drawn beside the tape | 0 |
| Tape off the line (more than 3 mm) or turned | 0.25 at most |
| Knife drawn down the tape with its edge aimed 4 mm off it, grazing it with under 2 N (variant `hover`), or touching it with under 2 N | 0 |
| Steep pass (variant `steep`, 60 degrees) or a dry, thin bed: starved | 0.5 at most (checked offline; in the sim run the arms knock the tape off first: 0) |
| Knife over part of the crack only | 0.5 at most |
| Corner of the blade dragged along the tape | 0.5 at most |
| Knife left on the wall in hand (variant `knife_left`), wrinkle, a hand, wrist or forearm left on the tape | 0.75 at most |
| Bedded, then peeled or slid off | that stretch no longer bedded |

`test_task.py` (offline, no simulator) builds each of these and checks its score.

## Results

| Run | What it does | Score |
|---|---|---|
| Scripted demo (the film) | The whole task: tape laid, dabbed, bedded with the knife in two passes, knife laid back | 1.0, at step 1,980 (99 s): 13 of 13 stations bedded, tape within 0.4 mm of the crack |
| `hand_only` | Tape laid and dabbed by hand, no knife | 0 |
| `bare_bed` | Knife drawn down the bare bed, tape left on the bench | 0 |
| `knife_left` | The whole task, but the knife is left on the wall in the hand | 0.75 |
| `steep` | Both passes at 60 degrees, the second from 4 cm below the top end. Squaring the blade that steep, the right arm meets the left arm holding the top end and knocks the tape off the line before the first pass (the starved-bed cap, 0.5, is checked offline); in that run the 60-degree passes still pushed the displaced tape (2.2 N median) and left the strip they pressed starved, not bonded | 0 |
| `hover` | The whole demo, but the knife's edge aimed 4 mm off the tape: it grazes it with 0.3 N (median), at most 1.8 N, and beds nothing | 0 |
| `light` | The whole demo, but the knife's edge aimed at the tape's surface instead of pressed in: the sagging knife still pushes 2.2 N (median), bedding 6 of 13 stations | 0.5 |
| Dream-1 | The command above with `--policy dream`, on this task as submitted | 0 (2,400 steps): its hands went to the tape and the knife but carried neither to the panel; the tape stayed on its block |

## Real-world build

A pine 1x2 stop and a 2x2 brace screwed to the bench so the offcut's face leans back 20 degrees; the crack a 12 cm
utility-knife cut through the face paper; the bed struck off level with two 1/8 in. (3 mm) hardboard rails, inner edges 50 mm either side of the crack (10 mm
inside the pencil lines), from 2 cm inside one tick to 2 cm inside the other; the crack's line ticked with a carpenter's
pencil or marker 2 cm past each end of the bed. At setup, before the bed is laid and before the trial starts, two
pencil lines are drawn on the face parallel to the crack, 60 mm either side of it (marked from the crack with a
combination square), outside the 10 cm bed, for the centering check. Ready-mixed all-purpose compound (ProForm All Purpose Ready Mix, one tub for all trials, used unthinned at room
temperature, 20-25 C: about 60 mL (1/4 cup) per bed, stirred with the knife before each bed, the lid kept on between trials; the bed is struck just before
each trial and the trial starts within 2 minutes, while it is wet), 2-1/16 in. paper joint tape cut to
18.8 cm on a 2x4 offcut on a plywood scrap, a 4 in. flexible joint knife. Each of the four gripper
fingers wears a disposable nitrile glove (size L). A YAM finger is about 6.8 cm wide, too wide for a glove's
finger, so the finger goes into the glove's palm, pad against the palm side, and the glove's fingers are folded back
over the finger's outer face and taped there with the cuff by two turns of painter's tape. Every press and grip on the
tape and the knife happens through the glove; between trials the gloves are wiped with a damp sponge (a torn glove is
replaced). A plastic drop sheet lies under the stand,
the tape block and the knife's spot; the compound is water-based and wipes off with a damp sponge. The knife's blade
is wiped clean before it is laid back, so the handle the gripper takes stays clean. An arm link on bedded tape fails
the trial, so a wrist or forearm in the compound is not a routine way for the arm to get dirty. A low-cost setup;
reset takes about 3 minutes: the same offcut is reused (the used tape peeled off, the compound scraped off with the
knife and the face wiped with a damp sponge), a new bed from the same tub and a new strip of tape are laid (the strip's ends at its marks; the 2x4 and the knife at
painter's-tape outlines on the bench), the knife's
blade is wiped and the knife laid back, and the gloves are wiped. The real trial is scored by eye from the overhead video, an approximation of the
simulated rules (task.json's scoring): the tape over the crack (on the ticks' line, its ends within 3 cm of the bed's ends: the crack stops 3 cm inside
them), the gaps to the two pencil lines
looking equal, compound squeezed out along both edges, the tape flat with no wrinkle or tear, the knife laid back and
hands off. Nothing is measured.

## What is simplified

task.json's fidelity line lists it: the tape is 28 rigid hinged strips with the real width, weight and bending
stiffness; the compound is a 1 mm height field, not a fluid, and the knife's angle sets what it leaves (0.8 mm at 40
degrees or less, 0.1 mm at 60 or more: an assumed law, set to the makers' 1/32 in., not a measured one); the
compound does not push back on the knife: bedding is gated by the knife's PhysX contact force on the tape (2 N or
more, estimated from the compound's yield stress, above), not by a pressure field. A tape strip weighs about 0.05 g: pressed between the 110 g knife and the
wall, PhysX's contact solver does not converge at the default iterations (the knife pushes through the strip onto the
panel and the strip reports a fraction of the push), so the knife and the tape are solved with 128 position iterations
(one velocity iteration) and a strip's depenetration speed cap is 1 m/s (`SOLVER_ITERATIONS`, `STRIP_DEPENETRATION`;
masses and sizes unchanged); a bedded strip's hold is a joint switched on by the knife and released by a
peel or a slide; a fingertip touching a strip that lies on the bed (its pad within 1 mm, a pose check) switches on a far weaker tack
joint that only slows its sliding and is never counted as bedded (an assumption; its 30 Pa shear cap is below
the 50-200 Pa yield stresses of joint compounds in US 12,054,627 B2), never scored.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s); the scripted demo scores 1.0 and ends at step 1,980 (99 s).
- No randomness: the layout is fixed (`-E object_jitter=0`). Layouts other than `object_jitter=0` are not supported
  (the tape can slide off its 2x4 at reset).

## Sources

- National Gypsum ProForm Paper Joint Tape technical information (2-1/16 in.; "Center joint tape over the joint and
  embed in the compound ... leaving about 1/32" of compound under the tape"),
  https://pdf.lowes.com/productdocuments/4fe0a129-9b36-4872-ae7b-7e120d63deb5/64424890.pdf
- USG Sheetrock Heavy Joint Tape submittal J1393 ("about 1/32" (0.7 mm) of compound under feathered edge"),
  https://images.homedepot-static.com/catalog/pdfImages/7a/7a2ad9f3-62c7-45ef-b58e-869ccdbca0c6.pdf
- USG ME Paper Joint Tape data sheet IF-TDS039-1 (52 mm x 76 m, about 550 g per roll),
  https://www.usgme.com/default/mwdownloads/download/link/id/334
- Toupret paper joint tape data sheet (0.215 mm, 138 g/m²; compound "to a depth of about 3mm (1/8")"),
  https://bo.toupret.com/storage/media/shares/FT_TOUPRET_BANDE_A_JOINT_PAPIER_MO.pdf
- US patent 12,054,627 B2, "Lightweight joint compound composition" (Proform Finishing Products): yield stress of a
  commercially available joint compound "approximately 2,000 dynes/cm2" (200 Pa); its own example about 500 dynes/cm2,
  https://patents.google.com/patent/US12054627B2/en
- Fine Homebuilding, "Taping Fastener Heads and Seams: First Coat",
  https://www.finehomebuilding.com/project-guides/drywall/taping-fastener-heads-and-seams-first-coat
- Family Handyman, "How to Tape Drywall With a Banjo" ("Press the wet tape into joints by dragging your fingers over
  the tape"), https://www.familyhandyman.com/project/how-to-tape-drywall-with-a-banjo/
- Family Handyman, "How to Repair a Drywall Crack", https://www.familyhandyman.com/project/how-to-repair-a-drywall-crack/
- Paper's elastic modulus: K. Niskanen (ed.), Paper Physics, 2nd ed., Paperi ja Puu, 2008.

## Assets and licenses

Every file in this folder is the task authors' own code and text, released under the MIT License (`LICENSE`). Every prop is built
at start-up from shapes defined in the code; the folder holds no meshes, textures, images, fonts or data files. The
external assets below load at run time from the installed competition environment and are not redistributed here.

| Asset | Source | License |
|---|---|---|
| YAM arms and parallel grippers: URDF and link meshes | I2RT Robotics, [i2rt @ 120c3c81](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058), as shipped in [yam-isaac-lab-arena @ bd62771e](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam) (`yam_arena/assets/yam/`) | [MIT](https://github.com/i2rt-robotics/i2rt/blob/120c3c81400171174604e503943f8d1ebc891058/LICENSE), Copyright (c) I2RT Robotics (`LICENSE.i2rt` in the environment) |
| YAM collision hulls (`collision/`) and robot USD (`usd/yam_arm`) | Hulls cut with CoACD from the I2RT link meshes by [YAM-inspect-Isaac](https://github.com/ttpierson/YAM-inspect-Isaac), shipped unchanged by the environment; the USD is converted from the same URDF with Isaac Lab's URDF importer. `yamgeom.py` reads the hulls at run time (`yam_arena.analysis.link_hulls()`) and stores nothing derived from them. | MIT (I2RT), as derived from the I2RT model. YAM-inspect-Isaac declares no license. |
| Black and white link 2/3 meshes (station arm colors) | I2RT meshes split by Dream Labs, from the dreamlabs 1.0.23 wheel ([SOURCE](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/dream/SOURCE)) | "MIT, see ../LICENSE.i2rt" (the environment's `dream/SOURCE`) |
| Station room: floor, table, camera stand, lights and the three cameras | Generated in code by the environment (no mesh or texture files). Camera and arm-mount poses follow the [yamlab @ ec0455d2](https://github.com/ARISE-Initiative/yamlab/tree/ec0455d2b4ce35f21fc126418ea5e74ac567133d) calibration. | Calibration: [MIT](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE), Copyright (c) 2026 Tianyuan Dai. The environment's README: "This repository has no license yet". |
| Simulator | [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) release/0.3.1 (c7edf3a9), Isaac Lab (91a0906f), Isaac Sim 6.1.0 | Apache License 2.0; BSD-3-Clause; Isaac Sim's package metadata: "NVIDIA Proprietary Software" |
