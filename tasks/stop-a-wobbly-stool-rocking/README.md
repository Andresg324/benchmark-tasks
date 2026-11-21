# Stop a wobbly stool rocking: find the rocking corners, then shim the right foot

A one-step hardwood stool stands on the table with one leg 4 mm short, so it rocks. Two plastic shims lie beside it.
The instruction is only:

> **"Stop the stool from rocking."**

A wobble is the four feet not lying in one plane. The stool rocks about the diagonal through the two feet that always
bear; the other two feet (the short leg and its diagonal partner) take turns lifting. A shim of the right thickness
under either of those two feet stops the rock; a shim under one of the other two feet makes it worse. **Which leg is
short changes with the seed, so the robot has to find the rocking corners (by pressing the corners, or by spotting the
foot that hangs) before it shims.** It then has to push a 6-degree wedge in just far enough: about 4 mm under the foot
(a 9 mm push error makes a 1 mm thickness error).

Files: `task.py` (scene, scorer, scripted demo; `yam_arena` embodiment, `station` scene); `yamkit.py` spawns the
props, `gripkit.py` gives the finger pads rubber, `yamgeom.py` reads the arm's hulls for the hands-off check;
`test_task.py` holds offline checks.

## Run a trial

From this folder, in a shell with the yam_arena environment active:

```bash
STOOL_SEED=0 inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Stop the stool from rocking." \
  --scorer task_score --max-steps 1600 --store-frames --log-dir stool-demo
```

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`. The scorer (`task.py`'s `score()`, via
`--scorer task_score`) reports the trial's score at its end; make a video with `inspect-robots video <log>.json`.
Demo variants: set `STOOL_VARIANT=nothing`, `wrong` (shim under a foot that always bears), `hold` (an arm left
resting on the seat), `pressonly` (the corner presses, no shim) or `deep` (the shim pushed 18 mm past snug).

## Why it matters (Societal relatability)

Steadying a wobbly stool, chair or table with a shim or a folded card is something most people have done at home, in
a cafe or at work. It takes a diagnosis by feel (press the corners, see which foot lifts) and a small, controlled push.

## Scene

Positions in cm from the midpoint between the arm bases, x forward, y to the robot's left; heights above the table.
Front is the side facing the robot.

- **Stool**: eHemco solid hardwood step stool, one step (Target listing: 12.25 in tall, 12.75 x 12.75 in overall,
  6.6 lb, square top, four stretchers). Modeled as a 28 cm square top 2.2 cm thick, four 3.2 cm square legs splayed
  from 23.6 cm apart under the top to 29.2 cm apart at the feet (feet 32.4 cm outside to outside), and four 1.8 x 2 cm
  stretchers 9.5 cm up; 3.0 kg. Part sizes are estimated from maker photos. Middle at (34, 0). One leg is 4 mm short
  (seed 0: front left, 1: back left, 2: back right, 3: front right); the stool stands level on its three long legs with the
  short foot 4 mm up; a press on that corner tips it onto the short foot and lifts the diagonal partner's foot.
- **Shims**: two Wobble Wedges, standard rigid (polypropylene, black), 1.92 x 1.11 in, tapering from 0.03 in to
  0.25 in (6.45 degrees). Each lies flat beside the stool at (34, +-23.5), long side along y, thin end toward the stool.
- **Arms** start at the ready pose, grippers open.

## Scoring

The scorer tests the rock the way a person checks a stool. Once the robot has worked on the stool (an arm within 2 cm
of it, or a shim within 2 cm of it) and both arms have then been at least 2 cm from the stool for 2 s (every arm link,
link 1 to the fingertips, against the stool's box from the feet to the seat), every shim within 2 cm of that box is let
go (no fingertip within 5 mm of it) and lying on the table (tipped under 15 degrees, its bottom within 2 mm of it), and
the stool is still (under 1 cm/s and 0.05 rad/s), it presses each seat corner in turn (11.5 cm from the middle along x
and y, inside the feet) straight down with 20 N for 0.5 s, and records each foot's height at the end of every press.
The rock is the most any foot moves between the four presses. A stacked or flipped shim near the stool counts as not
lying on the table. The scene's layout is fixed (`-E object_jitter=0`); a trial whose stool spawns more than 5 mm off
its start stops with an error.

- **1.0**: rock under 1 mm, and the stool within 2 cm of where it started and tipped under 10 degrees.
- **0.5**: rock under 2 mm (less than half the starting 4 mm), the stool still on its marks and upright.
- **0.25**: a shim lies flat on the table (tipped under 15 degrees, bottom within 2 mm of it) under at least a quarter
  of the bottom of a foot of the rocking pair, and no shim is under the other two feet.
- **0**: anything else: doing nothing (no presses; pressed, the rock is 4 mm), a shim under a foot that always bears, the stool moved more
  than 2 cm or tipped, an arm back within 2 cm of the stool (the result is dropped until the arms are clear again and
  the presses repeat), or the presses not finished.

Real trial (feeler gauges, 2 kg bag of sugar): once neither arm has touched the stool or a wedge for 2 s and both have
moved back from it, check that every wedge by the stool lies flat on the table (not on its side, not stacked) and the
stool's feet are on their tape marks (within 2 cm); if not, the score is 0. Then set the bag on each seat corner's
quarter in turn (as close to the corner as it sits without hanging over; only which corner matters) and try each foot
with the 1.00 mm blade. Slide it under a foot on the table from any side; for a foot on a wedge, slide it along the
wedge's top from its thick end toward the foot (past the foot's edge the wedge's top falls away, so a gap under the
rest of the foot is not a lift). Any foot the blade goes under, in any of the four placings, has lifted: 1.0 if none
does; 0.5 if none takes the 2.00 mm blade (or two 1.00 mm blades stacked). 0.25 (rock still 2 mm or more): with the
bag on a rocking-pair foot's corner, a wedge under that foot stays put under a light finger-and-thumb tug, and no
wedge is held that way under either other foot (bag on its own corner). The stool's start is marked with tape at its
feet.

| Policy | Score |
| --- | --- |
| Demo, seed 0 (short leg front left) | 1.0 |
| Demo, seed 1 (short leg back left) | 1.0 |
| Nothing | 0 |
| Shim under a foot that always bears | 0 |
| Arm left resting on the seat | 0 |
| Both corner presses, no shim | 0 |
| Dream-1 (seed 0) | 0 (the hands stayed near the arm bases; they never reached the stool) |

## Real-world build

A low-cost setup: the eHemco stool, a pack of Wobble Wedges, a feeler gauge set (1.00 mm blade; 2.00 mm blade or two
1.00 mm blades), a 2 kg bag of sugar, 4 mm and 5 mm drill bits, tape. Assumed owned: saw or sander, wood glue.

- Build once: stand the uncut stool on the rig table and run the bag-and-gauge check above; no foot may take the
  1.00 mm blade (if one does, sand the long foot or tape under the low one until none does). Then saw or sand 4 mm off
  one leg, or glue 4 mm pads (4 mm craft plywood) under the other three feet.
- Acceptance (after the build, weekly, and after any drop): at rest the 4 mm drill bit's shank slides under the hanging
  foot and the 5 mm shank does not.
- Seeds: turn the stool so its short leg sits at the seed's corner (0 front left, 1 back left, 2 back right, 3 front
  right); the stool is square, so one stool covers every seed.
- Reset (under a minute): take the wedge out, set the stool on its tape marks, lay the wedges on their marks.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes). The trial ends once 1.0 has held for 1 s.
- `STOOL_SEED` (environment variable, default 0) picks the short leg (seed mod 4). Fixed layout
  (`-E object_jitter=0`); the demo is open loop and reads no simulated poses.

## Simulation fidelity

- Rigid bodies, PhysX, 20 Hz control. The stool and shims are free bodies; a shim lifts a foot only by contact and
  stays by friction (wood on wood 0.4 / 0.3, within Engineering ToolBox's 0.25-0.5; the shim's ribbed polypropylene
  is given the same; the wedge's 6.45 degrees needs only 0.11 to hold). The shims' ribs are not modeled (flat faces). The stool's mass is centered over its
  middle, 22 cm up.
- The scorer's corner presses are forces the scorer applies to the stool in the simulator, in place of the bag of
  sugar.

## Sources and licenses

- eHemco solid hardwood step stool (Target listing, sold as a pair: 12.25 in H x 12.75 in W x 12.75 in D, 6.6 lb,
  four stretch bars): https://www.target.com/p/ehemco-solid-hardwood-step-stool-12-25-inches-height-set-of-2/-/A-1003032401
- The same stool sold singly (Kohl's listing: 12.75 in W x 12.75 in D x 12.25 in H, 6.6 lb):
  https://www.kohls.com/product/prd-6630702/ehemco-solid-hardwood-wooden-step-stool-with-200-lb-load-capacity-for-adults-and-kids-1225-in.jsp
- Wobble Wedges spec sheet (standard wedge 1.92 x 1.11 x 0.25 in, tapers from 0.03 in to 0.25 in, angle 6.45
  degrees; rigid wedges polypropylene): https://www.webstaurantstore.com/documents/specsheets/wobble_wedges_specsheet.pdf
- Friction: Engineering ToolBox, friction coefficients.
- All code in this folder was written by this task's team. The robot model and its collision hulls load at run time
  from the competition environment and are not redistributed; `yamgeom.py` reads them through
  `yam_arena.analysis.link_hulls()`. No third-party meshes, images or data files are in this folder.
