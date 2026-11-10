# Finish a wire splice: tips even, then twist the connector on snug

Two black 12 AWG solid copper conductors stand up out of an open 4-11/16-inch-square steel junction box, stripped 16 mm and
bent so their tips meet side by side. A red twist-on wire connector (a wire nut) has been pushed onto the tips by hand
but not turned, and the right-hand wire has slipped 12 mm back into the box, so only the left wire's tip reaches into
the connector. "Finish this wire splice." means doing what every electrician is trained to do: get the two wires side
by side with their tips even, so the connector's coil grips both, then turn the connector clockwise until it is snug
and no bare copper shows below its skirt. Twisting the connector on without evening the tips is the tempting wrong
answer: the short wire is never gripped and pulls out or arcs.

## How it differs from similar tasks

Other threaded tasks on the board are about how far to turn one part: #696 snugs a union nut without stripping it, and
our oil-filter task turns a filter hand-tight plus a set amount. Here the turning is the easy half. The core is the
preparation a splice needs before any turning counts: the two conductors' tips must be brought even so the
connector's coil captures both, and the scorer judges that at the moment the coil grips (a wire pulled up into an
already-tightened connector does not count). Twisting the connector on as found, which looks exactly like a finished
splice from above, scores 0.

## Spec

| Item | What it is |
|---|---|
| Scene | One fixed articulated object, `box`: a 4-11/16 inch square, 2-1/8 inch deep galvanized junction box (fixed), the left wire (fixed, part of the box), the right wire on a vertical slide (`box/wire_b`, starts 12 mm short, stays where it is left), and the connector on a thread (`box/nut`, twistkit: 4 mm per turn, bites 2.5 turns above snug, starts 915 degrees back, i.e. sitting on the left tip). |
| Instruction | "Finish this wire splice." |
| Unstated rule | Tips even (within 3 mm) when the coil grips the wires, then clockwise until snug with no bare copper showing. |
| Trap | Twisting the connector on as it sits: scores 0 (the right wire is not captured). |
| Scores | 0, 0.25 (both wires in, copper showing), 0.5 (covered but loose), 0.9 (snug, arm not home), 1.0. |
| Demo | Right arm pinches the right wire's horizontal run 5 cm in front of the connector and draws it up 12 mm, goes home; left arm rolls the wrist 960 degrees clockwise in 150-degree regrasp strokes (the connector turns about 930), goes home. |

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `twistkit.py` | Threaded parts: the connector turns on a hinge chained to a slide along the wires' axis, driven to 4 mm of travel per turn, with a seat (snug), a hard stop 60 degrees past it and "off the coil" more than 2.5 turns back. Gives the scorer the connector's angle from snug and the turning done while a gripper held it, and builds the demo's regrasp loop. This copy adds `Threaded.extra` (more driven links on the same base, here the sliding wire) and a stricter grip accounting (see Scoring) with `unheld_deg`. Built on `yamkit.py`. |
| `yamkit.py` | Helper that spawns the modeled props and articulations, and gives the scorer the grippers. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline scorer checks: step-by-step histories of the connector, the right wire and the grippers, near misses included. No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/wire-nut-twisted-on/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Finish this wire splice." --scorer task_score --max-steps 2400

Model run (Dream-1): the same command with `--policy dream`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/wire-nut-twisted-on/test_task.py` (prints `all pass`).

## The demo

1. Right arm: comes down over the right wire's horizontal run (the leg running toward the robot, 5 cm in front of
   the connector, so the 6.8 cm wide fingers stay clear of it) with the fingers across it, pinches it, draws it 12 mm
   straight up out of the box so its tip is level with the left one's, lets go and goes home.
2. Left arm, once the right arm is home: closes on the connector and turns it clockwise (seen from above) with a
   regrasp loop: the wrist rolls 150 degrees in 15-degree steps, riding down with the coil; the fingers open, the
   wrist rolls back, the fingers close again. Seven strokes, 960 degrees of wrist roll in all, take the connector from where it
   sits on the tips to about 13 degrees past snug (it slips a little at each regrasp). It lets go, lifts clear and goes home.
3. Both arms end at home.

## Scoring

The scorer watches the whole run (`success()` records it every step). The connector's angle is measured back from
snug, its seat, where its skirt covers the last 2 mm of insulation. The coil first bites 900 degrees back from snug (2.5
turns at 4 mm per turn: 10 mm of travel); the connector starts at 915, sitting on the left wire's tip.

| Outcome | Score |
|---|---|
| Connector less than a full turn onto the wires (more than 540 degrees back from snug), never turned, more than 90 degrees of its net turning not credited to a gripper holding it, or more than 90 degrees of clockwise turning in all done without such a grip | 0 |
| The right wire's tip more than 3 mm from the left one's when the connector came a full turn on, or the wire moved more than 2 mm after that | 0 |
| A gripper still holding the connector or the right wire at the end | 0 |
| Both wires in, the connector more than 180 degrees back from snug: bare copper shows below the skirt | 0.25 |
| Both wires in, 45 to 180 degrees back from snug: copper covered but the connector loose | 0.5 |
| Both wires in, within 45 degrees of snug, let go, an arm not home | 0.9 |
| The same with both arms home and still | 1.0 |

Possible scores: 0, 0.25, 0.5, 0.9, 1.0. A gripper holds the connector for the "turned by hand" rule when its fingers
are closed on it (open between its 2.0 cm width minus 15 % and plus 3 % of full opening: an empty closed hand and an
open hand hovering around it do not count), its tool point within 1.5 cm of its axis, and its pads overlapping the
connector's length. A step counts as held if the connector was held at either end of it, and it is credited only up to
the holding wrist's own roll that step (plus 3 degrees), so a connector turned by the other arm while a hand rests on
it is not credited. Uncredited clockwise turning is added up on its own (not netted against counterclockwise turning),
so turning it by hand, swiping it back and swiping it on again cannot launder a swipe. A gripper "still holds" something at the end when its fingers are under 60 % open with its tool point within 2
cm of the connector or of the right wire's horizontal run. The arms count as home with each tool point within 3 cm of
where it was at the trial's first scored step, and still (moved under 1 mm in the last step). Setting
`REQUIRE_ARMS_HOME = False` at the top of `task.py` turns that part off. `success()` is a full score held for 1 s. Each
scene reset by the simulator (counted by yamkit) starts a new history.

Why these limits: connector makers print "strip 5/8 in., hold the wires parallel with the ends even, turn clockwise
until snug" on the carton. 3 mm is under one 4 mm turn of the coil, so a wire within it is gripped by the coil's first
full turn. Capture is judged when the connector comes a full turn on, because that is when the coil has gripped the
wires: pulling a wire up into an already-tightened connector does not splice it, and moving a wire more than 2 mm
afterwards means it has been pulled out of the coil. If the connector is turned back off past that point, the wires
can be evened and the splice made again, as a real electrician would. Snug is within 45 degrees of the seat (0.5 mm of
travel); copper shows once the skirt is more than 2 mm above the seat, 180 degrees back. The 90-degree allowance for
turning without a grip covers the small turns between regrasps; a connector swiped or knocked round does not count.
The connector's turning resistance is a hold at its last angle, not a torque that builds toward snug, so "snug" is
scored from the angle (the seat and the hard stop 60 degrees past it), never from how hard it was to turn.

In a real trial: before the trial, mark a line across the connector's top and on the box. Afterwards count the turns
the connector made (it bottoms out at snug), check with a flashlight that no bare copper shows below the skirt, measure
with a ruler how far each wire's insulation sits below the skirt (both within 3 mm of each other means both were
captured), and tug each wire firmly: a captured wire does not pull out. Watch the video for the order (wire evened
before the connector was turned on).

## Runtime and seeds

- Demo: 1,109 control steps (55 s simulated) to a held full score.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

True size (a 4-11/16-inch-square box, 12 AWG THHN conductors 3.3 mm across, a connector 2.0 cm across and 2.8 cm long). The
wires are stiff and do not bend: the left one is fixed and the right one slides only along its own length through its
knockout, with a light hold so it stays where it is left. The coil is a thread model (twistkit): the connector rides a
slide along the wires' axis and a hinge, driven to 4 mm of travel per turn, held where it is left by a small torque, and
stopped 60 degrees past snug. The wires do not twist together under the connector, the connector does not need to be
held against the wires' turning (the wires cannot turn), and there is no torque build-up at snug beyond the stop. Off
the coil the connector stays on the wires' axis: it can be lifted 3 cm but not carried away. Capture is scored from the
wire's position when the coil grips, not from contact. The tug test electricians make afterwards is not scored.

## Real-world build

A 4-11/16 inch square, 2-1/8 inch deep galvanized steel junction box screwed to the bench, open face up, with two
1/2-inch knockouts in its back, one 5.3 cm toward the robot and 3 cm to its right of the center, the other mirrored
(5.3 cm away, 3 cm left), each with a snap-in plastic bushing. Two lengths of black 12 AWG THHN solid copper wire come up
through the knockouts from under the bench, each bent 9 cm above the bench to run 5.3 cm toward the middle along x, then 3 cm along y,
and stand up side by side, stripped 16 mm (5/8 inch). The right wire passes loosely through its knockout so it can be
pulled up by hand. A red twist-on connector rated for two to four 12 AWG wires (e.g. Ideal 76B or 3M Scotchlok R). For
each trial: push the right wire down 12 mm below the left one's tip and set the connector loosely on the left tip,
pushed on by hand but not turned.

## Licenses

All code here was written for this task; no third-party code is included. No third-party assets are used: every shape
is generated by `yamkit.py` and `twistkit.py`.
