# Finish a wire splice: tips even, then twist the connector on snug

Two black 12 AWG solid copper conductors stand up out of an open 4-11/16-inch-square steel junction box, stripped 16 mm and
bent so their tips meet side by side. A red twist-on wire connector (a wire nut) has been pushed onto the tips by hand
but not turned, and the right-hand wire has slipped 12 mm back into the box, so only the left wire's tip reaches into
the connector. "Finish this wire splice." means doing what every electrician is trained to do: get the two wires side
by side with their tips even, so the connector's coil grips both, then turn the connector clockwise until it is snug
and no bare copper shows below its skirt. Twisting the connector on without evening the tips is the tempting wrong
answer: the short wire is never gripped and pulls out or arcs.

## How it differs from similar tasks

Other threaded tasks on the board are about how far to turn one part: snugging a union nut without stripping it, or
turning a filter hand-tight plus a set amount. Here the turning is the easy half. The core is the
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
| Scores | 0, 0.25 (both wires in, copper showing), 0.5 (covered but loose, or a hand still on it), 1.0 (snug, covered, both wires held, let go). |
| Demo | Right arm pinches the right wire's horizontal run 5 cm in front of the connector and draws it up 12 mm, goes home; left arm rolls the wrist 960 degrees clockwise in 150-degree regrasp strokes (the connector turns about 930), goes home. |

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `twistkit.py` | Threaded parts: the connector turns on a hinge chained to a slide along the wires' axis, driven to 4 mm of travel per turn, with a seat (snug), a hard stop 60 degrees past it and "off the coil" more than 2.5 turns back. Gives the scorer the connector's angle from snug and the turning done while a gripper held it, and builds the demo's regrasp loop. This copy adds `Threaded.extra` (more driven links on the same base, here the sliding wire) and a stricter grip accounting (see Scoring) with `unheld_deg`; its grip test uses `guardkit.py`'s contact check. Built on `yamkit.py`. |
| `guardkit.py` | Contact between a gripper's own shape (palm and both fingers at their present opening, boxes fitted to its collision meshes) and the connector or the wires: `touching()` for let-go and `gripping()` (both fingers on it, the opening stopped by it) for holding. Trimmed to these two checks. |
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
   sits on the tips to about 13 degrees past snug (it slips a little at each regrasp). It lets go and lifts clear.
3. The run ends at 1.0 in 946 steps, once no hand has touched the splice for 1 s, while the left hand is still
   backing off (where the arms end is not scored).

## Scoring

The scorer watches the whole run (`success()` records it every step). The connector's angle is measured back from
snug, its seat, where its skirt covers the last 2 mm of insulation. The coil first bites 900 degrees back from snug (2.5
turns at 4 mm per turn: 10 mm of travel); the connector starts at 915, sitting on the left wire's tip.

| Outcome | Score |
|---|---|
| Doing nothing (the connector stays on one wire, nothing spliced) | 0 |
| Connector less than a full turn onto the wires (more than 540 degrees back from snug), never turned, more than 90 degrees of its net turning not credited to a gripper holding it, or more than 90 degrees of clockwise turning in all done without such a grip | 0 |
| The right wire's tip more than 3 mm from the left one's when the connector came a full turn on, or the wire moved more than 2 mm after that | 0 |
| Both wires in, the connector more than 180 degrees back from snug: bare copper shows below the skirt | 0.25 |
| Both wires in, 45 to 180 degrees back from snug: copper covered but the connector loose | 0.5 |
| Both wires in, within 45 degrees of snug, but a gripper still holding the connector or the right wire, or a hand still touching the connector or a wire | 0.5 |
| Both wires in, within 45 degrees of snug, let go: no hand touching the connector or the wires (a hand near them but off them is let go) | 1.0 |

Possible scores: 0, 0.25, 0.5, 1.0. Where the arms end up does not matter. In the sim, turning is credited when a gripper grips
the connector and turns with it (the wrist's roll, or any other turn of the gripper about the connector's axis): either hand, both in turn, a regrasp loop or one long roll, fingers on the skirt or
pinching the tapered top, with or without the other hand holding the wire. Up to 90 degrees of other turning is
allowed (the slip between regrasps). The reason is the sim's model: its coil holds the connector only lightly, so a
push or a knock could spin it on where a real connector would not turn. A gripper holds the connector for the "turned by hand" rule when its fingers
are closed on it: both fingers touching it (a gap of 2 mm or less, the solver's resting gap plus the fit, between the
connector and the gripper's own shape: palm and fingers at their present opening, fitted to its collision meshes) with their opening
stopped by it (between its 2.0 cm width minus 15 % and plus 3 % of full opening: an empty closed hand and an open hand
hovering around it do not count), and its pads over the connector's length. A hand near the connector that does not
touch it, or touches it with one finger, does not hold it. A step counts as held if the connector was held at either end of it, and it is credited only up to
1.25 times the holding wrist's own roll in the same direction (at most 15 degrees of unused roll carry over), so a connector turned by the other arm while a hand rests on
it is not credited. Uncredited clockwise turning is added up on its own (not netted against counterclockwise turning),
so turning it by hand, swiping it back and swiping it on again cannot launder a swipe. A gripper "still holds" something at the end when both its fingers touch the connector or the right wire with
their opening stopped by it (by the connector's 2.0 cm or the wire's 3.3 mm, 15 % narrower to 3 % wider); that caps the
score at 0.5, as for a hand resting on it, never 0. A hand that
only rests on the splice does not hold it, but it is not let go either: the 1.0 needs no gripper touching the
connector or either wire (the same 2 mm contact test). A hand near the splice but off it is let go, however close. `success()` is a full score held for 1 s. Each
scene reset by the simulator (counted by yamkit) starts a new history.

Why these limits: connector makers print "strip 5/8 in., hold the wires parallel with the ends even, turn clockwise
until snug" on the carton. 3 mm is under one 4 mm turn of the coil, so a wire within it is gripped by the coil's first
full turn. Capture is judged when the connector comes a full turn on, because that is when the coil has gripped the
wires: pulling a wire up into an already-tightened connector does not splice it, and moving a wire more than 2 mm
afterwards means it has been pulled out of the coil. If the connector is turned back off past that point, the wires
can be evened and the splice made again, as a real electrician would. Snug is within 45 degrees of the seat (0.5 mm of
travel); copper shows once the skirt is more than 2 mm above the seat, 180 degrees back. The 90-degree allowance for
turning without a grip covers the small turns between regrasps; a connector swiped or knocked round does not count,
because the sim's coil holds it only lightly where a real one would not spin from a knock. These angles, the 3 mm
and the 2 mm are the sim's stand-ins for what a real trial observes; the sim's coil does not hold the wire, so a
wire moved more than 2 mm after the bite is counted as pulled out.
The connector's turning resistance is a hold at its last angle, not a torque that builds toward snug, so "snug" is
scored from the angle (the seat and the hard stop 60 degrees past it), never from how hard it was to turn.

A real trial does not assume the coil's travel per turn, a snug angle or how far a wire may move. It scores what an
electrician checks, in this order, once the robot has let go (if a gripper still holds the connector or a wire at the
end, the operator opens it first):

1. Copper: look at the skirt from the side with a flashlight. Any bare copper below it is "copper shows": in a live
   box it is a shock and short-circuit hazard.
2. Tug test: hang a 2 kg weight (about 20 N, a firm tug) from the right wire's end under the bench for 5 s, then
   pull the connector straight up along the wires with a spring scale to 20 N. A tape flag on the right wire at its
   grommet shows any slide. The splice holds if the wire slides 1 mm or less and the connector stays on. A wire the
   coil never gripped slides straight out, since only the grommet's 2 to 3 N holds it; 20 N is far more than that and
   far less than a gripped 12 AWG wire holds. A gripped wire doesn't move, and 1 mm is the least the flag shows plainly.
3. Snug check: turn the connector clockwise with thumb and two fingers until it stops, counting the molded ribs that
   pass a mark on the box. The 8 ribs are 45 degrees apart. A connector left more than one rib short of snug works
   loose with heat and vibration.

0 if the splice fails the tug test. 0.25 if it
holds but copper shows; 0.5 if it holds and is covered but turns more than one rib further; 1.0 if it holds, is
covered and turns one rib or less further, with both hands let go, no finger touching the connector or either wire
in the final photo from the side, so the splice is seen to stay together on its own rather than being held or pressed
together by a hand (0.5 if a gripper still holds it or a hand still rests on it at the end). Any way the robot turns the connector counts in a real trial
(gripped and rolled, or pushed round by a fingertip): a real coil holds the connector, so a knock cannot spin it on,
and the outcome checks above decide the score. The sim's grip-and-turn rule exists only because its coil holds the
connector lightly.

Why each level is worth what it is. A splice that fails the tug test scores 0
because that wire would pull out and arc. 0.25 means both wires are joined, but bare copper still has to be dealt with
before the box can be closed. 0.5 means the copper is covered and the splice is safe to touch, but either it isn't
snug and would work loose, or a gripper still holds it or a hand still rests on it, so it hasn't yet been seen to
hold on its own (the hand could be what keeps the wire in). A finished splice still held is never 0. 1.0 is the
finished splice an electrician would close the box on.

## Runtime and seeds

- Demo: 946 control steps (47.3 s simulated) to a held full score.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

True size (a 4-11/16-inch-square box, 12 AWG THHN conductors 3.3 mm across, a connector 2.0 cm across and 2.8 cm long). The
wires are stiff and do not bend: the left one is fixed and the right one slides only along its own length through its
bushing, with a light hold so it stays where it is left. The coil is a thread model (twistkit): the connector rides a
slide along the wires' axis and a hinge, driven to 4 mm of travel per turn, held where it is left by a small torque, and
stopped 60 degrees past snug. The wires do not twist together under the connector, the connector does not need to be
held against the wires' turning (the wires cannot turn), and there is no torque build-up at snug beyond the stop. Off
the coil the connector stays on the wires' axis: it can be lifted 3 cm but not carried away. Capture is scored from the
wire's position when the coil grips, not from contact; a real trial checks it with the tug test above.

## Real-world build

A 4-11/16 inch square, 2-1/8 inch deep galvanized steel junction box screwed to the bench, open face up, with two
6.4 mm holes drilled in its back, one 5.3 cm toward the robot and 3 cm to its right of the center, the other mirrored
(5.3 cm away, 3 cm left), each with a white snap-in nylon bushing 9 mm across (the wires run 5 mm inside the box's
walls, too close to them for a standard knockout), over 13 mm holes drilled through the bench. Two 40 cm
lengths of black 12 AWG THHN solid copper wire come up through the bushings from under the bench, each bent 9 cm
above the bench to run 5.3 cm toward the middle along x, then 3 cm along y, and stand up side by side, stripped 16 mm
(0.625 inch). The wires are dead: neither end is connected to anything, and nothing in the setup carries voltage.

The left wire is fixed: a screw-down cable clamp on the bench's underside, 2 cm below its hole, holds it so it
neither slides nor turns. The right wire is supported only by a rubber grommet pressed into its hole in the bench,
sized to grip the insulation lightly (2 to 3 N to slide it), so it stays wherever it is left. A cable tie on it under
the grommet stops it 10 mm above even, and a paint mark on its insulation meets the grommet's underside when its tip
is 12 mm below the left one's. A red twist-on connector rated for two to four 12 AWG wires (e.g. Ideal 76B or 3M
Scotchlok R). For the tug test: a 2 kg weight with a hook and a spring scale reading to 20 N.

For each trial: turn the connector off, push the right wire down from under the bench until its paint mark meets the
grommet, and set the connector loosely on the left tip, pushed on by hand but not turned. A connector that has been
tugged off or strained can be reused; replace it if its coil looks spread.

## Category

Impact on jobs: electricians (about 800,000 US jobs, BLS) splice conductors with twist-on connectors as a core,
everyday part of wiring work.

## Licenses

All code here was written for this task; no third-party code is included. No third-party assets are used: every shape
is generated by `yamkit.py` and `twistkit.py`.
