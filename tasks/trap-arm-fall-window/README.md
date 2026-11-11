# Set the trap arm's fall inside the code window

A 1:2 model of a bathroom sink's 1.25 inch PVC trap arm being roughed in. The pipe runs from the sink's trap to the
vertical stack on three adjustable pipe stands, and a torpedo level lies on it. The instruction is "Adjust the pipe stands
so the sink drain is ready to connect." A plumber sets a horizontal drain to a fall inside a window, and the plumbing
code sets both ends of it. The International Plumbing Code (Table 704.1) requires at least a quarter inch per foot
(20.8 mm per m) for pipe 2.5 inches and smaller: with less, the line holds water and solids that build into a clog. On
a trap arm the IPC also limits the total fall from slope to one pipe diameter. If the arm is steeper than that, its
outlet end drops below the trap's weir, the arm runs full and siphons the trap dry, and sewer gas gets in. The pipe also
has to bear on every stand, or it sags into a belly. The arm starts dead level, with the level's bubble centered (it
looks right), and its middle stand 1 mm low. Leaving it level, or leveling it carefully, is the tempting wrong answer.

Run (from the repo root, Isaac Lab-Arena installed):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/trap-arm-fall-window/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Adjust the pipe stands so the sink drain is ready to connect." --scorer task_score --max-steps 1000
    python tasks/trap-arm-fall-window/test_task.py      # offline scorer checks, prints "all pass"

For a Dream-1 trial, use `--policy dream --max-steps 2400`. The scene is deterministic (no seeds, object_jitter=0).
The scripted demo is planned at 41.8 s (836 steps, then success held for 1 s; `--max-steps 1000` leaves margin for
arm lag), and a full trial is 2,400 steps. The job is 1.6 turns on each end screw (five 120-degree regrasp strokes
each, both arms at once) and half a turn on the middle one. The demo
scores 1.0 in 848 steps; the Dream-1 run is pending (it is the final step, after the demo is approved).

Files: `task.py` (scene, scorer, demo), `test_task.py` (offline checks), and local copies of `yamkit.py`,
`twistkit.py` and `levelkit.py`. The levelkit copy holds the resting model of a body on adjustable supports; in slope
mode it handles a pipe on stands in a line.

## Spec

Positions are in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Drain pipe | white PVC, 2.1 cm across (1.25 inch PVC at 1:2), 43 cm long; fixed in place and posed by the stand model | along y at x 30, y -21.5 to 21.5, axis 7.25 cm up at the start |
| Sink trap | gray upright cylinder, 3.2 cm across, 10 cm tall, fixed (stands for the trap and tailpiece) | x 30, y 25 |
| Stack | white upright cylinder, 4.4 cm across (3 inch pipe at 1:2), 16 cm tall, fixed | x 30, y -25 |
| Stands (3) | dark gray pad 3.2 cm across on the table; steel 1/2-13 screw; black hex head 2.4 cm across flats, 1.6 cm tall, 9.2 to 10.8 cm up; steel bracket 2 x 0.8 cm threaded on the screw, reaching under the pipe to a black V-saddle | screws at x 24; y 12.5 (sink end), 0 (middle), -12.5 (stack end) |
| Torpedo level | orange, 11.5 x 1.6 x 1.6 cm (a 9 inch level at 1:2), lies on the pipe; tube vial drawn 6 cm long, two black lines around center, white bubble that runs to the high end | centered x 30, y 7.5 |

Each screw raises its bracket 1.954 mm per turn when turned counterclockwise from above: the real lead of a 1/2-13 UNC
thread (13 threads per inch), the thread of common 0.5 inch leveling mounts. Full size, the heights double, so a real
job is twice the turns. Travel runs from fully in to a stop at 2.1 turns (ends) or 1.6 turns (middle).

| Stand | Bracket range (underside, mm above the table) | Start |
|---|---|---|
| Sink end | 50.0 to 54.1 | fully in: 50.0 |
| Middle | 49.0 to 52.1 | fully in: 49.0, which leaves it 1 mm below the pipe |
| Stack end | 45.9 to 50.0 | at its stop (2.1 turns): 50.0 |

The pipe rests on the two end stands, dead level. Running both end screws to their stops (the middle on the line) gives 32.8
mm per m, steeper than the window, so overdoing the fall is possible (by a margin of 1.1 mm per m, 0.27 mm
between the ends: the cost of keeping every hinge within 1.25 turns). The middle stand can also be
run up high enough to lift an end off its saddle, so the pipe hogs over the middle.

Model (`levelkit`, slope mode): each control step, the three saddle heights (set by the screws' turns) give the line
the rigid pipe rests on. That line is the upper hull of the three points, taking the face that contains the pipe's
center of mass, or the load point pulled toward a hand pressing on the pipe. A saddle below that line hangs slack by
its gap. The pipe is posed to the line kinematically. Each head is a twistkit thread whose hinge zero
sits one turn from its start (levelkit's `Foot(seat_turns=...)`: 1.0 turn out for the sink end, 0.5 for the middle,
1.1 for the stack end), so the hinge limits are -365/+426, -185/+426 and -401/+390 degrees, all within the 1.25 turns
(450 degrees) proven in the sim, and no head starts more than 360 degrees from its zero. A screw turned against its stop stays at the stop. The score reads both the saddles (the fall between the end saddles, and the middle saddle's offset from their line) and the pipe's own fall as it rests, so rocking the pipe onto a high middle stand cannot fake a fall.

## Scoring

Nothing is glued yet, so any setting can still be changed: only the end state counts (SCORE_OVER = "end"), and
success must hold for 1 s.

| Term | Definition | Why |
|---|---|---|
| Fall | the drop toward the stack, mm per m, read two ways that must both pass: between the two end stands' saddles, and along the pipe as it rests | in a real trial: a digital angle gauge laid on the pipe, or a steel rule reading the pipe-center heights at the end stands (25 cm apart) |
| Code minimum | 20.8 mm per m (a quarter inch per foot) | IPC Table 704.1, drains 2.5 inches and smaller: less fall leaves standing water and solids that clog |
| Trap limit | 31.75 mm per m | the IPC caps a fixture drain's total fall from slope at one pipe diameter: 1.25 in (15.9 mm at 1:2) over the 1.0 m (50 cm model) from the trap's weir to the stack's axis. Steeper, the outlet drops below the weir and the arm siphons the trap |
| Half fall | 10.4 mm per m (an eighth of an inch per foot) | the IPC minimum for 3 to 6 inch pipe: the right direction but under code for this pipe |
| Bearing | the middle saddle at most 0.5 mm below a string line between the end saddles and at most 0.25 mm above it | a 0.5 mm feeler gauge does not slide in: a low middle stand carries nothing and the pipe bellies there; a high one rocks the pipe and lifts an end saddle twice as far (the middle is halfway), so 0.25 mm above is 0.5 mm at the end |
| Hands off | no fingers closed on a head, no hand pressing on the pipe, no tool point within 3 cm of the drain assembly (pipe, level, brackets, heads) | nothing counts while a hand is still on the job; the arms' home is 8.5 cm past the pipe's ends and 15 cm above the heads |

| Outcome | Score |
|---|---|
| Fingers closed on a head, a hand pressing on the pipe, or a tool point within 3 cm of the assembly | 0 |
| Fall 20.8 to 31.75 mm per m, every stand bearing, both arms home and still | 1.0 |
| The same, arms not home and still (each tool point within 3 cm of where it started) | 0.9 |
| Fall 20.8 to 31.75 with one stand not bearing (a belly, or an end lifted off by a high middle stand) | 0.25 |
| Fall 10.4 up to 20.8, every stand bearing | 0.25 |
| Anything else: doing nothing, level, back-fall toward the sink, under 10.4, steeper than 31.75 | 0 |

Possible scores: 0, 0.25, 0.9 and 1.0. The window is about 1.33 to 2.03 turns on each end screw when both are turned equally. The demo sets 6.25 mm (25.0 mm per m,
0.30 inch per foot). No single screw on its own reaches the window: one end at its stop gives only 16.4 mm
per m, with the middle stand left slack, and any single screw scores 0. Lowering only the stack end until the line
meets the 1 mm low middle stand leaves the middle high, so the pipe rocks on it and falls under 10.4 mm per m: 0.

Checks for a real trial:

- Lay a digital angle gauge on the pipe, or read the pipe-center height at the two end stands with a steel rule:

  | Fall (mm per m) | Angle gauge | Rule, difference over the 25 cm between end stands |
  |---|---|---|
  | 10.4 (half fall) | 0.60 degrees | 2.6 mm |
  | 20.8 (code minimum) | 1.19 degrees | 5.2 mm |
  | 31.75 (trap limit) | 1.82 degrees | 7.9 mm |

- Pull a string line between the end saddles and check the middle saddle against it with feeler gauges: at most 0.5 mm
  below, at most 0.25 mm above (a 0.5 mm feeler must not slide under either end of the pipe).
- Check that the arms are back where they started.

## Real-world build

- 1.25 inch schedule 40 PVC pipe, about 90 cm long (full size), or 0.5 inch PVC for a 1:2 bench model.
- Three stands, each made from:
  - a 0.5 inch, 13 threads per inch threaded rod with a hex coupling nut pinned on top as the head;
  - the rod's foot resting in a cup on a steel base plate;
  - a 1.625 inch strut bracket with a tapped hole riding the rod, carrying a V-saddle (strut pipe roller or a short
    piece of angle).
- The 0.5-13 thread moves 1.95 mm per turn: 1.6 turns per end stand on the 1:2 model, about 3.2 at full size.
- A 9 inch torpedo level, a digital angle gauge or steel rule, and a 0.5 mm feeler gauge.
- A P-trap and a 3 inch stack stub with a sanitary tee as end markers. They are not connected.
- To reset: sink-end and middle screws fully in, stack-end screw out to its stop (2.1 turns on the model); check
  that the pipe is level and the middle saddle 1 mm below it.

## How it differs from nearby board tasks

- **drain-hose-slopes-downhill (#509).** A flexible condensate hose is laid over steps so that it runs downhill with no
  dip, and any downhill route passes. In this task the fall must land inside a numeric window whose two ends both come
  from the code, a fall that is too steep scores 0, and the pipe is rigid, set by turning three threaded supports, with
  every support required to bear.
- **level-platform-faulty-gauge (#71), shim-the-soft-foot-under-the-motor-base (#743).** These tasks are about leveling
  and stopping a rock. Here, level is the wrong answer: the pipe must be set to a measured incline.
- **beam-packers-to-datum (#1038).** Bearings are packed to a level datum with loose packers. Here there is no datum,
  and the target is a slope range.
- **hang-cable-between-hooks (#10).** A cable is hung with a set sag, a catenary shape. Here the job is grading a rigid
  run.
- No newer PR (#1494 to #1521) touches drains, slopes or pipe supports.
