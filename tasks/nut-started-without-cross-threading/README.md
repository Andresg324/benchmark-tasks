# Start a hex nut on a stud: back it until the thread drops, then run it down by hand

An M36 x 4 hold-down stud stands up out of a 25 mm steel base plate, with a flat washer on it. A heavy hex nut has been
set on the stud's end but not turned. "Start the nut on the stud and run it down two full turns by hand, then let go."
means doing what every mechanic is trained to do: set the nut square and first turn it backward (counterclockwise
seen from above) until its thread start passes the stud's and it drops onto the thread with a small click, then turn
it forward, and it runs down freely. Turning the nut forward straight away is the tempting wrong answer: in this model
it cross-threads, binds after half a turn and scores 0.

## How it differs from similar tasks

The core is the back-turn engagement, the step before any forward turning. #330 (flange caps in star order) only sets
nuts on studs and scores the bolting order; #696 (union nut snug) and our oil-filter task (turn a set amount past
gasket contact) are about how far to tighten; our wire-nut task twists a connector on until snug after evening the
wire tips; #936 backs a tap off to break the chip after cutting. None of them scores how a thread is engaged. Here how
far the nut goes is easy (two turns, stated in the instruction); what is scored is whether it was turned back past the
drop first, and a nut forced forward before the drop is ruined, irreversibly.

## Spec

| Item | What it is |
|---|---|
| Scene | One fixed articulated object, `stud`: a 200 x 160 x 25 mm steel base plate, an ISO 7089 washer and an M36 x 4 stud (all fixed), and an ISO 4032 M36 heavy hex nut on a thread (`stud/nut`, twistkit with the thread-start model below), perched on the stud's end 4.5 turns above the washer. |
| Instruction | "Start the nut on the stud and run it down two full turns by hand, then let go." |
| Unstated rule | Turn the nut backward until it drops onto the thread (180 degrees here, a 2 mm drop), only then forward. |
| Trap | Turning forward straight away: cross-threaded at 60 degrees forward (irreversible 0), jams at 180. |
| Scores | 0, 0.25 (dropped, under a turn down), 0.5 (1 to 2 turns), 0.9 (2+ turns, arm not home), 1.0. |
| Demo | Left arm turns the nut back 215 degrees in two strokes (it drops at 180), then forward 900 degrees of wrist roll in 150-degree regrasp strokes, lets go and goes home. |

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `twistkit.py` | Threaded parts (copy of the wire-nut task's, with its grip accounting: one-sided hold band, held at either end of a step, roll budget bounded by the net roll over each grip, unheld turning counted per direction). This copy adds the thread-start model: `Thread(drop, cross, bind)`, `start_model()`, and the scorer calls `engaged`, `cross_threaded`, `run_turns`, `thread_height_u`. Its slide joint is zeroed at the nut's start height and held stiffly from the first physics step, so the nut spawns perched instead of sagging while the scene settles. Built on `yamkit.py`. |
| `yamkit.py` | Helper that spawns the modeled props and articulations, and gives the scorer the grippers. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline scorer checks: step-by-step histories of the nut and the grippers, near misses and grip-accounting holes included. No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/nut-started-without-cross-threading/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Start the nut on the stud and run it down two full turns by hand, then let go." \
      --scorer task_score --max-steps 2400

Model run (Dream-1): the same command with `--policy dream`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/nut-started-without-cross-threading/test_task.py` (prints `all pass`).

## The demo

1. Left arm: comes down over the perched nut with the fingers open, closes on two flats and rolls the wrist
   counterclockwise (seen from above) 120 degrees, regrips and rolls another 120: at 180 degrees the nut drops 2 mm onto
   the thread.
2. It then turns the nut clockwise with a regrasp loop (grip, roll 120 degrees in 15-degree steps riding down with the
   thread, let go, roll back open, repeat; 900 degrees of wrist roll in all), so the nut runs down a little over two
   turns, lets go, lifts clear and goes home. The right arm stays home.

Every stroke and start yaw is a multiple of 60 degrees, so each regrip closes the pads square on a pair of flats, as it
must on a real hex nut.

## Scoring

The scorer watches the whole run (twistkit tracks the nut's angle step by step, unwrapped). Angles are seen from above.

- Cross-threaded: before the nut has dropped onto the thread, turned forward (clockwise) 60 degrees or more from where
  it sat. Irreversible for the trial: backing it off afterwards does not undo it (the first thread is already cut
  crooked). At 180 degrees forward it jams (a hard stop). 60 degrees is a sixth of a turn, 0.7 mm of travel at 4 mm
  per turn: about where a crooked nut's first thread has bitten into the wrong groove; a smaller wobble forward while
  setting the grip is forgiven.
- Dropped (engaged): turned backward 180 degrees from where it sat, the nut's thread start passes the stud's and it
  drops 2 mm (half the pitch) onto the thread. Turned further back after that it rides up and clicks over the start
  again once per turn (a full pitch), as a real nut does.
- Turns run down: counted from the height at which it dropped, 4 mm per turn.
- Turned by hand: at most 60 degrees of its turning in each direction (counted separately, so swiping back and forth
  cannot cancel out) may be done without a gripper holding it and rolling with it. 60 degrees leaves room for the nut
  coasting or being nudged a few degrees as the fingers open and close at each of a dozen regrips, and is a sixth of a
  turn, far short of the 180 back and 720 forward the job needs. Holding means the fingers closed on its 5.5 cm flats
  (opening no more than 2.8 mm wider than the flats, up to 15 % narrower), the tool point within 1.5 cm of its axis
  and from 2 mm below its underside to 10 mm above its top; a held step is credited only from a budget the holding
  wrist's own roll earns (1.25 x the roll, at most 15 degrees carried over), and over each grip the net credited turn
  may not exceed 1.25 x the wrist's net roll plus 5 degrees, in the same direction. So a nut pushed round by the other
  arm while a hand rests on it (even one wiggling its wrist back and forth), or an open hand hovering round it, does
  not count.
- Let go: no gripper with its fingers under 75 % open and its tool point within 2 cm of the nut.
- Arms home: each tool point within 3 cm of where it was at the first scored step, and still (under 1 mm per step).

| Outcome | Score |
|---|---|
| Cross-threaded at any point, never dropped, more than 60 degrees turned without a grip, or still held | 0 |
| Dropped, run down less than 1 turn | 0.25 |
| Dropped, run down 1 to under 2 turns | 0.5 |
| Dropped, run down 2 turns or more, let go, an arm not home | 0.9 |
| The same with both arms home and still | 1.0 |

Two turns (8 mm) is where two full threads carry the nut, so it has shown that it runs freely; running on to the washer
is fine (tightening is a later wrench step and not part of the job). Success is a full score held for 1 s. Real trial:
count the turns from the video by the white stripe and the paint dot, watch for the drop, and measure the nut's
underside above the washer with a ruler (1.8 cm perched; 1.0 cm or less after the drop and two turns down).

## Runtime and seeds

- Demo: 1,172 control steps (59 s simulated) to a held full score.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

True size and weight (M36 x 4 stud, a 55 mm across-flats, 31 mm, 0.37 kg nut). The thread is a model (twistkit): the
nut rides a slide along the stud's axis and a hinge, the slide driven to the nut's height on the thread, held where it
is left by a small torque. The thread-start model is deterministic: the nut is set with its thread start half a turn
from the stud's, so it always drops after 180 degrees back, and forcing it forward 60 degrees before the drop always
cross-threads it. In reality cross-threading is a risk that depends on how square and how hard the nut is pushed; the
deterministic rule models that risk, which is why mechanics back every nut off first. The nut cannot tilt or come off
the stud's axis, and the click is shown by the 2 mm drop, not heard. The nut is drawn as a hex but collides as a round
cylinder 5.5 cm across (its flats), so the sim does not need the pads on the flats; the demo still lands them there.

## Real-world build

A 200 x 160 x 25 mm steel plate (or any rigid 25 mm plate) bolted to the bench, with an M36 x 4 threaded rod (stud)
screwed and locked into a tapped hole so 48 mm stands above the plate, its end chamfered; a zinc-plated M36 flat washer
(ISO 7089, 66 mm across, 5 mm thick) over it. An ISO 4032 M36 black-oxide hex nut (55 mm across flats, 31 mm tall).
Find each part's thread start (where the full thread begins) and mark it with a yellow paint dot on the stud's end and
on the nut's top face. For each trial, set the nut square on the stud's end with its dot half a turn from the stud's,
without turning it. Replace stud and nut if a trial cross-threads them.

## Licenses

All code here was written for this task; no third-party code is included. No third-party assets are used: every shape
is generated by `yamkit.py` and `twistkit.py`.
