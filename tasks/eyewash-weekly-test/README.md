# Weekly activation test of a plumbed eyewash station

A bench-mounted emergency eyewash stands on the table: a steel bowl with two spray heads, each under a flip-top dust
cover, and a push flag on a quarter-turn stay-open ball valve. "Do the weekly test on the eyewash station." means what
ANSI/ISEA Z358.1 asks of whoever runs the weekly check: open the valve and let it run until the water comes out clear,
then shut it and put the dust covers back on. A quick press that shuts the valve again at once, leaving it running, or
leaving the covers off are the tempting wrong answers.

## Spec (written before the code)

| | |
|---|---|
| Instruction | "Do the weekly test on the eyewash station." (the rule is never stated) |
| Ability | Run a stay-open valve for a sustained flush (not a momentary press), then shut it and restore the unit |
| Scene | One fixed mechanism: housing, bowl, two heads, two hinged dust covers, push flag on a vertical valve stem, two water jets (rusty, then clear) that appear only while the valve is open |
| Flow | Water flows while the flag is turned 80 degrees or more (of 90). The covers pop open while it flows. The water runs rusty for its first 10 s without a break, then clear |
| Flush | The valve open without a break for 240 control steps (12 s at 20 Hz), so 2 s past the water running clear |
| Traps | Doing nothing: 0. A momentary press (open, then shut before 12 s): 0. Flushed but valve left open: 0.25. Flushed and shut but a cover left off, or a gripper still on the flag or a cover: 0.5. All done but arms not home: 0.9 |
| Scaling | About 0.7 size. The flush is scaled from the usual 3 minutes (or until clear) to 12 s |

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `yamkit.py` | Helper that spawns the eyewash as an articulation, gives the scorer the joints and the grippers, and adds demo waypoints. This copy adds linked parts: the dust covers are pushed open, and the water jets slide into view, while the valve is open. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline scorer checks: step-by-step histories of the valve, the covers and the grippers, traps included. No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/eyewash-weekly-test/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Do the weekly test on the eyewash station." --scorer task_score --max-steps 1200

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/eyewash-weekly-test/test_task.py` (prints `all pass`).

## The demo

1. Right arm: comes down beside the push flag and pushes it back along its arc, a quarter turn, until it stops. The
   valve stays open on its own; the covers pop open and rusty water sprays from both heads, then runs clear.
2. The right arm waits beside the unit while the water runs, about 14 s in all.
3. It pushes the flag back to closed from the other side. The water stops.
4. It pushes the right dust cover forward over its head from behind; the cover falls shut. Then it goes home.
5. The left arm does the same for the left cover and goes home. Both arms hold still at home.

## Scoring

The scorer watches the whole run (`success()` records it every step).

| Outcome | Score |
|---|---|
| The valve was never open (flag at 80 degrees or more) for 240 control steps in a row (12 s at 20 Hz) | 0 |
| Flushed, but the flag not back within 10 degrees of closed at the end (water still running or dribbling) | 0.25 |
| Flushed and shut, but a dust cover more than 10 degrees open | 0.5 |
| Flushed, shut and both covers on, but a gripper's tool point still within 4.5 cm of the flag's or a cover's center | 0.5 |
| Flushed, shut, covers on, hands off, but the arms not home and still | 0.9 |
| All of it, with both arms home and still | 1.0 |

Possible scores: 0, 0.25, 0.5, 0.9, 1.0. The arms count as home with each tool point within 3 cm of where it was at
the trial's first scored step, and still: they are put back so the workspace is clear for the next person or job.
Setting `REQUIRE_ARMS_HOME = False` at the top of `task.py` turns that part off. `success()` is a full score held for
1 s. Each scene reset by the simulator (counted by yamkit) starts a new history.

In a real trial: time the valve's open spell with a stopwatch from the video (12 s, or 3 minutes on a full-length
test), then check the flag is back to closed and both covers sit on their heads.

## Runtime and seeds

- Demo: 813 control steps (41 s simulated).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

About 0.7 scale. The unit is fixed to the table. The flag turns a quarter turn on a vertical stem and stays where it
is left, as a stay-open ball valve does. No water is simulated: two translucent streams rise from the heads and arc toward
the user while the valve is open, tinted rust for the first 10 s and clear after (the streams are visual only and
cannot be touched). The water's push on the covers is a torque that opens
them while it flows; once it stops they stay where they are, and one pushed past upright falls shut under its own
weight. The flush is scaled down from the usual 3 minutes (or until clear) to 12 s.

## Real-world build

A bench- or deck-mounted plumbed eyewash with a stay-open ball valve, a push flag and flip-top dust covers (or a
portable test rig on a utility sink with a quarter-turn ball valve). Put a bucket or sink under it. Mark the flag's
open and closed positions with tape.

## Licenses

All code here was written for this task; no third-party code is included. No third-party assets are used: every shape
is generated by `yamkit.py`.
