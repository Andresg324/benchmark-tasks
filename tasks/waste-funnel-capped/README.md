# Close a hazardous-waste jug: funnel out and onto its drip tray, cap back on

A lab's waste station at the end of the day. The funnel used to add waste is still standing in the jug's neck, and the
jug's screw cap lies on the bench. "You're done adding waste for today. Tidy up the waste station." means taking the
funnel out, setting it on its drip tray, and putting the cap back on the jug. Under EPA rules a waste container stays
closed except while waste is being added, and a funnel left in the neck counts as open. A cap set on top of the funnel
closes nothing.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `yamkit.py` | Helper that spawns the modeled props (jug, funnel, cap, trays) from generated USD and gives the scorer the grippers' positions. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline scorer checks with hand-made states (near misses included). No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/waste-funnel-capped/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "You're done adding waste for today. Tidy up the waste station." --scorer task_score --max-steps 700

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/waste-funnel-capped/test_task.py` (prints `all pass`).

## The demo

Right arm only: everything is on its side of the bench, clear of the middle where its reach is least exact.

1. Grips the funnel's tab across its width, lifts the funnel straight up until the stem clears the neck, carries it
   to the drip tray, lowers the stem into the holder's ring and lets go.
2. Picks the cap off the bench, carries it over the jug, lowers its skirt over the neck and presses it onto the rim,
   lets go and goes back to the exact pose it started in (`kit.Home`).

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the end of
the run.

| Outcome at the end | Score |
|---|---|
| Funnel still in the jug's neck (with or without a cap on top of it) | 0 |
| Cap on the neck and the funnel on the drip tray (seated in its holder, or lying inside the tray's walls), arms home | 1.0 |
| The same, with the arms not back home and still | 0.9 |
| Cap on the neck, funnel set down on the bare bench | 0.5 |
| Funnel on the drip tray, cap not on (on the bench, crooked or proud on the rim, upside down) | 0.25 |
| Anything else, including a funnel knocked off the table or draped over a tray's wall | 0 |

"Cap on" means centered on the neck within 9 mm, its top resting on the rim (height within 4 mm), upright within 10
degrees and still. A part still held by a closed gripper earns nothing. Both arms must also be back where they started
(each tool point within 3 cm of its position at the trial's first step) and still, so the workspace is clear for the
next person or job. The job done with the arms elsewhere scores 0.9. This is the `REQUIRE_ARMS_HOME` setting at the
top of `task.py`. Possible scores: 0, 0.25, 0.5, 0.9, 1.0. The score depends only on the end state, since parts may be set down anywhere
along the way. In a real trial: look at the neck and the drip tray, and try to lift the cap straight off.

## Runtime and seeds

- Demo: 527 control steps (26 s simulated).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

Close to real size. The jug and both trays are fixed; the funnel and cap are rigid props that collide as simple
blocks. The jug's neck, the cap's skirt and the funnel holder are square rings of blocks, so the stem really drops
into the neck and the holder and the cap really fits over the neck. The cap is set on the neck, not screwed: a seated
cap counts as closed. No liquid or vapor is simulated. The funnel's grip tab is larger than on most real funnels so the
gripper can hold it.

## Real-world build

A 1 to 2 L HDPE waste jug with a screw cap and a printed hazardous-waste label, empty or holding water; a 75 to 100 mm
polypropylene lab funnel with a stem that fits the neck (glue a small block to its rim if it has no tab); a
secondary-containment tray about 17 cm square; a small drip tray with a funnel holder (a ring clamp on a short rod, or
a funnel rack). Tape the jug's and trays' footprints. In each trial, stand the funnel in the neck with its tab toward the arms and set the cap open
side down at its mark.

## Licenses

All code here was written for this task; no third-party code is included. No third-party assets are used: every shape is generated by `yamkit.py`.
