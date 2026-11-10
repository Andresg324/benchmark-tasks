# Score a tile in one pass on a manual tile cutter, then snap it

A blue glazed wall tile lies on a manual tile cutter, squared against the side fence. "Cut the tile in half on the
tile cutter." means pushing the scoring wheel's carriage along the rails once, from the near edge of the tile to the
far edge, and then pressing the breaker handle down so its foot snaps the tile along the score. A tiler never runs the
wheel back over the score (it chips the glaze and the wheel wanders out of its track) and never presses the breaker
before the score is complete (the tile cracks at random). After the snap the handle is let go and its spring lifts it.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings and comments give the reason for every threshold. |
| `yamkit.py` | Helper that spawns the modeled cutter (an articulation with a sliding carriage and a sprung, hinged handle) and gives the scorer the joint states and the grippers. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline scorer checks with hand-made carriage and handle trajectories (near misses and loopholes included). No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/tile-cutter-score-once-then-snap/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Cut the tile in half on the tile cutter." --scorer task_score --max-steps 600

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/tile-cutter-score-once-then-snap/test_task.py` (prints `all pass`).

## The demo

1. Right arm: lowers its closed fingers behind the carriage's push knob and pushes the carriage in one smooth 4.5 s
   stroke to the far stop (the single pass), lifts straight up off the knob and goes home.
2. Left arm, after that: sets its closed fingers on top of the handle's grip and presses it down along its arc until
   the foot bears on the glaze (the snap), holds for 1 s, lifts off so the spring raises the handle, and goes home.
3. Both arms hold at home.

## Spec

| Part | Real-world reference | Model |
|---|---|---|
| Tile | 6-inch (15.2 cm) glazed ceramic wall tile, 8 mm thick | fixed box, 15.2 x 15.2 x 0.8 cm, x 22.5 to 37.7 cm, cut line at y 0 |
| Base | manual rail tile cutter's base with foam pads, breaking ridge and side fence | fixed, 30 x 22 cm, foam either side of a steel ridge, red fence at y -8.2 cm |
| Carriage | scoring carriage on twin rails with a 22 mm carbide wheel | prismatic link along x, 200 g, travel 18.2 cm (wheel from 1.5 cm short of the tile to 1.5 cm past it), drag 3 N per m/s |
| Push knob | the carriage's handle | 2.5 x 2.5 x 5 cm upright on the carriage's right side |
| Breaker handle | handle with a rubber-shod breaker foot | revolute link about y at x 42.4 cm, 9 cm up; 100 g; return spring 0.6 N m/rad; raised 31.5 degrees, lower stop 1.7 degrees past level |
| Drawn score line and split | the scored track in the glaze; the two halves after the break | visual only: six 2.5 cm lengths of a 4 mm near-black line on the glaze at y 0, each shown once the wheel has crossed all of it; after the snap the tile is drawn as two halves with a dark 6 mm gap (it stays one fixed collider) |

| Rule | Threshold | Why |
|---|---|---|
| Score line | the wheel's travel over the tile in 1 mm cells | 1 mm is about the width of a carbide wheel's track in the glaze |
| New stroke | carriage rolls back more than 3 mm | 3 mm is the play of a hand pausing on the carriage; any longer roll-back re-runs the wheel over the score |
| Second pass | any cell scored by two strokes | a second run over the score chips the glaze and the wheel leaves its first track |
| Full pass | every cell, near edge to far edge, scored by one stroke | the score must reach both edges or the break runs off the line at the unscored end |
| Snap | handle pressed to level (foot on the glaze) | on the ridge, a scored tile breaks under the first firm press of the foot |
| Carriage parked | wheel 8 mm or more past the far edge | the carriage at its far stop (1.5 cm past), off the tile, where it sits for the snap |
| Handle released | back up within 5 degrees of its stop | let go, the spring lifts it; it can only stay lower if something holds it down |
| Hands off | no tool point within 4 cm of the grip or the knob | 4 cm clears the 2.6 cm grip and the knob by more than a finger pad |
| Arms home | each tool point within 3 cm of its start, still | the workspace is clear for the next cut |

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the end of
the run.

| Outcome | Score |
|---|---|
| One clean pass, then the snap, handle let go and back up, nothing held, both arms home and still | 1.0 |
| The same, arms not back home and still | 0.9 |
| One clean pass edge to edge, carriage at rest at its far stop, no snap, nothing held | 0.25 |
| A second pass over the score at any time (irreversible) | 0 |
| The handle pressed to the snap before one full pass (irreversible) | 0 |
| No full pass, handle still pressed or held, or a gripper at the grip or knob at the end | 0 |

Possible scores: 0, 0.25, 0.9 and 1.0. Set `REQUIRE_ARMS_HOME = False` at the top of `task.py` to score the job alone.
The history (strokes, snap) is kept per trial and cleared on every scene reset through `kit.episode()`.

In a real trial: put tape marks on the rail where the wheel crosses the tile's edges, review the video for any
roll-back of the carriage between the marks and for the order of pass and snap, and check that the break follows a
single clean score line.

## Real-world build

A 20-inch (50 cm) manual rail tile cutter with a breaker handle and side fence (any common brand), clamped to the
table so its rails run away from the robot; 6-inch glazed ceramic wall tiles, 8 mm thick. Fit a short upright knob to
the carriage's handle if the gripper can't push the stock one. Mark the tile's outline on the foam with tape. Wear eye
protection near the snap.

## Runtime and seeds

- Demo: 443 control steps (22 s simulated).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

About 0.6 scale for the cutter, the tile full size. The carriage is a slide with a speed-opposing drag; the handle a
hinge with a return spring and a lower stop. Nothing is cut: the score line is the wheel's recorded travel, the
wheel's downward pressure is assumed, and the snap is the foot reaching the glaze. The score line on camera is drawn
from the recorded travel, 4 mm wide so the top camera sees it (a real score is a hairline), and the break is drawn,
not simulated: after the snap the tile shows as two halves with a 6 mm gap, but it stays one fixed piece. On a real
cutter the handle rides the carriage; here it is hinged at the far end, where the carriage sits for the snap.

## Licenses

All code here was written for this task; no third-party code or assets are included. Every shape is generated by
`yamkit.py`.
