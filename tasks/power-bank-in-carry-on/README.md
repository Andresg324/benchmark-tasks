# Pack the power bank and spare battery in the carry-on, not the checked bag

Two files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). `task.py` builds the scene,
scores it and holds the scripted demo; its docstrings explain every rule. `yamkit.py` spawns the modeled props: visible
meshes with simple box colliders underneath. It patches `yam_arena.scene` in memory at import and changes no files.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/power-bank-in-carry-on/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Pack the bags for the flight." --scorer task_score --max-steps 900

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the
end of the run. The rules and possible scores are in `task.json` (`scoring`) and in the docstrings of
`success()` and `score()`.

Only the end state is scored: a bag can be repacked freely until it is checked in, so nothing is remembered between steps.

## Runtime and seeds

- Demo: 793 control steps (40 s simulated).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).
