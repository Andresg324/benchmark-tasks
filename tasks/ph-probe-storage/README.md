# Put a pH electrode away in its storage solution

Two files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). `task.py` builds the scene,
scores it and holds the scripted demo; its docstrings explain every rule. `yamkit.py` spawns the modeled props: visible
meshes with simple box and cylinder colliders underneath, and see-through liquids with no colliders. It patches
`yam_arena.scene` in memory at import and changes no files.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/ph-probe-storage/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Put the pH probe away." --scorer task_score --max-steps 600

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the
end of the run. The rules and possible scores are in `task.json` (`scoring`) and in the docstrings of
`success()` and `score()`.

The scorer remembers what happened during the run: `success()` and `score()` are called after every control
step. A dip of at least 0.5 s in the DI water counts as the rinse, and a tip past 45 degrees sets a flag that nothing
later clears. Each scene reset by the simulator (counted by yamkit) starts a new history.

## Runtime and seeds

- Demo: 351 control steps (18 s simulated).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).
