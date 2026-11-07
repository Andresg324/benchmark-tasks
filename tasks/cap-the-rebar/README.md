# Cap upright rebar with impalement-rated caps

Two files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). `task.py` builds the scene,
scores it and holds the scripted demo; its docstrings explain every rule. `yamkit.py` spawns the modeled props: visible
meshes with simple box and cylinder colliders underneath, and adds the grippers' positions and openings to the scored
states. It patches `yam_arena.scene` in memory at import and changes no files.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/cap-the-rebar/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Make the rebar safe before the crew walks through." --scorer task_score --max-steps 800

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the
end of the run. The rules and possible scores are in `task.json` (`scoring`) and in the docstrings of
`success()` and `score()`.

The score depends only on the final state: which caps are seated on which bars.

## Runtime and seeds

- Demo: 603 control steps (30 s simulated); it ends on success, held for 1 s.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).
