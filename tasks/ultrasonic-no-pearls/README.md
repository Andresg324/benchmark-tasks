# Clean jewelry in an ultrasonic cleaner, keeping the pearls and the opal out

Two files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). `task.py` builds the scene,
scores it and holds the scripted demo; its docstrings explain every rule. `yamkit.py` spawns the modeled props: visible
meshes with simple box colliders underneath. It patches `yam_arena.scene` in memory at import and changes no files.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/ultrasonic-no-pearls/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Clean the jewelry in the ultrasonic cleaner." --scorer task_score --max-steps 600

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the
end of the run. The rules and possible scores are in `task.json` (`scoring`) and in the docstrings of
`success()` and `score()`.

With `REQUIRE_ARMS_HOME = True` (the setting at the top of `task.py`), a fully cleaned set scores 1.0 only once both
arms are back within 3 cm of their start pose and still, and 0.9 until then, so the bench is left clear for the
next job. The start pose is recorded at the trial's first step. Possible scores: 0, 0.5, 0.9 and 1.0.

The scorer remembers what happened during the run: `success()` and `score()` are called after every control
step. Putting the pearl bracelet or the opal ring into the solution, even briefly, or knocking either off the
table (below the table top or past its edges) sets a flag that nothing later clears. A piece's position in the
solution is worked out from its pose and modeled shape, not its bounding box. `yamkit.py` counts trials (it sees
the environment's step counter restart), and a new trial starts a new history.

## Runtime and seeds

- Demo: 548 control steps (27 s simulated), ending with the arm back in its start pose.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).
