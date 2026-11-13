# Scrub the IV connector's hub for 15 seconds before attaching the flush syringe

Three files for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). `task.py` builds the scene,
scores it and holds the scripted demo; its docstrings explain every rule. `yamkit.py` spawns the modeled props: visible
meshes with simple box colliders underneath. It patches `yam_arena.scene` in memory at import and changes no files. The
syringe is a loose yamkit `Mechanism`: its barrel is a free body and its plunger a link on a 6 cm slide along the
barrel's axis, damped so that it stays where it is pushed. `task.py` moves the articulation's root onto the barrel and
gives the slide a PhysX drive for that damping (`_free_root`). `guardkit.py` (trimmed to its one check) judges the
grip on the pad by contact: both fingers' own shapes, from the gripper's collision meshes at their present opening,
touching the pad, with the opening stopped by its 3.5 cm width.

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/scrub-the-hub/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Flush the IV line on the training arm." --scorer task_score --max-steps 1500

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

## Scoring

`--scorer task_score` gives 1.0 once `success()` has held for 1 s; otherwise the trial scores `score()` at the
end of the run. The rules and possible scores are in `task.json` (`scoring`) and in the docstrings of
`success()` and `score()`.

The scorer remembers what happened during the run: `success()` and `score()` are called after every control
step. It adds up the steps the pad spends rubbing the connector's access face, and the syringe's barrel or tip coming
within 2 mm of that face before 15 s of scrubbing sets a flag that nothing later clears. Seating the syringe means its
luer tip touching the access face: the tip end from 0.5 mm short of the face (the resting gap between two parts) to
pressed 3 mm in (the colliders' give), within 1.5 mm of the connector's axis and lined up within 5 degrees, for 0.5 s
in a row. A tip hovering off the face, however close, is not seated. The sim's connector is solid, so the tip pressed
square on the port stands in for the luer slipped into the valve; a real trial checks it with a gentle tug on the
barrel. Seating and a 3 cm push of the plunger while the syringe stays seated are latched once reached.

`python test_task.py` replays step-by-step histories offline (no simulator), including rubbing the connector's top
instead of its face, a leaning pad whose far corner passes the face while the face itself is 4 mm short, a syringe
left 8 mm off the hub, a tip hovering 0.8 or 1.5 mm off the port (not seated), a tip touching or pressed 1 mm in
(seated), and a plunger pushed before the syringe is seated. For the grip it checks a hand holding the pad
by its far end (counts), a hand closed to the pad's width with its fingertips 5 mm off it, one finger on the pad, an
open hand around it and an empty fist on it (none count). Each scene reset by the simulator (counted by
yamkit) starts a new history.

## Real-world build

An IV training arm (a nursing-skills forearm with a taped-in practice catheter) on a padded armrest, its clear
extension line run around the arm's right side to a split-septum needleless connector. Tape the connector to the bench
along y at the position in the setup, its open end facing the robot's right. Large 70% isopropyl alcohol prep pads in
foil packets, opened and folded once to about 6.5 x 3.5 cm, and empty 10 mL luer syringes with mL marks, plunger drawn
back to 10 mL, on a 15 x 15 cm tray. The line is not connected to anyone and the syringe is empty. Before each trial,
put a dot of water-soluble ink on the connector's access face. Score with the video, a stopwatch for the scrub time, a
gentle tug on the barrel for seating and the barrel's mL marks for the push.

## Category

Impact on jobs: registered nurses (more than 3 million US jobs) and licensed practical nurses (more than 600,000)
access and flush IV lines many times a shift.

## Runtime and seeds

- Demo: 1,056 control steps (53 s simulated), including a 20 s scrub; it ends on success.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).
