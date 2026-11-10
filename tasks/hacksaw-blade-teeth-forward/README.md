# hacksaw-blade-teeth-forward

Fit a new 12-inch blade to a hacksaw clamped flat in a bench vise: both holes over the frame's pins with the teeth
pointing away from the handle (the saw cuts on the push stroke), then tension it with the wing nut to 1 to 2.5 turns
past snug and let go. Instruction: "Fit the new blade to the hacksaw."

## Spec

| Object | Type, size, material | Position (cm; x forward, y left, table z 0) |
|---|---|---|
| Hacksaw frame | 12-inch frame, red enameled steel channel 2.2 x 1.2 cm, 10 cm throat, black pistol grip; fixed | lying flat, blade plane at z 9, blade line x 31, pins at y -5.7 and 24.7 |
| Wing nut | cast steel, wings 4.6 cm tip to tip, 10 mm thick, on an M6 x 1 tension screw (axis along y) | x 31, y -11.0, z 9; starts half a turn short of snug |
| Bench vise | 4-inch jaws, blue cast iron; fixed | jaws at x 40 to 44, centered y 9.5, clamping the back bar |
| New blade | 12-inch x 1/2-inch bi-metal, 0.65 mm thick, 18 TPI, blue with white arrows on both faces; 40 g loose prop | middle at x 21.5, y 7.0, z 3.2, 30 degrees askew, arrow toward +y |
| Two blocks | hardwood, 4.5 x 4 x 3 cm; fixed | x 18.0, y 0.9 and x 25.0, y 13.1 |

The trap: the blade starts with its arrow pointing toward the handle's side. Carried straight across, it ends up
backwards (teeth toward the handle and toward the back bar); it must be turned about 150 degrees as it is carried.

Scoring (`score()` in task.py; all thresholds with their reasons are in the constants block and task.json):

| Outcome | Score |
|---|---|
| Blade not on both pins (each hole within 3.25 mm of a pin, seated within 2 mm, flat within 5 degrees) | 0 |
| On the pins but the wrong way round (arrow or toothed edge more than 20 degrees off) | 0 |
| Blade or wing nut still held at the end | 0 |
| Fitted right, nut never past snug | 0.25 |
| Nut went past snug with the blade not on both pins (fitted to a tensioned frame), or more than 90 degrees of tightening without a grip rolling with the wings | 0 |
| Fitted right, under 1 turn past snug (slack) | 0.5 |
| Fitted right, over 2.5 turns past snug at any point, even if backed off after (overtight) | 0.25 |
| Fitted right, 1 to 2.5 turns past snug, let go, both arms home and still | 1.0 (0.9 if not home) |

The scorer keeps a history (reset on every scene reset through `kit.episode()`): the blade must be on both pins while
the nut is still slack and stay on them while the nut goes past snug; backing the nut off to slack re-arms the fit.
The nut's grip accounting is twistkit's (`gripped_deg`, `unheld_deg`), with the roll budget signed (rolling back while
held takes credit back) and measured about the nut's own horizontal axis.

## Real-world build

Any 12-inch hacksaw frame with a front wing nut (for example a Stanley or Bahco frame), a 4-inch bench vise bolted
so the frame lies flat 9 cm above the bench, a 12-inch 18 TPI bi-metal blade and two scrap hardwood blocks. Mark the
blocks' footprints and the blade's start pose with tape. Mark one wing of the nut with paint and note where the blade
goes snug before trials; to check a trial, count the turns past that mark, look at the arrow and see both pins
through the holes.

## Files

- `task.py`: scene, scorer and scripted demo. `twistkit.py` and `yamkit.py`: local kit copies. In twistkit the
  wrist-roll credit is now measured about the cap's own axis (`_grip_yaw(state, axis)`), so a horizontal wing nut
  turned by an end-on grip is credited correctly; worth merging into the master.
- `test_task.py`: offline scorer checks (`python tasks/hacksaw-blade-teeth-forward/test_task.py` prints `all pass`).

## Run

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/hacksaw-blade-teeth-forward/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Fit the new blade to the hacksaw." --scorer task_score --max-steps 2400 --log-dir ~/yam-eval

Set `HACKSAW_DEBUG=1` to print the blade's pose and the nut's state every step. One trial takes about 2 minutes of
sim time; there is no randomness (object_jitter=0).
