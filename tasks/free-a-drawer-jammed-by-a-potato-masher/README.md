# Free a kitchen drawer jammed by a potato masher

The arms open a kitchen drawer jammed by a potato masher. Put away with the drawer part open, the masher ended up at
the back, under the counter, on its side with its fin head standing on one end: 102 mm tall, against 85 mm under the
cabinet's top rail. Pulled, the drawer runs out about 21.5 cm and stops hard where the masher's forked neck catches
the back of the rail. The arms are asked only:

> **"Open the kitchen drawer all the way, leaving everything in it."**

Pulling harder only presses the fork harder into the rail. The fix is to reach into the open part of the drawer, roll
the masher onto its fins by the grip (66 mm tall), then pull the drawer out. The jam is a loose utensil whose pose has
to be read from the scene.

`task.py` holds the scene, scorer and scripted demo; `yamkit.py`, `gripkit.py` and `yamgeom.py` are helpers. The
robot model and its collision hulls load at run time and are not redistributed.

## Requirements

- The yam_arena environment (Isaac Sim, the YAM two-arm embodiment, `inspect-robots`).

## Run and score a trial

The scorer (`task_score`) reads the drawer's slide position, the masher's pose, the masher-cabinet contact force
(PhysX) and every arm link's collision hulls each control step and scores the end of the trial. "Flat": the masher's highest point at most 83 mm above the
drawer's floor, 2 mm under the rail.

- **1.0:** the drawer out at least 295 mm (stop at 300 mm), the masher flat and wholly inside the box, every arm link
  at least 5 mm from the masher and the drawer's front and pull, all still (under 1 cm/s) for 1 s.
- **0.5:** the masher flat and the drawer out at least 280 mm, but not fully out, still held, or still moving.
- **0.25:** the masher flat, the drawer short of 280 mm.
- **0:** anything else, including a masher left tilted or wedged so it would catch the rail. Latched at 0 for the rest
  of the trial: the masher taken out of the drawer (its center of mass carried out past a wall or the front), even if
  put back; or damage: the masher pressed on the cabinet's frame harder than about 450 N for 0.5 s (PhysX contact
  force; the window filters solver spikes). 450 N is a low bound on the force that starts to bend a fork arm: nylon
  assumed to be nylon 66 (yield 55 MPa at 50% RH), the whole force taken across one arm's 12 x 8 mm section, held at
  the head and the neck, at its worst point.

Lifting or tilting the masher inside the drawer is allowed; only where it ends up counts. In the yank run (`yank`)
the drawer stopped at the snag with the masher still on end; the masher-rail force peaked at 27 N before the hand
slipped off the pull, far under 450 N, so a yank scores 0 by its end state.

Real trial, from the video and a look afterwards: the drawer at its stop, the masher lying on its fins inside it, the
hands off and everything still. 0 if the masher was carried out of the drawer (past its sides or front) at any time, or if afterwards the masher is bent or
cracked, the rail is dented or split, or the drawer is off its slides.

Dream-1, from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the kitchen drawer all the way, leaving everything in it." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir masher-dream
```

Scripted demo (open loop, fixed waypoints, no simulator state read):

1. The left hand pinches the bar pull, draws the drawer out to the snag (about 215 mm), eases it back to 185 mm and
   lets go.
2. The right hand takes the masher's grip from above, rolls it 40 degrees toward the drawer's middle (the head tips
   over past about 33 degrees), lets go and slides its fingers off along the grip. The head falls onto its fins.
3. The left takes the pull again and draws the drawer out to its stop. Both arms go home.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the kitchen drawer all the way, leaving everything in it." \
  --scorer task_score --max-steps 1100 --store-frames --log-dir masher-demo
```

Results: the scripted demo scores 1.0 (step 997); Dream-1 scored 0 (2,400 steps). Videos: media ID in task.json.

## Failure modes

Demo variants (`MASHER_VARIANT=`) and offline tests:

| Way to game it | What happens | Score |
|---|---|---|
| Do nothing (`nothing`) | The start meets no tier | 0 |
| Pull to the snag and stop | Masher not flat | 0 |
| Yank to the stop (`yank`) | Jams at the snag, the masher on end | 0 |
| Press the masher on the rail past about 450 N for 0.5 s (offline test only) | Damage | 0 (latched) |
| Lift the masher out (`lift`) | Taken out of the drawer | 0 (latched) |
| Raise the grip 13 cm, tilting it past 30 degrees, and lay it back (`tilt`) | Allowed: it stays in the drawer | 1.0 |
| Open, then shut again (`close`) | End state: flat, drawer short of 280 mm | 0.25 |
| Keep holding the pull (`hold`) | Hands not clear | 0.5 |
| Leave it tilted 11 degrees or more | Not flat | 0 |

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes). The scripted demo succeeds at step 997 (50 s); the trial ends
  once the 1 s hold is met.
- No randomness: the layout is fixed (`-E object_jitter=0`).

## Real-world build

A real OXO Nylon Potato Masher and a plywood mock of the cabinet section at the sizes in `task.json`, the drawer on
12 in full-extension slides.

## Sources and license

- OXO Good Grips Nylon Potato Masher, Target TCIN 80221526 (10.2 in long, 3.9 in wide, 0.3 lb, nylon):
  https://www.target.com/p/oxo-nylon-potato-masher/-/A-80221526. The head's depth (64 mm) and length, the fork and
  the grip's thickness are estimated from the listing's photos.
- Nylon 66 yield stress (82 MPa dry, 55 MPa conditioned, ISO 527-2): Zytel 101L NC010 datasheet,
  https://plastics.ulprospector.com/datasheet/e5880/zytel-101-nc010 and https://Protolabs.com/media/tcebrq5j/zytel-101l.pdf
- Friction: Engineering ToolBox, https://www.engineeringtoolbox.com/friction-coefficients-d_778.html (wood on wood
  0.25-0.5; nylon on nylon 0.15-0.25, on steel 0.4; rubber on dry concrete 0.6-0.85, for the soft grip). The model
  uses wood 0.40 / 0.30, nylon 0.35 / 0.30, soft grip 0.8 / 0.6.
- Task statement: O*NET OnLine, Food Preparation Workers 35-2021.00, https://www.onetonline.org/link/summary/35-2021.00
- All code here was written for this task; no third-party meshes, images or data files are included.
