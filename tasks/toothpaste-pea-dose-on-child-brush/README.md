# Put a pea-sized dab of toothpaste on a child's toothbrush

`task.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), `station` scene. A child's manual
toothbrush lies on its back, bristles up, on the robot's left, and a half-used Crest Kid's 4.2 oz toothpaste tube lies
on its side on the right, flip cap open. The robot gets the brush ready for a 4-year-old: one hand holds the brush
bristles-up, the other squeezes a pea-sized dab (0.15 to 0.35 g) onto the bristles, then the tube goes back down.
The helper modules: `yamkit.py` spawns the props (the brush, and the tube as an articulated body with four sprung
wall panels), `guardkit.py` checks gripper contact, and `squeezekit.py` is the paste flow model (see "The paste
model"). `test_task.py` checks the scorer offline: `python3 test_task.py` (or `python3 -m pytest -q test_task.py`).

Requirements: `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its
`setup.sh` (Isaac Lab-Arena `release/0.3.1`, `inspect-robots` 1.0.0, `inspect-robots-yam` 0.38.0), and
`source $ARENA_DIR/yam.env` in the shell.

## Run a trial

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/toothpaste-pea-dose-on-child-brush/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the toothbrush ready for a 4-year-old." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/toothpaste-demo
```

Model run (Dream-1): the same command with `--policy dream --max-steps 1200`.
Video: `inspect-robots video logs/toothpaste-demo/*.json`. The scorer (`task_score`) reports the score each step;
the trial's score is the one at its end.

## Why it matters

Children aged 3 to 6 should brush with no more than a pea-sized amount of fluoride toothpaste, about 0.25 g; under 3,
a smear the size of a grain of rice (CDC; ADA and AAP guidance). Over 38% of US children aged 3 to 6 use more than
that, half or full brush loads (CDC MMWR 2019;68(4):87-90, PMC6400578), and swallowed excess raises the risk of dental
fluorosis. Parents do this twice a day. The robotic difficulty: it is bimanual (one hand holds the brush bristles-up,
the other squeezes), and the dose is set by how far and for how long the tube is squeezed, with a paste that keeps
coming for about a second after the squeeze eases. Too much is the easy failure; landing the dab on a 20 x 9 mm field
is the other.

## Scoring

Grams on the bristles = mL x 1.30 g/mL (toothpaste's density; weigh 1 mL from a syringe to confirm).

| Score | Condition |
|---|---|
| 1.0 | the 0.75 state, the bristles within 45 degrees of facing up (held or set down), the tube lying on the counter (its middle under 3 cm up), released, both still (under 1 cm/s) for 1 s |
| 0.75 | 0.15 to 0.35 g on the bristle field (a pea-sized dab) and at most 0.05 g anywhere else |
| 0.5 | 0.15 to 0.35 g on the bristles, but more than 0.05 g off them (counter, head sides, neck, handle) |
| 0.25 | 0.05 to 0.15 g (a smear or rice grain: the under-3 dose) or 0.35 to 0.70 g (too much: a half load) |
| 0 | under 0.05 g or over 0.70 g (a full adult strip), or nothing done |

Turning the bristles past sideways (more than 90 degrees from up) with paste on them caps the score at 0.25 for the
rest of the trial: the dab can fall or smear. In simulation paste lands on the bristles only while they face within
60 degrees of up; squeezed onto a brush tipped further, it lands off. A hand counts as holding the tube when its own
shape (palm and fingers, opened as they are) touches the tube within 2 mm (guardkit), not by nearness.

The dose is measured in the brush's own frame, so the score is the same wherever the brush is held or laid: on the
counter, held up anywhere, turned or tilted, or moved after dosing (`test_placement_robust`).

**Real trial (outcome only, no force gauge).** Tools: a 200 g x 0.01 g pocket scale, a mm scale, a phone camera.
1. Before: weigh the clean, dry brush and the tube (cap open) on the 0.01 g scale.
2. After: photograph the brush head from above next to the mm scale (the dab's position and size: a pea dab is about
   5-6 mm across). Wipe any paste off the brush outside the bristle field (head sides, neck, handle) with a dry tissue
   and weigh the brush: the gain is the paste on the bristles. Weigh the tube unwiped: the loss is the paste
   dispensed. Off the bristles = dispensed minus on the bristles.
3. The bristle angle at the end, the tube's release and the 1 s hold from the video.

Tolerance: the scale reads to 0.01 g, so the 0.15/0.35 g band edges sit 15 and 35 counts above zero and the 0.05 g
off-bristle limit is 5 counts; a minute of drying loses well under 0.01 g. Every weight is independent of where
the brush and tube end up.

## The paste model

`squeezekit` (the house squeeze-bottle kit) with a toothpaste tube's numbers:

- The tube's round body is a rigid core with four wall panels on prismatic joints and springs (400 N/m each). A hand
  dispenses only by pushing panels in; their inward push is read from the joints every control step. A grip on the
  cap, shoulder or crimp squeezes nothing.
- Each panel displaces 1.0 mL per cm pushed. The 8 mL of air inside is compressed isothermally; the paste flows
  through the nozzle at 0.10 mL/s per kPa above a 0.7 kPa yield pressure (plug flow: 4 x 100 Pa yield stress x 9 mm
  nozzle / 5 mm bore, rounded). As paste leaves, the air re-expands and the flow dies away (about 0.8 s), so it keeps
  coming briefly after the squeeze stops; easing off draws air back in and stops it. A 0.2 mm holding grip makes
  under 0.7 kPa and dispenses nothing.
- The paste's own weight is left out (an open toothpaste tube held nozzle-down does not drip), and it feeds whatever
  way the nozzle points, short of straight up.
- It drops plumb from the nozzle. The plumb line is cut with the bristle plane and the hit point is stored in the
  brush's frame on a 1 mm grid of the 20 x 9 mm bristle field, so the dab moves with the brush. A cell holds a bead
  4.5 mm tall; more slumps into its neighbors, and slump over the field's edge counts as off the bristles. Paste that
  misses the field (the counter, the sink) is off the bristles.
- In the offline model the demo's squeeze (1.5 mm a side for 1.2 s, then 1.5 s at the holding grip) leaves 0.26 g;
  2.0 mm for 1.5 s leaves 0.39 g (too much); 0.9 mm for 0.6 s leaves 0.10 g (a smear).

**Calibrating the constants on the trial tube (sim only; the real-trial score never uses them).** The panel area
(1.0 mL/cm) and the air (8 mL) are estimates. No bench tools are needed to check them: hold the tube nozzle-down in
the YAM's right gripper (prepared as in Real-world build), command closings of 1.0, 1.5 and 2.0 mm a side for 1.2 s,
five times each, read the closing from the gripper's own position readout, and weigh what comes out on the 0.01 g
scale; set `AREA_ML_PER_CM` and `AIR_ML` so the model's dose matches the means.

**Forces (computed, not measured).** The demo's squeeze makes about 4 kPa inside: about 5 N on each 12 cm^2 panel plus
0.6 N of wall spring, far inside the gripper's 50 N grip. The brush weighs 14 g and the tube about 84 g.

## Real-world build

- Toothbrush: a child's manual toothbrush for ages 3-5 (e.g. Colgate Kids extra soft), about 15 cm long; modeled with
  a 95 x 16 x 12 mm handle, 30 mm neck, 26 x 12 mm head and a 20 x 9 mm bristle field 9 mm tall. Measure the trial
  brush; it lies on its back, bristles up, along x with the head away from the robot, at a tape mark.
- Toothpaste: Crest Kid's Cavity Protection fluoride toothpaste, 4.2 oz (119 g) laminate tube with a flip cap (sold in
  this size, e.g. https://crest.com/en-us/oral-care-products/toothpaste/disney-raya-bubblegum-toothpaste),
  modeled with a 34 mm round body 10 cm long, a 46 mm crimped end, a 30 mm cap and a 5 mm nozzle (15.0 cm overall;
  measure the trial tube). Fill level: 50 to 70 g of paste left (gross weight minus the 12 g empty tube, weighed); a
  new tube is brought to this by squeezing 59 g into a cup. Before each trial roll it flat from the crimp up to the
  paste, wipe the nozzle flush, leave the cap open and let it lie 1 min (the laminate springs back), then lay it on
  its side at its tape mark, nozzle toward the robot.
- Counter: the station table (a bathroom counter stand-in), wiped dry.
- Safety: a toothbrush and toothpaste; nothing sharp, hot or heavy. The brush is not used in a mouth.

## Runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (1 minute).
- Scripted demo: the plan lasts 31.1 s (622 steps); its score is PENDING (no sim run yet). The left hand lifts the
  brush by its handle and holds it level, bristles up, 10 cm over the counter; the right hand lifts the tube by its
  body with a light holding grip, turns it nozzle-down (10 degrees off plumb) 4 mm over the bristles' middle,
  squeezes 1.5 mm a side for 1.2 s, eases to the holding grip for 1.5 s, lifts off and lays the tube back; then the
  left hand lays the brush back. The demo is open loop.
- No randomness: the layout is fixed (`-E object_jitter=0`).

## How the file works

- **The tube** is a yamkit `Mechanism` (a loose articulated body, `squeezekit.free_body`) with four prismatic wall
  panels whose spring law (`squeezekit.Wall`) also feels the live pressure inside.
- **Paste.** After every control step, `_update()` reads the brush's pose, the tube's pose and its four panel pushes,
  and steps a `BrushPaste` (squeezekit's `Sauce` landing on the brush's own bristle grid). `success()` and `score()`
  share one update per step. A new trial (yamkit's episode counter) starts a fresh tube and a clean brush.
- **Visible paste.** Cells with paste are drawn as blue tiles on the bristles with Isaac Lab's
  `VisualizationMarkers`, so camera-based policies see the dab. The tiles are only a picture: if drawing fails, they
  turn off and scoring carries on.
