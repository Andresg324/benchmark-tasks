# Fit a spin-on oil filter by hand to gasket contact, then to 18 N·m with a cap wrench on a ratchet, not loose or cranked

A new spin-on oil filter (Kawasaki 49065-0710, the Teryx/Teryx4 filter) has been started on the threaded nipple of a
remote oil filter mount (a filter head) bolted to the bench, pointing up, and stops three quarters of a turn short of
its gasket touching the head. Beside it lies a 65 mm, 14-flute cap-type filter wrench (3/8-inch drive, on an adapter) on a 24-inch ratchet. "Fit the new
oil filter." means the engine maker's service procedure, which mechanics follow at every oil change: spin it on by hand
until the gasket contacts the head, then tighten it to the specified torque, 18 N·m for this filter (about three
quarters of a turn past contact, the turn filter makers print), with the wrench, then take the wrench off. Stopping at
contact (it feels tight already) is the tempting wrong answer: the filter can work loose under engine vibration and
leak. Cranking it on well past a full turn can crush the gasket, which can roll out of its groove and blow out under oil
pressure, and the filter may not come off at the next change.

## Spec

| Item | What it is |
|---|---|
| Scene | One fixed filter head (die-cast aluminum, 7.5 x 9 x 3.4 cm block with a 7.2 cm round gasket land on top, its two side hose ports plugged), the filter's axis at x 30, y 30 cm; a black spin-on filter, 6.6 cm across and 7 cm tall, on its M20 x 1.5 nipple; the cap wrench on its 24-inch ratchet lying on the bench beside the head, cap down |
| Start | Filter 270 degrees (three quarters of a turn) short of gasket contact, about four turns onto the thread (five at contact); its index stripe faces left |
| Instruction | "Fit the new oil filter." |
| Unstated rule | By hand to gasket contact, then to the engine maker's torque with the filter wrench (about three quarters of a turn more): not left at contact, not cranked on; the wrench taken off |
| Core mechanism | A modeled thread (1.5 mm per turn) whose resistance past gasket contact rises to 18 N·m at three quarters of a turn; a hand on the can slips at about 2.5 N·m, so the rest is done with a modeled cap wrench and ratchet, swung by both hands |
| Category | Impact on jobs: automotive service technicians and mechanics (O*NET 49-3023.00; 825,800 US jobs in 2025, BLS Occupational Outlook Handbook) replace oil filters as regular work. O*NET lists the task "Perform routine and scheduled maintenance services, such as oil changes, lubrications, and tune-ups", and the BLS duties list "changing oil"; a new filter is normally fitted at each oil change |
| Trap | Stopping at contact or a little past it (0 or 0.25); over-tightening past a turn (0.5) or past 1.25 turns (0, irreversible) |
| Score | Degrees past contact at the end, in bands with reasons (below); the turning past contact must come from a holding hand's own wrist roll or from the wrench while a hand holds it; let go, the wrench laid on the bench |

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `guardkit.py` | The task's helper module for contact checks: the grippers' own shapes (boxes fitted to the YAM's collision meshes) and the 2 mm contact test used to decide whether a hand is still on the filter. |
| `task.py` (wrench) | The cap wrench and ratchet are a second articulation on the bench (lift, turn about the filter's axis, two slides) with laws in `task.py`: lying on the bench it stays put; lifted it moves freely; centered on the can and down it couples to the filter through the ratchet (`ToolModel`, `tool_step`). |
| `twistkit.py` | Threaded parts: the filter is the "cap" of an articulation (a slide along the nipple coupled to a hinge, 1.5 mm per turn), with a seat (gasket contact), a gasket friction growing with the angle past it, a hard stop 540 degrees past it, and "off" five turns back. Gives the scorer the filter's angle from contact and which gripper holds it, and builds the demo's regrasp loop. Built on `yamkit.py`. |
| `yamkit.py` | Helper that spawns the modeled props and articulations, and gives the scorer the grippers. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline scorer checks: step-by-step histories of the thread and the grippers, near misses included. No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/oil-filter-hand-tight-plus/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Fit the new oil filter." --scorer task_score --max-steps 2400

Model run (Dream-1): the same command with `--policy dream`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/oil-filter-hand-tight-plus/test_task.py` (prints `all pass`).

## The demo

The demo is closed loop: it senses the filter as a mechanic feels it. After every control step it reads the filter's
angle past gasket contact from the simulator (the thread's hinge, the same reading the scorer uses), and each stroke is
worked out from that reading when the stroke starts.

By hand. The left arm opens over the filter, comes down with its fingers around the top of the can and spins it on
clockwise (seen from above) in short regrip strokes: close, roll 90 degrees in 15-degree steps (sinking with the
thread), open, roll back, close again. After each stroke it checks how far that stroke turned the filter. There is no
set number of strokes. While the filter runs on freely, each stroke turns it nearly the full 90 degrees. Past gasket
contact the gasket's resistance rises until the pads slip on the can, at about 2.5 N·m (about 40 degrees past contact
in this model). The first stroke that turns the filter less than 10 degrees means the filter is hand-tight: the hand
lets go and lifts clear (at most 8 strokes). The right arm waits at home until then.

With the wrench. Both hands take the wrench's handle where it lies (HANDS_TEXT), lift it, carry the cap over the
filter and lower it onto the can. They swing the handle clockwise in strokes of up to 30 degrees from 280 degrees,
ratcheting back between them without letting go. Each stroke is sized from the filter's angle when it starts: it aims
at 280 degrees past contact, plus the few degrees the wrench flexes before the filter moves. The hands push with what
the gasket takes at each point plus 1 N·m, each hand taking a share in proportion to what its arm can push there. Each
hand is aimed past the handle by its arm's give (J K^-1 J^T from the servos' stiffness; while carrying the wrench, a
further 7-8 mm that the sim showed the load needs). Each also bears down on the handle with 3 N plus 0.6 N per N·m of
the stroke's torque, so the cap stays on the flutes. After each stroke the demo reads the angle again. Once the filter
is 270 degrees past contact, where the gasket takes Kawasaki's 18 N·m, the strokes stop. The hands ease off, lift the
wrench off the can, set it on the bench beside the head, let go and go home. Result on this version: RESULT_TEXT.

## Scoring

Over the whole run (`success()` records it every step), the filter's angle is measured from gasket
contact (the seat), positive when tightened past it; it starts at -270.

| Outcome | Score |
|---|---|
| The filter came off its thread at any point (unscrewed 4.25 turns from its start, five back from contact), even if spun back on: the fit is undone, and starting it again risks cross-threading | 0, irreversible |
| Tightened more than 450 degrees (1.25 turns) past contact at any point, even if backed off: the gasket is over-compressed and can roll out (angle as proxy; the real gasket is checked afterwards) | 0, irreversible |
| Ends under 90 degrees past contact (not touching, at contact, or under a quarter turn past), including doing nothing: it can work loose under vibration and leak | 0 |
| More than 45 degrees of the turning past contact done neither by a holding hand's wrist nor by the wrench while a hand held it (knocked, swiped): a spin has no control of where it stops | 0 |
| 90 to 180 degrees past contact: sealing, but at most two thirds of the makers' figure, so it can still loosen | 0.25 |
| 180 to 240 (close to the figure but short), or 360 to 450, or ever past 360 (past the most any maker allows, hard to remove by hand, not crushed) | 0.5 |
| 240 to 360 and never past 360, a hand still on the filter, or the wrench still on the filter or in a hand: not finished | 0.5 |
| 240 to 360, let go, the wrench laid on the bench: Kawasaki's 18 N·m is three quarters of a turn past contact on this model, and the makers print three quarters to one turn | 1.0 |

Where the arms end is not scored. Possible scores: 0, 0.25, 0.5, 1.0.

Who turned it. A gripper holds the filter (`twistkit.held`) when its fingers are closed on the can, its opening from 15
% of full travel tighter than the can's 6.6 cm (the pads pressing in) to 3 % looser (2.8 mm of air in all, about the
pads' compliance), so an empty fist or a hand hovering loosely around the can does not count, with its tool point
within 1.5 cm of the can's axis anywhere from 1 cm above the gasket face to 1.5 cm above the top (the pad tips sit
about 1.3 cm below the tool point, so that is every grasp height with the pads on the can). Each step, a gripper holding the filter at either end of the step (so the grasp and release
steps count) adds its own wrist roll, clockwise seen from above, to a budget (kept at most 30 degrees ahead of the filter, two
15-degree steps of the filter catching up late; more is the pads slipping on the can), and new ground the filter gains past
contact (beyond the furthest it has been) is credited from that budget; whatever the holding wrist did not turn is
booked as unheld, and turning back and forth over the same angles counts once. More than 45 degrees unheld scores 0:
that slack covers the filter settling as the hand lets go, and a swipe, a knock or the other arm turning it would add far
more. Each hand keeps its own budget, so a second hand near the can, or on it steadying or helping, never voids the
holding hand's turning; with both hands on the can the larger roll is credited (and used up from both). A swipe is
still caught: whatever turning no holding wrist rolled is unheld, wherever the other hand is.

The wrench. Turning the wrench does on the filter (the model's record of it driving the filter on) counts as a
hand's only at steps when a hand holds the wrench: fingers under 60 % open with the tool point within 5 cm of the
handle's axis. The wrench lies on the bench when its lift is within 5 mm of resting there; a 1.0 needs it there and no
hand holding it.

A hand is still on the filter at the end when it holds the can (as above) or when its palm or fingers touch the can:
the gripper's own shape (boxes fitted to the YAM's collision meshes, at the fingers' present opening) within 2 mm of the
can, from its gasket face to its top (2 mm is the solver's resting gap plus the boxes' fit to the meshes). That caps the
score at 0.5, since the job ends with the filter let go: a hand still on it can still turn it either way, so the angle
is not final. A hand hovering beside or over the can, open or closed, is not on it, however close it comes short of
2 mm. A real trial checks the last frames for a pad or finger still on the can.

`success()` is a full score held for 1 s. Each scene reset by the simulator (counted by yamkit) starts a new history.

Why these limits:
- 240 to 360 degrees: filter makers print "after the gasket contacts the base, tighten three quarters of a turn by
  hand", and several print "three quarters to one full turn". Nothing in the scene says which, so full credit spans
  both readings, less 30 degrees (a twelfth of a turn) for judging three quarters of a turn by hand against the can's
  quarter-turn ticks.
- 360 to 450 earns 0.5: past the most any maker allows, so the gasket is over-compressed, but within a quarter turn of
  it, so not counted as crushed.
- Over 450 (1.25 turns) is 0 and irreversible: the gasket is squeezed out of shape, and backing the filter off does not
  restore it. The sim does not deform the gasket, so the peak angle is the proxy; a real trial reads the peak from the
  video against the witness marks and checks the gasket for a flattened or rolled lip after the trial.
- 45 degrees not turned by a holding wrist: the filter settling as the hand lets go stays well under it, and a swipe,
  a knock or the other arm turning it adds far more. A real trial checks the video for the filter turning without a
  hand closed on it.
- 180 to 240 earns 0.5 (tightened, but under the makers' figure); 90 to 180 earns 0.25 (partly tightened, a third to
  two thirds of the figure); under 90, a third of the least any maker asks, is not counted as fitted and earns 0.

In a real trial: before the trial, spin the filter on to gasket contact by hand, draw a line across the can and the
head (the witness line), and back the filter off three quarters of a turn to its start. Tape a printed protractor ring
(15-degree marks) around the head's gasket land. Afterwards read the can's line against the ring, counting whole turns
from the video, and photograph it from above. Watch the video for the filter being spun by a swipe rather than turned
in the hand or with the wrench, and for the hands letting go. Then put a click torque wrench set to 18 N·m on the cap
and turn it clockwise: it must click before the can moves, which shows the specified torque was reached; if the can
moves first, the trial scores at most 0.5.

## Runtime and seeds

- Demo: 1.0 in 1,861 steps (93 s; the run ends 1 s after full credit is held).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

True size: the named filter is a small spin-on filter, 6.6 cm across and 7 cm tall here. The head is mounted with the
filter pointing up so a straight-down grip reaches it; on most engines the filter hangs down or sideways, but the
fitting procedure is the same, and some engines and remote mounts carry it upright. The thread is a model: the filter
sits on a slide along the nipple and a hinge, and every control step `twistkit.py` drives the slide to 1.5 mm of travel
per turn (at most 0.5 mm past gasket contact, the gasket compressing) and holds the filter where it was left. The slide's
spring (3,000 N/m) sags about 0.8 mm under the 250 g filter, so in the sim the gasket starts about 0.3 mm above the land,
not the real 1.1 mm; the score reads the thread's angle, not the filter's height.

The gasket's resistance: past contact the filter takes a torque that rises linearly with the angle, to 18 N·m at three
quarters of a turn past contact and on at the same slope (29 N·m at 1.25 turns). It is a stiff hold where the filter
was left that gives way at that torque, either way (stick-slip friction, not a spring). The endpoints come from the
makers (Kawasaki's 18 N·m for this filter; the filter makers' three quarters of a turn past contact); the straight line
between them is our model of the gasket's squeeze, stated as such. Before contact the filter runs on with about
0.3 N·m. A hand turning the can itself slips on it at about 2.5 N·m (measured in this sim in our previous revision:
two pads at the gripper's 50 N on a 3.3 cm radius), which the gasket reaches about 40 degrees past contact. So the rest
needs the wrench, as on a real engine.

The wrench: a 65 mm, 14-flute, 3/8-inch-drive cap filter wrench on a 24-inch, 1/2-inch-drive ratchet through a 1/2 to
3/8-inch adapter, as a second articulation (lift, turn about the filter's axis, two slides; 1.45 kg: GearWrench's
catalog weight for the ratchet, 3.0 lb or 1.36 kg, plus about 0.1 kg for the cap and adapter, estimated; only its
handle collides). The cap is drawn only. It goes down over the can only centered within 3 mm of its axis; once its rim
is below the can's top, the cup's wall keeps it within 2 mm of the axis each way (its play on the flutes) until it is
lifted clear (1.7 cm), so it cannot leave the can sideways and nothing pushes it up. Down on the can, the model holds
it on the flutes until it is lifted 8 mm (about half the cup's 15 mm depth), as the accepted compression-joint task's
model holds its spanner's jaws on the nut. The demo's hands bear down on the handle while they push, 3 N each plus 0.6 N per N·m of the stroke's torque (about
14 N each on the last strokes; the arm presses down about 52 N continuously). While the ratchet drives (the handle turned clockwise), the cap couples to the
filter through a stiff spring (400 N·m/rad: the cap, the drive and the can flexing, 2.6 degrees at 18 N·m), and the
filter turns once that spring's torque passes the gasket's; turned back, the ratchet freewheels with 0.2 N·m of pawl
drag. Lifted 1.7 cm the cap clears the can; lying on the bench the wrench stays where it was set down.

The arms' strength. The YAM's servos are Damiao DM-J4340 on joints 1-3 (9 N·m rated, 27 N·m peak) and DM-J4310 on
joints 4-6 (3 N·m rated, 7 N·m peak); the sim's arm uses the same peak limits (i2rt's yam_v1 config). Worked through the
arms' Jacobians at the stroke's hand positions (right 0.55 m and left 0.22 m along the handle, handle angles
280 to 250 degrees), with the torque that holds each arm up included, the two hands together turn the handle with
10.6 N·m at rated (continuous) torque and 23.6 N·m at peak, at the stroke's weakest point. 18 N·m is
1.7 times the rated figure and 76 % of the peak. It is needed only near the end of the last strokes, a few
seconds each, which is what a servo's peak rating is for; the continuous rating bounds heating over minutes of steady
load. We state this rather than claim the task is within the continuous rating. In the sim the arms are PD-controlled
(80 N·m/rad on joints 1-3, 10 on 4-6): pushing 20-30 N sideways, each hand has to be aimed well past the handle
(about 6 mm per newton, mostly the wrist's give), which the demo plans.

No oil, gasket deformation or leak is simulated; the scorer reads the angle, and a real trial reads the same angle from
the witness marks (the line across the can and head, and the protractor ring), then checks the torque directly with a
click torque wrench.

## Real-world build

- A universal remote oil filter mount with an M20 x 1.5 nipple (aluminum, about 7.5 x 9 cm, two side ports with plastic
  plugs) bolted to a board clamped to the bench, nipple up, so the filter's axis is at x 30, y 30 cm.
- Filter: Kawasaki 49065-0710 (Teryx, Teryx4 and other models), its gasket lightly oiled, a paint-pen index stripe down
  the can's side (the top left plain), three black quarter-turn ticks, and a black witness line on the front of the head,
  set as in the real-trial steps.
- Wrench: a 65 mm, 14-flute cap-type oil filter wrench (Parts Unlimited 3801-0295, 65 mm (2-9/16 in), 14 flutes,
  3/8-inch drive, sold for Honda and Kawasaki filters; 65 mm / 14 flutes is the size listed for the Hiflo HF204 /
  K&N KN-204, which cross to this filter on the Teryx4 750; check its fit on the filter's flutes when the rig is built)
  on a GearWrench 81364XP ratchet (1/2-inch drive, 24.0 inches overall, catalog weight 3.0 lb) through a GearWrench
  81354 adapter (1/2-inch female to 3/8-inch male). It lies
  on the bench cap down at the start, its handle where the setup gives.
- Torque check after each trial: a 3/8-inch-drive click torque wrench whose range covers 18 N·m (13 ft·lb), set to
  18 N·m, in the cap's square drive (no adapter needed).
- Torque spec source: Kawasaki Teryx4 750 owner's manual, torque table (p. 101): "Oil Filter 18 N·m (1.8 kgf·m,
  13 ft·lb)"; the manual says to apply a thin film of oil to the gasket and tighten the filter to the specified torque.

## Licenses

All code here was written for this task; no third-party code is included. No third-party assets are used: every shape
is generated by `yamkit.py` and `twistkit.py`.
