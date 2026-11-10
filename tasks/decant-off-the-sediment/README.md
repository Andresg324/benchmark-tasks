# Decant an aged wine off its sediment

A clear glass bottle of old red wine stands beside an empty decanter, its dark sediment settled on the bottom under the
clear wine. "Decant the bottle." means what every sommelier and wine waiter is trained to do: tip the bottle slowly
over the decanter with something light behind the neck, watch the sediment creep along the bottle toward the neck
once most of the wine has gone, and stop as it reaches the shoulder, so the last few centimeters of wine stay behind with the sediment, then stand the bottle back up. The tempting wrong
answer is to pour it like any other bottle, to the last drop, which puts the sediment in the decanter.

## Spec

| Item | What it is |
|---|---|
| Instruction | "Decant the bottle." (the stopping rule is never stated) |
| Job | Sommeliers, waiters and bartenders decanting aged red wine (O*NET 35-3031.00, 35-3011.00) |
| Rule | Pour slowly and stop when the sediment reaches the bottle's shoulder; set the bottle down upright |
| Cue | The near-black sediment inside the clear bottle, which slides toward the shoulder once less than 50 mL of wine is left and darkens the shoulder as it arrives, seen against a white card behind the neck |
| Trap | Pouring on as with any bottle, or holding a fixed tilt for a fixed time, which lets the sediment reach the shoulder and pour (0) |
| Scene | A free bottle (108 mL of wine over 12 mL of sediment), a free, empty decanter and a white card standing behind it; see the table below |
| Liquids | `pourkit.py` flow model with a settled liquid (local changes, see Files and Fidelity) |

### How it differs from neighboring tasks

- Our `kettle-fill-between-lines`, #371 (fill the day tank to the red mark) and #465 (fill the sample bottle between
  two lines) stop a pour at a printed level mark on the receiving container. Here nothing is marked: the stop cue is
  inside the source bottle, a contents cue (the sediment moving toward the neck), and the receiving decanter has no
  lines at all. Over-pouring is not overflow but contamination with the last part of the same bottle.
- #506 (pour wine with both hands), #103 (open a bottle and pour) and #463 (pour a pot) reward pouring everything
  across without spilling; here pouring everything is the failure.

### Scene (cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0)

| Object | Type and size | Material | Position |
|---|---|---|---|
| Bottle | wine bottle: 4.8 cm across, 13.1 cm tall (8.9 cm body, 1.2 cm shoulder, 3 cm neck 2.1 cm across, mouth 1.7 cm inside), cut dark red foil below the mouth; 220 g | clear (flint) glass | center (23, -12), upright |
| Sediment | 12 mL, 0.8 cm deep on the bottle's bottom, near black | tannin and pigment crust with trapped wine | in the bottle |
| Wine | 108 mL over the sediment, ruby | aged red wine | in the bottle |
| Decanter | wide base 11.6 cm across and 4.5 cm tall, neck 6.6 cm across (6 cm inside), 8.5 cm tall; 600 g; empty | clear glass | center (33, -1) |
| Light card | 16 cm wide, 16 cm tall, 3 mm thick, on a 16 by 5 cm foot; 600 g; faces the right arm, so a bottle tipped over the decanter is seen against it from the right wrist camera | white card | center (29, 6.8) |

## Scoring

The scorer watches the whole run (`success()` records it every step, after `pourkit` has moved the liquid).

| Outcome | Score |
|---|---|
| The bottle or the decanter knocked over: leaning more than 45 degrees for 2 steps in a row with no gripper holding it | 0, irreversible |
| Less than 10.8 mL (10 %) of the wine in the decanter (includes doing nothing, hovering, pouring beside it) | 0 |
| More than 1 mL of sediment in the decanter (poured on after the sediment reached the shoulder, or to the last drop) | 0 |
| Sediment kept out, but more than 3 mL of liquid spilled or splashed in all | 0.25 |
| 10.8 to 64.8 mL of wine in the decanter (stopped far too early) | 0.5 |
| At least 64.8 mL (60 %) in, but the bottle not standing upright (within 10 degrees), still, on the table and let go | 0.75 |
| The same with the bottle set down and both arms home and still | 1.0 |
| The same, with an arm not home | 0.9 |

Why these numbers: the sediment (`pourkit.py`, `Vessel.slide_*`) stays on the bottle's floor until the bottle
leans past 75 degrees, where the crust lies on the lower wall instead of the floor, with less than 50 mL of wine left
over it (under half the wine), when the wine no longer covers the crust along that wall. From then on it slides toward
the shoulder in 4 s with no stream, and in as little as 0.8 s under a full stream, which drags it along, and tipped
back under 75 degrees it stops where it is. Once at the shoulder it pours mixed with the wine, and past 85 degrees it
runs out of the neck on its own at 2 mL/s. Tipped past 93 degrees, with the neck below horizontal, the pour
glugs: air gulps back up the neck and churns the sediment toward the shoulder, faster the steeper it is (floor to
shoulder in 0.5 s at 103 degrees), so a steep flip held for a counted time fails however it is timed; a decanter keeps
the neck above horizontal for this reason. 1 mL of sediment in the decanter is a twelfth of it, enough to cloud the
wine. The 60 % bar (64.8 mL) sits 6.8 mL past the point where the sediment starts to move, so a pour that stops before
the sediment has visibly moved falls short, while a slow pour (0.1 to 0.2 degrees per step) that starts tipping back
anywhere from 0.4 to 0.9 of the way to the shoulder puts about 65 to 77 mL in and leaves about a third of the wine with
the sediment, as decanting an old bottle does. 3 mL of spill is a stain on the cloth, more than drips. A stream falling
more than 10 cm from the lip to the decanter's rim splashes a quarter of itself out. A gripper holds a container when
its fingers are under 60 % open with its tool point within 7 cm of the container's origin. Arms are home with each tool
point within 3 cm of its first-step position, and still (`REQUIRE_ARMS_HOME = False` turns that off). Success is a full
score held for 1 s. Each scene reset (counted by yamkit) starts a new history and refills the bottle. Possible scores:
0, 0.25, 0.5, 0.75, 0.9, 1.0.

The stop has to come from watching the sediment, because no fixed tilt and hold time is safe at every tipping speed:
over peaks of 86 to 92 degrees, holds of 0 to 5 s and three tipping speeds, a blind tilt scores 1.0 in 17 % of tries
(`test_task.py`), and in the sim the scripted demo, which is such a fixed plan tuned to this one bottle, tips back with
the sediment most of the way to the shoulder. In a real trial, weigh the decanter, let it stand 5 minutes and draw its
bottom off into a graduated syringe to measure the settled grounds, and weigh a white cloth laid under both vessels for
spills.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`, built by `pour_plan()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). |
| `pourkit.py` | The liquids (copy of `~/yam-tools/pourkit.py` with a local change): adds `settled` liquids, `Vessel.seal` and the sediment's slide (`Vessel.slide_*`, `slurry_*`, `front()`), so a sediment creeps toward the shoulder once the bottle is tipped past 75 degrees with under 50 mL of wine left and pours once it gets there, draws it as a dark layer on the floor upright, as a band at its front when tipped past 70 degrees and, for the last quarter of its slide, as a dark edge growing into the shoulder, and draws liquids opaque. |
| `yamkit.py` | Spawns the modeled props, gives the scorer the grippers and adds tilted demo waypoints; patches `yam_arena` in memory at import. |
| `test_task.py` | Offline scorer checks (prints `all pass`). No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/decant-off-the-sediment/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Decant the bottle." --scorer task_score --max-steps 700

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`. Videos: `inspect-robots video <log.json>`.
Scorer tests: `python tasks/decant-off-the-sediment/test_task.py`. `POURKIT_DEBUG=1` prints tilts and volumes each step.

The wrong-answer check is `pour_plan(peak=1.75)` (tipped to about 100 degrees, as for an ordinary pour): it empties
the bottle into the decanter, sediment included, and scores 0.

## The demo

1. The right arm comes in from the right side with its fingers closing front to back and grips the bottle 5.5 cm up.
2. It lifts it and brings it over the decanter already tipped about 50 degrees, with the lip 5 cm above the rim.
3. It tips the bottle on to about 87 degrees over 8 s, turning about the lip, holds it 0.5 s while the last of the
   stream runs and tips it back over 1.2 s. In the sim 72.4 mL of wine goes in (7.6 mL over the bar) and the sediment
   band stops 0.93 of the way to the shoulder, with 35.6 mL of wine left with it and none in the decanter.
4. It sets the bottle back on its spot, lets go and returns home; the left arm stays home.

## Runtime and seeds

- Demo: 554 control steps (27.7 s simulated), ending in success.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`). The liquid model is deterministic.

## Fidelity

About 0.45 scale of a 750 mL bottle (13.1 cm tall instead of 30 cm), holding 120 mL. There is no fluid simulation.
The bottle holds whatever fits below its lip at its current tilt and pours the rest at up to 10 mL/s per cm² of
mouth straight down, into the opening below or onto the table as spill. The sediment is modeled as a second liquid
with a front that slides from the floor to the shoulder (see Scoring for the rule), then pours well mixed with the
wine; real sediment creeps along the lower wall and arrives as a dark thread in the neck. The pour and the slide run
about 6 times faster than a real decant, which takes about a minute. The drawn liquids fill the bottle's sections along
its axis instead of lying level; tipped past 70 degrees the wine is drawn running to the neck and the sediment as a
band across the bottle at its front, and from 0.75 of the way on its leading edge is also drawn growing into the
shoulder, the part of the bottle the right wrist camera sees best (the band itself is often behind the fingers). The white card stands in for the candle or light a sommelier holds behind the
neck. The renderer hides the liquid behind any see-through mesh, so the bottle's clear glass is
drawn as thin pale green rings (base, heel, shoulder, neck, lip) and the liquid column shows its shape. Vessels keep their full weight and collide as solid shapes on hidden square feet.

## Real-world build

A clear glass bottle about 5 cm across and 13 cm tall with a narrow neck (a 125 to 200 mL flint-glass sauce or liquor
bottle), a small clear glass carafe about 11 cm across, and a white cotton cloth under both. Wine stand-in: water dyed
red; sediment stand-in: 12 mL of fine used coffee grounds with their water, added first and left to settle for 10
minutes with the bottle upright. A white card about 16 cm square on a folded foot stands behind the
carafe, as a sommelier holds the neck over a candle to see the sediment through the glass.

## Licenses

All code here was written for this task, and no third-party code is included. No third-party assets are used: every
shape is generated by `yamkit.py`, and the liquids are drawn by `pourkit.py`.
