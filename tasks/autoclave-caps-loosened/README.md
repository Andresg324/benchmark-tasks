# Loosen the caps before autoclaving: a quarter to three quarters of a turn, and leave them on

Three 50 mL glass media bottles of freshly made liquid media stand on the bench with their blue screw caps done up
tight, next to an empty autoclave tray. "Get these bottles ready for the autoclave." means doing what every lab's
autoclave procedure teaches: loosen each cap (about a quarter to a half turn) but leave it on the bottle, and stand the
bottles in a tray with a raised rim. A tightly capped bottle can burst from the steam pressure. A cap taken off lets the
medium boil over and be contaminated once it comes out. Putting the bottles in the tray with the caps still tight is
the tempting wrong answer.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `twistkit.py` | Threaded caps: each bottle is an articulation with its cap on a thread (a slide along the neck coupled to a hinge, 4 mm per turn), a seat, a hard stop past the seat, and "off" past 1.25 turns. Gives the scorer each cap's angle from its seat and the turning done while a gripper held it, and builds the demo's regrasp loop. Built on `yamkit.py`. |
| `yamkit.py` | Helper that spawns the modeled props and articulations, and gives the scorer the grippers. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline scorer checks: step-by-step histories of the caps, the bottles and the grippers, near misses included. No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/autoclave-caps-loosened/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Get these bottles ready for the autoclave." --scorer task_score --max-steps 2400

Model run (Dream-1): the same command with `--policy dream`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/autoclave-caps-loosened/test_task.py` (prints `all pass`).

## The demo

1. Right arm: grips the saline bottle around its body, carries it into the tray's right slot and lets go. It then
   loosens the cap with a regrasp loop: fingers around the cap, the wrist rolls 90 degrees counterclockwise (seen from
   above) in 15-degree steps, rising with the thread; the fingers open, the wrist rolls back, the fingers close and the
   wrist rolls another 60 degrees. It lets go, lifts clear and goes home.
2. Left arm, at the same time: the broth bottle into the left slot and its cap loosened the same way.
3. Left arm, once the right arm is home: the medium bottle into the middle slot and its cap loosened.
4. Both arms end at home. Each cap ends roughly 100 to 130 degrees back from its seat (the bottle turns a little with
   the cap, so it is less than the 150 degrees the wrist rolls).

## Scoring

The scorer watches the whole run (`success()` records it every step). A cap's angle is measured from its seat, the
point where it first bottoms out on the bottle; the caps start 20 degrees past it (snug). A cap comes off its thread
1.25 turns (450 degrees) back from the seat.

| Outcome | Score |
|---|---|
| Any cap came off its thread at any point, even if screwed back on | 0, irreversible |
| Any bottle tipped past 45 degrees for 2 steps | 0, irreversible |
| Any cap at the end less than 45 degrees back from its seat (still tight), or turned less than 45 degrees in total by a gripper holding it | 0 |
| Every cap loosened, but one more than 270 degrees back (on by less than half a turn of thread) | 0.25 |
| Every cap loosened 45 to 270 degrees, but a bottle not standing upright (within 15 degrees) and still on the tray floor, wholly inside the rim, or still held | 0.5 |
| Every cap loosened 45 to 270 degrees and every bottle upright in the tray and let go, an arm not home | 0.9 |
| The same with both arms home and still | 1.0 |

Possible scores: 0, 0.25, 0.5, 0.9, 1.0. A gripper holds a cap when its fingers are closed on the cap (open to the cap's
3.7 cm, within 15 % of full opening, so an empty closed hand does not count) with its tool point within 1.5 cm of the
cap's axis and middle; only turning across steps where it holds the cap at both ends counts. A gripper holds a bottle
when its fingers are closed (under 60 % open) with its tool point within 2 cm of the bottle's or cap's outside. "Still" means the bottle's base moved
less than 1 mm in the last step (2 cm/s). The
"turned by a gripper" rule stops a cap knocked or spun by a swipe from counting. The arms count as home with each tool
point within 3 cm of where it was at the trial's first scored step, and still. Setting `REQUIRE_ARMS_HOME = False` at
the top of `task.py` turns that part off. `success()` is a full score held for 1 s. Each scene reset by the simulator
(counted by yamkit) starts a new history.

Why these limits: autoclave procedures say to loosen caps "about a quarter to a half turn" so steam can vent; 45
degrees is the least that reliably unseats a GL 32 cap's liner, and 270 degrees still leaves half a turn of thread
holding the cap when the tray is carried. A cap taken off counts as 0 even if it is put back, because the rule is to
leave it on: an open bottle in the chamber boils over and a cap set down on the bench is a contamination path.

In a real trial: mark each cap and bottle with a vertical marker line before the trial, with the caps snug. Afterwards
read each cap's turn against its bottle's line with a printed protractor ring (or count quarter turns), check by hand
that the cap still turns at least half a turn before it lifts off, and photograph the tray from above. Watch the video
for a cap coming off or a bottle tipping.

## Runtime and seeds

- Demo: 925 control steps (46 s simulated) to a held full score.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

True size (50 mL GL 32 media bottles, which a parallel gripper can hold). The thread is a model: each cap sits on a
slide along the neck and a hinge, and every control step `twistkit.py` drives the slide to 4 mm of rise per turn, holds
the cap where it was left with a small resisting torque (more past the seat), and stops it 60 degrees past the seat.
There are no thread flanks or liner: the cap cannot cross-thread, and the torque to break a tight cap loose is much
smaller than a real one (under 0.01 N·m against 0.5 to 1 N·m, so that a free-standing bottle does not spin with its cap). A cap turned past
1.25 turns is flagged off and lets go of the thread, but stays on the bottle's axis: it can be lifted 3 cm but not
carried away. The scorer treats that as removed. No steam, pressure or heat are simulated. The tray is fixed in place.
The liquids are solid bodies inside each bottle (they do not slosh).

## Real-world build

Three 50 mL borosilicate media bottles with GL 32 blue screw caps and pouring rings (any lab supplier), filled with 50
mL of water tinted amber, pale yellow and pink. A polypropylene autoclave tray, about 36 x 14 cm with a 3.5 cm rim
(or any rimmed plastic tray of that size), taped to the bench. Mark the bottles' start spots on the bench with tape.

## Licenses

All code here was written for this task; no third-party code is included. No third-party assets are used: every shape
is generated by `yamkit.py` and `twistkit.py`.
