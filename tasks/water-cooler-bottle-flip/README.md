# Put a full bottle on a top-load water cooler

A top-load water cooler stands near the robot with its crown open and its reservoir nearly empty; behind it a full
3-gallon cooler bottle stands on the table, its cap already torn off, with a clip-on carry handle on each
side. "Put the full bottle on the water cooler." means what a water delivery driver or an office facilities
worker does many times a day: take the bottle with both hands, lift it over the crown and turn it neck-down in one
quick motion right above the receptacle, so the open neck drops straight into it and the bottle sits on its shoulder.
Turned over slowly, paused on its side, or turned anywhere but right over the crown, the open neck glugs water onto
the floor. The bottle is wider than one gripper opens and turns on its handles, so one arm cannot do it.

## Spec

| Item | What it is |
|---|---|
| Instruction | "Put the full bottle on the water cooler." (the quick flip over the crown is never stated) |
| Job | Water delivery drivers and office facilities workers (O*NET 53-3031.00 Driver/Sales Workers; 37-2011.00 Janitors and Cleaners, Except Maids and Housekeeping Cleaners) |
| Rule | Both hands; turn the open bottle neck-down in one quick motion right over the crown; seat the neck straight in the receptacle; let go |
| Physical core | The open neck lets water out at 73 mL/s whenever the bottle holds more than fits below its neck at its tilt (from about 60 degrees on) and the neck is not seated; a seated neck feeds the reservoir only until the water covers it (the cooler's air lock) |
| Two arms | The bottle is 10.6 cm across (the gripper opens 9.4 cm), so it can only be held by its two side handles, and a half turn is a 180-degree roll of both wrists about the handles' axis |
| Trap | Turning it slowly or pausing on its side (spill, 0.25 in the sim), turning it over the table and carrying it neck-down (spill), seating it off center (it props on the crown, leaks and topples: 0), one arm (0) |
| Scene | A fixed, scaled water cooler and a free, full bottle with two handles; see the table below |
| Liquids | `pourkit.py` flow model with a local change: a seated neck feeds its receiver only up to an air-lock level |

### How it differs from neighboring tasks

- Pouring tasks (our `acid-into-water` and `decant-off-the-sediment`, and board tasks that pour between containers)
  tip a container part-way about its lip into an open vessel and are scored on what goes where. Here nothing is
  poured: the whole bottle is turned upside down and docked, and the water counted is what escapes during the turn.
- Two-arm carrying tasks move a box or tray level. Here the two hands have to roll the load together through a half
  turn, and the timing of that turn over a small target is the skill.

### Scene (cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0)

| Object | Type and size | Material | Position |
|---|---|---|---|
| Water cooler | 12 by 12 cm cabinet, 9 cm tall (top-load cooler, shortened), white with red and blue tap marks on its front face; a gray crown plate on top with a 3.3 cm square receptacle opening, 3.5 cm deep; reservoir holding 195 mL up to a seated neck's tip (its air-lock level; 3 L real), with 60 mL in it | painted steel cabinet, plastic crown | fixed, center (20.5, 0) |
| Bottle | 3-gallon cooler bottle at 1:2.5 scale: 10.6 cm across, 14.3 cm tall (10.5 cm body, 1.8 cm shoulder in two steps, 2 cm neck 2.2 cm across), open neck (tear-off cap removed), 727 mL of water, 0.80 kg in all | clear blue PET, molded ribs | standing upright, center (33.5, 0) |
| Two clip-on handles | spring-clip carry handles of the kind sold for 3- and 5-gallon bottles, clipped to the bottle's sides 6.5 cm above its base (their clips drawn as one 3.2 cm band); each grip 4 cm long (x), 4.5 cm out (y), 2.5 cm thick | dark gray plastic | on the bottle, handles toward the arms |

## Scoring

The scorer watches the whole run (`success()` records it every step, after `pourkit` has moved the water).

| Outcome | Score |
|---|---|
| The bottle fell: tipped past 45 degrees for 3 steps with no gripper on a handle and its neck not in the receptacle | 0, irreversible |
| Turned past 90 degrees (neck below level) with only one gripper on a handle, outside the receptacle, for 3 steps in all | 0, irreversible |
| Not seated at the end (includes doing nothing, a bottle propped on the crown, leaning, or off the cooler) | 0 |
| Seated, but more than 73 mL spilled in all (water a stream puts in the reservoir past 195 mL counts as spill) | 0.25, irreversible spill |
| Seated, spill within 73 mL, but a gripper still holds a handle or a tool point is within 1 cm of the bottle | 0.5 |
| Seated, let go, both arms home and still | 1.0 |
| The same, with an arm not home | 0.9 |

Seated means: neck-down within 5 degrees of straight down, the neck's tip inside the receptacle opening (within 7.5
mm of its center line each way) and at least 1.5 cm below the crown, and the bottle still.

Why these numbers:

- Flow, 73 mL/s: a real 3-gallon bottle turned neck-down with its cap off glugs empty in about 10 s (about 1.1 L/s
  through a 5.5 cm neck); the scaled bottle keeps the same 10 s, so trial time is real time. It pours whenever it holds
  more than fits below its neck at its tilt, which for this full bottle starts at about 60 degrees.
- Spill limit, 73 mL: 10 % of the bottle (1.1 L real), a puddle that needs a mop and a wet-floor sign. A half turn in
  0.8 s right over the crown loses about half of that (38 mL in the sim) while the neck swings past the crown's edge,
  and once the neck is over the opening the stream falls into the reservoir. Offline, turns of up to about 2.4 s still
  pass; a 3 s turn, a 1 s pause on its side, or a turn over the table followed by a neck-down carry loses more than
  the limit. Water that falls into the receptacle is not spill, so a slower turn about the neck's tip held right over
  the opening (3 s: 51 mL out) also passes; past 4 s, or held on its side, the reservoir rises past its 195 mL
  air-lock level, and water above that counts as spill, since the seated neck and shoulder push it over the crown.
- Tilt band, 5 degrees: seated on its shoulder ring the bottle stands within a degree or two of straight down; at 5
  degrees one side of the 5 cm shoulder ring is 4.4 mm off the crown, so it is propped on its neck or the crown's edge
  and rocks, which is how a badly seated bottle leaks around the crown and falls off.
- Depth, 1.5 cm: the neck is 2 cm long; with its tip less than 1.5 cm in, the shoulder is not down on the crown.
- Center, 7.5 mm: the 2.2 cm neck has 5.5 mm of room each way in the 3.3 cm opening, plus 2 mm for the reported pose.
  A neck brought down 1.5 cm off center lands on the crown's edge instead and the bottle props, leans or topples.
- 45 degrees, 3 steps (0.15 s): a full bottle leaning that far with no hand on it is falling or has fallen; three
  steps in a row so a one-step physics pop does not count.
- One hand: a 12.5 kg bottle turned on one handle twists out of the hand or swings its open neck sideways, and
  bottled-water suppliers tell customers to lift and turn it with both hands. In the sim one arm can lift the
  0.8 kg bottle by one handle nearly level, but turning it alone it sags to about 170 degrees with its neck 3 cm off
  the opening, glugs about 190 mL and falls off the crown when let go.
- A gripper holds a handle when its fingers are stopped by it, 10 to 60 % open (the 2.5 cm handle stops the 9.4 cm
  opening at 18 to 34 % in the sim; an empty closed fist reads 0), with its tool point inside the handle's box grown by 1 cm.
  The one-hand rule counts 3 steps in all, so swapping hands every 2 steps does not dodge it. For 0.9 and 1.0 both
  tool points must be at least 1 cm clear of the bottle's outline (body, neck and handles), so a gripper still
  closed on the bottle anywhere scores 0.5. Arms are home with each tool point within 3 cm of its first-step position, and still
  (`REQUIRE_ARMS_HOME = False` turns that off). Success is a full score held for 1 s. Each scene reset (counted by
  yamkit) starts a new history and refills the bottle.

Possible scores: 0, 0.25, 0.5, 0.9, 1.0.

In a real trial, weigh the bottle before and after and weigh the reservoir's water drawn off through the taps (what
the bottle lost minus what reached the reservoir is the spill), or lay an absorbent mat around the cooler and weigh it;
check the seat by eye (the bottle stands straight and its shoulder sits on the crown) and watch the video for the
number of hands on the bottle during the turn.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`, built by `arm_plan()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). |
| `pourkit.py` | The water (copy of `~/yam-tools/pourkit.py` as changed for `decant-off-the-sediment`, with one more local change: `setup(docks=...)`, a vessel whose mouth is seated in a receiver pours only into it and only up to a seal level). |
| `tracekit.py` | Per-trial memory for the irreversible rules (spill, fell, one hand). |
| `yamkit.py` | Spawns the modeled props, gives the scorer the grippers and adds tilted demo waypoints; patches `yam_arena` in memory at import. |
| `test_task.py` | Offline scorer checks (prints `all pass`). No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/water-cooler-bottle-flip/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Put the full bottle on the water cooler." --scorer task_score --max-steps 520

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`. Videos: `inspect-robots video <log.json>`.
Scorer tests: `python tasks/water-cooler-bottle-flip/test_task.py`.

Wrong-answer checks: `arm_plan(side, flip_s=3.0)` (slow turn), `arm_plan(side, pause=(90, 1.0))` (pause on its
side), `arm_plan(side, aim=(0.015, 0))` (seated 1.5 cm off center) and a demo with only the right arm's plan; see
STATUS.md for their sim scores.

## The demo

1. Both arms come in from the sides at the handles' height, fingers open and closing vertically, and close on the
   handles.
2. They lift the bottle 11.6 cm and carry it 13 cm toward the robot until it hangs over the crown (its base clears
   the cooler while it turns).
3. Both wrists roll together from -90 to +90 degrees (joint6) in 0.8 s, turning the bottle neck-down about the
   handles' axis; then the arms lower its neck straight into the receptacle in 0.8 s. In the sim 38.2 mL spills
   (about half the limit), all of it while the neck swings past the crown's edge.
4. The bottle feeds the reservoir until the water covers its neck (60 to 195 mL); the arms let go, back out to the
   sides and return home.

## Runtime and seeds

- Demo: 391 control steps (19.6 s simulated), ending in success (full score from step 372, held 1 s).
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`). The water model is deterministic.

## Fidelity

1:2.5 scale of a 3-gallon bottle (0.80 kg instead of about 12.5 kg). The cooler's body is cut to 9 cm, since a real
crown is about 95 cm up and the arms lift only about 22 cm; the crown and receptacle are to scale. There is no fluid
simulation: the bottle holds what fits below its neck at its tilt and lets the rest out at 73 mL/s, straight down into
the opening below or onto the floor as spill; the water's weight does not slosh. The drawn water fills the bottle along
its axis instead of lying level. The open-reservoir type of cooler is modeled (no probe piercing a cap).

## Real-world build

- Cooler: a 12 cm cube of plywood or a small storage box, painted white, with a 3 mm plastic or aluminum top plate
  that has a 3.3 cm square hole over a 3.5 cm deep pocket and a shallow tray (about 9 cm across, 3 cm deep) below it
  as the reservoir; screw it to the table at (20.5, 0) and put a red and a blue dot on its front face.
- Bottle: a clear round PET or HDPE bottle about 10.5 cm across and 14 cm tall with a 2.2 cm neck (a 1 L round
  cooler-bottle replica or a wide 1 L juice jug cut to height), filled with 727 mL of water, open.
- Handles: two clip-on bottle handles (spring-clip carry handles sold for 3- and 5-gallon bottles, singly or as a
  2-pack), clipped to opposite sides of the bottle 6.5 cm above its base with their grips toward the arms; grips
  about 4 by 4.5 by 2.5 cm. A trial on a full-size 3-gallon bottle with the same two handles is the real job but
  weighs about 12.5 kg, beyond a YAM arm's payload, so the robot trial uses the scaled bottle (about 0.80 kg in all).
- An absorbent mat (or paper towel of known dry weight) around the cooler for the spill check; a kitchen scale.

## Licenses

All code here was written for this task, and no third-party code is included. No third-party assets are used: every
shape is generated by `yamkit.py`, and the water is drawn by `pourkit.py`.
