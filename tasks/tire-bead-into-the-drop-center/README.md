# Hand-mount the last section of a kids' bike tire: bead into the rim's center channel first, no tire lever

A 12-inch kids' bike wheel lies flat on a hub stand with its tire almost on: one bead (the tire's stiff inner edge) is
fully on, the other is on except the last 60 degrees over the valve, which sits up on the rim's flange (its raised edge).
"Finish putting this tire on the rim." means what a bike mechanic does: push the bead that is already on down into the
rim's center channel (the drop center, a deeper groove round the middle of the rim), on the side away from the valve. That
shortens the bead's path round the rim and frees a little extra length, so the last section rolls over the flange by
hand, while one hand holds the bead beside it down in the channel (it pops back up as the section is lifted). Then set the
valve stem straight toward the hub and let go. The tempting wrong moves are pushing on the last section without making
slack first (it stops on the flange's edge) and reaching for the tire lever on the table, which pinches the tube between
lever and rim.

## Spec

- Job: Bicycle Repairers (O*NET 49-3091.00); Tire Repairers and Changers (49-3093.00).
- Rule and sources: Park Tool, [Tire and Tube Removal and Installation](https://www.parktool.com/en-us/blog/repair-help/tire-and-tube-removal-and-installation):
  install the bead by hand, use levers only as a last resort because they can pinch the tube, and keep the valve
  straight toward the hub. Shop practice for tight tires: squeeze the bead that is on into the rim's center channel
  all round, starting opposite the valve, to gain the slack for the last section, and hold it down beside the last
  section while rolling that over, since it pops back up there.
- Mechanism: one `yamkit` articulation (the wheel). Five bead arcs slide down into the channel: the three away from
  the valve stay where pushed (`mechkit` two-detent `Detents`: on the shoulder or in the channel), the two beside the
  last section are pushed back up unless held (`Creep` in task.py, a 400 N/m force spring, 3.6 N at the channel floor).
  The last section is on a hinge with its own law (`LastSection` in task.py: a hard stop at the flange's edge until
  there is enough slack), and the valve stem turns about the vertical (`Free`). The lever is a loose prop.

| Quantity | Value | Why |
|---|---|---|
| Bead seat | radius 10.15 cm | ISO 203 (bead seat diameter 203 mm), the 12-inch kids' size |
| Flange top | 4.5 mm above the bead seat, z 9.55 | a hooked kids' rim |
| Channel (drop center) | 4 mm below the bead seat, 6 mm wide, in the rim's middle | a single-wall 203 rim |
| Bead arc travel | 9 mm down from its shoulder into the channel | half the rim's 17 mm inside width plus the bead toe |
| Arcs beside the last section | pushed back up onto the shoulder unless held | lifting the last section pulls the bead next to it up out of the channel |
| Slack per arc | 4.2 mm | a 60-degree arc moved 4 mm closer to the hub: 4 mm x 1.047 rad |
| Slack needed | 10 mm | lifting the last 60 degrees 4.5 mm onto the flange top takes 4.7 mm; its climb at each end about 2.5 mm more; rounded up |
| Slack counted | from each end of the last section round the wheel, while each arc is at least 6 mm down | bead still on its shoulder is held there by the tube's pressure, so length freed beyond it can't reach the last section; 6 mm is the channel's click zone (within 3 mm of its floor) |
| So | an arc beside the last section, held, and the next two (12.6 mm); two are not enough (8.4 mm) | the three far arcs alone give nothing until the arc beside is held down; in proportion between 6 and 9 mm down |
| One hand can't do both | as the last section passes the edge or comes into its seat, one gripper's pad on its press spots and the other's on a press spot of an arc beside it, that arc at least 6 mm down and the lever not on that spot; and the arcs beside have press spots only 10 and 20 degrees from their far end, the last section's within 10 degrees of its middle | one hand holds, the other rolls; the spots are 60 degrees apart, 9.7 cm edge to edge, more than the gripper's 9.4 cm full opening |
| A pad pressing a spot | the pad tip within 1.5 cm of the spot's center across, from 1 cm below its top to 6 mm above | the spot is 2 to 2.2 cm across, plus 4 mm; pressed into it, or touching it with the pad's rounded end |
| Last section | starts tilted 20 degrees up about the tread; stops at 6 degrees short of seated without slack | 6 degrees is where its lip, 1.2 cm over the flange top at the start, comes down to the flange's edge |
| Forced | the last section passing 0.5 degrees beyond that stop or into its seat without 10 mm of slack, at any time, or staying past it without slack for 3 steps (0.15 s) | 0.5 degrees is 0.4 mm at its bead, past the sim's contact give at the stop |
| Bead on | the last section within 2.6 degrees of seated | there its lip passes below the flange top; within 2.5 degrees it is pulled in and stays |
| Valve straight | within 10 degrees of the line to the hub | 5.6 mm at the cap of the 3.2 cm stem, a lean you can see; it starts at 20 degrees (1.1 cm) |
| Lever pinch | spoon tip between the rim bed and the flange (radius 9 to 10.6 cm), at or below the bead's height there (the raised lip over the last section) | that is where a lever goes to lift a bead, and where it traps the tube; a lever lying on top of the bead (tip 3 mm higher) is not |

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `yamkit.py` | Spawns the modeled wheel (an articulation) and the lever, and gives the scorer the grippers. It patches `yam_arena` in memory at import; no files change. |
| `mechkit.py` | Joint laws: the far bead arcs' two detents and the valve's free turn. |
| `guardkit.py` | Scoring guards: the pinch and forced latches (with the lever tip's path between steps tested), arms-home check, wheel-frame geometry. |
| `test_task.py` | Offline checks of the slack law and of the scorer (every mistake below, one-arm and forced attempts, near misses, a hand left on the tire, fast lever motion, reset). No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/tire-bead-into-the-drop-center/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Finish putting this tire on the rim." --scorer task_score --max-steps 1200

Model run (Dream-1): the same command with `--policy dream`. Render the videos of a run with
`inspect-robots video <log.json>`. Scorer tests: `python tasks/tire-bead-into-the-drop-center/test_task.py` (prints
`all pass`). One trial takes up to 1,200 steps (60 seconds at 20 Hz). No seeds: the layout is fixed.

## The demo

1. Right arm, closed fingers pointing down: presses the tire's sidewall 1 cm outside the flange at 228, 270 and 312
   degrees, 9 mm down each time, pushing those three bead arcs into the channel, then pushes the arc beside the last
   section down at 170 degrees and holds it (16.8 mm of slack reach the last section).
2. Meanwhile the left arm lowers its closed fingers beside the valve stem, on its leaning side, and pushes it straight
   toward the hub, then backs off.
3. While the right hand holds, the left arm presses the last section's sidewall down along its arc until the bead drops
   inside the flange. Both hands lift off (the held arc springs back up onto its shoulder; the section stays in) and go
   home. Score 1.0 in 553 steps.

## Scoring

| Outcome | Score |
|---|---|
| The lever's spoon tip under the bead at any moment (irreversible) | 0 |
| The last section forced more than 0.5 degrees past the flange's edge, or into its seat, without the slack, at any time (irreversible) | 0 |
| The last section rolled over or seated without one hand pressing it and the other holding an arc beside it down (one hand, an idle hand, or the lever doing the holding; irreversible) | 0 |
| The last section not over the flange at the end (nothing done, or pressed without slack, or with no arc beside it held: it stops at the flange's edge) | 0 |
| A gripper's tool point or pad tip within 1 cm of the tire, rim or valve at the end | 0 |
| Bead on, valve leaning more than 10 degrees | 0.5 |
| Bead on, valve straight, hands off, an arm not home | 0.9 |
| Bead on, valve straight, hands off, both arms home and still | 1.0 |

The lever's tip is checked at every control step and along its path since the last one, so a quick poke under the
bead between two camera frames still counts. There is no debounce: the lever lies on the table 8 cm below and 9 cm
outside the rim, so only a hand can put its tip there.

## Real-world build

- Wheel: any 12-inch (ISO 203) kids' bike wheel with a 12.5 x 2.25 (57-203) gumwall tire and a Schrader tube at about
  5 psi, on a hub stand (a 15 mm post through the hub, bolted to a weighted plate clamped to the table) with the rim
  8.5 cm above the table. Mount the first bead, tuck the tube in, and work the second bead on to 60 degrees each side of
  the valve. Kids' tires on 203 rims are tight: whether a given tire needs exactly three arcs in the channel is
  checked once by hand (count how much of the bead has to be pushed into the channel before the last section rolls
  over at 5 psi, and that the bead beside the last section rises back out of the channel when let go) and the setup's
  tube pressure adjusted until it takes about half the bead.
- Slack in a real trial is the bead's own position: the bead visibly sits down in the channel, the rim's bare shoulder showing
  above it, wherever it has been pushed in; the sim's 4.2 mm per arc stands for that.
- Tire lever: a blue nylon lever (11.5 cm, Park Tool TL-1.2 or similar) on a tape outline on the table.
- Valve: a protractor card (marks at 0 and 10 degrees) taped to the rim bed beside the stem.
- Checking a trial: the bead line inside the flange all round, the valve against the card, the video for any lever
  under the bead, then inflate to 40 psi and dunk the tube to confirm no pinch.
- Safety: the arms press 1 cm into a soft tire on a clamped stand; no sharp edges, nothing under pressure beyond 5 psi
  during the trial.
