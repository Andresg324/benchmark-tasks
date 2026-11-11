# Feed all 12 chutes of a riffle splitter evenly, tipping a full-width pan of sand with its lip held level by both hands

A bench riffle splitter for fine aggregate stands on the table with a catch pan under each row of outlets; beyond it a
steel pour pan as wide as the chute row holds 667 g of dry concrete sand. "Split the sand sample in the pan into two
halves with the riffle splitter." ASTM C702 has the sample poured from a pan as wide as the chutes, spread from edge to
edge so that approximately equal amounts flow through each chute (Iowa DOT IM 336 repeats it). The chutes alternate
between the two catch pans, so a pour that favors one end or a corner feeds a few chutes and gives halves of
different size and gradation. The task is scored on that evenness, chute by chute, and on the 50/50 split. In
practice a loaded pan this wide is held by both ends to keep its lip level, so both hands take its end handles, carry
it over the hopper and tip it together, then set it back down and let go.

## Spec

| Item | What it is |
|---|---|
| Instruction | "Split the sand sample in the pan into two halves with the riffle splitter." |
| Job | Civil engineering technologists and technicians (O*NET 17-3022.00) reducing a field sample of fine aggregate to test size (ASTM C702 Method A) before a sieve analysis (ASTM C136) |
| Rule | Pour along the full width with the lip level over the hopper, so each chute gets about the same amount; spill nothing; set the pan down and let go |
| Why both arms | The loaded pan (967 g) is held by both end handles, 28 cm apart, to keep its lip level; the scorer requires a hand gripping each handle while at least 90 % of the sand pours |
| Physics | `rifflekit.py`: the sand as a scored bulk model (see Fidelity), drawn with Isaac Lab visualization markers |
| Traps | One end lower than the other; the lip off center along the row; pouring from a corner; pouring short of the hopper; still holding the pan at the end |
| Nearest board tasks | #463 (lift a two-handled pot and pour soup into a bowl) and #506 (pour wine with two hands) score whether the pour lands; here the score is how evenly each of the 12 chutes is fed and how close the split comes to 50/50, so an off-level pour that lands entirely in the splitter earns 0.25 or 0.5 |

### Scene (cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0)

| Object | Type and size | Material | Position |
|---|---|---|---|
| Riffle splitter | 12 chutes 1.27 cm (1/2 in) wide between 1.6 mm vanes, a row 17.3 cm along y; open hopper mouth 6 cm across x, top 10 cm up; end plates 8 x 10 cm at y +/-8.8, each topped by a 6 mm flange that tips stray sand into the end chute; chute floors slope to outlets 4.5 cm up, alternating (the first chute, at -y, empties toward the robot); fixed | aluminum | hopper centered at (22, 0) |
| Catch pans (2) | 5 x 19 cm inside, 3.5 cm walls; fixed | steel | centered at (14.8, 0) and (29.2, 0) |
| Pour pan | inside 10 cm (x) by 17.2 cm (y), 3 mm floor, 4.5 cm walls on the far side and ends, 6 mm lip on the side facing the splitter; 300 g, 967 g loaded (the sand's weight is in the physics) | steel | centered at (38, 0), lip's outer face at x 32.9 |
| Pan handles | flat bars 2.5 cm wide, 2 cm tall, 2 to 4 cm up, from the end walls out to y +/-16 (lengthened from a stock pan's short tabs, our invention, so the grippers hold them 28 cm apart at y +/-14) | steel | part of the pan |
| Sand | 667 g of dry concrete sand, 1.45 g/mL loose, 34-degree angle of repose, spread along the width and banked against the far wall (4.5 cm deep there, its face sloping at 34 degrees to the floor 8 mm behind the lip); drawn tan | | in the pour pan |

### Why these numbers

- **Splitter.** C702 asks for an even number of equal chutes, at least 12 for fine aggregate, about 1/2 in wide, and a
  pour pan as wide as the chute row or slightly narrower: here 2.4 mm narrower, so a centered pan has 1.2 mm to spare
  at each end. The hopper is 10 cm tall (a bench splitter's is about 20 cm) so the arms carry the pan over it.
- **Sample.** 667 g splits into two halves of about 333 g; ASTM C136 needs at least 300 g of fine aggregate after
  drying, so each half has to land within 45 to 55 % (300 to 367 g).
- **Even feed.** "Approximately equal amounts through each chute" is scored as every chute within 35 % of the mean
  chute. In the model a centered, level pour gives chutes within 10 % of the mean (the strips and chutes do not line
  up exactly); a lip about 4.5 degrees off level (one end 1.4 cm lower across the 17.2 cm lip) or about 5 mm off center
  along the row reaches 35 %, and 7 degrees off level reaches 60 %, the 0.5 band.
- **Spill.** 13 g (2 %) is a pinch of sand on the table. The hopper's end flanges catch a stream up to 6 mm past
  either end of the chute row and tip it into the end chute, as a real hopper's end plates do, so a pan 3 to 8 mm off
  center spills little (0 g at 3 mm, 5.5 g at 8 mm) and is caught instead by the evenness rule.
- **Tilt.** The demo tips the pan 60 degrees, 26 degrees past the sand's repose angle, which leaves about 10 g (1.5 %)
  in the pan; 97 % through allows that.

## Scoring

| Outcome | Score |
|---|---|
| Pan not back on the table (underside within 8 mm of the table, level within 10 degrees, still) or not let go (a tool point or pad tip within 1 cm of the pan or its handles), including doing nothing with the arms resting on it | 0 |
| Less than 90 % of the grams that went down the chutes ran while each handle bar had its own gripper closed on it (fingers stopped by the 2.5 cm bar, tool point within 1 cm of it): one hand on a handle, a hand pinching the pan's back wall, both hands on one handle, an empty fist or an open hand at a handle | 0 |
| Less than 50 % of the sample went down the chutes (doing nothing, pouring onto the table or straight into a catch pan, tipping too little) | 0 |
| At least 97 % down the chutes, at most 13 g (2 %) spilled or dropped past the hopper into a catch pan, every chute within 35 % of the mean chute, each catch pan 45 to 55 % of the sample, both arms home and still | 1.0 |
| The same with an arm not home (a tool point more than 3 cm from where it started, or moving) | 0.9 |
| At least 90 % down the chutes, at most 5 % lost, every chute within 60 % of the mean, each catch pan 40 to 60 % | 0.5 |
| Anything else with half or more down the chutes (a pour from one end or a corner, a lip far off level or off center, a spill) | 0.25 |

Possible scores: 0, 0.25, 0.5, 0.9, 1.0. Spilled sand stays spilled, and every chute's grams only add up, so a jolt
that throws sand onto the table on the way still counts after a good pour. Each scene reset by the simulator (counted
by yamkit) refills the pan and empties the chutes and catch pans. `REQUIRE_ARMS_HOME = False` at the top of `task.py`
turns the arms-home part off.

## Demo and checked mistakes

Both arms come in beside the handles in a side grip (hand pointing inward along y, fingers closing along x), close,
lift the pan 15 cm, carry it back over the splitter, lower the lip to 2.2 cm over the hopper's middle and tip the pan
to 60 degrees in 12 steps over 6 s by turning both wrists together, keeping the lip on a fixed line. They hold 4 s,
tip back level, carry the pan to its spot (aimed 12 mm long, as the loaded pan trails the hands), set it down, let
go, back out sideways and go home. In the sim, with the pan at its loaded 967 g, it scores 1.0 at step 654: the pan
stayed within 0.4 degrees of level across the row and 2 mm of center through the pour, both hands gripped their
handles for all 655 g that went down the chutes (fingers 0.19 to 0.32 open on the bars), every chute got 54.3 to
55.6 g, nothing spilled and 11.6 g stayed in the pan.

Each mistake below ran in the simulator as a copy of the task with the demo edited. The one-arm run is on the final
task; the others ran with the pan at 300 g and before the two-hands rule, which they do not touch:

| Run | Score |
|---|---|
| Doing nothing | 0 |
| One arm only (the right hand runs the demo, the left stays home): the loaded pan hung from the one handle with its far end dragging on the table, rolled 26 degrees, and its sand ran out onto the table, none into the chutes (at 300 g, before the two-hands rule, one hand did carry it over the hopper and got 70 % down the chutes) | 0 |
| Left hand 2.5 cm higher and right 2.5 cm lower while pouring: the grips gave and the pan rolled only 3.5 degrees, chutes 43 to 66 g (within 35 % of the mean) | 1.0 |
| Left hand 7 cm higher and right 7 cm lower while pouring: the pan rolled 17 degrees, chutes 3.9 to 81 g, catch pans 347 and 305 g | 0.25 |
| Tipped from a corner, like dumping a scoop (6 cm up and 6 cm down, tipped in 1.2 s): the pan rolled 13 degrees, chutes 20 to 77 g | 0.25 |

`test_task.py` checks the same cases offline and more: a fast 1 s tip (1.0), the lip rolled 3, 6 and 10 degrees (1.0,
0.5, 0.25), one end lifted 35 degrees (0.25), the lip 3 cm off center (0.25), poured short of the hopper (0) or
straight into a catch pan (0), tipped only 30 degrees (0.25), turned right over along the lip (1.0), still holding the
pan (0), open hands left on the handles (0), set down tipped 20 degrees (0), a 2-step jolt onto the table before a
good pour (0.25), arms not home (0.9), the lip 3 mm off center (1.0) and 6 mm off (0.5), one-handed pours (one hand
pinching the middle of the back wall, one hand on its handle, both hands on one handle, an empty fist or an open hand
at a handle, the right hand letting go halfway through the tip: all 0), and a new trial after a pour.

## Real-world build

A bench riffle splitter for fine aggregate with 12 chutes of 1/2 in, its two catch pans, and its matching pour pan with steel flat bars bolted to the end walls as handles out to
16 cm from the middle; 667 g of oven-dried concrete sand. The model's per-chute grams are measured directly: fit each
catch pan with five thin sheet-metal dividers so that each chute's sand lands in its own compartment, then weigh each
compartment, the sand left in the pan and the sand swept from the table on a 0.1 g scale.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`) for the `yam_arena` embodiment. |
| `rifflekit.py` | The sand: strips at the angle of repose, a weir law over the lip, sideways drift, and where each stream lands (chute, catch pan or table). |
| `dosekit.py`, `pourkit.py` | The sand's material numbers and the scene hook rifflekit builds on. |
| `guardkit.py` | The let-go and arms-home checks. |
| `yamkit.py` | Spawns the modeled props, gives the scorer the grippers and adds tilted demo waypoints; it patches `yam_arena` in memory at import, and no files change. |
| `test_task.py` | Offline scorer checks: pan poses step by step through the sand model, no simulator. |
| `task.json` | The task card. |

## Run a trial

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/riffle-split-aggregate-sample/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 \
      --instruction "Split the sand sample in the pan into two halves with the riffle splitter." \
      --scorer task_score --max-steps 1500

Model run (Dream-1): the same command with `--policy dream`. Scorer tests: `python
tasks/riffle-split-aggregate-sample/test_task.py` (prints `all pass`).
