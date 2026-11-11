# Prime a pitcher pump: priming water onto the dry leather first, then stroke until water flows

A red cast-iron pitcher pump stands on a wooden well platform with its spout over a steel pail, and a white enamel
cup of water stands on the table. The pump has stood unused, so its leather plunger cup is dry and does not seal.
"Get the pump going and pump water into the pail." means doing what anyone who keeps such a pump knows: pour the cup
of priming water into the open top of the pump head, onto the leather, then stroke the handle steadily until water
runs from the spout, set the cup back down and leave the handle at rest. Working the handle straight away (it is
right there and the instruction says to pump), pouring too little, and stopping after a few strokes are the wrong
answers.

## How it differs from nearby tasks

- #256, hand pump to a dial pressure: there the instruction spells out every step and the skill is holding a
  free pump and watching a gauge. Here the instruction says only to get the pump going; the robot must know that a
  dry pitcher pump needs priming before the first stroke, and pour it into a fixed pump's head from a free cup.
- #372, open the suction valve before starting a pump, and #1243 / #830 (grease pail lever, floor jack): an order
  rule or a lever worked in strokes, but no liquid handled and no priming. Here the core is a pour that changes how a
  mechanism behaves (the leather seals only under water) followed by enough full strokes of a hinged handle.

## Spec

| Item | What it is |
|---|---|
| Instruction | "Get the pump going and pump water into the pail." (the priming rule is never stated) |
| Job | Farmworkers, groundskeepers and camp or park caretakers who start pitcher pumps on wells and cisterns (O*NET 45-2093.00, 37-3011.00) |
| Rule | Prime first: pour the cup of water into the pump head, onto the leather; then stroke the handle steadily until water flows; set the cup down, let the handle rest |
| Why | A dry leather cup has shrunk and does not seal, so strokes move only air, and dry strokes scuff the leather against the cast-iron bore. Too little water leaves the leather's lip dry and air leaks past it. The first strokes after priming only draw the air out of the drop pipe, so stopping early gets no water. |
| Traps | Pumping before priming; pouring part of the cup; stopping before flow; pouring the priming water into the pail |
| Scene | One fixed pump mechanism (platform, head with open top, spout, hinged handle), a fixed pail and a free cup; see the table below |
| Water | `pourkit.py` flow model for the priming water and the pail; the pump is `mechkit.HandPump` on the handle |

### Scale and why these numbers

The scene is 0.55 of a real No. 2 pitcher pump (3 in. bore), so lengths are 0.55x and volumes 1:6.

- Priming water: the cup holds 40 mL, a cup (240 mL) real, the usual amount poured into a dry pitcher pump.
- Seal: at least 20 mL on the leather. The bore is 5.6 cm across (24.6 cm²), so 20 mL stands 8 mm deep, enough to
  cover the leather cup's 6 mm flared lip so all of it soaks and swells against the bore (120 mL real, half the cup).
- Lift: the first 3 sealed strokes draw the air out of the drop pipe and bring no water (a short pipe; real shallow
  wells take 3 to 10 strokes). Each stroke after that delivers 75 mL, a real stroke's 450 mL (3 in. bore, 4 in.
  plunger stroke) at 1:6.
- A stroke: the handle from at least 12 degrees up to at least 15 degrees down, 60 % of its 45-degree swing, so a
  short jab at the handle's middle moves no water. Strokes are counted by the handle's own joint angle.
- Flowing: 150 mL in the pail, two delivering strokes (0.9 L real), so the pump has held its prime from one stroke
  to the next rather than giving a single gush.

### Scene (cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0)

| Object | Type and size | Material | Position |
|---|---|---|---|
| Well platform | box 10 x 10 x 2.5 cm; fixed | wood | center (31.5, 0) |
| Pump head | base flange 8.4 cm across, 0.8 cm tall; body 6.4 cm across up to its rim at z 9 cm, slightly flared at the top; fixed | red-painted cast iron | axis at (32, 0) |
| Bore (open top) | 5.6 cm across; the leather plunger cup and its nut 4 cm below the rim (z 5 cm); holds 98 mL above the leather; starts dry | brown leather, iron | in the head |
| Spout | iron pipe 1.6 cm across, out of the head's front (+x) at z 7.2 cm, turned down 9 cm from the axis | cast iron | over the pail |
| Handle | iron bar 1.2 x 1 cm with a wooden grip 3.6 x 2.2 x 2 cm (its middle 12 cm from the pivot), on a fulcrum bracket on the head's left side; pivot at (32, 3.6, 11.5), reaching toward +y; swings about x between 20 degrees up (rest) and 25 degrees down: the grip's top travels from z 16.6 to 7.3 cm | cast iron, wood | left of the head |
| Pail | 9 cm across, 5.2 cm tall, 8.4 cm inside, holds 271 mL; fixed | steel | center (41.7, 0) |
| Cup | 4 cm across, 7.5 cm tall, 3.5 cm inside, 40 g, blue rim; free | white enamel | (25, -8) |
| Water in the cup | 40 mL (240 mL real), shown blue | | in the cup |

## Scoring

The scorer reads the handle's stroke counts (kept by its `HandPump` law from the joint angle every step), the water
in the head and in the pail (by liquid: the pump's "well" water is kept apart from the priming water) and the
grippers.

| Outcome | Score |
|---|---|
| No pumped water in the pail: doing nothing, pumping unprimed or with under 20 mL in the head, stopping within the first 3 sealed strokes, or pouring the priming water into the pail | 0 |
| At the end a gripper still holds the cup, a tool point is within 4 cm of the handle's grip (open or closed), or the handle is not back up past 12 degrees | 0 |
| Pumped water in the pail, but the handle made more than two full strokes before the head held 20 mL (pumped dry; irreversible; one or two while the priming water goes in are common practice) | 0.25 |
| Pumped water in the pail, under 150 mL | 0.5 |
| 150 mL or more, but the cup not standing upright (within 10 degrees) and still on the table | 0.75 |
| All of that with both arms home and still | 1.0 |
| The same, with an arm not home | 0.9 |

Other thresholds: a gripper holds the cup when its fingers are under 60 % open with its tool point within 4.5 cm of the
cup's center; the cup is on the table when its bottom is within 1 cm of the table top; arms home is each tool point
within 3 cm of where it was at the trial's first scored step, and still (`REQUIRE_ARMS_HOME = False` at the top of
`task.py` turns that part off). `success()` is a full score held for 1 s. Each scene reset by the simulator (counted
by yamkit) refills the cup, empties the head and pail and clears the stroke counts. Possible scores: 0, 0.25, 0.5,
0.75, 0.9, 1.0.

In a real trial, note from the video whether the handle was worked before the cup was poured, read the priming
water left in the cup against marks inside it, measure the pail's water with a measuring jug and check the cup and
the handle at the end.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`, built by `pour_plan()` and `pump_plan()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). |
| `mechkit.py` | Joint laws. The local copy adds `HandPump`: the `Pump` law's stroke counting on the handle's own joint, with a seal test and a delivery callback, and a return pull to the handle's rest. |
| `pourkit.py` | The water: a scored flow model of pouring between containers. The local copy adds `add()`, which puts the spout's water in the pail and draws its stream. |
| `yamkit.py` | Spawns the modeled props and the pump mechanism, gives the scorer the grippers and joints, and adds tilted demo waypoints. It patches `yam_arena` in memory at import, and no files change. |
| `test_task.py` | Offline scorer checks (no simulator): the right order, each trap, near misses, held-at-the-end cases and a reset between trials. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/prime-a-pitcher-pump/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Get the pump going and pump water into the pail." --scorer task_score --max-steps 1500

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/prime-a-pitcher-pump/test_task.py` (prints `all pass`).

## The demo

1. The right arm comes in from the right with its fingers closing front to back and grips the cup 4.5 cm up.
2. It lifts it and brings it over the pump head already tipped 49 degrees, with the lip 5 cm above the rim.
3. It tips the cup on to 89 degrees about its lip, holds and tips it back: nearly all 40 mL land on the leather.
4. It sets the cup back on its spot, lets go and returns home.
5. The left arm puts its closed fingers on top of the handle's grip and pushes the handle down to its bottom stop,
   then lifts off so the handle swings back up; six strokes: three bring the water up, three deliver 225 mL.
6. The left arm returns home.

## Runtime and seeds

- Demo: about 905 control steps (45 s at 20 Hz), ending in success.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`). The water and pump models are deterministic.

## Fidelity

0.55 scale, so volumes are 1:6. There is no fluid or air simulation. The priming water is the flow model in
`pourkit.py`: the cup holds what fits below the lowest point of its rim at its tilt and the rest pours over that
point straight down into the opening below, or onto the table as spill. The pump is a stroke-counting model: the
leather seals while 20 mL or more stands on it, the first 3 sealed strokes lift the water up the drop pipe and each
later stroke adds 75 mL to the pail. Leather wear is modeled only as the dry-stroke rule, priming water does not
leak away during a trial, and the handle's return to rest (in a real pump, the plunger rod's weight) is a spring.
The pail is fixed in place.

## Real-world build

The sim scene is 0.55 scale to fit the simulated arms' reach. A real build uses an off-the-shelf cast-iron pitcher
pump (No. 1 or No. 2, sold for wells and garden cisterns) bolted to a plywood platform over a lidded tank or
5-gallon bucket of water with a 1.25 in. suction pipe, a steel pail under the spout (wedged in a wooden ring so it
cannot be knocked) and a 250 mL enamel cup filled to a mark. A full-size pump is 1.8 times the sim's, so its
handle's grip travels from about 30 to 13 cm above the platform; sink the platform into a cut-out in the table, over
the tank, so the grip's top stays within the arms' reach (about 22 cm above the table). Before each trial open the pump's drain (or lift the
plunger and sponge the bore) so the head is empty and the pump has lost its prime, and swap in a dry leather cup from a
set kept on hand, since a damp one would still seal.

## Licenses

All code here was written for this task, and no third-party code is included. No third-party assets are used: every
shape is generated by `yamkit.py`, and the water is drawn by `pourkit.py`.
