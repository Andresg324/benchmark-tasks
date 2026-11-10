# Hand truck: plate under, tilt back, roll, stand it up, slide out

A boxed mini fridge stands on the floor, too wide for the gripper and too heavy to carry, and a two-wheel hand truck
stands upright behind it with its nose plate facing it. "Move the boxed fridge onto the taped loading spot." means
doing what movers and delivery drivers are trained to do: slide the nose plate fully under the carton until it rests
against the frame, tilt the truck back so the weight rides on the axle, roll it to the spot, stand it up and slide the
plate out. Shoving the carton across the floor with the plate or the gripper, dragging it on an untilted truck,
lifting it, tipping it over or letting it slide off the plate all score 0.

## Spec

| | |
|---|---|
| Scale | 1:5. The carton stands for a 50 x 50 x 90 cm, 30 kg mini-fridge carton; the truck for a 100 cm, 15 kg light-duty hand truck with 10-inch wheels |
| Instruction | "Move the boxed fridge onto the taped loading spot." |
| Unstated rule | Load the truck properly (plate fully under, carton against the frame), tilt it back, roll it, stand it up and slide the plate out; never push, drag, lift or drop the carton |
| Tempting wrong answers | Push the truck so the plate shoves the carton along the floor; drag the carton on the plate without tilting; leave the truck tilted or the plate under the carton |
| Core mechanism | A free-rolling two-wheel articulation used as a lever about its axle, loaded through hard contact (plate under the carton, carton against the frame) |

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `yamkit.py` | Helper that spawns the modeled props and the hand truck as a free articulation (frame plus two wheels on revolute joints), and gives the scorer the grippers. It patches `yam_arena` in memory at import; no files change. Local additions: per-collider physics materials (a slick nose plate), sphere and convex-hull colliders, and unlimited joints for the wheels. |
| `test_task.py` | Offline scorer checks: step-by-step histories of the truck, the carton and the grippers, with near misses and cheap tricks. No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/hand-truck-tilt-and-roll/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Move the boxed fridge onto the taped loading spot." --scorer task_score --max-steps 1500

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/hand-truck-tilt-and-roll/test_task.py` (prints `all pass`).

## Scene

Positions in cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top (the floor)
z 0.

| Object | Type and material | Size | Where |
|---|---|---|---|
| Carton | Free, corrugated kraft cardboard, 240 g. Packing tape over the top seam; a white product panel with a blue band and two black up-arrows on the face toward the robot | 10 x 10 x 18 cm (the gripper opens 9.4 cm) | Standing, center (30, 5) |
| Hand truck frame | Free articulation base, 120 g. Red square steel tube: two rails 0.7 x 0.8 cm with centers 7.6 cm apart, three crossbars, a loop handle with a black rubber grip 2 x 2 x 6.8 cm (its square section turned 25 degrees) 19 cm above the floor and 2.8 cm behind the frame | 19.4 cm tall | Axle center (30, 20.3), 2.5 cm up; the frame's face 1.8 cm ahead of the axle, toward the carton (-y) |
| Nose plate | Part of the frame. Steel, 3 mm, low friction (0.15); its front 3 cm beveled to a 0.4 mm edge | 7 cm ahead of the frame x 7 cm wide | On the floor, its tip 1.5 cm from the carton (y 11.5) |
| Wheels | Two, on revolute joints, 12 g each. Black rubber with a gray hub and four lug marks | 5 cm across, 1.6 cm wide, centers 10.4 cm apart | On the axle |
| Loading spot | Fixed, yellow floor tape 1 cm wide, flat | 16 x 16 cm inside | Center (30, -13), 18 cm from the carton's start |

## The demo

1. Left arm: grips the handle's rubber grip from above and behind and pushes the truck 9.1 cm forward, so the beveled
   plate slides under the carton until the carton rests against the frame.
2. Tilts the truck back about its axle to 45 degrees (the carton rides on the plate, leaning on the frame), rolls it
   17.4 cm to the spot and tilts it back up until the carton stands on the floor.
3. Pulls the truck straight back 9.5 cm so the plate slides out, lets go and goes home. The right arm stays home.

## Scoring

The scorer watches the whole run (`success()` records it every step) and scores the end.

| Outcome | Score |
|---|---|
| The carton's footprint (the center of its bottom face) traveled 3 cm in all while not carried on the truck, summed over every stretch where it was not carried: pushed, dragged or shoved by the plate | 0, irreversible |
| The carton's lowest corner 1.5 cm off the floor while it is not on the truck (its center more than 2.5 cm from its seat): lifted, or slid off the plate | 0, irreversible |
| The carton tipped past 75 degrees | 0, irreversible |
| Both wheels 8 mm above their rolling height with the carton on the truck: carried, not rolled | 0, irreversible |
| The carton falling faster than 0.5 m/s (2.5 cm in one step): dropped | 0, irreversible |
| Nothing done, or the carton never carried | 0 |
| Carried, but its center not on the spot at the end | 0.25 |
| Its center inside the tape, not finished | 0.5 |
| Finished, both arms home and still | 1.0 |
| Finished, an arm not home | 0.9 |

Each violation must last 2 control steps in a row (0.1 s), so a one-step physics glitch does not count.

- **Seated:** the carton's near face within 1.5 cm of the frame's face (the 7 cm plate then still holds 6.5 cm of the
  10 cm carton; 8 mm the other way only absorbs contact penetration), its center within 3 cm of the truck's center
  line and within 1.2 cm of its height resting on the 3 mm plate (contact jitter; more means it rode up the frame or
  dropped off the plate's edge), and square to the frame within 12 degrees (which opens about a 1.9 cm gap at its far
  end, the same as the 1.5 cm limit plus slack).
- **Carried:** seated, the truck tilted back 10 degrees or more, and its wheels on the floor (one wheel center within 3
  mm of its rolling height and the other within 5 mm, so the truck may rock onto one wheel; in the demo the wheels stay
  within 1 mm while rolling and one rises 2.9 mm while the truck is tilted). At 10 degrees the plate's tip is 1.6 cm up and its back
  edge clears the floor, so only the wheels touch the floor. A truck held above the floor is not carried, and its
  carton's travel counts as a shove.
- **Shove:** each stretch not carried counts its net travel, so contact jitter does not add up, and carrying never
  resets the total, so short tilts between pushes cannot launder one. 3 cm (15 cm at full size) leaves room for the
  plate nudging the carton as it slides under (about 1 cm in the demo) and the small shifts as the truck is tilted up
  and set down.
- **On the truck** (for the lift, drop and carried-truck rules): the carton's center within 2.5 cm of its seat, so a
  bounce or a rock off the frame while tilted is not a lift. A carton sliding toward the plate's tip leaves that zone
  long before it falls off (its center is 5 cm from the frame, the plate 7 cm long).
- **Lift:** 1.5 cm (7.5 cm at full size) is five times the plate's thickness, so a carton resting on the plate's edge
  is not lifted. **Drop:** a free fall reaches 0.5 m/s after 1.3 cm; tilting or standing the truck lowers the carton
  at under 0.2 m/s in the demo. **Tip:** 75 degrees is beyond any carrying angle (the demo carries at 45).
- **Finished:** all four bottom corners of the carton inside the tape's inner edge (a carton turned on the spot has to
  fit too), flat (under 5 degrees, lowest corner within 4 mm of the floor), the plate out from under it (its near face
  5 mm past the plate's tip, or clear to the side), the truck upright within 10 degrees, no gripper closed within 3 cm
  of the truck or the carton, and both still (under 1 mm per step).
- **Arms home:** each tool point within 3 cm of where it was at the first scored step, and still, so the aisle is
  clear; `REQUIRE_ARMS_HOME = False` at the top of `task.py` turns that part off.

`success()` is a full score held for 1 s. Each scene reset (counted by yamkit) starts a new history.

Possible scores: 0, 0.25, 0.5, 0.9, 1.0.

In a real trial: watch the video for the carton sliding along the floor (in all, before and between tilts), lifting,
tipping or dropping, and for the wheels leaving the floor or the truck being rolled tilted less than 10 degrees (a
protractor on a still frame). Then check that all four corners of the carton's base are inside the tape's inner edge,
that it stands flat with the plate out from under it and the truck upright, and the arms.

## Runtime and seeds

- Demo: 564 control steps (28 s simulated), ending on success.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

1:5 scale. The carton is a solid box of uniform density (a real fridge carton is top-heavy). The wheels collide as
spheres of the tire's radius. The nose plate is low-friction steel, and the table top stands in for the floor. The
truck is light at this scale, so the arm's grip, not the load, holds it at its tilt.

## Real-world build

At 1:5: a 10 x 10 x 18 cm corrugated carton weighted to 240 g with a bag of sand taped to its floor, and a model hand
truck about 20 cm tall with a thin steel or aluminum nose plate at least 7 x 7 cm (sold as dollhouse or 1:6 action
figure accessories, or bent from 3 mm aluminum sheet on two 5 cm caster wheels). Mark the 16 x 16 cm spot with 1 cm
yellow floor tape, centered 18 cm from the carton's start.

## Licenses

All code here was written for this task; no third-party code is included. No third-party assets are used: every shape
is generated by `yamkit.py`.
