# Tip the mower onto its side, air filter side up

A 1:4 model of a 21 inch walk-behind rotary mower stands on the bench, waiting for a blade change. The instruction is
"Tip the mower over so the blade can be changed." To reach the blade the mower has to go over onto its side, and the
engine decides which side: its crankcase breather opens into the air filter's base, so tipped with the filter and
carburetor side down, the crankcase oil runs through the breather into the filter and the intake. On this engine the
breather also points rearward, so rocking the mower back on its handle floods it too (some engines, such as Honda's,
allow tipping back). The filter is then soaked and has to be replaced, and the engine smokes, fouls its plug and is hard to
start; with enough oil in the cylinder it hydro-locks. Tipped the other way, filter side up, nothing reaches it.
Owner's manuals say it in one line (Toro 20995 operator's manual, p. 16, checked on the published manual: "Always tip the mower onto its side with the
air filter up"). Here the filter, carburetor and spark plug are on the robot's right, so the right tip lays the mower
down on its muffler side, to the robot's left.

Run (from the repo root, Isaac Lab-Arena installed):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/mower-tip-filter-side-up/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Tip the mower over so the blade can be changed." --scorer task_score --max-steps 600
    python tasks/mower-tip-filter-side-up/test_task.py      # offline scorer checks, prints "all pass"

For a Dream-1 trial, use `--policy dream --max-steps 1200`. The scene is deterministic (no seeds, object_jitter=0).
The scripted demo is planned at 20.5 s (410 steps; it scores 1.0 and ends at about 400, once success has held 1 s); a full trial is 1,200 steps (1
minute), enough for a careful tip and return with margin. The demo scores 1.0 in 387 steps; Dream-1 scores 0 in 1,200
(it hovered near the mower and never tipped it).

Files: `task.py` (scene, scorer, oil visuals, demo), `test_task.py` (offline checks), and local copies of `yamkit.py`
(modeled props, gripper states) and `tracekit.py` (the run's history: how long the breather port was under the oil).

## Spec

Positions are in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.
The mower stands with its front away from the robot and its handle toward it; its filter side is on the robot's right (-y).

| Part | What it is | Where |
|---|---|---|
| Deck | green painted steel, 14 x 14 x 3 cm (56 cm full size), underside 2 cm up; silver blade and center bolt under it | centered x 26, y 0 |
| Wheels (4) | black rubber, 4 cm across, 1.5 cm wide (8 inch), outboard of the deck's sides | y -7.75 and 7.75, x 21 and 31 |
| Engine shroud | dark gray, 7.5 x 7.5 x 5.5 cm, with a black recoil-starter cover, red fuel cap and yellow oil-fill cap on top | centered x 25.5, y 0, on the deck |
| Air filter | yellow cover 2.2 x 4 x 3 cm, white paper element showing through three slots; steel carburetor under it; black breather tube from the crankcase into its base | right side of the engine (-y) |
| Spark plug | white plug, black boot taped back up (wire off) | right side, toward the front |
| Muffler | black, 1.5 x 3.5 x 2.5 cm, steel heat shield | left side of the engine (+y) |
| Lift lugs (2) | steel, 2.5 x 3 x 2.5 cm, on the deck's top at its side edges | y -5.75 and 5.75, x 31.5 |
| Carry handle | black bar 7 cm wide on two posts at the deck's front edge | x 34.3, 5 to 6 cm up |
| Handle | two dark gray bars from the deck's rear corners, 50 degrees up toward the robot, 16 cm long; black crossbar | bars at y -7.25 and 7.25; crossbar at x 8.7, 17.3 cm up |

The mower is one free rigid body of 300 g (a 19.2 kg mower at 1:4 is 19.2 / 64 = 0.30 kg); its mass is spread over its
colliders, which puts the center of mass about 5 cm up. It balances on its muffler-side wheel edges at 58 degrees, and
past that it settles on its side, resting on those wheels, the lug and the handle bar at about 93 degrees.

The oil (task code, scored with tracekit): the crankcase is taken as half full, so the oil's surface passes through
its center at any tilt. The breather port is 18.2 degrees above the center, toward the filter side and the rear,
which puts it under the oil at 25 degrees with the filter side down or the handle down (18 degrees toward the corner
between them), and never with the filter side up short of 155 degrees. While it is under, oil runs into the filter at
4 mL per s, up to the 7 mL the model holds (a 450 mL crankcase at 1:4), but the score needs no amount: once the
port has been under for 2 control steps in a row, oil is in the breather tube and runs on into the filter, which is
then soaked and latched. The real mock's tube behaves the same way: a short dip puts dyed water in it.
Visualization markers draw a dark stain on the filter's element once oil has reached it, spreading as more runs in,
and a drip at the cover's seam at 2 mL; they never affect the score.

## Scoring

| Term | Definition | Why |
|---|---|---|
| Port under | the breather port below the crankcase's center, 2 control steps in a row | half full, the oil's surface passes through the center; 2 steps so a one-step physics pop doesn't count |
| Soaked filter | the port under for 2 steps in a row (0.1 s), at any time | any wrong dip past 25 degrees puts oil in the breather tube and on into the filter; latched, because an oiled filter stays oiled |
| On its side | filter side raised 70 to 120 degrees | at 70 degrees the blade's bolt faces sideways and a wrench reaches it; past 120 it is going over onto its engine |
| Square | the deck's front-to-back line within 20 degrees of level | on its side, not propped on its handle or nose |
| In place | the deck's center within 20 cm of its start | a clean tip moves the deck's center about 12 cm (it swings about a wheel edge 8.5 cm out); 20 cm is a tip plus 8 cm of dragging or knocking |
| Let go | no gripper with its fingers closed within 2 cm of the mower's outline | a mower held up doesn't count |
| At rest | moving under 1 cm per s | set down, not still falling or rocking |
| Arms home | each tool point within 3 cm of its start, and still | the bench is clear for whoever changes the blade |

| Outcome | Score |
|---|---|
| Filter soaked at any time (wrong side down, back on the handle, rolled over upside down), even if set right after | 0 |
| On its side, filter side up, square, in place, let go, at rest, both arms home | 1.0 |
| The same with an arm not home | 0.9 |
| Anything else: untouched, partly tipped, propped, held, moving, dragged | 0 |

Possible scores: 0, 0.9 and 1.0. Only the end state counts for the tip (SCORE_OVER = "end", held 1 s); the oil is
counted over the whole run. Checks a reviewer can repeat: `test_task.py` covers each row, and the variants below.

| Variant (scripted, in the simulator) | Score |
|---|---|
| Demo: left arm lifts the filter-side lug on an arc about the muffler-side wheels, past balance, lets go; it rests at 92.7 degrees | 1.0 |
| Do nothing | 0 |
| Filter side down: the right arm lifts the muffler-side lug the same way, and the mower rests on its filter side at 92.7 degrees | 0 |
| Filter side down to 40 degrees (right arm), set back on its wheels, then tipped the right way as in the demo (rests at 92.7 degrees) | 0 |
| Back on the handle: the left arm lifts the front carry handle until the mower rests on its rear wheels and handle at 54 degrees | 0 |

## Real-world build

- Mower mock: 3D-printed or plywood deck 14 x 14 x 3 cm on four 40 mm rubber wheels (hobby wheels on M3 axles) with
  a printed engine block, filter box, muffler and lugs, painted to match; handle from 6 mm aluminum rod. Weigh it to
  300 g with steel washers glued low in the engine block.
- Oil path: a clear sealed box (about 5 x 5 x 3 cm) inside the engine block, half full of water dyed dark with food
  coloring, with a port drilled at the filter-side rear corner, 18 degrees above the box's center, and a 4 mm
  silicone tube from the port to a white felt pad in the filter box. A tip the wrong way stains the pad, and the stain
  is plain in a photo; replace the pad between trials.
- Grip points: the two lugs are steel or printed blocks screwed to the deck's top edges (the YAM grippers need
  something to close on; a person lifts the deck's rim).
- Measuring: a phone inclinometer app on the deck gives the final angle (70 degrees or more, filter side up); the
  felt pad gives the oil check; a tape on the table marks the start position.
- Safety: no fuel, no blade edge (the blade is a dull printed or aluminum blank), 300 g at most 16 cm off the table.
