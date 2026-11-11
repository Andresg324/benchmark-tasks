# Fill a springform pan only after seating its base and snapping its clasp shut

A 10 cm (4 inch) mini springform pan stands on the counter with its side clasp swung open and its loose base resting
on the top lip of the ring's groove instead of in it. A narrow glass jug of cake batter stands beside it. "Pour the
cake batter into the springform pan." means doing what every baker does first: press the base down into the groove,
push the clasp in until it snaps over center (drawing the ring tight around the base), and only then pour the batter
in and set the jug down. Pouring straight away is the tempting wrong answer: the batter runs out through the open
seam, or under the ring if the base is not seated.

## How it differs from nearby tasks

- #42 dual-latch-lid and #692 M.2 drive latch: release or turn a latch, and that is the whole task. Here closing the
  clasp is the precondition for a pour, and the clasp alone is not enough: the base must be seated first, since a
  locked ring clamps an unseated base where it is.
- #1406 kettle (shut the lid before switching on) and #1369 bonding lead before dispensing: both have "do X before
  a flow or a switch". Here the container itself is not yet a container: it holds liquid only once assembled (base
  into the groove, then an over-center clasp), and the failure is the pan leaking, scored from the batter's own
  path, not a separate flag. The two steps have their own order too (base, then clasp).
- No board task has a springform, a cake pan or a pan that leaks (pr_corpus search for springform, cake pan, clasp,
  batter, leak).

## Spec

| Item | What it is |
|---|---|
| Instruction | "Pour the cake batter into the springform pan." (the rule is never stated) |
| Job | Bakers, pastry cooks and restaurant cooks (O*NET 51-3011.00, 35-2014.00, 35-2021.00) |
| Rule | Base pressed into its groove, then the clasp snapped over center, before any batter goes in; jug set down after |
| Why | With the clasp open the seam stands open and the batter runs out; with the base above the groove it seeps out under the ring onto the counter, and in the oven onto the oven floor, where it burns |
| Traps | Pouring straight in (clasp open, base unseated); snapping the clasp first, which clamps the base on the lip |
| Scene | One fixed pan mechanism (ring half with the clasp's pin, the other ring half that follows the clasp, the clasp lever, the base) and a free glass jug |
| Batter | `pourkit.py` flow model, volumes in mL, with a local `leak` addition |

### Scene (cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0)

Everything is real size.

| Object | Type and size | Material | Position |
|---|---|---|---|
| Springform ring | 10.0 cm inside diameter, 4.5 cm tall, 1.5 mm wall, rolled top and bottom beads; groove seat 4 mm above the table; split at a seam on its left (+y) side; fixed (a real trial tapes it down) | tinplated steel | center (33, 0) |
| Clasp | lever 4.0 x 0.3 x 1.6 cm on a vertical pin at (34.2, 5.6), z 2.4 cm, with a 1 x 0.8 x 2 cm thumb tab at its free end; lies flat along the ring pointing to -x when closed; starts swung out 54 degrees (tab near (32.5, 8.7)), seam 5 mm open | steel | ring's left side |
| Base | disk 9.9 cm across, 3 mm thick, inside the ring, resting on the groove's top lip 6 mm above its seat (top at z 1.3 cm; seated, 0.7 cm) | dark nonstick steel | in the ring |
| Jug | cylinder 4.9 cm across, 12 cm tall, 4.4 cm inside, 4 mm base, 200 g, steel rim band | clear glass | (23, -13), on the table |
| Batter | 150 mL in the jug (about 10 cm deep), pale yellow | cake batter | in the jug |

### Mechanisms

- Clasp (mechkit `OverCenter`): 0 is flat, -54 degrees fully open. Past 17 degrees from flat (the over-center
  point) the ring's spring snaps it shut with 0.12 N*m (about 3 N at the tab); short of it, it springs back fully
  open. Locked: within 3.4 degrees of flat (2.4 mm at the lever's end).
- Ring: its left (-x) half turns on a pin at the ring's -y side, following the clasp (mechkit `Follow`), so the seam
  at +y stands 5 mm open with the clasp fully open and closes as the clasp does. A real ring springs open 4 to 6 mm.
- Base (`SeatBase` law in task.py): resting on the lip it is held up there by 300 N/m (0.9 N at the halfway point);
  pressed more than halfway (3 mm) its edge slips past the lip and it is pulled onto its seat. With the clasp locked
  the ring clamps it where it is (3,000 N/m), so it can't be seated after the clasp is shut.

### Why these numbers

- Leaks: slot flow, Q = w h^3 rho g H / (12 mu L), for cake batter (viscosity about 2 Pa*s, 1,000 kg/m^3) 1.5 cm
  deep (average head 7.5 mm). The open seam is a slot 5 mm wide, 1.5 cm tall and 2 mm deep (where the ends
  overlap): about 3 mL/s. A base on the lip leaves a 1 mm slot all round under the ring (31 cm long, 6 mm across the
  lip): about 0.16 mL/s by the formula; the task uses 0.4 mL/s, a 1.4 mm gap, since a base resting on the lip sits
  slightly crooked. The batter that leaks is drawn running down from the seam into a puddle on the table.
- Sealed: clasp within 3.4 degrees of flat is inside its lock (the over-center point is at 17 degrees); base within
  1 mm of its seat keeps its edge inside the 3 mm groove.
- 0.5 mL in the pan while unsealed (a few drops): any real pour puts more in within one step (0.05 s), so this
  catches pouring before sealing; 2 steps in a row rule out a one-step physics glitch.
- 10 mL before anything counts: a 1.3 mm skim over the base.
- 110 mL for full credit: 73 % of the jug, 1.4 cm deep. A real jug of batter keeps 15 to 25 mL clinging to its walls
  unless scraped, so a careful pour that tips the jug well over leaves at least this much in the pan.
- 3 mL spilled: a coin-sized drip on the counter; more is a mess to wipe up from under the pan.
- A stream falling more than 10 cm from the lip to the rim splashes a quarter of itself out (pourkit's default).
- Jug knocked over: leaning over 45 degrees for 2 steps with no gripper holding it. Set down: upright within 10
  degrees, still, bottom within 1 cm of the table, let go.
- Held: a tool point within 7 cm of the jug with the fingers under 60 % open. On the clasp: a tool point within 4 cm
  of its tab (a hand still holding it shut). Nothing earns credit while either is true at the end.
- Arms home: each tool point within 3 cm of where it was at the first scored step, and still (2 mm per step).

## Scoring

The scorer watches the whole run (`success()` records it every step, after `pourkit` has moved the batter and yamkit
has read the pan's joints).

| Outcome | Score |
|---|---|
| More than 0.5 mL of batter in the pan for 2 steps in a row while it is not sealed, or more than 0.5 mL leaked out in all (a slow trickle that runs straight out) | 0, irreversible |
| The jug knocked over | 0, irreversible |
| Under 10 mL in the pan at the end, or the pan not sealed at the end (includes doing nothing, and closing the pan without pouring). Sealed also needs the base seated when the clasp locked: a clasp locked on a raised base must be opened again before the base can be seated | 0 |
| The jug still held at the end, or a tool point within 4 cm of the clasp's tab | 0 |
| More than 3 mL spilled on the counter | 0.25 |
| 10 to 110 mL in the sealed pan | 0.5 |
| At least 110 mL in, but the jug not set down (upright, still, on the table, let go) | 0.75 |
| At least 110 mL in, the jug set down, both arms home and still | 1.0 |
| The same, with an arm not home | 0.9 |

Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1.0.

In a real trial: film the clasp and base before the pour (the base pressed down, the clasp flat), stand the pan on a
paper towel to show any leak, weigh the pan before and after (batter is about 1 g/mL), and check at the end that the
clasp lies flat against the ring and the jug stands upright with no hand on it.

## Real-world build

- Pan: any 10 cm (4 inch) mini springform pan (tinplated or nonstick steel, about 4.5 cm tall). Tape the ring's
  outside to the table with two strips of gaffer tape away from the clasp; the base stays loose.
- Start state: base laid in the ring with one edge on the groove's top lip (it sits about 6 mm high and rocks), clasp
  swung fully open.
- Jug: a narrow glass jug or a 250 mL glass graduated cylinder about 5 cm across and 12 cm tall, with 150 mL of
  batter. A stand-in batter of similar thickness: 100 g flour, 1 egg and 90 mL milk whisked smooth.
- Paper towel under the pan; a scale for the weigh-in.
- Safety: nothing hot or sharp; the clasp's tab needs about 3 N, well within the arms' force.

## Run

    ~/yam-tools/run_task.sh tasks/springform-clasp-before-batter demo 900    # scripted demo (746 steps)
    ~/yam-tools/run_task.sh tasks/springform-clasp-before-batter dream       # Dream-1, 2,400 steps
    python test_task.py                                                       # offline scorer tests ("all pass")

Runtime per trial: about 2 minutes of robot time (2,400 steps at 20 Hz). The scene has no randomness (object_jitter
0); `SPRINGFORM_DEBUG=1` prints the clasp, base, batter volumes and score every step.

## Fidelity

No fluid simulation: the batter is pourkit's deterministic flow model. The pan's leak is a local addition to
pourkit (`Vessel.leak`, a rate in mL/s from the pan's joints each step, drawn as a run and a puddle). The ring is two
rigid halves instead of a flexing band, its wall collider is a row of 4 mm posts, the base is a slide with a snap,
and the pan is fixed to the table. Batter does not cling to the jug.
