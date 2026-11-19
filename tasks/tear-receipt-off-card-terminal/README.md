# Tear the customer's receipt off a card terminal along its tear bar and lay it on the tip tray

`task.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), `station` scene. A handheld card terminal
lies screen up on the counter with the customer's receipt just printed: 10 cm of 58 mm thermal paper standing up out
of its printer at the printer end, past the serrated tear bar. The robot gives the customer the receipt: one hand
steadies the terminal, the other tears the paper off along the tear bar and lays it in the tip tray beside the
terminal, and both hands come away.

## Run a trial

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/tear-receipt-off-card-terminal/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Give the customer their receipt." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/receipt-demo
```

Model run (Dream-1): the same command with `--policy dream --max-steps 1200`.
Video: `inspect-robots video logs/receipt-demo/*.json`. The scorer (`task_score`) reports the score each step; the
trial's score is the one at its end. `RECEIPT_VERBOSE=1` prints the paper's loads at the tear line.

## Why it matters

Taking a card payment and handing over its receipt is a core task (rated "Core" by O*NET) of three large US
occupations (employment from O*NET, 2025):

- cashiers, 3,106,300 workers: "Issue receipts, refunds, credits, or change due to customers." (O*NET 41-2011.00);
- waiters and waitresses, 2,284,400: "Collect payments from customers." (O*NET 35-3031.00);
- bartenders, 762,600: "Collect money for drinks served." (O*NET 35-3011.00).

It comes with every card sale. The Move/5000 will "confirm the sale is approved and print a receipt" (Chase, Move/5000
help), and there were 86.1 billion in-person card payments in the US in 2018 (Federal Reserve Payments Study, 2019).
Handheld terminals take the payment where the customer is: Global Payments' iPay@Table app on the Move/5000 enables
"credit and debit payment acceptance at the table, bar or on an outdoor patio". The receipt is the customer's record
of the payment: for a debit payment at a point-of-sale terminal, US Regulation E requires the financial institution to
"make a receipt available to a consumer at the time the consumer initiates an electronic fund transfer at an
electronic terminal" (12 CFR 1005.9(a); small-value transfers are exempt).

The person tears the receipt off by hand against the printer's tear bar. Manuals for printers with a tear bar give the
technique: "Pull one edge of the paper against the tear bar at an angle as shown below, and then tear down and across
against the tear bar to tear the paper." (Printek MT2) and "Pulling up and pulling sideways without using the tear bar
can cause a paper jam." (Intermec PB3). On the Move/5000 the paper is unrolled back over the printer end, over the
tear edge: it goes "back over the top of the printer, not ... over the screen" (Chase, Move/5000 help), and is
unrolled "3 cm über die Abrisskante" (3 cm over the tear edge; Payone Move/5000 quick start guide). So the tear edge
is on the printer-end side, as on printers where the paper door's edge is the tear bar (Printek MT2), and a real
receipt leans back over the printer end; it is torn back over the bar, away from the screen.

The robotic difficulty is the direction of a small pull on a floppy strip, not force:

- thermal paper parts easily only where the bar's teeth start the tear. Its tear resistance is at least 220 mN, its
  tensile strength at least 2.1 kN/m (Jujo KSND-48 datasheet): pulled straight up out of the printer, ripped
  sideways, pushed or pressed against the bar, or pulled over the plain lip on the screen side, it does not tear
  along the bar;
- the paper must be pinched without crumpling it and drawn back over the teeth tight, edge first, while the other
  hand steadies the terminal;
- a pull that stops part-way leaves a ragged half-tear hanging from the printer;
- the torn receipt is light and floppy and must end up lying inside a 4-1/2 x 6-1/2 in tray.

## Scoring

**Torn off along the bar**: the receipt is fully separated at the tear bar, along its whole width, with no part-torn
flap left hanging from the printer and no ragged rip. **Lying in the tray**: the tray right side up and resting flat,
every corner of the receipt inside the rim's inner outline, at most 15 mm over the tray floor and no more than 1 mm
below the floor's top face, measured in the tray's own frame, so the check is the same wherever the tray sits or turns
(`test_tray_placement_robust`: 300 placements); a receipt standing up, hung over the rim, crumpled high or under the
tray does not pass. **Hands clear** (a release check): no part of either arm, any link from the shoulder to the
fingertips, in contact with the receipt or the tray (within 1 mm). In the simulation every arm link is checked by its
collision hulls from the robot model, posed each control step. The simulator simplifies each collision hull to at most
64 corners, and the simplified collider can stand proud of the model's hull (most on the wrist link's thin plate), so a
light touch of the simulated collider can pass as clear. **On their marks**: the terminal resting on the counter (flat,
within 5 degrees), its center within 2 cm of where it started and turned at most 10 degrees; the tray's center within 2
cm of its outline's.

| Score | Condition |
|---|---|
| 1.0 | torn off along the bar and lying in the tray, both arms clear, the receipt still (under 1 cm/s) for 1 s, the terminal and the tray on their marks |
| 0.75 | torn off along the bar and lying in the tray otherwise (still held, a hand or arm on the tray, or the terminal or tray off its mark, lifted or propped) |
| 0.5 | torn off along the bar, not lying in the tray |
| 0.25 | a tear started and stalled part-way (a ragged half-tear left in the printer), or a ragged rip |
| 0 | nothing torn, or nothing done |

The whole trial scores 0 if the receipt falls off the counter or the terminal is knocked over (screen more than
30 degrees from level at any time).

**The tear** (simulation). The paper bends with its own stiffness everywhere. Along the line where it leaves the
printer, just below the bar's teeth, it parts at its datasheet tear resistance (220 mN, ISO 1974, the tear running
across the paper) where the bar's teeth notch it: where the paper at the slot touches the toothed edge and the paper
above bends back over the bar. A tear already running continues at that resistance while the paper is still drawn
over the teeth. Anywhere else along that line the paper holds its full tensile strength (2.1 kN/m, ISO 1924-3). Only
tension in the paper counts: the paper drawn out of the slot, or sheared along its width; a push into the slot or a
press against the bar or the lip loads nothing. Paper that gives at its full strength instead makes a ragged rip,
which scores like a stalled tear.

**Cheats tested** (`test_task.py`): doing nothing; the start state (nothing torn, the paper off the teeth); a straight
pull out of the slot; a sideways rip without the bar; pressing the receipt onto the bar; pushing it down into the
slot or pressing it flat over the bar; a pull over the plain lip toward the screen; a tear nicked on the teeth and
then pulled straight up or sideways; pulling the roll out of the printer; pushing the receipt over onto the counter
untorn, or ripping it onto the tray without the bar; the tray flipped over the receipt, tilted, or carried off its
outline; the receipt under the tray (the tray set down on it); the terminal moved off its mark, turned, lifted,
propped or tipped over; the receipt dropped off the counter; a hand left on the receipt or the tray; a wrist or
forearm left resting on the tray's rim.

## Real-world build (real objects, full size)

A low-cost setup. Common tools are assumed (steel rule, masking tape, a phone camera on a stand).

- **Terminal:** Ingenico Move/5000 (169 x 78 x 57 mm, 310 g without its paper roll and battery: Ingenico user guide).
  A refurbished unit will do, e.g. Discount Credit Card Supply's "Ingenico Move 5000, 4G, Bluetooth, Wi-Fi Terminal -
  Refurbished". No payment account is needed to make a receipt: open the paper door, pull the paper out past the tear
  bar and close the door, as for loading a roll (Ingenico: "Pull the paper up to the top of the terminal"); a powered
  unit can also feed it ("The feed key will eject the thermal paper a few centimeters over the tear line if pressed
  for more than two seconds", Fiserv Move/5000 quick guide).
- **Receipt roll:** 2-1/4 in (57 mm) thermal rolls for the Move/5000 (Ingenico: "Width: 58 mm", "Diameter: 40 mm",
  "Paper thickness: 48µm"; POS Supply's Move/5000 supplies page: the printer "will hold a roll with an outside roll
  diameter of 1.57 inches (40mm)"), e.g. POS Supply's "2 1/4" x 50' Thermal Receipt Paper Rolls (50 Rolls)", listed
  as "48 gram basis weight / 2.1 mil caliper". The simulation's paper is a 48 g/m^2 thermal grade from the Jujo
  KSND-48 datasheet (48 g/m^2, 55 um; tensile at least 2.1 kN/m MD, tear at least 220 mN CD).
- **Tip tray:** Winco TT-46, black plastic, sold as "4-1/2 x 6-1/2" (Winco's own listing gives the width as 4.38 in),
  or any 4-1/2 x 6-1/2 in tip tray.
- **Counter:** the station table top.

## Real-world evaluation (reproducible protocol)

Outcome only; no force gauge. The real score is set by the torn receipt, the tray, the hands and the terminal, never
by how the robot moved. Cameras: the top camera plus a phone on a stand at counter height, level with the tray.

**Setup marks and tolerances** (coordinates in cm from the midpoint between the arm bases, x forward, y left).

- Terminal: a tape outline, screen up, long side along y, printer end toward the robot's right: center at
  (24.0, 3.0), printer end at y -5.45. Placed within 5 mm and 3 degrees of the outline.
- Receipt: pull (or feed) the paper out until 10 cm stands past the tear bar (rule, 2 mm). Do not straighten it.
- Tip tray: a tape outline, long side along y, center at (20.0, -20.5), within 5 mm and 5 degrees.

**Measurement (after the run ends, before anything is moved).**

1. Torn off along the bar: photograph the printer slot from the side: nothing above the bar but the serrated edge of
   the paper left in the printer (no flap or tongue of paper standing above the bar). Photograph the receipt's torn
   edge on the rule: it stays within 2 mm of a straight line across its whole 58 mm. A flap left in the printer, a
   tear that runs off the line by more than 2 mm, or a receipt still attached with a tear started at the bar: 0.25.
2. Lying in the tray: top photo, the tray right side up and within 2 cm of its outline, the receipt on the tray floor
   (not under the tray) and no part of it outside the rim's inner edge; side photo with a rule standing in the tray, no
   part of the receipt more than 15 mm above the floor.
3. Hands clear and still: from the video, no part of either arm touching the receipt or the tray, and the receipt
   not moving, for the last 1 s.
4. Terminal: from the top camera, lying flat on the counter, its center within 2 cm of its outline's center and
   turned at most 10 degrees; never tipped over or off the counter during the run.

**Real-trial score.** The same table as the simulation's.

**Reset** (about 1 minute). Take the receipt out of the tray, put the terminal back on its outline, pull or feed a new
10 cm receipt, put the tray back on its outline. About 10 cm of paper per trial; the roll is the only consumable.

**Tolerances.**

| Quantity | How it is read | Resolution | Threshold |
|---|---|---|---|
| Torn off along the bar | photo of the slot and of the torn edge on a rule | 1 mm | whole width, within 2 mm of straight, no flap above the bar |
| Lying in the tray | top and side photos, rule in the tray | 1 mm | tray right side up, inside the rim, at most 15 mm over the floor |
| Hands clear, still | video | one frame | no part of either arm on the receipt or tray, no motion for 1 s |
| Terminal and tray on their marks | top camera, tape outlines | 2 mm, 1 degree | terminal flat, within 2 cm, at most 10 degrees; tray within 2 cm |
| Paper past the bar | rule | 2 mm | 10 cm |

**Safety.** Battery power only, no mains in reach of the arm; no exposed blade (the tear bar's teeth are small);
nothing hot or heavy. Nothing wet, greasy or dusty touches the gripper pads: they handle only dry paper and the
plastic tray.

## How the simulation models it

- **Receipt** (`sheetkit.py`, `yamkit.py`): the paper's bending stiffness, D = E t^3 / 12(1 - nu^2) from its 55 um
  thickness and an office-paper elastic modulus in the machine direction, and its real mass (48 g/m^2),
  on a row of thin strips across the paper's width (58 mm wide, 6.9 mm long, the first a short strip at the slot)
  hinged to bend along the receipt's length. For contact each strip is 1 mm thick. 10 cm of this paper stands
  straight: it is shorter than its self-weight buckling length (`test_upright_under_self_buckling`). The paper's
  friction is the paper-on-paper range's middle (TAPPI); the fingertips get rubber pads.
- **Tear line:** the paper's strength across the line where it leaves the printer, as above, checked at every control
  step from the paper's stretch there (PhysX's own joint break force is not used); the bending across that line has
  the paper's stiffness until the receipt is off. The paper in the printer is held fixed there.
- **Terminal and tray:** free rigid bodies at their real sizes. The terminal is a 169 x 78 x 57 mm box; in the model
  the paper leaves the top face 16 mm from the printer end between the serrated tear bar on the printer-end side and
  a plain lip on the screen side, each 6 mm proud. The bar's teeth are drawn on it; its contact edge is straight, and
  the tear starts where the paper at the slot touches that edge.
- **Limits:** the paper bends one way only (along the receipt's length) and is rigid across its width; the tear runs
  only along the line at the bar (a real crooked pull can tear off the line; the real trial checks the edge); the
  receipt starts straight up, where a real one leans back over the printer end with the curl the roll leaves in it.

## Runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (1 minute).
- Scripted demo: about 21 s (416 control steps); it scores 1.0 in the record run (Dream-1: 0 in 1,200 steps).
  The left hand, fingers closed into one blunt tip, presses lightly on the terminal's keypad end until the receipt is off, as a thumb steadies it. The
  right hand comes over the standing receipt above its top edge and straight down over it, fingers fully open either
  side of it, pinches it near its top edge, toward its near edge, then swings the pinch back over the teeth toward the
  printer end and down and across, drawing the paper tight over the bar, so the tear starts at the near edge and runs
  across. It carries the receipt to the tip tray, touches its hanging end down at one end and lays it along the tray
  while lowering, lets go and lifts clear. The demo is open loop.
- No randomness with `-E object_jitter=0`. With jitter, each reset puts the receipt back in the slot of the terminal
  wherever the terminal starts.

## Files

- `task.py`: the scene (terminal, receipt, tray), the tear line, `success()`, `score()` and the scripted `demo()`.
- `sheetkit.py`: the paper strips, their hinge law, and the paper and fingertip friction.
- `yamkit.py`: spawns the receipt as an articulation and gives the scorer the grippers and joint states.
- `yamgeom.py`: the arms' collision hulls, read from the competition environment at run time, as surface points for
  the hands-clear check.
- `guardkit.py`: the hand's box model, used for the hands-clear check in the offline tests (no link poses there).
- `test_task.py`: offline checks of the scorer, the tear, the cheats and the demo's geometry and forces:
  `python3 test_task.py`.

## Sources and licenses

- Every file in this folder is the task's own code and text; there are no third-party files in it.
- The robot model and its collision hulls (the YAM URDF, its hulls and USD, `yam_arena/assets/yam`) load at run time
  from the competition environment (yam-isaac-lab-arena revision 0059d02a), and the simulator is the environment's
  Isaac Sim; neither is redistributed here. The arm shapes the hands-clear check uses are sampled from those collision
  hulls at run time (`yamgeom.py`); nothing derived from them is stored in this folder. Their terms are the
  environment's: the YAM model is I2RT Robotics' (MIT License), and the environment's README states that it has no
  license yet.
