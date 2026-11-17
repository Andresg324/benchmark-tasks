# Give an epinephrine auto-injector trainer into a training pad, release straight off, click, hold 3 s, back on the tray

An injection training pad lies on the table in front of the arms, standing for the middle of the outer thigh of a
patient lying down (the maker's leaflet tells the patient to lie down and raise the legs, so the outer-middle thigh
faces up and out, toward the person giving the pen). On a medication tray to the right, an epinephrine auto-injector trainer lies in the groove of a foam rest, blue safety
release on, orange tip toward the robot. The trainer ships loose in the EpiPen carton, without a carrier tube, so it
starts out lying on the tray as it is set out for teaching.

The instruction is: "Give the trainer auto-injector into the target on the training pad: pull off the blue safety release, press the orange tip into the pad until it clicks, hold it there for 3 seconds, then lay the trainer back on the tray."

Everything here is a training set-up: the pen is the maker's needle-free, drug-free trainer and the target is a
training pad on the bench, standing for the outer thigh. The pad stays on the bench for every trial.

Registered nurses ([BLS 29-1141](https://www.bls.gov/oes/current/oes291141.htm), about 3.3 million US jobs) have the O*NET task "Administer medications to patients and
monitor patients for reactions or side effects" (29-1141.00); licensed practical nurses ([29-2061](https://www.bls.gov/oes/current/oes292061.htm), about 650,000)
"Administer prescribed medications or start intravenous fluids". An epinephrine auto-injector, used on the outer thigh, is the
emergency treatment for anaphylaxis, and the maker's steps for the real pen are taught on the needle-free trainer that comes in every
EpiPen 2-Pak carton: grasp the pen with the orange tip pointing down ("blue to the sky, orange to the thigh"), pull the
blue safety release straight up without bending or twisting it, hold the orange tip about 10 cm from the outer thigh,
press it firmly against the middle of the outer thigh at a right angle until it clicks, and hold it there for 3 seconds (Viatris
patient leaflet and trainer instructions, UK-EPI-2025-00011; epipen.com "How to use"). Here the same steps are done with
the trainer on the training pad. Food allergy alone affects
10.8% of US adults (Gupta et al., JAMA Netw Open 2019;2:e185630, [PubMed 30499721](https://pubmed.ncbi.nlm.nih.gov/30499721/)), and the trainer is
for showing patients and carers how (the leaflet: "demonstrate to carers, work colleagues, friends and family").

The real pen's force is published and small. The EpiPen Jr, the same device at a lower dose, fires at 23.01 +- 3.96 N and its release
pulls off at 9.56 +- 2.36 N (n = 15 each; Schwirtz & Seeger, "Are adrenaline autoinjectors fit for purpose?", J Asthma
Allergy 2010;3:159-167). For the 0.3 mg EpiPen every activation measured was below 50 N, and the authors loaded it with
20 N as "the force required for activation" (Schwirtz & Seeger, J Asthma Allergy 2012;5:39-49). The trainer's own force
is not published. The simulation uses 23 N. The press here is straight down, where
the YAM can push about 52 N continuously (computed from the arm's URDF and its DM-J4340/DM-J4310 joint ratings,
gravity included), and its gripper closes at up to 50 N (the cap in the i2rt driver). Every score is read from positions. The task is about:

- turning a pen that lies on a tray so it stands orange tip down in one hand ("blue to the sky, orange to the thigh": held
  the other way, a real pen fires into the thumb that covers its tip);
- taking the release off with the other hand, straight along the pen (bent, it jams on its pin);
- pressing square onto a curved target, hard enough to click, without skidding off it;
- keeping it there, pushed in, for 3 s after the click, then laying it down. With this cartridge design the injected
  volume has plateaued 2.5 s after firing (2010 study above), so on the real pen the click and the 3 s hold are what
  decide the dose; lifting early gives a short one.

## Real objects (full size)

Positions in cm from the midpoint between the arm bases (x forward, y to the robot's left), table top at z 0.

| Object | Product and size | Where |
|---|---|---|
| Injection pad (fixed by its weight on a bench mat) | Limbs & Things 00310 Injection Trainer ([maker's page](https://limbsandthings.com/us/products/00310/00310-injection-trainer)): 21 cm long, 16 cm wide, 11 cm high, 0.6 kg; epidermis, dermis, fat and muscle layers, for intradermal, subcutaneous and intramuscular injection practice; made to strap to an arm or thigh. A 6 x 4 cm target outline is drawn on its cover at the top of its crown (6 cm along the pad, 4 cm across the crown) | long side along y, center at (30.0, -9.0) |
| Pen | EpiPen Trainer (Viatris; shaded gray and labeled TRAINER; one comes in every EpiPen 2-Pak carton): the EpiPen's body and trigger, no needle, no drug; modeled 14 cm long with its blue safety release on and 2.1 cm across; orange tip 1.2 cm proud; release about 2.4 cm long over a 10 mm pin. The scorer reads only the tip, the axis and the middle, so a pen 1 cm longer scores the same | in the groove, orange tip toward the robot, middle at (24.0, -27.5) |
| Tray (fixed) | Any flat white plastic medication tray, 22 x 14 x 1 cm (the size used here; the scorer uses its outline) | center at (24.0, -27.5) |
| Pen rest | Cut from 1 cm white craft foam (EVA) sheet: a block 16 x 5 x 1 cm with a 2.4 cm groove 4 mm deep along x, so the round pen cannot roll | on the tray, centered |

## Scoring

The task file follows the trial after every control step; `score()` gives the state at the end.

- **0.25:** the blue release is off (more than 15 mm from its seat) and the pen has not fired.
- **0.5:** the pen fired on target: its orange sleeve went 7 mm into the body (the click) with the tip on the pad's
  cover inside the 6 x 4 cm target and the pen within 20 degrees of the pad surface's normal, orange end in; but it was lifted,
  let off or slid off the target before 3 s.
- **0.75:** fired on target and held: for 3 s after the click the sleeve stayed pushed in at least 4 mm with the tip on
  the target and square; not yet laid on the tray, let go or still.
- **1.0:** as 0.75, and the pen lies on the tray (its middle inside the tray's outline and within 3 cm above it, its
  axis within 30 degrees of level), let go by both hands (every tool point and pad tip 1 cm clear) and still for 1 s.
  `success()` ends the episode once that has held for 1 s.
- **0** otherwise, including doing nothing. The whole trial scores 0 if the pen fires anywhere but the target (into
  the air, into the table, beside the target, at a slant, blue end first) or falls off the table.

The sleeve cannot reach the click while the release is on (as on the real pen), so pressing with the release on leaves
the pen unfired. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

## How the pen is modeled

The pen is a rigid body with its orange sleeve on a slide along the pen axis (8 mm travel). Its return spring is a
stiff implicit drive, 3,286 N/m with the release off, so the sleeve reaches its 7 mm click at 23 N, the published EpiPen
Jr activation force (`F_FIRE`, default 23, sets it; the trainer's own force is not published, so the probe also runs at
15, 30 and 45 N, the last near the 0.3 mg pen's 50 N upper bound). While the release is on its pin the drive is 200,000 N/m: the sleeve is blocked. The click is read from
the sleeve's position for two control steps in a row. The release is a loose cap whose bore fits over a 10 mm pin with
1.5 mm clearance; its 9.6 N press fit is not modeled, so any straight pull takes it off, and a bent pull binds on the
pin. The pad is rigid with its real size and crown (a cylinder of radius 8.5 cm over a 3 cm base: 16.2 cm wide, 11 cm
high); the tip counts as on the cover within 4 mm of its surface (the trainer's tip on the soft cover). Where the tip is
is measured in the pad's own frame.

## Run a trial

Following the task board's simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Give the trainer auto-injector into the target on the training pad: pull off the blue safety release, press the orange tip into the pad until it clicks, hold it there for 3 seconds, then lay the trainer back on the tray." \
  --scorer task_score --no-prompt --max-steps 1200 --store-frames --log-dir runs/dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end. The layout is fixed
(`object_jitter=0`). Runtime per trial: at most 1,200 steps (1 minute at 20 Hz); the scripted demo's plan ends after about
38 s and success ends it about 33 s in.

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream`.

- **Right arm:** comes down over the pen with its fingers across it at its middle, grips it, lifts it, and turns its
  grip forward over 5 s so the pen stands orange tip down at (26, 0), tip 2.6 cm above the table. It holds it there
  while the left arm takes the release off, carries it over the target with the tip 4 cm above the crown, pushes down
  (commanded 6.5 cm past touching, `PUSH_PAST`: the arm gives about 2.5 cm at 13 N) until the sleeve clicks, holds it 3.6 s, lifts it off, turns it back
  level at (27, -2), lowers it into the rest's groove, opens and moves away.
- **Left arm:** waits behind the release at its height, slides its open fingers round it from behind (the left arm
  cannot reach far enough above it to come straight down onto it), pinches it, pulls it straight up 2 cm, sets it down
  on the table at (24, 8), opens and moves away.

The demo plan was checked offline with the arm's own IK (`yam_arena.demo.ArmPlan` and `ik.tcp_pose`, every 0.05 s): no step misses its target by more
than 3 mm or moves a joint more than 4 degrees in a step, for either arm.

## Offline checks

`test_task.py` checks the layout, the scoring ladder on hand-built trials (release off, pressing with the release on,
an early lift, the hold, laying down, still held), every misfire, the pen off the table, a new trial, the target check
over 125 tip placements and tilts inside the target and 125 outside it, the tray check over 125 pen poses, the demo
poses and the thresholds in this README and task.json, without Isaac Sim:

```bash
python3 test_task.py
```

## Real-world build and reset

- Pad: a Limbs & Things 00310 Injection Trainer on a rubber bench mat; tape its outline on the mat at the position
  above. Draw the 6 x 4 cm target on its cover with a fine marker (or a printed outline under clear tape), centered on
  the top of the crown.
- Pen: an EpiPen Trainer. Tray and rest as in the table above; tape the pen's mark (a line at its middle) on the rest.
- Reset (about 30 s): reset the trainer as its instructions say: "Replace the blue safety cap. Place the orange tip on
  a hard surface, squeeze the sides of the orange tip and push down" (Viatris, UK-EPI-2025-00011: "To practise again,
  the Trainer Pen must be reset"). Lay it in the groove at its mark, orange tip toward the robot; check the pad is on
  its outline. Wipe the ink mark off the cover with a damp cloth. The ink dot is the only consumable.
- Safety: the trainer is needle-free and drug-free and is labeled TRAINER; keep real auto-injectors out of the work
  area. The pad stays on its bench mat, clear of people, for every trial. The press is the trainer's own spring
  (published for the real 0.3 mg pen: under 50 N), straight down onto the pad.
- Room 20-24 C (ordinary indoor range), so the pad's cover and the trainer's spring behave the same every trial.
- Grippers: the trial is clean. The ink dot sits on the orange tip, away from the grip at the pen's middle; wipe a
  gripper pad with a damp cloth if ink ever reaches it.

### Parts

- Injection pad: Limbs & Things 00310 Injection Trainer (its 8.5 cm crown and layers are what the target and the angle
  cards are drawn for).
- Pen: EpiPen Trainer (Viatris), needle-free and drug-free; one comes in every EpiPen 2-Pak carton, and school-nurse and
  first-aid suppliers sell it alone.
- Tray: any flat white plastic tray, 22 x 14 cm.
- Pen rest: 1 cm white EVA craft foam sheet, cut to 16 x 5 cm with a craft knife.
- Marks and cards: masking tape, a fine water-soluble marker, the two printed angle cards on paper.
- Camera: any phone camera at 30 fps on a stand.

About $195 in all, excluding the robot and the phone.

### Real-trial check (outcome only)

| Score item | How it is read on the real bench |
|---|---|
| Release off | The release is in neither hand's grip on the pen and lies apart from it (video) |
| Fired | The trainer's click (audible on the video); after it is lifted the trainer stays in its fired state until it is reset |
| On target | A water-soluble ink dot on the orange tip leaves its mark inside the target outline (photo) |
| Square | The pen within 20 degrees of the pad surface's normal at its mark, read on two side video frames at the click: one along the pad against a printed card of the crown's 8.5 cm profile with a 20-degree wedge drawn about the normal at every centimeter of the target, one across the pad against a printed 20-degree card (along the pad the normal is vertical) |
| Held 3 s | Video frames (30 fps) from the click to the tip leaving the cover: at least 90 frames |
| On the tray, let go, still 1 s | Photo of the pen on the tray; on video both hands off it and nothing moving for 30 frames |

### Tolerances

| Quantity | Value | How it is read | Source |
|---|---|---|---|
| Click depth | 7 mm of sleeve travel | the trainer's audible click | modeled; the click is the trainer's own mechanism |
| Fire force (sim) | 23 N (15, 30 and 45 N also probed) | sim only; the real trial reads the click | Schwirtz & Seeger, J Asthma Allergy 2010 (EpiPen Jr 23.01 +- 3.96 N); 0.3 mg pen under 50 N (2012); the trainer's force is unpublished |
| Target | 6 x 4 cm on the crown | ink mark inside the outline (photo) | design choice: the middle of the outer thigh, a hand-span target |
| Angle | within 20 degrees of the pad surface's normal | printed 20-degree cards, two video frames | Viatris leaflet: "at a right angle"; 20 degrees is the allowance |
| Hold | 3 s after the click, sleeve in at least 4 mm | 90 frames at 30 fps | Viatris leaflet (hold 3 seconds); volume plateaus at 2.5 s (2010 study) |
| Release off | more than 15 mm from its seat | video | past the 10 mm pin (modeled) |
| On the tray | middle inside the outline, within 3 cm above, axis within 30 degrees of level | photo | design choice: lying on the tray |
| Still | 1 s, both hands 1 cm clear | 30 video frames | design choice: read once nothing moves |
| Room | 20-24 C | room thermometer | ordinary indoor range |

## License

This task is released under the MIT license. `yamkit.py` and `guardkit.py` are the prop and scoring libraries it
uses, released under the same license. Every object is built from primitive shapes.
