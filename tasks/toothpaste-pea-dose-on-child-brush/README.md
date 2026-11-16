# Put a pea-sized dab of toothpaste on a child's toothbrush

`task.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), `station` scene. A child's manual
toothbrush lies on its back, bristles up, on the robot's left. A partly used Crest Kid's 4.6 oz (130 g) toothpaste
tube lies on its side on the right, its flip cap flipped open. The robot gets the brush ready for a 4-year-old:
- one hand holds the brush bristles-up while the other squeezes a pea-sized dab (0.15 to 0.35 g) onto the bristles;
- it puts the tube back down.

Helper modules:
- `yamkit.py` spawns the props: the brush, and the tube as an articulated body with four sprung wall panels and a
  hinged, snap-open flip lid.
- `guardkit.py` checks gripper contact.
- `squeezekit.py` is the paste flow model (see "The paste model").

`test_task.py` checks the scorer offline: `python3 test_task.py` (or `python3 -m pytest -q test_task.py`).

Requirements: `yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its
`setup.sh` (Isaac Lab-Arena `release/0.3.1`, `inspect-robots` 1.0.0, `inspect-robots-yam` 0.38.0), and
`source $ARENA_DIR/yam.env` in the shell.

## Run a trial

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/toothpaste-pea-dose-on-child-brush/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the toothbrush ready for a 4-year-old." \
  --scorer task_score --max-steps 1200 --store-frames --log-dir logs/toothpaste-demo
```

- Scorer checks: `TOOTH_VARIANT=much` runs the same demo with a firmer, longer squeeze (too much), and
  `TOOTH_VARIANT=smear` with a short light one.
- `TOOTH_PROBE=<file>` writes one line per control step with the lid angle, the four panel pushes, the pressure, the
  grams on and off the bristles, the bristle angle and the poses.
- Model run (Dream-1): the same command with `--policy dream --max-steps 1200`.
- Video: `inspect-robots video logs/toothpaste-demo/*.json`.
- The scorer (`task_score`) reports the score at each step; the trial's score is the one at its end.

## Why it matters (Societal relatability)

Putting toothpaste on a brush is something nearly everyone has done, and parents of young children do it for them
twice a day.

**The dose matters.** For children aged 3 to 6 the CDC, ADA and AAP advise no more than a pea-sized amount of
fluoride toothpaste, about 0.25 g. Under age 3 the advice is a smear the size of a grain of rice. In the US survey
behind that advice, only 49.2% of children aged 3 to 6 used a pea-sized amount, and over 38% used a half or full
brush load (20.6% half, 17.8% full) (CDC MMWR 2019;68(4):87-90,
https://www.cdc.gov/mmwr/volumes/68/wr/mm6804a3.htm).

Young children swallow much of the paste, and fluoride swallowed while the enamel is still forming (under about age
8) causes dental fluorosis. Fluorosis is the most common cosmetic enamel defect: white flecks or streaks, mostly
mild. Among US 16-17 year olds, fluorosis graded "very mild" or worse rose from 29.7% (NHANES 2001-02) to 61.3%
(2011-12) (Wiener et al. 2018, https://pubmed.ncbi.nlm.nih.gov/29500282/). Too little paste (a smear at age 3 to 6)
gives up the caries protection the fluoride is there for.

**The robotic difficulty is control, not a rule.**
- The dose is set by how far and for how long a deformable tube is squeezed.
- The paste is a yield-stress material: nothing comes until the squeeze passes a threshold, and it keeps coming for a
  moment after the squeeze eases.
- The nozzle has to be aimed at a 20 x 9 mm bristle field that the other hand is holding level.

Too much is the easy failure, as it is for parents.

### How it differs from other squeeze and dosing tasks

**Piping a shell border on a cake (another team's task):**
- Icing goes through a rigid syringe whose piston the hand pushes, and the score is the shape the icing leaves (shell
  heads, tails and ridges).
- Here the hand squeezes a deformable tube's walls. The output is a dose by mass, onto a target the robot holds and
  aims with its other hand.
- Nothing here is graded on shape.

**Pouring a 10 mL dose, or pouring to a line (other tasks):** these pour a liquid by tilting a container, with gravity
setting the flow. A toothpaste tube cannot be poured. The paste moves only while the walls are pressed hard enough to
overcome its yield stress, and the dose depends on the squeeze, not the tilt.

**Gluing a broken mug handle back on (another team's task):** one hand takes a gel-glue tube from its holder and
squeezes a thin bead onto two broken faces of a mug lying in a cradle, as one step of a repair scored mainly on the
dry fit, seating and press; the bead is a volume band, and the glue flows as soon as the squeeze passes a force. Here the squeeze is the whole task, with these differences:
- both hands work together: one holds the brush, the other squeezes;
- the target is held in the air, not fixed;
- the score is a weighed mass band from real children's dosing data, and the paste's yield and after-flow make the
  stop, not just the start, the hard part.

## Scoring

Grams on the bristles = mL x 1.30 g/mL. That density is for a silica toothpaste: toothpastes run 1.3 to 1.6 g/mL,
silica ones at the low end and chalk ones at the high end, and Crest Kid's lists hydrated silica (confirm on the box).
The real trial weighs grams directly, so the density matters only in simulation.

| Score | Condition | Why this level |
|---|---|---|
| 1.0 | the 0.75 state; the bristles within 45 degrees of facing up (held or set down); the tube lying on the counter (its middle under 3 cm up) and released; each let go lying on the counter, not dropped or thrown; both still (under 1 cm/s) for 1 s | the brush is ready and the tube is out of the hand, so no more paste can come |
| 0.75 | 0.15 to 0.35 g on the bristle field and at most 0.05 g anywhere else | the right dose, not yet finished |
| 0.5 | 0.15 to 0.35 g on the bristles but more than 0.05 g off them (counter, head sides, neck, handle) | the right dose, with a mess that a parent would wipe up |
| 0.25 | 0.05 to 0.15 g (a smear: the under-3 dose, too little at 4) or 0.35 to 0.70 g (too much: a half load) | the wrong dose for the child, but some control of the squeeze |
| 0 | under 0.05 g or over 0.70 g (a full adult strip) | no dose, or the full load the CDC survey counts as overuse |

Doing nothing scores 0: no paste is on the bristles. Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

**Why 0.15 to 0.35 g.**
- The CDC defines a pea-sized amount as 0.25 g.
- In a study of 40-48-month-olds told to use a pea, the doses averaged 0.256 g (range 0.035-0.620 g, n = 28; den
  Besten, https://www.aapd.org/globalassets/media/publications/archives/denbesten-18-04.pdf).
- UK parents asked for a "pea" dispensed about 0.30 g, and their usual dose was about 0.36 g
  (https://eld.bl.uk/catalog/100046207873_0x000001; figures from the abstract).

So the band's middle is the CDC pea and the study mean, and its 0.35 g top sits where parents actually dose. The 0.15
g bottom is half-way between the CDC pea and the about-0.1 g rice-grain smear for under-3s
(https://www.smilesforlifeoralhealth.org/topic/how-much-toothpaste/).

**Why 0.05 g off the bristles.** That is 5 counts on the 0.01 g scale, and about a fifth of a pea. It is the smallest
amount the real trial weighs reliably after a tissue wipe. More is a visible smear on the brush head or counter.

**Why the bristles within 45 degrees of up, and the tube released.** The brush is handed over ready, dab on top. Past
45 degrees a soft dab can start to slide off the bristle tips. This is a proxy, not modeled: the real trial reads the
angle from the video. The tube must be put down because while it is in a hand the squeeze is not over and more paste
can still come. Neither the arms' final pose nor the brush's place on the counter is scored.

**Why put down, not dropped or thrown.** A dropped open tube lands on its walls and can squirt paste, and a dropped
brush can bounce and smear or shed the dab; a parent lays both down. So whenever a hand lets go of the tube or the
brush, it must already be lying on the counter (the tube's middle, or the bristle top's middle, under 3 cm up: within
about 1.2 and 1.6 cm of their resting heights) and moving under 0.3 m/s. A free fall reaches 0.3 m/s after 4.6 mm,
so only a hand that lays the thing down passes; picking it up again and laying it down clears a drop. A tube or brush
that ends below the counter (on the floor) is not put down. Real trial: from the video, each is let go resting on or
within about 1 cm of the counter, not tossed, and both end on the counter or in a hand.

**Turning the bristles past sideways** (more than 90 degrees from up) with paste on them caps the score at 0.25 for
the rest of the trial. The dab can fall off or smear on the counter, and the real trial would have to re-weigh
whatever stayed on.

**What the scorer reads.**
- In simulation, paste lands on the bristles only while they face within 60 degrees of up; squeezed onto a brush
  tipped further, it lands off.
- A hand counts as holding the tube when its own shape (palm and fingers, opened as they are) touches the tube within
  2 mm (guardkit), not by nearness.
- The dose is stored in the brush's own frame, so the score is the same wherever the brush is held or laid: on the
  counter, held up anywhere, turned or tilted, or moved after dosing (`test_placement_robust`, `test_placement_sweep`).

**Why the cap starts open.** A parent flips the cap open with the thumb of the hand holding the tube, in the same
motion as picking it up, and the task starts just after that.
- The flip lid is modeled (a snap hinge) and starts at its open rest. Shut, it seals the nozzle: squeezing raises
  the pressure but nothing leaves (`test_lid_shut_seals`). Paste can leave only while the lid is at least 57 degrees
  open.
- A closed start was built and tried in four simulator probes. The left fingertip had to push a thumb tab only 3-4 mm
  proud of the rim, beside the right hand holding the tube, and it landed 1 to 1.5 cm off the tab in each probe. The
  arms track to about a centimeter that close together, and the YAM's parallel gripper has no thumb to flick a tab
  with the holding hand.
- With a closed start, the trial would mostly test whether a fingertip finds a 4 mm tab, not the dose this task is
  about. So the cap starts open, the same in simulation and in the real trial.
- Snapping it shut afterwards is good practice but is not scored, because the score is about the dose.

## The paste model

`squeezekit` (the house squeeze-bottle kit), with a toothpaste tube's numbers.

**Walls.**
- The tube's round body is a rigid core with four wall panels on prismatic joints, each sprung at 2.5 N per mm (an
  estimate; at 0.4 N per mm the holding grip could not keep the tube from sliding in the fingers once the dose left).
- A hand dispenses only by pushing panels in. Their inward push is read from the joints every control step. A grip on
  the cap, shoulder or rolled end squeezes nothing.
- Each panel displaces 1.0 mL per cm pushed. The 8 mL of air inside is compressed isothermally, and the air's pressure
  pushes back on the panels.

**Flow.**
- The paste flows through the nozzle at 0.10 mL/s per kPa above its yield pressure.
- The yield pressure is the pressure needed to push a yield-stress plug through the nozzle: 4 x yield stress x nozzle
  length / bore = 4 x 300 Pa x 9 mm / 7.5 mm = 1.44 kPa. The 300 Pa is the middle of the measured range: about 100 Pa
  on a model toothpaste (arXiv 1903.10888, https://arxiv.org/pdf/1903.10888) and about 500 Pa on a commercial-style
  formula (US 5202112). The 7.5 mm bore is a conventional tube opening of 0.28-0.32 in (US 9567138).
- As paste leaves, the air re-expands and the flow dies away, so paste keeps coming briefly after the squeeze stops.
  Easing off draws air back in and stops it.
- A 0.4 mm holding grip (about 2.2 N a side) makes 1.0 kPa, under the 1.44 kPa yield, and dispenses nothing.

**Where it lands.**
- The paste's own weight is left out (an open tube held nozzle-down does not drip), and it feeds whichever way the
  nozzle points.
- It drops plumb from the nozzle. The plumb line is cut with the bristle plane, and the hit point is stored in the
  brush's frame on a 1 mm grid of the 20 x 9 mm bristle field, so the dab moves with the brush.
- A grid cell holds a bead 4.5 mm tall; more slumps into its neighbors, and slump over the field's edge counts as off
  the bristles. Paste that misses the field (counter, hand, head sides) is off the bristles.
- The dab is drawn as blue tiles on the bristles, turned and carried with the brush.

**Fidelity note: the plumb drop.** A real dab leaves the nozzle as a soft ribbon. It touches down on the bristles,
builds up, and is broken off when the tube is lifted (often with a small twist), sometimes leaving a short string. The
model has none of that: each step's paste drops straight down from the nozzle's mouth and lands at once, with no
ribbon touching down, no stringing, no tail breaking off, and no soaking into the bristles. In the demo the nozzle is
4 mm over the bristles, about where a real 7.5 mm ribbon touches down, so the plumb drop and a real touch-down land
in the same place. A nozzle held high would string and drift in reality; the model would still land the paste exactly
underneath. The real trial weighs the paste, so none of this changes the real score; it only affects where the sim
puts the dab.

**Fidelity note: the cap.** The flip lid is a hinge with a snap. It is pulled shut within 29 degrees of closed (about
1 N at a 4 mm thumb tab) and pulled to its open rest (143 degrees) past 63 degrees; between those it swings freely. The
snap force is an estimate. A lid knocked shut seals the nozzle, as on the real tube.

**Offline doses** (`test_task.py` and the probe). The demo squeeze is 1.75 mm a side for 1.2 s, then 1.5 s at the
holding grip.

| Yield stress | Yield pressure | Push for 0.15-0.35 g | Demo squeeze (1.75 mm, 1.2 s) |
|---|---|---|---|
| 100 Pa | 0.48 kPa | about 0.9-1.75 mm | 0.34 g (in the band, high); the 1.0 kPa holding grip also leaks |
| 300 Pa (modeled) | 1.44 kPa | about 1.3-2.2 mm | 0.254 g |
| 500 Pa | 2.40 kPa | about 1.7-2.55 mm | 0.18 g (in the band, low) |

So the band can be reached across the whole literature range of yield stress. The squeeze that reaches it moves by
about 0.4 mm a side from end to end of that range. That is why the real tube's constants are calibrated (below), and
why the real trial scores grams, not the squeeze.

The scorer variants (the same squeeze model):
- 2.5 mm for 1.5 s leaves 0.43 g (too much, 0.25);
- 1.2 mm for 0.6 s leaves 0.10 g (a smear, 0.25);
- 0.9 mm for 0.3 s leaves 0.03 g (a trace, 0).

**Calibrating the constants on the trial tube** (simulation only; the real-trial score never uses them). The panel
area (1.0 mL/cm), the air (8 mL), the flow (0.10 mL/s per kPa) and the yield stress are estimates. To fit them:
1. Hold the tube nozzle-down in the YAM's right gripper (prepared as in the protocol below).
2. Command closings of 1.0, 1.5 and 2.0 mm a side for 1.2 s, five times each, reading the closing from the gripper's
   own position readout.
3. Weigh what comes out each time on the 0.01 g scale.
4. Set `AREA_ML_PER_CM`, `AIR_ML` and `TAU_Y_PA` so the model's doses match the means.

**Forces (computed, not measured).**
- The demo's squeeze makes about 4.6 kPa inside: about 9.9 N on each panel (4.4 N of wall spring, 5.6 N of
  pressure), far inside the gripper's 50 N.
- The finger's position servo (5,000 N/m) is a spring in series with the wall, so the demo commands each finger
  past the wall by the force it must make over that stiffness.
- The brush weighs 14 g and the tube about 72 g.

## Real-world evaluation (reproducible protocol)

The real trial scores only the outcome: grams on the bristles, grams off them, the bristle angle and the tube put
down. No force gauge and no squeeze readout are used.

**Products** (all off the shelf):

| Item | Product | Notes |
|---|---|---|
| Toothpaste | Crest Kid's Cavity Protection fluoride toothpaste, Sparkle Fun, 4.6 oz (130 g) tube with flip cap (Target A-13956414; CVS prodid 1010478) | the standard US size; no 4.2 oz Crest Kid's Sparkle was found |
| Toothbrush | Colgate Kids Extra Soft toothbrush, ages 2+ (small round head) | one new brush per trial, or wash and dry it fully between trials |
| Scale | American Weigh Scales AWS-100-CAL pocket scale: 100 g x 0.01 g, 2.5 x 3.0 in pan, with its 100 g calibration weight (https://www.scalesplus.com/aws-100-digital-pocket-scale) | calibrate at the start of each session |
| Rule | Starrett C604RE-6 steel rule, 150 mm with 0.5 mm graduations (or any 150 mm steel rule with 0.5 mm marks) | for the top photo of the dab, and for the tape marks |
| Also | a phone camera on a stand, masking tape, dry tissues, a small cup | |

**Starting state** (the same each trial):
1. Fill: the tube holds 50 to 70 g of paste (gross weight minus the empty tube's weight [M]; weigh one emptied,
   rinsed and dried tube of the same SKU once). Bring a new tube into range by squeezing paste into the cup.
2. Roll the tube flat from the crimp up to the paste and wipe the nozzle flush with a tissue. Flip the cap fully
   open. Let it lie 1 min so the laminate springs back.
3. Lay the tube on its side at its tape mark, nozzle toward the robot, the cap's hinge on top.
4. Lay the clean, dry brush on its back at its tape mark, bristles up, along the robot's forward direction with the
   head away from the robot.

**Measure:**
1. Before: weigh the brush (B0) and the tube (T0) on the scale.
2. Run the trial. Record video from above.
3. After: photograph the brush head from straight above next to the rule (the dab's position and size; a pea dab is
   about 5-6 mm across).
4. Wipe any paste off the brush outside the bristle field (head sides, neck, handle) with a dry tissue. Weigh the
   brush (B1). On the bristles = B1 - B0.
5. Weigh the tube unwiped (T1). Dispensed = T0 - T1. Off the bristles = dispensed - on the bristles.
6. From the video: the bristle angle at the end, whether the tube was let go on the counter, and the 1 s of
   stillness. Score with the table above.

Every weight is independent of where the brush and tube end up, and none of them credits or penalizes how the
squeeze was done.

**Safety.** Only a toothbrush and toothpaste are used: nothing sharp, hot or heavy. The brush is never used in a
mouth. Discard each dosed brush, or wash it.

**Tolerances**

| Quantity | Value | How it is measured | Source |
|---|---|---|---|
| Pea dose (full credit band) | 0.15 to 0.35 g on the bristles | scale, B1 - B0 | CDC pea = 0.25 g (MMWR 2019;68(4)); study mean 0.256 g (den Besten, AAPD); parents' usual dose about 0.36 g (BL record above) |
| Smear band | 0.05 to 0.15 g | scale | rice-grain smear about 0.1 g (Smiles for Life) |
| Too much | 0.35 to 0.70 g (0.25 credit); over 0.70 g scores 0 | scale | half and full loads in MMWR 2019;68(4); 0.70 g is about 3 peas, where the survey's "half" grades into "full" (our cut, an estimate) |
| Off the bristles | at most 0.05 g | scale, (T0 - T1) - (B1 - B0) | 5 counts of the scale; design choice (above) |
| Scale resolution | 0.01 g, 100 g capacity | AWS-100 spec | scalesplus.com listing; the tube (about 85 g with 70 g of paste) and the brush fit under 100 g |
| Bristle angle at the end | within 45 degrees of up | video frame, protractor overlay | design choice: dab on top; a proxy (above) |
| Stillness | under 1 cm/s for 1 s | video | design choice |
| Paste density | 1.30 g/mL (sim only) | not needed in the real trial | J. Soc. Cosmet. Chem. (https://library.scconline.org/v021n07/32): pastes 1.3-1.6 g/mL |
| Yield stress | 300 Pa modeled; 100-500 Pa checked (sim only) | not needed in the real trial | arXiv 1903.10888; US 5202112 |
| Tube body | 35 mm across [M: caliper] | 150 mm digital caliper, 0.01 mm | standard 35 mm laminate size (alltub.com data sheet); the carton is 44 x 48 x 180 mm (Target) |
| Tube length | 15.0 cm as modeled, rolled to the paste [M: caliper] | rule | 165-175 mm unrolled with cap (supplier tables, estimate) |
| Nozzle bore | 7.5 mm [M: caliper] | caliper | US 9567138 (conventional openings 0.28-0.32 in) |
| Flip-cap tab | 3-4 mm proud of the rim [M]; 4 mm in the model | caliper | estimate from retail photos |
| Brush | 151 mm long; head 26 x 12 mm; bristle field 20 x 9 mm, 9 mm tall; 14 g [M: all] | caliper, scale | no published dimensions for this SKU |
| Fill | 50 to 70 g of paste | scale, gross minus empty tube [M] | reset rule above |

[M] = must be measured on the trial object before the first trial; the model's values are estimates until then.

## Runtime and seeds

- Run limit: 1,200 control steps at 20 Hz (1 minute).
- Scripted demo: **1.0** (success at step 512, 25.6 s; task.py sha256 7bd09be57cce, before the drop/throw check
  was added: re-run pending). In the run, 0.284 g landed on
  the bristles and 0.010 g off them; the brush was held level (bristles within 1.1 degrees of up) and laid back flat;
  the tube was laid back and released. The left hand lifts the brush by its handle, turns it handle-left and holds it
  10 cm over the counter. The right hand lifts the tube by its body with a light holding grip, turns it nozzle-down
  (10 degrees off plumb) beside the brush, carries it over the bristles' middle with the nozzle 4 mm above them,
  squeezes 1.75 mm a side for 1.2 s, eases to the holding grip for 1.5 s, lifts off and lays the tube back. The left
  hand then lays the brush back. The demo is open loop.
- Scorer checks in the simulator (same task.py): `TOOTH_VARIANT=much` 0.25 (0.461 g on, 0.070 g off);
  `TOOTH_VARIANT=smear` 0.25 (0.111 g on); `TOOTH_VARIANT=nothing` (both hands park) 0.
- No randomness: the layout is fixed (`-E object_jitter=0`).

## How the file works

**The tube** is a yamkit `Mechanism`: a loose articulated body (`squeezekit.free_body`) with
- four prismatic wall panels whose spring law (`squeezekit.Wall`) also feels the live pressure inside;
- a revolute flip lid with a snap law, starting at its open rest.

The panels' and lid's travel opens only once their laws act (the first control step). Before that, in the arena's
settle, an unsprung panel sagged to its 10 mm stop under the lying tube (seen in the first probe). The used end is a
flat roll 55 mm wide: the tube's foot on the counter.

**Paste.**
- After every control step, `_update()` reads the brush's pose, the tube's pose, its four panel pushes and the lid's
  angle, and steps a `BrushPaste` (squeezekit's `Sauce` landing on the brush's own bristle grid).
- With the lid shut the nozzle is sealed: no flow, no venting.
- `success()` and `score()` share one update per step.
- A new trial (yamkit's episode counter) starts a fresh tube and a clean brush.

**Visible paste.** Cells with paste are drawn as blue tiles on the bristles with Isaac Lab's `VisualizationMarkers`,
placed in the brush's frame every step, so camera-based policies see the dab and it moves with the brush. The tiles
are only a picture: if drawing fails they turn off and scoring carries on. Nothing is drawn before paste lands, so
nothing pops in at the start.
