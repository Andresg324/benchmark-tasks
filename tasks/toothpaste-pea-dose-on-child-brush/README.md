# Unscrew a kids' toothpaste tube, put a pea-sized dab on a child's toothbrush and screw the cap back on

`task.py` for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena), `station` scene. A child's manual
toothbrush lies on its back, bristles up. A partly used Kid's Crest 4.2 oz stand-up toothpaste tube stands on its
screw cap, capped. The robot gets the brush ready for a 4-year-old and leaves the toothpaste capped:
- it unscrews the cap (the cap comes off only by turning it on its thread);
- it squeezes a pea-sized dab (0.15 to 0.35 g) onto the bristles, nothing off them;
- it screws the cap back on fully and lets go.

Helper modules:
- `capkit.py`: the tube's threaded neck and its screw cap as convex-hull colliders (see "The cap and thread").
- `squeezekit.py`: the paste flow model (see "The paste model").
- `yamkit.py`: spawns the props: the brush, the tube (an articulated body with four sprung wall panels) and the cap
  (a free body).
- `yamgeom.py`: the arms' own collision hulls, read at run time from the competition environment, as surface points
  for the contact checks.

`test_task.py` checks the scorer, the thread geometry and the demo plan offline: `python3 test_task.py`.

## Run a trial

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/toothpaste-pea-dose-on-child-brush/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Get the toothbrush ready for a 4-year-old, then leave the toothpaste capped." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/toothpaste-demo
```

- Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.
- Scorer checks: `TOOTH_VARIANT=much` runs the demo with a firmer, longer squeeze (too much), `TOOTH_VARIANT=smear`
  with a short light one, and `TOOTH_VARIANT=nothing` keeps both arms at their ready pose.
- `TOOTH_PROBE=<file>` writes one line per control step: the cap's rise, offset, tilt and turn on the neck, whether
  the nozzle is inside the cap, the pressure, the grams on and off the bristles, the arms' gaps to the tube and the
  cap, and the poses.
- The scorer (`task_score`) reports the score at each step; the trial's score is the one at its end.

## Why it matters (Societal relatability)

Putting toothpaste on a brush is something nearly everyone has done: unscrew the cap, squeeze, screw it back on.
Parents of young children do it for them twice a day.

**The dose matters.** For children aged 3 to 6 the CDC, ADA and AAP advise no more than a pea-sized amount of
fluoride toothpaste, about 0.25 g; under age 3, a smear the size of a grain of rice. In the US survey behind that
advice, only 49.2% of children aged 3 to 6 used a pea-sized amount, and over 38% used a half or full brush load
(20.6% half, 17.8% full) (CDC MMWR 2019;68(4):87-90, https://www.cdc.gov/mmwr/volumes/68/wr/mm6804a3.htm). Fluoride
swallowed while the enamel is still forming causes dental fluorosis; among US 16-17 year olds, fluorosis graded "very
mild" or worse rose from 29.7% (NHANES 2001-02) to 61.3% (2011-12) (Wiener et al. 2018,
https://pubmed.ncbi.nlm.nih.gov/29500282/).

**The robotic difficulty is control.**
- The cap comes off and goes back on only by turning it, about 1.1 turns, more than one wrist turn: the hand has to
  turn, let go, turn back and grip again, following the thread, and put the cap back on straight and all the way.
- The dose is set by how far and for how long a deformable tube is squeezed. The paste is a yield-stress material:
  nothing comes until the squeeze passes a threshold, and it keeps coming for a moment after the squeeze eases.
- The nozzle has to be aimed at a 20 x 9 mm bristle field.

## Scoring

Grams on the bristles = mL x 1.30 g/mL (a silica toothpaste: pastes run 1.3 to 1.6 g/mL). The real trial weighs
grams directly, so the density matters only in simulation.

| Score | Condition |
|---|---|
| 1.0 | 0.15 to 0.35 g on the bristle field and at most 0.05 g anywhere else; the cap screwed fully on; the bristles within 45 degrees of facing up; the tube resting on the counter (standing or lying) with no part of either arm touching it or its cap (in simulation every arm link at least 5 mm clear); the tube and brush each let go resting on the counter, not dropped or thrown; all still (under 1 cm/s) for 1 s |
| 0.75 | the same dose, clean and capped, but not yet let go or not yet still |
| 0.5 | a pea-sized dab with the tube left uncapped or its cap not screwed fully on; or a pea-sized dab with more than 0.05 g off the bristles |
| 0.25 | 0.05 to 0.15 g (a smear: the under-3 dose) or 0.35 to 0.70 g (too much: a half load) |
| 0 | under 0.05 g or over 0.70 g; doing nothing scores 0 (the tube starts capped, no paste out) |

Possible scores: 0, 0.25, 0.5, 0.75, 1.0. Turning the bristles past sideways (more than 90 degrees from up) with paste on them caps the score at 0.25 for the
rest of the trial: the dab can fall off or smear.

**Screwed fully on** means turned down to within 45 degrees of its seat, square and centered on the neck. In
simulation: the cap within 0.19 mm of its seat along the thread (45 degrees of the 1.5 mm pitch), its axis within
2 degrees and 0.6 mm of the neck's. In the real trial: after the trial a tester turns the cap clockwise by hand; it
must stop within an eighth of a turn. A cap set on without turning sits on the thread's crests, a cap screwed only
part way sits up the thread, and a cap started crooked cannot reach the seat: each scores at most 0.5.

**Why 0.15 to 0.35 g.** The CDC defines a pea-sized amount as 0.25 g. Children of 40-48 months told to use a pea
dosed 0.256 g on average (range 0.035-0.620 g, n = 28; den Besten,
https://www.aapd.org/globalassets/media/publications/archives/denbesten-18-04.pdf). UK parents' usual dose was about
0.36 g (https://eld.bl.uk/catalog/100046207873_0x000001; figures from the abstract). The 0.15 g bottom is half-way
between the CDC pea and the about-0.1 g rice-grain smear for under-3s
(https://www.smilesforlifeoralhealth.org/topic/how-much-toothpaste/).

**Why 0.05 g off the bristles.** That is 5 counts on the 0.01 g scale, about a fifth of a pea, the smallest amount
the real trial weighs reliably after a tissue wipe.

**Why the arms must be off the tube and cap.** A tube held on its cap, or pressed onto a loose one, is not closed
until it is let go; a cap set on loose and held there would read as closed while it is held.

**Why put down, not dropped or thrown.** Whenever the arms let go of the tube or the brush it must already rest on
the counter (its lowest point within 1 cm of it) and move under 0.3 m/s. A free fall reaches 0.3 m/s after 4.6 mm.

**What the scorer reads.**
- Paste can leave the nozzle only while the nozzle is out of the cap: with the cap on, or loosened but still over the
  nozzle, a squeeze raises the pressure and nothing leaves; the pressure comes out once the cap is off.
- In simulation, paste lands on the bristles only while they face within 60 degrees of up.
- Touching is contact on the arms' own collision hulls (link 3 to the fingertips, both arms): a hull point within
  1 mm of the tube's or the cap's outline. The hulls load at run time from the competition environment.
- The dose is stored in the brush's own frame, so the score is the same wherever the brush is held or laid
  (`test_placement_sweep`).

## The cap and thread

The tube is the Kid's Crest Cavity Protection Bubblegum 4.2 oz tube; its listing's directions read "twist off cap to
remove foil seal" (Target, TCIN 1001365458, UPC 037000951438): a screw cap. Its closure is modeled on the standard
35 mm tube screw closure (Amcor's 35 mm tube closures: neck 15 mm, neck finish "15 x 1,5", closure 34.2 mm across and
20.8 mm tall, 4.4-5.3 g, polypropylene).

- **Thread.** A single-start right-hand thread, 15 mm across at the neck's crest, 1.5 mm pitch. The neck carries a
  male ridge and the cap a female ridge in its bore. Each ridge is a helix of short convex hulls, one per 15 degrees of
  turn, with a trapezoid section (wider at the root than at the crest), each hull sharing its end faces with its
  neighbors so a ridge slides along the other without ledges; the ridges' height runs out at both ends as a molded
  thread starts. The cap's bore is a ring of convex wedges with a lead-in at its mouth; its top is one convex disc. The
  neck's core is one convex cylinder with a chamfered tip, the shoulder one convex cone.
- **Nothing holds the cap on but contact.** There is no joint, spring, capture zone or snap between cap and tube.
  The offline check (`test_thread`) finds no overlap between any cap hull and any neck hull seated or anywhere along
  the helix, a straight pull blocked by the ridges, and the cap free to pull off only after it has turned about 1.1
  turns.
- **Set as a molded thread of that size would be (not published), and checked on the trial tube:** the thread's depth,
  its turns and its clearances. The trial tube's cap is checked by counting the turns from seated to free.
- **Friction.** The cap and the neck are molded polyolefins: 0.3 static and 0.25 sliding, both on the thread and where
  the cap stands on the counter. The tube's body and the brush keep the arena's object friction.
- **Start.** The cap starts screwed fully on, the tube standing on it. If the arena shifts the tube at reset, the cap
  is put back screwed on under it before anything moves.

## The paste model

`squeezekit` (the house squeeze-bottle kit), with a toothpaste tube's numbers.

**Walls.** The tube's round body is a rigid core with four wall panels on prismatic joints, each sprung at 2.5 N per
mm. A hand dispenses only by pushing panels in; their push is read from the joints every control step. Each panel
displaces 1.0 mL per cm pushed. The 8 mL of air inside is compressed isothermally and pushes back on the panels.

**Flow.**
- The paste flows through the open nozzle at 0.10 mL/s per kPa above its yield pressure.
- The yield pressure is the pressure needed to push a yield-stress plug through the nozzle: 4 x yield stress x nozzle
  length / bore = 4 x 300 Pa x 9 mm / 7.5 mm = 1.44 kPa. The 300 Pa is the middle of the measured range: about 100 Pa
  on a model toothpaste (arXiv 1903.10888) and about 500 Pa on a commercial-style formula (US 5202112). The 7.5 mm
  bore is a conventional tube opening of 0.28-0.32 in (US 9567138).
- As paste leaves, the air re-expands and the flow dies away, so paste keeps coming briefly after the squeeze stops.
- The demo's holding grip (0.4 mm a side) makes 1.0 kPa, under the yield, and dispenses nothing.

**Where it lands.** The paste drops plumb from the nozzle (an open tube held nozzle-down does not drip, so its own
weight is left out). The plumb line is cut with the bristle plane and the hit point is stored in the brush's frame on
a 1 mm grid of the 20 x 9 mm bristle field, so the dab moves with the brush. A grid cell holds a bead 4.5 mm tall;
more slumps into its neighbors, and slump over the field's edge counts as off the bristles. The dab is drawn as blue
tiles on the bristles.

**Fidelity note: the plumb drop.** A real dab leaves the nozzle as a soft ribbon that touches down, builds up and is
broken off when the tube is lifted. The model has no ribbon, stringing or tail. In the demo the nozzle is 4 mm over
the bristles, about where a real 7.5 mm ribbon touches down, so both land in the same place. The real trial weighs
the paste, so this does not change the real score.

**Offline doses** (`test_task.py`): the demo's squeeze (1.75 mm a side for 1.2 s, then 1.5 s at the holding grip)
leaves a pea-sized dose; a firmer, longer squeeze (2.5 mm for 1.5 s) too much; a short light one (1.2 mm for 0.6 s) a
smear. The squeeze constants are calibrated on the trial tube (below); the real trial scores grams, not the squeeze.

**Calibrating the squeeze on the trial tube** (simulation only). Hold the prepared, uncapped tube nozzle-down in the
YAM's gripper at the holding grip over a cup on the 0.01 g scale. Close in 0.1 mm steps a side, 1.2 s each, until the
scale first gains 0.01 g (sets the yield stress). Then command closings of 1.0, 1.5 and 2.0 mm a side for 1.2 s, five
times each, weigh each dose, and fit the panel area and flow constants in `test_task.py`'s offline squeeze until each
mean matches within 0.02 g.

## The demo

Open loop, the right arm only (the left stays at its ready pose). The hand comes down over the standing tube with
its fingers open either side of the flat crimp and grips the body lightly across its upper half. It unscrews the
tube counterclockwise (seen from above) off the cap, which stays standing on the counter: half a turn per wrist
stroke, rising with the thread and aiming a little down into it, then opening wide, turning back and gripping again,
1.25 turns in all. It lifts the tube off, carries it nozzle-down over the bristles with the nozzle 4 mm above them,
squeezes 1.75 mm a side for 1.2 s, eases to the holding grip for 1.5 s and lifts off. It lowers the nozzle into the
cap, screws the tube back on clockwise in the same strokes, a little past the seat (the pads slip once it is tight),
lets go, lifts clear and returns to its ready pose.

## Real-world build

The real trial scores only the outcome: grams on and off the bristles, the cap's seat, the bristle angle, and the
tube let go on the counter with the arms off it. No force gauge and no squeeze readout are used.

**Products** (all off the shelf):

| Item | Product | Notes |
|---|---|---|
| Toothpaste | Kid's Crest Cavity Protection fluoride toothpaste, Bubblegum, 4.2 oz stand-up tube with screw cap (Target TCIN 1001365458, UPC 037000951438) | its directions: "twist off cap to remove foil seal"; remove the foil seal before the first trial |
| Toothbrush | Colgate Kids Extra Soft toothbrush, Ocean Explorer 4-pack (Target A-75563150, UPC 035000458476) | one new brush per trial, or wash and dry it fully between trials |
| Scale | American Weigh Scales AWS-100-CAL pocket scale: 100 g x 0.01 g, with its 100 g calibration weight | calibrate at the start of each session |
| Rule | any 150 mm steel rule with 0.5 mm marks | for the top photo of the dab and checking the marks |
| Tape measure | any metric tape measure | for laying out the marks from the arm bases |
| Caliper | any 150 mm digital caliper | for the tube, cap and brush dimensions marked [M] |
| Thermometer | any room thermometer reading to 1 C | the room must be 20-24 C |
| Also | a phone camera on a stand, masking tape, dry tissues, a small cup, nitrile exam gloves | |

A low-cost setup: the robot and a phone aside, every item is a common household or hobby item.

**Setup marks** (once). From the midpoint between the arm bases, x forward, y to the robot's left:
- Brush: a tape outline of the brush lying on its back, the bristle field's middle at x 300 mm, y -40 mm (within
  5 mm), its long axis along x, head away from the robot.
- Tube: a tape circle for the cap at x 300 mm, y -200 mm (within 5 mm); the tube stands on its cap with the flat
  crimp along y.

**Starting state** (each trial). Keep the room, the paste and the scale at 20-24 C (a paste's yield stress changes
with temperature).
1. The tube holds 50 to 70 g of paste (gross weight minus the weighed empty tube and cap). Roll it flat from the crimp
   up to the paste, wipe the nozzle flush, screw the cap on by hand until it stops, and stand the tube on its cap at
   its mark. Let it rest 1 min so the laminate springs back.
2. Lay the clean, dry brush on its back at its mark.
3. Both arms at their ready pose.

**Gripper protection.** Each gripper pad gets a disposable nitrile finger cover cut from an exam glove, replaced
whenever paste touches it and at the end of the session. A 10 s hold check on the tube at the start of the session
confirms the grip.

**Measure.**
1. Before: weigh the brush (B0) and the tube with its cap (T0).
2. Run the trial, recording video from above and from the side of the tube.
3. Cap check: turn the cap clockwise by hand; it must stop within an eighth of a turn (mark the cap and the tube's
   shoulder with a pen line first to read the turn). A cap left off or loose fails this check.
4. Photograph the brush head from straight above beside the rule.
5. Wipe any paste off the brush outside the bristle field and weigh the brush (B1): on the bristles = B1 - B0.
6. Weigh the tube with its cap, unwiped (T1): dispensed = T0 - T1; off the bristles = dispensed - (B1 - B0).
7. From the video: the bristle angle at the end, whether the arms are off the tube and cap at the end, whether the
   tube and brush were let go resting on the counter (not tossed), and the 1 s of stillness.

**Safety.** Only a toothbrush and toothpaste are used: nothing sharp, hot or heavy. The brush is never used in a
mouth; discard or wash each dosed brush.

**Tolerances**

| Quantity | Value | How it is measured | Source |
|---|---|---|---|
| Pea dose (full credit band) | 0.15 to 0.35 g on the bristles | scale, B1 - B0 | CDC pea 0.25 g (MMWR 2019;68(4)); study mean 0.256 g (den Besten); parents' usual dose about 0.36 g |
| Smear band | 0.05 to 0.15 g | scale | rice-grain smear about 0.1 g (Smiles for Life) |
| Too much | 0.35 to 0.70 g (0.25); over 0.70 g scores 0 | scale | half and full loads in MMWR 2019;68(4); 0.70 g is about 3 peas (our cut) |
| Off the bristles | at most 0.05 g | scale | 5 counts of the 0.01 g scale |
| Cap screwed fully on | stops within an eighth of a turn (45 degrees) when turned clockwise by hand | pen mark on cap and shoulder | design choice: a cap that still turns is not closed |
| Arms off the tube and cap | no contact at the end (in simulation every link at least 5 mm clear) | video | design choice: a held cap is not a closed one |
| Bristle angle at the end | within 45 degrees of up | video frame | design choice: dab on top |
| Stillness | under 1 cm/s for 1 s | video | the score is read once nothing is moving |
| Paste density | 1.30 g/mL (sim only) | not needed in the real trial | pastes 1.3-1.6 g/mL (J. Soc. Cosmet. Chem.) |
| Yield stress | 300 Pa modeled (sim only) | not needed in the real trial | arXiv 1903.10888; US 5202112 |
| Neck thread | 15 mm, 1.5 mm pitch; depth, turns and clearances [M] | caliper; count the turns from seated to free | Amcor 35 mm tube closures (neck finish 15 x 1,5) |
| Cap | 34.2 mm across, 20.8 mm tall, 4.5 g [M] | caliper, scale | Amcor 35 mm tube closures (34.2 mm, 20.8 mm, 4.4-5.3 g) |
| Tube body | 35 mm across, rolled to the paste [M] | caliper | standard 35 mm laminate size |
| Nozzle bore | 7.5 mm [M] | caliper | US 9567138 (0.28-0.32 in) |
| Brush | 151 mm long; head 26 x 12 mm; bristle field 20 x 9 mm, 9 mm tall; 14 g [M] | caliper, scale | no published dimensions for this SKU |
| Fill | 50 to 70 g of paste | scale | reset rule above |
| Setup marks | brush at (300, -40) mm, tube at (300, -200) mm, within 5 mm | tape measure, rule | the simulator's start |
| Room temperature | 20-24 C | room thermometer | keeps the paste's yield stress steady between trials |

[M] = measured on the trial object before the first trial; until then the value given is the model's. No [M] item is
part of the real score.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes).
- No randomness in the scripted demo: the layout is fixed (`-E object_jitter=0`).

## Sources and licenses

- `task.py`, `capkit.py`, `squeezekit.py`, `yamkit.py`, `yamgeom.py` and `test_task.py` were written by this task's
  team.
- The robot model and its collision hulls load at run time from the competition environment (arena revision
  0059d02a) and are not redistributed: `yamgeom.py` reads them through `yam_arena.analysis.link_hulls()` and caches
  surface points in the system temp folder, outside the task folder.
- No meshes, images or data files from third parties are in this folder.

## Files

- `task.py`: the scene, the scorer and the scripted demo.
- `capkit.py`, `squeezekit.py`, `yamkit.py`, `yamgeom.py`: the task's helper modules.
- `test_task.py`: offline checks.
- `task.json`, `README.md`.
