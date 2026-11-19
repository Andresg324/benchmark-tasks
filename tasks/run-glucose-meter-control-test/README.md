# Run a glucose meter's control check, seating a 0.4 mm strip and touching only its tip to a drop of control

A blood glucose meter is checked with the maker's control solution before it is used for patients. The robot runs
that check, in this order: it uncaps the control solution (one hand holds the bottle level, the other pulls the cap
off and stands it on the counter) and lays the bottle back in its rest; it takes a new strip from its bottle and
pushes it fully into the meter's port; it squeezes one drop of control solution onto a clean glass slide; it touches
the strip's tip (and only its tip) to the drop and holds it there until the strip draws the sample; then it puts the
meter down, caps the control solution (one hand holds the bottle level while the other pushes the cap on) and lays
it back in its rest.

Instruction: "Run a control-solution test on the glucose meter with a new strip, then put the meter down, cap the
control solution and lay it back in its rest."

## Why it matters (Impact on jobs)

- **Who does it:** licensed practical nurses (O*NET 29-2061, 666,900 employed), task "Collect samples, such as blood,
  urine, or sputum from patients, and perform routine laboratory tests on samples."; medical assistants (31-9092,
  833,900 employed), task "Perform routine laboratory tests and sample analyses." The Virginia Department of Health
  assisted glucose monitoring template also lists certified nursing assistants and medical assistants among the testers.
- **Why it is core, not ancillary QC:** in hospitals, clinics and long-term care, every point-of-care glucose meter
  must pass a control check with two control levels every 24 hours, and the meter locks out patient testing until it
  passes (Adventist HealthCare policy POC 0002: "Two levels of quality control must be run every 24 hours", with unit
  lock-out times; CentraCare StatStrip procedure: QC "minimally every 24 hours", testing "performed by patient care
  staff"). No patient can be tested on that meter until this check passes.
- **Same hands as patient testing:** the strip steps (seat a strip fully in the port, touch only its tip to the
  sample) are those of every fingerstick glucose test. The task uses the Contour Next One, a home meter; facility
  meters (Accu-Chek Inform II, Nova StatStrip) are sold to facilities, not to the public.
- **Home use (secondary):** the maker's user guide asks for a control test with every new bottle of strips, on first
  use, and when results look wrong.

## What makes it hard for a robot
- **A 0.4 mm part into a 1 mm slot.** The strip's gray end has to go fully into the port: the meter turns on only
  when the strip reaches its contacts. A strip a millimeter off its line hits the port's face; one pushed short leaves
  the meter off.
- **Two hands on one light object.** The meter weighs 36 g and the port's contacts push back on the strip: one hand
  holds the meter steady in the air while the other pushes the strip in, then the meter hand tips the meter so the
  strip's tip, 2.5 cm out of the port, goes down into a drop only 1.8 mm high.
- **Only the tip may touch.** The strip sips the sample through its tip; laying its face in the drop smears the
  sample (an invalid result), and touching the strip to the bottle's tip (putting control on it straight from the
  bottle) is what the maker forbids. The drop has to be squeezed out of a soft bottle with the tip down, onto the
  slide, not onto the strip or the meter.
- **Timing.** The tip has to stay in the drop until the strip has drawn it (1 s), and the strip has to stay in the
  port through the meter's 5 s countdown, or there is no result.

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy <policy> \
  -E task=tasks/run-glucose-meter-control-test/task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Run a control-solution test on the glucose meter with a new strip, then put the meter down, cap the control solution and lay it back in its rest." \
  --scorer task_score --max-steps 2400
```

## Scoring

- **Seated:** the strip's gray end within 1 mm of the port's back stop, the strip lined up with the port within
  6 degrees and right side up (the meter turns on; upside down it shows E 3, "Strip Upside Down").
- **A drop:** forms only with the cap off the nozzle (a capped tip lets nothing out), the bottle's tip pointing
  30 degrees or more below horizontal (the liquid reaches the tip only when it points down) and its walls squeezed
  with more than 8.3 N. It falls straight down as a 30 uL sphere (1.93 mm radius). It counts as put on the strip if it
  touches the strip or the meter on the way down, or lands on the slide with the strip already inside the bead it
  spreads into; otherwise it lands on the slide or the counter.
- **Sample drawn:** the strip seated, its tip held in a drop on the slide for 1 s without a break, the drop landed no
  more than 60 s before, and no part of the strip more than 3 mm from its tip has been in any drop on the slide
  (otherwise the strip is smeared and gives no valid result; once the meter shows its result it keeps it).
- **Result:** after the sample is drawn the meter counts down 5 s; the strip stays seated through it.

| Score | When |
|---|---|
| 0 | the meter or the control bottle was dropped (falling faster than 0.8 m/s, as after a free fall of about 3 cm); the strip came within 1 mm of the nozzle's end (its end face or the last 3 mm of its side) or touched a drop hanging from it; a drop was put on the strip or meter (as above); or none of the below |
| 0.25 | the strip is seated |
| 0.5 | the strip is seated and a drop is on the slide, with no result yet (not smeared, not pulled out during a countdown) |
| 0.75 | a valid result |
| 1.0 | a valid result, the meter resting on the counter (underside within 3 mm of it, tilted 10 degrees or less) with the strip in, the control bottle capped (cap's rim within 1.5 mm of the bottle's shoulder, its axis within 10 degrees of the bottle's) and lying in its rest (in the groove between the two slide stacks: at both ends of its body, from the base to where the shoulder starts, its axis is over the groove and within 2 mm of its resting height, the floor plus the bottle's 9.5 mm radius; on a stack it lies 4 mm higher; either way round), every hand (its gripper body and both fingers) at least 1 cm clear of the meter, strip, bottle and cap, all still, for 1 s |

The score is read at the end of the trial. Doing nothing scores 0. The meter can be held or set down anywhere and the
drop placed anywhere on the slide: the score is read from the strip's seat, the tip's contact with whichever drop it
touches, and the end state, not from a fixed path.

## Real-world build

### Real objects (full size)

| Object | Product | Size and mass |
|---|---|---|
| Meter | Ascensia Contour Next One blood glucose meter | 97 x 28 x 14.9 mm, 36 g (user guide) |
| Strips | Contour Next test strips, 50-count bottle | strip 33 x 7 x 0.4 mm, 0.2 g; bottle 32 mm across, 52 mm tall, flip-top lid |
| Control | Contour Next control solution, normal (Level 2), 2.5 mL dropper bottle | bottle 19 mm across, 53 mm long with its nozzle, 9 g; blue cap 13 mm across, 14 mm tall |
| Surface | one plain glass microscope slide (Fisherbrand 12-550-A3), a 12 mm strip of clear packing tape folded over each edge (about 5 mm on each face) | 75 x 25 x 1 mm |
| Rest | eight more of the same slides, in two stacks of four, each stack wrapped in clear packing tape and taped flat to the counter 20 mm apart | a groove 20 mm wide, 4 mm deep, 75 mm long |

The strip and bottle sizes above are the model's sizes; the setup checks each with the caliper (within 1 mm; the
strip's thickness within 0.1 mm).

## Real-world evaluation (reproducible protocol)

**Real trial at a glance**

| Item | Value |
|---|---|
| Meter | Ascensia Contour Next One |
| Strips | Contour Next test strips (50-count bottle), one new strip per trial |
| Control | Contour Next control solution, normal (Level 2), 2.5 mL dropper bottle |
| Drop surface | one plain glass microscope slide (Fisherbrand 12-550-A3), edges taped |
| Bottle rest | two stacks of four of the same slides, each wrapped in clear packing tape, taped 20 mm apart (a 20 x 4 mm groove) |
| Layout | same marks as the simulation (meter x 34, y 16; strip bottle x 20, y -20; control rest x 26, y -7; slide x 24, y 4; cap mark x 20, y 11, cm) |
| One-time setup | 20 minutes (tape marks, caliper and scale checks) plus the 20-minute room rest |
| Reset per trial | 3 minutes |
| Checked each trial | the meter's reading inside the strip bottle's printed control range; strip seated (meter on); drop on the slide, tip only; right side up (no E 3); no bottle-tip touch; nothing dropped; meter down with strip in, bottle capped and in its rest, hands clear, still 1 s |
| Consumables | each trial uses one strip, one drop and one pair of finger cots |

Room 20 to 24 degrees C (68 to 75 degrees F), inside the maker's control-testing range of 59 to 95 degrees F
(15 to 35 degrees C). Meter, strips and control solution rest in the room for 20 minutes before the trial (the
maker's wait after a temperature change). Strips and control solution within their dates; the control bottle opened
less than 6 months ago (the maker's limit).

**Products and tools:** the items in the table above; Kimwipes for the slide and the bottle's tip; ScotchBlue 2090
painter's tape; Scotch Heavy Duty Shipping Packaging Tape (clear) to cover every glass edge; disposable nitrile
finger cots (Medline nitrile finger cots, large) slipped over both grippers' finger pads, a fresh pair each trial, so
the shared grippers never touch the control solution (nitrile grips like the pads' rubber). Use a digital caliper
(e.g. Neiko 01407A) for the size check, a kitchen scale reading 1 g (e.g. Etekcity EK6015) for the force checks, a
tape measure for the marks, a room thermometer, and two phones on stands: one filming the port from
the robot's right at the meter's height, one filming the slide and the bottle rest from the side at counter height
(60 fps).

**Consumables:** each trial uses one strip, one drop of control solution and one pair of finger cots.

**Per-trial prep: 3 minutes** (discard the used strip, wipe the slide, pull up a new strip, shake the control
bottle, wipe its tip and cap it, lay it in its rest, fresh finger cots).

**Setup (marks and tolerances)**
1. Check a strip, the strip bottle and the control bottle against the sizes in the table with the caliper.
2. Force checks with the kitchen scale (once per new bottle of control solution and new meter):
   - Squeeze: lay the uncapped control bottle on the scale's platform with its base raised on the box of slides,
     so the nozzle points down past the platform's edge over a tissue; tare; press its upper wall at mid-body with
     a fingertip, slowly, until a drop forms, and read the grams. The simulation opens at
     8.3 N (846 g), the mean first-drop squeeze of 78 dropper bottles (range 4.2-15.7 N, 428-1,600 g); record the
     reading with the trial.
   - Strip contacts: stand the meter on its end, port up, on the platform, tare, and push a strip slowly fully in;
     record the peak reading with the trial.
3. Tape the meter's outline at x 34, y 16 cm (within 3 mm), port toward the robot. Insert no strip.
4. Flip the strip bottle open and stand it on its tape ring at x 20, y -20; pull one strip up by its tip end until it
   stands 15 mm above the rim (within 2 mm), gray end down, its gray-square face toward the robot's left.
5. Glass edges: the bottle hand's fingertips reach down beside the rest's stacks, so cover every glass edge. Wrap
   each stack of four slides along its length in clear packing tape (one layer, covering its top, both long edges
   and the underside) and fold a strip of the same tape over each of its ends. On the drop slide, fold a 12
   mm-wide strip cut from the tape over each edge (about 5 mm on each face); the drop goes on the bare glass
   inside it. Tape the two stacks for the rest (centered at x 26, y -7, long sides along x, 20 mm apart between
   the taped edges; check the groove with the caliper: 20 mm wide, 4 mm deep, within 0.5 mm). Shake the control
   bottle 15 times, take the cap off, wipe the tip with a Kimwipe, push the cap back on to the shoulder, and lay
   the bottle in the groove with its middle at x 24, tip toward the robot. Tape a mark at x 20, y 11 for the cap.
6. Wipe the slide clean and dry and tape its corners at x 24, y 4 (center), long side along x.

**Measurement (four readings)**
1. **The meter's result:** after the run, read the meter's display. The test passes when the reading lies inside the
   control range printed on the strip bottle's label (the maker's own criterion); an error code or no reading is no
   result. Record the reading and the printed range.
2. **Seat and drop:** from the port video, the meter turned on (port light, beep, no E 3) when the strip went in;
   from the slide video, a drop landed on the bare slide clear of the strip (not on or beside the strip, and not on
   the meter), the strip's tip went into the drop, and the strip's face stayed out of it.
3. **Touches:** from the videos, the strip never touched the control bottle's tip or a drop hanging from it, and
   neither the meter nor the bottle was dropped (let fall rather than set down).
4. **End state:** the meter rests on the counter with the strip in; the cap is pushed on to the shoulder (a gap of
   no more than 1.5 mm); the bottle lies in its rest, either way round: in the groove between the stacks, its whole
   body over the groove and on the counter (not on or across a stack; checked by eye at the end and on the slide
   video); both hands (grippers and fingers) at least 1 cm clear and everything still for 1 s (video).

Score with the table above, using reading 1 for "a valid result".

### Tolerance table

| Quantity | Value | Source |
|---|---|---|
| Meter size and mass | 97 x 28 x 14.9 mm, 36 g | Contour Next One user guide, Specifications |
| Countdown after the sample | 5 s | user guide, Specifications and control test step 7 |
| Control testing temperature | 59-95 F (15-35 C) | user guide, Control Solution |
| Pass criterion | reading inside the range printed on the strip bottle, foil packet or box | user guide, control test step 8 |
| Squeeze to start a drop | 8.3 N on the walls | mean first-drop force of 78 dropper bottles, 8.3 +- 3.0 N (range 4.2-15.7 N): "Wide Variation of Squeezing Force and Dispensing Time Interval among Eyedropper Bottles" (PMC6501172); checked on the scale |
| Strip contacts | one sprung roller in the port stands for all of the connector's contacts and holds the strip lightly | the real connector's push-back on the strip is read on the scale at setup (step 2) and recorded with the trials |
| Falling drop | a 30 uL sphere, 1.93 mm radius | geometry of the drop's volume |
| Bottle tip | the nozzle's end face and the last 3 mm of its side, 1 mm margin | protocol setting |
| Friction (each object's material; PhysX averages the two touching) | strip PET 0.40 / 0.30; cap PP and bottle LDPE 0.20 / 0.20; meter ABS 0.30 / 0.30; glass slide 0.9 / 0.4; taped rest 0.20 / 0.20; finger pads 1.0 / 0.8 | static / dynamic: untreated PET film, side A on side B (Transcendia GPXLH datasheet, ASTM D1894) for the strip; polyethylene on polyethylene 0.2 for the polyolefins (PP, LDPE, the tape's polypropylene film); polystyrene on steel 0.3-0.35 for the styrenic ABS; glass on glass 0.9-1.0 / 0.4; rubber on rubber 1.16 and on cardboard 0.5-0.8 for the rubber pad faces (Engineering ToolBox) |
| Strip, strip bottle, control bottle sizes | as in the table | protocol setting; checked with the caliper |
| Port depth and slot | 9 mm, 7.6 x 1.0 mm | protocol setting (the strip's width and thickness + 0.6 mm) |
| Drop | 30 uL, 3.1 mm footprint radius, 1.8 mm high | protocol setting (a 30 uL drop with a 60 degree contact angle on glass) |
| Sip time | 1 s | protocol setting (the maker says to hold the tip in the drop until the meter beeps) |
| Sample within | 60 s of the drop landing | protocol setting (the maker says "immediately") |
| Seat | gray end within 1 mm of the back stop, within 6 degrees | protocol setting (the meter turns on) |
| Capped | cap's rim within 1.5 mm of the shoulder | protocol setting (pushed on to the shoulder) |
| Rest groove | 20 mm wide, 4 mm deep | four 1 mm slides per stack, 20 mm apart |
| Positions | within 3 mm (meter, slide), 2 mm (strip height) | protocol setting |

## Design notes

- Start state: the control bottle starts capped and lying in its rest; the robot takes the cap off (one hand holds
  the bottle level, the other pulls the cap and stands it open side down on its mark), lays the bottle back, and caps
  it again at the end. Only the shake and wiping the tip before capping are done by the person setting up.
- The strip bottle is open with one strip pulled up 15 mm proud of the rest, gray end down: that first step (freeing
  one strip from a full bottle) is done by the person setting up. Taking a strip from a packed 32 mm vial needs a
  fingertip inside it, and tipping one out needs the bottle turned near mouth-down, further than the YAM's wrist
  turns a bottle it holds (a reach scan of the arm), and then leaves a 0.4 mm strip lying flat, which a parallel
  gripper cannot pick up. From the standing strip on, the robot does everything.
- The meter is picked up and held in the air: the strip goes in level while one hand holds the meter, and the meter
  is then tipped port-end down so the strip meets the drop tip first, as a person holding a meter does.
- The cap goes on two-handed: one hand holds the bottle level, the other pushes the cap onto the nozzle, lets go and
  presses its top home with closed fingertips.
- The scripted demo plays fixed waypoints from the fixed layout, closed loop on one thing: the strip's tip goes to
  where the drop landed, as a person watching does (left hand: the cap off, the meter, then the cap on; right hand:
  the bottle held for uncapping, the strip, then the bottle for the drop and held level while the cap goes on).

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes).
- Scripted demo: it scores 1.0 in 2,021 steps (about 101 s): cap off, strip seated 0.1 mm from the back stop, one
  30 uL drop squeezed onto the slide by the pinching hand, sample drawn by the tip alone, result after the 5 s
  countdown, meter set down, cap back on, bottle back in its rest, hands clear. Dream-1: 0 in 2,400 steps on the same
  task.py.
- No randomness: `-E object_jitter=0` keeps everything on its marks. Each trial is one run.

## Files

- `task.py`: scene, scorer and scripted demo.
- `yamkit.py`: the prop and mechanism helpers.
- `handkit.py`: the hands-clear check on the robot's own collision hulls (gripper and fingers). The robot model
  and its collision hulls load at run time from the competition environment (`yam_arena.analysis.link_hulls`) and
  are not redistributed in this folder.
- `dropkit.py`: the squeeze-bottle drop model (a drop forms above the opening force with the tip down and the cap
  off, and falls straight down).
- `gripkit.py`: binds rubber friction (1.0 / 0.8) to both arms' finger pads, as the real YAM's pads are rubber-faced.
- `test_task.py`: offline checks of the scorer, the seat, the drop zones, the dip geometry, the tiers and each cheat
  (a capped squeeze, touching the nozzle, a drop put beside the strip, the strip upside down, a hand on the meter).

## Fidelity

Rigid bodies throughout. The meter is a free 36 g body; its port is a 7.6 x 1.0 mm slot with rounded lips, and a
sprung roller inside it stands for the connector's contacts that press on a strip and hold it lightly in place (the
setup reads the real connector's push-back on the scale). The strip is a rigid 0.4 mm plate of 0.2 g with its gray
square on its top face. The control bottle is a rigid round body in contact; the force on its walls comes from a hand
that pinches its body between the fingers, from how far its fingers are commanded past the wall, through the finger
drive and the wall's give, up to the walls' stop; liquid flows with the squeeze above 8.3 N and the tip pointing
down, never while the cap covers the nozzle, and leaves as 30 uL drops that fall straight down as spheres of that
volume. The drop on the slide is a lumped spherical cap with that volume and a 60 degree contact angle, not a
simulated fluid; the strip's sip is that model's 1 s tip-in-drop rule. The meter's display and countdown are the
scorer's state. The cap is a close sliding fit on a cylindrical band at the nozzle's base, held on by friction. The
rest is the two slide stacks. The strip, meter, control bottle, cap, slide and rest each have their own material's
friction (tolerance table). The hands-clear check uses the robot's own collision hulls of each gripper and both of
its fingers.

## Sources

- Ascensia Diabetes Care, Contour Next One user guide (US, web90013719 rev 12-24, ascensiadiabetes.com): control
  solution testing steps 1-9, the caution against putting control solution on the strip straight from the bottle,
  when to run a control test, control testing temperature, specifications (size, mass, 5 s countdown), Second-Chance
  sampling, error codes.
- O*NET OnLine 29-2061.00 Licensed Practical and Licensed Vocational Nurses (employment 666,900, 2025) and 31-9092.00
  Medical Assistants (833,900, 2025), task statements quoted above.
- "Wide Variation of Squeezing Force and Dispensing Time Interval among Eyedropper Bottles"
  (PMC6501172): first-drop squeeze 8.3 +- 3.0 N, range 4.2-15.7 N, force gauge advanced until a drop fell.
- Engineering ToolBox, "Friction - Coefficients for Common Materials and Surfaces" (engineeringtoolbox.com): glass on
  glass 0.9-1.0 static, 0.4 dynamic; polyethylene on polyethylene 0.2; polystyrene on steel 0.3-0.35; rubber on
  rubber 1.16; rubber on cardboard 0.5-0.8.
- Transcendia, GPXLH polyester (PET) film technical data sheet (transcendia.com): COF film to film (A/B) static 0.4,
  kinetic 0.3, ASTM D1894 (an untreated polyester film): the strip.
- Adventist HealthCare, point-of-care policy POC 0002 (glucose meter QC: two levels every 24 hours, unit lock-out
  times): xconnect.adventisthealthcare.com/hmcpolicies/HRMC/Laboratory/Point of Care/POC 0002.pdf
- CentraCare, Glucose Testing - StatStrip Glucose Meter (QC every 24 hours; performed by patient care staff):
  centracare.com/documents/intranet/glucose-testing-statstrip-glucose-meter.pdf
- Virginia Department of Health, Assisted Blood Glucose Monitoring Policy and Procedure Template (testers include
  certified nursing assistants and medical assistants), vdh.virginia.gov, 2024.
