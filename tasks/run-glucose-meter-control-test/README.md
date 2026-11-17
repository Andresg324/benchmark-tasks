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
- **Same hands as patient testing:** the physical steps (seat a strip in the port, touch only its tip to a sample) are
  those of every fingerstick glucose test. Hospital meters (Accu-Chek Inform II, Nova StatStrip) are sold to
  hospitals, so the task uses a consumer meter of the kind clinics and physician offices also use; the strip-and-drop
  handling is the same.
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

- **Seated:** the strip's gray end within 1 mm of the port's back stop and the strip lined up with the port within
  6 degrees (the meter turns on).
- **A drop:** forms only with the bottle's tip pointing 30 degrees or more below horizontal and its walls squeezed with
  more than 8.3 N; it falls straight down and lands on the slide, the counter, or the strip or meter under it.
- **Sample drawn:** the strip seated, its tip held in a drop on the slide for 1 s without a break, the drop landed no
  more than 60 s before, and no part of the strip more than 3 mm from its tip has been in any drop on the slide
  (otherwise the strip is smeared and gives no valid result).
- **Result:** after the sample is drawn the meter counts down 5 s; the strip stays seated through it.

| Score | When |
|---|---|
| 0 | the meter or the control bottle was dropped (falling faster than 0.8 m/s); the strip came within 1 mm of the bottle's tip; a drop landed on the strip or meter; or none of the below |
| 0.25 | the strip is seated |
| 0.5 | the strip is seated and a drop is on the slide, with no result yet (not smeared, not pulled out during a countdown) |
| 0.75 | a valid result |
| 1.0 | a valid result, the meter resting on the counter (underside within 3 mm of it, tilted 10 degrees or less) with the strip in, the control bottle capped (cap's rim within 1.5 mm of the bottle's shoulder, its axis within 10 degrees of the bottle's) and lying in its rest (within 3 cm of where it started, level within 10 degrees, its axis within 4 mm of the groove's floor height), every hand at least 1 cm clear of the meter, strip, bottle and cap, all still, for 1 s |

The score is read at the end of the trial. Doing nothing scores 0. The meter can be held or set down anywhere and the
drop placed anywhere on the slide: the score is read from the strip's seat, the tip's contact with whichever drop it
touches, and the end state, not from a fixed path.

## Real-world build

### Real objects (full size)

| Object | Product | Size and mass |
|---|---|---|
| Meter | Ascensia Contour Next One blood glucose meter | 97 x 28 x 14.9 mm, 36 g (user guide) |
| Strips | Contour Next test strips, 50-count bottle | strip 33 x 7 x 0.4 mm, 0.2 g; bottle 32 mm across, 52 mm tall, flip-top lid |
| Control | Contour Next control solution, normal (Level 2), 2.5 mL dropper bottle | bottle 19 mm across, 53 mm long with its nozzle, 9 g; cap 13 mm across, 14 mm tall |
| Surface | one plain glass microscope slide (Fisherbrand 12-550-A3) | 75 x 25 x 1 mm |
| Rest | eight more of the same slides, in two stacks of four taped flat to the counter 20 mm apart | a groove 20 mm wide, 4 mm deep, 75 mm long |

The strip and bottle sizes above are the model's sizes; the setup checks each with the caliper (within 1 mm; the
strip's thickness within 0.1 mm).

## Real-world evaluation (reproducible protocol)

**Real trial at a glance**

| Item | Value |
|---|---|
| Meter | Ascensia Contour Next One |
| Strips | Contour Next test strips (50-count bottle), one new strip per trial |
| Control | Contour Next control solution, normal (Level 2), 2.5 mL dropper bottle |
| Drop surface | one plain glass microscope slide (Fisherbrand 12-550-A3) |
| Bottle rest | two stacks of four of the same slides taped 20 mm apart (a 20 x 4 mm groove) |
| Layout | same marks as the simulation (meter x 34, y 16; strip bottle x 20, y -20; control rest x 26, y -7; slide x 24, y 4; cap mark x 20, y 11, cm) |
| One-time setup | 20 minutes (tape marks, caliper and scale checks) plus the 20-minute room rest |
| Reset per trial | 3 minutes |
| Checked each trial | the meter's reading inside the strip bottle's printed control range; strip seated (meter on); drop on the slide, tip only; no bottle-tip touch or fall; meter down with strip in, bottle capped and in its rest, hands clear, still 1 s |
| Consumables | each trial uses one strip, one drop and one pair of finger cots |

Room 20 to 24 degrees C (68 to 75 degrees F), inside the maker's control-testing range of 59 to 95 degrees F
(15 to 35 degrees C). Meter, strips and control solution rest in the room for 20 minutes before the trial (the
maker's wait after a temperature change). Strips and control solution within their dates; the control bottle opened
less than 6 months ago (the maker's limit).

**Products and tools:** the items in the table above; Kimwipes for the slide and the bottle's tip; ScotchBlue 2090
painter's tape; disposable nitrile finger cots (Medline nitrile finger cots, large) slipped over both grippers' finger
pads, a fresh pair each trial, so the shared grippers never touch the control solution (nitrile grips like the
pads' rubber). Use a digital caliper (e.g. Neiko 01407A) for the size check, a kitchen scale reading 1 g (e.g.
Etekcity EK6015) for the force checks, a tape measure for the marks, a room thermometer, and two phones on stands
(the team's own): one filming the port from the robot's right at the meter's height, one filming the slide from
the side at counter height (60 fps).

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
     record the peak reading. The simulation's contact presses the strip with 0.3 N (31 g).
3. Tape the meter's outline at x 34, y 16 cm (within 3 mm), port toward the robot. Insert no strip.
4. Flip the strip bottle open and stand it on its tape ring at x 20, y -20; pull one strip up by its tip end until it
   stands 15 mm above the rim (within 2 mm), gray end down, faces toward the robot's left and right.
5. Tape the two slide stacks for the rest (centered at x 26, y -7, long sides along x, 20 mm apart). Shake the
   control bottle 15 times, take the cap off, wipe the tip with a Kimwipe, push the cap fully back on, and lay the
   bottle in the groove with its middle at x 24, tip toward the robot. Tape a mark at x 20, y 11 for the cap.
6. Wipe the slide clean and dry and tape its corners at x 24, y 4 (center), long side along x.

**Measurement (four readings)**
1. **The meter's result:** after the run, read the meter's display. The test passes when the reading lies inside the
   control range printed on the strip bottle's label (the maker's own criterion); an error code or no reading is no
   result. Record the reading and the printed range.
2. **Seat and drop:** from the port video, the meter turned on (port light, beep) when the strip went in; from the
   slide video, a drop landed on the slide (not on the strip or meter), the strip's tip went into the drop, and the
   strip's face stayed out of it.
3. **Touches:** from the videos, the strip never touched the control bottle's tip, and neither the meter nor the
   bottle fell.
4. **End state:** the meter rests on the counter with the strip in; the cap is pressed fully onto the bottle (no gap
   between the cap's rim and the shoulder); the bottle lies in its rest; both hands clear and everything still for
   1 s (video).

Score with the table above, using reading 1 for "a valid result".

### Tolerance table

| Quantity | Value | Source |
|---|---|---|
| Meter size and mass | 97 x 28 x 14.9 mm, 36 g | Contour Next One user guide, Specifications |
| Countdown after the sample | 5 s | user guide, Specifications and control test step 7 |
| Control testing temperature | 59-95 F (15-35 C) | user guide, Control Solution |
| Pass criterion | reading inside the range printed on the strip bottle, foil packet or box | user guide, control test step 8 |
| Squeeze to start a drop | 8.3 N on the walls | mean first-drop force of 78 dropper bottles, 8.3 +- 3.0 N (range 4.2-15.7 N): "Wide Variation of Squeezing Force and Dispensing Time Interval among Eyedropper Bottles" (PMC6501172); checked on the scale |
| Strip contact force | 0.3 N | the upper end of 0.1-0.3 N per contact in a strip-connector patent (Roche, US 8,398,443); checked on the scale |
| Strip, strip bottle, control bottle sizes | as in the table | protocol setting; checked with the caliper |
| Port depth and slot | 9 mm, 7.6 x 1.0 mm | protocol setting (the strip's width and thickness + 0.6 mm) |
| Drop | 30 uL, 3.1 mm footprint radius, 1.8 mm high | protocol setting (a 30 uL drop with a 60 degree contact angle on glass) |
| Sip time | 1 s | protocol setting (the maker says to hold the tip in the drop until the meter beeps) |
| Sample within | 60 s of the drop landing | protocol setting (the maker says "immediately") |
| Seat | gray end within 1 mm of the back stop, within 6 degrees | protocol setting (the meter turns on) |
| Capped | cap's rim within 1.5 mm of the shoulder | protocol setting (pressed fully on) |
| Rest groove | 20 mm wide, 4 mm deep | four 1 mm slides per stack, 20 mm apart |
| Positions | within 3 mm (meter, slide), 2 mm (strip height) | protocol setting |

## Design notes

- Start state: the control bottle starts capped and lying in its rest; the robot takes the cap off (one hand holds
  the bottle level, the other pulls the cap and stands it open side down on its mark), lays the bottle back, and caps
  it again at the end. Only the shake and wiping the tip before capping are done by the person setting up.
- The strip bottle is open with one strip pulled up 15 mm proud of the rest, gray end down: that
  first step (freeing one strip from a full bottle) is done by the person setting up. Taking a strip from a packed
  32 mm vial needs a fingertip inside it, and tipping one out needs the bottle near mouth-down (the YAM can tilt a
  bottle it picked up only 100 degrees) and then leaves a 0.4 mm strip lying flat, which a parallel gripper
  cannot pick up. From the standing strip on, the robot does everything.
- The meter is picked up and held in the air: the strip goes in level while one hand holds the meter, and the meter
  is then tipped port-end down so the strip meets the drop tip first, as a person holding a meter does.
- The cap goes on two-handed: one hand holds the bottle level, the other pushes the cap onto the nozzle, lets go and
  presses its top home with closed fingertips.
- The scripted demo is open loop (left hand: the cap off, the meter, then the cap on; right hand: the bottle held for
  uncapping, the strip, then the bottle for the drop and held level while the cap goes on).

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes).
- Scripted demo: the plan lasts 98.5 s; it scores 1.0 (success at step 1,930): cap off and stood open side down on
  its mark, bottle back in its rest, strip seated 0.1 mm from the back stop, one 30 uL drop on the slide 1.6 mm from
  the aim, sample drawn by the tip alone, result after the 5 s countdown, meter set down, cap seated 0.5 mm from the
  shoulder, bottle back in its rest, hands clear. The demo is open loop. Dream-1: 0 in 2,400 steps on the same task.py.
- No randomness: `-E object_jitter=0` keeps everything on its marks. Each trial is one run.

## Files

- `task.py`: scene, scorer and scripted demo.
- `yamkit.py`, `guardkit.py`: the shared prop/mechanism and hand-clearance helpers.
- `dropkit.py`: the squeeze-bottle drop model (sprung wall pads; a drop forms above the opening force with the tip
  down and falls straight down).
- `gripkit.py`: binds rubber friction (1.0 / 0.8) to both arms' finger pads, as the real YAM's pads are rubber-faced.
- `test_task.py`: offline checks of the scorer, the seat, the drop zones, the dip geometry and the tiers.

## Fidelity

Rigid bodies throughout. The meter is a free 36 g body; its port is a 7.6 x 1.0 mm slot with rounded lips, and a
sprung roller inside it (0.3 N preload) stands for the strip contacts that hold a strip in place. The strip is a rigid
0.4 mm plate of 0.2 g. The control bottle's squeeze is four sprung wall pads (3 N per mm); liquid flows with the
squeeze above 8.3 N and leaves as 30 uL drops that fall straight down. The drop on the slide is a lumped spherical
cap with that volume and a 60 degree contact angle, not a simulated fluid; the strip's sip is that model's 1 s
tip-in-drop rule. The meter's display and countdown are the scorer's state. The cap is a rigid cup that slides over
the nozzle's cone and is held on by friction (its snap is not modeled). The rest is the two slide stacks.

## Sources

- Ascensia Diabetes Care, Contour Next One user guide (US, web90013719 rev 12-24, ascensiadiabetes.com): control
  solution testing steps 1-9, the caution against putting control solution on the strip straight from the bottle,
  when to run a control test, control testing temperature, specifications (size, mass, 5 s countdown), Second-Chance
  sampling, error codes.
- O*NET OnLine 29-2061.00 Licensed Practical and Licensed Vocational Nurses (employment 666,900, 2025) and 31-9092.00
  Medical Assistants (833,900, 2025), task statements quoted above.
- "Wide Variation of Squeezing Force and Dispensing Time Interval among Eyedropper Bottles"
  (PMC6501172): first-drop squeeze 8.3 +- 3.0 N, range 4.2-15.7 N, force gauge advanced until a drop fell.
- Roche Diagnostics, US patent 8,398,443 "Biological testing system and connector therefor": contact normal force of 0.1 to 0.3 N per contact.
- Adventist HealthCare, point-of-care policy POC 0002 (glucose meter QC: two levels every 24 hours, unit lock-out
  times): xconnect.adventisthealthcare.com/hmcpolicies/HRMC/Laboratory/Point of Care/POC 0002.pdf
- CentraCare, Glucose Testing - StatStrip Glucose Meter (QC every 24 hours; performed by patient care staff):
  centracare.com/documents/intranet/glucose-testing-statstrip-glucose-meter.pdf
- Virginia Department of Health, Assisted Blood Glucose Monitoring Policy and Procedure Template (testers include
  certified nursing assistants and medical assistants), vdh.virginia.gov, 2024.
