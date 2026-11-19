# Open a latched vaccine fridge against its magnetic catch, shelve three cartons, and shut it to the catch

A full door cycle on a compact vaccine refrigerator (an Accucold ARS1PV Pharma-Vac, 1 cu ft, three wire shelves): its
self-closing door is shut on a magnetic catch. The robot pulls the latched door open against the catch, gets it to
stay open, puts the three vaccine cartons from a delivery tote on the shelves, each at least 5 cm (2 in) from the
unit's walls, floor, door and ceiling as CDC storage guidance says, then pushes the door shut until the closer pulls it
onto the catch. The door must end shut: a door left open caps the score at 0.5.

## Why it matters

Drug and vaccine deliveries are put away by pharmacy technicians (471,200 US jobs in 2025, BLS Occupational Outlook
Handbook; O*NET 29-2052.00, Core task: "Maintain proper storage and security conditions for drugs.") and pharmacists
(325,200 jobs; O*NET 29-1051.00, Core task: "Order and purchase pharmaceutical supplies, medical supplies, or drugs,
maintaining stock and storing and handling it properly."); at clinics, medical assistants (833,900 jobs; O*NET
31-9092.00) "inventory, order, and restock medical and office supplies" (BLS Occupational Outlook Handbook).
CDC's Vaccine Storage and Handling Toolkit: "Vaccines must always be immediately checked and appropriately stored upon
arrival", and "Position vaccines and diluents 2 to 3 inches from the unit walls, ceiling, floor, and door." The toolkit also notes that a temperature monitoring device placed near the unit's walls, floor, vent, ceiling or door "may indicate
temperatures that are colder or warmer than the actual vaccine temperature", and the ARS1PV has an open-door alarm
(specification sheet).

## The scene

Positions in cm from the midpoint between the arm bases (x forward, y to the robot's left), heights above the counter
(the station table).

- **Fridge:** Accucold ARS1PV at full size, fixed to the counter. Cabinet 55 cm high, 44 cm wide, 41 cm deep without its
  door; its front face at x 50 (its back flush with the counter's far edge), its middle at y -4. Interior 40 cm high, 36 cm wide, 20 cm deep; the interior floor
  5.7 cm above the counter. Three wire shelves across the full width, 18 cm deep, their wire tops 5.7,
  18.1 and 31.75 cm above the interior floor (the lower two where the maker's dimensioned front view draws them, the
  top one in its 1/2 in slot at 12.5 in, one slot above the drawing); each is a front and
  a back rail with round 4 mm wires running front to back about 2.4 cm apart. The top and both sides are closed: the only way in
  is through the 36 x 40 cm front opening, between the shelves.
- **Door:** solid, 9.2 cm thick, hinged on the right, SHUT at the start (opened, its outer face stands 0.64 cm outside
  the cabinet's side, as the published 46 cm width with the door open gives). The unit ships with a self-closing door
  (owner's manual): in the sim, once the door is within 30 degrees of closed it swings shut by itself and is held shut
  (the mock's hinges list a 35 degree closing angle); wider open it stays where it is left. The unit's door is held
  shut by its magnetic gasket; the mock and the sim hold it with a rated magnetic catch instead (Everbilt 4.4 lb),
  recessed in the left side wall's front edge, its counter plate on the door's inner face 2 cm in from the free edge.
  It takes about 21 N pulled at the handle to open the shut door (the catch's 4.4 lb plus the closer). The door weighs
  4 kg in the model (the mock door is weighted to match). The keyed lock in the control panel
  is left unlocked, as in daytime use.
- **Handle:** like the ARS1PV's in the maker's photo, a bar down the door's free (left) edge on a post at each end:
  here the mock's Amerock BP54008SS bar pull at its listed sizes, a 3/4 in (1.9 cm) round bar 14 5/8 in (37.1 cm)
  long, its posts 12 in (30.5 cm) apart, standing 2 in (5.1 cm) proud of the door, a 3.2 cm finger gap behind the bar
  (the ARS1PV's own handle stands 1.75 in proud: 21.5 in deep with the handle, 19.75 in without).
- **Tote:** a Sterilite 6 Qt storage box without its lid (inside 29.2 x 15.2 cm at the bottom, 10.8 cm deep; 34.3 x
  20.3 cm outside at the rim), its middle at x 33, y 50, long side across the robot.
- **Cartons:** three Gardasil 9 cartons of ten single-dose vials, 8.6 x 3.95 x 4.45 cm, 60 g each in the
  model, lying flat in the tote in a row, long side toward the fridge, at y 42, 50 and 58 (8 cm apart, middle to middle).

The top shelf sits 8.25 cm under the ceiling: any carton on it (4.45 cm lying flat, 8.6 cm standing) reaches into the
5 cm ceiling margin, so the cartons go on the bottom and middle shelves, which are 12 cm apart.

## Run a trial

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/open-latched-vaccine-fridge-shelve-cartons/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Open the fridge, put the three vaccine cartons from the tote away in it, each on a shelf at least 5 cm from the walls, floor, door and ceiling, then shut the fridge door." \
  --scorer task_score --max-steps 3600 --store-frames --log-dir logs/vaccine-demo
```

Model run (Dream-1): the same command with `--policy dream --max-steps 3600`. Video:
`inspect-robots video logs/vaccine-demo/*.json`. The scorer (`task_score`) reports the score each step; the trial's
score is the one at its end. `VACCINE_TRACE=<file>` writes one line per control step (door angle, the widest it has
been open, each carton's shelf,
whether it counts, its six gaps in mm, dropped cartons). The run log says "[vaccine] hands-off check: 16 link poses from
the sim, hull check on" once the arms' collision hulls are read. Offline checks: `python3 test_task.py` in this folder.

## Runtime and seeds

- Run limit: 3,600 control steps at 20 Hz (3 minutes) per trial.
- Scripted demo: the plan lasts about 135 s and is open loop; its score is in task.json (scores).
- No randomness: `-E object_jitter=0` keeps every object on its mark.

## Scoring

Read at the end of the trial, in the fridge's own frame (in from the door plane, across, up from the interior floor).

A carton **counts** when all of these hold:
- it rests square on a shelf: one of its axes within 10 degrees of vertical, its lowest corner within 4 mm above that
  shelf's wire tops (not more than 2 mm below them), its middle over the shelf, and nothing of it outside the interior;
- every corner is at least 5 cm from the door plane, the back wall, both side walls, the ceiling and the interior floor;
- it has been let go: no part of either arm within 5 mm of it (a release check); and it is still (no corner moving more than 0.5 mm in a control step);
- it was never dropped: a carton whose middle comes to rest more than 1 cm lower than where a hand last carried it
  (it fell or slid out of the fingers), or one whose corner moves faster than 0.4 m/s with no hand on it (thrown or knocked
  over), counts for nothing for the rest of the trial. A carton set down by hand somewhere else first and later
  carried to its place by hand still counts.

The score is 0.25 for each carton that counts (0.25, 0.5, 0.75), and 1.0 when all three count and the door is shut
(within 0.5 degrees of closed: its free edge within 4 mm of the cabinet front) and both arms off it (no part of either arm within 5 mm
of the door or its handle: a release check), all held for 1 s, which ends the trial. While the door is not shut the score is at most
0.5. Doing nothing scores 0.

A carton stacked on another, propped on a rail, leaning on a wall, lying on the interior floor or under a shelf, or
on the top shelf does not count.

## Real-world build (a low-cost setup)

A full-size mock of the ARS1PV with its real interior, as only the reach-in matters:
- **Cabinet:** 1/2 in plywood, 55 cm high, 44 cm wide, 41 cm deep outside, painted white, with the interior
  40 x 36 x 20 cm: each side wall a 4 cm thick panel (two plywood skins on 1.5 cm strips), the top a closed plywood
  box 9.3 cm tall, the floor on a 5.7 cm plinth, and a plywood block filling the 21 cm behind the back wall. Screw it to
  the counter with its back at the counter's far edge.
- **Shelves:** three pieces cut with a bolt cutter from a ClosetMaid 4 ft x 12 in ventilated wire shelf
  (vinyl-coated steel, shelf kit 104100) to 36 x 18 cm, wires running front to back, each resting on
  two wood cleats screwed to the side walls so the wire tops stand 5.7, 18.1 and 31.75 cm above the interior floor.
- **Door:** a 44 x 48 cm panel of 3/4 in plywood as its outer face, built up behind to 9.2 cm thick with foam board,
  weighted to 4 kg with a steel plate screwed inside. It hangs on a pair of Hettich full-overlay hinges, part number
  9073626 ("a wide automatic closing angle of 35°", for doors 15-24 mm thick), their cups drilled into the inner face
  of the plywood panel. Their mounting plates go on a 0.75 x 3 in (1.9 x 7.6 cm) hardwood stile screwed to the front of the cabinet's
  right side wall, flush with its outside and standing 7.3 cm proud of the cabinet front (the foam is notched round
  it), so the plywood panel closes onto the stile's front edge. The hinges then turn the door about a line at the back
  of the plywood panel, about 2 cm behind the door's outer face where the sim's hinge axis is; opened, the mock's door
  stands about 2 cm further right than the sim's.
- **Handle:** an Amerock 12 in center-to-center stainless bar appliance pull (BP54008SS) screwed straight onto the
  door's left edge (or any 12 in center-to-center appliance pull with a 3/4 in round bar standing 2 in proud).
- **Catch:** an Everbilt 4.4 lb magnetic catch, white (9235997), recessed at mid-height into the left side wall's
  front edge (in the 1.5 cm cavity between its skins: the catch body stands upright in the cavity, its screw tabs on a filler
  block), its face flush with the cabinet front; its counter plate on the
  door's inner face 2 cm in from the free edge, so the shut door closes onto it. Nothing stands in the opening.
- **Tote:** a Sterilite 6 Qt storage box, held to the counter at its mark with double-sided tape, a paper template on its floor outlining the three cartons.
- **Cartons:** three, plus two spares: tuck-end boxes 8.6 x 3.95 x 4.45 cm folded from white folding boxboard about
  0.4 mm thick (a cereal-box panel will do), each with a folded boxboard insert that holds ten 2 mL screw-cap glass
  vials (12 x 32 mm) upright in two rows of five, each with 0.5 mL of water, and fills the carton so its 3.95 cm sides
  do not give when gripped. Steel washers taped to the insert's floor bring each to 60 g (within 2 g) on the kitchen
  scale. Marked "training carton"; a carton with a crushed corner, a dented face or a cracked vial is replaced.
- Common tools: tape measure, two 30 cm steel rulers (one taped inside the fridge), square, spirit level, kitchen scale, digital luggage scale with peak hold,
  drill, 4 mm drill bit (the door-gap gauge), saw, bolt cutter, 35 mm hinge-cup bit, double-sided tape, a phone on a stand.

## Real-world evaluation

Marks (once): the midpoint between the arm bases is taped on the counter; from it, the fridge's front face 50 cm
forward and its middle 4 cm to the right (it is screwed down there), and the tote's middle 33 cm forward and 50 cm to
the left. The tote is held to the counter with double-sided tape (it is fixed in the sim). A paper template on the
tote's floor outlines the three cartons, long side toward the fridge, their middles 8 cm apart. A second 30 cm steel ruler is taped upright and flat to the inside of the right side wall,
1 cm behind the opening, its zero on the interior floor, so the video reads the shelves' heights and any drop against
it (it stands about 1 mm off the wall, well inside the 5 cm margin the cartons keep).

Build check (after building, and at the start of each session; a steel ruler and a spirit level):
- interior 40 x 36 x 20 cm and the shelves' wire tops 5.7, 18.1 and 31.75 cm above the interior floor, each within 3 mm;
- the cabinet level both ways (spirit level on its top);
- the door weighed at 4.0 kg (within 0.1 kg) before it is hung; shut on its catch, its free edge within 2 mm of the
  cabinet front and its top edge at the same height above the counter at both ends, within 2 mm (no sag); hinge
  screws tight.

Before each trial (about 1 minute; the trial does not start until every check passes):
1. Catch: a digital luggage scale with peak hold, hooked on the middle of the handle's bar and pulled straight out,
   square to the door, slowly (about 2 s), reads 17 to 25 N (3.8 to 5.6 lb) as the door pulls open. If it reads more,
   take the counter plate off, measure the hinges' pull alone, and shim the plate back from the catch (a layer of
   tape) until the total is in the band; if less, re-seat the plate, and replace the catch if it still reads low.
2. Closer: released 25 degrees open (a cardboard angle template on the counter at the hinge), the door swings shut
   onto its catch; left 45 degrees open, it stays within 2 degrees for 10 s.
3. Cartons: each weighs 60 g (within 2 g), is square, and has no crushed corner or dented face (over 1 mm against
   the steel ruler); a spare replaces any that fails. Each lies flat on its outline in the tote.
4. Door shut on its catch (free edge within 4 mm of the cabinet front); both arms at their home pose.

Trial: run the policy for up to 3 minutes. Video: a phone at 30 fps or more on a stand about 60 cm in front of the
fridge's left front corner, at the middle shelf's height, seeing the whole opening, both shelves' fronts and the
door's free edge, plus the robot's cameras. The operator holds the stop and stops the arms if one jams in the
opening; a stopped trial is scored as it stands when stopped.

Scoring, in this order:
1. Door, before anyone touches it: a 4 mm drill bit as a gauge at the free edge (if it does not enter, the door is
   shut); from the video, no part of either arm touches the door or its handle for the last second.
2. From the video, against the ruler taped inside the fridge: whether a carton fell more than 1 cm out of the
   fingers, or fell or tipped with no hand on it, and that the hands left it.
3. Then open the door to 90 degrees and measure each carton with the steel ruler: from its nearest corner to the door
   plane (a straightedge across the front frame), the back wall, each side wall, the ceiling and the interior floor;
   check it stands on the wires of a shelf and is square (a small square against it: 10 degrees is about 1.5 cm over
   8.6 cm).

Reset (about 3 minutes with the checks): cartons back on their outlines in the tote, door shut. Nothing is consumed.

## Fidelity

Rigid bodies at full size. The cabinet, shelves, door and tote are their real shapes; the cartons are boxes of the real
size, 60 g each (the mock cartons are weighed to match). The self-closing hinge is a spring on the hinge joint that acts within 30 degrees of shut (inside the 35
degrees the mock's hinges list) and keeps pressing the door shut once closed; wider open, the damped hinge leaves the
door where it is. The magnetic catch acts only against the robot: an effort toward shut equal to the
catch's rated 4.4 lb pull at its counter plate while the door is shut, fading to nothing as the gap at the catch opens
by a few millimeters; the door opens only once the pull on it passes that. The handle is a round bar (a 12-sided hull) on two posts: the fingers hold it by real contact. Carton contacts are paperboard on plastic and coated wire
(friction 0.5 static, 0.4 dynamic).

## Design notes

- Contact is real throughout: cartons are free bodies on the wire colliders, the door moves only when something pushes
  or pulls on it; nothing snaps, captures or pulls a carton or the door into place. While the door is shut, its slab
  covers the whole front opening, so nothing gets in until the door has been opened.
- The hands-off check uses each arm's collision hulls (every link from the shoulder to the fingertips), read from the competition
  environment at run time; a missing robot or link stops the run with an error instead of falling back.
- Scripted demo: the left hand, level, its open fingers pointing along the door toward the hinge, slides them either
  side of the handle's bar from the free-edge side (the inner finger into the gap behind the bar) and closes; it
  pulls, the inner finger bearing on the back of the bar, the catch lets go, and the hand draws the door open along
  its arc to 40 degrees (wider open than the closer's reach, the door stays), then opens and slides its fingers back
  off the bar. It reaches over the door's top to its inner face and pushes it open to about 70 degrees with the edge of its
  fingertips, out of the way of the arm's path into the fridge. Then it takes each carton from above by its 3.95 cm sides, lifts it out, turns fingers-forward
  beside the fridge (the carton now stands on end), slides it in 8 mm above the wires to 6 cm past the door plane,
  sets it down and backs out; two go on the bottom shelf and one on the middle shelf. The right hand then pushes the
  door round its arc with the tips of its closed fingers from 70 to 10 degrees open and backs off; the self-closing
  hinge shuts it.

## Files

- `task.py`: scene, scoring and the scripted demo. `test_task.py`: offline checks (cheats, every side of the
  placement rule, dropped cartons, the door, a 1,500-pose sweep against an independent rule).
- `yamkit.py`, `guardkit.py`, `yamgeom.py`: scene building, hand checks and run-time arm hulls.

## Sources

- Accucold ARS1PV specification sheet (Summit Appliance; its product photo shows the bar handle down the door's left
  edge and the lock in the control panel): cabinet 21.5 x 17.5 x 19.75 in, width with door open
  18.0 in, interior 15.75 x 14.0 x 8.0 in, 3 adjustable wire shelves, RHD reversible door, front-view drawing.
  https://www.laboratory-equipment.com/media/asset-library/a/r/ars1pv-accucold-compact-vaccine-refrigerator-brochure.pdf
- Accucold owner's manual covering ARS1PV: "your unit has shipped with a right hand self-closing door swing"; its
  parts list names a "Keyed lock":
  https://assets.ajmadison.com/ajmadison/itemdocs/ARS6PV_MANUAL.pdf
- CDC Vaccine Storage and Handling Toolkit: https://www.cdc.gov/vaccines/hcp/downloads/storage-handling-toolkit.pdf
- WHO prequalification, Gardasil 9: carton of 10 vials 8.6 x 3.95 x 4.45 cm.
  https://extranet.who.int/prequal/vaccines/p/gardasil-9
- Gardasil 9 prescribing information (Merck): carton of ten 0.5 mL single-dose vials; "Store refrigerated at 2 to 8°C".
  https://www.merck.com/product/usa/pi_circulars/g/gardasil_9/gardasil_9_pi.pdf
- BLS Occupational Outlook Handbook: https://www.bls.gov/ooh/healthcare/medical-assistants.htm,
  https://www.bls.gov/ooh/healthcare/pharmacy-technicians.htm
- O*NET: https://www.onetonline.org/link/details/31-9092.00, https://www.onetonline.org/link/details/29-2052.00
- Hettich full-overlay hinge, part number 9073626 ("a wide automatic closing angle of 35°", door thickness 15-24 mm):
  https://www.wurthwoodgroup.com/hettich-sensys-soft-close-silent-system-full-overlay-face-frame-frameless-hinges-nickel-110deg-toolless-9073626-9073626
- Amerock BP54008SS bar pull (12 in center to center, 14 5/8 in long, 2 in projection, 3/4 in bar):
  https://www.cabinetparts.com/p/amerock-decorative-knobs-and-pulls-AM54008SS-p1796
- Everbilt 4.4 lbs. Magnetic Catch, White (1-Pack), model 9235997, Home Depot internet #303007626:
  https://www.homedepot.com/p/Everbilt-4-4-lbs-Magnetic-Catch-White-1-Pack-9235997/303007626
- Sterilite 6 Qt storage box: https://www.sterilite.com/product/set-of-10-6-qt-storage-boxes/

## Sources and licenses

No third-party data files are included. The robot model and its collision hulls load at run time from the competition
environment (arena revision 0059d02a) and are not redistributed.
