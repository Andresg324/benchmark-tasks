# Thread a spin-on oil filter onto its engine nipple by hand, square, then tighten it to 18 N·m with a cap wrench

A new spin-on oil filter (Kawasaki 16097-0007, the Teryx and Teryx4 750 filter) stands on the bench beside the engine. Its mount is
where it is on the engine: the M20 x 1.5 nipple low on the front of the crankcase, its axis level and pointing at the
robot, with the frame's lower cross tube under it and the CVT intake duct beside it, as on the vehicle with its bottom
guard removed. A drip pan is under the engine. Beside the filter lies the tool Kawasaki's torque figure assumes: a
65 mm, 14-flute cap-type filter wrench on a 24-inch ratchet. "Fit the new oil filter." means the engine maker's
service procedure, which mechanics follow at every oil change: start the filter square on the thread by hand (a filter
started crooked cross-threads and ruins the nipple's thread), spin it on until its gasket contacts the mount, then
tighten it with the wrench to the specified torque, 18 N·m for this filter (about three quarters of a turn past
contact, the turn spin-on filter instructions commonly give), and put the wrench back. Stopping at contact (it feels
tight already) is the tempting wrong answer: the filter can work loose under engine vibration and leak. Cranking it well past a full turn
can crush the gasket, which can blow out under oil pressure.

## Spec

| Item | What it is |
|---|---|
| Scene | The lower front of the engine's crankcase (aluminum) with its machined filter boss and M20 x 1.5 nipple, the nipple's axis level, 13 cm above the bench, pointing at the robot; the frame's 32 mm cross tube 4.5 cm under the filter; the 60 mm CVT intake duct 3 cm to the filter's right; a drip pan under the engine. A new filter (black, 6.6 cm across, 7.1 cm tall, 250 g) stands on its gasket on the bench at the robot's left; the cap wrench on its ratchet lies flat on the bench near the robot, right of the nipple's line, cup down |
| Start | The filter loose, off the thread. The wrench off the filter |
| Instruction | "Fit the new oil filter." |
| Unstated rule | Start it square by hand, spin it to gasket contact, then tighten it to the engine maker's torque with the filter wrench (about three quarters of a turn more): not crooked, not left at contact, not cranked on; the wrench put back |
| Core mechanism | A free filter and the real M20 x 1.5 nipple: the filter's threaded bore goes over the nipple's end only centered within the thread's 0.2 mm clearance (colliders at the real diameters; before the thread catches nothing drives the filter), and the thread catches only if it is then pressed on the crests and turned clockwise with its axis within 2.6 degrees of the nipple's (more and it cross-threads); once caught, the thread is an axial drive at its 1.5 mm pitch; past gasket contact a resistance rising to 18 N·m at three quarters of a turn; a free cap wrench whose cup drives the filter only through its flutes (colliders), on a ratchet with a pawl |
| Category | Impact on jobs: automotive service technicians and mechanics (O*NET 49-3023.00; 825,800 US jobs in 2025, BLS Occupational Outlook Handbook) replace oil filters as regular work. O*NET lists the task "Perform routine and scheduled maintenance services, such as oil changes, lubrications, and tune-ups"; a new filter is normally fitted at each oil change |
| Trap | Starting it crooked (0, irreversible); stopping at contact or short of the 18 N·m (0, 0.25 or 0.5); over-tightening past a turn (0.5) or past 1.25 turns (0, irreversible) |
| Score | Degrees past contact at the end, in bands with reasons (below); outcome-based: the angle reads the torque (a real trial checks it with a click wrench), and the turning past contact must come from a holding hand's own wrist roll or from the wrench while its cap is on the filter and a hand holds it; then let go, and the wrench put back flat on its marks |

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `filterkit.py` | The loose filter and its thread, and the free wrench's ratchet. The filter is a free part on six undriven joints from its nipple (three slides, two tilts, its spin) whose ranges cover the whole bench and every way it can lie, and the thread's model (`FilterModel`) decides each control step whether it is loose, on the thread, or cross-threaded, and drives the thread when it is on: pitch, the run-on friction and the gasket's torque. The ratchet's pawl (`RatchetModel`) is a moving stop on the hinge between the ratchet and the cap. |
| `handkit.py` | Contact checks on the robot's own collision hulls, loaded at run time from the competition environment (`yam_arena.analysis.link_hulls`; nothing robot-derived is shipped): whether a gripper touches the filter or the wrench, or pinches one between its fingertips, within 1 mm. |
| `yamkit.py` | Spawns the modeled props and articulations (now with turned box colliders, for the flutes) and gives the scorer the grippers. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline checks of the thread model's rules, with the filter placed by hand (centered and 0.1 and 0.2 mm off the axis it goes on and catches; placed 0.5, 1, 2 and 6 mm off, resting on the nipple's end as the colliders leave it, it does not go on; in the sim the colliders do the blocking, and in the demo the base plate landed on the nipple's end and needed re-aiming; tilts either side of 2.6 degrees; turned while held off the crests; round stops, diagonal included; short of the nipple; unscrewed and started again), the free filter's joint ranges (lying every way across the bench), the pawl and the scorer, with near misses and cheats (pushed on without turning, slowly or in one step; turned the wrong way; over the nipple's end off its axis; tilted or offset just past the limit and turned back and forth; wobbled after the catch; the wrench seated before the thread starts; the wrench left resting on the filter, held off it, dropped by the filter, dropped upside down, or still sliding; the start state; doing nothing). No simulator needed; also runs under pytest. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/oil-filter-hand-tight-plus/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Fit the new oil filter." --scorer task_score --max-steps 4200

Model run (Dream-1): the same command with `--policy dream`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/oil-filter-hand-tight-plus/test_task.py` (prints `all pass`).

## The demo

The demo is closed loop: after every control step it reads the filter's pose and the thread's state, and the wrench's
pose, from the simulator, and works each move out from them when it starts.

By hand. The left arm picks the filter up off the bench by its closed end (top-down, the pads 2.5 cm in from the end,
closed to 8 mm under the can's width, about 20 N a finger, the fingers closing along x so that turning the filter over
cannot pivot it between the pads). It carries the filter out from the engine and across to the nipple and turns it to
point at it. It then reads where the center of the filter's threaded bore is and which way the filter points, and
corrects both, five times 8 mm out (in this pose the arm sags 2-3 cm under the filter, and the aim takes that up), then
again with the bore 1.5 mm short of the nipple's end (up to three looks, each held while the arm settles). It then
pushes the filter on. The bore clears the nipple's end by only 0.2 mm and the arm sways about a millimeter, so the
base plate usually lands on the nipple's end face first: pressing there, the hand reads where the bore is each second
and re-aims it a little, as a hand feels for the start, until the bore drops over the nipple's end (up to ten
seconds); that look also centers the bore on the nipple within its clearance before the turn. If it never does, the hand backs off, lines it up again and retries (twice at most). It spins the filter on
clockwise (seen from its end) in wrist rolls of up to 150 degrees, following the thread in, regripping between rolls.
Once a roll past gasket contact turns the filter less than 8 degrees (the pads slipping: hand-tight) it lets go and
backs off. Then, reading where the wrench lies, it takes the can again and turns the filter back up to a flute's
width (PENDING_BACK degrees in this run), so that the cap's teeth will meet the gaps between the filter's flutes at a handle
angle the right arm reaches. The right arm waits at home meanwhile.

With the wrench. Both hands take the wrench top-down where it lies flat: the left across the ratchet's head (on the
drive's axis), the right on the round grip 55 cm out. Each hand checks that it is holding, and if not opens and tries
once more 8 mm lower. They lift it 7 cm and turn it a quarter turn about the handle while bringing it across, so the
cup faces the engine. They hold it 3 cm in front of the filter's end with the handle at an angle the right arm can
reach that lines the cap's teeth up with the gaps between the filter's flutes (the left hand set the filter's angle
for this), read where the cap is and which way it points: twice they square it to the filter and center it, three
times more they center it, then twice again with its teeth 3.5 mm short of the flutes (the cup clears them by 0.4 mm),
and push the cup over the fluted end. The right hand moves to a top-down grip on the grip. Then strokes: the right
hand swings the handle down from 6 degrees above level to 10 below, pushing what the gasket takes (plus 1 N·m), aimed past the handle by its
arm's give (J K^-1 J^T from the servos' stiffness), while the left presses the cup on (8 N toward the engine); then
it swings the handle back up while the ratchet clicks. Each stroke is sized from the filter's angle when it starts,
and they stop once it is 270 degrees past contact (where the gasket takes 18 N·m), after two strokes that turn it
less than 2 degrees, or after 26 strokes. The right hand goes back to the grip it carried with; they pull the cup
straight off the flutes along the filter's axis, turn the wrench back cup down, carry it to its start marks, lay it
flat on the bench there, let go, lift clear and go home.

Result on this version: 1.0 in PENDING_STEPS steps (PENDING_S s): the thread caught PENDING_CATCH degrees off square;
hand-tight PENDING_HAND degrees past contact, turned back PENDING_BACK degrees for the cap; the wrench strokes took it
to PENDING_PAST degrees past contact (PENDING_NM N·m by the gasket's law).

## Scoring

Over the whole run (`success()` records it every step), the filter's angle is measured from gasket contact, positive
when tightened past it.

| Outcome | Score |
|---|---|
| Cross-threaded: the thread was started (the filter turned 20 degrees clockwise while pressed on the crests) with its axis more than 2.6 degrees off the nipple's. The real nipple's thread is damaged | 0, irreversible |
| Not on the thread at the end (left on the bench, dropped, pushed on without starting it, or unscrewed off and not started again) | 0 |
| Tightened more than 450 degrees (1.25 turns) past contact at any point, even if backed off: the gasket is over-compressed and can roll out (read from the peak angle; the real gasket is checked afterwards) | 0, irreversible |
| Ends under 90 degrees past contact, including at contact: it can work loose under vibration and leak | 0 |
| More than 45 degrees of the turning past contact done neither by a holding hand's own wrist roll nor by the wrench while its cap was on the filter and a hand held it (knocked, swiped) | 0 |
| 90 to 180 degrees past contact: sealing, but at most two thirds of the three-quarter turn | 0.25 |
| 180 to 270 (short of 18 N·m), or 360 to 450, or ever past 360 (past the most any instruction allows, not crushed) | 0.5 |
| 270 to 360 and never past 360, but a hand still touching the filter, or the wrench not yet put back: not finished | 0.5 |
| 270 to 360, never past 360, no hand touching the filter, and the wrench put back: lying flat on the bench, cup down (its back within 15 degrees of straight up), its cup's center within 10 cm of its start marks, at rest, no hand touching it. Kawasaki's 18 N·m is three quarters of a turn past contact on this model, and spin-on filter instructions commonly give three quarters to one turn | 1.0 |

Where the arms end is not scored. Possible scores: 0, 0.25, 0.5, 1.0.

Who turned it. Contact is real contact between the robot's own collision hulls (loaded at run time from the
competition environment) and the part, within 1 mm. A gripper holds the filter when its fingers are closed on the can
(its opening from 15 % of full travel tighter than the can's 6.6 cm to 3 % looser) and both fingertips touch it. Each step a gripper holding the filter at either end of the step adds its
own wrist roll about the filter's axis, clockwise seen from the filter's end, to its budget (kept at most 30 degrees
ahead of the filter), and new ground past contact is credited from that budget. While the wrench's cap is seated on
the filter (its axis within 3 mm and 5 degrees of the filter's, its floor within 4 mm of the filter's end) and a hand
holds the wrench (fingers under 60 % open, both fingertips touching its head, handle or grip), the new ground is the
wrench's, held. Whatever neither turned is unheld; more than 45 degrees unheld scores 0. Each hand keeps its own
budget, so a second hand near the can or on it never voids the holder's turning.

A hand is still on the filter at the end when any part of the gripper touches the can. Turning the filter by hand
past contact counts too: the score is outcome-based, read from the angle as a real trial reads the torque; in the demo
and on the rig the pads slip a few degrees past contact. Unscrewing the filter off the thread is a recoverable mistake:
it can be started again (the run's peak angle is kept). `success()` is a full score held for 1 s. Each scene reset
starts a new history.

Why these limits:
- Cross-threading: a filter started crooked cross-threads, jams and damages the nipple's thread. 2.6 degrees is where
  the filter's first thread, tilted across the 19.03 mm pitch diameter, lands half a pitch (0.75 mm) plus the flanks'
  0.12 mm axial play out on its far side, so that side drops into the next groove (geometry from the ISO 965 fit,
  below; the rig checks it, below).
- 270 to 360 degrees: 270 degrees past contact is Kawasaki's 18 N·m on the gasket's law, the same torque the real
  trial's click check reads; spin-on filter instructions commonly say three quarters of a turn after the gasket
  contacts the base, and some three quarters to one full turn, the band's two ends.
- 360 to 450 earns 0.5: past the most any maker allows but within a quarter turn of it. Over 450 is 0 and
  irreversible: the gasket is squeezed out of shape, and backing off does not restore it.
- 45 degrees unheld: the filter settling as a hand lets go stays well under it; a swipe or knock adds far more.

In a real trial: before the trial, spin the filter on to gasket contact, draw a witness line across the can and the
boss, and take the filter off; tape a printed protractor ring (15-degree marks) round the boss. Afterwards read the
line against the ring, counting whole turns from the video. Watch the video for a crooked start (the filter jams
within a turn, or the can sits visibly askew; check the nipple's thread with the filter off), for a swipe, and for
the hands letting go. Then put the build's 3/8-inch-drive click torque wrench, set to 18 N·m (13 ft·lb), in the cap's
square drive and turn it clockwise: it must click before the can moves; if the can moves first, the trial scores at
most 0.5. The sim's full-credit band starts at the same 18 N·m.

## Runtime and seeds

- Demo: 1.0 in PENDING_STEPS steps (PENDING_S s; the run ends 1 s after full credit is held).
- Model run limit: 4,200 steps (3.5 minutes at 20 Hz), up from 3,000. The demo needs about PENDING_STEPS steps
  (threading the filter on, spinning it on three turns, the wrench strokes and putting the wrench back).
- Dream-1: pending on this task.py.
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

The installed location. Kawasaki's parts diagram for the Teryx4 750 (KRT750, "Oil Pump/Oil Filter", E1710) draws
the spin-on filter (16097-0003, now 16097-0007) on a hollow nipple ("Pipe, oil filter" 32033-1535) on the crankcase,
its axis along the vehicle's FRONT arrow; an owner's guide puts it "on the bottom front section of the engine next to
the CVT intake", and the owner's manual (pp. 100-101) says the bottom guard has to be removed to reach it. So the sim
mounts the nipple on the crankcase's front face, level, with the frame's cross tube under the filter and the intake
duct beside it, the robot working from the front as from below the vehicle. The clearances round it are set for the
robot and are what the rig reproduces:

| Part | Sim | Real build tolerance | Source |
|---|---|---|---|
| Filter | 66 mm across, 71 mm tall, 250 g | as bought | KN-204/HF204 cross-reference listings (66 x 71 mm); the rig's filter is weighed at build |
| Thread | M20 x 1.5, right-hand | the real nipple | K&N KN-204 listing (M20 x 1.5) |
| Thread fit | nipple 6g: crests 19.97 mm, root 18.13 mm; filter 6H: crests 18.53 mm. The nipple's collider is its root cylinder; its end is flat at the root diameter (the small end of the ISO 4753 chamfer), and the crests' meeting point is a stop on the filter's axial slide. The filter's crests clear the nipple's end by 0.20 mm all round (0.12-0.27 mm over both tolerance bands); pitch diameters 0.21 mm apart | the real stud and filter | ISO 965-1/-2 (M20 x 1.5, 6H/6g, middle of each band); ISO 4753 (end chamfered to no more than the root diameter) |
| Turns from the first thread to gasket contact | 3 (4.5 mm) | counted on the rig; set the start mark from it | the rig's filter and stud |
| Nipple's axis | level, pointing at the robot, 13 cm above the bench | ±5 mm, ±2 degrees | parts diagram (direction); height set for the robot |
| Boss | 76 mm across, 8 mm proud of the case | ±2 mm | set for the robot |
| Frame cross tube | 32 mm steel, 45 mm under the can | ±5 mm | set for the robot |
| CVT intake duct | 60 mm, 30 mm from the can's side, on its right | ±5 mm | set for the robot |
| Gasket's torque | rises linearly past contact to 18 N·m at 270 degrees, 29 N·m at 450 | the click check after the trial | Kawasaki Teryx4 750 owner's manual p. 101 (18 N·m); the common three-quarter-turn instruction; the straight line is our model |
| Cross-thread angle | over 2.6 degrees | measure on the rig: tilt a real filter on the nipple until it no longer threads | derived: atan((0.75 + 0.12) / 19.03) from the fit above |
| Going on the nipple | the bore over the nipple's end only within its 0.20 mm clearance; no capture | the real stud and filter | the fit above |
| Crankcase front | 34 cm wide, 24 cm tall, its face 8 mm behind the gasket land | a plate or board at that face | set for the robot |
| Room | 20-24 degrees C | 20-24 degrees C | lab practice (gasket rubber and oil film) |

The filter is a free part. It sits on six joints with no drive from the nipple (three slides, two tilts and its spin)
whose ranges cover the whole bench the arms reach and every way it can lie (standing on either end, on its side in any
direction, lifted well clear), so it falls, tips, is picked up, carried and set down like a loose part; the joints are
its coordinates, not a constraint. Its colliders are hollow where the nipple goes (a ring for the gasket; the threaded
bore, 36 colliders at its thread's crests, 18.53 mm across; the base plate round it), and the nipple's collider is its
root cylinder, 18.13 mm across, standing 8.4 mm out of the land, its end flat at that diameter.

Before the thread catches, the filter is free and undriven: only colliders at the real diameters (the nipple's M20
root, the filter's 6H crests, 0.20 mm radial clearance, ISO 965) decide whether its bore goes over the nipple, so the
robot must center it to within about 0.2 mm and square it itself; nothing draws it onto the axis. Off its axis by
more than the clearance, the base plate lands on the nipple's end and stops there. With the nipple's end in the bore,
the threads' crests stop the filter 4.5 mm out (a stop on its axial joint). Whether the started thread catches or
cross-threads is judged once, when it has been turned 20 degrees clockwise while pressed on the crests, from its
measured tilt against the same ISO clearances: within 2.6 degrees it catches; over that, where the first thread lands
in the wrong groove, it cross-threads and jams. Past the catch the thread's run-down is an axial drive at the real
1.5 mm pitch, with round stops (no drive) at the engaged flanks' play, 0.10 mm and 0.36 degrees about the axis, or at
the offset and tilt it caught with if those are larger, so nothing pulls it in; nothing squares it but contact, the
gasket meeting the land. The flanks are not collision geometry. A filter that ends up over the nipple's end without
its thread having caught (only the solver's give could put it there) is treated as not on the thread: it cannot catch
until it is backed off. Unscrewed past where it caught, it is loose again and can be started again.

Turning resistance follows the gasket's law: before contact it runs on freely with light friction; past contact the
squeezed gasket holds it where it is left and gives way only at the gasket's torque, either way (a stiff stick-slip
hold capped at that torque, not a spring). A hard stop at 1.5 turns past contact is the gasket squeezed flat. In the
demo the hand, pinching at about 20 N a finger on the can's 33 mm radius, stopped turning it PENDING_HAND degrees past
contact; the rest needs the wrench, as on a real engine.

The wrench is a free body: the ratchet (handle and head) floats, and the drive (adapter and cap) turns on it about
the drive's axis. Nothing ties it to the filter. Its cup has 14 flutes inside (4 mm wide, 12 mm deep, reaching in to
31.5 mm) that sit between the filter's 14 flutes (5 mm wide on its last 14 mm, roots at 31 mm, crests at the can's
33 mm), with 0.4 mm of radial play at the wall and about 3 degrees of turning play. The cup's teeth drive the filter
by contact. The cup stays on only while it is pushed over the filter's end, and it tilts, slides off and falls if it
is not held there. The ratchet is a pawl: the hinge between the head and the cap has a moving stop. Turned to tighten,
the handle drives the cap against it; turned back, the cap stays and the stop follows it down in 3-degree clicks (the
ratchet's 120 positions), against a light drag. Dimensions (P: the maker's published figure; R: the rig's part,
measured when the rig is built):

| Part | Dimension | Sim | Source |
|---|---|---|---|
| Ratchet, GearWrench 81364XP | overall length | 610.6 mm (24.04 in) | P |
| | head width, thickness | 36.8 mm (1.449 in), 17.1 mm (0.674 in) | P (GearWrench brochure) |
| | weight | 1.361 kg (3.000 lb) | P (catalog) |
| | grip | 22 mm thick on the handle's last 15 cm | R |
| Adapter, GearWrench 81354 | length, weight | 35.1 mm (1.38 in), 27 g | P |
| Cap, Parts Unlimited 3801-0295 | across the flutes, flutes, drive | 65 mm, 14, 3/8 in | P |
| | wall, depth, end plate, weight | 3 mm, 13 mm, 4 mm, 60 g | R |
| All | mass | 1.448 kg | sum |

The arms' strength. The YAM's servos are Damiao DM-J4340 on joints 1-3 (9 N·m rated, 27 N·m peak) and DM-J4310 on
joints 4-6 (3 N·m rated, 7 N·m peak); the sim uses the peak limits (i2rt's yam_v1 config). The right hand swings the
handle alone, 0.55 m out: at 18 N·m (plus the demo's 1 N·m margin) it pushes about 35 N down the swing, while the left
hand, on the head, only presses the cup on (8 N). A Jacobian study of the same ratchet at this reach puts each
hand's push at 30-45 N at peak torque and well under that at rated torque, so the last strokes, a few seconds each,
run the right arm above its rated torque, within its peak. In the demo the strokes took the filter to PENDING_PAST
degrees past contact (PENDING_NM N·m by the gasket's law). Pushing down, the hand bears on its fingers, and the arm's
own weight helps.

No oil, gasket deformation or leak is simulated; the scorer reads the angle, and a real trial reads the same angle from
the witness line, then checks the torque directly.

## Real-world build

The rig is a mock engine front built at the sim's dimensions from plywood, a steel plate, conduit and PVC. It puts the
filter where it sits on the Teryx4 750 (on the crankcase's front face, low down, its axis along the vehicle's FRONT
arrow, per Kawasaki's parts diagram E1710) with the same access constraints round it. Positions are in cm from the
midpoint between the arm bases (x forward, y to the robot's left, bench top at z 0), as in the sim.

Parts:

- Engine face: 18 mm (3/4-inch) plywood, one 34 x 24 cm panel and one base board about 34 x 20 cm.
- Mounting plate: 6 mm steel plate, about 10 x 10 cm, with four M8 bolts, washers and nuts.
- Nipple: a Mishimoto MMSPA-M20 stainless steel adapter stud, M20 x 1.5 on both ends (the filter's thread: the K&N
  KN-204 listing gives M20 x 1.5, and K&N cross-references it to Kawasaki 16097-0007), with one M20 x 1.5 nut, one
  thin M20 x 1.5 jam nut (under 8 mm), two spare M20 x 1.5 nuts for the mount check, and blue thread-locking compound (Loctite 243).
- Gasket land: a 76 mm steel ring 8 mm thick (a 76 mm steel washer or a slice of 3-inch round bar, bored to clear
  the jam nut), with three wood screws or M4 screws.
- Frame cross tube: 32 cm of 1-inch rigid steel conduit (33.4 mm across, the sim's 32 mm within 2 mm), sprayed black,
  with two wooden blocks and two 1-inch pipe straps.
- Intake duct: 26 cm of 2-inch Schedule 40 PVC pipe (60.3 mm across, the sim's 60 mm), sprayed black, with one 2-inch
  pipe strap.
- Drip tray: a black plastic oil drip tray about 61 x 91 cm and 3.8 cm deep (Lisle 19722).
- Filter: a genuine Kawasaki 16097-0007 (kawasaki.com lists it for the 2012-2013 Teryx4 750 4x4, EPS and EPS LE).
- Wrench, as modeled: a 65 mm, 14-flute, 3/8-inch-drive cap-type filter wrench (Parts Unlimited 3801-0295; 65 mm and
  14 flutes is the size listed for the Hiflo HF204 / K&N KN-204 class, the cross-reference of this filter) on a
  GearWrench 81364XP 24-inch, 1/2-inch-drive ratchet through a GearWrench 81354 adapter (1/2-inch female to
  3/8-inch male). The sim models these parts' published sizes and mass, and the 24-inch handle is what lets one arm
  reach 18 N·m.
- Gripper covers: disposable nitrile gloves (GloveWorks HD GWON48100, Ammex, 8 mil, diamond-textured, size XL).
- Small items: wood screws, two C-clamps, black spray paint, a white paint pen, masking tape, a printed protractor
  ring (15-degree marks), motor oil for the gasket, shop towels, an oily-waste bag.

Tools: use a drill with an 8.5 mm bit and a step drill bit that reaches 21 mm, a 30 mm wrench and a 30 mm 3/8-inch-drive socket for the M20
nuts, a 3/8-inch-drive click torque wrench that covers 18 and 30 N·m (Pittsburgh Pro 63880, 5-80 ft·lb, Harbor
Freight), a steel rule, a tape measure and a spirit level.

Build steps:

1. Screw the plywood panel upright onto the base board. Set it on the bench with its front face at x 46.3, centered
   on y 5, and clamp the base board to the bench with the two C-clamps. Spray the panel's front black.
2. Mark the nipple's axis on the panel at y 5, z 13. Center the steel plate on that mark, drill four 8.5 mm holes
   through the plate and the panel at its corners, and bolt the plate on with the M8 bolts, washers and nuts.
3. Drill a 21 mm hole through the plate and the panel on the axis mark.
4. Run the M20 nut onto the stud's back end. Put the stud through the hole from the front, run the jam nut onto it
   against the plate's face, and set the nuts so the stud's thread stands 16.4 mm proud of the plate face, level and
   square to the plate (check with the spirit level and the steel rule). Add thread-locking compound and tighten both
   nuts with the 30 mm wrench.
5. Screw the 76 mm steel ring to the plate round the jam nut, centered on the stud. This is the gasket land (the sim's
   boss): the thread now stands 8.4 mm past it, and the filter's gasket seats 0.8 cm in front of the plate.
6. Check the mount once, before the first trial: jam two spare M20 x 1.5 nuts together on the stud's end, set the
   torque wrench to 30 N·m (22 ft·lb), and turn the outer nut clockwise with the 30 mm socket until the wrench
   clicks. The stud, its nuts and the plate stay put, which covers the 18 N·m of a good trial and the 29 N·m of an
   over-tightened filter. Take the spare nuts off.
7. Screw a wooden block to the base board at each end of the cross tube's span. Strap the black conduit across the
   blocks at x 41.8 from y -12 to 20, its top at z 5.2 (4.5 cm under the filter's can).
8. Stand the black PVC duct on the cross tube's level at x 42.8, y -4.3 (3 cm of air between it and the filter's can,
   on its right) and strap it to the panel.
9. Set the drip tray under the panel from x 40 to 101.
10. Spin the filter on to gasket contact by hand, draw a white index stripe across the can and the land and three
    black quarter-turn ticks on the can, tape the protractor ring round the land, and take the filter off.
11. Tape a mark on the bench for the filter at x 30, y 30, and a tape outline for the wrench lying flat, cup down,
    its cup centered at x 12, y -2, the handle toward the robot's right (to about y -63).
12. Measure every clearance in the table below with the steel rule and the spirit level.

The robot meets the same clearances in the rig as in the sim, each within the tolerance below (checked when the rig
is built and before each session):

| What the robot meets | Sim | Rig | Tolerance | Source |
|---|---|---|---|---|
| Nipple's axis | level, at x 45.5 (land), y 5, z 13, pointing at the robot | stud through the plate, set with a level | ±5 mm, ±2 degrees | parts diagram E1710 (direction); position set for the robot |
| Engine face behind the land | 8 mm | 8 mm steel ring | ±1 mm | set for the robot |
| Gasket land | 76 mm across | 76 mm steel ring | ±2 mm | set for the robot |
| Nipple past the land | 8.4 mm, M20 x 1.5 (6g) | MMSPA-M20 stud, 16.4 mm proud of the plate, set by the nuts | ±1 mm | thread: K&N KN-204 listing (M20 x 1.5) |
| Engine face | 34 x 24 cm | plywood panel | ±1 cm | set for the robot |
| Frame cross tube | 32 mm, top 45 mm under the can, x 41.8 | 1-inch rigid steel conduit, 33.4 mm | ±2 mm across, ±5 mm position | set for the robot |
| Intake duct | 60 mm, 30 mm right of the can, x 42.8 | 2-inch Sch 40 PVC, 60.3 mm | ±5 mm | set for the robot |
| Drip tray rim | 3.8 cm tall, from x 40 | Lisle 19722 | ±1 cm | Lisle 19722 listing |
| Filter's start | standing at x 30, y 30 | tape mark | ±5 mm | set for the robot |
| Wrench's start | flat, cup at x 12, y -2, handle to y -63 | tape outline | ±5 mm, ±2 degrees | set for the robot |
| Filter | 66 x 71 mm, 250 g | genuine 16097-0007 | as bought | KN-204/HF204 cross-reference listings |
| Gasket's torque | 18 N·m at 270 degrees past contact | the click check after each trial | the wrench clicks before the can moves | owner's manual p. 101 |

Torque spec source: Kawasaki Teryx4 750 owner's manual, torque table (p. 101): "Oil Filter 18 N·m (1.8 kgf·m,
13 ft·lb)", the gasket lightly oiled. Room 20-24 degrees C.

Before each session:

1. Fit glove covers on the gripper fingers: each finger goes into a glove's palm, the glove's fingers folded back along
   the outside of the robot's finger, the cuff taped above the pad. The textured cover gives the grip a rubber-like
   face, as the sim's contact.
2. Have the covered gripper lift the filter and spin it a quarter turn on the stud: it turns without a slip.
3. Check the clearances in the table.

After each trial, read the outcome (see Scoring), then put the 3/8-inch click torque wrench, set to 18 N·m
(13 ft·lb), in the cap's square drive and turn it clockwise: it must click before the can moves.

Reset per trial (about 2 minutes):

1. Unscrew the filter. Wipe the stud, the land and the gasket, and re-oil the gasket lightly.
2. Stand the filter on its gasket on its tape mark.
3. Lay the wrench flat on its tape outline, cup down.
4. Fit fresh glove covers; the used ones and the towel go in the oily-waste bag.
5. Replace the filter if it went past a full turn or cross-threaded, and chase the stud's thread with an M20 x 1.5 die
   if it cross-threaded.

Build time: about 2 hours.

## Sources and licenses

Sources: Kawasaki Teryx4 750 owner's manual, torque table p. 101 (oil filter 18 N·m) and pp. 100-101 (access);
Kawasaki parts diagram E1710 (KRT750, "Oil Pump/Oil Filter"); ISO 965-1/-2 (M20 x 1.5, 6H/6g) and ISO 4753 (thread
ends); K&N KN-204 listing (M20 x 1.5, cross-reference to 16097-0007); GearWrench 81364XP and 81354 listings and
brochure; Parts Unlimited 3801-0295 listing; Pittsburgh Pro 63880 listing; BLS Occupational Outlook Handbook and O*NET
49-3023.00 (the category); Damiao DM-J4340 / DM-J4310 and i2rt's yam_v1 config (the arms).

All code here was written for this task; no third-party code is included. No third-party assets are used: every shape
is generated by `yamkit.py`, `filterkit.py` and `task.py`. The robot model and its collision hulls load at run time
from the competition environment (arena revision 0059d02a) and are not redistributed.
