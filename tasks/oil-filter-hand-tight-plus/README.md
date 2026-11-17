# Thread a spin-on oil filter onto its engine nipple by hand, square, then tighten it to 18 N·m with a cap wrench

A new spin-on oil filter (Kawasaki 16097-0007, the Teryx and Teryx4 750 filter) stands in the drip pan beside the engine. Its mount is
where it is on the engine: the M20 x 1.5 nipple low on the front of the crankcase, its axis level and pointing at the
robot, with the frame's lower cross tube under it and the CVT intake duct beside it, as on the vehicle with its bottom
guard removed. A drip pan is under the engine. Beside the filter lies the tool Kawasaki's torque figure assumes: a
65 mm, 14-flute cap-type filter wrench on a 24-inch ratchet. "Fit the new oil filter." means the engine maker's
service procedure, which mechanics follow at every oil change: start the filter square on the thread by hand (a filter
started crooked cross-threads and ruins the nipple's thread), spin it on until its gasket contacts the mount, then
tighten it with the wrench to the specified torque, 18 N·m for this filter (about three quarters of a turn past
contact, the turn filter makers print), and take the wrench off. Stopping at contact (it feels tight already) is the
tempting wrong answer: the filter can work loose under engine vibration and leak. Cranking it well past a full turn
can crush the gasket, which can blow out under oil pressure.

## Spec

| Item | What it is |
|---|---|
| Scene | The lower front of the engine's crankcase (aluminum) with its machined filter boss and M20 x 1.5 nipple, the nipple's axis level, 12 cm above the bench, pointing at the robot; the frame's 32 mm cross tube 4.5 cm under the filter; the 60 mm CVT intake duct 3 cm to the filter's right; a drip pan under the engine. A new filter (black, 6.6 cm across, 7.1 cm tall, 250 g) stands on its gasket in the drip pan, left of the crankcase; the cap wrench on its ratchet lies flat on the bench, cup down |
| Start | The filter loose, off the thread. The wrench off the filter |
| Instruction | "Fit the new oil filter." |
| Unstated rule | Start it square by hand, spin it to gasket contact, then tighten it to the engine maker's torque with the filter wrench (about three quarters of a turn more): not crooked, not left at contact, not cranked on; the wrench taken off |
| Core mechanism | A free filter that the thread takes only if it is pressed onto the nipple with its axis within 4 degrees of the nipple's and turned clockwise (more and it cross-threads); past gasket contact a resistance rising to 18 N·m at three quarters of a turn; a free cap wrench whose cup drives the filter only through its flutes (colliders), on a ratchet with a pawl |
| Category | Impact on jobs: automotive service technicians and mechanics (O*NET 49-3023.00; 825,800 US jobs in 2025, BLS Occupational Outlook Handbook) replace oil filters as regular work. O*NET lists the task "Perform routine and scheduled maintenance services, such as oil changes, lubrications, and tune-ups"; a new filter is normally fitted at each oil change |
| Trap | Starting it crooked (0, irreversible); stopping at contact or a little past it (0 or 0.25); over-tightening past a turn (0.5) or past 1.25 turns (0, irreversible) |
| Score | Degrees past contact at the end, in bands with reasons (below); the turning past contact must come from a holding hand's own wrist roll or from the wrench while its cap is on the filter and a hand holds it; let go, the wrench off the filter and out of the hands |

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `filterkit.py` | The loose filter and its thread, and the free wrench's ratchet. The filter is a free part on six undriven joints from its nipple (three slides, two tilts, its spin), and the thread's model (`FilterModel`) decides each control step whether it is loose, on the thread, or cross-threaded, and drives the thread when it is on: pitch, the run-on friction and the gasket's torque. The ratchet's pawl (`RatchetModel`) is a moving stop on the hinge between the ratchet and the cap. |
| `guardkit.py` | Contact checks: the grippers' own shapes (boxes fitted to the YAM's collision meshes) and the 2 mm contact test that decides whether a hand is still on the filter. |
| `yamkit.py` | Spawns the modeled props and articulations (now with turned box colliders, for the flutes) and gives the scorer the grippers. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline checks of the thread's model (square, crooked, off the axis, short of the nipple, unscrewed), the pawl and the scorer, with near misses and cheats. No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/oil-filter-hand-tight-plus/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Fit the new oil filter." --scorer task_score --max-steps 2400

Model run (Dream-1): the same command with `--policy dream`.

Render the videos of a run with `inspect-robots video <log.json>`.

Scorer tests: `python tasks/oil-filter-hand-tight-plus/test_task.py` (prints `all pass`).

## The demo

The demo is closed loop: after every control step it reads the filter's pose and the thread's state, and the wrench's
pose, from the simulator, and works each move out from them when it starts.

By hand. The left arm picks the filter up off the bench by its closed end (top-down, the pads 2.5 cm in from the end,
closed to 8 mm under the can's width, about 20 N a finger), turns it to point at the nipple, and holds it 1 cm short.
It then reads where the gasket face's center is and which way the filter points, and corrects both, three times. It
pushes the filter onto the nipple and spins it on clockwise (seen from its end) in 150-degree wrist rolls, following
the thread in as it goes, regripping between rolls. Once a roll past gasket contact turns the filter less than 8
degrees (the pads slipping: hand-tight) it lets go and backs off. The right arm waits at home meanwhile.

With the wrench. Both hands take the wrench's handle top-down where it lies flat (the left 13 cm from the drive, the
right 55 cm, on the grip), lift it 10 cm and turn it a quarter turn about the handle, so the cup faces the engine,
while carrying it across to the filter. They hold it 3 cm in front of the filter's end, with the handle at the angle
that lines the cap's flutes up with the filter's (within the right arm's reach), read where the cap is and correct,
twice, and push the cup over the fluted end. The flutes' ends are rounded to a point, so a cup that meets them end to
end turns into the gaps. The right hand moves to a top-down grip on the handle. Then strokes: the handle swung down
from 7 degrees above level to 10 below, the right hand pushing 85 % of what the gasket takes (plus 1 N·m) and the
left pressing the cup on (8 N toward the engine), each hand aimed past the handle by its arm's give (J K^-1 J^T from
the servos' stiffness); then the handle swung back up while the ratchet clicks. Each stroke is sized from the
filter's angle when it starts, and they stop once it is 270 degrees past contact (where the gasket takes 18 N·m), or
after two strokes that turn it less than 2 degrees. They pull the cup off, set it on the bench in front of the
engine, the left hand lets go, the right lowers the handle to the bench and lets go, and both go home.

Result on this version: DEMO_RESULT.

## Scoring

Over the whole run (`success()` records it every step), the filter's angle is measured from gasket contact, positive
when tightened past it.

| Outcome | Score |
|---|---|
| Cross-threaded: the thread was started (the filter turned 20 degrees clockwise while pressed on the nipple) with its axis more than 4 degrees off the nipple's. The real nipple's thread is damaged | 0, irreversible |
| Unscrewed off the thread after it was started, even if started again | 0, irreversible |
| Not on the thread at the end (left on the bench, dropped, or pushed on without starting it) | 0 |
| Tightened more than 450 degrees (1.25 turns) past contact at any point, even if backed off: the gasket is over-compressed and can roll out (the angle as proxy; the real gasket is checked afterwards) | 0, irreversible |
| Ends under 90 degrees past contact, including at contact: it can work loose under vibration and leak | 0 |
| More than 45 degrees of the turning past contact done neither by a holding hand's own wrist roll nor by the wrench while its cap was on the filter and a hand held it (knocked, swiped) | 0 |
| 90 to 180 degrees past contact: sealing, but at most two thirds of the makers' figure | 0.25 |
| 180 to 240 (short of the figure), or 360 to 450, or ever past 360 (past the most any maker allows, not crushed) | 0.5 |
| 240 to 360 and never past 360, but a hand still on the filter, or the wrench's cap still on it or a hand still holding the wrench: not finished | 0.5 |
| 240 to 360, never past 360, let go, the wrench off the filter and out of the hands: Kawasaki's 18 N·m is three quarters of a turn past contact on this model, and the makers print three quarters to one turn | 1.0 |

Where the arms end is not scored. Possible scores: 0, 0.25, 0.5, 1.0.

Who turned it. A gripper holds the filter when its fingers are closed on the can (its opening from 15 % of full travel
tighter than the can's 6.6 cm to 3 % looser) with its tool point within 1.5 cm of the can's axis, anywhere from 1 cm
past its gasket face to 1.5 cm past its end. Each step a gripper holding the filter at either end of the step adds its
own wrist roll about the filter's axis, clockwise seen from the filter's end, to its budget (kept at most 30 degrees
ahead of the filter), and new ground past contact is credited from that budget. While the wrench's cap is seated on
the filter (its axis within 3 mm and 5 degrees of the filter's, its floor within 4 mm of the filter's end) and a hand
holds the wrench (fingers under 60 % open, the tool point within 3 cm of the handle's axis), the new ground is the
wrench's, held. Whatever neither turned is unheld; more than 45 degrees unheld scores 0. Each hand keeps its own
budget, so a second hand near the can or on it never voids the holder's turning.

A hand is still on the filter at the end when it holds the can or its palm or fingers touch it (the gripper's own
shape within 2 mm of the can). `success()` is a full score held for 1 s. Each scene reset starts a new history.

Why these limits:
- Cross-threading: MAHLE's bulletin on spin-on filter installation warns to screw it on "with care to avoid cross-threading" because "cross-threading mis-aligns the
  filter causing improper rubber gasket contact". 4 degrees is about where an M20 x 1.5 thread's first turn can still
  draw the filter square (an estimate; the rig measures the angle at which a real filter cross-threads, below).
- 240 to 360 degrees: filter makers print "after the gasket contacts the base, tighten three quarters of a turn", and
  several print "three quarters to one full turn". Full credit spans both readings, less 30 degrees for judging three
  quarters of a turn against the can's quarter-turn ticks.
- 360 to 450 earns 0.5: past the most any maker allows but within a quarter turn of it. Over 450 is 0 and
  irreversible: the gasket is squeezed out of shape, and backing off does not restore it.
- 45 degrees unheld: the filter settling as a hand lets go stays well under it; a swipe or knock adds far more.

In a real trial: before the trial, spin the filter on to gasket contact, draw a witness line across the can and the
boss, and take the filter off; tape a printed protractor ring (15-degree marks) round the boss. Afterwards read the
line against the ring, counting whole turns from the video. Watch the video for a crooked start (the filter jams
within a turn, or the can sits visibly askew; check the nipple's thread with the filter off), for a swipe, and for
the hands letting go. Then put a low-range 3/8-inch-drive click torque wrench set to 18 N·m (13 ft·lb) on the cap
and turn it clockwise: it must click before the can moves; if the can moves first, the trial scores at most 0.5.

## Runtime and seeds

- Demo: DEMO_RESULT.
- Model run limit: 2,400 steps (2 minutes at 20 Hz).
- Dream-1: DREAM_RESULT.
- Seed 0, one fixed layout (`object_jitter=0`).

## Fidelity

The installed location. Kawasaki's parts diagram for the Teryx4 750 (KRT750, "Oil Pump/Oil Filter", E1710) draws
the spin-on filter (16097-0003, now 16097-0007) on a hollow nipple ("Pipe, oil filter" 32033-1535) on the crankcase,
its axis along the vehicle's FRONT arrow; an owner's guide puts it "on the bottom front section of the engine next to
the CVT intake", and the owner's manual (pp. 100-101) says the bottom guard has to be removed to reach it. So the sim
mounts the nipple on the crankcase's front face, level, with the frame's cross tube under the filter and the intake
duct beside it, the robot working from the front as from below the vehicle. The clearances are estimates (no
published drawing gives them) and are what the rig reproduces:

| Part | Sim | Real build tolerance | Source |
|---|---|---|---|
| Filter | 66 mm across, 71 mm tall, 250 g | as bought | KN-204/HF204 cross-reference listings (66 x 71 mm); mass estimated |
| Thread | M20 x 1.5, right-hand | the real nipple | K&N KN-204 listing (M20 x 1.5) |
| Turns from the first thread to gasket contact | 3 (4.5 mm) | measure; set the start mark from it | estimated |
| Nipple's axis | level, pointing at the robot, 12 cm above the bench | ±5 mm, ±2 degrees | parts diagram (direction); height set for the robot |
| Boss | 76 mm across, 8 mm proud of the case | ±2 mm | estimated |
| Frame cross tube | 32 mm steel, 45 mm under the can | ±5 mm | estimated (forum reports; no drawing) |
| CVT intake duct | 60 mm, 30 mm from the can's side, on its right | ±5 mm | estimated |
| Gasket's torque | rises linearly past contact to 18 N·m at 270 degrees, 29 N·m at 450 | the click check after the trial | Kawasaki owner's manual p. 101 (18 N·m); makers' three quarters of a turn; the straight line is our model |
| Run-on before contact | 0.3 N·m | n/a | estimated |
| Cross-thread angle | over 4 degrees | measure on the rig: tilt a real filter on the nipple until it no longer threads | estimated |
| Room | 20-24 degrees C | 20-24 degrees C | lab practice (gasket rubber and oil film) |

The filter is a free part. It sits on six joints with no drive from the nipple (three slides, two tilts and its spin)
that cover everything within reach, so it falls, tips, is picked up, carried and set down like a loose part; the joints
are its coordinates, not a constraint. Its colliders are hollow where the nipple goes (a ring for the gasket, a ring
for the base plate with the threaded hole), and the nipple is a collider, so the filter goes on only centered on it.
Pressed onto the nipple, the threads' crests stop it 4.5 mm out (a stop on its axial joint). Turned 20 degrees
clockwise there, the thread starts: with its axis within 4 degrees of the nipple's, it catches, and from then on the
filter turns only about the nipple's axis and travels 1.5 mm a turn; tilted more, it cross-threads and jams. Before
gasket contact it runs on with 0.3 N·m; past contact the squeezed gasket holds it where it is left and gives way only
at the gasket's torque, either way (a stiff hold, 2,000 N·m/rad, whose drive is capped at that torque: stick-slip
friction, not a spring). A hard stop at 1.5 turns past contact is the gasket squeezed flat. In the demo the hand,
pinching at about 20 N a finger on the 33 mm radius, slips at about 1.5 N·m, about 22 degrees past contact; the rest
needs the wrench, as on a real engine.

The wrench is a free body: the ratchet (handle and head) floats, and the drive (adapter and cap) turns on it about
the drive's axis. Nothing ties it to the filter. Its cup has 14 flutes inside (4 mm wide, 12 mm deep, reaching in to
31.5 mm) that sit between the filter's 14 flutes (5 mm wide on its last 14 mm, roots at 31 mm, crests at the can's
33 mm), with 0.4 mm of radial play at the wall and about 3 degrees of turning play. The cup's teeth drive the filter
by contact. The cup stays on only while it is pushed over the filter's end, and it tilts, slides off and falls if it
is not held there. The ratchet is a pawl: the hinge between the head and the cap has a moving stop. Turned to tighten,
the handle drives the cap against it; turned back, the cap stays and the stop follows it down in 3-degree clicks (the
ratchet's 120 teeth), against 0.2 N·m of drag. Dimensions (P: the maker's published figure; E: estimated, measured
when the rig is built):

| Part | Dimension | Sim | Source |
|---|---|---|---|
| Ratchet, GearWrench 81364XP | overall length | 610.6 mm (24.04 in) | P |
| | head width, thickness | 36.8 mm (1.45 in), 17.5 mm (0.69 in) | P |
| | weight | 1.361 kg (3.000 lb) | P (catalog) |
| | grip | 22 mm thick on the handle's last 15 cm | E |
| Adapter, GearWrench 81354 | length, weight | 35.1 mm (1.38 in), 27 g | P |
| Cap, Parts Unlimited 3801-0295 | across the flutes, flutes, drive | 65 mm, 14, 3/8 in | P |
| | wall, depth, end plate, weight | 3 mm, 13 mm, 4 mm, 60 g | E |
| All | mass | 1.448 kg | sum |

The arms' strength. The YAM's servos are Damiao DM-J4340 on joints 1-3 (9 N·m rated, 27 N·m peak) and DM-J4310 on
joints 4-6 (3 N·m rated, 7 N·m peak); the sim uses the peak limits (i2rt's yam_v1 config). At 18 N·m the right hand,
0.55 m out, pushes about 28 N down the swing and the left hand, 0.13 m out, about 21 N, plus its 8 N press. That needs
the arms above their rated torque on the last strokes, a few seconds each, within their peak; we state it rather than
claim otherwise (the earlier revision's Jacobian study of the same ratchet gave 10.7 N·m rated and 23.8 N·m peak for
two hands). Pushing down, each hand bears on its fingers' faces, and the arm's own weight helps.

No oil, gasket deformation or leak is simulated; the scorer reads the angle, and a real trial reads the same angle from
the witness line, then checks the torque directly.

## Real-world build

A whole engine is not needed: the rig reproduces the installed location with a remote filter mount.

- Mount: a universal remote oil filter mount with an M20 x 1.5 nipple (Mishimoto MMOC-RFH-M20BK, M20 x 1.5, billet aluminum, $139.95 at onefastshop.com; only the head is used, not Mishimoto's engine take-off plate), bolted to a steel plate standing
  upright on the bench (an angle bracket and two clamps), so the filter's axis is level, pointing at the robot, 12 cm
  above the bench. A 32 mm steel tube clamped across under it (45 mm under the can) stands in for the frame's cross
  tube, and a 60 mm length of flexible duct stood 30 mm to the filter's right for the CVT intake (the clearance table
  above gives the tolerances). Its two hose ports are closed with their plugs.
- Filter: Kawasaki 16097-0007 (MSRP $15.86 at kawasaki.com, which sells it in tens; listed for the 2012-2013 Teryx4 750 4x4, EPS and EPS LE), its gasket lightly oiled, a paint-pen witness line, three black
  quarter-turn ticks. It stands on its gasket in the drip pan at the start, left of the mount.
- Wrench: a 65 mm, 14-flute cap-type filter wrench (Parts Unlimited 3801-0295, 65 mm, 14 flutes, 3/8-inch drive;
  65 mm / 14 flutes is the size listed for the Hiflo HF204 / K&N KN-204 class; check its fit on the filter's flutes
  when the rig is built) on a GearWrench 81364XP ratchet (1/2-inch drive, 24.04 in) through a GearWrench 81354
  adapter (1/2-inch female to 3/8-inch male). It lies flat on the bench, cup down, on its tape outline.
- Torque check after each trial: a low-range 3/8-inch-drive click torque wrench, 40-250 in·lb (Neiko NEI03708B, $69.90 at arizonatools.com; 156 in·lb is in the middle of its range), set to 18 N·m (13 ft·lb), in the cap's square drive.
- Protecting the shared grippers from gasket oil: each gripper finger wears a disposable nitrile glove as a cover
  (GloveWorks HD GWON48100, Ammex, 8 mil, diamond-textured, size XL; $28.53 a box of 100 at Fisheries Supply): the finger goes into the glove's
  palm, the glove's fingers folded back along the outside of the robot's finger, the cuff taped above the pad. Fresh
  covers each trial; used ones and the towel go in an oily-waste bag. The textured cover keeps a rubber-like face on
  the grip, as the sim's contact (static friction 1.2 on objects, 1.0 on the pads). Before each session the covered
  gripper lifts the filter and spins it a quarter turn on the nipple without a slip. The mount stands in a Lisle 19722
  drip pan (24 x 36 x 1.5 in; about $34 at Speedway), as in the sim, which catches the drop of oil from the gasket and the
  nipple.
- Room 20-24 degrees C.
- Torque spec source: Kawasaki Teryx4 750 owner's manual, torque table (p. 101): "Oil Filter 18 N·m (1.8 kgf·m,
  13 ft·lb)", the gasket lightly oiled.

### Budget build

| Item | Budget choice | Price | Why it still works |
|---|---|---|---|
| Filter | Kawasaki 16097-0007 at street price, or a single aftermarket 16097-0007 | about $13-17 (Walmart and FridayParts listings) | The same part number and size; the kawasaki.com MSRP ($15.86) comes in tens |
| Mount | Mishimoto MMOC-RFH-M20BK head alone, no take-off plate or hoses | $139.95 | The only remote head we could verify with an M20 x 1.5 filter thread; cheap relocation kits we found take a 3/4-16 filter (Derale 15718, $81.54) or are sandwich plates, which do not fit this filter |
| Ratchet | Keep the GearWrench 81364XP | about $110-125 | The sim models its published length, head and mass; a cheaper 24-inch ratchet would need the sim re-set and the demo re-run |
| Cap wrench, adapter | Parts Unlimited 3801-0295, GearWrench 81354 | about $12, $8 | As named |
| Torque check | Neiko NEI03708B, 40-250 in·lb | $69.90 | 13 ft·lb (156 in·lb) sits mid-range, where a click wrench is most accurate; a 10-80 ft·lb wrench would be near the bottom of its range |
| Gloves, pan | GloveWorks HD GWON48100; Lisle 19722 | about $19-29 a box; about $31-41 | Shared across tasks; any 8 mil textured nitrile glove in XL serves |
| Stand-ins | 32 mm steel tube, 60 mm flexible duct, steel plate and angle bracket, two clamps, paint pen | about $40 | Only their positions matter (the clearance table) |

Rough rig cost: about $430-460 in all (about $270 if the lab already has the ratchet and a low-range torque wrench).

## Licenses

All code here was written for this task; no third-party code is included. No third-party assets are used: every shape
is generated by `yamkit.py`, `filterkit.py` and `task.py`.
