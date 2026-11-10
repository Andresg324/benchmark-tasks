# Make a moka pot ready to brew: water just below the safety valve, top screwed on snug

A 3-cup aluminum stovetop moka pot stands taken apart on a gas hob: its empty water chamber (the base, with the
brass safety valve on its side toward the robot) is already on the cold burner grate, its upper chamber stands on the
counter to the left and a glass carafe holding 175 mL of water stands to the right. "Get the moka pot ready to brew."
means doing what every home cook and barista who uses one knows: fill the base with water up to just below the safety
valve, never over it, then screw the upper chamber on until it is snug. Emptying the whole carafe in (19 mL over the
valve) and setting the top on without screwing it tight are the tempting wrong answers.

The real task also drops the filled funnel basket into the base between the two steps. That step is cut here (a pour,
a place and a 1.7-turn regrasp twist already make a 90 s demo); the basket is left out of the scene and out of the
real build.

## How it differs from nearby tasks

- Board #665, tamp coffee and lock the portafilter: an espresso machine's portafilter is held level, tamped and locked
  in with a quarter turn. Here nothing is tamped (a moka pot's basket is never tamped), there is no machine, and the
  core is a fill level capped by a safety valve plus a multi-turn screw joint whose tightness is scored.
- Our kettle-fill-between-lines: also a pour stopped inside a window, but there the window is two printed lines and
  the closing act is a hinged lid and a switch. Here the upper limit is a safety part (the valve's bore, nothing
  printed), and the pot has to be closed by screwing a free upper chamber onto the base with regrasp strokes, scored
  for being on straight and turned into a stated snug band.

## Spec

| Item | What it is |
|---|---|
| Instruction | "Get the moka pot ready to brew." (the rule is never stated) |
| Job | Baristas and cooks, and anyone making coffee at home on a stovetop moka pot |
| Rule | Water up to just below the safety valve (not over it, not more than 1 cm under it), then the upper chamber screwed on snug and let go; the carafe set down off the stove |
| Why | Over the valve, the valve stands in water and can no longer vent steam if the funnel clogs (the pot can over-pressure); far under it, too little water for the grounds makes weak, under-extracted coffee; a loose or crooked top leaks steam and hot coffee at the gasket |
| Traps | Emptying the carafe (175 mL, the valve is at 155.8 mL); setting the top on without screwing it; turning it one turn and stopping |
| Water | `pourkit.py` flow model (copied from the kettle task, with its `accepts` hook for a covered mouth) |
| Screwing | Scored from the upper chamber's turning on the base, credited with twistkit's signed wrist-roll budget (see Scoring) |

### Scene (cm from the midpoint between the two arm bases, x forward, y to the robot's left, table top z = 0)

| Object | Type and size | Material | Position |
|---|---|---|---|
| Hob plate | 14 x 14 x 0.4 cm with a black burner cap 5.6 cm across; fixed | stainless steel | centered (27, 5) |
| Burner grate | four diagonal fingers (1.2 cm wide, 1.4 cm tall, from 2.2 to 6.6 cm off the axis) and a ring 13.2 cm across; fixed | cast iron | on the plate |
| Moka pot base | 3-cup water chamber, 7.2 cm tall, 8.2 cm across at the foot tapering to a 6.6 cm threaded collar (top 1.2 cm); inside it steps from 7.2 to 6.6 to 6.2 cm across (floor 4 mm up, steps at 2.4 and 4.4 cm); fixed on the grate (a real cook steadies it with the other hand) | cast aluminum | standing on the grate, axis at (27, 5), foot at z 1.8 |
| Safety valve | brass hex 1.3 cm across with a pin, its center 5.0 cm above the foot; bore 8 mm across, bottom edge 4.6 cm above the foot (155.8 mL) | brass | on the base's side toward the robot (-x) |
| Upper chamber | 8.7 cm tall, 7.0 cm across on its lower body, 8.0 cm at the top, with a lid knob, a small spout and a black handle 45 degrees off the x axis; a skirt 1.2 cm deep (female thread) under it, 8.5 cm across, with eight round ribs inside 2 mm clear of the collar, so once down over the collar it stays centered on it; 200 g; free | aluminum, black plastic handle | standing on the counter at (24, 19) |
| Carafe | cylinder 4.9 cm across, 13.5 cm tall, 4.4 cm inside, 200 g | glass | standing at (23, -12) |
| Water | 175 mL in the carafe, none in the base | | |

Assembled the pot is 15.9 cm tall, as a real 3-cup pot.

## Scoring

The scorer watches the whole run (`success()` records it every step, after `pourkit` has moved the water).

| Outcome at the end | Score |
|---|---|
| More than 155.8 mL in the base (above the bottom of the valve bore) | 0 |
| Under 30 mL in the base (nothing to brew; includes doing nothing) | 0 |
| 30 to 122.5 mL and the upper chamber not screwed snug either (neither half done) | 0 |
| 30 to 122.5 mL (more than 1 cm under the valve: weak coffee), top screwed snug | 0.25 |
| 122.5 to 155.8 mL, upper chamber not screwed snug: not on, crooked, or screwed under 450 or over 720 degrees | 0.25 |
| Filled and screwed, but over 5 mL spilled, the carafe not set down upright, let go and off the stove, the base knocked over or off the grate, or a gripper still holding the upper chamber | 0.5 |
| All of it, both arms home and still | 1.0 (0.9 with an arm not home) |

Possible scores: 0, 0.25, 0.5, 0.9, 1.

Definitions and why each number:

- Valve line, 155.8 mL: the bore's bottom edge, 4.6 cm above the foot (4.2 cm of water). Water above it stands in the
  valve. Low line, 122.5 mL: 1 cm lower, a fifth less water than the dose; the 3-cup basket's 15 g of coffee then
  brews strong and under-extracted, the usual "weak coffee" complaint. The band is 33 mL wide (8 mm of level).
- On the base: the upper chamber's bottom face within 4 mm of the rim height, its axis within 4 mm of the base's (the
  skirt's 1.5 mm clearance plus the thread's play) and within 4 degrees of it (more is cross-threaded).
- Screwed: once the upper chamber has sat on the base, and while its skirt stays round the collar (bottom face under
  1.2 cm above the rim, axis within 6 mm and 8 degrees), its turning relative to the base counts. Clockwise turning
  (seen from above) is credited only while a gripper's fingers are closed on it (opening between 0.45 and its 7 cm
  width + 3 %: an empty closed hand reads 0, an open one 1; tool point within 1.5 cm of its axis) and only up to 1.25
  times that gripper's own wrist roll in the same direction (twistkit's signed budget; up to 15 degrees of unused
  clockwise roll carries over, counterclockwise roll is kept as debt). Another arm swiping the top round, a hand
  resting on it, or a closed hand wiggling back and forth earns nothing. Counterclockwise turning counts against it
  whoever does it, and lifting it off the collar restarts the count.
- Snug band, 450 to 720 degrees: a 3-cup pot's thread catches and runs about 1.25 turns before the gasket meets the
  filter plate, and a firm hand squeezes a little more. Past 2 turns a real thread has stopped, so a top that kept
  turning was not biting (cross-threaded or stripped).
- Spill: 5 mL, a teaspoon on the hob (a stream falling more than 10 cm splashes a quarter of itself out).
- Upright and in place: the carafe and the base within 10 degrees of upright (a glass standing on its own on a flat
  counter is within a degree or two, so 10 means tipped or leaning on something), and the base within 2 cm of the
  burner's center (its 8.2 cm foot then still sits on all four grate fingers, centered over the flame).
- Arms home: each tool point within 3 cm of its first-step position and moving under 2 mm per step (the arm settles
  within a few mm of a target, so 3 cm tells a parked arm from one left hanging over the stove).

In a real trial: weigh the water in the base before the top goes on (or read the level against the valve bore with a
torch from above, as the camera does), check by hand that the top will not turn further clockwise without force and
does not lift, and weigh a towel under the hob for spills.

## Run a trial

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/moka-pot-filled-below-valve/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Get the moka pot ready to brew." --scorer task_score --max-steps 2400

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`. Offline scorer checks:
`python test_task.py` (prints "all pass").

## The demo

The right arm grips the carafe from the side, brings it over the base already tipped 49 degrees, tips it about its lip 5
cm over the rim to 80 degrees (1.40 rad; tipped that far the carafe keeps 41.7 mL), holds, tips it back and sets it down
where it stood: the base ends with 132.8 mL (7 mm under the valve bore's bottom edge), between the lines, with 0.5 mL
spilled. Meanwhile the left arm picks the upper chamber up by its body with a straight-down grip and holds it aside;
once the carafe is away it lowers it over the base's collar, lets go, and rolls its wrist 630 degrees clockwise in seven
90-degree regrasp strokes; the pads slip a little, so the top turns about 615 degrees (1.7 turns), inside the snug band.
Between strokes the open hand centers itself on the top, rises 8.5 cm until its pad tips clear the handle, spout and
lid, rolls back up there and comes down round the body again (after the last stroke it rises the same way before going
home), as a person lifts their hand off to regrip: a top turning more than 45 degrees per stroke always brings its
handle under the fingers' way back. Its grip commands are offset so the wrist turns about the top's axis, not the tool
point (the held top sits about 1 cm off the tool point in the sim). The burner stands at (27, 5), toward the left arm,
because only there can that arm lift its hand that high over the pot. Both arms go home.

## Runtime and seeds

- Demo: 1,730 control steps, ending in success (score 1.0). Dream-1: 0 at the 2,400-step limit.
- Model run limit: 2,400 steps (2 minutes at 20 Hz). One fixed layout, `object_jitter=0`.

## Fidelity

Real size. No fluid simulation: the water is pourkit's deterministic flow model, and its level is drawn as a blue column
inside the base. There is no physical thread: the upper chamber is a free body that drops over the base's collar (ribs
in its skirt keep it centered), and screwing is scored from its turning there (see Scoring), so the sim gives no torque
rise at snug. No heating or steam. The base is fixed on the grate (a real cook steadies it with the other hand). The
funnel basket step is left out.

## Real-world build

A 3-cup aluminum moka pot (Bialetti Moka Express 3-cup or similar, 15-16 cm tall), its funnel basket removed; a gas
hob or a single-burner camping stove with a cast-iron grate, unlit and cold; a 250 mL glass carafe with 175 mL of
water. Reset: empty and dry the base, unscrew the top and stand it at (24, 19), refill the carafe to 175 mL.

## Licenses

All geometry is generated in task.py; no third-party assets.
