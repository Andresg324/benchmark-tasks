# Tap a loose sixtel keg with a Sankey D coupler, steadying the keg by its chime

A sixtel beer keg (a sixth-barrel keg) stands loose on a bar's back counter, valve up. To its right a chrome Sankey D coupler
stands on a drip tray with its handle up (OFF) and pointing at the robot. Its beer line and its gas line stay
connected the whole time, and both run to the cooler wall in front of the robot: the beer line to the beer shank, the
gas line to the regulator.

The instruction is: "Tap the keg with the coupler."

The instruction does not say how. These are the maker's steps for a D-system coupler (Micro Matic,
[How do I tap a keg](https://www.micromatic.com/en-us/how-do-i-tap-a-keg)):

1. "Ensure the keg coupler handle is in the upright (OFF) position."
2. "Align the two coupler lugs with the corresponding openings in the keg valve."
3. "Insert the keg coupler and turn clockwise to engage in position (about 90 degrees)."
4. "Press the handle downward (ON)."

The keg stands loose, neither clamped nor chocked, as behind a bar: the coupler's quarter turn pushes sideways on the
valve, and an empty sixtel (7.5 kg, KegWorks) turns with it unless held. So one hand steadies the keg by its chime
(the rolled top rim) while the other hand seats the coupler, turns it and presses the handle.

The job is changing a keg, which bartender helpers and bartenders do on shift. Dining Room and Cafeteria Attendants
and Bartender Helpers (O*NET [35-9011.00](https://www.onetonline.org/link/summary/35-9011.00): 546,600 US jobs in
2025) have the task "Stock refrigerating units with wines or bottled beer or replace empty beer kegs." Bartenders
(O*NET [35-3011.00](https://www.onetonline.org/link/summary/35-3011.00): 762,600 US jobs in 2025) have the task
"Serve wine, and bottled or draft beer." Category: Impact on jobs.

The skill:

- **Lining up two lugs with two openings.** The coupler is carried by its body and must come down on the valve's axis
  with only a few millimeters to spare, its two lugs over the two openings.
- **A quarter turn clockwise to the stops while pressing down.** Turned the wrong way, the coupler stops before the
  lock.
- **Keeping the loose keg from turning or tipping while doing so.**
- **Only then pressing the handle down past level until it locks.** Pressing it before the coupler is locked would, on
  a pressurised keg, push the probe into an unlocked coupler and lift it off the valve, spraying beer.
- **Working against the pull of two connected lines.**

## Real objects (full size)

Positions are in cm from the midpoint between the arm bases (x forward, y to the robot's left). The counter (rig
table) top is z 0.

| Object | Product and size | Where |
|---|---|---|
| Keg (loose) | A sixth-barrel (sixtel) keg with a Sankey D spear, empty and vented: 23-3/8 in (59.4 cm) tall, 9-1/4 in (23.5 cm) across, 16.5 lb (7.5 kg) empty, 58 lb full ([KegWorks keg size guide](https://www.kegworks.com/blog/guide-to-beer-keg-sizes)). Stainless steel, with a rolled chime above its top plate. The valve's top flange has two openings on its left and right (y) sides | valve axis at (26.0, 0.0); openings along y |
| Coupler | Micro Matic 7485BB D-system coupler, solid chrome-plated brass, black lever. A square body, a handle pivoted on the body's side; beer outlet elbow on top; gas inlet on the side | standing on the drip tray at (20.0, -26.0), handle up and pointing at the robot (-x), lines toward +x |
| Drip tray (fixed) | Black rubber bar drip tray, 12 x 12 x 0.8 cm | center at (20.0, -26.0) |
| Beer line | 3/16 in ID clear vinyl beer line (about 11 mm OD), 76 cm between its clamped barb on the coupler's outlet and the beer shank | shank on the cooler wall at (54.5, 10.0), 15 cm up |
| Gas line | 5/16 in ID red vinyl gas line (about 14 mm OD), 91 cm between the coupler's gas inlet and the regulator | regulator outlet at (54.5, 22.0), 8 cm up |
| Cooler wall (fixed) | A 2 cm board, 40 cm wide and 25 cm tall, standing across the counter (the walk-in or kegerator wall) with the shank and the regulator on it | front face at x 56.0, from y -5 to +35 |

Two yellow paint marks show the turn. One is on the valve flange at the keg's +x side. The other is on the
coupler body's +x face.

### The keg's height (real rig)

The simulation models only the top 10 cm of the keg: its valve, top plate and chime, at the keg's real width and
empty mass. In the real trial the whole empty sixtel (59.4 cm tall) stands loose on a low stand on the floor in front
of the rig table. Its top comes up through a round cut-out in a plywood shelf that is flush with the table top, so the
chime top is 10.1 cm (plus or minus 0.5 cm) above the table, where the simulated keg's top is. The arms never go
below the chime.

- **Target height.** The top of the chime is 10.1 cm (plus or minus 0.5 cm) above the rig table top. That puts the
  keg's bottom 49.3 cm below the rig table top.
- **Stand height.** Stand height = table height minus 49.3 cm (for a 75 cm table, 25.7 cm: a sturdy box or step
  stool shimmed with plywood). Check the height with a tape measure from the floor to the rig table top, then to the
  chime top.
- **Rig requirement.** The rig table's front edge must be at most 14 cm in front of the arm bases' line. The keg's
  back edge sits at x 14.25 when its valve is at x 26.0.
- **Shelf for the tray and the wall.** The drip tray (x 20) and the cooler wall board (x 56) are beyond that edge, so
  they stand on a 3/4 in plywood shelf screwed to the table's front edge, flush with the table top, about 50 x 60 cm
  (x 14 to 64, y -40 to +40). A round cut-out 26 cm across, centered on the valve's axis at (26.0, 0.0), lets the
  keg's top come up through the shelf without touching it. Clamp the shelf's front edge to two 2x4 legs to the
  floor.
- **Marking the keg's place.** Mark the keg's outline with tape on the stand. The keg stands loose inside it: it is
  neither clamped nor chocked.
- **Setting the valve.** Set the valve axis at x 26.0 plus or minus 0.5 cm and y 0.0 plus or minus 0.5 cm (tape
  measure from the bases' midpoint). Turn the keg until the flange openings point along y (plus or minus 5 degrees),
  checked with a square against the tape line.

### How it is simulated

- **Keg.** A free rigid body (7.5 kg, its empty mass) standing on the counter, free to slide, turn and tip. Its
  friction is that of clean, dry metal on wood, 0.2 to 0.6 ([Engineering ToolBox, friction
  coefficients](https://www.engineeringtoolbox.com/friction-coefficients-d_778.html)): the middle of that range, 0.4,
  on the counter and on the fingers. Its valve flange, the two openings, the stops under the flange and its chime ring
  are built from exact annular-sector colliders. The real trial judges tipping on the side video.
- **Bayonet.** It is the contact geometry itself, not a joint. The lugs pass the openings with a few millimeters to
  spare, and the skirt fits round the flange with a few millimeters to spare. The flange's top edge is slightly
  rounded. Turned clockwise, the lugs slide under the flange and stop against the valve's stops at a quarter turn.
  Locked, the coupler can rise a little before its lugs meet the flange.
- **Handle.** An over-center lever: up (OFF), with an over-center point at level and a down stop below level (ON).
  It snaps down once pushed past level and stays down. The score reads only the handle's angle, never a force. The
  handle also drives the probe.
- **Lines.** Chains of short vinyl segments, using the cable kit; each line collides with itself, the keg, the
  coupler, the counter, the wall and the arms. Each is anchored at the wall and attached to its port on the coupler for
  the whole trial. A line pulled hard comes off its barb.

## Scoring

The task file follows the trial after every control step, and `score()` gives the state at the end. Everything is
read in the keg's own frame, so the keg may slide or turn on the counter: moving it is not penalised, only tipping
it more than 10 degrees.

- **0.25: seated.** The coupler sits on the valve with its lugs through the openings: its axis within 4 mm of the
  valve's, its roof within 2 mm of the flange's top, upright within 6 degrees. Resting on the flange with the lugs
  on top instead, it would stand well above it.
- **0.5: turned to lock.** Seated, and turned to within 10 degrees of a locked position. In a locked position the lugs
  are under the flange, against the stops. The two lugs are alike, so a turn of 0 or 180 degrees from the start
  orientation counts.
- **0.75: handle down.** Locked, and the handle pressed down at least 17 degrees below level (over center, ON).
- **1.0: done.** As 0.75, with both hands let go and the keg and coupler still for 1 s. Let go means each
  whole hand (palm, both fingers and their pads) is 1 cm clear of the coupler, its handle and the keg: an open hand
  resting on any of them has not let go. Still means each moves less
  than 0.5 mm per control step. `success()` ends the episode once this has held for 1 s.
- **0 otherwise,** including doing nothing.

The whole trial scores 0 if any of these happens:

- the handle is pushed more than 3 degrees past level while the coupler is not locked (even for one control step);
- the keg tips more than 10 degrees, or falls off the counter;
- the coupler is dropped or knocked over (tilted more than 45 degrees), or falls off the counter;
- a line is pulled off its fitting.

Possible scores: 0, 0.25, 0.5, 0.75, 1.0.

Results on this task.py (sha256 49b88f540b29...): the scripted demo scores 1.0 in 750 steps; Dream-1 scores 0 in 1,200 steps.

### Real-trial check (outcome only)

| Score item | How it is read on the real bench |
|---|---|
| Seated | The coupler sits down on the valve with no gap under its skirt's edge. Photo from the side against the flange; a 2 mm card cannot pass under it. Lugs on the flange leave a wide gap |
| Turned to lock | The top video shows the coupler's paint line within 10 degrees of the flange's paint line, or of the opposite side. A printed 10-degree card sits on the keg top for reference. The coupler cannot be lifted off when pulled up gently by hand after the trial |
| Handle down | The handle is in its down detent (photo). It stays there |
| Let go, still 1 s | Both whole grippers (palm and fingers) 1 cm clear (top video, a 1 cm card for reference); nothing moving for 30 frames at 30 fps |
| Zero events | Handle pressed before the paint lines meet (video); the keg tipped past 10 degrees (side video against a printed 10-degree card); the coupler dropped; a line off its barb (photo) |

## Run a trial

Following the task board's simulation walkthrough (`source /data/IsaacLab-Arena/yam.env`), from this folder:

```bash
inspect-robots run --embodiment yam_arena --policy dream \
  -E task=task.py -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Tap the keg with the coupler." \
  --scorer task_score --no-prompt --max-steps 1200 --log-dir runs/dream
```

`task_score` prints the trial's score: 1.0 on success, otherwise `score()` at the end. The layout is fixed
(`object_jitter=0`). Runtime per trial: at most 1,200 steps (1 minute at 20 Hz). The scripted demo succeeds in about
38 s.

## Scripted demo

Run it with `--policy yam_scripted` instead of `--policy dream`. Its moves come from the fixed layout, except over
the valve, where it reads the coupler's pose from the simulator (step 4).

- **Left arm.** It goes to the chime in front of the valve on the left and leans 35 degrees out over the rim,
  fingertips in. There it pinches the keg's chime across its rim and holds the keg steady. That spot is clear of
  the handle's sweep (front right) and of the lines' sweep (back left) while the coupler turns. Once the coupler is
  turned to lock and let go, it lets go of the keg and moves away (pressing the handle down does not turn the keg).
- **Right arm.**
  1. It takes the coupler by its body, fingers across it between its two ports, and lifts it high enough to carry
     it over the chime.
  2. It carries it in over the rim on the right, turning it a quarter turn counter-clockwise on the way so the lugs
     line up with the openings.
  3. It brings it over the valve, lugs over the openings, leaning its grip against the sag of its loaded wrist.
  4. **It lines the coupler up (closed loop).** Every half second it reads the coupler's pose from the simulator
     (where its skirt hangs over the valve and how far it tilts) and re-aims the hand and its lean by most of the
     miss, holding the coupler clear above the flange. It lowers the coupler, still held and still re-aimed, only
     while it is within 2 mm of the valve's axis and level, until the lugs are down through the openings; off center,
     it holds or lifts it again. Only the hand's targets change; nothing acts on the coupler but contact.
  5. It opens its fingers where the hand actually is, and the coupler settles onto the valve under its own weight.
     It then comes down round the coupler where it actually sits and grips it again.
  6. It turns it clockwise a quarter turn, a little past square so the lugs end firmly against the stops, eases back
     to square and lets go.
  7. It waits over the coupler while the left arm moves off, reads where the coupler's handle is, closes its fingers
     over it, and pushes it down on its grip until it locks. It then lifts off and moves away.

## Offline checks

`test_task.py` checks the following without Isaac Sim, and also runs `test_clearance.py`:

- **Bayonet geometry.** The lugs pass the openings at every height. Turned 45 to 135 degrees, they land on the
  flange. Clockwise they end at the stops at a locked angle; the wrong way they stop 40 degrees short of a lock.
- **Start layout.** Nothing overlaps at spawn: the keg and coupler stand 1 mm proud, and both lines are clear of the
  keg and the wall, above the counter, with their fittings on the ports.
- **Scoring ladder.** Seated, locked, handle down, let go and still, the other locked position, a keg slid and turned
  on the counter, and the keg turned along with the coupler.
- **Faults.** Every zero latch, a one-step early press (latched), and a new trial.
- **Let go.** An open hand with a finger resting on the locked coupler, its tool point and pad tip clear, has not let
  go.
- **Text.** The thresholds in this README and in task.json.

`test_clearance.py` steps through the scripted plan every 0.05 s with the arm's own forward kinematics. It checks the
gripper's shape (palm, finger bodies, pads and tips at the commanded opening) against everything: the keg, counter,
tray, wall, coupler and handle, the line fittings, and the other hand. It also checks the carried coupler against
the keg, counter, tray and wall. Contact is planned in only three places:

- the right pads on the coupler body while holding it;
- its fingertips on the handle while pressing;
- the left pads on the chime.

Everything else stays at least 5 mm clear. The demo plan was also checked with the arm's own IK (`yam_arena.demo.ArmPlan`
and `ik.tcp_pose`, every 0.05 s): no step misses its target by more than 3 mm or moves a joint more than 4 degrees, for either arm.

```bash
python3 test_task.py
```

## Real-world build and reset

- **Keg.** An empty, vented sixtel (sixth-barrel) with a D spear: a returned deposit keg or a new one (MoreBeer sells
  them). Stand it as described in "The keg's height". Paint a 5 mm yellow mark on the flange's +x side.
- **Coupler and lines.** A Micro Matic 7485BB. 76 cm of 3/16 in ID beer line on its outlet barb and 91 cm
  of 5/16 in ID gas line on its inlet barb, each with an Oetiker-style clamp. The far ends go to a shank and a regulator body fixed to a board standing across the shelf at x 56. Paint
  the yellow line on the coupler's +x face.
- **Tray.** A 12 x 12 cm rubber bar mat or drip tray, taped at its outline on the shelf.
- **Reset, about 40 s.**
  1. Lift the handle up past level to OFF.
  2. Turn the coupler counter-clockwise a quarter turn and lift it off.
  3. Stand it on the tray with its handle toward the robot, lines toward the wall. Both lines lie on the shelf in the
     loops shown in setup.png; neither hangs off its edge.
  4. Check the keg sits in its outline with the openings along y.
- **Safety.** No beer and no CO2. The keg is empty and vented, and the gas line ends at a regulator body that is
  connected to no cylinder, so nothing can spray. The coupler's probe only opens the keg's valve. Pinch point: the
  handle closing onto its down stop. The keg is not clamped.

This is a low-cost setup: the keg, coupler, lines, tray, paint and scrap plywood, with common tools.

### Tolerances

| Quantity | Value | How it is read | Source |
|---|---|---|---|
| Keg top height | chime top 10.1 cm (plus or minus 0.5 cm) above the rig table | tape measure | the simulated keg top |
| Valve position | (26.0, 0.0) plus or minus 0.5 cm, openings along y plus or minus 5 degrees | tape measure, square | layout |
| Seated | axis within 4 mm; roof within 2 mm of the flange top; upright within 6 degrees | no gap under the skirt (2 mm card) | the 2 mm card |
| Locked | within 10 degrees of a locked position | paint lines on the top video, 10-degree card | Micro Matic: "turn clockwise ... (about 90 degrees)" |
| Handle down | at least 17 degrees below level | in its detent (photo) | the handle's down detent |
| Pressed early | more than 3 degrees past level before the lock | video | the hazard on a pressurised keg |
| Keg tipped | more than 10 degrees | side video, 10-degree card | design choice |
| Coupler dropped | tilted more than 45 degrees | video | design choice |
| Let go, still | whole hands 1 cm clear; under 0.5 mm per step for 1 s | top video, 30 frames | design choice |

## License

This task is released under the MIT license. `yamkit.py`, `mechkit.py`, `cablekit.py` and `guardkit.py` are the prop,
mechanism, cable and scoring libraries it uses, released under the same license. Every object is built from primitive
shapes.
