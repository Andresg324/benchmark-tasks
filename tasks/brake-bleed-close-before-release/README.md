# Bleed the air out of a brake caliper, shutting the bleeder each cycle before the pedal comes back up

The job is to push the air out of a brake caliper, cycle after cycle, until the pedal is firm. A clear bench brake
trainer: a brake pedal on a stand works a master cylinder whose steel line runs to a disc-brake caliper.
An 8 mm box-end wrench is already on the caliper's bleeder screw, and a clear hose runs from the bleeder into a catch
jar. There is air in the caliper, so the pedal is spongy. "Bleed the air out of this brake caliper." In the two-person
pedal method one hand presses the pedal and keeps it down, the other opens the bleeder a quarter turn so fluid and air
push out, then shuts it while the pedal is still down, and only then does the pedal come up; repeat until no bubbles
come out. Letting the pedal up while the bleeder is open draws air straight back in at the bleeder, because the master
cylinder's piston springs back and pulls on the line.

Run (from the repo root, Isaac Lab-Arena installed):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/brake-bleed-close-before-release/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Bleed the air out of this brake caliper." --scorer task_score --max-steps 1000
    python tasks/brake-bleed-close-before-release/test_task.py      # offline scorer checks, prints "all pass"

Use `--policy dream --max-steps 2400` for a Dream-1 trial. The scene is deterministic (no seeds, object_jitter=0).
The scripted demo plan is 842 steps (42.1 s: five bleed cycles of 7 s each, plus the approach and the return home)
plus the 1 s success hold, so run it with `--max-steps 1000`; a full trial is 2,400 steps. Set `LIVEKIT_DEBUG=1` to
print the caliper's pressure and the event log every step.

Files: `task.py` (scene, network, scorer, demo), `test_task.py`, and the kits it imports, copied here: `yamkit.py`,
`mechkit.py` and `livekit.py` (imported last).

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Pedal stand | steel post 3 x 4 cm on a 12 x 10 cm foot plate | hinge at x 38, y 16, 8 cm up |
| Brake pedal | steel lever 11 cm long, black rubber pad 4.5 x 5 x 1.2 cm; 3.3 cm of travel at the pad (17 degrees) to a floor stop; return spring about 2 N at rest, 7 N fully down | pad centered x 27, y 16, top 9 cm up at rest |
| Master cylinder | aluminum, 3.2 cm across, 7 cm long along x, translucent reservoir 6 x 4 x 4.5 cm with MIN and MAX lines and amber fluid | behind the hinge, x 41 to 48, y 16 |
| Brake line | 5 mm steel tube along the table | from the master cylinder to the caliper's back |
| Caliper | black cast-iron disc-brake caliper 9 x 6 x 6 cm on a 5 x 4 x 3 cm steel bracket | centered x 30, y -12, 3 to 9 cm up |
| Bleeder screw | 1.2 cm hex with a yellow paint ring, nipple to 11.8 cm | x 29, y -11, on the caliper's top |
| Wrench | 8 mm chrome box-end wrench on the bleeder's hex; handle 12.5 cm from the axis, 1.6 cm wide, 0.8 cm thick | 10 cm up; points -y at shut (start), +x a quarter turn open; detents at both, stops 3 degrees past each |
| Bleeder hose | clear vinyl, 8 mm, from the nipple up and over to the jar, its end 2 cm under the fluid | |
| Catch jar | glass, 7 cm across, 9 cm tall, in a wire holder, 4.5 cm of dark old fluid; an inverted graduated tube (1 cm across) over the hose's end collects the air | x 21.5, y -4.5 |

Start: pedal up, bleeder shut, 1.2 mL of air in the caliper and its line (8 mL).

Mechanisms: the pedal is a hinge about y with a return spring (`mk.Spring`, preloaded against its up stop); the
wrench a hinge about z on the bleeder's axis with `mk.Detents` at shut and a quarter turn open (10-degree catch zones).

Pressure network (`livekit`, solved every control step, test section 6 of `test_livekit.py`): the reservoir (a tank
that holds far more than a trial uses), the caliper (an 8 mL tank with 1.2 mL of air, guarded by `protect="gas"`: more
than 0.05 mL of air flowing in logs a latched `air_in` hazard), the pedal (a supply holding 150 psi, at most 4 mL per
press, on for one press stroke (below), when it comes up it pulls 1.5 psi of suction for 0.8 s, the
piston's return, then opens to the reservoir), the jar (fluid 2 cm over the hose's end), the bleeder (a pipe from the
caliper to the jar, its opening rising from 0 at 20 degrees of wrench to 1 at a quarter turn, carrying air and fluid
in proportion), and the bleeder's threads (a one-way pipe from the room into the caliper, open as far as the bleeder).

## Scoring

Air drawn back in is latched for the whole trial; everything else is read from the final state, which must hold 1 s
(SCORE_OVER = "end").

| Outcome | Score |
|---|---|
| Air drawn back in at any time, latched: a press stroke ending with the wrench more than 20 degrees from shut on the step before the pedal started up or after, the wrench past 20 degrees during the pedal's 0.8 s return, or more than 0.05 mL of air flowing in | 0 |
| A tool point within 3 cm of the pedal pad or the wrench handle at the end | 0 |
| Pedal or wrench still moving faster than 0.2 rad/s | 0 |
| Wrench more than 10 degrees from shut (bleeder left open) | 0 |
| Pedal more than 30% down | 0 |
| Air left above 0.84 mL (less than 30% out; doing nothing ends here) | 0 |
| Air left 0.84 mL or less but above 0.2 mL | 0.25 |
| Air left 0.2 mL or less, both arms home and still | 1.0 |
| The same, arms not home | 0.9 |

Possible scores: 0, 0.25, 0.9, 1.0.

Why the numbers:
- 20 degrees on the pedal's return: the bleeder starts passing fluid 20 degrees past snug, and from there its loosened
  threads let air in on the return stroke. The network alone would only flag a return with the bleeder past about 35
  degrees (its 0.05 mL threshold), which left a cheap path: crack the bleeder to 30 degrees and pump the pedal
  one-handed. The task rule closes it (test_task: 25 pumps with the bleeder held at 30 or 35 degrees score 0), and
  also catches the bleeder opened during the pedal's 0.8 s return.
- Latched, though more strokes would push that air out again: each pedal-up-first cycle pulls air back, and ten such
  cycles in a row leave 0.57 mL, never a firm pedal. The latch makes the score follow the procedure's one rule rather
  than how many extra strokes a run had time for.
- 0.2 mL left: about one sixth of the starting air; four correct cycles reach it (0.19 mL), three do not (0.31 mL).
  It stands for the firm, high pedal a technician checks for at the end.
- 0.84 mL (30% out): one correct cycle (0.76 mL) is real progress; pumping the pedal or working the wrench alone
  moves no air.
- 70% of the stroke: the master cylinder's piston has covered its compensating port and pushes fluid; a half press
  does nothing (tested). A press stroke starts past 70% only after the pedal was back above 30% down (a wiggle around
  70% is not a new pump) and ends, the piston returning, once the pedal is above 70% or 10% of its stroke above its
  deepest point in that press, for 2 steps in a row (a one-step PhysX pop is not a return). The wrench is checked on
  the step before the pedal started up, so snapping it shut on the same step does not count as shut first (QC review,
  tested).
- 10 degrees: the wrench's detent catch zone; inside it the detent pulls the bleeder snug.
- 3 cm hands-off: just over the pad's and the handle's thickness, so a gripper on either is inside it and one lifted
  clear is outside.

In a real trial: inject 1.2 mL of air into the caliper by syringe at reset; the inverted graduated tube over the hose's
end in the jar collects the air pushed out, so the air left is 1.2 mL minus its reading. On the video, check on every
cycle that the wrench was back within 20 degrees of shut before the pedal started up, and at the end that the wrench
sits in its shut detent.

## Real-world build

- A bench brake-bleeding trainer from parts: a single-circuit master cylinder with its reservoir, on a stand with a
  pedal lever (or a manual master cylinder with a short lever arm, as sold for hot-rod and kart brakes), a 3/16 in.
  steel brake line with flare fittings, and a single-piston disc-brake caliper on a steel bracket.
- An 8 mm box-end wrench, about 1 ft of clear 1/4 in. vinyl hose, a glass jar in a wire holder, and a 2 mL graduated
  tube (or a 3 mL syringe barrel, tip sealed) inverted over the hose's end in the jar to collect the air.
- A clear bench rig: a clear-body caliper or a clear acrylic stand-in for it, so the air pocket can be seen.
- Fluid: propylene-glycol RV antifreeze (nontoxic, water-clean, safe on the robot's table), not brake fluid (DOT 3
  and 4 strip paint and irritate), not mineral oil (it swells the ordinary seals of a DOT-fluid master cylinder) and
  not DOT 5 silicone. Keep the rig over a tray.
- A 5 mL syringe with a short tube to inject 1.2 mL of air at reset through a fill port on the caliper.
- A light return spring on the pedal (about 7 N at the pad pressed), so trials are safe for a YAM setup.

## How it differs from nearby board tasks

- Lower a cabinet with a hydraulic release valve (#1509, #1502 closed): one hand meters one valve to control a load's
  descent speed. Here two hands must be timed against each other, cycle after cycle: the pedal hand may only let go
  after the wrench hand has shut the bleeder, and the score comes from the air the network moves.
- Hold a pedal while the other hand swaps a filter (#1107): one static hold, released once at the end; nothing depends
  on when it is released relative to a valve.
- Double block and bleed before removing a cap (#378), verify zero pressure before opening (#663): venting is a
  precondition for opening; here bleeding air out is the job.
- Fit a caliper over the disc / fit pads (#976, #1089): the same equipment, no hydraulics.
- Connect jump leads in the safe order (#21): a connection order for spark safety, nothing timed.
