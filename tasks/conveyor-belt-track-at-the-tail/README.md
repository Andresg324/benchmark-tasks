# Track a drifting conveyor belt back to center at the tail take-ups: at most a quarter turn per two belt revolutions

A benchtop belt conveyor runs with its belt walking toward the far side frame. "The belt on this conveyor is running
off to one side. Fix it before it rubs the frame." means what a maintenance mechanic does: track it at the tail
pulley's take-up screws, a quarter turn at most at a time on the side that steers it back, then watch several belt
revolutions before the next turn, because a belt answers a pulley change over the next revolutions, not at once. It
must end centered within 3 mm and running steady, without ever touching a side frame, with the belt's tension left in
its working range. The practice itself is scored: more than 0.3 turn of adjustment within any two belt revolutions
(7.4 s) is a 0, so the whole half-turn correction this start needs cannot be made in one go or in back-to-back
quarters. The other wrong moves, cranking a screw well past the fix and turning the side that makes it worse, run the
belt into a frame.

## Spec

- Job: Industrial Machinery Mechanics (O*NET 49-9041.00); Maintenance Workers, Machinery (49-9043.00); Millwrights
  (49-9044.00).
- Rule and sources: track at the tail take-ups, "Adjust no more than a quarter turn at a time, then run several
  revolutions before adjusting again" ([Texas Belting, belt tracking guide](https://texasbelting.com/pages/conveyor-belt-tracking-problems-causes-how-to-fix-them-texas-belting-supply));
  the belt tracks toward the side with the least tension ([ERO](https://www.ero-joint.com/en/track-and-adjust-conveyor-belt-tension-alignment),
  Habasit via [Nedco](https://feeds.nedcofold.com/blog/conveyor-belt-alignment-system)); repeat until the belt is
  "within a quarter-inch of the center" and expect it to move "within seven revolutions"
  ([Hytrol, How to Track a Belt](https://blog.hytrol.com/ask-hytrol-how-to-track-a-belt)). The sources disagree on
  which side to turn (Texas Belting says the belt moves toward the side tightened); the task makes no claim beyond the
  law below (the real build calibrates the direction first), and either correct move (tighten the far screw or loosen
  the near one) scores.
- Mechanism: `beltkit.py` (new, small): the belt's sideways offset e follows a lagged steering law driven by the tail
  pulley's skew, which the two take-up screws set (twistkit threads, end-on knobs). Law and constants below.

| Belt law (beltkit) | Value | Why |
|---|---|---|
| Belt speed v | 0.20 m/s | 40 ft/min, a common light-duty package-conveyor speed; one revolution of the 0.74 m loop every 3.7 s |
| Take-up per turn | 1.5 mm | M10 x 1.5 adjusting screw |
| Skew from the screws | (a_far - a_near) / 0.18 m | the screws are 18 cm apart, on the pulley's bearing line |
| Steering | de/dt = -v (skew_eff + skew0) - (v / 3.2 m) e | a belt on a skewed pulley walks toward the end with less tension at v x skew; the crowned pulleys pull it back toward center with a 16 s time constant (3.2 m / v), so it settles where the two balance |
| Lag | skew_eff follows skew with a 1-revolution (3.7 s) time constant | the belt must run over the pulley before the new angle is wrapped into it |
| Quarter turn of one screw | moves where the belt settles by 6.7 mm | 0.375 mm / 0.18 m x 3.2 m |
| Start | e = +2 mm, tail pulley 0.75 mm out of square (half a turn on the far screw) | settles 13.3 mm toward the far frame: left alone it touches the frame (10 mm) about 20 s in |
| Rub | latched at any substep with \|e\| >= 10 mm | a 130 mm belt between frames 150 mm apart |
| Stretch | 0.5 % + (a_near + a_far) / 0.74 m | both take-ups lengthen the loop |

The law is integrated over 10 substeps per control step with the knob angles interpolated, so a knob spun fast between
two steps steers (and rubs) the same as a slow turn.

## Files

| File | What it is |
|---|---|
| `task.py` | Scene, scorer (`score()`, `success()`) and scripted demo (`demo()`), for the `yam_arena` embodiment (Isaac Lab-Arena, bimanual YAM, `station` scene). Its docstrings explain every rule. |
| `beltkit.py` | The belt: the tracking law above, the rub latch, the belt's stretch, and its drawing (a black belt with yellow chevrons moving at belt speed and a white center stripe, shifted sideways by e; the edge sensor's LED bar on the tail guard, one green LED lit per mm of e; a red wear mark on the frame after a rub). Built on `twistkit.py`. |
| `twistkit.py` | Threaded parts: each take-up knob is an articulation turning on its screw's axis; gives the knob's angle and whether a gripper's fingers are closed on it. |
| `guardkit.py` | The let-go check (every tool point and pad tip clear of the whole knob) and the arms-home check. |
| `yamkit.py` | Helper that spawns the modeled props and articulations, and gives the scorer the grippers. It patches `yam_arena` in memory at import; no files change. |
| `test_task.py` | Offline checks of the law (drift direction, lag, settling, slot stops, tension, fast turns) and of the scorer (every mistake below, near misses, holding and hovering at the end, reset). No simulator needed. |
| `task.json` | The task card. |

## Run a trial

Scripted demo (reaches full score):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/conveyor-belt-track-at-the-tail/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "The belt on this conveyor is running off to one side. Fix it before it rubs the frame." \
      --scorer task_score --max-steps 3600

Model run (Dream-1): the same command with `--policy dream`. Render the videos of a run with
`inspect-robots video <log.json>`. Scorer tests: `python tasks/conveyor-belt-track-at-the-tail/test_task.py` (prints
`all pass`). One trial takes up to 3,600 steps (3 minutes at 20 Hz). No seeds: the layout and the start are fixed.

## The demo

1. Right arm: turns its hand to point along +y, end-on to the far take-up knob (x 37), closes on its top and bottom
   lobes and turns it a quarter turn clockwise as seen from the knob's end (tightening the far side), lets go and
   backs off 3 cm.
2. It waits 8 s (about 2 revolutions): the belt keeps drifting out for about one revolution, then turns back.
3. A second quarter turn on the same knob, let go, back off, home. The left arm stays home.
4. The belt settles on the center line; the run ends when the scorer has seen it steady for 10 s.

## Scoring

e is the larger of the belt's offset from the center line now and where it settles with the screws as left (the
belt's own law; in a real trial, measured again after 2 more minutes of running).

| Outcome | Score |
|---|---|
| The belt's edge reached a side frame (10 mm off center) at any moment, even if tracked back after | 0, irreversible |
| A take-up screw backed out of its bearing block (more than 2 turns loosened) | 0, irreversible |
| More than 0.3 turn of adjustment (both screws together, a screw turned out and back counting its whole excursion) within any 7.4 s | 0, irreversible |
| A gripper holds a knob at the end, or a tool point or pad tip is within 4 cm of any part of a knob | 0 |
| The belt's edge moved 0.1 mm/s or more at any step of the last 10 s (still drifting) | 0 |
| e > 6 mm | 0 |
| 3 mm < e <= 6 mm (running clear, off center), or e <= 3 mm with the stretch outside 0.3 to 0.8 % | 0.25 |
| e <= 3 mm, stretch 0.3 to 0.8 %, let go, an arm not home | 0.9 |
| The same with both arms home and still | 1.0 |

Thresholds: 0.3 turn is Texas Belting's quarter turn plus 0.05 turn (18 degrees) for the knob's slip and coast under a
full quarter-turn stroke, and 7.4 s is two revolutions of the 0.74 m loop at 0.2 m/s, the fewest that are "several"
(the window runs from the end of one turn to the end of the next, so turns made one revolution apart are rushed);
3 mm is Hytrol's quarter inch (6.35 mm) at the model's 1:2 scale; 6 mm is half an inch at 1:2; 10 mm is the
belt's clearance to each frame; 0.1 mm/s is 1 mm over the last 10 s (about 2.7 revolutions), less than a ruler at the
tail can show; the 0.3 to 0.8 % stretch window is the model's working range around the 0.5 % set tension (under it
the drive pulley slips under load, over it the splice and bearings are overloaded); 4 cm is a finger's width plus
the arm's 1.6 cm lag behind its target.

Mistakes and what they score (each one is a test in `test_task.py`):

| Mistake | Score |
|---|---|
| Do nothing | 0 (rubs the far frame at about 20 s) |
| Tighten the near screw (wrong side) | 0 (rubs sooner) |
| One big crank, 1.5 turns on the far screw | 0 (rushed; it also overshoots and rubs the near frame) |
| The whole fix at once: one half turn on the far screw (or 0.6 turn) | 0 (rushed; the belt itself would center) |
| Two quarter turns back to back, or one revolution apart | 0 (rushed) |
| A full crank, backed off half a turn 2 s later | 0 (rushed) |
| A quick 0.4-turn wrong-way flick, undone at once | 0 (rushed, latched) |
| Impatient: a quarter turn every 4 s, four times | 0 (rushed; it also rubs the near frame) |
| Too late: the belt rubbed, then was tracked to center | 0 |
| Only one quarter turn | 0 (settles 6.6 mm off) |
| 3/8 or 5/8 of a turn, in waited steps | 0.25 (settles 3.3 mm off) |
| Centered, but both screws a turn tighter (or looser), in waited quarters | 0.25 |
| Two waited quarter turns on the far screw (the demo), or two on the near screw loosening | 1.0 |
| Still holding a knob, or hand resting by it | 0 |

## Real-world build

- Conveyor: a benchtop 24 V DC flat-belt conveyor (about $300), about 60 cm long with a 130 mm belt (or a 150 mm belt
  between frames 170 mm apart, keeping 10 mm of clearance a side), speed set to 20 cm/s. Its tail take-ups are M10
  screws; fit each with a 34 mm four-lobe knob (threaded-insert star knob) on the screw's outer end and paint an index
  mark on the top lobe. Guards: sheet guards over both pulley nips and the return run, the knobs outside the guards, an
  e-stop within reach of the operator. The belt speed and forces are low, but this is a powered trial: keep the arms'
  path clear of the head pulley and motor.
- Edge sensor: a belt-edge position sensor with a 21-LED bar (the infrared or ultrasonic edge sensors sold for web
  guiding, about $250), clamped on the tail guard and set to light one LED per mm of offset across +-10 mm. The belt's
  millimeters are too small for the overhead camera; the bar shows the same offset the model scores, and the robot and
  a reviewer read it the same way.
- Calibration (do once, before trials): from centered, give the far screw a quarter turn and log the belt edge against
  a ruler at the tail every revolution until it stops moving; the settled shift and the 63 % time set the model's
  3.2 m centering length (6.7 mm per quarter turn) and 16 s time constant. Update the law's constants if they differ.
  The same run confirms the direction: a quarter turn tightening the far screw must send the belt toward the near
  frame (the model's rule, "toward the side with the least tension"). If this conveyor's belt goes the other way (Texas
  Belting's "toward the side being tightened"), start trials with the near screw half a turn short instead, so the
  correct fix is still two quarter turns.
- Start: set both knobs to their paint marks, then back the far one half a turn (0.75 mm out of square), center the
  belt by hand with the conveyor stopped, start it and begin the trial when the edge passes 2 mm toward the far frame.
- A strip of masking tape along each frame's inner face at the tail shows a rub (scuffed tape); replace it after a
  trial that scuffs it.
- Real-trial check: the edge sensor's reading (and the belt edge against the center mark with a ruler) at the end and again after 2 more
  minutes of running; video and the tape for any rub; the knobs' index marks for the turns (stretch).
