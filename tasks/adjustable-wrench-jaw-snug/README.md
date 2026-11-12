# Loosen a nut with an adjustable wrench: roll the jaw snug on the flats before pulling, so it doesn't round the nut

An 8-inch adjustable wrench sits on a tight M12 hex nut (19 mm across flats) on a hold-down stud of a steel base
plate, its jaw left 2 mm wider than the nut. The robot is asked to loosen the nut and take the wrench off. A trained
mechanic first rolls the worm (thumbwheel) until the jaw is snug on the flats, then pulls so that the nut presses
into the fixed jaw, then lifts the wrench off and lays it down. Pulling with the loose jaw can round the nut's corners;
pulling the other way loads the movable jaw, which can spring open. Both score 0.

## Spec

| Object | Type, size, material | Position (cm, table frame) |
|---|---|---|
| Base plate | steel plate 26 x 30 x 0.8 cm, painted blue-gray, four corner bolts; fixed | center (27, 4) |
| Stud, washer, nut | M12 stud; zinc washer 24 mm OD x 2.5 mm; soft aluminum M12 nut, DIN 934 size, 19 mm AF, 10 mm thick, a fresh one each trial | axis at (30, 0) |
| Adjustable wrench | 8-inch (205 mm), chrome steel, 0.25 kg, red grip; jaw open 21 mm; worm 16 mm across, 2 mm jaw per turn | on the nut, handle toward 60 deg |

Scoring (see `score()` in task.py):

| Outcome | Score |
|---|---|
| Doing nothing | 0 |
| Nut turned 2 deg or more in total, over all pulls, with jaw play over 1 mm (loose-jaw pull), or forced 2 deg or more clockwise (movable jaw loaded) | 0 (irreversible) |
| Nut loosened under 5 deg, or a gripper still holding the wrench at the end (fingers under 60 % open, tool point within 5 cm of it) | 0 |
| Loosened 5 to 60 deg | 0.25 |
| Loosened 60 deg or more, wrench not lifted off and laid down 3 cm or more from the nut | 0.5 |
| Loosened 60 deg or more, wrench laid down, not held | 1.0 |

Why each rule and level:

- Loose-jaw pull. A jaw with play lets the wrench turn on the nut until it bears on the corners, which round off;
  then the wrench slips and the nut can't be gripped again. The sim's nut can't deform, so the scorer uses a proxy:
  the nut turning 2 degrees or more, summed over every pull, while the play is over 1 mm. 1 mm comes from the hex's
  geometry: with 1 mm of play the nut turns 5.7 degrees in the jaws before they bear, and they bear on its corners.
  Play is tracked over the whole run, so a jaw snugged for the first pull and opened again before a later one is
  caught. A real trial reads the same play and turn from video, frame by frame, and also checks the corners of a
  soft aluminum nut in before and after photos (see the real-world build).
- Movable jaw loaded. A tightening pull lands on the movable jaw's tip, which levers it open in its slot, and the
  wrench springs off. A 0.48 N*m nut won't spring a real jaw, so this is a proxy too: the nut forced 2 degrees or
  more clockwise. The nut only moves once the wrench has flexed 3.4 degrees against the thread, so 2 degrees of nut
  turn is a real tightening pull, not flex. A real trial reads that turn from the paint line on the nut.
- 5 degrees: more than the jaws' play and the thread's flex, so the nut has really turned on its thread.
- 0.25 for 5 to 60 degrees: the hard part is done (a snug jaw and the right pull broke the nut loose), but it is less
  than one swing.
- 60 degrees: one flat of the hex, a full swing before a mechanic resets the wrench on the next flats.
- 0.5 with the wrench left on: the nut is loose, but the wrench is still on it, in the way of spinning the nut off
  by hand. Set down means lifted off and lying on the plate with its jaws 3 cm or more from the nut's axis, where the
  21 mm jaws are wholly clear of the nut's 21.9 mm corners.
- Held at the end scores 0 because the job isn't finished while the hand is still on the wrench. Held means fingers
  under 60 % open with the tool point within 5 cm of the wrench, so an open hand resting near it doesn't count.

Where the arms end up does not matter.

## How the sim models it

- `base` is one yamkit Mechanism (fixed). The nut is a link turned only by the model. The wrench is a chain: `lift`
  (vertical slide) -> `wr_turn` (about the nut's axis) -> `wr_x`, `wrench` (slides in the wrench's frame), with the
  movable `jaw` (kinematic, set from the worm) and the `wheel` (worm: free to roll, rubbed round by a fingertip) on
  the wrench body.
- The model (`Model`, `model_step`) runs once per control step from the nut's law. On the nut, with the jaw at least
  as wide as the nut, the wrench couples to the nut through a breakaway thread: within the jaws' play (`play_angle`,
  from the hex's geometry) it turns freely, then flexes against a spring (8 N*m/rad) up to 3.4 degrees (0.48 N*m),
  past which the nut turns with it. The worm stops when the jaw meets the flats.
- Off the nut (lifted 16 mm or more, where the jaws clear the stud's end) the wrench moves freely in its plane; laid
  down, its slides and turn are held. It lifts straight off the flats with the jaw snug, as a real one does: the
  jaw is never clamped on the nut, so it does not need opening first.
- Local kit: `yamkit.py` is a local copy of the shared kit with Link.parent chains, cylinder colliders along x/y and
  law-set joint limits.

## Real-world build

Off-the-shelf parts: a 260 x 300 x 8 mm steel plate bolted to the bench, an M12 x 50 stud welded or threaded into
it, a DIN 125 M12 washer, M12 nuts machined from soft aluminum (annealed 1100 hex bar, 19 mm across flats, 10 mm thick;
one per trial), and an 8-inch (205 mm) adjustable wrench (Crescent, Bahco or
similar) with a dipped grip.

The nut breaks loose at 0.48 N*m, the same as in the sim: 3.7 N at the grip, 13 cm from the nut, which the arm
can push. Set it with an adjustable torque screwdriver (0.3 to 1.2 N*m range) driving a 19 mm 3/8-inch-drive socket
through a hex-to-square adapter. Calibrate the setting once: tighten, then turn the nut loose with a dial torque screwdriver
that holds its peak, and adjust the setting until the breakaway reads 0.43 to 0.53 N*m. Recheck it every ten
trials. A nut in service would take about 50 N*m; at 0.48 N*m a steel nut would not round, which is why the nut is
soft aluminum, so the light load has the best chance of marking a corner the jaw bears on. This has not been
bench-tested, so the photos are a second check beside the video readings, not the only one.

Before and after each trial, photograph the nut's top square-on from 15 cm with a macro lens or a phone in macro mode,
in the same raking light and at the same magnification. Any visible flattening or burr on a corner in the after
photo that is not in the before photo is a loose-jaw pull.

Measuring during the run (the nut's turn and the jaw's play, which decide both faults): fix a camera 25 cm
above the nut, looking straight down. Paint a thin line across the nut's top and a reference line on the plate. Draw
a fine line on the movable jaw's top face and stick a 0.5 mm scale on the head beside it, zeroed with the jaw closed
on the nut's flats; check it with 1 mm and 2 mm feeler gauges between jaw and flat. After the run, step through the
video with an angle tool (Kinovea or similar): in every frame read the nut's angle and the jaw's play. Add up the
nut's turn in the frames where the play reads over 1 mm, and its clockwise turn, across every pull and every worm
adjustment the robot makes. 2 degrees or more of either is a fault, as in the sim. If the scale is hidden by a finger while the nut turns, use the larger of the readings
just before and just after. Measure where the wrench was laid down with a tape measure.

Open the jaw to 21 mm with a caliper and lay the wrench on the nut with its handle toward the robot's front-left
(60 degrees), the flats square in the jaws.

## Run

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/adjustable-wrench-jaw-snug/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Loosen the nut on the base plate with the adjustable wrench, then take the wrench off it and set it down." \
      --scorer task_score --max-steps 2400 --log-dir ~/yam-eval
    python tasks/adjustable-wrench-jaw-snug/test_task.py      # offline scorer checks, prints "all pass"

Dream-1: the same with `--policy dream`. One trial takes about 2 minutes (2,400 steps at 20 Hz);
object_jitter=0, so there is no seed dependence.
