# adjustable-wrench-jaw-snug

An 8-inch adjustable wrench sits on a tight M12 hex nut (19 mm across flats) on a hold-down stud of a steel base
plate, its jaw left 2 mm wider than the nut. The robot is asked to loosen the nut and take the wrench off. A trained
mechanic first rolls the worm (thumbwheel) until the jaw is snug on the flats, then pulls so that the nut presses
into the fixed jaw, then lifts the wrench off and lays it down. Pulling with the loose jaw rounds the nut's corners;
pulling the other way loads the movable jaw, which springs open. Both score 0.

## Spec

| Object | Type, size, material | Position (cm, table frame) |
|---|---|---|
| Base plate | steel plate 26 x 30 x 0.8 cm, painted blue-gray, four corner bolts; fixed | center (27, 4) |
| Stud, washer, nut | M12 stud; zinc washer 24 mm OD x 2.5 mm; zinc DIN 934 M12 nut, 19 mm AF, 10 mm thick | axis at (30, 0) |
| Adjustable wrench | 8-inch (205 mm), chrome steel, 0.25 kg, red grip; jaw open 21 mm; worm 16 mm across, 2 mm jaw per turn | on the nut, handle toward 60 deg |

Scoring (see `score()` in task.py; every threshold has its reason in the code and task.json):

| Outcome | Score |
|---|---|
| Nut turned 2 deg or more with jaw play over 1 mm (corners rounded), or forced 2 deg or more clockwise (movable jaw loaded) | 0 (irreversible) |
| Nut loosened under 5 deg, or a gripper within 5 cm of the wrench at the end | 0 |
| Loosened 5 to 60 deg | 0.25 |
| Loosened 60 deg or more, wrench not lifted off and laid down 3 cm or more from the nut | 0.5 |
| Loosened 60 deg or more, wrench laid down, hands off, arms home | 1.0 (0.9 if the arms are not home) |

## How the sim models it

- `base` is one yamkit Mechanism (fixed). The nut is a link turned only by the model. The wrench is a chain: `lift`
  (vertical slide) -> `wr_turn` (about the nut's axis) -> `wr_x`, `wrench` (slides in the wrench's frame), with the
  movable `jaw` (kinematic, set from the worm) and the `wheel` (worm: free to roll, rubbed round by a fingertip) on
  the wrench body.
- The model (`Model`, `model_step`) runs once per control step from the nut's law. On the nut, with the jaw at least
  as wide as the nut, the wrench couples to the nut through a breakaway thread: within the jaws' play (`play_angle`,
  from the hex's geometry) it turns freely, then flexes against a spring (8 N*m/rad) up to 3.4 degrees (0.48 N*m),
  past which the nut turns with it. The worm stops when the jaw meets the flats.
- Off the nut (lifted 16 mm or more) the wrench moves freely in its plane; laid down, its slides and turn are held.
- Local kit: `yamkit.py` is the sewing-pivot-at-corner copy (Link.parent chains, cylinder colliders along x/y,
  law-set joint limits).

## Real-world build

Off-the-shelf parts: a 300 x 260 x 8 mm steel plate bolted to the bench, an M12 x 50 stud welded or threaded into
it, a DIN 125 M12 washer and DIN 934 M12 zinc nut, and an 8-inch (200 mm) adjustable wrench (Crescent, Bahco or
similar) with a dipped grip. Tighten the nut to a set torque the arm can break (about 5 N*m with a torque wrench)
and mark a paint line across nut and washer. Open the jaw to 21 mm with a caliper and lay the wrench on the nut with
its handle toward the robot's front-left (60 degrees).

## Run

    ~/yam-tools/run_task.sh tasks/adjustable-wrench-jaw-snug demo 1000
    ~/yam-tools/run_task.sh tasks/adjustable-wrench-jaw-snug dream 2400
    python tasks/adjustable-wrench-jaw-snug/test_task.py      # offline scorer checks, prints "all pass"

`WRENCH_DEBUG=1` prints the model's state every 10 steps. One trial takes about 2 minutes (2,400 steps at 20 Hz);
object_jitter=0, so there is no seed dependence.
