# Raise a ratcheting jack stand's saddle three notches and push its locking handle down to check it

A 3-ton steel jack stand stands on a garage workbench in front of the arms, its saddle at its lowest notch and its
locking handle hanging at the front. The instruction:

> **"Raise the jack stand's saddle three notches from its lowest position, let it settle on the ratchet, then push the
> locking handle down to check that it is locked, then let go."**

Before an oil change or a tire rotation at home, people set each jack stand to the height they need. The stand's
maker describes exactly this: "Adjust height by pulling up on ratchet bar. The weight of the locking handle should
secure the ratchet bar in desired position. To confirm this, simply push down on the locking handle." The robot has
to lift a 1.62 kg steel bar by its saddle without shoving or rocking the 1.4 kg stand on the bench, count the notches
as the pawl clicks over the teeth, stop between the third and the fourth, set the bar down so the third tooth lands on
the pawl, let go, and then press the handle down.

The scene, scoring and scripted demo are in [`task.py`](task.py), for the `yam_arena` embodiment and its `station`
scene. Its helper modules sit next to it: `yamkit.py` spawns the stand as one articulation (a free frame, the bar on a
slide, the handle on its pin) from convex-hull colliders, `gripkit.py` gives the finger pads their rubber friction, and
`yamgeom.py` reads the robot's own collision hulls for the hands-clear and handle-push checks.

## Run a trial

From the repository root, in a shell with the yam_arena environment active:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/raise-jack-stand-three-notches/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 \
  --instruction "Raise the jack stand's saddle three notches from its lowest position, let it settle on the ratchet, then push the locking handle down to check that it is locked, then let go." \
  --scorer task_score --max-steps 800
```

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`. The failure-mode variants below are set
with the `JACK_VARIANT` environment variable. Videos: `inspect-robots video <log>.json`.

## The stand and its ratchet

- **Product:** Big Red T43002 3-ton steel jack stand (Torin). Its operator's manual gives "Minimum Lifting Height
  11¼ inch" and "Maximum Lifting Height 16¾ inch", and describes the ratchet bar, the "locking pawl (stopper)" and the
  "locking handle" on a spring pin. One stand of the pair is used.
- **Sizes estimated from maker photos:** a four-leg stamped-steel frame 23 cm square at the feet and 24 cm tall,
  topped by a square column head; a 26 mm square ratchet bar with teeth 5 mm deep on its front face; eleven positions
  spaced evenly over the published range, 13.97 mm apart; a U saddle 70 x 56 mm over its ears; a bent 10 mm rod
  handle whose grip sleeve sits 10 cm in front of and 5 cm below its pin (the pin's place is set for the simulation,
  below). Three notches up, the saddle's top is
  32.8 cm (12.9 in) above the bench.
- **Masses** come from the parts' modeled volumes and steel's density (7,850 kg/m³): bar and saddle 1.62 kg, frame
  1.42 kg (its leg cut-outs taken out), handle and pawl 0.25 kg; 3.3 kg in all. The bar is within the YAM arm's 2 kg
  payload; the peak pull while a tooth cams the pawl out is about 21 N, the bar's weight plus about 5 N.
- **How it holds:** each tooth has a sloped top face and an undercut load face (leaning back 20 degrees). Lifting the
  bar, the tooth below the pawl pushes the pawl out along its slope, and once the tooth has passed, the handle's own
  weight swings the pawl back in under it. Set down, the tooth's load face lands on the pawl's top; the undercut turns
  the load into a push that seats the pawl further, so a loaded pawl does not walk out. The bar is held up only by
  this contact. Let go between notches, the bar falls to the notch below.
- **Friction:** painted steel on painted steel 0.4 static, 0.35 dynamic (PhysX averages the two surfaces); the YAM's
  rubber-faced pads 1.0 / 0.8.
- **Changed for the simulation:** the teeth's sloped tops are steepened to 58 degrees from horizontal, steeper than a
  real bar's, and the handle's pin is placed high and close to the teeth (6 cm above the pawl's tip and 11 mm in
  front of it), so a rising tooth cams the pawl out with about 5 N, a third of the bar's weight, and the bar lifts
  with little more than its own weight, as a real stand's does. With a lower pin farther forward, the simulated pawl
  took several newtons more to cam out and the pull shoved the stand along the bench.
- **What is simplified:** the bar is guided by its slide joint (the column's guide faces are not collided); at its
  lowest position the bar rests on the pawl by its lowest tooth, as at every notch; the bar's other ends of travel
  are joint limits: 2 mm under its lowest position (standing in for the saddle landing on the column's rim, reached
  only while the handle is held up and the bar dropped) and 4 mm over the tenth notch (not reached by the demo); the
  bar's slide has light viscous damping (0.5 N s/m, no force at rest) and no guide friction; the handle's down and up stops are joint limits (in the real stand the
  pawl seats near the notch's root; here it stops 0.5 mm short), so a push on the handle presses against that stop;
  the handle and the frame do not collide: the pin passes through the column head's sides, where the real stand has
  holes, and a handle lifted past 7 degrees swings the pawl through the column head's front wall, where the real stand
  has a slot; the frame's leg cut-outs are counted in its mass but not cut from its colliders. No vehicle load is
  placed on it.

## Scoring

`task_score` reports `score()` from `task.py`; `success()` is true on each step that counts toward the 1 s hold.

| Score | When |
|---|---|
| 1.0 | The bar is locked three notches up, the handle's grip was pushed down on top by a hand while the bar sat locked there and no hand held the bar, then every arm link is at least 5 mm from the bar, saddle, handle and column head, and everything is still (under 1 cm/s) for 1 s |
| 0.5 | Locked at the third notch and let go, but the handle was never pushed down while it was (or not still for 1 s) |
| 0.25 | Locked at any other notch (1, 2 or 4 to 10) and let go |
| 0 | Anything else, including doing nothing, the bar held up by a hand or left between notches, the pawl not fully home, a hand still on the bar, saddle, handle or column head at the end, and the stand tipped more than 5 degrees from upright at any moment |

- **Locked at notch k:** the bar's slide within 1.5 mm of k notches up (it rests on the pawl there), and the handle
  within 1.4 degrees of its down stop (the pawl's tip within about 1.5 mm of its full 4.5 mm engagement, at least
  3 mm still under the tooth). Read from the bar's slide joint and the handle's hinge, not from any flag.
- **Pushed down:** some arm link within 0.6 mm of the upper half of the handle's front bar or sleeve (the arm's hull
  points lie within 0.5 mm of its surface) for at least 0.25 s in a row (5 control steps; a graze does not count),
  at steps where the bar is locked at notch 3 and every arm link is at
  least 5 mm from the bar and saddle. A hand hovering 1 mm over the grip, touching it from below, or pressing it
  before the bar is set or while a hand still holds the saddle does not count. The check is remembered only while
  the bar stays locked at notch 3: a bar lifted off and set back, or a handle lifted, needs a new check.
- **Tipped** is latched: a stand that tipped past 5 degrees and was set back still scores 0. A stand rocked that far
  under a car would be unsafe, so keeping it upright is part of doing the job right.
- **Not judged:** where the stand ends up on the bench (it may slide a little while the bar is lifted), hands on the
  frame's legs, and a hand on the stand's column head at any time before the end (it must be off the column head,
  at least 5 mm clear, by the end; the demo's left hand steadies the stand there while the bar is lifted).
- **Real trial:** the clicks and the saddle's height (32.8 cm at notch 3, measured with a tape) give the notch; the
  handle hangs at its stop when the pawl is home; the push on the handle, any tipping (a foot lifting about 2 cm) and
  hands left on the bar, saddle, handle or column head come from the video. The push counts only if the hand stays on top of the grip
  for at least a quarter of a second, no hand is on the bar or saddle at that moment and the bar is not lifted
  afterwards.

## Runtime and seeds

The scripted demo takes 581 steps (29 s at 20 Hz) and scores 1.0; the model run limit is 2,400 steps
(2 minutes at 20 Hz). The layout is fixed (`-E object_jitter=0`): no seeds or slots are needed, and the scripted demo
is open loop for this layout (it never reads the stand's pose). It uses both hands, as a person does: the left grips
the column head front to back, low on it (below the handle's arms), and bears down a little so the stand neither slides
nor rocks on the bench while the right lifts the saddle; the left lets go once the bar is set on the pawl and the right
has let go of the saddle, before the handle check. The right's lift is aimed half a notch past the third notch plus
10.3 cm (11 cm above the third notch in all) and 1 cm back and to the right, because the simulated arm droops under
the bar's weight and the cams' push. With the left steadying it, the stand tipped under half a degree and slid about
2 mm during the lift (telemetry from a traced copy of this demo).

## Failure modes (each a demo variant)

| Variant | What the robot does | Score |
|---|---|---|
| `JACK_VARIANT=nothing` | Both arms stay at the ready pose | 0 |
| `JACK_VARIANT=brace` | Lifts the bar past notch 3 and holds it there by hand to the end (the left steadies the stand and lets go) | 0 |
| `JACK_VARIANT=two` | Raises the bar two notches, sets it on the pawl and pushes the handle | 0.25 |
| `JACK_VARIANT=nopush` | Raises three notches and sets it down, never pushes the handle | 0.5 |
| (offline test) | The stand tipped 6 degrees at any moment, then finished correctly | 0 |
| (offline test) | The bar at notch-3 height with the handle lifted (pawl out), or a hand still on the saddle | 0 |
| (offline test) | A hover 1 mm over the grip, a graze on it under 0.25 s, a press while a hand holds the saddle, or a check before the bar was lifted off and set back | 0.5 |

`test_task.py` checks the ratchet's geometry in section (the bar resting on the pawl at every notch, the swung-out
pawl clearing every tooth, a bar let go between notches falling to the notch below, a load seating the pawl), every
row of both tables, the demo plan's reach and its hands' clearance, and the numbers in this text:
`python3 test_task.py`.

## Results

| Run | Score | Steps |
|---|---|---|
| Scripted demo | 1.0 | 581 |
| Dream-1 | 0 | 2,400 steps (the run limit) |

## Real-world build

One Big Red T43002 stand on a sturdy bench (a low-cost setup), with tape marks for its feet. Nothing is consumed and
nothing is under load. Reset: lift the handle, guide the bar down to its lowest position, let the handle drop and set
the stand on its marks, under 30 seconds.

## Sources

- Torin, Big Red T43002 3-ton jack stands operator's manual: lifting range, ratchet bar, locking pawl and handle,
  and the operating steps quoted above.
- NHTSA Part 573 Safety Recall Report 20E-016 (2020), for the failure the handle check guards against: "the jack stand
  pawl may disengage from the extension lifting post, allowing the stand to drop suddenly".

## License

The task's code and text are released under the repository's license. The robot's collision hulls are not shipped
here: `yamgeom.py` reads them at run time from the yam_arena environment.
