# Baste a roast with a bulb baster held by its bulb: squeeze, dip in the juices, release, carry, squeeze onto the roast

A roast sits in the middle of a 13 x 9 in roasting pan with 3 cups of juices around it, and a bulb baster lies on the
counter beside the pan. The robot bastes the roast: it holds the 12 in baster by its bulb alone, tip-down, squeezes
the bulb, dips the tip into the shallow juices, eases off so the bulb draws them up, carries the column of juices
(held in the tube only by the bulb's suction) over the pan's rim and the roast without dripping, and squeezes it out
onto the roast's top. Then it sets the baster back down on the counter.

How it differs from the merged pipette tasks: there, a lab pipette or filler moves a set volume between fixed
vessels (in the tube-filling task a squeeze and release moves one fixed pipetteful by bookkeeping). Here the draw,
the carry and the release are physical: the bulb's vacuum is what holds the juices in a 30 cm tube carried
tip-down over a rim and an obstacle, the same light squeeze on the soft bulb both holds the baster and pumps it, the
juices come from a pool only 1.3 cm deep, and the outcome is a jet aimed onto an open surface, scored by how much
lands on it and how little on the counter.

Instruction: "Baste the roast with the juices from the pan, then put the baster down on the counter."

## What the task tests

- **A long tool held only by its soft end.** The baster is 30 cm long and is held by a light squeeze on its bulb,
  hanging tip-down, the one-handed way people baste (steadying the tube with the other hand is also allowed and also
  scores). The same fingers that hold it also work it: squeezing harder empties the bulb, easing back to the
  holding squeeze draws the juices up. Squeeze too lightly and the baster slips; ease off too far and it drops.
- **A column held by suction, carried.** Between the dip and the roast, the juices stay in the tube only because the
  air in the bulb is below atmospheric pressure. Squeezing on the way spills them on the counter or the pan's rim;
  tilting the baster level or tip-up lets air in through the opening, and when it points down again part of the
  juices run out.
- **Dipping in a shallow pool.** The juices stand about 1.3 cm deep around the roast. The tip must be under the
  surface, in the strip between the roast and the pan's wall, when the bulb opens; if the tip is out of the juices
  when the bulb opens, it draws air.
- **Aiming the release onto a target.** The juices leave the tip as a jet. Squeezed with the tip over the roast's
  top, they land on it; squeezed while moving or with the baster tilted, the jet carries past it, the further the
  higher the tip.

## Run a trial

```
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/baste-a-roast-with-a-bulb-baster/task.py -E object_jitter=0 -E control_hz=20 \
  --instruction "Baste the roast with the juices from the pan, then put the baster down on the counter." \
  --scorer task_score --max-steps 2400
```

`python3 test_task.py` runs the offline checks (no simulator).

## Why it matters (Societal relatability)

Basting a roast chicken, a pork roast or the holiday turkey with a bulb baster is an everyday home-kitchen job that
most people have done or watched. It only works in one order: squeeze, dip, let the bulb open with the tip still
under, lift out without squeezing, aim, squeeze. At home the pan is basted with the oven rack slid part way out
or with the pan lifted onto the stovetop; here it stands on the counter at the same reach, with room-temperature water
for the hot juices so nothing hot is near the robot or the operator.

## Scoring

In simulation, the juices that land on the roast stand-in's top and on the counter are counted in mL.

| Score | Outcome |
|---|---|
| 1.0 | At least 20 mL on the roast's top, at most 2 mL on the counter, the pan never tipped more than 8 degrees, and the baster let go resting on the counter outside the pan (not dropped or thrown) with both arms clear of it, all still for 1 s |
| 0.75 | The same juices, the baster not yet set down, let go or still |
| 0.5 | At least 20 mL on the roast, more than 2 and at most 10 mL on the counter |
| 0.25 | 8 to 20 mL on the roast, or at least 8 mL on the roast with more than 10 mL on the counter |
| 0 | Under 8 mL on the roast, or the pan tipped past 8 degrees; doing nothing |

Juices that fall back into the pan count neither way. 20 mL is half of the baster's 40 mL. Juices count on the roast
only while it is in the pan with its top facing up (within 20 degrees of up).

Details, in simulation:

- Set down: when the arms let go of the baster, its lowest point is within 5 mm of the counter, it is outside the
  pan, and it moves under 0.3 m/s. Clear: every arm link (link1 to the fingertips, both arms) at least 5 mm from the
  baster, read from the arms' own collision hulls. Still: the baster moves under 1 cm/s.

## Real trial

Judged by eye from the overhead and wrist-camera videos and one end photo; an approximation of the simulated rules
above.

- Props: the items under Real-world build. A white wipe-clean vinyl tablecloth covers the counter, with the pan
  standing on it, so a brown drop shows on it. Two tape marks on the tablecloth: the baster's outline at its start,
  and the pan's corners.
- Start: the pan on its marks with the stand-in in its middle and the 3 cups of colored water around it; the baster
  empty and dry on its outline; both arms at the ready pose. Nothing is random.
- Score, from the video and the end photo:
  - 0: one side of the pan visibly lifts (the pan tips), no brown water lands on the stand-in's top, the tube never
    filled to its first (10 mL) ring, or doing nothing.
  - 0.25: brown water lands on the stand-in's top, but the tube was filled to less than half its 40 mL markings, or
    a puddle or a run of water lands on the tablecloth.
  - 0.5: the tube filled at least half way up its markings, and the stream from the tip lands on the stand-in's top,
    but more than a few drops land on the tablecloth.
  - 0.75: the same baste with at most a few drops on the tablecloth.
  - 1.0: the 0.75 outcome, then the baster laid down on the counter outside the pan (not dropped), the hands off it
    and everything still for 1 s.
- Reset (about 1 min): squeeze any water left in the baster back into the pan, wipe its tip and bulb dry and lay it on
  its outline. Most of the water that landed on the stand-in has run back into the pan; wipe its top dry so the next
  trial's landing shows in the video. Wipe the tablecloth dry, and top the pan up to its 3-cup fill mark (a strip
  of tape on the pan's outside, marked when the 710 mL is first poured in). Nothing is consumed but a little tap
  water; nothing is weighed or measured.

## The baster and its suction

The baster is a free body at the OXO baster's size and weight: a rigid tube (a convex hull tapering from the tip to
the bulb) with a flat bulb whose four walls are sprung panels on prismatic joints around a rigid core. A hand squeezes
the bulb only by pushing those panels in; their pushes are read from the joints every control step. A grip on the
tube, the bulb's neck or its top squeezes nothing, and a finger near a panel that does not press it moves nothing.
The silicone bulb's friction is rubber on rubber, clean and dry, 1.16 (Engineering ToolBox, friction coefficients);
the YAM's finger pads are rubber-faced. The wall's spring (2,000 N/m per panel, so about 30 N a finger to empty the
bulb, within the YAM's roughly 40 N per finger), the bulb's 100 mL inside volume and 21 cm² inside cross-section, the
pan's 0.9 kg and the props' friction on the counter are estimates,
not measurements.

How it is drawn: the bulb as a rounded flat oval, each panel carrying its quarter of the oval so a squeeze shows as
the sides going in. The bulb's rounded top, the short flare at its neck and the corners between the panels are drawn
only: the collision shapes there are the rigid core, the neck's collar and a small dome inside them. The clear tube
is drawn opaque (the renderer draws see-through surfaces as solid) with dark rings at its 10, 20, 30 and 40 mL
markings, and the juices inside it are drawn over it as a brown column of the same length. The pan has a rolled rim,
drawn only.

The suction is a stated law in `bulbkit.py`, driven only by the panels' joints and the tip's pose:

- The air inside the bulb and the tube's bore follows Boyle's law: gauge pressure p = P_atm (air / V_gas - 1), with
  V_gas the inside volume less what the panels displace (each panel's face area times how far it is in) and less the
  juices inside.
- A panel is in as far as a finger pushes it, or, under a vacuum, as far as the vacuum draws it in against its own
  spring (k x = |p| A), whichever is further. So the bulb holds only the vacuum its wall resists, and a release draws
  only as the wall springs back. This balance is worked out in the law each control step rather than through the
  simulator's joints.
- The tip's opening passes juices by the orifice law, Q = Cd A sqrt(2 dp / rho), with the sharp-edged coefficient
  0.61 for its thin lip, driven by the pressure difference across it. Under the juices' surface by a depth s, the
  pressure outside the opening is rho g s over the atmosphere and inside it is p plus the column of juices already
  inside: juices come in only while the outside is higher. With no juices inside, air bubbles out while p is above
  rho g s.
- In the air, juices inside run out while p plus their column beats the surface tension that holds a meniscus in the
  opening (4 sigma / d). When nothing seals the opening (no juices inside, or the baster held level or tip-up so they
  lie away from it), the inside is at atmospheric pressure.

So the bulb draws juices up only when the air inside is below the pressure at the submerged opening. That takes air
pushed out first (a squeeze) and the bulb opening again with the tip under the surface. A baster that was never
squeezed lets the juices rise in its tip only to about the outside level, a fraction of a mL; dipping it again and
again without a squeeze and release keeps a fraction of a mL per dip. A squeeze let go in the air pulls in air; a
squeeze held while the tip is dipped and lifted draws nothing. A filled baster held tip-down in the air keeps its
juices: the air inside goes slightly below atmospheric and holds the column.

The juices' weight is put on the baster as a force at the middle of the column they fill, every control step.

The juices that leave the tip in the air are a jet. It leaves along the tube at the speed the orifice gives it, plus
the tip's own velocity, and falls ballistically. It counts on the roast if it comes down through the roast's top,
back in the pan if it comes down inside the pan's rim, and on the counter otherwise.

The pan's juices are a level pool around the roast: their depth is their volume over the floor between the pan's
walls and the roast. The tip is under the surface when it is over that floor and below that level. What the baster
draws comes out of the pool, and what falls back into the pan goes back in. Volume is conserved.

Limits: the air inside is one lumped volume; there is no film or wetting inside the tube; the pan's surface stays
level (the task does not move the pan, and tipping it past 8 degrees scores 0); the pan's walls are modeled straight.
Held level or laid down, the juices stay in the tube in the model; a real baster may dribble, and the real trial
counts whatever reaches the tablecloth.

## The demo

One hand, the way people baste. The left takes the lying baster by its bulb from above, with a firm squeeze across
the bulb's narrow sides (the panels' grip carries the baster), and lifts it, turning to point forward in steps so the
baster turns with the hand and ends hanging tip-down below the bulb, its tip always clear of the counter; then it
eases to a light holding squeeze. Over the pan's right-hand strip
of juices, between the roast's end and the pan's wall, it squeezes the bulb, lowers the tip under the surface to
4 mm off the pan's floor, and eases back to the light hold: the bulb opens and draws the juices up. It lifts the
baster out over the roast, tip 2.5 cm above the roast's top, squeezes the bulb (the juices go onto the roast) and
eases off. Then it carries the baster back, lays it down where it was, lets go and goes home. The right arm stays at
its ready pose.

## Real-world build

A low-cost setup from common kitchen and hardware items:

- OXO Good Grips baster with cleaning brush (12 x 2.5 x 2 in, a flat silicone bulb, a Tritan tube, 1.5 oz / 40 mL).
- Chicago Metallic Professional roast pan, 13 x 9 x 2-1/4 in, its rack left out.
- Roast stand-in: a 9 in length of 4x4 lumber (3.5 x 3.5 in dressed, about the size of a 3 lb boneless pork loin
  roast; estimated), wrapped in plastic film, lying in the middle
  of the pan.
- 3 US cups (710 mL) of room-temperature water with a few drops of brown food coloring; a white wipe-clean vinyl
  tablecloth; a bath towel; masking tape.

The stand-in is a roast-sized target, not meat, so nothing spoils and nothing is cooked: what lands on its film-wrapped
top runs back into the pan. Room-temperature water stands in for hot pan juices so the trial is safe for the robot and
the operator. Rig protection: the fingers touch only the bulb, which stays above the tube and the water for the whole
trial; the water is inert (tap water with a few drops of food coloring) and wipes off; the pan stands on the
tablecloth, its near edge about 25 cm in front of the arms' bases, and a rolled bath towel lies along the
tablecloth's edge in front of the bases as a dam: the most that can spill is the pan's 3 cups (a tipped pan scores 0
and ends the trial), and the towel holds it back from the arms. Wipe the bulb dry before each trial, and the
finger pads after any trial in which water reached them.

## Runtime and seeds

The scripted demo scores 1.0 and ends after 1,550 control steps (77.5 s at 20 Hz), when the 1 s hold after the
set-down is complete: it draws 31.5 mL, puts 29.6 mL on the roast's top and 0 mL on the counter. The layout is fixed
(`object_jitter=0`).

Scripted variants on the same task file, each scoring 0:

| Variant | What it does | Score |
|---|---|---|
| nothing | both arms stay at the ready pose | 0 (nothing drawn) |
| no squeeze | the demo without the squeeze before the dip | 0 (0.13 mL drawn, none on the roast) |
| air release | squeezes and lets go in the air, then dips | 0 (0.13 mL drawn, none on the roast) |
| wrong way | squeezes only once the tip is under the juices, lets go after lifting out | 0 (0.12 mL drawn, none on the roast) |
| near miss | the full demo, squeezed out 1.5 cm off the roast's near edge | 0 (31.5 mL drawn, all back in the pan) |

Dream-1 run (task.py 41772e25b144, the same instruction, object_jitter 0, 2,400 steps): 0 after 2,400 steps.

## Assets and licenses

Every file in this folder is the task authors' own code and text, released under the MIT License (`LICENSE`). Every
prop is built at start-up from shapes defined in the code; the folder holds no meshes, textures, images, fonts or data
files, and no text is drawn in the scene. The external assets below load at run time from the installed competition
environment and are not redistributed here.

| Asset | Source | License |
|---|---|---|
| YAM arms and parallel grippers: URDF and link meshes | I2RT Robotics, [i2rt @ 120c3c81](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058), as shipped in [yam-isaac-lab-arena @ bd62771e](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam) (`yam_arena/assets/yam/`) | [MIT](https://github.com/i2rt-robotics/i2rt/blob/120c3c81400171174604e503943f8d1ebc891058/LICENSE), Copyright (c) I2RT Robotics (`LICENSE.i2rt` in the environment) |
| YAM collision hulls (`collision/`) and robot USD (`usd/yam_arm`) | Hulls cut with CoACD from the I2RT link meshes by [YAM-inspect-Isaac](https://github.com/ttpierson/YAM-inspect-Isaac), shipped unchanged by the environment; the USD is converted from the same URDF with Isaac Lab's URDF importer. `yamgeom.py` reads the hulls at run time (`yam_arena.analysis.link_hulls()`) and stores nothing derived from them. | MIT (I2RT), as derived from the I2RT model. YAM-inspect-Isaac declares no license. |
| Black and white link 2/3 meshes (station arm colors) | I2RT meshes split by Dream Labs, from the dreamlabs 1.0.23 wheel ([SOURCE](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/dream/SOURCE)) | "MIT, see ../LICENSE.i2rt" (the environment's `dream/SOURCE`) |
| Station room: floor, table, camera stand, lights and the three cameras | Generated in code by the environment (no mesh or texture files). Camera and arm-mount poses follow the [yamlab @ ec0455d2](https://github.com/ARISE-Initiative/yamlab/tree/ec0455d2b4ce35f21fc126418ea5e74ac567133d) calibration. | Calibration: [MIT](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE), Copyright (c) 2026 Tianyuan Dai. The environment's README: "This repository has no license yet". |
| Simulator | [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) release/0.3.1 (c7edf3a9), Isaac Lab (91a0906f), Isaac Sim 6.1.0 | Apache License 2.0; BSD-3-Clause; Isaac Sim's package metadata: "NVIDIA Proprietary Software" |

Physical constants (water's density, viscosity and surface tension, the orifice coefficient, gravity) are from the
sources listed in `liquidkit.py`'s SOURCES. Product sizes and weights are from the makers' listings named above.

## Files

- `task.py`: the scene, the scorer and the scripted demo.
- `bulbkit.py`: the baster's suction law.
- `liquidkit.py`: water's constants with their sources, the orifice law and the ballistic jet.
- `squeezekit.py`, `capkit.py`, `yamkit.py`, `gripkit.py`, `yamgeom.py`: modeling helpers (sprung panels, convex
  hulls, articulated props, rubber finger pads, the arms' hulls for contact checks).
- `test_task.py`: offline checks, including the cheat tests.
- `LICENSE`: the MIT License for the files in this folder.
