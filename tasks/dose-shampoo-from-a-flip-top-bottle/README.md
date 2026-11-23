# Dose shampoo from a flip-top bottle: flip the cap open, squeeze 1.5-4 g into a dish, snap it shut

`task.py` for the `yam_arena` embodiment, `station` scene. A half-used 12.6 fl oz Suave Professionals shampoo bottle
stands on the counter with its flip-top cap snapped shut, and a small dish stands beside it. The instruction:

> **"Put a coin-sized dose of shampoo in the dish, then close the bottle."**

One hand steadies the bottle while the other flips the lid open past its catch; the bottle is squeezed cap-down over
the dish, stood back, and the lid snapped shut. Files: `task.py`, `test_task.py` (offline scorer checks), kits
`yamkit.py` (props), `squeezekit.py` (bottle and flow), `yamgeom.py` (arm hulls), `gripkit.py` (pad friction), and
`LICENSE`.

## Why it matters

Dosing shampoo from a flip-top bottle is an everyday bathroom job; the dish stands in for the palm the dose goes
into, as when helping someone wash their hair. Mental Floss, citing dermatologist Dr. Debra
Jaliman: "a quarter-sized amount of shampoo is indeed appropriate"; Keune says a 1 euro coin's worth is sufficient for
short to medium-length hair. A coin-sized dollop,
about 24 mm across and domed 6 to 12 mm high, is 1.5 to 3.7 mL, so the full-credit band is 1.5 to 4.0 g (shampoo is
about 1.03 g/mL). Head & Shoulders' larger general guide is "about 10 milliliters of shampoo" (two teaspoons), so
4 to 15 g is too much for a coin-sized dose (0.25), and over 15 g, half again that guide, is a dumped dose (0).

What makes it hard: the flip cap is a real snap, and a lid left on its catch lies over the orifice (below 61 degrees
open), so the shampoo runs over the cap. Flipping the lid over upright takes up to 0.38 N sideways at the cap, more
than the 0.21 N that tips the free 234 g bottle there (23.7 cm up, a 43 mm deep base), so one hand steadies it. Held open
cap-down, the bottle does not run until it is squeezed: the cap's orifice opens only above a set pressure, which the
shampoo's weight never reaches. A light squeeze gives nothing, a firm one a dose that grows while it is held, and
too firm or too long overshoots, so the policy must squeeze with the walls, watch the pool grow and let go.

## Run and score a trial

| Score | Condition |
|---|---|
| 1.0 | 1.5 to 4.0 g in the dish, at most 0.3 g outside it, the lid clicked shut, the bottle and dish standing upright (within 15 degrees) on the counter, let go (every arm link at least 5 mm away in simulation), all still (under 1 cm/s) for 1 s |
| 0.5 | the dose and no drips, but not that finish: cap left open or resting on its catch, lid held down by a hand, bottle still held or not standing |
| 0.25 | 0.5 to 1.5 g or 4.0 to 15 g in the dish, or 0.5 to 15 g in the dish with more than 0.3 g outside it (on the counter, the dish's rim or outside, the cap, the lid or the bottle) |
| 0 | under 0.5 g or over 15 g in the dish: doing nothing (the dish starts empty), holding it open cap-down unsqueezed or only lightly squeezed (nothing comes out), squeezing with the cap shut, squeezing through a lid left on its catch (it all runs over the cap), a dumped dose, the dish knocked over |

**Real trial**, judged by eye from the overhead video (no scale or gauge), with a US quarter (24 mm) lying beside
the dish as a size reference. The dose is the dollop in the dish, its width read in the first overhead frame after the bottle last leaves the dish,
before it spreads (24 mm domed 6 mm is 1.5 g; a wider dollop
slumps lower, and 34 mm domed 8 mm is 4.0 g; by eye this is an approximation of the simulated band). Same tiers as the simulation:

- **1.0:** a dollop from a quarter's width to about 1.4 quarters' (24 to 34 mm), no drop outside the dish, and at the
  end the lid flush on the cap, the bottle and dish standing upright on the pad, both hands off and clear of them,
  nothing moving over the last second.
- **0.5:** that dollop with no drop outside, but the lid open or on its catch (it then stands visibly open), a hand
  still on the lid or bottle, the bottle not standing, or something still moving at the end.
- **0.25:** a dollop at least a dime's width (18 mm, about 0.6 g) and not across most of the dish's floor that is
  narrower than a quarter or wider than 34 mm, or has any drop on the pad, the dish's rim or outside, the cap, the lid
  or the bottle.
- **0:** nothing in the dish or less than a dime's width, shampoo across most of the dish's floor (a dumped dose, over
  15 g), or the dish knocked over. Doing nothing scores 0.

Scripted demo:

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/dose-shampoo-from-a-flip-top-bottle/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Put a coin-sized dose of shampoo in the dish, then close the bottle." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/shampoo-demo
```

Model run (Dream-1): the same command with `--policy dream --max-steps 2400`.
Video: `inspect-robots video logs/shampoo-demo/*.json`.

The demo, open loop for the fixed layout:

1. The left hand lightly grips the standing bottle across its broad walls.
2. The right hand's fingertips lift the thumb tab along the lid's arc past upright; the hinge flips it open.
3. The left rolls the bottle 118 degrees cap-down over the dish (nothing comes out), waits 2.5 s, squeezes 5.5 mm
   harder for 2.5 s, eases off and stands it back.
4. The left lets go; the right pushes the lid back over onto its catch, then presses it past the catch.

`SHAMPOO_VARIANT=<name>` runs a cheat or near-miss instead. Simulated: demo 1.0 (2.64 g); `wait` (45 s open
cap-down, no squeeze, then shut) 0 (0 g); `weak` (a light squeeze held 5 s) 0 (0 g); `open` (cap left open) 0.5;
`held` (lid on its catch, hand on it) 0.5; `closed` (squeezed shut) 0; `dump` (walls fully in for 10 s) 0 (15.5 g);
`nothing` 0.

## The shampoo and the cap

- **Bottle and walls.** The two broad walls are panels on sprung slides (1.5 N/mm each), their push read from their
  joints; holding the base or shoulder squeezes nothing. Each mm of push displaces 1.5 mL and compresses the 200 mL
  of air inside. The body is drawn as a rounded oval; its box colliders reach up to 7 mm past the drawn corners
  (the hands touch only the broad walls and the cap).
- **Flow.** Shampoo leaves the 4 mm orifice while the lid is unclicked, the orifice points within 70 degrees of
  down (after 2 s to run into the cap) and the pressure there, squeeze plus the shampoo's own weight, exceeds 3.06
  kPa: 12.3 inches of water, the opening pressure of the silicone slit valve in US patent 4,728,006, "particularly
  suitable as a dispenser for shampoos", whose package can be "stored inverted". The shampoo's weight gives at most 1.16 kPa (straight down), so the open
  bottle never drips unsqueezed. Each real bottle is checked to hold cap-down unsqueezed for 60 s before first use
  (Real-world build); the opening pressure is a model choice from the patent. Above it shampoo flows at 0.5 mL/s per kPa;
  Poiseuille at 8 Pa s (7.5 to 12 Pa s at 1/s, Thermo Fisher) through a 4 mm, 4 mm-long orifice gives 0.2, and
  shampoo thins several fold at the orifice's shear rates, hence 0.5 (a model choice). As it leaves, the air
  expands, so a held squeeze's flow dies away; let go, the pressure falls below the opening pressure and air is
  drawn back in (0.05 mL/s per kPa of vacuum, a model choice). Below 61 degrees open (where the tab end starts to
  uncover the orifice) it all runs onto the cap, outside the dish; past that it falls plumb
  into the dish or outside it.
- **The lid.** A rigid lid cannot flex over a bead, so the snap is an angle-only spring on the hinge: within 2.9
  degrees it is held shut (0.05 N m, 1.3 N at the tab, to pull it past the catch); from there to 11.5 degrees it is
  pushed open onto the catch (0.04 N m, 2.1 N at the lid's middle, to press it back); the living hinge then pulls it
  back toward the catch below upright and opens it to its 115 degree stop above (over-center action, as in US patent
  5,070,599). Contact alone moves it. Clicked = under 2.9 degrees. The open stop and snap torques are model
  choices from the cap's photos.

## Real-world build

- **Bottle:** Suave Professionals Rose Oil Infusion Volumizing Shampoo, 12.6 fl oz (the listing gives 2.812 x 1.687 x
  8.375 in), the same product and scent for every trial. Its cap is a snap flip-top, as the maker's photos show (cap
  sizes from them). Fill: halfway, to a pen line on the label (200 mL). Before first use: opened and held cap-down
  over a cup for 60 s unsqueezed, nothing comes out (as in the simulation); else use another bottle.
- **Conditions.** Room at 20 to 24 C (shampoo thickens when cold). After each reset the bottle stands upright, lid
  shut, for 5 minutes before the trial. After a top-up, 30 minutes.
- **Dish:** any round porcelain dip bowl about 3.5 in across and 1 in tall.
- **Gripper protection (shared rig).** Each gripper finger wears a nitrile glove (size L), its loose fingers folded
  back and taped clear of the pads, fingertips and wrist camera; a glove that tears or gets shampoo on it is replaced.
- **Containment.** The bottle and dish stand on a plastic-backed absorbent bench pad (about 50 x 60 cm), taped flat;
  replace it after any spill. Reset: wash and dry the dish, wipe the orifice
  and lid with a damp, then a dry paper towel, snap the lid shut, set both on their tape marks; once the shampoo is a finger's width under the pen line,
  top it up to the line from the same product. About 3 minutes, then the 5 minute rest. Each trial uses about 2 to 3 g
  of shampoo and a paper towel; nothing else is consumed.
- **Safety:** a shampoo bottle and a dish; nothing sharp, hot or heavy.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (2 minutes). Demo: 1.0 in simulation; the recorded run stopped at
  success after 1,079 steps (a 54 s video).
- Dream-1: 0 in 2,400 steps: the arms drew back toward their bases and never reached the bottle or the dish; the cap stayed shut and the dish empty.
- No randomness in the scripted demo: the layout is fixed (`-E object_jitter=0`).

## Sources

- Suave Professionals 12.6 fl oz shampoo listing (size): https://www.target.com/p/suave-professionals-shampoo-rose-oil-infusion-12-6-oz/-/A-1001712691
- "Quarter-sized amount": https://www.mentalfloss.com/article/83949/truth-behind-8-recommended-product-usage-amounts
- Coin-sized amounts: https://www.keune.com/faq-products/shampoo/how-many-ml-shampoo-per-wash/
- About 10 mL, two teaspoons: https://headandshoulders.com/en-us/healthy-hair-and-scalp/hair-care/how-much-shampoo-should-you-use
- Dispensing valve opening pressure, inverted storage: US patent 4,728,006, https://patents.google.com/patent/US4728006A/en
- Shampoo viscosity: https://www.thermofisher.com/blog/materials/which-shampoo-is-right-for-you-testing-polymers-to-find-out
- Flip-cap over-center hinge: US patent 5,070,599, https://patents.google.com/patent/US5070599A/en

## Assets and licenses

Every file in this folder is the task authors' own code and text, released under the MIT License (`LICENSE`). Every prop is built
at start-up from shapes defined in the code; the folder holds no meshes, textures, images, fonts or data files. The
external assets below load at run time from the installed competition environment and are not redistributed here.

| Asset | Source | License |
|---|---|---|
| YAM arms and parallel grippers: URDF and link meshes | I2RT Robotics, [i2rt @ 120c3c81](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058), as shipped in [yam-isaac-lab-arena @ bd62771e](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam) (`yam_arena/assets/yam/`) | [MIT License](https://github.com/i2rt-robotics/i2rt/blob/120c3c81400171174604e503943f8d1ebc891058/LICENSE), Copyright (c) I2RT Robotics (`LICENSE.i2rt` in the environment) |
| YAM collision hulls (`collision/`) and robot USD (`usd/yam_arm`) | Hulls cut with CoACD from the I2RT link meshes by [YAM-inspect-Isaac](https://github.com/ttpierson/YAM-inspect-Isaac), shipped unchanged by the environment; the USD is converted from the same URDF with Isaac Lab's URDF importer. `yamgeom.py` reads the hulls at run time (`yam_arena.analysis.link_hulls()`) and stores nothing derived from them. | MIT License (I2RT), as derived from the I2RT model. YAM-inspect-Isaac declares no license. |
| Black and white link 2 and 3 meshes (station arm colors) | I2RT meshes split by Dream Labs, from the dreamlabs 1.0.23 wheel ([SOURCE](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/dream/SOURCE)) | MIT License, per the environment's `dream/SOURCE`, which points to `../LICENSE.i2rt` |
| Station room: floor, table, camera stand, lights and the three cameras | Generated in code by the environment (no mesh or texture files). Camera and arm-mount poses follow the [yamlab @ ec0455d2](https://github.com/ARISE-Initiative/yamlab/tree/ec0455d2b4ce35f21fc126418ea5e74ac567133d) calibration. | Calibration: [MIT License](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE), Copyright (c) 2026 Tianyuan Dai. The environment's README: "This repository has no license yet". |
| Simulator | [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) release/0.3.1 (c7edf3a9), Isaac Lab (91a0906f), Isaac Sim 6.1.0 | Apache License 2.0; BSD-3-Clause; Isaac Sim's package metadata: "NVIDIA Proprietary Software" |
