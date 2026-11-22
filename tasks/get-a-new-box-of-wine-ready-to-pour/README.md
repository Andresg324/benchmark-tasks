# Get a new box of wine ready to pour

A new, full 3 L box of wine (a Black Box 3 L tower carton) stands on the counter. Its tap is tucked inside behind a
perforated tab near the bottom of the front panel. The instruction is only "Get the box of wine ready to pour."

Setting up a bag-in-box is the same on every brand: push the tab in until it tears, reach into the opening, draw the
tap out on its bag, and set its neck down into the die-cut slot under the opening. The panel then sits between the
tap's two flanges, and the tap points out, outlet down.

The demo uses both hands, as people do: the left hand lies flat on the carton's top and presses it down while the
right tears the tab, draws the tap out and seats it.

The hard part is physical. The opening is only 6 cm wider than the tap, so the fingers go in either side of it with a
few millimeters to spare. The tap is tethered to the bag, and its 7 mm groove has to meet the 3 mm panel before the
neck can drop into the 24 mm slot. Nothing snaps it into place: the slot, the flanges and gravity hold it.

## Files

- `task.py`: the scene, the scorer and the scripted demo.
- `yamkit.py`: builds the carton, the tab and the tap's chain, and reads object poses.
- `gripkit.py`: gives the YAM's finger pads their rubber friction.
- `yamgeom.py`: the robot's collision hulls, used for the hands-clear check.
- `test_task.py`: offline checks of the scorer, including each cheat in the failure table.

## Sources

- Black Box 3 L: a 5 x 5 x 9.5 in "tower" carton (Packaging World, "Boxed wine aims high",
  https://www.packworld.com/trends/contract-packaging/news/13337830/boxed-wine-aims-high).
- Vitop tap (Smurfit Kappa product sheet RD EN 056.2, amkFa/VT): tap 10.2 g, spout 6.1 g; PP, HDPE and elastomer,
  LDPE spout.
- Setup steps: Gratsi's "How to open your new Gratsi tap" (punch through the perforated circle, pull the spigot
  forward onto the cutout), the same sequence as other bag-in-box wines.

The opening, the slot, the tap's sizes and the tab's tear force are not published. They are estimated from maker
photos. The full box's mass is 3 L of wine plus the bag, the tap and the carton: 3.25 kg.

## The model

- **Carton.** 12.7 x 12.7 x 24.1 cm, standing free on the counter: 3.25 kg (3 L of wine plus the bag, the tap and
  the carton), board friction 0.45 / 0.40. It slides at about 14 N and tips at about 16 N pushed in at the tab's
  height, so a careless push or pull can shove it or knock it over. The full bag is rigid and fills the carton,
  except for a pocket behind the opening where the tap lies.
- **Tab.** A 3.1 g plate filling the 9.6 x 7.6 cm opening, with a 1.5 mm cut all round. Its perforations are a
  stiff spring: it gives 2 mm under an 8 N push, and at 8 N they tear for good. Until then they also hold it square
  (at most about 2 degrees of tilt). Torn, the tab slides in freely, drops into the box and can tip back about its
  bottom edge (up to 80 degrees, not outward into the panel).
- **Tap.** A rigid 16.3 g body: body, outlet, two flanges 7 mm apart on a 20 mm neck, and the bag's spout behind.
  Simple shapes: the flanges are 30 mm squares (the real ones are round), the body a box. Plastic friction 0.35 /
  0.30, estimated.
- **Bag's film.** A chain of three slides and two hinges (pitch, and roll about the tap's axis) joins the tap to the
  bag. Its limits are the film's reach: 2 cm further out than seated, 3.5 cm to either side, 8 cm down, 3 cm up,
  pitch 34 degrees. It has no yaw joint (seated, the flanges on the panel hold the tap square anyway).
  Each joint carries 2 g of film. Nothing pulls the tap anywhere: it moves by contact and gravity.
- **Slot.** A 24 mm notch under the opening, its bottom 5.4 cm above the counter. The board is 3 mm thick.
- No liquid.

## Scoring

| Score | Outcome |
|---|---|
| 1.0 | The tap's neck rests down in the slot (not caught on its edges; in the simulator, within 3 mm of the slot's bottom) with the panel between its flanges. The tap is level within 15 degrees, points out within 30 degrees, and its outlet is down (its up within 30 degrees of the carton's). The tap is let go (fingers open and the hand drawn back, not touching it; in the simulator, no part of either arm, from the upper arm to the fingertips, within 5 mm of it). The carton was never knocked over, stands upright on the counter (leaning at most 10 degrees, not held up), and its footprint is within 5 cm of its tape mark. All is still for 1 s. |
| 0.5 | The tap is out through the opening but not seated so: hanging outside, upside down, or still held. Or it is seated, but the carton was shoved more than 5 cm off its mark, or is held tilted or off the counter. |
| 0.25 | The tab is torn, the tap still inside. |
| 0 | Nothing torn, or the carton was knocked over (leaned more than 45 degrees) at any time. |

A judge watching a real trial applies the same rules. The 5 cm rule is there because the carton should be steadied,
not dragged or pushed across the counter while the tap is worked; knocked over means the box fell onto its side or
back (a full box of wine falling over is a failed setup).

## Failure table (demo variants, `WINEBOX_VARIANT`)

| Variant | What the robot does | Score |
|---|---|---|
| `nothing` | Both arms stay at the ready pose. | 0 |
| `dangling` | Draws the tap out past the slot and lets go; it hangs outside. | 0.5 |
| `upside` | Turns the tap over, then seats it. | 0.5 |
| `held` | Seats the tap but never lets go. | 0.5 |
| `nosteady` | The whole setup with the right hand only; nothing holds the carton. | 1.0 (the 8 N tear push does not shove a 3.25 kg box) |
| (default) | The whole setup, the left hand pressing the carton down. | 1.0 (557 steps; the carton moved under 1 mm) |

Dream-1 (`--policy dream`, 2,400 steps): 0. Its arms moved, but it never tore the tab; the carton stayed upright.

## Runtime and seeds

The layout is fixed (`object_jitter=0`), with no random seed; the demo is open loop for that layout. A model run is
limited to 2,400 steps (2 minutes at 20 Hz).

## Run a trial

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/get-a-new-box-of-wine-ready-to-pour/task.py -E object_jitter=0 -E control_hz=20 \
  -E cam_width=320 -E cam_height=176 --instruction "Get the box of wine ready to pour." --scorer task_score \
  --max-steps 1100
```

Set `WINEBOX_VARIANT=nothing|dangling|upside|held|nosteady` for a variant. For a model run, use `--policy dream
--max-steps 2400`. Offline checks: `python -m pytest test_task.py`.

## Real-world build

A low-cost setup: new 3 L boxes of wine, one per scored trial (the perforations tear only once; the wine is kept), and
masking tape to mark the carton's footprint on the counter (shoved = its footprint ends more than 5 cm off the mark).
Reset: a new box.

## Assets and licenses

Every file in this folder is the task authors' own code and text, released under the MIT License. Every prop is built
at start-up from shapes defined in the code; the folder holds no meshes, textures, images, fonts or data files. The
external assets below load at run time from the installed competition environment and are not redistributed here.

| Asset | Source | License |
|---|---|---|
| YAM arms and parallel grippers: URDF and link meshes | I2RT Robotics, [i2rt @ 120c3c81](https://github.com/i2rt-robotics/i2rt/tree/120c3c81400171174604e503943f8d1ebc891058), as shipped in [yam-isaac-lab-arena @ bd62771e](https://github.com/kairosiann/yam-isaac-lab-arena/tree/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam) (`yam_arena/assets/yam/`) | [MIT License](https://github.com/i2rt-robotics/i2rt/blob/120c3c81400171174604e503943f8d1ebc891058/LICENSE), Copyright (c) I2RT Robotics (`LICENSE.i2rt` in the environment) |
| YAM collision hulls (`collision/`) and robot USD (`usd/yam_arm`) | Hulls cut with CoACD from the I2RT link meshes by [YAM-inspect-Isaac](https://github.com/ttpierson/YAM-inspect-Isaac), shipped unchanged by the environment; the USD is converted from the same URDF with Isaac Lab's URDF importer. `yamgeom.py` reads the hulls at run time (`yam_arena.analysis.link_hulls()`) and stores nothing derived from them. | MIT License (I2RT), as derived from the I2RT model. YAM-inspect-Isaac declares no license. |
| Black and white link 2/3 meshes (station arm colors) | I2RT meshes split by Dream Labs, from the dreamlabs 1.0.23 wheel ([SOURCE](https://github.com/kairosiann/yam-isaac-lab-arena/blob/bd62771e60320433893bd1d2df0fc79b5ddcd6d1/yam_arena/assets/yam/dream/SOURCE)) | MIT License: the environment's `dream/SOURCE` points to `../LICENSE.i2rt` (the I2RT MIT License) |
| Station room: floor, table, camera stand, lights and the three cameras | Generated in code by the environment (no mesh or texture files). Camera and arm-mount poses follow the [yamlab @ ec0455d2](https://github.com/ARISE-Initiative/yamlab/tree/ec0455d2b4ce35f21fc126418ea5e74ac567133d) calibration. | Calibration: [MIT License](https://github.com/ARISE-Initiative/yamlab/blob/ec0455d2b4ce35f21fc126418ea5e74ac567133d/LICENSE), Copyright (c) 2026 Tianyuan Dai. The environment's README: "This repository has no license yet". |
| Simulator | [Isaac Lab-Arena](https://github.com/isaac-sim/IsaacLab-Arena/blob/c7edf3a9d5d8276565208f72422f8f8a0806d6cf/LICENSE.md) release/0.3.1 (c7edf3a9), Isaac Lab (91a0906f), Isaac Sim 6.1.0 | Apache License 2.0; BSD-3-Clause; Isaac Sim's package metadata: "NVIDIA Proprietary Software" |
