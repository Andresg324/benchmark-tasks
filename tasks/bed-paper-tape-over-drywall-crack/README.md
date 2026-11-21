# Tape a drywall crack: lay paper joint tape centered along it and bed it in the wet compound with a joint knife

Two YAM arms tape a 12 cm crack in a half-inch drywall offcut that leans back 20 degrees in a bench stand. A bed of
wet joint compound (3 mm deep, 10 cm wide) is already laid along the crack. The arms lay an 18.8 cm strip of
2-1/16 in. paper joint tape centered on it, press it onto the wet bed, and bed it with a 4 in. joint knife, then set
the knife down. `task.py` is the task for the `yam_arena` embodiment of
[yam-isaac-lab-arena](https://github.com/kairosiann/yam-isaac-lab-arena); `sheetkit.py` (the tape), `bedkit.py` (the
compound, bedding and bond), `guardkit.py` (gripper contact), `yamgeom.py` (the arm links' collision hulls) and
`yamkit.py` (props, demo waypoints) are its helpers.

The tape makers' order: compound along the joint, the tape centered on it and embedded while wet, the excess taken
off with a broad knife "leaving about 1/32" of compound under the tape" (National Gypsum ProForm). The crack is short,
like those that "radiate from the corners of doors and windows" (Family Handyman). task.json gives every size and
position, and the real-world trial.

The offcut leans back so its face is within both arms' reach at bench height; this is not the same as a vertical
wall. The bed is laid first because the task begins at the taping step of the makers' order, and the strip is cut to
length beforehand, as for any short crack. The bed hides the crack, so pencil ticks on its line 2 cm past each end of
the bed show where it runs (as a finisher marks it before bedding).

## Requirements

`yam-isaac-lab-arena` at commit `bd62771e60320433893bd1d2df0fc79b5ddcd6d1`, installed with its `setup.sh` (Isaac
Lab-Arena `release/0.3.1`, `inspect-robots` 1.0.0, `inspect-robots-yam` 0.38.0), and `source $ARENA_DIR/yam.env`.

## Run and score a trial

`--scorer task_score` gives 1.0 when `success()` holds for 1 s, otherwise `score()` at the end of the trial. The
crack is checked at 13 stations, 1 cm apart. Each threshold is defined once in `task.py` or bedkit's `Law`.

| Score | Rule |
|---|---|
| 0.25 | The tape covers the crack at every station, and the held knife has been drawn over it, pressing, at half the stations or more. Tape set by hand alone scores 0. |
| 0.5 | And centered: its center line within 3 mm of the crack at every station. |
| 0.75 | And bedded at 12 of 13 stations (90%): 0.4-1.6 mm of compound under it on the crack and 18 mm to either side, compound squeezed out at both edges, bonded, and no part of either arm on it after it was bedded. |
| 1.0 | And no wrinkle (every strip over the crack flat, corners within 0.7 mm), the knife released on the bench at least 2 mm clear of the face, no part of either arm on the tape or within 2 mm of the knife, knife and tape still for 1 s. |
| cap 0.5 | Torn tape: the knife pressing with one corner (edge turned over 15 degrees out of the face) dragged 5 mm or more. |

"Part of an arm" is every link from the shoulder to the fingertips, checked against each link's collision hulls as
the simulator poses them. The knife's edge counts as on the tape up to 0.6 mm off a strip, and up to 1.0 mm once a
stroke has begun; a stroke ends after 3 control steps with no contact. Pressure is not read, so a pass with no force
counts in simulation; the real trial's 24 h adhesion and hump checks (task.json `scoring`) catch it. Doing nothing scores 0. Tape laid off the line can be lifted and laid again before it is
bedded.

Model run (Dream-1): the command below with `--policy dream`. Scripted demo (open loop):

1. The left hand pinches the tape's top end where it overhangs the offcut, carries it out in front of the panel with
   the fingers tipped down the crack's line, then tips them square to the face and moves in: the tape lays itself
   down the line, its top end held folded out 2 cm above the crack.
2. The right hand's closed fingertips press the tape onto the bed in dabs 1.5 cm apart (never sliding on the tape),
   from 6 cm below the crack's top end to past its bottom end.
3. The left lets go, dabs the top part with lighter presses about 2 cm apart, pressing along its own arm's give, and
   goes home; the wet bed's tack now holds the tape.
4. The right takes the knife by its handle, turns the blade up the tape at 35 degrees to the face, sets the edge 2 cm
   above the crack's top end and draws it down to 2.1 cm past the bottom end at 1.5 cm/s, then lays the knife back.

```bash
inspect-robots run --embodiment yam_arena --policy yam_scripted \
  -E task=tasks/bed-paper-tape-over-drywall-crack/task.py -E object_jitter=0 \
  -E control_hz=20 -E cam_width=320 -E cam_height=176 \
  --instruction "Tape the crack in the wall panel with the paper tape and the joint knife, then lay the knife back on the bench." \
  --scorer task_score --max-steps 2400 --store-frames --log-dir logs/tape-demo
```

Video: `inspect-robots video logs/tape-demo/*.json`. `DRYWALL_VARIANT=hand_only`, `bare_bed`, `knife_left` or `steep`
runs a cheat instead of the demo (rows marked "variant" below).

## Failure modes

| Attempt | Score |
|---|---|
| Nothing done, or the start state | 0 |
| Tape laid and pressed on by hand with no knife (variant `hand_only`), or dropped loosely on the crack | 0 |
| Knife drawn down the bare bed, tape left on the bench (variant `bare_bed`) | 0 |
| Knife pushed with no hand on it, or drawn beside the tape | 0 |
| Tape off the line (more than 3 mm) or turned | 0.25 at most |
| Steep pass (variant `steep`, 60 degrees) or a dry, thin bed: starved | 0.5 at most |
| Knife over part of the crack only | 0.5 at most |
| Corner of the blade dragged along the tape | 0.5 at most |
| Knife left on the wall in hand (variant `knife_left`), wrinkle, a hand, wrist or forearm left on the tape | 0.75 at most |
| Bedded, then peeled or slid off | that stretch no longer bedded |

`test_task.py` (offline, no simulator) builds each of these and checks its score.

## Results

| Run | What it does | Score |
|---|---|---|
| Scripted demo (the film) | The whole task: tape laid, dabbed, bedded with the knife, knife laid back | 1.0, at step 1,920 (96 s) |
| Scripted demo, two more full runs | The same | 1.0, 1.0 |
| `hand_only` | Tape laid and dabbed by hand, no knife | 0 |
| `bare_bed` | Knife drawn down the bare bed, tape left on the bench | 0 |
| `knife_left` | The whole task, but the knife is left on the wall in the hand | 0.75 |
| `steep` | The knife pass at 60 degrees; it shoved the tape 3.5-4.5 mm across, past the 3 mm centring limit | 0 |
| Dream-1 | The command above with `--policy dream` | 0: its hands went to the tape and the knife but carried neither to the panel; the tape stayed on its block |

## Real-world build

A pine 1x2 stop and a 2x2 brace screwed to the bench so the offcut's face leans back 20 degrees; the crack a 12 cm
utility-knife cut through the face paper; the bed struck off with two 1/8 in. hardboard rails 50 mm either side; the crack's line ticked with a carpenter's
pencil or marker 2 cm past each end of the bed. Ready-mixed all-purpose compound, 2-1/16 in. paper joint tape cut to
18.8 cm on a 2x4 offcut on a plywood scrap, a 4 in. flexible joint knife. Before every trial each of the four gripper
fingers gets a fresh disposable nitrile glove (size L). A YAM finger is about 6.8 cm wide, too wide for a glove's
finger, so the finger goes into the glove's palm, pad against the palm side, and the glove's fingers are folded back
over the finger's outer face and taped there with the cuff by two turns of painter's tape. Every press and grip on the
tape and the knife happens through the glove; after the trial the gloves come off, tape and all, and go in the bin. A plastic drop sheet lies under the stand,
the tape block and the knife's spot; the compound is water-based and wipes off with a damp sponge. The knife's blade
is wiped clean before it is laid back, so the handle the gripper takes stays clean. An arm link on bedded tape fails
the trial, so a wrist or forearm in the compound is not a routine way for the arm to get dirty. A low-cost setup;
reset takes a few minutes (a fresh offcut, a new bed and strip of tape, the knife's blade wiped and the knife laid
back, new gloves on the fingers).

## What is simplified

task.json's fidelity line lists it: the tape is 28 rigid hinged strips with the real width, weight and bending
stiffness; the compound is a 1 mm height field, not a fluid, and the knife's angle sets what it leaves (0.8 mm at 40
degrees or less, 0.1 mm at 60 or more: an assumed law, set to the makers' 1/32 in., not a measured one); a bedded strip's hold is a joint switched on by the knife and released by a
peel or a slide; a fingertip's press switches on a far weaker tack joint (an assumption; its 30 Pa shear cap is below
the 200-1000 Pa yield stress of wall (spackling and joint) compounds, US 11,267,758 B2), never scored.

## Runtime and seeds

- Run limit: 2,400 control steps at 20 Hz (120 s); the scripted demo scores 1.0 and ends at step 1,920 (96 s).
- No randomness: the layout is fixed (`-E object_jitter=0`). Layouts other than `object_jitter=0` are not supported
  (the tape can slide off its 2x4 at reset).

## Sources

- National Gypsum ProForm Paper Joint Tape technical information (2-1/16 in.; "Center joint tape over the joint and
  embed in the compound ... leaving about 1/32" of compound under the tape"),
  https://pdf.lowes.com/productdocuments/4fe0a129-9b36-4872-ae7b-7e120d63deb5/64424890.pdf
- USG Sheetrock Heavy Joint Tape submittal J1393 ("about 1/32" (0.7 mm) of compound under feathered edge"),
  https://images.homedepot-static.com/catalog/pdfImages/7a/7a2ad9f3-62c7-45ef-b58e-869ccdbca0c6.pdf
- USG ME Paper Joint Tape data sheet IF-TDS039-1 (52 mm x 76 m, about 550 g per roll),
  https://www.usgme.com/default/mwdownloads/download/link/id/334
- Toupret paper joint tape data sheet (0.215 mm, 138 g/m²; compound "to a depth of about 3mm (1/8")"),
  https://bo.toupret.com/storage/media/shares/FT_TOUPRET_BANDE_A_JOINT_PAPIER_MO.pdf
- US patent 11,267,758 B2, "Wall compounds and methods of use": yield stress of wall (spackling and joint) compounds
  "at least 200 and no greater than 1000 Pa", https://patents.google.com/patent/US11267758B2/en
- Fine Homebuilding, "Taping Fastener Heads and Seams: First Coat",
  https://www.finehomebuilding.com/project-guides/drywall/taping-fastener-heads-and-seams-first-coat
- Family Handyman, "How to Tape Drywall With a Banjo" ("Press the wet tape into joints by dragging your fingers over
  the tape"), https://www.familyhandyman.com/project/how-to-tape-drywall-with-a-banjo/
- Family Handyman, "How to Repair a Drywall Crack", https://www.familyhandyman.com/project/how-to-repair-a-drywall-crack/
- Paper's elastic modulus: K. Niskanen (ed.), Paper Physics, 2nd ed., Paperi ja Puu, 2008.

## License

All code in this folder was written for this task; there are no third-party data files. The robot model and its
collision hulls load at run time from yam-isaac-lab-arena (commit bd62771e) and are not redistributed.
