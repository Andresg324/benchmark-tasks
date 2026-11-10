# pallet-jack-forks-lowered

A loaded pallet sits on the lowered forks of a manual (hand) pallet jack. The job is to move it into a taped bay. A
trained material mover pumps the handle to raise the forks until the pallet rides clear of the floor, rolls the jack
into the bay, then holds the release pedal until the forks are fully down and leaves the handle standing upright.
Leaving the forks raised under the parked load, or pushing the jack with the forks down so the pallet is dragged, is
the wrong answer. 1:8 bench-scale model.

## Spec

Scene (1:8 scale, table top = warehouse floor; cm from the midpoint between the arm bases, x forward, y left):

| Object | Model | Position |
| --- | --- | --- |
| Manual pallet jack | red steel, about 175 g (90 kg). Forks 14.4 cm long (1,150 mm), 2 cm wide, 8.6 cm overall (685 mm), tops 1.06 cm up lowered (85 mm), 1.44 cm lift (115 mm). Pump unit on two 2.25 cm steer wheels (180 mm); a 1 cm load roller (80 mm) under each fork tip. Hinged loop handle, grip bar 15.2 cm above the floor (1,220 mm). Red foot release pedal on the pump unit's robot side. A green lift flag rises out of the fork frame's far end with the forks, 2 cm above it at full lift (a display-only marker for the overhead camera) | handle pivot at x 28, y 21; forks point to -y |
| Loaded pallet | single-deck stringer pallet 15 x 12.5 cm (1,200 x 1,000 mm), three 1.43 cm stringers, 1.67 cm tall; a block of brown cartons 13 x 10.5 x 6 cm on it; 300 g in all (about 150 kg) | on the forks, center x 28, y 9.8 |
| Bay | yellow floor tape 1 cm wide, 15.5 cm (along x) by 18 cm (along y) inside | center x 28, y -7.2 |

Instruction: "Move the loaded pallet into the taped bay with the pallet jack."

Unstated rule: raise the forks (pump strokes) so the pallet clears the floor before rolling; once in the bay, hold
the release until the forks are fully down; leave the handle upright and let go.

| Outcome | Score |
| --- | --- |
| Pallet dragged on the floor: more than 1.8 cm before the first lift, or more than 5 mm of set-down slides in all after it (irreversible) | 0 |
| Pallet tipped past 15 degrees (2 steps) | 0 |
| Nothing done, only pumping, or the pallet never carried clear 3 cm closer to the bay | 0 |
| Carried, but not fully inside the bay at the end | 0.25 |
| In the bay with the forks not fully down (the trap) | 0.25 |
| In the bay, forks down, but the handle not upright, the pedal or jack still held, or things moving | 0.5 |
| Done, an arm not home | 0.9 |
| Done, both arms home | 1.0 |

Thresholds and why:
- Clear of the floor: the stringers' lowest point 1 mm up (8 mm): the whole load is on the forks, none on the floor.
  Two pump strokes leave the pallet 2.4 mm up after the loaded forks' 1.1 mm sag, a real (if low) carry.
- Dragged, before the first lift (carried clear 1 cm toward the bay): the footprint 1.8 cm (14 cm) from its start.
  Each pump stroke pulls the jack back toward the operator and the pallet rocks with it: 1.3 cm in the scripted demo
  (`PALLET_JACK_DEBUG=1`), plus a 5 mm margin.
- Dragged, after the first lift: each set-down's farthest slide from where it landed, summed over the run and never
  reset by lifting again, at most 5 mm (4 cm). So carrying short, lowering and pushing the rest of the way scores 0.
- Moved: carried clear 3 cm closer to the bay (along the line from its start to the bay's center), so pumping and
  lowering in place, or carrying it the wrong way, earns nothing.
- In the bay: every corner of the pallet's footprint inside the middle of the 1 cm tape (the line is the limit).
  The bay leaves 1.5 cm (12 cm) of room on each side.
- Fully down: the pump's level 0 in the kit's memory, the fork joint within 1.5 mm (12 mm) of the bottom, and the
  pallet resting on its stringers (lowest corner within 2 mm of the floor, flat within 3 degrees). 1.5 mm is above
  the 1.1 mm the loaded forks sag under the drive.
- Handle upright: within 7 degrees of vertical, the zone its return spring holds; the pedal back within 6 degrees of
  rest (let go).
- Released: no gripper with closed fingers within 3 cm of the jack's or the pallet's outline. Still: under 1 mm per
  step. Arms home: within 3 cm of their start pose, still.

## Run and score

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/pallet-jack-forks-lowered/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Move the loaded pallet into the taped bay with the pallet jack." \
      --scorer task_score --max-steps 2400 --store-frames --log-dir ~/yam-eval

Use `--policy dream` for Dream-1. The scripted demo finishes (score 1.0, success held 1 s) in 660 steps (33 s); Dream-1 scored 0 in 2,400 steps (it hovered at the handle and nudged it). A full trial is 2,400 steps
(2 minutes at 20 Hz), about 15 minutes of wall time on an L40S. No random seeds: `object_jitter=0`, one fixed
layout. `python test_task.py` checks the scorer offline (prints "all pass"). `PALLET_JACK_DEBUG=1` prints the
scorer's view every step.

Files: `task.py` (scene, scorer, demo), `yamkit.py` (modeled props and mechanisms; local copy of the master with the
hand-truck task's sphere colliders and physics materials merged in), `mechkit.py` (joint laws: Pump, Detents,
Spring, Free), `test_task.py`.

## Real-world build

- A 1:8 model hand pallet jack (hobby/diorama scale models of hand pallet trucks exist; or build one from 3 mm
  steel or 3D-printed parts): forks on a frame that a small lever or screw lifts 14 mm when the handle is worked, a
  spring-returned release pedal that lets the forks drop, two caster wheels and two small rollers. A real trial
  can also use a full-size jack and pallet with a full-size robot; the rule and the checks are the same.
- A 150 x 125 mm wooden stringer pallet (craft-wood strips) with a 300 g block of cartons glued on it.
- 10 mm yellow tape on the table for the bay.

Checking a real trial: a 1 mm lift is a few pixels from an overhead camera, so film from the side at floor level (or
slide a 1 mm feeler gauge under a stringer) to check the pallet is clear before the jack rolls, and mark the pallet's
start on the table to measure any slide; then measure each pallet corner to the tape line, check the forks are fully down (on their
bottom stops, with the pallet resting on the table on its stringers), check the handle stands within 7 degrees of vertical, and that the arms are back home.
