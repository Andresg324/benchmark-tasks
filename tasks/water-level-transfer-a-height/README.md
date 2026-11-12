# Carry a height to a far post with a reservoir water level

A reservoir water level: a clear 1-liter jar of water with a carry handle on its lid, a clear end tube taped to a
wooden stick, and the clear hose between them coiled on the table, the jar and the end each standing in a stand. "Use
the water level to mark the height of the datum post's black line on the far post with the red marker, then put the
jar and the hose end back in their stands." The water in the jar and in the end tube always stands at one height, so
the end is set at the far post, the jar is held beside the datum post and raised or lowered until its water sits on the
line, and the marker is slid to the end tube's water surface (Fine Homebuilding, "Making and Using Water Levels", 1994;
JLC, "Making and Using a Water Level"). The jar's cross-section is 55 times the end tube's, so the water follows the
jar almost 1:1 and barely moves when the end tube moves (4 cm moves it 0.7 mm). The far post stands on a pad 2.5 cm
higher than the table, so marking the line's height above the far post's floor puts the mark 2.5 cm too high.

Run (from the repo root, Isaac Lab-Arena installed):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/water-level-transfer-a-height/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Use the water level to mark the height of the datum post's black line on the far post with the red marker, then put the jar and the hose end back in their stands." \
      --scorer task_score --max-steps 900
    python tasks/water-level-transfer-a-height/test_task.py      # offline model and scorer checks, prints "all pass"

Use `--policy dream --max-steps RUN_LIMIT` for a Dream-1 trial. The scene is deterministic (no seeds, object_jitter=0).

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Jar (the reservoir) | a clear round plastic jar 10 cm across and 10 cm tall (9.4 cm inside), a vented clear lid with a black carry handle across it (a bar 5 x 2.5 cm, 3 cm tall), the hose leaving through a fitting in its bottom; 150 g in the sim. Water 6 cm deep at the start | standing in its stand, centered x 22, y 14 |
| Hose end | a pine stick 2.5 x 2.5 x 15 cm on a 4 x 4 x 0.8 cm pine foot, with a clear acrylic tube (1.6 cm outside, 1.27 cm bore, 9 cm long, from 3 to 12 cm above the stick's bottom) taped to its front face; 80 g | standing in its stand, centered x 22, y -14 |
| Hose | 1.2 m of 3/8 inch (9.5 mm) bore clear vinyl from the jar's bottom to the end tube's bottom, coiled on the table (three loops 8 to 12 cm across); drawn, no collider | coil centered x 22, y 0 |
| Stands | two plywood trays on the table, each a 1 cm rim (4 mm thick): the jar's 12 x 12 cm inside, the end's 8 x 8 cm; fixed | centered x 22, y 14 and y -14 |
| Datum post | pine, 4 x 4 cm, 18 cm tall, a black datum line 2 mm wide painted all round at z 13.0; fixed | centered x 30, y 23.5 |
| Far post | pine, 4 x 4 cm, standing on a gray pad 12 x 12 x 2.5 cm (the floor there is 2.5 cm higher), top at z 18; fixed. A plywood shelf on its inboard (+y) face, floor at z 4 with a 1 cm rim on three sides, 5.2 (x) x 5.6 (y) cm inside | centered x 30, y -23.5 |
| Red marker | a sliding marker on the far post's front face, held where it is left by friction: a collar plate 4.6 x 1.5 cm, a grip tab 4.5 (x) x 2.4 (y) x 2 cm sticking out toward the robot from its outboard half, and a thin pointer reaching across in front of the end tube when the end stands on the shelf. The pointer's top edge is the mark | mark at z 9.2 (3.8 cm below the datum height) |

Model (`levelhose.py`): the water is not simulated. The level holds a fixed volume of water; every control step the
one surface height that holds that volume is found from the jar's and the end's poses (jar and tube cross-sections,
plus the whole hose whenever the water stands above both bottoms).

| Quantity | Value | Why |
|---|---|---|
| Fill | the water 6 cm above the table with both in their stands | 2.2 cm under the end tube's bottom on the shelf when the jar is down, so the end reads only once the jar is up |
| Jar vs. end tube | 69.4 cm2 against 1.27 cm2 inside (55 to 1) | raising the jar 5 mm raises the water 4.9 mm; raising the end tube 4 cm (stand to shelf) raises it 0.65 mm |
| Holding height | the jar's bottom 7.05 cm above the table puts its water on the line with the end on the shelf | the shelf is 4 cm up, so the end's mouth stands 3 cm above the line |
| Spill | a mouth below the water overflows at 0.6 x its area x sqrt(2 g x depth) (Torricelli) per step, lost for good, drawn as a stream and a puddle | the end tube in its stand has its mouth at 12 cm: with the jar on the 13 cm line it overflows (over 50 mL), so the end goes up to the shelf before the jar goes up, and back down after the jar is down |
| Re-leveling | instant | a short hose with a wide reservoir settles in about a second |

Visuals (markers, no physics, never scored): the blue water in the jar and in the end tube up to its surface, the
hose from each to the coil, a stream while a mouth spills and a puddle growing with the water lost. They are drawn
before the first frame.

## Scoring

The marker counts as moved once its edge is more than 0.5 mm from where it was last counted; at each such step the
scorer records whether that step is a transfer, and keeps the record of the last one. A transfer is a step at which:
the jar stands within 3 cm of its spot 1 cm off the datum post, within 15 degrees of upright, gripped by its handle by
a hand that is not on the marker, with its water within 1.5 mm of the datum line; the end's tube stands within 2.5 cm of
its spot behind the marker's pointer, within 15 degrees of upright; and the marker's edge is within 1.5 mm of the end's
water surface.

| Outcome | Score |
|---|---|
| More than 1 mL spilled, or the hose pulled taut, at any point | 0 |
| The jar or the end not back in its stand (within 2.5 cm of its center, its bottom on the table), leaning over 10 degrees, moving, or with a tool point or pad tip within 1 cm; the marker still held | 0 |
| The marker never moved, or its last move was not a transfer | 0 |
| The mark more than 1.5 mm from the datum height | 0 |
| The mark within 1.5 mm, all put back and let go, both arms home and still | 1.0 |
| The same, arms not home and still (each tool point within 3 cm of where it started) | 0.9 |

Possible scores: 0, 0.9, 1.0. Doing nothing scores 0.

Why the numbers: 1.5 mm is a sixteenth of an inch, carpentry tolerance for a ledger or a cabinet line carried around a room. The
jar's water within 1.5 mm of the line is a meniscus read against a 2 mm pencil line through a clear wall; the marker's
edge within 1.5 mm of the end's water is the same reading at the far end. 0.5 mm of marker travel is a move a real
marker would show. 1 mL is about 20 drops: a careful transfer spills none, and an end left in its stand under a raised
jar spills over 50 mL. The 3 cm and 2.5 cm spots keep each surface right beside the line or the pointer it is read
against.

Why it takes two hands: the jar has nowhere to stand at the line's height (the datum post has no shelf, and the water
follows the jar 1:1), so one hand holds it there while the other, 45 cm away at the far post, slides the marker, and
the scorer counts the marker's last move only while a hand that is not on the marker grips the jar's handle with its
water on the line. One hand alone can only set the marker with the jar parked in its stand (the water 7 cm low) or
from memory alone (no live reading: 0).

The demo: the right hand takes the end by its stick's top, lifts it out of its stand onto the far post's shelf and lets
go, then closes on the marker's tab; the left takes the jar by its handle, lifts it out of its stand and holds it beside
the datum post with its water on the line; the right slides the marker up to the end's water surface, holds, lets go
and backs off; the left sets the jar back in its stand; the right takes the end off the shelf and back into its stand;
both arms home. In the sim: 1.0 in 783 steps; the jar's water holds within 0.1 mm of the line while the
marker moves, the mark ends 0.24 mm below the line, nothing spills.

Offline (test_task.py): doing nothing, the marker 2.5 cm high (the line's height above the pad), the jar 1 cm
off the line (marker to the water or to the datum height), the jar 1.2 mm high with the marker 1 mm over the water,
the end let down into its stand while the jar is up, the end put back before the jar, one gripper (jar parked, marker
to the datum height or to the water), the jar left at the line without a hand, the jar 10 cm off its spot, the end 8
cm from the far post, the marker nudged or flicked after the jar is down, open hands and empty fists left on the
marker or the jar, the end 3 cm off its stand and the jar leaning all score 0; the job done (1.0), arms not home (0.9)
and a 0.15 s marker slide (1.0) pass.

In a real trial: measure the mark and the datum line from a laser level line or a builder's level sighted between the
posts, to 0.5 mm with a steel rule; weigh the jar and the end with the hose before and after (1 g = 1 mL) and check
the table with a towel for spilled water; watch the video for the jar's water against the line while the marker was
last moved, and which hand held the jar.

## Real-world build

- A reservoir water level: a clear 1-liter round plastic jar with a vented lid and a carry handle, a bulkhead hose
  fitting in its bottom, 1.2 m of 3/8 inch bore clear vinyl tubing, and a 9 cm clear acrylic tube with a 1/2 inch bore
  taped to a 2.5 x 2.5 x 15 cm pine stick on a 4 x 4 cm foot (reservoir-type water levels are sold as a jug or
  bottle on a long clear hose with a clear end tube). Filled with water tinted blue with a drop of food coloring,
  without bubbles, to 6 cm deep with the jar and the end side by side on the table.
- Two plywood trays with a 1 cm rim for the jar and the end; a 4 x 4 cm pine datum post with a black line at 13.0 cm; a
  second post on a 2.5 cm pad, with a small rimmed shelf screwed to it at 4 cm and a red sliding marker (a stiff
  plastic clip that grips the post by friction, with a tab and a pointer).
- A steel rule, a towel, a scale.

## What the task tests

Using communicating vessels as a tool: the far end shows the datum height only while the jar's water is held on the
line, so one hand keeps a reservoir steady on a mark by eye while the other sets a marker 45 cm away, and the order of
moves matters (end up before the jar, jar down before the end). Leveling tasks that read a bubble or a gauge at one
place, or stack packers up to a datum standing beside each bearing, have no reference to carry between two points.
