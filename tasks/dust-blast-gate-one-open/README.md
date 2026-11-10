# Set the blast gates for the machine about to run

A 1:2.5 bench model of a small woodshop's dust collection: one collector, a trunk duct across the bench and three
4-inch branches (here 4 cm) to a table saw, a planer and a disc sander, each with a sliding blast gate. A rough board
lies on the planer's infeed table. "Get the dust collection ready to plane this board." Before a machine runs, its
gate goes fully open and every other gate fully shut, because a small collector has the airflow for one branch at a
time: an idle gate open, or part open, splits the suction and the running machine's dust goes into the air. The
shop was left with the saw's gate open and the sander's half open, so opening only the planer's gate is the tempting
wrong answer.

Run (from the repo root, Isaac Lab-Arena installed):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/dust-blast-gate-one-open/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Get the dust collection ready to plane this board." --scorer task_score --max-steps 600
    python tasks/dust-blast-gate-one-open/test_task.py      # offline scorer checks, prints "all pass"

Use `--policy dream --max-steps 2400` for a Dream-1 trial. The scene is deterministic (no seeds, object_jitter=0).
The scripted demo finishes in 364 steps (success held 1 s); a full trial is 2,400 steps (about 10 minutes of sim time on an L40S).

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Trunk duct | galvanized steel pipe, 5.6 cm across, on gray saddles, end cap at the saw's end | along y at x 22, y -45.8 to 19, center z 3.2; an elbow into the collector |
| Collector | blue steel drum 13 cm across, 11 cm tall, white filter bag on top (about 12 cm) | x 30, y -43 |
| Branch ducts | galvanized pipe, 4.4 cm across (4 cm bore), on saddles | along x from the trunk (x 22) to the machines (x 47), center z 3, at y 15 (saw), 0 (planer), -15 (sander) |
| Blast gates (3) | galvanized housing 2 x 6 x 5.4 cm on each branch at x 31, open on its +y side; steel blade 0.3 cm thick, 7.2 x 4.4 cm, 45 g, slides along y; black pull handle 3 x 2.5 x 2.8 cm on a short stem above the blade's tail, 6 to 8.8 cm up | handle center at y = branch + 4.5 + blade travel; travel 0 (shut) to 4.6 cm (stop) |
| Table saw | blue cabinet 12 x 11 x 8.4 cm, cast-iron top 13 x 13 cm, blade, fence, red switch | x 47 to 59, centered y 15 |
| Planer | orange stand 13 x 10 x 5 cm, cast-iron bed, cutter head on two columns (top 11.5 cm), red switch | x 47 to 60, centered y 0 |
| Board | rough oak 9 x 5.5 x 1.3 cm, saw marks across it, lying on the planer's infeed bed with its end under the cutter head | x 45.5 to 54.5, y 0, z 5.6 to 6.9 |
| Disc sander | green base 12 x 10 x 4.4 cm, motor, 8.4 cm sanding disc facing the robot, work table, red switch | x 47 to 59, centered y -15 |

Start: saw gate open (4.6 cm, at its stop), planer gate shut (0), sander gate half open (2.2 cm).

Mechanism (`FrictionSlide` in `task.py`, through the law hook in the local `yamkit.py`): each blade is a slide with
end stops at 0 and 4.6 cm, no spring and no detent. Its friction in the housing is a soft hold at its current
position (60 N/m) plus drag (2 N*s/m): about 0.1 N against a 3 cm/s pull, so it moves under any push and stays where
it is left.

## Scoring

Scored on the final state (SCORE_OVER = "end"; success must hold 1 s).

| Gate state | Definition | Why |
|---|---|---|
| Open | blade pulled out at least 4.0 cm (of 4.6) | its leading edge is then at most 2 mm into the 4 cm bore, blocking 1.9% of the bore's area (circular segment); estimate: under 2% less airflow at the planer |
| Shut | blade withdrawn at most 0.4 cm | shut, the blade overlaps the bore's far side by 2 mm in its seat; 0.4 cm out uncovers a 2 mm sliver, 1.9% of the bore's area; estimate: such a sliver passes about 2% of the collector's flow, within the margin of a small collector |
| Part open | anything between | splits the suction |

| Outcome | Score |
|---|---|
| A tool point within 4 cm of any gate's handle center (a hand still on a gate) | 0 |
| A gate still sliding faster than 5 mm/s | 0 |
| Planer's gate not open | 0 |
| Planer's gate open, neither the saw's nor the sander's shut (e.g. all three opened, or only the planer's) | 0 |
| Planer's gate open, one of the other two shut | 0.25 |
| Planer's gate open, saw's and sander's shut, both arms home and still | 1.0 |
| The same, arms not home and still (each tool point within 3 cm of where it started) | 0.9 |

Possible scores: 0, 0.25, 0.9, 1.0. Doing nothing scores 0 (the planer's gate starts shut).

Why the other numbers: the 4 cm hands-off radius is just over the handle's size (2.5 to 3 cm), so a gripper on the
handle is within it and one lifted clear is outside it; 5 mm/s separates a gate at rest from one still coasting.

In a real trial: measure each blade's withdrawal with a ruler against a pencil mark at its shut position, and check
the arms are back where they started.

## Real-world build

- Three 4-inch aluminum blast gates and 4-inch flexible or PVC duct, a 6-inch main with three 6x4 wyes, and a
  1 hp single-bag collector, at full scale; or, for a bench rig at the modeled 1:2.5 scale, 40 mm PVC pipe with
  hand-cut gates (two 2 mm aluminum plates with a 3.6 mm slot, a 3 mm aluminum blade, a printed handle on a stem).
- Stand-ins for the machines: painted plywood blocks of the listed sizes; a rough oak offcut as the board.
- Mark each blade's shut position on the housing; set the start positions with a ruler.

## How it differs from nearby board tasks

- chute-gates-to-green-bin (two diverter gates set for a named bin, then a release): the target is named in the
  instruction, its score counts where the balls land, and there is no rule about idle paths. Here the instruction names
  the job (planing, with the board waiting on the planer), and the rule is about the gates it never mentions: every
  idle gate, including the two left open from earlier work, fully shut, since part open still splits the suction.
- stuck-valve-diagnosis, open-the-vent-before-draining-the-tank, double-block-and-bleed-before-removing-the-cap:
  valve tasks about a fault or an order of operations, not one-open-the-rest-shut line-ups.
