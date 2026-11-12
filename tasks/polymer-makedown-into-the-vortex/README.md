# Make up a polymer stock without fish-eyes: one hand stirs while the other sprinkles the powder in

A 250 mL beaker holds 200 mL of water in a collar on the bench; a white powder dredger (a round shaker with a
perforated cap and a side handle) holds 0.5 g of dry polymer, and a glass stirring rod stands in its wooden rest.
"Make up the polymer stock: get all of the powder in the shaker dissolved into the water in the beaker, then put the
rod and the shaker back." Dry polyacrylamide flocculant, and gums such as xanthan or guar, are made down by sifting the
powder slowly into the shoulder of a running vortex so each grain is wetted on its own (911Metallurgist, "Dry
Polymers": 0.5 g into 200 mL for a 0.25 % stock, sprinkled slowly into the vortex to avoid fish-eyes). Gums start to
hydrate almost as soon as they touch water (Ingredion, "How to disperse gums"), so powder that lands on still
water, or arrives faster than the swirl spreads it, gels on its outside within seconds into fish-eyes: lumps
with a dry core that never dissolve (Ross, mixers.com, "Eliminate fish eyes in your mix"), clog filters and the feed pump's strainer and leave the stock weak. So one hand keeps the rod
going the whole time the other is sprinkling; one hand alternating between rod and shaker leaves the water still for
seconds at every switch.

Run (from the repo root, Isaac Lab-Arena installed; yamkit.py, guardkit.py and dispersekit.py sit next to task.py):

    inspect-robots run --embodiment yam_arena --policy yam_scripted \
      -E task=tasks/polymer-makedown-into-the-vortex/task.py -E object_jitter=0 -E control_hz=20 \
      -E cam_width=320 -E cam_height=176 \
      --instruction "Make up the polymer stock: get all of the powder in the shaker dissolved into the water in the beaker, then put the rod and the shaker back." \
      --scorer task_score --max-steps 1000
    python tasks/polymer-makedown-into-the-vortex/test_task.py      # offline model and scorer checks, prints "all pass"

Use `--policy dream --max-steps 2000` for a Dream-1 trial. The scene is deterministic (object_jitter=0).

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0.

| Object | What it is | Where |
|---|---|---|
| Beaker | a 250 mL low-form Griffin beaker, 7 cm across (6.6 cm inside) and 9.5 cm tall, 200 mL of water (5.8 cm deep), held in a fixed black rubber collar round its foot | axis at x 28, y 0 |
| Rod rest | a wooden block 4 x 4 x 3 cm with a 1.8 cm square hole 2.5 cm deep; fixed | centered x 28, y 13 |
| Stirring rod | glass, 15 cm long and 8 mm across, a blue silicone grip sleeve 1.4 cm across and 4 cm long round its middle with a collar 2.2 cm across under it; 30 g | standing in the rest's hole |
| Dredger | a round white HDPE body 4 cm across and 9 cm tall, a red screw cap 4.2 cm across with seven 3.6 mm holes (face 10.2 cm up), and a red round handle 2.2 cm across and 5 cm long out of its side toward the right arm at 5 cm up (its center of mass); 40 g with 0.5 g of dry polymer | standing on the bench, x 31, y -11 |

Model (`dispersekit.py`, read from the poses every control step; no particles in the sim):

| Quantity | Value | Why |
|---|---|---|
| Flow from the cap | none until the cap's axis points 95 degrees from straight up; then 5 g/s x ((tilt - 95) / 85)^2, so about 0.07 g/s at 105 degrees and 5 g/s cap straight down | a perforated cap only trickles once the powder rests on it past level; turned over, a fine powder pours |
| Where it lands | plumb below the cap; inside the beaker's mouth on the surface, else on the bench (spilled) | |
| Stir | the rod's tip path through the water (interpolated between steps) drives a swirl u that rises in 0.2 s and dies in 0.5 s | turbulence behind a rod stops within about half a second |
| Wetting | with u at 5 cm/s or more, floating powder wets apart one grain-layer at a time (0.006 g/cm2), faster with a faster swirl | separate grains wet; a heap's inside stays dry |
| Fish-eyes | powder floating 1 s on still water (fresh powder landing on a patch joins it without making it younger), or heaped thicker than one layer for 1 s, gels into lumps for good | gums start to hydrate almost at once; 1 s is the model's figure for the gel skin (real powders take a few seconds) |

Visuals (markers, never scored): the water column, each floating patch of powder as a pale disk, each fish-eye as a
white ball, and the powder stream from the cap. They are drawn before the first frame.

## Scoring

At the end of the run (held 1 s):

| Outcome | Score |
|---|---|
| A tool point or pad tip within 1 cm of the rod or the dredger (still holding either) | 0 |
| The dredger not standing on the bench (more than 10 degrees from upright, or its foot over 5 mm up) | 0 |
| The rod not back in its rest (its tip not in the hole, within 3 mm, below the rest's top) | 0 |
| Less than 95 % of the 0.5 g on the water (left in the dredger or spilled) | 0 |
| More than 0.01 g fell beside the beaker | 0 |
| More than 3 % of the dose (0.015 g) gelled into fish-eyes or still floating | 0 |
| Powder came out of the cap for 3 steps in a row (0.15 s) while no hand gripped the dredger's handle or no other hand gripped the rod's sleeve (tool point within 1 cm, fingers stopped by it: 0.06-0.26 on the handle, which the fingers pinch off center, 0.075-0.18 on the sleeve); latched | 0 |
| All of the above met, both arms back home and still | 1.0 |
| The same, arms not home and still (each tool point within 3 cm of where it started) | 0.9 |

Possible scores: 0, 0.9, 1.0. Doing nothing scores 0.

Why the numbers: 95 % keeps the 0.25 % stock within 5 % of its label strength, the tolerance a jar test's dose is
read against. 3 % is a few fish-eyes in 200 mL: more is waste a pump screen catches and an operator would make the batch
again for. 0.01 g is a dusting: wet polyacrylamide on a bench or floor is as slick as ice, so spilled powder is wiped
up at once, and a careful sprinkle spills none. Fish-eyes are latched: stirring harder later never dissolves them.

Why it takes two hands: powder only wets apart while the rod keeps the water moving, and lumps after 1 s on still
water, so the sprinkling and the stirring have to overlap in time. Setting the shaker down, taking up the rod and
starting to stir takes well over a second, so one hand switching tools lumps every portion (offline: over 90 % of the
dose). Stirring with the dredger itself does not stir (it is not a stirrer) and the cap under water loses the powder. A
dredger parked tipped over the beaker would dose itself while the one hand stirs, so powder may only come out while one
hand holds the dredger by its handle and the other holds the rod (offline: parked at 98 to 105 degrees, 0).

The demo: the left hand takes the rod by its sleeve, carries it over the rim into the beaker and stirs circles at 0.6 s
a turn (the tip runs about 1 to 1.5 cm circles, 14 to 19 cm/s); the right hand comes down onto the dredger's handle,
swings it over and holds the cap above the water about 2 cm off the axis (the vortex's shoulder) at 105 degrees, where
it trickles empty in about 7 s, then stands it back on its spot and lets go; the left stirs 3 s more, lifts the rod
out, stands it in its rest and lets go; both arms home. In the sim: 1.0 at 849 steps (42 s); all 0.5 g on the water,
no fish-eyes, nothing spilled. The dredger turns about 19 degrees in the hand on the way over, the same every run, so
the cap's aim allows for it.

Offline (test_task.py, 32 tests): the demo's pattern 1.0 and a 50 cm/s stir 1.0; 0 for doing nothing, powder
before the stirring (over 90 % lumped), a 1.5 s pause mid-sprinkle, the dredger turned over in one step (into the demo's
stir or a 50 cm/s one), a two-step flick of it (latched), 135 degrees held (1.1 g/s), one hand alternating (over 90 %
lumped), the dredger parked tipped while one hand stirs (with the other hand hovering half open or as a fist near the handle: 0; one- and two-step grip flickers: still 1.0), the rod swirling with no hand on it, stirring only, sprinkling only, stirring with the dredger's cap in the water, the cap over the bench, a 0.13 g
spill on the way in, the rod left in the beaker, the dredger left tipped, a closed or open hand on any part of either
tool; 0.9 with the arms away from home.

In a real trial: weigh the dredger before and after, and wipe the bench round the beaker onto a tared weigh paper
(spill); pour the stock through a 1 mm sieve and weigh the drained gel lumps (fish-eyes), and watch the video for the
rod moving during every second the dredger was tipped past level.

## Real-world build

- A 250 mL low-form glass beaker in a rubber foot collar (or taped down), 200 mL of tap water.
- A 15 cm glass stirring rod with a silicone sleeve, standing in a drilled wooden block.
- A small stainless or HDPE flour dredger with a side handle and a perforated cap, holding 0.5 g of dry
  polyacrylamide flocculant (anionic, water-treatment grade) or xanthan gum.
- A 0.01 g balance, a 1 mm sieve, weigh paper.

## What the task tests

Two hands doing coupled, timed work on one process: the rate the powder goes in is limited by the swirl the other hand
keeps up, and neither job can pause for more than a second. Board tasks where one hand holds a pan or mortar still
while the other whisks or grinds have no such coupling, and pour tasks pour into still liquid with one hand.
