# Drain a stock tank by siphon

A 1:5 bench model of emptying a round livestock water tank that has no drain plug: the tank on a concrete pad, a
lower rubber tub beside it, and a hose whose two ends stand on the table. "Drain the stock tank into the tub with the
hose." The way to start a siphon without a pump and without sucking on the hose is to sink the hose so it fills, close
one end (a thumb over it, or the hose pinched), carry that end out over the rim and down below the tank's water while
the other end stays under, and only then open it; the tank runs down to the intake and the siphon stops by itself.
The common failures are carrying the outlet out open (the hose drains back as it comes over the rim) and letting go of
the intake (light, on a stiff hose, it springs up out of the water once the hose is bent out over the rim). With no
clip or weight on the rim, one hand keeps the intake down while the other carries the outlet, until the tank is empty.

Run (from the repo root, Isaac Lab-Arena installed; yamkit.py, pourkit.py, siphonkit.py and guardkit.py sit next to
task.py):

    inspect-robots run --embodiment yam_arena --policy yam_scripted -E task=tasks/drain-a-stock-tank-by-siphon/task.py \
      -E object_jitter=0 -E control_hz=20 -E cam_width=320 -E cam_height=176 \
      --instruction "Drain the stock tank into the tub with the hose." --scorer task_score --max-steps 2400
    python tasks/drain-a-stock-tank-by-siphon/test_task.py      # offline scorer checks, prints "all pass"

Use `--policy dream` for a Dream-1 trial. The scene is deterministic (no seeds, object_jitter=0). A full trial is
2,400 steps (2 minutes at 20 Hz). In the sim the demo scores 1.0 and ends at step 1,091: the left hand stands the
intake on the tank's floor and holds it; the right hand picks the outlet up pinched, sinks it beside the intake and eases
its fingers off while the hose fills (primed at step 310), pinches it again, carries it out over the rim and down into
the tub, lets go at step 515 and goes home; the tank is drawn down at step 903 with 1,163 mL in the tub and nothing
spilled, and only then does the left hand let go of the intake and go home.

## Spec

Positions in cm from the midpoint between the two arm bases: x forward, y to the robot's left, table top z 0. Lengths
are 1:5, so the 20 cm tank stands for a 3.3 ft round poly tank of about 50 gallons and the 60 cm hose for a 10 ft one.

| Object | What it is | Where |
|---|---|---|
| Stock tank (fixed) | gray poly, 20 cm across and 5.8 cm deep inside, 5 mm wall and floor; 1,571 mL of water, 5 cm deep | on a concrete pad 19 x 23 x 3.7 cm; both centered at x 33, y 0; inside floor 4.2 cm up, rim 10 cm up, water surface 9.2 cm up |
| Tub (fixed) | black rubber, 20 cm across and 6.2 cm deep inside, 3 mm floor, empty | centered at x 27, y -21.5 |
| Intake end (loose, 40 g) | pine stick 2.5 x 2.5 x 7.9 cm on a 4 x 4 x 0.6 cm plastic foot; clear end tube 1.4 cm across taped to its left (+y) face, mouth 0.8 cm up (pointing down), hose joining its top 3.5 cm up; green tape | standing at x 22, y 20 |
| Outlet end (loose, 120 g) | the same on a steel foot, red tape, turned so its tube faces the robot (-x) and 23 degrees to its right, and its soft hose taped on up that face to the stick's top (3.9 cm front to back with the hose), leaving over the top | standing at x 21, y -9 |
| Hose | 12.7 mm bore clear vinyl, 60 cm, 76 mL; drawn pale when dry, water-colored where it holds water, crossing the tank's rim toward the tub while one end is in and the other out | between the ends |

The model (siphonkit, on pourkit's vessels; numbers from task.py):

- Filling: with both mouths and both end-tube tops under the tank's surface (the hose sunk), the outlet open and no
  more than 10 mL of air left where the outlet's taped run stands above the water (up the stick and back down: 9 mL
  with the outlet on the tank's floor, a slug the starting 0.6 m/s flow sweeps out; held 1.2 cm higher it is 12 mL
  and the hose does not fill), the hose fills in 2 s (a real 10 ft hose takes about 10 s, until the bubbles stop). A pinched outlet traps the air, so a
  hose sunk pinched never fills.
- The pinch (the robot's thumb): a gripper closed across the outlet's stick and its taped hose (fingers stopped by the
  3.9 cm width, `gk.band`), its tool point at least 5 cm up the stick (on the hose run, above the end tube) and its
  finger line within 30 degrees of square across the hose. Fingers on the two plain faces (2.5 cm) carry the outlet
  open; an empty fist or an open hand does not pinch.
- Flow: while primed, open and with the intake's mouth at least 5 mm under the surface (shallower, the intake draws a
  vortex and gulps air), water runs at Q = k sqrt(head) with k = 283 mL/s per sqrt(m) (Darcy friction 0.03, entry and
  exit losses 1.5, for a 12.7 mm bore and 60 cm): 81 mL/s at the starting 8.1 cm head (surface 9.2 cm, outlet mouth 1.1 cm
  up on the tub's floor). It goes into the vessel the outlet's mouth is in, or the one its stream falls into, else onto
  the floor (spilled). Once the tub's water covers the mouth, the head is to the tub's surface.
- Losing the prime (latched with its reason): the intake's mouth out of the water; the open outlet's mouth above the
  tank's surface for one control step; or the tank drawn down to the intake, its surface 5.5 cm up (the intake's
  mouth plus 5 mm), with 1,162 mL run off above that: the normal end.
- The intake rising: the intake end is light (40 g) and the vinyl hose stiff, so whenever the outlet's mouth is outside
  the tank (the hose bent out over the rim) while no gripper holds the intake (fingers stopped by its stick), the
  intake is taken as sprung up to the rim, out of the water (the hose is drawn hanging there).

## Scoring

| Outcome | Score |
|---|---|
| Either end still held at the end (any tool point or pad tip within 1 cm of it, fingers open or closed) | 0 |
| More than 20 mL spilled (a quarter second of the starting 81 mL/s stream missing the tub) | 0 |
| Under 25 % of the 1,162 mL in the tub (nothing done, never primed, the outlet carried out open and never restarted) | 0 |
| A quarter or more in the tub, but the prime lost on the way (outlet open above the surface, intake let go before the tank was empty or lifted out), even if restarted | 0.25 |
| Primed and never lost, but stopped short with under 90 % (1,046 mL) in the tub: an intake held 1 cm off the floor stops with 27 % left | 0.5 |
| Drained (drawn down, 90 % or more in the tub), but the outlet's mouth not in the tub or the intake's not in the tank or the tub | 0.75 |
| All of it, both arms back home and still | 1.0 |
| All of it, an arm not home | 0.9 |

Possible scores: 0, 0.25, 0.5, 0.75, 0.9, 1.0. Success is a full score held for 1 s.

Why two hands: the intake has to stay held on the tank's floor from the moment the outlet leaves the tank until the
tank is empty; nothing in the scene holds it (a clip or a weight hung on the rim would let one hand do the job, and
there is none). One gripper alone can sink both ends and fill the hose, but carrying the outlet out lifts the intake
(0 in test_task.py, including a low carry with the ends far apart, and in the sim).

## Real-world build

A 20 cm round plastic tub (5 mm walls) on a 19 x 23 x 3.7 cm concrete paver, filled with 1,571 mL of water; a black
rubber pan 20 cm across and 6.5 cm tall beside it; 60 cm of 12.7 mm (1/2 in) ID clear vinyl hose with a 3.5 cm piece of 14 mm
OD rigid clear tube in each end, each taped to a 2.5 cm square pine stick screwed to a 4 cm plate (plastic for the intake, steel for the outlet); on the outlet
the hose runs up the stick's face under tape, so the gripper's pads squeeze it flat. A trial is scored by weighing the
pan before and after (1 g = 1 mL), mopping any spill with a weighed towel, and checking the video for the outlet
coming over the rim pinched and a hand on the intake until the tank was empty; the siphon's own stop (the hose
gulping air with about 1.3 cm of water left) is the drawn-down end.

Sources: NC State Extension, "Controlling Algae in Livestock Water Tanks" (clean tanks periodically; some algae are
toxic); CDC, "About Harmful Algal Blooms" (swallowing contaminated water
is how people get sick), which is why the hose is not started by mouth.
