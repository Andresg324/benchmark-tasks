# STATUS: toothpaste-pea-dose-on-child-brush (cbc build, Mon Nov 16 2026)

Worktree `~/cbc-toothpaste` (from `~/benchmark-tasks` at upstream/main c82d718b). Branch pushed to the private repo
`Andresg324/yam-kits` as `cbc/toothpaste` only. No push to robocurve, no PR, no media upload, no GitHub comments,
no Dream-1.

## Base
`origin/inst/toothpaste` appeared on yam-kits during step 2 (the primary's offline build, 33f6ca7). Per the brief,
its task folder became the base and every spec change from `~/ideas-nov16/TOOTHPASTE-SPECS.md` went on top. The
primary has pushed nothing newer since (checked before every sim).

## Result
- **Demo: 1.0**, success at step 512 (25.6 s). task.py sha256 7bd09be57cce, the same file for every run below.
  0.284 g on the bristles, 0.010 g off; the brush held level (1.1 deg); the brush and tube laid back and released.
- Variants: too much (`TOOTH_VARIANT=much`, 2.5 mm for 1.5 s) **0.25** (0.461 g on, 0.070 g off). Smear (`smear`,
  1.2 mm for 0.6 s) **0.25** (0.111 g on). Do nothing (`nothing`, both hands park) **0**.
- Offline: test_task.py passes all checks, including `test_lid_shut_seals`. ikplan finds no reach misses on either
  arm. check_task: the only FAIL is the missing dream_top.mp4 (Dream-1 is not to be run here).
- Media: ~/media/toothpaste-pea-dose-on-child-brush/demo_top.mp4 (final run), setup.png (its first frame) and
  setup_hires.png (1280 x 704, t = 0). Copies of the variant videos are in ~/yam-eval/toothpaste/keep/.
- Key frames (hi-res rerun of the final file): at t = 0 the bristles are clean white (no pop-in); at the end there is
  a blue dab in the middle of the bristles with the brush lying flat. Visible result = score.
- Grip-envelope note (coordinator, ~/cbc-grip/GRIP-ENVELOPE.md): that study commanded the fingers fully closed on
  rigid parts. Here the tube is held by a position command 0.4 mm into sprung panels (about 2.2 N); the probes show
  the real push equal to the command (0.4-0.5 mm holding, about 2 mm squeezing), so there is no 33 mm pad sink, and
  the tube tracked the hand through lift, carry and turn. The brush is gripped across its 16 mm handle (in the
  reliable 1-20 mm band), 1 mm in (about 5 N, not the 50 N cap), and stayed level in every run from probe 3 on. Stock
  pads were kept, so rubber pads would need re-validation. Fingers do sit up to 1 mm inside the drawn tube cylinder
  at the squeeze (a dent).

## Spec changes applied (TOOTHPASTE-SPECS.md, "what to change")
1. SKU: Crest Kid's 4.6 oz (130 g), partly used (60 g, reset fill 50-70 g). Done in task.py, task.json, README.
2. 35 mm body (was 34 mm) and 7.5 mm nozzle. The yield pressure is now computed as 4 tau L / D with the 7.5 mm bore
   (it was 5 mm).
3. Offline doses at 100, 300 and 500 Pa yield. Modelled value: 300 Pa, the middle of the literature range (100 Pa was
   the low end). The demo squeeze gives 0.34 g at 100 Pa, 0.254 g at 300 Pa and 0.18 g at 500 Pa, all inside
   0.15-0.35 g. The push that reaches the band at each yield is tabled in the README. arXiv 1903.10888 and US 5202112
   are cited.
4. Density 1.30 g/mL is labelled an estimate (range 1.3-1.6; silica low). Done.
5. The 0.15-0.35 g band is backed by data: CDC pea 0.25 g; den Besten mean 0.256 g; parents' "pea" 0.30 g and actual
   0.36 g. Done (README "Why 0.15 to 0.35 g", task.json scoring).
6. Cap: **starts open, with justification.** A closed snap-hinged lid plus a left-fingertip flick was built and run in
   four probes (p1-p4). The fingertip landed 1-1.5 cm off a 3-4 mm tab next to the right hand each time (arm
   tracking; the hands collided in p2-p3), and the trial became a tab-finding test. The lid is still modelled: a snap
   hinge at its open rest, and if shut it seals the nozzle (`test_lid_shut_seals`). README "Why the cap starts open".
7. Fidelity note on the plumb drop (no ribbon touch-down, stringing or tail break-off; the 4 mm gap is about where a
   real ribbon touches down). Done (README and task.json fidelity).
8. Brush named (Colgate Kids Extra Soft 2+), all dimensions marked [M]. Done.
9. Fluorosis: Wiener 2018, 61.3% very mild or worse at ages 16-17 (2011-12), called cosmetic. The 65% figure is not
   used. Done.
10. Overlap paragraph vs #1919 and #2254/#2101, plus #2283 (found in the duplicate check). Done.

## Sim fixes found by the probes (all in task.py comments)
- p1: the panels sagged 10 mm before the first control step, and the sealed lid vented while the tube lay level. Fix:
  the panels' and lid's travel opens only once their laws act; a sealed tube never vents.
- p1-p2: soft walls (0.4 N/mm) in series with the finger servo meant the "0.2 mm" hold gripped with about 0.1 N, and
  the tube was never lifted. Fix: `tube_grip()` commands the finger past the wall by force / servo stiffness.
- p2: GRIP_YAW was -90 deg in the primary's code; on the real arena that closes the fingers along x (GLOBAL-FIXES A1).
  Fix: yaw 0.
- p3: the tube rocked over 40 deg lying on its sprung bottom panel. Fix: a rolled flat end 55 mm wide as a foot.
- p2-p3: a level pose at tcp z of about 1.03 m is out of reach (ikplan), and the swing over the brush knocked it.
  Fix: the brush is held turned (handle to +y); the tube is turned nozzle-down beside the brush (y -0.06), then carried
  over.
- p5: the bottom panel bore the weight, bounced and pumped paste out (0.7 g off). Fix: a 36 mm stand-up cap plus the
  foot carry the tube 0.5 mm off its panels.
- p6: level, the nozzle sat at squeezekit's 90 deg vent boundary; the hold lost its pressure and the tube slipped.
  Fix: down_deg 180 (a full tube has no air at the nozzle).
- p7: after the dose the paste re-drew air, the hold had only 0.4 N/mm walls, and the tube slid onto the brush. That
  knocked the brush out of the hand, and the scorer still gave 1.0 (the brush fell at 43 deg, under 45): an accidental
  pass, not used. Fix: 2.5 N/mm walls (estimate [M]).
- p8: clean demo.
- The tile drawer now draws the dab in the brush's frame every step. The primary's world-fixed tiles would have left
  the dab floating where it was dosed after the brush was laid down.

## Open risks
1. **Duplicate risk, #2283** (another team, opened Nov 16 09:31Z): it squeezes a sprung-wall glue tube to put a
   10-30 uL bead on a target, one step in a mug repair. Same sub-ability (squeeze-tube dosing). Not a KILL in my
   judgment: ours is bimanual, with a held and aimed target, a mass band and no repair steps. But GLOBAL-FIXES C9
   ("anything close to an earlier pending PR by another team is finaled") makes this the top risk. Also near #1919
   (piping) and the dosing pours #2254/#2101. The README explains the differences.
2. **The cap starts open** (justified in the README; four failed flick probes). It can still draw a "start state"
   note (#620 "bag should start closed").
3. **"Trivial" importance attack** (SPECS Part 1): putting toothpaste on a brush is everyday. Societal relatability
   is the right category, but "a dose rule is knowledge" may still be argued.
4. **Model constants are estimates [M]:** wall 2.5 N/mm (raised from 0.4 to keep the grip after dosing), 1.0 mL/cm,
   8 mL air, flow, 300 Pa yield, 36 mm cap, 4 mm tab, all brush dimensions. The calibration procedure is in the
   README. At 100 Pa the holding grip would leak.
5. Scorer leniency seen in p7: a dropped tube plus a brush knocked out of the hand still reached 1.0 (the brush fell
   at 43 deg, under the 45 deg limit). The demo no longer does this, but "released on the counter" does not tell
   placing from dropping.
6. Only one demo seed and a fixed layout (object_jitter 0); the demo is open loop.
7. No Dream-1 run (not allowed here), so the scores field says Dream-1 not run.

**P(accept) estimate: about 6%** (SPECS said 9% before the #2283 overlap surfaced and before the cap had to start
open). Societal relatability base rate 12%.
