# BendyRuler vs path_planner: obstacle-avoidance evaluation

**Status: evaluation only. Nothing in the mission changes until this says
BendyRuler is better.** The mission still flies the corridors with
`path_planner` (`corridor_follow.py`) on `stage5_6_corridor.param` (OA off).

The idea: our side of the chain is the OAK-D scan, its parameters, and how we
send it to ArduPilot (`sector_builder` → `/avoidance/scan` →
`obstacle_publisher` → OBSTACLE_DISTANCE). ArduPilot's BendyRuler does the
avoiding. Why it was turned off before: in a 5 ft corridor it can't fit
(`params/README.md`). The competition corridor is now 11.5 × 22 ft
(3.5 × 6.7 m), and a centred 1.2 m obstacle leaves about 1.15 m gaps, so the
question is open again. These tests answer it.

## What was added (nothing existing was changed)

| File | What |
|---|---|
| `obstacle_avoidance/params/bendyruler_eval.param` | FC set: `OA_TYPE 1`, `OA_MARGIN_MAX 0.45`, `OA_BR_LOOKAHEAD 3`, `GUID_OPTIONS 64`, `WP_SPD 0.5`. A starting point for tuning. |
| `obstacle_avoidance/oa_eval.py` (+ `test/test_oa_eval.py`) | The judge. It scores one pass from the true position: contact, clearances, exit, stall, side flip, rejoin, and pass side per obstacle. It scores both planners the same way. |
| `obstacle_avoidance/oa_eval_node.py` | Flies one pass in SITL. `planner:=bendy` sends one GUIDED target (the corridor exit) and leaves the rest to ArduPilot. `planner:=current` calls `/path_planner/fly_corridor_2`. It writes a JSON result line and a trajectory CSV. |
| `obstacle_avoidance/sim_depth_dropout.py` | Drops a fraction of sim depth pixels before both consumers, so `dropout` hurts both planners equally. `sector_builder`'s own `sim_dropout_fraction` would hit only the BendyRuler path. |
| `mission_bringup/launch/oa_eval.launch.py` | One pass: a single corridor ahead of home, the scenario's boxes, the full depth → proximity chain, and the chosen planner. |
| `obstacle_avoidance/scripts/oa_eval.sh` | Batch runner. Each run gets a fresh SITL with the planner's FC set (`--set` overrides for tuning). It ends with a summary table. |
| `obstacle_avoidance/scripts/oa_eval_report.py` | The summary table, re-runnable on any `results.jsonl`. |
| `obstacle_avoidance/scripts/prx_side_check.py` | Test 1: our scan side vs. the FC's sectors, plus FC proximity health. |
| `obstacle_avoidance/scripts/depth_material_check.py` | Test 2: depth steadiness on the real material at a taped distance. |

## The tests

### 1. Scan direction and health (bench, real hardware)

Bring up the OAK driver (our params file), `sector_builder_hw.launch.py`,
MAVROS, and the FC on `stage1_proximity.param` (rebooted). Point the vehicle
at open space, then:

```bash
python3 scripts/prx_side_check.py --expect left    # box ~1.5 m left of the nose
python3 scripts/prx_side_check.py --expect right   # then on the right
```

**Pass:** both PASS. The FC's nearest sector is on the same side as our
nearest bin, and FC proximity reads HEALTHY the whole time. The FC side comes
from ArduPilot's own DISTANCE_SENSOR orientations, not from our `geometry.py`.
Also look at Mission Planner's proximity view once.
**Fail:** a sign, frame or MAVROS problem in our code (`obstacle_publisher`,
`geometry.py`). Fix it. It is not a reason to drop BendyRuler, and it would
break the current planner's pilot-takeover protection too.

### 2. Depth on the real obstacle material (bench)

Use the real obstacle face, face-on and centred, at taped distances:

```bash
python3 scripts/depth_material_check.py --dist 1.5
python3 scripts/depth_material_check.py --dist 2.5
python3 scripts/depth_material_check.py --dist 3.5
```

**Pass:** all three PASS. The forward scan bins are finite in ≥ 95 % of
scans, the median is within max(0.10 m, 5 %), jitter is ≤ 0.02 m + 3 %, and
image valid pixels are ≥ 15 % in ≥ 95 % of frames.
**Fail:** the planner choice doesn't matter. Add texture to the obstacles if
the rules allow it. Otherwise fly much slower with a pilot ready. No
avoidance method works without depth.

### 3. SITL, one centred obstacle

```bash
scripts/oa_eval.sh --planner bendy   --scenario centred --runs 3
scripts/oa_eval.sh --planner current --scenario centred --runs 3   # baseline
```

The scenario is a 3.5052 × 6.7056 m corridor with a 1.2 m wide, 1.83 m tall
box (0.4 m deep) on the centreline at mid-length, flown at 1.2 m.
**Pass:** clean, meaning it bends around the box, rejoins the centreline
(within 0.3 m) and has no stall (no 8 s without 0.2 m of progress).
**Fail → tune**, one change at a time, re-testing after each:

```bash
scripts/oa_eval.sh --planner bendy --scenario centred --set OA_MARGIN_MAX=0.40 --label "margin 0.40"
scripts/oa_eval.sh --planner bendy --scenario centred --set OA_BR_LOOKAHEAD=2  --label "lookahead 2"
scripts/oa_eval.sh --planner bendy --scenario centred --set WP_SPD=0.3         --label "speed 0.3"
```

Air to the props equals `OA_MARGIN_MAX` minus half the drone width. At 0.55 m
tip to tip and margin 0.40, that leaves 0.125 m. Don't go below about 0.35.
BendyRuler replans only once a second (`OA_UPDATE_MS`), which is another
reason to keep the speed low.

### 4. SITL, two staggered obstacles (the hard one)

```bash
scripts/oa_eval.sh --planner bendy   --scenario staggered --runs 3
scripts/oa_eval.sh --planner current --scenario staggered --runs 3
```

Box A sits 0.75 m left of centre and box B 0.75 m right, with front faces 2.5 m
apart (`--spacing`). Each leaves only 0.4 m on its near-wall side, so the
pass is a forced slalom.
**Pass:** clean, meaning through both with no contact and no side flip (it
doesn't swap sides in front of a box). Rejoin isn't required here.
**Fail:** use the same tuning as test 3. If it still fails after tuning,
that is the main reason to stay with `path_planner`.

### 5. Repeatability

```bash
scripts/oa_eval.sh --planner bendy   --scenario centred   --runs 10 --dropout 0.2
scripts/oa_eval.sh --planner bendy   --scenario staggered --runs 10 --dropout 0.2 --spacing 2.0,2.5,3.0
# and the same two with --planner current
```

**Pass:** at least 9 of 10 clean (the report prints PASS/FAIL against 90 %,
with at least 5 runs). Flaky results mean the setup is marginal: slow down
further, or take it as a reason to stay with `path_planner`.

### 6. Slow real flight (about 0.5 m/s, fully autonomous)

Only after 1–5 pass. The competition flight is fully autonomous: no pilot
takeover. The only fallback is the mission's own exit-corridor failsafe
(climb above the corridor walls, then return to launch), so test 6 flies with
that failsafe in place. Load `bendyruler_eval.param` on the FC and reboot. Run
the OAK driver and `sector_builder_hw.launch.py` (it includes
`obstacle_publisher`). Plan an AUTO mission (or a GUIDED target) from 2.5 m
before the corridor mouth to 1 m past its end, at the test altitude. AUTO uses
the same OA-aware waypoint navigator as GUIDED with `GUID_OPTIONS 64`. After the flight, look at the
dataflash `OABR` messages (BendyRuler's chosen bearing and margin) and `PRX`.
**Pass:** it behaves the same as in SITL.
**Fail when SITL passed:** suspect camera noise, vibration or pitch
calibration (`camera_pitch_offset_deg`, Gate 3C), not BendyRuler.

## Decision rule

Switch the corridors to BendyRuler only if all of these hold:

1. Tests 1 and 2 pass. They are needed for either planner.
2. In tests 3–5, BendyRuler's clean rate is at least `path_planner`'s on
   every scenario, and it is ≥ 9/10 on both scenarios with dropout.
3. Its worst obstacle and wall clearances are no worse than `path_planner`'s.
4. Test 6 matches SITL.

Otherwise keep `path_planner`. Either way, record the results table below
together with the exact FC parameter line the report prints.

## Results

SITL = ArduCopter 4.7.1 (dbe79216), the firmware on the Pixhawk. Defaults
unless stated: 3.5052 × 6.7056 m corridor, 1.2 m wide × 1.83 m tall boxes,
1.2 m altitude, path_planner at 0.5 m/s, BendyRuler `WP_SPD 0.5`.

| Date | Test | Planner | FC params / label | Clean | Worst clearance (wall / obstacle) | Notes |
|---|---|---|---|---|---|---|
| 2026-10-29 | 3 | bendy | file defaults (margin 0.45, DB max 6) | 0/1 | outside the corridor | Saw the box from the entry point, went round the outside of the left wall's front end and along the outside of it |
| 2026-10-29 | 3 | bendy | `OA_DB_DIST_MAX 3` | 0/1 | outside the corridor | Entered the mouth, backed out, went round the outside |
| 2026-10-29 | 3 | bendy | `OA_DB_DIST_MAX 3`, `OA_MARGIN_MAX 0.35` | 0/1 | outside the corridor | Same |
| 2026-10-29 | 3 | bendy | `OA_DB_DIST_MAX 3`, `OA_MARGIN_MAX 0.25` | 0/1 | outside the corridor | Same |
| 2026-10-29 | 3 | current | stage5_6_corridor | 1/1 | 0.24 / 0.30 m | Passed right, rejoined, 38.9 s |
| 2026-10-29 | 4 | bendy | `OA_DB_DIST_MAX 3`, `OA_MARGIN_MAX 0.35` | 0/1 | outside the corridor | Left the corridor, side flip in front of box A |
| 2026-10-29 | 4 | current | stage5_6_corridor, spacing 2.5 m | 0/1 | 0.60 / 0.65 m | Passed A right; chose B's side 1 m behind B's face, walls left the view, held, `SENSOR_TIMEOUT` at x = 3.7 m |
| 2026-10-29 | 5 (test 3 ×5) | current | stage5_6_corridor, dropout 0.2 | 5/5 | 0.15 / 0.29 m | 3 left, 2 right; worst wall 0.15 m |

### What the BendyRuler runs show

- BendyRuler only knows walls the camera has seen. Behind the wall ends and
  outside the walls looks free, so with the box ahead it keeps choosing to go
  round the corridor, even back out of the mouth (it searches bearings up to
  170° either side). Lowering `OA_MARGIN_MAX` to 0.25 did not change that.
- Our scan is 72 bins of 5° each. ArduPilot places each reading at its bin's
  centre with a radius of about distance × tan(2.5°), which smears box and
  wall edges outward by up to ~0.13 m each side at 3 m and narrows the
  1.15 m gaps further.
- Making it stay inside would need either an inclusion fence around the
  corridor, uploaded when the corridor is located, or finer angular data
  (OBSTACLE_DISTANCE allows 72 bins at 1° over the ~72° camera view via
  `increment_f`). Neither exists yet.

### After the GPS-anchored planner change (2026-10-30)

On the drone the OAK-D rarely saw both walls of the 11.5 ft corridor, so
`path_planner` now flies the corridor's GPS line, corrected by the walls
(`docs/CORRIDOR_HARDWARE_CHECK.md`). The sim can now degrade the walls the
same way (`--launch-arg wall_depth_max_m:=3.5`). The runs above used perfect
walls, which flatter both planners. Re-run results with drone-like walls are
added below.

## Caveats of the SITL harness

- The sim renders boxes and walls with perfect stereo. Dropout is random
  pixels, not real textureless patches. Test 2 is what covers the real
  material.
- Truth is the SITL position in the sim's frame (constant `gps_offset_m` is
  supported; drift is not). Contact means the drone's footprint circle
  (`drone_width_m`/2) touched a wall or box below its top. SITL has no
  physical collision, so the flight continues after contact. Only the first
  contact is reported.
- `path_planner` may climb over a box (`corridor_max_altitude_m`), while
  horizontal BendyRuler can't. The judge reports `over` as the pass side,
  which isn't a contact. Set `obstacle_top_m` above what `path_planner` can
  climb to compare like for like.
- Obstacle height, depth and the drone width are placeholders. Set the real
  values (`--launch-arg obstacle_top_m:=… drone_width_m:=…`).
