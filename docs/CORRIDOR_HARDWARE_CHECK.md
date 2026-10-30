# Corridor flight on hardware: what changed and how to check it

## Why it changed

On the drone, the entry corridor never saw both walls. Perception showed one
wall, or none. The OAK-D bench logs (`~/RPi/archived_logs/corridor_bench_*.csv`)
show the same thing on the real 11.5 ft corridor:

| walls in view | frames |
|---|---|
| 2 | 81 of ~2000 (4 %) |
| 1 | ~1800 (88 %) |
| 0 | ~150 (8 %) |

There were three causes.

1. **Geometry and stereo.** A wall 1.75 m to the side only enters the
   camera's ~73° view 2.4 m ahead. Stereo on the white walls fades out
   beyond about 3–3.5 m. That leaves very little wall in view, and usually
   only on one side.
2. **Perception needed a wall pair first.** Until it had seen both walls
   together, a single wall gave no fit at all (`walls = 0`). It also forgot
   the width after 5 s without walls.
3. **The planner steered only on walls.** No fit meant hold, back off, then
   `SENSOR_TIMEOUT`. The entry corridor's line-up waited for "walls under the
   banner" that never came.

**The competition corridor's inner walls (the ones the drone sees) are
black; the outer walls are white.** The bench logs were taken on white
walls. Matte black is also poor for passive stereo (the OAK-D Lite has no
IR projector), so expect depth walls to be as rare or rarer. The RGB
wall-base detector (`corridor_walls_from: auto`) should do better there: it
looks for the wall's base against the floor, and black on a lighter floor is
strong contrast. That is not measured yet. The walk-through below on the
real corridor is what measures it: look at how often `walls` is 1 or 2.
The planner does not depend on it either way; it flies the GPS line.

SITL hid all of this because its walls had perfect depth.
`sitl_corridor_sim` now has `wall_depth_max_m`. At 3.5 it reproduces the
bench numbers (12 % none, 72 % one, 15 % two).

## What it does now

- **GPS is the reference; walls correct it**
  (`corridor_follow.GpsCorridorTrack`).
  - The corridor is a line in GPS terms. For the entry and exit corridors
    it comes from the banner fix, measured in the same flight. For the
    surveyed legs it comes from the waypoints.
  - A small Kalman filter keeps the line's sideways offset and angle. GPS
    drift is modelled as a slow random walk.
  - Every frame with one or two walls is a measurement. It counts only if
    it agrees with GPS within 3 SD, so the other corridor's walls, an
    obstacle's side or a bad fit are rejected.
  - Walls never move the line more than 1 m or turn it more than 10°.
  - **No walls in view: the drone flies the GPS line. It does not hold.**
  - The settings are the `corridor_fuse_*` entries in `mission_params.yaml`.
    Raising `corridor_fuse_wall_sd_*` trusts the walls less.
- **Perception.**
  - `corridor_width_prior_m: 3.5052` lets one wall give a fit from the
    first frame.
  - Obstacles are also reported when no wall is fitted. They are looked for
    in a band around the nose, turned by the last fitted corridor direction.
- **Entry/exit line-up.** The drone holds 2 s at the line-up point (walls
  seen then correct the line), then goes in. Walls are not required. The
  leg ends at the measured wall end, or at the 22 ft nominal length plus
  1 m.
- **Staggered obstacles.** These failed in SITL before: the box filled the
  view, the walls dropped out, and the drone held. That no longer stops it.
  Two deadlocks at the second box's stop point were also fixed.
- **Exit-corridor failsafe, exit corridor only.** On any failure of that
  phase, it runs fully autonomously:
  1. If the drone failed within 1 m of the mouth, it first backs out from
     under the banner.
  2. It climbs straight up to `exit_failsafe_alt_m` (5 m).
  3. It flies home at that height and lands.

  If 5 m isn't confirmed, it lands where it is rather than fly home low.
  Battery, RC and FC-mode aborts are unchanged.
- **Your measurements:** `drone_width_m: 0.6294` and
  `camera_pitch_offset_deg: 0`.

## How it was checked (no hardware)

- **Unit tests:**
  - the track: no walls, one wall, which side, rejections, limits, drift
  - unfitted obstacles: a box against a wall never looks passable on its
    wall side
- **Closed-loop tests with the real corridor fit on ray-cast depth**, with
  walls cut at 3.5 m, at 3.0 m, and with no wall depth at all, plus
  drifting GPS:
  - entry corridor: through, at most 0.45 m off centre
  - exit corridor: one, alternating and staggered obstacles, all passed
    with clearance
- **ArduPilot 4.7.1 SITL runs** (results in `docs/OA_EVALUATION.md` and the
  summary given with this change).

### SITL results (ArduCopter 4.7.1, walls cut at 3.5 m, GPS wandering ±0.3 m over 60 s)

"Air" is space between the props and the wall (drone 0.63 m wide).

| Run | Entry corridor | Exit corridor (3 obstacles) |
|---|---|---|
| Final settings | through; max 0.67 m off centre, 0.78 m air (banner heading 17° off) | through; **0.03 m air** at the worst point |
| Before the last obstacle fix | through; max 0.76 m off, 0.68 m air | `BLOCKED` by a phantom obstacle → **failsafe: backed out from under the banner, 5 m, home, landed** |
| Earlier settings | through; max 0.18 m off, 1.26 m air | through; touched a wall |

**Entry corridor: OK.** The wall clearance comes from the 11.5 ft width.

**Exit corridor: NOT safe yet.** A 3.5 ft obstacle leaves 1.21 m each
side, which is only 0.29 m of air per side for this drone. The track's sideways
error (GPS plus a banner fix that can be about 1 m and 20° off) is about that
size. Ways to improve it, from most effective down:

- **Better lateral position.** RTK GPS, or a sideways range sensor for the
  near wall: the OAK-D cannot see a wall right beside the drone.
- **Set `obstacle_length_m` to the real obstacle depth.** The 1.0 m assumption
  makes close obstacles look like an impassable pair.
- **Fly slower** (0.3 m/s) so walls get more frames.

The failsafe is in place for when it fails.

**Banner approach (code unchanged).** With fast GPS drift (±0.3 m every 20 s)
it lost the banner once and timed out once before the line-up point. On the
drone, watch for `NO_BANNER` / approach `LEG_TIMEOUT`.

None of this is the real camera. The two steps below are.

## Step 1: walk-through dry run (no flying, nothing sent to the FC)

`corridor_dryrun` runs the flight code's own track, obstacle memory and plan
on the real OAK-D, GPS and compass, and logs what it **would** command.

1. On the drone, start the sensors (no mission, no planner):

   ```bash
   ros2 launch depthai_ros_driver camera.launch.py name:=oak camera_model:=OAK-D-LITE \
       params_file:=$(ros2 pkg prefix obstacle_avoidance)/share/obstacle_avoidance/config/oak_d_lite_depth.yaml
   ros2 launch obstacle_avoidance sector_builder_hw.launch.py      # includes corridor_perception
   ros2 launch mavros apm.launch fcu_url:=/dev/ttyAMA2:921600     # GPS + compass only
   ```

2. Carry the drone to about 2.5 m in front of the mouth. Hold it on the
   centreline, nose down the corridor, at flight height (about 1.2 m camera
   height), with props off. Then start:

   ```bash
   ros2 run path_planner corridor_dryrun --ros-args \
       --params-file $(ros2 pkg prefix mission_bringup)/share/mission_bringup/config/mission_params.yaml \
       -p csv_file:=$HOME/dryrun_$(date +%H%M).csv -p obstacles:=true
   ```

3. Walk slowly (about 0.3 m/s) through the corridor, keeping the drone at
   flight height:
   - first on the centreline
   - then 0.5 m left of it
   - then 0.5 m right
   - then with the real obstacles in place

   Tape marks every metre on the floor make the comparison easy.

**Pass if:**

- `offset` matches the tape within about 0.25 m along the whole corridor,
  including stretches where `walls=0`. That stretch is GPS alone; if it
  wanders, GPS is the problem, so check sats and HDOP.
- Wall frames show `ACCEPTED` far more often than `rejected`. Many rejects
  mean the perception centre disagrees with GPS: check the width prior and
  the camera mounting.
- `corr` stays well under 1 m.
- With obstacles in place, `pass_left` / `pass_right` picks the open side
  before the stop distance (1 m before the face). It never shows `blocked`
  where a 0.83 m gap exists (drone 0.63 m + 2 × 0.10 m).

Send me the CSV if anything looks off.

## Step 2: flights, in this order, fully autonomous

1. `entry_corridor.launch.py takeoff_alt:=5` with `corridor_follow_speed_ms: 0.3`.
2. The same at 0.5 m/s.
3. `exit_corridor.launch.py` with one obstacle, then the real layout.
4. One exit-corridor run with the corridor blocked on purpose (an obstacle
   across it, or a person with a board): **the failsafe must climb, fly home
   and land.**

Record a rosbag of `/corridor/perception`, `/mavros/global_position/*` and
`/oak/stereo/image_raw` on every flight, and keep the dataflash log.

Before step 2, the flight controller setup must be done. That list is kept
separately and will be gone through before the first flight.
