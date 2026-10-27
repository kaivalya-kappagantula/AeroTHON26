# AeroTHON26 — Project Guide

Everything you need to understand, run and change this code: what the drone has to
do, how the software is built, how each mission phase works and why it works that
way, how to test it, and what is still open.

For the quick "clone and run" steps, see the [README](../README.md). This guide is
the long version.

---

## 1. What the drone has to do

The AeroTHON 2026 mission, flown fully autonomously, in **under 15 minutes**:

1. **Reference QR.** Take off. A QR code lies on the ground at most ~1 m ahead of the
   launch point. Read it from 5 m and remember it. It may hold **no information at
   all** (so it is remembered by its picture, not just its text).
2. **Entry corridor.** Two corridors stand side by side in front of the drone. The one
   to enter has a **green banner** across the top of its entrance. Find the banner,
   line up with the corridor under it, fly through, climb to 10 m.
3. **Arena search.** The arena is a **30 m × 40 m** rectangle (the corridors are on a
   30 m side; its GPS corners are given on competition day). It holds **5–6 QR codes**
   (1 m × 1 m, flat on the ground); exactly **one matches** the reference. There is
   one **red mat (10 ft × 10 ft)** that must **never be flown over, at any height** —
   its position is not given, the drone must find it. Sweep the arena at **10 m** to
   find the matching QR.
4. **Payload drop.** Descend to **5 m** over the matching QR and release the payload.
   (Descending is only allowed for the drop.)
5. **Corridor 2** (with obstacles), **fly home, land.**

If the reference QR can't be read, the mission carries on without a target (no arena
search, payload kept). If the arena has no match, it carries on to corridor 2 with
the payload.

### Real dimensions (for SITL only — the code never hard-codes them)

| Thing | Size |
|---|---|
| Entry corridor | 8 ft tall × 11.5 ft wide × 22 ft long (2.44 × 3.51 × 6.71 m) |
| Green banner | 11.5 ft × 1.5 ft (3.51 × 0.46 m), sitting on top of the corridor walls |
| Arena | 30 m × 40 m |
| Red mat | 10 ft × 10 ft (3.05 m) |
| Arena QRs | 1 m × 1 m, 5–6 of them |

The flight code **measures** corridor width, length and banner height in flight;
test rigs differ from the competition, so nothing is assumed.

---

## 2. Hardware

| Part | Role |
|---|---|
| Raspberry Pi 5 (Ubuntu 24.04, ROS 2 Jazzy) | Runs all of this code |
| Pixhawk 6C Mini (ArduCopter) | Flight controller; the Pi talks to it over TELEM2 (`/dev/ttyAMA2:921600`) through **MAVROS** |
| OAK-D Lite (forward) | Depth (walls, obstacles) + colour (green banner) |
| RPi Camera Module 3 (downward) | QR codes and the red mat |
| Payload release | Servo on Pixhawk output 10 (`release_channel`), lock = SERVO10_TRIM, open = 1100 PWM |
| RC transmitter CH7 | Mission start switch; flipping it LOW at any time hands control back to the pilot (LOITER) |

---

## 3. Software architecture

### Big picture

```
            ┌──────────────── mission_manager ────────────────┐
            │ mission_node: runs the phases in order           │
            │   phase modules:  takeoff (in mission_node)      │
            │                   reference_qr.py   (phase 1)    │
            │                   entry corridor    (phase 2)    │
            │                   arena_search.py   (phase 3)    │
            │ state machine (state_machine.py) tracks phase     │
            └───────┬──────────────┬──────────────┬────────────┘
                    │ services     │ topics       │ MAVROS (GUIDED position targets)
   path_planner ◄───┘              │              └──► Pixhawk
   (corridors, entry corridor)     │
   qr_detection  ── qr_scanner_node, red_zone_node (downward Cam3)
   obstacle_avoidance ── banner_node, corridor_perception, sector_builder (OAK-D)
   payload ── payload_node (servo release)
```

- **All flying goes through MAVROS in GUIDED mode**: the code streams position targets
  (lat, lon, altitude, heading). ArduPilot does the low-level control.
- **Every phase is a module shared by the real mission and a test flight.** The test
  flight for a phase runs exactly the mission's code for that phase, then lands. This
  is the main design rule — we can't build the whole arena on campus, and it makes it
  obvious what went wrong.
- **Safety is checked continuously** (`MissionNode._tick()`, called inside every wait):
  RC CH7 low → LOITER (pilot flies); flight controller leaves GUIDED → stop commanding;
  battery critical → leave it to ArduPilot's failsafe; obstacle sensor dead → abort.
  A failure in the air → LAND in place.
- **The FC's own parameters are never written by the code** — only runtime commands
  (mode, arm, takeoff, position targets, speed limit, telemetry rates).

### Packages (`ros2_ws/src/`)

| Package | What's in it |
|---|---|
| `mission_manager` | `mission_node` (the full mission), the phase modules, the state machine, and one test-flight node per phase |
| `path_planner` | Entry corridor logic (`entry_corridor.py`), corridor following, the arena geometry (`arena_frame.py`), sweep re-planning around red zones (`sweep_replan.py`), lawnmower generator (`waypoint_gen.py`) |
| `qr_detection` | `qr_scanner_node` (find / read / match QRs), `red_zone_node` (red mat detection), `sim_down_camera_node` (SITL camera), `qr_pipeline.py` (detection + picture matching) |
| `obstacle_avoidance` | OAK-D pipeline: `banner_node` (green banner), `corridor_perception_node` (walls), `sector_builder_node` (obstacles → ArduPilot proximity), `coverage_guard_node` (sensor health), `sitl_corridor_sim` (simulated corridors + banner) |
| `mission_common` | Shared helpers: `config_loader.py` (every mission parameter + default), `mavros_utils.py` (arm, takeoff, goto, …), `srv_helpers.py` (service names) |
| `mission_bringup` | Launch files + `config/mission_params.yaml` (all tunable values) |
| `payload` | `payload_node`: releases the payload on `/payload/release` |

Outside `ros2_ws`: `camera_ws/src/camera_ros` (Pi camera driver, built on the Pi only,
with a one-line fix for the Pi's libcamera) and `tools/` (standalone scripts:
`waypoint_mission.py` pymavlink flight test, `green_detect.py`, camera viewers).

### Coordinate conventions (common source of bugs)

- **ENU yaw**: radians, 0 = East, counter-clockwise positive (what MAVROS takes).
- **Compass heading**: degrees clockwise from North (`/mavros/global_position/compass_hdg`).
  Conversion: `yaw_enu = radians(90 − heading)`.
- Local frames are metres, x = East, y = North, unless a module says otherwise (the
  arena search uses its own **arena frame**, see phase 3).
- **Downward camera mounting**: image top = the drone's nose (`cam_yaw_offset_deg` / `cam_yaw`
  if it's mounted otherwise).

---

## 4. The phases in detail

### Takeoff (shared by everything)

`MissionNode._takeoff()` — the same gradual takeoff as `tools/waypoint_mission.py`:

1. Check the position estimate holds still on the ground for 3 s (max drift 0.8 m).
2. GUIDED, arm, then **sit armed for 7 s** (a deliberate safety pause — gives the pilot
   time to abort; don't remove it). Drift is still checked.
3. Lift off to 0.5 m, then slide the altitude target up at 0.5 m/s to `cruise_altitude_m`
   (5 m), holding the heading the drone was placed with.

That placed heading is remembered (`placed_heading_deg`) and **held through phases 1
and 2** — the banner search is centred on it. (Lesson learned: in GUIDED, a move with no
heading makes ArduCopter turn the nose toward the target, so every move passes the
heading explicitly.)

### Phase 1 — reference QR (`reference_qr.py`)

At 5 m over the launch point:

1. **Find**: `/perception/locate_qr` reports where a QR is relative to the drone
   (detection only — works before it's readable). None in view → step 0.5 m forward
   along the placed heading, up to 4 steps. (At 5 m the camera sees ~±2 m, so a QR
   1 m ahead is normally in view without stepping.)
2. **Centre** over it (up to 2 correction moves).
3. **Read**: `/perception/scan_qr` — the same QR in 3 frames in a row. Retries
   re-centre first.
4. **Store**: `qr_scanner_node` writes `~/aerothon_captures/reference_qr.json` (+ `.png`
   and a snapshot) and publishes it on `/qr/reference` (latched).

#### Why "picture matching"

The organisers said the QR may contain no information. So the scanner stores the QR's
**picture** too: it straightens the QR from its 4 corners and reads its grid of
black/white squares (`qr_fingerprint` in `qr_pipeline.py`). Two prints of the same QR
give the same grid; different QRs agree on only ~50–60 % of the data squares.

- Both QRs decoded → the **text** decides (different text is never a match).
- Either has no text → the **picture** decides (≥ 85 % of data squares agree,
  `match_min_similarity`).

Tested in simulation: same QR scores 0.93–1.00, others ≤ 0.59, from 2–10 m, tilted,
blurred.

### Phase 2 — entry corridor (`MissionNode._entry_corridor_phase()` + `path_planner/entry_corridor.py`)

1. Fly back over the takeoff point at 5 m (the QR search may have moved the drone),
   descend to `corridor_start_alt_m` (3 m), nose on the placed heading.
2. **Banner search** (`path_planner_node._banner_search`): sweep the nose **±90°** around
   the placed heading (left first — corridor 1 is ahead-left), then right. Not seen →
   next altitude (3.0 … 1.0 m in 0.5 m steps), sweeping back the other way.
   The banner is never behind the drone, so no 360° turns.
3. **Fix the banner**: each frame's banner shape (the bottom edge's elevation at both
   ends) gives the entrance line relative to the nose — no compass needed; the banner's
   height comes from several frames (least squares). The corridor altitude = half the
   banner height (middle of the opening).
4. Fly to a **line-up point** 2.5 m straight out from the entrance, then **line up on
   the walls** under the banner (depth + floor edge in the colour image — the walls are
   white).
5. **Fly through** at the middle of the opening until the walls are seen to end.
6. **Climb straight up to `post_entry_alt_m` (10 m).**

**Staying centred (GPS drift, wind).** The corridor is flown on the WALLS, not on GPS:
every cycle (5 Hz) the follower measures where the walls are relative to the drone
and steps toward the centre (≤ 0.2 m sideways per step). GPS only turns "0.2 m left"
into a target. **Centring first**: more than 0.10 m off centre, the forward step shrinks
(to a quarter at 0.30 m off — `corridor_centre_slow_m`, `corridor_centre_min_advance`),
so more of the speed goes sideways. The last ~2.5 m of a wide corridor are blind (the
OAK-D sees ±36°: walls 1.75 m to the side are only in view from ~2.4 m ahead), so the
"walls end — fly out" run holds the centreline **as last measured** (in GPS terms) —
wind pushes are corrected there; GPS error cannot be until walls are seen again.

SITL, 3.5 m corridor, true distance from the centreline:

| Case | Max | RMS |
|---|---|---|
| No wind (fixed 0.5 m GPS error) | 0.08 m | 0.03 m |
| 3 m/s crosswind, 1 m/s gusts | 0.02 m | 0.02 m |
| Wind + GPS wandering ±0.3 m over 60 s | 0.14 m | 0.06 m |
| Wind + extreme GPS drift ±0.5 m over 20 s | 0.67 m | 0.37 m |

Robustness rules that came out of SITL testing:
- Only fresh banner detections (**LOCK**) steer. While the tracker is coasting (**HOLD**)
  the drone waits on its last good heading — steering on stale HOLD frames while turning
  chased the banner out of view.
- Banner lost for more than 5 s (`entry_research_after_s`) → one ±90° sweep around where
  it was last seen; still not found → `NO_BANNER` (the mission lands) instead of hovering
  until the timeout.

### Phase 3 — arena search (`mission_manager/arena_search.py`)

1. **Plan** (`path_planner/arena_frame.py`): the arena comes from its **4 GPS corners**
   (`arena_corners_lat/lon`). All planning happens in an **arena frame** (metres along two
   sides from a corner), so the existing sweep/re-plan code works on any rotation. Of the
   8 ways to lay the frame on the rectangle, it picks the shortest sweep from where the
   drone is (strip 0 nearest it).
2. **Strip spacing**: the nose points along the strips, so the image's wide side looks
   across them (~13 m at 10 m). Strips are spaced so a whole 1 m QR is always in the
   frame between two strips: ~11.8 m max → 3 strips for the 30 × 40 m arena (~140 m to
   fly, ~5 min at 0.5 m/s).
3. **QR matching**: `qr_scanner_node` Phase 5 (`/perception/start_qr_scan`) checks every
   QR in view against the stored reference (text or picture) and publishes confirmed
   matches on `/qr/detection` (`"content|lat|lon"`). A match outside the arena (e.g. the
   phase 1 QR) is ignored.
4. **Red mat** (same logic as `bcd_redzone_mission.py`): `red_zone_node` reports new red in
   view → **brake** → hover-confirm 4 s (`/red_zone/confirm`) → the mat + **2 m margin**
   is cut out of the arena → if the drone stopped inside the margin it moves straight
   out → the rest of the sweep is re-planned with every hop routed **around** the mat.
   Unreachable strip pieces are skipped, never flown across.
   There is only one mat, so after the first confirm `red_zone_node`
   (`grow_known_zones`) adds red **connected to it** while flying — no stop per slice;
   the outline is re-published and the sweep re-plans on the move. Red not connected to
   it still triggers a stop-and-confirm.
   Red camera silent for more than 0.5 s → brake and hold ("blind"); silent 10 s → abort.
5. **On a match**: brake, fly over it, scan again to refine (up to 2 passes, ≤ 1 m).
   Returns the QR's GPS position for phase 4.

Speed: **0.5 m/s** for now (the proven `bcd_sweep_mission` was flown at 0.25–0.5 m/s).
The flight method — hold, then `DO_CHANGE_SPEED`, then short position-target bursts — is
copied from that proven code.

### Phase 4+ — drop, corridor 2, home (not reworked yet)

`MissionNode._drop_payload()` flies over the match at `drop_altitude_m` (5 m), settles,
calls `/payload/release`. Then `/path_planner/fly_corridor_2` (obstacle avoidance on),
fly home, land. **These are the next phases to rework** — see Open issues.

---

## 5. Test flights — one per phase

| Test | Drone | SITL |
|---|---|---|
| Takeoff (5 m, hover 10 s, land) | `ros2 launch mission_bringup takeoff_test.launch.py` | add `sim:=true` |
| Phase 1 (QR) | `ros2 launch mission_bringup qr_scan_mission.launch.py` | add `sim:=true` |
| Phase 2 (entry corridor) | `ros2 launch mission_bringup entry_corridor.launch.py takeoff_alt:=5` | `ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true mission:=entry_corridor` |
| Phase 3 (arena, 25 × 25 m) | `ros2 launch mission_bringup arena_search_mission.launch.py` | add `sim:=true` |
| Whole mission | `ros2 launch mission_bringup mission.launch.py launch_mavros:=true launch_camera:=true` | `ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true` |

What each test does after its phase:
- **Phase 2 test**: climbs to 10 m after the corridor, hovers 5 s, lands.
- **Phase 3 test**: the drone takes off at a **corner** of the test area; the area runs
  25 m **ahead along its nose** and 25 m **to its right**. Over the match: hover 5 s, land
  there. It loads the reference from `~/aerothon_captures/reference_qr.json`, so run
  phase 1 first (SITL makes its own reference).

On the real drone, every flight: place it **facing the corridors** (phase 2 only looks
±90°), RC pilot ready, check the fence ceiling allows 10 m.

### SITL (simulation on a laptop)

```bash
# terminal 1 — the simulated flight controller
cd ~/ardupilot/Tools/autotest
python3 sim_vehicle.py -v ArduCopter -f quad --no-mavproxy -w -l 12.9716,77.5946,0,0

# terminal 2
source ~/AeroTHON26/env.sh
ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true
```

`sitl_mission.launch.py` simulates the whole world: corridors (competition size),
the green banner, the OAK-D depth, the downward camera with the reference QR 1 m ahead,
the arena (30 × 40 m) with 6 QRs and a red mat. Real perception nodes run on the
simulated images. Move corridor 1 with `c1_ahead_m`, `c1_left_m`, `c1_yaw_deg`.
`mission:=takeoff|qr_scan|entry_corridor|full` picks the flight. Disturbances:
`wind_ms`, `wind_dir_deg`, `wind_turb_ms` (ArduPilot SITL wind) and `gps_drift_m`,
`gps_drift_period_s` (a drifting GPS error); `sitl_corridor_sim` logs the true
distance from the centreline after each corridor pass.

Tips: SITL simulates a 3S battery, so the launch files lower `battery_critical_v` in SITL
only. A cold SITL needs ~30–60 s before it has a position (the takeoff waits).

### Unit tests

```bash
cd ~/AeroTHON26/ros2_ws/src
python3 -m pytest mission_manager/test path_planner/test qr_detection/test obstacle_avoidance/test
```

252 tests: QR picture matching, the banner search/approach (closed loop in a fake world),
the arena geometry, the state machine, red-zone re-planning, corridor geometry.

---

## 6. Configuration

Everything tunable is in `ros2_ws/src/mission_bringup/config/mission_params.yaml`
(defaults in `mission_common/config_loader.py`). The box at the top lists what changes on
**competition day** — most importantly `arena_corners_lat` / `arena_corners_lon`.

Key values:

| Parameter | Value | Meaning |
|---|---|---|
| `cruise_altitude_m` | 5.0 | takeoff + reference QR height |
| `takeoff_liftoff_alt_m`, `takeoff_climb_rate_ms` | 0.5, 0.5 | gradual takeoff |
| `corridor_start_alt_m` | 3.0 | descend to this before the banner search |
| `entry_search_half_angle_deg` | 90 | banner sweep each side of the placed heading |
| `entry_research_after_s` | 5.0 | banner lost this long → sweep again |
| `post_entry_alt_m` | 10.0 | climb after the corridor |
| `arena_alt_m` | 10.0 | arena sweep height |
| `arena_sweep_speed_ms` | 0.5 | arena sweep speed |
| `arena_red_margin_m` | 2.0 | distance kept from the red mat |
| `arena_qr_size_m` | 1.0 | sets the strip spacing |
| `drop_altitude_m` | 5.0 | payload drop height |
| `match_min_similarity` (qr_scanner_node) | 0.85 | picture-match threshold |

Team rule: **don't change an existing value without talking about it first** — a silently
changed threshold can crash the drone. New parameters get their default written in the
yaml and mentioned when the change is shared.

---

## 7. Setup, environment gotchas

- `./setup.sh` installs everything (apt, rosdep, pip) and builds; `./setup.sh --build`
  only rebuilds. `source ~/AeroTHON26/env.sh` in every terminal.
- **ROS nodes run on the system Python** (`/usr/bin/python3`). Don't run them from a
  venv: a venv's NumPy 2 breaks ROS Jazzy's `cv_bridge`.
- **OpenCV must stay 4.x** (pinned 4.10 in `requirements.txt`). OpenCV 5 changed image
  type codes and every camera node crashes (`KeyError: 16` in `cv_bridge`).
- On the Pi, `setup.sh` also builds `camera_ws` (the Pi camera driver) against the Pi's
  own libcamera, and adds you to the `dialout` group for the serial port (log out and in).
- The code is in GitHub repo **AeroTHON26**. Work on a branch and open a pull request;
  don't push straight to `main`.

---

## 8. Status

| Part | Code | SITL | Real drone |
|---|---|---|---|
| Gradual takeoff | ✅ | ✅ | ⬜ |
| Phase 1 — reference QR (incl. no-text QR) | ✅ | ✅ | ⬜ |
| Phase 2 — entry corridor | ✅ | ✅ (alone and in the full mission, different layouts) | ⬜ |
| Phase 3 — arena search | ✅ | ✅ (alone and in the full mission; red mat: 1 stop) | ⬜ |
| Phase 4 — payload drop | old code | runs, has a bug (below) | ⬜ |
| Corridor 2, homing, landing | old code | not re-checked | ⬜ |

`mission_manager/bcd_sweep_mission.py` (a simple lawnmower) **has flown on the real drone**
and is the proven reference for how to fly a sweep; it and `bcd_redzone_mission.py` are kept
as standalone tests (unchanged apart from the throttle cap). The old `qr_bcd_mission.py`
was removed — `arena_search_mission` is the full QR-search test now.

### To do on the Pi / drone (in this order)

1. Clone the repo on the Pi, run `./setup.sh` (first full run anywhere — watch apt/rosdep,
   the camera driver build, the OpenCV/NumPy check).
2. Check the flight controller's fence ceiling (`FENCE_ENABLE`, `FENCE_TYPE`,
   `FENCE_ALT_MAX`) allows 10 m. Don't change FC parameters without agreeing first.
3. Takeoff test.
4. Phase 1 test with the printed QR ≤ 1 m ahead: does the Cam3 find it at 5 m? Is
   `cam_yaw` right for how it's mounted? Real QR size (1 m assumed)?
5. Phase 2 test at a real corridor with the green banner: banner detection in real light,
   OAK-D on the white walls, the 10 m climb.
6. Phase 3 test on a 25 × 25 m area with printed QRs and a red mat: is the QR readable from
   10 m with the real camera (sim says yes at 1280×720; if not, 1920×1080)? Red detection
   in real light (HSV values are untuned for the venue).
7. Phases together.

### Open issues / decisions

- **Mission without a target**: if the reference QR can't be read, the mission currently
  **skips the arena search**. Not confirmed yet — open question.
- **"Blind" stops in the arena**: the sweep brakes if the red detector is silent for
  more than 0.5 s (value from the proven 2 m flights). On a laptop SITL this happened 0–6
  times per run, each over in a second or two. At 10 m the camera sees 3.8 m ahead, so
  ~1.5 s would likely still be safe — undecided.
- **Corridor blind end**: the last ~2.5 m of a wide corridor can't be centred on the
  walls (camera field of view); GPS error there isn't corrected. Fine with realistic GPS
  (SITL: ≤ 0.14 m), worth watching on the real drone.
- **Phase 4 bug**: `_drop_payload()` only checks the horizontal distance to the QR. Since
  the drone is already over it, it "arrives" immediately and may release before reaching
  5 m. To fix when phase 4 is reworked.
- **Later phases to define and rework**: drop (5 m), corridor 2 (obstacles), homing.
  Altitudes after the arena are "mostly 10 m, 5 m for the drop".
