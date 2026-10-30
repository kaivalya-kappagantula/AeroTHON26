# AeroTHON26 — Project Guide

Everything you need to understand, run and change this code: what the drone has to
do, how the software is built, how each mission phase works and why it works that
way, how to test it, and what is still open.

For the quick "clone and run" steps, see the [README](../README.md). This guide is
the long version.

---

## 1. What the drone has to do

The AeroTHON 2026 mission, flown fully autonomously, in **under 15 minutes**:

1. **Reference QR.** Take off. A "QR" lies on the ground at most ~1 m ahead of the
   launch point. Read it from 5 m and remember it. The "QRs" are **not real QR codes**
   but **AprilTags (36h11)** — the one sample seen is tag id 2 — so they are known by
   their decoded tag ID, and by their picture when one doesn't decode.
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

**Phase failsafe: a failed phase moves the mission on to the next one** (see below).
If the reference QR can't be read, the mission carries on without a target. If the
whole arena is swept with no match (or with no target), the payload goes on a **random
QR** seen in the sweep (inside the arena, clear of red); none seen → payload kept. If
the payload doesn't drop properly, the mission moves on to corridor 2.

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
| Payload winch | Motor spool, driver on FC relays RELAY1 (pin 50) / RELAY2 (pin 51), on/off = full speed: reels IN from payload_node start to hold the 100 g payload on its 5 m thread; at the drop OUT for `winch_unwind_s` (~4.7 m) then IN at once (the jerk throws it off), IN until landing; both off when it disarms after a flight |
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
  A failure in a phase (leg failed, timeout, crash) → **phase failsafe**: stop, move on
  to the next phase (`MissionNode._phase`); pilot / FC / battery / dead-sensor aborts
  still end the mission. Takeoff failures still land / stay on the ground. In the exit
  corridor (phase 5) — whose next phase is going home — any
  failure there (a planner failure, a companion "failure" abort or a crash) → **exit
  corridor failsafe**: straight up to `exit_failsafe_alt_m` (5 m, above the walls), fly
  home — around the red zones at `arena_alt_m` once the arena has been flown, never a
  straight line over the mat — and land (`MissionNode._exit_failsafe`). Inside the exit
  corridor leg path_planner owns the obstacle-sensor timeout, so it backs out from
  under the banner before the climb.
- **Mission clock** (from lift-off, `mission_time_limit_s` 900 s): the arena sweep ends
  early so `mission_drop_reserve_s` + `mission_exit_reserve_s` are still left; a
  `CLOCK` log line at every phase (and `mission_left_s` in `/mission/telemetry`) shows
  where the time went — tune the reserves from those.
- **Downward camera** in `mission.launch.py` is configured exactly as in the phase
  test launches (node `camera`, BGR888, manual focus, daylight white balance); the
  mission sets the focus for 5 m (phase 1), 10 m (sweep) and 5 m (drop).
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

### Site survey (GPS points measured on site — `mission_common/survey.py`)

`mission_params.yaml`'s `/**` block holds the takeoff spot, the reference QR and both
corridors' wall corners (named from the drone's view while flying **through** that
corridor; *mouth* = the banner side). Each corridor's centreline is a least-squares fit
through all its corners with the known 3.5 m width — measured pairs come out 2.97–3.55 m
apart (GPS scatter), and a missing corner is the other wall + the width. At takeoff
`mission_node` places the survey on today's home fix (drone on `survey_home` within 3 m:
everything, arena corners included, is shifted by the difference — the GPS error common
to both) and publishes it on `/mission/survey_offset`; more than 100 m from it (SITL,
another field) the survey is off and the old banner search runs.

With a survey: phase 1 expects the QR at `survey_qr` (flies there if it is not in view);
phase 2 descends over the QR, flies to the line-up point 2.5 m in front of the surveyed
mouth, down to half the banner height, and looks for the **green banner** for 4 s — seen,
its centre moves the line sideways (≤ 1.5 m) and its bottom edge sets the altitude;
**not seen, it goes in on the surveyed line anyway**. Phase 5 is the same with the
**blue banner**, from a point 6 m in front of the exit corridor's mouth (reached at 9 m
around the red mat). A path to a line-up point that would cross a corridor is refused
(`WRONG_SIDE`). Walls still correct the line in the corridor, and the "not in the
corridor" check still applies.

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

At 5 m over the launch point. The QR lies **~1 m straight ahead of the takeoff point**
(along the heading the drone was placed with), so:

0. **Expect it there**: that spot is sent to `qr_scanner_node` (`/qr_scanner/locate_hint`).
   Locate then uses only a QR within `qr_expected_radius_m` (2.5 m — GPS drift) of it,
   the nearest one: clutter elsewhere at the launch site is never flown to. The hint is
   cleared after phase 1 (the payload drop looks anywhere). From over takeoff at 5 m the
   QR 1 m ahead is fully in view (frame ±1.9 m along the nose), so normally the first look
   finds it.
1. **Find**: `/perception/locate_qr` reports where a QR is relative to the drone
   (detection only — works before it's readable). It looks from over the takeoff point
   first: at 5 m the camera sees ~±1.9 m along the nose, so a QR 1 m ahead is normally
   in view. **Only part of it in view** (cut by the frame edge) is enough: the QR's
   size at this height is known, and the edges of it that are in view are its real
   edges, so the whole square — and its centre, even outside the frame — is worked
   out from that part (`_complete_cut_marker` in `qr_pipeline.py`), and the drone flies
   straight there. Simulation: with ≥ 20 % of the QR in view, 248/251 found, centre
   within 0.24 m (any rotation, 3–6 m); less than ~20 % is not used. Only when nothing
   of a QR is in view: fly 1 m ahead along the placed (compass) heading, then
   ±0.3 / ±0.6 m along the nose, looking after each step.
2. **Centre** over it, forward and sideways, by what the camera sees: locate → move →
   locate again, up to 4 passes, until within 0.3 m. A move longer than `qr_max_jump_m`
   (3 m — e.g. towards a QR cut by the side of the frame) is shortened to 3 m towards it,
   not refused; the next look sees all of it. A wrong compass heading only
   turns each move a little off line and the next look corrects it — the drone no
   longer flies diagonally away from the QR.
3. **Read**: `/perception/scan_qr` — the same picture in 3 frames in a row. Retries
   re-centre first.
4. **Store**: `qr_scanner_node` writes `~/aerothon_captures/reference_qr.json` (+ `.png`
   and a snapshot) and publishes it on `/qr/reference` (latched).

#### Tag ID first, picture as fallback

The "QRs" are not QR codes but AprilTags 36h11 (`tag_dict`). `qr_pipeline.detect_markers`
decodes them with OpenCV's `cv2.aruco` (on a frame scaled so the tag is ~96 px — fast on
the Pi) and names each by its ID, e.g. `tag36h11:2`. Both sides decoded → the **IDs
decide**; a different ID is never a match.

For a marker that does not decode (blur, a corner out of the frame, or markers that
turn out not to be tags) it falls back to the picture. It finds a marker as a patch that is **colourless** (not grass, not the red mat), **busy or
very dark/bright**, and **the size a 1 m square has at this height**
(`camera_fx × marker_size_m / alt`). It assumes no border: a pattern with white cells on
its edge is found too, with an outline up to ~20 % small. A marker cut by the frame edge
is reported (to steer towards) but never matched.

A real QR code inside a found square is decoded too (`qr:<text>`, from the straightened
crop), in case the markers are QRs after all: different QRs look too alike as pictures.

The picture: the marker straightened to 48 × 48 grey, brightness-normalised. Two
pictures are compared by correlation, best of the 4 rotations, ±20 % stretch and a
small shift (`fingerprint_similarity`); match at ≥ `match_min_similarity` (0.78).

Tested in simulation (2–10 m, tilted, turned, blurred, noisy), 8 markers of each kind:

| Marker kind | Known by | Same marker matched | Wrong matches |
|---|---|---|---|
| AprilTag 36h11 (the sample) | tag ID | 48/48 | 0 |
| AprilTag 25h9 / 16h5, ArUco 4×4 / 6×6 | picture | 48/48 each | 0 |
| Random 5–12-cell patterns, with or without border | picture | 48/48 | 0 |
| QR code, no white margin | QR text, else picture | 47/48 | 0 |
| QR code with white margin | QR text, else picture | 42/48 (misses: 10 m, too small to read) | 0 |
| Coarse 3×3 patterns | picture | 39/48 (some not found) | 0 |

Picture scores on random patterns: same marker ≥ 0.84, different ≤ 0.73. The threshold
is set to miss rather than mis-match: a wrong match drops the payload on the wrong marker.
**Not yet tried on a real print** — re-check the threshold with the stored
`reference_qr.png` and arena snapshots, and the detector on the real ground with the
`locate_miss_*.jpg` frames. A test print of the sample: AprilTag 36h11 id 2
(`ros2_ws/src/qr_detection/test/data/sample_marker.png`), printed 1 m with its black
border — the white margin around it is not part of the marker.

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
4. Fly to a **line-up point** 2.5 m straight out from the entrance. The banner fix IS the
   corridor's line in GPS terms; walls seen in the 2 s there correct it, but are not
   needed (on the drone the OAK-D rarely sees both walls of this corridor).
5. **Fly through** at the middle of the opening until the walls are seen to end, or the
   22 ft nominal length + 1 m.
6. **Climb straight up to `post_entry_alt_m` (10 m).**

**Staying centred (GPS drift, wind).** The corridor is flown on its **GPS centreline,
corrected by the walls** (`corridor_follow.GpsCorridorTrack`, details and the hardware
check in `docs/CORRIDOR_HARDWARE_CHECK.md`). On the drone the OAK-D saw both walls of
the 11.5 ft corridor in ~4 % of frames, one in ~88 %, none in ~8 % (it sees ±36°: walls
1.75 m to the side are only in view from ~2.4 m ahead, and stereo on the white walls
fades beyond ~3–3.5 m). Walls-only steering held and failed there. Now each frame with
a wall is a measurement of the line (gated against GPS, ≤ 1 m / 10° correction), and
with no wall in view the drone flies the GPS line. Every cycle (5 Hz) it steps toward
the line (≤ 0.2 m sideways per step); **centring first**: more than 0.10 m off, the
forward step shrinks (to a quarter at 0.30 m off — `corridor_centre_slow_m`,
`corridor_centre_min_advance`). SITL: `wall_depth_max_m:=3.5` reproduces the drone's
wall statistics — use it.

Older SITL table (perfect walls, wall-only steering), true distance from the centreline:

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

### Phase 4 — payload drop (`mission_manager/payload_drop.py`)

Shared by `mission_node`, `payload_drop_mission` and `arena_search_mission drop:=true`.

1. **Over the QR** at 10 m, routed around the red zones (the arena search's `go_to`).
   No route clear of red → the payload is **kept** (never over red).
2. **Straight down** to `drop_altitude_m` (5 m), waiting for the altitude. No slanted
   descent.
3. **Centre with the down camera**: `/perception/locate_qr` (the largest QR in view = the
   one below) → move by its offset, up to `drop_center_passes` (3) times, until within
   `drop_center_tol_m` (0.3 m). No move when the QR seen is more than
   `drop_center_max_offset_m` (2 m) from the matched point (probably another QR) or when
   the move would enter a red zone's 2 m margin.
4. **Release** after `drop_settle_s` (3 s): `/payload/release` → winch OUT for `winch_unwind_s`
   (lowers the payload ~4.7 m on its 5 m thread), then straight back IN: the jerk throws it off
   its hook. Reels in until landing. If it couldn't
   centre, or 5 m wasn't reached in `drop_arrival_timeout_s`, it **releases anyway** on
   GPS (a rough drop scores, carrying the payload home doesn't).
5. **Straight back up** to 10 m (`arena_alt_m`), then phase 5.

"Released" means payload_node's servo command was accepted. Nothing senses that the
payload actually fell.

### Phase 5 — exit corridor, obstacles (`path_planner/corridor_follow.py`)

Flown like the entry corridor (blue banner) with obstacles on. A 3.5 ft obstacle in the
middle of the 11.5 ft corridor leaves 1.22 m each side — 0.29 m per side for the 0.63 m
drone, and the drone loses sight of the obstacle ~0.8 m before it (SITL: down to 0.03 m).
So an obstacle is passed **beside** only if the gap leaves `exit_pass_side_clearance_m`
(0.40 m) each side; otherwise it is flown **over** (top + `body_below_camera_m` +
`climb_clearance_m`, ~2.28 m for a 6 ft obstacle), down the middle. Under the banner the
altitude stays below `banner bottom − body_above_camera_m − climb_clearance_m` until
`banner_climb_after_m` past the mouth: the climb waits there (at the latest
`min_stop_dist_m` before the obstacle). An obstacle that cannot be climbed (too tall, or
too near the mouth to climb before it) is still passed beside at `side_clearance_m`.
**Measure `body_above_camera_m`** (camera to the top of props / GPS mast) and the real
obstacles' depth (`obstacle_length_m`: the drone comes back down this far past the face).

Homing comes next — see Open issues.

---

## 5. Test flights — one per phase

| Test | Drone | SITL |
|---|---|---|
| Takeoff (5 m, hover 10 s, land) | `ros2 launch mission_bringup takeoff_test.launch.py` | add `sim:=true` |
| Phase 1 (QR) | `ros2 launch mission_bringup qr_scan_mission.launch.py` | add `sim:=true` |
| Phase 2 (entry corridor) | `ros2 launch mission_bringup entry_corridor.launch.py takeoff_alt:=5` | `ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true mission:=entry_corridor` |
| Phase 3 (arena, 25 × 25 m) | `ros2 launch mission_bringup arena_search_mission.launch.py` | add `sim:=true` |
| Phase 4 (drop on a QR 5 m ahead) | `ros2 launch mission_bringup payload_drop_mission.launch.py` | add `sim:=true` |
| Phases 3 + 4 | `ros2 launch mission_bringup arena_search_mission.launch.py drop:=true` | add `sim:=true start:=corner` |
| Whole mission | `ros2 launch mission_bringup mission.launch.py launch_mavros:=true launch_camera:=true` | `ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true` |

Whole mission, live view + logs: Cam3 at `http://<pi>:8080/`, OAK colour at
`http://<pi>:8081/` (watch only; frames are dropped, never queued). Each flight writes
`~/aerothon_logs/mission_<time>/`: both streams as `.mjpeg` (`ffplay -f mjpeg <file>`)
and a rosbag of telemetry, MAVROS, perception results and `/rosout` (no images).
`live_view:=false` / `record:=false` turn them off.

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
| `mission_time_limit_s` | 900 | mission clock, from lift-off |
| `mission_drop_reserve_s`, `mission_exit_reserve_s` | 90, 300 | kept for the drop / exit + home when ending the sweep |
| `require_arena_corners` | true | corners all zero → the full mission won't take off |
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
| Phase 3 — arena search | ✅ | ✅ (alone and in the full mission; red mat: 1 stop). 2026-10-30, AprilTags, phases 1 + 3, 4 runs (0.5 and 1.0 m/s, mat across a strip): target matched by tag ID, centred 0.05–0.54 m; closest to the mat edge 2.16–2.28 m (margin 2 m), never inside | ⬜ |
| Phase 4 — payload drop | ✅ | ✅ (alone: centred to 0.03 m with a decoy QR beside it; after phase 3: 0.06 m) | ⬜ |
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
4. Phase 1 test with a printed AprilTag 36h11 (e.g. the sample, id 2 — not a QR
   code) ≤ 1 m ahead: does the Cam3 find it at 5 m on the real
   ground? Is `cam_yaw` right for how it's mounted? Real marker size (1 m assumed —
   `marker_size_m`; `camera_fx` matters too, it sets the size looked for)?
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
- **Corridor walls rarely in view** (real OAK-D, 11.5 ft corridor): the corridor is now
  flown on the GPS line, walls correct it. Check on the drone with the walk-through dry
  run before flying (`docs/CORRIDOR_HARDWARE_CHECK.md`).
- **Payload winch on relays (bench-check)**: now switched with DO_SET_RELAY (RELAY1/2 on
  pins 50/51), not a servo PWM. Time `winch_unwind_s` for ~4.7 m with the payload on (never the
  full 5 m: the spool would wind the thread back on the other way), and check one IN jerk
  throws the payload off (else `winch_jerks` 2–3).
- **Phase 4 on the drone**: check the winch on the bench first
  (`ros2 service call /payload/release std_srvs/srv/Trigger`), and the FC's relay setup
  (RELAYn_PIN / FUNCTION / DEFAULT, SERVO9/10_FUNCTION -1; see `payload_node.py`). Is the
  1 m QR still found by `locate_qr` at 5 m with the real camera and focus?
- **Later phases to define and rework**: corridor 2 (obstacles), homing.
