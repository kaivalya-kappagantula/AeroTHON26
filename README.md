# AeroTHON26

Autonomous mission stack for the AeroTHON 2026 drone: ROS 2 Jazzy on a Raspberry
Pi 5, talking to an ArduPilot (ArduCopter) Pixhawk through MAVROS.

**New to the project? Start with [docs/PROJECT_GUIDE.md](docs/PROJECT_GUIDE.md)** — the
mission, the architecture, how every phase works and why, testing, and what's open.

The mission flies **hardcoded GPS waypoints** (`wp_*` in
`ros2_ws/src/mission_bringup/config/mission_params.yaml`, from the site survey). Every
phase starts and ends on them; only four parts have logic, the rest are plain waypoints:

| # | Phase (log file) | From -> to | How |
|---|---|---|---|
| 0 | `0_preflight` | on the ground | GPS, waypoints checked, OAK-D + down camera up |
| 1 | `1_takeoff_qr` | `wp_home` (takeoff 5 m) -> `wp_qr` | **logic**: find, centre, read the reference QR |
| 2 | `2_entry_corridor` | `wp_entry_mouth` -> `wp_entry_end` at 5 m, up to 9.25 m | waypoints (flown over) |
| 3 | `3_arena_search` | from the entry end | **logic**: BCD sweep around the red mat until the matching QR |
| 4 | `4_payload_drop` | over the match | **logic**: down to 5 m, centre, release, back up |
| 5 | `5_exit_corridor` | 6 m in front of `wp_exit_mouth` -> `wp_exit_end` at 1.22 m | **logic**: obstacle avoidance (failure: up to 5 m, home, land) |
| 6 | `6_home_land` | `wp_exit_end` -> `wp_home` at 5 m | waypoints, land |

## Quick start

Needs **Ubuntu 24.04 + ROS 2 Jazzy** ([install guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)).

```bash
git clone <repo-url> ~/AeroTHON26
cd ~/AeroTHON26
./setup.sh                    # apt + rosdep + pip packages, then builds everything
source ~/AeroTHON26/env.sh    # in every new terminal (or add it to ~/.bashrc)
```

`setup.sh` works on the Pi and on laptops; on the Pi it also builds the Pi camera
driver (`camera_ws`). After a `git pull`, rebuild with `./setup.sh --build`.

Do **not** run the ROS nodes from a Python venv, and do not upgrade OpenCV to 5.x or
NumPy to 2.x for the system Python: ROS Jazzy's `cv_bridge` breaks and every camera
node crashes. `requirements.txt` pins safe versions.

## Fly the mission (one command)

```bash
./fly.sh          # on the drone: MAVROS on /dev/ttyAMA2, OAK-D, down Cam 1.3, all phases, land
./fly.sh sim      # on a laptop: ArduPilot SITL at wp_home + simulated cameras, same code
```

Before the first flight, on the drone with props off: `./tools/pi_check.sh` tests the
OAK-D and Cam 1.3 feeds and live views, then runs every mission node at once for 2 min
and checks CPU, RAM, temperature, 5 V supply and throttling (report + snapshots in
`~/aerothon_logs/pi_check_<time>/`).

Place the drone **on `wp_home`, facing the arena**. More than 30 m from `wp_home` (wrong
field / yaml), no GPS, no OAK-D or no camera frame -> it refuses to take off.

- **Nodes per phase**: MAVROS, payload and QR scanner run throughout; OAK-D and down
  camera from preflight to landing; `red_zone_node` from phase 3 to the exit corridor's
  start point; `path_planner` + `corridor_perception` in phase 5 only.
- **Live view** (drone's WiFi): `http://<pi>:8080/` down camera, `http://<pi>:8081/`
  OAK-D. `./fly.sh record_video:=true` also saves both.
- **Logs**: `~/aerothon_logs/mission_<time>/` — `summary.log` (one line per phase:
  result, time), `<n>_<phase>.log` (every node's messages during that phase + a 1 Hz
  `TELEM` line), `stage_<name>.out` (console of each phase's nodes).
- SITL needs ArduPilot in `~/ardupilot` (below).

Single-phase test flights still exist (`takeoff_test`, `qr_scan_mission`,
`arena_search_mission`, `payload_drop_mission` launch files, `sim:=true` for SITL).

## SITL (laptop)

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile && ./waf configure --board sitl && ./waf copter
```

then `./fly.sh sim`.

## Tests

```bash
cd ~/AeroTHON26/ros2_ws/src
python3 -m pytest mission_manager/test path_planner/test qr_detection/test obstacle_avoidance/test
```

## Layout

```
ros2_ws/src/        the ROS 2 packages (details: ros2_ws/src/README.md)
  mission_manager     mission sequence + per-phase test flights
  path_planner        entry corridor, corridors, arena search
  obstacle_avoidance  OAK-D depth, corridor walls, green banner, SITL corridor world
  qr_detection        QR scanner (text + picture matching), red zones, sim camera
  mission_common      shared helpers, parameters
  mission_bringup     launch files + mission_params.yaml (all tunable values)
  payload             payload release
camera_ws/src/camera_ros   Pi camera driver (upstream camera_ros, MIT, with a one-line
                           fix for the Pi's libcamera); built on the Pi only
tools/              standalone scripts: waypoint_mission.py (pymavlink flight test),
                    green_detect.py, camera live views, MAVLink check
setup.sh, env.sh, requirements.txt
```

## Working together

Don't push straight to `main`: make a branch, push it, open a pull request.

```bash
git checkout -b my-change
git add -A && git commit -m "what changed"
git push -u origin my-change
```
