# AeroTHON26

Autonomous mission stack for the AeroTHON 2026 drone: ROS 2 Jazzy on a Raspberry
Pi 5, talking to an ArduPilot (ArduCopter) Pixhawk through MAVROS.

**New to the project? Start with [docs/PROJECT_GUIDE.md](docs/PROJECT_GUIDE.md)** — the
mission, the architecture, how every phase works and why, testing, and what's open.

| Phase | What it does | Status |
|---|---|---|
| 1 | Gradual takeoff to 5 m, find + centre over the reference QR, read it and store it (by its text, or by its picture if it holds none) | SITL ✅, drone not yet |
| 2 | Entry corridor: descend to 3 m, find the green banner (±90° sweep), line up on the walls under it, fly through, climb to 10 m | SITL ✅, drone not yet |
| 3 | Arena search: sweep the arena at 10 m (strips sized so a 1 m QR always fits), never over the red mat (stop, confirm, re-plan around it with a 2 m margin), find the QR that matches the reference | SITL ✅, drone not yet |
| later | Payload drop, corridor 2, return home | in progress |

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

## Run one phase at a time

Every phase has its own test flight. Each flies exactly the mission's code for that
phase, then lands.

| Test | On the drone | In SITL |
|---|---|---|
| Takeoff only (climb to 5 m, hover 10 s, land) | `ros2 launch mission_bringup takeoff_test.launch.py` | `ros2 launch mission_bringup takeoff_test.launch.py sim:=true` |
| Phase 1 (QR) | `ros2 launch mission_bringup qr_scan_mission.launch.py` | `ros2 launch mission_bringup qr_scan_mission.launch.py sim:=true` |
| Phase 2 (entry corridor) | `ros2 launch mission_bringup entry_corridor.launch.py takeoff_alt:=5` | `ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true mission:=entry_corridor` |
| Phase 3 (arena search, 25 x 25 m test area) | `ros2 launch mission_bringup arena_search_mission.launch.py` | `ros2 launch mission_bringup arena_search_mission.launch.py sim:=true` |
| Whole mission | `ros2 launch mission_bringup mission.launch.py launch_mavros:=true launch_camera:=true` | `ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true` |

On the drone:
- Place it facing the corridors (the banner search only looks ±90° around where the
  nose points), with the reference QR at most ~1 m ahead.
- It sits armed for 5 s before every takeoff. The RC CH7 switch starts the mission
  where the launch file asks for it; switching CH7 low, or changing flight mode, hands
  control back to the pilot.
- Phase 2 climbs to 10 m after the corridor: check the fence ceiling
  (`FENCE_ALT_MAX`) allows it, and that the sky above the corridor exit is clear.

The stored reference QR ends up in `~/aerothon_captures/reference_qr.json` (+ `.png`
and a snapshot). The phase 3 test loads it from there, so fly (or bench-run) phase 1
first. For phase 3 place the drone at a corner of the test area: the area runs ahead
along its nose and to its right. On competition day put the arena's 4 GPS corners in
`arena_corners_lat` / `arena_corners_lon` (`mission_params.yaml`).

## SITL (laptop)

SITL needs ArduPilot built separately (it is too big to live in this repo):

```bash
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git ~/ardupilot
cd ~/ardupilot && Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile && ./waf configure --board sitl && ./waf copter
```

Then start the simulator in one terminal:

```bash
cd ~/ardupilot/Tools/autotest
python3 sim_vehicle.py -v ArduCopter -f quad --no-mavproxy -w -l 12.9716,77.5946,0,0
```

and a test from the table above in another (after `source ~/AeroTHON26/env.sh`).
`sitl_mission.launch.py` simulates the depth camera, the corridors and the green banner.
Move corridor 1 to match a test field with `c1_ahead_m`, `c1_left_m` and `c1_yaw_deg`
(entrance distance ahead / to the left of the takeoff point, and corridor angle).

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
