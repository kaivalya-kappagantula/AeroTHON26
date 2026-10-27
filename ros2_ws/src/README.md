# Aerothon 2026 Mission 2 drone stack

ROS 2 Jazzy + MAVROS + ArduPilot Copter 4.7, on a Raspberry Pi 5 with a Pixhawk 6C Mini,
an OAK-D Lite (forward depth) and an RPi Camera Module 3 (downward, QR codes).

Mission: phase 1 = read the reference QR on the ground → take off → find the green
banner and fly the entry corridor under it → climb to cruise; then lawnmower search for
the matching QR → drop the payload on it → corridor 2 (obstacles) → fly home → land.
State machine: `mission_manager/state_machine.py`.

Corridor width and length are not configured: the width is measured from the walls
(pairs 1.0–4.0 m apart are accepted) and a corridor ends where its walls are seen to
end. Obstacles are 30–40 % of the width. Corridors are flown by `path_planner` from
the OAK-D corridor fit (`corridor_perception_node`, walls from stereo depth and from
the wall bases in the colour image — white walls give little depth): stay centred,
move to the free side of an obstacle, or climb over one that leaves no side gap.
ArduPilot's BendyRuler is off — at 5 ft it cannot pass the obstacles
(`obstacle_avoidance/params/README.md`).

**Entry corridor (end of phase 1, `/path_planner/entry_corridor`, state
`ENTRY_CORRIDOR`).** Two corridors stand side by side; the one to enter has a green
banner across the top of its entrance (position and height unknown). Starting where it
took off, the vehicle turns a full circle looking for it (then again at
0.5 m altitude steps) and flies toward it. From the banner's shape in each frame (its
level bottom edge looks steeper at the near end) it reads where the entrance is and which
way the corridor runs, and from frames at different places or heights how high the
banner is — no sizes, and the compass error is not amplified. It then flies to a point
2.5 m straight out in front of the entrance (so it can come from an angle: a wide
corridor's walls are only visible from nearly in line), lines up on the walls under the
banner — not the neighbour's — and flies through at half the banner's height (the middle
of the opening) until the walls end, then climbs straight up to cruise altitude. Logic:
`path_planner/entry_corridor.py`.

## Packages

| Package | Role | Executables |
|---|---|---|
| `mission_bringup` | **Start here.** Launch files, `config/mission_params.yaml`, SITL script | `mission.launch.py`, `sitl_mission.launch.py`, `run_sim.sh` |
| `mission_common` | Shared library: parameters (`config_loader`), MAVROS helpers (`mavros_utils`), service names (`srv_helpers`) | — |
| `mission_manager` | Mission state machine and the node that runs it; SITL QR stand-in; standalone flights: BCD sweep with red-zone avoidance, the full QR search (reference QR -> sweep -> land on the match), and the entry corridor test (take off -> entry corridor -> land) | `mission_node`, `sitl_qr_simulator`, `flight_primitives`, `bcd_redzone_mission`, `qr_bcd_mission`, `entry_corridor_mission` |
| `path_planner` | Flies each leg as GUIDED position targets: lawnmower routed around red zones; corridors followed from perception (`corridor_follow.py`); entry corridor: banner search, approach, line-up (`entry_corridor.py`) | `path_planner_node` |
| `payload` | Payload release servo | `payload_node` |
| `qr_detection` | Downward camera: QR detection, confirmation gate, ground projection; corridor entrance/exit colour markers; red (no-fly) zone detection; SITL downward-camera renderer | `qr_scanner_node`, `ground_marker_node`, `red_zone_node`, `sim_down_camera_node` |
| `obstacle_avoidance` | OAK-D depth → 72-bin scan → ArduPilot proximity; sensor-health signals; corridor fit (walls from depth + colour wall bases, measured width, centring error, obstacles, entrance/exit); green entrance banner (colour). SITL depth stand-in. Bench tools in `scripts/`, FC parameter sets in `params/` | `sector_builder_node`, `obstacle_publisher_node`, `coverage_guard_node`, `corridor_perception_node`, `banner_node`, `sitl_corridor_sim`, `sector_viz`, `synthetic_sector_node` |
| `depthai-ros` | Luxonis OAK driver, built from source (separate repo) | — |

`mission_manager`, `path_planner`, `payload` and `obstacle_avoidance` depend on
`mission_common` (`corridor_msg.py` defines the `/corridor/perception` layout);
`mission_bringup` depends on everything; `qr_detection` stands alone.

## How the nodes talk

```
mission_node ── /perception/scan_qr, /perception/start_qr_scan ──▶ qr_scanner_node
     │       ── /path_planner/entry_corridor, run_lawnmower, fly_corridor_2, stop ──▶ path_planner_node ──▶ MAVROS (GUIDED)
     │       ── /payload/release ──▶ payload_node ──▶ MAVROS (DO_SET_SERVO)
     │◀── /qr/detection ("content|lat|lon", after the confirmation gate)
     │◀── /avoidance/sensor_live, /avoidance/coverage_ok ◀── coverage_guard_node
     ▼                                                     sector_builder_node ── /avoidance/scan ──▶ obstacle_publisher_node ──▶ ArduPilot
                     corridor_perception_node ── /corridor/perception ──▶ path_planner_node
                     banner_node ── /corridor/banner ──▶ path_planner_node (entry corridor)
                     ground_marker_node ── /corridor/ground_marker ("green"|"blue"|"none") ──▶ path_planner_node
 /mission/status, /mission/telemetry
```

All services are `std_srvs/Trigger`; leg services block until the leg finishes, is
stopped or fails. `mission_node` runs the phases in a worker thread and callbacks only
record data. The behaviour rules (RC CH7 start/abort, QR retries, drop, homing,
sensor-health holds) are documented at the top of `mission_node.py` and
`path_planner_node.py`.

## Build

Use `setup.sh` at the repo root (dependencies + build); afterwards
`source ~/AeroTHON26/env.sh`. By hand:

```bash
cd ~/AeroTHON26/ros2_ws && source /opt/ros/jazzy/setup.bash
colcon build --symlink-install && source install/setup.bash
```

Python packages not covered by rosdep are in the repo's `requirements.txt`.

## Run

SITL (ArduPilot SITL + MAVROS + mission, each in its own terminal):

```bash
ros2 run mission_bringup run_sim.sh             # or: --mavros-only, then
ros2 launch mission_bringup sitl_mission.launch.py
```

`sitl_mission.launch.py` is the only place the hardware gates are relaxed (no RC
switch, fallback QR ID, 3S battery thresholds). By default (`corridor_sim:=true`)
`sitl_corridor_sim` ray-casts the OAK-D depth and the ground markers into 5 ft
corridors with obstacles (with a deliberate 0.5 m GPS offset), and the real
avoidance and corridor-perception nodes run on it. It also draws the green banner
(11.5 × 1.5 ft) sitting on top of corridor 1's mouth into an OAK colour stream for the
real `banner_node`; the detector finds it from up to ~15 m in front of the entrance and
~10–12 m to the side, not edge-on or from behind.
`corridor_sim:=false` flies the corridors on the GPS centreline with no depth at all
(the entry corridor needs perception, so the mission stops there).

Hardware (Pi 5 ↔ Pixhawk on TELEM2):

```bash
ros2 launch mission_bringup mission.launch.py launch_mavros:=true \
    fcu_url:=/dev/ttyAMA0:921600 launch_camera:=true
```

The mission waits for the pilot's RC CH7 switch; switching it low at any time returns
control to the pilot (LOITER).

Entry corridor test flight (phase 2 on its own: take off, no reference QR scan →
descend → banner search → corridor → climb to 10 m → hover 5 s → land). In SITL:
`sitl_mission.launch.py mission:=entry_corridor`.

```bash
ros2 launch mission_bringup entry_corridor.launch.py takeoff_alt:=<m>
```

## Configure

| File | Contents |
|---|---|
| `mission_bringup/config/mission_params.yaml` | Mission settings; the box at the top lists what changes on competition day |
| `obstacle_avoidance/config/avoidance.yaml` | Depth → scan pipeline; corridor perception (accepted width range, wall sources, slice, ranges); green banner detector (green_detect.py's values) |
| `obstacle_avoidance/config/oak_d_lite_depth.yaml` | **Required** OAK-D driver settings: depth + 640x360 colour, no NN (the defaults corrupt obstacle bearings) |
| `obstacle_avoidance/params/*.param` | ArduPilot parameter sets per test stage (see its README) |

## Test

```bash
cd ~/AeroTHON26/ros2_ws/src
python3 -m pytest mission_manager/test path_planner/test qr_detection/test obstacle_avoidance/test
python3 path_planner/path_planner/waypoint_gen.py   # preview the lawnmower (PNG in cwd)
ros2 run mission_manager flight_primitives          # arm/takeoff/goto/land self-test on SITL
```
