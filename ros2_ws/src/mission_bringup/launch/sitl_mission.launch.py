# sitl_mission.launch.py — Mission 2 stack against ArduPilot SITL
#
#   ros2 launch mission_bringup sitl_mission.launch.py
#   ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true
#   ros2 launch mission_bringup sitl_mission.launch.py corridor_sim:=false
#
# Same nodes as mission.launch.py, with simulated cameras: sim_down_camera_node
# renders what the downward Cam3 would see — the reference QR 1 m ahead of the
# takeoff point (phase 1) and the arena's QRs and red mat (phase 3) — for the
# REAL qr_scanner_node and red_zone_node.
#
# The arena (30 m across x 40 m deep) starts just past the far ends of both
# corridors, centred between them. Its QRs: the match (the same picture as the
# reference, holding no text) and 5 decoys; one 10 ft red mat.
#
# corridor_sim:=true (default): sitl_corridor_sim ray-casts the OAK-D depth
# and the ground markers from the SITL pose into 5 ft corridors with
# obstacles (sitl_corridor_sim section of mission_params.yaml), and the REAL
# sector_builder, coverage_guard and corridor_perception nodes run on it — the
# corridor legs are flown closed-loop, sensor health gating included. It also
# renders the green banner on corridor 1 and the blue one on corridor 2 into a
# colour stream for the REAL banner_node / banner_blue_node, so the entry
# (phase 2) and exit (phase 5) corridors run closed-loop too.
# corridor_sim:=false: no depth at all; corridors are flown on the GPS
# centreline and the obstacle-sensor gates are off. The entry corridor needs
# perception, so the mission stops there (LAND).
#
# Arena layout, as on the real field: the takeoff point is BETWEEN the two
# corridors, which stand side by side in front of the drone (SITL starts nose
# North). Nobody knows the real distances, so they are arguments:
#   c1_ahead_m  corridor 1 entrance (banner) this far ahead of takeoff (m)
#   c1_left_m   ... and this far to the left (m)
#   c1_yaw_deg  corridor 1 direction vs the nose, deg, + = turned left
#               (0 = running straight ahead, entrance facing the drone)
# Corridor 2 (the exit corridor, blue banner, obstacles) is the mirror image on
# the right, entered from its far (arena) end. Sizes and obstacles stay those
# of mission_params.yaml.
#   ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true \
#     c1_ahead_m:=8 c1_left_m:=3
#
# Disturbances (to check the corridors are flown centred, not on GPS):
#   gps_drift_m     GPS error swinging +-this sideways (gps_drift_period_s), on top of
#                   the fixed gps_offset_m in mission_params.yaml
#   wind_ms, wind_dir_deg, wind_turb_ms
#                   ArduPilot SITL wind (SIM_WIND_SPD / _DIR — direction it
#                   blows FROM — / _TURB), set through MAVROS once connected
# sitl_corridor_sim logs the true centre offset after every corridor pass.
#
# One phase on its own, same world (mission:=...):
#   full            the whole mission (default)
#   takeoff         takeoff_test: gradual takeoff -> hover -> land
#   qr_scan         qr_scan_mission_node: phase 1 -> land
#   entry_corridor  entry_corridor_mission: takeoff -> phase 2 -> hover -> land
#   exit_corridor   exit_corridor_mission (exit_test_after_entry): takeoff ->
#                   phase 2 -> phase 5 (predicted from corridor 1) -> hover -> land

import json
import math
import os

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


# mission:= -> mission_manager executable (each flies the mission's own code
# for its phases, then lands)
MISSIONS = {
    "full": "mission_node",
    "takeoff": "takeoff_test",
    "qr_scan": "qr_scan_mission_node",
    "entry_corridor": "entry_corridor_mission",
    "exit_corridor": "exit_corridor_mission",
}


def corridor_layout(ahead: float, left: float, yaw_deg: float, sim_cfg: dict) -> dict:
    """Corridor waypoints (x East, y North of the takeoff point, nose North)
    for the layout arguments. sitl_corridor_sim: the first waypoint is
    mouth_offset_m before a corridor's entrance and the walls run toward the
    second. Corridor 1's entrance is at (-left, ahead), running away from the
    drone; corridor 2 mirrors it on the right and is entered from its far
    end, its walls ending exit_offset_m before its second waypoint, at the
    near end."""
    mouth, exit_ = sim_cfg["mouth_offset_m"], sim_cfg["exit_offset_m"]
    th = math.radians(yaw_deg)
    d1 = (-math.sin(th), math.cos(th))           # corridor 1 direction (E, N)
    d2 = (math.sin(th), math.cos(th))            # mirror image
    # corridor 2 keeps its length (its obstacles are placed along it):
    # corridor2_length_m if set, else from its yaml waypoints
    x2, y2 = sim_cfg["corridor2_waypoints_x"], sim_cfg["corridor2_waypoints_y"]
    len2 = sim_cfg.get("corridor2_length_m") or \
        math.hypot(x2[1] - x2[0], y2[1] - y2[0]) - mouth - exit_
    m1 = (-left, ahead)
    near2 = (left, ahead)
    far2 = (near2[0] + len2 * d2[0], near2[1] + len2 * d2[1])
    wp1 = [(m1[0] - mouth * d1[0], m1[1] - mouth * d1[1]),
           (m1[0] + 20.0 * d1[0], m1[1] + 20.0 * d1[1])]     # length: corridor1_length_m
    wp2 = [(far2[0] + mouth * d2[0], far2[1] + mouth * d2[1]),
           (near2[0] - exit_ * d2[0], near2[1] - exit_ * d2[1])]
    r = lambda v: round(v, 3)                                # noqa: E731
    return {
        "corridor1_waypoints_x": [r(p[0]) for p in wp1],
        "corridor1_waypoints_y": [r(p[1]) for p in wp1],
        "corridor2_waypoints_x": [r(p[0]) for p in wp2],
        "corridor2_waypoints_y": [r(p[1]) for p in wp2],
    }


def arena_world(layout: dict, sim_cfg: dict) -> dict:
    """The SITL arena from the corridor layout (x East, y North of the
    takeoff point): its corners, QRs (sim_down_camera_node format) and red
    mat. Starts 1 m past the farthest corridor end."""
    mouth = sim_cfg["mouth_offset_m"]
    ends = []
    for i, length in ((1, sim_cfg["corridor1_length_m"]), (2, None)):
        xs, ys = layout[f"corridor{i}_waypoints_x"], layout[f"corridor{i}_waypoints_y"]
        d = (xs[1] - xs[0], ys[1] - ys[0])
        n = math.hypot(*d)
        d = (d[0] / n, d[1] / n)
        mouth_pt = (xs[0] + mouth * d[0], ys[0] + mouth * d[1])
        ends.append(mouth_pt)                                   # corridor 2: far end
        if length:
            ends.append((mouth_pt[0] + length * d[0], mouth_pt[1] + length * d[1]))
    y0 = max(y for _, y in ends) + 1.0
    x0 = -15.0
    corners = [(x0, y0), (x0 + 30.0, y0), (x0 + 30.0, y0 + 40.0), (x0, y0 + 40.0)]
    qrs = [[0.0, 1.0, ""],                                      # phase 1 reference
           [x0 + 6.0, y0 + 31.0, ""],                           # the match
           [x0 + 23.0, y0 + 6.0, "AT-17"], [x0 + 27.0, y0 + 27.0, "XX-99"],
           [x0 + 12.0, y0 + 16.0, "B-7"], [x0 + 3.0, y0 + 8.0, "Q9"],
           [x0 + 18.0, y0 + 36.0, "AT-43"]]
    cx, cy, h = x0 + 20.0, y0 + 18.0, 3.048 / 2                  # 10 ft mat
    mat = [[cx - h, cy - h], [cx + h, cy - h], [cx + h, cy + h], [cx - h, cy + h]]
    lat0, lon0 = sim_cfg["origin_lat"], sim_cfg["origin_lon"]
    k = 111_320.0 * math.cos(math.radians(lat0))
    return {
        "corners_lat": [lat0 + y / 111_320.0 for _, y in corners],
        "corners_lon": [lon0 + x / k for x, _ in corners],
        "qr_codes": json.dumps(qrs), "red_zones": json.dumps([mat]),
    }


def _stack(context, *args, **kwargs):
    mission_params = os.path.join(
        get_package_share_directory("mission_bringup"), "config", "mission_params.yaml")
    avoidance_cfg = os.path.join(
        get_package_share_directory("obstacle_avoidance"), "config", "avoidance.yaml")
    sim = LaunchConfiguration("corridor_sim").perform(context) == "true"
    with open(mission_params) as f:
        sim_cfg = yaml.safe_load(f)["sitl_corridor_sim"]["ros__parameters"]
    layout = corridor_layout(
        float(LaunchConfiguration("c1_ahead_m").perform(context)),
        float(LaunchConfiguration("c1_left_m").perform(context)),
        float(LaunchConfiguration("c1_yaw_deg").perform(context)), sim_cfg)

    world = arena_world(layout, sim_cfg)

    # SITL has no RC pilot, so that gate is relaxed here (and ONLY here). The
    # obstacle-sensor gates stay on when the simulated depth pipeline runs.
    sensor = {"require_avoidance_sensor": sim}
    mission_overrides = {
        "require_rc_switch": False,         # no pilot switch in SITL
        "battery_critical_v": 10.5,         # SITL simulates a 3S pack (12.6 V)
        "arena_corners_lat": world["corners_lat"],
        "arena_corners_lon": world["corners_lon"],
        **sensor,
    }

    def node(package, executable, overrides=None, params=None):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=(params or [mission_params]) + ([overrides] if overrides else []),
        )

    nodes = [
        node("payload", "payload_node"),
        # same corridors as the simulated world (they must match)
        node("path_planner", "path_planner_node",
             {**sensor, "require_corridor_perception": sim, **layout}),
        # Downward Cam3 stand-in (origin = first GPS fix = the takeoff point)
        Node(package="qr_detection", executable="sim_down_camera_node",
             name="sim_down_camera", output="screen",
             parameters=[{"marker_size_m": 1.0, "qr_codes": world["qr_codes"],
                          "red_zones": world["red_zones"]}]),
        node("qr_detection", "qr_scanner_node"),
        Node(package="qr_detection", executable="red_zone_node", name="red_zone_node",
             output="screen", parameters=[{"source": "topic", "grow_known_zones": True,
                                                 "confirm_in_flight": True}]),
    ]
    if sim:
        remaps = [("~/depth", "/oak/stereo/image_raw"),
                  ("~/camera_info", "/oak/stereo/camera_info")]
        drift = float(LaunchConfiguration("gps_drift_m").perform(context))
        period = float(LaunchConfiguration("gps_drift_period_s").perform(context))
        nodes.append(node("obstacle_avoidance", "sitl_corridor_sim",
                          {**layout, "gps_drift_m": drift, "gps_drift_period_s": period}))
        # The sim renders at half resolution (render_scale 0.5): stride 2 gives
        # the same 160x120 samples the OAK-D gives at stride 4.
        # 5 Hz sim depth (hardware: 10 Hz): widen the stale cutoffs, keeping
        # the guard 0.2 s ahead of the builder watchdog as it asserts.
        timeouts = {"depth_stale_timeout_s": 0.6, "depth_watchdog_timeout_s": 0.8}
        for exe, extra in (("sector_builder_node", timeouts),
                           ("corridor_perception_node", {"corridor_stride_px": 2})):
            nodes.append(Node(package="obstacle_avoidance", executable=exe, name=exe,
                              output="screen", parameters=[avoidance_cfg, extra],
                              remappings=remaps))
        nodes.append(node("obstacle_avoidance", "coverage_guard_node", timeouts,
                          params=[avoidance_cfg]))
        # The sim's colour stream is for banner_node only (no wall detail):
        # corridor_perception's ~/rgb stays unmapped, so its walls come from depth.
        blue_cfg = os.path.join(get_package_share_directory("obstacle_avoidance"),
                                "config", "banner_blue.yaml")
        for name, cfgs in (("banner_node", [avoidance_cfg]),
                           ("banner_blue_node", [avoidance_cfg, blue_cfg])):
            nodes.append(Node(package="obstacle_avoidance", executable="banner_node",
                              name=name, output="screen",
                              parameters=cfgs + [{"corridor_stride_px": 2}],
                              remappings=remaps + [
                                  ("~/rgb", "/oak/rgb/image_raw"),
                                  ("~/rgb_camera_info", "/oak/rgb/camera_info")]))

    # SITL wind: simulator parameters only (SIM_*), set once MAVROS is up.
    wind = {"SIM_WIND_SPD": LaunchConfiguration("wind_ms").perform(context),
            "SIM_WIND_DIR": LaunchConfiguration("wind_dir_deg").perform(context),
            "SIM_WIND_TURB": LaunchConfiguration("wind_turb_ms").perform(context)}
    if float(wind["SIM_WIND_SPD"]) > 0.0 or float(wind["SIM_WIND_TURB"]) > 0.0:
        # MAVROS refuses PARAM_SET until it has the FC's parameter list: retry
        nodes.append(TimerAction(period=6.0, actions=[ExecuteProcess(
            cmd=["bash", "-c",
                 "for i in $(seq 60); do ros2 service call /mavros/param/set "
                 "mavros_msgs/srv/ParamSetV2 "
                 f"'{{param_id: {name}, value: {{type: 3, double_value: {float(v)}}}}}' "
                 f"| grep -q 'success=True' && echo 'SITL {name} = {float(v)}' && exit 0; "
                 "sleep 2; done; echo 'SITL " + name + " NOT set'"],
            output="screen") for name, v in wind.items()]))

    # Mission node last, after SITL + MAVROS have settled.
    exe = MISSIONS[LaunchConfiguration("mission").perform(context)]
    if exe == "exit_corridor_mission":
        # SITL starts between the corridors, not facing the exit corridor:
        # fly the entry corridor first and predict the exit from it.
        mission_overrides["exit_test_after_entry"] = True
    nodes.append(TimerAction(period=8.0, actions=[Node(
        package="mission_manager", executable=exe,
        name="mission_node",                # reads the mission_node: yaml section
        output="screen", parameters=[mission_params, mission_overrides])]))
    return nodes


def generate_launch_description():
    # MAVROS (Jazzy ships apm.launch as XML -> AnyLaunchDescriptionSource)
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
        ]),
        launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
        condition=IfCondition(LaunchConfiguration("launch_mavros")),
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "launch_mavros", default_value="false",
            description="Launch MAVROS (set false if started separately)"),
        DeclareLaunchArgument(
            "fcu_url", default_value="tcp://127.0.0.1:5760",
            description="MAVROS FCU URL (SITL tcp, or serial device:baud for hardware)"),
        DeclareLaunchArgument(
            "corridor_sim", default_value="true", choices=["true", "false"],
            description="Simulated OAK-D depth + ground markers in the corridors"),
        DeclareLaunchArgument(
            "mission", default_value="full", choices=list(MISSIONS),
            description="Whole mission, or one phase on its own then land"),
        DeclareLaunchArgument("gps_drift_m", default_value="0.0",
                              description="GPS error swinging +-this sideways (m)"),
        DeclareLaunchArgument("gps_drift_period_s", default_value="20.0",
                              description="period of that swing (s)"),
        DeclareLaunchArgument("wind_ms", default_value="0.0", description="SITL wind (m/s)"),
        DeclareLaunchArgument("wind_dir_deg", default_value="90.0",
                              description="SITL wind direction it blows FROM (deg)"),
        DeclareLaunchArgument("wind_turb_ms", default_value="0.0",
                              description="SITL wind turbulence (m/s)"),
        DeclareLaunchArgument(
            "c1_ahead_m", default_value="10.0",
            description="Corridor 1 entrance this far ahead (North) of takeoff (m)"),
        DeclareLaunchArgument(
            "c1_left_m", default_value="5.0",
            description="Corridor 1 entrance this far left (West) of takeoff (m)"),
        DeclareLaunchArgument(
            "c1_yaw_deg", default_value="0.0",
            description="Corridor 1 direction vs the nose, deg, + = left"),
        mavros_launch,
        # Full-resolution sim frames (2.7 MB) must go through shared memory or
        # most are lost as UDP fragments (config/fastdds_large_frames.xml).
        SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            PathJoinSubstitution([FindPackageShare("mission_bringup"), "config",
                                  "fastdds_large_frames.xml"])),
        OpaqueFunction(function=_stack),
    ])
