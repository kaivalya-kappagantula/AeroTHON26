# sitl_mission.launch.py — Mission 2 stack against ArduPilot SITL
#
#   ros2 launch mission_bringup sitl_mission.launch.py
#   ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true
#   ros2 launch mission_bringup sitl_mission.launch.py corridor_sim:=false
#
# Same nodes as mission.launch.py minus the cameras; sitl_qr_simulator stands
# in for the downward camera's QR detections.
#
# corridor_sim:=true (default): sitl_corridor_sim ray-casts the OAK-D depth
# and the ground markers from the SITL pose into 5 ft corridors with
# obstacles (sitl_corridor_sim section of mission_params.yaml), and the REAL
# sector_builder, coverage_guard and corridor_perception nodes run on it — the
# corridor legs are flown closed-loop, sensor health gating included. It also
# renders the green banner on corridor 1 into a colour stream for the REAL
# banner_node, so the mission's entry corridor (phase 1) runs closed-loop too.
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
# Corridor 2 is the mirror image on the right, entered from its far (arena)
# end. Sizes and obstacles stay those of mission_params.yaml.
#   ros2 launch mission_bringup sitl_mission.launch.py launch_mavros:=true \
#     c1_ahead_m:=8 c1_left_m:=3
#
# One phase on its own, same world (mission:=...):
#   full            the whole mission (default)
#   takeoff         takeoff_test: gradual takeoff -> hover -> land
#   qr_scan         qr_scan_mission_node: phase 1 -> land (no downward camera
#                   here, so the QR is not found: use qr_scan_mission.launch.py
#                   sim:=true for the QR itself)
#   entry_corridor  entry_corridor_mission: takeoff -> phase 2 -> hover -> land

import math
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
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
    # corridor 2 keeps its length (its obstacles are placed along it)
    x2, y2 = sim_cfg["corridor2_waypoints_x"], sim_cfg["corridor2_waypoints_y"]
    len2 = math.hypot(x2[1] - x2[0], y2[1] - y2[0]) - mouth - exit_
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

    # SITL has no RC pilot and no downward camera, so those gates are relaxed
    # here (and ONLY here). The obstacle-sensor gates stay on when the
    # simulated depth pipeline runs.
    sensor = {"require_avoidance_sensor": sim}
    mission_overrides = {
        "require_rc_switch": False,         # no pilot switch in SITL
        "allow_qr_fallback": True,          # no camera: fly with qr_fallback_id
        "qr_scan_attempts": 1,
        "qr_scan_timeout_s": 5.0,
        "battery_critical_v": 10.5,         # SITL simulates a 3S pack (12.6 V)
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
        node("mission_manager", "sitl_qr_simulator"),
        node("payload", "payload_node"),
        # same corridors as the simulated world (they must match)
        node("path_planner", "path_planner_node",
             {**sensor, "require_corridor_perception": sim, **layout}),
        # Provides /perception/scan_qr; with no camera the reference scan times
        # out and mission_node uses qr_fallback_id.
        node("qr_detection", "qr_scanner_node", {"scan_timeout_s": 5.0}),
    ]
    if sim:
        remaps = [("~/depth", "/oak/stereo/image_raw"),
                  ("~/camera_info", "/oak/stereo/camera_info")]
        nodes.append(node("obstacle_avoidance", "sitl_corridor_sim", layout))
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
        nodes.append(Node(package="obstacle_avoidance", executable="banner_node",
                          name="banner_node", output="screen",
                          parameters=[avoidance_cfg, {"corridor_stride_px": 2}],
                          remappings=remaps + [
                              ("~/rgb", "/oak/rgb/image_raw"),
                              ("~/rgb_camera_info", "/oak/rgb/camera_info")]))

    # Mission node last, after SITL + MAVROS have settled.
    exe = MISSIONS[LaunchConfiguration("mission").perform(context)]
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
        OpaqueFunction(function=_stack),
    ])
