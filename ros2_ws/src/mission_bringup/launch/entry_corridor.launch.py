# entry_corridor.launch.py — test flight for the entry corridor (end of
# mission phase 1):
#
#   ros2 launch mission_bringup entry_corridor.launch.py takeoff_alt:=<m>
#
# PHASE 2 on its own: take off (no reference QR scan) -> the mission's phase 2
# (descend to corridor_start_alt_m, find the green banner sweeping +-90 deg
# around the placed heading, line up on the corridor under it, fly through at
# the middle of the opening until the walls end, climb to post_entry_alt_m)
# -> hover entry_test_hover_s -> land. Needs clear sky above the corridor
# exit for the climb. SITL: sitl_mission.launch.py mission:=entry_corridor.
#
# Starts: MAVROS, the OAK-D Lite driver (oak_d_lite_depth.yaml: depth +
# colour), the avoidance pipeline (sector_builder -> ArduPilot proximity,
# coverage_guard, corridor_perception with depth + RGB wall bases),
# banner_node, path_planner_node and entry_corridor_mission.
# mission_params.yaml and avoidance.yaml are read as-is; the arguments below
# override them for this run only (nothing is written back).
#
#   takeoff_alt  REQUIRED: take-off height (m); the drone then descends to
#                corridor_start_alt_m for the banner search (which also tries
#                1.0-3.0 m)
#   rc_switch    true = wait for RC CH7 HIGH to start (CH7 LOW = LOITER)
#   drone_width  tip-to-tip incl. props (m) — decides the centring margins
#
# Nothing about the corridor is configured: its width, length and the
# altitude to fly at are measured in flight. Place the vehicle FACING the
# corridors: the banner search only sweeps +-90 deg around that heading.
# All output is also in ~/.ros/log/<date-time>/launch.log.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float(name):
    # float even when typed as "2" (the params are declared double)
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def generate_launch_description():
    mission_params = os.path.join(
        get_package_share_directory("mission_bringup"), "config", "mission_params.yaml")
    avoidance_share = get_package_share_directory("obstacle_avoidance")
    avoidance_params = os.path.join(avoidance_share, "config", "avoidance.yaml")

    args = [
        DeclareLaunchArgument("fcu_url", default_value="/dev/ttyAMA0:921600"),
        DeclareLaunchArgument("takeoff_alt",
                              description="take-off height (m) before the banner search"),
        DeclareLaunchArgument("rc_switch", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("drone_width", default_value="0.61"),
    ]

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
        ]),
        launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
    )

    oakd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("depthai_ros_driver"), "launch", "camera.launch.py"
            ])
        ]),
        launch_arguments={
            "name": "oak",                  # must match the /oak: key in the yaml
            "camera_model": "OAK-D-LITE",
            "params_file": os.path.join(avoidance_share, "config", "oak_d_lite_depth.yaml"),
        }.items(),
    )

    avoidance = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(avoidance_share, "launch", "sector_builder_hw.launch.py")),
        launch_arguments={"obstacle_path": "sink"}.items(),
    )

    banner = Node(
        package="obstacle_avoidance",
        executable="banner_node",
        name="banner_node",
        output="screen",
        parameters=[avoidance_params],
        remappings=[
            ("~/rgb", "/oak/rgb/image_raw"),
            ("~/rgb_camera_info", "/oak/rgb/camera_info"),
            ("~/depth", "/oak/stereo/image_raw"),
            ("~/camera_info", "/oak/stereo/camera_info"),
        ],
    )

    planner = Node(
        package="path_planner",
        executable="path_planner_node",
        name="path_planner_node",
        output="screen",
        parameters=[mission_params, {
            "drone_width_m": _float("drone_width"),
        }],
    )

    mission = Node(
        package="mission_manager",
        executable="entry_corridor_mission",
        name="mission_node",                # reads the mission_node: yaml section
        output="screen",
        parameters=[mission_params, {
            "cruise_altitude_m": _float("takeoff_alt"),
            "require_rc_switch": ParameterValue(LaunchConfiguration("rc_switch"), value_type=bool),
        }],
    )

    return LaunchDescription(args + [
        mavros, oakd, avoidance, banner, planner,
        # Mission last, once MAVROS and the planner are up.
        TimerAction(period=5.0, actions=[mission]),
    ])
