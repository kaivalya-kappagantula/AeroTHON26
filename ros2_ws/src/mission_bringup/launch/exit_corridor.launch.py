# exit_corridor.launch.py — test flight for the exit corridor (mission
# phase 5):
#
#   ros2 launch mission_bringup exit_corridor.launch.py takeoff_alt:=<m>
#
# PHASE 5's corridor on its own: take off -> descend to corridor_start_alt_m
# over the takeoff point -> find the BLUE banner sweeping +-90 deg around the
# placed heading, line up on the corridor under it, fly through at the middle
# of the opening, passing obstacles beside them (never over), until the walls
# end -> hover exit_test_hover_s -> land (exit_corridor_mission). The
# mission's prediction of where the exit corridor is (corridor 1 mirrored)
# is NOT used here — it needs the entry corridor flown first; SITL:
# sitl_mission.launch.py mission:=exit_corridor flies both.
#
# Starts: MAVROS, the OAK-D Lite driver (oak_d_lite_depth.yaml: depth +
# colour), the avoidance pipeline (sector_builder -> ArduPilot proximity,
# coverage_guard, corridor_perception with depth + RGB wall bases),
# banner_blue_node, path_planner_node and exit_corridor_mission.
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
# exit corridor's entrance (the blue banner), a few metres out: the banner
# search only sweeps +-90 deg around that heading. The blue hue band is
# untuned (banner_blue.yaml): check banner_blue_node's log on the ground first.
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
    blue_params = os.path.join(avoidance_share, "config", "banner_blue.yaml")

    args = [
        DeclareLaunchArgument("fcu_url", default_value="/dev/ttyAMA2:921600"),
        DeclareLaunchArgument("takeoff_alt",
                              description="take-off height (m) before the banner search"),
        DeclareLaunchArgument("rc_switch", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("drone_width", default_value="0.6294"),
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
        name="banner_blue_node",
        output="screen",
        parameters=[avoidance_params, blue_params],
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
        executable="exit_corridor_mission",
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
