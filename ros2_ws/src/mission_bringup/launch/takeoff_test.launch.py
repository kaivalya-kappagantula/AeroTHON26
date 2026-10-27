# takeoff_test.launch.py — takeoff-only flight test in one command:
#
#   ros2 launch mission_bringup takeoff_test.launch.py
#
# Starts: MAVROS and takeoff_test (the mission's gradual takeoff -> hover
# -> land). mission_params.yaml is read as-is; the arguments below override
# it for this run only.
#
#   alt        takeoff altitude (m)
#   hover      hover time at alt before landing (s)
#   rc_switch  true = wait for the RC start switch; false = start on launch
#              (then the flight-mode switch is the pilot's takeover)
#   fcu_url    MAVROS link
#   sim        true = SITL: fcu_url defaults to SITL's
#
# SITL:
#   ros2 launch mission_bringup takeoff_test.launch.py sim:=true

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _yaml_battery_v(mission_params: str) -> str:
    """battery_critical_v as mission_params.yaml sets it (hardware value)."""
    with open(mission_params) as f:
        return str(yaml.safe_load(f)["mission_node"]["ros__parameters"]["battery_critical_v"])


def generate_launch_description():
    mission_params = os.path.join(
        get_package_share_directory("mission_bringup"), "config", "mission_params.yaml")

    sim = LaunchConfiguration("sim")
    args = [
        DeclareLaunchArgument("sim", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument(
            "fcu_url", default_value=PythonExpression(
                ["'tcp://127.0.0.1:5760' if '", sim, "' == 'true' else '/dev/ttyAMA2:921600'"])),
        DeclareLaunchArgument("alt", default_value="5.0"),
        DeclareLaunchArgument("hover", default_value="10.0"),
        DeclareLaunchArgument("rc_switch", default_value="false", choices=["true", "false"]),
    ]

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
        ]),
        launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
    )

    mission = Node(
        package="mission_manager",
        executable="takeoff_test",
        name="mission_node",                # reads the mission_node: yaml section
        output="screen",
        parameters=[mission_params, {
            "cruise_altitude_m": ParameterValue(LaunchConfiguration("alt"), value_type=float),
            "takeoff_test_hover_s": ParameterValue(LaunchConfiguration("hover"),
                                                   value_type=float),
            "require_rc_switch": ParameterValue(LaunchConfiguration("rc_switch"), value_type=bool),
            "require_avoidance_sensor": False,    # no OAK-D pipeline in this test
            # SITL simulates a 3S pack (12.6 V); the 4S limit applies on hardware
            "battery_critical_v": ParameterValue(PythonExpression(
                ["10.5 if '", sim, "' == 'true' else ", _yaml_battery_v(mission_params)]),
                value_type=float),
        }],
    )

    return LaunchDescription(args + [
        mavros,
        TimerAction(period=5.0, actions=[mission]),
    ])
