# qr_scan_mission.launch.py — PHASE 1 flight test in one command:
#
#   ros2 launch mission_bringup qr_scan_mission.launch.py
#
# Starts: MAVROS, the RPi Cam3 (camera_ros -> /down_frame), qr_scanner_node
# and qr_scan_mission_node (gradual takeoff -> alt -> find + centre over the
# QR -> read + store it -> land). The reference is stored in
# ~/aerothon_captures/reference_qr.json + .png (text, if any, and picture).
# mission_params.yaml is read as-is; the arguments below override it for
# this run only.
#
#   alt           takeoff / scan altitude (m)
#   max_radius    never fly further than this from the takeoff point (m)
#   scan_timeout  seconds per scan attempt (scanner + mission use the same)
#   attempts      scan attempts (a re-centre between each)
#   rc_switch     true = wait for the RC start switch; false = start on launch
#                 (then the flight-mode switch is the pilot's takeover)
#   cam_yaw       Cam3 image-top direction vs the nose, deg clockwise (0 = aligned)
#   cam_mirror    true (default, as the Cam3 is mounted) = its image is mirrored
#                 left-right vs the real scene; the nodes flip it back
#   af_mode       Cam3 focus mode: 0 = manual (default). Autofocus cannot work on
#                 this camera stack (libcamera: "PDAF data in unsupported
#                 format"); the mission sets a fixed focus for alt (1/alt dioptres)
#   live_view     true (default) = stream the Cam3 to http://<pi>:live_view_port/
#                 (480 px, <= 5 fps to keep it light); false in flight frees CPU
#   fcu_url       MAVROS link
#   sim           true = SITL: no camera_ros, sim_down_camera_node renders
#                 sim_qr_codes; fcu_url defaults to SITL's
#   sim_qr_codes  JSON [[east, north, text], ...] metres from the takeoff
#                 point. Default: a QR holding NO text 1 m North (SITL's
#                 default nose direction) — the hard case.
#   heading_error SITL: deg the FC's heading reads clockwise of the true nose,
#                 when the SITL compass is made wrong on purpose; the sim
#                 camera renders the true view (default 0)
#
# Needs both workspaces sourced (camera_ws for camera_ros, ros2_ws for the rest).
# Place the vehicle with the QR at most ~1 m ahead, nose pointing at it.
#
# SITL:
#   ros2 launch mission_bringup qr_scan_mission.launch.py sim:=true
#   ros2 launch mission_bringup qr_scan_mission.launch.py sim:=true \
#     sim_qr_codes:='[[0,0.8,"AT-42"]]'

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


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
        DeclareLaunchArgument("max_radius", default_value="4.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("cam_mirror", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("scan_timeout", default_value="20.0"),
        DeclareLaunchArgument("attempts", default_value="2"),
        DeclareLaunchArgument("rc_switch", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("af_mode", default_value="0"),
        DeclareLaunchArgument("live_view", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("live_view_port", default_value="8080"),
        DeclareLaunchArgument("sim_qr_codes", default_value='[[0,1,""]]'),
        DeclareLaunchArgument("heading_error", default_value="0.0"),
    ]

    mavros = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
        ]),
        launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
    )

    camera = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="screen",
        condition=UnlessCondition(sim),
        parameters=[{
            "width": 1280,
            "height": 720,
            "format": "BGR888",
            # Camera Module 1.3: its full-width binned mode (else libcamera may
            # pick the cropped 1080p one, ~41 deg). Remove for a Camera Module 3.
            "sensor_mode": "1296:972",
            "AfMode": ParameterValue(LaunchConfiguration("af_mode"), value_type=int),
            # fixed focus for alt from the start (the mission sets it again)
            "LensPosition": ParameterValue(PythonExpression(
                ["1.0 / max(float('", LaunchConfiguration("alt"), "'), 0.1)"]), value_type=float),
        }],
        remappings=[("/camera/image_raw", "/down_frame")],
    )

    sim_camera = Node(
        package="qr_detection",
        executable="sim_down_camera_node",
        name="sim_down_camera",
        output="screen",
        condition=IfCondition(sim),
        parameters=[{
            "mount_yaw_deg": _float("cam_yaw"),
            "mirror": ParameterValue(LaunchConfiguration("cam_mirror"), value_type=bool),
            "heading_error_deg": _float("heading_error"),
            "qr_codes": ParameterValue(LaunchConfiguration("sim_qr_codes"), value_type=str),
        }],
    )

    scanner = Node(
        package="qr_detection",
        executable="qr_scanner_node",
        name="qr_scanner_node",
        output="screen",
        parameters=[{
            "scan_timeout_s": _float("scan_timeout"),
            "cam_yaw_offset_deg": _float("cam_yaw"),
            "cam_mirror": ParameterValue(LaunchConfiguration("cam_mirror"), value_type=bool),
            "delivery_altitude": _float("alt"),
        }],
    )

    mission = Node(
        package="mission_manager",
        executable="qr_scan_mission_node",
        name="mission_node",                # reads the mission_node: yaml section
        output="screen",
        parameters=[mission_params, {
            "cruise_altitude_m": _float("alt"),
            "require_rc_switch": ParameterValue(LaunchConfiguration("rc_switch"), value_type=bool),
            "require_avoidance_sensor": False,    # no OAK-D pipeline in this test
            # SITL simulates a 3S pack (12.6 V); the 4S limit applies on hardware
            "battery_critical_v": ParameterValue(PythonExpression(
                ["10.5 if '", sim, "' == 'true' else ", _yaml_battery_v(mission_params)]),
                value_type=float),
            "qr_scan_timeout_s": _float("scan_timeout"),
            "qr_scan_attempts": ParameterValue(LaunchConfiguration("attempts"), value_type=int),
            "qr_max_radius_m": _float("max_radius"),
        }],
    )

    # Full-resolution sim frames (2.7 MB) must go through shared memory or
    # most are lost as UDP fragments (config/fastdds_large_frames.xml).
    large_frames = SetEnvironmentVariable(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        PathJoinSubstitution([FindPackageShare("mission_bringup"), "config",
                              "fastdds_large_frames.xml"]),
        condition=IfCondition(sim))

    live_view = Node(
        package="qr_detection",
        executable="live_view_node",
        name="live_view_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("live_view")),
        parameters=[{"port": ParameterValue(LaunchConfiguration("live_view_port"), value_type=int),
                     "width": 480, "max_fps": 5.0}],
    )

    return LaunchDescription(args + [
        large_frames,
        mavros,
        # The sim camera takes its origin from the first GPS fix: vehicle on
        # the takeoff point, so start it once MAVROS is up.
        TimerAction(period=3.0, actions=[camera, sim_camera, scanner, live_view]),
        # Mission last, once MAVROS, the camera and the scanner are up.
        TimerAction(period=6.0, actions=[mission]),
    ])
