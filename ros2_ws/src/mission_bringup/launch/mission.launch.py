# mission.launch.py — Master launch file for Aerothon UAS Mission 2 (hardware)
#
#   ros2 launch mission_bringup mission.launch.py launch_mavros:=true \
#       fcu_url:=/dev/ttyAMA2:921600 launch_camera:=true
#
# STAGED START (the Pi 5 browned out with everything powering up at once).
# This file starts only the light nodes, a few seconds apart: MAVROS, then
# path planner, payload, QR scanner (idle until called) and the rosbag, then
# the mission node. mission_node starts the rest when each phase needs it
# (mission_stage.launch.py, mission_manager/stages.py), one at a time:
#   preflight   OAK-D Lite + obstacle pipeline, then the downward camera
#   phase 2     corridor_perception + green banner + ground marker (stopped after)
#   phase 3     red_zone_node (until landing)
#   phase 5     corridor_perception + blue banner + ground marker
#
# Pi 5 <-> Pixhawk 6C Mini on TELEM2 via the Pi's /dev/ttyAMA2 (the default
# fcu_url of every drone launch; that UART must be enabled in
# /boot/firmware/config.txt)
#   FC: SERIAL2_PROTOCOL=2, SERIAL2_BAUD=921
# USB instead: fcu_url:=/dev/ttyACM0:115200
#
# Live view + logs (one folder per flight, ~/aerothon_logs/mission_<time>/):
#   live_view:=true (default)  Cam3 at http://<pi>:8080/, OAK colour at
#                              http://<pi>:8081/ (640 px, <= 10 fps, latest
#                              frame only: a slow viewer never lags or holds up
#                              the mission); both streams also saved there as
#                              .mjpeg (ffplay -f mjpeg <file>)
#   record:=true (default)     rosbag of telemetry, MAVROS state/position,
#                              perception results and /rosout (no images)

import os
import time
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    mission_params = os.path.join(
        get_package_share_directory("mission_bringup"), "config", "mission_params.yaml")

    # ----- Launch arguments -----
    launch_mavros_arg = DeclareLaunchArgument(
        "launch_mavros", default_value="false",
        description="Launch MAVROS (set false if started separately)")
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url", default_value="/dev/ttyAMA2:921600",
        description="MAVROS FCU URL (Pi TELEM2 by default; SITL: tcp://127.0.0.1:5760)")
    launch_camera_arg = DeclareLaunchArgument(
        "launch_camera", default_value="false",
        description="Start camera_ros for the downward RPi Camera Module 1.3 "
                    "(needs camera_ros built against Raspberry Pi's libcamera)")
    camera_arg = DeclareLaunchArgument(
        "camera", default_value="0",
        description="libcamera camera index or name for the downward camera")
    awb_mode_arg = DeclareLaunchArgument(
        "awb_mode", default_value="5",
        description="Cam3 white balance (5 = daylight: keeps the red mat's colour stable, "
                    "as the arena test flights; 0 = auto)")

    live_view_arg = DeclareLaunchArgument(
        "live_view", default_value="true", choices=["true", "false"],
        description="Stream Cam3 (:8080) and OAK colour (:8081) to a browser, and save them")
    record_arg = DeclareLaunchArgument(
        "record", default_value="true", choices=["true", "false"],
        description="rosbag of the mission's non-image topics")
    log_dir = os.path.expanduser(
        os.path.join("~", "aerothon_logs", time.strftime("mission_%Y%m%d_%H%M%S")))

    # ================================================================
    # MAVROS (Jazzy ships apm.launch as XML -> AnyLaunchDescriptionSource)
    # ================================================================
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])
        ]),
        launch_arguments={"fcu_url": LaunchConfiguration("fcu_url")}.items(),
        condition=IfCondition(LaunchConfiguration("launch_mavros")),
    )

    def mission_node_of(package, executable, extra_params=()):
        return Node(
            package=package,
            executable=executable,
            name=executable,
            output="screen",
            parameters=[mission_params, *extra_params],
        )

    path_planner_node = mission_node_of("path_planner", "path_planner_node")
    payload_node = mission_node_of("payload", "payload_node")
    qr_scanner_node = mission_node_of("qr_detection", "qr_scanner_node")
    # Starts the OAK, downward camera and per-phase nodes itself (stages.py)
    mission_node = mission_node_of("mission_manager", "mission_node", [{
        "staged_launch": True,
        # one string: "launch_camera:=true camera:=0 ..."
        "stage_launch_args": ParameterValue([
            "launch_camera:=", LaunchConfiguration("launch_camera"),
            " camera:=", LaunchConfiguration("camera"),
            " awb_mode:=", LaunchConfiguration("awb_mode"),
            " live_view:=", LaunchConfiguration("live_view"),
            f" log_dir:={log_dir}"], value_type=str),
    }])

    # ================================================================
    # Flight logs (the live views start with their camera's stage)
    # ================================================================
    record = ExecuteProcess(
        cmd=["ros2", "bag", "record", "-o", os.path.join(log_dir, "bag"),
             "/rosout", "/mission/telemetry", "/mission/status",
             "/mavros/state", "/mavros/battery", "/mavros/global_position/global",
             "/mavros/global_position/rel_alt", "/mavros/local_position/pose",
             "/corridor/perception", "/corridor/banner", "/corridor/banner_blue",
             "/corridor/ground_marker", "/qr_scanner/status", "/qr_scanner/delivery_id",
             "/red_zone/sighting", "/red_zone/zones", "/payload/release"],
        output="log",
        condition=IfCondition(LaunchConfiguration("record")),
    )

    return LaunchDescription([
        launch_mavros_arg,
        fcu_url_arg,
        launch_camera_arg,
        camera_arg,
        awb_mode_arg,
        live_view_arg,
        record_arg,

        # A few seconds apart so the Pi never starts everything at once
        mavros_launch,
        TimerAction(period=3.0, actions=[payload_node, path_planner_node]),
        TimerAction(period=5.0, actions=[qr_scanner_node, record]),
        # Mission node last; it waits for MAVROS, then starts the stages.
        TimerAction(period=8.0, actions=[mission_node]),
    ])
