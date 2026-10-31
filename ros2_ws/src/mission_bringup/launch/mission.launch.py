# mission.launch.py — the whole mission, one command (or just ./fly.sh)
#
#   ros2 launch mission_bringup mission.launch.py             # on the drone
#   ros2 launch mission_bringup mission.launch.py sim:=true   # SITL (fly.sh sim
#                                                             # also starts ArduPilot)
#
# Defaults are the drone's: MAVROS on the Pi's /dev/ttyAMA2 (Pixhawk 6C
# TELEM2: SERIAL2_PROTOCOL=2, SERIAL2_BAUD=921; USB instead:
# fcu_url:=/dev/ttyACM0:115200), downward RPi Camera Module 1.3 on camera 0.
#
# Only the light nodes start here, a few seconds apart: MAVROS, payload_node
# (holds the payload from the start), qr_scanner_node (idle until called),
# then mission_node. mission_node starts every other node PHASE BY PHASE
# and stops it when its phase is over (mission_stage.launch.py,
# mission_manager/stages.py):
#   preflight -> landing     OAK-D + obstacle pipeline, downward camera, live views
#   phase 3 -> exit start    red_zone_node
#   phase 5                  path_planner + corridor_perception
#
# Live view (any browser on the drone's WiFi):
#   http://<pi>:8080/   downward Cam 1.3        http://<pi>:8081/   OAK-D colour
#   (640 px, <= 10 fps, latest frame only: a slow viewer never holds up the
#   mission). record_video:=true also saves both as .mjpeg in the log folder.
#
# Logs: ~/aerothon_logs/mission_<time>/ — one text file per phase + summary.log
# (mission_manager/phase_log.py).

import os
import time

import yaml
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def _setup(context, *args, **kwargs):
    mission_params = os.path.join(
        get_package_share_directory("mission_bringup"), "config", "mission_params.yaml")
    arg = lambda n: LaunchConfiguration(n).perform(context)        # noqa: E731
    sim = arg("sim") == "true"
    fcu_url = arg("fcu_url") or ("tcp://127.0.0.1:5760" if sim else "/dev/ttyAMA2:921600")
    log_dir = os.path.expanduser(os.path.join(
        "~", "aerothon_logs", time.strftime("mission_%Y%m%d_%H%M%S") + ("_sim" if sim else "")))

    actions = []
    if sim:
        # Full-resolution sim camera frames (2.7 MB) must go through shared
        # memory or most are lost as UDP fragments.
        actions.append(SetEnvironmentVariable(
            "FASTRTPS_DEFAULT_PROFILES_FILE",
            os.path.join(get_package_share_directory("mission_bringup"), "config",
                         "fastdds_large_frames.xml")))

    # MAVROS (Jazzy ships apm.launch as XML -> AnyLaunchDescriptionSource)
    actions.append(IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare("mavros"), "launch", "apm.launch"])]),
        launch_arguments={"fcu_url": fcu_url}.items(),
        condition=IfCondition(LaunchConfiguration("launch_mavros")),
    ))

    with open(mission_params) as f:
        cfg = yaml.safe_load(f)

    def node(package, executable, extra=None):
        # extra: ONE dict with the yaml's own sections merged in — a dict
        # listed after the yaml file does not override the yaml's
        # node-specific section (node-specific wins over the dict's /**)
        params = [mission_params]
        if extra:
            params = [{**cfg["/**"]["ros__parameters"],
                       **cfg.get(executable, {}).get("ros__parameters", {}), **extra}]
        return Node(package=package, executable=executable, name=executable,
                    output="screen", parameters=params)

    stage_args = " ".join([
        f"sim:={'true' if sim else 'false'}",
        f"launch_camera:={arg('launch_camera')}", f"camera:={arg('camera')}",
        f"awb_mode:={arg('awb_mode')}", f"live_view:={arg('live_view')}",
        f"record_video:={arg('record_video')}", f"log_dir:={log_dir}"])
    mission = {"staged_launch": True, "stage_launch_args": stage_args, "log_dir": log_dir}
    if sim:
        mission["battery_critical_v"] = 10.5        # SITL simulates a 3S pack (12.6 V)

    # A few seconds apart so the Pi never starts everything at once
    actions += [
        TimerAction(period=3.0, actions=[node("payload", "payload_node")]),
        TimerAction(period=5.0, actions=[node("qr_detection", "qr_scanner_node")]),
        # mission_node last; it waits for MAVROS, then runs the phases.
        TimerAction(period=8.0, actions=[node("mission_manager", "mission_node", mission)]),
    ]
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false", choices=["true", "false"],
                              description="ArduPilot SITL + simulated cameras at the site"),
        DeclareLaunchArgument("launch_mavros", default_value="true", choices=["true", "false"],
                              description="Start MAVROS (false if it runs separately)"),
        DeclareLaunchArgument("fcu_url", default_value="",
                              description="MAVROS FCU URL; empty = /dev/ttyAMA2:921600 "
                                          "(sim: tcp://127.0.0.1:5760)"),
        DeclareLaunchArgument("launch_camera", default_value="true", choices=["true", "false"],
                              description="Start camera_ros for the downward RPi Camera "
                                          "Module 1.3"),
        DeclareLaunchArgument("camera", default_value="0",
                              description="libcamera camera index or name (downward camera)"),
        DeclareLaunchArgument("awb_mode", default_value="5",
                              description="White balance (5 = daylight: keeps the red mat's "
                                          "colour stable; 0 = auto)"),
        DeclareLaunchArgument("live_view", default_value="true", choices=["true", "false"],
                              description="Live view: Cam 1.3 :8080, OAK-D :8081"),
        DeclareLaunchArgument("record_video", default_value="false", choices=["true", "false"],
                              description="Also save both live views as .mjpeg in the log folder"),
        OpaqueFunction(function=_setup),
    ])
