# mission.launch.py — Master launch file for Aerothon UAS Mission 2 (hardware)
#
#   ros2 launch mission_bringup mission.launch.py launch_mavros:=true \
#       fcu_url:=/dev/ttyAMA2:921600 launch_camera:=true
#
# Starts: MAVROS (optional), path planner, payload, QR scanner, corridor
# ground-marker detector (downward camera), the OAK-D Lite driver +
# obstacle_avoidance pipeline (sector_builder -> OBSTACLE_DISTANCE to
# ArduPilot, coverage_guard health Bools, corridor_perception for the
# corridors), banner_node (green entrance banner) and banner_blue_node (blue
# exit-corridor banner) on the OAK colour camera, the RPi
# Camera 3 driver (optional), and finally the mission node.
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
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
    AnyLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue


def _yaml_value(mission_params: str, name: str) -> str:
    """A mission_node parameter as mission_params.yaml sets it (hardware value)."""
    import yaml
    with open(mission_params) as f:
        return str(yaml.safe_load(f)["mission_node"]["ros__parameters"][name])


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
        description="Start camera_ros for the downward RPi Camera Module 3 "
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
    ground_marker_node = mission_node_of("qr_detection", "ground_marker_node")
    # Red no-fly mats in the arena (phase 3), on the same /down_frame feed
    red_zone_node = Node(
        package="qr_detection",
        executable="red_zone_node",
        name="red_zone_node",
        output="screen",
        # one red mat: confirmed in flight (no hover), red joined to it added in flight
        parameters=[mission_params,     # red_zone_node: cam_mirror
                    {"source": "topic", "grow_known_zones": True, "confirm_in_flight": True}],
    )
    mission_node = mission_node_of("mission_manager", "mission_node")

    # ================================================================
    # Downward camera: RPi Camera Module 3 via camera_ros
    # ================================================================
    # Exactly the camera the phase test flights use (qr_scan_mission,
    # arena_search_mission, payload_drop_mission): node "camera" (the
    # mission's _focus_camera sets /camera LensPosition per phase), BGR888,
    # manual focus — this libcamera build cannot read the IMX708's PDAF data,
    # so autofocus never focuses. Starts focused for the reference QR
    # (cruise_altitude_m). camera_info is NOT remapped to /camera_info: an
    # uncalibrated camera_ros publishes K = 0, which would overwrite
    # qr_scanner_node's camera_fx (it sets the marker size looked for).
    down_camera = Node(
        package="camera_ros",
        executable="camera_node",
        name="camera",
        output="screen",
        parameters=[{
            "camera": LaunchConfiguration("camera"),
            "width": 1280,
            "height": 720,
            "format": "BGR888",
            # Camera Module 1.3: its full-width binned mode (else libcamera may
            # pick the cropped 1080p one, ~41 deg). Remove for a Camera Module 3.
            "sensor_mode": "1296:972",
            "AfMode": 0,
            "LensPosition": 1.0 / max(float(_yaml_value(mission_params, "cruise_altitude_m")),
                                      0.1),
            "AwbMode": ParameterValue(LaunchConfiguration("awb_mode"), value_type=int),
        }],
        remappings=[("/camera/image_raw", "/down_frame")],
        condition=IfCondition(LaunchConfiguration("launch_camera")),
    )

    # ================================================================
    # OAK-D Lite driver (depthai-ros)
    # ================================================================
    # MUST use obstacle_avoidance's params file. The driver defaults publish
    # 1280x720 intrinsics next to a 640x480 depth image on the OAK-D Lite,
    # which silently mis-bins every obstacle bearing, and the default RGBD
    # pipeline's NN blob upload can crash the container. See the yaml header.
    avoidance_share = get_package_share_directory("obstacle_avoidance")
    oak_params = os.path.join(avoidance_share, "config", "oak_d_lite_depth.yaml")
    oakd_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("depthai_ros_driver"), "launch", "camera.launch.py"
            ])
        ]),
        launch_arguments={
            "name": "oak",  # must match the /oak: key in oak_params
            "camera_model": "OAK-D-LITE",
            "params_file": oak_params,
        }.items(),
    )

    # ================================================================
    # Obstacle pipeline -> ArduPilot proximity (PRX1_TYPE=2) + health Bools
    # ================================================================
    avoidance_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(avoidance_share, "launch", "sector_builder_hw.launch.py")),
        launch_arguments={"obstacle_path": "sink"}.items(),
    )

    # Green entrance banner (entry corridor). Needs the OAK colour stream
    # (oak_d_lite_depth.yaml: i_pipeline_type RGBD).
    banner_node = Node(
        package="obstacle_avoidance",
        executable="banner_node",
        name="banner_node",
        output="screen",
        parameters=[os.path.join(avoidance_share, "config", "avoidance.yaml")],
        remappings=[
            ("~/rgb", "/oak/rgb/image_raw"),
            ("~/rgb_camera_info", "/oak/rgb/camera_info"),
            ("~/depth", "/oak/stereo/image_raw"),
            ("~/camera_info", "/oak/stereo/camera_info"),
        ],
    )

    # Blue exit-corridor banner (phase 5): the same node, blue values from
    # banner_blue.yaml on top, publishing /corridor/banner_blue.
    banner_blue_node = Node(
        package="obstacle_avoidance",
        executable="banner_node",
        name="banner_blue_node",
        output="screen",
        parameters=[os.path.join(avoidance_share, "config", "avoidance.yaml"),
                    os.path.join(avoidance_share, "config", "banner_blue.yaml")],
        remappings=[
            ("~/rgb", "/oak/rgb/image_raw"),
            ("~/rgb_camera_info", "/oak/rgb/camera_info"),
            ("~/depth", "/oak/stereo/image_raw"),
            ("~/camera_info", "/oak/stereo/camera_info"),
        ],
    )

    # ================================================================
    # Live view (watch only) + flight logs
    # ================================================================
    def live_view_of(label, topic, port, downward):
        return Node(
            package="qr_detection",
            executable="live_view_node",
            name=f"live_view_{'down' if downward else 'oak'}",
            output="screen",
            parameters=[{"label": label, "topic": topic, "port": port,
                         "downward": downward, "record_dir": log_dir}],
            condition=IfCondition(LaunchConfiguration("live_view")),
        )

    live_view_down = live_view_of("Cam3 (down)", "/down_frame", 8080, True)
    live_view_oak = live_view_of("OAK (forward)", "/oak/rgb/image_raw", 8081, False)

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

        mavros_launch,
        payload_node,
        path_planner_node,
        qr_scanner_node,
        ground_marker_node,
        red_zone_node,
        down_camera,
        oakd_launch,
        avoidance_launch,
        banner_node,
        banner_blue_node,
        live_view_down,
        live_view_oak,
        record,

        # Mission node last; it also waits for MAVROS + the pilot's switch.
        TimerAction(period=5.0, actions=[mission_node]),
    ])
