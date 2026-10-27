# mission.launch.py — Master launch file for Aerothon UAS Mission 2 (hardware)
#
#   ros2 launch mission_bringup mission.launch.py launch_mavros:=true \
#       fcu_url:=/dev/ttyAMA0:921600 launch_camera:=true
#
# Starts: MAVROS (optional), path planner, payload, QR scanner, corridor
# ground-marker detector (downward camera), the OAK-D Lite driver +
# obstacle_avoidance pipeline (sector_builder -> OBSTACLE_DISTANCE to
# ArduPilot, coverage_guard health Bools, corridor_perception for the
# corridors), banner_node (green entrance banner, OAK colour camera), the RPi
# Camera 3 driver (optional), and finally the mission node.
#
# Pi 5 <-> Pixhawk 6C Mini on TELEM2 (GPIO14/15 UART0):
#   /boot/firmware/config.txt: dtparam=uart0=on  (gives /dev/ttyAMA0 on Pi 5)
#   FC: SERIAL2_PROTOCOL=2, SERIAL2_BAUD=921
# USB instead: fcu_url:=/dev/ttyACM0:115200

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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


def generate_launch_description():
    mission_params = os.path.join(
        get_package_share_directory("mission_bringup"), "config", "mission_params.yaml")

    # ----- Launch arguments -----
    launch_mavros_arg = DeclareLaunchArgument(
        "launch_mavros", default_value="false",
        description="Launch MAVROS (set false if started separately)")
    fcu_url_arg = DeclareLaunchArgument(
        "fcu_url", default_value="tcp://127.0.0.1:5760",
        description="MAVROS FCU URL (SITL tcp, or serial device:baud for hardware)")
    launch_camera_arg = DeclareLaunchArgument(
        "launch_camera", default_value="false",
        description="Start camera_ros for the downward RPi Camera Module 3 "
                    "(needs camera_ros built against Raspberry Pi's libcamera)")
    camera_arg = DeclareLaunchArgument(
        "camera", default_value="0",
        description="libcamera camera index or name for the downward camera")

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
    mission_node = mission_node_of("mission_manager", "mission_node")

    # ================================================================
    # Downward camera: RPi Camera Module 3 via camera_ros
    # ================================================================
    # libcamera's RGB888 is laid out B,G,R in memory, which camera_ros
    # publishes as "bgr8" — what qr_scanner_node converts to anyway.
    down_camera = Node(
        package="camera_ros",
        executable="camera_node",
        name="down_camera",
        output="screen",
        parameters=[{
            "camera": LaunchConfiguration("camera"),
            "width": 1280,
            "height": 720,
            "format": "RGB888",
            "role": "video",
        }],
        remappings=[
            ("~/image_raw", "/down_frame"),
            ("~/camera_info", "/camera_info"),
        ],
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

    return LaunchDescription([
        launch_mavros_arg,
        fcu_url_arg,
        launch_camera_arg,
        camera_arg,

        mavros_launch,
        payload_node,
        path_planner_node,
        qr_scanner_node,
        ground_marker_node,
        down_camera,
        oakd_launch,
        avoidance_launch,
        banner_node,

        # Mission node last; it also waits for MAVROS + the pilot's switch.
        TimerAction(period=5.0, actions=[mission_node]),
    ])
