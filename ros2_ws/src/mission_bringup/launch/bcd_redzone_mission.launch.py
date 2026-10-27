# bcd_redzone_mission.launch.py — BCD sweep that detours around red zones
# (no QR), in one command:
#
#   ros2 launch mission_bringup bcd_redzone_mission.launch.py
#
# Starts: MAVROS, the downward Cam3 (camera_ros -> /down_frame), red_zone_node
# and bcd_redzone_mission. The takeoff point is the SOUTH-WEST corner of the
# area; the area extends East (width) and North (height). For the full QR
# search use arena_search_mission.launch.py.
#
#   area_w, area_h  sweep area (m)
#   alt             sweep altitude (m)
#   speed           sweep speed (m/s) — 1.0 got within 0.13 m of red in SITL
#   margin          keep-out margin around every red zone (m)
#   heading         compass heading held in the sweep (deg; 0 = North puts the
#                   camera's wide axis along the strips when cam_yaw is 0)
#   cam_yaw         Cam3 image-top direction vs the nose, deg clockwise (0 = aligned)
#   awb_mode        Cam3 white balance (5 = daylight, 0 = auto)
#   sim             true = SITL: sim_down_camera_node renders sim_zones instead
#                   of camera_ros; fcu_url defaults to SITL's
#   sim_zones       JSON red polygons, x East / y North metres from the takeoff point
#
# SITL example:
#   ros2 launch mission_bringup bcd_redzone_mission.launch.py sim:=true \
#       sim_zones:='[[[4,4],[6,4],[6,6],[4,6]]]'

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


def generate_launch_description():
    sim = LaunchConfiguration("sim")
    args = [
        DeclareLaunchArgument("sim", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument(
            "fcu_url", default_value=PythonExpression(
                ["'tcp://127.0.0.1:5760' if '", sim, "' == 'true' else '/dev/ttyAMA2:921600'"])),
        DeclareLaunchArgument("area_w", default_value="10.0"),
        DeclareLaunchArgument("area_h", default_value="10.0"),
        DeclareLaunchArgument("alt", default_value="2.0"),
        DeclareLaunchArgument("speed", default_value="0.5"),
        DeclareLaunchArgument("margin", default_value="1.0"),
        DeclareLaunchArgument("heading", default_value="0.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("awb_mode", default_value="5"),
        DeclareLaunchArgument("sim_zones", default_value="[]"),
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
            "AwbMode": ParameterValue(LaunchConfiguration("awb_mode"), value_type=int),
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
            "red_zones": ParameterValue(LaunchConfiguration("sim_zones"), value_type=str),
        }],
    )

    red_zone = Node(
        package="qr_detection",
        executable="red_zone_node",
        name="red_zone_node",
        output="screen",
        parameters=[{"source": "topic", "mount_yaw_deg": _float("cam_yaw")}],
    )

    mission = Node(
        package="mission_manager",
        executable="bcd_redzone_mission",
        name="bcd_redzone_mission",
        output="screen",
        parameters=[{
            "area_width_m": _float("area_w"),
            "area_height_m": _float("area_h"),
            "altitude": _float("alt"),
            "speed_ms": _float("speed"),
            "red_margin_m": _float("margin"),
            "heading_deg": _float("heading"),
        }],
    )

    # Full-resolution frames (2.7 MB) must go through shared memory or
    # most are lost as UDP fragments: ~4 of 30 fps reached the Python
    # nodes in SITL without this (config/fastdds_large_frames.xml).
    large_frames = SetEnvironmentVariable(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        PathJoinSubstitution([FindPackageShare("mission_bringup"), "config",
                              "fastdds_large_frames.xml"]))

    return LaunchDescription(args + [
        large_frames,
        mavros,
        TimerAction(period=3.0, actions=[camera, sim_camera, red_zone]),
        TimerAction(period=5.0, actions=[mission]),
    ])
