# qr_bcd_mission.launch.py — full search mission in one command:
#
#   ros2 launch mission_bringup qr_bcd_mission.launch.py
#
# take off -> find + read the reference QR -> BCD sweep of the arena with
# red-zone avoidance, matching every QR against the reference -> fly to the
# match, hover hover_s, land on it (no match -> land at the takeoff point).
#
# Starts: MAVROS, the downward Cam3 (camera_ros -> /down_frame), qr_scanner_node,
# red_zone_node (same /down_frame feed) and qr_bcd_mission.
# Needs both workspaces sourced (camera_ws for camera_ros, ros2_ws for the rest).
#
# Place the vehicle ~1 m behind the reference QR, nose pointing at it. The
# arena's SOUTH-WEST corner is (area_x0, area_y0) metres East / North of the
# takeoff point; set them so the reference QR is NOT inside the arena.
#
#   area_x0, area_y0  arena SW corner from the takeoff point (m)
#   area_w, area_h    arena size East / North (m)
#   alt               sweep altitude (m)
#   scan_alt          reference-QR scan altitude (m)
#   speed             sweep speed (m/s). With heading 90 the camera sees only 0.77 m
#                     ahead at 2 m: SITL closest approach to red 0.51 m at 0.3 m/s,
#                     0.36 m at 0.5 m/s. Fly higher (more lookahead) to go faster.
#   margin            keep-out margin around every red zone (m)
#   heading           compass heading held in the sweep (deg; 90 = East puts
#                     the camera's wide axis across the strips)
#   marker            QR side (m) — sets the strip spacing so it fits the frame
#   hover             hover over the matching QR before landing (s)
#   cam_yaw           Cam3 image-top direction vs the nose, deg clockwise
#   awb_mode          Cam3 white balance (libcamera AwbMode; 5 = daylight keeps
#                     red stable, 0 = auto)
#   af_mode           Cam3 autofocus (2 = continuous, bench-validated for QR)
#   fcu_url           MAVROS link
#   sim               true = SITL: no camera_ros, sim_down_camera_node renders
#                     sim_red_zones / sim_qr_codes; fcu_url defaults to SITL's
#
# SITL:
#   ros2 launch mission_bringup qr_bcd_mission.launch.py sim:=true area_y0:=3.0 \
#     sim_red_zones:='[[[4,6],[6,6],[6,8],[4,8]]]' \
#     sim_qr_codes:='[[0,1,"AT-42"],[2.5,11.5,"AT-42"],[8,5,"AT-17"]]'

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
                ["'tcp://127.0.0.1:5760' if '", sim, "' == 'true' else '/dev/ttyAMA0:921600'"])),
        DeclareLaunchArgument("area_x0", default_value="0.0"),
        DeclareLaunchArgument("area_y0", default_value="3.0"),
        DeclareLaunchArgument("area_w", default_value="10.0"),
        DeclareLaunchArgument("area_h", default_value="10.0"),
        DeclareLaunchArgument("alt", default_value="2.0"),
        DeclareLaunchArgument("scan_alt", default_value="2.0"),
        DeclareLaunchArgument("speed", default_value="0.3"),
        DeclareLaunchArgument("margin", default_value="1.0"),
        DeclareLaunchArgument("heading", default_value="90.0"),
        DeclareLaunchArgument("marker", default_value="1.0"),
        DeclareLaunchArgument("hover", default_value="7.0"),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("awb_mode", default_value="5"),
        DeclareLaunchArgument("af_mode", default_value="2"),
        DeclareLaunchArgument("sim_red_zones", default_value="[]"),
        DeclareLaunchArgument("sim_qr_codes", default_value="[]"),
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
            "AfMode": ParameterValue(LaunchConfiguration("af_mode"), value_type=int),
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
            "marker_size_m": _float("marker"),
            "red_zones": ParameterValue(LaunchConfiguration("sim_red_zones"), value_type=str),
            "qr_codes": ParameterValue(LaunchConfiguration("sim_qr_codes"), value_type=str),
        }],
    )

    scanner = Node(
        package="qr_detection",
        executable="qr_scanner_node",
        name="qr_scanner_node",
        output="screen",
        parameters=[{
            "scan_timeout_s": 20.0,
            "cam_yaw_offset_deg": _float("cam_yaw"),
            "delivery_altitude": _float("alt"),
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
        executable="qr_bcd_mission",
        name="qr_bcd_mission",
        output="screen",
        parameters=[{
            "area_x0_m": _float("area_x0"),
            "area_y0_m": _float("area_y0"),
            "area_width_m": _float("area_w"),
            "area_height_m": _float("area_h"),
            "altitude": _float("alt"),
            "scan_altitude_m": _float("scan_alt"),
            "speed_ms": _float("speed"),
            "red_margin_m": _float("margin"),
            "heading_deg": _float("heading"),
            "marker_size_m": _float("marker"),
            "land_hover_s": _float("hover"),
            "qr_scan_timeout_s": 20.0,
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
        # Camera + perception once MAVROS is up (the sim camera and red_zone_node
        # take their origin from the first GPS fix: vehicle on the takeoff point).
        TimerAction(period=3.0, actions=[camera, sim_camera, scanner, red_zone]),
        TimerAction(period=6.0, actions=[mission]),
    ])
