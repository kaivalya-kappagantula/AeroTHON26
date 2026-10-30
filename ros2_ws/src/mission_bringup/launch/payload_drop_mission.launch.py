# payload_drop_mission.launch.py — PHASE 4 (payload drop) test flight:
#
#   ros2 launch mission_bringup payload_drop_mission.launch.py           (drone)
#   ros2 launch mission_bringup payload_drop_mission.launch.py sim:=true (SITL)
#
# take off -> alt -> `ahead` m forward along the nose -> find the QR below ->
# the mission's drop (straight down to drop_alt, camera-centred, release,
# straight back up to alt) -> back over the takeoff point -> land
# (payload_drop_mission.py). Place a printed QR `ahead` m in front of the
# drone's nose.
#
# Starts: MAVROS, the downward Cam3 (camera_ros -> /down_frame) or in SITL
# sim_down_camera_node, qr_scanner_node (/perception/locate_qr),
# payload_node and payload_drop_mission. mission_params.yaml is read as-is;
# the arguments below override it for this run only.
#
#   ahead           the QR this far ahead of the takeoff point (m)
#   alt             flight altitude to and from the QR (m)   [arena_alt_m]
#   drop_alt        release altitude (m)                     [drop_altitude_m]
#   rc_switch       true = wait for the RC CH7 switch to start
#   cam_yaw         Cam3 image-top direction vs the nose, deg clockwise
#   cam_mirror    true (default, as the Cam3 is mounted) = its image is mirrored
#                 left-right vs the real scene; the nodes flip it back
#   lens_pos        Cam3 manual focus at start-up, dioptres = 1 / distance (m);
#                   in flight the mission sets 1/alt, then 1/drop_alt
#   awb_mode/af_mode  Cam3 white balance / autofocus (0 = manual)
#   sim             true = SITL: sim_down_camera_node renders sim_qr_codes
#                   ([x East, y North, text] of the takeoff point; SITL
#                   starts nose North). Default: the QR 5 m ahead, 0.4 m off
#                   to the right (so the camera centring has work to do), and
#                   a second QR 3.5 m to the side of it
#   live_view       true = live_view_node streams the down camera to
#                   http://<drone>:live_view_port/
#
# FC: payload_node drives the winch on RELAY1/RELAY2 with DO_SET_RELAY — see
# payload_node.py for the RELAYn_* / SERVOn_FUNCTION setup. Check the mechanism
# on the bench (ros2 service call /payload/release std_srvs/srv/Trigger)
# before flying it.
#
# Needs both workspaces sourced on the drone (source ~/AeroTHON26/env.sh).

import os

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

SIM_QR_CODES = '[[0.4,5,"DROP"],[3.9,5.5,"OTHER"]]'


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _yaml_value(mission_params: str, name: str) -> str:
    """A mission_node parameter as mission_params.yaml sets it (hardware value)."""
    import yaml
    with open(mission_params) as f:
        return str(yaml.safe_load(f)["mission_node"]["ros__parameters"][name])


def generate_launch_description():
    mission_params = os.path.join(
        get_package_share_directory("mission_bringup"), "config", "mission_params.yaml")
    sim = LaunchConfiguration("sim")
    args = [
        DeclareLaunchArgument("sim", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument(
            "fcu_url", default_value=PythonExpression(
                ["'tcp://127.0.0.1:5760' if '", sim, "' == 'true' else '/dev/ttyAMA2:921600'"])),
        DeclareLaunchArgument("ahead", default_value="5.0"),
        DeclareLaunchArgument("alt", default_value=_yaml_value(mission_params, "arena_alt_m")),
        DeclareLaunchArgument("drop_alt",
                              default_value=_yaml_value(mission_params, "drop_altitude_m")),
        DeclareLaunchArgument("rc_switch", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("cam_mirror", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("awb_mode", default_value="5"),
        DeclareLaunchArgument("af_mode", default_value="0"),
        DeclareLaunchArgument("lens_pos", default_value="0.1"),
        DeclareLaunchArgument("sim_qr_codes", default_value=SIM_QR_CODES),
        DeclareLaunchArgument("live_view", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("live_view_port", default_value="8080"),
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
            "LensPosition": _float("lens_pos"),
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
            "mirror": ParameterValue(LaunchConfiguration("cam_mirror"), value_type=bool),
            "marker_size_m": 1.0,
            "red_zones": "[]",
            "qr_codes": ParameterValue(LaunchConfiguration("sim_qr_codes"), value_type=str),
        }],
    )

    scanner = Node(
        package="qr_detection",
        executable="qr_scanner_node",
        name="qr_scanner_node",
        output="screen",
        parameters=[{"cam_yaw_offset_deg": _float("cam_yaw"),
                     "cam_mirror": ParameterValue(LaunchConfiguration("cam_mirror"), value_type=bool),
                     "delivery_altitude": _float("drop_alt")}],
    )

    payload = Node(
        package="payload",
        executable="payload_node",
        name="payload_node",
        output="screen",
        parameters=[mission_params],
    )

    mission = Node(
        package="mission_manager",
        executable="payload_drop_mission",
        name="mission_node",                # reads the mission_node: yaml section
        output="screen",
        parameters=[mission_params, {
            "drop_test_ahead_m": _float("ahead"),
            "arena_alt_m": _float("alt"),
            "drop_altitude_m": _float("drop_alt"),
            "require_rc_switch": ParameterValue(LaunchConfiguration("rc_switch"), value_type=bool),
            "require_avoidance_sensor": False,    # no OAK-D pipeline in this test
            # SITL simulates a 3S pack (12.6 V); on the drone the yaml's value
            "battery_critical_v": ParameterValue(PythonExpression(
                ["10.5 if '", sim, "' == 'true' else ",
                 _yaml_value(mission_params, "battery_critical_v")]),
                value_type=float),
        }],
    )

    live_view = Node(
        package="qr_detection",
        executable="live_view_node",
        name="live_view_node",
        output="screen",
        condition=IfCondition(LaunchConfiguration("live_view")),
        parameters=[{"port": ParameterValue(LaunchConfiguration("live_view_port"),
                                            value_type=int)}],
    )

    # Full-resolution sim frames must go through shared memory
    # (config/fastdds_large_frames.xml).
    large_frames = SetEnvironmentVariable(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        PathJoinSubstitution([FindPackageShare("mission_bringup"), "config",
                              "fastdds_large_frames.xml"]),
        condition=IfCondition(sim))

    return LaunchDescription(args + [
        large_frames,
        mavros,
        TimerAction(period=3.0, actions=[camera, sim_camera, scanner, payload, live_view]),
        TimerAction(period=6.0, actions=[mission]),
    ])
