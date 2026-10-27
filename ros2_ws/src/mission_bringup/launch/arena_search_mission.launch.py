# arena_search_mission.launch.py — PHASE 3 (arena search) test flight:
#
#   ros2 launch mission_bringup arena_search_mission.launch.py           (drone)
#   ros2 launch mission_bringup arena_search_mission.launch.py sim:=true (SITL)
#
# take off -> 10 m -> BCD sweep of the test arena, never over red (stop,
# confirm, re-plan around it), matching every QR against the stored
# reference -> over the match: hover, land there (arena_search_mission.py).
#
# Starts: MAVROS, the downward Cam3 (camera_ros -> /down_frame) or in SITL
# sim_down_camera_node, qr_scanner_node (loaded with the reference),
# red_zone_node and arena_search_mission. mission_params.yaml is read as-is;
# the arguments below override it for this run only.
#
# Place the drone in the middle of the test area's near edge (start:=centre,
# default): the arena runs `length` m ahead along its nose and `width`/2 m to
# each side. start:=corner: at its near-left corner, `width` m to the right.
#
#   length, width   test arena (m), ahead of / across the takeoff point
#   start           centre (default) | corner — where the drone sits on the
#                   near edge
#   offset          the arena starts this far ahead of the takeoff point (m)
#   scan_ref        true = PHASES 1 + 3: read the reference QR in the air
#                   first (at alt: cruise_altitude_m := alt for this run;
#                   place it <= ~1 m ahead, nose at it), then sweep. Use
#                   offset:=3 so the reference QR lies outside the arena.
#   alt             sweep altitude (m)                       [arena_alt_m]
#   ref_alt         scan_ref: read the reference QR at this altitude (m),
#                   then straight down/up to alt for the sweep ("" = alt).
#                   A 1 m QR needs height: at 2 m the camera sees only
#                   +-0.77 m along the nose, at 3 m +-1.15 m
#   speed           sweep speed (m/s)                        [arena_sweep_speed_ms]
#   hover           hover over the match before landing (s)
#   use_config      true = fly arena_corners_lat/lon from mission_params.yaml
#   reference_file  the reference QR (phase 1 stores it there); sim: made
#                   from sim_reference_text when left empty. Not loaded
#                   with scan_ref:=true (read in this flight).
#   rc_switch       true = wait for the RC CH7 switch to start
#   cam_yaw         Cam3 image-top direction vs the nose, deg clockwise
#   lens_pos        Cam3 manual focus at start-up, dioptres = 1 / distance (m).
#                   In flight the mission sets it per phase: 1/ref_alt for the
#                   reference QR, 1/alt for the sweep (3 m -> 0.33, 2 m -> 0.5).
#                   Check on the ground first:
#                   rpicam-still -n --autofocus-mode manual --lens-position 0.33 -o t.jpg
#   awb_mode/af_mode  Cam3 white balance (5 = daylight: keeps red stable) /
#                   autofocus (2 = continuous)
#   sim             true = SITL: no camera_ros; sim_down_camera_node renders
#                   sim_qr_codes / sim_red_zones (x East, y North of the
#                   takeoff point; SITL starts nose North, so the arena is
#                   y offset..offset+length and x -width/2..width/2, or
#                   x 0..width with start:=corner — the default sim_qr_codes
#                   are laid out for start:=corner); fcu_url defaults to SITL's
#   sim_reference_text  the QR text the SITL reference is made from ("" = a
#                   QR holding no information; matched by picture)
#   red_ignore      image areas red_zone_node never counts as red (the
#                   drone's orange leg mounts): JSON [[x0,y0,x1,y1],...],
#                   fractions of the frame; drawn grey on the live view
#   live_view       true = live_view_node streams the down camera (with a
#                   status overlay) to http://<drone>:live_view_port/
#
# Needs both workspaces sourced on the drone (source ~/AeroTHON26/env.sh).

import os
import tempfile

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

# SITL test arena (25 x 25 m): the match (no text) at (22, 3); the red mat
# (10 ft square) across the second strip, so the drone meets it first.
SIM_QR_CODES = ('[[22,3,""],[5,18,"AT-17"],[21,22,"XX-99"],[8,5,"B-7"],'
                '[3,12,"Q9"],[12,22,"AT-43"]]')
SIM_RED_ZONES = '[[[15.975,9.475],[19.025,9.475],[19.025,12.525],[15.975,12.525]]]'


def _float(name):
    return ParameterValue(LaunchConfiguration(name), value_type=float)


def _scanner(context, *args, **kwargs):
    """qr_scanner_node with the reference loaded. SITL with no reference_file:
    one made from sim_reference_text."""
    sim = LaunchConfiguration("sim").perform(context) == "true"
    scan_ref = LaunchConfiguration("scan_ref").perform(context) == "true"
    ref = ("" if scan_ref else
           os.path.expanduser(LaunchConfiguration("reference_file").perform(context)))
    if sim and not ref and not scan_ref:
        from qr_detection.sim_reference import make_reference_record
        import json
        text = LaunchConfiguration("sim_reference_text").perform(context)
        ref = os.path.join(tempfile.gettempdir(), "aerothon_sim_reference.json")
        with open(ref, "w") as f:
            json.dump(make_reference_record(text), f)
    return [Node(
        package="qr_detection",
        executable="qr_scanner_node",
        name="qr_scanner_node",
        output="screen",
        parameters=[{
            "cam_yaw_offset_deg": _float("cam_yaw"),
            "delivery_altitude": _float("alt"),
            "reference_file": ref,
        }],
    )]


def generate_launch_description():
    mission_params = os.path.join(
        get_package_share_directory("mission_bringup"), "config", "mission_params.yaml")
    sim = LaunchConfiguration("sim")
    args = [
        DeclareLaunchArgument("sim", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument(
            "fcu_url", default_value=PythonExpression(
                ["'tcp://127.0.0.1:5760' if '", sim, "' == 'true' else '/dev/ttyAMA2:921600'"])),
        DeclareLaunchArgument("length", default_value="25.0"),
        DeclareLaunchArgument("width", default_value="25.0"),
        DeclareLaunchArgument("offset", default_value="0.0"),
        DeclareLaunchArgument("scan_ref", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("start", default_value="centre", choices=["centre", "corner"]),
        DeclareLaunchArgument("red_ignore", default_value=""),
        DeclareLaunchArgument("alt", default_value="10.0"),
        DeclareLaunchArgument("ref_alt", default_value=""),
        DeclareLaunchArgument("speed", default_value="0.5"),
        DeclareLaunchArgument("hover", default_value="5.0"),
        DeclareLaunchArgument("use_config", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument(
            "reference_file", default_value=PythonExpression(
                ["'' if '", sim, "' == 'true' else '~/aerothon_captures/reference_qr.json'"])),
        DeclareLaunchArgument("rc_switch", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("cam_yaw", default_value="0.0"),
        DeclareLaunchArgument("awb_mode", default_value="5"),
        # Manual focus: this libcamera build cannot read the IMX708's PDAF
        # data ("PDAF data in unsupported format"), so autofocus never
        # focuses and the frames were blurry in flight.
        DeclareLaunchArgument("af_mode", default_value="0"),
        DeclareLaunchArgument("lens_pos", default_value="0.4"),
        DeclareLaunchArgument("sim_qr_codes", default_value=SIM_QR_CODES),
        DeclareLaunchArgument("sim_red_zones", default_value=SIM_RED_ZONES),
        DeclareLaunchArgument("sim_reference_text", default_value=""),
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
            "marker_size_m": 1.0,
            "red_zones": ParameterValue(LaunchConfiguration("sim_red_zones"), value_type=str),
            "qr_codes": ParameterValue(LaunchConfiguration("sim_qr_codes"), value_type=str),
        }],
    )

    red_zone = Node(
        package="qr_detection",
        executable="red_zone_node",
        name="red_zone_node",
        output="screen",
        # red confirmed in flight (no hover), and red joined to it added
        parameters=[{"source": "topic", "mount_yaw_deg": _float("cam_yaw"),
                     "grow_known_zones": True, "confirm_in_flight": True,
                     "ignore_regions": ParameterValue(LaunchConfiguration("red_ignore"),
                                                      value_type=str)}],
    )

    mission = Node(
        package="mission_manager",
        executable="arena_search_mission",
        name="mission_node",                # reads the mission_node: yaml section
        output="screen",
        parameters=[mission_params, {
            "arena_test_length_m": _float("length"),
            "arena_test_width_m": _float("width"),
            "arena_test_use_config": ParameterValue(LaunchConfiguration("use_config"),
                                                    value_type=bool),
            "arena_test_hover_s": _float("hover"),
            "arena_test_offset_m": _float("offset"),
            "arena_test_start": LaunchConfiguration("start"),
            "arena_test_scan_reference": ParameterValue(LaunchConfiguration("scan_ref"),
                                                        value_type=bool),
            # phase 1 reads the reference at cruise_altitude_m: ref_alt (or
            # the sweep's altitude) when it is flown here, else the yaml's
            "cruise_altitude_m": ParameterValue(PythonExpression(
                ["float('", LaunchConfiguration("ref_alt"), "' or '",
                 LaunchConfiguration("alt"), "') if '",
                 LaunchConfiguration("scan_ref"), "' == 'true' else ",
                 _yaml_value(mission_params, "cruise_altitude_m")]), value_type=float),
            "arena_alt_m": _float("alt"),
            "arena_sweep_speed_ms": _float("speed"),
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
                                            value_type=int),
                     "ignore_regions": ParameterValue(LaunchConfiguration("red_ignore"),
                                                      value_type=str)}],
    )

    # Full-resolution sim frames (2.7 MB) must go through shared memory or
    # most are lost as UDP fragments (config/fastdds_large_frames.xml).
    large_frames = SetEnvironmentVariable(
        "FASTRTPS_DEFAULT_PROFILES_FILE",
        PathJoinSubstitution([FindPackageShare("mission_bringup"), "config",
                              "fastdds_large_frames.xml"]),
        condition=IfCondition(sim))

    return LaunchDescription(args + [
        large_frames,
        mavros,
        # Camera + perception once MAVROS is up (the sim camera and
        # red_zone_node take their origin from the first GPS fix: vehicle on
        # the takeoff point).
        TimerAction(period=3.0, actions=[camera, sim_camera, red_zone, live_view,
                                         OpaqueFunction(function=_scanner)]),
        TimerAction(period=6.0, actions=[mission]),
    ])


def _yaml_value(mission_params: str, name: str) -> str:
    """A mission_node parameter as mission_params.yaml sets it (hardware value)."""
    import yaml
    with open(mission_params) as f:
        return str(yaml.safe_load(f)["mission_node"]["ros__parameters"][name])
