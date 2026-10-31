# mission_stage.launch.py — one group of mission nodes, started by mission_node
# only when its phase needs it and stopped when it no longer does
# (mission_manager/stages.py; the Pi 5 browned out with everything powering
# up at once).
#
#   ros2 launch mission_bringup mission_stage.launch.py stage:=<stage> \
#       sim:=false launch_camera:=true camera:=0 awb_mode:=5 live_view:=true \
#       record_video:=false log_dir:=<dir>
#
# Stages:
#   oak            OAK-D Lite driver + obstacle pipeline (sector_builder ->
#                  OBSTACLE_DISTANCE to the FC, coverage_guard -> sensor_live)
#                  + OAK live view http://<pi>:8081/. Preflight -> landing.
#   down_cam       Downward RPi Camera Module 1.3 (camera_ros) + its live view
#                  http://<pi>:8080/. Preflight -> landing.
#   arena          red_zone_node. Phase 3 -> the exit corridor's start point.
#   exit_corridor  path_planner_node + corridor_perception_node. Phase 5.
#
# sim:=true (fly.sh sim): the same stages against ArduPilot SITL, cameras
# simulated AT THE REAL SITE (mission waypoints): sitl_corridor_sim renders
# the OAK-D depth / colour of both corridors (exit corridor with obstacles),
# sim_down_camera_node the reference QR at wp_qr and the arena's QRs + red mat.

import json
import math
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

STAGES = ("oak", "down_cam", "arena", "exit_corridor")
M_PER_DEG_LAT = 111_320.0       # as mission_common.survey


def _local(p, home):
    """(east, north) metres of [lat, lon] p from home (as survey.to_local)."""
    return ((p[1] - home[1]) * M_PER_DEG_LAT * math.cos(math.radians(home[0])),
            (p[0] - home[0]) * M_PER_DEG_LAT)


def sim_world(cfg: dict) -> dict:
    """The SITL world at the real site, x East / y North of wp_home:
    corridor waypoints for sitl_corridor_sim (first = mouth_offset_m before
    the mouth, last = exit_offset_m past the end, so the walls run exactly
    mouth -> end) and the downward camera's QRs and red mat (the reference
    at wp_qr; the match, decoys and a 10 ft red mat inside the arena)."""
    wp = cfg["/**"]["ros__parameters"]
    mn = cfg["mission_node"]["ros__parameters"]
    sim = cfg["sitl_corridor_sim"]["ros__parameters"]
    home = wp["wp_home"]
    out = {}
    for i, name in ((1, "entry"), (2, "exit")):
        m, e = _local(wp[f"wp_{name}_mouth"], home), _local(wp[f"wp_{name}_end"], home)
        n = math.hypot(e[0] - m[0], e[1] - m[1])
        u = ((e[0] - m[0]) / n, (e[1] - m[1]) / n)
        a, b = sim["mouth_offset_m"], sim["exit_offset_m"]
        out[f"corridor{i}_waypoints_x"] = [m[0] - a * u[0], e[0] + b * u[0]]
        out[f"corridor{i}_waypoints_y"] = [m[1] - a * u[1], e[1] + b * u[1]]
    # arena point at fractions (a along corner 0 -> 1, b along corner 0 -> 3)
    c = [_local((la, lo), home) for la, lo in zip(mn["arena_corners_lat"],
                                                  mn["arena_corners_lon"])]

    def at(a, b):
        return [c[0][0] + a * (c[1][0] - c[0][0]) + b * (c[3][0] - c[0][0]),
                c[0][1] + a * (c[1][1] - c[0][1]) + b * (c[3][1] - c[0][1])]

    qr = _local(wp["wp_qr"], home)
    qrs = [[qr[0], qr[1], ""],                          # phase 1 reference
           at(0.25, 0.30) + [""],                       # the match (same picture)
           at(0.80, 0.20) + ["AT-17"], at(0.15, 0.75) + ["XX-99"],
           at(0.75, 0.80) + ["B-7"]]
    mx, my = at(0.55, 0.55)
    h = 3.048 / 2                                       # 10 ft mat
    mat = [[mx - h, my - h], [mx + h, my - h], [mx + h, my + h], [mx - h, my + h]]
    out["qr_codes"] = json.dumps(qrs)
    out["red_zones"] = json.dumps([mat])
    return out


def _setup(context, *args, **kwargs):
    stage = LaunchConfiguration("stage").perform(context)
    if stage not in STAGES:
        raise RuntimeError(f"stage:={stage} — one of {', '.join(STAGES)}")
    sim = LaunchConfiguration("sim").perform(context) == "true"
    launch_camera = LaunchConfiguration("launch_camera").perform(context) == "true"
    live_view = LaunchConfiguration("live_view").perform(context) == "true"
    record = LaunchConfiguration("record_video").perform(context) == "true"
    log_dir = os.path.expanduser(LaunchConfiguration("log_dir").perform(context))

    mission_params = os.path.join(
        get_package_share_directory("mission_bringup"), "config", "mission_params.yaml")
    with open(mission_params) as f:
        cfg = yaml.safe_load(f)
    avoidance_share = get_package_share_directory("obstacle_avoidance")
    avoidance_config = os.path.join(avoidance_share, "config", "avoidance.yaml")
    depth_remaps = [("~/depth", "/oak/stereo/image_raw"),
                    ("~/camera_info", "/oak/stereo/camera_info")]
    rgb_remaps = [("~/rgb", "/oak/rgb/image_raw"),
                  ("~/rgb_camera_info", "/oak/rgb/camera_info")]
    # SITL depth comes at 5 Hz (hardware 10 Hz): wider stale cutoffs, the
    # guard 0.2 s ahead of the builder watchdog; half-resolution render.
    sim_timeouts = {"depth_stale_timeout_s": 0.6, "depth_watchdog_timeout_s": 0.8}

    def live_view_of(label, topic, port, downward):
        return Node(
            package="qr_detection",
            executable="live_view_node",
            name=f"live_view_{'down' if downward else 'oak'}",
            output="screen",
            parameters=[{"label": label, "topic": topic, "port": port,
                         "downward": downward, "record_dir": log_dir if record else ""}],
        )

    if stage == "oak":
        if sim:
            world = sim_world(cfg)
            nodes = [
                # ONE dict: a dict listed after the yaml file does not override
                # the yaml's own sitl_corridor_sim: section (node-specific wins)
                Node(package="obstacle_avoidance", executable="sitl_corridor_sim",
                     name="sitl_corridor_sim", output="screen",
                     parameters=[{
                         **cfg["sitl_corridor_sim"]["ros__parameters"],
                         "origin_lat": cfg["/**"]["ros__parameters"]["wp_home"][0],
                         "origin_lon": cfg["/**"]["ros__parameters"]["wp_home"][1],
                         **{k: v for k, v in world.items() if k.startswith("corridor")},
                         "corridor1_length_m": 0.0, "corridor2_length_m": 0.0,
                         # the real OAK-D on the 11.5 ft corridor: walls past
                         # 3.5 m give no depth; GPS wanders 0.3 m over 60 s
                         "wall_depth_max_m": 3.5,
                         "gps_drift_m": 0.3, "gps_drift_period_s": 60.0}]),
                Node(package="obstacle_avoidance", executable="sector_builder_node",
                     name="sector_builder_node", output="screen",
                     parameters=[avoidance_config, sim_timeouts], remappings=depth_remaps),
                Node(package="obstacle_avoidance", executable="coverage_guard_node",
                     name="coverage_guard_node", output="screen",
                     parameters=[avoidance_config, sim_timeouts]),
            ]
        else:
            # MUST use obstacle_avoidance's params file (oak_d_lite_depth.yaml
            # header: wrong intrinsics mis-bin every bearing).
            nodes = [
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([PathJoinSubstitution([
                        FindPackageShare("depthai_ros_driver"), "launch", "camera.launch.py"])]),
                    launch_arguments={
                        "name": "oak",
                        "camera_model": "OAK-D-LITE",
                        "params_file": os.path.join(avoidance_share, "config",
                                                    "oak_d_lite_depth.yaml"),
                    }.items(),
                ),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(avoidance_share, "launch", "sector_builder_hw.launch.py")),
                    launch_arguments={"obstacle_path": "sink",
                                      "corridor_perception": "false"}.items(),
                ),
            ]
        if live_view:
            nodes.append(live_view_of("OAK-D (forward)", "/oak/rgb/image_raw", 8081, False))
        return nodes

    if stage == "down_cam":
        nodes = []
        if sim:
            world = sim_world(cfg)
            nodes.append(Node(
                package="qr_detection", executable="sim_down_camera_node",
                name="sim_down_camera", output="screen",
                parameters=[{"marker_size_m": 1.0, "qr_codes": world["qr_codes"],
                             "red_zones": world["red_zones"],
                             # rendered as mounted: the nodes flip it back
                             "mirror": bool(cfg["qr_scanner_node"]["ros__parameters"]
                                            .get("cam_mirror", False))}]))
        elif launch_camera:
            # camera_ros: an INTEGER is the camera's index, a STRING its name
            # ("0" as a string = "camera with name 0 does not exist")
            camera = LaunchConfiguration("camera").perform(context)
            camera = int(camera) if camera.isdigit() else camera
            # RPi Camera Module 1.3 (OV5647, fixed focus). camera_ros only
            # declares controls the sensor has, so AfMode / LensPosition (and
            # mission_node's _focus_camera) are ignored on it; they only act on a
            # Camera Module 3. camera_info is NOT remapped to /camera_info: an
            # uncalibrated camera_ros publishes K = 0, which would overwrite
            # qr_scanner_node's camera_fx.
            cruise = float(cfg["mission_node"]["ros__parameters"]["cruise_altitude_m"])
            nodes.append(Node(
                package="camera_ros",
                executable="camera_node",
                name="camera",
                output="screen",
                parameters=[{
                    "camera": camera,
                    "width": 1280,
                    "height": 720,
                    "format": "BGR888",
                    # full-width binned mode (else libcamera may pick the cropped
                    # 1080p one, ~41 deg). Remove for a Camera Module 3.
                    "sensor_mode": "1296:972",
                    "AfMode": 0,
                    "LensPosition": 1.0 / max(cruise, 0.1),
                    "AwbMode": int(LaunchConfiguration("awb_mode").perform(context)),
                }],
                remappings=[("/camera/image_raw", "/down_frame")],
            ))
        if live_view:
            nodes.append(live_view_of("Cam 1.3 (down)", "/down_frame", 8080, True))
        return nodes

    if stage == "exit_corridor":
        # path_planner flies the exit corridor on corridor_perception's walls
        # and obstacles (depth + colour on hardware; the sim's colour stream
        # is banner-only, so there it runs on depth at the sim's resolution).
        perception = Node(
            package="obstacle_avoidance",
            executable="corridor_perception_node",
            name="corridor_perception_node",
            output="screen",
            parameters=[avoidance_config] + ([{"corridor_stride_px": 2}] if sim else []),
            remappings=depth_remaps + ([] if sim else rgb_remaps),
        )
        planner = Node(
            package="path_planner",
            executable="path_planner_node",
            name="path_planner_node",
            output="screen",
            parameters=[mission_params],
        )
        return [perception, planner]

    # arena: red no-fly mats on the downward camera; one red mat: confirmed in
    # flight (no hover), red joined to it added in flight
    return [Node(
        package="qr_detection",
        executable="red_zone_node",
        name="red_zone_node",
        output="screen",
        parameters=[mission_params,     # red_zone_node: cam_mirror
                    {"source": "topic", "grow_known_zones": True, "confirm_in_flight": True}],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("stage", description=" | ".join(STAGES)),
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("launch_camera", default_value="true"),
        DeclareLaunchArgument("camera", default_value="0"),
        DeclareLaunchArgument("awb_mode", default_value="5"),
        DeclareLaunchArgument("live_view", default_value="true"),
        DeclareLaunchArgument("record_video", default_value="false"),
        DeclareLaunchArgument("log_dir", default_value="~/aerothon_logs/stage"),
        OpaqueFunction(function=_setup),
    ])
