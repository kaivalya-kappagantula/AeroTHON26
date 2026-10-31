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
#   down_cam       Downward RPi Camera Module 1.3 (camera_ros) + its live view
#                  http://<pi>:8080/. Preflight -> landing.
#   arena          red_zone_node. Phase 3 -> the exit corridor's start point.
#
# sim:=true (fly.sh sim): the same stages against ArduPilot SITL, the
# downward camera simulated AT THE REAL SITE (mission waypoints):
# sim_down_camera_node renders the reference QR at wp_qr and the arena's
# QRs + red mat.

import json
import math
import os

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

STAGES = ("down_cam", "arena")
M_PER_DEG_LAT = 111_320.0       # as mission_common.survey


def _local(p, home):
    """(east, north) metres of [lat, lon] p from home (as survey.to_local)."""
    return ((p[1] - home[1]) * M_PER_DEG_LAT * math.cos(math.radians(home[0])),
            (p[0] - home[0]) * M_PER_DEG_LAT)


def sim_world(cfg: dict) -> dict:
    """The SITL world at the real site, x East / y North of wp_home: the
    downward camera's QRs and red mat (the reference at wp_qr; the match,
    decoys and a 10 ft red mat inside the arena)."""
    wp = cfg["/**"]["ros__parameters"]
    mn = cfg["mission_node"]["ros__parameters"]
    home = wp["wp_home"]
    out = {}
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
    def live_view_of(label, topic, port, downward):
        return Node(
            package="qr_detection",
            executable="live_view_node",
            name="live_view_down",
            output="screen",
            parameters=[{"label": label, "topic": topic, "port": port,
                         "downward": downward, "record_dir": log_dir if record else ""}],
        )

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
