# mission_stage.launch.py — one group of mission nodes, started by mission_node
# only when its phase needs it (mission.launch.py starts the light nodes; the
# Pi 5 browned out with everything powering up at once).
#
#   ros2 launch mission_bringup mission_stage.launch.py stage:=<stage> \
#       launch_camera:=true camera:=0 awb_mode:=5 live_view:=true log_dir:=<dir>
#
# Stages (mission_manager/stages.py starts and stops them):
#   oak            OAK-D Lite driver + obstacle pipeline (sector_builder ->
#                  OBSTACLE_DISTANCE, coverage_guard) + OAK live view.
#                  Preflight -> landing (the obstacle sensor is required).
#   down_cam       Downward RPi Camera Module 1.3 (camera_ros) + its live view.
#                  Preflight (after the OAK is live) -> landing.
#   entry_corridor corridor_perception + green banner_node + ground_marker. Phase 2.
#   arena          red_zone_node. Phase 3 -> landing (the red camera watches
#                  every leg flown over the arena, homing included).
#   exit_corridor  corridor_perception + blue banner_blue_node + ground_marker. Phase 5.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory

STAGES = ("oak", "down_cam", "entry_corridor", "arena", "exit_corridor")


def _yaml_value(mission_params: str, name: str) -> str:
    import yaml
    with open(mission_params) as f:
        return str(yaml.safe_load(f)["mission_node"]["ros__parameters"][name])


def _setup(context, *args, **kwargs):
    stage = LaunchConfiguration("stage").perform(context)
    if stage not in STAGES:
        raise RuntimeError(f"stage:={stage} — one of {', '.join(STAGES)}")
    launch_camera = LaunchConfiguration("launch_camera").perform(context) == "true"
    live_view = LaunchConfiguration("live_view").perform(context) == "true"
    log_dir = os.path.expanduser(LaunchConfiguration("log_dir").perform(context))

    mission_params = os.path.join(
        get_package_share_directory("mission_bringup"), "config", "mission_params.yaml")
    avoidance_share = get_package_share_directory("obstacle_avoidance")
    avoidance_config = os.path.join(avoidance_share, "config", "avoidance.yaml")
    oak_remaps = [
        ("~/rgb", "/oak/rgb/image_raw"),
        ("~/rgb_camera_info", "/oak/rgb/camera_info"),
        ("~/depth", "/oak/stereo/image_raw"),
        ("~/camera_info", "/oak/stereo/camera_info"),
    ]

    def live_view_of(label, topic, port, downward):
        return Node(
            package="qr_detection",
            executable="live_view_node",
            name=f"live_view_{'down' if downward else 'oak'}",
            output="screen",
            parameters=[{"label": label, "topic": topic, "port": port,
                         "downward": downward, "record_dir": log_dir}],
        )

    def corridor_nodes(banner_name, banner_params):
        # corridor_perception on the OAK depth + colour (-> /corridor/perception),
        # the entrance/exit banner, and the downward ground marker: what
        # path_planner reads inside a corridor leg.
        return [
            Node(
                package="obstacle_avoidance",
                executable="corridor_perception_node",
                name="corridor_perception_node",
                output="screen",
                parameters=[avoidance_config],
                remappings=oak_remaps,
            ),
            Node(
                package="obstacle_avoidance",
                executable="banner_node",
                name=banner_name,
                output="screen",
                parameters=banner_params,
                remappings=oak_remaps,
            ),
            Node(
                package="qr_detection",
                executable="ground_marker_node",
                name="ground_marker_node",
                output="screen",
                parameters=[mission_params],
            ),
        ]

    if stage == "oak":
        # MUST use obstacle_avoidance's params file (see mission.launch.py history /
        # oak_d_lite_depth.yaml header: wrong intrinsics mis-bin every bearing).
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
            nodes.append(live_view_of("OAK (forward)", "/oak/rgb/image_raw", 8081, False))
        return nodes

    if stage == "down_cam":
        # camera_ros: an INTEGER is the camera's index, a STRING its name
        # ("0" as a string = "camera with name 0 does not exist")
        camera = LaunchConfiguration("camera").perform(context)
        camera = int(camera) if camera.isdigit() else camera
        nodes = []
        if launch_camera:
            # RPi Camera Module 1.3 (OV5647, fixed focus). camera_ros only
            # declares controls the sensor has, so AfMode / LensPosition (and
            # mission_node's _focus_camera) are ignored on it; they only act on a
            # Camera Module 3. camera_info is NOT remapped to /camera_info: an
            # uncalibrated camera_ros publishes K = 0, which would overwrite
            # qr_scanner_node's camera_fx.
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
                    "LensPosition": 1.0 / max(float(_yaml_value(mission_params,
                                                                "cruise_altitude_m")), 0.1),
                    "AwbMode": int(LaunchConfiguration("awb_mode").perform(context)),
                }],
                remappings=[("/camera/image_raw", "/down_frame")],
            ))
        if live_view:
            nodes.append(live_view_of("Cam (down)", "/down_frame", 8080, True))
        return nodes

    if stage == "entry_corridor":
        return corridor_nodes("banner_node", [avoidance_config])

    if stage == "exit_corridor":
        return corridor_nodes("banner_blue_node", [
            avoidance_config, os.path.join(avoidance_share, "config", "banner_blue.yaml")])

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
        DeclareLaunchArgument("launch_camera", default_value="true"),
        DeclareLaunchArgument("camera", default_value="0"),
        DeclareLaunchArgument("awb_mode", default_value="5"),
        DeclareLaunchArgument("live_view", default_value="true"),
        DeclareLaunchArgument("log_dir", default_value="~/aerothon_logs/stage"),
        OpaqueFunction(function=_setup),
    ])
