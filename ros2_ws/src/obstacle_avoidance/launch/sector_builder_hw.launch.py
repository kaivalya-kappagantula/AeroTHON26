# sector_builder_hw.launch.py — sector builder against the real OAK-D.
#
# Runs sector_builder_node on the depthai_ros_driver topics and publishes the
# static camera TF. It does NOT start the DepthAI driver — that is a separate,
# already-installed package (depthai_ros_driver v2.12.2 on Jazzy); launch it on
# its own so the two lifecycles stay independent. Depth arrives as 16UC1
# (millimetres) on hardware; the builder handles the units from the encoding.
#
# The driver MUST be started with our params file (depth aligned to the 720P
# colour camera, so camera_info matches the depth image) — see
# config/oak_d_lite_depth.yaml.
#
#   # terminal 1: the driver (its own launch, our params)
#   ros2 launch depthai_ros_driver camera.launch.py \
#     params_file:=$(ros2 pkg prefix obstacle_avoidance)/share/obstacle_avoidance/config/oak_d_lite_depth.yaml \
#     parent_frame:=base_link pointcloud.enable:=false rectify_rgb:=false use_gdb:=false
#   # terminal 2: our stack
#   ros2 launch obstacle_avoidance sector_builder_hw.launch.py
#   (corridor_width_min_m:=... corridor_width_max_m:=... override the yaml's
#    accepted corridor width range for this run only; the width itself is
#    measured, never configured)
#
# Sanity-check the result without RViz or a vehicle:
#   python3 scripts/bench_scan_monitor.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context, *args, **kwargs):
    pkg = get_package_share_directory('obstacle_avoidance')
    config = os.path.join(pkg, 'config', 'avoidance.yaml')
    path = LaunchConfiguration('obstacle_path').perform(context)
    mavlink_url = LaunchConfiguration('mavlink_url').perform(context)
    depth_topic = LaunchConfiguration('depth_topic').perform(context)
    info_topic = LaunchConfiguration('camera_info_topic').perform(context)
    rgb_topic = LaunchConfiguration('rgb_topic').perform(context)
    rgb_info_topic = LaunchConfiguration('rgb_camera_info_topic').perform(context)
    # Optional one-run overrides of avoidance.yaml (empty = use the yaml).
    corridor_overrides = {
        name: float(value) for name, value in (
            (n, LaunchConfiguration(n).perform(context))
            for n in ('corridor_width_min_m', 'corridor_width_max_m'))
        if value}

    nodes = []

    # /avoidance/scan is never remapped onto the MAVROS plugin (mirror bug); it
    # stays canonical for RViz/sector_viz/guard.
    builder_remaps = [
        ('~/depth', depth_topic),
        ('~/camera_info', info_topic),
    ]
    nodes.append(Node(
        package='obstacle_avoidance',
        executable='sector_builder_node',
        name='sector_builder_node',
        output='screen',
        parameters=[config],
        remappings=builder_remaps,
    ))

    # Our own publisher injects a correct OBSTACLE_DISTANCE. On hardware the
    # single UART means transport=mavros (/uas1/mavlink_sink); pymavlink opens a
    # 2nd connection (SITL bench only).
    transport = 'mavros' if path == 'sink' else 'pymavlink'
    nodes.append(Node(
        package='obstacle_avoidance',
        executable='obstacle_publisher_node',
        name='obstacle_publisher_node',
        output='screen',
        parameters=[config, {'transport': transport,
                             'mavlink_url': mavlink_url}],
    ))

    # Corridor fit on the same depth stream, plus the colour stream for the
    # wall bases (-> /corridor/perception, read by path_planner in the
    # corridors). Without colour frames it runs on depth alone.
    colour_remaps = [
        ('~/rgb', rgb_topic),
        ('~/rgb_camera_info', rgb_info_topic),
    ]
    # corridor_perception:=false: the staged mission starts it per corridor
    # (mission_bringup mission_stage.launch.py).
    if LaunchConfiguration('corridor_perception').perform(context) == 'true':
        nodes.append(Node(
            package='obstacle_avoidance',
            executable='corridor_perception_node',
            name='corridor_perception_node',
            output='screen',
            parameters=[config, corridor_overrides],
            remappings=builder_remaps + colour_remaps,
        ))

    # Coverage guard runs alongside the builder, consuming its scan + coverage.
    nodes.append(Node(
        package='obstacle_avoidance',
        executable='coverage_guard_node',
        name='coverage_guard_node',
        output='screen',
        parameters=[config],
    ))

    nodes.append(Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='oak_forward_tf',
        # Named flags: the old-style positional form (x y z yaw pitch roll
        # frame child) is deprecated and warns on Jazzy.
        arguments=['--x', '0.1', '--y', '0.0', '--z', '0.0',
                   '--yaw', '0', '--pitch', '0', '--roll', '0',
                   '--frame-id', 'base_link',
                   '--child-frame-id', 'oak_forward'],
    ))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('obstacle_path', default_value='sink',
                              choices=['sink', 'pymavlink']),
        DeclareLaunchArgument('mavlink_url',
                              default_value='udpout:127.0.0.1:14551'),
        # depthai_ros_driver default depth + info topics (400P stereo).
        DeclareLaunchArgument('depth_topic',
                              default_value='/oak/stereo/image_raw'),
        DeclareLaunchArgument('camera_info_topic',
                              default_value='/oak/stereo/camera_info'),
        DeclareLaunchArgument('rgb_topic', default_value='/oak/rgb/image_raw'),
        DeclareLaunchArgument('rgb_camera_info_topic',
                              default_value='/oak/rgb/camera_info'),
        DeclareLaunchArgument('corridor_perception', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('corridor_width_min_m', default_value='',
                              description='override avoidance.yaml (m); empty = yaml'),
        DeclareLaunchArgument('corridor_width_max_m', default_value='',
                              description='override avoidance.yaml (m); empty = yaml'),
        OpaqueFunction(function=_setup),
    ])
