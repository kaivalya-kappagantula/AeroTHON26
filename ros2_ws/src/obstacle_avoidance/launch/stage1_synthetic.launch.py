# stage1_synthetic.launch.py — Stage 1 (ArduPilot leg, fake data).
#
# Brings up synthetic_sector_node and gets its scan to ArduPilot. Which path
# carries the scan is a LAUNCH ARGUMENT, not a code branch:
#
#   obstacle_path:=sink       (default) our obstacle_publisher_node builds the
#                             OBSTACLE_DISTANCE and injects it via
#                             /uas1/mavlink_sink. We do NOT use the MAVROS
#                             obstacle_distance plugin: it mirrors left/right
#                             (copies ranges[] with NO CCW->CW reversal) and
#                             defaults frame=GLOBAL. /avoidance/scan is never
#                             remapped, so RViz/sector_viz/guard still see it.
#   obstacle_path:=pymavlink  same publisher over a 2nd MAVLink connection (SITL).
#
# Also publishes the static base_link -> oak_forward TF so RViz will render the
# scan.
#
#   ros2 launch obstacle_avoidance stage1_synthetic.launch.py
#   ros2 launch obstacle_avoidance stage1_synthetic.launch.py obstacle_path:=pymavlink

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

    nodes = []

    # /avoidance/scan stays canonical (RViz/sector_viz/guard observe it); it is
    # NEVER remapped onto /mavros/obstacle/send.
    nodes.append(Node(
        package='obstacle_avoidance',
        executable='synthetic_sector_node',
        name='synthetic_sector_node',
        output='screen',
        parameters=[config],
    ))

    # Our own publisher builds the correct OBSTACLE_DISTANCE (CCW->CW reversed,
    # frame=BODY_FRD) and injects it, bypassing the mirror-buggy MAVROS plugin.
    transport = 'mavros' if path == 'sink' else 'pymavlink'
    nodes.append(Node(
        package='obstacle_avoidance',
        executable='obstacle_publisher_node',
        name='obstacle_publisher_node',
        output='screen',
        parameters=[config, {'transport': transport,
                             'mavlink_url': mavlink_url}],
    ))

    # base_link -> oak_forward: forward-facing, level with the body x-axis.
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
        DeclareLaunchArgument(
            'obstacle_path', default_value='sink',
            choices=['sink', 'pymavlink'],
            description='Our OBSTACLE_DISTANCE publisher via /uas1/mavlink_sink '
                        '(sink, default) or a 2nd MAVLink connection (pymavlink, '
                        'SITL). The MAVROS plugin remap was removed (mirror bug).'),
        DeclareLaunchArgument(
            'mavlink_url', default_value='udpout:127.0.0.1:14551',
            description='Second MAVLink endpoint for the pymavlink fallback '
                        '(SITL only).'),
        OpaqueFunction(function=_setup),
    ])
