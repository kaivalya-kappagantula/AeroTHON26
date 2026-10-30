# oa_eval.launch.py — ONE corridor pass in SITL, BendyRuler or path_planner,
# scored by oa_eval_node (docs/OA_EVALUATION.md). Normally started by
# obstacle_avoidance/scripts/oa_eval.sh, which also starts SITL with the
# matching FC parameter set and repeats runs; by hand:
#
#   # SITL started with params/bendyruler_eval.param (bendy) or
#   # params/stage5_6_corridor.param (current), then:
#   ros2 launch mission_bringup oa_eval.launch.py planner:=bendy scenario:=centred
#
# World: a single corridor (corridor 2 of sitl_corridor_sim) straight ahead
# (North) of the SITL home, entered 2 m in front of it — corridor_width_m x
# corridor_length_m (competition: 11.5 x 22 ft = 3.5052 x 6.7056 m) with the
# scenario's boxes (oa_eval.scenario_obstacles). Corridor 1 is moved 200 m away.
#
# Depth path, the same for both planners:
#   sitl_corridor_sim -> sim_depth_dropout (dropout:=) -> /oak/stereo/image_raw
#     -> sector_builder -> /avoidance/scan -> obstacle_publisher
#          -> /uas1/mavlink_sink -> OBSTACLE_DISTANCE -> ArduPilot (BendyRuler)
#     -> corridor_perception -> /corridor/perception -> path_planner (current)
# coverage_guard runs too (path_planner's sensor gates).

import json
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction,
                            SetEnvironmentVariable, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from obstacle_avoidance.oa_eval import scenario_obstacles

ENTRY_Y = 2.0          # first corridor waypoint, m North of home


def _stack(context, *args, **kwargs):
    arg = lambda n: LaunchConfiguration(n).perform(context)     # noqa: E731
    mission_params = os.path.join(
        get_package_share_directory('mission_bringup'), 'config', 'mission_params.yaml')
    avoidance_cfg = os.path.join(
        get_package_share_directory('obstacle_avoidance'), 'config', 'avoidance.yaml')
    with open(mission_params) as f:
        sim_cfg = yaml.safe_load(f)['sitl_corridor_sim']['ros__parameters']

    planner, scenario = arg('planner'), arg('scenario')
    width, length = float(arg('corridor_width_m')), float(arg('corridor_length_m'))
    alt, speed = float(arg('altitude_m')), float(arg('speed_ms'))
    mouth, exit_off = float(sim_cfg['mouth_offset_m']), float(sim_cfg['exit_offset_m'])
    obstacles = scenario_obstacles(scenario, length, float(arg('obstacle_width_m')),
                                   float(arg('obstacle_top_m')), float(arg('spacing_m')),
                                   float(arg('stagger_m')))
    wp_x = [0.0, 0.0]
    wp_y = [ENTRY_Y, ENTRY_Y + mouth + length + exit_off]
    depth_m = float(arg('obstacle_depth_m'))
    gps_offset = float(arg('gps_offset_m'))

    world = {
        'corridor2_waypoints_x': wp_x, 'corridor2_waypoints_y': wp_y,
        'corridor2_width_m': width, 'corridor2_length_m': length,
        'corridor2_obstacles': obstacles,
        'corridor1_waypoints_x': [200.0, 216.0], 'corridor1_waypoints_y': [200.0, 200.0],
        'corridor1_obstacles': [0.0],
    }
    tags = json.loads(arg('tags') or '{}')
    wall_max, drift = float(arg('wall_depth_max_m')), float(arg('gps_drift_m'))
    tags.update(scenario=scenario, spacing_m=float(arg('spacing_m')),
                dropout=float(arg('dropout')), speed_ms=speed, altitude_m=alt,
                obstacles=obstacles, wall_depth_max_m=wall_max, gps_drift_m=drift)

    clean_depth = [('/oak/stereo/image_raw', '/oak/stereo/image_sim')]
    remaps = [('~/depth', '/oak/stereo/image_raw'),
              ('~/camera_info', '/oak/stereo/camera_info')]
    # 5 Hz sim depth: the stale cutoffs sitl_mission.launch.py uses
    timeouts = {'depth_stale_timeout_s': 0.6, 'depth_watchdog_timeout_s': 0.8}
    nodes = [
        Node(package='obstacle_avoidance', executable='sitl_corridor_sim',
             name='sitl_corridor_sim', output='screen',
             parameters=[mission_params, {**world, 'obstacle_depth_m': depth_m,
                                          'gps_offset_m': gps_offset, 'gps_drift_m': drift,
                                          'wall_depth_max_m': wall_max}],
             remappings=clean_depth),
        Node(package='obstacle_avoidance', executable='sim_depth_dropout',
             name='sim_depth_dropout', output='screen',
             parameters=[{'dropout_fraction': float(arg('dropout'))}],
             remappings=[('~/in', '/oak/stereo/image_sim'),
                         ('~/out', '/oak/stereo/image_raw')]),
        Node(package='obstacle_avoidance', executable='sector_builder_node',
             name='sector_builder_node', output='screen',
             parameters=[avoidance_cfg, timeouts], remappings=remaps),
        Node(package='obstacle_avoidance', executable='obstacle_publisher_node',
             name='obstacle_publisher_node', output='screen',
             parameters=[avoidance_cfg, {'transport': 'mavros'}]),
        Node(package='obstacle_avoidance', executable='coverage_guard_node',
             name='coverage_guard_node', output='screen',
             parameters=[avoidance_cfg, timeouts]),
        Node(package='tf2_ros', executable='static_transform_publisher', name='oak_forward_tf',
             arguments=['--x', '0.1', '--frame-id', 'base_link',
                        '--child-frame-id', 'oak_forward']),
    ]
    if planner == 'current':
        nodes += [
            Node(package='obstacle_avoidance', executable='corridor_perception_node',
                 name='corridor_perception_node', output='screen',
                 parameters=[avoidance_cfg, {'corridor_stride_px': 2}], remappings=remaps),
            Node(package='path_planner', executable='path_planner_node',
                 name='path_planner_node', output='screen',
                 parameters=[mission_params, {
                     'corridor2_waypoints_x': wp_x, 'corridor2_waypoints_y': wp_y,
                     'corridor_altitude_m': alt, 'corridor_follow_speed_ms': speed,
                     'require_corridor_perception': True,
                     'require_avoidance_sensor': True}]),
        ]
    nodes.append(TimerAction(period=8.0, actions=[Node(
        package='obstacle_avoidance', executable='oa_eval_node', name='oa_eval_node',
        output='screen',
        parameters=[{
            'planner': planner,
            'origin_lat': float(sim_cfg['origin_lat']),
            'origin_lon': float(sim_cfg['origin_lon']),
            'corridor_waypoints_x': wp_x, 'corridor_waypoints_y': wp_y,
            'corridor_width_m': width, 'corridor_length_m': length,
            'corridor_wall_height_m': float(sim_cfg['corridor2_wall_height_m']),
            'mouth_offset_m': mouth, 'obstacles': obstacles,
            'obstacle_depth_m': depth_m, 'gps_offset_m': gps_offset,
            'altitude_m': alt, 'drone_width_m': float(arg('drone_width_m')),
            'require_rejoin': scenario == 'centred',
            'timeout_s': float(arg('timeout_s')),
            'result_file': arg('result_file'), 'trace_file': arg('trace_file'),
            'tags': json.dumps(tags)}])]))
    return nodes


def generate_launch_description():
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'apm.launch'])]),
        launch_arguments={'fcu_url': LaunchConfiguration('fcu_url')}.items(),
        condition=IfCondition(LaunchConfiguration('launch_mavros')))
    a = DeclareLaunchArgument
    return LaunchDescription([
        a('planner', default_value='bendy', choices=['bendy', 'current']),
        a('scenario', default_value='centred', choices=['centred', 'staggered', 'empty']),
        a('corridor_width_m', default_value='3.5052'),
        a('corridor_length_m', default_value='6.7056'),
        a('obstacle_width_m', default_value='1.2'),
        a('obstacle_top_m', default_value='1.8288',
          description='obstacle height (m); above altitude_m so it must be flown around'),
        a('obstacle_depth_m', default_value='0.4'),
        a('spacing_m', default_value='2.5', description='staggered: front-face spacing (m)'),
        a('stagger_m', default_value='0.75',
          description='staggered: box centres this far left / right of the centreline (m)'),
        a('dropout', default_value='0.0', description='fraction of depth pixels dropped'),
        a('altitude_m', default_value='1.2'),
        a('speed_ms', default_value='0.5',
          description='path_planner corridor speed; BendyRuler uses WP_SPD from its param file'),
        a('drone_width_m', default_value='0.6294', description='tip to tip incl. props'),
        a('gps_offset_m', default_value='0.0'),
        a('gps_drift_m', default_value='0.0',
          description='GPS error swinging +-this sideways (20 s period); the judge '
                      'scores truth only with 0'),
        a('wall_depth_max_m', default_value='0.0',
          description='walls give no depth beyond this (m), as the OAK-D on the real '
                      '11.5 ft corridor: 3.5 ~ the bench; 0 = perfect walls'),
        a('timeout_s', default_value='120.0'),
        a('result_file', default_value='/tmp/oa_eval.jsonl'),
        a('trace_file', default_value=''),
        a('tags', default_value='{}', description='extra JSON tags for the result line'),
        a('launch_mavros', default_value='true'),
        a('fcu_url', default_value='tcp://127.0.0.1:5760'),
        mavros_launch,
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            PathJoinSubstitution([FindPackageShare('mission_bringup'), 'config',
                                  'fastdds_large_frames.xml'])),
        OpaqueFunction(function=_stack),
    ])
