"""
navigation.launch.py  --  Phase 7.2+: Nav2 on the saved map + the mission node.

Start the robot base FIRST (Pi), then on the Pi:
    ros2 launch hospital_robot_nav navigation.launch.py site:=apartment
    ros2 launch hospital_robot_nav navigation.launch.py site:=hospital

A "site" is one place you mapped. Each site has its own folder:
    ~/robot_ws/maps/<site>/map.yaml + map.pgm     (saved in guide Stage 5)
    ~/robot_ws/maps/<site>/waypoints.yaml         (saved in guide Stage 6)
map:= and waypoints_file:= still work and override the site folder.
    ros2 launch hospital_robot_nav navigation.launch.py use_ultrasonic:=false   # LiDAR only
    ros2 launch hospital_robot_nav navigation.launch.py head_optional:=true     # base without use_head

Starts:
    scan_body_filter   /scan -> /scan_filtered
    nav2_bringup       map_server + AMCL + the Nav2 stack (bringup_launch.py, slam:=False)
    mission            named destinations from the waypoint file (start_mission:=true)

What this file does before starting Nav2 (so you never edit these by hand):
  * AMCL's initial pose  = the 'home' waypoint (robot must START at home,
    facing the saved direction - otherwise use RViz "2D Pose Estimate")
  * bt_navigator's tree  = behavior_trees/navigate_no_backup.xml
  * use_ultrasonic:=false removes the range layer + the 3 range sources
The finished parameter file is written to /tmp/hospital_nav2_params.yaml so
you can read exactly what Nav2 got.

Nothing here moves the robot: Nav2 only drives after a goal is sent AND the
robot is ARMED by a person at the Pico (the mission node never arms).
"""
import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RUNTIME_PARAMS = '/tmp/hospital_nav2_params.yaml'


def _home_pose(wp_file):
    """(x, y, yaw_rad) of 'home', or None. Same file format as waypoints.py."""
    try:
        with open(os.path.expanduser(wp_file)) as f:
            h = (yaml.safe_load(f) or {}).get('waypoints', {}).get('home')
        return float(h['x']), float(h['y']), math.radians(float(h.get('yaw_deg', 0.0)))
    except Exception:                                   # noqa: BLE001
        return None


def build_params(src, bt_xml, home, use_ultrasonic):
    with open(src) as f:
        p = yaml.safe_load(f)
    amcl = p['amcl']['ros__parameters']
    if home is not None:
        amcl['set_initial_pose'] = True
        amcl['initial_pose'] = {'x': home[0], 'y': home[1], 'z': 0.0, 'yaw': home[2]}
    else:
        amcl['set_initial_pose'] = False               # set it in RViz instead
    p['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = bt_xml
    if not use_ultrasonic:
        lc = p['local_costmap']['local_costmap']['ros__parameters']
        lc['plugins'] = [x for x in lc['plugins'] if x != 'range_layer']
        cm = p['collision_monitor']['ros__parameters']
        cm['observation_sources'] = [s for s in cm['observation_sources']
                                     if not s.startswith('us_')]
    return p


def _setup(context):
    share = get_package_share_directory('hospital_robot_nav')
    get = lambda n: LaunchConfiguration(n).perform(context)   # noqa: E731
    site = get('site').strip() or 'apartment'
    site_dir = os.path.expanduser(os.path.join('~/robot_ws/maps', site))
    wp_file = os.path.expanduser(get('waypoints_file') or os.path.join(site_dir, 'waypoints.yaml'))
    map_yaml = os.path.expanduser(get('map') or os.path.join(site_dir, 'map.yaml'))
    use_us = get('use_ultrasonic').lower() in ('true', '1', 'yes')
    home = _home_pose(wp_file)
    params = build_params(os.path.expanduser(get('params_file')),
                          os.path.join(share, 'behavior_trees', 'navigate_no_backup.xml'),
                          home, use_us)
    params.setdefault('map_server', {}).setdefault('ros__parameters', {})['yaml_filename'] = map_yaml
    with open(RUNTIME_PARAMS, 'w') as f:
        yaml.safe_dump(params, f, sort_keys=False)

    msgs = [LogInfo(msg=f'[navigation] SITE: {site}'),
            LogInfo(msg=f'[navigation] map: {map_yaml}'),
            LogInfo(msg=f'[navigation] waypoints: {wp_file}'),
            LogInfo(msg=f'[navigation] ultrasonics in costmap/collision monitor: {use_us}'),
            LogInfo(msg=(f'[navigation] AMCL initial pose = home x={home[0]:.2f} y={home[1]:.2f} '
                         f'yaw={math.degrees(home[2]):.0f} deg') if home else
                    f'[navigation] no "home" in {wp_file}: set the pose with RViz 2D Pose Estimate'),
            LogInfo(msg=f'[navigation] parameters written to {RUNTIME_PARAMS}')]
    if not os.path.isfile(map_yaml):
        msgs.append(LogInfo(msg=f'[navigation] WARNING map file not found: {map_yaml}'))

    actions = msgs + [
        Node(package='hospital_robot_nav', executable='scan_body_filter',
             name='scan_body_filter', output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
            launch_arguments={'map': map_yaml, 'params_file': RUNTIME_PARAMS,
                              'use_sim_time': 'false', 'slam': 'False',
                              'autostart': get('autostart'),
                              'use_composition': get('use_composition')}.items()),
    ]
    if get('start_mission').lower() in ('true', '1', 'yes'):
        actions.append(Node(package='hospital_robot_nav', executable='mission', name='mission',
                            output='screen',
                            parameters=[os.path.join(share, 'config', 'mission.yaml'),
                                        {'waypoints_file': wp_file,
                                         'head_optional': get('head_optional').lower()
                                         in ('true', '1', 'yes')}]))
    return actions


def generate_launch_description():
    share = get_package_share_directory('hospital_robot_nav')
    return LaunchDescription([
        DeclareLaunchArgument('site', default_value='apartment',
                              description='folder in ~/robot_ws/maps with map.yaml + waypoints.yaml'),
        DeclareLaunchArgument('map', default_value='',
                              description='override: full path of a map .yaml'),
        DeclareLaunchArgument('waypoints_file', default_value='',
                              description='override: full path of a waypoints .yaml'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(share, 'config', 'nav2_params_real.yaml')),
        DeclareLaunchArgument('use_ultrasonic', default_value='true'),
        DeclareLaunchArgument('start_mission', default_value='true'),
        DeclareLaunchArgument('head_optional', default_value='false',
                              description='true = mission may drive when NO head_column node '
                                          'runs at all (base started with use_head:=false). '
                                          'If head_column runs, PARKED is still required.'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('use_composition', default_value='True',
                              description='True = all Nav2 servers in one process (less RAM on the Pi)'),
        OpaqueFunction(function=_setup),
    ])
