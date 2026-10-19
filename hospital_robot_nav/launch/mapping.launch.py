"""
mapping.launch.py  --  Phase 7.1: build the map of the floor with SLAM Toolbox.

Start the robot base FIRST (in its own terminal, on the Pi):
    ros2 launch hospital_robot_base base.launch.py use_head:=false
then this (Pi, or the Dell if the Pi is too busy - same ROS_DOMAIN_ID):
    ros2 launch hospital_robot_nav mapping.launch.py

Starts:
    scan_body_filter   /scan -> /scan_filtered (robot's own mast removed)
    slam_toolbox       online async mapping, publishes /map and map -> odom
Drive slowly with teleop (wheels on the floor, E-stop helper ready), then save:
    mkdir -p ~/robot_ws/maps/apartment
    ros2 run nav2_map_server map_saver_cli -f ~/robot_ws/maps/apartment/map
    (+ in RViz's SlamToolboxPlugin: "Serialize Map" to keep the pose graph)

NEVER run this together with navigation.launch.py: both publish map -> odom.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hospital_robot_nav')
    slam_share = get_package_share_directory('slam_toolbox')
    params = LaunchConfiguration('slam_params_file')

    return LaunchDescription([
        DeclareLaunchArgument('slam_params_file',
                              default_value=os.path.join(share, 'config', 'slam_mapping.yaml'),
                              description='SLAM Toolbox parameters'),
        Node(package='hospital_robot_nav', executable='scan_body_filter',
             name='scan_body_filter', output='screen'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(slam_share, 'launch', 'online_async_launch.py')),
            launch_arguments={'slam_params_file': params,
                              'use_sim_time': 'false'}.items()),
    ])
