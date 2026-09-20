"""lidar.launch.py -- the LD06 on its own (guide Part 4).

    ros2 launch hospital_robot_base lidar.launch.py

In RViz on the PC set Fixed Frame = lidar_link to see the scan without a URDF.
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('hospital_robot_base'),
                          'config', 'base.yaml')
    return LaunchDescription([
        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
             name='ldlidar', output='screen', parameters=[params]),
    ])
