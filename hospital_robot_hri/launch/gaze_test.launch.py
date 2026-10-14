"""gaze_test.launch.py -- Phase 9 test: LiDAR -> gaze -> eyes, robot model for RViz.

    ros2 launch hospital_robot_hri gaze_test.launch.py
    ros2 launch hospital_robot_hri gaze_test.launch.py use_lidar:=false use_gaze:=false
        ^ eyes only (then: ros2 run hospital_robot_hri simulate_gaze sweep)

Starts:
  robot_state_publisher   your URDF -> /robot_description + the fixed TF frames
  joint_state_publisher   wheel and column joints held at 0, so EVERY link has a TF
                          frame and the whole model can be drawn in RViz
  ldlidar                 the LD06 driver -> /scan        (use_lidar)
  lidar_gaze              /scan -> /hri/gaze/status       (use_gaze)
  hri_face                the eyes on http://<robot>:8080 (use_face)

SAFETY: this launch file does NOT start pico_bridge. Nothing here can arm the
robot or move a wheel or the lift. Do not run it together with base.launch.py:
both start robot_state_publisher and the LiDAR driver.

Because pico_bridge is not running there is no /odom and no odom frame.
In RViz use Fixed Frame = base_footprint.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    base_share = get_package_share_directory('hospital_robot_base')
    hri_share = get_package_share_directory('hospital_robot_hri')
    base_params = os.path.join(base_share, 'config', 'base.yaml')
    gaze_params = os.path.join(base_share, 'config', 'gaze.yaml')
    face_params = os.path.join(hri_share, 'config', 'face.yaml')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution(
            [FindPackageShare('hospital_robot_description'), 'urdf', 'robot.urdf.xacro'])]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_gaze', default_value='true'),
        DeclareLaunchArgument('use_face', default_value='true'),

        LogInfo(msg='gaze_test: NO pico_bridge - motors and lift cannot be armed. '
                    'RViz Fixed Frame: base_footprint'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen', parameters=[{'robot_description': robot_description}]),

        Node(package='joint_state_publisher', executable='joint_state_publisher',
             output='screen', parameters=[{'rate': 10}]),

        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
             name='ldlidar', output='screen', parameters=[base_params],
             condition=IfCondition(LaunchConfiguration('use_lidar'))),

        Node(package='hospital_robot_base', executable='lidar_gaze',
             name='lidar_gaze', output='screen', parameters=[gaze_params],
             condition=IfCondition(LaunchConfiguration('use_gaze'))),

        Node(package='hospital_robot_hri', executable='hri_face',
             name='hri_face', output='screen', parameters=[face_params],
             condition=IfCondition(LaunchConfiguration('use_face'))),
    ])
