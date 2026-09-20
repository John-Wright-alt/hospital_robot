"""base.launch.py -- the whole real robot base (guide Part 7).

    ros2 launch hospital_robot_base base.launch.py
    ros2 launch hospital_robot_base base.launch.py use_imu:=true
    ros2 launch hospital_robot_base base.launch.py use_imu:=true use_ekf:=true
    ros2 launch hospital_robot_base base.launch.py pico_port:=/tmp/pico use_lidar:=false
        ^ with the fake Pico (fake_pico running in another terminal), no LiDAR

Starts: robot_state_publisher (your URDF), joint_state_publisher (merges the
wheel angles), pico_bridge, the LD06 driver, and optionally the IMU and EKF.

The URDF defaults match your hospital_robot_description package. If your xacro
file has a different name, change the default marked 'CHECK' or pass
    urdf_file:=urdf/<your file>.xacro
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution, PythonExpression)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = get_package_share_directory('hospital_robot_base')
    base_params = os.path.join(share, 'config', 'base.yaml')
    ekf_params = os.path.join(share, 'config', 'ekf.yaml')

    description_package = LaunchConfiguration('description_package')
    urdf_file = LaunchConfiguration('urdf_file')
    use_lidar = LaunchConfiguration('use_lidar')
    use_imu = LaunchConfiguration('use_imu')
    use_ekf = LaunchConfiguration('use_ekf')
    pico_port = LaunchConfiguration('pico_port')

    robot_description = ParameterValue(
        Command(['xacro ', PathJoinSubstitution(
            [FindPackageShare(description_package), urdf_file])]),
        value_type=str)

    # The bridge publishes odom->base TF only when the EKF is NOT running.
    bridge_publishes_tf = ParameterValue(
        PythonExpression(["'", use_ekf, "'.lower() != 'true'"]), value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument('description_package', default_value='hospital_robot_description',
                              description='Package that holds your URDF/xacro'),
        DeclareLaunchArgument('urdf_file', default_value='urdf/robot.urdf.xacro',        # CHECK
                              description='Path of the xacro inside that package'),
        DeclareLaunchArgument('pico_port', default_value='/dev/pico',
                              description='/dev/pico for the real Pico, /tmp/pico for fake_pico'),
        DeclareLaunchArgument('use_lidar', default_value='true'),
        DeclareLaunchArgument('use_imu', default_value='false'),
        DeclareLaunchArgument('use_ekf', default_value='false'),

        Node(package='robot_state_publisher', executable='robot_state_publisher',
             output='screen',
             parameters=[{'robot_description': robot_description}]),

        Node(package='joint_state_publisher', executable='joint_state_publisher',
             output='screen', parameters=[base_params]),

        Node(package='hospital_robot_base', executable='pico_bridge',
             name='pico_bridge', output='screen',
             parameters=[base_params, {'publish_tf': bridge_publishes_tf,
                                       'port': pico_port}]),

        Node(package='ldlidar_stl_ros2', executable='ldlidar_stl_ros2_node',
             name='ldlidar', output='screen', parameters=[base_params],
             condition=IfCondition(use_lidar)),

        Node(package='hospital_robot_base', executable='mpu6050_node',
             name='mpu6050', output='screen', parameters=[base_params],
             condition=IfCondition(use_imu)),

        Node(package='robot_localization', executable='ekf_node',
             name='ekf_filter_node', output='screen', parameters=[ekf_params],
             condition=IfCondition(use_ekf)),
    ])
