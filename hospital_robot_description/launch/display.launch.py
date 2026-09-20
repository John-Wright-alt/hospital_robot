from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('hospital_robot_description')

    default_model = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    default_rviz = PathJoinSubstitution([pkg_share, 'rviz', 'display.rviz'])

    model_arg = DeclareLaunchArgument(
        'model', default_value=default_model,
        description='Absolute path to the robot xacro file')

    rviz_arg = DeclareLaunchArgument(
        'rvizconfig', default_value=default_rviz,
        description='Absolute path to the RViz config file')

    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}])

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui')

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')])

    return LaunchDescription([
        model_arg,
        rviz_arg,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz,
    ])
