"""hri.launch.py -- Anna: eyes + voice + wake-up flow (+ optional LiDAR gaze).

    # A) The real robot. Pi-1 runs the base WITH the head column (Phase 8):
    #      ros2 launch hospital_robot_base base.launch.py use_head:=true
    #    Pi-2:
    ros2 launch hospital_robot_hri hri.launch.py use_gaze:=true

    # B) No lift at all: a SIMULATED head column (nothing can move)
    ros2 launch hospital_robot_hri hri.launch.py sim_head:=true head_scenario:=normal
       head scenarios: normal noface notarmed

    # Testing from the PC's browser (tap with the mouse) instead of the robot's screen:
    ros2 launch hospital_robot_hri hri.launch.py sim_head:=true remote_touch:=true

Starts:
  hri_face          the eyes + menu on http://<robot>:8080      (use_face)
  hri_speech        Anna's voice                                 (use_speech)
  hri_interaction   sleep -> tap -> intro -> adjust -> menu      (always)
  lidar_gaze        LiDAR "curiosity" for the eyes               (use_gaze)
  simulate_head     test head column on /hri/sim/* only          (sim_head)

Do not use use_face:=true / use_gaze:=true while gaze_test.launch.py runs (it
already starts both). SAFETY: nothing here arms the robot. The real column only
moves through head_column, only when the robot is armed, and only after a
person has tapped the screen.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('hospital_robot_hri')
    face_params = os.path.join(share, 'config', 'face.yaml')
    speech_params = os.path.join(share, 'config', 'speech.yaml')
    inter_params = os.path.join(share, 'config', 'interaction.yaml')
    sim_head = LaunchConfiguration('sim_head')
    remote_touch = ParameterValue(LaunchConfiguration('remote_touch'), value_type=bool)
    # Phase 7: nav:=true -> Anna drives with the mission node (hospital_robot_nav)
    nav_param = {'nav_available': ParameterValue(LaunchConfiguration('nav'),
                                                 value_type=bool)}

    sim_topics = {'head_state_topic': 'hri/sim/head_state',
                  'head_command_topic': 'hri/sim/head_command',
                  'lift_state_topic': 'hri/sim/lift_state'}

    nodes = [
        DeclareLaunchArgument('use_face', default_value='true'),
        DeclareLaunchArgument('use_speech', default_value='true'),
        DeclareLaunchArgument('use_gaze', default_value='false',
                              description='start lidar_gaze (needs the LiDAR running)'),
        DeclareLaunchArgument('sim_head', default_value='false',
                              description='use simulate_head instead of the real column'),
        DeclareLaunchArgument('head_scenario', default_value='normal'),
        DeclareLaunchArgument('nav', default_value='false',
                              description='true = guide with Nav2 '
                                          '(navigation.launch.py must run)'),
        DeclareLaunchArgument('remote_touch', default_value='false',
                              description='accept taps from other computers (testing only)'),

        Node(package='hospital_robot_hri', executable='hri_face', name='hri_face',
             output='screen', parameters=[face_params, {'allow_remote_touch': remote_touch}],
             condition=IfCondition(LaunchConfiguration('use_face'))),

        Node(package='hospital_robot_hri', executable='hri_speech', name='hri_speech',
             output='screen', parameters=[speech_params],
             condition=IfCondition(LaunchConfiguration('use_speech'))),

        Node(package='hospital_robot_hri', executable='hri_interaction', name='hri_interaction',
             output='screen', parameters=[inter_params, nav_param],
             condition=UnlessCondition(sim_head)),

        Node(package='hospital_robot_hri', executable='hri_interaction', name='hri_interaction',
             output='screen', parameters=[inter_params, sim_topics, nav_param],
             condition=IfCondition(sim_head)),

        Node(package='hospital_robot_hri', executable='simulate_head', name='simulate_head',
             output='screen', arguments=[LaunchConfiguration('head_scenario')],
             condition=IfCondition(sim_head)),

        LogInfo(msg='hri: Anna is asleep. Tap the screen to wake her.'),
    ]

    # lidar_gaze lives in hospital_robot_base (Phase 9 gaze guide)
    try:
        gaze_params = os.path.join(get_package_share_directory('hospital_robot_base'),
                                   'config', 'gaze.yaml')
        nodes.append(Node(package='hospital_robot_base', executable='lidar_gaze',
                          name='lidar_gaze', output='screen', parameters=[gaze_params],
                          condition=IfCondition(LaunchConfiguration('use_gaze'))))
    except Exception:                                  # package not built on this machine
        pass
    return LaunchDescription(nodes)
