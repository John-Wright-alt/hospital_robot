"""speech.launch.py -- the robot's voice (hri_speech), optionally with the eyes.

    # A) next to gaze_test.launch.py (which already runs the eyes):
    ros2 launch hospital_robot_hri speech.launch.py

    # B) next to base.launch.py (real motors), with the eyes too:
    ros2 launch hospital_robot_hri speech.launch.py use_face:=true

    # C) no robot at all: SIMULATED robot inputs drive the speaker
    ros2 launch hospital_robot_hri speech.launch.py simulate:=true scenario:=drive
       scenarios: drive long estop fault lost battery noise

Starts:
  hri_speech       the speaker                                (always)
  hri_face         the eyes on http://<robot>:8080           (use_face)
  simulate_robot   test inputs on /hri/sim/* only            (simulate)

SAFETY: nothing here can arm or move the robot. hri_speech only listens and
talks; simulate_robot publishes only on /hri/sim/* (never /odom or /cmd_vel).
Do not use use_face:=true while gaze_test.launch.py runs (both would serve port 8080).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hospital_robot_hri')
    speech_params = os.path.join(share, 'config', 'speech.yaml')
    face_params = os.path.join(share, 'config', 'face.yaml')
    simulate = LaunchConfiguration('simulate')

    sim_inputs = {'odom_topic': 'hri/sim/odom',
                  'robot_state_topic': 'hri/sim/robot_state',
                  'pico_status_topic': 'hri/sim/pico_status',
                  'simulated': True}

    return LaunchDescription([
        DeclareLaunchArgument('use_face', default_value='false',
                              description='also start hri_face (not with gaze_test)'),
        DeclareLaunchArgument('simulate', default_value='false',
                              description='drive the speaker with simulate_robot test data'),
        DeclareLaunchArgument('scenario', default_value='drive',
                              description='simulate_robot scenario'),

        Node(package='hospital_robot_hri', executable='hri_speech', name='hri_speech',
             output='screen', parameters=[speech_params],
             condition=UnlessCondition(simulate)),

        Node(package='hospital_robot_hri', executable='hri_speech', name='hri_speech',
             output='screen', parameters=[speech_params, sim_inputs],
             condition=IfCondition(simulate)),

        Node(package='hospital_robot_hri', executable='simulate_robot', name='simulate_robot',
             output='screen', arguments=[LaunchConfiguration('scenario')],
             condition=IfCondition(simulate)),

        Node(package='hospital_robot_hri', executable='hri_face', name='hri_face',
             output='screen', parameters=[face_params],
             condition=IfCondition(LaunchConfiguration('use_face'))),

        LogInfo(msg='speech: hri_speech only listens and talks - it cannot move the robot.'),
    ])
