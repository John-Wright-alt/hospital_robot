import os
from glob import glob
from setuptools import setup

package_name = 'hospital_robot_hri'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Human-robot interaction: Anna - eyes, speech, wake-up flow and test tools',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hri_face = hospital_robot_hri.face_server_node:main',
            'simulate_gaze = hospital_robot_hri.simulate_gaze:main',
            # --- Phase 9 speech ---
            'hri_speech = hospital_robot_hri.speech_node:main',
            'simulate_robot = hospital_robot_hri.simulate_robot:main',
            'audio_tool = hospital_robot_hri.audio_tool:main',
            # --- Anna's wake-up flow ---
            'hri_interaction = hospital_robot_hri.interaction_node:main',
            'simulate_head = hospital_robot_hri.simulate_head:main',
        ],
    },
)
