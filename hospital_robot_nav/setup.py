from glob import glob

from setuptools import find_packages, setup

package_name = 'hospital_robot_nav'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/behavior_trees', glob('behavior_trees/*.xml')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Marine',
    maintainer_email='marine@freroux.com',
    description='Phase 7 SLAM / Nav2 / mission layer for the hospital guide robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scan_body_filter = hospital_robot_nav.scan_body_filter:main',
            'mission = hospital_robot_nav.mission_node:main',
            'save_waypoint = hospital_robot_nav.save_waypoint:main',
            'go_to = hospital_robot_nav.go_to:main',
            'ultrasonic_check = hospital_robot_nav.ultrasonic_check:main',
            'simulate_mission = hospital_robot_nav.simulate_mission:main',
        ],
    },
)
