import os
from glob import glob
from setuptools import setup

package_name = 'hospital_robot_hri'

setup(
    name=package_name,
    version='0.1.0',
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
    description='Human-robot interaction: animated eyes and gaze test tools',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hri_face = hospital_robot_hri.face_server_node:main',
            'simulate_gaze = hospital_robot_hri.simulate_gaze:main',
        ],
    },
)
