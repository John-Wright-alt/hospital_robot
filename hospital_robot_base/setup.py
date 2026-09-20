import os
from glob import glob
from setuptools import setup

package_name = 'hospital_robot_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='Real-robot bring-up for the hospital assistant robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pico_bridge = hospital_robot_base.pico_bridge:main',
            'mpu6050_node = hospital_robot_base.mpu6050_node:main',
            'odom_calibration = hospital_robot_base.odom_calibration:main',
            'ld06_check = hospital_robot_base.ld06_check:main',
            'fake_pico = hospital_robot_base.fake_pico:main',
        ],
    },
)
