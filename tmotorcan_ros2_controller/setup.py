from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tmotorcan_ros2_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kriz',
    maintainer_email='kriz@todo.todo',
    description='ROS2 wrapper for TMotor CAN MIT-mode controller',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = tmotorcan_ros2_controller.motor_node:main',
        ],
    },
)
