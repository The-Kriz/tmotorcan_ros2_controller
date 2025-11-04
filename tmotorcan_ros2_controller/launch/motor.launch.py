from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('tmotorcan_ros2_controller')
    params_file = os.path.join(pkg_share, 'config', 'motors.yaml')

    return LaunchDescription([
        Node(
            package='tmotorcan_ros2_controller',
            executable='motor_node',
            name='tmotorcan_node',
            output='screen',
            parameters=[params_file]
        )
    ])