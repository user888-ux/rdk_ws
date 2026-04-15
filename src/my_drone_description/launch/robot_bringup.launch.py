from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    package_name = 'dh_robot_pkg'
    package_dir = get_package_share_directory(package_name)
    
    return LaunchDescription([
        Node(
            package=package_name,
            executable='dh_robot_node',
            name='dh_robot_controller',
            output='screen',
            parameters=[os.path.join(package_dir, 'config', 'dh_parameters.yaml')]
        )
    ])