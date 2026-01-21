from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab_14_1',
            executable='lab_14_1_node',
            name='lab_14_1_node',
            output='screen',
            parameters=[
                # Add parameters specific to Safety & Robustness
            ]
        )
    ])
