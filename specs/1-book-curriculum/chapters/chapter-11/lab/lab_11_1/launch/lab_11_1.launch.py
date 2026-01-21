from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab_11_1',
            executable='lab_11_1_node',
            name='lab_11_1_node',
            output='screen',
            parameters=[
                # Add parameters specific to Imitation Learning
            ]
        )
    ])
