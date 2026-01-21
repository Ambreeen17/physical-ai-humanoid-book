from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab_15_3',
            executable='lab_15_3_node',
            name='lab_15_3_node',
            output='screen',
            parameters=[
                # Add parameters specific to Deployment & Operations
            ]
        )
    ])
