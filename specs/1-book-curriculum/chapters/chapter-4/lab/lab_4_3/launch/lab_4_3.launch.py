from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab_4_3',
            executable='lab_4_3_node',
            name='lab_4_3_node',
            output='screen',
            parameters=[
                # Add parameters specific to State Estimation
            ]
        )
    ])
