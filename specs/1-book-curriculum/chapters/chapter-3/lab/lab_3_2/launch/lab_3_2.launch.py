from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lab_3_2',
            executable='lab_3_2_node',
            name='lab_3_2_node',
            output='screen',
            parameters=[
                # Add parameters specific to Sensors & Actuators
            ]
        )
    ])
