import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """
    Launch file for Lab 1.
    Starts the simulation (turtlesim) and the student's control node.
    """
    return LaunchDescription([
        # 1. The Simulation (Sense/Act environment)
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim',
            output='screen'
        ),

        # 2. The Student's Node (The Sensorimotor Loop)
        Node(
            package='hello_physical_ai',
            executable='hello_physical_ai_node.py',
            name='control_node',
            parameters=[{'boundary_x': 2.0}],
            output='screen'
        )
    ])
