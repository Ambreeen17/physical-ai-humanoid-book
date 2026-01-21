#!/usr/bin/env python3
"""
Lab 2.2 Inverse Kinematics Launch File

Launches IK solver node for testing and demonstration.

Usage:
    ros2 launch lab2_2_inverse_kinematics lab2_2_ik.launch.py

Author: Physical AI Lab
ROS 2 Version: Humble
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for Lab 2.2."""

    # Declare launch arguments
    ik_method = LaunchConfiguration('ik_method', default='analytical')
    max_iterations = LaunchConfiguration('max_iterations', default='100')

    # IK Solver Node
    ik_solver_node = Node(
        package='lab2_2_inverse_kinematics',
        executable='ik_solver',
        name='ik_solver',
        output='screen',
        parameters=[{
            'link_lengths': [1.0, 0.8, 0.5],
            'ik_method': ik_method,
            'max_iterations': max_iterations,
            'epsilon': 0.001,
            'alpha': 0.5
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'ik_method',
            default_value='analytical',
            description='IK method: analytical or numerical'
        ),
        DeclareLaunchArgument(
            'max_iterations',
            default_value='100',
            description='Maximum iterations for numerical IK'
        ),
        ik_solver_node
    ])
