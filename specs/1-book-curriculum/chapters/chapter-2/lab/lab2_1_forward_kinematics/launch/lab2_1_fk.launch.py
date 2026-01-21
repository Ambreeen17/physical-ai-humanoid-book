#!/usr/bin/env python3
"""
Lab 2.1 Forward Kinematics Launch File

This launch file starts all necessary nodes for Lab 2.1:
  1. Robot State Publisher (publishes robot model to RViz)
  2. Joint State Publisher GUI (provides sliders to control joint angles)
  3. Forward Kinematics Calculator (computes and publishes end-effector pose)
  4. RViz2 (visualization)

Usage:
    ros2 launch lab2_1_forward_kinematics lab2_1_fk.launch.py

Author: Physical AI Lab
ROS 2 Version: Humble
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate launch description for Lab 2.1."""

    # Declare launch arguments
    use_gui = LaunchConfiguration('use_gui', default='true')
    use_rviz = LaunchConfiguration('use_rviz', default='true')

    # Get package paths
    pkg_share = get_package_share_directory('lab2_1_forward_kinematics')

    # Path to URDF file
    urdf_file = os.path.join(pkg_share, 'urdf', '3dof_arm.urdf')

    # Path to RViz config file
    rviz_config_file = os.path.join(pkg_share, 'config', 'rviz_config.rviz')

    # Read URDF file
    with open(urdf_file, 'r') as f:
        robot_description = f.read()

    # ========== Node 1: Robot State Publisher ==========
    # Publishes robot model to /robot_description topic
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }]
    )

    # ========== Node 2: Joint State Publisher GUI ==========
    # Provides sliders to manually control joint angles
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen',
        condition=lambda context: context.perform_substitution(use_gui) == 'true'
    )

    # ========== Node 3: Forward Kinematics Calculator ==========
    # Custom node that computes FK and publishes end-effector pose
    fk_calculator_node = Node(
        package='lab2_1_forward_kinematics',
        executable='fk_calculator',
        name='fk_calculator',
        output='screen',
        parameters=[{
            'link_lengths': [1.0, 0.8, 0.5],
            'publish_rate': 10.0
        }]
    )

    # ========== Node 4: RViz2 ==========
    # Visualization tool
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file] if os.path.exists(rviz_config_file) else [],
        condition=lambda context: context.perform_substitution(use_rviz) == 'true'
    )

    # ========== Launch Description ==========
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Launch Joint State Publisher GUI for manual control'
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz2 for visualization'
        ),

        # Launch nodes
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        fk_calculator_node,
        rviz_node,
    ])
