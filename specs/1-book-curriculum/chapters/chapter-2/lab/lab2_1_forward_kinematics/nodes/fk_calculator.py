#!/usr/bin/env python3
"""
Forward Kinematics Calculator Node

This ROS 2 node computes forward kinematics for a 3-DOF robot arm using
Denavit-Hartenberg (DH) parameters. It subscribes to joint states and
publishes the end-effector pose.

Author: Physical AI Lab
License: MIT
ROS 2 Version: Humble
"""

import numpy as np
from typing import Tuple, List

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Quaternion
from tf_transformations import quaternion_from_matrix


class ForwardKinematicsCalculator(Node):
    """
    Computes forward kinematics for a 3-DOF planar arm.

    Subscribed Topics:
        /joint_states (sensor_msgs/JointState): Joint angles for the robot arm

    Published Topics:
        /end_effector_pose (geometry_msgs/PoseStamped): End-effector pose

    Parameters:
        - link_lengths (list): Lengths of each link [a1, a2, a3] in meters
        - publish_rate (float): Publishing frequency in Hz
    """

    def __init__(self):
        super().__init__('fk_calculator')

        # Declare and get parameters
        self.declare_parameter('link_lengths', [1.0, 0.8, 0.5])
        self.declare_parameter('publish_rate', 10.0)

        self.link_lengths = self.get_parameter('link_lengths').value
        self.publish_rate = self.get_parameter('publish_rate').value

        # Validate parameters
        if len(self.link_lengths) != 3:
            self.get_logger().error('Expected 3 link lengths, got {}'.format(len(self.link_lengths)))
            raise ValueError('Invalid link_lengths parameter')

        # DH parameter table (constant parameters)
        # Format: [d, alpha] for each joint (theta and a are variable/from link_lengths)
        self.dh_table = [
            {'d': 0.0, 'a': self.link_lengths[0], 'alpha': 0.0},
            {'d': 0.0, 'a': self.link_lengths[1], 'alpha': 0.0},
            {'d': 0.0, 'a': self.link_lengths[2], 'alpha': 0.0}
        ]

        # ROS 2 publisher and subscriber
        self.pose_publisher = self.create_publisher(
            PoseStamped,
            '/end_effector_pose',
            10
        )

        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Store current joint angles
        self.current_joint_angles = None

        self.get_logger().info('Forward Kinematics Calculator Node Started')
        self.get_logger().info(f'DH Parameters:')
        for i, params in enumerate(self.dh_table, 1):
            self.get_logger().info(
                f"  Joint {i}: θ=q{i}, d={params['d']}, a={params['a']}, α={params['alpha']}"
            )
        self.get_logger().info('Waiting for joint states on /joint_states...')

    def compute_dh_transform(self, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """
        Compute the homogeneous transformation matrix using DH parameters.

        The transformation represents the pose of frame i with respect to frame i-1.

        DH Convention:
            T_i = Rot_z(θ) × Trans_z(d) × Trans_x(a) × Rot_x(α)

        Args:
            theta: Joint angle (rotation about z-axis) [radians]
            d: Link offset (translation along z-axis) [meters]
            a: Link length (translation along x-axis) [meters]
            alpha: Link twist (rotation about x-axis) [radians]

        Returns:
            4x4 homogeneous transformation matrix

        Mathematical Form:
            ┌ cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)  a·cos(θ) ┐
            │ sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)  a·sin(θ) │
            │   0         sin(α)         cos(α)         d     │
            └   0           0               0           1     ┘
        """
        # Precompute trigonometric values
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)

        # Construct 4x4 transformation matrix
        T = np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1]
        ], dtype=np.float64)

        return T

    def compute_forward_kinematics(self, joint_angles: List[float]) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics for the 3-DOF arm.

        This function computes the end-effector pose by multiplying individual
        DH transformation matrices: T_end = T_1 × T_2 × T_3

        Args:
            joint_angles: List of 3 joint angles [q1, q2, q3] in radians

        Returns:
            Tuple of (position, rotation_matrix):
                - position: 3D position [x, y, z] in meters
                - rotation_matrix: 3x3 rotation matrix

        Raises:
            ValueError: If joint_angles length is not 3
        """
        if len(joint_angles) != 3:
            raise ValueError(f'Expected 3 joint angles, got {len(joint_angles)}')

        # Initialize transformation as identity
        T_end = np.eye(4, dtype=np.float64)

        # Multiply transformation matrices for each joint
        for i, (q, dh) in enumerate(zip(joint_angles, self.dh_table)):
            T_i = self.compute_dh_transform(
                theta=q,
                d=dh['d'],
                a=dh['a'],
                alpha=dh['alpha']
            )
            T_end = T_end @ T_i  # Matrix multiplication

            # Debug: Log intermediate transformations
            self.get_logger().debug(f'T_{i+1} =\n{T_i}')

        # Extract position (last column, first 3 rows)
        position = T_end[0:3, 3]

        # Extract rotation (upper-left 3x3 submatrix)
        rotation = T_end[0:3, 0:3]

        # Validate results
        if np.any(np.isnan(position)) or np.any(np.isnan(rotation)):
            self.get_logger().error('NaN detected in FK computation')
            raise ValueError('Invalid FK result: NaN values')

        self.get_logger().debug(f'Final transformation:\n{T_end}')

        return position, rotation

    def rotation_matrix_to_quaternion(self, rotation: np.ndarray) -> Quaternion:
        """
        Convert a 3x3 rotation matrix to a quaternion.

        Args:
            rotation: 3x3 rotation matrix

        Returns:
            geometry_msgs/Quaternion message
        """
        # Create 4x4 homogeneous matrix for tf_transformations
        T = np.eye(4)
        T[0:3, 0:3] = rotation

        # Convert to quaternion [x, y, z, w]
        q = quaternion_from_matrix(T)

        # Create ROS message
        quaternion_msg = Quaternion()
        quaternion_msg.x = q[0]
        quaternion_msg.y = q[1]
        quaternion_msg.z = q[2]
        quaternion_msg.w = q[3]

        return quaternion_msg

    def joint_state_callback(self, msg: JointState):
        """
        Callback function for joint state updates.

        Computes forward kinematics and publishes end-effector pose.

        Args:
            msg: JointState message containing joint positions
        """
        # Extract joint positions
        if len(msg.position) < 3:
            self.get_logger().warning(f'Expected at least 3 joint positions, got {len(msg.position)}')
            return

        joint_angles = list(msg.position[0:3])
        self.current_joint_angles = joint_angles

        # Log received joint angles
        self.get_logger().info(
            f'Received joint states: [{joint_angles[0]:.3f}, {joint_angles[1]:.3f}, {joint_angles[2]:.3f}] rad'
        )

        try:
            # Compute forward kinematics
            position, rotation = self.compute_forward_kinematics(joint_angles)

            # Log end-effector position
            self.get_logger().info(
                f'End-effector position: [{position[0]:.3f}, {position[1]:.3f}, {position[2]:.3f}] m'
            )

            # Create and publish PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'world'

            pose_msg.pose.position.x = float(position[0])
            pose_msg.pose.position.y = float(position[1])
            pose_msg.pose.position.z = float(position[2])

            pose_msg.pose.orientation = self.rotation_matrix_to_quaternion(rotation)

            self.pose_publisher.publish(pose_msg)
            self.get_logger().info('Published to /end_effector_pose')

        except Exception as e:
            self.get_logger().error(f'Error in FK computation: {str(e)}')


def main(args=None):
    """Main entry point for the FK calculator node."""
    rclpy.init(args=args)

    try:
        fk_calculator = ForwardKinematicsCalculator()
        rclpy.spin(fk_calculator)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
