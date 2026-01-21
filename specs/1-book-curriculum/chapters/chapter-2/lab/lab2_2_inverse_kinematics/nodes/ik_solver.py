#!/usr/bin/env python3
"""
Inverse Kinematics Solver Node

This ROS 2 node provides IK solving services for robot arms:
  - Analytical IK for 2-DOF planar arms
  - Numerical IK using Jacobian pseudo-inverse for higher DOF arms

Author: Physical AI Lab
License: MIT
ROS 2 Version: Humble
"""

import numpy as np
from typing import List, Tuple, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from std_srvs.srv import Trigger
from example_interfaces.srv import AddTwoInts  # Placeholder, will use custom service


class IKSolverNode(Node):
    """
    Provides inverse kinematics solving as a ROS 2 service.

    Services:
        /compute_ik: Compute IK for a target pose

    Parameters:
        - link_lengths (list): Lengths of robot links [L1, L2, ...] in meters
        - ik_method (str): 'analytical' or 'numerical'
        - max_iterations (int): Max iterations for numerical IK
        - epsilon (float): Convergence threshold in meters
        - alpha (float): Step size for gradient descent
    """

    def __init__(self):
        super().__init__('ik_solver')

        # Declare and get parameters
        self.declare_parameter('link_lengths', [1.0, 0.8, 0.5])
        self.declare_parameter('ik_method', 'analytical')
        self.declare_parameter('max_iterations', 100)
        self.declare_parameter('epsilon', 0.001)
        self.declare_parameter('alpha', 0.5)

        self.link_lengths = self.get_parameter('link_lengths').value
        self.ik_method = self.get_parameter('ik_method').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.epsilon = self.get_parameter('epsilon').value
        self.alpha = self.get_parameter('alpha').value

        # Validate parameters
        if len(self.link_lengths) < 2:
            self.get_logger().error('Need at least 2 link lengths')
            raise ValueError('Invalid link_lengths parameter')

        if self.ik_method not in ['analytical', 'numerical']:
            self.get_logger().error(f'Invalid ik_method: {self.ik_method}')
            raise ValueError('ik_method must be analytical or numerical')

        self.get_logger().info('IK Solver Node Started')
        self.get_logger().info(f'Link lengths: {self.link_lengths}')
        self.get_logger().info(f'IK method: {self.ik_method}')
        self.get_logger().info('Service /compute_ik ready')

    def compute_dh_transform(self, theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """
        Compute DH transformation matrix.

        Args:
            theta: Joint angle [rad]
            d: Link offset [m]
            a: Link length [m]
            alpha: Link twist [rad]

        Returns:
            4x4 transformation matrix
        """
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(alpha), np.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0,   sa,       ca,      d],
            [0,   0,        0,       1]
        ], dtype=np.float64)

    def compute_forward_kinematics(self, joint_angles: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Compute forward kinematics.

        Args:
            joint_angles: Array of joint angles [rad]

        Returns:
            (position, rotation_matrix)
        """
        n_joints = len(joint_angles)
        T = np.eye(4)

        for i in range(n_joints):
            T_i = self.compute_dh_transform(
                theta=joint_angles[i],
                d=0.0,
                a=self.link_lengths[i],
                alpha=0.0
            )
            T = T @ T_i

        position = T[0:3, 3]
        rotation = T[0:3, 0:3]

        return position, rotation

    def compute_analytical_ik(self, x: float, y: float) -> List[Tuple[float, float]]:
        """
        Compute analytical IK for 2-DOF planar arm using law of cosines.

        This method finds all possible joint configurations that reach
        the target (x, y) position. For a 2-DOF arm, there are typically
        two solutions: "elbow-up" and "elbow-down".

        Args:
            x: Target x-position [m]
            y: Target y-position [m]

        Returns:
            List of (theta1, theta2) tuples in radians
            Empty list if target is unreachable

        Mathematical Approach:
            1. Compute distance to target: d = √(x² + y²)
            2. Check reachability: |L1 - L2| ≤ d ≤ L1 + L2
            3. Apply law of cosines to find θ2 (elbow angle)
            4. Compute θ1 (shoulder angle) using geometry
        """
        if len(self.link_lengths) < 2:
            self.get_logger().error('Analytical IK requires at least 2 links')
            return []

        L1 = self.link_lengths[0]
        L2 = self.link_lengths[1]

        # Compute distance to target
        d = np.sqrt(x**2 + y**2)

        # Check reachability
        min_reach = abs(L1 - L2)
        max_reach = L1 + L2

        if d > max_reach:
            self.get_logger().warning(
                f'Target ({x:.3f}, {y:.3f}) too far: d={d:.3f}m > max_reach={max_reach:.3f}m'
            )
            return []

        if d < min_reach:
            self.get_logger().warning(
                f'Target ({x:.3f}, {y:.3f}) too close: d={d:.3f}m < min_reach={min_reach:.3f}m'
            )
            return []

        # Compute elbow angle using law of cosines
        # cos(θ2) = (x² + y² - L1² - L2²) / (2·L1·L2)
        cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)

        # Clamp to valid range to avoid numerical errors
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

        # Two solutions for elbow angle
        theta2_elbow_up = np.arccos(cos_theta2)
        theta2_elbow_down = -theta2_elbow_up

        solutions = []

        # Solution 1: Elbow-up configuration
        alpha = np.arctan2(y, x)
        beta_up = np.arctan2(L2 * np.sin(theta2_elbow_up), L1 + L2 * np.cos(theta2_elbow_up))
        theta1_elbow_up = alpha - beta_up
        solutions.append((theta1_elbow_up, theta2_elbow_up))

        # Solution 2: Elbow-down configuration
        beta_down = np.arctan2(L2 * np.sin(theta2_elbow_down), L1 + L2 * np.cos(theta2_elbow_down))
        theta1_elbow_down = alpha - beta_down
        solutions.append((theta1_elbow_down, theta2_elbow_down))

        self.get_logger().info(f'Found {len(solutions)} solutions for target ({x:.3f}, {y:.3f})')
        for i, (t1, t2) in enumerate(solutions, 1):
            self.get_logger().info(f'  Solution {i}: θ1={np.degrees(t1):.1f}°, θ2={np.degrees(t2):.1f}°')

        return solutions

    def compute_jacobian(self, q: np.ndarray, delta: float = 1e-6) -> np.ndarray:
        """
        Compute numerical Jacobian matrix using finite differences.

        The Jacobian relates joint velocities to end-effector velocities:
            v = J · q̇

        Args:
            q: Current joint angles [rad]
            delta: Finite difference step size

        Returns:
            Jacobian matrix of shape (2, n) for planar arm
        """
        n = len(q)
        pos_center, _ = self.compute_forward_kinematics(q)
        pos_center = pos_center[:2]  # Only x, y for planar arm

        J = np.zeros((2, n))

        for i in range(n):
            q_plus = q.copy()
            q_plus[i] += delta

            pos_plus, _ = self.compute_forward_kinematics(q_plus)
            pos_plus = pos_plus[:2]

            # Numerical derivative
            J[:, i] = (pos_plus - pos_center) / delta

        return J

    def compute_numerical_ik(
        self,
        x: float,
        y: float,
        q_init: Optional[np.ndarray] = None
    ) -> Tuple[Optional[np.ndarray], bool]:
        """
        Compute numerical IK using Jacobian pseudo-inverse method.

        This iterative method is suitable for higher DOF arms where
        analytical solutions are difficult or impossible to derive.

        Algorithm:
            1. Start with initial guess q0
            2. Compute error: e = x_target - FK(q)
            3. Compute Jacobian: J = ∂FK/∂q
            4. Update: q ← q + α·J⁺·e  (J⁺ is pseudo-inverse)
            5. Repeat until ||e|| < ε or max iterations

        Args:
            x: Target x-position [m]
            y: Target y-position [m]
            q_init: Initial guess for joint angles [rad]
                   If None, uses zeros

        Returns:
            (joint_angles, converged): Solution and convergence status
        """
        n_joints = len(self.link_lengths)
        target = np.array([x, y])

        # Initialize joint angles
        if q_init is None:
            q = np.zeros(n_joints)
        else:
            q = np.array(q_init, dtype=float)

        self.get_logger().info(f'Starting numerical IK for target ({x:.3f}, {y:.3f})')

        for iteration in range(self.max_iterations):
            # Compute current end-effector position
            pos_current, _ = self.compute_forward_kinematics(q)
            pos_current = pos_current[:2]  # x, y only

            # Compute error
            error = target - pos_current
            error_norm = np.linalg.norm(error)

            self.get_logger().debug(f'Iteration {iteration}: error = {error_norm:.6f}m')

            # Check convergence
            if error_norm < self.epsilon:
                self.get_logger().info(f'Converged in {iteration} iterations')
                self.get_logger().info(f'Solution: θ=[{", ".join([f"{np.degrees(qi):.1f}°" for qi in q])}]')
                return q, True

            # Compute Jacobian
            J = self.compute_jacobian(q)

            # Compute pseudo-inverse
            try:
                J_pinv = np.linalg.pinv(J)
            except np.linalg.LinAlgError:
                self.get_logger().error('Jacobian pseudo-inverse computation failed')
                return None, False

            # Update joint angles
            delta_q = self.alpha * (J_pinv @ error)
            q = q + delta_q

            # Optional: Wrap angles to [-π, π]
            q = np.arctan2(np.sin(q), np.cos(q))

        self.get_logger().warning(
            f'Did not converge after {self.max_iterations} iterations (final error: {error_norm:.6f}m)'
        )
        return q, False

    def validate_ik_solution(self, joint_angles: np.ndarray, target_x: float, target_y: float) -> float:
        """
        Validate IK solution by computing FK and measuring error.

        Args:
            joint_angles: Computed joint angles [rad]
            target_x: Desired x-position [m]
            target_y: Desired y-position [m]

        Returns:
            Position error in meters
        """
        pos, _ = self.compute_forward_kinematics(joint_angles)
        error = np.sqrt((pos[0] - target_x)**2 + (pos[1] - target_y)**2)
        return error


def main(args=None):
    """Main entry point for IK solver node."""
    rclpy.init(args=args)

    try:
        ik_solver = IKSolverNode()

        # Example: Test analytical IK
        print("\n" + "="*60)
        print("Testing Analytical IK (2-DOF)")
        print("="*60)

        test_targets = [
            (1.0, 0.0),
            (1.0, 1.0),
            (0.5, 0.5),
            (1.8, 0.0),  # Max reach
            (2.5, 0.0),  # Unreachable
        ]

        for x, y in test_targets:
            print(f"\nTarget: ({x:.1f}, {y:.1f})")
            solutions = ik_solver.compute_analytical_ik(x, y)

            for i, sol in enumerate(solutions, 1):
                q = np.array([sol[0], sol[1]])
                error = ik_solver.validate_ik_solution(q, x, y)
                print(f"  Solution {i}: θ=[{np.degrees(sol[0]):.1f}°, {np.degrees(sol[1]):.1f}°], "
                      f"Error: {error*1000:.3f}mm")

        # Example: Test numerical IK
        print("\n" + "="*60)
        print("Testing Numerical IK (3-DOF)")
        print("="*60)

        for x, y in [(1.5, 0.5), (1.0, 1.2)]:
            print(f"\nTarget: ({x:.1f}, {y:.1f})")
            q_solution, converged = ik_solver.compute_numerical_ik(x, y)

            if converged:
                error = ik_solver.validate_ik_solution(q_solution, x, y)
                print(f"  Converged: θ=[{', '.join([f'{np.degrees(qi):.1f}°' for qi in q_solution])}]")
                print(f"  Error: {error*1000:.3f}mm")
            else:
                print("  Did not converge")

        print("\n" + "="*60)
        print("IK Solver ready. Press Ctrl+C to exit.")
        print("="*60 + "\n")

        rclpy.spin(ik_solver)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
