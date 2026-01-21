#!/usr/bin/env python3
"""
Lab 2.1: Forward Kinematics Implementation

This script implements forward kinematics for a 2-DOF planar arm using
Denavit-Hartenberg parameters.

Learning Objectives:
- Define DH table for 2-link arm
- Implement DH transformation function
- Compute end-effector position from joint angles
- Visualize arm configuration

Expected Output:
    End-effector position: [x, y] = [1.366, 1.166]
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List

# Robot parameters
L1 = 1.0  # Length of link 1 (meters)
L2 = 0.8  # Length of link 2 (meters)


def compute_dh_transform(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """
    TODO: Implement this function

    Compute Denavit-Hartenberg transformation matrix.

    The DH transformation matrix maps from frame i-1 to frame i.
    It combines rotation about z-axis (theta) and translation along
    x-axis (a) with proper rotations and translations.

    Parameters:
    -----------
    theta : float
        Joint angle in radians (rotation about z_{i-1})
    d : float
        Link offset in meters (translation along z_{i-1})
    a : float
        Link length in meters (translation along x_i)
    alpha : float
        Link twist in radians (rotation about x_i)

    Returns:
    --------
    np.ndarray
        4x4 homogeneous transformation matrix

    Hints:
    - Use np.cos() and np.sin() for trigonometric functions
    - The transformation matrix format is:
    | cos(theta)  -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)  a*cos(theta) |
    | sin(theta)   cos(theta)*cos(alpha)   -cos(theta)*sin(alpha) a*sin(theta) |
    | 0            sin(alpha)              cos(alpha)              d            |
    | 0            0                        0                       1            |
    """
    # Your code here: compute the 4x4 transformation matrix

    # === YOUR CODE START ===
    pass
    # === YOUR CODE END ===

    # Replace this with actual implementation
    # transform = np.eye(4)
    # return transform


def define_dh_table(L1: float, L2: float) -> List[List[float]]:
    """
    Define the DH parameter table for a 2-DOF planar arm.

    For a planar arm with revolute joints along the x-axis:
    - Joint 1: theta varies, d=0, a=L1, alpha=0
    - Joint 2: theta varies, d=0, a=L2, alpha=0

    Parameters:
    -----------
    L1 : float
        Length of link 1 in meters
    L2 : float
        Length of link 2 in meters

    Returns:
    --------
    List[List[float]]
        DH parameters as [[theta, d, a, alpha], ...] for each joint
    """
    dh_table = [
        [0, 0, L1, 0],  # Joint 1: [theta1, d=0, a=L1, alpha=0]
        [0, 0, L2, 0]   # Joint 2: [theta2, d=0, a=L2, alpha=0]
    ]
    return dh_table


def forward_kinematics(joint_angles: List[float], L1: float, L2: float) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute forward kinematics for 2-DOF planar arm.

    Parameters:
    -----------
    joint_angles : List[float]
        [q1, q2] joint angles in radians
    L1 : float
        Length of link 1
    L2 : float
        Length of link 2

    Returns:
    --------
    Tuple[np.ndarray, np.ndarray, np.ndarray]
        (T_total, joint_positions, end_effector_position)
        - T_total: 4x4 transformation from base to end-effector
        - joint_positions: [[x0,y0], [x1,y1], [x2,y2]] for each joint
        - end_effector_position: [x, y] coordinates
    """
    q1, q2 = joint_angles

    # Compute individual transforms
    # Transform from base to joint 1 (theta=q1, d=0, a=L1, alpha=0)
    T1 = compute_dh_transform(q1, 0, L1, 0)

    # Transform from joint 1 to joint 2 (theta=q2, d=0, a=L2, alpha=0)
    T2 = compute_dh_transform(q2, 0, L2, 0)

    # Total transformation: T_total = T1 * T2
    T_total = T1 @ T2

    # Extract joint positions for visualization
    joint_positions = np.array([
        [0, 0],                    # Base
        [T1[0, 3], T1[1, 3]],      # Joint 1
        [T_total[0, 3], T_total[1, 3]]  # End-effector
    ])

    end_effector_position = T_total[:2, 3]

    return T_total, joint_positions, end_effector_position


def visualize_arm(joint_positions: np.ndarray, L1: float, L2: float,
                  joint_angles: List[float]) -> None:
    """
    Visualize the 2-DOF arm configuration.

    Parameters:
    -----------
    joint_positions : np.ndarray
        Array of shape (3, 2) containing [x, y] for base, joint1, end-effector
    L1 : float
        Length of link 1
    L2 : float
        Length of link 2
    joint_angles : List[float]
        Joint angles in degrees for labeling
    """
    fig, ax = plt.subplots(figsize=(10, 8))

    # Plot links as lines
    ax.plot([joint_positions[0, 0], joint_positions[1, 0]],
            [joint_positions[0, 1], joint_positions[1, 1]],
            'b-', linewidth=3, label='Link 1')
    ax.plot([joint_positions[1, 0], joint_positions[2, 0]],
            [joint_positions[1, 1], joint_positions[2, 1]],
            'r-', linewidth=3, label='Link 2')

    # Plot joints as points
    ax.scatter(joint_positions[:, 0], joint_positions[:, 1],
               s=100, c=['green', 'blue', 'red'],
               zorder=5, edgecolors='black', linewidths=2)

    # Add labels
    ax.annotate('Base\n(q1 joint)', joint_positions[0],
                textcoords="offset points", xytext=(15, 15),
                fontsize=10, ha='left')

    joint1_label = f'Joint 1\n(L1={L1}m)'
    ax.annotate(joint1_label, joint_positions[1],
                textcoords="offset points", xytext=(15, 15),
                fontsize=10, ha='left')

    end_effector_label = f'End Effector\n({joint_positions[2, 0]:.3f}, {joint_positions[2, 1]:.3f})'
    ax.annotate(end_effector_label, joint_positions[2],
                textcoords="offset points", xytext=(15, -20),
                fontsize=10, ha='left')

    # Formatting
    ax.set_xlim(-0.5, max(L1 + L2 + 0.5, joint_positions[2, 0] + 0.5))
    ax.set_ylim(-0.5, max(L1 + L2 + 0.5, joint_positions[2, 1] + 0.5))
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.axhline(y=0, color='k', linestyle='-', linewidth=0.5)
    ax.axvline(x=0, color='k', linestyle='-', linewidth=0.5)

    angle_str = f"q1 = {joint_angles[0]:.1f}°, q2 = {joint_angles[1]:.1f}°"
    ax.set_title(f'2-DOF Planar Arm Forward Kinematics\n{angle_str}', fontsize=14)
    ax.set_xlabel('X (meters)', fontsize=12)
    ax.set_ylabel('Y (meters)', fontsize=12)
    ax.legend(loc='upper right')

    # Add text box with FK result
    result_text = f"End-effector: ({joint_positions[2, 0]:.3f}, {joint_positions[2, 1]:.3f}) m"
    ax.text(0.02, 0.98, result_text, transform=ax.transAxes,
            fontsize=11, verticalalignment='top',
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))

    plt.tight_layout()
    plt.savefig('lab2_1_arm_visualization.png', dpi=150, bbox_inches='tight')
    print(f"Visualization saved to 'lab2_1_arm_visualization.png'")
    plt.show()


def main():
    """
    Main function to run forward kinematics lab.
    """
    print("=" * 60)
    print("Lab 2.1: Forward Kinematics Implementation")
    print("=" * 60)

    # Test configuration
    q1_deg = 30  # degrees
    q2_deg = 45  # degrees

    # Convert to radians
    q1 = np.radians(q1_deg)
    q2 = np.radians(q2_deg)

    print(f"\nRobot Parameters:")
    print(f"  L1 = {L1} m")
    print(f"  L2 = {L2} m")
    print(f"\nJoint Configuration:")
    print(f"  q1 = {q1_deg} degrees = {q1:.4f} radians")
    print(f"  q2 = {q2_deg} degrees = {q2:.4f} radians")

    # Compute forward kinematics
    T_total, joint_positions, ee_position = forward_kinematics([q1, q2], L1, L2)

    print(f"\n" + "-" * 60)
    print("RESULTS")
    print("-" * 60)

    print(f"\nEnd-effector position: [x, y] = [{ee_position[0]:.3f}, {ee_position[1]:.3f}]")
    print(f"\nJoint positions:")
    print(f"  Base: ({joint_positions[0, 0]:.3f}, {joint_positions[0, 1]:.3f})")
    print(f"  Joint 1: ({joint_positions[1, 0]:.3f}, {joint_positions[1, 1]:.3f})")
    print(f"  End-effector: ({joint_positions[2, 0]:.3f}, {joint_positions[2, 1]:.3f})")

    print(f"\nFull transformation matrix:")
    print(T_total)

    # Verification: compute manually for validation
    manual_x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
    manual_y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
    print(f"\nVerification (manual calculation):")
    print(f"  x = L1*cos(q1) + L2*cos(q1+q2) = {manual_x:.3f}")
    print(f"  y = L1*sin(q1) + L2*sin(q1+q2) = {manual_y:.3f}")

    # Visualize
    print(f"\n" + "-" * 60)
    print("VISUALIZATION")
    print("-" * 60)
    visualize_arm(joint_positions, L1, L2, [q1_deg, q2_deg])

    print("\n" + "=" * 60)
    print("Lab 2.1 Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
