#!/usr/bin/env python3
"""
Lab 2.2: Inverse Kinematics Solver - STARTER CODE

This lab implements:
1. Analytical IK for 2-DOF arm using law of cosines
2. Numerical IK for 3-DOF arm using Jacobian/damped least squares

Learning Objectives:
- Analytical IK using geometric approach
- Handle multiple solutions (elbow-up vs elbow-down)
- Jacobian-based numerical IK with damping
- Convergence visualization

Expected Output:
    Target position: [1.0, 1.0]
    Solution 1 (elbow-up): [45.0, 90.0] degrees
    Solution 2 (elbow-down): [135.0, -90.0] degrees
    Numerical IK converged in 8 iterations
"""

import numpy as np
import matplotlib.pyplot as plt
from typing import Tuple, List, Optional
import time


# Robot parameters for 2-DOF arm
L1_2DOF = 1.0  # meters
L2_2DOF = 0.8  # meters

# Robot parameters for 3-DOF arm
L1_3DOF = 0.8  # meters
L2_3DOF = 0.6  # meters
L3_3DOF = 0.4  # meters


def analytical_ik_2dof(x: float, y: float, L1: float, L2: float) -> Tuple[Optional[List[float]], Optional[List[float]]]:
    """
    TODO: Implement this function

    Analytical inverse kinematics for 2-DOF planar arm.

    Uses the law of cosines to find the elbow angle, then computes
    the shoulder angle using atan2. Returns both elbow-up and elbow-down
    solutions.

    Parameters:
    -----------
    x : float
        Target x coordinate (meters)
    y : float
        Target y coordinate (meters)
    L1 : float
        Length of link 1
    L2 : float
        Length of link 2

    Returns:
    --------
    Tuple[List[float], List[float]]
        (solution1, solution2) where each is [theta1, theta2] in radians
        Returns (None, None) if target is unreachable
    """
    # Distance from base to target
    r = np.sqrt(x**2 + y**2)

    # Check reachability
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)

    if r > max_reach or r < min_reach:
        print(f"Target ({x}, {y}) is unreachable!")
        print(f"  Distance: {r:.3f}, Max reach: {max_reach}, Min reach: {min_reach}")
        return None, None

    # === YOUR CODE START ===
    pass
    # === YOUR CODE END ===

    return None, None  # Replace with actual implementation


def forward_kinematics_2dof(joint_angles: List[float], L1: float, L2: float) -> Tuple[float, float]:
    """
    Compute end-effector position for 2-DOF arm (for verification).
    """
    q1, q2 = joint_angles
    x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2)
    y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2)
    return x, y


def forward_kinematics_3dof(joint_angles: List[float], L1: float, L2: float, L3: float) -> np.ndarray:
    """
    Compute end-effector position for 3-DOF arm.
    """
    q1, q2, q3 = joint_angles
    x = L1 * np.cos(q1) + L2 * np.cos(q1 + q2) + L3 * np.cos(q1 + q2 + q3)
    y = L1 * np.sin(q1) + L2 * np.sin(q1 + q2) + L3 * np.sin(q1 + q2 + q3)
    return np.array([x, y])


def compute_jacobian_3dof(q: List[float], L1: float, L2: float, L3: float,
                          delta: float = 1e-6) -> np.ndarray:
    """
    TODO: Implement this function

    Compute numerical Jacobian for 3-DOF arm.

    The Jacobian maps joint velocities to end-effector velocities:
        [dx/dq1, dx/dq2, dx/dq3]
    J = [dy/dq1, dy/dq2, dy/dq3]

    Parameters:
    -----------
    q : List[float]
        Current joint angles [q1, q2, q3] in radians
    L1, L2, L3 : float
        Link lengths
    delta : float
        Finite difference step size

    Returns:
    --------
    np.ndarray
        2x3 Jacobian matrix
    """
    # Current end-effector position
    p_current = forward_kinematics_3dof(q, L1, L2, L3)

    # Initialize Jacobian
    J = np.zeros((2, 3))

    # === YOUR CODE START ===
    # HINT: Use finite differences to compute partial derivatives
    # For each joint i, compute the position when q[i] is increased by delta
    # The column J[:, i] is the gradient of position with respect to q[i]
    pass
    # === YOUR CODE END ===

    return J


def numerical_ik_3dof(target: np.ndarray, initial_q: List[float],
                      L1: float, L2: float, L3: float,
                      max_iterations: int = 100,
                      tolerance: float = 1e-4,
                      lambda_damping: float = 0.1) -> Tuple[List[float], int, List[float]]:
    """
    TODO: Implement this function

    Numerical inverse kinematics using Damped Least Squares.

    Iteratively adjusts joint angles to minimize position error:
        q_{k+1} = q_k + alpha * J^+ * error

    Where J^+ is the damped pseudo-inverse:
        J^+ = J^T * (J * J^T + lambda^2 * I)^(-1)

    Parameters:
    -----------
    target : np.ndarray
        Target position [x, y]
    initial_q : List[float]
        Initial joint configuration
    L1, L2, L3 : float
        Link lengths
    max_iterations : int
        Maximum number of iterations
    tolerance : float
        Convergence threshold (error magnitude)
    lambda_damping : float
        Damping parameter for singularity handling

    Returns:
    --------
    Tuple[List[float], int, List[float]]
        (final_joint_angles, iterations, error_history)
    """
    q = np.array(initial_q)
    error_history = []
    alpha = 0.5  # Step size

    for iteration in range(max_iterations):
        # === YOUR CODE START ===
        pass
        # === YOUR CODE END ===

    return q.tolist(), max_iterations, error_history


def visualize_ik_solutions(solution1: List[float], solution2: List[float],
                           L1: float, L2: float, target: Tuple[float, float]) -> None:
    """
    Visualize both IK solutions for 2-DOF arm.
    """
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    solutions = [solution1, solution2]
    titles = ["Solution 1 (Elbow-Up)", "Solution 2 (Elbow-Down)"]

    for ax, solution, title in zip(axes, solutions, titles):
        if solution is None:
            ax.text(0.5, 0.5, "No solution", ha='center', va='center')
            continue

        q1, q2 = solution
        x1 = L1 * np.cos(q1)
        y1 = L1 * np.sin(q1)
        x2 = x1 + L2 * np.cos(q1 + q2)
        y2 = y1 + L2 * np.sin(q1 + q2)

        # Plot links
        ax.plot([0, x1], [0, y1], 'b-', linewidth=3, label='Link 1')
        ax.plot([x1, x2], [y1, y2], 'r-', linewidth=3, label='Link 2')

        # Plot joints
        ax.scatter([0, x1, x2], [0, y1, y2], s=100, c=['g', 'b', 'r'],
                   zorder=5, edgecolors='black', linewidths=2)

        # Plot target
        ax.scatter([target[0]], [target[1]], s=200, c=['purple'], marker='*',
                   zorder=6, label='Target')

        ax.set_xlim(-0.5, L1 + L2 + 0.5)
        ax.set_ylim(-0.5, L1 + L2 + 0.5)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.axhline(y=0, color='k', linewidth=0.5)
        ax.axvline(x=0, color='k', linewidth=0.5)

        angles = [np.degrees(a) for a in solution]
        ax.set_title(f"{title}\nq = [{angles[0]:.1f}, {angles[1]:.1f}] degrees")
        ax.set_xlabel('X (meters)')
        ax.set_ylabel('Y (meters)')
        ax.legend()

    plt.tight_layout()
    plt.savefig('lab2_2_ik_solutions.png', dpi=150, bbox_inches='tight')
    print("IK solutions visualization saved to 'lab2_2_ik_solutions.png'")
    plt.show()


def visualize_convergence(error_history: List[float], iterations: int,
                          initial_q: List[float], final_q: List[float],
                          L1: float, L2: float, L3: float,
                          target: np.ndarray) -> None:
    """
    Visualize numerical IK convergence.
    """
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))

    # Plot 1: Error vs iteration
    ax = axes[0, 0]
    ax.plot(error_history, 'b-', linewidth=2)
    ax.set_yscale('log')
    ax.set_xlabel('Iteration')
    ax.set_ylabel('Error (m, log scale)')
    ax.set_title(f'Convergence: {iterations} iterations')
    ax.grid(True, alpha=0.3)

    # Plot 2: Initial vs Final configuration
    ax = axes[0, 1]
    for i, q in enumerate([initial_q, final_q]):
        x0, y0 = 0, 0
        x1 = L1 * np.cos(q[0])
        y1 = L1 * np.sin(q[0])
        x2 = x1 + L2 * np.cos(q[0] + q[1])
        y2 = y1 + L2 * np.sin(q[0] + q[1])
        x3 = x2 + L3 * np.cos(q[0] + q[1] + q[2])
        y3 = y2 + L3 * np.sin(q[0] + q[1] + q[2])

        color = 'blue' if i == 0 else 'green'
        label = 'Initial' if i == 0 else 'Final'
        linestyle = '--' if i == 0 else '-'

        ax.plot([x0, x1], [y0, y1], color=color, linestyle=linestyle, linewidth=2)
        ax.plot([x1, x2], [y1, y2], color=color, linestyle=linestyle, linewidth=2)
        ax.plot([x2, x3], [y2, y3], color=color, linestyle=linestyle, linewidth=2, label=f'{label} config')

    ax.scatter([target[0]], [target[1]], s=200, c=['red'], marker='*', zorder=5, label='Target')
    ax.set_xlim(-0.5, L1 + L2 + L3 + 0.5)
    ax.set_ylim(-0.5, L1 + L2 + L3 + 0.5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title('Initial (blue) vs Final (green) Configuration')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.legend(loc='upper right', fontsize=8)

    # Plot 3: Joint angle comparison
    ax = axes[1, 0]
    initial_deg = [np.degrees(a) for a in initial_q]
    final_deg = [np.degrees(a) for a in final_q]
    x_joints = range(len(initial_q))
    width = 0.35
    ax.bar([x - width/2 for x in x_joints], initial_deg, width, label='Initial', color='blue', alpha=0.7)
    ax.bar([x + width/2 for x in x_joints], final_deg, width, label='Final', color='green', alpha=0.7)
    ax.set_xlabel('Joint')
    ax.set_ylabel('Angle (degrees)')
    ax.set_title('Joint Angle Change')
    ax.set_xticks(x_joints)
    ax.set_xticklabels(['q1', 'q2', 'q3'])
    ax.legend()
    ax.grid(True, alpha=0.3, axis='y')

    # Plot 4: Workspace reachability
    ax = axes[1, 1]
    theta = np.linspace(0, 2*np.pi, 100)
    max_r = L1 + L2 + L3
    ax.fill_between(theta, 0, max_r, alpha=0.2, color='blue', label='Reachable workspace')
    r_target = np.linalg.norm(target)
    ax.scatter([target[0]], [target[1]], s=200, c=['red'], marker='*', label='Target')
    ax.set_xlim(-max_r - 0.2, max_r + 0.2)
    ax.set_ylim(-max_r - 0.2, max_r + 0.2)
    ax.set_aspect('equal')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Target in Workspace')
    ax.legend()
    ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig('lab2_2_convergence.png', dpi=150, bbox_inches='tight')
    print("Convergence visualization saved to 'lab2_2_convergence.png'")
    plt.show()


def main():
    """
    Main function to run inverse kinematics lab.
    """
    print("=" * 60)
    print("Lab 2.2: Inverse Kinematics Solver")
    print("=" * 60)

    # Part 1: Analytical IK for 2-DOF arm
    print("\n" + "-" * 60)
    print("PART 1: Analytical IK for 2-DOF Arm")
    print("-" * 60)

    target_2dof = (1.0, 1.0)
    print(f"\nTarget position: {target_2dof}")

    sol1, sol2 = analytical_ik_2dof(target_2dof[0], target_2dof[1], L1_2DOF, L2_2DOF)

    if sol1 is not None:
        sol1_deg = [np.degrees(a) for a in sol1]
        sol2_deg = [np.degrees(a) for a in sol2]
        print(f"Solution 1 (elbow-up): [{sol1_deg[0]:.1f}, {sol1_deg[1]:.1f}] degrees")
        print(f"Solution 2 (elbow-down): [{sol2_deg[0]:.1f}, {sol2_deg[1]:.1f}] degrees")

        # Verify solutions
        x1, y1 = forward_kinematics_2dof(sol1, L1_2DOF, L2_2DOF)
        x2, y2 = forward_kinematics_2dof(sol2, L1_2DOF, L2_2DOF)
        print(f"\nVerification - Solution 1 reaches: ({x1:.3f}, {y1:.3f})")
        print(f"Verification - Solution 2 reaches: ({x2:.3f}, {y2:.3f})")

        # Visualize
        visualize_ik_solutions(sol1, sol2, L1_2DOF, L2_2DOF, target_2dof)

    # Part 2: Numerical IK for 3-DOF arm
    print("\n" + "-" * 60)
    print("PART 2: Numerical IK for 3-DOF Arm")
    print("-" * 60)

    target_3dof = np.array([1.2, 0.8])
    initial_q = [np.pi/4, np.pi/6, np.pi/12]  # Initial guess
    print(f"\nTarget position: {target_3dof}")
    print(f"Initial joint configuration: {[np.degrees(a) for a in initial_q]} degrees")

    start_time = time.time()
    final_q, iterations, error_history = numerical_ik_3dof(
        target_3dof, initial_q,
        L1_3DOF, L2_3DOF, L3_3DOF,
        max_iterations=100,
        tolerance=1e-4,
        lambda_damping=0.1
    )
    elapsed = time.time() - start_time

    final_q_deg = [np.degrees(a) for a in final_q]
    print(f"\nNumerical IK converged in {iterations} iterations (took {elapsed:.4f}s)")
    print(f"Final joint configuration: [{final_q_deg[0]:.1f}, {final_q_deg[1]:.1f}, {final_q_deg[2]:.1f}] degrees")

    # Verify
    final_pos = forward_kinematics_3dof(final_q, L1_3DOF, L2_3DOF, L3_3DOF)
    error = np.linalg.norm(target_3dof - final_pos)
    print(f"Final position: {final_pos}")
    print(f"Position error: {error:.6f} m")

    # Visualize
    visualize_convergence(error_history, iterations, initial_q, final_q,
                          L1_3DOF, L2_3DOF, L3_3DOF, target_3dof)

    print("\n" + "=" * 60)
    print("Lab 2.2 Complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
