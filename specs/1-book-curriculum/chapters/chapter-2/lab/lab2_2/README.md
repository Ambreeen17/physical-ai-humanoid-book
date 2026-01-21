# Lab 2.2: Inverse Kinematics Solver

**Duration**: 45 minutes
**Difficulty**: Intermediate
**Prerequisites**: Chapter 2 Sections 2.2-2.3, Lab 2.1 (Forward Kinematics), numpy, matplotlib

---

## Learning Objectives

By completing this lab, you will be able to:

1. Implement analytical inverse kinematics for a 2-DOF planar arm using the law of cosines
2. Handle multiple solutions (elbow-up vs elbow-down configurations)
3. Implement Jacobian-based numerical IK for a 3-DOF arm using damped least squares
4. Visualize the convergence of iterative solvers

---

## Background Theory

### The Inverse Kinematics Problem

Inverse kinematics asks: "Given a desired end-effector position, what joint angles achieve it?" Unlike forward kinematics which has a unique solution, IK may have:

- **No solution**: Target is outside the workspace
- **Multiple solutions**: Different configurations reach the same point
- **Infinite solutions**: Redundant robots (more DOF than task requires)

### Analytical IK for 2-DOF Arms

For a 2-link planar arm, we can derive closed-form solutions using geometry. The target point (x, y) and link lengths (L1, L2) form a triangle:

```
        (x, y)
        /    \
       /      \
      /   r    \
     /__________\
   Base        Joint 2
```

The law of cosines gives the elbow angle:

$$
\cos\theta_2 = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}
$$

Then theta_1 is computed using atan2:

$$
\theta_1 = \text{atan2}(y, x) - \text{atan2}(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2)
$$

The two solutions correspond to elbow-up (+) and elbow-down (-).

### Numerical IK for 3-DOF Arms

For arms with more degrees of freedom, we use iterative methods based on the Jacobian:

$$
\dot{x} = J(q) \dot{q}
$$

Inverting (using pseudo-inverse):

$$
\dot{q} = J^+ \dot{x}
$$

**Damped Least Squares** adds regularization to handle singularities:

$$
J^+ = J^T (J J^T + \lambda^2 I)^{-1}
$$

---

## Tasks

### Task 1: Analytical IK for 2-DOF Arm

Implement `analytical_ik_2dof(x, y, L1, L2)` that:
1. Checks reachability (target within workspace)
2. Computes elbow angle using law of cosines
3. Returns both elbow-up and elbow-down solutions

### Task 2: Handle Multiple Solutions

Add functionality to:
1. Return joint limits for each solution
2. Select the solution closer to current configuration
3. Visualize both configurations

### Task 3: Numerical IK for 3-DOF Arm

Implement `numerical_ik_3dof(target_pose, initial_q, L1, L2, L3)` using:
1. Numerical Jacobian computation
2. Damped least squares for stability
3. Iteration until convergence

### Task 4: Visualize Convergence

Track and plot:
1. Error magnitude vs iteration
2. Joint trajectory during convergence
3. Initial and final arm configurations

---

## Expected Output

```
Target position: [1.0, 1.0]
Solution 1 (elbow-up): [45.0, 90.0] degrees
Solution 2 (elbow-down): [135.0, -90.0] degrees
Numerical IK converged in 8 iterations
```

---

## Starter Code

Save the following as `lab2_2_starter.py`:

```python
#!/usr/bin/env python3
"""
Lab 2.2: Inverse Kinematics Solver

This lab implements:
1. Analytical IK for 2-DOF arm using law of cosines
2. Numerical IK for 3-DOF arm using Jacobian/damped least squares

Learning Objectives:
- Analytical IK using geometric approach
- Handle multiple solutions (elbow-up vs elbow-down)
- Jacobian-based numerical IK with damping
- Convergence visualization
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
    # Law of cosines for theta2 (elbow angle)
    # cos(theta2) = (r^2 - L1^2 - L2^2) / (2 * L1 * L2)
    cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1, 1)  # Numerical safety

    # Two solutions: elbow-up (+) and elbow-down (-)
    theta2_up = np.arccos(cos_theta2)
    theta2_down = -np.arccos(cos_theta2)

    # Compute theta1 (shoulder angle) for each case
    # theta1 = atan2(y, x) - atan2(L2*sin(theta2), L1 + L2*cos(theta2))
    phi = np.arctan2(y, x)

    k1 = L1 + L2 * np.cos(theta2_up)
    k2 = L2 * np.sin(theta2_up)
    theta1_up = phi - np.arctan2(k2, k1)

    k1 = L1 + L2 * np.cos(theta2_down)
    k2 = L2 * np.sin(theta2_down)
    theta1_down = phi - np.arctan2(k2, k1)

    return [theta1_up, theta2_up], [theta1_down, theta2_down]
    # === YOUR CODE END ===
    # return None, None  # Replace with actual implementation


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
    # Compute partial derivatives using finite differences
    for i in range(3):
        q_plus = q.copy()
        q_plus[i] += delta
        p_plus = forward_kinematics_3dof(q_plus, L1, L2, L3)
        J[:, i] = (p_plus - p_current) / delta
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

        # Compute current position
        current = forward_kinematics_3dof(q, L1, L2, L3)

        # Compute error
        error = target - current
        error_mag = np.linalg.norm(error)
        error_history.append(error_mag)

        # Check convergence
        if error_mag < tolerance:
            return q.tolist(), iteration + 1, error_history

        # Compute Jacobian
        J = compute_jacobian_3dof(q, L1, L2, L3)

        # Damped Least Squares: J_dls = J^T * (J * J^T + lambda^2 * I)^(-1)
        J_jtj = J @ J.T + lambda_damping**2 * np.eye(2)
        J_dls = J.T @ np.linalg.solve(J_jtj, np.eye(2))

        # Update joint angles
        delta_q = alpha * J_dls @ error
        q = q + delta_q
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

    # Plot 2: Initial configuration
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

        ax.plot([x0, x1], [y0, y1], color=color, linestyle=linestyle, linewidth=2, label=f'{label} L1')
        ax.plot([x1, x2], [y1, y2], color=color, linestyle=linestyle, linewidth=2, label=f'{label} L2')
        ax.plot([x2, x3], [y2, y3], color=color, linestyle=linestyle, linewidth=2, label=f'{label} L3')

    ax.scatter([target[0]], [target[1]], s=200, c=['red'], marker='*', zorder=5, label='Target')
    ax.set_xlim(-0.5, L1 + L2 + L3 + 0.5)
    ax.set_ylim(-0.5, L1 + L2 + L3 + 0.5)
    ax.set_aspect('equal')
    ax.grid(True, alpha=0.3)
    ax.set_title('Initial (blue) vs Final (green) Configuration')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.legend(loc='upper right', fontsize=8)

    # Plot 3: Joint trajectory
    ax = axes[1, 0]
    # We don't have per-iteration joint values stored, so just show initial/final
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

    # Plot 4: Workspace reachability check
    ax = axes[1, 1]
    # Show reachable workspace (annulus)
    theta = np.linspace(0, 2*np.pi, 100)
    max_r = L1 + L2 + L3
    min_r = max(0, L1 - L2 - L3)  # Simplified
    ax.fill_between(theta, min_r, max_r, alpha=0.2, color='blue', label='Reachable workspace')

    # Plot target
    r_target = np.linalg.norm(target)
    theta_target = np.arctan2(target[1], target[0])
    ax.scatter([r_target * np.cos(theta_target)], [r_target * np.sin(theta_target)],
               s=200, c=['red'], marker='*', label='Target')

    ax.set_xlim(-max_r - 0.2, max_r + 0.2)
    ax.set_ylim(-max_r - 0.2, max_r + 0.2)
    ax.set_aspect('equal')
    ax.set_xlabel('X (meters)')
    ax.set_ylabel('Y (meters)')
    ax.set_title('Target Position in Workspace')
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
```

---

## Implementation Hints

### Analytical IK (Law of Cosines)

```python
def analytical_ik_2dof(x, y, L1, L2):
    r = sqrt(x^2 + y^2)
    cos_theta2 = (r^2 - L1^2 - L2^2) / (2 * L1 * L2)
    theta2_up = arccos(cos_theta2)
    theta2_down = -arccos(cos_theta2)

    phi = arctan2(y, x)
    k1 = L1 + L2 * cos(theta2)
    k2 = L2 * sin(theta2)
    theta1 = phi - arctan2(k2, k1)
```

### Jacobian Computation (Numerical)

```python
def compute_jacobian(q, L1, L2, L3, delta=1e-6):
    p_current = forward_kinematics_3dof(q, L1, L2, L3)
    J = zeros((2, 3))
    for i in range(3):
        q_plus = q.copy()
        q_plus[i] += delta
        p_plus = forward_kinematics_3dof(q_plus, L1, L2, L3)
        J[:, i] = (p_plus - p_current) / delta
    return J
```

### Damped Least Squares Update

```python
J_jtj = J @ J.T + lambda^2 * I  # Regularized Gram matrix
J_dls = J.T @ inv(J_jtj)        # Damped pseudo-inverse
delta_q = alpha * J_dls @ error  # Configuration update
```

---

## Verification Steps

1. **Run the script**:
   ```bash
   python lab2_2_starter.py
   ```

2. **Check analytical IK output**:
   ```
   Target position: [1.0, 1.0]
   Solution 1 (elbow-up): [45.0, 90.0] degrees
   Solution 2 (elbow-down): [135.0, -90.0] degrees
   ```

3. **Check numerical IK output**:
   ```
   Numerical IK converged in 8 iterations
   Final position: [1.200, 0.800]
   Position error: 0.000001 m
   ```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Target unreachable | Check that `r <= L1 + L2` for 2-DOF |
| Jacobian singular | Increase `lambda_damping` |
| Slow convergence | Increase step size `alpha` |
| Oscillations | Decrease step size `alpha` |
| No solution returned | Handle None cases in visualization |

---

## Extensions (Optional)

1. **Joint limit handling**: Clamp solutions to [-pi, pi] or custom limits
2. **Solution selection**: Choose solution closest to current configuration
3. **Null-space optimization**: Add secondary objective (e.g., minimize torque)
4. **Orientation IK**: Extend to 3D with full SE(3) pose control
5. **Velocity IK**: Implement real-time velocity tracking

---

## Submission

Complete the TODO sections in `lab2_2_starter.py` and save your solution as `lab2_2_solution.py`. Include:
- Working analytical IK with both solutions
- Working numerical IK with convergence visualization
- Console output showing correct results
