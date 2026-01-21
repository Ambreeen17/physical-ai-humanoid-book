# Chapter 2: Kinematics and Dynamics of Robot Manipulators

**Chapter Overview**: This chapter develops the mathematical foundations for describing and predicting robot motion. We begin with the geometry of robot configurations—how we represent where each joint is and what positions the robot can reach. From there, we derive the forward kinematics equations that map joint positions to end-effector pose, and the inverse kinematics that solve the opposite problem. The second half of the chapter introduces dynamics: how forces and torques produce acceleration, using the elegant Lagrangian formulation that underlies modern robot control. Throughout, we ground these concepts in the Unitree G1 humanoid robot with its 6-DOF arms.

**Learning Objectives**: After completing this chapter, you will be able to define joint space and configuration space for robot manipulators, compute forward kinematics using Denavit-Hartenberg parameters, solve inverse kinematics analytically for simple arms and numerically for general manipulators, derive equations of motion using Lagrangian mechanics, and simulate robot dynamics in physics engines.

**Prerequisites**: Chapter 1 (Physical AI fundamentals), calculus (derivatives, chain rule), linear algebra (matrices, vectors, matrix multiplication), basic programming in Python.

---

## 2.1 Joint Spaces and Configuration Spaces

When you look at a robot arm, you see physical links moving through space. But to control that arm mathematically, you need a way to represent its configuration—not where it is in the world, but what shape it takes. This distinction leads us to two fundamental concepts: joint space and configuration space.

### Understanding the Space of Robot Configurations

Imagine a simple 2-link planar arm, like a bent arm reaching forward. Each joint can rotate, and together the two joint angles determine the arm's entire shape. If joint 1 is at 30 degrees and joint 2 is at 45 degrees, that's a specific configuration. Change either angle and you get a different shape. The space of all possible pairs (q1, q2) is called **joint space**, denoted mathematically as the set of all joint angle combinations that the robot can physically achieve.

Now consider the Unitree G1 humanoid robot. Each arm has 6 degrees of freedom: three at the shoulder, one at the elbow, and two at the wrist. That means the joint space is 6-dimensional—impossible to visualize directly, but mathematically straightforward. A configuration is simply a vector q = [q1, q2, q3, q4, q5, q6]^T where each element is the position of one joint.

The **configuration space** (or C-space) extends this idea to account for physical constraints. Joints have limits—your shoulder can't rotate 360 degrees without something breaking. The C-space is the subset of joint space where all these constraints are satisfied. For revolute joints with no constraints, C-space is a torus (like the surface of a donut, but n-dimensional). With limits, it becomes a bounded region shaped like a hyper-rectangle cut by additional constraints from self-collision avoidance.

[DIAGRAM: 2D visualization showing a square representing joint space, with a shaded inner region representing the configuration space after applying joint limits. Labels show axes as q1 and q2 (joint angles), with boundary lines marked "Joint 1 limit" and "Joint 2 limit".]

### Workspace: Where Can the Robot Reach?

While joint space describes internal configurations, **workspace** describes what the robot can accomplish in the external world. Specifically, the workspace is the set of all positions the end-effector can reach. For the Unitree G1 arm with approximately 0.8 meters of reach, this forms roughly a sphere of radius 0.8 meters centered at the shoulder.

There are two important distinctions in workspace analysis. The **reachable workspace** includes all points the end-effector can reach, regardless of orientation—you just need to touch that point somehow. The **dexterous workspace** is smaller: points you can reach with any orientation you want. Near the boundary of the reachable workspace, you typically cannot achieve arbitrary orientations; the arm is fully extended and has "used up" its degrees of freedom just to position the hand.

> **Foundations Box**: The relationship between C-space and workspace is not one-to-one. Multiple configurations in C-space can map to the same end-effector position (consider the elbow-up and elbow-down poses that reach the same point). This many-to-one mapping is why inverse kinematics has multiple solutions, while forward kinematics is always unique.

### Singularities: When Robots Lose Mobility

A **singularity** occurs when a robot temporarily loses the ability to move in certain directions, even though it still has the same number of joints. Mathematically, singularities happen when the Jacobian matrix—which maps joint velocities to end-effector velocities—drops in rank. At these configurations, the robot becomes temporarily "stuck" in certain directions.

Consider the 2-link arm when it becomes fully extended. With both links pointing in a straight line, you cannot produce motion perpendicular to that line without bending the arm first. The Jacobian becomes singular, and the joint velocities required to produce certain Cartesian velocities become infinitely large. This is why robots move slowly and carefully near singularities—the control system must avoid demanding impossible motions.

The Unitree G1 has several singularity configurations to avoid. The wrist singularity occurs when the two wrist joints align, losing one degree of freedom. Shoulder singularities happen when the arm is fully extended in certain directions. Good motion planning avoids these configurations or uses null-space projection to maneuver through them carefully.

> **Gotchas**: Many beginners assume singularities only occur at workspace boundaries. In reality, internal singularities can appear anywhere in the workspace. For a 6-DOF arm, the wrist singularity occurs when the wrist axes become parallel—an internal configuration that can trap unwary controllers.

### Summary and Key Takeaways

- Joint space (q-space) is the n-dimensional space of joint positions; for the Unitree G1's 6-DOF arm, this is R^6.
- Configuration space (C-space) constrains joint space with physical limits and collision avoidance.
- Workspace describes reachable end-effector positions; dexterous workspace additionally requires arbitrary orientation.
- Singularities occur when the Jacobian loses rank, causing unbounded joint velocities for some Cartesian motions.
- Multiple C-space configurations can map to the same workspace position, leading to multiple IK solutions.

---

## 2.2 Forward Kinematics

If you know all the joint angles of a robot, where is its hand? This is the forward kinematics problem, and it's the foundation for everything else we do with robot arms. The key insight is that we can compute the end-effector pose by chaining together simple transformations from link to link.

### Homogeneous Transformations: Combining Rotation and Translation

Every link in a robot has its own coordinate frame. The base frame sits at the robot's base; the end-effector frame sits at the gripper. Between any two consecutive frames, we have a rotation and a translation. The elegant solution is to combine both into a single 4x4 matrix called a **homogeneous transformation**:

$$
T = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

Here, R is a 3x3 rotation matrix and t is a 3x1 translation vector. The magic is that we can multiply these matrices to compose transformations. If frame B is at pose T_B relative to frame A, and frame C is at pose T_C relative to frame B, then frame C relative to frame A is simply T_A^C = T_A^B * T_B^C.

[DIAGRAM: Chain of coordinate frames showing base (frame 0), intermediate frames (1, 2, 3), and end-effector frame (4). Arrows between frames show transformation matrices T01, T12, T23, T34. Final arrow shows T04 = T01 * T12 * T23 * T34.]

### The Denavit-Hartenberg Convention

Assigning coordinate frames to every link would be messy without a standard approach. The **Denavit-Hartenberg (DH) convention** provides a systematic method using just four parameters per joint:

- **theta_i**: The joint angle (rotation about z_{i-1})
- **d_i**: The link offset (translation along z_{i-1})
- **a_i**: The link length (translation along x_i)
- **alpha_i**: The link twist (rotation about x_i)

The transformation from frame i-1 to frame i is:

$$
T_i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\ 0 & \sin\alpha_i & \cos\alpha_i & d_i \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

For a robot with n joints, we compute n such matrices and multiply them together. The end-effector pose relative to the base is T_ee = T_1 * T_2 * ... * T_n.

> **Foundations Box**: There are actually two DH conventions: "standard" (where the frame is attached to the joint's input) and "modified" (where the frame is attached to the joint's output). These differ in whether the frame transformation uses pre-multiplication or post-multiplication. Always document which convention you're using!

### Computing Forward Kinematics for the Unitree G1 Arm

The Unitree G1 has 6-DOF arms. Let me show you how to implement forward kinematics in Python. We'll define a function to compute each DH transform and chain them together.

```python
import numpy as np

def dh_transform(theta, d, a, alpha):
    """
    Compute Denavit-Hartenberg transformation matrix.

    Parameters:
    - theta: joint angle (radians)
    - d: link offset
    - a: link length
    - alpha: link twist (radians)

    Returns: 4x4 homogeneous transformation matrix
    """
    ct = np.cos(theta)
    st = np.sin(theta)
    ca = np.cos(alpha)
    sa = np.sin(alpha)

    T = np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])
    return T

def forward_kinematics_g1(joint_angles):
    """
    Compute forward kinematics for Unitree G1 arm.

    DH parameters for G1 arm (approximate):
    Joint 1 (shoulder abduction/adduction): theta1, d=0, a=0, alpha=-pi/2
    Joint 2 (shoulder flexion/extension): theta2, d=0, a=0.1, alpha=pi/2
    Joint 3 (shoulder rotation): theta3, d=0, a=0.1, alpha=-pi/2
    Joint 4 (elbow flexion): theta4, d=0, a=0.2, alpha=-pi/2
    Joint 5 (wrist deviation): theta5, d=0, a=0, alpha=pi/2
    Joint 6 (wrist rotation): theta6, d=0.1, a=0, alpha=0
    """
    # Extract joint angles
    q1, q2, q3, q4, q5, q6 = joint_angles

    # Define DH parameters [theta, d, a, alpha]
    dh_params = [
        [q1, 0.0, 0.0, -np.pi/2],   # Joint 1: shoulder yaw
        [q2, 0.0, 0.10, np.pi/2],   # Joint 2: shoulder pitch
        [q3, 0.0, 0.10, -np.pi/2],  # Joint 3: shoulder roll
        [q4, 0.0, 0.25, -np.pi/2],  # Joint 4: elbow
        [q5, 0.0, 0.0, np.pi/2],    # Joint 5: wrist pitch
        [q6, 0.12, 0.0, 0.0],       # Joint 6: wrist roll
    ]

    # Compute cumulative transformation
    T_total = np.eye(4)
    for params in dh_params:
        T_i = dh_transform(*params)
        T_total = T_total @ T_i  # Post-multiplication for standard DH

    # Extract position and orientation
    position = T_total[:3, 3]
    rotation = T_total[:3, :3]

    return T_total, position, rotation

# Example: compute gripper position for a "ready" pose
joint_angles = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
T, pos, rot = forward_kinematics_g1(joint_angles)

print(f"End-effector position: {pos}")
print(f"End-effector orientation matrix:\n{rot}")
```

Running this code gives the gripper position when all joints are at zero. In the robot's zero configuration, the arm typically points forward, so you should see a position around (0.47, 0, 0) meters—approximately the sum of the link lengths from shoulder to wrist.

> **Gotchas**: Floating-point errors accumulate in long transform chains. For 6-DOF arms, errors are typically tiny, but for robots with more joints, you might see noticeable drift. Always test your FK implementation against known configurations (like the zero pose) before trusting it.

### Verification and Testing

Before using forward kinematics in a real control system, verify it carefully. The zero configuration test confirms that the computed position matches the geometric expectation. A symmetry test—checking that rotating joint 1 by 90 degrees and then -90 degrees returns to the original pose—verifies your angle conventions. Finally, compare your implementation against a known-good library like KDL or IKPy to catch systematic errors.

---

## 2.3 Inverse Kinematics

If forward kinematics asks "where is my hand given my joints?", inverse kinematics asks the opposite: "what joints do I need to reach a desired hand position?" This problem is fundamentally harder because multiple solutions may exist, some solutions may not exist at all, and finding solutions efficiently is non-trivial.

### The Inverse Kinematics Problem

Formally, given a desired end-effector pose T_des in SE(3), we want to find joint angles q such that FK(q) = T_des. Unlike forward kinematics, which has a unique solution for any configuration, inverse kinematics may have:

- **No solution**: The desired pose is outside the workspace
- **One solution**: A specific configuration reaches the pose
- **Multiple solutions**: Several configurations reach the same pose
- **Infinite solutions**: In redundant configurations (more DOF than needed)

For the Unitree G1's 6-DOF arm, there can be up to 16 solutions for a given reachable pose, though joint limits typically reduce this number.

### Analytical Solutions for Simple Arms

For arms with 2 or 3 degrees of freedom, we can derive closed-form solutions using geometry. Consider a 2-DOF planar arm with link lengths L1 and L2. To reach a target (x, y), we form a triangle with sides L1, L2, and the distance r = sqrt(x^2 + y^2).

[DIAGRAM: 2-DOF planar arm showing link 1 (length L1), link 2 (length L2), target point (x, y), and distance r from origin to target. Triangle formed by the two links and r is highlighted, with angles labeled.]

The law of cosines gives us the elbow angle:

$$
\cos\theta_2 = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}
$$

Then we compute theta1 using atan2:

$$
\theta_1 = \text{atan2}(y, x) - \text{atan2}\left(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2\right)
$$

Notice that cos(theta_2) gives two possibilities: theta_2 and -theta_2. These correspond to the elbow-up and elbow-down configurations—the two major solution branches for 2-DOF arms.

```python
def analytical_ik_2dof(x, y, L1, L2):
    """
    Analytical inverse kinematics for 2-DOF planar arm.
    Returns both solutions (elbow-up and elbow-down).
    """
    r = np.sqrt(x**2 + y**2)

    # Check reachability
    if r > L1 + L2 or r < abs(L1 - L2):
        return None, None  # Target unreachable

    # Law of cosines for elbow angle
    cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1, 1)  # Numerical safety

    # Two solutions: elbow-up and elbow-down
    theta2_up = np.arccos(cos_theta2)
    theta2_down = -np.arccos(cos_theta2)

    # Compute theta1 for each case
    phi = np.arctan2(y, x)
    k1 = L1 + L2 * np.cos(theta2_up)
    k2_up = L2 * np.sin(theta2_up)
    theta1_up = phi - np.arctan2(k2_up, k1)

    k1 = L1 + L2 * np.cos(theta2_down)
    k2_down = L2 * np.sin(theta2_down)
    theta1_down = phi - np.arctan2(k2_down, k1)

    return [theta1_up, theta2_up], [theta1_down, theta2_down]

# Example: reach position (1.0, 0.5) with L1=1.0, L2=0.8
solutions = analytical_ik_2dof(1.0, 0.5, 1.0, 0.8)
print(f"Solution 1 (elbow-up): {[np.degrees(a) for a in solutions[0]]} degrees")
print(f"Solution 2 (elbow-down): {[np.degrees(a) for a in solutions[1]]} degrees")
```

For 3-DOF arms, a common approach is to first solve for the wrist position (decoupling position from orientation), then compute the wrist orientation separately. This works because the wrist joints often have intersecting axes at a common point.

### Numerical Methods for General Manipulators

For 6-DOF arms like the Unitree G1, analytical solutions are complex or impossible to derive. Instead, we use iterative numerical methods that start from an initial guess and converge toward a solution.

The core idea uses the Jacobian matrix, which relates joint velocities to end-effector velocities:

$$
\dot{x} = J(q) \dot{q}
$$

Inverting this relationship gives:

$$
\dot{q} = J^+ \dot{x}
$$

where J^+ is the Moore-Penrose pseudo-inverse. For a small step, we can approximate the pose error as delta_x, then compute the joint correction:

$$
q_{k+1} = q_k + \alpha J^+ e
$$

where e is the error between current and desired pose, and alpha is a step size.

> **Foundations Box**: The Jacobian pseudo-inverse J^+ = J^T (J J^T)^(-1) provides the minimum-norm solution—the smallest joint motion that achieves the desired Cartesian motion. This is important for smooth, efficient robot trajectories.

The basic Jacobian method fails near singularities because J J^T becomes nearly singular, causing enormous joint velocities. **Damped Least Squares (DLS)** fixes this by adding regularization:

$$
J^+ = J^T (J J^T + \lambda^2 I)^(-1)
$$

The damping parameter lambda trades off between accuracy (small lambda) and stability near singularities (large lambda).

```python
def numerical_ik(target_pose, initial_q, dh_func, max_iterations=100,
                 lambda_damping=0.1, tolerance=1e-4):
    """
    Damped Least Squares inverse kinematics.

    Parameters:
    - target_pose: 4x4 desired end-effector transformation
    - initial_q: starting joint configuration
    - dh_func: function that computes FK given joint angles
    - lambda_damping: regularization parameter
    - tolerance: convergence threshold
    """
    q = np.array(initial_q)

    for iteration in range(max_iterations):
        # Compute current pose and error
        T_current, _, _ = dh_func(q)

        # Pose error (using small angle approximation)
        error = np.zeros(6)
        error[:3] = target_pose[:3, 3] - T_current[:3, 3]  # Position error
        rotation_error = target_pose[:3, :3] @ T_current[:3, :3].T
        # Convert rotation matrix to axis-angle
        rot_trace = np.trace(rotation_error)
        if rot_trace > 2.9:  # Nearly identity
            error[3:] = np.zeros(3)
        else:
            axis_angle = rotation_matrix_to_axis_angle(rotation_error)
            error[3:] = axis_angle

        # Check convergence
        if np.linalg.norm(error) < tolerance:
            print(f"Converged in {iteration} iterations")
            return q

        # Compute Jacobian (numerical approximation)
        J = compute_jacobian(q, dh_func)

        # Damped Least Squares
        J_jtj = J @ J.T + lambda_damping**2 * np.eye(6)
        delta_q = J.T @ np.linalg.solve(J_jtj, error)

        # Update configuration
        q = q + delta_q

    print(f"Failed to converge after {max_iterations} iterations")
    return q

def compute_jacobian(q, dh_func, delta=1e-6):
    """Numerical Jacobian computation."""
    n = len(q)
    J = np.zeros((6, n))
    T_base, _, _ = dh_func(q)

    for i in range(n):
        q_plus = q.copy()
        q_plus[i] += delta
        T_plus, _, _ = dh_func(q_plus)

        # Linear velocity approximation
        J[:3, i] = (T_plus[:3, 3] - T_base[:3, 3]) / delta

        # Angular velocity approximation (simplified)
        R_error = T_plus[:3, :3] @ T_base[:3, :3].T
        rot_axis = rotation_matrix_to_axis_angle(R_error)
        J[3:, i] = rot_axis / delta

    return J
```

> **Gotchas**: Numerical IK requires a good initial guess. Starting from zero configuration for every frame is slow and may converge to local minima. Instead, use the previous solution as the initial guess—this exploits temporal coherence in trajectory tracking and dramatically improves convergence.

### Handling Real-World Challenges

Practical IK implementations must handle several complications. Joint limits are hard constraints—clamping angles after computation violates the constraints. Include limits as constraints in the optimization or use null-space projection to bias solutions away from limits. Unreachable targets require fallback behavior: either return the closest reachable configuration or signal failure to the motion planner. Multiple solutions require selection criteria—choose the solution closest to the current configuration for smooth motion, or the solution that maximizes manipulability to avoid singularities.

---

## 2.4 Lagrangian Dynamics

Forward and inverse kinematics tell us where the robot is and where we want it to go. But how do we get there? That requires dynamics—understanding how forces and torques produce acceleration. The Lagrangian formulation provides an elegant, energy-based approach to deriving equations of motion.

### The Lagrangian Framework

Instead of working with forces directly, the Lagrangian approach uses energies. The **Lagrangian** L is the difference between kinetic energy T and potential energy V:

$$
L = T - V
$$

The **Euler-Lagrange equation** relates the Lagrangian to generalized forces (like joint torques):

$$
\frac{d}{dt}\left(\frac{\partial L}{\partial \dot{q}_i}\right) - \frac{\partial L}{\partial q_i} = \tau_i
$$

This single equation generates complete equations of motion for any mechanical system—no free body diagrams, no constraint forces, no vector calculus spaghetti.

### Kinetic Energy and the Mass Matrix

For a robot with n joints, the kinetic energy is:

$$
T = \frac{1}{2} \dot{q}^T M(q) \dot{q}
$$

where M(q) is the symmetric positive-definite **mass matrix** (also called the inertia matrix). The mass matrix depends on configuration because each link's velocity depends on all joint angles—rotate the shoulder and the entire arm's center of mass shifts.

Computing M(q) requires summing contributions from each link. For link i with mass m_i and inertia tensor I_i about its center of mass:

$$
M_{ij} = \sum_k \left( m_k J_{v_k}^T J_{v_k} + J_{\omega_k}^T I_k J_{\omega_k} \right)_{ij}
$$

where J_{v_k} and J_{\omega_k} are the Jacobian columns mapping joint velocities to linear and angular velocities of link k's center of mass.

### Potential Energy and Gravity

Gravitational potential energy is simply:

$$
V = \sum_i m_i g h_i
$$

where h_i is the height of link i's center of mass above some reference (typically the floor). Unlike kinetic energy, potential energy depends only on configuration, not on velocity.

### The Canonical Robot Dynamics Equation

Applying the Euler-Lagrange equation to the Lagrangian L = T - V yields the standard form of robot dynamics:

$$
M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + g(q) = \tau
$$

where:
- M(q) is the n x n mass matrix
- C(q, dot{q}) is the n x n Coriolis/centrifugal matrix
- g(q) is the n x 1 gravity vector
- tau is the n x 1 joint torque vector

The matrix C(q, dot{q}) deserves special attention. It contains two types of velocity-dependent terms: centrifugal terms (proportional to dot{q}_i^2) and Coriolis terms (proportional to dot{q}_i dot{q}_j). Together, they account for how motion in one joint affects forces in others.

> **Foundations Box**: The Coriolis matrix is not unique—multiple C matrices can satisfy the dynamics equation. The standard construction uses Christoffel symbols derived from the mass matrix: C_ij = sum_k Gamma_ijk dot{q}_k where Gamma_ijk = (partial M_ij / partial q_k + partial M_ik / partial q_j - partial M_jk / partial q_i) / 2.

### Step-by-Step Derivation: The 2-Link Planar Arm

Let me walk through the Lagrangian derivation for a 2-link planar arm. This builds intuition for the structure before tackling 6-DOF systems.

[DIAGRAM: Two-link planar arm with link 1 (length L1, mass m1) and link 2 (length L2, mass m2). Angles q1 and q2 measured from horizontal. Centers of mass at distances Lc1 and Lc2 from each joint.]

For simplicity, assume point masses at the centers of mass and neglect link lengths beyond where mass is concentrated.

**Kinetic energy of link 1:**
$$
T_1 = \frac{1}{2} m_1 (L_{c1} \dot{q}_1)^2
$$

**Kinetic energy of link 2:**
Link 2's velocity has two components: rotation about joint 1 and rotation about joint 2.
$$
T_2 = \frac{1}{2} m_2 \left[ (L_1 \dot{q}_1)^2 + (L_{c2} (\dot{q}_1 + \dot{q}_2))^2 + 2 L_1 L_{c2} \dot{q}_1 (\dot{q}_1 + \dot{q}_2) \cos q_2 \right]
$$

**Total kinetic energy:**
Combining and collecting terms by dot{q}^T M(q) dot{q}:
$$
T = \frac{1}{2} \dot{q}^T \begin{bmatrix} m_1 L_{c1}^2 + m_2(L_1^2 + L_{c2}^2 + 2 L_1 L_{c2} \cos q_2) & m_2(L_{c2}^2 + L_1 L_{c2} \cos q_2) \\ m_2(L_{c2}^2 + L_1 L_{c2} \cos q_2) & m_2 L_{c2}^2 \end{bmatrix} \dot{q}
$$

**Potential energy:**
$$
V = -m_1 g L_{c1} \cos q_1 - m_2 g (L_1 \cos q_1 + L_{c2} \cos(q_1 + q_2))
$$

**Lagrangian:** L = T - V

**Applying Euler-Lagrange for q1:**
After substantial algebra (which we would do symbolically in practice), we get:
$$
(m_1 L_{c1}^2 + m_2 L_1^2 + m_2 L_{c2}^2 + 2 m_2 L_1 L_{c2} \cos q_2) \ddot{q}_1 + (m_2 L_{c2}^2 + m_2 L_1 L_{c2} \cos q_2) \ddot{q}_2 - 2 m_2 L_1 L_{c2} \sin q_2 \dot{q}_1 \dot{q}_2 - m_2 L_1 L_{c2} \sin q_2 \dot{q}_2^2 + (m_1 L_{c1} + m_2 L_1) g \sin q_1 + m_2 g L_{c2} \sin(q_1 + q_2) = \tau_1
$$

This looks complex, but the pattern is clear: each term belongs to M(q), C(q, dot{q}), or g(q). The symbolic derivation clarifies the structure; numerical implementation just evaluates the matrices.

> **Gotchas**: Gravity terms are easy to get wrong with sign errors. The potential energy V increases with height, so g(q) = partial V / partial q. If V = mgh with h positive upward, then the gravity term in the dynamics equation is negative (pulling down). Always verify with a simple 1-DOF pendulum: tau = mgl sin(q) should produce the correct swinging behavior.

### Properties of Robot Dynamics

The dynamics equation has several important properties exploited in control design:

1. **Positive definiteness**: M(q) is always symmetric positive-definite, meaning it has a well-defined inverse and can be treated like a mass.

2. **Skew-symmetry**: The matrix dot{M} - 2C is skew-symmetric. This property underlies passivity proofs for robot controllers.

3. **Linearity in parameters**: The dynamics can be written as Y(q, dot{q}, q_ddot) theta = tau, where theta is a vector of physical parameters (masses, inertias). This linear form enables adaptive control.

4. **Computational complexity**: Computing M(q) naively is O(n^3). Efficient algorithms reduce this to O(n^2) or O(n) using recursive methods.

---

## 2.5 Newton-Euler Formulation (Optional/Advanced)

The Lagrangian approach is elegant and provides deep insight into robot dynamics. However, it's computationally expensive—computing the mass matrix requires O(n^3) operations for an n-joint arm. For real-time control at 500 Hz on the Unitree G1, we need something faster.

### The Recursive Newton-Euler Algorithm

The **Newton-Euler formulation** computes dynamics in linear time O(n) using two passes through the kinematic chain:

1. **Forward pass**: Compute angular velocities, angular accelerations, and linear accelerations for each link, propagating from base to end-effector.

2. **Backward pass**: Compute forces and torques at each joint, propagating from end-effector back to the base.

For each link i, the forward pass computes:

$$
\begin{aligned}
\omega_i &= R_{i-1}^i \omega_{i-1} + \dot{q}_i z_{i-1} \\
\alpha_i &= R_{i-1}^i \alpha_{i-1} + \ddot{q}_i z_{i-1} + \omega_{i-1} \times \dot{q}_i z_{i-1} \\
a_i &= R_{i-1}^i a_{i-1} + \alpha_i \times p_i^{i-1} + \omega_i \times (\omega_i \times p_i^{i-1})
\end{aligned}
$$

where R is rotation, p is position, omega is angular velocity, alpha is angular acceleration, and a is linear acceleration.

The backward pass computes forces and torques:

$$
\begin{aligned}
f_i &= R_i^{i+1} f_{i+1} + m_i (a_i + g) \\
n_i &= R_i^{i+1} n_{i+1} + p_{com}^i \times f_i + I_i \alpha_i + \omega_i \times (I_i \omega_i)
\end{aligned}
$$

The joint torque is the projection of n_i onto the joint axis.

[DIAGRAM: Flowchart showing forward pass (base to tip) computing velocities and accelerations, and backward pass (tip to base) computing forces and torques. Each link shows the computed quantities.]

### Computational Efficiency Comparison

| Method | Complexity | Memory | Cache Efficiency |
|--------|------------|--------|------------------|
| Lagrangian (naive) | O(n^3) | O(n^2) | Poor |
| Lagrangian (efficient) | O(n^2) | O(n^2) | Moderate |
| Newton-Euler | O(n) | O(n) | Excellent |

For a 6-DOF arm like the Unitree G1, this difference might seem modest. But for humanoid robots with 20+ joints, or for algorithms that need the full mass matrix (like model predictive control), the Newton-Euler approach becomes essential.

### When to Use Each Approach

Use the **Lagrangian approach** when:
- Deriving symbolic equations for analysis
- Teaching dynamics fundamentals
- Computing the mass matrix explicitly (needed for some controllers)
- The robot has few degrees of freedom

Use **Newton-Euler** when:
- Implementing real-time control at high frequency
- Computing inverse dynamics (torques from motions)
- The robot has many degrees of freedom
- Memory and computation are constrained

> **Foundations Box**: Most modern robot control libraries use Newton-Euler internally for efficiency. Pinocchio, Drake, and MuJoCo all implement recursive algorithms. Understanding Newton-Euler helps you debug performance issues and design custom controllers that match the library's computational model.

---

## 2.6 Simulation of Multi-Body Dynamics

Theory guides us, but robots exist in the physical world. Before deploying control algorithms on the Unitree G1, we validate them in simulation. This section covers the physics engines that power robot simulation and the practical considerations for meaningful results.

### Physics Engines for Robotics

Three engines dominate robotics simulation:

**MuJoCo** (Multi-Joint Dynamics with Contact) uses optimization-based contact dynamics with a convex formulation. It's the gold standard for manipulation and contact-rich tasks. The native format is MJCF (XML), though URDF import is supported.

**Gazebo Harmonic** (the latest ROS-integrated version) provides a complete simulation environment with sensor models, plugin systems, and tight ROS 2 integration. The native format is SDF (Simulation Description Format), and it can use multiple physics backends.

**PyBullet** offers a Python-native interface to the Bullet physics engine. It's popular for machine learning research because of its simplicity and easy installation via pip.

[DIAGRAM: Pipeline diagram showing robot model (URDF/MJCF) flowing into physics engine, which computes dynamics at each timestep, producing joint states that feed into control algorithms, which output torques back to the physics engine.]

### Building a Simulation Model

The Unitree G1's dynamics can be simulated in MuJoCo by defining an MJCF model:

```xml
<mujoco model="unitree_g1_arm">
  <compiler inertiafromgeom="true" />

  <worldbody>
    <!-- Base link -->
    <body name="shoulder" pos="0 0 0.5">
      <geom type="cylinder" size="0.05 0.1" />
      <joint name="j1_yaw" type="hinge" axis="0 0 1" />

      <!-- Upper arm -->
      <body name="upper_arm" pos="0.1 0 0">
        <geom type="box" size="0.02 0.02 0.1" mass="1.0" />
        <joint name="j2_pitch" type="hinge" axis="1 0 0" />

        <!-- Forearm -->
        <body name="forearm" pos="0 0 0.2">
          <geom type="box" size="0.015 0.015 0.25" mass="0.8" />
          <joint name="j3_roll" type="hinge" axis="1 0 0" />

          <!-- Continue for remaining joints... -->
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="j1_motor" joint="j1_yaw" />
    <motor name="j2_motor" joint="j2_pitch" />
    <!-- Remaining motors... -->
  </actuator>
</mujoco>
```

```python
import mujoco
import numpy as np

# Load the model
model = mujoco.MjModel.from_xml_path("unitree_g1_arm.xml")
data = mujoco.MjData(model)

# Run a simulation step
def simulate_step(tau):
    """Apply torques and step simulation."""
    data.ctrl[:] = tau  # Set joint torques
    mujoco.mj_step(model, data)  # Advance physics
    return data.qpos.copy(), data.qvel.copy()  # Return state

# Simulate gravity-only dynamics
print("Simulating passive dynamics under gravity...")
for _ in range(1000):
    # Zero torque - let gravity act
    q, qd = simulate_step(np.zeros(6))

print(f"Final joint positions: {np.degrees(q)} degrees")
print(f"Final joint velocities: {qd} rad/s")
```

### Integration Methods and Stability

Physics engines solve differential equations numerically. The integration method determines accuracy and stability:

**Euler integration** (explicit) is first-order accurate but unstable for stiff systems. It's rarely used for robot dynamics.

**RK4 (Runge-Kutta 4th order)** offers a good balance of accuracy and stability for most robotics applications.

**Implicit integration** solves for acceleration given forces, providing unconditional stability at the cost of more computation per timestep. MuJoCo uses implicit integration by default.

For the Unitree G1 running at 500 Hz control frequency, MuJoCo's default 1ms timestep with implicit integration provides stable, accurate dynamics.

> **Gotchas**: The sim-to-real gap is real. Contact dynamics in simulation differ significantly from reality due to friction model approximations, contact point computation, and deformation. A controller that tracks perfectly in simulation may fail on hardware. Validate with domain randomization: vary friction, damping, and mass parameters in simulation and test across the range on hardware.

### Validation Against Analytical Dynamics

A crucial verification step compares simulation output against analytical dynamics computed via Lagrangian methods. If simulation and theory disagree, there's a modeling error somewhere.

```python
def validate_simulation_against_analytical():
    """
    Compare MuJoCo simulation with analytical dynamics.

    For a 2-link arm with known parameters:
    1. Set initial configuration
    2. Apply zero torque (passive dynamics)
    3. Compare simulated acceleration with M(q)^(-1) * (-C(q, qd) - g(q))
    """
    # Analytical mass matrix (from Section 2.4)
    M = compute_mass_matrix(q, params)

    # Gravity and Coriolis terms
    g_vec = compute_gravity(q, params)
    C_term = compute_coriolis(q, qd, params)

    # Analytical acceleration: q_ddot = M^(-1) * (-C*qd - g)
    q_ddot_analytical = np.linalg.solve(M, -C_term @ qd - g_vec)

    # MuJoCo provides accelerations directly
    q_ddot_simulated = data.qacc.copy()

    # Compare
    error = np.linalg.norm(q_ddot_simulated - q_ddot_analytical)
    print(f"Acceleration error: {error:.6f} rad/s^2")

    if error < 0.01:
        print("Validation PASSED: Simulation matches analytical model")
    else:
        print("Validation FAILED: Check model parameters")
```

### Best Practices for Robot Simulation

1. **Verify model parameters**: Masses, inertias, and centers of mass must match the physical robot. CAD models provide good approximations but may need calibration.

2. **Check units**: MuJoCo uses SI units (meters, kilograms, seconds). Mixing units (centimeters instead of meters) causes physically unrealistic motion.

3. **Tune contact parameters**: Friction coefficients, contact shapes, and solver iterations significantly affect contact behavior. Calibrate against real surfaces.

4. **Use deterministic seeding**: For reproducible results, set random seeds for both physics and rendering.

5. **Profile computation time**: Ensure simulation runs faster than real-time for training applications, or matches real-time for hardware-in-the-loop testing.

### Summary and Key Takeaways

- Physics engines (MuJoCo, Gazebo, PyBullet) provide numerically integrated multi-body dynamics
- Model definition formats (MJCF, SDF, URDF) describe robot structure and properties
- Integration method (Euler, RK4, implicit) affects stability and accuracy
- Validation against analytical dynamics catches modeling errors
- Sim-to-real gap requires domain randomization and hardware validation

---

## Chapter Summary

This chapter developed the mathematical foundations for robot manipulation. We began with configuration spaces—understanding what configurations a robot can achieve and what positions it can reach. Forward kinematics maps joint angles to end-effector pose using homogeneous transformations and the Denavit-Hartenberg convention. Inverse kinematics solves the opposite problem, with analytical solutions for simple arms and numerical methods for general manipulators.

The second half introduced dynamics through the Lagrangian formulation, deriving the canonical equation M(q)q_ddot + C(q, q_dot)q_dot + g(q) = tau that underlies all robot control. We explored the Newton-Euler formulation for efficient real-time computation, and concluded with practical simulation using physics engines.

> **Key Takeaways**:
> 1. Joint space and configuration space are different—C-space accounts for physical constraints
> 2. Forward kinematics is unique and computationally efficient; inverse kinematics is non-unique and requires iteration
> 3. The Lagrangian formulation provides elegant, constraint-free dynamics equations
> 4. Newton-Euler recursion achieves O(n) complexity for real-time control
> 5. Simulation validation catches modeling errors before hardware deployment

**Check Your Understanding**:
- Can you explain why a 6-DOF arm has a 6-dimensional configuration space?
- Why does the Jacobian lose rank at singularities, and what happens to joint velocities?
- What is the difference between the standard and modified DH conventions?
- Why is the Coriolis matrix configuration-dependent?
- What causes the sim-to-real gap, and how can you mitigate it?

**Next Steps**: With kinematics and dynamics fundamentals established, Chapter 3 will cover trajectory planning—generating smooth, feasible motion plans through configuration space. Chapter 4 introduces feedback control, using the dynamics models developed here to track those trajectories accurately.

---

## Lab Exercises

**Lab 2.1** (30 min): Implement forward kinematics for a 2-DOF planar arm. Define the DH table, compute the end-effector position for various joint configurations, and visualize the arm in matplotlib.

**Lab 2.2** (45 min): Implement analytical inverse kinematics for a 2-DOF arm, handling both elbow-up and elbow-down solutions. Then implement numerical IK for a 3-DOF arm using damped least squares.

**Lab 2.3** (60 min): Create a MuJoCo simulation of a 2-link arm. Derive Lagrangian dynamics equations analytically, then compare simulated accelerations against analytical predictions to validate your model.

---

## References

- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.
- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley.
- MuJoCo Documentation: https://mujoco.readthedocs.io/
- ROS 2 Documentation: https://docs.ros.org/en/humble/
