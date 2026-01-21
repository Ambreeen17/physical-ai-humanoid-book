# Chapter 2: Kinematics and Dynamics of Robot Manipulators

**Variant**: Beginner
**Difficulty Score**: 3/10
**Estimated Reading Time**: 90 minutes

**Chapter Overview**: This chapter develops the mathematical foundations for describing and predicting robot motion. We begin with the geometry of robot configurations—how we represent where each joint is and what positions the robot can reach. From there, we derive the forward kinematics equations that map joint positions to end-effector pose, and the inverse kinematics that solve the opposite problem. We focus on a simple 2-link arm to build intuition before introducing more complex robots.

**Learning Objectives**: After completing this chapter, you will be able to explain joint space and configuration space in everyday terms, compute forward kinematics for a 2-DOF planar arm using Denavit-Hartenberg parameters, solve inverse kinematics analytically for simple arms, and visualize robot motion in simulation.

**Prerequisites**: Chapter 1 (Physical AI fundamentals), basic algebra, basic Python programming (we provide complete starter code).

---

## 2.1 Joint Spaces and Configuration Spaces

When you look at a robot arm, you see physical links moving through space. But to control that arm mathematically, you need a way to represent its configuration—not where it is in the world, but what shape it takes. This distinction leads us to two fundamental concepts: joint space and configuration space.

### Visual Analogy: What Does "Joint Space" Mean?

Think about your own arm. When you reach for a cup, your brain doesn't think in terms of x, y, z coordinates. Instead, you think about the angle of your shoulder, how much your elbow is bent, and how your wrist is oriented. These angles are your "joint positions," and the collection of all possible angle combinations is called **joint space**.

Imagine a slider control on a mixing board—each knob controls one joint angle. Joint space is like the space of all possible combinations of knob settings. For a 2-joint arm, you have two knobs; for a 6-joint arm (like the Unitree G1), you have six knobs.

### Understanding the Space of Robot Configurations

Consider a simple 2-link planar arm, like a bent arm reaching forward on a flat table. Each joint can rotate, and together the two joint angles determine the arm's entire shape.

- If joint 1 is at 30 degrees and joint 2 is at 45 degrees, that's a specific configuration.
- Change either angle and you get a different shape.

The space of all possible pairs (q1, q2) is called **joint space**, denoted mathematically as the set of all joint angle combinations that the robot can physically achieve.

> **Why This Matters**: Understanding joint space is essential because robot controllers actually command joint angles, not hand positions. When you tell a robot to "reach here," the computer translates that into specific angle commands for each motor.

### Configuration Space: Adding Real-World Constraints

The **configuration space** (or C-space) adds physical reality to joint space. In an ideal world, joints could rotate forever in any direction. In the real world:

- Joints have limits (your shoulder can't rotate 360 degrees without something breaking)
- Links can't pass through each other (self-collision)
- Links can't pass through the environment (obstacles)

The C-space is the subset of joint space where all these constraints are satisfied. For our 2-link arm:

- Joint 1 might be limited to -90 to +90 degrees
- Joint 2 might be limited to -150 to +150 degrees

The configuration space is then a rectangle within the larger joint space rectangle.

[DIAGRAM: Simple 2D visualization showing a square representing joint space, with a shaded inner region representing the configuration space. The x-axis is labeled "Joint 1 Angle (degrees)" and the y-axis is labeled "Joint 2 Angle (degrees)". Boundary lines show "Joint 1 Limit" at -90 and +90 degrees, and "Joint 2 Limit" at -150 and +150 degrees.]

### Workspace: Where Can the Robot Reach?

While joint space describes internal configurations, **workspace** describes what the robot can accomplish in the external world. Specifically, the workspace is the set of all positions the end-effector can reach.

For our 2-link arm with link lengths L1 and L2:
- The reachable workspace is the area between a circle of radius (L1 + L2) and a circle of radius |L1 - L2|
- This forms an annular region (a donut shape) on the table

**Key insight**: Many different joint configurations can reach the same point! This is called redundancy, and it's why inverse kinematics (coming up next) can have multiple solutions.

> **Foundations Box**: The relationship between C-space and workspace is not one-to-one. Multiple configurations in C-space can map to the same end-effector position (consider the elbow-up and elbow-down poses that reach the same point). This many-to-one mapping is why inverse kinematics has multiple solutions, while forward kinematics is always unique.

### Singularities: When Robots Lose Mobility

A **singularity** occurs when a robot temporarily loses the ability to move in certain directions, even though it still has the same number of joints. This happens when the robot is "stretched out straight."

Consider our 2-link arm when it becomes fully extended (both links pointing in the same direction):
- You cannot produce motion perpendicular to that line without bending the arm first
- The robot becomes temporarily "stuck" in certain directions

> **Intuition**: Think about trying to write with a pen held straight out from your body. It's hard to move the pen in certain directions without rotating your shoulder first. That's a singularity!

### Summary and Key Takeaways

- **Joint space** is the space of all possible joint angle combinations
- **Configuration space** (C-space) is joint space minus physical constraints like joint limits
- **Workspace** is the set of all positions the end-effector can reach in the world
- **Singularities** are configurations where the robot loses mobility in certain directions
- Multiple C-space configurations can map to the same workspace position

---

## 2.2 Forward Kinematics

If you know all the joint angles of a robot, where is its hand? This is the **forward kinematics** problem, and it's the foundation for everything else we do with robot arms.

### The Core Idea: Chaining Simple Transformations

The key insight is this: a robot arm is just a chain of simple movements. Each joint adds one transformation—either a rotation or a translation—to the previous one. By chaining these together, we can compute where the end-effector ends up.

Think of it like giving directions:
1. Start at the origin (shoulder)
2. Rotate by angle q1 about the shoulder axis
3. Move forward by length L1 (upper arm)
4. Rotate by angle q2 about the elbow axis
5. Move forward by length L2 (forearm)

The final position is your hand!

### Homogeneous Transformations: Combining Rotation and Translation

Every link in a robot has its own coordinate frame (like a mini coordinate system attached to that link). The base frame sits at the robot's base; the end-effector frame sits at the gripper.

Between any two consecutive frames, we have:
- A rotation (turning)
- A translation (moving)

The elegant solution is to combine both into a single 4x4 matrix called a **homogeneous transformation**:

$$
T = \begin{bmatrix} R & t \\ 0 & 1 \end{bmatrix} = \begin{bmatrix} r_{11} & r_{12} & r_{13} & t_x \\ r_{21} & r_{22} & r_{23} & t_y \\ r_{31} & r_{32} & r_{33} & t_z \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

Where:
- R is a 3x3 rotation matrix (describes how the frame is oriented)
- t is a 3x1 translation vector (describes where the frame is positioned)

The magic is that we can multiply these matrices to compose transformations:
- If frame B is at pose T_B relative to frame A
- And frame C is at pose T_C relative to frame B
- Then frame C relative to frame A is: T_A^C = T_A^B * T_B^C

[DIAGRAM: Chain of coordinate frames showing base (frame 0), intermediate frame 1, and end-effector frame 2. Arrows between frames show transformation matrices T01 and T12. A final arrow shows T02 = T01 * T12.]

### The Denavit-Hartenberg Convention

Assigning coordinate frames to every link would be messy without a standard approach. The **Denavit-Hartenberg (DH) convention** provides a systematic method using just four parameters per joint.

For a 2-DOF planar arm, here's what each parameter means:

| Parameter | Symbol | Meaning for Our Arm |
|-----------|--------|---------------------|
| Joint angle | theta_i | How much joint i has rotated from its zero position |
| Link length | a_i | Length of link i (distance between joint i and joint i+1) |
| Link offset | d_i | For planar arms, this is usually 0 (we work on a flat plane) |
| Link twist | alpha_i | For planar arms on a flat table, this is 0 |

### Step-by-Step DH Table Construction for a 2-DOF Arm

Let's build the DH table step by step for our 2-link planar arm.

**Step 1: Draw the arm and label everything**

```
                    (x2, y2)  End-effector
                        /
                       / Link 2 (length L2)
                      /
    Joint 2 ---------/
                     \
                      \ Link 1 (length L1)
                       \
                        \ Joint 1 ---- Base (x0, y0)
```

**Step 2: Place coordinate frames**

- Frame 0: Base frame at Joint 1 (origin)
- Frame 1: At Joint 2, after Link 1
- Frame 2: At end-effector, after Link 2

**Step 3: Complete the DH table**

| Joint | theta (radians) | d (meters) | a (meters) | alpha (radians) |
|-------|-----------------|------------|------------|-----------------|
| 1 | q1 (variable) | 0 | L1 | 0 |
| 2 | q2 (variable) | 0 | L2 | 0 |

**Key observation**: For a planar arm on a flat table, alpha = 0 because all links lie in the same plane.

### Computing the Transformation Matrix

For our simplified case (alpha = 0), the DH transformation matrix simplifies to:

$$
T_i = \begin{bmatrix} \cos\theta_i & -\sin\theta_i & 0 & a_i\cos\theta_i \\ \sin\theta_i & \cos\theta_i & 0 & a_i\sin\theta_i \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 1 \end{bmatrix}
$$

### Complete Working Code for Forward Kinematics

Here's a complete, well-commented implementation you can run right away:

```python
import numpy as np
import matplotlib.pyplot as plt

def dh_transform(theta, d, a, alpha):
    """
    Compute Denavit-Hartenberg transformation matrix.

    This function creates a 4x4 matrix that represents the transformation
    from one coordinate frame to the next.

    Parameters:
    - theta: joint angle in radians (rotation about z-axis)
    - d: link offset in meters (translation along z-axis)
    - a: link length in meters (translation along x-axis)
    - alpha: link twist in radians (rotation about x-axis)

    Returns: 4x4 homogeneous transformation matrix
    """
    ct = np.cos(theta)  # cosine of theta
    st = np.sin(theta)  # sine of theta
    ca = np.cos(alpha)  # cosine of alpha
    sa = np.sin(alpha)  # sine of alpha

    # Build the transformation matrix
    T = np.array([
        [ct, -st * ca, st * sa, a * ct],
        [st, ct * ca, -ct * sa, a * st],
        [0, sa, ca, d],
        [0, 0, 0, 1]
    ])
    return T


def forward_kinematics_2dof(joint_angles, L1=0.3, L2=0.25):
    """
    Compute forward kinematics for a 2-DOF planar arm.

    Parameters:
    - joint_angles: [q1, q2] in radians
    - L1: length of link 1 in meters (default 0.3 m = 30 cm)
    - L2: length of link 2 in meters (default 0.25 m = 25 cm)

    Returns:
    - T_total: combined transformation from base to end-effector
    - position: [x, y] end-effector position
    - joint_positions: list of [x, y] for each joint
    """
    q1, q2 = joint_angles

    # Define DH parameters for each joint
    # [theta, d, a, alpha]
    dh_params = [
        [q1, 0.0, L1, 0.0],  # Joint 1
        [q2, 0.0, L2, 0.0],  # Joint 2
    ]

    # Track positions for visualization
    joint_positions = [[0.0, 0.0]]  # Start at base

    # Compute cumulative transformation
    T_total = np.eye(4)  # Start with identity matrix

    for params in dh_params:
        T_i = dh_transform(*params)
        T_total = T_total @ T_i  # Matrix multiplication

        # Extract position of this joint for visualization
        pos = T_total[:2, 3]
        joint_positions.append([pos[0], pos[1]])

    # Extract final end-effector position
    position = T_total[:2, 3]

    return T_total, position, joint_positions


def visualize_arm(joint_positions, L1, L2, title="2-DOF Arm Forward Kinematics"):
    """
    Draw the robot arm using matplotlib.
    """
    plt.figure(figsize=(8, 8))

    # Extract x and y coordinates
    x_coords = [p[0] for p in joint_positions]
    y_coords = [p[1] for p in joint_positions]

    # Plot arm links
    plt.plot(x_coords, y_coords, 'b-', linewidth=3, label='Arm links')

    # Plot joints
    plt.scatter(x_coords, y_coords, c='red', s=100, zorder=5, label='Joints')

    # Plot base
    plt.scatter([0], [0], c='green', s=200, marker='s', zorder=5, label='Base')

    # Plot end-effector
    plt.scatter([x_coords[-1]], [y_coords[-1]], c='purple', s=150,
                marker='*', zorder=5, label='End-effector')

    # Draw workspace circle (max reach)
    theta_circle = np.linspace(0, 2*np.pi, 100)
    plt.plot(L1 + L2 * np.cos(theta_circle), L1 + L2 * np.sin(theta_circle),
             'k--', alpha=0.3, label='Max reach')

    # Formatting
    plt.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
    plt.axvline(x=0, color='gray', linestyle='-', linewidth=0.5)
    plt.xlabel('X position (meters)')
    plt.ylabel('Y position (meters)')
    plt.title(title)
    plt.legend(loc='upper right')
    plt.axis('equal')
    plt.grid(True, alpha=0.3)
    plt.show()


# ============================================
# MAIN PROGRAM - Run this to see FK in action!
# ============================================

if __name__ == "__main__":
    # Set link lengths (in meters)
    L1 = 0.3  # 30 cm
    L2 = 0.25  # 25 cm

    # Test case 1: Both joints at 0 degrees (arm pointing right)
    q = [0.0, 0.0]
    T, pos, joints = forward_kinematics_2dof(q, L1, L2)
    print(f"Configuration: q1={np.degrees(q[0]):.1f}°, q2={np.degrees(q[1]):.1f}°")
    print(f"End-effector position: x={pos[0]:.3f}m, y={pos[1]:.3f}m")
    print(f"Expected: x={L1+L2:.3f}m, y=0.000m")
    print()

    # Test case 2: First joint at 90 degrees (arm pointing up)
    q = [np.pi/2, 0.0]
    T, pos, joints = forward_kinematics_2dof(q, L1, L2)
    print(f"Configuration: q1={np.degrees(q[0]):.1f}°, q2={np.degrees(q[1]):.1f}°")
    print(f"End-effector position: x={pos[0]:.3f}m, y={pos[1]:.3f}m")
    print(f"Expected: x=0.000m, y={L1+L2:.3f}m")
    print()

    # Test case 3: L-shaped arm (q1=90°, q2=90°)
    q = [np.pi/2, np.pi/2]
    T, pos, joints = forward_kinematics_2dof(q, L1, L2)
    print(f"Configuration: q1={np.degrees(q[0]):.1f}°, q2={np.degrees(q[1]):.1f}°")
    print(f"End-effector position: x={pos[0]:.3f}m, y={pos[1]:.3f}m")
    print(f"Expected: x={L2:.3f}m, y={L1:.3f}m")
    print()

    # Visualize the L-shaped arm
    visualize_arm(joints, L1, L2, "2-DOF Arm: q1=90°, q2=90°")
```

### Understanding the Output

When you run this code, you should see:

1. **Configuration 1** (both angles 0): The arm points straight along the x-axis. The end-effector is at (0.55, 0.000) meters—the sum of both link lengths.

2. **Configuration 2** (q1=90 degrees): The arm points straight up along the y-axis. The end-effector is at (0.000, 0.55) meters.

3. **Configuration 3** (L-shape): The arm forms an "L" shape, with the end-effector at the corner of the L.

### Verification: Why This Matters

Before trusting forward kinematics in a real robot control system, always verify it:

1. **Zero configuration test**: When all joints are at 0, the arm should point along the positive x-axis.

2. **Symmetry test**: Rotating joint 1 by +90 degrees then -90 degrees should return to the original pose.

3. **Link length test**: The maximum reach should equal L1 + L2.

> **Gotchas**: Floating-point errors accumulate in long transform chains. For a 2-DOF arm, errors are tiny. But for robots with many joints, you might see small drift. Always test your FK implementation against known configurations!

### Summary and Key Takeaways

- Forward kinematics answers: "Given all joint angles, where is the end-effector?"
- We compute this by chaining together simple transformations (DH matrices)
- The DH convention provides a systematic 4-parameter method for any robot
- Visualization helps verify your implementation is correct
- Always test against known configurations before trusting your code

---

## 2.3 Inverse Kinematics

Now we ask the opposite question: **if I want my hand at a specific position, what joint angles do I need?**

This is the **inverse kinematics** problem, and it's fundamentally harder than forward kinematics because:

1. Multiple solutions may exist (different arm poses can reach the same point)
2. Some positions might be unreachable
3. Finding solutions efficiently is non-trivial

### The Inverse Kinematics Problem

Formally: given a desired end-effector position (x, y), find joint angles q = [q1, q2] such that FK(q) = [x, y].

For our 2-DOF arm, there are typically up to 2 solutions:
- **Elbow-up**: The elbow points upward
- **Elbow-down**: The elbow points downward

Both reach the same point but with different joint configurations!

### Geometric Solution: Using the Law of Cosines

For a 2-DOF planar arm, we can solve IK analytically using geometry. Here's the intuition:

1. Draw lines from the base to the target point and from the target point to the end of link 2
2. This forms a triangle with sides: L1, L2, and the distance r from base to target

[DIAGRAM: 2-DOF planar arm showing link 1 (length L1), link 2 (length L2), target point (x, y), and the distance r from origin to target. The triangle formed by the two links and r is highlighted.]

Using the **law of cosines**, we can find the elbow angle:

$$
\cos\theta_2 = \frac{x^2 + y^2 - L_1^2 - L_2^2}{2 L_1 L_2}
$$

Then we compute q1 using atan2:

$$
\theta_1 = \text{atan2}(y, x) - \text{atan2}\left(L_2 \sin\theta_2, L_1 + L_2 \cos\theta_2\right)
$$

Notice that cos(theta_2) gives two possibilities: +theta_2 and -theta_2. These correspond to elbow-up and elbow-down!

### Complete Working Code for Inverse Kinematics

```python
import numpy as np

def inverse_kinematics_2dof(x, y, L1, L2):
    """
    Analytical inverse kinematics for 2-DOF planar arm.

    Parameters:
    - x, y: desired end-effector position (meters)
    - L1: length of link 1 (meters)
    - L2: length of link 2 (meters)

    Returns:
    - solutions: list of [q1, q2] pairs (in radians)
    - None if target is unreachable
    """
    # Calculate distance from origin to target
    r = np.sqrt(x**2 + y**2)

    # Check if target is reachable
    # Target must be within the annular workspace
    max_reach = L1 + L2
    min_reach = abs(L1 - L2)

    if r > max_reach:
        print(f"ERROR: Target too far! Distance {r:.3f}m > max reach {max_reach:.3f}m")
        return None
    if r < min_reach:
        print(f"ERROR: Target too close! Distance {r:.3f}m < min reach {min_reach:.3f}m")
        return None

    # Step 1: Calculate theta2 using law of cosines
    # cos(theta2) = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2)
    cos_theta2 = (r**2 - L1**2 - L2**2) / (2 * L1 * L2)

    # Clamp to [-1, 1] to handle floating-point errors
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)

    # Two solutions: elbow-up and elbow-down
    theta2_up = np.arccos(cos_theta2)       # Positive angle
    theta2_down = -np.arccos(cos_theta2)    # Negative angle

    # Step 2: Calculate theta1 for each solution
    # phi = angle from origin to target
    phi = np.arctan2(y, x)

    # For elbow-up solution
    # k1 = L1 + L2 * cos(theta2)
    # k2 = L2 * sin(theta2)
    # theta1 = phi - atan2(k2, k1)
    k1_up = L1 + L2 * np.cos(theta2_up)
    k2_up = L2 * np.sin(theta2_up)
    theta1_up = phi - np.arctan2(k2_up, k1_up)

    # For elbow-down solution
    k1_down = L1 + L2 * np.cos(theta2_down)
    k2_down = L2 * np.sin(theta2_down)
    theta1_down = phi - np.arctan2(k2_down, k1_down)

    # Return both solutions
    return [
        [theta1_up, theta2_up],   # Elbow-up
        [theta1_down, theta2_down]  # Elbow-down
    ]


def visualize_ik_solutions(x, y, L1, L2, solutions):
    """
    Visualize both IK solutions.
    """
    import matplotlib.pyplot as plt

    plt.figure(figsize=(10, 5))

    for i, (label, solution) in enumerate([("Elbow-Up", solutions[0]),
                                            ("Elbow-Down", solutions[1])]):
        plt.subplot(1, 2, i+1)

        q1, q2 = solution

        # Compute joint positions
        x1 = L1 * np.cos(q1)
        y1 = L1 * np.sin(q1)
        x2 = x1 + L2 * np.cos(q1 + q2)
        y2 = y1 + L2 * np.sin(q1 + q2)

        # Draw arm
        plt.plot([0, x1, x2], [0, y1, y2], 'b-', linewidth=3)
        plt.scatter([0, x1, x2], [0, y1, y2], c='red', s=100)

        # Draw target
        plt.scatter([x], [y], c='green', s=200, marker='*')

        plt.axhline(y=0, color='gray', linestyle='-', linewidth=0.5)
        plt.axvline(x=0, color='gray', linestyle='-', linewidth=0.5)
        plt.xlabel('X (meters)')
        plt.ylabel('Y (meters)')
        plt.title(f'{label}\nq1={np.degrees(q1):.1f}°, q2={np.degrees(q2):.1f}°')
        plt.axis('equal')
        plt.grid(True, alpha=0.3)

        # Set consistent axis limits
        plt.xlim(-0.1, L1 + L2 + 0.1)
        plt.ylim(-0.3, 0.3)

    plt.tight_layout()
    plt.show()


# ============================================
# MAIN PROGRAM - Run this to see IK in action!
# ============================================

if __name__ == "__main__":
    # Set link lengths
    L1 = 0.3  # 30 cm
    L2 = 0.25  # 25 cm

    # Target position: reach forward and up
    x, y = 0.4, 0.15

    print("=" * 50)
    print("Inverse Kinematics for 2-DOF Planar Arm")
    print("=" * 50)
    print(f"Link lengths: L1={L1}m, L2={L2}m")
    print(f"Target position: ({x}m, {y}m)")
    print()

    # Compute IK
    solutions = inverse_kinematics_2dof(x, y, L1, L2)

    if solutions is not None:
        print("Found 2 solutions:")
        for i, (label, sol) in enumerate([("Elbow-Up", solutions[0]),
                                          ("Elbow-Down", solutions[1])]):
            q1, q2 = sol
            print(f"  {i+1}. {label}: q1={np.degrees(q1):.1f}°, q2={np.degrees(q2):.1f}°")

        # Visualize
        visualize_ik_solutions(x, y, L1, L2, solutions)
    else:
        print("No solution found!")

    # Test unreachable cases
    print("\n" + "=" * 50)
    print("Testing unreachable cases:")
    print("=" * 50)

    # Too far
    inverse_kinematics_2dof(0.7, 0.0, L1, L2)

    # Too close
    inverse_kinematics_2dof(0.05, 0.0, L1, L2)
```

### Understanding the Results

When you run this code:

1. **Reachable target**: The algorithm finds two solutions—elbow-up and elbow-down. Both reach the same point but have different joint angles.

2. **Unreachable targets**: The algorithm checks if the target is within the annular workspace (between min and max reach) and returns None if not.

### Summary and Key Takeaways

- Inverse kinematics answers: "Given a desired position, what joint angles do I need?"
- For 2-DOF arms, we can solve IK analytically using geometry (law of cosines)
- There are typically 2 solutions: elbow-up and elbow-down
- Always check if a target is reachable before trying to solve IK
- Different solutions have different advantages (elbow-up may avoid obstacles that elbow-down hits)

---

## 2.4 Lagrangian Dynamics (Simplified)

[OPTIONAL SECTION - SKIP ON FIRST READING]

This section introduces how robots move when we apply forces. We focus on the key results and skip the mathematical derivation.

### The Core Equation

When we apply torques to robot joints, the robot accelerates according to:

$$
M(q) \ddot{q} + C(q, \dot{q}) \dot{q} + g(q) = \tau
$$

Where:
- M(q) is the **mass matrix** (how hard it is to accelerate each joint)
- C(q, dot{q}) is the **Coriolis/centrifugal matrix** (effects of moving joints on each other)
- g(q) is the **gravity vector** (how gravity pulls on each joint)
- tau is the **joint torque** vector (what the motors apply)

### Key Insights (No Math Required!)

1. **The mass matrix depends on configuration**: It's harder to lift the arm when it's horizontal than when it's vertical (because the center of mass is further from the joint).

2. **Joints affect each other**: Moving one joint affects the forces needed at other joints (this is the Coriolis/centrifugal effect).

3. **Gravity is configuration-dependent**: The gravitational torque on each joint depends on the arm's pose.

> **Result**: The canonical robot dynamics equation is M(q)q_ddot + C(q, q_dot)q_dot + g(q) = tau. This equation underlies all modern robot control algorithms.

### Summary and Key Takeaways

- Dynamics describes how forces produce acceleration
- The mass matrix M(q) depends on configuration
- Gravity effects depend on arm pose
- This equation is the foundation for robot control (Chapter 4)

---

## 2.5 Simulation with PyBullet

[SKIP THIS SECTION - COVERED IN BEGINNER LAB]

---

## Chapter Summary

This chapter developed the mathematical foundations for robot manipulation using a 2-DOF planar arm for clarity.

**Key Concepts**:
1. **Joint space** is the space of all possible joint angle combinations
2. **Configuration space** (C-space) adds physical constraints like joint limits
3. **Workspace** is the set of all positions the end-effector can reach
4. **Forward kinematics** maps joint angles to end-effector position using DH transformations
5. **Inverse kinematics** finds joint angles to reach a desired position (multiple solutions exist)

**Key Takeaways**:
- Forward kinematics is unique and computationally efficient
- Inverse kinematics is non-unique and requires geometric or numerical methods
- For 2-DOF arms, IK has analytical solutions (elbow-up and elbow-down)
- Always verify your implementations against known configurations

**Check Your Understanding**:
- Can you explain why a 2-DOF arm has a 2-dimensional configuration space?
- What is the difference between joint space and workspace?
- Why does inverse kinematics have multiple solutions for some targets?
- What happens to the arm at a singularity?

**Next Steps**: With kinematics fundamentals established, Chapter 3 will cover trajectory planning—generating smooth, feasible motion plans through configuration space.

---

## Lab Exercises

**Lab 2.1** (30 min): **Visualize Forward Kinematics**
Implement the forward kinematics code provided in Section 2.2. Create a function that animates the arm moving through various configurations. Observe how the end-effector traces different paths.

**Lab 2.2** (45 min): **Implement and Test Inverse Kinematics**
Implement the inverse kinematics code provided in Section 2.3. Test it with:
- A target at maximum reach
- A target at minimum reach
- A target near the base
- A target at various angles

Visualize both solutions for each target.

**Lab 2.3** (60 min): **Workspace Visualization**
Create a program that samples random joint configurations and plots all reachable end-effector positions. This will help you visualize the workspace boundary and understand which positions are reachable.

**Lab 2.4** (Bonus - 30 min): **Interactive IK Solver**
Create an interactive program where you can click on a 2D canvas to set a target position, and the arm animates to reach it using inverse kinematics.

---

## References

- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson.
- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley.
- PyBullet Documentation: https://docs.google.com/document/d/10sXEhzFQQnI2V0bMbqDZ1eV8WtY6CG4pjGY5z6i-tuw/
