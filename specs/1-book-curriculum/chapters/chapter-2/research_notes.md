# Chapter 2 Research Notes: Kinematics & Dynamics

**Research Date**: 2026-01-04
**Target Platform**: Unitree G1 Humanoid Robot
**Control Frequency**: 500 Hz
**Arm DOF**: 6-DOF per arm

---

## Overview

Kinematics and dynamics form the mathematical foundation of robot motion. Kinematics describes the geometry of motion without considering forces, while dynamics incorporates mass, inertia, and external forces to predict actual motion. For the Unitree G1 humanoid robot with its 6-DOF arms, these concepts are essential for precise manipulation tasks.

The chapter bridges theoretical mechanics (Lagrangian formulation) with practical implementation (numerical simulation). Students will learn to compute where a robot's end-effector will be given joint positions (forward kinematics), determine joint positions needed to reach a target (inverse kinematics), and predict how the robot will move under applied torques (dynamics). This knowledge directly enables trajectory planning, force control, and sim-to-real deployment for physical AI systems.

---

## 1. Joint Space, Configuration Space, Workspace, and Singularities

### Core Concepts

- **Joint Space (q-space)**: The n-dimensional space where each dimension represents a single joint variable (angles for revolute joints, displacements for prismatic joints). For a 6-DOF arm, this is R^6 (6 real numbers). The configuration is represented as q = [q1, q2, q3, q4, q5, q6]^T. All robot control commands ultimately specify positions or velocities in this space.

- **Configuration Space (C-space)**: The complete set of all possible robot configurations, accounting for joint limits and physical constraints. For a robot with n revolute joints and no constraints, C-space is an n-torus (T^n). With joint limits, it becomes a subset bounded by hyper-rectangles. The topology of C-space directly impacts motion planning complexity.

- **Workspace**: The set of all positions the end-effector can reach, categorized into **reachable workspace** (all reachable points regardless of orientation) and **dexterous workspace** (points reachable with any orientation). For the Unitree G1 arm with approximately 0.8m reach, the workspace is a sphere of radius 0.8m, though actual shape is modified by self-collision constraints and joint limits.

- **Singularities**: Configurations where the robot loses instantaneous mobility. Mathematically, these occur when the Jacobian matrix J loses full rank (det(J) = 0). At singularities, the manipulator cannot produce motion in certain directions, and joint velocities required for Cartesian motion become unbounded. Types include **boundary singularities** (at workspace edges) and **internal singularities** (within workspace, e.g., wrist singularity when wrist axes align).

- **Degree of Mobility (DOF)**: The number of independent parameters defining configuration minus number of constraints. For open-chain manipulators, mobility equals the number of joints. A 6-DOF arm can position and orient the end-effector in 3D space (6 constraints for SE(3) configuration), making it fully capable for general manipulation.

### Industry-Standard Tools

- **ROS 2 MoveIt**: Uses configuration spaces for motion planning with collision checking. Integrates with OMPL planners that operate in joint space.
- **PyBullet/KDL**: Provides Jacobian computation and singularity analysis. The KDL kinematic tree library implements forward/inverse kinematics for arbitrary robot topologies.
- **RoboDK**: Commercial tool with workspace visualization and singularity analysis GUI.
- **Matlab Robotics System Toolbox**: Academic standard for C-space visualization and workspace analysis.

### Practical Constraints

- **Joint Limits**: The Unitree G1 arm joints have limits (typically +/- 150-170 degrees for shoulder/elbow, +/- 90-120 degrees for wrist). These create non-convex boundaries in C-space that can trap motion planners.
- **Self-Collision**: The humanoid torso and opposite arm constrain C-space, particularly when reaching across the body. Configuration space must exclude self-collision regions.
- **Computation**: C-space dimensionality grows exponentially with DOF for complete coverage. Practically, sampling-based methods (PRM, RRT) are used for high-DOF systems.
- **Singularity Proximity**: Near singularities, joint velocity requirements exceed actuator limits. Controllers must detect and gracefully handle singularity approach.

### Common Beginner Mistakes

- **Confusing joint space with Cartesian space**: Using Euclidean distance in joint space to measure reachability. Always transform to Cartesian space for workspace analysis. Prevention: Visualize both spaces side-by-side in simulation.

- **Ignoring joint limits in IK**: Writing IK solvers that return solutions outside physical limits. Impact: Robot may hit mechanical stops or tracking becomes unstable. Prevention: Add joint limit constraints as inequality bounds in the solver.

- **Believing singularities only occur at extremes**: Internal singularities (e.g., when wrist axes align) can occur in middle of workspace. Impact: Sudden large joint velocity requirements cause instability. Prevention: Implement singularity avoidance through null-space optimization or task redundancy.

- **Assuming workspace is convex**: Reachable workspace for multi-link arms has non-convex voids due to joint limits and self-collision. Prevention: Use sampling to map actual workspace boundaries rather than geometric assumptions.

---

## 2. Forward Kinematics

### Core Concepts

- **Homogeneous Transformations**: 4x4 matrices combining 3x3 rotation R and 3x1 translation t into a single representation:

```
T = [R  t]
    [0  1]
```

  These matrices form the SE(3) group (Special Euclidean group in 3D). Multiplication composes transformations, enabling chain computation from base to end-effector. The homogeneous representation unifies rotation and translation, simplifying computational implementation.

- **Denavit-Hartenberg (DH) Parameters**: A standardized convention for assigning coordinate frames to robot links using 4 parameters per joint:
  - **theta_i**: Joint angle (rotation about z_{i-1})
  - **d_i**: Link offset (translation along z_{i-1})
  - **a_i**: Link length (translation along x_i)
  - **alpha_i**: Link twist (rotation about x_i)

  The DH transformation matrix for each joint is:

```
T_i = [cos(theta)  -sin(theta)*cos(alpha)   sin(theta)*sin(alpha)  a_i*cos(theta)]
      [sin(theta)   cos(theta)*cos(alpha)  -cos(theta)*sin(alpha)  a_i*sin(theta)]
      [0            sin(alpha)              cos(alpha)              d_i]
      [0            0                        0                       1]
```

- **DH Table Construction**: Systematic procedure: (1) Establish base frame at joint 1, (2) align z-axes with joint rotation axes, (3) set x-axes perpendicular to consecutive z-axes pointing from joint i to i+1, (4) compute parameters by projecting onto axes. Common convention: use modified DH (frame at joint output) vs standard DH (frame at joint input).

- **Forward Kinematics Equation**: The end-effector pose is computed as the product of individual link transforms: T_ee = T_0^1 * T_1^2 * ... * T_{n-1}^n. Each 4x4 matrix multiplication is O(1), making total computation O(n) for n joints. For the Unitree G1 6-DOF arm, this chain computes gripper pose from 6 joint angles.

- **Frame Conventions**: Critical distinction between fixed frames (attached to links) and moving frames (end-effector). The world frame serves as the global reference. Understanding which frames transform relative to which (post-multiplication for standard DH, pre-multiplication for modified DH) prevents sign errors.

### Industry-Standard Tools

- **Python NumPy**: Core library for matrix operations. Implementing DH transforms requires careful indexing and floating-point operations. NumPy's matrix multiplication (`@` operator) handles homogeneous transforms naturally.

- **IKPy**: Python library for inverse kinematics that includes forward kinematics as a building block. Handles URDF import and joint chain parsing automatically.

- **ROS 2 tf2/tf2_ros**: Real-time frame transformation library. Publishers maintain transform trees; subscribers query poses between any two frames. Essential for multi-sensor systems.

- **KDL (Kinematics and Dynamics Library)**: Part of Orocos project. Provides verified forward/inverse kinematics for arbitrary kinematic chains. Integrates with MoveIt for motion planning.

- **Modern Robotics Library (MATLAB/Python)**: Academic library implementing rigorous kinematics with formal verification. Useful for teaching the mathematical foundations.

### Practical Constraints

- **Singular Configurations**: At singularities, the Jacobian becomes rank-deficient, and forward kinematics itself is still valid but inverse kinematics fails. Forward kinematics is always computable.

- **Numerical Stability**: Accumulated floating-point errors in long transform chains. For 6-DOF arms, errors are typically small (< 1e-10), but for arms with many joints, consider symbolic computation or error compensation.

- **DH Parameter Variations**: Two conventions exist (standard vs modified DH). Mixing conventions in a single robot model causes systematic errors. Always document which convention is used.

- **Frame Definition Errors**: Incorrectly placed frames (non-perpendicular axes, wrong origin) cause systematic FK errors. Prevention: Verify FK against known configurations (e.g., all-zeros pose) before proceeding.

### Common Beginner Mistakes

- **Parameter ordering errors**: Confusing whether alpha corresponds to rotation about the old or new x-axis. Impact: Systematic pose error that propagates through all configurations. Prevention: Write parameter extraction clearly and verify with test poses.

- **Neglecting joint types**: Treating prismatic and revolute joints identically. Revolute joints vary theta with fixed d; prismatic joints vary d with fixed theta. Prevention: Explicitly handle joint type in FK implementation.

- **Frame multiplication order**: Using pre-multiplication when post-multiplication is needed (or vice versa). Impact: Non-physical poses (gimbal lock, incorrect positions). Prevention: Use a consistent convention and test with simple 2-link arm first.

- **Forgetting base transform**: FK typically gives pose relative to frame 0. For absolute position, multiply by the base-to-world transform. Impact: Robot thinks it is elsewhere than it actually is. Prevention: Always track the full transform chain from world frame.

---

## 3. Inverse Kinematics

### Core Concepts

- **Problem Statement**: Given a desired end-effector pose T_des in SE(3) (position + orientation), find joint angles q such that FK(q) = T_des. This is the inverse of forward kinematics. Unlike FK, IK may have zero, one, or infinitely many solutions, and computation is more complex.

- **Analytical (Geometric) IK**: Closed-form solutions using trigonometry. For 2-DOF planar arm, use law of cosines to solve the triangle formed by the two links and the target point. For 3-DOF arms, decouple position and orientation (wrist center first, then wrist orientation). Advantages: Fast, guaranteed convergence, provides all solutions. Disadvantages: Robot-specific, complex derivation for 6-DOF.

- **Numerical IK**: Iterative methods that start from an initial guess and converge toward a solution. The core iteration is: q_{k+1} = q_k + alpha * J^+ * e, where J^+ is the Jacobian pseudo-inverse and e is the pose error. Suitable for any robot geometry. Convergence is not guaranteed and depends on initial guess and singularities.

- **Jacobian Pseudo-Inverse**: J^+ = J^T (J J^T)^(-1) for full-rank matrices. Provides minimum-norm solution (smallest joint motion) for a given Cartesian error. Computed via SVD for numerical stability. The pseudo-inverse formulation: dq = J^+ * dx.

- **Damped Least Squares (DLS)**: Modified pseudo-inverse that adds regularization: J^+ = J^T (J J^T + lambda^2 I)^(-1). The damping parameter lambda trades off convergence speed (small lambda) against stability near singularities (large lambda). This is the Levenberg-Marquardt method adapted for IK. Typical lambda values: 0.01 to 0.5.

- **Multiple Solutions**: IK problems often have multiple valid configurations. For planar 2-DOF arms: elbow-up and elbow-down solutions. For 6-DOF arms: up to 16 solutions exist in the absence of joint limits. Selection criteria include: joint limit avoidance, singularity avoidance, and minimum torque.

### Industry-Standard Tools

- **SciPy optimize**: The `optimize.minimize` function with various methods (L-BFGS-B, SLSQP) handles constrained IK. Joint limits are inequality constraints. Slower than analytical but flexible.

- **IKPy**: Python library implementing multiple IK algorithms including Jacobian-based, L-BFGS, and_TRAC-IK. Handles URDF import and joint chain parsing.

- **MoveIt IKFast**: Generates optimized analytical IK solvers for specific robots. Produces C++ code that is orders of magnitude faster than numerical methods. Robot-specific; requires robot model and target DOF.

- **ROS 2 MoveIt KDL IK**: Default numerical IK solver. Uses KDL's Jacobian-based iterative solver. Good balance of speed and robustness.

- **TRAC-IK**: Improved IK solver that handles joint limits better than standard KDL. Available as MoveIt plugin.

### Practical Constraints

- **Real-Time Performance**: At 500 Hz control rate for Unitree G1, IK must compute within 2ms. Analytical IK is essential for real-time control loops. Numerical IK may require 10-100ms depending on convergence.

- **Orientation Handling**: Full 6-DOF IK requires matching both position and orientation. Orientation error typically uses quaternion or angle-axis representation to avoid gimbal lock.

- **Initial Guess Sensitivity**: Numerical IK may converge to local minima or fail entirely with poor initial guesses. Solution: Use multiple initial guesses or FK seed from previous timestep.

- **Joint Limit Constraints**: Physical limits are hard constraints. Clamping joint angles after IK produces constraint violations. Solution: Include limits as constraints in the optimization or use null-space projection to bias solutions away from limits.

### Common Beginner Mistakes

- **Expecting always-convergent solutions**: Assuming numerical IK will always find a solution. Impact: Robot may twitch unpredictably when IK fails. Prevention: Implement convergence detection, fallback behavior, and trajectory replanning.

- **Using naive Jacobian inverse**: Computing J^(-1) directly instead of pseudo-inverse J^+. Impact: Works only for square full-rank Jacobians; fails at singularities with division by zero. Prevention: Always use pseudo-inverse or DLS.

- **Ignoring orientation coupling**: Using only position IK for tasks requiring orientation. Impact: End-effector reaches position but points wrong direction. Prevention: Include full 6-DOF error in IK formulation.

- **Poor initial guess selection**: Starting IK from zero configuration for every frame. Impact: Slow convergence, potential divergence. Prevention: Use previous solution as initial guess (递推, iterative refinement).

- **Not handling unreachable targets**: Assuming all desired poses are reachable. Impact: Robot attempts impossible motions, potentially unsafe. Prevention: Check workspace reachability before invoking IK; return nearest reachable configuration for unreachable targets.

---

## 4. Lagrangian Dynamics

### Core Concepts

- **Lagrangian Formulation**: Energy-based approach where L = T - V, with T = kinetic energy and V = potential energy. The Euler-Lagrange equation relates generalized coordinates to applied torques:

```
d/dt(partial L / partial q_dot) - partial L / partial q = tau
```

  This compact equation generates equations of motion for arbitrary mechanical systems. The formulation naturally handles constraints without explicit constraint forces.

- **Kinetic Energy T**: For an n-link robot, T = 1/2 * q_dot^T * M(q) * q_dot, where M(q) is the symmetric positive-definite mass (inertia) matrix. M(q) depends on configuration because link velocities depend on joint angles through the Jacobian of each link center of mass.

- **Potential Energy V**: Gravitational potential energy V = sum over i of m_i * g * h_i, where h_i is the height of link i's center of mass. V depends only on configuration q, not on velocities.

- **Equations of Motion**: The general form is:

```
M(q) * q_ddot + C(q, q_dot) * q_dot + g(q) = tau
```

  Where:
  - M(q) = mass matrix (n x n)
  - C(q, q_dot) = Coriolis/centrifugal matrix (n x n)
  - g(q) = gravity vector (n x 1)
  - tau = applied joint torques (n x 1)

  This is the canonical robot dynamics equation, used for simulation, control, and analysis.

- **Christoffel Symbols**: The Coriolis matrix elements are computed from mass matrix derivatives: c_ij = sum_k (G_ijk * q_dot_k) where G_ijk = (partial M_ij / partial q_k + partial M_ik / partial q_j - partial M_jk / partial q_i) / 2.

- **Property: Skew-Symmetry**: The matrix N(q, q_dot) = dot(M) - 2*C is skew-symmetric. This property is used in passivity-based control design.

### Industry-Standard Tools

- **Pinocchio (C++/Python)**: Modern rigid body dynamics library. Computes forward/inverse dynamics, Jacobians, and geometric quantities. Powers the dynamics layer of many research projects. Integrates with ROS 2.

- **RBDL (Rigid Body Dynamics Library)**: C++ library with Python bindings. Implements recursive algorithms for efficient dynamics computation. Good for teaching dynamics fundamentals.

- **KDL (Kinematics and Dynamics Library)**: ROS-integrated library. Slower than Pinocchio but widely available in ROS ecosystems. Handles joint chain dynamics computation.

- **MuJoCo Native Functions**: MuJoCo provides mj_kinematics, mj_forward, mj_inverse for dynamics computation. The internal computation uses optimized algorithms including analytical gradients.

- **Symbolic Math Tools**: Maple, Mathematica, or SymPy can derive Lagrangian equations symbolically. For teaching, showing the symbolic derivation clarifies the structure before numerical implementation.

### Practical Constraints

- **Real-Time Control**: At 500 Hz, dynamics must be computed in under 2ms. For 6-DOF arms, M(q) is 6x6, making matrix operations manageable. Recursive algorithms (Newton-Euler) are O(n) vs O(n^3) for Lagrangian.

- **Numerical Integration**: Simulating dynamics requires numerical integration (Euler, RK4). Error accumulates over time. For accurate simulation, use implicit integrators or small timesteps.

- **Model Parameter Uncertainty**: Mass, inertia, and center of mass locations are often approximated from CAD models. Errors of 5-10% are common. Impact: Trajectory tracking errors, potential instability in aggressive motions.

- **Computation Complexity**: M(q) computation is O(n^3) naive, O(n^2) with efficient algorithms, O(n) with Newton-Euler. For real-time control on embedded systems, recursive algorithms are essential.

### Common Beginner Mistakes

- **Forgetting configuration dependence**: Treating M(q) as constant. Impact: Dynamics become inaccurate, controller may become unstable. Prevention: Recompute M(q) at each control cycle or use efficient incremental updates.

- **Confusing Coriolis and centrifugal terms**: C(q, q_dot) includes both. Centrifugal terms scale with q_dot^2; Coriolis terms scale with q_dot_i * q_dot_j. Incorrect modeling causes velocity-dependent errors.

- **Omitting gravity compensation**: Forgetting g(q) in control law. Impact: Robot sags under gravity, tracking becomes poor, especially for slow movements. Prevention: Include gravity compensation in computed-torque control.

- **Sign errors in derivation**: Common in applying chain rule in Euler-Lagrange equation. Impact: Wrong dynamics equations cause simulation mismatch. Prevention: Verify with simple systems (1-DOF pendulum) before extending to n-link arms.

- **Assuming linearity**: The dynamics equation is highly nonlinear. Linear control design is only valid near operating points. Prevention: Use feedback linearization or adaptive control for wide operating ranges.

---

## 5. Simulation Tools for Multi-Body Dynamics

### Core Concepts

- **Physics Engines**: Software that computes multi-body dynamics with contacts. Engines differ in integration method (explicit vs implicit), contact formulation (penalty vs constraint-based), and solver approach. Selection impacts simulation fidelity and stability.

- **MuJoCo (Multi-Joint dynamics with Contact)**: Physics engine using optimization-based contact dynamics. Soft, convex contact formulation based on convex optimization rather than NP-hard complementarity problems. Handles tendon geometry, flexible objects, and general actuation. Native XML format (MJCF) and URDF support.

- **Gazebo Harmonic**: Robot simulation environment with physics engine abstraction (ODE, Bullet, DART, SIMPHONY). Native format is SDF (Simulation Description Format). Strong ROS 2 integration via gazebo_ros packages. Long-term support (2023-2028).

- **PyBullet**: Python interface to Bullet physics engine. Designed for games and ML research. Supports differentiable simulation, deformable objects, and neural network integration. Easy installation via pip (`pip install pybullet`).

- **Integration Methods**: Euler (first-order, simple but unstable), RK4 (fourth-order, better stability), implicit (solves for acceleration, unconditionally stable). Step size selection balances accuracy and speed.

- **Sim-to-Real Gap**: Differences between simulation and reality in contact dynamics, friction models, sensor noise, and actuation delays. Validation requires comparing simulated trajectories against real robot data.

### Industry-Standard Tools

- **MuJoCo (DeepMind)**: Primary choice for dynamics research and control validation. Features include:
  - MJCF XML format for model definition
  - Native OpenGL visualization
  - Python bindings (mujoco, mujoco-py)
  - Optimized C/C++ internals
  - Extensions for Unity, OpenUSD

- **Gazebo Harmonic (Open Robotics)**: Primary choice for ROS-integrated simulation. Features include:
  - SDF world format
  - ROS 2 topic/services integration
  - Multiple physics engines
  - Sensor plugins (cameras, LiDAR, IMU)
  - Fuel model repository

- **PyBullet (Bullet)**: Python-native choice for ML/learning. Features include:
  - Easy pip installation
  - Google Colab compatibility
  - Reinforcement learning examples
  - URDF support
  - Real-time physics for games

- ** Drake (MIT)**: C++/Python planning and control framework. Provides analytical and simulation capabilities. Strong dynamics verification tools. Less focused on real-time visualization.

### Practical Constraints

- **Real-Time Factor**: Simulation speed relative to real-time. MuJoCo achieves 10-100x real-time for simple models. Gazebo typically 1-10x. PyBullet varies widely based on model complexity.

- **Contact Fidelity**: Contact simulation is inherently approximate. Friction models (Coulomb, viscous), restitution coefficients, and contact point computations differ from reality. Important for manipulation tasks involving contact.

- **Timestep Selection**: Physics stability requires small timesteps. Common choices: 1ms for MuJoCo, 0.5ms for Bullet. Larger timesteps cause instability or inaccuracy.

- **Hardware Requirements**: GPU recommended for rendering. MuJoCo visualization is OpenGL-based. Headless simulation (no rendering) reduces requirements significantly.

- **Determinism**: Simulation should be deterministic for reproducibility. MuJoCo provides deterministic mode. Gazebo with ROS 2 may have timing variations affecting determinism.

### Common Beginner Mistakes

- **Mismatched timesteps**: Using different timesteps for physics and control (e.g., 1ms physics, 10ms control). Impact: Integration instability, simulation divergence. Prevention: Match control timestep to physics step or use proper multi-rate design.

- **Ignoring friction**: Using default friction values without verification. Impact: Objects slide unexpectedly, robot wheels spin in place. Prevention: Calibrate friction coefficients against real surfaces.

- **Over-trusting simulation**: Assuming simulation perfectly predicts reality. Impact: Controllers that work in simulation fail on real robot. Prevention: Always validate with hardware; use sim-to-real techniques (domain randomization).

- **Resource leaks**: Not closing simulation properly. Impact: Memory leaks, GPU memory exhaustion. Prevention: Use context managers (Python `with` statement) and explicit cleanup.

- **Incorrect model units**: Mixing meters and centimeters, or radians and degrees. Impact: Physically unrealistic motion, explosions. Prevention: Verify units match (SI units standard); MuJoCo uses meters/kg/seconds.

---

## Key Terms Glossary

- **Configuration Space (C-space)**: The complete set of all possible robot configurations, respecting joint limits and physical constraints.

- **DH Parameters**: Four parameters (theta, d, a, alpha) defining each link's coordinate frame in the Denavit-Hartenberg convention.

- **DOF (Degree of Freedom)**: Number of independent parameters defining configuration; equals number of joints for open-chain manipulators.

- **Euler-Lagrange Equation**: d/dt(partial L/partial q_dot) - partial L/partial q = tau, relating Lagrangian to generalized forces.

- **Forward Kinematics**: Computing end-effector pose from joint positions using geometric transformations.

- **Generalized Coordinates**: Minimal set of coordinates (typically joint angles) describing robot configuration.

- **Homogeneous Transformation**: 4x4 matrix representing SE(3) transformation (rotation + translation).

- **Inverse Kinematics**: Computing joint positions to achieve a desired end-effector pose.

- **Jacobian Matrix**: Matrix mapping joint velocities to Cartesian velocities; J = d(pose)/dq.

- **Lagrangian (L)**: Difference between kinetic and potential energy, L = T - V.

- **Mass Matrix M(q)**: Symmetric positive-definite matrix in dynamics equation, representing configuration-dependent inertia.

- **Pseudo-Inverse (Moore-Penrose)**: Matrix generalization of inverse for non-square or rank-deficient matrices; J^+ = J^T (J J^T)^(-1).

- **Singularity**: Configuration where Jacobian loses rank; manipulator loses instantaneous mobility.

- **Workspace**: Set of all positions the end-effector can reach; may be reachable (any orientation) or dexterous (all orientations).

---

## References

### Academic Texts
- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.). Pearson. Standard textbook for kinematics and dynamics.
- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*. Wiley. Comprehensive dynamics coverage.
- Siciliano, B., Sciavicco, L., Villani, L., & Oriolo, G. (2010). *Robotics: Modelling, Planning and Control*. Springer. Advanced treatment.

### Official Documentation
- MuJoCo Documentation: https://mujoco.readthedocs.io/
- Gazebo Harmonic Documentation: https://gazebosim.org/docs/harmonic/
- ROS 2 Documentation: https://docs.ros.org/en/humble/
- PyBullet Quickstart: https://pybullet.org/wordpress/

### Libraries and Frameworks
- Pinocchio: https://stack-of-tasks.github.io/pinocchio/
- MoveIt: https://moveit.ros.org/
- IKPy: https://github.com/Phylliade/ikpy
- KDL: https://www.orocos.org/kdl

### Unitree G1 Resources
- Unitree Robotics Official Documentation
- Unitree G1 SDK and ROS 2 packages

### Online Learning
- MIT OpenCourseWare: Robotics ( courses on kinematics and dynamics)
- ROS 2 Navigation and Manipulation courses
- Modern Robotics Coursera Specialization
