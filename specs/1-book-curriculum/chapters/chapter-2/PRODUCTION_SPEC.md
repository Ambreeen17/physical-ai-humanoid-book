# Chapter 2 Production Specification: Kinematics & Dynamics

**Status**: Ready for agent orchestration
**Target Audience**: Intermediate robotics students with calculus and linear algebra background
**Prerequisites**: Chapter 1 (Physical AI fundamentals, ROS 2 basics)

---

## Learning Objectives

By the end of this chapter, learners will be able to:

1. **Define joint space and configuration space** for robot manipulators and explain the relationship between them
2. **Compute forward kinematics** using Denavit-Hartenberg (DH) parameters for 2-6 DOF arms
3. **Solve inverse kinematics** analytically for 2-DOF/3-DOF arms and numerically for general manipulators
4. **Apply Lagrangian mechanics** to derive equations of motion for multi-link systems
5. **Simulate robot dynamics** in MuJoCo and Gazebo, comparing simulation with analytical predictions

---

## Key Topics (Detailed Outline)

### Section 2.1: Joint Spaces and Configuration Spaces
- **Joint space (q-space)**: Vector of joint angles/positions
- **Configuration space (C-space)**: Set of all possible robot configurations
- **Workspace**: Reachable positions of end-effector in Cartesian space
- **Singularities**: Configurations where manipulator loses degrees of freedom
- **Examples**: 2-DOF planar arm, 6-DOF Unitree G1 arm

### Section 2.2: Forward Kinematics
- **Homogeneous transformations**: 4x4 matrices for position + orientation
- **Denavit-Hartenberg (DH) convention**: 4 parameters (θ, d, a, α) per joint
- **DH table construction**: Step-by-step methodology
- **Forward kinematics equation**: T_end = T1 × T2 × ... × Tn
- **Tool**: Python implementation with NumPy
- **Real-world example**: Computing Unitree G1 gripper position from joint angles

### Section 2.3: Inverse Kinematics
- **Problem statement**: Given desired end-effector pose, find joint angles
- **Analytical solutions**: Geometric approach for 2-DOF, 3-DOF arms
  - Law of cosines
  - Atan2 for angle computation
  - Multiple solutions (elbow-up vs elbow-down)
- **Numerical solutions**: Iterative methods for 6-DOF+ arms
  - Jacobian pseudo-inverse
  - Damped least squares
  - Cyclic Coordinate Descent (CCD)
- **Singularity handling**: Detecting and avoiding singular configurations
- **Tool**: SciPy optimization, IKPy library

### Section 2.4: Lagrangian Dynamics
- **Energy-based formulation**: Kinetic energy (T) and potential energy (V)
- **Lagrangian**: L = T - V
- **Euler-Lagrange equation**: d/dt(∂L/∂q̇) - ∂L/∂q = τ
- **Generalized coordinates**: Joint angles q1, q2, ..., qn
- **Deriving equations of motion**: Step-by-step for 2-link planar arm
- **Mass matrix M(q)**, **Coriolis/centrifugal terms C(q, q̇)**, **gravity terms g(q)**
- **Canonical form**: M(q)q̈ + C(q, q̇)q̇ + g(q) = τ

### Section 2.5: Newton-Euler Formulation (Optional/Advanced)
- **Recursive algorithm**: Forward pass (velocities, accelerations) + Backward pass (forces, torques)
- **Computational efficiency**: O(n) vs O(n³) for Lagrangian
- **Use case**: Real-time control on embedded systems

### Section 2.6: Simulation of Multi-Body Dynamics
- **Physics engines**: MuJoCo, PyBullet, Gazebo
- **Contact dynamics**: Collision detection, friction models
- **Integration methods**: Euler, RK4, implicit integrators
- **Sim-to-real validation**: Comparing simulated vs measured joint torques
- **Unitree G1 case study**: Simulating full-body dynamics

---

## Hardware Context

**Target Platform**: Unitree G1 Humanoid Robot
- **Arm specifications**: 6-DOF per arm (shoulder: 3-DOF, elbow: 1-DOF, wrist: 2-DOF)
- **Joint types**: Revolute joints, direct-drive motors
- **Control frequency**: 500 Hz real-time control loop
- **Payload capacity**: ~5 kg per arm
- **Reach**: ~0.8 m from shoulder

**Sensors for validation**:
- Joint encoders (position feedback)
- Current sensors (torque estimation via I-D model)
- RGB-D camera (optional, for visual servoing)

---

## Simulation Tools

1. **MuJoCo** (primary for dynamics)
   - XML model definition
   - High-fidelity contact simulation
   - Python bindings (mujoco-py)

2. **Gazebo Harmonic** (primary for ROS 2 integration)
   - URDF/SDF robot description
   - ROS 2 control integration
   - Sensor plugins

3. **PyBullet** (alternative, Python-native)
   - Lightweight, easy to install
   - Good for rapid prototyping

---

## Lab Exercises

### Lab 2.1: Forward Kinematics Implementation (30 min, Beginner)
**Objective**: Compute FK for a 2-DOF planar arm using DH parameters

**Tasks**:
1. Define DH table for 2-link arm (L1=1.0m, L2=0.8m)
2. Implement `compute_dh_transform(theta, d, a, alpha)` function
3. Compute end-effector position for q = [30°, 45°]
4. Visualize arm configuration in matplotlib

**Expected Output**:
```
End-effector position: [x, y] = [1.366, 1.166]
```

**Starter code**: `lab2_1_starter.py`

---

### Lab 2.2: Inverse Kinematics Solver (45 min, Intermediate)
**Objective**: Implement analytical IK for 2-DOF arm, numerical IK for 3-DOF arm

**Tasks**:
1. Implement geometric IK solver using law of cosines
2. Handle multiple solutions (elbow-up vs elbow-down)
3. Implement Jacobian-based numerical IK for 3-DOF arm
4. Visualize convergence of iterative solver

**Expected Output**:
```
Target position: [1.0, 1.0]
Solution 1 (elbow-up): [45.0°, 90.0°]
Solution 2 (elbow-down): [135.0°, -90.0°]
Numerical IK converged in 8 iterations
```

**Starter code**: `lab2_2_starter.py`

---

### Lab 2.3: Dynamics Simulation in MuJoCo (60 min, Advanced)
**Objective**: Simulate 2-link arm with gravity, compare with analytical dynamics

**Tasks**:
1. Create MuJoCo XML model for 2-link arm
2. Derive Lagrangian dynamics equations analytically
3. Run simulation with zero torque input (passive dynamics)
4. Compare simulated joint accelerations with analytical predictions
5. Plot joint trajectories over 5 seconds

**Expected Output**:
```
Max position error: 0.003 rad
Max velocity error: 0.02 rad/s
Conclusion: Simulation matches analytical model within 1% error
```

**Starter code**: `lab2_3_starter.py`, `arm_model.xml`

---

## Assessment Suite

### Conceptual Questions (6 points)
1. **MC**: What does a singularity in a robot manipulator mean? (2 pts)
2. **MC**: Which DH parameter represents joint rotation? (2 pts)
3. **Short Answer**: Explain why inverse kinematics may have multiple solutions. Give a physical example. (2 pts)

### Coding Task (10 points)
**Task**: Extend Lab 2.2 to handle workspace boundaries. If target is unreachable, return the closest reachable configuration.

**Rubric**:
- Correct reachability check (4 pts)
- Closest config computation (4 pts)
- Visualization of workspace boundary (2 pts)

### Simulation Challenge (5 bonus points)
**Task**: Implement PD control for the 2-link arm to track a circular trajectory. Tune Kp and Kd gains.

**Success Metrics**:
- Trajectory tracking error < 5 cm
- Smooth motion (no oscillations)
- Plots of desired vs actual trajectory

---

## Personalization Variants

### Beginner Variant
- Focus on 2-DOF planar arm only
- Geometric intuition before equations
- Step-by-step DH parameter construction
- Skip Newton-Euler formulation
- Labs with complete starter code

### Intermediate Variant (Baseline)
- Full coverage of topics as specified
- Balance between intuition and math
- 3-DOF and 6-DOF examples
- Labs with partial starter code

### Advanced Variant
- Include Newton-Euler formulation
- Screw theory representation
- Optimal control formulations
- Research papers on contact-rich manipulation
- Labs from scratch (minimal starter code)

---

## Diagrams Required

1. **Joint space vs Configuration space** (2D vs 3D visualization)
2. **DH parameter definition** (coordinate frames on robot link)
3. **Forward kinematics chain** (T1 × T2 × ... × Tn)
4. **Inverse kinematics solutions** (elbow-up vs elbow-down)
5. **Workspace boundary** (reachable vs unreachable regions)
6. **Lagrangian dynamics flowchart** (T, V → L → Euler-Lagrange → M, C, g)
7. **MuJoCo simulation pipeline** (XML → Physics → Visualization)

---

## Urdu Localization Notes

**Key terms**:
- Kinematics → حرکیات (harakiyat)
- Dynamics → حرکی علوم (harki uloom)
- Joint space → مشترکہ جگہ (mushtarika jagah)
- Forward kinematics → آگے کی حرکیات (aage ki harakiyat)
- Inverse kinematics → الٹی حرکیات (ulti harakiyat)

**Preserve in English**: DH parameters, Lagrangian, Jacobian, MuJoCo, URDF

---

## RAG Indexing Strategy

**Chunk Types**:
- **Theory sections** (300-500 tokens): Definitions, explanations, examples
- **Math derivations** (200-400 tokens): Equations with context
- **Code examples** (100-300 tokens): Snippets with comments
- **Lab instructions** (150-300 tokens): Step-by-step tasks

**Metadata**:
- `chapter`: 2
- `section`: "2.1", "2.2", etc.
- `difficulty`: "beginner", "intermediate", "advanced"
- `content_type`: "theory", "math", "code", "lab"
- `keywords`: ["forward_kinematics", "DH_parameters", "jacobian", ...]

**Expected chunks**: ~15 semantic chunks

---

## QA Checklist

- [ ] All learning objectives covered
- [ ] Math notation consistent (LaTeX formatting)
- [ ] Code examples run without errors
- [ ] Hardware specs accurate (Unitree G1)
- [ ] Labs executable in Docker (MuJoCo, Gazebo, PyBullet)
- [ ] Assessments align with objectives
- [ ] Diagrams specified (ASCII, Mermaid, descriptions)
- [ ] Personalization variants complete
- [ ] Urdu translation glossary provided
- [ ] RAG chunks prepared

---

**Production Spec Version**: 1.0
**Ready for Agent Orchestration**: ✅
