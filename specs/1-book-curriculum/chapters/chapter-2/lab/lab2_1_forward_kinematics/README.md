# Lab 2.1: Forward Kinematics Implementation

## Learning Objectives
By the end of this lab, you will:
- Understand the Denavit-Hartenberg (DH) parameter convention
- Implement homogeneous transformation matrices
- Compute forward kinematics for a multi-link robot arm
- Visualize robot configurations in RViz2
- Publish end-effector poses to ROS 2 topics

**Time Estimate**: 30 minutes
**Difficulty**: Beginner

---

## Theory Background

### Denavit-Hartenberg Parameters

The DH convention provides a systematic way to define coordinate frames on robot links. Each joint has 4 parameters:

1. **θ (theta)**: Joint angle (rotation about z-axis) [variable for revolute joints]
2. **d**: Link offset (translation along z-axis) [variable for prismatic joints]
3. **a**: Link length (translation along x-axis)
4. **α (alpha)**: Link twist (rotation about x-axis)

### Transformation Matrix

The transformation from frame i-1 to frame i is:

```
T_i = Rot_z(θ_i) × Trans_z(d_i) × Trans_x(a_i) × Rot_x(α_i)

     ┌ cos(θ)  -sin(θ)cos(α)   sin(θ)sin(α)  a·cos(θ) ┐
T_i =│ sin(θ)   cos(θ)cos(α)  -cos(θ)sin(α)  a·sin(θ) │
     │   0         sin(α)         cos(α)         d     │
     └   0           0               0           1     ┘
```

### Forward Kinematics Chain

The end-effector transformation from base frame:

```
T_end = T_1 × T_2 × T_3 × ... × T_n
```

---

## Robot Specification

We'll use a **3-DOF planar arm** with the following parameters:

| Joint | θ (deg) | d (m) | a (m) | α (deg) |
|-------|---------|-------|-------|---------|
| 1     | q1      | 0     | 1.0   | 0       |
| 2     | q2      | 0     | 0.8   | 0       |
| 3     | q3      | 0     | 0.5   | 0       |

**Physical Interpretation**:
- Joint 1: Base rotation (shoulder)
- Joint 2: Elbow joint
- Joint 3: Wrist joint
- All joints are revolute (rotating)
- Total reach: 1.0 + 0.8 + 0.5 = 2.3 m (fully extended)

---

## Lab Tasks

### Task 1: Understand the DH Table
Study the DH parameters table above and answer:
1. What is the maximum reach of this arm?
2. Which parameters are constant vs variable?
3. Why is α = 0 for all joints? (Hint: planar arm)

### Task 2: Implement DH Transformation Function
Open `nodes/fk_calculator.py` and complete the `compute_dh_transform()` function.

**Inputs**:
- theta: Joint angle (radians)
- d: Link offset (meters)
- a: Link length (meters)
- alpha: Link twist (radians)

**Output**:
- 4x4 homogeneous transformation matrix (NumPy array)

**Implementation checklist**:
- [ ] Use `np.cos()` and `np.sin()` for trigonometric functions
- [ ] Construct the 4x4 matrix according to DH convention
- [ ] Handle edge cases (theta=0, alpha=0)
- [ ] Return a NumPy array of shape (4, 4)

### Task 3: Compute Forward Kinematics
Complete the `compute_forward_kinematics()` function to:
1. Define the DH parameter table for the 3-DOF arm
2. Compute individual transformation matrices T1, T2, T3
3. Multiply matrices to get end-effector pose: T_end = T1 × T2 × T3
4. Extract position (x, y, z) from the transformation matrix

**Expected output for q = [0, 0, 0]** (all joints at zero):
```
End-effector position: [2.3, 0.0, 0.0] m
```

**Expected output for q = [π/2, 0, 0]** (90° shoulder rotation):
```
End-effector position: [0.0, 2.3, 0.0] m
```

### Task 4: Create ROS 2 Publisher
Complete the ROS 2 node to:
1. Subscribe to `/joint_states` topic (sensor_msgs/JointState)
2. Compute FK whenever new joint angles are received
3. Publish end-effector pose to `/end_effector_pose` (geometry_msgs/PoseStamped)

**Topic Details**:
- Input: `/joint_states` → Joint angles [q1, q2, q3]
- Output: `/end_effector_pose` → End-effector position and orientation

### Task 5: Visualize in RViz2
Launch the complete system and verify:
1. Robot model loads correctly
2. End-effector pose marker appears
3. Moving joint sliders updates the marker position

---

## Implementation Guide

### Step 1: Review the Starter Code
```bash
cat nodes/fk_calculator.py
```

Key sections to implement:
- `compute_dh_transform()` → DH matrix computation
- `compute_forward_kinematics()` → FK chain multiplication
- `joint_state_callback()` → ROS 2 callback logic

### Step 2: Test DH Transform Function
Before running the full node, test your DH function:

```python
# Test case 1: Identity transformation
T = compute_dh_transform(0, 0, 0, 0)
# Expected: 4x4 identity matrix

# Test case 2: Pure rotation
T = compute_dh_transform(np.pi/2, 0, 0, 0)
# Expected: 90° rotation about z-axis

# Test case 3: Pure translation
T = compute_dh_transform(0, 0, 1.0, 0)
# Expected: 1m translation along x-axis
```

### Step 3: Test Forward Kinematics
Test your FK function with known configurations:

```python
# Configuration 1: All joints at zero
q = [0, 0, 0]
pos, rot = compute_forward_kinematics(q)
print(f"Position: {pos}")  # Expected: [2.3, 0, 0]

# Configuration 2: 90° shoulder rotation
q = [np.pi/2, 0, 0]
pos, rot = compute_forward_kinematics(q)
print(f"Position: {pos}")  # Expected: [0, 2.3, 0]

# Configuration 3: Fully folded
q = [0, np.pi, -np.pi]
pos, rot = compute_forward_kinematics(q)
print(f"Position: {pos}")  # Expected: [0.7, 0, 0]
```

### Step 4: Launch the Node
```bash
# Terminal 1: Launch RViz2 with robot model
ros2 launch lab2_1_forward_kinematics lab2_1_fk.launch.py

# Terminal 2: Publish test joint states
ros2 topic pub /joint_states sensor_msgs/JointState \
  "{name: ['joint1', 'joint2', 'joint3'], position: [0.0, 0.5, 1.0]}"
```

### Step 5: Verify Output
```bash
# Check if FK node is running
ros2 node list
# Expected: /fk_calculator

# Check published topics
ros2 topic list
# Expected: /end_effector_pose

# Echo end-effector pose
ros2 topic echo /end_effector_pose
```

---

## Expected Outputs

### Console Output
```
[INFO] [fk_calculator]: Forward Kinematics Calculator Node Started
[INFO] [fk_calculator]: DH Parameters:
[INFO] [fk_calculator]:   Joint 1: θ=q1, d=0.0, a=1.0, α=0.0
[INFO] [fk_calculator]:   Joint 2: θ=q2, d=0.0, a=0.8, α=0.0
[INFO] [fk_calculator]:   Joint 3: θ=q3, d=0.0, a=0.5, α=0.0
[INFO] [fk_calculator]: Waiting for joint states...
[INFO] [fk_calculator]: Received joint states: [0.0, 0.5, 1.0]
[INFO] [fk_calculator]: End-effector position: [1.856, 0.234, 0.0]
[INFO] [fk_calculator]: Published to /end_effector_pose
```

### RViz2 Visualization
You should see:
1. A 3-link robot arm with three revolute joints
2. A red sphere marker at the end-effector position
3. Coordinate frame axes at the end-effector
4. Joint State Publisher GUI sliders to control joint angles

### Topic Output
```bash
$ ros2 topic echo /end_effector_pose
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: 'world'
pose:
  position:
    x: 1.856
    y: 0.234
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.479
    w: 0.878
```

---

## Verification Checklist

After implementation, verify:

- [ ] DH transform function passes unit tests
- [ ] FK computation matches expected positions (within 0.01m)
- [ ] ROS 2 node starts without errors
- [ ] Node subscribes to `/joint_states` topic
- [ ] Node publishes to `/end_effector_pose` topic
- [ ] RViz2 displays robot model correctly
- [ ] End-effector marker moves when joint sliders change
- [ ] Console logs show correct FK calculations

---

## Troubleshooting

### Issue: Matrix Multiplication Error
**Symptom**: `ValueError: shapes (4,4) and (3,1) not aligned`

**Solution**: Ensure you're using `@` operator or `np.matmul()` for matrix multiplication:
```python
# Correct
T_end = T1 @ T2 @ T3

# Incorrect
T_end = T1 * T2 * T3  # Element-wise multiplication, not matrix multiplication
```

### Issue: End-Effector Position is NaN
**Symptom**: Position shows `[nan, nan, nan]`

**Solution**: Check for division by zero or uninitialized variables:
```python
# Add validation
if np.any(np.isnan(T_end)):
    self.get_logger().error("NaN detected in transformation matrix")
    return
```

### Issue: RViz2 Shows No Robot
**Symptom**: RViz2 window is empty

**Solutions**:
1. Check Fixed Frame is set to `world` or `base_link`
2. Add RobotModel display: Click "Add" → "RobotModel"
3. Verify URDF is published:
```bash
ros2 topic echo /robot_description
```

### Issue: Joint State Publisher Not Working
**Symptom**: Sliders don't update joint positions

**Solution**: Ensure Joint State Publisher GUI is launched:
```bash
ros2 run joint_state_publisher_gui joint_state_publisher_gui
```

---

## Extensions & Challenges

### Challenge 1: Workspace Visualization
Modify the node to compute and plot the reachable workspace:
- Sample joint angles uniformly: q1 ∈ [0, 2π], q2 ∈ [-π, π], q3 ∈ [-π, π]
- Compute FK for each configuration
- Plot end-effector positions as a 3D scatter plot

**Expected output**: Donut-shaped reachable workspace

### Challenge 2: Velocity Kinematics
Extend the node to compute end-effector velocity:
- Compute Jacobian matrix J(q)
- Given joint velocities q̇, compute v = J(q) × q̇
- Publish end-effector velocity to `/end_effector_velocity`

### Challenge 3: Multiple Robot Arms
Modify the URDF and code to support:
- Two 3-DOF arms (left and right)
- Independent FK computation for each arm
- Dual visualization in RViz2

### Challenge 4: Performance Optimization
Optimize your FK computation:
- Pre-compute constant transformation matrices
- Use vectorized NumPy operations
- Benchmark computation time (should be < 1ms)

**Target**: Process 1000 FK computations in < 1 second

---

## Assessment Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| DH Transform | 25 | Correct implementation of 4x4 DH matrix |
| FK Computation | 25 | Accurate end-effector position for test cases |
| ROS 2 Integration | 20 | Proper subscriber/publisher setup |
| Code Quality | 15 | Clean code, docstrings, type hints |
| Visualization | 10 | RViz2 correctly displays robot and marker |
| Documentation | 5 | Comments explain logic and equations |
| **Total** | **100** | |

**Passing Score**: 70/100

---

## Reference Solutions

### DH Transformation Matrix (Pseudocode)
```python
def compute_dh_transform(theta, d, a, alpha):
    ct, st = cos(theta), sin(theta)
    ca, sa = cos(alpha), sin(alpha)

    return [
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,   sa,     ca,    d],
        [0,   0,      0,     1]
    ]
```

### Forward Kinematics (Pseudocode)
```python
def compute_forward_kinematics(q):
    # DH table (theta, d, a, alpha)
    dh_params = [
        (q[0], 0, 1.0, 0),
        (q[1], 0, 0.8, 0),
        (q[2], 0, 0.5, 0)
    ]

    # Compute individual transforms
    T = eye(4)
    for params in dh_params:
        T = T @ compute_dh_transform(*params)

    # Extract position and orientation
    position = T[0:3, 3]
    rotation = T[0:3, 0:3]

    return position, rotation
```

---

## Next Steps

After completing this lab:
1. Review your implementation with the reference solution
2. Test with at least 5 different joint configurations
3. Document any discrepancies or interesting findings
4. Prepare for Lab 2.2: Inverse Kinematics

**Proceed to**: `lab2_2_inverse_kinematics/README.md`

---

**Lab 2.1 Version**: 1.0
**Last Updated**: 2026-01-03
