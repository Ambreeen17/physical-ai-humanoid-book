# Lab 2.2: Inverse Kinematics Solver

## Learning Objectives
By the end of this lab, you will:
- Understand the inverse kinematics problem formulation
- Implement analytical IK using geometric methods (law of cosines)
- Handle multiple solutions (elbow-up vs elbow-down configurations)
- Implement numerical IK using Jacobian-based iterative methods
- Create ROS 2 service interfaces for IK computation
- Validate IK solutions using forward kinematics

**Time Estimate**: 45 minutes
**Difficulty**: Intermediate

---

## Theory Background

### Inverse Kinematics Problem

**Forward Kinematics**: Given joint angles q → Find end-effector pose x
```
x = FK(q)
```

**Inverse Kinematics**: Given desired end-effector pose x → Find joint angles q
```
q = IK(x)
```

**Key Challenges**:
1. **Multiple Solutions**: Different joint configurations can reach the same position
2. **No Solution**: Target pose may be outside workspace
3. **Singularities**: Infinite solutions at certain configurations
4. **Non-linearity**: Equations may require numerical methods

---

### Analytical IK for 2-DOF Planar Arm

For a 2-link planar arm with link lengths L1 and L2:

**Given**: Target position (x, y)
**Find**: Joint angles (θ1, θ2)

**Method: Law of Cosines**

```
Distance to target: d = √(x² + y²)

Elbow angle (θ2):
    cos(θ2) = (x² + y² - L1² - L2²) / (2·L1·L2)
    θ2 = ± acos(cos(θ2))  // Two solutions: elbow-up (+) and elbow-down (-)

Shoulder angle (θ1):
    α = atan2(y, x)
    β = atan2(L2·sin(θ2), L1 + L2·cos(θ2))
    θ1 = α - β  (for elbow-up)
    θ1 = α + β  (for elbow-down)
```

**Reachability Condition**:
```
|L1 - L2| ≤ d ≤ L1 + L2
```

---

### Numerical IK for Higher DOF

For robots with > 3 DOF, analytical solutions become complex. Use iterative numerical methods:

**Jacobian-Based Method**:

```
1. Initialize: q0 = initial guess
2. Compute error: e = x_target - FK(q)
3. Compute Jacobian: J = ∂FK/∂q
4. Update: q = q + J⁺ · e  (J⁺ is pseudo-inverse)
5. Repeat until ||e|| < ε
```

**Damped Least Squares (DLS)**:
```
Δq = Jᵀ(JJᵀ + λ²I)⁻¹ · e
```
Where λ is damping factor to avoid singularities.

---

## Robot Specification

### 2-DOF Planar Arm (Analytical IK)
- Link 1 length: L1 = 1.0 m
- Link 2 length: L2 = 0.8 m
- Workspace: Annular region with inner radius 0.2m, outer radius 1.8m

### 3-DOF Planar Arm (Numerical IK)
- Link 1 length: L1 = 1.0 m
- Link 2 length: L2 = 0.8 m
- Link 3 length: L3 = 0.5 m
- Workspace: More complex, requires numerical methods

---

## Lab Tasks

### Task 1: Implement Analytical IK for 2-DOF Arm

Open `nodes/ik_solver.py` and complete the `compute_analytical_ik()` function.

**Implementation Checklist**:
- [ ] Compute distance to target: d = √(x² + y²)
- [ ] Check reachability: |L1 - L2| ≤ d ≤ L1 + L2
- [ ] Compute elbow angle using law of cosines
- [ ] Handle acos domain errors (clamp to [-1, 1])
- [ ] Compute both elbow-up and elbow-down solutions
- [ ] Return list of valid solutions

**Test Cases**:
```python
# Test 1: Target at maximum reach
x, y = 1.8, 0.0
# Expected: θ1 = 0°, θ2 = 0° (fully extended)

# Test 2: Target at 45°
x, y = 1.2, 1.2
# Expected: Two solutions (elbow-up and elbow-down)

# Test 3: Unreachable target
x, y = 2.5, 0.0
# Expected: No solution (outside workspace)
```

### Task 2: Implement Numerical IK for 3-DOF Arm

Complete the `compute_numerical_ik()` function using Jacobian-based iteration.

**Implementation Checklist**:
- [ ] Define FK function for 3-DOF arm
- [ ] Compute numerical Jacobian using finite differences
- [ ] Implement iterative update: q ← q + J⁺ · e
- [ ] Add convergence check: ||e|| < ε
- [ ] Set maximum iterations (e.g., 100)
- [ ] Handle singularities (damped least squares)

**Algorithm**:
```python
q = q_init  # Initial guess
for i in range(max_iterations):
    x_current = FK(q)
    error = x_target - x_current

    if norm(error) < epsilon:
        return q, True  # Converged

    J = compute_jacobian(q)
    delta_q = pinv(J) @ error
    q = q + alpha * delta_q  # alpha = step size

return q, False  # Did not converge
```

### Task 3: Create ROS 2 Service Interface

Implement a ROS 2 service that accepts a target pose and returns joint angles.

**Service Definition** (use built-in messages):
```
Request: geometry_msgs/Pose target_pose
Response: float64[] joint_angles
         bool success
         string message
```

**Node Structure**:
```python
class IKSolverNode(Node):
    def __init__(self):
        self.service = self.create_service(
            ComputeIK,
            '/compute_ik',
            self.compute_ik_callback
        )

    def compute_ik_callback(self, request, response):
        target = request.target_pose.position
        solutions = self.compute_analytical_ik(target.x, target.y)

        if solutions:
            response.joint_angles = solutions[0]
            response.success = True
        else:
            response.success = False

        return response
```

### Task 4: Create Test Client

Implement `ik_test_client.py` that:
1. Sends multiple test target poses to the IK service
2. Validates solutions using forward kinematics
3. Compares analytical vs numerical IK performance
4. Prints results in a formatted table

**Test Scenarios**:
- 10 random reachable targets
- 5 boundary targets (near workspace limits)
- 3 unreachable targets (outside workspace)

### Task 5: Visualize Solutions

Extend the test client to:
1. Plot workspace boundary
2. Show target positions
3. Display both elbow-up and elbow-down solutions
4. Animate arm motion from one solution to another

---

## Implementation Guide

### Step 1: Implement Analytical IK

```python
def compute_analytical_ik(self, x: float, y: float) -> List[Tuple[float, float]]:
    """
    Compute analytical IK for 2-DOF planar arm.

    Args:
        x: Target x-position [m]
        y: Target y-position [m]

    Returns:
        List of (theta1, theta2) solutions in radians
    """
    L1, L2 = self.link_lengths[0], self.link_lengths[1]

    # Distance to target
    d = np.sqrt(x**2 + y**2)

    # Check reachability
    if d > (L1 + L2) or d < abs(L1 - L2):
        self.get_logger().warning(f'Target ({x}, {y}) is unreachable')
        return []

    # Compute elbow angle (two solutions)
    cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
    cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)  # Clamp to valid range

    theta2_up = np.arccos(cos_theta2)
    theta2_down = -theta2_up

    solutions = []

    # Solution 1: Elbow-up
    alpha = np.arctan2(y, x)
    beta = np.arctan2(L2 * np.sin(theta2_up), L1 + L2 * np.cos(theta2_up))
    theta1_up = alpha - beta
    solutions.append((theta1_up, theta2_up))

    # Solution 2: Elbow-down
    beta = np.arctan2(L2 * np.sin(theta2_down), L1 + L2 * np.cos(theta2_down))
    theta1_down = alpha - beta
    solutions.append((theta1_down, theta2_down))

    return solutions
```

### Step 2: Test Analytical IK

```bash
# Terminal 1: Launch IK solver service
ros2 run lab2_2_inverse_kinematics ik_solver

# Terminal 2: Run test client
ros2 run lab2_2_inverse_kinematics ik_test_client
```

### Step 3: Implement Numerical IK

```python
def compute_numerical_ik(self, x: float, y: float, q_init: np.ndarray) -> Tuple[np.ndarray, bool]:
    """
    Compute numerical IK using Jacobian pseudo-inverse.

    Args:
        x, y: Target position
        q_init: Initial joint angles

    Returns:
        (joint_angles, converged)
    """
    q = np.array(q_init, dtype=float)
    target = np.array([x, y])

    for iteration in range(self.max_iterations):
        # Compute current position
        pos_current, _ = self.compute_forward_kinematics(q)
        pos_current = pos_current[:2]  # Only x, y

        # Compute error
        error = target - pos_current

        # Check convergence
        if np.linalg.norm(error) < self.epsilon:
            self.get_logger().info(f'Converged in {iteration} iterations')
            return q, True

        # Compute Jacobian (numerical)
        J = self.compute_jacobian(q)

        # Compute pseudo-inverse
        J_pinv = np.linalg.pinv(J)

        # Update joint angles
        delta_q = J_pinv @ error
        q = q + self.alpha * delta_q

    self.get_logger().warning('Did not converge')
    return q, False
```

### Step 4: Run Complete Lab

```bash
# Terminal 1: Launch IK solver
ros2 launch lab2_2_inverse_kinematics lab2_2_ik.launch.py

# Terminal 2: Send service request
ros2 service call /compute_ik example_interfaces/srv/Trigger

# Terminal 3: Run test suite
python3 test/test_ik_solver.py
```

---

## Expected Outputs

### Console Output (Analytical IK)
```
[INFO] [ik_solver]: IK Solver Node Started
[INFO] [ik_solver]: Service /compute_ik ready
[INFO] [ik_test_client]: Testing target: (1.0, 1.0)
[INFO] [ik_test_client]: Solution 1 (elbow-up): θ1=0.785, θ2=1.571 rad
[INFO] [ik_test_client]: Solution 2 (elbow-down): θ1=2.356, θ2=-1.571 rad
[INFO] [ik_test_client]: FK validation:
[INFO] [ik_test_client]:   Solution 1: (1.000, 1.000) ✓ Error: 0.000m
[INFO] [ik_test_client]:   Solution 2: (1.000, 1.000) ✓ Error: 0.000m
```

### Console Output (Numerical IK)
```
[INFO] [ik_solver]: Numerical IK for target: (1.5, 0.8)
[INFO] [ik_solver]: Iteration 0: error = 0.523m
[INFO] [ik_solver]: Iteration 1: error = 0.234m
[INFO] [ik_solver]: Iteration 2: error = 0.089m
[INFO] [ik_solver]: Iteration 3: error = 0.021m
[INFO] [ik_solver]: Iteration 4: error = 0.003m
[INFO] [ik_solver]: Converged in 4 iterations
[INFO] [ik_solver]: Solution: θ=[0.512, 0.834, 0.123] rad
```

### Test Results Table
```
╔═══════════════════════════════════════════════════════════════════════╗
║                     IK Solver Test Results                            ║
╠════════════╦═════════════╦════════════╦═══════════╦═══════════════════╣
║   Target   ║  Reachable  ║  Solutions ║    Time   ║   FK Error (mm)   ║
╠════════════╬═════════════╬════════════╬═══════════╬═══════════════════╣
║ (1.0, 0.0) ║     ✓       ║      2     ║   0.5ms   ║      0.001        ║
║ (1.2, 1.2) ║     ✓       ║      2     ║   0.6ms   ║      0.002        ║
║ (0.5, 0.5) ║     ✓       ║      2     ║   0.5ms   ║      0.001        ║
║ (2.5, 0.0) ║     ✗       ║      0     ║   0.2ms   ║       N/A         ║
╚════════════╩═════════════╩════════════╩═══════════╩═══════════════════╝

Summary: 3/4 targets reachable, 100% of solutions validated
```

---

## Verification Checklist

- [ ] Analytical IK returns correct solutions for test cases
- [ ] Both elbow-up and elbow-down solutions are computed
- [ ] Reachability check correctly rejects invalid targets
- [ ] FK validation confirms IK solutions (error < 1mm)
- [ ] Numerical IK converges for 3-DOF arm
- [ ] ROS 2 service responds within 10ms
- [ ] Test client runs all scenarios without errors
- [ ] Workspace visualization displays correctly

---

## Troubleshooting

### Issue: acos Domain Error
**Symptom**: `ValueError: math domain error` in arccos

**Solution**: Clamp cosine value to [-1, 1]:
```python
cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
```

### Issue: Numerical IK Not Converging
**Symptom**: Reaches max iterations without convergence

**Solutions**:
1. Reduce epsilon: `epsilon = 0.01` instead of `0.001`
2. Add damping: Use damped least squares
3. Better initial guess: Use analytical IK for 2-DOF subchain
4. Increase step size: `alpha = 0.5` instead of `0.1`

### Issue: Wrong Elbow Configuration
**Symptom**: IK returns elbow-down when elbow-up is expected

**Solution**: Add configuration preference parameter:
```python
def compute_ik(x, y, prefer_elbow_up=True):
    solutions = compute_analytical_ik(x, y)
    if prefer_elbow_up:
        return solutions[0]  # Elbow-up
    else:
        return solutions[1]  # Elbow-down
```

### Issue: Jacobian Singularity
**Symptom**: Numerical IK produces large joint velocities

**Solution**: Use damped least squares:
```python
lambda_damping = 0.01
J_damped = J.T @ np.linalg.inv(J @ J.T + lambda_damping**2 * np.eye(J.shape[0]))
delta_q = J_damped @ error
```

---

## Extensions & Challenges

### Challenge 1: Obstacle Avoidance
Modify IK solver to reject solutions where the arm collides with obstacles:
- Define obstacle as circle: center (x, y), radius r
- Check if link segments intersect obstacle
- Return only collision-free solutions

### Challenge 2: Joint Limit Constraints
Add joint angle limits and modify IK to respect them:
```python
joint_limits = [(-π, π), (-π, π), (-π, π)]
```
Use constrained optimization (SciPy's minimize with bounds).

### Challenge 3: Velocity IK
Implement velocity-level IK:
- Given desired end-effector velocity v
- Compute required joint velocities: q̇ = J⁺ · v
- Publish to `/joint_velocity_command`

### Challenge 4: Redundancy Resolution
For 3-DOF reaching 2D target (redundant system):
- Primary task: Reach target position
- Secondary task: Minimize elbow height
Use null-space projection:
```python
q̇ = J⁺ · v + (I - J⁺J) · q̇₀
```

---

## Assessment Rubric

| Criterion | Points | Description |
|-----------|--------|-------------|
| Analytical IK | 30 | Correct implementation with both solutions |
| Numerical IK | 25 | Converges for 3-DOF arm within 10 iterations |
| ROS 2 Service | 15 | Proper service interface and error handling |
| Validation | 15 | FK validation confirms IK accuracy < 1mm |
| Test Coverage | 10 | Comprehensive test cases including edge cases |
| Documentation | 5 | Clear comments and explanations |
| **Total** | **100** | |

**Passing Score**: 70/100

---

## Next Steps

After completing this lab:
1. Compare computation time: analytical vs numerical
2. Analyze convergence behavior for different initial guesses
3. Visualize workspace and reachable regions
4. Prepare for Lab 2.3: Dynamics Simulation

**Proceed to**: `lab2_3_dynamics_simulation/README.md`

---

**Lab 2.2 Version**: 1.0
**Last Updated**: 2026-01-03
