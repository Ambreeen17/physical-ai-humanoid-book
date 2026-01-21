---
sidebar_position: 7
title: "Chapter 6: Control Theory"
---

# Chapter 6: Control Theory for Robotics

<PersonalizationToggle chapterId="6" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Implement PID controllers** with proper tuning for position and velocity control
2. **Design trajectory tracking controllers** for smooth robot motion
3. **Apply Linear Quadratic Regulator (LQR)** for optimal control
4. **Implement impedance and force control** for safe manipulation
5. **Use Model Predictive Control (MPC)** for constrained optimization

---

## Introduction

You've learned how robots perceive their world through sensors and vision. Now comes the critical question: *How does a robot actually move?*

A robot that knows exactly where it is and where it wants to go still needs to compute the right motor commands to get there. Send too much current, and the arm overshoots. Send too little, and it never reaches the target. Send the wrong sequence, and the motion is jerky and unsafe.

**Control theory** is the mathematical framework that bridges perception and action. It's the science of making systems behave as desired, despite disturbances, model errors, and physical constraints.

In this chapter, you'll learn the control algorithms that make robots move smoothly, track trajectories precisely, and interact safely with their environment.

---

## Section 6.1: Feedback Control Fundamentals

### The Control Loop

Every robot controller follows this fundamental pattern:

```
Reference ──►[Controller]──► Command ──►[Robot]──► State
    ▲                                               │
    │                                               │
    └──────────────── Error ◄──[Sensor]◄────────────┘
```

**Key components:**
- **Reference (r)**: Desired state (position, velocity, force)
- **Error (e)**: Difference between reference and actual state
- **Controller**: Algorithm that computes commands from error
- **Plant**: The robot system being controlled
- **Sensor**: Measures actual state

### Open-Loop vs Closed-Loop Control

**Open-loop**: Command → Robot → Output (no feedback)
- Problem: No correction for disturbances or model errors

**Closed-loop**: Command → Robot → Output → Feedback → Correction
- Advantage: Automatically compensates for errors

```python
# Open-loop: hopes the motor moves 90 degrees
motor.set_position(90)

# Closed-loop: keeps adjusting until we're there
while abs(current_position - target_position) > tolerance:
    error = target_position - current_position
    command = controller.compute(error)
    motor.set_command(command)
    current_position = encoder.read()
```

---

## Section 6.2: PID Control

### The PID Controller

The **Proportional-Integral-Derivative (PID)** controller is the workhorse of industrial control:

```
u(t) = Kp·e(t) + Ki·∫e(τ)dτ + Kd·(de/dt)
```

**Components:**
- **P (Proportional)**: Reacts to current error
- **I (Integral)**: Eliminates steady-state error
- **D (Derivative)**: Dampens oscillations

### Python Implementation

```python
import numpy as np
import time

class PIDController:
    """
    PID controller with anti-windup and derivative filtering.
    """
    def __init__(self, Kp, Ki, Kd, dt=0.01,
                 output_limits=(-1.0, 1.0),
                 integral_limits=(-10.0, 10.0)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.dt = dt

        self.output_limits = output_limits
        self.integral_limits = integral_limits

        # State
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0

        # Derivative filter coefficient (0.0-1.0)
        self.alpha = 0.1  # Low-pass filter

    def compute(self, error):
        """
        Compute PID output given current error.
        """
        # Proportional term
        P = self.Kp * error

        # Integral term with anti-windup
        self.integral += error * self.dt
        self.integral = np.clip(self.integral,
                                self.integral_limits[0],
                                self.integral_limits[1])
        I = self.Ki * self.integral

        # Derivative term with filtering
        derivative = (error - self.prev_error) / self.dt
        # Low-pass filter to reduce noise
        derivative = self.alpha * derivative + (1 - self.alpha) * self.prev_derivative
        D = self.Kd * derivative

        self.prev_error = error
        self.prev_derivative = derivative

        # Compute output with saturation
        output = P + I + D
        output = np.clip(output, self.output_limits[0], self.output_limits[1])

        return output

    def reset(self):
        """Reset controller state."""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_derivative = 0.0


# Example: Position control
pid = PIDController(Kp=10.0, Ki=1.0, Kd=2.0, dt=0.01)

target_position = 1.0
current_position = 0.0
velocity = 0.0

for step in range(500):
    error = target_position - current_position
    force = pid.compute(error)

    # Simple dynamics simulation
    acceleration = force - 0.1 * velocity  # with damping
    velocity += acceleration * 0.01
    current_position += velocity * 0.01

    if step % 50 == 0:
        print(f"Step {step}: pos={current_position:.3f}, error={error:.3f}")
```

### PID Tuning Methods

**Ziegler-Nichols Method:**
1. Set Ki = Kd = 0
2. Increase Kp until sustained oscillation (critical gain Ku)
3. Measure oscillation period Tu
4. Use tuning rules:

| Controller | Kp | Ki | Kd |
|------------|----|----|-----|
| P | 0.5·Ku | 0 | 0 |
| PI | 0.45·Ku | 0.54·Ku/Tu | 0 |
| PID | 0.6·Ku | 1.2·Ku/Tu | 0.075·Ku·Tu |

:::caution Gotcha: Derivative Kick
When the setpoint changes suddenly, the derivative term spikes (derivative kick). Solutions:
1. Filter the derivative
2. Compute derivative on measurement, not error
3. Use setpoint ramping
:::

---

## Section 6.3: Trajectory Tracking

### What Is a Trajectory?

A **trajectory** specifies desired position, velocity, and acceleration over time:

```python
class Trajectory:
    """
    Represents a time-parameterized path.
    """
    def __init__(self):
        self.waypoints = []
        self.durations = []

    def evaluate(self, t):
        """
        Get desired position, velocity, acceleration at time t.
        """
        # Interpolate between waypoints
        pass

    @staticmethod
    def trapezoidal_profile(start, end, v_max, a_max, dt=0.01):
        """
        Generate trapezoidal velocity profile.

        Returns arrays of position, velocity, acceleration over time.
        """
        distance = abs(end - start)
        direction = 1 if end > start else -1

        # Time to accelerate to max velocity
        t_accel = v_max / a_max

        # Distance covered during acceleration
        d_accel = 0.5 * a_max * t_accel**2

        if 2 * d_accel > distance:
            # Triangular profile (can't reach max velocity)
            t_accel = np.sqrt(distance / a_max)
            t_cruise = 0
            t_total = 2 * t_accel
        else:
            # Trapezoidal profile
            d_cruise = distance - 2 * d_accel
            t_cruise = d_cruise / v_max
            t_total = 2 * t_accel + t_cruise

        # Generate profile
        times = np.arange(0, t_total + dt, dt)
        positions = []
        velocities = []
        accelerations = []

        for t in times:
            if t < t_accel:
                # Acceleration phase
                a = a_max * direction
                v = a_max * t * direction
                p = start + 0.5 * a_max * t**2 * direction
            elif t < t_accel + t_cruise:
                # Cruise phase
                a = 0
                v = v_max * direction
                p = start + (d_accel + v_max * (t - t_accel)) * direction
            else:
                # Deceleration phase
                t_decel = t - t_accel - t_cruise
                a = -a_max * direction
                v = (v_max - a_max * t_decel) * direction
                p = start + (d_accel + v_max * t_cruise + v_max * t_decel - 0.5 * a_max * t_decel**2) * direction

            positions.append(p)
            velocities.append(v)
            accelerations.append(a)

        return times, np.array(positions), np.array(velocities), np.array(accelerations)
```

### Computed Torque Control

For robot arms, we can use the **inverse dynamics** to compute required torques:

```python
import numpy as np

class ComputedTorqueController:
    """
    Computed torque control for robot manipulators.

    Uses inverse dynamics:
    τ = M(q)·(q̈_d + Kd·ė + Kp·e) + C(q, q̇)·q̇ + g(q)

    where:
    - M(q): Mass matrix
    - C(q, q̇): Coriolis/centrifugal matrix
    - g(q): Gravity vector
    """
    def __init__(self, robot_model, Kp, Kd):
        self.robot = robot_model
        self.Kp = np.diag(Kp)  # Position gains
        self.Kd = np.diag(Kd)  # Velocity gains

    def compute(self, q, q_dot, q_des, q_dot_des, q_ddot_des):
        """
        Compute torques for trajectory tracking.

        Args:
            q: Current joint positions
            q_dot: Current joint velocities
            q_des: Desired joint positions
            q_dot_des: Desired joint velocities
            q_ddot_des: Desired joint accelerations

        Returns:
            tau: Joint torques
        """
        # Compute errors
        e = q_des - q
        e_dot = q_dot_des - q_dot

        # Compute PD command with feedforward
        q_ddot_cmd = q_ddot_des + self.Kd @ e_dot + self.Kp @ e

        # Inverse dynamics
        M = self.robot.mass_matrix(q)
        C = self.robot.coriolis_matrix(q, q_dot)
        g = self.robot.gravity_vector(q)

        tau = M @ q_ddot_cmd + C @ q_dot + g

        return tau
```

---

## Section 6.4: Linear Quadratic Regulator (LQR)

### Optimal Control

**LQR** finds the control that minimizes a quadratic cost:

```
J = ∫(x'Qx + u'Ru)dt
```

Where:
- Q: State cost matrix (penalizes deviation from reference)
- R: Control cost matrix (penalizes control effort)

### LQR Solution

For linear system ẋ = Ax + Bu, the optimal control is:

```
u = -Kx
```

Where K is computed by solving the Riccati equation.

```python
import numpy as np
from scipy.linalg import solve_continuous_are

class LQRController:
    """
    Linear Quadratic Regulator for linear systems.
    """
    def __init__(self, A, B, Q, R):
        """
        Args:
            A: System matrix (n x n)
            B: Input matrix (n x m)
            Q: State cost matrix (n x n)
            R: Control cost matrix (m x m)
        """
        self.A = A
        self.B = B
        self.Q = Q
        self.R = R

        # Solve Riccati equation
        P = solve_continuous_are(A, B, Q, R)

        # Compute optimal gain
        self.K = np.linalg.inv(R) @ B.T @ P

        print(f"LQR gain matrix K:\n{self.K}")

    def compute(self, x, x_ref=None):
        """
        Compute optimal control input.

        Args:
            x: Current state
            x_ref: Reference state (default: origin)

        Returns:
            u: Optimal control input
        """
        if x_ref is None:
            x_ref = np.zeros_like(x)

        error = x - x_ref
        u = -self.K @ error

        return u


# Example: Cart-pole balancing
# State: [x, x_dot, theta, theta_dot]
# Input: force on cart

# Linearized system matrices (around upright equilibrium)
A = np.array([
    [0, 1, 0, 0],
    [0, 0, -9.81, 0],  # gravity
    [0, 0, 0, 1],
    [0, 0, 19.62, 0]   # gravity effect on pole
])

B = np.array([
    [0],
    [1],
    [0],
    [-1]
])

# Cost matrices
Q = np.diag([1, 0.1, 10, 0.1])  # Penalize angle more than position
R = np.array([[0.01]])          # Low control cost

lqr = LQRController(A, B, Q, R)

# Simulate
x = np.array([0.1, 0, 0.1, 0])  # Initial state: slightly displaced
for step in range(100):
    u = lqr.compute(x)
    # Euler integration
    x_dot = A @ x + B @ u.flatten()
    x = x + x_dot * 0.01

    if step % 20 == 0:
        print(f"Step {step}: x={x[0]:.3f}, theta={x[2]:.3f}")
```

:::note Foundations Box: LQR Optimality
LQR provides the globally optimal linear feedback for linear systems with quadratic costs. For nonlinear systems, we can:
1. Linearize around operating point (local LQR)
2. Use iterative LQR (iLQR) for trajectory optimization
3. Apply differential dynamic programming (DDP)
:::

---

## Section 6.5: Impedance Control

### Why Impedance Control?

Position control alone fails when robots interact with the environment:
- Rigid position control during contact → huge forces
- Compliant environments require force awareness

**Impedance control** makes the robot behave like a spring-damper system:

```
F = K(x_d - x) + B(ẋ_d - ẋ)
```

### Implementation

```python
class ImpedanceController:
    """
    Cartesian impedance controller for robot manipulators.

    Makes end-effector behave like: M·ẍ + B·ẋ + K·x = F_ext
    """
    def __init__(self, robot_model, K_cart, B_cart, M_cart=None):
        """
        Args:
            K_cart: Cartesian stiffness (6x6)
            B_cart: Cartesian damping (6x6)
            M_cart: Cartesian inertia (6x6, optional)
        """
        self.robot = robot_model
        self.K = np.diag(K_cart) if len(K_cart.shape) == 1 else K_cart
        self.B = np.diag(B_cart) if len(B_cart.shape) == 1 else B_cart
        self.M = M_cart

    def compute(self, q, q_dot, x_des, x_dot_des=None, F_ext=None):
        """
        Compute joint torques for impedance behavior.

        Args:
            q: Joint positions
            q_dot: Joint velocities
            x_des: Desired Cartesian pose (6D: position + orientation)
            x_dot_des: Desired Cartesian velocity
            F_ext: External force (measured or estimated)

        Returns:
            tau: Joint torques
        """
        # Forward kinematics
        x = self.robot.forward_kinematics(q)
        J = self.robot.jacobian(q)
        x_dot = J @ q_dot

        if x_dot_des is None:
            x_dot_des = np.zeros(6)
        if F_ext is None:
            F_ext = np.zeros(6)

        # Cartesian error
        x_error = x_des - x
        x_dot_error = x_dot_des - x_dot

        # Impedance force
        F_impedance = self.K @ x_error + self.B @ x_dot_error

        # Map to joint torques
        tau_impedance = J.T @ F_impedance

        # Add gravity compensation
        tau_gravity = self.robot.gravity_vector(q)

        return tau_impedance + tau_gravity

    def set_stiffness(self, K_new):
        """Dynamically adjust stiffness (variable impedance)."""
        self.K = np.diag(K_new) if len(K_new.shape) == 1 else K_new


# Example usage
K_cart = np.array([500, 500, 500, 50, 50, 50])   # N/m, Nm/rad
B_cart = np.array([50, 50, 50, 5, 5, 5])         # Damping

controller = ImpedanceController(robot, K_cart, B_cart)

# During peg-in-hole insertion, reduce lateral stiffness
K_compliant = np.array([100, 100, 500, 50, 50, 50])
controller.set_stiffness(K_compliant)
```

---

## Section 6.6: Model Predictive Control (MPC)

### What Is MPC?

**Model Predictive Control** optimizes control over a future horizon:

1. Predict future states using a model
2. Optimize control sequence to minimize cost
3. Apply first control, then re-plan

```
At each timestep:
    Solve: min_{u_0,...,u_{N-1}} Σ(cost(x_k, u_k))
    Subject to: x_{k+1} = f(x_k, u_k)
                constraints on x and u

    Apply: u_0
```

### MPC Implementation

```python
import numpy as np
from scipy.optimize import minimize

class MPC:
    """
    Model Predictive Controller for nonlinear systems.
    """
    def __init__(self, dynamics, cost_fn, horizon=10, dt=0.1,
                 state_dim=4, control_dim=2,
                 u_min=None, u_max=None):
        self.dynamics = dynamics
        self.cost_fn = cost_fn
        self.horizon = horizon
        self.dt = dt
        self.state_dim = state_dim
        self.control_dim = control_dim
        self.u_min = u_min if u_min is not None else -np.inf * np.ones(control_dim)
        self.u_max = u_max if u_max is not None else np.inf * np.ones(control_dim)

    def predict(self, x0, u_sequence):
        """
        Predict trajectory given initial state and control sequence.
        """
        trajectory = [x0]
        x = x0.copy()

        for u in u_sequence:
            x = self.dynamics(x, u, self.dt)
            trajectory.append(x)

        return np.array(trajectory)

    def optimize(self, x0, x_ref):
        """
        Solve MPC optimization problem.

        Args:
            x0: Current state
            x_ref: Reference trajectory (horizon+1 x state_dim)

        Returns:
            u_optimal: Optimal control sequence
        """
        # Initial guess: zero controls
        u_init = np.zeros(self.horizon * self.control_dim)

        # Bounds
        bounds = [(self.u_min[i % self.control_dim],
                   self.u_max[i % self.control_dim])
                  for i in range(self.horizon * self.control_dim)]

        def objective(u_flat):
            u_sequence = u_flat.reshape(self.horizon, self.control_dim)
            trajectory = self.predict(x0, u_sequence)

            total_cost = 0
            for k in range(self.horizon + 1):
                x_k = trajectory[k]
                u_k = u_sequence[k] if k < self.horizon else np.zeros(self.control_dim)
                ref_k = x_ref[k] if k < len(x_ref) else x_ref[-1]
                total_cost += self.cost_fn(x_k, u_k, ref_k)

            return total_cost

        result = minimize(objective, u_init, method='SLSQP', bounds=bounds)

        u_optimal = result.x.reshape(self.horizon, self.control_dim)
        return u_optimal

    def compute(self, x, x_ref):
        """
        Compute next control action using MPC.
        """
        u_sequence = self.optimize(x, x_ref)
        return u_sequence[0]  # Return first action


# Example: Differential drive robot
def robot_dynamics(x, u, dt):
    """
    State: [x, y, theta, v]
    Control: [acceleration, steering_rate]
    """
    px, py, theta, v = x
    a, omega = u

    px_new = px + v * np.cos(theta) * dt
    py_new = py + v * np.sin(theta) * dt
    theta_new = theta + omega * dt
    v_new = v + a * dt

    return np.array([px_new, py_new, theta_new, v_new])

def tracking_cost(x, u, x_ref):
    """Quadratic cost for trajectory tracking."""
    Q = np.diag([10, 10, 1, 0.1])  # State cost
    R = np.diag([0.1, 0.1])        # Control cost

    x_error = x - x_ref
    cost = x_error @ Q @ x_error + u @ R @ u
    return cost

mpc = MPC(
    dynamics=robot_dynamics,
    cost_fn=tracking_cost,
    horizon=20,
    dt=0.1,
    state_dim=4,
    control_dim=2,
    u_min=np.array([-2.0, -1.0]),
    u_max=np.array([2.0, 1.0])
)
```

---

## Section 6.7: ROS 2 Control Integration

### ros2_control Framework

```python
# Joint trajectory controller configuration
# controller_config.yaml

controller_manager:
  ros__parameters:
    update_rate: 500  # Hz

    joint_trajectory_controller:
      type: joint_trajectory_controller/JointTrajectoryController

    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster

joint_trajectory_controller:
  ros__parameters:
    joints:
      - joint_1
      - joint_2
      - joint_3
      - joint_4
      - joint_5
      - joint_6

    command_interfaces:
      - position

    state_interfaces:
      - position
      - velocity

    state_publish_rate: 100.0
    action_monitor_rate: 20.0

    constraints:
      goal_time: 0.0
      joint_1:
        goal: 0.05
```

### Custom Controller Plugin

```cpp
// custom_controller.hpp
#include "controller_interface/controller_interface.hpp"

class CustomController : public controller_interface::ControllerInterface
{
public:
    controller_interface::InterfaceConfiguration command_interface_configuration() const override;
    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> command_interfaces_;
    std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> state_interfaces_;

    PIDController pid_;
};
```

---

## Section 6.8: Hardware Considerations

### Unitree G1 Control Architecture

| Level | Rate | Controller | Task |
|-------|------|------------|------|
| High-level | 50 Hz | MPC/RL Policy | Locomotion planning |
| Mid-level | 200 Hz | Whole-body control | Torque distribution |
| Low-level | 1 kHz | PD + Feedforward | Joint servo |

### Motor Control for Humanoids

```python
class JointController:
    """
    Low-level joint controller for Unitree G1.
    Runs at 1kHz on embedded hardware.
    """
    def __init__(self, joint_id, Kp=50, Kd=2):
        self.joint_id = joint_id
        self.Kp = Kp
        self.Kd = Kd

    def compute_torque(self, q_des, q_dot_des, q, q_dot, tau_ff=0):
        """
        PD control with feedforward torque.

        Args:
            q_des: Desired position
            q_dot_des: Desired velocity
            q: Current position
            q_dot: Current velocity
            tau_ff: Feedforward torque (from inverse dynamics)

        Returns:
            tau: Motor torque command
        """
        tau = self.Kp * (q_des - q) + self.Kd * (q_dot_des - q_dot) + tau_ff
        return tau
```

---

## Summary

### Key Takeaways

1. **PID control** is the foundation—understand tuning and limitations

2. **Trajectory tracking** requires feedforward terms for good performance

3. **LQR provides optimal linear control** when you can model the system

4. **Impedance control** is essential for safe physical interaction

5. **MPC handles constraints** and is state-of-the-art for complex systems

### Looking Ahead

In Chapter 7, we'll explore **Motion Planning**—how to find collision-free paths through complex environments.

---

## Exercises

1. **PID Tuning**: Implement Ziegler-Nichols tuning and compare with manual tuning.

2. **Trajectory Generation**: Create smooth minimum-jerk trajectories for a 2-DOF arm.

3. **LQR Design**: Design an LQR controller for an inverted pendulum and analyze stability.

4. **Impedance Control**: Implement variable impedance that increases stiffness near obstacles.

5. **MPC Comparison**: Compare MPC with PID for trajectory tracking with constraints.

---

**Chapter completed**: 2026-01-20
**Chapter**: 6 - Control Theory
