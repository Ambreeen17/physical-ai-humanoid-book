---
sidebar_position: 5
title: "Chapter 4: State Estimation"
---

# Chapter 4: State Estimation

<PersonalizationToggle chapterId="4" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Implement Kalman Filters** for linear state estimation with sensor fusion
2. **Apply Extended Kalman Filters (EKF)** for nonlinear robotic systems
3. **Understand Particle Filters** for non-Gaussian, multimodal distributions
4. **Fuse IMU, odometry, and visual data** for robust robot localization
5. **Evaluate estimation quality** using NEES, covariance analysis, and ground truth comparison

---

## Introduction

Imagine you're blindfolded and asked to walk across a room. You can feel the floor beneath your feet, hear echoes from walls, and sense your body's acceleration—but you can't see. How do you know where you are?

This is the fundamental challenge every robot faces: **determining its true state** (position, velocity, orientation) from imperfect, noisy sensor measurements. A robot's encoders drift, its IMU accumulates bias, its cameras see only snapshots. Yet somehow, the robot must maintain an accurate internal model of reality.

**State estimation** is the mathematical framework that solves this problem. It's the robot's internal GPS—except it works indoors, underwater, on Mars, and anywhere GPS cannot reach.

In this chapter, you'll learn the algorithms that power every autonomous vehicle, every drone, and every humanoid robot's sense of self-location. By the end, you'll implement a sensor fusion system that combines wheel odometry with IMU data to track a robot's pose in real-time.

---

## Section 4.1: The State Estimation Problem

### What Is State?

In robotics, **state** refers to the minimal set of variables that completely describes a system at a given moment. For a mobile robot, this typically includes:

```
State vector x = [x, y, θ, vx, vy, ω]ᵀ
```

Where:
- `x, y`: Position in 2D space
- `θ`: Heading/orientation (yaw angle)
- `vx, vy`: Linear velocities
- `ω`: Angular velocity (yaw rate)

For a humanoid like Unitree G1, the state vector explodes to include 23 joint positions, 23 joint velocities, base pose, and contact states—potentially 100+ variables.

### Why Is State Estimation Hard?

**Problem 1: Sensors Lie**

Every sensor measurement contains noise:
```python
# What the encoder "sees" vs. reality
true_position = 1.0  # meters
encoder_noise = np.random.normal(0, 0.02)  # 2cm standard deviation
measured_position = true_position + encoder_noise  # 0.98 - 1.02 meters
```

**Problem 2: Models Are Wrong**

Our motion models are approximations:
```python
# Simple motion model
x_new = x_old + v * dt

# Reality includes:
# - Wheel slip on different surfaces
# - Actuator delays and backlash
# - Unmodeled friction and drag
```

**Problem 3: Information Is Incomplete**

No single sensor provides complete state information:
- **IMU**: Provides acceleration and angular velocity, but not position
- **Wheel encoders**: Provide displacement, but drift over time
- **Camera**: Provides relative position to landmarks, but not velocity
- **GPS**: Provides absolute position, but not orientation or velocity

:::tip The Estimation Philosophy
State estimation is fundamentally about **optimal information fusion**: combining multiple imperfect sources to produce an estimate better than any single source alone.
:::

### Probabilistic State Representation

Instead of claiming "the robot is at position (3.2, 4.1)", we represent state as a **probability distribution**:

```
P(x | z₁, z₂, ..., zₙ)
```

"Given all measurements z₁ through zₙ, what's the probability the robot is at state x?"

For Gaussian distributions, this reduces to tracking two quantities:
- **Mean** (μ): Our best estimate
- **Covariance** (Σ): Our uncertainty

---

## Section 4.2: The Kalman Filter

### Historical Context

In 1960, Rudolf Kálmán published a paper that would become one of the most influential in engineering history. The **Kalman Filter** provided an optimal solution to the linear state estimation problem and enabled:
- The Apollo moon landing navigation
- Modern GPS receivers
- Every smartphone's motion tracking
- Autonomous vehicle localization

### The Kalman Filter Algorithm

The Kalman Filter operates in two steps:

**Step 1: Prediction** (Time Update)
```
x̂ₖ⁻ = A · x̂ₖ₋₁ + B · uₖ₋₁     # Predict state
Pₖ⁻ = A · Pₖ₋₁ · Aᵀ + Q        # Predict covariance
```

**Step 2: Update** (Measurement Update)
```
Kₖ = Pₖ⁻ · Hᵀ · (H · Pₖ⁻ · Hᵀ + R)⁻¹   # Kalman gain
x̂ₖ = x̂ₖ⁻ + Kₖ · (zₖ - H · x̂ₖ⁻)        # Update state
Pₖ = (I - Kₖ · H) · Pₖ⁻                  # Update covariance
```

Where:
- `x̂`: State estimate
- `P`: State covariance (uncertainty)
- `A`: State transition matrix
- `B`: Control input matrix
- `H`: Observation matrix
- `Q`: Process noise covariance
- `R`: Measurement noise covariance
- `K`: Kalman gain (how much to trust measurements)

### Intuition: The Kalman Gain

The **Kalman gain K** determines how much the filter trusts new measurements versus its predictions:

```
K ≈ Prediction_Uncertainty / (Prediction_Uncertainty + Measurement_Uncertainty)
```

- **High K (close to 1)**: Trust measurements more (prediction is uncertain)
- **Low K (close to 0)**: Trust predictions more (measurements are noisy)

### Python Implementation

```python
import numpy as np

class KalmanFilter:
    """
    1D Kalman Filter for position tracking from noisy velocity commands
    and position measurements.
    """
    def __init__(self, dt=0.1):
        # State: [position, velocity]
        self.x = np.array([[0.0],   # Initial position
                          [0.0]])   # Initial velocity

        # State transition matrix
        self.A = np.array([[1, dt],
                          [0, 1]])

        # Control input matrix (acceleration command)
        self.B = np.array([[0.5 * dt**2],
                          [dt]])

        # Observation matrix (we only measure position)
        self.H = np.array([[1, 0]])

        # Initial covariance
        self.P = np.eye(2) * 1.0

        # Process noise (model uncertainty)
        self.Q = np.array([[0.01, 0],
                          [0, 0.01]])

        # Measurement noise (sensor uncertainty)
        self.R = np.array([[0.1]])

    def predict(self, u=0):
        """Prediction step with control input u (acceleration)"""
        # Predict state
        self.x = self.A @ self.x + self.B * u

        # Predict covariance
        self.P = self.A @ self.P @ self.A.T + self.Q

        return self.x.flatten()

    def update(self, z):
        """Update step with measurement z (position)"""
        # Innovation (measurement residual)
        y = z - self.H @ self.x

        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R

        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)

        # Update state
        self.x = self.x + K @ y

        # Update covariance
        I = np.eye(self.P.shape[0])
        self.P = (I - K @ self.H) @ self.P

        return self.x.flatten()

# Example usage
kf = KalmanFilter(dt=0.1)

# Simulate robot moving with noisy measurements
true_positions = []
measured_positions = []
estimated_positions = []

true_pos = 0.0
true_vel = 1.0  # Moving at 1 m/s

for t in range(100):
    # Ground truth
    true_pos += true_vel * 0.1
    true_positions.append(true_pos)

    # Noisy measurement
    measured = true_pos + np.random.normal(0, 0.3)
    measured_positions.append(measured)

    # Kalman filter
    kf.predict(u=0)
    estimated = kf.update(np.array([[measured]]))[0]
    estimated_positions.append(estimated)

print(f"Final true position: {true_pos:.3f}")
print(f"Final measurement: {measured_positions[-1]:.3f}")
print(f"Final estimate: {estimated_positions[-1]:.3f}")
```

:::note Foundations Box: Why Kalman Is Optimal
The Kalman Filter is the **minimum variance unbiased estimator** for linear systems with Gaussian noise. This means no other linear estimator can achieve lower estimation error on average. The proof relies on minimizing the trace of the covariance matrix Tr(P).
:::

---

## Section 4.3: Extended Kalman Filter (EKF)

### The Nonlinearity Problem

The standard Kalman Filter assumes linear system dynamics:
```
x_new = A · x_old + B · u
```

But most robotics systems are **nonlinear**. Consider a differential-drive robot:

```python
# Nonlinear motion model
x_new = x_old + v * cos(θ) * dt
y_new = y_old + v * sin(θ) * dt
θ_new = θ_old + ω * dt
```

The `cos(θ)` and `sin(θ)` terms make this nonlinear—there's no matrix A that captures this relationship.

### EKF: Linearization via Jacobians

The **Extended Kalman Filter** solves this by **linearizing** the system at each time step:

1. Compute the **Jacobian** (matrix of partial derivatives) of the motion model
2. Use this linearized model for the Kalman Filter equations
3. Re-linearize at each new state estimate

**Motion Model Jacobian:**
```
        ∂f/∂x   ∂f/∂y   ∂f/∂θ
F = [    1       0      -v·sin(θ)·dt ]
    [    0       1       v·cos(θ)·dt ]
    [    0       0       1           ]
```

### EKF for Robot Localization

```python
import numpy as np

class EKFLocalization:
    """
    Extended Kalman Filter for differential-drive robot localization.
    State: [x, y, θ]
    """
    def __init__(self):
        # State: [x, y, theta]
        self.x = np.zeros(3)

        # Covariance
        self.P = np.diag([0.1, 0.1, 0.05])

        # Process noise
        self.Q = np.diag([0.01, 0.01, 0.005])

        # Measurement noise (GPS-like position measurement)
        self.R = np.diag([0.5, 0.5])

    def predict(self, v, omega, dt):
        """
        Prediction step with velocity commands.
        v: linear velocity
        omega: angular velocity
        dt: time step
        """
        theta = self.x[2]

        # Nonlinear motion model
        if abs(omega) > 1e-6:
            # Curved motion
            dx = -v/omega * np.sin(theta) + v/omega * np.sin(theta + omega*dt)
            dy = v/omega * np.cos(theta) - v/omega * np.cos(theta + omega*dt)
        else:
            # Straight motion
            dx = v * np.cos(theta) * dt
            dy = v * np.sin(theta) * dt

        dtheta = omega * dt

        # Update state
        self.x[0] += dx
        self.x[1] += dy
        self.x[2] += dtheta

        # Normalize angle to [-π, π]
        self.x[2] = np.arctan2(np.sin(self.x[2]), np.cos(self.x[2]))

        # Compute Jacobian of motion model
        F = np.array([
            [1, 0, -v * np.sin(theta) * dt],
            [0, 1,  v * np.cos(theta) * dt],
            [0, 0,  1]
        ])

        # Update covariance
        self.P = F @ self.P @ F.T + self.Q

        return self.x.copy()

    def update(self, z_x, z_y):
        """
        Update with position measurement (e.g., from landmarks or GPS).
        """
        # Observation matrix (we measure x and y directly)
        H = np.array([
            [1, 0, 0],
            [0, 1, 0]
        ])

        # Innovation
        z = np.array([z_x, z_y])
        y = z - H @ self.x

        # Innovation covariance
        S = H @ self.P @ H.T + self.R

        # Kalman gain
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        self.x = self.x + K @ y

        # Update covariance
        I = np.eye(3)
        self.P = (I - K @ H) @ self.P

        return self.x.copy()

# Demo: Robot driving in a circle
ekf = EKFLocalization()
v = 0.5  # m/s
omega = 0.2  # rad/s
dt = 0.1

for i in range(100):
    # Predict
    ekf.predict(v, omega, dt)

    # Every 10 steps, get a "GPS" measurement (with noise)
    if i % 10 == 0:
        true_x = ekf.x[0]  # In real system, this would be actual GPS
        true_y = ekf.x[1]
        noisy_x = true_x + np.random.normal(0, 0.3)
        noisy_y = true_y + np.random.normal(0, 0.3)
        ekf.update(noisy_x, noisy_y)

    if i % 20 == 0:
        print(f"Step {i}: Position ({ekf.x[0]:.2f}, {ekf.x[1]:.2f}), "
              f"Heading {np.degrees(ekf.x[2]):.1f}°")
```

:::caution Gotcha: EKF Linearization Errors
The EKF assumes the system is "approximately linear" near the current estimate. For highly nonlinear systems or large uncertainties, this assumption breaks down. If your covariance grows unbounded or estimates diverge, consider:
1. Using Unscented Kalman Filter (UKF)
2. Using particle filters
3. Reducing prediction time steps
:::

---

## Section 4.4: Particle Filters

### Beyond Gaussians

Both Kalman Filter and EKF assume **Gaussian uncertainty**—the posterior distribution is always a bell curve. But reality isn't always Gaussian:

- **Multimodal distributions**: The robot might be in one of several rooms
- **Bounded constraints**: Position can't be negative or inside walls
- **Non-Gaussian sensors**: Lidar returns may have outliers

**Particle Filters** (also called Sequential Monte Carlo) handle arbitrary distributions by representing beliefs as a set of **weighted samples**.

### How Particle Filters Work

1. **Initialize**: Scatter N particles randomly (or around initial guess)
2. **Predict**: Move each particle according to motion model + noise
3. **Update**: Weight each particle by how well it matches measurements
4. **Resample**: Clone particles with high weights, discard low-weight ones
5. **Repeat**

```python
import numpy as np

class ParticleFilter:
    """
    Particle Filter for robot localization.
    """
    def __init__(self, num_particles=1000, map_bounds=(-10, 10)):
        self.num_particles = num_particles
        self.bounds = map_bounds

        # Initialize particles uniformly
        self.particles = np.random.uniform(
            map_bounds[0], map_bounds[1],
            (num_particles, 3)  # [x, y, theta]
        )
        self.particles[:, 2] = np.random.uniform(-np.pi, np.pi, num_particles)

        # Uniform weights
        self.weights = np.ones(num_particles) / num_particles

    def predict(self, v, omega, dt, noise_std=[0.1, 0.1, 0.05]):
        """
        Move particles according to motion model with noise.
        """
        for i in range(self.num_particles):
            # Add noise to control inputs
            v_noisy = v + np.random.normal(0, noise_std[0])
            omega_noisy = omega + np.random.normal(0, noise_std[2])

            theta = self.particles[i, 2]

            # Motion model
            self.particles[i, 0] += v_noisy * np.cos(theta) * dt
            self.particles[i, 1] += v_noisy * np.sin(theta) * dt
            self.particles[i, 2] += omega_noisy * dt

    def update(self, z, measurement_std=0.5):
        """
        Update weights based on measurement z = [x, y].
        """
        for i in range(self.num_particles):
            # Distance from particle to measurement
            dist = np.sqrt(
                (self.particles[i, 0] - z[0])**2 +
                (self.particles[i, 1] - z[1])**2
            )

            # Gaussian likelihood
            self.weights[i] = np.exp(-0.5 * (dist / measurement_std)**2)

        # Normalize weights
        self.weights /= self.weights.sum()

    def resample(self):
        """
        Resample particles based on weights (systematic resampling).
        """
        cumsum = np.cumsum(self.weights)
        indices = np.searchsorted(
            cumsum,
            np.random.uniform(0, 1, self.num_particles)
        )

        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles

    def estimate(self):
        """
        Return weighted mean estimate.
        """
        return np.average(self.particles, weights=self.weights, axis=0)

# Demo: Kidnapped robot problem
pf = ParticleFilter(num_particles=500)

# Robot is actually at (2, 3)
true_position = [2.0, 3.0, 0.5]

for step in range(50):
    # Robot moves forward
    pf.predict(v=0.3, omega=0.1, dt=0.1)

    # Get noisy measurement
    z = [true_position[0] + np.random.normal(0, 0.3),
         true_position[1] + np.random.normal(0, 0.3)]

    pf.update(z)
    pf.resample()

    estimate = pf.estimate()

    if step % 10 == 0:
        print(f"Step {step}: Estimate ({estimate[0]:.2f}, {estimate[1]:.2f})")
```

### When to Use Particle Filters

| Scenario | Best Filter |
|----------|-------------|
| Linear system, Gaussian noise | Kalman Filter |
| Nonlinear, Gaussian, small uncertainty | EKF |
| Nonlinear, Gaussian, moderate uncertainty | UKF |
| Non-Gaussian, multimodal, global localization | Particle Filter |

---

## Section 4.5: Sensor Fusion in Practice

### The Robot Localization Stack

Real robots combine multiple sensors, each with different strengths:

| Sensor | What It Measures | Strength | Weakness |
|--------|-----------------|----------|----------|
| Wheel Encoders | Displacement | High rate (1kHz), precise | Drift, slip |
| IMU | Acceleration, Angular Velocity | High rate, no slip | Drift, bias |
| Lidar/Camera | Position relative to landmarks | Absolute position | Lower rate, outliers |
| GPS | Absolute position | Global reference | Low rate, urban canyons |

### ROS 2 Robot Localization Package

The `robot_localization` package provides production-ready EKF and UKF implementations:

```python
# ROS 2 node configuration for sensor fusion
# File: ekf_config.yaml

ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    two_d_mode: true

    # Odometry input
    odom0: /wheel_odom
    odom0_config: [true,  true,  false,  # x, y, z
                   false, false, true,   # roll, pitch, yaw
                   true,  true,  false,  # vx, vy, vz
                   false, false, true,   # vroll, vpitch, vyaw
                   false, false, false]  # ax, ay, az

    # IMU input
    imu0: /imu/data
    imu0_config: [false, false, false,   # x, y, z
                  true,  true,  true,    # roll, pitch, yaw
                  false, false, false,   # vx, vy, vz
                  true,  true,  true,    # vroll, vpitch, vyaw
                  true,  true,  true]    # ax, ay, az

    # Process noise
    process_noise_covariance: [0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               ...]  # 15x15 matrix
```

### Launch File

```python
# File: sensor_fusion_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=['config/ekf_config.yaml']
        ),
    ])
```

---

## Section 4.6: IMU Integration and Bias Estimation

### The IMU Bias Problem

IMUs measure acceleration and angular velocity, but real IMUs have **biases** that drift over time:

```
measured_accel = true_accel + bias + noise
```

A bias of just 0.01 m/s² causes:
- After 1 second: 0.005 m position error
- After 10 seconds: 0.5 m position error
- After 100 seconds: 50 m position error!

### Augmented State for Bias Estimation

We can estimate bias by including it in the state vector:

```python
class IMUBiasEKF:
    """
    EKF with IMU bias estimation.
    State: [x, y, z, vx, vy, vz, bias_ax, bias_ay, bias_az]
    """
    def __init__(self):
        self.state = np.zeros(9)
        self.P = np.diag([1, 1, 1, 0.1, 0.1, 0.1, 0.01, 0.01, 0.01])

    def predict(self, accel_meas, gyro_meas, dt):
        # Extract current estimates
        pos = self.state[0:3]
        vel = self.state[3:6]
        bias = self.state[6:9]

        # Compensate for bias
        accel_corrected = accel_meas - bias

        # Update state
        self.state[0:3] = pos + vel * dt + 0.5 * accel_corrected * dt**2
        self.state[3:6] = vel + accel_corrected * dt
        # Bias evolves slowly (random walk)

        # ... (Jacobian and covariance update)
```

:::tip Real-World Practice
Modern IMUs like the BMI088 have bias stability specifications:
- Accelerometer bias stability: 40 μg
- Gyroscope bias stability: 2°/hr

Always check your IMU's datasheet and configure noise parameters accordingly.
:::

---

## Section 4.7: Visual-Inertial Odometry (VIO)

### Combining Cameras and IMUs

**Visual-Inertial Odometry (VIO)** combines camera images with IMU measurements for robust localization:

- **Camera**: Provides scale and absolute position (from features)
- **IMU**: Provides high-rate motion interpolation between frames

### VIO Pipeline Overview

```
┌─────────────┐     ┌─────────────────┐     ┌──────────────┐
│   Camera    │────▶│ Feature Tracker │────▶│              │
│  (30 Hz)    │     │  (ORB, FAST)    │     │    EKF /     │
└─────────────┘     └─────────────────┘     │   Factor     │
                                            │   Graph      │──▶ Pose
┌─────────────┐     ┌─────────────────┐     │              │
│    IMU      │────▶│ Pre-integration │────▶│              │
│  (200 Hz)   │     │                 │     │              │
└─────────────┘     └─────────────────┘     └──────────────┘
```

### Using VINS-Fusion in ROS 2

```bash
# Install VINS-Fusion
sudo apt install ros-humble-vins-fusion

# Launch with RealSense camera
ros2 launch vins_fusion vins_rviz.launch.py \
    config_file:=config/realsense_config.yaml
```

---

## Section 4.8: Hardware Considerations for Unitree G1

### G1 Sensor Suite

The Unitree G1 humanoid includes:

| Sensor | Specification | Use in State Estimation |
|--------|--------------|------------------------|
| IMU | 9-axis, 400 Hz | Orientation, angular velocity |
| Joint Encoders | 23 joints, 1 kHz | Joint positions, velocities |
| Foot Force Sensors | 4 per foot | Contact detection, CoP estimation |
| RealSense D435 | 640x480 @ 30 Hz | Visual odometry, mapping |
| Lidar (optional) | 360°, 10 Hz | Localization, obstacle detection |

### State Estimation for Humanoids

Humanoid state estimation is more complex than wheeled robots:

```python
class HumanoidStateEstimator:
    """
    State estimator for bipedal robots.

    State includes:
    - Base pose (position + quaternion): 7 values
    - Base velocity (linear + angular): 6 values
    - Joint positions: 23 values
    - Joint velocities: 23 values
    - Contact states: 2 values (left/right foot)
    - IMU biases: 6 values

    Total: 67 state variables
    """
    def __init__(self):
        self.state_dim = 67
        self.state = np.zeros(self.state_dim)
        self.P = np.eye(self.state_dim) * 0.1

    def predict_with_kinematics(self, joint_commands, dt):
        """
        Use forward kinematics to predict foot positions,
        then use foot contacts as pseudo-measurements.
        """
        # If foot is in contact, it shouldn't move
        # This constrains the base pose estimate
        pass

    def update_with_contacts(self, contact_forces):
        """
        Contact-aided invariant EKF update.
        """
        # Feet in contact provide zero-velocity constraints
        pass
```

---

## Section 4.9: Estimation Quality Metrics

### Normalized Estimation Error Squared (NEES)

NEES tests if your filter is **consistent**—if the covariance actually reflects the true uncertainty:

```python
def compute_nees(estimated_state, true_state, covariance):
    """
    NEES should average to state dimension if filter is consistent.
    """
    error = estimated_state - true_state
    nees = error.T @ np.linalg.inv(covariance) @ error
    return nees

# For a 3D state, NEES should average to ~3
# If NEES >> 3: Filter is overconfident (covariance too small)
# If NEES << 3: Filter is underconfident (covariance too large)
```

### Absolute Trajectory Error (ATE)

```python
def compute_ate(estimated_trajectory, ground_truth):
    """
    Root mean squared error over entire trajectory.
    """
    errors = []
    for est, gt in zip(estimated_trajectory, ground_truth):
        error = np.linalg.norm(est[:3] - gt[:3])  # Position error
        errors.append(error)
    return np.sqrt(np.mean(np.array(errors)**2))
```

---

## Section 4.10: Lab Exercise

### Lab 4: Sensor Fusion for Mobile Robot Localization

**Objective**: Implement an EKF that fuses wheel odometry and IMU data.

**Setup**:
```bash
# Clone the lab repository
git clone https://github.com/physical-ai-book/lab-4-state-estimation.git
cd lab-4-state-estimation

# Build with colcon
colcon build
source install/setup.bash

# Launch simulation
ros2 launch state_estimation_lab simulation.launch.py
```

**Tasks**:

1. **Implement EKF Prediction** (15 points)
   - Complete the `predict()` method in `ekf_node.py`
   - Use differential-drive motion model

2. **Implement EKF Update** (15 points)
   - Complete the `update_odom()` and `update_imu()` methods
   - Handle different measurement rates

3. **Tune Noise Parameters** (10 points)
   - Adjust Q and R matrices for best performance
   - Document your tuning process

4. **Evaluate Performance** (10 points)
   - Compute ATE against ground truth
   - Plot estimation error over time

**Expected Output**:
```
Trajectory length: 50.0 m
Wheel odometry only ATE: 2.3 m
IMU only ATE: 15.7 m
Fused EKF ATE: 0.8 m
```

---

## Summary

### Key Takeaways

1. **State estimation** is the foundation of robot autonomy—knowing where you are enables everything else

2. **Kalman Filters** are optimal for linear systems with Gaussian noise; the **Kalman gain** balances prediction vs. measurement trust

3. **Extended Kalman Filters** handle nonlinear systems through linearization, but can fail with high nonlinearity

4. **Particle Filters** handle arbitrary distributions and are essential for global localization

5. **Sensor fusion** combines multiple imperfect sensors to achieve better accuracy than any single sensor

6. **IMU bias estimation** is critical for long-term accuracy; always include biases in your state vector

### Looking Ahead

In Chapter 5, we'll explore **Computer Vision for Robotics**—how to extract information from cameras that feeds into state estimation and enables object detection, scene understanding, and visual navigation.

---

## Exercises

1. **Kalman Filter Derivation**: Starting from the minimum variance criterion, derive the Kalman gain equation.

2. **EKF for 3D Orientation**: Extend the 2D EKF to estimate full 3D orientation using quaternions. Why are quaternions preferred over Euler angles?

3. **Particle Filter Convergence**: Implement a particle filter and measure how estimation accuracy changes with particle count. What's the minimum number of particles needed for 10cm accuracy in a 10m × 10m environment?

4. **Sensor Failure Detection**: Design an algorithm to detect when a sensor fails and automatically exclude it from the fusion. Test with simulated sensor dropouts.

5. **NEES Consistency Test**: Run your EKF 100 times with different noise realizations. Compute average NEES and verify it matches the state dimension.

---

## References

- Thrun, S., Burgard, W., & Fox, D. (2005). *Probabilistic Robotics*. MIT Press.
- Barfoot, T. D. (2017). *State Estimation for Robotics*. Cambridge University Press.
- Moore, T., & Stouch, D. (2016). A Generalized Extended Kalman Filter Implementation for the Robot Operating System. *MEMS and Nanotechnology*, Vol. 5.

---

**Chapter completed**: 2026-01-20
**Chapter**: 4 - State Estimation
