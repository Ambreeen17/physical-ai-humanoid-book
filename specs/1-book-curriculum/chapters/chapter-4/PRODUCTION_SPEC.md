# Chapter 4 Production Specification: State Estimation

**Status**: Ready for agent orchestration
**Target Audience**: Intermediate robotics students with probability and linear algebra background
**Prerequisites**: Chapter 3 (Sensors), basic probability (Gaussian distributions)

---

## Learning Objectives

1. **Implement Kalman filters** for linear Gaussian state estimation (position/velocity tracking)
2. **Apply Extended Kalman Filter (EKF)** to nonlinear systems (pendulum, mobile robot)
3. **Use particle filters** for non-Gaussian, multi-modal distributions
4. **Fuse IMU and odometry data** for robust mobile robot localization
5. **Evaluate estimation accuracy** using RMSE and computational cost metrics

---

## Key Topics

### 4.1: Bayesian Filtering Framework
- **State estimation problem**: Estimate hidden state x from noisy observations z
- **Bayes' rule**: P(x|z) ∝ P(z|x) × P(x)
- **Recursive estimation**: Prediction step + Update step
- **Gaussian assumption**: State and measurement noise ~ N(0, Σ)

### 4.2: Kalman Filter (KF)
- **Linear system model**: x_k = A x_{k-1} + B u_k + w_k
- **Measurement model**: z_k = H x_k + v_k
- **Prediction**: x̂⁻ = A x̂, P⁻ = A P Aᵀ + Q
- **Update**: K = P⁻ Hᵀ (H P⁻ Hᵀ + R)⁻¹, x̂ = x̂⁻ + K(z - H x̂⁻), P = (I - K H) P⁻
- **Example**: 1D position tracking with noisy GPS

### 4.3: Extended Kalman Filter (EKF)
- **Nonlinear system**: x_k = f(x_{k-1}, u_k) + w_k
- **Linearization**: Jacobian F = ∂f/∂x
- **Prediction**: x̂⁻ = f(x̂, u), P⁻ = F P Fᵀ + Q
- **Update**: Same as KF but with linearized H
- **Example**: Inverted pendulum, mobile robot with heading angle

### 4.4: Unscented Kalman Filter (UKF)
- **Sigma points**: Deterministic sampling around mean
- **Unscented transform**: Propagate sigma points through nonlinear function
- **Advantage**: Better accuracy than EKF for highly nonlinear systems
- **Computational cost**: O(n³) vs EKF's O(n²)

### 4.5: Particle Filters (Sequential Monte Carlo)
- **Non-Gaussian distributions**: Multi-modal, arbitrary shapes
- **Particle representation**: {(x_i, w_i)} where Σw_i = 1
- **Prediction**: Propagate particles through motion model
- **Update**: Reweight particles based on likelihood
- **Resampling**: Eliminate low-weight particles
- **Example**: Robot localization with ambiguous landmarks

### 4.6: Sensor Fusion Architectures
- **Centralized**: Single filter with all sensor inputs
- **Decentralized**: Multiple filters, fuse estimates
- **ROS 2 robot_localization**: EKF node for IMU + odometry + GPS

### 4.7: SLAM Basics (Preview)
- **Simultaneous Localization and Mapping**: Estimate robot pose AND map
- **EKF-SLAM**: Augmented state [robot_pose, landmark_1, ..., landmark_n]
- **Particle Filter SLAM (FastSLAM)**: Particle for pose, EKF for landmarks

---

## Hardware Context

**Mobile Robot Platform** (e.g., TurtleBot3, Unitree Go1 quadruped):
- **Wheel odometry**: Encoder-based, subject to slip (±5% error)
- **IMU**: 6-axis, measures linear acceleration + angular velocity
- **LiDAR**: 2D scan for localization (optional in this chapter)

---

## Labs

### Lab 4.1: 1D Kalman Filter (30 min, Beginner)
- Implement KF for position tracking
- Simulate noisy position measurements
- Plot true vs estimated position, covariance over time
- Vary process/measurement noise, observe filter behavior

### Lab 4.2: EKF for Nonlinear Pendulum (45 min, Intermediate)
- Model: θ̈ = -g/L sin(θ) + u
- Measurements: Noisy angle from encoder
- Implement EKF with Jacobian computation
- Compare EKF vs true state

### Lab 4.3: IMU + Odometry Fusion (60 min, Advanced)
- ROS 2 robot_localization package
- Subscribe to `/imu/data` and `/odom` topics
- Configure EKF parameters (process noise, sensor covariances)
- Compare fused estimate vs ground truth (motion capture)

---

## Assessments

1. **MC**: What is the Kalman gain K? (2 pts)
2. **MC**: When should you use EKF instead of KF? (2 pts)
3. **Short Answer**: Explain particle filter resampling. Why is it needed? (3 pts)
4. **Lab Task**: Implement 2D KF for robot tracking (x, y, vx, vy) (10 pts)
5. **Challenge**: Implement FastSLAM for 2D robot with known landmarks (5 bonus pts)

---

**Production Spec Version**: 1.0
**Ready for Agent Orchestration**: ✅
