# Chapter 3 Production Specification: Sensors & Actuators

**Status**: Ready for agent orchestration
**Target Audience**: Intermediate robotics students with signal processing basics
**Prerequisites**: Chapter 1 (Physical AI), Chapter 2 (Kinematics)

---

## Learning Objectives

1. **Characterize sensor performance** using resolution, accuracy, noise, and latency metrics
2. **Process LiDAR point clouds** for obstacle detection and environment mapping
3. **Calibrate RGB-D cameras** using checkerboard patterns and intrinsic/extrinsic parameters
4. **Select actuators** based on torque-speed curves, precision requirements, and power constraints
5. **Implement sensor fusion** using complementary filters and Kalman filters for IMU data

---

## Key Topics

### 3.1: Sensor Taxonomy
- Proprioceptive (internal state) vs Exteroceptive (environment)
- Active vs Passive sensors
- Performance metrics: Resolution, accuracy, precision, range, FOV, latency, power

### 3.2: LiDAR Sensors
- **Principles**: Time-of-Flight (ToF) vs Phase-Shift
- **2D LiDAR**: Sick TIM, Hokuyo URG (range: 30m, resolution: 0.25°)
- **3D LiDAR**: Velodyne, Ouster, Livox (360° FOV, 100m range)
- **Point cloud processing**: PCL library, downsampling, RANSAC plane fitting
- **Unitree G1**: Mid-360 LiDAR (range: 30m, 360° × 59° FOV)

### 3.3: RGB-D Cameras
- **Structured light**: Intel RealSense D435 (range: 0.3-10m, 1280×720)
- **Stereo vision**: ZED 2 (range: 0.5-20m, passive)
- **ToF cameras**: Azure Kinect (range: 0.5-5m, 1024×1024)
- **Calibration**: Intrinsic matrix (fx, fy, cx, cy), distortion coefficients
- **Depth estimation**: Disparity map, triangulation
- **Unitree G1**: RealSense D435i (RGB-D + IMU)

### 3.4: Inertial Measurement Units (IMU)
- **Components**: 3-axis accelerometer, 3-axis gyroscope, 3-axis magnetometer
- **Noise characteristics**: White noise, bias drift, temperature sensitivity
- **Complementary filter**: High-pass gyro + Low-pass accel → Orientation estimate
- **Sensor fusion**: Madgwick filter, Mahony filter
- **Unitree G1**: BMI088 IMU (±16g accel, ±2000°/s gyro, 1kHz rate)

### 3.5: Force-Torque Sensors
- **6-axis F/T sensors**: ATI Nano17 (±12 N force, ±120 Nmm torque)
- **Applications**: Contact detection, impedance control, assembly tasks
- **Calibration**: Zero-offset, gravity compensation

### 3.6: Actuators
- **DC motors**: Brushed vs brushless, back-EMF, torque constant
- **Servo motors**: Position control, PWM interface, torque limits
- **Stepper motors**: Open-loop positioning, holding torque
- **Selection criteria**: Torque-speed curve, efficiency, weight, cost
- **Unitree G1**: Direct-drive motors (peak torque: 70 Nm per joint)

---

## Hardware Context

**Unitree G1 Sensor Suite**:
- **LiDAR**: Mid-360 (360° horizontal, 59° vertical FOV)
- **RGB-D Camera**: RealSense D435i (1280×720, 90° FOV, 0.3-10m range)
- **IMU**: BMI088 (6-axis, 1kHz sampling rate)
- **Joint Encoders**: Magnetic encoders (0.01° resolution per joint)
- **Force Sensors**: Foot pressure sensors (4 per foot, 0-50 kg range)

---

## Labs

### Lab 3.1: LiDAR Point Cloud Processing (45 min)
- Parse LiDAR data from ROS 2 topic
- Downsample using voxel grid filter
- Detect obstacles via RANSAC plane fitting
- Visualize in RViz

### Lab 3.2: RGB-D Camera Calibration (30 min)
- Capture checkerboard images
- Compute intrinsic matrix using OpenCV
- Measure depth accuracy vs ground truth
- Plot error vs distance

### Lab 3.3: IMU Complementary Filter (45 min)
- Read raw IMU data (accel + gyro)
- Implement complementary filter for roll/pitch
- Compare with ground truth (robot_localization)
- Tune filter alpha parameter

---

## Assessments

1. **MC**: Which sensor is exteroceptive? (LiDAR, IMU, Encoder) (2 pts)
2. **MC**: What does calibration correct in RGB-D cameras? (2 pts)
3. **Short Answer**: Why combine accelerometer and gyroscope for orientation? (3 pts)
4. **Lab Task**: Implement obstacle avoidance using LiDAR (10 pts)
5. **Challenge**: Fuse IMU + wheel odometry for mobile robot localization (5 bonus pts)

---

**Production Spec Version**: 1.0
**Ready for Agent Orchestration**: ✅
