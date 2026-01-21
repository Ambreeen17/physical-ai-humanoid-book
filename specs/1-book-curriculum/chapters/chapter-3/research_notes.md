# Chapter 3 Research Notes: Sensors & Actuators

## Overview

Sensors and actuators form the fundamental interface between a robot and its environment. Sensors provide the robot with perception of both its internal state (proprioception) and external surroundings (exteroception), while actuators translate computational commands into physical motion. This chapter establishes the foundational knowledge required to understand, select, and integrate these components in physical AI systems, with particular emphasis on the sensor suite found in the Unitree G1 humanoid robot.

The integration of multiple sensor modalities and the fusion of their data streams represents a critical capability for modern robotics. No single sensor provides complete, accurate information about the robot's state or environment; thus, understanding sensor characteristics, limitations, and complementary relationships is essential for building robust perception systems.

---

## 3.1 Sensor Taxonomy

### Core Concepts

#### Proprioceptive vs Exteroceptive Sensors

**Proprioceptive sensors** measure the internal state of the robot—its joint positions, velocities, accelerations, and forces. These sensors provide feedback about the robot's own body without requiring information from the external environment.

- **Joint encoders**: Measure angular position of joints with high precision (often 0.01 or better resolution). Found in all motorized joints of the Unitree G1.
- **Current sensors**: Measure motor current draw, enabling torque estimation through the torque constant (Kt). Used for force estimation and motor protection.
- **IMUs**: Measure the robot's acceleration and angular velocity, providing state estimates for balance and locomotion.
- **Potentiometers**: Analog position sensors used in lower-cost applications.

**Exteroceptive sensors** capture information about the environment external to the robot. They enable the robot to perceive obstacles, objects, terrain, and other environmental features.

- **LiDAR**: Laser-based ranging sensors that create detailed 3D maps of the environment.
- **RGB-D cameras**: Provide both color imagery and depth maps simultaneously.
- **Stereo cameras**: Use dual cameras for depth estimation through triangulation.
- **Ultrasonic sensors**: Use sound waves for distance measurement, common in underwater or outdoor applications.
- **Touch/force sensors**: Detect physical contact with objects or surfaces.

#### Active vs Passive Sensors

**Passive sensors** detect energy naturally present in the environment without emitting their own signals. They are generally simpler and less prone to interference between multiple sensors.

- **Passive stereo cameras**: Rely on ambient light; struggle in low-light conditions.
- **Photodiodes/light sensors**: Measure existing light levels.
- **Magnetometers**: Detect Earth's magnetic field for heading estimation.

**Active sensors** emit energy (light, sound, radio waves) and measure the response. They can operate independently of ambient conditions but may interfere with each other when multiple robots operate nearby.

- **LiDAR**: Emits laser pulses and measures return time.
- **Structured light projectors**: Cast known patterns and analyze deformation.
- **Ultrasonic sensors**: Emit sound pulses and measure echo return time.
- **ToF cameras**: Modulated light sources enable precise distance measurement.

#### Key Performance Metrics

| Metric | Definition | Unit | Typical Values (Sensors) |
|--------|------------|------|--------------------------|
| **Resolution** | Smallest detectable change | Depends on sensor | Encoder: 0.01 deg; IMU: 0.001 deg/s |
| **Accuracy** | Deviation from true value | mm, deg, % | LiDAR: 2-5mm; Encoder: 0.01% |
| **Precision/Repeatability** | Variation in repeated measurements | Same as accuracy | Often better than accuracy |
| **Range** | Minimum to maximum detectable distance | m | D435i: 0.3-10m; LiDAR: 0.1-200m |
| **Field of View (FOV)** | Angular coverage | degrees | Mid-360: 360x59 deg; D435i: 87x58 deg |
| **Latency** | Time from measurement to data availability | ms | IMU: 1-5ms; LiDAR: 10-50ms |
| **Sampling Rate** | Measurements per second | Hz | IMU: 100-1000Hz; LiDAR: 10-30Hz |
| **Power Consumption** | Energy usage during operation | W | IMU: 0.05W; LiDAR: 5-30W |

### Industry-Standard Tools

- **ROS 2 sensor interfaces**: Standardized message types (sensor_msgs/Image, sensor_msgs/PointCloud2, sensor_msgs/Imu) for sensor data interoperability.
- **ros2_control**: Framework for integrating proprioceptive sensors (encoders, force-torque) with robot hardware interfaces.
- **sensor_calibration**: ROS 2 packages for intrinsic and extrinsic calibration of cameras, LiDAR, and IMUs.

### Practical Constraints

1. **Data bandwidth**: High-resolution sensors (RGB-D at 30fps, LiDAR at 20Hz) generate significant data volumes requiring careful topic QoS configuration.
2. **Synchronization**: Multi-sensor systems require hardware or software timestamping to align data streams (ROS 2 Time synchronizer or external PTP synchronization).
3. **Coordinate frames**: Each sensor has its own frame; transforms must be accurately known and maintained via TF2 for proper data fusion.
4. **Environmental interference**: Active sensors (multiple LiDARs) can interfere with each other in shared spaces.

### Common Beginner Mistakes

- **Ignoring frame transformations**: Using sensor data without transforming to a common frame leads to incorrect perception. Always use TF2 to transform point clouds to robot frame.
- **Assuming perfect accuracy**: Beginners often trust sensor readings without considering noise, bias, or environmental factors that degrade accuracy.
- **Mismatched sampling rates**: Subscribing to high-rate sensors with low-rate processing nodes causes dropped frames or lag. Match processing rates to data rates.

---

## 3.2 LiDAR Sensors

### Core Concepts

#### Time-of-Flight (ToF) Principles

LiDAR (Light Detection and Ranging) sensors measure distance by emitting laser pulses and calculating the time required for the light to travel to a target and return. The fundamental distance equation is:

```
Distance = (c * Δt) / 2
```

where c is the speed of light (3 x 10^8 m/s) and Δt is the round-trip time of the pulse.

**Pulsed ToF LiDAR** emits short, high-power laser pulses (typically 5-10 ns duration) and measures the return time directly using high-speed timing circuits. This approach enables long-range sensing (up to 200m+ for automotive applications) but requires expensive high-speed electronics.

#### Phase-Shift LiDAR

**Phase-shift (or continuous wave) LiDAR** modulates the laser intensity at a high frequency and measures the phase shift between the emitted and returned signals. The distance is calculated from the phase difference:

```
Distance = (φ * c) / (4π * f * 2)
```

where φ is the phase shift and f is the modulation frequency. Phase-shift LiDAR typically uses lower power but enables higher precision at shorter ranges. The Livox Mid-360 uses a custom implementation with non-repeating scan patterns.

#### Point Cloud Processing with PCL

The **Point Cloud Library (PCL)** provides comprehensive algorithms for processing 3D point cloud data. Key operations include:

**Filtering Algorithms:**
- **Voxel Grid Filter**: Downsamples point clouds by grouping points into voxels (3D pixels) and keeping only centroid or representative points. Critical for reducing computation while maintaining geometric structure.
- **Statistical Outlier Removal**: Removes points that are significantly farther from their neighbors than the average, eliminating noise from specular reflections or dust.
- **Pass-through Filter**: Removes points outside specified x, y, or z bounds for region of interest selection.

**Segmentation and Fitting:**
- **RANSAC Plane Fitting**: Random Sample Consensus identifies planar surfaces (floors, walls) by iteratively fitting planes to random point subsets and counting inliers. Essential for ground plane removal in navigation.
- **Euclidean Cluster Extraction**: Groups nearby points into clusters for obstacle detection and object segmentation.
- **SACSegmentation**: General framework for fitting geometric primitives (planes, spheres, cylinders).

**Feature Estimation:**
- **Normal Estimation**: Computes surface normals at each point using k-nearest neighbor or radius search.
- **FPFH (Fast Point Feature Histograms)**: Describes local geometric properties for object recognition and registration.

#### LiDAR Specifications Comparison

| Model | Type | Range | FOV | Points/sec | Typical Use |
|-------|------|-------|-----|------------|-------------|
| **Hokuyo URG-04LX** | 2D ToF | 5.6m | 270 deg | 2400 | Indoor navigation |
| **Sick TIM571** | 2D ToF | 25m | 220 deg | 5000 | Industrial AGV |
| **Velodyne Puck (VLP-16)** | 3D ToF | 100m | 360x30 deg | 300,000 | Autonomous vehicles |
| **Ouster OS1-64** | 3D ToF | 120m | 360x45 deg | 1,280,000 | High-res mapping |
| **Livox Mid-360** | 3D Phase | 30m | 360x59 deg | 200,000 | Mobile robotics |

#### Unitree G1: Mid-360 LiDAR

The Unitree G1 humanoid robot integrates the **Livox Mid-360** 3D LiDAR with the following characteristics:

- **Field of View**: 360 horizontal x 59 vertical degrees (near-omnidirectional)
- **Range**: 0.1m to 30m (optimal for humanoid operation in indoor/urban environments)
- **Point Rate**: Up to 200,000 points/second
- **Non-repeating scan pattern**: Unlike traditional spinning LiDARs, the Mid-360 uses a unique pattern that accumulates detail over time, improving obstacle detection during stationary periods.
- **ROS 2 Support**: Via livox_ros2_driver package, publishing sensor_msgs/PointCloud2 at configurable rates (10-20Hz)

### Industry-Standard Tools

- **PCL (Point Cloud Library)**: Open-source, BSD-licensed library with filtering, segmentation, registration, and recognition algorithms. Supports Linux, macOS, Windows.
- **Open3D**: Modern Python/C++ library for 3D data processing with superior Python integration and visualization.
- **pcl_ros**: ROS package providing bridge between PCL and ROS 2 topic subscriptions.
- **livox_ros2_driver**: Official Livox driver for ROS 2, supporting Mid-360 and other Livox sensors.
- **velodyne_ros2**, **ouster_ros**: Manufacturer-provided ROS 2 drivers for respective LiDAR families.
- **RViz2**: Visualization tool for point clouds with color mapping based on distance, intensity, or rings.

### Practical Constraints

1. **Motion distortion**: At 20Hz update rate, a robot moving at 1 m/s accumulates 5cm of motion between scan start and end. Requires motion compensation for accurate mapping.
2. **Specular reflections**: Smooth, reflective surfaces (glass, polished metal) return weak or erroneous signals. Multi-path returns can cause false positives.
3. **Ambient light interference**: Direct sunlight can overwhelm detectors; automotive LiDARs use near-infrared (905nm) with optical filtering.
4. **Point density variation**: Far from sensor, points are sparse; close to sensor, points cluster. Affects obstacle detection reliability.
5. **Computation requirements**: Processing full-rate point clouds requires GPU acceleration for real-time applications.

### Common Beginner Mistakes

- **Processing raw point clouds without filtering**: Attempting obstacle detection on raw 10Hz point clouds leads to missed obstacles and high latency. Always downsample first.
- **Ignoring motion distortion**: Using uncorrected point clouds from a moving robot causes systematic errors in mapping. Implement motion compensation or use higher-rate sensors.
- **Using Euclidean distance without normals**: Simple distance thresholds fail on sloped surfaces. Use normal-based segmentation for accurate ground detection.
- **Not accounting for LiDAR mounting orientation**: The Mid-360 has a 59-degree vertical FOV; obstacles directly above or below the sensor are not detected. Account for blind spots in navigation planning.

---

## 3.3 RGB-D Cameras

### Core Concepts

#### Depth Sensing Technologies

**Structured Light** cameras project a known infrared pattern onto the scene and analyze the deformation to compute depth. The Intel RealSense D435 uses active infrared projectors to enhance pattern visibility.

- **Advantages**: Low cost, works in ambient darkness, good accuracy at close range
- **Disadvantages**: Degraded performance in bright sunlight (IR washout), struggles with transparent or highly reflective surfaces
- **Range**: Typically 0.3-10m for consumer devices

**Stereo Vision** uses two cameras separated by a known baseline to compute depth through triangulation. Passive stereo relies on natural texture; active stereo adds an IR projector to provide texture on untextured surfaces.

- **Advantages**: Works in any lighting (passive), no interference between units, long-range capability
- **Disadvantages**: Computationally intensive (requires dense correspondence matching), degraded on textureless surfaces
- **Range**: 0.5-20m for high-quality stereo (e.g., ZED 2)

**Time-of-Flight (ToF)** cameras modulate an infrared light source and measure the phase shift between emitted and received signals. Direct ToF sensors measure return time directly; indirect ToF measures phase.

- **Advantages**: Single-frame depth capture (no scanning), compact form factor
- **Disadvantages**: Multi-path interference, lower resolution than stereo, temperature-dependent accuracy
- **Range**: 0.5-5m for typical sensors; up to 10m with high-power illumination

#### Camera Calibration: Intrinsic Parameters

Camera calibration determines the mapping from 3D world points to 2D image coordinates. The **pinhole camera model** describes this relationship:

```
[u, v, 1]^T = K * [X/Z, Y/Z, 1]^T
```

where K is the 3x3 intrinsic matrix:

```
[f_x  0   c_x]
[ 0  f_y  c_y]
[ 0   0    1 ]
```

**Intrinsic parameters** include:
- **f_x, f_y**: Focal lengths in pixels (depends on sensor resolution and lens)
- **c_x, c_y**: Principal point coordinates (typically near image center)
- **Distortion coefficients**: k1, k2, k3 (radial), p1, p2 (tangential) for lens distortion correction

**Calibration procedure**:
1. Capture 10-20 images of a checkerboard pattern at various angles and distances
2. Detect corner points in each image using cv::findChessboardCorners
3. Optimize intrinsic parameters and distortion coefficients via cv::calibrateCamera
4. Verify with reprojection error (< 1 pixel is typical)

#### Depth Camera Calibration

For RGB-D cameras, depth calibration involves:
- **Depth scale**: Conversion from disparity/depth values to metric units (mm)
- **Depth correction**: Polynomial or lookup table mapping raw depth to actual distance
- **Color-depth alignment**: Extrinsic transform between RGB and depth sensors

The RealSense D435i provides factory-calibrated intrinsics accessible via:
```cpp
rs2::pipeline pipe;
rs2::config cfg;
cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
auto profile = pipe.start(cfg);
auto intrinsics = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
```

#### Unitree G1: RealSense D435i

The Unitree G1 integrates an **Intel RealSense D435i** RGB-D camera with:

- **RGB Resolution**: 1920x1080 at 30fps or 1280x720 at 90fps
- **Depth Resolution**: 1280x720 at 30fps or 848x480 at 90fps
- **Field of View**: 87 horizontal x 58 vertical degrees
- **Depth Range**: 0.3m to 10m
- **Baseline**: 50mm (distance between IR projector and right IR camera)
- **IMU**: Integrated BMI055 6-axis IMU (accel + gyro) for motion tracking
- **Minimum Depth**: ~28cm (shorter than most competing sensors)

### Industry-Standard Tools

- **OpenCV**: Computer vision library with camera calibration (cv::calibrateCamera, cv::undistort), depth processing, and RGB-D utilities.
- **Intel RealSense SDK 2.0**: Provides librealsense library for camera access, calibration, and depth processing. Supports ROS 2 via realsense_ros2_camera package.
- **ROS 2 camera_calibration**: Package for calibrating cameras using checkerboard patterns with automated corner detection.
- **Kalibr**: Advanced multi-sensor calibration tool for calibrating camera-IMU extrinsics with spatial and temporal optimization.
- **AprilCal**: Easy-to-use tool for high-accuracy camera calibration using AprilTags.

### Practical Constraints

1. **Depth accuracy vs distance**: Depth error typically scales with square of distance. At 2m, expect ~1-2cm accuracy; at 5m, expect ~5-10cm.
2. **Ambient light sensitivity**: Structured light and ToF cameras degrade in direct sunlight. Outdoor depth sensing requires specialized sensors (e.g., RGB-IR cameras with solar-filtered ToF).
3. **Texture requirements**: Passive stereo requires texture; structured light and active stereo solve this with IR projectors but consume more power.
4. **Resolution trade-offs**: High-resolution depth requires more computation; real-time applications often use depth resolutions of 640x480 or lower.
5. **Synchronization with RGB**: Color and depth streams have different exposure times; motion during capture creates color-depth misalignment.

### Common Beginner Mistakes

- **Skipping factory calibration**: Assuming all cameras have identical intrinsics leads to systematic errors. Always read or perform calibration for each unit.
- **Ignoring depth noise**: Treating depth pixels as independent measurements causes poor filtering. Depth noise is spatially correlated; use bilateral filtering.
- **Mismatched color-depth timestamps**: Processing color and depth as if simultaneous creates artifacts. Use hardware-synchronized streams or apply motion compensation.
- **Using depth without confidence masks**: RealSense provides confidence/disparity error estimates. Ignoring these leads to false obstacles at depth discontinuities and low-texture regions.

---

## 3.4 Inertial Measurement Units (IMUs)

### Core Concepts

#### IMU Components and Measurements

An IMU combines three sensing elements to measure motion:

**Accelerometer**:
- Measures proper acceleration (including gravity) along 3 axes
- Output: [a_x, a_y, a_z] in m/s^2 or g
- Responds to both translation and tilt (gravity component)
- High-bandwidth (100-1000Hz) but noisy, especially for orientation

**Gyroscope (Gyro)**:
- Measures angular velocity about 3 axes
- Output: [omega_x, omega_y, omega_z] in rad/s or deg/s
- Integrates to obtain orientation
- Low noise but subject to bias drift over time

**Magnetometer** (optional, not in BMI088):
- Measures magnetic field strength along 3 axes
- Provides absolute heading reference (yaw)
- Affected by ferromagnetic materials and EM interference

#### Noise Characteristics

Understanding IMU noise is critical for sensor fusion design:

| Noise Type | Description | Time Behavior | Impact on Integration |
|------------|-------------|---------------|----------------------|
| **White noise** | Random, uncorrelated | Instantaneous | Accumulates as sqrt(t) in position |
| **Bias (offset)** | Constant offset | Slow drift (10-100 deg/hr) | Accumulates as t^2 in position |
| **Bias instability** | Random walk of bias | 1/f (flicker noise) | Slow position drift |
| **Temperature sensitivity** | Bias change with temperature | Correlated to temperature | Requires temperature compensation |

**BMI088 Specific Noise Parameters**:
- Accelerometer noise density: 175 μg/√Hz (typical)
- Gyroscope noise density: 0.014 deg/s/√Hz (typical)
- Accelerometer bias stability: +/-20 mg (lifetime)
- Gyroscope bias stability: +/-1 deg/s (lifetime)
- Temperature coefficient: 0.2 mg/K (accel), 0.015 deg/s/K (gyro)

#### Complementary Filter

The complementary filter exploits the frequency-domain complementarity of accelerometer and gyroscope measurements:

```
θ_filtered = α * (θ_previous + ω * Δt) + (1 - α) * θ_accel
```

where:
- θ_accel = atan2(a_y, a_z) for roll (or atan2(-a_x, sqrt(a_y^2 + a_z^2)) for pitch)
- α = α_cut / (α_cut + Δt) is the filter coefficient
- α_cut = 2π * f_cut (cutoff frequency)

**Intuition**: The gyroscope provides accurate high-frequency orientation changes, while the accelerometer provides accurate low-frequency tilt (due to gravity). The filter combines them.

**Implementation example** (C++):
```cpp
class ComplementaryFilter {
    float alpha_;
    float angle_ = 0;
public:
    ComplementaryFilter(float alpha) : alpha_(alpha) {}
    void update(float gyro_rate, float accel_angle) {
        angle_ = alpha_ * (angle_ + gyro_rate * 0.01f) +
                 (1 - alpha_) * accel_angle;
    }
    float getAngle() const { return angle_; }
};
```

#### Kalman Filter for IMU Fusion

The Kalman filter provides optimal state estimation for linear Gaussian systems. For IMU orientation, an **Extended Kalman Filter (EKF)** handles the nonlinear attitude representation.

**State vector**: [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]^T

**Process model**:
```
roll_{k+1} = roll_k + (omega_x - bias_x) * dt
pitch_{k+1} = pitch_k + (omega_y - bias_y) * dt
yaw_{k+1} = yaw_k + (omega_z - bias_z) * dt
bias_{k+1} = bias_k  (random walk)
```

**Measurement model**: Accelerometer provides direct (noisy) roll and pitch measurements:
```
z_roll = roll + v_roll
z_pitch = pitch + v_pitch
```

**Implementation in robot_localization** (ROS 2):
```yaml
ekf_node:
  ros__parameters:
    frequency: 50
    two_d_mode: false
    world_frame: odom
    odom_frame: odom
    base_link_frame: base_link
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  false, false, false]
    imu0_remove_gravity: true
```

#### Advanced Filters

**Madgwick Filter**:
- Computationally efficient orientation filter using gradient descent optimization
- Publicly available as open-source implementation
- Provides quaternion output (no gimbal lock)
- Tuning parameter (beta) trades noise rejection against response speed

**Mahony Filter**:
- PI-based feedback controller for error correction
- Similar performance to Madgwick with different tuning parameters
- Often preferred for real-time embedded systems due to simpler math

#### Unitree G1: BMI088 IMU

The Unitree G1 uses the **Bosch BMI088** high-performance IMU:

**Accelerometer Specifications**:
- Measurement ranges: ±3g, ±6g, ±12g, ±24g (user selectable)
- Resolution: 16-bit (0.09mg per LSB at ±3g range)
- Noise density: 175 μg/√Hz
- Output rate: Up to 1600Hz (internal), 1000Hz (typical for ROS 2)

**Gyroscope Specifications**:
- Measurement ranges: ±125, ±250, ±500, ±1000, ±2000 deg/s (user selectable)
- Resolution: 16-bit (0.004 deg/s per LSB at ±125 deg/s range)
- Noise density: 0.014 deg/s/√Hz
- Output rate: Up to 2000Hz (internal)

**Integration in ROS 2**:
```yaml
# Unitree G1 sensor configuration
sensors:
  imu:
    type: bmi088
    frame_id: imu_link
    rate: 1000  # Hz
    accel_range: 16  # g
    gyro_range: 2000  # deg/s
    topic: /imu/data
```

### Industry-Standard Tools

- **ROS 2 imu_tools**: Provides imu_complementary_filter and imu_filter_madgwick nodes for sensor fusion.
- **robot_localization**: Comprehensive EKF implementation for fusing multiple sensors (IMU, wheel odometry, GPS).
- **Xsens MTw Awinda**: Professional-grade IMU for ground truth and calibration reference.
- **kalibr**: Multi-sensor calibration tool for determining IMU intrinsics and camera-IMU extrinsics.
- **imu_utils**: ROS tool for IMU noise analysis and Allan variance computation.

### Practical Constraints

1. **Sampling rate requirements**: Accelerometer and gyroscope should be sampled at high rates (500-1000Hz) and averaged for lower-rate fusion (50-100Hz). Aliasing occurs if sampling below sensor bandwidth.
2. **Bias instability**: Gyro bias drift causes orientation errors that grow quadratically. Regular zero-velocity updates or external corrections are needed for long-duration operation.
3. **Temperature effects**: IMU bias changes with temperature. Active temperature compensation or sensor warm-up before operation improves accuracy.
4. **Magnetic interference**: In humanoid robots, motors and electronics create strong magnetic fields. Magnetometer data (when available) must be heavily filtered or ignored.
5. **Vibration sensitivity**: High-frequency vibrations from motors cause accelerometer noise. Mechanical isolation and software filtering are essential.

### Common Beginner Mistakes

- **Ignoring gyro bias**: Using raw gyro integration without bias correction causes rapid heading drift. Always estimate and subtract bias.
- **Using the same filter parameters for all applications**: A filter tuned for a ground robot fails on a flying drone. Adjust parameters based on expected motion dynamics.
- **Not accounting for gravity in accelerometer readings**: The accelerometer measures both motion and gravity. Simple integration without gravity compensation is incorrect.
- **Fusing accelerometer data during acceleration**: When the robot accelerates (not just tilts), accelerometer-based pitch/roll estimates are wrong. Use high-pass filtering or motion detection to exclude acceleration phases.

---

## 3.5 Force-Torque Sensors

### Core Concepts

#### 6-Axis Force-Torque Sensors

Force-torque (F/T) sensors measure three forces (F_x, F_y, F_z) and three torques (T_x, T_y, T_z) applied at the sensor location. They are typically strain-gauge based, with deformation measured in multiple directions.

**Common configurations**:
- **Wrist F/T sensors**: Mounted at the robot wrist, between end-effector and tool
- **Foot F/T sensors**: Embedded in robot feet for ground contact detection
- **Joint F/T sensors**: Integrated in joints for interaction estimation without extra hardware

**Specifications** (ATI Nano17):
- Force range: ±17 N (Fx, Fy), ±34 N (Fz)
- Torque range: ±0.5 Nm (Tx, Ty), ±0.7 Nm (Tz)
- Resolution: 1/80 N, 1/400 Nm
- Sampling rate: Up to 5000Hz

#### Calibration and Compensation

**Zero offset**: F/T sensors require initial zeroing with no external load. Gravity compensation must account for the weight of attached hardware.

**Gravity compensation**:
```
F_z_compensated = F_z - m * g * cos(θ)
T_x_compensated = T_x + m * g * l * cos(θ)
```
where m is the tool mass, l is the center of mass offset, and θ is the tilt angle.

### Practical Constraints

1. **Overload protection**: F/T sensors have limited ranges. Impact loads can damage sensors or cause saturation.
2. **Temperature drift**: Strain gauges are temperature-sensitive. Some applications require active temperature compensation.
3. **Cross-talk**: Forces can cause apparent torques due to imperfect calibration. High-quality sensors minimize this.

---

## 3.6 Actuators

### Core Concepts

#### DC Motors

**Brushed DC motors** use mechanical brushes to commutate the armature winding:
- Simple control: voltage proportional to speed
- Lower cost, but brushes wear out
- Typical efficiency: 70-85%

**Brushless DC (BLDC) motors** use electronic commutation:
- Higher efficiency (85-95%), longer lifespan
- More complex control (requires ESC or driver)
- Higher power density

**Key motor parameters**:
- **Torque constant (Kt)**: Torque per ampere (Nm/A)
- **Back-EMF constant (Ke)**: Voltage per rad/s (V/(rad/s))
- **Resistance (R)**: Armature resistance
- **Inductance (L)**: Armature inductance (affects current response time)

**Motor equations**:
```
V = I * R + L * dI/dt + Ke * ω      (Electrical)
T = Kt * I                           (Mechanical)
```

#### Servo Motors

Servo motors integrate a DC/BLDC motor, gearbox, position encoder, and controller:

- **Position control**: Typically 180-270 degree range (limited rotation)
- **Continuous rotation servos**: Position command becomes speed command
- **Digital vs analog**: Digital servos have faster response and higher resolution
- **PWM interface**: 1-2ms pulse width controls position (1.5ms = center)

**Limitations**:
- Limited rotation angle
- Backlash in gear train
- Lower power compared to direct-drive motors

#### Stepper Motors

Stepper motors move in discrete steps without feedback:
- **Open-loop control**: Position known from step count
- **Holding torque**: Maintains position without power (when energized)
- **Microstepping**: Finer resolution (1/16, 1/32 steps) but reduced holding torque

**Disadvantages**:
- Can lose steps under load (silent failure)
- Lower efficiency (current flows even at standstill)
- Limited top speed due to torque fall-off

#### Torque-Speed Curves

The torque-speed relationship is fundamental to actuator selection:

```
T = T_stall * (1 - ω / ω_no_load)
```

where:
- T_stall: Maximum torque at zero speed
- ω_no_load: Maximum speed with zero torque
- Efficiency peaks near 50% of stall torque

**Key parameters**:
- **Power density**: Power per unit mass (W/kg)
- **Torque density**: Torque per unit mass (Nm/kg)
- **Speed range**: Ratio of max to min usable speed
- **Efficiency map**: Efficiency as function of speed and torque

#### Unitree G1: Direct-Drive Actuators

The Unitree G1 uses **proprietary direct-drive actuators** (no gear reduction):
- **Peak torque**: Up to 70 Nm per joint
- **Position control**: Magnetic encoders with 0.01 degree resolution
- **Bandwidth**: High (>50 Hz) due to direct coupling
- **Efficiency**: >90% (no gear losses)

This design eliminates backlash and reduces maintenance but requires high-torque, low-speed motors.

### Industry-Standard Tools

- **ROS 2 control**: Framework for commanding robot hardware, including joint trajectory controllers and effort controllers.
- **gazebo_ros2_control**: Simulates hardware interfaces and joint controllers in Gazebo.
- **Arduino/Pico servo libraries**: For low-level servo PWM control.
- **ODrive**: High-performance BLDC motor controller for robotics (SPI/CAN interface).
- **Robotis Dynamixel**: Smart actuators with integrated controllers, PID tuning, and ROS 2 support.

### Practical Constraints

1. **Thermal limits**: Motors have continuous and peak torque ratings. Sustained operation at peak causes overheating.
2. **Back-EMF limits**: At high speeds, back-EMF can exceed supply voltage. Regen braking or active braking may be required.
3. **Inertia matching**: The reflected inertia of the load should be similar to the motor inertia for good control performance.
4. **Power supply**: High-torque actuators require high current (10-50A). Power supply and cabling must be sized accordingly.
5. **Control bandwidth**: The joint controller bandwidth should be 5-10x higher than the desired system bandwidth to avoid resonance.

### Common Beginner Mistakes

- **Selecting actuators based on peak torque only**: Ignoring continuous ratings leads to overheating during sustained operation. Always check thermal limits.
- **Neglecting reflected inertia**: A fast-moving heavy link looks like a large inertia to the motor. Calculate reflected inertia (load inertia / gear ratio^2).
- **Ignoring gear backlash**: When reversing direction, the first several degrees of motion close the gear gap without moving the output. Use preloading or direct-drive.
- **Underestimating power requirements**: Peak current during acceleration can be 3-5x steady-state. Power supply and batteries must handle transients.

---

## Hardware Context Summary: Unitree G1

| Component | Model | Key Specifications | ROS 2 Topic | Rate |
|-----------|-------|-------------------|-------------|------|
| LiDAR | Livox Mid-360 | 30m range, 360x59 FOV, 200k pts/s | /points_raw | 10-20Hz |
| RGB-D | Intel RealSense D435i | 0.3-10m, 1280x720, 90 FOV | /camera/camera/... | 30Hz |
| IMU | Bosch BMI088 | +/-16g, +/-2000 deg/s, 1kHz | /imu/data | 1000Hz |
| Joint Encoders | Magnetic | 0.01 deg resolution | /joint_states | 500Hz |
| Foot Sensors | Pressure | 0-50 kg per foot | /foot_pressure | 100Hz |

---

## Key Terms Glossary

- **Accelerometer**: Sensor measuring proper acceleration including gravity, used for tilt estimation.
- **Back-EMF**: Voltage generated by motor rotation, proportional to speed.
- **Complementary filter**: Combines high-frequency gyro and low-frequency accelerometer data for orientation estimation.
- **Depth map**: 2D array where each pixel value represents distance to the corresponding 3D point.
- **Encoder**: Position sensor that converts angular motion into digital pulses.
- **Exteroceptive**: Sensor type that measures the external environment.
- **Field of View (FOV)**: Angular range a sensor can detect.
- **Frame transformation**: Mathematical operation converting coordinates between sensor and robot reference frames.
- **Gyroscope**: Sensor measuring angular velocity.
- **IMU (Inertial Measurement Unit)**: Combined accelerometer and gyroscope for motion sensing.
- **Intrinsic parameters**: Camera calibration values (focal length, principal point, distortion) mapping 3D to 2D.
- **Kalman filter**: Optimal state estimator for linear systems with Gaussian noise.
- **LiDAR**: Light Detection and Ranging sensor using laser pulses for distance measurement.
- **Motion distortion**: Point cloud deformation caused by robot motion during scan acquisition.
- **Noise density**: Power spectral density of sensor noise, expressed in units per sqrt(Hz).
- **PCL**: Point Cloud Library for 3D point cloud processing.
- **Proprioceptive**: Sensor type measuring internal robot state.
- **RANSAC**: Random Sample Consensus algorithm for robust model fitting to noisy data.
- **Resolution**: Smallest detectable change in measurement.
- **Stereo triangulation**: Depth estimation from correspondences between two cameras.
- **Structured light**: Depth sensing using projected pattern deformation analysis.
- **Time-of-Flight (ToF)**: Distance measurement from pulse or phase shift of light.
- **Torque constant (Kt)**: Relationship between motor current and output torque.
- **Voxel grid**: 3D analog of pixels, used for downsampling point clouds.

---

## References

### Sensor Taxonomy
- [ROS 2 Sensor Concepts](https://docs.ros.org/en/humble/Concepts/About-Sensors.html)

### LiDAR Technology
- [Point Cloud Library Documentation](https://pointclouds.org/documentation/)
- [Livox Mid-360 Product Page](https://www.livox.com/mid-360/)
- [PCL ROS Integration (pcl_ros)](https://github.com/ros-perception/pcl_ros)

### RGB-D Cameras
- [Intel RealSense D435i Documentation](https://www.intelrealsense.com/d435i/)
- [RealSense ROS 2 Wrapper](https://github.com/IntelRealSense/realsense-ros)
- [OpenCV Camera Calibration](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

### IMU and Sensor Fusion
- [Bosch BMI088 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi088/)
- [ROS 2 imu_tools Package](https://github.com/ros-perception/imu_tools)
- [robot_localization EKF Package](https://github.com/cra-ros-pkg/robot_localization)
- [Kalibr Multi-Sensor Calibration](https://github.com/ethz-asl/kalibr)

### Actuators
- [ROS 2 Control Documentation](https://control.ros.org/)
- [ODrive Motor Controller](https://odriverobotics.com/)

### Unitree G1
- [Unitree Robotics Official](https://en.unitree.com/)
- [Livox ROS 2 Driver](https://github.com/Livox-SDK/livox_ros2_driver)

---

## Lab Implementation Notes

### Lab 3.1: LiDAR Point Cloud Processing

**Prerequisites**:
```bash
# Install dependencies
sudo apt install ros-humble-pcl-ros ros-humble-pointcloud-to-laserscan
# Clone Livox driver
git clone https://github.com/Livox-SDK/livox_ros2_driver.git
```

**Key Code Concepts**:
1. Subscribe to sensor_msgs/PointCloud2
2. Apply voxel grid filter (leaf_size ~0.05m)
3. Use RANSAC for ground plane removal (distance_threshold ~0.03m)
4. Euclidean clustering for obstacle segmentation
5. Publish to /obstacles topic for navigation

### Lab 3.2: RGB-D Camera Calibration

**Procedure**:
1. Print 9x6 checkerboard pattern
2. Capture 20 images at varying angles and distances (0.5-2m)
3. Run calibration: ros2 run camera_calibration cameracalibrator
4. Save calibration to camera_info/headerless YAML format
5. Validate with reprojection error < 0.5 pixels

### Lab 3.3: IMU Complementary Filter

**Implementation Steps**:
1. Read raw IMU data at high rate (500-1000Hz)
2. Compute accelerometer angles (roll, pitch)
3. Integrate gyroscope for orientation
4. Apply complementary filter (alpha ~0.98-0.99)
5. Compare with robot_localization EKF output
6. Tune alpha based on motion characteristics

---

*Research completed for Chapter 3: Sensors & Actuators*
*Document version: 1.0*
*Date: 2026-01-04*
