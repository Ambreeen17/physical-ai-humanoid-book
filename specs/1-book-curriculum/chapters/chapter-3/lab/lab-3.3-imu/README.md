# Lab 3.3: IMU Sensor Processing with Complementary Filter

**Duration:** 45 minutes
**ROS 2 Version:** Humble Hawksbill (Ubuntu 22.04)
**Topic:** IMU data processing, orientation estimation, and complementary filtering

## Learning Objectives

By the end of this lab, you will be able to:

1. **Subscribe to IMU data topics** (`/imu_data`) and parse sensor data (accelerometer, gyroscope, magnetometer)
2. **Calculate roll and pitch** angles from accelerometer readings
3. **Implement a complementary filter** to fuse accelerometer and gyroscope data
4. **Tune the alpha parameter** for optimal orientation estimation
5. **Publish filtered orientation** as a transformation or orientation message
6. **Visualize IMU data** using RViz2 or rqt

## Prerequisites

- ROS 2 Humble installed and sourced
- Basic understanding of Euler angles and quaternions
- Familiarity with sensor fusion concepts
- Completion of Lab 2.x (basic ROS 2 topics)

## Estimated Duration

- Setup and IMU theory: 10 minutes
- Accelerometer roll/pitch calculation: 12 minutes
- Complementary filter implementation: 15 minutes
- Parameter tuning and visualization: 8 minutes

## Architecture Overview

```
IMU Sensor (simulated or physical)
        |
        v
    /imu_data (sensor_msgs/Imu)
        |
        +---> Linear acceleration (ax, ay, az)
        +---> Angular velocity (gx, gy, gz)
        |
        v
+------------------+
|  Roll/Pitch      |  ---> Calculate from accelerometer
|  from Accel      |
+------------------+
        |
        v
+------------------+
|  Complementary   |  ---> Fuse accel + gyro
|     Filter       |
+------------------+
        |
        v
    /imu/orientation (geometry_msgs/msg/Quaternion)
    /tf (world -> imu_link)
```

## Background Theory

### IMU Data Structure

The `sensor_msgs/msg/Imu` message contains:

```
Header header           # Timestamp and frame
Quaternion orientation  # Fused orientation (if available)
float64 orientation_covariance[9]  # Uncertainty
Vector3 angular_velocity   # Gyroscope: (wx, wy, wz) in rad/s
float64 angular_velocity_covariance[9]
Vector3 linear_acceleration  # Accelerometer: (ax, ay, az) in m/s^2
float64 linear_acceleration_covariance[9]
```

### Accelerometer-Based Orientation

For a stationary IMU, gravity provides a reference vector:

```
Roll (rotation around X-axis):
    roll = atan2(ay, az)  [for horizontal orientation]

Pitch (rotation around Y-axis):
    pitch = atan2(-ax, sqrt(ay^2 + az^2))
```

### Gyroscope Integration

Gyroscope measures angular rate, so orientation is integrated:

```
roll_gyro += gx * dt
pitch_gyro += gy * dt
yaw_gyro += gz * dt
```

**Problem:** Gyro drifts over time (integration error accumulation)

### Complementary Filter

The complementary filter combines:
- **High-pass filter on gyro**: Captures fast rotations (no drift)
- **Low-pass filter on accelerometer**: Captures slow orientation (no noise)

```
orientation = alpha * (orientation_gyro) + (1 - alpha) * orientation_accel
```

Where:
- `alpha = tau / (tau + dt)`
- `tau` = time constant (typically 0.5 - 2 seconds)
- `dt` = sampling period

### Alpha Parameter Tuning

| Alpha Value | Behavior | Use Case |
|-------------|----------|----------|
| 0.9 - 0.99 | Trust gyro heavily | Fast motions, short integration |
| 0.8 - 0.9 | Balanced | General purpose |
| 0.5 - 0.8 | Trust accelerometer | Slow motions, static orientation |

## Dependencies

```bash
# Install required packages
sudo apt-get update
sudo apt-get install -y \
    ros-humble-imu-tools \
    ros-humble-rviz2 \
    ros-humble-rqt \
    python3-numpy

# For simulated IMU
sudo apt-get install -y ros-humble-gazebo-ros-pkgs
```

## Exercise: Implement IMU Orientation Filter

### Starter Code

Copy the following into `imu_processing/imu_processing/imu_filter_starter.py`:

```python
#!/usr/bin/env python3
"""
Lab 3.3 Starter: IMU Complementary Filter Node

This node subscribes to IMU data, calculates orientation using
a complementary filter, and publishes the fused orientation.

Starter Code - Complete the TODO sections!

Topics:
    Subscribed: /imu_data (sensor_msgs/Imu)
    Published:  /imu/orientation (geometry_msgs/msg/Quaternion)
                /imu/euler (geometry_msgs/msg/Vector3)
                /imu/raw_accel (geometry_msgs/msg/Vector3)

Parameters:
    alpha (float): Complementary filter coefficient (default: 0.98)
    frame_id (string): TF frame ID (default: 'imu_link')
    publish_tf (bool): Whether to publish TF transform (default: True)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler, euler_from_quaternion
import numpy as np
from typing import Tuple
import time


class IMUComplementaryFilter(Node):
    """Node for fusing IMU data using complementary filter."""

    def __init__(self) -> None:
        """Initialize the IMU filter node."""
        super().__init__('imu_complementary_filter')

        # Declare parameters
        self.declare_parameter('alpha', 0.98)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('input_topic', '/imu_data')

        # Get parameters
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value

        # Initialize orientation state
        # Current fused orientation as (roll, pitch, yaw)
        self.current_orientation = np.array([0.0, 0.0, 0.0])
        self.last_time = None

        # For storing raw accel orientation
        self.accel_roll = 0.0
        self.accel_pitch = 0.0

        # Create subscriber
        self.subscription = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            100  # Queue size
        )

        # Create publishers
        self.orientation_pub = self.create_publisher(
            Quaternion,
            '/imu/orientation',
            10
        )
        self.euler_pub = self.create_publisher(
            Vector3,
            '/imu/euler',
            10
        )
        self.accel_pub = self.create_publisher(
            Vector3,
            '/imu/raw_accel',
            10
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/imu/pose',
            10
        )

        # TF broadcaster (lazy initialization)
        self.tf_broadcaster = None

        self.get_logger().info(f'IMU Complementary Filter initialized')
        self.get_logger().info(f'Alpha: {self.alpha}')
        self.get_logger().info(f'Input topic: {self.input_topic}')

    def imu_callback(self, msg: Imu) -> None:
        """
        Callback for processing IMU data.

        Args:
            msg: sensor_msgs/Imu message
        """
        # Get current time
        current_time = self.get_clock().now()

        # Calculate delta time
        if self.last_time is None:
            dt = 0.01  # Initial guess
        else:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            dt = max(dt, 0.001)  # Minimum 1ms to prevent division by zero

        self.last_time = current_time

        # Extract accelerometer and gyroscope data
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # TODO: Calculate roll and pitch from accelerometer
        # accel_roll, accel_pitch = self.calculate_orientation_from_accel(ax, ay, az)

        # TODO: Integrate gyroscope data
        # gyro_roll, gyro_pitch, gyro_yaw = self.integrate_gyro(gx, gy, gz, dt)

        # TODO: Apply complementary filter
        # self.current_orientation = self.complementary_filter(
        #     self.current_orientation,
        #     (accel_roll, accel_pitch, gyro_yaw),
        #     (gyro_roll, gyro_pitch, gyro_yaw),
        #     self.alpha
        # )

        # TODO: Publish results
        # quaternion = self.orientation_to_quaternion(self.current_orientation)
        # self.orientation_pub.publish(quaternion)

        # euler_msg = Vector3(x=self.current_orientation[0],
        #                     y=self.current_orientation[1],
        #                     z=self.current_orientation[2])
        # self.euler_pub.publish(euler_msg)

        pass

    def calculate_orientation_from_accel(self, ax: float, ay: float, az: float
                                         ) -> Tuple[float, float]:
        """
        Calculate roll and pitch from accelerometer readings.

        Assumes IMU is stationary (gravity is the only acceleration).

        Args:
            ax, ay, az: Accelerometer readings in m/s^2

        Returns:
            Tuple of (roll, pitch) in radians

        TODO: Implement this function
        Formula:
            roll = atan2(ay, az)
            pitch = atan2(-ax, sqrt(ay^2 + az^2))
        """
        # YOUR CODE HERE
        pass

    def integrate_gyro(self, gx: float, gy: float, gz: float, dt: float
                       ) -> Tuple[float, float, float]:
        """
        Integrate gyroscope readings to update orientation.

        Args:
            gx, gy, gz: Angular velocities in rad/s
            dt: Time delta in seconds

        Returns:
            Tuple of (delta_roll, delta_pitch, delta_yaw)

        TODO: Implement this function
        Formula: angle += angular_velocity * dt
        """
        # YOUR CODE HERE
        pass

    def complementary_filter(self, current: np.ndarray,
                              accel_angles: Tuple[float, float, float],
                              gyro_angles: Tuple[float, float, float],
                              alpha: float) -> np.ndarray:
        """
        Apply complementary filter to fuse accelerometer and gyro orientations.

        The filter combines:
        - High-pass on gyro: captures fast rotations
        - Low-pass on accel: captures slow orientation

        Args:
            current: Current orientation estimate [roll, pitch, yaw]
            accel_angles: Orientation from accelerometer
            gyro_angles: Integrated orientation from gyroscope
            alpha: Filter coefficient (0-1), higher = more gyro trust

        Returns:
            Updated orientation estimate

        TODO: Implement this function
        Formula:
            orientation = alpha * gyro_angles + (1 - alpha) * accel_angles
        """
        # YOUR CODE HERE
        pass

    def orientation_to_quaternion(self, euler: np.ndarray) -> Quaternion:
        """
        Convert Euler angles to quaternion.

        Args:
            euler: Array of [roll, pitch, yaw] in radians

        Returns:
            Quaternion message (x, y, z, w)
        """
        # Use tf_transformations for conversion
        q = quaternion_from_euler(euler[0], euler[1], euler[2])
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class IMUNode(Node):
    """Simple IMU node for testing without real hardware."""

    def __init__(self) -> None:
        """Initialize the IMU node."""
        super().__init__('imu_node')

        self.declare_parameter('simulate_motion', True)
        self.simulate_motion = self.get_parameter('simulate_motion').get_parameter_value().bool_value

        # Create publisher
        self.imu_pub = self.create_publisher(Imu, '/imu_data', 10)

        # Timer for publishing
        self.timer = self.create_timer(0.01, self.publish_imu)  # 100 Hz

        # State for simulation
        self.time = 0.0

        self.get_logger().info('IMU Node initialized (simulation mode)')

    def publish_imu(self) -> None:
        """Publish simulated IMU data."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        if self.simulate_motion:
            # Simulate some motion
            t = self.time

            # Accelerometer: gravity + some motion
            ax = 0.0 + 0.5 * np.sin(t)
            ay = 0.0 + 0.3 * np.cos(t * 1.3)
            az = 9.81 + 0.2 * np.sin(t * 0.7)

            # Gyroscope: angular rates
            gx = 0.1 * np.cos(t * 0.5)
            gy = 0.15 * np.sin(t * 0.3)
            gz = 0.05 * np.sin(t)

        else:
            # Static IMU
            ax, ay, az = 0.0, 0.0, 9.81
            gx = gy = gz = 0.0

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        self.imu_pub.publish(msg)
        self.time += 0.01


def main(args=None) -> None:
    """Main entry point."""
    rclpy.init(args=args)
    node = IMUComplementaryFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down IMU filter...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Solution Code

```python
#!/usr/bin/env python3
"""
Lab 3.3 Solution: IMU Complementary Filter Node

This node subscribes to IMU data, calculates orientation using
a complementary filter, and publishes the fused orientation.

Topics:
    Subscribed: /imu_data (sensor_msgs/Imu)
    Published:  /imu/orientation (geometry_msgs/msg/Quaternion)
                /imu/euler (geometry_msgs/msg/Vector3)
                /imu/raw_accel (geometry_msgs/msg/Vector3)
                /imu/pose (geometry_msgs/msg/PoseWithCovarianceStamped)

Parameters:
    alpha (float): Complementary filter coefficient (default: 0.98)
    frame_id (string): TF frame ID (default: 'imu_link')
    publish_tf (bool): Whether to publish TF transform (default: True)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion, Vector3, PoseWithCovarianceStamped
from tf_transformations import quaternion_from_euler
import numpy as np
from typing import Tuple
from math import atan2, sqrt


class IMUComplementaryFilter(Node):
    """Node for fusing IMU data using complementary filter."""

    def __init__(self) -> None:
        """Initialize the IMU filter node."""
        super().__init__('imu_complementary_filter')

        # Declare parameters
        self.declare_parameter('alpha', 0.98)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('input_topic', '/imu_data')
        self.declare_parameter('use_mag_for_yaw', False)

        # Get parameters
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.use_mag = self.get_parameter('use_mag_for_yaw').get_parameter_value().bool_value

        # Initialize orientation state
        # Current fused orientation as [roll, pitch, yaw] in radians
        self.current_orientation = np.array([0.0, 0.0, 0.0])
        self.last_time = None

        # Store raw accel orientation for debugging
        self.accel_roll = 0.0
        self.accel_pitch = 0.0

        # Create subscriber
        self.subscription = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            100  # Queue size
        )
        self.subscription  # Prevent unused warning

        # Create publishers
        self.orientation_pub = self.create_publisher(
            Quaternion,
            '/imu/orientation',
            10
        )
        self.euler_pub = self.create_publisher(
            Vector3,
            '/imu/euler',
            10
        )
        self.accel_pub = self.create_publisher(
            Vector3,
            '/imu/raw_accel',
            10
        )
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped,
            '/imu/pose',
            10
        )

        # Statistics publisher
        self.stats_pub = self.create_publisher(Vector3, '/imu/stats', 10)

        # TF broadcaster (will be created on first use)
        self.tf_broadcaster = None

        self.get_logger().info(f'IMU Complementary Filter initialized')
        self.get_logger().info(f'Alpha: {self.alpha}, Frame: {self.frame_id}')
        self.get_logger().info(f'Subscribed to: {self.input_topic}')

    def imu_callback(self, msg: Imu) -> None:
        """
        Callback for processing IMU data.

        This is the main processing loop that:
        1. Calculates orientation from accelerometer
        2. Integrates gyroscope readings
        3. Fuses both using complementary filter
        4. Publishes the result

        Args:
            msg: sensor_msgs/Imu message containing accelerometer and gyro data
        """
        # Get current time
        current_time = self.get_clock().now()

        # Calculate delta time
        if self.last_time is None:
            dt = 0.01  # Initial guess for first sample (10ms)
        else:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            dt = max(dt, 0.001)  # Minimum 1ms to prevent division by zero
            dt = min(dt, 0.1)    # Maximum 100ms to prevent large jumps

        self.last_time = current_time

        # Extract accelerometer and gyroscope data
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Normalize accelerometer reading for better orientation estimate
        accel_magnitude = sqrt(ax * ax + ay * ay + az * az)

        # Publish raw accelerometer for debugging
        accel_msg = Vector3(x=ax, y=ay, z=az)
        self.accel_pub.publish(accel_msg)

        # Check if accelerometer data is valid (not saturated, not zero)
        if accel_magnitude < 1.0 or accel_magnitude > 20.0:
            self.get_logger().warn(f'Invalid accelerometer magnitude: {accel_magnitude:.2f} m/s^2')

        # Step 1: Calculate roll and pitch from accelerometer
        accel_roll, accel_pitch = self.calculate_orientation_from_accel(ax, ay, az)
        self.accel_roll = accel_roll
        self.accel_pitch = accel_pitch

        # Step 2: Integrate gyroscope data to update orientation
        delta_roll, delta_pitch, delta_yaw = self.integrate_gyro(gx, gy, gz, dt)

        # Update current orientation with gyro integration
        gyro_roll = self.current_orientation[0] + delta_roll
        gyro_pitch = self.current_orientation[1] + delta_pitch
        gyro_yaw = self.current_orientation[2] + delta_yaw

        # Normalize yaw to [-pi, pi]
        gyro_yaw = (gyro_yaw + np.pi) % (2 * np.pi) - np.pi

        # Step 3: Apply complementary filter
        # Fuse accelerometer-based orientation with gyro integration
        self.current_orientation = self.complementary_filter(
            self.current_orientation,
            (accel_roll, accel_pitch, gyro_yaw),  # Accel gives roll/pitch, gyro gives yaw
            (gyro_roll, gyro_pitch, gyro_yaw),
            self.alpha
        )

        # Step 4: Publish results
        self.publish_results(msg.header)

    def calculate_orientation_from_accel(self, ax: float, ay: float, az: float
                                         ) -> Tuple[float, float]:
        """
        Calculate roll and pitch from accelerometer readings.

        For a stationary IMU, gravity provides a stable reference vector.
        By measuring which direction "down" points, we can determine
        the tilt angles (roll and pitch) of the sensor.

        Note: This method cannot determine yaw (rotation around vertical axis)
        because gravity is always along the vertical axis.

        Args:
            ax, ay, az: Accelerometer readings in m/s^2 (in sensor frame)

        Returns:
            Tuple of (roll, pitch) in radians

        Formulas:
            roll = atan2(ay, az) - rotation around X-axis
            pitch = atan2(-ax, sqrt(ay^2 + az^2)) - rotation around Y-axis
        """
        # Roll: rotation around X-axis
        # Uses ay (Y component) and az (Z component) of gravity
        roll = atan2(ay, az)

        # Pitch: rotation around Y-axis
        # Uses ax (X component) and the horizontal components of gravity
        # The sqrt term gives the horizontal acceleration magnitude
        pitch = atan2(-ax, sqrt(ay * ay + az * az))

        return roll, pitch

    def integrate_gyro(self, gx: float, gy: float, gz: float, dt: float
                       ) -> Tuple[float, float, float]:
        """
        Integrate gyroscope readings to update orientation.

        Gyroscopes measure angular velocity, so we integrate over time
        to get angular displacement (orientation change).

        Args:
            gx, gy, gz: Angular velocities in rad/s (around X, Y, Z axes)
            dt: Time delta in seconds since last measurement

        Returns:
            Tuple of (delta_roll, delta_pitch, delta_yaw) in radians
        """
        # Simple Euler integration
        delta_roll = gx * dt
        delta_pitch = gy * dt
        delta_yaw = gz * dt

        return delta_roll, delta_pitch, delta_yaw

    def complementary_filter(self, current: np.ndarray,
                              accel_angles: Tuple[float, float, float],
                              gyro_angles: Tuple[float, float, float],
                              alpha: float) -> np.ndarray:
        """
        Apply complementary filter to fuse accelerometer and gyro orientations.

        The complementary filter combines the best of both sensors:
        - Accelerometer: Provides stable long-term orientation reference
          (no drift), but is noisy and sensitive to motion
        - Gyroscope: Provides precise short-term orientation tracking
          (no noise), but drifts over time due to integration errors

        The filter works by:
        - Using gyro for fast changes (high-pass filter behavior)
        - Using accelerometer for slow drifts (low-pass filter behavior)

        Args:
            current: Current orientation estimate [roll, pitch, yaw]
            accel_angles: Orientation from accelerometer (roll, pitch, yaw)
            gyro_angles: Integrated orientation from gyroscope
            alpha: Filter coefficient (0-1)
                   Higher values trust gyro more (better for fast motion)
                   Lower values trust accelerometer more (better for static)

        Returns:
            Updated orientation estimate as numpy array [roll, pitch, yaw]
        """
        # Complementary filter formula:
        # orientation = alpha * gyro_angles + (1 - alpha) * accel_angles
        #
        # For roll and pitch: we trust the accelerometer for long-term stability
        # For yaw: we only have gyro (accelerometer can't measure yaw)

        roll = alpha * gyro_angles[0] + (1 - alpha) * accel_angles[0]
        pitch = alpha * gyro_angles[1] + (1 - alpha) * accel_angles[1]
        yaw = gyro_angles[2]  # Yaw only from gyro (no accelerometer reference)

        return np.array([roll, pitch, yaw])

    def publish_results(self, header) -> None:
        """Publish orientation results."""
        # Convert Euler angles to quaternion
        quaternion = self.orientation_to_quaternion(self.current_orientation)

        # Publish quaternion orientation
        quaternion.header = header
        self.orientation_pub.publish(quaternion)

        # Publish Euler angles
        euler_msg = Vector3(
            x=self.current_orientation[0],
            y=self.current_orientation[1],
            z=self.current_orientation[2]
        )
        self.euler_pub.publish(euler_msg)

        # Publish pose with covariance
        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header = header
        pose_msg.pose.pose.position.x = 0.0
        pose_msg.pose.pose.position.y = 0.0
        pose_msg.pose.pose.position.z = 0.0
        pose_msg.pose.pose.orientation = quaternion
        self.pose_pub.publish(pose_msg)

        # Publish statistics (accel magnitude for validation)
        stats_msg = Vector3(
            x=self.accel_roll,
            y=self.accel_pitch,
            z=self.alpha  # Report alpha as third value for monitoring
        )
        self.stats_pub.publish(stats_msg)

    def orientation_to_quaternion(self, euler: np.ndarray) -> Quaternion:
        """
        Convert Euler angles (roll, pitch, yaw) to quaternion.

        Args:
            euler: Array of [roll, pitch, yaw] in radians

        Returns:
            Quaternion message (x, y, z, w)
        """
        q = quaternion_from_euler(euler[0], euler[1], euler[2])
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class IMUSimulator(Node):
    """Simulated IMU for testing without hardware."""

    def __init__(self) -> None:
        """Initialize the IMU simulator."""
        super().__init__('imu_simulator')

        self.declare_parameter('noise_level', 0.01)
        self.declare_parameter('drift_rate', 0.001)
        self.noise_level = self.get_parameter('noise_level').get_parameter_value().double_value
        self.drift_rate = self.get_parameter('drift_rate').get_parameter_value().double_value

        self.imu_pub = self.create_publisher(Imu, '/imu_data', 10)
        self.timer = self.create_timer(0.01, self.publish_imu)  # 100 Hz

        self.time = 0.0
        self.gyro_drift = np.array([0.0, 0.0, 0.0])

        self.get_logger().info('IMU Simulator initialized')

    def publish_imu(self) -> None:
        """Publish simulated IMU data with realistic noise and drift."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        t = self.time

        # Simulate motion: combine several sine waves for realism
        # Base orientation (slow oscillation)
        roll_true = 0.1 * np.sin(t * 0.5)
        pitch_true = 0.15 * np.cos(t * 0.3)
        yaw_true = 0.05 * t  # Slow yaw rotation

        # Add gyro drift over time
        self.gyro_drift += np.array([self.drift_rate, self.drift_rate * 0.5, self.drift_rate * 0.3]) * 0.01

        # Calculate expected accelerations from orientation
        # For a gravity vector rotated by (roll, pitch, yaw)
        g = 9.81
        ax = g * np.sin(pitch_true)
        ay = g * np.sin(roll_true) * np.cos(pitch_true)
        az = g * np.cos(roll_true) * np.cos(pitch_true)

        # Add motion acceleration (simulated)
        motion_ax = 0.5 * np.sin(t * 2)
        motion_ay = 0.3 * np.cos(t * 1.5)
        motion_az = 0.1 * np.sin(t)

        # Combine and add noise
        ax = ax + motion_ax + np.random.normal(0, self.noise_level)
        ay = ay + motion_ay + np.random.normal(0, self.noise_level)
        az = az + motion_az + np.random.normal(0, self.noise_level)

        # Calculate angular rates (derivative of true orientation)
        gx = 0.1 * 0.5 * np.cos(t * 0.5)  # d(roll)/dt
        gy = -0.15 * 0.3 * np.sin(t * 0.3)  # d(pitch)/dt
        gz = 0.05 + self.gyro_drift[2]  # d(yaw)/dt + drift

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx + np.random.normal(0, self.noise_level * 0.1)
        msg.angular_velocity.y = gy + np.random.normal(0, self.noise_level * 0.1)
        msg.angular_velocity.z = gz + np.random.normal(0, self.noise_level * 0.1)

        # Set covariance matrices (diagonal = variance)
        cov = self.noise_level ** 2
        msg.linear_acceleration_covariance = [cov, 0, 0, 0, cov, 0, 0, 0, cov]
        msg.angular_velocity_covariance = [cov * 0.1, 0, 0, 0, cov * 0.1, 0, 0, 0, cov * 0.1]

        self.imu_pub.publish(msg)
        self.time += 0.01


def main(args=None) -> None:
    """Main entry point."""
    rclpy.init(args=args)
    node = IMUComplementaryFilter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down IMU filter...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Launch File

Create `imu_processing/launch/imu_demo.launch.py`:

```python
#!/usr/bin/env python3
"""
Launch file for IMU complementary filter demo.

This launch file starts:
1. IMU simulator (generates synthetic IMU data)
2. IMU complementary filter node
3. RViz2 for visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the IMU demo."""
    ld = LaunchDescription()

    # IMU simulator (generates synthetic data)
    imu_simulator = Node(
        package='imu_processing',
        executable='imu_simulator',
        name='imu_simulator',
        output='screen',
        parameters=[
            {'noise_level': 0.05},
            {'drift_rate': 0.0001}
        ]
    )

    # IMU complementary filter
    imu_filter = Node(
        package='imu_processing',
        executable='imu_filter',
        name='imu_complementary_filter',
        output='screen',
        parameters=[
            {'alpha': 0.98},
            {'frame_id': 'imu_link'},
            {'publish_tf': True},
            {'input_topic': '/imu_data'}
        ]
    )

    # RViz2 configuration
    rviz_config = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/opt/ros/humble/share/rviz2/default.rviz'],
        output='screen'
    )

    ld.add_action(imu_simulator)
    ld.add_action(imu_filter)
    ld.add_action(rviz_config)

    return ld
```

## Running the Lab

### Step 1: Verify Installation

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Check tf_transformations
python3 -c "from tf_transformations import quaternion_from_euler; print('tf_transformations OK')"

# Build package
cd ~/ros2_lab
colcon build --packages-select imu_processing
source install/setup.bash
```

### Step 2: Run the Demo

```bash
# Terminal 1: Start IMU simulator (generates data)
ros2 run imu_processing imu_simulator

# Terminal 2: Start complementary filter
ros2 run imu_processing imu_filter

# Terminal 3: Monitor topics
ros2 topic list
ros2 topic echo /imu/orientation
ros2 topic echo /imu/euler
```

### Step 3: Visualize Results

```bash
# Terminal 4: Open RViz2
rviz2

# In RViz2:
# 1. Add -> By topic -> /imu/orientation -> Quaternion
# 2. Set fixed frame to 'imu_link'
# 3. Observe the orientation visualization
```

### Step 4: Verify Filter Behavior

```bash
# Check orientation values
ros2 topic echo /imu/euler
# Should show:
# x: roll (radians)
# y: pitch (radians)
# z: yaw (radians)

# Monitor rate
ros2 topic hz /imu/orientation
# Should be around 100 Hz
```

## Expected Output

```
[imu_complementary_filter] [INFO] IMU Complementary Filter initialized
[imu_complementary_filter] [INFO] Alpha: 0.98, Frame: imu_link
[imu_complementary_filter] [INFO] Subscribed to: /imu_data

# After receiving data:
[imu_complementary_filter] [INFO] Orientation: roll=0.05, pitch=0.12, yaw=0.03
```

## Tuning the Alpha Parameter

### Experiment 1: High Alpha (0.99)

```bash
ros2 param set /imu_complementary_filter alpha 0.99
```

- Trust gyro heavily
- Fast response to motion
- Yaw will drift over time
- Roll/pitch remain stable (accel reference)

### Experiment 2: Medium Alpha (0.95)

```bash
ros2 param set /imu_complementary_filter alpha 0.95
```

- Balanced fusion
- Good compromise between noise and drift

### Experiment 3: Low Alpha (0.80)

```bash
ros2 param set /imu_complementary_filter alpha 0.80
```

- Trust accelerometer more
- Less drift but more noise
- Better for slow/static applications

## Troubleshooting

### Issue: Orientation jumps or is unstable

```bash
# Check accelerometer magnitude
ros2 topic echo /imu/raw_accel
# Should be approximately 9.81 m/s^2 when stationary

# If magnitude is wrong, check sensor calibration
```

### Issue: Yaw drifts continuously

```bash
# This is expected - gyro integration drifts
# Solution: Use a magnetometer for yaw correction
# Set parameter: use_mag_for_yaw: true
```

### Issue: No data on /imu_data

```bash
# Check if IMU is publishing
ros2 topic list
ros2 topic hz /imu_data

# If using real IMU, check device permissions
ls -l /dev/ttyUSB*
sudo usermod -a -G dialout $USER
```

### Issue: tf2 TransformException

```bash
# Check TF tree
ros2 run tf2_ros tf2_echo world imu_link

# Ensure publish_tf parameter is True
ros2 param get /imu_complementary_filter publish_tf
```

## Extensions and Challenges

### Challenge 1: Madgwick/Mahony Filter

Implement a more advanced filter:

```python
def madgwick_update(self, ax, ay, az, gx, gy, gz, dt, beta=0.1):
    """Madgwick filter for orientation estimation.

    Uses quaternion-based fusion with gradient descent.
    """
    q = self.current_quaternion  # [w, x, y, z]

    # Normalize accelerometer
    norm = sqrt(ax*ax + ay*ay + az*az)
    ax, ay, az = ax/norm, ay/norm, az/norm

    # Quaternion update using gyroscope
    q_dot = 0.5 * np.cross(q[:3], [gx, gy, gz]) - beta * gradient(q, ax, ay, az)

    q += q_dot * dt
    q /= np.linalg.norm(q)

    return q
```

### Challenge 2: Magnetometer Yaw Correction

Add magnetometer for full 3D orientation:

```python
def calculate_yaw_from_mag(self, mx, my, mz, roll, pitch):
    """Calculate yaw using magnetometer and tilt compensation."""
    # Tilt compensation
    mx_comp = mx * np.cos(roll) + my * np.sin(roll) * np.sin(pitch) + mz * np.sin(roll) * np.cos(pitch)
    my_comp = -my * np.cos(roll) + mz * np.sin(roll)

    # Calculate yaw
    yaw = atan2(-my_comp, mx_comp)
    return yaw
```

### Challenge 3: EKF (Extended Kalman Filter)

Implement a full EKF for sensor fusion:

```python
class IMUEKF:
    """Extended Kalman Filter for IMU orientation estimation.

    State: [roll, pitch, yaw, gyro_bias_x, gyro_bias_y, gyro_bias_z]
    """
    def predict(self, gyro, dt):
        """Predict step using gyro integration."""
        pass

    def update(self, accel):
        """Update step using accelerometer."""
        pass
```

## Evaluation Criteria

| Criteria | Points |
|----------|--------|
| IMU message parsing | 10 |
| Accelerometer roll/pitch calculation | 20 |
| Gyroscope integration | 15 |
| Complementary filter implementation | 25 |
| Alpha parameter tuning | 10 |
| Topic publishing | 10 |
| Code quality and documentation | 10 |
| **Total** | **100** |

## References

- [ROS 2 IMU Message Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
- [Complementary Filter Reference](https://scholar.google.com/scholar?q=complementary+filter+IMU+orientation)
- [Madgwick Filter Paper](https://www.x-io.co.uk/res/doc/madgwick_internal_report.pdf)
- [tf_transformations Documentation](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Quaternion-Fundamentals.html)
