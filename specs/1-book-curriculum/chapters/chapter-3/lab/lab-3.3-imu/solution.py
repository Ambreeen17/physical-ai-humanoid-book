#!/usr/bin/env python3
"""
Lab 3.3 Solution: IMU Complementary Filter Node

This node subscribes to IMU data, calculates orientation using
a complementary filter, and publishes the fused orientation.
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

        # Get parameters
        self.alpha = self.get_parameter('alpha').get_parameter_value().double_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value

        # Initialize orientation state
        self.current_orientation = np.array([0.0, 0.0, 0.0])
        self.last_time = None
        self.accel_roll = 0.0
        self.accel_pitch = 0.0

        # Create subscriber
        self.subscription = self.create_subscription(
            Imu,
            self.input_topic,
            self.imu_callback,
            100
        )
        self.subscription

        # Create publishers
        self.orientation_pub = self.create_publisher(Quaternion, '/imu/orientation', 10)
        self.euler_pub = self.create_publisher(Vector3, '/imu/euler', 10)
        self.accel_pub = self.create_publisher(Vector3, '/imu/raw_accel', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/imu/pose', 10)

        self.get_logger().info(f'IMU Complementary Filter initialized')
        self.get_logger().info(f'Alpha: {self.alpha}, Frame: {self.frame_id}')

    def imu_callback(self, msg: Imu) -> None:
        """Callback for processing IMU data."""
        current_time = self.get_clock().now()

        if self.last_time is None:
            dt = 0.01
        else:
            dt = (current_time - self.last_time).nanoseconds / 1e9
            dt = max(dt, 0.001)
            dt = min(dt, 0.1)

        self.last_time = current_time

        # Extract accelerometer and gyroscope data
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z

        # Publish raw accelerometer
        accel_msg = Vector3(x=ax, y=ay, z=az)
        self.accel_pub.publish(accel_msg)

        # Check accelerometer magnitude
        accel_magnitude = sqrt(ax * ax + ay * ay + az * az)
        if accel_magnitude < 1.0 or accel_magnitude > 20.0:
            self.get_logger().warn(f'Invalid accel magnitude: {accel_magnitude:.2f}')

        # Step 1: Calculate roll and pitch from accelerometer
        accel_roll, accel_pitch = self.calculate_orientation_from_accel(ax, ay, az)
        self.accel_roll = accel_roll
        self.accel_pitch = accel_pitch

        # Step 2: Integrate gyroscope data
        delta_roll, delta_pitch, delta_yaw = self.integrate_gyro(gx, gy, gz, dt)

        # Update current orientation with gyro integration
        gyro_roll = self.current_orientation[0] + delta_roll
        gyro_pitch = self.current_orientation[1] + delta_pitch
        gyro_yaw = self.current_orientation[2] + delta_yaw

        # Normalize yaw to [-pi, pi]
        gyro_yaw = (gyro_yaw + np.pi) % (2 * np.pi) - np.pi

        # Step 3: Apply complementary filter
        self.current_orientation = self.complementary_filter(
            self.current_orientation,
            (accel_roll, accel_pitch, gyro_yaw),
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

        Args:
            ax, ay, az: Accelerometer readings in m/s^2

        Returns:
            Tuple of (roll, pitch) in radians
        """
        # Roll: rotation around X-axis
        # Uses ay (Y component) and az (Z component) of gravity
        roll = atan2(ay, az)

        # Pitch: rotation around Y-axis
        # Uses ax (X component) and the horizontal components of gravity
        pitch = atan2(-ax, sqrt(ay * ay + az * az))

        return roll, pitch

    def integrate_gyro(self, gx: float, gy: float, gz: float, dt: float
                       ) -> Tuple[float, float, float]:
        """
        Integrate gyroscope readings to update orientation.

        Gyroscopes measure angular velocity, so we integrate over time
        to get angular displacement (orientation change).

        Args:
            gx, gy, gz: Angular velocities in rad/s
            dt: Time delta in seconds

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

        The complementary filter combines:
        - Accelerometer: stable long-term reference (no drift), but noisy
        - Gyroscope: precise short-term tracking (no noise), but drifts

        Args:
            current: Current orientation estimate [roll, pitch, yaw]
            accel_angles: Orientation from accelerometer (roll, pitch, yaw)
            gyro_angles: Integrated orientation from gyroscope
            alpha: Filter coefficient (0-1)

        Returns:
            Updated orientation estimate
        """
        # Fuse roll and pitch: trust both sensors proportionally to alpha
        # Higher alpha = trust gyro more (faster response, some drift)
        # Lower alpha = trust accelerometer more (smoother, but noisier)
        roll = alpha * gyro_angles[0] + (1 - alpha) * accel_angles[0]
        pitch = alpha * gyro_angles[1] + (1 - alpha) * accel_angles[1]

        # Yaw: only from gyro (accelerometer cannot measure yaw)
        yaw = gyro_angles[2]

        return np.array([roll, pitch, yaw])

    def publish_results(self, header) -> None:
        """Publish orientation results."""
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

    def orientation_to_quaternion(self, euler: np.ndarray) -> Quaternion:
        """Convert Euler angles to quaternion."""
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
        roll_true = 0.1 * np.sin(t * 0.5)
        pitch_true = 0.15 * np.cos(t * 0.3)
        yaw_true = 0.05 * t  # Slow yaw rotation

        # Add gyro drift over time
        self.gyro_drift += np.array([self.drift_rate, self.drift_rate * 0.5, self.drift_rate * 0.3]) * 0.01

        # Calculate expected accelerations from orientation
        g = 9.81
        ax = g * np.sin(pitch_true)
        ay = g * np.sin(roll_true) * np.cos(pitch_true)
        az = g * np.cos(roll_true) * np.cos(pitch_true)

        # Add motion acceleration
        motion_ax = 0.5 * np.sin(t * 2)
        motion_ay = 0.3 * np.cos(t * 1.5)
        motion_az = 0.1 * np.sin(t)

        # Combine and add noise
        ax = ax + motion_ax + np.random.normal(0, self.noise_level)
        ay = ay + motion_ay + np.random.normal(0, self.noise_level)
        az = az + motion_az + np.random.normal(0, self.noise_level)

        # Calculate angular rates
        gx = 0.1 * 0.5 * np.cos(t * 0.5)
        gy = -0.15 * 0.3 * np.sin(t * 0.3)
        gz = 0.05 + self.gyro_drift[2]

        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        msg.angular_velocity.x = gx + np.random.normal(0, self.noise_level * 0.1)
        msg.angular_velocity.y = gy + np.random.normal(0, self.noise_level * 0.1)
        msg.angular_velocity.z = gz + np.random.normal(0, self.noise_level * 0.1)

        # Set covariance matrices
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
