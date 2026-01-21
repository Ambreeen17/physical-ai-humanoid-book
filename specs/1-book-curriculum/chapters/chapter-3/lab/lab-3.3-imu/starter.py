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

        # Create publishers
        self.orientation_pub = self.create_publisher(Quaternion, '/imu/orientation', 10)
        self.euler_pub = self.create_publisher(Vector3, '/imu/euler', 10)
        self.accel_pub = self.create_publisher(Vector3, '/imu/raw_accel', 10)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/imu/pose', 10)

        self.get_logger().info(f'IMU Complementary Filter initialized')
        self.get_logger().info(f'Alpha: {self.alpha}')

    def imu_callback(self, msg: Imu) -> None:
        """
        Callback for processing IMU data.

        Args:
            msg: sensor_msgs/Imu message
        """
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

        # TODO: Calculate roll and pitch from accelerometer
        # accel_roll, accel_pitch = self.calculate_orientation_from_accel(ax, ay, az)

        # TODO: Integrate gyroscope data
        # delta_roll, delta_pitch, delta_yaw = self.integrate_gyro(gx, gy, gz, dt)

        # TODO: Update current orientation with gyro
        # gyro_roll = self.current_orientation[0] + delta_roll
        # gyro_pitch = self.current_orientation[1] + delta_pitch
        # gyro_yaw = self.current_orientation[2] + delta_yaw

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

        pass

    def calculate_orientation_from_accel(self, ax: float, ay: float, az: float
                                         ) -> Tuple[float, float]:
        """
        Calculate roll and pitch from accelerometer readings.

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

        Args:
            current: Current orientation estimate [roll, pitch, yaw]
            accel_angles: Orientation from accelerometer
            gyro_angles: Integrated orientation from gyroscope
            alpha: Filter coefficient (0-1)

        Returns:
            Updated orientation estimate

        TODO: Implement this function
        Formula:
            roll = alpha * gyro + (1 - alpha) * accel
            pitch = alpha * gyro + (1 - alpha) * accel
            yaw = gyro (no accel reference)
        """
        # YOUR CODE HERE
        pass

    def orientation_to_quaternion(self, euler: np.ndarray) -> Quaternion:
        """Convert Euler angles to quaternion."""
        q = quaternion_from_euler(euler[0], euler[1], euler[2])
        return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])


class IMUSimulator(Node):
    """Simple IMU node for testing without real hardware."""

    def __init__(self) -> None:
        """Initialize the IMU node."""
        super().__init__('imu_simulator')

        self.imu_pub = self.create_publisher(Imu, '/imu_data', 10)
        self.timer = self.create_timer(0.01, self.publish_imu)  # 100 Hz
        self.time = 0.0

        self.get_logger().info('IMU Simulator initialized')

    def publish_imu(self) -> None:
        """Publish simulated IMU data."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        t = self.time

        # Simulate some motion
        ax = 0.0 + 0.5 * np.sin(t)
        ay = 0.0 + 0.3 * np.cos(t * 1.3)
        az = 9.81 + 0.2 * np.sin(t * 0.7)

        gx = 0.1 * np.cos(t * 0.5)
        gy = 0.15 * np.sin(t * 0.3)
        gz = 0.05 * np.sin(t)

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
