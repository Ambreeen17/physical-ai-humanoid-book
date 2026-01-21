#!/usr/bin/env python3
"""
Hello Physical AI: Your First Sensorimotor Loop.

This node implements a simple feedback loop for a simulated robot.
It subscribes to pose data and publishes velocity commands.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose  # Using turtlesim Pose for simplicity in Chapter 1
import sys


class SensorimotorLoopNode(Node):
    """
    A ROS 2 Node that demonstrates the Sense-Think-Act cycle.

    Attributes:
        threshold (float): The X-coordinate limit to trigger a reaction.
        publisher (Publisher): Sends velocity commands.
        subscription (Subscription): Receives robot pose updates.
    """

    def __init__(self) -> None:
        """Initialize the node, publishers, and subscribers."""
        super().__init__('sensorimotor_loop_node')

        # Parameters
        self.declare_parameter('boundary_x', 2.0)
        self.threshold = self.get_parameter('boundary_x').get_parameter_value().double_value

        # Actuator: Publisher to cmd_vel
        self.cmd_publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)

        # Sensor: Subscriber to pose
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.get_logger().info('Sensorimotor Loop Node started.')
        self.get_logger().info(f'Boundary threshold set to X = {self.threshold}')

    def pose_callback(self, msg: Pose) -> None:
        """
        Sense: Receive the robot's current pose.
        Think: Determine if the robot has crossed the boundary.
        Act: Send a velocity command based on the calculation.
        """
        # 1. Sense: Read the incoming data
        current_x = msg.x

        self.get_logger().info(
            f'TurtleBot Position: x={current_x:.1f}, y={msg.y:.1f}, theta={msg.theta:.1f}',
            throttle_duration_sec=1.0  # Log once per second
        )

        # 2. Think: Logic to react to the sensor data
        cmd = Twist()

        if current_x > self.threshold:
            # 3. Act: Generate a corrective command
            self.get_logger().warn(f'Robot past x={self.threshold} boundary, moving backward!')
            cmd.linear.x = -1.0
        else:
            # Move forward normally
            cmd.linear.x = 1.0

        # Send the actuator command
        self.cmd_publisher.publish(cmd)


def main(args: list[str] = None) -> None:
    """Entry point for the node."""
    rclpy.init(args=args)

    try:
        node = SensorimotorLoopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error starting node: {e}', file=sys.stderr)
    finally:
        # Cleanup
        rclpy.shutdown()


if __name__ == '__main__':
    main()
