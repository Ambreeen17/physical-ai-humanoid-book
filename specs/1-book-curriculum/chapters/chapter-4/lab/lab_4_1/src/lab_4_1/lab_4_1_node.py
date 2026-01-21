#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math


class lab41Node(Node):
    def __init__(self):
        super().__init__('lab_4_1_node')
        
        # Publisher for lab results
        self.publisher = self.create_publisher(String, 'lab_4_1/result', 10)
        
        # Subscriber for sensor data
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )
        
        # Timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_data)
        
        self.get_logger().info('Introduction to State Estimation - Basic Concepts node initialized')
        
    def joint_callback(self, msg):
        # Process joint state data for State Estimation
        self.get_logger().info(f'Received joint states: {len(msg.name)} joints')
        
    def process_data(self):
        # Process data according to State Estimation principles
        result = String()
        result.data = f'Processing State Estimation data'
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = lab41Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
