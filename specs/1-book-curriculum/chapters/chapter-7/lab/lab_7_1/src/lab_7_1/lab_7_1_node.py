#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math


class lab71Node(Node):
    def __init__(self):
        super().__init__('lab_7_1_node')
        
        # Publisher for lab results
        self.publisher = self.create_publisher(String, 'lab_7_1/result', 10)
        
        # Subscriber for sensor data
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )
        
        # Timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_data)
        
        self.get_logger().info('Introduction to Motion Planning - Basic Concepts node initialized')
        
    def joint_callback(self, msg):
        # Process joint state data for Motion Planning
        self.get_logger().info(f'Received joint states: {len(msg.name)} joints')
        
    def process_data(self):
        # Process data according to Motion Planning principles
        result = String()
        result.data = f'Processing Motion Planning data'
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = lab71Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
