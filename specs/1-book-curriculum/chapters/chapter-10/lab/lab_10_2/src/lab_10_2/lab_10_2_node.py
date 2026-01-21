#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math


class lab102Node(Node):
    def __init__(self):
        super().__init__('lab_10_2_node')
        
        # Publisher for lab results
        self.publisher = self.create_publisher(String, 'lab_10_2/result', 10)
        
        # Subscriber for sensor data
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )
        
        # Timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_data)
        
        self.get_logger().info('Intermediate Reinforcement Learning for Robotics - Implementation node initialized')
        
    def joint_callback(self, msg):
        # Process joint state data for Reinforcement Learning for Robotics
        self.get_logger().info(f'Received joint states: {len(msg.name)} joints')
        
    def process_data(self):
        # Process data according to Reinforcement Learning for Robotics principles
        result = String()
        result.data = f'Processing Reinforcement Learning for Robotics data'
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = lab102Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
