#!/usr/bin/env python3
"""
Lab 3.2 Starter: RGB-D Camera Calibration Node

This node performs camera calibration using checkerboard patterns
and verifies depth accuracy of RGB-D sensors.

Starter Code - Complete the TODO sections!

Topics:
    Subscribed: /camera/color/image_raw (sensor_msgs/Image)
                /camera/depth/image_raw (sensor_msgs/Image)
    Published:  /camera_info (sensor_msgs/CameraInfo)
                /calibration/parameters (std_msgs/String)
                /depth_accuracy (std_msgs/Float64)

Services:
    /calibrate (std_srvs/srv/Trigger)
    /save_calibration (std_srvs/srv/Trigger)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger, TriggerResponse
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import List, Tuple, Optional
import json


class CameraCalibrator(Node):
    """Node for RGB-D camera calibration and depth accuracy verification."""

    def __init__(self) -> None:
        """Initialize the camera calibrator node."""
        super().__init__('camera_calibrator')

        # Calibration parameters
        self.declare_parameter('checkerboard_width', 9)
        self.declare_parameter('checkerboard_height', 6)
        self.declare_parameter('square_size', 0.025)  # 25mm squares
        self.declare_parameter('min_images', 10)
        self.declare_parameter('calibration_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth/image_raw')

        # Get parameters
        self.checkerboard_width = self.get_parameter('checkerboard_width').get_parameter_value().integer_value
        self.checkerboard_height = self.get_parameter('checkerboard_height').get_parameter_value().integer_value
        self.square_size = self.get_parameter('square_size').get_parameter_value().double_value
        self.min_images = self.get_parameter('min_images').get_parameter_value().integer_value
        self.calibration_topic = self.get_parameter('calibration_topic').get_parameter_value().string_value
        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value

        # Initialize calibration state
        self.captured_images: List[np.ndarray] = []
        self.corners_list: List[np.ndarray] = []
        self.calibrated = False
        self.calibration_result = None

        # Camera matrix and distortion (will be populated after calibration)
        self.camera_matrix = None
        self.distortion_coeffs = None

        # CV bridge for image conversion
        self.bridge = CvBridge()

        # Create subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.calibration_topic,
            self.image_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        # Create publishers
        self.camera_info_pub = self.create_publisher(
            CameraInfo,
            '/camera_info',
            10
        )
        self.calibration_pub = self.create_publisher(
            String,
            '/calibration/parameters',
            10
        )
        self.depth_accuracy_pub = self.create_publisher(
            Float64,
            '/depth_accuracy',
            10
        )

        # Create services
        self.calibrate_srv = self.create_service(
            Trigger,
            '/calibrate',
            self.calibrate_callback
        )
        self.save_srv = self.create_service(
            Trigger,
            '/save_calibration',
            self.save_calibration_callback
        )

        self.get_logger().info('Camera Calibrator initialized')

    def image_callback(self, msg: Image) -> None:
        """
        Callback for processing color images.

        Args:
            msg: ROS Image message
        """
        # TODO: Convert Image to OpenCV format using CvBridge
        # cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # TODO: Detect checkerboard corners
        # success, corners = self.detect_checkerboard(cv_image)

        # TODO: If detected, store image and corners
        # self.captured_images.append(cv_image.copy())
        # self.corners_list.append(corners)

        pass

    def detect_checkerboard(self, image: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Detect checkerboard corners in an image.

        Args:
            image: OpenCV image (BGR format)

        Returns:
            Tuple of (success, corners) where corners is Nx2 array

        TODO: Implement checkerboard detection
        Hint: Use cv2.findChessboardCorners with appropriate flags
        Then use cv2.cornerSubPix for subpixel refinement
        """
        # YOUR CODE HERE
        pass

    def depth_callback(self, msg: Image) -> None:
        """
        Callback for processing depth images.

        Args:
            msg: ROS Image message
        """
        if not self.calibrated:
            return

        # TODO: Convert depth image and verify accuracy
        # depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # if len(self.corners_list) > 0:
        #     error = self.verify_depth_accuracy(depth, self.corners_list[-1])
        pass

    def calibrate_callback(self, request, response: TriggerResponse) -> TriggerResponse:
        """
        Service callback to perform camera calibration.

        Args:
            request: Trigger request
            response: Trigger response to populate

        Returns:
            Response with success status and message

        TODO: Implement camera calibration using cv2.calibrateCamera
        1. Check minimum number of images
        2. Create 3D object points (world coordinates of corners)
        3. Call cv2.calibrateCamera
        4. Calculate reprojection error
        5. Store results and publish
        """
        if len(self.captured_images) < self.min_images:
            response.success = False
            response.message = f'Need at least {self.min_images} images'
            return response

        # YOUR CODE HERE
        pass

    def verify_depth_accuracy(self, depth_image: np.ndarray,
                                corners_2d: np.ndarray) -> Optional[float]:
        """
        Verify depth accuracy by comparing measured vs expected distances.

        Args:
            depth_image: Depth map in meters
            corners_2d: 2D corner coordinates in image

        Returns:
            Mean depth error in meters

        TODO: Implement depth accuracy verification
        1. Get depth values at corner positions
        2. Calculate statistics (mean, std)
        3. Return error metric
        """
        # YOUR CODE HERE
        pass

    def save_calibration_callback(self, request, response: TriggerResponse) -> TriggerResponse:
        """
        Service callback to save calibration results to file.

        Args:
            request: Trigger request
            response: Trigger response to populate

        Returns:
            Response with success status and message

        TODO: Save calibration to YAML file
        Use standard camera calibration YAML format
        """
        # YOUR CODE HERE
        pass

    def publish_camera_info(self) -> None:
        """Publish CameraInfo message with calibration parameters."""
        if not self.calibrated:
            return

        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.width = 640
        msg.height = 480
        msg.k = self.camera_matrix.flatten().tolist()
        msg.d = self.distortion_coeffs.flatten().tolist()
        msg.distortion_model = 'plumb_bob'

        self.camera_info_pub.publish(msg)


class DepthAnalyzer(Node):
    """Node for analyzing RGB-D depth accuracy."""

    def __init__(self) -> None:
        """Initialize the depth analyzer node."""
        super().__init__('depth_analyzer')

        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_topic', '/camera/color/image_raw')

        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.color_topic = self.get_parameter('color_topic').get_parameter_value().string_value

        self.bridge = CvBridge()

        # Create subscribers
        self.depth_sub = self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        # Publisher for depth statistics
        self.depth_stats_pub = self.create_publisher(String, '/depth/stats', 10)

        self.get_logger().info('Depth Analyzer initialized')

    def depth_callback(self, msg: Image) -> None:
        """
        Callback for processing depth images.

        Args:
            msg: ROS Image message
        """
        # TODO: Convert depth image and analyze
        # Calculate statistics (mean, std, min, max)
        # Publish as JSON string
        pass


def main(args=None) -> None:
    """Main entry point."""
    rclpy.init(args=args)
    node = CameraCalibrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Camera Calibrator...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
