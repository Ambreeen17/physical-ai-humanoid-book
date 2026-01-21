#!/usr/bin/env python3
"""
Lab 3.2 Solution: RGB-D Camera Calibration Node

This node performs camera calibration using checkerboard patterns
and verifies depth accuracy of RGB-D sensors.
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
import os


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

        # Camera matrix and distortion
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
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera_info', 10)
        self.calibration_pub = self.create_publisher(String, '/calibration/parameters', 10)
        self.depth_accuracy_pub = self.create_publisher(Float64, '/depth_accuracy', 10)

        # Create services
        self.calibrate_srv = self.create_service(Trigger, '/calibrate', self.calibrate_callback)
        self.save_srv = self.create_service(Trigger, '/save_calibration', self.save_calibration_callback)

        self.get_logger().info('Camera Calibrator initialized')
        self.get_logger().info(f'Checkerboard: {self.checkerboard_width}x{self.checkerboard_height} squares')
        self.get_logger().info(f'Waiting for {self.min_images} images...')

    def image_callback(self, msg: Image) -> None:
        """Callback for processing color images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        success, corners = self.detect_checkerboard(cv_image)

        if success:
            self.captured_images.append(cv_image.copy())
            self.corners_list.append(corners)
            self.get_logger().info(f'Captured image {len(self.captured_images)}/{self.min_images}')

            # Visualize
            vis_image = cv_image.copy()
            cv2.drawChessboardCorners(vis_image, (self.checkerboard_width - 1, self.checkerboard_height - 1), corners, True)
            cv2.imshow('Checkerboard', vis_image)
            cv2.waitKey(1)

    def detect_checkerboard(self, image: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """Detect checkerboard corners in an image."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        pattern_size = (self.checkerboard_width - 1, self.checkerboard_height - 1)
        success, corners = cv2.findChessboardCorners(
            gray, pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if not success:
            return False, None

        # Subpixel refinement
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        return True, corners_refined

    def depth_callback(self, msg: Image) -> None:
        """Callback for processing depth images."""
        if not self.calibrated:
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting depth: {e}')
            return

        if len(self.corners_list) > 0:
            error = self.verify_depth_accuracy(depth, self.corners_list[-1])
            if error is not None:
                self.depth_accuracy_pub.publish(Float64(data=error))
                self.get_logger().info(f'Depth error: {error*1000:.2f}mm')

    def calibrate_callback(self, request, response: TriggerResponse) -> TriggerResponse:
        """Service callback to perform camera calibration."""
        if len(self.captured_images) < self.min_images:
            response.success = False
            response.message = f'Need {self.min_images} images, have {len(self.captured_images)}'
            return response

        self.get_logger().info(f'Calibrating with {len(self.captured_images)} images...')

        # Prepare 3D object points
        obj_points = []
        img_points = []

        obj_pattern = np.zeros(
            ((self.checkerboard_width - 1) * (self.checkerboard_height - 1), 3),
            np.float32
        )
        obj_pattern[:, :2] = np.mgrid[
            0:self.checkerboard_width - 1,
            0:self.checkerboard_height - 1
        ].T.reshape(-1, 2) * self.square_size

        for corners in self.corners_list:
            obj_points.append(obj_pattern)
            img_points.append(corners)

        # Perform calibration
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points,
            self.captured_images[0].shape[:2][::-1],
            None, None
        )

        # Calculate reprojection error
        total_error = 0
        for i in range(len(obj_points)):
            img_proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
            error = cv2.norm(img_points[i], img_proj, cv2.NORM_L2) / len(img_proj)
            total_error += error
        mean_error = total_error / len(obj_points)

        # Store results
        self.camera_matrix = camera_matrix
        self.distortion_coeffs = dist_coeffs
        self.calibrated = True
        self.calibration_result = {
            'camera_matrix': camera_matrix.tolist(),
            'distortion_coeffs': dist_coeffs.tolist(),
            'reprojection_error': float(mean_error),
            'num_images': len(self.captured_images)
        }

        # Publish
        self.publish_camera_info()

        response.success = True
        response.message = f'Calibration complete! Mean error: {mean_error:.4f} pixels'
        self.get_logger().info(response.message)

        return response

    def verify_depth_accuracy(self, depth_image: np.ndarray, corners_2d: np.ndarray) -> Optional[float]:
        """Verify depth accuracy by comparing measured depths at corners."""
        try:
            depths = []
            for corner in corners_2d:
                x, y = int(corner[0]), int(corner[1])
                if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                    depth = depth_image[y, x]
                    if depth > 0 and not np.isnan(depth):
                        depths.append(depth)

            if len(depths) < 4:
                return None

            # Return std as error metric
            return float(np.std(depths))

        except Exception as e:
            self.get_logger().error(f'Error in depth verification: {e}')
            return None

    def save_calibration_callback(self, request, response: TriggerResponse) -> TriggerResponse:
        """Save calibration results to YAML file."""
        if not self.calibrated:
            response.success = False
            response.message = 'No calibration to save'
            return response

        calib_file = os.path.expanduser('~/.ros/camera_calibration.yaml')

        try:
            with open(calib_file, 'w') as f:
                f.write('# Camera Calibration Results\n\n')
                f.write(f'image_width: {self.captured_images[0].shape[1]}\n')
                f.write(f'image_height: {self.captured_images[0].shape[0]}\n\n')
                f.write('camera_name: rgb_camera\n\n')
                f.write('camera_matrix:\n')
                f.write('  rows: 3\n')
                f.write('  cols: 3\n')
                f.write('  data: [')
                for i, val in enumerate(self.camera_matrix.flatten()):
                    f.write(f'{val:.8f}')
                    if i < len(self.camera_matrix.flatten()) - 1:
                        f.write(', ')
                f.write(']\n\n')
                f.write('distortion_model: plumb_bob\n\n')
                f.write('distortion_coefficients:\n')
                f.write('  rows: 1\n')
                f.write('  cols: 5\n')
                f.write('  data: [')
                for i, val in enumerate(self.distortion_coeffs.flatten()):
                    f.write(f'{val:.8f}')
                    if i < len(self.distortion_coeffs.flatten()) - 1:
                        f.write(', ')
                f.write(']\n')

            response.success = True
            response.message = f'Calibration saved to {calib_file}'

        except Exception as e:
            response.success = False
            response.message = f'Failed to save: {e}'

        return response

    def publish_camera_info(self) -> None:
        """Publish CameraInfo message."""
        if not self.calibrated:
            return

        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.width = self.captured_images[0].shape[1]
        msg.height = self.captured_images[0].shape[0]
        msg.k = self.camera_matrix.flatten().tolist()
        msg.d = self.distortion_coeffs.flatten().tolist()
        msg.distortion_model = 'plumb_bob'
        self.camera_info_pub.publish(msg)


class DepthAnalyzer(Node):
    """Node for analyzing RGB-D depth accuracy."""

    def __init__(self) -> None:
        super().__init__('depth_analyzer')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10
        )
        self.depth_stats_pub = self.create_publisher(String, '/depth/stats', 10)

    def depth_callback(self, msg: Image) -> None:
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            return

        valid_depth = depth[(depth > 0) & (depth < 10) & ~np.isnan(depth)]

        if len(valid_depth) == 0:
            return

        stats = {
            'mean': float(np.mean(valid_depth)),
            'std': float(np.std(valid_depth)),
            'min': float(np.min(valid_depth)),
            'max': float(np.max(valid_depth))
        }

        self.depth_stats_pub.publish(String(data=json.dumps(stats)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraCalibrator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
