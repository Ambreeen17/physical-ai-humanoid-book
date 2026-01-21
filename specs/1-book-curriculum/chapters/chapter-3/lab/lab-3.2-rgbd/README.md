# Lab 3.2: RGB-D Camera Calibration

**Duration:** 30 minutes
**ROS 2 Version:** Humble Hawksbill (Ubuntu 22.04)
**Topic:** Camera calibration with checkerboard patterns and depth accuracy verification

## Learning Objectives

By the end of this lab, you will be able to:

1. **Create calibration targets** (checkerboard patterns) and understand camera calibration requirements
2. **Implement OpenCV camera calibration** using `calibrateCamera` for intrinsic parameters
3. **Calculate and verify depth accuracy** by comparing measured vs. actual distances
4. **Publish calibration results** as ROS 2 parameters for downstream nodes
5. **Apply undistortion** to rectify RGB and depth images

## Prerequisites

- ROS 2 Humble installed and sourced
- OpenCV installed with Python bindings (`python3-opencv`)
- Basic understanding of camera geometry and pinhole model
- A printed checkerboard pattern (8x6 or 9x6 squares)

## Estimated Duration

- Setup and theory: 8 minutes
- Checkerboard detection: 10 minutes
- Camera calibration: 7 minutes
- Depth accuracy verification: 5 minutes

## Architecture Overview

```
RGB-D Camera (simulated or physical)
        |
        +---> /camera/color/image_raw (sensor_msgs/Image)
        +---> /camera/color/camera_info (sensor_msgs/CameraInfo)
        +---> /camera/depth/image_raw (sensor_msgs/Image)
        |
        v
+------------------+
|  Checkerboard    |  ---> Detect corners
|    Detection     |
+------------------+
        |
        v
+------------------+
|  Calibrate       |  ---> Intrinsic matrix (K)
|    Camera        |  ---> Distortion coefficients (D)
+------------------+
        |
        v
+------------------+
|  Depth Accuracy  |  ---> Measure depth errors
|    Verification  |
+------------------+
        |
        v
    /camera_info (updated)
    /undistorted_image
```

## Background Theory

### Camera Pinhole Model

A camera projects 3D points onto a 2D image plane using the pinhole model:

```
[Xc, Yc, Zc]  --->  [u, v] = [fx * Xc/Zc + cx, fy * Yc/Zc + cy]

Where:
- fx, fy: Focal lengths (in pixels)
- cx, cy: Principal point coordinates
- K: Intrinsic matrix = [[fx, 0, cx], [0, fy, cy], [0, 0, 1]]
```

### Lens Distortion

Real cameras have radial and tangential distortion:

**Radial distortion** (barrel/pincushion):
- Points further from center are displaced more
- Model: `x_distorted = x(1 + k1*r^2 + k2*r^4 + k3*r^6)`

**Tangential distortion** (decentering):
- Caused by lens not being perfectly parallel to sensor
- Model: `x_distorted = x + (2*k1*x*y + k2*(r^2 + 2*x^2))`

### Checkerboard Calibration

The standard approach:
1. Capture 10-20 images of a checkerboard from various angles
2. Detect corner positions in each image
3. Use known square size to create 3D-2D correspondences
4. Optimize camera parameters using bundle adjustment

### Depth Accuracy

For RGB-D cameras, depth accuracy depends on:
- Baseline (for stereo) or structured light pattern (for structured light)
- Environmental factors (ambient light, surface reflectivity)
- Distance from camera (typically degrades with distance)

## Dependencies

```bash
# Install required packages
sudo apt-get update
sudo apt-get install -y \
    ros-humble-image-transport \
    ros-humble-cv-bridge \
    ros-humble-image-publisher \
    python3-opencv \
    python3-numpy

# For simulated camera
sudo apt-get install -y ros-humble-gazebo-ros-pkgs
```

## Exercise: Implement RGB-D Calibration Node

### Starter Code

Copy the following into `camera_calibration/camera_calibration/camera_calibrator_starter.py`:

```python
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
        self.get_logger().info(f'Waiting for images on {self.calibration_topic}')

    def image_callback(self, msg: Image) -> None:
        """
        Callback for processing color images.

        Args:
            msg: ROS Image message
        """
        # TODO: Convert Image to OpenCV format
        # TODO: Detect checkerboard corners
        # TODO: Store image if corners found
        pass

    def detect_checkerboard(self, image: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Detect checkerboard corners in an image.

        Args:
            image: OpenCV image (BGR format)

        Returns:
            Tuple of (success, corners) where corners is Nx2 array
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # TODO: Find checkerboard corners using cv2.findChessboardCorners
        # Use cv2.CALIB_CB_ADAPTIVE_THRESH and cv2.CALIB_CB_NORMALIZE_IMAGE

        # TODO: Refine corners using cv2.cornerSubPix

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
        pass

    def calibrate_callback(self, request, response: TriggerResponse) -> TriggerResponse:
        """
        Service callback to perform camera calibration.

        Args:
            request: Trigger request
            response: Trigger response to populate

        Returns:
            Response with success status and message
        """
        # TODO: Implement camera calibration using cv2.calibrateCamera
        # Requirements:
        # 1. Check minimum number of images
        # 2. Prepare 3D object points (world coordinates)
        # 3. Call cv2.calibrateCamera
        # 4. Calculate reprojection error
        # 5. Set calibrated flag and store results

        pass

    def verify_depth_accuracy(self, depth_image: np.ndarray,
                                corners_2d: np.ndarray) -> float:
        """
        Verify depth accuracy by comparing measured vs expected distances.

        Args:
            depth_image: Depth map in meters
            corners_2d: 2D corner coordinates in image

        Returns:
            Mean depth error in meters
        """
        # TODO: Implement depth accuracy verification
        # 1. Get depth values at corner positions
        # 2. Calculate expected distance from camera to checkerboard
        # 3. Compare and compute error statistics

        pass

    def save_calibration_callback(self, request, response: TriggerResponse) -> TriggerResponse:
        """
        Service callback to save calibration results to file.

        Args:
            request: Trigger request
            response: Trigger response to populate

        Returns:
            Response with success status and message
        """
        # TODO: Save calibration to YAML file
        # Use standard camera calibration YAML format

        pass

    def publish_camera_info(self) -> None:
        """Publish CameraInfo message with calibration parameters."""
        if not self.calibrated:
            return

        msg = CameraInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.width = 640  # Adjust based on your camera
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
        self.latest_depth = None

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
        # Convert depth image
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        # TODO: Analyze depth image
        # 1. Calculate statistics (mean, std, min, max)
        # 2. Filter invalid values (0, NaN,Inf)
        # 3. Publish statistics

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
```

### Solution Code

```python
#!/usr/bin/env python3
"""
Lab 3.2 Solution: RGB-D Camera Calibration Node

This node performs camera calibration using checkerboard patterns
and verifies depth accuracy of RGB-D sensors.

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
        self.get_logger().info(f'Checkerboard: {self.checkerboard_width}x{self.checkerboard_height} squares')
        self.get_logger().info(f'Square size: {self.square_size*1000:.1f}mm')
        self.get_logger().info(f'Waiting for images on {self.calibration_topic}')

    def image_callback(self, msg: Image) -> None:
        """
        Callback for processing color images.

        Args:
            msg: ROS Image message
        """
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
            return

        # Detect checkerboard corners
        success, corners = self.detect_checkerboard(cv_image)

        if success:
            # Store image and corners for later calibration
            self.captured_images.append(cv_image.copy())
            self.corners_list.append(corners)

            self.get_logger().info(
                f'Captured image {len(self.captured_images)}/{self.min_images}'
            )

            # Visualize detected corners
            vis_image = cv_image.copy()
            cv2.drawChessboardCorners(
                vis_image,
                (self.checkerboard_width - 1, self.checkerboard_height - 1),
                corners,
                True
            )
            cv2.imshow('Checkerboard Detection', vis_image)
            cv2.waitKey(1)

    def detect_checkerboard(self, image: np.ndarray) -> Tuple[bool, Optional[np.ndarray]]:
        """
        Detect checkerboard corners in an image.

        The function converts the image to grayscale, then uses OpenCV's
        findChessboardCorners to locate the internal corner points.

        Args:
            image: OpenCV image (BGR format)

        Returns:
            Tuple of (success, corners) where corners is Nx2 array of (x, y)
        """
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Find checkerboard corners
        # Note: OpenCV uses (columns, rows) which is (width-1, height-1)
        pattern_size = (self.checkerboard_width - 1, self.checkerboard_height - 1)
        success, corners = cv2.findChessboardCorners(
            gray,
            pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
        )

        if not success:
            return False, None

        # Refine corner positions using subpixel accuracy
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        corners_refined = cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            criteria
        )

        return True, corners_refined

    def depth_callback(self, msg: Image) -> None:
        """
        Callback for processing depth images.

        Args:
            msg: ROS Image message
        """
        if not self.calibrated:
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
            return

        # Verify depth accuracy at the last detected corners
        if len(self.corners_list) > 0:
            last_corners = self.corners_list[-1]
            error = self.verify_depth_accuracy(depth, last_corners)

            if error is not None:
                msg = Float64()
                msg.data = error
                self.depth_accuracy_pub.publish(msg)
                self.get_logger().info(f'Depth error: {error*1000:.2f}mm')

    def calibrate_callback(self, request, response: TriggerResponse) -> TriggerResponse:
        """
        Service callback to perform camera calibration.

        This function performs the actual camera calibration using the
        captured checkerboard images. It sets up 3D object points and
        2D image correspondences, then calls OpenCV's calibrateCamera.

        Args:
            request: Trigger request
            response: Trigger response to populate

        Returns:
            Response with success status and message
        """
        if len(self.captured_images) < self.min_images:
            response.success = False
            response.message = (
                f'Need at least {self.min_images} images, '
                f'but only have {len(self.captured_images)}'
            )
            self.get_logger().warn(response.message)
            return response

        self.get_logger().info(f'Starting calibration with {len(self.captured_images)} images...')

        # Prepare object points (3D coordinates in world space)
        # For a checkerboard, points are at (x*square_size, y*square_size, 0)
        obj_points = []
        img_points = []

        # Create 3D object points for checkerboard
        obj_pattern = np.zeros(
            ((self.checkerboard_width - 1) * (self.checkerboard_height - 1), 3),
            np.float32
        )
        obj_pattern[:, :2] = np.mgrid[
            0:self.checkerboard_width - 1,
            0:self.checkerboard_height - 1
        ].T.reshape(-1, 2) * self.square_size

        # Add points from all images
        for corners in self.corners_list:
            obj_points.append(obj_pattern)
            img_points.append(corners)

        # Perform camera calibration
        # Returns: camera_matrix, distortion_coeffs, rotation_vectors, translation_vectors
        ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj_points,
            img_points,
            self.captured_images[0].shape[:2][::-1],  # (width, height)
            None,
            None
        )

        # Calculate reprojection error (measure of calibration quality)
        total_error = 0
        for i in range(len(obj_points)):
            img_points_proj, _ = cv2.projectPoints(
                obj_points[i],
                rvecs[i],
                tvecs[i],
                camera_matrix,
                dist_coeffs
            )
            error = cv2.norm(img_points[i], img_points_proj, cv2.NORM_L2) / len(img_points_proj)
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

        # Publish calibration parameters
        self.publish_calibration()

        response.success = True
        response.message = (
            f'Calibration successful! Mean reprojection error: {mean_error:.4f} pixels'
        )
        self.get_logger().info(response.message)
        self.get_logger().info(f'Camera matrix:\n{camera_matrix}')
        self.get_logger().info(f'Distortion coefficients: {dist_coeffs.flatten()}')

        return response

    def verify_depth_accuracy(self, depth_image: np.ndarray,
                                corners_2d: np.ndarray) -> Optional[float]:
        """
        Verify depth accuracy by comparing measured vs expected distances.

        Args:
            depth_image: Depth map in meters (or millimeters depending on format)
            corners_2d: 2D corner coordinates in image (Nx2 array)

        Returns:
            Mean depth error in meters, or None if calculation fails
        """
        try:
            # Get depth values at corner positions
            depths = []
            for corner in corners_2d:
                x, y = int(corner[0]), int(corner[1])
                if 0 <= x < depth_image.shape[1] and 0 <= y < depth_image.shape[0]:
                    depth = depth_image[y, x]
                    if depth > 0 and not np.isnan(depth):
                        depths.append(depth)

            if len(depths) < 4:
                return None

            # Calculate mean measured depth
            measured_depth = np.mean(depths)

            # For a well-calibrated checkerboard, we expect depth to match
            # the actual distance from camera to checkerboard
            # In a real scenario, you'd know the true distance
            # For verification, we calculate consistency (std dev of depths)
            depth_std = np.std(depths)

            # Also report the measured depth range
            depth_range = np.max(depths) - np.min(depths)

            self.get_logger().debug(
                f'Depth stats: mean={measured_depth:.4f}m, '
                f'std={depth_std:.4f}m, range={depth_range:.4f}m'
            )

            # Return std as error metric (lower is better)
            return float(depth_std)

        except Exception as e:
            self.get_logger().error(f'Error in depth verification: {e}')
            return None

    def save_calibration_callback(self, request, response: TriggerResponse) -> TriggerResponse:
        """
        Service callback to save calibration results to file.

        Args:
            request: Trigger request
            response: Trigger response to populate

        Returns:
            Response with success status and message
        """
        if not self.calibrated:
            response.success = False
            response.message = 'No calibration to save. Run /calibrate first.'
            return response

        # Save to YAML file
        calib_file = os.path.expanduser('~/.ros/camera_calibration.yaml')

        try:
            with open(calib_file, 'w') as f:
                f.write('# Camera Calibration Results\n')
                f.write('# Generated by camera_calibration node\n\n')
                f.write('image_width: 640\n')
                f.write('image_height: 480\n\n')
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
                f.write(']\n\n')
                f.write(f'reprojection_error: {self.calibration_result["reprojection_error"]:.4f}\n')

            response.success = True
            response.message = f'Calibration saved to {calib_file}'

        except Exception as e:
            response.success = False
            response.message = f'Failed to save calibration: {e}'

        return response

    def publish_calibration(self) -> None:
        """Publish calibration parameters as JSON message."""
        if not self.calibrated:
            return

        msg = String()
        msg.data = json.dumps(self.calibration_result)
        self.calibration_pub.publish(msg)

        # Also publish CameraInfo
        self.publish_camera_info()

    def publish_camera_info(self) -> None:
        """Publish CameraInfo message with calibration parameters."""
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
        """Initialize the depth analyzer node."""
        super().__init__('depth_analyzer')

        self.declare_parameter('depth_topic', '/camera/depth/image_raw')
        self.declare_parameter('color_topic', '/camera/color/image_raw')

        self.depth_topic = self.get_parameter('depth_topic').get_parameter_value().string_value
        self.color_topic = self.get_parameter('color_topic').get_parameter_value().string_value

        self.bridge = CvBridge()
        self.latest_depth = None

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
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
            return

        # Filter out invalid values
        valid_depth = depth[(depth > 0) & (depth < 10) & ~np.isnan(depth)]

        if len(valid_depth) == 0:
            return

        # Calculate statistics
        stats = {
            'mean': float(np.mean(valid_depth)),
            'std': float(np.std(valid_depth)),
            'min': float(np.min(valid_depth)),
            'max': float(np.max(valid_depth)),
            'valid_pixels': int(np.sum((depth > 0) & (depth < 10) & ~np.isnan(depth)))
        }

        # Publish statistics
        msg = String()
        msg.data = json.dumps(stats)
        self.depth_stats_pub.publish(msg)


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
```

## Running the Lab

### Step 1: Verify Installation

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Check OpenCV installation
python3 -c "import cv2; print(f'OpenCV version: {cv2.__version__}')"

# Build package
cd ~/ros2_lab
colcon build --packages-select camera_calibration
source install/setup.bash
```

### Step 2: Prepare Checkerboard

Print a checkerboard pattern with:
- **9x6 squares** (8x5 internal corners) or **7x5 squares**
- Square size: **25mm** (adjust `square_size` parameter if different)
- Use high contrast (black/white)

### Step 3: Run Calibration

```bash
# Terminal 1: Start camera (or Gazebo simulation)
ros2 launch realsense2_camera d435i.launch.py

# Terminal 2: Run calibration node
ros2 run camera_calibration camera_calibrator

# Terminal 3: Capture images
# Move checkerboard to different positions and angles
# The node will auto-capture when corners are detected

# Terminal 4: Trigger calibration (after 10+ images)
ros2 service call /calibrate std_srvs/srv/Trigger

# Terminal 5: Save calibration
ros2 service call /save_calibration std_srvs/srv/Trigger
```

### Step 4: Verify Results

```bash
# Check published camera info
ros2 topic echo /camera_info

# Check calibration parameters
ros2 topic echo /calibration/parameters

# Check depth accuracy
ros2 topic echo /depth_accuracy
```

## Expected Output

```
[camera_calibrator] [INFO] Captured image 1/10
[camera_calibrator] [INFO] Captured image 2/10
...
[camera_calibrator] [INFO] Captured image 10/10
[camera_calibrator] [INFO] Starting calibration with 10 images...
[camera_calibrator] [INFO] Calibration successful! Mean reprojection error: 0.235 pixels
[camera_calibrator] [INFO] Camera matrix:
[[615.234 0.    320.456]
 [0.    615.678 240.123]
 [0.    0.    1.    ]]
[camera_calibrator] [INFO] Distortion coefficients: [-0.052 0.031 0.002 -0.001 0.000]
```

## Troubleshooting

### Issue: Checkerboard not detected

```bash
# Ensure good lighting
# Check checkerboard is fully in frame
# Try different distances (0.5m - 2m)
# Verify checkerboard dimensions match parameters
```

### Issue: High reprojection error

```bash
# Capture more images from various angles
# Include corner positions (not just center)
# Avoid reflections on checkerboard
# Check for motion blur
```

### Issue: Depth accuracy poor

```bash
# Verify depth camera is calibrated (factory calibration)
# Check for ambient light interference
# Ensure surfaces are not too reflective
# Verify depth image encoding (meters vs millimeters)
```

## Extensions and Challenges

### Challenge 1: Undistortion Node

Create a node that subscribes to raw images and publishes undistorted versions:

```python
def undistort_image(self, image: np.ndarray) -> np.ndarray:
    """Apply undistortion using calibration parameters."""
    h, w = image.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        self.camera_matrix,
        self.distortion_coeffs,
        (w, h),
        1,
        (w, h)
    )
    undistorted = cv2.undistort(image, self.camera_matrix,
                                 self.distortion_coeffs,
                                 None, new_camera_matrix)
    return undistorted
```

### Challenge 2: Depth-Camera-to-RGB Registration

Register depth points to RGB camera space:

```python
def register_depth_to_rgb(self, depth_image: np.ndarray,
                           depth_intrinsics: dict) -> np.ndarray:
    """Convert depth image to point cloud in RGB camera frame."""
    # Use depth_intrinsics (fx, fy, cx, cy) to create point cloud
    # Then transform to RGB camera frame
    pass
```

### Challenge 3: Multi-Camera Calibration

Extend to calibrate stereo camera rigs or RGB-D with IMU:

```python
def calibrate_stereo(self, left_images: List[np.ndarray],
                      right_images: List[np.ndarray]) -> dict:
    """Perform stereo camera calibration."""
    # Use cv2.stereoCalibrate for extrinsic calibration
    pass
```

## Evaluation Criteria

| Criteria | Points |
|----------|--------|
| Checkerboard detection | 15 |
| Image capture and storage | 10 |
| Camera calibration (intrinsics) | 25 |
| Reprojection error calculation | 15 |
| Depth accuracy verification | 15 |
| Calibration save/publish | 10 |
| Code quality and documentation | 10 |
| **Total** | **100** |

## References

- [OpenCV Camera Calibration Documentation](https://docs.opencv.org/master/dc/dbb/tutorial_py_calibration.html)
- [ROS Camera Calibration Documentation](https://wiki.ros.org/camera_calibration)
- [Intel RealSense Calibration Guide](https://github.com/IntelRealSense/librealsense/wiki/Calibration)
- [Camera Information Message](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
