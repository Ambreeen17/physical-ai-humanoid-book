#!/usr/bin/env python3
"""
Lab 3.1 Starter: LiDAR Point Cloud Processing Node

This node subscribes to point cloud data, applies voxel grid filtering,
and uses RANSAC to detect ground planes.

Starter Code - Complete the TODO sections!

Topics:
    Subscribed: /camera/points (sensor_msgs/PointCloud2)
    Published:  /obstacles (sensor_msgs/PointCloud2)
                /ground_plane (sensor_msgs/PointCloud2)
                /debug/filtered (sensor_msgs/PointCloud2)

Parameters:
    voxel_size (float): Size of each voxel in meters (default: 0.05)
    ransac_threshold (float): Distance threshold for RANSAC inliers (default: 0.01)
    ransac_max_iterations (int): Maximum RANSAC iterations (default: 1000)
    max_z_filter (float): Maximum Z value to keep (remove ceiling) (default: 2.0)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
from typing import Tuple, Optional
import open3d as o3d


class LiDARProcessor(Node):
    """Node for processing LiDAR point clouds with filtering and plane detection."""

    def __init__(self) -> None:
        """Initialize the LiDAR processor node."""
        super().__init__('lidar_processor')

        # Declare parameters with defaults
        self.declare_parameter('voxel_size', 0.05)
        self.declare_parameter('ransac_threshold', 0.01)
        self.declare_parameter('ransac_max_iterations', 1000)
        self.declare_parameter('max_z_filter', 2.0)
        self.declare_parameter('input_topic', '/camera/points')

        # Get parameter values
        self.voxel_size = self.get_parameter('voxel_size').get_parameter_value().double_value
        self.ransac_threshold = self.get_parameter('ransac_threshold').get_parameter_value().double_value
        self.ransac_max_iterations = self.get_parameter('ransac_max_iterations').get_parameter_value().integer_value
        self.max_z_filter = self.get_parameter('max_z_filter').get_parameter_value().double_value
        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value

        # Create subscriber
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.pointcloud_callback,
            10
        )
        self.subscription  # Prevent unused variable warning

        # Create publishers
        self.obstacle_pub = self.create_publisher(
            PointCloud2,
            '/obstacles',
            10
        )
        self.ground_pub = self.create_publisher(
            PointCloud2,
            '/ground_plane',
            10
        )
        self.debug_pub = self.create_publisher(
            PointCloud2,
            '/debug/filtered',
            10
        )

        self.get_logger().info(f'LiDAR Processor initialized')
        self.get_logger().info(f'Input topic: {self.input_topic}')
        self.get_logger().info(f'Voxel size: {self.voxel_size}m')

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        """
        Callback for processing incoming point cloud messages.

        Args:
            msg: PointCloud2 message from LiDAR sensor
        """
        self.get_logger().debug(f'Received point cloud: {msg.width * msg.height} points')

        # Step 1: Convert ROS PointCloud2 to NumPy array
        # TODO: Implement pointcloud2_to_numpy conversion
        points = self.pointcloud2_to_numpy(msg)

        if points is None or len(points) == 0:
            self.get_logger().warn('Empty point cloud received')
            return

        self.get_logger().info(f'Raw points: {len(points)}')

        # Step 2: Apply voxel grid filtering
        # TODO: Implement voxel_grid_filter
        filtered_points = self.voxel_grid_filter(points, self.voxel_size)

        self.get_logger().info(f'Filtered points: {len(filtered_points)}')

        # Step 3: RANSAC plane detection
        # TODO: Implement ransac_plane_detection
        ground_coefficients, ground_indices, obstacle_indices = self.ransac_plane_detection(
            filtered_points,
            self.ransac_threshold,
            self.ransac_max_iterations
        )

        if ground_coefficients is None:
            self.get_logger().warn('No plane detected')
            return

        # Log plane coefficients (ax + by + cz + d = 0)
        self.get_logger().info(f'Ground plane: {ground_coefficients}')

        # Step 4: Publish results
        ground_cloud = self.create_pointcloud2(filtered_points[ground_indices], msg.header)
        obstacle_cloud = self.create_pointcloud2(filtered_points[obstacle_indices], msg.header)

        self.ground_pub.publish(ground_cloud)
        self.obstacle_pub.publish(obstacle_cloud)

        # Debug: publish filtered cloud
        debug_cloud = self.create_pointcloud2(filtered_points, msg.header)
        self.debug_pub.publish(debug_cloud)

    def pointcloud2_to_numpy(self, msg: PointCloud2) -> Optional[np.ndarray]:
        """
        Convert a PointCloud2 message to a NumPy array of (N, 3) shape.

        Args:
            msg: ROS PointCloud2 message

        Returns:
            NumPy array of shape (num_points, 3) with x, y, z coordinates,
            or None if conversion fails

        TODO: Implement this function
        Hint: Use numpy.frombuffer to parse the data field
        Extract x, y, z fields (typically fields[0], fields[1], fields[2])
        Return points as np.ndarray with dtype=np.float64
        """
        # YOUR CODE HERE
        pass

    def voxel_grid_filter(self, points: np.ndarray, voxel_size: float) -> np.ndarray:
        """
        Apply voxel grid downsampling to point cloud.

        Args:
            points: NumPy array of shape (N, 3) containing x, y, z coordinates
            voxel_size: Size of each voxel in meters

        Returns:
            Downsampled point cloud as NumPy array

        TODO: Implement this function
        Strategy options:
        1. Use Open3D: o3d.geometry.VoxelGrid.create_from_point_cloud()
        2. Manual implementation: bin points into voxels and compute centroids
        """
        # YOUR CODE HERE
        pass

    def ransac_plane_detection(
        self,
        points: np.ndarray,
        threshold: float,
        max_iterations: int
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Detect ground plane using RANSAC algorithm.

        Args:
            points: NumPy array of shape (N, 3) containing x, y, z coordinates
            threshold: Distance threshold for inlier classification
            max_iterations: Maximum number of RANSAC iterations

        Returns:
            Tuple of (plane_coefficients, ground_indices, obstacle_indices)
            - plane_coefficients: np.ndarray [a, b, c, d] for plane ax+by+cz+d=0
            - ground_indices: Indices of points belonging to ground plane
            - obstacle_indices: Indices of non-ground points

        TODO: Implement RANSAC plane detection
        Algorithm:
        1. For each iteration:
           - Randomly select 3 points
           - Calculate plane equation from these points
           - Count inliers (points within threshold distance)
        2. Return model with most inliers
        3. Classify all points as ground or obstacle
        """
        # YOUR CODE HERE
        pass

    def create_pointcloud2(
        self,
        points: np.ndarray,
        header: Header
    ) -> PointCloud2:
        """
        Create a PointCloud2 message from NumPy array.

        Args:
            points: NumPy array of shape (N, 3)
            header: ROS header to use

        Returns:
            PointCloud2 message
        """
        msg = PointCloud2()
        msg.header = header
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.is_dense = True
        msg.point_step = 12  # 3 floats * 4 bytes each
        msg.row_step = msg.point_step * msg.width
        msg.data = points.astype(np.float32).tobytes()
        return msg


def main(args=None) -> None:
    """Main entry point for the LiDAR processor node."""
    rclpy.init(args=args)
    node = LiDARProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down LiDAR processor...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
