#!/usr/bin/env python3
"""
Lab 3.1 Solution: LiDAR Point Cloud Processing Node

This node subscribes to point cloud data, applies voxel grid filtering,
and uses RANSAC to detect ground planes.

Topics:
    Subscribed: /camera/points (sensor_msgs/PointCloud2)
    Published:  /obstacles (sensor_msgs/PointCloud2)
                /ground_plane (sensor_msgs/PointCloud2)
                /debug/filtered (sensor_msgs/PointCloud2)

Parameters:
    voxel_size (float): Size of each voxel in meters (default: 0.05)
    ransac_threshold (float): Distance threshold for RANSAC inliers (default: 0.01)
    ransac_max_iterations (int): Maximum RANSAC iterations (default: 1000)
    max_z_filter (float): Maximum Z value to keep (default: 2.0)
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
        self.get_logger().info(f'Voxel size: {self.voxel_size}m, RANSAC threshold: {self.ransac_threshold}m')

    def pointcloud_callback(self, msg: PointCloud2) -> None:
        """
        Callback for processing incoming point cloud messages.

        Args:
            msg: PointCloud2 message from LiDAR sensor
        """
        self.get_logger().debug(f'Received point cloud: {msg.width * msg.height} points')

        # Step 1: Convert ROS PointCloud2 to NumPy array
        points = self.pointcloud2_to_numpy(msg)

        if points is None or len(points) == 0:
            self.get_logger().warn('Empty point cloud received')
            return

        self.get_logger().info(f'Raw points: {len(points)}')

        # Step 2: Apply voxel grid filtering
        filtered_points = self.voxel_grid_filter(points, self.voxel_size)

        self.get_logger().info(f'Filtered points: {len(filtered_points)}')

        # Step 3: RANSAC plane detection
        ground_coefficients, ground_indices, obstacle_indices = self.ransac_plane_detection(
            filtered_points,
            self.ransac_threshold,
            self.ransac_max_iterations
        )

        if ground_coefficients is None:
            self.get_logger().warn('No plane detected')
            return

        # Log plane coefficients (ax + by + cz + d = 0)
        self.get_logger().info(
            f'Ground plane: a={ground_coefficients[0]:.4f}, b={ground_coefficients[1]:.4f}, '
            f'c={ground_coefficients[2]:.4f}, d={ground_coefficients[3]:.4f}'
        )
        self.get_logger().info(f'Ground points: {len(ground_indices)}, Obstacles: {len(obstacle_indices)}')

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

        This function parses the binary PointCloud2 data and extracts x, y, z
        coordinates into a NumPy array.

        Args:
            msg: ROS PointCloud2 message

        Returns:
            NumPy array of shape (num_points, 3) with x, y, z coordinates,
            or None if conversion fails
        """
        try:
            # Check that we have at least x, y, z fields
            if len(msg.fields) < 3:
                self.get_logger().error('PointCloud2 has fewer than 3 fields')
                return None

            # Find offsets for x, y, z fields
            field_offsets = {}
            for field in msg.fields:
                if field.name in ['x', 'y', 'z']:
                    field_offsets[field.name] = field.offset

            if len(field_offsets) != 3:
                self.get_logger().error('Could not find x, y, z fields in PointCloud2')
                return None

            # Parse the data field as float32 array
            # PointCloud2 stores data as a byte array, we need to interpret it
            points = np.frombuffer(msg.data, dtype=np.uint8).astype(np.float32)

            # Reshape to extract individual points with all fields
            # msg.point_step is the number of bytes per point
            points = points.reshape(-1, msg.point_step // 4)

            # Extract x, y, z columns using field offsets (converted to float32 indices)
            x_idx = int(field_offsets['x'] / 4)
            y_idx = int(field_offsets['y'] / 4)
            z_idx = int(field_offsets['z'] / 4)

            result = np.column_stack([
                points[:, x_idx],
                points[:, y_idx],
                points[:, z_idx]
            ])

            return result

        except Exception as e:
            self.get_logger().error(f'Error converting point cloud: {e}')
            return None

    def voxel_grid_filter(self, points: np.ndarray, voxel_size: float) -> np.ndarray:
        """
        Apply voxel grid downsampling to point cloud using Open3D.

        A voxel grid divides 3D space into small cubic volumes. For each voxel,
        we compute the centroid of all points within it, resulting in a
        significantly downsampled point cloud while preserving spatial structure.

        Args:
            points: NumPy array of shape (N, 3) containing x, y, z coordinates
            voxel_size: Size of each voxel in meters

        Returns:
            Downsampled point cloud as NumPy array
        """
        if len(points) == 0:
            return points

        # Create Open3D point cloud from NumPy array
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Apply voxel grid downsampling
        downpcd = pcd.voxel_down_sample(voxel_size)

        # Convert back to NumPy
        return np.asarray(downpcd.points)

    def ransac_plane_detection(
        self,
        points: np.ndarray,
        threshold: float,
        max_iterations: int
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        """
        Detect ground plane using RANSAC algorithm.

        RANSAC (Random Sample Consensus) is a robust estimation algorithm that
        handles outliers by iteratively fitting models to random subsets of data.

        The algorithm:
        1. Randomly samples 3 points (minimum for plane definition)
        2. Fits plane equation: ax + by + cz + d = 0
        3. Counts inliers (points within threshold distance)
        4. Repeats for N iterations, returning model with most inliers

        Args:
            points: NumPy array of shape (N, 3) containing x, y, z coordinates
            threshold: Distance threshold for inlier classification
            max_iterations: Maximum number of RANSAC iterations

        Returns:
            Tuple of (plane_coefficients, ground_indices, obstacle_indices)
            - plane_coefficients: np.ndarray [a, b, c, d] for plane ax+by+cz+d=0
            - ground_indices: Indices of points belonging to ground plane
            - obstacle_indices: Indices of non-ground points
        """
        if len(points) < 3:
            self.get_logger().warn('Not enough points for plane detection')
            return None, None, None

        # Create Open3D point cloud
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Segment plane using Open3D's RANSAC implementation
        # This is more efficient than a pure Python implementation
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=threshold,
            ransac_n=3,
            num_iterations=max_iterations
        )

        # plane_model = [a, b, c, d] where ax + by + cz + d = 0
        # Normalize for better numerical stability
        norm = np.linalg.norm(plane_model[:3])
        if norm > 0:
            plane_model = plane_model / norm

        # Convert inlier indices to numpy arrays
        ground_indices = np.array(inliers)

        # Get obstacle indices (all points not in inliers)
        all_indices = set(range(len(points)))
        inlier_set = set(inliers)
        obstacle_indices = np.array(list(all_indices - inlier_set))

        return np.array(plane_model), ground_indices, obstacle_indices

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
            PointCloud2 message ready for publishing
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
