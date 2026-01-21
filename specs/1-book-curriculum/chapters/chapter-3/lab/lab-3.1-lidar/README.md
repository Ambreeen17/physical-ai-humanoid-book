# Lab 3.1: LiDAR Point Cloud Processing

**Duration:** 45 minutes
**ROS 2 Version:** Humble Hawksbill (Ubuntu 22.04)
**Topic:** Point cloud filtering and plane detection with RANSAC

## Learning Objectives

By the end of this lab, you will be able to:

1. **Subscribe to point cloud topics** (`/camera/points`) and understand point cloud data structure
2. **Implement voxel grid filtering** to downsample 3D point clouds for efficient processing
3. **Apply RANSAC plane detection** to segment ground planes from LiDAR data
4. **Calculate ground plane coefficients** and extract non-ground points (obstacles)
5. **Visualize results** using RViz2 and publish filtered point clouds

## Prerequisites

- ROS 2 Humble installed and sourced (`source /opt/ros/humble/setup.bash`)
- Basic understanding of NumPy and Open3D or PCL
- Completion of Lab 2.x (publisher/subscriber basics)

## Estimated Duration

- Setup and theory: 10 minutes
- Voxel filter implementation: 12 minutes
- RANSAC plane detection: 15 minutes
- Testing and visualization: 8 minutes

## Architecture Overview

```
/camera/points (sensor_msgs/PointCloud2)
        |
        v
+------------------+
|  Voxel Filter    |  ---> Downsample point cloud
+------------------+
        |
        v
+------------------+
|  RANSAC Plane    |  ---> Detect ground plane
|    Detection     |
+------------------+
        |
        v
    [Two outputs]
        |           |
        v           v
  Ground Plane   Non-Ground Points
  Coefficients   (/obstacles topic)
```

## Background Theory

### Point Cloud Data Structure

Point clouds in ROS 2 use the `sensor_msgs/PointCloud2` message type. This contains:
- `height` and `width`: dimensions of the point cloud
- `fields`: list of PointField structures describing each field (x, y, z, intensity, etc.)
- `is_bigendian`, `is_dense`: data format flags
- `data`: raw byte array containing point data

### Voxel Grid Filtering

A voxel grid divides 3D space into small cubic volumes (voxels). For each voxel, we compute a representative point (typically the centroid of all points in that voxel). This dramatically reduces point count while preserving spatial structure.

**Benefits:**
- Reduces computation from O(n) to O(v) where v = number of voxels
- Makes subsequent processing (RANSAC, clustering) feasible in real-time
- Removes outliers while preserving object shape

### RANSAC Plane Detection

RANSAC (Random Sample Consensus) is a robust estimation algorithm that:
1. Randomly samples minimum sets of points (3 for a plane)
2. Fits a model (plane equation: ax + by + cz + d = 0)
3. Counts inliers (points within threshold distance)
4. Repeats for N iterations and returns the model with most inliers

**Why RANSAC for ground detection?**
- Handles outliers (obstacles, noise) naturally
- Works without perfect sensor calibration
- Robust to partial occlusions

## Dependencies

```bash
# Install required packages
sudo apt-get update
sudo apt-get install -y \
    ros-humble-pcl-ros \
    ros-humble-rviz2 \
    python3-numpy \
    python3-open3d

# For development
pip3 install numpy open3d
```

## Package Setup

```bash
# Create workspace if needed
mkdir -p ~/ros2_lab/src
cd ~/ros2_lab
colcon build --packages-select sensor_processing

# Source workspace
source install/setup.bash
```

## Exercise: Implement Point Cloud Processing Node

### File Structure

```
sensor_processing/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/
│   └── sensor_processing
├── sensor_processing/
│   ├── __init__.py
│   ├── lidar_processor.py    # Your solution
│   └── lidar_processor_starter.py  # Starter code
├── launch/
│   └── lidar_demo.launch.py
└── worlds/
    └── simple_world.sdf
```

### Starter Code

Copy the following starter code into `sensor_processing/sensor_processing/lidar_processor_starter.py`:

```python
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
from geometry_msgs.msg import PoseArray, Pose
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
        self.max_z_filter = self.get_parameter('max_z_filter').getParameter_value().double_value
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
        """
        # TODO: Implement this function
        # Hint: Use numpy.frombuffer to parse the data field
        # Extract x, y, z fields (typically fields[0], fields[1], fields[2])
        # Return points as np.ndarray with dtype=np.float64

        pass  # Replace with your implementation

    def voxel_grid_filter(self, points: np.ndarray, voxel_size: float) -> np.ndarray:
        """
        Apply voxel grid downsampling to point cloud.

        Args:
            points: NumPy array of shape (N, 3) containing x, y, z coordinates
            voxel_size: Size of each voxel in meters

        Returns:
            Downsampled point cloud as NumPy array
        """
        # TODO: Implement this function
        # Strategy options:
        # 1. Use Open3D: o3d.geometry.VoxelGrid.create_from_point_cloud()
        # 2. Manual implementation: bin points into voxels and compute centroids

        pass  # Replace with your implementation

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
        """
        # TODO: Implement RANSAC plane detection

        # Algorithm:
        # 1. For each iteration:
        #    - Randomly select 3 points
        #    - Calculate plane equation from these points
        #    - Count inliers (points within threshold distance)
        # 2. Return model with most inliers
        # 3. Classify all points as ground or obstacle

        pass  # Replace with your implementation

    def create_pointcloud2(
        self,
        points: np.ndarray,
        header
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
```

### Solution Code

Here's the complete solution in `sensor_processing/sensor_processing/lidar_processor.py`:

```python
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
import random


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
        self.subscription

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
            points = np.frombuffer(msg.data, dtype=np.uint8).astype(np.float32)

            # Reshape and extract x, y, z
            points = points.reshape(-1, msg.point_step // 4)

            # Extract x, y, z columns
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

        Args:
            points: NumPy array of shape (N, 3) containing x, y, z coordinates
            voxel_size: Size of each voxel in meters

        Returns:
            Downsampled point cloud as NumPy array
        """
        if len(points) == 0:
            return points

        # Create Open3D point cloud
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

        # Using Open3D's RANSAC implementation (more efficient)
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)

        # Segment plane using RANSAC
        plane_model, inliers = pcd.segment_plane(
            distance_threshold=threshold,
            ransac_n=3,
            num_iterations=max_iterations
        )

        # plane_model = [a, b, c, d] where ax + by + cz + d = 0
        ground_indices = np.array(inliers)
        obstacle_indices = np.array([i for i in range(len(points)) if i not in inliers])

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
```

## Launch File

Create `sensor_processing/launch/lidar_demo.launch.py`:

```python
#!/usr/bin/env python3
"""
Launch file for LiDAR point cloud processing demo.

This launch file starts:
1. Gazebo simulation with a simple world
2. LiDAR sensor plugin
3. LiDAR processor node
4. RViz2 for visualization
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description() -> LaunchDescription:
    """Generate the launch description for the LiDAR demo."""
    ld = LaunchDescription()

    # Get package directories
    pkg_share = get_package_share_directory('sensor_processing')
    gazebo_pkg_share = get_package_share_directory('gazebo_ros')

    # Launch Gazebo with simple world
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_pkg_share, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': os.path.join(pkg_share, 'worlds', 'simple_world.sdf')}.items()
    )

    # LiDAR processor node
    lidar_processor = Node(
        package='sensor_processing',
        executable='lidar_processor',
        name='lidar_processor',
        output='screen',
        parameters=[
            {'voxel_size': 0.05},
            {'ransac_threshold': 0.01},
            {'ransac_max_iterations': 1000},
            {'input_topic': '/camera/points'}
        ]
    )

    # RViz2 configuration
    rviz_config = os.path.join(pkg_share, 'config', 'lidar_rviz.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )

    ld.add_action(gazebo)
    ld.add_action(lidar_processor)
    ld.add_action(rviz)

    return ld
```

## Gazebo World File

Create `sensor_processing/worlds/simple_world.sdf`:

```xml
<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="simple_lidar_world">
    <physics name="default_physics" default="true" type="ode">
      <gravity>0 0 -9.8</gravity>
      <ode>
        <solver>
          <type>quick</type>
          <iters>10</iters>
          <sor>1.0</sor>
        </solver>
      </ode>
    </physics>

    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <surface>
            <friction>
              <ode>
                <mu>0.5</mu>
                <mu2>0.5</mu2>
              </ode>
            </friction>
          </surface>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Grey</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Obstacles for testing -->
    <model name="box_obstacle_1">
      <pose>2 0 0.5 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>1 1 1</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Red</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <model name="cylinder_obstacle_2">
      <pose>-1.5 1.5 0.75 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <height>1.5</height>
            </cylinder>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.5</radius>
              <height>1.5</height>
            </cylinder>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Blue</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Simple robot with LiDAR -->
    <model name="robot">
      <pose>0 0 0.1 0 0 0</pose>
      <link name="base_link">
        <collision name="collision">
          <geometry>
            <box>
              <size>0.5 0.5 0.2</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>0.5 0.5 0.2</size>
            </box>
          </geometry>
          <material>
            <script>
              <name>Gazebo/Yellow</name>
            </script>
          </material>
        </visual>
        <sensor name="lidar" type="ray">
          <pose>0 0 0.1 0 0 0</pose>
          <ray>
            <scan>
              <horizontal>
                <samples>360</samples>
                <resolution>1</resolution>
                <min_angle>-3.14159</min_angle>
                <max_angle>3.14159</max_angle>
              </horizontal>
            </scan>
            <range>
              <min>0.1</min>
              <max>10.0</max>
              <resolution>0.01</resolution>
            </range>
          </ray>
          <plugin name="lidar_plugin" filename="libgazebo_ros_ray_sensor.so">
            <ros>
              <namespace>/camera</namespace>
              <remapping>points:=points</remapping>
            </ros>
            <output_type>sensor_msgs/PointCloud2</output_type>
            <frame_name>base_link</frame_name>
          </plugin>
        </sensor>
      </link>
    </model>

    <!-- Ambient lighting -->
    <light name="sun" type="directional">
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8</diffuse>
      <specular>0.2 0.2 0.2</specular>
    </light>
  </world>
</sdf>
```

## Running the Lab

### Step 1: Verify Installation

```bash
# Source ROS 2
source /opt/ros/humble/setup.bash

# Check Open3D installation
python3 -c "import open3d; print(f'Open3D version: {open3d.__version__}')"

# Build package
cd ~/ros2_lab
colcon build --packages-select sensor_processing
source install/setup.bash
```

### Step 2: Run the Demo

```bash
# Terminal 1: Start Gazebo simulation
source /opt/ros/humble/setup.bash
cd ~/ros2_lab
source install/setup.bash
ros2 launch sensor_processing lidar_demo.launch.py

# Terminal 2: Run the processor node
source /opt/ros/humble/setup.bash
cd ~/ros2_lab
source install/setup.bash
ros2 run sensor_processing lidar_processor

# Terminal 3: Monitor topics
ros2 topic list
ros2 topic echo /obstacles
ros2 topic echo /ground_plane
```

### Step 3: Verify in RViz2

1. RViz2 should open automatically or manually:
   ```bash
   rviz2 -d ~/ros2_lab/src/sensor_processing/config/lidar_rviz.rviz
   ```

2. Add displays for:
   - `/camera/points` (raw point cloud, set color to Z)
   - `/obstacles` (red color)
   - `/ground_plane` (green color)

3. Verify:
   - Ground plane is detected (green points on floor)
   - Obstacles are separated (red points on boxes/cylinders)
   - Point counts are reduced after filtering

### Step 4: Check Node and Topics

```bash
# Verify node is running
ros2 node list
# Output: /lidar_processor

# Verify topics
ros2 topic list
# Should see: /camera/points, /obstacles, /ground_plane, /debug/filtered

# Check node info
ros2 node info /lidar_processor
```

## Expected Output

When running successfully, you should see:

```
[lidar_processor] [INFO] LiDAR Processor initialized
[lidar_processor] [INFO] Input topic: /camera/points
[lidar_processor] [INFO] Raw points: 14400
[lidar_processor] [INFO] Filtered points: 2456
[lidar_processor] [INFO] Ground plane: a=0.0021, b=0.0052, c=0.9999, d=-0.1001
[lidar_processor] [INFO] Ground points: 1892, Obstacles: 564
```

## Troubleshooting

### Issue: No point cloud data

```bash
# Check if Gazebo is publishing
ros2 topic hz /camera/points
# Should show > 1 Hz

# Check for errors in Gazebo
# Ensure gazebo_ros packages are installed
sudo apt-get install ros-humble-gazebo-ros-pkgs
```

### Issue: RANSAC fails to find plane

- Increase `ransac_threshold` (try 0.02-0.05)
- Ensure ground plane is visible (not too many obstacles)
- Check that point cloud contains ground points (Z values near 0)

### Issue: Open3D import fails

```bash
# Install Open3D with pip
pip3 install open3d

# Verify installation
python3 -c "import open3d; print(open3d.__version__)"
```

### Issue: PointCloud2 conversion errors

- Verify field names are 'x', 'y', 'z' (case-sensitive)
- Check that data is not compressed
- Ensure point_step matches actual data layout

## Extensions and Challenges

### Challenge 1: Real-time Performance Optimization

Implement a faster voxel filter without Open3D:

```python
def voxel_grid_filter_manual(self, points: np.ndarray, voxel_size: float) -> np.ndarray:
    """Manual voxel grid implementation without Open3D."""
    # Quantize points to voxel grid indices
    voxel_indices = np.floor(points / voxel_size).astype(int)

    # Find unique voxels and compute centroids
    # ...
```

### Challenge 2: Adaptive RANSAC

Modify the algorithm to dynamically adjust parameters:

- Increase iterations if inlier ratio is low
- Decrease threshold if plane detection fails
- Add minimum inlier ratio check

### Challenge 3: Multi-plane Segmentation

Extend to detect multiple planes (walls, tables):

```python
def multi_plane_segmentation(self, points: np.ndarray, num_planes: int = 3) -> list:
    """Segment multiple planes from point cloud."""
    planes = []
    remaining_points = points.copy()

    for _ in range(num_planes):
        # Detect next plane from remaining points
        # Remove inliers and repeat
        pass

    return planes
```

### Challenge 4: Ground Height Estimation

Calculate the actual ground height (not assuming Z=0):

```python
def estimate_ground_height(self, ground_points: np.ndarray) -> float:
    """Estimate the ground plane height from detected ground points."""
    # Use plane coefficients to find height at origin
    # Or compute mean Z of ground points
    pass
```

## Evaluation Criteria

| Criteria | Points |
|----------|--------|
| PointCloud2 to NumPy conversion | 15 |
| Voxel grid filtering | 20 |
| RANSAC plane detection | 30 |
| Topic publishing (obstacles/ground) | 15 |
| Correct plane coefficients output | 10 |
| Code quality and documentation | 10 |
| **Total** | **100** |

## References

- [ROS 2 PointCloud2 Message Documentation](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html)
- [Open3D Point Cloud Documentation](http://www.open3d.org/docs/release/)
- [RANSAC Algorithm](https://en.wikipedia.org/wiki/Random_sample_consensus)
- [Gazebo Ray Sensor Plugin](https://gazebosim.org/docs/harmonic/ros2_integration)
