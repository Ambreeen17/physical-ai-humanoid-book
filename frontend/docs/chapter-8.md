---
sidebar_position: 9
title: "Chapter 8: Manipulation & Grasping"
---

# Chapter 8: Manipulation & Grasping

<PersonalizationToggle chapterId="8" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Understand grasp theory** including force closure and form closure
2. **Implement grasp planning algorithms** for parallel-jaw and multi-finger grippers
3. **Apply force control** for delicate manipulation tasks
4. **Use learning-based grasping** with neural networks
5. **Integrate manipulation pipelines** in ROS 2

---

## Introduction

A robot can navigate a room and see objects on a table. But can it pick up a coffee mug without breaking it? Can it insert a key into a lock? Can it hand you a tool?

**Manipulation**—the ability to physically interact with and change the world—is what separates robots from cameras on wheels. It's also one of the hardest problems in robotics.

---

## Section 8.1: Grasp Theory Fundamentals

### What Makes a Good Grasp?

A grasp must satisfy two conditions:
1. **Force Closure**: Can resist external forces in any direction
2. **Form Closure**: Geometry alone constrains the object

```python
import numpy as np

class GraspAnalyzer:
    """
    Analyze grasp quality using force closure.
    """
    def __init__(self):
        self.friction_coefficient = 0.5

    def compute_grasp_wrench_space(self, contact_points, contact_normals):
        """
        Compute the wrench space spanned by contact forces.

        Each contact can apply force within friction cone.
        """
        wrenches = []

        for point, normal in zip(contact_points, contact_normals):
            # Friction cone discretization
            for angle in np.linspace(0, 2*np.pi, 8):
                # Force direction within friction cone
                tangent1 = np.cross(normal, [0, 0, 1])
                if np.linalg.norm(tangent1) < 0.01:
                    tangent1 = np.cross(normal, [0, 1, 0])
                tangent1 /= np.linalg.norm(tangent1)
                tangent2 = np.cross(normal, tangent1)

                force = (normal +
                        self.friction_coefficient * np.cos(angle) * tangent1 +
                        self.friction_coefficient * np.sin(angle) * tangent2)
                force /= np.linalg.norm(force)

                # Compute wrench (force and torque about origin)
                torque = np.cross(point, force)
                wrench = np.concatenate([force, torque])
                wrenches.append(wrench)

        return np.array(wrenches)

    def check_force_closure(self, wrenches):
        """
        Check if wrench space contains the origin.
        """
        from scipy.spatial import ConvexHull

        try:
            hull = ConvexHull(wrenches)
            # Check if origin is inside convex hull
            # Simplified check: can we achieve zero wrench?
            return True  # More rigorous check needed
        except:
            return False

    def compute_grasp_quality(self, contact_points, contact_normals):
        """
        Compute grasp quality metric (largest inscribed ball radius).
        """
        wrenches = self.compute_grasp_wrench_space(contact_points, contact_normals)

        # Quality = distance from origin to convex hull boundary
        # Higher is better
        if len(wrenches) < 6:
            return 0.0

        # Simplified: use minimum singular value
        U, S, V = np.linalg.svd(wrenches.T)
        return np.min(S)
```

### Antipodal Grasps

For parallel-jaw grippers, **antipodal grasps** are optimal:

```python
def find_antipodal_grasps(point_cloud, normals, gripper_width=0.08):
    """
    Find antipodal grasp candidates on a point cloud.
    """
    grasps = []
    n_points = len(point_cloud)

    for i in range(n_points):
        for j in range(i+1, n_points):
            p1, n1 = point_cloud[i], normals[i]
            p2, n2 = point_cloud[j], normals[j]

            # Check distance constraint
            dist = np.linalg.norm(p1 - p2)
            if dist > gripper_width:
                continue

            # Check antipodal condition
            # Normals should be roughly opposite
            if np.dot(n1, n2) > -0.9:
                continue

            # Grasp axis should align with line between points
            axis = (p2 - p1) / dist
            if abs(np.dot(axis, n1)) < 0.8:
                continue

            # Valid antipodal grasp
            center = (p1 + p2) / 2
            grasps.append({
                'center': center,
                'axis': axis,
                'width': dist,
                'points': (p1, p2)
            })

    return grasps
```

---

## Section 8.2: Grasp Planning

### GraspIt! Style Planning

```python
class GraspPlanner:
    """
    Grasp planning using sampling and ranking.
    """
    def __init__(self, gripper, object_mesh):
        self.gripper = gripper
        self.object_mesh = object_mesh

    def sample_grasps(self, n_samples=1000):
        """
        Sample grasp candidates on object surface.
        """
        candidates = []

        for _ in range(n_samples):
            # Sample point on object surface
            point, normal = self.object_mesh.sample_surface()

            # Sample approach direction within cone
            approach = self._sample_approach(normal)

            # Sample wrist rotation
            rotation = np.random.uniform(0, 2*np.pi)

            # Create grasp pose
            grasp_pose = self._compute_grasp_pose(point, approach, rotation)

            # Check collision
            if not self._check_collision(grasp_pose):
                candidates.append(grasp_pose)

        return candidates

    def rank_grasps(self, candidates):
        """
        Rank grasps by quality metrics.
        """
        ranked = []

        for grasp in candidates:
            # Simulate closing gripper
            contacts = self._simulate_close(grasp)

            if len(contacts) < 2:
                continue

            # Compute quality
            quality = self._compute_quality(contacts)
            ranked.append((grasp, quality))

        # Sort by quality (descending)
        ranked.sort(key=lambda x: x[1], reverse=True)
        return ranked

    def _sample_approach(self, normal, cone_angle=np.pi/6):
        """Sample approach direction within cone around normal."""
        # Random rotation within cone
        theta = np.random.uniform(0, cone_angle)
        phi = np.random.uniform(0, 2*np.pi)

        # Perturb normal
        perturb = np.array([
            np.sin(theta) * np.cos(phi),
            np.sin(theta) * np.sin(phi),
            np.cos(theta)
        ])

        # Rotate to align with normal
        # ... (rotation matrix computation)
        return -normal  # Simplified: approach along normal
```

---

## Section 8.3: Force Control

### Impedance Control for Manipulation

```python
class ManipulationController:
    """
    Impedance controller for contact-rich manipulation.
    """
    def __init__(self, robot, stiffness, damping):
        self.robot = robot
        self.K = np.diag(stiffness)  # 6x6 stiffness
        self.D = np.diag(damping)    # 6x6 damping

    def compute_torques(self, x_des, x_dot_des, x, x_dot, F_ext=None):
        """
        Compute joint torques for impedance behavior.
        """
        # Position error in task space
        error = x_des - x
        error_dot = x_dot_des - x_dot

        # Desired task-space force
        F_task = self.K @ error + self.D @ error_dot

        if F_ext is not None:
            F_task += F_ext  # Compensate for external forces

        # Map to joint torques via Jacobian transpose
        J = self.robot.get_jacobian()
        tau = J.T @ F_task

        # Add gravity compensation
        tau += self.robot.gravity_compensation()

        return tau

class HybridForcePositionController:
    """
    Hybrid force/position control for constrained tasks.
    """
    def __init__(self, robot):
        self.robot = robot
        self.S = np.eye(6)  # Selection matrix (1=position, 0=force)

    def set_constraint(self, force_axes):
        """
        Set which axes are force-controlled.
        force_axes: list of axis indices (0-5) for force control
        """
        self.S = np.eye(6)
        for axis in force_axes:
            self.S[axis, axis] = 0

    def compute(self, x_des, F_des, x, x_dot, F_meas):
        """
        Compute control for hybrid force/position.
        """
        # Position control on position-controlled axes
        Kp = 100 * np.eye(6)
        Kd = 20 * np.eye(6)

        pos_error = x_des - x
        F_pos = self.S @ (Kp @ pos_error - Kd @ x_dot)

        # Force control on force-controlled axes
        Kf = 0.001 * np.eye(6)
        force_error = F_des - F_meas
        F_force = (np.eye(6) - self.S) @ (Kf @ force_error)

        F_total = F_pos + F_force

        J = self.robot.get_jacobian()
        tau = J.T @ F_total + self.robot.gravity_compensation()

        return tau
```

---

## Section 8.4: Learning-Based Grasping

### Grasp Detection with Neural Networks

```python
import torch
import torch.nn as nn

class GraspNet(nn.Module):
    """
    Neural network for grasp detection from depth images.
    Outputs grasp rectangle parameters.
    """
    def __init__(self):
        super().__init__()

        # Encoder
        self.encoder = nn.Sequential(
            nn.Conv2d(1, 32, 5, stride=2, padding=2),
            nn.ReLU(),
            nn.Conv2d(32, 64, 5, stride=2, padding=2),
            nn.ReLU(),
            nn.Conv2d(64, 128, 3, stride=2, padding=1),
            nn.ReLU(),
        )

        # Grasp quality head
        self.quality_head = nn.Sequential(
            nn.Conv2d(128, 64, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 1, 1),
            nn.Sigmoid()
        )

        # Grasp angle head
        self.angle_head = nn.Sequential(
            nn.Conv2d(128, 64, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 2, 1)  # cos, sin of angle
        )

        # Grasp width head
        self.width_head = nn.Sequential(
            nn.Conv2d(128, 64, 3, padding=1),
            nn.ReLU(),
            nn.Conv2d(64, 1, 1),
            nn.Sigmoid()
        )

    def forward(self, depth_image):
        """
        Forward pass.

        Returns:
            quality: Grasp quality heatmap
            angle: Grasp angle (cos, sin)
            width: Grasp width
        """
        features = self.encoder(depth_image)

        quality = self.quality_head(features)
        angle = self.angle_head(features)
        width = self.width_head(features)

        return quality, angle, width

    def predict_grasps(self, depth_image, n_grasps=5):
        """
        Predict top N grasps from depth image.
        """
        with torch.no_grad():
            quality, angle, width = self.forward(depth_image)

        # Find peaks in quality map
        quality_np = quality.squeeze().cpu().numpy()

        # Non-maximum suppression
        grasps = []
        for _ in range(n_grasps):
            idx = np.unravel_index(np.argmax(quality_np), quality_np.shape)

            if quality_np[idx] < 0.5:
                break

            # Extract grasp parameters
            y, x = idx
            cos_a = angle[0, 0, y, x].item()
            sin_a = angle[0, 1, y, x].item()
            theta = np.arctan2(sin_a, cos_a)
            w = width[0, 0, y, x].item() * 0.1  # Scale to meters

            grasps.append({
                'center': (x, y),
                'angle': theta,
                'width': w,
                'quality': quality_np[idx]
            })

            # Suppress nearby peaks
            quality_np[max(0,y-10):y+10, max(0,x-10):x+10] = 0

        return grasps
```

---

## Section 8.5: ROS 2 Manipulation Pipeline

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup, Grasp
from sensor_msgs.msg import PointCloud2

class ManipulationNode(Node):
    """
    Complete manipulation pipeline in ROS 2.
    """
    def __init__(self):
        super().__init__('manipulation_node')

        # Perception subscriber
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points',
            self.pointcloud_callback, 10)

        # MoveIt action client
        self.move_group_client = ActionClient(self, MoveGroup, 'move_action')

        # Gripper action client
        self.gripper_client = ActionClient(self, GripperCommand, 'gripper_action')

        # Grasp planner
        self.grasp_net = GraspNet()
        self.grasp_net.load_state_dict(torch.load('grasp_model.pth'))

    def pick_object(self, object_pose):
        """
        Execute pick operation.
        """
        # 1. Plan grasps
        grasps = self.plan_grasps(object_pose)

        if not grasps:
            self.get_logger().error('No valid grasps found')
            return False

        # 2. Move to pre-grasp pose
        pre_grasp = self.compute_pre_grasp(grasps[0])
        self.move_to_pose(pre_grasp)

        # 3. Open gripper
        self.set_gripper(0.08)  # Open to 8cm

        # 4. Approach (linear motion)
        self.linear_move(grasps[0].pose)

        # 5. Close gripper
        self.set_gripper(grasps[0].width)

        # 6. Lift
        lift_pose = grasps[0].pose
        lift_pose.position.z += 0.1
        self.linear_move(lift_pose)

        return True

    def place_object(self, place_pose):
        """
        Execute place operation.
        """
        # 1. Move above place location
        approach_pose = place_pose
        approach_pose.position.z += 0.1
        self.move_to_pose(approach_pose)

        # 2. Lower to place
        self.linear_move(place_pose)

        # 3. Open gripper
        self.set_gripper(0.08)

        # 4. Retreat
        self.linear_move(approach_pose)

        return True
```

---

## Summary

### Key Takeaways

1. **Force closure** ensures grasps can resist external forces
2. **Antipodal grasps** are optimal for parallel-jaw grippers
3. **Impedance control** enables compliant manipulation
4. **Learning-based methods** can predict grasps from visual input
5. **Complete pipelines** combine perception, planning, and execution

### Looking Ahead

In Chapter 9, we'll explore **Task & Motion Planning**—how to sequence manipulation actions to achieve complex goals.

---

**Chapter completed**: 2026-01-20
**Chapter**: 8 - Manipulation & Grasping
