---
sidebar_position: 8
title: "Chapter 7: Motion Planning"
---

# Chapter 7: Motion Planning

<PersonalizationToggle chapterId="7" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Understand configuration space** and its role in motion planning
2. **Implement sampling-based planners** (RRT, RRT*, PRM)
3. **Apply optimization-based planning** (CHOMP, TrajOpt)
4. **Use real-time planners** for dynamic environments
5. **Integrate MoveIt 2** for robot arm planning

---

## Introduction

A robot knows where it is and where it wants to go. But the world is full of obstacles. How does it find a path that avoids collisions while satisfying joint limits, velocity constraints, and time requirements?

**Motion planning** answers this question. It's the algorithmic bridge between "I want to reach that point" and "here's exactly how to move my joints."

---

## Section 7.1: Configuration Space

### From Workspace to C-Space

**Workspace**: The 3D physical space where the robot operates
**Configuration Space (C-Space)**: The space of all possible robot configurations

For a 6-DOF arm, C-space is 6-dimensional. Each point represents a unique joint configuration.

```python
import numpy as np

class ConfigurationSpace:
    """
    Configuration space for a robot manipulator.
    """
    def __init__(self, robot, joint_limits):
        self.robot = robot
        self.joint_limits = joint_limits  # [(min, max), ...]
        self.dim = len(joint_limits)

    def sample_random(self):
        """Sample random configuration."""
        config = []
        for low, high in self.joint_limits:
            config.append(np.random.uniform(low, high))
        return np.array(config)

    def is_collision_free(self, config):
        """Check if configuration is collision-free."""
        # Check self-collision
        if self.robot.check_self_collision(config):
            return False
        # Check environment collision
        if self.robot.check_environment_collision(config):
            return False
        return True

    def distance(self, q1, q2):
        """Euclidean distance in C-space."""
        return np.linalg.norm(q1 - q2)
```

---

## Section 7.2: Sampling-Based Planning

### Rapidly-exploring Random Trees (RRT)

```python
class RRT:
    """
    Rapidly-exploring Random Tree for motion planning.
    """
    def __init__(self, c_space, step_size=0.1, max_iterations=10000):
        self.c_space = c_space
        self.step_size = step_size
        self.max_iterations = max_iterations

    def plan(self, start, goal, goal_threshold=0.1):
        """
        Find path from start to goal.
        """
        tree = {tuple(start): None}  # node -> parent
        nodes = [start]

        for _ in range(self.max_iterations):
            # Sample random configuration (with goal bias)
            if np.random.random() < 0.1:
                q_rand = goal
            else:
                q_rand = self.c_space.sample_random()

            # Find nearest node
            q_near = min(nodes, key=lambda q: self.c_space.distance(q, q_rand))

            # Extend toward random sample
            direction = q_rand - q_near
            distance = np.linalg.norm(direction)
            if distance > self.step_size:
                direction = direction / distance * self.step_size

            q_new = q_near + direction

            # Check collision
            if self.c_space.is_collision_free(q_new):
                tree[tuple(q_new)] = tuple(q_near)
                nodes.append(q_new)

                # Check if goal reached
                if self.c_space.distance(q_new, goal) < goal_threshold:
                    return self._extract_path(tree, q_new)

        return None  # No path found

    def _extract_path(self, tree, end):
        """Extract path from tree."""
        path = [end]
        current = tuple(end)
        while tree[current] is not None:
            current = tree[current]
            path.append(np.array(current))
        return list(reversed(path))
```

### RRT* (Optimal RRT)

```python
class RRTStar(RRT):
    """
    RRT* with rewiring for asymptotically optimal paths.
    """
    def __init__(self, c_space, step_size=0.1, max_iterations=10000, neighbor_radius=0.5):
        super().__init__(c_space, step_size, max_iterations)
        self.neighbor_radius = neighbor_radius
        self.costs = {}

    def plan(self, start, goal, goal_threshold=0.1):
        tree = {tuple(start): None}
        nodes = [start]
        self.costs[tuple(start)] = 0

        for iteration in range(self.max_iterations):
            q_rand = goal if np.random.random() < 0.1 else self.c_space.sample_random()
            q_near = min(nodes, key=lambda q: self.c_space.distance(q, q_rand))

            direction = q_rand - q_near
            distance = np.linalg.norm(direction)
            if distance > self.step_size:
                direction = direction / distance * self.step_size
            q_new = q_near + direction

            if not self.c_space.is_collision_free(q_new):
                continue

            # Find nearby nodes
            neighbors = [q for q in nodes
                        if self.c_space.distance(q, q_new) < self.neighbor_radius]

            # Choose best parent
            q_min = q_near
            cost_min = self.costs[tuple(q_near)] + self.c_space.distance(q_near, q_new)

            for q_neighbor in neighbors:
                cost = self.costs[tuple(q_neighbor)] + self.c_space.distance(q_neighbor, q_new)
                if cost < cost_min and self._collision_free_path(q_neighbor, q_new):
                    q_min = q_neighbor
                    cost_min = cost

            tree[tuple(q_new)] = tuple(q_min)
            nodes.append(q_new)
            self.costs[tuple(q_new)] = cost_min

            # Rewire tree
            for q_neighbor in neighbors:
                cost = cost_min + self.c_space.distance(q_new, q_neighbor)
                if cost < self.costs.get(tuple(q_neighbor), float('inf')):
                    if self._collision_free_path(q_new, q_neighbor):
                        tree[tuple(q_neighbor)] = tuple(q_new)
                        self.costs[tuple(q_neighbor)] = cost

            if self.c_space.distance(q_new, goal) < goal_threshold:
                return self._extract_path(tree, q_new)

        return None

    def _collision_free_path(self, q1, q2, resolution=10):
        """Check if straight-line path is collision-free."""
        for t in np.linspace(0, 1, resolution):
            q = q1 + t * (q2 - q1)
            if not self.c_space.is_collision_free(q):
                return False
        return True
```

---

## Section 7.3: Probabilistic Roadmaps (PRM)

```python
class PRM:
    """
    Probabilistic Roadmap for multi-query planning.
    """
    def __init__(self, c_space, n_samples=1000, k_neighbors=10):
        self.c_space = c_space
        self.n_samples = n_samples
        self.k_neighbors = k_neighbors
        self.nodes = []
        self.edges = {}

    def build_roadmap(self):
        """Construct roadmap offline."""
        # Sample configurations
        while len(self.nodes) < self.n_samples:
            q = self.c_space.sample_random()
            if self.c_space.is_collision_free(q):
                self.nodes.append(q)

        # Connect neighbors
        for i, q in enumerate(self.nodes):
            distances = [(j, self.c_space.distance(q, qj))
                        for j, qj in enumerate(self.nodes) if i != j]
            distances.sort(key=lambda x: x[1])

            neighbors = []
            for j, dist in distances[:self.k_neighbors]:
                if self._collision_free_path(q, self.nodes[j]):
                    neighbors.append((j, dist))
            self.edges[i] = neighbors

    def query(self, start, goal):
        """Find path between start and goal."""
        # Connect start and goal to roadmap
        start_neighbors = self._find_neighbors(start)
        goal_neighbors = self._find_neighbors(goal)

        if not start_neighbors or not goal_neighbors:
            return None

        # A* search on roadmap
        return self._astar_search(start, goal, start_neighbors, goal_neighbors)

    def _find_neighbors(self, q, k=5):
        """Find k nearest collision-free neighbors."""
        distances = [(i, self.c_space.distance(q, qi))
                    for i, qi in enumerate(self.nodes)]
        distances.sort(key=lambda x: x[1])

        neighbors = []
        for i, dist in distances[:k*2]:
            if self._collision_free_path(q, self.nodes[i]):
                neighbors.append((i, dist))
                if len(neighbors) >= k:
                    break
        return neighbors

    def _collision_free_path(self, q1, q2, resolution=10):
        for t in np.linspace(0, 1, resolution):
            q = q1 + t * (q2 - q1)
            if not self.c_space.is_collision_free(q):
                return False
        return True

    def _astar_search(self, start, goal, start_neighbors, goal_neighbors):
        """A* search on roadmap."""
        import heapq

        # Add virtual start and goal nodes
        open_set = []
        g_score = {-1: 0}  # -1 = start
        came_from = {}

        for idx, dist in start_neighbors:
            heapq.heappush(open_set, (dist + self._heuristic(self.nodes[idx], goal), idx))
            g_score[idx] = dist
            came_from[idx] = -1

        while open_set:
            _, current = heapq.heappop(open_set)

            # Check if we can reach goal
            for idx, dist in goal_neighbors:
                if idx == current:
                    path = self._reconstruct_path(came_from, current, start, goal)
                    return path

            for neighbor_idx, edge_cost in self.edges.get(current, []):
                tentative_g = g_score[current] + edge_cost

                if tentative_g < g_score.get(neighbor_idx, float('inf')):
                    came_from[neighbor_idx] = current
                    g_score[neighbor_idx] = tentative_g
                    f_score = tentative_g + self._heuristic(self.nodes[neighbor_idx], goal)
                    heapq.heappush(open_set, (f_score, neighbor_idx))

        return None

    def _heuristic(self, q, goal):
        return self.c_space.distance(q, goal)

    def _reconstruct_path(self, came_from, current, start, goal):
        path = [goal]
        while current != -1:
            path.append(self.nodes[current])
            current = came_from[current]
        path.append(start)
        return list(reversed(path))
```

---

## Section 7.4: Optimization-Based Planning

### CHOMP (Covariant Hamiltonian Optimization)

```python
class CHOMP:
    """
    Trajectory optimization using CHOMP.
    """
    def __init__(self, c_space, n_waypoints=50, learning_rate=0.01, max_iterations=100):
        self.c_space = c_space
        self.n_waypoints = n_waypoints
        self.learning_rate = learning_rate
        self.max_iterations = max_iterations

    def optimize(self, initial_trajectory, obstacles):
        """
        Optimize trajectory to minimize cost.
        """
        trajectory = np.array(initial_trajectory)

        for iteration in range(self.max_iterations):
            # Compute smoothness cost gradient
            smooth_grad = self._smoothness_gradient(trajectory)

            # Compute obstacle cost gradient
            obs_grad = self._obstacle_gradient(trajectory, obstacles)

            # Update trajectory (keep endpoints fixed)
            trajectory[1:-1] -= self.learning_rate * (smooth_grad[1:-1] + obs_grad[1:-1])

            if iteration % 20 == 0:
                cost = self._total_cost(trajectory, obstacles)
                print(f"Iteration {iteration}: cost = {cost:.4f}")

        return trajectory

    def _smoothness_gradient(self, trajectory):
        """Gradient of smoothness cost."""
        n = len(trajectory)
        grad = np.zeros_like(trajectory)

        # Second derivative approximation
        for i in range(1, n-1):
            grad[i] = 2 * trajectory[i] - trajectory[i-1] - trajectory[i+1]

        return grad

    def _obstacle_gradient(self, trajectory, obstacles):
        """Gradient of obstacle cost."""
        grad = np.zeros_like(trajectory)

        for i, q in enumerate(trajectory):
            for obs_center, obs_radius in obstacles:
                # Compute distance to obstacle in workspace
                ee_pos = self.c_space.robot.forward_kinematics(q)[:3]
                diff = ee_pos - obs_center
                dist = np.linalg.norm(diff)

                if dist < obs_radius * 2:  # Safety margin
                    # Repulsive gradient
                    jacobian = self.c_space.robot.jacobian(q)[:3, :]
                    workspace_grad = diff / (dist + 1e-6) * (1 / (dist - obs_radius + 1e-6))
                    grad[i] += jacobian.T @ workspace_grad

        return grad

    def _total_cost(self, trajectory, obstacles):
        smooth_cost = np.sum(np.diff(trajectory, axis=0)**2)
        obs_cost = 0
        for q in trajectory:
            for obs_center, obs_radius in obstacles:
                ee_pos = self.c_space.robot.forward_kinematics(q)[:3]
                dist = np.linalg.norm(ee_pos - obs_center)
                if dist < obs_radius * 2:
                    obs_cost += (obs_radius * 2 - dist)**2
        return smooth_cost + 10 * obs_cost
```

---

## Section 7.5: MoveIt 2 Integration

### Planning with MoveIt 2

```python
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import RobotState, Constraints
from moveit_msgs.srv import GetMotionPlan
from geometry_msgs.msg import PoseStamped

class MoveItPlanner(Node):
    """
    Motion planning using MoveIt 2.
    """
    def __init__(self):
        super().__init__('moveit_planner')

        self.plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path')

    def plan_to_pose(self, target_pose, planning_group='manipulator'):
        """
        Plan to target end-effector pose.
        """
        request = GetMotionPlan.Request()

        # Set planning group
        request.motion_plan_request.group_name = planning_group

        # Set goal constraints
        goal = Constraints()
        # Add position and orientation constraints...

        request.motion_plan_request.goal_constraints = [goal]
        request.motion_plan_request.num_planning_attempts = 10
        request.motion_plan_request.allowed_planning_time = 5.0

        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.motion_plan_response.error_code.val == 1:
            return response.motion_plan_response.trajectory
        return None
```

---

## Summary

### Key Takeaways

1. **Configuration space** transforms the planning problem to a searchable space
2. **RRT** efficiently explores high-dimensional spaces
3. **RRT*** provides asymptotically optimal paths
4. **PRM** is efficient for multi-query scenarios
5. **CHOMP** optimizes trajectories for smoothness and collision avoidance

### Looking Ahead

In Chapter 8, we'll explore **Manipulation & Grasping**—how to plan and execute robot grasps.

---

**Chapter completed**: 2026-01-20
**Chapter**: 7 - Motion Planning
