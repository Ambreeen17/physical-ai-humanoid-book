---
sidebar_position: 2
---

# Chapter 1 Lab: Hello Physical AI

## Overview

In this hands-on lab, you'll build a ROS 2 node that implements a simple sensorimotor loop. You'll control a simulated 2-wheeled robot in Gazebo, commanding it to stay within bounds using feedback from its position sensor.

**Duration**: 30 minutes (including setup)
**Difficulty**: Beginner
**Requirements**: Docker, basic Python knowledge

---

## Learning Outcomes

By the end of this lab, you will:
1. Understand how ROS 2 pub/sub patterns work in practice
2. Implement a closed-loop control system that responds to sensor feedback
3. Deploy and test code in a simulated robotics environment
4. Observe the sensorimotor loop in action

---

## Architecture

```
┌──────────────────┐
│   turtlesim      │
│  (Simulator)     │
│                  │
│  /turtle1/pose   │──┐
│  (publishes)     │  │
└──────────────────┘  │
                      │
                      ▼
            ┌─────────────────┐
            │ Control Node    │
            │ (Your code)     │
            │                 │
            │ Subscribe to    │
            │ /turtle1/pose   │
            │                 │
            │ Publish to      │
            │ /turtle1/cmd_vel│
            └─────────────────┘
                      │
                      │
                      ▼
            ┌──────────────────┐
            │ Motor Driver     │
            │ (/turtle1/cmd_vel)
            │ (moves robot)    │
            └──────────────────┘
```

---

## Part 1: Setup (5 minutes)

### Step 1.1: Create Project Directory

```bash
mkdir -p ~/hello_physical_ai
cd ~/hello_physical_ai
```

### Step 1.2: Create Docker Compose File

Create `docker-compose.yml`:

```yaml
version: '3.8'

services:
  ros:
    image: osrf/ros:humble-desktop
    container_name: hello_physical_ai
    stdin_open: true
    tty: true
    environment:
      - DISPLAY=${DISPLAY}
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./src:/root/ros_ws/src
    working_dir: /root/ros_ws
    command: bash
```

### Step 1.3: Start Container

```bash
xhost +local:docker  # Allow Docker to access X11 display
docker-compose up -d
docker-compose exec ros bash
```

Inside the container:

```bash
# Install dependencies
apt-get update
apt-get install -y python3-pip python3-rosdep
pip install rclpy geometry-msgs turtlesim

# Source ROS 2 setup
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/ros_ws/src/hello_physical_ai
cd ~/ros_ws/src/hello_physical_ai
```

---

## Part 2: Write the Control Node (10 minutes)

Create `src/hello_physical_ai.py`:

```python
#!/usr/bin/env python3
"""
Hello Physical AI - Control Node
Implements a simple sensorimotor loop: monitor position, command velocity.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
import logging

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


class ControlNode(Node):
    """Simple feedback control: keep robot bounded within x < 2.0"""

    def __init__(self):
        super().__init__('hello_physical_ai')

        # Publishers and Subscribers
        self.velocity_publisher = self.create_publisher(
            Twist, '/turtle1/cmd_vel', 10
        )
        self.pose_subscriber = self.create_subscription(
            Pose, '/turtle1/pose', self.pose_callback, 10
        )

        # Control parameters
        self.x_max = 2.0
        self.forward_speed = 0.5
        self.backward_speed = -0.3
        self.current_x = 0.0

        self.get_logger().info("Control node initialized. Boundary: x < 2.0")

    def pose_callback(self, msg):
        """
        Callback when position update arrives.
        Decision logic: if robot passes x=2.0, command backward motion.
        """
        self.current_x = msg.x

        # Create velocity command
        twist = Twist()

        if self.current_x > self.x_max:
            # Robot past boundary, back up
            twist.linear.x = self.backward_speed
            self.get_logger().warn(
                f'Robot past boundary (x={self.current_x:.2f}), moving backward'
            )
        else:
            # Robot safe, move forward
            twist.linear.x = self.forward_speed
            self.get_logger().info(
                f'Robot position: x={self.current_x:.2f}, moving forward'
            )

        # Publish velocity command (closes the loop)
        self.velocity_publisher.publish(twist)


def main():
    rclpy.init()
    node = ControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down control node')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Understand the Code

1. **Publisher** (`/turtle1/cmd_vel`): Sends velocity commands to the robot
2. **Subscriber** (`/turtle1/pose`): Receives position feedback from the simulator
3. **Callback** (`pose_callback`): Runs every time a new pose arrives
4. **Control Logic**:
   - If `x > 2.0`, command backward motion
   - Otherwise, command forward motion

This is a **closed-loop control system**: the output (velocity) depends on the input (position).

---

## Part 3: Create Launch File (5 minutes)

Create `launch/hello_physical_ai.launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # turtlesim node
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim',
            output='screen'
        ),
        # Control node
        Node(
            package='hello_physical_ai',
            executable='hello_physical_ai_node',
            name='control_node',
            output='screen'
        )
    ])
```

---

## Part 4: Build and Run (10 minutes)

Inside the Docker container:

```bash
# Setup ROS 2 environment
cd ~/ros_ws
source /opt/ros/humble/setup.bash

# Build
colcon build --packages-select hello_physical_ai

# Source the build
source install/setup.bash

# Launch
ros2 launch hello_physical_ai hello_physical_ai.launch.py
```

---

## Expected Output

When the lab runs, you'll see:

```
[hello_physical_ai-1] [INFO] Control node initialized. Boundary: x < 2.0
[hello_physical_ai-1] [INFO] Robot position: x=0.00, moving forward
[hello_physical_ai-1] [INFO] Robot position: x=0.12, moving forward
[hello_physical_ai-1] [INFO] Robot position: x=0.24, moving forward
...
[hello_physical_ai-1] [INFO] Robot position: x=1.98, moving forward
[hello_physical_ai-1] [WARN] Robot past boundary (x=2.05), moving backward
[hello_physical_ai-1] [INFO] Robot position: x=1.95, moving forward
[hello_physical_ai-1] [INFO] Robot position: x=2.07, moving backward
[hello_physical_ai-1] [WARN] Robot past boundary (x=2.04), moving backward
```

The robot oscillates around the x=2.0 boundary, demonstrating the sensorimotor loop.

---

## Validation Criteria

✅ **Lab passes if:**
- Control node launches without errors
- Position updates arrive at least 10 times
- Velocity commands are published to `/turtle1/cmd_vel`
- Robot remains approximately bounded around x=2.0
- Log output matches expected format

Run this to validate:

```bash
# In a new terminal within the container
ros2 topic echo /turtle1/pose | head -20
```

You should see position updates with x values oscillating around 2.0.

---

## Challenges (Optional Extensions)

### Challenge 1: Add Rotation Control
Modify the control node to also command rotation when the robot rotates beyond theta = π/4.

**Hint**: Add `twist.angular.z = -0.5` when `msg.theta > pi/4`.

### Challenge 2: Implement PID Control
Replace the simple on/off control with a PID (proportional-integral-derivative) controller that smoothly regulates position.

**Hint**:
```python
error = self.x_max - self.current_x
twist.linear.x = self.Kp * error
```

### Challenge 3: Multi-Robot Coordination
Launch two turtles and have them maintain a fixed distance from each other using pub/sub.

---

## Troubleshooting

**Issue**: "Cannot connect to turtlesim"
- **Solution**: Ensure turtlesim service is running: `ros2 run turtlesim turtlesim_node`

**Issue**: Control node not receiving pose updates
- **Solution**: Check topic is published: `ros2 topic list` (should show `/turtle1/pose`)

**Issue**: X11 display not working in Docker
- **Solution**: Ensure `xhost +local:docker` was run on host, and `DISPLAY` env var is set

---

## Key Takeaways

1. **The sensorimotor loop is real**: Sensor data flows in, control decisions flow out, and the environment changes
2. **ROS 2 pub/sub is flexible**: The control node doesn't care *who* publishes positions or actuates motors
3. **Feedback is essential**: Without the pose subscription, the robot would blindly move forward forever
4. **Simulation is safe**: You can test unstable control laws (like oscillation) without damaging hardware

---

## Next Steps

- Advance to **Challenge 2** and implement smooth PID control
- Read Chapter 1, Section 1.4 for deeper ROS 2 concepts
- Explore the Gazebo simulator for more realistic physics
- Prepare for Chapter 2: Sensors & Perception

---

## Files Summary

```
hello_physical_ai/
├── docker-compose.yml         # Container setup
├── src/
│   └── hello_physical_ai.py    # Control node (main code)
├── launch/
│   └── hello_physical_ai.launch.py  # ROS 2 launch configuration
└── CMakeLists.txt             # Build configuration
```

All code is also available in the project repository.
