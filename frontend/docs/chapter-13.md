---
sidebar_position: 14
title: "Chapter 13: System Integration"
---

# Chapter 13: System Integration

<PersonalizationToggle chapterId="13" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Design a behavior tree** to orchestrate robot capabilities
2. **Implement an asynchronous state machine**
3. **Containerize your ROS 2 stack** with Docker
4. **Build a web interface** for robot teleoperation
5. **Debug distributed systems** effectively

---

## Introduction

We've built individual components: Perception (Ch 5), Control (Ch 6), Planning (Ch 7, 9), Learning (Ch 10-12).

Now, we must assemble them into a coherent **System**. A robot is not just a collection of algorithms; it's a real-time distributed system that must run reliably for hours.

---

## Section 13.1: Behavior Trees (BT)

State machines (FSMs) get spaghetti-like quickly. **Behavior Trees** are modular, reactive, and composable.

**Core Nodes:**
- **Sequence ($\rightarrow$)**: Run children in order; fail if one fails.
- **Selector ($?$)**: Run children in order; succeed if one succeeds (fallback).
- **Parallel ($\rightrightarrows$)**: Run children simultaneously.
- **Action**: Leaf node doing actual work.
- **Condition**: Leaf node checking state.

### Using `py_trees` with ROS 2

```python
import py_trees
import py_trees_ros
from py_trees.common import Status

class PickObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(PickObject, self).__init__(name)
        self.blackboard = py_trees.blackboard.Client(name="Picker")
        self.blackboard.register_key(key="target_pose", access=py_trees.common.Access.READ)

    def update(self):
        target = self.blackboard.target_pose
        if not target:
            return Status.FAILURE

        # Call pick action...
        return Status.RUNNING

def create_root():
    # Root: Priority (Selector)
    root = py_trees.composites.Selector(name="Main Loop", memory=False)

    # 1. Safety Check (High Priority)
    safety = py_trees.composites.Sequence(name="Safety Check", memory=False)
    chk_batt = CheckBattery(name="Battery OK?")
    chk_emg = CheckEstop(name="E-Stop Clear?")
    safety.add_children([chk_batt, chk_emg])

    # 2. Perform Task (Lower Priority)
    task = py_trees.composites.Sequence(name="Pick Task", memory=True)
    detect = DetectObject(name="Find Mug")
    pick = PickObject(name="Pick Mug")
    place = PlaceObject(name="Place Mug")
    task.add_children([detect, pick, place])

    root.add_children([safety, task])
    return root
```

---

## Section 13.2: Docker for Robotics

Robotics software is dependency hell (CUDA, ROS versions, Python libs). **Docker** solves this by packaging the entire OS and environment.

### Dockerfile for ROS 2 Humble + PyTorch

```dockerfile
# Base image with ROS 2 and CUDA
FROM osrf/ros:humble-desktop-full

# Install system dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-moveit \
    ros-humble-nav2-bringup \
    && rm -rf /var/lib/apt/lists/*

# Install Python ML libs
RUN pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
RUN pip3 install ultralytics  # For YOLO

# Setup workspace
WORKDIR /ws
COPY src/ ./src/
RUN . /opt/ros/humble/setup.sh && colcon build

# Source workspace in entrypoint
COPY entrypoint.sh /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "my_robot", "main.launch.py"]
```

### Docker Compose

Manage multi-container setups (e.g., Robot Stack + Web Server + Simulation).

```yaml
version: '3'
services:
  robot_stack:
    build: .
    network_mode: host
    privileged: true
    volumes:
      - /dev:/dev  # Access hardware
    environment:
      - DISPLAY=${DISPLAY}

  web_interface:
    image: nginx
    ports:
      - "8080:80"
```

---

## Section 13.3: Web Interface (Foxglove Studio)

Visualizing data is crucial. **Foxglove Studio** is the modern standard for robotics visualization via WebSocket.

### Rosbridge Server

To talk to the web, we need a bridge.

```python
# Launch rosbridge
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}]
        )
    ])
```

Now you can connect Foxglove Studio to `ws://localhost:9090` to see camera feeds, TF frames, and point clouds in the browser.

---

## Section 13.4: Data Recording (Rosbag)

Always record data. When the robot fails, you need to replay what happened.

```bash
# Record all topics
ros2 bag record -a

# Record specific topics with compression
ros2 bag record -o my_test_data /camera/image_raw /joint_states --compression-mode file --compression-format zstd
```

### Data Replay

```bash
ros2 bag play my_test_data
```

---

## Section 13.5: Latency & Synchronization

In a distributed system, messages arrive at different times.
- Camera: 30Hz
- LiDAR: 10Hz
- Odometry: 100Hz

Don't process just the "latest" message; use **Time Synchronization**.

```python
import message_filters

# Synchronize Image and CameraInfo
image_sub = message_filters.Subscriber(node, Image, '/camera/rgb')
info_sub = message_filters.Subscriber(node, CameraInfo, '/camera/info')

ts = message_filters.ApproximateTimeSynchronizer([image_sub, info_sub], 10, 0.1)
ts.registerCallback(callback)

def callback(image, info):
    # image and info are from approximately the same time
    pass
```

---

## Summary

### Key Takeaways

1. **Behavior Trees** provide a scalable way to orchestrate complex robot logic.
2. **Docker** ensures reproducibility and solves dependency hell.
3. **Foxglove** enables powerful web-based visualization and debugging.
4. **Time Synchronization** is critical for multi-sensor fusion.
5. **System Integration** is about gluing components together reliably.

### Looking Ahead

In Chapter 14, we'll discuss **Safety & Robustness**—ensuring that when our code crashes (and it will), the robot doesn't hurt anyone or itself.

---

**Chapter completed**: 2026-01-20
**Chapter**: 13 - System Integration
