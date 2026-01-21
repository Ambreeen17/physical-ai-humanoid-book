# Lab 1: Hello Physical AI: Your First Sensorimotor Loop

## Overview
In this lab, you will implement the most fundamental pattern in robotics: the **Sensorimotor Loop**. You will use ROS 2 (Humble) to create a control system that reads position data from a simulated robot (Sense), processes that information (Think), and sends movement commands back to the robot (Act).

**Learning Objectives:**
* Understand the concept of a "node" in ROS 2.
* Implement a Subscriber to read sensor data.
* Implement a Publisher to send actuator commands.
* Observe a closed-loop feedback system in simulation.

**Time Estimate:** 30 minutes

---

## Architectural Diagram
```text
      +-----------------------------------------+
      |             ROS 2 Network               |
      |                                         |
      |  +----------+      /turtle1/pose      +------------+
      |  |          | ----------------------> |            |
      |  |  Robot   |       (Sensor)          |   Your     |
      |  | (Sim)    |                         |  Node      |
      |  |          | <---------------------- | (Control)  |
      |  +----------+     /turtle1/cmd_vel    +------------+
      |                    (Actuator)           |
      +-----------------------------------------+
```

## Prerequisites
- Docker and Docker Compose installed
- Basic knowledge of Python

## Getting Started

### 1. Setup the Environment
Run the following command to start the simulation environment:
```bash
make up
```
This will launch a Gazebo simulation container with a simple robot.

### 2. Implementation: The Sensorimotor Node
Open `src/hello_physical_ai_node.py`. You will find a template for a ROS 2 node. Your task is to:
1. Subscribe to the `/turtle1/pose` topic.
2. Calculate a control signal based on the robot's X position.
3. Publish velocity commands to `/turtle1/cmd_vel`.

### 3. Running the Lab
Launch your node alongside the simulation:
```bash
make run
```

### 4. Verification
Watch the robot in the Gazebo window (accessible via VNC/Browser if configured, or console output).
The robot should:
- Move forward initially.
- Automatically reverse once it crosses X = 2.0.

## Troubleshooting
- **No data appearing?** Check if the simulation is running with `ros2 topic list`.
- **Robot not moving?** Verify your publisher topic name matches `/turtle1/cmd_vel`.

---
🤖 Generated with [Claude Code](https://claude.com/claude-code)
