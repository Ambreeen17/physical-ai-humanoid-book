---
sidebar_position: 16
title: "Chapter 15: Deployment & Operations"
---

# Chapter 15: Deployment & Operations

<PersonalizationToggle chapterId="15" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Manage fleet deployments** with OTA updates
2. **Implement cloud logging** and telemetry
3. **Set up CI/CD pipelines** for robotics
4. **Optimize onboard compute** (Jetson/Orin)
5. **Handle network connectivity** challenges

---

## Introduction

It works on your laptop. Great. Now make it work on 100 robots, in a warehouse with bad Wi-Fi, for 3 years without crashing.

**Robotics Operations (RoboOps)** is the bridge between a research prototype and a commercial product.

---

## Section 15.1: Packaging & OTA Updates

You can't `git pull` on 100 robots manually. You need **Over-The-Air (OTA)** updates.

### Building Debian Packages

ROS 2 packages can be built as `.deb` files.

```bash
# Install bloom
sudo apt install python3-bloom fakeroot

# Initialize
bloom-generate rosdebian --os-name ubuntu --os-version jammy --ros-distro humble

# Build
fakeroot debian/rules binary
```

Now you have `ros-humble-my-robot_1.0.0_amd64.deb`.

### OTA Update Strategies

1.  **A/B Partitioning**:
    *   System has two partitions: Active (A) and Standby (B).
    *   Update is written to B.
    *   Reboot into B. If it fails, fallback to A automatically.
    *   *Tools: Mender, ostree*

2.  **Container Swapping**:
    *   Pull new Docker image.
    *   Stop old container, start new one.
    *   *Tools: Watchtower, Balena*

---

## Section 15.2: CI/CD for Robotics

Robots need rigorous testing before deployment. A bad update can physically break hardware.

### GitHub Actions Workflow

```yaml
name: Robot CI

on: [push, pull_request]

jobs:
  build-and-test:
    runs-on: ubuntu-22.04
    container: osrf/ros:humble-desktop
    steps:
      - uses: actions/checkout@v3

      - name: Install dependencies
        run: |
          apt-get update
          rosdep update
          rosdep install --from-paths src --ignore-src -y

      - name: Build
        run: colcon build

      - name: Unit Tests
        run: colcon test

      - name: Integration Test (Simulation)
        run: |
          # Headless Gazebo test
          timeout 60s ros2 launch my_robot_tests simulation_test.launch.py
```

---

## Section 15.3: Telemetry & Cloud Logging

Robots generate TBs of data. You can't upload everything.

**Strategy:**
1.  **Metrics (Low Cost)**: CPU, Battery, Error Rates. Upload continuously.
2.  **Events (Medium Cost)**: Text logs, critical warnings. Upload standard (ELK Stack).
3.  **Blackbox (High Cost)**: Rosbags of raw sensor data. **Only upload on trigger** (e.g., collision, e-stop).

### Cloud Upload Script

```python
import boto3
import os
from watchdog.events import FileSystemEventHandler

class CrashUploader(FileSystemEventHandler):
    def on_created(self, event):
        if event.src_path.endswith(".bag"):
            print(f"New crash bag detected: {event.src_path}")
            self.upload_to_s3(event.src_path)

    def upload_to_s3(self, filepath):
        s3 = boto3.client('s3')
        filename = os.path.basename(filepath)
        s3.upload_file(filepath, "my-robot-logs", filename)
```

---

## Section 15.4: Edge Optimization

Robots have limited compute/power. Running heavy models requires optimization.

### TensorRT Optimization

NVIDIA TensorRT can speed up inference by 10x on Jetson/Orin.

```python
import torch
from torch2trt import torch2trt

# 1. Load PyTorch model
model = ResNet50().cuda().eval()

# 2. Convert to TensorRT
data = torch.randn((1, 3, 224, 224)).cuda()
model_trt = torch2trt(model, [data], fp16_mode=True)

# 3. Save
torch.save(model_trt.state_dict(), 'resnet_trt.pth')

# 4. Inference
output = model_trt(data)
```

---

## Section 15.5: Network & Connectivity

Robots move. Wi-Fi has dead zones. 5G has latency.

### VPN & NAT Traversal

Robots are behind NATs (Firewalls). To SSH into them remotely, you need a VPN.
*   **Tailscale / WireGuard**: Best modern options. Zero-config mesh VPN.
*   **Husarnet**: Peer-to-peer VPN localized for ROS.

### QoS (Quality of Service)

Configure ROS 2 DDS to handle packet loss ("Best Effort" vs "Reliable").

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy

# Reliable (TCP-like) - Good for params/services
reliable_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

# Best Effort (UDP-like) - Good for video/LiDAR
sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

pub = node.create_publisher(Image, 'camera', sensor_qos)
```

---

## Summary

### Key Takeaways

1.  **CI/CD** prevents bad code from reaching robots.
2.  **OTA Updates** must be atomic and reversible (A/B partitions).
3.  **Selective Telemetry** saves bandwidth and storage costs.
4.  **Edge Optimization (TensorRT)** is essential for real-time AI.
5.  **VPNs** enable secure remote access to fleet devices.

### Looking Ahead

In Chapter 16, we'll conclude with the **Capstone Project**—building a complete, autonomous humanoid system from scratch using everything we've learned.

---

**Chapter completed**: 2026-01-20
**Chapter**: 15 - Deployment & Operations
