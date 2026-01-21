---
sidebar_position: 15
title: "Chapter 14: Safety & Robustness"
---

# Chapter 14: Safety & Robustness

<PersonalizationToggle chapterId="14" />

## Learning Objectives

By the end of this chapter, you will be able to:
1. **Implement safety barriers** using Control Barrier Functions (CBF)
2. **Handle system faults** with watchdogs and heartbeats
3. **Design redundancy** for critical components
4. **Comply with safety standards** (ISO 10218, ISO 13482)
5. **Detect anomalies** in sensor data

---

## Introduction

Robots are dangerous. They have powerful motors, heavy link, and operate near humans.

Safety is not an add-on; it must be baked into the architecture. In this chapter, we explore how to guarantee safety even when software bugs or hardware failures occur.

---

## Section 14.1: Control Barrier Functions (CBF)

How do we ensure a robot *never* hits an obstacle, even if the RL policy wants to?

**Control Barrier Functions (CBF)** act as a safety filter. They modify the controller's output minimaly to satisfy safety constraints.

Constraint: $h(x) \ge 0$ (e.g., distance to wall > 0)
Safety Condition: $\dot{h}(x) + \alpha h(x) \ge 0$

### QP-Based Safety Filter

We solve a Quadratic Program (QP) at 1000Hz:

$$
\min_{u} \| u - u_{des} \|^2
$$
$$
s.t. \quad \nabla h(x)^T (f(x) + g(x)u) + \alpha h(x) \ge 0
$$

```python
import numpy as np
from cvxopt import matrix, solvers

class SafetyFilter:
    def __init__(self, alpha=1.0):
        self.alpha = alpha

    def filter_control(self, x, u_des, obstacle_pos, safe_dist):
        """
        Modify u_des to ensure safety.
        Model: x_dot = u (single integrator)
        Barrier: h(x) = ||x - obs||^2 - dist^2 >= 0
        """
        # 1. Compute Barrier h(x)
        diff = x - obstacle_pos
        dist_sq = np.dot(diff, diff)
        h = dist_sq - safe_dist**2

        # 2. Compute Lie Derivative Lgh
        # grad_h = 2 * (x - obs)
        grad_h = 2 * diff

        # Constraint: grad_h * u >= -alpha * h
        # -grad_h * u <= alpha * h

        # 3. Setup QP
        P = matrix(np.eye(len(x)))
        q_vec = matrix(-u_des)
        G = matrix(-grad_h.reshape(1, -1))
        h_vec = matrix(self.alpha * h)

        # 4. Solve
        solvers.options['show_progress'] = False
        sol = solvers.qp(P, q_vec, G, h_vec)

        if sol['status'] == 'optimal':
            return np.array(sol['x']).flatten()
        else:
            print("Infeasible! Emergency Stop.")
            return np.zeros_like(u_des)
```

---

## Section 14.2: Watchdogs & Heartbeats

Software crashes. Threads hang.

A **Watchdog** expects a periodic "heartbeat" signal from a node. If the heartbeat stops, the watchdog triggers a safety state (E-Stop).

### ROS 2 Watchdog Node

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Bool

class Watchdog(Node):
    def __init__(self, timeout=0.1):
        super().__init__('safety_watchdog')
        self.timeout = timeout
        self.last_heartbeat = self.get_clock().now()

        # Input: Heartbeat
        self.create_subscription(Empty, 'heartbeat', self.on_heartbeat, 10)

        # Output: Enable signal
        self.enable_pub = self.create_publisher(Bool, 'robot_enable', 10)

        # Check timer
        self.create_timer(0.05, self.check_safety)

    def on_heartbeat(self, msg):
        self.last_heartbeat = self.get_clock().now()

    def check_safety(self):
        time_since = (self.get_clock().now() - self.last_heartbeat).nanoseconds / 1e9

        msg = Bool()
        if time_since > self.timeout:
            self.get_logger().error(f"Watchdog Timeout! {time_since:.3f}s")
            msg.data = False # Disable robot
        else:
            msg.data = True  # Enable robot

        self.enable_pub.publish(msg)
```

---

## Section 14.3: Anomaly Detection

Sensors can fail in subtle ways (drift, freeze, noise). We can use statistical methods or autoencoders to detect this.

### Autoencoder for Fault Detection

Train an autoencoder on "normal" sensor data. If reconstruction error is high, something is wrong.

```python
import torch.nn as nn

class SensorMonitor(nn.Module):
    def __init__(self, input_dim):
        super().__init__()
        self.encoder = nn.Sequential(
            nn.Linear(input_dim, 32), nn.ReLU(),
            nn.Linear(32, 8)
        )
        self.decoder = nn.Sequential(
            nn.Linear(8, 32), nn.ReLU(),
            nn.Linear(32, input_dim)
        )

    def forward(self, x):
        latent = self.encoder(x)
        return self.decoder(latent)

    def is_anomaly(self, x, threshold=0.1):
        reconst = self.forward(x)
        error = torch.mean((x - reconst)**2)
        return error > threshold
```

---

## Section 14.4: Multiplexing Control (Twist Mux)

Multiple nodes want to move the robot:
- Teleop (Joystick)
- Nav2 (Navigation)
- Emergency Stop

We use a **Twist Mux** to prioritize commands.

```yaml
# twitch_mux.yaml
topics:
  - name: teleop
    topic: cmd_vel_teleop
    timeout: 0.5
    priority: 100
  - name: navigation
    topic: cmd_vel_nav
    timeout: 0.5
    priority: 10
  - name: safety_stop
    topic: cmd_vel_stop
    timeout: 0.1
    priority: 255  # Highest
```

If `cmd_vel_stop` publishes (usually all zeros), it overrides everything else.

---

## Section 14.5: Safety Standards

Designing safe robots isn't just good practice; it's the law.

1.  **ISO 10218-1/2**: Safety requirements for industrial robots.
    *   Key concept: "Collaborative Operation" modes (Safety-rated monitored stop, Hand guiding, Speed and separation monitoring, Power and force limiting).

2.  **ISO 13482**: Safety for personal care robots.
    *   Focuses on lower energy impacts and "friendly" failure modes.

3.  **ISO 26262**: Functional safety for automotive (often applied to AMRs).
    *   ASIL (Automotive Safety Integrity Level) classification.

---

## Summary

### Key Takeaways

1. **Safety Filters (CBF)** force controllers to respect constraints mathematically.
2. **Watchdogs** catch software freezes and disconnects.
3. **Muxing** ensures high-priority safety signals override navigation commands.
4. **Redundancy** (hardware and software) is key for critical systems.
5. **Standards** provide a checklist for verifiable safety.

### Looking Ahead

In Chapter 15, we'll cover **Deployment & Operations**—how to manage fleets of robots, handle OTA updates, and monitor health in the cloud.

---

**Chapter completed**: 2026-01-20
**Chapter**: 14 - Safety & Robustness
