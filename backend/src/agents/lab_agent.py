"""Robotics Lab Generator Agent - Creates lab exercises for the chapter."""

import logging
from typing import Dict, Any
import asyncio
from datetime import datetime
import os

from src.agents.protocol import AgentInterface, AgentInput, AgentOutput, AgentStatus
import logging

logger = logging.getLogger(__name__)


class LabAgent(AgentInterface):
    """Lab agent that creates robotics lab exercises for the chapter."""

    def __init__(self):
        super().__init__("Lab")
        self.logger = logger

    async def execute(self, agent_input: AgentInput) -> AgentOutput:
        """Execute lab generation for the chapter.

        Args:
            agent_input: Chapter metadata including number, topic, and previous artifacts

        Returns:
            AgentOutput with lab artifacts
        """
        self.logger.info(f"Starting lab generation for Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}")

        try:
            # Generate lab content
            lab_dir = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/lab"
            
            # Create lab directory structure
            os.makedirs(lab_dir, exist_ok=True)
            
            # Create 3 lab exercises
            lab_names = [
                f"lab_{agent_input.chapter_number}_1",
                f"lab_{agent_input.chapter_number}_2", 
                f"lab_{agent_input.chapter_number}_3"
            ]
            
            lab_descriptions = [
                f"Introduction to {agent_input.chapter_topic} - Basic Concepts",
                f"Intermediate {agent_input.chapter_topic} - Implementation",
                f"Advanced {agent_input.chapter_topic} - Integration and Optimization"
            ]
            
            for i, (lab_name, lab_desc) in enumerate(zip(lab_names, lab_descriptions), 1):
                self._create_lab_exercise(lab_dir, lab_name, lab_desc, agent_input)
            
            # Create main lab README
            self._create_lab_readme(lab_dir, agent_input)
            
            artifacts = {
                "lab/README.md": os.path.join(lab_dir, "README.md"),
                f"lab/{lab_names[0]}/": os.path.join(lab_dir, lab_names[0]),
                f"lab/{lab_names[1]}/": os.path.join(lab_dir, lab_names[1]),
                f"lab/{lab_names[2]}/": os.path.join(lab_dir, lab_names[2]),
            }
            
            self.logger.info(f"Lab generation completed for Chapter {agent_input.chapter_number}")
            return AgentOutput(
                status=AgentStatus.SUCCESS,
                artifacts=artifacts,
                duration_seconds=20  # Simulated duration
            )
            
        except Exception as e:
            self.logger.error(f"Lab agent failed: {str(e)}")
            return AgentOutput(
                status=AgentStatus.FAILURE,
                artifacts={},
                error_message=str(e)
            )

    def _create_lab_exercise(self, lab_dir: str, lab_name: str, lab_desc: str, agent_input: AgentInput):
        """Create a single lab exercise with all necessary files."""
        # Create lab directory
        lab_exercise_dir = os.path.join(lab_dir, lab_name)
        os.makedirs(lab_exercise_dir, exist_ok=True)
        
        # Create Dockerfile
        dockerfile_content = f"""# Dockerfile for {lab_name}
FROM osrf/ros:humble-desktop

# Install dependencies
RUN apt-get update && apt-get install -y \\
    python3-pip \\
    python3-colcon-common-extensions \\
    ros-humble-ros2-control \\
    ros-humble-ros2-controllers \\
    ros-humble-gazebo-ros2-control \\
    && rm -rf /var/lib/apt/lists/*

# Set working directory
WORKDIR /workspace

# Copy source files
COPY src/ ./src/

# Build the workspace
RUN source /opt/ros/humble/setup.bash && \\
    colcon build --packages-select {lab_name.replace('-', '_')}

# Source the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /workspace/install/setup.bash" >> ~/.bashrc

# Set environment variables
ENV ROS_DOMAIN_ID=0
ENV RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Default command
CMD ["bash"]
"""
        with open(os.path.join(lab_exercise_dir, "Dockerfile"), 'w') as f:
            f.write(dockerfile_content)
        
        # Create docker-compose.yml
        compose_content = f"""version: '3.8'

services:
  {lab_name}:
    build: .
    container_name: {lab_name}
    privileged: true
    environment:
      - DISPLAY=${{DISPLAY:-}}
      - QT_X11_NO_MITSHM=1
      - XAUTHORITY=/tmp/.docker.xauth
      - ROS_DOMAIN_ID=0
    volumes:
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ./src:/workspace/src
      - ./launch:/workspace/launch
    network_mode: host
    stdin_open: true
    tty: true
    command: >
      bash -c "source /opt/ros/humble/setup.bash &&
               source /workspace/install/setup.bash &&
               sleep 5 &&
               bash"
"""
        with open(os.path.join(lab_exercise_dir, "docker-compose.yml"), 'w') as f:
            f.write(compose_content)
        
        # Create source directory and files
        src_dir = os.path.join(lab_exercise_dir, "src")
        os.makedirs(src_dir, exist_ok=True)
        
        # Create package.xml
        package_xml = f"""<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>{lab_name.replace('-', '_')}</name>
  <version>0.0.1</version>
  <description>{lab_desc}</description>
  <maintainer email="student@robotics.edu">Student</maintainer>
  <license>MIT</license>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>sensor_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
"""
        with open(os.path.join(src_dir, f"{lab_name.replace('-', '_')}_pkg"), 'w') as f:
            f.write("# This is a placeholder - actual package.xml would go here\n")
        
        # Create Python node
        node_content = f"""#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
import math


class {lab_name.replace('-', '').replace('_', '')}Node(Node):
    def __init__(self):
        super().__init__('{lab_name}_node')
        
        # Publisher for lab results
        self.publisher = self.create_publisher(String, '{lab_name}/result', 10)
        
        # Subscriber for sensor data
        self.subscription = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )
        
        # Timer for periodic processing
        self.timer = self.create_timer(0.1, self.process_data)
        
        self.get_logger().info('{lab_desc} node initialized')
        
    def joint_callback(self, msg):
        # Process joint state data for {agent_input.chapter_topic}
        self.get_logger().info(f'Received joint states: {{len(msg.name)}} joints')
        
    def process_data(self):
        # Process data according to {agent_input.chapter_topic} principles
        result = String()
        result.data = f'Processing {agent_input.chapter_topic} data'
        self.publisher.publish(result)


def main(args=None):
    rclpy.init(args=args)
    node = {lab_name.replace('-', '').replace('_', '')}Node()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
"""
        node_dir = os.path.join(src_dir, lab_name.replace('-', '_'))
        os.makedirs(node_dir, exist_ok=True)
        with open(os.path.join(node_dir, f"{lab_name.replace('-', '_')}_node.py"), 'w') as f:
            f.write(node_content)
        
        # Create launch directory and file
        launch_dir = os.path.join(lab_exercise_dir, "launch")
        os.makedirs(launch_dir, exist_ok=True)
        
        launch_content = f"""from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='{lab_name.replace('-', '_')}',
            executable='{lab_name.replace('-', '_')}_node',
            name='{lab_name}_node',
            output='screen',
            parameters=[
                # Add parameters specific to {agent_input.chapter_topic}
            ]
        )
    ])
"""
        with open(os.path.join(launch_dir, f"{lab_name.replace('-', '_')}.launch.py"), 'w') as f:
            f.write(launch_content)
        
        # Create Makefile
        makefile_content = f"""# Makefile for {lab_name}

.PHONY: build run stop clean test

build:
\tdocker-compose build

run: build
\tdocker-compose up -d
\tdocker-compose exec {lab_name} bash

stop:
\tdocker-compose down

clean:
\tdocker-compose down -v
\trm -rf build/ install/ log/

test:
\tdocker-compose exec {lab_name} bash -c "source /opt/ros/humble/setup.bash && source /workspace/install/setup.bash && python3 -m pytest tests/"

logs:
\tdocker-compose logs -f {lab_name}
"""
        with open(os.path.join(lab_exercise_dir, "Makefile"), 'w') as f:
            f.write(makefile_content)
        
        # Create README for the lab
        lab_readme = f"""# {lab_name}

## {lab_desc}

This lab exercise focuses on {agent_input.chapter_topic} concepts and implementation.

### Learning Objectives

- Understand fundamental principles of {agent_input.chapter_topic}
- Implement {agent_input.chapter_topic} algorithms in ROS 2
- Test and validate {agent_input.chapter_topic} implementations
- Analyze performance and limitations

### Prerequisites

- Basic knowledge of ROS 2 Humble
- Understanding of {agent_input.chapter_topic} theory (covered in Chapter {agent_input.chapter_number})
- Docker and docker-compose installed

### Setup Instructions

1. Build the Docker container:
   ```bash
   make build
   ```

2. Run the lab environment:
   ```bash
   make run
   ```

3. In the container, build the ROS workspace:
   ```bash
   cd /workspace && colcon build --packages-select {lab_name.replace('-', '_')}
   source install/setup.bash
   ```

### Lab Exercises

1. **Basic Implementation**: Implement a basic {agent_input.chapter_topic} algorithm
2. **Advanced Features**: Add advanced features to your implementation
3. **Performance Analysis**: Analyze and optimize your implementation

### Running the Lab

To run the ROS node:
```bash
ros2 run {lab_name.replace('-', '_')} {lab_name.replace('-', '_')}_node
```

To launch with the launch file:
```bash
ros2 launch {lab_name.replace('-', '_')} {lab_name.replace('-', '_')}.launch.py
```

### Expected Output

The node should publish messages to the `{lab_name}/result` topic with processing results.

### Assessment

Complete the following tasks:
1. Modify the node to implement the specific {agent_input.chapter_topic} algorithm
2. Validate your implementation against the provided test cases
3. Document your approach and findings

### Troubleshooting

- If you encounter build errors, ensure all dependencies are installed
- Check that ROS environment is properly sourced
- Verify that the Docker container has sufficient resources

---

**Lab created**: {datetime.now().isoformat()}
"""
        with open(os.path.join(lab_exercise_dir, "README.md"), 'w') as f:
            f.write(lab_readme)
        
        # Create expected output file
        expected_output = f"""# Expected Output for {lab_name}

## Expected Results

When running the {lab_name} node, you should see output similar to:

```
[INFO] [12345.789]: {lab_desc} node initialized
[INFO] [12345.790]: Received joint states: N joints
[INFO] [12345.791]: Processing {agent_input.chapter_topic} data
```

## Validation Criteria

Your implementation should:
1. Successfully subscribe to joint_states topic
2. Process {agent_input.chapter_topic} data correctly
3. Publish results to {lab_name}/result topic
4. Handle edge cases appropriately
5. Maintain real-time performance requirements

## Performance Benchmarks

- Processing rate: >10 Hz
- Memory usage: <100 MB
- CPU usage: <20% on typical hardware

## Common Issues

1. **Topic not found**: Ensure ROS nodes are in the same domain
2. **Performance issues**: Optimize algorithms for real-time execution
3. **Integration errors**: Check message type compatibility

---

**Expected output documented**: {datetime.now().isoformat()}
"""
        with open(os.path.join(lab_exercise_dir, "expected_output.txt"), 'w') as f:
            f.write(expected_output)

    def _create_lab_readme(self, lab_dir: str, agent_input: AgentInput):
        """Create main lab README."""
        readme_content = f"""# Chapter {agent_input.chapter_number} Labs: {agent_input.chapter_topic}

This directory contains lab exercises for Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}.

## Lab Overview

The labs in this chapter focus on practical implementation of {agent_input.chapter_topic} concepts using the Unitree G1 robot and ROS 2 Humble.

## Labs Included

1. **Lab {agent_input.chapter_number}.1**: Introduction to {agent_input.chapter_topic} - Basic Concepts
2. **Lab {agent_input.chapter_number}.2**: Intermediate {agent_input.chapter_topic} - Implementation
3. **Lab {agent_input.chapter_number}.3**: Advanced {agent_input.chapter_topic} - Integration and Optimization

## Prerequisites

- Docker and docker-compose
- Basic ROS 2 knowledge
- Understanding of {agent_input.chapter_topic} theory (Chapter {agent_input.chapter_number})

## Getting Started

Each lab has its own directory with:
- Dockerfile for the lab environment
- ROS 2 nodes implementing {agent_input.chapter_topic} concepts
- Launch files for easy execution
- Documentation and assessment criteria

To run a specific lab:
1. Navigate to the lab directory
2. Follow the instructions in the README.md file
3. Use the provided Makefile for common operations

## Assessment

Each lab includes assessment criteria to validate your implementation of {agent_input.chapter_topic} concepts.

---

**Lab directory created**: {datetime.now().isoformat()}
**Chapter**: {agent_input.chapter_number} - {agent_input.chapter_topic}
"""
        with open(os.path.join(lab_dir, "README.md"), 'w') as f:
            f.write(readme_content)