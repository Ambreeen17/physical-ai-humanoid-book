# Chapter 16 Assessments: Capstone Project

## Assessment Overview

This document contains assessments for Chapter 16: Capstone Project. The assessments are designed to test understanding of the learning objectives and practical application of concepts.

## Assessment Structure

- **Multiple Choice Questions**: 3 questions (3 points total)
- **Short Answer Questions**: 1 question (5 points)
- **Coding Task**: 1 task (8 points)
- **Challenge Problem**: 1 problem (5 points)
- **Total Points**: 21 points

## Learning Objectives Alignment

Each assessment aligns with the following learning objectives:

- LO1: Synthesize knowledge from all previous chapters into a cohesive project
- LO2: Design and implement a complete robotic system with multiple capabilities
- LO3: Integrate perception, planning, control, learning, and safety components
- LO4: Deploy and validate the system in a real-world or simulated environment
- LO5: Evaluate and present the project with technical documentation and analysis


## Multiple Choice Questions

### Question 1 (1 point)
Which of the following best describes the primary purpose of Capstone Project in robotics?

A) To provide power to robotic systems
B) To enable robots to perceive and interact with their environment
C) To store data for long-term analysis
D) To provide structural support for robotic components

**Correct Answer**: B
**Learning Objective**: LO1
**Difficulty**: Beginner
**Explanation**: Capstone Project enables robots to perceive and interact with their environment, which is fundamental to robotic functionality.

### Question 2 (1 point)
In the context of Capstone Project, what does the term "real-time constraint" refer to?

A) The requirement for systems to operate continuously without interruption
B) The requirement for systems to respond within a specified time frame
C) The requirement for systems to operate only during certain hours
D) The requirement for systems to process data from real sensors only

**Correct Answer**: B
**Learning Objective**: LO2
**Difficulty**: Intermediate
**Explanation**: Real-time constraints require systems to respond within specified time frames to ensure proper operation.

### Question 3 (1 point)
Which of the following is a key consideration when implementing Capstone Project on the Unitree G1 robot?

A) The robot's color scheme
B) The robot's weight distribution
C) The computational resources and sensor integration
D) The robot's maximum speed

**Correct Answer**: C
**Learning Objective**: LO3
**Difficulty**: Advanced
**Explanation**: Computational resources and sensor integration are critical for effective Capstone Project implementation on the Unitree G1.

## Short Answer Questions

### Question 4 (5 points)
Explain the key differences between traditional approaches to Capstone Project and modern implementations. Include at least three specific differences in your response.

**Learning Objective**: LO1, LO2
**Difficulty**: Intermediate
**Expected Length**: 150-200 words
**Scoring Rubric**:
- 5 points: Comprehensive explanation with 3+ specific differences clearly articulated
- 4 points: Good explanation with 3 differences, minor omissions
- 3 points: Adequate explanation with 2-3 differences
- 2 points: Basic explanation with 1-2 differences
- 1 point: Limited explanation with minimal differences
- 0 points: No response or irrelevant content

**Sample Answer**: Modern implementations of Capstone Project differ from traditional approaches in several key ways. First, modern approaches leverage increased computational power to perform more complex calculations in real-time. Second, they incorporate advanced sensor fusion techniques to combine data from multiple sources. Third, they utilize machine learning algorithms to adapt to changing conditions and improve performance over time.

## Coding Task

### Question 5 (8 points)
Implement a basic Capstone Project algorithm in ROS 2 Humble that processes sensor data and produces an appropriate output. Your implementation should:

1. Subscribe to a sensor data topic
2. Process the data according to Capstone Project principles
3. Publish the processed data to an output topic
4. Include appropriate error handling
5. Be well-documented with comments

**Learning Objective**: LO3, LO4
**Difficulty**: Advanced
**Expected Files**: capstone_project_node.py
**Scoring Rubric**:
- 8 points: Complete implementation meeting all requirements with excellent documentation
- 6-7 points: Implementation meeting most requirements with good documentation
- 4-5 points: Implementation meeting basic requirements with adequate documentation
- 2-3 points: Partial implementation with minimal documentation
- 1 point: Attempted implementation with significant issues
- 0 points: No implementation or completely non-functional code

**Skeleton Code**:
```python
import rclpy
from rclpy.node import Node
# Add other necessary imports

class CapstoneProjectNode(Node):
    def __init__(self):
        super().__init__('capstone_project_node')
        # Initialize node components
        self.get_logger().info('Capstone Project node initialized')
    
    def sensor_callback(self, msg):
        # Process sensor data according to Capstone Project principles
        pass

def main(args=None):
    rclpy.init(args=args)
    node = CapstoneProjectNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Challenge Problem

### Question 6 (5 points)
Design an optimization strategy for Capstone Project implementation that balances computational efficiency with accuracy. Consider the constraints of the Unitree G1 robot and provide specific recommendations for:

1. Algorithm selection
2. Data processing techniques
3. Resource allocation
4. Performance monitoring

**Learning Objective**: LO4, LO5
**Difficulty**: Advanced
**Expected Length**: 200-250 words
**Scoring Rubric**:
- 5 points: Comprehensive strategy addressing all four areas with specific, practical recommendations
- 4 points: Good strategy addressing most areas with practical recommendations
- 3 points: Adequate strategy addressing key areas
- 2 points: Basic strategy with limited recommendations
- 1 point: Limited strategy with minimal recommendations
- 0 points: No response or irrelevant content

**Sample Answer**: An effective optimization strategy for Capstone Project implementation on the Unitree G1 should prioritize algorithm efficiency by selecting algorithms with appropriate computational complexity for the robot's capabilities. Data processing should utilize filtering techniques to reduce computational load while maintaining accuracy. Resource allocation should prioritize critical functions and implement fallback mechanisms. Performance monitoring should include real-time metrics and adaptive adjustments.

## Answer Key

### Multiple Choice Answers
1. B
2. B
3. C

### Short Answer Sample Answer
See Question 4 for sample answer.

### Coding Task Evaluation Criteria
- Correct ROS 2 node structure (2 points)
- Proper topic subscription and publishing (2 points)
- Implementation of Capstone Project logic (2 points)
- Error handling (1 point)
- Code documentation (1 point)

### Challenge Problem Sample Answer
See Question 6 for sample answer.

## Assessment Administration

### Timing
- Multiple Choice: 15 minutes
- Short Answer: 20 minutes
- Coding Task: 45 minutes
- Challenge Problem: 20 minutes
- Total: 100 minutes (approximately 1.5 hours)

### Grading Scale
- A: 90-100% (19-21 points)
- B: 80-89% (17-18 points)
- C: 70-79% (15-16 points)
- D: 60-69% (13-14 points)
- F: Below 60% (Below 13 points)

---

**Assessments generated**: 2026-01-05T23:06:04.817291
**Chapter**: 16 - Capstone Project
