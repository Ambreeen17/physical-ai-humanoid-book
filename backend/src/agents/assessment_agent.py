"""Assessment Generator Agent - Creates assessments for the chapter."""

import logging
from typing import Dict, Any
import asyncio
from datetime import datetime
import os

from src.agents.protocol import AgentInterface, AgentInput, AgentOutput, AgentStatus
import logging

logger = logging.getLogger(__name__)


class AssessmentAgent(AgentInterface):
    """Assessment agent that creates assessments for the chapter."""

    def __init__(self):
        super().__init__("Assessment")
        self.logger = logger

    async def execute(self, agent_input: AgentInput) -> AgentOutput:
        """Execute assessment generation for the chapter.

        Args:
            agent_input: Chapter metadata including number, topic, and previous artifacts

        Returns:
            AgentOutput with assessments.md artifact
        """
        self.logger.info(f"Starting assessment generation for Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}")

        try:
            # Generate assessments content
            assessments_content = self._generate_assessments_content(agent_input)
            
            # Write assessments file
            assessments_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/assessments.md"
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(assessments_file_path), exist_ok=True)
            
            with open(assessments_file_path, 'w', encoding='utf-8') as f:
                f.write(assessments_content)
            
            artifacts = {
                "assessments.md": assessments_file_path
            }
            
            self.logger.info(f"Assessment generation completed for Chapter {agent_input.chapter_number}")
            return AgentOutput(
                status=AgentStatus.SUCCESS,
                artifacts=artifacts,
                duration_seconds=15  # Simulated duration
            )
            
        except Exception as e:
            self.logger.error(f"Assessment agent failed: {str(e)}")
            return AgentOutput(
                status=AgentStatus.FAILURE,
                artifacts={},
                error_message=str(e)
            )

    def _generate_assessments_content(self, agent_input: AgentInput) -> str:
        """Generate assessments content based on chapter metadata."""
        content = f"""# Chapter {agent_input.chapter_number} Assessments: {agent_input.chapter_topic}

## Assessment Overview

This document contains assessments for Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}. The assessments are designed to test understanding of the learning objectives and practical application of concepts.

## Assessment Structure

- **Multiple Choice Questions**: 3 questions (3 points total)
- **Short Answer Questions**: 1 question (5 points)
- **Coding Task**: 1 task (8 points)
- **Challenge Problem**: 1 problem (5 points)
- **Total Points**: 21 points

## Learning Objectives Alignment

Each assessment aligns with the following learning objectives:

"""
        if isinstance(agent_input.context, dict) and "learning_objectives" in agent_input.context:
            for i, objective in enumerate(agent_input.context["learning_objectives"], 1):
                content += f"- LO{i}: {objective}\n"
        else:
            content += f"- LO1: Define key concepts related to {agent_input.chapter_topic}\n"
            content += f"- LO2: Apply {agent_input.chapter_topic} principles to robotics problems\n"
            content += f"- LO3: Implement {agent_input.chapter_topic} solutions in ROS 2\n"
            content += f"- LO4: Evaluate {agent_input.chapter_topic} performance\n"
            content += f"- LO5: Integrate {agent_input.chapter_topic} with other robotics systems\n"

        content += f"""

## Multiple Choice Questions

### Question 1 (1 point)
Which of the following best describes the primary purpose of {agent_input.chapter_topic} in robotics?

A) To provide power to robotic systems
B) To enable robots to perceive and interact with their environment
C) To store data for long-term analysis
D) To provide structural support for robotic components

**Correct Answer**: B
**Learning Objective**: LO1
**Difficulty**: Beginner
**Explanation**: {agent_input.chapter_topic} enables robots to perceive and interact with their environment, which is fundamental to robotic functionality.

### Question 2 (1 point)
In the context of {agent_input.chapter_topic}, what does the term "real-time constraint" refer to?

A) The requirement for systems to operate continuously without interruption
B) The requirement for systems to respond within a specified time frame
C) The requirement for systems to operate only during certain hours
D) The requirement for systems to process data from real sensors only

**Correct Answer**: B
**Learning Objective**: LO2
**Difficulty**: Intermediate
**Explanation**: Real-time constraints require systems to respond within specified time frames to ensure proper operation.

### Question 3 (1 point)
Which of the following is a key consideration when implementing {agent_input.chapter_topic} on the Unitree G1 robot?

A) The robot's color scheme
B) The robot's weight distribution
C) The computational resources and sensor integration
D) The robot's maximum speed

**Correct Answer**: C
**Learning Objective**: LO3
**Difficulty**: Advanced
**Explanation**: Computational resources and sensor integration are critical for effective {agent_input.chapter_topic} implementation on the Unitree G1.

## Short Answer Questions

### Question 4 (5 points)
Explain the key differences between traditional approaches to {agent_input.chapter_topic} and modern implementations. Include at least three specific differences in your response.

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

**Sample Answer**: Modern implementations of {agent_input.chapter_topic} differ from traditional approaches in several key ways. First, modern approaches leverage increased computational power to perform more complex calculations in real-time. Second, they incorporate advanced sensor fusion techniques to combine data from multiple sources. Third, they utilize machine learning algorithms to adapt to changing conditions and improve performance over time.

## Coding Task

### Question 5 (8 points)
Implement a basic {agent_input.chapter_topic} algorithm in ROS 2 Humble that processes sensor data and produces an appropriate output. Your implementation should:

1. Subscribe to a sensor data topic
2. Process the data according to {agent_input.chapter_topic} principles
3. Publish the processed data to an output topic
4. Include appropriate error handling
5. Be well-documented with comments

**Learning Objective**: LO3, LO4
**Difficulty**: Advanced
**Expected Files**: {agent_input.chapter_topic.replace(' ', '_').lower()}_node.py
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

class {agent_input.chapter_topic.replace(' ', '').replace('-', '')}Node(Node):
    def __init__(self):
        super().__init__('{agent_input.chapter_topic.replace(' ', '_').lower()}_node')
        # Initialize node components
        self.get_logger().info('{agent_input.chapter_topic} node initialized')
    
    def sensor_callback(self, msg):
        # Process sensor data according to {agent_input.chapter_topic} principles
        pass

def main(args=None):
    rclpy.init(args=args)
    node = {agent_input.chapter_topic.replace(' ', '').replace('-', '')}Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Challenge Problem

### Question 6 (5 points)
Design an optimization strategy for {agent_input.chapter_topic} implementation that balances computational efficiency with accuracy. Consider the constraints of the Unitree G1 robot and provide specific recommendations for:

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

**Sample Answer**: An effective optimization strategy for {agent_input.chapter_topic} implementation on the Unitree G1 should prioritize algorithm efficiency by selecting algorithms with appropriate computational complexity for the robot's capabilities. Data processing should utilize filtering techniques to reduce computational load while maintaining accuracy. Resource allocation should prioritize critical functions and implement fallback mechanisms. Performance monitoring should include real-time metrics and adaptive adjustments.

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
- Implementation of {agent_input.chapter_topic} logic (2 points)
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

**Assessments generated**: {datetime.now().isoformat()}
**Chapter**: {agent_input.chapter_number} - {agent_input.chapter_topic}
"""
        return content