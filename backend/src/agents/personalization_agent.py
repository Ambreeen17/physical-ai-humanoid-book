"""Personalization Agent - Creates personalized content variants for the chapter."""

import logging
from typing import Dict, Any
import asyncio
from datetime import datetime
import os
import json

from src.agents.protocol import AgentInterface, AgentInput, AgentOutput, AgentStatus
import logging

logger = logging.getLogger(__name__)


class PersonalizationAgent(AgentInterface):
    """Personalization agent that creates difficulty variants for the chapter."""

    def __init__(self):
        super().__init__("Personalization")
        self.logger = logger

    async def execute(self, agent_input: AgentInput) -> AgentOutput:
        """Execute personalization for the chapter.

        Args:
            agent_input: Chapter metadata including number, topic, and previous artifacts

        Returns:
            AgentOutput with personalization artifacts
        """
        self.logger.info(f"Starting personalization for Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}")

        try:
            # Get chapter content if available from previous artifacts
            chapter_content = ""
            if agent_input.previous_artifacts and "chapter-draft.md" in agent_input.previous_artifacts:
                chapter_path = agent_input.previous_artifacts["chapter-draft.md"]
                try:
                    with open(chapter_path, 'r', encoding='utf-8') as f:
                        chapter_content = f.read()
                except FileNotFoundError:
                    self.logger.warning(f"Chapter file not found: {chapter_path}")

            # Generate personalization content
            personalization_content = self._generate_personalization_content(agent_input, chapter_content)
            
            # Write personalization file
            personalization_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/personalization.md"
            
            # Ensure directory exists
            os.makedirs(os.path.dirname(personalization_file_path), exist_ok=True)
            
            with open(personalization_file_path, 'w', encoding='utf-8') as f:
                f.write(personalization_content)
            
            # Generate personalization config
            config = self._generate_personalization_config(agent_input)
            config_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/personalization_config.json"
            
            with open(config_file_path, 'w', encoding='utf-8') as f:
                json.dump(config, f, indent=2)
            
            artifacts = {
                "personalization.md": personalization_file_path,
                "personalization_config.json": config_file_path
            }
            
            self.logger.info(f"Personalization completed for Chapter {agent_input.chapter_number}")
            return AgentOutput(
                status=AgentStatus.SUCCESS,
                artifacts=artifacts,
                duration_seconds=20  # Simulated duration
            )
            
        except Exception as e:
            self.logger.error(f"Personalization agent failed: {str(e)}")
            return AgentOutput(
                status=AgentStatus.FAILURE,
                artifacts={},
                error_message=str(e)
            )

    def _generate_personalization_content(self, agent_input: AgentInput, chapter_content: str) -> str:
        """Generate personalization content based on chapter metadata and content."""
        content = f"""# Chapter {agent_input.chapter_number} Personalization: {agent_input.chapter_topic}

## Personalization Overview

This document outlines the personalization strategy for Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}. The content is adapted for three difficulty levels: Beginner, Intermediate, and Advanced.

## Difficulty Levels

### Beginner
- Focus on intuitive understanding
- Minimal mathematical complexity
- Complete starter code
- Detailed explanations
- Visual aids and analogies

### Intermediate
- Balance between intuition and mathematical rigor
- Partial starter code
- Moderate mathematical content
- Practical examples and applications

### Advanced
- Research-level content
- High mathematical rigor
- Minimal starter code
- Focus on implementation details
- Research papers and advanced concepts

## Content Adaptations

### Beginner Adaptations

#### Simplified Explanations
{agent_input.chapter_topic} can be thought of as [intuitive explanation with analogies]. This approach helps beginners understand the core concepts without getting overwhelmed by mathematical details.

#### Complete Starter Code
For beginners, we provide complete starter code with detailed comments:

```python
# Beginner-friendly implementation of {agent_input.chapter_topic}
# All necessary components are provided
# Just fill in the specific logic for your use case

import rclpy
from rclpy.node import Node

class Beginner{agent_input.chapter_topic.replace(' ', '').replace('-', '')}Node(Node):
    def __init__(self):
        super().__init__('beginner_{agent_input.chapter_topic.replace(' ', '_').lower()}_node')
        # All setup is provided
        # Your task: implement the core logic
        self.get_logger().info('Beginner {agent_input.chapter_topic} node initialized')
    
    def process_data(self, raw_data):
        # Beginner-friendly approach
        # Simple, clear implementation
        processed_data = raw_data  # Placeholder - student fills in
        return processed_data

def main(args=None):
    rclpy.init(args=args)
    node = Beginner{agent_input.chapter_topic.replace(' ', '').replace('-', '')}Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Visual Learning Aids
- Emphasis on diagrams and flowcharts
- Step-by-step visual guides
- Interactive elements where possible

### Intermediate Adaptations

#### Balanced Approach
Intermediate learners get a balance of intuitive explanations and mathematical foundations. The content provides enough rigor to understand the underlying principles while remaining accessible.

#### Partial Starter Code
For intermediate learners, we provide partial starter code with some components missing:

```python
# Intermediate implementation of {agent_input.chapter_topic}
# Some components provided, others to be implemented

import rclpy
from rclpy.node import Node
# Additional imports as needed

class Intermediate{agent_input.chapter_topic.replace(' ', '').replace('-', '')}Node(Node):
    def __init__(self):
        super().__init__('intermediate_{agent_input.chapter_topic.replace(' ', '_').lower()}_node')
        # Basic setup provided
        # Student implements additional functionality
        self.get_logger().info('Intermediate {agent_input.chapter_topic} node initialized')
        
        # TODO: Add subscriber for sensor data
        # self.subscription = ...
        
        # TODO: Add publisher for processed data
        # self.publisher = ...
    
    def sensor_callback(self, msg):
        # Process sensor data
        # Student implements the core logic
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Intermediate{agent_input.chapter_topic.replace(' ', '').replace('-', '')}Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Mathematical Foundations
- Core equations provided with explanations
- Derivations for key concepts
- Practical applications of theory

### Advanced Adaptations

#### Research-Level Content
Advanced learners receive content that reflects current research in {agent_input.chapter_topic}, including recent developments and open problems.

#### Minimal Starter Code
Advanced learners receive minimal starter code, requiring them to implement most components:

```python
# Advanced implementation of {agent_input.chapter_topic}
# Minimal starter code provided
# Student implements complete solution

import rclpy
from rclpy.node import Node
import numpy as np
# Additional imports as needed

class Advanced{agent_input.chapter_topic.replace(' ', '').replace('-', '')}Node(Node):
    def __init__(self):
        super().__init__('advanced_{agent_input.chapter_topic.replace(' ', '_').lower()}_node')
        # Student implements complete initialization
        self.get_logger().info('Advanced {agent_input.chapter_topic} node initialized')
        
        # Student implements complete ROS interface
        # Subscribers, publishers, services, etc.
    
    def process_{agent_input.chapter_topic.replace(' ', '_').lower()}(self, data):
        # Advanced implementation required
        # Optimized algorithms, error handling, etc.
        pass

def main(args=None):
    rclpy.init(args=args)
    node = Advanced{agent_input.chapter_topic.replace(' ', '').replace('-', '')}Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

#### Advanced Topics
- Cutting-edge research topics
- Implementation of recent papers
- Performance optimization techniques
- Advanced debugging and validation

## Assessment Adaptations

### Beginner Assessments
- Focus on conceptual understanding
- Multiple choice and simple applications
- Detailed feedback and hints
- Step-by-step guidance

### Intermediate Assessments
- Balance of conceptual and practical
- Short answer and implementation tasks
- Moderate complexity problems
- Constructive feedback

### Advanced Assessments
- Complex implementation challenges
- Research-style problems
- Performance optimization tasks
- Self-directed learning components

## Learning Path Recommendations

### For Beginners
1. Start with intuitive explanations
2. Work through complete examples
3. Focus on understanding core concepts
4. Gradually introduce mathematical foundations

### For Intermediate Learners
1. Review mathematical foundations
2. Implement partial solutions
3. Understand trade-offs between approaches
4. Connect theory to practice

### For Advanced Learners
1. Focus on implementation details
2. Explore research papers
3. Optimize for performance
4. Contribute to open problems

---

**Personalization generated**: {datetime.now().isoformat()}
**Chapter**: {agent_input.chapter_number} - {agent_input.chapter_topic}
"""
        return content

    def _generate_personalization_config(self, agent_input: AgentInput) -> Dict:
        """Generate personalization configuration."""
        return {
            "chapter": agent_input.chapter_number,
            "topic": agent_input.chapter_topic,
            "difficulty_levels": {
                "beginner": {
                    "content_modifications": [
                        "simplified_explanations",
                        "complete_starter_code",
                        "visual_aids",
                        "intuitive_approach"
                    ],
                    "assessment_modifications": [
                        "conceptual_focus",
                        "multiple_choice",
                        "detailed_feedback",
                        "step_by_step_guidance"
                    ],
                    "learning_path": [
                        "intuitive_understanding",
                        "complete_examples",
                        "core_concepts",
                        "mathematical_foundations"
                    ]
                },
                "intermediate": {
                    "content_modifications": [
                        "balanced_approach",
                        "partial_starter_code",
                        "mathematical_foundations",
                        "practical_applications"
                    ],
                    "assessment_modifications": [
                        "balanced_assessment",
                        "short_answer",
                        "implementation_tasks",
                        "constructive_feedback"
                    ],
                    "learning_path": [
                        "mathematical_foundations",
                        "partial_implementation",
                        "trade_offs",
                        "theory_practice_connection"
                    ]
                },
                "advanced": {
                    "content_modifications": [
                        "research_level_content",
                        "minimal_starter_code",
                        "cutting_edge_topics",
                        "performance_optimization"
                    ],
                    "assessment_modifications": [
                        "complex_implementation",
                        "research_problems",
                        "performance_optimization",
                        "self_directed_learning"
                    ],
                    "learning_path": [
                        "implementation_details",
                        "research_papers",
                        "performance_optimization",
                        "open_problems"
                    ]
                }
            },
            "generated_at": datetime.now().isoformat()
        }