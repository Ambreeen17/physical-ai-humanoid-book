# Assessment Suite: Introduction to Physical AI

## Overview
- **Learning Objectives Assessed**:
  1. Define embodied intelligence and contrast with disembodied AI.
  2. Explain the sim-to-real gap and bridge strategies.
  3. Identify current humanoid platforms and their capabilities.
  4. Understand ROS 2 nodes, topics, and pub/sub messaging.
  5. Articulate why physical constraints matter in robotics.
- **Capstone Alignment Summary**: This assessment sets foundations for the capstone humanoid project by introducing the sensorimotor loop, ROS 2 communication, and simulation-to-real transfer concepts—all critical for building a functional humanoid.
- **Difficulty Progression Map**:
  - Foundational: Concept retrieval and terminology (Multiple Choice).
  - Intermediate: Conceptual synthesis and explanation (Short Answer).
  - Advanced: Practical implementation and modification (Lab Exercise).
  - Capstone-prep: Independent system design and safety logic (Integration Challenge).

## 1. Conceptual Questions

| ID | Difficulty | Question Stem | Alignment |
|----|------------|---------------|-----------|
| 1.1 | Foundational | What is the primary difference between an LLM (like ChatGPT) and a Vision-Language-Action (VLA) model? | Objective 1 |
| 1.2 | Foundational | Which of the following is NOT a strategy for bridging the Sim-to-Real gap? | Objective 2 |
| 1.3 | Foundational | In ROS 2, what is a "Topic"? | Objective 4 |

---

## 2. Coding Tasks

### Task 1: Extend the Hello Physical AI Lab
- **Problem Statement**: Modify the `hello_physical_ai_node.py` to implement real-time safety warnings based on robot rotation.
- **Specifications**: Add a rule that detect rotation changes in `theta` and logs a warning message.
- **Learning Objective**: Objective 4 & 5 (Implementing logic within a ROS 2 node).
- **Time Estimate**: 30 minutes.

---

## 3. Simulation Challenges

### Challenge 1: The Safety Watchdog
- **Scenario**: A warehouse robot requires a secondary safety layer to prevent collisions or out-of-bounds movement.
- **Task**: Design a `safety_watchdog_node.py` that monitors `/turtle1/pose` and publishes a zero-velocity command to `/turtle1/cmd_vel` if boundaries (X > 3.0 or Y > 2.0) are exceeded.
- **Decision Points**: Sampling rate of the pose, priority of the stop command, and logging verbosity.
- **Success Metrics**: Robot stops before hitting virtual walls; watchdog reacts in < 100ms.

---

## 4. Evaluation Rubrics

### 4.1 conceptual Understanding Rubric (Total: 6 pts)
- **Excellent (5-6 pts)**: Accurate, deep explanations; clear understanding of feedback loops.
- **Proficient (3-4 pts)**: Correct terminology but minor gaps in nuance.
- **Developing (1-2 pts)**: Basic recall but struggles with "why" (causality).

### 4.2 Lab & Code Quality Rubric (Total: 10 pts)
- **Code Correctness (4 pts)**: Node runs, rules trigger accurately.
- **Evidence (3 pts)**: Logs/Screenshots confirm the sensorimotor loop in action.
- **Reflection (3 pts)**: Explanation correctly identifies how actions feed back into perception.

### 4.3 Integration Challenge Rubric (Total: 5 bonus pts)
- **Implementation (3 pts)**: Separate node correctly subscribes and publishes.
- **Safety Logic (2 pts)**: Handles boundary conditions without false positives.

## Implementation Notes
- **Prerequisite knowledge**: Basic Python, Ubuntu/Docker terminal basics.
- **Common Misconceptions**: "Sim-to-real is just higher resolution graphics" (forgetting physics), "Nodes must be in the same file to communicate."
- **Differentiation**:
  - Beginners: Focus on the Multiple Choice and simple rule modification.
  - Intermediate: Complete the full Lab Exercise.
  - Advanced: Complete the Integration Challenge with a separate node.
