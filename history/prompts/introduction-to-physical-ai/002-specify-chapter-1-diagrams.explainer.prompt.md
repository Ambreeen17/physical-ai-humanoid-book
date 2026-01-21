---
id: 2
title: Specify diagrams for Chapter 1 Introduction to Physical AI
stage: explainer
date: 2025-12-31
surface: agent
model: claude-sonnet-4-5-20250929
feature: introduction-to-physical-ai
branch: main
user: user
command: none
labels:
  - diagrams
  - physical-ai
  - robotics
  - mermaid
  - ascii
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - C:\boook\diagrams.md
tests:
  - null
---

## Prompt
You are the Diagram & Visualization Agent for the AI-native robotics textbook.

**Your Task**: Specify diagrams needed for Chapter 1: "Introduction to Physical AI"

The Chapter Author has written the following content sections that need visual support:
1. Introduction to sensorimotor loop
2. Contrast between LLMs and embodied AI
3. Sim-to-Real Gap explanation
4. Humanoid platform comparison
5. ROS 2 node architecture
6. Pub/Sub messaging pattern

**Diagrams to Create**:

1. **Figure 1.1: Sensorimotor Loop** (referenced in text)
   - Show feedback cycle: Brain → Body → Sensors → World → Feedback
   - Create in: ASCII (for markdown), Mermaid diagram, and text description
   - Purpose: Explain the fundamental loop of embodied intelligence

2. **Figure 1.2: LLM vs Embodied AI**
   - Side-by-side comparison showing LLM as disembodied (text only) vs Robot as embodied (sensors + actuators)
   - Include arrows showing information flow
   - Create in: ASCII and Mermaid diagram

3. **Figure 1.3: Sim-to-Real Gap**
   - Show simulation environment on left, real world on right
   - Highlight differences: Friction, latency, sensor noise
   - Include domain randomization concept
   - Create in: ASCII diagram

4. **Figure 1.4: ROS 2 Node Architecture**
   - Multiple nodes (Camera Node, Controller Node, Motor Node) communicating via topics
   - Show pub/sub pattern with topic as central "hub"
   - Create in: Mermaid graph diagram

5. **Figure 1.5: Pub/Sub Message Flow**
   - Timeline showing Publisher pushing message to topic
   - Subscribers receiving message
   - Create in: ASCII sequence diagram

**Output Format**: For each diagram, provide:
- **Diagram Type**: ASCII / Mermaid / Description
- **Diagram Code/ASCII**: The actual diagram
- **Placement**: Where in chapter this should appear (e.g., "After section 1.1")
- **Caption**: Figure number and descriptive caption
- **Alt Text**: Accessible description for screen readers

**Mermaid Examples**:
- Use `graph LR` for left-to-right flows
- Use `graph TD` for top-down diagrams
- Use `sequenceDiagram` for message flows

**ASCII Guidelines**:
- Use box drawing characters: `+`, `-`, `|`, `├`, `└`, `┌`, `┐`, `┬`, `┴`, etc.
- Keep diagrams readable (max 80 chars wide)
- Label all major components
- Show data/control flows with arrows

**Target Audience**: Software engineers (understand code/architecture, appreciate technical accuracy)

**Deliverable**: A structured diagrams.md file with all specifications ready for implementation.

## Response
I have created the requested diagrams for Chapter 1: "Introduction to Physical AI". The diagrams cover the Sensorimotor Loop, LLM vs Embodied AI comparison, the Sim-to-Real Gap, ROS 2 Node Architecture, and Pub/Sub Message Flow. Each figure includes an ASCII representation for terminal/markdown readability, a Mermaid diagram for interactive rendering, and accessible descriptions (alt-text/captions) for Docusaurus. The specifications are structured in `C:\boook\diagrams.md`.

## Outcome
Diagram specifications completed and saved to `C:\boook\diagrams.md`. This provides the visual foundation for Chapter 1 content.
