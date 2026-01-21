"""Diagram Generator Agent - Creates diagrams for the chapter."""

import logging
from typing import Dict, Any
import asyncio
from datetime import datetime

from src.agents.protocol import AgentInterface, AgentInput, AgentOutput, AgentStatus
import logging

logger = logging.getLogger(__name__)


class DiagramAgent(AgentInterface):
    """Diagram agent that creates diagrams for the chapter."""

    def __init__(self):
        super().__init__("Diagram")
        self.logger = logger

    async def execute(self, agent_input: AgentInput) -> AgentOutput:
        """Execute diagram generation for the chapter.

        Args:
            agent_input: Chapter metadata including number, topic, and previous artifacts

        Returns:
            AgentOutput with diagrams.md artifact
        """
        self.logger.info(f"Starting diagram generation for Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}")

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

            # Generate diagrams content
            diagrams_content = self._generate_diagrams_content(agent_input, chapter_content)
            
            # Write diagrams file
            diagrams_file_path = f"specs/1-book-curriculum/chapters/chapter-{agent_input.chapter_number}/diagrams.md"
            
            # Ensure directory exists
            import os
            os.makedirs(os.path.dirname(diagrams_file_path), exist_ok=True)
            
            with open(diagrams_file_path, 'w', encoding='utf-8') as f:
                f.write(diagrams_content)
            
            artifacts = {
                "diagrams.md": diagrams_file_path
            }
            
            self.logger.info(f"Diagram generation completed for Chapter {agent_input.chapter_number}")
            return AgentOutput(
                status=AgentStatus.SUCCESS,
                artifacts=artifacts,
                duration_seconds=10  # Simulated duration
            )
            
        except Exception as e:
            self.logger.error(f"Diagram agent failed: {str(e)}")
            return AgentOutput(
                status=AgentStatus.FAILURE,
                artifacts={},
                error_message=str(e)
            )

    def _generate_diagrams_content(self, agent_input: AgentInput, chapter_content: str) -> str:
        """Generate diagrams content based on chapter metadata and content."""
        content = """# Chapter {chapter_number} Diagrams: {chapter_topic}

## Diagram Descriptions

This document contains diagrams for Chapter {chapter_number}: {chapter_topic}. Each diagram is provided in multiple formats for accessibility.

## Diagram 1: System Architecture

### ASCII Art
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Sensor Data   │───▶│  Processing     │───▶│  Output         │
│   (Raw)         │    │  Unit           │    │  (Processed)    │
└─────────────────┘    └─────────────────┘    └─────────────────┘
```

### Mermaid Diagram
```mermaid
graph LR
    A[Sensor Data] --> B(Processing Unit)
    B --> C[Output Processed]
```

### Description
This diagram shows the basic data flow for {chapter_topic} systems, illustrating how raw sensor data is processed and transformed into actionable outputs.

## Diagram 2: Component Interaction

### ASCII Art
```
┌─────────────┐
│   ROS Node  │
└──────┬──────┘
       │
┌──────▼──────┐    ┌─────────────┐
│   Service   │◀───┤   Client    │
│   Server    │    │   Node      │
└─────────────┘    └─────────────┘
```

### Mermaid Diagram
```mermaid
sequenceDiagram
    participant Client
    participant Server
    Client->>Server: Request
    Server->>Client: Response
```

### Description
This diagram illustrates the interaction between ROS nodes for {chapter_topic}, showing how services and clients communicate.

## Diagram 3: Data Flow

### ASCII Art
```
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Input     │───▶│   Process   │───▶│   Output    │
│   Data      │    │             │    │   Result    │
└─────────────┘    └─────────────┘    └─────────────┘
       │                   │                   │
       ▼                   ▼                   ▼
┌─────────────┐    ┌─────────────┐    ┌─────────────┐
│   Buffer    │    │   Buffer    │    │   Buffer    │
└─────────────┘    └─────────────┘    └─────────────┘
```

### Mermaid Diagram
```mermaid
graph LR
    A[Input Data] --> B[Process]
    B --> C[Output Result]
    A --> D[Buffer]
    B --> E[Buffer]
    C --> F[Buffer]
```

### Description
This diagram shows the data flow and buffering mechanism for {chapter_topic} implementations, highlighting how data moves through the system.

## Diagram 4: Hardware Integration

### ASCII Art
```
┌─────────────────────────────────────┐
│              Robot                │
│  ┌─────────┐  ┌─────────┐  ┌─────────┐  │
│  │  Arm    │  │  Base   │  │  Head   │  │
│  │         │  │         │  │         │  │
│  └─────────┘  └─────────┘  └─────────┘  │
│         │           │           │        │
│         ▼           ▼           ▼        │
│  ┌─────────────────────────────────────┐ │
│  │         Control System              │ │
│  └─────────────────────────────────────┘ │
└─────────────────────────────────────────────┘
```

### Mermaid Diagram
```mermaid
graph TB
    subgraph Robot
        A[Arm]
        B[Base]
        C[Head]
    end
    subgraph Control
        D[Control System]
    end
    A --> D
    B --> D
    C --> D
```

### Description
This diagram shows the hardware integration for {chapter_topic}, illustrating how different robot components interact with the control system.

## Diagram 5: Algorithm Flow

### ASCII Art
```
    Start
      │
      ▼
┌─────────────┐
│   Initialize│
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Process   │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Validate  │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Output    │
└──────┬──────┘
       │
       ▼
     End
```

### Mermaid Diagram
```mermaid
flowchart TD
    A[Start] --> B[Initialize]
    B --> C[Process]
    C --> D[Validate]
    D --> E[Output]
    E --> F[End]
```

### Description
This diagram shows the algorithmic flow for {chapter_topic}, outlining the key steps in the processing pipeline.

## Diagram 6: Performance Metrics

### ASCII Art
```
Performance vs Time
│
│     ●
│   ●   ●
│ ●       ●
│●         ●
├────────────────► Time
 0   5   10   15
```

### Mermaid Diagram
```mermaid
graph LR
    A[Time] --> B[Performance]
    B --> C[Optimization]
```

### Description
This diagram illustrates the performance characteristics of {chapter_topic} implementations over time, showing how performance can be optimized.

## Diagram 7: Error Handling

### ASCII Art
```
┌─────────────┐
│   Input     │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Process   │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│   Check     │
└──────┬──────┘
       │
   ┌───▼───┐
   │ Error?│
   └───┬───┘
       │
   ┌───▼───┐    ┌─────────────┐
   │ Yes   │───▶│   Handle    │
   └───────┘    └─────────────┘
       │
   ┌───▼───┐    ┌─────────────┐
   │  No   │───▶│   Continue  │
   └───────┘    └─────────────┘
```

### Mermaid Diagram
```mermaid
flowchart TD
    A[Input] --> B[Process]
    B --> C[Check]
    C --> D[Error?]
    D -->|Yes| E[Handle]
    D -->|No| F[Continue]
    E --> G[Output]
    F --> G
```

### Description
This diagram shows the error handling flow for {chapter_topic} implementations, demonstrating how errors are detected and handled.

---

**Diagrams generated**: {datetime_iso}
**Chapter**: {chapter_number} - {chapter_topic}
"""
        # Replace placeholders in the content
        content = content.format(
            chapter_number=agent_input.chapter_number,
            chapter_topic=agent_input.chapter_topic,
            datetime_iso=datetime.now().isoformat()
        )
        return content