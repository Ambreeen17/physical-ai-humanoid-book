"""Agent communication protocol and interfaces."""

from typing import Dict, List, Any, Optional
from dataclasses import dataclass, asdict
from datetime import datetime
from enum import Enum
import uuid


class AgentStatus(str, Enum):
    """Agent execution status."""
    PENDING = "pending"
    RUNNING = "running"
    SUCCESS = "success"
    FAILURE = "failure"


@dataclass
class AgentInput:
    """Input to an agent."""
    chapter_id: Optional[str] = None
    chapter_number: Optional[int] = None
    chapter_topic: Optional[str] = None
    chapter_outline: Optional[str] = None
    previous_artifacts: Optional[Dict[str, str]] = None  # e.g., {"research.md": "path/to/file"}

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class AgentOutput:
    """Output from an agent."""
    status: AgentStatus
    artifacts: Dict[str, str]  # {"research.md": "path/to/file"}
    error_message: Optional[str] = None
    duration_seconds: Optional[int] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "status": self.status.value,
            "artifacts": self.artifacts,
            "error_message": self.error_message,
            "duration_seconds": self.duration_seconds,
        }


@dataclass
class AgentExecutionRecord:
    """Record of agent execution."""
    id: str = None
    agent_name: str = None
    chapter_id: Optional[str] = None
    status: AgentStatus = AgentStatus.PENDING
    started_at: Optional[datetime] = None
    completed_at: Optional[datetime] = None
    input_artifacts: List[str] = None
    output_artifacts: List[str] = None
    error_message: Optional[str] = None
    duration_seconds: Optional[int] = None

    def __post_init__(self):
        if self.id is None:
            self.id = str(uuid.uuid4())
        if self.input_artifacts is None:
            self.input_artifacts = []
        if self.output_artifacts is None:
            self.output_artifacts = []

    def to_dict(self) -> Dict[str, Any]:
        return {
            "id": self.id,
            "agent_name": self.agent_name,
            "chapter_id": self.chapter_id,
            "status": self.status.value,
            "started_at": self.started_at.isoformat() if self.started_at else None,
            "completed_at": self.completed_at.isoformat() if self.completed_at else None,
            "input_artifacts": self.input_artifacts,
            "output_artifacts": self.output_artifacts,
            "error_message": self.error_message,
            "duration_seconds": self.duration_seconds,
        }


class AgentInterface:
    """Base interface for all agents."""

    def __init__(self, name: str):
        self.name = name

    async def execute(self, agent_input: AgentInput) -> AgentOutput:
        """Execute the agent.

        Args:
            agent_input: Input to the agent

        Returns:
            AgentOutput with status, artifacts, and optional error
        """
        raise NotImplementedError("Subclasses must implement execute()")
