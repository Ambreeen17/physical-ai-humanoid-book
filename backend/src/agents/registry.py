"""Agent registry - manages agent instances and metadata."""

import logging
from typing import Dict, Optional
from src.agents.protocol import AgentInterface

logger = logging.getLogger(__name__)


class AgentRegistry:
    """Central registry for all agents."""

    def __init__(self):
        self._agents: Dict[str, AgentInterface] = {}
        self._metadata: Dict[str, Dict] = {}

    def register(self, name: str, agent: AgentInterface, metadata: Optional[Dict] = None):
        """Register an agent.

        Args:
            name: Agent name (e.g., "Research", "Author")
            agent: Agent instance
            metadata: Optional metadata about the agent (version, description, etc.)
        """
        self._agents[name] = agent
        self._metadata[name] = metadata or {}
        logger.info(f"Registered agent: {name}")

    def get(self, name: str) -> Optional[AgentInterface]:
        """Get an agent by name.

        Args:
            name: Agent name

        Returns:
            Agent instance or None
        """
        return self._agents.get(name)

    def list_agents(self) -> Dict[str, Dict]:
        """List all registered agents with metadata.

        Returns:
            Dictionary of agent names to metadata
        """
        return {
            name: {
                "name": name,
                "metadata": self._metadata.get(name, {}),
                "available": True,
            }
            for name in self._agents.keys()
        }

    def is_registered(self, name: str) -> bool:
        """Check if an agent is registered.

        Args:
            name: Agent name

        Returns:
            True if agent is registered
        """
        return name in self._agents


# Global registry instance
registry = AgentRegistry()
