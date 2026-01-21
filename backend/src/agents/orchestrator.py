"""Book Orchestrator Agent - controls the 10-agent pipeline for each chapter."""

import logging
from datetime import datetime
from typing import Dict, List, Optional
import asyncio
import uuid

from src.agents.protocol import AgentInterface, AgentInput, AgentOutput, AgentStatus, AgentExecutionRecord
from src.logging.logger import setup_logging

# Setup logging
setup_logging()

logger = logging.getLogger(__name__)


class BookOrchestrator(AgentInterface):
    """Orchestrator agent that sequences all other agents for chapter production."""

    def __init__(self):
        super().__init__("BookOrchestrator")
        self.agent_registry = {}
        self.execution_records: List[AgentExecutionRecord] = []

    def register_agent(self, agent_name: str, agent: AgentInterface):
        """Register an agent in the registry.

        Args:
            agent_name: Name of the agent (e.g., "Research", "Author", "Lab")
            agent: Agent instance
        """
        self.agent_registry[agent_name] = agent
        logger.info(f"Registered agent: {agent_name}")

    async def execute(self, agent_input: AgentInput) -> AgentOutput:
        """Execute the full chapter production pipeline.

        Sequence:
        1. Research Agent
        2. Chapter Author Agent
        3. Diagram Agent
        4. Robotics Lab Agent (if technical chapter)
        5. Assessment Agent
        6. Personalization Agent
        7. Localization Agent (Urdu)
        8. RAG Indexing Agent
        9. QA Agent

        Args:
            agent_input: Chapter metadata (number, topic, outline)

        Returns:
            AgentOutput with all produced artifacts
        """
        logger.info(f"Starting orchestration for Chapter {agent_input.chapter_number}: {agent_input.chapter_topic}")
        start_time = datetime.utcnow()

        agent_sequence = [
            "Research",
            "Author",
            "Diagram",
            "Lab",
            "Assessment",
            "Personalization",
            "Localization",
            "RAGIndexing",
            "QA",
        ]

        all_artifacts = {}
        previous_output = None

        for agent_name in agent_sequence:
            if agent_name not in self.agent_registry:
                logger.warning(f"Agent '{agent_name}' not registered, skipping")
                continue

            agent = self.agent_registry[agent_name]
            record = AgentExecutionRecord(
                agent_name=agent_name,
                chapter_id=agent_input.chapter_id,
                started_at=datetime.utcnow(),
            )

            try:
                logger.info(f"Invoking {agent_name} Agent for Chapter {agent_input.chapter_number}")

                # Pass previous output as context
                if previous_output:
                    agent_input.previous_artifacts = all_artifacts

                # Execute agent
                output = await agent.execute(agent_input)
                record.status = output.status
                record.completed_at = datetime.utcnow()
                record.duration_seconds = int((record.completed_at - record.started_at).total_seconds())
                record.output_artifacts = list(output.artifacts.keys())

                if output.status == AgentStatus.SUCCESS:
                    logger.info(f"{agent_name} Agent completed successfully")
                    all_artifacts.update(output.artifacts)
                    previous_output = output
                else:
                    logger.error(f"{agent_name} Agent failed: {output.error_message}")
                    record.error_message = output.error_message
                    # Continue with next agent (non-blocking failure)

            except Exception as e:
                logger.error(f"Error executing {agent_name} Agent: {str(e)}")
                record.status = AgentStatus.FAILURE
                record.error_message = str(e)
                record.completed_at = datetime.utcnow()
                record.duration_seconds = int((record.completed_at - record.started_at).total_seconds())

            self.execution_records.append(record)

        end_time = datetime.utcnow()
        total_duration = (end_time - start_time).total_seconds()

        logger.info(f"Orchestration completed for Chapter {agent_input.chapter_number} in {total_duration:.1f}s")

        # Determine overall status (all critical agents succeeded)
        critical_agents = {"Research", "Author", "Assessment", "QA"}
        critical_records = [r for r in self.execution_records if r.agent_name in critical_agents]
        all_succeeded = all(r.status == AgentStatus.SUCCESS for r in critical_records)

        return AgentOutput(
            status=AgentStatus.SUCCESS if all_succeeded else AgentStatus.FAILURE,
            artifacts=all_artifacts,
            duration_seconds=int(total_duration),
        )

    def get_execution_report(self) -> Dict:
        """Generate a report of the orchestration execution."""
        return {
            "total_agents": len(self.execution_records),
            "successful_agents": sum(1 for r in self.execution_records if r.status == AgentStatus.SUCCESS),
            "failed_agents": sum(1 for r in self.execution_records if r.status == AgentStatus.FAILURE),
            "total_duration_seconds": sum(r.duration_seconds or 0 for r in self.execution_records),
            "records": [r.to_dict() for r in self.execution_records],
        }
