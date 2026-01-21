"""Agents package initialization - registers all agents with the orchestrator."""

from .orchestrator import BookOrchestrator
from .research_agent import ResearchAgent
from .author_agent import AuthorAgent
from .diagram_agent import DiagramAgent
from .lab_agent import LabAgent
from .assessment_agent import AssessmentAgent
from .personalization_agent import PersonalizationAgent
from .localization_agent import LocalizationAgent
from .rag_indexing_agent import RAGIndexingAgent
from .qa_agent import QAAgent


def initialize_agents():
    """Initialize and register all agents with the orchestrator."""
    orchestrator = BookOrchestrator()
    
    # Create and register all agents
    agents = {
        "Research": ResearchAgent(),
        "Author": AuthorAgent(),
        "Diagram": DiagramAgent(),
        "Lab": LabAgent(),
        "Assessment": AssessmentAgent(),
        "Personalization": PersonalizationAgent(),
        "Localization": LocalizationAgent(),
        "RAGIndexing": RAGIndexingAgent(),
        "QA": QAAgent()
    }
    
    for name, agent in agents.items():
        orchestrator.register_agent(name, agent)
    
    return orchestrator