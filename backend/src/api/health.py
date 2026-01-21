"""Health check endpoints for monitoring and debugging."""

from fastapi import APIRouter, status
from datetime import datetime

router = APIRouter()


@router.get("/health", status_code=status.HTTP_200_OK)
async def health_check():
    """Health check endpoint for monitoring."""
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "service": "AI-Native Robotics Textbook Backend",
    }


@router.get("/health/detailed", status_code=status.HTTP_200_OK)
async def detailed_health_check():
    """Detailed health check including dependencies."""
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "service": "AI-Native Robotics Textbook Backend",
        "checks": {
            "database": "pending",  # TODO: Implement database connection check
            "qdrant": "pending",    # TODO: Implement Qdrant connection check
            "llm_api": "pending",   # TODO: Implement LLM API check
        },
    }
