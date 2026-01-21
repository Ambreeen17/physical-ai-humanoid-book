"""FastAPI Application for AI-Native Robotics Textbook

This is the main entry point for the backend service that manages:
- Learner profiles and personalization
- RAG-powered chatbot
- Assessment submission and grading
- Agent orchestration for chapter production
"""

from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging

from src.config import settings
from src.api.health import router as health_router
from src.api.auth import router as auth_router
from src.api.user_profiles import router as user_profiles_router
from src.api.onboarding import router as onboarding_router
from src.api.personalization import router as personalization_router
from src.api.learner_profile import router as learner_profile_router
from src.api.assessment import router as assessment_router
from src.api.chat import router as chat_router

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='{"timestamp": "%(asctime)s", "level": "%(levelname)s", "message": "%(message)s"}',
)
logger = logging.getLogger(__name__)


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Lifespan context manager for app startup and shutdown."""
    # Startup
    logger.info("Starting AI-Native Robotics Textbook Backend")
    from src.db.session import init_db
    await init_db()
    yield
    # Shutdown
    logger.info("Shutting down backend")


# Initialize FastAPI app
app = FastAPI(
    title="AI-Native Robotics Textbook",
    description="Backend service for orchestrated agent-driven content production",
    version="1.0.0",
    lifespan=lifespan,
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=settings.CORS_ORIGINS,
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include routers
app.include_router(health_router, prefix="/api/v1", tags=["health"])
app.include_router(auth_router, tags=["authentication"])
app.include_router(user_profiles_router, tags=["user-profiles"])
app.include_router(onboarding_router, tags=["onboarding"])
app.include_router(personalization_router, tags=["personalization"])
app.include_router(learner_profile_router, tags=["learner-profile"])
app.include_router(assessment_router, tags=["assessments"])
app.include_router(chat_router, tags=["chat"])


@app.get("/")
async def root():
    """Root endpoint - basic health check."""
    return {
        "service": "AI-Native Robotics Textbook Backend",
        "status": "operational",
        "version": "1.0.0",
    }


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(
        "src.main:app",
        host="0.0.0.0",
        port=8000,
        reload=True,
    )
