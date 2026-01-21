"""Configuration management for the textbook backend.

Uses environment variables and .env file for sensitive configuration.
All settings validated with Pydantic.
"""

from pydantic_settings import BaseSettings
from typing import List


class Settings(BaseSettings):
    """Application settings from environment variables."""

    # Application
    APP_NAME: str = "AI-Native Robotics Textbook"
    DEBUG: bool = False
    VERSION: str = "1.0.0"

    # Database
    DATABASE_URL: str = "postgresql://user:password@localhost:5432/textbook_db"
    DB_ECHO: bool = False

    # External APIs
    OPENAI_API_KEY: str = ""
    CLAUDE_API_KEY: str = ""

    # Qdrant (RAG)
    QDRANT_URL: str = "http://localhost:6333"
    QDRANT_API_KEY: str = ""

    # CORS
    CORS_ORIGINS: List[str] = [
        "http://localhost:3000",
        "http://localhost:8000",
        "http://localhost:8080",
    ]

    # File paths
    CHAPTERS_DIR: str = "./specs/1-book-curriculum/chapters"
    ARTIFACTS_DIR: str = "./artifacts"

    class Config:
        env_file = ".env"
        env_file_encoding = "utf-8"
        case_sensitive = True


settings = Settings()
