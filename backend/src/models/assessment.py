"""Assessment model - represents quizzes and exercises."""

from sqlalchemy import Column, String, Integer, Text, DateTime, ForeignKey, Enum
from datetime import datetime
import uuid
import enum

from src.db.base import Base


class AssessmentType(str, enum.Enum):
    """Assessment type."""
    QUIZ = "quiz"
    LAB_EXERCISE = "lab_exercise"
    CAPSTONE_CHALLENGE = "capstone_challenge"


class Difficulty(str, enum.Enum):
    """Assessment difficulty."""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class Assessment(Base):
    """Assessment entity."""

    __tablename__ = "assessments"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    chapter_id = Column(String(36), ForeignKey("chapters.id"), nullable=False, index=True)
    type = Column(Enum(AssessmentType), nullable=False)
    questions = Column(Text, nullable=True)  # JSON stored as text
    rubric = Column(Text, nullable=True)  # JSON stored as text
    difficulty = Column(Enum(Difficulty), nullable=True)
    time_limit = Column(Integer, nullable=True)  # minutes
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    def __repr__(self):
        return f"<Assessment {self.type} for Chapter {self.chapter_id}>"
