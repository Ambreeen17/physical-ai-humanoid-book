"""Chapter model - represents a textbook chapter."""

from sqlalchemy import Column, String, Integer, Text, DateTime, Enum
from datetime import datetime
import uuid
import enum

from src.db.base import Base


class ChapterStatus(str, enum.Enum):
    """Chapter publication status."""
    DRAFT = "draft"
    RESEARCH = "research"
    IN_PROGRESS = "in_progress"
    REVIEW = "review"
    PUBLISHED = "published"


class Chapter(Base):
    """Chapter entity."""

    __tablename__ = "chapters"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    number = Column(Integer, unique=True, nullable=False, index=True)
    title = Column(String(255), nullable=False)
    topic = Column(String(255), nullable=False)
    learning_objectives = Column(Text, nullable=True)  # JSON array stored as text
    prerequisites = Column(Text, nullable=True)  # JSON array stored as text
    content = Column(Text, nullable=True)
    status = Column(Enum(ChapterStatus), default=ChapterStatus.DRAFT, index=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    def __repr__(self):
        return f"<Chapter {self.number}: {self.title}>"
