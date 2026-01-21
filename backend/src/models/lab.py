"""Lab model - represents hands-on lab exercises."""

from sqlalchemy import Column, String, Integer, Text, DateTime, ForeignKey, Enum
from datetime import datetime
import uuid
import enum

from src.db.base import Base


class Difficulty(str, enum.Enum):
    """Lab difficulty level."""
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class LabStatus(str, enum.Enum):
    """Lab test status."""
    PENDING = "pending"
    PASS = "pass"
    FAIL = "fail"


class Lab(Base):
    """Lab entity."""

    __tablename__ = "labs"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    chapter_id = Column(String(36), ForeignKey("chapters.id"), nullable=False, index=True)
    title = Column(String(255), nullable=False)
    description = Column(Text, nullable=True)
    difficulty = Column(Enum(Difficulty), nullable=True)
    estimated_duration = Column(Integer, nullable=True)  # minutes
    tools = Column(Text, nullable=True)  # JSON array stored as text
    docker_image = Column(String(255), nullable=True)
    source_code_repo = Column(String(255), nullable=True)
    expected_output = Column(Text, nullable=True)
    test_status = Column(Enum(LabStatus), default=LabStatus.PENDING)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    def __repr__(self):
        return f"<Lab {self.title}>"
