"""Assessment result model - tracks learner assessment submissions."""

from sqlalchemy import Column, String, DateTime, ForeignKey, Text, Float
from datetime import datetime
import uuid

from src.db.base import Base


class AssessmentResult(Base):
    """Assessment result entity."""

    __tablename__ = "assessment_results"

    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    learner_id = Column(String(36), ForeignKey("learner_profiles.id"), nullable=False)
    chapter_id = Column(String(36), ForeignKey("chapters.id"), nullable=False)
    assessment_id = Column(String(36), ForeignKey("assessments.id"), nullable=False)
    submission_date = Column(DateTime, default=datetime.utcnow)
    answers = Column(Text, nullable=True)  # JSON stored as text
    score = Column(Float, nullable=True)
    feedback = Column(Text, nullable=True)
    status = Column(String(50), default="in_progress")  # in_progress, pass, fail
    lab_output_url = Column(String(255), nullable=True)

    def __repr__(self):
        return f"<AssessmentResult {self.learner_id} - Chapter {self.chapter_id}>"
