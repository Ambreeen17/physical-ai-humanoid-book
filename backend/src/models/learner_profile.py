"""Learner profile model - tracks learner background and preferences."""

from sqlalchemy import Column, String, DateTime, Integer, Text
from datetime import datetime
import uuid
import json

from src.db.base import Base


class LearnerProfile(Base):
    """Learner profile entity for tracking learner skills and recommendations."""

    __tablename__ = "learner_profiles"

    id = Column(Integer, primary_key=True, autoincrement=True)
    learner_id = Column(String(255), unique=True, nullable=False, index=True)
    python_score = Column(Integer, default=0)  # 0-10 scale
    ml_score = Column(Integer, default=0)
    robotics_score = Column(Integer, default=0)
    ros_score = Column(Integer, default=0)
    _recommended_chapters = Column("recommended_chapters", Text, nullable=True)  # JSON array stored as text
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)

    def __repr__(self):
        return f"<LearnerProfile {self.learner_id}>"

    @property
    def difficulty_level(self) -> str:
        """Calculate difficulty level based on weighted scores."""
        # Weighted average: Python 30%, ML 25%, Robotics 25%, ROS 20%
        weighted = (
            self.python_score * 0.30 +
            self.ml_score * 0.25 +
            self.robotics_score * 0.25 +
            self.ros_score * 0.20
        )
        if weighted >= 7:
            return "Advanced"
        elif weighted >= 4:
            return "Intermediate"
        else:
            return "Beginner"

    @property
    def recommended_chapters(self) -> list:
        """Get recommended chapters based on difficulty level."""
        if self._recommended_chapters:
            try:
                return json.loads(self._recommended_chapters)
            except json.JSONDecodeError:
                pass

        # Default recommendations based on difficulty
        level = self.difficulty_level
        if level == "Beginner":
            return [
                "Chapter 1: Introduction to Physical AI",
                "Chapter 2: Sensors & Perception",
                "Chapter 3: Control Systems"
            ]
        elif level == "Intermediate":
            return [
                "Chapter 4: Motion Planning",
                "Chapter 5: Computer Vision",
                "Chapter 6: Machine Learning for Robotics"
            ]
        else:
            return [
                "Chapter 10: Advanced Control",
                "Chapter 14: Sim-to-Real Transfer",
                "Chapter 16: Capstone Project"
            ]

    @recommended_chapters.setter
    def recommended_chapters(self, value: list):
        """Set recommended chapters."""
        self._recommended_chapters = json.dumps(value) if value else None
