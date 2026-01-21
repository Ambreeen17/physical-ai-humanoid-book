"""User profile model - tracks user background, hardware access, and preferences."""

from sqlalchemy import Column, String, Boolean, DateTime, Text, Index
from datetime import datetime
import uuid

from src.db.base import Base


class UserProfile(Base):
    """User profile entity tracking background, hardware, and preferences."""

    __tablename__ = "user_profiles"

    # Primary Key
    id = Column(String(36), primary_key=True, default=lambda: str(uuid.uuid4()))
    email = Column(String(255), unique=True, nullable=False, index=True)

    # Software Background Levels
    programming_level = Column(
        String(20),
        nullable=False,
        default="beginner",
        comment="Level: beginner, intermediate, advanced"
    )
    ai_experience = Column(
        String(20),
        nullable=False,
        default="none",
        comment="Experience: none, basic, applied"
    )
    robotics_experience = Column(
        String(20),
        nullable=False,
        default="none",
        comment="Experience: none, basic, advanced"
    )

    # GPU/Hardware Access - NVIDIA RTX
    has_rtx = Column(Boolean, default=False, nullable=False)
    gpu_model = Column(
        String(100),
        nullable=True,
        comment="e.g., RTX 4090, RTX 3080, RTX 4080 Super"
    )
    rtx_vram_gb = Column(
        String(50),
        nullable=True,
        comment="GPU VRAM amount, e.g., 24GB, 48GB"
    )

    # Edge Computing - NVIDIA Jetson
    has_jetson = Column(Boolean, default=False, nullable=False)
    jetson_model = Column(
        String(100),
        nullable=True,
        comment="e.g., Jetson Orin Nano, Jetson Orin NX, Jetson AGX Orin"
    )

    # Physical Robotics Hardware
    has_robot = Column(Boolean, default=False, nullable=False)
    robot_type = Column(
        String(100),
        nullable=True,
        comment="e.g., Unitree G1, Boston Dynamics Atlas, Tesla Optimus, TurtleBot"
    )
    robot_specs = Column(
        Text,
        nullable=True,
        comment="Additional robot specs: {model, year, dof, weight_kg}"
    )

    # User Preferences
    preferred_language = Column(
        String(10),
        default="en",
        nullable=False,
        comment="Language code: en, ur, es, zh, etc."
    )
    learning_goal = Column(
        Text,
        nullable=True,
        comment="Free-text goal: e.g., 'Learn ROS 2 for quadruped control'"
    )

    # Content Preferences
    content_preferences = Column(
        Text,
        nullable=True,
        comment="User preferences: {theory_depth, simulation_focus, lab_difficulty}"
    )

    # Metadata
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    last_login = Column(DateTime, nullable=True)

    # Composite Indexes for efficient queries
    __table_args__ = (
        Index('idx_programming_level', 'programming_level'),
        Index('idx_robotics_experience', 'robotics_experience'),
        Index('idx_has_hardware', 'has_rtx', 'has_jetson', 'has_robot'),
    )

    def __repr__(self):
        return f"<UserProfile {self.email} ({self.programming_level}|{self.robotics_experience})>"

    # =====================================================================
    # Helper Methods
    # =====================================================================

    def get_difficulty_level(self) -> str:
        """
        Determine content difficulty level based on background.

        Returns:
            str: "beginner", "intermediate", or "advanced"
        """
        # Score programming level
        prog_score = {
            "beginner": 1,
            "intermediate": 2,
            "advanced": 3
        }.get(self.programming_level, 1)

        # Score AI experience
        ai_score = {
            "none": 0,
            "basic": 1,
            "applied": 2
        }.get(self.ai_experience, 0)

        # Score robotics experience
        robotics_score = {
            "none": 0,
            "basic": 1,
            "advanced": 2
        }.get(self.robotics_experience, 0)

        # Weighted average (programming 40%, robotics 40%, AI 20%)
        avg_score = (prog_score * 0.4 + robotics_score * 0.4 + ai_score * 0.2)

        if avg_score >= 2.2:
            return "advanced"
        elif avg_score >= 1.2:
            return "intermediate"
        else:
            return "beginner"

    def get_recommended_labs(self) -> list:
        """
        Recommend lab difficulty based on user profile.

        Returns:
            list: List of recommended lab difficulty levels
        """
        difficulty = self.get_difficulty_level()

        recommendations = {
            "beginner": ["beginner"],
            "intermediate": ["beginner", "intermediate"],
            "advanced": ["beginner", "intermediate", "advanced"]
        }

        return recommendations.get(difficulty, ["beginner"])

    def get_recommended_simulation_environment(self) -> str:
        """
        Recommend simulation environment based on hardware access.

        Returns:
            str: Recommended simulator ("gazebo", "isaac_sim", or "both")
        """
        # If user has GPU, recommend Isaac Sim for high-fidelity training
        if self.has_rtx and self.get_rtx_vram_gb() >= 24:
            return "isaac_sim"

        # If user has Jetson, recommend Gazebo for edge deployment
        if self.has_jetson:
            return "gazebo"

        # Default: Gazebo (lightweight, no GPU required)
        return "gazebo"

    def get_rtx_vram_gb(self) -> int:
        """
        Extract GPU VRAM as integer.

        Returns:
            int: GPU VRAM in GB, or 0 if not available
        """
        if not self.rtx_vram_gb:
            return 0

        try:
            # Extract number from strings like "24GB", "48 GB", etc.
            return int(''.join(filter(str.isdigit, self.rtx_vram_gb)))
        except (ValueError, TypeError):
            return 0

    def can_run_vla_training(self) -> bool:
        """
        Check if user has sufficient hardware for VLA (Vision-Language-Action) model training.

        Requires:
        - RTX GPU with >= 24GB VRAM, OR
        - Multiple GPUs

        Returns:
            bool: True if user can train VLAs
        """
        return self.has_rtx and self.get_rtx_vram_gb() >= 24

    def can_run_isaac_sim(self) -> bool:
        """
        Check if user has sufficient hardware for NVIDIA Isaac Sim.

        Requires:
        - RTX GPU (recommended >= 24GB for smooth simulation)

        Returns:
            bool: True if user can run Isaac Sim
        """
        return self.has_rtx and self.get_rtx_vram_gb() >= 12

    def has_edge_deployment_capability(self) -> bool:
        """
        Check if user can deploy models to edge devices (Jetson).

        Returns:
            bool: True if user has Jetson
        """
        return self.has_jetson

    def has_real_robot_access(self) -> bool:
        """
        Check if user has access to physical robotics hardware.

        Returns:
            bool: True if user has a robot
        """
        return self.has_robot

    def get_hardware_summary(self) -> dict:
        """
        Get a summary of user's hardware capabilities.

        Returns:
            dict: Hardware summary with keys: gpu, jetson, robot, simulation_capable
        """
        return {
            "gpu": {
                "available": self.has_rtx,
                "model": self.gpu_model,
                "vram_gb": self.get_rtx_vram_gb(),
                "can_vla_train": self.can_run_vla_training()
            },
            "jetson": {
                "available": self.has_jetson,
                "model": self.jetson_model,
                "edge_deployable": self.has_edge_deployment_capability()
            },
            "robot": {
                "available": self.has_robot,
                "type": self.robot_type,
                "specs": self.robot_specs or {}
            },
            "recommended_simulator": self.get_recommended_simulation_environment(),
            "lab_difficulties": self.get_recommended_labs()
        }

    def to_dict(self) -> dict:
        """
        Convert profile to dictionary.

        Returns:
            dict: User profile as dictionary
        """
        return {
            "id": str(self.id),
            "email": self.email,
            "programming_level": self.programming_level,
            "ai_experience": self.ai_experience,
            "robotics_experience": self.robotics_experience,
            "difficulty_level": self.get_difficulty_level(),
            "hardware": self.get_hardware_summary(),
            "preferred_language": self.preferred_language,
            "learning_goal": self.learning_goal,
            "created_at": self.created_at.isoformat() if self.created_at else None,
            "last_login": self.last_login.isoformat() if self.last_login else None
        }
