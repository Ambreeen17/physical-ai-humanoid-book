"""Pydantic schemas for user onboarding questionnaire."""

from pydantic import BaseModel, EmailStr, Field
from typing import Optional, List
from enum import Enum


# =====================================================================
# Enum Definitions (for form options)
# =====================================================================

class ProgrammingLevel(str, Enum):
    """Programming experience level."""

    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

    @classmethod
    def to_list(cls):
        return [e.value for e in cls]

    @classmethod
    def to_display(cls, value: str) -> str:
        """Convert enum value to display label."""
        return {
            "beginner": "Beginner",
            "intermediate": "Intermediate",
            "advanced": "Advanced"
        }.get(value, value)


class AIExperience(str, Enum):
    """AI/ML experience level."""

    NONE = "none"
    BASIC = "basic"  # LLMs, APIs
    APPLIED = "applied"  # Agents, RAG, robotics

    @classmethod
    def to_list(cls):
        return [e.value for e in cls]

    @classmethod
    def to_display(cls, value: str) -> str:
        """Convert enum value to display label."""
        return {
            "none": "None",
            "basic": "Basic (LLMs, APIs)",
            "applied": "Applied (agents, RAG, robotics)"
        }.get(value, value)

    @classmethod
    def get_description(cls, value: str) -> str:
        """Get description for enum value."""
        return {
            "none": "No prior AI/ML experience",
            "basic": "Familiar with LLMs and APIs (ChatGPT, OpenAI)",
            "applied": "Built agents, RAG systems, or robotics projects"
        }.get(value, "")


class RoboticsExperience(str, Enum):
    """Robotics experience level."""

    NONE = "none"
    BASIC = "basic"  # ROS concepts
    ADVANCED = "advanced"  # Controllers, kinematics

    @classmethod
    def to_list(cls):
        return [e.value for e in cls]

    @classmethod
    def to_display(cls, value: str) -> str:
        """Convert enum value to display label."""
        return {
            "none": "None",
            "basic": "Basic (ROS concepts)",
            "advanced": "Advanced (controllers, kinematics)"
        }.get(value, value)

    @classmethod
    def get_description(cls, value: str) -> str:
        """Get description for enum value."""
        return {
            "none": "No prior robotics experience",
            "basic": "Familiar with ROS concepts and basic robot control",
            "advanced": "Experienced with controllers, kinematics, and robot design"
        }.get(value, "")


class GPUModel(str, Enum):
    """NVIDIA RTX GPU models."""

    RTX_3080 = "RTX 3080"
    RTX_3090 = "RTX 3090"
    RTX_4070 = "RTX 4070"
    RTX_4080 = "RTX 4080"
    RTX_4090 = "RTX 4090"
    OTHER = "other"

    @classmethod
    def to_list(cls):
        return [e.value for e in cls]


class JetsonModel(str, Enum):
    """NVIDIA Jetson hardware models."""

    NONE = "none"
    ORIN_NANO = "orin_nano"
    ORIN_NX = "orin_nx"
    ORIN_AGX = "orin_agx"

    @classmethod
    def to_list(cls):
        return [e.value for e in cls]

    @classmethod
    def to_display(cls, value: str) -> str:
        """Convert enum value to display label."""
        return {
            "none": "None",
            "orin_nano": "Jetson Orin Nano",
            "orin_nx": "Jetson Orin NX",
            "orin_agx": "Jetson AGX Orin"
        }.get(value, value)


class RobotType(str, Enum):
    """Physical robot types."""

    NONE = "none"
    QUADRUPED = "quadruped"
    HUMANOID = "humanoid"
    ROBOTIC_ARM = "robotic_arm"

    @classmethod
    def to_list(cls):
        return [e.value for e in cls]

    @classmethod
    def to_display(cls, value: str) -> str:
        """Convert enum value to display label."""
        return {
            "none": "None",
            "quadruped": "Quadruped (e.g., Unitree G1, Boston Dynamics Spot)",
            "humanoid": "Humanoid (e.g., Tesla Optimus, Boston Dynamics Atlas)",
            "robotic_arm": "Robotic Arm (e.g., UR, ABB, Franka)"
        }.get(value, value)


class LearningGoal(str, Enum):
    """Primary learning goals."""

    CONCEPTS = "concepts"
    SIMULATIONS = "simulations"
    REAL_ROBOTS = "real_robots"

    @classmethod
    def to_list(cls):
        return [e.value for e in cls]

    @classmethod
    def to_display(cls, value: str) -> str:
        """Convert enum value to display label."""
        return {
            "concepts": "Learn core concepts",
            "simulations": "Build simulations (Gazebo/Isaac)",
            "real_robots": "Deploy on real robots"
        }.get(value, value)

    @classmethod
    def get_description(cls, value: str) -> str:
        """Get description for enum value."""
        return {
            "concepts": "Understand embodied AI, control theory, and robotics fundamentals",
            "simulations": "Build and test algorithms in simulated environments",
            "real_robots": "Deploy trained controllers to physical hardware"
        }.get(value, "")


# =====================================================================
# Onboarding Request/Response Schemas
# =====================================================================

class OnboardingQuestionnaireRequest(BaseModel):
    """User onboarding questionnaire data."""

    email: EmailStr
    first_name: Optional[str] = None
    last_name: Optional[str] = None

    # Question 1: Programming Level
    programming_level: ProgrammingLevel = Field(
        ...,
        description="Programming experience level"
    )

    # Question 2: AI Experience
    ai_experience: AIExperience = Field(
        ...,
        description="AI/ML experience level"
    )

    # Question 3: Robotics Experience
    robotics_experience: RoboticsExperience = Field(
        ...,
        description="Robotics experience level"
    )

    # Question 4: RTX GPU
    has_rtx_gpu: bool = Field(
        default=False,
        description="Do you have an RTX GPU?"
    )
    rtx_gpu_model: Optional[GPUModel] = Field(
        default=None,
        description="RTX GPU model (if available)"
    )
    rtx_gpu_vram_gb: Optional[int] = Field(
        default=None,
        description="GPU VRAM in GB",
        ge=4,
        le=192
    )

    # Question 5: Jetson Hardware
    jetson_model: JetsonModel = Field(
        default=JetsonModel.NONE,
        description="Jetson hardware model"
    )

    # Question 6: Physical Robot
    robot_type: RobotType = Field(
        default=RobotType.NONE,
        description="Physical robot type"
    )
    robot_model: Optional[str] = Field(
        default=None,
        description="Specific robot model (e.g., Unitree G1, UR10e)"
    )

    # Question 7: Primary Goal
    learning_goal: LearningGoal = Field(
        ...,
        description="Primary learning goal"
    )

    # Optional fields for future extensibility
    additional_interests: Optional[List[str]] = Field(
        default=None,
        description="Additional interests or topics"
    )
    comments: Optional[str] = Field(
        default=None,
        description="Any additional comments or questions"
    )

    class Config:
        example = {
            "email": "learner@example.com",
            "first_name": "Alex",
            "last_name": "Chen",
            "programming_level": "intermediate",
            "ai_experience": "basic",
            "robotics_experience": "none",
            "has_rtx_gpu": True,
            "rtx_gpu_model": "RTX 4080",
            "rtx_gpu_vram_gb": 16,
            "jetson_model": "none",
            "robot_type": "none",
            "learning_goal": "simulations",
            "additional_interests": ["RL", "VLA models"],
            "comments": "Interested in humanoid robotics research"
        }


class OnboardingQuestionnaireResponse(BaseModel):
    """Response after completing onboarding questionnaire."""

    user_id: str
    email: str
    first_name: Optional[str]
    last_name: Optional[str]
    status: str  # "completed"

    # User profile data
    programming_level: str
    ai_experience: str
    robotics_experience: str
    difficulty_level: str  # Calculated

    # Hardware
    has_rtx_gpu: bool
    rtx_gpu_model: Optional[str]
    rtx_gpu_vram_gb: Optional[int]
    jetson_model: str
    robot_type: str
    robot_model: Optional[str]

    # Learning goal
    learning_goal: str

    # Personalization recommendations
    recommendations: dict = Field(
        description="Personalized recommendations based on profile"
    )

    class Config:
        from_attributes = True


class OnboardingRecommendations(BaseModel):
    """Personalized recommendations after onboarding."""

    difficulty_level: str = Field(
        description="Recommended content difficulty: beginner, intermediate, advanced"
    )
    recommended_labs: List[str] = Field(
        description="Lab difficulty levels learner should attempt"
    )
    recommended_simulator: str = Field(
        description="Primary simulator: gazebo or isaac_sim"
    )
    can_train_vla_models: bool = Field(
        description="Whether user can train VLA models (RTX 24GB+)"
    )
    can_use_isaac_sim: bool = Field(
        description="Whether user can run Isaac Sim"
    )
    can_deploy_to_jetson: bool = Field(
        description="Whether user can deploy to Jetson edge devices"
    )
    can_validate_on_real_robot: bool = Field(
        description="Whether user has real robot for validation"
    )
    suggested_learning_path: List[str] = Field(
        description="Suggested chapter sequence based on goals and background"
    )
    warnings: List[str] = Field(
        default=[],
        description="Any limitations or warnings (e.g., no GPU)"
    )

    class Config:
        example = {
            "difficulty_level": "intermediate",
            "recommended_labs": ["beginner", "intermediate"],
            "recommended_simulator": "isaac_sim",
            "can_train_vla_models": False,
            "can_use_isaac_sim": True,
            "can_deploy_to_jetson": False,
            "can_validate_on_real_robot": False,
            "suggested_learning_path": [
                "Chapter 1: Introduction to Physical AI",
                "Chapter 2: Sensors & Perception",
                "Chapter 3: Control Systems",
                "Chapter 4: Simulation & Deployment"
            ],
            "warnings": [
                "RTX GPU has only 16GB VRAM; VLA training requires 24GB+",
                "No Jetson device; edge deployment labs will be simulation-only"
            ]
        }
