"""Onboarding questionnaire endpoints."""

import logging
from typing import Dict, List, Optional
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from datetime import datetime
import uuid

from src.db.session import get_db
from src.models.user_profile import UserProfile
from src.schemas.onboarding import (
    OnboardingQuestionnaireRequest,
    OnboardingQuestionnaireResponse,
    OnboardingRecommendations,
    JetsonModel,
    RobotType,
    LearningGoal,
)

logger = logging.getLogger(__name__)

router = APIRouter(
    prefix="/api/v1/onboarding",
    tags=["onboarding"],
)


# =====================================================================
# Helper Functions for Recommendations
# =====================================================================

def calculate_recommendations(
    programming_level: str,
    ai_experience: str,
    robotics_experience: str,
    has_rtx_gpu: bool,
    rtx_gpu_vram_gb: Optional[int],
    jetson_model: str,
    robot_type: str,
    learning_goal: str,
) -> OnboardingRecommendations:
    """
    Calculate personalized recommendations based on user profile.

    Args:
        programming_level: beginner, intermediate, advanced
        ai_experience: none, basic, applied
        robotics_experience: none, basic, advanced
        has_rtx_gpu: Whether user has RTX GPU
        rtx_gpu_vram_gb: GPU VRAM in GB
        jetson_model: Jetson hardware model
        robot_type: Physical robot type
        learning_goal: Learning goal

    Returns:
        OnboardingRecommendations: Personalized recommendations
    """

    # Score levels (for difficulty calculation)
    prog_score = {
        "beginner": 1,
        "intermediate": 2,
        "advanced": 3
    }.get(programming_level, 1)

    ai_score = {
        "none": 0,
        "basic": 1,
        "applied": 2
    }.get(ai_experience, 0)

    robotics_score = {
        "none": 0,
        "basic": 1,
        "advanced": 2
    }.get(robotics_experience, 0)

    # Weighted average: 40% programming + 40% robotics + 20% AI
    avg_score = (prog_score * 0.4 + robotics_score * 0.4 + ai_score * 0.2)

    # Determine difficulty level
    if avg_score >= 2.2:
        difficulty_level = "advanced"
    elif avg_score >= 1.2:
        difficulty_level = "intermediate"
    else:
        difficulty_level = "beginner"

    # Recommended labs based on difficulty
    recommended_labs = {
        "beginner": ["beginner"],
        "intermediate": ["beginner", "intermediate"],
        "advanced": ["beginner", "intermediate", "advanced"]
    }[difficulty_level]

    # Hardware capability checks
    vram_gb = rtx_gpu_vram_gb or 0
    can_train_vla = has_rtx_gpu and vram_gb >= 24
    can_use_isaac_sim = has_rtx_gpu and vram_gb >= 12
    can_deploy_to_jetson = jetson_model != "none"
    can_validate_on_real_robot = robot_type != "none"

    # Recommend simulator based on hardware
    if can_use_isaac_sim:
        recommended_simulator = "isaac_sim"
    else:
        recommended_simulator = "gazebo"

    # Generate warnings
    warnings = []

    if has_rtx_gpu and vram_gb < 24:
        warnings.append(
            f"RTX GPU has only {vram_gb}GB VRAM; "
            "VLA model training requires 24GB+ (available on RTX 4090, 6000 series)"
        )

    if has_rtx_gpu and vram_gb < 12:
        warnings.append(
            f"RTX GPU has only {vram_gb}GB VRAM; "
            "Isaac Sim simulation requires 12GB+ VRAM"
        )

    if not has_rtx_gpu:
        warnings.append(
            "No RTX GPU detected; using Gazebo for simulation "
            "(lighter weight, good for learning, but lower fidelity)"
        )

    if not can_deploy_to_jetson:
        warnings.append(
            "No Jetson hardware detected; edge deployment labs will be simulation-only"
        )

    if not can_validate_on_real_robot:
        warnings.append(
            "No physical robot detected; validation will be limited to simulation"
        )

    # Suggest learning path based on goal
    learning_path = _suggest_learning_path(
        learning_goal,
        difficulty_level,
        can_train_vla,
        can_deploy_to_jetson,
        can_validate_on_real_robot
    )

    return OnboardingRecommendations(
        difficulty_level=difficulty_level,
        recommended_labs=recommended_labs,
        recommended_simulator=recommended_simulator,
        can_train_vla_models=can_train_vla,
        can_use_isaac_sim=can_use_isaac_sim,
        can_deploy_to_jetson=can_deploy_to_jetson,
        can_validate_on_real_robot=can_validate_on_real_robot,
        suggested_learning_path=learning_path,
        warnings=warnings
    )


def _suggest_learning_path(
    learning_goal: str,
    difficulty_level: str,
    can_train_vla: bool,
    can_deploy_to_jetson: bool,
    can_validate_on_real_robot: bool
) -> List[str]:
    """
    Suggest learning chapter sequence based on goal and capabilities.

    Args:
        learning_goal: "concepts", "simulations", or "real_robots"
        difficulty_level: "beginner", "intermediate", or "advanced"
        can_train_vla: Whether user can train VLA models
        can_deploy_to_jetson: Whether user can deploy to Jetson
        can_validate_on_real_robot: Whether user has real robot

    Returns:
        List[str]: Suggested chapter sequence
    """

    # Core chapters (always included)
    core_chapters = [
        "Chapter 1: Introduction to Physical AI",
        "Chapter 2: Sensors & Perception",
        "Chapter 3: Control Systems"
    ]

    # Path for concept learners
    if learning_goal == "concepts":
        path = core_chapters + [
            "Chapter 4: Simulation Fundamentals",
            "Chapter 5: Learning from Simulation",
            "Chapter 6: Real-World Constraints"
        ]

    # Path for simulation builders
    elif learning_goal == "simulations":
        path = core_chapters + [
            "Chapter 4: Simulation Fundamentals",
            "Chapter 7: ROS 2 Advanced (for simulation)",
            "Chapter 8: Gazebo for Roboticists",
        ]
        if can_train_vla:
            path.append("Chapter 9: Vision-Language-Action Models")

    # Path for real robot deployment
    else:  # real_robots
        path = core_chapters + [
            "Chapter 4: Simulation Fundamentals",
            "Chapter 7: ROS 2 Advanced (for deployment)",
            "Chapter 10: From Simulation to Reality"
        ]
        if can_validate_on_real_robot:
            path.append("Chapter 11: Real Robot Integration")
        if can_deploy_to_jetson:
            path.append("Chapter 12: Edge Deployment (Jetson)")
        if can_train_vla:
            path.append("Chapter 13: Learning-Based Control")

    # Advanced extensions for advanced learners
    if difficulty_level == "advanced":
        path.extend([
            "Chapter 14: Multi-Robot Systems",
            "Chapter 15: Advanced Control Theory"
        ])

    return path


# =====================================================================
# API Endpoints
# =====================================================================

@router.post(
    "/questionnaire",
    response_model=OnboardingQuestionnaireResponse,
    status_code=status.HTTP_201_CREATED
)
async def complete_onboarding_questionnaire(
    questionnaire: OnboardingQuestionnaireRequest,
    db: Session = Depends(get_db)
):
    """
    Complete onboarding questionnaire and create user profile.

    This endpoint:
    1. Validates questionnaire responses
    2. Creates or updates user profile
    3. Calculates personalized recommendations
    4. Returns next steps and suggested learning path
    """

    try:
        # Check if user already exists
        existing_user = db.query(UserProfile).filter(
            UserProfile.email == questionnaire.email
        ).first()

        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"User with email {questionnaire.email} already exists"
            )

        # Calculate recommendations
        recommendations = calculate_recommendations(
            programming_level=questionnaire.programming_level.value,
            ai_experience=questionnaire.ai_experience.value,
            robotics_experience=questionnaire.robotics_experience.value,
            has_rtx_gpu=questionnaire.has_rtx_gpu,
            rtx_gpu_vram_gb=questionnaire.rtx_gpu_vram_gb,
            jetson_model=questionnaire.jetson_model.value,
            robot_type=questionnaire.robot_type.value,
            learning_goal=questionnaire.learning_goal.value,
        )

        # Create user profile
        user_profile = UserProfile(
            email=questionnaire.email,
            programming_level=questionnaire.programming_level.value,
            ai_experience=questionnaire.ai_experience.value,
            robotics_experience=questionnaire.robotics_experience.value,
            has_rtx=questionnaire.has_rtx_gpu,
            gpu_model=questionnaire.rtx_gpu_model.value if questionnaire.rtx_gpu_model else None,
            rtx_vram_gb=f"{questionnaire.rtx_gpu_vram_gb}GB" if questionnaire.rtx_gpu_vram_gb else None,
            has_jetson=questionnaire.jetson_model.value != "none",
            jetson_model=questionnaire.jetson_model.value if questionnaire.jetson_model.value != "none" else None,
            has_robot=questionnaire.robot_type.value != "none",
            robot_type=questionnaire.robot_type.value if questionnaire.robot_type.value != "none" else None,
            preferred_language="en",
            learning_goal=questionnaire.learning_goal.value,
            content_preferences={
                "learning_goal": questionnaire.learning_goal.value,
                "additional_interests": questionnaire.additional_interests or [],
                "created_at": datetime.utcnow().isoformat()
            }
        )

        db.add(user_profile)
        db.commit()
        db.refresh(user_profile)

        logger.info(f"Created user profile via onboarding: {questionnaire.email}")

        return OnboardingQuestionnaireResponse(
            user_id=str(user_profile.id),
            email=user_profile.email,
            first_name=questionnaire.first_name,
            last_name=questionnaire.last_name,
            status="completed",
            programming_level=user_profile.programming_level,
            ai_experience=user_profile.ai_experience,
            robotics_experience=user_profile.robotics_experience,
            difficulty_level=user_profile.get_difficulty_level(),
            has_rtx_gpu=user_profile.has_rtx,
            rtx_gpu_model=user_profile.gpu_model,
            rtx_gpu_vram_gb=user_profile.get_rtx_vram_gb(),
            jetson_model=user_profile.jetson_model or "none",
            robot_type=user_profile.robot_type or "none",
            robot_model=questionnaire.robot_model,
            learning_goal=user_profile.learning_goal,
            recommendations=recommendations.model_dump()
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to complete onboarding: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to complete onboarding: {str(e)}"
        )


@router.post("/submit", status_code=status.HTTP_201_CREATED)
async def simple_registration(
    data: Dict,
    db: Session = Depends(get_db)
):
    """
    Simple registration endpoint for basic signup form.
    Creates a user profile with minimal information.
    """
    import jwt
    from datetime import datetime, timedelta

    try:
        email = data.get("email")
        if not email:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail="Email is required"
            )

        # Check if user already exists
        existing_user = db.query(UserProfile).filter(
            UserProfile.email == email
        ).first()

        if existing_user:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"User with email {email} already exists"
            )

        # Create user profile with defaults
        user_profile = UserProfile(
            email=email,
            programming_level=data.get("programming_level", "beginner"),
            ai_experience=data.get("ai_experience", "none"),
            robotics_experience=data.get("robotics_experience", "none"),
            has_rtx=data.get("gpu_available", False),
            has_jetson=False,
            has_robot=data.get("has_robot", False),
            preferred_language="en",
            learning_goal=data.get("learning_goal", "career"),
        )

        db.add(user_profile)
        db.commit()
        db.refresh(user_profile)

        # Generate JWT token
        secret_key = "textbook-secret-key-change-in-production"
        token_data = {
            "sub": str(user_profile.id),
            "email": user_profile.email,
            "exp": datetime.utcnow() + timedelta(days=30)
        }
        auth_token = jwt.encode(token_data, secret_key, algorithm="HS256")

        logger.info(f"Created user profile via simple registration: {email}")

        return {
            "user_id": str(user_profile.id),
            "email": user_profile.email,
            "auth_token": auth_token,
            "status": "registered"
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to register user: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to register: {str(e)}"
        )


@router.get("/form-options")
async def get_form_options():
    """
    Get all options for onboarding form (for frontend rendering).

    Returns form field options for dynamic UI generation.
    """
    return {
        "programming_level": [
            {
                "value": "beginner",
                "label": "Beginner",
                "description": "New to programming or robotics"
            },
            {
                "value": "intermediate",
                "label": "Intermediate",
                "description": "Comfortable with code and basic concepts"
            },
            {
                "value": "advanced",
                "label": "Advanced",
                "description": "Strong programming and system design skills"
            }
        ],
        "ai_experience": [
            {
                "value": "none",
                "label": "None",
                "description": "No prior AI/ML experience"
            },
            {
                "value": "basic",
                "label": "Basic (LLMs, APIs)",
                "description": "Familiar with ChatGPT, GPT APIs, or similar tools"
            },
            {
                "value": "applied",
                "label": "Applied (agents, RAG, robotics)",
                "description": "Built AI agents, RAG systems, or robotics projects"
            }
        ],
        "robotics_experience": [
            {
                "value": "none",
                "label": "None",
                "description": "No prior robotics experience"
            },
            {
                "value": "basic",
                "label": "Basic (ROS concepts)",
                "description": "Familiar with ROS concepts and basic robot control"
            },
            {
                "value": "advanced",
                "label": "Advanced (controllers, kinematics)",
                "description": "Experienced with controllers, kinematics, or robot design"
            }
        ],
        "gpu": [
            {
                "value": False,
                "label": "No",
                "description": "No RTX GPU available"
            },
            {
                "value": True,
                "label": "Yes",
                "description": "I have an RTX GPU",
                "followup_field": "rtx_gpu_model"
            }
        ],
        "rtx_gpu_model": [
            {
                "value": "RTX 3080",
                "label": "RTX 3080 (10GB)",
                "vram": 10
            },
            {
                "value": "RTX 3090",
                "label": "RTX 3090 (24GB)",
                "vram": 24
            },
            {
                "value": "RTX 4070",
                "label": "RTX 4070 (12GB)",
                "vram": 12
            },
            {
                "value": "RTX 4080",
                "label": "RTX 4080 (16GB)",
                "vram": 16
            },
            {
                "value": "RTX 4090",
                "label": "RTX 4090 (24GB)",
                "vram": 24
            },
            {
                "value": "other",
                "label": "Other (please specify VRAM)",
                "vram": None
            }
        ],
        "jetson_model": [
            {
                "value": "none",
                "label": "None",
                "description": "No Jetson hardware"
            },
            {
                "value": "orin_nano",
                "label": "Jetson Orin Nano",
                "description": "Entry-level edge AI (8GB RAM, ~40 TFLOPS)"
            },
            {
                "value": "orin_nx",
                "label": "Jetson Orin NX",
                "description": "Mid-range edge AI (8GB RAM, ~100 TFLOPS)"
            },
            {
                "value": "orin_agx",
                "label": "Jetson AGX Orin",
                "description": "High-end edge AI (64GB/96GB RAM, 1.5 PFLOPS)"
            }
        ],
        "robot_type": [
            {
                "value": "none",
                "label": "None",
                "description": "No physical robot"
            },
            {
                "value": "quadruped",
                "label": "Quadruped",
                "description": "4-legged robot (e.g., Unitree G1, Boston Dynamics Spot)"
            },
            {
                "value": "humanoid",
                "label": "Humanoid",
                "description": "Human-shaped robot (e.g., Tesla Optimus, Boston Dynamics Atlas)"
            },
            {
                "value": "robotic_arm",
                "label": "Robotic Arm",
                "description": "Articulated arm (e.g., UR10e, ABB IRB, Franka Emika)"
            }
        ],
        "learning_goal": [
            {
                "value": "concepts",
                "label": "Learn core concepts",
                "description": "Understand embodied AI, control theory, and robotics fundamentals"
            },
            {
                "value": "simulations",
                "label": "Build simulations",
                "description": "Create and test algorithms in Gazebo or Isaac Sim"
            },
            {
                "value": "real_robots",
                "label": "Deploy on real robots",
                "description": "Get algorithms running on physical hardware"
            }
        ]
    }


@router.get("/recommendations/{user_id}", response_model=OnboardingRecommendations)
async def get_user_recommendations(
    user_id: str,
    db: Session = Depends(get_db)
):
    """
    Get or recalculate personalized recommendations for a user.
    """
    try:
        user = db.query(UserProfile).filter(
            UserProfile.id == uuid.UUID(user_id)
        ).first()

        if not user:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"User {user_id} not found"
            )

        recommendations = calculate_recommendations(
            programming_level=user.programming_level,
            ai_experience=user.ai_experience,
            robotics_experience=user.robotics_experience,
            has_rtx_gpu=user.has_rtx,
            rtx_gpu_vram_gb=user.get_rtx_vram_gb(),
            jetson_model=user.jetson_model or "none",
            robot_type=user.robot_type or "none",
            learning_goal=user.learning_goal or "concepts"
        )

        return recommendations

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Failed to get recommendations: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get recommendations: {str(e)}"
        )
