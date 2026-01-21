"""Personalization API endpoints."""

import logging
from typing import Optional
from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel, Field
from sqlalchemy.orm import Session

from src.db.session import get_db
from src.auth.dependencies import get_current_user
from src.models.user_profile import UserProfile
from src.services.personalization_service import PersonalizationService

logger = logging.getLogger(__name__)

router = APIRouter(
    prefix="/api/v1/personalization",
    tags=["personalization"],
)

# Initialize personalization service
personalization_service = PersonalizationService()


# =====================================================================
# Pydantic Models
# =====================================================================

class ContentSectionRequest(BaseModel):
    """Raw content section."""

    id: str
    title: str
    content: str
    difficulty: str = "intermediate"
    type: str = "theory"  # theory, math, code, example, challenge
    prerequisites: Optional[list] = None
    read_time_minutes: int = 5


class ChapterRequest(BaseModel):
    """Chapter data for personalization."""

    id: int
    title: str
    sections: list[ContentSectionRequest]

    class Config:
        example = {
            "id": 1,
            "title": "Introduction to Physical AI",
            "sections": [
                {
                    "id": "intro",
                    "title": "Introduction",
                    "content": "Physical AI is...",
                    "difficulty": "beginner",
                    "type": "theory"
                },
                {
                    "id": "math",
                    "title": "Mathematics",
                    "content": "∇f = ...",
                    "difficulty": "advanced",
                    "type": "math"
                }
            ]
        }


class ContentTransformationResponse(BaseModel):
    """Content transformation that was applied."""

    type: str
    section_id: str
    reason: str


class PersonalizedSectionResponse(BaseModel):
    """Personalized content section."""

    id: str
    title: str
    content: str
    difficulty: str
    section_type: str
    estimated_read_time_minutes: int


class PersonalizedChapterResponse(BaseModel):
    """Fully personalized chapter."""

    chapter_id: int
    chapter_title: str
    user_difficulty_level: str
    sections: list[PersonalizedSectionResponse]
    transformations: list[ContentTransformationResponse]
    total_estimated_read_time: int
    recommended_simulator: str
    recommended_labs: list[str]
    warnings: list[str]

    class Config:
        from_attributes = True


class PersonalizationStatsResponse(BaseModel):
    """Statistics about personalization applied."""

    total_sections: int
    total_read_time_minutes: int
    transformations_applied: int
    warnings_count: int
    user_difficulty_level: str
    recommended_simulator: str
    recommended_labs: list[str]


# =====================================================================
# API Endpoints
# =====================================================================

@router.post("/chapters/{chapter_id}", response_model=PersonalizedChapterResponse)
async def personalize_chapter(
    chapter_id: int,
    chapter_data: ChapterRequest,
    current_user: UserProfile = Depends(get_current_user)
):
    """
    Get a personalized version of a chapter for the current user.

    Personalization includes:
    - Filtering sections by difficulty and hardware
    - Simplifying math for beginners
    - Replacing Isaac Sim with Gazebo if no GPU
    - Adding prerequisite primers (ROS 2, Python, etc.)
    - Adding contextual warnings

    Args:
        chapter_id: Chapter number
        chapter_data: Raw chapter content
        current_user: Current authenticated user

    Returns:
        PersonalizedChapterResponse: Customized chapter
    """
    try:
        logger.info(f"Personalizing chapter {chapter_id} for user {current_user.email}")

        # Convert request to service format
        chapter_dict = {
            "id": chapter_data.id,
            "title": chapter_data.title,
            "sections": [
                {
                    "id": section.id,
                    "title": section.title,
                    "content": section.content,
                    "difficulty": section.difficulty,
                    "type": section.type,
                    "prerequisites": section.prerequisites or [],
                    "read_time_minutes": section.read_time_minutes
                }
                for section in chapter_data.sections
            ]
        }

        # Personalize
        personalized = personalization_service.personalize_chapter_for_user(
            chapter_dict, current_user
        )

        # Convert to response format
        return PersonalizedChapterResponse(
            chapter_id=personalized.chapter_id,
            chapter_title=personalized.chapter_title,
            user_difficulty_level=personalized.user_difficulty_level,
            sections=[
                PersonalizedSectionResponse(
                    id=section.id,
                    title=section.title,
                    content=section.content,
                    difficulty=section.difficulty.value,
                    section_type=section.section_type,
                    estimated_read_time_minutes=section.estimated_read_time_minutes
                )
                for section in personalized.sections
            ],
            transformations=[
                ContentTransformationResponse(
                    type=t.type,
                    section_id=t.section_id,
                    reason=t.reason
                )
                for t in personalized.transformations
            ],
            total_estimated_read_time=personalized.total_estimated_read_time,
            recommended_simulator=personalized.recommended_simulator,
            recommended_labs=personalized.recommended_labs,
            warnings=personalized.warnings
        )

    except Exception as e:
        logger.error(f"Personalization failed: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Personalization failed: {str(e)}"
        )


@router.get("/chapters/{chapter_id}/stats", response_model=PersonalizationStatsResponse)
async def get_personalization_stats(
    chapter_id: int,
    chapter_data: ChapterRequest,
    current_user: UserProfile = Depends(get_current_user)
):
    """
    Get statistics about how a chapter will be personalized.

    Returns metrics like:
    - How many sections will be shown/hidden
    - How many transformations will be applied
    - Total read time for this user
    - Warnings for this user

    Args:
        chapter_id: Chapter number
        chapter_data: Raw chapter content
        current_user: Current authenticated user

    Returns:
        PersonalizationStatsResponse: Personalization statistics
    """
    try:
        # Convert to service format
        chapter_dict = {
            "id": chapter_data.id,
            "title": chapter_data.title,
            "sections": [
                {
                    "id": section.id,
                    "title": section.title,
                    "content": section.content,
                    "difficulty": section.difficulty,
                    "type": section.type,
                    "prerequisites": section.prerequisites or [],
                    "read_time_minutes": section.read_time_minutes
                }
                for section in chapter_data.sections
            ]
        }

        # Personalize and get stats
        personalized = personalization_service.personalize_chapter_for_user(
            chapter_dict, current_user
        )
        stats = personalization_service.get_personalization_stats(personalized)

        return PersonalizationStatsResponse(**stats)

    except Exception as e:
        logger.error(f"Stats calculation failed: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Stats calculation failed: {str(e)}"
        )


@router.get("/profile/recommendations")
async def get_user_recommendations(
    current_user: UserProfile = Depends(get_current_user)
):
    """
    Get personalization recommendations for the current user.

    Returns:
    - Difficulty level
    - Recommended labs
    - Recommended simulator
    - Hardware warnings
    - Learning path suggestions

    Args:
        current_user: Current authenticated user

    Returns:
        dict: User recommendations
    """
    return {
        "user_id": str(current_user.id),
        "email": current_user.email,
        "difficulty_level": current_user.get_difficulty_level(),
        "recommended_labs": current_user.get_recommended_labs(),
        "recommended_simulator": current_user.get_recommended_simulation_environment(),
        "hardware_summary": current_user.get_hardware_summary(),
        "can_train_vla": current_user.can_run_vla_training(),
        "can_use_isaac_sim": current_user.can_run_isaac_sim(),
        "has_jetson": current_user.has_jetson,
        "has_robot": current_user.has_robot
    }


@router.post("/debug/apply-transformation")
async def debug_apply_transformation(
    chapter_id: int,
    chapter_data: ChapterRequest,
    current_user: UserProfile = Depends(get_current_user)
):
    """
    DEBUG ENDPOINT: Get detailed transformation information.

    Shows exactly what transformations are applied and why.

    Args:
        chapter_id: Chapter number
        chapter_data: Raw chapter content
        current_user: Current authenticated user

    Returns:
        dict: Detailed transformation info
    """
    try:
        # Convert to service format
        chapter_dict = {
            "id": chapter_data.id,
            "title": chapter_data.title,
            "sections": [
                {
                    "id": section.id,
                    "title": section.title,
                    "content": section.content,
                    "difficulty": section.difficulty,
                    "type": section.type,
                    "prerequisites": section.prerequisites or [],
                    "read_time_minutes": section.read_time_minutes
                }
                for section in chapter_data.sections
            ]
        }

        personalized = personalization_service.personalize_chapter_for_user(
            chapter_dict, current_user
        )

        return {
            "user_profile": {
                "difficulty_level": current_user.get_difficulty_level(),
                "programming_level": current_user.programming_level,
                "ai_experience": current_user.ai_experience,
                "robotics_experience": current_user.robotics_experience,
                "has_gpu": current_user.has_rtx,
                "has_jetson": current_user.has_jetson,
                "has_robot": current_user.has_robot,
                "learning_goal": current_user.learning_goal
            },
            "original_sections": len(chapter_data.sections),
            "personalized_sections": len(personalized.sections),
            "sections_hidden": len([t for t in personalized.transformations if t.type == "hide"]),
            "transformations": [
                {
                    "type": t.type,
                    "section_id": t.section_id,
                    "reason": t.reason
                }
                for t in personalized.transformations
            ],
            "total_read_time": personalized.total_estimated_read_time,
            "warnings": personalized.warnings,
            "recommended_simulator": personalized.recommended_simulator,
            "recommended_labs": personalized.recommended_labs
        }

    except Exception as e:
        logger.error(f"Debug failed: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Debug failed: {str(e)}"
        )
