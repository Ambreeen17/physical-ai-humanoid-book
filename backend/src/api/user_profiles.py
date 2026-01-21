"""User profile API endpoints."""

import logging
from typing import Optional
from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel, EmailStr, Field
from sqlalchemy.orm import Session
import uuid

from src.db.session import get_db
from src.models.user_profile import UserProfile
from src.auth.dependencies import get_current_user, get_current_user_optional

logger = logging.getLogger(__name__)

router = APIRouter(
    prefix="/api/v1/user-profiles",
    tags=["user-profiles"],
)


# =====================================================================
# Pydantic Models (Request/Response Schemas)
# =====================================================================

class UserProfileCreate(BaseModel):
    """Schema for creating a user profile."""

    email: EmailStr
    programming_level: str = Field(
        default="beginner",
        description="Level: beginner, intermediate, advanced"
    )
    ai_experience: str = Field(
        default="none",
        description="Experience: none, basic, applied"
    )
    robotics_experience: str = Field(
        default="none",
        description="Experience: none, basic, advanced"
    )
    has_rtx: bool = False
    gpu_model: Optional[str] = None
    rtx_vram_gb: Optional[str] = None
    has_jetson: bool = False
    jetson_model: Optional[str] = None
    has_robot: bool = False
    robot_type: Optional[str] = None
    robot_specs: Optional[dict] = None
    preferred_language: str = "en"
    learning_goal: Optional[str] = None
    content_preferences: Optional[dict] = None

    class Config:
        example = {
            "email": "researcher@example.com",
            "programming_level": "intermediate",
            "ai_experience": "basic",
            "robotics_experience": "none",
            "has_rtx": True,
            "gpu_model": "RTX 4090",
            "rtx_vram_gb": "24GB",
            "has_jetson": False,
            "has_robot": False,
            "preferred_language": "en",
            "learning_goal": "Learn ROS 2 for quadruped control",
            "content_preferences": {
                "theory_depth": "intermediate",
                "simulation_focus": True,
                "lab_difficulty": "intermediate"
            }
        }


class UserProfileUpdate(BaseModel):
    """Schema for updating a user profile."""

    programming_level: Optional[str] = None
    ai_experience: Optional[str] = None
    robotics_experience: Optional[str] = None
    has_rtx: Optional[bool] = None
    gpu_model: Optional[str] = None
    rtx_vram_gb: Optional[str] = None
    has_jetson: Optional[bool] = None
    jetson_model: Optional[str] = None
    has_robot: Optional[bool] = None
    robot_type: Optional[str] = None
    robot_specs: Optional[dict] = None
    preferred_language: Optional[str] = None
    learning_goal: Optional[str] = None
    content_preferences: Optional[dict] = None

    class Config:
        example = {
            "programming_level": "advanced",
            "robotics_experience": "intermediate"
        }


class UserProfileResponse(BaseModel):
    """Schema for user profile response."""

    id: str
    email: str
    programming_level: str
    ai_experience: str
    robotics_experience: str
    difficulty_level: str
    has_rtx: bool
    gpu_model: Optional[str]
    rtx_vram_gb: Optional[str]
    has_jetson: bool
    jetson_model: Optional[str]
    has_robot: bool
    robot_type: Optional[str]
    preferred_language: str
    learning_goal: Optional[str]
    hardware_summary: dict
    lab_difficulties: list

    class Config:
        from_attributes = True


class HardwareSummaryResponse(BaseModel):
    """Hardware capability summary."""

    gpu: dict
    jetson: dict
    robot: dict
    recommended_simulator: str
    lab_difficulties: list


class DifficultyResponse(BaseModel):
    """Difficulty level response."""

    difficulty_level: str
    recommended_labs: list
    recommended_simulator: str


# =====================================================================
# Endpoint Handlers
# =====================================================================

@router.post("", response_model=UserProfileResponse, status_code=status.HTTP_201_CREATED)
async def create_user_profile(
    profile_data: UserProfileCreate,
    db: Session = Depends(get_db)
):
    """Create a new user profile."""
    try:
        # Check if email already exists
        existing = db.query(UserProfile).filter(
            UserProfile.email == profile_data.email
        ).first()

        if existing:
            raise HTTPException(
                status_code=status.HTTP_400_BAD_REQUEST,
                detail=f"User with email {profile_data.email} already exists"
            )

        # Create new profile
        profile = UserProfile(
            **profile_data.model_dump()
        )
        db.add(profile)
        db.commit()
        db.refresh(profile)

        logger.info(f"Created user profile: {profile.email}")

        return UserProfileResponse(
            id=str(profile.id),
            email=profile.email,
            programming_level=profile.programming_level,
            ai_experience=profile.ai_experience,
            robotics_experience=profile.robotics_experience,
            difficulty_level=profile.get_difficulty_level(),
            has_rtx=profile.has_rtx,
            gpu_model=profile.gpu_model,
            rtx_vram_gb=profile.rtx_vram_gb,
            has_jetson=profile.has_jetson,
            jetson_model=profile.jetson_model,
            has_robot=profile.has_robot,
            robot_type=profile.robot_type,
            preferred_language=profile.preferred_language,
            learning_goal=profile.learning_goal,
            hardware_summary=profile.get_hardware_summary(),
            lab_difficulties=profile.get_recommended_labs()
        )

    except Exception as e:
        logger.error(f"Failed to create user profile: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to create user profile: {str(e)}"
        )


@router.get("/{user_id}", response_model=UserProfileResponse)
async def get_user_profile(
    user_id: str,
    db: Session = Depends(get_db)
):
    """Get user profile by ID."""
    try:
        profile = db.query(UserProfile).filter(
            UserProfile.id == uuid.UUID(user_id)
        ).first()

        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"User profile {user_id} not found"
            )

        return UserProfileResponse(
            id=str(profile.id),
            email=profile.email,
            programming_level=profile.programming_level,
            ai_experience=profile.ai_experience,
            robotics_experience=profile.robotics_experience,
            difficulty_level=profile.get_difficulty_level(),
            has_rtx=profile.has_rtx,
            gpu_model=profile.gpu_model,
            rtx_vram_gb=profile.rtx_vram_gb,
            has_jetson=profile.has_jetson,
            jetson_model=profile.jetson_model,
            has_robot=profile.has_robot,
            robot_type=profile.robot_type,
            preferred_language=profile.preferred_language,
            learning_goal=profile.learning_goal,
            hardware_summary=profile.get_hardware_summary(),
            lab_difficulties=profile.get_recommended_labs()
        )

    except Exception as e:
        logger.error(f"Failed to fetch user profile: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to fetch user profile: {str(e)}"
        )


@router.get("/email/{email}", response_model=UserProfileResponse)
async def get_user_profile_by_email(
    email: str,
    db: Session = Depends(get_db)
):
    """Get user profile by email."""
    try:
        profile = db.query(UserProfile).filter(
            UserProfile.email == email
        ).first()

        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"User profile with email {email} not found"
            )

        return UserProfileResponse(
            id=str(profile.id),
            email=profile.email,
            programming_level=profile.programming_level,
            ai_experience=profile.ai_experience,
            robotics_experience=profile.robotics_experience,
            difficulty_level=profile.get_difficulty_level(),
            has_rtx=profile.has_rtx,
            gpu_model=profile.gpu_model,
            rtx_vram_gb=profile.rtx_vram_gb,
            has_jetson=profile.has_jetson,
            jetson_model=profile.jetson_model,
            has_robot=profile.has_robot,
            robot_type=profile.robot_type,
            preferred_language=profile.preferred_language,
            learning_goal=profile.learning_goal,
            hardware_summary=profile.get_hardware_summary(),
            lab_difficulties=profile.get_recommended_labs()
        )

    except Exception as e:
        logger.error(f"Failed to fetch user profile by email: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to fetch user profile: {str(e)}"
        )


@router.put("/{user_id}", response_model=UserProfileResponse)
async def update_user_profile(
    user_id: str,
    profile_data: UserProfileUpdate,
    db: Session = Depends(get_db)
):
    """Update user profile."""
    try:
        profile = db.query(UserProfile).filter(
            UserProfile.id == uuid.UUID(user_id)
        ).first()

        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"User profile {user_id} not found"
            )

        # Update only provided fields
        update_data = profile_data.model_dump(exclude_unset=True)
        for field, value in update_data.items():
            setattr(profile, field, value)

        db.commit()
        db.refresh(profile)

        logger.info(f"Updated user profile: {profile.email}")

        return UserProfileResponse(
            id=str(profile.id),
            email=profile.email,
            programming_level=profile.programming_level,
            ai_experience=profile.ai_experience,
            robotics_experience=profile.robotics_experience,
            difficulty_level=profile.get_difficulty_level(),
            has_rtx=profile.has_rtx,
            gpu_model=profile.gpu_model,
            rtx_vram_gb=profile.rtx_vram_gb,
            has_jetson=profile.has_jetson,
            jetson_model=profile.jetson_model,
            has_robot=profile.has_robot,
            robot_type=profile.robot_type,
            preferred_language=profile.preferred_language,
            learning_goal=profile.learning_goal,
            hardware_summary=profile.get_hardware_summary(),
            lab_difficulties=profile.get_recommended_labs()
        )

    except Exception as e:
        logger.error(f"Failed to update user profile: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to update user profile: {str(e)}"
        )


@router.get("/{user_id}/difficulty", response_model=DifficultyResponse)
async def get_difficulty_recommendation(
    user_id: str,
    db: Session = Depends(get_db)
):
    """Get recommended difficulty level and simulator for user."""
    try:
        profile = db.query(UserProfile).filter(
            UserProfile.id == uuid.UUID(user_id)
        ).first()

        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"User profile {user_id} not found"
            )

        return DifficultyResponse(
            difficulty_level=profile.get_difficulty_level(),
            recommended_labs=profile.get_recommended_labs(),
            recommended_simulator=profile.get_recommended_simulation_environment()
        )

    except Exception as e:
        logger.error(f"Failed to get difficulty recommendation: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get difficulty recommendation: {str(e)}"
        )


@router.get("/{user_id}/hardware", response_model=HardwareSummaryResponse)
async def get_hardware_capabilities(
    user_id: str,
    db: Session = Depends(get_db)
):
    """Get user's hardware capabilities and constraints."""
    try:
        profile = db.query(UserProfile).filter(
            UserProfile.id == uuid.UUID(user_id)
        ).first()

        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"User profile {user_id} not found"
            )

        summary = profile.get_hardware_summary()
        return HardwareSummaryResponse(**summary)

    except Exception as e:
        logger.error(f"Failed to get hardware capabilities: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to get hardware capabilities: {str(e)}"
        )


@router.delete("/{user_id}", status_code=status.HTTP_204_NO_CONTENT)
async def delete_user_profile(
    user_id: str,
    db: Session = Depends(get_db)
):
    """Delete a user profile (soft delete recommended in production)."""
    try:
        profile = db.query(UserProfile).filter(
            UserProfile.id == uuid.UUID(user_id)
        ).first()

        if not profile:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail=f"User profile {user_id} not found"
            )

        db.delete(profile)
        db.commit()

        logger.info(f"Deleted user profile: {user_id}")

    except Exception as e:
        logger.error(f"Failed to delete user profile: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to delete user profile: {str(e)}"
        )


# =====================================================================
# Authenticated Endpoints
# =====================================================================

@router.get("/me/profile", response_model=UserProfileResponse)
async def get_my_profile(
    current_user: UserProfile = Depends(get_current_user)
):
    """
    Get the current authenticated user's profile.

    Uses JWT token from Authorization header to identify user.

    Args:
        current_user: Current authenticated user (from JWT token)

    Returns:
        UserProfileResponse: Current user's profile
    """
    return UserProfileResponse(
        id=str(current_user.id),
        email=current_user.email,
        programming_level=current_user.programming_level,
        ai_experience=current_user.ai_experience,
        robotics_experience=current_user.robotics_experience,
        difficulty_level=current_user.get_difficulty_level(),
        has_rtx=current_user.has_rtx,
        gpu_model=current_user.gpu_model,
        rtx_vram_gb=current_user.rtx_vram_gb,
        has_jetson=current_user.has_jetson,
        jetson_model=current_user.jetson_model,
        has_robot=current_user.has_robot,
        robot_type=current_user.robot_type,
        preferred_language=current_user.preferred_language,
        learning_goal=current_user.learning_goal,
        hardware_summary=current_user.get_hardware_summary(),
        lab_difficulties=current_user.get_recommended_labs()
    )


@router.put("/me/profile", response_model=UserProfileResponse)
async def update_my_profile(
    profile_data: UserProfileUpdate,
    current_user: UserProfile = Depends(get_current_user),
    db: Session = Depends(get_db)
):
    """
    Update the current authenticated user's profile.

    Args:
        profile_data: Fields to update
        current_user: Current authenticated user
        db: Database session

    Returns:
        UserProfileResponse: Updated profile
    """
    try:
        # Update only provided fields
        update_data = profile_data.model_dump(exclude_unset=True)
        for field, value in update_data.items():
            setattr(current_user, field, value)

        db.commit()
        db.refresh(current_user)

        logger.info(f"Updated user profile: {current_user.email}")

        return UserProfileResponse(
            id=str(current_user.id),
            email=current_user.email,
            programming_level=current_user.programming_level,
            ai_experience=current_user.ai_experience,
            robotics_experience=current_user.robotics_experience,
            difficulty_level=current_user.get_difficulty_level(),
            has_rtx=current_user.has_rtx,
            gpu_model=current_user.gpu_model,
            rtx_vram_gb=current_user.rtx_vram_gb,
            has_jetson=current_user.has_jetson,
            jetson_model=current_user.jetson_model,
            has_robot=current_user.has_robot,
            robot_type=current_user.robot_type,
            preferred_language=current_user.preferred_language,
            learning_goal=current_user.learning_goal,
            hardware_summary=current_user.get_hardware_summary(),
            lab_difficulties=current_user.get_recommended_labs()
        )

    except Exception as e:
        logger.error(f"Failed to update user profile: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail=f"Failed to update user profile: {str(e)}"
        )
