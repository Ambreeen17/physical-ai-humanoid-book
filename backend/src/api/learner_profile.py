"""
Learner Profile API Endpoints
Manages learner profiles, background assessment, and personalization tier calculation.
"""
from typing import Optional
from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session
from pydantic import BaseModel, Field, validator

from ..db.session import get_db
from ..models.learner_profile import LearnerProfile
from ..logging.logger import get_logger

logger = get_logger(__name__)
router = APIRouter(prefix="/api/learner-profile", tags=["learner-profile"])


# Request/Response Schemas
class LearnerProfileCreate(BaseModel):
    """Request schema for creating a learner profile."""
    learner_id: str = Field(..., description="Unique learner identifier (UUID or email)")
    python_score: int = Field(..., ge=0, le=10, description="Python experience (0-10)")
    ml_score: int = Field(..., ge=0, le=10, description="ML/AI experience (0-10)")
    robotics_score: int = Field(..., ge=0, le=10, description="Robotics experience (0-10)")
    ros_score: int = Field(..., ge=0, le=10, description="ROS 2 experience (0-10)")

    @validator('python_score', 'ml_score', 'robotics_score', 'ros_score')
    def validate_score_range(cls, v):
        if not (0 <= v <= 10):
            raise ValueError("Score must be between 0 and 10")
        return v


class LearnerProfileUpdate(BaseModel):
    """Request schema for updating a learner profile."""
    python_score: Optional[int] = Field(None, ge=0, le=10)
    ml_score: Optional[int] = Field(None, ge=0, le=10)
    robotics_score: Optional[int] = Field(None, ge=0, le=10)
    ros_score: Optional[int] = Field(None, ge=0, le=10)


class LearnerProfileResponse(BaseModel):
    """Response schema for learner profile."""
    id: int
    learner_id: str
    python_score: int
    ml_score: int
    robotics_score: int
    ros_score: int
    difficulty_level: str
    recommended_chapters: list[str]

    class Config:
        from_attributes = True


# Endpoints
@router.post("/", response_model=LearnerProfileResponse, status_code=status.HTTP_201_CREATED)
def create_learner_profile(
    profile_data: LearnerProfileCreate,
    db: Session = Depends(get_db)
):
    """
    Create a new learner profile with background assessment.

    Calculates difficulty level based on weighted average:
    - Python: 30%
    - ML/AI: 25%
    - Robotics: 25%
    - ROS 2: 20%

    Returns recommended starting chapters based on difficulty.
    """
    # Check if learner already exists
    existing = db.query(LearnerProfile).filter(
        LearnerProfile.learner_id == profile_data.learner_id
    ).first()

    if existing:
        logger.warning(f"Learner profile already exists: {profile_data.learner_id}")
        raise HTTPException(
            status_code=status.HTTP_409_CONFLICT,
            detail=f"Learner profile for '{profile_data.learner_id}' already exists"
        )

    # Create new profile
    new_profile = LearnerProfile(
        learner_id=profile_data.learner_id,
        python_score=profile_data.python_score,
        ml_score=profile_data.ml_score,
        robotics_score=profile_data.robotics_score,
        ros_score=profile_data.ros_score
    )

    db.add(new_profile)
    db.commit()
    db.refresh(new_profile)

    logger.info(
        f"Created learner profile: {new_profile.learner_id} "
        f"(difficulty: {new_profile.difficulty_level})"
    )

    return LearnerProfileResponse(
        id=new_profile.id,
        learner_id=new_profile.learner_id,
        python_score=new_profile.python_score,
        ml_score=new_profile.ml_score,
        robotics_score=new_profile.robotics_score,
        ros_score=new_profile.ros_score,
        difficulty_level=new_profile.difficulty_level,
        recommended_chapters=new_profile.recommended_chapters
    )


@router.get("/{learner_id}", response_model=LearnerProfileResponse)
def get_learner_profile(
    learner_id: str,
    db: Session = Depends(get_db)
):
    """
    Retrieve a learner profile by learner_id.
    """
    profile = db.query(LearnerProfile).filter(
        LearnerProfile.learner_id == learner_id
    ).first()

    if not profile:
        logger.warning(f"Learner profile not found: {learner_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Learner profile for '{learner_id}' not found"
        )

    return LearnerProfileResponse(
        id=profile.id,
        learner_id=profile.learner_id,
        python_score=profile.python_score,
        ml_score=profile.ml_score,
        robotics_score=profile.robotics_score,
        ros_score=profile.ros_score,
        difficulty_level=profile.difficulty_level,
        recommended_chapters=profile.recommended_chapters
    )


@router.put("/{learner_id}", response_model=LearnerProfileResponse)
def update_learner_profile(
    learner_id: str,
    update_data: LearnerProfileUpdate,
    db: Session = Depends(get_db)
):
    """
    Update an existing learner profile.
    Recalculates difficulty level and recommended chapters.
    """
    profile = db.query(LearnerProfile).filter(
        LearnerProfile.learner_id == learner_id
    ).first()

    if not profile:
        logger.warning(f"Learner profile not found for update: {learner_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Learner profile for '{learner_id}' not found"
        )

    # Update only provided fields
    update_dict = update_data.dict(exclude_unset=True)
    for field, value in update_dict.items():
        setattr(profile, field, value)

    db.commit()
    db.refresh(profile)

    logger.info(
        f"Updated learner profile: {profile.learner_id} "
        f"(new difficulty: {profile.difficulty_level})"
    )

    return LearnerProfileResponse(
        id=profile.id,
        learner_id=profile.learner_id,
        python_score=profile.python_score,
        ml_score=profile.ml_score,
        robotics_score=profile.robotics_score,
        ros_score=profile.ros_score,
        difficulty_level=profile.difficulty_level,
        recommended_chapters=profile.recommended_chapters
    )


@router.delete("/{learner_id}", status_code=status.HTTP_204_NO_CONTENT)
def delete_learner_profile(
    learner_id: str,
    db: Session = Depends(get_db)
):
    """
    Delete a learner profile.
    """
    profile = db.query(LearnerProfile).filter(
        LearnerProfile.learner_id == learner_id
    ).first()

    if not profile:
        logger.warning(f"Learner profile not found for deletion: {learner_id}")
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail=f"Learner profile for '{learner_id}' not found"
        )

    db.delete(profile)
    db.commit()

    logger.info(f"Deleted learner profile: {learner_id}")
    return None
