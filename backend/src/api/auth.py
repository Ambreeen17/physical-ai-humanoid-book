"""Authentication endpoints."""

import logging
from typing import Optional
from datetime import timedelta
from fastapi import APIRouter, Depends, HTTPException, status
from pydantic import BaseModel, EmailStr
from sqlalchemy.orm import Session

from src.db.session import get_db
from src.models.user_profile import UserProfile
from src.auth.dependencies import (
    create_access_token,
    get_current_user,
    ACCESS_TOKEN_EXPIRE_MINUTES,
)

logger = logging.getLogger(__name__)

router = APIRouter(
    prefix="/api/v1/auth",
    tags=["auth"],
)


# =====================================================================
# Pydantic Models
# =====================================================================

class LoginRequest(BaseModel):
    """Login request with email."""

    email: EmailStr

    class Config:
        example = {"email": "learner@example.com"}


class TokenResponse(BaseModel):
    """Access token response."""

    access_token: str
    token_type: str = "bearer"
    expires_in: int  # seconds
    user_id: str
    email: str

    class Config:
        example = {
            "access_token": "eyJhbGciOiJIUzI1NiIs...",
            "token_type": "bearer",
            "expires_in": 1800,
            "user_id": "550e8400-e29b-41d4-a716-446655440000",
            "email": "learner@example.com"
        }


class UserResponse(BaseModel):
    """Current user response."""

    id: str
    email: str
    first_name: Optional[str] = None
    last_name: Optional[str] = None
    programming_level: str
    ai_experience: str
    robotics_experience: str
    difficulty_level: str
    preferred_language: str
    learning_goal: Optional[str]
    created_at: str
    last_login: Optional[str]

    class Config:
        from_attributes = True


# =====================================================================
# Authentication Endpoints
# =====================================================================

@router.post("/login", response_model=TokenResponse, status_code=status.HTTP_200_OK)
async def login(
    request: LoginRequest,
    db: Session = Depends(get_db)
):
    """
    Login endpoint: authenticate user by email.

    This endpoint:
    1. Verifies the email exists in the system
    2. Creates a JWT access token
    3. Returns token and user info

    Note: In production, use OAuth2 (Google, GitHub) or email magic links.

    Args:
        request: Login request with email
        db: Database session

    Returns:
        TokenResponse: JWT token and user info
    """
    try:
        # Find user by email
        user = db.query(UserProfile).filter(
            UserProfile.email == request.email
        ).first()

        if not user:
            # Auto-create user for demo purposes (simplified auth)
            logger.info(f"Auto-creating user for: {request.email}")
            user = UserProfile(
                email=request.email,
                programming_level="beginner",
                ai_experience="none",
                robotics_experience="none",
                has_rtx=False,
                has_jetson=False,
                has_robot=False,
                preferred_language="en",
                learning_goal="career",
            )
            db.add(user)
            db.commit()
            db.refresh(user)

        # Create access token
        access_token = create_access_token(
            user_id=str(user.id),
            email=user.email,
            expires_delta=timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)
        )

        logger.info(f"User logged in: {user.email}")

        return TokenResponse(
            access_token=access_token,
            token_type="bearer",
            expires_in=ACCESS_TOKEN_EXPIRE_MINUTES * 60,  # Convert to seconds
            user_id=str(user.id),
            email=user.email
        )

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Login error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Login failed"
        )


@router.post("/logout", status_code=status.HTTP_200_OK)
async def logout(
    current_user: UserProfile = Depends(get_current_user)
):
    """
    Logout endpoint (stateless - no server-side action needed).

    In a stateless JWT system, logout is handled client-side by discarding the token.
    This endpoint can be used for audit logging.

    Args:
        current_user: Current authenticated user

    Returns:
        dict: Success message
    """
    logger.info(f"User logged out: {current_user.email}")

    return {"message": "Logged out successfully"}


@router.get("/me", response_model=UserResponse)
async def get_me(
    current_user: UserProfile = Depends(get_current_user)
):
    """
    Get current user profile.

    Args:
        current_user: Current authenticated user

    Returns:
        UserResponse: User profile details
    """
    return UserResponse(
        id=str(current_user.id),
        email=current_user.email,
        first_name=None,  # Not stored in UserProfile yet
        last_name=None,
        programming_level=current_user.programming_level,
        ai_experience=current_user.ai_experience,
        robotics_experience=current_user.robotics_experience,
        difficulty_level=current_user.get_difficulty_level(),
        preferred_language=current_user.preferred_language,
        learning_goal=current_user.learning_goal,
        created_at=current_user.created_at.isoformat() if current_user.created_at else None,
        last_login=current_user.last_login.isoformat() if current_user.last_login else None
    )


@router.get("/verify", status_code=status.HTTP_200_OK)
async def verify_token(
    current_user: UserProfile = Depends(get_current_user)
):
    """
    Verify that the current token is valid.

    Useful for frontend to check auth status before making requests.

    Args:
        current_user: Current authenticated user

    Returns:
        dict: User info and token status
    """
    return {
        "valid": True,
        "user_id": str(current_user.id),
        "email": current_user.email,
        "difficulty_level": current_user.get_difficulty_level()
    }
