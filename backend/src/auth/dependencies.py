"""Authentication dependencies for FastAPI."""

import logging
from typing import Optional
from fastapi import Depends, HTTPException, status
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from sqlalchemy.orm import Session
import jwt
from datetime import datetime, timedelta
import os

from src.db.session import get_db
from src.models.user_profile import UserProfile

logger = logging.getLogger(__name__)

# Configuration
SECRET_KEY = os.getenv("SECRET_KEY", "your-secret-key-change-in-production")
ALGORITHM = "HS256"
ACCESS_TOKEN_EXPIRE_MINUTES = int(os.getenv("ACCESS_TOKEN_EXPIRE_MINUTES", "30"))

security = HTTPBearer()


# =====================================================================
# Token Models
# =====================================================================

class TokenData:
    """Decoded JWT token data."""

    def __init__(self, user_id: str, email: str, exp: datetime):
        self.user_id = user_id
        self.email = email
        self.exp = exp


# =====================================================================
# Token Functions
# =====================================================================

def create_access_token(user_id: str, email: str, expires_delta: Optional[timedelta] = None) -> str:
    """
    Create a JWT access token.

    Args:
        user_id: User UUID
        email: User email
        expires_delta: Token expiration time (default: ACCESS_TOKEN_EXPIRE_MINUTES)

    Returns:
        str: Encoded JWT token
    """
    if expires_delta:
        expire = datetime.utcnow() + expires_delta
    else:
        expire = datetime.utcnow() + timedelta(minutes=ACCESS_TOKEN_EXPIRE_MINUTES)

    to_encode = {
        "user_id": user_id,
        "email": email,
        "exp": expire,
        "iat": datetime.utcnow()
    }

    encoded_jwt = jwt.encode(to_encode, SECRET_KEY, algorithm=ALGORITHM)
    return encoded_jwt


def decode_token(token: str) -> TokenData:
    """
    Decode and validate a JWT token.

    Args:
        token: JWT token string

    Returns:
        TokenData: Decoded token data

    Raises:
        HTTPException: If token is invalid or expired
    """
    try:
        payload = jwt.decode(token, SECRET_KEY, algorithms=[ALGORITHM])
        user_id: str = payload.get("user_id")
        email: str = payload.get("email")
        exp: datetime = datetime.fromtimestamp(payload.get("exp"))

        if user_id is None or email is None:
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Invalid token: missing user_id or email"
            )

        return TokenData(user_id=user_id, email=email, exp=exp)

    except jwt.ExpiredSignatureError:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Token has expired"
        )
    except jwt.InvalidTokenError as e:
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail=f"Invalid token: {str(e)}"
        )
    except Exception as e:
        logger.error(f"Token decode error: {str(e)}")
        raise HTTPException(
            status_code=status.HTTP_401_UNAUTHORIZED,
            detail="Could not validate token"
        )


# =====================================================================
# Dependency: Current User
# =====================================================================

def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security),
    db: Session = Depends(get_db)
) -> UserProfile:
    """
    Dependency to get the current authenticated user.

    Args:
        credentials: HTTP Bearer token from Authorization header
        db: Database session

    Returns:
        UserProfile: The authenticated user's profile

    Raises:
        HTTPException: If token is invalid or user not found
    """
    token = credentials.credentials

    # Decode token
    token_data = decode_token(token)

    # Fetch user from database
    user = db.query(UserProfile).filter(
        UserProfile.id == token_data.user_id
    ).first()

    if user is None:
        raise HTTPException(
            status_code=status.HTTP_404_NOT_FOUND,
            detail="User not found"
        )

    # Update last_login
    user.last_login = datetime.utcnow()
    db.commit()

    logger.info(f"User authenticated: {user.email}")

    return user


def get_current_user_optional(
    credentials: Optional[HTTPAuthorizationCredentials] = Depends(security),
    db: Session = Depends(get_db)
) -> Optional[UserProfile]:
    """
    Optional dependency to get the current user.

    Returns None if no credentials provided (for public endpoints).

    Args:
        credentials: HTTP Bearer token (optional)
        db: Database session

    Returns:
        Optional[UserProfile]: The authenticated user, or None
    """
    if credentials is None:
        return None

    token = credentials.credentials

    try:
        token_data = decode_token(token)
        user = db.query(UserProfile).filter(
            UserProfile.id == token_data.user_id
        ).first()

        if user:
            user.last_login = datetime.utcnow()
            db.commit()

        return user
    except HTTPException:
        return None


# =====================================================================
# Dependency: Admin User (future)
# =====================================================================

def get_admin_user(
    current_user: UserProfile = Depends(get_current_user)
) -> UserProfile:
    """
    Dependency to ensure current user is an admin.

    Args:
        current_user: Current authenticated user

    Returns:
        UserProfile: The authenticated user if admin

    Raises:
        HTTPException: If user is not an admin
    """
    # TODO: Add is_admin field to UserProfile
    # if not current_user.is_admin:
    #     raise HTTPException(
    #         status_code=status.HTTP_403_FORBIDDEN,
    #         detail="Admin access required"
    #     )

    return current_user


# =====================================================================
# Integration with Better-Auth (if available)
# =====================================================================

def get_current_user_from_better_auth(
    # This would come from better-auth library
    # Example: user_session = Depends(get_session)
    db: Session = Depends(get_db)
) -> UserProfile:
    """
    Get current user from better-auth session (if using that library).

    This is a template for integration with better-auth.
    Requires: pip install better-auth

    Args:
        db: Database session

    Returns:
        UserProfile: The authenticated user

    Raises:
        HTTPException: If user not authenticated or not found
    """
    # Example implementation (would require better-auth dependency):
    # from better_auth import get_session
    #
    # try:
    #     session = get_session()
    #     if not session or not session.user:
    #         raise HTTPException(
    #             status_code=status.HTTP_401_UNAUTHORIZED,
    #             detail="Not authenticated"
    #         )
    #
    #     user = db.query(UserProfile).filter(
    #         UserProfile.email == session.user.email
    #     ).first()
    #
    #     if not user:
    #         raise HTTPException(
    #             status_code=status.HTTP_404_NOT_FOUND,
    #             detail="User profile not found"
    #         )
    #
    #     return user
    # except Exception as e:
    #     raise HTTPException(
    #         status_code=status.HTTP_401_UNAUTHORIZED,
    #         detail=f"Authentication error: {str(e)}"
    #     )

    raise HTTPException(
        status_code=status.HTTP_501_NOT_IMPLEMENTED,
        detail="better-auth integration not configured"
    )
