# Complete System Integration Guide

## Overview

This document describes the complete integration of four interconnected systems for the Physical AI & Humanoid Robotics textbook backend:

1. **User Profiles** - Track learner background and hardware
2. **Onboarding** - Collect user information via 7-question form
3. **Authentication** - JWT-based stateless auth
4. **Personalization** - Dynamically adapt content

All systems are now integrated into the FastAPI application (`backend/src/main.py`).

---

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────┐
│                          FastAPI App                             │
├─────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐            │
│  │   Auth       │  │ Onboarding   │  │ User         │            │
│  │   Router     │  │ Router       │  │ Profiles     │            │
│  │              │  │              │  │ Router       │            │
│  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘            │
│         │                 │                  │                    │
│         └─────────────────┼──────────────────┘                    │
│                           │                                        │
│                    ┌──────▼────────┐                              │
│                    │ Personalization│                             │
│                    │ Router         │                             │
│                    └────────────────┘                             │
│                                                                   │
│  Dependency Injection Layer                                      │
│  ├── get_current_user() ──────▶ JWT validation + DB fetch        │
│  ├── get_current_user_optional()                                 │
│  └── PersonalizationService() ──▶ Content adaptation             │
│                                                                   │
│  Database Layer (SQLAlchemy)                                     │
│  ├── UserProfile (ORM model)                                     │
│  ├── User profiles table                                         │
│  └── Session management                                          │
│                                                                   │
└─────────────────────────────────────────────────────────────────┘
```

---

## System Flow

### 1. User Registration & Onboarding

```
┌─────────┐
│ Browser │
└────┬────┘
     │
     │ POST /api/v1/onboarding/form-options
     ▼
┌──────────────────────────────────────────────────┐
│ GET /onboarding/form-options                     │
│ Returns: Programming levels, GPU models, etc.    │
└──────────────────────────────────────────────────┘
     │
     │ User fills 7-question form
     │
     │ POST /api/v1/onboarding/questionnaire
     │ {
     │   "email": "learner@example.com",
     │   "name": "Alice",
     │   "programming_level": "intermediate",
     │   "ai_experience": "none",
     │   "robotics_experience": "none",
     │   "has_rtx": true,
     │   "gpu_model": "RTX 4080",
     │   "has_jetson": false,
     │   "has_robot": false,
     │   "learning_goal": "simulations"
     │ }
     ▼
┌──────────────────────────────────────────────────┐
│ POST /onboarding/questionnaire                   │
│ ├─ Create UserProfile in DB                      │
│ ├─ Calculate difficulty level                    │
│ ├─ Generate recommendations                      │
│ └─ Return recommendations                        │
└──────────────────────────────────────────────────┘
     │
     │ Response:
     │ {
     │   "user_id": "550e8400-...",
     │   "difficulty_level": "intermediate",
     │   "recommended_labs": ["beginner", "intermediate"],
     │   "recommended_simulator": "isaac_sim",
     │   "warnings": [...]
     │ }
     │
     ▼
┌─────────────────────────────────────────────────┐
│ Store user_id in localStorage                   │
│ Redirect to /login                              │
└─────────────────────────────────────────────────┘
```

### 2. Authentication

```
┌─────────┐
│ Browser │
└────┬────┘
     │
     │ POST /api/v1/auth/login
     │ {"email": "learner@example.com"}
     ▼
┌──────────────────────────────────────────────────┐
│ POST /auth/login                                 │
│ ├─ Find user by email                           │
│ ├─ Create JWT token (HS256, 30 min expiry)      │
│ └─ Return token                                  │
└──────────────────────────────────────────────────┘
     │
     │ Response:
     │ {
     │   "access_token": "eyJhbGc...",
     │   "token_type": "bearer",
     │   "expires_in": 1800,
     │   "user_id": "550e8400-..."
     │ }
     │
     ▼
┌─────────────────────────────────────────────────┐
│ Store token in localStorage:                    │
│ localStorage.setItem('access_token', token)     │
└─────────────────────────────────────────────────┘
```

### 3. Protected Request with Auth

```
┌─────────┐
│ Browser │
└────┬────┘
     │
     │ GET /api/v1/auth/me
     │ Headers: Authorization: Bearer eyJhbGc...
     ▼
┌──────────────────────────────────────────────────┐
│ get_current_user() Dependency                    │
│ ├─ Extract token from Authorization header       │
│ ├─ Decode JWT (validate signature)               │
│ ├─ Check token expiration                        │
│ ├─ Fetch UserProfile from DB                     │
│ ├─ Update last_login timestamp                   │
│ └─ Return UserProfile object                     │
└──────────────────────────────────────────────────┘
     │
     │ GET /auth/me endpoint
     │ ├─ Access current_user object (auto-injected)
     │ └─ Return user data
     ▼
┌──────────────────────────────────────────────────┐
│ Response:                                        │
│ {                                                │
│   "id": "550e8400-...",                          │
│   "email": "learner@example.com",               │
│   "name": "Alice",                               │
│   "programming_level": "intermediate",           │
│   "has_rtx": true,                               │
│   "learning_goal": "simulations",                │
│   "last_login": "2025-12-31T12:34:56"           │
│ }                                                │
└──────────────────────────────────────────────────┘
```

### 4. Content Personalization

```
┌─────────┐
│ Browser │
└────┬────┘
     │
     │ POST /api/v1/personalization/chapters/1
     │ Headers: Authorization: Bearer <token>
     │ Body: {chapter data with all sections}
     ▼
┌──────────────────────────────────────────────────┐
│ personalize_chapter() Endpoint                   │
│ ├─ Extract current_user from token               │
│ └─ Pass chapter + user to PersonalizationService │
└──────────────────────────────────────────────────┘
     │
     ▼
┌──────────────────────────────────────────────────┐
│ PersonalizationEngine.personalize_chapter()      │
│                                                  │
│ 1. Build context from user profile               │
│    ├─ difficulty_level: "intermediate"           │
│    ├─ programming_level: "intermediate"          │
│    ├─ has_rtx_gpu: true                          │
│    ├─ can_use_isaac_sim: true                    │
│    └─ learning_goal: "simulations"               │
│                                                  │
│ 2. Filter sections                               │
│    for section in chapter.sections:              │
│      if should_include_section(section):         │
│        transform & keep                          │
│      else:                                       │
│        hide (record transformation)              │
│                                                  │
│ 3. Apply transformations                         │
│    ├─ SIMPLIFY: math for non-advanced users      │
│    ├─ REPLACE: Isaac Sim → Gazebo (no GPU)       │
│    ├─ AMPLIFY: extra examples for beginners      │
│    └─ INSERT: cloud GPU path (no local GPU)      │
│                                                  │
│ 4. Insert prerequisites                          │
│    ├─ ROS 2 Primer (robotics_experience=none)    │
│    ├─ Python Refresher (programming=beginner)    │
│    └─ Simulation Fundamentals (goal=simulations) │
│                                                  │
│ 5. Generate warnings                             │
│    ├─ "No GPU detected: Using Gazebo"            │
│    └─ "ROS 2 Primer included"                    │
│                                                  │
└──────────────────────────────────────────────────┘
     │
     │ Returns PersonalizedChapter
     ▼
┌──────────────────────────────────────────────────┐
│ Response:                                        │
│ {                                                │
│   "chapter_id": 1,                               │
│   "chapter_title": "Introduction to Physical AI",│
│   "user_difficulty_level": "intermediate",       │
│   "sections": [                                  │
│     {"id": "ros2-primer", "title": "...", ...},  │
│     {"id": "intro", "title": "...", ...},        │
│     // ... more sections                         │
│   ],                                             │
│   "transformations": [                           │
│     {                                            │
│       "type": "insert",                          │
│       "section_id": "ros2-primer",               │
│       "reason": "ROS 2 primer - no robotics exp" │
│     },                                           │
│     // ... more transformations                  │
│   ],                                             │
│   "total_estimated_read_time": 125,              │
│   "recommended_simulator": "isaac_sim",          │
│   "recommended_labs": ["beginner", "intermediate"],
│   "warnings": [                                  │
│     "⚠️ ROS 2 Primer included: Start here!"     │
│   ]                                              │
│ }                                                │
└──────────────────────────────────────────────────┘
     │
     ▼
┌─────────────────────────────────────────────────┐
│ Frontend renders personalized chapter            │
│ ├─ Show warnings prominently                     │
│ ├─ Display ROS 2 Primer first                    │
│ ├─ Then original content (filtered)              │
│ ├─ Highlight transformations                     │
│ └─ Display estimated read time                   │
└─────────────────────────────────────────────────┘
```

---

## API Endpoint Summary

### Authentication Endpoints

| Method | Path | Auth | Purpose |
|--------|------|------|---------|
| POST | `/api/v1/auth/login` | ❌ | Login with email, get JWT token |
| GET | `/api/v1/auth/me` | ✅ | Get current user profile |
| GET | `/api/v1/auth/verify` | ✅ | Verify token is valid |
| POST | `/api/v1/auth/logout` | ✅ | Logout (client-side discard token) |

### User Profile Endpoints

| Method | Path | Auth | Purpose |
|--------|------|------|---------|
| GET | `/api/v1/user-profiles/me/profile` | ✅ | Get my profile |
| PUT | `/api/v1/user-profiles/me/profile` | ✅ | Update my profile |
| GET | `/api/v1/user-profiles/{user_id}` | ✅ | Get user by ID |
| GET | `/api/v1/user-profiles/email/{email}` | ✅ | Get user by email |

### Onboarding Endpoints

| Method | Path | Auth | Purpose |
|--------|------|------|---------|
| POST | `/api/v1/onboarding/questionnaire` | ❌ | Submit questionnaire, create profile |
| GET | `/api/v1/onboarding/form-options` | ❌ | Get form field options (enums) |
| GET | `/api/v1/onboarding/recommendations/{user_id}` | ❌ | Get recommendations |

### Personalization Endpoints

| Method | Path | Auth | Purpose |
|--------|------|------|---------|
| POST | `/api/v1/personalization/chapters/{id}` | ✅ | Personalize chapter for user |
| GET | `/api/v1/personalization/chapters/{id}/stats` | ✅ | Get personalization stats |
| GET | `/api/v1/personalization/profile/recommendations` | ✅ | Get user recommendations |
| POST | `/api/v1/personalization/debug/apply-transformation` | ✅ | DEBUG: Show transformation details |

---

## Integration Checklist

- [x] User Profile ORM model created (`backend/src/models/user_profile.py`)
- [x] User profile endpoints implemented (`backend/src/api/user_profiles.py`)
- [x] Onboarding schemas created (`backend/src/schemas/onboarding.py`)
- [x] Onboarding endpoints implemented (`backend/src/api/onboarding.py`)
- [x] Onboarding form component created (`website/src/components/OnboardingQuestionnaire.jsx`)
- [x] Authentication dependencies created (`backend/src/auth/dependencies.py`)
- [x] Auth endpoints implemented (`backend/src/api/auth.py`)
- [x] Personalization service created (`backend/src/services/personalization_service.py`)
- [x] Personalization endpoints implemented (`backend/src/api/personalization.py`)
- [x] **Routers registered in main.py** ✅ **COMPLETED**

---

## Testing the Integration

### 1. Get Form Options (No Auth Required)

```bash
curl -X GET http://localhost:8000/api/v1/onboarding/form-options | jq
```

**Response:**
```json
{
  "programming_levels": ["beginner", "intermediate", "advanced"],
  "ai_experiences": ["none", "basics", "applied"],
  "robotics_experiences": ["none", "basics", "advanced"],
  "gpu_models": ["None", "RTX 4090", "RTX 4080", ...],
  "jetson_models": ["None", "Jetson Orin", "Jetson AGX Orin", ...],
  "robot_types": ["None", "Unitree G1", "Boston Dynamics Spot", ...],
  "learning_goals": ["simulations", "real_robots", "ai_integration", "general"]
}
```

### 2. Submit Onboarding (No Auth Required)

```bash
curl -X POST http://localhost:8000/api/v1/onboarding/questionnaire \
  -H "Content-Type: application/json" \
  -d '{
    "email": "learner@example.com",
    "name": "Alice Johnson",
    "programming_level": "intermediate",
    "ai_experience": "none",
    "robotics_experience": "none",
    "has_rtx": true,
    "gpu_model": "RTX 4080",
    "has_jetson": false,
    "jetson_model": null,
    "has_robot": false,
    "robot_type": null,
    "learning_goal": "simulations"
  }' | jq
```

**Response:** User profile created with recommendations

### 3. Login (No Auth Required)

```bash
TOKEN=$(curl -X POST http://localhost:8000/api/v1/auth/login \
  -H "Content-Type: application/json" \
  -d '{"email": "learner@example.com"}' | jq -r '.access_token')

echo "Token: $TOKEN"
```

### 4. Get Current User (Auth Required)

```bash
curl -X GET http://localhost:8000/api/v1/auth/me \
  -H "Authorization: Bearer $TOKEN" | jq
```

### 5. Personalize Chapter (Auth Required)

```bash
curl -X POST http://localhost:8000/api/v1/personalization/chapters/1 \
  -H "Authorization: Bearer $TOKEN" \
  -H "Content-Type: application/json" \
  -d '{
    "id": 1,
    "title": "Introduction to Physical AI",
    "sections": [
      {
        "id": "intro",
        "title": "Introduction",
        "content": "Physical AI is the study of...",
        "difficulty": "beginner",
        "type": "theory",
        "read_time_minutes": 10
      },
      {
        "id": "advanced_math",
        "title": "Advanced Math",
        "content": "∇f = ∂f/∂x + ∂f/∂y",
        "difficulty": "advanced",
        "type": "math",
        "read_time_minutes": 20
      }
    ]
  }' | jq
```

---

## Code Examples

### Backend: Using Authenticated User

```python
from fastapi import APIRouter, Depends
from src.auth.dependencies import get_current_user
from src.models.user_profile import UserProfile
from src.services.personalization_service import PersonalizationService

router = APIRouter()
personalization_service = PersonalizationService()

@router.post("/personalization/chapters/{chapter_id}")
async def personalize_chapter(
    chapter_id: int,
    chapter_data: ChapterRequest,
    current_user: UserProfile = Depends(get_current_user)  # ✅ Auto-injected
):
    # current_user is fully loaded from DB and validated
    personalized = personalization_service.personalize_chapter_for_user(
        chapter_data.dict(),
        current_user
    )
    return personalized
```

### Frontend: React Hook

```javascript
import { useState, useEffect } from 'react';

function usePersonalizedChapter(chapterId, chapterData) {
    const [personalized, setPersonalized] = useState(null);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        async function fetch() {
            const token = localStorage.getItem('access_token');
            const response = await fetch(
                `/api/v1/personalization/chapters/${chapterId}`,
                {
                    method: 'POST',
                    headers: {
                        'Authorization': `Bearer ${token}`,
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify(chapterData)
                }
            );

            if (response.ok) {
                const data = await response.json();
                setPersonalized(data);
            }
            setLoading(false);
        }
        fetch();
    }, [chapterId]);

    return { personalized, loading };
}

function ChapterPage() {
    const { personalized, loading } = usePersonalizedChapter(1, chapterData);

    if (loading) return <div>Loading...</div>;

    return (
        <div>
            <h1>{personalized.chapter_title}</h1>
            {personalized.warnings.map(w => (
                <div key={w} className="warning">{w}</div>
            ))}
            {personalized.sections.map(section => (
                <Section key={section.id} section={section} />
            ))}
        </div>
    );
}
```

### Frontend: Login & Store Token

```javascript
async function login(email) {
    const response = await fetch('/api/v1/auth/login', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ email })
    });

    const data = await response.json();
    localStorage.setItem('access_token', data.access_token);
    localStorage.setItem('user_id', data.user_id);

    window.location.href = '/dashboard';
}
```

---

## Data Flow Summary

```
User Onboards
    ↓
User Profile Created in DB
    ↓
User Logs In
    ↓
JWT Token Generated
    ↓
Client Makes Request with Token
    ↓
get_current_user() Validates Token & Fetches User
    ↓
Endpoint Receives UserProfile Object
    ↓
PersonalizationService Uses UserProfile to Adapt Content
    ↓
Client Receives Personalized Chapter
```

---

## Key Dependencies

```python
# Authentication
- PyJWT: JWT token creation/validation
- Python-multipart: Form data parsing

# Database
- SQLAlchemy: ORM
- psycopg2: PostgreSQL driver

# API
- FastAPI: Web framework
- Pydantic: Request/response validation

# Utilities
- python-dotenv: Environment configuration
- logging: Application logging
```

---

## Environment Configuration

Create `.env` file in project root:

```bash
# Database
DATABASE_URL=postgresql://user:password@localhost/textbook_db

# Authentication
SECRET_KEY=your-super-secret-key-here
ACCESS_TOKEN_EXPIRE_MINUTES=30

# CORS
CORS_ORIGINS=["http://localhost:3000", "http://localhost:8000"]

# Logging
LOG_LEVEL=INFO
```

---

## Next Steps

1. **Database Setup**: Run migrations to create `user_profiles` table
2. **Environment Setup**: Configure `.env` with database and secret key
3. **Start Backend**: `python -m uvicorn src.main:app --reload`
4. **Frontend Integration**: Wire React components to API endpoints
5. **Testing**: Run integration tests for all 4 systems
6. **Deployment**: Deploy to production with HTTPS enabled

---

## Documentation References

- **User Profiles**: `USER_PROFILE_DOCUMENTATION.md`
- **Onboarding**: `ONBOARDING_SYSTEM_DOCUMENTATION.md` + `ONBOARDING_QUICK_REFERENCE.md`
- **Authentication**: `AUTHENTICATION_GUIDE.md` + `AUTH_QUICK_INTEGRATION.md`
- **Personalization**: `PERSONALIZATION_SYSTEM_GUIDE.md` + `PERSONALIZATION_QUICK_START.md`

---

**Status**: ✅ All systems implemented and integrated into main.py
**Version**: 1.0.0
**Last Updated**: 2025-12-31
