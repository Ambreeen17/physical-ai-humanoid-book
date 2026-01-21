# User Profile System Documentation

## Overview

The User Profile system tracks learner background, hardware access, and preferences to deliver personalized learning experiences in the Physical AI & Humanoid Robotics textbook.

**Key Features**:
- Background level tracking (programming, AI, robotics)
- Hardware capability detection (RTX GPU, Jetson, physical robots)
- Intelligent content recommendations (difficulty level, simulator choice)
- Multi-language preference support
- Hardware constraints awareness for lab selection

---

## Database Schema

### `user_profiles` Table

```sql
CREATE TABLE user_profiles (
    id UUID PRIMARY KEY,
    email VARCHAR(255) UNIQUE NOT NULL,

    -- Background Levels
    programming_level VARCHAR(20) DEFAULT 'beginner',
    ai_experience VARCHAR(20) DEFAULT 'none',
    robotics_experience VARCHAR(20) DEFAULT 'none',

    -- GPU Hardware (NVIDIA RTX)
    has_rtx BOOLEAN DEFAULT FALSE,
    gpu_model VARCHAR(100),
    rtx_vram_gb VARCHAR(50),

    -- Edge Computing (NVIDIA Jetson)
    has_jetson BOOLEAN DEFAULT FALSE,
    jetson_model VARCHAR(100),

    -- Physical Robotics
    has_robot BOOLEAN DEFAULT FALSE,
    robot_type VARCHAR(100),
    robot_specs JSONB,

    -- Preferences
    preferred_language VARCHAR(10) DEFAULT 'en',
    learning_goal TEXT,
    content_preferences JSONB,

    -- Metadata
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_login TIMESTAMP
);
```

### Field Definitions

#### Background Levels

| Field | Type | Values | Purpose |
|-------|------|--------|---------|
| `programming_level` | ENUM | beginner, intermediate, advanced | Software engineering experience |
| `ai_experience` | ENUM | none, basic, applied | ML/AI knowledge level |
| `robotics_experience` | ENUM | none, basic, advanced | Robotics hands-on experience |

**Scoring Algorithm** (for difficulty detection):
```
score = (programming * 0.4) + (robotics * 0.4) + (ai * 0.2)

Mapped values:
  programming: beginner=1, intermediate=2, advanced=3
  robotics:    none=0, basic=1, advanced=2
  ai:          none=0, basic=1, applied=2

Result:
  score >= 2.2  → "advanced"
  score >= 1.2  → "intermediate"
  score <  1.2  → "beginner"
```

#### Hardware Fields

| Field | Type | Example | Lab Implication |
|-------|------|---------|-----------------|
| `has_rtx` | Boolean | TRUE | Can run Isaac Sim |
| `gpu_model` | String | "RTX 4090" | Determines simulation fidelity |
| `rtx_vram_gb` | String | "24GB" | Can train VLA models (≥24GB) |
| `has_jetson` | Boolean | TRUE | Can deploy to edge devices |
| `jetson_model` | String | "Jetson AGX Orin" | Performance constraints |
| `has_robot` | Boolean | TRUE | Can validate on real hardware |
| `robot_type` | String | "Unitree G1" | Determines advanced labs |
| `robot_specs` | JSON | `{model, year, dof, weight_kg}` | Detailed specs for labs |

#### Preferences

| Field | Type | Example | Purpose |
|-------|------|---------|---------|
| `preferred_language` | String | "en", "ur", "es" | Content language (Docusaurus i18n) |
| `learning_goal` | String | "Learn ROS 2 for quadruped control" | Personalization context |
| `content_preferences` | JSON | `{theory_depth, simulation_focus, lab_difficulty}` | Fine-tuned content delivery |

---

## SQLAlchemy ORM Model

**Location**: `backend/src/models/user_profile.py`

### Class Definition

```python
class UserProfile(Base):
    """User profile entity."""

    __tablename__ = "user_profiles"

    # Fields (as above)
    id: UUID
    email: str
    programming_level: str
    # ... (all fields)
```

### Key Methods

#### 1. `get_difficulty_level() → str`
Determines content difficulty based on weighted background scores.

```python
profile = UserProfile(
    programming_level="intermediate",  # 2
    robotics_experience="none",        # 0
    ai_experience="basic"              # 1
)

difficulty = profile.get_difficulty_level()
# Returns: "intermediate" (score = 2*0.4 + 0*0.4 + 1*0.2 = 1.0 → intermediate)
```

**Return Values**:
- `"beginner"`: Score < 1.2
- `"intermediate"`: Score 1.2-2.2
- `"advanced"`: Score ≥ 2.2

#### 2. `get_recommended_labs() → list`
Recommends lab difficulty levels learner should attempt.

```python
profile.get_recommended_labs()
# Beginner → ["beginner"]
# Intermediate → ["beginner", "intermediate"]
# Advanced → ["beginner", "intermediate", "advanced"]
```

#### 3. `get_recommended_simulation_environment() → str`
Recommends optimal simulator based on hardware.

```python
profile.get_recommended_simulation_environment()
# Returns: "isaac_sim" if RTX GPU with ≥24GB VRAM
# Returns: "gazebo" if Jetson only
# Returns: "gazebo" if no GPU (default, lightweight)
```

#### 4. `get_rtx_vram_gb() → int`
Extracts GPU VRAM as integer.

```python
profile.rtx_vram_gb = "24GB"
profile.get_rtx_vram_gb()  # Returns: 24

profile.rtx_vram_gb = "48 GB"
profile.get_rtx_vram_gb()  # Returns: 48
```

#### 5. `can_run_vla_training() → bool`
Checks if user can train Vision-Language-Action models.

**Requirements**: RTX GPU with ≥24GB VRAM

```python
profile.has_rtx = True
profile.rtx_vram_gb = "24GB"
profile.can_run_vla_training()  # Returns: True

profile.rtx_vram_gb = "12GB"
profile.can_run_vla_training()  # Returns: False
```

#### 6. `can_run_isaac_sim() → bool`
Checks if user can run NVIDIA Isaac Sim.

**Requirements**: RTX GPU (recommended ≥12GB VRAM)

```python
profile.has_rtx = True
profile.rtx_vram_gb = "12GB"
profile.can_run_isaac_sim()  # Returns: True
```

#### 7. `has_edge_deployment_capability() → bool`
Checks if user can deploy to Jetson edge devices.

```python
profile.has_jetson = True
profile.jetson_model = "Jetson AGX Orin"
profile.has_edge_deployment_capability()  # Returns: True
```

#### 8. `has_real_robot_access() → bool`
Checks if user has physical robotics hardware.

```python
profile.has_robot = True
profile.robot_type = "Unitree G1"
profile.has_real_robot_access()  # Returns: True
```

#### 9. `get_hardware_summary() → dict`
Comprehensive hardware capabilities summary.

```python
summary = profile.get_hardware_summary()
# Returns:
# {
#     "gpu": {
#         "available": True,
#         "model": "RTX 4090",
#         "vram_gb": 24,
#         "can_vla_train": True
#     },
#     "jetson": {
#         "available": False,
#         "model": None,
#         "edge_deployable": False
#     },
#     "robot": {
#         "available": True,
#         "type": "Unitree G1",
#         "specs": {model, year, dof, weight_kg}
#     },
#     "recommended_simulator": "isaac_sim",
#     "lab_difficulties": ["beginner", "intermediate", "advanced"]
# }
```

#### 10. `to_dict() → dict`
Convert profile to dictionary (useful for JSON responses).

---

## FastAPI Endpoints

**Base Path**: `/api/v1/user-profiles`

### 1. Create User Profile
```
POST /api/v1/user-profiles
Content-Type: application/json

{
    "email": "researcher@example.com",
    "programming_level": "intermediate",
    "ai_experience": "basic",
    "robotics_experience": "none",
    "has_rtx": true,
    "gpu_model": "RTX 4090",
    "rtx_vram_gb": "24GB",
    "has_jetson": false,
    "has_robot": false,
    "preferred_language": "en",
    "learning_goal": "Learn ROS 2 for humanoid control",
    "content_preferences": {
        "theory_depth": "intermediate",
        "simulation_focus": true,
        "lab_difficulty": "intermediate"
    }
}

Response (201 Created):
{
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "researcher@example.com",
    "programming_level": "intermediate",
    "ai_experience": "basic",
    "robotics_experience": "none",
    "difficulty_level": "intermediate",
    "has_rtx": true,
    "gpu_model": "RTX 4090",
    "rtx_vram_gb": "24GB",
    "has_jetson": false,
    "jetson_model": null,
    "has_robot": false,
    "robot_type": null,
    "preferred_language": "en",
    "learning_goal": "Learn ROS 2 for humanoid control",
    "hardware_summary": {...},
    "lab_difficulties": ["beginner", "intermediate", "advanced"]
}
```

**Error Responses**:
- `400 Bad Request`: Email already exists
- `500 Internal Server Error`: Database error

### 2. Get User Profile by ID
```
GET /api/v1/user-profiles/{user_id}

Response (200 OK):
{
    "id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "researcher@example.com",
    ...
}
```

**Error Responses**:
- `404 Not Found`: User ID doesn't exist
- `500 Internal Server Error`: Database error

### 3. Get User Profile by Email
```
GET /api/v1/user-profiles/email/{email}

Response (200 OK):
{same as above}
```

### 4. Update User Profile
```
PUT /api/v1/user-profiles/{user_id}
Content-Type: application/json

{
    "programming_level": "advanced",
    "robotics_experience": "intermediate",
    "rtx_vram_gb": "48GB"
}

Response (200 OK):
{updated profile}
```

**Notes**:
- Only provided fields are updated
- Null fields are preserved (not deleted)

### 5. Get Difficulty Recommendation
```
GET /api/v1/user-profiles/{user_id}/difficulty

Response (200 OK):
{
    "difficulty_level": "intermediate",
    "recommended_labs": ["beginner", "intermediate"],
    "recommended_simulator": "isaac_sim"
}
```

### 6. Get Hardware Capabilities
```
GET /api/v1/user-profiles/{user_id}/hardware

Response (200 OK):
{
    "gpu": {
        "available": true,
        "model": "RTX 4090",
        "vram_gb": 24,
        "can_vla_train": true
    },
    "jetson": {
        "available": false,
        "model": null,
        "edge_deployable": false
    },
    "robot": {
        "available": false,
        "type": null,
        "specs": {}
    },
    "recommended_simulator": "isaac_sim",
    "lab_difficulties": ["beginner", "intermediate", "advanced"]
}
```

### 7. Delete User Profile
```
DELETE /api/v1/user-profiles/{user_id}

Response (204 No Content)
```

---

## Integration Examples

### Example 1: Determine Lab Content Path

```python
# Fetch user profile
user = db.query(UserProfile).filter_by(email="learner@example.com").first()

# Get recommended difficulty
difficulty = user.get_difficulty_level()
# → "intermediate"

# Get recommended labs
labs = user.get_recommended_labs()
# → ["beginner", "intermediate"]

# Recommend lab based on chapter and user level
chapter = db.query(Chapter).filter_by(number=2).first()
available_labs = db.query(Lab).filter(
    Lab.chapter_id == chapter.id,
    Lab.difficulty.in_(labs)
).all()
```

### Example 2: Recommend Simulator Environment

```python
# Get user from FastAPI request
user_id = request.user.id
user = db.query(UserProfile).filter_by(id=user_id).first()

# Check hardware and recommend simulator
recommended_sim = user.get_recommended_simulation_environment()
# → "isaac_sim" if GPU available, else "gazebo"

# Return in lab response
lab_config = {
    "simulator": recommended_sim,
    "docker_image": f"robotics-lab:{recommended_sim}-latest",
    "estimated_duration_seconds": 1800
}
```

### Example 3: Filter Content by Hardware Capability

```python
# User with only Gazebo capability
user = UserProfile(
    programming_level="advanced",
    robotics_experience="basic",
    has_rtx=False,
    has_jetson=False
)

# Exclude Isaac Sim-only labs
labs = db.query(Lab).filter(
    Lab.required_simulator != "isaac_sim"  # or custom field
).all()

# Or recommend Gazebo alternative
if user.get_recommended_simulation_environment() == "gazebo":
    labs = db.query(Lab).filter(Lab.tags.contains(["gazebo"])).all()
```

### Example 4: Backend Content Personalization

```python
# In a content serving endpoint
@router.get("/chapters/{chapter_id}/content")
async def get_chapter_content(
    chapter_id: int,
    user_id: str,
    db: Session = Depends(get_db)
):
    user = db.query(UserProfile).filter_by(id=user_id).first()
    chapter = db.query(Chapter).filter_by(id=chapter_id).first()

    # Get difficulty-appropriate content
    difficulty = user.get_difficulty_level()

    # Return content with personalization markers
    return {
        "chapter": chapter.title,
        "content": chapter.content,
        "personalization": {
            "difficulty_level": difficulty,
            "recommended_labs": user.get_recommended_labs(),
            "hardware_summary": user.get_hardware_summary(),
            "suggested_simulator": user.get_recommended_simulation_environment()
        }
    }
```

---

## Database Constraints & Validation

### CHECK Constraints

```sql
-- Validate enum values
programming_level IN ('beginner','intermediate','advanced')
ai_experience IN ('none','basic','applied')
robotics_experience IN ('none','basic','advanced')
```

### Unique Constraints

```sql
-- Email must be unique (one profile per email)
UNIQUE(email)
```

### Indexes for Performance

```sql
-- Fast lookups by email
CREATE INDEX idx_user_profiles_email ON user_profiles(email);

-- Filter by background level
CREATE INDEX idx_user_profiles_programming_level ON user_profiles(programming_level);
CREATE INDEX idx_user_profiles_robotics_experience ON user_profiles(robotics_experience);

-- Hardware capability queries
CREATE INDEX idx_user_profiles_hardware ON user_profiles(has_rtx, has_jetson, has_robot);
```

---

## Example User Profiles

### Profile 1: Student with RTX GPU

```python
student = UserProfile(
    email="student@university.edu",
    programming_level="intermediate",
    ai_experience="basic",
    robotics_experience="none",
    has_rtx=True,
    gpu_model="RTX 4080",
    rtx_vram_gb="16GB",
    has_jetson=False,
    has_robot=False,
    preferred_language="en",
    learning_goal="Learn RL for robot control"
)

# Properties:
# difficulty_level → "intermediate"
# recommended_labs → ["beginner", "intermediate"]
# can_run_vla_training() → False (only 16GB, need ≥24GB)
# can_run_isaac_sim() → True
# recommended_simulator → "isaac_sim"
```

### Profile 2: Researcher with Full Hardware

```python
researcher = UserProfile(
    email="researcher@lab.org",
    programming_level="advanced",
    ai_experience="applied",
    robotics_experience="advanced",
    has_rtx=True,
    gpu_model="RTX 4090",
    rtx_vram_gb="48GB",
    has_jetson=True,
    jetson_model="Jetson AGX Orin",
    has_robot=True,
    robot_type="Unitree G1",
    robot_specs={
        "model": "G1",
        "year": 2025,
        "dof": 43,
        "weight_kg": 35
    },
    preferred_language="en",
    learning_goal="Implement end-to-end learning for humanoid locomotion"
)

# Properties:
# difficulty_level → "advanced"
# recommended_labs → ["beginner", "intermediate", "advanced"]
# can_run_vla_training() → True
# can_run_isaac_sim() → True
# has_edge_deployment_capability() → True
# has_real_robot_access() → True
# recommended_simulator → "isaac_sim"
```

### Profile 3: Developer Without Hardware

```python
developer = UserProfile(
    email="dev@startup.com",
    programming_level="advanced",
    ai_experience="none",
    robotics_experience="none",
    has_rtx=False,
    has_jetson=False,
    has_robot=False,
    preferred_language="en",
    learning_goal="Understand robotics fundamentals"
)

# Properties:
# difficulty_level → "intermediate" (programming advanced but no robotics/ai)
# recommended_labs → ["beginner", "intermediate"]
# can_run_vla_training() → False
# can_run_isaac_sim() → False
# recommended_simulator → "gazebo" (default, no GPU needed)
```

---

## Migration Guide (Adding New Fields)

To add new fields to UserProfile:

### Step 1: Update Schema
```sql
ALTER TABLE user_profiles
ADD COLUMN new_field VARCHAR(100) DEFAULT 'default_value';

CREATE INDEX idx_user_profiles_new_field ON user_profiles(new_field);
```

### Step 2: Update SQLAlchemy Model
```python
class UserProfile(Base):
    new_field = Column(String(100), default="default_value")
```

### Step 3: Update Pydantic Schemas
```python
class UserProfileCreate(BaseModel):
    new_field: str = "default_value"

class UserProfileUpdate(BaseModel):
    new_field: Optional[str] = None

class UserProfileResponse(BaseModel):
    new_field: str
```

### Step 4: Update Endpoints
Add field to response serialization in FastAPI endpoints.

---

## Best Practices

1. **Always validate email format** (Pydantic's `EmailStr` enforces this)
2. **Use checked values for enums** (programming_level, ai_experience, robotics_experience)
3. **Prefer strings for hardware specs** (e.g., "24GB" instead of integer, allows flexibility)
4. **Use JSONB for extensible data** (robot_specs, content_preferences allow future fields)
5. **Index frequently-queried fields** (email, background levels, hardware flags)
6. **Keep last_login updated** for user analytics
7. **Soft delete in production** (add `deleted_at` field instead of hard delete)
8. **Cache difficulty calculations** in frontend (rarely changes)

---

## Support & Further Reading

- **SQLAlchemy Docs**: https://docs.sqlalchemy.org/
- **PostgreSQL JSON**: https://www.postgresql.org/docs/current/datatype-json.html
- **FastAPI Validation**: https://fastapi.tiangolo.com/tutorial/body/
- **Pydantic Models**: https://docs.pydantic.dev/

---

**Last Updated**: 2025-12-31
**Status**: Production Ready
**Version**: 1.0.0
