# Onboarding System Documentation

## Overview

The Onboarding System is a comprehensive questionnaire that collects user background, hardware capabilities, and learning goals to personalize the Physical AI textbook experience.

**Key Features**:
- Multi-step form with progress tracking
- Intelligent recommendations based on profile
- Hardware capability detection
- Adaptive learning path suggestions
- Persistent user profiles

---

## Questionnaire Structure

The onboarding questionnaire consists of **7 questions** across **5 steps**:

### Step 1: Basic Information
- Email (required, unique)
- First name (optional)
- Last name (optional)

### Step 2: Experience Levels
- **Programming Level** (required)
  - Beginner
  - Intermediate
  - Advanced
- **AI Experience** (required)
  - None
  - Basic (LLMs, APIs)
  - Applied (agents, RAG, robotics)

### Step 3: Robotics & Goals
- **Robotics Experience** (required)
  - None
  - Basic (ROS concepts)
  - Advanced (controllers, kinematics)
- **Primary Learning Goal** (required)
  - Learn core concepts
  - Build simulations (Gazebo/Isaac)
  - Deploy on real robots

### Step 4: Hardware Capabilities
- **RTX GPU** (required)
  - Yes / No
  - If yes: Model, VRAM
- **Jetson Hardware** (required)
  - None
  - Orin Nano (8GB RAM, ~40 TFLOPS)
  - Orin NX (8GB RAM, ~100 TFLOPS)
  - AGX Orin (64GB/96GB RAM, 1.5 PFLOPS)
- **Physical Robot** (required)
  - None
  - Quadruped (e.g., Unitree G1, Spot)
  - Humanoid (e.g., Tesla Optimus, Atlas)
  - Robotic Arm (e.g., UR10e, Franka)

### Step 5: Review
- Summary of all responses
- Option to go back and edit

---

## Backend Architecture

### Pydantic Schemas (`backend/src/schemas/onboarding.py`)

#### Enums

```python
class ProgrammingLevel(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"

class AIExperience(str, Enum):
    NONE = "none"
    BASIC = "basic"
    APPLIED = "applied"

class RoboticsExperience(str, Enum):
    NONE = "none"
    BASIC = "basic"
    ADVANCED = "advanced"

class JetsonModel(str, Enum):
    NONE = "none"
    ORIN_NANO = "orin_nano"
    ORIN_NX = "orin_nx"
    ORIN_AGX = "orin_agx"

class RobotType(str, Enum):
    NONE = "none"
    QUADRUPED = "quadruped"
    HUMANOID = "humanoid"
    ROBOTIC_ARM = "robotic_arm"

class LearningGoal(str, Enum):
    CONCEPTS = "concepts"
    SIMULATIONS = "simulations"
    REAL_ROBOTS = "real_robots"
```

#### Request Schema

```python
class OnboardingQuestionnaireRequest(BaseModel):
    email: EmailStr
    first_name: Optional[str]
    last_name: Optional[str]
    programming_level: ProgrammingLevel
    ai_experience: AIExperience
    robotics_experience: RoboticsExperience
    has_rtx_gpu: bool
    rtx_gpu_model: Optional[GPUModel]
    rtx_gpu_vram_gb: Optional[int]
    jetson_model: JetsonModel
    robot_type: RobotType
    robot_model: Optional[str]
    learning_goal: LearningGoal
    additional_interests: Optional[List[str]]
    comments: Optional[str]
```

#### Response Schema

```python
class OnboardingQuestionnaireResponse(BaseModel):
    user_id: str
    email: str
    difficulty_level: str  # Calculated
    recommendations: dict  # Personalized recommendations
    # ... other fields
```

### API Endpoints

**Base URL**: `/api/v1/onboarding`

#### 1. Submit Questionnaire
```
POST /api/v1/onboarding/questionnaire
Content-Type: application/json

Request:
{
    "email": "learner@example.com",
    "first_name": "Alex",
    "programming_level": "intermediate",
    "ai_experience": "basic",
    "robotics_experience": "none",
    "has_rtx_gpu": true,
    "rtx_gpu_model": "RTX 4080",
    "rtx_gpu_vram_gb": 16,
    "jetson_model": "none",
    "robot_type": "none",
    "learning_goal": "simulations"
}

Response (201 Created):
{
    "user_id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "learner@example.com",
    "difficulty_level": "intermediate",
    "recommendations": {
        "difficulty_level": "intermediate",
        "recommended_labs": ["beginner", "intermediate"],
        "recommended_simulator": "isaac_sim",
        "can_train_vla_models": false,
        "can_use_isaac_sim": true,
        "can_deploy_to_jetson": false,
        "can_validate_on_real_robot": false,
        "suggested_learning_path": [
            "Chapter 1: Introduction to Physical AI",
            "Chapter 2: Sensors & Perception",
            "Chapter 3: Control Systems",
            "Chapter 4: Simulation Fundamentals",
            ...
        ],
        "warnings": [
            "RTX GPU has only 16GB VRAM; VLA training requires 24GB+",
            "No Jetson device; edge deployment labs will be simulation-only"
        ]
    }
}
```

#### 2. Get Form Options
```
GET /api/v1/onboarding/form-options

Response (200 OK):
{
    "programming_level": [
        {
            "value": "beginner",
            "label": "Beginner",
            "description": "New to programming or robotics"
        },
        ...
    ],
    "ai_experience": [...],
    "robotics_experience": [...],
    "gpu": [...],
    "rtx_gpu_model": [...],
    "jetson_model": [...],
    "robot_type": [...],
    "learning_goal": [...]
}
```

#### 3. Get User Recommendations
```
GET /api/v1/onboarding/recommendations/{user_id}

Response (200 OK):
{
    "difficulty_level": "intermediate",
    "recommended_labs": ["beginner", "intermediate"],
    "recommended_simulator": "isaac_sim",
    ...
}
```

---

## Frontend Implementation

### React Component (`website/src/components/OnboardingQuestionnaire.jsx`)

**Features**:
- 5-step multi-step form
- Progress bar
- Conditional field display (e.g., GPU model only if has_rtx_gpu = true)
- Form validation
- Error handling
- Persistent storage (localStorage)

**Usage**:
```jsx
import OnboardingQuestionnaire from '@site/src/components/OnboardingQuestionnaire';

export default function OnboardingPage() {
  const handleComplete = (result) => {
    console.log('User profile:', result);
    // Redirect to dashboard or chapter 1
    window.location.href = '/docs/chapter-1';
  };

  return (
    <OnboardingQuestionnaire onComplete={handleComplete} />
  );
}
```

**Data Flow**:
1. User fills out questionnaire (steps 1-5)
2. On submit, fetch POST `/api/v1/onboarding/questionnaire`
3. Backend calculates recommendations and creates UserProfile
4. Response stored in localStorage: `user_profile`, `user_recommendations`
5. Callback `onComplete()` invoked with result
6. Redirect to dashboard or first chapter

### Component Props

```jsx
<OnboardingQuestionnaire
  onComplete={(result) => {
    // result = OnboardingQuestionnaireResponse
    console.log(result.user_id);
    console.log(result.recommendations);
  }}
/>
```

---

## Recommendation Algorithm

### Difficulty Level Calculation

The system calculates difficulty based on **weighted background scores**:

```
Score = (Programming × 0.4) + (Robotics × 0.4) + (AI × 0.2)

Where:
  Programming: beginner=1, intermediate=2, advanced=3
  Robotics:    none=0,    basic=1,        advanced=2
  AI:          none=0,    basic=1,        applied=2

Result:
  score ≥ 2.2  →  "advanced"
  score ≥ 1.2  →  "intermediate"
  score <  1.2 →  "beginner"
```

**Examples**:

| Programming | Robotics | AI | Score | Level |
|---|---|---|---|---|
| Beginner | None | None | 0.4 | Beginner |
| Intermediate | None | None | 0.8 | Beginner |
| Intermediate | None | Basic | 1.0 | Intermediate |
| Intermediate | Basic | Basic | 1.4 | Intermediate |
| Advanced | Advanced | Applied | 2.6 | Advanced |

### Recommended Labs

Based on difficulty level:

```
Beginner      → ["beginner"]
Intermediate  → ["beginner", "intermediate"]
Advanced      → ["beginner", "intermediate", "advanced"]
```

### Recommended Simulator

Based on hardware:

```
If has RTX GPU with ≥12GB VRAM → "isaac_sim"
Else                            → "gazebo" (default)
```

### Hardware Capabilities

```python
can_train_vla_models = has_rtx_gpu AND rtx_gpu_vram_gb >= 24
can_use_isaac_sim = has_rtx_gpu AND rtx_gpu_vram_gb >= 12
can_deploy_to_jetson = jetson_model != "none"
can_validate_on_real_robot = robot_type != "none"
```

### Learning Path Suggestions

The system suggests a chapter sequence based on **learning goal** and **capabilities**:

#### Goal: Learn Core Concepts
```
Chapter 1: Introduction to Physical AI
Chapter 2: Sensors & Perception
Chapter 3: Control Systems
Chapter 4: Simulation Fundamentals
Chapter 5: Learning from Simulation
Chapter 6: Real-World Constraints
```

#### Goal: Build Simulations
```
Chapter 1: Introduction to Physical AI
Chapter 2: Sensors & Perception
Chapter 3: Control Systems
Chapter 4: Simulation Fundamentals
Chapter 7: ROS 2 Advanced
Chapter 8: Gazebo for Roboticists
[Optional] Chapter 9: Vision-Language-Action (if can_train_vla)
```

#### Goal: Deploy on Real Robots
```
Chapter 1: Introduction to Physical AI
Chapter 2: Sensors & Perception
Chapter 3: Control Systems
Chapter 4: Simulation Fundamentals
Chapter 7: ROS 2 Advanced
Chapter 10: From Simulation to Reality
[Optional] Chapter 11: Real Robot Integration (if has_robot)
[Optional] Chapter 12: Edge Deployment (if has_jetson)
[Optional] Chapter 13: Learning-Based Control (if can_train_vla)
```

#### Extensions for Advanced Learners
```
Chapter 14: Multi-Robot Systems
Chapter 15: Advanced Control Theory
```

### Warnings

The system generates contextual warnings:

```python
if has_rtx_gpu and rtx_gpu_vram_gb < 24:
    warnings.append("RTX GPU only has {vram}GB; VLA training needs 24GB+")

if has_rtx_gpu and rtx_gpu_vram_gb < 12:
    warnings.append("RTX GPU only has {vram}GB; Isaac Sim needs 12GB+")

if not has_rtx_gpu:
    warnings.append("No GPU; using Gazebo (lightweight but lower fidelity)")

if not jetson_model:
    warnings.append("No Jetson device; edge deployment labs simulation-only")

if not robot_type:
    warnings.append("No physical robot; validation limited to simulation")
```

---

## Integration with User Profiles

The onboarding questionnaire creates a **UserProfile** record:

```python
UserProfile(
    email=questionnaire.email,
    programming_level=questionnaire.programming_level.value,
    ai_experience=questionnaire.ai_experience.value,
    robotics_experience=questionnaire.robotics_experience.value,
    has_rtx=questionnaire.has_rtx_gpu,
    gpu_model=questionnaire.rtx_gpu_model.value,
    rtx_vram_gb=f"{questionnaire.rtx_gpu_vram_gb}GB",
    has_jetson=questionnaire.jetson_model.value != "none",
    jetson_model=questionnaire.jetson_model.value,
    has_robot=questionnaire.robot_type.value != "none",
    robot_type=questionnaire.robot_type.value,
    learning_goal=questionnaire.learning_goal.value,
    content_preferences={
        "learning_goal": questionnaire.learning_goal.value,
        "additional_interests": questionnaire.additional_interests or []
    }
)
```

The profile inherits all methods from UserProfile:
- `get_difficulty_level()`
- `get_recommended_labs()`
- `get_recommended_simulation_environment()`
- `can_run_vla_training()`
- `can_run_isaac_sim()`
- `has_edge_deployment_capability()`
- `has_real_robot_access()`

---

## Data Persistence

### localStorage (Client-Side)

After successful onboarding, data is stored in browser localStorage:

```javascript
// User profile
localStorage.setItem('user_profile', JSON.stringify({
    user_id: "...",
    email: "...",
    difficulty_level: "...",
    ...
}));

// Recommendations
localStorage.setItem('user_recommendations', JSON.stringify({
    difficulty_level: "intermediate",
    recommended_labs: [...],
    ...
}));
```

**Usage**:
```javascript
const profile = JSON.parse(localStorage.getItem('user_profile'));
const recommendations = JSON.parse(localStorage.getItem('user_recommendations'));
```

### Database (Server-Side)

UserProfile record persists in PostgreSQL:
- Can be updated anytime via `/api/v1/user-profiles/{user_id}` endpoints
- Supports soft delete (add `deleted_at` field in future)
- All fields indexed for fast queries

---

## Example User Profiles

### Profile 1: Student with RTX GPU

```json
{
    "email": "student@university.edu",
    "first_name": "Alex",
    "programming_level": "intermediate",
    "ai_experience": "basic",
    "robotics_experience": "none",
    "has_rtx_gpu": true,
    "rtx_gpu_model": "RTX 4080",
    "rtx_gpu_vram_gb": 16,
    "jetson_model": "none",
    "robot_type": "none",
    "learning_goal": "simulations",
    "difficulty_level": "intermediate",
    "recommendations": {
        "difficulty_level": "intermediate",
        "recommended_labs": ["beginner", "intermediate"],
        "recommended_simulator": "isaac_sim",
        "can_train_vla_models": false,
        "can_use_isaac_sim": true,
        "can_deploy_to_jetson": false,
        "can_validate_on_real_robot": false,
        "suggested_learning_path": [
            "Chapter 1: Introduction to Physical AI",
            "Chapter 2: Sensors & Perception",
            "Chapter 3: Control Systems",
            "Chapter 4: Simulation Fundamentals",
            "Chapter 7: ROS 2 Advanced",
            "Chapter 8: Gazebo for Roboticists"
        ],
        "warnings": [
            "RTX GPU has only 16GB VRAM; VLA training requires 24GB+",
            "No Jetson device; edge deployment labs will be simulation-only",
            "No physical robot detected; validation will be limited to simulation"
        ]
    }
}
```

### Profile 2: Researcher with Full Stack

```json
{
    "email": "researcher@lab.org",
    "first_name": "Dr.",
    "last_name": "Smith",
    "programming_level": "advanced",
    "ai_experience": "applied",
    "robotics_experience": "advanced",
    "has_rtx_gpu": true,
    "rtx_gpu_model": "RTX 4090",
    "rtx_gpu_vram_gb": 24,
    "jetson_model": "orin_agx",
    "robot_type": "humanoid",
    "robot_model": "Unitree G1",
    "learning_goal": "real_robots",
    "difficulty_level": "advanced",
    "recommendations": {
        "difficulty_level": "advanced",
        "recommended_labs": ["beginner", "intermediate", "advanced"],
        "recommended_simulator": "isaac_sim",
        "can_train_vla_models": true,
        "can_use_isaac_sim": true,
        "can_deploy_to_jetson": true,
        "can_validate_on_real_robot": true,
        "suggested_learning_path": [
            "Chapter 1: Introduction to Physical AI",
            "Chapter 2: Sensors & Perception",
            "Chapter 3: Control Systems",
            "Chapter 4: Simulation Fundamentals",
            "Chapter 7: ROS 2 Advanced",
            "Chapter 10: From Simulation to Reality",
            "Chapter 11: Real Robot Integration",
            "Chapter 12: Edge Deployment (Jetson)",
            "Chapter 13: Learning-Based Control",
            "Chapter 14: Multi-Robot Systems",
            "Chapter 15: Advanced Control Theory"
        ],
        "warnings": []
    }
}
```

### Profile 3: Developer Without Hardware

```json
{
    "email": "dev@startup.com",
    "first_name": "Jordan",
    "programming_level": "advanced",
    "ai_experience": "none",
    "robotics_experience": "none",
    "has_rtx_gpu": false,
    "rtx_gpu_model": null,
    "rtx_gpu_vram_gb": null,
    "jetson_model": "none",
    "robot_type": "none",
    "learning_goal": "concepts",
    "difficulty_level": "intermediate",
    "recommendations": {
        "difficulty_level": "intermediate",
        "recommended_labs": ["beginner", "intermediate"],
        "recommended_simulator": "gazebo",
        "can_train_vla_models": false,
        "can_use_isaac_sim": false,
        "can_deploy_to_jetson": false,
        "can_validate_on_real_robot": false,
        "suggested_learning_path": [
            "Chapter 1: Introduction to Physical AI",
            "Chapter 2: Sensors & Perception",
            "Chapter 3: Control Systems",
            "Chapter 4: Simulation Fundamentals",
            "Chapter 5: Learning from Simulation",
            "Chapter 6: Real-World Constraints"
        ],
        "warnings": [
            "No RTX GPU detected; using Gazebo for simulation (lighter weight, good for learning, but lower fidelity)",
            "No Jetson hardware detected; edge deployment labs will be simulation-only",
            "No physical robot detected; validation will be limited to simulation"
        ]
    }
}
```

---

## Testing

### Manual Testing Checklist

- [ ] Form renders all 5 steps
- [ ] Progress bar updates correctly
- [ ] Back/Next buttons work
- [ ] Form validation enforces required fields
- [ ] GPU VRAM input only shows when "has_rtx_gpu" = true
- [ ] Robot model input only shows when "robot_type" != "none"
- [ ] Submit creates UserProfile record
- [ ] User localStorage updated with profile & recommendations
- [ ] Callback invoked with correct data

### API Testing

```bash
# Test form options endpoint
curl http://localhost:8000/api/v1/onboarding/form-options

# Test questionnaire submission
curl -X POST http://localhost:8000/api/v1/onboarding/questionnaire \
  -H "Content-Type: application/json" \
  -d '{
    "email": "test@example.com",
    "programming_level": "intermediate",
    "ai_experience": "basic",
    "robotics_experience": "none",
    "has_rtx_gpu": true,
    "rtx_gpu_model": "RTX 4080",
    "rtx_gpu_vram_gb": 16,
    "jetson_model": "none",
    "robot_type": "none",
    "learning_goal": "simulations"
  }'

# Get recommendations for user
curl http://localhost:8000/api/v1/onboarding/recommendations/{user_id}
```

---

## Future Enhancements

1. **Multi-language onboarding** (currently English only)
2. **Progressive profiling** (ask fewer questions, get recommendations later)
3. **Profile editing** (allow users to update after initial onboarding)
4. **Dynamic recommendations** (update suggestions based on progress)
5. **Cohort matching** (connect learners with similar goals)
6. **Hardware borrowing** (suggest labs for users without hardware)
7. **Skill assessments** (quiz-based difficulty calibration)

---

## Support & Resources

- **Backend Code**: `backend/src/schemas/onboarding.py`, `backend/src/api/onboarding.py`
- **Frontend Code**: `website/src/components/OnboardingQuestionnaire.jsx`
- **User Profiles**: `USER_PROFILE_DOCUMENTATION.md`
- **API Docs**: http://localhost:8000/docs (Swagger UI)

---

**Last Updated**: 2025-12-31
**Status**: Production Ready
**Version**: 1.0.0
