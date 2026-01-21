# Personalization System Guide

## Overview

The **Personalization Engine** adapts chapter content in real-time based on user profiles. Different users see different content:

- 🎓 **Beginners** see simplified math, additional examples, ROS 2 primers
- 🚀 **Advanced learners** see challenges, research connections, advanced math
- 💻 **GPU users** see Isaac Sim examples; **non-GPU users** see Gazebo alternatives
- 🤖 **Robotics beginners** get ROS 2 primers; **advanced** see deep dives
- ☁️ **No GPU users** see cloud simulation paths

---

## Core Concept

```python
def personalize(chapter, user):
    if user.programming_level == "beginner":
        chapter.add("Foundations Box")
        chapter.hide("Advanced Math")

    if not user.has_rtx:
        chapter.switch_to("Cloud Simulation Path")

    if user.robotics_experience == "none":
        chapter.insert("ROS Primer")

    return chapter
```

---

## Architecture

### Personalization Flow

```
┌──────────────────┐
│  Raw Chapter     │
│  (all content)   │
└────────┬─────────┘
         │
         ▼
┌──────────────────┐
│  User Profile    │
│  (background,    │
│   hardware)      │
└────────┬─────────┘
         │
         ▼
┌──────────────────────────────────┐
│ Personalization Engine           │
├──────────────────────────────────┤
│ 1. Filter sections by level      │
│ 2. Transform content             │
│ 3. Insert prerequisites          │
│ 4. Generate warnings             │
│ 5. Calculate read time           │
└────────┬─────────────────────────┘
         │
         ▼
┌──────────────────┐
│ Personalized     │
│ Chapter          │
│ (user-specific)  │
└──────────────────┘
```

### Key Components

**ContentSection**: Individual piece of content with metadata
```python
@dataclass
class ContentSection:
    id: str                        # "intro", "math", "code"
    title: str                     # "Introduction"
    content: str                   # Actual text
    difficulty: ContentDifficulty  # beginner, intermediate, advanced
    section_type: str             # theory, math, code, example, challenge
    estimated_read_time_minutes: int
```

**ContentTransformation**: Changes applied to content
```python
@dataclass
class ContentTransformation:
    type: str                     # hide, show, insert, replace, simplify, amplify
    section_id: str              # Which section
    reason: str                  # Why the change
```

**PersonalizedChapter**: Final output for user
```python
@dataclass
class PersonalizedChapter:
    chapter_id: int
    sections: List[ContentSection]        # After filtering
    transformations: List[ContentTransformation]
    total_estimated_read_time: int
    recommended_simulator: str
    recommended_labs: List[str]
    warnings: List[str]
```

---

## Transformation Types

### 1. Hide
**When**: Content irrelevant to user
**Examples**:
- Hide advanced math for beginners
- Hide Isaac Sim for GPU-less users
- Hide real robot content for simulator-only users

```python
ContentTransformation(
    type="hide",
    section_id="advanced_math",
    reason="Advanced mathematics hidden for beginner level"
)
```

### 2. Replace
**When**: Alternative exists for similar concept
**Examples**:
- Replace Isaac Sim with Gazebo
- Replace cloud simulation with local options

```python
ContentTransformation(
    type="replace",
    section_id="isaac_sim_section",
    reason="Isaac Sim replaced with Gazebo (no GPU available)"
)
```

### 3. Simplify
**When**: Content should be made easier
**Examples**:
- Replace calculus notation with verbal descriptions
- Remove advanced control theory for beginners

```python
ContentTransformation(
    type="simplify",
    section_id="math_section",
    reason="Mathematical notation simplified for beginner level"
)
```

### 4. Amplify
**When**: Content needs more detail for learner
**Examples**:
- Add extra examples for beginners
- Add step-by-step walkthroughs

```python
ContentTransformation(
    type="amplify",
    section_id="examples",
    reason="Additional examples added for beginner level"
)
```

### 5. Insert
**When**: New content should be added
**Examples**:
- Insert ROS 2 primer for robotics beginners
- Insert Python refresher for programming beginners
- Insert cloud simulation path for non-GPU users

```python
ContentTransformation(
    type="insert",
    section_id="ros2_primer",
    reason="ROS 2 primer inserted - user has no robotics experience"
)
```

---

## Filtering Logic

### Section Inclusion Criteria

```python
if section.difficulty == ADVANCED and user.level == BEGINNER:
    HIDE  # Too hard

if "isaac_sim" in section and not user.has_gpu:
    HIDE or REPLACE  # Can't run

if "jetson" in section and not user.has_jetson:
    HIDE  # Can't use

if "real robot" in section and not user.has_robot:
    HIDE or INFO_ONLY  # Can't practice

if section.type == "challenge" and user.level == BEGINNER:
    HIDE  # Too difficult
```

---

## Prerequisite Insertion

Automatically add primers if user lacks background:

### 1. ROS 2 Primer
**When**: `robotics_experience == "none"`
**Content**:
- What is ROS 2?
- Nodes, topics, pub/sub
- Simple example

**Read Time**: 15 min

### 2. Python Refresher
**When**: `programming_level == "beginner"`
**Content**:
- Variables and types
- Functions
- Control flow
- Lists

**Read Time**: 20 min

### 3. Simulation Fundamentals
**When**: `learning_goal == "simulations"`
**Content**:
- Why simulate?
- Physics simulation
- Sim-to-real gap
- Available tools

**Read Time**: 10 min

---

## Transformation Examples

### Example 1: Beginner Physics Student, No GPU

**User Profile**:
- Programming: Intermediate
- AI Experience: None
- Robotics: None
- GPU: No

**Transformations Applied**:

```
HIDE "Advanced Control Theory"
  → Too mathematical, beginner level

REPLACE "Isaac Sim simulation" with "Gazebo"
  → No GPU available

INSERT "ROS 2 Primer"
  → New to robotics

INSERT "Simulation Fundamentals"
  → Learning goal is simulations

INSERT "Cloud GPU Path"
  → Intermediate learner might need GPU later
```

**Result**:
- Original: 12 sections, 90 min read
- Personalized: 15 sections (3 primers added), 125 min read
- 2 sections hidden, 3 sections modified

---

### Example 2: Advanced Researcher with Full Stack

**User Profile**:
- Programming: Advanced
- AI Experience: Applied
- Robotics: Advanced
- GPU: RTX 4090 (24GB)
- Jetson: Yes (AGX Orin)
- Robot: Yes (Unitree G1)

**Transformations Applied**:

```
SHOW "Advanced Control Theory"
  → Advanced level

SHOW "Isaac Sim Deep Dive"
  → 24GB GPU available

SHOW "Real Robot Integration"
  → Has physical robot

SHOW "Jetson Deployment"
  → Has Jetson hardware

AMPLIFY "Research Connections"
  → Advanced learner wants cutting-edge
```

**Result**:
- Original: 12 sections, 90 min read
- Personalized: 14 sections (2 advanced sections added), 180 min read
- 0 sections hidden, all sections shown
- Advanced challenges included

---

## API Endpoints

### 1. Personalize Chapter
```
POST /api/v1/personalization/chapters/{chapter_id}
Authorization: Bearer <token>
Content-Type: application/json

Request:
{
    "id": 1,
    "title": "Introduction to Physical AI",
    "sections": [
        {
            "id": "intro",
            "title": "Introduction",
            "content": "Physical AI is...",
            "difficulty": "beginner",
            "type": "theory",
            "read_time_minutes": 10
        },
        {
            "id": "math",
            "title": "Advanced Math",
            "content": "∇f = ∂f/∂x + ∂f/∂y",
            "difficulty": "advanced",
            "type": "math",
            "read_time_minutes": 20
        }
    ]
}

Response:
{
    "chapter_id": 1,
    "chapter_title": "Introduction to Physical AI",
    "user_difficulty_level": "beginner",
    "sections": [
        // Filtered & transformed sections
    ],
    "transformations": [
        {
            "type": "hide",
            "section_id": "math",
            "reason": "Advanced mathematics hidden for beginner level"
        }
    ],
    "total_estimated_read_time": 45,
    "recommended_simulator": "gazebo",
    "recommended_labs": ["beginner"],
    "warnings": [
        "⚠️ No GPU detected: This chapter uses Gazebo...",
        "📚 ROS 2 Primer included: Start here if new to robotics!"
    ]
}
```

### 2. Get Personalization Stats
```
GET /api/v1/personalization/chapters/{chapter_id}/stats

Response:
{
    "total_sections": 15,
    "total_read_time_minutes": 125,
    "transformations_applied": 5,
    "warnings_count": 2,
    "user_difficulty_level": "beginner",
    "recommended_simulator": "gazebo",
    "recommended_labs": ["beginner"]
}
```

### 3. Get User Recommendations
```
GET /api/v1/personalization/profile/recommendations
Authorization: Bearer <token>

Response:
{
    "user_id": "550e8400-e29b-41d4-a716-446655440000",
    "email": "learner@example.com",
    "difficulty_level": "intermediate",
    "recommended_labs": ["beginner", "intermediate"],
    "recommended_simulator": "isaac_sim",
    "hardware_summary": {
        "gpu": {
            "available": true,
            "model": "RTX 4080",
            "can_vla_train": false
        },
        "jetson": {"available": false},
        "robot": {"available": false}
    },
    "can_train_vla": false,
    "can_use_isaac_sim": true,
    "has_jetson": false,
    "has_robot": false
}
```

### 4. DEBUG: Get Transformation Details
```
POST /api/v1/personalization/debug/apply-transformation

Response:
{
    "user_profile": {
        "difficulty_level": "beginner",
        "programming_level": "intermediate",
        "ai_experience": "none",
        "robotics_experience": "none",
        "has_gpu": false,
        "has_jetson": false,
        "has_robot": false,
        "learning_goal": "simulations"
    },
    "original_sections": 12,
    "personalized_sections": 15,
    "sections_hidden": 2,
    "transformations": [
        {
            "type": "insert",
            "section_id": "ros2-primer",
            "reason": "ROS 2 primer inserted - user has no robotics experience"
        },
        // ...
    ],
    "total_read_time": 125,
    "warnings": [
        "⚠️ No GPU detected: This chapter uses Gazebo..."
    ],
    "recommended_simulator": "gazebo",
    "recommended_labs": ["beginner"]
}
```

---

## Implementation Example: Frontend

### React Hook for Personalization

```javascript
function usePersonalizedChapter(chapterId, chapter) {
    const [personalized, setPersonalized] = useState(null);
    const [loading, setLoading] = useState(true);

    useEffect(() => {
        async function fetchPersonalized() {
            const token = localStorage.getItem('access_token');
            const response = await fetch(
                `/api/v1/personalization/chapters/${chapterId}`,
                {
                    method: 'POST',
                    headers: {
                        'Authorization': `Bearer ${token}`,
                        'Content-Type': 'application/json'
                    },
                    body: JSON.stringify(chapter)
                }
            );

            if (response.ok) {
                const data = await response.json();
                setPersonalized(data);
            }
            setLoading(false);
        }

        fetchPersonalized();
    }, [chapterId]);

    return { personalized, loading };
}

function ChapterPage({ chapterId, chapter }) {
    const { personalized, loading } = usePersonalizedChapter(chapterId, chapter);

    if (loading) return <div>Loading personalized content...</div>;
    if (!personalized) return <div>Error loading chapter</div>;

    return (
        <div>
            <h1>{personalized.chapter_title}</h1>

            {/* Show warnings */}
            {personalized.warnings.length > 0 && (
                <div className="warnings">
                    {personalized.warnings.map((w, i) => (
                        <div key={i} className="warning-box">{w}</div>
                    ))}
                </div>
            )}

            {/* Show sections */}
            {personalized.sections.map(section => (
                <div key={section.id} className="section">
                    <h2>{section.title}</h2>
                    <p>Read time: {section.estimated_read_time_minutes} min</p>
                    <div>{section.content}</div>
                </div>
            ))}

            {/* Show stats */}
            <div className="stats">
                Total reading time: {personalized.total_estimated_read_time} min
                Recommended simulator: {personalized.recommended_simulator}
            </div>
        </div>
    );
}
```

---

## Performance Considerations

### Caching
- Cache personalized chapters per user (15 min TTL)
- Invalidate when user profile changes
- Pre-calculate for popular chapters

### Computation
- Personalization is O(n) where n = sections
- Typical chapter: 50-100 sections → <100ms
- No database calls during personalization

### Storage
- Store transformations for analytics
- Track which content is hidden/shown per user
- Feed into recommendation system

---

## Future Enhancements

1. **Learning Path Recommendations**
   - Suggest chapter sequence based on goals
   - Adaptive sequencing based on assessment scores

2. **Content Variants**
   - Multiple explanations per concept
   - Interactive vs. passive learning
   - Video vs. text-based

3. **Progressive Difficulty**
   - Start at beginner, unlock advanced
   - Adaptive based on quiz performance

4. **A/B Testing**
   - Test different transformations
   - Measure comprehension outcomes
   - Optimize variants over time

5. **ML-Driven Personalization**
   - Use learning history to predict difficulty
   - Recommend sections dynamically
   - Predict time needed for completion

---

## Testing

### Test Cases

```python
def test_hide_advanced_math_for_beginners():
    user = UserProfile(
        programming_level="beginner",
        robotics_experience="none"
    )
    chapter = create_chapter_with_advanced_math()

    personalized = personalize(chapter, user)

    assert not any(s.id == "advanced_math" for s in personalized.sections)

def test_insert_ros_primer_for_robotics_beginners():
    user = UserProfile(robotics_experience="none")
    chapter = create_chapter()

    personalized = personalize(chapter, user)

    assert any("ros2-primer" in s.id for s in personalized.sections)

def test_replace_isaac_sim_for_no_gpu_users():
    user = UserProfile(has_rtx=False)
    chapter = create_isaac_sim_chapter()

    personalized = personalize(chapter, user)

    transformations = [t for t in personalized.transformations if t.type == "replace"]
    assert len(transformations) > 0
```

---

**Status**: ✅ Production Ready
**Version**: 1.0.0
**Last Updated**: 2025-12-31
