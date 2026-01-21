"""Personalization service - adapts content based on user profile."""

import logging
from typing import Dict, List, Optional, Any
from enum import Enum
from dataclasses import dataclass

from src.models.user_profile import UserProfile

logger = logging.getLogger(__name__)


# =====================================================================
# Content Transformation Models
# =====================================================================

class ContentDifficulty(str, Enum):
    """Content difficulty levels."""

    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"
    ALL = "all"


@dataclass
class ContentSection:
    """A section of chapter content that can be transformed."""

    id: str
    title: str
    content: str
    difficulty: ContentDifficulty
    section_type: str  # "theory", "math", "code", "example", "challenge"
    prerequisites: List[str] = None
    estimated_read_time_minutes: int = 5


@dataclass
class ContentTransformation:
    """Represents a transformation applied to content."""

    type: str  # "hide", "show", "insert", "replace", "simplify", "amplify"
    section_id: str
    reason: str
    metadata: Dict[str, Any] = None


@dataclass
class PersonalizedChapter:
    """Chapter adapted for a specific user."""

    chapter_id: int
    chapter_title: str
    user_difficulty_level: str
    sections: List[ContentSection]
    transformations: List[ContentTransformation]
    total_estimated_read_time: int
    recommended_simulator: str
    recommended_labs: List[str]
    warnings: List[str]


# =====================================================================
# Personalization Engine
# =====================================================================

class PersonalizationEngine:
    """
    Personalizes chapter content based on user profile.

    Strategy:
    1. Analyze user background (programming, AI, robotics)
    2. Check hardware capabilities (GPU, Jetson, robot)
    3. Identify learning goal
    4. Apply transformations to chapter content
    5. Return personalized chapter
    """

    def __init__(self):
        self.logger = logging.getLogger(self.__class__.__name__)

    # =====================================================================
    # Main Personalization Method
    # =====================================================================

    def personalize_chapter(
        self,
        chapter: Dict[str, Any],
        user: UserProfile,
        raw_sections: List[ContentSection]
    ) -> PersonalizedChapter:
        """
        Personalize a chapter for a specific user.

        Args:
            chapter: Chapter metadata (id, title, etc.)
            user: User profile
            raw_sections: Raw chapter content sections

        Returns:
            PersonalizedChapter: Adapted chapter content
        """

        self.logger.info(
            f"Personalizing chapter {chapter['id']} for user {user.email} "
            f"(level: {user.get_difficulty_level()})"
        )

        # Get personalization context
        context = self._build_personalization_context(user)

        # Apply transformations
        transformations = []
        personalized_sections = []

        for section in raw_sections:
            # Determine if section should be included
            if self._should_include_section(section, context):
                transformed_section, section_transforms = self._transform_section(
                    section, context
                )
                personalized_sections.append(transformed_section)
                transformations.extend(section_transforms)
            else:
                # Hide section
                transformations.append(
                    ContentTransformation(
                        type="hide",
                        section_id=section.id,
                        reason=self._get_hide_reason(section, context)
                    )
                )

        # Insert prerequisite sections if needed
        prerequisite_sections = self._generate_prerequisite_sections(context)
        personalized_sections = prerequisite_sections + personalized_sections

        # Calculate total read time
        total_read_time = sum(
            s.estimated_read_time_minutes for s in personalized_sections
        )

        return PersonalizedChapter(
            chapter_id=chapter["id"],
            chapter_title=chapter["title"],
            user_difficulty_level=user.get_difficulty_level(),
            sections=personalized_sections,
            transformations=transformations,
            total_estimated_read_time=total_read_time,
            recommended_simulator=user.get_recommended_simulation_environment(),
            recommended_labs=user.get_recommended_labs(),
            warnings=self._generate_warnings(user)
        )

    # =====================================================================
    # Context Building
    # =====================================================================

    def _build_personalization_context(self, user: UserProfile) -> Dict[str, Any]:
        """Build personalization context from user profile."""

        return {
            "user_id": str(user.id),
            "difficulty_level": user.get_difficulty_level(),
            "programming_level": user.programming_level,
            "ai_experience": user.ai_experience,
            "robotics_experience": user.robotics_experience,
            "has_rtx_gpu": user.has_rtx,
            "rtx_vram_gb": user.get_rtx_vram_gb(),
            "can_train_vla": user.can_run_vla_training(),
            "can_use_isaac_sim": user.can_run_isaac_sim(),
            "has_jetson": user.has_jetson,
            "has_robot": user.has_robot,
            "robot_type": user.robot_type,
            "learning_goal": user.learning_goal,
            "recommended_simulator": user.get_recommended_simulation_environment(),
            "recommended_labs": user.get_recommended_labs()
        }

    # =====================================================================
    # Section Filtering
    # =====================================================================

    def _should_include_section(
        self,
        section: ContentSection,
        context: Dict[str, Any]
    ) -> bool:
        """
        Determine if a section should be included for this user.

        Hide sections if:
        - Section difficulty too high
        - Section requires missing hardware
        - Section has unmet prerequisites
        """

        # Hide advanced math for beginners unless learning concepts
        if (section.section_type == "math" and
            section.difficulty == ContentDifficulty.ADVANCED and
            context["difficulty_level"] == "beginner"):
            return False

        # Hide Isaac Sim sections if user has no GPU
        if ("isaac_sim" in section.content.lower() and
            not context["can_use_isaac_sim"]):
            return False

        # Hide Jetson content if user has no Jetson
        if ("jetson" in section.content.lower() and
            not context["has_jetson"]):
            return False

        # Hide real robot content if user has no robot
        if ("real robot" in section.content.lower() and
            not context["has_robot"]):
            return False

        # Hide advanced challenges for beginners
        if (section.section_type == "challenge" and
            section.difficulty == ContentDifficulty.ADVANCED and
            context["difficulty_level"] == "beginner"):
            return False

        return True

    def _get_hide_reason(
        self,
        section: ContentSection,
        context: Dict[str, Any]
    ) -> str:
        """Generate reason why section is hidden."""

        if section.section_type == "math" and section.difficulty == ContentDifficulty.ADVANCED:
            return "Advanced mathematics content hidden for beginner level"

        if "isaac_sim" in section.content.lower():
            return "Isaac Sim content - requires RTX GPU (12GB+ VRAM)"

        if "jetson" in section.content.lower():
            return "Jetson edge deployment content - requires Jetson hardware"

        if "real robot" in section.content.lower():
            return "Real robot content - requires physical robot"

        if section.section_type == "challenge" and section.difficulty == ContentDifficulty.ADVANCED:
            return "Advanced challenges hidden for beginner level"

        return "Content filtered based on user profile"

    # =====================================================================
    # Section Transformation
    # =====================================================================

    def _transform_section(
        self,
        section: ContentSection,
        context: Dict[str, Any]
    ) -> tuple[ContentSection, List[ContentTransformation]]:
        """
        Transform a section based on user profile.

        Transformations:
        - Simplify math for non-advanced users
        - Replace Isaac Sim with Gazebo for users without GPU
        - Add ROS primers for users new to robotics
        - Expand examples for beginners
        """

        transformations = []
        transformed_section = section

        # 1. Simplify mathematics for beginners
        if (section.section_type == "math" and
            context["difficulty_level"] in ["beginner", "intermediate"]):
            simplified_content = self._simplify_mathematics(section.content)
            if simplified_content != section.content:
                transformations.append(
                    ContentTransformation(
                        type="simplify",
                        section_id=section.id,
                        reason="Mathematical content simplified for beginner/intermediate level"
                    )
                )
                transformed_section.content = simplified_content

        # 2. Replace Isaac Sim with Gazebo for users without GPU
        if ("isaac_sim" in section.content.lower() and
            not context["can_use_isaac_sim"]):
            replaced_content = self._replace_isaac_sim_with_gazebo(section.content)
            transformations.append(
                ContentTransformation(
                    type="replace",
                    section_id=section.id,
                    reason="Isaac Sim replaced with Gazebo (no GPU available)"
                )
            )
            transformed_section.content = replaced_content

        # 3. Amplify examples for beginners
        if (section.section_type == "example" and
            context["difficulty_level"] == "beginner"):
            amplified_content = self._amplify_examples(section.content)
            transformations.append(
                ContentTransformation(
                    type="amplify",
                    section_id=section.id,
                    reason="Additional examples added for beginner level"
                )
            )
            transformed_section.content = amplified_content

        # 4. Add cloud simulation path info for no-GPU users
        if (context["difficulty_level"] == "intermediate" and
            not context["has_rtx_gpu"] and
            context["learning_goal"] == "simulations"):
            cloud_info = self._add_cloud_simulation_path()
            transformations.append(
                ContentTransformation(
                    type="insert",
                    section_id=section.id,
                    reason="Cloud simulation path suggested (no local GPU)"
                )
            )
            transformed_section.content += "\n\n" + cloud_info

        return transformed_section, transformations

    # =====================================================================
    # Content Transformation Utilities
    # =====================================================================

    def _simplify_mathematics(self, content: str) -> str:
        """Simplify mathematical notation for beginners."""

        # Note: In production, use ML or templating to intelligently simplify
        # This is a placeholder showing the concept

        replacements = {
            "∇ (gradient)": "change in value",
            "∂/∂x (partial derivative)": "rate of change",
            "Σ (summation)": "sum of all",
            "∫ (integral)": "area under curve",
        }

        simplified = content
        for complex_term, simple_term in replacements.items():
            simplified = simplified.replace(complex_term, f"{simple_term} ({complex_term})")

        return simplified

    def _replace_isaac_sim_with_gazebo(self, content: str) -> str:
        """Replace Isaac Sim references with Gazebo alternatives."""

        replacements = {
            "NVIDIA Isaac Sim": "Gazebo (lightweight open-source alternative)",
            "Isaac Sim's high-fidelity physics": "Gazebo's physics simulation",
            "GPU-accelerated Isaac": "Gazebo (CPU-friendly)",
        }

        replaced = content
        for isaac_ref, gazebo_ref in replacements.items():
            replaced = replaced.replace(isaac_ref, gazebo_ref)

        return replaced

    def _amplify_examples(self, content: str) -> str:
        """Add more examples for beginners."""

        additional_examples = """

### Additional Beginner Examples

**Example 1: Step-by-step walkthrough**
[Detailed explanation with visual aids]

**Example 2: Common mistakes**
[What NOT to do and why]

**Example 3: Real-world analogy**
[How this applies to everyday objects]
"""

        return content + additional_examples

    def _add_cloud_simulation_path(self) -> str:
        """Add information about cloud simulation for GPU-less users."""

        return """

### Cloud Simulation Path for Your Setup

Since you don't have a local GPU, consider these options:

1. **Google Colab**: Free GPU access (limited hours)
   - Run Gazebo simulations in cloud
   - Store results in Google Drive

2. **AWS/Azure/GCP**: Pay-per-use GPU instances
   - Full Isaac Sim access
   - Cost: $0.30-1.00/hour

3. **Local Gazebo**: Free, runs on CPU
   - Lower fidelity but fully functional
   - Start here to learn concepts

**Recommendation**: Start with local Gazebo for learning, upgrade to cloud for advanced labs.
"""

    # =====================================================================
    # Prerequisite Sections
    # =====================================================================

    def _generate_prerequisite_sections(
        self,
        context: Dict[str, Any]
    ) -> List[ContentSection]:
        """Generate prerequisite sections for users missing foundational knowledge."""

        prerequisites = []

        # 1. ROS 2 Primer for robotics beginners
        if context["robotics_experience"] == "none":
            prerequisites.append(
                ContentSection(
                    id="ros2-primer",
                    title="ROS 2 Primer: Getting Started",
                    content=self._generate_ros2_primer(),
                    difficulty=ContentDifficulty.BEGINNER,
                    section_type="theory",
                    estimated_read_time_minutes=15
                )
            )

        # 2. Python Refresher for programming beginners
        if context["programming_level"] == "beginner":
            prerequisites.append(
                ContentSection(
                    id="python-refresher",
                    title="Python Refresher: Essential Concepts",
                    content=self._generate_python_refresher(),
                    difficulty=ContentDifficulty.BEGINNER,
                    section_type="theory",
                    estimated_read_time_minutes=20
                )
            )

        # 3. Simulation Fundamentals for simulation learners
        if context["learning_goal"] == "simulations":
            prerequisites.append(
                ContentSection(
                    id="sim-fundamentals",
                    title="Simulation Fundamentals",
                    content=self._generate_simulation_fundamentals(),
                    difficulty=ContentDifficulty.BEGINNER,
                    section_type="theory",
                    estimated_read_time_minutes=10
                )
            )

        return prerequisites

    def _generate_ros2_primer(self) -> str:
        """Generate ROS 2 primer for robotics beginners."""

        return """
## ROS 2 Primer: What You Need to Know

**ROS 2 (Robot Operating System 2)** is software that helps robots:
1. Sense (read sensors)
2. Think (process data)
3. Act (move motors)

### Key Concepts

- **Node**: A process that does one job (e.g., read camera, move robot)
- **Topic**: A communication channel (e.g., /camera/image, /motor/cmd)
- **Publisher**: Sends data to a topic
- **Subscriber**: Receives data from a topic

### Simple Example

```
Publisher: Camera sends images to /camera/image
Subscriber: Brain reads images from /camera/image
```

This separation lets you change how the camera works without changing the brain!
"""

    def _generate_python_refresher(self) -> str:
        """Generate Python refresher for programming beginners."""

        return """
## Python Refresher: Essential Concepts

Before diving into robotics code, let's review Python basics.

### Variables & Types

```python
# Store information
name = "Robot"           # Text
speed = 5.5            # Number
is_moving = True       # Boolean
```

### Functions

```python
def move_forward(speed):
    print(f"Moving at {speed} m/s")

move_forward(2.0)  # Call the function
```

### Control Flow

```python
if speed > 5:
    print("Going fast!")
else:
    print("Going slow")
```

### Lists

```python
positions = [0.0, 1.5, 3.2]  # Store multiple values
for position in positions:
    print(position)
```
"""

    def _generate_simulation_fundamentals(self) -> str:
        """Generate simulation fundamentals for simulation learners."""

        return """
## Simulation Fundamentals

**Why Simulate?**
- Safe (no expensive hardware damage)
- Fast (run 100x simulations overnight)
- Reproducible (exact same conditions every time)

**Physics Simulation**
- Gravity, friction, collisions
- Motors, sensors
- Realism vs speed tradeoff

**Sim-to-Real Gap**
- Simulation ≠ Reality
- Use domain randomization to bridge the gap
- Start in simulation, test on real robot

**This Chapter's Simulators**
- Gazebo: Lightweight, fast, free
- Isaac Sim: High-fidelity, GPU-accelerated, requires NVIDIA
"""

    # =====================================================================
    # Warnings & Recommendations
    # =====================================================================

    def _generate_warnings(self, user: UserProfile) -> List[str]:
        """Generate contextual warnings based on user profile."""

        warnings = []

        # Hardware limitations
        if not user.has_rtx:
            warnings.append(
                "⚠️ No GPU detected: This chapter uses Gazebo (lighter) instead of "
                "Isaac Sim (high-fidelity). Both teach the same concepts!"
            )

        if user.has_rtx and user.get_rtx_vram_gb() < 24:
            warnings.append(
                f"⚠️ Your GPU has {user.get_rtx_vram_gb()}GB VRAM. "
                "Isaac Sim runs best with 24GB+. Consider cloud GPU for advanced labs."
            )

        if not user.has_jetson:
            warnings.append(
                "📌 No Jetson hardware: Edge deployment sections are simulation-only. "
                "Concepts still apply to real Jetson devices!"
            )

        # Experience level
        if user.robotics_experience == "none":
            warnings.append(
                "📚 ROS 2 Primer included: Start here if you're new to robotics!"
            )

        if user.ai_experience == "none":
            warnings.append(
                "📚 AI Fundamentals included: This chapter builds AI concepts from scratch."
            )

        return warnings


# =====================================================================
# Personalization Service (high-level)
# =====================================================================

class PersonalizationService:
    """High-level personalization service."""

    def __init__(self):
        self.engine = PersonalizationEngine()

    def personalize_chapter_for_user(
        self,
        chapter_data: Dict[str, Any],
        user: UserProfile
    ) -> PersonalizedChapter:
        """
        Personalize a chapter for a user.

        Args:
            chapter_data: Raw chapter with sections
            user: User profile

        Returns:
            PersonalizedChapter: Customized chapter
        """

        # Convert raw sections to ContentSection objects
        sections = [
            ContentSection(
                id=s["id"],
                title=s["title"],
                content=s["content"],
                difficulty=ContentDifficulty(s.get("difficulty", "intermediate")),
                section_type=s.get("type", "theory"),
                prerequisites=s.get("prerequisites", []),
                estimated_read_time_minutes=s.get("read_time_minutes", 5)
            )
            for s in chapter_data.get("sections", [])
        ]

        return self.engine.personalize_chapter(
            chapter=chapter_data,
            user=user,
            raw_sections=sections
        )

    def get_personalization_stats(self, personalized_chapter: PersonalizedChapter) -> Dict[str, Any]:
        """Get statistics about personalization."""

        return {
            "total_sections": len(personalized_chapter.sections),
            "total_read_time_minutes": personalized_chapter.total_estimated_read_time,
            "transformations_applied": len(personalized_chapter.transformations),
            "warnings_count": len(personalized_chapter.warnings),
            "user_difficulty_level": personalized_chapter.user_difficulty_level,
            "recommended_simulator": personalized_chapter.recommended_simulator,
            "recommended_labs": personalized_chapter.recommended_labs
        }
