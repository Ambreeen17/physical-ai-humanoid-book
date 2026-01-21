"""
Database Seeding Script for Chapter 1 Assessments
Populates Assessment table with quiz questions, lab exercises, and challenges.
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from sqlalchemy.orm import Session
from src.db.session import SessionLocal
from src.models.chapter import Chapter
from src.models.assessment import Assessment
from src.logging.logger import get_logger

logger = get_logger(__name__)


def seed_chapter_1_assessments(db: Session):
    """
    Seed Chapter 1: Introduction to Physical AI assessments.

    Includes:
    - 3 Multiple Choice Questions (Foundational)
    - 1 Short Answer Question (Intermediate)
    - 1 Lab Exercise (Advanced)
    - 1 Integration Challenge (Bonus)
    """

    # Verify Chapter 1 exists or create it
    chapter_1 = db.query(Chapter).filter(Chapter.number == 1).first()
    if not chapter_1:
        chapter_1 = Chapter(
            number=1,
            title="Introduction to Physical AI",
            description="Understanding embodied intelligence, sim-to-real gap, humanoid platforms, and ROS 2 fundamentals",
            status="PUBLISHED"
        )
        db.add(chapter_1)
        db.commit()
        db.refresh(chapter_1)
        logger.info(f"Created Chapter 1: {chapter_1.title}")

    # Clear existing assessments for Chapter 1 (idempotent seeding)
    db.query(Assessment).filter(Assessment.chapter_id == chapter_1.id).delete()
    db.commit()
    logger.info("Cleared existing Chapter 1 assessments")

    # Assessment 1.1: Multiple Choice - Embodied vs Disembodied AI
    assessment_1_1 = Assessment(
        chapter_id=chapter_1.id,
        title="Q1.1: Embodied Intelligence vs LLMs",
        description="What is the primary difference between an LLM (like ChatGPT) and a Vision-Language-Action (VLA) model?",
        assessment_type="QUIZ",
        difficulty="BEGINNER",
        max_score=2,
        content={
            "question_id": "1.1",
            "question_text": "What is the primary difference between an LLM (like ChatGPT) and a Vision-Language-Action (VLA) model?",
            "options": {
                "A": "LLMs process text tokens; VLAs process sensory input and output motor commands",
                "B": "LLMs are larger; VLAs are smaller models",
                "C": "LLMs run on CPUs; VLAs require GPUs",
                "D": "LLMs are proprietary; VLAs are open-source"
            },
            "correct_answer": "A",
            "explanation": "VLAs integrate perception (vision/sensors), language understanding, and action (motor commands) in a closed sensorimotor loop, unlike LLMs which operate purely on text."
        }
    )

    # Assessment 1.2: Multiple Choice - Sim-to-Real Gap
    assessment_1_2 = Assessment(
        chapter_id=chapter_1.id,
        title="Q1.2: Sim-to-Real Gap Strategies",
        description="Which of the following is NOT a strategy for bridging the Sim-to-Real gap?",
        assessment_type="QUIZ",
        difficulty="BEGINNER",
        max_score=2,
        content={
            "question_id": "1.2",
            "question_text": "Which of the following is NOT a strategy for bridging the Sim-to-Real gap?",
            "options": {
                "A": "Domain Randomization (randomizing physics parameters)",
                "B": "System Identification (measuring real robot parameters)",
                "C": "Increasing simulation graphics resolution to 8K",
                "D": "Residual Physics (learning correction networks)"
            },
            "correct_answer": "C",
            "explanation": "Graphics resolution does not address physics modeling errors, sensor noise, or control latency—the true causes of the sim-to-real gap. Domain randomization, SysID, and residual learning are proven strategies."
        }
    )

    # Assessment 1.3: Multiple Choice - ROS 2 Topics
    assessment_1_3 = Assessment(
        chapter_id=chapter_1.id,
        title="Q1.3: ROS 2 Topics",
        description="In ROS 2, what is a 'Topic'?",
        assessment_type="QUIZ",
        difficulty="BEGINNER",
        max_score=2,
        content={
            "question_id": "1.3",
            "question_text": "In ROS 2, what is a 'Topic'?",
            "options": {
                "A": "A named communication channel for publishing and subscribing to messages",
                "B": "A configuration file for robot parameters",
                "C": "A hardware driver for sensors",
                "D": "A logging mechanism for debugging"
            },
            "correct_answer": "A",
            "explanation": "Topics are named channels (e.g., /turtle1/cmd_vel) where nodes publish and subscribe to messages anonymously. This decouples producers from consumers."
        }
    )

    # Assessment 1.4: Short Answer - Sensorimotor Loop
    assessment_1_4 = Assessment(
        chapter_id=chapter_1.id,
        title="Q1.4: Sensorimotor Loop Explanation",
        description="Explain the sensorimotor loop and why it is fundamental to embodied intelligence.",
        assessment_type="QUIZ",
        difficulty="INTERMEDIATE",
        max_score=3,
        content={
            "question_id": "1.4",
            "question_text": "Explain the sensorimotor loop and why it is fundamental to embodied intelligence. (3-5 sentences)",
            "question_type": "short_answer",
            "rubric": {
                "3_points": "Clear explanation of Perception → Decision → Action → Feedback cycle; explains why physical grounding requires closed-loop interaction; mentions real-time constraints (latency sensitivity).",
                "2_points": "Describes the cycle but misses either physical grounding importance or real-time constraints.",
                "1_point": "Minimal description of perception and action but no mention of feedback or closed-loop nature.",
                "0_points": "Incorrect or missing response."
            },
            "sample_answer": "The sensorimotor loop is the cycle of Perception → Decision → Action → Feedback that runs continuously in embodied systems. Unlike text-based AI, robots must respond to sensor input in real-time (10-1000 Hz), execute motor commands, and immediately perceive the consequences of their actions. This closed loop grounds intelligence in physical interaction, making delay-sensitive and context-dependent—a 100ms delay can cause a robot to drop an object."
        }
    )

    # Assessment 1.5: Lab Exercise - Extend Hello Physical AI Lab
    assessment_1_5 = Assessment(
        chapter_id=chapter_1.id,
        title="Lab 1.5: Rotation Detection Extension",
        description="Modify hello_physical_ai_node.py to detect rotation changes and log warnings.",
        assessment_type="LAB_EXERCISE",
        difficulty="INTERMEDIATE",
        max_score=10,
        content={
            "task_id": "1.5",
            "task_title": "Extend the Hello Physical AI Lab",
            "problem_statement": "Modify the hello_physical_ai_node.py to implement real-time safety warnings based on robot rotation.",
            "specifications": "Add logic that detects when theta (rotation) changes by more than 0.5 radians and logs a warning message: 'WARNING: Significant rotation detected: {theta}'",
            "time_estimate": "30 minutes",
            "expected_patterns": [
                r"WARNING:\s*Significant rotation detected",
                r"theta.*=.*[\d\.\-]+",
                r"Perception.*Decision.*Action"
            ],
            "rubric": {
                "10_points": "Code runs without errors, rotation detection logic works correctly, logs show sensorimotor loop in action with clear WARNING messages.",
                "7-9_points": "Code runs with minor issues (e.g., threshold not exactly 0.5), logs show rotation detection but formatting inconsistent.",
                "4-6_points": "Code runs but logic partially incorrect (e.g., detects rotation but doesn't log properly), or thresholds not implemented.",
                "1-3_points": "Code has errors or rotation detection not implemented, but shows some understanding of ROS 2 structure.",
                "0_points": "No submission or code does not run."
            },
            "starter_code_path": "specs/1-book-curriculum/chapters/chapter-1/lab/src/hello_physical_ai_node.py",
            "expected_output_path": "specs/1-book-curriculum/chapters/chapter-1/lab/expected_output.txt"
        }
    )

    # Assessment 1.6: Integration Challenge - Safety Watchdog
    assessment_1_6 = Assessment(
        chapter_id=chapter_1.id,
        title="Challenge 1.6: Safety Watchdog Node",
        description="Design a separate safety_watchdog_node.py that prevents out-of-bounds movement.",
        assessment_type="CAPSTONE_CHALLENGE",
        difficulty="ADVANCED",
        max_score=5,
        content={
            "challenge_id": "1.6",
            "challenge_title": "The Safety Watchdog",
            "scenario": "A warehouse robot requires a secondary safety layer to prevent collisions or out-of-bounds movement.",
            "task": "Design a safety_watchdog_node.py that monitors /turtle1/pose and publishes a zero-velocity command to /turtle1/cmd_vel if boundaries (X > 3.0 or Y > 2.0) are exceeded.",
            "decision_points": [
                "Sampling rate of the pose topic",
                "Priority of the stop command over other control nodes",
                "Logging verbosity for debugging"
            ],
            "success_metrics": [
                "Robot stops before hitting virtual walls",
                "Watchdog reacts in < 100ms",
                "No false positives (stops only when necessary)"
            ],
            "rubric": {
                "5_points": "Separate node correctly subscribes to /turtle1/pose, publishes zero-velocity on boundary breach, reacts within 100ms, handles edge cases (corner boundaries).",
                "3-4_points": "Node works but has minor issues (e.g., reaction time > 100ms, or doesn't handle Y boundary correctly).",
                "1-2_points": "Node structure is correct but safety logic incomplete or buggy.",
                "0_points": "No submission or node does not subscribe/publish correctly."
            }
        }
    )

    # Add all assessments to database
    assessments = [
        assessment_1_1,
        assessment_1_2,
        assessment_1_3,
        assessment_1_4,
        assessment_1_5,
        assessment_1_6
    ]

    for assessment in assessments:
        db.add(assessment)

    db.commit()

    logger.info(f"Seeded {len(assessments)} assessments for Chapter 1")
    return assessments


def main():
    """Main seeding function."""
    db = SessionLocal()
    try:
        logger.info("Starting assessment seeding...")
        assessments = seed_chapter_1_assessments(db)
        logger.info("✅ Assessment seeding complete")

        # Print summary
        print("\n" + "="*60)
        print("Chapter 1 Assessments Seeded Successfully")
        print("="*60)
        for i, assessment in enumerate(assessments, 1):
            print(f"{i}. {assessment.title} ({assessment.assessment_type}, {assessment.difficulty}, {assessment.max_score} pts)")
        print("="*60)
        print(f"\nTotal: {len(assessments)} assessments, {sum(a.max_score for a in assessments)} points")
        print("\nNext steps:")
        print("1. Start backend: cd backend && uvicorn src.main:app --reload")
        print("2. View assessments: GET /api/assessments/chapter/1")
        print("3. Submit quiz: POST /api/assessments/quiz/submit")

    except Exception as e:
        logger.error(f"Seeding failed: {e}")
        db.rollback()
        raise
    finally:
        db.close()


if __name__ == "__main__":
    main()
