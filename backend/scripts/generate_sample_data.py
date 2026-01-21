"""
Generate sample data for testing and development.
Creates learner profiles, assessment submissions, and chat history.
"""
import sys
import os
import random
from datetime import datetime, timedelta
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from sqlalchemy.orm import Session
from src.db.session import SessionLocal
from src.models.learner_profile import LearnerProfile
from src.models.assessment_result import AssessmentResult
from src.models.chapter import Chapter
from src.models.assessment import Assessment
from src.logging.logger import get_logger

logger = get_logger(__name__)


# Sample learner personas
LEARNERS = [
    {
        "learner_id": "alice_undergrad_cs",
        "python_score": 8,
        "ml_score": 6,
        "robotics_score": 3,
        "ros_score": 2
    },
    {
        "learner_id": "bob_engineer_ml",
        "python_score": 9,
        "ml_score": 8,
        "robotics_score": 5,
        "ros_score": 4
    },
    {
        "learner_id": "carol_phd_robotics",
        "python_score": 9,
        "ml_score": 9,
        "robotics_score": 9,
        "ros_score": 8
    },
    {
        "learner_id": "dave_bootcamp_grad",
        "python_score": 6,
        "ml_score": 4,
        "robotics_score": 2,
        "ros_score": 1
    },
    {
        "learner_id": "eve_mecheng_student",
        "python_score": 5,
        "ml_score": 3,
        "robotics_score": 7,
        "ros_score": 3
    },
    {
        "learner_id": "frank_career_changer",
        "python_score": 7,
        "ml_score": 5,
        "robotics_score": 1,
        "ros_score": 0
    },
    {
        "learner_id": "grace_researcher",
        "python_score": 10,
        "ml_score": 9,
        "robotics_score": 8,
        "ros_score": 9
    },
    {
        "learner_id": "henry_hobbyist",
        "python_score": 4,
        "ml_score": 2,
        "robotics_score": 3,
        "ros_score": 1
    }
]


def create_learner_profiles(db: Session):
    """Create sample learner profiles."""
    logger.info("Creating learner profiles...")

    for learner_data in LEARNERS:
        # Check if learner already exists
        existing = db.query(LearnerProfile).filter(
            LearnerProfile.learner_id == learner_data["learner_id"]
        ).first()

        if existing:
            logger.info(f"Learner {learner_data['learner_id']} already exists, skipping")
            continue

        profile = LearnerProfile(**learner_data)
        db.add(profile)
        logger.info(f"Created learner: {learner_data['learner_id']} (difficulty: {profile.difficulty_level})")

    db.commit()
    logger.info(f"✅ Created {len(LEARNERS)} learner profiles")


def create_assessment_submissions(db: Session):
    """Create sample assessment submissions for testing."""
    logger.info("Creating assessment submissions...")

    # Get all learners and Chapter 1 assessments
    learners = db.query(LearnerProfile).all()
    chapter_1 = db.query(Chapter).filter(Chapter.number == 1).first()

    if not chapter_1:
        logger.warning("Chapter 1 not found, skipping assessment submissions")
        return

    assessments = db.query(Assessment).filter(Assessment.chapter_id == chapter_1.id).all()

    if not assessments:
        logger.warning("No assessments found for Chapter 1, skipping submissions")
        return

    submissions_created = 0

    for learner in learners:
        # Simulate learner completing some assessments based on their skill level
        # Higher skill = more assessments completed
        avg_score = (learner.python_score + learner.ml_score + learner.robotics_score + learner.ros_score) / 4
        completion_rate = min(0.9, avg_score / 10)  # 0-90% completion based on skill

        num_to_complete = int(len(assessments) * completion_rate)
        assessments_to_complete = random.sample(assessments, num_to_complete) if num_to_complete > 0 else []

        for assessment in assessments_to_complete:
            # Check if submission already exists
            existing = db.query(AssessmentResult).filter(
                AssessmentResult.learner_profile_id == learner.id,
                AssessmentResult.assessment_id == assessment.id
            ).first()

            if existing:
                continue

            # Simulate score based on learner skill and assessment difficulty
            if assessment.difficulty == "BEGINNER":
                base_score = avg_score * 0.9
            elif assessment.difficulty == "INTERMEDIATE":
                base_score = avg_score * 0.75
            else:  # ADVANCED
                base_score = avg_score * 0.6

            # Add randomness
            score_factor = random.uniform(0.8, 1.2)
            score = min(assessment.max_score, assessment.max_score * (base_score / 10) * score_factor)
            passed = score >= (assessment.max_score * 0.7)  # 70% threshold

            # Create submission
            result = AssessmentResult(
                learner_profile_id=learner.id,
                assessment_id=assessment.id,
                score=round(score, 2),
                max_score=assessment.max_score,
                passed=passed,
                feedback=f"Good work! You scored {round(score, 2)}/{assessment.max_score}." if passed else "Keep practicing! Review the material and try again.",
                raw_submission={"simulated": True, "timestamp": datetime.now().isoformat()}
            )

            # Set created_at to simulate activity over time (last 7 days)
            days_ago = random.randint(0, 7)
            result.created_at = datetime.now() - timedelta(days=days_ago)

            db.add(result)
            submissions_created += 1

    db.commit()
    logger.info(f"✅ Created {submissions_created} assessment submissions")


def generate_all_sample_data(db: Session):
    """Generate all sample data."""
    logger.info("="*60)
    logger.info("Generating Sample Data")
    logger.info("="*60)

    create_learner_profiles(db)
    create_assessment_submissions(db)

    logger.info("="*60)
    logger.info("✅ Sample data generation complete")
    logger.info("="*60)


def main():
    """Main execution function."""
    db = SessionLocal()
    try:
        generate_all_sample_data(db)

        # Print summary
        print("\n" + "="*60)
        print("Sample Data Summary")
        print("="*60)

        learners = db.query(LearnerProfile).all()
        results = db.query(AssessmentResult).all()

        print(f"Learner Profiles: {len(learners)}")
        for learner in learners:
            print(f"  - {learner.learner_id} (difficulty: {learner.difficulty_level})")

        print(f"\nAssessment Submissions: {len(results)}")
        passed_count = sum(1 for r in results if r.passed)
        print(f"  - Passed: {passed_count}")
        print(f"  - Failed: {len(results) - passed_count}")

        print("\nNext steps:")
        print("1. Access backend: http://localhost:8000/docs")
        print("2. Access frontend: http://localhost:3000")
        print("3. Test learner profiles: GET /api/learner-profile/{learner_id}")
        print("4. Test assessments: GET /api/assessments/results/{learner_id}")

    except Exception as e:
        logger.error(f"Sample data generation failed: {e}")
        db.rollback()
        raise
    finally:
        db.close()


if __name__ == "__main__":
    main()
