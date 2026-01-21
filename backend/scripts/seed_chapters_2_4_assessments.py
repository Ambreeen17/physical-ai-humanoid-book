"""
Database Seeding Script for Chapters 2-4 Assessments
Populates Assessment table with quiz questions, lab exercises, and challenges for multiple chapters.
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


def seed_chapter_2_assessments(db: Session):
    """
    Seed Chapter 2: Kinematics & Dynamics assessments.
    """
    # Verify Chapter 2 exists or create it
    chapter_2 = db.query(Chapter).filter(Chapter.number == 2).first()
    if not chapter_2:
        chapter_2 = Chapter(
            number=2,
            title="Kinematics & Dynamics",
            description="Joint spaces, forward/inverse kinematics, equations of motion",
            status="PUBLISHED"
        )
        db.add(chapter_2)
        db.commit()
        db.refresh(chapter_2)
        logger.info(f"Created Chapter 2: {chapter_2.title}")

    # Clear existing assessments for Chapter 2 (idempotent seeding)
    db.query(Assessment).filter(Assessment.chapter_id == chapter_2.id).delete()
    db.commit()
    logger.info("Cleared existing Chapter 2 assessments")

    # Assessment 2.1: Multiple Choice - Forward Kinematics
    assessment_2_1 = Assessment(
        chapter_id=chapter_2.id,
        title="Q2.1: Forward Kinematics",
        description="What does forward kinematics compute?",
        assessment_type="QUIZ",
        difficulty="BEGINNER",
        max_score=2,
        content={
            "question_id": "2.1",
            "question_text": "What does forward kinematics compute?",
            "options": {
                "A": "Joint angles from end-effector position",
                "B": "End-effector position from joint angles",
                "C": "Torques from forces",
                "D": "Velocities from positions"
            },
            "correct_answer": "B",
            "explanation": "Forward kinematics computes the end-effector position and orientation from known joint angles."
        }
    )

    # Assessment 2.2: Multiple Choice - Inverse Kinematics
    assessment_2_2 = Assessment(
        chapter_id=chapter_2.id,
        title="Q2.2: Inverse Kinematics",
        description="Which statement about inverse kinematics is true?",
        assessment_type="QUIZ",
        difficulty="BEGINNER",
        max_score=2,
        content={
            "question_id": "2.2",
            "question_text": "Which statement about inverse kinematics is true?",
            "options": {
                "A": "It always has a unique solution",
                "B": "It computes joint angles from desired end-effector position",
                "C": "It is easier than forward kinematics",
                "D": "It only applies to 2-DOF arms"
            },
            "correct_answer": "B",
            "explanation": "Inverse kinematics computes the joint angles required to achieve a desired end-effector position and orientation."
        }
    )

    # Assessment 2.3: Multiple Choice - DH Parameters
    assessment_2_3 = Assessment(
        chapter_id=chapter_2.id,
        title="Q2.3: DH Parameters",
        description="What do Denavit-Hartenberg (DH) parameters describe?",
        assessment_type="QUIZ",
        difficulty="INTERMEDIATE",
        max_score=2,
        content={
            "question_id": "2.3",
            "question_text": "What do Denavit-Hartenberg (DH) parameters describe?",
            "options": {
                "A": "Joint torques and forces",
                "B": "The relative position and orientation of consecutive joint frames",
                "C": "Motor control parameters",
                "D": "Sensor calibration values"
            },
            "correct_answer": "B",
            "explanation": "DH parameters provide a systematic way to define the relative position and orientation of consecutive joint coordinate frames."
        }
    )

    # Assessment 2.4: Short Answer - Lagrangian Dynamics
    assessment_2_4 = Assessment(
        chapter_id=chapter_2.id,
        title="Q2.4: Lagrangian Dynamics Explanation",
        description="Explain the Lagrangian approach to deriving equations of motion.",
        assessment_type="QUIZ",
        difficulty="INTERMEDIATE",
        max_score=3,
        content={
            "question_id": "2.4",
            "question_text": "Explain the Lagrangian approach to deriving equations of motion. What are the key quantities involved? (3-5 sentences)",
            "question_type": "short_answer",
            "rubric": {
                "3_points": "Correctly identifies kinetic energy (T), potential energy (V), and Lagrangian (L = T - V); explains how Euler-Lagrange equations yield equations of motion; mentions advantages over Newtonian approach.",
                "2_points": "Describes kinetic and potential energy but misses Lagrangian definition or Euler-Lagrange equations.",
                "1_point": "Mentions energy concepts but doesn't explain the approach clearly.",
                "0_points": "Incorrect or missing response."
            },
            "sample_answer": "The Lagrangian approach uses the difference between kinetic energy (T) and potential energy (V) to define the Lagrangian L = T - V. The Euler-Lagrange equations d/dt(∂L/∂q̇) - ∂L/∂q = Q are then applied to derive the equations of motion. This approach is advantageous for complex systems as it naturally handles constraints and generalized coordinates."
        }
    )

    # Assessment 2.5: Lab Exercise - Forward Kinematics Implementation
    assessment_2_5 = Assessment(
        chapter_id=chapter_2.id,
        title="Lab 2.5: Forward Kinematics Implementation",
        description="Implement forward kinematics for a 2-DOF planar arm.",
        assessment_type="LAB_EXERCISE",
        difficulty="INTERMEDIATE",
        max_score=10,
        content={
            "task_id": "2.5",
            "task_title": "Forward Kinematics for 2-DOF Arm",
            "problem_statement": "Implement forward kinematics for a 2-DOF planar arm with link lengths l1=1.0 and l2=0.8.",
            "specifications": "Create a function that takes joint angles [q1, q2] and returns end-effector position [x, y]. Use trigonometric relationships to compute the position.",
            "time_estimate": "45 minutes",
            "expected_patterns": [
                r"def forward_kinematics",
                r"numpy|math",
                r"x =.*l1.*cos",
                r"y =.*l1.*sin"
            ],
            "rubric": {
                "10_points": "Function correctly computes end-effector position for any joint angles, handles angle wrapping, includes proper documentation and error checking.",
                "7-9_points": "Function works correctly but has minor issues (e.g., missing documentation or edge case handling).",
                "4-6_points": "Function has correct general approach but mathematical errors in computation.",
                "1-3_points": "Function structure is present but implementation is incorrect.",
                "0_points": "No submission or code does not run."
            },
            "starter_code_path": "specs/1-book-curriculum/chapters/chapter-2/lab/lab_2_1/src/lab_2_1_node.py",
            "expected_output_path": "specs/1-book-curriculum/chapters/chapter-2/lab/lab_2_1/expected_output.txt"
        }
    )

    # Assessment 2.6: Integration Challenge - Inverse Kinematics Solver
    assessment_2_6 = Assessment(
        chapter_id=chapter_2.id,
        title="Challenge 2.6: Inverse Kinematics Solver",
        description="Implement an inverse kinematics solver for a 3-DOF arm.",
        assessment_type="CAPSTONE_CHALLENGE",
        difficulty="ADVANCED",
        max_score=5,
        content={
            "challenge_id": "2.6",
            "challenge_title": "Analytical Inverse Kinematics",
            "scenario": "A 3-DOF arm needs to reach a specific position in 3D space.",
            "task": "Implement an analytical inverse kinematics solver for a 3-DOF arm that computes joint angles from desired end-effector position.",
            "decision_points": [
                "Choice of arm configuration (e.g., spherical wrist)",
                "Handling of multiple solutions",
                "Singularity detection and handling"
            ],
            "success_metrics": [
                "Solver computes correct joint angles for given position",
                "Handles multiple valid solutions",
                "Detects and reports singularities"
            ],
            "rubric": {
                "5_points": "Complete analytical solution with singularity detection, handles multiple solutions, accurate computation for various positions.",
                "3-4_points": "Solution works for most positions but has issues with singularities or multiple solutions.",
                "1-2_points": "Basic approach implemented but with significant errors in computation.",
                "0_points": "No submission or solution does not compute joint angles correctly."
            }
        }
    )

    # Add all assessments to database
    assessments = [
        assessment_2_1,
        assessment_2_2,
        assessment_2_3,
        assessment_2_4,
        assessment_2_5,
        assessment_2_6
    ]

    for assessment in assessments:
        db.add(assessment)

    db.commit()

    logger.info(f"Seeded {len(assessments)} assessments for Chapter 2")
    return assessments


def seed_chapter_3_assessments(db: Session):
    """
    Seed Chapter 3: Sensors & Actuators assessments.
    """
    # Verify Chapter 3 exists or create it
    chapter_3 = db.query(Chapter).filter(Chapter.number == 3).first()
    if not chapter_3:
        chapter_3 = Chapter(
            number=3,
            title="Sensors & Actuators",
            description="LiDAR, cameras, servo motors, force-torque sensors",
            status="PUBLISHED"
        )
        db.add(chapter_3)
        db.commit()
        db.refresh(chapter_3)
        logger.info(f"Created Chapter 3: {chapter_3.title}")

    # Clear existing assessments for Chapter 3 (idempotent seeding)
    db.query(Assessment).filter(Assessment.chapter_id == chapter_3.id).delete()
    db.commit()
    logger.info("Cleared existing Chapter 3 assessments")

    # Assessment 3.1: Multiple Choice - LiDAR Technology
    assessment_3_1 = Assessment(
        chapter_id=chapter_3.id,
        title="Q3.1: LiDAR Technology",
        description="What is the primary principle behind Time-of-Flight (ToF) LiDAR?",
        assessment_type="QUIZ",
        difficulty="BEGINNER",
        max_score=2,
        content={
            "question_id": "3.1",
            "question_text": "What is the primary principle behind Time-of-Flight (ToF) LiDAR?",
            "options": {
                "A": "Measuring phase shift of reflected light",
                "B": "Measuring time for light pulse to return",
                "C": "Measuring frequency shift of reflected light",
                "D": "Measuring intensity of reflected light"
            },
            "correct_answer": "B",
            "explanation": "ToF LiDAR measures the time it takes for a laser pulse to travel to an object and back to calculate distance."
        }
    )

    # Assessment 3.2: Multiple Choice - Camera Types
    assessment_3_2 = Assessment(
        chapter_id=chapter_3.id,
        title="Q3.2: Camera Types",
        description="Which camera type is best for depth estimation?",
        assessment_type="QUIZ",
        difficulty="BEGINNER",
        max_score=2,
        content={
            "question_id": "3.2",
            "question_text": "Which camera type is best for direct depth estimation?",
            "options": {
                "A": "Monocular camera",
                "B": "Stereo camera",
                "C": "RGB camera",
                "D": "Thermal camera"
            },
            "correct_answer": "B",
            "explanation": "Stereo cameras use two lenses to capture images from slightly different positions, allowing triangulation to estimate depth."
        }
    )

    # Assessment 3.3: Multiple Choice - IMU Sensors
    assessment_3_3 = Assessment(
        chapter_id=chapter_3.id,
        title="Q3.3: IMU Sensors",
        description="What does an IMU typically measure?",
        assessment_type="QUIZ",
        difficulty="INTERMEDIATE",
        max_score=2,
        content={
            "question_id": "3.3",
            "question_text": "What does an IMU (Inertial Measurement Unit) typically measure?",
            "options": {
                "A": "Position and velocity only",
                "B": "Acceleration and angular velocity",
                "C": "Magnetic field only",
                "D": "Temperature and pressure"
            },
            "correct_answer": "B",
            "explanation": "An IMU typically combines accelerometers (measuring linear acceleration) and gyroscopes (measuring angular velocity)."
        }
    )

    # Assessment 3.4: Short Answer - Sensor Fusion
    assessment_3_4 = Assessment(
        chapter_id=chapter_3.id,
        title="Q3.4: Sensor Fusion Explanation",
        description="Explain why sensor fusion is important in robotics.",
        assessment_type="QUIZ",
        difficulty="INTERMEDIATE",
        max_score=3,
        content={
            "question_id": "3.4",
            "question_text": "Explain why sensor fusion is important in robotics. What are the main benefits? (3-5 sentences)",
            "question_type": "short_answer",
            "rubric": {
                "3_points": "Correctly identifies redundancy, accuracy improvement, and robustness as key benefits; explains how different sensors complement each other; mentions Kalman filters or similar fusion techniques.",
                "2_points": "Describes some benefits but misses one or more key points.",
                "1_point": "Mentions one benefit but explanation is limited.",
                "0_points": "Incorrect or missing response."
            },
            "sample_answer": "Sensor fusion combines data from multiple sensors to improve accuracy, reliability, and robustness. Different sensors have complementary strengths and weaknesses - for example, IMUs provide high-frequency data but drift over time, while cameras provide absolute position references but at lower frequency. By combining these sensors, robots can achieve more accurate and reliable state estimation than with any single sensor."
        }
    )

    # Assessment 3.5: Lab Exercise - LiDAR Point Cloud Processing
    assessment_3_5 = Assessment(
        chapter_id=chapter_3.id,
        title="Lab 3.5: LiDAR Point Cloud Processing",
        description="Process LiDAR point cloud data to detect obstacles.",
        assessment_type="LAB_EXERCISE",
        difficulty="INTERMEDIATE",
        max_score=10,
        content={
            "task_id": "3.5",
            "task_title": "LiDAR Obstacle Detection",
            "problem_statement": "Process LiDAR scan data to detect obstacles in the robot's path.",
            "specifications": "Create a function that takes LiDAR scan data and identifies points within a certain distance threshold that represent obstacles.",
            "time_estimate": "60 minutes",
            "expected_patterns": [
                r"laser_scan_callback",
                r"np.array",
                r"distance_threshold",
                r"obstacle_detection"
            ],
            "rubric": {
                "10_points": "Function correctly processes LiDAR data, detects obstacles with appropriate thresholding, handles edge cases, includes visualization of results.",
                "7-9_points": "Function works correctly but has minor issues with threshold selection or visualization.",
                "4-6_points": "Function has correct general approach but implementation issues with distance calculations.",
                "1-3_points": "Function structure is present but implementation is incorrect.",
                "0_points": "No submission or code does not run."
            },
            "starter_code_path": "specs/1-book-curriculum/chapters/chapter-3/lab/lab_3_1/src/lab_3_1_node.py",
            "expected_output_path": "specs/1-book-curriculum/chapters/chapter-3/lab/lab_3_1/expected_output.txt"
        }
    )

    # Assessment 3.6: Integration Challenge - Multi-Sensor Integration
    assessment_3_6 = Assessment(
        chapter_id=chapter_3.id,
        title="Challenge 3.6: Multi-Sensor Integration",
        description="Integrate data from multiple sensors for state estimation.",
        assessment_type="CAPSTONE_CHALLENGE",
        difficulty="ADVANCED",
        max_score=5,
        content={
            "challenge_id": "3.6",
            "challenge_title": "Multi-Sensor State Estimation",
            "scenario": "A mobile robot needs to estimate its position using wheel encoders, IMU, and LiDAR.",
            "task": "Implement a sensor fusion algorithm that combines data from wheel encoders, IMU, and LiDAR for improved position estimation.",
            "decision_points": [
                "Choice of fusion algorithm (EKF, UKF, particle filter)",
                "Handling of different sensor update rates",
                "Management of sensor uncertainties"
            ],
            "success_metrics": [
                "Accurate position estimation compared to ground truth",
                "Robustness to sensor noise",
                "Real-time performance"
            ],
            "rubric": {
                "5_points": "Complete implementation with appropriate fusion algorithm, handles different update rates, accurate estimation with uncertainty quantification.",
                "3-4_points": "Implementation works but has issues with timing or uncertainty handling.",
                "1-2_points": "Basic approach implemented but with significant errors in fusion logic.",
                "0_points": "No submission or solution does not integrate sensors correctly."
            }
        }
    )

    # Add all assessments to database
    assessments = [
        assessment_3_1,
        assessment_3_2,
        assessment_3_3,
        assessment_3_4,
        assessment_3_5,
        assessment_3_6
    ]

    for assessment in assessments:
        db.add(assessment)

    db.commit()

    logger.info(f"Seeded {len(assessments)} assessments for Chapter 3")
    return assessments


def seed_chapter_4_assessments(db: Session):
    """
    Seed Chapter 4: State Estimation assessments.
    """
    # Verify Chapter 4 exists or create it
    chapter_4 = db.query(Chapter).filter(Chapter.number == 4).first()
    if not chapter_4:
        chapter_4 = Chapter(
            number=4,
            title="State Estimation",
            description="Kalman filters, particle filters, sensor fusion",
            status="PUBLISHED"
        )
        db.add(chapter_4)
        db.commit()
        db.refresh(chapter_4)
        logger.info(f"Created Chapter 4: {chapter_4.title}")

    # Clear existing assessments for Chapter 4 (idempotent seeding)
    db.query(Assessment).filter(Assessment.chapter_id == chapter_4.id).delete()
    db.commit()
    logger.info("Cleared existing Chapter 4 assessments")

    # Assessment 4.1: Multiple Choice - Kalman Filter
    assessment_4_1 = Assessment(
        chapter_id=chapter_4.id,
        title="Q4.1: Kalman Filter",
        description="What type of systems is the standard Kalman filter designed for?",
        assessment_type="QUIZ",
        difficulty="BEGINNER",
        max_score=2,
        content={
            "question_id": "4.1",
            "question_text": "What type of systems is the standard Kalman filter designed for?",
            "options": {
                "A": "Nonlinear systems with non-Gaussian noise",
                "B": "Linear systems with Gaussian noise",
                "C": "Nonlinear systems with Gaussian noise",
                "D": "Linear systems with non-Gaussian noise"
            },
            "correct_answer": "B",
            "explanation": "The standard Kalman filter is optimal for linear systems with Gaussian process and measurement noise."
        }
    )

    # Assessment 4.2: Multiple Choice - Particle Filter
    assessment_4_2 = Assessment(
        chapter_id=chapter_4.id,
        title="Q4.2: Particle Filter",
        description="What is a key advantage of particle filters over Kalman filters?",
        assessment_type="QUIZ",
        difficulty="BEGINNER",
        max_score=2,
        content={
            "question_id": "4.2",
            "question_text": "What is a key advantage of particle filters over Kalman filters?",
            "options": {
                "A": "Lower computational complexity",
                "B": "Ability to handle nonlinear and non-Gaussian systems",
                "C": "Guaranteed convergence",
                "D": "Exact solution for any system"
            },
            "correct_answer": "B",
            "explanation": "Particle filters can handle nonlinear systems and non-Gaussian noise distributions, unlike standard Kalman filters."
        }
    )

    # Assessment 4.3: Multiple Choice - EKF
    assessment_4_3 = Assessment(
        chapter_id=chapter_4.id,
        title="Q4.3: Extended Kalman Filter",
        description="How does the Extended Kalman Filter handle nonlinear systems?",
        assessment_type="QUIZ",
        difficulty="INTERMEDIATE",
        max_score=2,
        content={
            "question_id": "4.3",
            "question_text": "How does the Extended Kalman Filter handle nonlinear systems?",
            "options": {
                "A": "By using Monte Carlo sampling",
                "B": "By linearizing the system around the current state estimate",
                "C": "By discretizing the state space",
                "D": "By assuming the system is actually linear"
            },
            "correct_answer": "B",
            "explanation": "The EKF linearizes the nonlinear system dynamics and measurement models using Jacobians around the current state estimate."
        }
    )

    # Assessment 4.4: Short Answer - Bayes Filter
    assessment_4_4 = Assessment(
        chapter_id=chapter_4.id,
        title="Q4.4: Bayes Filter Explanation",
        description="Explain the two main steps of the Bayes filter framework.",
        assessment_type="QUIZ",
        difficulty="INTERMEDIATE",
        max_score=3,
        content={
            "question_id": "4.4",
            "question_text": "Explain the two main steps of the Bayes filter framework and their purposes. (3-5 sentences)",
            "question_type": "short_answer",
            "rubric": {
                "3_points": "Correctly identifies prediction and update steps; explains that prediction propagates state estimate forward using motion model; explains that update corrects estimate using sensor measurements.",
                "2_points": "Describes both steps but explanation of purpose is incomplete.",
                "1_point": "Mentions one step or describes steps without explaining purpose.",
                "0_points": "Incorrect or missing response."
            },
            "sample_answer": "The Bayes filter has two main steps: prediction and update. The prediction step propagates the state estimate forward in time using the motion model and accounts for motion uncertainty. The update step corrects the predicted state using sensor measurements and accounts for measurement uncertainty. These steps recursively estimate the state probability distribution over time."
        }
    )

    # Assessment 4.5: Lab Exercise - 1D Kalman Filter
    assessment_4_5 = Assessment(
        chapter_id=chapter_4.id,
        title="Lab 4.5: 1D Kalman Filter Implementation",
        description="Implement a 1D Kalman filter for position estimation.",
        assessment_type="LAB_EXERCISE",
        difficulty="INTERMEDIATE",
        max_score=10,
        content={
            "task_id": "4.5",
            "task_title": "1D Kalman Filter for Position",
            "problem_statement": "Implement a 1D Kalman filter to estimate position from noisy measurements.",
            "specifications": "Create a Kalman filter that estimates position and velocity from noisy position measurements. Implement both prediction and update steps.",
            "time_estimate": "60 minutes",
            "expected_patterns": [
                r"kalman_gain",
                r"prediction_step",
                r"update_step",
                r"state_transition",
                r"measurement_model"
            ],
            "rubric": {
                "10_points": "Complete implementation with correct prediction and update steps, proper covariance updates, accurate estimation compared to ground truth.",
                "7-9_points": "Implementation works correctly but has minor issues with covariance handling or documentation.",
                "4-6_points": "Function has correct general approach but implementation errors in prediction or update steps.",
                "1-3_points": "Basic structure is present but implementation is significantly incorrect.",
                "0_points": "No submission or code does not run."
            },
            "starter_code_path": "specs/1-book-curriculum/chapters/chapter-4/lab/lab_4_1/src/lab_4_1_node.py",
            "expected_output_path": "specs/1-book-curriculum/chapters/chapter-4/lab/lab_4_1/expected_output.txt"
        }
    )

    # Assessment 4.6: Integration Challenge - IMU-Odometry Fusion
    assessment_4_6 = Assessment(
        chapter_id=chapter_4.id,
        title="Challenge 4.6: IMU-Odometry Fusion",
        description="Implement sensor fusion for mobile robot localization.",
        assessment_type="CAPSTONE_CHALLENGE",
        difficulty="ADVANCED",
        max_score=5,
        content={
            "challenge_id": "4.6",
            "challenge_title": "IMU-Odometry Sensor Fusion",
            "scenario": "A mobile robot needs to fuse IMU and wheel odometry data for robust localization.",
            "task": "Implement an Extended Kalman Filter that fuses IMU angular rate and wheel odometry to estimate robot pose.",
            "decision_points": [
                "State representation (e.g., [x, y, theta, vx, vy, omega])",
                "Process and measurement noise modeling",
                "Handling of different sensor update rates"
            ],
            "success_metrics": [
                "Accurate pose estimation compared to ground truth",
                "Robustness to sensor noise and drift",
                "Real-time performance"
            ],
            "rubric": {
                "5_points": "Complete EKF implementation with appropriate state representation, correct Jacobians, accurate estimation with proper noise modeling.",
                "3-4_points": "Implementation works but has issues with Jacobian computation or noise modeling.",
                "1-2_points": "Basic approach implemented but with significant errors in EKF equations.",
                "0_points": "No submission or solution does not implement EKF correctly."
            }
        }
    )

    # Add all assessments to database
    assessments = [
        assessment_4_1,
        assessment_4_2,
        assessment_4_3,
        assessment_4_4,
        assessment_4_5,
        assessment_4_6
    ]

    for assessment in assessments:
        db.add(assessment)

    db.commit()

    logger.info(f"Seeded {len(assessments)} assessments for Chapter 4")
    return assessments


def main():
    """Main seeding function."""
    db = SessionLocal()
    try:
        logger.info("Starting assessment seeding for Chapters 2-4...")
        
        # Seed assessments for all chapters
        assessments_2 = seed_chapter_2_assessments(db)
        assessments_3 = seed_chapter_3_assessments(db)
        assessments_4 = seed_chapter_4_assessments(db)
        
        logger.info("✅ Assessment seeding for Chapters 2-4 complete")

        # Print summary
        print("\n" + "="*60)
        print("Chapters 2-4 Assessments Seeded Successfully")
        print("="*60)
        print(f"Chapter 2: {len(assessments_2)} assessments")
        for i, assessment in enumerate(assessments_2, 1):
            print(f"  {i}. {assessment.title} ({assessment.assessment_type}, {assessment.difficulty}, {assessment.max_score} pts)")
        
        print(f"\nChapter 3: {len(assessments_3)} assessments")
        for i, assessment in enumerate(assessments_3, 1):
            print(f"  {i}. {assessment.title} ({assessment.assessment_type}, {assessment.difficulty}, {assessment.max_score} pts)")
        
        print(f"\nChapter 4: {len(assessments_4)} assessments")
        for i, assessment in enumerate(assessments_4, 1):
            print(f"  {i}. {assessment.title} ({assessment.assessment_type}, {assessment.difficulty}, {assessment.max_score} pts)")
        
        total_assessments = len(assessments_2) + len(assessments_3) + len(assessments_4)
        total_points = sum(a.max_score for a in assessments_2 + assessments_3 + assessments_4)
        
        print("="*60)
        print(f"\nTotal: {total_assessments} assessments, {total_points} points")
        print("\nNext steps:")
        print("1. Start backend: cd backend && uvicorn src.main:app --reload")
        print("2. View assessments: GET /api/assessments/chapter/{2,3,4}")
        print("3. Submit quiz: POST /api/assessments/quiz/submit")

    except Exception as e:
        logger.error(f"Seeding failed: {e}")
        db.rollback()
        raise
    finally:
        db.close()


if __name__ == "__main__":
    main()