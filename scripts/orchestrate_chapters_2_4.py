"""
Phase 4 Orchestration Script: Chapters 2-4 Production
Executes the 10-agent pipeline for Part I: Foundations (Kinematics, Sensors, State Estimation)
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend'))

from src.agents import initialize_agents
from src.agents.protocol import AgentInput, AgentStatus
import logging

logger = logging.getLogger(__name__)


# Chapter metadata
CHAPTERS = [
    {
        "number": 2,
        "title": "Kinematics & Dynamics",
        "description": "Joint spaces, forward/inverse kinematics, equations of motion",
        "learning_objectives": [
            "Define joint space and configuration space for robot manipulators",
            "Compute forward kinematics using Denavit-Hartenberg parameters",
            "Solve inverse kinematics for 2-DOF and 3-DOF arms",
            "Apply Lagrangian mechanics to derive equations of motion",
            "Simulate robot dynamics in MuJoCo and Gazebo"
        ],
        "key_topics": [
            "Joint spaces and configuration spaces",
            "Denavit-Hartenberg (DH) parameters",
            "Forward kinematics computation",
            "Inverse kinematics (analytical and numerical)",
            "Lagrangian dynamics",
            "Newton-Euler formulation",
            "Simulation of multi-body dynamics"
        ],
        "hardware_context": "Focuses on Unitree G1 arm kinematics (6-DOF manipulation chain)",
        "sim_tools": ["MuJoCo", "Gazebo Harmonic", "PyBullet"]
    },
    {
        "number": 3,
        "title": "Sensors & Actuators",
        "description": "LiDAR, cameras, servo motors, force-torque sensors",
        "learning_objectives": [
            "Characterize sensor performance (resolution, noise, latency)",
            "Process LiDAR point clouds for obstacle detection",
            "Calibrate RGB-D cameras for depth estimation",
            "Select actuators based on torque, speed, and precision requirements",
            "Implement sensor fusion using Kalman filters"
        ],
        "key_topics": [
            "Sensor taxonomy (proprioceptive vs exteroceptive)",
            "LiDAR: 2D vs 3D, time-of-flight vs phase-shift",
            "Cameras: RGB, depth (structured light, stereo, ToF)",
            "IMU: accelerometers, gyroscopes, magnetometers",
            "Force-torque sensors for manipulation",
            "Servo motors: DC, stepper, brushless",
            "Actuator selection criteria"
        ],
        "hardware_context": "Unitree G1 sensor suite (3D LiDAR, RGB-D camera, IMU, joint encoders)",
        "sim_tools": ["Isaac Sim (sensor simulation)", "Gazebo sensor plugins"]
    },
    {
        "number": 4,
        "title": "State Estimation",
        "description": "Kalman filters, particle filters, sensor fusion",
        "learning_objectives": [
            "Implement Kalman filters for position/velocity estimation",
            "Apply Extended Kalman Filter (EKF) for nonlinear systems",
            "Use particle filters for non-Gaussian distributions",
            "Fuse IMU and odometry data for robust localization",
            "Evaluate estimation accuracy and computational cost"
        ],
        "key_topics": [
            "Bayesian filtering framework",
            "Kalman Filter: prediction and update steps",
            "Extended Kalman Filter (EKF) for nonlinear dynamics",
            "Unscented Kalman Filter (UKF)",
            "Particle filters (Sequential Monte Carlo)",
            "Sensor fusion architectures",
            "SLAM (Simultaneous Localization and Mapping) basics"
        ],
        "hardware_context": "Mobile robot localization using wheel encoders, IMU, and LiDAR",
        "sim_tools": ["ROS 2 robot_localization package", "Gazebo"]
    }
]


import asyncio

async def orchestrate_chapter(chapter_metadata: dict):
    """
    Orchestrate 10-agent pipeline for a single chapter.

    Agents:
    1. Research Agent
    2. Chapter Author
    3. Diagram Generator
    4. Robotics Lab Generator
    5. Assessment Generator
    6. Personalization Agent
    7. Localization Agent (Urdu)
    8. RAG Indexing Agent
    9. QA Agent
    10. (Implicit) File Management
    """
    logger.info(f"Starting orchestration for Chapter {chapter_metadata['number']}: {chapter_metadata['title']}")

    # Initialize orchestrator with all agents
    orchestrator = initialize_agents()

    # Construct agent input
    agent_input = AgentInput(
        chapter_id=f"chapter_{chapter_metadata['number']}",
        chapter_number=chapter_metadata["number"],
        chapter_topic=chapter_metadata["title"],
    )

    # Add context as an attribute (not part of the dataclass constructor)
    agent_input.context = {
        "chapter_number": chapter_metadata["number"],
        "chapter_title": chapter_metadata["title"],
        "chapter_description": chapter_metadata["description"],
        "learning_objectives": chapter_metadata["learning_objectives"],
        "key_topics": chapter_metadata["key_topics"],
        "hardware_context": chapter_metadata["hardware_context"],
        "sim_tools": chapter_metadata["sim_tools"]
    }

    agent_input.constraints = [
        "Beginner-safe ROS 2 Humble labs (30-60 minutes)",
        "All code must run in Docker",
        "Hardware specs accurate as of 2025",
        "Assessments align with learning objectives",
        "Content personalized for Beginner/Intermediate/Advanced"
    ]

    # Execute orchestration pipeline
    try:
        output = await orchestrator.execute(agent_input)

        if output.status == AgentStatus.SUCCESS:
            logger.info(f"Chapter {chapter_metadata['number']} production complete")
            logger.info(f"Artifacts generated: {output.artifacts}")
            return output
        else:
            logger.error(f"Chapter {chapter_metadata['number']} production failed: {output.error_message}")
            return None

    except Exception as e:
        logger.error(f"Orchestration error for Chapter {chapter_metadata['number']}: {e}")
        return None


async def main():
    """Main execution function."""
    logger.info("="*60)
    logger.info("Phase 4: Chapters 2-4 Production")
    logger.info("Part I: Foundations (Kinematics, Sensors, State Estimation)")
    logger.info("="*60)

    results = []

    for chapter in CHAPTERS:
        print(f"\n{'='*60}")
        print(f"Chapter {chapter['number']}: {chapter['title']}")
        print(f"{'='*60}\n")

        result = await orchestrate_chapter(chapter)
        results.append({
            "chapter": chapter['number'],
            "title": chapter['title'],
            "success": result is not None and result.status == AgentStatus.SUCCESS
        })

    # Summary report
    print("\n" + "="*60)
    print("Phase 4 Execution Summary")
    print("="*60)

    for result in results:
        status = "SUCCESS" if result['success'] else "FAILED"  # Changed from emojis to text
        print(f"Chapter {result['chapter']}: {result['title']} - {status}")

    total_chapters = len(results)
    successful_chapters = sum(1 for r in results if r['success'])

    print(f"\nTotal: {successful_chapters}/{total_chapters} chapters completed successfully")

    if successful_chapters == total_chapters:
        print("\nPhase 4 complete! Part I: Foundations ready for integration.")
        print("\nNext steps:")
        print("1. Integrate Chapters 2-4 into Docusaurus")
        print("2. Seed assessments for Chapters 2-4")
        print("3. Upload RAG chunks to Qdrant")
        print("4. Begin Phase 5 (Part II: Perception & Control)")
        return 0
    else:
        print("\nPhase 4 incomplete. Review errors above.")
        return 1


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
