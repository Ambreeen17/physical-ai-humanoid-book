"""
Phase 5-7 Orchestration Script: Chapters 5-16 Production
Executes the 10-agent pipeline for Parts II-IV (Perception & Control, Planning & Learning, Integration & Deployment)
"""
import sys
import os
sys.path.append(os.path.join(os.path.dirname(__file__), '..', 'backend'))

from src.agents import initialize_agents
from src.agents.protocol import AgentInput, AgentStatus
import logging

# Setup logging
from src.logging.logger import setup_logging
setup_logging()
logger = logging.getLogger(__name__)

# Chapter metadata for Parts II-IV
CHAPTERS = [
    {
        "number": 5,
        "title": "Computer Vision for Robotics",
        "description": "Image processing, feature detection, object recognition, visual SLAM",
        "learning_objectives": [
            "Apply fundamental image processing techniques for robotic perception",
            "Implement feature detection and matching algorithms for visual tracking",
            "Design object recognition systems using deep learning approaches",
            "Integrate visual-inertial odometry for robust localization",
            "Evaluate computer vision performance in real-time robotic applications"
        ],
        "key_topics": [
            "Image processing fundamentals",
            "Feature detection and matching",
            "Object recognition and detection",
            "Visual SLAM",
            "3D vision and reconstruction",
            "Hardware considerations"
        ],
        "hardware_context": "Focuses on Unitree G1 vision capabilities with Intel RealSense D435i and NVIDIA Jetson Orin",
        "sim_tools": ["Gazebo Harmonic", "Isaac Sim", "PyBullet", "OpenCV"]
    },
    {
        "number": 6,
        "title": "Control Theory Fundamentals",
        "description": "PID control, state-space methods, adaptive control for robotics",
        "learning_objectives": [
            "Design PID controllers for robotic systems with appropriate tuning methods",
            "Apply state-space control techniques for multi-input multi-output systems",
            "Implement adaptive control algorithms for uncertain robotic dynamics",
            "Analyze stability and performance of robotic control systems",
            "Integrate control systems with perception and planning modules"
        ],
        "key_topics": [
            "Classical control theory",
            "State-space control",
            "Nonlinear control",
            "Adaptive control",
            "Optimal control",
            "Hardware considerations"
        ],
        "hardware_context": "Unitree G1 joint control with high-bandwidth servo motors and ROS 2 control framework",
        "sim_tools": ["Gazebo Harmonic", "ROS 2 Control", "Matlab/Simulink"]
    },
    {
        "number": 7,
        "title": "Motion Planning",
        "description": "Sampling-based, optimization-based, and real-time planning for robotics",
        "learning_objectives": [
            "Implement sampling-based motion planning algorithms (PRM, RRT, RRT*)",
            "Apply optimization-based planning methods (CHOMP, STOMP, TrajOpt)",
            "Design real-time replanning systems for dynamic environments",
            "Integrate perception and planning for obstacle avoidance",
            "Evaluate planning performance in terms of completeness, optimality, and efficiency"
        ],
        "key_topics": [
            "Configuration space and planning fundamentals",
            "Sampling-based planning",
            "Optimization-based planning",
            "Real-time planning",
            "Multi-robot planning",
            "Hardware considerations"
        ],
        "hardware_context": "Unitree G1 navigation with LiDAR, cameras, and ROS 2 navigation stack",
        "sim_tools": ["Gazebo Harmonic", "MoveIt!", "OMPL", "Nav2"]
    },
    {
        "number": 8,
        "title": "Manipulation & Grasping",
        "description": "Kinematic control, grasp planning, force control, dexterous manipulation",
        "learning_objectives": [
            "Analyze and control robotic manipulator kinematics for manipulation tasks",
            "Plan grasps for objects of various shapes and properties",
            "Implement force and impedance control for compliant manipulation",
            "Design dexterous manipulation strategies using multi-fingered hands",
            "Integrate perception and planning for autonomous manipulation"
        ],
        "key_topics": [
            "Manipulator kinematics for grasping",
            "Grasp planning and synthesis",
            "Force control and impedance control",
            "Dexterous manipulation",
            "Learning-based manipulation",
            "Hardware considerations"
        ],
        "hardware_context": "Unitree G1 manipulation with 6-DOF arms and adaptive grippers",
        "sim_tools": ["Gazebo Harmonic", "MoveIt!", "GraspIt!", "PyBullet"]
    },
    {
        "number": 9,
        "title": "Task & Motion Planning",
        "description": "Symbolic-geometric planning, temporal planning, hierarchical planning",
        "learning_objectives": [
            "Formulate robotic tasks using symbolic representations and PDDL",
            "Integrate task planning with motion planning for complex behaviors",
            "Implement temporal planning for multi-step robotic tasks",
            "Design hierarchical planning architectures for complex tasks",
            "Evaluate planning performance in terms of solution quality and computational efficiency"
        ],
        "key_topics": [
            "Symbolic task planning",
            "Task and motion planning integration",
            "Temporal planning",
            "Hierarchical planning",
            "Learning-based planning",
            "Hardware considerations"
        ],
        "hardware_context": "Unitree G1 task planning with ROS 2 planning frameworks",
        "sim_tools": ["Gazebo Harmonic", "ROSPlan", "MoveIt Task Composer", "PDDL planners"]
    },
    {
        "number": 10,
        "title": "Reinforcement Learning for Robotics",
        "description": "Model-free and model-based RL, deep RL, safe exploration for robotics",
        "learning_objectives": [
            "Apply model-free RL algorithms (Q-Learning, SARSA, Actor-Critic) to robotic tasks",
            "Implement deep RL methods (DQN, PPO, SAC) for continuous control problems",
            "Design model-based RL approaches for sample-efficient learning",
            "Implement safe exploration strategies for real robot deployment",
            "Evaluate RL performance in terms of sample efficiency, stability, and safety"
        ],
        "key_topics": [
            "Foundations of reinforcement learning",
            "Model-free reinforcement learning",
            "Deep reinforcement learning for robotics",
            "Model-based reinforcement learning",
            "Safe RL and exploration",
            "Hardware considerations"
        ],
        "hardware_context": "Unitree G1 learning with on-board computation and safety layers",
        "sim_tools": ["Gazebo Harmonic", "Isaac Gym", "MuJoCo", "Stable Baselines3"]
    },
    {
        "number": 11,
        "title": "Imitation Learning",
        "description": "Behavioral cloning, inverse RL, generative approaches for robotics",
        "learning_objectives": [
            "Implement behavioral cloning for learning from expert demonstrations",
            "Apply inverse reinforcement learning to infer reward functions",
            "Design generative adversarial approaches for imitation learning",
            "Address distribution shift and covariate shift in imitation learning",
            "Evaluate imitation learning performance in terms of policy quality and generalization"
        ],
        "key_topics": [
            "Foundations of imitation learning",
            "Behavioral cloning",
            "Inverse reinforcement learning",
            "Generative adversarial imitation learning",
            "Learning from observation",
            "Hardware considerations"
        ],
        "hardware_context": "Unitree G1 learning from human demonstrations via teleoperation",
        "sim_tools": ["Gazebo Harmonic", "PyTorch", "TensorFlow", "ROS 2 interfaces"]
    },
    {
        "number": 12,
        "title": "Vision-Language-Action Models",
        "description": "Multimodal learning, VLA models, embodied learning for robotics",
        "learning_objectives": [
            "Understand the architecture and training of Vision-Language-Action models",
            "Implement multimodal fusion techniques for perception-action tasks",
            "Fine-tune pre-trained VLA models for specific robotic tasks",
            "Design embodied learning approaches for VLA models",
            "Evaluate VLA performance in terms of perception accuracy, language understanding, and action success"
        ],
        "key_topics": [
            "Foundations of multimodal learning",
            "Vision-Language models for robotics",
            "Vision-Language-Action integration",
            "Embodied learning and training",
            "Instruction following and task generalization",
            "Hardware considerations"
        ],
        "hardware_context": "Unitree G1 with vision, language, and action integration on Jetson Orin",
        "sim_tools": ["Isaac Sim", "PyTorch", "Transformers", "CLIP models"]
    },
    {
        "number": 13,
        "title": "System Integration",
        "description": "Component integration, debugging, validation for complex robotic systems",
        "learning_objectives": [
            "Design modular system architectures for complex robotic systems",
            "Integrate perception, planning, control, and learning components",
            "Implement debugging and monitoring systems for robotic applications",
            "Validate system performance through systematic testing",
            "Deploy integrated robotic systems in real-world environments"
        ],
        "key_topics": [
            "System architecture and design",
            "Component integration",
            "System debugging and monitoring",
            "System validation and testing",
            "Safety and reliability",
            "Hardware considerations"
        ],
        "hardware_context": "Unitree G1 with all subsystems integrated via ROS 2",
        "sim_tools": ["Gazebo Harmonic", "ROS 2", "DDS", "Monitoring tools"]
    },
    {
        "number": 14,
        "title": "Safety & Robustness",
        "description": "Risk assessment, safety mechanisms, fault tolerance for robotics",
        "learning_objectives": [
            "Conduct risk assessment and hazard analysis for robotic systems",
            "Implement safety mechanisms and protective measures",
            "Design fault-tolerant systems with graceful degradation",
            "Validate safety and robustness through systematic testing",
            "Evaluate safety performance in terms of risk reduction and reliability"
        ],
        "key_topics": [
            "Risk assessment and hazard analysis",
            "Safety mechanisms and protective measures",
            "Fault detection and recovery",
            "Robustness and resilience",
            "Human-robot interaction safety",
            "Hardware considerations"
        ],
        "hardware_context": "Unitree G1 with multiple safety layers and ISO 13482 compliance",
        "sim_tools": ["Gazebo Harmonic", "Safety analysis tools", "FMEA", "FTA"]
    },
    {
        "number": 15,
        "title": "Deployment & Operations",
        "description": "Deployment strategies, monitoring, maintenance, scaling for robotics",
        "learning_objectives": [
            "Plan and execute robotic system deployments in real-world environments",
            "Implement monitoring and logging systems for operational robots",
            "Design maintenance and update procedures for deployed systems",
            "Scale robotic deployments across multiple units and environments",
            "Evaluate deployment success in terms of reliability, maintainability, and operational efficiency"
        ],
        "key_topics": [
            "Deployment planning and preparation",
            "Deployment procedures and protocols",
            "Monitoring and observability",
            "Maintenance and updates",
            "Scaling and fleet management",
            "Hardware considerations"
        ],
        "hardware_context": "Unitree G1 deployment with operational and maintenance procedures",
        "sim_tools": ["Deployment tools", "Monitoring systems", "Fleet management", "OTA update tools"]
    },
    {
        "number": 16,
        "title": "Capstone Project",
        "description": "Integrative project combining all concepts from the textbook",
        "learning_objectives": [
            "Synthesize knowledge from all previous chapters into a cohesive project",
            "Design and implement a complete robotic system with multiple capabilities",
            "Integrate perception, planning, control, learning, and safety components",
            "Deploy and validate the system in a real-world or simulated environment",
            "Evaluate and present the project with technical documentation and analysis"
        ],
        "key_topics": [
            "Project planning and design",
            "System implementation",
            "Integration and validation",
            "Documentation and presentation",
            "Advanced integration challenges",
            "Hardware considerations"
        ],
        "hardware_context": "Unitree G1 integration of all learned concepts into a complete system",
        "sim_tools": ["Gazebo Harmonic", "Complete toolchain", "Integration tools", "Validation environments"]
    }
]


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
    logger.info("Phase 5-7: Chapters 5-16 Production")
    logger.info("Parts II-IV: Perception & Control, Planning & Learning, Integration & Deployment")
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
    print("Phase 5-7 Execution Summary")
    print("="*60)

    for result in results:
        status = "SUCCESS" if result['success'] else "FAILED"  # Changed from emojis to text
        print(f"Chapter {result['chapter']}: {result['title']} - {status}")

    total_chapters = len(results)
    successful_chapters = sum(1 for r in results if r['success'])

    print(f"\nTotal: {successful_chapters}/{total_chapters} chapters completed successfully")

    if successful_chapters == total_chapters:
        print("\nPhase 5-7 complete! Parts II-IV ready for integration.")
        print("\nNext steps:")
        print("1. Integrate Chapters 5-16 into Docusaurus")
        print("2. Seed assessments for Chapters 5-16")
        print("3. Upload RAG chunks to Qdrant")
        print("4. Complete full textbook integration")
        print("5. Begin final validation and testing")
        return 0
    else:
        print("\nPhase 5-7 incomplete. Review errors above.")
        return 1


if __name__ == "__main__":
    import asyncio
    sys.exit(asyncio.run(main()))