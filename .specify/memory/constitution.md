# Physical AI & Humanoid Robotics – An AI-Native Textbook Constitution

<!--
Sync Impact Report: v1.0.0 (initial ratification)
- Stage: Constitution Creation
- New Principles: 5 (Embodied Intelligence First, Simulation-to-Real Continuity, Agent-Driven Knowledge Creation, Hardware-Aware Teaching, Modular and Scalable Design)
- New Sections: Agent Invocation Rules, Technical Requirements, Brand Voice
- Templates Requiring Alignment: spec-template.md, plan-template.md, tasks-template.md, phr-template.prompt.md
- No prior version; first constitution for this project
-->

## Core Principles

### I. Embodied Intelligence First

Every concept in this textbook MUST be grounded in the physical world. Theory alone is insufficient; all chapters MUST include hands-on experiments, simulations, or real robot interactions. Abstraction must follow embodiment—explain why hardware constraints matter before introducing algorithms. Learners MUST understand how perception, actuation, and control interact in a closed loop.

**Rationale**: Physical AI is fundamentally different from pure software AI. Learners cannot internalize concepts without experiencing the friction between theory and reality—latency, sensor noise, actuator limits, gravity, friction.

### II. Simulation-to-Real Continuity

Simulation is the bridge to real robotics, not a replacement. Every lab MUST start in simulation (Gazebo, Isaac Sim, Unity), validate in simulation, then provide a clear path to real hardware. Code MUST be portable between simulators and real robots. No simulator-only labs or purely theoretical exercises.

**Rationale**: Sim-to-real gap is real but bridgeable with discipline. Learners gain confidence and speed in simulation, then transfer to hardware with minimal friction. Reusable code across both environments is non-negotiable.

### III. Agent-Driven Knowledge Creation

This textbook is created *by* intelligent agents, *for* learners who will use agents. Every chapter MUST be produced using Claude Code agents (research, writing, visualization, assessment). The production workflow itself MUST demonstrate how AI agents collaborate on complex tasks. Agents are not hidden; their contributions are transparent and learnable.

**Rationale**: If we teach with agents as invisible tools, learners miss the deeper lesson: AI agents are collaborators that can amplify human expertise. By explicitly using agents for research, writing, and assessment, we teach by example.

### IV. Hardware-Aware Teaching

All explanations MUST respect real hardware constraints and capabilities. Never abstract away actuator limits, sensor resolution, or computational budgets. Use industry-standard hardware (UR robots, Boston Dynamics, humanoids) as reference, but always provide open-source alternatives (HEBI, CHOP, STOMPY). Every chapter MUST address cost, maintenance, and practical deployment.

**Rationale**: Learners transitioning to industry need realistic expectations. Omitting constraints leads to failed projects and frustration.

### V. Modular, Reusable, and Scalable Design

Every artifact—chapter, lab, assessment, diagram, translation—MUST be modular and reusable across the textbook and future projects. Agents MUST have single responsibilities and work together orchestrated by a Book Orchestrator. No duplicated content; no chapter-specific workarounds.

**Rationale**: Scalability to multiple books, translations, and personalized learning paths depends on modular design from day one.

## Agent Invocation Rules (MANDATORY)

All content creation MUST follow this workflow:

1. **Book Orchestrator Agent**: Invoked at project start and at the beginning of each chapter. Decides which subagents to call and in what order.
2. **Curriculum Analyst Agent**: Converts syllabus into textbook chapters with learning outcomes and prerequisite dependencies.
3. **Robotics Research Agent**: Invoked at the beginning of every chapter. Provides accurate, up-to-date robotics and AI knowledge with industry context.
4. **Chapter Author Agent**: Writes conceptual and explanatory content after research completion. Builds intuition before technical details.
5. **Robotics Lab Generator**: Produces ROS 2, Gazebo, Isaac Sim, and VLA labs for every technical chapter. Code MUST be executable on Ubuntu 22.04 + ROS 2 Humble.
6. **Diagram & Visualization Agent**: Creates ASCII, Mermaid, and Docusaurus-compatible diagrams for architectures, pipelines, and system flows.
7. **Assessment Generator**: Produces progressive quizzes, coding labs, simulation challenges, and capstone-aligned tasks at chapter end.
8. **Personalization Agent**: Dynamically adjusts content for learners with different backgrounds (software/hardware/AI emphasis).
9. **Localization Agent (Urdu)**: Translates chapters while preserving technical accuracy and code examples (on user request).
10. **RAG Indexing Agent**: Prepares completed chapters for semantic chunking and embedding into a vector database for book-aware chatbot retrieval.
11. **QA & Consistency Agent**: Validates terminology, versions, hardware assumptions, and cross-chapter alignment before publishing.

## Technical Requirements

### Tooling and Platforms

- **ROS 2 Humble** on Ubuntu 22.04 (baseline). All labs must work on this stack.
- **Gazebo** (open-source robotics simulator) for foundational labs.
- **NVIDIA Isaac Sim** for advanced sim-to-real labs (hardware requirements respected; fallback to Gazebo always provided).
- **Unity** (optional) for visualization-heavy chapters.
- **VLA (Vision Language Action)** frameworks for embodied language understanding.
- **Docusaurus** for textbook deployment on GitHub Pages.
- **Python 3.9+** as primary language; C++ for performance-critical sections with bindings.

### Lab Requirements (Non-Negotiable)

- Every lab MUST include step-by-step instructions executable by a beginner.
- All labs MUST provide a Docker/container setup for reproducibility.
- Code MUST be tested on actual hardware or verified in simulation before publication.
- Every lab MUST include expected output and common failure modes with solutions.
- No closed-source dependencies; all tools must be open-source or freely available.

### Content Constraints

- All chapters MUST support on-prem (local GPU/CPU) and cloud-based (e.g., Google Colab, AWS) execution paths.
- No proprietary APIs or frameworks without an open-source equivalent documented.
- Beginner-safe but industry-accurate—never oversimplify at the cost of correctness.
- Every code example MUST be runnable and tested.

## Brand Voice and Tone

- **Technical but Accessible**: Explain the "why" before the "how"; use metaphors grounded in physical intuition.
- **Engineering-Driven**: Prefer concrete examples, working code, and deployable systems over pure theory.
- **Visionary and Future-Focused**: Contextualize each chapter within the broader trajectory of embodied AI and robotics; connect to real-world applications (manufacturing, healthcare, exploration).
- **Clear, Structured, and Precise**: Headings guide navigation; code is self-documenting; no ambiguous language. Use MUST/SHOULD/MAY terminology consistently.

## Content Architecture

### Target Audience

Advanced beginners to intermediate learners with basic programming or AI knowledge who want to transition into robotics, embodied AI, and humanoid systems.

### Learning Outcomes (Overarching)

By completing this textbook, learners will:

- Understand embodied intelligence principles and why simulation-to-real matters.
- Build and deploy ROS 2 nodes, services, and actions on a humanoid robot.
- Design and run closed-loop control systems with feedback.
- Implement perception pipelines (vision, IMU, force feedback).
- Develop manipulation and navigation strategies in simulation and on real hardware.
- Train and deploy simple vision-language models for robotic tasks.
- Demonstrate autonomous humanoid behavior in a capstone project.

### Scope (In)

- ROS 2 fundamentals and advanced patterns.
- Gazebo and Isaac Sim simulation.
- Perception (vision, depth, tactile) and sensor fusion.
- Control systems (kinematics, dynamics, trajectory planning).
- Manipulation and grasping.
- Navigation and path planning.
- Vision-language action models.
- Humanoid-specific challenges (balance, whole-body coordination).
- Capstone: Autonomous humanoid in simulation and on real hardware (where available).

### Scope (Out)

- Deep reinforcement learning (mention, don't teach in depth).
- Theoretical dynamics proofs (concepts only, not derivations).
- Manufacturing-specific control (e.g., CNC).
- Closed proprietary frameworks (e.g., Tesla Optimus internals).
- Advanced computer vision (covered at application level, not research level).

## Governance

### Amendment Procedure

1. **Proposal**: Any team member may propose an amendment by creating an issue or discussion.
2. **Rationale**: Amendment MUST include:
   - Clear problem or gap being addressed.
   - Proposed change to principle or section.
   - Impact on existing chapters and agents.
   - Migration plan (if backward-incompatible).
3. **Approval**: Lead architext or appointed governance reviewer approves.
4. **Versioning**:
   - MAJOR bump: Principle removal or backward-incompatible redefinition.
   - MINOR bump: New principle, expanded guidance, or new section.
   - PATCH bump: Clarifications, wording fixes, typos.
5. **Documentation**: Update constitution, flag affected templates and chapters, commit with clear message.

### Compliance and Enforcement

- **Chapter Reviews**: Every chapter MUST be reviewed against this constitution before publication.
- **Agent Alignment**: Agent instructions (prompts, templates) MUST align with these principles and be updated when constitution changes.
- **Periodic Review**: Constitution and agent workflows reviewed quarterly or after significant project pivots.
- **Runtime Guidance**: See `.specify/memory/` and `.specify/templates/` for implementation-level guidance on testing, documentation, and deployment.

### Amendment Record

- **v1.0.0** | **Ratified**: 2025-12-31 | **Last Amended**: 2025-12-31
  - Initial constitution for Physical AI & Humanoid Robotics textbook project.
  - Five core principles, 11-agent system, technical requirements, governance framework established.

**Version**: 1.0.0 | **Ratified**: 2025-12-31 | **Last Amended**: 2025-12-31
