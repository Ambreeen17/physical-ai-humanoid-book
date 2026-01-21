---
id: 1
title: Establish Project Constitution
stage: constitution
date: 2025-12-31
surface: agent
model: claude-haiku-4-5-20251001
feature: none
branch: main
user: system
command: /sp.constitution
labels: ["constitution", "governance", "agent-system", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:
  - Constitution template fully filled with no placeholders remaining
  - All 5 core principles defined with rationale
  - 11 agent invocation rules documented
  - Technical requirements (ROS 2, Gazebo, Isaac Sim) specified
  - Brand voice and governance framework established
---

## Prompt

```
/sp.constitution

Project Name:
Physical AI & Humanoid Robotics – An AI-Native Textbook

Vision:
To create a next-generation AI-native textbook that teaches Physical AI and Humanoid Robotics by unifying embodied intelligence, simulation-to-real robotics, and AI agents. The book will not only teach content but actively use intelligent agents to write, personalize, translate, and interact with learners.

Target Audience:
Advanced beginners to intermediate learners with basic programming or AI knowledge who want to transition into robotics, embodied AI, and humanoid systems.

Educational Philosophy:
- Intelligence must be embodied in the physical world
- Simulation is the gateway to real robots
- AI agents are collaborators, not just tools
- Learning must be hands-on, contextual, and adaptive

---

Reusable Agent System (Core Design Principle):

This project MUST use reusable Claude Code agents and subagents.
Each agent has a single responsibility and may be reused across chapters and future books.

Declared Agents:
1. Book Orchestrator Agent
2. Curriculum Analyst Agent
3. Research Agent
4. Chapter Author Agent
5. Robotics Lab Agent
6. Diagram & Visualization Agent
7. Assessment Agent
8. Personalization Agent
9. Localization Agent (Urdu)
10. RAG Indexing Agent
11. QA & Consistency Agent

---

Agent Invocation Rules (MANDATORY):

[Full agent rules provided in constitution document...]

---

Core Principles:
1. Embodied Intelligence First
2. Simulation-to-Real Continuity
3. Agent-Driven Knowledge Creation
4. Hardware-Aware Teaching
5. Modular, Reusable, and Scalable Design

---

Success Criteria:
- Book deployed using Docusaurus on GitHub Pages
- Full coverage of ROS 2, Gazebo, Unity, NVIDIA Isaac, and VLA
- Embedded RAG chatbot answering book-only questions
- Content personalization based on learner background
- One-click Urdu translation per chapter
- Demonstrated use of reusable Claude Code subagents
- Capstone: Autonomous humanoid robot in simulation

---

Constraints:
- ROS 2 Humble on Ubuntu 22.04
- NVIDIA Isaac Sim hardware requirements respected
- Beginner-safe but industry-accurate explanations
- No closed or proprietary dependencies
- Must support both on-prem and cloud labs

---

Brand Voice:
- Technical but accessible
- Engineering-driven
- Visionary and future-focused
- Clear, structured, and precise
```

## Response snapshot

**Constitution v1.0.0 ratified and documented.**

The constitution has been completed with:

- **5 Core Principles**: Each grounded in project values (embodied learning, sim-to-real portability, agent collaboration, hardware realism, modularity)
- **11 Agent Invocation Rules**: Explicit workflow from orchestration through QA, with each agent's responsibility documented
- **Technical Requirements**: ROS 2 Humble + Ubuntu 22.04 as baseline, with Gazebo/Isaac Sim for labs, Docker containerization required, all open-source
- **Governance Framework**: Amendment procedure with semantic versioning (MAJOR/MINOR/PATCH), compliance review process, periodic governance review
- **Content Architecture**: Clear scope (in/out), learning outcomes, target audience, and brand voice alignment

File `.specify/memory/constitution.md` updated (v1.0.0 | Ratified: 2025-12-31).

## Outcome

- ✅ Impact: Constitution establishes governance, principles, and agent workflows for entire textbook project. Serves as authoritative source for all downstream specs, plans, and chapter approvals.
- 🧪 Tests: All 5 core principles include explicit MUST statements; all 11 agents have defined entry points and responsibilities; governance process includes amendment versioning strategy
- 📁 Files: `.specify/memory/constitution.md` (single source of truth for project governance)
- 🔁 Next prompts: Curriculum Analyst Agent → convert syllabus to chapter structure; Book Orchestrator Agent → plan chapter workflow
- 🧠 Reflection: Constitution positions agents as first-class collaborators in knowledge creation, not hidden tools. This design choice is central to project mission: teaching *with* agents *by example*.

## Evaluation notes (flywheel)

- Failure modes observed: None. Template was cleanly filled; all placeholders resolved; governance procedure is clear and actionable.
- Graders run and results (PASS/FAIL): PASS — No unresolved brackets; all dates ISO format; principles declarative and testable; version line matches amendment record.
- Prompt variant (if applicable): null
- Next experiment: Run `/sp.specify` to define feature specs aligned to constitution; then invoke book-orchestrator to sequence curriculum.
