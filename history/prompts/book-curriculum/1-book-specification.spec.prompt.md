---
id: 1
title: Specification for AI-Native Textbook Curriculum
stage: spec
date: 2025-12-31
surface: agent
model: claude-haiku-4-5-20251001
feature: book-curriculum
branch: 1-book-curriculum
user: system
command: /sp.specify
labels: ["spec", "book-curriculum", "chapter-structure", "agent-orchestration", "textbook"]
links:
  spec: specs/1-book-curriculum/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-book-curriculum/spec.md
  - specs/1-book-curriculum/checklists/requirements.md
  - history/prompts/book-curriculum/1-book-specification.spec.prompt.md
tests:
  - All 20 specification quality checklist items pass
  - 4 user stories with P1/P2 priorities defined with acceptance scenarios
  - 21 functional requirements with clear inputs/outputs
  - 15 measurable success criteria (quantified thresholds)
  - 10-step agent orchestration sequence per chapter documented
  - No implementation details or technology-specific constraints leaked
  - Zero [NEEDS CLARIFICATION] markers
---

## Prompt

```
/sp.specify

Context:
This project implements an AI-native textbook titled
"Physical AI & Humanoid Robotics".

The book teaches embodied intelligence, humanoid robotics,
and simulation-to-real AI using ROS 2, Gazebo, Unity,
NVIDIA Isaac, and Vision-Language-Action systems.

The project must strictly follow the constitution and
use reusable Claude Code agents.

---

Specification Goals:
1. Convert the approved Table of Contents into executable book specifications
2. Define chapter-level responsibilities for agents
3. Ensure every chapter produces:
   - Theory
   - Diagrams
   - Labs (where applicable)
   - Assessments
   - RAG-ready content

---

Chapter Specification Rules:

For EACH chapter:
- Invoke Book Orchestrator Agent first
- Invoke Research Agent to gather domain knowledge
- Invoke Chapter Author Agent to write explanations
- Invoke Diagram Agent for system architectures
- Invoke Robotics Lab Agent for hands-on chapters
- Invoke Assessment Agent for quizzes and projects
- Invoke RAG Indexing Agent after chapter completion
- Invoke QA & Consistency Agent before publishing

---

Functional Requirements:

Content:
- Beginner-safe, industry-accurate explanations
- Hardware-aware (Jetson, RTX, sensors)
- Simulation-first, real-world aligned
- AI-agent-aware learning flow

AI-Native Features:
- Embedded RAG chatbot answering book-only questions
- Ability to answer from user-selected text only
- Content personalization based on user background
- Urdu translation toggle per chapter

---

Technical Stack:
- Documentation: Docusaurus
- Backend: FastAPI
- Vector DB: Qdrant Cloud
- Database: Neon Serverless Postgres
- Auth: better-auth
- AI: OpenAI Agents / ChatKit
- Robotics: ROS 2 Humble, Gazebo, Unity, NVIDIA Isaac

---

Quality Gates:
- Each chapter must pass QA Agent validation
- All labs must run on Ubuntu 22.04
- No chapter is considered complete without RAG indexing
- Capstone must integrate Voice, Vision, Navigation, and Manipulation

---

Output Expectations:
- Structured chapter specs
- Agent invocation plan per chapter
- Ready-to-implement tasks for /sp.implement

---

User Input (Chapter Outline):

Chapter 1: Introduction to Physical AI
- Book Orchestrator → assigns tasks to Research, Author, Diagram, Assessment
- Research Agent → collects latest embodied AI references
- Chapter Author Agent → writes chapter objectives, explanations
- Diagram Agent → specifies diagrams: AI-brain, robot-body map
- Assessment Agent → defines 3 multiple-choice + 1 short-lab exercise
- Personalization Agent → sets adaptive notes for beginner vs intermediate
- Localization Agent → marks Urdu translation sections
- RAG Indexing Agent → prepares chapter content for RAG embedding
- QA Agent → validates correctness, terminology, and coherence

Chapter 2: Sensors & Perception
- Book Orchestrator → invokes relevant agents
- Research Agent → sensor types, data fusion, calibration
- Chapter Author → theory & explanation
- Robotics Lab → RealSense simulation lab + LIDAR lab
- Diagram Agent → sensor architecture diagrams
- Assessment → quiz & lab exercise
- Personalization → beginner/intermediate notes
- Localization → Urdu sections
- RAG Indexing → embeddings ready
- QA → validation

Chapter 3–16: [Repeat same agent chain, tailored to content]
- ROS 2 chapters → Robotics Lab Agent adds ROS nodes + topics lab
- Gazebo/Unity → Lab Agent adds simulation setup + environment config
- Isaac → Lab Agent adds SLAM & navigation pipelines
- VLA chapters → Lab Agent adds voice-action pipelines
- Capstone → All agents invoked to orchestrate final project

Appendices:
- Research Agent → collects hardware specs
- Chapter Author → writes appendices
- Diagram Agent → hardware diagrams
- Assessment → optional troubleshooting exercises
- Localization → Urdu-ready
- QA → verify appendices
```

## Response snapshot

**Specification v1.0.0 created and validated.**

A comprehensive 16-chapter AI-native textbook specification has been produced covering:

- **4 User Stories** (P1/P2 priorities) with acceptance scenarios for learner journey, editor orchestration, RAG chatbot, and Urdu translation.
- **21 Functional Requirements** spanning content production (9 FR), learner experience (5 FR), content management (3 FR), and quality/reliability (4 FR).
- **15 Measurable Success Criteria** quantifying content completeness (SC-001 to -003), learner engagement (SC-004 to -007), quality & consistency (SC-008 to -010), system reliability (SC-011 to -013), and adoption (SC-014 to -015).
- **10-Step Agent Orchestration Sequence** per chapter: Book Orchestrator → Research → Author → Diagram → Lab (if technical) → Assessment → Personalization → Localization → RAG Indexing → QA.
- **8 Documented Assumptions** covering learner hardware, internet/tooling, agent availability, semantic search, translation quality, ROS 2 stability, tool updates, and capstone scope.
- **Clear Scope**: 16 chapters + appendices, 15-20 labs, multi-agent orchestration, personalization, Urdu translation, RAG chatbot, Docusaurus deployment, and QA validation.
- **Specification Quality Checklist**: All 20 items PASS; 100% quality; ready for planning.

## Outcome

- ✅ Impact: Specification serves as the authoritative product definition for the entire textbook project. Guides all planning, task generation, and chapter development.
- 🧪 Tests: All 20 specification quality checklist items pass. No [NEEDS CLARIFICATION] markers. Zero implementation details leaked. All user stories have independent test scenarios.
- 📁 Files: specs/1-book-curriculum/spec.md (2500+ words), specs/1-book-curriculum/checklists/requirements.md (quality validation)
- 🔁 Next prompts: Run `/sp.plan` to generate architecture and implementation plan; then `/sp.tasks` to create actionable task list for each chapter and agent workflow.
- 🧠 Reflection: Specification successfully abstracts implementation detail while defining measurable, testable outcomes. The 10-agent orchestration pattern is the key differentiator—it codifies the "AI-native" thesis at the production level. Learners will not just read about AI collaboration; they'll see the artifacts of it in every chapter's attribution and lineage.

## Evaluation notes (flywheel)

- Failure modes observed: None. Specification is internally consistent, comprehensive, and testable.
- Graders run and results (PASS/FAIL): **PASS** — Specification Quality Checklist: 20/20 items passed. All sections complete with zero placeholders. User stories map to functional requirements. Success criteria are measurable and technology-agnostic.
- Prompt variant (if applicable): null
- Next experiment: Proceed to planning phase (`/sp.plan`). Key planning focus: agent orchestration workflow, learner personalization algorithm, RAG pipeline architecture, and chapter-by-chapter task breakdown.
