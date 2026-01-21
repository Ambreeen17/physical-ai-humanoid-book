---
id: 2
title: Implementation Plan for Textbook Production
stage: plan
date: 2025-12-31
surface: agent
model: claude-haiku-4-5-20251001
feature: book-curriculum
branch: 1-book-curriculum
user: system
command: /sp.plan
labels: ["plan", "book-curriculum", "agent-orchestration", "architecture", "phase-breakdown"]
links:
  spec: specs/1-book-curriculum/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-book-curriculum/plan.md
  - specs/1-book-curriculum/quickstart.md
  - specs/1-book-curriculum/contracts/learner-profile-api.md
  - specs/1-book-curriculum/contracts/rag-chatbot-api.md
  - specs/1-book-curriculum/contracts/assessment-api.md
  - history/prompts/book-curriculum/2-implementation-plan.plan.prompt.md
tests:
  - Constitution Check: All 5 principles aligned (PASS)
  - All 3 phases documented with clear deliverables
  - 10-agent orchestration sequence defined per chapter
  - Phase 1 parallel execution model (4 chapters at a time)
  - API contracts specified for learner profile, RAG chatbot, assessment submission
  - Data model defined with core entities and relationships
  - Project structure clear (backend FastAPI + frontend Docusaurus)
  - Timeline: 30 weeks total (12-16 weeks with parallelization)
  - Success metrics aligned to spec (95% lab success, 85% RAG accuracy, etc.)
  - Risks identified with mitigations (ROS 2 API changes, Isaac Sim availability, etc.)
---

## Prompt

```
/sp.plan

Project Name:
Physical AI & Humanoid Robotics — AI-Native, Spec-Driven Book

Objective:
Plan the complete execution flow of the book using reusable agents and specialized subagents, transforming the constitution into a staged, executable roadmap.

---

## 🧠 GLOBAL AGENT INVOCATION

Invoke the following reusable agents globally for this plan:

- Book Orchestrator Agent (primary controller)
- Curriculum Analyst Agent
- Research Agent
- Chapter Author Agent
- Robotics Lab Agent
- Diagram & Visualization Agent
- Assessment Agent
- Personalization Agent
- Localization Agent (Urdu)
- RAG Indexing Agent
- QA & Consistency Agent

---

## 📍 PHASE 0: Governance & Alignment

- Invoke Book Orchestrator Agent
  - Read `/sp.constitution`
  - Enforce vision, constraints, and success criteria

- Invoke Curriculum Analyst Agent
  - Validate learning progression (Beginner → Intermediate)
  - Align chapters with hands-on outcomes

Output:
- Approved learning roadmap
- Validated chapter sequencing

---

## 📍 PHASE 1: Chapter Planning (Chapters 1–16)

For EACH chapter:

1. Invoke Research Agent
   - Gather up-to-date Physical AI, Robotics, ROS 2, Simulation, and VLA knowledge
   - Identify required
```

## Response snapshot

**Implementation Plan v1.0.0 created and validated.**

A comprehensive 30-week roadmap (12-16 weeks with parallelization) has been produced covering:

- **Technical Context**: Python 3.9+, ROS 2 Humble, Gazebo + NVIDIA Isaac Sim, FastAPI backend, Docusaurus frontend, Neon Postgres, Qdrant Cloud RAG, Docker containerization
- **Constitution Check**: All 5 principles PASS (Embodied Intelligence First, Simulation-to-Real Continuity, Agent-Driven Knowledge Creation, Hardware-Aware Teaching, Modular & Reusable Design)
- **3-Phase Execution Plan**:
  - Phase 0 (Week 1): Governance & curriculum validation
  - Phase 1 (Weeks 2–20): Content production via 10-agent orchestration (4 chapters parallel)
  - Phase 2 (Weeks 20–24): Learner experience & integration (Docusaurus, RAG, assessments)
  - Phase 3 (Weeks 24–30): QA, beta testing, production launch
- **10-Agent Orchestration** per chapter: Book Orchestrator → Research → Author → Diagram → Lab → Assessment → Personalization → Localization → RAG Indexing → QA
- **Data Model**: 8 core entities (Chapter, Lab, Assessment, LearnerProfile, AssessmentResult, RAGChunk, AgentExecutionRecord, Urdu Translation)
- **API Contracts**: Learner profile detection, RAG chatbot endpoint, assessment submission & validation
- **Project Structure**: Backend (FastAPI + Postgres), Frontend (Docusaurus + React), ROS 2 labs (Docker containers)
- **Timeline**: 30 weeks worst-case; 12-16 weeks with 4-chapter parallel waves
- **Success Metrics**: 95% lab success, 85% RAG accuracy, ≥80% assessment pass rate, 2–4 hour chapters, 500–2000 learner adoption

## Outcome

- ✅ Impact: Plan transforms specification into executable roadmap. Guides all downstream task generation, agent orchestration, and engineering implementation. Ready for Phase 0 kickoff.
- 🧪 Tests: Constitution Check passes all 5 principles. All 3 phases have clear deliverables. 10-agent sequence is unambiguous. API contracts are detailed. Data model supports all use cases (learner profiles, assessments, RAG, orchestration tracking).
- 📁 Files: specs/1-book-curriculum/plan.md (6000+ words), specs/1-book-curriculum/quickstart.md (setup guide), API contract files
- 🔁 Next prompts: `/sp.tasks` to generate per-chapter task lists, agent workflows, and integration checkpoints. Book Orchestrator Agent ready to begin Phase 0 (governance validation).
- 🧠 Reflection: Plan is architecture-first, agent-centric, and constitution-aligned. The 10-agent sequence per chapter is the core innovation; it transforms content production into an orchestrated, transparent, reusable process. This design scales to 16+ chapters and future books.

## Evaluation notes (flywheel)

- Failure modes observed: None. Plan is internally consistent, comprehensive, and operationalized. Technical context is grounded in 2025 ROS 2 / Isaac Sim best practices.
- Graders run and results (PASS/FAIL): **PASS** — Constitution Check: 5/5 principles aligned. Phase breakdown: Clear deliverables per phase. Agent orchestration: 10-step sequence unambiguous. Data model: All spec requirements mapped to entities. API contracts: Sufficient detail for implementation. Timeline: Realistic with documented parallelization.
- Prompt variant (if applicable): null
- Next experiment: Invoke Book Orchestrator for Phase 0 (governance validation); then begin Phase 1 with Chapters 1, 2, 15, 16 in parallel.
