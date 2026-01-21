---
id: 3
title: Executable Tasks for Textbook Production Pipeline
stage: tasks
date: 2025-12-31
surface: agent
model: claude-haiku-4-5-20251001
feature: book-curriculum
branch: 1-book-curriculum
user: system
command: /sp.tasks
labels: ["tasks", "book-curriculum", "agent-orchestration", "phases", "execution-ready"]
links:
  spec: specs/1-book-curriculum/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-book-curriculum/tasks.md
  - history/prompts/book-curriculum/3-task-breakdown.tasks.prompt.md
tests:
  - All 120 tasks organized into 12 phases
  - MVP scope clearly defined (Phase 1–3, 5 weeks)
  - Full scope clearly defined (All phases, 30 weeks)
  - Each task follows checklist format: [ID] [P?] [Story?] Description + file path
  - Dependencies explicitly documented (phase → phase, task → task)
  - Parallel execution opportunities identified and grouped
  - User Stories 1–4 independently testable per spec
  - Agent invocations explicit (which agents, inputs, outputs)
  - File paths concrete and consistent
  - Success metrics mapped to tasks
---

## Prompt

```
/sp.tasks

Project:
Physical AI & Humanoid Robotics — Spec-Driven, AI-Native Book

Goal:
Convert the approved /sp.plan into executable, agent-driven tasks that generate the complete book, labs, diagrams, assessments, RAG chatbot, personalization, and localization.

---

## 🧠 GLOBAL AGENT REGISTRATION (Reusable)

Register and invoke these agents across all tasks:

- Book Orchestrator Agent (controller)
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

[Full user input: See tasks.md for complete breakdown]
```

## Response snapshot

**Task Breakdown v1.0.0 created and validated.**

A comprehensive, 120-task breakdown has been produced covering the complete textbook production pipeline:

- **12 Phases**: Setup → Foundational → User Stories 1–4 → Parallel Chapters → Capstone → RAG Indexing → QA → Deployment → Maintenance
- **120 Tasks**: All tasks follow strict checklist format [ID] [P?] [Story?] Description + file path
- **4 User Stories** from spec mapped to phases:
  - US1: Learner completes Chapter 1 with adaptive content (Phase 3, 16 tasks)
  - US2: Editor orchestrates multi-agent Chapter 2 production (Phase 4, 13 tasks)
  - US3: Learner queries book via RAG chatbot (Phase 5, 7 tasks)
  - US4: Translator converts chapter to Urdu (Phase 6, 5 tasks)
- **Parallel Chapters** (Phase 7): 4 chapters at a time across 5 waves, 17 tasks to coordinate orchestration
- **Capstone, RAG, QA, Deployment, Maintenance**: Full phases with explicit tasks
- **MVP Scope**: Phases 1–3 (35 tasks, 5 weeks) → Chapter 1 deployable on Docusaurus
- **Full Scope**: All 120 tasks, 30 weeks (12–16 weeks with parallelization)
- **Dependencies**: Phase dependencies documented; task-level dependencies within phases (e.g., T046 depends on T045 via Book Orchestrator sequencing)
- **Parallel Opportunities**: Identified [P] tasks that can run in parallel; team strategy provided (1-person serial = 30 weeks; 5-person parallel = 12–14 weeks)
- **Agent Invocations**: Explicit for each task (e.g., T028 invokes Research Agent, outputs to `research.md`)
- **Success Metrics**: 12 KPIs mapped to task completion

## Outcome

- ✅ Impact: Task breakdown is immediately executable. Each task is specific enough for an agent or developer to complete without additional context. Ready for `/sp.implement` or direct agent invocation.
- 🧪 Tests: 120 tasks validated against format rules. All tasks have file paths. All phases have clear acceptance criteria. MVP scope is 5 weeks. Full scope is 30 weeks worst-case.
- 📁 Files: specs/1-book-curriculum/tasks.md (8000+ words), execution-ready
- 🔁 Next prompts: `/sp.implement` to begin Phase 1 setup tasks; or invoke Book Orchestrator Agent to start Phase 0 governance (optional, governance was already approved in Phase 0 of planning).
- 🧠 Reflection: Task breakdown translates abstract agent roles (Research, Author, Lab, etc.) into concrete deliverables with file paths. The 10-agent sequence is operational; Book Orchestrator controls sequencing; each agent knows its inputs/outputs. The parallel chapter strategy (4 at a time × 5 waves) is realistic for a lean team and demonstrates the reusability and scalability of the agent system. MVP (Chapter 1 alone) is achievable in 5 weeks; this proves the system before scaling to 16 chapters.

## Evaluation notes (flywheel)

- Failure modes observed: None. Task list is comprehensive, specific, and actionable. Dependencies are clear. Parallel opportunities are identified with team strategies.
- Graders run and results (PASS/FAIL): **PASS** — Format validation: All 120 tasks follow [ID] [P?] [Story?] format. File paths: All concrete and consistent. Dependencies: Clear phase → phase and task → task ordering. Parallel execution: Identified and documented. MVP scope: 35 tasks in Phase 1–3 delivers Chapter 1 complete in 5 weeks. Full scope: All 12 phases documented with realistic timelines.
- Prompt variant (if applicable): null
- Next experiment: Execute Phase 1 (Setup tasks T001–T010) immediately. These are infrastructure tasks and unblock all downstream work. Once Phase 2 (Foundational) is done, can begin Chapter 1 production (Phase 3) and see real agent execution + output artifacts.
