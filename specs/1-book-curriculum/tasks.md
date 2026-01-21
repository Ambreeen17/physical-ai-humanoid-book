---
description: "Task list for AI-native Physical AI & Humanoid Robotics textbook"
---

# Tasks: AI-Native Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/1-book-curriculum/`
**Prerequisites**: plan.md ✅, spec.md ✅, constitution.md ✅
**Status**: Ready for Phase 0 Execution
**Agent System**: 10 reusable agents (Book Orchestrator, Research, Author, Lab, Diagram, Assessment, Personalization, Localization, RAG Indexing, QA)

---

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different agents, no dependencies)
- **[Story]**: Which user story (US1, US2, US3, US4 from spec.md)
- Include exact file paths and agent names

---

## Phase 1: Setup & Infrastructure

**Purpose**: Project initialization, repository structure, deployment infrastructure

**Duration**: 1 week

- [ ] T001 Initialize GitHub repository with branch protection, CI/CD pipeline (GitHub Actions)
- [ ] T002 Create directory structure: `/chapters`, `/backend`, `/website`, `/labs` per plan.md
- [ ] T003 [P] Set up Docker infrastructure: Dockerfile for FastAPI, ROS 2 lab base images
- [ ] T004 [P] Configure Neon Postgres: Create `textbook_db` with connection pooling
- [ ] T005 [P] Provision Qdrant Cloud instance and test connection
- [ ] T006 Initialize Docusaurus project in `website/` directory
- [ ] T007 [P] Create GitHub Actions workflow for lab testing (Ubuntu 22.04 ROS 2 Humble)
- [ ] T008 [P] Set up monitoring: Datadog/CloudWatch for FastAPI, Docusaurus metrics
- [ ] T009 Create `.env.example` with all required environment variables (OpenAI, Qdrant, Database keys)
- [ ] T010 Initialize backend FastAPI project structure: `backend/src/main.py`, `config.py`, `requirements.txt`

**Checkpoint**: Project structure ready, infrastructure provisioned, CI/CD pipeline active

---

## Phase 2: Foundational Infrastructure (Agent & Data Layer)

**Purpose**: Core services, data models, and agent orchestration framework that ALL chapters depend on

**⚠️ CRITICAL**: This phase MUST complete before ANY chapter work begins

**Duration**: 1–2 weeks

### Data Model Foundation

- [ ] T011 Create Postgres schema: `chapters`, `labs`, `assessments`, `learner_profiles`, `assessment_results` in `backend/src/db/schema.sql`
- [ ] T012 [P] Implement SQLAlchemy ORM models in `backend/src/models/`:
  - Chapter model (`chapter.py`)
  - Lab model (`lab.py`)
  - Assessment model (`assessment.py`)
  - LearnerProfile model (`learner_profile.py`)
  - AssessmentResult model (`assessment_result.py`)
  - RAGChunk model (`rag_chunk.py`)
  - AgentExecutionRecord model (`agent_execution_record.py`)
- [ ] T013 [P] Create database migrations framework (Alembic) in `backend/src/db/migrations/`
- [ ] T014 Implement database session management in `backend/src/db/session.py`

### Agent Orchestration Framework

- [ ] T015 Create Book Orchestrator Agent controller in `backend/src/agents/orchestrator.py`
  - Task queuing and sequencing
  - Agent invocation logic
  - Artifact linking (research → author → lab, etc.)
  - Failure handling and logging
  - Input: chapter_id, chapter_outline
  - Output: orchestration_record with all agent task statuses
- [ ] T016 [P] Create agent registry in `backend/src/agents/registry.py`:
  - Research Agent interface
  - Chapter Author Agent interface
  - Robotics Lab Agent interface
  - Diagram Agent interface
  - Assessment Agent interface
  - Personalization Agent interface
  - Localization Agent interface
  - RAG Indexing Agent interface
  - QA Agent interface
  - Storage: AgentExecutionRecord with status, duration, input/output artifacts
- [ ] T017 Implement agent communication protocol in `backend/src/agents/protocol.py`
  - Artifact passing between agents
  - Error reporting
  - Logging of all agent execution
- [ ] T018 Create chapter artifact directory generator in `backend/src/services/artifact_service.py`:
  - Creates `specs/1-book-curriculum/chapters/chapter-N/` structure
  - Initializes files: research.md, chapter-draft.md, diagrams.md, lab/, assessments.md, personalization.md, chapter-urdu.md, rag-manifest.json, qa-report.md

### API & Service Foundation

- [ ] T019 Implement LLM service in `backend/src/services/llm_service.py`:
  - Claude API calls (chapter drafting, synthesis, etc.)
  - OpenAI API calls (embeddings for RAG)
  - Error handling and retries
  - Logging of API usage
- [ ] T020 Implement Qdrant service in `backend/src/services/qdrant_service.py`:
  - Upsert chunks with embeddings
  - Similarity search
  - Metadata filtering
  - Connection pooling
- [ ] T021 Implement assessment grading service in `backend/src/services/assessment_service.py`:
  - Quiz answer validation
  - Lab output pattern matching
  - Score calculation
  - Feedback generation
- [ ] T022 [P] Set up error handling middleware in `backend/src/middleware/error_handler.py`
- [ ] T023 [P] Set up logging infrastructure in `backend/src/logging/logger.py` (structured JSON logs)
- [ ] T024 Create health check endpoint in `backend/src/api/health.py`

### Utilities & Helpers

- [ ] T025 Create chapter outline parser in `backend/src/utils/outline_parser.py`
- [ ] T026 Create artifact file manager in `backend/src/utils/file_manager.py`
- [ ] T027 [P] Create environment configuration manager in `backend/src/config.py`

**Checkpoint**: Agent orchestration framework ready, data models defined, services stubbed, database schema deployed

---

## Phase 3: User Story 1 - Learner Completes Chapter with Adaptive Content (Priority: P1) 🎯 MVP

**Goal**: Enable a learner to access Chapter 1, have content personalize to their background, complete theory + labs + assessment, and pass with feedback

**Independent Test**: Single learner (software engineer profile) can access Chapter 1 in Docusaurus, read personalized content, run a lab in 30 min, take quiz, and pass with detailed feedback

**Duration**: 2–3 weeks (Chapter 1 only for MVP)

### Chapter 1 Production (Invoke All Agents Sequentially)

- [ ] T028 [US1] Invoke Research Agent for Chapter 1: "Introduction to Physical AI"
  - Output: `specs/1-book-curriculum/chapters/chapter-1/research.md`
  - Gather: Embodied AI definitions, current robotics landscape, humanoid platforms, simulation tools, ROS 2 basics, VLA models (2025 state-of-art)
  - References: 10+ recent papers, tool docs, hardware specs
  - Record AgentExecutionRecord: status=success, duration, output artifacts
  - File: `backend/src/agents/research_agent.py` calls Claude API with chapter topic

- [ ] T029 [US1] Invoke Chapter Author Agent for Chapter 1 (depends on T028)
  - Input: research.md from T028
  - Output: `specs/1-book-curriculum/chapters/chapter-1/chapter-draft.md`
  - Write: Learning objectives (3–5), core theory sections (2000+ words), real-world examples, embedded code snippets, key concepts summary
  - Cite: Research sources explicitly with [research.md] links
  - Record AgentExecutionRecord
  - File: `backend/src/agents/chapter_author_agent.py` calls Claude API

- [ ] T030 [US1] Invoke Diagram Agent for Chapter 1 (depends on T029)
  - Input: chapter-draft.md from T029
  - Output: `specs/1-book-curriculum/chapters/chapter-1/diagrams.md`
  - Specify: 3–4 diagrams needed (ASCII, Mermaid, descriptions)
    - Example: "AI-brain, robot-body feedback loop" diagram
    - Example: "Embodied intelligence vs pure software AI" comparison table
  - Include placement anchors (e.g., "Insert Diagram A after section 1.2")
  - Record AgentExecutionRecord
  - File: `backend/src/agents/diagram_agent.py` outputs structured diagram specs

- [ ] T031 [US1] Invoke Robotics Lab Agent for Chapter 1 (depends on T029)
  - Input: chapter-draft.md
  - Output: `specs/1-book-curriculum/chapters/chapter-1/lab/`
  - Create: "Hello Physical AI" lab (introductory, 30 min)
    - ROS 2 node that publishes "Hello" message to a topic
    - Gazebo world with a simple sphere robot
    - Expected output: "Hello message received on /hello_topic"
  - Include: `docker-compose.yml`, `src/hello_node.py`, `launch/hello.launch.py`, `worlds/empty_world.sdf`, `README.md`, `expected-output.txt`
  - Record AgentExecutionRecord
  - File: `backend/src/agents/robotics_lab_agent.py` generates lab code

- [ ] T032 [US1] Invoke Assessment Agent for Chapter 1 (depends on T029)
  - Input: chapter-draft.md, lab output
  - Output: `specs/1-book-curriculum/chapters/chapter-1/assessments.md`
  - Create: 3 multiple-choice questions + 1 lab exercise
    - Q1: "What is embodied intelligence?" (3 options)
    - Q2: "Why is simulation important?" (3 options)
    - Q3: "How does feedback improve robot control?" (3 options)
    - Lab: "Modify the hello_node to count messages and publish count"
  - Include: Answer key, rubric, passing score (70%)
  - Record AgentExecutionRecord
  - File: `backend/src/agents/assessment_agent.py`

- [ ] T033 [US1] Invoke Personalization Agent for Chapter 1 (depends on T029)
  - Input: chapter-draft.md
  - Output: `specs/1-book-curriculum/chapters/chapter-1/personalization.md`
  - Create: Beginner notes & intermediate notes
    - Beginner: "Embodied means the robot feels and learns from real-world friction"
    - Intermediate: "Contrast with purely learning-based approaches; cite papers on sim-to-real gap"
  - Tag sections: `[BEGINNER: ...]` and `[INTERMEDIATE: ...]`
  - Record AgentExecutionRecord
  - File: `backend/src/agents/personalization_agent.py`

- [ ] T034 [US1] Invoke Localization Agent for Chapter 1 (depends on T029)
  - Input: chapter-draft.md
  - Output: `specs/1-book-curriculum/chapters/chapter-1/chapter-urdu.md`
  - Translate: All explanatory text to Urdu, preserve code, preserve technical terms (e.g., `/hello_topic` stays English)
  - Format: Bilingual side-by-side (English | اردو)
  - Record AgentExecutionRecord
  - File: `backend/src/agents/localization_agent.py` calls Claude API for translation

- [ ] T035 [US1] Invoke RAG Indexing Agent for Chapter 1 (depends on T029, T030, T031, T032, T033, T034)
  - Input: chapter-draft.md, diagrams.md, lab/README.md, assessments.md, personalization.md
  - Output: `specs/1-book-curriculum/chapters/chapter-1/rag-manifest.json`
  - Process:
    1. Chunk chapter into semantic segments (paragraphs/sections), ~200 tokens each
    2. Generate embeddings for each chunk via OpenAI API
    3. Upsert to Qdrant Cloud with metadata: {chapter_id: 1, section: "1.2", position: 0, type: "theory"}
    4. Create manifest JSON listing all chunks with IDs
  - Record AgentExecutionRecord
  - File: `backend/src/agents/rag_indexing_agent.py`

- [ ] T036 [US1] Invoke QA & Consistency Agent for Chapter 1 (depends on T035)
  - Input: All chapter artifacts from T028–T035
  - Output: `specs/1-book-curriculum/chapters/chapter-1/qa-report.md`
  - Validate:
    - Terminology: consistent use of "embodied intelligence", "simulation-to-real", etc.
    - Cross-references: All [research.md] links exist and are correct
    - Labs: Dry-run on Ubuntu 22.04 ROS 2 Humble (or Docker simulation)
    - Diagrams: All specified diagrams have implementations
    - Assessments: Answer key is complete, rubric is clear
    - Personalization: All beginner/intermediate tags are properly paired
    - Urdu: No code blocks in translation, technical terms preserved
  - Record: Pass/fail status, issues found, mitigation steps
  - Record AgentExecutionRecord
  - File: `backend/src/agents/qa_agent.py`

### Learner Profile & Personalization

- [ ] T037 [US1] Create onboarding quiz API endpoint in `backend/src/api/v1/learners.py`
  - POST `/api/v1/learners/onboarding`
  - Input: 5 questions (software years, robotics exp, AI exp, preferred language, learning pace)
  - Output: learner_id, background_scores, recommended_difficulty
  - Store in LearnerProfile table
  - File: `backend/src/api/v1/learners.py`

- [ ] T038 [US1] Implement learner profile detection logic in `backend/src/services/profile_service.py`
  - Calculate software, robotics, AI background scores (0–10)
  - Determine: beginner vs intermediate personalization
  - Return: personalization_settings JSON

### Docusaurus Chapter Content

- [ ] T039 [US1] Create Chapter 1 Docusaurus page in `website/docs/chapter-1/index.md`
  - Merge: chapter-draft.md + diagrams.md + assessments.md sections
  - Include: Interactive code blocks (lab starter code)
  - Include: Embedded assessment widget (quiz + lab submission)
  - Include: Urdu language toggle button
  - File structure: `website/docs/chapter-1/index.md`, `website/docs/chapter-1/lab.md`

- [ ] T040 [US1] Update Docusaurus sidebar in `website/sidebars.js` to include Chapter 1

### Assessment Endpoint

- [ ] T041 [US1] Create assessment submission endpoint in `backend/src/api/v1/assessments.py`
  - POST `/api/v1/assessments/chapter-1/submit`
  - Input: learner_id, chapter_id, answers (quiz + lab output)
  - Validate: Quiz answers, lab output pattern matching
  - Output: score, feedback per question, pass/fail
  - Store AssessmentResult
  - File: `backend/src/api/v1/assessments.py`

- [ ] T042 [US1] Implement lab output validator in `backend/src/services/lab_validator.py`
  - For Chapter 1 lab: Check if output contains "Hello message received on /hello_topic"
  - Extensible for other labs (pattern-based validation)

### Learner Journey Integration

- [ ] T043 [US1] Create learner dashboard page in `website/src/pages/dashboard.jsx`
  - Show: Completed chapters, current chapter progress, assessment scores, preferred language
  - File: `website/src/pages/dashboard.jsx`

- [ ] T044 [US1] Integrate personalization in Docusaurus rendering
  - Load learner profile from backend API
  - Show/hide [BEGINNER:...] or [INTERMEDIATE:...] content sections
  - Store preference in localStorage
  - File: `website/src/components/PersonalizationToggle.jsx`

**Checkpoint**: Chapter 1 fully produced by agents, Docusaurus site renders, learner can complete chapter, assessments work, pass with feedback

---

## Phase 4: User Story 2 - Editor Orchestrates Multi-Agent Chapter Production (Priority: P1) 🎯

**Goal**: Validate that the Book Orchestrator Agent can sequence all 9 agents for Chapter 2, producing publication-ready output automatically

**Independent Test**: Editor invokes Book Orchestrator for Chapter 2, orchestrator completes all 9 agents within 4 hours, QA report shows zero critical issues, chapter is deployable

**Duration**: 2–3 weeks (Chapter 2 full pipeline validation)

**Note**: Chapter 2 is "Sensors & Perception" — includes a Robotics Lab with RealSense simulation + LIDAR

### Chapter 2 Production (Book Orchestrator Controls All Agents)

- [ ] T045 [US2] Implement Book Orchestrator chapter scheduling in `backend/src/agents/orchestrator.py`
  - Input: chapter_id=2, chapter_topic="Sensors & Perception", chapter_outline (from spec)
  - Sequence: T046 → T047 → T048 → T049 → T050 → T051 → T052 → T053 → T054
  - Track: Start time, each agent start/end, total duration
  - Output: Orchestration record with all agent results
  - File: `backend/src/agents/orchestrator.py` (chapter sequencing logic)

- [ ] T046 [US2] Invoke Research Agent for Chapter 2
  - Topic: "Sensors & Perception"
  - Output: `specs/1-book-curriculum/chapters/chapter-2/research.md`
  - Gather: Sensor types (vision, IMU, LIDAR, tactile), data fusion, ROS 2 message types, Gazebo sensor simulation, RealSense API, LIDAR processing libraries (PCL, Open3D)
  - File: Orchestrator calls `backend/src/agents/research_agent.py`

- [ ] T047 [US2] Invoke Chapter Author Agent for Chapter 2 (depends on T046)
  - Output: `specs/1-book-curriculum/chapters/chapter-2/chapter-draft.md`
  - Content: Sensor fundamentals, calibration, data fusion basics, ROS 2 sensor topics, Gazebo sensor plugins
  - Length: 3000+ words with code examples
  - File: Orchestrator calls `backend/src/agents/chapter_author_agent.py`

- [ ] T048 [US2] Invoke Diagram Agent for Chapter 2 (depends on T047)
  - Output: `specs/1-book-curriculum/chapters/chapter-2/diagrams.md`
  - Diagrams: Sensor architecture, data flow (sensor → ROS topic → processing node), fusion pipeline
  - File: Orchestrator calls `backend/src/agents/diagram_agent.py`

- [ ] T049 [US2] Invoke Robotics Lab Agent for Chapter 2 (depends on T047)
  - Output: `specs/1-book-curriculum/chapters/chapter-2/lab/`
  - Labs (2 options):
    1. RealSense simulation lab (RGB-D camera in Gazebo)
    2. LIDAR lab (point cloud processing with PCL)
  - Each lab: Docker setup, ROS 2 nodes, expected output
  - File: Orchestrator calls `backend/src/agents/robotics_lab_agent.py`

- [ ] T050 [US2] Invoke Assessment Agent for Chapter 2 (depends on T047)
  - Output: `specs/1-book-curriculum/chapters/chapter-2/assessments.md`
  - 3 MCQs + 1 lab exercise (choose RealSense OR LIDAR)
  - File: Orchestrator calls `backend/src/agents/assessment_agent.py`

- [ ] T051 [US2] Invoke Personalization Agent for Chapter 2 (depends on T047)
  - Output: `specs/1-book-curriculum/chapters/chapter-2/personalization.md`
  - Beginner: "A camera is like your robot's eyes"
  - Intermediate: "Sensor fusion combines multiple streams; trade-offs in latency vs accuracy"
  - File: Orchestrator calls `backend/src/agents/personalization_agent.py`

- [ ] T052 [US2] Invoke Localization Agent for Chapter 2 (depends on T047)
  - Output: `specs/1-book-curriculum/chapters/chapter-2/chapter-urdu.md`
  - Bilingual translation, preserve code/diagrams
  - File: Orchestrator calls `backend/src/agents/localization_agent.py`

- [ ] T053 [US2] Invoke RAG Indexing Agent for Chapter 2 (depends on T047–T052)
  - Output: `specs/1-book-curriculum/chapters/chapter-2/rag-manifest.json`
  - Chunk, embed, upsert to Qdrant
  - File: Orchestrator calls `backend/src/agents/rag_indexing_agent.py`

- [ ] T054 [US2] Invoke QA Agent for Chapter 2 (depends on T053)
  - Output: `specs/1-book-curriculum/chapters/chapter-2/qa-report.md`
  - Validate all artifacts, dry-run labs
  - Sign-off: publication_ready=true/false
  - File: Orchestrator calls `backend/src/agents/qa_agent.py`

### Orchestrator Validation

- [ ] T055 [US2] Validate orchestration workflow in `backend/src/services/orchestration_service.py`
  - Check: All agents ran in correct order
  - Check: Artifacts properly linked (research → author, etc.)
  - Check: Total duration < 4 hours
  - Log: AgentExecutionRecords for all agents
  - Output: Orchestration report with status

- [ ] T056 [US2] Implement error recovery in Book Orchestrator
  - If agent fails: Log error, notify editor, offer manual override or re-run
  - Re-run: Skip completed agents, resume from failure point
  - File: `backend/src/agents/orchestrator.py` (error handling)

### Chapter 2 Publishing

- [ ] T057 [US2] Create Chapter 2 Docusaurus page in `website/docs/chapter-2/index.md`
  - Merge artifacts, include assessment widget, lab instructions
  - File: `website/docs/chapter-2/index.md`

- [ ] T058 [US2] Update sidebar to include Chapter 2 in `website/sidebars.js`

**Checkpoint**: Chapter 2 auto-produced by orchestrator, validation passed, chapter published, editor can confirm workflow

---

## Phase 5: User Story 3 - Learner Queries Book Using RAG Chatbot (Priority: P2)

**Goal**: Implement RAG chatbot that retrieves relevant chapter content and synthesizes answers with citations

**Independent Test**: Learner submits query "How do topics differ from services in ROS 2?", chatbot returns relevant section from Chapter 3 with link

**Duration**: 1–2 weeks

### RAG Chatbot Backend

- [ ] T059 [US3] Create RAG chat endpoint in `backend/src/api/v1/chat.py`
  - POST `/api/v1/chat`
  - Input: query, optional chapter_scope, learner_id
  - Output: answer, citations (section + link), confidence
  - File: `backend/src/api/v1/chat.py`

- [ ] T060 [US3] Implement semantic search in `backend/src/services/rag_service.py`
  - Embed query via OpenAI
  - Search Qdrant for top-5 similar chunks
  - Filter by chapter_scope if provided
  - Return: chunks with metadata and similarity score
  - File: `backend/src/services/rag_service.py`

- [ ] T061 [US3] Implement answer synthesis in `backend/src/services/synthesis_service.py`
  - Input: retrieved chunks
  - Call Claude API: "Synthesize answer from chunks, cite sources"
  - Output: Natural language answer with citations
  - File: `backend/src/services/synthesis_service.py`

- [ ] T062 [US3] Create citation linking in `backend/src/services/citation_service.py`
  - Map chunk metadata (chapter_id, section, position) → Docusaurus URL
  - Output: Clickable links like `/chapters/3#section-3-2`
  - File: `backend/src/services/citation_service.py`

### Docusaurus Chatbot Widget

- [ ] T063 [US3] Create RAG chatbot React component in `website/src/components/ChatBot.jsx`
  - Chat UI: Input field, submit button, message history
  - Display: Chatbot response, citations as clickable links
  - File: `website/src/components/ChatBot.jsx`

- [ ] T064 [US3] Integrate chatbot widget into chapter pages
  - Add ChatBot component to each chapter (`website/docs/chapter-*/index.md`)
  - Scope: Default to current chapter, option for book-wide
  - File: `website/src/components/ChatBot.jsx` integration in sidebars

### RAG Validation & Testing

- [ ] T065 [US3] Create RAG test queries in `backend/tests/integration/test_rag.py`
  - 10+ test queries covering Chapter 1–2 content
  - Expected answers should appear in top-3 results
  - Acceptance: ≥8 queries return relevant results
  - File: `backend/tests/integration/test_rag.py`

**Checkpoint**: RAG chatbot operational, queries return relevant results with citations, learners can navigate to source sections

---

## Phase 6: User Story 4 - Translator Converts Chapter to Urdu (Priority: P2)

**Goal**: One-click Urdu translation of chapters, bilingual rendering in Docusaurus

**Independent Test**: Editor clicks "Translate to Urdu" on Chapter 1, system produces chapter-urdu.md, Docusaurus renders bilingual version correctly

**Duration**: 1 week

### Urdu Translation Integration

- [ ] T066 [US4] Create translation trigger endpoint in `backend/src/api/v1/translations.py`
  - POST `/api/v1/translations/chapter/urdu`
  - Input: chapter_id
  - Output: Invokes Localization Agent (if not already done), returns chapter-urdu.md path
  - File: `backend/src/api/v1/translations.py`

- [ ] T067 [US4] Implement Urdu language toggle in Docusaurus
  - Add language dropdown in chapter header
  - Store preference in localStorage
  - Switch between chapter.md and chapter-urdu.md
  - File: `website/src/components/LanguageToggle.jsx`

- [ ] T068 [US4] Configure Docusaurus for RTL (Right-to-Left) Urdu rendering
  - CSS: `direction: rtl` for Urdu content
  - Fonts: Include Urdu-compatible fonts (e.g., Arial Unicode, Scheherazade)
  - File: `website/docusaurus.config.js`, `website/src/styles/custom.css`

- [ ] T069 [US4] Create Urdu translation QA checks in `backend/src/services/qa_service.py`
  - Verify: No code blocks in Urdu text (should be English)
  - Verify: Technical terms (ROS 2, `/cmd_vel`, etc.) remain English
  - Verify: Urdu grammar is correct (basic spell check)
  - File: `backend/src/services/qa_service.py` (urdu_validation function)

### Urdu Content Management

- [ ] T070 [US4] Create Urdu glossary in `website/docs/appendices/glossary-urdu.md`
  - Map technical terms (ROS 2 topics, robot parts) → Urdu + English
  - Reference: Used by Localization Agent for consistency
  - File: `website/docs/appendices/glossary-urdu.md`

**Checkpoint**: Chapter 1–2 available in Urdu, language toggle works, bilingual rendering correct

---

## Phase 7: Parallel Chapter Production (Chapters 3–16)

**Purpose**: Scale Chapter 1 & 2 pipeline to all remaining chapters using agent orchestration

**Duration**: 12–16 weeks (4 chapters at a time, 5 waves)

### Wave 1: Chapters 3, 4, 5, 6 (Parallel)

- [ ] T071 [P] [US1/US2] Invoke Book Orchestrator for Chapter 3: "ROS 2 Fundamentals"
  - Orchestrator sequences all 9 agents
  - Output: Full chapter package (research, draft, labs, diagrams, assessments, personalization, translation, RAG, QA)
  - File: Orchestrator uses `backend/src/agents/orchestrator.py`

- [ ] T072 [P] [US1/US2] Invoke Book Orchestrator for Chapter 4: "Gazebo Simulation & Modeling"
  - Same as T071, different topic
  - Includes: Gazebo world building lab, physics simulation

- [ ] T073 [P] [US1/US2] Invoke Book Orchestrator for Chapter 5: "ROS 2 Services & Actions"
  - Same pipeline, different topic
  - Includes: ROS 2 service/action labs

- [ ] T074 [P] [US1/US2] Invoke Book Orchestrator for Chapter 6: "Perception Pipelines"
  - Includes: Vision, depth, sensor fusion labs

### Wave 2: Chapters 7, 8, 9, 10 (Parallel)

- [ ] T075 [P] [US1/US2] Invoke Book Orchestrator for Chapter 7: "Control Systems Basics"
- [ ] T076 [P] [US1/US2] Invoke Book Orchestrator for Chapter 8: "Trajectory Planning"
- [ ] T077 [P] [US1/US2] Invoke Book Orchestrator for Chapter 9: "Manipulator Kinematics & Control"
- [ ] T078 [P] [US1/US2] Invoke Book Orchestrator for Chapter 10: "Mobile Robot Navigation"

### Wave 3: Chapters 11, 12, 13, 14 (Parallel)

- [ ] T079 [P] [US1/US2] Invoke Book Orchestrator for Chapter 11: "SLAM & Mapping"
- [ ] T080 [P] [US1/US2] Invoke Book Orchestrator for Chapter 12: "Vision-Language-Action Models"
- [ ] T081 [P] [US1/US2] Invoke Book Orchestrator for Chapter 13: "Humanoid Robot Dynamics"
- [ ] T082 [P] [US1/US2] Invoke Book Orchestrator for Chapter 14: "Whole-Body Control"

### Wave 4: Chapters 15, 16 + Appendices (Parallel)

- [ ] T083 [P] [US1/US2] Invoke Book Orchestrator for Chapter 15: "Integration: Perception → Planning → Action"
- [ ] T084 [P] [US1/US2] Invoke Book Orchestrator for Chapter 16: "Capstone: Autonomous Humanoid"
  - Note: All agents invoked (capstone integrates all topics)
- [ ] T085 [P] [US1/US2] Invoke Research & Chapter Author for Appendices:
  - Appendix A: Hardware Specs & Datasheets
  - Appendix B: Glossary (technical terms)
  - Appendix C: Troubleshooting Guide
  - Appendix D: Tool Installation Guide

### Integration & Publishing

- [ ] T086 Update Docusaurus sidebar in `website/sidebars.js` to include all 16 chapters + appendices
- [ ] T087 [P] Create Docusaurus pages for all chapters (T071–T085 artifacts → `website/docs/chapter-*/index.md`)
- [ ] T088 Build Docusaurus static site: `npm run build`
- [ ] T089 Deploy to GitHub Pages: Docusaurus site public

**Checkpoint**: All 16 chapters produced, published on Docusaurus, learners can browse and complete any chapter

---

## Phase 8: Capstone Project Integration (Weeks 20–24)

**Purpose**: Create Capstone project that integrates all prior chapters

**Duration**: 2–3 weeks

### Capstone Chapter Production

- [ ] T090 [US1/US2] Invoke ALL agents for Capstone (Chapter 16 Part 2: "Build & Deploy Autonomous Humanoid")
  - Topic: End-to-end system: Perception (vision + LIDAR) → Planning (navigation + manipulation) → Action (whole-body control) + Voice (VLA)
  - Output: Complete capstone chapter with high-difficulty labs
  - Files: Research, draft, 2–3 capstone labs (Phase 1: simulation; Phase 2: real hardware extension), capstone project rubric, capstone assessments

### Capstone Labs

- [ ] T091 [US1/US2] Create Capstone Lab 1: "Humanoid Perception Stack"
  - Task: Robot perceives environment (vision + depth), detects objects, localizes itself
  - Output: Point cloud visualization, object detection results
  - Duration: 45 min in Gazebo
  - File: `specs/1-book-curriculum/chapters/chapter-16/lab/capstone-lab-1/`

- [ ] T092 [US1/US2] Create Capstone Lab 2: "Planning & Manipulation"
  - Task: Robot plans trajectory, grasps object, places it
  - Output: Collision-free trajectory, grasp success
  - Duration: 60 min in Isaac Sim (or Gazebo fallback)
  - File: `specs/1-book-curriculum/chapters/chapter-16/lab/capstone-lab-2/`

- [ ] T093 [US1/US2] Create Capstone Lab 3: "Voice-Commanded Task Execution"
  - Task: Robot listens to voice command ("Pick up the cup"), executes end-to-end
  - Output: VLA model processes command, control system executes
  - Duration: 60 min in Isaac Sim
  - File: `specs/1-book-curriculum/chapters/chapter-16/lab/capstone-lab-3/`

### Capstone Assessment

- [ ] T094 [US1/US2] Create Capstone Project Rubric
  - Difficulty tiers: Beginner (simulation perception only), Intermediate (add manipulation), Advanced (add voice + real hardware)
  - Evaluation: Code quality, system design, documentation, real-world applicability
  - File: `specs/1-book-curriculum/chapters/chapter-16/capstone-rubric.md`

**Checkpoint**: Capstone chapter complete, integrates all prior learning, learners can build autonomous humanoid system

---

## Phase 9: Full RAG Indexing (Weeks 22–24)

**Purpose**: Index all 16 chapters + appendices for book-wide semantic search

**Duration**: 1 week

- [ ] T095 Run RAG Indexing Agent for all chapters (post-publication)
  - Process: For each chapter (T071–T085 outputs), chunk, embed, upsert to Qdrant
  - Total: 16 chapters × 50+ chunks/chapter = 800+ embeddings
  - Record: rag-manifest.json for each chapter
  - File: `backend/src/agents/rag_indexing_agent.py` (batch mode)

- [ ] T096 Validate RAG index completeness
  - Query test suite: 100+ questions covering all chapters
  - Target: ≥85% return relevant results in top-3
  - File: `backend/tests/integration/test_rag_full_book.py`

- [ ] T097 Create RAG admin dashboard in `backend/src/api/v1/admin/rag.py`
  - Endpoint: GET `/api/v1/admin/rag/stats` → total chunks, chapters indexed, embedding versions
  - Endpoint: POST `/api/v1/admin/rag/reindex-chapter/{id}` → re-index single chapter after updates
  - File: `backend/src/api/v1/admin/rag.py`

**Checkpoint**: Book-wide RAG chatbot fully operational, 85%+ accuracy on test queries

---

## Phase 10: Quality Assurance & Testing (Weeks 24–28)

**Purpose**: Full QA suite, beta learner testing, production readiness

**Duration**: 3–4 weeks

### Lab Testing & Validation

- [ ] T098 [P] Test all labs on Ubuntu 22.04 + ROS 2 Humble
  - For each lab (Chapters 1–16):
    1. Build Docker image
    2. Run lab in container
    3. Verify expected output
    4. Log results
  - Target: ≥95% success rate
  - File: `backend/tests/integration/test_all_labs.sh` (automation)

- [ ] T099 Create lab results dashboard in `backend/src/api/v1/admin/labs.py`
  - Endpoint: GET `/api/v1/admin/labs/status` → all labs, last test date, success rate
  - File: `backend/src/api/v1/admin/labs.py`

### RAG Testing

- [ ] T100 Run comprehensive RAG test suite
  - 100+ queries covering all chapters
  - Expected: ≥85% return relevant results
  - File: `backend/tests/integration/test_rag_full_book.py`

### Learner Flow Testing

- [ ] T101 [P] Create learner journey test scenarios in `backend/tests/integration/test_learner_flow.py`
  - Scenario 1: New learner → onboarding → Chapter 1 → personalization → lab → assessment → pass
  - Scenario 2: Experienced learner → Chapter 5 → skip beginner content → complete quickly
  - Scenario 3: Learner with Urdu preference → see Urdu content → toggle English → see English
  - Scenario 4: Learner uses RAG chatbot → queries → navigates to source → continues reading

### Performance Testing

- [ ] T102 Load testing for Docusaurus site
  - Target: <2 second page load time (globally, CDN-backed)
  - Tools: Lighthouse, WebPageTest
  - File: `backend/tests/performance/test_docusaurus_perf.md`

- [ ] T103 Load testing for FastAPI backend
  - Target: Chatbot response <2 seconds, assessment submission <1 second
  - Tools: Apache JMeter or similar
  - File: `backend/tests/performance/test_api_perf.md`

### Beta Learner Cohort

- [ ] T104 Recruit and onboard beta learner cohort (10–20 learners)
  - Diversity: Mix of software, robotics, AI backgrounds
  - Duration: 4 weeks
  - Tracking: Completion rate, time per chapter, assessment scores, NPS

- [ ] T105 Create learner feedback survey
  - Questions: Clarity (1–5), relevance (1–5), lab difficulty (1–5), overall NPS (0–10)
  - Collect: Post-chapter feedback
  - File: `website/src/components/FeedbackSurvey.jsx`

- [ ] T106 Analyze beta feedback and create iteration backlog
  - Prioritize: Top 10 issues by frequency/severity
  - Assign: Fixes to relevant agents (Chapter Author for clarity, Lab Agent for difficulty, etc.)
  - File: `BETA_FEEDBACK_ANALYSIS.md`

### Iteration Based on Feedback

- [ ] T107 [P] Fix high-priority issues from beta feedback
  - Re-invoke agents for chapters with issues
  - Re-validate with QA Agent
  - Update Docusaurus site

**Checkpoint**: All labs working ≥95%, RAG ≥85% accurate, beta learners show positive NPS, production-ready

---

## Phase 11: Deployment & Launch (Weeks 28–30)

**Purpose**: Deploy to production, monitor, launch publicly

**Duration**: 2 weeks

### Infrastructure Finalization

- [ ] T108 [P] Final infrastructure checks:
  - Docusaurus deployment: GitHub Pages live
  - FastAPI deployment: Cloud hosting (AWS/Google/Render) configured, monitoring active
  - Database: Neon Postgres production instance, backups enabled
  - Qdrant: Cloud instance with failover, backups enabled
  - SSL/TLS: All endpoints HTTPS
  - File: `DEPLOYMENT_CHECKLIST.md`

- [ ] T109 [P] Configure monitoring & alerting
  - Datadog/CloudWatch: API latency, error rates, RAG accuracy, uptime
  - Alerts: Page editor if API down, RAG accuracy drops, high error rates
  - File: `infrastructure/monitoring.yaml`

- [ ] T110 Create runbooks for common operational tasks
  - Runbook: "How to re-index a chapter in RAG"
  - Runbook: "How to fix a broken lab"
  - Runbook: "How to add a new chapter"
  - Runbook: "How to respond to learner feedback"
  - File: `docs/RUNBOOKS.md`

### Launch

- [ ] T111 Announce public launch
  - Blog post: Describe AI-native approach, agent orchestration, reusable architecture
  - Social media: Link to textbook, highlight capstone
  - Email: Beta learners, community partners
  - File: `LAUNCH_ANNOUNCEMENT.md`

- [ ] T112 Create onboarding guide for new learners
  - Quick start: How to access Chapter 1, take quiz, run lab
  - FAQ: Common questions, troubleshooting
  - File: `website/docs/getting-started.md`

- [ ] T113 Set up public feedback mechanism
  - GitHub Discussions: For learners to ask questions, report issues
  - Email: Support contact for critical issues
  - File: Configure GitHub Discussions in repository settings

**Checkpoint**: Textbook publicly live, monitoring active, learners can onboard and complete chapters

---

## Phase 12: Maintenance & Evolution (Weeks 30+)

**Purpose**: Sustain textbook, iterate based on learner feedback, keep content current

**Duration**: Ongoing

### Content Updates

- [ ] T114 [Recurring quarterly] Invoke Research Agent to gather latest Physical AI/Robotics developments
  - Check: New ROS 2 releases, Isaac Sim updates, VLA model improvements
  - Output: Research summary of what's changed
  - File: `UPDATES_Q1_2025.md`

- [ ] T115 [Recurring quarterly] Re-index all chapters in RAG if content changed significantly
  - File: `backend/src/agents/rag_indexing_agent.py` (batch re-index mode)

- [ ] T116 [Recurring] Monitor learner feedback and create chapter update tickets
  - Prioritize: Issues reported by >2 learners or affecting completion
  - Assign: Fixes to relevant agents
  - File: `LEARNER_FEEDBACK_TRACKER.md`

### Lab Maintenance

- [ ] T117 [Recurring per ROS 2 release] Test all labs on new ROS 2 LTS releases
  - ROS 2 Iron (2024 LTS): Test all labs, update if needed
  - ROS 2 Jazzy (2025 LTS): Test all labs, document any breaking changes
  - File: `VERSION_COMPATIBILITY_MATRIX.md`

- [ ] T118 [Recurring] Update Isaac Sim labs as new versions released
  - File: Track Isaac Sim version in each lab's docker-compose.yml

### Scaling

- [ ] T119 Monitor learner adoption and system performance
  - Target: 500+ learners in 6 months, 2000+ in 1 year
  - Track: Docusaurus page views, API request volume, RAG query volume
  - File: `ANALYTICS_DASHBOARD.md`

- [ ] T120 Plan for future books using the same agent-driven approach
  - Design: How to adapt orchestrator for other technical topics (e.g., "Quadruped Locomotion", "Multi-Robot Systems")
  - Document: Reusable patterns from this project
  - File: `FUTURE_BOOKS_ROADMAP.md`

**Checkpoint**: Textbook evolving sustainably, new chapters possible, agent system proven reusable

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    ↓
Phase 2 (Foundational) ← BLOCKS all user stories
    ↓
Phase 3 (US1: Ch1 Production) ← MVP
    ↓
Phase 4 (US2: Orchestrator Validation)
    ↓
Phase 5 (US3: RAG Chatbot) [Can run parallel with Phase 4]
    ↓
Phase 6 (US4: Urdu Translation) [Can run parallel with Phase 5]
    ↓
Phase 7 (Chapters 3–16) [Parallel: 4 chapters at a time]
    ↓
Phase 8 (Capstone Integration)
    ↓
Phase 9 (Full RAG Indexing)
    ↓
Phase 10 (QA & Beta Testing)
    ↓
Phase 11 (Deployment & Launch)
    ↓
Phase 12 (Maintenance & Evolution)
```

### Parallel Execution Opportunities

**Immediately Parallel (After Phase 2)**:
- Phase 3 (US1: Chapter 1)
- Phase 4 (US2: Orchestrator) can start once US1 chapter structure is clear

**After US1 Complete**:
- Phase 5 (US3: RAG) - depends on Chapter 1 being indexed
- Phase 6 (US4: Urdu) - depends on Chapter 1 draft complete

**Parallel Chapters (Phase 7)**:
- Chapters 3, 4, 5, 6 via 4 independent Book Orchestrator invocations
- Then Chapters 7–10, 11–14, 15–16 in subsequent waves

**Team Strategy**:
- Person A: Phase 1–2 setup (blocking work)
- Person B (Week 2): Begin Phase 3 (US1)
- Person C (Week 2): Begin Phase 4 (US2) in parallel
- Persons D, E, F (Week 4+): Each manage one Chapter 3–5 orchestration
- Result: 16 chapters in 12–16 weeks instead of 30 weeks

---

## Implementation Strategy

### MVP First (Recommended)

**Deliver minimum viable product in 4–5 weeks**:

1. **Week 1**: Phase 1 (Setup) + Phase 2 (Foundational)
2. **Weeks 2–4**: Phase 3 (User Story 1: Chapter 1 complete)
3. **Week 5**: Deploy Chapter 1 to Docusaurus, learners can use it
4. **Stop and Validate**: Measure
   - Learner completion rate for Chapter 1
   - Quiz/lab pass rate
   - Learner NPS
5. **Decision**: If positive, proceed to Chapters 2–16; if issues, iterate on Chapter 1

### Incremental Delivery

After MVP validation:

1. **Weeks 6–7**: Phase 4 (US2: Validate orchestrator on Chapter 2)
2. **Weeks 8–9**: Phase 5 (US3: RAG chatbot)
3. **Weeks 10–11**: Phase 6 (US4: Urdu)
4. **Weeks 12–20**: Phase 7 (Chapters 3–16 via parallel orchestration)
5. **Weeks 21–24**: Phase 8–9 (Capstone + full RAG indexing)
6. **Weeks 25–28**: Phase 10 (QA + beta testing + iteration)
7. **Weeks 29–30**: Phase 11 (Launch)

### Resource Allocation

**Minimal team (1–2 people)**:
- Phase 1–2: 1 person (infrastructure setup)
- Phase 3+: 1 person manages orchestrations, monitors agent runs, reviews QA reports
- Total: 30 weeks

**Medium team (3–4 people)**:
- Phase 1–2: 1–2 people (parallel infrastructure)
- Phase 3+: Rotate orchestrator management, 2 people handle iteration/feedback
- Total: 16–20 weeks

**Larger team (5+ people)**:
- Phase 1–2: 2 people (parallel)
- Phase 3–7: 3+ people manage parallel chapters, RAG, Urdu in parallel
- Phase 8–11: 2 people handle QA, launch, monitoring
- Total: 12–14 weeks

---

## Success Metrics

| Metric | Target | Status | Notes |
|--------|--------|--------|-------|
| **Setup Complete** | Phase 1 done in 1 week | — | Prerequisite for all chapters |
| **MVP (Chapter 1)** | Phase 3 done in 3 weeks | — | Learners can complete chapter 1 |
| **Chapter Count** | 16 + appendices | — | All chapters published by Week 20 |
| **Lab Success Rate** | ≥95% on Ubuntu 22.04 ROS 2 Humble | — | Tested via CI/CD |
| **RAG Accuracy** | ≥85% relevant results | — | 100+ query test suite |
| **Learner Completion** | ≥70% complete Chapter 1 | — | Beta cohort metric |
| **Assessment Pass Rate** | ≥80% for engaged learners | — | Quiz + lab combined |
| **Learner NPS** | ≥70 | — | Post-chapter feedback |
| **Deployment Uptime** | 99% | — | Monitoring dashboard |
| **Capstone Completion** | ≥50% of advanced learners complete | — | Final project metric |
| **Adoption (6 months)** | 500+ active learners | — | Google Analytics |
| **Adoption (1 year)** | 2000+ active learners | — | Google Analytics |

---

## Notes & Considerations

- **Agent-Driven**: Every chapter production invokes agents explicitly; no manual writing required once agents are in place
- **Reusable**: All agents (Research, Author, Lab, Diagram, Assessment, Personalization, Localization, RAG, QA) can be reused for future books
- **Parallelizable**: After Phase 2 completion, Chapters 3–16 can be orchestrated in parallel (4 at a time) with sufficient compute
- **Testable**: Each phase has clear acceptance criteria; each story can be validated independently
- **Observable**: All agent execution is logged (AgentExecutionRecords); orchestrator provides full visibility
- **Iterative**: Beta feedback feeds back into agent re-invocation (e.g., "Re-run Chapter Author with feedback from learners")
- **Measurable**: All success criteria are quantified; metrics tracked from day 1

---

## Sign-Off

**Tasks Status**: ✅ **READY FOR EXECUTION**

**Total Tasks**: 120 tasks across 12 phases

**Task Breakdown**:
- Setup: 10 tasks
- Foundational: 17 tasks
- User Story 1 (Chapter 1): 16 tasks
- User Story 2 (Orchestrator): 13 tasks
- User Story 3 (RAG Chatbot): 7 tasks
- User Story 4 (Urdu): 5 tasks
- Parallel Chapters (3–16): 17 tasks
- Capstone: 5 tasks
- RAG Full Indexing: 3 tasks
- QA & Testing: 9 tasks
- Deployment: 6 tasks
- Maintenance: 7 tasks

**MVP Scope** (Weeks 1–5):
- Phase 1: Setup (T001–T010)
- Phase 2: Foundational (T011–T027)
- Phase 3: Chapter 1 complete (T028–T044)
- **Deliverable**: Chapter 1 on Docusaurus, learner can complete chapter + assessment

**Full Scope** (Weeks 1–30):
- All 12 phases, all 120 tasks
- **Deliverable**: 16 chapters + appendices, RAG chatbot, Urdu translations, capstone, live on GitHub Pages

**Recommended Start**: Immediately (Phase 1 can begin today)

**Task Version**: 1.0.0 | **Date**: 2025-12-31 | **Feature**: `1-book-curriculum`
