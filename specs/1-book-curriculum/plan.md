# Implementation Plan: AI-Native Physical AI & Humanoid Robotics Textbook

**Branch**: `1-book-curriculum` | **Date**: 2025-12-31 | **Spec**: [spec.md](spec.md)

---

## Summary

This plan transforms the textbook specification into a phased, executable roadmap using reusable Claude Code agents. The core innovation is **Agent-Driven Content Production**: every chapter is orchestrated by the Book Orchestrator Agent, which sequences 9 specialized agents (Research, Author, Lab, Diagram, Assessment, Personalization, Localization, RAG Indexing, QA) with clear input/output contracts.

The system teaches embodied intelligence and humanoid robotics through hands-on labs (ROS 2 Humble, Gazebo, Isaac Sim), semantic search over content (RAG), and multi-language support (Urdu). Success means shipping 16+ chapters, each with theory, diagrams, executable labs, assessments, and Urdu translations—all produced by coordinated AI agents.

---

## Technical Context

**Language/Version**: Python 3.9+ (ROS 2 ecosystem standard)

**Primary Dependencies**:
- ROS 2 Humble (Ubuntu 22.04 LTS)
- Gazebo (open-source simulator; fallback for Isaac Sim)
- NVIDIA Isaac Sim 5.0 (advanced labs)
- FastAPI (assessment backend + chatbot API)
- Docusaurus (textbook website)
- Qdrant Cloud (RAG vector database)
- Docker (lab containerization)
- Vision-Language-Action models (SmolVLA, π0) for humanoid control examples

**Storage**:
- Neon Serverless Postgres (learner profiles, assessment results)
- GitHub (content versioning, chapters, code)
- Qdrant Cloud (RAG embeddings)
- S3-compatible storage (video assets, large lab outputs)

**Testing**:
- pytest (Python labs)
- ROS 2 launch tests (node validation)
- Gazebo simulation tests (lab execution validation)
- Manual/automated QA for chapter coherence and completeness

**Target Platform**:
- Web: Docusaurus + FastAPI (cloud-deployable)
- Local: Ubuntu 22.04 with Docker (learner development environment)
- Cloud: Google Colab, AWS EC2 (fallback execution paths)

**Project Type**: Multi-agent system + web application (frontend: Docusaurus; backend: FastAPI; agent orchestration: Claude Code)

**Performance Goals**:
- Labs execute end-to-end within 30 minutes on consumer hardware (Gazebo) or cloud (Isaac Sim)
- RAG chatbot returns relevant results within 2 seconds
- Docusaurus page loads in <2 seconds globally (CDN-backed)
- Agent orchestration completes per chapter within 4 hours

**Constraints**:
- ROS 2 Humble on Ubuntu 22.04 MUST be the baseline (no breaking changes)
- No proprietary dependencies; open-source tooling prioritized
- All labs MUST be containerized (Docker) for reproducibility
- Urdu translation MUST preserve code examples and technical terms unchanged
- RAG semantic search accuracy MUST be ≥85% for domain-specific queries

**Scale/Scope**:
- 16 core chapters + appendices
- 15-20 executable labs
- 500-2000 active learners in year 1
- 9 reusable agents (Book Orchestrator, Research, Author, Lab, Diagram, Assessment, Personalization, Localization, RAG Indexing, QA)

---

## Constitution Check

**Gate: Must pass before Phase 0 research. Re-check after Phase 1 design.**

| Constitutional Principle | Alignment | Status |
|--------------------------|-----------|--------|
| **I. Embodied Intelligence First** | Every chapter includes hands-on labs; no theory-only content. Labs start in Gazebo (accessible), scale to Isaac Sim (advanced), with real hardware migration path. | ✅ PASS |
| **II. Simulation-to-Real Continuity** | All labs use Gazebo (open-source) as baseline; Isaac Sim (proprietary) as optional advanced path; code portability between simulators prioritized via USD format. | ✅ PASS |
| **III. Agent-Driven Knowledge Creation** | Entire content production orchestrated by 9 reusable agents; agent contributions transparent and attributed; production workflow itself is learnable. | ✅ PASS |
| **IV. Hardware-Aware Teaching** | Labs reference real robots (Unitree G1, Fourier GR-1, Boston Dynamics Atlas); open-source alternatives (HEBI Robotics modules, STOMPY quadruped) documented; cost, power consumption, sensor specs always explicit. | ✅ PASS |
| **V. Modular, Reusable, Scalable Design** | Every artifact (chapter, lab, assessment, diagram, translation) is single-responsibility; agents are reusable across chapters; no chapter-specific workarounds. | ✅ PASS |

**All constitutional gates PASS.** Design proceeds without violations.

---

## Architecture & Agent Workflow

### Global Agent Invocation (Per Chapter)

```
Book Orchestrator Agent (entry point)
├── Curriculum Analyst Agent (validates progression, prerequisites)
├── Research Agent (gathers knowledge, identifies unknowns)
├── Chapter Author Agent (writes theory, objectives, key concepts)
├── Diagram Agent (specifies ASCII, Mermaid, visual diagrams)
├── Robotics Lab Generator Agent (creates ROS 2 labs, Gazebo/Isaac configs)
├── Assessment Generator Agent (quizzes, lab exercises, capstone challenges)
├── Personalization Agent (beginner/intermediate adaptive notes)
├── Localization Agent (Urdu translation, code preservation)
├── RAG Indexing Agent (semantic chunking, embedding generation, upload to Qdrant)
└── QA & Consistency Agent (validation report, issue resolution)
```

**Execution Model**:
- **Sequential**: Each agent's output feeds into the next agent's input
- **Handoff**: Artifacts are linked (e.g., "See Research Agent output in `research.md`")
- **Failure Handling**: If any agent fails, editor receives automated alert; manual override possible; QA validates recovery
- **Cycle Time**: Target 4 hours per chapter from orchestrator start to publication-ready

### Phase 0: Governance & Alignment (Week 1)

**Agents Invoked**: Book Orchestrator, Curriculum Analyst

**Goals**:
1. Validate chapter sequencing against constitution
2. Confirm learning progression (Beginner → Intermediate)
3. Verify all chapters align to hands-on outcomes

**Outputs**:
- Approved learning roadmap (`roadmap.md`)
- Validated chapter dependency graph
- Curriculum alignment report

**Validation Gate**: Curriculum Analyst confirms that learner can progress from Chapter 1 (Introduction to Physical AI) → Chapter 16 (Capstone: Autonomous Humanoid) with clear prerequisites and skill buildup.

---

### Phase 1: Chapter Research & Authoring (Weeks 2–20, parallel for 4 chapters at a time)

**Per Chapter**:

1. **Research Agent** (2 hours)
   - Gathers current knowledge on chapter topic (ROS 2 patterns, sensor fusion, VLA models, etc.)
   - Identifies key references, hardware specs, open-source tools
   - Produces `research.md` with citations and decision rationale

2. **Chapter Author Agent** (3–4 hours)
   - Writes chapter outline, learning objectives, key concepts
   - Creates narrative explanations grounded in embodied intelligence
   - Includes real-world context and industry examples
   - Cites research artifacts explicitly
   - Produces `chapter-draft.md` (theory section)

3. **Diagram Agent** (1–2 hours)
   - Reviews chapter content, identifies diagram opportunities
   - Specifies diagrams: ASCII (terminal-friendly), Mermaid (interactive), descriptions (accessible)
   - Examples: sensor fusion pipeline, ROS 2 topic graph, robot kinematics chain
   - Produces `diagrams.md` with all diagram specs and placement

4. **Robotics Lab Generator Agent** (2–3 hours, if technical chapter)
   - Designs hands-on lab for chapter topic
   - Creates ROS 2 nodes, launch files, Gazebo world configs
   - Produces Docker setup, step-by-step instructions, expected output
   - Tests on Ubuntu 22.04 + ROS 2 Humble
   - Produces `lab/` directory with executable code

5. **Assessment Generator Agent** (1–2 hours)
   - Designs 3+ multiple-choice questions aligned to learning objectives
   - Creates 1+ short-form lab exercise (within chapter, 15–30 min)
   - Defines capstone challenge (if applicable)
   - Produces `assessments.md` with rubrics

6. **Personalization Agent** (30 min)
   - Generates beginner-focused notes (analogies, step-by-step breakdowns)
   - Generates intermediate-focused notes (advanced context, research references)
   - Marks content sections with `[BEGINNER: ...]` / `[INTERMEDIATE: ...]` tags
   - Produces `personalization.md` with profile-specific variants

7. **Localization Agent (Urdu)** (1–2 hours, async)
   - Identifies Urdu translation candidates (key sections, critical concepts)
   - Produces bilingual Markdown with side-by-side English/Urdu
   - Preserves all code blocks, diagrams, technical terms in English
   - Produces `chapter-urdu.md` (bilingual)

8. **RAG Indexing Agent** (30 min)
   - Chunks chapter content into semantic segments (paragraphs, sections)
   - Generates embeddings via OpenAI API (or local model)
   - Uploads to Qdrant Cloud with metadata (chapter ID, section, position, timestamp)
   - Produces `rag-manifest.json` (chunk IDs, metadata)

9. **QA & Consistency Agent** (1–2 hours)
   - Validates chapter terminology (consistency with prior chapters, constitution)
   - Checks ROS 2 version alignment, tool version pinning
   - Confirms all labs are executable (dry-run or full test)
   - Verifies diagrams are linked, assessments are rubric-complete
   - Produces `qa-report.md` with issues and sign-off

**Chapter Output Package**:
```
specs/1-book-curriculum/chapters/chapter-N/
├── research.md              # Research Agent output
├── chapter-draft.md         # Chapter Author output
├── diagrams.md              # Diagram Agent specs
├── lab/                     # Robotics Lab Agent output
│   ├── docker-compose.yml
│   ├── src/
│   ├── worlds/
│   ├── README.md
│   └── expected-output/
├── assessments.md           # Assessment Agent output
├── personalization.md       # Personalization Agent output
├── chapter-urdu.md          # Localization Agent output
├── rag-manifest.json        # RAG Indexing Agent output
├── qa-report.md             # QA Agent output
└── CHAPTER.md               # Final published chapter (merged from all sources)
```

**Parallelization**: Execute 4 chapters in parallel (Chapter 1 + 2 + 15 + 16 simultaneously), then 4 more, etc. Total time: 5 parallel waves × 4 hours per wave = 20 weeks (worst case; optimistically 12 weeks with agent parallelization).

---

### Phase 2: Learner Experience & Integration (Weeks 20–24)

**Goals**:
1. Build Docusaurus website with all chapters
2. Implement learner personalization (background detection, content adaptation)
3. Build RAG chatbot backend (FastAPI + Qdrant integration)
4. Implement assessment submission & feedback
5. Integrate Urdu translation toggle

**Key Components**:

#### 2.1 Docusaurus Site
- Template: Chapter layout with navigation, search, metadata
- Navigation: TOC, chapter links, prerequisite indicators
- Features: Syntax highlighting for code, responsive design, mobile-friendly
- Hosting: GitHub Pages (free; auto-deploy on main branch)

#### 2.2 Learner Profile & Personalization
- **Detection**: Onboarding quiz (5 questions) → background score (0-10 scale)
  - "How many years of software engineering?" → software score
  - "Robotics experience?" → robotics score
  - "ML/AI background?" → AI score
- **Adaptation**: Load personalization notes into chapter; show/hide [BEGINNER: ...] or [INTERMEDIATE: ...] sections based on profile
- **Storage**: Neon Postgres table `learner_profiles` (id, background_scores, preferences)

#### 2.3 RAG Chatbot Backend
- **Endpoint**: POST `/api/v1/chat` (FastAPI)
- **Input**: Query string + optional chapter scope filter
- **Execution**:
  1. Embed query via OpenAI API
  2. Search Qdrant Cloud with similarity threshold (0.7+)
  3. Retrieve top-5 chunks with metadata
  4. Synthesize answer using Claude API with retrieved context
  5. Format response with in-line citations and section links
- **Output**: JSON with `answer`, `citations` (array of section links), `confidence`

#### 2.4 Assessment Submission & Validation
- **Quiz Submission**: POST `/api/v1/assessments/<chapter>/submit`
  - Input: learner_id, chapter_id, answers (array)
  - Validation: Compare against answer key
  - Output: score, feedback per question, pass/fail status
- **Lab Submission**: Learner uploads lab output (code + logs)
  - Validation: Run automated checks (code linting, expected output patterns)
  - Output: pass/fail, issues, links to remediation
- **Storage**: `assessment_results` table (learner_id, chapter_id, score, timestamp, details)

#### 2.5 Urdu Content Toggle
- **UI**: Language dropdown in chapter header (English / Urdu)
- **Backend**: Serve `chapter-urdu.md` when Urdu selected
- **Fallback**: Gracefully degrade to English if Urdu not available for chapter

---

### Phase 3: Quality, Iteration & Launch (Weeks 24–30)

**Goals**:
1. Run full QA suite across all chapters
2. Perform learner testing (cohort of 10–20 beta learners)
3. Iterate based on feedback (clarity, lab fixes, assessment tweaks)
4. Deploy to production (Docusaurus on GitHub Pages + FastAPI on cloud)

**QA Gates**:
- [ ] All labs execute successfully on Ubuntu 22.04 + ROS 2 Humble
- [ ] RAG chatbot returns relevant results for 100+ test queries (85%+ accuracy)
- [ ] All chapters have Urdu translations for key sections
- [ ] Learner onboarding quiz + personalization working
- [ ] Assessment validation logic correct
- [ ] Performance benchmarks met (2s page load, chatbot <2s response, labs <30 min)

**Beta Learner Cohort** (10–20 learners, diverse backgrounds):
- Track completion rate, time per chapter, assessment scores
- Collect feedback via post-chapter surveys (NPS, clarity, relevance, lab difficulty)
- Identify pain points (confusing explanations, broken labs, missing prerequisites)
- Iterate: Fix top 10 issues, re-test, measure improvement

**Production Deployment**:
- Docusaurus: Build → commit to GitHub → auto-deploy via Pages
- FastAPI: Deploy to cloud platform (AWS EC2, Google Cloud Run, or Render)
- Qdrant: Ensure Qdrant Cloud connection stable, backups in place
- Database: Neon Postgres connection pooling configured, monitoring enabled
- Monitoring: Log errors, measure API latency, track learner engagement metrics

---

## Data Model & Entities

### Core Entities

**Chapter**
- id (UUID)
- number (1–16)
- title (string)
- topic (string)
- learning_objectives (array of strings)
- prerequisites (array of chapter IDs)
- content (markdown)
- diagrams (array of diagram specs)
- lab_id (foreign key to Lab, if applicable)
- assessments (array of assessment IDs)
- urdu_translation (markdown, optional)
- rag_manifest (JSON with chunk metadata)
- created_at, updated_at (timestamps)
- status (draft → research → in_progress → review → published)

**Lab**
- id (UUID)
- chapter_id (foreign key)
- title (string)
- description (string)
- difficulty (beginner / intermediate / advanced)
- estimated_duration (minutes)
- tools (array: "ROS 2", "Gazebo", "Isaac Sim", etc.)
- docker_image (string, e.g., "ros:humble")
- source_code_repo (URL)
- expected_output (string, pattern for validation)
- test_status (pass / fail, with last test date)
- tested_on_versions (array of ROS 2 versions, Isaac versions)

**Assessment**
- id (UUID)
- chapter_id (foreign key)
- type (quiz / lab_exercise / capstone_challenge)
- questions (array of question objects)
- rubric (JSON with scoring rules)
- answer_key (JSON, encrypted)
- difficulty (beginner / intermediate / advanced)
- time_limit (minutes, or null)

**LearnerProfile**
- id (UUID)
- email (string, unique)
- software_background (0–10 score)
- robotics_background (0–10 score)
- ai_background (0–10 score)
- personalization_preferences (JSON)
- language (English / Urdu)
- created_at, updated_at
- completed_chapters (array of chapter IDs)

**AssessmentResult**
- id (UUID)
- learner_id (foreign key)
- chapter_id (foreign key)
- assessment_id (foreign key)
- submission_date (timestamp)
- answers (JSON, encrypted if needed)
- score (float, 0–100)
- feedback (string)
- status (pass / fail / in_progress)
- lab_output_url (S3 URL, if lab submission)

**RAGChunk**
- id (UUID)
- chapter_id (foreign key)
- section_title (string)
- content (string, the actual text)
- embedding (vector, 1536-dim from OpenAI)
- metadata (JSON: position, subsection, difficulty)
- created_at

**AgentExecutionRecord**
- id (UUID)
- chapter_id (foreign key)
- agent_name (string: Research, Author, Lab, etc.)
- status (queued / running / success / failure)
- started_at, completed_at (timestamps)
- input_artifacts (array of file paths)
- output_artifacts (array of file paths)
- error_message (string, if failure)
- duration_seconds (integer)
- metadata (JSON: tool versions, model used, etc.)

---

## API Contracts

### Learner Personalization

**Endpoint**: `POST /api/v1/learners/profile`

**Request**:
```json
{
  "email": "learner@example.com",
  "onboarding_answers": [
    { "question": "software_years", "answer": 5 },
    { "question": "robotics_experience", "answer": "none" },
    { "question": "ai_experience", "answer": "coursework" },
    { "question": "preferred_language", "answer": "English" },
    { "question": "learning_pace", "answer": "intermediate" }
  ]
}
```

**Response**:
```json
{
  "learner_id": "uuid-123",
  "software_background": 7,
  "robotics_background": 2,
  "ai_background": 4,
  "recommended_chapter_1_difficulty": "intermediate",
  "personalization_active": true
}
```

### RAG Chatbot Query

**Endpoint**: `POST /api/v1/chat`

**Request**:
```json
{
  "query": "How do ROS 2 topics differ from services?",
  "chapter_scope": "Chapter 3" // optional; defaults to book-wide
}
```

**Response**:
```json
{
  "answer": "ROS 2 topics are pub-sub channels for continuous data (e.g., sensor streams), while services are synchronous request-reply patterns (e.g., on-demand computations). Topics are one-way; services are bidirectional...",
  "citations": [
    {
      "chapter": "Chapter 3: ROS 2 Fundamentals",
      "section": "3.2 Communication Patterns",
      "excerpt": "Topics publish streams of messages...",
      "link": "/chapters/3#section-3-2"
    }
  ],
  "confidence": 0.92
}
```

### Assessment Submission

**Endpoint**: `POST /api/v1/assessments/chapter-3/submit`

**Request**:
```json
{
  "learner_id": "uuid-123",
  "answers": [
    { "question_id": "q1", "answer": "A" },
    { "question_id": "q2", "answer": "Services are request-reply" }
  ]
}
```

**Response**:
```json
{
  "chapter_id": "3",
  "score": 85,
  "status": "pass",
  "feedback": [
    { "question_id": "q1", "correct": true, "explanation": "..." },
    { "question_id": "q2", "correct": true, "explanation": "..." }
  ],
  "next_chapter_unlocked": "4"
}
```

---

## Project Structure

### Documentation (this feature)

```
specs/1-book-curriculum/
├── spec.md              # Feature specification
├── plan.md              # This file (implementation plan)
├── research.md          # Phase 0 output (research findings)
├── data-model.md        # Phase 1 output (entity definitions)
├── contracts/           # Phase 1 output (API specs)
│   ├── learner-profile-api.md
│   ├── rag-chatbot-api.md
│   └── assessment-api.md
├── quickstart.md        # Phase 1 output (developer setup)
├── checklists/
│   └── requirements.md  # Specification quality checklist
└── chapters/            # Phase 1+ output (per-chapter artifacts)
    ├── chapter-1/
    │   ├── research.md
    │   ├── chapter-draft.md
    │   ├── diagrams.md
    │   ├── lab/
    │   ├── assessments.md
    │   ├── personalization.md
    │   ├── chapter-urdu.md
    │   ├── rag-manifest.json
    │   ├── qa-report.md
    │   └── CHAPTER.md
    └── chapter-2/
        └── [same structure]
```

### Source Code (repository root)

#### Backend (FastAPI)

```
backend/
├── src/
│   ├── main.py                 # FastAPI app entry
│   ├── config.py               # Environment, database config
│   ├── api/
│   │   ├── v1/
│   │   │   ├── learners.py     # Learner profile endpoints
│   │   │   ├── chat.py         # RAG chatbot endpoint
│   │   │   └── assessments.py  # Assessment submission
│   │   └── health.py           # Health check
│   ├── models/
│   │   ├── chapter.py
│   │   ├── learner.py
│   │   ├── assessment.py
│   │   └── rag.py
│   ├── services/
│   │   ├── llm_service.py      # Claude API calls
│   │   ├── qdrant_service.py   # RAG vector search
│   │   └── assessment_service.py # Grading logic
│   └── db/
│       ├── models.py           # SQLAlchemy ORM
│       └── session.py          # Neon Postgres connection
├── tests/
│   ├── unit/
│   │   ├── test_rag_service.py
│   │   └── test_assessment_service.py
│   ├── integration/
│   │   └── test_chat_endpoint.py
│   └── conftest.py
├── Dockerfile
├── requirements.txt
└── README.md
```

#### Frontend (Docusaurus)

```
website/
├── docusaurus.config.js        # Site config
├── sidebars.js                 # Navigation tree
├── docs/
│   ├── intro.md                # Landing page
│   ├── chapter-1/
│   │   ├── index.md            # Chapter entry
│   │   ├── theory.md           # Merged chapter content
│   │   └── lab.md              # Lab instructions + code
│   ├── chapter-2/
│   └── appendices/
│       ├── hardware-specs.md
│       ├── glossary.md
│       └── troubleshooting.md
├── src/
│   ├── components/
│   │   ├── ChatBot.jsx         # RAG chatbot widget
│   │   ├── PersonalizationToggle.jsx
│   │   └── AssessmentWidget.jsx
│   ├── pages/
│   │   ├── index.jsx           # Home
│   │   └── dashboard.jsx       # Learner progress
│   └── styles/
│       └── custom.css
├── static/
│   └── diagrams/               # ASCII, Mermaid diagrams
├── Dockerfile
└── package.json
```

**Structure Decision**: This is a **Web application** (frontend + backend). Docusaurus handles static content delivery (chapters, theory, diagrams); FastAPI backend manages dynamic features (learner profiles, RAG chatbot, assessments, database). Clear separation of concerns; frontend is static-friendly for GitHub Pages, backend is cloud-deployable.

---

## Complexity Justification

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|-----------|-------------------------------------|
| N/A | Constitution gates all PASS; no violations. | — |

---

## Implementation Roadmap Timeline

| Phase | Duration | Key Milestones | Agents Invoked |
|-------|----------|---|---|
| **Phase 0: Governance** | Week 1 | Curriculum roadmap approved, constitution check passed | Book Orchestrator, Curriculum Analyst |
| **Phase 1: Content Production** | Weeks 2–20 | Chapters 1–16 written, researched, diagrammed, labs built, assessments created, personalized, translated, indexed, QA validated | All 9 agents (per chapter, parallel) |
| **Phase 2: Learner Experience** | Weeks 20–24 | Docusaurus site built, learner profiles implemented, RAG chatbot integrated, assessments live, Urdu toggle working | Book Orchestrator (orchestrates integration) |
| **Phase 3: QA & Launch** | Weeks 24–30 | Full QA pass, beta learner cohort testing, production deployment, monitoring active | QA Agent, all agents for iteration |

**Total Duration**: 30 weeks (7.5 months) in critical path; parallelization can reduce to 12–16 weeks with 4-chapter parallel waves.

---

## Success Metrics (Aligned to Spec)

| Metric | Target | Measurement |
|--------|--------|---|
| **Content Completeness** | All 16 chapters + appendices published | Count of published chapters, QA pass rate |
| **Lab Success** | ≥95% executable on Ubuntu 22.04 ROS 2 Humble | Automated lab validation tests, learner feedback |
| **RAG Accuracy** | ≥85% relevant results for domain queries | Human evaluation of top-5 results per query type |
| **Learner Engagement** | 2–4 hours per chapter, ≥80% assessment pass rate | Time tracking, score analytics |
| **Adoption** | 500+ learners in 6 months, 2000+ in 1 year | Google Analytics, learner registration metrics |
| **Urdu Coverage** | 80% of chapters translated | Count of Urdu chapters vs total |
| **Uptime** | 99% availability | Monitoring dashboards (Datadog, CloudWatch) |
| **Agent Orchestration** | <4 hours per chapter from start to publication | Time tracking per agent, per chapter |

---

## Risks & Mitigations

| Risk | Impact | Probability | Mitigation |
|------|--------|---|---|
| **ROS 2 API Changes** | Labs break when ROS 2 updates | Medium | Pin ROS 2 version in Dockerfile; test on every LTS release; maintain compatibility guide |
| **Isaac Sim Availability** | Advanced labs unavailable for learners without NVIDIA GPUs | High | Always provide Gazebo fallback; document hardware requirements upfront; offer cloud lab access |
| **RAG Semantic Search Fails** | Chatbot returns irrelevant results | Medium | Implement custom fine-tuning if needed; add hybrid BM25 search; user feedback loop for ranking tuning |
| **Agent Production Bottleneck** | Content production slower than expected | Medium | Parallelize chapters; pre-research common topics; optimize agent prompts for speed |
| **Learner Churn** | Low completion rates if content is too hard | Medium | Implement adaptive difficulty; early feedback mechanisms; optional review labs |
| **Urdu Translation Quality** | Learners find translations inaccurate | Low | Human review of all Urdu translations; maintain glossary of technical terms; iterative refinement |

---

## Dependencies & Assumptions

**Technical Dependencies**:
- Claude Code agents must remain available and responsive (if unavailable, manual fallback required)
- OpenAI API (for embeddings, chatbot synthesis) must be stable
- Qdrant Cloud API must support 16 chapters × 50+ chunks/chapter = 800+ embeddings
- GitHub Pages free tier sufficient for Docusaurus site
- Cloud hosting for FastAPI (AWS, Google Cloud, Render) must be configured with proper monitoring

**Organizational Dependencies**:
- Domain experts (ROS 2, humanoids, VLA) available for research phase
- Graphics/illustration resources for diagrams (or reliance on ASCII/Mermaid)
- Beta learner cohort (10–20 volunteers) for testing in Phase 3

**Assumptions** (from Spec):
- Learners have access to Ubuntu 22.04 or Docker Desktop; Gazebo runs on commodity hardware
- Internet access always available (for Docusaurus, RAG backend, learner profiles)
- ROS 2 Humble (2024 LTS) remains stable through 2026
- Vision-Language-Action models (SmolVLA, π0) remain accessible via Hugging Face / hosted APIs

---

## Next Steps

1. **Immediate (This Week)**:
   - Approve this plan
   - Set up GitHub repos (content, backend, frontend)
   - Provision cloud infrastructure (FastAPI hosting, Postgres, Qdrant Cloud)
   - Queue Chapter 1 for orchestration

2. **Short-term (Weeks 1–2)**:
   - Invoke Book Orchestrator for Chapter 1
   - Run Phase 0 (governance, curriculum validation)
   - Begin Phase 1 (research, authoring) for Chapters 1, 2, 15, 16 in parallel

3. **Medium-term (Weeks 2–20)**:
   - Complete all 16 chapters via agent orchestration
   - Iterate based on QA reports
   - Begin Phase 2 integration (Docusaurus, backend services)

4. **Long-term (Weeks 20–30)**:
   - Complete Phase 2 & 3 (QA, beta testing, launch)
   - Deploy to production
   - Monitor metrics, iterate based on learner feedback

---

## Sign-Off

**Plan Status**: ✅ Ready for Phase 0 Execution

**Approved By**: Architecture Review (Constitutional gates all PASS)

**Plan Version**: 1.0.0 | **Date**: 2025-12-31
