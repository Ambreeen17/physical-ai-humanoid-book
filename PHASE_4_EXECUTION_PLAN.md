# Phase 4 Execution Plan: Chapters 2-4 Production

**Target**: Part I: Foundations (Kinematics, Sensors, State Estimation)
**Status**: Ready for execution
**Agent Pipeline**: 10-agent orchestration per chapter

---

## Chapter Specifications

### Chapter 2: Kinematics & Dynamics

**Description**: Joint spaces, forward/inverse kinematics, equations of motion

**Learning Objectives**:
1. Define joint space and configuration space for robot manipulators
2. Compute forward kinematics using Denavit-Hartenberg parameters
3. Solve inverse kinematics for 2-DOF and 3-DOF arms
4. Apply Lagrangian mechanics to derive equations of motion
5. Simulate robot dynamics in MuJoCo and Gazebo

**Key Topics**:
- Joint spaces and configuration spaces
- Denavit-Hartenberg (DH) parameters
- Forward kinematics computation
- Inverse kinematics (analytical and numerical)
- Lagrangian dynamics
- Newton-Euler formulation
- Simulation of multi-body dynamics

**Hardware Context**: Unitree G1 arm kinematics (6-DOF manipulation chain)

**Simulation Tools**: MuJoCo, Gazebo Harmonic, PyBullet

**Lab Exercise Ideas**:
- Lab 2.1: Compute forward kinematics for a 2-DOF arm
- Lab 2.2: Implement numerical inverse kinematics solver
- Lab 2.3: Simulate dynamics and compare with analytical predictions

---

### Chapter 3: Sensors & Actuators

**Description**: LiDAR, cameras, servo motors, force-torque sensors

**Learning Objectives**:
1. Characterize sensor performance (resolution, noise, latency)
2. Process LiDAR point clouds for obstacle detection
3. Calibrate RGB-D cameras for depth estimation
4. Select actuators based on torque, speed, and precision requirements
5. Implement sensor fusion using Kalman filters

**Key Topics**:
- Sensor taxonomy (proprioceptive vs exteroceptive)
- LiDAR: 2D vs 3D, time-of-flight vs phase-shift
- Cameras: RGB, depth (structured light, stereo, ToF)
- IMU: accelerometers, gyroscopes, magnetometers
- Force-torque sensors for manipulation
- Servo motors: DC, stepper, brushless
- Actuator selection criteria

**Hardware Context**: Unitree G1 sensor suite (3D LiDAR, RGB-D camera, IMU, joint encoders)

**Simulation Tools**: Isaac Sim (sensor simulation), Gazebo sensor plugins

**Lab Exercise Ideas**:
- Lab 3.1: Process LiDAR point cloud and detect obstacles
- Lab 3.2: Calibrate RGB-D camera using checkerboard pattern
- Lab 3.3: Characterize IMU noise and implement complementary filter

---

### Chapter 4: State Estimation

**Description**: Kalman filters, particle filters, sensor fusion

**Learning Objectives**:
1. Implement Kalman filters for position/velocity estimation
2. Apply Extended Kalman Filter (EKF) for nonlinear systems
3. Use particle filters for non-Gaussian distributions
4. Fuse IMU and odometry data for robust localization
5. Evaluate estimation accuracy and computational cost

**Key Topics**:
- Bayesian filtering framework
- Kalman Filter: prediction and update steps
- Extended Kalman Filter (EKF) for nonlinear dynamics
- Unscented Kalman Filter (UKF)
- Particle filters (Sequential Monte Carlo)
- Sensor fusion architectures
- SLAM (Simultaneous Localization and Mapping) basics

**Hardware Context**: Mobile robot localization using wheel encoders, IMU, and LiDAR

**Simulation Tools**: ROS 2 robot_localization package, Gazebo

**Lab Exercise Ideas**:
- Lab 4.1: Implement 1D Kalman filter for position tracking
- Lab 4.2: Apply EKF to nonlinear pendulum system
- Lab 4.3: Fuse IMU + odometry for mobile robot localization

---

## Agent Orchestration Pipeline (Per Chapter)

### Agent 1: Research Agent
**Input**: Chapter title, key topics, 2025 context
**Output**: `research.md` with state-of-art techniques, hardware specs, best practices
**Duration**: 15 minutes (per chapter)

### Agent 2: Chapter Author
**Input**: Research notes, learning objectives, audience context
**Output**: `chapter-draft.md` (4,000-5,000 words, publication-quality)
**Duration**: 30 minutes

### Agent 3: Diagram Generator
**Input**: Chapter content, key concepts requiring visualization
**Output**: `diagrams.md` (ASCII, Mermaid, accessibility descriptions)
**Duration**: 10 minutes

### Agent 4: Robotics Lab Generator
**Input**: Learning objectives, ROS 2 requirements, simulation tools
**Output**: Lab directory with Dockerfile, Python nodes, launch files, README
**Duration**: 20 minutes

### Agent 5: Assessment Generator
**Input**: Learning objectives, chapter content, difficulty progression
**Output**: `assessments.md` with MCQs, short-answer, lab exercises, challenges, rubrics
**Duration**: 15 minutes

### Agent 6: Personalization Agent
**Input**: Base chapter content, learner tiers (Beginner/Intermediate/Advanced)
**Output**: `personalization.md` with 3 content variants + metadata config
**Duration**: 20 minutes

### Agent 7: Localization Agent (Urdu)
**Input**: English chapter content
**Output**: `chapter-urdu.md` with bilingual format, glossary, style guide
**Duration**: 25 minutes

### Agent 8: RAG Indexing Agent
**Input**: Chapter content (all sections)
**Output**: `rag-manifest.json`, `chunks.jsonl`, `embeddings.npy`, upload script
**Duration**: 10 minutes

### Agent 9: QA Agent
**Input**: All chapter artifacts (draft, labs, assessments, personalization)
**Output**: `qa-report.md` with quality score, issues, recommendations
**Duration**: 15 minutes

### Agent 10: File Management (Implicit)
**Actions**: Organize artifacts, validate paths, create README files
**Duration**: 5 minutes

**Total Time per Chapter**: ~2.5 hours (agent execution + validation)

---

## Execution Instructions

### Option 1: Automated Orchestration (Recommended)

```bash
# Run orchestration script
cd scripts
python orchestrate_chapters_2_4.py
```

This will:
1. Sequentially invoke all 10 agents for each chapter
2. Store artifacts in `specs/1-book-curriculum/chapters/chapter-{N}/`
3. Generate execution report with pass/fail status
4. Output next-step recommendations

### Option 2: Manual Agent Invocation

For each chapter (2, 3, 4), run:

```bash
# 1. Research Agent
python -m backend.src.agents.research_agent --chapter {N} --output specs/1-book-curriculum/chapters/chapter-{N}/research.md

# 2. Chapter Author
python -m backend.src.agents.chapter_author --research specs/1-book-curriculum/chapters/chapter-{N}/research.md --output specs/1-book-curriculum/chapters/chapter-{N}/chapter-draft.md

# 3. Diagram Generator
python -m backend.src.agents.diagram_generator --chapter specs/1-book-curriculum/chapters/chapter-{N}/chapter-draft.md --output specs/1-book-curriculum/chapters/chapter-{N}/diagrams.md

# 4. Robotics Lab Generator
python -m backend.src.agents.robotics_lab_generator --chapter {N} --output specs/1-book-curriculum/chapters/chapter-{N}/lab/

# 5. Assessment Generator
python -m backend.src.agents.assessment_generator --chapter {N} --output specs/1-book-curriculum/chapters/chapter-{N}/assessments.md

# 6. Personalization Agent
python -m backend.src.agents.personalization_agent --chapter specs/1-book-curriculum/chapters/chapter-{N}/chapter-draft.md --output specs/1-book-curriculum/chapters/chapter-{N}/personalization.md

# 7. Localization Agent
python -m backend.src.agents.localization_agent --chapter specs/1-book-curriculum/chapters/chapter-{N}/chapter-draft.md --output specs/1-book-curriculum/chapters/chapter-{N}/chapter-urdu.md

# 8. RAG Indexing Agent
python -m backend.src.agents.rag_indexing_agent --chapter {N} --output specs/1-book-curriculum/chapters/chapter-{N}/rag-manifest.json

# 9. QA Agent
python -m backend.src.agents.qa_agent --chapter {N} --output specs/1-book-curriculum/chapters/chapter-{N}/qa-report.md
```

### Option 3: Claude Code Agent Invocation

Use the Task tool to invoke specialized agents:

```
Invoke Research Agent for Chapter 2: Kinematics & Dynamics
- Context: Unitree G1 arm kinematics, DH parameters, MuJoCo/Gazebo simulation
- Output: research.md with 2025 state-of-art
```

---

## Post-Execution Checklist

After each chapter is produced:

### 1. **Quality Validation**
- [ ] QA score ≥ 85/100
- [ ] All learning objectives covered
- [ ] Labs executable in Docker
- [ ] Assessments align with objectives
- [ ] Hardware specs accurate (2025)

### 2. **Integration into Docusaurus**
- [ ] Convert `chapter-draft.md` to `frontend/docs/chapter-{N}.md`
- [ ] Add front matter (sidebar_position, title)
- [ ] Insert PersonalizationToggle component
- [ ] Update sidebars.js if needed

### 3. **Database Seeding**
- [ ] Run `seed_assessments.py` for Chapter {N}
- [ ] Verify assessments via Swagger UI
- [ ] Test quiz/lab submission endpoints

### 4. **RAG Upload**
- [ ] Upload chunks to Qdrant Cloud
- [ ] Verify semantic search retrieval
- [ ] Test chatbot with chapter-specific queries

### 5. **Lab Testing**
- [ ] Run `test_lab.sh` for Chapter {N} lab
- [ ] Verify Docker build succeeds
- [ ] Test lab execution manually
- [ ] Update CI/CD workflow

---

## Expected Deliverables (Per Chapter)

```
specs/1-book-curriculum/chapters/chapter-{N}/
├── research.md                     # Research agent output
├── chapter-draft.md                # Main chapter content (4,000-5,000 words)
├── diagrams.md                     # Diagram specifications (ASCII, Mermaid)
├── assessments.md                  # Assessment suite (MCQs, labs, challenges)
├── personalization.md              # Beginner/Intermediate/Advanced variants
├── chapter-urdu.md                 # Bilingual English-Urdu content
├── rag-manifest.json               # RAG chunk metadata
├── chunks.jsonl                    # Semantic chunks for Qdrant
├── embeddings.npy                  # OpenAI embeddings (1536-dim)
├── upload_script.py                # Qdrant upload automation
├── qa-report.md                    # Quality assurance report
├── lab/
│   ├── Dockerfile
│   ├── docker-compose.yml
│   ├── src/
│   │   └── {chapter_node}.py       # ROS 2 Python node
│   ├── launch/
│   │   └── {chapter_launch}.launch.py
│   ├── Makefile
│   ├── README.md                   # Lab instructions
│   ├── test_lab.sh                 # Automated test script
│   └── expected_output.txt         # Sample console output
```

---

## Success Metrics

**Phase 4 Complete When**:
- ✅ All 3 chapters (2, 3, 4) produced with 10-agent pipeline
- ✅ QA scores ≥ 85/100 for all chapters
- ✅ All labs executable in Docker
- ✅ All assessments seeded in database
- ✅ All RAG chunks uploaded to Qdrant
- ✅ Docusaurus integration complete
- ✅ CI/CD pipeline passing for all labs

**Key Performance Indicators**:
- Chapter production time: ~2.5 hours/chapter
- Total Phase 4 duration: ~8 hours (3 chapters + integration)
- Quality baseline: 85/100 (QA Agent score)
- Lab success rate: 100% (all tests pass)

---

## Risk Mitigation

### Risk 1: Agent execution failures
**Mitigation**: Manual fallback for failed agents, retry with adjusted parameters

### Risk 2: Hardware spec inaccuracies
**Mitigation**: Research Agent validates all specs against 2025 sources, QA Agent cross-checks

### Risk 3: Lab Docker build failures
**Mitigation**: Pre-test base images, use CI/CD for automated validation

### Risk 4: RAG embedding generation rate limits
**Mitigation**: Batch embeddings, use OpenAI rate limit handling, fallback to local models

---

## Next Steps After Phase 4

### Phase 5: Part II Production (Chapters 5-8)
- Computer Vision for Robotics
- Control Theory Fundamentals
- Motion Planning
- Manipulation & Grasping

### Phase 6: Part III Production (Chapters 9-12)
- Task & Motion Planning
- Reinforcement Learning for Robotics
- Imitation Learning
- Vision-Language-Action Models

### Phase 7: Part IV Production (Chapters 13-16)
- System Integration
- Safety & Robustness
- Deployment & Operations
- Capstone Project

---

**Phase 4: Ready for Execution ✅**
**Estimated Completion**: 8 hours (3 chapters + integration)
