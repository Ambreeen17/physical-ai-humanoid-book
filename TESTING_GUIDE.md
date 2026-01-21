# Full-Stack Testing Guide

## Prerequisites

### Required Services
- ✅ PostgreSQL database running
- ✅ Qdrant vector database (local or cloud)
- ✅ Backend API (FastAPI on port 8000)
- ✅ Frontend (Docusaurus on port 3000)
- ✅ Docker daemon (for lab testing)

### Environment Variables
Create `.env` files in appropriate locations:

**`backend/.env`**:
```env
DATABASE_URL=postgresql://user:password@localhost:5432/textbook_db
QDRANT_URL=http://localhost:6333  # or Qdrant Cloud URL
QDRANT_API_KEY=your_api_key_here  # if using Qdrant Cloud
OPENAI_API_KEY=your_openai_key
CLAUDE_API_KEY=your_claude_key
CORS_ORIGINS=["http://localhost:3000"]
```

**`frontend/.env`** (optional):
```env
REACT_APP_API_URL=http://localhost:8000
```

---

## Test Execution Order

### 1. Database Setup

```bash
# Start PostgreSQL (if using Docker)
docker run -d \
  --name textbook-postgres \
  -e POSTGRES_USER=textbook \
  -e POSTGRES_PASSWORD=password \
  -e POSTGRES_DB=textbook_db \
  -p 5432:5432 \
  postgres:15

# Start Qdrant (if using Docker)
docker run -d \
  --name textbook-qdrant \
  -p 6333:6333 \
  -p 6334:6334 \
  qdrant/qdrant:latest

# Apply database migrations (if using Alembic)
cd backend
alembic upgrade head

# Seed assessments
python scripts/seed_assessments.py
```

### 2. Start Backend

```bash
cd backend

# Install dependencies (first time)
pip install -r requirements.txt

# Start FastAPI server
uvicorn src.main:app --reload --host 0.0.0.0 --port 8000

# Verify: http://localhost:8000/docs
```

### 3. Start Frontend

```bash
cd frontend

# Install dependencies (first time)
npm install

# Start Docusaurus dev server
npm start

# Verify: http://localhost:3000
```

### 4. Run Full-Stack Tests

```bash
# Install test dependencies
pip install requests

# Run test suite
python scripts/test_fullstack.py
```

### 5. Test Lab Environment

```bash
cd specs/1-book-curriculum/chapters/chapter-1/lab

# Make script executable
chmod +x test_lab.sh

# Run lab tests
./test_lab.sh
```

---

## Expected Test Results

### Backend API Tests (8 tests)
```
BACKEND API TESTS
==========================================================

Testing: Backend health endpoint ... PASS
Testing: Health check endpoint ... PASS
Testing: Create learner profile ... PASS (or WARN if exists)
Testing: Get learner profile ... PASS
Testing: List chapter assessments ... PASS (requires seeding)
Testing: Submit quiz ... PASS (requires seeding)
Testing: Chat service health ... PASS
Testing: Chat RAG endpoint ... PASS (or WARN if Qdrant not configured)
```

### Frontend Tests (4 tests)
```
FRONTEND TESTS
==========================================================

Testing: Frontend home page ... PASS
Testing: Chapter 1 page loads ... PASS
Testing: Dashboard page loads ... PASS
Testing: Chat page loads ... PASS
```

### Integration Tests (1 test)
```
INTEGRATION TESTS
==========================================================

Testing: End-to-end user flow ... PASS
```

### Lab Environment Tests (12 tests)
```
Testing: Docker daemon running ... PASS
Testing: Docker Compose available ... PASS
Testing: Dockerfile exists ... PASS
Testing: docker-compose.yml exists ... PASS
Testing: hello_physical_ai_node.py exists ... PASS
Testing: Launch file exists ... PASS
Testing: Makefile exists ... PASS
Testing: Docker image builds successfully ... PASS
Testing: ROS 2 environment sourced ... PASS
Testing: turtlesim package available ... PASS
Testing: Python 3 available ... PASS
Testing: Node imports successfully ... PASS
```

**Total**: 25 tests

---

## Manual Testing Checklist

### Backend API (via Swagger UI)

Navigate to `http://localhost:8000/docs`

#### 1. Learner Profile
- [x] POST `/api/learner-profile/` - Create profile
  - **Test Data**: `{"learner_id": "manual-test-1", "python_score": 8, "ml_score": 6, "robotics_score": 4, "ros_score": 5}`
  - **Expected**: Returns profile with difficulty_level calculated
- [x] GET `/api/learner-profile/manual-test-1` - Retrieve profile
- [x] PUT `/api/learner-profile/manual-test-1` - Update profile
  - **Test Data**: `{"python_score": 9}`
  - **Expected**: Difficulty level recalculated
- [x] DELETE `/api/learner-profile/manual-test-1` - Delete profile

#### 2. Assessments
- [x] GET `/api/assessments/chapter/1` - List assessments
  - **Expected**: Returns 6 assessments (if seeded)
- [x] POST `/api/assessments/quiz/submit` - Submit quiz
  - **Test Data**: `{"learner_id": "manual-test-1", "assessment_id": 1, "answers": [{"question_id": "1.1", "selected_option": "A"}]}`
  - **Expected**: Returns score, pass/fail status, feedback
- [x] POST `/api/assessments/lab/submit` - Submit lab
  - **Test Data**: `{"learner_id": "manual-test-1", "assessment_id": 5, "console_output": "WARNING: Significant rotation detected: theta=1.2"}`
  - **Expected**: Validates output against regex patterns
- [x] GET `/api/assessments/results/manual-test-1` - Get results

#### 3. Chat
- [x] POST `/api/chat/` - Send chat message
  - **Test Data**: `{"query": "What is the sensorimotor loop?", "learner_id": "manual-test-1", "conversation_history": [], "top_k": 5}`
  - **Expected**: Returns response with source citations (if Qdrant configured)

### Frontend (via Browser)

#### 1. Homepage (`http://localhost:3000`)
- [x] Navigation bar renders (Chapters, AI Assistant, Dashboard, Locale, GitHub)
- [x] Hero section with textbook description
- [x] Footer with copyright

#### 2. Chapter 1 (`http://localhost:3000/docs/chapter-1`)
- [x] PersonalizationToggle component renders at top
- [x] Learning objectives list displays
- [x] All 5 sections render (1.1-1.5)
- [x] Code blocks syntax-highlighted
- [x] Tables render correctly
- [x] Lab callout box styled
- [x] Can switch difficulty (Beginner/Intermediate/Advanced)

#### 3. Dashboard (`http://localhost:3000/dashboard`)
- [x] Profile card displays (difficulty level, experience scores)
- [x] Progress overview shows stats
- [x] Assessment results list (if any submissions exist)
- [x] "Continue Learning" and "Edit Profile" buttons work

#### 4. Chat (`http://localhost:3000/chat`)
- [x] Chat interface renders
- [x] Can type and send messages
- [x] Messages display in conversation
- [x] Loading indicator shows while waiting
- [x] Source citations display (if configured)
- [x] Clear conversation button works

#### 5. Responsiveness
- [x] Mobile viewport (375px): Components stack correctly
- [x] Tablet viewport (768px): Grid layouts adjust
- [x] Desktop viewport (1920px): Max-width constraints respected

#### 6. Internationalization
- [x] Locale dropdown shows English and اردو
- [x] Can switch to Urdu (if locale files exist)
- [x] RTL layout applies for Urdu

### Lab Environment (via Terminal)

```bash
cd specs/1-book-curriculum/chapters/chapter-1/lab

# Test 1: Docker build
docker compose build
# Expected: Image builds in < 10 minutes (first time)

# Test 2: Start turtlesim
docker compose run --rm ros2_lab bash
ros2 run turtlesim turtlesim_node
# Expected: Window opens with turtle

# Test 3: Run lab node (in separate terminal)
docker compose run --rm ros2_lab bash
cd /ros2_ws/src/hello_physical_ai
python3 src/hello_physical_ai_node.py
# Expected: Console logs show "Position: x=..., Decision: ..., Action: ..."

# Test 4: Verify boundary behavior
# Expected: When turtle x > 2.0, it backs up
```

---

## Troubleshooting

### Issue: Backend not starting
**Symptoms**: `ModuleNotFoundError`, import errors

**Solution**:
```bash
cd backend
pip install -r requirements.txt
python -m src.main  # Test imports
```

### Issue: Database connection failed
**Symptoms**: `sqlalchemy.exc.OperationalError`

**Solution**:
```bash
# Check PostgreSQL is running
docker ps | grep postgres

# Verify connection string in .env
# Test connection
psql postgresql://textbook:password@localhost:5432/textbook_db
```

### Issue: Qdrant not reachable
**Symptoms**: Chat endpoint returns 500, `ConnectionRefusedError`

**Solution**:
```bash
# Check Qdrant is running
docker ps | grep qdrant

# Test connection
curl http://localhost:6333/collections

# If using Qdrant Cloud, verify API key in .env
```

### Issue: Frontend build errors
**Symptoms**: `Module not found`, React errors

**Solution**:
```bash
cd frontend
rm -rf node_modules package-lock.json
npm install
npm start
```

### Issue: Assessment seeding fails
**Symptoms**: `IntegrityError`, foreign key violations

**Solution**:
```bash
# Clear existing data
psql textbook_db -c "TRUNCATE TABLE assessments, assessment_results, chapters CASCADE;"

# Re-run seeding
python backend/scripts/seed_assessments.py
```

### Issue: Lab Docker image won't build
**Symptoms**: `failed to solve`, network timeout

**Solution**:
```bash
# Clear Docker cache
docker system prune -a

# Try pulling base image manually
docker pull osrf/ros:humble-desktop-full

# Build with verbose output
docker compose build --progress=plain
```

### Issue: Personalization toggle not working
**Symptoms**: Component doesn't render, console errors

**Solution**:
```bash
# Check MDXComponents.js registered correctly
cat frontend/src/theme/MDXComponents.js

# Verify import path
# Clear Docusaurus cache
cd frontend
npm run clear
npm start
```

---

## Performance Benchmarks

| Test | Expected Duration | Notes |
|------|-------------------|-------|
| Backend startup | < 5 seconds | First time may take longer (dependencies) |
| Frontend startup | 10-30 seconds | npm start initial build |
| Database seeding | < 10 seconds | 6 assessments for Chapter 1 |
| Docker lab build | 5-10 minutes | First time (cached: < 30 seconds) |
| Full test suite | < 2 minutes | Assumes all services running |
| RAG chat query | 2-5 seconds | Depends on Qdrant + Claude API |
| Assessment submission | < 500ms | Database write + scoring |

---

## Success Criteria

### Backend
- ✅ All 8 API tests pass
- ✅ Swagger UI accessible and functional
- ✅ No console errors during startup
- ✅ Database migrations applied
- ✅ Assessments seeded successfully

### Frontend
- ✅ All 4 page tests pass
- ✅ No console errors in browser
- ✅ All components render correctly
- ✅ Personalization toggle functional
- ✅ Chat interface sends/receives messages

### Lab
- ✅ All 12 lab tests pass
- ✅ Docker image builds successfully
- ✅ ROS 2 node imports without errors
- ✅ Sensorimotor loop executes correctly

### Integration
- ✅ End-to-end user flow completes
- ✅ Profile → Assessment → Results workflow works
- ✅ Chat retrieves relevant context from Qdrant
- ✅ Dashboard displays real data

---

## CI/CD Integration

### GitHub Actions Workflow

```yaml
name: Full-Stack Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04

    services:
      postgres:
        image: postgres:15
        env:
          POSTGRES_USER: textbook
          POSTGRES_PASSWORD: password
          POSTGRES_DB: textbook_db
        ports:
          - 5432:5432

      qdrant:
        image: qdrant/qdrant:latest
        ports:
          - 6333:6333

    steps:
      - uses: actions/checkout@v3

      - name: Set up Python
        uses: actions/setup-python@v4
        with:
          python-version: '3.10'

      - name: Install backend dependencies
        run: |
          cd backend
          pip install -r requirements.txt

      - name: Run database migrations
        run: |
          cd backend
          alembic upgrade head

      - name: Seed assessments
        run: |
          cd backend
          python scripts/seed_assessments.py

      - name: Start backend
        run: |
          cd backend
          uvicorn src.main:app --host 0.0.0.0 --port 8000 &
          sleep 5

      - name: Run full-stack tests
        run: python scripts/test_fullstack.py

      - name: Test lab environment
        run: |
          cd specs/1-book-curriculum/chapters/chapter-1/lab
          chmod +x test_lab.sh
          ./test_lab.sh
```

---

## Next Steps After Testing

1. **If all tests pass**:
   - ✅ Mark Task 1 complete
   - → Proceed to Task 2: Phase 4 execution (Chapters 2-4)

2. **If tests fail**:
   - Review error messages
   - Apply fixes from troubleshooting section
   - Re-run tests
   - Document any new issues

3. **Performance optimization** (if needed):
   - Profile slow endpoints
   - Optimize database queries
   - Add caching (Redis)
   - Compress frontend bundles

---

**Testing Guide Version**: 1.0
**Last Updated**: 2026-01-02
