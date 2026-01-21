# Startup Validation Checklist

Use this guide to validate your local development setup is working correctly.

## Phase 1: Environment Validation

### 1.1 Prerequisites Check

```bash
# Docker version (should be 20.0+)
docker --version

# Docker Compose version (should be 2.0+)
docker-compose --version

# Verify .env file exists
ls .env

# Check API keys are set (should not be empty)
grep OPENAI_API_KEY .env
grep CLAUDE_API_KEY .env
```

**Expected**: All commands succeed, API keys present.

### 1.2 Start Services

```bash
# Start all services
docker-compose -f docker-compose.dev.yml up -d

# Wait 30 seconds for services to initialize
timeout 30

# Check service status
docker-compose -f docker-compose.dev.yml ps
```

**Expected**: All services show "Up (healthy)" status:
- textbook-postgres
- textbook-qdrant
- textbook-redis
- textbook-backend
- textbook-frontend
- textbook-pgadmin

## Phase 2: Database Validation

### 2.1 PostgreSQL Health

```bash
# Test database connection
docker-compose -f docker-compose.dev.yml exec postgres pg_isready -U textbook

# List databases
docker-compose -f docker-compose.dev.yml exec postgres psql -U textbook -c "\l"

# Verify textbook_db exists
docker-compose -f docker-compose.dev.yml exec postgres psql -U textbook -c "SELECT version();"
```

**Expected**:
- `pg_isready` returns "accepting connections"
- `textbook_db` appears in database list
- PostgreSQL version displayed

### 2.2 Run Migrations

```bash
# Run Alembic migrations
docker-compose -f docker-compose.dev.yml exec backend alembic upgrade head

# Verify tables created
docker-compose -f docker-compose.dev.yml exec postgres psql -U textbook -d textbook_db -c "\dt"
```

**Expected**: 5 tables created:
- chapters
- labs
- assessments
- learner_profiles
- assessment_results

### 2.3 Seed Sample Data

```bash
# Generate sample data
docker-compose -f docker-compose.dev.yml exec backend python scripts/generate_sample_data.py

# Verify learner profiles created (should show 8)
docker-compose -f docker-compose.dev.yml exec postgres psql -U textbook -d textbook_db -c "SELECT COUNT(*) FROM learner_profiles;"

# Verify assessments created
docker-compose -f docker-compose.dev.yml exec postgres psql -U textbook -d textbook_db -c "SELECT COUNT(*) FROM assessment_results;"
```

**Expected**:
- 8 learner profiles created
- Multiple assessment results created
- Success messages in output

## Phase 3: Backend API Validation

### 3.1 Health Endpoint

```bash
# Backend health check
curl http://localhost:8000/health

# Should return 200 OK
curl -I http://localhost:8000/health
```

**Expected**: `{"status":"healthy"}` or similar response.

### 3.2 API Documentation

```bash
# Test OpenAPI docs endpoint
curl http://localhost:8000/docs

# Test ReDoc endpoint
curl http://localhost:8000/redoc
```

**Expected**: HTML documentation pages returned (status 200).

### 3.3 Learner Profile API

```bash
# Get learner profile (alice_undergrad_cs)
curl http://localhost:8000/api/learner-profile/alice_undergrad_cs

# Expected output:
# {
#   "learner_id": "alice_undergrad_cs",
#   "python_score": 8,
#   "ml_score": 6,
#   "robotics_score": 3,
#   "ros_score": 2,
#   "difficulty_level": "BEGINNER"
# }
```

**Expected**: JSON profile data for alice_undergrad_cs.

### 3.4 Assessment Results API

```bash
# Get assessment results for alice
curl http://localhost:8000/api/assessments/results/alice_undergrad_cs

# Should return array of assessment submissions
```

**Expected**: JSON array with assessment results.

### 3.5 RAG Chat API

```bash
# Test chat endpoint
curl -X POST http://localhost:8000/api/chat \
  -H "Content-Type: application/json" \
  -d '{
    "learner_id": "alice_undergrad_cs",
    "message": "What is forward kinematics?"
  }'
```

**Expected**: JSON response with chatbot message and context.

## Phase 4: Frontend Validation

### 4.1 Homepage Access

```bash
# Test frontend homepage
curl http://localhost:3000

# Check status code
curl -I http://localhost:3000
```

**Expected**: HTML page returned (status 200).

### 4.2 Dashboard Page

```bash
# Test dashboard
curl http://localhost:3000/dashboard

# Test chat page
curl http://localhost:3000/chat
```

**Expected**: Both pages return HTML (status 200).

### 4.3 Static Assets

```bash
# Test CSS loading
curl http://localhost:3000/assets/css/styles.css

# Test JavaScript loading
curl http://localhost:3000/assets/js/runtime.js
```

**Expected**: Assets load successfully (status 200).

### 4.4 Chapter Content

```bash
# Test Chapter 1 content
curl http://localhost:3000/docs/chapter-1

# Test Chapter 1 lab
curl http://localhost:3000/docs/chapter-1/lab
```

**Expected**: Chapter content pages return HTML.

## Phase 5: Integration Validation

### 5.1 Frontend-Backend Communication

**Manual Test**:
1. Open http://localhost:3000/dashboard in browser
2. Click on learner profile (alice_undergrad_cs)
3. Verify learner data loads from backend API
4. Check browser console for errors (should be none)

**Expected**:
- Learner profile displays correctly
- Assessment results appear
- No console errors

### 5.2 RAG Chatbot Integration

**Manual Test**:
1. Open http://localhost:3000/chat in browser
2. Enter message: "What is forward kinematics?"
3. Submit and wait for response
4. Verify response includes relevant content from Chapter 1

**Expected**:
- Chat UI responsive
- Message sent successfully
- Response received with context
- Conversation history persists

### 5.3 Personalization Toggle

**Manual Test**:
1. Open http://localhost:3000/docs/chapter-1
2. Select learner: alice_undergrad_cs (Beginner)
3. Toggle difficulty to Intermediate
4. Verify content adjusts (more technical)
5. Toggle to Advanced
6. Verify content further adjusts

**Expected**:
- Toggle switches difficulty levels
- Content changes appropriately
- No errors in console

## Phase 6: Vector Database Validation

### 6.1 Qdrant Health

```bash
# Check Qdrant health
curl http://localhost:6333/health

# List collections
curl http://localhost:6333/collections
```

**Expected**: Health check passes, collections listed.

### 6.2 Qdrant Dashboard

**Manual Test**:
1. Open http://localhost:6333/dashboard in browser
2. Navigate to collections
3. Verify "textbook_chapters" collection exists
4. Check vector count (should be > 0 if indexed)

**Expected**: Dashboard accessible, collections visible.

## Phase 7: Redis Cache Validation

### 7.1 Redis Connection

```bash
# Test Redis connection
docker-compose -f docker-compose.dev.yml exec redis redis-cli ping

# Check Redis info
docker-compose -f docker-compose.dev.yml exec redis redis-cli info server
```

**Expected**: `PONG` response, Redis version info.

## Phase 8: Full Test Suite

### 8.1 Automated Tests

```bash
# Run full test suite
docker-compose -f docker-compose.dev.yml exec backend python scripts/test_fullstack.py
```

**Expected Output**:
```
============================================================
FULL-STACK TEST SUITE
============================================================

Testing Backend API (8 tests)
------------------------------
Testing: Backend health endpoint ... PASS
Testing: OpenAPI docs endpoint ... PASS
Testing: Learner profile retrieval ... PASS
Testing: Assessment results retrieval ... PASS
Testing: Assessment submission ... PASS
Testing: RAG chat endpoint ... PASS
Testing: Invalid learner ID handling ... PASS
Testing: Database connectivity ... PASS

Testing Frontend Pages (4 tests)
---------------------------------
Testing: Homepage accessibility ... PASS
Testing: Dashboard page ... PASS
Testing: Chat page ... PASS
Testing: Chapter 1 content ... PASS

Testing Integration (1 test)
-----------------------------
Testing: Frontend-Backend API calls ... PASS

Testing Lab Environment (12 tests)
-----------------------------------
[Lab tests output...]

============================================================
TEST SUMMARY
============================================================
Tests Run: 25
Passed: 25
Failed: 0
Warnings: 0

Success Rate: 100%
============================================================
```

**Expected**: All 25 tests pass with 0 failures.

## Phase 9: Log Validation

### 9.1 Check for Errors

```bash
# Backend logs (should have no ERROR lines)
docker-compose -f docker-compose.dev.yml logs backend | grep ERROR

# Frontend logs (should have no errors)
docker-compose -f docker-compose.dev.yml logs frontend | grep -i error

# Database logs
docker-compose -f docker-compose.dev.yml logs postgres | grep ERROR
```

**Expected**: No critical errors in logs.

### 9.2 Monitor Real-time Logs

```bash
# Watch all logs
docker-compose -f docker-compose.dev.yml logs -f
```

**Expected**: Services logging normal operation, no repeated errors.

## Phase 10: Performance Validation

### 10.1 Response Times

```bash
# Measure backend response time
time curl http://localhost:8000/health

# Measure learner profile API
time curl http://localhost:8000/api/learner-profile/alice_undergrad_cs

# Measure frontend response
time curl http://localhost:3000
```

**Expected**:
- Health endpoint: < 100ms
- API calls: < 500ms
- Frontend: < 1s (first load), < 200ms (cached)

### 10.2 Concurrent Requests

```bash
# Test with 10 concurrent requests
for i in {1..10}; do
  curl http://localhost:8000/api/learner-profile/alice_undergrad_cs &
done
wait

# All should succeed
```

**Expected**: All 10 requests complete successfully.

## Troubleshooting Decision Tree

### Issue: Services Won't Start

1. Check Docker daemon: `docker ps`
2. Check logs: `docker-compose -f docker-compose.dev.yml logs`
3. Try rebuild: `docker-compose -f docker-compose.dev.yml up -d --build`

### Issue: Database Migrations Fail

1. Check PostgreSQL is running: `docker-compose -f docker-compose.dev.yml ps postgres`
2. Verify DATABASE_URL in .env matches docker-compose.dev.yml
3. Try manual connection: `docker-compose -f docker-compose.dev.yml exec postgres psql -U textbook -d textbook_db`
4. Reset database: `docker-compose -f docker-compose.dev.yml down -v && docker-compose -f docker-compose.dev.yml up -d`

### Issue: Backend API Not Responding

1. Check backend logs: `docker-compose -f docker-compose.dev.yml logs backend`
2. Verify backend is healthy: `docker-compose -f docker-compose.dev.yml ps backend`
3. Check API keys in .env are set
4. Restart backend: `docker-compose -f docker-compose.dev.yml restart backend`

### Issue: Frontend Not Loading

1. Check frontend logs: `docker-compose -f docker-compose.dev.yml logs frontend`
2. Verify port 3000 not in use: `lsof -i :3000`
3. Check REACT_APP_API_URL in docker-compose.dev.yml
4. Rebuild frontend: `docker-compose -f docker-compose.dev.yml up -d --build frontend`

### Issue: RAG Chat Not Working

1. Verify Qdrant is running: `curl http://localhost:6333/health`
2. Check API keys (OPENAI_API_KEY, CLAUDE_API_KEY) in .env
3. Verify backend can connect to Qdrant: check backend logs
4. Test embedding endpoint: `curl http://localhost:8000/api/embeddings/test`

## Success Criteria

Your local development environment is fully validated when:

- ✅ All 6 Docker services are running and healthy
- ✅ Database migrations completed (5 tables created)
- ✅ Sample data loaded (8 learners, multiple assessments)
- ✅ Backend API responds to all endpoints (health, learner-profile, assessments, chat)
- ✅ Frontend loads all pages (homepage, dashboard, chat, chapter-1)
- ✅ Frontend-backend integration works (dashboard loads data from API)
- ✅ RAG chatbot responds to queries with context
- ✅ Personalization toggle changes content appropriately
- ✅ All 25 automated tests pass
- ✅ No errors in service logs
- ✅ Response times within acceptable ranges

## Next Steps After Validation

1. **Explore Content**: Browse Chapter 1 at http://localhost:3000/docs/chapter-1
2. **Try Labs**: Attempt lab exercises with Docker environment
3. **Test Assessments**: Submit quiz answers via API
4. **Develop Features**: Make changes to backend/frontend code (hot-reload enabled)
5. **Run Tests**: Execute test suite after changes
6. **Prepare for Production**: Review `DEPLOYMENT_GUIDE.md` for production setup

## Validation Script

For automated validation, run:

```bash
# One-command validation
bash scripts/validate_setup.sh
```

This script will execute all validation steps and report results.
