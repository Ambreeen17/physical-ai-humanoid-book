# Quick Start Guide

Get the AI-Native Robotics Textbook platform running locally in under 10 minutes.

## Prerequisites

- Docker & Docker Compose (v2.0+)
- Git
- OpenAI API Key (for embeddings)
- Claude API Key (for chat)
- 8GB RAM minimum, 16GB recommended

## Quick Setup (3 Steps)

### 1. Clone and Configure

```bash
# Clone repository
git clone <repository-url>
cd boook

# Copy environment template
cp .env.example .env

# Edit .env and add your API keys
# Required: OPENAI_API_KEY, CLAUDE_API_KEY
# Required: POSTGRES_PASSWORD, REDIS_PASSWORD, SECRET_KEY
```

### 2. Start Development Environment

```bash
# Start all services (first run takes 5-10 minutes)
docker-compose -f docker-compose.dev.yml up -d

# Wait for services to be healthy
docker-compose -f docker-compose.dev.yml ps

# Check logs if needed
docker-compose -f docker-compose.dev.yml logs -f backend
```

### 3. Initialize Database

```bash
# Run database migrations
docker-compose -f docker-compose.dev.yml exec backend alembic upgrade head

# Seed with sample data (8 learners, assessments)
docker-compose -f docker-compose.dev.yml exec backend python scripts/generate_sample_data.py
```

## Access the Platform

- **Frontend (Docusaurus)**: http://localhost:3000
- **Backend API Docs**: http://localhost:8000/docs
- **PgAdmin (Database UI)**: http://localhost:5050
  - Email: admin@textbook.local
  - Password: admin
- **Qdrant Dashboard**: http://localhost:6333/dashboard

## Verify Installation

```bash
# Test backend health
curl http://localhost:8000/health

# Test frontend
curl http://localhost:3000

# View sample learner profiles
curl http://localhost:8000/api/learner-profile/alice_undergrad_cs
```

## Sample Learners

The system comes with 8 pre-configured learner personas:

1. **alice_undergrad_cs** - Undergrad CS student (beginner robotics)
2. **bob_engineer_ml** - ML Engineer (intermediate robotics)
3. **carol_phd_robotics** - PhD Robotics (advanced)
4. **dave_bootcamp_grad** - Bootcamp graduate (beginner)
5. **eve_mecheng_student** - Mechanical engineering student
6. **frank_career_changer** - Career changer (beginner)
7. **grace_researcher** - Experienced researcher (advanced)
8. **henry_hobbyist** - Hobbyist (beginner)

## Development Workflow

### Hot Reload (Code Changes Reflect Immediately)

Both backend and frontend support hot-reload in development mode:

```bash
# Edit backend code in ./backend/src/**
# Backend automatically reloads

# Edit frontend code in ./frontend/src/**
# Frontend automatically reloads
```

### View Logs

```bash
# All services
docker-compose -f docker-compose.dev.yml logs -f

# Specific service
docker-compose -f docker-compose.dev.yml logs -f backend
docker-compose -f docker-compose.dev.yml logs -f frontend
```

### Restart Services

```bash
# Restart all
docker-compose -f docker-compose.dev.yml restart

# Restart specific service
docker-compose -f docker-compose.dev.yml restart backend
```

### Stop Services

```bash
# Stop (keeps data)
docker-compose -f docker-compose.dev.yml down

# Stop and remove data
docker-compose -f docker-compose.dev.yml down -v
```

## Testing

### Run Full Test Suite

```bash
# All tests (backend + frontend + integration + lab)
docker-compose -f docker-compose.dev.yml exec backend python scripts/test_fullstack.py

# Backend only
docker-compose -f docker-compose.dev.yml exec backend pytest

# Frontend only (in separate terminal)
cd frontend && npm test
```

### Manual Testing

1. **Test Personalization**:
   - Go to http://localhost:3000/dashboard
   - Click learner: alice_undergrad_cs
   - Toggle difficulty: Beginner → Intermediate → Advanced
   - Observe content changes

2. **Test RAG Chatbot**:
   - Go to http://localhost:3000/chat
   - Ask: "What is forward kinematics?"
   - Verify response includes Chapter 1 context

3. **Test Assessment Submission**:
   - POST to http://localhost:8000/api/assessments/submit
   - Use sample assessment_id and learner_profile_id
   - Verify score and feedback

## Production Deployment

### Build Production Images

```bash
# Build optimized images
docker-compose build

# Test production build locally
docker-compose up -d
```

### Environment Variables for Production

```env
ENVIRONMENT=production
POSTGRES_PASSWORD=<strong-password>
REDIS_PASSWORD=<strong-password>
SECRET_KEY=<long-random-string>
CORS_ORIGINS=["https://yourdomain.com"]
```

See `DEPLOYMENT_GUIDE.md` for full production setup.

## Troubleshooting

### Services Not Starting

```bash
# Check service status
docker-compose -f docker-compose.dev.yml ps

# Check specific service logs
docker-compose -f docker-compose.dev.yml logs backend

# Restart problematic service
docker-compose -f docker-compose.dev.yml restart backend
```

### Database Connection Errors

```bash
# Verify PostgreSQL is healthy
docker-compose -f docker-compose.dev.yml exec postgres pg_isready -U textbook

# Check database exists
docker-compose -f docker-compose.dev.yml exec postgres psql -U textbook -c "\l"

# Recreate database (WARNING: deletes data)
docker-compose -f docker-compose.dev.yml down -v
docker-compose -f docker-compose.dev.yml up -d
```

### Port Already in Use

```bash
# Change ports in .env file
POSTGRES_PORT=5433  # Instead of 5432
BACKEND_PORT=8001   # Instead of 8000
FRONTEND_PORT=3001  # Instead of 3000

# Restart with new ports
docker-compose -f docker-compose.dev.yml down
docker-compose -f docker-compose.dev.yml up -d
```

### API Keys Not Working

```bash
# Verify keys are loaded
docker-compose -f docker-compose.dev.yml exec backend env | grep API_KEY

# Re-source environment
docker-compose -f docker-compose.dev.yml down
docker-compose -f docker-compose.dev.yml up -d
```

## Next Steps

1. **Explore the Curriculum**: http://localhost:3000/docs/chapter-1
2. **Try Lab Exercises**: http://localhost:3000/docs/chapter-1/lab
3. **Test Assessments**: http://localhost:3000/docs/chapter-1/assessments
4. **Use RAG Chat**: http://localhost:3000/chat
5. **Review Testing Guide**: `TESTING_GUIDE.md`
6. **Production Deployment**: `DEPLOYMENT_GUIDE.md`

## Additional Resources

- **API Documentation**: http://localhost:8000/docs
- **Database Schema**: `backend/alembic/versions/001_initial_schema.py`
- **Frontend Components**: `frontend/src/components/`
- **Sample Data**: `backend/scripts/generate_sample_data.py`

## Need Help?

- Check logs: `docker-compose -f docker-compose.dev.yml logs -f`
- Read `TESTING_GUIDE.md` for detailed troubleshooting
- Review architecture: `specs/1-book-curriculum/ARCHITECTURE.md`
