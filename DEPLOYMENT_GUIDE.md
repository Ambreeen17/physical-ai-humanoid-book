# Production Deployment Guide

## Overview
This guide provides step-by-step instructions for deploying the AI-Native Robotics Textbook platform to production.

## Architecture Overview
```
┌─────────────────────────────────────────────────────────────┐
│                        Frontend                             │
│  Docusaurus + React (Vercel/Netlify)                      │
│  - Dashboard, Chat, Chapter Content                       │
│  - PersonalizationToggle, LearnerDashboard, RAGChatbot    │
└────────────────────┬────────────────────────────────────────┘
                     │ HTTP/REST
┌────────────────────▼────────────────────────────────────────┐
│                       Backend API                          │
│  FastAPI (Railway/Render)                                 │
│  - Learner Profiles, Assessments, Chat                    │
│  - RAG Service, Personalization Service                   │
└──┬──────────────┬──────────────┬──────────────┬────────────┘
   │              │              │              │
   │ PostgreSQL   │ Qdrant       │ Redis        │ OpenAI/Claude
   │ (Supabase)   │ (Qdrant Cloud)│ (Redis Cloud)│ APIs
   │              │              │              │
   ▼              ▼              ▼              ▼
[Relational]  [Vector DB]   [Cache]        [AI Models]
 - Learners    - Embeddings  - Sessions     - Embeddings
 - Assessments - Chapters    - Results      - Chat
```

## Prerequisites
- GitHub account
- Railway or Render account
- Supabase account
- Qdrant Cloud account
- OpenAI API key
- Anthropic API key (Claude)

## Step 1: Prepare Repository
1. Push all changes to your GitHub repository
2. Ensure all 16 chapters are in the `frontend/docs/` directory
3. Verify all backend files are in the `backend/` directory
4. Confirm all chapter specifications are in `specs/1-book-curriculum/chapters/`

## Step 2: Database Setup (Supabase)
1. Create a new project in Supabase
2. Note your Project URL and Database password
3. Create a new database user and password
4. Run the Alembic migrations to set up the schema:

```bash
# Install dependencies
pip install -r backend/requirements.txt

# Set environment variables
export DATABASE_URL="postgresql://username:password@project-name.supabase.co:5432/postgres"

# Run migrations
cd backend
python -m alembic upgrade head
```

## Step 3: Vector Database Setup (Qdrant Cloud)
1. Create a new cluster in Qdrant Cloud
2. Note your API key and cluster URL
3. Collections will be created automatically by the RAG indexing agents

## Step 4: Deploy Backend (Railway/Render)

### Option A: Railway Deployment
1. Install Railway CLI:
```bash
npm install -g @railway/cli
```

2. Login to Railway:
```bash
railway login
```

3. Link your project:
```bash
railway init
```

4. Set environment variables:
```bash
railway vars set DATABASE_URL="postgresql://..."
railway vars set QDRANT_URL="https://..."
railway vars set QDRANT_API_KEY="..."
railway vars set OPENAI_API_KEY="..."
railway vars set CLAUDE_API_KEY="..."
railway vars set REDIS_URL="..."
```

5. Deploy:
```bash
railway up
```

### Option B: Render Deployment
1. Create a new Web Service on Render
2. Connect to your GitHub repository
3. Set runtime to Python
4. Set build command: `pip install -r requirements.txt`
5. Set start command: `uvicorn src.main:app --host 0.0.0.0 --port $PORT`
6. Add environment variables in Render dashboard

## Step 5: Deploy Frontend (Vercel/Netlify)

### Option A: Vercel Deployment
1. Install Vercel CLI:
```bash
npm install -g vercel
```

2. Login to Vercel:
```bash
vercel login
```

3. Build the frontend:
```bash
cd frontend
npm install
npm run build
```

4. Deploy:
```bash
vercel --prod
```

### Option B: Netlify Deployment
1. Create a new site in Netlify
2. Connect to your GitHub repository
3. Set build command: `npm run build`
4. Set publish directory: `build/`
5. Set environment variables:
   - `REACT_APP_API_URL`: Your backend URL

## Step 6: Seed Assessments
After both backend and frontend are deployed:

1. Run the assessment seeding script:
```bash
python backend/scripts/seed_assessments.py
```

2. This will populate the database with all chapter assessments

## Step 7: Upload RAG Chunks
For each chapter, run the upload script to populate Qdrant:

```bash
# For each chapter 1-16
cd specs/1-book-curriculum/chapters/chapter-1
python upload_script.py

cd ../chapter-2
python upload_script.py

# ... repeat for all chapters
```

## Step 8: Verification
1. Visit your frontend URL
2. Verify all 16 chapters are accessible
3. Test the RAG chatbot functionality
4. Verify personalization toggle works
5. Test assessment submission
6. Verify lab environments are accessible

## Environment Variables Required

### Backend (.env)
```env
DATABASE_URL=postgresql://username:password@host:port/database
QDRANT_URL=https://your-cluster.qdrant.tech:6333
QDRANT_API_KEY=your-api-key
OPENAI_API_KEY=your-openai-api-key
CLAUDE_API_KEY=your-claude-api-key
REDIS_URL=redis://host:port
SECRET_KEY=your-secret-key
ALGORITHM=HS256
ACCESS_TOKEN_EXPIRE_MINUTES=30
CORS_ORIGINS=["https://your-frontend-url.com"]
ENVIRONMENT=production
```

### Frontend (.env)
```env
REACT_APP_API_URL=https://your-backend-url.onrender.com
```

## Monitoring & Maintenance

### Health Checks
- Backend: `GET /health` endpoint
- Frontend: Status page at `/status`
- Database: Supabase dashboard
- Vector DB: Qdrant Cloud dashboard

### Logs
- Backend: Railway/Render logs
- Frontend: Vercel/Netlify logs
- Database: Supabase logs
- Errors: Set up Sentry for error tracking

### Performance
- Database: Monitor query performance in Supabase
- API: Monitor response times in Railway/Render
- Frontend: Performance metrics in Vercel/Netlify
- Vector DB: Qdrant performance metrics

## Scaling Recommendations

### Traffic Growth
- Small (<100 users): Free tiers of all services
- Medium (100-1000 users): 
  - Supabase Pro ($25/month)
  - Qdrant Cloud Basic ($50/month)
  - Railway Pro ($20/month)
- Large (>1000 users):
  - Dedicated PostgreSQL instance
  - Qdrant dedicated cluster
  - Load balancer setup

### Cost Estimation
- Development: $0 (free tiers)
- Small Production: $100-200/month
- Medium Production: $300-500/month
- Large Production: $800-1500/month

## Rollback Procedure
1. If issues arise, revert to previous deployment in Railway/Render
2. For database issues, use Supabase point-in-time recovery
3. For frontend issues, redeploy previous build

## Troubleshooting

### Common Issues
1. **API Timeout**: Increase timeout settings or reduce input size
2. **Database Connection**: Verify DATABASE_URL format and credentials
3. **Qdrant Connection**: Check API key and cluster URL
4. **Frontend Backend Communication**: Verify CORS settings and API URL

### Support Resources
- Backend API documentation: `/docs`
- Frontend developer tools: Browser console
- Database: Supabase SQL editor
- Vector DB: Qdrant dashboard
- Logs: Platform-specific logging systems

---

**Deployment Complete**: Your AI-Native Robotics Textbook platform is now live and ready for users!