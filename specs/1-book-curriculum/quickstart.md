# Quickstart: Set Up Local Development Environment

This guide helps you set up the backend, frontend, and ROS 2 lab environment for the AI-native robotics textbook project.

## Prerequisites

- **OS**: Ubuntu 22.04 (or Docker Desktop on any OS)
- **Hardware**: 8GB+ RAM, 20GB+ free disk
- **Tools**: Git, Docker, Docker Compose, Python 3.9+, Node.js 18+

## Backend Setup (FastAPI + Postgres)

### 1. Clone Repo & Install Dependencies

```bash
git clone https://github.com/your-org/robotics-textbook.git
cd robotics-textbook/backend

# Create virtual environment
python3.9 -m venv venv
source venv/bin/activate  # On Windows: venv\Scripts\activate

# Install dependencies
pip install -r requirements.txt
```

### 2. Configure Environment

Create `.env` file in `backend/`:

```env
DATABASE_URL=postgresql://user:password@localhost:5432/textbook_db
OPENAI_API_KEY=sk-...
QDRANT_URL=https://your-instance.qdrant.io
QDRANT_API_KEY=...
CLAUDE_API_KEY=sk-...
```

### 3. Start Database (Docker)

```bash
docker-compose up -d postgres
docker exec textbook-postgres psql -U postgres -d textbook_db < schema.sql
```

### 4. Run FastAPI Server

```bash
uvicorn src.main:app --reload --port 8000
```

**API Docs**: Open http://localhost:8000/docs

---

## Frontend Setup (Docusaurus)

### 1. Install Dependencies

```bash
cd robotics-textbook/website
npm install
```

### 2. Start Development Server

```bash
npm run start
```

**Site**: Opens http://localhost:3000

### 3. Build for Production

```bash
npm run build
```

---

## ROS 2 Lab Development

### 1. Pull ROS 2 Humble Docker Image

```bash
docker pull ros:humble
```

### 2. Run Lab Template

```bash
cd robotics-textbook/specs/1-book-curriculum/chapters/chapter-3/lab

# Build Docker image for this lab
docker-compose build

# Run lab container
docker-compose up
```

### 3. Run a Gazebo Simulation Lab

```bash
docker-compose exec lab bash

# Inside container
ros2 launch sim_world empty.launch.py
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
```

---

## Integration Testing

### Run All Backend Tests

```bash
cd backend
pytest tests/
```

### Run Lab Validation Tests

```bash
cd specs/1-book-curriculum/chapters/chapter-3/lab
pytest tests/test_lab.py -v
```

---

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Docker daemon not running | `sudo systemctl start docker` (Linux) or start Docker Desktop |
| Postgres connection refused | Check `DATABASE_URL` in `.env`; verify postgres container running |
| ROS 2 command not found | Ensure Docker container has sourced `/opt/ros/humble/setup.bash` |
| Gazebo window not showing | Check `DISPLAY` variable; may need X11 forwarding on headless systems |
| API 502 Bad Gateway | Check FastAPI logs: `docker-compose logs backend` |

---

## Next Steps

- **Read**: `/docs/ARCHITECTURE.md` for system design
- **Contribute**: See `CONTRIBUTING.md` for agent orchestration guidelines
- **Deploy**: See `DEPLOY.md` for cloud deployment steps
