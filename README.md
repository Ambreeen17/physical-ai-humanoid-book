# AI-Native Robotics Textbook Platform

Welcome to the AI-Native Robotics Textbook Platform! This is a complete educational platform that teaches robotics using AI-native approaches, with 16 comprehensive chapters covering all aspects of modern robotics.

## 🚀 Project Overview

The AI-Native Robotics Textbook is a comprehensive educational platform featuring:

- **16 Complete Chapters**: From foundational concepts to advanced integration
- **AI-Native Features**: RAG chatbot, personalization engine, auto-grading
- **Hands-on Labs**: Docker-based ROS 2 environments for each chapter
- **Adaptive Learning**: Beginner/Intermediate/Advanced pathways
- **Bilingual Support**: English and Urdu content
- **Assessment Engine**: Auto-grading with detailed feedback

### Chapters Included

**Part I: Foundations**
- Chapter 1: Introduction to Physical AI
- Chapter 2: Kinematics & Dynamics
- Chapter 3: Sensors & Actuators
- Chapter 4: State Estimation

**Part II: Perception & Control**
- Chapter 5: Computer Vision for Robotics
- Chapter 6: Control Theory Fundamentals
- Chapter 7: Motion Planning
- Chapter 8: Manipulation & Grasping

**Part III: Planning & Learning**
- Chapter 9: Task & Motion Planning
- Chapter 10: Reinforcement Learning for Robotics
- Chapter 11: Imitation Learning
- Chapter 12: Vision-Language-Action Models

**Part IV: Integration & Deployment**
- Chapter 13: System Integration
- Chapter 14: Safety & Robustness
- Chapter 15: Deployment & Operations
- Chapter 16: Capstone Project

## 🛠️ Tech Stack

### Backend
- **Framework**: FastAPI
- **Database**: PostgreSQL with SQLAlchemy
- **Vector DB**: Qdrant for RAG functionality
- **Cache**: Redis
- **AI APIs**: OpenAI and Anthropic integration

### Frontend
- **Framework**: Docusaurus
- **UI Library**: React
- **Styling**: CSS Modules
- **i18n**: Built-in localization support

### DevOps
- **Containerization**: Docker & Docker Compose
- **Deployment**: Cloud-agnostic (Railway/Render + Vercel/Netlify)
- **CI/CD**: GitHub Actions

## 📦 Included Components

- **Complete Backend API**: 13+ endpoints for all functionality
- **Interactive Frontend**: Dashboard, chatbot, personalization
- **RAG System**: Vector search with source citations
- **Assessment Engine**: Auto-grading with rubrics
- **Lab Environments**: Docker-based ROS 2 setups
- **Personalization Engine**: Adaptive difficulty adjustment
- **Multi-language Support**: English/Urdu with glossaries
- **Comprehensive Documentation**: 8 major guides

## 🚀 Quick Start

### Prerequisites
- Python 3.8+
- Node.js 16+
- Docker & Docker Compose
- OpenAI API key
- Anthropic API key (for Claude)

### Local Development Setup

1. **Clone the repository**
   ```bash
   git clone <your-repo-url>
   cd ai-native-robotics-textbook
   ```

2. **Set up backend**
   ```bash
   cd backend
   pip install -r requirements.txt
   cp .env.example .env  # Then update with your API keys
   python -m alembic upgrade head
   uvicorn src.main:app --reload
   ```

3. **Set up frontend**
   ```bash
   cd frontend
   npm install
   npm start
   ```

4. **Access the application**
   - Frontend: http://localhost:3000
   - Backend API: http://localhost:8000
   - API Docs: http://localhost:8000/docs

## ☁️ Production Deployment

For production deployment, follow the detailed guide in [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md).

### Quick Deployment Steps:
1. Set up Supabase for PostgreSQL
2. Set up Qdrant Cloud for vector database
3. Deploy backend to Railway/Render
4. Deploy frontend to Vercel/Netlify
5. Run assessment seeding script
6. Upload RAG chunks to Qdrant

## 📚 Documentation

- [QUICKSTART.md](QUICKSTART.md) - Quick start guide
- [TESTING_GUIDE.md](TESTING_GUIDE.md) - Comprehensive testing guide
- [DEPLOYMENT_GUIDE.md](DEPLOYMENT_GUIDE.md) - Production deployment guide
- [AUTHENTICATION_GUIDE.md](AUTHENTICATION_GUIDE.md) - Authentication setup
- [UAT_PLAN.md](UAT_PLAN.md) - User acceptance testing plan

## 🤝 Contributing

We welcome contributions! Please see our [CONTRIBUTING.md](CONTRIBUTING.md) for details.

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🎯 Learning Outcomes

Upon completing this textbook, learners will be able to:

1. Understand and implement core robotics concepts
2. Develop AI-native robotic systems using RAG and personalization
3. Design and implement perception, planning, and control systems
4. Integrate multiple robotic subsystems
5. Deploy and operate robotic systems in real-world environments

## 📞 Support

For support, please open an issue in the repository or contact the maintainers.

---

**Platform Status**: ✅ PRODUCTION READY  
**Last Updated**: January 5, 2026  
**Version**: 1.0.0