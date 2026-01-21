"""
Simplified backend for demonstration purposes.
This version removes the database dependency to allow basic API functionality.
"""
import os
from fastapi import FastAPI, Depends, HTTPException, status
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
from typing import List, Optional
import asyncio
import json
from datetime import datetime

# Create a lifespan context manager that doesn't require database
@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup
    print("AI-Native Robotics Textbook Backend Starting...")
    yield
    # Shutdown
    print("AI-Native Robotics Textbook Backend Shutting Down...")

# Initialize FastAPI app
app = FastAPI(
    title="AI-Native Robotics Textbook API",
    description="API for the AI-Native Robotics Textbook Platform",
    version="1.0.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, replace with specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Mock data for demonstration
MOCK_LEARNER_PROFILES = [
    {
        "id": "alice_undergrad_cs",
        "name": "Alice Johnson",
        "email": "alice@example.com",
        "programming_skills": 8,
        "ml_skills": 6,
        "robotics_experience": 3,
        "ros_experience": 2,
        "preferred_difficulty": "beginner",
        "learning_goals": ["kinematics", "control", "navigation"]
    }
]

MOCK_ASSESSMENTS = [
    {
        "id": 1,
        "chapter": 1,
        "title": "Introduction to Physical AI Quiz",
        "questions": [
            {
                "id": 1,
                "type": "multiple_choice",
                "question": "What makes embodied intelligence different from traditional AI?",
                "options": [
                    "A) It operates in continuous, real-time loops with physical world",
                    "B) It uses more computational power",
                    "C) It processes more data types",
                    "D) It requires more memory"
                ],
                "correct_answer": "A",
                "explanation": "Embodied intelligence operates in continuous sensorimotor loops with the physical world, unlike traditional AI which processes static inputs."
            }
        ],
        "max_score": 10
    }
]

MOCK_CHAT_HISTORY = []

@app.get("/")
async def root():
    return {
        "message": "Welcome to the AI-Native Robotics Textbook API",
        "status": "healthy",
        "version": "1.0.0",
        "documentation": "/docs"
    }

@app.get("/health")
async def health_check():
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "service": "textbook-api"
    }

@app.get("/api/learners")
async def get_learners():
    """Get all learner profiles (mock data)."""
    return MOCK_LEARNER_PROFILES

@app.get("/api/learners/{learner_id}")
async def get_learner(learner_id: str):
    """Get a specific learner profile (mock data)."""
    for learner in MOCK_LEARNER_PROFILES:
        if learner["id"] == learner_id:
            return learner
    raise HTTPException(status_code=404, detail="Learner not found")

@app.post("/api/learners")
async def create_learner(learner_data: dict):
    """Create a new learner profile (mock data)."""
    new_learner = {
        "id": learner_data.get("id", f"learner_{len(MOCK_LEARNER_PROFILES)+1}"),
        "name": learner_data.get("name", "New Learner"),
        "email": learner_data.get("email", ""),
        "programming_skills": learner_data.get("programming_skills", 5),
        "ml_skills": learner_data.get("ml_skills", 5),
        "robotics_experience": learner_data.get("robotics_experience", 1),
        "ros_experience": learner_data.get("ros_experience", 0),
        "preferred_difficulty": learner_data.get("preferred_difficulty", "beginner"),
        "learning_goals": learner_data.get("learning_goals", [])
    }
    MOCK_LEARNER_PROFILES.append(new_learner)
    return new_learner

@app.get("/api/assessments/chapter/{chapter_number}")
async def get_assessments(chapter_number: int):
    """Get assessments for a specific chapter (mock data)."""
    chapter_assessments = [a for a in MOCK_ASSESSMENTS if a["chapter"] == chapter_number]
    return chapter_assessments

@app.post("/api/assessments/submit")
async def submit_assessment(submission: dict):
    """Submit an assessment (mock data)."""
    # In a real implementation, this would validate answers and store results
    result = {
        "submission_id": f"sub_{len(MOCK_CHAT_HISTORY)+1}",
        "learner_id": submission.get("learner_id"),
        "chapter": submission.get("chapter"),
        "score": submission.get("score", 0),
        "max_score": submission.get("max_score", 10),
        "feedback": "Assessment submitted successfully (demo mode)"
    }
    return result

@app.post("/api/chat")
async def chat_with_bot(message_data: dict):
    """Chat with the RAG-enabled bot (mock data)."""
    global MOCK_CHAT_HISTORY
    
    user_message = message_data.get("message", "")
    learner_id = message_data.get("learner_id", "unknown")
    
    # Mock response based on keywords in the user message
    response = "I'm the AI tutor for the AI-Native Robotics Textbook. I can help explain concepts from any of the 16 chapters. What would you like to learn about?"
    
    if "kinematics" in user_message.lower():
        response = "Kinematics is the study of motion without considering the forces that cause it. Forward kinematics computes end-effector position from joint angles, while inverse kinematics computes joint angles from end-effector position."
    elif "dynamics" in user_message.lower():
        response = "Robot dynamics studies the relationship between forces acting on a robot and the resulting motion. It involves modeling mass, inertia, friction, and external forces to predict robot behavior."
    elif "chapter 1" in user_message.lower() or "introduction" in user_message.lower():
        response = "Chapter 1 introduces Physical AI - the integration of artificial intelligence with physical systems. It covers the sensorimotor loop, embodied cognition, and the sim-to-real gap challenges."
    elif "sensors" in user_message.lower():
        response = "Robotic sensors include LiDAR for distance measurement, cameras for vision, IMUs for orientation, force/torque sensors for interaction, and encoders for joint position feedback."
    elif "control" in user_message.lower():
        response = "Robot control involves designing algorithms to make robots behave as desired. Common approaches include PID control, state-space methods, and modern learning-based control."
    
    chat_entry = {
        "id": len(MOCK_CHAT_HISTORY) + 1,
        "learner_id": learner_id,
        "timestamp": datetime.utcnow().isoformat(),
        "user_message": user_message,
        "bot_response": response,
        "sources": ["Chapter 1: Introduction to Physical AI", "Chapter 2: Kinematics & Dynamics"]  # Mock sources
    }
    
    MOCK_CHAT_HISTORY.append(chat_entry)
    
    return {
        "response": response,
        "sources": chat_entry["sources"],
        "chat_id": chat_entry["id"]
    }

@app.get("/api/chat/history/{learner_id}")
async def get_chat_history(learner_id: str):
    """Get chat history for a learner (mock data)."""
    learner_history = [entry for entry in MOCK_CHAT_HISTORY if entry["learner_id"] == learner_id]
    return learner_history

@app.get("/api/chapters")
async def get_chapters():
    """Get list of all chapters (mock data)."""
    chapters = []
    for i in range(1, 17):
        title = ""
        if i == 1:
            title = "Introduction to Physical AI"
        elif i == 2:
            title = "Kinematics & Dynamics"
        elif i == 3:
            title = "Sensors & Actuators"
        elif i == 4:
            title = "State Estimation"
        elif i == 5:
            title = "Computer Vision for Robotics"
        elif i == 6:
            title = "Control Theory Fundamentals"
        elif i == 7:
            title = "Motion Planning"
        elif i == 8:
            title = "Manipulation & Grasping"
        elif i == 9:
            title = "Task & Motion Planning"
        elif i == 10:
            title = "Reinforcement Learning for Robotics"
        elif i == 11:
            title = "Imitation Learning"
        elif i == 12:
            title = "Vision-Language-Action Models"
        elif i == 13:
            title = "System Integration"
        elif i == 14:
            title = "Safety & Robustness"
        elif i == 15:
            title = "Deployment & Operations"
        elif i == 16:
            title = "Capstone Project"
            
        chapters.append({
            "number": i,
            "title": title,
            "sections": 7,  # Mock number of sections
            "estimated_time": f"{i*2}-{i*3} hours",  # Mock estimated time
            "difficulty": "multi-level"  # Beginner/Intermediate/Advanced
        })
    
    return chapters

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)