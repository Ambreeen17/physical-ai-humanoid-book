#!/bin/bash
# Production Deployment Script for AI-Native Robotics Textbook Platform

set -e  # Exit on any error

echo "============================================================"
echo "AI-Native Robotics Textbook Platform - Production Deployment"
echo "============================================================"

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check prerequisites
echo "Checking prerequisites..."

if ! command_exists git; then
    echo "❌ git is not installed. Please install git first."
    exit 1
fi

if ! command_exists python3; then
    echo "❌ python3 is not installed. Please install Python 3.8+ first."
    exit 1
fi

if ! command_exists npm; then
    echo "❌ npm is not installed. Please install Node.js first."
    exit 1
fi

echo "✅ All prerequisites met"

# Function to deploy backend
deploy_backend() {
    echo "Deploying backend..."
    
    cd backend
    
    # Install dependencies
    echo "Installing backend dependencies..."
    pip install -r requirements.txt || {
        echo "❌ Failed to install backend dependencies"
        exit 1
    }
    
    # Run database migrations
    echo "Running database migrations..."
    python -m alembic upgrade head || {
        echo "❌ Failed to run database migrations"
        exit 1
    }
    
    # Seed assessments
    echo "Seeding assessments..."
    python scripts/seed_assessments.py || {
        echo "❌ Failed to seed assessments"
        exit 1
    }
    
    cd ..
    echo "✅ Backend deployed successfully"
}

# Function to deploy frontend
deploy_frontend() {
    echo "Deploying frontend..."
    
    cd frontend
    
    # Install dependencies
    echo "Installing frontend dependencies..."
    npm install || {
        echo "❌ Failed to install frontend dependencies"
        exit 1
    }
    
    # Build frontend
    echo "Building frontend..."
    npm run build || {
        echo "❌ Failed to build frontend"
        exit 1
    }
    
    cd ..
    echo "✅ Frontend deployed successfully"
}

# Function to upload RAG chunks
upload_rag_chunks() {
    echo "Uploading RAG chunks to Qdrant..."
    
    for i in {1..16}; do
        chunk_dir="specs/1-book-curriculum/chapters/chapter-$i"
        upload_script="$chunk_dir/upload_script.py"
        
        if [ -f "$upload_script" ]; then
            echo "Uploading chunks for Chapter $i..."
            cd "$chunk_dir"
            python upload_script.py || {
                echo "⚠️  Failed to upload chunks for Chapter $i (this may be expected if Qdrant is not running)"
            }
            cd ../../../..
        else
            echo "⚠️  Upload script not found for Chapter $i"
        fi
    done
    
    echo "✅ RAG chunks upload process completed"
}

# Function to run validation tests
run_validation_tests() {
    echo "Running validation tests..."
    
    # Run the validation script
    python scripts/final_validation.py || {
        echo "❌ Validation tests failed"
        exit 1
    }
    
    echo "✅ All validation tests passed"
}

# Main deployment process
echo "Starting deployment process..."

# 1. Run validation tests first
run_validation_tests

# 2. Deploy backend
deploy_backend

# 3. Deploy frontend
deploy_frontend

# 4. Upload RAG chunks
upload_rag_chunks

# 5. Final verification
echo "Performing final verification..."

# Check if all chapter files exist in frontend
missing_chapters=0
for i in {1..16}; do
    if [ ! -f "frontend/docs/chapter-$i.md" ]; then
        echo "❌ Chapter $i not found in frontend"
        ((missing_chapters++))
    fi
done

if [ $missing_chapters -eq 0 ]; then
    echo "✅ All 16 chapters found in frontend"
else
    echo "❌ $missing_chapters chapters missing from frontend"
    exit 1
fi

# Check if all chapter specs exist
missing_specs=0
for i in {1..16}; do
    if [ ! -d "specs/1-book-curriculum/chapters/chapter-$i" ]; then
        echo "❌ Chapter $i specs not found"
        ((missing_specs++))
    fi
done

if [ $missing_specs -eq 0 ]; then
    echo "✅ All 16 chapter specs found"
else
    echo "❌ $missing_specs chapter specs missing"
    exit 1
fi

echo "============================================================"
echo "DEPLOYMENT SUCCESSFUL!"
echo "============================================================"
echo ""
echo "Platform components:"
echo "- Backend: Available at configured port (typically :8000)"
echo "- Frontend: Available at configured port (typically :3000)"
echo "- API Documentation: /docs"
echo "- Dashboard: /dashboard"
echo "- Chat: /chat"
echo ""
echo "Next steps:"
echo "1. Configure your domain names"
echo "2. Set up SSL certificates"
echo "3. Configure monitoring and logging"
echo "4. Begin user acceptance testing"
echo ""
echo "Congratulations! The AI-Native Robotics Textbook platform is now deployed and ready for production use."
echo "============================================================"