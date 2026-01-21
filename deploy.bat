@echo off
REM Production Deployment Script for AI-Native Robotics Textbook Platform
REM Windows Batch Version

echo ============================================================
echo AI-Native Robotics Textbook Platform - Production Deployment
echo ============================================================

REM Check prerequisites
echo Checking prerequisites...

python --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Python is not installed or not in PATH. Please install Python 3.8+ first.
    pause
    exit /b 1
)

node --version >nul 2>&1
if errorlevel 1 (
    echo ❌ Node.js is not installed or not in PATH. Please install Node.js first.
    pause
    exit /b 1
)

echo ✅ All prerequisites met

REM Function to deploy backend
:deploy_backend
echo Deploying backend...

cd backend

echo Installing backend dependencies...
pip install -r requirements.txt
if errorlevel 1 (
    echo ❌ Failed to install backend dependencies
    pause
    exit /b 1
)

echo Running database migrations...
python -m alembic upgrade head
if errorlevel 1 (
    echo ❌ Failed to run database migrations
    pause
    exit /b 1
)

echo Seeding assessments...
python scripts/seed_assessments.py
if errorlevel 1 (
    echo ❌ Failed to seed assessments
    pause
    exit /b 1
)

cd ..
echo ✅ Backend deployed successfully
goto :eof

REM Function to deploy frontend
:deploy_frontend
echo Deploying frontend...

cd frontend

echo Installing frontend dependencies...
npm install
if errorlevel 1 (
    echo ❌ Failed to install frontend dependencies
    pause
    exit /b 1
)

echo Building frontend...
npm run build
if errorlevel 1 (
    echo ❌ Failed to build frontend
    pause
    exit /b 1
)

cd ..
echo ✅ Frontend deployed successfully
goto :eof

REM Function to upload RAG chunks
:upload_rag_chunks
echo Uploading RAG chunks to Qdrant...

for /L %%i in (1,1,16) do (
    set "chunk_dir=specs\1-book-curriculum\chapters\chapter-%%i"
    set "upload_script=!chunk_dir!\upload_script.py"
    
    if exist "!upload_script!" (
        echo Uploading chunks for Chapter %%i...
        cd "!chunk_dir!"
        python upload_script.py
        if errorlevel 1 (
            echo ⚠️  Failed to upload chunks for Chapter %%i (this may be expected if Qdrant is not running)
        )
        cd ..\..\..
    ) else (
        echo ⚠️  Upload script not found for Chapter %%i
    )
)

echo ✅ RAG chunks upload process completed
goto :eof

REM Function to run validation tests
:run_validation_tests
echo Running validation tests...

python scripts\final_validation.py
if errorlevel 1 (
    echo ❌ Validation tests failed
    pause
    exit /b 1
)

echo ✅ All validation tests passed
goto :eof

REM Main deployment process starts here
echo Starting deployment process...

REM 1. Run validation tests first
call :run_validation_tests

REM 2. Deploy backend
call :deploy_backend

REM 3. Deploy frontend
call :deploy_frontend

REM 4. Upload RAG chunks
call :upload_rag_chunks

REM 5. Final verification
echo Performing final verification...

REM Check if all chapter files exist in frontend
set missing_chapters=0
for /L %%i in (1,1,16) do (
    if not exist "frontend\docs\chapter-%%i.md" (
        echo ❌ Chapter %%i not found in frontend
        set /a missing_chapters=!missing_chapters!+1
    )
)

if !missing_chapters! equ 0 (
    echo ✅ All 16 chapters found in frontend
) else (
    echo ❌ !missing_chapters! chapters missing from frontend
    pause
    exit /b 1
)

REM Check if all chapter specs exist
set missing_specs=0
for /L %%i in (1,1,16) do (
    if not exist "specs\1-book-curriculum\chapters\chapter-%%i" (
        echo ❌ Chapter %%i specs not found
        set /a missing_specs=!missing_specs!+1
    )
)

if !missing_specs! equ 0 (
    echo ✅ All 16 chapter specs found
) else (
    echo ❌ !missing_specs! chapter specs missing
    pause
    exit /b 1
)

echo ============================================================
echo DEPLOYMENT SUCCESSFUL!
echo ============================================================
echo.
echo Platform components:
echo - Backend: Available at configured port (typically :8000)
echo - Frontend: Available at configured port (typically :3000)
echo - API Documentation: /docs
echo - Dashboard: /dashboard
echo - Chat: /chat
echo.
echo Next steps:
echo 1. Configure your domain names
echo 2. Set up SSL certificates
echo 3. Configure monitoring and logging
echo 4. Begin user acceptance testing
echo.
echo Congratulations! The AI-Native Robotics Textbook platform is now deployed and ready for production use.
echo ============================================================
pause