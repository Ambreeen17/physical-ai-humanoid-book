-- AI-Native Robotics Textbook Database Schema
-- PostgreSQL 14+

-- User profiles table (hardware, background, preferences)
CREATE TABLE IF NOT EXISTS user_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,

    -- Software Background
    programming_level VARCHAR(20) DEFAULT 'beginner' CHECK (programming_level IN ('beginner','intermediate','advanced')),
    ai_experience VARCHAR(20) DEFAULT 'none' CHECK (ai_experience IN ('none','basic','applied')),
    robotics_experience VARCHAR(20) DEFAULT 'none' CHECK (robotics_experience IN ('none','basic','advanced')),

    -- Hardware Access - NVIDIA RTX
    has_rtx BOOLEAN DEFAULT FALSE,
    gpu_model VARCHAR(100),
    rtx_vram_gb VARCHAR(50),

    -- Hardware Access - NVIDIA Jetson
    has_jetson BOOLEAN DEFAULT FALSE,
    jetson_model VARCHAR(100),

    -- Physical Robotics Hardware
    has_robot BOOLEAN DEFAULT FALSE,
    robot_type VARCHAR(100),
    robot_specs JSONB,

    -- User Preferences
    preferred_language VARCHAR(10) DEFAULT 'en',
    learning_goal TEXT,
    content_preferences JSONB,

    -- Metadata
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    last_login TIMESTAMP
);

-- Chapters table
CREATE TABLE IF NOT EXISTS chapters (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    number INTEGER NOT NULL UNIQUE,
    title VARCHAR(255) NOT NULL,
    topic VARCHAR(255) NOT NULL,
    learning_objectives TEXT[],
    prerequisites INTEGER[],
    content TEXT,
    status VARCHAR(50) DEFAULT 'draft',
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Labs table
CREATE TABLE IF NOT EXISTS labs (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id UUID NOT NULL REFERENCES chapters(id),
    title VARCHAR(255) NOT NULL,
    description TEXT,
    difficulty VARCHAR(50),
    estimated_duration INTEGER,
    tools TEXT[],
    docker_image VARCHAR(255),
    source_code_repo VARCHAR(255),
    expected_output TEXT,
    test_status VARCHAR(50),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Assessments table
CREATE TABLE IF NOT EXISTS assessments (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id UUID NOT NULL REFERENCES chapters(id),
    type VARCHAR(50),
    questions JSONB,
    rubric JSONB,
    difficulty VARCHAR(50),
    time_limit INTEGER,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Learner profiles table
CREATE TABLE IF NOT EXISTS learner_profiles (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    email VARCHAR(255) UNIQUE NOT NULL,
    software_background DECIMAL(3,1),
    robotics_background DECIMAL(3,1),
    ai_background DECIMAL(3,1),
    personalization_preferences JSONB,
    language VARCHAR(10) DEFAULT 'en',
    completed_chapters UUID[],
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Assessment results table
CREATE TABLE IF NOT EXISTS assessment_results (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    learner_id UUID NOT NULL REFERENCES learner_profiles(id),
    chapter_id UUID NOT NULL REFERENCES chapters(id),
    assessment_id UUID NOT NULL REFERENCES assessments(id),
    submission_date TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    answers JSONB,
    score DECIMAL(5,2),
    feedback TEXT,
    status VARCHAR(50),
    lab_output_url VARCHAR(255)
);

-- RAG chunks table
CREATE TABLE IF NOT EXISTS rag_chunks (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id UUID NOT NULL REFERENCES chapters(id),
    section_title VARCHAR(255),
    content TEXT NOT NULL,
    embedding VECTOR(1536),
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Agent execution records table
CREATE TABLE IF NOT EXISTS agent_execution_records (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id UUID REFERENCES chapters(id),
    agent_name VARCHAR(100) NOT NULL,
    status VARCHAR(50),
    started_at TIMESTAMP,
    completed_at TIMESTAMP,
    input_artifacts TEXT[],
    output_artifacts TEXT[],
    error_message TEXT,
    duration_seconds INTEGER,
    metadata JSONB,
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP
);

-- Indexes for performance
CREATE INDEX idx_user_profiles_email ON user_profiles(email);
CREATE INDEX idx_user_profiles_programming_level ON user_profiles(programming_level);
CREATE INDEX idx_user_profiles_robotics_experience ON user_profiles(robotics_experience);
CREATE INDEX idx_user_profiles_hardware ON user_profiles(has_rtx, has_jetson, has_robot);

CREATE INDEX idx_chapters_number ON chapters(number);
CREATE INDEX idx_chapters_status ON chapters(status);
CREATE INDEX idx_labs_chapter ON labs(chapter_id);
CREATE INDEX idx_assessments_chapter ON assessments(chapter_id);
CREATE INDEX idx_learner_email ON learner_profiles(email);
CREATE INDEX idx_assessment_results_learner ON assessment_results(learner_id);
CREATE INDEX idx_assessment_results_chapter ON assessment_results(chapter_id);
CREATE INDEX idx_rag_chunks_chapter ON rag_chunks(chapter_id);
CREATE INDEX idx_agent_records_chapter ON agent_execution_records(chapter_id);
CREATE INDEX idx_agent_records_status ON agent_execution_records(status);
