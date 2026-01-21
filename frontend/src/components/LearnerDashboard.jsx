import React, { useState, useEffect } from 'react';
import { useTranslation } from './TranslationProvider';
import { useAuth } from './AuthProvider';
import './LearnerDashboard.css';

// Demo data for when backend is not available
const DEMO_PROFILES = {
  'demo-learner-123': {
    learner_id: 'demo-learner-123',
    difficulty_level: 'Beginner',
    python_score: 7,
    ml_score: 5,
    robotics_score: 3,
    ros_score: 2,
    recommended_chapters: ['Chapter 1: Introduction to Physical AI', 'Chapter 2: Kinematics', 'Chapter 3: Dynamics'],
  },
  'alice_undergrad_cs': {
    learner_id: 'alice_undergrad_cs',
    difficulty_level: 'Beginner',
    python_score: 8,
    ml_score: 6,
    robotics_score: 3,
    ros_score: 2,
    recommended_chapters: ['Chapter 1: Introduction to Physical AI', 'Chapter 2: Kinematics'],
  },
  'bob_engineer_ml': {
    learner_id: 'bob_engineer_ml',
    difficulty_level: 'Intermediate',
    python_score: 9,
    ml_score: 8,
    robotics_score: 5,
    ros_score: 4,
    recommended_chapters: ['Chapter 3: Dynamics', 'Chapter 4: Motion Planning', 'Chapter 5: Computer Vision'],
  },
  'carol_phd_robotics': {
    learner_id: 'carol_phd_robotics',
    difficulty_level: 'Advanced',
    python_score: 9,
    ml_score: 9,
    robotics_score: 9,
    ros_score: 8,
    recommended_chapters: ['Chapter 10: Advanced Control', 'Chapter 14: Sim-to-Real', 'Chapter 16: Capstone'],
  },
};

const DEMO_RESULTS = [
  {
    id: 1,
    assessment_id: 'ch1-quiz-1',
    score: 8,
    max_score: 10,
    passed: true,
    submitted_at: new Date(Date.now() - 86400000 * 2).toISOString(),
    feedback: 'Great job understanding Physical AI fundamentals! Review the sensorimotor loop section.'
  },
  {
    id: 2,
    assessment_id: 'ch1-lab-1',
    score: 15,
    max_score: 20,
    passed: true,
    submitted_at: new Date(Date.now() - 86400000).toISOString(),
    feedback: 'Good work on the ROS 2 lab. Consider optimizing your control loop timing.'
  },
  {
    id: 3,
    assessment_id: 'ch2-quiz-1',
    score: 6,
    max_score: 10,
    passed: false,
    submitted_at: new Date().toISOString(),
    feedback: 'Review forward kinematics and DH parameters. Practice with the 2-link arm example.'
  },
];

/**
 * Learner Dashboard Component
 *
 * Displays:
 * - Learner profile (difficulty level, recommended chapters)
 * - Assessment results (score, pass/fail, feedback)
 * - Progress tracking across chapters
 * - Personalization settings toggle
 * - Demo mode when backend unavailable
 */
const LearnerDashboard = ({ learnerId: initialLearnerId }) => {
  const { user, isAuthenticated } = useAuth();
  const learnerId = isAuthenticated && user ? user.id : initialLearnerId;

  const [profile, setProfile] = useState(null);
  const [results, setResults] = useState([]);
  const [loading, setLoading] = useState(true);
  const [error, setError] = useState(null);

  const [isDemo, setIsDemo] = useState(false);

  // Fetch learner profile and assessment results
  useEffect(() => {
    const fetchDashboardData = async () => {
      // If not logged in, show demo mode immediately
      if (!isAuthenticated || !learnerId) {
        const demoProfile = DEMO_PROFILES['demo-learner-123'];
        setProfile(demoProfile);
        setResults(DEMO_RESULTS);
        setIsDemo(true);
        setLoading(false);
        return;
      }

      try {
        // Fetch learner profile - try API but gracefully fall back to demo
        const profileResponse = await fetch(`http://localhost:8000/api/learner-profile/${learnerId}`);
        if (profileResponse.ok) {
          const profileData = await profileResponse.json();
          setProfile(profileData);
        } else {
          // Use demo profile for this user
          setProfile({
            learner_id: learnerId,
            difficulty_level: 'Beginner',
            python_score: 5,
            ml_score: 3,
            robotics_score: 2,
            ros_score: 1,
            recommended_chapters: ['Chapter 1: Introduction to Physical AI', 'Chapter 2: Kinematics'],
          });
        }

        // Fetch assessment results - try API but gracefully fall back
        try {
          const resultsResponse = await fetch(`http://localhost:8000/api/assessments/results/${learnerId}`);
          if (resultsResponse.ok) {
            const resultsData = await resultsResponse.json();
            setResults(resultsData);
          } else {
            setResults(DEMO_RESULTS);
          }
        } catch {
          setResults(DEMO_RESULTS);
        }

        setLoading(false);
        setIsDemo(false);
      } catch (err) {
        console.error('Dashboard fetch error, using demo mode:', err);
        // Fallback to demo data
        const demoProfile = DEMO_PROFILES[learnerId] || DEMO_PROFILES['demo-learner-123'];
        setProfile(demoProfile);
        setResults(DEMO_RESULTS);
        setIsDemo(true);
        setLoading(false);
      }
    };

    fetchDashboardData();
  }, [learnerId, isAuthenticated]);

  // Calculate progress statistics
  const calculateProgress = () => {
    if (results.length === 0) return { totalAttempts: 0, passed: 0, averageScore: 0 };

    const totalAttempts = results.length;
    const passed = results.filter(r => r.passed).length;
    const totalScore = results.reduce((sum, r) => sum + (r.score / r.max_score) * 100, 0);
    const averageScore = totalScore / totalAttempts;

    return {
      totalAttempts,
      passed,
      averageScore: averageScore.toFixed(1)
    };
  };

  const progress = calculateProgress();
  const { t, isRTL } = useTranslation();

  if (loading) {
    return (
      <div className="dashboard-container">
        <div className="loading-spinner">Loading dashboard...</div>
      </div>
    );
  }

  if (error) {
    return (
      <div className="dashboard-container">
        <div className="error-message">Error: {error}</div>
      </div>
    );
  }

  if (!profile) {
    return (
      <div className="dashboard-container">
        <div className="error-message">Learner profile not found</div>
      </div>
    );
  }

  return (
    <div className={`dashboard-container ${isRTL ? 'rtl' : ''}`}>
      {isDemo && (
        <div className="demo-banner">
          <span className="demo-icon">🎮</span>
          <span>{t('demoMode')}</span>
        </div>
      )}
      <header className="dashboard-header">
        <h1>{t('dashboard')}</h1>
        <p className="learner-id">ID: {learnerId}</p>
      </header>

      {/* Profile Card */}
      <section className="profile-card">
        <h2>{t('profile')}</h2>
        <div className="profile-details">
          <div className="profile-item">
            <span className="label">{t('difficulty')}:</span>
            <span className={`difficulty-badge difficulty-${profile.difficulty_level.toLowerCase()}`}>
              {t(profile.difficulty_level.toLowerCase())}
            </span>
          </div>
          <div className="profile-item">
            <span className="label">{t('experienceScores')}:</span>
            <ul className="experience-scores">
              <li>{t('python')}: {profile.python_score}/10</li>
              <li>{t('ml')}: {profile.ml_score}/10</li>
              <li>{t('robotics')}: {profile.robotics_score}/10</li>
              <li>{t('ros')}: {profile.ros_score}/10</li>
            </ul>
          </div>
          <div className="profile-item">
            <span className="label">{t('recommendedChapters')}:</span>
            <ul className="recommended-chapters">
              {profile.recommended_chapters.map((chapter, idx) => (
                <li key={idx}>{chapter}</li>
              ))}
            </ul>
          </div>
        </div>
      </section>

      {/* Progress Card */}
      <section className="progress-card">
        <h2>{t('progress')}</h2>
        <div className="progress-stats">
          <div className="stat-item">
            <span className="stat-value">{progress.totalAttempts}</span>
            <span className="stat-label">{t('totalAssessments')}</span>
          </div>
          <div className="stat-item">
            <span className="stat-value">{progress.passed}</span>
            <span className="stat-label">Passed</span>
          </div>
          <div className="stat-item">
            <span className="stat-value">{progress.averageScore}%</span>
            <span className="stat-label">Average Score</span>
          </div>
        </div>
      </section>

      {/* Assessment Results */}
      <section className="results-section">
        <h2>Recent Assessment Results</h2>
        {results.length === 0 ? (
          <p className="no-results">No assessment results yet. Complete assessments to see them here.</p>
        ) : (
          <div className="results-list">
            {results.map((result) => (
              <div key={result.id} className={`result-card ${result.passed ? 'passed' : 'failed'}`}>
                <div className="result-header">
                  <h3>Assessment #{result.assessment_id}</h3>
                  <span className={`status-badge ${result.passed ? 'passed' : 'failed'}`}>
                    {result.passed ? 'PASSED' : 'FAILED'}
                  </span>
                </div>
                <div className="result-details">
                  <p className="score">
                    Score: <strong>{result.score}/{result.max_score}</strong>
                    ({((result.score / result.max_score) * 100).toFixed(1)}%)
                  </p>
                  <p className="timestamp">
                    Submitted: {new Date(result.submitted_at).toLocaleString()}
                  </p>
                  {result.feedback && (
                    <div className="feedback">
                      <h4>Feedback:</h4>
                      <p>{result.feedback}</p>
                    </div>
                  )}
                </div>
              </div>
            ))}
          </div>
        )}
      </section>

      {/* Actions */}
      <section className="actions-section">
        <button className="btn-primary" onClick={() => window.location.href = '/docs/chapter-1'}>
          Continue Learning
        </button>
        <button className="btn-secondary" onClick={() => window.location.href = '/profile/edit'}>
          Edit Profile
        </button>
      </section>
    </div>
  );
};

export default LearnerDashboard;
