import React, { useState } from 'react';
import Layout from '@theme/Layout';
import Link from '@docusaurus/Link';
import { TranslationProvider } from '../components/TranslationProvider';
import LanguageSwitcher from '../components/LanguageSwitcher';
import FloatingChatbot from '../components/FloatingChatbot';
import './index.css';

// Chapter data with titles and icons
const CHAPTERS = [
  { id: 1, title: 'Introduction to Physical AI', icon: '🤖', color: '#4F46E5', description: 'Foundations of embodied intelligence' },
  { id: 2, title: 'Kinematics & Dynamics', icon: '⚙️', color: '#7C3AED', description: 'Robot motion mathematics' },
  { id: 3, title: 'Sensors & Perception', icon: '👁️', color: '#2563EB', description: 'Understanding the environment' },
  { id: 4, title: 'State Estimation', icon: '📊', color: '#0891B2', description: 'Kalman filters & sensor fusion' },
  { id: 5, title: 'Computer Vision', icon: '📷', color: '#059669', description: 'Visual perception for robots' },
  { id: 6, title: 'Control Theory', icon: '🎛️', color: '#65A30D', description: 'PID, LQR, and adaptive control' },
  { id: 7, title: 'Motion Planning', icon: '🗺️', color: '#CA8A04', description: 'RRT, PRM, and path planning' },
  { id: 8, title: 'Manipulation & Grasping', icon: '🦾', color: '#EA580C', description: 'Robotic arms and grippers' },
  { id: 9, title: 'Task & Motion Planning', icon: '📋', color: '#DC2626', description: 'TAMP and hierarchical planning' },
  { id: 10, title: 'Reinforcement Learning', icon: '🧠', color: '#DB2777', description: 'Learning from experience' },
  { id: 11, title: 'Imitation Learning', icon: '🎓', color: '#9333EA', description: 'Learning from demonstrations' },
  { id: 12, title: 'Vision-Language-Action', icon: '💬', color: '#4F46E5', description: 'Multimodal AI for robots' },
  { id: 13, title: 'System Integration', icon: '🔧', color: '#0EA5E9', description: 'Building complete systems' },
  { id: 14, title: 'Safety & Robustness', icon: '🛡️', color: '#14B8A6', description: 'Safe robot deployment' },
  { id: 15, title: 'Deployment & Operations', icon: '🚀', color: '#22C55E', description: 'Real-world deployment' },
  { id: 16, title: 'Capstone Project', icon: '🏆', color: '#F59E0B', description: 'Build your own robot system' },
];

// Animated 3D Book Component
const AnimatedBook = ({ chapter, isOpen, onClick }) => {
  return (
    <div
      className={`book-container ${isOpen ? 'book-open' : ''}`}
      onClick={onClick}
      style={{ '--book-color': chapter.color }}
    >
      <div className="book">
        <div className="book-cover">
          <div className="book-spine"></div>
          <div className="book-front">
            <div className="book-icon">{chapter.icon}</div>
            <div className="book-number">Chapter {chapter.id}</div>
            <div className="book-title">{chapter.title}</div>
          </div>
        </div>
        <div className="book-pages">
          <div className="book-page"></div>
          <div className="book-page"></div>
          <div className="book-page"></div>
        </div>
      </div>
      {isOpen && (
        <div className="book-preview">
          <h3>{chapter.title}</h3>
          <p>{chapter.description}</p>
          <Link to={`/docs/chapter-${chapter.id}`} className="read-btn">
            Read Chapter →
          </Link>
        </div>
      )}
    </div>
  );
};

// Main Landing Page
export default function Home() {
  const [openBook, setOpenBook] = useState(null);

  const handleBookClick = (chapterId) => {
    setOpenBook(openBook === chapterId ? null : chapterId);
  };

  return (
    <TranslationProvider>
      <Layout
        title="Physical AI & Humanoid Robotics"
        description="An AI-Native Textbook for Embodied Intelligence">

        {/* Language Switcher */}
        <div className="navbar-language">
          <LanguageSwitcher />
        </div>

        {/* Hero Section */}
        <header className="hero-section">
          <div className="hero-content">
            <div className="hero-badge">AI-Native Textbook</div>
            <h1 className="hero-title">
              Physical AI &<br />
              <span className="gradient-text">Humanoid Robotics</span>
            </h1>
            <p className="hero-subtitle">
              Master embodied intelligence with hands-on labs, ROS 2 integration,
              and AI-powered learning. From kinematics to deployment.
            </p>
            <div className="hero-buttons">
              <Link to="/docs/intro" className="btn-primary">
                Start Learning
              </Link>
              <Link to="/dashboard" className="btn-secondary">
                View Dashboard
              </Link>
            </div>
            <div className="hero-stats">
              <div className="stat">
                <span className="stat-number">16</span>
                <span className="stat-label">Chapters</span>
              </div>
              <div className="stat">
                <span className="stat-number">48+</span>
                <span className="stat-label">Labs</span>
              </div>
              <div className="stat">
                <span className="stat-number">7</span>
                <span className="stat-label">Languages</span>
              </div>
            </div>
          </div>
          <div className="hero-visual">
            <div className="hero-banner">
              <img
                src="/img/humanoid-robot-banner.jpg"
                alt="Humanoid Robot - Physical AI"
                className="banner-image"
              />
              <div className="banner-overlay">
                <div className="banner-text">
                  <span className="banner-tag">AI-Native Learning</span>
                  <h3 className="banner-title">Build the Future with Physical AI</h3>
                  <p className="banner-description">From simulation to real-world deployment</p>
                </div>
              </div>
            </div>
          </div>
        </header>

        {/* Interactive Book Shelf */}
        <section className="bookshelf-section">
          <h2 className="section-title">
            <span className="title-icon">📚</span>
            Interactive Chapter Library
          </h2>
          <p className="section-subtitle">Click on any book to preview the chapter</p>

          <div className="bookshelf">
            {CHAPTERS.map((chapter) => (
              <AnimatedBook
                key={chapter.id}
                chapter={chapter}
                isOpen={openBook === chapter.id}
                onClick={() => handleBookClick(chapter.id)}
              />
            ))}
          </div>
        </section>

        {/* Features Section */}
        <section className="features-section">
          <h2 className="section-title">
            <span className="title-icon">✨</span>
            Why This Textbook?
          </h2>
          <div className="features-grid">
            <div className="feature-card">
              <div className="feature-icon">🎯</div>
              <h3>Hands-On Labs</h3>
              <p>Docker-based ROS 2 labs you can run anywhere. Real code, real robots.</p>
            </div>
            <div className="feature-card">
              <div className="feature-icon">🤖</div>
              <h3>AI Assistant</h3>
              <p>RAG-powered chatbot answers your questions using course content.</p>
            </div>
            <div className="feature-card">
              <div className="feature-icon">📊</div>
              <h3>Personalized</h3>
              <p>Adaptive difficulty levels match your experience in Python, ML, and robotics.</p>
            </div>
            <div className="feature-card">
              <div className="feature-icon">🌍</div>
              <h3>Multi-Language</h3>
              <p>Available in English, Urdu, Chinese, Spanish, Arabic, Hindi, and French.</p>
            </div>
          </div>
        </section>

        {/* CTA Section */}
        <section className="cta-section">
          <div className="cta-content">
            <h2>Ready to Build Intelligent Robots?</h2>
            <p>Start with Chapter 1 and build your way to a complete capstone project.</p>
            <Link to="/docs/chapter-1" className="btn-primary btn-large">
              Begin Your Journey →
            </Link>
          </div>
        </section>

        {/* Floating AI Chatbot */}
        <FloatingChatbot />

      </Layout>
    </TranslationProvider>
  );
}
