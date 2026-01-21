import React, { useState } from 'react';
import { useAuth } from './AuthProvider';
import { useHistory } from '@docusaurus/router';
import './AuthButtons.css';

const AuthModal = ({ isOpen, onClose, initialMode = 'login' }) => {
  const [mode, setMode] = useState(initialMode);
  const [email, setEmail] = useState('');
  const [formData, setFormData] = useState({
    email: '',
    firstName: '',
    lastName: '',
    programmingLevel: 'beginner',
    roboticsExperience: 'none',
    aiExperience: 'none',
    goal: 'career'
  });
  const [error, setError] = useState('');
  const { login, register } = useAuth();
  const history = useHistory();

  if (!isOpen) return null;

  const handleSubmit = async (e) => {
    e.preventDefault();
    setError('');

    if (mode === 'login') {
      const result = await login(email);
      if (result.success) {
        onClose();
        history.push('/dashboard');
      } else {
        setError(result.error || 'Login failed');
      }
    } else {
      // Register
      const result = await register({
        email: formData.email,
        first_name: formData.firstName,
        last_name: formData.lastName,
        programming_level: formData.programmingLevel,
        robotics_experience: formData.roboticsExperience,
        ai_experience: formData.aiExperience,
        learning_goal: formData.goal,
        camera_available: true,
        gpu_available: false,
        has_robot: false
      });

      if (result.success) {
        onClose();
        history.push('/dashboard');
      } else {
        setError(result.error || 'Registration failed');
      }
    }
  };

  return (
    <div className="auth-modal-overlay" onClick={onClose}>
      <div className="auth-modal" onClick={e => e.stopPropagation()}>
        <button className="auth-modal-close" onClick={onClose}>×</button>
        <h2>{mode === 'login' ? 'Welcome Back' : 'Start Learning'}</h2>

        {error && <div className="auth-error">{error}</div>}

        <form onSubmit={handleSubmit}>
          {mode === 'login' ? (
            <div className="form-group">
              <label>Email Address</label>
              <input
                type="email"
                value={email}
                onChange={e => setEmail(e.target.value)}
                placeholder="you@example.com"
                required
              />
            </div>
          ) : (
            <>
              <div className="form-row">
                <div className="form-group">
                  <label>First Name</label>
                  <input
                    type="text"
                    value={formData.firstName}
                    onChange={e => setFormData({...formData, firstName: e.target.value})}
                    required
                  />
                </div>
                <div className="form-group">
                  <label>Last Name</label>
                  <input
                    type="text"
                    value={formData.lastName}
                    onChange={e => setFormData({...formData, lastName: e.target.value})}
                    required
                  />
                </div>
              </div>

              <div className="form-group">
                <label>Email</label>
                <input
                  type="email"
                  value={formData.email}
                  onChange={e => setFormData({...formData, email: e.target.value})}
                  required
                />
              </div>

              <div className="form-group">
                <label>Experience Level</label>
                <select
                  value={formData.programmingLevel}
                  onChange={e => setFormData({...formData, programmingLevel: e.target.value})}
                >
                  <option value="beginner">Beginner (Python basics)</option>
                  <option value="intermediate">Intermediate (Used NumPy/Pandas)</option>
                  <option value="advanced">Advanced (Production code)</option>
                </select>
              </div>
            </>
          )}

          <button type="submit" className="auth-submit-btn">
            {mode === 'login' ? 'Log In' : 'Create Account'}
          </button>
        </form>

        <div className="auth-footer">
          {mode === 'login' ? (
            <p>New here? <button className="link-btn" onClick={() => setMode('register')}>Sign Up</button></p>
          ) : (
            <p>Already have an account? <button className="link-btn" onClick={() => setMode('login')}>Log In</button></p>
          )}
        </div>
      </div>
    </div>
  );
};

const AuthButtons = () => {
  const { isAuthenticated, user, logout } = useAuth();
  const [showModal, setShowModal] = useState(false);
  const [modalMode, setModalMode] = useState('login');
  const history = useHistory();

  const handleLogin = () => {
    setModalMode('login');
    setShowModal(true);
  };

  const handleSignUp = () => {
    setModalMode('register');
    setShowModal(true);
  };

  const handleDashboard = () => {
    history.push('/dashboard');
  };

  if (isAuthenticated) {
    return (
      <div className="auth-buttons">
        <button className="auth-btn dashboard-btn" onClick={handleDashboard}>
          Dashboard
        </button>
        <button className="auth-btn logout-btn" onClick={logout}>
          Log Out
        </button>
      </div>
    );
  }

  return (
    <>
      <div className="auth-buttons">
        <button className="auth-btn login-btn" onClick={handleLogin}>
          Log In
        </button>
        <button className="auth-btn signup-btn" onClick={handleSignUp}>
          Sign Up
        </button>
      </div>
      <AuthModal
        isOpen={showModal}
        onClose={() => setShowModal(false)}
        initialMode={modalMode}
      />
    </>
  );
};

export default AuthButtons;
