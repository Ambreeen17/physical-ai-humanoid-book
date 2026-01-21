import React, { useState, useEffect } from 'react';
import styles from './OnboardingQuestionnaire.module.css';

/**
 * OnboardingQuestionnaire Component
 *
 * A multi-step form that collects user background, hardware,
 * and learning goals to personalize the textbook experience.
 *
 * Steps:
 * 1. Basic info (email, name)
 * 2. Programming & AI experience
 * 3. Robotics experience & learning goal
 * 4. Hardware (GPU, Jetson, Robot)
 * 5. Review & submit
 */

const OnboardingQuestionnaire = ({ onComplete }) => {
  const [currentStep, setCurrentStep] = useState(1);
  const [loading, setLoading] = useState(false);
  const [formOptions, setFormOptions] = useState(null);
  const [error, setError] = useState(null);

  const [formData, setFormData] = useState({
    email: '',
    first_name: '',
    last_name: '',
    programming_level: '',
    ai_experience: '',
    robotics_experience: '',
    has_rtx_gpu: false,
    rtx_gpu_model: null,
    rtx_gpu_vram_gb: null,
    jetson_model: 'none',
    robot_type: 'none',
    robot_model: '',
    learning_goal: '',
    additional_interests: [],
    comments: ''
  });

  // Fetch form options on mount
  useEffect(() => {
    const fetchFormOptions = async () => {
      try {
        const response = await fetch('/api/v1/onboarding/form-options');
        if (!response.ok) throw new Error('Failed to fetch form options');
        const data = await response.json();
        setFormOptions(data);
      } catch (err) {
        setError('Failed to load form options: ' + err.message);
      }
    };
    fetchFormOptions();
  }, []);

  const handleInputChange = (e) => {
    const { name, value, type, checked } = e.target;
    setFormData(prev => ({
      ...prev,
      [name]: type === 'checkbox' ? checked : value
    }));
  };

  const handleMultiSelect = (name, value) => {
    setFormData(prev => {
      const current = prev[name] || [];
      if (current.includes(value)) {
        return { ...prev, [name]: current.filter(v => v !== value) };
      } else {
        return { ...prev, [name]: [...current, value] };
      }
    });
  };

  const handleSubmit = async (e) => {
    e.preventDefault();
    setLoading(true);
    setError(null);

    try {
      const response = await fetch('/api/v1/onboarding/questionnaire', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          ...formData,
          rtx_gpu_vram_gb: formData.has_rtx_gpu ? parseInt(formData.rtx_gpu_vram_gb) : null
        })
      });

      if (!response.ok) {
        const data = await response.json();
        throw new Error(data.detail || 'Failed to submit questionnaire');
      }

      const result = await response.json();

      // Save to localStorage for later reference
      localStorage.setItem('user_profile', JSON.stringify(result));
      localStorage.setItem('user_recommendations', JSON.stringify(result.recommendations));

      // Call callback
      if (onComplete) {
        onComplete(result);
      }

    } catch (err) {
      setError('Failed to submit questionnaire: ' + err.message);
    } finally {
      setLoading(false);
    }
  };

  if (!formOptions) {
    return <div className={styles.loading}>Loading questionnaire...</div>;
  }

  const totalSteps = 5;
  const progressPercent = (currentStep / totalSteps) * 100;

  return (
    <div className={styles.container}>
      <div className={styles.header}>
        <h1>Welcome to Physical AI & Humanoid Robotics</h1>
        <p>Help us personalize your learning experience</p>
      </div>

      {/* Progress Bar */}
      <div className={styles.progressContainer}>
        <div className={styles.progressBar}>
          <div
            className={styles.progressFill}
            style={{ width: `${progressPercent}%` }}
          />
        </div>
        <p className={styles.progressText}>
          Step {currentStep} of {totalSteps}
        </p>
      </div>

      {error && (
        <div className={styles.error}>
          <strong>Error:</strong> {error}
        </div>
      )}

      <form onSubmit={handleSubmit}>
        {/* Step 1: Basic Info */}
        {currentStep === 1 && (
          <Step1BasicInfo
            formData={formData}
            handleInputChange={handleInputChange}
          />
        )}

        {/* Step 2: Programming & AI Experience */}
        {currentStep === 2 && (
          <Step2Experience
            formData={formData}
            handleInputChange={handleInputChange}
            formOptions={formOptions}
          />
        )}

        {/* Step 3: Robotics & Learning Goal */}
        {currentStep === 3 && (
          <Step3RoboticsGoal
            formData={formData}
            handleInputChange={handleInputChange}
            formOptions={formOptions}
          />
        )}

        {/* Step 4: Hardware */}
        {currentStep === 4 && (
          <Step4Hardware
            formData={formData}
            handleInputChange={handleInputChange}
            formOptions={formOptions}
          />
        )}

        {/* Step 5: Review */}
        {currentStep === 5 && (
          <Step5Review formData={formData} />
        )}

        {/* Navigation Buttons */}
        <div className={styles.buttonGroup}>
          <button
            type="button"
            className={styles.buttonSecondary}
            onClick={() => setCurrentStep(prev => Math.max(1, prev - 1))}
            disabled={currentStep === 1}
          >
            ← Previous
          </button>

          {currentStep < totalSteps ? (
            <button
              type="button"
              className={styles.buttonPrimary}
              onClick={() => setCurrentStep(prev => Math.min(totalSteps, prev + 1))}
            >
              Next →
            </button>
          ) : (
            <button
              type="submit"
              className={styles.buttonSubmit}
              disabled={loading}
            >
              {loading ? 'Submitting...' : 'Complete Onboarding'}
            </button>
          )}
        </div>
      </form>
    </div>
  );
};

// =====================================================================
// Step Components
// =====================================================================

const Step1BasicInfo = ({ formData, handleInputChange }) => (
  <div className={`section`}>
    <h2>About You</h2>
    <p>Let's start with the basics.</p>

    <div className={`formGroup`}>
      <label>Email *</label>
      <input
        type="email"
        name="email"
        value={formData.email}
        onChange={handleInputChange}
        placeholder="learner@example.com"
        required
      />
    </div>

    <div className={`formGroup`}>
      <label>First Name (optional)</label>
      <input
        type="text"
        name="first_name"
        value={formData.first_name}
        onChange={handleInputChange}
        placeholder="Alex"
      />
    </div>

    <div className={`formGroup`}>
      <label>Last Name (optional)</label>
      <input
        type="text"
        name="last_name"
        value={formData.last_name}
        onChange={handleInputChange}
        placeholder="Chen"
      />
    </div>
  </div>
);

const Step2Experience = ({ formData, handleInputChange, formOptions }) => (
  <div className={`section`}>
    <h2>Your Experience</h2>
    <p>Help us understand your background.</p>

    <div className={`formGroup`}>
      <label>Programming Level *</label>
      <div className={styles.radioGroup}>
        {formOptions.programming_level.map(option => (
          <label key={option.value} className={styles.radioLabel}>
            <input
              type="radio"
              name="programming_level"
              value={option.value}
              checked={formData.programming_level === option.value}
              onChange={handleInputChange}
              required
            />
            <div className={styles.radioContent}>
              <strong>{option.label}</strong>
              <p>{option.description}</p>
            </div>
          </label>
        ))}
      </div>
    </div>

    <div className={`formGroup`}>
      <label>AI/ML Experience *</label>
      <div className={styles.radioGroup}>
        {formOptions.ai_experience.map(option => (
          <label key={option.value} className={styles.radioLabel}>
            <input
              type="radio"
              name="ai_experience"
              value={option.value}
              checked={formData.ai_experience === option.value}
              onChange={handleInputChange}
              required
            />
            <div className={styles.radioContent}>
              <strong>{option.label}</strong>
              <p>{option.description}</p>
            </div>
          </label>
        ))}
      </div>
    </div>
  </div>
);

const Step3RoboticsGoal = ({ formData, handleInputChange, formOptions }) => (
  <div className={`section`}>
    <h2>Robotics & Goals</h2>
    <p>Tell us about your robotics background and learning goals.</p>

    <div className={`formGroup`}>
      <label>Robotics Experience *</label>
      <div className={styles.radioGroup}>
        {formOptions.robotics_experience.map(option => (
          <label key={option.value} className={styles.radioLabel}>
            <input
              type="radio"
              name="robotics_experience"
              value={option.value}
              checked={formData.robotics_experience === option.value}
              onChange={handleInputChange}
              required
            />
            <div className={styles.radioContent}>
              <strong>{option.label}</strong>
              <p>{option.description}</p>
            </div>
          </label>
        ))}
      </div>
    </div>

    <div className={`formGroup`}>
      <label>Primary Learning Goal *</label>
      <div className={styles.radioGroup}>
        {formOptions.learning_goal.map(option => (
          <label key={option.value} className={styles.radioLabel}>
            <input
              type="radio"
              name="learning_goal"
              value={option.value}
              checked={formData.learning_goal === option.value}
              onChange={handleInputChange}
              required
            />
            <div className={styles.radioContent}>
              <strong>{option.label}</strong>
              <p>{option.description}</p>
            </div>
          </label>
        ))}
      </div>
    </div>
  </div>
);

const Step4Hardware = ({ formData, handleInputChange, formOptions }) => (
  <div className={`section`}>
    <h2>Your Hardware</h2>
    <p>Tell us about the hardware you have access to.</p>

    {/* RTX GPU */}
    <div className={`formGroup`}>
      <label>Do you have an RTX GPU? *</label>
      <div className={styles.yesNoGroup}>
        <label>
          <input
            type="radio"
            name="has_rtx_gpu"
            value={true}
            checked={formData.has_rtx_gpu === true}
            onChange={() => handleInputChange({ target: { name: 'has_rtx_gpu', type: 'checkbox', checked: true } })}
          />
          Yes
        </label>
        <label>
          <input
            type="radio"
            name="has_rtx_gpu"
            value={false}
            checked={formData.has_rtx_gpu === false}
            onChange={() => handleInputChange({ target: { name: 'has_rtx_gpu', type: 'checkbox', checked: false } })}
          />
          No
        </label>
      </div>

      {formData.has_rtx_gpu && (
        <>
          <label>GPU Model</label>
          <select
            name="rtx_gpu_model"
            value={formData.rtx_gpu_model || ''}
            onChange={handleInputChange}
          >
            <option value="">Select model...</option>
            {formOptions.rtx_gpu_model.map(option => (
              <option key={option.value} value={option.value}>
                {option.label}
              </option>
            ))}
          </select>

          {formData.rtx_gpu_model === 'other' && (
            <>
              <label>GPU VRAM (GB)</label>
              <input
                type="number"
                name="rtx_gpu_vram_gb"
                value={formData.rtx_gpu_vram_gb || ''}
                onChange={handleInputChange}
                min="4"
                max="192"
                placeholder="24"
              />
            </>
          )}
        </>
      )}
    </div>

    {/* Jetson */}
    <div className={`formGroup`}>
      <label>Jetson Hardware *</label>
      <select
        name="jetson_model"
        value={formData.jetson_model}
        onChange={handleInputChange}
      >
        {formOptions.jetson_model.map(option => (
          <option key={option.value} value={option.value}>
            {option.label}
          </option>
        ))}
      </select>
    </div>

    {/* Physical Robot */}
    <div className={`formGroup`}>
      <label>Physical Robot *</label>
      <select
        name="robot_type"
        value={formData.robot_type}
        onChange={handleInputChange}
      >
        {formOptions.robot_type.map(option => (
          <option key={option.value} value={option.value}>
            {option.label}
          </option>
        ))}
      </select>

      {formData.robot_type !== 'none' && (
        <>
          <label>Robot Model (e.g., Unitree G1, UR10e)</label>
          <input
            type="text"
            name="robot_model"
            value={formData.robot_model}
            onChange={handleInputChange}
            placeholder="Unitree G1"
          />
        </>
      )}
    </div>
  </div>
);

const Step5Review = ({ formData }) => (
  <div className={`section`}>
    <h2>Review Your Profile</h2>
    <p>Please review your information before submitting.</p>

    <div className={styles.reviewBox}>
      <h3>Background</h3>
      <p><strong>Programming:</strong> {formData.programming_level}</p>
      <p><strong>AI Experience:</strong> {formData.ai_experience}</p>
      <p><strong>Robotics:</strong> {formData.robotics_experience}</p>
      <p><strong>Learning Goal:</strong> {formData.learning_goal}</p>

      <h3>Hardware</h3>
      <p><strong>RTX GPU:</strong> {formData.has_rtx_gpu ? `Yes (${formData.rtx_gpu_model})` : 'No'}</p>
      <p><strong>Jetson:</strong> {formData.jetson_model !== 'none' ? formData.jetson_model : 'None'}</p>
      <p><strong>Robot:</strong> {formData.robot_type !== 'none' ? `${formData.robot_type} (${formData.robot_model})` : 'None'}</p>
    </div>

    <p className={styles.disclaimer}>
      You can update your profile anytime in settings.
    </p>
  </div>
);

export default OnboardingQuestionnaire;
