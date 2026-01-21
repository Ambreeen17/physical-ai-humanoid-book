import React, { useState, useEffect } from 'react';

/**
 * PersonalizedContent Component
 *
 * Renders content based on the learner's profile (beginner, intermediate, advanced).
 * Falls back to intermediate if no profile is found.
 */
const PersonalizedContent = ({ sectionId, variants }) => {
  const [profile, setProfile] = useState('intermediate');

  useEffect(() => {
    // Load profile from localStorage (populated by diagnostic assessment)
    const storedProfile = localStorage.getItem('learner_profile');
    if (storedProfile && ['beginner', 'intermediate', 'advanced'].includes(storedProfile)) {
      setProfile(storedProfile);
    }
  }, []);

  const handleProfileChange = (newProfile) => {
    setProfile(newProfile);
    localStorage.setItem('learner_profile', newProfile);
  };

  const currentVariant = variants[profile];

  return (
    <div className={`personalized-section variant-${profile}`}>
      <div className="variant-selector" style={{
        display: 'flex',
        gap: '10px',
        marginBottom: '1rem',
        fontSize: '0.8rem',
        borderBottom: '1px solid #eee',
        paddingBottom: '5px'
      }}>
        <span style={{ fontWeight: 'bold' }}>View as:</span>
        {['beginner', 'intermediate', 'advanced'].map((p) => (
          <button
            key={p}
            onClick={() => handleProfileChange(p)}
            style={{
              padding: '2px 8px',
              borderRadius: '4px',
              border: '1px solid #ccc',
              background: profile === p ? '#007bff' : '#fff',
              color: profile === p ? '#fff' : '#000',
              cursor: 'pointer',
              textTransform: 'capitalize'
            }}
          >
            {p}
          </button>
        ))}
      </div>

      <div className="variant-metadata" style={{
        backgroundColor: '#f8f9fa',
        padding: '10px',
        borderRadius: '5px',
        marginBottom: '1rem',
        fontSize: '0.9rem'
      }}>
        <div><strong>Complexity:</strong> {currentVariant.difficulty}</div>
        <div><strong>Est. Reading Time:</strong> {currentVariant.readingTimeMinutes} mins</div>
        {currentVariant.prerequisites.length > 0 && (
          <div><strong>Prerequisites:</strong> {currentVariant.prerequisites.join(', ')}</div>
        )}
      </div>

      <div className="content-body">
        {/* In a real Docusaurus implementation, this would render MDX components */}
        {/* For this prototype, we'll assume the children are passed in */}
        {currentVariant.content}
      </div>

      {currentVariant.nextLevelId && (
        <div className="next-level-prompt" style={{
          marginTop: '1rem',
          fontStyle: 'italic',
          color: '#666'
        }}>
          Feeling comfortable? Try the <a href={`#${currentVariant.nextLevelId}`}>Intermediate depth</a> variant of this section.
        </div>
      )}
    </div>
  );
};

export default PersonalizedContent;
