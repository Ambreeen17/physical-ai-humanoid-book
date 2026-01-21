import React, { useState, useEffect } from 'react';
import './PersonalizationToggle.css';

/**
 * Personalization Toggle Component
 *
 * Allows users to switch between difficulty variants:
 * - Beginner: Intuitive analogies, minimal math
 * - Intermediate: Formal definitions, code examples
 * - Advanced: Research connections, challenging extensions
 *
 * Stores preference in localStorage and fetches personalized content from API.
 */
const PersonalizationToggle = ({ chapterId, onVariantChange }) => {
  const [selectedVariant, setSelectedVariant] = useState('intermediate');
  const [loading, setLoading] = useState(false);

  // Load saved preference from localStorage on mount
  useEffect(() => {
    const savedVariant = localStorage.getItem('preferredDifficulty') || 'intermediate';
    setSelectedVariant(savedVariant);
    if (onVariantChange) {
      onVariantChange(savedVariant);
    }
  }, [onVariantChange]);

  // Handle variant selection
  const handleVariantChange = async (variant) => {
    setLoading(true);
    setSelectedVariant(variant);

    // Save preference to localStorage
    localStorage.setItem('preferredDifficulty', variant);

    // Fetch personalized content from API
    try {
      const response = await fetch(`http://localhost:8000/api/personalization/chapter/${chapterId}?variant=${variant}`);
      if (!response.ok) {
        throw new Error('Failed to fetch personalized content');
      }
      const data = await response.json();

      // Notify parent component of the variant change
      if (onVariantChange) {
        onVariantChange(variant, data);
      }

      setLoading(false);
    } catch (error) {
      console.error('Personalization fetch error:', error);
      setLoading(false);
      // Still allow variant change even if fetch fails
      if (onVariantChange) {
        onVariantChange(variant);
      }
    }
  };

  return (
    <div className="personalization-toggle">
      <div className="toggle-header">
        <span className="toggle-icon">🎯</span>
        <h3>Content Difficulty</h3>
      </div>
      <p className="toggle-description">
        Adjust content to match your experience level
      </p>
      <div className="toggle-buttons">
        <button
          className={`toggle-btn beginner ${selectedVariant === 'beginner' ? 'active' : ''}`}
          onClick={() => handleVariantChange('beginner')}
          disabled={loading}
        >
          <span className="btn-label">Beginner</span>
          <span className="btn-description">Intuitive explanations</span>
        </button>
        <button
          className={`toggle-btn intermediate ${selectedVariant === 'intermediate' ? 'active' : ''}`}
          onClick={() => handleVariantChange('intermediate')}
          disabled={loading}
        >
          <span className="btn-label">Intermediate</span>
          <span className="btn-description">Balanced depth</span>
        </button>
        <button
          className={`toggle-btn advanced ${selectedVariant === 'advanced' ? 'active' : ''}`}
          onClick={() => handleVariantChange('advanced')}
          disabled={loading}
        >
          <span className="btn-label">Advanced</span>
          <span className="btn-description">Research-level</span>
        </button>
      </div>
      {loading && (
        <div className="loading-indicator">
          <span className="spinner"></span>
          <span>Loading personalized content...</span>
        </div>
      )}
      <div className="variant-info">
        <h4>Current Selection: {selectedVariant.charAt(0).toUpperCase() + selectedVariant.slice(1)}</h4>
        {selectedVariant === 'beginner' && (
          <p>
            <strong>Beginner:</strong> Concepts explained with real-world analogies, minimal
            mathematical notation, "Why This Matters" callouts for motivation.
          </p>
        )}
        {selectedVariant === 'intermediate' && (
          <p>
            <strong>Intermediate:</strong> Formal definitions with practical examples, code
            snippets, ROS 2 implementation details, balanced technical depth.
          </p>
        )}
        {selectedVariant === 'advanced' && (
          <p>
            <strong>Advanced:</strong> Research connections, information-theoretic perspectives,
            challenging extensions, links to cutting-edge papers and techniques.
          </p>
        )}
      </div>
    </div>
  );
};

export default PersonalizationToggle;
