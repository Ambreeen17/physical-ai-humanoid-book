import React, { useState, useEffect } from 'react';
import styles from './PersonalizationToggle.module.css';

export default function PersonalizationToggle() {
  const [difficulty, setDifficulty] = useState('intermediate');
  const [mounted, setMounted] = useState(false);

  // Load preference from localStorage on mount
  useEffect(() => {
    setMounted(true);
    const saved = localStorage.getItem('personalization-difficulty');
    if (saved) {
      setDifficulty(saved);
      applyDifficulty(saved);
    }
  }, []);

  const handleChange = (newDifficulty) => {
    setDifficulty(newDifficulty);
    localStorage.setItem('personalization-difficulty', newDifficulty);
    applyDifficulty(newDifficulty);
  };

  const applyDifficulty = (level) => {
    // Hide/show content based on difficulty level
    const allContent = document.querySelectorAll('[data-difficulty]');
    allContent.forEach((el) => {
      const elLevel = el.getAttribute('data-difficulty');
      if (level === 'all') {
        el.style.display = 'block';
      } else if (elLevel === level || elLevel === 'all') {
        el.style.display = 'block';
      } else {
        el.style.display = 'none';
      }
    });
  };

  if (!mounted) return null;

  return (
    <div className={styles.personalizationToggle}>
      <label htmlFor="difficulty-select">Content Level:</label>
      <select
        id="difficulty-select"
        value={difficulty}
        onChange={(e) => handleChange(e.target.value)}
        className={styles.select}
      >
        <option value="beginner">Beginner (Intuitive explanations)</option>
        <option value="intermediate">Intermediate (Balanced depth)</option>
        <option value="advanced">Advanced (Research-focused)</option>
        <option value="all">Show All (Compare perspectives)</option>
      </select>
      <p className={styles.description}>
        {difficulty === 'beginner' &&
          'Simplified explanations, real-world analogies, foundational concepts.'}
        {difficulty === 'intermediate' &&
          'Balanced approach with mathematical notation and research connections.'}
        {difficulty === 'advanced' &&
          'Deep dives into cutting-edge research, theoretical foundations, and challenges.'}
        {difficulty === 'all' &&
          'Display all content levels side-by-side for comprehensive understanding.'}
      </p>
    </div>
  );
}
