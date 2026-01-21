import React, { useState } from 'react';
import { useTranslation } from './TranslationProvider';
import './LanguageSwitcher.css';

/**
 * Language Switcher Component
 *
 * Uses TranslationProvider for instant language switching
 * without page reload or separate locale files
 */
const LanguageSwitcher = () => {
  const [isOpen, setIsOpen] = useState(false);
  const { currentLang, changeLanguage, languages, getCurrentLanguage, t } = useTranslation();

  const currentLanguage = getCurrentLanguage();

  const handleLanguageChange = (langCode) => {
    changeLanguage(langCode);
    setIsOpen(false);
  };

  return (
    <div className="language-switcher">
      <button
        className="language-btn"
        onClick={() => setIsOpen(!isOpen)}
        aria-label="Select language"
      >
        <span className="lang-flag">{currentLanguage.flag}</span>
        <span className="lang-label">{currentLanguage.native}</span>
        <span className="lang-arrow">{isOpen ? '▲' : '▼'}</span>
      </button>

      {isOpen && (
        <>
          <div className="language-backdrop" onClick={() => setIsOpen(false)} />
          <div className="language-dropdown">
            <div className="dropdown-header">
              {t('selectLanguage')}
            </div>
            {languages.map((lang) => (
              <button
                key={lang.code}
                className={`language-option ${lang.code === currentLang ? 'active' : ''}`}
                onClick={() => handleLanguageChange(lang.code)}
              >
                <span className="lang-flag">{lang.flag}</span>
                <div className="lang-info">
                  <span className="lang-native">{lang.native}</span>
                  <span className="lang-english">{lang.label}</span>
                </div>
                {lang.code === currentLang && <span className="check-mark">✓</span>}
              </button>
            ))}
          </div>
        </>
      )}
    </div>
  );
};

export default LanguageSwitcher;
