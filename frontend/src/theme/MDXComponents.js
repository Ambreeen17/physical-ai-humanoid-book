import React from 'react';
// Import the original mapper
import MDXComponents from '@theme-original/MDXComponents';
import PersonalizationToggle from '../components/PersonalizationToggle';
import ChapterContent from '../components/ChapterContent';

export default {
  // Re-use the default mapping
  ...MDXComponents,
  // Add custom components here
  PersonalizationToggle,
  ChapterContent,
};
