/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Part I: Foundations',
      items: [
        'chapter-1',
        'chapter-2',
        'chapter-3',
        'chapter-4',
      ],
    },
    {
      type: 'category',
      label: 'Part II: Perception & Control',
      items: [
        'chapter-5',
        'chapter-6',
        'chapter-7',
        'chapter-8',
      ],
    },
    {
      type: 'category',
      label: 'Part III: Planning & Learning',
      items: [
        'chapter-9',
        'chapter-10',
        'chapter-11',
        'chapter-12',
      ],
    },
    {
      type: 'category',
      label: 'Part IV: Integration & Deployment',
      items: [
        'chapter-13',
        'chapter-14',
        'chapter-15',
        'chapter-16',
      ],
    },
  ],
};

export default sidebars;
