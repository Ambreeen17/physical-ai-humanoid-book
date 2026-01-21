// @ts-check

const lightCodeTheme = require('prism-react-renderer/themes/github');
const darkCodeTheme = require('prism-react-renderer/themes/dracula');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An AI-Native Textbook for Embodied Intelligence',
  favicon: 'img/favicon.ico',

  url: 'https://robotics-textbook.github.io',
  baseUrl: '/',
  organizationName: 'robotics-textbook',
  projectName: 'robotics-textbook',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  i18n: {
    defaultLocale: 'en',
    locales: ['en', 'ur'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          editUrl: 'https://github.com/robotics-textbook/robotics-textbook/tree/main/',
          lastVersion: 'current',
          versions: {
            current: {
              label: '1.0.0',
            },
          },
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/styles/custom.css'),
        },
      }),
    ],
  ],

  plugins: [
    [
      '@docusaurus/plugin-sitemap',
      {
        changefreq: 'weekly',
        priority: 0.5,
      },
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      image: 'img/social-card.png',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Robotics Textbook Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Chapters',
          },
          {
            type: 'localeDropdown',
            position: 'right',
          },
          {
            href: 'https://github.com/robotics-textbook/robotics-textbook',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Chapters',
            items: [
              { label: 'Chapter 1: Introduction to Physical AI', to: '/docs/chapter-1' },
              { label: 'Chapter 2: Sensors & Perception', to: '/docs/chapter-2' },
              { label: 'All Chapters', to: '/docs' },
            ],
          },
          {
            title: 'Resources',
            items: [
              { label: 'ROS 2 Humble', href: 'https://docs.ros.org/en/humble/' },
              { label: 'Gazebo', href: 'https://gazebosim.org/' },
              { label: 'NVIDIA Isaac', href: 'https://developer.nvidia.com/isaac/' },
            ],
          },
          {
            title: 'Community',
            items: [
              { label: 'GitHub Discussions', href: 'https://github.com/robotics-textbook/robotics-textbook/discussions' },
              { label: 'Report Issues', href: 'https://github.com/robotics-textbook/robotics-textbook/issues' },
            ],
          },
        ],
        copyright: `Copyright © 2025 AI-Native Robotics Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['bash', 'python', 'cpp', 'yaml', 'json', 'xml'],
      },
      algolia: {
        appId: 'YOUR_ALGOLIA_APP_ID',
        apiKey: 'YOUR_ALGOLIA_SEARCH_KEY',
        indexName: 'robotics_textbook',
        contextualSearch: true,
      },
    }),
};

module.exports = config;
