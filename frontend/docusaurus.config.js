// @ts-check
// `@type` JSDoc annotations allow editor autocompletion and type checking
// (when paired with `@ts-check`).
// There are various equivalent ways to declare your Docusaurus config.
// See: https://docusaurus.io/docs/api/docusaurus-config

import {themes as prismThemes} from 'prism-react-renderer';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An AI-Native Textbook for Embodied Intelligence',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://physical-ai-humanoid-book-fwk2.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'your-org', // Usually your GitHub org/user name.
  projectName: 'physical-ai-textbook', // Usually your repo name.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  // i18n handled client-side via TranslationProvider component
  // This avoids needing separate locale folders
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/your-org/physical-ai-textbook/tree/main/frontend/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI Textbook',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            to: '/dashboard',
            label: 'Dashboard',
            position: 'left',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Parts',
            items: [
              {
                label: 'Part 1: Foundations',
                to: '/docs/intro',
              },
              {
                label: 'Part 2: Kinematics & Dynamics',
                to: '/docs/chapter-2',
              },
              {
                label: 'Part 3: Perception & Sensors',
                to: '/docs/chapter-3',
              },
              {
                label: 'Part 4: Control Systems',
                to: '/docs/chapter-6',
              },
              {
                label: 'Part 5: Motion & Manipulation',
                to: '/docs/chapter-7',
              },
              {
                label: 'Part 6: Learning & AI',
                to: '/docs/chapter-10',
              },
              {
                label: 'Part 7: System Integration',
                to: '/docs/chapter-13',
              },
              {
                label: 'Part 8: Safety & Deployment',
                to: '/docs/chapter-14',
              },
              {
                label: 'Part 9: Capstone Project',
                to: '/docs/chapter-16',
              },
            ],
          },
          {
            title: 'Resources',
            items: [
              {
                label: 'ROS 2 Documentation',
                href: 'https://docs.ros.org/en/humble/',
              },
              {
                label: 'Isaac Sim',
                href: 'https://developer.nvidia.com/isaac-sim',
              },
            ],
          },
          {
            title: 'More',
            items: [],
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI Textbook. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash', 'yaml', 'docker'],
      },
    }),
};

export default config;
