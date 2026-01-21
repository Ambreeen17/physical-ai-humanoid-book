/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a set of docs in a scoped menu
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

module.exports = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Chapters',
      items: [
        {
          type: 'category',
          label: 'Chapter 1: Introduction to Physical AI',
          items: [
            {
              type: 'doc',
              id: 'chapter-1/index',
              label: 'Chapter Overview',
            },
            {
              type: 'doc',
              id: 'chapter-1/lab',
              label: 'Lab: Hello Physical AI',
            },
          ],
        },
      ],
    },
    {
      type: 'category',
      label: 'Resources',
      items: [
        {
          type: 'link',
          label: 'ROS 2 Documentation',
          href: 'https://docs.ros.org/en/humble/',
        },
        {
          type: 'link',
          label: 'Gazebo Simulator',
          href: 'https://gazebosim.org/',
        },
        {
          type: 'link',
          label: 'NVIDIA Isaac Sim',
          href: 'https://developer.nvidia.com/isaac/',
        },
      ],
    },
  ],
};
