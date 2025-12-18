/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a set of docs in the sidebar
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

// @ts-check

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'docs',
      label: 'Home',
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System',
      items: [
        {
          type: 'doc',
          id: 'module-1/README',
          label: 'Module Overview',
          key: 'module1-overview',
        },
        {
          type: 'doc',
          id: 'module-1/intro',
          label: 'Chapter 1: Introduction to ROS 2',
        },
        {
          type: 'doc',
          id: 'module-1/communication',
          label: 'Chapter 2: ROS 2 Communication',
        },
        {
          type: 'doc',
          id: 'module-1/python-agents',
          label: 'Chapter 3: Python AI Agents & URDF',
        },
      ],
      collapsible: false,
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      items: [
        {
          type: 'doc',
          id: 'module-2/README',
          label: 'Module Overview',
          key: 'module2-overview',
        },
        {
          type: 'doc',
          id: 'module-2/intro',
          label: 'Chapter 0: Philosophy & Goals',
        },
      ],
      collapsible: false,
      collapsed: false,
    },
  ],
};

module.exports = sidebars;
