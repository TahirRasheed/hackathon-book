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
          id: 'module-1/intro',
          label: 'Chapter 1: Introduction to ROS 2',
        },
        {
          type: 'doc',
          id: 'module-1/README',
          label: 'Module Overview',
        },
      ],
      collapsible: false,
      collapsed: false,
    },
  ],
};

module.exports = sidebars;
