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
        {
          type: 'doc',
          id: 'module-2/gazebo',
          label: 'Chapter 1: Physics with Gazebo',
        },
        {
          type: 'doc',
          id: 'module-2/unity',
          label: 'Chapter 2: Human-Robot Interaction in Unity',
        },
        {
          type: 'doc',
          id: 'module-2/sensors',
          label: 'Chapter 3: Sensors & Sim-to-Real Gaps',
        },
      ],
      collapsible: false,
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: Humanoid Robot Architecture',
      items: [
        {
          type: 'doc',
          id: 'module-3/intro',
          label: 'Chapter 1: Introduction & Learning Objectives',
        },
        {
          type: 'doc',
          id: 'module-3/mechanical',
          label: 'Chapter 2: Mechanical Structure & Kinematics',
        },
        {
          type: 'doc',
          id: 'module-3/sensors',
          label: 'Chapter 3: Sensors & Proprioception',
        },
        {
          type: 'doc',
          id: 'module-3/actuators',
          label: 'Chapter 4: Actuators & Power',
        },
        {
          type: 'doc',
          id: 'module-3/compute',
          label: 'Chapter 5: Hardware Compute & Real-Time OS',
        },
        {
          type: 'doc',
          id: 'module-3/ros2',
          label: 'Chapter 6: ROS 2 Software Stack',
        },
        {
          type: 'doc',
          id: 'module-3/safety',
          label: 'Chapter 7: Safety, Redundancy & Integration',
        },
        {
          type: 'doc',
          id: 'module-3/case-study',
          label: 'Chapter 8: Boston Dynamics Atlas Case Study',
        },
        {
          type: 'doc',
          id: 'module-3/summary',
          label: 'Chapter 9: Summary & Review Questions',
        },
      ],
      collapsible: false,
      collapsed: false,
    },
  ],
};

module.exports = sidebars;
